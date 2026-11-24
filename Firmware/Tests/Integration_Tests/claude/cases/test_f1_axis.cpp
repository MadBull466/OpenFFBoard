/*
 * F1 axis layer between the effects and the TMC4671 driver (WO I1-7): F1-X1 torque limit, F1-X2 sign table, F1-X3 position chain and
 * endstop onset, F1-X5 multi-turn and out-of-bounds handling.
 *
 * The position is scripted through 0x6B (PositionScript: no plant, no motor model), the torque target is observed at 0x64 of chip1 once
 * per FFB cycle (TorqueTrace). Settings are made over CDC (the user's path), power and fxratio first (IB 7 / bug 23).
 * Note for I1-8 (S2): after an out-of-bounds shutdown 0x64 keeps its last non-zero target (turn() is not called while the driver is
 * stopped, Axis.cpp:488-495), so a restart drives that stale target until the axis torque next changes.
 *
 * Oracles
 *  - TMC4671-LA datasheet Rev 2.07 (Tests/Code_Coverage/shared/specs/TMC4671-LA_datasheet_Rev2.07.pdf):
 *      0x5E PID_TORQUE_FLUX_LIMITS, p98: u16(15:0) "PID torque limit and PID flux limit, limits the target values coming from the target
 *        registers" (one field for both controllers; unsigned 16 bit; the limit is in the units of the target register 0x64, so an axis
 *        power P that is the largest torque the axis may command at 0x64 gives 0x5E = P: DS + MATH, no firmware constant);
 *      0x64 PID_TORQUE_FLUX_TARGET: flux [15:0], torque [31:16], s16 each;
 *      0x6B PID_POSITION_ACTUAL, p98-99: s32 "Actual multi turn position ... Input position differences are accumulated. Lower 16 bits
 *        display one revolution of input angle. Upper 16 bits display revolutions": 65536 counts are one mechanical revolution, and the
 *        register holds +-32768 revolutions. The datasheet says nothing about what happens at the s32 limit (this reading of the DS is
 *        kept here; PositionScript.h:29 differs on the wrap at 32 bit; reconciled by the lead); the scripted values below stay inside the
 *        s32 range (PositionScript flags a saturated value);
 *      0x63 MODE_MOTION [7:0] 0 = stopped_mode, 0x1A PWM_SV_CHOP [7:0] 0 = PWM off (the "drive off" of the out-of-bounds case).
 *  - USB_PID1_01.pdf 5.3 Condition effects: a spring is restoring (the force opposes the displacement from CP Offset), a damper opposes
 *    the motion. Both are defined relative to the displacement / the motion the test scripts, so no polar zero (5.1 defines none) is needed.
 *  - Unit charter Tests/Code_Coverage/claude/charters/driver/TMC4671_REQUIREMENTS.md R42 (power limit), R62 (inversion in turn()).
 *  - MATH: Axis::scaleEncValue (Axis.cpp:587) maps D degrees of range to 0xffff scaled counts: scaled = 0xffff / D * angle, with
 *    angle = 360 * counts / cpr. A correct chain has cpr = 65536 (datasheet) and gives scaled = counts * 65535 / 65536 * 360 / D. The firmware
 *    reports cpr 0xffff (TMC4671::getCpr, TMC4671.cpp:1932-1941), so scaled = counts * 360 / D (seam S1). The proportionality case
 *    expects counts * 360 / D with a tolerance of 2 counts (float truncation is at most 1): it does not re-report IB 16, whose own cases
 *    (cpr and a 30 revolution position) are the detector, where the difference is large enough to be above the float resolution.
 *
 * Sign convention of the sign table (F1-X2): the chip's torque is positive in the direction in which 0x6B counts up (DS 0x64 / 0x6B
 * share the convention of the closed loop: positive torque -> positive velocity). A restoring spring then has torque opposite to the scripted
 * displacement, an opposing damper has torque opposite to the scripted velocity.
 *
 * Settling rule: the force passes the axis fade-in (Axis.cpp:384, :449, :620, :640: at most 1.0 s) and the filters; every window below is
 * taken after at least 2500 ms (more than the 1.0 s, a bound from the callers, not from observed output) and must have one sign in all
 * of its cycles.
 */
#include "doctest.h"

#include "FfbHost.h"
#include "HidReportDescriptor.h"
#include "IntegrationFixture.h"
#include "PidReportBuilder.h"
#include "PositionScript.h"
#include "TestHelpers.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using hid::ReportType;

namespace {

constexpr uint32_t kDirectionX = hid::usage(0x0A, 1);
constexpr int32_t kRev = PositionScript::kCountsPerRev; // 65536 counts per mechanical revolution, DS 0x6B

struct Window {
	int16_t lo = 0;
	int16_t hi = 0;
	double mean = 0;
	size_t n = 0;
	bool positive() const { return n > 0 && lo > 0; }
	bool negative() const { return n > 0 && hi < 0; }
	bool zero() const { return n > 0 && lo == 0 && hi == 0; }
};

struct AxisFixture : IntegrationFixture {
	std::optional<FfbHost> host;
	PositionScript script;

	void up() {
		bringUp();
		setSpiFrameUs(0);
		host.emplace(*this);
	}
	FfbHost &h() { return *host; }

	/** power, fxratio 255 (IB 7), degrees: every case states the settings its expected values depend on */
	void configure(int64_t power, int64_t degrees) {
		h().command("axis.0.power=" + std::to_string(power));
		h().command("axis.0.fxratio=255");
		h().command("axis.0.degrees=" + std::to_string(degrees));
		// Axis::invertAxis defaults to true (Axis.h:510): the cases that read positions state the direction they use
		h().command("axis.0.invert=0");
		// without a running effect the idle spring acts (HidFFB ffb_active false, Axis.cpp:663: torque proportional to the position), which
		// would put torque at 0x64 where the cases expect none (endstop onset); it is switched off, the effects are started explicitly
		h().command("axis.0.idlespring=0");
	}

	/** Script restarts from `startCounts` (time base = attach); the attach writes value(0) at once */
	void restartRamp(int32_t startCounts, double countsPerMs) {
		detachPositionScript();
		script.setRamp(startCounts, countsPerMs);
		attachPositionScript(script);
	}
	void restartConstant(int32_t counts) {
		detachPositionScript();
		script.setConstant(counts);
		attachPositionScript(script);
	}

	pid::PidReport makeSetEffect(int64_t index, uint32_t typeUsage, int64_t direction) {
		pid::PidReport se = h().builder().make(ReportType::Output, pid::use::SetEffectReport);
		se.set(pid::use::EffectBlockIndex, index).select(typeUsage).set(pid::use::Duration, 32767).set(pid::use::Gain, 255);
		se.flag(pid::use::DirectionEnable, true);
		se.setIn({pid::use::Direction}, kDirectionX, direction);
		return se;
	}
	int64_t newEffect(uint32_t typeUsage) {
		const FfbHost::CreateResult c = h().createEffect(typeUsage);
		REQUIRE(c.load.success());
		return c.load.effectBlockIndex;
	}
	void start(int64_t index) {
		pid::PidReport eo = h().builder().make(ReportType::Output, pid::use::EffectOperationReport);
		eo.set(pid::use::EffectBlockIndex, index).select(pid::use::OpEffectStart).set(pid::use::LoopCount, 1);
		h().send(eo);
	}

	/** Condition effect (spring / damper) with equal coefficients on both sides, no dead band, full saturation, CP Offset 0 */
	void startCondition(uint32_t typeUsage, int64_t coefficient) {
		const int64_t idx = newEffect(typeUsage);
		h().send(makeSetEffect(idx, typeUsage, 0));
		pid::PidReport cond = h().builder().make(ReportType::Output, pid::use::SetConditionReport);
		cond.set(pid::use::EffectBlockIndex, idx).set(pid::use::ParameterBlockOffset, 0).set(pid::use::CpOffset, 0);
		cond.set(pid::use::PositiveCoefficient, coefficient).set(pid::use::NegativeCoefficient, coefficient);
		cond.set(pid::use::PositiveSaturation, 32767).set(pid::use::NegativeSaturation, 32767).set(pid::use::DeadBand, 0);
		h().send(cond);
		start(idx);
	}

	/** Constant force, Direction Enable, polar direction (0.01 degrees), Magnitude; returns the block index */
	int64_t startConstant(int64_t direction, int64_t magnitude) {
		const int64_t idx = newEffect(pid::use::EtConstantForce);
		h().send(makeSetEffect(idx, pid::use::EtConstantForce, direction));
		pid::PidReport sc = h().builder().make(ReportType::Output, pid::use::SetConstantForceReport);
		sc.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, magnitude);
		h().send(sc);
		start(idx);
		return idx;
	}

	/** Runs `settleMs`, then records the torque at 0x64 of the following `windowMs` (one entry per FFB cycle) */
	Window window(uint32_t settleMs, uint32_t windowMs = 100) {
		h().run(settleMs);
		TorqueTrace trace(chip1);
		trace.start(*this);
		h().observe(&trace);
		h().run(windowMs);
		h().observe(nullptr);
		Window w;
		w.n = trace.samples().size();
		if (w.n == 0) {
			return w;
		}
		w.lo = w.hi = trace.samples().front().torque;
		double sum = 0;
		for (const TorqueTrace::Sample &s : trace.samples()) {
			w.lo = std::min(w.lo, s.torque);
			w.hi = std::max(w.hi, s.torque);
			sum += s.torque;
		}
		w.mean = sum / static_cast<double>(w.n);
		return w;
	}

	std::string describe(const Window &w) const {
		std::ostringstream o;
		o << "[" << w.lo << ".." << w.hi << "] mean " << w.mean << " over " << w.n << " cycles";
		return o.str();
	}

	int64_t curpos() { return h().getValue("axis.0.curpos?"); }

	uint32_t torqueLimitField() const { return chip1.reg(0x5E) & 0xffffu; }
};

std::string combo(int invert, int invertForce) {
	std::ostringstream o;
	o << "axis.invert=" << invert << " tmc.invertForce=" << invertForce;
	return o.str();
}

} // namespace

TEST_SUITE("F1Axis") {

// ---------------------------------------------------------------------------------------------------------------------
// F1-X1  torque limit
// ---------------------------------------------------------------------------------------------------------------------

// R42 / Axis::setPower (Axis.cpp:498-505) calls TMC4671::setPowerLimit -> setTorqueLimit (TMC4671.cpp:2143-2152), which writes the value
// into 0x5E. DS 0x5E (p98): the field limits the target values (register 0x64, torque and flux) in the units of those registers, and the
// axis clips its torque at power in the same units (Axis.cpp:892), so the chip limit equals power (DS + MATH). Four values.
TEST_CASE_FIXTURE(AxisFixture, "F1-X1: axis.0.power=P sets the torque limit field of register 0x5E to P" * IT_DELETES_FFBHIDMAIN) {
	up();
	for (int64_t p : {1000, 5000, 20000, 32767}) {
		h().command("axis.0.power=" + std::to_string(p));
		h().run(10);
		INFO("power " << p);
		CHECK(torqueLimitField() == static_cast<uint32_t>(p));
	}
	REQUIRE_CHAIN_MODELLED();
}

// A driver created at runtime never receives the axis power: Axis::setDrvType (Axis.cpp:512-544) does not call setPowerLimit, and
// TMC4671::setupDriver writes the driver's own default limit 30000 (TMC4671.cpp:249, TMC4671.h:238) although its comment at :245 says the
// axis sets it. Seen here as 0x5E = 30000 while axis.0.power? reports 5000 (IB 7 is a different cause: the scalers stay 0 on a fresh
// flash; applying the IB 7 fix does not change this case, calling drv->setPowerLimit(power) after setupDriver in setDrvType does). Impact:
// every runtime drvtype selection; the flux dissipation clips at the 30000 limit (TMC4671.cpp:1736). A configured board is fine at boot
// (restoreFlash calls setDrvType :231 before setPower :262). Intended: 0x5E equals the reported power.
TEST_CASE_FIXTURE(AxisFixture, "F1-X1: on a fresh flash the torque limit field of 0x5E equals the power axis.0.power? reports" * KNOWN_ISSUE("(IB 17) a driver created by Axis::setDrvType never gets the axis power: 0x5E holds the driver default 30000 while axis.0.power? reports 5000") * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t reported = h().getValue("axis.0.power?");
	INFO("axis.0.power? " << reported << ", 0x5E " << torqueLimitField());
	REQUIRE_CHAIN_MODELLED();
	CHECK(torqueLimitField() == static_cast<uint32_t>(reported));
}

// The axis clips before the chip: a demand beyond power arrives at 0x64 as |torque| <= power. Demand: a full-scale constant force plus a
// spring with a large coefficient at a displacement of 6000 counts, in both signs of each. Axis::updateTorque sums the effects torque
// (calculateFFBTorque), the mechanical effects and the endstop; the sum is not clipped before the scaling (EffectsCalculator.cpp:158 does
// not clip it), is multiplied with power / 0x7fff and clipped at power (Axis.cpp:862-892). In the combinations where spring and constant
// force agree the unclipped value exceeds power, so |t| <= P proves the clip; the case also requires that at least one window sits at the
// limit (|t| >= P - 2) per power, so the demand really reached it. MATH: |t| <= P always.
TEST_CASE_FIXTURE(AxisFixture, "F1-X1: a demand beyond power reaches 0x64 clipped to |torque| <= power, both signs, at three power values" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 360);
	script.setConstant(0);
	attachPositionScript(script);
	// full-scale constant force (the other sign follows below through a 180 degree reversal of the direction, PID 5.1)
	const int64_t idx = startConstant(9000, 32767);
	// a spring with a large coefficient adds to it or subtracts from it, depending on the scripted displacement
	startCondition(pid::use::EtSpring, 32767);
	for (int64_t p : {2000, 5000, 12000}) {
		h().command("axis.0.power=" + std::to_string(p));
		int atLimit = 0;
		for (int32_t disp : {-6000, 6000}) {
			restartConstant(disp);
			const Window w = window(2500);
			INFO("power " << p << ", displacement " << disp << ", torque " << describe(w));
			REQUIRE(w.n > 50);
			CHECK(std::abs(static_cast<int>(w.lo)) <= p);
			CHECK(std::abs(static_cast<int>(w.hi)) <= p);
			atLimit += (std::max(std::abs(static_cast<int>(w.lo)), std::abs(static_cast<int>(w.hi))) >= p - 2) ? 1 : 0;
		}
		INFO("power " << p);
		CHECK(atLimit >= 1);
	}
	// the other sign of the constant force (180 degree reversal) with the same spring: still inside the limit
	pid::PidReport se = makeSetEffect(idx, pid::use::EtConstantForce, 27000);
	h().send(se);
	for (int64_t p : {2000, 5000, 12000}) {
		h().command("axis.0.power=" + std::to_string(p));
		for (int32_t disp : {-6000, 6000}) {
			restartConstant(disp);
			const Window w = window(2500);
			INFO("reversed constant force, power " << p << ", displacement " << disp << ", torque " << describe(w));
			CHECK(std::abs(static_cast<int>(w.lo)) <= p);
			CHECK(std::abs(static_cast<int>(w.hi)) <= p);
		}
	}
	REQUIRE_CHAIN_MODELLED();
}

// A full-scale demand uses the whole limit: the torque limit is not lower than power (the clip does not eat range). Both signs of the
// demand are reached by the two directions 180 degrees apart (PID 5.1 reversal), power 3000 / 8000 / 20000.
TEST_CASE_FIXTURE(AxisFixture, "F1-X1: a full-scale constant force reaches the power limit at 0x64 in both directions" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 360);
	const int64_t idx = startConstant(9000, 32767);
	for (int64_t p : {3000, 8000, 20000}) {
		h().command("axis.0.power=" + std::to_string(p));
		const Window a = window(2500);
		h().send(makeSetEffect(idx, pid::use::EtConstantForce, 27000));
		const Window b = window(2500);
		h().send(makeSetEffect(idx, pid::use::EtConstantForce, 9000));
		INFO("power " << p << ": direction 9000 " << describe(a) << ", direction 27000 " << describe(b));
		REQUIRE(a.n > 50);
		// opposite signs (180 degree reversal), each the size of the limit within 1 count of truncation
		CHECK(((a.mean > 0) != (b.mean > 0)));
		CHECK_NEAR(std::abs(a.mean), p, 2);
		CHECK_NEAR(std::abs(b.mean), p, 2);
	}
	REQUIRE_CHAIN_MODELLED();
}

// Bug 32 (coverage register): a power above 0x7fff reverses the motor at full force: the int32 torque is narrowed to the int16 of
// TMC4671::turn() without a clip. Intended: more power never reverses a force, and 0x64 (s16) saturates at +-32767.
TEST_CASE_FIXTURE(AxisFixture, "F1-X1: a power above 0x7fff never reverses the direction of a full-scale force" * KNOWN_ISSUE("(IB 14; same as bug 32) power above 0x7fff wraps the int16 torque of turn() and reverses the force") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(20000, 360);
	startConstant(9000, 32767);
	const Window ref = window(2500);
	REQUIRE(ref.n > 50);
	REQUIRE((ref.positive() || ref.negative()));
	h().command("axis.0.power=40000");
	const Window big = window(2500);
	MESSAGE("power 20000: " << describe(ref) << "; power 40000: " << describe(big));
	REQUIRE_CHAIN_MODELLED();
	CHECK(((big.mean > 0) == (ref.mean > 0)));
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-X2  sign table
// ---------------------------------------------------------------------------------------------------------------------
//
// Reachable settings: axis.0.invert (Axis.cpp:1067-1075) flips both the measured angle (getEncAngle, Axis.cpp:601-606) and the torque
// (updateTorque step 7, :884), so a restoring effect stays restoring. tmc.0.invertForce flips only the torque in TMC4671::turn() (:1753).
// Encoder direction by command: tmc.0.encdir is a CMDFLAG_DEBUG command (TMC4671.cpp:2794), reachable after sys.debug=1
// (SystemCommands.cpp:68, :143-144), but the encoder-direction rows move to F1-T9 (I2): encdir sets abnconf.rdir and re-runs
// setup_ABN_Enc, which copies it to conf.encoderReversed (TMC4671.cpp:3028-3035, 1309), the flag turn() uses (:1753) only with an
// external encoder; the encoder type of the fresh-flash bring-up is NONE (INR 1); and FakeTmc4671 does not tie the ABN direction to the
// scripted 0x6B.
//
// Spring check: torque at the chip opposes the scripted displacement; damper check: opposes the scripted velocity.

namespace {

struct SignTable : AxisFixture {
	/** Spring: displacement +-4096 counts (22.5 degrees of a D = 900 range, no endstop); true if restoring for both */
	bool springRestoring(std::string &why) {
		bool ok = true;
		for (int32_t d : {4096, -4096}) {
			restartConstant(d);
			const Window w = window(2500);
			const bool restoring = (d > 0) ? w.negative() : w.positive();
			ok = ok && restoring;
			std::ostringstream o;
			o << " displacement " << d << " -> torque " << describe(w) << (restoring ? "" : " NOT RESTORING;");
			why += o.str();
		}
		return ok;
	}
	/** Damper: constant speed +-10 counts / ms (about 55 degrees / s) from 0; true if opposing for both */
	bool damperOpposing(std::string &why) {
		bool ok = true;
		for (double v : {10.0, -10.0}) {
			restartRamp(0, v);
			const Window w = window(2500);
			const bool opposing = (v > 0) ? w.negative() : w.positive();
			ok = ok && opposing;
			std::ostringstream o;
			o << " speed " << v << " counts/ms -> torque " << describe(w) << (opposing ? "" : " NOT OPPOSING;");
			why += o.str();
		}
		return ok;
	}
	void setCombo(int invert, int invertForce) {
		h().command("axis.0.invert=" + std::to_string(invert));
		h().command("tmc.0.invertForce=" + std::to_string(invertForce));
		h().run(10);
	}
};

} // namespace

// The settings the sign table varies are read back (the commands exist and take effect), then the spring is restoring for the two
// values of axis.0.invert with tmc.0.invertForce = 0. PID 5.3.
TEST_CASE_FIXTURE(SignTable, "F1-X2: a spring is restoring at the chip for axis.0.invert 0 / 1 with tmc.0.invertForce 0" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 900);
	script.setConstant(0);
	attachPositionScript(script);
	startCondition(pid::use::EtSpring, 10000);
	for (int inv : {0, 1}) {
		setCombo(inv, 0);
		REQUIRE(h().getValue("axis.0.invert?") == inv);
		REQUIRE(h().getValue("tmc.0.invertForce?") == 0);
		std::string why;
		const bool ok = springRestoring(why);
		INFO(combo(inv, 0) << ":" << why);
		CHECK(ok);
	}
	REQUIRE_CHAIN_MODELLED();
}

TEST_CASE_FIXTURE(SignTable, "F1-X2: a damper is opposing at the chip for axis.0.invert 0 / 1 with tmc.0.invertForce 0" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 900);
	script.setConstant(0);
	attachPositionScript(script);
	startCondition(pid::use::EtDamper, 10000);
	for (int inv : {0, 1}) {
		setCombo(inv, 0);
		std::string why;
		const bool ok = damperOpposing(why);
		INFO(combo(inv, 0) << ":" << why);
		CHECK(ok);
	}
	REQUIRE_CHAIN_MODELLED();
}

// Seam S3. tmc.0.invertForce is documented "Invert incoming forces" (TMC4671.cpp:2805) and exists to compensate a motor that is wired
// the other way round than the encoder it is combined with: the flash restore keeps it only with combineEncoder (TMC4671.cpp:2720:
// invertForce = bit7 && combineEncoder) and clearing combineEncoder clears it (:2822-2824). It flips the torque in turn() but NOT the
// measured position (axis.0.invert flips both: getEncAngle and the final torque). With the integrated encoder (combineEncoder 0, the
// bring-up of this executable, whose position is the chip's own 0x6B) the command is nevertheless accepted, and a torque flipped
// against an unflipped position is positive feedback: a spring pushes away from the centre, a damper assists the motion (PID 5.3 never
// intends that). The oracle rests on the sign convention of the header (a positive torque moves 0x6B up): an assumption about a
// commissioned loop, not a datasheet sentence; PID 5.3 (no positive feedback) is the property asserted. The firmware's own invariant is
// that invertForce needs combineEncoder (TMC4671.cpp:2720 on restore, :2823 when combineEncoder is cleared), but the setter (:2832) bypasses
// it, so the setting silently disappears at the next boot. Intended: the command is rejected or ignored without combineEncoder, or both
// the torque and the measured position flip.
// Whether a motor wired against a combined external encoder is compensated correctly is a closed-loop question (F1-T9, I2 / I4).
TEST_CASE_FIXTURE(SignTable, "F1-X2: a spring stays restoring after tmc.0.invertForce=1 with the integrated encoder (axis.0.invert 0 and 1)" * KNOWN_ISSUE("(IB 15) tmc.0.invertForce=1 is accepted without combineEncoder and flips only the torque: the spring pushes away from the centre") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 900);
	script.setConstant(0);
	attachPositionScript(script);
	startCondition(pid::use::EtSpring, 10000);
	for (int inv : {0, 1}) {
		setCombo(inv, 1);
		std::string why;
		const bool ok = springRestoring(why);
		const int64_t combineEncoder = h().getValue("tmc.0.combineEncoder?"); // never a REQUIRE inside an INFO expression (doctest deadlock under -s)
		INFO(combo(inv, 1) << ":" << why << " combineEncoder " << combineEncoder);
		REQUIRE(combineEncoder == 0);
		REQUIRE_CHAIN_MODELLED();
		CHECK(ok);
	}
}

TEST_CASE_FIXTURE(SignTable, "F1-X2: a damper stays opposing after tmc.0.invertForce=1 with the integrated encoder (axis.0.invert 0 and 1)" * KNOWN_ISSUE("(IB 15) tmc.0.invertForce=1 is accepted without combineEncoder and flips only the torque: the damper assists the motion") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 900);
	script.setConstant(0);
	attachPositionScript(script);
	startCondition(pid::use::EtDamper, 10000);
	for (int inv : {0, 1}) {
		setCombo(inv, 1);
		std::string why;
		const bool ok = damperOpposing(why);
		INFO(combo(inv, 1) << ":" << why);
		REQUIRE(h().getValue("tmc.0.combineEncoder?") == 0);
		REQUIRE_CHAIN_MODELLED();
		CHECK(ok);
	}
}

// Axis invert and invertForce are independent torque inversions: each one alone reverses the sign of a constant force at the chip
// (PID 5.1: a constant force is a force in one direction; the chip's torque sign is the direction). Both together give the original.
TEST_CASE_FIXTURE(SignTable, "F1-X2: axis.0.invert and tmc.0.invertForce each reverse the sign of a constant force at the chip and both together restore it" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 900);
	const int64_t idx = startConstant(9000, 10000);
	// The driver writes 0x64 only when the axis torque changes (Axis.cpp:488-495): changing invertForce alone does not reach the chip until
	// the torque changes (observed: the sign stayed after tmc.0.invertForce=1 with a constant force; recorded in the report as a note). The
	// force is therefore nudged (magnitude 5000, then 10000 again) after every setting change.
	auto settle = [&]() {
		pid::PidReport sc = h().builder().make(ReportType::Output, pid::use::SetConstantForceReport);
		sc.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, 5000);
		h().send(sc);
		h().run(300);
		pid::PidReport sc2 = h().builder().make(ReportType::Output, pid::use::SetConstantForceReport);
		sc2.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, 10000);
		h().send(sc2);
		return window(2500);
	};
	setCombo(0, 0);
	const Window base = settle();
	REQUIRE((base.positive() || base.negative()));
	setCombo(1, 0);
	const Window a = settle();
	setCombo(0, 1);
	const Window f = settle();
	setCombo(1, 1);
	const Window both = settle();
	INFO("base " << describe(base) << ", invert " << describe(a) << ", invertForce " << describe(f) << ", both " << describe(both));
	CHECK(((a.mean > 0) != (base.mean > 0)));
	CHECK(((f.mean > 0) != (base.mean > 0)));
	CHECK(((both.mean > 0) == (base.mean > 0)));
	CHECK_NEAR(a.mean, -base.mean, 3);
	CHECK_NEAR(f.mean, -base.mean, 3);
	CHECK_NEAR(both.mean, base.mean, 3);
	REQUIRE_CHAIN_MODELLED();
}

// INR 9: the driver is called only when the axis torque changes (Axis::updateDriveTorque, Axis.cpp:488-495), so a change of
// tmc.0.invertForce under a steady force does not reach 0x64 until the torque changes. Observed: the window after the change is
// identical to the one before; after a nudge of the force the sign is reversed.
TEST_CASE_FIXTURE(SignTable, "characterization: [F1-X2] a tmc.0.invertForce change under a steady force reaches 0x64 only when the axis torque changes" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 900);
	setCombo(0, 0);
	const int64_t idx = startConstant(9000, 10000);
	const Window before = window(2500);
	REQUIRE((before.positive() || before.negative()));
	h().command("tmc.0.invertForce=1");
	const Window unchanged = window(500);
	// INR 9: turn() is called only on a torque change (Axis.cpp:488-495): the stale sign stays
	pid::PidReport sc = h().builder().make(ReportType::Output, pid::use::SetConstantForceReport);
	sc.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, 5000);
	h().send(sc);
	h().run(300);
	pid::PidReport sc2 = h().builder().make(ReportType::Output, pid::use::SetConstantForceReport);
	sc2.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, 10000);
	h().send(sc2);
	const Window after = window(2500);
	INFO("before " << describe(before) << ", after the setting " << describe(unchanged) << ", after a nudge " << describe(after));
	CHECK_NEAR(unchanged.mean, before.mean, 0);
	CHECK_NEAR(after.mean, -before.mean, 3);
	REQUIRE_CHAIN_MODELLED();
}

// Absolute direction -> sign (polar zero undefined, PID 1.01 5.1; reconciled by the lead): the sign at the chip of a constant force
// of polar direction 9000 (90 degrees) with axis.0.invert 0 and tmc.0.invertForce 0, and of 27000.
TEST_CASE_FIXTURE(SignTable, "characterization: [F1-X2] absolute sign of a constant force of polar direction 9000 and 27000 with both inversions off" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 900);
	setCombo(0, 0);
	const int64_t idx = startConstant(9000, 10000);
	const Window d9 = window(2500);
	h().send(makeSetEffect(idx, pid::use::EtConstantForce, 27000));
	const Window d27 = window(2500);
	// INR 7: polar zero undefined, PID 1.01 5.1; reconciled by the lead. Observed: 9000 -> negative, 27000 -> positive.
	MESSAGE("direction 9000: " << describe(d9) << "; direction 27000: " << describe(d27));
	REQUIRE(d9.n > 50);
	CHECK(d9.mean < 0);
	CHECK(d27.mean > 0);
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-X3  position chain, endstop onset
// ---------------------------------------------------------------------------------------------------------------------

// DS 0x6B: 65536 counts = one revolution = 360 degrees. Axis::scaleEncValue maps the range D to 0xffff scaled counts (curpos). The case
// expects c * 360 / D (0 -> 0, +-16384 -> +-16384 at D = 360, +-65536 -> +-65536, 12345 -> 12345; half of it at D = 720). A fully correct
// chain would give c * 65535 / 65536 * 360 / D, one count in 65535 less (the firmware's cpr 0xffff, IB 16): the tolerance of 2 counts
// absorbs the float truncation (at most 1 here) and so does NOT separate the two, on purpose: that difference is IB 16's subject, whose
// own cases (cpr and the 30 revolution position, where it is 30 counts) are the detector. Odd symmetry is exact.
TEST_CASE_FIXTURE(AxisFixture, "F1-X3: scripted 0x6B counts give the proportional axis.0.curpos for D = 360 and D = 720, with odd symmetry" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 360);
	script.setConstant(0);
	attachPositionScript(script);
	for (int64_t deg : {360, 720}) {
		h().command("axis.0.degrees=" + std::to_string(deg));
		for (int32_t c : {0, kRev / 4, -kRev / 4, kRev, -kRev, 12345, -12345}) {
			restartConstant(c);
			h().run(50);
			const int64_t pos = curpos();
			const double expected = static_cast<double>(c) * 360.0 / static_cast<double>(deg);
			INFO("degrees " << deg << ", counts " << c << " -> curpos " << pos << ", proportional value " << expected);
			CHECK_NEAR(pos, expected, 2);
		}
		restartConstant(12345);
		h().run(50);
		const int64_t plus = curpos();
		restartConstant(-12345);
		h().run(50);
		const int64_t minus = curpos();
		INFO("degrees " << deg << ", +12345 -> " << plus << ", -12345 -> " << minus);
		CHECK(plus == -minus);
	}
	REQUIRE_CHAIN_MODELLED();
}

TEST_CASE_FIXTURE(AxisFixture, "F1-X3: axis.0.invert reverses the sign of axis.0.curpos for a scripted position" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 360);
	script.setConstant(20000);
	attachPositionScript(script);
	h().run(50);
	REQUIRE(h().getValue("axis.0.invert?") == 0);
	const int64_t normal = curpos();
	h().command("axis.0.invert=1");
	h().run(50);
	const int64_t inverted = curpos();
	INFO("normal " << normal << ", inverted " << inverted);
	REQUIRE(normal != 0);
	CHECK(inverted == -normal);
	REQUIRE_CHAIN_MODELLED();
}

// Seam S1: the integrated encoder's cpr is returned as 0xffff (TMC4671.cpp:1932-1941, Encoder.cpp:67-72 divides the count by it,
// Axis.cpp:1252-1268 reports it). DS 0x6B: one revolution is 65536 counts. Intended: axis.0.cpr reports 65536.
TEST_CASE_FIXTURE(AxisFixture, "F1-X3: the cpr reported for the integrated encoder is the 65536 counts of one revolution" * KNOWN_ISSUE("(IB 16) the integrated encoder reports cpr 0xffff, DS 0x6B has 65536 counts per revolution (TMC4671.cpp:1932-1941)") * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t cpr = h().getValue("axis.0.cpr?");
	INFO("axis.0.cpr? " << cpr);
	REQUIRE_CHAIN_MODELLED();
	CHECK(cpr == kRev);
}

// The same defect seen in the position: 30 revolutions of the scripted counter with D = 360 (D degrees <-> 0xffff scaled counts). The
// intended chain gives 30 * 65535 = 1966050 scaled counts, the firmware reports one count per revolution more (cpr 0xffff: 360.0055
// degrees per revolution). At this size the difference of 30 counts is far above the float resolution (0.125).
TEST_CASE_FIXTURE(AxisFixture, "F1-X3: thirty revolutions of 0x6B read as 30 times 360 degrees in axis.0.curpos" * KNOWN_ISSUE("(IB 16) one revolution reads 360.0055 degrees (cpr 0xffff), 30 revolutions are 30 scaled counts too many") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 360);
	script.setConstant(30 * kRev);
	attachPositionScript(script);
	h().run(50);
	const int64_t pos = curpos();
	INFO("curpos " << pos << ", intended 30 * 0xffff = " << 30 * 0xffff);
	REQUIRE_CHAIN_MODELLED();
	CHECK_NEAR(pos, 30 * 0xffff, 2);
}

// Endstop onset (Axis::calculateEndstopTorque, Axis.cpp:826-836): with D = 360 the soft endstop acts beyond +-D/2 = 180 degrees =
// +-32768 counts and pushes back. Plain: no torque at 32700 counts (0.2 percent inside the edge), torque of the opposite sign beyond
// the edge (33000 counts, 0.7 percent outside), for both ends. The gain is I1-8.
TEST_CASE_FIXTURE(AxisFixture, "F1-X3: the endstop torque starts at the range edge of axis.0.degrees and pushes back toward the centre" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 360);
	script.setConstant(0);
	attachPositionScript(script);
	for (int32_t sign : {1, -1}) {
		restartConstant(sign * 32700);
		const Window inside = window(2500);
		restartConstant(sign * 33000);
		const Window outside = window(2500);
		INFO("end " << sign << ": inside (32700) " << describe(inside) << ", outside (33000) " << describe(outside));
		REQUIRE(inside.n > 50);
		CHECK(inside.zero());
		CHECK((sign > 0 ? outside.negative() : outside.positive()));
	}
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-X5  multi-turn and out of bounds
// ---------------------------------------------------------------------------------------------------------------------

namespace {

struct MultiTurn : AxisFixture {
	/**
	 * Ramps 0x6B at `countsPerMs` for `ms`, reads axis.0.curpos every 20 ms (the command itself takes simulated time, so the nominal value is
	 * taken from what the script wrote last): returns the largest deviation of curpos from lastValue * 360 / D and the final curpos. The axis
	 * reads the value one cycle later (curpos is metric.previous) and the curpos command itself runs for some simulated ms after the script
	 * value is read: 3.6 scaled counts per ms of ramp; the measured worst deviation is 15.0 (about 4 ms), so the bound 20 (5.5 ms) is a
	 * bound on the skew of sampling, not on the chain.
	 */
	void ramp(double countsPerMs, uint32_t ms, int64_t degrees, double &worstDeviation, double &worstStep, int64_t &finalPos, int64_t &finalScript) {
		restartRamp(0, countsPerMs);
		worstDeviation = 0;
		worstStep = 0;
		bool have = false;
		int64_t prevPos = 0, prevScript = 0;
		for (uint32_t t = 0; t < ms; t += 20) {
			h().run(20);
			const int64_t pos = curpos();
			const int64_t sc = script.lastValue();
			const double nominal = static_cast<double>(sc) * 360.0 / static_cast<double>(degrees);
			worstDeviation = std::max(worstDeviation, std::abs(static_cast<double>(pos) - nominal));
			if (have) {
				// successive difference against the script's own difference: a step per revolution (720 scaled counts) shows here
				const double nominalStep = static_cast<double>(sc - prevScript) * 360.0 / static_cast<double>(degrees);
				worstStep = std::max(worstStep, std::abs(static_cast<double>(pos - prevPos) - nominalStep));
			}
			have = true;
			prevPos = pos;
			prevScript = sc;
			finalPos = pos;
			finalScript = sc;
		}
	}
};

} // namespace

// 20 revolutions each way in 4000 ms (327.68 counts / ms = 5 revolutions per second); D = 32767 (the largest range, 0x7fff) keeps the
// scaled value inside +-0xffff for 20 turns so the axis never leaves its range. A step of 20 ms is 6553.6 counts = 72.0 scaled counts
// (counts * 360 / D). Two checks: every sample is within 20 scaled counts of the script's value (skew of sampling, see ramp()), and every
// successive difference is within 20 of the script's own difference; a per-revolution step would be 720 scaled counts (65536 * 360 / D).
TEST_CASE_FIXTURE(MultiTurn, "F1-X5: a ramp of 20 revolutions in each direction keeps axis.0.curpos continuous with no step per revolution" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 32767);
	script.setConstant(0);
	attachPositionScript(script);
	for (double rate : {327.68, -327.68}) {
		double worst = 0, worstStep = 0;
		int64_t finalPos = 0, finalScript = 0;
		ramp(rate, 4000, 32767, worst, worstStep, finalPos, finalScript);
		INFO("rate " << rate << ": worst deviation from the script " << worst << ", worst step deviation " << worstStep << ", final curpos " << finalPos << ", script " << finalScript);
		CHECK(worst <= 20.0);
		CHECK(worstStep <= 20.0);
		CHECK(std::abs(finalScript) >= 19 * kRev); // the ramp really covered about 20 revolutions
	}
	REQUIRE_CHAIN_MODELLED();
}

// Out of bounds. Axis::updateParams (Axis.cpp:431-444): |scaled| > 0xffff with a ready driver -> drv->stopMotor(), outOfBounds = true;
// the torque is forced to 0 in updateTorque (:867), and TMC4671::stopMotor sets MODE_MOTION to stopped and PWM off (TMC4671.cpp:1671-1680).
// "Torque 0 at the chip" is therefore the drive being off, not a 0 written into 0x64: turn() is not called with a stopped driver, so 0x64
// keeps its last non-zero target (Axis.cpp:488-495; a note for I1-8 / seam S2, not asserted: a restart drives the stale target until the
// torque next changes). The motor restarts only after the error is cleared (Axis::errorCallback), so one case per sign, each with a fresh
// bring-up: a second jump in the same case would find the drive already off. Safety property (plain): after the scripted position jumps to
// +-2e9 counts (about 30500 revolutions: inside the s32 range, which holds +-32768 revolutions per DS 0x6B; a real session needs about
// an hour of continuous spinning at 10 rev/s to get there, so only a script reaches it) a running constant force does not drive the
// motor: MODE_MOTION = 0 and PWM_SV_CHOP = 0 (precondition: 1 and 7 while in range).
namespace {

struct OutOfBounds : AxisFixture {
	void jumpAndCheck(int32_t target) {
		up();
		configure(5000, 900);
		script.setConstant(0);
		attachPositionScript(script);
		startConstant(9000, 20000);
		const Window before = window(2500);
		REQUIRE((before.positive() || before.negative()));
		REQUIRE((chip1.reg(0x63) & 0xff) == 1);
		REQUIRE((chip1.reg(0x1A) & 0xff) == 7);
		restartConstant(target);
		h().run(100);
		const uint32_t mode = chip1.reg(0x63) & 0xff;
		const uint32_t pwm = chip1.reg(0x1A) & 0xff;
		MESSAGE("position " << target << ": MODE_MOTION " << mode << ", PWM_SV_CHOP " << pwm << ", 0x64 " << chip1.reg(0x64));
		CHECK(mode == 0);
		CHECK(pwm == 0);
		REQUIRE_CHAIN_MODELLED();
	}
	/** HD threshold (INR 8): D = 900 -> counts 163840 = D; the drive is on at 0.9 D and off at 1.1 D, towards `sign` */
	void thresholdCheck(int32_t sign) {
		up();
		configure(5000, 900);
		script.setConstant(0);
		attachPositionScript(script);
		startConstant(9000, 20000);
		h().run(2500);
		restartConstant(sign * 147456); // 0.9 D
		h().run(200);
		const uint32_t modeIn = chip1.reg(0x63) & 0xff;
		restartConstant(sign * 180224); // 1.1 D
		h().run(200);
		const uint32_t modeOut = chip1.reg(0x63) & 0xff;
		// INR 8: threshold from Axis.cpp:431 (|scaled| > 0xffff), scale from the firmware
		MESSAGE("sign " << sign << ": MODE_MOTION at 0.9 D " << modeIn << ", at 1.1 D " << modeOut);
		CHECK(modeIn == 1);
		CHECK(modeOut == 0);
		REQUIRE_CHAIN_MODELLED();
	}
};

} // namespace

TEST_CASE_FIXTURE(OutOfBounds, "F1-X5: a scripted position far beyond the positive end switches the drive off at the chip while a constant force is running" * IT_DELETES_FFBHIDMAIN) {
	jumpAndCheck(2000000000);
}

TEST_CASE_FIXTURE(OutOfBounds, "F1-X5: a scripted position far beyond the negative end switches the drive off at the chip while a constant force is running" * IT_DELETES_FFBHIDMAIN) {
	jumpAndCheck(-2000000000);
}

// HD: the out-of-bounds threshold is |scaled| > 0xffff = the whole range D beyond the centre, i.e. one half range beyond the endstop edge
// (Axis.cpp:431). INR 8: with D = 900 the drive is on at 0.9 D (810 degrees = 147456 counts) and off at 1.1 D (990 degrees = 180224).
TEST_CASE_FIXTURE(OutOfBounds, "characterization: [F1-X5] the out-of-bounds shutdown starts when the scaled position passes 0xffff, one range D beyond the centre, positive end" * IT_DELETES_FFBHIDMAIN) {
	thresholdCheck(1);
}

TEST_CASE_FIXTURE(OutOfBounds, "characterization: [F1-X5] the out-of-bounds shutdown starts when the scaled position passes 0xffff, one range D beyond the centre, negative end" * IT_DELETES_FFBHIDMAIN) {
	thresholdCheck(-1);
}

// Seam F1-B2 input. Axis::scaleEncValue (Axis.cpp:592) converts a float to int32: with a small range D and a large 0x6B the float is out
// of the int32 range (D = 1: scaled = counts * 360 / 1 beyond 2^31 from about 6e6 counts = 91 turns, reachable by a real session). That
// conversion is undefined behaviour. On x86 (cvttss2si, the host and the i386 build) the result is INT_MIN, abs() of it is negative, so
// neither the |scaled| > 0xffff shutdown nor anything else fires and the drive stays on; on ARM (VCVT) the result saturates at INT_MAX
// and the shutdown fires. Intended: the drive is off beyond the range. Skipped in the sanitizer builds (UBSan reports the conversion).
TEST_CASE_FIXTURE(AxisFixture, "F1-X5: a scripted position beyond the s32 scaling of a one degree range switches the drive off at the chip" * KNOWN_ISSUE_UB("(IB 18) scaleEncValue converts an out-of-range float to int32 (Axis.cpp:592): x86 gives INT_MIN, abs() stays negative and the out-of-bounds shutdown is skipped; ARM saturates and shuts down") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 1);
	script.setConstant(0);
	attachPositionScript(script);
	startConstant(9000, 20000);
	const Window before = window(2500);
	REQUIRE((before.positive() || before.negative()));
	REQUIRE((chip1.reg(0x63) & 0xff) == 1);
	restartConstant(6000000);
	h().run(100);
	const uint32_t mode = chip1.reg(0x63) & 0xff;
	MESSAGE("D = 1, 6000000 counts: curpos " << curpos() << ", MODE_MOTION " << mode);
	REQUIRE_CHAIN_MODELLED();
	CHECK(mode == 0);
}


// =====================================================================================================================
// WO I1-8: F1-X4 endstop gain, F1-X6 axis-internal effects, F1-X7 torque shaping, seam S2 (late driver readiness)
// =====================================================================================================================
//
// Oracles of this part
//  - Endstop: PID 1.01 has no endstop; the plain parts are the physical ones (a soft endstop pushes back toward the centre, harder the
//    further the axis is pushed, harder with a larger stiffness setting; torque of independent sources adds and the chip never gets more
//    than the power limit: the same DS 0x5E / 0x64 sentences as F1-X1). Absolute gains are HD from Axis.cpp:826-836 (characterization,
//    INR 17).
//  - Internal effects: PID 5.3 sentences quoted in the header (a damper opposes the motion; inertia opposes the acceleration; a spring
//    restores), friction is a force opposing the motion that does not grow with the speed (Coulomb friction, the physical definition and
//    the PID 5.3 friction effect). Scales are HD (INR 18 idle spring, INR 19 damper / friction / inertia).
//  - Slew limiter: the command help of Axis.cpp:186 "Torque rate limit in counts/ms" and Axis.h:484 ("per millisecond"); the chip's
//    0x64 torque is in the same units as axis.0.power (counts). The intended bound is R counts per millisecond of time, i.e. R per cycle
//    at the default 1 kHz FFB rate and 2 R per cycle at 500 Hz. The firmware limits per FFB cycle without filter_f (Axis.cpp:908-921):
//    the 500 Hz case is IB 26.
//  - Fade-in, speed limiter, expo, equalizer: the firmware's own help text and header comments; plain properties are the structural
//    ones (end value, monotony, sign, zero at zero, transparency of a disabled stage); the curves are HD (INR 20-22 speed limiter, INR 23 fade-in, INR 24 expo, INR 32 equalizer).
//  - Late driver readiness: the axis slew limiter bounds the change at the chip (help text above), the safety property of the
//    out-of-bounds stop is that a stopped motor restarts without a torque the host no longer asks for.
//
// Defaults relied on (fresh flash): maxspeed 0 = speed limiter off (Axis.h:483), slewrate 0 = off (Axis.h:484), axisdamper 30
// (Axis.h:536; internalOff() sets it to 0 where a case needs the plain effect torque), axisinertia / axisfriction 0, expo 0, equalizer off.
//
// F1-B2 note (not tested here): Axis::calculateEndstopTorque converts the float endstop torque to int32 before its clip
// (clip<int32_t,int32_t>(endstopTorque, ...), Axis.cpp:835): out of the int32 range beyond about 676000 degrees of penetration at esgain
// 127 (x86 gives INT_MIN, ARM saturates). The out-of-bounds shutdown (|scaled| > 0xffff) comes long before that for every D.
//
// Sign convention as in the header: a positive torque at the chip counts 0x6B up. Absolute direction -> sign is not used here: every
// case gets the sign of its own torque from a measurement first (the absolute mapping is the one characterization case of F1-X2).

namespace {

struct ShapeFixture : AxisFixture {
	/** bring-up, settings, a constant scripted position 0 and the end of the start fade (the first ready cycle starts a fade of 1 s) */
	void prepare(int64_t power, int64_t degrees) {
		up();
		configure(power, degrees);
		script.setConstant(0);
		attachPositionScript(script);
		h().run(1500);
	}
	void internalOff() {
		h().command("axis.0.axisdamper=0");
		h().command("axis.0.axisinertia=0");
		h().command("axis.0.axisfriction=0");
	}
	/** Position moves with `rate` counts per FFB cycle from the current value (keyed to the FFB cycle count, IB 1) */
	void rampFrom(int32_t rate) {
		const int32_t base = script.lastValue();
		const uint32_t c0 = ffbCycles();
		script.setFunction([=, this](uint64_t) { return static_cast<int32_t>(base + static_cast<int64_t>(rate) * static_cast<int64_t>(ffbCycles() - c0)); });
	}
	/** Constant acceleration `a` counts per cycle^2 from the current value at rest: p(n) = base + a n (n + 1) / 2 */
	void parabolaFrom(int32_t a) {
		const int32_t base = script.lastValue();
		const uint32_t c0 = ffbCycles();
		script.setFunction([=, this](uint64_t) {
			const int64_t n = static_cast<int64_t>(ffbCycles() - c0);
			return static_cast<int32_t>(base + a * (n * (n + 1) / 2));
		});
	}
	void setMagnitude(int64_t idx, int64_t magnitude) {
		pid::PidReport sc = h().builder().make(ReportType::Output, pid::use::SetConstantForceReport);
		sc.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, magnitude);
		h().send(sc);
	}
	/** Effect created and parameterised but not started; returns the block index */
	int64_t defineConstant(int64_t direction, int64_t magnitude) {
		const int64_t idx = newEffect(pid::use::EtConstantForce);
		h().send(makeSetEffect(idx, pid::use::EtConstantForce, direction));
		setMagnitude(idx, magnitude);
		return idx;
	}
	void control(uint32_t dcUsage) { h().send(h().builder().deviceControl(dcUsage)); }
	double meanOf(uint32_t settleMs, uint32_t windowMs = 100) {
		const Window w = window(settleMs, windowMs);
		REQUIRE(w.n > 50);
		return w.mean;
	}
	/** Records the torque per cycle for the next `ms` ms: (cycle index, torque) */
	std::vector<TorqueTrace::Sample> record(uint32_t ms) {
		TorqueTrace trace(chip1);
		trace.start(*this);
		h().observe(&trace);
		h().run(ms);
		h().observe(nullptr);
		return trace.samples();
	}
	int16_t chipTorque() const { return static_cast<int16_t>((chip1.reg(0x64) >> 16) & 0xffff); }
	bool driverReady() { return tmc()->getState() == TMC_ControlState::Running && tmc()->motorReady(); }
	/** Runs 1 ms steps until the driver is Running and motorReady (or maxMs passed). @return true if reached */
	bool runUntilReady(uint32_t maxMs) {
		for (uint32_t ms = 0; ms < maxMs; ms++) {
			if (driverReady()) {
				return true;
			}
			h().run(1);
		}
		return driverReady();
	}
};

// the endstop at the positive / negative end of D = 360 is at +-32768 counts; position `edge + penetration` counts
constexpr int32_t kEdge360 = 32768;

} // namespace

// ---------------------------------------------------------------------------------------------------------------------
// F1-X4  endstop gain
// ---------------------------------------------------------------------------------------------------------------------

// Plain: the further the axis is pushed beyond the range edge, the harder the torque pushes back (strictly monotone over four
// penetrations), at both ends, always toward the centre (opposite to the side the scripted position is on). Power 20000, D = 360.
TEST_CASE_FIXTURE(ShapeFixture, "F1-X4: the endstop torque pushes back harder with every step of penetration, at both ends" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 360);
	for (int32_t side : {1, -1}) {
		double last = 0;
		for (int32_t pen : {300, 600, 1000, 1500}) {
			restartConstant(side * (kEdge360 + pen));
			const Window w = window(600);
			REQUIRE(w.n > 50);
			INFO("side " << side << ", penetration " << pen << " counts: " << describe(w));
			CHECK((side > 0 ? w.negative() : w.positive()));
			CHECK(std::abs(w.mean) > last);
			last = std::abs(w.mean);
		}
	}
	REQUIRE_CHAIN_MODELLED();
}

// Plain: a larger esgain (stiffness) gives a larger endstop torque at the same penetration; esgain 0 gives none.
TEST_CASE_FIXTURE(ShapeFixture, "F1-X4: a larger axis.0.esgain gives a larger endstop torque at the same penetration, esgain 0 none" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 360);
	restartConstant(kEdge360 + 400);
	double last = -1;
	for (int gain : {0, 32, 64, 127, 255}) {
		h().command("axis.0.esgain=" + std::to_string(gain));
		REQUIRE(h().getValue("axis.0.esgain?") == gain);
		const Window w = window(600);
		REQUIRE(w.n > 50);
		INFO("esgain " << gain << ": " << describe(w));
		if (gain == 0) {
			CHECK(w.zero());
		}
		CHECK(std::abs(w.mean) > last);
		last = std::abs(w.mean);
	}
	REQUIRE_CHAIN_MODELLED();
}

// HD (INR 17): Axis::calculateEndstopTorque (Axis.cpp:826-836): raw = (|posDegrees| - D / 2) * esgain * 25 (endstopGain, Axis.h:512),
// clipped at +-0x7fff, then updateTorque scales by power / 0x7fff (Axis.cpp:863). posDegrees = counts * 360 / 0xffff (the cpr of the
// integrated encoder, IB 16, so 0x6B counts are one count in 65535 more degrees than a 65536 revolution). Checked: the value at two
// penetrations (linear law), the ratio between them, and esgain 64. Tolerance 2 counts (two float -> int truncations): at 1200 counts
// the cpr 65536 reading would give 5.5 counts less, so the case also pins the IB 16 scale.
TEST_CASE_FIXTURE(ShapeFixture, "characterization: [F1-X4] the endstop torque is linear in the penetration with 25 raw counts per degree and esgain, scaled by power" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 360);
	const double scaler = 20000.0 / 32767.0;
	auto expected = [&](int32_t counts, int gain) {
		const double deg = counts * 360.0 / 65535.0 - 180.0;
		return std::min(deg * gain * 25.0, 32767.0) * scaler;
	};
	double measured[2] = {0, 0};
	int i = 0;
	for (int32_t pen : {400, 1200}) {
		restartConstant(kEdge360 + pen);
		const Window w = window(600);
		measured[i++] = std::abs(w.mean);
		const double e = expected(kEdge360 + pen, 127);
		INFO("penetration " << pen << ": " << describe(w) << ", expected magnitude " << e);
		REQUIRE(w.n > 50);
		CHECK_NEAR(std::abs(w.mean), e, 2);
	}
	CHECK_NEAR(measured[1] / measured[0], expected(kEdge360 + 1200, 127) / expected(kEdge360 + 400, 127), 0.05);
	h().command("axis.0.esgain=64");
	const double half = std::abs(window(600).mean);
	INFO("esgain 64: " << half << " against esgain 127: " << measured[1] << " at penetration 1200 (the mean above is of the last window)");
	CHECK_NEAR(half, expected(kEdge360 + 1200, 64), 2);
	REQUIRE_CHAIN_MODELLED();
}

// Plain: independent torques add. The endstop torque e (measured alone, no effect running: FFB inactive, idle spring 0) plus the torque
// c of a running constant force (measured in range) is what the chip gets at the penetration, in both relative directions of the
// constant force (reversal over 180 degrees, PID 5.1). Tolerance 3 counts: three float -> int truncations.
TEST_CASE_FIXTURE(ShapeFixture, "F1-X4: the endstop torque adds to the torque of a running constant force, in both directions of the force" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 360);
	restartConstant(kEdge360 + 800);
	const double e = meanOf(600);
	REQUIRE(std::abs(e) > 100);
	const int64_t idx = startConstant(9000, 8000);
	for (int64_t direction : {9000, 27000}) {
		h().send(makeSetEffect(idx, pid::use::EtConstantForce, direction));
		restartConstant(0);
		const double c = meanOf(600);
		restartConstant(kEdge360 + 800);
		const double sum = meanOf(600);
		INFO("direction " << direction << ": endstop alone " << e << ", force alone " << c << ", both " << sum);
		REQUIRE(std::abs(c) > 100);
		CHECK_NEAR(sum, e + c, 3);
	}
	REQUIRE_CHAIN_MODELLED();
}

// Plain (MATH |t| <= power, DS 0x5E): a full-scale constant force plus a saturated endstop (penetration 7232 counts = 39.7 degrees: raw
// 126000, clipped at 32767) is clipped at the power limit and never wraps: in the direction where force and endstop agree the sum is
// exactly power in magnitude (with the sign of the endstop push), against it the sum is below power. At both ends.
TEST_CASE_FIXTURE(ShapeFixture, "F1-X4: a saturated endstop plus a full-scale constant force is clipped at power and keeps the endstop's sign" * IT_DELETES_FFBHIDMAIN) {
	prepare(12000, 360);
	const int64_t idx = startConstant(9000, 32767);
	for (int32_t side : {1, -1}) {
		bool atPower = false;
		for (int64_t direction : {9000, 27000}) {
			h().send(makeSetEffect(idx, pid::use::EtConstantForce, direction));
			restartConstant(side * 40000);
			const Window w = window(600);
			REQUIRE(w.n > 50);
			INFO("side " << side << ", direction " << direction << ": " << describe(w));
			CHECK(std::abs(static_cast<int>(w.lo)) <= 12000);
			CHECK(std::abs(static_cast<int>(w.hi)) <= 12000);
			CHECK((side > 0 ? w.hi < 12000 : w.lo > -12000));
			if (side > 0 ? w.lo == -12000 : w.hi == 12000) {
				atPower = true;
				CHECK((side > 0 ? w.hi == -12000 : w.lo == 12000));
			}
		}
		INFO("side " << side);
		CHECK(atPower);
	}
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-X6  axis-internal effects
// ---------------------------------------------------------------------------------------------------------------------

// The idle spring (Axis.cpp:690-695, :662-664) is added by calculateMechanicalEffects only while the effects calculator is inactive
// (HidFFB ffb_active false: no effect has been set since boot, or a Device Control stop / disable / reset came). Plain (header of
// Axis::updateIdleSpringForce "Changes intensity of idle spring when FFB is off"): while inactive it restores toward the centre (torque
// opposite to the displacement); once an effect is set (FFB active) it is gone (zero torque at the same displacement with a constant
// force of magnitude 0 running); a Device Control Stop All Effects brings it back.
TEST_CASE_FIXTURE(ShapeFixture, "F1-X6: the idle spring restores toward the centre only while FFB is off: set effect removes it, stop all effects brings it back" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 900);
	internalOff();
	h().command("axis.0.idlespring=40");
	for (int32_t d : {800, -800}) {
		restartConstant(d);
		const Window w = window(600);
		INFO("FFB off, displacement " << d << ": " << describe(w));
		REQUIRE(w.n > 50);
		CHECK((d > 0 ? w.negative() : w.positive()));
	}
	startConstant(9000, 0);
	for (int32_t d : {800, -800}) {
		restartConstant(d);
		const Window w = window(600);
		INFO("effect set, displacement " << d << ": " << describe(w));
		CHECK(w.zero());
	}
	control(pid::use::DcStopAllEffects);
	for (int32_t d : {800, -800}) {
		restartConstant(d);
		const Window w = window(600);
		INFO("after stop all effects, displacement " << d << ": " << describe(w));
		CHECK((d > 0 ? w.negative() : w.positive()));
	}
	REQUIRE_CHAIN_MODELLED();
}

// Plain: a stronger idlespring setting gives a stronger restoring torque at the same displacement (strictly), 0 gives none (clip 0).
TEST_CASE_FIXTURE(ShapeFixture, "F1-X6: a larger axis.0.idlespring gives a stronger restoring torque, idlespring 0 none" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 900);
	internalOff();
	restartConstant(800);
	double last = 0;
	for (int s : {0, 20, 40, 80}) {
		h().command("axis.0.idlespring=" + std::to_string(s));
		const Window w = window(600);
		INFO("idlespring " << s << ": " << describe(w));
		REQUIRE(w.n > 50);
		if (s == 0) {
			CHECK(w.zero());
		} else {
			CHECK(w.negative());
			CHECK(std::abs(w.mean) > last);
		}
		last = std::abs(w.mean);
	}
	REQUIRE_CHAIN_MODELLED();
}

// HD (INR 18): Axis::setIdleSpringStrength (Axis.cpp:669-673): scale = 0.5 + 0.01 * S, clip = min(S * 35, 10000); torque =
// -pos_scaled_16b * scale (clipped), then power / 0x7fff. pos_scaled_16b at 800 counts with D = 900: 0xffff / 900 * (800 * 360 /
// 65535) = 320.0. S = 20 -> scale 0.7, S = 40 -> 0.9, S = 80 -> 1.3 (the clips 700 / 1400 / 2800 are not reached). Tolerance 2 counts.
TEST_CASE_FIXTURE(ShapeFixture, "characterization: [F1-X6] the idle spring torque is displacement * (0.5 + 0.01 * idlespring) scaled by power" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 900);
	internalOff();
	restartConstant(800);
	const double scaler = 20000.0 / 32767.0;
	for (int s : {20, 40, 80}) {
		h().command("axis.0.idlespring=" + std::to_string(s));
		const Window w = window(600);
		const double e = 320.0 * (0.5 + 0.01 * s) * scaler;
		INFO("idlespring " << s << ": " << describe(w) << ", expected magnitude " << e);
		REQUIRE(w.n > 50);
		CHECK_NEAR(std::abs(w.mean), e, 2);
	}
	REQUIRE_CHAIN_MODELLED();
}

namespace {

struct MechFixture : ShapeFixture {
	/** One scripted-motion window of an internal effect: rampFrom(rate) after a rest, `settleMs` then 100 ms */
	Window moving(int32_t rate, uint32_t settleMs = 400) {
		restartConstant(0);
		h().run(300);
		rampFrom(rate);
		return window(settleMs, 100);
	}
	Window accelerating(int32_t a, uint32_t settleMs = 150) {
		restartConstant(0);
		h().run(300);
		parabolaFrom(a);
		return window(settleMs, 50);
	}
};

} // namespace

// PID 5.3 damper: the force opposes the velocity. Axis damper (Axis.cpp:698-701): torque is minus the filtered speed times the setting.
// Plain: opposes the motion in both directions (10 counts per cycle = 55 deg/s), twice the setting gives twice the torque (5 percent),
// setting 0 gives none. Inertia and friction are off, the idle spring is 0.
TEST_CASE_FIXTURE(MechFixture, "F1-X6: axis.0.axisdamper opposes the motion, scales with its setting and is silent at 0" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 1800);
	internalOff();
	h().command("axis.0.axisdamper=60");
	const Window up60 = moving(10);
	const Window down60 = moving(-10);
	h().command("axis.0.axisdamper=120");
	const Window up120 = moving(10);
	h().command("axis.0.axisdamper=0");
	const Window off = moving(10);
	INFO("60: +" << describe(up60) << " -" << describe(down60) << "; 120: +" << describe(up120) << "; 0: " << describe(off));
	REQUIRE(up60.n > 50);
	CHECK(up60.negative());
	CHECK(down60.positive());
	CHECK_NEAR(up120.mean / up60.mean, 2.0, 0.1);
	CHECK(off.zero());
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.3 friction: opposes the motion with a force that does not grow with the speed (Coulomb friction). Axis friction
// (Axis.cpp:709-726): above the ramp-up ceiling (speed * 45 >= 4096: 30 counts per cycle = 165 deg/s qualifies) the magnitude is the setting only.
// Plain: opposes the motion in both directions, the same magnitude at 30 and 60 counts per cycle (1 count), twice the setting twice the
// torque (5 percent), none at 0 and none at rest (the ramp-up factor is 0 at speed 0).
TEST_CASE_FIXTURE(MechFixture, "F1-X6: axis.0.axisfriction opposes the motion with a magnitude independent of the speed, scales with its setting, is silent at rest and at 0" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 1800);
	internalOff();
	h().command("axis.0.axisfriction=50");
	const Window rest = window(600);
	const Window up30 = moving(30);
	const Window up60 = moving(60);
	const Window down30 = moving(-30);
	h().command("axis.0.axisfriction=100");
	const Window up30x2 = moving(30);
	h().command("axis.0.axisfriction=0");
	const Window off = moving(30);
	INFO("rest " << describe(rest) << "; 30: +" << describe(up30) << " -" << describe(down30) << "; 60: +" << describe(up60) << "; setting 100: +" << describe(up30x2) << "; 0: " << describe(off));
	REQUIRE(up30.n > 50);
	CHECK(rest.zero());
	CHECK(up30.negative());
	CHECK(down30.positive());
	CHECK_NEAR(up60.mean, up30.mean, 1.5);
	CHECK_NEAR(up30x2.mean / up30.mean, 2.0, 0.1);
	CHECK(off.zero());
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.3 inertia: the force opposes the acceleration. Axis inertia (Axis.cpp:703-707) acts on the filtered acceleration. Plain: a
// constant acceleration of +1 / -1 count per cycle^2 (the second difference of the scripted position is exactly that) gives a torque
// opposing it, twice the setting twice the torque (5 percent), none at 0; and no torque at constant speed (inertia does not react to the speed).
TEST_CASE_FIXTURE(MechFixture, "F1-X6: axis.0.axisinertia opposes the acceleration, scales with its setting, is silent at 0 and at constant speed" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 1800);
	internalOff();
	h().command("axis.0.axisinertia=100");
	const Window plus = accelerating(1);
	const Window minus = accelerating(-1);
	h().command("axis.0.axisinertia=200");
	const Window plus2 = accelerating(1);
	const Window steady = moving(10);
	h().command("axis.0.axisinertia=0");
	const Window off = accelerating(1);
	INFO("100: +" << describe(plus) << " -" << describe(minus) << "; 200: +" << describe(plus2) << "; constant speed " << describe(steady) << "; 0: " << describe(off));
	REQUIRE(plus.n > 30);
	CHECK(plus.negative());
	CHECK(minus.positive());
	CHECK_NEAR(plus2.mean / plus.mean, 2.0, 0.1);
	CHECK(steady.zero());
	CHECK(off.zero());
	REQUIRE_CHAIN_MODELLED();
}

// HD (INR 19): absolute scales from the constants of Axis.h / EffectsCalculator.h, torque scaled by power / 0x7fff (power 20000).
//  damper:   |t| = speed[deg/s] * S * (INTERNAL_SCALER_DAMPER 40 * 0.7 / 255), speed = 10 counts * 1000 / s * 360 / 65535 = 54.93;
//  friction: |t| = S * 0.7 * 32 (above the ramp-up ceiling);
//  inertia:  |t| = accel[deg/s^2] * S * (INTERNAL_SCALER_INERTIA 4 * 0.7 / 255), accel = 1 count/cycle^2 * 1e6 * 360 / 65535 = 5493.2.
// All inside the internal clip 20000 before the scaling. Tolerance 2 counts (the filters have unity DC gain); the inertia window starts
// 450 ms after the parabola starts (accel filter and inertia filter settled).
TEST_CASE_FIXTURE(MechFixture, "characterization: [F1-X6] the absolute scale of the axis damper, friction and inertia follows the firmware constants" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 1800);
	internalOff();
	const double scaler = 20000.0 / 32767.0;
	h().command("axis.0.axisdamper=100");
	const Window d = moving(10);
	h().command("axis.0.axisdamper=0");
	h().command("axis.0.axisfriction=100");
	const Window f = moving(30);
	h().command("axis.0.axisfriction=0");
	h().command("axis.0.axisinertia=100");
	const Window i = accelerating(1, 450);
	const double ed = 10 * 1000.0 * 360.0 / 65535.0 * 100 * (40 * 0.7 / 255.0) * scaler;
	const double ef = 100 * 0.7 * 32 * scaler;
	const double ei = 1e6 * 360.0 / 65535.0 * 100 * (4 * 0.7 / 255.0) * scaler;
	INFO("damper " << describe(d) << " expected " << ed << "; friction " << describe(f) << " expected " << ef << "; inertia " << describe(i) << " expected " << ei);
	REQUIRE(d.n > 50);
	CHECK_NEAR(std::abs(d.mean), ed, 2);
	CHECK_NEAR(std::abs(f.mean), ef, 2);
	CHECK_NEAR(std::abs(i.mean), ei, 2);
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-X7  torque shaping
// ---------------------------------------------------------------------------------------------------------------------

namespace {

/** Largest change between two consecutive samples per elapsed cycle (cycle-keyed, IB 1: a missing cycle gives a longer interval); `from` is the value before the first sample */
double maxStepPerCycle(const std::vector<TorqueTrace::Sample> &s, int16_t from, uint32_t firstCycleBefore) {
	double worst = 0;
	int prev = from;
	uint32_t prevCycle = firstCycleBefore;
	for (const TorqueTrace::Sample &x : s) {
		const double dc = static_cast<double>(x.cycle - prevCycle);
		worst = std::max(worst, std::abs(static_cast<double>(x.torque) - prev) / std::max(1.0, dc));
		prev = x.torque;
		prevCycle = x.cycle;
	}
	return worst;
}

} // namespace

// Slew limiter, help "Torque rate limit in counts/ms" (Axis.cpp:186, Axis.h:484; applied in Axis.cpp:908-921 against the previous cycle's
// torque). At the default FFB rate of 1 kHz one cycle is one millisecond, so the bound is R per cycle. Plain: with slewrate R the torque at
// the chip changes by at most R per FFB cycle: when a constant force starts (0 -> T), when its direction reverses over 180 degrees (T -> -T,
// which takes at least 2 T / R cycles), and the force does reach its target (+-2 counts). Without the limit (slewrate 0) the same reversal is
// one step of the whole torque. Power 10000, magnitude 20000: |T| = 6104 (MATH, power / 32767 * magnitude).
TEST_CASE_FIXTURE(ShapeFixture, "F1-X7: slewrate R limits the change of the torque to R per 1 ms FFB cycle for a start and a reversal, and 0 disables it" * IT_DELETES_FFBHIDMAIN) {
	prepare(10000, 360);
	const double target = 20000.0 * 10000.0 / 32767.0;
	h().command("axis.0.slewrate=300");
	REQUIRE(h().getValue("axis.0.slewrate?") == 300);
	REQUIRE(h().getValue("main.hidsendspd?") == 0); // 1 kHz (FFBHIDMain.h:124-126)
	const int64_t idx = defineConstant(9000, 20000);
	TorqueTrace trace(chip1);
	trace.start(*this);
	h().observe(&trace);
	const uint32_t c0 = ffbCycles();
	start(idx);
	h().run(300);
	h().observe(nullptr);
	trace.stop();
	const double startStep = maxStepPerCycle(trace.samples(), 0, c0);
	const int16_t reached = trace.samples().back().torque;
	INFO("start: largest step per cycle " << startStep << ", final " << reached);
	REQUIRE(trace.samples().size() > 50);
	CHECK(startStep <= 300);
	CHECK_NEAR(std::abs(reached), target, 2);
	const std::vector<TorqueTrace::Sample> sBefore = record(10);
	const int16_t before = sBefore.back().torque;
	const uint32_t cBefore = sBefore.back().cycle;
	h().send(makeSetEffect(idx, pid::use::EtConstantForce, 27000));
	const std::vector<TorqueTrace::Sample> rev = record(300);
	const double revStep = maxStepPerCycle(rev, before, cBefore);
	INFO("reversal: largest step per cycle " << revStep << ", from " << before << " to " << rev.back().torque);
	CHECK(revStep <= 300);
	CHECK_NEAR(rev.back().torque, -before, 2);
	size_t transit = 0;
	for (const TorqueTrace::Sample &x : rev) {
		transit += (std::abs(x.torque) < std::abs(before) - 2) ? 1 : 0;
	}
	CHECK(transit + 3 >= static_cast<size_t>(2 * target / 300)); // at least 2 T / R cycles on the way
	h().command("axis.0.slewrate=0");
	const std::vector<TorqueTrace::Sample> sBack = record(10);
	h().send(makeSetEffect(idx, pid::use::EtConstantForce, 9000));
	const std::vector<TorqueTrace::Sample> back = record(100);
	const double backStep = maxStepPerCycle(back, sBack.back().torque, sBack.back().cycle);
	INFO("slewrate 0, reversal: largest step per cycle " << backStep);
	CHECK(backStep > 0.9 * 2 * target);
	REQUIRE_CHAIN_MODELLED();
}

// IB 26. The slew rate is documented in counts per millisecond (help Axis.cpp:186, Axis.h:484 "Maximum torque rate of change per
// millisecond"). At main.hidsendspd=1 (FFB rate 500 Hz, FFBHIDMain.h:124-126: divider 2, so one cycle is 2 ms; setReportRate passes the
// new rate to the axis, Axis::updateSamplerate sets filter_f, Axis.cpp:975-977) the intended limit is 2 R per cycle, and a start 0 -> T
// takes T / R ms. The firmware clips at R per cycle without filter_f (Axis.cpp:908-921): the start takes 2 T / R ms, the rate is half
// the documented one. R = 100, |T| = 6104 (power 10000, magnitude 20000): intended 61 ms (about 31 cycles), the per-cycle step reaches 2 R.
TEST_CASE_FIXTURE(ShapeFixture, "F1-X7: at an FFB rate of 500 Hz slewrate R still allows R counts per millisecond, 2 R per FFB cycle" * KNOWN_ISSUE("(IB 26) the slew limiter clips per FFB cycle without the sample rate (Axis.cpp:908-921): at 500 Hz the documented counts/ms rate is halved") * IT_DELETES_FFBHIDMAIN) {
	prepare(10000, 360);
	h().command("main.hidsendspd=1");
	REQUIRE(h().getValue("main.hidsendspd?") == 1);
	h().run(100);
	h().command("axis.0.slewrate=100");
	const double target = 20000.0 * 10000.0 / 32767.0;
	const int64_t idx = defineConstant(9000, 20000);
	TorqueTrace trace(chip1);
	trace.start(*this);
	h().observe(&trace);
	const uint32_t c0 = ffbCycles();
	const uint64_t t0 = HostPlatform::nowUs();
	start(idx);
	h().run(400);
	h().observe(nullptr);
	trace.stop();
	const std::vector<TorqueTrace::Sample> &s = trace.samples();
	REQUIRE(s.size() > 100);
	double worst = 0;
	int prev = 0;
	uint64_t reachedUs = 0;
	for (const TorqueTrace::Sample &x : s) {
		worst = std::max(worst, std::abs(static_cast<double>(x.torque) - prev));
		prev = x.torque;
		if (reachedUs == 0 && std::abs(x.torque) >= target - 2) {
			reachedUs = x.timeUs;
		}
	}
	const double cycleUs = static_cast<double>(s.back().timeUs - s.front().timeUs) / static_cast<double>(s.back().cycle - s.front().cycle);
	const double rampMs = static_cast<double>(reachedUs - t0) / 1000.0;
	MESSAGE("500 Hz: " << s.size() << " samples in 400 ms (" << (s.back().cycle - c0) << " cycles, " << cycleUs << " us per cycle), largest step per cycle " << worst << ", target reached after " << rampMs << " ms (intended " << target / 100 << ")");
	REQUIRE(cycleUs > 1900);
	REQUIRE(reachedUs > 0);
	REQUIRE_CHAIN_MODELLED();
	CHECK(worst >= 1.9 * 100);
	CHECK_NEAR(rampMs, target / 100, 4);
}

namespace {

/**
 * Speed limiter settings that keep both controller builds in their linear range: power P = 1000 and maxspeed M = 1000 deg/s. With a
 * speed error e = v - M (deg/s, the filtered speed of the axis):
 *  - USE_DSP_FUNCTIONS (arm_pid_f32, Axis.cpp:944-945; Kp 0.3, Ki 0.03, Kd 0, Axis.cpp:108-111 and Axis.h:37-42): CMSIS PID
 *    y[n] = y[n-1] + (Kp + Ki) e[n] - Kp e[n-1], so for a constant e the reduction is 0.33 e at once and grows by Ki e = 0.03 e per cycle;
 *  - without (Axis.cpp:947-949): reducer r = e * 0x7fff / M, I += r * Ki * power / 0x7fff (clip 0..power), reduction = Kp r + I: it grows by
 *    Ki e P / M per cycle = 0.03 e per cycle with P = M, intercept 0.3 e * 0x7fff / M = 9.83 e.
 * So the growth per cycle is 0.03 e in both builds (P = M), the intercepts differ. The speed is computed per cycle (filter_f 1000 Hz):
 * a ramp of k counts per FFB cycle is k * 1000 * 360 / 65535 deg/s (cpr 0xffff, IB 16), so 186 -> 1021.8 deg/s (e 21.8), 190 -> 1043.7
 * (e 43.7), 150 -> 824.0 (below M). The speed filter (default profile 1, Axis.h:531: 70 Hz low pass, Axis.h:526) settles within 50 ms (settling rule of the
 * speed cases: nothing is read before 60 ms after the ramp starts). D = 20000 keeps every ramp of these cases far inside the range.
 */
struct SpeedFixture : ShapeFixture {
	int s = 1; //!< sign of the torque at rest (the motion is scripted along it)
	double t0 = 0;
	void setUpSpeed() {
		prepare(1000, 20000);
		internalOff();
		startConstant(9000, 32767);
		t0 = meanOf(600);
		REQUIRE(std::abs(t0) > 900);
		s = t0 > 0 ? 1 : -1;
	}
	/** Stops the motion (position held, the filtered speed decays to 0 and the controller resets below M), waits 300 ms */
	void rest() {
		rampFrom(0);
		h().run(300);
	}
	/** From rest: ramp at `rate` counts per cycle along the force, records the next `ms` ms (sample i is ~i ms after the start) */
	std::vector<TorqueTrace::Sample> runAlong(int32_t rate, uint32_t ms) {
		rest();
		rampFrom(s * rate);
		return record(ms);
	}
	static double meanBetween(const std::vector<TorqueTrace::Sample> &v, size_t from, size_t to) {
		double sum = 0;
		size_t n = 0;
		for (size_t i = from; i < to && i < v.size(); i++) {
			sum += v[i].torque;
			n++;
		}
		REQUIRE(n > 0);
		return sum / static_cast<double>(n);
	}
};

} // namespace

// Speed limiter, help "Speed limit in deg/s" (Axis.cpp:185, applySpeedLimiterTorque Axis.cpp:924-972). Plain, as slopes of the torque
// against the speed (mean of 60..80 ms after a ramp from rest, settling rule of SpeedFixture; the motion is scripted along the force, its
// sign is the sign of the torque of a first measurement, no polar zero is used): below the limit M the slope is 0 (the force is untouched,
// +-1 count over 275 deg/s; also with the limiter off, maxspeed 0 = the default, Axis.h:483), above M the slope is negative (the force
// along the motion is reduced the more the faster the motion). The same expectations hold in the build without CMSIS-DSP
// (FFB_TESTS_USE_DSP=OFF, arm_pid_f32 replaced by the own PI, Axis.cpp:947-949; run in ../ffb-test-build/it-I1-8-nodsp: passes), which
// the SpeedFixture derivation shows to have the same growth per cycle.
TEST_CASE_FIXTURE(SpeedFixture, "F1-X7: maxspeed reduces a force along the motion above the limit, more at a higher speed, and leaves it below the limit" * IT_DELETES_FFBHIDMAIN) {
	setUpSpeed();
	const std::vector<TorqueTrace::Sample> off = runAlong(190, 100);
	h().command("axis.0.maxspeed=1000");
	REQUIRE(h().getValue("axis.0.maxspeed?") == 1000);
	const std::vector<TorqueTrace::Sample> low = runAlong(100, 100);
	const std::vector<TorqueTrace::Sample> below = runAlong(150, 100);
	const std::vector<TorqueTrace::Sample> slow = runAlong(186, 100);
	const std::vector<TorqueTrace::Sample> fast = runAlong(190, 100);
	const double mOff = meanBetween(off, 60, 80), mLow = meanBetween(low, 60, 80), mBelow = meanBetween(below, 60, 80);
	const double mSlow = meanBetween(slow, 60, 80), mFast = meanBetween(fast, 60, 80);
	const double slopeBelow = (mBelow - mLow) / ((150.0 - 100.0) * 1000.0 * 360.0 / 65535.0);  // counts per deg/s
	const double slopeAbove = s * (mFast - mSlow) / ((190.0 - 186.0) * 1000.0 * 360.0 / 65535.0);
	INFO("torque at rest " << t0 << "; 60..80 ms after the start: limiter off at 1043.7 deg/s " << mOff << ", limit 1000 at 549.3 deg/s " << mLow << ", at 824.0 deg/s " << mBelow << ", at 1021.8 deg/s " << mSlow << ", at 1043.7 deg/s " << mFast << "; slope below the limit " << slopeBelow << ", above " << slopeAbove << " counts per deg/s");
	CHECK_NEAR(mOff, t0, 1);
	CHECK_NEAR(mLow, t0, 1);
	CHECK_NEAR(mBelow, t0, 1);
	CHECK(std::abs(slopeBelow) < 0.01);
	CHECK(std::abs(mSlow) < std::abs(t0) - 5);
	CHECK(slopeAbove < -0.1);
	REQUIRE_CHAIN_MODELLED();
}

// HD (INR 20): the limiter acts only for torque along the motion (effectiveSpeed = speed * sign(torque), Axis.cpp:935): moving against the
// force at 1043.7 deg/s with maxspeed 1000 leaves the force untouched (+-2 counts).
TEST_CASE_FIXTURE(SpeedFixture, "characterization: [F1-X7] the speed limiter leaves a force that points against the motion untouched" * IT_DELETES_FFBHIDMAIN) {
	setUpSpeed();
	h().command("axis.0.maxspeed=1000");
	rest();
	rampFrom(-s * 190);
	const double against = meanOf(500);
	INFO("torque at rest " << t0 << ", moving against it at 1043.7 deg/s with maxspeed 1000: " << against);
	CHECK_NEAR(against, t0, 2);
	REQUIRE_CHAIN_MODELLED();
}

// HD (INR 21): growth of the reduction per cycle, Ki * e (derivation at SpeedFixture: the same 0.03 e per cycle in the DSP and the non-DSP
// build because P = M). e = 1043.7 - 1000 = 43.7 deg/s -> 1.311 counts per cycle. Least-squares slope of |torque| over the cycle index
// between 100 and 300 ms after the start (filter settled, reduction below |T| in both builds: DSP 14 + 1.3 n, non-DSP 430 + 1.3 n).
// Tolerance 5 percent.
TEST_CASE_FIXTURE(SpeedFixture, "characterization: [F1-X7] above maxspeed the reduction of the force grows by 0.03 counts per cycle per deg/s of excess speed" * IT_DELETES_FFBHIDMAIN) {
	setUpSpeed();
	h().command("axis.0.maxspeed=1000");
	const std::vector<TorqueTrace::Sample> v = runAlong(190, 320);
	REQUIRE(v.size() > 300);
	double sx = 0, sy = 0, sxx = 0, sxy = 0;
	size_t n = 0;
	for (size_t i = 100; i < 300; i++) {
		const double x = v[i].cycle, y = std::abs(static_cast<double>(v[i].torque));
		sx += x;
		sy += y;
		sxx += x * x;
		sxy += x * y;
		n++;
	}
	const double slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
	const double e = 190.0 * 1000.0 * 360.0 / 65535.0 - 1000.0;
	INFO("torque at rest " << t0 << ", at 100 ms " << v[100].torque << ", at 300 ms " << v[299].torque << ": slope " << slope << " counts per cycle, expected " << -0.03 * e);
	CHECK(v[299].torque * s > 0); // still in the linear range
	CHECK_NEAR(slope, -0.03 * e, 0.05 * 0.03 * e);
	REQUIRE_CHAIN_MODELLED();
}

// HD (INR 22): the reduction is clipped at |torque| (Axis.cpp:954, comment Axis.cpp:952-953 "only reduce the magnitude of the torque, not invert
// it"; no document says what a speed limiter should do at the end of its range): after 2 s at 1043.7 deg/s the force along the motion is
// 0 and no cycle had the opposite sign.
TEST_CASE_FIXTURE(SpeedFixture, "characterization: [F1-X7] a long run above maxspeed reduces the force along the motion to 0 and never reverses it" * IT_DELETES_FFBHIDMAIN) {
	setUpSpeed();
	h().command("axis.0.maxspeed=1000");
	const std::vector<TorqueTrace::Sample> v = runAlong(190, 2000);
	REQUIRE(v.size() > 1900);
	int16_t opposite = 0;
	for (const TorqueTrace::Sample &x : v) {
		if (x.torque * s < opposite * s) {
			opposite = x.torque;
		}
	}
	INFO("torque at rest " << t0 << ", most opposite sample " << opposite << ", last " << v.back().torque);
	CHECK(opposite * s >= 0);
	CHECK(v.back().torque == 0);
	REQUIRE_CHAIN_MODELLED();
}

// Fade-in (Axis::startForceFadeIn, Axis.cpp:998-1001; callers :384, :449, :620, :640). axis.0.zeroenc (Axis::setPos) restarts a fade.
// Plain: a running constant force of torque T drops below T at the restart, rises without ever falling again (+-1 count) and ends at T (+-2).
TEST_CASE_FIXTURE(ShapeFixture, "F1-X7: a restarted fade-in takes the force below its value and brings it back monotonically to the full torque" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 360);
	startConstant(9000, 20000);
	const double t = meanOf(600);
	REQUIRE(std::abs(t) > 1000);
	TorqueTrace trace(chip1);
	trace.start(*this);
	h().observe(&trace);
	h().command("axis.0.zeroenc?");
	h().run(2500);
	h().observe(nullptr);
	trace.stop();
	const std::vector<TorqueTrace::Sample> &s = trace.samples();
	REQUIRE(s.size() > 100);
	size_t low = 0;
	for (size_t i = 0; i < s.size(); i++) {
		if (std::abs(s[i].torque) < std::abs(s[low].torque)) {
			low = i;
		}
	}
	bool monotone = true;
	for (size_t i = low + 1; i < s.size(); i++) {
		monotone = monotone && std::abs(s[i].torque) >= std::abs(s[i - 1].torque) - 1;
	}
	INFO("torque " << t << ", lowest " << s[low].torque << " at sample " << low << " of " << s.size() << ", last " << s.back().torque);
	CHECK(std::abs(s[low].torque) < 0.9 * std::abs(t));
	CHECK(monotone);
	CHECK_NEAR(s.back().torque, t, 2);
	REQUIRE_CHAIN_MODELLED();
}

namespace {

/** The fade after axis.0.zeroenc: (torque before, samples of the 2500 ms after the command, index of the lowest sample) */
struct FadeRun {
	double t = 0;
	std::vector<TorqueTrace::Sample> s;
	size_t low = 0;
};

FadeRun zeroencFade(ShapeFixture &f) {
	FadeRun r;
	f.prepare(20000, 360);
	f.startConstant(9000, 20000);
	r.t = f.meanOf(600);
	REQUIRE(std::abs(r.t) > 1000);
	TorqueTrace trace(f.chip1);
	trace.start(f);
	f.h().observe(&trace);
	f.h().command("axis.0.zeroenc?");
	f.h().run(2500);
	f.h().observe(nullptr);
	trace.stop();
	r.s = trace.samples();
	REQUIRE(r.s.size() > 100);
	for (size_t i = 0; i < r.s.size(); i++) {
		if (std::abs(r.s[i].torque) < std::abs(r.s[r.low].torque)) {
			r.low = i;
		}
	}
	return r;
}

} // namespace

// IB 23 (same as coverage bug 30). Axis::startForceFadeIn is documented "Starts fading in force from start to 1 over fadeTime"
// (Axis.cpp:996, Axis.h:361 "fadeTime" in seconds) and Axis::setPos calls startForceFadeIn(0.25, 0.5) (Axis.cpp:384): intended, the force is
// back at T 0.5 s after axis.0.zeroenc, about 500 FFB cycles at 1 kHz (+-20). The firmware adds fadeTime / filter_f per cycle
// (Axis.cpp:872-876), so 0.75 / (0.5 / 1000) = 1500 cycles: 1.5 s at any FFB rate (updateSamplerate sets filter_f to the FFB rate,
// Axis.cpp:975-977).
TEST_CASE_FIXTURE(ShapeFixture, "F1-X7: the fade-in after axis.0.zeroenc brings the force back to full torque within its fade time of 0.5 s" * KNOWN_ISSUE("(IB 23; same as bug 30) the fade adds fadeTime / filter_f per cycle (Axis.cpp:872-876): the zeroenc fade of 0.5 s takes 1.5 s") * IT_DELETES_FFBHIDMAIN) {
	const FadeRun r = zeroencFade(*this);
	size_t full = r.low;
	while (full < r.s.size() && std::abs(r.s[full].torque) < std::abs(r.t) - 2) {
		full++;
	}
	REQUIRE(full < r.s.size());
	const double cycles = static_cast<double>(r.s[full].cycle - r.s[r.low].cycle);
	MESSAGE("torque " << r.t << ", lowest " << r.s[r.low].torque << ", cycles from the lowest sample to the full torque " << cycles);
	REQUIRE_CHAIN_MODELLED();
	CHECK_NEAR(cycles, 500.0, 20);
}

// HD (INR 23): the zeroenc fade starts at a quarter of the torque (startForceFadeIn(0.25, ...), Axis.cpp:384; the first cycle after the
// command applies the multiplier 0.25 before incrementing it, Axis.cpp:872-876): |lowest| = trunc(0.25 * |T|), +-2 counts.
TEST_CASE_FIXTURE(ShapeFixture, "characterization: [F1-X7] the fade-in after axis.0.zeroenc starts at a quarter of the torque" * IT_DELETES_FFBHIDMAIN) {
	const FadeRun r = zeroencFade(*this);
	INFO("torque " << r.t << ", lowest " << r.s[r.low].torque);
	CHECK_NEAR(std::abs(r.s[r.low].torque), 0.25 * std::abs(r.t), 2);
	REQUIRE_CHAIN_MODELLED();
}

namespace {

struct ExpoFixture : ShapeFixture {
	/** mean torque of a constant force (direction 9000) of `magnitude` after `settleMs` */
	double torqueFor(int64_t idx, int64_t magnitude, uint32_t settleMs = 500) {
		setMagnitude(idx, magnitude);
		return meanOf(settleMs);
	}
};

} // namespace

// Expo (Axis.cpp:1019-1037, help "Exponential curve correction (x^(val/exposcale)+1)"). Plain: with expo != 0 the shaping leaves 0 at 0,
// the sign of the force unchanged (also for the reversed direction), full scale at full scale (n = 1 stays 1: +-2 counts) and is strictly
// monotone over four magnitudes.
TEST_CASE_FIXTURE(ExpoFixture, "F1-X7: axis.0.expo keeps zero at zero, the sign, the full-scale value and the monotony of the force" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 360);
	const int64_t idx = defineConstant(9000, 0);
	start(idx);
	const int mags[] = {0, 4000, 8000, 16000, 32767};
	for (int expo : {50, -50}) {
		h().command("axis.0.expo=" + std::to_string(expo));
		REQUIRE(h().getValue("axis.0.expo?") == expo);
		double lin[5], shaped[5];
		h().command("axis.0.expo=0");
		for (int i = 0; i < 5; i++) {
			lin[i] = torqueFor(idx, mags[i]);
		}
		h().command("axis.0.expo=" + std::to_string(expo));
		double last = -1;
		for (int i = 0; i < 5; i++) {
			shaped[i] = torqueFor(idx, mags[i]);
			INFO("expo " << expo << ", magnitude " << mags[i] << ": linear " << lin[i] << ", shaped " << shaped[i]);
			CHECK(std::abs(shaped[i]) > last);
			last = std::abs(shaped[i]);
			if (i > 0) {
				CHECK(((shaped[i] > 0) == (lin[i] > 0)));
			}
		}
		CHECK_NEAR(shaped[0], 0, 0);
		CHECK_NEAR(shaped[4], lin[4], 2);
		h().send(makeSetEffect(idx, pid::use::EtConstantForce, 27000));
		const double rev = torqueFor(idx, 16000);
		INFO("expo " << expo << ", reversed direction at 16000: " << rev << " against " << shaped[3]);
		CHECK_NEAR(rev, -shaped[3], 2);
		h().send(makeSetEffect(idx, pid::use::EtConstantForce, 9000));
	}
	REQUIRE_CHAIN_MODELLED();
}

// HD (INR 24): the curve is torque = T_full * n^p with n = magnitude / 32767 and p = 1 + v / 50 for v > 0, p = 1 / (1 + |v| / 50) for
// v < 0 (Axis::setExpo Axis.cpp:1019-1031, exposcale 50). v = 50 -> p = 2, v = -50 -> p = 0.5. Tolerance 2 counts.
TEST_CASE_FIXTURE(ExpoFixture, "characterization: [F1-X7] the expo curve is n to the power 1 + v / 50 for v > 0 and n to the power 1 / (1 + |v| / 50) for v < 0" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 360);
	REQUIRE(h().getValue("axis.0.exposcale?") == 50);
	const int64_t idx = defineConstant(9000, 0);
	start(idx);
	h().command("axis.0.expo=0");
	const double full = torqueFor(idx, 32767);
	for (int expo : {50, -50, 25}) {
		h().command("axis.0.expo=" + std::to_string(expo));
		const double p = expo > 0 ? 1.0 + expo / 50.0 : 1.0 / (1.0 + std::abs(expo) / 50.0);
		for (int m : {8000, 16000, 24000}) {
			const double got = torqueFor(idx, m);
			const double e = full * std::pow(m / 32767.0, p);
			INFO("expo " << expo << " (p " << p << "), magnitude " << m << ": " << got << ", expected " << e);
			CHECK_NEAR(got, e, 2);
		}
	}
	REQUIRE_CHAIN_MODELLED();
}

// Equalizer (Axis.cpp:148-155, :798-804; help "Equalizer enable", "Equalizer band 1 gain (-120/120)"). Plain: disabled it is transparent
// whatever the band gains are (+-1 count); enabled with all gains 0 it is transparent too (a 0 dB filter, +-2 counts); a force of 0 stays 0;
// a larger band-1 gain gives a larger torque for a constant force (band 1 is the lowest, 10 Hz low shelf, the only band with DC gain
// other than 1), the sign is unchanged.
TEST_CASE_FIXTURE(ExpoFixture, "F1-X7: the equalizer is transparent when disabled or flat, keeps zero and the sign, and a larger low band gain gives a larger constant torque" * IT_DELETES_FFBHIDMAIN) {
	prepare(20000, 360);
	const int64_t idx = defineConstant(9000, 8000);
	start(idx);
	const double ref = meanOf(600);
	REQUIRE(std::abs(ref) > 500);
	h().command("axis.0.eqb1=60");
	const double disabledBoost = meanOf(600);
	h().command("axis.0.equalizer=1");
	REQUIRE(h().getValue("axis.0.equalizer?") == 1);
	h().command("axis.0.eqb1=0");
	const double flat = meanOf(600);
	h().command("axis.0.eqb1=-60");
	const double cut = meanOf(600);
	h().command("axis.0.eqb1=60");
	const double boost = meanOf(600);
	setMagnitude(idx, 0);
	const double zero = meanOf(600);
	INFO("reference " << ref << ", disabled with eqb1 60: " << disabledBoost << ", flat " << flat << ", eqb1 -60: " << cut << ", eqb1 60: " << boost << ", magnitude 0: " << zero);
	CHECK_NEAR(disabledBoost, ref, 1);
	CHECK_NEAR(flat, ref, 2);
	CHECK(std::abs(cut) < std::abs(flat));
	CHECK(std::abs(boost) > std::abs(flat));
	CHECK(((cut > 0) == (ref > 0)));
	CHECK(((boost > 0) == (ref > 0)));
	CHECK_NEAR(zero, 0, 1);
	REQUIRE_CHAIN_MODELLED();
}

// HD (INR 32): setEqGain (Axis.cpp:1413-1418) passes gain / 10 dB to the biquad (setPeakGain); the low shelf's gain at DC is that many
// dB: eqb1 = 60 -> +6 dB = x1.995, eqb1 = -60 -> x0.501, eqb1 = 120 -> +12 dB = x3.981. Power 32767 makes the torque scaler exactly 1
// (Axis.cpp:501), so the chip carries the axis value itself: expected = trunc(F * 10^(g / 200)) with F the reference torque, +-2 counts
// (float biquad coefficients, one truncation). Magnitude 4000 keeps 4 F inside the range. The other bands (15 / 25 / 40 / 60 Hz peaks,
// 100 Hz high shelf, Axis.h:549) have unity gain at DC.
TEST_CASE_FIXTURE(ExpoFixture, "characterization: [F1-X7] a band-1 gain of g / 10 dB scales a constant force by 10 to the power g / 200, bands 2 to 6 do not at DC" * IT_DELETES_FFBHIDMAIN) {
	prepare(32767, 360);
	const int64_t idx = defineConstant(9000, 4000);
	start(idx);
	const double ref = meanOf(600);
	REQUIRE(std::abs(ref) > 3000);
	h().command("axis.0.equalizer=1");
	for (int g : {60, -60, 120}) {
		h().command("axis.0.eqb1=" + std::to_string(g));
		const double got = meanOf(800);
		const double e = std::trunc(ref * std::pow(10.0, g / 200.0));
		INFO("eqb1 " << g << ": " << got << ", expected " << e);
		CHECK_NEAR(got, e, 2);
	}
	h().command("axis.0.eqb1=0");
	for (int b = 2; b <= 6; b++) {
		h().command("axis.0.eqb" + std::to_string(b) + "=100");
	}
	const double others = meanOf(800);
	INFO("bands 2 to 6 at 100: " << others << " against " << ref);
	CHECK_NEAR(others, ref, 2);
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// Seam S2  late driver readiness; stale target after the out-of-bounds stop
// ---------------------------------------------------------------------------------------------------------------------

namespace {

struct LateFixture : ShapeFixture {
	/** Bring-up to the driver in Shutdown only (the PC has not connected yet); the host side is ready */
	void upLate() {
		reachShutdown();
		host.emplace(*this);
	}
	/**
	 * The PC plugs the device in (tud_umount_cb deletes all effects, tud_mount_cb -> startMotor), `send` runs (effects sent while the
	 * driver is not Running yet: the replug is the only way to start a fresh-flash driver, and a replug deletes earlier effects, so
	 * they must come after it), then the chain runs until the driver is Running and motorReady (as mountAndRun does). A replug deleting
	 * all effects is intended for a detach (USB 2.0 9.1.1); a bus suspend takes the same path (tud_suspend_cb, global_callbacks.cpp:365-367,
	 * FFBHIDMain::usbSuspend): that belongs to F1-T7 (I2), noted here only.
	 */
	template <class F> void mountSendRun(F send) {
		replugUsb();
		REQUIRE(!(tmc()->getState() == TMC_ControlState::Running && tmc()->motorReady()));
		send();
		bool upNow = false;
		for (uint32_t ms = 0; ms < 20000 && !upNow; ms++) {
			upNow = tmc()->getState() == TMC_ControlState::Running && tmc()->motorReady();
			if (!upNow) {
				sched.advanceMs(1);
			}
		}
		REQUIRE(upNow);
	}
	/** Torque values written to 0x64 after write index `from`, in order */
	std::vector<int16_t> writesFrom(size_t from) {
		std::vector<int16_t> out;
		const std::vector<FluxTorque> all = fluxTorqueWrites(chip1);
		for (size_t i = from; i < all.size(); i++) {
			out.push_back(all[i].torque);
		}
		return out;
	}
};

} // namespace

// Effects sent while the driver is not Running (reachShutdown, USB replug, effects, then the wait for Running): the torque the host asked
// for must reach the chip once the driver runs (Axis::updateDriveTorque writes only on change and only with motorReady, Axis.cpp:488-495;
// the fade-in of the first ready cycle, Axis.cpp:447-449, makes the torque change every cycle for 1000 cycles, so the steady value does
// arrive). Plain: 2500 ms after Running the chip's torque is the effect torque (MATH: magnitude 20000 * power 10000 / 32767 = 6103.7,
// truncated twice: +-2 counts), no write ever exceeds it in magnitude and none wraps.
TEST_CASE_FIXTURE(LateFixture, "F1-X7: a constant force set before the driver runs reaches the chip at its full torque once the driver is Running" * IT_DELETES_FFBHIDMAIN) {
	upLate();
	configure(10000, 360);
	script.setConstant(0);
	attachPositionScript(script);
	const size_t mark = fluxTorqueWrites(chip1).size();
	mountSendRun([&] { startConstant(9000, 20000); });
	setSpiFrameUs(0);
	h().run(2500);
	const double target = 20000.0 * 10000.0 / 32767.0;
	const std::vector<int16_t> w = writesFrom(mark);
	int worst = 0;
	for (int16_t v : w) {
		worst = std::max(worst, std::abs(static_cast<int>(v)));
	}
	INFO("torque writes after the mount " << w.size() << ", largest magnitude " << worst << ", torque now " << chipTorque() << ", target " << target);
	CHECK_NEAR(std::abs(chipTorque()), target, 2);
	CHECK(worst <= target + 2);
	REQUIRE_CHAIN_MODELLED();
}

// Seam S2 + slew limiter, short not-ready window. With slewrate R the chip's torque never changes by more than R between two writes (help
// text, see above), also across the moment the driver starts running: the chip held 0x64 = 0 until then. Axis::updateDriveTorque drops
// the command while the driver is not ready (Axis.cpp:488-495) but updateTorque advances metric.previous.torque (the slew limiter's base,
// Axis.cpp:908-921). In THIS path that does no harm: readiness comes about 104 ms after the replug, while the usbResume fade (from 0,
// Axis.cpp:640, running on the not-ready cycles too, Axis.cpp:872-876) still keeps previous.torque near 0, so the first writes are small.
// The property holds only for a not-ready window shorter than that fade. A longer window through the replug path never reaches Running
// (IB 24, next case), so the slew jump of coverage bug 33 is not reachable through the chain on THIS path; it is reachable through a
// short USB detach of a running wheel (IB 39, `test_f1_driver.cpp`, WO I2-2).
TEST_CASE_FIXTURE(LateFixture, "F1-X7: with slewrate R the first torque written after the driver runs is within R of the value the chip held" * IT_DELETES_FFBHIDMAIN) {
	upLate();
	configure(10000, 360);
	script.setConstant(0);
	attachPositionScript(script);
	h().command("axis.0.slewrate=300");
	const int16_t held = chipTorque();
	const size_t mark = fluxTorqueWrites(chip1).size();
	mountSendRun([&] {
		startConstant(9000, 20000);
		h().run(5); // the axis computes the torque, the driver drops it
	});
	setSpiFrameUs(0);
	h().run(200);
	const std::vector<int16_t> w = writesFrom(mark);
	REQUIRE(!w.empty());
	double worst = 0;
	int prev = held;
	for (int16_t v : w) {
		worst = std::max(worst, std::abs(static_cast<double>(v) - prev));
		prev = v;
	}
	INFO("chip held " << held << ", writes after the mount " << w.size() << ", first " << w.front() << ", largest step between writes " << worst);
	REQUIRE_CHAIN_MODELLED();
	CHECK(worst <= 300);
}

// IB 24 (related to coverage bug 152: the same waitPower recovery, TMC4671.cpp:643-650). Start request lost during a supply wait.
// The supply drops (driver: waitPower, TMC4671.cpp:643-650, the state before it saved in requestedState), then the PC
// connects (replug: usbResume -> startMotor, Axis.cpp:639-643). TMC4671::startMotor only asks for Running when the state is Shutdown
// (TMC4671.cpp:1682-1687) and so only sets motorEnabledRequested. When the supply returns, the state machine leaves waitPower for the
// saved requestedState Shutdown (TMC4671.cpp:453, 473-492) and nothing asks for Running again: PWM is on, MODE_MOTION stays stopped, the
// driver is never Running. Intended (firmware sequence, charter F1-X7 "recover correctly after the driver becomes ready late"): startMotor
// records the request in motorEnabledRequested (TMC4671.cpp:1683) and the start-up sequence honours a recorded request (EncoderFinished,
// TMC4671.cpp:619-621), so the motor runs once power is there.
TEST_CASE_FIXTURE(LateFixture, "F1-X7: the motor starts once the supply returns after the PC connected while the supply was off" * KNOWN_ISSUE("(IB 24; related to bug 152) a start request made while the driver waits for power is lost: after the supply returns the driver stays in Shutdown (TMC4671.cpp:453, 1682-1687) with MODE_MOTION stopped") * IT_DELETES_FFBHIDMAIN) {
	upLate();
	configure(10000, 360);
	script.setConstant(0);
	attachPositionScript(script);
	powerOff();
	h().run(500);
	REQUIRE(static_cast<uint32_t>(tmc()->getState()) == 1); // waitPower
	replugUsb();
	h().run(300);
	REQUIRE(!driverReady());
	startConstant(9000, 20000);
	h().run(2500);
	powerOn();
	const bool ready = runUntilReady(20000);
	MESSAGE("after the supply returned: TMC state " << static_cast<uint32_t>(tmc()->getState()) << " (2 Shutdown, 3 Running), ready " << ready << ", MODE_MOTION " << (chip1.reg(0x63) & 0xff) << ", PWM " << (chip1.reg(0x1A) & 0xff));
	REQUIRE_CHAIN_MODELLED();
	CHECK(ready);
}

// IB 25 (extends coverage bug 152: the undervoltage recovery does not restore the motion mode). Supply dropout while a constant force runs:
// the driver stops the motion (TMC4671.cpp:643-650) and waits for power; the host sets the magnitude to 0 meanwhile (PID 1.01 5.6:
// Set Constant Force changes the force of the running effect). After the supply returns and the driver is Running and ready again, the chip
// must carry the torque the axis now computes (charter rule 3: 0x64 equals the axis torque, axis.0.curtorque) within a few FFB cycles,
// with MODE_MOTION back on. The firmware writes 0x64 only when the axis torque changes (Axis.cpp:488-495), and only the first
// not-ready -> ready transition after boot forces one (the fade, Axis.cpp:447-449): the axis torque went to 0 while the driver dropped it,
// so the chip keeps the old force.
TEST_CASE_FIXTURE(ShapeFixture, "F1-X7: after a supply dropout the chip carries the torque the axis computes now, not the force set before the dropout" * KNOWN_ISSUE("(IB 25) after a supply dropout 0x64 keeps the torque from before it: updateDriveTorque writes only on a change (Axis.cpp:488-495), see bug 152") * IT_DELETES_FFBHIDMAIN) {
	prepare(10000, 360);
	const int64_t idx = startConstant(9000, 20000);
	const Window before = window(1000);
	REQUIRE((before.positive() || before.negative()));
	powerOff();
	h().run(500);
	const uint32_t offState = static_cast<uint32_t>(tmc()->getState());
	REQUIRE(!driverReady());
	setMagnitude(idx, 0);
	h().run(300);
	powerOn();
	REQUIRE(runUntilReady(20000));
	h().run(3);
	const int64_t axisTorque = h().getValue("axis.0.curtorque?");
	const int16_t chip = chipTorque();
	const uint32_t mode = chip1.reg(0x63) & 0xff;
	MESSAGE("before the dropout " << describe(before) << "; TMC state during it " << offState << "; after: axis torque " << axisTorque << ", 0x64 torque " << chip << ", MODE_MOTION " << mode);
	REQUIRE(axisTorque == 0);
	REQUIRE_CHAIN_MODELLED();
	CHECK(chip == axisTorque);
	CHECK(mode == 1);
}

// IB 25, second path: no effect runs before the dropout (FFB inactive, idle spring 0: the chip holds 0); the host starts a constant force
// during the dropout. After the supply returns the axis computes the effect torque but the chip never gets it (no change after ready,
// Axis.cpp:488-495): the force the host started is lost. Intended as above: 0x64 equals axis.0.curtorque within a few cycles.
TEST_CASE_FIXTURE(ShapeFixture, "F1-X7: an effect started during a supply dropout reaches the chip once the driver is ready again" * KNOWN_ISSUE("(IB 25) an effect started during a supply dropout never reaches 0x64: updateDriveTorque writes only on a change (Axis.cpp:488-495), see bug 152") * IT_DELETES_FFBHIDMAIN) {
	prepare(10000, 360);
	const Window before = window(200);
	REQUIRE(before.zero());
	powerOff();
	h().run(500);
	REQUIRE(!driverReady());
	startConstant(9000, 20000);
	h().run(300);
	powerOn();
	REQUIRE(runUntilReady(20000));
	h().run(3);
	const int64_t axisTorque = h().getValue("axis.0.curtorque?");
	const int16_t chip = chipTorque();
	const uint32_t mode = chip1.reg(0x63) & 0xff;
	MESSAGE("after the dropout: axis torque " << axisTorque << ", 0x64 torque " << chip << ", MODE_MOTION " << mode);
	REQUIRE(axisTorque != 0);
	REQUIRE_CHAIN_MODELLED();
	CHECK(chip == axisTorque);
	CHECK(mode == 1);
}

// IB 22. Stale target after the out-of-bounds shutdown (WO I1-7 review). Oracle: PID 1.01 5.13 Device Control Stop All Effects (no effect
// plays, so the host asks for no force) and charter rule 3 (0x64 equals the axis torque). The out-of-bounds stop (Axis.cpp:431-444,
// TMC4671::stopMotor) leaves 0x64 at the last torque (turn() is not called while the driver is stopped). The host stops all effects while
// the axis is out of bounds (the axis torque becomes 0, idle spring 0) and the script returns to the centre; sys.errorsclr clears the
// error and Axis::errorCallback restarts the motor (Axis.cpp:480-485; MODE_MOTION back to torque mode, PWM on, TMC4671.cpp:1682-1694).
// The value 0x64 holds at that moment is the old force, and it stays: the axis torque does not change any more, so turn() is never
// called (probe: 3 s after errorsclr 0x64 -3051, MODE_MOTION 1, PWM 7, axis.0.curtorque 0). Without the Stop All Effects the restart
// correctly drives the running force. Intended: the drive comes back on with 0 at 0x64.
TEST_CASE_FIXTURE(ShapeFixture, "F1-X5: the motor restarts after the out-of-bounds stop without driving the old torque the host has withdrawn" * KNOWN_ISSUE("(IB 22) 0x64 keeps the last torque through an out-of-bounds stop and the restart after sys.errorsclr turns the drive on with it (Axis.cpp:488-495, errorCallback :480-485)") * IT_DELETES_FFBHIDMAIN) {
	prepare(5000, 900);
	startConstant(9000, 20000);
	const Window before = window(1000);
	REQUIRE((before.positive() || before.negative()));
	restartConstant(2000000000);
	h().run(100);
	REQUIRE((chip1.reg(0x63) & 0xff) == 0); // stopped
	const int16_t stale = chipTorque();
	control(pid::use::DcStopAllEffects);
	restartConstant(0);
	h().run(200);
	const size_t mark = chip1.log().size();
	int16_t atOn = static_cast<int16_t>(0x7fff);
	bool seen = false;
	int16_t target = chipTorque();
	h().command("sys.errorsclr?");
	h().run(20);
	const std::vector<FakeTmc4671::Frame> &log = chip1.log();
	for (size_t i = mark; i < log.size() && !seen; i++) {
		const FakeTmc4671::Frame &f = log[i];
		if (!f.write) {
			continue;
		}
		if (f.addr == 0x64) {
			target = static_cast<int16_t>((f.data >> 16) & 0xffff);
		} else if ((f.addr == 0x63 && (f.data & 0xff) != 0) || (f.addr == 0x1A && (f.data & 0xff) != 0)) {
			atOn = target;
			seen = true;
		}
	}
	INFO("torque before the stop " << before.mean << ", 0x64 while stopped " << stale << ", 0x64 when the drive came back on " << atOn << ", MODE_MOTION " << (chip1.reg(0x63) & 0xff) << ", PWM " << (chip1.reg(0x1A) & 0xff));
	REQUIRE(seen);
	REQUIRE(stale != 0);
	REQUIRE_CHAIN_MODELLED();
	CHECK(atOn == 0);
}

} // TEST_SUITE
