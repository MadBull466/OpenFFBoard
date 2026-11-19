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
// (SystemCommands.cpp:68, :143-144), but the encoder-direction rows move to F1-T9 (I2): encdir sets only abnconf.rdir and re-runs
// setup_ABN_Enc (TMC4671.cpp:3028-3035), not conf.encoderReversed, which is what turn() uses (:1753, and only with an external encoder);
// the encoder type of the fresh-flash bring-up is NONE (INR 1); and FakeTmc4671 does not tie the ABN direction to the scripted 0x6B.
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
// Axis.cpp:1259-1264 reports it). DS 0x6B: one revolution is 65536 counts. Intended: axis.0.cpr reports 65536.
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

// Endstop onset (Axis::calculateEndstopTorque, Axis.cpp:828-838): with D = 360 the soft endstop acts beyond +-D/2 = 180 degrees =
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
		// INR 8: threshold from Axis.cpp:433 (|scaled| > 0xffff), scale from the firmware
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
// (Axis.cpp:433). INR 8: with D = 900 the drive is on at 0.9 D (810 degrees = 147456 counts) and off at 1.1 D (990 degrees = 180224).
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

} // TEST_SUITE
