/*
 * F1 condition effects with a scripted position (WO I1-5): F1-E5 spring, F1-E6 damper, F1-E7 inertia, F1-E8 friction.
 *
 * Chain: PID reports over the real USB stack -> HidFFB -> EffectsCalculator -> Axis -> TMC4671 driver -> register 0x64 of the fake chip.
 * The position is scripted through 0x6B (PositionScript, no plant, open loop); a value written in step k is read by the FFB cycle of
 * step k + 1 (PositionScript.h). The torque target is observed per FFB cycle (TorqueTrace).
 *
 * Oracles
 *  - USB_PID1_01.pdf 5.3 (p. 12) Condition Block Parameters: "force resulting from a condition is equal to A(q - q0)"; below
 *    CP Offset - Dead Band: force = Negative Coefficient * (q - (CP Offset - Dead Band)); above CP Offset + Dead Band:
 *    force = Positive Coefficient * (q - (CP Offset + Dead Band)); Positive / Negative Saturation = "maximum positive / negative force
 *    output"; Dead Band = "the condition is not active between Offset - Dead Band and Offset + Dead Band"; "A spring condition uses axis
 *    position as the metric. A damper ... velocity ... An inertia ... acceleration"; a two-axis spring with CP Offset 0 "would have the same
 *    effect as the joystick self-centering spring" (so a spring with positive coefficients is restoring, a damper opposes the motion, an
 *    inertia opposes the acceleration). PID 5.3 gives no friction formula and says nothing about negative coefficients (the Normalized
 *    values of p. 1 are 0..100 percent); the negative-coefficient case relies on the descriptor, see F1-E8 below. 5.1: Effect Gain, Direction Enable.
 *  - TMC4671-LA datasheet Rev 2.07, register 0x6B PID_POSITION_ACTUAL (p. 98-99): s32, lower 16 bits = one mechanical revolution,
 *    so 65536 counts = 360 degrees.
 *  - Descriptor FFBWheel sends: FFBWheel.cpp:41-51 picks hid_2ffb_desc (usb_hid_2ffb_desc.c) because target_constants.h leaves
 *    FFBWHEEL_USE_1AXIS_DESC and HIDAXISRES_USE_32B_DESC undefined (the 1-axis variant usb_hid_1ffb_desc.c:33-34 declares Axes Enable and
 *    Direction Enable with logical 0..0 and is not the one sent; the 2-axis one declares logical 0..1, usb_hid_2ffb_desc.c:187-188). Its Set
 *    Condition report declares CP Offset and both coefficients signed, logical -32768..32767 (usb_hid_2ffb_desc.c:298-309); the cases read
 *    this from the descriptor parsed over USB.
 *
 * Host side: "positive" in PID 5.3 is the host's axis (the HID gamepad X value). The joystick input report is decoded from the descriptor
 * read over USB (hostX(), the report FfbHost's drainHidIn keeps); the side assertions (asymmetric coefficients, saturations) take the
 * side with X > 0 as the positive one, so they hold for any axis.0.invert. X follows invert (AxesManager.cpp:62, Axis::getLastScaledEnc).
 *
 * Mapping counts -> normalised position q (MATH). The axis maps an angle to q = 65535 * angle / degrees (q = +-0x7fff at +-degrees / 2).
 * With axis.0.degrees = 3600 and angle = 360 * counts / cpr (cpr = 65536 or 65535, the mapping constant is HD / seam S1, checked in I1-7):
 * q = 65535 * 360 * counts / (cpr * 3600) = counts / 10 (within 1.5e-5). Cases use counts that are 10 * the q they intend and
 * tolerances derived below, so S1 does not matter. All positions stay inside |q| < 0x7fff: no endstop torque.
 *
 * Settings the cases depend on (set by CDC command, rule 2 / 6): axis.0.invert 0 (firmware default 1, Axis.h:510; with 0 a host-positive
 * displacement is a positive 0x6B count, which keeps the physical sign statements simple; the host-anchored side cases also run with
 * the default 1), axis.0.power 20000 (every measured torque is checked to be below 19000, so none is clipped), axis.0.fxratio 255,
 * axis.0.degrees 3600, the Axis internal always-on damper / inertia / friction axisdamper / axisinertia / axisfriction 0 (default
 * axisdamper is 30 and would add its own opposing force), fx.0.spring / damper / inertia / friction gain 255 (read back; firmware default
 * 64 / 64 / 127 / 254, EffectsCalculator.h:40), fx.0.frictionPctSpeedToRampup 25 (set and read back; firmware default 25,
 * EffectsCalculator.h:238). Effect Gain 255, Direction Enable off, one condition block (Parameter Block Offset 0). The idle spring only
 * acts while FFB is off.
 * Every case relies on the axis-magnitude promotion 0 -> 1.0 of HidFFB::set_condition (HidFFB.cpp:391-393): newEffect sends Set Effect
 * without Axes Enable X, so axisMagnitudes[0] is 0 until the Set Condition arrives. The order cases (condition before effect) belong to
 * I1-6 / F1-E9 and are not tested here.
 *
 * Moving position (item of the review, IB 1): the FFB period is 1001 us, so about one step in 1000 has no FFB cycle. A position function
 * keyed to the step time would show two steps of motion to the next cycle. The scripted ramps and parabolas are therefore keyed to the FFB
 * cycle count (ffbCycles(), local helpers rampFrom / parabolaFrom): the value written in a step is base + f(cycles so far), the cycle after
 * it reads exactly one more increment than the cycle before. Every FFB cycle sees exactly one increment, so the speed per cycle is exactly
 * the rate (the axis computes speed per cycle with filter_f = 1000, so the numbers are in counts per cycle = "per ms" of the file).
 *
 * Settling rule (from the filter settings, not from output). Fade-in after bring-up: 1.0 s (Axis.cpp startForceFadeIn(0, 1.0)), the cases
 * wait 1100 ms after configuring. Speed filter 40 Hz / Q 0.55 (Axis.h:526), accel filter 40 Hz / Q 0.30; the damper biquad 30 Hz / Q 0.4,
 * friction 50 Hz / Q 0.2, inertia 15 Hz / Q 0.2 (EffectsCalculator.h:61-64). All are RBJ low passes whose DC gain is (b0+b1+b2)/(1+a1+a2) = 1
 * (Filters.cpp calcBiquad), so in a steady state the output equals the unfiltered product. Slowest pole of each (s = w0 (-1/(2Q) +
 * sqrt(1/(4Q^2) - 1))): damper 0.5 w0 -> tau 10.6 ms, friction 0.21 w0 -> 15 ms, inertia 0.21 w0 -> 50 ms, speed filter 4.4 ms, accel filter
 * 12 ms. Every steady measurement runs 500 ms (damper, friction, parabola 450 ms, constant-speed inertia 1000 ms) = >= 8 tau of the slowest
 * pole (3e-4 residue) and requires the last 50 FFB cycles to differ by at most SPREAD counts (3: the torque is the float chain
 * rounded to int32 force and to int16 torque, +-1 count each, so a constant input gives 2 values at most 2 counts apart, 3 with a carry).
 *
 * Ratio tolerance (derived): every torque is a truncation chain, force (int32) then torque = (int)(force * power / 32767): error <= 1 count
 * of torque plus 0.61 count from the force truncation, so +-1.7 counts. A ratio of two values both >= 800 counts is therefore within
 * 2 * 1.7 / 800 = 0.43 percent; ratio tolerance 0.01 where both values are >= 800 counts, absolute tolerances (counts) elsewhere.
 * A difference of two differences (dead-band slope) carries four such errors: 4 * 1.7 = 6.8 counts.
 *
 * Float to int (input for F1-B2, nothing is tested here): spring force = float clipped to +-saturation and narrowed to int32 in
 * calcConditionEffectForce; the largest reachable spring force (coefficient 32767, |q| 32767, scale 16) is 5.2e5, inside int32. Damper /
 * inertia: the metric is float (a 0x6B step of d counts in one cycle = 5.49 d deg/s; damper metric 40 * that, force 4 * metric) and the
 * float -> int32 conversion happens when the force is passed to clip<int32_t, int32_t> before the clip: out of range for a jump of
 * about d = 2.4e6 counts per cycle at coefficient 32767 (878 d > 2.1e9). Friction: `(int32_t)speed` at EffectsCalculator.cpp:394 (speed =
 * 247 d) is out of range for d = 8.7e6. The friction coefficient is read into a uint16_t from an int16_t field (EffectsCalculator.cpp:408,
 * seam S4, IB 19).
 */
#include "doctest.h"

#include "FfbHost.h"
#include "HidReportDescriptor.h"
#include "IntegrationFixture.h"
#include "PidReportBuilder.h"
#include "PositionScript.h"
#include "TestHelpers.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using hid::ReportType;

namespace {

constexpr uint32_t kDirectionX = hid::usage(0x0A, 1);
constexpr int kPower = 20000;

struct CondFixture : IntegrationFixture {
	std::optional<FfbHost> host;
	PositionScript script;
	int lastDir = 0; //!< damperAt / parabola: sign of the change of the host X value over the last moving measurement

	FfbHost &h() { return *host; }

	/** Chain up, 0 us per SPI frame, host, settings of the file comment, script at 0, fade-in over */
	void up(int invert = 0) {
		bringUp();
		setSpiFrameUs(0);
		host.emplace(*this);
		h().command("axis.0.power=" + std::to_string(kPower));
		h().command("axis.0.fxratio=255");
		h().command("axis.0.degrees=3600");
		h().command("axis.0.invert=" + std::to_string(invert));
		h().command("axis.0.axisdamper=0");
		h().command("axis.0.axisinertia=0");
		h().command("axis.0.axisfriction=0");
		for (const char *g : {"spring", "damper", "inertia", "friction"}) {
			h().command(std::string("fx.0.") + g + "=255");
			REQUIRE(h().getValue(std::string("fx.0.") + g + "?") == 255);
		}
		h().command("fx.0.frictionPctSpeedToRampup=25");
		REQUIRE(h().getValue("fx.0.frictionPctSpeedToRampup?") == 25);
		script.setConstant(0);
		attachPositionScript(script);
		h().run(1100);
	}

	int64_t newEffect(uint32_t type, int64_t direction = 0, bool directionEnable = false) {
		const FfbHost::CreateResult c = h().createEffect(type);
		REQUIRE(c.load.success());
		const int64_t idx = c.load.effectBlockIndex;
		pid::PidReport se = h().builder().make(ReportType::Output, pid::use::SetEffectReport);
		se.set(pid::use::EffectBlockIndex, idx).select(type).set(pid::use::Duration, 32767).set(pid::use::Gain, 255);
		se.flag(pid::use::DirectionEnable, directionEnable);
		se.setIn({pid::use::Direction}, kDirectionX, direction);
		h().send(se);
		return idx;
	}
	void setCondition(int64_t idx, int64_t offset, int64_t pc, int64_t nc, int64_t ps, int64_t ns, int64_t db) {
		pid::PidReport cond = h().builder().make(ReportType::Output, pid::use::SetConditionReport);
		cond.set(pid::use::EffectBlockIndex, idx).set(pid::use::ParameterBlockOffset, 0).set(pid::use::CpOffset, offset);
		cond.set(pid::use::PositiveCoefficient, pc).set(pid::use::NegativeCoefficient, nc);
		cond.set(pid::use::PositiveSaturation, ps).set(pid::use::NegativeSaturation, ns).set(pid::use::DeadBand, db);
		h().send(cond);
	}
	void start(int64_t idx) {
		pid::PidReport eo = h().builder().make(ReportType::Output, pid::use::EffectOperationReport);
		eo.set(pid::use::EffectBlockIndex, idx).select(pid::use::OpEffectStart).set(pid::use::LoopCount, 1);
		h().send(eo);
	}

	/** The X value of the newest HID joystick input report (decoded from the descriptor read over USB); REQUIREs that one exists */
	int64_t hostX() {
		drainHidIn();
		const hid::Field *f = nullptr;
		for (const hid::Field &fl : h().descriptor().fields()) {
			if (fl.type == ReportType::Input && fl.count > 0 && fl.usageOf(0) == hid::usage(0x01, 0x30)) {
				f = &fl;
				break;
			}
		}
		REQUIRE(f != nullptr);
		const uint32_t first = h().descriptor().usesReportIds() ? 1 : 0;
		const uint32_t lastBit = f->bitOffset + f->bitSize;
		for (size_t i = hidInReports.size(); i-- > 0;) {
			const std::vector<uint8_t> &b = hidInReports[i].bytes;
			if (first == 1 && (b.empty() || b[0] != f->reportId)) {
				continue;
			}
			if (b.size() < first + (lastBit + 7) / 8) {
				continue;
			}
			uint64_t v = 0;
			for (uint32_t k = 0; k < f->bitSize; k++) {
				const uint32_t bit = f->bitOffset + k;
				v |= static_cast<uint64_t>((b[first + bit / 8] >> (bit % 8)) & 1) << k;
			}
			if (f->isSigned && f->bitSize < 64 && (v >> (f->bitSize - 1)) != 0) {
				return static_cast<int64_t>(v) - (static_cast<int64_t>(1) << f->bitSize);
			}
			return static_cast<int64_t>(v);
		}
		FAIL("no HID joystick input report with an X value was captured");
		return 0;
	}

	/** Runs `ms` ms and returns the mean torque of the last `last` FFB cycles; REQUIRE: enough cycles, spread <= `spread`, below the power clip */
	int meas(uint32_t ms, uint32_t last = 50, int spread = 3) {
		TorqueTrace trace(chip1);
		trace.start(*this);
		h().observe(&trace);
		h().run(ms);
		h().observe(nullptr);
		const auto &s = trace.samples();
		REQUIRE(s.size() >= last);
		int lo = 100000, hi = -100000;
		double sum = 0;
		for (size_t i = s.size() - last; i < s.size(); i++) {
			lo = std::min<int>(lo, s[i].torque);
			hi = std::max<int>(hi, s[i].torque);
			sum += s[i].torque;
		}
		INFO("window min " << lo << " max " << hi);
		REQUIRE(hi - lo <= spread);
		REQUIRE(std::abs(lo) < kPower - 1000);
		REQUIRE(std::abs(hi) < kPower - 1000);
		return static_cast<int>(std::lround(sum / last));
	}

	/** Spring: hold at `counts`, wait 100 ms, measure; the host X value is returned in `x` if given */
	int springAt(int32_t counts, int64_t *x = nullptr) {
		script.setConstant(counts);
		const int t = meas(100, 30);
		if (x != nullptr) {
			*x = hostX();
		}
		return t;
	}

	/** Position moves with `rate` counts per FFB cycle from the current value (file comment: keyed to the FFB cycle count) */
	void rampFrom(int32_t rate) {
		const int32_t base = script.lastValue();
		const uint32_t c0 = ffbCycles();
		script.setFunction([=, this](uint64_t) { return static_cast<int32_t>(base + static_cast<int64_t>(rate) * static_cast<int64_t>(ffbCycles() - c0)); });
	}
	/** Constant acceleration `a` counts per cycle^2 from rest: p(n) = base + a n (n + 1) / 2, second difference exactly a */
	void parabolaFrom(int32_t a) {
		const int32_t base = script.lastValue();
		const uint32_t c0 = ffbCycles();
		script.setFunction([=, this](uint64_t) {
			const int64_t n = static_cast<int64_t>(ffbCycles() - c0);
			return static_cast<int32_t>(base + a * (n * (n + 1) / 2));
		});
	}
	/** Moving condition: ramp at `rate`, run 500 ms, steady mean of the last 50 cycles; lastDir = direction of the host X value */
	int damperAt(int32_t rate, int spread = 3) {
		const int64_t x0 = hostX();
		rampFrom(rate);
		const int t = meas(500, 50, spread);
		const int64_t x1 = hostX();
		lastDir = (x1 > x0) - (x1 < x0);
		return t;
	}
	/**
	 * Holds the position and waits 1000 ms: the stop is a speed (and acceleration) spike that saturates the filters; the slowest pole is
	 * the inertia filter's, tau 50 ms (file comment), 1000 ms = 20 tau leaves 2e-9 of the spike.
	 */
	void settleHold() {
		script.setConstant(script.lastValue());
		h().run(1000);
	}
	int parabolaAt(int32_t a) {
		const int64_t x0 = hostX();
		parabolaFrom(a);
		const int t = meas(450, 50, 3);
		const int64_t x1 = hostX();
		lastDir = (x1 > x0) - (x1 < x0);
		return t;
	}
	/** Splits two measurements (torque, host direction or X sign) into the one on the host-positive side and the other; REQUIREs opposite signs */
	static void bySide(int t1, int64_t d1, int t2, int64_t d2, int &pos, int &neg) {
		REQUIRE(d1 != 0);
		REQUIRE(d2 != 0);
		REQUIRE((d1 > 0) != (d2 > 0));
		pos = d1 > 0 ? t1 : t2;
		neg = d1 > 0 ? t2 : t1;
	}
};

} // namespace

TEST_SUITE("F1Conditions") {

// ---------------------------------------------------------------------------------------------------------------------
// F1-E5 spring
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.3: spring with positive coefficients is the self-centering spring: the torque at the chip opposes the displacement from CP Offset 0
// on both sides, and is 0 at the centre. +-6000 counts = q +-600.
TEST_CASE_FIXTURE(CondFixture, "F1-E5: a spring torque opposes the scripted displacement on both sides of the centre and is zero at the centre" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtSpring);
	setCondition(idx, 0, 16000, 16000, 32767, 32767, 0);
	start(idx);
	const int c = springAt(0);
	const int plus = springAt(6000);
	const int minus = springAt(-6000);
	MESSAGE("centre " << c << ", +6000 counts " << plus << ", -6000 counts " << minus);
	CHECK(c == 0);
	CHECK(plus < 0);
	CHECK(minus > 0);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.3 formula: Positive Coefficient on the positive side (the host's axis: X > 0), Negative Coefficient on the negative side.
// 20000 : 10000 gives the ratio -2 (opposite torques). Run for invert 0 and for the firmware default 1: the host X value decides the side.
TEST_CASE_FIXTURE(CondFixture, "F1-E5: asymmetric spring coefficients give the host-positive and host-negative side in the ratio of the coefficients" * IT_DELETES_FFBHIDMAIN) {
	for (int invert : {0, 1}) {
		SUBCASE(invert == 0 ? "axis.0.invert 0" : "axis.0.invert 1 (default)") {
			up(invert);
			const int64_t idx = newEffect(pid::use::EtSpring);
			setCondition(idx, 0, 20000, 10000, 32767, 32767, 0);
			start(idx);
			int64_t xa = 0, xb = 0;
			const int ta = springAt(6000, &xa);
			const int tb = springAt(-6000, &xb);
			int pos = 0, neg = 0;
			bySide(ta, xa, tb, xb, pos, neg);
			MESSAGE("invert " << invert << ": +6000 counts X " << xa << " torque " << ta << ", -6000 counts X " << xb << " torque " << tb);
			REQUIRE(neg != 0);
			CHECK_NEAR(static_cast<double>(pos) / neg, -2.0, 0.02);
			CHECK(std::abs(pos) > std::abs(neg));
		}
	}
}

// PID 5.3: force = A (q - q0) is linear in the displacement: doubling the displacement doubles the torque, on both sides.
TEST_CASE_FIXTURE(CondFixture, "F1-E5: the spring torque is linear in the displacement" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtSpring);
	setCondition(idx, 0, 16000, 16000, 32767, 32767, 0);
	start(idx);
	const int p1 = springAt(3000), p2 = springAt(6000), n1 = springAt(-3000), n2 = springAt(-6000);
	MESSAGE("+3000 " << p1 << " +6000 " << p2 << " -3000 " << n1 << " -6000 " << n2);
	REQUIRE(p1 != 0);
	REQUIRE(n1 != 0);
	CHECK_NEAR(static_cast<double>(p2) / p1, 2.0, 0.01);
	CHECK_NEAR(static_cast<double>(n2) / n1, 2.0, 0.01);
}

// PID 5.3: no force between CP Offset +- Dead Band; outside, force = A (q - (CP Offset +- Dead Band)). Dead Band 300 (q): +-1500 counts
// (q 150) is inside; q 600 and q 900 give (600 - 300) : (900 - 300) = 1 : 2 on both sides.
TEST_CASE_FIXTURE(CondFixture, "F1-E5: a spring has no torque inside the dead band and measures from the dead band edge outside it" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtSpring);
	setCondition(idx, 0, 16000, 16000, 32767, 32767, 300);
	start(idx);
	CHECK(springAt(1500) == 0);
	CHECK(springAt(-1500) == 0);
	const int p6 = springAt(6000), p9 = springAt(9000), n6 = springAt(-6000), n9 = springAt(-9000);
	MESSAGE("q600 " << p6 << " q900 " << p9 << " q-600 " << n6 << " q-900 " << n9);
	REQUIRE(p6 != 0);
	REQUIRE(n6 != 0);
	CHECK_NEAR(static_cast<double>(p9) / p6, 2.0, 0.01);
	CHECK_NEAR(static_cast<double>(n9) / n6, 2.0, 0.01);
	CHECK(p6 < 0);
	CHECK(n6 > 0);
}

// PID 5.3: Positive / Negative Saturation = maximum force output of each side (host-positive side: X > 0). Coefficient 32767 saturates
// quickly (force 16 q). Below the saturation both sides are equal (equal coefficients); above it each side is constant, in the ratio of the
// saturations 3000 : 1500 (the saturated torque 1831 and 915 counts: ratio error 0.2 percent).
TEST_CASE_FIXTURE(CondFixture, "F1-E5: a spring is clipped at the positive and negative saturation" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtSpring);
	setCondition(idx, 0, 32767, 32767, 3000, 1500, 0);
	start(idx);
	const int p_small = springAt(500), n_small = springAt(-500);	// q 50: force about 800 < 1500
	int64_t xa3 = 0, xb3 = 0, xa6 = 0, xb6 = 0;
	const int a3 = springAt(3000, &xa3), a6 = springAt(6000, &xa6), b3 = springAt(-3000, &xb3), b6 = springAt(-6000, &xb6);
	int pos6 = 0, neg6 = 0, pos3 = 0, neg3 = 0;
	bySide(a6, xa6, b6, xb6, pos6, neg6);
	bySide(a3, xa3, b3, xb3, pos3, neg3);
	MESSAGE("small " << p_small << " / " << n_small << ", host-positive side: " << pos3 << " " << pos6 << ", host-negative side: " << neg3 << " " << neg6);
	REQUIRE(n_small != 0);
	CHECK_NEAR(static_cast<double>(p_small) / n_small, -1.0, 0.01);
	CHECK_NEAR(pos6, pos3, 3);
	CHECK_NEAR(neg6, neg3, 3);
	REQUIRE(neg6 != 0);
	CHECK_NEAR(static_cast<double>(pos6) / neg6, -2.0, 0.01);
	CHECK(std::abs(pos3) > std::abs(p_small));
}

// PID 5.3: CP Offset is the neutral value of the metric. CP Offset 1000 (q): the zero is at 10000 counts (axis.0.invert 0: host X = counts
// side), the torque at offset +- 3000 counts is opposite and equal (equal coefficients), restoring toward the offset.
TEST_CASE_FIXTURE(CondFixture, "F1-E5: CP Offset shifts the zero of the spring" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtSpring);
	setCondition(idx, 1000, 16000, 16000, 32767, 32767, 0);
	start(idx);
	const int atZero = springAt(10005);		// q 1000.5: force about +2 counts
	const int above = springAt(13000), below = springAt(7000);
	MESSAGE("at offset " << atZero << ", above " << above << ", below " << below);
	CHECK(std::abs(atZero) <= 8);
	CHECK(above < 0);
	CHECK(below > 0);
	REQUIRE(below != 0);
	CHECK_NEAR(static_cast<double>(above) / below, -1.0, 0.01);
	// the centre of the other sign moves the zero the other way
	setCondition(idx, -1000, 16000, 16000, 32767, 32767, 0);
	h().run(20);
	const int neg0 = springAt(-10005);
	CHECK(std::abs(neg0) <= 8);
	CHECK(springAt(0) < 0);		// 0 counts is above the offset -1000
}

// The formula is linear in the coefficient, so a negative coefficient gives the force the opposite sign (anti-restoring), with the
// same size as the positive one. The descriptor declares the coefficients signed (file comment), the spring path treats them as signed.
TEST_CASE_FIXTURE(CondFixture, "F1-E5: a negative spring coefficient reverses the torque with the same size" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtSpring);
	setCondition(idx, 0, 15000, 15000, 32767, 32767, 0);
	start(idx);
	const int pos = springAt(6000);
	setCondition(idx, 0, -15000, -15000, 32767, 32767, 0);
	const int neg = springAt(6000);
	MESSAGE("coefficient +15000: " << pos << ", -15000: " << neg);
	CHECK(pos < 0);
	CHECK(neg > 0);
	CHECK_NEAR(neg, -pos, 4);
}

// PID 5.1: with Direction Enable the Direction is a polar direction and a force along theta acts with a factor that reverses when theta
// changes by 180 degrees. Set Effect (Direction Enable) first, then Set Condition. Only the reversal is asserted.
TEST_CASE_FIXTURE(CondFixture, "F1-E5: with Direction Enable the spring torque reverses when the direction changes by 180 degrees" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t a = newEffect(pid::use::EtSpring, 9000, true);
	setCondition(a, 0, 16000, 16000, 32767, 32767, 0);
	start(a);
	const int t9 = springAt(6000);
	const int64_t b = newEffect(pid::use::EtSpring, 27000, true);
	setCondition(b, 0, 16000, 16000, 32767, 32767, 0);
	h().run(5);
	pid::PidReport eo = h().builder().make(ReportType::Output, pid::use::EffectOperationReport);
	eo.set(pid::use::EffectBlockIndex, a).select(pid::use::OpEffectStop).set(pid::use::LoopCount, 1);
	h().send(eo);
	start(b);
	const int t27 = springAt(6000);
	MESSAGE("direction 9000: " << t9 << ", 27000: " << t27);
	REQUIRE(t9 != 0);
	CHECK_NEAR(t27, -t9, 4);
}

// characterization: the absolute direction -> sign mapping of the polar direction. Polar zero undefined, PID 1.01 5.1; reconciled by the lead.
// Firmware: axisMagnitudes[0] = sin(2 pi direction / 36000) (HidFFB.cpp:337), Direction 9000 = +1: restoring spring (T < 0 for +6000).
// INR 14
TEST_CASE_FIXTURE(CondFixture, "characterization: [F1-E5] polar direction 9000 gives the restoring spring torque, 27000 the reversed one" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t a = newEffect(pid::use::EtSpring, 9000, true);
	setCondition(a, 0, 16000, 16000, 32767, 32767, 0);
	start(a);
	const int t9 = springAt(6000);
	CHECK(t9 < 0);
}

// characterization: absolute scale, one setting. HD chain: q = 1000 (counts 10005 -> q 1000.5 truncated to 1000), force = 16000 / 32767 *
// (255 + 1) / 256 * 16 (scaler.spring, EffectsCalculator.h:52) * 1000 = 7813.6 -> int 7813, global gain 255 / 255, fxratio 255 / 255 -> torque
// 7813 * 20000 / 32767 = 4769 (opposing the positive displacement: -4769).
// INR 15
TEST_CASE_FIXTURE(CondFixture, "characterization: [F1-E5] spring coefficient 16000 at q 1000 gives the hand-derived torque count" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtSpring);
	setCondition(idx, 0, 16000, 16000, 32767, 32767, 0);
	start(idx);
	const int t = springAt(10005);
	MESSAGE("torque " << t);
	CHECK_NEAR(t, -4769, 6);
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-E6 damper
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.3: the damper metric is the velocity: the torque opposes the motion in both directions.
TEST_CASE_FIXTURE(CondFixture, "F1-E6: a damper torque opposes the scripted motion in both directions" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtDamper);
	setCondition(idx, 0, 32767, 32767, 32767, 32767, 0);
	start(idx);
	const int up_ = damperAt(4);
	const int down = damperAt(-4);
	script.setConstant(script.lastValue());
	const int still = meas(600, 50, 3);
	MESSAGE("+4 counts/ms " << up_ << ", -4 counts/ms " << down << ", standstill after the ramp " << still);
	CHECK(up_ < 0);
	CHECK(down > 0);
	CHECK(std::abs(still) <= 2);		// zero at standstill after the transient (>= 50 tau of the filters)
}

// PID 5.3 with DC gain 1 filters (file comment): steady torque = coefficient * unfiltered velocity, proportional to the speed.
TEST_CASE_FIXTURE(CondFixture, "F1-E6: the damper torque is proportional to the speed in a steady state" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtDamper);
	setCondition(idx, 0, 32767, 32767, 32767, 32767, 0);
	start(idx);
	const int t2 = damperAt(2), t4 = damperAt(4), t8 = damperAt(8);
	MESSAGE("2: " << t2 << ", 4: " << t4 << ", 8: " << t8);
	REQUIRE(t2 != 0);
	CHECK_NEAR(static_cast<double>(t4) / t2, 2.0, 0.01);
	CHECK_NEAR(static_cast<double>(t8) / t4, 2.0, 0.01);
}

// PID 5.3: dead band on the velocity metric: no force inside (both signs of the speed); outside force = A (v - D), i.e. the same slope as
// without the dead band and a constant offset A D. Dead Band D = 1000: 1 count/ms (metric about 220: HD scale 40 * deg/s, margin 4) is
// inside, 8 and 10 counts/ms (about 1760 and 2200) outside. Slope equality is independent of the metric scale; tolerance 7 counts (four
// truncation errors, file comment).
TEST_CASE_FIXTURE(CondFixture, "F1-E6: a damper has no torque inside the velocity dead band on both sides and the same slope outside it" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtDamper);
	setCondition(idx, 0, 32767, 32767, 32767, 32767, 0);
	start(idx);
	const int f8 = damperAt(8), f10 = damperAt(10);
	setCondition(idx, 0, 32767, 32767, 32767, 32767, 1000);
	const int inside = damperAt(1);
	const int insideNeg = damperAt(-1);
	const int d8 = damperAt(8), d10 = damperAt(10), dm8 = damperAt(-8);
	MESSAGE("no dead band: 8 -> " << f8 << ", 10 -> " << f10 << "; dead band: 1 -> " << inside << ", -1 -> " << insideNeg << ", 8 -> " << d8 << ", 10 -> " << d10 << ", -8 -> " << dm8);
	CHECK(inside == 0);
	CHECK(insideNeg == 0);
	CHECK_NEAR(d10 - d8, f10 - f8, 7);
	CHECK(std::abs(d8) < std::abs(f8));
	CHECK(d8 < 0);
	CHECK_NEAR(dm8, -d8, 3);
}

// PID 5.3: Positive / Negative Saturation clip the damper force per side (host-positive motion uses the positive one). 800 : 400 -> ratio 2
// (243 counts: 1.7 / 243 = 0.7 percent, tolerance 0.02).
TEST_CASE_FIXTURE(CondFixture, "F1-E6: a damper is clipped at the saturation of the side of the motion" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtDamper);
	setCondition(idx, 0, 32767, 32767, 800, 400, 0);
	start(idx);
	const int a8 = damperAt(8);
	const int da = lastDir;
	const int a10 = damperAt(10);
	const int b8 = damperAt(-8);
	const int db = lastDir;
	int pos = 0, neg = 0;
	bySide(a8, da, b8, db, pos, neg);
	MESSAGE("+8 " << a8 << " +10 " << a10 << " -8 " << b8);
	CHECK_NEAR(a10, a8, 3);
	REQUIRE(neg != 0);
	CHECK_NEAR(static_cast<double>(pos) / neg, -2.0, 0.02);
}

// PID 5.3: Positive / Negative Coefficient per side for the velocity metric: 32767 : 16384 -> ratio -2 between host-positive and host-negative motion.
TEST_CASE_FIXTURE(CondFixture, "F1-E6: asymmetric damper coefficients give the host-positive and host-negative motion in the ratio of the coefficients" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtDamper);
	setCondition(idx, 0, 32767, 16384, 32767, 32767, 0);
	start(idx);
	const int a = damperAt(4);
	const int da = lastDir;
	const int b = damperAt(-4);
	const int db = lastDir;
	int pos = 0, neg = 0;
	bySide(a, da, b, db, pos, neg);
	MESSAGE("host-positive " << pos << ", host-negative " << neg);
	REQUIRE(neg != 0);
	CHECK_NEAR(static_cast<double>(pos) / neg, -32767.0 / 16384.0, 0.02);
}

// characterization: absolute scale of the damper (HD chain, cpr 65536 assumed, +-0.002 % for 65535): 4 counts/ms = 4 * 1000 * 360 / 65536 = 21.97 deg/s,
// metric = speed * INTERNAL_SCALER_DAMPER (40) = 878.9, force = 32767 / 32767 * (255 + 1) / 256 * scaler.damper (4.0) * 878.9 = 3515.6, torque
// = 3515.6 * 20000 / 32767 = 2145.7 (negative for positive speed).
// INR 15
TEST_CASE_FIXTURE(CondFixture, "characterization: [F1-E6] damper coefficient 32767 at 4 counts/ms gives the hand-derived torque count" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtDamper);
	setCondition(idx, 0, 32767, 32767, 32767, 32767, 0);
	start(idx);
	const int t = damperAt(4);
	MESSAGE("torque " << t);
	CHECK_NEAR(t, -2146, 6);
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-E7 inertia
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.3: the inertia metric is the acceleration: the torque opposes it, in both signs, proportional to it. Parabolic positions with an
// integer second difference a counts / cycle^2 (accel metric exact). Coefficient 1000 keeps the torque far below the clip.
TEST_CASE_FIXTURE(CondFixture, "F1-E7: an inertia torque opposes the acceleration and is proportional to it in a steady state" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtInertia);
	setCondition(idx, 0, 1000, 1000, 32767, 32767, 0);
	start(idx);
	// +1 runs from 0 up to 101k counts, -1 back down to 0, +2 up to 202k counts (q 20k < 0x7fff): the position never jumps
	const int a1 = parabolaAt(1);
	settleHold();
	const int m1 = parabolaAt(-1);
	settleHold();
	const int a2 = parabolaAt(2);
	MESSAGE("a=1: " << a1 << ", a=2: " << a2 << ", a=-1: " << m1);
	CHECK(a1 < 0);
	CHECK(m1 > 0);
	REQUIRE(a1 != 0);
	CHECK_NEAR(static_cast<double>(a2) / a1, 2.0, 0.01);
	CHECK_NEAR(m1, -a1, 3);
}

// PID 5.3: Positive / Negative Coefficient per side for the acceleration metric: 1000 : 500 -> ratio -2 between host-positive and
// host-negative acceleration (the sides by the host X direction of the parabola).
TEST_CASE_FIXTURE(CondFixture, "F1-E7: asymmetric inertia coefficients give the host-positive and host-negative acceleration in the ratio of the coefficients" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtInertia);
	setCondition(idx, 0, 1000, 500, 32767, 32767, 0);
	start(idx);
	const int a = parabolaAt(1);
	const int da = lastDir;
	settleHold();
	const int b = parabolaAt(-1);
	const int db = lastDir;
	int pos = 0, neg = 0;
	bySide(a, da, b, db, pos, neg);
	MESSAGE("host-positive " << pos << ", host-negative " << neg);
	REQUIRE(neg != 0);
	CHECK_NEAR(static_cast<double>(pos) / neg, -2.0, 0.02);
}

// PID 5.3: at constant speed the acceleration is zero: after the start transient (accel step decays with tau 50 ms: 1000 ms = 20 tau) the
// inertia torque is 0.
TEST_CASE_FIXTURE(CondFixture, "F1-E7: an inertia torque is zero during constant speed after the transient" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtInertia);
	setCondition(idx, 0, 1000, 1000, 32767, 32767, 0);
	start(idx);
	rampFrom(8);
	const int steady = meas(1000, 50, 3);
	MESSAGE("constant speed 8 counts/ms: " << steady);
	CHECK(std::abs(steady) <= 2);
}

// characterization: absolute scale of the inertia: 1 count/cycle^2 = 360 / 65536 * 1e6 = 5493.2 deg/s^2, metric * INTERNAL_SCALER_INERTIA (4) = 21972.7,
// force = 1000 / 32767 * 1 * scaler.inertia (2.0) * 21972.7 = 1341.2, torque = 1341.2 * 20000 / 32767 = 818.7 (negative for positive acceleration).
// INR 15
TEST_CASE_FIXTURE(CondFixture, "characterization: [F1-E7] inertia coefficient 1000 at 1 count/ms^2 gives the hand-derived torque count" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtInertia);
	setCondition(idx, 0, 1000, 1000, 32767, 32767, 0);
	start(idx);
	const int t = parabolaAt(1);
	MESSAGE("torque " << t);
	CHECK_NEAR(t, -819, 4);
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-E8 friction
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.3: friction opposes the motion in both directions. 100 counts/ms is far above the ramp-up speed (about 33 counts/ms, HD, file
// comment of the ramp case) so the coefficient acts in full; it saturates: 100 and 150 counts/ms give the same torque.
TEST_CASE_FIXTURE(CondFixture, "F1-E8: a friction torque opposes the motion and does not grow with the speed above the ramp-up speed" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtFriction);
	setCondition(idx, 0, 8000, 8000, 32767, 32767, 0);
	start(idx);
	const int p100 = damperAt(100), p150 = damperAt(150), m100 = damperAt(-100);
	MESSAGE("+100 " << p100 << ", +150 " << p150 << ", -100 " << m100);
	CHECK(p100 < 0);
	CHECK(m100 > 0);
	CHECK_NEAR(p150, p100, 3);
	CHECK_NEAR(m100, -p100, 3);
}

// PID 5.3: Saturation per side clips the friction force: 6000 : 3000 with coefficient 20000 (host-positive motion: positive saturation).
TEST_CASE_FIXTURE(CondFixture, "F1-E8: a friction is clipped at the saturation of the side of the motion" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtFriction);
	setCondition(idx, 0, 20000, 20000, 6000, 3000, 0);
	start(idx);
	const int a = damperAt(100);
	const int da = lastDir;
	const int b = damperAt(-100);
	const int db = lastDir;
	int pos = 0, neg = 0;
	bySide(a, da, b, db, pos, neg);
	MESSAGE("host-positive motion " << pos << ", host-negative " << neg);
	REQUIRE(neg != 0);
	CHECK_NEAR(static_cast<double>(pos) / neg, -2.0, 0.01);
}

// Command documentation (EffectsCalculator.cpp:61): frictionPctSpeedToRampup is the "% of max speed for gradual increase". The force
// ramps up smoothly from zero: slower than the ramp-up speed the torque is smaller the lower the speed, and a longer ramp (50 %
// instead of 25 %) gives less torque at the same speed below the ramp. Speeds 1, 4, 16 counts/ms (ramp-up speed 33 or 66 counts/ms, HD).
TEST_CASE_FIXTURE(CondFixture, "F1-E8: the friction torque ramps up with the speed below the ramp-up speed, slower for a larger ramp-up percentage" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtFriction);
	setCondition(idx, 0, 8000, 8000, 32767, 32767, 0);
	start(idx);
	const int full = std::abs(damperAt(-100));
	const int s1 = std::abs(damperAt(1)), s4 = std::abs(damperAt(4)), s16 = std::abs(damperAt(16));
	h().command("fx.0.frictionPctSpeedToRampup=50");
	REQUIRE(h().getValue("fx.0.frictionPctSpeedToRampup?") == 50);
	const int s16long = std::abs(damperAt(16));
	MESSAGE("full " << full << ", 1: " << s1 << ", 4: " << s4 << ", 16: " << s16 << ", 16 with 50 %: " << s16long);
	REQUIRE(full > 0);
	CHECK(s1 < s4);
	CHECK(s4 < s16);
	CHECK(s16 < full);
	CHECK(s1 < full / 50);		// smooth start: well below 2 percent of the full value at 1 count/ms
	CHECK(s16long < s16);
}

// PID 5.3 (dead band, CP Offset, applied to the velocity metric of the friction like to the damper): no torque inside the dead band on both
// sides, the torque outside it. Dead Band D = 3000 (metric; HD scale 247 per count/ms, 12 counts/ms): 3 counts/ms (metric 742, margin 4)
// is inside, with D = 0 it gives a clearly non-zero torque (so the zero is the dead band); 100 counts/ms is outside and far above
// the ramp, the full torque; 20 counts/ms is on the ramp shifted by D: less than without a dead band.
TEST_CASE_FIXTURE(CondFixture, "F1-E8: a friction has no torque inside the dead band on both sides and the full torque far outside it" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtFriction);
	setCondition(idx, 0, 8000, 8000, 32767, 32767, 0);
	start(idx);
	const int base3 = std::abs(damperAt(3)), base20 = std::abs(damperAt(20)), full = std::abs(damperAt(100));
	setCondition(idx, 0, 8000, 8000, 32767, 32767, 3000);
	const int in_p = damperAt(3), in_n = damperAt(-3);
	const int mid = std::abs(damperAt(20)), out = damperAt(100);
	MESSAGE("no dead band: 3 -> " << base3 << ", 20 -> " << base20 << ", 100 -> " << full << "; dead band 3000: 3 -> " << in_p << ", -3 -> " << in_n << ", 20 -> " << mid << ", 100 -> " << out);
	CHECK(base3 > 20);
	CHECK(in_p == 0);
	CHECK(in_n == 0);
	CHECK(mid < base20);
	CHECK_NEAR(out, -full, 3);
}

// PID 5.3: CP Offset is the neutral value of the metric: a friction with CP Offset 10000 (metric; HD scale 247 per count/ms: neutral at
// 40 counts/ms) acts on the speed relative to it. At 100 counts/ms (far above) the torque opposes the motion in full, at 20 counts/ms
// (below the neutral speed by a factor 2) the sign of (speed - offset) is negative and the torque has the opposite sign.
TEST_CASE_FIXTURE(CondFixture, "F1-E8: a friction measures the speed from CP Offset: below the neutral speed the torque reverses" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtFriction);
	setCondition(idx, 10000, 8000, 8000, 32767, 32767, 0);
	start(idx);
	const int fast = damperAt(100);
	const int slow = damperAt(20);
	MESSAGE("CP Offset 10000: 100 counts/ms -> " << fast << ", 20 counts/ms -> " << slow);
	CHECK(fast < 0);
	CHECK(slow > 0);
}

// characterization: the ramp shape (HD). rampupFactor = (1 + sin(pi (x - 0.5))) / 2 with x = |speed * 45 / (0.25 * 32767)| (EffectsCalculator.cpp:
// 395-405), speed in deg/s = counts/ms * 1000 * 360 / 65536 = 5.493 counts/ms. At 16 counts/ms: x = 16 * 5.493 * 45 / 8191.75 = 0.4828,
// factor 0.4724 of the full torque. Tolerance 0.01 (both torques > 2000 counts: 1.7 / 2300 per value; cpr 65535 vs 65536 changes x by 1.5e-5).
// INR 16
TEST_CASE_FIXTURE(CondFixture, "characterization: [F1-E8] friction at 16 counts/ms is the sine ramp factor of the full torque" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtFriction);
	setCondition(idx, 0, 8000, 8000, 32767, 32767, 0);
	start(idx);
	const int full = std::abs(damperAt(-100));
	const int s16 = std::abs(damperAt(16));
	REQUIRE(full > 0);
	const double x = 16 * 5.4932 * 45 / 8191.75;
	const double expected = (1.0 + std::sin(M_PI * (x - 0.5))) / 2.0;
	MESSAGE("ratio " << static_cast<double>(s16) / full << ", expected " << expected);
	CHECK_NEAR(static_cast<double>(s16) / full, expected, 0.01);
}

// Negative friction coefficient. Oracle: PID 5.3 gives the formula force = Coefficient * (q - ...) per side and says nothing about
// negative values, no friction formula. What defines the intended behaviour here: (1) the descriptor FFBWheel sends declares the
// coefficients signed (logical -32768..32767, usb_hid_2ffb_desc.c:298-309; asserted from the parsed descriptor below), (2) the firmware's own
// spring / damper / inertia paths treat the coefficient as signed (a negative one reverses the force: the plain spring case above) and
// the friction path of the same effect type group is documented to work like them. So a coefficient of -8000 gives the force of +8000 with
// the opposite sign (assisting the motion: positive torque for host-positive motion). The firmware reads the coefficient into a uint16_t
// (EffectsCalculator.cpp:408), -8000 becomes 57536, and the friction opposes the motion more strongly.
TEST_CASE_FIXTURE(CondFixture, "F1-E8: a negative friction coefficient reverses the force like the signed coefficient of the other conditions"
	* KNOWN_ISSUE("(IB 19) the friction coefficient is read into a uint16_t (EffectsCalculator.cpp:408), a negative coefficient becomes a large positive one") * IT_DELETES_FFBHIDMAIN) {
	up();
	const hid::Field *pc = h().descriptor().findField(ReportType::Output, pid::use::SetConditionReport, pid::use::PositiveCoefficient);
	REQUIRE(pc != nullptr);
	REQUIRE(pc->logicalMin < 0);
	const int64_t idx = newEffect(pid::use::EtFriction);
	setCondition(idx, 0, 8000, 8000, 10000, 10000, 0);
	start(idx);
	const int pos = damperAt(100);
	REQUIRE(pos < 0);
	setCondition(idx, 0, -8000, -8000, 10000, 10000, 0);
	const int neg = damperAt(100);
	MESSAGE("coefficient +8000: " << pos << ", -8000: " << neg);
	REQUIRE_CHAIN_MODELLED();
	CHECK(neg > 0);
	CHECK_NEAR(neg, -pos, 3);
}

// characterization: absolute scale of the friction above the ramp-up speed: force = coefficient 8000 * rampup 1 * sign, ((255 + 1) * 8000) >> 8 = 8000,
// scaler.friction 1.0 -> 8000, torque = 8000 * 20000 / 32767 = 4883 (negative for positive speed).
// INR 15
TEST_CASE_FIXTURE(CondFixture, "characterization: [F1-E8] friction coefficient 8000 above the ramp-up speed gives the hand-derived torque count" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtFriction);
	setCondition(idx, 0, 8000, 8000, 32767, 32767, 0);
	start(idx);
	const int t = damperAt(100);
	MESSAGE("torque " << t);
	CHECK_NEAR(t, -4883, 6);
}

} // TEST_SUITE
