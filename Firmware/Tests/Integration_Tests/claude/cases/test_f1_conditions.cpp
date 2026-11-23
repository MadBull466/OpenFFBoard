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

// characterization: which polar direction gives the full force (polar zero undefined, PID 1.01 5.1; reconciled by the lead). Firmware:
// axisMagnitudes[0] = sin(2 pi direction / 36000) (HidFFB.cpp:333): Direction 9000 gives share 1.0, the same restoring torque as the spring
// without a direction (+6000 counts). How the torque behaves over all directions (a rotated condition acts along a line) is F1-E9 (IB 34).
// INR 14
TEST_CASE_FIXTURE(CondFixture, "characterization: [F1-E5] polar direction 9000 gives the full restoring spring torque" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t ref = newEffect(pid::use::EtSpring);
	setCondition(ref, 0, 16000, 16000, 32767, 32767, 0);
	start(ref);
	const int t0 = springAt(6000);
	pid::PidReport eo = h().builder().make(ReportType::Output, pid::use::EffectOperationReport);
	eo.set(pid::use::EffectBlockIndex, ref).select(pid::use::OpEffectStop).set(pid::use::LoopCount, 1);
	h().send(eo);
	const int64_t a = newEffect(pid::use::EtSpring, 9000, true);
	setCondition(a, 0, 16000, 16000, 32767, 32767, 0);
	start(a);
	const int t9 = springAt(6000);
	MESSAGE("no direction " << t0 << ", direction 9000 " << t9);
	CHECK(t9 < 0);
	CHECK_NEAR(t9, t0, 2);
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

// =====================================================================================================================
// WO I1-6: F1-E9 geometry, F1-E12 combined effects, F1-E13 per-effect Gain, F1-E14 type change
//
// Oracles: PID 1.01 5.1 (Gain: "normalized scaling factor that is applied to all magnitudes and envelopes of an effect"; Direction Enable:
// "the Axes Enable flags are ignored, only one Condition Parameter Block is defined and the Direction is applied to the Condition Parameter
// Block as a polar direction"), 5.3 (last paragraphs: with one block per axis the first block applies to the first axis and the effect "must
// not be rotated"; with a single block for a multi-axis effect the Direction rotates it, and it acts along a line), Table 3. The polar zero is
// undefined (5.1); the line rule needs none (lead decision 2026-11-23, MATH at the F1-E9 cases); which direction gives the full force is INR 14.
// Settings: as the file comment (power 20000, fxratio 255, invert 0, ...). torqueScaler = 20000 / 32767 = 0.6103.
// Rounding bound for sums: every part has a truncation chain of at most 0.61 + 1 = 1.61 counts, the sum of four parts 6.4, the combined
// one 1.61: tolerance 9 counts (kSumTol).
// Float to int: nothing here converts out of range for a reachable setting (sums of at most three forces of 32767 stay far below int32; the
// largest float -> int32 conversion is the 32767-magnitude constant force times the sine share 1.0).
// =====================================================================================================================
namespace {

constexpr int kSumTol = 9;

struct MixFixture : CondFixture {
	int64_t createOnly(uint32_t type) {
		const FfbHost::CreateResult c = h().createEffect(type);
		REQUIRE(c.load.success());
		return c.load.effectBlockIndex;
	}
	void setEffectRaw(int64_t idx, uint32_t type, int64_t direction, bool directionEnable, int64_t gain = 255) {
		pid::PidReport se = h().builder().make(ReportType::Output, pid::use::SetEffectReport);
		se.set(pid::use::EffectBlockIndex, idx).select(type).set(pid::use::Duration, 32767).set(pid::use::Gain, gain);
		se.flag(pid::use::DirectionEnable, directionEnable);
		se.setIn({pid::use::Direction}, kDirectionX, direction);
		h().send(se);
	}
	void setConditionAt(int64_t idx, int64_t block, int64_t offset, int64_t pc, int64_t nc, int64_t ps, int64_t ns, int64_t db) {
		pid::PidReport cond = h().builder().make(ReportType::Output, pid::use::SetConditionReport);
		cond.set(pid::use::EffectBlockIndex, idx).set(pid::use::ParameterBlockOffset, block).set(pid::use::CpOffset, offset);
		cond.set(pid::use::PositiveCoefficient, pc).set(pid::use::NegativeCoefficient, nc);
		cond.set(pid::use::PositiveSaturation, ps).set(pid::use::NegativeSaturation, ns).set(pid::use::DeadBand, db);
		h().send(cond);
	}
	void stopFx(int64_t idx) {
		pid::PidReport eo = h().builder().make(ReportType::Output, pid::use::EffectOperationReport);
		eo.set(pid::use::EffectBlockIndex, idx).select(pid::use::OpEffectStop).set(pid::use::LoopCount, 1);
		h().send(eo);
	}
	void setConstant(int64_t idx, int64_t magnitude) {
		pid::PidReport sc = h().builder().make(ReportType::Output, pid::use::SetConstantForceReport);
		sc.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, magnitude);
		h().send(sc);
	}
	void setPeriodic(int64_t idx, int64_t magnitude, int64_t offset, int64_t phase, int64_t period) {
		pid::PidReport per = h().builder().make(ReportType::Output, pid::use::SetPeriodicReport);
		per.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, magnitude).set(pid::use::Offset, offset).set(pid::use::Phase, phase).set(pid::use::Period, period);
		h().send(per);
	}
	void setRamp(int64_t idx, int64_t startLevel, int64_t endLevel) {
		pid::PidReport r = h().builder().make(ReportType::Output, pid::use::SetRampForceReport);
		r.set(pid::use::EffectBlockIndex, idx).set(pid::use::RampStart, startLevel).set(pid::use::RampEnd, endLevel);
		h().send(r);
	}
	/** A running constant force (Direction Enable, 9000 = share 1.0 of the firmware's sin; torque = -magnitude * 0.6103) */
	int64_t startConstant(int64_t magnitude, int64_t gain = 255) {
		const int64_t idx = createOnly(pid::use::EtConstantForce);
		setEffectRaw(idx, pid::use::EtConstantForce, 9000, true, gain);
		setConstant(idx, magnitude);
		start(idx);
		return idx;
	}
	std::vector<TorqueTrace::Sample> record(uint32_t ms) {
		TorqueTrace trace(chip1);
		trace.start(*this);
		h().observe(&trace);
		h().run(ms);
		h().observe(nullptr);
		return trace.samples();
	}
	/** Runs `ms`, REQUIREs the last `last` cycles within `spread`, returns the last torque (no clip limit: saturation cases) */
	int steady(uint32_t ms, size_t last = 30, int spread = 2) {
		const std::vector<TorqueTrace::Sample> s = record(ms);
		REQUIRE(s.size() >= last);
		int lo = 100000, hi = -100000;
		for (size_t i = s.size() - last; i < s.size(); i++) {
			lo = std::min<int>(lo, s[i].torque);
			hi = std::max<int>(hi, s[i].torque);
		}
		INFO("window min " << lo << " max " << hi);
		REQUIRE(hi - lo <= spread);
		return s.back().torque;
	}
	int maxAbs(uint32_t ms) {
		const std::vector<TorqueTrace::Sample> s = record(ms);
		REQUIRE(s.size() > 100);
		int m = 0;
		for (const auto &x : s) {
			m = std::max<int>(m, std::abs(x.torque));
		}
		return m;
	}
};

} // namespace

TEST_SUITE("F1ConditionsGeometry") {

// ---------------------------------------------------------------------------------------------------------------------
// F1-E9 geometry
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.3 (last paragraphs): with one block per axis the first block (Parameter Block Offset 0) applies to the first axis. FFBWheel has one
// axis: a second block (offset 1 = second axis) changes nothing at 0x64, whatever its coefficient.
TEST_CASE_FIXTURE(MixFixture, "F1-E9: a second condition block does not change the torque of axis 0" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtSpring);
	setConditionAt(idx, 0, 0, 16000, 16000, 32767, 32767, 0);
	start(idx);
	const int before = springAt(6000);
	const int beforeNeg = springAt(-6000);
	setConditionAt(idx, 1, 0, 32767, 32767, 32767, 32767, 0);
	const int after = springAt(6000);
	setConditionAt(idx, 1, 3000, 1000, 5000, 20000, 20000, 100);
	const int after2 = springAt(6000);
	const int afterNeg = springAt(-6000);
	MESSAGE("block 0 only " << before << " / " << beforeNeg << ", with block 1 " << after << " / " << after2 << " / " << afterNeg);
	REQUIRE(before != 0);
	CHECK_NEAR(after, before, 1);
	CHECK_NEAR(after2, before, 1);
	CHECK_NEAR(afterNeg, beforeNeg, 1);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.3 / Table 3: Condition (Y Axis) is the second block; an effect that has only that block has nothing for the X axis: torque 0.
TEST_CASE_FIXTURE(MixFixture, "F1-E9: an effect with only the condition block of the second axis gives axis 0 no torque" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtSpring);
	setConditionAt(idx, 1, 0, 32767, 32767, 32767, 32767, 0);
	start(idx);
	const int t = springAt(6000);
	const int tn = springAt(-6000);
	MESSAGE("only block 1: " << t << " / " << tn);
	CHECK(std::abs(t) <= 1);
	CHECK(std::abs(tn) <= 1);
	REQUIRE_CHAIN_MODELLED();
}

// Rotated condition (lead decision 2026-11-23; PID 1.01 5.3, last paragraphs): a single condition block with a direction acts along a LINE (5.3:
// a friction rotated 45 degrees resists motion along north-east / south-west and has no effect on motion along north-west / south-east).
// MATH: the condition acts along the unit vector u = (cos a, sin a), a = the angle between u and axis 0 (a function of the polar direction
// theta whose zero PID does not define). A displacement m0 along axis 0 has the component m_u = m0 cos a along u, the 1-D condition gives the
// force F = k m_u along u, and axis 0 receives F cos a = k m0 cos^2 a. Hence (a) theta and theta + 18000 (u and -u, the same line) give the same
// torque; (b) the share cos^2 a >= 0: a spring with positive coefficients restores or is zero at every direction, a friction / damper opposes the
// motion or is zero; (c) 30 and 60 degrees: with the polar zero along or across axis 0 (a = theta or 90 deg - theta) the 60 : 30 torque ratio is
// cos^2 60 / cos^2 30 = 1/3 or sin^2 60 / sin^2 30 = 3; "3 or 1/3" needs no choice of the zero. The firmware scales linearly by sin(theta)
// (HidFFB.cpp:333): the torque reverses over 180 degrees, 27000 is an anti-spring, the 60 : 30 ratio is 1.73.
// Spring: +6000 counts (axis.0.invert 0: host-positive), restoring = torque <= 0 (F1-E5). Directions are changed by Set Effect on the running
// effect (the polar share is written by Set Effect, HidFFB.cpp:333). First failing assertion: (a) at 30 / 210 degrees (-1430 / +1430).
TEST_CASE_FIXTURE(MixFixture, "F1-E9: a rotated spring acts along a line: equal torque at theta and theta plus 180 degrees, restoring at every direction, 60 to 30 degree ratio 3 or 1/3"
	* KNOWN_ISSUE("(IB 34) a rotated condition is scaled linearly by sin(theta) (HidFFB.cpp:333): the torque reverses over 180 degrees, an anti-spring at 27000") * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtSpring, 3000, true);
	setConditionAt(idx, 0, 0, 16000, 16000, 32767, 32767, 0);
	start(idx);
	auto at = [&](int64_t dir) {
		setEffectRaw(idx, pid::use::EtSpring, dir, true);
		return springAt(6000);
	};
	const int t30 = springAt(6000);
	const int t210 = at(21000), t60 = at(6000), t240 = at(24000);
	const int t0 = at(0), t90 = at(9000), t180 = at(18000), t270 = at(27000);
	MESSAGE("30 " << t30 << ", 210 " << t210 << ", 60 " << t60 << ", 240 " << t240 << "; 0 " << t0 << ", 90 " << t90 << ", 180 " << t180 << ", 270 " << t270);
	REQUIRE(t30 != 0);
	REQUIRE(t60 != 0);
	REQUIRE_CHAIN_MODELLED();
	CHECK_NEAR(t210, t30, 4);
	CHECK_NEAR(t240, t60, 4);
	CHECK(t0 <= 0);
	CHECK(t90 <= 0);
	CHECK(t180 <= 0);
	CHECK(t270 <= 0);
	const double r = static_cast<double>(t60) / t30;
	CHECK((std::fabs(r - 3.0) < 0.06 || std::fabs(r - 1.0 / 3.0) < 0.01));
}

// Same line rule for a friction (MATH above, (b)): it opposes the motion or is zero at every direction. Motion +100 counts per cycle (far above the
// ramp-up speed, F1-E8): opposing = torque <= 0. 9000 and 4500 come first (the firmware's sin > 0: opposing), the first failing assertion is
// 27000, where the firmware's friction pushes along the motion (+4882 counts: it adds energy).
TEST_CASE_FIXTURE(MixFixture, "F1-E9: a rotated friction opposes the motion or is zero at every direction"
	* KNOWN_ISSUE("(IB 34) a rotated condition is scaled linearly by sin(theta) (HidFFB.cpp:333): at 27000 the friction pushes along the motion") * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtFriction, 9000, true);
	setConditionAt(idx, 0, 0, 8000, 8000, 32767, 32767, 0);
	start(idx);
	const int t90 = damperAt(100);
	setEffectRaw(idx, pid::use::EtFriction, 4500, true);
	const int t45 = damperAt(100);
	setEffectRaw(idx, pid::use::EtFriction, 27000, true);
	const int t270 = damperAt(100);
	setEffectRaw(idx, pid::use::EtFriction, 22500, true);
	const int t225 = damperAt(100);
	MESSAGE("+100 counts/cycle: 9000 " << t90 << ", 4500 " << t45 << ", 27000 " << t270 << ", 22500 " << t225);
	REQUIRE(t90 < 0);
	CHECK(t45 <= 0);
	REQUIRE_CHAIN_MODELLED();
	CHECK(t270 <= 0);
	CHECK(t225 <= 0);
}

// PID 5.1 / 5.3: Direction Enable off, Axes Enable X on (what a host sends for an unrotated condition), one condition block: one block per enabled
// axis, "the effect must not be rotated": the block acts on axis 0 unrotated, a spring with positive coefficients restores on both sides.
// Firmware: axisMagnitudes[0] = (directionX - 18000) / 18000 = -1 for direction 0 (HidFFB.cpp:333; useSingleCondition defaults to true,
// ffb_defs.h:359, so the condition override at HidFFB.cpp:317-327 is skipped; the promotion at :391 needs a zero magnitude): a full anti-spring.
TEST_CASE_FIXTURE(MixFixture, "F1-E9: one condition block with Axes Enable X and Direction Enable off acts on axis 0 as an unrotated restoring spring"
	* KNOWN_ISSUE("(IB 35) with Axes Enable X and Direction Enable off the axis magnitude is (direction - 18000) / 18000 = -1 for direction 0 (HidFFB.cpp:333): an anti-spring") * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = createOnly(pid::use::EtSpring);
	pid::PidReport se = h().builder().make(ReportType::Output, pid::use::SetEffectReport);
	se.set(pid::use::EffectBlockIndex, idx).select(pid::use::EtSpring).set(pid::use::Duration, 32767).set(pid::use::Gain, 255);
	se.flag(pid::use::DirectionEnable, false);
	se.setIn({pid::use::AxesEnable}, hid::usage(0x01, 0x30), 1);
	se.setIn({pid::use::Direction}, kDirectionX, 0);
	h().send(se);
	setConditionAt(idx, 0, 0, 16000, 16000, 32767, 32767, 0);
	start(idx);
	const int t = springAt(6000);
	const int tn = springAt(-6000);
	MESSAGE("Axes Enable X, direction 0: +6000 counts " << t << ", -6000 counts " << tn);
	REQUIRE(t != 0);
	REQUIRE_CHAIN_MODELLED();
	CHECK(t < 0);
	CHECK(tn > 0);
	CHECK_NEAR(t, -tn, 4);
}

// PID 5.1 / 5.3 do not make the result depend on the order of the reports. Set Effect (Direction Enable off, no Axes Enable X) before Set
// Condition: the firmware promotes the axis magnitude 0 -> 1.0 in Set Condition (HidFFB.cpp:391-393). Set Condition before Set Effect: Set
// Effect writes 0 (HidFFB.cpp:333-334) and nothing promotes it again: torque 0. Neither value is called the defect here: with Direction Enable
// and Axes Enable X off, PID 5.1 gives axis 0 no share (0); the full force comes from the firmware's 0 -> 1.0 promotion in set_condition
// (HidFFB.cpp:391-393). Which value is intended is INR 30; the defect asserted is that the torque depends on the order of the reports (IB 33).
TEST_CASE_FIXTURE(MixFixture, "F1-E9: the order of Set Condition and Set Effect does not change the torque"
	* KNOWN_ISSUE("(IB 33) the torque depends on the order of the reports and on repeating an identical Set Effect (HidFFB.cpp:333 writes 0, only set_condition promotes 0 -> 1.0, :391-393)") * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t a = newEffect(pid::use::EtSpring);
	setConditionAt(a, 0, 0, 16000, 16000, 32767, 32767, 0);
	start(a);
	const int tA = springAt(6000);
	stopFx(a);
	const int64_t b = createOnly(pid::use::EtSpring);
	setConditionAt(b, 0, 0, 16000, 16000, 32767, 32767, 0);
	setEffectRaw(b, pid::use::EtSpring, 0, false);
	start(b);
	const int tB = springAt(6000);
	MESSAGE("effect first: " << tA << ", condition first: " << tB);
	REQUIRE(tA != 0);
	REQUIRE_CHAIN_MODELLED();
	CHECK_NEAR(tB, tA, 1);
}

// PID 5.1: Set Effect may be sent again while the effect exists (parameters unchanged here): the effect keeps acting. Direction Enable off, no
// Axes Enable X, Set Condition already received: the firmware writes the axis magnitude 0 again (HidFFB.cpp:333) and the running condition
// effect goes silent: the torque depends on repeating an identical report
// (IB 33; which value is intended: INR 30).
TEST_CASE_FIXTURE(MixFixture, "F1-E9: a repeated Set Effect with unchanged parameters does not silence a running condition effect"
	* KNOWN_ISSUE("(IB 33) the torque depends on the order of the reports and on repeating an identical Set Effect (HidFFB.cpp:333 writes 0, only set_condition promotes 0 -> 1.0, :391-393)") * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t a = newEffect(pid::use::EtSpring);
	setConditionAt(a, 0, 0, 16000, 16000, 32767, 32767, 0);
	start(a);
	const int before = springAt(6000);
	setEffectRaw(a, pid::use::EtSpring, 0, false);
	const int after = springAt(6000);
	MESSAGE("before the repeat " << before << ", after " << after);
	REQUIRE(before != 0);
	REQUIRE_CHAIN_MODELLED();
	CHECK_NEAR(after, before, 1);
}

// Same with Direction Enable: the polar share is written by Set Effect in both orders, so the order does not matter (plain).
TEST_CASE_FIXTURE(MixFixture, "F1-E9: with Direction Enable the order of Set Condition and Set Effect does not change the torque" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t a = newEffect(pid::use::EtSpring, 9000, true);
	setConditionAt(a, 0, 0, 16000, 16000, 32767, 32767, 0);
	start(a);
	const int tA = springAt(6000);
	stopFx(a);
	const int64_t b = createOnly(pid::use::EtSpring);
	setConditionAt(b, 0, 0, 16000, 16000, 32767, 32767, 0);
	setEffectRaw(b, pid::use::EtSpring, 9000, true);
	start(b);
	const int tB = springAt(6000);
	MESSAGE("effect first: " << tA << ", condition first: " << tB);
	REQUIRE(tA != 0);
	CHECK_NEAR(tB, tA, 1);
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-E12 combined effects
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.1: the effects of a device are summed. Position -20000 counts (q -2000) moving at -4 counts per FFB cycle (cycle-keyed script), a
// constant force (-6000), a spring (coefficient 2000) and a damper (coefficient 5000), all with positive torque here, and a constant force of
// +2500 with negative torque (a sum that lost a sign would fail). Each alone, then
// together: per FFB cycle, the combined torque equals the sum of the three single ones (the position sequence relative to the ramp start is
// identical in every run: cycle index keyed). The cycles 400-480 after the ramp start are compared (damper filter settled: tau 10.6 ms).
TEST_CASE_FIXTURE(MixFixture, "F1-E12: a constant force, a spring and a damper together give the sum of their single torques before the clip" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t c = startConstant(-6000);
	const int64_t sp = newEffect(pid::use::EtSpring);
	setCondition(sp, 0, 2000, 2000, 32767, 32767, 0);
	const int64_t d = newEffect(pid::use::EtDamper);
	setCondition(d, 0, 5000, 5000, 32767, 32767, 0);
	const int64_t o = startConstant(2500);
	const std::vector<int64_t> all = {c, sp, d, o};
	auto runWith = [&](const std::vector<int64_t> &on) {
		for (int64_t i : all) {
			stopFx(i);
		}
		for (int64_t i : on) {
			start(i);
		}
		script.setConstant(-20000);
		h().run(1000);
		const uint32_t c0 = ffbCycles();
		rampFrom(-4);
		std::vector<int> byCycle(500, 0);
		for (const TorqueTrace::Sample &s : record(500)) {
			const int64_t rel = static_cast<int64_t>(s.cycle) - c0;
			if (rel >= 0 && rel < 500) {
				byCycle[rel] = s.torque;
			}
		}
		return byCycle;
	};
	const std::vector<int> tc = runWith({c}), ts = runWith({sp}), td = runWith({d}), to = runWith({o}), ta = runWith(all);
	MESSAGE("constant " << tc[450] << ", spring " << ts[450] << ", damper " << td[450] << ", opposite constant " << to[450] << ", all " << ta[450]);
	CHECK(tc[450] > 1000);
	CHECK(ts[450] > 500);
	CHECK(td[450] > 100);
	CHECK(to[450] < -1000);
	int worst = 0;
	for (int i = 400; i < 480; i++) {
		worst = std::max(worst, std::abs(ta[i] - (tc[i] + ts[i] + td[i] + to[i])));
	}
	MESSAGE("largest |all - sum of parts| " << worst);
	CHECK(worst <= kSumTol);
	CHECK(ta[450] < 19000);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.1 sums the effects; the device cannot output more than its power: 0x64 saturates at +-power. Constant force (-20000) alone gives
// 20000 * 20000 / 32767 = 12207 (within 3); two give 24414 > 20000: +power exactly; a third with the opposite sign (+5000: sum 35000 -> 21362)
// is still beyond: +power, sign of the sum. The mirrored sum gives -power.
TEST_CASE_FIXTURE(MixFixture, "F1-E12: a sum beyond power saturates at power with the sign of the sum" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t a = startConstant(-20000);
	const int one = steady(300);
	const int64_t b = startConstant(-20000);
	const int two = steady(300);
	const int64_t c = startConstant(5000);
	const int three = steady(300);
	stopFx(a);
	stopFx(b);
	stopFx(c);
	h().run(100);
	const int64_t m1 = startConstant(20000);
	const int64_t m2 = startConstant(20000);
	const int neg = steady(300);
	MESSAGE("one " << one << ", two " << two << ", three " << three << ", mirrored " << neg);
	CHECK_NEAR(one, 12207, 3);
	CHECK(two == kPower);
	CHECK(three == kPower);
	CHECK(neg == -kPower);
	(void)m1;
	(void)m2;
	REQUIRE_CHAIN_MODELLED();
}

// The int32 -> int16 narrowing before turn() never wraps for settings reachable by command with power <= 0x7fff (power > 0x7fff is IB 14, same
// as bug 32, covered by I1-7): the largest power 32767 (torqueScaler 1.0), three constant forces of the extreme magnitudes (sum +-98301) give
// exactly +-32767 in every cycle after the change, never a wrapped (opposite sign) value.
TEST_CASE_FIXTURE(MixFixture, "F1-E12: the largest sums at the largest power give plus or minus 32767 and never wrap" * IT_DELETES_FFBHIDMAIN) {
	up();
	h().command("axis.0.power=32767");
	REQUIRE(h().getValue("axis.0.power?") == 32767);
	h().run(50);
	const int64_t a = startConstant(-32767);
	const int64_t b = startConstant(-32767);
	const int64_t c = startConstant(-32767);
	h().run(100);
	const std::vector<TorqueTrace::Sample> s = record(300);
	REQUIRE(s.size() > 100);
	int lo = 100000, hi = -100000;
	for (const auto &x : s) {
		lo = std::min<int>(lo, x.torque);
		hi = std::max<int>(hi, x.torque);
	}
	stopFx(a);
	stopFx(b);
	stopFx(c);
	h().run(100);
	const int64_t d = startConstant(32767);
	const int64_t e = startConstant(32767);
	const int64_t f = startConstant(32767);
	h().run(100);
	const std::vector<TorqueTrace::Sample> s2 = record(300);
	int lo2 = 100000, hi2 = -100000;
	for (const auto &x : s2) {
		lo2 = std::min<int>(lo2, x.torque);
		hi2 = std::max<int>(hi2, x.torque);
	}
	MESSAGE("positive sums: " << lo << ".." << hi << ", negative sums: " << lo2 << ".." << hi2);
	CHECK(lo == 32767);
	CHECK(hi == 32767);
	CHECK(lo2 == -32767);
	CHECK(hi2 == -32767);
	(void)d;
	(void)e;
	(void)f;
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-E13 per-effect Gain
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.1: Gain is a normalized scaling factor applied to all magnitudes of an effect. Square (one waveform) magnitude 16000, period 100:
// the peak |torque| over 300 ms with Gain 128 is 128 / 255 of the one with Gain 255 (both > 4900 counts: ratio tolerance 0.01).
TEST_CASE_FIXTURE(MixFixture, "F1-E13: the effect Gain scales a periodic effect" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = createOnly(pid::use::EtSquare);
	setEffectRaw(idx, pid::use::EtSquare, 9000, true, 255);
	setPeriodic(idx, 16000, 0, 0, 100);
	start(idx);
	h().run(100);
	const int full = maxAbs(300);
	setEffectRaw(idx, pid::use::EtSquare, 9000, true, 128);
	h().run(20);
	const int half = maxAbs(300);
	MESSAGE("gain 255: " << full << ", gain 128: " << half);
	REQUIRE(full > 4000);
	CHECK_NEAR(static_cast<double>(half) / full, 128.0 / 255.0, 0.01);
}

// PID 5.1: same for a ramp (Ramp Start = Ramp End = 16000 keeps it constant: the firmware multiplies every non-condition force by gain / 255,
// EffectsCalculator.cpp:314; the I1-3 mutation note that
// ramp Gain is ignored is refuted).
TEST_CASE_FIXTURE(MixFixture, "F1-E13: the effect Gain scales a ramp" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = createOnly(pid::use::EtRamp);
	setEffectRaw(idx, pid::use::EtRamp, 9000, true, 255);
	setRamp(idx, 16000, 16000);
	start(idx);
	h().run(100);
	const int full = steady(200);
	setEffectRaw(idx, pid::use::EtRamp, 9000, true, 128);
	const int half = steady(200);
	MESSAGE("gain 255: " << full << ", gain 128: " << half);
	REQUIRE(std::abs(full) > 4000);
	CHECK_NEAR(static_cast<double>(half) / full, 128.0 / 255.0, 0.01);
}

// characterization: PID 5.1 applies the Gain "to all magnitudes and envelopes of an effect"; condition blocks have neither (5.1.1.1 rule 2:
// effects that use Condition Blocks do not use Envelope Blocks; 5.3 has coefficients and saturations), and Table 3 says nothing about the Gain,
// so PID does not fix whether the Gain scales a condition. The firmware ignores it (seam S9 confirmed): calcNonConditionEffectForce returns 0
// for the condition types (EffectsCalculator.cpp:180-185) before its Gain line (:314), and the condition path never reads effect->gain.
// Gain 128 gives the torque of Gain 255 for spring, damper, friction and inertia (ratio 1.0).
// INR 31
TEST_CASE_FIXTURE(MixFixture, "characterization: [F1-E13] condition effects ignore the Effect Gain" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t sp = newEffect(pid::use::EtSpring, 9000, true);
	setCondition(sp, 0, 16000, 16000, 32767, 32767, 0);
	start(sp);
	const int sp255 = springAt(6000);
	setEffectRaw(sp, pid::use::EtSpring, 9000, true, 128);
	const int sp128 = springAt(6000);
	MESSAGE("spring: " << sp255 << " -> " << sp128);
	REQUIRE(sp255 != 0);
	CHECK_NEAR(static_cast<double>(sp128) / sp255, 1.0, 0.01);
	stopFx(sp);
	const int64_t dp = newEffect(pid::use::EtDamper, 9000, true);
	setCondition(dp, 0, 32767, 32767, 32767, 32767, 0);
	start(dp);
	script.setConstant(0);
	h().run(100);
	const int dp255 = damperAt(4);
	setEffectRaw(dp, pid::use::EtDamper, 9000, true, 128);
	const int dp128 = damperAt(4);
	MESSAGE("damper: " << dp255 << " -> " << dp128);
	REQUIRE(dp255 != 0);
	CHECK_NEAR(static_cast<double>(dp128) / dp255, 1.0, 0.01);
	stopFx(dp);
	settleHold();
	const int64_t fr = newEffect(pid::use::EtFriction, 9000, true);
	setCondition(fr, 0, 8000, 8000, 32767, 32767, 0);
	start(fr);
	const int fr255 = damperAt(100);
	setEffectRaw(fr, pid::use::EtFriction, 9000, true, 128);
	const int fr128 = damperAt(100);
	MESSAGE("friction: " << fr255 << " -> " << fr128);
	REQUIRE(fr255 != 0);
	CHECK_NEAR(static_cast<double>(fr128) / fr255, 1.0, 0.01);
	stopFx(fr);
	settleHold();
	const int64_t in = newEffect(pid::use::EtInertia, 9000, true);
	setCondition(in, 0, 1000, 1000, 32767, 32767, 0);
	start(in);
	const int in255 = parabolaAt(1);
	settleHold();
	setEffectRaw(in, pid::use::EtInertia, 9000, true, 128);
	const int in128 = parabolaAt(1);
	MESSAGE("inertia: " << in255 << " -> " << in128);
	REQUIRE(in255 != 0);
	CHECK_NEAR(static_cast<double>(in128) / in255, 1.0, 0.01);
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-E14 type change
// ---------------------------------------------------------------------------------------------------------------------
// Type change on a RUNNING effect to damper, friction or inertia from a type without filters (spring, sine, ramp, ...): HidFFB::set_effect calls
// set_filters before it writes the new type (HidFFB.cpp:301-307, bug 11: the filters of the OLD type, none), and Effect Operation Start sets the
// filters only for an effect that is not running (:423-425), so the next cycle calls process() on a null filter (EffectsCalculator.cpp:416 /
// :425 / :433) and the host process dies: the skipped case below (IB 36; same as bug 12). Only running effects are affected: a STOPPED effect gets
// the filters of its new type at the next Start (the plain cases below).

// A stopped spring changed by Set Effect to a damper (Set Condition for the damper, then Start): the damper opposes the motion and gives the
// torque of a damper created as such with the same parameters (same chain: equal within 3 counts, the measurement spread).
TEST_CASE_FIXTURE(MixFixture, "F1-E14: a stopped spring changed to a damper and started damps the motion like a new damper" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtSpring, 9000, true);
	setConditionAt(idx, 0, 0, 16000, 16000, 32767, 32767, 0);
	start(idx);
	h().run(100);
	stopFx(idx);
	setEffectRaw(idx, pid::use::EtDamper, 9000, true);
	setConditionAt(idx, 0, 0, 32767, 32767, 32767, 32767, 0);
	start(idx);
	const int changed = damperAt(4);
	stopFx(idx);
	settleHold();
	const int64_t fresh = newEffect(pid::use::EtDamper, 9000, true);
	setConditionAt(fresh, 0, 0, 32767, 32767, 32767, 32767, 0);
	start(fresh);
	const int ref = damperAt(4);
	MESSAGE("changed " << changed << ", new damper " << ref);
	CHECK(changed < 0);
	CHECK_NEAR(changed, ref, 3);
}

// A stopped sine changed to an inertia and started: opposes the acceleration like a new inertia with the same parameters.
TEST_CASE_FIXTURE(MixFixture, "F1-E14: a stopped sine changed to an inertia and started opposes the acceleration like a new inertia" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = createOnly(pid::use::EtSine);
	setEffectRaw(idx, pid::use::EtSine, 9000, true);
	setPeriodic(idx, 4000, 0, 0, 100);
	start(idx);
	h().run(200);
	stopFx(idx);
	setEffectRaw(idx, pid::use::EtInertia, 9000, true);
	setConditionAt(idx, 0, 0, 1000, 1000, 32767, 32767, 0);
	start(idx);
	const int changed = parabolaAt(1);
	stopFx(idx);
	settleHold();
	const int64_t fresh = newEffect(pid::use::EtInertia, 9000, true);
	setConditionAt(fresh, 0, 0, 1000, 1000, 32767, 32767, 0);
	start(fresh);
	const int ref = parabolaAt(1);
	MESSAGE("changed " << changed << ", new inertia " << ref);
	CHECK(changed < 0);
	CHECK_NEAR(changed, ref, 3);
}

// A stopped ramp changed to a friction and started: opposes the motion like a new friction with the same parameters.
TEST_CASE_FIXTURE(MixFixture, "F1-E14: a stopped ramp changed to a friction and started opposes the motion like a new friction" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = createOnly(pid::use::EtRamp);
	setEffectRaw(idx, pid::use::EtRamp, 9000, true);
	setRamp(idx, 8000, 8000);
	start(idx);
	h().run(200);
	stopFx(idx);
	setEffectRaw(idx, pid::use::EtFriction, 9000, true);
	setConditionAt(idx, 0, 0, 8000, 8000, 32767, 32767, 0);
	start(idx);
	const int changed = damperAt(100);
	stopFx(idx);
	settleHold();
	const int64_t fresh = newEffect(pid::use::EtFriction, 9000, true);
	setConditionAt(fresh, 0, 0, 8000, 8000, 32767, 32767, 0);
	start(fresh);
	const int ref = damperAt(100);
	MESSAGE("changed " << changed << ", new friction " << ref);
	CHECK(changed < 0);
	CHECK_NEAR(changed, ref, 3);
}

// Intended behaviour (PID 5.1: Set Effect changes the parameters of an existing effect, here its type): a running spring changed to a damper
// damps the motion. Not run: the running effect keeps null filters (comment above) and the first cycle after the change calls process() on a
// null Biquad (EffectsCalculator.cpp:425), which kills the test process (host SIGSEGV; on the target the vector table is read as a Biquad, bug 12).
TEST_CASE_FIXTURE(MixFixture, "F1-E14: a running spring changed to a damper by Set Effect damps the motion" * doctest::skip()
	* doctest::description("KNOWN ISSUE (crash): (IB 36; same as bug 12) a running effect changed to a damper keeps null filters (HidFFB.cpp:301-307, :423-425), EffectsCalculator.cpp:425 calls process() on a null Biquad. Skipped because it terminates the test run")) {
	up();
	const int64_t idx = newEffect(pid::use::EtSpring, 9000, true);
	setConditionAt(idx, 0, 0, 16000, 16000, 32767, 32767, 0);
	start(idx);
	h().run(100);
	setEffectRaw(idx, pid::use::EtDamper, 9000, true);
	setConditionAt(idx, 0, 0, 32767, 32767, 32767, 32767, 0);
	const int t = damperAt(4);
	CHECK(t < 0);
}

// PID 5.1 / 5.9: a Set Effect changes the parameters of an existing effect; the effect keeps playing, its Duration counts from Op Effect Start.
// The firmware sets startTime = 0 on a type change (HidFFB.cpp:301-302) and ends a finite effect when tick - startTime > Duration
// (EffectsCalculator.cpp:138): after more than Duration ms of uptime the changed effect ends at once. Duration 32767 (the largest), uptime
// beyond 33.5 s first; a running constant force (-10000) changed to a sine (magnitude 4000, period 100 ms): in the 300 ms after the change the
// sine reaches both polarities beyond 2000 counts (bound 2441, see the constant -> sine case below). Firmware: 0 in every cycle.
TEST_CASE_FIXTURE(MixFixture, "F1-E14: a running effect changed to another type after more than Duration ms of uptime keeps playing"
	* KNOWN_ISSUE("(IB 37) a type change sets startTime = 0 (HidFFB.cpp:301-302), the effect ends at once when the uptime exceeds its Duration (EffectsCalculator.cpp:138)") * IT_DELETES_FFBHIDMAIN) {
	up();
	const uint64_t target = 33500000ULL;
	if (HostPlatform::nowUs() < target) {
		h().run(static_cast<uint32_t>((target - HostPlatform::nowUs()) / 1000 + 1));
	}
	REQUIRE(HostPlatform::nowUs() > 32767000ULL + 500000ULL);
	const int64_t idx = startConstant(-10000);
	const int before = steady(200);
	REQUIRE(before > 6000);
	setPeriodic(idx, 4000, 0, 0, 100);
	setEffectRaw(idx, pid::use::EtSine, 9000, true);
	const std::vector<TorqueTrace::Sample> s = record(300);
	REQUIRE(s.size() > 200);
	int lo = 100000, hi = -100000;
	for (const auto &x : s) {
		lo = std::min<int>(lo, x.torque);
		hi = std::max<int>(hi, x.torque);
	}
	MESSAGE("constant before " << before << ", after the change " << lo << ".." << hi);
	REQUIRE_CHAIN_MODELLED();
	CHECK(hi > 2000);
	CHECK(lo < -2000);
}

// A constant force changed by Set Effect to a sine (Set Periodic: magnitude 4000): bound of the new effect 4000 * 0.6103 = 2441 counts, +3 for
// the truncation chain; no cycle after the change beyond it, both polarities reached (the effect is the sine, not the old constant force).
TEST_CASE_FIXTURE(MixFixture, "F1-E14: an effect changed from a constant force to a sine stays within the bound of the sine in every cycle" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = startConstant(-10000);
	const int before = steady(200);
	setPeriodic(idx, 4000, 0, 0, 100);
	TorqueTrace trace(chip1);
	trace.start(*this);
	h().observe(&trace);
	setEffectRaw(idx, pid::use::EtSine, 9000, true);
	h().run(600);
	h().observe(nullptr);
	int lo = 100000, hi = -100000;
	for (const auto &s : trace.samples()) {
		lo = std::min<int>(lo, s.torque);
		hi = std::max<int>(hi, s.torque);
	}
	MESSAGE("constant -10000: " << before << ", after the change " << lo << ".." << hi);
	CHECK_NEAR(before, 6103, 3);
	CHECK(hi <= 2444);
	CHECK(lo >= -2444);
	CHECK(hi > 2000);
	CHECK(lo < -2000);
}

// A sine changed to a spring without coefficients gives 0 (bound 0 +- 2) in every cycle after the change (2 ms of margin for the report); a
// Set Condition then gives the restoring spring torque, never beyond its saturation 1500 * 0.6103 = 916 counts (+2).
TEST_CASE_FIXTURE(MixFixture, "F1-E14: an effect changed from a sine to a spring gives no torque until the coefficients arrive and stays within the saturation" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = createOnly(pid::use::EtSine);
	setEffectRaw(idx, pid::use::EtSine, 9000, true);
	setPeriodic(idx, 8000, 0, 0, 100);
	start(idx);
	h().run(300);
	script.setConstant(6000);
	TorqueTrace trace(chip1);
	trace.start(*this);
	h().observe(&trace);
	setEffectRaw(idx, pid::use::EtSpring, 9000, true);
	const uint64_t tChange = HostPlatform::nowUs();
	h().run(200);
	setConditionAt(idx, 0, 0, 16000, 16000, 1500, 1500, 0);
	const uint64_t tCond = HostPlatform::nowUs();
	h().run(300);
	h().observe(nullptr);
	int worstBefore = 0, worstAfter = 0;
	for (const auto &s : trace.samples()) {
		if (s.timeUs > tChange + 2000 && s.timeUs < tCond) {
			worstBefore = std::max(worstBefore, std::abs(static_cast<int>(s.torque)));
		}
		if (s.timeUs > tCond + 2000) {
			worstAfter = std::max(worstAfter, std::abs(static_cast<int>(s.torque)));
		}
	}
	const int last = trace.samples().back().torque;
	MESSAGE("spring without coefficients: worst " << worstBefore << ", with: worst " << worstAfter << ", last " << last);
	CHECK(worstBefore <= 2);
	CHECK(worstAfter <= 918);
	CHECK(last < 0);
	CHECK(last >= -918);
}

// A damper changed to a friction (both have filters, so the filter state is valid): saturation 1000 bounds both: 1000 * 0.6103 = 610 counts
// (+2) in every cycle, before and after the change, while the position moves at 4 counts per cycle; the torque opposes the motion afterwards.
// Bug 11 cross-reference: set_filters runs before the new type is written (HidFFB.cpp:301-307), so the friction keeps the damper's filter
// parameters (30 Hz / Q 0.4 instead of 50 Hz / Q 0.2, EffectsCalculator.h:61-64); the saturation bound and the sign cannot see that.
TEST_CASE_FIXTURE(MixFixture, "F1-E14: an effect changed from a damper to a friction stays within the saturation in every cycle and opposes the motion" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = newEffect(pid::use::EtDamper, 9000, true);
	setCondition(idx, 0, 32767, 32767, 1000, 1000, 0);
	start(idx);
	rampFrom(4);
	h().run(300);
	TorqueTrace trace(chip1);
	trace.start(*this);
	h().observe(&trace);
	setEffectRaw(idx, pid::use::EtFriction, 9000, true);
	h().run(500);
	h().observe(nullptr);
	int worst = 0;
	for (const auto &s : trace.samples()) {
		worst = std::max(worst, std::abs(static_cast<int>(s.torque)));
	}
	const int last = trace.samples().back().torque;
	MESSAGE("worst " << worst << ", last " << last);
	CHECK(worst <= 612);
	CHECK(last < 0);
	CHECK(last >= -612);
}

} // TEST_SUITE F1ConditionsGeometry
