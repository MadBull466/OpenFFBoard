/*
 * F1 effect-to-torque, part 1 (WO I1-3): F1-E1 constant force, F1-E3 ramp, F1-E4 envelope.
 *
 * Chain: PID output reports over the real USB stack (FfbHost / PidReportBuilder, descriptor read over USB) -> HidFFB ->
 * EffectsCalculator -> Axis -> TMC4671 driver -> register 0x64 of the fake chip (DS 0x64: flux [15:0], torque [31:16], s16 each).
 * Position: the fake chip's 0x6B stays at its power-up value 0 and nothing scripts it (no PositionScript, no plant), so the axis
 * sees position 0 and speed 0: the idle spring, the endstops and every condition add nothing. Only the effect under test is created.
 *
 * Oracle: USB_PID1_01.pdf. 4.1 (normalised force), 5.1 (Set Effect: Duration in ms, Gain "normalized scaling factor that is applied to
 * all magnitudes and envelopes of an effect", Direction with Direction Enable "polar direction", Start Delay "time between an Op Effect
 * Start ... and initiation of the effect"), 5.2 (envelope: Attack Level "Normalized amplitude for the start of the envelope", Attack Time
 * "transition time to reach the sustain level", Fade Level "Normalized amplitude to end the envelope", Fade Time "the fade time to reach
 * the fade level"; the text does NOT say the transition is linear, so the envelope cases assert end points, monotonicity and holds only),
 * 5.6 (constant force: Magnitude), 5.7 (ramp: Ramp Start / Ramp End "Normalized magnitude at the start / end of the effect"; "Duration
 * for a ramp force effect cannot be INFINITE"), 5.10 (Device Gain "normalized scaling factor applied to all magnitudes and envelopes of
 * an effect, scaling the final output of the device"). Descriptor ranges read over USB (see the first case): Magnitude -32767..32767,
 * Ramp Start / End -32768..32767, Attack / Fade Level 0..32767 (same scale as Magnitude), Gain and Device Gain 0..255 (physical 0..10000),
 * Duration / Attack Time / Fade Time / Start Delay in ms (unit exponent -3).
 *
 * Settings: every torque case sets `axis.0.power` and `axis.0.fxratio` first (IB 7 / bug 23: the scalers stay 0 until set by command),
 * and states the others it relies on. Defaults read back (fresh flash): axis.0.invert 1, axis.0.slewrate 65535, fx.0.filterCfFreq 500,
 * fx.0.filterCfQ 70, tmc.0.fluxoffset 0, tmc.0.invertForce 0 (read back in the cases).
 *
 * Scaling chain (HD, used for tolerances and the one absolute count), for a constant force of Magnitude M, effect Gain g, Device Gain G,
 * power P, fxratio r, Direction angle theta with Direction Enable:
 *   EffectsCalculator::calculateEffectForceOnAxis (EffectsCalculator.cpp:340-361, :442): v = (M * g) / 255 (int, :314, truncating),
 *     result = int(-v * sin(theta)) (float -> int32 truncation, the axis share, HidFFB.cpp:333), forceOnAxis = (result * G) / 255 (int);
 *   Axis::calculateFFBTorque (Axis.cpp:793-813): int(forceOnAxis * (1/32767) * 32767 * r/255) (float, truncation);
 *   Axis::updateTorque (:846-905): int(t * P/32767) (torqueScaler, :863), then x -1 if axis.0.invert (:884), slew limiter, clip +-P;
 *   TMC4671::turn (TMC4671.cpp:1745-1771): torque = that (negated if tmc invertForce), flux = idleFlux - min(|torque|, fluxoffset).
 * Rounding bound used by the ratio cases: 5 truncations lose less than 1 count of their own stage each: the effect gain division (:314),
 * int(-v * sin) (:359), the device gain division (:442), the FFB float -> int (Axis.cpp:813) and the power scaling (:863). The four
 * before the final power scaling reach the output multiplied by s = P/32767 (<= 1). So a measured torque differs from the exact product by
 * less than 1 + 4 s counts (s = 0.153 at P 5000 -> 1.61), and a linear relation |a - k b| is allowed (1 + 4 s) (1 + |k|) counts (kBound
 * below). That is the "integer rounding bound" of every ratio assertion in this file.
 *
 * Sign (common rule 4): PID 1.01 5.1 defines no polar zero. Only properties that need none are plain tests: reversing the Direction by
 * 18000 (180 degrees) reverses the torque; toggling axis.0.invert reverses it. The absolute direction-to-sign mapping is one
 * two characterization cases (polar zero undefined, PID 1.01 5.1; reconciled by the lead; INR 10).
 *
 * Settling rule (common rule 5): the filter of a constant force is bypassed at the default setting (S5 case below: filterCfFreq 500 at
 * 1000 Hz gives Fc = 0.5 and the firmware applies the filter only if Fc < 0.5), the slew limit axis.0.slewrate = 65535 counts per ms is
 * above any step that fits the clip (|step| <= 5000), and the fade-in of Axis::updateTorque is finished long before the first effect
 * (it runs 0.5 s / 1.0 s from the callers of startForceFadeIn, Axis.cpp:384, :449, :620, :640; the cases wait 2500 ms after setting power
 * and fxratio). The reconstruction filter of the effect magnitude (evaluateReconstructionFilter, EffectsCalculator.cpp:190-200, 636)
 * is in mode 0 = None (fx.0.reconFilterMode read back = 0 by configure()): the constant force's magnitude passes unchanged. So the
 * transient is 0 FFB cycles and the steady value is asserted over 20 cycles (settled(); the S5 characterization case shows the
 * first-cycle rule instead of assuming it). With a configured filter (S5 case) the settling time is derived from the filter there.
 * IB 1: the FFB period is 1001 us, so N cycles are not N ms where a case counts both. Device Gain default 255 (EffectsCalculator.h:235)
 * is relied on by every case that does not send a Device Gain report.
 * Limit for I1-4 (not tested here): Start Delay is ignored when Duration is infinite (EffectsCalculator.cpp:131-135 skips the delay check
 * for an infinite effect).
 */
#include "doctest.h"

#include "FfbHost.h"
#include "HidReportDescriptor.h"
#include "IntegrationFixture.h"
#include "PidReportBuilder.h"
#include "TestHelpers.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

using hid::ReportType;

namespace {

// Direction collection of Set Effect: first Ordinal value = polar angle with Direction Enable (INR 3, same as test_f1_hid.cpp)
constexpr uint32_t kDirectionX = hid::usage(0x0A, 1);

/** Allowed difference |a - k b| for two torques of the scaling chain at torqueScaler s, see the file comment */
double kBound(double k, double s) { return (1.0 + 4.0 * s) * (1.0 + std::fabs(k)); }

struct EffFixture : IntegrationFixture {
	std::optional<FfbHost> host;
	const int powerDefault = 5000;

	void up() {
		bringUp();
		setSpiFrameUs(0);
		host.emplace(*this);
	}
	FfbHost &h() { return *host; }
	double scalerOf(int power) const { return static_cast<double>(power) / 32767.0; }

	/** power and fxratio first (IB 7), invert and the others as the case needs; waits out the fade-in (see settling rule) */
	void configure(int power = 5000, int fxratio = 255) {
		h().command("axis.0.power=" + std::to_string(power));
		h().command("axis.0.fxratio=" + std::to_string(fxratio));
		CHECK(h().getValue("axis.0.slewrate?") == 65535); // default read back: above any step within the clip (settling rule)
		CHECK(h().getValue("fx.0.reconFilterMode?") == 0); // default read back: reconstruction filter None
		h().run(2500);
	}

	int64_t newEffect(uint32_t type) {
		const FfbHost::CreateResult c = h().createEffect(type);
		REQUIRE(c.load.success());
		return c.load.effectBlockIndex;
	}

	/** Set Effect: Direction Enable, polar direction (0.01 deg), Gain, Duration (ms), Start Delay (ms) */
	void setEffect(int64_t idx, uint32_t type, int64_t direction, int64_t gain = 255, int64_t duration = 32767, int64_t startDelay = 0) {
		pid::PidReport se = h().builder().make(ReportType::Output, pid::use::SetEffectReport);
		se.set(pid::use::EffectBlockIndex, idx).select(type).set(pid::use::Duration, duration).set(pid::use::Gain, gain);
		se.set(pid::use::StartDelay, startDelay);
		se.flag(pid::use::DirectionEnable, true);
		se.setIn({pid::use::Direction}, kDirectionX, direction);
		h().send(se);
	}
	void setConstant(int64_t idx, int64_t magnitude) {
		pid::PidReport sc = h().builder().make(ReportType::Output, pid::use::SetConstantForceReport);
		sc.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, magnitude);
		h().send(sc);
	}
	void setRamp(int64_t idx, int64_t start, int64_t end) {
		pid::PidReport r = h().builder().make(ReportType::Output, pid::use::SetRampForceReport);
		r.set(pid::use::EffectBlockIndex, idx).set(pid::use::RampStart, start).set(pid::use::RampEnd, end);
		h().send(r);
	}
	void setEnvelope(int64_t idx, int64_t attackLevel, int64_t attackTime, int64_t fadeLevel, int64_t fadeTime) {
		pid::PidReport e = h().builder().make(ReportType::Output, pid::use::SetEnvelopeReport);
		e.set(pid::use::EffectBlockIndex, idx).set(pid::use::AttackLevel, attackLevel).set(pid::use::AttackTime, attackTime);
		e.set(pid::use::FadeLevel, fadeLevel).set(pid::use::FadeTime, fadeTime);
		h().send(e);
	}
	void operation(int64_t idx, uint32_t op) {
		pid::PidReport eo = h().builder().make(ReportType::Output, pid::use::EffectOperationReport);
		eo.set(pid::use::EffectBlockIndex, idx).select(op).set(pid::use::LoopCount, 1);
		h().send(eo);
	}
	void setDeviceGain(int64_t g) {
		pid::PidReport r = h().builder().make(ReportType::Output, pid::use::DeviceGainReport);
		r.set(pid::use::DeviceGain, g);
		h().send(r);
	}

	/** A running constant force on a new effect (Direction Enable, 9000 = 90 degrees) */
	int64_t startConstant(int64_t magnitude, int64_t direction = 9000, int64_t gain = 255, int64_t duration = 32767) {
		const int64_t idx = newEffect(pid::use::EtConstantForce);
		setEffect(idx, pid::use::EtConstantForce, direction, gain, duration);
		setConstant(idx, magnitude);
		operation(idx, pid::use::OpEffectStart);
		return idx;
	}

	/** The cycles of the next `ms` ms: one sample per FFB cycle (TorqueTrace) */
	std::vector<TorqueTrace::Sample> run(uint32_t ms) {
		TorqueTrace trace(chip1);
		trace.start(*this);
		h().observe(&trace);
		h().run(ms);
		h().observe(nullptr);
		return trace.samples();
	}

	/**
	 * Torque after `waitMs`; REQUIREs the last 20 cycles equal (the settling rule: transient 0 cycles at the default settings, the wait is
	 * long enough for every case here; a case with a filter passes a longer wait derived in its comment)
	 */
	int16_t settled(uint32_t waitMs = 300) {
		const std::vector<TorqueTrace::Sample> s = run(waitMs);
		REQUIRE(s.size() >= 20);
		const int16_t v = s.back().torque;
		for (size_t i = s.size() - 20; i < s.size(); i++) {
			REQUIRE(s[i].torque == v);
		}
		return v;
	}
	int16_t settledFlux(uint32_t waitMs = 300) {
		const std::vector<TorqueTrace::Sample> s = run(waitMs);
		REQUIRE(s.size() >= 20);
		const int16_t v = s.back().flux;
		for (size_t i = s.size() - 20; i < s.size(); i++) {
			REQUIRE(s[i].flux == v);
		}
		return v;
	}


	/** Effect Operation Start now; returns the simulated time of the report (the firmware's start tick is this time, 1 ms granularity) and traces the next `ms` ms */
	uint64_t startAndTrace(int64_t idx, uint32_t ms, std::vector<TorqueTrace::Sample> &out) {
		TorqueTrace trace(chip1);
		trace.start(*this);
		h().observe(&trace);
		const uint64_t t0 = HostPlatform::nowUs();
		operation(idx, pid::use::OpEffectStart);
		h().run(ms);
		h().observe(nullptr);
		out = trace.samples();
		return t0;
	}

	/** Stops an effect and lets the torque return to 0 */
	void stop(int64_t idx) {
		operation(idx, pid::use::OpEffectStop);
		h().run(200);
	}

	/** Reference: torque of a constant force of the given magnitude (direction 9000, gains 255), on its own effect, then stopped */
	int16_t reference(int64_t magnitude) {
		const int64_t idx = startConstant(magnitude);
		const int16_t t = settled();
		stop(idx);
		return t;
	}
};

int sgn(int v) { return (v > 0) - (v < 0); }

using Sample = TorqueTrace::Sample;

/** ms since t0 of a sample (the sample is taken at the end of its scheduler step, up to 1 ms after the cycle; the firmware's start tick has 1 ms granularity: together 2 ms of timing uncertainty, kTimeMs) */
double tMs(const Sample &s, uint64_t t0) { return (static_cast<double>(s.timeUs) - static_cast<double>(t0)) / 1000.0; }
constexpr double kTimeMs = 2.0;
constexpr double kEdgeMs = 1.0; //!< edges of start delay / duration / ramp end: samples land on whole ms, tick granularity 1 ms

/** true if every sample with from <= t <= to is still short of `target` seen from the side `dir` (dir * (target - torque) > 0); counts them */
bool notReached(const std::vector<Sample> &v, uint64_t t0, double from, double to, int target, int dir, int &n) {
	n = 0;
	bool ok = true;
	for (const Sample &x : v) {
		const double t = tMs(x, t0);
		if (t >= from && t <= to) {
			n++;
			ok = ok && dir * (target - static_cast<int>(x.torque)) > 0;
		}
	}
	return ok;
}
/** Torque of the first sample with t >= ms. REQUIREs one exists */
int firstAtOrAfter(const std::vector<Sample> &v, uint64_t t0, double ms) {
	for (const Sample &x : v) {
		if (tMs(x, t0) >= ms) {
			return x.torque;
		}
	}
	FAIL("no sample at or after " << ms << " ms");
	return 0;
}

/** Torque of the sample nearest in time to `ms` after t0 */
int valueAt(const std::vector<Sample> &v, uint64_t t0, double ms) {
	REQUIRE(!v.empty());
	size_t best = 0;
	for (size_t i = 0; i < v.size(); i++) {
		if (std::fabs(tMs(v[i], t0) - ms) < std::fabs(tMs(v[best], t0) - ms)) {
			best = i;
		}
	}
	return v[best].torque;
}

/** true if every sample with from <= t <= to differs from its predecessor in direction `dir` (+1 non-decreasing, -1 non-increasing); counts the samples in the window */
bool monotone(const std::vector<Sample> &v, uint64_t t0, double from, double to, int dir, int &n) {
	n = 0;
	bool ok = true;
	for (size_t i = 1; i < v.size(); i++) {
		const double t = tMs(v[i], t0);
		if (t >= from && t <= to) {
			n++;
			ok = ok && dir * (static_cast<int>(v[i].torque) - static_cast<int>(v[i - 1].torque)) >= 0;
		}
	}
	return ok;
}

/** true if every sample with from <= t <= to is within tol of expected; counts them */
bool holds(const std::vector<Sample> &v, uint64_t t0, double from, double to, int expected, double tol, int &n) {
	n = 0;
	bool ok = true;
	for (const Sample &s : v) {
		const double t = tMs(s, t0);
		if (t >= from && t <= to) {
			n++;
			ok = ok && std::fabs(static_cast<double>(s.torque) - expected) <= tol;
		}
	}
	return ok;
}

} // namespace

TEST_SUITE("F1Effects") {

// ---------------------------------------------------------------------------------------------------------------------
// F1-E1 sign
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.1: with Direction Enable the Direction is a polar direction; a force along theta and along theta + 180 degrees are opposite
// (reversal only, no polar zero needed). Bound: kBound(1) for the two measured torques (k = -1).
TEST_CASE_FIXTURE(EffFixture, "F1-E1: reversing the Direction of a constant force by 18000 (180 degrees) reverses the torque at the chip" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const int64_t idx = startConstant(10000, 9000);
	const int16_t fwd = settled();
	REQUIRE(fwd != 0);
	setEffect(idx, pid::use::EtConstantForce, 27000);
	const int16_t rev = settled();
	CHECK_NEAR(rev, -static_cast<int>(fwd), kBound(1, scalerOf(5000)));
	CHECK(sgn(rev) == -sgn(fwd));
	REQUIRE_CHAIN_MODELLED();
}

// The axis inversion (axis.0.invert, Axis.cpp:884) reverses the torque. Default read back first: 1 on a fresh flash.
// After the inversion the torque is exactly the negative (integer negation after all truncations): bound 0.
TEST_CASE_FIXTURE(EffFixture, "F1-E1: toggling axis.0.invert reverses the torque of a constant force" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const int64_t def = h().getValue("axis.0.invert?");
	CHECK(def == 1); // the fresh-flash default (read back, WO I1-1)
	startConstant(10000, 9000);
	const int16_t before = settled();
	REQUIRE(before != 0);
	h().command("axis.0.invert=" + std::to_string(def ? 0 : 1));
	CHECK(h().getValue("axis.0.invert?") == (def ? 0 : 1));
	const int16_t after = settled(300); // the invert setter only calls resetMetrics (Axis.cpp:1074-1075): no fade-in restarts, nothing to settle; the exact-negation rule holds from the first cycle
	CHECK(after == -before);
	REQUIRE_CHAIN_MODELLED();
}

// polar zero undefined, PID 1.01 5.1; reconciled by the lead. The direction-to-sign mapping with the defaults (invert 1, tmc
// invertForce 0, read back): Direction 9000 (90 degrees) gives a positive torque field, 27000 a negative one. Chain: the axis share is
// sin(theta) (HidFFB.cpp:333), the force is negated (EffectsCalculator.cpp:359), then axis.0.invert = 1 negates again (Axis.cpp:884).
// INR 10.
TEST_CASE_FIXTURE(EffFixture, "characterization: [F1-E1] with the defaults Direction 9000 gives a positive and Direction 27000 a negative torque at register 0x64 (polar zero undefined, PID 1.01 5.1; INR 10)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 255);
	CHECK(h().getValue("axis.0.invert?") == 1);
	CHECK(h().getValue("tmc.0.invertForce?") == 0);
	const int64_t idx = startConstant(10000, 9000);
	CHECK(settled() > 0);
	setEffect(idx, pid::use::EtConstantForce, 27000);
	CHECK(settled() < 0);
	REQUIRE_CHAIN_MODELLED();
}

// polar zero undefined, PID 1.01 5.1. The absolute count, HD:
//   M = 10000, g = 255, G = 255 (Device Gain default, sent explicitly), P = 5000, r = 255, theta = 9000, invert = 1, invertForce = 0:
//   v = 10000*255/255 = 10000; sin(90 deg) = 1.0f -> result = -10000; * G/255 = -10000;
//   FFB: int(-10000 * (1/32767.0f) * 32767 * 255/255) = -10000 or -9999 (float rounding of 1/32767);
//   power: int(-10000 * 5000/32767) = int(-1525.9) = -1525 (and int(-9999 * 0.15259) = int(-1525.8) = -1525);
//   invert: +1525; turn(): +1525 -> 0x64 torque field = +1525.
// INR 10.
TEST_CASE_FIXTURE(EffFixture, "characterization: [F1-E1] Magnitude 10000 at Direction 9000 with the defaults gives +1525 at register 0x64 (hand derivation in the comment; INR 10)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 255);
	setDeviceGain(255);
	startConstant(10000, 9000);
	CHECK(settled() == 1525);
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-E1 linearity
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.6 / 4.1: the force is proportional to the normalised Magnitude. T(M) : T(M/2) : T(-M) = 1 : 1/2 : -1, with M = 20000 and 10000
// (3052 / 1526 counts at P = 5000, far from the clip 5000 and int16). Bounds: kBound(k) with k = 2 and k = -1... (a - k b).
TEST_CASE_FIXTURE(EffFixture, "F1-E1: the torque of a constant force is linear in the Magnitude (M, M/2 and -M)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double s = scalerOf(5000);
	const int16_t tM = reference(20000);
	const int16_t tH = reference(10000);
	const int16_t tN = reference(-20000);
	REQUIRE(tM != 0);
	CHECK_NEAR(tM, 2.0 * tH, kBound(2, s));
	CHECK_NEAR(tN, -static_cast<int>(tM), kBound(1, s));
	CHECK(sgn(tN) == -sgn(tM));
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.1 Gain: "normalized scaling factor applied to all magnitudes of an effect"; descriptor Gain 0..255 (physical 0..10000).
// T(g = 128) = T(g = 255) * 128/255 (k = 128/255 = 0.502).
TEST_CASE_FIXTURE(EffFixture, "F1-E1: the Set Effect Gain scales the torque of a constant force (128 against 255)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const int64_t idx = startConstant(20000, 9000, 255);
	const int16_t full = settled();
	REQUIRE(full != 0);
	setEffect(idx, pid::use::EtConstantForce, 9000, 128);
	const int16_t half = settled();
	CHECK_NEAR(half, full * 128.0 / 255.0, kBound(128.0 / 255.0, scalerOf(5000)));
	setEffect(idx, pid::use::EtConstantForce, 9000, 0);
	CHECK(settled() == 0); // gain 0 = no force
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.10: the Device Gain scales the final output of the device. T(G = 128) = T(G = 255) * 128/255.
TEST_CASE_FIXTURE(EffFixture, "F1-E1: the Device Gain scales the torque of a constant force (128 against 255)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	setDeviceGain(255);
	startConstant(20000);
	const int16_t full = settled();
	REQUIRE(full != 0);
	setDeviceGain(128);
	const int16_t half = settled();
	CHECK_NEAR(half, full * 128.0 / 255.0, kBound(128.0 / 255.0, scalerOf(5000)));
	REQUIRE_CHAIN_MODELLED();
}

// axis.0.fxratio (Axis.cpp:735, :813) scales every effect torque: T(r = 128) = T(r = 255) * 128/255.
TEST_CASE_FIXTURE(EffFixture, "F1-E1: axis.0.fxratio scales the torque of a constant force (128 against 255)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 255);
	startConstant(20000);
	const int16_t full = settled();
	REQUIRE(full != 0);
	h().command("axis.0.fxratio=128");
	CHECK(h().getValue("axis.0.fxratio?") == 128);
	const int16_t half = settled(2600);
	CHECK_NEAR(half, full * 128.0 / 255.0, kBound(128.0 / 255.0, scalerOf(5000)));
	REQUIRE_CHAIN_MODELLED();
}

// axis.0.power sets the scale to the motor's torque range: T(P = 2500) = T(P = 5000) / 2 for a force with T < 2500 at both
// (M = 10000: 1525 and 762 counts; below the clip of both). k = 2500/5000 = 0.5 (the same scaling chain with s = P / 32767).
TEST_CASE_FIXTURE(EffFixture, "F1-E1: axis.0.power scales the torque of a constant force (2500 against 5000, both below the clip)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 255);
	startConstant(10000);
	const int16_t full = settled();
	REQUIRE(full != 0);
	h().command("axis.0.power=2500");
	CHECK(h().getValue("axis.0.power?") == 2500);
	const int16_t half = settled(2600);
	CHECK_NEAR(half, full * 0.5, kBound(0.5, scalerOf(5000)));
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-E1 flux field
// ---------------------------------------------------------------------------------------------------------------------

// characterization of the field weakening policy (TMC4671.cpp:1750-1752, R62; the DS only defines the layout 0x64: flux [15:0]):
// flux = idleFlux - min(|torque|, tmc.0.fluxoffset) with idleFlux = 0: so flux 0 at fluxoffset 0 (default), -100 at fluxoffset 100 and
// -|torque| when fluxoffset exceeds |torque| (torque 1525: fluxoffset 5000 -> -1525). Torque and flux are read from the same register.
// Extends the unit-level characterization of driver charter R62 (TMC4671_REQUIREMENTS.md) to the chain. INR 11.
TEST_CASE_FIXTURE(EffFixture, "characterization: [F1-E1] the flux half of register 0x64 is -min(|torque|, tmc.0.fluxoffset) while a constant force runs (R62; INR 11)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	CHECK(h().getValue("tmc.0.fluxoffset?") == 0);
	startConstant(10000);
	const int16_t t = settled();
	REQUIRE(t != 0);
	CHECK(settledFlux() == 0);
	h().command("tmc.0.fluxoffset=100");
	CHECK(h().getValue("tmc.0.fluxoffset?") == 100);
	// the driver writes 0x64 only when the torque changes (Axis.cpp:488-495): restart the effect so a write happens
	const int64_t idx2 = startConstant(10000);
	(void)idx2;
	h().run(50);
	{
		// two constant forces now run: torque 2 t, flux -min(|torque|, 100) = -100 (|torque| about 3050 > 100)
		const std::vector<TorqueTrace::Sample> s = run(200);
		REQUIRE(!s.empty());
		CHECK_NEAR(s.back().torque, 2 * t, kBound(2, scalerOf(5000))); // two forces sum before the power scaling (3051 = int(20000 * 0.15259))
		CHECK(s.back().flux == -100);
	}
	h().command("tmc.0.fluxoffset=5000");
	const int64_t idx3 = startConstant(2000);
	(void)idx3;
	const std::vector<TorqueTrace::Sample> s3 = run(200);
	REQUIRE(!s3.empty());
	CHECK(s3.back().flux == -std::abs(static_cast<int>(s3.back().torque))); // |torque| < 5000: flux = -|torque|
	REQUIRE_CHAIN_MODELLED();
}


// ---------------------------------------------------------------------------------------------------------------------
// F1-E1 seam S5: the constant force filter
// ---------------------------------------------------------------------------------------------------------------------

// S5 (charter): the constant force filter is chosen by filter[0] (fx.0.filterCfFreq / filterCfQ). Default read back 500 Hz, Q 70 (0.70);
// the FFB rate read back by `main.hidsendspd?` (rate table FFBHIDMain.h:124-126: index 0 = 1 kHz, the default mode of FFBWheel), and
// EffectsCalculator::calcfrequency = 1000 Hz (EffectsCalculator.h:231). No document says that 500 means "off": the default {500, 70} is
// uncommented (EffectsCalculator.h:61) and the command help is only "Constant force filter frequency"; checkFilterCoeff
// (EffectsCalculator.cpp:868-872) maps 0 to calcfrequency/2 and clips to [1, calcfrequency/2], so 500 is the maximum. Fc = 500/1000 = 0.5, and
// the application at EffectsCalculator.cpp:345 happens only for Fc < 0.5: the configured default filter is not applied. The bypass is
// necessary (a biquad low-pass at Fc = 0.5 saturates a constant input), so this is how the firmware makes 500 the "no filter" setting.
// HD of the expected value: Magnitude 20000 at P 5000, r 255: int(20000 * 5000/32767) = 3051 (3051.9; the float chain may pass 19999 first:
// 3051.7, same count). The first 20 cycles after the start equal it: transient 0 cycles. A filter applied at Fc = 0.5 would drive the
// torque to the clip, so this case fails against it. Limit (not testable here): the condition at :345 tests filter[axis] and filter[0]
// (`effect->filter[axis]->getFc() < 0.5 && effect->filter[0]->getFc() != 0.0`); with the one axis of FFBWheel the two are the same object.
// INR 12.
TEST_CASE_FIXTURE(EffFixture, "characterization: [F1-E1] the default constant force filter (500 Hz at the 1000 Hz FFB rate) is off: the first 20 cycles after the start already equal the steady torque 3051 (INR 12)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	CHECK(h().getValue("fx.0.filterCfFreq?") == 500);
	CHECK(h().getValue("fx.0.filterCfQ?") == 70);
	const int64_t rateIndex = h().getValue("main.hidsendspd?");
	MESSAGE("FFB rate setting main.hidsendspd = " << rateIndex << " (0 = 1000 Hz)");
	CHECK(rateIndex == 0);
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	setEffect(idx, pid::use::EtConstantForce, 9000);
	setConstant(idx, 20000);
	std::vector<Sample> s;
	startAndTrace(idx, 100, s);
	REQUIRE(s.size() >= 50);
	for (size_t i = 0; i < 20; i++) {
		CHECK(s[i].torque == 3051); // first cycle included
	}
	CHECK(s.back().torque == 3051);
	REQUIRE_CHAIN_MODELLED();
}

// The filter setting acts below the Nyquist frequency: filterCfFreq = 50 gives Fc = 0.05 (applied), a 2nd order low-pass (Q 0.70): the first
// cycle after the start is b0 * step = 0.0200 * step (K = tan(0.05 pi) = 0.158, b0 = K^2 / (1 + K/Q + K^2) = 0.0200: 61 of 3051 counts), asserted
// as < 1/4 of steady (loose). The rising edge (first 9 cycles) is monotone; afterwards the Butterworth-like step response (Q 0.70) overshoots
// by about 4 %, so no monotone rise is claimed beyond the rising edge, overshoot < 10 %; the torque is at the unfiltered value (reference
// measured with the filter at 500) after 300 ms (> 90 time constants of 1/(2 pi 50 Hz) = 3.2 ms).
TEST_CASE_FIXTURE(EffFixture, "F1-E1: a constant force filter of 50 Hz delays the first cycles and the torque still reaches the unfiltered value" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const int16_t unfiltered = reference(20000);
	REQUIRE(unfiltered != 0);
	h().command("fx.0.filterCfFreq=50");
	CHECK(h().getValue("fx.0.filterCfFreq?") == 50);
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	setEffect(idx, pid::use::EtConstantForce, 9000);
	setConstant(idx, 20000);
	std::vector<Sample> s;
	startAndTrace(idx, 300, s);
	REQUIRE(s.size() >= 50);
	CHECK(std::abs(static_cast<int>(s.front().torque)) * 4 < std::abs(static_cast<int>(unfiltered)));
	int n = 0;
	CHECK(monotone(s, s.front().timeUs, 0, 8.5, sgn(unfiltered), n));
	CHECK(n >= 7);
	int peak = 0;
	for (const Sample &x : s) {
		peak = std::max(peak, std::abs(static_cast<int>(x.torque)));
	}
	CHECK(peak <= std::abs(static_cast<int>(unfiltered)) * 1.10);
	CHECK_NEAR(s.back().torque, unfiltered, 1);
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-E1 duration and start delay (PID 5.1)
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.1: Start Delay = time between Op Effect Start and initiation of the effect, Duration = length of the effect (ms). Start Delay 300 ms,
// Duration 500 ms: torque 0 up to 299 ms, T from 301 to 799 ms, 0 from 801 ms (+-1 ms edges: samples land on whole ms and the firmware's
// start tick has 1 ms granularity; the samples at 300 and 800 ms are not asserted).
TEST_CASE_FIXTURE(EffFixture, "F1-E1: a constant force with Start Delay 300 ms and Duration 500 ms acts from 300 ms to 800 ms after the start" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const int16_t ref = reference(20000);
	REQUIRE(ref != 0);
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	setEffect(idx, pid::use::EtConstantForce, 9000, 255, 500, 300);
	setConstant(idx, 20000);
	std::vector<Sample> s;
	const uint64_t t0 = startAndTrace(idx, 1200, s);
	int n = 0;
	CHECK(holds(s, t0, 0, 300 - kEdgeMs, 0, 0, n));
	CHECK(n > 250);
	CHECK(holds(s, t0, 300 + kEdgeMs, 800 - kEdgeMs, ref, 0, n));
	CHECK(n > 450);
	CHECK(holds(s, t0, 800 + kEdgeMs, 1190, 0, 0, n));
	CHECK(n > 300);
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-E3 ramp
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.7: Ramp Start / Ramp End are the normalised magnitude at the start / end of the effect, Duration D in ms (unit exponent -3 in the
// descriptor). Plain assertions: monotone from the torque of Ramp Start to that of Ramp End (references: constant forces of the same
// magnitudes), End reached at D within one FFB cycle, the effect ends at D (the firmware: tick - start > D, EffectsCalculator.cpp:138, so
// 0 from D + 1 ms). Start 4000, End 20000, D = 2000 ms (TS 610, TE 3051 counts at P 5000, slope 1.22 counts/ms). Tolerance at the end
// points: 2 ms of timing (kTimeMs) x slope + the rounding bound kBound(2). IB 1: the FFB period is 1001 us, so "one FFB cycle" (1.001 ms)
// is inside kTimeMs.
TEST_CASE_FIXTURE(EffFixture, "F1-E3: a ramp moves monotonically from the torque of its Ramp Start to that of its Ramp End, reaches the End at Duration and ends there" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t tS = reference(4000);
	const int16_t tE = reference(20000);
	REQUIRE(tS != 0);
	REQUIRE(tE != tS);
	const double D = 2000.0;
	const double slope = std::fabs(static_cast<double>(tE) - tS) / D;
	const double tolEnd = kTimeMs * slope + kBound(2, sc);
	const int64_t idx = newEffect(pid::use::EtRamp);
	setEffect(idx, pid::use::EtRamp, 9000, 255, 2000);
	setRamp(idx, 4000, 20000);
	std::vector<Sample> s;
	const uint64_t t0 = startAndTrace(idx, 2300, s);
	REQUIRE(s.size() > 2000);
	CHECK_NEAR(s.front().torque, tS, (kTimeMs + 1.0) * slope + kBound(2, sc));
	int n = 0;
	CHECK(monotone(s, t0, 0, D, sgn(tE - tS), n));
	CHECK(n > 1900);
	CHECK_NEAR(valueAt(s, t0, D - 1.001), tE, tolEnd);
	CHECK(holds(s, t0, D + kEdgeMs, 2290, 0, 0, n));
	CHECK(n > 250);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.7 does not name the interpolation between Ramp Start and Ramp End ("Normalized magnitude at the start / end of the effect"); the
// firmware is linear in time (EffectsCalculator.cpp:228). Linearity at 25, 50 and 75 % of D: T = TS + (TE - TS) f, tolerance 2 ms x slope
// + kBound(2). INR 13.
TEST_CASE_FIXTURE(EffFixture, "characterization: [F1-E3] a ramp is linear in time between the torques of Ramp Start and Ramp End at 25, 50 and 75 percent of Duration (PID 5.7 names no shape; INR 13)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t tS = reference(4000);
	const int16_t tE = reference(20000);
	REQUIRE(tE != tS);
	const double D = 2000.0;
	const double tol = kTimeMs * std::fabs(static_cast<double>(tE) - tS) / D + kBound(2, sc);
	const int64_t idx = newEffect(pid::use::EtRamp);
	setEffect(idx, pid::use::EtRamp, 9000, 255, 2000);
	setRamp(idx, 4000, 20000);
	std::vector<Sample> s;
	const uint64_t t0 = startAndTrace(idx, 2100, s);
	REQUIRE(s.size() > 2000);
	for (double f : {0.25, 0.5, 0.75}) {
		INFO("fraction " << f);
		CHECK_NEAR(valueAt(s, t0, f * D), tS + (static_cast<double>(tE) - tS) * f, tol);
	}
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-E4 envelope
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.2 does not state that the transition is linear ("transition time to reach the sustain level"): end points, monotonicity, holds and
// timing are asserted, no interior shape. Baseline of a constant force = 0 (no offset). Attack only: M = 20000, Attack Level 4000, Attack
// Time 1000 ms, Fade Time 0, Duration 4000 ms. Torque: starts at T(AL), non-decreasing, NOT yet T(M) before Attack Time minus 2 ms (PID 5.2:
// Attack Time is the transition time to reach the sustain level; 2 ms = kTimeMs), T(M) after it until the end at 4000 ms.
TEST_CASE_FIXTURE(EffFixture, "F1-E4: an attack-only envelope starts at the torque of the Attack Level, reaches the magnitude's torque at the Attack Time and holds it" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t tA = reference(4000), tM = reference(20000);
	REQUIRE(tM != tA);
	const double slope = std::fabs(static_cast<double>(tM) - tA) / 1000.0;
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	setEffect(idx, pid::use::EtConstantForce, 9000, 255, 4000);
	setConstant(idx, 20000);
	setEnvelope(idx, 4000, 1000, 0, 0);
	std::vector<Sample> s;
	const uint64_t t0 = startAndTrace(idx, 4300, s);
	REQUIRE(s.size() > 4000);
	CHECK_NEAR(s.front().torque, tA, (kTimeMs + 1.0) * slope + kBound(1, sc));
	int n = 0;
	CHECK(monotone(s, t0, 0, 1000, sgn(tM - tA), n));
	CHECK(n > 900);
	CHECK(notReached(s, t0, 0, 1000 - kTimeMs, tM, sgn(tM - tA), n)); // has not reached the sustain level before the Attack Time
	CHECK(n > 900);
	CHECK_NEAR(valueAt(s, t0, 1000 + kTimeMs), tM, kBound(1, sc));
	CHECK(holds(s, t0, 1000 + kTimeMs + 1, 3990, tM, kBound(1, sc), n));
	CHECK(n > 2900);
	CHECK(holds(s, t0, 4000 + kEdgeMs, 4290, 0, 0, n)); // the effect ends at Duration
	CHECK(n > 250);
	REQUIRE_CHAIN_MODELLED();
}

// Fade only: M = 20000, Attack Time 0 (no attack), Fade Level 2000, Fade Time 1000 ms, Duration 4000 ms. Torque: T(M) until 3000 ms; it has
// LEFT T(M) (towards T(FL)) by 3002 ms (PID 5.2: Fade Time is the time to reach the fade level; the firmware starts when elapsed > D - Fade
// Time, 1.8 counts per ms away from T(M), so by the first sample from 3002 ms it is below T(M) by more than the rounding); non-increasing
// towards T(FL), within the end tolerance of T(FL) at the last cycle before Duration; 0 after.
TEST_CASE_FIXTURE(EffFixture, "F1-E4: a fade-only envelope holds the magnitude's torque, then fades towards the torque of the Fade Level over the last Fade Time" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t tF = reference(2000), tM = reference(20000);
	REQUIRE(tM != tF);
	const double slope = std::fabs(static_cast<double>(tM) - tF) / 1000.0;
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	setEffect(idx, pid::use::EtConstantForce, 9000, 255, 4000);
	setConstant(idx, 20000);
	setEnvelope(idx, 0, 0, 2000, 1000);
	std::vector<Sample> s;
	const uint64_t t0 = startAndTrace(idx, 4300, s);
	REQUIRE(s.size() > 4000);
	int n = 0;
	CHECK(holds(s, t0, 0, 3000 - kTimeMs - 1, tM, kBound(1, sc), n));
	CHECK(n > 2900);
	CHECK(sgn(tF - tM) * (firstAtOrAfter(s, t0, 3002) - tM) > 0); // already fading
	CHECK(monotone(s, t0, 3000, 4000, sgn(tF - tM), n));
	CHECK(n > 900);
	CHECK_NEAR(valueAt(s, t0, 4000 - 1.001), tF, (kTimeMs + 1.0) * slope + kBound(1, sc));
	CHECK(holds(s, t0, 4000 + kEdgeMs, 4290, 0, 0, n));
	CHECK(n > 250);
	REQUIRE_CHAIN_MODELLED();
}

// Attack and fade, Attack Level above the magnitude (PID 5.2 limits Attack Level to the Normalized range, the descriptor's 0..32767 is the
// range of Magnitude; no rule forbids a level above the magnitude): M = 10000, Attack Level 30000, Attack Time 1000, Fade Level 2000,
// Fade Time 1000, Duration 4000. Torque: T(30000) (4577 counts, below the clip 5000) falling to T(M), not yet at T(M) before 998 ms, T(M)
// from 1000 to 3000 ms, already fading at 3002 ms, falling to T(2000) at 4000 ms.
TEST_CASE_FIXTURE(EffFixture, "F1-E4: an envelope with an Attack Level above the magnitude starts high, falls to the magnitude's torque, holds, and fades to the Fade Level" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t tA = reference(30000), tM = reference(10000), tF = reference(2000);
	REQUIRE(std::abs(static_cast<int>(tA)) > std::abs(static_cast<int>(tM)));
	REQUIRE(std::abs(static_cast<int>(tM)) > std::abs(static_cast<int>(tF)));
	const double slopeA = std::fabs(static_cast<double>(tA) - tM) / 1000.0;
	const double slopeF = std::fabs(static_cast<double>(tM) - tF) / 1000.0;
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	setEffect(idx, pid::use::EtConstantForce, 9000, 255, 4000);
	setConstant(idx, 10000);
	setEnvelope(idx, 30000, 1000, 2000, 1000);
	std::vector<Sample> s;
	const uint64_t t0 = startAndTrace(idx, 4300, s);
	REQUIRE(s.size() > 4000);
	CHECK_NEAR(s.front().torque, tA, (kTimeMs + 1.0) * slopeA + kBound(1, sc));
	int n = 0;
	CHECK(monotone(s, t0, 0, 1000, sgn(tM - tA), n));
	CHECK(n > 900);
	CHECK(notReached(s, t0, 0, 1000 - kTimeMs, tM, sgn(tM - tA), n));
	CHECK(n > 900);
	CHECK_NEAR(valueAt(s, t0, 1000 + kTimeMs), tM, kBound(1, sc));
	CHECK(holds(s, t0, 1000 + kTimeMs + 1, 3000 - kTimeMs - 1, tM, kBound(1, sc), n));
	CHECK(n > 1900);
	CHECK(sgn(tF - tM) * (firstAtOrAfter(s, t0, 3002) - tM) > 0);
	CHECK(monotone(s, t0, 3000, 4000, sgn(tF - tM), n));
	CHECK(n > 900);
	CHECK_NEAR(valueAt(s, t0, 4000 - 1.001), tF, (kTimeMs + 1.0) * slopeF + kBound(1, sc));
	CHECK(holds(s, t0, 4000 + kEdgeMs, 4290, 0, 0, n));
	CHECK(n > 250);
	REQUIRE_CHAIN_MODELLED();
}

// IB 20. PID 5.1 Duration: "If an envelope has been applied to the effect, then the attack will be applied, followed by an infinite
// sustain." The firmware maps Duration 0 to INFINITE (HidFFB.cpp:341; the descriptor declares no Null state for Duration,
// usb_hid_ffb_desc.h:240-252, so infinite is reachable only through that 0 = infinite rule; lead's decision: still a defect of the
// firmware's infinite mode) and getEnvelopeMagnitude returns the plain magnitude for an infinite effect (EffectsCalculator.cpp:490-492):
// no attack. Intended (shape-free): the first cycle is at the Attack Level's torque, far below T(M).
TEST_CASE_FIXTURE(EffFixture, "F1-E4: an envelope on a constant force of infinite Duration still applies its attack (PID 5.1 Duration)"
	* KNOWN_ISSUE("(IB 20) getEnvelopeMagnitude returns the plain magnitude when Duration is infinite (EffectsCalculator.cpp:490-492), so the attack is never applied") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const int16_t tA = reference(4000), tM = reference(20000);
	REQUIRE(tM != tA);
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	setEffect(idx, pid::use::EtConstantForce, 9000, 255, 0); // Duration 0 = infinite in the firmware
	setConstant(idx, 20000);
	setEnvelope(idx, 4000, 1000, 0, 0);
	std::vector<Sample> s;
	startAndTrace(idx, 300, s);
	REQUIRE(s.size() > 100);
	REQUIRE_CHAIN_MODELLED();
	CHECK(std::abs(static_cast<int>(s.front().torque)) * 2 < std::abs(static_cast<int>(tM))); // the defect: the first cycle is already T(M)
	CHECK(sgn(tM - tA) * (tM - static_cast<int>(s[100].torque)) > 0); // and still short of T(M) after 100 cycles
}

// IB 21. PID 1.01 Table 3 (5.1.1.1): "ET Ramp | Ramp | Envelope": a ramp effect may carry an envelope. The firmware skips the envelope for
// every Ramp (EffectsCalculator.cpp:190-210 `if (effect->type != FFB_EFFECT_RAMP)`), although set_ramp sets magnitude 0x7fff "for envelope
// calculation" (HidFFB.cpp:457). Intended (shape-free): a ramp from 20000 to 20000 with Attack Level 4000 over 1000 ms starts at the torque
// of 4000, far below the ramp value T(20000).
TEST_CASE_FIXTURE(EffFixture, "F1-E4: an envelope on a ramp effect shapes the ramp (PID 1.01 Table 3: ET Ramp uses an Envelope)"
	* KNOWN_ISSUE("(IB 21) the envelope is skipped for every Ramp effect (EffectsCalculator.cpp:190-210) although set_ramp prepares a magnitude for it (HidFFB.cpp:457)") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const int16_t tA = reference(4000), tM = reference(20000);
	REQUIRE(tM != tA);
	const int64_t idx = newEffect(pid::use::EtRamp);
	setEffect(idx, pid::use::EtRamp, 9000, 255, 4000);
	setRamp(idx, 20000, 20000);
	setEnvelope(idx, 4000, 1000, 0, 0);
	std::vector<Sample> s;
	startAndTrace(idx, 300, s);
	REQUIRE(s.size() > 100);
	REQUIRE_CHAIN_MODELLED();
	CHECK(std::abs(static_cast<int>(s.front().torque)) * 2 < std::abs(static_cast<int>(tM))); // the defect: the first cycle is already the ramp value
	CHECK(sgn(tM - tA) * (tM - static_cast<int>(s[100].torque)) > 0);
}

} // TEST_SUITE
