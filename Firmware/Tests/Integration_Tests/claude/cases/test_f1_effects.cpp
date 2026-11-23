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
 * Start Delay with an infinite Duration (ignored, EffectsCalculator.cpp:131-135) is tested by WO I1-4 below: case "F1-E10: a constant force
 * with Start Delay 300 ms and infinite Duration (0) starts 300 ms after the Start" (IB 27).
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

// ---------------------------------------------------------------------------------------------------------------------
// WO I1-4 helpers (periodic effects F1-E2, Effect Operation F1-E10, Device Control over time F1-E11)
//
// Sense of a torque: PID 1.01 5.1 defines no polar zero, so no case here relies on the sign the firmware produces. `fsign` is the sign of
// the torque of a constant force of POSITIVE Magnitude at the same Direction (measured once per case); a periodic torque is multiplied by
// it, so "x > 0" means "along the positive-Magnitude force" and "rises" means "rises in that sense". The expected levels are the torques
// of reference constant forces (the same scaling chain), so Offset +- Magnitude is asserted as T(Offset +- Magnitude).
// Timing: one sample per FFB cycle (IB 1: 1.001 ms), a crossing time is linearly interpolated between two samples. Each crossing time is
// uncertain by up to 1 ms (the sample is taken at the end of its 1 ms scheduler step, the waveforms advance in whole ms of the elapsed time),
// so the difference of two crossings by up to kTimeMs = 2 ms: the mean period over n crossings is known to kTimeMs / (n - 1), a phase shift to kTimeMs.
// ---------------------------------------------------------------------------------------------------------------------
namespace {

struct Wave {
	uint32_t type;
	const char *name;
};
const Wave kSine = {pid::use::EtSine, "sine"};
const Wave kSquare = {pid::use::EtSquare, "square"};
const Wave kTriangle = {pid::use::EtTriangle, "triangle"};
const Wave kSawUp = {pid::use::EtSawtoothUp, "sawtooth up"};
const Wave kSawDown = {pid::use::EtSawtoothDown, "sawtooth down"};
const Wave kAllWaves[] = {kSine, kSquare, kTriangle, kSawUp, kSawDown};
const Wave kWavesButSquare[] = {kSine, kTriangle, kSawUp, kSawDown};

struct PerFixture : EffFixture {
	void setPeriodic(int64_t idx, int64_t magnitude, int64_t offset, int64_t phase, int64_t period) {
		pid::PidReport r = h().builder().make(ReportType::Output, pid::use::SetPeriodicReport);
		r.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, magnitude).set(pid::use::Offset, offset);
		r.set(pid::use::Phase, phase).set(pid::use::Period, period);
		h().send(r);
	}
	/** A periodic effect created and configured, not started. Direction 9000, gain 255 */
	int64_t makePeriodic(const Wave &w, int64_t magnitude, int64_t offset, int64_t phase, int64_t period, int64_t duration = 32767, int64_t startDelay = 0) {
		const int64_t idx = newEffect(w.type);
		setEffect(idx, w.type, 9000, 255, duration, startDelay);
		setPeriodic(idx, magnitude, offset, phase, period);
		return idx;
	}
	/** The sign of a constant force of positive Magnitude at Direction 9000 (the sense of the other torques, see above) */
	int fsign() {
		const int16_t t = reference(10000);
		REQUIRE(t != 0);
		return sgn(t);
	}
	/** Creates, starts and traces a periodic effect for `ms` ms, then stops it. x = torque * fsign */
	std::vector<Sample> runPeriodic(const Wave &w, int64_t magnitude, int64_t offset, int64_t phase, int64_t period, uint32_t ms, uint64_t &t0) {
		const int64_t idx = makePeriodic(w, magnitude, offset, phase, period);
		std::vector<Sample> s;
		t0 = startAndTrace(idx, ms, s);
		stop(idx);
		return s;
	}
	void control(uint32_t dcUsage) { h().send(h().builder().deviceControl(dcUsage)); }
	/** Device Control as the firmware's bit flags (HidFFB::ffb_control: 1 enable, 2 disable, 4 stop, 8 reset, 16 pause, 32 continue), raw byte after the report ID (IB 8) */
	void controlFlags(uint8_t flags) {
		std::vector<uint8_t> b = h().builder().deviceControl(pid::use::DcDeviceReset).bytes();
		REQUIRE(b.size() == 2);
		b[1] = flags;
		CHECK(h().sendRaw(b) == HostDcd::Handshake::Ack);
	}
	/** Effect Operation with an explicit Loop Count */
	void operationLoop(int64_t idx, uint32_t op, int64_t loops) {
		pid::PidReport eo = h().builder().make(ReportType::Output, pid::use::EffectOperationReport);
		eo.set(pid::use::EffectBlockIndex, idx).select(op).set(pid::use::LoopCount, loops);
		h().send(eo);
	}
};

std::vector<double> xs(const std::vector<Sample> &s, int sign) {
	std::vector<double> x;
	for (const Sample &v : s) {
		x.push_back(sign * static_cast<double>(v.torque));
	}
	return x;
}

/** Times (ms after t0) at which x goes from < 0 to >= 0, linearly interpolated between the two samples */
std::vector<double> upCrossings(const std::vector<Sample> &s, int sign, uint64_t t0) {
	std::vector<double> out;
	const std::vector<double> x = xs(s, sign);
	for (size_t i = 1; i < s.size(); i++) {
		if (x[i - 1] < 0 && x[i] >= 0) {
			const double ta = tMs(s[i - 1], t0), tb = tMs(s[i], t0);
			out.push_back(ta + (0 - x[i - 1]) / (x[i] - x[i - 1]) * (tb - ta));
		}
	}
	return out;
}

double circDist(double a, double b, double period) {
	double d = std::fmod(a - b, period);
	if (d < 0) {
		d += period;
	}
	return std::min(d, period - d);
}

/** The shift (ms) between the first up-crossings of two runs, modulo the period: (u0 - uPhi) mod P */
double firstShift(const std::vector<double> &u0, const std::vector<double> &uPhi, double period) {
	REQUIRE(!u0.empty());
	REQUIRE(!uPhi.empty());
	double d = std::fmod(u0.front() - uPhi.front(), period);
	return d < 0 ? d + period : d;
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


// =====================================================================================================================
// WO I1-4. Common settings of every case: configure() (power 5000, fxratio 255 first, IB 7), Direction 9000, Gain 255, Device Gain 255.
// Expected torque levels are T(M) = reference(M), the torque of a constant force of Magnitude M (same chain): T(10000) = 1525 at P 5000.
// =====================================================================================================================

namespace {

/** Torque extremes of the samples of a window */
void extremes(const std::vector<Sample> &s, int &lo, int &hi) {
	lo = 1 << 20;
	hi = -(1 << 20);
	for (const Sample &x : s) {
		lo = std::min(lo, static_cast<int>(x.torque));
		hi = std::max(hi, static_cast<int>(x.torque));
	}
}

/** true if every sample with from <= t <= to has |torque| within tol of |level|; counts them */
bool absNear(const std::vector<Sample> &v, uint64_t t0, double from, double to, int level, double tol, int &n) {
	n = 0;
	bool ok = true;
	for (const Sample &s : v) {
		const double t = tMs(s, t0);
		if (t >= from && t <= to) {
			n++;
			ok = ok && std::fabs(std::fabs(static_cast<double>(s.torque)) - std::abs(level)) <= tol;
		}
	}
	return ok;
}

/**
 * Period check, F1-E2: the mean spacing of the up-crossings of x, (last - first) / (n - 1), equals the Period within kTimeMs / (n - 1) (each
 * end uncertain by 1 ms, see above; 20 periods in the window, so n - 1 >= 17 and the bound is <= 0.12 ms: a period 1 ms off fails), and no single
 * spacing is off by more than kTimeMs.
 */
void checkPeriod(PerFixture &f, const Wave &w, int64_t period) {
	INFO("waveform " << std::string(w.name) << ", Period " << period << " ms");
	const int sg = f.fsign();
	uint64_t t0 = 0;
	const std::vector<Sample> s = f.runPeriodic(w, 10000, 0, 0, period, static_cast<uint32_t>(period * 20), t0);
	const std::vector<double> u = upCrossings(s, sg, t0);
	REQUIRE(u.size() >= 18); // 20 periods in the window, the first crossing needs a negative sample before it
	const double n1 = static_cast<double>(u.size() - 1);
	const double mean = (u.back() - u.front()) / n1;
	double worst = 0;
	for (size_t i = 1; i < u.size(); i++) {
		worst = std::max(worst, std::fabs((u[i] - u[i - 1]) - static_cast<double>(period)));
	}
	INFO("mean period " << mean << " ms over " << n1 << " spacings, worst single spacing error " << worst << " ms");
	CHECK_NEAR(mean, static_cast<double>(period), kTimeMs / n1);
	CHECK(worst <= kTimeMs);
}

/** Phase check, F1-E2: Phase (0.01 degree units) moves the up-crossing earlier by Phase/36000 of the period (PID 5.5: the position in the wave where playback begins) */
void checkPhase(PerFixture &f, const Wave &w, int64_t phase, int64_t period) {
	INFO("waveform " << std::string(w.name) << ", Phase " << phase << ", Period " << period << " ms");
	const int sg = f.fsign();
	uint64_t t0a = 0, t0b = 0;
	const std::vector<Sample> a = f.runPeriodic(w, 10000, 0, 0, period, static_cast<uint32_t>(period * 6), t0a);
	const std::vector<Sample> b = f.runPeriodic(w, 10000, 0, phase, period, static_cast<uint32_t>(period * 6), t0b);
	const std::vector<double> ua = upCrossings(a, sg, t0a), ub = upCrossings(b, sg, t0b);
	REQUIRE(ua.size() >= 3);
	REQUIRE(ub.size() >= 3);
	const double pm = (ua.back() - ua.front()) / static_cast<double>(ua.size() - 1); // measured period (own value, so a period error is not counted twice)
	const double expected = static_cast<double>(phase) / 36000.0 * pm;
	const double shift = firstShift(ua, ub, pm);
	INFO("measured period " << pm << " ms, shift " << shift << " ms, expected " << expected << " ms");
	CHECK(circDist(shift, expected, pm) <= kTimeMs);
}

struct Steps {
	int pos = 0, neg = 0, jumps = 0, jumpUp = 0, jumpDown = 0;
};
Steps stepStats(const std::vector<double> &x, double jumpMin) {
	Steps r;
	for (size_t i = 1; i < x.size(); i++) {
		const double d = x[i] - x[i - 1];
		if (std::fabs(d) > jumpMin) {
			r.jumps++;
			(d > 0 ? r.jumpUp : r.jumpDown)++;
		} else if (d > 0) {
			r.pos++;
		} else if (d < 0) {
			r.neg++;
		}
	}
	return r;
}

/**
 * F1-E2 span: Offset +- Magnitude (PID 5.5) as the torques of the constant forces Offset + Magnitude and Offset - Magnitude (Period P = 100 ms,
 * 6 periods). Tolerance per waveform = kBound(1) (the rounding bound of two torques of the chain) + the distance of the nearest sample from the
 * extreme, as a fraction of the span:
 *  - sine: flat extreme; the nearest of the samples (1.001 ms apart) is within 0.5 ms of the peak: (1 - cos(2 pi 0.5 / P)) / 2 = 0.012 % of
 *    the span; square: the levels themselves are sampled. Both are allowed 1 % (float rounding of sin / the reference, margin): a sine 5 % low fails;
 *  - triangle: slope 2 span / P per ms, sharp extreme; with whole-ms sampling the nearest sample is within 1 ms of it: 2 % (plus 1 % margin) = 3 %;
 *  - sawtooth: slope span / P per ms, the end of the ramp is reached one sample before the jump at the latest: 1 % (plus 1 % margin) = 2 %.
 */
void checkSpan(PerFixture &f, const Wave &w, int64_t offset, int64_t magnitude) {
	INFO("waveform " << std::string(w.name) << ", Offset " << offset << ", Magnitude " << magnitude);
	const double sc = f.scalerOf(5000);
	const int16_t tHi = f.reference(offset + magnitude), tLo = f.reference(offset - magnitude);
	const double span = std::fabs(static_cast<double>(tHi) - tLo);
	REQUIRE(span > 0);
	const double frac = (w.type == pid::use::EtTriangle) ? 0.03 : (w.type == pid::use::EtSawtoothUp || w.type == pid::use::EtSawtoothDown) ? 0.02 : 0.01;
	const double tol = span * frac + kBound(1, sc);
	uint64_t t0 = 0;
	const std::vector<Sample> s = f.runPeriodic(w, magnitude, offset, 0, 100, 600, t0);
	int lo = 0, hi = 0;
	extremes(s, lo, hi);
	INFO("torque range " << lo << " .. " << hi << ", expected " << std::min(tHi, tLo) << " .. " << std::max(tHi, tLo));
	CHECK_NEAR(hi, std::max(tHi, tLo), tol);
	CHECK_NEAR(lo, std::min(tHi, tLo), tol);
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------------
// F1-E2 periodic effects. PID 5.5: "The range of forces generated by the effect will be Offset - Magnitude to Offset + Magnitude";
// "Once started a periodic effect will loop every Period"; Phase: "the position in the wave where playback begins ... between 0 and 360
// degrees" (descriptor units 0.01 degree, 0..35999); Period in ms (descriptor unit exponent -3). The descriptor declares Magnitude 0..32767
// (usb_hid_ffb_desc.h:381-388), so a negative Magnitude cannot be sent (the report builder would reject it): negative values are tested for the Offset
// (-32767..32767) only.
// ---------------------------------------------------------------------------------------------------------------------

#define PERIODIC_SPAN_CASE(WAVE, TEXT) \
	TEST_CASE_FIXTURE(PerFixture, "F1-E2: a " TEXT " effect of Magnitude 10000 and Offset 0 spans the torques of the constant forces -10000 and +10000" * IT_DELETES_FFBHIDMAIN) { \
		up(); \
		configure(); \
		checkSpan(*this, WAVE, 0, 10000); \
		REQUIRE_CHAIN_MODELLED(); \
	}
PERIODIC_SPAN_CASE(kSine, "sine")
PERIODIC_SPAN_CASE(kSquare, "square")
PERIODIC_SPAN_CASE(kTriangle, "triangle")
PERIODIC_SPAN_CASE(kSawUp, "sawtooth up")
PERIODIC_SPAN_CASE(kSawDown, "sawtooth down")

// PID 5.5: range Offset - Magnitude .. Offset + Magnitude, with a negative Offset (-6000, Magnitude 10000: -16000 .. +4000).
TEST_CASE_FIXTURE(PerFixture, "F1-E2: every periodic waveform with a negative Offset spans the torques of the constant forces Offset - Magnitude and Offset + Magnitude" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	for (const Wave &w : kAllWaves) {
		checkSpan(*this, w, -6000, 10000);
	}
	REQUIRE_CHAIN_MODELLED();
}

// Same with a positive Offset (5000, Magnitude 10000: -5000 .. +15000).
TEST_CASE_FIXTURE(PerFixture, "F1-E2: every periodic waveform with a positive Offset spans the torques of the constant forces Offset - Magnitude and Offset + Magnitude" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	for (const Wave &w : kAllWaves) {
		checkSpan(*this, w, 5000, 10000);
	}
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.5 "the amplitude of the entire effect" with Magnitude 0: only the Offset remains: a constant torque T(Offset) for every waveform.
TEST_CASE_FIXTURE(PerFixture, "F1-E2: a periodic effect of Magnitude 0 and Offset 8000 gives the constant torque of a constant force of 8000" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t t = reference(8000);
	REQUIRE(t != 0);
	for (const Wave &w : kAllWaves) {
		INFO("waveform " << std::string(w.name));
		uint64_t t0 = 0;
		const std::vector<Sample> s = runPeriodic(w, 0, 8000, 0, 100, 300, t0);
		int n = 0;
		CHECK(holds(s, t0, 0, 295, t, kBound(1, sc), n));
		CHECK(n > 250);
	}
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.5 "Once started a periodic effect will loop every Period": the up-crossings of the torque are one Period apart (+-1.5 ms: one FFB
// cycle, IB 1 stated, plus the tick granularity). The square wave is a separate case (period + 2 ms in the firmware).
TEST_CASE_FIXTURE(PerFixture, "F1-E2: sine, triangle and both sawtooth effects repeat every Period (Period 80 ms, 20 periods)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	for (const Wave &w : kWavesButSquare) {
		checkPeriod(*this, w, 80);
	}
	REQUIRE_CHAIN_MODELLED();
}

// IB 30: EffectsCalculator.cpp:234-235 `(elapsed_time + phase) % (period + 2)` and `< (period + 2) / 2`: the square wave repeats every
// Period + 2 ms (82 ms for Period 80), PID 5.5: every Period. The mean period over 20 periods is 2 ms off (bound 0.12 ms).
TEST_CASE_FIXTURE(PerFixture, "F1-E2: a square effect repeats every Period (Period 80 ms, 20 periods)"
	* KNOWN_ISSUE("(IB 30) the square wave uses the modulus period + 2 (EffectsCalculator.cpp:234-235), so it repeats every Period + 2 ms") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	REQUIRE_CHAIN_MODELLED();
	checkPeriod(*this, kSquare, 80);
}

// PID 5.5 Phase: "the position in the wave where playback begins", a quarter turn (9000 = 90 degrees) of a Period 80 ms wave starts 20 ms
// further into the cycle: the up-crossing comes 20 ms earlier (compared with Phase 0, modulo the Period, measured period used).
TEST_CASE_FIXTURE(PerFixture, "F1-E2: Phase 9000 (90 degrees) starts sine, triangle and both sawtooth effects a quarter Period into the wave" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	for (const Wave &w : kWavesButSquare) {
		checkPhase(*this, w, 9000, 80);
	}
	REQUIRE_CHAIN_MODELLED();
}

// IB 31: EffectsCalculator.cpp:234-235 adds the raw Phase (0.01 degree units, 0..35999) to the elapsed milliseconds: Phase 18000 shifts
// the square wave by 18000 ms mod 52, not by half a Period.
TEST_CASE_FIXTURE(PerFixture, "F1-E2: Phase 18000 (180 degrees) starts a square effect half a Period into the wave"
	* KNOWN_ISSUE("(IB 31) the square wave adds the raw Phase value (units of 0.01 degree) to the elapsed milliseconds (EffectsCalculator.cpp:234-235)") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	REQUIRE_CHAIN_MODELLED();
	checkPhase(*this, kSquare, 18000, 50);
}

// PID 5.5 names the waveforms only. Shape properties every reading of the names shares. Square: two levels, +-T(Magnitude), both used.
TEST_CASE_FIXTURE(PerFixture, "F1-E2: a square effect takes exactly two torque levels, +T and -T of its Magnitude" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t t = reference(10000);
	REQUIRE(t != 0);
	uint64_t t0 = 0;
	const std::vector<Sample> s = runPeriodic(kSquare, 10000, 0, 0, 100, 800, t0);
	REQUIRE(s.size() > 700);
	int high = 0, low = 0, other = 0;
	for (const Sample &x : s) {
		if (std::fabs(static_cast<double>(x.torque) - std::abs(t)) <= kBound(1, sc) + 1) {
			high++;
		} else if (std::fabs(static_cast<double>(x.torque) + std::abs(t)) <= kBound(1, sc) + 1) {
			low++;
		} else {
			other++;
		}
	}
	CHECK(other == 0);
	CHECK(high > 200);
	CHECK(low > 200);
	REQUIRE_CHAIN_MODELLED();
}

// Triangle: what every reading of the name shares: it rises linearly from the minimum to the maximum and falls linearly back, symmetric
// (rise time = fall time = Period/2) and with one constant slope 2 span / P. Period 100 ms, 800 ms = 8 periods, span = 2 T(10000).
// Asserted: (a) every rise and every fall between two consecutive turning points lasts P/2 +- kTimeMs (at least 12 of them); this rejects a
// sawtooth (99 + 1 ms) and a square; (b) no step larger than 2 x the slope per FFB cycle (a cycle with a skipped ms of elapsed time, IB 1, makes one
// double step; anything larger is a jump: rejects saw / square) and (c) at most 2 steps (the IB 1 skip, the window ends) larger than 1.25 x the
// slope: a sine of the same span has its steepest slope pi/2 = 1.57 x and exceeds 1.25 x in about 40 % of its cycles. Rounding: + 2 counts.
TEST_CASE_FIXTURE(PerFixture, "F1-E2: a triangle effect rises and falls linearly with equal rise and fall times of half a Period" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const int16_t t = reference(10000);
	REQUIRE(t != 0);
	const double span = 2.0 * std::abs(static_cast<int>(t));
	const double slope = 2.0 * span / 100.0; // counts per ms
	uint64_t t0 = 0;
	const std::vector<Sample> s = runPeriodic(kTriangle, 10000, 0, 0, 100, 800, t0);
	REQUIRE(s.size() > 700);
	std::vector<double> turns;
	int last = 0, big = 0;
	double maxStep = 0;
	for (size_t i = 1; i < s.size(); i++) {
		const int step = static_cast<int>(s[i].torque) - static_cast<int>(s[i - 1].torque);
		maxStep = std::max(maxStep, std::fabs(static_cast<double>(step)));
		if (std::fabs(static_cast<double>(step)) > 1.25 * slope + 2) {
			big++;
		}
		const int d = sgn(step);
		if (d != 0) {
			if (last != 0 && d != last) {
				turns.push_back(tMs(s[i - 1], t0));
			}
			last = d;
		}
	}
	INFO("turning points " << turns.size() << ", largest step " << maxStep << " (slope " << slope << " per ms), steps > 1.25 slope: " << big);
	REQUIRE(turns.size() >= 13);
	double worst = 0;
	for (size_t i = 1; i < turns.size(); i++) {
		worst = std::max(worst, std::fabs((turns[i] - turns[i - 1]) - 50.0));
	}
	INFO("worst rise / fall time error " << worst << " ms");
	CHECK(worst <= kTimeMs);
	CHECK(maxStep <= 2.0 * slope + 2);
	CHECK(big <= 2);
	REQUIRE_CHAIN_MODELLED();
}

// Sawtooth (both): inside a period the torque changes monotonically in ONE direction and drops (or rises) back once per period: one jump
// per Period (8 periods: 7..9 jumps larger than half the span), all other steps of the same sign, the jump of the opposite sign.
TEST_CASE_FIXTURE(PerFixture, "F1-E2: a sawtooth effect (up and down) is monotone inside a Period and jumps back once per Period" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const int16_t t = reference(10000);
	REQUIRE(t != 0);
	for (const Wave &w : {kSawUp, kSawDown}) {
		INFO("waveform " << std::string(w.name));
		uint64_t t0 = 0;
		const std::vector<Sample> s = runPeriodic(w, 10000, 0, 0, 100, 800, t0);
		REQUIRE(s.size() > 700);
		const Steps st = stepStats(xs(s, 1), std::abs(t));
		INFO("steps up " << st.pos << " down " << st.neg << ", jumps " << st.jumps << " (up " << st.jumpUp << ", down " << st.jumpDown << ")");
		CHECK(st.jumps >= 7);
		CHECK(st.jumps <= 9);
		CHECK((st.pos == 0 || st.neg == 0));
		CHECK(st.pos + st.neg > 600);
		CHECK((st.pos > 0 ? st.jumpDown == st.jumps : st.jumpUp == st.jumps));
	}
	REQUIRE_CHAIN_MODELLED();
}

// INR 29. No supplied document defines the direction of the sawtooth waveforms: PID 1.01 5.1.1 / Table 2 only name ET Sawtooth Up and ET
// Sawtooth Down (5.1 even describes ET Sawtooth Down as "a Sawtooth Up waveform"), 5.5 names no shape and HID 1.11 says nothing. The lead asks
// the owner (IQ13). Observed: relative to a constant force of positive Magnitude (fsign; the absolute sign is INR 10) the Sawtooth Up force falls
// within each period (EffectsCalculator.cpp:278 `minMagnitude + slope * (period - remainder)`) and the Sawtooth Down force rises (:296
// `minMagnitude + slope * remainder`). Steps larger than 1500 counts are the once-per-period jumps (span 3050) and are not counted.
TEST_CASE_FIXTURE(PerFixture, "characterization: [F1-E2] the Sawtooth Up force falls within each period relative to a positive-Magnitude constant force, Sawtooth Down rises (INR 29)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const int sg = fsign();
	uint64_t t0 = 0;
	const std::vector<Sample> up = runPeriodic(kSawUp, 10000, 0, 0, 100, 800, t0);
	REQUIRE(up.size() > 700);
	const Steps su = stepStats(xs(up, sg), 1500);
	INFO("sawtooth up: steps up " << su.pos << ", down " << su.neg);
	CHECK(su.pos == 0);
	CHECK(su.neg > 600);
	const std::vector<Sample> down = runPeriodic(kSawDown, 10000, 0, 0, 100, 800, t0);
	REQUIRE(down.size() > 700);
	const Steps sd = stepStats(xs(down, sg), 1500);
	INFO("sawtooth down: steps up " << sd.pos << ", down " << sd.neg);
	CHECK(sd.neg == 0);
	CHECK(sd.pos > 600);
	REQUIRE_CHAIN_MODELLED();
}

// characterization of the shapes PID does not define, from the firmware's formulas. INR 25: sine at t = P/12, P/4, 3P/4 is
// T * sin(30 deg) (T/2), T, -T (EffectsCalculator.cpp:300-306: offset + magnitude * sin(2 pi (t / P + phase / 35999))), in the
// sense of the positive-Magnitude constant force (the absolute sign is INR 10). Period 1200 ms (t = 100, 300, 900 ms) so the timing
// uncertainty kTimeMs costs little: tolerance at phase theta = T (w kTimeMs |cos theta| + (w kTimeMs)^2 / 2) + kBound(1) with w = 2 pi / P:
// 0.91 % of T at 30 degrees, 0.005 % at the extremes, plus the rounding bound (a sine 5 % low fails at every point).
TEST_CASE_FIXTURE(PerFixture, "characterization: [F1-E2] a sine effect follows sin(2 pi t / Period): T/2 at Period/12, T at Period/4 and -T at 3 Period/4 (INR 25)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int sg = fsign();
	const int16_t t = reference(10000);
	const double T = std::abs(static_cast<int>(t));
	uint64_t t0 = 0;
	const std::vector<Sample> s = runPeriodic(kSine, 10000, 0, 0, 1200, 1000, t0);
	const double wk = 2.0 * M_PI / 1200.0 * kTimeMs;
	auto tol = [&](double thetaDeg) { return T * (wk * std::fabs(std::cos(thetaDeg * M_PI / 180.0)) + wk * wk / 2.0) + kBound(1, sc); };
	CHECK_NEAR(sg * valueAt(s, t0, 100), T / 2, tol(30));
	CHECK_NEAR(sg * valueAt(s, t0, 300), T, tol(90));
	CHECK_NEAR(sg * valueAt(s, t0, 900), -T, tol(270));
	REQUIRE_CHAIN_MODELLED();
}

// characterization (INR 26): the triangle starts at its minimum -T, rises to +T at Period/2 and is back at -T after one Period
// (EffectsCalculator.cpp:240-262: force = slope * remainder + minMagnitude, mirrored after half the period).
TEST_CASE_FIXTURE(PerFixture, "characterization: [F1-E2] a triangle effect starts at -T, reaches +T at Period/2 and returns to -T at Period (INR 26)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int sg = fsign();
	const int16_t t = reference(10000);
	const double T = std::abs(static_cast<int>(t));
	uint64_t t0 = 0;
	const std::vector<Sample> s = runPeriodic(kTriangle, 10000, 0, 0, 100, 300, t0);
	const double tol = 2.0 * 2.0 * T / 100.0 * kTimeMs + kBound(1, sc);
	CHECK_NEAR(sg * s.front().torque, -T, tol);
	CHECK_NEAR(sg * valueAt(s, t0, 50), T, tol);
	CHECK_NEAR(sg * valueAt(s, t0, 100), -T, tol);
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-E10 Effect Operation (PID 5.9) and F1-E1 timing for periodic effects (PID 5.1)
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.9 Op Effect Start: "Start the effect": a defined effect gives no torque before it and its torque after.
TEST_CASE_FIXTURE(PerFixture, "F1-E10: Effect Operation Start starts a defined effect: no torque before it, the torque of the effect after" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t t = reference(10000);
	REQUIRE(t != 0);
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	setEffect(idx, pid::use::EtConstantForce, 9000);
	setConstant(idx, 10000);
	CHECK(settled(200) == 0);
	operation(idx, pid::use::OpEffectStart);
	CHECK_NEAR(settled(), t, kBound(1, sc));
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.9 Op Effect Stop: "Stop the effect": with two effects running, stopping one leaves the torque of the other; stopping both gives 0.
TEST_CASE_FIXTURE(PerFixture, "F1-E10: Effect Operation Stop of one of two running effects leaves the torque of the other" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t tA = reference(10000), tB = reference(20000);
	const int64_t a = startConstant(10000);
	const int64_t b = startConstant(20000);
	CHECK_NEAR(settled(), tA + tB, 2 * kBound(1, sc));
	operation(a, pid::use::OpEffectStop);
	CHECK_NEAR(settled(), tB, kBound(1, sc));
	operation(b, pid::use::OpEffectStop);
	CHECK(settled() == 0);
	REQUIRE_CHAIN_MODELLED();
}

// Starting an effect while another runs adds the torques (both are active: sum of the forces, PID 5.9 only says Start starts "the effect").
TEST_CASE_FIXTURE(PerFixture, "F1-E10: Effect Operation Start of a second effect while the first runs adds the two torques" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t tA = reference(10000), tB = reference(20000);
	startConstant(10000);
	CHECK_NEAR(settled(), tA, kBound(1, sc));
	startConstant(20000);
	CHECK_NEAR(settled(), tA + tB, 2 * kBound(1, sc));
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.9 Op Effect Start Solo: "Start the effect ... and stop all other effects": with A and B running (torque tA + tB), Start Solo of A gives
// tA alone, and B does not come back after A is stopped.
TEST_CASE_FIXTURE(PerFixture, "F1-E10: Effect Operation Start Solo leaves only the soloed effect, the others stay stopped" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t tA = reference(10000), tB = reference(20000);
	const int64_t a = startConstant(10000);
	startConstant(20000);
	CHECK_NEAR(settled(), tA + tB, 2 * kBound(1, sc));
	operation(a, pid::use::OpEffectStartSolo);
	CHECK_NEAR(settled(), tA, kBound(1, sc));
	operation(a, pid::use::OpEffectStop);
	CHECK(settled() == 0);
	REQUIRE_CHAIN_MODELLED();
}

// Start Solo of an effect that was not running: it starts and stops the running one (torque tB -> tA).
TEST_CASE_FIXTURE(PerFixture, "F1-E10: Effect Operation Start Solo of a stopped effect starts it and stops the running one" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t tA = reference(10000), tB = reference(20000);
	startConstant(20000);
	CHECK_NEAR(settled(), tB, kBound(1, sc));
	const int64_t a = newEffect(pid::use::EtConstantForce);
	setEffect(a, pid::use::EtConstantForce, 9000);
	setConstant(a, 10000);
	operation(a, pid::use::OpEffectStartSolo);
	CHECK_NEAR(settled(), tA, kBound(1, sc));
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.1 Duration for a periodic effect: it acts for Duration ms after the start and then ends (constant force: I1-3). Square, Magnitude
// 10000, Period 50, Duration 300: |torque| = T in [0, 299] ms, 0 from 301 ms.
TEST_CASE_FIXTURE(PerFixture, "F1-E10: a periodic effect with Duration 300 ms ends after 300 ms" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t t = reference(10000);
	const int64_t idx = makePeriodic(kSquare, 10000, 0, 0, 50, 300);
	std::vector<Sample> s;
	const uint64_t t0 = startAndTrace(idx, 700, s);
	int n = 0;
	CHECK(absNear(s, t0, 0, 300 - kEdgeMs, t, kBound(1, sc) + 1, n));
	CHECK(n > 250);
	CHECK(holds(s, t0, 300 + kEdgeMs, 690, 0, 0, n));
	CHECK(n > 350);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.1 Start Delay: "the start delay interval, for the playback of effect": a periodic effect with Start Delay 300 ms and Duration 500
// ms: 0 up to 299 ms, |T| from 301 to 799 ms (a delay does not shorten the duration: it counts from the end of the delay), 0 after.
TEST_CASE_FIXTURE(PerFixture, "F1-E10: a periodic effect with Start Delay 300 ms and Duration 500 ms acts from 300 ms to 800 ms after the start" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t t = reference(10000);
	const int64_t idx = makePeriodic(kSquare, 10000, 0, 0, 50, 500, 300);
	std::vector<Sample> s;
	const uint64_t t0 = startAndTrace(idx, 1100, s);
	int n = 0;
	CHECK(holds(s, t0, 0, 300 - kEdgeMs, 0, 0, n));
	CHECK(n > 250);
	CHECK(absNear(s, t0, 300 + kEdgeMs, 800 - kEdgeMs, t, kBound(1, sc) + 1, n));
	CHECK(n > 450);
	CHECK(holds(s, t0, 800 + kEdgeMs, 1090, 0, 0, n));
	CHECK(n > 250);
	REQUIRE_CHAIN_MODELLED();
}

// IB 27. PID 5.1 Start Delay: the delay applies to the playback of the effect, whatever its Duration. The firmware maps Duration 0 to INFINITE
// (HidFFB.cpp:341; the descriptor declares no Null state for Duration, usb_hid_ffb_desc.h:240-252, so an infinite Duration is reachable only
// through that firmware convention, as for IB 20) and EffectsCalculator.cpp:131-135 checks the start time only for a finite effect: an infinite
// effect acts at once.
// F1-B2 note (not tested here): on this path a periodic effect would be evaluated before its start time, so elapsed_time = micros() - startTime * 1000
// is negative and `uint32_t timeTemp = elapsed_time + phasetime * 1000` (EffectsCalculator.cpp:251-252 triangle, :274-275 saw up, :292-293 saw
// down) converts a negative float to uint32_t: out of range, undefined behaviour (x86 and ARM differ: ARM vcvt saturates to 0). micros() held in a
// float also loses 1 us resolution after about 16.7 s of uptime (2^24 us) and the 32-bit microsecond count wraps after about 71.6 min.
TEST_CASE_FIXTURE(PerFixture, "F1-E10: a constant force with Start Delay 300 ms and infinite Duration (0) starts 300 ms after the Start"
	* KNOWN_ISSUE("(IB 27) the start delay is ignored for an infinite effect (EffectsCalculator.cpp:131-135 skips the start time check)") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const int16_t t = reference(20000);
	REQUIRE(t != 0);
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	setEffect(idx, pid::use::EtConstantForce, 9000, 255, 0, 300);
	setConstant(idx, 20000);
	std::vector<Sample> s;
	const uint64_t t0 = startAndTrace(idx, 600, s);
	int n = 0;
	REQUIRE_CHAIN_MODELLED();
	CHECK(holds(s, t0, 0, 300 - kEdgeMs, 0, 0, n)); // the defect: the torque is already T
	CHECK(n > 250);
	CHECK(holds(s, t0, 300 + kEdgeMs, 590, t, kBound(1, scalerOf(5000)), n));
	CHECK(n > 250);
}

// IB 28 (seam S10). PID 5.9 Loop Count: "The number of times the device will repeat the operation. i.e. a Stop command is not required."
// The firmware never reads Loop Count (HidFFB.cpp:400-436 set_effect_operation uses only the index and the operation; the field exists only in
// the struct, ffb_defs.h:332). Intended (shape-free): Duration 200 ms with Loop Count 3 is still acting at 350 ms (inside the second repeat) and
// is over at 700 ms (after three repeats of 200 ms: 600 ms).
TEST_CASE_FIXTURE(PerFixture, "F1-E10: Effect Operation Start with Loop Count 3 repeats a 200 ms effect three times"
	* KNOWN_ISSUE("(IB 28) Loop Count of Effect Operation is not evaluated (HidFFB.cpp:400-436): the effect plays once") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const int16_t t = reference(20000);
	REQUIRE(t != 0);
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	setEffect(idx, pid::use::EtConstantForce, 9000, 255, 200);
	setConstant(idx, 20000);
	TorqueTrace trace(chip1);
	trace.start(*this);
	h().observe(&trace);
	const uint64_t t0 = HostPlatform::nowUs();
	operationLoop(idx, pid::use::OpEffectStart, 3);
	h().run(900);
	h().observe(nullptr);
	const std::vector<Sample> s = trace.samples();
	int n = 0;
	REQUIRE_CHAIN_MODELLED();
	CHECK(absNear(s, t0, 250, 550, t, kBound(1, scalerOf(5000)) + 1, n)); // the defect: 0 after 200 ms
	CHECK(n > 250);
	CHECK(holds(s, t0, 700, 890, 0, 0, n));
	CHECK(n > 150);
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-E11 Device Control over time (PID 5.13). Ramp 4000 -> 30000 over 4000 ms: T(4000) = 610 .. T(30000) = 4577, about 1 count per ms.
// ---------------------------------------------------------------------------------------------------------------------

namespace {
struct RampRefs {
	int16_t tS, tE;
	double slope; //!< counts per ms
	double at(double ms) const { return tS + (static_cast<double>(tE) - tS) * ms / 4000.0; }
};
} // namespace

#define RAMP_SETUP() \
	up(); \
	configure(); \
	const double sc = scalerOf(5000); \
	RampRefs rr; \
	rr.tS = reference(4000); \
	rr.tE = reference(30000); \
	REQUIRE(rr.tE != rr.tS); \
	rr.slope = std::fabs(static_cast<double>(rr.tE) - rr.tS) / 4000.0; \
	const double rampTol = kTimeMs * rr.slope + kBound(2, sc); \
	const int64_t ramp = newEffect(pid::use::EtRamp); \
	setEffect(ramp, pid::use::EtRamp, 9000, 255, 4000); \
	setRamp(ramp, 4000, 30000)

// PID 5.13 DC Disable Actuators: "Disable all the device actuators", DC Enable Actuators enables them: no torque while disabled, the ramp's
// torque is back after the Enable and stays within the ramp's range.
TEST_CASE_FIXTURE(PerFixture, "F1-E11: Disable Actuators gives torque 0 while a ramp runs and Enable Actuators gives the ramp torque back" * IT_DELETES_FFBHIDMAIN) {
	RAMP_SETUP();
	std::vector<Sample> all;
	const uint64_t t0 = startAndTrace(ramp, 1000, all);
	control(pid::use::DcDisableActuators);
	std::vector<Sample> part = run(500);
	all.insert(all.end(), part.begin(), part.end());
	control(pid::use::DcEnableActuators);
	part = run(1000);
	all.insert(all.end(), part.begin(), part.end());
	int n = 0;
	CHECK(holds(all, t0, 0, 1000 - 3, rr.at(500), rr.at(500) - rr.tS + rampTol, n)); // before: inside the ramp's range so far
	CHECK(holds(all, t0, 1000 + 2 * kTimeMs, 1500 - kTimeMs, 0, 0, n));
	CHECK(n > 400);
	for (const Sample &x : all) {
		const double t = tMs(x, t0);
		if (t >= 1500 + 2 * kTimeMs && t <= 2500) {
			CHECK(sgn(x.torque) == sgn(rr.tS));
			CHECK(std::abs(static_cast<int>(x.torque)) >= std::abs(static_cast<int>(rr.tS)) - rampTol);
			CHECK(std::abs(static_cast<int>(x.torque)) <= std::abs(static_cast<int>(rr.tE)) + rampTol);
		}
	}
	REQUIRE_CHAIN_MODELLED();
}

// characterization (INR 27): PID 5.13 says only "Disable all the device actuators", nothing about the effects' time. The firmware's
// effect time is wall time (HAL_GetTick, micros): after a 500 ms Disable at 1000 ms the ramp is at its value for 2000 ms (not 1500 ms).
TEST_CASE_FIXTURE(PerFixture, "characterization: [F1-E11] the effect time of a ramp continues while the actuators are disabled (INR 27)" * IT_DELETES_FFBHIDMAIN) {
	RAMP_SETUP();
	std::vector<Sample> all;
	const uint64_t t0 = startAndTrace(ramp, 1000, all);
	control(pid::use::DcDisableActuators);
	std::vector<Sample> part = run(500);
	all.insert(all.end(), part.begin(), part.end());
	control(pid::use::DcEnableActuators);
	part = run(1000);
	all.insert(all.end(), part.begin(), part.end());
	CHECK_NEAR(valueAt(all, t0, 2000), rr.at(2000), rampTol);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.13 DC Device Pause: "all effects are paused": a ramp paused at 1000 ms for 1000 ms continues from the value it had at 1000 ms after
// Device Continue and ends 1000 ms later (5000 ms after the start). Encoding: Figure 16 command numbers (owner IQ12). IB 8: under that
// encoding Continue (6 = flags 2 | 4) is read as Disable + Stop and the torque stays 0: the first failing assertion (torque after Continue != 0)
// is the IB 8 symptom; the time hold is asserted after it (and shown independent of IB 8 by the cases with the flag encoding below: IB 29).
TEST_CASE_FIXTURE(PerFixture, "F1-E11: Device Pause holds the effect time: a ramp paused for 1000 ms continues from its held value after Device Continue"
	* KNOWN_ISSUE("(IB 8) Device Continue under the PID 5.13 command numbering is decoded as bit flags 2 | 4 (Disable, Stop), the torque never comes back (HidFFB.cpp:221-239)") * IT_DELETES_FFBHIDMAIN) {
	RAMP_SETUP();
	std::vector<Sample> all;
	const uint64_t t0 = startAndTrace(ramp, 1000, all);
	control(pid::use::DcDevicePause);
	std::vector<Sample> part = run(1000);
	all.insert(all.end(), part.begin(), part.end());
	int n = 0;
	CHECK(holds(all, t0, 1000 + 2 * kTimeMs, 2000 - kTimeMs, 0, 0, n)); // paused: no torque (passes by the accident described in test_f1_hid.cpp)
	control(pid::use::DcDeviceContinue);
	part = run(1000);
	all.insert(all.end(), part.begin(), part.end());
	REQUIRE_CHAIN_MODELLED();
	CHECK(std::abs(static_cast<int>(valueAt(all, t0, 2500))) > 0); // IB 8: Continue gives no torque
	CHECK_NEAR(valueAt(all, t0, 2500), rr.at(1500), rampTol + 5);
}

// IB 29. The time hold with the flag encoding the firmware understands (Pause = 0x10, Continue = 0x20, sent raw), so IB 8 is out of the
// way: the torque comes back after Continue (passes), but the ramp has run on during the pause: at 2500 ms (500 ms after Continue) it is at
// its value for 2500 ms, not for the 1500 ms the held time gives. PID 5.13: Device Pause pauses all effects.
TEST_CASE_FIXTURE(PerFixture, "F1-E11: Device Pause holds the effect time (flag encoding): after Device Continue the ramp continues from its held value"
	* KNOWN_ISSUE("(IB 29) Device Pause only gates the output (stop_FFB): the effects' start times are not moved, the ramp runs on during the pause (HidFFB.cpp:221-239, EffectsCalculator.cpp:138)") * IT_DELETES_FFBHIDMAIN) {
	RAMP_SETUP();
	std::vector<Sample> all;
	const uint64_t t0 = startAndTrace(ramp, 1000, all);
	controlFlags(0x10);
	std::vector<Sample> part = run(1000);
	all.insert(all.end(), part.begin(), part.end());
	int n = 0;
	CHECK(holds(all, t0, 1000 + 2 * kTimeMs, 2000 - kTimeMs, 0, 0, n));
	CHECK(n > 900);
	controlFlags(0x20);
	part = run(1000);
	all.insert(all.end(), part.begin(), part.end());
	CHECK(std::abs(static_cast<int>(valueAt(all, t0, 2500))) > 0); // the torque is back after Continue
	REQUIRE_CHAIN_MODELLED();
	CHECK_NEAR(valueAt(all, t0, 2500), rr.at(1500), rampTol + 5); // the defect: the value is rr.at(2500)
}

// characterization (INR 28): with the flag encoding the observed behaviour of Pause / Continue: the ramp value at 500 ms after Continue is
// the value for the wall time (2500 ms), and the effect ends at its original end (4000 ms after the start), pause included.
TEST_CASE_FIXTURE(PerFixture, "characterization: [F1-E11] with the flag encoding the ramp continues at its wall-clock value after Continue and ends at 4000 ms (INR 28)" * IT_DELETES_FFBHIDMAIN) {
	RAMP_SETUP();
	std::vector<Sample> all;
	const uint64_t t0 = startAndTrace(ramp, 1000, all);
	controlFlags(0x10);
	std::vector<Sample> part = run(1000);
	all.insert(all.end(), part.begin(), part.end());
	controlFlags(0x20);
	part = run(3200);
	all.insert(all.end(), part.begin(), part.end());
	CHECK_NEAR(valueAt(all, t0, 2500), rr.at(2500), rampTol + 5);
	int n = 0;
	CHECK(holds(all, t0, 4000 + 2 * kTimeMs, 5100, 0, 0, n));
	CHECK(n > 900);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.13 DC Stop All Effects: "Issues a stop on every running effect": two running effects give torque 0, and 0 stays. Note: under the
// Figure 16 numbering this passes only because of IB 8: Stop All (3) is decoded as the flags 0x01 | 0x02 (Enable, then Disable), so the torque is
// 0 because the actuators are disabled, not because the effects were stopped (the 0x04 branch is never reached; see the IB 8 and IB 32 cases below).
TEST_CASE_FIXTURE(PerFixture, "F1-E11: Stop All Effects while two effects run gives torque 0" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	startConstant(10000);
	startConstant(20000);
	REQUIRE(settled() != 0);
	control(pid::use::DcStopAllEffects);
	CHECK(settled(300) == 0);
	CHECK(settled(500) == 0);
	REQUIRE_CHAIN_MODELLED();
}

// IB 9 with two effects: after Stop All a Start of effect A gives A's torque alone (PID 5.13: the device stays enabled; B, stopped by Stop All, stays
// stopped). Firmware: stop_FFB, the Start never re-enables (IB 9); the first assertion is the IB 9 symptom.
TEST_CASE_FIXTURE(PerFixture, "F1-E11: after Stop All Effects an Effect Operation Start of one of the two effects gives its torque alone"
	* KNOWN_ISSUE("(IB 9) Device Control Stop All Effects ends in stop_FFB and Effect Operation Start never re-enables the actuators (HidFFB.cpp:227-229, :400-436)") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	const double sc = scalerOf(5000);
	const int16_t tA = reference(10000);
	const int64_t a = startConstant(10000);
	startConstant(20000);
	REQUIRE(settled() != 0);
	control(pid::use::DcStopAllEffects);
	REQUIRE(settled(300) == 0);
	operation(a, pid::use::OpEffectStart);
	const int16_t v = settled();
	REQUIRE_CHAIN_MODELLED();
	CHECK_NEAR(v, tA, kBound(1, sc));
}

// IB 8 (cited). PID 5.13 DC Stop All Effects stops every running effect: after it and a DC Enable Actuators nothing plays. Under the Figure 16
// numbering the firmware receives 3 and decodes it as the flags 0x01 | 0x02 (HidFFB.cpp:221-239): Enable, then Disable; the Stop All branch (0x04)
// is never reached, the effects keep running behind the disabled output and Enable Actuators brings both back.
TEST_CASE_FIXTURE(PerFixture, "F1-E11: after Stop All Effects and Enable Actuators the stopped effects do not play"
	* KNOWN_ISSUE("(IB 8) Stop All Effects (3) is decoded as the flags 0x01 | 0x02, Enable then Disable (HidFFB.cpp:221-239): the 0x04 branch is never reached, the effects are not stopped and Enable Actuators resumes them") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	startConstant(10000);
	startConstant(20000);
	REQUIRE(settled() != 0);
	control(pid::use::DcStopAllEffects);
	REQUIRE(settled(300) == 0);
	control(pid::use::DcEnableActuators);
	const int16_t v = settled();
	REQUIRE_CHAIN_MODELLED();
	CHECK(v == 0);
}

// IB 32. The Stop All branch itself, reached with the flag encoding the firmware understands (0x04, then Enable = 0x01, sent raw, so IB 8 is
// out of the way). PID 5.13 DC Stop All Effects "Issues a stop on every running effect": the effects stay stopped until they are started again,
// so the Enable gives no torque. The firmware's 0x04 branch only calls stop_FFB (HidFFB.cpp:227-229, the start_FFB there is commented out):
// the effects keep state 1 and the Enable brings both back.
TEST_CASE_FIXTURE(PerFixture, "F1-E11: after Stop All Effects (flag encoding) and Enable Actuators the stopped effects do not play"
	* KNOWN_ISSUE("(IB 32) the Stop All branch of ffb_control only disables the output (stop_FFB, HidFFB.cpp:227-229): the effects are not stopped and Enable Actuators resumes them") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	startConstant(10000);
	startConstant(20000);
	const int16_t before = settled();
	REQUIRE(before != 0);
	controlFlags(0x04);
	REQUIRE(settled(300) == 0);
	controlFlags(0x01);
	const int16_t v = settled();
	MESSAGE("before " << before << ", after Stop All (0x04) and Enable (0x01) " << v);
	REQUIRE_CHAIN_MODELLED();
	CHECK(v == 0);
}

// IB 8 (cited): PID 5.13 DC Device Reset clears the effects from memory: after a Reset and an Enable Actuators the old effects do not play.
// Firmware under the Figure 16 numbering: Reset (4) is decoded as Stop (flag 4), reset_ffb is never called, the effects stay, and Enable brings
// the torque back.
TEST_CASE_FIXTURE(PerFixture, "F1-E11: after Device Reset and Enable Actuators the effects of before the reset do not play"
	* KNOWN_ISSUE("(IB 8) Device Reset is decoded as flag 4 (stop): reset_ffb is never called and the effects survive (HidFFB.cpp:221-239)") * IT_DELETES_FFBHIDMAIN) {
	up();
	configure();
	startConstant(10000);
	startConstant(20000);
	REQUIRE(settled() != 0);
	control(pid::use::DcDeviceReset);
	REQUIRE(settled(300) == 0);
	control(pid::use::DcEnableActuators);
	const int16_t v = settled();
	REQUIRE_CHAIN_MODELLED();
	CHECK(v == 0);
}

} // TEST_SUITE
