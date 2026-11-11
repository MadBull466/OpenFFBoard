/*
 * CMSIS-DSP Controller, PID (phase P11 group G2, work order CP): arm_pid_init_f32 / arm_pid_reset_f32 / arm_pid_f32,
 * arm_pid_init_q31 / arm_pid_reset_q31 / arm_pid_q31, arm_pid_init_q15 / arm_pid_reset_q15 / arm_pid_q15 (9 units).
 * Requirement ids: Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * ARM has NO test for the Controller module (no suite in Testing/desc.txt, no patterns), so there is no C1 here. The
 * oracle is DEF/DOC (C3, C4, C5): the difference equation quoted in Include/dsp/controller_functions.h ("PID" group):
 *     y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]
 *     A0 = Kp + Ki + Kd,  A1 = -Kp - 2 Kd,  A2 = Kd
 * and the documented state: S->state[0] = x[n-1], state[1] = x[n-2], state[2] = y[n-1] (from the "Update state" lines
 * of the process functions in the same header; the reset functions "clear the state array", the init functions
 * "Zero out the values in the state buffer" when resetStateFlag is 1 and leave it when it is 0).
 *
 * Two closed forms follow from the equation and serve as a second oracle (zero initial state, telescoping sums):
 *   - only Kp:  A0 = Kp, A1 = -Kp, A2 = 0:   y[n] = y[n-1] + Kp (x[n] - x[n-1])  =>  y[n] = Kp x[n]
 *   - only Ki:  A0 = Ki, A1 = A2 = 0:        y[n] = y[n-1] + Ki x[n]              (integrator)
 *   - only Kd:  A0 = Kd, A1 = -2Kd, A2 = Kd: y[n] = y[n-1] + Kd (x[n] - 2 x[n-1] + x[n-2])
 *                                                                              =>  y[n] = Kd (x[n] - x[n-1])
 *
 * Hand derived sequence used by the three types (gains Kp = 1/4, Ki = 1/8, Kd = 1/16 -> A0 = 7/16, A1 = -3/8 = -6/16,
 * A2 = 1/16; every number below is a binary fraction, so float, q31 and q15 are exact; inputs 1/2, 0, 0, 1/4, 0, 0):
 *   y0 = A0 * 1/2                                  = 7/32            = 0.21875
 *   y1 = y0 + A1 * 1/2                             = 7/32 - 6/32     = 0.03125
 *   y2 = y1 + A2 * 1/2 (x[n-2] = x0)               = 1/32 + 1/32     = 0.0625
 *   y3 = y2 + A0 * 1/4                             = 1/16 + 7/64     = 0.171875
 *   y4 = y3 + A1 * 1/4 (x[n-1] = x3, x[n-2] = x2 = 0) = 11/64 - 6/64 = 0.078125
 *   y5 = y4 + A2 * 1/4 (x[n-2] = x3)               = 5/64 + 1/64     = 0.09375
 *
 * Error bounds (derived where used):
 *   - f32: out = ((A0 in + A1 s0) + A2 s1) + s2 has 3 multiplications p0, p1, p2 and 3 additions; each operation has
 *     a relative error of at most 2^-24 of its result. The multiplications contribute at most 2^-24 (|p0| + |p1| +
 *     |p2|), the first two additions at most 2^-24 |p0 + p1| and 2^-24 |p0 + p1 + p2| (each at most sum|p|), the last
 *     one 2^-24 |out|. Per-step bound: 2^-24 (3 sum|p| + |out|). It is applied PER STEP and is not accumulated: each
 *     step is compared with the double recursion started from the library's own previous output (the library feeds
 *     its own y back, so the earlier errors are part of the input of this step). FMA contraction on the target only
 *     removes multiplication roundings. The factor 1.001 covers the difference between |out| and the reference.
 *   - q31: acc = A0 x + A1 x1 + A2 x2 is exact (2.62, 64 bit); out = floor(acc / 2^31) loses less than 1 LSB per step
 *     (the shift of a negative value rounds down), y[n-1] is added exactly, so after n steps the output is within n LSB
 *     (below the exact value, never above) of the exact recursion.
 *   - q15: acc (34.30) includes y[n-1] << 15 exactly, out = SSAT(acc >> 15): same argument, at most n LSB per n steps.
 *
 * Host vs target (C6): the host runs the portable C path (no ARM_MATH_DSP). On the target the q15 instance has a packed
 * A1 (q31, A1 | A2 as halves) and no A2 field, and the q15 process / init functions use SMUAD / SMLALD / QADD16; the
 * fields A1 / A2 of the q15 instance are therefore only read under `#if !defined(ARM_MATH_DSP)`. The q31 init uses
 * __QADD on the target (saturating, same result as clip_q63_to_q31 on the host). The f32 results use single precision
 * float; on the target FMA contraction may change the last bit, the bounds above cover it.
 * UB in vendor code (shown by the sanitizer build with --no-skip): arm_pid_q31 adds out += state[2] with signed
 * overflow (controller_functions.h:341, "out += y[n-1]") and arm_pid_init_q31 negates clip_q63_to_q31() which can be
 * INT32_MIN (arm_pid_init_q31.c:76). The cases that reach them are marked KNOWN_ISSUE_UB (B-DCP-1, B-DCP-2).
 * arm_pid_q15 left shifts a negative state[2] (`(q31_t) S->state[2] << 15`, controller_functions.h:393): that is
 * undefined in C but defined in C++20, and the inline function is compiled with the test file (C++20), so it is
 * neither reported by UBSan nor a defect of the build under test. The target compiles it as C (GNU C shifts
 * arithmetically), so it is not listed as a finding.
 */
#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math.h"

namespace {

const double kTwo31 = 2147483648.0;
const double kTwo15 = 32768.0;

int32_t q31From(double v) { return (int32_t)std::llround(v * kTwo31); }
int16_t q15From(double v) { return (int16_t)std::llround(v * kTwo15); }

// deterministic pseudo random values (no <random>: identical on every platform); one call per expression
uint32_t lcg(uint32_t &s) {
	s = s * 1664525u + 1013904223u;
	return s;
}

// uniform value in [-1, 1) with a step of 1/1024 (exactly representable in float, q31 and q15)
double randIn(uint32_t &s) {
	uint32_t r = lcg(s);
	int v = (int)((r >> 16) & 0x7FF) - 1024;
	return (double)v / 1024.0;
}

// one-sided floor error of the fixed point recursion after `steps` steps (header): exact - steps < out <= exact,
// with 0.5 LSB slack for the rounding of the double reference
bool oneSided(double out, double exact, int steps) { return out <= exact + 0.5 && out > exact - (double)steps - 0.5; }

arm_pid_instance_f32 makeF32(float kp, float ki, float kd) {
	arm_pid_instance_f32 s;
	std::memset(&s, 0, sizeof(s));
	s.Kp = kp;
	s.Ki = ki;
	s.Kd = kd;
	arm_pid_init_f32(&s, 1);
	return s;
}

arm_pid_instance_q31 makeQ31(double kp, double ki, double kd) {
	arm_pid_instance_q31 s;
	std::memset(&s, 0, sizeof(s));
	s.Kp = q31From(kp);
	s.Ki = q31From(ki);
	s.Kd = q31From(kd);
	arm_pid_init_q31(&s, 1);
	return s;
}

arm_pid_instance_q15 makeQ15(double kp, double ki, double kd) {
	arm_pid_instance_q15 s;
	std::memset(&s, 0, sizeof(s));
	s.Kp = q15From(kp);
	s.Ki = q15From(ki);
	s.Kd = q15From(kd);
	arm_pid_init_q15(&s, 1);
	return s;
}

// the hand derived sequence of the header comment
const double kSeqIn[6] = {0.5, 0.0, 0.0, 0.25, 0.0, 0.0};
const double kSeqOut[6] = {0.21875, 0.03125, 0.0625, 0.171875, 0.078125, 0.09375};

}  // namespace

TEST_SUITE("DSP Controller PID") {

// ---------------------------------------------------------------------------------------------------------------
// f32
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_pid_init_f32] the gains are derived as A0 = Kp + Ki + Kd, A1 = -Kp - 2 Kd, A2 = Kd") {
	// hand derivation: 1/4 + 1/8 + 1/16 = 7/16; -1/4 - 2/16 = -3/8; 1/16 (all exact in float)
	arm_pid_instance_f32 s = makeF32(0.25f, 0.125f, 0.0625f);
	CHECK(s.A0 == 0.4375f);
	CHECK(s.A1 == -0.375f);
	CHECK(s.A2 == 0.0625f);
}

TEST_CASE("[C3 arm_pid_init_f32] negative gains are derived with the right signs") {
	// Kp = -1.5, Ki = 2, Kd = -0.25: A0 = -1.5 + 2 - 0.25 = 0.25; A1 = 1.5 + 0.5 = 2; A2 = -0.25
	arm_pid_instance_f32 s = makeF32(-1.5f, 2.0f, -0.25f);
	CHECK(s.A0 == 0.25f);
	CHECK(s.A1 == 2.0f);
	CHECK(s.A2 == -0.25f);
}

TEST_CASE("[C3 arm_pid_init_f32] gains that are not binary fractions are derived within the rounding of the float sums") {
	// reference in double from the float gains; A0 has 2 float additions, A1 a multiplication by 2 (exact) and one
	// subtraction: each addition adds at most 2^-24 relative error of its result -> bound 2^-23 * (|Kp|+|Ki|+|Kd|) * 2
	const float kp = 0.1f, ki = 0.01f, kd = 0.001f;
	arm_pid_instance_f32 s = makeF32(kp, ki, kd);
	const double mag = std::fabs((double)kp) + std::fabs((double)ki) + std::fabs((double)kd);
	const double bound = 2.0 * std::ldexp(mag, -23);
	CHECK(std::fabs((double)s.A0 - ((double)kp + (double)ki + (double)kd)) <= bound);
	CHECK(std::fabs((double)s.A1 - (-(double)kp - 2.0 * (double)kd)) <= bound);
	CHECK(s.A2 == kd);
}

TEST_CASE("[C3 arm_pid_init_f32] the gains Kp, Ki and Kd are not modified") {
	arm_pid_instance_f32 s = makeF32(0.3f, 0.2f, 0.1f);
	CHECK(s.Kp == 0.3f);
	CHECK(s.Ki == 0.2f);
	CHECK(s.Kd == 0.1f);
}

TEST_CASE("[C4 arm_pid_init_f32] resetStateFlag 1 clears the state") {
	arm_pid_instance_f32 s = makeF32(0.25f, 0.125f, 0.0625f);
	s.state[0] = 1.0f;
	s.state[1] = 2.0f;
	s.state[2] = 3.0f;
	arm_pid_init_f32(&s, 1);
	CHECK(s.state[0] == 0.0f);
	CHECK(s.state[1] == 0.0f);
	CHECK(s.state[2] == 0.0f);
}

TEST_CASE("[C4 arm_pid_init_f32] resetStateFlag 0 keeps the state and re-derives the gains") {
	arm_pid_instance_f32 s = makeF32(0.25f, 0.125f, 0.0625f);
	s.state[0] = 1.0f;
	s.state[1] = 2.0f;
	s.state[2] = 3.0f;
	s.Kp = 0.5f;  // changed gain: A0 = 0.5 + 0.125 + 0.0625 = 0.6875, A1 = -0.5 - 0.125 = -0.625
	arm_pid_init_f32(&s, 0);
	CHECK(s.A0 == 0.6875f);
	CHECK(s.A1 == -0.625f);
	CHECK(s.A2 == 0.0625f);
	CHECK(s.state[0] == 1.0f);
	CHECK(s.state[1] == 2.0f);
	CHECK(s.state[2] == 3.0f);
}

TEST_CASE("[C4 arm_pid_reset_f32] the reset clears the three state values and keeps the gains") {
	arm_pid_instance_f32 s = makeF32(0.25f, 0.125f, 0.0625f);
	s.state[0] = 1.0f;
	s.state[1] = -2.0f;
	s.state[2] = 3.0f;
	arm_pid_reset_f32(&s);
	CHECK(s.state[0] == 0.0f);
	CHECK(s.state[1] == 0.0f);
	CHECK(s.state[2] == 0.0f);
	CHECK(s.A0 == 0.4375f);
	CHECK(s.A1 == -0.375f);
	CHECK(s.A2 == 0.0625f);
}

TEST_CASE("[C3 arm_pid_f32] the hand derived sequence of the header comment is reproduced exactly") {
	arm_pid_instance_f32 s = makeF32(0.25f, 0.125f, 0.0625f);
	for (int i = 0; i < 6; i++) {
		CAPTURE(i);
		CHECK((double)arm_pid_f32(&s, (float)kSeqIn[i]) == kSeqOut[i]);
	}
}

TEST_CASE("[C3 arm_pid_f32] with only Kp the output is Kp times the input") {
	arm_pid_instance_f32 s = makeF32(0.5f, 0.0f, 0.0f);
	uint32_t seed = 11;
	for (int i = 0; i < 20; i++) {
		float x = (float)randIn(seed);
		CAPTURE(i);
		CHECK(arm_pid_f32(&s, x) == 0.5f * x);
	}
}

TEST_CASE("[C3 arm_pid_f32] with only Ki the output is the running sum of Ki times the input") {
	arm_pid_instance_f32 s = makeF32(0.0f, 0.25f, 0.0f);
	double sum = 0.0;
	uint32_t seed = 12;
	for (int i = 0; i < 20; i++) {
		float x = (float)randIn(seed);
		sum += 0.25 * (double)x;  // exact in double and in float (multiples of 2^-12, at most 20 terms)
		CAPTURE(i);
		CHECK((double)arm_pid_f32(&s, x) == sum);
	}
}

TEST_CASE("[C3 arm_pid_f32] with only Kd the output is Kd times the difference of the last two inputs") {
	arm_pid_instance_f32 s = makeF32(0.0f, 0.0f, 0.5f);
	float prev = 0.0f;  // zero initial state: x[-1] = 0
	uint32_t seed = 13;
	for (int i = 0; i < 20; i++) {
		float x = (float)randIn(seed);
		CAPTURE(i);
		CHECK(arm_pid_f32(&s, x) == 0.5f * (x - prev));
		prev = x;
	}
}

TEST_CASE("[C4 arm_pid_f32] the state holds x[n-1], x[n-2] and y[n-1] after each call") {
	arm_pid_instance_f32 s = makeF32(0.25f, 0.125f, 0.0625f);
	float y0 = arm_pid_f32(&s, 0.5f);
	CHECK(s.state[0] == 0.5f);
	CHECK(s.state[1] == 0.0f);
	CHECK(s.state[2] == y0);
	float y1 = arm_pid_f32(&s, 0.25f);
	CHECK(s.state[0] == 0.25f);
	CHECK(s.state[1] == 0.5f);
	CHECK(s.state[2] == y1);
}

TEST_CASE("[C4 arm_pid_f32] a zero input keeps the integrator output constant (y[n-1] is carried)") {
	arm_pid_instance_f32 s = makeF32(0.0f, 0.5f, 0.0f);
	CHECK(arm_pid_f32(&s, 0.5f) == 0.25f);
	CHECK(arm_pid_f32(&s, 0.0f) == 0.25f);
	CHECK(arm_pid_f32(&s, 0.0f) == 0.25f);
}

TEST_CASE("[C4 arm_pid_f32] 200 consecutive samples agree step by step with the double recursion within the per-step rounding bound") {
	arm_pid_instance_f32 s = makeF32(0.3f, 0.07f, 0.02f);
	const double a0 = s.A0, a1 = s.A1, a2 = s.A2;  // verified against DEF in the init cases
	double x1 = 0.0, x2 = 0.0, y = 0.0;
	uint32_t seed = 21;
	bool ok = true;
	for (int n = 0; n < 200 && ok; n++) {
		float x = (float)randIn(seed);
		// reference: one step of the recursion in double from the library's previous output y
		double yn = y + a0 * x + a1 * x1 + a2 * x2;
		// per-step bound 2^-24 (3 sum|p| + |out|) (header comment), not accumulated
		const double err = 1.001 * std::ldexp(3.0 * (std::fabs(a0 * x) + std::fabs(a1 * x1) + std::fabs(a2 * x2)) + std::fabs(yn), -24);
		float out = arm_pid_f32(&s, x);
		if (!(std::fabs((double)out - yn) <= err)) {
			CAPTURE(n);
			CAPTURE(out);
			CAPTURE(yn);
			CAPTURE(err);
			CHECK(std::fabs((double)out - yn) <= err);
			ok = false;
		}
		x2 = x1;
		x1 = x;
		y = (double)out;  // the library feeds back its own output: the next step starts from it
	}
	CHECK(ok);
}

TEST_CASE("[C4 arm_pid_f32] after a reset the controller behaves like a new one") {
	arm_pid_instance_f32 s = makeF32(0.25f, 0.125f, 0.0625f);
	for (int i = 0; i < 6; i++) arm_pid_f32(&s, (float)kSeqIn[i]);
	arm_pid_reset_f32(&s);
	for (int i = 0; i < 6; i++) {
		CAPTURE(i);
		CHECK((double)arm_pid_f32(&s, (float)kSeqIn[i]) == kSeqOut[i]);
	}
}

TEST_CASE("[C4 arm_pid_f32] without a reset the state of the previous run continues (the next output differs from a fresh run)") {
	arm_pid_instance_f32 s = makeF32(0.25f, 0.125f, 0.0625f);
	for (int i = 0; i < 6; i++) arm_pid_f32(&s, (float)kSeqIn[i]);
	// continuing: y[n-1] = 0.09375, x[n-1] = x5 = 0, x[n-2] = x4 = 0, x = 0.5:
	// y = 0.09375 + 7/16*0.5 = 0.09375 + 0.21875 = 0.3125
	CHECK((double)arm_pid_f32(&s, 0.5f) == 0.3125);
}

TEST_CASE("[C4 arm_pid_f32] two instances do not share state") {
	arm_pid_instance_f32 a = makeF32(0.25f, 0.125f, 0.0625f);
	arm_pid_instance_f32 b = makeF32(0.25f, 0.125f, 0.0625f);
	arm_pid_f32(&a, 0.5f);
	arm_pid_f32(&a, 0.5f);
	CHECK((double)arm_pid_f32(&b, 0.5f) == kSeqOut[0]);
}

// ---------------------------------------------------------------------------------------------------------------
// q31
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_pid_init_q31] the gains are derived as A0 = Kp + Ki + Kd, A1 = -Kp - 2 Kd, A2 = Kd") {
	// 1/4 + 1/8 + 1/16 = 7/16 -> 7 * 2^27; -(1/4 + 2/16) = -3/8 -> -3 * 2^28; 1/16 -> 2^27
	arm_pid_instance_q31 s = makeQ31(0.25, 0.125, 0.0625);
	CHECK(s.A0 == 7 * 134217728);
	CHECK(s.A1 == -3 * 268435456);
	CHECK(s.A2 == 134217728);
}

TEST_CASE("[C3 arm_pid_init_q31] negative gains are derived with the right signs") {
	// Kp = -0.5, Ki = 0.25, Kd = -0.125: A0 = -0.375; A1 = 0.5 + 0.25 = 0.75; A2 = -0.125
	arm_pid_instance_q31 s = makeQ31(-0.5, 0.25, -0.125);
	CHECK(s.A0 == q31From(-0.375));
	CHECK(s.A1 == q31From(0.75));
	CHECK(s.A2 == q31From(-0.125));
}

TEST_CASE("[C4 arm_pid_init_q31] resetStateFlag 1 clears the state and 0 keeps it") {
	arm_pid_instance_q31 s = makeQ31(0.25, 0.125, 0.0625);
	s.state[0] = 11;
	s.state[1] = -22;
	s.state[2] = 33;
	s.Kp = q31From(0.5);  // changed gain: A0 = 0.5 + 0.125 + 0.0625 = 0.6875, A1 = -(0.5 + 0.125) = -0.625
	arm_pid_init_q31(&s, 0);
	CHECK(s.A0 == q31From(0.6875));
	CHECK(s.A1 == q31From(-0.625));
	CHECK(s.A2 == q31From(0.0625));
	CHECK(s.state[0] == 11);
	CHECK(s.state[1] == -22);
	CHECK(s.state[2] == 33);
	arm_pid_init_q31(&s, 1);
	CHECK(s.state[0] == 0);
	CHECK(s.state[1] == 0);
	CHECK(s.state[2] == 0);
}

TEST_CASE("[C4 arm_pid_reset_q31] the reset clears the three state values and keeps the gains") {
	arm_pid_instance_q31 s = makeQ31(0.25, 0.125, 0.0625);
	s.state[0] = 11;
	s.state[1] = -22;
	s.state[2] = 33;
	arm_pid_reset_q31(&s);
	CHECK(s.state[0] == 0);
	CHECK(s.state[1] == 0);
	CHECK(s.state[2] == 0);
	CHECK(s.A0 == 7 * 134217728);
	CHECK(s.Kp == q31From(0.25));
}

TEST_CASE("[C4 arm_pid_init_q31] characterization: a sum of gains above 1.0 saturates A0 and A1 (the header does not document it)") {
	// Kp = Ki = Kd = 0.75: A0 = 2.25 -> 0x7FFFFFFF; A1 = -(1.5 + 0.75) = -(clip(2.25)) = -0x7FFFFFFF; A2 = Kd.
	// Well-defined, not suspicious (the clip is explicit in arm_pid_init_q31.c:71-76); NEEDS REVIEW item.
	arm_pid_instance_q31 s = makeQ31(0.75, 0.75, 0.75);
	CHECK(s.A0 == INT32_MAX);
	CHECK(s.A1 == -INT32_MAX);
	CHECK(s.A2 == q31From(0.75));
}

TEST_CASE("[C4 arm_pid_init_q31] a sum of gains below -1.0 saturates A1 to the largest positive value" * KNOWN_ISSUE_UB("B-DCP-1: arm_pid_init_q31.c:76 negates clip_q63_to_q31(), which is INT32_MIN here: -INT32_MIN overflows (UB) and stays INT32_MIN, a negative A1 for A1 = -Kp - 2 Kd = +2.25; the Q15 init saturates the same case to +32767")) {
	// Kp = Kd = -0.75: Kd + Kd + Kp = -2.25 -> clipped to INT32_MIN; A1 = -(-2.25) = +2.25, saturated: INT32_MAX
	arm_pid_instance_q31 s = makeQ31(-0.75, 0.0, -0.75);
	CHECK(s.A1 == INT32_MAX);
}

TEST_CASE("[C3 arm_pid_q31] the hand derived sequence of the header comment is reproduced exactly") {
	arm_pid_instance_q31 s = makeQ31(0.25, 0.125, 0.0625);
	for (int i = 0; i < 6; i++) {
		CAPTURE(i);
		CHECK(arm_pid_q31(&s, q31From(kSeqIn[i])) == q31From(kSeqOut[i]));
	}
}

TEST_CASE("[C3 arm_pid_q31] with only Kp the output is Kp times the input within n LSB and never above") {
	// y[n] = Kp x[n]; the exact product 0.5 * x is an integer for even x, otherwise the floor loses 0.5 LSB per step;
	// the floor error is one-sided: exact - (i + 1) < out <= exact (header), checked with 0.5 LSB slack for the double
	arm_pid_instance_q31 s = makeQ31(0.5, 0.0, 0.0);
	uint32_t seed = 31;
	for (int i = 0; i < 20; i++) {
		int32_t x = q31From(randIn(seed));
		int32_t out = arm_pid_q31(&s, x);
		CAPTURE(i);
		CHECK(oneSided((double)out, 0.5 * (double)x, i + 1));
	}
}

TEST_CASE("[C3 arm_pid_q31] with only Ki the output is the running sum of Ki times the input within n LSB") {
	arm_pid_instance_q31 s = makeQ31(0.0, 0.0625, 0.0);
	double sum = 0.0;
	uint32_t seed = 32;
	for (int i = 0; i < 20; i++) {
		int32_t x = q31From(randIn(seed));
		sum += 0.0625 * (double)x;
		int32_t out = arm_pid_q31(&s, x);
		CAPTURE(i);
		CHECK(oneSided((double)out, sum, i + 1));
	}
}

TEST_CASE("[C4 arm_pid_q31] the state holds x[n-1], x[n-2] and y[n-1] after each call") {
	arm_pid_instance_q31 s = makeQ31(0.25, 0.125, 0.0625);
	int32_t x0 = q31From(0.5), x1 = q31From(0.25);
	int32_t y0 = arm_pid_q31(&s, x0);
	CHECK(s.state[0] == x0);
	CHECK(s.state[1] == 0);
	CHECK(s.state[2] == y0);
	int32_t y1 = arm_pid_q31(&s, x1);
	CHECK(s.state[0] == x1);
	CHECK(s.state[1] == x0);
	CHECK(s.state[2] == y1);
}

TEST_CASE("[C4 arm_pid_q31] 200 consecutive samples agree with the exact recursion within n LSB") {
	arm_pid_instance_q31 s = makeQ31(0.2, 0.05, 0.03);
	const double a0 = s.A0, a1 = s.A1, a2 = s.A2;  // integers; A0..A2 verified against DEF in the init cases
	double x1 = 0.0, x2 = 0.0, y = 0.0;  // exact recursion in units of LSB (double holds the 2^62 products with < 1e-3 LSB error)
	uint32_t seed = 33;
	bool ok = true;
	for (int n = 0; n < 200 && ok; n++) {
		int32_t x = q31From(randIn(seed));
		y = y + (a0 * x + a1 * x1 + a2 * x2) / kTwo31;
		int32_t out = arm_pid_q31(&s, x);
		// header: floor loses < 1 LSB per step and never rounds up: exact - (n + 1) < out <= exact
		if (!oneSided((double)out, y, n + 1)) {
			CAPTURE(n);
			CAPTURE(out);
			CAPTURE(y);
			CHECK(oneSided((double)out, y, n + 1));
			ok = false;
		}
		x2 = x1;
		x1 = x;
	}
	CHECK(ok);
}

TEST_CASE("[C4 arm_pid_q31] after a reset the controller behaves like a new one") {
	arm_pid_instance_q31 s = makeQ31(0.25, 0.125, 0.0625);
	for (int i = 0; i < 6; i++) arm_pid_q31(&s, q31From(kSeqIn[i]));
	arm_pid_reset_q31(&s);
	for (int i = 0; i < 6; i++) {
		CAPTURE(i);
		CHECK(arm_pid_q31(&s, q31From(kSeqIn[i])) == q31From(kSeqOut[i]));
	}
}

TEST_CASE("[C4 arm_pid_q31] an integrator that exceeds +1.0 saturates to 0x7FFFFFFF" * KNOWN_ISSUE_UB("B-DCP-2: arm_pid_q31 (controller_functions.h) adds out += y[n-1] in 32 bit with signed overflow (UB) and truncates (q31_t)(acc >> 31) without saturation; DOC lines 317 and 319 differ: the accumulator wraps (317), but it is truncated to 1.32 and then saturated to 1.31 (319); the sum with y[n-1] is not saturated, the result wraps to a negative value")) {
	// Ki = 0.75, input 0.5 each step: y0 = 0.375, y1 = 0.75, y2 = 1.125 -> DOC: saturated to 0x7FFFFFFF
	arm_pid_instance_q31 s = makeQ31(0.0, 0.75, 0.0);
	const int32_t x = q31From(0.5);
	CHECK(arm_pid_q31(&s, x) == q31From(0.375));
	CHECK(arm_pid_q31(&s, x) == q31From(0.75));
	CHECK(arm_pid_q31(&s, x) == INT32_MAX);
}

TEST_CASE("[C4 arm_pid_q31] an integrator that falls below -1.0 saturates to 0x80000000" * KNOWN_ISSUE_UB("B-DCP-2: arm_pid_q31 (controller_functions.h) adds out += y[n-1] in 32 bit with signed overflow (UB) and truncates (q31_t)(acc >> 31) without saturation; DOC lines 317 and 319 differ: the accumulator wraps (317), but it is truncated to 1.32 and then saturated to 1.31 (319); the sum with y[n-1] is not saturated, the result wraps to a positive value")) {
	arm_pid_instance_q31 s = makeQ31(0.0, 0.75, 0.0);
	const int32_t x = q31From(-0.5);
	CHECK(arm_pid_q31(&s, x) == q31From(-0.375));
	CHECK(arm_pid_q31(&s, x) == q31From(-0.75));
	CHECK(arm_pid_q31(&s, x) == INT32_MIN);
}

TEST_CASE("[C4 arm_pid_q31] an accumulator beyond -1.0 in 1.32 format saturates to 0x80000000 instead of wrapping" * KNOWN_ISSUE("B-DCP-2b: arm_pid_q31 casts (q31_t)(acc >> 31) without saturation (implementation-defined wrap, no UB); DOC (controller_functions.h:319) says the 2.62 accumulator is truncated to 1.32 format and then saturated to 1.31 format, the result here wraps to +0.29")) {
	// Kp = Kd = 0.45: A0 = 0.9, A1 = -(0.45 + 0.9) = -1.35 -> clipped to -0.99999999953, A2 = 0.45.
	// State x[n-1] = 0.9, x[n-2] = 0, y[n-1] = 0, input -0.9: acc = 0.9 * -0.9 + A1 * 0.9 = -0.81 - 0.9 = -1.71 (2.62
	// format, no wrap of the accumulator). DOC: saturated to 1.31 -> INT32_MIN; the cast wraps to -1.71 + 2 = +0.29.
	arm_pid_instance_q31 s = makeQ31(0.45, 0.0, 0.45);
	s.state[0] = q31From(0.9);
	CHECK(arm_pid_q31(&s, q31From(-0.9)) == INT32_MIN);
}

TEST_CASE("[C3 arm_pid_init_q31] in-range gains whose partial sums leave the range still give the exact coefficients" * KNOWN_ISSUE("B-DCP-5: arm_pid_init_q31 (host path, arm_pid_init_q31.c:71-76) clips each partial sum: for Kp = 0.75, Ki = 0.75, Kd = -0.75 clip(Kp + Ki) = 0.99999 and A0 = 0.99999 - 0.75 = 0.25 instead of 0.75; A1 = -clip(clip(Kd + Kd) + Kp) = -(-1.0 + 0.75) = 0.25 instead of 0.75 (the q15 init gives 24576 = 0.75 for A0)")) {
	// exact: A0 = Kp + Ki + Kd = 0.75 + 0.75 - 0.75 = 0.75; A1 = -Kp - 2 Kd = -0.75 + 1.5 = 0.75; both in range
	arm_pid_instance_q31 s = makeQ31(0.75, 0.75, -0.75);
	CHECK(s.A0 == q31From(0.75));
	CHECK(s.A1 == q31From(0.75));
	CHECK(s.A2 == q31From(-0.75));
}

// ---------------------------------------------------------------------------------------------------------------
// q15
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_pid_init_q15] the gains are derived as A0 = Kp + Ki + Kd, A1 = -Kp - 2 Kd, A2 = Kd") {
	// 1/4 + 1/8 + 1/16 = 7/16 -> 14336; -(1/4 + 2/16) = -3/8 -> -12288; 1/16 -> 2048 (host path, see header)
	arm_pid_instance_q15 s = makeQ15(0.25, 0.125, 0.0625);
	CHECK(s.A0 == 14336);
#if !defined(ARM_MATH_DSP)
	CHECK(s.A1 == -12288);
	CHECK(s.A2 == 2048);
#endif
}

TEST_CASE("[C4 arm_pid_init_q15] resetStateFlag 1 clears the state and 0 keeps it") {
	arm_pid_instance_q15 s = makeQ15(0.25, 0.125, 0.0625);
	s.state[0] = 11;
	s.state[1] = -22;
	s.state[2] = 33;
	s.Kp = q15From(0.5);  // changed gain: A0 = 0.5 + 0.125 + 0.0625 = 0.6875 (22528), A1 = -0.625 (-20480)
	arm_pid_init_q15(&s, 0);
	CHECK(s.A0 == q15From(0.6875));
#if !defined(ARM_MATH_DSP)
	CHECK(s.A1 == q15From(-0.625));
	CHECK(s.A2 == q15From(0.0625));
#endif
	CHECK(s.state[0] == 11);
	CHECK(s.state[1] == -22);
	CHECK(s.state[2] == 33);
	arm_pid_init_q15(&s, 1);
	CHECK(s.state[0] == 0);
	CHECK(s.state[1] == 0);
	CHECK(s.state[2] == 0);
}

TEST_CASE("[C4 arm_pid_reset_q15] the reset clears the three state values and keeps the gains") {
	arm_pid_instance_q15 s = makeQ15(0.25, 0.125, 0.0625);
	s.state[0] = 11;
	s.state[1] = -22;
	s.state[2] = 33;
	arm_pid_reset_q15(&s);
	CHECK(s.state[0] == 0);
	CHECK(s.state[1] == 0);
	CHECK(s.state[2] == 0);
	CHECK(s.A0 == 14336);
	CHECK(s.Kp == 8192);
}

TEST_CASE("[C4 arm_pid_init_q15] characterization: a sum of gains beyond +-1.0 saturates A0 and A1 to 16 bit (the header does not document it)") {
	// Kp = Ki = Kd = 0.75 (24576): A0 = 2.25 -> 32767; A1 = -(1.5 + 0.75) = -2.25 -> -32768; A2 = Kd.
	// Well-defined (__SSAT in arm_pid_init_q15.c); NEEDS REVIEW item.
	arm_pid_instance_q15 s = makeQ15(0.75, 0.75, 0.75);
	CHECK(s.A0 == 32767);
#if !defined(ARM_MATH_DSP)
	CHECK(s.A1 == -32768);
	CHECK(s.A2 == 24576);
#endif
}

TEST_CASE("[C3 arm_pid_q15] the hand derived sequence of the header comment is reproduced exactly") {
	arm_pid_instance_q15 s = makeQ15(0.25, 0.125, 0.0625);
	for (int i = 0; i < 6; i++) {
		CAPTURE(i);
		CHECK(arm_pid_q15(&s, q15From(kSeqIn[i])) == q15From(kSeqOut[i]));
	}
}

TEST_CASE("[C3 arm_pid_q15] with only Ki and positive inputs the output is the running sum within n LSB") {
	arm_pid_instance_q15 s = makeQ15(0.0, 0.0625, 0.0);
	double sum = 0.0;
	uint32_t seed = 41;
	for (int i = 0; i < 20; i++) {
		int16_t x = q15From(std::fabs(randIn(seed)) * 0.5);  // 0 .. 0.5, so the sum stays < 1 and positive
		sum += 0.0625 * (double)x;
		int16_t out = arm_pid_q15(&s, x);
		CAPTURE(i);
		CHECK(oneSided((double)out, sum, i + 1));
	}
}

TEST_CASE("[C4 arm_pid_q15] the state holds x[n-1], x[n-2] and y[n-1] after each call") {
	arm_pid_instance_q15 s = makeQ15(0.25, 0.125, 0.0625);
	int16_t x0 = q15From(0.5), x1 = q15From(0.25);
	int16_t y0 = arm_pid_q15(&s, x0);
	CHECK(s.state[0] == x0);
	CHECK(s.state[1] == 0);
	CHECK(s.state[2] == y0);
	int16_t y1 = arm_pid_q15(&s, x1);
	CHECK(s.state[0] == x1);
	CHECK(s.state[1] == x0);
	CHECK(s.state[2] == y1);
}

TEST_CASE("[C4 arm_pid_q15] after a reset the controller behaves like a new one") {
	arm_pid_instance_q15 s = makeQ15(0.25, 0.125, 0.0625);
	for (int i = 0; i < 6; i++) arm_pid_q15(&s, q15From(kSeqIn[i]));
	arm_pid_reset_q15(&s);
	for (int i = 0; i < 6; i++) {
		CAPTURE(i);
		CHECK(arm_pid_q15(&s, q15From(kSeqIn[i])) == q15From(kSeqOut[i]));
	}
}

TEST_CASE("[C4 arm_pid_q15] an integrator that exceeds +1.0 saturates to 0x7FFF and stays there") {
	// DOC: "the accumulator is saturated to yield a result in 1.15 format". Ki = 0.75, input 0.5: 0.375, 0.75, 1.125.
	arm_pid_instance_q15 s = makeQ15(0.0, 0.75, 0.0);
	const int16_t x = q15From(0.5);
	CHECK(arm_pid_q15(&s, x) == q15From(0.375));
	CHECK(arm_pid_q15(&s, x) == q15From(0.75));
	CHECK(arm_pid_q15(&s, x) == 32767);
	CHECK(arm_pid_q15(&s, x) == 32767);
}

TEST_CASE("[C4 arm_pid_q15] an integrator that falls below -1.0 saturates to 0x8000") {
	arm_pid_instance_q15 s = makeQ15(0.0, 0.75, 0.0);
	const int16_t x = q15From(-0.5);
	CHECK(arm_pid_q15(&s, x) == q15From(-0.375));
	CHECK(arm_pid_q15(&s, x) == q15From(-0.75));
	CHECK(arm_pid_q15(&s, x) == -32768);
	CHECK(arm_pid_q15(&s, x) == -32768);
}

TEST_CASE("[C4 arm_pid_q15] 200 consecutive signed samples agree with the exact recursion within n LSB") {
	// small gains and inputs keep |y| well below 1, so saturation is not involved: A0 = 0.0625+... see the instance
	arm_pid_instance_q15 s = makeQ15(0.0625, 0.015625, 0.0078125);
	const double a0 = s.A0;
#if !defined(ARM_MATH_DSP)
	const double a1 = s.A1, a2 = s.A2;
#else
	const double a1 = 0.0, a2 = 0.0;
#endif
	double x1 = 0.0, x2 = 0.0, y = 0.0;
	uint32_t seed = 42;
	bool ok = true;
	for (int n = 0; n < 200 && ok; n++) {
		int16_t x = q15From(randIn(seed) * 0.25);
		y = y + (a0 * x + a1 * x1 + a2 * x2) / kTwo15;
		int16_t out = arm_pid_q15(&s, x);
		// header: floor loses < 1 LSB per step and never rounds up: exact - (n + 1) < out <= exact
		if (!oneSided((double)out, y, n + 1)) {
			CAPTURE(n);
			CAPTURE(out);
			CAPTURE(y);
			CHECK(oneSided((double)out, y, n + 1));
			ok = false;
		}
		x2 = x1;
		x1 = x;
	}
	CHECK(ok);
}

// ---------------------------------------------------------------------------------------------------------------
// C5: the fixed point controllers agree with the f32 controller
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C5 arm_pid_q31] 30 samples agree with arm_pid_f32 of the same gains and inputs within n LSB") {
	// gains and inputs are multiples of 2^-10 / 2^-6: exact in float and in q31; the float recursion is exact too
	// (all terms are multiples of 2^-20 and below 2, 24 bit mantissa); the q31 error is below n LSB (header)
	arm_pid_instance_f32 f = makeF32(0.25f, 0.125f, 0.0625f);
	arm_pid_instance_q31 q = makeQ31(0.25, 0.125, 0.0625);
	uint32_t seed = 51;
	for (int n = 0; n < 30; n++) {
		double x = randIn(seed) * 0.25;
		float yf = arm_pid_f32(&f, (float)x);
		int32_t yq = arm_pid_q31(&q, q31From(x));
		CAPTURE(n);
		CHECK(std::fabs((double)yq / kTwo31 - (double)yf) <= (double)(n + 1) / kTwo31);
	}
}

TEST_CASE("[C5 arm_pid_q15] 30 samples agree with arm_pid_f32 of the same gains and inputs within n LSB") {
	// same argument; gains 2^-4 .. 2^-6 and inputs of at most 0.25 keep |y| < 1; the float recursion is exact
	arm_pid_instance_f32 f = makeF32(0.0625f, 0.03125f, 0.015625f);
	arm_pid_instance_q15 q = makeQ15(0.0625, 0.03125, 0.015625);
	uint32_t seed = 52;
	for (int n = 0; n < 30; n++) {
		double x = (double)q15From(randIn(seed) * 0.25) / kTwo15;
		float yf = arm_pid_f32(&f, (float)x);
		int16_t yq = arm_pid_q15(&q, q15From(x));
		CAPTURE(n);
		CHECK(std::fabs((double)yq / kTwo15 - (double)yf) <= (double)(n + 1) / kTwo15);
	}
}

}  // TEST_SUITE
