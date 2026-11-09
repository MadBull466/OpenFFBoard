/*
 * CMSIS-DSP Controller, sin_cos (phase P11 group G2, work order CP): arm_sin_cos_f32 and arm_sin_cos_q31 (2 units).
 * Requirement ids: Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * ARM has NO test for these functions (no suite in Testing/desc.txt, no patterns), so there is no C1. The oracle is
 * DEF/DOC (C3, C4, C5). DOC (Include/dsp/controller_functions.h, group "SinCos"): "The input to the floating-point
 * version is in degrees while the fixed-point Q31 have a scaled input with the range [-1 0.9999] mapping to
 * [-180 +180] degrees." "The floating point function also allows values that are out of the usual range. When this
 * happens, the function will take extra time to adjust the input value to the range of [-180 180]." "The result is
 * accurate to 5 digits after the decimal point." (arm_sin_cos_q31.c: "The Q31 input value is in the range
 * [-1 0.999999] and is mapped to a degree value in the range [-180 179].") The DOC text says "linear interpolation"
 * but the code evaluates a cubic Hermite polynomial on a 512 interval table; the DEF checks are independent of that.
 *
 * DEF: f32: sin(theta * pi / 180), cos(theta * pi / 180) in double from the float input value.
 *      q31: angle = x / 2^31 * pi (x = -2^31 is -180 degrees), results scaled by 2^31 and clipped to the q31 range.
 *
 * Error bounds (derived here, used in the cases):
 *   - f32 (a function of theta, B(theta)): the table values are rounded to float (6e-8), the Hermite interpolation
 *     error is h^4/384 * max|f''''| with h = 2 pi / 512, i.e. 5.9e-11 (negligible), the float evaluation of the
 *     polynomial adds below 1e-7 (terms up to 0.04 with 6e-8 relative error, the node values are up to 1). Dominant is
 *     the argument: in = theta * 0.00277777777778f; the float constant differs from 1/360 by 2.4e-8 relative
 *     (hand check: 0.0027777778450 vs 0.0027777777778) and the product is rounded to float (half ulp, at most 3e-8
 *     absolute for in < 1), so for |theta| <= 360 degrees the argument error is at most 2.4e-8 + 3e-8 = 5.4e-8 turns =
 *     3.4e-7 rad. findex = 512 * in is exact (scaling by a power of two) and fract = findex - indexS is exact. So
 *     for |theta| <= 360 the value error is at most 3.4e-7 + 6e-8 (table) + 1e-7 (arithmetic) = 5.0e-7: B = 5e-7.
 *     For larger |theta| the argument error grows with the unreduced in = |theta| / 360: (2.4e-8 + 6e-8) * 2 pi =
 *     5.3e-7 rad per turn; B(theta) = 5e-7 + 2 pi |theta| / 360 * 1.5e-7 (margin 1.8 on the growing term).
 *   - q31 (5 LSB, i.e. 2.3e-9): the Hermite interpolation error is 5.9e-11 * 2^31 = 0.13 LSB, the table values and
 *     their derivative terms are rounded (0.5 LSB each, the derivative terms are weighted by Dn = 0.0123), the constant
 *     Dn = 0x1921FB5 differs from 2 pi / 512 by 1.0e-8 relative (derivative term <= 0.3 * Dn: below 0.1 LSB), and
 *     each of the three truncating `>> 31` steps loses less than 1 LSB (weighted by fract < 1): in total below
 *     about 4 LSB, bound 5 LSB.
 *   - sin^2 + cos^2 = 1: with a value error e on each, |sin^2 + cos^2 - 1| <= 2 e (|sin| + |cos|) + 2 e^2 <= 3 e.
 *
 * Host vs target (C6): the host runs the portable C path; the f32 function is plain float arithmetic and may differ in
 * the last bit on the target (FMA contraction), covered by the bounds. The q31 function is integer arithmetic. The
 * q31 function has undefined behaviour in C that the host happens to compile as intended (B-DCP-3): `indexS << 23`
 * with indexS >= 256 overflows int, `theta - (indexS << 23)` overflows, `<< 8` and `(q63_t)d1 << 1` shift negative
 * values, `3 * (q63_t)Df << 31` shifts a negative value, `(q63_t)Df << 32` shifts a negative value (arm_sin_cos_q31.c:69,
 * 81, 83, 99, 101; line 81 is the first UBSan hit). All q31 cases that reach them are therefore KNOWN_ISSUE_UB_HIDDEN
 * (skipped in sanitizer builds); UBSan stops at the first hit, so more may be hidden. B-DCP-4 (the negation of
 * sinTable_q31[384] = INT32_MIN in the cosine) is confirmed by the table value and the measured results, not by UBSan,
 * which stops earlier in the same function.
 * The f32 function converts `(int32_t) in` for in = |theta| / 360: |theta| >= about 7.7e11 degrees is out of the range
 * of that conversion and is NOT tested; the cases keep |theta| <= 3.6e6 degrees.
 */
#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <vector>

#include "TestHelpers.h"
#include "arm_math.h"

namespace {

const double kPi = 3.14159265358979323846;
const double kTwo31 = 2147483648.0;

// ---- f32 ----
double defSinF(float theta) { return std::sin((double)theta * kPi / 180.0); }
double defCosF(float theta) { return std::cos((double)theta * kPi / 180.0); }
double boundF(float theta) {
	const double a = std::fabs((double)theta);
	return a <= 360.0 ? 5e-7 : 5e-7 + 2.0 * kPi * a / 360.0 * 1.5e-7;
}

// ---- q31 ----
double clipQ31(double v) {
	if (v >= 2147483647.0) return 2147483647.0;
	if (v <= -2147483648.0) return -2147483648.0;
	return v;
}
double defSinQ(int32_t x) { return clipQ31(std::sin(kPi * (double)x / kTwo31) * kTwo31); }
double defCosQ(int32_t x) { return clipQ31(std::cos(kPi * (double)x / kTwo31) * kTwo31); }
const double kBoundQ = 5.0;

// B-DCP-4: the cosine is wrong in the two table intervals next to -90 degrees (index 383 and 384, x in
// [-90.70, -89.30] degrees): sinTable_q31[384] is INT32_MIN and the code negates it (d1 = -sinTable_q31[indexS],
// d2 = -sinTable_q31[indexS+1]), which overflows. The DEF checks of the other cases leave the cosine of these inputs
// out; the KNOWN_ISSUE case at the end of the q31 section checks them.
bool inBadCosInterval(int32_t x) {
	const uint32_t idx = (uint32_t)x >> 23;
	return idx == 383 || idx == 384;
}

uint32_t lcg(uint32_t &s) {
	s = s * 1664525u + 1013904223u;
	return s;
}

#define UB_Q31 KNOWN_ISSUE_UB_HIDDEN("B-DCP-3: arm_sin_cos_q31 shifts negative values and overflows signed int (indexS << 23, theta - (indexS << 23), << 8, (q63_t)d1 << 1, 3 * (q63_t)Df << 31; arm_sin_cos_q31.c:69-101), undefined in C; UBSan reports it, the result is the intended one on the host")

}  // namespace

TEST_SUITE("DSP Controller sin_cos") {

// ---------------------------------------------------------------------------------------------------------------
// f32
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C4 arm_sin_cos_f32] 0, 90, 180 and 270 degrees give 0 / 1 / 0 / -1 within the argument rounding") {
	// bound 1e-6 = B(theta) for theta <= 270 (1e-6 + 4.7e-7 -> use B)
	const float deg[4] = {0.0f, 90.0f, 180.0f, 270.0f};
	const double s[4] = {0.0, 1.0, 0.0, -1.0};
	const double c[4] = {1.0, 0.0, -1.0, 0.0};
	for (int i = 0; i < 4; i++) {
		float sv = 9.0f, cv = 9.0f;
		arm_sin_cos_f32(deg[i], &sv, &cv);
		CAPTURE(deg[i]);
		CHECK(std::fabs((double)sv - s[i]) <= boundF(deg[i]));
		CHECK(std::fabs((double)cv - c[i]) <= boundF(deg[i]));
	}
}

TEST_CASE("[C4 arm_sin_cos_f32] the negative angles -90, -180 and -270 degrees give -1 / 0 / 1 for the sine and 0 / -1 / 0 for the cosine") {
	const float deg[3] = {-90.0f, -180.0f, -270.0f};
	const double s[3] = {-1.0, 0.0, 1.0};
	const double c[3] = {0.0, -1.0, 0.0};
	for (int i = 0; i < 3; i++) {
		float sv = 9.0f, cv = 9.0f;
		arm_sin_cos_f32(deg[i], &sv, &cv);
		CAPTURE(deg[i]);
		CHECK(std::fabs((double)sv - s[i]) <= boundF(deg[i]));
		CHECK(std::fabs((double)cv - c[i]) <= boundF(deg[i]));
	}
}

TEST_CASE("[C4 arm_sin_cos_f32] the signs are right in each quadrant, also for negative angles and angles above 360 degrees") {
	// middle of each quadrant: |sin| and |cos| are 0.707, far from the zero crossings
	struct Q { float deg; int sSign; int cSign; };
	const Q q[] = {{45, 1, 1},    {135, 1, -1},   {225, -1, -1},  {315, -1, 1},
	               {-45, -1, 1},  {-135, -1, -1}, {-225, 1, -1},  {-315, 1, 1},
	               {405, 1, 1},   {495, 1, -1},   {585, -1, -1},  {675, -1, 1},
	               {-405, -1, 1}, {1125, 1, 1},   {-1125, -1, 1}, {3600 + 135, 1, -1}};
	for (const Q &t : q) {
		float sv = 0.0f, cv = 0.0f;
		arm_sin_cos_f32(t.deg, &sv, &cv);
		CAPTURE(t.deg);
		CHECK(((sv > 0.0f) ? 1 : -1) == t.sSign);
		CHECK(((cv > 0.0f) ? 1 : -1) == t.cSign);
		CHECK(std::fabs((double)sv - defSinF(t.deg)) <= boundF(t.deg));
		CHECK(std::fabs((double)cv - defCosF(t.deg)) <= boundF(t.deg));
	}
}

TEST_CASE("[C3 arm_sin_cos_f32] a sweep from -720 to 720 degrees with a prime stride matches the definition within the derived bound") {
	// step 0.37 degrees: about 3900 samples, many different fractions of the 512 interval table
	bool ok = true;
	for (int i = -1946; i <= 1946 && ok; i++) {
		const float t = (float)((double)i * 0.37);
		float sv = 0.0f, cv = 0.0f;
		arm_sin_cos_f32(t, &sv, &cv);
		if (!(std::fabs((double)sv - defSinF(t)) <= boundF(t)) || !(std::fabs((double)cv - defCosF(t)) <= boundF(t))) {
			CAPTURE(t);
			CAPTURE(sv);
			CAPTURE(cv);
			CHECK(std::fabs((double)sv - defSinF(t)) <= boundF(t));
			CHECK(std::fabs((double)cv - defCosF(t)) <= boundF(t));
			ok = false;
		}
	}
	CHECK(ok);
}

TEST_CASE("[C3 arm_sin_cos_f32] every table node of two turns (multiples of 360/512 degrees, positive and negative) matches the definition") {
	// 360 / 512 = 0.703125 is exact in float; at a node the argument sits on the edge of two table intervals
	bool ok = true;
	for (int k = -1024; k <= 1024 && ok; k++) {
		const float t = (float)k * 0.703125f;
		float sv = 0.0f, cv = 0.0f;
		arm_sin_cos_f32(t, &sv, &cv);
		if (!(std::fabs((double)sv - defSinF(t)) <= boundF(t)) || !(std::fabs((double)cv - defCosF(t)) <= boundF(t))) {
			CAPTURE(k);
			CAPTURE(sv);
			CAPTURE(cv);
			CHECK(std::fabs((double)sv - defSinF(t)) <= boundF(t));
			CHECK(std::fabs((double)cv - defCosF(t)) <= boundF(t));
			ok = false;
		}
	}
	CHECK(ok);
}

TEST_CASE("[C3 arm_sin_cos_f32] angles far outside one turn (up to +-3.6e6 degrees) match the definition within the growing argument bound") {
	// two strides: 13.7 degrees up to +-36000 and 1369.3 degrees up to +-3.6e6 (10000 turns); beyond ~7.7e11 degrees the
	// (int32_t) conversion of the library is out of range (not tested)
	bool ok = true;
	for (int i = -5258; i <= 5258 && ok; i++) {
		const double step = (i % 2 == 0) ? 1369.3 : 13.7;
		const float t = (float)((double)(i / 2) * step);  // even i: coarse stride, odd i: fine stride
		float sv = 0.0f, cv = 0.0f;
		arm_sin_cos_f32(t, &sv, &cv);
		if (!(std::fabs((double)sv - defSinF(t)) <= boundF(t)) || !(std::fabs((double)cv - defCosF(t)) <= boundF(t))) {
			CAPTURE(t);
			CAPTURE(sv);
			CAPTURE(cv);
			CHECK(std::fabs((double)sv - defSinF(t)) <= boundF(t));
			CHECK(std::fabs((double)cv - defCosF(t)) <= boundF(t));
			ok = false;
		}
	}
	CHECK(ok);
}

TEST_CASE("[C3 arm_sin_cos_f32] sin squared plus cos squared is 1 over the sweep") {
	// value error e <= B(theta); |s^2 + c^2 - 1| <= 3 e (header)
	bool ok = true;
	for (int i = -1000; i <= 1000 && ok; i++) {
		const float t = (float)((double)i * 0.91);
		float sv = 0.0f, cv = 0.0f;
		arm_sin_cos_f32(t, &sv, &cv);
		const double d = (double)sv * sv + (double)cv * cv - 1.0;
		if (!(std::fabs(d) <= 3.0 * boundF(t))) {
			CAPTURE(t);
			CHECK(std::fabs(d) <= 3.0 * boundF(t));
			ok = false;
		}
	}
	CHECK(ok);
}

TEST_CASE("[C3 arm_sin_cos_f32] the sine is odd and the cosine is even") {
	bool ok = true;
	for (int i = 1; i <= 800 && ok; i++) {
		const float t = (float)((double)i * 1.13);
		float sp = 0.0f, cp = 0.0f, sn = 0.0f, cn = 0.0f;
		arm_sin_cos_f32(t, &sp, &cp);
		arm_sin_cos_f32(-t, &sn, &cn);
		if (!(std::fabs((double)sp + (double)sn) <= 2.0 * boundF(t)) || !(std::fabs((double)cp - (double)cn) <= 2.0 * boundF(t))) {
			CAPTURE(t);
			CHECK(std::fabs((double)sp + (double)sn) <= 2.0 * boundF(t));
			CHECK(std::fabs((double)cp - (double)cn) <= 2.0 * boundF(t));
			ok = false;
		}
	}
	CHECK(ok);
}

TEST_CASE("[C3 arm_sin_cos_f32] an angle and the same angle plus one turn give the same result within the bound of the larger argument") {
	bool ok = true;
	for (int i = -300; i <= 300 && ok; i++) {
		const float t = (float)((double)i * 0.97);
		float s1 = 0.0f, c1 = 0.0f, s2 = 0.0f, c2 = 0.0f;
		arm_sin_cos_f32(t, &s1, &c1);
		arm_sin_cos_f32(t + 360.0f, &s2, &c2);
		const double b = boundF(t) + boundF(t + 360.0f) + 1e-6;  // + 1e-6: the float sum t + 360 is rounded (ulp(700) = 6e-5 degrees = 1e-6 rad)
		if (!(std::fabs((double)s1 - (double)s2) <= b) || !(std::fabs((double)c1 - (double)c2) <= b)) {
			CAPTURE(t);
			CHECK(std::fabs((double)s1 - (double)s2) <= b);
			CHECK(std::fabs((double)c1 - (double)c2) <= b);
			ok = false;
		}
	}
	CHECK(ok);
}

// ---------------------------------------------------------------------------------------------------------------
// q31
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C4 arm_sin_cos_q31] 0, +90, -90 and -180 degrees (0, 0x40000000, 0xC0000000, 0x80000000) give the exact axis values within 5 LSB" * UB_Q31) {
	// ranges: 0x80000000 is -1.0 = -180 degrees (DOC); the results +1.0 saturate to 0x7FFFFFFF, -1.0 is 0x80000000
	const int32_t in[4] = {0, 0x40000000, (int32_t)0xC0000000u, INT32_MIN};
	const double s[4] = {0.0, 2147483647.0, -2147483648.0, 0.0};
	const double c[4] = {2147483647.0, 0.0, 0.0, -2147483648.0};
	for (int i = 0; i < 4; i++) {
		int32_t sv = 77, cv = 77;
		arm_sin_cos_q31(in[i], &sv, &cv);
		CAPTURE(in[i]);
		CHECK(std::fabs((double)sv - s[i]) <= kBoundQ);
		CHECK(std::fabs((double)cv - c[i]) <= kBoundQ);
	}
}

TEST_CASE("[C4 arm_sin_cos_q31] the table nodes at +90 and 0 degrees give a sine and a cosine of 0x7FFFFFFF (+1.0 is not wrapped)" * UB_Q31) {
	// DEF: sin(90 deg) = 1.0 = 2^31 which is not a q31 value; the nearest is 0x7FFFFFFF (as the table node itself).
	// These nodes do not reach the clip of the interpolation (see the next cases).
	int32_t sv = 0, cv = 0;
	arm_sin_cos_q31(0x40000000, &sv, &cv);
	CHECK(sv >= INT32_MAX - (int32_t)kBoundQ);
	arm_sin_cos_q31(0, &sv, &cv);
	CHECK(cv >= INT32_MAX - (int32_t)kBoundQ);
}

TEST_CASE("[C4 arm_sin_cos_q31] the sine just below -90 degrees (40000 inputs, including the B-DCP-4 interval) stays at the negative end and does not wrap" * UB_Q31) {
	// the interpolated sine of x slightly below 0xC0000000 would be below -2^31; the code clips it (clip_q63_to_q31).
	// DEF: sin(-pi/2 - e) = -cos(e), -2^31 * (1 - e^2 / 2): within 5 LSB of -2^31 for e <= 29774 LSB of x, close to
	// it everywhere in the scan; the DEF with the clip is compared within the bound and the sign must stay negative.
	int clipped = 0;
	bool ok = true;
	for (int d = 0; d <= 40000 && ok; d++) {
		const int32_t x = (int32_t)(0xC0000000u - (uint32_t)d);
		int32_t sv = 0, cv = 0;
		arm_sin_cos_q31(x, &sv, &cv);
		if (sv == INT32_MIN) clipped++;
		if (!(sv < 0) || !(std::fabs((double)sv - defSinQ(x)) <= kBoundQ)) {
			CAPTURE(x);
			CAPTURE(sv);
			CHECK(sv < 0);
			CHECK(std::fabs((double)sv - defSinQ(x)) <= kBoundQ);
			ok = false;
		}
	}
	CHECK(ok);
	CHECK(clipped > 0);  // the clip is reached in this scan
}

TEST_CASE("[C4 arm_sin_cos_q31] the cosine of the last 30000 inputs below +180 degrees stays at the negative end and does not wrap" * UB_Q31) {
	// DEF: cos(pi - e) = -cos(e): about -2^31 * (1 - e^2 / 2); the interpolation can fall below -2^31 and is clipped.
	int clipped = 0;
	bool ok = true;
	for (int d = 0; d < 30000 && ok; d++) {
		const int32_t x = INT32_MAX - d;
		int32_t sv = 0, cv = 0;
		arm_sin_cos_q31(x, &sv, &cv);
		if (cv == INT32_MIN) clipped++;
		if (!(cv < 0) || !(std::fabs((double)cv - defCosQ(x)) <= kBoundQ)) {
			CAPTURE(x);
			CAPTURE(cv);
			CHECK(cv < 0);
			CHECK(std::fabs((double)cv - defCosQ(x)) <= kBoundQ);
			ok = false;
		}
	}
	CHECK(ok);
	CHECK(clipped > 0);
}

TEST_CASE("[C4 arm_sin_cos_q31] the signs are right in each quadrant (45, 135, -135 and -45 degrees)" * UB_Q31) {
	struct Q { int32_t x; int sSign; int cSign; };
	const Q q[] = {{0x20000000, 1, 1}, {0x60000000, 1, -1}, {(int32_t)0xA0000000u, -1, -1}, {(int32_t)0xE0000000u, -1, 1}};
	for (const Q &t : q) {
		int32_t sv = 0, cv = 0;
		arm_sin_cos_q31(t.x, &sv, &cv);
		CAPTURE(t.x);
		CHECK(((sv > 0) ? 1 : -1) == t.sSign);
		CHECK(((cv > 0) ? 1 : -1) == t.cSign);
		CHECK(std::fabs((double)sv - defSinQ(t.x)) <= kBoundQ);
		CHECK(std::fabs((double)cv - defCosQ(t.x)) <= kBoundQ);
	}
}

TEST_CASE("[C3 arm_sin_cos_q31] all 512 table nodes (multiples of 2^23 from -180 degrees) match the definition within 5 LSB" * UB_Q31) {
	bool ok = true;
	for (int k = -256; k < 256 && ok; k++) {
		const int32_t x = (int32_t)((int64_t)k * 8388608);
		int32_t sv = 0, cv = 0;
		arm_sin_cos_q31(x, &sv, &cv);
		if (!(std::fabs((double)sv - defSinQ(x)) <= kBoundQ) || !((inBadCosInterval(x) || std::fabs((double)cv - defCosQ(x)) <= kBoundQ))) {
			CAPTURE(k);
			CAPTURE(sv);
			CAPTURE(cv);
			CHECK(std::fabs((double)sv - defSinQ(x)) <= kBoundQ);
			CHECK((inBadCosInterval(x) || std::fabs((double)cv - defCosQ(x)) <= kBoundQ));
			ok = false;
		}
	}
	CHECK(ok);
}

TEST_CASE("[C3 arm_sin_cos_q31] a strided sweep over the whole input range (about 524000 values) matches the definition within 5 LSB" * UB_Q31) {
	// stride 8191 (prime, not a divisor of the node spacing 2^23) from INT32_MIN to the largest value
	bool ok = true;
	for (int64_t v = INT32_MIN; v <= INT32_MAX && ok; v += 8191) {
		const int32_t x = (int32_t)v;
		int32_t sv = 0, cv = 0;
		arm_sin_cos_q31(x, &sv, &cv);
		if (!(std::fabs((double)sv - defSinQ(x)) <= kBoundQ) || !((inBadCosInterval(x) || std::fabs((double)cv - defCosQ(x)) <= kBoundQ))) {
			CAPTURE(x);
			CAPTURE(sv);
			CAPTURE(cv);
			CHECK(std::fabs((double)sv - defSinQ(x)) <= kBoundQ);
			CHECK((inBadCosInterval(x) || std::fabs((double)cv - defCosQ(x)) <= kBoundQ));
			ok = false;
		}
	}
	CHECK(ok);
}

TEST_CASE("[C3 arm_sin_cos_q31] the input extremes 0x80000000 and 0x7FFFFFFF and their neighbours match the definition within 5 LSB" * UB_Q31) {
	const int32_t xs[] = {INT32_MIN, INT32_MIN + 1, INT32_MIN + 2, -1, 1, 2, INT32_MAX - 2, INT32_MAX - 1, INT32_MAX};
	for (int32_t x : xs) {
		int32_t sv = 0, cv = 0;
		arm_sin_cos_q31(x, &sv, &cv);
		CAPTURE(x);
		CHECK(std::fabs((double)sv - defSinQ(x)) <= kBoundQ);
		CHECK((inBadCosInterval(x) || std::fabs((double)cv - defCosQ(x)) <= kBoundQ));
	}
}

TEST_CASE("[C3 arm_sin_cos_q31] 3000 pseudo random inputs match the definition within 5 LSB" * UB_Q31) {
	uint32_t seed = 61;
	bool ok = true;
	for (int i = 0; i < 3000 && ok; i++) {
		const int32_t x = (int32_t)lcg(seed);
		int32_t sv = 0, cv = 0;
		arm_sin_cos_q31(x, &sv, &cv);
		if (!(std::fabs((double)sv - defSinQ(x)) <= kBoundQ) || !((inBadCosInterval(x) || std::fabs((double)cv - defCosQ(x)) <= kBoundQ))) {
			CAPTURE(x);
			CAPTURE(sv);
			CAPTURE(cv);
			CHECK(std::fabs((double)sv - defSinQ(x)) <= kBoundQ);
			CHECK((inBadCosInterval(x) || std::fabs((double)cv - defCosQ(x)) <= kBoundQ));
			ok = false;
		}
	}
	CHECK(ok);
}

TEST_CASE("[C3 arm_sin_cos_q31] sin squared plus cos squared is 1 within 3 times the value error" * UB_Q31) {
	// e = 5 LSB / 2^31 (header)
	const double e = kBoundQ / kTwo31;
	uint32_t seed = 62;
	bool ok = true;
	for (int i = 0; i < 2000 && ok; i++) {
		const int32_t x = (int32_t)lcg(seed);
		int32_t sv = 0, cv = 0;
		arm_sin_cos_q31(x, &sv, &cv);
		const double s = (double)sv / kTwo31, c = (double)cv / kTwo31;
		if (!inBadCosInterval(x) && !(std::fabs(s * s + c * c - 1.0) <= 3.0 * e)) {
			CAPTURE(x);
			CHECK(std::fabs(s * s + c * c - 1.0) <= 3.0 * e);
			ok = false;
		}
	}
	CHECK(ok);
}

TEST_CASE("[C3 arm_sin_cos_q31] the sine is odd and the cosine is even within twice the value error" * UB_Q31) {
	uint32_t seed = 63;
	bool ok = true;
	for (int i = 0; i < 1000 && ok; i++) {
		int32_t x = (int32_t)lcg(seed);
		if (x == INT32_MIN) x = 5;  // -x would overflow
		int32_t sp = 0, cp = 0, sn = 0, cn = 0;
		arm_sin_cos_q31(x, &sp, &cp);
		arm_sin_cos_q31(-x, &sn, &cn);
		if (!(std::fabs((double)sp + (double)sn) <= 2.0 * kBoundQ) || !((inBadCosInterval(x) || inBadCosInterval(-x) || std::fabs((double)cp - (double)cn) <= 2.0 * kBoundQ))) {
			CAPTURE(x);
			CHECK(std::fabs((double)sp + (double)sn) <= 2.0 * kBoundQ);
			CHECK((inBadCosInterval(x) || inBadCosInterval(-x) || std::fabs((double)cp - (double)cn) <= 2.0 * kBoundQ));
			ok = false;
		}
	}
	CHECK(ok);
}

TEST_CASE("[C3 arm_sin_cos_q31] the cosine in the two table intervals around -90 degrees matches the definition within 5 LSB" * KNOWN_ISSUE_UB("B-DCP-4: sinTable_q31[384] is INT32_MIN and arm_sin_cos_q31.c negates it (d1 = -sinTable_q31[indexS], d2 = -sinTable_q31[indexS+1]): the negation overflows (UB) and stays negative, so the cosine is wrong by up to 7.8e6 LSB (0.0036, DOC: accurate to 5 digits) between -90.70 and -89.30 degrees, e.g. the cosine of -89.99 degrees is negative")) {
	// DEF: cos(-90 deg + e) = sin(e) > 0 for e > 0. First input: -90 degrees + 1000 LSB: cos = sin(pi * 1000 / 2^31) * 2^31 = 3141.6
	const int32_t first = (int32_t)0xC0000000u + 1000;
	int32_t sv = 0, cv = 0;
	arm_sin_cos_q31(first, &sv, &cv);
	CHECK(std::fabs((double)cv - defCosQ(first)) <= kBoundQ);
	// the whole interval, every 65536th input (about 256 values)
	for (int64_t v = (int64_t)(int32_t)0xC0000000u - 8388608; v < (int64_t)(int32_t)0xC0000000u + 8388608; v += 65536) {
		const int32_t x = (int32_t)v;
		arm_sin_cos_q31(x, &sv, &cv);
		CHECK(std::fabs((double)cv - defCosQ(x)) <= kBoundQ);
	}
}

// ---------------------------------------------------------------------------------------------------------------
// C5: q31 against f32
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C5 arm_sin_cos_q31] the result agrees with arm_sin_cos_f32 of the same angle in degrees" * UB_Q31) {
	// angle in degrees = x / 2^31 * 180 (rounded to float: <= 180 * 6e-8 = 1.1e-5 degrees = 1.9e-7 rad);
	// bound = B_f32 (5e-7 for |theta| <= 360) + 1.9e-7 + 5 LSB (2.3e-9) -> 1e-6
	uint32_t seed = 64;
	bool ok = true;
	for (int i = 0; i < 1500 && ok; i++) {
		const int32_t x = (int32_t)lcg(seed);
		const float deg = (float)((double)x / kTwo31 * 180.0);
		float sf = 0.0f, cf = 0.0f;
		int32_t sq = 0, cq = 0;
		arm_sin_cos_f32(deg, &sf, &cf);
		arm_sin_cos_q31(x, &sq, &cq);
		if (!(std::fabs((double)sq / kTwo31 - (double)sf) <= 1e-6) || !((inBadCosInterval(x) || std::fabs((double)cq / kTwo31 - (double)cf) <= 1e-6))) {
			CAPTURE(x);
			CAPTURE(deg);
			CHECK(std::fabs((double)sq / kTwo31 - (double)sf) <= 1e-6);
			CHECK((inBadCosInterval(x) || std::fabs((double)cq / kTwo31 - (double)cf) <= 1e-6));
			ok = false;
		}
	}
	CHECK(ok);
}

}  // TEST_SUITE
