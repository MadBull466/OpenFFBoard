/*
 * CMSIS-DSP Controller transforms (phase P11 group G2, work order CT): arm_clarke_f32, arm_inv_clarke_f32,
 * arm_park_f32, arm_inv_park_f32, arm_clarke_q31, arm_inv_clarke_q31, arm_park_q31, arm_inv_park_q31 (inline
 * functions of Include/dsp/controller_functions.h, groups clarke, inv_clarke, park, inv_park).
 * Requirement ids: Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * ARM has NO test and no pattern for these functions (lead decision of the G2 work orders, C3): the oracle is DEF
 * (the formulas quoted in the header, evaluated in double in this file) plus the round trips as a second oracle.
 * No pattern file is used, therefore no ARM tolerance exists; every bound is derived below.
 *
 * DEF (header comments of the functions; the formula images of the doc are not available, the code comments state
 * them):
 *   clarke      Ialpha = Ia                       Ibeta = (1/sqrt(3)) Ia + (2/sqrt(3)) Ib
 *   inv clarke  Ia = Ialpha                       Ib = -(1/2) Ialpha + (sqrt(3)/2) Ibeta
 *   park        Id = Ialpha cos + Ibeta sin       Iq = -Ialpha sin + Ibeta cos
 *   inv park    Ialpha = Id cos - Iq sin          Ibeta = Id sin + Iq cos
 * Second oracle: inv_clarke(clarke(Ia, Ib)) = (Ia, Ib) because Ib' = -Ia/2 + (sqrt(3)/2)(Ia + 2 Ib)/sqrt(3) = Ib;
 * inv_park(park(x)) = x for sin^2 + cos^2 = 1. For balanced three phase currents Ia = cos t, Ib = cos(t - 2 pi/3)
 * the amplitude invariant Clarke transform gives Ialpha = cos t, Ibeta = sin t (hand derivation: Ibeta =
 * (cos t + 2 cos(t - 2pi/3)) / sqrt(3) = (cos t - cos t + sqrt(3) sin t) / sqrt(3) = sin t).
 *
 * Q31 scale (header code): a q31 value x stands for x / 2^31. clarke: product1 = Ia * 0x24F34E8B >> 30, i.e. the
 * constant is 0x24F34E8B / 2^30 = 1/sqrt(3) = 0.57735 and product2 uses 0x49E69D16 / 2^30 = 2/sqrt(3) = 1.1547.
 * inv clarke: 0x40000000 / 2^31 = 1/2 and 0x6ED9EBA1 / 2^31 = sqrt(3)/2 = 0.86603. (The code comments of
 * arm_inv_clarke_q31 call them "1/(2*sqrt(3))" and "1/sqrt(3)": wrong comments, the numbers are the ones of the
 * f32 formula.) The checks of the constants are in the "scale" tests with hand derived expected values.
 *
 * Bounds (derived):
 *   - f32 clarke / inv clarke: the float literals have relative error <= 2^-24 = 6e-8, each product and the sum add
 *     one rounding of 2^-24 relative: error <= 3 * 6e-8 * (|term1| + |term2|) <= 2e-7 * (|a| + |b|) (the scales 0.58,
 *     1.155 and 0.87 are all <= 1.155). Bound used: 3e-7 * (|a| + |b|). A fused multiply add on the target only removes
 *     a rounding. f32 park / inv park: two products (1 rounding each) and one sum: 3 * 6e-8 * (|a cos| + |b sin|)
 *     <= 2e-7 * (|a| + |b|), same bound 3e-7 * (|a| + |b|); the DEF uses the float sin / cos values that are passed.
 *   - f32 round trips: clarke -> inv clarke: the error of Ibeta (3e-7 (|a|+|b|)) is multiplied by 0.866, the inverse adds
 *     3e-7 (|Ialpha| + |Ibeta|) with |Ibeta| <= 1.155 (|a|+|b|): <= 1.5e-6 (|a|+|b|). park -> inv park: first error
 *     3e-7 (|a|+|b|), propagated with |cos|+|sin| <= 1.42, own error 3e-7 sqrt(2) (|a|+|b|), and the error of
 *     sin^2 + cos^2 = 1 +- 2.4e-7 of the float values: <= 2e-6 (|a|+|b|) is a safe bound.
 *   - q31: all products are (int64 product) >> 31 (or >> 30), i.e. floor, error in (-1, 0] LSB each; the constants
 *     0x24F34E8B / 2^30 etc. are rounded to nearest, giving at most 0.5 constant-ulp * |x| = 1 LSB for the Clarke
 *     constants (ulp 2^-30 times |x| <= 2^31 gives 2 LSB, half of it is 1) and 0.5 LSB for the other constants.
 *     clarke: Ibeta error <= 2 (floors) + 1 + 1 (constants) = 4 LSB. inv clarke: Ib error <= 1 (Ialpha / 2 floors by
 *     at most 1) + 1 (floor) + 0.5 (constant) = 2.5, bound 3 LSB. park Id / inv park Ibeta (sum of two floors):
 *     error in (-2, 0] against DEF with exact sin / cos values, bound 2 LSB; park Iq / inv park Ialpha (difference
 *     of two floors): error in (-1, 1), bound 1 LSB; 2 LSB is used for all four for simplicity, plus the double
 *     rounding of the reference (products up to 2^62, relative error 1e-16, negligible).
 *   - Saturation: DEF values are clipped to [-2^31, 2^31 - 1] (QADD / QSUB saturate). clip is 1-Lipschitz, so the same
 *     LSB bound holds for the clipped values.
 *   - q31 round trips: clarke -> inv clarke: alpha exact, Ib <= 0.866 * 4 + 3 < 7 LSB, bound 8. park -> inv park:
 *     (|cos|+|sin|) * 2 + 2 + 1 (sin^2 + cos^2 of the rounded q31 values differs from 1 by up to 2 LSB * |x|) < 8 LSB.
 *   - C5: q31 result / 2^31 against the f32 function on exactly representable inputs (multiples of 1/16, and the
 *     rounded sin / cos): bound = f32 bound + the q31 bound / 2^31 + the rounding of the float sin / cos (6e-8).
 *
 * Findings: B-DCT-1 arm_clarke_q31 wraps instead of saturating when 2/sqrt(3)*Ib exceeds the Q31 range; B-DCT-2 park / inv park wrap
 * for the product (-1.0) * (-1.0) (see the tests). The sweeps avoid -1.0 as sin / cos value (qSc) for that reason.
 *
 * Host vs target (C6): the host runs the C code of the header (generic __QADD / __QSUB saturating adds), the target
 * the single-cycle QADD / QSUB instructions: identical results for in-range values. The f32 functions may be
 * contracted to FMA on the target (fewer roundings, within the bounds above). The conversion of an out of range int64
 * to q31 in the cast `(q31_t)(((q63_t)x * k) >> 30)` is implementation defined (not undefined) and wraps modulo 2^32 on
 * GCC, clang and ARM; the right shift of a negative q63 is arithmetic on all of them.
 */
#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "TestHelpers.h"
#include "arm_math.h"

namespace {

const double kPi = 3.14159265358979323846;
const double kTwo31 = 2147483648.0;
const double kSqrt3 = 1.7320508075688772935;

// deterministic pseudo random values (no <random>: identical on every platform)
uint32_t lcg(uint32_t &s) {
	s = s * 1664525u + 1013904223u;
	return s;
}

int64_t clipQ31(double v) {
	if (v >= 2147483647.0) return 2147483647;
	if (v <= -2147483648.0) return -2147483648LL;
	return (int64_t)std::floor(v + 0.5);
}

// saturation without rounding
double clipD(double v) { return v >= 2147483647.0 ? 2147483647.0 : (v <= -2147483648.0 ? -2147483648.0 : v); }

// q31 of x/2^31 saturated
q31_t q(double x) { return (q31_t)clipQ31(x * kTwo31); }

// sin / cos value without -1.0 (INT32_MIN): (-1.0) * (-1.0) does not fit, see B-DCT-2
q31_t qSc(double x) {
	q31_t v = q(x);
	return v == INT32_MIN ? -INT32_MAX : v;
}

// collects the first failure of a loop so that a sweep is a single assertion
struct Acc {
	bool ok = true;
	std::string msg;
	void check(bool cond, const char *what, long idx, double got, double want, double bound) {
		if (cond || !ok) return;
		ok = false;
		char buf[256];
		std::snprintf(buf, sizeof buf, "%s: case %ld got %.10g want %.10g bound %.3g", what, idx, got, want, bound);
		msg = buf;
	}
};

// ---- DEF in double ----
double defClarkeBeta(double a, double b) { return (a + 2.0 * b) / kSqrt3; }
double defInvClarkeB(double alpha, double beta) { return -0.5 * alpha + 0.5 * kSqrt3 * beta; }
double defParkD(double a, double b, double s, double c) { return a * c + b * s; }
double defParkQ(double a, double b, double s, double c) { return -a * s + b * c; }
double defInvParkAlpha(double d, double qq, double s, double c) { return d * c - qq * s; }
double defInvParkBeta(double d, double qq, double s, double c) { return d * s + qq * c; }

// f32 inputs: exact multiples of 1/8 in [-2, 2] and pseudo random values in [-1000, 1000]
std::vector<float> f32Values() {
	std::vector<float> v;
	for (int k = -16; k <= 16; k++) v.push_back((float)k / 8.0f);
	uint32_t seed = 12345u;
	for (int i = 0; i < 24; i++) {
		uint32_t r = lcg(seed);
		v.push_back((float)((double)(r >> 8) / 16777216.0 * 2000.0 - 1000.0));
	}
	return v;
}

// angles 2*pi*k/64 for k in [-80, 160]: negative, one turn and beyond (sin / cos do not care)
std::vector<double> angles() {
	std::vector<double> v;
	for (int k = -80; k <= 160; k++) v.push_back(2.0 * kPi * (double)k / 64.0);
	return v;
}

// q31 inputs, multiples of 1/16 (exact in float): -0.75 .. 0.75
std::vector<q31_t> q31Grid() {
	std::vector<q31_t> v;
	for (int k = -12; k <= 12; k++) v.push_back((q31_t)((int64_t)k * (int64_t)(kTwo31 / 16.0)));
	return v;
}

}  // namespace

TEST_SUITE("DSP Controller transforms") {

// ---------------------------------------------------------------------------------------------------------------
// f32, C3 (DEF)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_clarke_f32] Ialpha equals Ia exactly and Ibeta equals (Ia + 2 Ib) / sqrt(3) within 3e-7 (|Ia| + |Ib|)") {
	std::vector<float> v = f32Values();
	Acc acc;
	long n = 0;
	for (float a : v)
		for (float b : v) {
			float al = 0, be = 0;
			arm_clarke_f32(a, b, &al, &be);
			double bound = 3e-7 * (std::fabs((double)a) + std::fabs((double)b));
			acc.check(al == a, "alpha", n, al, a, 0);
			acc.check(std::fabs((double)be - defClarkeBeta(a, b)) <= bound, "beta", n, be, defClarkeBeta(a, b), bound);
			n++;
		}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

TEST_CASE("[C3 arm_inv_clarke_f32] Ia equals Ialpha exactly and Ib equals -Ialpha / 2 + sqrt(3) / 2 Ibeta within 3e-7 (|Ialpha| + |Ibeta|)") {
	std::vector<float> v = f32Values();
	Acc acc;
	long n = 0;
	for (float a : v)
		for (float b : v) {
			float ia = 0, ib = 0;
			arm_inv_clarke_f32(a, b, &ia, &ib);
			double bound = 3e-7 * (std::fabs((double)a) + std::fabs((double)b));
			acc.check(ia == a, "Ia", n, ia, a, 0);
			acc.check(std::fabs((double)ib - defInvClarkeB(a, b)) <= bound, "Ib", n, ib, defInvClarkeB(a, b), bound);
			n++;
		}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

TEST_CASE("[C3 arm_park_f32] Id and Iq match the rotation of (Ialpha, Ibeta) for angles from -2.5 to 5 turns within 3e-7 (|Ialpha| + |Ibeta|)") {
	std::vector<float> v = f32Values();
	std::vector<double> th = angles();
	Acc acc;
	long n = 0;
	for (double t : th) {
		float s = (float)std::sin(t), c = (float)std::cos(t);
		for (size_t i = 0; i < v.size(); i += 3)
			for (size_t j = 0; j < v.size(); j += 2) {
				float id = 0, iq = 0;
				arm_park_f32(v[i], v[j], &id, &iq, s, c);
				double bound = 3e-7 * (std::fabs((double)v[i]) + std::fabs((double)v[j]));
				double wd = defParkD(v[i], v[j], s, c), wq = defParkQ(v[i], v[j], s, c);
				acc.check(std::fabs((double)id - wd) <= bound, "Id", n, id, wd, bound);
				acc.check(std::fabs((double)iq - wq) <= bound, "Iq", n, iq, wq, bound);
				n++;
			}
	}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

TEST_CASE("[C3 arm_inv_park_f32] Ialpha and Ibeta match the inverse rotation of (Id, Iq) for angles from -2.5 to 5 turns within 3e-7 (|Id| + |Iq|)") {
	std::vector<float> v = f32Values();
	std::vector<double> th = angles();
	Acc acc;
	long n = 0;
	for (double t : th) {
		float s = (float)std::sin(t), c = (float)std::cos(t);
		for (size_t i = 0; i < v.size(); i += 3)
			for (size_t j = 0; j < v.size(); j += 2) {
				float al = 0, be = 0;
				arm_inv_park_f32(v[i], v[j], &al, &be, s, c);
				double bound = 3e-7 * (std::fabs((double)v[i]) + std::fabs((double)v[j]));
				double wa = defInvParkAlpha(v[i], v[j], s, c), wb = defInvParkBeta(v[i], v[j], s, c);
				acc.check(std::fabs((double)al - wa) <= bound, "Ialpha", n, al, wa, bound);
				acc.check(std::fabs((double)be - wb) <= bound, "Ibeta", n, be, wb, bound);
				n++;
			}
	}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

// ---------------------------------------------------------------------------------------------------------------
// f32, known values and round trips
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_clarke_f32] balanced three phase currents give Ialpha = cos t and Ibeta = sin t") {
	// Ia = cos t, Ib = cos(t - 2pi/3): hand derivation in the header comment; bound: input rounding 6e-8 + 3e-7 * 2
	Acc acc;
	for (int k = -48; k <= 96; k++) {
		double t = 2.0 * kPi * (double)k / 48.0;
		float a = (float)std::cos(t), b = (float)std::cos(t - 2.0 * kPi / 3.0), al = 0, be = 0;
		arm_clarke_f32(a, b, &al, &be);
		acc.check(std::fabs((double)al - std::cos(t)) <= 1e-6, "alpha", k, al, std::cos(t), 1e-6);
		acc.check(std::fabs((double)be - std::sin(t)) <= 1e-6, "beta", k, be, std::sin(t), 1e-6);
	}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

TEST_CASE("[C3 arm_clarke_f32] the constants are 1/sqrt(3) and 2/sqrt(3): unit inputs give 0.57735 and 1.1547") {
	float al = 0, be = 0;
	arm_clarke_f32(1.0f, 0.0f, &al, &be);
	CHECK(be == doctest::Approx(1.0 / kSqrt3).epsilon(1e-7));
	arm_clarke_f32(0.0f, 1.0f, &al, &be);
	CHECK(be == doctest::Approx(2.0 / kSqrt3).epsilon(1e-7));
	arm_clarke_f32(1.0f, -0.5f, &al, &be);  // Ic = -0.5: a balanced point on the alpha axis, Ibeta = 0
	CHECK(std::fabs((double)be) <= 1e-7);
}

TEST_CASE("[C3 arm_inv_clarke_f32] the constants are -1/2 and sqrt(3)/2: unit inputs give -0.5 and 0.86603") {
	float ia = 0, ib = 0;
	arm_inv_clarke_f32(1.0f, 0.0f, &ia, &ib);
	CHECK(ib == -0.5f);
	arm_inv_clarke_f32(0.0f, 1.0f, &ia, &ib);
	CHECK(ib == doctest::Approx(kSqrt3 / 2.0).epsilon(1e-7));
}

TEST_CASE("[C3 arm_park_f32] the angles 0, 90, 180 and 270 degrees map (Ialpha, Ibeta) to the exact rotated components") {
	const float al = 0.75f, be = -0.25f;
	float id = 0, iq = 0;
	arm_park_f32(al, be, &id, &iq, 0.0f, 1.0f);  // 0 deg: Id = Ialpha, Iq = Ibeta
	CHECK(id == al);
	CHECK(iq == be);
	arm_park_f32(al, be, &id, &iq, 1.0f, 0.0f);  // 90 deg: Id = Ibeta, Iq = -Ialpha
	CHECK(id == be);
	CHECK(iq == -al);
	arm_park_f32(al, be, &id, &iq, 0.0f, -1.0f);  // 180 deg: Id = -Ialpha, Iq = -Ibeta
	CHECK(id == -al);
	CHECK(iq == -be);
	arm_park_f32(al, be, &id, &iq, -1.0f, 0.0f);  // 270 deg: Id = -Ibeta, Iq = Ialpha
	CHECK(id == -be);
	CHECK(iq == al);
}

TEST_CASE("[C3 arm_inv_park_f32] the angles 0, 90, 180 and 270 degrees map (Id, Iq) to the exact rotated components") {
	const float d = 0.75f, qq = -0.25f;
	float al = 0, be = 0;
	arm_inv_park_f32(d, qq, &al, &be, 0.0f, 1.0f);  // 0 deg: Ialpha = Id, Ibeta = Iq
	CHECK(al == d);
	CHECK(be == qq);
	arm_inv_park_f32(d, qq, &al, &be, 1.0f, 0.0f);  // 90 deg: Ialpha = -Iq, Ibeta = Id
	CHECK(al == -qq);
	CHECK(be == d);
	arm_inv_park_f32(d, qq, &al, &be, 0.0f, -1.0f);  // 180 deg: both negated
	CHECK(al == -d);
	CHECK(be == -qq);
	arm_inv_park_f32(d, qq, &al, &be, -1.0f, 0.0f);  // 270 deg: Ialpha = Iq, Ibeta = -Id
	CHECK(al == qq);
	CHECK(be == -d);
}

TEST_CASE("[C3 arm_clarke_f32 arm_inv_clarke_f32] the inverse Clarke transform undoes the Clarke transform within 1.5e-6 (|Ia| + |Ib|)") {
	std::vector<float> v = f32Values();
	Acc acc;
	long n = 0;
	for (float a : v)
		for (float b : v) {
			float al = 0, be = 0, ra = 0, rb = 0;
			arm_clarke_f32(a, b, &al, &be);
			arm_inv_clarke_f32(al, be, &ra, &rb);
			double bound = 1.5e-6 * (std::fabs((double)a) + std::fabs((double)b));
			acc.check(ra == a, "Ia", n, ra, a, 0);
			acc.check(std::fabs((double)rb - b) <= bound, "Ib", n, rb, b, bound);
			n++;
		}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

TEST_CASE("[C3 arm_park_f32 arm_inv_park_f32] the inverse Park transform undoes the Park transform within 2e-6 (|a| + |b|)") {
	std::vector<float> v = f32Values();
	Acc acc;
	long n = 0;
	for (double t : angles()) {
		float s = (float)std::sin(t), c = (float)std::cos(t);
		for (size_t i = 0; i < v.size(); i += 3)
			for (size_t j = 0; j < v.size(); j += 2) {
				float id = 0, iq = 0, ra = 0, rb = 0;
				arm_park_f32(v[i], v[j], &id, &iq, s, c);
				arm_inv_park_f32(id, iq, &ra, &rb, s, c);
				double bound = 2e-6 * (std::fabs((double)v[i]) + std::fabs((double)v[j]));
				acc.check(std::fabs((double)ra - v[i]) <= bound, "Ialpha", n, ra, v[i], bound);
				acc.check(std::fabs((double)rb - v[j]) <= bound, "Ibeta", n, rb, v[j], bound);
				n++;
			}
	}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

// ---------------------------------------------------------------------------------------------------------------
// q31, C3 (DEF)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_clarke_q31] Ialpha equals Ia exactly and Ibeta equals (Ia + 2 Ib) / sqrt(3) within 4 LSB when the result is in range") {
	// inputs |x| <= 0.75: products stay in range (0.75 * 1.1547 = 0.87 < 1), only the sum can saturate (tested below)
	std::vector<q31_t> g = q31Grid();
	Acc acc;
	long n = 0, tested = 0;
	for (q31_t a : g)
		for (q31_t b : g) {
			q31_t al = 0, be = 0;
			arm_clarke_q31(a, b, &al, &be);
			double want = defClarkeBeta((double)a, (double)b);
			acc.check(al == a, "alpha", n, al, a, 0);
			acc.check(std::fabs((double)be - (double)clipQ31(want)) <= 4.0, "beta", n, be, (double)clipQ31(want), 4);
			if (std::fabs(want) < kTwo31) tested++;
			n++;
		}
	CHECK_MESSAGE(acc.ok, acc.msg);
	CHECK(tested > 300);  // the grid really covers non saturated results
}

TEST_CASE("[C3 arm_clarke_q31] a pseudo random sweep with Ia over the full range and |Ib| <= 0.86 matches the saturated DEF within 4 LSB") {
	// |Ib| <= 0.86: product2 = 1.1547 * 0.86 = 0.993 stays in range. Ia over the whole q31 range.
	Acc acc;
	uint32_t seed = 777u;
	for (long n = 0; n < 4000; n++) {
		uint32_t ra = lcg(seed);
		uint32_t rb = lcg(seed);  // separate statements: evaluation order of the generator calls
		q31_t a = (q31_t)ra;
		q31_t b = q((double)(rb >> 8) / 16777216.0 * 1.72 - 0.86);
		q31_t al = 0, be = 0;
		arm_clarke_q31(a, b, &al, &be);
		double want = (double)clipQ31(defClarkeBeta((double)a, (double)b));
		acc.check(al == a, "alpha", n, al, a, 0);
		acc.check(std::fabs((double)be - want) <= 4.0, "beta", n, be, want, 4);
	}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

TEST_CASE("[C3 arm_inv_clarke_q31] Ia equals Ialpha exactly and Ib equals -Ialpha / 2 + sqrt(3) / 2 Ibeta within 3 LSB, saturated, over the full input range") {
	// grid of the full range (including the extreme values) and pseudo random values
	std::vector<q31_t> v = {INT32_MIN, INT32_MIN + 1, -1073741824, -1, 0, 1, 1073741824, INT32_MAX - 1, INT32_MAX};
	uint32_t seed = 4242u;
	for (int i = 0; i < 60; i++) v.push_back((q31_t)lcg(seed));
	Acc acc;
	long n = 0;
	for (q31_t a : v)
		for (q31_t b : v) {
			q31_t ia = 0, ib = 0;
			arm_inv_clarke_q31(a, b, &ia, &ib);
			double want = (double)clipQ31(defInvClarkeB((double)a, (double)b));
			acc.check(ia == a, "Ia", n, ia, a, 0);
			acc.check(std::fabs((double)ib - want) <= 3.0, "Ib", n, ib, want, 3);
			n++;
		}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

TEST_CASE("[C3 arm_park_q31] Id and Iq match the saturated rotation of (Ialpha, Ibeta) within 2 LSB over the full input range and angles from -2.5 to 5 turns") {
	std::vector<q31_t> v = {INT32_MIN, -1073741824, -1, 0, 1, 1073741824, INT32_MAX};
	uint32_t seed = 99u;
	for (int i = 0; i < 12; i++) v.push_back((q31_t)lcg(seed));
	Acc acc;
	long n = 0;
	for (size_t k = 0; k < 241; k += 2) {
		double t = 2.0 * kPi * ((double)k - 80.0) / 64.0;
		q31_t s = qSc(std::sin(t)), c = qSc(std::cos(t));
		for (q31_t a : v)
			for (q31_t b : v) {
				q31_t id = 0, iq = 0;
				arm_park_q31(a, b, &id, &iq, s, c);
				// Id = sum of two floors: error in (-2, 0] (one sided, plus 0.5 for the double reference); Iq =
				// difference of two floors: error in (-1, 1)
				double wd = clipD(defParkD((double)a, (double)b, s / kTwo31, c / kTwo31));
				double wq = clipD(defParkQ((double)a, (double)b, s / kTwo31, c / kTwo31));
				acc.check((double)id <= wd + 0.5 && (double)id > wd - 2.5, "Id (one sided)", n, id, wd, 2);
				acc.check(std::fabs((double)iq - wq) <= 1.5, "Iq", n, iq, wq, 1.5);
				n++;
			}
	}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

TEST_CASE("[C3 arm_inv_park_q31] Ialpha and Ibeta match the saturated inverse rotation of (Id, Iq) within 2 LSB over the full input range and angles from -2.5 to 5 turns") {
	std::vector<q31_t> v = {INT32_MIN, -1073741824, -1, 0, 1, 1073741824, INT32_MAX};
	uint32_t seed = 31337u;
	for (int i = 0; i < 12; i++) v.push_back((q31_t)lcg(seed));
	Acc acc;
	long n = 0;
	for (size_t k = 0; k < 241; k += 2) {
		double t = 2.0 * kPi * ((double)k - 80.0) / 64.0;
		q31_t s = qSc(std::sin(t)), c = qSc(std::cos(t));
		for (q31_t d : v)
			for (q31_t qq : v) {
				q31_t al = 0, be = 0;
				arm_inv_park_q31(d, qq, &al, &be, s, c);
				// Ialpha: difference of two floors, error in (-1, 1); Ibeta: sum of two floors, error in (-2, 0]
				double wa = clipD(defInvParkAlpha((double)d, (double)qq, s / kTwo31, c / kTwo31));
				double wb = clipD(defInvParkBeta((double)d, (double)qq, s / kTwo31, c / kTwo31));
				acc.check(std::fabs((double)al - wa) <= 1.5, "Ialpha", n, al, wa, 1.5);
				acc.check((double)be <= wb + 0.5 && (double)be > wb - 2.5, "Ibeta (one sided)", n, be, wb, 2);
				n++;
			}
	}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

// ---------------------------------------------------------------------------------------------------------------
// q31, scale and known angles
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_clarke_q31] the scale is 1/sqrt(3) for Ia and 2/sqrt(3) for Ib: half scale inputs give 0.28868 and 0.57735") {
	// Hand derivation: Ia = 0.5 -> Ibeta = 0.5 / sqrt(3) = 0.288675 (x 2^31 = 619925130.6); Ib = 0.5 -> 1 / sqrt(3) = 0.57735
	q31_t al = 0, be = 0;
	arm_clarke_q31(0x40000000, 0, &al, &be);
	CHECK(std::fabs((double)be - 0.5 / kSqrt3 * kTwo31) <= 2.0);
	arm_clarke_q31(0, 0x40000000, &al, &be);
	CHECK(std::fabs((double)be - 1.0 / kSqrt3 * kTwo31) <= 2.0);
	arm_clarke_q31(-0x40000000, 0, &al, &be);
	CHECK(std::fabs((double)be + 0.5 / kSqrt3 * kTwo31) <= 2.0);
}

TEST_CASE("[C3 arm_inv_clarke_q31] the scale is -1/2 for Ialpha and sqrt(3)/2 for Ibeta: half scale inputs give -0.25 and 0.43301") {
	q31_t ia = 0, ib = 0;
	arm_inv_clarke_q31(0x40000000, 0, &ia, &ib);  // -0.5 * 0.5 = -0.25
	CHECK(ib == -0x20000000);
	arm_inv_clarke_q31(0, 0x40000000, &ia, &ib);  // sqrt(3)/2 * 0.5 = 0.4330127
	CHECK(std::fabs((double)ib - kSqrt3 / 4.0 * kTwo31) <= 2.0);
}

TEST_CASE("[C3 arm_clarke_q31 arm_inv_clarke_q31] balanced three phase currents of amplitude 0.5 give (0.5 cos t, 0.5 sin t) within 8 LSB") {
	// Ia = 0.5 cos t, Ib = 0.5 cos(t - 2pi/3) (hand derivation in the header), amplitude 0.5 keeps all values in range.
	Acc acc;
	for (int k = -48; k <= 96; k++) {
		double t = 2.0 * kPi * (double)k / 48.0;
		q31_t a = q(0.5 * std::cos(t)), b = q(0.5 * std::cos(t - 2.0 * kPi / 3.0)), al = 0, be = 0;
		arm_clarke_q31(a, b, &al, &be);
		acc.check(std::fabs((double)al - 0.5 * std::cos(t) * kTwo31) <= 1.0, "alpha", k, al, 0.5 * std::cos(t) * kTwo31, 1);
		acc.check(std::fabs((double)be - 0.5 * std::sin(t) * kTwo31) <= 8.0, "beta", k, be, 0.5 * std::sin(t) * kTwo31, 8);
	}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

TEST_CASE("[C3 arm_park_q31] the angles 0, 90, 180 and 270 degrees map (Ialpha, Ibeta) to the rotated components within 1 LSB") {
	// full scale sin / cos is 0x7FFFFFFF = 1 - 2^-31: floor(x * (1 - 2^-31)) differs from x by at most 1
	const q31_t one = INT32_MAX, al = 0x30000000, be = -0x10000000;
	q31_t id = 0, iq = 0;
	arm_park_q31(al, be, &id, &iq, 0, one);  // 0 deg
	CHECK(std::llabs((int64_t)id - al) <= 1);
	CHECK(std::llabs((int64_t)iq - be) <= 1);
	arm_park_q31(al, be, &id, &iq, one, 0);  // 90 deg: Id = Ibeta, Iq = -Ialpha
	CHECK(std::llabs((int64_t)id - be) <= 1);
	CHECK(std::llabs((int64_t)iq + al) <= 1);
	arm_park_q31(al, be, &id, &iq, 0, -one);  // 180 deg
	CHECK(std::llabs((int64_t)id + al) <= 1);
	CHECK(std::llabs((int64_t)iq + be) <= 1);
	arm_park_q31(al, be, &id, &iq, -one, 0);  // 270 deg: Id = -Ibeta, Iq = Ialpha
	CHECK(std::llabs((int64_t)id + be) <= 1);
	CHECK(std::llabs((int64_t)iq - al) <= 1);
}

TEST_CASE("[C3 arm_inv_park_q31] the angles 0, 90, 180 and 270 degrees map (Id, Iq) to the rotated components within 1 LSB") {
	const q31_t one = INT32_MAX, d = 0x30000000, qq = -0x10000000;
	q31_t al = 0, be = 0;
	arm_inv_park_q31(d, qq, &al, &be, 0, one);  // 0 deg
	CHECK(std::llabs((int64_t)al - d) <= 1);
	CHECK(std::llabs((int64_t)be - qq) <= 1);
	arm_inv_park_q31(d, qq, &al, &be, one, 0);  // 90 deg: Ialpha = -Iq, Ibeta = Id
	CHECK(std::llabs((int64_t)al + qq) <= 1);
	CHECK(std::llabs((int64_t)be - d) <= 1);
	arm_inv_park_q31(d, qq, &al, &be, 0, -one);  // 180 deg
	CHECK(std::llabs((int64_t)al + d) <= 1);
	CHECK(std::llabs((int64_t)be + qq) <= 1);
	arm_inv_park_q31(d, qq, &al, &be, -one, 0);  // 270 deg: Ialpha = Iq, Ibeta = -Id
	CHECK(std::llabs((int64_t)al - qq) <= 1);
	CHECK(std::llabs((int64_t)be + d) <= 1);
}

// ---------------------------------------------------------------------------------------------------------------
// q31, round trips
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_clarke_q31 arm_inv_clarke_q31] the inverse Clarke transform undoes the Clarke transform within 8 LSB when no value saturates") {
	std::vector<q31_t> g = q31Grid();
	Acc acc;
	long n = 0, tested = 0;
	for (q31_t a : g)
		for (q31_t b : g) {
			if (std::fabs(defClarkeBeta((double)a, (double)b)) >= 0.99 * kTwo31) continue;  // Ibeta would saturate
			q31_t al = 0, be = 0, ra = 0, rb = 0;
			arm_clarke_q31(a, b, &al, &be);
			arm_inv_clarke_q31(al, be, &ra, &rb);
			acc.check(ra == a, "Ia", n, ra, a, 0);
			acc.check(std::llabs((int64_t)rb - b) <= 8, "Ib", n, rb, b, 8);
			tested++;
			n++;
		}
	CHECK_MESSAGE(acc.ok, acc.msg);
	CHECK(tested > 300);
}

TEST_CASE("[C3 arm_park_q31 arm_inv_park_q31] the inverse Park transform undoes the Park transform within 8 LSB when no value saturates") {
	// inputs |x| <= 0.5 keep |Id|, |Iq| <= 0.71 and the inverse in range
	std::vector<q31_t> v;
	for (int k = -8; k <= 8; k += 2) v.push_back((q31_t)((int64_t)k * (int64_t)(kTwo31 / 16.0)));
	uint32_t seed = 2024u;
	for (int i = 0; i < 6; i++) v.push_back(q(((double)(lcg(seed) >> 8) / 16777216.0) - 0.5));
	Acc acc;
	long n = 0;
	for (int k = -80; k <= 160; k += 3) {
		double t = 2.0 * kPi * (double)k / 64.0;
		q31_t s = qSc(std::sin(t)), c = qSc(std::cos(t));
		for (q31_t a : v)
			for (q31_t b : v) {
				q31_t id = 0, iq = 0, ra = 0, rb = 0;
				arm_park_q31(a, b, &id, &iq, s, c);
				arm_inv_park_q31(id, iq, &ra, &rb, s, c);
				acc.check(std::llabs((int64_t)ra - a) <= 8, "Ialpha", n, ra, a, 8);
				acc.check(std::llabs((int64_t)rb - b) <= 8, "Ibeta", n, rb, b, 8);
				n++;
			}
	}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

// ---------------------------------------------------------------------------------------------------------------
// C4: saturation of q31 results
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C4 arm_clarke_q31] a sum of the two products beyond full scale saturates to the q31 limits") {
	// DOC: "There is saturation on the addition". Ia = 0.5 -> 0.28868, Ib = 0.8 -> 0.92376 (in range): sum 1.2124 > 1
	const q31_t a = 0x40000000, b = (q31_t)(0.8 * kTwo31);
	q31_t al = 0, be = 0;
	arm_clarke_q31(a, b, &al, &be);
	CHECK(be == INT32_MAX);
	arm_clarke_q31(-a, -b, &al, &be);
	CHECK(be == INT32_MIN);
	arm_clarke_q31(INT32_MAX, (q31_t)(0.8 * kTwo31), &al, &be);  // 0.5774 + 0.9238
	CHECK(be == INT32_MAX);
	arm_clarke_q31(INT32_MIN, (q31_t)(-0.8 * kTwo31), &al, &be);
	CHECK(be == INT32_MIN);
}

TEST_CASE("[C4 arm_clarke_q31] 2/sqrt(3) Ib above full scale saturates Ibeta instead of wrapping around" * KNOWN_ISSUE("B-DCT-1: arm_clarke_q31 casts (Ib * 0x49E69D16) >> 30 to q31_t before the saturating add, so 2/sqrt(3) * Ib > 1.0 wraps to a negative value (Ia = 0, Ib = 0.9 gives about -0.96 instead of the saturated +1.0). DOC (controller_functions.h arm_clarke_q31): \"There is saturation on the addition, hence there is no risk of overflow.\"")) {
	// Ib = 0.9: 2/sqrt(3) * 0.9 = 1.0392 > 1 -> intended result INT32_MAX (saturated); with Ia = 0.5 as well: 1.3279.
	q31_t al = 0, be = 0;
	arm_clarke_q31(0, (q31_t)(0.9 * kTwo31), &al, &be);
	CHECK(be == INT32_MAX);  // first failing assertion: the defect
	arm_clarke_q31(0x40000000, (q31_t)(0.9 * kTwo31), &al, &be);
	CHECK(be == INT32_MAX);
	arm_clarke_q31(0, (q31_t)(-0.9 * kTwo31), &al, &be);
	CHECK(be == INT32_MIN);
	arm_clarke_q31(0, INT32_MAX, &al, &be);
	CHECK(be == INT32_MAX);
	arm_clarke_q31(0, INT32_MIN, &al, &be);
	CHECK(be == INT32_MIN);
}

TEST_CASE("[C4 arm_inv_clarke_q31] a difference beyond full scale saturates Ib to the q31 limits") {
	// Ib = sqrt(3)/2 Ibeta - Ialpha/2: Ialpha = -0.9, Ibeta = 0.9 -> 0.45 + 0.7794 = 1.2294 > 1
	q31_t ia = 0, ib = 0;
	arm_inv_clarke_q31((q31_t)(-0.9 * kTwo31), (q31_t)(0.9 * kTwo31), &ia, &ib);
	CHECK(ib == INT32_MAX);
	arm_inv_clarke_q31((q31_t)(0.9 * kTwo31), (q31_t)(-0.9 * kTwo31), &ia, &ib);
	CHECK(ib == INT32_MIN);
	arm_inv_clarke_q31(INT32_MIN, INT32_MAX, &ia, &ib);
	CHECK(ib == INT32_MAX);
	CHECK(ia == INT32_MIN);
	arm_inv_clarke_q31(INT32_MAX, INT32_MIN, &ia, &ib);
	CHECK(ib == INT32_MIN);
}

TEST_CASE("[C4 arm_park_q31] sums beyond full scale saturate Id and Iq to the q31 limits") {
	// 45 degrees: sin = cos = 0.70711. Id = 0.7071 (Ialpha + Ibeta), Iq = 0.7071 (Ibeta - Ialpha)
	const q31_t sc = (q31_t)(std::sqrt(0.5) * kTwo31), x = (q31_t)(0.9 * kTwo31);
	q31_t id = 0, iq = 0;
	arm_park_q31(x, x, &id, &iq, sc, sc);  // Id = 1.2728 -> max; Iq = 0
	CHECK(id == INT32_MAX);
	CHECK(std::llabs((int64_t)iq) <= 1);
	arm_park_q31(-x, -x, &id, &iq, sc, sc);  // Id = -1.2728 -> min
	CHECK(id == INT32_MIN);
	arm_park_q31(-x, x, &id, &iq, sc, sc);  // Iq = 1.2728 -> max; Id = 0
	CHECK(iq == INT32_MAX);
	CHECK(std::llabs((int64_t)id) <= 1);
	arm_park_q31(x, -x, &id, &iq, sc, sc);  // Iq = -1.2728 -> min
	CHECK(iq == INT32_MIN);
}

TEST_CASE("[C4 arm_inv_park_q31] sums and differences beyond full scale saturate Ialpha and Ibeta to the q31 limits") {
	const q31_t sc = (q31_t)(std::sqrt(0.5) * kTwo31), x = (q31_t)(0.9 * kTwo31);
	q31_t al = 0, be = 0;
	arm_inv_park_q31(x, -x, &al, &be, sc, sc);  // Ialpha = Id cos - Iq sin = 1.2728 -> max; Ibeta = 0
	CHECK(al == INT32_MAX);
	CHECK(std::llabs((int64_t)be) <= 1);
	arm_inv_park_q31(-x, x, &al, &be, sc, sc);  // Ialpha = -1.2728 -> min
	CHECK(al == INT32_MIN);
	arm_inv_park_q31(x, x, &al, &be, sc, sc);  // Ibeta = 1.2728 -> max; Ialpha = 0
	CHECK(be == INT32_MAX);
	CHECK(std::llabs((int64_t)al) <= 1);
	arm_inv_park_q31(-x, -x, &al, &be, sc, sc);  // Ibeta = -1.2728 -> min
	CHECK(be == INT32_MIN);
}

TEST_CASE("[C4 arm_park_q31 arm_inv_park_q31] the product (-1.0) * (-1.0) saturates to the largest q31 value instead of wrapping" * KNOWN_ISSUE("B-DCT-2: arm_park_q31 / arm_inv_park_q31 cast ((q63_t)x * y) >> 31 to q31_t before the saturating add, so INT32_MIN * INT32_MIN (= +1.0, 2^31 after the shift) wraps to INT32_MIN (-1.0). DOC: \"There is saturation on the addition and subtraction, hence there is no risk of overflow.\"")) {
	// Ialpha = -1.0 and cosVal = -1.0 (angle 180 degrees with the exact value -1.0): Id = 1.0 -> saturated INT32_MAX
	q31_t id = 0, iq = 0, al = 0, be = 0;
	arm_park_q31(INT32_MIN, 0, &id, &iq, 0, INT32_MIN);
	CHECK(id == INT32_MAX);  // first failing assertion: the defect
	arm_inv_park_q31(INT32_MIN, 0, &al, &be, 0, INT32_MIN);
	CHECK(al == INT32_MAX);
}

// ---------------------------------------------------------------------------------------------------------------
// C5: q31 against f32
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C5 arm_clarke_q31] agrees with arm_clarke_f32 after conversion within the f32 bound plus 4 LSB") {
	// inputs are multiples of 1/16, exact in float; compare in units of full scale
	std::vector<q31_t> g = q31Grid();
	Acc acc;
	long n = 0;
	for (q31_t a : g)
		for (q31_t b : g) {
			float fa = (float)((double)a / kTwo31), fb = (float)((double)b / kTwo31), fal = 0, fbe = 0;
			arm_clarke_f32(fa, fb, &fal, &fbe);
			q31_t al = 0, be = 0;
			arm_clarke_q31(a, b, &al, &be);
			double fwant = std::fmax(-1.0, std::fmin((double)fbe, 1.0));  // q31 saturates
			double bound = 3e-7 * (std::fabs((double)fa) + std::fabs((double)fb)) + 4.0 / kTwo31;
			acc.check((double)al / kTwo31 == (double)fal, "alpha", n, (double)al / kTwo31, fal, 0);
			acc.check(std::fabs((double)be / kTwo31 - fwant) <= bound, "beta", n, (double)be / kTwo31, fwant, bound);
			n++;
		}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

TEST_CASE("[C5 arm_inv_clarke_q31] agrees with arm_inv_clarke_f32 after conversion within the f32 bound plus 3 LSB") {
	std::vector<q31_t> g = q31Grid();
	Acc acc;
	long n = 0;
	for (q31_t a : g)
		for (q31_t b : g) {
			float fa = (float)((double)a / kTwo31), fb = (float)((double)b / kTwo31), fia = 0, fib = 0;
			arm_inv_clarke_f32(fa, fb, &fia, &fib);
			q31_t ia = 0, ib = 0;
			arm_inv_clarke_q31(a, b, &ia, &ib);
			double fwant = std::fmax(-1.0, std::fmin((double)fib, 1.0));
			double bound = 3e-7 * (std::fabs((double)fa) + std::fabs((double)fb)) + 3.0 / kTwo31;
			acc.check((double)ia / kTwo31 == (double)fia, "Ia", n, (double)ia / kTwo31, fia, 0);
			acc.check(std::fabs((double)ib / kTwo31 - fwant) <= bound, "Ib", n, (double)ib / kTwo31, fwant, bound);
			n++;
		}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

TEST_CASE("[C5 arm_park_q31] agrees with arm_park_f32 after conversion within the f32 bound plus the sin / cos rounding and 2 LSB") {
	std::vector<q31_t> g = q31Grid();
	Acc acc;
	long n = 0;
	for (int k = -80; k <= 160; k += 3) {
		double t = 2.0 * kPi * (double)k / 64.0;
		q31_t sq = q(std::sin(t)), cq = q(std::cos(t));
		float sf = (float)((double)sq / kTwo31), cf = (float)((double)cq / kTwo31);
		for (size_t i = 0; i < g.size(); i += 2)
			for (size_t j = 0; j < g.size(); j += 2) {
				float fa = (float)((double)g[i] / kTwo31), fb = (float)((double)g[j] / kTwo31), fd = 0, fq = 0;
				arm_park_f32(fa, fb, &fd, &fq, sf, cf);
				q31_t id = 0, iq = 0;
				arm_park_q31(g[i], g[j], &id, &iq, sq, cq);
				double bound = 5e-7 * (std::fabs((double)fa) + std::fabs((double)fb)) + 2.0 / kTwo31;
				double wd = std::fmax(-1.0, std::fmin((double)fd, 1.0)), wq = std::fmax(-1.0, std::fmin((double)fq, 1.0));
				acc.check(std::fabs((double)id / kTwo31 - wd) <= bound, "Id", n, (double)id / kTwo31, wd, bound);
				acc.check(std::fabs((double)iq / kTwo31 - wq) <= bound, "Iq", n, (double)iq / kTwo31, wq, bound);
				n++;
			}
	}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

TEST_CASE("[C5 arm_inv_park_q31] agrees with arm_inv_park_f32 after conversion within the f32 bound plus the sin / cos rounding and 2 LSB") {
	std::vector<q31_t> g = q31Grid();
	Acc acc;
	long n = 0;
	for (int k = -80; k <= 160; k += 3) {
		double t = 2.0 * kPi * (double)k / 64.0;
		q31_t sq = q(std::sin(t)), cq = q(std::cos(t));
		float sf = (float)((double)sq / kTwo31), cf = (float)((double)cq / kTwo31);
		for (size_t i = 0; i < g.size(); i += 2)
			for (size_t j = 0; j < g.size(); j += 2) {
				float fd = (float)((double)g[i] / kTwo31), fq = (float)((double)g[j] / kTwo31), fa = 0, fb = 0;
				arm_inv_park_f32(fd, fq, &fa, &fb, sf, cf);
				q31_t al = 0, be = 0;
				arm_inv_park_q31(g[i], g[j], &al, &be, sq, cq);
				double bound = 5e-7 * (std::fabs((double)fd) + std::fabs((double)fq)) + 2.0 / kTwo31;
				double wa = std::fmax(-1.0, std::fmin((double)fa, 1.0)), wb = std::fmax(-1.0, std::fmin((double)fb, 1.0));
				acc.check(std::fabs((double)al / kTwo31 - wa) <= bound, "Ialpha", n, (double)al / kTwo31, wa, bound);
				acc.check(std::fabs((double)be / kTwo31 - wb) <= bound, "Ibeta", n, (double)be / kTwo31, wb, bound);
				n++;
			}
	}
	CHECK_MESSAGE(acc.ok, acc.msg);
}

}  // TEST_SUITE
