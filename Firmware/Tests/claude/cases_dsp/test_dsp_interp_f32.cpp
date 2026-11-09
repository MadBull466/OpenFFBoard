/*
 * CMSIS-DSP Interpolation f32 (work order IF, phase P11 group G2). Requirement ids: Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * Units covered: arm_linear_interp_f32, arm_bilinear_interp_f32, arm_spline_init_f32 / arm_spline_f32.
 *
 * Pattern files used (Libraries/CMSIS-DSP/Testing/Patterns/DSP/Interpolation/InterpolationF32/), ARM's test functions are in
 * Testing/Source/Tests/InterpolationTestsF32.cpp and the suite "Interpolation Tests F32" in Testing/desc.txt:844-:885:
 *   linear   : Input1_f32 (42) , YVals1_f32 (41 table values, x1 = 0, xSpacing = 1)  -> Reference1_f32 (42)
 *              test_linear_interp_f32 (InterpolationTestsF32.cpp:19), setUp :133-:146
 *   bilinear : Input2_f32 (300 = 150 (x,y) pairs), Config2_s16 (numCols 7, numRows 8), YVals2_f32 (56) -> Reference2_f32 (150)
 *              test_bilinear_interp_f32 (:38), setUp :148-:162
 *   spline   : square: InputX3/InputY3 (4 knots), OutputX3 (20 queries) -> Reference3, ARM_SPLINE_PARABOLIC_RUNOUT, (:57)
 *              sine  : InputX4/InputY4 (9 knots),  OutputX4 (33 queries) -> Reference4, ARM_SPLINE_NATURAL (:73; the work order
 *                      says "PARABOLIC_RUNOUT square, sine, ramp", ARM's sine test uses the NATURAL type)
 *              ramp  : InputX5/InputY5 (3 knots),  OutputX5 (30 queries) -> Reference5, ARM_SPLINE_PARABOLIC_RUNOUT (:89)
 * All five ARM test ids of the suite are covered (desc.txt:878-:883; nothing is commented out or disabled).
 *
 * ARM tolerances (InterpolationTestsF32.cpp:5-:13): SNR_THRESHOLD 119 dB (:5); REL_ERROR 8.0e-5 (:13) for linear and bilinear,
 * ASSERT_REL_ERROR of ARM's framework (FrameworkSource/Error.cpp:218) is |a-b| / ((|a|+|b|)/2) <= REL (not relative to the
 * reference alone, and nothing is compared if both are 0); the helper armRel() below does the same but also fails on NaN
 * (stricter than ARM). The spline tests only assert the SNR (and the empty tail of the buffers: guards here).
 *
 * Suspected vendor bug of the spline (suspected bug 2 in PROGRESS "Known issues", arm_spline_interp_f32.c:223):
 * arm_spline_f32 reads one float past the query block (`while (*pXq <= x[i+1] && blkCnt > 0)` dereferences before the count
 * check). Same convention as claude/cases_target/test_cmsis_dsp.cpp: every query vector passed to arm_spline_f32 in this file
 * has one extra element behind the block (padded()), so the over-read stays inside the test's array and sanitizer builds can
 * run these tests; the extra element never changes a result. The defect itself is shown by ONE test with an exactly sized
 * one element query vector (the call of EffectsCalculator), marked KNOWN_ISSUE_UB_HIDDEN (passes on the host, skipped in
 * sanitizer builds, where ASan reports it with --no-skip).
 *
 * Host vs target (C6): the host runs the portable C path (no ARM_MATH_DSP / MVE / Neon). (1) The target (-O2 -std=gnu11) may
 * contract a*b+c into VFMA (-ffp-contract=fast), the host does not: results can differ in the last bits. Tests with a tolerance
 * have it derived below; the "exact" tests use dyadic data (multiples of a power of two with a small magnitude), for which
 * every intermediate value is exactly representable in binary32, so a fused multiply-add gives the same result. (2) The index
 * computation `(int32_t)((x - x1) / xSpacing)` (arm_linear_interp_f32.c) and `(int32_t)X` (arm_bilinear_interp_f32.c) is
 * undefined for a float outside int32 range: x86 gives INT_MIN, ARM saturates (INT_MAX / INT_MIN, NaN -> 0). The two tests
 * that feed such values are marked KNOWN_ISSUE_UB_HIDDEN (the result is the intended one on both platforms). (3) uint16_t
 * numRows / numCols are promoted to int before `numCols - 2`: no difference between host and target.
 */
#include "doctest.h"

#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math.h"

namespace {

const char *const kDir = "Interpolation/InterpolationF32/";

// ARM constants (InterpolationTestsF32.cpp:5, :13)
const double kSnr = 119.0;
const double kRel = 8.0e-5;

std::vector<float> pattern(const char *name) { return dsp::loadPattern(std::string(kDir) + name).f32(); }

// ARM's ASSERT_REL_ERROR (FrameworkSource/Error.cpp:218), plus NaN fails
dsp::Check armRel(const std::vector<float> &ref, const std::vector<float> &out, double thr) {
	if (ref.size() != out.size()) return dsp::detail::lengthMismatch(ref.size(), out.size());
	for (size_t i = 0; i < ref.size(); i++) {
		const double a = out[i], b = ref[i];
		if (std::isnan(a) || std::isnan(b)) return dsp::detail::fail("relative error", i, a, b, thr);
		const double avg = (std::fabs(a) + std::fabs(b)) / 2.0;
		if (avg != 0.0 && std::fabs(a - b) / avg > thr) return dsp::detail::fail("relative error", i, a, b, thr);
	}
	return dsp::Check();
}

std::vector<float> floatsFrom(const std::vector<double> &v) { return std::vector<float>(v.begin(), v.end()); }

// One extra element behind the block, see the file header (suspected bug 2)
std::vector<float> padded(const std::vector<float> &q) {
	std::vector<float> p(q);
	p.push_back(FLT_MAX);
	return p;
}

bool allPoison(const std::vector<float> &o) {
	for (float f : o) {
		unsigned char b[sizeof(float)];
		std::memcpy(b, &f, sizeof b);
		for (unsigned char c : b)
			if (c != 0xCD) return false;
	}
	return true;
}

// ---------------------------------------------------------------------------------------------------------------
// DEF of the linear interpolation (arm_linear_interp_f32.c doc "y = y0 + (x - x0) * ((y1 - y0)/(x1-x0))" and "if x is outside
// of the table boundary, Linear interpolation returns first value of the table if x is below input range and returns last
// value of table if x is above range"), double precision
// ---------------------------------------------------------------------------------------------------------------
double defLinear(const std::vector<float> &y, double x1, double h, double x) {
	const size_t n = y.size();
	if (x < x1) return y[0];
	const double pos = (x - x1) / h;
	if (pos >= static_cast<double>(n - 1)) return y[n - 1];
	const size_t i = static_cast<size_t>(std::floor(pos));
	return y[i] + (x - (x1 + i * h)) / h * (static_cast<double>(y[i + 1]) - y[i]);
}

// DEF of the bilinear interpolation (arm_bilinear_interp_f32.c doc), table element (x,y) is data[x + y*numCols]
double defBilinear(const std::vector<float> &d, int cols, double X, double Y) {
	const int xi = static_cast<int>(std::floor(X)), yi = static_cast<int>(std::floor(Y));
	const double fx = X - xi, fy = Y - yi;
	return d[xi + yi * cols] * (1 - fx) * (1 - fy) + d[xi + 1 + yi * cols] * fx * (1 - fy) +
	       d[xi + (yi + 1) * cols] * (1 - fx) * fy + d[xi + 1 + (yi + 1) * cols] * fx * fy;
}

// ---------------------------------------------------------------------------------------------------------------
// DEF of the cubic spline: the tridiagonal system of the documentation (arm_spline_interp_f32.c:63-:90)
//   row 0      : c(0) - t c(1) = 0                           t = 0 natural, t = 1 parabolic runout
//   row i      : h(i-1) c(i-1) + 2 (h(i-1) + h(i)) c(i) + h(i) c(i+1) = 3/h(i) (y(i+1)-y(i)) - 3/h(i-1) (y(i)-y(i-1))
//   row n-1    : -t c(n-2) + c(n-1) = 0
//   b(i) = (y(i+1)-y(i))/h(i) - h(i) (c(i+1) + 2 c(i)) / 3,  d(i) = (c(i+1) - c(i)) / (3 h(i)),  a(i) = y(i)
//   S_i(x) = y(i) + b(i) t + c(i) t^2 + d(i) t^3, t = x - x(i); queries below x(0) use S_0, above x(n-1) use S_(n-2)
//   (doc "Behaviour outside the given intervals"). Solved here with the Thomas algorithm in double (the library uses the Crout
//   recursion in float; the two are different code).
// ---------------------------------------------------------------------------------------------------------------
struct SplineDef {
	std::vector<double> x, y, b, c, d;
	SplineDef(const std::vector<float> &xs, const std::vector<float> &ys, arm_spline_type type)
	    : x(xs.begin(), xs.end()), y(ys.begin(), ys.end()) {
		const size_t n = x.size();
		const double t = (type == ARM_SPLINE_PARABOLIC_RUNOUT) ? 1.0 : 0.0;
		std::vector<double> lo(n, 0.0), di(n, 0.0), up(n, 0.0), rhs(n, 0.0);
		di[0] = 1;
		up[0] = -t;
		lo[n - 1] = -t;
		di[n - 1] = 1;
		for (size_t i = 1; i + 1 < n; i++) {
			const double h0 = x[i] - x[i - 1], h1 = x[i + 1] - x[i];
			lo[i] = h0;
			di[i] = 2 * (h0 + h1);
			up[i] = h1;
			rhs[i] = 3 / h1 * (y[i + 1] - y[i]) - 3 / h0 * (y[i] - y[i - 1]);
		}
		for (size_t i = 1; i < n; i++) {   // forward elimination
			const double w = lo[i] / di[i - 1];
			di[i] -= w * up[i - 1];
			rhs[i] -= w * rhs[i - 1];
		}
		c.assign(n, 0.0);
		c[n - 1] = rhs[n - 1] / di[n - 1];
		for (size_t i = n - 1; i-- > 0;) c[i] = (rhs[i] - up[i] * c[i + 1]) / di[i];
		for (size_t i = 0; i + 1 < n; i++) {
			const double h = x[i + 1] - x[i];
			b.push_back((y[i + 1] - y[i]) / h - h * (c[i + 1] + 2 * c[i]) / 3);
			d.push_back((c[i + 1] - c[i]) / (3 * h));
		}
	}
	size_t interval(double q) const {
		size_t i = 0;
		while (i + 2 < x.size() && q > x[i + 1]) i++;
		return i;
	}
	double eval(double q) const {
		const size_t i = interval(q);
		const double t = q - x[i];
		return y[i] + b[i] * t + c[i] * t * t + d[i] * t * t * t;
	}
	// sum of the magnitudes of the terms: scale of the rounding error of the evaluation
	double terms(double q) const {
		const size_t i = interval(q);
		const double t = std::fabs(q - x[i]);
		return std::fabs(y[i]) + std::fabs(b[i]) * t + std::fabs(c[i]) * t * t + std::fabs(d[i]) * t * t * t;
	}
};

// Bound of the float library against the double DEF. The float recursion has about 10 operations per coefficient on a
// diagonally dominant system (diagonal 2(h0+h1) against off-diagonals h0+h1: the elimination does not amplify more than a
// factor 3), every operation adds a relative error of at most 2^-24 = 6e-8, so the coefficients carry a relative error of
// about 3 * 10 * 6e-8 = 2e-6 and the evaluation (terms of the size terms()) one more; 2e-5 (10x margin) * (1 + terms) is used.
double splineBound(const SplineDef &def, double q) { return 2.0e-5 * (1.0 + def.terms(q)); }

// A spline in the library with guarded coefficient and temporary buffers (sizes 3 (n-1) and 2n-1, doc of
// arm_spline_interp_init_f32.c: "size must be 3*(n-1)", "its size is n+n-1")
struct LibSpline {
	std::vector<float> x, y;
	dsp::GuardedBuffer<float> coeffs, temp;
	arm_spline_instance_f32 S;
	bool outGuardOk = true;
	LibSpline(arm_spline_type type, std::vector<float> xs, std::vector<float> ys)
	    : x(std::move(xs)), y(std::move(ys)), coeffs(3 * (x.size() - 1)), temp(2 * x.size() - 1) {
		std::memset(&S, 0, sizeof S);
		arm_spline_init_f32(&S, type, x.data(), y.data(), static_cast<uint32_t>(x.size()), coeffs.data(), temp.data());
	}
	// evaluates the block (padded copy, see the file header)
	std::vector<float> eval(const std::vector<float> &q) {
		const std::vector<float> p = padded(q);
		dsp::GuardedBuffer<float> out(q.size());
		arm_spline_f32(&S, p.data(), out.data(), static_cast<uint32_t>(q.size()));
		outGuardOk = out.guardIntact();
		return out.output();
	}
};

// Knots of the spline tests: non uniform spacing, value changes of both signs
const std::vector<float> kKnotsX = {-3.0f, -1.5f, -0.2f, 1.0f, 2.5f, 4.0f};
const std::vector<float> kKnotsY = {2.0f, -1.0f, 0.5f, 3.0f, 1.25f, -2.0f};

// ascending query sweep from lo to hi, step st (in float arithmetic of the loop index, so strictly ascending)
std::vector<float> sweep(double lo, double hi, double st) {
	std::vector<float> q;
	for (int k = 0; lo + k * st <= hi; k++) q.push_back(static_cast<float>(lo + k * st));
	return q;
}

void compareSpline(const SplineDef &def, const std::vector<float> &q, const std::vector<float> &out) {
	REQUIRE(out.size() == q.size());
	size_t bad = q.size();
	double worstRatio = 0.0;
	for (size_t i = 0; i < q.size(); i++) {
		const double err = std::fabs(static_cast<double>(out[i]) - def.eval(q[i]));
		const double ratio = err / splineBound(def, q[i]);
		if (!(err <= splineBound(def, q[i])) && bad == q.size()) bad = i;
		if (!(ratio <= worstRatio)) worstRatio = ratio;
	}
	INFO("first bad index " << bad << (bad < q.size() ? " query " : "") << (bad < q.size() ? q[bad] : 0.0f)
	     << ", worst error / bound " << worstRatio);
	CHECK(bad == q.size());
}

}  // namespace

TEST_SUITE("DSP Interpolation f32") {

// ---------------------------------------------------------------------------------------------------------------
// C1: ARM's tests
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C1 arm_linear_interp_f32] linear interpolation of ARM's table over ARM's 42 inputs matches ARM's reference within 119 dB and 8e-5 relative") {
	const std::vector<float> in = pattern("Input1_f32.txt");
	std::vector<float> table = pattern("YVals1_f32.txt");
	const std::vector<float> ref = pattern("Reference1_f32.txt");
	REQUIRE(in.size() == ref.size());
	arm_linear_interp_instance_f32 S;
	S.nValues = static_cast<uint32_t>(table.size());   // 41, as in setUp (:140)
	S.x1 = 0.0f;
	S.xSpacing = 1.0f;
	S.pYData = table.data();
	std::vector<float> out;
	for (float x : in) out.push_back(arm_linear_interp_f32(&S, x));
	CHECK(dsp::snr(ref, out) >= kSnr);
	auto c = armRel(ref, out, kRel);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C1 arm_bilinear_interp_f32] bilinear interpolation of ARM's 7x8 table over ARM's 150 points matches ARM's reference within 119 dB and 8e-5 relative") {
	const std::vector<float> in = pattern("Input2_f32.txt");
	std::vector<float> table = pattern("YVals2_f32.txt");
	const std::vector<float> ref = pattern("Reference2_f32.txt");
	const std::vector<int16_t> cfg = dsp::loadPattern(std::string(kDir) + "Config2_s16.txt").q15();
	REQUIRE(cfg.size() == 2);
	REQUIRE(in.size() == 2 * ref.size());
	arm_bilinear_interp_instance_f32 S;
	S.numRows = static_cast<uint16_t>(cfg[1]);   // setUp :155-:156
	S.numCols = static_cast<uint16_t>(cfg[0]);
	S.pData = table.data();
	REQUIRE(table.size() == static_cast<size_t>(S.numRows) * S.numCols);
	std::vector<float> out;
	for (size_t i = 0; i < in.size(); i += 2) out.push_back(arm_bilinear_interp_f32(&S, in[i], in[i + 1]));
	CHECK(dsp::snr(ref, out) >= kSnr);
	auto c = armRel(ref, out, kRel);
	CHECK_MESSAGE(c, c.message);
}

namespace {
void armSpline(arm_spline_type type, const char *xName, const char *yName, const char *qName, const char *refName,
               size_t n, size_t nq) {
	const std::vector<float> xs = pattern(xName), ys = pattern(yName), q = pattern(qName), ref = pattern(refName);
	REQUIRE(xs.size() == n);
	REQUIRE(ys.size() == n);
	REQUIRE(q.size() == nq);
	REQUIRE(ref.size() == nq);
	LibSpline sp(type, xs, ys);
	const std::vector<float> out = sp.eval(q);
	CHECK(sp.coeffs.guardIntact());   // ARM: ASSERT_EMPTY_TAIL(splineCoefs)
	CHECK(sp.temp.guardIntact());     // ARM: ASSERT_EMPTY_TAIL(buffer)
	CHECK(sp.outGuardOk);             // ARM: ASSERT_EMPTY_TAIL(output)
	CHECK(dsp::snr(ref, out) >= kSnr);
}
}  // namespace

TEST_CASE("[C1 arm_spline_f32] the parabolic runout spline of ARM's square (4 knots, 20 queries) reaches 119 dB against ARM's reference") {
	armSpline(ARM_SPLINE_PARABOLIC_RUNOUT, "InputX3_f32.txt", "InputY3_f32.txt", "OutputX3_f32.txt", "Reference3_f32.txt", 4, 20);
}

TEST_CASE("[C1 arm_spline_f32] the natural spline of ARM's sine (9 knots, 33 queries) reaches 119 dB against ARM's reference") {
	armSpline(ARM_SPLINE_NATURAL, "InputX4_f32.txt", "InputY4_f32.txt", "OutputX4_f32.txt", "Reference4_f32.txt", 9, 33);
}

TEST_CASE("[C1 arm_spline_f32] the parabolic runout spline of ARM's ramp (3 knots, 30 queries) reaches 119 dB against ARM's reference") {
	armSpline(ARM_SPLINE_PARABOLIC_RUNOUT, "InputX5_f32.txt", "InputY5_f32.txt", "OutputX5_f32.txt", "Reference5_f32.txt", 3, 30);
}

TEST_CASE("[C1 arm_spline_f32] a single query in an exactly sized one element vector (the call of EffectsCalculator) gives the independent double solution"
          * KNOWN_ISSUE_UB_HIDDEN("B-DIF-1: arm_spline_f32 (arm_spline_interp_f32.c:223) evaluates '*pXq <= x[i+1]' before the "
                                  "'blkCnt > 0' check and reads one float behind the query block (heap-buffer-overflow under ASan "
                                  "with --no-skip). The result is correct on the host")) {
	const SplineDef def(kKnotsX, kKnotsY, ARM_SPLINE_NATURAL);
	LibSpline sp(ARM_SPLINE_NATURAL, kKnotsX, kKnotsY);
	const std::vector<float> q(1, 0.3f);   // exactly one element: no element behind the block
	float out = 0.0f;
	arm_spline_f32(&sp.S, q.data(), &out, 1);
	CHECK(std::fabs(static_cast<double>(out) - def.eval(0.3f)) <= splineBound(def, 0.3f));
}

// ---------------------------------------------------------------------------------------------------------------
// C2: spline block sizes, guard words (linear and bilinear return a scalar: no output buffer)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C2 arm_spline_f32] every sample is correct for the block sizes 0, 1, 3, 4, 5 and 9 and nothing is written beyond them") {
	// ascending queries below, inside and above the knots; block n uses the first n of them
	const std::vector<float> allQ = {-4.0f, -3.0f, -2.0f, -1.5f, 0.0f, 1.0f, 2.5f, 3.3f, 5.0f};
	for (arm_spline_type type : {ARM_SPLINE_NATURAL, ARM_SPLINE_PARABOLIC_RUNOUT}) {
		const SplineDef def(kKnotsX, kKnotsY, type);
		for (uint32_t n : {0u, 1u, 3u, 4u, 5u, 9u}) {
			CAPTURE(static_cast<int>(type));
			CAPTURE(n);
			LibSpline sp(type, kKnotsX, kKnotsY);
			const std::vector<float> q(allQ.begin(), allQ.begin() + n);
			const std::vector<float> out = sp.eval(q);
			compareSpline(def, q, out);
			CHECK(sp.outGuardOk);
		}
	}
}

TEST_CASE("[C2 arm_spline_f32] a block of size 0 writes nothing") {
	LibSpline sp(ARM_SPLINE_NATURAL, kKnotsX, kKnotsY);
	const std::vector<float> p = padded({0.5f});
	dsp::GuardedBuffer<float> out(1);   // poisoned with 0xCD bytes
	arm_spline_f32(&sp.S, p.data(), out.data(), 0);
	CHECK(allPoison(out.output()));
	CHECK(out.guardIntact());
}

TEST_CASE("[C2 arm_spline_init_f32] initialisation writes only the 3 (n-1) coefficients and uses only 2n-1 temporary floats") {
	// doc (arm_spline_interp_init_f32.c): coeffs size 3*(n-1), tempBuffer size n+n-1. Both buffers have guards behind them.
	for (arm_spline_type type : {ARM_SPLINE_NATURAL, ARM_SPLINE_PARABOLIC_RUNOUT}) {
		CAPTURE(static_cast<int>(type));
		LibSpline sp(type, kKnotsX, kKnotsY);
		CHECK(sp.coeffs.guardIntact());
		CHECK(sp.temp.guardIntact());
	}
}

TEST_CASE("[C2 arm_spline_init_f32] initialisation stores the data pointers, the number of knots and the coefficient buffer in the instance and leaves the knots unchanged") {
	LibSpline sp(ARM_SPLINE_NATURAL, kKnotsX, kKnotsY);
	CHECK(sp.S.x == sp.x.data());
	CHECK(sp.S.y == sp.y.data());
	CHECK(sp.S.n_x == kKnotsX.size());
	CHECK(sp.S.coeffs == sp.coeffs.data());
	CHECK(dsp::exactEqual(kKnotsX, sp.x));
	CHECK(dsp::exactEqual(kKnotsY, sp.y));
}

// ---------------------------------------------------------------------------------------------------------------
// C3: DEF where ARM has no pattern
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_linear_interp_f32] a table with x1 = -2.5 and spacing 0.5 is interpolated exactly over the whole range and beyond it") {
	// DEF: defLinear(). Dyadic data: table values are multiples of 1/8 with |y| <= 4, the spacing is 0.5 and the queries are
	// multiples of 1/64: (y1-y0) is a multiple of 1/8 below 8, /0.5 doubles it, (x-x0) is a multiple of 1/64 below 0.5, the
	// product is a multiple of 1/256 below 8 and the sum a multiple of 1/256 below 16: 12 bits, exact in binary32 also if a
	// fused multiply-add is used (C6). Hence an exact comparison.
	std::vector<float> table = {1.5f, -2.25f, 0.125f, 3.875f, -1.0f, 2.5f, 0.75f, -3.5f, 1.0f};
	arm_linear_interp_instance_f32 S;
	S.nValues = static_cast<uint32_t>(table.size());
	S.x1 = -2.5f;
	S.xSpacing = 0.5f;
	S.pYData = table.data();
	std::vector<double> ref;
	std::vector<float> out;
	// from 1.5 below the first to 1.5 above the last table position (last position: -2.5 + 8 * 0.5 = 1.5): strided over the
	// whole range, every table position (multiples of 0.5) is hit
	for (int k = -4 * 64; k <= 3 * 64; k++) {
		const float x = static_cast<float>(k) / 64.0f;
		out.push_back(arm_linear_interp_f32(&S, x));
		ref.push_back(defLinear(table, -2.5, 0.5, x));
	}
	auto c = dsp::exactEqual(floatsFrom(ref), out);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C3 arm_linear_interp_f32] a table with x1 = 0.3 and spacing 0.1 is interpolated within the rounding error of the position") {
	// DEF: defLinear() with the float values of x1 and the spacing. Bound: the library computes x0 = x1 + i*h and x1' = x1 +
	// (i+1)*h in float (positions up to 1.3: two roundings of 6e-8 each, 1.2e-7 per position) and divides by their
	// difference: the interpolation fraction t = (x-x0)/(x1'-x0) has an error of 1.2e-7/0.1 = 1.2e-6 (numerator) plus
	// 2.4e-7/0.1 = 2.4e-6 (denominator, times t <= 1) = 3.6e-6; times |y1-y0| <= 4: 1.5e-5; the float roundings of the
	// slope, the product and the sum add 6e-8 * (4 + 4 + 2) = 6e-7. Bound 3e-5. Where the library takes the neighbour interval
	// at a table position, the function is continuous there and the same bound holds.
	std::vector<float> table = {0.5f, -1.7f, 2.0f, 0.3f, -0.9f, 1.1f, 1.9f, -2.0f, 0.0f, 0.8f, -1.3f};
	const float x1 = 0.3f, h = 0.1f;
	arm_linear_interp_instance_f32 S;
	S.nValues = static_cast<uint32_t>(table.size());
	S.x1 = x1;
	S.xSpacing = h;
	S.pYData = table.data();
	std::vector<double> ref;
	std::vector<float> out;
	std::vector<float> xs;
	for (int k = 0; k < 130; k++) xs.push_back(static_cast<float>(-0.4 + k * 0.0173));   // -0.4 .. 1.83, strided
	for (size_t i = 0; i < table.size(); i++) xs.push_back(static_cast<float>(static_cast<double>(x1) + i * static_cast<double>(h)));
	for (float x : xs) {
		out.push_back(arm_linear_interp_f32(&S, x));
		ref.push_back(defLinear(table, x1, h, x));
	}
	auto c = dsp::closeAbsRel(ref, out, 3.0e-5, 0.0);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C3 arm_bilinear_interp_f32] a 4x5 table is interpolated exactly at every 1/16 inside the table") {
	// DEF: defBilinear() (the formula of the doc). Dyadic data: table values are multiples of 1/8 with |f| <= 4, the queries
	// multiples of 1/16 below the last row / column, so b2, b3 are multiples of 1/8 below 8, b4 of 1/8 below 16, xdiff*ydiff
	// products of b4 are multiples of 1/2048 below 16 (15 bits) and the sums of 1/2048 below 32 (16 bits): exact in binary32,
	// also with a fused multiply-add (C6). Hence an exact comparison.
	const int cols = 4, rows = 5;
	std::vector<float> table = {1.0f, -2.5f, 3.75f, 0.125f,   -4.0f, 2.0f, 0.5f, -1.125f,   3.0f, 3.5f, -2.75f, 1.625f,
	                            0.25f, -3.0f, 4.0f, -0.625f,  2.25f, -1.5f, 1.0f, 3.875f};
	REQUIRE(table.size() == static_cast<size_t>(cols * rows));
	arm_bilinear_interp_instance_f32 S;
	S.numRows = rows;
	S.numCols = cols;
	S.pData = table.data();
	std::vector<double> ref;
	std::vector<float> out;
	for (int ky = 0; ky < (rows - 1) * 16; ky++) {
		for (int kx = 0; kx < (cols - 1) * 16; kx++) {
			const float X = static_cast<float>(kx) / 16.0f, Y = static_cast<float>(ky) / 16.0f;
			out.push_back(arm_bilinear_interp_f32(&S, X, Y));
			ref.push_back(defBilinear(table, cols, X, Y));
		}
	}
	auto c = dsp::exactEqual(floatsFrom(ref), out);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C3 arm_spline_f32] the natural spline of 6 non uniform knots equals the independent double solution over -5 to 6 (below, inside and above the knots)") {
	const SplineDef def(kKnotsX, kKnotsY, ARM_SPLINE_NATURAL);
	LibSpline sp(ARM_SPLINE_NATURAL, kKnotsX, kKnotsY);
	const std::vector<float> q = sweep(-5.0, 6.0, 0.037);   // 298 queries, strided over the whole range
	compareSpline(def, q, sp.eval(q));
	CHECK(sp.outGuardOk);
}

TEST_CASE("[C3 arm_spline_f32] the parabolic runout spline of 6 non uniform knots equals the independent double solution over -5 to 6") {
	const SplineDef def(kKnotsX, kKnotsY, ARM_SPLINE_PARABOLIC_RUNOUT);
	LibSpline sp(ARM_SPLINE_PARABOLIC_RUNOUT, kKnotsX, kKnotsY);
	const std::vector<float> q = sweep(-5.0, 6.0, 0.037);
	compareSpline(def, q, sp.eval(q));
	CHECK(sp.outGuardOk);
}

TEST_CASE("[C3 arm_spline_f32] the natural spline of 9 uniform knots of a sine equals the independent double solution") {
	// a second data set (uniform spacing 0.5, n = 9, values of sin) so that the result does not depend on the knots above
	std::vector<float> xs, ys;
	for (int i = 0; i < 9; i++) {
		xs.push_back(static_cast<float>(i) * 0.5f);
		ys.push_back(static_cast<float>(std::sin(i * 0.5)));
	}
	const SplineDef def(xs, ys, ARM_SPLINE_NATURAL);
	LibSpline sp(ARM_SPLINE_NATURAL, xs, ys);
	const std::vector<float> q = sweep(-0.5, 4.5, 0.0131);
	compareSpline(def, q, sp.eval(q));
}

TEST_CASE("[C3 arm_spline_f32] the natural spline of points on a straight line reproduces the line, also outside the knots") {
	// Hand derivation: for y = 1.5 - 0.5 x every right hand side of the system is 0, so c = 0 for both boundary types,
	// d = 0 and b = slope: S(x) = y(x). Float bound: y(x) is computed with the float knots; error <= 1e-5 (b from the
	// quotient of two float differences, few roundings, |x| <= 6).
	std::vector<float> xs = kKnotsX, ys;
	for (float x : xs) ys.push_back(1.5f - 0.5f * x);
	LibSpline sp(ARM_SPLINE_NATURAL, xs, ys);
	const std::vector<float> q = sweep(-6.0, 6.0, 0.05);
	const std::vector<float> out = sp.eval(q);
	std::vector<double> ref;
	for (float x : q) ref.push_back(1.5 - 0.5 * static_cast<double>(x));
	auto c = dsp::closeAbsRel(ref, out, 1.0e-5, 0.0);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C3 arm_spline_f32] the parabolic runout spline of points on a parabola reproduces the parabola, also outside the knots") {
	// Hand derivation: for y = 2 - x + 0.5 x^2 the slope between a and b is (a+b)/2 - 1, so (y2-y1)/h1 - (y1-y0)/h0 =
	// (x2-x0)/2 = (h0+h1)/2. The interior rows read h0 c + 2(h0+h1) c + h1 c = 3 (h0+h1) c for a constant c and have the right
	// hand side 3 (h0+h1)/2: c = 0.5 for all i solves them, with d = 0 and b = slope (the quadratic itself). The parabolic
	// runout rows c(0) = c(1) and c(n-1) = c(n-2) hold too. The system has one solution, so the spline is the parabola.
	// Float bound 2e-5 * (1 + |y|).
	std::vector<float> xs = kKnotsX, ys;
	for (float x : xs) ys.push_back(2.0f - x + 0.5f * x * x);
	LibSpline sp(ARM_SPLINE_PARABOLIC_RUNOUT, xs, ys);
	const std::vector<float> q = sweep(-6.0, 6.0, 0.05);
	const std::vector<float> out = sp.eval(q);
	size_t bad = q.size();
	for (size_t i = 0; i < q.size() && bad == q.size(); i++) {
		const double x = q[i], want = 2.0 - x + 0.5 * x * x;
		if (!(std::fabs(out[i] - want) <= 2.0e-5 * (1.0 + std::fabs(want)))) bad = i;
	}
	INFO("first bad index " << bad);
	CHECK(bad == q.size());
}

TEST_CASE("[C3 arm_spline_f32] the natural spline of constant data is exactly that constant, also outside the knots") {
	// Hand derivation: all differences of y are 0, the right hand sides are 0, so c = b = d = 0 in every operation: z and u are
	// products with 0 and every coefficient is 0 (or -0); S(x) = y + 0 + 0 + 0 = 2.5 exactly.
	std::vector<float> ys(kKnotsX.size(), 2.5f);
	LibSpline sp(ARM_SPLINE_NATURAL, kKnotsX, ys);
	const std::vector<float> q = sweep(-5.0, 6.0, 0.25);
	const std::vector<float> out = sp.eval(q);
	CHECK(dsp::exactEqual(std::vector<float>(q.size(), 2.5f), out));
}

TEST_CASE("[C3 arm_spline_f32] the natural spline has a zero second derivative at the first and at the last knot") {
	// Doc: "Natural spline: S1''(x1) = 2*c(1) = 0 ; Sn''(xn) = 2*c(n) = 0". Independent of the DEF solver: the polynomial of
	// the first interval is evaluated left and right of x(0) (documented extension below the range), and for a cubic the
	// second difference S(x+e) + S(x-e) - 2 S(x) equals e^2 S''(x) exactly. At x(0) the first polynomial is used on both
	// sides; at x(n-1) the last polynomial is used on both sides (documented extension above the range).
	// Float bound: values below 60 (the extrapolated cubic with e = 0.5), 6 roundings of 6e-8 relative plus the coefficient
	// error of 2e-6 relative: 5e-5 absolute (S'' e^2 = 0.25 * S'' would be several 0.1 for a boundary of another type).
	LibSpline sp(ARM_SPLINE_NATURAL, kKnotsX, kKnotsY);
	const float e = 0.5f;
	const float x0 = kKnotsX.front(), xn = kKnotsX.back();
	const std::vector<float> o = sp.eval({x0 - e, x0, x0 + e});
	CHECK(std::fabs(static_cast<double>(o[0]) + o[2] - 2.0 * o[1]) <= 5.0e-5);
	const std::vector<float> p = sp.eval({xn - e, xn, xn + e});
	CHECK(std::fabs(static_cast<double>(p[0]) + p[2] - 2.0 * p[1]) <= 5.0e-5);
}

// ---------------------------------------------------------------------------------------------------------------
// C4: documented edge behaviour
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C4 arm_linear_interp_f32] an input below the first table position returns the first table value and one above the last returns the last") {
	// DOC (arm_linear_interp_f32.c): "if x is outside of the table boundary, Linear interpolation returns first value of the
	// table if x is below input range and returns last value of table if x is above range."
	std::vector<float> table = {4.0f, -1.0f, 7.5f, 2.0f, -3.0f};
	arm_linear_interp_instance_f32 S;
	S.nValues = 5;
	S.x1 = 2.0f;
	S.xSpacing = 0.25f;   // positions 2.0 .. 3.0
	S.pYData = table.data();
	for (float x : {1.999f, 1.5f, 0.0f, -1.0f, -1.0e6f}) {
		CAPTURE(x);
		CHECK(arm_linear_interp_f32(&S, x) == 4.0f);
	}
	for (float x : {3.0f, 3.001f, 3.5f, 100.0f, 1.0e6f}) {
		CAPTURE(x);
		CHECK(arm_linear_interp_f32(&S, x) == -3.0f);
	}
}

TEST_CASE("[C4 arm_linear_interp_f32] the first position returns the first value, the last position the last value, and the second position the second value") {
	// DEF: y(x1) = y[0], y(x1 + (n-1) h) = y[n-1]; the first and the last interval are the end points of the table.
	// 0.25 spacing and positions 2 .. 3 are exact in binary32, the products are exact.
	std::vector<float> table = {4.0f, -1.0f, 7.5f, 2.0f, -3.0f};
	arm_linear_interp_instance_f32 S;
	S.nValues = 5;
	S.x1 = 2.0f;
	S.xSpacing = 0.25f;
	S.pYData = table.data();
	CHECK(arm_linear_interp_f32(&S, 2.0f) == 4.0f);
	CHECK(arm_linear_interp_f32(&S, 2.25f) == -1.0f);
	CHECK(arm_linear_interp_f32(&S, 3.0f) == -3.0f);
	// last interval (positions 2.75 .. 3.0): halfway is (2 + -3)/2
	CHECK(arm_linear_interp_f32(&S, 2.875f) == -0.5f);
	// first interval (positions 2.0 .. 2.25): halfway is (4 + -1)/2
	CHECK(arm_linear_interp_f32(&S, 2.125f) == 1.5f);
}

TEST_CASE("[C4 arm_linear_interp_f32] a table with one value returns that value for every input") {
	// DOC: below the range the first value, above the range the last value; with one value both are the same value.
	std::vector<float> table = {5.5f};
	arm_linear_interp_instance_f32 S;
	S.nValues = 1;
	S.x1 = 0.0f;
	S.xSpacing = 1.0f;
	S.pYData = table.data();
	for (float x : {-3.0f, 0.0f, 0.5f, 7.0f}) {
		CAPTURE(x);
		CHECK(arm_linear_interp_f32(&S, x) == 5.5f);
	}
}

TEST_CASE("[C4 arm_linear_interp_f32] inputs of very large magnitude return the first and the last table value") {
	// C6 / NEEDS REVIEW (host-vs-target), B-DIF-3: arm_linear_interp_f32.c:94 converts '(int32_t)((x - x1) / xSpacing)' before the
	// range check. A float outside int32 range is undefined in C: x86 gives INT_MIN, ARM saturates; the intended result (first /
	// last value) occurs on both. The suite's sanitizers (-fsanitize=undefined has no float-cast-overflow) cannot detect it.
	std::vector<float> table = {4.0f, -1.0f, 7.5f};
	arm_linear_interp_instance_f32 S;
	S.nValues = 3;
	S.x1 = 0.0f;
	S.xSpacing = 1.0f;
	S.pYData = table.data();
	const float inf = std::numeric_limits<float>::infinity();
	CHECK(arm_linear_interp_f32(&S, 1.0e30f) == 7.5f);
	CHECK(arm_linear_interp_f32(&S, inf) == 7.5f);
	CHECK(arm_linear_interp_f32(&S, -1.0e30f) == 4.0f);
	CHECK(arm_linear_interp_f32(&S, -inf) == 4.0f);
}

TEST_CASE("[C4 arm_bilinear_interp_f32] a 2x2 table (the minimum) is interpolated in the single cell and returns 0 outside it") {
	// DOC: "if (x,y) are outside of the table boundary, Bilinear interpolation returns zero output". The cell of a 2x2 table
	// is X in [0,1), Y in [0,1) (xIndex <= numCols - 2 = 0). Table f(x,y) = data[x + 2y] = {1, 3, -2, 6}; DEF at
	// (0.5, 0.5) = (1+3-2+6)/4 = 2; at (0.25, 0.75): 1*0.75*0.25 + 3*0.25*0.25 + (-2)*0.75*0.75 + 6*0.25*0.75 =
	// 0.1875 + 0.1875 - 1.125 + 1.125 = 0.375. All values are dyadic: exact.
	std::vector<float> table = {1.0f, 3.0f, -2.0f, 6.0f};
	arm_bilinear_interp_instance_f32 S;
	S.numRows = 2;
	S.numCols = 2;
	S.pData = table.data();
	CHECK(arm_bilinear_interp_f32(&S, 0.0f, 0.0f) == 1.0f);
	CHECK(arm_bilinear_interp_f32(&S, 0.5f, 0.5f) == 2.0f);
	CHECK(arm_bilinear_interp_f32(&S, 0.25f, 0.75f) == 0.375f);
	CHECK(arm_bilinear_interp_f32(&S, 2.0f, 0.5f) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, 0.5f, 2.0f) == 0.0f);
}

TEST_CASE("[C4 arm_bilinear_interp_f32] points clearly outside the table (X or Y beyond the last cell, or at most -1) return 0") {
	// DOC: "Bilinear interpolation returns zero output" outside the table. Table 4 columns x 3 rows: the cells are X in
	// [0, 3), Y in [0, 2). The values -1.5, -7 (index < 0 after truncation) and 3.5, 10, 2.5 are outside by any reading of the
	// boundary.
	std::vector<float> table(12, 1.0f);
	arm_bilinear_interp_instance_f32 S;
	S.numRows = 3;
	S.numCols = 4;
	S.pData = table.data();
	CHECK(arm_bilinear_interp_f32(&S, -1.5f, 1.0f) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, -7.0f, 1.0f) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, 3.5f, 1.0f) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, 10.0f, 1.0f) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, 1.0f, -1.5f) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, 1.0f, 2.5f) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, 1.0f, 10.0f) == 0.0f);
	// and just inside: the last cell is interpolated (all values 1)
	CHECK(arm_bilinear_interp_f32(&S, 2.5f, 1.5f) == 1.0f);
}

TEST_CASE("[C4 arm_bilinear_interp_f32] a point with X between -1 and 0 is outside the table and returns 0"
          * KNOWN_ISSUE("B-DIF-2: arm_bilinear_interp_f32.c:116 'xIndex = (int32_t) X' truncates towards zero, so X in (-1, 0) gives "
                        "xIndex 0, passes the check 'xIndex < 0' and the point is extrapolated from the first cell; DOC: "
                        "'if (x,y) are outside of the table boundary, Bilinear interpolation returns zero output'")) {
	std::vector<float> table(12, 1.0f);   // every table value 1: an extrapolated value is 1 (or close), not 0
	arm_bilinear_interp_instance_f32 S;
	S.numRows = 3;
	S.numCols = 4;
	S.pData = table.data();
	CHECK(arm_bilinear_interp_f32(&S, -0.5f, 1.0f) == 0.0f);
}

TEST_CASE("[C4 arm_bilinear_interp_f32] a point with Y between -1 and 0 is outside the table and returns 0"
          * KNOWN_ISSUE("B-DIF-2: same defect for Y: arm_bilinear_interp_f32.c:117 'yIndex = (int32_t) Y' truncates towards zero, "
                        "Y in (-1, 0) gives yIndex 0 and the point is extrapolated instead of returning 0")) {
	std::vector<float> table(12, 1.0f);
	arm_bilinear_interp_instance_f32 S;
	S.numRows = 3;
	S.numCols = 4;
	S.pData = table.data();
	CHECK(arm_bilinear_interp_f32(&S, 1.0f, -0.5f) == 0.0f);
}

TEST_CASE("[C4 arm_bilinear_interp_f32] a 2x2 table {1,2,3,5} at X = -0.5, Y = 0 is outside the table and returns 0"
          * KNOWN_ISSUE("B-DIF-2: twin of B-DIH-1. arm_bilinear_interp_f32.c:116 '(int32_t) X' truncates towards zero, X = -0.5 gives "
                        "xIndex 0 and the point is extrapolated (0.5) instead of returning 0; DOC: 'if (x,y) are outside of the "
                        "table boundary, Bilinear interpolation returns zero output'")) {
	std::vector<float> table = {1.0f, 2.0f, 3.0f, 5.0f};
	arm_bilinear_interp_instance_f32 S;
	S.numRows = 2;
	S.numCols = 2;
	S.pData = table.data();
	CHECK(arm_bilinear_interp_f32(&S, -0.5f, 0.0f) == 0.0f);
}

TEST_CASE("[C4 arm_bilinear_interp_f32] characterization: a coordinate on the last column or last row returns zero (NEEDS REVIEW)") {
	// DOC is ambiguous: the last column / row is on the table, but the code requires xIndex <= numCols-2 and yIndex <= numRows-2,
	// so X = 1 or Y = 1 of a 2x2 table gives 0 although the interpolated value would not be 0 (table {1,2,3,5}: f(1,0) = 2,
	// f(1,0.5) = 3.5, f(1,1) = 5, f(0,1) = 3, f(0.5,1) = 4). NEEDS REVIEW (listed in the report).
	std::vector<float> table = {1.0f, 2.0f, 3.0f, 5.0f};
	arm_bilinear_interp_instance_f32 S;
	S.numRows = 2;
	S.numCols = 2;
	S.pData = table.data();
	CHECK(arm_bilinear_interp_f32(&S, 1.0f, 0.0f) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, 1.0f, 0.5f) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, 1.0f, 1.0f) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, 0.0f, 1.0f) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, 0.5f, 1.0f) == 0.0f);
}

TEST_CASE("[C4 arm_bilinear_interp_f32] coordinates of very large magnitude return 0") {
	// C6 / NEEDS REVIEW (host-vs-target), B-DIF-4: arm_bilinear_interp_f32.c:116-117 convert '(int32_t) X' and '(int32_t) Y' before the
	// range check; out of int32 range this is undefined in C (x86: INT_MIN, ARM: saturation), the intended result 0 occurs on both.
	// Not detectable by the suite's sanitizers (no float-cast-overflow in -fsanitize=undefined).
	std::vector<float> table(12, 1.0f);
	arm_bilinear_interp_instance_f32 S;
	S.numRows = 3;
	S.numCols = 4;
	S.pData = table.data();
	const float inf = std::numeric_limits<float>::infinity();
	CHECK(arm_bilinear_interp_f32(&S, 1.0e30f, 1.0f) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, -1.0e30f, 1.0f) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, 1.0f, 1.0e30f) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, 1.0f, -inf) == 0.0f);
	CHECK(arm_bilinear_interp_f32(&S, inf, 1.0f) == 0.0f);
}

TEST_CASE("[C4 arm_spline_f32] three knots (the minimum) give the independent double solution for both boundary types") {
	// ARM's ramp test (C1) has the parabolic runout with n = 3; here a natural and a parabolic spline of a non linear set of
	// three points, so that c is not 0. DOC: "size must be 3*(n-1)" for n = 3 coefficients 6, temp 5.
	const std::vector<float> xs = {-1.0f, 0.5f, 2.0f}, ys = {1.0f, -2.0f, 0.5f};
	for (arm_spline_type type : {ARM_SPLINE_NATURAL, ARM_SPLINE_PARABOLIC_RUNOUT}) {
		CAPTURE(static_cast<int>(type));
		const SplineDef def(xs, ys, type);
		LibSpline sp(type, xs, ys);
		const std::vector<float> q = sweep(-2.0, 3.0, 0.0625);
		compareSpline(def, q, sp.eval(q));
		CHECK(sp.outGuardOk);
		CHECK(sp.coeffs.guardIntact());
		CHECK(sp.temp.guardIntact());
	}
}

TEST_CASE("[C4 arm_spline_f32] evaluating at the knots reproduces the knot values and the first knot exactly") {
	// DEF: S(x(i)) = y(i). The first knot is evaluated with t = 0 on the first interval: y + 0 + 0 + 0 = y exactly. A later
	// knot is evaluated at t = h on the interval that ends there (the loop condition is 'xq <= x[i+1]'); the float sum of the
	// four terms deviates by the rounding bound of splineBound() (the bound takes the size of the terms into account).
	for (arm_spline_type type : {ARM_SPLINE_NATURAL, ARM_SPLINE_PARABOLIC_RUNOUT}) {
		CAPTURE(static_cast<int>(type));
		const SplineDef def(kKnotsX, kKnotsY, type);
		LibSpline sp(type, kKnotsX, kKnotsY);
		const std::vector<float> out = sp.eval(kKnotsX);
		REQUIRE(out.size() == kKnotsY.size());
		CHECK(out[0] == kKnotsY[0]);
		for (size_t i = 1; i < out.size(); i++) {
			CAPTURE(i);
			CHECK(std::fabs(static_cast<double>(out[i]) - kKnotsY[i]) <= splineBound(def, kKnotsX[i]));
		}
	}
}

TEST_CASE("[C4 arm_spline_f32] queries far below the first and far above the last knot are extended with the first and the last polynomial") {
	// DOC (arm_spline_interp_f32.c, "Behaviour outside the given intervals"): "The coefficients used to compute the y values for
	// xq<x(1) are going to be the ones used for the first interval, while for xq>x(n) the coefficients used for the last
	// interval." DEF: SplineDef::eval() takes S_0 below x(0) and S_(n-2) above x(n-1). The cubic grows with t^3, hence the
	// bound is relative to the size of the terms.
	const SplineDef def(kKnotsX, kKnotsY, ARM_SPLINE_NATURAL);
	LibSpline sp(ARM_SPLINE_NATURAL, kKnotsX, kKnotsY);
	const std::vector<float> below = {-50.0f, -20.0f, -8.0f, -3.5f};
	const std::vector<float> above = {4.5f, 6.0f, 15.0f, 40.0f};
	compareSpline(def, below, sp.eval(below));
	compareSpline(def, above, sp.eval(above));
	// the extension is the polynomial of the end interval, not a constant: values differ from the end knots
	CHECK(std::fabs(sp.eval({40.0f})[0] - kKnotsY.back()) > 1.0f);
}

TEST_CASE("[C4 arm_spline_f32] a block that starts below, crosses all intervals and ends above the knots is evaluated as one call") {
	// the three loops of arm_spline_f32 (per interval, then the remaining samples) in one block: -4 (below), one query in each
	// interval, the knots themselves, and 4.5 / 7 (above)
	const std::vector<float> q = {-4.0f, -3.0f, -2.0f, -1.5f, -1.0f, 0.0f, 0.5f, 1.0f, 2.0f, 2.5f, 3.0f, 4.0f, 4.5f, 7.0f};
	for (arm_spline_type type : {ARM_SPLINE_NATURAL, ARM_SPLINE_PARABOLIC_RUNOUT}) {
		CAPTURE(static_cast<int>(type));
		const SplineDef def(kKnotsX, kKnotsY, type);
		LibSpline sp(type, kKnotsX, kKnotsY);
		compareSpline(def, q, sp.eval(q));
		CHECK(sp.outGuardOk);
	}
}

}  // TEST_SUITE
