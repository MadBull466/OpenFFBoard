/*
 * CMSIS-DSP (vendor code, tier C): only the functions the firmware calls (Filters.cpp, HidFFB.cpp,
 * EffectsCalculator.cpp, Axis.cpp): arm_sin_f32, arm_cos_f32, arm_sqrt_f32, arm_biquad_cascade_df1_init_f32 /
 * arm_biquad_cascade_df1_f32, arm_spline_init_f32 / arm_spline_f32, arm_pid_init_f32 / arm_pid_reset_f32 /
 * arm_pid_f32, arm_var_f32 / arm_std_f32. Vendor defects are reported (V-numbers), never fixed.
 *
 * Every function is checked against an independent reference computed here in double precision:
 *  - sin/cos: std::sin / std::cos of the same float angle.
 *  - biquad: a hand-written direct form I filter, y[n] = b0 x[n] + b1 x[n-1] + b2 x[n-2] + a1 y[n-1] + a2 y[n-2]
 *    (the CMSIS sign convention: a1, a2 are the negated denominator coefficients, arm_biquad_cascade_df1_f32.c doc).
 *  - spline: the textbook cubic spline (the linear system of the CMSIS documentation, arm_spline_interp_f32.c:63-90,
 *    solved by dense Gaussian elimination, not the Crout recursion of the library) and closed-form cases (lines,
 *    parabolas).
 *  - PID: the positional form u[n] = Kp e[n] + Ki sum(e) + Kd (e[n] - e[n-1]), which equals the incremental form of
 *    arm_pid_f32 for a zero initial state.
 *  - variance / standard deviation: two-pass sample statistics with n - 1 in the denominator.
 *
 * Tolerances:
 *  - sin/cos: the table has 512 intervals over one period (FAST_MATH_TABLE_SIZE, linear interpolation), so the
 *    interpolation error is at most h^2/8 * max|sin''| = (2 pi / 512)^2 / 8 = 1.88e-5. Float rounding of the
 *    table values and the interpolation adds a few 1e-8. The scaling of the angle (x * 0.159154943092f in float)
 *    adds a phase error of about |x| * 2^-24 (rounding of the product) + |x| * 4e-8 (the float constant): 1.2e-7
 *    per radian with margin. Tolerance: 2.0e-5 + 1.2e-7 * |x|.
 *  - biquad, spline, PID, variance: float (24 bit, ulp 6e-8 relative) against double; TOL = 2e-6 (absolute for
 *    the biquad and PID outputs of magnitude <= 5, relative with scale 1 for spline / statistics). Measured worst on
 *    the host: biquad 6.5e-7 (the poles at radius 0.87 accumulate a few ulps), spline 5.9e-7, PID 3.6e-7, so about
 *    3x margin for a different rounding on the target (VFMA, below). Exact comparisons only where every operation is
 *    exact (powers of 2, sqrt).
 *
 * Host vs target: the host compiles the portable C code of CMSIS-DSP (__GNUC_PYTHON__ selects sqrtf in
 * arm_sqrt_f32; the target uses VSQRT.F32, both are correctly rounded IEEE square roots). The target (Makefile:
 * -O2 -std=gnu11, -mfpu=fpv4-sp-d16) may contract a*b+c into VFMA (GNU C default -ffp-contract=fast); x86-64
 * without -mfma does not fuse. Results can differ in the last bits: all comparisons use tolerances except sqrt
 * and the exact closed-form zeros. Out-of-range float -> int conversions (V32) differ between x86 (0x80000000) and
 * ARM (saturation).
 *
 * arm_spline_f32 reads one element past the query block when the last query point is not beyond the last knot
 * (suspected bug 2, arm_spline_interp_f32.c:223, existing cases in test_effects_calculator*.cpp). The query buffers
 * here have one extra element after the block (QueryBlock) so that this over-read stays inside the test's array and
 * the sanitizer builds can run the larger blocks; the extra element never changes a result (the loop condition
 * `blkCnt > 0` is false whenever it is read).
 *
 * Only compiled with CMSIS-DSP (USE_DSP_FUNCTIONS, FFB_TESTS_USE_DSP ON/AUTO); in DSP-off builds this file is
 * empty. Pure functions without global state: no fixture needed (like test_crc.cpp).
 */
#include "doctest.h"
#include "TestHelpers.h"

#ifdef USE_DSP_FUNCTIONS

#include "arm_math.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace {

constexpr double TWO_PI = 6.283185307179586476925;

/** Tolerance of the float recursions against the double references (see the file header) */
constexpr double TOL = 2e-6;

/** Tolerance of arm_sin_f32 / arm_cos_f32 against the exact value (see the file header) */
double trigTolerance(float x) {
	return 2.0e-5 + 1.2e-7 * std::fabs(static_cast<double>(x));
}

/** Largest deviation of f from ref over the angles first, first + step, ..., relative to trigTolerance */
template <typename F, typename R>
double worstTrigRatio(F f, R ref, double first, double last, int steps, float* worstX) {
	double worst = 0;
	for (int i = 0; i <= steps; i++) {
		const float x = static_cast<float>(first + (last - first) * i / steps);
		const double err = std::fabs(static_cast<double>(f(x)) - ref(static_cast<double>(x)));
		const double ratio = err / trigTolerance(x);
		// "!(ratio <= worst)" also takes a NaN result (CDX-014): "ratio > worst" would drop it and the check would pass
		if (!(ratio <= worst)) {
			worst = ratio;
			*worstX = x;
		}
	}
	return worst;
}

/** Deterministic pseudo random values in [-1, 1] (LCG, Numerical Recipes constants) */
std::vector<float> testSignal(size_t n, uint32_t seed = 12345) {
	std::vector<float> v(n);
	uint32_t s = seed;
	for (auto& x : v) {
		s = s * 1664525U + 1013904223U;
		x = static_cast<float>((s >> 8) * (2.0 / 16777216.0) - 1.0);
	}
	return v;
}

// ------------------------------------------------------------------------------------------------ biquad reference

/** One stage in the CMSIS order {b0, b1, b2, a1, a2}, a1/a2 negated denominator (y += a1 y[n-1] + a2 y[n-2]) */
using Stage = std::array<float, 5>;

/** A stable 2nd order low pass (RBJ cookbook, Fc = 0.05 fs, Q = 0.707) and a peaking stage in CMSIS form */
const Stage LOWPASS = {0.020083366f, 0.040166732f, 0.020083366f, 1.5610181f, -0.6413515f};
const Stage PEAK = {1.0461f, -1.6522f, 0.7064f, 1.6522f, -0.7525f};

/** Direct form I cascade in double with the float coefficients (state: x1, x2, y1, y2 per stage) */
struct ReferenceCascade {
	std::vector<Stage> stages;
	std::vector<std::array<double, 4>> state;
	explicit ReferenceCascade(std::vector<Stage> s) : stages(std::move(s)), state(stages.size(), {0, 0, 0, 0}) {}
	double step(double x) {
		for (size_t k = 0; k < stages.size(); k++) {
			const Stage& c = stages[k];
			auto& st = state[k];
			const double y = c[0] * x + c[1] * st[0] + c[2] * st[1] + c[3] * st[2] + c[4] * st[3];
			st = {x, st[0], y, st[2]};
			x = y;
		}
		return x;
	}
};

/** A CMSIS DF1 cascade with its own coefficient and state memory (state followed by a guard word) */
struct DspCascade {
	std::vector<float> coeffs;
	std::vector<float> state;
	arm_biquad_casd_df1_inst_f32 inst{};
	explicit DspCascade(const std::vector<Stage>& stages) {
		for (const Stage& s : stages) {
			coeffs.insert(coeffs.end(), s.begin(), s.end());
		}
		state.assign(4 * stages.size() + 1, 99.0f); // dirty, the last word is a guard
		arm_biquad_cascade_df1_init_f32(&inst, static_cast<uint8_t>(stages.size()), coeffs.data(), state.data());
	}
	std::vector<float> run(const float* in, size_t n) {
		std::vector<float> out(n);
		arm_biquad_cascade_df1_f32(&inst, in, out.data(), static_cast<uint32_t>(n));
		return out;
	}
};

double maxAbsDiff(const std::vector<float>& a, const std::vector<double>& b) {
	double worst = 0;
	for (size_t i = 0; i < a.size(); i++) {
		const double diff = std::fabs(static_cast<double>(a[i]) - b[i]);
		if (!(diff <= worst)) worst = diff; // keeps a NaN result (std::max would drop it, CDX-014)
	}
	return worst;
}

// ------------------------------------------------------------------------------------------------ spline reference

/** Cubic spline polynomials S_i(x) = y_i + b_i t + c_i t^2 + d_i t^3, t = x - x_i, in double */
struct ReferenceSpline {
	std::vector<double> x, y, b, c, d; // c has n entries (c_n for the end condition), b and d n - 1

	/**
	 * Solves the system of the CMSIS documentation (arm_spline_interp_f32.c:63-90) with dense Gaussian elimination:
	 * row 0: c0 = 0 (natural) or c0 - c1 = 0 (parabolic runout); rows 1..n-2: h(i-1) c(i-1) + 2 (h(i-1) + h(i)) c(i)
	 * + h(i) c(i+1) = 3/h(i) (y(i+1) - y(i)) - 3/h(i-1) (y(i) - y(i-1)); row n-1: c(n-1) = 0 or -c(n-2) + c(n-1) = 0.
	 */
	ReferenceSpline(const std::vector<float>& xs, const std::vector<float>& ys, arm_spline_type type)
		: x(xs.begin(), xs.end()), y(ys.begin(), ys.end()) {
		const size_t n = x.size();
		std::vector<std::vector<double>> a(n, std::vector<double>(n + 1, 0.0));
		a[0][0] = 1;
		a[n - 1][n - 1] = 1;
		if (type == ARM_SPLINE_PARABOLIC_RUNOUT) {
			a[0][1] = -1;
			a[n - 1][n - 2] = -1;
		}
		for (size_t i = 1; i + 1 < n; i++) {
			const double h0 = x[i] - x[i - 1], h1 = x[i + 1] - x[i];
			a[i][i - 1] = h0;
			a[i][i] = 2 * (h0 + h1);
			a[i][i + 1] = h1;
			a[i][n] = 3 / h1 * (y[i + 1] - y[i]) - 3 / h0 * (y[i] - y[i - 1]);
		}
		for (size_t col = 0; col < n; col++) { // elimination with partial pivoting
			size_t piv = col;
			for (size_t r = col + 1; r < n; r++) {
				if (std::fabs(a[r][col]) > std::fabs(a[piv][col])) piv = r;
			}
			std::swap(a[col], a[piv]);
			for (size_t r = 0; r < n; r++) {
				if (r == col) continue;
				const double f = a[r][col] / a[col][col];
				for (size_t k = col; k <= n; k++) a[r][k] -= f * a[col][k];
			}
		}
		c.resize(n);
		for (size_t i = 0; i < n; i++) c[i] = a[i][n] / a[i][i];
		for (size_t i = 0; i + 1 < n; i++) {
			const double h = x[i + 1] - x[i];
			b.push_back((y[i + 1] - y[i]) / h - h * (c[i + 1] + 2 * c[i]) / 3);
			d.push_back((c[i + 1] - c[i]) / (3 * h));
		}
	}
	/** Polynomial of interval i evaluated at q */
	double poly(size_t i, double q) const {
		const double t = q - x[i];
		return y[i] + b[i] * t + c[i] * t * t + d[i] * t * t * t;
	}
	/** S(q): first interval below the range, last interval above it (CMSIS documentation) */
	double operator()(double q) const {
		size_t i = 0;
		while (i + 2 < x.size() && q > x[i + 1]) i++;
		return poly(i, q);
	}
};

/** A CMSIS spline with its buffers (coeffs 3 (n-1), tempBuffer 2n - 1, arm_spline_interp_init_f32.c:42-45) */
struct DspSpline {
	std::vector<float> x, y, coeffs, temp;
	arm_spline_instance_f32 inst{};
	DspSpline(std::vector<float> xs, std::vector<float> ys, arm_spline_type type)
		: x(std::move(xs)), y(std::move(ys)), coeffs(3 * (x.size() - 1)), temp(2 * x.size() - 1) {
		arm_spline_init_f32(&inst, type, x.data(), y.data(), static_cast<uint32_t>(x.size()), coeffs.data(),
							temp.data());
	}
	/** Interpolates a block of query points (with one spare element after the block, see the file header) */
	std::vector<float> run(const std::vector<float>& q) {
		std::vector<float> queries(q);
		queries.push_back(std::numeric_limits<float>::infinity());
		std::vector<float> out(q.size());
		arm_spline_f32(&inst, queries.data(), out.data(), static_cast<uint32_t>(q.size()));
		return out;
	}
};

/** Query points first, first + step, ... (count points) */
std::vector<float> queryRange(double first, double last, int count) {
	std::vector<float> q;
	for (int i = 0; i < count; i++) {
		q.push_back(static_cast<float>(first + (last - first) * i / (count - 1)));
	}
	return q;
}

// Non-uniform knots used by several spline cases
const std::vector<float> KNOTS_X = {0.0f, 0.5f, 1.5f, 2.0f, 3.5f, 4.0f, 6.0f};
const std::vector<float> KNOTS_Y = {1.0f, 2.0f, -1.0f, 0.5f, 3.0f, 2.5f, -2.0f};

// ------------------------------------------------------------------------------------------------ statistics

/** Two-pass sample variance in double (n - 1 in the denominator) */
double referenceVariance(const std::vector<float>& v) {
	double mean = 0;
	for (float f : v) mean += f;
	mean /= static_cast<double>(v.size());
	double sum = 0;
	for (float f : v) sum += (f - mean) * (f - mean);
	return sum / static_cast<double>(v.size() - 1);
}

} // namespace

TEST_SUITE("CmsisDsp") {

// ============================================================================================ arm_sin_f32 / arm_cos_f32

TEST_CASE("arm_sin_f32 matches the sine within the table interpolation error for angles from 0 to 4 pi") {
	float worstX = 0;
	const double worst = worstTrigRatio(arm_sin_f32, [](double x) { return std::sin(x); }, 0.0, 2 * TWO_PI, 20000,
										&worstX);
	INFO("worst error / tolerance " << worst << " at x = " << worstX);
	CHECK(worst <= 1.0);
}

TEST_CASE("arm_cos_f32 matches the cosine within the table interpolation error for angles from 0 to 4 pi") {
	float worstX = 0;
	const double worst = worstTrigRatio(arm_cos_f32, [](double x) { return std::cos(x); }, 0.0, 2 * TWO_PI, 20000,
										&worstX);
	INFO("worst error / tolerance " << worst << " at x = " << worstX);
	CHECK(worst <= 1.0);
}

TEST_CASE("arm_sin_f32 and arm_cos_f32 match the exact values for negative angles down to -4 pi") {
	float worstSin = 0, worstCos = 0;
	CHECK(worstTrigRatio(arm_sin_f32, [](double x) { return std::sin(x); }, -2 * TWO_PI, 0.0, 20000, &worstSin) <=
		  1.0);
	CHECK(worstTrigRatio(arm_cos_f32, [](double x) { return std::cos(x); }, -2 * TWO_PI, 0.0, 20000, &worstCos) <=
		  1.0);
}

TEST_CASE("arm_sin_f32 is odd and arm_cos_f32 is even within the table interpolation error") {
	for (float x : {0.1f, 0.7853982f, 1.0f, 2.5f, 3.1f, 4.0f, 6.0f, 10.0f}) {
		CAPTURE(x);
		CHECK(std::fabs(arm_sin_f32(-x) + arm_sin_f32(x)) <= 2 * trigTolerance(x));
		CHECK(std::fabs(arm_cos_f32(-x) - arm_cos_f32(x)) <= 2 * trigTolerance(x));
	}
}

TEST_CASE("arm_sin_f32 and arm_cos_f32 give the same value for angles one period apart") {
	for (float x : {-3.0f, -0.4f, 0.3f, 1.2f, 2.9f, 5.5f}) {
		CAPTURE(x);
		const float y = x + static_cast<float>(TWO_PI);
		CHECK(std::fabs(arm_sin_f32(y) - arm_sin_f32(x)) <= trigTolerance(x) + trigTolerance(y));
		CHECK(std::fabs(arm_cos_f32(y) - arm_cos_f32(x)) <= trigTolerance(x) + trigTolerance(y));
	}
}

TEST_CASE("arm_sin_f32 and arm_cos_f32 of 0 and of multiples of pi/2 hit the table values") {
	CHECK(arm_sin_f32(0.0f) == 0.0f);
	CHECK(arm_cos_f32(0.0f) == doctest::Approx(1.0).epsilon(1e-7));
	CHECK(arm_sin_f32(static_cast<float>(TWO_PI / 4)) == doctest::Approx(1.0).epsilon(1e-6));
	CHECK(std::fabs(arm_cos_f32(static_cast<float>(TWO_PI / 2)) + 1.0f) <= 1e-6f);
	CHECK(std::fabs(arm_sin_f32(static_cast<float>(-TWO_PI / 4)) + 1.0f) <= 1e-6f);
}

TEST_CASE("arm_sin_f32 and arm_cos_f32 of large angles lose accuracy only by the rounding of the scaled angle") {
	// tolerance 2e-5 + 1.2e-7 |x|: 1.4e-4 at 1000 rad, 1.2e-3 at 10000 rad
	for (double first : {1000.0, 10000.0, -10000.0}) {
		CAPTURE(first);
		float worstX = 0;
		CHECK(worstTrigRatio(arm_sin_f32, [](double x) { return std::sin(x); }, first, first + 7, 2000, &worstX) <=
			  1.0);
		CHECK(worstTrigRatio(arm_cos_f32, [](double x) { return std::cos(x); }, first, first + 7, 2000, &worstX) <=
			  1.0);
	}
}

// The two wrap cases check the value. "Instead of reading past its end" is only verified in sanitizer builds: without
// the rotation the index is 512 with fract 0 and the code reads sinTable_f32[513], one past the 513-entry table, with
// weight 0, so the value is still right and only ASan reports the read (review p9r3 mutant s5).
TEST_CASE("arm_sin_f32 of an angle just below 0 uses the start of the table instead of reading past its end") {
	// x * (1/2pi) = -1.6e-10; mapped to [0, 1] it rounds to exactly 1.0, the index 512 is rotated to 0
	// (arm_sin_f32.c "when in is exactly 1")
	const float x = -1e-9f;
	CHECK(std::fabs(arm_sin_f32(x) - std::sin(static_cast<double>(x))) <= trigTolerance(x));
}

TEST_CASE("arm_cos_f32 of an angle just below -pi/2 uses the start of the table instead of reading past its end") {
	// x * (1/2pi) + 0.25 = -2^-25 (found by search on the host); mapped to [0, 1] it rounds to exactly 1.0
	const float x = -0x1.921fb8p+0f;
	CHECK(std::fabs(arm_cos_f32(x) - std::cos(static_cast<double>(x))) <= trigTolerance(x));
}

#define V32_TEXT                                                                                                       \
	"bug V32: arm_sin_f32.c / arm_cos_f32.c `n = (int32_t) in` with in = x / 2pi: for |x| > 2^31 * 2pi (1.35e10 "     \
	"rad), infinity or NaN the float -> int32 conversion is undefined (x86: 0x80000000, ARM VCVT saturates) and for " \
	"negative x `n--` then overflows INT32_MIN; `(uint16_t)findex` is out of range too. The host returns about "     \
	"3.3e10 for sin(2e10) instead of a value in [-1, 1] (ARM: also far outside, via the saturated index). Latent: "  \
	"the firmware's angles are bounded (Filters.cpp:102 pi*Fc, HidFFB.cpp:333-339 directions, Axis.cpp:717 ramp). "   \
	"Chosen intended result: a value within [-1, 1] (like C sinf/cosf, which are finite for every finite x); a fix "    \
	"returning NaN for these inputs keeps this case failing"

TEST_CASE("arm_sin_f32 and arm_cos_f32 of an angle beyond the int32 range stay within -1 and 1" *
		  KNOWN_ISSUE_UB(V32_TEXT)) {
	for (float x : {2e10f, -1e11f}) {
		CAPTURE(x);
		CHECK(std::fabs(arm_sin_f32(x)) <= 1.0f);
		CHECK(std::fabs(arm_cos_f32(x)) <= 1.0f);
	}
}

// ============================================================================================ arm_sqrt_f32

TEST_CASE("arm_sqrt_f32 of a positive number is the correctly rounded square root and succeeds") {
	for (float in : {2.0f, 4.0f, 0.25f, 1e-30f, 3e30f, 1e-40f /* subnormal */, 12345.678f}) {
		CAPTURE(in);
		float out = -1.0f;
		CHECK(arm_sqrt_f32(in, &out) == ARM_MATH_SUCCESS);
		CHECK(out == static_cast<float>(std::sqrt(static_cast<double>(in))));
	}
}

TEST_CASE("arm_sqrt_f32 of zero is zero and succeeds") {
	float out = -1.0f;
	CHECK(arm_sqrt_f32(0.0f, &out) == ARM_MATH_SUCCESS);
	CHECK(out == 0.0f);
	out = -1.0f;
	CHECK(arm_sqrt_f32(-0.0f, &out) == ARM_MATH_SUCCESS); // -0 >= 0: IEEE sqrt(-0) = -0
	CHECK(out == 0.0f);
	CHECK(std::signbit(out));
}

TEST_CASE("arm_sqrt_f32 of a negative number reports an argument error and outputs 0") {
	for (float in : {-1.0f, -1e-30f, -std::numeric_limits<float>::infinity()}) {
		CAPTURE(in);
		float out = -1.0f;
		CHECK(arm_sqrt_f32(in, &out) == ARM_MATH_ARGUMENT_ERROR);
		CHECK(out == 0.0f);
	}
}

TEST_CASE("arm_sqrt_f32 of NaN reports an argument error and outputs 0") {
	// NaN >= 0 is false (fast_math_functions.h:240): the negative branch
	float out = -1.0f;
	CHECK(arm_sqrt_f32(std::numeric_limits<float>::quiet_NaN(), &out) == ARM_MATH_ARGUMENT_ERROR);
	CHECK(out == 0.0f);
}

TEST_CASE("arm_sqrt_f32 of infinity is infinity") {
	float out = 0.0f;
	CHECK(arm_sqrt_f32(std::numeric_limits<float>::infinity(), &out) == ARM_MATH_SUCCESS);
	CHECK(std::isinf(out));
}

// ============================================================================================ biquad DF1

TEST_CASE("arm_biquad_cascade_df1_init_f32 stores the stages and coefficients and zeroes 4 state words per stage") {
	std::array<float, 15> coeffs{};
	std::array<float, 13> state;
	state.fill(7.0f);
	arm_biquad_casd_df1_inst_f32 inst{};
	arm_biquad_cascade_df1_init_f32(&inst, 3, coeffs.data(), state.data());
	CHECK(inst.numStages == 3);
	CHECK(inst.pCoeffs == coeffs.data());
	CHECK(inst.pState == state.data());
	for (size_t i = 0; i < 12; i++) {
		CAPTURE(i);
		CHECK(state[i] == 0.0f);
	}
	CHECK(state[12] == 7.0f); // nothing beyond the state of the last stage
}

TEST_CASE("a single biquad stage processed one sample at a time matches a direct form I reference") {
	// The firmware's use: Filters.cpp:76, one stage, blockSize 1
	DspCascade dsp({LOWPASS});
	ReferenceCascade ref({LOWPASS});
	const std::vector<float> in = testSignal(200);
	std::vector<float> out;
	std::vector<double> expected;
	for (float x : in) {
		out.push_back(dsp.run(&x, 1)[0]);
		expected.push_back(ref.step(x));
	}
	CHECK(maxAbsDiff(out, expected) <= TOL);
}

TEST_CASE("the biquad feedback coefficients are added to the output (CMSIS sign convention)") {
	// y[n] = x[n] + 0.5 y[n-1] - 0.25 y[n-2]: impulse response 1, 0.5, 0, -0.125, -0.0625 (exact in float)
	DspCascade dsp({{1.0f, 0.0f, 0.0f, 0.5f, -0.25f}});
	const std::array<float, 5> impulse = {1, 0, 0, 0, 0};
	const std::vector<float> out = dsp.run(impulse.data(), impulse.size());
	CHECK(out == std::vector<float>{1.0f, 0.5f, 0.0f, -0.125f, -0.0625f});
}

TEST_CASE("the biquad feedforward coefficients delay the input by 0, 1 and 2 samples") {
	DspCascade dsp({{0.5f, 0.25f, 0.125f, 0.0f, 0.0f}});
	const std::array<float, 5> impulse = {1, 0, 0, 0, 0};
	CHECK(dsp.run(impulse.data(), impulse.size()) == std::vector<float>{0.5f, 0.25f, 0.125f, 0.0f, 0.0f});
}

TEST_CASE("a biquad block of any size matches the direct form I reference") {
	// blockSize 4n + r: the loop unrolled by 4 (ARM_MATH_LOOPUNROLL like the target) and the remainder loop
	const std::vector<float> in = testSignal(64, 777);
	for (size_t block : {1U, 2U, 3U, 4U, 5U, 7U, 8U, 13U, 16U, 64U}) {
		CAPTURE(block);
		DspCascade dsp({PEAK});
		ReferenceCascade ref({PEAK});
		std::vector<float> out;
		for (size_t pos = 0; pos < in.size(); pos += block) {
			const size_t n = std::min(block, in.size() - pos);
			const std::vector<float> part = dsp.run(&in[pos], n);
			out.insert(out.end(), part.begin(), part.end());
		}
		std::vector<double> expected;
		for (float x : in) expected.push_back(ref.step(x));
		CHECK(maxAbsDiff(out, expected) <= TOL);
	}
}

TEST_CASE("a biquad block of 0 samples writes nothing and keeps the state") {
	DspCascade dsp({LOWPASS, PEAK});
	const std::vector<float> in = testSignal(5, 3);
	dsp.run(in.data(), in.size());
	const std::vector<float> before = dsp.state;
	float out = -1.0f;
	arm_biquad_cascade_df1_f32(&dsp.inst, in.data(), &out, 0);
	CHECK(out == -1.0f);
	CHECK(dsp.state == before);
}

TEST_CASE("a two stage biquad cascade feeds the output of the first stage into the second") {
	const std::vector<float> in = testSignal(37, 4242);
	DspCascade dsp({LOWPASS, PEAK});
	ReferenceCascade ref({LOWPASS, PEAK});
	std::vector<double> expected;
	for (float x : in) expected.push_back(ref.step(x));
	CHECK(maxAbsDiff(dsp.run(in.data(), in.size()), expected) <= TOL);
	// the same as two separate single stages in a row
	DspCascade first({LOWPASS}), second({PEAK});
	const std::vector<float> mid = first.run(in.data(), in.size());
	const std::vector<float> chained = second.run(mid.data(), mid.size());
	std::vector<double> chainedD(chained.begin(), chained.end());
	DspCascade again({LOWPASS, PEAK});
	CHECK(maxAbsDiff(again.run(in.data(), in.size()), chainedD) <= 1e-6);
}

TEST_CASE("the biquad state holds the last two inputs and outputs of every stage after a block") {
	// pState per stage {x[n-1], x[n-2], y[n-1], y[n-2]} (arm_biquad_cascade_df1_f32.c, state layout)
	const std::vector<float> in = testSignal(11, 99);
	DspCascade dsp({LOWPASS, PEAK});
	const std::vector<float> out = dsp.run(in.data(), in.size());
	DspCascade first({LOWPASS});
	const std::vector<float> mid = first.run(in.data(), in.size());
	CHECK(dsp.state[0] == in[10]);
	CHECK(dsp.state[1] == in[9]);
	CHECK(dsp.state[2] == doctest::Approx(mid[10]).epsilon(1e-6));
	CHECK(dsp.state[3] == doctest::Approx(mid[9]).epsilon(1e-6));
	CHECK(dsp.state[4] == doctest::Approx(mid[10]).epsilon(1e-6));
	CHECK(dsp.state[5] == doctest::Approx(mid[9]).epsilon(1e-6));
	CHECK(dsp.state[6] == out[10]);
	CHECK(dsp.state[7] == out[9]);
	CHECK(dsp.state[8] == 99.0f); // guard after the last stage untouched
}

#define V33_TEXT                                                                                                       \
	"bug V33 (crash, skipped): arm_biquad_cascade_df1_f32.c:534-687 (portable C version, the one the F407 uses) "     \
	"processes the stages in a do { ... stage--; } while (stage > 0U) loop: a cascade with numStages 0 runs one "      \
	"stage and then stage wraps to 0xFFFFFFFF, so it walks about 2^32 stages through memory beyond the coefficient "  \
	"and state arrays (host SIGSEGV; target BusFault or watchdog). The NEON version (:382) uses while (stage > 0) "   \
	"and does nothing: the chosen intended result follows it (no output written, nothing read or written beyond the " \
	"arrays), so a fix that copies the input to the output (identity for an empty cascade) keeps this case failing. " \
	"Reachable through bug 124 (a null Biquad reads numStages 0)"

TEST_CASE("a biquad cascade with zero stages leaves the output and the coefficient and state memory untouched" *
		  doctest::skip() * doctest::description("KNOWN ISSUE (crash): " V33_TEXT)) {
	std::array<float, 6> coeffs = {1, 0, 0, 0, 0, 55.0f}; // last word: guard
	std::array<float, 5> state = {11.0f, 22.0f, 33.0f, 44.0f, 66.0f};
	arm_biquad_casd_df1_inst_f32 inst{};
	arm_biquad_cascade_df1_init_f32(&inst, 0, coeffs.data(), state.data()); // clears 0 words
	const std::array<float, 2> in = {1.0f, 2.0f};
	std::array<float, 2> out = {-1.0f, -1.0f};
	arm_biquad_cascade_df1_f32(&inst, in.data(), out.data(), 2);
	CHECK(out[0] == -1.0f);
	CHECK(out[1] == -1.0f);
	CHECK(state == std::array<float, 5>{11.0f, 22.0f, 33.0f, 44.0f, 66.0f});
	CHECK(coeffs[5] == 55.0f);
}

// ============================================================================================ cubic spline

TEST_CASE("a natural spline through points on a straight line is that line inside and outside the range") {
	DspSpline spline({0.0f, 1.0f, 2.5f, 3.0f, 5.0f}, {1.0f, 3.0f, 6.0f, 7.0f, 11.0f}, ARM_SPLINE_NATURAL); // y = 2x+1
	const std::vector<float> q = queryRange(-2.0, 7.0, 19); // last point beyond the last knot: no over-read
	const std::vector<float> out = spline.run(q);
	for (size_t i = 0; i < q.size(); i++) {
		CAPTURE(q[i]);
		CHECK(out[i] == doctest::Approx(2.0 * q[i] + 1.0).epsilon(TOL).scale(1.0));
	}
}

TEST_CASE("a parabolic runout spline through points on a parabola is that parabola inside and outside the range") {
	// constant curvature satisfies c(1) = c(2) and c(n-1) = c(n) exactly
	std::vector<float> x = {-2.0f, -0.5f, 0.0f, 1.0f, 3.0f}, y;
	for (float v : x) y.push_back(0.5f * v * v - v + 2.0f);
	DspSpline spline(x, y, ARM_SPLINE_PARABOLIC_RUNOUT);
	const std::vector<float> q = queryRange(-3.0, 4.0, 15);
	const std::vector<float> out = spline.run(q);
	for (size_t i = 0; i < q.size(); i++) {
		CAPTURE(q[i]);
		CHECK(out[i] == doctest::Approx(0.5 * q[i] * q[i] - q[i] + 2.0).epsilon(TOL).scale(1.0));
	}
}

TEST_CASE("a natural spline matches the textbook cubic spline at non-uniform knots for a block of 50 points") {
	DspSpline spline(KNOTS_X, KNOTS_Y, ARM_SPLINE_NATURAL);
	const ReferenceSpline ref(KNOTS_X, KNOTS_Y, ARM_SPLINE_NATURAL);
	const std::vector<float> q = queryRange(0.0, 6.0, 50);
	const std::vector<float> out = spline.run(q);
	for (size_t i = 0; i < q.size(); i++) {
		CAPTURE(q[i]);
		CHECK(out[i] == doctest::Approx(ref(q[i])).epsilon(TOL).scale(1.0));
	}
}

TEST_CASE("a parabolic runout spline matches the textbook cubic spline at non-uniform knots for a block of 50 points") {
	DspSpline spline(KNOTS_X, KNOTS_Y, ARM_SPLINE_PARABOLIC_RUNOUT);
	const ReferenceSpline ref(KNOTS_X, KNOTS_Y, ARM_SPLINE_PARABOLIC_RUNOUT);
	const std::vector<float> q = queryRange(0.0, 6.0, 50);
	const std::vector<float> out = spline.run(q);
	for (size_t i = 0; i < q.size(); i++) {
		CAPTURE(q[i]);
		CHECK(out[i] == doctest::Approx(ref(q[i])).epsilon(TOL).scale(1.0));
	}
}

TEST_CASE("natural and parabolic runout splines through the same points differ") {
	DspSpline natural(KNOTS_X, KNOTS_Y, ARM_SPLINE_NATURAL);
	DspSpline parabolic(KNOTS_X, KNOTS_Y, ARM_SPLINE_PARABOLIC_RUNOUT);
	const std::vector<float> q = {0.25f};
	CHECK(std::fabs(natural.run(q)[0] - parabolic.run(q)[0]) > 0.01f);
}

TEST_CASE("a spline passes through every knot") {
	for (arm_spline_type type : {ARM_SPLINE_NATURAL, ARM_SPLINE_PARABOLIC_RUNOUT}) {
		CAPTURE(static_cast<int>(type));
		DspSpline spline(KNOTS_X, KNOTS_Y, type);
		const std::vector<float> out = spline.run(KNOTS_X);
		for (size_t i = 0; i < KNOTS_X.size(); i++) {
			CAPTURE(i);
			CHECK(out[i] == doctest::Approx(KNOTS_Y[i]).epsilon(TOL).scale(1.0));
		}
	}
}

TEST_CASE("the coefficients of a natural spline give zero curvature at both ends") {
	// coeffs = b[0..n-2], c[0..n-2], d[0..n-2] (arm_spline_interp_init_f32.c:42-45); S'' = 2c + 6d t
	DspSpline spline(KNOTS_X, KNOTS_Y, ARM_SPLINE_NATURAL);
	const size_t m = KNOTS_X.size() - 1;
	const float* c = spline.coeffs.data() + m;
	const float* d = spline.coeffs.data() + 2 * m;
	const float hLast = KNOTS_X[m] - KNOTS_X[m - 1];
	CHECK(c[0] == 0.0f);
	CHECK(std::fabs(2 * c[m - 1] + 6 * d[m - 1] * hLast) <= 1e-4f);
}

TEST_CASE("the coefficients of a parabolic runout spline give equal curvature in the first two and last two knots") {
	DspSpline spline(KNOTS_X, KNOTS_Y, ARM_SPLINE_PARABOLIC_RUNOUT);
	const size_t m = KNOTS_X.size() - 1;
	const float* c = spline.coeffs.data() + m;
	const float* d = spline.coeffs.data() + 2 * m;
	const float hLast = KNOTS_X[m] - KNOTS_X[m - 1];
	CHECK(c[0] == doctest::Approx(c[1]).epsilon(TOL).scale(1.0));
	CHECK(c[m - 1] + 3 * d[m - 1] * hLast == doctest::Approx(c[m - 1]).epsilon(TOL).scale(1.0));
	CHECK(std::fabs(c[0]) > 0.1f); // not trivially a natural spline
}

TEST_CASE("outside the range a spline continues with the polynomial of the first or the last interval") {
	for (arm_spline_type type : {ARM_SPLINE_NATURAL, ARM_SPLINE_PARABOLIC_RUNOUT}) {
		CAPTURE(static_cast<int>(type));
		DspSpline spline(KNOTS_X, KNOTS_Y, type);
		const ReferenceSpline ref(KNOTS_X, KNOTS_Y, type);
		const std::vector<float> q = {-1.5f, -0.25f, 6.5f, 8.0f};
		const std::vector<float> out = spline.run(q);
		CHECK(out[0] == doctest::Approx(ref.poly(0, q[0])).epsilon(TOL).scale(1.0));
		CHECK(out[1] == doctest::Approx(ref.poly(0, q[1])).epsilon(TOL).scale(1.0));
		CHECK(out[2] == doctest::Approx(ref.poly(KNOTS_X.size() - 2, q[2])).epsilon(TOL).scale(1.0));
		CHECK(out[3] == doctest::Approx(ref.poly(KNOTS_X.size() - 2, q[3])).epsilon(TOL).scale(1.0));
	}
}

TEST_CASE("a single query point beyond the last knot uses the last interval") {
	DspSpline spline(KNOTS_X, KNOTS_Y, ARM_SPLINE_NATURAL);
	const ReferenceSpline ref(KNOTS_X, KNOTS_Y, ARM_SPLINE_NATURAL);
	const float q = 7.0f;
	float out = 0;
	arm_spline_f32(&spline.inst, &q, &out, 1); // the last point is beyond x(n): read of q only, no over-read
	CHECK(out == doctest::Approx(ref.poly(KNOTS_X.size() - 2, q)).epsilon(TOL).scale(1.0));
}

TEST_CASE("a natural spline through two points is the straight line") {
	DspSpline spline({1.0f, 3.0f}, {2.0f, -2.0f}, ARM_SPLINE_NATURAL);
	const std::vector<float> out = spline.run({0.0f, 1.5f, 2.0f, 4.0f});
	CHECK(out[0] == doctest::Approx(4.0).epsilon(1e-6));
	CHECK(out[1] == doctest::Approx(1.0).epsilon(1e-6));
	CHECK(out[2] == doctest::Approx(0.0).scale(1.0).epsilon(1e-6));
	CHECK(out[3] == doctest::Approx(-4.0).epsilon(1e-6));
}

// NEEDS REVIEW 161: with two points the parabolic runout conditions c(1) = c(2) and c(n-1) = c(n) are the same
// equation and the system is singular; arm_spline_interp_init_f32.c:145-146 divides 0 / (1 + u[0]) = 0 / 0 and every
// coefficient becomes NaN. The documentation (arm_spline_interp_init_f32.c:42-50) sets no minimum n. Should n = 2
// be rejected / documented, or fall back to the straight line?
TEST_CASE("characterization: a parabolic runout spline through two points gives NaN (NEEDS REVIEW 161)") {
	DspSpline spline({1.0f, 3.0f}, {2.0f, -2.0f}, ARM_SPLINE_PARABOLIC_RUNOUT);
	const std::vector<float> out = spline.run({1.5f});
	CHECK(std::isnan(out[0]));
}

// NEEDS REVIEW 162: arm_spline_f32 walks the intervals once (arm_spline_interp_f32.c:185-232): a query point that is
// smaller than an earlier one is evaluated with the interval of the earlier point (here the last interval, i.e. an
// extrapolation) instead of its own. The documentation of xq (interpolation_functions.h:127) does not require
// ascending query points. Is ascending order a documented precondition (the firmware passes one point)?
TEST_CASE("characterization: a query point after a larger one is evaluated with the larger point's interval "
		  "(NEEDS REVIEW 162)") {
	DspSpline spline(KNOTS_X, KNOTS_Y, ARM_SPLINE_NATURAL);
	const ReferenceSpline ref(KNOTS_X, KNOTS_Y, ARM_SPLINE_NATURAL);
	const std::vector<float> out = spline.run({5.0f, 1.0f});
	CHECK(out[0] == doctest::Approx(ref(5.0)).epsilon(TOL).scale(1.0));
	CHECK(out[1] == doctest::Approx(ref.poly(KNOTS_X.size() - 2, 1.0)).epsilon(TOL).scale(1.0));
	CHECK(std::fabs(out[1] - ref(1.0)) > 1.0); // not the spline value at 1.0
}

// ============================================================================================ PID

TEST_CASE("arm_pid_init_f32 derives A0 = Kp + Ki + Kd, A1 = -Kp - 2 Kd and A2 = Kd") {
	arm_pid_instance_f32 pid{};
	pid.Kp = 0.5f;
	pid.Ki = 0.25f;
	pid.Kd = 2.0f;
	arm_pid_init_f32(&pid, 1);
	CHECK(pid.A0 == 2.75f);
	CHECK(pid.A1 == -4.5f);
	CHECK(pid.A2 == 2.0f);
}

TEST_CASE("arm_pid_init_f32 clears the state only when the reset flag is set") {
	arm_pid_instance_f32 pid{};
	pid.Kp = 1.0f;
	pid.state[0] = 1.0f;
	pid.state[1] = 2.0f;
	pid.state[2] = 3.0f;
	arm_pid_init_f32(&pid, 0);
	CHECK(pid.state[0] == 1.0f);
	CHECK(pid.state[1] == 2.0f);
	CHECK(pid.state[2] == 3.0f);
	arm_pid_init_f32(&pid, 1);
	CHECK(pid.state[0] == 0.0f);
	CHECK(pid.state[1] == 0.0f);
	CHECK(pid.state[2] == 0.0f);
}

TEST_CASE("arm_pid_reset_f32 clears the state and keeps the gains and derived coefficients") {
	arm_pid_instance_f32 pid{};
	pid.Kp = 0.3f;
	pid.Ki = 0.03f;
	pid.Kd = 0.1f;
	arm_pid_init_f32(&pid, 1);
	arm_pid_f32(&pid, 5.0f);
	arm_pid_f32(&pid, -2.0f);
	const arm_pid_instance_f32 before = pid;
	arm_pid_reset_f32(&pid);
	CHECK(pid.state[0] == 0.0f);
	CHECK(pid.state[1] == 0.0f);
	CHECK(pid.state[2] == 0.0f);
	CHECK(pid.A0 == before.A0);
	CHECK(pid.A1 == before.A1);
	CHECK(pid.A2 == before.A2);
	CHECK(pid.Kp == before.Kp);
	CHECK(pid.Ki == before.Ki);
	CHECK(pid.Kd == before.Kd);
	// after the reset the controller starts again like a new one
	CHECK(arm_pid_f32(&pid, 1.0f) == doctest::Approx(0.43).epsilon(1e-6));
}

TEST_CASE("arm_pid_f32 equals the positional PID Kp e + Ki sum(e) + Kd (e - previous e)") {
	arm_pid_instance_f32 pid{};
	pid.Kp = 0.8f;
	pid.Ki = 0.05f;
	pid.Kd = 0.3f;
	arm_pid_init_f32(&pid, 1);
	const std::vector<float> e = testSignal(100, 31337);
	double integral = 0, previous = 0;
	double worst = 0;
	for (float err : e) {
		integral += err;
		const double expected = pid.Kp * double(err) + pid.Ki * integral + pid.Kd * (double(err) - previous);
		previous = err;
		const double diff = std::fabs(arm_pid_f32(&pid, err) - expected);
		if (!(diff <= worst)) worst = diff; // keeps a NaN result (std::max would drop it, CDX-014)
	}
	CHECK(worst <= TOL);
}

TEST_CASE("arm_pid_f32 does not limit its output") {
	// documented CMSIS behaviour (controller_functions.h:287-303): no saturation or anti-windup, the state keeps the
	// full output (the Axis speed limiter relies on this, NEEDS REVIEW 158 in test_axis_speed_limiter.cpp)
	arm_pid_instance_f32 pid{};
	pid.Kp = 0.5f;
	pid.Ki = 1.0f;
	arm_pid_init_f32(&pid, 1);
	float out = 0;
	for (int n = 0; n < 100000; n++) out = arm_pid_f32(&pid, 1.0f);
	CHECK(out == doctest::Approx(100000.5).epsilon(TOL));
	CHECK(pid.state[2] == out);
}

TEST_CASE("arm_pid_f32 integrates a constant error linearly") {
	// Kp e + Ki (n + 1) e: the speed limiter of Axis.cpp:945 (Kd 0)
	arm_pid_instance_f32 pid{};
	pid.Kp = 0.3f;
	pid.Ki = 0.03f;
	pid.Kd = 0.0f;
	arm_pid_init_f32(&pid, 1);
	for (int n = 0; n < 50; n++) {
		CAPTURE(n);
		CHECK(arm_pid_f32(&pid, 10.0f) == doctest::Approx(3.0 + 0.3 * (n + 1)).epsilon(TOL));
	}
}

TEST_CASE("arm_pid_f32 with only a derivative gain outputs the scaled difference of consecutive errors") {
	arm_pid_instance_f32 pid{};
	pid.Kd = 2.0f;
	arm_pid_init_f32(&pid, 1);
	const std::array<float, 5> e = {1.0f, 1.0f, 4.0f, 2.0f, 2.0f};
	const std::array<float, 5> expected = {2.0f, 0.0f, 6.0f, -4.0f, 0.0f};
	for (size_t i = 0; i < e.size(); i++) {
		CAPTURE(i);
		CHECK(arm_pid_f32(&pid, e[i]) == doctest::Approx(expected[i]).scale(1.0).epsilon(1e-6));
	}
}

// ============================================================================================ variance / std

TEST_CASE("arm_var_f32 is the sample variance with n - 1 in the denominator for any block size") {
	// blockSize 4n + r: the two loops unrolled by 4 and the remainder loops
	const std::vector<float> all = testSignal(17, 2024);
	for (size_t n : {2U, 3U, 4U, 5U, 7U, 8U, 16U, 17U}) {
		CAPTURE(n);
		const std::vector<float> v(all.begin(), all.begin() + static_cast<long>(n));
		float var = -1;
		arm_var_f32(v.data(), static_cast<uint32_t>(n), &var);
		CHECK(var == doctest::Approx(referenceVariance(v)).epsilon(TOL));
	}
}

TEST_CASE("arm_var_f32 of two values is half their squared difference") {
	const std::array<float, 2> v = {1.0f, 4.0f};
	float var = -1;
	arm_var_f32(v.data(), 2, &var);
	CHECK(var == 4.5f);
}

TEST_CASE("arm_var_f32 of one sample is 0") {
	const float v = 42.0f;
	float var = -1;
	arm_var_f32(&v, 1, &var);
	CHECK(var == 0.0f);
}

TEST_CASE("arm_var_f32 of no samples is 0") {
	const float v = 42.0f;
	float var = -1;
	arm_var_f32(&v, 0, &var);
	CHECK(var == 0.0f);
}

TEST_CASE("arm_var_f32 of a constant signal is exactly 0") {
	// 3.25 * 16 is exact in float, so the mean is exactly 3.25 (a value like 3.7 gives a rounding residue of 2e-13)
	const std::vector<float> v(16, 3.25f);
	float var = -1;
	arm_var_f32(v.data(), 16, &var);
	CHECK(var == 0.0f);
}

TEST_CASE("arm_var_f32 stays accurate for values with a large common offset (two-pass algorithm)") {
	// 10000 + small deviations: the textbook formula (sum of squares - square of sum) in float would lose all digits
	std::vector<float> v;
	for (float d : testSignal(16, 5)) v.push_back(10000.0f + d);
	float var = -1;
	arm_var_f32(v.data(), 16, &var);
	CHECK(var == doctest::Approx(referenceVariance(v)).epsilon(1e-3));
}

TEST_CASE("arm_std_f32 is the square root of the sample variance") {
	// 16 samples like Axis.cpp:1359 (accel_buffer[16])
	std::vector<float> v;
	for (float d : testSignal(16, 77)) v.push_back(50.0f * d);
	float sd = -1;
	arm_std_f32(v.data(), 16, &sd);
	CHECK(sd == doctest::Approx(std::sqrt(referenceVariance(v))).epsilon(TOL));
}

TEST_CASE("arm_std_f32 of one sample is 0") {
	const float v = -3.0f;
	float sd = -1;
	arm_std_f32(&v, 1, &sd);
	CHECK(sd == 0.0f);
}

} // TEST_SUITE

#endif // USE_DSP_FUNCTIONS
