/*
 * Interpolation f16 (phase P11, group G2, work order IH): arm_linear_interp_f16 and arm_bilinear_interp_f16.
 * Requirements of Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md: C1 (ARM's own test ids), C2 (both functions return a scalar by value, so
 * there is no library output buffer; the guard words behind the output array of the C1 cases only mirror ARM's
 * ASSERT_EMPTY_TAIL and only prove that the test's own loop stays in bounds), C3 (DEF in double with a derived
 * tolerance), C4 (documented edge behaviour), C5 (agreement with the f32 function), C6 (host notes).
 *
 * Compiled with -D__ARM_FP16_FORMAT_IEEE=1 -D__fp16=_Float16 (file name ends in _f16.cpp, lead decision L2).
 *
 * Pattern files (Libraries/CMSIS-DSP/Testing/Patterns/DSP/Interpolation/InterpolationF16/) as wired in
 * Testing/desc_f16.txt (suite "Interpolation Tests F16", lines 218-255) and Testing/Source/Tests/InterpolationTestsF16.cpp:
 *   test_linear_interp_f16   (id 1)  Input1_f16 (42 x values) with the table YVals1_f16 (41 values, x1 = 0, xSpacing = 1,
 *                                    nValues = 41) -> Reference1_f16 (42)               InterpolationTestsF16.cpp:12, setUp 110-122
 *   test_bilinear_interp_f16 (id 2)  Input2_f16 (300 values = 150 (x,y) pairs), Config2_s16 (numCols = 7, numRows = 8),
 *                                    table YVals2_f16 (56 values) -> Reference2_f16 (150)   :35, setUp 124-137
 *   ids 3, 4, 5 (test_spline_square / sine / ramp_f16): under #if 0 (InterpolationTestsF16.cpp:62-114, :150-186) and not
 *     listed in desc_f16.txt (Functions block lines 252-253 has only ids 1 and 2). The library has no arm_spline_f16
 *     (interpolation_functions_f16.h declares only the two functions above), so they cannot be run. The patterns
 *     InputX/InputY/OutputX/Reference 3..5 exist; one case only documents the gap.
 * ARM tolerances (InterpolationTestsF16.cpp): SNR_THRESHOLD 44 dB (:4), REL_ERROR 6.0e-3 (:13), ABS_ERROR 7.0e-3 (:14);
 *   ARM checks ASSERT_EMPTY_TAIL(output), ASSERT_SNR and ASSERT_CLOSE_ERROR (|out-ref| <= abs + rel*|ref|). Used unchanged (L4).
 *   The suite name here is "DSP Interpolation F16".
 *
 * Tolerances of the DEF cases (eps = 2^-11 = 4.8828125e-4, half an ulp of binary16 relative to the value):
 *  - On dyadic inputs (small integer table values, coordinates that are multiples of 1/16 or 1/8) every intermediate value
 *    is exactly representable in binary16 (<= 11 significant bits, derivation at each case) so the result must be EXACT
 *    (tolerance 0), whatever the evaluation precision of the compiler.
 *  - On general inputs, a rigorous bound from the rounding of each binary16 operation, derived at linear and bilinear
 *    below (linearBound(), bilinearBound()). They are looser than the typical error but hold for any evaluation order.
 *
 * Host vs target (C6): the host runs the portable C code with _Float16 emulated by the compiler (x86-64 without
 * AVX512-FP16: arithmetic through float, rounded to binary16 when stored). A target with native f16 arithmetic (Cortex-M55,
 * MVE) rounds every operation, so the last bit can differ; the derived bounds hold for both. The conversions
 * (int32_t)(float16) in the index calculation are in range for every finite f16 input of the tests (|x| <= 65504). NaN or
 * infinite inputs convert to int32_t with undefined result (C float-to-int conversion) and are NOT tested. The firmware
 * ships no f16 code (Makefile excludes %_f16.c).
 */
#include "doctest.h"

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math_f16.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#ifndef ARM_FLOAT16_SUPPORTED
#error "ARM_FLOAT16_SUPPORTED is not defined: the f16 compile definitions are missing for this file"
#endif

namespace {

const double kAbsErr = 7.0e-3;        // ARM ABS_ERROR (InterpolationTestsF16.cpp:14)
const double kRelErr = 6.0e-3;        // ARM REL_ERROR (:13)
const double kSnr = 44.0;             // ARM SNR_THRESHOLD (:4)
const double kEps = 4.8828125e-4;     // 2^-11
const double kTiny = 1.0e-7;          // absolute slack for results below the smallest normal f16 (6.1e-5, ulp 6e-8)

std::vector<float16_t> toF16(const std::vector<uint16_t> &bits) {
	std::vector<float16_t> v(bits.size());
	for (size_t i = 0; i < bits.size(); i++) std::memcpy(&v[i], &bits[i], sizeof(float16_t));
	return v;
}

std::vector<float16_t> load(const char *name) {
	return toF16(dsp::loadPattern(std::string("Interpolation/InterpolationF16/") + name).f16Bits());
}

float16_t h(double d) { return static_cast<float16_t>(d); }
double d(float16_t v) { return static_cast<double>(v); }

float16_t fromBits(uint16_t b) {
	float16_t v;
	std::memcpy(&v, &b, sizeof v);
	return v;
}

uint16_t bitsOf(float16_t v) {
	uint16_t b;
	std::memcpy(&b, &v, sizeof b);
	return b;
}

std::vector<float> toFloat(const std::vector<float16_t> &v) { return std::vector<float>(v.begin(), v.end()); }

// ---------------------------------------------------------------------------------------------------------------
// Linear interpolation: DEF and bound
// DEF (header text of arm_linear_interp_f32.c: y = y0 + (x - x0) * ((y1 - y0)/(x1 - x0)); below the table the first value,
// above the last the last value). Bound for the binary16 evaluation, x0 = x1(table) + i*spacing:
//  - x - x0: exact (both on a grid of at least 2^-10 relative to <= 1 spacing; a difference of two binary16 values
//    within a factor 2 is exact, and in the tests x0 is a small multiple of the spacing);
//  - y1 - y0 rounded:           error <= eps*|y1 - y0|,                 scaled by the fraction (x-x0)/spacing <= 1
//  - (y1-y0)/(x1-x0) rounded:   error <= eps*|y1 - y0| / spacing (once more the slope, same scale)
//  - (x - x0)*slope rounded:    error <= eps*|(x-x0)*slope| <= eps*|y1 - y0|
//  - y0 + product rounded:      error <= eps*|y| <= eps*max(|y0|,|y1|)
// Sum: eps*(3*|y1 - y0| + max(|y0|,|y1|)) (+ kTiny for results below the normal range).
// ---------------------------------------------------------------------------------------------------------------
struct LinTable {
	std::vector<float16_t> y;
	double x1;
	double spacing;
};

arm_linear_interp_instance_f16 makeLinear(const LinTable &t) {
	arm_linear_interp_instance_f16 S;
	S.nValues = static_cast<uint32_t>(t.y.size());
	S.x1 = h(t.x1);
	S.xSpacing = h(t.spacing);
	S.pYData = t.y.data();
	return S;
}

double linearDef(const LinTable &t, double x, double *bound) {
	size_t n = t.y.size();
	*bound = 0.0;
	if (x < t.x1) return d(t.y[0]);
	double pos = std::floor((x - t.x1) / t.spacing);
	if (pos >= static_cast<double>(n - 1)) return d(t.y[n - 1]);
	size_t i = static_cast<size_t>(pos);
	double y0 = d(t.y[i]), y1 = d(t.y[i + 1]);
	double x0 = t.x1 + static_cast<double>(i) * t.spacing;
	*bound = kEps * (3.0 * std::fabs(y1 - y0) + std::max(std::fabs(y0), std::fabs(y1))) + kTiny;
	return y0 + (x - x0) * (y1 - y0) / t.spacing;
}

// Checks every x against the DEF. exact: tolerance 0 (dyadic case). Returns "" or the first deviation.
std::string checkLinear(const LinTable &t, const std::vector<float16_t> &xs, bool exact) {
	arm_linear_interp_instance_f16 S = makeLinear(t);
	for (float16_t x : xs) {
		double bound;
		double ref = linearDef(t, d(x), &bound);
		if (exact) bound = 0.0;
		double out = d(arm_linear_interp_f16(&S, x));
		if (!(std::fabs(out - ref) <= bound))
			return "x = " + std::to_string(d(x)) + ": out " + std::to_string(out) + ", DEF " + std::to_string(ref) +
				   ", bound " + std::to_string(bound);
	}
	return "";
}

// All binary16 values with the bit patterns lo..hi (positive range; for negative values set the sign bit)
std::vector<float16_t> patterns(uint16_t lo, uint16_t hi, unsigned stride, bool negate) {
	std::vector<float16_t> v;
	for (unsigned b = lo; b <= hi; b += stride) v.push_back(fromBits(static_cast<uint16_t>(negate ? (b | 0x8000u) : b)));
	return v;
}

LinTable armTable() { return {load("YVals1_f16.txt"), 0.0, 1.0}; }

// ---------------------------------------------------------------------------------------------------------------
// Bilinear interpolation: DEF and bound
// DEF (header text): XF = floor(x), YF = floor(y), f = f00*(1-dx)*(1-dy) + f01*dx*(1-dy) + f10*(1-dx)*dy + f11*dx*dy with
// dx = x - XF, dy = y - YF, table element (x, y) at pData[x + y*numCols]; outside the table the result is 0.
// Bound for the binary16 evaluation of the code (b1 = f00, b2 = f01-f00, b3 = f10-f00, b4 = f00-f01-f10+f11,
// out = b1 + b2*dx + b3*dy + b4*dx*dy), M = max |f|, dx and dy exact (|X - XF| exact for the same reason as above),
// dx, dy <= 1, every rounding <= eps*|value|:
//   b2, b3 (|.| <= 2M)                 2 eps M each, scaled by dx or dy <= 1       -> 4 eps M
//   b4: three subtractions/additions with partial results <= 2M, 3M, 4M            -> 9 eps M (scale dx*dy <= 1)
//   products b2*dx, b3*dy (<= 2M each), b4*dx, b4*dx*dy (<= 4M each)              -> 12 eps M
//   three sums with partial results <= 3M, 5M, 9M                                  -> 17 eps M
//   total 42 eps M; 43 used (second order terms) plus kTiny.
// ---------------------------------------------------------------------------------------------------------------
struct BiTable {
	std::vector<float16_t> v;
	uint16_t rows, cols;
};

arm_bilinear_interp_instance_f16 makeBilinear(const BiTable &t) {
	arm_bilinear_interp_instance_f16 S;
	S.numRows = t.rows;
	S.numCols = t.cols;
	S.pData = t.v.data();
	return S;
}

// Only for points inside [0, cols-1) x [0, rows-1) (see the lead decision about the last row/column)
double bilinearDef(const BiTable &t, double x, double y, double *bound) {
	double xf = std::floor(x), yf = std::floor(y);
	size_t xi = static_cast<size_t>(xf), yi = static_cast<size_t>(yf);
	double dx = x - xf, dy = y - yf;
	double f00 = d(t.v[xi + yi * t.cols]), f01 = d(t.v[xi + 1 + yi * t.cols]);
	double f10 = d(t.v[xi + (yi + 1) * t.cols]), f11 = d(t.v[xi + 1 + (yi + 1) * t.cols]);
	double M = std::max(std::max(std::fabs(f00), std::fabs(f01)), std::max(std::fabs(f10), std::fabs(f11)));
	*bound = 43.0 * kEps * M + kTiny;
	return f00 * (1 - dx) * (1 - dy) + f01 * dx * (1 - dy) + f10 * (1 - dx) * dy + f11 * dx * dy;
}

BiTable armMatrix() {
	std::vector<int16_t> cfg = dsp::loadPattern("Interpolation/InterpolationF16/Config2_s16.txt").q15();
	REQUIRE(cfg.size() == 2);
	BiTable t{load("YVals2_f16.txt"), static_cast<uint16_t>(cfg[1]), static_cast<uint16_t>(cfg[0])};
	REQUIRE(t.v.size() == static_cast<size_t>(t.rows) * t.cols);
	return t;
}

// Dyadic matrix with values in -2..2: v[i] = ((3*i) mod 5) - 2
BiTable dyadicMatrix(uint16_t rows, uint16_t cols) {
	BiTable t{{}, rows, cols};
	for (unsigned i = 0; i < static_cast<unsigned>(rows) * cols; i++) t.v.push_back(h(static_cast<int>((3 * i) % 5) - 2));
	return t;
}

}  // namespace

TEST_SUITE("DSP Interpolation F16") {

// ---------------------------------------------------------------------------------------------------------------
// C1: ARM's test ids
// ---------------------------------------------------------------------------------------------------------------

// Runs ARM's linear test body: 42 inputs through the instance of ARM's table, output in a guarded buffer
struct ArmLinearRun {
	std::vector<float16_t> ref, out;
	bool guardIntact;
};

ArmLinearRun runArmLinear() {
	std::vector<float16_t> in = load("Input1_f16.txt"), ref = load("Reference1_f16.txt");
	LinTable t = armTable();
	REQUIRE(in.size() == 42);
	REQUIRE(ref.size() == in.size());
	REQUIRE(t.y.size() == 41);
	arm_linear_interp_instance_f16 S = makeLinear(t);
	dsp::GuardedBuffer<float16_t> buf(in.size());
	for (size_t i = 0; i < in.size(); i++) buf.data()[i] = arm_linear_interp_f16(&S, in[i]);
	return {ref, buf.output(), buf.guardIntact()};
}

const size_t kArmWorstSample = 35;   // x = 34.09375, see the case below

TEST_CASE("[C1 arm_linear_interp_f16] ARM test_linear_interp_f16 (id 1): SNR >= 44 dB and 41 of the 42 samples within 7e-3 abs + 6e-3 rel") {
	// Sample 35 is excluded here and checked in the next case (it exceeds ARM's per-sample bound by 1.3e-5)
	ArmLinearRun r = runArmLinear();
	CHECK(r.guardIntact);
	CHECK(dsp::snr(r.ref, r.out) >= kSnr);
	r.ref.erase(r.ref.begin() + kArmWorstSample);
	r.out.erase(r.out.begin() + kArmWorstSample);
	auto c = dsp::closeAbsRel(r.ref, r.out, kAbsErr, kRelErr);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C1 arm_linear_interp_f16] ARM test_linear_interp_f16 (id 1): sample 35, where the host result is marginally outside ARM's own bound, matches the exact definition on the f16 data within 2 f16 ulps") {
	// NEEDS REVIEW: at sample 35 (x = 34.09375) ARM's per-sample bound (7.4944e-3) fails on the host. Exact double DEF on the
	// f16 input and table: -0.08988953; ARM reference -0.08239746 (7.4921e-3 from the DEF, inside ARM's bound); library
	// result -0.0899048 (1.5e-5 from the DEF, 7.5070e-3 from the reference, over the bound by 1.3e-5). The overshoot comes
	// from the f16 intermediate rounding of the host (emulated _Float16), not from the reference.
	// Bound: 2 f16 ulps at the result magnitude. |result| = 0.0899 lies in [2^-4, 2^-3), ulp = 2^-4 * 2^-10 = 6.1e-5, so
	// 1.22e-4. Worst case of the roundings for this sample: y1-y0 = 1.2027 (in [1,2), ulp 2^-10, half ulp 4.9e-4, times the
	// fraction 0.094: 4.6e-5), the slope the same (4.6e-5), the product 0.113 (half ulp 3.1e-5), the sum (half ulp 3.1e-5):
	// total 1.5e-4 worst case; both roundings of the slope chain can only partly add up, 2 ulps = 1.22e-4 is used and
	// the measured error is 1.5e-5.
	// C6: the C1 result depends on how the compiler rounds f16 intermediates; the result on the target (native f16
	// arithmetic or MVE) is unverified.
	ArmLinearRun r = runArmLinear();
	LinTable t = armTable();
	double bound;
	double ref = linearDef(t, 34.09375, &bound);
	CHECK(bound > 0.0);
	int e;
	std::frexp(std::fabs(ref), &e);
	double ulp = std::ldexp(1.0, e - 11);   // |ref| = m * 2^e, m in [0.5, 1): ulp of binary16 = 2^(e-1-10)
	CHECK_NEAR(r.out[kArmWorstSample], ref, 2.0 * ulp);
}

TEST_CASE("[C1 arm_bilinear_interp_f16] ARM test_bilinear_interp_f16 (id 2): 150 (x,y) pairs match the reference within SNR 44 dB and 7e-3 abs + 6e-3 rel") {
	std::vector<float16_t> in = load("Input2_f16.txt"), ref = load("Reference2_f16.txt");
	BiTable t = armMatrix();
	REQUIRE(t.cols == 7);
	REQUIRE(t.rows == 8);
	REQUIRE(in.size() == 300);
	REQUIRE(ref.size() == 150);
	arm_bilinear_interp_instance_f16 S = makeBilinear(t);
	dsp::GuardedBuffer<float16_t> buf(ref.size());
	for (size_t i = 0; i < ref.size(); i++) buf.data()[i] = arm_bilinear_interp_f16(&S, in[2 * i], in[2 * i + 1]);
	std::vector<float16_t> out = buf.output();
	CHECK(buf.guardIntact());
	CHECK(dsp::snr(ref, out) >= kSnr);
	auto c = dsp::closeAbsRel(ref, out, kAbsErr, kRelErr);
	CHECK_MESSAGE(c, c.message);
}

// ---------------------------------------------------------------------------------------------------------------
// C3: DEF, linear
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_linear_interp_f16] on dyadic data the result is exactly the linear interpolation (x multiples of 1/16, integer table, spacing 1)") {
	// Table y[k] = ((7k) mod 9) - 4 in -4..4, k = 0..40, x1 = 0, spacing 1. Exactness: x - x0 is a multiple of 1/16 below 1,
	// y1 - y0 is an integer |.| <= 8, slope = the same (division by 1), (x-x0)*slope is a multiple of 1/16 with |.| <= 8
	// (<= 7 significant bits), y0 + product is a multiple of 1/16 with |.| <= 12 (<= 8 bits): all exact in binary16.
	LinTable t{{}, 0.0, 1.0};
	for (int k = 0; k <= 40; k++) t.y.push_back(h(((7 * k) % 9) - 4));
	std::vector<float16_t> xs;
	for (int j = -32; j <= 44 * 16; j++) xs.push_back(h(j / 16.0));
	std::string err = checkLinear(t, xs, true);
	CHECK_MESSAGE(err.empty(), err);
}

TEST_CASE("[C3 arm_linear_interp_f16] with x1 = -2 and spacing 0.5 the result is exactly the linear interpolation") {
	// Table {-3, 1, 0, 4, -2} at x = -2, -1.5, -1, -0.5, 0. Spacing is a power of two so x - x1 and the division by the
	// spacing are exact; x multiples of 1/16, y1-y0 an integer |.| <= 6, slope an even integer |.| <= 12 and
	// (x-x0)*slope a multiple of 1/16 with |.| <= 6: all exact.
	LinTable t{{h(-3), h(1), h(0), h(4), h(-2)}, -2.0, 0.5};
	std::vector<float16_t> xs;
	for (int j = -4 * 16; j <= 2 * 16; j++) xs.push_back(h(j / 16.0));
	std::string err = checkLinear(t, xs, true);
	CHECK_MESSAGE(err.empty(), err);
}

TEST_CASE("[C3 arm_linear_interp_f16] every binary16 x from 0 to 48 (stride 1) matches the DEF within the derived bound (ARM table)") {
	// All 0x5200 non-negative bit patterns up to 48 (beyond the table end at 40: clamped to the last value), so isolated
	// bad inputs between ARM's 42 patterns are found. Bound: linearBound derivation at the top of the file.
	std::string err = checkLinear(armTable(), patterns(0x0000, 0x5200, 1, false), false);
	CHECK_MESSAGE(err.empty(), err);
}

TEST_CASE("[C3 arm_linear_interp_f16] every negative binary16 x down to -64 returns the first table value exactly") {
	// DEF: x below x1 returns pYData[0]; the bound is 0 in linearDef for that region
	LinTable t = armTable();
	std::vector<float16_t> xs = patterns(0x0001, 0x5400, 1, true);
	std::string err = checkLinear(t, xs, false);
	CHECK_MESSAGE(err.empty(), err);
}

TEST_CASE("[C3 arm_linear_interp_f16] at every table point the result is the table value exactly") {
	// DEF: x = x1 + k*spacing -> y[k]. Code: x - x0 = 0 so y = y0 + 0*slope = y0 (last point: the clamp, y[n-1]).
	LinTable t = armTable();
	arm_linear_interp_instance_f16 S = makeLinear(t);
	bool ok = true;
	size_t bad = 0;
	for (size_t k = 0; k < t.y.size(); k++) {
		float16_t out = arm_linear_interp_f16(&S, h(static_cast<double>(k)));
		if (bitsOf(out) != bitsOf(t.y[k]) && !(d(out) == d(t.y[k]))) {
			ok = false;
			bad = k;
			break;
		}
	}
	CHECK_MESSAGE(ok, "table point " << bad << " not reproduced");
}

// ---------------------------------------------------------------------------------------------------------------
// C4: documented edge behaviour, linear
// DOC (arm_linear_interp_f32.c, group text, valid for the f16 variant): "if x is outside of the table boundary, Linear
// interpolation returns first value of the table if x is below input range and returns last value of table if x is above
// range." Code comments in arm_linear_interp_f16.c agree.
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C4 arm_linear_interp_f16] an input below the table start returns the first table value") {
	LinTable t = armTable();
	arm_linear_interp_instance_f16 S = makeLinear(t);
	for (double x : {-0.0009765625, -0.5, -1.0, -1000.0, -65504.0}) {
		INFO("x = " << x);
		CHECK(d(arm_linear_interp_f16(&S, h(x))) == d(t.y[0]));
	}
}

TEST_CASE("[C4 arm_linear_interp_f16] an input at or above the last table point returns the last table value") {
	LinTable t = armTable();
	arm_linear_interp_instance_f16 S = makeLinear(t);
	for (double x : {40.0, 40.03125, 41.0, 1000.0, 65504.0}) {
		INFO("x = " << x);
		CHECK(d(arm_linear_interp_f16(&S, h(x))) == d(t.y[40]));
	}
}

TEST_CASE("[C4 arm_linear_interp_f16] the first and the last interval interpolate between their own two points") {
	// DEF on {2, 6, ..., 3, 0}: x = 0.5 -> 4 (between y0 = 2 and y1 = 6); x = 39.5 between y[39] and y[40]
	LinTable t = armTable();
	arm_linear_interp_instance_f16 S = makeLinear(t);
	double mid0 = 0.5 * (d(t.y[0]) + d(t.y[1])), mid39 = 0.5 * (d(t.y[39]) + d(t.y[40]));
	double b0 = kEps * (3.0 * std::fabs(d(t.y[1]) - d(t.y[0])) + std::max(std::fabs(d(t.y[0])), std::fabs(d(t.y[1])))) + kTiny;
	double b39 = kEps * (3.0 * std::fabs(d(t.y[40]) - d(t.y[39])) + std::max(std::fabs(d(t.y[39])), std::fabs(d(t.y[40])))) + kTiny;
	CHECK_NEAR(arm_linear_interp_f16(&S, h(0.5)), mid0, b0);
	CHECK_NEAR(arm_linear_interp_f16(&S, h(39.5)), mid39, b39);
}

TEST_CASE("[C4 arm_linear_interp_f16] a table of two points interpolates between them and clamps outside") {
	LinTable t{{h(1), h(5)}, 0.0, 1.0};
	arm_linear_interp_instance_f16 S = makeLinear(t);
	CHECK(d(arm_linear_interp_f16(&S, h(-1))) == 1.0);
	CHECK(d(arm_linear_interp_f16(&S, h(0))) == 1.0);
	CHECK(d(arm_linear_interp_f16(&S, h(0.25))) == 2.0);   // 1 + 0.25*4, exact
	CHECK(d(arm_linear_interp_f16(&S, h(0.5))) == 3.0);
	CHECK(d(arm_linear_interp_f16(&S, h(0.75))) == 4.0);
	CHECK(d(arm_linear_interp_f16(&S, h(1))) == 5.0);
	CHECK(d(arm_linear_interp_f16(&S, h(7))) == 5.0);
}

TEST_CASE("[C4 arm_linear_interp_f16] a table of one point returns that value for every input") {
	// DOC: first value below the range, last value above it; with one point both are pYData[0] (the code takes the
	// "above" branch for every x >= x1 because nValues - 1 == 0)
	LinTable t{{h(3.5)}, 0.0, 1.0};
	arm_linear_interp_instance_f16 S = makeLinear(t);
	for (double x : {-5.0, 0.0, 0.5, 100.0}) {
		INFO("x = " << x);
		CHECK(d(arm_linear_interp_f16(&S, h(x))) == 3.5);
	}
}

TEST_CASE("[C4 arm_linear_interp_f16] a falling table interpolates downwards (negative slope)") {
	LinTable t{{h(4), h(0), h(-4)}, 0.0, 1.0};
	arm_linear_interp_instance_f16 S = makeLinear(t);
	CHECK(d(arm_linear_interp_f16(&S, h(0.5))) == 2.0);
	CHECK(d(arm_linear_interp_f16(&S, h(1.5))) == -2.0);
	CHECK(d(arm_linear_interp_f16(&S, h(1.0))) == 0.0);
}

// ---------------------------------------------------------------------------------------------------------------
// C3: DEF, bilinear
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_bilinear_interp_f16] on a dyadic 5 x 4 matrix the result is exactly the bilinear interpolation (x, y multiples of 1/8)") {
	// Matrix v[i] = ((3i) mod 5) - 2 (values -2..2), numCols 5, numRows 4. dx, dy multiples of 1/8 below 1; b2, b3 integers
	// |.| <= 4; b4 an integer |.| <= 8; b4*dx multiple of 1/8 (|.| <= 8), *dy multiple of 1/64 (|.| <= 8: <= 10 bits);
	// partial sums are multiples of 1/64 with |.| <= 18 (<= 11 bits): every operation is exact in binary16.
	BiTable t = dyadicMatrix(4, 5);
	arm_bilinear_interp_instance_f16 S = makeBilinear(t);
	bool ok = true;
	std::string msg;
	for (int jy = 0; jy < 3 * 8 && ok; jy++)
		for (int jx = 0; jx < 4 * 8; jx++) {
			double x = jx / 8.0, y = jy / 8.0, bound;
			double ref = bilinearDef(t, x, y, &bound);
			double out = d(arm_bilinear_interp_f16(&S, h(x), h(y)));
			if (out != ref) {
				ok = false;
				msg = "x = " + std::to_string(x) + ", y = " + std::to_string(y) + ": out " + std::to_string(out) + ", DEF " + std::to_string(ref);
				break;
			}
		}
	CHECK_MESSAGE(ok, msg);
}

TEST_CASE("[C3 arm_bilinear_interp_f16] binary16 coordinates on a stride grid over the ARM matrix match the DEF within the derived bound") {
	// X over the bit patterns 0x0000..0x45FF (0 .. just below 6 = numCols-1), Y over 0x0000..0x46FF (0 .. just below 7),
	// strides 61 and 67 (about 300 x 275 pairs), so isolated bad inputs between ARM's 150 pairs are found.
	BiTable t = armMatrix();
	arm_bilinear_interp_instance_f16 S = makeBilinear(t);
	std::vector<float16_t> xs = patterns(0x0000, 0x45FF, 61, false), ys = patterns(0x0000, 0x46FF, 67, false);
	std::string err;
	for (float16_t y : ys) {
		for (float16_t x : xs) {
			double bound;
			double ref = bilinearDef(t, d(x), d(y), &bound);
			double out = d(arm_bilinear_interp_f16(&S, x, y));
			if (!(std::fabs(out - ref) <= bound)) {
				err = "x = " + std::to_string(d(x)) + ", y = " + std::to_string(d(y)) + ": out " + std::to_string(out) +
					  ", DEF " + std::to_string(ref) + ", bound " + std::to_string(bound);
				break;
			}
		}
		if (!err.empty()) break;
	}
	CHECK_MESSAGE(err.empty(), err);
}

TEST_CASE("[C3 arm_bilinear_interp_f16] at every table point not in the last row or column the result is the table value exactly") {
	// DEF: (x, y) integers -> f(x, y) = pData[x + y*numCols]. Code: dx = dy = 0, out = b1 = f00 (+ 0 + 0 + 0)
	BiTable t = armMatrix();
	arm_bilinear_interp_instance_f16 S = makeBilinear(t);
	std::string err;
	for (unsigned y = 0; y + 1 < t.rows && err.empty(); y++)
		for (unsigned x = 0; x + 1 < t.cols; x++) {
			double out = d(arm_bilinear_interp_f16(&S, h(x), h(y)));
			if (out != d(t.v[x + y * t.cols])) {
				err = "table point (" + std::to_string(x) + "," + std::to_string(y) + ")";
				break;
			}
		}
	CHECK_MESSAGE(err.empty(), err);
}

TEST_CASE("[C3 arm_bilinear_interp_f16] the table is read row by row with x along the columns") {
	// Hand derivation: 3 columns x 2 rows, pData = {0,10,20, 100,110,120}. (x,y) = (1.5, 0.5): xIndex 1, yIndex 0;
	// f00 = pData[1] = 10, f01 = pData[2] = 20, f10 = pData[1+3] = 110, f11 = pData[2+3] = 120; dx = dy = 0.5:
	// 10*0.25 + 20*0.25 + 110*0.25 + 120*0.25 = 65. If x and y were swapped the other quadrant would give a different value.
	BiTable t{{h(0), h(10), h(20), h(100), h(110), h(120)}, 2, 3};
	arm_bilinear_interp_instance_f16 S = makeBilinear(t);
	CHECK(d(arm_bilinear_interp_f16(&S, h(1.5), h(0.5))) == 65.0);
	// (0.5, 0.0): between pData[0] = 0 and pData[1] = 10 -> 5. (0.0, 0.5): between pData[0] = 0 and pData[3] = 100 -> 50
	CHECK(d(arm_bilinear_interp_f16(&S, h(0.5), h(0.0))) == 5.0);
	CHECK(d(arm_bilinear_interp_f16(&S, h(0.0), h(0.5))) == 50.0);
}

// ---------------------------------------------------------------------------------------------------------------
// C4: documented edge behaviour, bilinear
// DOC (arm_bilinear_interp_f32.c group text): "if (x,y) are outside of the table boundary, Bilinear interpolation returns
// zero output." and "XF = floor(x), YF = floor(y)". Code comment in arm_bilinear_interp_f16.c: "Returns zero output when
// values are outside table boundary".
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C4 arm_bilinear_interp_f16] the smallest table, 2 x 2, interpolates inside its only cell") {
	// pData = {1, 2, 3, 5}: f00 = 1, f01 = 2, f10 = 3, f11 = 5. (0.5, 0.5): (1+2+3+5)/4 = 2.75. (0.5, 0): 1.5. (0, 0.5): 2.
	BiTable t{{h(1), h(2), h(3), h(5)}, 2, 2};
	arm_bilinear_interp_instance_f16 S = makeBilinear(t);
	CHECK(d(arm_bilinear_interp_f16(&S, h(0.5), h(0.5))) == 2.75);
	CHECK(d(arm_bilinear_interp_f16(&S, h(0.0), h(0.0))) == 1.0);
	CHECK(d(arm_bilinear_interp_f16(&S, h(0.5), h(0.0))) == 1.5);
	CHECK(d(arm_bilinear_interp_f16(&S, h(0.0), h(0.5))) == 2.0);
}

TEST_CASE("[C4 arm_bilinear_interp_f16] coordinates beyond the table end in x or in y return zero") {
	// ARM matrix: numCols 7 (x 0..6), numRows 8 (y 0..7). Points clearly outside (not on the last row or column)
	BiTable t = armMatrix();
	arm_bilinear_interp_instance_f16 S = makeBilinear(t);
	for (double x : {6.5, 7.0, 100.0, 65504.0}) {
		INFO("x = " << x);
		CHECK(d(arm_bilinear_interp_f16(&S, h(x), h(1.5))) == 0.0);
	}
	for (double y : {7.5, 8.0, 100.0, 65504.0}) {
		INFO("y = " << y);
		CHECK(d(arm_bilinear_interp_f16(&S, h(1.5), h(y))) == 0.0);
	}
	CHECK(d(arm_bilinear_interp_f16(&S, h(10), h(10))) == 0.0);
}

TEST_CASE("[C4 arm_bilinear_interp_f16] coordinates of -1 and below return zero") {
	BiTable t = armMatrix();
	arm_bilinear_interp_instance_f16 S = makeBilinear(t);
	for (double v : {-1.0, -1.5, -100.0, -65504.0}) {
		INFO("negative coordinate " << v);
		CHECK(d(arm_bilinear_interp_f16(&S, h(v), h(1.5))) == 0.0);
		CHECK(d(arm_bilinear_interp_f16(&S, h(1.5), h(v))) == 0.0);
	}
}

TEST_CASE("[C4 arm_bilinear_interp_f16] a coordinate between -1 and 0 is outside the table and returns zero" *
		  KNOWN_ISSUE("B-DIH-1: arm_bilinear_interp_f16.c:68-69 converts X and Y with '(int32_t) X', which truncates toward zero, so "
					  "X = -0.5 gives xIndex 0 and passes the 'xIndex < 0' test. DOC says XF = floor(x) and that points outside the "
					  "table return zero; the code instead extrapolates the first cell (xdiff = -0.5). Same in arm_bilinear_interp_f32.c")) {
	// 2 x 2 table {1, 2, 3, 5}. DEF: x = -0.5 is outside [0, 1] -> 0. Code extrapolates: 1 + 1*(-0.5) = 0.5.
	BiTable t{{h(1), h(2), h(3), h(5)}, 2, 2};
	arm_bilinear_interp_instance_f16 S = makeBilinear(t);
	CHECK(d(arm_bilinear_interp_f16(&S, h(-0.5), h(0.0))) == 0.0);
	CHECK(d(arm_bilinear_interp_f16(&S, h(0.0), h(-0.5))) == 0.0);
}

TEST_CASE("characterization: [C4 arm_bilinear_interp_f16] a coordinate on the last column or last row returns zero (NEEDS REVIEW)") {
	// DOC: points outside the table return zero, the last column (x = numCols-1) and row are table points, yet the code
	// tests 'xIndex > numCols - 2' and returns 0 there. Documents today's behaviour; DOC is silent on the limit.
	BiTable t{{h(1), h(2), h(3), h(5)}, 2, 2};
	arm_bilinear_interp_instance_f16 S = makeBilinear(t);
	CHECK(d(arm_bilinear_interp_f16(&S, h(1.0), h(0.0))) == 0.0);
	CHECK(d(arm_bilinear_interp_f16(&S, h(0.0), h(1.0))) == 0.0);
	CHECK(d(arm_bilinear_interp_f16(&S, h(1.0), h(1.0))) == 0.0);
}

// ---------------------------------------------------------------------------------------------------------------
// C5: agreement with the f32 function of the same input
// Bound: the f32 function equals the double DEF to about 1e-6 relative (float evaluation), the f16 function is within the
// derived bound of the DEF, so they differ by at most that bound plus 1e-6 * (|value| + M).
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C5 arm_linear_interp_f16] the f16 result agrees with arm_linear_interp_f32 for every binary16 x from -2 to 48 within the derived bound") {
	LinTable t = armTable();
	arm_linear_interp_instance_f16 S16 = makeLinear(t);
	std::vector<float> y32 = toFloat(t.y);
	arm_linear_interp_instance_f32 S32;
	S32.nValues = static_cast<uint32_t>(y32.size());
	S32.x1 = 0.0f;
	S32.xSpacing = 1.0f;
	S32.pYData = y32.data();
	std::vector<float16_t> xs = patterns(0x0000, 0x5200, 1, false);
	std::vector<float16_t> neg = patterns(0x0001, 0x4000, 1, true);
	xs.insert(xs.end(), neg.begin(), neg.end());
	std::string err;
	for (float16_t x : xs) {
		double bound;
		linearDef(t, d(x), &bound);
		double ref32 = static_cast<double>(arm_linear_interp_f32(&S32, static_cast<float>(x)));
		double out = d(arm_linear_interp_f16(&S16, x));
		bound += 1.0e-6 * (std::fabs(ref32) + 1.0);
		if (!(std::fabs(out - ref32) <= bound)) {
			err = "x = " + std::to_string(d(x)) + ": f16 " + std::to_string(out) + ", f32 " + std::to_string(ref32) + ", bound " + std::to_string(bound);
			break;
		}
	}
	CHECK_MESSAGE(err.empty(), err);
}

TEST_CASE("[C5 arm_bilinear_interp_f16] the f16 result agrees with arm_bilinear_interp_f32 on the ARM pairs and a stride grid, zero outside, within the derived bound") {
	BiTable t = armMatrix();
	arm_bilinear_interp_instance_f16 S16 = makeBilinear(t);
	std::vector<float> v32 = toFloat(t.v);
	arm_bilinear_interp_instance_f32 S32;
	S32.numRows = t.rows;
	S32.numCols = t.cols;
	S32.pData = v32.data();
	// ARM's 150 pairs plus a coarser grid, plus points outside the table (both functions return 0 there)
	std::vector<float16_t> in = load("Input2_f16.txt");
	std::vector<float16_t> xs, ys;
	for (size_t i = 0; i + 1 < in.size(); i += 2) {
		xs.push_back(in[i]);
		ys.push_back(in[i + 1]);
	}
	for (float16_t y : patterns(0x0000, 0x46FF, 211, false))
		for (float16_t x : patterns(0x0000, 0x45FF, 197, false)) {
			xs.push_back(x);
			ys.push_back(y);
		}
	for (double v : {6.5, 7.5, 9.0, -1.0, -3.0}) {
		xs.push_back(h(v));
		ys.push_back(h(1.5));
		xs.push_back(h(1.5));
		ys.push_back(h(v));
	}
	std::string err;
	for (size_t i = 0; i < xs.size(); i++) {
		double x = d(xs[i]), y = d(ys[i]);
		double ref32 = static_cast<double>(arm_bilinear_interp_f32(&S32, static_cast<float>(xs[i]), static_cast<float>(ys[i])));
		double out = d(arm_bilinear_interp_f16(&S16, xs[i], ys[i]));
		double M = 0.0;
		for (float16_t e : t.v) M = std::max(M, std::fabs(d(e)));
		// ARM's own inputs may lie on the last row or column (the f32 function returns 0 there as well, same code)
		double bound = 43.0 * kEps * M + kTiny + 1.0e-6 * (std::fabs(ref32) + M);
		if (!(std::fabs(out - ref32) <= bound)) {
			err = "x = " + std::to_string(x) + ", y = " + std::to_string(y) + ": f16 " + std::to_string(out) + ", f32 " + std::to_string(ref32);
			break;
		}
	}
	CHECK_MESSAGE(err.empty(), err);
}

}  // TEST_SUITE
