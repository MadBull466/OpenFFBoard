/*
 * FastMath f16 (phase P11, work order FF16): arm_atan2_f16, arm_vexp_f16, arm_vinverse_f16, arm_vlog_f16,
 * arm_sqrt_f16 (inline in dsp/fast_math_functions_f16.h) and the helper arm_add_f16 (BasicMathFunctions).
 * Requirements of Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md: C1 (ARM's own test ids), C2 (block sizes, guard), C3 (DEF
 * with a derived tolerance), C4 (documented edge behaviour), C5 (agreement with the f32 function), C6 (host notes).
 *
 * Compiled with -D__ARM_FP16_FORMAT_IEEE=1 -D__fp16=_Float16 (file name ends in _f16.cpp, lead decision L2).
 *
 * Pattern files (Libraries/CMSIS-DSP/Testing/Patterns/DSP/FastMath/FastMathF16/), as wired in Testing/desc_f16.txt
 * (suite "Fast Maths F16", lines 406-433) and Testing/Source/Tests/FastMathF16.cpp (setUp, lines 145-270):
 *   test_sqrt_f16         (id 3)  SqrtInput1_f16 (1024 values) -> Sqrt1_f16                        FastMathF16.cpp:83
 *   test_vlog_f16         (id 4)  LogInput1_f16 (125 values) -> Log1_f16, all samples; ids 5/6/7: first 7 / 16 / 23
 *                                 samples (reload(.., maxSamples)); desc.txt labels them "nb=3", "nb=4n", "nb=4n+1"
 *                                 but the real sizes are 7, 16, 23                                  FastMathF16.cpp:104
 *   test_vexp_f16         (id 8)  ExpInput1_f16 (52 values) -> Exp1_f16, ids 9/10/11: first 7 / 16 / 23 :117
 *   test_inverse_f16      (id 12) Samples1_f16 (256 values) -> Inverse1_f16                        FastMathF16.cpp:130
 *   test_atan2_scalar_f16 (id 13) Atan2Input1_f16 (20020 values = 10010 (y,x) pairs) -> Atan2Ref1_f16  :60
 *   test_cos_f16 / test_sin_f16 (ids 1, 2): "disabled" in desc_f16.txt (lines 428-429) and under #if 0 in
 *     FastMathF16.cpp:25-58. There is NO arm_sin_f16 / arm_cos_f16 in the library (fast_math_functions_f16.h declares
 *     only vlog, vexp, vinverse, atan2 and inline sqrt), so these two ids cannot be run. The patterns Angles1/Sin1/Cos1
 *     exist but have no function to test.
 * ARM tolerances (FastMathF16.cpp): SNR_THRESHOLD 60 dB (:6), SNR_ATAN2_THRESHOLD 60 dB (:8), SNR_LOG_THRESHOLD 40 dB
 *   (:7, defined but not used by any test); REL_ERROR 1e-3 and ABS_ERROR 1e-3 (:18-19) for sqrt, vlog, vexp, inverse;
 *   atan2: ABS_ERROR_ATAN 2e-3 and REL_ERROR_ATAN 1e-3 (:22-23). ARM checks "ref" with ASSERT_CLOSE_ERROR
 *   (|out-ref| <= abs + rel*|ref|) and ASSERT_SNR; the vector tests also ASSERT_EMPTY_TAIL (guard). Used unchanged (L4).
 *   The ARM sqrt test accepts ARM_MATH_ARGUMENT_ERROR only for negative inputs (:90).
 *
 * Host vs target (C6): the host runs the portable C paths with _Float16 emulated by the compiler (x86-64 without
 * AVX512-FP16: arithmetic is evaluated through float and rounded to binary16 when stored, so intermediate roundings
 * can differ from a target with native f16 arithmetic, or the MVE paths (arm_vexp_f16 uses vexpq_f16, arm_vinverse_f16
 * vrecip_hiprec_f16, arm_vlog_f16 vlogq_lut_f16 with fused multiply-add on Cortex-M55). The firmware ships no f16 code
 * (Makefile excludes %_f16.c). arm_vexp_f16 computes expf() in float (libm) then converts, so exp results depend on the
 * libm. FMA contraction can change the last bit of the polynomial in arm_vlog_f16 (tolerance 1e-3 covers it).
 * Float to f16 conversion of out-of-range values gives +-infinity (IEEE) on host and target.
 */
#include "doctest.h"

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math_f16.h"

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#ifndef ARM_FLOAT16_SUPPORTED
#error "ARM_FLOAT16_SUPPORTED is not defined: the f16 compile definitions are missing for this file"
#endif

namespace {

const double kAbsErr = 1.0e-3;        // ARM ABS_ERROR (FastMathF16.cpp:19)
const double kRelErr = 1.0e-3;        // ARM REL_ERROR (:18)
const double kAbsAtan = 2.0e-3;       // ARM ABS_ERROR_ATAN (:23)
const double kRelAtan = 1.0e-3;       // ARM REL_ERROR_ATAN (:22)
const double kSnr = 60.0;             // ARM SNR_THRESHOLD / SNR_ATAN2_THRESHOLD
// Half an ulp of binary16 relative to the value: 2^-11. A correctly rounded f16 result is within this of the exact one
const double kHalfUlp = 4.8828125e-4;
const double kPi = 3.14159265358979323846;

typedef void (*VecFn)(const float16_t *, float16_t *, uint32_t);

std::vector<float16_t> toF16(const std::vector<uint16_t> &bits) {
	std::vector<float16_t> v(bits.size());
	for (size_t i = 0; i < bits.size(); i++) std::memcpy(&v[i], &bits[i], sizeof(float16_t));
	return v;
}

std::vector<float16_t> load(const char *name) {
	return toF16(dsp::loadPattern(std::string("FastMath/FastMathF16/") + name).f16Bits());
}

std::vector<float16_t> make(std::initializer_list<double> values) {
	std::vector<float16_t> v;
	for (double d : values) v.push_back(static_cast<float16_t>(d));
	return v;
}

std::vector<float> toFloat(const std::vector<float16_t> &v) { return std::vector<float>(v.begin(), v.end()); }

uint16_t bitsOf(float16_t h) {
	uint16_t b;
	std::memcpy(&b, &h, sizeof b);
	return b;
}

struct VecResult {
	std::vector<float16_t> out;
	bool guardIntact;
};

// Runs a vector function on the first n samples of in, output in a guarded buffer
VecResult runVec(VecFn fn, const std::vector<float16_t> &in, size_t n) {
	dsp::GuardedBuffer<float16_t> buf(n);
	fn(in.data(), buf.data(), static_cast<uint32_t>(n));
	return {buf.output(), buf.guardIntact()};
}

std::vector<float16_t> head(const std::vector<float16_t> &v, size_t n) {
	return std::vector<float16_t>(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(n));
}

// ARM's check of a vector test: SNR >= 60, per-sample abs/rel bound, guard intact
void checkArmVector(VecFn fn, const char *inName, const char *refName, size_t n) {
	std::vector<float16_t> in = load(inName), ref = load(refName);
	REQUIRE(in.size() >= n);
	REQUIRE(ref.size() >= n);
	in = head(in, n);
	ref = head(ref, n);
	VecResult r = runVec(fn, in, n);
	CHECK(dsp::snr(ref, r.out) >= kSnr);
	auto c = dsp::closeAbsRel(ref, r.out, kAbsErr, kRelErr);
	CHECK_MESSAGE(c, c.message);
	CHECK(r.guardIntact);
}

// DEF check of a vector function on hand-picked inputs: every size 1,3,4,5,9 (4 = one unrolled group, 5 = group + tail,
// 3 = tail only, 9 = two groups + tail), every sample against the double reference, guard intact
void checkSizes(VecFn fn, const std::vector<float16_t> &in, double (*def)(double), double absTol, double relTol) {
	REQUIRE(in.size() == 9);
	for (size_t n : {1u, 3u, 4u, 5u, 9u}) {
		INFO("block size " << n);
		VecResult r = runVec(fn, head(in, n), n);
		std::vector<double> ref;
		for (size_t i = 0; i < n; i++) ref.push_back(def(static_cast<double>(in[i])));
		auto c = dsp::closeAbsRel(ref, r.out, absTol, relTol);
		CHECK_MESSAGE(c, c.message);
		CHECK(r.guardIntact);
	}
}

// block size 0: nothing is written (the 0xCD poison stays) and nothing behind it
void checkSizeZero(VecFn fn, const std::vector<float16_t> &in) {
	dsp::GuardedBuffer<float16_t> buf(1);
	fn(in.data(), buf.data(), 0);
	CHECK(bitsOf(buf.output()[0]) == 0xCDCD);
	CHECK(buf.guardIntact());
}

double defExp(double x) { return std::exp(x); }
double defLog(double x) { return std::log(x); }
double defInv(double x) { return 1.0 / x; }

const double kInf = HUGE_VAL;

}  // namespace

TEST_SUITE("DSP FastMath F16") {

// ---------------------------------------------------------------------------------------------------------------
// C1: ARM's test ids
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C1 arm_atan2_f16] ARM test_atan2_scalar_f16 (id 13): 10010 pairs match the reference within SNR 60 dB and 2e-3 abs + 1e-3 rel") {
	std::vector<float16_t> in = load("Atan2Input1_f16.txt"), ref = load("Atan2Ref1_f16.txt");
	REQUIRE(in.size() == 2 * ref.size());
	std::vector<float16_t> out(ref.size());
	bool allSuccess = true;
	for (size_t i = 0; i < ref.size(); i++) {
		float16_t res = 0;
		// ARM: arm_atan2_f16(inp[2*i], inp[2*i+1], &res), i.e. y first, x second
		if (arm_atan2_f16(in[2 * i], in[2 * i + 1], &res) != ARM_MATH_SUCCESS) allSuccess = false;
		out[i] = res;
	}
	CHECK(allSuccess);
	CHECK(dsp::snr(ref, out) >= kSnr);
	auto c = dsp::closeAbsRel(ref, out, kAbsAtan, kRelAtan);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C1 arm_sqrt_f16] ARM test_sqrt_f16 (id 3): 1024 inputs match the reference within SNR 60 dB and 1e-3 abs + 1e-3 rel") {
	std::vector<float16_t> in = load("SqrtInput1_f16.txt"), ref = load("Sqrt1_f16.txt");
	REQUIRE(in.size() == ref.size());
	std::vector<float16_t> out(ref.size());
	bool statusOk = true;
	for (size_t i = 0; i < in.size(); i++) {
		arm_status s = arm_sqrt_f16(in[i], &out[i]);
		// ARM: SUCCESS, or ARGUMENT_ERROR for a negative input
		if (!(s == ARM_MATH_SUCCESS || (static_cast<float>(in[i]) < 0.0f && s == ARM_MATH_ARGUMENT_ERROR))) statusOk = false;
	}
	CHECK(statusOk);
	CHECK(dsp::snr(ref, out) >= kSnr);
	auto c = dsp::closeAbsRel(ref, out, kAbsErr, kRelErr);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C1 arm_vlog_f16] ARM test_vlog_f16 (id 4): all 125 inputs match the reference within SNR 60 dB and 1e-3 abs + 1e-3 rel" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFF16-2: logf16_scalar (arm_vlog_f16.c:83) computes 'vecExpUnBiased << 10' with a negative "
			  "vecExpUnBiased for every input below 1: left shift of a negative value is undefined behaviour in C (UBSan: "
			  "'left shift of negative value'). The intended result occurs on the host; skipped in sanitizer builds")) {
	checkArmVector(arm_vlog_f16, "LogInput1_f16.txt", "Log1_f16.txt", 125);
}

TEST_CASE("[C1 arm_vlog_f16] ARM test_vlog_f16 (id 5): the first 7 inputs match the reference" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFF16-2: logf16_scalar (arm_vlog_f16.c:83) computes 'vecExpUnBiased << 10' with a negative "
			  "vecExpUnBiased for every input below 1: left shift of a negative value is undefined behaviour in C (UBSan: "
			  "'left shift of negative value'). The intended result occurs on the host; skipped in sanitizer builds")) {
	checkArmVector(arm_vlog_f16, "LogInput1_f16.txt", "Log1_f16.txt", 7);
}

TEST_CASE("[C1 arm_vlog_f16] ARM test_vlog_f16 (id 6): the first 16 inputs match the reference" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFF16-2: logf16_scalar (arm_vlog_f16.c:83) computes 'vecExpUnBiased << 10' with a negative "
			  "vecExpUnBiased for every input below 1: left shift of a negative value is undefined behaviour in C (UBSan: "
			  "'left shift of negative value'). The intended result occurs on the host; skipped in sanitizer builds")) {
	checkArmVector(arm_vlog_f16, "LogInput1_f16.txt", "Log1_f16.txt", 16);
}

TEST_CASE("[C1 arm_vlog_f16] ARM test_vlog_f16 (id 7): the first 23 inputs match the reference" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFF16-2: logf16_scalar (arm_vlog_f16.c:83) computes 'vecExpUnBiased << 10' with a negative "
			  "vecExpUnBiased for every input below 1: left shift of a negative value is undefined behaviour in C (UBSan: "
			  "'left shift of negative value'). The intended result occurs on the host; skipped in sanitizer builds")) {
	checkArmVector(arm_vlog_f16, "LogInput1_f16.txt", "Log1_f16.txt", 23);
}

TEST_CASE("[C1 arm_vexp_f16] ARM test_vexp_f16 (id 8): all 52 inputs match the reference within SNR 60 dB and 1e-3 abs + 1e-3 rel") {
	checkArmVector(arm_vexp_f16, "ExpInput1_f16.txt", "Exp1_f16.txt", 52);
}

TEST_CASE("[C1 arm_vexp_f16] ARM test_vexp_f16 (id 9): the first 7 inputs match the reference") {
	checkArmVector(arm_vexp_f16, "ExpInput1_f16.txt", "Exp1_f16.txt", 7);
}

TEST_CASE("[C1 arm_vexp_f16] ARM test_vexp_f16 (id 10): the first 16 inputs match the reference") {
	checkArmVector(arm_vexp_f16, "ExpInput1_f16.txt", "Exp1_f16.txt", 16);
}

TEST_CASE("[C1 arm_vexp_f16] ARM test_vexp_f16 (id 11): the first 23 inputs match the reference") {
	checkArmVector(arm_vexp_f16, "ExpInput1_f16.txt", "Exp1_f16.txt", 23);
}

TEST_CASE("[C1 arm_vinverse_f16] ARM test_inverse_f16 (id 12): 256 inputs match the reference within SNR 60 dB and 1e-3 abs + 1e-3 rel") {
	checkArmVector(arm_vinverse_f16, "Samples1_f16.txt", "Inverse1_f16.txt", 256);
}

TEST_CASE("[C1 arm_sin_f16 arm_cos_f16] ARM ids 1 and 2 (test_cos_f16, test_sin_f16) are disabled and the library has no f16 sin or cos") {
	// desc_f16.txt:428-429 "disabled{...}", FastMathF16.cpp:25-58 under #if 0. fast_math_functions_f16.h declares no
	// arm_sin_f16 / arm_cos_f16, so nothing can be run. This case only documents the gap (patterns exist).
	CHECK(load("Angles1_f16.txt").size() == load("Sin1_f16.txt").size());
	CHECK(load("Angles1_f16.txt").size() == load("Cos1_f16.txt").size());
}

// ---------------------------------------------------------------------------------------------------------------
// C2 / C3: vector functions, block sizes, every sample against the DEF (double), guard
// Tolerances:
//  - vexp: expf() in float (error ~1e-7 relative) converted to binary16 with round to nearest: error <= half ulp =
//    2^-11 relative (kHalfUlp) plus 1e-7; relative bound 5e-4 (> 4.883e-4 + 1e-7), no absolute part (results are
//    normal numbers, e^-2 = 0.135 > 2^-14).
//  - vinverse: 1.0f16 / x is one correctly rounded f16 division: |out - 1/x| <= 2^-11 * |1/x| (normal results).
//  - vlog: a degree 3 polynomial with binary16 coefficients and binary16 arithmetic, no tighter derivation is
//    available than ARM's measured bound: abs 1e-3 + rel 1e-3 (ARM, L4). log(1) = 0 needs the absolute part.
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C2 arm_vexp_f16] every sample of the block sizes 1 3 4 5 9 is exp(x) within half an f16 ulp and the guard is intact") {
	checkSizes(arm_vexp_f16, make({-2, -0.5, 0, 0.5, 1, 1.5, 2, -1, 0.25}), defExp, 0.0, 5.0e-4);
}

TEST_CASE("[C2 arm_vexp_f16] block size 0 writes nothing") {
	checkSizeZero(arm_vexp_f16, make({1.0}));
}

TEST_CASE("[C2 arm_vlog_f16] every sample of the block sizes 1 3 4 5 9 is log(x) within 1e-3 abs + 1e-3 rel and the guard is intact (inputs >= 1)") {
	// inputs >= 1 have a non-negative exponent: no shift of a negative value (see B-DFF16-2), so this runs in sanitizer builds
	checkSizes(arm_vlog_f16, make({1, 1.5, 2, 3, 4, 10, 7, 1.25, 100}), defLog, kAbsErr, kRelErr);
}

TEST_CASE("[C2 arm_vlog_f16] every sample of the block sizes 1 3 4 5 9 is log(x) within 1e-3 abs + 1e-3 rel and the guard is intact (inputs < 1)" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFF16-2: logf16_scalar (arm_vlog_f16.c:83) computes 'vecExpUnBiased << 10' with a negative "
			  "vecExpUnBiased for every input below 1: left shift of a negative value is undefined behaviour in C (UBSan: "
			  "'left shift of negative value'). The intended result occurs on the host; skipped in sanitizer builds")) {
	checkSizes(arm_vlog_f16, make({0.5, 0.9, 0.25, 0.1, 0.75, 0.3, 0.99, 0.01, 0.6}), defLog, kAbsErr, kRelErr);
}

TEST_CASE("[C2 arm_vlog_f16] block size 0 writes nothing") {
	checkSizeZero(arm_vlog_f16, make({1.0}));
}

TEST_CASE("[C2 arm_vinverse_f16] every sample of the block sizes 1 3 4 5 9 is 1/x within half an f16 ulp and the guard is intact") {
	checkSizes(arm_vinverse_f16, make({1, 2, -4, 0.5, 3, -0.25, 10, 100, -7}), defInv, 0.0, kHalfUlp * 1.0001);
}

TEST_CASE("[C2 arm_vinverse_f16] block size 0 writes nothing") {
	checkSizeZero(arm_vinverse_f16, make({2.0}));
}

TEST_CASE("[C2 arm_add_f16] every sample of the block sizes 1 3 4 5 9 is the correctly rounded sum and the guard is intact") {
	// DEF: a + b rounded to binary16. The library adds in (at least) float, exact for these operands, and rounds once.
	std::vector<float16_t> a = make({1.5, -2.0, 0.25, 4.0, 1024.0, 0.1, -3.3, 100.0, 7.0});
	std::vector<float16_t> b = make({2.5, 0.5, -0.25, -4.0, 0.5, 0.2, 3.3, 0.01, -0.001});
	for (size_t n : {1u, 3u, 4u, 5u, 9u}) {
		INFO("block size " << n);
		dsp::GuardedBuffer<float16_t> buf(n);
		arm_add_f16(a.data(), b.data(), buf.data(), static_cast<uint32_t>(n));
		std::vector<float16_t> out = buf.output();
		for (size_t i = 0; i < n; i++) {
			float16_t ref = static_cast<float16_t>(static_cast<double>(a[i]) + static_cast<double>(b[i]));
			INFO("sample " << i);
			CHECK(bitsOf(out[i]) == bitsOf(ref));
		}
		CHECK(buf.guardIntact());
	}
}

TEST_CASE("[C2 arm_add_f16] block size 0 writes nothing") {
	std::vector<float16_t> a = make({1.0}), b = make({1.0});
	dsp::GuardedBuffer<float16_t> buf(1);
	arm_add_f16(a.data(), b.data(), buf.data(), 0);
	CHECK(bitsOf(buf.output()[0]) == 0xCDCD);
	CHECK(buf.guardIntact());
}

// ---------------------------------------------------------------------------------------------------------------
// C4: edge behaviour
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C4 arm_sqrt_f16] a negative input returns ARM_MATH_ARGUMENT_ERROR and the output is 0") {
	// DOC (fast_math_functions_f16.h:55-57): "ARM_MATH_ARGUMENT_ERROR : input value is negative; *pOut is set to 0"
	float16_t out = static_cast<float16_t>(5.0);
	CHECK(arm_sqrt_f16(static_cast<float16_t>(-1.0), &out) == ARM_MATH_ARGUMENT_ERROR);
	CHECK(bitsOf(out) == 0);
}

TEST_CASE("[C4 arm_sqrt_f16] the input 0 returns ARM_MATH_SUCCESS and 0") {
	// DOC: "ARM_MATH_SUCCESS : input value is positive"; sqrt(0) = 0 (DEF)
	float16_t out = static_cast<float16_t>(5.0);
	CHECK(arm_sqrt_f16(static_cast<float16_t>(0.0), &out) == ARM_MATH_SUCCESS);
	CHECK(static_cast<float>(out) == 0.0f);
}

TEST_CASE("[C3 arm_sqrt_f16] exact squares and the largest f16 give the correctly rounded root") {
	// sqrt(4) = 2, sqrt(0.25) = 0.5, sqrt(65504) = 255.9922 (binary16 spacing 0.125 in [128,256)): 256.0 is 0.0078 away,
	// 255.875 is 0.117 away -> rounds to 256.0. DEF in double, tolerance 2^-11 relative.
	const double in[] = {4.0, 0.25, 65504.0, 2.0, 1024.0};
	for (double v : in) {
		float16_t out = 0;
		CHECK(arm_sqrt_f16(static_cast<float16_t>(v), &out) == ARM_MATH_SUCCESS);
		double ref = std::sqrt(v);
		CHECK(std::fabs(static_cast<double>(out) - ref) <= kHalfUlp * ref * 1.0001);
	}
}

TEST_CASE("[C4 arm_atan2_f16] atan2(0, 0) does not return ARM_MATH_SUCCESS") {
	// DOC (fast_math_functions_f16.h:113): "@return error status." The angle of the origin is undefined; the call must
	// not report success. The value of the result is not specified and not checked.
	float16_t res = 0;
	CHECK(arm_atan2_f16(static_cast<float16_t>(0.0), static_cast<float16_t>(0.0), &res) != ARM_MATH_SUCCESS);
}

TEST_CASE("[C4 arm_atan2_f16] the axes give 0 and plus minus pi/2 within 2e-3") {
	// DEF: atan2(0,1) = 0, atan2(1,0) = pi/2, atan2(-1,0) = -pi/2. Tolerance ARM's 2e-3 absolute (library constant
	// PI16HALF = 1.571 as binary16 = 1.5712890625, error 5e-4).
	struct { double y, x, ref; } cases[] = {{0.0, 1.0, 0.0}, {1.0, 0.0, kPi / 2}, {-1.0, 0.0, -kPi / 2}, {5.0, 0.0, kPi / 2}, {0.0, 8.0, 0.0}};
	for (auto &t : cases) {
		float16_t res = static_cast<float16_t>(99.0);
		CHECK(arm_atan2_f16(static_cast<float16_t>(t.y), static_cast<float16_t>(t.x), &res) == ARM_MATH_SUCCESS);
		INFO("y=" << t.y << " x=" << t.x);
		CHECK(std::fabs(static_cast<double>(res) - t.ref) <= kAbsAtan);
	}
}

TEST_CASE("[C4 arm_atan2_f16] on the negative x axis +0 gives +pi and -0 gives -pi within 2e-3") {
	// DEF (C99 atan2): atan2(+0, -1) = +pi, atan2(-0, -1) = -pi; DOC: "The sign of y and x are used to determine the
	// right quadrant ... Returned value is between -Pi and Pi."
	float16_t pos = 0, neg = 0;
	CHECK(arm_atan2_f16(static_cast<float16_t>(0.0), static_cast<float16_t>(-1.0), &pos) == ARM_MATH_SUCCESS);
	CHECK(arm_atan2_f16(static_cast<float16_t>(-0.0), static_cast<float16_t>(-1.0), &neg) == ARM_MATH_SUCCESS);
	CHECK(std::fabs(static_cast<double>(pos) - kPi) <= kAbsAtan);
	CHECK(std::fabs(static_cast<double>(neg) + kPi) <= kAbsAtan);
}

TEST_CASE("[C3 arm_atan2_f16] every quadrant and the diagonals match std::atan2 within 2e-3 abs + 1e-3 rel") {
	// Unit vectors every 15 degrees from -180 to 180, rounded to binary16 (DEF: std::atan2 of the rounded inputs).
	// Tolerance: ARM's (L4), the library uses a degree 4 polynomial with binary16 constants.
	for (int deg = -180; deg <= 180; deg += 15) {
		double a = deg * kPi / 180.0;
		float16_t y = static_cast<float16_t>(std::sin(a)), x = static_cast<float16_t>(std::cos(a));
		float16_t res = 0;
		INFO("angle " << deg);
		CHECK(arm_atan2_f16(y, x, &res) == ARM_MATH_SUCCESS);
		double ref = std::atan2(static_cast<double>(y), static_cast<double>(x));
		CHECK(std::fabs(static_cast<double>(res) - ref) <= kAbsAtan + kRelAtan * std::fabs(ref));
	}
}

TEST_CASE("[C4 arm_atan2_f16] the result stays between -pi and pi for every angle of the circle") {
	// DOC (arm_atan2_f16.c): "Returned value is between -Pi and Pi."
	for (int deg = -180; deg <= 180; deg += 5) {
		double a = deg * kPi / 180.0;
		float16_t y = static_cast<float16_t>(std::sin(a)), x = static_cast<float16_t>(std::cos(a));
		float16_t res = 0;
		INFO("angle " << deg);
		CHECK(arm_atan2_f16(y, x, &res) == ARM_MATH_SUCCESS);
		CHECK(std::fabs(static_cast<double>(res)) <= kPi);
	}
}

TEST_CASE("[C4 arm_vexp_f16] an argument above ln(65504) overflows to +infinity and a very negative one underflows to 0") {
	// DEF: exp(12) = 162755 > 65504 (largest f16) -> +inf (IEEE conversion of the float result to binary16);
	// exp(-20) = 2.06e-9 < 2^-25 (half the smallest subnormal 2^-24 = 5.96e-8) -> 0. DOC is silent about both.
	std::vector<float16_t> in = make({12.0, -20.0});
	VecResult r = runVec(arm_vexp_f16, in, 2);
	CHECK(static_cast<double>(r.out[0]) == kInf);
	CHECK(static_cast<double>(r.out[1]) == 0.0);
	CHECK(r.guardIntact);
}

TEST_CASE("[C4 arm_vexp_f16] exp(0) is 1 and exp(1) is e within half an f16 ulp") {
	VecResult r = runVec(arm_vexp_f16, make({0.0, 1.0}), 2);
	CHECK(static_cast<double>(r.out[0]) == 1.0);
	CHECK(std::fabs(static_cast<double>(r.out[1]) - std::exp(1.0)) <= 5.0e-4 * std::exp(1.0));
}

TEST_CASE("[C4 arm_vlog_f16] log(1) is 0 within the absolute bound 1e-3") {
	VecResult r = runVec(arm_vlog_f16, make({1.0}), 1);
	CHECK(std::fabs(static_cast<double>(r.out[0])) <= kAbsErr);
}

TEST_CASE("[C4 arm_vlog_f16] log of a positive subnormal input is log(x) within 1e-3 abs + 1e-3 rel" *
		  KNOWN_ISSUE_UB("B-DFF16-1: arm_vlog_f16 (arm_vlog_f16.c, logf16_scalar) splits the f16 bit pattern into exponent and "
					  "mantissa without normalising subnormals: log(1e-5 as f16 = 1.0014e-5) should be -11.5 but the "
					  "result is -10.24; log(2^-24) should be -16.6, result -10.40. DOC gives no input range "
					  "('Floating-point vector of log values'); a positive subnormal is a valid input; the call also shifts a negative value (B-DFF16-2), so the case is skipped in sanitizer builds")) {
	// DEF: std::log of the (exactly representable) f16 input. 1e-5 rounds to the subnormal 167 * 2^-24.
	std::vector<float16_t> in = make({1.0e-5, 5.9604644775390625e-8});
	VecResult r = runVec(arm_vlog_f16, in, 2);
	for (size_t i = 0; i < 2; i++) {
		double ref = std::log(static_cast<double>(in[i]));
		INFO("input " << static_cast<double>(in[i]));
		CHECK(std::fabs(static_cast<double>(r.out[i]) - ref) <= kAbsErr + kRelErr * std::fabs(ref));
	}
	CHECK(r.guardIntact);
}

TEST_CASE("[C4 arm_vinverse_f16] the inverse of +0 is +infinity and of -0 is -infinity") {
	// DEF: IEEE division 1/(+-0) = +-inf (the library computes 1.0f16 / x). DOC is silent.
	VecResult r = runVec(arm_vinverse_f16, make({0.0, -0.0}), 2);
	CHECK(static_cast<double>(r.out[0]) == kInf);
	CHECK(static_cast<double>(r.out[1]) == -kInf);
}

TEST_CASE("[C4 arm_vinverse_f16] the inverse of 1 and of -1 is exact") {
	VecResult r = runVec(arm_vinverse_f16, make({1.0, -1.0, 2.0, 0.5}), 4);
	CHECK(static_cast<double>(r.out[0]) == 1.0);
	CHECK(static_cast<double>(r.out[1]) == -1.0);
	CHECK(static_cast<double>(r.out[2]) == 0.5);
	CHECK(static_cast<double>(r.out[3]) == 2.0);
}

TEST_CASE("[C4 arm_add_f16] a sum above the largest f16 becomes +infinity and zeros add to zero") {
	// DEF (IEEE binary16, round to nearest): 60000 + 10000 = 70000 > 65504 + half ulp (65520) -> +inf.
	std::vector<float16_t> a = make({60000.0, -60000.0, 0.0}), b = make({10000.0, -10000.0, 0.0});
	dsp::GuardedBuffer<float16_t> buf(3);
	arm_add_f16(a.data(), b.data(), buf.data(), 3);
	std::vector<float16_t> o = buf.output();
	CHECK(static_cast<double>(o[0]) == kInf);
	CHECK(static_cast<double>(o[1]) == -kInf);
	CHECK(static_cast<double>(o[2]) == 0.0);
	CHECK(buf.guardIntact());
}

// ---------------------------------------------------------------------------------------------------------------
// C5: agreement with the f32 function of the same input
// Bound: the f32 functions agree with the double DEF to about 1e-5 or better, the f16 functions to their ARM
// tolerance, so the two differ by at most the f16 tolerance (1e-3 abs + 1e-3 rel; atan2 2e-3 abs + 1e-3 rel).
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C5 arm_vexp_f16] the f16 result agrees with arm_vexp_f32 of the same inputs within 1e-3 abs + 1e-3 rel") {
	std::vector<float16_t> in = load("ExpInput1_f16.txt");
	std::vector<float> inF = toFloat(in), f32(in.size());
	arm_vexp_f32(inF.data(), f32.data(), static_cast<uint32_t>(in.size()));
	VecResult r = runVec(arm_vexp_f16, in, in.size());
	auto c = dsp::closeAbsRel(f32, r.out, kAbsErr, kRelErr);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C5 arm_vlog_f16] the f16 result agrees with arm_vlog_f32 of the same inputs within 1e-3 abs + 1e-3 rel" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFF16-2: logf16_scalar (arm_vlog_f16.c:83) computes 'vecExpUnBiased << 10' with a negative "
			  "vecExpUnBiased for every input below 1: left shift of a negative value is undefined behaviour in C (UBSan: "
			  "'left shift of negative value'). The intended result occurs on the host; skipped in sanitizer builds")) {
	std::vector<float16_t> in = load("LogInput1_f16.txt");
	std::vector<float> inF = toFloat(in), f32(in.size());
	arm_vlog_f32(inF.data(), f32.data(), static_cast<uint32_t>(in.size()));
	VecResult r = runVec(arm_vlog_f16, in, in.size());
	auto c = dsp::closeAbsRel(f32, r.out, kAbsErr, kRelErr);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C5 arm_sqrt_f16] the f16 result is the binary16 rounding of arm_sqrt_f32 of the same input") {
	// arm_sqrt_f16 is documented as the f32 square root converted to f16 (header, body of the inline function), so the
	// bit pattern must equal the conversion of the f32 result. No tolerance.
	std::vector<float16_t> in = load("SqrtInput1_f16.txt");
	size_t bad = 0;
	for (size_t i = 0; i < in.size(); i++) {
		float32_t f = 0;
		float16_t h = 0;
		arm_sqrt_f32(static_cast<float32_t>(in[i]), &f);
		arm_sqrt_f16(in[i], &h);
		if (bitsOf(h) != bitsOf(static_cast<float16_t>(f))) bad++;
	}
	CHECK(bad == 0);
}

TEST_CASE("[C5 arm_atan2_f16] the f16 result agrees with arm_atan2_f32 of the same inputs within 2e-3 abs + 1e-3 rel") {
	std::vector<float16_t> in = load("Atan2Input1_f16.txt");
	size_t n = in.size() / 2;
	std::vector<float> f32(n), f16(n);
	for (size_t i = 0; i < n; i++) {
		float16_t h = 0;
		arm_atan2_f16(in[2 * i], in[2 * i + 1], &h);
		arm_atan2_f32(static_cast<float32_t>(in[2 * i]), static_cast<float32_t>(in[2 * i + 1]), &f32[i]);
		f16[i] = static_cast<float>(h);
	}
	auto c = dsp::closeAbsRel(f32, f16, kAbsAtan, kRelAtan);
	CHECK_MESSAGE(c, c.message);
}

}  // TEST_SUITE
