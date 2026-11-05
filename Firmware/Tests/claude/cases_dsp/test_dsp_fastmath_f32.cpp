/*
 * CMSIS-DSP FastMath f32 (work order FF32, phase P11 group G1). Requirement ids: Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * Units covered: arm_sin_f32, arm_cos_f32, arm_sqrt_f32 (inline, dsp/fast_math_functions.h:236), arm_atan2_f32,
 * arm_vexp_f32, arm_vlog_f32, and the helper arm_add_f32 (C2/C4 beyond the smoke test).
 *
 * Pattern files used (Libraries/CMSIS-DSP/Testing/Patterns/DSP/FastMath/FastMathF32/), ARM's test functions are in
 * Testing/Source/Tests/FastMathF32.cpp and the suite "Fast Maths F32" in Testing/desc.txt:1827:
 *   cos   : Angles1_f32 (23)     -> Cos1_f32          test_cos_f32 (FastMathF32.cpp:44)
 *   sin   : Angles1_f32 (23)     -> Sin1_f32          test_sin_f32 (:59)
 *   sqrt  : SqrtInput1_f32 (1024)-> Sqrt1_f32         test_sqrt_f32 (:74)
 *   vlog  : LogInput1_f32 (125)  -> Log1_f32          test_vlog_f32 (:92), block sizes 125, 3, 8, 11 (setUp :145-:175)
 *   vexp  : ExpInput1_f32 (52)   -> Exp1_f32          test_vexp_f32 (:104), block sizes 52, 3, 8, 11 (setUp :177-:207)
 *   atan2 : Atan2Input1_f32 (20020 = 10010 pairs y,x) -> Atan2Ref1_f32 (10010)   test_atan2_scalar_f32 (:21)
 * All ARM test ids of the suite are covered (desc.txt:1847-:1858 lists 12 functions: cos, sin, sqrt, 4x vlog, 4x vexp,
 * atan2). "nb=3 / 4n / 4n+1" in desc.txt are the block sizes 3, 8 and 11 of setUp (the first nb samples of the pattern).
 *
 * ARM tolerances (FastMathF32.cpp:10-:21, "reference patterns are generated with a double precision computation"):
 *   SNR_THRESHOLD 119 dB (:10), SNR_ATAN2_THRESHOLD 120 dB (:11), REL_ERROR 1e-6 and ABS_ERROR 1e-5 (:19, :20),
 *   REL_ERROR_ATAN 5e-7 and ABS_ERROR_ATAN 5e-7 (:22, :23). Per sample |out-ref| <= ABS + REL*|ref|.
 *   ARM comments out the SNR assertion of atan2 (FastMathF32.cpp:37, only the per-sample bound is checked): the SNR
 *   check is run here as its own test with ARM's 120 dB threshold.
 *
 * Host vs target (C6): the host runs the portable C path (ARM_MATH_LOOPUNROLL, no ARM_MATH_DSP, no MVE/Neon). The
 * floating-point code is IEEE binary32 on both. Differences that can exist on the target: expf/logf/sqrtf come from the
 * C library in use (glibc here, newlib on the target), so the last bit of expf/logf may differ (the bounds below allow
 * 2 ulp); a compiler may contract a*b+c into a fused multiply-add on the target (sin/cos/atan2 polynomial, differences
 * of 1 ulp, far inside the bounds); arm_sin_f32/arm_cos_f32 convert x/(2*pi) to int32_t with a plain cast, which is
 * undefined for |x| >= 2^31*2*pi (about 1.3e10 rad): no test goes there. arm_sqrt_f32 uses sqrtf on the host
 * (__GNUC_PYTHON__) and the VSQRT.F32 instruction on the target, both correctly rounded.
 */
#include "doctest.h"

#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "DspPatterns.h"
#include "arm_math.h"

namespace {

const char *const kDir = "FastMath/FastMathF32/";

// ARM tolerances (FastMathF32.cpp:10-:23)
const double kSnr = 119.0, kSnrAtan2 = 120.0;
const double kAbs = 1.0e-5, kRel = 1.0e-6;
const double kAbsAtan = 5.0e-7, kRelAtan = 5.0e-7;

// Both ARM assertions of a block: SNR and per-sample bound
void checkBlock(const std::vector<float> &ref, const std::vector<float> &out, double snrMin, double absTol, double relTol) {
	CHECK(dsp::snr(ref, out) >= snrMin);
	auto c = dsp::closeAbsRel(ref, out, absTol, relTol);
	CHECK_MESSAGE(c, c.message);
}

// Number of float-resolution steps: 2 ulp relative bound for libm functions (DEF in double)
const double kLibmRel = 2.4e-7;   // 2 * 2^-24 = 1.19e-7 relative per ulp, times 2

std::vector<float> floatsFrom(const std::vector<double> &v) { return std::vector<float>(v.begin(), v.end()); }

// ARM's vlog test for the first nb samples of the pattern (nb = 0: all)
void armVlog(size_t nb) {
	std::vector<float> in = dsp::loadPattern(std::string(kDir) + "LogInput1_f32.txt").f32();
	std::vector<float> ref = dsp::loadPattern(std::string(kDir) + "Log1_f32.txt").f32();
	REQUIRE(in.size() == ref.size());
	if (nb) { in.resize(nb); ref.resize(nb); }
	dsp::GuardedBuffer<float> out(ref.size());
	arm_vlog_f32(in.data(), out.data(), static_cast<uint32_t>(ref.size()));
	checkBlock(ref, out.output(), kSnr, kAbs, kRel);
	CHECK(out.guardIntact());
}

void armVexp(size_t nb) {
	std::vector<float> in = dsp::loadPattern(std::string(kDir) + "ExpInput1_f32.txt").f32();
	std::vector<float> ref = dsp::loadPattern(std::string(kDir) + "Exp1_f32.txt").f32();
	REQUIRE(in.size() == ref.size());
	if (nb) { in.resize(nb); ref.resize(nb); }
	dsp::GuardedBuffer<float> out(ref.size());
	arm_vexp_f32(in.data(), out.data(), static_cast<uint32_t>(ref.size()));
	checkBlock(ref, out.output(), kSnr, kAbs, kRel);
	CHECK(out.guardIntact());
}

const uint32_t kSizes[] = {0, 1, 3, 4, 5, 9};   // 4 is one full unrolled group, 5 and 9 add a tail, 3 is tail only

}  // namespace

TEST_SUITE("DSP FastMath f32") {

// ---------------------------------------------------------------------------------------------------------------
// C1: ARM's tests
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C1 arm_cos_f32] cos over ARM's angles matches ARM's reference within 119 dB and 1e-5 + 1e-6*|ref|") {
	auto in = dsp::loadPattern(std::string(kDir) + "Angles1_f32.txt").f32();
	auto ref = dsp::loadPattern(std::string(kDir) + "Cos1_f32.txt").f32();
	REQUIRE(in.size() == ref.size());
	std::vector<float> out;
	for (float x : in) out.push_back(arm_cos_f32(x));
	checkBlock(ref, out, kSnr, kAbs, kRel);
}

TEST_CASE("[C1 arm_sin_f32] sin over ARM's angles matches ARM's reference within 119 dB and 1e-5 + 1e-6*|ref|") {
	auto in = dsp::loadPattern(std::string(kDir) + "Angles1_f32.txt").f32();
	auto ref = dsp::loadPattern(std::string(kDir) + "Sin1_f32.txt").f32();
	REQUIRE(in.size() == ref.size());
	std::vector<float> out;
	for (float x : in) out.push_back(arm_sin_f32(x));
	checkBlock(ref, out, kSnr, kAbs, kRel);
}

TEST_CASE("[C1 arm_sqrt_f32] sqrt over ARM's inputs matches ARM's reference and reports an argument error only for negative inputs") {
	auto in = dsp::loadPattern(std::string(kDir) + "SqrtInput1_f32.txt").f32();
	auto ref = dsp::loadPattern(std::string(kDir) + "Sqrt1_f32.txt").f32();
	REQUIRE(in.size() == ref.size());
	std::vector<float> out(in.size());
	bool statusOk = true;
	for (size_t i = 0; i < in.size(); i++) {
		arm_status s = arm_sqrt_f32(in[i], &out[i]);
		// ARM's assertion (FastMathF32.cpp:81): success, or negative input with ARM_MATH_ARGUMENT_ERROR
		if (!(s == ARM_MATH_SUCCESS || (in[i] < 0.0f && s == ARM_MATH_ARGUMENT_ERROR))) statusOk = false;
	}
	CHECK(statusOk);
	checkBlock(ref, out, kSnr, kAbs, kRel);
}

TEST_CASE("[C1 arm_vlog_f32] log of ARM's 125 inputs matches ARM's reference and leaves the guard intact") { armVlog(0); }
TEST_CASE("[C1 arm_vlog_f32] log of the first 3 inputs (ARM nb=3) matches ARM's reference") { armVlog(3); }
TEST_CASE("[C1 arm_vlog_f32] log of the first 8 inputs (ARM nb=4n) matches ARM's reference") { armVlog(8); }
TEST_CASE("[C1 arm_vlog_f32] log of the first 11 inputs (ARM nb=4n+1) matches ARM's reference") { armVlog(11); }

TEST_CASE("[C1 arm_vexp_f32] exp of ARM's 52 inputs matches ARM's reference and leaves the guard intact") { armVexp(0); }
TEST_CASE("[C1 arm_vexp_f32] exp of the first 3 inputs (ARM nb=3) matches ARM's reference") { armVexp(3); }
TEST_CASE("[C1 arm_vexp_f32] exp of the first 8 inputs (ARM nb=4n) matches ARM's reference") { armVexp(8); }
TEST_CASE("[C1 arm_vexp_f32] exp of the first 11 inputs (ARM nb=4n+1) matches ARM's reference") { armVexp(11); }

TEST_CASE("[C1 arm_atan2_f32] atan2 over ARM's 10010 (y,x) pairs matches ARM's reference within 5e-7 + 5e-7*|ref| and succeeds") {
	auto in = dsp::loadPattern(std::string(kDir) + "Atan2Input1_f32.txt").f32();
	auto ref = dsp::loadPattern(std::string(kDir) + "Atan2Ref1_f32.txt").f32();
	REQUIRE(in.size() == 2 * ref.size());
	std::vector<float> out(ref.size());
	bool statusOk = true;
	for (size_t i = 0; i < ref.size(); i++) {
		// ARM calls arm_atan2_f32(inp[2*i], inp[2*i+1], &res): y first, then x (FastMathF32.cpp:30)
		if (arm_atan2_f32(in[2 * i], in[2 * i + 1], &out[i]) != ARM_MATH_SUCCESS) statusOk = false;
	}
	CHECK(statusOk);
	auto c = dsp::closeAbsRel(ref, out, kAbsAtan, kRelAtan);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C1 arm_atan2_f32] atan2 over ARM's pairs reaches 120 dB SNR (the assertion ARM commented out)") {
	auto in = dsp::loadPattern(std::string(kDir) + "Atan2Input1_f32.txt").f32();
	auto ref = dsp::loadPattern(std::string(kDir) + "Atan2Ref1_f32.txt").f32();
	REQUIRE(in.size() == 2 * ref.size());
	std::vector<float> out(ref.size());
	for (size_t i = 0; i < ref.size(); i++) arm_atan2_f32(in[2 * i], in[2 * i + 1], &out[i]);
	CHECK(dsp::snr(ref, out) >= kSnrAtan2);
}

// ---------------------------------------------------------------------------------------------------------------
// C2: block sizes, guard words, every sample
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C2 arm_vlog_f32] every sample is correct for the block sizes 0, 1, 3, 4, 5 and 9 and nothing is written beyond them") {
	// DEF: log of the float input in double; logf is within 2 ulp (rel 2.4e-7), no absolute term: results near 0
	// (input 1) are exactly 0 in logf as well.
	const float in[9] = {0.5f, 1.0f, 2.0f, 3.0f, 10.0f, 0.1f, 100.0f, 1.0e-3f, 7.5f};
	for (uint32_t n : kSizes) {
		CAPTURE(n);
		dsp::GuardedBuffer<float> out(n);
		arm_vlog_f32(in, out.data(), n);
		std::vector<double> ref;
		for (uint32_t i = 0; i < n; i++) ref.push_back(std::log(static_cast<double>(in[i])));
		auto c = dsp::closeAbsRel(floatsFrom(ref), out.output(), 0.0, kLibmRel);
		CHECK_MESSAGE(c, c.message);
		CHECK(out.guardIntact());
	}
}

TEST_CASE("[C2 arm_vexp_f32] every sample is correct for the block sizes 0, 1, 3, 4, 5 and 9 and nothing is written beyond them") {
	const float in[9] = {0.0f, 1.0f, -1.0f, 2.5f, -7.0f, 10.0f, 0.001f, -0.5f, 20.0f};
	for (uint32_t n : kSizes) {
		CAPTURE(n);
		dsp::GuardedBuffer<float> out(n);
		arm_vexp_f32(in, out.data(), n);
		std::vector<double> ref;
		for (uint32_t i = 0; i < n; i++) ref.push_back(std::exp(static_cast<double>(in[i])));
		auto c = dsp::closeAbsRel(floatsFrom(ref), out.output(), 0.0, kLibmRel);
		CHECK_MESSAGE(c, c.message);
		CHECK(out.guardIntact());
	}
}

TEST_CASE("[C2 arm_vlog_f32] a block of size 0 writes nothing") {
	const float in[1] = {2.0f};
	dsp::GuardedBuffer<float> out(1);   // poisoned with 0xCD bytes
	arm_vlog_f32(in, out.data(), 0);
	const std::vector<float> o = out.output();
	unsigned char b[sizeof(float)];
	std::memcpy(b, &o[0], sizeof b);
	bool untouched = true;
	for (unsigned char c : b) if (c != 0xCD) untouched = false;
	CHECK(untouched);
	CHECK(out.guardIntact());
}

TEST_CASE("[C2 arm_vexp_f32] a block of size 0 writes nothing") {
	const float in[1] = {2.0f};
	dsp::GuardedBuffer<float> out(1);
	arm_vexp_f32(in, out.data(), 0);
	const std::vector<float> o = out.output();
	unsigned char b[sizeof(float)];
	std::memcpy(b, &o[0], sizeof b);
	bool untouched = true;
	for (unsigned char c : b) if (c != 0xCD) untouched = false;
	CHECK(untouched);
	CHECK(out.guardIntact());
}

TEST_CASE("[C2 arm_add_f32] every sample is the sum for the block sizes 0, 1, 3, 4, 5 and 9 and nothing is written beyond them") {
	// All values are small multiples of 1/8, so every sum is exact in binary32 and equals the double sum.
	const float a[9] = {1.5f, -2.0f, 0.25f, 4.0f, 1024.0f, -0.125f, 3.0f, 100.5f, -7.75f};
	const float b[9] = {2.5f, 0.5f, -0.25f, -4.0f, 0.0625f, 0.125f, 9.0f, -0.5f, 8.0f};
	for (uint32_t n : kSizes) {
		CAPTURE(n);
		dsp::GuardedBuffer<float> out(n);
		arm_add_f32(a, b, out.data(), n);
		std::vector<float> ref;
		for (uint32_t i = 0; i < n; i++) ref.push_back(static_cast<float>(static_cast<double>(a[i]) + b[i]));
		auto c = dsp::exactEqual(ref, out.output());
		CHECK_MESSAGE(c, c.message);
		CHECK(out.guardIntact());
	}
}

TEST_CASE("[C2 arm_add_f32] a block of size 0 writes nothing") {
	const float a[1] = {1.0f}, b[1] = {2.0f};
	dsp::GuardedBuffer<float> out(1);
	arm_add_f32(a, b, out.data(), 0);
	const std::vector<float> o = out.output();
	unsigned char bytes[sizeof(float)];
	std::memcpy(bytes, &o[0], sizeof bytes);
	bool untouched = true;
	for (unsigned char c : bytes) if (c != 0xCD) untouched = false;
	CHECK(untouched);
	CHECK(out.guardIntact());
}

// ---------------------------------------------------------------------------------------------------------------
// C3: DEF where ARM has no pattern
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_sin_f32] sin over -7..7 rad is within the linear interpolation error of the 512 interval table") {
	// DOC (arm_sin_f32.c): table lookup of 512 values with linear interpolation over one period, h = 2*pi/512 =
	// 0.012272 rad. Interpolation error <= h^2/8 * max|sin''| = 1.508e-4/8 = 1.88e-5. Float rounding: the table value
	// 6e-8, the phase x*0.159154943092f has a relative error of two roundings (6e-8 each) of at most 7/(2*pi) = 1.1 cycles
	// -> 1.3e-7 cycles = 8.4e-7 rad. Bound 1.88e-5 + 1e-6 + 1e-6 = 2.2e-5; 2.5e-5 used.
	std::vector<double> ref;
	std::vector<float> out;
	for (int k = -7000; k <= 7000; k++) {
		float x = static_cast<float>(k) * 0.001f;
		out.push_back(arm_sin_f32(x));
		ref.push_back(std::sin(static_cast<double>(x)));
	}
	auto c = dsp::closeAbsRel(ref, out, 2.5e-5, 0.0);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C3 arm_cos_f32] cos over -7..7 rad is within the linear interpolation error of the 512 interval table") {
	// Same derivation as for sin: cos reads the sine table at phase + 0.25 cycle (arm_cos_f32.c), the added 0.25f is exact.
	std::vector<double> ref;
	std::vector<float> out;
	for (int k = -7000; k <= 7000; k++) {
		float x = static_cast<float>(k) * 0.001f;
		out.push_back(arm_cos_f32(x));
		ref.push_back(std::cos(static_cast<double>(x)));
	}
	auto c = dsp::closeAbsRel(ref, out, 2.5e-5, 0.0);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C3 arm_sin_f32] sin and cos satisfy sin^2 + cos^2 = 1 within the interpolation error") {
	// Both are chords of the unit circle sampled at 512 points: each is at most 1.88e-5 + 1e-6 off, and the sum of
	// squares deviates by at most 2*(2.0e-5)*(|s|+|c|) <= 2*2.0e-5*1.42 = 5.7e-5 (first order); 1e-4 used.
	bool ok = true;
	for (int k = -300; k <= 300; k++) {
		float x = static_cast<float>(k) * 0.0137f;
		double s = arm_sin_f32(x), c = arm_cos_f32(x);
		if (std::fabs(s * s + c * c - 1.0) > 1.0e-4) ok = false;
	}
	CHECK(ok);
}

TEST_CASE("[C3 arm_sqrt_f32] sqrt equals the correctly rounded square root of the input, including subnormals and the largest float") {
	// IEEE 754 requires sqrt to be correctly rounded; sqrt in double rounded to float is correctly rounded too
	// (double rounding is harmless for sqrt, because 53 >= 2*24 + 2). Hence an exact comparison.
	const float in[] = {0.0f, FLT_MIN, 1.0e-45f /* smallest subnormal */, 1.0e-40f, 1.0e-10f, 0.25f, 1.0f, 2.0f, 3.0f,
	                    10.0f, 12345.678f, 1.0e30f, FLT_MAX};
	for (float x : in) {
		CAPTURE(x);
		float out = -1.0f;
		CHECK(arm_sqrt_f32(x, &out) == ARM_MATH_SUCCESS);
		CHECK(out == static_cast<float>(std::sqrt(static_cast<double>(x))));
	}
}

TEST_CASE("[C3 arm_atan2_f32] atan2 of points on a circle in all four quadrants returns the polar angle") {
	// DEF: for (y,x) = (sin t, cos t) in double (rounded to float) atan2 = t for t in (-pi, pi). Bound: ARM's
	// per-sample tolerance of its own double reference test, 5e-7 absolute + 5e-7 relative (L4), covers the polynomial.
	std::vector<double> ref;
	std::vector<float> out;
	for (int deg = -179; deg <= 179; deg++) {
		double t = deg * 3.14159265358979323846 / 180.0;
		float y = static_cast<float>(std::sin(t)), x = static_cast<float>(std::cos(t));
		float r = 0.0f;
		REQUIRE(arm_atan2_f32(y, x, &r) == ARM_MATH_SUCCESS);
		out.push_back(r);
		ref.push_back(std::atan2(static_cast<double>(y), static_cast<double>(x)));
	}
	auto c = dsp::closeAbsRel(ref, out, kAbsAtan, kRelAtan);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C3 arm_atan2_f32] atan2 does not depend on the scale of (y,x)") {
	// atan2(k*y, k*x) = atan2(y, x) for k > 0 (DEF). k = 1e-20 and 1e20 keep y/x and all values normal.
	const float pts[][2] = {{1.0f, 2.0f}, {3.0f, -1.5f}, {-0.5f, -4.0f}, {-2.0f, 0.75f}};
	const float scales[] = {1.0e-20f, 1.0e20f};
	for (const auto &p : pts) {
		for (float k : scales) {
			CAPTURE(k);
			float r = 0.0f;
			REQUIRE(arm_atan2_f32(p[0] * k, p[1] * k, &r) == ARM_MATH_SUCCESS);
			CHECK(std::fabs(static_cast<double>(r) - std::atan2(static_cast<double>(p[0]), static_cast<double>(p[1]))) <= 1.0e-6);
		}
	}
}

// ---------------------------------------------------------------------------------------------------------------
// C4: documented edge behaviour
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C4 arm_sqrt_f32] a negative input returns ARM_MATH_ARGUMENT_ERROR and sets the output to 0") {
	// DOC (fast_math_functions.h:234): "ARM_MATH_ARGUMENT_ERROR : input value is negative; *pOut is set to 0"
	const float in[] = {-1.0f, -0.4f, -FLT_MIN, -FLT_MAX, -std::numeric_limits<float>::infinity()};
	for (float x : in) {
		CAPTURE(x);
		float out = 123.0f;
		CHECK(arm_sqrt_f32(x, &out) == ARM_MATH_ARGUMENT_ERROR);
		CHECK(out == 0.0f);
	}
}

TEST_CASE("[C4 arm_sqrt_f32] zero returns ARM_MATH_SUCCESS and 0") {
	// DOC: argument error only if the input value is negative; 0 is not negative.
	float out = 123.0f;
	CHECK(arm_sqrt_f32(0.0f, &out) == ARM_MATH_SUCCESS);
	CHECK(out == 0.0f);
}

TEST_CASE("[C4 arm_sqrt_f32] characterization: negative zero returns ARM_MATH_SUCCESS and a zero") {
	// DOC is silent on -0.0f. IEEE 754 compares -0 == +0 (not negative) and sqrt(-0) = -0 (a zero); the sign of the zero is
	// not asserted. NEEDS REVIEW (listed in the report).
	float out = 123.0f;
	CHECK(arm_sqrt_f32(-0.0f, &out) == ARM_MATH_SUCCESS);
	CHECK(out == 0.0f);
}

TEST_CASE("[C4 arm_sqrt_f32] positive infinity gives infinity and success") {
	// DEF: IEEE 754 sqrt(+inf) = +inf; not negative, hence no argument error (DOC)
	float out = 0.0f;
	CHECK(arm_sqrt_f32(std::numeric_limits<float>::infinity(), &out) == ARM_MATH_SUCCESS);
	CHECK(std::isinf(out));
	CHECK(out > 0.0f);
}

TEST_CASE("[C4 arm_sqrt_f32] characterization: NaN returns ARM_MATH_ARGUMENT_ERROR and sets the output to 0") {
	// DOC is silent on NaN ("input value is negative"). The check `in >= 0` is false for NaN, so the error branch is taken
	// and *pOut = 0 as for a negative input; a NaN is "not a valid positive input". NEEDS REVIEW (listed in the report).
	float out = 123.0f;
	CHECK(arm_sqrt_f32(std::numeric_limits<float>::quiet_NaN(), &out) == ARM_MATH_ARGUMENT_ERROR);
	CHECK(out == 0.0f);
}

TEST_CASE("[C4 arm_sin_f32] sin of 0 is 0 and of the multiples of 2*pi is within the table error of 0, indexes up to the table end are in range") {
	// DOC: "a value of 2*pi wraps around to 0" (arm_sin_f32.c; the sentence is about the fixed point input, the code
	// handles the float case: "when in is exactly 1, rotate the index down to 0"). The ulp neighbours of k*2*pi (k = -3..3)
	// reach phases just below and exactly at a whole number of cycles, i.e. the last table interval and the wrap.
	// Bound: 2.5e-5 as derived in the [C3 arm_sin_f32] sweep test (phase error grows to 3 cycles * 1.2e-7 = 3.6e-7
	// cycles = 2.3e-6 rad, inside the margin).
	CHECK(arm_sin_f32(0.0f) == 0.0f);
	bool ok = true;
	for (int k = -3; k <= 3; k++) {
		float centre = static_cast<float>(k * 2.0 * 3.14159265358979323846);
		float x = centre;
		for (int s = 0; s < 3; s++) x = std::nextafter(x, -1.0e9f);   // start 3 ulp below
		for (int s = -3; s <= 3; s++, x = std::nextafter(x, 1.0e9f)) {
			if (std::fabs(static_cast<double>(arm_sin_f32(x)) - std::sin(static_cast<double>(x))) > 2.5e-5) ok = false;
			if (std::fabs(static_cast<double>(arm_cos_f32(x)) - std::cos(static_cast<double>(x))) > 2.5e-5) ok = false;
		}
	}
	CHECK(ok);
}

TEST_CASE("[C4 arm_cos_f32] cos of 0 is 1 and cos of pi is -1 within the table error") {
	// DEF: cos(0) = 1, cos(pi) = -1. cos(0) reads the table node of phase 0.25 cycle (index 128, a node, no
	// interpolation): bound 1e-6 (float rounding of the node). cos(pi): float pi differs from pi by 8.7e-8 rad and the
	// table is flat there: bound 2.5e-5 as in the sweep test.
	CHECK(std::fabs(arm_cos_f32(0.0f) - 1.0f) <= 1.0e-6f);
	CHECK(std::fabs(arm_cos_f32(3.14159265358979f) + 1.0f) <= 2.5e-5f);
}

TEST_CASE("[C4 arm_atan2_f32] the four axes return 0, +-pi/2 and +-pi") {
	// DOC (arm_atan2_f32.c): "Returned value is between -Pi and Pi". DEF: atan2(0,+x) = 0, atan2(+y,0) = pi/2,
	// atan2(-y,0) = -pi/2, atan2(+0,-x) = pi, atan2(-0,-x) = -pi. Bound 1e-6: PI in the library is the float
	// 3.14159265358979f (error 8.7e-8) and PIHALFF32 the float pi/2 (4.4e-8).
	const double kPi = 3.14159265358979323846;
	struct Case { float y, x; double want; } cases[] = {
		{0.0f, 2.0f, 0.0}, {3.0f, 0.0f, kPi / 2}, {-3.0f, 0.0f, -kPi / 2}, {0.0f, -2.0f, kPi}, {-0.0f, -2.0f, -kPi},
	};
	for (const auto &c : cases) {
		CAPTURE(c.y);
		CAPTURE(c.x);
		float r = 99.0f;
		CHECK(arm_atan2_f32(c.y, c.x, &r) == ARM_MATH_SUCCESS);
		CHECK(std::fabs(static_cast<double>(r) - c.want) <= 1.0e-6);
	}
}

TEST_CASE("[C4 arm_atan2_f32] the diagonals of the four quadrants return +-pi/4 and +-3pi/4") {
	// DEF: atan2(1,1) = pi/4, atan2(1,-1) = 3pi/4, atan2(-1,-1) = -3pi/4, atan2(-1,1) = -pi/4. Bound: ARM's 5e-7 + 5e-7*|ref|.
	const double kPi = 3.14159265358979323846;
	struct Case { float y, x; double want; } cases[] = {
		{1.0f, 1.0f, kPi / 4}, {1.0f, -1.0f, 3 * kPi / 4}, {-1.0f, -1.0f, -3 * kPi / 4}, {-1.0f, 1.0f, -kPi / 4},
	};
	for (const auto &c : cases) {
		CAPTURE(c.y);
		CAPTURE(c.x);
		float r = 99.0f;
		CHECK(arm_atan2_f32(c.y, c.x, &r) == ARM_MATH_SUCCESS);
		CHECK(std::fabs(static_cast<double>(r) - c.want) <= kAbsAtan + kRelAtan * std::fabs(c.want));
	}
}

TEST_CASE("[C4 arm_atan2_f32] a ratio y/x that overflows or underflows still gives the limit angle") {
	// DEF: atan2(1e30, 1e-30) -> pi/2 (y/x = inf in binary32), atan2(1e30, -1e-30) -> pi/2 (y>0, x<0, y/x = -inf),
	// atan2(-1e30, -1e-30) -> -pi/2, atan2(1e-30, -1e30) -> pi (y/x underflows to -0), atan2(-1e-30, 1e30) -> -0.
	// Bound 1e-6 (float constants of the library).
	const double kPi = 3.14159265358979323846;
	struct Case { float y, x; double want; } cases[] = {
		{1.0e30f, 1.0e-30f, kPi / 2}, {1.0e30f, -1.0e-30f, kPi / 2}, {-1.0e30f, -1.0e-30f, -kPi / 2},
		{1.0e-30f, -1.0e30f, kPi}, {-1.0e-30f, 1.0e30f, 0.0},
	};
	for (const auto &c : cases) {
		CAPTURE(c.y);
		CAPTURE(c.x);
		float r = 99.0f;
		CHECK(arm_atan2_f32(c.y, c.x, &r) == ARM_MATH_SUCCESS);
		CHECK(std::fabs(static_cast<double>(r) - c.want) <= 1.0e-6);
	}
}

TEST_CASE("[C4 arm_atan2_f32] characterization: atan2(0, 0) returns ARM_MATH_NANINF") {
	// DOC only says "@return error status" and does not name the code. The angle of the origin is undefined, so an error
	// status is expected; the code ARM_MATH_NANINF (the final return of the function) is what this checks, and the
	// output value is not asserted. NEEDS REVIEW (listed in the report).
	float r = 99.0f;
	CHECK(arm_atan2_f32(0.0f, 0.0f, &r) == ARM_MATH_NANINF);
}

TEST_CASE("[C4 arm_vlog_f32] log of 1 is 0, of 0 is -inf and of a negative value is NaN") {
	// DEF: C99 Annex F: log(1) = +0, log(+-0) = -inf, log(x < 0) = NaN. The library calls logf (arm_vlog_f32.c); on the
	// target newlib's logf follows the same Annex F.
	const float in[5] = {1.0f, 0.0f, -1.0f, -0.0f, -1.0e-30f};
	dsp::GuardedBuffer<float> out(5);
	arm_vlog_f32(in, out.data(), 5);
	const std::vector<float> o = out.output();
	CHECK(o[0] == 0.0f);
	CHECK(std::isinf(o[1]));
	CHECK(o[1] < 0.0f);
	CHECK(std::isnan(o[2]));
	CHECK(std::isinf(o[3]));
	CHECK(o[3] < 0.0f);
	CHECK(std::isnan(o[4]));
	CHECK(out.guardIntact());
}

TEST_CASE("[C4 arm_vlog_f32] log of subnormals and of the largest floats is correct") {
	// DEF in double, bound 2 ulp relative; the results are -103.28 (smallest subnormal), -87.34 (FLT_MIN) and 88.72.
	const float in[4] = {1.0e-45f, FLT_MIN, FLT_MAX, 1.0e-40f};
	dsp::GuardedBuffer<float> out(4);
	arm_vlog_f32(in, out.data(), 4);
	std::vector<double> ref;
	for (float x : in) ref.push_back(std::log(static_cast<double>(x)));
	auto c = dsp::closeAbsRel(floatsFrom(ref), out.output(), 0.0, kLibmRel);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C4 arm_vlog_f32] log of +inf is +inf and log of NaN is NaN") {
	const float in[2] = {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()};
	dsp::GuardedBuffer<float> out(2);
	arm_vlog_f32(in, out.data(), 2);
	const std::vector<float> o = out.output();
	CHECK(std::isinf(o[0]));
	CHECK(o[0] > 0.0f);
	CHECK(std::isnan(o[1]));
}

TEST_CASE("[C4 arm_vexp_f32] exp of 0 is 1, of a large positive value overflows to +inf and of a large negative value is 0") {
	// DEF: C99 Annex F: exp(0) = 1, exp(+inf) = +inf, exp(-inf) = +0; ln(FLT_MAX) = 88.7228, so exp(89) overflows
	// to +inf in binary32 and exp(-104) is below half the smallest subnormal (ln(0.7e-45) = -103.97) and rounds to 0.
	const float in[6] = {0.0f, 89.0f, -104.0f, std::numeric_limits<float>::infinity(),
	                     -std::numeric_limits<float>::infinity(), 1000.0f};
	dsp::GuardedBuffer<float> out(6);
	arm_vexp_f32(in, out.data(), 6);
	const std::vector<float> o = out.output();
	CHECK(o[0] == 1.0f);
	CHECK(std::isinf(o[1]));
	CHECK(o[1] > 0.0f);
	CHECK(o[2] == 0.0f);
	CHECK(std::isinf(o[3]));
	CHECK(o[3] > 0.0f);
	CHECK(o[4] == 0.0f);
	CHECK(std::isinf(o[5]));
	CHECK(out.guardIntact());
}

TEST_CASE("[C4 arm_vexp_f32] exp near the overflow limit and in the subnormal range is correct") {
	// DEF in double. Normal results: 2 ulp relative. Subnormal results (exp(-100) = 3.7e-44, 26 subnormal ulps): the
	// resolution is 1.4e-45 absolute, so the bound is one subnormal ulp (1.5e-45) plus the relative part.
	const float in[4] = {88.0f, 88.7f, -100.0f, -87.0f};
	dsp::GuardedBuffer<float> out(4);
	arm_vexp_f32(in, out.data(), 4);
	std::vector<double> ref;
	for (float x : in) ref.push_back(std::exp(static_cast<double>(x)));
	auto c = dsp::closeAbsRel(floatsFrom(ref), out.output(), 1.5e-45, kLibmRel);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C4 arm_vexp_f32] exp of NaN is NaN") {
	const float in[1] = {std::numeric_limits<float>::quiet_NaN()};
	dsp::GuardedBuffer<float> out(1);
	arm_vexp_f32(in, out.data(), 1);
	CHECK(std::isnan(out.output()[0]));
}

TEST_CASE("[C4 arm_add_f32] overflow gives infinity and inf - inf gives NaN, in the unrolled groups and in the tail") {
	// DEF: IEEE 754 binary32 addition (the function has no saturation, DOC: pDst[n] = pSrcA[n] + pSrcB[n]).
	// Block of 9: indexes 0-3 and 4-7 are the unrolled groups, index 8 the tail.
	const float inf = std::numeric_limits<float>::infinity();
	const float a[9] = {FLT_MAX, 1.0f, inf, -FLT_MAX, 2.0f, -inf, 3.0f, 4.0f, FLT_MAX};
	const float b[9] = {FLT_MAX, 1.0f, -inf, -FLT_MAX, 2.0f, inf, 3.0f, 4.0f, FLT_MAX};
	dsp::GuardedBuffer<float> out(9);
	arm_add_f32(a, b, out.data(), 9);
	const std::vector<float> o = out.output();
	CHECK((std::isinf(o[0]) && o[0] > 0.0f));
	CHECK(o[1] == 2.0f);
	CHECK(std::isnan(o[2]));
	CHECK((std::isinf(o[3]) && o[3] < 0.0f));
	CHECK(o[4] == 4.0f);
	CHECK(std::isnan(o[5]));
	CHECK(o[6] == 6.0f);
	CHECK(o[7] == 8.0f);
	CHECK((std::isinf(o[8]) && o[8] > 0.0f));
	CHECK(out.guardIntact());
}

}  // TEST_SUITE
