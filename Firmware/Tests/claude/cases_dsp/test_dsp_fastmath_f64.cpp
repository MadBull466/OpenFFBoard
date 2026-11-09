/*
 * FastMath f64 (work order FF64, requirements C1-C4 of Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md).
 *
 * Units covered: arm_vlog_f64 and arm_vexp_f64 (Source/FastMathFunctions/arm_vlog_f64.c, arm_vexp_f64.c: a loop over
 * the C library log() / exp(), no unrolling, no tables).
 * No other f64 FastMath function exists in the library: there is no arm_sin_f64, arm_cos_f64, arm_sqrt_f64 or
 * arm_atan2_f64 (fast_math_functions.h declares only the two vector functions for float64_t). ARM's own suite
 * "Fast Maths F64" (Testing/desc.txt:1795-1825) lists test_cos_f64, test_sin_f64 and test_sqrt_f64 as disabled{...} and
 * their bodies are commented out in Testing/Source/Tests/FastMathF64.cpp:17-67. So there is nothing to run for them.
 *
 * ARM ids covered (desc.txt:1816-1823): test_vlog_f64 (all 125 samples), nb=2, nb=2n (4 samples), nb=2n+1 (5 samples);
 * test_vexp_f64 (all 52 samples), nb=2, nb=2n, nb=2n+1. Block sizes as in FastMathF64.cpp:106-171 (setUp, the
 * optional third argument of reload is the sample count).
 * Pattern files (Testing/Patterns/DSP/FastMath/FastMathF64/): LogInput1_f64.txt -> Log1_f64.txt, ExpInput1_f64.txt ->
 * Exp1_f64.txt.
 * ARM tolerances (FastMathF64.cpp:9-17): SNR_THRESHOLD 310 dB (ASSERT_SNR), REL_ERROR 2.0e-16 and ABS_ERROR 2.0e-16
 * (ASSERT_CLOSE_ERROR: |out-ref| <= abs + rel*|ref|), ASSERT_EMPTY_TAIL. Used unchanged (L4).
 *
 * C2: output in a GuardedBuffer, block sizes 0, 1, 3, 4, 5, 9 and 16; reference by DEF (std::log / std::exp in double).
 *   The tolerance is 4 ulp of binary64 (relative 4 * 2^-52 = 8.9e-16, rounded up to 1e-15): the function is the host
 *   libm, and a faithfully rounded result is within 1 ulp; 4 ulp leaves room for a different libm.
 * C4: DOC (fast_math_functions.h:141-146 and :189-194) only says "Floating-point vector of log/exp values" and gives no
 *   edge behaviour, so the edge cases follow the DEF, the C99/IEEE 754 results of log() and exp() (Annex F.10.3):
 *   log(+0) = -inf, log(x<0) = NaN, log(+inf) = +inf, log(NaN) = NaN, exp(-inf) = 0, exp(+inf) = +inf, exp(overflow)
 *   = +inf, exp(underflow) = 0 (or a denormal). These are well defined, not suspicious: no characterization tests.
 *
 * Host vs target (C6): the target links newlib's libm (soft double on a Cortex-M4F: double is NOT in the FPU), the host
 * the glibc libm. Both claim at most 1 ulp, but the last bit can differ between them, so the bounds above are in ulp
 * and are not bit-exact. No FMA, no float to integer conversion, no shift in this code.
 */
#include "doctest.h"

#include "DspPatterns.h"
#include "arm_math.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace {

// ARM's tolerances (FastMathF64.cpp:9-17)
constexpr double kSnrThreshold = 310.0;
constexpr double kRel = 2.0e-16;
constexpr double kAbs = 2.0e-16;

using VecFn = void (*)(const float64_t *, float64_t *, uint32_t);

// One ARM test: the first nb samples (nb = 0 means all) of input and reference, run the function in one call
void runArm(VecFn fn, const char *inFile, const char *refFile, size_t nb) {
	std::vector<double> in = dsp::loadPattern(std::string("FastMath/FastMathF64/") + inFile).f64();
	std::vector<double> ref = dsp::loadPattern(std::string("FastMath/FastMathF64/") + refFile).f64();
	REQUIRE(in.size() == ref.size());
	if (nb) {
		REQUIRE(in.size() >= nb);
		in.resize(nb);
		ref.resize(nb);
	}
	dsp::GuardedBuffer<double> out(ref.size());
	fn(in.data(), out.data(), static_cast<uint32_t>(ref.size()));
	const std::vector<double> got = out.output();
	CHECK(dsp::snr(ref, got) >= kSnrThreshold);
	auto c = dsp::closeAbsRel(ref, got, kAbs, kRel);
	CHECK_MESSAGE(c, c.message);
	CHECK(out.guardIntact());
}

// C2: block size n of the DEF reference, tolerance 4 ulp (relative 1e-15)
void runBlock(VecFn fn, double (*def)(double), const std::vector<double> &pool, size_t n) {
	REQUIRE(pool.size() >= n);
	std::vector<double> in(pool.begin(), pool.begin() + static_cast<std::ptrdiff_t>(n));
	std::vector<double> ref;
	for (double x : in) ref.push_back(def(x));
	dsp::GuardedBuffer<double> out(n);
	// non-null source pointer even for n = 0
	const double dummy = 1.0;
	fn(n ? in.data() : &dummy, out.data(), static_cast<uint32_t>(n));
	CHECK(out.guardIntact());
	auto c = dsp::closeAbsRel(ref, out.output(), 0.0, 1e-15);
	CHECK_MESSAGE(c, c.message);
}

// GuardedBuffer fills the output with 0xCD bytes; true if the sample still has that pattern (never written)
bool poisoned(double v) {
	unsigned char b[sizeof v];
	std::memcpy(b, &v, sizeof v);
	for (unsigned char c : b) if (c != 0xCD) return false;
	return true;
}

double defLog(double x) { return std::log(x); }
double defExp(double x) { return std::exp(x); }

// Inputs of the block size tests (16 values, all in the normal range of the function)
const std::vector<double> kLogPool = {1.0, 2.0, 0.5, 10.0, 1e-3, 1e3, 3.14159, 1e-300,
                                      1e300, 7.0, 0.25, 123456.789, 1.0000001, 0.9999999, 42.0, 5e-5};
const std::vector<double> kExpPool = {0.0, 1.0, -1.0, 2.5, -2.5, 10.0, -10.0, 0.1,
                                      700.0, -700.0, 0.5, 20.25, 1e-10, -1e-10, 3.0, -0.75};

const double kInf = std::numeric_limits<double>::infinity();
const double kNaN = std::numeric_limits<double>::quiet_NaN();

}  // namespace

TEST_SUITE("DSP FastMath f64") {

// ---------------------------------------------------------------------------------------------------------------
// C1: ARM's test ids
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_vlog_f64] all 125 samples of ARM's log pattern match the reference (test_vlog_f64)") {
	runArm(arm_vlog_f64, "LogInput1_f64.txt", "Log1_f64.txt", 0);
}
TEST_CASE("[C1 arm_vlog_f64] the first 2 samples match the reference (test_vlog_f64 nb=2)") {
	runArm(arm_vlog_f64, "LogInput1_f64.txt", "Log1_f64.txt", 2);
}
TEST_CASE("[C1 arm_vlog_f64] the first 4 samples match the reference (test_vlog_f64 nb=2n)") {
	runArm(arm_vlog_f64, "LogInput1_f64.txt", "Log1_f64.txt", 4);
}
TEST_CASE("[C1 arm_vlog_f64] the first 5 samples match the reference (test_vlog_f64 nb=2n+1)") {
	runArm(arm_vlog_f64, "LogInput1_f64.txt", "Log1_f64.txt", 5);
}
TEST_CASE("[C1 arm_vexp_f64] all 52 samples of ARM's exp pattern match the reference (test_vexp_f64)") {
	runArm(arm_vexp_f64, "ExpInput1_f64.txt", "Exp1_f64.txt", 0);
}
TEST_CASE("[C1 arm_vexp_f64] the first 2 samples match the reference (test_vexp_f64 nb=2)") {
	runArm(arm_vexp_f64, "ExpInput1_f64.txt", "Exp1_f64.txt", 2);
}
TEST_CASE("[C1 arm_vexp_f64] the first 4 samples match the reference (test_vexp_f64 nb=2n)") {
	runArm(arm_vexp_f64, "ExpInput1_f64.txt", "Exp1_f64.txt", 4);
}
TEST_CASE("[C1 arm_vexp_f64] the first 5 samples match the reference (test_vexp_f64 nb=2n+1)") {
	runArm(arm_vexp_f64, "ExpInput1_f64.txt", "Exp1_f64.txt", 5);
}

// ---------------------------------------------------------------------------------------------------------------
// C2: block sizes, guard words
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C2 arm_vlog_f64] block sizes 0, 1, 3, 4, 5, 9 and 16 write exactly the requested samples, correct to 4 ulp") {
	for (size_t n : {0u, 1u, 3u, 4u, 5u, 9u, 16u}) {
		CAPTURE(n);
		runBlock(arm_vlog_f64, defLog, kLogPool, n);
	}
}
TEST_CASE("[C2 arm_vexp_f64] block sizes 0, 1, 3, 4, 5, 9 and 16 write exactly the requested samples, correct to 4 ulp") {
	for (size_t n : {0u, 1u, 3u, 4u, 5u, 9u, 16u}) {
		CAPTURE(n);
		runBlock(arm_vexp_f64, defExp, kExpPool, n);
	}
}
TEST_CASE("[C2 arm_vlog_f64] block size 0 leaves the output untouched") {
	const double in[1] = {2.0};
	dsp::GuardedBuffer<double> out(4);
	arm_vlog_f64(in, out.data(), 0);
	for (double v : out.output()) CHECK(poisoned(v));
	CHECK(out.guardIntact());
}
TEST_CASE("[C2 arm_vexp_f64] block size 0 leaves the output untouched") {
	const double in[1] = {2.0};
	dsp::GuardedBuffer<double> out(4);
	arm_vexp_f64(in, out.data(), 0);
	for (double v : out.output()) CHECK(poisoned(v));
	CHECK(out.guardIntact());
}

// ---------------------------------------------------------------------------------------------------------------
// C4: edge values (DEF, C99 Annex F)
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C2 arm_vexp_f64 arm_vlog_f64] hand-derived constants: exp(1), exp(-1), log(2) and log(10) within 4 ulp") {
	// The C2 block tests compare with std::exp / std::log, the functions the library calls, so their values are
	// self-referential (placement and guard are the point there). These literals are independent (known decimal
	// expansions: e = 2.718281828459045, 1/e = 0.36787944117144233, ln 2 = 0.6931471805599453, ln 10 = 2.302585092994046).
	const double in[] = {1.0, -1.0};
	const double expect[] = {2.718281828459045, 0.36787944117144233};
	dsp::GuardedBuffer<double> o(2);
	arm_vexp_f64(in, o.data(), 2);
	for (int i = 0; i < 2; i++) CHECK(std::fabs(o.output()[i] - expect[i]) <= 4.0 * 2.2e-16 * expect[i]);
	CHECK(o.guardIntact());
	const double in2[] = {2.0, 10.0};
	const double expect2[] = {0.6931471805599453, 2.302585092994046};
	dsp::GuardedBuffer<double> l(2);
	arm_vlog_f64(in2, l.data(), 2);
	for (int i = 0; i < 2; i++) CHECK(std::fabs(l.output()[i] - expect2[i]) <= 4.0 * 2.2e-16 * expect2[i]);
	CHECK(l.guardIntact());
}

TEST_CASE("[C4 arm_vlog_f64] log of +0 is -inf and of -0 is -inf") {
	const double in[2] = {0.0, -0.0};
	double out[2] = {1, 1};
	arm_vlog_f64(in, out, 2);
	CHECK(out[0] == -kInf);
	CHECK(out[1] == -kInf);
}
TEST_CASE("[C4 arm_vlog_f64] log of a negative value and of -inf is NaN, of NaN is NaN") {
	const double in[3] = {-1.0, -kInf, kNaN};
	double out[3] = {0, 0, 0};
	arm_vlog_f64(in, out, 3);
	for (double v : out) CHECK(std::isnan(v));
}
TEST_CASE("[C4 arm_vlog_f64] log of +inf is +inf and log of 1 is exactly 0") {
	const double in[2] = {kInf, 1.0};
	double out[2] = {0, 1};
	arm_vlog_f64(in, out, 2);
	CHECK(out[0] == kInf);
	CHECK(out[1] == 0.0);
}
TEST_CASE("[C4 arm_vlog_f64] log of the smallest denormal and the smallest normal is finite and correct to 4 ulp") {
	// DEF: log(2^-1074) = -1074 ln 2 = -744.4400719213812..., log(2^-1022) = -1022 ln 2 = -708.3964185322641...
	const double in[2] = {std::numeric_limits<double>::denorm_min(), std::numeric_limits<double>::min()};
	double out[2] = {0, 0};
	arm_vlog_f64(in, out, 2);
	CHECK(out[0] == doctest::Approx(-744.4400719213812).epsilon(1e-15));
	CHECK(out[1] == doctest::Approx(-708.3964185322641).epsilon(1e-15));
}
TEST_CASE("[C4 arm_vexp_f64] exp of -inf is 0, of +inf is +inf, of NaN is NaN") {
	const double in[3] = {-kInf, kInf, kNaN};
	double out[3] = {1, 1, 1};
	arm_vexp_f64(in, out, 3);
	CHECK(out[0] == 0.0);
	CHECK(out[1] == kInf);
	CHECK(std::isnan(out[2]));
}
TEST_CASE("[C4 arm_vexp_f64] exp overflows to +inf above ln(DBL_MAX) = 709.78 and is finite just below") {
	const double in[2] = {710.0, 709.0};
	double out[2] = {0, 0};
	arm_vexp_f64(in, out, 2);
	CHECK(out[0] == kInf);
	CHECK(std::isfinite(out[1]));
	CHECK(out[1] == doctest::Approx(8.218407461554972e307).epsilon(1e-15));  // e^709
}
TEST_CASE("[C4 arm_vexp_f64] exp underflows to 0 below -745.14 and exp(0) is exactly 1") {
	const double in[2] = {-746.0, 0.0};
	double out[2] = {1, 0};
	arm_vexp_f64(in, out, 2);
	CHECK(out[0] == 0.0);
	CHECK(out[1] == 1.0);
}

}
