/*
 * FastMath q15 (work order FQ15, requirements C1-C6 of Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md).
 *
 * Units covered: arm_sin_q15, arm_cos_q15, arm_sqrt_q15, arm_divide_q15, arm_atan2_q15, arm_vlog_q15
 * (Source/FastMathFunctions/arm_*_q15.c). Also the ARM id test_recip_q15 (inline arm_recip_q15 in dsp/utils.h:119,
 * called by arm_lms_norm_q15 of the in-scope Filtering module).
 *
 * ARM ids covered (Testing/desc.txt:1906-1946, "Fast Maths Q15"; ids in setUp order, FastMathQ15.cpp:108-216):
 *   test_cos_q15, test_sin_q15, test_sqrt_q15, test_division_q15, test_vlog_q15 (all 125 samples), nb=3 (setUp reloads
 *   7 samples), nb=4n (16 samples), nb=4n+1 (23 samples), test_atan2_scalar_q15, test_recip_q15. None is disabled
 *   or commented out, all ten are run.
 * Pattern files (Testing/Patterns/DSP/FastMath/FastMathQ15/): Angles1 -> Cos1 / Sin1 (23 samples), SqrtInput1 -> Sqrt1
 *   (1024), LogInput1 -> Log1 (125, output q4.11), Numerator1 / Denominator1 -> DivisionValue1 + DivisionShift1_s16
 *   (2704), Atan2Input1 (interleaved y, x; 1001 pairs) -> Atan2Ref1 (Q2.13), RecipInput1 -> RecipRef1 + RecipShift1_s16
 *   (7). Samples1_q15.txt is the benchmark input and not used.
 * ARM tolerances (FastMathQ15.cpp:7-20), used unchanged (L4): SNR_THRESHOLD 69 dB (ASSERT_SNR) for every id;
 *   per sample (ASSERT_NEAR_EQ, in LSB): sqrt 6 (:14), sin and cos 10 (:16), vlog 3 (:18), atan2 3 (:19),
 *   divide 2 (:20), recip 2 (:21); ASSERT_EQ for the shift values (divide, recip); ASSERT_EMPTY_TAIL (vlog).
 *
 * C2: arm_vlog_q15 is the only vector function: GuardedBuffer, block sizes 0, 1, 3, 4, 5, 9, every sample against the
 *   DEF in double (ARM's 3 LSB bound), nothing written for size 0. The scalar functions need no guard test.
 * C3 (DEF, std:: in double):
 *   sin / cos: sin(2*pi*x/32768)*32768 for all 65536 inputs. ARM's 10 LSB is loose, a tighter bound is derived:
 *     the method is a 512 entry table (sinTable_q15, step h = 2*pi/512 = 0.01227 rad) with linear interpolation.
 *     Linear interpolation error <= h^2/8 * max|f''| = 1.88e-5 = 0.62 LSB (1 LSB = 2^-15); the table is rounded to q15
 *     (<= 0.5 LSB; the interpolation weights sum to 1, so this stays <= 0.5 LSB); the two products are shifted right by
 *     16 (arm_sin_q15.c:79,80), the results are in q14 and the last statement shifts them left by one: each of the two
 *     truncations loses < 1 q14 unit = 2 LSB, so < 4 LSB. Total < 0.62 + 0.5 + 4 = 5.12 LSB, bound 5.2 LSB.
 *   sqrt: round(sqrt(x/32768)*32768) for x = 1..32767 (ARM's reference is rounded, np.round), ARM's 6 LSB (the Newton
 *     iteration over a 12 entry LUT is not derived).
 *   divide: the quotient is temp = floor(|a| * 2^15 / |b|) (integer division, truncation), then temp >> shift, then the
 *     sign. floor(floor(e) / 2^s) = floor(e / 2^s), so the normalised quotient is the exact value truncated toward
 *     zero: error < 1 LSB (tighter than ARM's 2). The shift is the one that brings |a/b| into [0,1): 0 if |a/b| < 1,
 *     else floor(log2(|a/b|)) + 1 (temp has 16 + k bits for |a/b| in [2^k, 2^(k+1)), shift = bits - 15 = k + 1).
 *     Operands -32768 are left out of the sweep (arm_abs_q15 saturates them to 32767, a separate test).
 *   atan2: atan2(y, x) * 8192 (Q2.13: pi = 25735.9) on a grid of y and x values, ARM's 3 LSB (the polynomial
 *     of 10 coefficients is not derived).
 *   vlog: log(x/32768) / 16 * 32768 = log(x/32768) * 2048 for x = 1..32767 (output q4.11), ARM's 3 LSB.
 * C4: DOC quotes are in the tests. DOC is silent on: sin/cos of the range ends (taken from the DOC "mapped to
 *   [0 2*pi)" and DEF), vlog of 0 (not asserted, see the report), atan2(0,0) (characterization).
 * C5: q15 against the f32 function of the same input after conversion: arm_sin_f32 / arm_cos_f32 (the same table
 *   method, 512 entries, error 0.62 LSB + f32 rounding ~0.01), arm_sqrt_f32 (exact to 0.5 ulp), arm_atan2_f32 (ARM
 *   bound 5e-7 rad = 0.004 LSB), arm_vlog_f32 (f32 accuracy ~1e-6 absolute = 0.002 LSB of q4.11), division in float.
 *   Bounds: q15 bound against the DEF plus the f32 error: sin/cos 5.12 + 0.62 + 0.01 -> 6.0 (the two errors may both
 *   be as large as stated, so 5.74 is the worst case difference), sqrt 6.6 (ARM's 6 LSB is against a reference rounded to q15, so up to 6.5 against the exact value, plus f32 0.004), atan2 3.1, vlog 3.1, divide 1.1.
 *
 * Host vs target (C6): the host runs the portable C path (no ARM_MATH_DSP, __CLZ from dsp/none.h); the target uses
 * the same C code here (these q15 functions have no DSP intrinsics except through __CLZ / __SSAT /__QSUB16 in
 * atan2). Signed left shifts of negative values (arm_sin_q15.c:84 sinVal << 1, :80) are UB in C but defined by
 * GCC / clang; host long is 64 bit but only q31_t (int) arithmetic is used. The results are bit exact on the
 * target for the portable path, but the target's __QSUB16 path in arm_atan_q15 is not exercised here.
 */
#include "doctest.h"

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math.h"
#include "arm_common_tables.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

// ARM's tolerances (FastMathQ15.cpp:7-20)
constexpr double kSnrThreshold = 69.0;
constexpr int64_t kSqrtLsb = 6;
constexpr int64_t kSinCosLsb = 10;
constexpr int64_t kLogLsb = 3;
constexpr int64_t kAtanLsb = 3;
constexpr int64_t kDivLsb = 2;
constexpr int64_t kRecipLsb = 2;

// Derived bounds (see the header comment)
constexpr double kSinCosDefLsb = 5.2;

const double kPi = 3.14159265358979323846;

std::vector<int16_t> q15(const std::string &file) {
	return dsp::loadPattern("FastMath/FastMathQ15/" + file).q15();
}

// DEF functions in double (results in LSB of the output format)
double defSin(int x) { return std::sin(2.0 * kPi * x / 32768.0) * 32768.0; }
double defCos(int x) { return std::cos(2.0 * kPi * x / 32768.0) * 32768.0; }
double defSqrt(int x) { return std::sqrt(x / 32768.0) * 32768.0; }
double defLog(int x) { return std::log(x / 32768.0) * 2048.0; }          // q4.11
double defAtan2(int y, int x) { return std::atan2((double)y, (double)x) * 8192.0; }  // Q2.13

// Expected shift of arm_divide_q15 for |a| < 32768 and |b| < 32768 (derivation in the header)
int defShift(double r) {
	r = std::fabs(r);
	if (r < 1.0) return 0;
	return (int)std::floor(std::log2(r)) + 1;
}

// ARM's vlog test on the first nb samples (0 = all) of the pattern
void runVlogArm(size_t nb) {
	std::vector<int16_t> in = q15("LogInput1_q15.txt");
	std::vector<int16_t> ref = q15("Log1_q15.txt");
	REQUIRE(in.size() == ref.size());
	if (nb) {
		REQUIRE(in.size() >= nb);
		in.resize(nb);
		ref.resize(nb);
	}
	dsp::GuardedBuffer<q15_t> out(ref.size());
	arm_vlog_q15(in.data(), out.data(), (uint32_t)in.size());
	std::vector<int16_t> o = out.output();
	CHECK(dsp::snr(ref, o) >= kSnrThreshold);
	auto c = dsp::withinLsb(ref, o, kLogLsb);
	CHECK_MESSAGE(c, c.message);
	CHECK(out.guardIntact());
}

// Fixed inputs for the C2 tests (distinct, inside (0, 1))
const q15_t kLogIn[9] = {1, 100, 2048, 16384, 32767, 5, 777, 12345, 30000};

void checkAtan(int y, int x, double lsb) {
	q15_t res = 0x1234;
	arm_status st = arm_atan2_q15((q15_t)y, (q15_t)x, &res);
	CHECK(st == ARM_MATH_SUCCESS);
	CHECK_MESSAGE(std::fabs(res - defAtan2(y, x)) <= lsb,
	              "y=" << y << " x=" << x << " res=" << res << " DEF=" << defAtan2(y, x));
}

}  // namespace

TEST_SUITE("DSP FastMath q15") {

// ------------------------------------------------------------------------------------------------------------------
// C1: ARM's ids
// ------------------------------------------------------------------------------------------------------------------

TEST_CASE("[C1 arm_cos_q15] the cosine of ARM's 23 angles has SNR >= 69 dB and is within 10 LSB of the reference" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFQ15-2: arm_sin_q15.c:78 and arm_cos_q15.c:79 shift a negative value left "
			  "('(q31_t) sinVal << 16') and :81 / :82 'sinVal << 1' for every input whose result is negative: left shift of a "
			  "negative value is undefined behaviour in C (UBSan: 'left shift of negative value'). The intended result occurs "
			  "on the host; skipped in sanitizer builds")) {
	std::vector<int16_t> in = q15("Angles1_q15.txt"), ref = q15("Cos1_q15.txt");
	REQUIRE(in.size() == ref.size());
	std::vector<int16_t> out;
	for (int16_t a : in) out.push_back(arm_cos_q15(a));
	CHECK(dsp::snr(ref, out) >= kSnrThreshold);
	auto c = dsp::withinLsb(ref, out, kSinCosLsb);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C1 arm_sin_q15] the sine of ARM's 23 angles has SNR >= 69 dB and is within 10 LSB of the reference" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFQ15-2: arm_sin_q15.c:78 and arm_cos_q15.c:79 shift a negative value left "
			  "('(q31_t) sinVal << 16') and :81 / :82 'sinVal << 1' for every input whose result is negative: left shift of a "
			  "negative value is undefined behaviour in C (UBSan: 'left shift of negative value'). The intended result occurs "
			  "on the host; skipped in sanitizer builds")) {
	std::vector<int16_t> in = q15("Angles1_q15.txt"), ref = q15("Sin1_q15.txt");
	REQUIRE(in.size() == ref.size());
	std::vector<int16_t> out;
	for (int16_t a : in) out.push_back(arm_sin_q15(a));
	CHECK(dsp::snr(ref, out) >= kSnrThreshold);
	auto c = dsp::withinLsb(ref, out, kSinCosLsb);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C1 arm_sqrt_q15] the square root of ARM's 1024 inputs has SNR >= 69 dB, is within 6 LSB and the status is as ARM expects") {
	std::vector<int16_t> in = q15("SqrtInput1_q15.txt"), ref = q15("Sqrt1_q15.txt");
	REQUIRE(in.size() == ref.size());
	std::vector<int16_t> out(in.size(), 0);
	bool statusOk = true;
	for (size_t i = 0; i < in.size(); i++) {
		arm_status st = arm_sqrt_q15(in[i], &out[i]);
		// FastMathQ15.cpp:100: success, or (input <= 0 and argument error)
		statusOk = statusOk && (st == ARM_MATH_SUCCESS || (in[i] <= 0 && st == ARM_MATH_ARGUMENT_ERROR));
	}
	CHECK(statusOk);
	CHECK(dsp::snr(ref, out) >= kSnrThreshold);
	auto c = dsp::withinLsb(ref, out, kSqrtLsb);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C1 arm_divide_q15] the quotient of ARM's 2704 pairs has SNR >= 69 dB, is within 2 LSB and the shift is exact") {
	std::vector<int16_t> num = q15("Numerator1_q15.txt"), den = q15("Denominator1_q15.txt");
	std::vector<int16_t> ref = q15("DivisionValue1_q15.txt"), refShift = q15("DivisionShift1_s16.txt");
	REQUIRE(num.size() == den.size());
	REQUIRE(num.size() == ref.size());
	REQUIRE(num.size() == refShift.size());
	std::vector<int16_t> out(num.size(), 0), shift(num.size(), 0);
	for (size_t i = 0; i < num.size(); i++) arm_divide_q15(num[i], den[i], &out[i], &shift[i]);
	CHECK(dsp::snr(ref, out) >= kSnrThreshold);
	auto c = dsp::withinLsb(ref, out, kDivLsb);
	CHECK_MESSAGE(c, c.message);
	auto s = dsp::exactEqual(refShift, shift);
	CHECK_MESSAGE(s, s.message);
}

TEST_CASE("[C1 arm_vlog_q15] the log of ARM's 125 inputs has SNR >= 69 dB, is within 3 LSB and leaves the guard intact") {
	runVlogArm(0);
}

TEST_CASE("[C1 arm_vlog_q15] the log of ARM's first 7 inputs (nb=3 case) has SNR >= 69 dB and is within 3 LSB") {
	runVlogArm(7);
}

TEST_CASE("[C1 arm_vlog_q15] the log of ARM's first 16 inputs (nb=4n case) has SNR >= 69 dB and is within 3 LSB") {
	runVlogArm(16);
}

TEST_CASE("[C1 arm_vlog_q15] the log of ARM's first 23 inputs (nb=4n+1 case) has SNR >= 69 dB and is within 3 LSB") {
	runVlogArm(23);
}

TEST_CASE("[C1 arm_atan2_q15] the arc tangent of ARM's 1001 (y, x) pairs has SNR >= 69 dB, is within 3 LSB and succeeds") {
	std::vector<int16_t> in = q15("Atan2Input1_q15.txt"), ref = q15("Atan2Ref1_q15.txt");
	REQUIRE(in.size() == 2 * ref.size());
	std::vector<int16_t> out(ref.size(), 0);
	bool statusOk = true;
	for (size_t i = 0; i < ref.size(); i++) {
		// FastMathQ15.cpp:57: arm_atan2_q15(inp[2*i], inp[2*i+1], &res): y first, then x
		q15_t res = 0;
		statusOk = statusOk && (arm_atan2_q15(in[2 * i], in[2 * i + 1], &res) == ARM_MATH_SUCCESS);
		out[i] = res;
	}
	CHECK(statusOk);
	CHECK(dsp::snr(ref, out) >= kSnrThreshold);
	auto c = dsp::withinLsb(ref, out, kAtanLsb);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C1 arm_recip_q15] the reciprocal of ARM's 7 inputs has SNR >= 69 dB, is within 2 LSB and the shift is exact") {
	// arm_recip_q15 (dsp/utils.h:119) is an inline helper of the in-scope arm_lms_norm_q15
	std::vector<int16_t> in = q15("RecipInput1_q15.txt"), ref = q15("RecipRef1_q15.txt");
	std::vector<int16_t> refShift = q15("RecipShift1_s16.txt");
	REQUIRE(in.size() == ref.size());
	REQUIRE(in.size() == refShift.size());
	std::vector<int16_t> out(in.size(), 0), shift(in.size(), 0);
	for (size_t i = 0; i < in.size(); i++) shift[i] = (int16_t)arm_recip_q15(in[i], &out[i], armRecipTableQ15);
	CHECK(dsp::snr(ref, out) >= kSnrThreshold);
	auto c = dsp::withinLsb(ref, out, kRecipLsb);
	CHECK_MESSAGE(c, c.message);
	auto s = dsp::exactEqual(refShift, shift);
	CHECK_MESSAGE(s, s.message);
}

// ------------------------------------------------------------------------------------------------------------------
// C2: arm_vlog_q15 block sizes
// ------------------------------------------------------------------------------------------------------------------

TEST_CASE("[C2 arm_vlog_q15] every written sample is correct and the guard is intact for block sizes 0, 1, 3, 4, 5 and 9") {
	const uint32_t sizes[] = {0, 1, 3, 4, 5, 9};
	for (uint32_t n : sizes) {
		CAPTURE(n);
		dsp::GuardedBuffer<q15_t> out(n);
		arm_vlog_q15(kLogIn, out.data(), n);
		std::vector<double> ref;
		for (uint32_t i = 0; i < n; i++) ref.push_back(defLog(kLogIn[i]));
		auto c = dsp::closeAbsRel(ref, out.output(), (double)kLogLsb, 0.0);
		CHECK_MESSAGE(c, c.message);
		CHECK(out.guardIntact());
	}
}

TEST_CASE("[C2 arm_vlog_q15] a block of size 0 writes nothing") {
	dsp::GuardedBuffer<q15_t> out(4);
	q15_t *p = out.data();
	for (int i = 0; i < 4; i++) p[i] = 0x1111;
	arm_vlog_q15(kLogIn, p, 0);
	CHECK(out.output() == std::vector<q15_t>(4, 0x1111));
	CHECK(out.guardIntact());
}

// ------------------------------------------------------------------------------------------------------------------
// C3: DEF sweeps
// ------------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_sin_q15] for all 65536 inputs the sine is within the derived 5.2 LSB of sin(2*pi*x)" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFQ15-2: arm_sin_q15.c:78 and arm_cos_q15.c:79 shift a negative value left "
			  "('(q31_t) sinVal << 16') and :81 / :82 'sinVal << 1' for every input whose result is negative: left shift of a "
			  "negative value is undefined behaviour in C (UBSan: 'left shift of negative value'). The intended result occurs "
			  "on the host; skipped in sanitizer builds")) {
	std::vector<double> ref;
	std::vector<int16_t> out;
	for (int x = -32768; x <= 32767; x++) {
		ref.push_back(defSin(x));
		out.push_back(arm_sin_q15((q15_t)x));
	}
	auto c = dsp::closeAbsRel(ref, out, kSinCosDefLsb, 0.0);
	CHECK_MESSAGE(c, c.message << " (x=" << (int)(c.index) - 32768 << ")");
}

TEST_CASE("[C3 arm_cos_q15] for all 65536 inputs the cosine is within the derived 5.2 LSB of cos(2*pi*x)" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFQ15-2: arm_sin_q15.c:78 and arm_cos_q15.c:79 shift a negative value left "
			  "('(q31_t) sinVal << 16') and :81 / :82 'sinVal << 1' for every input whose result is negative: left shift of a "
			  "negative value is undefined behaviour in C (UBSan: 'left shift of negative value'). The intended result occurs "
			  "on the host; skipped in sanitizer builds")) {
	std::vector<double> ref;
	std::vector<int16_t> out;
	for (int x = -32768; x <= 32767; x++) {
		ref.push_back(defCos(x));
		out.push_back(arm_cos_q15((q15_t)x));
	}
	auto c = dsp::closeAbsRel(ref, out, kSinCosDefLsb, 0.0);
	CHECK_MESSAGE(c, c.message << " (x=" << (int)(c.index) - 32768 << ")");
}

TEST_CASE("[C3 arm_sqrt_q15] for x = 1..32767 the square root is within ARM's 6 LSB of the rounded sqrt(x)" *
		  KNOWN_ISSUE("B-DFQ15-1: 6 of the 32767 inputs exceed ARM's own bound of 6 LSB (ABS_SQRT_ERROR, FastMathQ15.cpp:14) "
			  "against the rounded double sqrt (ARM's reference is np.round of it), e.g. x=30840 gives 31796 (reference 31789, "
			  "error 7); the others are x=31373, 31782, 31796, 31797, 31812. ARM's 1024 point pattern does not contain them")) {
	std::vector<double> ref;
	std::vector<int16_t> out;
	for (int x = 1; x <= 32767; x++) {
		q15_t r = 0x1234;
		REQUIRE(arm_sqrt_q15((q15_t)x, &r) == ARM_MATH_SUCCESS);
		ref.push_back(std::nearbyint(defSqrt(x)));  // ARM's reference is rounded to the nearest q15 (np.round)
		out.push_back(r);
	}
	auto c = dsp::closeAbsRel(ref, out, (double)kSqrtLsb, 0.0);
	CHECK_MESSAGE(c, c.message << " (x=" << c.index + 1 << ")");
}

TEST_CASE("[C3 arm_divide_q15] the normalised quotient is the exact quotient truncated toward zero (error < 1 LSB) and the shift is exact") {
	const int vals[] = {0, 1, 2, 3, 7, 100, 255, 1000, 8191, 8192, 16383, 16384, 20000, 32766, 32767};
	size_t cases = 0;
	for (int a0 : vals)
		for (int b0 : vals) {
			if (b0 == 0) continue;
			for (int sa = -1; sa <= 1; sa += 2)
				for (int sb = -1; sb <= 1; sb += 2) {
					int a = sa * a0, b = sb * b0;
					q15_t q = 0x1234;
					int16_t sh = 0x77;
					REQUIRE(arm_divide_q15((q15_t)a, (q15_t)b, &q, &sh) == ARM_MATH_SUCCESS);
					double r = (double)a / (double)b;
					int es = (a == 0) ? 0 : defShift(r);
					double expect = r / std::ldexp(1.0, es) * 32768.0;
					CAPTURE(a);
					CAPTURE(b);
					CHECK(sh == es);
					CHECK(std::fabs(q - expect) <= 1.0);
					cases++;
				}
		}
	CHECK(cases > 800);
}

TEST_CASE("[C3 arm_atan2_q15] on a grid of (y, x) values the angle in Q2.13 is within ARM's 3 LSB of atan2") {
	const int vals[] = {-32768, -20000, -1000, -100, -3, -1, 0, 1, 3, 100, 1000, 20000, 32767};
	for (int y : vals)
		for (int x : vals) {
			if (x == 0 && y == 0) continue;  // undefined, see the C4 test
			checkAtan(y, x, (double)kAtanLsb);
		}
}

TEST_CASE("[C3 arm_vlog_q15] for x = 1..32767 the log in q4.11 is within ARM's 3 LSB of log(x/32768)") {
	std::vector<int16_t> in;
	std::vector<double> ref;
	for (int x = 1; x <= 32767; x++) {
		in.push_back((int16_t)x);
		ref.push_back(defLog(x));
	}
	dsp::GuardedBuffer<q15_t> out(in.size());
	arm_vlog_q15(in.data(), out.data(), (uint32_t)in.size());
	auto c = dsp::closeAbsRel(ref, out.output(), (double)kLogLsb, 0.0);
	CHECK_MESSAGE(c, c.message << " (x=" << c.index + 1 << ")");
	CHECK(out.guardIntact());
}

// ------------------------------------------------------------------------------------------------------------------
// C4: documented edge behaviour
// ------------------------------------------------------------------------------------------------------------------

TEST_CASE("[C4 arm_sin_q15] the range ends 0, 0x7FFF and the most negative value follow sin(2*pi*x)" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFQ15-2: arm_sin_q15.c:78 and arm_cos_q15.c:79 shift a negative value left "
			  "('(q31_t) sinVal << 16') and :81 / :82 'sinVal << 1' for every input whose result is negative: left shift of a "
			  "negative value is undefined behaviour in C (UBSan: 'left shift of negative value'). The intended result occurs "
			  "on the host; skipped in sanitizer builds")) {
	// DOC (arm_sin_q15.c): "The Q15 input value is in the range [0 +0.9999] and is mapped to a radian value in the
	// range [0 2*PI)". 0 -> 0; 0x7FFF = 2*pi - 1 LSB-of-angle -> -1.92e-4 * 32768 = -6.3; 0x8000 = -1.0 = -2*pi -> 0
	CHECK(std::fabs(arm_sin_q15(0) - 0.0) <= kSinCosDefLsb);
	CHECK(std::fabs(arm_sin_q15(0x7FFF) - defSin(0x7FFF)) <= kSinCosDefLsb);
	CHECK(std::fabs(arm_sin_q15((q15_t)0x8000) - 0.0) <= kSinCosDefLsb);
}

TEST_CASE("[C4 arm_sin_q15] the quarter turns give 0, +1, 0, -1 within the derived bound" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFQ15-2: arm_sin_q15.c:78 and arm_cos_q15.c:79 shift a negative value left "
			  "('(q31_t) sinVal << 16') and :81 / :82 'sinVal << 1' for every input whose result is negative: left shift of a "
			  "negative value is undefined behaviour in C (UBSan: 'left shift of negative value'). The intended result occurs "
			  "on the host; skipped in sanitizer builds")) {
	// x = 0, 0x2000, 0x4000, 0x6000 are 0, pi/2, pi, 3*pi/2
	CHECK(std::fabs(arm_sin_q15(0x2000) - 32768.0) <= kSinCosDefLsb);
	CHECK(std::fabs(arm_sin_q15(0x4000) - 0.0) <= kSinCosDefLsb);
	CHECK(std::fabs(arm_sin_q15(0x6000) + 32768.0) <= kSinCosDefLsb);
}

TEST_CASE("[C4 arm_cos_q15] the range ends 0, 0x7FFF and the most negative value follow cos(2*pi*x)") {
	// DOC as for the sine. cos(0) = 1 (32768 is not representable: 32767), cos(0x7FFF) = 1 - 1.8e-8, cos(-1.0) = cos(-2*pi) = 1
	CHECK(std::fabs(arm_cos_q15(0) - 32768.0) <= kSinCosDefLsb);
	CHECK(std::fabs(arm_cos_q15(0x7FFF) - 32768.0) <= kSinCosDefLsb);
	CHECK(std::fabs(arm_cos_q15((q15_t)0x8000) - 32768.0) <= kSinCosDefLsb);
}

TEST_CASE("[C4 arm_cos_q15] the quarter turns give +1, 0, -1, 0 within the derived bound" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFQ15-2: arm_sin_q15.c:78 and arm_cos_q15.c:79 shift a negative value left "
			  "('(q31_t) sinVal << 16') and :81 / :82 'sinVal << 1' for every input whose result is negative: left shift of a "
			  "negative value is undefined behaviour in C (UBSan: 'left shift of negative value'). The intended result occurs "
			  "on the host; skipped in sanitizer builds")) {
	CHECK(std::fabs(arm_cos_q15(0x2000) - 0.0) <= kSinCosDefLsb);
	CHECK(std::fabs(arm_cos_q15(0x4000) + 32768.0) <= kSinCosDefLsb);
	CHECK(std::fabs(arm_cos_q15(0x6000) - 0.0) <= kSinCosDefLsb);
}

TEST_CASE("[C4 arm_sqrt_q15] a negative input returns ARM_MATH_ARGUMENT_ERROR and sets the output to 0") {
	// DOC (fast_math_functions.h:299): "ARM_MATH_ARGUMENT_ERROR : input value is negative; *pOut is set to 0"
	const int vals[] = {-1, -2, -100, -16384, -32767, -32768};
	for (int v : vals) {
		CAPTURE(v);
		q15_t out = 0x1234;
		CHECK(arm_sqrt_q15((q15_t)v, &out) == ARM_MATH_ARGUMENT_ERROR);
		CHECK(out == 0);
	}
}

TEST_CASE("[C4 arm_sqrt_q15] the square root of 0 is 0 with status success") {
	// DOC: argument error only for a negative input; 0 is not negative, the root of 0 is 0 (DEF)
	q15_t out = 0x1234;
	CHECK(arm_sqrt_q15(0, &out) == ARM_MATH_SUCCESS);
	CHECK(out == 0);
}

TEST_CASE("[C4 arm_sqrt_q15] the smallest and the largest input give sqrt(1/32768) and sqrt(0.99997)") {
	// DEF: sqrt(1/32768) = 0.005524 -> 181.02 LSB; sqrt(32767/32768) = 0.99998 -> 32767.5 LSB
	q15_t lo = 0, hi = 0;
	CHECK(arm_sqrt_q15(1, &lo) == ARM_MATH_SUCCESS);
	CHECK(arm_sqrt_q15(0x7FFF, &hi) == ARM_MATH_SUCCESS);
	CHECK(std::fabs(lo - 181.02) <= (double)kSqrtLsb);
	CHECK(std::fabs(hi - 32767.5) <= (double)kSqrtLsb);
}

TEST_CASE("[C4 arm_divide_q15] division by zero returns ARM_MATH_NANINF and the saturated value by the sign of the numerator") {
	// DOC (fast_math_functions.h:319): "When dividing by 0, an error ARM_MATH_NANINF is returned. And the quotient is
	// forced to the saturated negative or positive value."
	q15_t q = 0;
	int16_t sh = 0;
	CHECK(arm_divide_q15(1000, 0, &q, &sh) == ARM_MATH_NANINF);
	CHECK(q == 32767);
	CHECK(arm_divide_q15(-1000, 0, &q, &sh) == ARM_MATH_NANINF);
	CHECK(q == -32768);
	CHECK(arm_divide_q15((q15_t)-32768, 0, &q, &sh) == ARM_MATH_NANINF);
	CHECK(q == -32768);
	CHECK(arm_divide_q15(32767, 0, &q, &sh) == ARM_MATH_NANINF);
	CHECK(q == 32767);
}

TEST_CASE("[C4 arm_divide_q15] a numerator of 0 gives quotient 0 and shift 0") {
	// DEF: 0 / b = 0 for any b != 0
	q15_t q = 0x1234;
	int16_t sh = 0x77;
	CHECK(arm_divide_q15(0, 12345, &q, &sh) == ARM_MATH_SUCCESS);
	CHECK(q == 0);
	CHECK(sh == 0);
	CHECK(arm_divide_q15(0, -12345, &q, &sh) == ARM_MATH_SUCCESS);
	CHECK(q == 0);
	CHECK(sh == 0);
}

TEST_CASE("[C4 arm_divide_q15] a quotient of exactly 1.0 or -1.0 is returned as 0.5 or -0.5 with shift 1") {
	// DEF: the normalised quotient is < 1 in magnitude, 1.0 = 0.5 * 2^1; 16384/16384: temp = 32768 (16 bits) -> shift 1
	q15_t q = 0;
	int16_t sh = 0;
	CHECK(arm_divide_q15(16384, 16384, &q, &sh) == ARM_MATH_SUCCESS);
	CHECK(q == 16384);
	CHECK(sh == 1);
	CHECK(arm_divide_q15(-16384, 16384, &q, &sh) == ARM_MATH_SUCCESS);
	CHECK(q == -16384);
	CHECK(sh == 1);
}

TEST_CASE("[C4 arm_divide_q15] the operand -1.0 (0x8000) is saturated to 0x7FFF by arm_abs_q15 before the division") {
	// DOC (arm_abs_q15.c): "The Q15 value -1 (0x8000) will be saturated to the maximum allowable positive value 0x7FFF".
	// -32768 / 16384 is computed as -32767 / 16384 = -1.99994: temp = 65534 (16 bits) -> shift 1, quotient -32767 =
	// -0.99997; value = -32767 * 2 / 32768 = -1.99994 (the true -2.0 is 3e-5 away)
	q15_t q = 0;
	int16_t sh = 0;
	CHECK(arm_divide_q15((q15_t)-32768, 16384, &q, &sh) == ARM_MATH_SUCCESS);
	CHECK(sh == 1);
	CHECK(q == -32767);
	CHECK(std::fabs(q * std::ldexp(1.0, sh) / 32768.0 + 2.0) <= 1e-4);
}

TEST_CASE("[C4 arm_atan2_q15] the four axes give 0, pi/2, pi and -pi/2 in Q2.13 with status success") {
	// DEF: pi = 25735.93, pi/2 = 12867.96 in Q2.13 (1 LSB = 1/8192 rad)
	checkAtan(0, 1000, kAtanLsb);     // +x axis -> 0
	checkAtan(1000, 0, kAtanLsb);     // +y axis -> pi/2
	checkAtan(0, -1000, kAtanLsb);    // -x axis -> pi
	checkAtan(-1000, 0, kAtanLsb);    // -y axis -> -pi/2
}

TEST_CASE("[C4 arm_atan2_q15] the four quadrants at 45 degrees give pi/4, 3*pi/4, -3*pi/4 and -pi/4") {
	// DEF: pi/4 = 6433.98, 3*pi/4 = 19301.95 in Q2.13
	checkAtan(16384, 16384, kAtanLsb);
	checkAtan(16384, -16384, kAtanLsb);
	checkAtan(-16384, -16384, kAtanLsb);
	checkAtan(-16384, 16384, kAtanLsb);
}

TEST_CASE("[C4 arm_atan2_q15] the most negative operands (-1.0) are handled as in the DEF within the bound") {
	checkAtan(-32768, 32767, kAtanLsb);
	checkAtan(32767, -32768, kAtanLsb);
	checkAtan(-32768, -32768, kAtanLsb);
	checkAtan(-32768, 0, kAtanLsb);
	checkAtan(0, -32768, kAtanLsb);
}

TEST_CASE("[C4 arm_atan2_q15] characterization: atan2(0, 0) returns ARM_MATH_NANINF") {
	// DOC (fast_math_functions.h:366) only says "@return error status". atan2(0,0) is undefined; the code (arm_atan2_q15.c
	// end of the function) returns ARM_MATH_NANINF without writing the result. NEEDS REVIEW: confirm the intended status.
	q15_t res = 0x1234;
	CHECK(arm_atan2_q15(0, 0, &res) == ARM_MATH_NANINF);
}

TEST_CASE("[C4 arm_vlog_q15] the smallest input (1/32768) and the largest (0x7FFF) follow the log within 3 LSB") {
	// DEF: log(2^-15) = -10.3972 -> -21294 in q4.11 (x 2048); log(32767/32768) = -3.05e-5 -> -0.06
	q15_t in[2] = {1, 0x7FFF};
	q15_t out[2] = {0x1234, 0x1234};
	arm_vlog_q15(in, out, 2);
	CHECK(std::fabs(out[0] - defLog(1)) <= (double)kLogLsb);
	CHECK(std::fabs(out[1] - defLog(0x7FFF)) <= (double)kLogLsb);
}

TEST_CASE("[C4 arm_vlog_q15] the log of 0.5 is -ln(2) in q4.11 within 3 LSB") {
	// DEF: log(0.5) = -0.693147 -> -1419.5 LSB of q4.11 (x 2048)
	q15_t in = 16384, out = 0x1234;
	arm_vlog_q15(&in, &out, 1);
	CHECK(std::fabs(out + 1419.5) <= (double)kLogLsb);
}

// ------------------------------------------------------------------------------------------------------------------
// C5: agreement with the f32 functions of the same input
// ------------------------------------------------------------------------------------------------------------------

TEST_CASE("[C5 arm_sin_q15] the q15 sine agrees with arm_sin_f32 of the converted angle within 6 LSB" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFQ15-2: arm_sin_q15.c:78 and arm_cos_q15.c:79 shift a negative value left "
			  "('(q31_t) sinVal << 16') and :81 / :82 'sinVal << 1' for every input whose result is negative: left shift of a "
			  "negative value is undefined behaviour in C (UBSan: 'left shift of negative value'). The intended result occurs "
			  "on the host; skipped in sanitizer builds")) {
	for (int x = -32768; x <= 32767; x += 7) {
		float angle = (float)(2.0 * kPi * x / 32768.0);
		double f = (double)arm_sin_f32(angle) * 32768.0;
		CAPTURE(x);
		REQUIRE(std::fabs(arm_sin_q15((q15_t)x) - f) <= 6.0);
	}
}

TEST_CASE("[C5 arm_cos_q15] the q15 cosine agrees with arm_cos_f32 of the converted angle within 6 LSB" *
		  KNOWN_ISSUE_UB_HIDDEN("B-DFQ15-2: arm_sin_q15.c:78 and arm_cos_q15.c:79 shift a negative value left "
			  "('(q31_t) sinVal << 16') and :81 / :82 'sinVal << 1' for every input whose result is negative: left shift of a "
			  "negative value is undefined behaviour in C (UBSan: 'left shift of negative value'). The intended result occurs "
			  "on the host; skipped in sanitizer builds")) {
	for (int x = -32768; x <= 32767; x += 7) {
		float angle = (float)(2.0 * kPi * x / 32768.0);
		double f = (double)arm_cos_f32(angle) * 32768.0;
		CAPTURE(x);
		REQUIRE(std::fabs(arm_cos_q15((q15_t)x) - f) <= 6.0);
	}
}

TEST_CASE("[C5 arm_sqrt_q15] the q15 square root agrees with arm_sqrt_f32 of the converted input within 6.6 LSB on ARM's inputs") {
	// ARM's 1024 inputs (the full range exceeds the bound, see B-DFQ15-1 in the C3 test)
	std::vector<int16_t> in = q15("SqrtInput1_q15.txt");
	for (int16_t x : in) {
		if (x <= 0) continue;
		float rf = 0.0f;
		REQUIRE(arm_sqrt_f32((float)x / 32768.0f, &rf) == ARM_MATH_SUCCESS);
		q15_t r = 0;
		REQUIRE(arm_sqrt_q15(x, &r) == ARM_MATH_SUCCESS);
		CAPTURE(x);
		REQUIRE(std::fabs(r - (double)rf * 32768.0) <= 6.6);  // 6 LSB against ARM's rounded reference = up to 6.5 against the exact value
	}
}

TEST_CASE("[C5 arm_atan2_q15] the q15 arc tangent agrees with arm_atan2_f32 of the converted inputs within 3.1 LSB") {
	const int vals[] = {-32768, -20000, -1000, -100, -3, -1, 0, 1, 3, 100, 1000, 20000, 32767};
	for (int y : vals)
		for (int x : vals) {
			if (x == 0 && y == 0) continue;
			float rf = 0.0f;
			REQUIRE(arm_atan2_f32((float)y / 32768.0f, (float)x / 32768.0f, &rf) == ARM_MATH_SUCCESS);
			q15_t r = 0;
			REQUIRE(arm_atan2_q15((q15_t)y, (q15_t)x, &r) == ARM_MATH_SUCCESS);
			CAPTURE(y);
			CAPTURE(x);
			REQUIRE(std::fabs(r - (double)rf * 8192.0) <= 3.1);
		}
}

TEST_CASE("[C5 arm_vlog_q15] the q15 log agrees with arm_vlog_f32 of the converted input within 3.1 LSB of q4.11") {
	std::vector<q15_t> in;
	std::vector<float> inf;
	for (int x = 1; x <= 32767; x += 5) {
		in.push_back((q15_t)x);
		inf.push_back((float)x / 32768.0f);
	}
	std::vector<q15_t> out(in.size());
	std::vector<float> outf(in.size());
	arm_vlog_q15(in.data(), out.data(), (uint32_t)in.size());
	arm_vlog_f32(inf.data(), outf.data(), (uint32_t)in.size());
	std::vector<double> ref;
	for (float v : outf) ref.push_back((double)v * 2048.0);
	auto c = dsp::closeAbsRel(ref, out, 3.1, 0.0);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C5 arm_divide_q15] the q15 quotient times 2^shift agrees with the float division within 1.1 LSB") {
	const int vals[] = {1, 3, 100, 1000, 8192, 16384, 32767, -2, -999, -20000};
	for (int a : vals)
		for (int b : vals) {
			q15_t q = 0;
			int16_t sh = 0;
			REQUIRE(arm_divide_q15((q15_t)a, (q15_t)b, &q, &sh) == ARM_MATH_SUCCESS);
			float f = ((float)a / 32768.0f) / ((float)b / 32768.0f);
			double fn = (double)f / std::ldexp(1.0, sh) * 32768.0;  // normalised quotient in LSB
			CAPTURE(a);
			CAPTURE(b);
			REQUIRE(std::fabs(q - fn) <= 1.1);
		}
}

}
