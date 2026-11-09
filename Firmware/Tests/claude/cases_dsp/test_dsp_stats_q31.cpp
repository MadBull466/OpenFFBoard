/*
 * CMSIS-DSP Statistics, q31 (phase P11 group G3, work order SQ31), plus the FastMath Q63 helpers
 * arm_norm_64_to_32u / arm_div_int64_to_int32 (lead decision L7). Requirement ids: Tests/claude/charters/dsp/
 * CMSIS_DSP_REQUIREMENTS.md.
 *
 * Units (Libraries/CMSIS-DSP/Source/StatisticsFunctions/arm_*_q31.c): arm_max_q31, arm_min_q31, arm_absmax_q31,
 * arm_absmin_q31, arm_max_no_idx_q31, arm_min_no_idx_q31, arm_absmax_no_idx_q31, arm_absmin_no_idx_q31, arm_mean_q31,
 * arm_power_q31, arm_rms_q31, arm_std_q31, arm_var_q31, arm_mse_q31. (accumulate, entropy, kullback_leibler, logsumexp
 * have no q31 variant.)
 *
 * ARM ids (Testing/desc.txt:272-373 "Statistics Tests Q31"; tests in Testing/Source/Tests/StatsTestsQ31.cpp, test bodies
 * at :22-340; setUp cases from :345 on; all 45 setUp cases are run here):
 *   every unit above with nb = 3 / 4n (8) / 4n+1 (11): the first nb samples of the input pattern, reference at
 *   refOffset 0 / 1 / 2 of the reference pattern (inputs: Input1 for max, min, power, rms, std, var and the _no_idx of
 *   max and min; Input2 for mean; InputNew1 for the abs functions, mse also InputNew2);
 *   mse "long" (nb = 100, refOffset 3); "saturation" arm_absmax_no_idx_q31 and arm_absmax_q31 (all 600 samples of
 *   InputNew1, refOffset 3, desc.txt lists both; the setUp cases are TEST_ABSMAX_NO_IDX_Q31_44 and TEST_ABSMAX_Q31_45).
 *   None is commented out or disabled.
 * FastMath Q63 (desc.txt:1948-1965; FastMathQ63.cpp:19-58): test_norm_64_to_32u (4 inputs) and test_div_int64_to_int32
 *   (12 pairs), compared exactly (ABS_ERROR 0, FastMathQ63.cpp:17; ASSERT_EQ at :38-39, :57). ARM's arguments of the
 *   division test are named confusingly: DivDenInput1_s64 is the 64-bit NUMERATOR (first argument), DivNumInput1_s32 the
 *   32-bit DIVISOR; the call is arm_div_int64_to_int32(DivDenInput1[i], DivNumInput1[i]) and is reproduced as is.
 * Pattern files: Patterns/DSP/Stats/StatsQ31/{Input1,Input2,InputNew1,InputNew2}_q31.txt, {MaxVals1,MeanVals2,MinVals3,
 *   RmsVals5,StdVals6,VarVals7,AbsMaxVals8,AbsMinVals9,MSEVals10}_q31.txt, PowerVals4_q63.txt, {MaxIndexes1,
 *   MinIndexes3,AbsMaxIndexes8,AbsMinIndexes9}_s16.txt; Patterns/DSP/FastMath/FastMathQ63/{Norm64To32_Input1_u64,
 *   RefNorm64To32_Vals1_s32,RefNorm64To32_Norms1_s16,DivDenInput1_s64,DivNumInput1_s32,DivRef1_s32}.txt.
 *
 * ARM tolerances (StatsTestsQ31.cpp:8-20; the references are computed by ARM in double precision): SNR 100 dB
 *   (SNR_THRESHOLD :8, SNR_THRESHOLD_MSE :9) AND ABS_ERROR_Q31 100 LSB (:17) for mean, rms, std, var; ABS_ERROR_Q31_MSE
 *   100 (:18) for mse; ABS_ERROR_Q63 2^18 (:20) for power (power: result in 16.48); EXACT (ASSERT_EQ) for all max / min /
 *   abs values and for the indexes (ARM compares (int16_t)index; here the full uint32_t index is compared). FastMath Q63:
 *   exact. Used unchanged (L4).
 *
 * Oracles of the C3/C4 tests, derived from the DOC text of each .c file ("Scaling and Overflow Behavior"):
 *   mean   : 64 bit accumulator, "the accumulator is truncated to yield a result of 1.31": out = sum / n in C integer
 *            division (toward zero). The sum is exact (n < 2^32 samples, |sum| < 2^63). DOC does not say whether the
 *            truncation of the quotient is toward zero or toward minus infinity: positive means are asserted exactly,
 *            negative means with |out - sum / n| < 1, and the toward-zero rule is a characterization test.
 *   power  : "intermediate multiplication yields 2.62, truncated to 2.48 by discarding the lower 14 bits, added without
 *            saturation to a 64 bit accumulator in 16.48, 15 guard bits": out = sum_i floor(in_i^2 / 2^14) (int64 test
 *            arithmetic: in^2 <= 2^62), i.e. out / 2^48 = sum x_i^2 with x = in / 2^31, error in (-n, 0] units (one sided).
 *            Overflow: 2^63 / 2^48 = 2^15 samples of full scale fit, so n = 32767 full scale samples are tested.
 *   rms    : out = sqrt(mean(x^2)) in q31. m_q = floor(sum in^2 / n / 2^31) clipped to 0x7FFFFFFF (clip_q63_to_q31) is
 *            passed to arm_sqrt_q31, whose measured worst case is 9 LSB (kSqrtLsb; ARM's bound of 7, FastMathQ31.cpp:16, is exceeded, G1 B-DFQ31-2). With M = min(sum in^2 /
 *            (n 2^31), 2^31 - 1): m_q in (M - 1, M] and out in [sqrt((M - 1) 2^31) - 7, sqrt(M 2^31) + 7] (the lower
 *            end clamped at M - 1 >= 0). The truncation of the argument is the dominant error for small rms (up to
 *            2^15.5 LSB for an rms of one LSB), which is why the bound depends on the data.
 *            DOC: "If the accumulator overflows, it wraps around and distorts the result": the accumulator is uint64
 *            and four full scale samples wrap it; the value is not documented, not tested (Lead decisions).
 *   var    : "input downshifted by 8 bits (1.23), products 2.46, ... After division, internal variables should be
 *            Q18.46, finally right shifted by 15 bits to yield 1.31", unbiased estimator (n - 1, DOC of the f32 sibling:
 *            Result = (sumOfSquares - sum^2 / blockSize) / (blockSize - 1)). With q_i = floor(in_i / 256): V =
 *            sum (q - qmean)^2 / (n - 1) is the exact value, mos = trunc(sumsq / (n - 1)), som = trunc(sum^2 / (n (n - 1))),
 *            mos - som = V - e1 + e2 with e1, e2 in [0, 1), so out = floor((mos - som) / 2^15) is in (D - 1 - 2^-15,
 *            D + 2^-15] with D = V / 2^15 the exact unbiased variance of the QUANTISED inputs in LSB of q31.
 *            For inputs that are multiples of 256 (q exact) D is the variance of the inputs: window (D - 1.001, D + 0.001].
 *            For arbitrary inputs x - x_q = e in [0, 255 / 2^31] (< 2^-23): var(x_q) = var(x) - 2 cov(x, e) + var(e) with
 *            sd(e) <= sqrt(n / (n - 1)) 2^-24 <= sqrt(2) 2^-24 (Cauchy-Schwarz, same n - 1 normalisation), so |var(x_q) -
 *            var(x)| <= 2 sqrt(2) 2^-24 sd(x) + 2 2^-48; times 2^31 = 362.04 sd(x) + 2^-16 LSB; adding the window above:
 *            |out - D(x)| <= 363 sd(x) + 2 LSB, sd(x) in fractions of full scale.
 *            Format limit: the unbiased variance of data near full scale exceeds 1.0 (not representable); DOC says nothing
 *            about saturation. Tested only with variances below 1 (inputs within +-0.5 of full scale: var <= 0.5).
 *   std    : out = arm_sqrt_q31((mos - som) >> 15): the var window above (dlo, dhi) is the window of the argument v_q,
 *            and out in [sqrt(max(D - dlo, 0) 2^31) - 9, sqrt((D + dhi) 2^31) + 9] (9 LSB: arm_sqrt_q31, exhaustively measured worst case, x = 2147483616).
 *   mse    : 'diff = floor(a / 2) - floor(b / 2)' (saturating subtraction, never saturates: |diff| <= 2^31 - 1),
 *            sum += (diff^2) >> 14, out = (sum / n) >> 15. diff = (a - b) / 2 + d with d in {-1/2, 0, 1/2}, so
 *            diff^2 differs from (a - b)^2 / 4 by at most |a - b| / 2 + 1/4; two truncations: the >> 14 (loses [0, 1) per
 *            sample), the division and the final >> 15 (together < 2 units of 2^15 and < 1 LSB). With D = sum (a - b)^2 /
 *            (n 2^31) (the mean square error as a q31 number): out in (D - m - 1.001, D + m + 0.001] with m = mean|a - b| 2^-30 (the truncations only lower the value; the
 *            input halving error is +- m). D must stay
 *            below 1.0 (2^31 LSB) to be representable; DOC says nothing about larger values (Lead decisions).
 *   max / min family: exact values, "index of maximum value": first index on ties (the code compares with < / >),
 *            DOC silent about ties -> characterization tests. abs functions: |INT32_MIN| = INT32_MAX (saturated), backed
 *            by ARM's "saturation" ids (pattern AbsMaxVals8 offset 3 is 0x7FFFFFFF, index 40).
 * Sweeps (lesson 2 of the work orders): sweepData() has 4096 values i * 2654435761 (Weyl sequence, uniform over the whole
 *   int32 range, stride 2^32 / 4096 on average in a scrambled order) plus the 64 values at both ends and around 0; the
 *   functions are run on sliding windows (block sizes 1..9 in rotation, so the 4-way unrolled loop and its tail are hit
 *   at every phase) against the DEF. The single sample domain of a q31 function cannot be exhaustive (2^32).
 *
 * NEEDS REVIEW (format limits, values deliberately NOT pinned): the q31 DOC does not promise saturation. var > 1.0 wraps to a
 *   negative variance (true 2.0 for {INT32_MAX, INT32_MIN} gives -512 LSB, true 1.53 for {0.875, -0.875} gives -0.4688);
 *   std of such data returns 0 (arm_sqrt_q31 of a negative argument); mse >= 1.0 wraps ((q31_t) cast: 0.75 against -0.75,
 *   true 2.25, gives 0.25; 0.5 against -0.5, true 1.0, gives INT32_MIN). The q15 siblings saturate (mse, std), so this
 *   is inconsistent. Not tested: block size 0 of max / min / abs (out-of-bounds read), mean / rms (division by zero),
 *   arm_div_int64_to_int32(x, 0) (SIGFPE), rms accumulator wrap (4 full scale samples).
 * B-DSQ31-3: blockSize * (blockSize - 1U) is computed in uint32 in arm_var_q31.c:208 and arm_std_q31.c:152 and wraps for
 *   n > 65536; DOC allows large n for inputs scaled down by log2(n) - 8 bits, so a DOC-compliant input gives a wrong
 *   (here 0, reviewer probe: negative) variance. Two KNOWN_ISSUE cases below.
 * C5 bounds (q31 against the f32 function of the same input x_f = float(in) / 2^31; u = 2^-24 the f32 unit roundoff;
 *   host sums are sequential in f32): derived at each test.
 *
 * Host vs target (C6): the host runs the portable C path (no ARM_MATH_DSP, no MVE, ARM_MATH_LOOPUNROLL on): the inline
 *   __QSUB of dsp/none.h is the saturating subtraction of the intrinsic. All q31 functions are integer arithmetic and
 *   are expected to give identical results on the target; the MVE variants (asrl, vaddlva) are different code and are
 *   not run here (mse: the MVE path shifts by 21 where the portable path shifts by 15, arm_mse_q31.c:103 / :172, the
 *   accumulation differs accordingly). The f32 reference functions of the C5 tests sum sequentially on the host (FMA
 *   contraction could change the last bits on the target, the bounds contain n u). var / std / mse compute
 *   blockSize * (blockSize - 1) in uint32 (overflows above 65536 samples; not tested). Right shifts of negative values
 *   (in >> 8) are implementation defined (arithmetic on GCC/clang/ARM). The norm helper left/right shifts by a count of 32
 *   or more / negative for inputs >= 2^62 (undefined behaviour, B-DSQ31-1): x86 masks the count; the target result is
 *   unverified (a register shift by >= 32 gives 0 on Cortex-M, so for inputs >= 2^63 the target value is likely 0; the
 *   bit 62 case behaves as described at the test). Only ARM's div pattern reaches this UB (UBSan-confirmed, utils.h:221,
 *   through arm_div_int64_to_int32 -> arm_norm_64_to_32u); ARM's norm pattern (4 inputs) is clean.
 */
#include "doctest.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math.h"

namespace {

const double kTwo31 = 2147483648.0;
__extension__ typedef unsigned __int128 U128;  // GCC / clang / MinGW; exact sums in the power test
const double kU = 5.9604644775390625e-08;  // 2^-24, f32 unit roundoff
// accuracy of arm_sqrt_q31 used by the rms / std windows: ARM's 7 LSB (FastMathQ31.cpp:16) is exceeded by arm_sqrt_q31 itself
// (G1 finding B-DFQ31-2: worst error 9.0 LSB over all inputs, exhaustively verified at x = 2147483616; e.g. x = 2147483632 gives 2147483632, error 8). The rms / std
// tests use the measured worst case 9 so that they check the statistics code and do not repeat that defect.
const double kSqrtLsb = 9.0;
// right shift that scales n samples down by ceil(log2 n) bits, as the DOC of rms / var / std demands (overflow of the 64 bit accumulator)
int docShift(size_t n) {
	int s = 0;
	while (((size_t)1 << s) < n) s++;
	return s;
}
const char *const kDir = "Stats/StatsQ31/";
const char *const kDirQ63 = "FastMath/FastMathQ63/";

dsp::Pattern pat(const char *name) { return dsp::loadPattern(std::string(kDir) + name); }
dsp::Pattern pat63(const char *name) { return dsp::loadPattern(std::string(kDirQ63) + name); }

using ExtFn = void (*)(const q31_t *, uint32_t, q31_t *, uint32_t *);
using NoIdxFn = void (*)(const q31_t *, uint32_t, q31_t *);  // also mean, rms, std, var
using PowerFn = void (*)(const q31_t *, uint32_t, q63_t *);
using MseFn = void (*)(const q31_t *, const q31_t *, uint32_t, q31_t *);

// deterministic generator (xorshift32): one call per statement (lesson 1)
uint32_t rnd(uint32_t &s) {
	s ^= s << 13;
	s ^= s >> 17;
	s ^= s << 5;
	return s;
}

// n values, full range shifted right by `shift` bits (shift 1: +-0.5 of full scale)
std::vector<int32_t> randomData(size_t n, uint32_t seed, int shift) {
	std::vector<int32_t> v(n);
	uint32_t s = seed;
	for (size_t i = 0; i < n; i++) {
		uint32_t r = rnd(s);
		v[i] = (int32_t)r >> shift;
	}
	return v;
}

int32_t multipleOf256(int32_t x) { return (int32_t)((uint32_t)x & 0xFFFFFF00u); }
int32_t multipleOf128(int32_t x) { return (int32_t)((uint32_t)x & 0xFFFFFF80u); }

// 4096 Weyl sequence values over the whole range, plus the neighbourhood of both ends and of zero
std::vector<int32_t> sweepData() {
	std::vector<int32_t> v;
	for (uint32_t i = 0; i < 4096; i++) v.push_back((int32_t)(i * 2654435761u));
	for (int d = 0; d < 64; d++) {
		v.push_back((int32_t)((int64_t)INT32_MIN + d));
		v.push_back((int32_t)((int64_t)INT32_MAX - d));
		v.push_back(d);
		v.push_back(-d);
	}
	return v;
}

// saturating abs: |INT32_MIN| = INT32_MAX
int32_t absSat(int32_t x) {
	int64_t a = x < 0 ? -(int64_t)x : (int64_t)x;
	return (int32_t)std::min<int64_t>(a, INT32_MAX);
}

struct Ext {
	int32_t val;
	uint32_t idx;
};

// DEF of the max / min / absmax / absmin family: first extremum (std::max_element / min_element return the first)
Ext defExt(const int32_t *p, size_t n, bool wantMax, bool useAbs) {
	std::vector<int32_t> w(p, p + n);
	if (useAbs)
		for (auto &e : w) e = absSat(e);
	auto it = wantMax ? std::max_element(w.begin(), w.end()) : std::min_element(w.begin(), w.end());
	return Ext{*it, (uint32_t)(it - w.begin())};
}

int64_t sumOf(const int32_t *p, size_t n) {
	int64_t s = 0;
	for (size_t i = 0; i < n; i++) s += p[i];
	return s;
}

// DOC of arm_power_q31: sum of floor(in^2 / 2^14) (int64 test arithmetic)
int64_t docPower(const int32_t *p, size_t n) {
	int64_t s = 0;
	for (size_t i = 0; i < n; i++) s += ((int64_t)p[i] * p[i]) >> 14;
	return s;
}

// sum in^2 / (n 2^31): the mean square as a q31 number, in double (in^2 <= 2^62, relative error 2^-53 per term)
double meanSquareLsb(const int32_t *p, size_t n) {
	double s = 0.0;
	for (size_t i = 0; i < n; i++) s += (double)p[i] * (double)p[i];
	return s / ((double)n * kTwo31);
}

// window of arm_rms_q31 (derivation in the header comment)
void rmsBounds(const int32_t *p, size_t n, double &lo, double &hi) {
	const double M = std::min(meanSquareLsb(p, n), 2147483647.0);
	hi = std::sqrt(M * kTwo31) + kSqrtLsb;
	lo = std::sqrt(std::max(M - 1.0, 0.0) * kTwo31) - kSqrtLsb;
}

// unbiased variance (two pass, double) of the inputs as fractions of full scale
double defVar(const int32_t *p, size_t n) {
	double mean = 0.0;
	for (size_t i = 0; i < n; i++) mean += (double)p[i] / kTwo31;
	mean /= (double)n;
	double acc = 0.0;
	for (size_t i = 0; i < n; i++) {
		const double d = (double)p[i] / kTwo31 - mean;
		acc += d * d;
	}
	return acc / (double)(n - 1);
}

// window (dlo, dhi) of arm_var_q31 around D = var * 2^31
void varWindow(const int32_t *p, size_t n, bool exactInputs, double &lo, double &hi) {
	const double D = defVar(p, n) * kTwo31;
	if (exactInputs) {
		lo = D - 1.001;
		hi = D + 0.001;
	} else {
		const double w = 363.0 * std::sqrt(defVar(p, n)) + 2.0;
		lo = D - w;
		hi = D + w;
	}
}

// window of arm_std_q31 from the window of its argument
void stdWindow(double varLo, double varHi, double &lo, double &hi) {
	lo = std::sqrt(std::max(varLo, 0.0) * kTwo31) - kSqrtLsb;
	hi = std::sqrt(std::max(varHi, 0.0) * kTwo31) + kSqrtLsb;
}

// DEF of arm_mse_q31 and its bound (header comment)
double defMse(const int32_t *a, const int32_t *b, size_t n) {
	double s = 0.0;
	for (size_t i = 0; i < n; i++) {
		const double d = (double)a[i] - (double)b[i];
		s += d * d;
	}
	return s / ((double)n * kTwo31);
}
double mseBound(const int32_t *a, const int32_t *b, size_t n) {
	double s = 0.0;
	for (size_t i = 0; i < n; i++) s += std::fabs((double)a[i] - (double)b[i]);
	return s / (double)n / 1073741824.0 + 1.001;
}

// one-sided window of arm_mse_q31 (derivation in the header): out in (D - m - 1.001, D + m + 0.001], m = mean|a - b| 2^-30
bool mseInWindow(double out, const int32_t *a, const int32_t *b, size_t n) {
	double s = 0.0;
	for (size_t i = 0; i < n; i++) s += std::fabs((double)a[i] - (double)b[i]);
	const double m = s / (double)n / 1073741824.0;
	const double D = defMse(a, b, n);
	return out > D - m - 1.001 && out <= D + m + 0.001;
}

// mean checks: positive sums exactly floor(sum / n); negative sums |out - sum / n| < 1
bool meanOk(int32_t out, int64_t sum, size_t n) {
	if (sum >= 0) return (int64_t)out == sum / (int64_t)n;
	return std::fabs((double)out - (double)sum / (double)n) < 1.0;
}

// ---- C1 helpers -------------------------------------------------------------------------------------------------

// ARM's tests of the max / min / absmax / absmin with index (nb = 0: all samples of the input pattern)
void c1Ext(ExtFn fn, const char *in, const char *refVal, const char *refIdx, size_t nb, size_t off) {
	std::vector<int32_t> x = pat(in).q31(), rv = pat(refVal).q31();
	std::vector<int16_t> ri = pat(refIdx).q15();
	if (nb == 0) nb = x.size();
	REQUIRE(nb <= x.size());
	REQUIRE(off < rv.size());
	REQUIRE(off < ri.size());
	q31_t r = 0;
	uint32_t idx = 0xFFFFFFFFu;
	fn(x.data(), (uint32_t)nb, &r, &idx);
	CHECK(r == rv[off]);
	CHECK(idx == (uint32_t)ri[off]);
}

void c1NoIdx(NoIdxFn fn, const char *in, const char *refVal, size_t nb, size_t off) {
	std::vector<int32_t> x = pat(in).q31(), rv = pat(refVal).q31();
	if (nb == 0) nb = x.size();
	REQUIRE(nb <= x.size());
	REQUIRE(off < rv.size());
	q31_t r = 0;
	fn(x.data(), (uint32_t)nb, &r);
	CHECK(r == rv[off]);
}

// mean, rms, std, var: SNR 100 dB and 100 LSB
void c1Scalar(NoIdxFn fn, const char *in, const char *refVal, size_t nb, size_t off) {
	std::vector<int32_t> x = pat(in).q31(), rv = pat(refVal).q31();
	REQUIRE(nb <= x.size());
	REQUIRE(off < rv.size());
	q31_t r = 0;
	fn(x.data(), (uint32_t)nb, &r);
	std::vector<int32_t> ref{rv[off]}, out{r};
	CHECK(dsp::snr(ref, out) >= 100.0);
	auto c = dsp::withinLsb(ref, out, 100);
	CHECK_MESSAGE(c, c.message);
}

void c1Power(size_t nb, size_t off) {
	std::vector<int32_t> x = pat("Input1_q31.txt").q31();
	std::vector<int64_t> rv = pat("PowerVals4_q63.txt").s64();
	REQUIRE(nb <= x.size());
	REQUIRE(off < rv.size());
	q63_t r = 0;
	arm_power_q31(x.data(), (uint32_t)nb, &r);
	std::vector<int64_t> ref{rv[off]}, out{r};
	CHECK(dsp::snr(ref, out) >= 100.0);
	auto c = dsp::withinLsb(ref, out, (int64_t)1 << 18);
	CHECK_MESSAGE(c, c.message);
}

void c1Mse(size_t nb, size_t off) {
	std::vector<int32_t> a = pat("InputNew1_q31.txt").q31(), b = pat("InputNew2_q31.txt").q31();
	std::vector<int32_t> rv = pat("MSEVals10_q31.txt").q31();
	REQUIRE(nb <= a.size());
	REQUIRE(nb <= b.size());
	REQUIRE(off < rv.size());
	q31_t r = 0;
	arm_mse_q31(a.data(), b.data(), (uint32_t)nb, &r);
	std::vector<int32_t> ref{rv[off]}, out{r};
	CHECK(dsp::snr(ref, out) >= 100.0);
	auto c = dsp::withinLsb(ref, out, 100);
	CHECK_MESSAGE(c, c.message);
}

void c1Max(size_t nb, size_t off) { c1Ext(arm_max_q31, "Input1_q31.txt", "MaxVals1_q31.txt", "MaxIndexes1_s16.txt", nb, off); }
void c1Min(size_t nb, size_t off) { c1Ext(arm_min_q31, "Input1_q31.txt", "MinVals3_q31.txt", "MinIndexes3_s16.txt", nb, off); }
void c1AbsMax(size_t nb, size_t off) { c1Ext(arm_absmax_q31, "InputNew1_q31.txt", "AbsMaxVals8_q31.txt", "AbsMaxIndexes8_s16.txt", nb, off); }
void c1AbsMin(size_t nb, size_t off) { c1Ext(arm_absmin_q31, "InputNew1_q31.txt", "AbsMinVals9_q31.txt", "AbsMinIndexes9_s16.txt", nb, off); }
void c1MaxNoIdx(size_t nb, size_t off) { c1NoIdx(arm_max_no_idx_q31, "Input1_q31.txt", "MaxVals1_q31.txt", nb, off); }
void c1MinNoIdx(size_t nb, size_t off) { c1NoIdx(arm_min_no_idx_q31, "Input1_q31.txt", "MinVals3_q31.txt", nb, off); }
void c1AbsMaxNoIdx(size_t nb, size_t off) { c1NoIdx(arm_absmax_no_idx_q31, "InputNew1_q31.txt", "AbsMaxVals8_q31.txt", nb, off); }
void c1AbsMinNoIdx(size_t nb, size_t off) { c1NoIdx(arm_absmin_no_idx_q31, "InputNew1_q31.txt", "AbsMinVals9_q31.txt", nb, off); }
void c1Mean(size_t nb, size_t off) { c1Scalar(arm_mean_q31, "Input2_q31.txt", "MeanVals2_q31.txt", nb, off); }
void c1Rms(size_t nb, size_t off) { c1Scalar(arm_rms_q31, "Input1_q31.txt", "RmsVals5_q31.txt", nb, off); }
void c1Std(size_t nb, size_t off) { c1Scalar(arm_std_q31, "Input1_q31.txt", "StdVals6_q31.txt", nb, off); }
void c1Var(size_t nb, size_t off) { c1Scalar(arm_var_q31, "Input1_q31.txt", "VarVals7_q31.txt", nb, off); }

// ---- sweep helpers (windows of rotating size over sweepData) ------------------------------------------------------

void sweepExt(ExtFn fn, bool wantMax, bool useAbs, bool dropIntMin) {
	std::vector<int32_t> v = sweepData();
	if (dropIntMin) v.erase(std::remove(v.begin(), v.end(), INT32_MIN), v.end());
	size_t windows = 0;
	for (size_t i = 0; i + 9 <= v.size(); i++) {
		const size_t n = 1 + (i % 9);
		const Ext e = defExt(&v[i], n, wantMax, useAbs);
		q31_t r = 0;
		uint32_t idx = 0xFFFFFFFFu;
		fn(&v[i], (uint32_t)n, &r, &idx);
		windows++;
		if (r != e.val || idx != e.idx) {
			FAIL_CHECK("window at " << i << " size " << n << ": value " << r << " index " << idx << ", expected "
			                        << e.val << " index " << e.idx);
			return;
		}
	}
	CHECK(windows > 4000);
}

void sweepNoIdx(NoIdxFn fn, bool wantMax, bool useAbs, bool dropIntMin) {
	std::vector<int32_t> v = sweepData();
	if (dropIntMin) v.erase(std::remove(v.begin(), v.end(), INT32_MIN), v.end());
	size_t windows = 0;
	for (size_t i = 0; i + 9 <= v.size(); i++) {
		const size_t n = 1 + (i % 9);
		const Ext e = defExt(&v[i], n, wantMax, useAbs);
		q31_t r = 0;
		fn(&v[i], (uint32_t)n, &r);
		windows++;
		if (r != e.val) {
			FAIL_CHECK("window at " << i << " size " << n << ": value " << r << ", expected " << e.val);
			return;
		}
	}
	CHECK(windows > 4000);
}

// random data of many sizes (full range, or shifted right by `shift` bits)
void randomExt(ExtFn fn, bool wantMax, bool useAbs, int shift) {
	const size_t sizes[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 16, 17, 33, 64, 100, 257};
	uint32_t seed = 12345u;
	for (size_t n : sizes)
		for (int rep = 0; rep < 8; rep++) {
			seed = seed * 747796405u + 2891336453u;
			std::vector<int32_t> v = randomData(n, seed | 1u, shift);
			const Ext e = defExt(v.data(), n, wantMax, useAbs);
			q31_t r = 0;
			uint32_t idx = 0xFFFFFFFFu;
			fn(v.data(), (uint32_t)n, &r, &idx);
			CAPTURE(n);
			CAPTURE(rep);
			CHECK(r == e.val);
			CHECK(idx == e.idx);
		}
}

void randomNoIdx(NoIdxFn fn, bool wantMax, bool useAbs, int shift) {
	const size_t sizes[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 16, 17, 33, 64, 100, 257};
	uint32_t seed = 54321u;
	for (size_t n : sizes)
		for (int rep = 0; rep < 8; rep++) {
			seed = seed * 747796405u + 2891336453u;
			std::vector<int32_t> v = randomData(n, seed | 1u, shift);
			const Ext e = defExt(v.data(), n, wantMax, useAbs);
			q31_t r = 0;
			fn(v.data(), (uint32_t)n, &r);
			CAPTURE(n);
			CAPTURE(rep);
			CHECK(r == e.val);
		}
}

// guard test of the index functions: one result word and one index word, nothing around them is written
void guardExt(ExtFn fn, bool wantMax, bool useAbs) {
	const size_t sizes[] = {1, 2, 3, 4, 5, 9, 13};
	for (size_t n : sizes) {
		std::vector<int32_t> v = randomData(n, 777u + (uint32_t)n, 0);
		dsp::GuardedBuffer<int32_t> val(1);
		dsp::GuardedBuffer<uint32_t> idx(1);
		fn(v.data(), (uint32_t)n, val.data(), idx.data());
		const Ext e = defExt(v.data(), n, wantMax, useAbs);
		CAPTURE(n);
		CHECK(val.data()[0] == e.val);
		CHECK(idx.data()[0] == e.idx);
		CHECK(val.guardIntact());
		CHECK(idx.guardIntact());
	}
}

void guardNoIdx(NoIdxFn fn, bool wantMax, bool useAbs) {
	const size_t sizes[] = {1, 2, 3, 4, 5, 9, 13};
	for (size_t n : sizes) {
		std::vector<int32_t> v = randomData(n, 888u + (uint32_t)n, 0);
		dsp::GuardedBuffer<int32_t> val(1);
		fn(v.data(), (uint32_t)n, val.data());
		const Ext e = defExt(v.data(), n, wantMax, useAbs);
		CAPTURE(n);
		CHECK(val.data()[0] == e.val);
		CHECK(val.guardIntact());
	}
}

// ---- norm helper ------------------------------------------------------------------------------------------------

// DEF of arm_norm_64_to_32u (derived from the code and ARM's pattern): in = 0 -> (0, 0). Otherwise q = index of the most
// significant set bit of in (0..63); normalized = in * 2^(30 - q) truncated (bit 30 set), norm = 30 - q.
// (In the code: 32 bit input with n1 leading zeros -> norm = n1 - 1 = 30 - q because q = 31 - n1; in >= 2^32 ->
// norm = clz(hi) - 33 = 30 - q because q = 63 - clz(hi).)
void defNorm(uint64_t in, int32_t &val, int32_t &norm) {
	if (in == 0) {
		val = 0;
		norm = 0;
		return;
	}
	int q = 63;
	while (!((in >> q) & 1u)) q--;
	norm = 30 - q;
	val = (int32_t)(q >= 30 ? (in >> (q - 30)) : (in << (30 - q)));
}

// DEF of arm_div_int64_to_int32: the quotient truncated toward zero, saturated to the int32 range
// ("The function saturates the result to fit within the 32-bit integer range", dsp/utils.h)
int32_t defDiv(int64_t num, int32_t den) {
	// the only int64 overflow, INT64_MIN / -1 = 2^63, is saturated explicitly
	if (num == INT64_MIN && den == -1) return INT32_MAX;
	const int64_t q = num / (int64_t)den;
	return (int32_t)std::min<int64_t>(std::max<int64_t>(q, INT32_MIN), INT32_MAX);
}

}  // namespace

TEST_SUITE("DSP Statistics q31") {

// ---------------------------------------------------------------------------------------------------------------
// C1: ARM's tests, block sizes nb = 3, 4n (8), 4n+1 (11) with the reference at offset 0, 1, 2
// ---------------------------------------------------------------------------------------------------------------

#define C1_BLOCK_SIZES(UNIT, FN, DESC)                                                                             \
	TEST_CASE("[C1 arm_" UNIT "_q31] " DESC " for block size 3 (nb=3)") { FN(3, 0); }                            \
	TEST_CASE("[C1 arm_" UNIT "_q31] " DESC " for block size 8 (nb=4n)") { FN(8, 1); }                           \
	TEST_CASE("[C1 arm_" UNIT "_q31] " DESC " for block size 11 (nb=4n+1)") { FN(11, 2); }

C1_BLOCK_SIZES("max", c1Max, "the maximum and its index equal ARM's reference exactly")
C1_BLOCK_SIZES("min", c1Min, "the minimum and its index equal ARM's reference exactly")
C1_BLOCK_SIZES("absmax", c1AbsMax, "the absolute maximum and its index equal ARM's reference exactly")
C1_BLOCK_SIZES("absmin", c1AbsMin, "the absolute minimum and its index equal ARM's reference exactly")
C1_BLOCK_SIZES("max_no_idx", c1MaxNoIdx, "the maximum equals ARM's reference exactly")
C1_BLOCK_SIZES("min_no_idx", c1MinNoIdx, "the minimum equals ARM's reference exactly")
C1_BLOCK_SIZES("absmax_no_idx", c1AbsMaxNoIdx, "the absolute maximum equals ARM's reference exactly")
C1_BLOCK_SIZES("absmin_no_idx", c1AbsMinNoIdx, "the absolute minimum equals ARM's reference exactly")
C1_BLOCK_SIZES("mean", c1Mean, "the mean is within 100 dB SNR and 100 LSB of ARM's reference")
C1_BLOCK_SIZES("power", c1Power, "the power is within 100 dB SNR and 2^18 of ARM's reference")
C1_BLOCK_SIZES("rms", c1Rms, "the rms is within 100 dB SNR and 100 LSB of ARM's reference")
C1_BLOCK_SIZES("std", c1Std, "the standard deviation is within 100 dB SNR and 100 LSB of ARM's reference")
C1_BLOCK_SIZES("var", c1Var, "the variance is within 100 dB SNR and 100 LSB of ARM's reference")

TEST_CASE("[C1 arm_mse_q31] the mean square error is within 100 dB SNR and 100 LSB of ARM's reference for block size 3 (nb=3)") { c1Mse(3, 0); }
TEST_CASE("[C1 arm_mse_q31] the mean square error is within 100 dB SNR and 100 LSB of ARM's reference for block size 8 (nb=4n)") { c1Mse(8, 1); }
TEST_CASE("[C1 arm_mse_q31] the mean square error is within 100 dB SNR and 100 LSB of ARM's reference for block size 11 (nb=4n+1)") { c1Mse(11, 2); }
TEST_CASE("[C1 arm_mse_q31] the mean square error is within 100 dB SNR and 100 LSB of ARM's reference for block size 100 (ARM's long test)") { c1Mse(100, 3); }

TEST_CASE("[C1 arm_absmax_no_idx_q31] the absolute maximum of all 600 samples saturates to 0x7FFFFFFF like ARM's saturation test") {
	c1NoIdx(arm_absmax_no_idx_q31, "InputNew1_q31.txt", "AbsMaxVals8_q31.txt", 0, 3);
}
TEST_CASE("[C1 arm_absmax_q31] the absolute maximum of all 600 samples saturates to 0x7FFFFFFF at index 40 like ARM's saturation test") {
	c1Ext(arm_absmax_q31, "InputNew1_q31.txt", "AbsMaxVals8_q31.txt", "AbsMaxIndexes8_s16.txt", 0, 3);
}

// ---------------------------------------------------------------------------------------------------------------
// C1 FastMath Q63 (L7)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C1 arm_norm_64_to_32u] the normalised value and the shift of ARM's 4 inputs equal the reference exactly") {
	std::vector<uint64_t> in = pat63("Norm64To32_Input1_u64.txt").u64();
	std::vector<int32_t> refVal = pat63("RefNorm64To32_Vals1_s32.txt").q31();
	std::vector<int16_t> refNorm = pat63("RefNorm64To32_Norms1_s16.txt").q15();
	REQUIRE(in.size() == refVal.size());
	REQUIRE(in.size() == refNorm.size());
	std::vector<int32_t> val(in.size()), norm(in.size());
	for (size_t i = 0; i < in.size(); i++) arm_norm_64_to_32u(in[i], &val[i], &norm[i]);
	auto cv = dsp::exactEqual(refVal, val);
	CHECK_MESSAGE(cv, cv.message);
	auto cn = dsp::exactEqual(refNorm, norm);
	CHECK_MESSAGE(cn, cn.message);
}

TEST_CASE("[C1 arm_div_int64_to_int32] the quotient of ARM's 12 divisions equals the reference exactly" * KNOWN_ISSUE_UB_HIDDEN("B-DSQ31-1: arm_div_int64_to_int32 calls arm_norm_64_to_32u, whose shifts are undefined for |num| >= 2^62 (utils.h:221); the quotient itself is correct on the host (the result only uses the sign of the shift)")) {
	std::vector<int64_t> num = pat63("DivDenInput1_s64.txt").s64();  // the 64-bit numerator (ARM's naming is swapped)
	std::vector<int32_t> den = pat63("DivNumInput1_s32.txt").q31();
	std::vector<int32_t> ref = pat63("DivRef1_s32.txt").q31();
	REQUIRE(num.size() == ref.size());
	REQUIRE(den.size() == ref.size());
	std::vector<int32_t> out(ref.size());
	for (size_t i = 0; i < ref.size(); i++) out[i] = arm_div_int64_to_int32(num[i], den[i]);
	auto c = dsp::exactEqual(ref, out);
	CHECK_MESSAGE(c, c.message);
}

// ---------------------------------------------------------------------------------------------------------------
// C2: the outputs are exactly one result word (and one index word); nothing around them is written
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C2 arm_max_q31] for block sizes 1 to 13 the value and index are correct and the words behind them are untouched") { guardExt(arm_max_q31, true, false); }
TEST_CASE("[C2 arm_min_q31] for block sizes 1 to 13 the value and index are correct and the words behind them are untouched") { guardExt(arm_min_q31, false, false); }
TEST_CASE("[C2 arm_absmax_q31] for block sizes 1 to 13 the value and index are correct and the words behind them are untouched") { guardExt(arm_absmax_q31, true, true); }
TEST_CASE("[C2 arm_absmin_q31] for block sizes 1 to 13 the value and index are correct and the words behind them are untouched") { guardExt(arm_absmin_q31, false, true); }
TEST_CASE("[C2 arm_max_no_idx_q31] for block sizes 1 to 13 the value is correct and the words behind it are untouched") { guardNoIdx(arm_max_no_idx_q31, true, false); }
TEST_CASE("[C2 arm_min_no_idx_q31] for block sizes 1 to 13 the value is correct and the words behind it are untouched") { guardNoIdx(arm_min_no_idx_q31, false, false); }
TEST_CASE("[C2 arm_absmax_no_idx_q31] for block sizes 1 to 13 the value is correct and the words behind it are untouched") { guardNoIdx(arm_absmax_no_idx_q31, true, true); }
TEST_CASE("[C2 arm_absmin_no_idx_q31] for block sizes 1 to 13 the value is correct and the words behind it are untouched") { guardNoIdx(arm_absmin_no_idx_q31, false, true); }

TEST_CASE("[C2 arm_mean_q31] for block sizes 1 to 13 the mean is written and the words behind it are untouched") {
	const size_t sizes[] = {1, 2, 3, 4, 5, 9, 13};
	for (size_t n : sizes) {
		std::vector<int32_t> v = randomData(n, 31u + (uint32_t)n, 0);
		dsp::GuardedBuffer<int32_t> out(1);
		arm_mean_q31(v.data(), (uint32_t)n, out.data());
		CAPTURE(n);
		CHECK(meanOk(out.data()[0], sumOf(v.data(), n), n));
		CHECK(out.guardIntact());
	}
}

TEST_CASE("[C2 arm_power_q31] for block sizes 1 to 13 the 64 bit power is written and the words behind it are untouched") {
	const size_t sizes[] = {1, 2, 3, 4, 5, 9, 13};
	for (size_t n : sizes) {
		std::vector<int32_t> v = randomData(n, 41u + (uint32_t)n, 0);
		dsp::GuardedBuffer<int64_t> out(1);
		arm_power_q31(v.data(), (uint32_t)n, out.data());
		CAPTURE(n);
		CHECK(out.data()[0] == docPower(v.data(), n));
		CHECK(out.guardIntact());
	}
}

TEST_CASE("[C2 arm_rms_q31] for block sizes 1 to 13 the rms is within its window and the words behind it are untouched") {
	const size_t sizes[] = {1, 2, 3, 4, 5, 9, 13};
	for (size_t n : sizes) {
		std::vector<int32_t> v = randomData(n, 51u + (uint32_t)n, docShift(n));
		dsp::GuardedBuffer<int32_t> out(1);
		arm_rms_q31(v.data(), (uint32_t)n, out.data());
		double lo, hi;
		rmsBounds(v.data(), n, lo, hi);
		CAPTURE(n);
		CHECK(out.data()[0] >= lo);
		CHECK(out.data()[0] <= hi);
		CHECK(out.guardIntact());
	}
}

TEST_CASE("[C2 arm_var_q31] for block sizes 2 to 13 the variance is within its window and the words behind it are untouched") {
	const size_t sizes[] = {2, 3, 4, 5, 9, 13};
	for (size_t n : sizes) {
		std::vector<int32_t> v = randomData(n, 61u + (uint32_t)n, 1);
		dsp::GuardedBuffer<int32_t> out(1);
		arm_var_q31(v.data(), (uint32_t)n, out.data());
		double lo, hi;
		varWindow(v.data(), n, false, lo, hi);
		CAPTURE(n);
		CHECK(out.data()[0] >= lo);
		CHECK(out.data()[0] <= hi);
		CHECK(out.guardIntact());
	}
}

TEST_CASE("[C2 arm_std_q31] for block sizes 2 to 13 the standard deviation is within its window and the words behind it are untouched") {
	const size_t sizes[] = {2, 3, 4, 5, 9, 13};
	for (size_t n : sizes) {
		std::vector<int32_t> v = randomData(n, 71u + (uint32_t)n, 1);
		dsp::GuardedBuffer<int32_t> out(1);
		arm_std_q31(v.data(), (uint32_t)n, out.data());
		double vlo, vhi, lo, hi;
		varWindow(v.data(), n, false, vlo, vhi);
		stdWindow(vlo, vhi, lo, hi);
		CAPTURE(n);
		CHECK(out.data()[0] >= lo);
		CHECK(out.data()[0] <= hi);
		CHECK(out.guardIntact());
	}
}

TEST_CASE("[C2 arm_mse_q31] for block sizes 1 to 13 the mean square error is within its window and the words behind it are untouched") {
	const size_t sizes[] = {1, 2, 3, 4, 5, 9, 13};
	for (size_t n : sizes) {
		std::vector<int32_t> a = randomData(n, 81u + (uint32_t)n, 1), b = randomData(n, 91u + (uint32_t)n, 1);
		dsp::GuardedBuffer<int32_t> out(1);
		arm_mse_q31(a.data(), b.data(), (uint32_t)n, out.data());
		CAPTURE(n);
		CHECK(std::fabs((double)out.data()[0] - defMse(a.data(), b.data(), n)) <= mseBound(a.data(), b.data(), n));
		CHECK(out.guardIntact());
	}
}

// ---------------------------------------------------------------------------------------------------------------
// C3: DEF checks, max / min family
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_max_q31] the maximum and the index of the first maximum match the definition on random data of 17 block sizes") { randomExt(arm_max_q31, true, false, 0); }
TEST_CASE("[C3 arm_min_q31] the minimum and the index of the first minimum match the definition on random data of 17 block sizes") { randomExt(arm_min_q31, false, false, 0); }
TEST_CASE("[C3 arm_absmax_q31] the absolute maximum and its index match the definition on random data of 17 block sizes") { randomExt(arm_absmax_q31, true, true, 0); }
TEST_CASE("[C3 arm_absmin_q31] the absolute minimum and its index match the definition on random data of 17 block sizes (no INT32_MIN in the data)") {
	// 8 reps of full range data: INT32_MIN occurs with probability 2^-32 per sample, i.e. never; the saturating case is a
	// characterization test below
	randomExt(arm_absmin_q31, false, true, 0);
}
TEST_CASE("[C3 arm_max_no_idx_q31] the maximum matches the definition on random data of 17 block sizes") { randomNoIdx(arm_max_no_idx_q31, true, false, 0); }
TEST_CASE("[C3 arm_min_no_idx_q31] the minimum matches the definition on random data of 17 block sizes") { randomNoIdx(arm_min_no_idx_q31, false, false, 0); }
TEST_CASE("[C3 arm_absmax_no_idx_q31] the absolute maximum matches the definition on random data of 17 block sizes") { randomNoIdx(arm_absmax_no_idx_q31, true, true, 0); }
TEST_CASE("[C3 arm_absmin_no_idx_q31] the absolute minimum matches the definition on random data of 17 block sizes") { randomNoIdx(arm_absmin_no_idx_q31, false, true, 0); }

TEST_CASE("[C3 arm_max_q31] over the whole int32 range (sliding windows of 1 to 9 samples) the maximum and index match the definition") { sweepExt(arm_max_q31, true, false, false); }
TEST_CASE("[C3 arm_min_q31] over the whole int32 range (sliding windows of 1 to 9 samples) the minimum and index match the definition") { sweepExt(arm_min_q31, false, false, false); }
TEST_CASE("[C3 arm_absmax_q31] over the whole int32 range including INT32_MIN (sliding windows of 1 to 9 samples) the absolute maximum and index match the definition") { sweepExt(arm_absmax_q31, true, true, false); }
TEST_CASE("[C3 arm_absmin_q31] over the whole int32 range without INT32_MIN (sliding windows of 1 to 9 samples) the absolute minimum and index match the definition") { sweepExt(arm_absmin_q31, false, true, true); }
TEST_CASE("[C3 arm_max_no_idx_q31] over the whole int32 range (sliding windows of 1 to 9 samples) the maximum matches the definition") { sweepNoIdx(arm_max_no_idx_q31, true, false, false); }
TEST_CASE("[C3 arm_min_no_idx_q31] over the whole int32 range (sliding windows of 1 to 9 samples) the minimum matches the definition") { sweepNoIdx(arm_min_no_idx_q31, false, false, false); }
TEST_CASE("[C3 arm_absmax_no_idx_q31] over the whole int32 range including INT32_MIN (sliding windows of 1 to 9 samples) the absolute maximum matches the definition") { sweepNoIdx(arm_absmax_no_idx_q31, true, true, false); }
TEST_CASE("[C3 arm_absmin_no_idx_q31] over the whole int32 range without INT32_MIN (sliding windows of 1 to 9 samples) the absolute minimum matches the definition") { sweepNoIdx(arm_absmin_no_idx_q31, false, true, true); }

TEST_CASE("[C3 arm_max_q31] the maximum of negative data is the value closest to zero, not the largest magnitude") {
	const int32_t v[5] = {-1000, -5, -2000000000, -77, -6};
	q31_t r = 0;
	uint32_t idx = 99;
	arm_max_q31(v, 5, &r, &idx);
	CHECK(r == -5);
	CHECK(idx == 1u);
}

TEST_CASE("[C3 arm_min_q31] the minimum of positive data is the value closest to zero, not the smallest magnitude of a negative one") {
	const int32_t v[5] = {1000, 5, 2000000000, 77, 6};
	q31_t r = 0;
	uint32_t idx = 99;
	arm_min_q31(v, 5, &r, &idx);
	CHECK(r == 5);
	CHECK(idx == 1u);
}

TEST_CASE("[C3 arm_absmax_q31] the absolute maximum of mixed signs returns the magnitude, with the index of the sample that has it") {
	const int32_t v[6] = {100, -700, 300, 699, -50, 7};
	q31_t r = 0;
	uint32_t idx = 99;
	arm_absmax_q31(v, 6, &r, &idx);
	CHECK(r == 700);
	CHECK(idx == 1u);
}

TEST_CASE("[C3 arm_absmin_q31] the absolute minimum of mixed signs returns the magnitude, with the index of the sample that has it") {
	const int32_t v[6] = {100, -700, 30, -29, 50, 7};
	q31_t r = 0;
	uint32_t idx = 99;
	arm_absmin_q31(v, 6, &r, &idx);
	CHECK(r == 7);
	CHECK(idx == 5u);
}

TEST_CASE("[C3 arm_max_q31] characterization: with equal maxima the index of the first one is returned at every position of the unrolled loop") {
	// the code updates only on a strictly greater value; DOC: "index of maximum value" (silent about ties)
	for (uint32_t n = 2; n <= 13; n++)
		for (uint32_t first = 0; first + 1 < n; first++) {
			std::vector<int32_t> v(n, 5);
			v[first] = 100;
			v[n - 1] = 100;  // a second equal maximum after the first one
			q31_t r = 0;
			uint32_t idx = 99;
			arm_max_q31(v.data(), n, &r, &idx);
			CAPTURE(n);
			CAPTURE(first);
			CHECK(r == 100);
			CHECK(idx == first);
		}
}

TEST_CASE("[C3 arm_min_q31] characterization: with equal minima the index of the first one is returned at every position of the unrolled loop") {
	for (uint32_t n = 2; n <= 13; n++)
		for (uint32_t first = 0; first + 1 < n; first++) {
			std::vector<int32_t> v(n, 5);
			v[first] = -100;
			v[n - 1] = -100;
			q31_t r = 0;
			uint32_t idx = 99;
			arm_min_q31(v.data(), n, &r, &idx);
			CAPTURE(n);
			CAPTURE(first);
			CHECK(r == -100);
			CHECK(idx == first);
		}
}

// Host note for the abs tie tests: the portable abs functions have no unrolled loop (only arm_max_q31 / arm_min_q31 do);
// the ARM_MATH_DSP path (4-way unrolled, __QSUB for the magnitude) exists only on the target and is not run here.
TEST_CASE("[C3 arm_absmax_q31] characterization: when +x and -x have the same magnitude the index of the first one is returned") {
	for (uint32_t n = 2; n <= 13; n++)
		for (uint32_t first = 0; first + 1 < n; first++) {
			std::vector<int32_t> v(n, 5);
			v[first] = -100;
			v[n - 1] = 100;
			q31_t r = 0;
			uint32_t idx = 99;
			arm_absmax_q31(v.data(), n, &r, &idx);
			CAPTURE(n);
			CAPTURE(first);
			CHECK(r == 100);
			CHECK(idx == first);
		}
}

TEST_CASE("[C3 arm_absmin_q31] characterization: when +x and -x have the same magnitude the index of the first one is returned") {
	for (uint32_t n = 2; n <= 13; n++)
		for (uint32_t first = 0; first + 1 < n; first++) {
			std::vector<int32_t> v(n, 500);
			v[first] = -3;
			v[n - 1] = 3;
			q31_t r = 0;
			uint32_t idx = 99;
			arm_absmin_q31(v.data(), n, &r, &idx);
			CAPTURE(n);
			CAPTURE(first);
			CHECK(r == 3);
			CHECK(idx == first);
		}
}

// ---------------------------------------------------------------------------------------------------------------
// C3: mean
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_mean_q31] on random data of 19 block sizes the mean is exact (positive sums) or within 1 LSB of sum / n (negative sums)") {
	const size_t sizes[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 16, 17, 33, 64, 100, 257, 1000, 4097};
	uint32_t seed = 99u;
	for (size_t n : sizes)
		for (int rep = 0; rep < 6; rep++) {
			seed = seed * 747796405u + 2891336453u;
			std::vector<int32_t> v = randomData(n, seed | 1u, rep % 3 == 0 ? 0 : 3);
			q31_t r = 0;
			arm_mean_q31(v.data(), (uint32_t)n, &r);
			CAPTURE(n);
			CAPTURE(rep);
			CHECK(meanOk(r, sumOf(v.data(), n), n));
		}
}

TEST_CASE("[C3 arm_mean_q31] the mean of equal samples is that sample, also at both ends of the range") {
	const int32_t values[] = {0, 1, -1, 12345, -12345, INT32_MAX, INT32_MIN, INT32_MAX - 1, INT32_MIN + 1};
	const size_t sizes[] = {1, 2, 3, 4, 5, 8, 9, 100, 4097};
	for (int32_t c : values)
		for (size_t n : sizes) {
			std::vector<int32_t> v(n, c);
			q31_t r = 0;
			arm_mean_q31(v.data(), (uint32_t)n, &r);
			CAPTURE(c);
			CAPTURE(n);
			CHECK(r == c);
		}
}

TEST_CASE("[C3 arm_mean_q31] the sum does not overflow: 70000 samples of INT32_MAX and of INT32_MIN give INT32_MAX and INT32_MIN") {
	// DOC: "accumulated in a 64-bit accumulator in 33.31 format. There is no risk of internal overflow"; the sum 70000 *
	// 2^31 = 1.5e14 does not fit in 32 bits (a 32 bit accumulator would fail)
	std::vector<int32_t> hi(70000, INT32_MAX), lo(70000, INT32_MIN);
	q31_t r = 0;
	arm_mean_q31(hi.data(), 70000, &r);
	CHECK(r == INT32_MAX);
	arm_mean_q31(lo.data(), 70000, &r);
	CHECK(r == INT32_MIN);
}

TEST_CASE("[C3 arm_mean_q31] over the whole int32 range the mean of sliding windows of 1 to 9 samples matches the definition") {
	std::vector<int32_t> v = sweepData();
	size_t windows = 0;
	for (size_t i = 0; i + 9 <= v.size(); i++) {
		const size_t n = 1 + (i % 9);
		q31_t r = 0;
		arm_mean_q31(&v[i], (uint32_t)n, &r);
		windows++;
		if (!meanOk(r, sumOf(&v[i], n), n)) {
			FAIL_CHECK("window at " << i << " size " << n << ": mean " << r << ", sum " << sumOf(&v[i], n));
			return;
		}
	}
	CHECK(windows > 4000);
}

TEST_CASE("[C3 arm_mean_q31] the mean of two samples over a 64 x 64 grid of the whole range matches the definition") {
	for (int i = 0; i < 64; i++)
		for (int j = 0; j < 64; j++) {
			const int32_t pair[2] = {(int32_t)((int64_t)INT32_MIN + (int64_t)i * 67108993), (int32_t)((int64_t)INT32_MAX - (int64_t)j * 67108879)};
			q31_t r = 0;
			arm_mean_q31(pair, 2, &r);
			if (!meanOk(r, sumOf(pair, 2), 2)) {
				FAIL_CHECK("pair " << pair[0] << ", " << pair[1] << ": mean " << r);
				return;
			}
		}
}

TEST_CASE("[C3 arm_mean_q31] characterization: a negative mean that is not an integer is truncated toward zero (C division)") {
	// DOC only says "the accumulator is truncated"; the code divides the signed 64 bit sum by the block size
	const int32_t a[2] = {-1, 0};
	const int32_t b[3] = {-4, -4, -3};  // sum -11, n 3: -3.67 -> -3
	q31_t r = 99;
	arm_mean_q31(a, 2, &r);
	CHECK(r == 0);
	arm_mean_q31(b, 3, &r);
	CHECK(r == -3);
}

// ---------------------------------------------------------------------------------------------------------------
// C3: power
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_power_q31] on random data of 19 block sizes the power equals the sum of floor(in^2 / 2^14) of the DOC") {
	const size_t sizes[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 16, 17, 33, 64, 100, 257, 1000, 4097};
	uint32_t seed = 17u;
	for (size_t n : sizes)
		for (int rep = 0; rep < 6; rep++) {
			seed = seed * 747796405u + 2891336453u;
			std::vector<int32_t> v = randomData(n, seed | 1u, rep % 2 == 0 ? 0 : 5);
			q63_t r = 0;
			arm_power_q31(v.data(), (uint32_t)n, &r);
			CAPTURE(n);
			CAPTURE(rep);
			CHECK(r == docPower(v.data(), n));
		}
}

TEST_CASE("[C3 arm_power_q31] the power in 16.48 is the exact sum of squares of the fractions floored: one sided error in (-n, 0] units") {
	// independent of the per-term DOC algorithm: S = sum in^2 exactly in 128 bit integers, Sq = floor(S / 2^14) = sum x^2 * 2^48
	// floored; sum of the floors is <= Sq and > S / 2^14 - n, i.e. >= Sq - n + 1 (a double sum would round at 2^54)
	const size_t sizes[] = {1, 2, 3, 5, 8, 11, 100, 1000};
	uint32_t seed = 4242u;
	for (size_t n : sizes) {
		seed = seed * 747796405u + 2891336453u;
		std::vector<int32_t> v = randomData(n, seed | 1u, 0);
		U128 S = 0;
		for (size_t i = 0; i < n; i++) S += (U128)((int64_t)v[i] * v[i]);
		const int64_t Sq = (int64_t)(S >> 14);
		q63_t r = 0;
		arm_power_q31(v.data(), (uint32_t)n, &r);
		CAPTURE(n);
		CHECK(r <= Sq);
		CHECK(r >= Sq - (int64_t)n + 1);
	}
}

TEST_CASE("[C3 arm_power_q31] samples that are multiples of 128 give exactly the sum of (in / 128)^2 (no truncation loss)") {
	// in = 128 k: in^2 = 2^14 k^2, divisible by 2^14; k^2 <= 2^48 and 30 terms are exact in double
	for (size_t n : {1, 2, 3, 4, 5, 9, 30}) {
		std::vector<int32_t> v = randomData(n, 5150u + (uint32_t)n, 0);
		double expect = 0.0;
		for (auto &e : v) {
			e = multipleOf128(e);
			expect += ((double)e / 128.0) * ((double)e / 128.0);
		}
		q63_t r = 0;
		arm_power_q31(v.data(), (uint32_t)n, &r);
		CAPTURE(n);
		CHECK((double)r == expect);
	}
}

TEST_CASE("[C3 arm_power_q31] zero data gives zero and full scale samples give n * 2^48 up to 32767 samples without overflow") {
	// DOC: "With 15 guard bits in the accumulator, there is no risk of overflow": n * 2^48 < 2^63 for n <= 32767
	std::vector<int32_t> zeros(9, 0);
	q63_t r = 123;
	arm_power_q31(zeros.data(), 9, &r);
	CHECK(r == 0);
	for (size_t n : {1u, 2u, 4u, 5u, 100u, 32767u}) {
		std::vector<int32_t> v(n, INT32_MIN);
		arm_power_q31(v.data(), (uint32_t)n, &r);
		CAPTURE(n);
		CHECK(r == (int64_t)n * ((int64_t)1 << 48));
	}
}

TEST_CASE("[C3 arm_power_q31] over the whole int32 range the power of single samples and sliding windows matches the DOC") {
	std::vector<int32_t> v = sweepData();
	size_t windows = 0;
	for (size_t i = 0; i + 9 <= v.size(); i++) {
		const size_t n = 1 + (i % 9);
		q63_t r = 0;
		arm_power_q31(&v[i], (uint32_t)n, &r);
		windows++;
		if (r != docPower(&v[i], n)) {
			FAIL_CHECK("window at " << i << " size " << n << ": power " << r << ", expected " << docPower(&v[i], n));
			return;
		}
	}
	CHECK(windows > 4000);
}

// ---------------------------------------------------------------------------------------------------------------
// C3: rms
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_rms_q31] on random data of 19 block sizes the rms is within the window of the truncated mean square and the 9 LSB of the square root") {
	const size_t sizes[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 16, 17, 33, 64, 100, 257, 1000, 4097};
	uint32_t seed = 271u;
	for (size_t n : sizes)
		for (int rep = 0; rep < 6; rep++) {
			seed = seed * 747796405u + 2891336453u;
			// scaled down by log2(n) bits as the DOC demands; rep 3..5 additionally by 6 / 12 / 20 bits (small rms stresses the truncation of the argument)
			std::vector<int32_t> v = randomData(n, seed | 1u, std::min(30, docShift(n) + (rep < 3 ? 0 : (rep == 3 ? 6 : (rep == 4 ? 12 : 20)))));
			q31_t r = 0;
			arm_rms_q31(v.data(), (uint32_t)n, &r);
			double lo, hi;
			rmsBounds(v.data(), n, lo, hi);
			CAPTURE(n);
			CAPTURE(rep);
			CHECK((double)r >= lo);
			CHECK((double)r <= hi);
		}
}

TEST_CASE("[C3 arm_rms_q31] the rms of equal samples is their magnitude within the window, for positive and negative samples") {
	const int32_t values[] = {0, 1, -1, 1000, -1000, 123456789, -123456789, 1073741824, -1073741824, INT32_MAX, INT32_MIN};
	const size_t sizes[] = {1, 2, 3};
	for (int32_t c : values)
		for (size_t n : sizes) {
			std::vector<int32_t> v(n, c);
			q31_t r = 0;
			arm_rms_q31(v.data(), (uint32_t)n, &r);
			double lo, hi;
			rmsBounds(v.data(), n, lo, hi);
			CAPTURE(c);
			CAPTURE(n);
			CHECK((double)r >= lo);
			CHECK((double)r <= hi);
		}
}

TEST_CASE("[C3 arm_rms_q31] over the whole int32 range the rms of sliding windows of 1 to 3 samples is within its window") {
	// windows of at most 3 samples: four full scale samples would wrap the uint64 accumulator (Lead decisions)
	std::vector<int32_t> v = sweepData();
	size_t windows = 0;
	for (size_t i = 0; i + 3 <= v.size(); i++) {
		const size_t n = 1 + (i % 3);
		q31_t r = 0;
		arm_rms_q31(&v[i], (uint32_t)n, &r);
		double lo, hi;
		rmsBounds(&v[i], n, lo, hi);
		windows++;
		if ((double)r < lo || (double)r > hi) {
			FAIL_CHECK("window at " << i << " size " << n << ": rms " << r << ", window [" << lo << ", " << hi << "]");
			return;
		}
	}
	CHECK(windows > 4000);
}

// ---------------------------------------------------------------------------------------------------------------
// C3: var and std
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_var_q31] for inputs that are multiples of 256 the unbiased variance is within the window (D - 1.001, D + 0.001] of the DOC") {
	const size_t sizes[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 16, 17, 33, 64, 100, 257};
	uint32_t seed = 3131u;
	for (size_t n : sizes)
		for (int rep = 0; rep < 6; rep++) {
			seed = seed * 747796405u + 2891336453u;
			std::vector<int32_t> v = randomData(n, seed | 1u, rep < 3 ? 1 : 4);
			for (auto &e : v) e = multipleOf256(e);
			q31_t r = 0;
			arm_var_q31(v.data(), (uint32_t)n, &r);
			double lo, hi;
			varWindow(v.data(), n, true, lo, hi);
			CAPTURE(n);
			CAPTURE(rep);
			CHECK((double)r > lo);
			CHECK((double)r <= hi);
		}
}

TEST_CASE("[C3 arm_var_q31] for arbitrary inputs the variance is within 363 sd + 2 LSB of the unbiased variance (input quantisation of the >> 8)") {
	const size_t sizes[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 16, 17, 33, 64, 100, 257, 1000};
	uint32_t seed = 3232u;
	for (size_t n : sizes)
		for (int rep = 0; rep < 6; rep++) {
			seed = seed * 747796405u + 2891336453u;
			std::vector<int32_t> v = randomData(n, seed | 1u, rep < 3 ? 1 : 3);
			q31_t r = 0;
			arm_var_q31(v.data(), (uint32_t)n, &r);
			double lo, hi;
			varWindow(v.data(), n, false, lo, hi);
			CAPTURE(n);
			CAPTURE(rep);
			CHECK((double)r >= lo);
			CHECK((double)r <= hi);
		}
}

TEST_CASE("[C3 arm_var_q31] the variance of equal samples is exactly 0 (both truncated terms are the same quotient)") {
	// sumsq = n c^2, sum = n c: mos = trunc(n c^2 / (n - 1)), som = trunc(n^2 c^2 / (n (n - 1))) = the same rational
	const int32_t values[] = {0, 1, -1, 100000, -100000, 987654321, -987654321, INT32_MAX, INT32_MIN};
	const size_t sizes[] = {2, 3, 4, 5, 9, 100, 4097};
	for (int32_t c : values)
		for (size_t n : sizes) {
			// DOC: scale down by log2(n) - 8 bits for large n (sum^2 needs n^2 2^46 < 2^63, i.e. n < 363 at full scale)
			std::vector<int32_t> v(n, n > 300 ? c >> 4 : c);
			q31_t r = 99;
			arm_var_q31(v.data(), (uint32_t)n, &r);
			CAPTURE(c);
			CAPTURE(n);
			CHECK(r == 0);
		}
}

TEST_CASE("[C3 arm_var_q31] two samples a, b give (a - b)^2 / 2 (the n - 1 divisor, not n) over a grid within +-0.5 of full scale") {
	// V = (a - b)^2 / 2 for n = 2 (unbiased); the biased estimator would give half of it
	for (int i = 0; i < 24; i++)
		for (int j = 0; j < 24; j++) {
			const int32_t pair[2] = {multipleOf256(-1073741824 + i * 89478485), multipleOf256(1073741823 - j * 89478480)};
			q31_t r = 0;
			arm_var_q31(pair, 2, &r);
			const double d = ((double)pair[0] - (double)pair[1]) / kTwo31;
			const double D = d * d / 2.0 * kTwo31;
			if (!((double)r > D - 1.001 && (double)r <= D + 0.001)) {
				FAIL_CHECK("pair " << pair[0] << ", " << pair[1] << ": var " << r << ", expected " << D);
				return;
			}
		}
}

TEST_CASE("[C3 arm_var_q31] over the whole range halved (+-0.5) the variance of sliding windows of 2 to 9 samples is within its window") {
	std::vector<int32_t> v = sweepData();
	for (auto &e : v) e >>= 1;
	size_t windows = 0;
	for (size_t i = 0; i + 9 <= v.size(); i++) {
		const size_t n = 2 + (i % 8);
		q31_t r = 0;
		arm_var_q31(&v[i], (uint32_t)n, &r);
		double lo, hi;
		varWindow(&v[i], n, false, lo, hi);
		windows++;
		if ((double)r < lo || (double)r > hi) {
			FAIL_CHECK("window at " << i << " size " << n << ": var " << r << ", window [" << lo << ", " << hi << "]");
			return;
		}
	}
	CHECK(windows > 4000);
}

TEST_CASE("[C3 arm_std_q31] for inputs that are multiples of 256 the standard deviation is within the window of its argument and of the 9 LSB of arm_sqrt_q31") {
	const size_t sizes[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 16, 17, 33, 64, 100, 257};
	uint32_t seed = 3333u;
	for (size_t n : sizes)
		for (int rep = 0; rep < 6; rep++) {
			seed = seed * 747796405u + 2891336453u;
			std::vector<int32_t> v = randomData(n, seed | 1u, rep < 3 ? 1 : 6);
			for (auto &e : v) e = multipleOf256(e);
			q31_t r = 0;
			arm_std_q31(v.data(), (uint32_t)n, &r);
			double vlo, vhi, lo, hi;
			varWindow(v.data(), n, true, vlo, vhi);
			stdWindow(vlo, vhi, lo, hi);
			CAPTURE(n);
			CAPTURE(rep);
			CHECK((double)r >= lo);
			CHECK((double)r <= hi);
		}
}

TEST_CASE("[C3 arm_std_q31] for arbitrary inputs the standard deviation is within the window of its argument and of the 9 LSB of arm_sqrt_q31") {
	const size_t sizes[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 16, 17, 33, 64, 100, 257, 1000};
	uint32_t seed = 3434u;
	for (size_t n : sizes)
		for (int rep = 0; rep < 6; rep++) {
			seed = seed * 747796405u + 2891336453u;
			std::vector<int32_t> v = randomData(n, seed | 1u, rep < 3 ? 1 : 5);
			q31_t r = 0;
			arm_std_q31(v.data(), (uint32_t)n, &r);
			double vlo, vhi, lo, hi;
			varWindow(v.data(), n, false, vlo, vhi);
			stdWindow(vlo, vhi, lo, hi);
			CAPTURE(n);
			CAPTURE(rep);
			CHECK((double)r >= lo);
			CHECK((double)r <= hi);
		}
}

TEST_CASE("[C3 arm_std_q31] the standard deviation of equal samples is exactly 0") {
	const int32_t values[] = {0, 1, -1, 100000, -987654321, INT32_MAX, INT32_MIN};
	const size_t sizes[] = {2, 3, 4, 5, 9, 100};
	for (int32_t c : values)
		for (size_t n : sizes) {
			std::vector<int32_t> v(n, c);
			q31_t r = 99;
			arm_std_q31(v.data(), (uint32_t)n, &r);
			CAPTURE(c);
			CAPTURE(n);
			CHECK(r == 0);
		}
}

TEST_CASE("[C3 arm_std_q31] over the whole range halved (+-0.5) the standard deviation of sliding windows of 2 to 9 samples is within its window") {
	std::vector<int32_t> v = sweepData();
	for (auto &e : v) e >>= 1;
	size_t windows = 0;
	for (size_t i = 0; i + 9 <= v.size(); i++) {
		const size_t n = 2 + (i % 8);
		q31_t r = 0;
		arm_std_q31(&v[i], (uint32_t)n, &r);
		double vlo, vhi, lo, hi;
		varWindow(&v[i], n, false, vlo, vhi);
		stdWindow(vlo, vhi, lo, hi);
		windows++;
		if ((double)r < lo || (double)r > hi) {
			FAIL_CHECK("window at " << i << " size " << n << ": std " << r << ", window [" << lo << ", " << hi << "]");
			return;
		}
	}
	CHECK(windows > 4000);
}

// ---------------------------------------------------------------------------------------------------------------
// C3: mse
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_mse_q31] on random data of 19 block sizes the mean square error is within the one-sided window of the definition") {
	const size_t sizes[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 16, 17, 33, 64, 100, 257, 1000, 4097};
	uint32_t seed = 515u;
	for (size_t n : sizes)
		for (int rep = 0; rep < 6; rep++) {
			seed = seed * 747796405u + 2891336453u;
			const uint32_t sa = seed | 1u;
			seed = seed * 747796405u + 2891336453u;
			const uint32_t sb = seed | 1u;
			const int shift = rep < 3 ? 1 : 4;  // +-0.5 so |a - b| < 1 and the result is representable
			std::vector<int32_t> a = randomData(n, sa, shift), b = randomData(n, sb, shift);
			q31_t r = 0;
			arm_mse_q31(a.data(), b.data(), (uint32_t)n, &r);
			CAPTURE(n);
			CAPTURE(rep);
			CHECK(std::fabs((double)r - defMse(a.data(), b.data(), n)) <= mseBound(a.data(), b.data(), n));
		}
}

TEST_CASE("[C3 arm_mse_q31] identical vectors give exactly 0 and the order of the vectors does not change the result") {
	const size_t sizes[] = {1, 2, 3, 4, 5, 9, 100};
	for (size_t n : sizes) {
		std::vector<int32_t> a = randomData(n, 600u + (uint32_t)n, 0), b = randomData(n, 700u + (uint32_t)n, 1);
		q31_t r = 99;
		arm_mse_q31(a.data(), a.data(), (uint32_t)n, &r);
		CAPTURE(n);
		CHECK(r == 0);
		q31_t ab = 0, ba = 0;
		arm_mse_q31(a.data(), b.data(), (uint32_t)n, &ab);
		arm_mse_q31(b.data(), a.data(), (uint32_t)n, &ba);
		CHECK(ab == ba);
	}
}

TEST_CASE("[C3 arm_mse_q31] over the whole range halved (+-0.5) the mean square error of sliding windows of 1 to 9 samples is within its bound") {
	std::vector<int32_t> a = sweepData();
	for (auto &e : a) e >>= 1;
	std::vector<int32_t> b(a.size());
	for (size_t i = 0; i < a.size(); i++) b[i] = a[(i * 7 + 3) % a.size()];  // a different order of the same values
	size_t windows = 0;
	for (size_t i = 0; i + 9 <= a.size(); i++) {
		const size_t n = 1 + (i % 9);
		q31_t r = 0;
		arm_mse_q31(&a[i], &b[i], (uint32_t)n, &r);
		windows++;
		if (!mseInWindow((double)r, &a[i], &b[i], n)) {
			FAIL_CHECK("window at " << i << " size " << n << ": mse " << r << ", expected " << defMse(&a[i], &b[i], n));
			return;
		}
	}
	CHECK(windows > 4000);
}

// ---------------------------------------------------------------------------------------------------------------
// C4: block sizes 1 and 2, all-equal data, extremes, documented edges
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C4 arm_max_q31] a single sample is returned with index 0") {
	for (int32_t x : {0, 1, -1, INT32_MAX, INT32_MIN}) {
		q31_t r = 99;
		uint32_t idx = 99;
		arm_max_q31(&x, 1, &r, &idx);
		CHECK(r == x);
		CHECK(idx == 0u);
	}
}

TEST_CASE("[C4 arm_min_q31] a single sample is returned with index 0") {
	for (int32_t x : {0, 1, -1, INT32_MAX, INT32_MIN}) {
		q31_t r = 99;
		uint32_t idx = 99;
		arm_min_q31(&x, 1, &r, &idx);
		CHECK(r == x);
		CHECK(idx == 0u);
	}
}

TEST_CASE("[C4 arm_max_q31] two samples give the larger one with its index in both orders, INT32_MAX against INT32_MIN included") {
	const int32_t a[2] = {INT32_MIN, INT32_MAX}, b[2] = {INT32_MAX, INT32_MIN}, c[2] = {-3, -4};
	q31_t r = 0;
	uint32_t idx = 99;
	arm_max_q31(a, 2, &r, &idx);
	CHECK(r == INT32_MAX);
	CHECK(idx == 1u);
	arm_max_q31(b, 2, &r, &idx);
	CHECK(r == INT32_MAX);
	CHECK(idx == 0u);
	arm_max_q31(c, 2, &r, &idx);
	CHECK(r == -3);
	CHECK(idx == 0u);
}

TEST_CASE("[C4 arm_min_q31] two samples give the smaller one with its index in both orders, INT32_MAX against INT32_MIN included") {
	const int32_t a[2] = {INT32_MIN, INT32_MAX}, b[2] = {INT32_MAX, INT32_MIN}, c[2] = {-3, -4};
	q31_t r = 0;
	uint32_t idx = 99;
	arm_min_q31(a, 2, &r, &idx);
	CHECK(r == INT32_MIN);
	CHECK(idx == 0u);
	arm_min_q31(b, 2, &r, &idx);
	CHECK(r == INT32_MIN);
	CHECK(idx == 1u);
	arm_min_q31(c, 2, &r, &idx);
	CHECK(r == -4);
	CHECK(idx == 1u);
}

TEST_CASE("[C4 arm_max_q31] characterization: when all samples are equal the index is 0 for every block size") {
	for (uint32_t n = 1; n <= 13; n++) {
		std::vector<int32_t> v(n, -42);
		q31_t r = 0;
		uint32_t idx = 99;
		arm_max_q31(v.data(), n, &r, &idx);
		CAPTURE(n);
		CHECK(r == -42);
		CHECK(idx == 0u);
	}
}

TEST_CASE("[C4 arm_min_q31] characterization: when all samples are equal the index is 0 for every block size") {
	for (uint32_t n = 1; n <= 13; n++) {
		std::vector<int32_t> v(n, 42);
		q31_t r = 0;
		uint32_t idx = 99;
		arm_min_q31(v.data(), n, &r, &idx);
		CAPTURE(n);
		CHECK(r == 42);
		CHECK(idx == 0u);
	}
}

TEST_CASE("[C4 arm_absmax_q31] a single sample returns its magnitude with index 0, INT32_MIN saturates to INT32_MAX") {
	// the saturation is backed by ARM's saturation id (AbsMaxVals8 offset 3 = 0x7FFFFFFF)
	const int32_t in[] = {0, 5, -5, INT32_MAX, INT32_MIN, INT32_MIN + 1};
	const int32_t expect[] = {0, 5, 5, INT32_MAX, INT32_MAX, INT32_MAX};
	for (size_t i = 0; i < 6; i++) {
		q31_t r = 99;
		uint32_t idx = 99;
		arm_absmax_q31(&in[i], 1, &r, &idx);
		CAPTURE(in[i]);
		CHECK(r == expect[i]);
		CHECK(idx == 0u);
	}
}

TEST_CASE("[C4 arm_absmax_no_idx_q31] a single sample returns its magnitude, INT32_MIN saturates to INT32_MAX") {
	const int32_t in[] = {0, 5, -5, INT32_MAX, INT32_MIN, INT32_MIN + 1};
	const int32_t expect[] = {0, 5, 5, INT32_MAX, INT32_MAX, INT32_MAX};
	for (size_t i = 0; i < 6; i++) {
		q31_t r = 99;
		arm_absmax_no_idx_q31(&in[i], 1, &r);
		CAPTURE(in[i]);
		CHECK(r == expect[i]);
	}
}

TEST_CASE("[C4 arm_absmax_q31] characterization: INT32_MIN and INT32_MAX in one block both have magnitude INT32_MAX and the first one wins") {
	const int32_t a[3] = {5, INT32_MIN, INT32_MAX}, b[3] = {5, INT32_MAX, INT32_MIN};
	q31_t r = 0;
	uint32_t idx = 99;
	arm_absmax_q31(a, 3, &r, &idx);
	CHECK(r == INT32_MAX);
	CHECK(idx == 1u);
	arm_absmax_q31(b, 3, &r, &idx);
	CHECK(r == INT32_MAX);
	CHECK(idx == 1u);
}

TEST_CASE("[C4 arm_absmin_q31] a single sample returns its magnitude with index 0, two samples give the smaller magnitude in both orders") {
	const int32_t in[] = {0, 5, -5, INT32_MAX, INT32_MIN + 1};
	const int32_t expect[] = {0, 5, 5, INT32_MAX, INT32_MAX};
	for (size_t i = 0; i < 5; i++) {
		q31_t r = 99;
		uint32_t idx = 99;
		arm_absmin_q31(&in[i], 1, &r, &idx);
		CAPTURE(in[i]);
		CHECK(r == expect[i]);
		CHECK(idx == 0u);
	}
	const int32_t a[2] = {-7, 3}, b[2] = {3, -7};
	q31_t r = 0;
	uint32_t idx = 99;
	arm_absmin_q31(a, 2, &r, &idx);
	CHECK(r == 3);
	CHECK(idx == 1u);
	arm_absmin_q31(b, 2, &r, &idx);
	CHECK(r == 3);
	CHECK(idx == 0u);
}

TEST_CASE("[C4 arm_absmin_q31] characterization: INT32_MIN counts as magnitude INT32_MAX (saturating abs), as ARM's saturation test shows for absmax") {
	// DOC of the absmin functions is silent about INT32_MIN; the code is the same saturating abs as arm_absmax_q31
	const int32_t a[2] = {INT32_MIN, INT32_MAX}, b[1] = {INT32_MIN};
	q31_t r = 0;
	uint32_t idx = 99;
	arm_absmin_q31(b, 1, &r, &idx);
	CHECK(r == INT32_MAX);
	CHECK(idx == 0u);
	arm_absmin_q31(a, 2, &r, &idx);
	CHECK(r == INT32_MAX);
	CHECK(idx == 0u);
	arm_absmin_no_idx_q31(b, 1, &r);
	CHECK(r == INT32_MAX);
	const int32_t c[3] = {INT32_MIN, 7, INT32_MIN};
	arm_absmin_no_idx_q31(c, 3, &r);
	CHECK(r == 7);
}

TEST_CASE("[C4 arm_absmin_q31] INT32_MIN at positions 1 and 2 counts as magnitude INT32_MAX: it does not become the minimum and the first index rule holds") {
	// {INT32_MAX, INT32_MIN}: both magnitudes INT32_MAX, first wins (index 0); {INT32_MIN, INT32_MIN, INT32_MAX}: index 0;
	// {INT32_MAX, INT32_MIN, INT32_MIN}: index 0; {5, INT32_MIN, 9}: 5 at index 0; {INT32_MIN, 9, INT32_MIN}: 9 at index 1.
	// A non-saturated abs (INT32_MIN stays negative or wraps) would give a magnitude below INT32_MAX here.
	const int32_t a[2] = {INT32_MAX, INT32_MIN}, b[3] = {INT32_MIN, INT32_MIN, INT32_MAX}, c[3] = {INT32_MAX, INT32_MIN, INT32_MIN};
	const int32_t d[3] = {5, INT32_MIN, 9}, e[3] = {INT32_MIN, 9, INT32_MIN};
	q31_t r = 0;
	uint32_t idx = 99;
	arm_absmin_q31(a, 2, &r, &idx);
	CHECK(r == INT32_MAX);
	CHECK(idx == 0u);
	arm_absmin_q31(b, 3, &r, &idx);
	CHECK(r == INT32_MAX);
	CHECK(idx == 0u);
	arm_absmin_q31(c, 3, &r, &idx);
	CHECK(r == INT32_MAX);
	CHECK(idx == 0u);
	arm_absmin_q31(d, 3, &r, &idx);
	CHECK(r == 5);
	CHECK(idx == 0u);
	arm_absmin_q31(e, 3, &r, &idx);
	CHECK(r == 9);
	CHECK(idx == 1u);
}

TEST_CASE("[C4 arm_absmin_no_idx_q31] INT32_MIN at positions 1 and 2 counts as magnitude INT32_MAX and does not become the minimum") {
	const int32_t a[2] = {INT32_MAX, INT32_MIN}, b[3] = {INT32_MIN, INT32_MIN, INT32_MAX}, c[3] = {INT32_MAX, INT32_MIN, INT32_MIN};
	const int32_t d[3] = {5, INT32_MIN, 9}, e[3] = {INT32_MIN, 9, INT32_MIN};
	q31_t r = 0;
	arm_absmin_no_idx_q31(a, 2, &r);
	CHECK(r == INT32_MAX);
	arm_absmin_no_idx_q31(b, 3, &r);
	CHECK(r == INT32_MAX);
	arm_absmin_no_idx_q31(c, 3, &r);
	CHECK(r == INT32_MAX);
	arm_absmin_no_idx_q31(d, 3, &r);
	CHECK(r == 5);
	arm_absmin_no_idx_q31(e, 3, &r);
	CHECK(r == 9);
}

TEST_CASE("[C4 arm_max_no_idx_q31] a single sample and the two orders of two samples give the maximum, extremes included") {
	for (int32_t x : {0, 1, -1, INT32_MAX, INT32_MIN}) {
		q31_t r = 99;
		arm_max_no_idx_q31(&x, 1, &r);
		CHECK(r == x);
	}
	const int32_t a[2] = {INT32_MIN, INT32_MAX}, b[2] = {INT32_MAX, INT32_MIN};
	q31_t r = 0;
	arm_max_no_idx_q31(a, 2, &r);
	CHECK(r == INT32_MAX);
	arm_max_no_idx_q31(b, 2, &r);
	CHECK(r == INT32_MAX);
}

TEST_CASE("[C4 arm_min_no_idx_q31] a single sample and the two orders of two samples give the minimum, extremes included") {
	for (int32_t x : {0, 1, -1, INT32_MAX, INT32_MIN}) {
		q31_t r = 99;
		arm_min_no_idx_q31(&x, 1, &r);
		CHECK(r == x);
	}
	const int32_t a[2] = {INT32_MIN, INT32_MAX}, b[2] = {INT32_MAX, INT32_MIN};
	q31_t r = 0;
	arm_min_no_idx_q31(a, 2, &r);
	CHECK(r == INT32_MIN);
	arm_min_no_idx_q31(b, 2, &r);
	CHECK(r == INT32_MIN);
}

TEST_CASE("[C4 arm_mean_q31] block sizes 1 and 2 give the sample and the truncated average of two samples") {
	const int32_t one[1] = {-123456};
	q31_t r = 0;
	arm_mean_q31(one, 1, &r);
	CHECK(r == -123456);
	const int32_t two[2] = {INT32_MAX, INT32_MAX - 1};  // sum 2^32 - 3, mean 2^31 - 1.5 -> 2147483646
	arm_mean_q31(two, 2, &r);
	CHECK(r == INT32_MAX - 1);
	const int32_t ext[2] = {INT32_MAX, INT32_MIN};  // sum -1, mean -0.5: within 1 of the exact mean
	arm_mean_q31(ext, 2, &r);
	CHECK(std::fabs((double)r + 0.5) < 1.0);
}

TEST_CASE("[C4 arm_power_q31] a single sample gives floor(in^2 / 2^14), at the extremes 2^48") {
	const int32_t in[] = {0, 1, -1, 128, -128, 16384, INT32_MAX, INT32_MIN};
	for (size_t i = 0; i < 8; i++) {
		q63_t r = 99;
		arm_power_q31(&in[i], 1, &r);
		CAPTURE(in[i]);
		CHECK(r == (((int64_t)in[i] * in[i]) >> 14));
	}
	// hand derivation: INT32_MIN^2 = 2^62, >> 14 = 2^48; INT32_MAX^2 = 2^62 - 2^32 + 1, >> 14 = 2^48 - 2^18
	q63_t r = 0;
	const int32_t mn = INT32_MIN, mx = INT32_MAX;
	arm_power_q31(&mn, 1, &r);
	CHECK(r == (int64_t)1 << 48);
	arm_power_q31(&mx, 1, &r);
	CHECK(r == ((int64_t)1 << 48) - ((int64_t)1 << 18));
}

TEST_CASE("[C4 arm_rms_q31] full scale samples (INT32_MIN, 1, 2 and 3 of them) give the saturated 1.0 within the 9 LSB of the square root") {
	// mean square 1.0 is not representable: clip_q63_to_q31 gives 0x7FFFFFFF; the true sqrt(1 - 2^-31) * 2^31 = 2^31 - 0.5
	for (size_t n : {1u, 2u, 3u}) {
		std::vector<int32_t> v(n, INT32_MIN);
		q31_t r = 0;
		arm_rms_q31(v.data(), (uint32_t)n, &r);
		CAPTURE(n);
		CHECK((double)r >= 2147483647.0 - kSqrtLsb);
		CHECK(r <= INT32_MAX);
	}
}

TEST_CASE("[C4 arm_rms_q31] the rms of zeros is 0") {
	std::vector<int32_t> v(7, 0);
	q31_t r = 99;
	arm_rms_q31(v.data(), 7, &r);
	CHECK(r == 0);
}

TEST_CASE("[C4 arm_var_q31] characterization: with block size 1 or 0 the variance is set to 0 (no division by n - 1 = 0)") {
	// the code returns 0 for blockSize <= 1 (arm_var_q31.c:143); DOC is silent about this case
	const int32_t one[1] = {INT32_MAX};
	q31_t r = 99;
	arm_var_q31(one, 1, &r);
	CHECK(r == 0);
	r = 99;
	arm_var_q31(one, 0, &r);
	CHECK(r == 0);
}

TEST_CASE("[C4 arm_std_q31] characterization: with block size 1 or 0 the standard deviation is set to 0 (no division by n - 1 = 0)") {
	const int32_t one[1] = {INT32_MAX};
	q31_t r = 99;
	arm_std_q31(one, 1, &r);
	CHECK(r == 0);
	r = 99;
	arm_std_q31(one, 0, &r);
	CHECK(r == 0);
}

TEST_CASE("[C4 arm_var_q31] two samples at +0.5 and -0.5 give the variance (a - b)^2 / 2 = 0.5") {
	// a = 0.5 (0x40000000), b = -0.5 (0xC0000000): V = 1 / 2 = 0.5 = 0x40000000 exactly (both multiples of 256)
	const int32_t pair[2] = {0x40000000, (int32_t)0xC0000000};
	q31_t r = 0;
	arm_var_q31(pair, 2, &r);
	CHECK((double)r > 1073741824.0 - 1.001);
	CHECK((double)r <= 1073741824.0 + 0.001);
}

TEST_CASE("[C4 arm_var_q31] inputs of the form 256k + 255 are truncated by the >> 8 (not rounded): {0x400000FF, -0x40000000} gives exactly 0x40000000") {
	// hand derivation: q = in >> 8 (arithmetic): 0x400000FF >> 8 = 0x400000 = 2^22, -0x40000000 >> 8 = -2^22; sum 0,
	// sumsq 2 * 2^44 = 2^45, n - 1 = 1: mos = 2^45, som = 0, 2^45 >> 15 = 2^30 = 0x40000000.
	// Rounding ((in + 128) >> 8) would give q = 2^22 + 1 and 0x40000100; round half up of 255 / 256 as well
	const int32_t a[2] = {0x400000FF, -0x40000000};
	q31_t r = 0;
	arm_var_q31(a, 2, &r);
	CHECK(r == 0x40000000);
	// negative input of the form -2^30 + 255: floor division gives q = -2^22 again (toward zero would give -2^22 + 1)
	const int32_t b[2] = {0x40000000, -0x40000000 + 255};
	arm_var_q31(b, 2, &r);
	CHECK(r == 0x40000000);
	// {255, 0}: both truncate to q = 0, variance 0
	const int32_t c[2] = {255, 0};
	arm_var_q31(c, 2, &r);
	CHECK(r == 0);
}

TEST_CASE("[C4 arm_std_q31] inputs of the form 256k + 255 are truncated by the >> 8 (not rounded): {0x400000FF, -0x40000000} gives sqrt(0.5)") {
	// the argument of arm_sqrt_q31 is exactly 0x40000000 (see the var test); sqrt(0.5) * 2^31 = 1518500249.98;
	// rounding the inputs would move the argument by 256 and the result by about 181 LSB
	const int32_t a[2] = {0x400000FF, -0x40000000}, b[2] = {0x40000000, -0x40000000 + 255};
	q31_t r = 0;
	arm_std_q31(a, 2, &r);
	CHECK(std::fabs((double)r - std::sqrt(0.5) * kTwo31) <= kSqrtLsb);
	arm_std_q31(b, 2, &r);
	CHECK(std::fabs((double)r - std::sqrt(0.5) * kTwo31) <= kSqrtLsb);
	const int32_t c[2] = {255, 0};
	arm_std_q31(c, 2, &r);
	CHECK(r == 0);
}

TEST_CASE("[C4 arm_var_q31] a block of 65537 samples gives the variance of the definition" * KNOWN_ISSUE("B-DSQ31-3: arm_var_q31.c:208 computes blockSize * (blockSize - 1U) in uint32, which wraps for n > 65536 (65537 * 65536 = 2^32 + 2^16 -> 2^16); the square of the mean is then 65537 times too large and the variance of this DOC-compliant input comes out as 0 instead of about 512 LSB (reviewer probe with another signal: negative values)")) {
	// one sample 0.125 (0x10000000, a multiple of 256), 65536 zeros: V = c^2 / n (k = 1 in k c^2 (n - k) / (n (n - 1))),
	// c = 2^-3: 2^-6 / 65537 = 2^-22 / 1.0000153 -> 511.992 LSB; window of the exact inputs. DOC: scaled down enough
	// (sum = 2^20, sum^2 = 2^40, no overflow of the 64 bit accumulators)
	std::vector<int32_t> v(65537, 0);
	v[0] = 0x10000000;
	q31_t r = 99;
	arm_var_q31(v.data(), 65537, &r);
	double lo, hi;
	varWindow(v.data(), v.size(), true, lo, hi);
	CHECK((double)r > lo);
	CHECK((double)r <= hi);
}

TEST_CASE("[C4 arm_std_q31] a block of 65537 samples gives the standard deviation of the definition" * KNOWN_ISSUE("B-DSQ31-3: arm_std_q31.c:152 computes blockSize * (blockSize - 1U) in uint32, which wraps for n > 65536; the argument of arm_sqrt_q31 becomes 0 instead of about 512 and the result 0 instead of about 1.05e6")) {
	std::vector<int32_t> v(65537, 0);
	v[0] = 0x10000000;
	q31_t r = 99;
	arm_std_q31(v.data(), 65537, &r);
	double vlo, vhi, lo, hi;
	varWindow(v.data(), v.size(), true, vlo, vhi);
	stdWindow(vlo, vhi, lo, hi);
	CHECK((double)r >= lo);
	CHECK((double)r <= hi);
}

TEST_CASE("[C4 arm_mse_q31] a single pair gives (a - b)^2 as a q31 number, e.g. 0.5 against -0.25 gives 0.5625") {
	// hand derivation: a = 0x40000000 (0.5), b = 0xE0000000 (-0.25): a >> 1 = 0x20000000, b >> 1 = 0xF0000000 (-0x10000000),
	// diff = 0x30000000, diff^2 = 0x09 << 56 >> 14 = 9 * 2^54 / 2^14... evaluated in double: 0.75^2 = 0.5625
	const int32_t a[1] = {0x40000000}, b[1] = {(int32_t)0xE0000000};
	q31_t r = 0;
	arm_mse_q31(a, b, 1, &r);
	CHECK(mseInWindow((double)r, a, b, 1));
	CHECK(std::fabs(defMse(a, b, 1) - 0.5625 * kTwo31) < 1e-6);  // the definition itself is the hand value
}

TEST_CASE("[C4 arm_mse_q31] the same vector twice gives 0 also for the extreme samples") {
	const int32_t v[2] = {INT32_MIN, INT32_MAX};
	q31_t r = 99;
	arm_mse_q31(v, v, 2, &r);
	CHECK(r == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// C5: against the f32 functions
// ---------------------------------------------------------------------------------------------------------------

namespace {
std::vector<float> toFloat(const std::vector<int32_t> &v) {
	std::vector<float> f(v.size());
	for (size_t i = 0; i < v.size(); i++) f[i] = (float)v[i] / 2147483648.0f;  // int -> float rounds once (u), /2^31 exact
	return f;
}
double meanAbsFraction(const std::vector<int32_t> &v) {
	double s = 0.0;
	for (int32_t e : v) s += std::fabs((double)e) / kTwo31;
	return s / (double)v.size();
}
double sumSquaresFraction(const std::vector<int32_t> &v) {
	double s = 0.0;
	for (int32_t e : v) s += ((double)e / kTwo31) * ((double)e / kTwo31);
	return s;
}
const size_t kC5Sizes[] = {2, 5, 8, 16, 33};
}  // namespace

TEST_CASE("[C5 arm_mean_q31] the mean agrees with arm_mean_f32 of the same samples within (n + 1) u mean|x| + 1 LSB") {
	// f32: input conversion u|x_i|, n - 1 sequential additions, one division: (n + 1) u of mean|x| (u = 2^-24); q31: < 1 LSB
	for (size_t n : kC5Sizes) {
		std::vector<int32_t> v = randomData(n, 9100u + (uint32_t)n, 0);
		std::vector<float> f = toFloat(v);
		float fm = 0.0f;
		arm_mean_f32(f.data(), (uint32_t)n, &fm);
		q31_t r = 0;
		arm_mean_q31(v.data(), (uint32_t)n, &r);
		const double bound = ((double)n + 1.0) * kU * meanAbsFraction(v) * kTwo31 + 1.0;
		CAPTURE(n);
		CHECK(std::fabs((double)r - (double)fm * kTwo31) <= bound);
	}
}

TEST_CASE("[C5 arm_power_q31] the power agrees with arm_power_f32 of the same samples within (n + 2) u sum x^2 + n 2^-48") {
	// f32: conversion 2u (squared), product u, n - 1 additions: (n + 2) u of sum x^2; q31 result in 16.48 loses < n units of 2^-48
	for (size_t n : kC5Sizes) {
		std::vector<int32_t> v = randomData(n, 9200u + (uint32_t)n, 0);
		std::vector<float> f = toFloat(v);
		float fp = 0.0f;
		arm_power_f32(f.data(), (uint32_t)n, &fp);
		q63_t r = 0;
		arm_power_q31(v.data(), (uint32_t)n, &r);
		const double bound = ((double)n + 2.0) * kU * sumSquaresFraction(v) + (double)n / 281474976710656.0;
		CAPTURE(n);
		CHECK(std::fabs((double)r / 281474976710656.0 - (double)fp) <= bound);
	}
}

TEST_CASE("[C5 arm_rms_q31] the rms agrees with arm_rms_f32 of the same samples within (n + 5) u / 2 of the rms plus the q31 window") {
	// f32: sum (n + 2) u, division u, square root halves the relative error and adds u: (n + 5) u / 2; q31: rmsBounds window
	for (size_t n : kC5Sizes) {
		std::vector<int32_t> v = randomData(n, 9300u + (uint32_t)n, docShift(n));
		std::vector<float> f = toFloat(v);
		float fr = 0.0f;
		arm_rms_f32(f.data(), (uint32_t)n, &fr);
		q31_t r = 0;
		arm_rms_q31(v.data(), (uint32_t)n, &r);
		double lo, hi;
		rmsBounds(v.data(), n, lo, hi);
		const double ideal = std::sqrt(std::min(meanSquareLsb(v.data(), n), 2147483647.0) * kTwo31);
		const double qErr = std::max(hi - ideal, ideal - lo);
		const double bound = (double)fr * kTwo31 * ((double)n + 5.0) * kU / 2.0 + qErr;
		CAPTURE(n);
		CHECK(std::fabs((double)r - (double)fr * kTwo31) <= bound);
	}
}

TEST_CASE("[C5 arm_var_q31] the variance agrees with arm_var_f32 of the same samples within the f32 error plus 363 sd + 2 LSB") {
	// arm_var_f32 is two pass: mean, then sum (x - mean)^2 / (n - 1). Error (V = var, Q = sum x^2 / (n - 1)): relative
	// (n + 4) u on V (sums, subtraction, products), input conversion 2 u sum |x - m||x| / (n - 1) <= 2 u sqrt(V Q)
	// (Cauchy-Schwarz), the error of the mean enters squared: 2 n / (n - 1) ((n + 1) u mean|x|)^2.
	for (size_t n : kC5Sizes) {
		std::vector<int32_t> v = randomData(n, 9400u + (uint32_t)n, 1);
		std::vector<float> f = toFloat(v);
		float fv = 0.0f;
		arm_var_f32(f.data(), (uint32_t)n, &fv);
		q31_t r = 0;
		arm_var_q31(v.data(), (uint32_t)n, &r);
		const double V = defVar(v.data(), n), Q = sumSquaresFraction(v) / (double)(n - 1);
		const double m = ((double)n + 1.0) * kU * meanAbsFraction(v);
		const double fErr = kU * (((double)n + 4.0) * V + 2.0 * std::sqrt(V * Q)) + 2.0 * (double)n / (double)(n - 1) * m * m;
		const double bound = fErr * kTwo31 + 363.0 * std::sqrt(V) + 2.0;
		CAPTURE(n);
		CHECK(std::fabs((double)r - (double)fv * kTwo31) <= bound);
	}
}

TEST_CASE("[C5 arm_std_q31] the standard deviation agrees with arm_std_f32 of the same samples within the propagated f32 error plus the q31 window") {
	// f32 variance error B (as in the var test) and sqrt rounding u sd; sqrt is monotone: the f32 std lies in
	// [sqrt(max(V - B, 0)), sqrt(V + B)] (1 + u); q31: stdWindow
	for (size_t n : kC5Sizes) {
		std::vector<int32_t> v = randomData(n, 9500u + (uint32_t)n, 1);
		std::vector<float> f = toFloat(v);
		float fs = 0.0f;
		arm_std_f32(f.data(), (uint32_t)n, &fs);
		q31_t r = 0;
		arm_std_q31(v.data(), (uint32_t)n, &r);
		const double V = defVar(v.data(), n), Q = sumSquaresFraction(v) / (double)(n - 1);
		const double m = ((double)n + 1.0) * kU * meanAbsFraction(v);
		const double B = kU * (((double)n + 4.0) * V + 2.0 * std::sqrt(V * Q)) + 2.0 * (double)n / (double)(n - 1) * m * m;
		const double fLo = std::sqrt(std::max(V - B, 0.0)) * (1.0 - kU) * kTwo31;
		const double fHi = std::sqrt(V + B) * (1.0 + kU) * kTwo31;
		double vlo, vhi, qlo, qhi;
		varWindow(v.data(), n, false, vlo, vhi);
		stdWindow(vlo, vhi, qlo, qhi);
		CAPTURE(n);
		// both functions must lie in their own window around the true value
		CHECK((double)r >= qlo);
		CHECK((double)r <= qhi);
		CHECK((double)fs * kTwo31 >= fLo);
		CHECK((double)fs * kTwo31 <= fHi);
	}
}

TEST_CASE("[C5 arm_mse_q31] the mean square error agrees with arm_mse_f32 of the same samples within (n + 6) u mean(s^2) plus the q31 bound") {
	// f32: with s = |a| + |b| >= |a - b|: conversions and the subtraction give |d_f - d| <= 2 u s, the square 5 u s^2,
	// the n - 1 additions and the division n u of the sum: (n + 6) u mean(s^2); q31: mseBound
	for (size_t n : kC5Sizes) {
		std::vector<int32_t> a = randomData(n, 9600u + (uint32_t)n, 1), b = randomData(n, 9700u + (uint32_t)n, 1);
		std::vector<float> fa = toFloat(a), fb = toFloat(b);
		float fm = 0.0f;
		arm_mse_f32(fa.data(), fb.data(), (uint32_t)n, &fm);
		q31_t r = 0;
		arm_mse_q31(a.data(), b.data(), (uint32_t)n, &r);
		double s2 = 0.0;
		for (size_t i = 0; i < n; i++) {
			const double s = (std::fabs((double)a[i]) + std::fabs((double)b[i])) / kTwo31;
			s2 += s * s;
		}
		s2 /= (double)n;
		const double bound = ((double)n + 6.0) * kU * s2 * kTwo31 + mseBound(a.data(), b.data(), n);
		CAPTURE(n);
		CHECK(std::fabs((double)r - (double)fm * kTwo31) <= bound);
	}
}

namespace {
// multiples of 128 are exact in f32 (24 significant bits), so the max / min / abs functions must agree exactly
std::vector<int32_t> exactFloatData(size_t n, uint32_t seed) {
	std::vector<int32_t> v = randomData(n, seed, 0);
	for (auto &e : v) {
		e = multipleOf128(e);
		if (e == INT32_MIN) e = INT32_MIN + 128;  // the f32 abs of -1.0 is 1.0, not representable in q31
	}
	return v;
}
}  // namespace

TEST_CASE("[C5 arm_max_q31] the maximum and its index agree exactly with arm_max_f32 on samples that are exact in f32") {
	for (size_t n : kC5Sizes) {
		std::vector<int32_t> v = exactFloatData(n, 9800u + (uint32_t)n);
		std::vector<float> f = toFloat(v);
		float fv = 0.0f;
		uint32_t fi = 99, qi = 99;
		arm_max_f32(f.data(), (uint32_t)n, &fv, &fi);
		q31_t r = 0;
		arm_max_q31(v.data(), (uint32_t)n, &r, &qi);
		CAPTURE(n);
		CHECK((double)r == (double)fv * kTwo31);
		CHECK(qi == fi);
	}
}

TEST_CASE("[C5 arm_min_q31] the minimum and its index agree exactly with arm_min_f32 on samples that are exact in f32") {
	for (size_t n : kC5Sizes) {
		std::vector<int32_t> v = exactFloatData(n, 9900u + (uint32_t)n);
		std::vector<float> f = toFloat(v);
		float fv = 0.0f;
		uint32_t fi = 99, qi = 99;
		arm_min_f32(f.data(), (uint32_t)n, &fv, &fi);
		q31_t r = 0;
		arm_min_q31(v.data(), (uint32_t)n, &r, &qi);
		CAPTURE(n);
		CHECK((double)r == (double)fv * kTwo31);
		CHECK(qi == fi);
	}
}

TEST_CASE("[C5 arm_absmax_q31] the absolute maximum and its index agree exactly with arm_absmax_f32 on samples that are exact in f32") {
	for (size_t n : kC5Sizes) {
		std::vector<int32_t> v = exactFloatData(n, 9810u + (uint32_t)n);
		std::vector<float> f = toFloat(v);
		float fv = 0.0f;
		uint32_t fi = 99, qi = 99;
		arm_absmax_f32(f.data(), (uint32_t)n, &fv, &fi);
		q31_t r = 0;
		arm_absmax_q31(v.data(), (uint32_t)n, &r, &qi);
		CAPTURE(n);
		CHECK((double)r == (double)fv * kTwo31);
		CHECK(qi == fi);
	}
}

TEST_CASE("[C5 arm_absmin_q31] the absolute minimum and its index agree exactly with arm_absmin_f32 on samples that are exact in f32") {
	for (size_t n : kC5Sizes) {
		std::vector<int32_t> v = exactFloatData(n, 9910u + (uint32_t)n);
		std::vector<float> f = toFloat(v);
		float fv = 0.0f;
		uint32_t fi = 99, qi = 99;
		arm_absmin_f32(f.data(), (uint32_t)n, &fv, &fi);
		q31_t r = 0;
		arm_absmin_q31(v.data(), (uint32_t)n, &r, &qi);
		CAPTURE(n);
		CHECK((double)r == (double)fv * kTwo31);
		CHECK(qi == fi);
	}
}

// ---------------------------------------------------------------------------------------------------------------
// FastMath Q63: arm_norm_64_to_32u, arm_div_int64_to_int32 (C3 / C4, DEF in the helpers above)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_norm_64_to_32u] for 0 and every input below 2^62 (all powers of two and their neighbours, random values) the value and shift match the definition") {
	std::vector<uint64_t> in{0u};
	for (int k = 0; k < 62; k++) {
		const uint64_t p = (uint64_t)1 << k;
		in.push_back(p);
		in.push_back(p - 1);
		in.push_back(p + 1);
		in.push_back(p | (p >> 1));
	}
	uint32_t s = 2024u;
	for (int i = 0; i < 2000; i++) {
		const uint64_t hi = rnd(s);
		const uint64_t lo = rnd(s);
		const int sh = (int)(rnd(s) % 62u);
		in.push_back(((hi << 32) | lo) >> (2 + sh));  // below 2^62, all magnitudes
	}
	size_t checked = 0;
	for (uint64_t x : in) {
		if (x >= ((uint64_t)1 << 62)) continue;
		int32_t v = 99, nm = 99, ev, en;
		arm_norm_64_to_32u(x, &v, &nm);
		defNorm(x, ev, en);
		checked++;
		if (v != ev || nm != en) {
			FAIL_CHECK("input " << x << ": value " << v << " shift " << nm << ", expected value " << ev << " shift " << en);
			return;
		}
	}
	CHECK(checked > 2000);
}

TEST_CASE("[C3 arm_norm_64_to_32u] the normalised value always has bit 30 set (or is 0 for the input 0) and shifting back reproduces the top 31 bits") {
	// property of the definition: val = in * 2^norm truncated, bit 30 set; checked for inputs below 2^62 (the UB range is a separate test)
	uint32_t s = 77u;
	for (int i = 0; i < 2000; i++) {
		const uint64_t hi = rnd(s);
		const uint64_t lo = rnd(s);
		const int sh = (int)(rnd(s) % 62u);
		const uint64_t x = ((hi << 32) | lo) >> (2 + sh);
		int32_t v = 0, nm = 0;
		arm_norm_64_to_32u(x, &v, &nm);
		if (x == 0) {
			CHECK(v == 0);
			CHECK(nm == 0);
			continue;
		}
		if (((uint32_t)v >> 30) != 1u) {
			FAIL_CHECK("input " << x << ": value " << v << " has not exactly bit 30 as top bit");
			return;
		}
	}
}

TEST_CASE("[C3 arm_norm_64_to_32u] for inputs 2^62 and above whose low word is not zero the value and shift match the definition" * KNOWN_ISSUE_UB("B-DSQ31-1: for inputs >= 2^62 the shifts `lo >> 32`, `lo >> 33`, `hi << -1` (utils.h:221) are undefined; x86 masks the count, so the result is (lo | hi) instead of hi (e.g. 2^62 + 1 gives 0x40000001, intended 0x40000000); UB: the target result is unverified (a register shift by >= 32 gives 0 on Cortex-M, so for inputs >= 2^63 the target value is likely 0; the bit 62 case is as stated); ARM's norm pattern has no input >= 2^62, only ARM's div pattern reaches this code")) {
	std::vector<uint64_t> in;
	for (int k = 62; k < 64; k++) {
		const uint64_t p = (uint64_t)1 << k;
		in.push_back(p);
		in.push_back(p - 1);
		in.push_back(p + 1);
		in.push_back(p | (p >> 1));
		in.push_back(p | 0xFFFFFFFFull);
		in.push_back(p | 0x123456789ABCull);
	}
	in.push_back(UINT64_MAX);
	in.push_back(UINT64_MAX - 1);
	in.push_back(0x7FFFFFFF00000000ull);
	in.push_back(0x4000000100000000ull);
	for (uint64_t x : in) {
		int32_t v = 99, nm = 99, ev, en;
		arm_norm_64_to_32u(x, &v, &nm);
		defNorm(x, ev, en);
		CAPTURE(x);
		CHECK(v == ev);
		CHECK(nm == en);
	}
}

TEST_CASE("[C4 arm_norm_64_to_32u] the input 0 gives value 0 and shift 0, a 32 bit input with bit 31 set gives shift -1") {
	int32_t v = 99, nm = 99;
	arm_norm_64_to_32u(0, &v, &nm);
	CHECK(v == 0);
	CHECK(nm == 0);
	arm_norm_64_to_32u(0x80000000ull, &v, &nm);
	CHECK(v == 0x40000000);
	CHECK(nm == -1);
	arm_norm_64_to_32u(0xFFFFFFFFull, &v, &nm);
	CHECK(v == 0x7FFFFFFF);
	CHECK(nm == -1);
	arm_norm_64_to_32u(1, &v, &nm);
	CHECK(v == 0x40000000);
	CHECK(nm == 30);
	arm_norm_64_to_32u(0x40000000ull, &v, &nm);
	CHECK(v == 0x40000000);
	CHECK(nm == 0);
}

TEST_CASE("[C3 arm_div_int64_to_int32] random 64 bit numerators below 2^62 and divisors give the truncated quotient, saturated to the int32 range") {
	uint32_t s = 6060u;
	size_t checked = 0;
	for (int i = 0; i < 4000; i++) {
		const uint64_t hi = rnd(s);
		const uint64_t lo = rnd(s);
		const int sh = 2 + (int)(rnd(s) % 61u);  // |num| < 2^62 (larger numerators: UB test below)
		const int dsh = (int)(rnd(s) % 32u);
		const uint32_t dr = rnd(s);
		int64_t num = (int64_t)(((hi << 32) | lo) >> sh);
		if (i & 1) num = -num;
		int32_t den = (int32_t)(dr >> dsh);
		if (i & 2) den = -den;
		if (den == 0) den = 1;
		const int32_t r = arm_div_int64_to_int32(num, den);
		checked++;
		if (r != defDiv(num, den)) {
			FAIL_CHECK("num " << num << " den " << den << ": " << r << ", expected " << defDiv(num, den));
			return;
		}
	}
	CHECK(checked == 4000);
}

TEST_CASE("[C3 arm_div_int64_to_int32] random numerators of 2^62 and above give the truncated quotient, saturated to the int32 range" * KNOWN_ISSUE_UB_HIDDEN("B-DSQ31-1: arm_div_int64_to_int32 calls arm_norm_64_to_32u, whose shifts are undefined for |num| >= 2^62 (utils.h:221); the quotient itself is correct on the host (the result only uses the sign of the shift)")) {
	uint32_t s = 6161u;
	for (int i = 0; i < 1000; i++) {
		const uint64_t hi = rnd(s);
		const uint64_t lo = rnd(s);
		const uint32_t dr = rnd(s);
		const int dsh = (int)(rnd(s) % 32u);
		int64_t num = (int64_t)((((hi << 32) | lo) >> 1) | ((uint64_t)1 << 62));  // 2^62 .. 2^63 - 1
		if (i & 1) num = -num;
		int32_t den = (int32_t)(dr >> dsh);
		if (i & 2) den = -den;
		if (den == 0) den = 1;
		const int32_t r = arm_div_int64_to_int32(num, den);
		if (r != defDiv(num, den)) {
			FAIL_CHECK("num " << num << " den " << den << ": " << r << ", expected " << defDiv(num, den));
			return;
		}
	}
}

TEST_CASE("[C3 arm_div_int64_to_int32] numerators around the switch between the 32 bit and the 64 bit division (|num| = 2^30) give the exact quotient") {
	// in the code norm > 0 (|num| < 2^30) selects the 32 bit division
	const int64_t nums[] = {0, 1, -1, 2, 1073741823, 1073741824, 1073741825, -1073741823, -1073741824, -1073741825,
	                        2147483647, 2147483648ll, -2147483648ll, 4294967295ll, 4294967296ll, -4294967296ll};
	const int32_t dens[] = {1, -1, 2, -2, 3, -3, 7, 1000, 65536, INT32_MAX, INT32_MIN, INT32_MIN + 1};
	for (int64_t num : nums)
		for (int32_t den : dens) {
			CAPTURE(num);
			CAPTURE(den);
			CHECK(arm_div_int64_to_int32(num, den) == defDiv(num, den));
		}
}

TEST_CASE("[C4 arm_div_int64_to_int32] results beyond the int32 range saturate to INT32_MAX and INT32_MIN, the boundary values do not (numerators below 2^62)") {
	// hand derivation: 3 * 2^31 / 3 = 2^31 -> INT32_MAX; (3 * 2^31 - 1) / 3 = 2147483647 (no saturation);
	// -3 * 2^31 / 3 = -2^31 = INT32_MIN exactly; (-3 * 2^31 - 3) / 3 = -2^31 - 1 -> INT32_MIN
	CHECK(arm_div_int64_to_int32(3ll * 2147483648ll, 3) == INT32_MAX);
	CHECK(arm_div_int64_to_int32(3ll * 2147483648ll - 1, 3) == INT32_MAX);
	CHECK(arm_div_int64_to_int32(3ll * 2147483648ll - 3, 3) == INT32_MAX - 0);  // (3 * 2^31 - 3) / 3 = 2^31 - 1
	CHECK(arm_div_int64_to_int32(3ll * 2147483648ll - 6, 3) == INT32_MAX - 1);
	CHECK(arm_div_int64_to_int32(-3ll * 2147483648ll, 3) == INT32_MIN);
	CHECK(arm_div_int64_to_int32(-3ll * 2147483648ll - 3, 3) == INT32_MIN);
	CHECK(arm_div_int64_to_int32(-3ll * 2147483648ll + 3, 3) == INT32_MIN + 1);
	CHECK(arm_div_int64_to_int32(0, 5) == 0);
	CHECK(arm_div_int64_to_int32(0, -5) == 0);
}

TEST_CASE("[C4 arm_div_int64_to_int32] numerators INT64_MAX, -INT64_MAX and INT64_MIN saturate to the int32 range" * KNOWN_ISSUE_UB_HIDDEN("B-DSQ31-1: arm_div_int64_to_int32 calls arm_norm_64_to_32u, whose shifts are undefined for |num| >= 2^62 (utils.h:221); the quotient itself is correct on the host (the result only uses the sign of the shift)")) {
	CHECK(arm_div_int64_to_int32(INT64_MAX, 1) == INT32_MAX);
	CHECK(arm_div_int64_to_int32(INT64_MAX, -1) == INT32_MIN);
	CHECK(arm_div_int64_to_int32(-INT64_MAX, -1) == INT32_MAX);
	CHECK(arm_div_int64_to_int32(INT64_MAX, INT32_MIN) == INT32_MIN);
	CHECK(arm_div_int64_to_int32(INT64_MIN, 1) == INT32_MIN);
	CHECK(arm_div_int64_to_int32(INT64_MIN, 2) == INT32_MIN);
	CHECK(arm_div_int64_to_int32(INT64_MIN, INT32_MIN) == INT32_MAX);  // 2^63 / 2^31 = 2^32 -> INT32_MAX
	CHECK(arm_div_int64_to_int32(INT64_MIN, INT32_MAX) == INT32_MIN);
}

TEST_CASE("[C4 arm_div_int64_to_int32] the quotient is truncated toward zero for small negative and positive numerators (32 bit path)") {
	CHECK(arm_div_int64_to_int32(7, 2) == 3);
	CHECK(arm_div_int64_to_int32(-7, 2) == -3);
	CHECK(arm_div_int64_to_int32(7, -2) == -3);
	CHECK(arm_div_int64_to_int32(-7, -2) == 3);
	CHECK(arm_div_int64_to_int32(-1, 2) == 0);
	CHECK(arm_div_int64_to_int32(1, INT32_MIN) == 0);
	CHECK(arm_div_int64_to_int32(-1, INT32_MIN) == 0);
}

TEST_CASE("[C4 arm_div_int64_to_int32] INT64_MIN divided by -1 saturates to INT32_MAX" * doctest::skip() * doctest::description("KNOWN ISSUE (crash): B-DSQ31-2: arm_div_int64_to_int32(INT64_MIN, -1) executes the 64 bit division INT64_MIN / -1 (utils.h:261), which overflows: SIGFPE on x86, INT64_MIN on ARM (then saturated to INT32_MIN, the wrong sign). Skipped because it terminates the test run; the DOC says the result is saturated, i.e. INT32_MAX")) {
	CHECK(arm_div_int64_to_int32(INT64_MIN, -1) == INT32_MAX);
}

}  // TEST_SUITE
