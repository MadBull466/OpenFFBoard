/*
 * CMSIS-DSP Statistics f32 (work order SF32, phase P11 group G3). Requirement ids: Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * Units covered (all 19 f32 units of Source/StatisticsFunctions): arm_max_f32, arm_min_f32, arm_absmax_f32, arm_absmin_f32 (value and
 * index), arm_max_no_idx_f32, arm_min_no_idx_f32, arm_absmax_no_idx_f32, arm_absmin_no_idx_f32, arm_accumulate_f32, arm_mean_f32,
 * arm_mse_f32, arm_power_f32, arm_rms_f32, arm_std_f32, arm_var_f32, arm_entropy_f32, arm_kullback_leibler_f32, arm_logsumexp_f32,
 * arm_logsumexp_dot_prod_f32. Requirement C5 does not apply (f32 is the reference type of the other width files).
 *
 * Pattern files used (Libraries/CMSIS-DSP/Testing/Patterns/DSP/Stats/StatsF32/). ARM's tests are in Testing/Source/Tests/StatsTestsF32.cpp
 * (test functions :16-:386, setUp :388-:end) and the suite "Statistics Tests F32" in Testing/desc.txt:142-:268 (53 test ids, all run here;
 * nothing is commented out or disabled). "nb=3 / 4n / 4n+1" are the first 3, 8 and 11 samples of the pattern (inputA.reload(..., nb)),
 * "long" is the first 100 samples; refOffset selects the 0th..3rd reference value:
 *   max / min       : Input1_f32 -> MaxVals1 + MaxIndexes1_s16 / MinVals3 + MinIndexes3_s16         (nb 3, 8, 11)      :16, :119
 *   absmax / absmin : InputNew1_f32 -> AbsMaxVals26 + AbsMaxIndexes26_s16 / AbsMinVals27 + AbsMinIndexes27_s16 (3, 8, 11)  :39, :139
 *   *_no_idx        : same inputs and values, no index (exact)                                         :62, :74, :86, :98, :109
 *   mean            : Input2_f32 -> MeanVals2   (nb 3, 8, 11, long 100)                                 :160
 *   power           : Input1_f32 -> PowerVals4  (nb 3, 8, 11)                                           :174
 *   rms / std / var : Input1_f32 -> RmsVals5 / StdVals6 / VarVals7 (nb 3, 8, 11, long 100)              :188, :202, :216
 *   mse             : InputNew1_f32 + InputNew2_f32 -> MSEVals28 (nb 3, 8, 11, long 100)                :314
 *   std stability   : no pattern, {4,7,13,16} + 3e4, result within 1e-4 of 5.47723                       :230
 *   entropy         : Input22_f32 + Dims22_s16 -> RefEntropy22 (11 vectors)                             :270
 *   logsumexp       : Input23_f32 + Dims23_s16 -> RefLogSumExp23                                        :285
 *   kullback_leibler: InputA24 + InputB24 + Dims24 -> RefKL24                                           :299
 *   logsumexp_dot   : InputA25 + InputB25 + Dims25 -> RefLogSumExpDot25 (temp buffer 12)                :316
 *   accumulate      : InputAccumulate1 + InputAccumulateConfig1_s16 -> RefAccumulate1 (sizes 4: 3, 8, 11, 300)  :345
 *     (ARM's accumulate test never advances the input pointer: every size sums the first N samples. Run the same way here.)
 *
 * ARM tolerances (StatsTestsF32.cpp:6-:13): SNR_THRESHOLD 120 dB, REL_ERROR 1.0e-5 ("reference patterns are generated with a double
 * precision computation"). ASSERT_REL_ERROR (FrameworkSource/Error.cpp:218) is |a-b| / ((|a|+|b|)/2) <= REL, nothing compared if both
 * are 0; armRel() below does the same but also fails on NaN. max / min / absmax / absmin values and all indexes are compared EXACT
 * (ASSERT_EQ; the index is cast to int16_t as in ARM).
 *
 * DEF tests (C3): the reference is evaluated in double inside this file, never with a library function. Bounds are derived from the
 * standard rounding model of binary32 (round to nearest, eps = 2^-24, a chain of k operations costs gam(k) = k*eps*(1+1e-4)) and from
 * the summation order of the library (one running sum in a loop unrolled by 4, so a chain of at most n-1 additions):
 *   accumulate : |err| <= gam(n-1) * sum|x|
 *   mean       : sum then one division: |err| <= gam(n) * sum|x| / n
 *   power      : every term x*x (1 op) and n-1 additions of non-negative terms: relative error <= gam(n)
 *   rms        : sqrt(power / n): relative <= gam(n+1)/2 + 1.01*eps (sqrt halves the relative error and adds one rounding)
 *   mse        : d = a-b (1), d*d (1), squares ... 2 more for the first-order effect of d, n-1 additions, one division: relative <= gam(n+3)
 *   var        : two pass. With the mean m_f off by delta (|delta| <= D = gam(n) sum|x| / n) the library sums (x-m_f)^2 = V0 + n delta^2
 *                (the first order term is 0 because sum(x-m) = 0), V0 = sum (x-m)^2. Operations on that sum: subtraction, square, n-1
 *                additions, division by n-1: relative gam(n+3). Bound: var*gam(n+3) + n*D^2/(n-1)*(1+gam(n+3)).
 *   std        : sqrt of var: relative (bound_var/var)/2*1.01 + 1.01*eps.
 *   entropy    : term p*logf(p): logf is within 1 ulp (relative 2*eps of |ln p|), product 1 rounding, n-1 additions:
 *                |err| <= gam(n+2) * sum|p ln p|. (glibc logf is below 1 ulp; newlib on the target is documented below 1 ulp as well.)
 *   KL         : term pA*logf(pB/pA): division error eps (absolute error in ln), logf 2*eps*|ln r|, product eps*|term|, n-1 additions:
 *                |err| <= 1.001*eps * sum(pA(1+2|ln r|) + |term|) + gam(n-1) sum|term|.
 *   logsumexp  : d_i = fl(x_i-m) (relative eps), expf within 1 ulp (2*eps), so the relative error of each exp term <= 2*eps + eps*|d_i|,
 *                n-1 additions, then logf(S): absolute error of ln S <= rel(S) + 2*eps*|ln S|, then m + ln S rounds with eps*|result|:
 *                |err| <= 1.001 * (eps*(2 + max|d_i|) + gam(n-1) + 2*eps*|ln S| + eps*|result|).
 *   lse dot    : t = fl(a+b) has error eps*|a+b| and logsumexp is 1-Lipschitz in the maximum norm: add eps*max|a+b| to the bound.
 *
 * Index outputs (max / min / absmax / absmin): DOC says nothing about ties. The source compares with a strict '>' / '<' (the first
 * occurrence wins, like numpy argmax which generated ARM's patterns): tested as `characterization:` (NEEDS REVIEW), listed in the report.
 * Also not documented: blockSize 0. The max family loads pSrc[0] unconditionally and computes (blockSize-1U)>>2U = 2^30-1 loop
 * passes for blockSize 0: NOT TESTED (it would read gigabytes); see the report. arm_var_f32 / arm_std_f32 return 0 for blockSize <= 1
 * (explicit branch in the source, DOC silent): characterization.
 *
 * NEEDS REVIEW (lead decisions, not tested): (1) blockSize 0 for the max / min / absmax / absmin family reads out of bounds (pSrc[0] is loaded
 * unconditionally and the unrolled loop count wraps); (2) overflow near FLT_MAX (power, mean, accumulate give inf): DOC silent.
 *
 * More NEEDS REVIEW notes (no test pins them): (3) NaN input: max / min WITH index return NaN at index 0 for {NaN,1,2} (the extreme starts
 * from sample 0 and no comparison with NaN succeeds) while max_no_idx returns 2 and min_no_idx returns 1 (they start from -FLT_MAX / FLT_MAX);
 * absmax / absmin return NaN in both variants. (4) KL with a subnormal pA (1e-39, pB 0.5): pB/pA overflows to inf and the result is -inf.
 * (5) Long sums saturate: arm_mean_f32 of 3e7 ones gives about 0.559 (the running float sum stops growing at 2^24) and arm_var_f32 of 1e6
 * samples 1000 +- 1 is about 80, not 1 (reviewer probe). Not pinned by a test: the values are suspicious and depend on the summation order; the
 * n = 4096 test below stays inside the derived gam(n) bound. (6) arm_std_f32 / arm_rms_f32 / logsumexp with NaN or infinity: B-DSF32-3/-4/-5.
 *
 * Host vs target (C6): the host runs the portable C path (ARM_MATH_LOOPUNROLL, no ARM_MATH_DSP, no MVE / Neon): one running sum in the
 * unrolled loop (sum += *p++ four times per pass). The target build (Makefile -O2, fpv4-sp-d16) may contract a*b+c into VFMA
 * (var, power, mse, rms, entropy, KL) and logf / expf come from newlib instead of glibc: all bounds above are for the rounding model and
 * hold for either; the exact comparisons use dyadic data (every intermediate value exactly representable, so FMA gives the same
 * result) or one-operation functions. sqrt (rms, std) is correctly rounded on both (sqrtf on the host, VSQRT.F32 on the target).
 * Float -> int conversions: only the index (uint32_t, no conversion of a float).
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

const char *const kDir = "Stats/StatsF32/";

// ARM tolerances (StatsTestsF32.cpp:6-:13)
const double kSnr = 120.0;
const double kRel = 1.0e-5;

const double kEps = 5.9604644775390625e-8;   // 2^-24
double gam(double k) { return k * kEps * 1.0001; }

std::vector<float> pat(const std::string &name) { return dsp::loadPattern(kDir + name).f32(); }
std::vector<int16_t> patIdx(const std::string &name) { return dsp::loadPattern(kDir + name).q15(); }

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

// Both ARM assertions of a result block (SNR and relative error)
void checkArm(const std::vector<float> &ref, const std::vector<float> &out) {
	CHECK(dsp::snr(ref, out) >= kSnr);
	auto c = armRel(ref, out, kRel);
	CHECK_MESSAGE(c, c.message);
}

// Deterministic generator (LCG, Numerical Recipes constants). One call per statement.
struct Rng {
	uint32_t s;
	explicit Rng(uint32_t seed) : s(seed) {}
	uint32_t next() { s = s * 1664525u + 1013904223u; return s; }
	float uni(float lo, float hi) {
		const uint32_t v = next() >> 8;                         // 24 bit
		return lo + (hi - lo) * (static_cast<float>(v) / 16777216.0f);
	}
};

std::vector<float> randVec(Rng &r, size_t n, float lo, float hi) {
	std::vector<float> v(n);
	for (size_t i = 0; i < n; i++) v[i] = r.uni(lo, hi);
	return v;
}

// ---------------------------------------------------------------------------------------------------------------
// function tables
// ---------------------------------------------------------------------------------------------------------------
enum Mode { MAX = 0, MIN = 1, ABSMAX = 2, ABSMIN = 3 };
typedef void (*IdxFn)(const float *, uint32_t, float *, uint32_t *);
typedef void (*NoIdxFn)(const float *, uint32_t, float *);
typedef void (*ScalarFn)(const float *, uint32_t, float *);

const IdxFn kIdx[4] = {arm_max_f32, arm_min_f32, arm_absmax_f32, arm_absmin_f32};
const NoIdxFn kNoIdx[4] = {arm_max_no_idx_f32, arm_min_no_idx_f32, arm_absmax_no_idx_f32, arm_absmin_no_idx_f32};

double keyOf(Mode m, double x) {
	switch (m) {
	case MAX: return x;
	case MIN: return -x;
	case ABSMAX: return std::fabs(x);
	default: return -std::fabs(x);
	}
}

struct Ext { double value; size_t index; };
// DEF: first occurrence of the extreme key; abs variants return the absolute value (DOC of absmax: "maximum value of absolute values")
Ext defExt(Mode m, const std::vector<float> &x) {
	size_t bi = 0;
	for (size_t i = 1; i < x.size(); i++)
		if (keyOf(m, x[i]) > keyOf(m, x[bi])) bi = i;
	const double v = x[bi];
	return {(m == ABSMAX || m == ABSMIN) ? std::fabs(v) : v, bi};
}

// ---------------------------------------------------------------------------------------------------------------
// C1 helpers: ARM's tests
// ---------------------------------------------------------------------------------------------------------------
// test_max_f32 / test_absmax_f32 / test_min_f32 / test_absmin_f32: exact value and exact index
void armIdx(Mode m, const char *input, const char *vals, const char *idxs, size_t nb, size_t refOffset) {
	std::vector<float> in = pat(input);
	const std::vector<float> ref = pat(vals);
	const std::vector<int16_t> refInd = patIdx(idxs);
	REQUIRE(in.size() >= nb);
	REQUIRE(ref.size() > refOffset);
	REQUIRE(refInd.size() > refOffset);
	in.resize(nb);
	float result = -12345.0f;
	uint32_t indexval = 0xFFFFFFFFu;
	kIdx[m](in.data(), static_cast<uint32_t>(nb), &result, &indexval);
	CHECK(result == ref[refOffset]);
	CHECK(static_cast<int16_t>(indexval) == refInd[refOffset]);
}

void armNoIdx(Mode m, const char *input, const char *vals, size_t nb, size_t refOffset) {
	std::vector<float> in = pat(input);
	const std::vector<float> ref = pat(vals);
	REQUIRE(in.size() >= nb);
	REQUIRE(ref.size() > refOffset);
	in.resize(nb);
	float result = -12345.0f;
	kNoIdx[m](in.data(), static_cast<uint32_t>(nb), &result);
	CHECK(result == ref[refOffset]);
}

// mean, power, rms, std, var: SNR and relative error against the reference value at refOffset
void armScalar(ScalarFn fn, const char *input, const char *vals, size_t nb, size_t refOffset) {
	std::vector<float> in = pat(input);
	const std::vector<float> ref = pat(vals);
	REQUIRE(in.size() >= nb);
	REQUIRE(ref.size() > refOffset);
	in.resize(nb);
	float result = -12345.0f;
	fn(in.data(), static_cast<uint32_t>(nb), &result);
	checkArm({ref[refOffset]}, {result});
}

void armMse(size_t nb, size_t refOffset) {
	std::vector<float> a = pat("InputNew1_f32.txt"), b = pat("InputNew2_f32.txt");
	const std::vector<float> ref = pat("MSEVals28_f32.txt");
	REQUIRE(a.size() >= nb);
	REQUIRE(b.size() >= nb);
	REQUIRE(ref.size() > refOffset);
	a.resize(nb);
	b.resize(nb);
	float result = -12345.0f;
	arm_mse_f32(a.data(), b.data(), static_cast<uint32_t>(nb), &result);
	checkArm({ref[refOffset]}, {result});
}

// ---------------------------------------------------------------------------------------------------------------
// DEF helpers (double)
// ---------------------------------------------------------------------------------------------------------------
double sumAbs(const std::vector<float> &x) { double a = 0; for (float v : x) a += std::fabs(static_cast<double>(v)); return a; }
double sumOf(const std::vector<float> &x) { double a = 0; for (float v : x) a += v; return a; }
double defMean(const std::vector<float> &x) { return sumOf(x) / static_cast<double>(x.size()); }
double defPower(const std::vector<float> &x) { double a = 0; for (float v : x) a += static_cast<double>(v) * v; return a; }
double defVar(const std::vector<float> &x) {
	const double m = defMean(x);
	double a = 0;
	for (float v : x) a += (v - m) * (v - m);
	return a / static_cast<double>(x.size() - 1);
}

// DEF of logsumexp: m + ln(sum exp(x - m)). Returns the value, S = sum exp(x-m) and Dmax = max|x-m|.
struct Lse { double value, S, dmax; };
Lse defLse(const std::vector<double> &x) {
	double m = x[0];
	for (double v : x) if (v > m) m = v;
	double S = 0, dmax = 0;
	for (double v : x) { S += std::exp(v - m); dmax = std::fmax(dmax, std::fabs(v - m)); }
	return {m + std::log(S), S, dmax};
}
double lseBound(const Lse &d, size_t n) {
	return 1.001 * (kEps * (2.0 + d.dmax) + gam(static_cast<double>(n) - 1.0) + 2.0 * kEps * std::fabs(std::log(d.S)) + kEps * std::fabs(d.value));
}

// Blocks of n finite floats taken from a golden-ratio stride over all 2^32 bit patterns (an odd stride visits every pattern once
// per full cycle); NaN and infinity patterns are skipped. Covers every exponent, both signs, subnormals.
template <class F>
void sweepBlocks(size_t n, size_t blocks, F &&f) {
	uint32_t k = 12345u;
	for (size_t b = 0; b < blocks; b++) {
		std::vector<float> x(n);
		for (size_t i = 0; i < n; i++) {
			uint32_t bits;
			do { k += 2654435761u; bits = k; } while (((bits >> 23) & 0xFFu) == 0xFFu);
			float v;
			std::memcpy(&v, &bits, sizeof v);
			x[i] = v;
		}
		f(x);
	}
}

const uint32_t kSizes[] = {1, 2, 3, 4, 5, 9};   // 4 is one full unrolled group (blockSize-1 = 3 is tail only, 5 and 9 add groups)

}  // namespace

TEST_SUITE("DSP Statistics f32") {

// ===============================================================================================================
// C1: ARM's test ids
// ===============================================================================================================

TEST_CASE("[C1 arm_max_f32] max and index of the first 3 samples (ARM nb=3) equal ARM's reference exactly") { armIdx(MAX, "Input1_f32.txt", "MaxVals1_f32.txt", "MaxIndexes1_s16.txt", 3, 0); }
TEST_CASE("[C1 arm_max_f32] max and index of the first 8 samples (ARM nb=4n) equal ARM's reference exactly") { armIdx(MAX, "Input1_f32.txt", "MaxVals1_f32.txt", "MaxIndexes1_s16.txt", 8, 1); }
TEST_CASE("[C1 arm_max_f32] max and index of the first 11 samples (ARM nb=4n+1) equal ARM's reference exactly") { armIdx(MAX, "Input1_f32.txt", "MaxVals1_f32.txt", "MaxIndexes1_s16.txt", 11, 2); }
TEST_CASE("[C1 arm_min_f32] min and index of the first 3 samples (ARM nb=3) equal ARM's reference exactly") { armIdx(MIN, "Input1_f32.txt", "MinVals3_f32.txt", "MinIndexes3_s16.txt", 3, 0); }
TEST_CASE("[C1 arm_min_f32] min and index of the first 8 samples (ARM nb=4n) equal ARM's reference exactly") { armIdx(MIN, "Input1_f32.txt", "MinVals3_f32.txt", "MinIndexes3_s16.txt", 8, 1); }
TEST_CASE("[C1 arm_min_f32] min and index of the first 11 samples (ARM nb=4n+1) equal ARM's reference exactly") { armIdx(MIN, "Input1_f32.txt", "MinVals3_f32.txt", "MinIndexes3_s16.txt", 11, 2); }
TEST_CASE("[C1 arm_absmax_f32] absmax and index of the first 3 samples (ARM nb=3) equal ARM's reference exactly") { armIdx(ABSMAX, "InputNew1_f32.txt", "AbsMaxVals26_f32.txt", "AbsMaxIndexes26_s16.txt", 3, 0); }
TEST_CASE("[C1 arm_absmax_f32] absmax and index of the first 8 samples (ARM nb=4n) equal ARM's reference exactly") { armIdx(ABSMAX, "InputNew1_f32.txt", "AbsMaxVals26_f32.txt", "AbsMaxIndexes26_s16.txt", 8, 1); }
TEST_CASE("[C1 arm_absmax_f32] absmax and index of the first 11 samples (ARM nb=4n+1) equal ARM's reference exactly") { armIdx(ABSMAX, "InputNew1_f32.txt", "AbsMaxVals26_f32.txt", "AbsMaxIndexes26_s16.txt", 11, 2); }
TEST_CASE("[C1 arm_absmin_f32] absmin and index of the first 3 samples (ARM nb=3) equal ARM's reference exactly") { armIdx(ABSMIN, "InputNew1_f32.txt", "AbsMinVals27_f32.txt", "AbsMinIndexes27_s16.txt", 3, 0); }
TEST_CASE("[C1 arm_absmin_f32] absmin and index of the first 8 samples (ARM nb=4n) equal ARM's reference exactly") { armIdx(ABSMIN, "InputNew1_f32.txt", "AbsMinVals27_f32.txt", "AbsMinIndexes27_s16.txt", 8, 1); }
TEST_CASE("[C1 arm_absmin_f32] absmin and index of the first 11 samples (ARM nb=4n+1) equal ARM's reference exactly") { armIdx(ABSMIN, "InputNew1_f32.txt", "AbsMinVals27_f32.txt", "AbsMinIndexes27_s16.txt", 11, 2); }

TEST_CASE("[C1 arm_max_no_idx_f32] max of the first 3 samples (ARM nb=3) equals ARM's reference exactly") { armNoIdx(MAX, "Input1_f32.txt", "MaxVals1_f32.txt", 3, 0); }
TEST_CASE("[C1 arm_max_no_idx_f32] max of the first 8 samples (ARM nb=4n) equals ARM's reference exactly") { armNoIdx(MAX, "Input1_f32.txt", "MaxVals1_f32.txt", 8, 1); }
TEST_CASE("[C1 arm_max_no_idx_f32] max of the first 11 samples (ARM nb=4n+1) equals ARM's reference exactly") { armNoIdx(MAX, "Input1_f32.txt", "MaxVals1_f32.txt", 11, 2); }
TEST_CASE("[C1 arm_min_no_idx_f32] min of the first 3 samples (ARM nb=3) equals ARM's reference exactly") { armNoIdx(MIN, "Input1_f32.txt", "MinVals3_f32.txt", 3, 0); }
TEST_CASE("[C1 arm_min_no_idx_f32] min of the first 8 samples (ARM nb=4n) equals ARM's reference exactly") { armNoIdx(MIN, "Input1_f32.txt", "MinVals3_f32.txt", 8, 1); }
TEST_CASE("[C1 arm_min_no_idx_f32] min of the first 11 samples (ARM nb=4n+1) equals ARM's reference exactly") { armNoIdx(MIN, "Input1_f32.txt", "MinVals3_f32.txt", 11, 2); }
TEST_CASE("[C1 arm_absmax_no_idx_f32] absmax of the first 3 samples (ARM nb=3) equals ARM's reference exactly") { armNoIdx(ABSMAX, "InputNew1_f32.txt", "AbsMaxVals26_f32.txt", 3, 0); }
TEST_CASE("[C1 arm_absmax_no_idx_f32] absmax of the first 8 samples (ARM nb=4n) equals ARM's reference exactly") { armNoIdx(ABSMAX, "InputNew1_f32.txt", "AbsMaxVals26_f32.txt", 8, 1); }
TEST_CASE("[C1 arm_absmax_no_idx_f32] absmax of the first 11 samples (ARM nb=4n+1) equals ARM's reference exactly") { armNoIdx(ABSMAX, "InputNew1_f32.txt", "AbsMaxVals26_f32.txt", 11, 2); }
TEST_CASE("[C1 arm_absmin_no_idx_f32] absmin of the first 3 samples (ARM nb=3) equals ARM's reference exactly") { armNoIdx(ABSMIN, "InputNew1_f32.txt", "AbsMinVals27_f32.txt", 3, 0); }
TEST_CASE("[C1 arm_absmin_no_idx_f32] absmin of the first 8 samples (ARM nb=4n) equals ARM's reference exactly") { armNoIdx(ABSMIN, "InputNew1_f32.txt", "AbsMinVals27_f32.txt", 8, 1); }
TEST_CASE("[C1 arm_absmin_no_idx_f32] absmin of the first 11 samples (ARM nb=4n+1) equals ARM's reference exactly") { armNoIdx(ABSMIN, "InputNew1_f32.txt", "AbsMinVals27_f32.txt", 11, 2); }

TEST_CASE("[C1 arm_mean_f32] mean of the first 3 samples (ARM nb=3) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_mean_f32, "Input2_f32.txt", "MeanVals2_f32.txt", 3, 0); }
TEST_CASE("[C1 arm_mean_f32] mean of the first 8 samples (ARM nb=4n) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_mean_f32, "Input2_f32.txt", "MeanVals2_f32.txt", 8, 1); }
TEST_CASE("[C1 arm_mean_f32] mean of the first 11 samples (ARM nb=4n+1) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_mean_f32, "Input2_f32.txt", "MeanVals2_f32.txt", 11, 2); }
TEST_CASE("[C1 arm_mean_f32] mean of the first 100 samples (ARM long) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_mean_f32, "Input2_f32.txt", "MeanVals2_f32.txt", 100, 3); }

TEST_CASE("[C1 arm_power_f32] power of the first 3 samples (ARM nb=3) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_power_f32, "Input1_f32.txt", "PowerVals4_f32.txt", 3, 0); }
TEST_CASE("[C1 arm_power_f32] power of the first 8 samples (ARM nb=4n) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_power_f32, "Input1_f32.txt", "PowerVals4_f32.txt", 8, 1); }
TEST_CASE("[C1 arm_power_f32] power of the first 11 samples (ARM nb=4n+1) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_power_f32, "Input1_f32.txt", "PowerVals4_f32.txt", 11, 2); }

TEST_CASE("[C1 arm_rms_f32] rms of the first 3 samples (ARM nb=3) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_rms_f32, "Input1_f32.txt", "RmsVals5_f32.txt", 3, 0); }
TEST_CASE("[C1 arm_rms_f32] rms of the first 8 samples (ARM nb=4n) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_rms_f32, "Input1_f32.txt", "RmsVals5_f32.txt", 8, 1); }
TEST_CASE("[C1 arm_rms_f32] rms of the first 11 samples (ARM nb=4n+1) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_rms_f32, "Input1_f32.txt", "RmsVals5_f32.txt", 11, 2); }
TEST_CASE("[C1 arm_rms_f32] rms of the first 100 samples (ARM long) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_rms_f32, "Input1_f32.txt", "RmsVals5_f32.txt", 100, 3); }

TEST_CASE("[C1 arm_std_f32] std of the first 3 samples (ARM nb=3) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_std_f32, "Input1_f32.txt", "StdVals6_f32.txt", 3, 0); }
TEST_CASE("[C1 arm_std_f32] std of the first 8 samples (ARM nb=4n) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_std_f32, "Input1_f32.txt", "StdVals6_f32.txt", 8, 1); }
TEST_CASE("[C1 arm_std_f32] std of the first 11 samples (ARM nb=4n+1) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_std_f32, "Input1_f32.txt", "StdVals6_f32.txt", 11, 2); }
TEST_CASE("[C1 arm_std_f32] std of the first 100 samples (ARM long) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_std_f32, "Input1_f32.txt", "StdVals6_f32.txt", 100, 3); }

TEST_CASE("[C1 arm_var_f32] var of the first 3 samples (ARM nb=3) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_var_f32, "Input1_f32.txt", "VarVals7_f32.txt", 3, 0); }
TEST_CASE("[C1 arm_var_f32] var of the first 8 samples (ARM nb=4n) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_var_f32, "Input1_f32.txt", "VarVals7_f32.txt", 8, 1); }
TEST_CASE("[C1 arm_var_f32] var of the first 11 samples (ARM nb=4n+1) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_var_f32, "Input1_f32.txt", "VarVals7_f32.txt", 11, 2); }
TEST_CASE("[C1 arm_var_f32] var of the first 100 samples (ARM long) matches ARM's reference within 120 dB and 1e-5 relative") { armScalar(arm_var_f32, "Input1_f32.txt", "VarVals7_f32.txt", 100, 3); }

TEST_CASE("[C1 arm_mse_f32] mse of the first 3 samples (ARM nb=3) matches ARM's reference within 120 dB and 1e-5 relative") { armMse(3, 0); }
TEST_CASE("[C1 arm_mse_f32] mse of the first 8 samples (ARM nb=4n) matches ARM's reference within 120 dB and 1e-5 relative") { armMse(8, 1); }
TEST_CASE("[C1 arm_mse_f32] mse of the first 11 samples (ARM nb=4n+1) matches ARM's reference within 120 dB and 1e-5 relative") { armMse(11, 2); }
TEST_CASE("[C1 arm_mse_f32] mse of the first 100 samples (ARM long) matches ARM's reference within 120 dB and 1e-5 relative") { armMse(100, 3); }

TEST_CASE("[C1 arm_std_f32] std of {4,7,13,16} + 3e4 stays within 1e-4 of 5.47723 (ARM stability test)") {
	// ARM StatsTestsF32.cpp:230-:264: the textbook formula would give a negative variance here
	float in[4] = {4.0f, 7.0f, 13.0f, 16.0f};
	for (int i = 0; i < 4; i++) in[i] += 3.0e4f;
	float result = -1.0f;
	arm_std_f32(in, 4, &result);
	CHECK(std::fabs(5.47723f - result) < 1.0e-4f);
}

TEST_CASE("[C1 arm_entropy_f32] entropy of ARM's 11 vectors matches ARM's reference within 120 dB and 1e-5 relative") {
	const std::vector<float> in = pat("Input22_f32.txt"), ref = pat("RefEntropy22_f32.txt");
	const std::vector<int16_t> dims = patIdx("Dims22_s16.txt");
	const size_t np = static_cast<size_t>(dims[0]);
	REQUIRE(ref.size() == np);
	std::vector<float> out;
	size_t off = 0;
	for (size_t i = 0; i < np; i++) {
		REQUIRE(in.size() >= off + static_cast<size_t>(dims[i + 1]));
		out.push_back(arm_entropy_f32(in.data() + off, static_cast<uint32_t>(dims[i + 1])));
		off += static_cast<size_t>(dims[i + 1]);
	}
	checkArm(ref, out);
}

TEST_CASE("[C1 arm_logsumexp_f32] logsumexp of ARM's vectors matches ARM's reference within 120 dB and 1e-5 relative") {
	const std::vector<float> in = pat("Input23_f32.txt"), ref = pat("RefLogSumExp23_f32.txt");
	const std::vector<int16_t> dims = patIdx("Dims23_s16.txt");
	const size_t np = static_cast<size_t>(dims[0]);
	REQUIRE(ref.size() == np);
	std::vector<float> out;
	size_t off = 0;
	for (size_t i = 0; i < np; i++) {
		REQUIRE(in.size() >= off + static_cast<size_t>(dims[i + 1]));
		out.push_back(arm_logsumexp_f32(in.data() + off, static_cast<uint32_t>(dims[i + 1])));
		off += static_cast<size_t>(dims[i + 1]);
	}
	checkArm(ref, out);
}

TEST_CASE("[C1 arm_kullback_leibler_f32] KL divergence of ARM's vector pairs matches ARM's reference within 120 dB and 1e-5 relative") {
	const std::vector<float> a = pat("InputA24_f32.txt"), b = pat("InputB24_f32.txt"), ref = pat("RefKL24_f32.txt");
	const std::vector<int16_t> dims = patIdx("Dims24_s16.txt");
	const size_t np = static_cast<size_t>(dims[0]);
	REQUIRE(ref.size() == np);
	std::vector<float> out;
	size_t off = 0;
	for (size_t i = 0; i < np; i++) {
		REQUIRE(a.size() >= off + static_cast<size_t>(dims[i + 1]));
		REQUIRE(b.size() >= off + static_cast<size_t>(dims[i + 1]));
		out.push_back(arm_kullback_leibler_f32(a.data() + off, b.data() + off, static_cast<uint32_t>(dims[i + 1])));
		off += static_cast<size_t>(dims[i + 1]);
	}
	checkArm(ref, out);
}

TEST_CASE("[C1 arm_logsumexp_dot_prod_f32] log dot product of ARM's vector pairs matches ARM's reference within 120 dB and 1e-5 relative") {
	const std::vector<float> a = pat("InputA25_f32.txt"), b = pat("InputB25_f32.txt"), ref = pat("RefLogSumExpDot25_f32.txt");
	const std::vector<int16_t> dims = patIdx("Dims25_s16.txt");
	const size_t np = static_cast<size_t>(dims[0]);
	REQUIRE(ref.size() == np);
	std::vector<float> out, tmp(12);   // 12 is the maximum vector length of ARM's generator (setUp: tmp.create(12))
	size_t off = 0;
	for (size_t i = 0; i < np; i++) {
		REQUIRE(a.size() >= off + static_cast<size_t>(dims[i + 1]));
		REQUIRE(b.size() >= off + static_cast<size_t>(dims[i + 1]));
		REQUIRE(static_cast<size_t>(dims[i + 1]) <= tmp.size());
		out.push_back(arm_logsumexp_dot_prod_f32(a.data() + off, b.data() + off, static_cast<uint32_t>(dims[i + 1]), tmp.data()));
		off += static_cast<size_t>(dims[i + 1]);
	}
	checkArm(ref, out);
}

TEST_CASE("[C1 arm_accumulate_f32] sums of the first 3, 8, 11 and 300 samples of ARM's input match ARM's reference within 120 dB and 1e-5 relative") {
	const std::vector<float> in = pat("InputAccumulate1_f32.txt"), ref = pat("RefAccumulate1_f32.txt");
	const std::vector<int16_t> dims = patIdx("InputAccumulateConfig1_s16.txt");
	const size_t np = static_cast<size_t>(dims[0]);
	REQUIRE(ref.size() == np);
	std::vector<float> out(np, -1.0f);
	for (size_t i = 0; i < np; i++) {
		REQUIRE(in.size() >= static_cast<size_t>(dims[i + 1]));
		arm_accumulate_f32(in.data(), static_cast<uint32_t>(dims[i + 1]), &out[i]);   // same pointer for each size, as in ARM
	}
	checkArm(ref, out);
}

// ===============================================================================================================
// C2: nothing written beyond the result, the index or the temporary buffer; inputs untouched
// ===============================================================================================================

TEST_CASE("[C2 arm_max_f32] value and index are written to exactly one word each for the block sizes 1, 2, 3, 4, 5 and 9 and are correct") {
	const float in[9] = {-4.0f, 7.5f, 0.25f, -9.0f, 3.0f, 6.0f, -1.0f, 2.0f, 5.0f};
	for (uint32_t n : kSizes) {
		CAPTURE(n);
		dsp::GuardedBuffer<float> r(1);
		dsp::GuardedBuffer<uint32_t> ix(1);
		arm_max_f32(in, n, r.data(), ix.data());
		const std::vector<float> x(in, in + n);
		const Ext e = defExt(MAX, x);
		CHECK(r.data()[0] == e.value);
		CHECK(ix.data()[0] == e.index);
		CHECK(r.guardIntact());
		CHECK(ix.guardIntact());
	}
}

TEST_CASE("[C2 arm_min_f32] value and index are written to exactly one word each for the block sizes 1, 2, 3, 4, 5 and 9 and are correct") {
	const float in[9] = {4.0f, -7.5f, 0.25f, 9.0f, -3.0f, -6.0f, 1.0f, -2.0f, -5.0f};
	for (uint32_t n : kSizes) {
		CAPTURE(n);
		dsp::GuardedBuffer<float> r(1);
		dsp::GuardedBuffer<uint32_t> ix(1);
		arm_min_f32(in, n, r.data(), ix.data());
		const std::vector<float> x(in, in + n);
		const Ext e = defExt(MIN, x);
		CHECK(r.data()[0] == e.value);
		CHECK(ix.data()[0] == e.index);
		CHECK(r.guardIntact());
		CHECK(ix.guardIntact());
	}
}

TEST_CASE("[C2 arm_absmax_f32] value and index are written to exactly one word each for the block sizes 1, 2, 3, 4, 5 and 9 and are correct") {
	const float in[9] = {-4.0f, 7.5f, 0.25f, -9.5f, 3.0f, 6.0f, -1.0f, 2.0f, 5.0f};
	for (uint32_t n : kSizes) {
		CAPTURE(n);
		dsp::GuardedBuffer<float> r(1);
		dsp::GuardedBuffer<uint32_t> ix(1);
		arm_absmax_f32(in, n, r.data(), ix.data());
		const std::vector<float> x(in, in + n);
		const Ext e = defExt(ABSMAX, x);
		CHECK(r.data()[0] == e.value);
		CHECK(ix.data()[0] == e.index);
		CHECK(r.guardIntact());
		CHECK(ix.guardIntact());
	}
}

TEST_CASE("[C2 arm_absmin_f32] value and index are written to exactly one word each for the block sizes 1, 2, 3, 4, 5 and 9 and are correct") {
	const float in[9] = {-4.0f, 7.5f, -3.25f, -9.5f, 3.0f, -0.5f, -1.0f, 2.0f, 5.0f};
	for (uint32_t n : kSizes) {
		CAPTURE(n);
		dsp::GuardedBuffer<float> r(1);
		dsp::GuardedBuffer<uint32_t> ix(1);
		arm_absmin_f32(in, n, r.data(), ix.data());
		const std::vector<float> x(in, in + n);
		const Ext e = defExt(ABSMIN, x);
		CHECK(r.data()[0] == e.value);
		CHECK(ix.data()[0] == e.index);
		CHECK(r.guardIntact());
		CHECK(ix.guardIntact());
	}
}

TEST_CASE("[C2 arm_max_no_idx_f32] the four no-index functions write exactly one word for the block sizes 1, 2, 3, 4, 5 and 9 and are correct") {
	const float in[9] = {-4.0f, 7.5f, -3.25f, -9.5f, 3.0f, -0.5f, -1.0f, 2.0f, 5.0f};
	for (int m = 0; m < 4; m++) {
		for (uint32_t n : kSizes) {
			CAPTURE(m);
			CAPTURE(n);
			dsp::GuardedBuffer<float> r(1);
			kNoIdx[m](in, n, r.data());
			const std::vector<float> x(in, in + n);
			CHECK(r.data()[0] == defExt(static_cast<Mode>(m), x).value);
			CHECK(r.guardIntact());
		}
	}
}

TEST_CASE("[C2 arm_accumulate_f32] the sum is written to exactly one word for the block sizes 1, 2, 3, 4, 5 and 9 and is correct") {
	const float in[9] = {1.0f, 2.0f, 4.0f, 8.0f, 16.0f, 32.0f, 64.0f, 128.0f, 256.0f};   // powers of two: every partial sum exact
	for (uint32_t n : kSizes) {
		CAPTURE(n);
		dsp::GuardedBuffer<float> r(1);
		arm_accumulate_f32(in, n, r.data());
		CHECK(r.data()[0] == static_cast<float>((1u << n) - 1u));   // 1+2+...+2^(n-1) = 2^n - 1
		CHECK(r.guardIntact());
	}
}

TEST_CASE("[C2 arm_mean_f32] mean, power, rms, std, var and mse write exactly one word, with the expected value, for the block sizes 1, 2, 3, 4, 5 and 9") {
	// data 1..9 (integers): mean (n+1)/2, power n(n+1)(2n+1)/6, var n(n+1)/12 (n >= 2). mse against 2: sum (i-2)^2 / n. Expected values in double.
	const float a[9] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f};
	const float b[9] = {2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f};
	const ScalarFn fns[5] = {arm_mean_f32, arm_power_f32, arm_rms_f32, arm_std_f32, arm_var_f32};
	for (uint32_t n : kSizes) {
		CAPTURE(n);
		const double dn = n;
		double expect[5] = {(dn + 1) / 2, dn * (dn + 1) * (2 * dn + 1) / 6, 0, 0, dn * (dn + 1) / 12};
		expect[2] = std::sqrt(expect[1] / dn);
		expect[3] = std::sqrt(expect[4]);
		for (int f = 0; f < 5; f++) {
			if (n == 1 && f >= 3) continue;   // var / std of one sample: characterization test
			CAPTURE(f);
			dsp::GuardedBuffer<float> r(1);
			fns[f](a, n, r.data());
			CHECK(r.guardIntact());
			CHECK(std::fabs(r.data()[0] - expect[f]) <= 4.0 * kEps * expect[f]);   // 4 eps: a few correctly rounded operations on exact data
		}
		dsp::GuardedBuffer<float> r(1);
		arm_mse_f32(a, b, n, r.data());
		CHECK(r.guardIntact());
		double m = 0;
		for (uint32_t i = 0; i < n; i++) m += (a[i] - 2.0) * (a[i] - 2.0);
		m /= dn;
		CHECK(std::fabs(r.data()[0] - m) <= 4.0 * kEps * m);
	}
}

TEST_CASE("[C2 arm_logsumexp_dot_prod_f32] the temporary buffer of blockSize words is not overrun for the block sizes 1, 2, 3, 4, 5 and 9") {
	const float a[9] = {0.5f, -1.0f, 2.0f, 0.0f, 1.5f, -2.5f, 3.0f, 0.25f, -0.75f};
	const float b[9] = {1.0f, 0.5f, -1.0f, 2.0f, -0.5f, 1.5f, 0.0f, 1.25f, 0.75f};
	for (uint32_t n : kSizes) {
		CAPTURE(n);
		dsp::GuardedBuffer<float> tmp(n);
		const float r = arm_logsumexp_dot_prod_f32(a, b, n, tmp.data());
		CHECK(tmp.guardIntact());
		std::vector<double> t(n);
		for (uint32_t i = 0; i < n; i++) t[i] = static_cast<double>(a[i]) + b[i];
		const Lse d = defLse(t);
		double amax = 0;
		for (double v : t) amax = std::fmax(amax, std::fabs(v));
		CHECK(std::fabs(r - d.value) <= lseBound(d, n) + kEps * amax);
	}
}

// ===============================================================================================================
// C3: DEF (double) for the max / min family
// ===============================================================================================================
namespace {

void idxRandom(Mode m) {
	Rng rng(777u + static_cast<uint32_t>(m));
	for (size_t n = 1; n <= 40; n++) {
		for (int trial = 0; trial < 20; trial++) {
			CAPTURE(n);
			CAPTURE(trial);
			const std::vector<float> x = randVec(rng, n, -100.0f, 100.0f);   // continuous data: ties have probability ~ n^2 / 2^24
			const Ext e = defExt(m, x);
			float v = -1.0f;
			uint32_t ix = 0xFFFFFFFFu;
			kIdx[m](x.data(), static_cast<uint32_t>(n), &v, &ix);
			CHECK(v == e.value);
			CHECK(ix == e.index);
			float v2 = -1.0f;
			kNoIdx[m](x.data(), static_cast<uint32_t>(n), &v2);
			CHECK(v2 == e.value);
		}
	}
}

// The extreme sits at every position of blocks of 1..14 samples (all unroll groups and tail positions); the other samples are distinct
void idxPosition(Mode m) {
	for (size_t n = 1; n <= 14; n++) {
		for (size_t pos = 0; pos < n; pos++) {
			CAPTURE(n);
			CAPTURE(pos);
			std::vector<float> x(n);
			for (size_t i = 0; i < n; i++) {
				const float mag = 1.0f + 0.125f * static_cast<float>(i);                  // 1 .. 2.625, distinct
				x[i] = (i % 2 == 0) ? mag : -mag;                                         // mixed signs
			}
			float expect;
			switch (m) {
			case MAX: x[pos] = 100.0f; expect = 100.0f; break;
			case MIN: x[pos] = -100.0f; expect = -100.0f; break;
			case ABSMAX: x[pos] = -100.0f; expect = 100.0f; break;
			default: x[pos] = -0.0625f; expect = 0.0625f; break;
			}
			float v = -1.0f;
			uint32_t ix = 0xFFFFFFFFu;
			kIdx[m](x.data(), static_cast<uint32_t>(n), &v, &ix);
			CHECK(v == expect);
			CHECK(ix == pos);
			float v2 = -1.0f;
			kNoIdx[m](x.data(), static_cast<uint32_t>(n), &v2);
			CHECK(v2 == expect);
		}
	}
}

// Strided sweep over all finite binary32 bit patterns (see sweepBlocks): blocks of 13 samples (3 unrolled groups + 1)
void idxFullRange(Mode m) {
	size_t bad = 0;
	std::string first;
	sweepBlocks(13, 6000, [&](const std::vector<float> &x) {
		const Ext e = defExt(m, x);
		float v = 0.0f, v2 = 0.0f;
		uint32_t ix = 0;
		kIdx[m](x.data(), 13, &v, &ix);
		kNoIdx[m](x.data(), 13, &v2);
		if (!(v == e.value && ix == e.index && v2 == e.value)) {
			if (!bad) first = "value " + std::to_string(v) + " (no_idx " + std::to_string(v2) + "), index " + std::to_string(ix) + " expected " + std::to_string(e.value) + ", " + std::to_string(e.index);
			bad++;
		}
	});
	CHECK_MESSAGE(bad == 0, bad << " of 6000 blocks differ, first: " << first);
}

void idxSingle(Mode m) {
	const float vals[] = {0.0f, 1.0f, -1.0f, 3.5f, -3.5f, FLT_MAX, -FLT_MAX, FLT_MIN, -FLT_MIN, 1.0e-45f, -1.0e-45f};
	for (float s : vals) {
		CAPTURE(s);
		float v = -7.0f;
		uint32_t ix = 99;
		kIdx[m](&s, 1, &v, &ix);
		const double expect = (m == ABSMAX || m == ABSMIN) ? std::fabs(static_cast<double>(s)) : static_cast<double>(s);
		CHECK(v == expect);
		CHECK(ix == 0u);
		float v2 = -7.0f;
		kNoIdx[m](&s, 1, &v2);
		CHECK(v2 == expect);
	}
}

// All ordered pairs of distinct values, including negatives and zero
void idxPairs(Mode m) {
	const float vals[] = {-3.0f, -1.0f, -0.5f, 0.0f, 0.5f, 2.0f};
	for (float a : vals) {
		for (float b : vals) {
			if (keyOf(m, a) == keyOf(m, b)) continue;   // ties are the characterization tests
			CAPTURE(a);
			CAPTURE(b);
			const std::vector<float> x = {a, b};
			const Ext e = defExt(m, x);
			float v = -7.0f;
			uint32_t ix = 99;
			kIdx[m](x.data(), 2, &v, &ix);
			CHECK(v == e.value);
			CHECK(ix == e.index);
			float v2 = -7.0f;
			kNoIdx[m](x.data(), 2, &v2);
			CHECK(v2 == e.value);
		}
	}
}

// All samples negative: max is the one closest to zero, absmax the most negative. The data are distinct.
void idxNegative(Mode m) {
	for (size_t n : {3u, 4u, 5u, 8u, 9u}) {
		CAPTURE(n);
		std::vector<float> x(n);
		for (size_t i = 0; i < n; i++) x[i] = -1.0f - 0.5f * static_cast<float>((i * 5) % n);   // permutation of -1 .. -(1+0.5(n-1))
		const Ext e = defExt(m, x);
		float v = 1.0f;
		uint32_t ix = 99;
		kIdx[m](x.data(), static_cast<uint32_t>(n), &v, &ix);
		CHECK(v == e.value);
		CHECK(ix == e.index);
	}
}

// Ties: DOC is silent. The source uses a strict comparison so the first occurrence wins.
void idxTies(Mode m) {
	// extreme value at the positions 3 and 7 (and for abs: with opposite signs), block of 10
	std::vector<float> x(10);
	for (size_t i = 0; i < 10; i++) x[i] = 0.25f * static_cast<float>(i % 3) + 1.0f;   // 1.0, 1.25, 1.5 repeated: below the extreme
	float hi, lo;
	switch (m) {
	case MAX: hi = 50.0f; lo = 50.0f; break;
	case MIN: hi = -50.0f; lo = -50.0f; break;
	case ABSMAX: hi = -50.0f; lo = 50.0f; break;
	default: hi = 0.0f; lo = -0.0f; break;        // absmin: +0 and -0 have the same absolute value (all other samples are >= 1)
	}
	x[3] = hi;
	x[7] = lo;
	float v = -1.0f;
	uint32_t ix = 99;
	kIdx[m](x.data(), 10, &v, &ix);
	CHECK(ix == 3u);
	// all samples equal, every block size: index 0
	for (size_t n = 1; n <= 9; n++) {
		CAPTURE(n);
		const std::vector<float> c(n, 2.5f);
		uint32_t ix2 = 99;
		float v2 = 0.0f;
		kIdx[m](c.data(), static_cast<uint32_t>(n), &v2, &ix2);
		CHECK(v2 == 2.5f);
		CHECK(ix2 == 0u);
	}
}

}  // namespace

TEST_CASE("[C3 arm_max_f32] value and index of random blocks of 1..40 samples equal the double definition (also max_no_idx)") { idxRandom(MAX); }
TEST_CASE("[C3 arm_min_f32] value and index of random blocks of 1..40 samples equal the double definition (also min_no_idx)") { idxRandom(MIN); }
TEST_CASE("[C3 arm_absmax_f32] value and index of random blocks of 1..40 samples equal the double definition (also absmax_no_idx)") { idxRandom(ABSMAX); }
TEST_CASE("[C3 arm_absmin_f32] value and index of random blocks of 1..40 samples equal the double definition (also absmin_no_idx)") { idxRandom(ABSMIN); }

TEST_CASE("[C3 arm_max_f32] the maximum is found at every position of blocks of 1..14 samples") { idxPosition(MAX); }
TEST_CASE("[C3 arm_min_f32] the minimum is found at every position of blocks of 1..14 samples") { idxPosition(MIN); }
TEST_CASE("[C3 arm_absmax_f32] a negative absolute maximum is found at every position of blocks of 1..14 samples and returned as a magnitude") { idxPosition(ABSMAX); }
TEST_CASE("[C3 arm_absmin_f32] a negative absolute minimum is found at every position of blocks of 1..14 samples and returned as a magnitude") { idxPosition(ABSMIN); }

TEST_CASE("[C3 arm_max_f32] a strided sample of the finite float bit patterns (6000 blocks of 13) agrees with the double definition (also max_no_idx)") { idxFullRange(MAX); }
TEST_CASE("[C3 arm_min_f32] a strided sample of the finite float bit patterns (6000 blocks of 13) agrees with the double definition (also min_no_idx)") { idxFullRange(MIN); }
TEST_CASE("[C3 arm_absmax_f32] a strided sample of the finite float bit patterns (6000 blocks of 13) agrees with the double definition (also absmax_no_idx)") { idxFullRange(ABSMAX); }
TEST_CASE("[C3 arm_absmin_f32] a strided sample of the finite float bit patterns (6000 blocks of 13) agrees with the double definition (also absmin_no_idx)") { idxFullRange(ABSMIN); }

TEST_CASE("[C3 arm_max_f32] one sample (positive, negative, zero, FLT_MAX, FLT_MIN, subnormal) is its own maximum at index 0 (also max_no_idx)") { idxSingle(MAX); }
TEST_CASE("[C3 arm_min_f32] one sample (positive, negative, zero, FLT_MAX, FLT_MIN, subnormal) is its own minimum at index 0 (also min_no_idx)") { idxSingle(MIN); }
TEST_CASE("[C3 arm_absmax_f32] one sample (positive, negative, zero, FLT_MAX, FLT_MIN, subnormal) gives its magnitude at index 0 (also absmax_no_idx)") { idxSingle(ABSMAX); }
TEST_CASE("[C3 arm_absmin_f32] one sample (positive, negative, zero, FLT_MAX, FLT_MIN, subnormal) gives its magnitude at index 0 (also absmin_no_idx)") { idxSingle(ABSMIN); }

TEST_CASE("[C4 arm_max_f32] every ordered pair of distinct values (block size 2) gives the larger value and its index (also max_no_idx)") { idxPairs(MAX); }
TEST_CASE("[C4 arm_min_f32] every ordered pair of distinct values (block size 2) gives the smaller value and its index (also min_no_idx)") { idxPairs(MIN); }
TEST_CASE("[C4 arm_absmax_f32] every ordered pair with different magnitudes (block size 2) gives the larger magnitude and its index (also absmax_no_idx)") { idxPairs(ABSMAX); }
TEST_CASE("[C4 arm_absmin_f32] every ordered pair with different magnitudes (block size 2) gives the smaller magnitude and its index (also absmin_no_idx)") { idxPairs(ABSMIN); }

TEST_CASE("[C4 arm_max_f32] blocks of only negative samples give the sample closest to zero") { idxNegative(MAX); }
TEST_CASE("[C4 arm_min_f32] blocks of only negative samples give the most negative sample") { idxNegative(MIN); }
TEST_CASE("[C4 arm_absmax_f32] blocks of only negative samples give the largest magnitude") { idxNegative(ABSMAX); }
TEST_CASE("[C4 arm_absmin_f32] blocks of only negative samples give the smallest magnitude") { idxNegative(ABSMIN); }

TEST_CASE("characterization: [C4 arm_max_f32] equal maxima return the lowest index, and a block of equal samples returns index 0") { idxTies(MAX); }
TEST_CASE("characterization: [C4 arm_min_f32] equal minima return the lowest index, and a block of equal samples returns index 0") { idxTies(MIN); }
TEST_CASE("characterization: [C4 arm_absmax_f32] equal absolute maxima (opposite signs) return the lowest index, and a block of equal samples returns index 0") { idxTies(ABSMAX); }
TEST_CASE("characterization: [C4 arm_absmin_f32] equal absolute minima (+0 and -0) return the lowest index, and a block of equal samples returns index 0") { idxTies(ABSMIN); }

// ---- infinities (ids B-DSF32-1 / B-DSF32-2 below) ----
namespace {
const float kInf = std::numeric_limits<float>::infinity();
}

TEST_CASE("[C4 arm_max_no_idx_f32] a block of only -FLT_MAX returns -FLT_MAX") {
	for (size_t n : {1u, 4u, 5u, 9u}) {
		CAPTURE(n);
		const std::vector<float> x(n, -FLT_MAX);
		float r = 0.0f;
		arm_max_no_idx_f32(x.data(), static_cast<uint32_t>(n), &r);
		CHECK(r == -FLT_MAX);
	}
}

TEST_CASE("[C4 arm_min_no_idx_f32] a block of only FLT_MAX returns FLT_MAX") {
	for (size_t n : {1u, 4u, 5u, 9u}) {
		CAPTURE(n);
		const std::vector<float> x(n, FLT_MAX);
		float r = 0.0f;
		arm_min_no_idx_f32(x.data(), static_cast<uint32_t>(n), &r);
		CHECK(r == FLT_MAX);
	}
}

TEST_CASE("[C4 arm_max_no_idx_f32] a block of only -infinity returns -infinity" * KNOWN_ISSUE("B-DSF32-1: arm_max_no_idx_f32.c:116 starts from F32_MIN (-FLT_MAX), so a block of only -inf returns -FLT_MAX instead of -inf")) {
	// DEF: the maximum of {-inf, ..., -inf} is -inf (arm_max_f32 and arm_absmax_no_idx_f32 return it)
	for (size_t n : {1u, 4u, 5u, 9u}) {
		CAPTURE(n);
		const std::vector<float> x(n, -kInf);
		float r = 0.0f;
		arm_max_no_idx_f32(x.data(), static_cast<uint32_t>(n), &r);
		CHECK(r == -kInf);
	}
}

TEST_CASE("[C4 arm_min_no_idx_f32] a block of only +infinity returns +infinity" * KNOWN_ISSUE("B-DSF32-2: arm_min_no_idx_f32.c:116 starts from F32_MAX (FLT_MAX), so a block of only +inf returns FLT_MAX instead of +inf")) {
	for (size_t n : {1u, 4u, 5u, 9u}) {
		CAPTURE(n);
		const std::vector<float> x(n, kInf);
		float r = 0.0f;
		arm_min_no_idx_f32(x.data(), static_cast<uint32_t>(n), &r);
		CHECK(r == kInf);
	}
}

TEST_CASE("[C4 arm_max_f32] infinities: -inf only, mixed with finite values, and the other six units agree with the definition") {
	// every unit except max_no_idx (B-DSF32-1) and min_no_idx (B-DSF32-2) with the only-infinite blocks; DEF compares in double
	const std::vector<std::vector<float>> blocks = {
		{-kInf}, {kInf}, {-kInf, -kInf, -kInf, -kInf, -kInf}, {kInf, kInf, kInf, kInf, kInf},
		{1.0f, kInf, -kInf, 3.0f, -2.0f}, {-kInf, -5.0f, 2.0f, kInf, 0.0f, 7.0f, -kInf, 1.0f, 2.0f}, {-kInf, -kInf, 3.0f}, {kInf, kInf, -3.0f}};
	for (int m = 0; m < 4; m++) {
		for (size_t b = 0; b < blocks.size(); b++) {
			const std::vector<float> &x = blocks[b];
			CAPTURE(m);
			CAPTURE(b);
			const Ext e = defExt(static_cast<Mode>(m), x);
			float v = 0.0f;
			uint32_t ix = 99;
			kIdx[m](x.data(), static_cast<uint32_t>(x.size()), &v, &ix);
			CHECK(v == e.value);
			CHECK(ix == e.index);
			if (m >= 2) {   // absmax / absmin no_idx
				float v2 = 0.0f;
				kNoIdx[m](x.data(), static_cast<uint32_t>(x.size()), &v2);
				CHECK(v2 == e.value);
			}
		}
	}
}

TEST_CASE("[C3 arm_max_no_idx_f32] max_no_idx and min_no_idx agree with the definition on blocks mixing infinities and finite values") {
	// blocks where the result is finite or the infinity is the extreme AND a larger/smaller finite value exists, so the start value does not matter
	const std::vector<std::vector<float>> blocks = {{1.0f, kInf, -kInf, 3.0f, -2.0f}, {-kInf, -5.0f, 2.0f}, {kInf, -3.0f, 4.0f}, {-kInf, kInf}};
	for (size_t b = 0; b < blocks.size(); b++) {
		CAPTURE(b);
		float vmax = 0.0f, vmin = 0.0f;
		arm_max_no_idx_f32(blocks[b].data(), static_cast<uint32_t>(blocks[b].size()), &vmax);
		arm_min_no_idx_f32(blocks[b].data(), static_cast<uint32_t>(blocks[b].size()), &vmin);
		CHECK(vmax == defExt(MAX, blocks[b]).value);
		CHECK(vmin == defExt(MIN, blocks[b]).value);
	}
}

// ===============================================================================================================
// C3: DEF for accumulate, mean, power, rms, mse, var, std (bounds derived in the header)
// ===============================================================================================================
namespace {

struct DataClass { float lo, hi; const char *name; };
// signed data (cancellation in sums), positive data (no cancellation: relative bound is tight), data with a large offset (var / std)
const DataClass kClasses[] = {{-10.0f, 10.0f, "signed"}, {1.0f, 2.0f, "positive"}, {99.0f, 101.0f, "offset"}};

}  // namespace

TEST_CASE("[C3 arm_accumulate_f32] the sum of random blocks of 1..40 samples equals the double sum within (n-1)*eps*sum|x|") {
	Rng rng(101);
	for (const DataClass &dc : kClasses) {
		for (size_t n = 1; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<float> x = randVec(rng, n, dc.lo, dc.hi), orig = x;
			float r = -1.0f;
			arm_accumulate_f32(x.data(), static_cast<uint32_t>(n), &r);
			CHECK(std::fabs(r - sumOf(x)) <= gam(static_cast<double>(n) - 1.0) * sumAbs(x));
			CHECK(x == orig);
		}
	}
}

TEST_CASE("[C3 arm_mean_f32] the mean of random blocks of 1..40 samples equals the double mean within n*eps*sum|x|/n") {
	Rng rng(102);
	for (const DataClass &dc : kClasses) {
		for (size_t n = 1; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<float> x = randVec(rng, n, dc.lo, dc.hi), orig = x;
			float r = -1.0f;
			arm_mean_f32(x.data(), static_cast<uint32_t>(n), &r);
			CHECK(std::fabs(r - defMean(x)) <= gam(static_cast<double>(n)) * sumAbs(x) / static_cast<double>(n));
			CHECK(x == orig);
		}
	}
}

TEST_CASE("[C3 arm_power_f32] the power of random blocks of 1..40 samples equals the double sum of squares within (n)*eps relative") {
	Rng rng(103);
	for (const DataClass &dc : kClasses) {
		for (size_t n = 1; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<float> x = randVec(rng, n, dc.lo, dc.hi), orig = x;
			float r = -1.0f;
			arm_power_f32(x.data(), static_cast<uint32_t>(n), &r);
			const double p = defPower(x);
			CHECK(std::fabs(r - p) <= gam(static_cast<double>(n)) * p);
			CHECK(x == orig);
		}
	}
}

TEST_CASE("[C3 arm_rms_f32] the rms of random blocks of 1..40 samples equals sqrt(sum of squares / n) within (n+1)*eps/2 + eps relative") {
	Rng rng(104);
	for (const DataClass &dc : kClasses) {
		for (size_t n = 1; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<float> x = randVec(rng, n, dc.lo, dc.hi);
			float r = -1.0f;
			arm_rms_f32(x.data(), static_cast<uint32_t>(n), &r);
			const double d = std::sqrt(defPower(x) / static_cast<double>(n));
			CHECK(std::fabs(r - d) <= (0.5 * gam(static_cast<double>(n) + 1.0) + 1.01 * kEps) * d);
		}
	}
}

TEST_CASE("[C3 arm_mse_f32] the mse of random block pairs of 1..40 samples equals the double mean of squared differences within (n+3)*eps relative") {
	Rng rng(105);
	for (const DataClass &dc : kClasses) {
		for (size_t n = 1; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<float> a = randVec(rng, n, dc.lo, dc.hi);
			const std::vector<float> b = randVec(rng, n, dc.lo, dc.hi);
			float r = -1.0f;
			arm_mse_f32(a.data(), b.data(), static_cast<uint32_t>(n), &r);
			double d = 0;
			for (size_t i = 0; i < n; i++) d += (static_cast<double>(a[i]) - b[i]) * (static_cast<double>(a[i]) - b[i]);
			d /= static_cast<double>(n);
			CHECK(std::fabs(r - d) <= gam(static_cast<double>(n) + 3.0) * d);
		}
	}
}

namespace {

// Bound of the variance derived in the header
double varBound(const std::vector<float> &x) {
	const double n = static_cast<double>(x.size());
	const double D = gam(n) * sumAbs(x) / n;
	return defVar(x) * gam(n + 3.0) + n * D * D / (n - 1.0) * (1.0 + gam(n + 3.0));
}

}  // namespace

// ---- NaN and infinity through sqrt / logsumexp (ids B-DSF32-3 .. 5) ----
TEST_CASE("[C4 arm_var_f32] the variance of {1, NaN, 3} and of {inf, 1} is NaN") {
	const float a[3] = {1.0f, std::numeric_limits<float>::quiet_NaN(), 3.0f}, b[2] = {kInf, 1.0f};
	float r = 0.0f;
	arm_var_f32(a, 3, &r);
	CHECK(std::isnan(r));
	r = 0.0f;
	arm_var_f32(b, 2, &r);   // mean inf, inf - inf = NaN
	CHECK(std::isnan(r));
}

TEST_CASE("[C4 arm_std_f32] the standard deviation of {1, NaN, 3} is NaN" * KNOWN_ISSUE("B-DSF32-3: arm_std_f32.c:80 passes the NaN variance to arm_sqrt_f32, whose guard 'in >= 0.0f' fails and writes 0, so std is 0 where var is NaN (arm_std_f64 gives NaN)")) {
	const float a[3] = {1.0f, std::numeric_limits<float>::quiet_NaN(), 3.0f};
	float r = -1.0f;
	arm_std_f32(a, 3, &r);
	CHECK(std::isnan(r));
}

TEST_CASE("[C4 arm_std_f32] the standard deviation of {inf, 1} is NaN" * KNOWN_ISSUE("B-DSF32-3: same defect, the variance of {inf, 1} is NaN and std returns 0")) {
	const float b[2] = {kInf, 1.0f};
	float r = -1.0f;
	arm_std_f32(b, 2, &r);
	CHECK(std::isnan(r));
}

TEST_CASE("[C4 arm_rms_f32] the rms of {1, NaN, 3} is NaN" * KNOWN_ISSUE("B-DSF32-4: arm_rms_f32.c:126 / :187 pass the NaN mean square to arm_sqrt_f32, whose guard 'in >= 0.0f' fails and writes 0, so rms is 0 for NaN input")) {
	const float a[3] = {1.0f, std::numeric_limits<float>::quiet_NaN(), 3.0f};
	float r = -1.0f;
	arm_rms_f32(a, 3, &r);
	CHECK(std::isnan(r));
}

TEST_CASE("[C4 arm_logsumexp_f32] logsumexp of {-inf, 1} is 1") {
	const float a[2] = {-kInf, 1.0f};
	CHECK(std::fabs(arm_logsumexp_f32(a, 2) - 1.0f) <= 2.0f * static_cast<float>(kEps));
}

TEST_CASE("[C4 arm_logsumexp_f32] logsumexp of {-inf, -inf} is -inf" * KNOWN_ISSUE("B-DSF32-5: arm_logsumexp_f32.c:251-271 the maximum is -inf, so expf(-inf - -inf) = expf(NaN) and the result is NaN (math: -inf)")) {
	// arm_logsumexp_dot_prod_f32 calls arm_logsumexp_f32 and inherits the defect
	const float a[2] = {-kInf, -kInf};
	CHECK(arm_logsumexp_f32(a, 2) == -kInf);
}

TEST_CASE("[C4 arm_logsumexp_f32] logsumexp of {inf, 1, 2} is inf" * KNOWN_ISSUE("B-DSF32-5: the maximum is +inf, so expf(inf - inf) = NaN and the result is NaN (math: +inf)")) {
	const float a[3] = {kInf, 1.0f, 2.0f};
	CHECK(arm_logsumexp_f32(a, 3) == kInf);
}

// ---- long block (n = 4096) with the derived bounds ----
TEST_CASE("[C3 arm_mean_f32] blocks of 4096 samples (+-1 alternating, and random in [1,2]) satisfy the derived gam(n) bounds for mean, power, rms, var and std") {
	const size_t n = 4096;
	const double dn = static_cast<double>(n);
	Rng rng(4096);
	for (int cls = 0; cls < 2; cls++) {
		std::vector<float> x(n);
		if (cls == 0) { for (size_t i = 0; i < n; i++) x[i] = (i % 2) ? -1.0f : 1.0f; }   // mean 0, power 4096, var 4096/4095
		else x = randVec(rng, n, 1.0f, 2.0f);
		CAPTURE(cls);
		float m = -9.0f, p = -9.0f, r = -9.0f, v = -9.0f, sd = -9.0f;
		arm_mean_f32(x.data(), static_cast<uint32_t>(n), &m);
		arm_power_f32(x.data(), static_cast<uint32_t>(n), &p);
		arm_rms_f32(x.data(), static_cast<uint32_t>(n), &r);
		arm_var_f32(x.data(), static_cast<uint32_t>(n), &v);
		arm_std_f32(x.data(), static_cast<uint32_t>(n), &sd);
		CHECK(std::fabs(m - defMean(x)) <= gam(dn) * sumAbs(x) / dn);
		const double P = defPower(x);
		CHECK(std::fabs(p - P) <= gam(dn) * P);
		const double R = std::sqrt(P / dn);
		CHECK(std::fabs(r - R) <= (0.5 * gam(dn + 1.0) + 1.01 * kEps) * R);
		const double V = defVar(x);
		CHECK(std::fabs(v - V) <= varBound(x));
		const double S = std::sqrt(V);
		CHECK(std::fabs(sd - S) <= S * (0.5 * (varBound(x) / V) * 1.01 + 1.01 * kEps));
	}
}

TEST_CASE("[C3 arm_var_f32] the variance of random blocks of 2..40 samples equals the double n-1 variance within the two pass bound") {
	Rng rng(106);
	for (const DataClass &dc : kClasses) {
		for (size_t n = 2; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<float> x = randVec(rng, n, dc.lo, dc.hi), orig = x;
			float r = -1.0f;
			arm_var_f32(x.data(), static_cast<uint32_t>(n), &r);
			CHECK(std::fabs(r - defVar(x)) <= varBound(x));
			CHECK(x == orig);
		}
	}
}

TEST_CASE("[C3 arm_std_f32] the standard deviation of random blocks of 2..40 samples equals sqrt of the double n-1 variance within the derived bound") {
	Rng rng(107);
	for (const DataClass &dc : kClasses) {
		for (size_t n = 2; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<float> x = randVec(rng, n, dc.lo, dc.hi);
			float r = -1.0f;
			arm_std_f32(x.data(), static_cast<uint32_t>(n), &r);
			const double v = defVar(x), s = std::sqrt(v);
			CHECK(std::fabs(r - s) <= s * (0.5 * (varBound(x) / v) * 1.01 + 1.01 * kEps));
		}
	}
}

TEST_CASE("[C3 arm_var_f32] the variance uses n-1: {1, 2, 3} gives exactly 1 and {1, 4} gives exactly 4.5") {
	// hand derivation: {1,2,3}: mean 2, deviations -1, 0, 1, sum of squares 2, / (3-1) = 1. {1,4}: mean 2.5, deviations +-1.5, 4.5 / 1 = 4.5.
	// every intermediate value is a dyadic number, so the result is exact on any platform.
	const float a[3] = {1.0f, 2.0f, 3.0f}, b[2] = {1.0f, 4.0f};
	float r = -1.0f;
	arm_var_f32(a, 3, &r);
	CHECK(r == 1.0f);
	arm_var_f32(b, 2, &r);
	CHECK(r == 4.5f);
}

TEST_CASE("[C3 arm_std_f32] the standard deviation of {1, 3} is sqrt(2) and of {30004, 30007, 30013, 30016} is sqrt(30), both correctly rounded") {
	// {1,3}: mean 2, deviations -+1, sum 2, / 1 = 2, std = sqrt(2).
	// {30004,30007,30013,30016}: all exact floats (spacing 2^-9 at 3e4), sum 120040, mean 30010 exact, deviations -6, -3, 3, 6, sum of squares 90,
	// / 3 = 30, std = sqrt(30). The library takes the square root of an exactly computed variance, so the result is the rounded square root
	// (sqrt of a double rounded to float is correctly rounded because 53 >= 2*24+2).
	const float a[2] = {1.0f, 3.0f}, b[4] = {30004.0f, 30007.0f, 30013.0f, 30016.0f};
	float r = -1.0f;
	arm_std_f32(a, 2, &r);
	CHECK(r == static_cast<float>(std::sqrt(2.0)));
	arm_std_f32(b, 4, &r);
	CHECK(r == static_cast<float>(std::sqrt(30.0)));
}

// ===============================================================================================================
// C4: block size 1 and 2, all-equal data (dyadic: exact on any platform), exact one operation sweeps
// ===============================================================================================================

TEST_CASE("[C4 arm_mean_f32] blocks of 1..40 equal samples of 0.375 give exactly +-0.375 for the mean and 0.375 for the rms") {
	// 0.375 = 3/8: n * 0.375 is exact for n <= 40 (needs 6+2 bits), the division by n is exact, 0.375^2 = 9/64 exact, sqrt(9/64) = 0.375.
	for (size_t n = 1; n <= 40; n++) {
		for (float c : {0.375f, -0.375f}) {
			CAPTURE(n);
			CAPTURE(c);
			const std::vector<float> x(n, c);
			float r = -1.0f;
			arm_mean_f32(x.data(), static_cast<uint32_t>(n), &r);
			CHECK(r == c);
			arm_rms_f32(x.data(), static_cast<uint32_t>(n), &r);
			CHECK(r == 0.375f);
		}
	}
}

TEST_CASE("[C4 arm_power_f32] blocks of 1..40 equal samples of 0.375 give exactly n * 9/64, and the accumulate gives n * 0.375") {
	for (size_t n = 1; n <= 40; n++) {
		CAPTURE(n);
		const std::vector<float> x(n, 0.375f);
		float r = -1.0f;
		arm_power_f32(x.data(), static_cast<uint32_t>(n), &r);
		CHECK(r == static_cast<float>(n) * 0.140625f);
		arm_accumulate_f32(x.data(), static_cast<uint32_t>(n), &r);
		CHECK(r == static_cast<float>(n) * 0.375f);
	}
}

TEST_CASE("[C4 arm_var_f32] blocks of 2..40 equal samples have exactly zero variance and zero standard deviation") {
	// mean of equal dyadic samples is exact (see above), so all deviations are exactly 0
	for (size_t n = 2; n <= 40; n++) {
		CAPTURE(n);
		const std::vector<float> x(n, 0.375f);
		float r = -1.0f;
		arm_var_f32(x.data(), static_cast<uint32_t>(n), &r);
		CHECK(r == 0.0f);
		r = -1.0f;
		arm_std_f32(x.data(), static_cast<uint32_t>(n), &r);
		CHECK(r == 0.0f);
	}
}

TEST_CASE("[C4 arm_mse_f32] the mse of blocks of 1..40 identical vectors is exactly 0 and against a zero vector exactly 9/64") {
	for (size_t n = 1; n <= 40; n++) {
		CAPTURE(n);
		const std::vector<float> a(n, 0.375f), z(n, 0.0f);
		float r = -1.0f;
		arm_mse_f32(a.data(), a.data(), static_cast<uint32_t>(n), &r);
		CHECK(r == 0.0f);
		arm_mse_f32(a.data(), z.data(), static_cast<uint32_t>(n), &r);
		CHECK(r == 0.140625f);   // n * 9/64 / n
	}
}

TEST_CASE("characterization: [C4 arm_var_f32] a block of 0 or 1 sample gives variance 0 and standard deviation 0 (DOC is silent, the source returns 0 explicitly)") {
	// mathematically the n-1 estimator of one sample is 0/0. NEEDS REVIEW (the existing test in cases_target documents the same).
	const float x = 42.0f;
	float r = -1.0f;
	arm_var_f32(&x, 1, &r);
	CHECK(r == 0.0f);
	r = -1.0f;
	arm_var_f32(&x, 0, &r);
	CHECK(r == 0.0f);
	r = -1.0f;
	arm_std_f32(&x, 1, &r);
	CHECK(r == 0.0f);
	r = -1.0f;
	arm_std_f32(&x, 0, &r);
	CHECK(r == 0.0f);
}

TEST_CASE("characterization: [C4 arm_accumulate_f32] the sum of 0 samples is 0 and nothing is read (DOC is silent)") {
	float r = -1.0f;
	arm_accumulate_f32(nullptr, 0, &r);
	CHECK(r == 0.0f);
}

TEST_CASE("[C4 arm_mean_f32] a strided sample of the finite float bit patterns: the mean and the accumulate of one sample is the sample") {
	size_t bad = 0;
	sweepBlocks(1, 20000, [&](const std::vector<float> &x) {
		float m = 0.0f, a = 0.0f;
		arm_mean_f32(x.data(), 1, &m);
		arm_accumulate_f32(x.data(), 1, &a);
		if (!(m == x[0] && a == x[0])) bad++;   // sum = 0 + x, / 1.0f: both exact
	});
	CHECK(bad == 0u);
}

TEST_CASE("[C4 arm_power_f32] a strided sample of the finite float bit patterns: the power of one sample is the correctly rounded square (|x| < 1.8e19)") {
	// power = 0 + x*x, one rounding. The double product of two floats is exact (48 bits), its conversion to float is the correct rounding.
	// Squares of |x| >= 1.8e19 overflow float (not tested: float overflow of a double is a separate IEEE case).
	size_t bad = 0, tested = 0;
	sweepBlocks(1, 40000, [&](const std::vector<float> &x) {
		if (std::fabs(x[0]) >= 1.8e19f) return;
		float p = 0.0f;
		arm_power_f32(x.data(), 1, &p);
		tested++;
		if (!(p == static_cast<float>(static_cast<double>(x[0]) * x[0]))) bad++;
	});
	CHECK(tested > 1000u);
	CHECK(bad == 0u);
}

TEST_CASE("[C4 arm_rms_f32] a sweep over magnitudes 1e-18 .. 1e19 with both signs: the rms of one sample is |x| within 2*eps relative") {
	// fl(x*x) has relative error eps, sqrt halves it and adds eps: <= 1.5 eps (< 2 eps); x*x stays normal for |x| >= 1e-18
	size_t bad = 0;
	for (int e = -180; e <= 190; e++) {
		for (float sgn : {1.0f, -1.0f}) {
			const float x = sgn * static_cast<float>(std::pow(10.0, e / 10.0)) * 1.2345f;
			float r = -1.0f;
			arm_rms_f32(&x, 1, &r);
			if (!(std::fabs(static_cast<double>(r) - std::fabs(static_cast<double>(x))) <= 2.0 * kEps * std::fabs(static_cast<double>(x)))) bad++;
		}
	}
	CHECK(bad == 0u);
}

// ===============================================================================================================
// C3: entropy, Kullback-Leibler, logsumexp, logsumexp dot product
// ===============================================================================================================

TEST_CASE("[C3 arm_entropy_f32] the entropy of random probability vectors of 1..20 entries equals -sum p ln p within (n+2)*eps*sum|p ln p|") {
	Rng rng(201);
	for (size_t n = 1; n <= 20; n++) {
		for (int trial = 0; trial < 10; trial++) {
			CAPTURE(n);
			CAPTURE(trial);
			const std::vector<float> p = randVec(rng, n, 0.001f, 1.0f);
			double ref = 0, T = 0;
			for (float v : p) { const double t = static_cast<double>(v) * std::log(static_cast<double>(v)); ref -= t; T += std::fabs(t); }
			const float r = arm_entropy_f32(p.data(), static_cast<uint32_t>(n));
			CHECK(std::fabs(r - ref) <= gam(static_cast<double>(n) + 2.0) * T);
		}
	}
}

TEST_CASE("[C3 arm_entropy_f32] the entropy of the uniform distribution over 8 outcomes is ln 8") {
	const std::vector<float> p(8, 0.125f);
	const double ref = std::log(8.0);
	const double T = ref;    // sum |p ln p| = ln 8
	CHECK(std::fabs(arm_entropy_f32(p.data(), 8) - ref) <= gam(10.0) * T);
}

TEST_CASE("[C3 arm_kullback_leibler_f32] the divergence of random distribution pairs of 1..20 entries equals sum pA ln(pB/pA) negated within the derived bound") {
	Rng rng(202);
	for (size_t n = 1; n <= 20; n++) {
		for (int trial = 0; trial < 10; trial++) {
			CAPTURE(n);
			CAPTURE(trial);
			const std::vector<float> a = randVec(rng, n, 0.01f, 1.0f);
			const std::vector<float> b = randVec(rng, n, 0.01f, 1.0f);
			double ref = 0, T = 0, E = 0;
			for (size_t i = 0; i < n; i++) {
				const double r = static_cast<double>(b[i]) / a[i];
				const double t = a[i] * std::log(r);
				ref -= t;
				T += std::fabs(t);
				E += a[i] * (1.0 + 2.0 * std::fabs(std::log(r))) + std::fabs(t);
			}
			const float res = arm_kullback_leibler_f32(a.data(), b.data(), static_cast<uint32_t>(n));
			CHECK(std::fabs(res - ref) <= 1.001 * kEps * E + gam(static_cast<double>(n) - 1.0) * T);
		}
	}
}

TEST_CASE("[C4 arm_kullback_leibler_f32] the divergence of a distribution from itself is exactly 0") {
	// pB / pA = 1 exactly, logf(1) = 0, 0 * pA = 0
	Rng rng(203);
	for (size_t n = 1; n <= 20; n++) {
		CAPTURE(n);
		const std::vector<float> a = randVec(rng, n, 0.01f, 1.0f);
		CHECK(arm_kullback_leibler_f32(a.data(), a.data(), static_cast<uint32_t>(n)) == 0.0f);
	}
}

TEST_CASE("characterization: [C4 arm_entropy_f32] a zero probability gives NaN because 0*logf(0) = 0*(-inf) (DOC is silent about 0 ln 0)") {
	// NEEDS REVIEW: the usual convention is 0 ln 0 = 0
	const float p[3] = {0.5f, 0.0f, 0.5f};
	CHECK(std::isnan(arm_entropy_f32(p, 3)));
}

TEST_CASE("characterization: [C4 arm_kullback_leibler_f32] a zero in distribution A gives NaN on the portable path (DOC: A may contain 0 'with Neon version')") {
	// pA * logf(pB / pA) = 0 * logf(inf) = 0 * inf = NaN. NEEDS REVIEW
	const float a[3] = {0.5f, 0.0f, 0.5f}, b[3] = {0.3f, 0.4f, 0.3f};
	CHECK(std::isnan(arm_kullback_leibler_f32(a, b, 3)));
}

TEST_CASE("[C3 arm_logsumexp_f32] logsumexp of random blocks of 1..20 values equals m + ln(sum exp(x-m)) within the derived bound") {
	Rng rng(204);
	for (size_t n = 1; n <= 20; n++) {
		for (int trial = 0; trial < 10; trial++) {
			CAPTURE(n);
			CAPTURE(trial);
			const std::vector<float> x = randVec(rng, n, -20.0f, 20.0f);
			const std::vector<double> xd(x.begin(), x.end());
			const Lse d = defLse(xd);
			const float r = arm_logsumexp_f32(x.data(), static_cast<uint32_t>(n));
			CHECK(std::fabs(r - d.value) <= lseBound(d, n));
		}
	}
}

TEST_CASE("[C4 arm_logsumexp_f32] one value gives that value, two equal values give the value plus ln 2") {
	for (float v : {-30.0f, -1.5f, 0.0f, 2.25f, 40.0f}) {
		CAPTURE(v);
		const std::vector<double> one = {v}, two = {v, v};
		const float xv[2] = {v, v};
		CHECK(std::fabs(arm_logsumexp_f32(&v, 1) - v) <= lseBound(defLse(one), 1));
		CHECK(std::fabs(arm_logsumexp_f32(xv, 2) - (v + std::log(2.0))) <= lseBound(defLse(two), 2));
	}
}

TEST_CASE("[C4 arm_logsumexp_f32] inputs of +-1000 do not overflow or underflow (DOC: avoids underflow and overflow)") {
	// DOC: "To avoid underflow and overflow issues, the values are represented by their log." expf(1000) would overflow, expf(-1000) is 0.
	const float hi[2] = {1000.0f, 1000.0f}, lo[2] = {-1000.0f, -1001.0f};
	const std::vector<double> dhi = {1000.0, 1000.0}, dlo = {-1000.0, -1001.0};
	const float rh = arm_logsumexp_f32(hi, 2), rl = arm_logsumexp_f32(lo, 2);
	CHECK(std::isfinite(rh));
	CHECK(std::isfinite(rl));
	CHECK(std::fabs(rh - defLse(dhi).value) <= lseBound(defLse(dhi), 2));
	CHECK(std::fabs(rl - defLse(dlo).value) <= lseBound(defLse(dlo), 2));
}

TEST_CASE("[C3 arm_logsumexp_dot_prod_f32] the log dot product of random block pairs of 1..20 values equals logsumexp of the sums a+b within the derived bound") {
	Rng rng(205);
	for (size_t n = 1; n <= 20; n++) {
		for (int trial = 0; trial < 10; trial++) {
			CAPTURE(n);
			CAPTURE(trial);
			const std::vector<float> a = randVec(rng, n, -10.0f, 10.0f);
			const std::vector<float> b = randVec(rng, n, -10.0f, 10.0f);
			std::vector<double> t(n);
			double amax = 0;
			for (size_t i = 0; i < n; i++) { t[i] = static_cast<double>(a[i]) + b[i]; amax = std::fmax(amax, std::fabs(t[i])); }
			const Lse d = defLse(t);
			std::vector<float> tmp(n);
			const float r = arm_logsumexp_dot_prod_f32(a.data(), b.data(), static_cast<uint32_t>(n), tmp.data());
			CHECK(std::fabs(r - d.value) <= lseBound(d, n) + kEps * amax);
		}
	}
}

}  // TEST_SUITE
