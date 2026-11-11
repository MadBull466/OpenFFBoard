/*
 * CMSIS-DSP Statistics q15 (work order SQ15, phase P11 group G3). Requirement ids: Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * Units covered (all 14 q15 units of Source/StatisticsFunctions): arm_max_q15, arm_min_q15, arm_absmax_q15, arm_absmin_q15 (value and
 * index), arm_max_no_idx_q15, arm_min_no_idx_q15, arm_absmax_no_idx_q15, arm_absmin_no_idx_q15, arm_mean_q15, arm_power_q15 (q63 result,
 * 34.30), arm_rms_q15, arm_std_q15, arm_var_q15, arm_mse_q15. (Entropy, KL, logsumexp and accumulate have no q15 version.)
 *
 * ARM ids (suite "Statistics Tests Q15", Testing/desc.txt:369-:462, tests in Testing/Source/Tests/StatsTestsQ15.cpp, setUp :341-:920): 45 ids,
 * all run here, none is disabled. Patterns in Libraries/CMSIS-DSP/Testing/Patterns/DSP/Stats/StatsQ15/. "nb=7 / 8n / 8n+1" are the first
 * 7, 16 and 23 samples (inputA.reload(..., 7 / 16 / 23)), "long" is the first 100, refOffset selects the 0th..3rd reference value:
 *   max / min       : Input1 -> MaxVals1 + MaxIndexes1 / MinVals3 + MinIndexes3          (nb 7, 16, 23)   :22, :114
 *   absmax / absmin : InputNew1 -> AbsMaxVals8 + AbsMaxIndexes8 / AbsMinVals9 + AbsMinIndexes9 (7, 16, 23) :88, :180
 *   *_no_idx        : same inputs and values, no index                                    :48, :68, :140, :160
 *   mean            : Input2 -> MeanVals2 (7, 16, 23)                                      :206
 *   power           : Input1 -> PowerVals4_q63 (7, 16, 23)                                 :228
 *   rms / std / var : Input1 -> RmsVals5 / StdVals6 / VarVals7 (7, 16, 23)                 :250, :272, :294
 *   mse             : InputNew1 + InputNew2 -> MSEVals10 (7, 16, 23, long 100)             :317
 *   saturation      : arm_absmax_q15 and arm_absmax_no_idx_q15 over the whole InputNew1 (refOffset 3: 0x7FFF, index 288), ids 44 and 45
 * ARM tolerances (StatsTestsQ15.cpp:9-:18), used unchanged (L4): SNR_THRESHOLD 50 dB (ASSERT_SNR, :9) and SNR_THRESHOLD_MSE 50 (:10);
 * per sample (ASSERT_NEAR_EQ, LSB) ABS_ERROR_Q15 100 (:17), ABS_ERROR_Q15_MSE 100 (:18), ABS_ERROR_Q63 1<<17 (:20, power). max / min /
 * absmax / absmin values and all indexes are compared EXACT (ASSERT_EQ; the index is cast to int16_t as in ARM).
 * The 100 LSB bound is loose (0.3 percent of full scale); the DEF tests below derive much tighter bounds from the scaling of the code.
 *
 * Scaling of the units (read from the generic C code and DOC; DEF = the mathematical definition evaluated in double / exact integers):
 *   mean  (B-DSQ15-2: UB shift in the unrolled loop, the tests with 4 or more samples are KNOWN_ISSUE_UB_HIDDEN) DOC: "accumulated in a 32-bit accumulator in 17.15 format, no risk of internal overflow, ... truncated to yield 1.15": the result
 *         is sum / n, C division, truncation toward zero. DOC does not say "toward zero" explicitly (NEEDS REVIEW, minor).
 *   power DOC: 64-bit accumulator, 2.30 products, result 34.30 and exact ("full precision of the intermediate multiplication is
 *         preserved"): result == sum of x^2 exactly (int64).
 *   rms   DOC: "truncated to 34.15 format by discarding the lower 15 bits, and then saturated to 1.15": p = SSAT16((P / n) >> 15), then
 *         arm_sqrt_q15(p). With q = P / n / 2^15 (real) p lies in [q-1, q] (floor, then saturation). DEF in q15 LSB: sqrt(P / n)
 *         = sqrt(32768 q). arm_sqrt_q15 itself is up to 7.15 LSB off the exact root (V41, G1) so the bound is 7.2 + the quantisation of
 *         p: sqrtBound() below.
 *   var   DOC as rms. out = (trunc(S2 / (n-1)) - trunc(S1^2 / (n (n-1)))) >> 15, S2 = sum x^2 (2.30), S1 = sum x. With a = S2/(n-1),
 *         b = S1^2/(n(n-1)), V = a - b >= 0 (Cauchy-Schwarz): each trunc loses < 1, the difference lies in (V-1, V+1) and is >= 0, the
 *         arithmetic shift floors: out in ((V-1)/2^15 - 1, (V+1)/2^15) = (def - 1 - 2^-15, def + 2^-15), def = V / 2^15 the unbiased
 *         variance (n-1) in q15 LSB. No saturation is applied in the code (DOC says there is: see B-DSQ15-1).
 *   std   DOC as rms: sqrt of the saturated variance: bound 7.2 + the quantisation of p in (def - 1 - 2^-15, def + 2^-15).
 *   mse   code: da = (a >> 1) - (b >> 1) (floor halves, so da = D + {-1/2, 0, 1/2}, D = (a-b)/2), sum of da^2 in q63, then
 *         SSAT16((q31)(sum / n) >> 13). floor(floor(S/n) / 2^13) = floor(S / (n 2^13)) so out = SSAT(floor(mean(da^2) / 2^13)). DEF: mean((a-b)^2)
 *         / 2^15 (real mean square error 1/n sum ((a-b)/32768)^2 times 32768). |da^2 - D^2| <= |D| + 1/4, so out lies in
 *         (def - 1 - e, def + e] with e = (mean|D| + 1/4) / 8192 before saturation; SSAT is 1-Lipschitz, so the bound holds after clamping def.
 *   absmax / absmin: |-32768| saturates to 32767 (ARM's saturation ids and the source: "(*pSrc == (q15_t) 0x8000) ? 0x7fff"). absmin and
 *         the no_idx variants use the same saturation (their source is the same pattern; checked by the exhaustive sweeps).
 *
 * Index outputs: DOC says nothing about ties. The source compares strictly, the first occurrence wins: `characterization:` (NEEDS REVIEW).
 * Not tested (NEEDS REVIEW): (1) blockSize 0 for the max / min / absmax / absmin family reads out of bounds (pSrc[0] is loaded
 * unconditionally and the loop count (blockSize-1)>>2 wraps); (2) blockSize 0 for arm_mean_q15 / arm_rms_q15 / arm_mse_q15 divides by zero
 * (SIGFPE on x86); (3) blockSize > 65536 for arm_mean_q15 (the q31 accumulator overflows, signed overflow; DOC claims "no risk" for the 17.15
 * format, which holds for up to 2^16 full scale samples: tested at exactly 65536); (3b) var / std for n = 65537 and above: blockSize * (blockSize - 1U) wraps in uint32 (B-DSQ15-3, tested at 65537); (4) DOC silent: var / std for blockSize <= 1.
 *
 * Host vs target (C6): q15 var / power / rms / std on the target use the SMLALD / read_q15x2 path and mean the packed pair loop; the host runs neither SMLALD
 * (the DSP branches), and the unrolled __QSUB16 abs path of absmax / absmin is target-only too. The host runs the portable C path (ARM_MATH_LOOPUNROLL, no ARM_MATH_DSP, no MVE): the SMLALD / QSUB16 /
 * read_q15x2 paths of Cortex-M4 are not run here. Integer results are exact on both, but the DSP path accumulates the same sums in a
 * different order (integer addition is associative, so the same value). a >> 1 of negative values is an arithmetic shift on GCC / clang / armcc (implementation
 * defined in C). The (q31_t) conversion of a q63 value above 2^31 (arm_var / arm_std with two samples of -32768) is implementation
 * defined; GCC wraps, as on target. f32 reference values in the C5 tests use bounds from the rounding model (host and target alike).
 */
#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math.h"

namespace {

const char *const kDir = "Stats/StatsQ15/";

// ARM tolerances (StatsTestsQ15.cpp:9-:20)
const double kSnr = 50.0;
const int64_t kLsb = 100;
const int64_t kLsbMse = 100;
const int64_t kLsbQ63 = 1 << 17;

const double kEps = 5.9604644775390625e-8;   // f32 unit roundoff 2^-24 (C5 only)
double gam(double k) { return k * kEps * 1.0001; }
const double kSqrtErr = 7.2;                 // arm_sqrt_q15 worst error against the exact root: 7.15 LSB (V41)
const double kHalf15 = 1.0 / 32768.0;

std::vector<int16_t> pat(const std::string &name) { return dsp::loadPattern(kDir + name).q15(); }
std::vector<int64_t> pat63(const std::string &name) { return dsp::loadPattern(kDir + name).s64(); }

// Deterministic generator (LCG, Numerical Recipes constants). One call per statement.
struct Rng {
	uint32_t s;
	explicit Rng(uint32_t seed) : s(seed) {}
	uint32_t next() { s = s * 1664525u + 1013904223u; return s; }
	int16_t uni(int lo, int hi) {
		const uint32_t v = next() >> 8;
		return static_cast<int16_t>(lo + static_cast<int>(v % static_cast<uint32_t>(hi - lo + 1)));
	}
};

std::vector<int16_t> randVec(Rng &r, size_t n, int lo, int hi) {
	std::vector<int16_t> v(n);
	for (size_t i = 0; i < n; i++) v[i] = r.uni(lo, hi);
	return v;
}

// Blocks of n samples taken from a stride over all 2^16 values (k += 40503 is odd: visits every value once per cycle of 65536). 6000 blocks of
// 13 samples = 78000 samples cover every value at least once.
template <class F>
void sweepBlocks(size_t n, size_t blocks, uint16_t start, F &&f) {
	uint16_t k = start;
	for (size_t b = 0; b < blocks; b++) {
		std::vector<int16_t> x(n);
		for (size_t i = 0; i < n; i++) { k = static_cast<uint16_t>(k + 40503u); x[i] = static_cast<int16_t>(k); }
		f(x);
	}
}

const uint32_t kSizes[] = {1, 2, 3, 4, 5, 9};

// ---------------------------------------------------------------------------------------------------------------
// function tables
// ---------------------------------------------------------------------------------------------------------------
enum Mode { MAX = 0, MIN = 1, ABSMAX = 2, ABSMIN = 3 };
typedef void (*IdxFn)(const q15_t *, uint32_t, q15_t *, uint32_t *);
typedef void (*NoIdxFn)(const q15_t *, uint32_t, q15_t *);
typedef void (*ScalarFn)(const q15_t *, uint32_t, q15_t *);

const IdxFn kIdx[4] = {arm_max_q15, arm_min_q15, arm_absmax_q15, arm_absmin_q15};
const NoIdxFn kNoIdx[4] = {arm_max_no_idx_q15, arm_min_no_idx_q15, arm_absmax_no_idx_q15, arm_absmin_no_idx_q15};

int satAbs(int x) { return std::min(std::abs(x), 32767); }   // |-32768| saturates to 32767
int keyOf(Mode m, int x) {
	switch (m) {
	case MAX: return x;
	case MIN: return -x;
	case ABSMAX: return satAbs(x);
	default: return -satAbs(x);
	}
}
struct Ext { int value; size_t index; };
// DEF: first occurrence of the extreme key; the abs variants return the saturated magnitude
Ext defExt(Mode m, const std::vector<int16_t> &x) {
	size_t bi = 0;
	for (size_t i = 1; i < x.size(); i++)
		if (keyOf(m, x[i]) > keyOf(m, x[bi])) bi = i;
	const int v = x[bi];
	return {(m == ABSMAX || m == ABSMIN) ? satAbs(v) : v, bi};
}

// ---------------------------------------------------------------------------------------------------------------
// DEF helpers (double / exact integer sums)
// ---------------------------------------------------------------------------------------------------------------
double sumOf(const std::vector<int16_t> &x) { double a = 0; for (int16_t v : x) a += v; return a; }
double sumAbs(const std::vector<int16_t> &x) { double a = 0; for (int16_t v : x) a += std::fabs(static_cast<double>(v)); return a; }
double sumSq(const std::vector<int16_t> &x) { double a = 0; for (int16_t v : x) a += static_cast<double>(v) * v; return a; }
double clampQ15(double v) { return std::min(32767.0, std::max(-32768.0, v)); }
double defMean(const std::vector<int16_t> &x) { return std::trunc(sumOf(x) / static_cast<double>(x.size())); }
// unbiased variance (n-1) in q15 LSB: V / 2^15 with V = S2/(n-1) - S1^2/(n(n-1))
double defVar(const std::vector<int16_t> &x) {
	const double n = static_cast<double>(x.size()), s1 = sumOf(x);
	return (sumSq(x) / (n - 1.0) - s1 * s1 / (n * (n - 1.0))) / 32768.0;
}
double defMse(const std::vector<int16_t> &a, const std::vector<int16_t> &b) {
	double s = 0;
	for (size_t i = 0; i < a.size(); i++) { const double d = static_cast<double>(a[i]) - b[i]; s += d * d; }
	return s / static_cast<double>(a.size()) / 32768.0;
}
double mseBound(const std::vector<int16_t> &a, const std::vector<int16_t> &b) {
	double md = 0;
	for (size_t i = 0; i < a.size(); i++) md += std::fabs((static_cast<double>(a[i]) - b[i]) / 2.0);
	md /= static_cast<double>(a.size());
	return 1.0 + (md + 0.25) / 8192.0;
}
// result of sqrt(32768 q) = q15 root, where the integer fed to arm_sqrt_q15 lies in [lo, hi]: 7.2 (arm_sqrt_q15) + quantisation of the argument
double sqrtBound(double q, double lo, double hi) {
	lo = std::max(lo, 0.0);
	hi = std::min(hi, 32767.0);
	const double r = std::sqrt(32768.0 * std::max(q, 0.0));
	return kSqrtErr + std::max(std::fabs(r - std::sqrt(32768.0 * lo)), std::fabs(r - std::sqrt(32768.0 * std::min(std::max(hi, lo), 32767.0)))) + 1e-9;
}

// ---------------------------------------------------------------------------------------------------------------
// C1 helpers: ARM's tests
// ---------------------------------------------------------------------------------------------------------------
void armIdx(Mode m, const char *input, const char *vals, const char *idxs, size_t nb, size_t refOffset) {
	std::vector<int16_t> in = pat(input);
	const std::vector<int16_t> ref = pat(vals);
	const std::vector<int16_t> refInd = pat(idxs);
	REQUIRE(in.size() >= nb);
	REQUIRE(ref.size() > refOffset);
	REQUIRE(refInd.size() > refOffset);
	in.resize(nb);
	q15_t result = -12345;
	uint32_t indexval = 0xFFFFFFFFu;
	kIdx[m](in.data(), static_cast<uint32_t>(nb), &result, &indexval);
	CHECK(result == ref[refOffset]);
	CHECK(static_cast<int16_t>(indexval) == refInd[refOffset]);
}

void armNoIdx(Mode m, const char *input, const char *vals, size_t nb, size_t refOffset) {
	std::vector<int16_t> in = pat(input);
	const std::vector<int16_t> ref = pat(vals);
	REQUIRE(in.size() >= nb);
	REQUIRE(ref.size() > refOffset);
	in.resize(nb);
	q15_t result = -12345;
	kNoIdx[m](in.data(), static_cast<uint32_t>(nb), &result);
	CHECK(result == ref[refOffset]);
}

void checkArm(const std::vector<int16_t> &ref, const std::vector<int16_t> &out, int64_t lsb) {
	CHECK(dsp::snr(ref, out) >= kSnr);
	auto c = dsp::withinLsb(ref, out, lsb);
	CHECK_MESSAGE(c, c.message);
}

void armScalar(ScalarFn fn, const char *vals, size_t nb, size_t refOffset) {
	std::vector<int16_t> in = pat("Input1_q15.txt");
	const std::vector<int16_t> ref = pat(vals);
	REQUIRE(in.size() >= nb);
	REQUIRE(ref.size() > refOffset);
	in.resize(nb);
	q15_t result = -12345;
	fn(in.data(), static_cast<uint32_t>(nb), &result);
	checkArm({ref[refOffset]}, {result}, kLsb);
}

void armMean(size_t nb, size_t refOffset) {
	std::vector<int16_t> in = pat("Input2_q15.txt");
	const std::vector<int16_t> ref = pat("MeanVals2_q15.txt");
	REQUIRE(in.size() >= nb);
	REQUIRE(ref.size() > refOffset);
	in.resize(nb);
	q15_t result = -12345;
	arm_mean_q15(in.data(), static_cast<uint32_t>(nb), &result);
	checkArm({ref[refOffset]}, {result}, kLsb);
}

void armPower(size_t nb, size_t refOffset) {
	std::vector<int16_t> in = pat("Input1_q15.txt");
	const std::vector<int64_t> ref = pat63("PowerVals4_q63.txt");
	REQUIRE(in.size() >= nb);
	REQUIRE(ref.size() > refOffset);
	in.resize(nb);
	q63_t result = -12345;
	arm_power_q15(in.data(), static_cast<uint32_t>(nb), &result);
	const std::vector<int64_t> r = {ref[refOffset]}, o = {result};
	CHECK(dsp::snr(r, o) >= kSnr);
	auto c = dsp::withinLsb(r, o, kLsbQ63);
	CHECK_MESSAGE(c, c.message);
}

void armMse(size_t nb, size_t refOffset) {
	std::vector<int16_t> a = pat("InputNew1_q15.txt"), b = pat("InputNew2_q15.txt");
	const std::vector<int16_t> ref = pat("MSEVals10_q15.txt");
	REQUIRE(a.size() >= nb);
	REQUIRE(b.size() >= nb);
	REQUIRE(ref.size() > refOffset);
	a.resize(nb);
	b.resize(nb);
	q15_t result = -12345;
	arm_mse_q15(a.data(), b.data(), static_cast<uint32_t>(nb), &result);
	checkArm({ref[refOffset]}, {result}, kLsbMse);
}

}  // namespace

// B-DSQ15-2: arm_mean_q15.c:120-125 (the unrolled loop of the generic path) computes ((in << 16U) >> 16U) on a packed pair of two q15 samples held in a
// q31_t: the left shift of a value >= 2^15 or of a negative value by 16 is undefined (signed overflow), UBSan stops at the first hit. GCC / clang give the
// intended sign extension, so the results below are the intended ones on the host.
#define B_DSQ15_2 "B-DSQ15-2: arm_mean_q15.c:120 shifts a packed q31 pair left by 16 (signed overflow, undefined) in the unrolled loop for blocks of 4 or more samples; the intended result occurs on the host"
#define MEAN_UB KNOWN_ISSUE_UB_HIDDEN(B_DSQ15_2)

TEST_SUITE("DSP Statistics q15") {

// ===============================================================================================================
// C1: ARM's test ids
// ===============================================================================================================
#define IDX_CASES(FN, M, IN, VALS, IDXS, WHAT) \
	TEST_CASE("[C1 " FN "] " WHAT " of the first 7 samples (ARM nb=7) and its index equal ARM's reference exactly") { armIdx(M, IN, VALS, IDXS, 7, 0); } \
	TEST_CASE("[C1 " FN "] " WHAT " of the first 16 samples (ARM nb=8n) and its index equal ARM's reference exactly") { armIdx(M, IN, VALS, IDXS, 16, 1); } \
	TEST_CASE("[C1 " FN "] " WHAT " of the first 23 samples (ARM nb=8n+1) and its index equal ARM's reference exactly") { armIdx(M, IN, VALS, IDXS, 23, 2); }
#define NOIDX_CASES(FN, M, IN, VALS, WHAT) \
	TEST_CASE("[C1 " FN "] " WHAT " of the first 7 samples (ARM nb=7) equals ARM's reference exactly") { armNoIdx(M, IN, VALS, 7, 0); } \
	TEST_CASE("[C1 " FN "] " WHAT " of the first 16 samples (ARM nb=8n) equals ARM's reference exactly") { armNoIdx(M, IN, VALS, 16, 1); } \
	TEST_CASE("[C1 " FN "] " WHAT " of the first 23 samples (ARM nb=8n+1) equals ARM's reference exactly") { armNoIdx(M, IN, VALS, 23, 2); }
#define SCALAR_CASES(FN, FNP, VALS, WHAT) \
	TEST_CASE("[C1 " FN "] " WHAT " of the first 7 samples (ARM nb=7) matches ARM's reference within 50 dB and 100 LSB") { armScalar(FNP, VALS, 7, 0); } \
	TEST_CASE("[C1 " FN "] " WHAT " of the first 16 samples (ARM nb=8n) matches ARM's reference within 50 dB and 100 LSB") { armScalar(FNP, VALS, 16, 1); } \
	TEST_CASE("[C1 " FN "] " WHAT " of the first 23 samples (ARM nb=8n+1) matches ARM's reference within 50 dB and 100 LSB") { armScalar(FNP, VALS, 23, 2); }

IDX_CASES("arm_max_q15", MAX, "Input1_q15.txt", "MaxVals1_q15.txt", "MaxIndexes1_s16.txt", "max")
IDX_CASES("arm_min_q15", MIN, "Input1_q15.txt", "MinVals3_q15.txt", "MinIndexes3_s16.txt", "min")
IDX_CASES("arm_absmax_q15", ABSMAX, "InputNew1_q15.txt", "AbsMaxVals8_q15.txt", "AbsMaxIndexes8_s16.txt", "absmax")
IDX_CASES("arm_absmin_q15", ABSMIN, "InputNew1_q15.txt", "AbsMinVals9_q15.txt", "AbsMinIndexes9_s16.txt", "absmin")
NOIDX_CASES("arm_max_no_idx_q15", MAX, "Input1_q15.txt", "MaxVals1_q15.txt", "max")
NOIDX_CASES("arm_min_no_idx_q15", MIN, "Input1_q15.txt", "MinVals3_q15.txt", "min")
NOIDX_CASES("arm_absmax_no_idx_q15", ABSMAX, "InputNew1_q15.txt", "AbsMaxVals8_q15.txt", "absmax")
NOIDX_CASES("arm_absmin_no_idx_q15", ABSMIN, "InputNew1_q15.txt", "AbsMinVals9_q15.txt", "absmin")

TEST_CASE("[C1 arm_mean_q15] mean of the first 7 samples (ARM nb=7) matches ARM's reference within 50 dB and 100 LSB" * MEAN_UB) { armMean(7, 0); }
TEST_CASE("[C1 arm_mean_q15] mean of the first 16 samples (ARM nb=8n) matches ARM's reference within 50 dB and 100 LSB" * MEAN_UB) { armMean(16, 1); }
TEST_CASE("[C1 arm_mean_q15] mean of the first 23 samples (ARM nb=8n+1) matches ARM's reference within 50 dB and 100 LSB" * MEAN_UB) { armMean(23, 2); }

TEST_CASE("[C1 arm_power_q15] power of the first 7 samples (ARM nb=7) matches ARM's reference within 50 dB and 2^17") { armPower(7, 0); }
TEST_CASE("[C1 arm_power_q15] power of the first 16 samples (ARM nb=8n) matches ARM's reference within 50 dB and 2^17") { armPower(16, 1); }
TEST_CASE("[C1 arm_power_q15] power of the first 23 samples (ARM nb=8n+1) matches ARM's reference within 50 dB and 2^17") { armPower(23, 2); }

SCALAR_CASES("arm_rms_q15", arm_rms_q15, "RmsVals5_q15.txt", "rms")
SCALAR_CASES("arm_std_q15", arm_std_q15, "StdVals6_q15.txt", "std")
SCALAR_CASES("arm_var_q15", arm_var_q15, "VarVals7_q15.txt", "var")

TEST_CASE("[C1 arm_mse_q15] mse of the first 7 samples (ARM nb=7) matches ARM's reference within 50 dB and 100 LSB") { armMse(7, 0); }
TEST_CASE("[C1 arm_mse_q15] mse of the first 16 samples (ARM nb=8n) matches ARM's reference within 50 dB and 100 LSB") { armMse(16, 1); }
TEST_CASE("[C1 arm_mse_q15] mse of the first 23 samples (ARM nb=8n+1) matches ARM's reference within 50 dB and 100 LSB") { armMse(23, 2); }
TEST_CASE("[C1 arm_mse_q15] mse of the first 100 samples (ARM long) matches ARM's reference within 50 dB and 100 LSB") { armMse(100, 3); }

TEST_CASE("[C1 arm_absmax_q15] absmax and index over all of ARM's input with -32768 saturate to 32767 at index 288 (ARM saturation id)") {
	const std::vector<int16_t> in = pat("InputNew1_q15.txt");
	const std::vector<int16_t> ref = pat("AbsMaxVals8_q15.txt"), refInd = pat("AbsMaxIndexes8_s16.txt");
	REQUIRE(ref.size() > 3);
	REQUIRE(refInd.size() > 3);
	q15_t result = -12345;
	uint32_t indexval = 0xFFFFFFFFu;
	arm_absmax_q15(in.data(), static_cast<uint32_t>(in.size()), &result, &indexval);
	CHECK(result == ref[3]);
	CHECK(static_cast<int16_t>(indexval) == refInd[3]);
}

TEST_CASE("[C1 arm_absmax_no_idx_q15] absmax over all of ARM's input with -32768 saturates to 32767 (ARM saturation id)") {
	const std::vector<int16_t> in = pat("InputNew1_q15.txt");
	const std::vector<int16_t> ref = pat("AbsMaxVals8_q15.txt");
	REQUIRE(ref.size() > 3);
	q15_t result = -12345;
	arm_absmax_no_idx_q15(in.data(), static_cast<uint32_t>(in.size()), &result);
	CHECK(result == ref[3]);
}

// ===============================================================================================================
// C2: nothing written beyond the result or the index
// ===============================================================================================================

TEST_CASE("[C2 arm_max_q15] value and index are written to exactly one word each for the block sizes 1, 2, 3, 4, 5 and 9 and are correct") {
	const q15_t in[9] = {-4, 7500, 25, -9000, 300, 6000, -1, 2, 5000};
	for (int m = 0; m < 4; m++) {
		for (uint32_t n : kSizes) {
			CAPTURE(m);
			CAPTURE(n);
			dsp::GuardedBuffer<q15_t> r(1);
			dsp::GuardedBuffer<uint32_t> ix(1);
			kIdx[m](in, n, r.data(), ix.data());
			const std::vector<int16_t> x(in, in + n);
			const Ext e = defExt(static_cast<Mode>(m), x);
			CHECK(r.data()[0] == e.value);
			CHECK(ix.data()[0] == e.index);
			CHECK(r.guardIntact());
			CHECK(ix.guardIntact());
		}
	}
}

TEST_CASE("[C2 arm_max_no_idx_q15] the four no-index functions write exactly one word for the block sizes 1, 2, 3, 4, 5 and 9 and are correct") {
	const q15_t in[9] = {-4, 7500, -3250, -9500, 300, -5, -1, 2, 5000};
	for (int m = 0; m < 4; m++) {
		for (uint32_t n : kSizes) {
			CAPTURE(m);
			CAPTURE(n);
			dsp::GuardedBuffer<q15_t> r(1);
			kNoIdx[m](in, n, r.data());
			const std::vector<int16_t> x(in, in + n);
			CHECK(r.data()[0] == defExt(static_cast<Mode>(m), x).value);
			CHECK(r.guardIntact());
		}
	}
}

TEST_CASE("[C2 arm_mean_q15] the mean writes exactly one q15 word for the block sizes 1, 2, 3, 4, 5 and 9 and is correct" * MEAN_UB) {
	const q15_t a[9] = {1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000};
	for (uint32_t n : kSizes) {
		CAPTURE(n);
		dsp::GuardedBuffer<q15_t> r(1);
		arm_mean_q15(a, n, r.data());
		CHECK(r.data()[0] == defMean(std::vector<int16_t>(a, a + n)));
		CHECK(r.guardIntact());
	}
}

TEST_CASE("[C2 arm_rms_q15] rms, std, var and mse leave the guard words behind their one q15 result intact for the block sizes 1, 2, 3, 4, 5 and 9") {
	const q15_t a[9] = {1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000};
	const q15_t b[9] = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000};
	const ScalarFn fns[3] = {arm_rms_q15, arm_std_q15, arm_var_q15};
	for (uint32_t n : kSizes) {
		CAPTURE(n);
		for (int f = 0; f < 3; f++) {
			CAPTURE(f);
			dsp::GuardedBuffer<q15_t> r(1);
			fns[f](a, n, r.data());
			CHECK(r.guardIntact());
		}
		dsp::GuardedBuffer<q15_t> r(1);
		arm_mse_q15(a, b, n, r.data());
		CHECK(r.guardIntact());
		const std::vector<int16_t> xa(a, a + n), xb(b, b + n);
		CHECK(std::fabs(r.data()[0] - defMse(xa, xb)) <= mseBound(xa, xb));
	}
}

TEST_CASE("[C2 arm_power_q15] the 64-bit sum is written to exactly one q63 word for the block sizes 0, 1, 2, 3, 4, 5 and 9 and is the exact sum of squares") {
	const q15_t in[9] = {1000, -2000, 3000, -4000, 5000, -6000, 7000, -8000, 9000};
	for (uint32_t n : {0u, 1u, 2u, 3u, 4u, 5u, 9u}) {
		CAPTURE(n);
		dsp::GuardedBuffer<q63_t> r(1);
		arm_power_q15(in, n, r.data());
		CHECK(r.data()[0] == static_cast<int64_t>(sumSq(std::vector<int16_t>(in, in + n))));   // n = 0: empty sum is 0
		CHECK(r.guardIntact());
	}
}

// ===============================================================================================================
// C3: DEF for the max / min family
// ===============================================================================================================
namespace {

void idxRandom(Mode m) {
	Rng rng(777u + static_cast<uint32_t>(m));
	for (size_t n = 1; n <= 40; n++) {
		for (int trial = 0; trial < 20; trial++) {
			CAPTURE(n);
			CAPTURE(trial);
			const std::vector<int16_t> x = randVec(rng, n, -32768, 32767);   // ties are possible (n^2/2^17) and resolved first-wins by DEF
			const Ext e = defExt(m, x);
			q15_t v = 1;
			uint32_t ix = 0xFFFFFFFFu;
			kIdx[m](x.data(), static_cast<uint32_t>(n), &v, &ix);
			CHECK(v == e.value);
			CHECK(ix == e.index);
			q15_t v2 = 1;
			kNoIdx[m](x.data(), static_cast<uint32_t>(n), &v2);
			CHECK(v2 == e.value);
		}
	}
}

// The extreme sits at every position of blocks of 1..14 samples; the other samples are distinct with mixed signs
void idxPosition(Mode m) {
	for (size_t n = 1; n <= 14; n++) {
		for (size_t pos = 0; pos < n; pos++) {
			CAPTURE(n);
			CAPTURE(pos);
			std::vector<int16_t> x(n);
			for (size_t i = 0; i < n; i++) {
				const int mag = 1000 + 125 * static_cast<int>(i);
				x[i] = static_cast<int16_t>((i % 2 == 0) ? mag : -mag);
			}
			int expect;
			switch (m) {
			case MAX: x[pos] = 30000; expect = 30000; break;
			case MIN: x[pos] = -30000; expect = -30000; break;
			case ABSMAX: x[pos] = -30000; expect = 30000; break;
			default: x[pos] = -10; expect = 10; break;
			}
			q15_t v = 1;
			uint32_t ix = 0xFFFFFFFFu;
			kIdx[m](x.data(), static_cast<uint32_t>(n), &v, &ix);
			CHECK(v == expect);
			CHECK(ix == pos);
			q15_t v2 = 1;
			kNoIdx[m](x.data(), static_cast<uint32_t>(n), &v2);
			CHECK(v2 == expect);
		}
	}
}

void idxFullRange(Mode m) {
	size_t bad = 0;
	std::string first;
	sweepBlocks(13, 6000, 4321, [&](const std::vector<int16_t> &x) {
		const Ext e = defExt(m, x);
		q15_t v = 0, v2 = 0;
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

// EXHAUSTIVE over the single sample domain: all 65536 values, block size 1
void idxExhaustiveSingle(Mode m) {
	size_t bad = 0;
	int firstBad = 0;
	for (int s = -32768; s <= 32767; s++) {
		const q15_t sample = static_cast<q15_t>(s);
		q15_t v = 1, v2 = 1;
		uint32_t ix = 99;
		kIdx[m](&sample, 1, &v, &ix);
		kNoIdx[m](&sample, 1, &v2);
		const int expect = (m == ABSMAX || m == ABSMIN) ? satAbs(s) : s;
		if (!(v == expect && v2 == expect && ix == 0u)) {
			if (!bad) firstBad = s;
			bad++;
		}
	}
	CHECK_MESSAGE(bad == 0, bad << " of 65536 values differ, first: " << firstBad);
}

void idxPairs(Mode m) {
	const int16_t vals[] = {-32768, -32767, -1000, -1, 0, 1, 1000, 32766, 32767};
	for (int16_t a : vals) {
		for (int16_t b : vals) {
			// ties (equal keys, e.g. -32768 and 32767 for the abs units) return the first index (characterization, see the tie tests)
			CAPTURE(a);
			CAPTURE(b);
			const std::vector<int16_t> x = {a, b};
			const Ext e = defExt(m, x);
			q15_t v = 1;
			uint32_t ix = 99;
			kIdx[m](x.data(), 2, &v, &ix);
			CHECK(v == e.value);
			CHECK(ix == e.index);
			q15_t v2 = 1;
			kNoIdx[m](x.data(), 2, &v2);
			CHECK(v2 == e.value);
		}
	}
}

void idxNegative(Mode m) {
	for (size_t n : {3u, 4u, 5u, 8u, 9u}) {
		CAPTURE(n);
		std::vector<int16_t> x(n);
		for (size_t i = 0; i < n; i++) x[i] = static_cast<int16_t>(-100 - 500 * static_cast<int>((i * 5) % n));   // distinct negatives
		const Ext e = defExt(m, x);
		q15_t v = 1;
		uint32_t ix = 99;
		kIdx[m](x.data(), static_cast<uint32_t>(n), &v, &ix);
		CHECK(v == e.value);
		CHECK(ix == e.index);
	}
}

// Ties: DOC is silent; the source uses a strict comparison so the first occurrence wins.
void idxTies(Mode m) {
	std::vector<int16_t> x(10);
	for (size_t i = 0; i < 10; i++) x[i] = static_cast<int16_t>(1000 + 250 * static_cast<int>(i % 3));
	int16_t hi, lo;
	switch (m) {
	case MAX: hi = 20000; lo = 20000; break;
	case MIN: hi = -20000; lo = -20000; break;
	case ABSMAX: hi = -32768; lo = 32767; break;   // both have the saturated magnitude 32767
	default: hi = 0; lo = 0; break;
	}
	x[3] = hi;
	x[7] = lo;
	q15_t v = 1;
	uint32_t ix = 99;
	kIdx[m](x.data(), 10, &v, &ix);
	CHECK(ix == 3u);
	for (size_t n = 1; n <= 9; n++) {
		CAPTURE(n);
		const std::vector<int16_t> c(n, 2500);
		uint32_t ix2 = 99;
		q15_t v2 = 0;
		kIdx[m](c.data(), static_cast<uint32_t>(n), &v2, &ix2);
		CHECK(v2 == 2500);
		CHECK(ix2 == 0u);
	}
}

}  // namespace

#define IDX_DEF_CASES(FN, M, WHAT, NEG) \
	TEST_CASE("[C3 " FN "] value and index of random blocks of 1..40 samples equal the definition (also the no_idx variant)") { idxRandom(M); } \
	TEST_CASE("[C3 " FN "] " WHAT " is found at every position of blocks of 1..14 samples (also the no_idx variant)") { idxPosition(M); } \
	TEST_CASE("[C3 " FN "] a strided sweep over all 16 bit values (6000 blocks of 13) agrees with the definition (also the no_idx variant)") { idxFullRange(M); } \
	TEST_CASE("[C3 " FN "] every one of the 65536 single samples is its own result at index 0 (saturated magnitude for abs) (also the no_idx variant)") { idxExhaustiveSingle(M); } \
	TEST_CASE("[C4 " FN "] every ordered pair of edge values (block size 2, ties return the first index) gives the right value and index (also the no_idx variant)") { idxPairs(M); } \
	TEST_CASE("[C4 " FN "] blocks of only negative samples give " NEG) { idxNegative(M); } \
	TEST_CASE("[C4 " FN "] characterization: equal extremes return the lowest index, and a block of equal samples returns index 0") { idxTies(M); }

IDX_DEF_CASES("arm_max_q15", MAX, "the maximum", "the sample closest to zero")
IDX_DEF_CASES("arm_min_q15", MIN, "the minimum", "the most negative sample")
IDX_DEF_CASES("arm_absmax_q15", ABSMAX, "a negative absolute maximum (returned as a magnitude)", "the largest magnitude")
IDX_DEF_CASES("arm_absmin_q15", ABSMIN, "a negative absolute minimum (returned as a magnitude)", "the smallest magnitude")

// ===============================================================================================================
// C3 / C4: DEF for mean, power, rms, var, std, mse
// ===============================================================================================================
namespace {

struct DataClass { int lo, hi; const char *name; };
// full range (mean, power, max family), moderate (var / std / mse never saturate), offset (var cancellation), narrow
const DataClass kClasses[] = {{-32768, 32767, "full"}, {-16000, 16000, "moderate"}, {10000, 12000, "offset"}, {-300, 300, "narrow"}};

}  // namespace

TEST_CASE("[C4 arm_absmin_q15] characterization: blocks where every sample saturates to 32767 give 32767 at index 0 (also absmin_no_idx and absmax)") {
	const std::vector<std::vector<int16_t>> blocks = {{32767, -32768}, {-32768, -32768, 32767}, {-32768, 32767, -32768, -32768, 32767}, {32767, 32767, -32768, 32767, -32768, -32768, -32768}};
	for (const auto &x : blocks) {
		CAPTURE(x.size());
		for (int m : {int(ABSMIN), int(ABSMAX)}) {
			q15_t v = 1, v2 = 1;
			uint32_t ix = 99;
			kIdx[m](x.data(), static_cast<uint32_t>(x.size()), &v, &ix);
			kNoIdx[m](x.data(), static_cast<uint32_t>(x.size()), &v2);
			CHECK(v == 32767);
			CHECK(v2 == 32767);
			CHECK(ix == 0u);
		}
	}
	// a saturated sample at a later position is the absmax and is returned as 32767: {0, -32768} index 1, {5, 7, 0, 0, -32768} index 4
	const std::vector<int16_t> y = {5, 7, 0, 0, -32768};
	q15_t v = 1;
	uint32_t ix = 99;
	arm_absmax_q15(y.data(), 5, &v, &ix);
	CHECK(v == 32767);
	CHECK(ix == 4u);
	arm_absmax_no_idx_q15(y.data(), 5, &v);
	CHECK(v == 32767);
}

TEST_CASE("[C3 arm_mean_q15] the mean of random blocks of 1..40 samples equals the sum divided by n truncated toward zero" * MEAN_UB) {
	Rng rng(102);
	for (const DataClass &dc : kClasses) {
		for (size_t n = 1; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<int16_t> x = randVec(rng, n, dc.lo, dc.hi), orig = x;
			q15_t r = 1;
			arm_mean_q15(x.data(), static_cast<uint32_t>(n), &r);
			CHECK(r == defMean(x));
			CHECK(x == orig);
		}
	}
}

TEST_CASE("[C3 arm_mean_q15] the mean of 2 and of 3 samples (no unrolled loop, no UB) is the truncated mean for random full range data") {
	Rng rng(108);
	for (size_t n : {2u, 3u}) {
		for (int trial = 0; trial < 500; trial++) {
			CAPTURE(n);
			const std::vector<int16_t> x = randVec(rng, n, -32768, 32767);
			q15_t r = 1;
			arm_mean_q15(x.data(), static_cast<uint32_t>(n), &r);
			CHECK(r == defMean(x));
		}
	}
}

TEST_CASE("[C3 arm_mean_q15] a strided sweep over all 16 bit values (6000 blocks of 13) gives the truncated mean" * MEAN_UB) {
	size_t bad = 0;
	sweepBlocks(13, 6000, 99, [&](const std::vector<int16_t> &x) {
		q15_t r = 0;
		arm_mean_q15(x.data(), 13, &r);
		if (r != defMean(x)) bad++;
	});
	CHECK(bad == 0);
}

TEST_CASE("[C3 arm_mean_q15] the mean of a single sample is the sample for all 65536 values") {
	size_t bad = 0;
	for (int s = -32768; s <= 32767; s++) {
		const q15_t sample = static_cast<q15_t>(s);
		q15_t r = 1;
		arm_mean_q15(&sample, 1, &r);
		if (r != s) bad++;
	}
	CHECK(bad == 0);
}

TEST_CASE("[C4 arm_mean_q15] 65536 samples of -32768 (sum -2^31, the limit of the 17.15 accumulator) and of 32767 give the sample" * MEAN_UB) {
	// DOC: "accumulated in a 32-bit accumulator in 17.15 format ... no risk of internal overflow": 65536 * -32768 = -2^31 = INT32_MIN fits
	const std::vector<int16_t> lo(65536, -32768), hi(65536, 32767);
	q15_t r = 1;
	arm_mean_q15(lo.data(), 65536, &r);
	CHECK(r == -32768);
	arm_mean_q15(hi.data(), 65536, &r);   // 65536 * 32767 = 2^31 - 65536 fits
	CHECK(r == 32767);
}

TEST_CASE("[C4 arm_mean_q15] the mean of a block of equal samples is the sample, and the truncation goes toward zero for a negative mean" * MEAN_UB) {
	for (size_t n = 1; n <= 9; n++) {
		CAPTURE(n);
		const std::vector<int16_t> c(n, -1234);
		q15_t r = 1;
		arm_mean_q15(c.data(), static_cast<uint32_t>(n), &r);
		CHECK(r == -1234);
	}
	// hand derivation: {-1, -2} sums to -3, -3 / 2 = -1.5, truncated toward zero is -1 (floor would give -2)
	const q15_t x[2] = {-1, -2};
	q15_t r = 1;
	arm_mean_q15(x, 2, &r);
	CHECK(r == -1);
}

TEST_CASE("[C3 arm_power_q15] the power of random blocks of 1..40 samples is exactly the sum of squares (34.30)") {
	Rng rng(103);
	for (const DataClass &dc : kClasses) {
		for (size_t n = 1; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<int16_t> x = randVec(rng, n, dc.lo, dc.hi), orig = x;
			q63_t r = 1;
			arm_power_q15(x.data(), static_cast<uint32_t>(n), &r);
			CHECK(r == static_cast<int64_t>(sumSq(x)));   // sums below 2^53, exact in double
			CHECK(x == orig);
		}
	}
}

TEST_CASE("[C3 arm_power_q15] a strided sweep over all 16 bit values (6000 blocks of 13) gives the exact sum of squares, and every single value squares exactly") {
	size_t bad = 0;
	sweepBlocks(13, 6000, 7, [&](const std::vector<int16_t> &x) {
		q63_t r = 0;
		arm_power_q15(x.data(), 13, &r);
		if (r != static_cast<int64_t>(sumSq(x))) bad++;
	});
	CHECK(bad == 0);
	for (int s = -32768; s <= 32767; s++) {
		const q15_t sample = static_cast<q15_t>(s);
		q63_t r = 1;
		arm_power_q15(&sample, 1, &r);
		if (r != static_cast<int64_t>(s) * s) bad++;
	}
	CHECK(bad == 0);
}

TEST_CASE("[C4 arm_power_q15] 65536 samples of -32768 sum to exactly 2^46 (no wrap, 33 guard bits)") {
	// DOC: "64-bit accumulator ... 34.30 ... With 33 guard bits in the accumulator, there is no risk of overflow": 65536 * 2^30 = 2^46
	const std::vector<int16_t> x(65536, -32768);
	q63_t r = 1;
	arm_power_q15(x.data(), 65536, &r);
	CHECK(r == (static_cast<int64_t>(1) << 46));
}

TEST_CASE("[C3 arm_rms_q15] the rms of random blocks of 1..40 samples is within 7.2 LSB plus the quantisation of the 34.15 power of the exact root") {
	Rng rng(104);
	for (const DataClass &dc : kClasses) {
		for (size_t n = 1; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<int16_t> x = randVec(rng, n, dc.lo, dc.hi);
			q15_t r = 1;
			arm_rms_q15(x.data(), static_cast<uint32_t>(n), &r);
			const double q = sumSq(x) / static_cast<double>(n) / 32768.0;   // power / n in 34.15
			const double def = std::sqrt(sumSq(x) / static_cast<double>(n));
			CHECK(std::fabs(r - def) <= sqrtBound(q, q - 1.0, q));
		}
	}
}

TEST_CASE("[C3 arm_rms_q15] the rms of a single sample is its magnitude within the bound, for all 65536 values") {
	size_t bad = 0;
	int firstBad = 0;
	for (int s = -32768; s <= 32767; s++) {
		const q15_t sample = static_cast<q15_t>(s);
		q15_t r = 1;
		arm_rms_q15(&sample, 1, &r);
		const double q = static_cast<double>(s) * s / 32768.0;
		if (!(std::fabs(r - std::fabs(static_cast<double>(s))) <= sqrtBound(q, q - 1.0, q))) {
			if (!bad) firstBad = s;
			bad++;
		}
	}
	CHECK_MESSAGE(bad == 0, bad << " values outside the bound, first: " << firstBad);
}

TEST_CASE("[C4 arm_rms_q15] blocks of equal full scale samples (-32768 and 32767) saturate the power and give 32767 within the bound") {
	// DOC: "truncated to 34.15 ... and then saturated to 1.15": P/n = 2^30 gives 2^15, saturated to 32767; arm_sqrt_q15(32767) = sqrt(32767 * 32768) = 32767.5
	for (int16_t s : {int16_t(-32768), int16_t(32767)}) {
		for (size_t n : {1u, 2u, 5u, 8u}) {
			CAPTURE(s);
			CAPTURE(n);
			const std::vector<int16_t> x(n, s);
			q15_t r = 1;
			arm_rms_q15(x.data(), static_cast<uint32_t>(n), &r);
			CHECK(std::fabs(r - 32767.0) <= kSqrtErr + 0.5);
		}
	}
}

TEST_CASE("[C4 arm_rms_q15] hand derived rms of {0, 0, 0, 16384} is 16384 / 2 = 8192 within the bound") {
	// P = 2^28, n = 4: P/n = 2^26, >> 15 = 2048 exactly; sqrt(2048 * 32768) = sqrt(2^26) = 8192 (q = 2048, no quantisation)
	const q15_t x[4] = {0, 0, 0, 16384};
	q15_t r = 1;
	arm_rms_q15(x, 4, &r);
	CHECK(std::fabs(r - 8192.0) <= kSqrtErr);
}

TEST_CASE("[C4 arm_var_q15] hand derived variance of {0, 16384} is 4096 (0.125) exactly") {
	// real values 0 and 0.5: unbiased variance (0.5)^2 / 2 = 0.125 -> 0.125 * 32768 = 4096.
	// bound derived in the header: out in (def - 1 - 2^-15, def + 2^-15) = (4094.99997, 4096.00003), i.e. 4095 or 4096.
	// Exact here: S2 = 2^28 (exact), a = 2^28, S1 = 16384, b = 2^28 / 2 = 2^27 (exact), pre = 2^27, >> 15 = 4096.
	const q15_t x[2] = {0, 16384};
	q15_t r = 1;
	arm_var_q15(x, 2, &r);
	CHECK(r == 4096);
}

TEST_CASE("[C3 arm_var_q15] the variance of random blocks of 2..40 samples is within (def-1-2^-15, def+2^-15), def the unbiased variance in LSB") {
	Rng rng(105);
	for (const DataClass &dc : kClasses) {
		if (dc.hi - dc.lo > 40000 ) continue;   // full range data can have a variance above 1 (saturation, see the KNOWN_ISSUE case)
		for (size_t n = 2; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<int16_t> x = randVec(rng, n, dc.lo, dc.hi), orig = x;
			q15_t r = 1;
			arm_var_q15(x.data(), static_cast<uint32_t>(n), &r);
			const double def = defVar(x);
			CHECK(r > def - 1.0 - kHalf15 - 1e-9);
			CHECK(r < def + kHalf15 + 1e-9);
			CHECK(x == orig);
		}
	}
}

TEST_CASE("[C3 arm_var_q15] a strided sweep over all 16 bit values (6000 blocks of 13, variance below 1) is within the derived bound") {
	size_t bad = 0, used = 0;
	sweepBlocks(13, 6000, 31, [&](const std::vector<int16_t> &x) {
		const double def = defVar(x);
		if (def > 32700.0) return;   // saturation range: separate test
		used++;
		q15_t r = 0;
		arm_var_q15(x.data(), 13, &r);
		if (!(r > def - 1.0 - kHalf15 - 1e-9 && r < def + kHalf15 + 1e-9)) bad++;
	});
	CHECK(used > 5000);
	CHECK(bad == 0);
}

TEST_CASE("[C4 arm_var_q15] the variance of equal samples is 0 (including two samples of -32768 where the 2.30 mean of squares reaches 2^31)") {
	// n = 2, x = -32768: S2 = 2^31 and S1^2/2 = 2^31 both wrap in the q31 conversion (implementation defined) and cancel: 0 is also the DEF
	for (int16_t s : {int16_t(-32768), int16_t(-1), int16_t(0), int16_t(1), int16_t(32767)}) {
		for (size_t n : {2u, 3u, 4u, 5u, 9u, 13u}) {
			CAPTURE(s);
			CAPTURE(n);
			const std::vector<int16_t> x(n, s);
			q15_t r = 1;
			arm_var_q15(x.data(), static_cast<uint32_t>(n), &r);
			CHECK(r == 0);
		}
	}
}

TEST_CASE("[C4 arm_var_q15] characterization: the variance of 0 or 1 samples is 0 (explicit branch, DOC silent, DEF with n-1 undefined)") {
	const q15_t x[1] = {12345};
	q15_t r = 1;
	arm_var_q15(x, 1, &r);
	CHECK(r == 0);
	r = 1;
	arm_var_q15(x, 0, &r);
	CHECK(r == 0);
}

TEST_CASE("[C4 arm_var_q15] a variance above 1.0 saturates to 32767 (DOC) - two samples {-32768, 32767}" *
		  KNOWN_ISSUE("B-DSQ15-1: arm_var_q15.c:226 stores (meanOfSquares - squareOfMean) >> 15 without saturation although DOC says "
					  "'truncated to 34.15 format ... and then saturated to yield a result in 1.15 format'. For {-32768, 32767}: S2 = 2147418113, "
					  "S1 = -1, unbiased variance 2.0 real = 65534.00002 LSB (2147418113 >> 15 = 65534); the result is (q15_t)65534 = -2 instead of 32767. The sibling arm_std_q15 "
					  "does saturate (__SSAT, arm_std_q15.c)")) {
	// DEF: V = 2147418113 - 1/2 = 2147418112.5, / 2^15 = 65534.00002 LSB, above 32767: saturated result 32767 (the q15 maximum)
	const q15_t x[2] = {-32768, 32767};
	q15_t r = 1;
	arm_var_q15(x, 2, &r);
	CHECK(r == 32767);
}

TEST_CASE("[C4 arm_var_q15] a variance above 1.0 never gives a negative result - two samples {-32768, 32767}" *
		  KNOWN_ISSUE("B-DSQ15-1: the unsaturated store wraps the variance 65534 LSB to -2, a negative variance")) {
	const q15_t x[2] = {-32768, 32767};
	q15_t r = 1;
	arm_var_q15(x, 2, &r);
	CHECK(r >= 0);
}

TEST_CASE("[C4 arm_var_q15] the variance of 65537 samples is correct (DOC gives no block size limit)" *
		  KNOWN_ISSUE("B-DSQ15-3: arm_var_q15.c:223 computes blockSize * (blockSize - 1U) in uint32_t; for n = 65537 the product 4295032832 wraps to 65536, so "
					  "the square of the mean is divided by the wrong number and the result is garbage (reviewer probe on other data: -16514 against 0.08). "
					  "The mean-of-squares accumulator (q63) and the q31 sum are fine for this n")) {
	// n = 65537 (32769 samples of 8192 = 0.25 at the even indices, 32768 zeros): S1 = 32769 * 8192 (fits q31), S2 = 32769 * 2^26.
	// DEF: V = S2/(n-1) - S1^2/(n (n-1)) = 33555456 - 33554944.0 approx, in LSB V / 2^15 = 512.0078 (computed in double below)
	const size_t n = 65537;
	std::vector<int16_t> x(n, 0);
	for (size_t i = 0; i < n; i += 2) x[i] = 8192;
	q15_t r = 1;
	arm_var_q15(x.data(), static_cast<uint32_t>(n), &r);
	const double def = defVar(x);
	CHECK(std::fabs(r - def) <= 1.0 + kHalf15 + 1e-9);
}

TEST_CASE("[C4 arm_std_q15] the std of 65537 samples is correct (DOC gives no block size limit)" *
		  KNOWN_ISSUE("B-DSQ15-3: arm_std_q15.c:165 computes blockSize * (blockSize - 1U) in uint32_t and wraps for n = 65537 (same defect as arm_var_q15.c:223)")) {
	const size_t n = 65537;
	std::vector<int16_t> x(n, 0);
	for (size_t i = 0; i < n; i += 2) x[i] = 8192;
	q15_t r = 1;
	arm_std_q15(x.data(), static_cast<uint32_t>(n), &r);
	const double q = defVar(x);
	CHECK(std::fabs(r - std::sqrt(32768.0 * q)) <= sqrtBound(q, q - 1.0 - kHalf15, q + kHalf15));
}

TEST_CASE("[C3 arm_std_q15] the std of random blocks of 2..40 samples is within 7.2 LSB plus the quantisation of the variance of the exact root") {
	Rng rng(106);
	for (const DataClass &dc : kClasses) {
		if (dc.hi - dc.lo > 40000) continue;
		for (size_t n = 2; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<int16_t> x = randVec(rng, n, dc.lo, dc.hi);
			q15_t r = 1;
			arm_std_q15(x.data(), static_cast<uint32_t>(n), &r);
			const double q = defVar(x);
			CHECK(std::fabs(r - std::sqrt(32768.0 * q)) <= sqrtBound(q, q - 1.0 - kHalf15, q + kHalf15));
		}
	}
}

TEST_CASE("[C3 arm_std_q15] a strided sweep over all 16 bit values (6000 blocks of 13, variance below 1) is within the derived bound") {
	size_t bad = 0, used = 0;
	sweepBlocks(13, 6000, 57, [&](const std::vector<int16_t> &x) {
		const double q = defVar(x);
		if (q > 32700.0) return;
		used++;
		q15_t r = 0;
		arm_std_q15(x.data(), 13, &r);
		if (!(std::fabs(r - std::sqrt(32768.0 * q)) <= sqrtBound(q, q - 1.0 - kHalf15, q + kHalf15))) bad++;
	});
	CHECK(used > 5000);
	CHECK(bad == 0);
}

TEST_CASE("[C4 arm_std_q15] the std of equal samples is 0 (including -32768 and 32767)") {
	for (int16_t s : {int16_t(-32768), int16_t(0), int16_t(32767)}) {
		for (size_t n : {2u, 3u, 5u, 9u}) {
			CAPTURE(s);
			CAPTURE(n);
			const std::vector<int16_t> x(n, s);
			q15_t r = 1;
			arm_std_q15(x.data(), static_cast<uint32_t>(n), &r);
			CHECK(r == 0);
		}
	}
}

TEST_CASE("[C4 arm_std_q15] hand derived std of {0, 16384} is sqrt(0.125) = 0.35355 = 11585.2 LSB within the bound") {
	// variance 4096 LSB (see the var hand derivation, quantisation interval (4094.99997, 4096.00003)); sqrt(32768 * 4096) = 11585.24
	const q15_t x[2] = {0, 16384};
	q15_t r = 1;
	arm_std_q15(x, 2, &r);
	CHECK(std::fabs(r - 11585.2378) <= sqrtBound(4096.0, 4096.0 - 1.0 - kHalf15, 4096.0 + kHalf15));
}

TEST_CASE("[C4 arm_std_q15] two samples {-32768, 32767} (variance above 1) give 32767 within the bound because the variance is saturated before the root") {
	// DOC: "saturated to yield a result in 1.15 format": the variance 65534 LSB saturates to 32767, sqrt(32767 * 32768) = 32767.5
	const q15_t x[2] = {-32768, 32767};
	q15_t r = 1;
	arm_std_q15(x, 2, &r);
	CHECK(std::fabs(r - 32767.0) <= kSqrtErr + 0.5);
}

TEST_CASE("[C4 arm_std_q15] characterization: the std of 0 or 1 samples is 0 (explicit branch, DOC silent)") {
	const q15_t x[1] = {12345};
	q15_t r = 1;
	arm_std_q15(x, 1, &r);
	CHECK(r == 0);
	r = 1;
	arm_std_q15(x, 0, &r);
	CHECK(r == 0);
}

TEST_CASE("[C3 arm_mse_q15] the mse of random blocks of 1..40 sample pairs is within (def-1-e, def+e], def the real mean square error in LSB") {
	Rng rng(107);
	for (const DataClass &dc : kClasses) {
		for (size_t n = 1; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<int16_t> a = randVec(rng, n, dc.lo, dc.hi);
			const std::vector<int16_t> b = randVec(rng, n, dc.lo, dc.hi);
			const std::vector<int16_t> ao = a, bo = b;
			q15_t r = 1;
			arm_mse_q15(a.data(), b.data(), static_cast<uint32_t>(n), &r);
			CHECK(std::fabs(r - clampQ15(defMse(a, b))) <= mseBound(a, b));
			CHECK(a == ao);
			CHECK(b == bo);
		}
	}
}

TEST_CASE("[C3 arm_mse_q15] a strided sweep over all 16 bit values (6000 block pairs of 13, saturating pairs included) is within the derived bound") {
	size_t bad = 0;
	uint16_t k = 11;
	for (size_t blk = 0; blk < 6000; blk++) {
		std::vector<int16_t> a(13), b(13);
		for (size_t i = 0; i < 13; i++) { k = static_cast<uint16_t>(k + 40503u); a[i] = static_cast<int16_t>(k); }
		for (size_t i = 0; i < 13; i++) { k = static_cast<uint16_t>(k + 40503u); b[i] = static_cast<int16_t>(k); }
		q15_t r = 0;
		arm_mse_q15(a.data(), b.data(), 13, &r);
		if (!(std::fabs(r - clampQ15(defMse(a, b))) <= mseBound(a, b))) bad++;
	}
	CHECK(bad == 0);
}

TEST_CASE("[C4 arm_mse_q15] identical inputs give 0 and a hand derived pair gives 8192") {
	// identical: da = 0. Hand: a = 16384, b = 0: real (0.5 - 0)^2 = 0.25 -> 0.25 * 32768 = 8192; code: 8192^2 = 2^26, >> 13 = 8192 exactly
	const q15_t a[4] = {-32768, -5, 0, 32767};
	q15_t r = 1;
	arm_mse_q15(a, a, 4, &r);
	CHECK(r == 0);
	const q15_t x = 16384, y = 0;
	arm_mse_q15(&x, &y, 1, &r);
	CHECK(r == 8192);
}

TEST_CASE("[C4 arm_mse_q15] characterization: a mean square error of 1.0 or more saturates to 32767 (DOC silent; the saturation is only in the code, arm_mse_q15.c:171)") {
	// a = 32767, b = -32768: real (1.99997)^2 = 4.0 > 1: saturated 32767; the same for the swapped inputs (the error is squared)
	const q15_t hi = 32767, lo = -32768;
	q15_t r = 1;
	arm_mse_q15(&hi, &lo, 1, &r);
	CHECK(r == 32767);
	r = 1;
	arm_mse_q15(&lo, &hi, 1, &r);
	CHECK(r == 32767);
	const std::vector<int16_t> a(9, 32767), b(9, -32768);
	r = 1;
	arm_mse_q15(a.data(), b.data(), 9, &r);
	CHECK(r == 32767);
}

// ===============================================================================================================
// C5: q15 against the f32 function of the same input (x / 32768, exact in f32)
// ===============================================================================================================
namespace {

std::vector<float> toF32(const std::vector<int16_t> &x) {
	std::vector<float> f(x.size());
	for (size_t i = 0; i < x.size(); i++) f[i] = static_cast<float>(x[i]) / 32768.0f;
	return f;
}

}  // namespace

TEST_CASE("[C5 arm_max_q15] value and index of the eight max / min units equal the f32 functions of the converted input (exact)") {
	Rng rng(201);
	for (size_t n = 1; n <= 30; n++) {
		CAPTURE(n);
		std::vector<int16_t> x = randVec(rng, n, -32767, 32767);   // -32768 left out: the q15 abs saturates, f32 does not
		const std::vector<float> f = toF32(x);
		const uint32_t nn = static_cast<uint32_t>(n);
		q15_t qv = 0;
		float fv = 0;
		uint32_t qi = 0, fi = 0;
		arm_max_q15(x.data(), nn, &qv, &qi);
		arm_max_f32(f.data(), nn, &fv, &fi);
		CHECK(qi == fi);
		CHECK(static_cast<float>(qv) / 32768.0f == fv);
		arm_min_q15(x.data(), nn, &qv, &qi);
		arm_min_f32(f.data(), nn, &fv, &fi);
		CHECK(qi == fi);
		CHECK(static_cast<float>(qv) / 32768.0f == fv);
		arm_absmax_q15(x.data(), nn, &qv, &qi);
		arm_absmax_f32(f.data(), nn, &fv, &fi);
		CHECK(qi == fi);
		CHECK(static_cast<float>(qv) / 32768.0f == fv);
		arm_absmin_q15(x.data(), nn, &qv, &qi);
		arm_absmin_f32(f.data(), nn, &fv, &fi);
		CHECK(qi == fi);
		CHECK(static_cast<float>(qv) / 32768.0f == fv);
	}
}

TEST_CASE("[C5 arm_mean_q15] the mean agrees with arm_mean_f32 of the converted input within 1 LSB plus the f32 summation error" * MEAN_UB) {
	Rng rng(202);
	for (const DataClass &dc : kClasses) {
		for (size_t n = 1; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<int16_t> x = randVec(rng, n, dc.lo, dc.hi);
			const std::vector<float> f = toF32(x);
			q15_t q = 0;
			float fr = 0;
			arm_mean_q15(x.data(), static_cast<uint32_t>(n), &q);
			arm_mean_f32(f.data(), static_cast<uint32_t>(n), &fr);
			// truncation < 1 LSB; f32 mean: |err| <= gam(n) * sum|x| / n (real units), times 32768 for LSB
			CHECK(std::fabs(q - static_cast<double>(fr) * 32768.0) <= 1.0 + gam(static_cast<double>(n)) * sumAbs(x) / static_cast<double>(n));
		}
	}
}

TEST_CASE("[C5 arm_power_q15] the power agrees with arm_power_f32 of the converted input times 2^30 within the f32 relative error") {
	Rng rng(203);
	for (const DataClass &dc : kClasses) {
		for (size_t n = 1; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<int16_t> x = randVec(rng, n, dc.lo, dc.hi);
			const std::vector<float> f = toF32(x);
			q63_t q = 0;
			float fr = 0;
			arm_power_q15(x.data(), static_cast<uint32_t>(n), &q);
			arm_power_f32(f.data(), static_cast<uint32_t>(n), &fr);
			// f32: n squares and n-1 additions of non-negative terms: relative error <= gam(n); q15 is exact
			CHECK(std::fabs(static_cast<double>(q) - static_cast<double>(fr) * 1073741824.0) <= gam(static_cast<double>(n)) * static_cast<double>(q));
		}
	}
}

TEST_CASE("[C5 arm_rms_q15] the rms agrees with arm_rms_f32 of the converted input within the q15 and f32 bounds") {
	Rng rng(204);
	for (const DataClass &dc : kClasses) {
		for (size_t n = 1; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<int16_t> x = randVec(rng, n, dc.lo, dc.hi);
			const std::vector<float> f = toF32(x);
			q15_t q = 0;
			float fr = 0;
			arm_rms_q15(x.data(), static_cast<uint32_t>(n), &q);
			arm_rms_f32(f.data(), static_cast<uint32_t>(n), &fr);
			const double qq = sumSq(x) / static_cast<double>(n) / 32768.0;
			const double def = std::sqrt(sumSq(x) / static_cast<double>(n));
			const double fBound = def * (gam(static_cast<double>(n) + 1.0) / 2.0 + 1.01 * kEps);   // f32 rms relative error (power/n then sqrt)
			CHECK(std::fabs(q - static_cast<double>(fr) * 32768.0) <= sqrtBound(qq, qq - 1.0, qq) + fBound);
		}
	}
}

namespace {
// f32 variance error bound in real units (two pass: mean off by D = gam(n) sum|x| / n): var*gam(n+3) + n*D^2/(n-1)*(1+gam(n+3))
double f32VarBound(const std::vector<int16_t> &x, double varReal) {
	const double n = static_cast<double>(x.size());
	const double D = gam(n) * sumAbs(x) / 32768.0 / n;
	return varReal * gam(n + 3.0) + n * D * D / (n - 1.0) * (1.0 + gam(n + 3.0));
}
}  // namespace

TEST_CASE("[C5 arm_var_q15] the variance agrees with arm_var_f32 of the converted input within the q15 and f32 bounds") {
	Rng rng(205);
	for (const DataClass &dc : kClasses) {
		if (dc.hi - dc.lo > 40000) continue;
		for (size_t n = 2; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<int16_t> x = randVec(rng, n, dc.lo, dc.hi);
			const std::vector<float> f = toF32(x);
			q15_t q = 0;
			float fr = 0;
			arm_var_q15(x.data(), static_cast<uint32_t>(n), &q);
			arm_var_f32(f.data(), static_cast<uint32_t>(n), &fr);
			const double def = defVar(x);
			CHECK(std::fabs(q - static_cast<double>(fr) * 32768.0) <= 1.0 + kHalf15 + 32768.0 * f32VarBound(x, def / 32768.0) + 1e-9);
		}
	}
}

TEST_CASE("[C5 arm_std_q15] the std agrees with arm_std_f32 of the converted input within the q15 and f32 bounds") {
	Rng rng(206);
	for (const DataClass &dc : kClasses) {
		if (dc.hi - dc.lo > 40000) continue;
		for (size_t n = 2; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<int16_t> x = randVec(rng, n, dc.lo, dc.hi);
			const std::vector<float> f = toF32(x);
			q15_t q = 0;
			float fr = 0;
			arm_std_q15(x.data(), static_cast<uint32_t>(n), &q);
			arm_std_f32(f.data(), static_cast<uint32_t>(n), &fr);
			const double def = defVar(x);
			const double varReal = def / 32768.0;
			const double relVar = f32VarBound(x, varReal) / varReal;
			const double fBound = std::sqrt(32768.0 * def) * ((relVar / 2.0) * 1.01 + 1.01 * kEps);
			CHECK(std::fabs(q - static_cast<double>(fr) * 32768.0) <= sqrtBound(def, def - 1.0 - kHalf15, def + kHalf15) + fBound);
		}
	}
}

TEST_CASE("[C5 arm_mse_q15] the mse agrees with arm_mse_f32 of the converted input within the q15 and f32 bounds") {
	Rng rng(207);
	for (const DataClass &dc : kClasses) {
		if (dc.hi - dc.lo > 40000) continue;   // mse of full range data saturates in q15 (C4)
		for (size_t n = 1; n <= 40; n++) {
			CAPTURE(dc.name);
			CAPTURE(n);
			const std::vector<int16_t> a = randVec(rng, n, dc.lo, dc.hi);
			const std::vector<int16_t> b = randVec(rng, n, dc.lo, dc.hi);
			const std::vector<float> fa = toF32(a), fb = toF32(b);
			q15_t q = 0;
			float fr = 0;
			arm_mse_q15(a.data(), b.data(), static_cast<uint32_t>(n), &q);
			arm_mse_f32(fa.data(), fb.data(), static_cast<uint32_t>(n), &fr);
			const double def = defMse(a, b);
			// f32: relative error gam(n+3) (difference, square, n-1 additions, division)
			CHECK(std::fabs(q - static_cast<double>(fr) * 32768.0) <= mseBound(a, b) + def * gam(static_cast<double>(n) + 3.0));
		}
	}
}

}  // TEST_SUITE
