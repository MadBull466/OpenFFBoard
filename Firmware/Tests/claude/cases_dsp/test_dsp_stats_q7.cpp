/*
 * CMSIS-DSP Statistics q7 (work order SQ7, phase P11 group G3). Requirement ids: Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * Units covered (all 11 q7 units of Source/StatisticsFunctions): arm_max_q7, arm_min_q7, arm_absmax_q7, arm_absmin_q7 (value and
 * index), arm_max_no_idx_q7, arm_min_no_idx_q7, arm_absmax_no_idx_q7, arm_absmin_no_idx_q7, arm_mean_q7, arm_power_q7, arm_mse_q7.
 * There is no q7 accumulate / rms / std / var / entropy / KL / logsumexp (ARM's tests for rms, std and var are under `#if 0` in
 * StatsTestsQ7.cpp:290-:358, "those functions do not yet exist": nothing to run).
 *
 * Pattern files used (Libraries/CMSIS-DSP/Testing/Patterns/DSP/Stats/StatsQ7/). ARM's tests: Testing/Source/Tests/StatsTestsQ7.cpp
 * (test functions :24-:290, setUp :360-:end), suite "Statistics Tests Q7" in Testing/desc.txt:466-:563 (40 test ids, all run here,
 * none commented out or disabled). "nb=15 / 16n / 16n+1" are the first 15, 32 and 47 samples of the pattern (inputA.reload(..., nb)),
 * "big index" = first 280 samples (index above 255), "full" = the whole pattern; refOffset selects the 0th..4th reference value:
 *   max / min       : Input1_q7 -> MaxVals1 + MaxIndexes1_s16 / MinVals3 + MinIndexes3_s16 (nb 15, 32, 47; big index:
 *                     InputMaxIndexMax1 / InputMinIndexMax3, 280 samples, offset 3)
 *   absmax / absmin : InputNew1_q7 -> AbsMaxVals8 + AbsMaxIndexes8_s16 / AbsMinVals9 + AbsMinIndexes9_s16 (nb 15, 32, 47; big index:
 *                     InputAbsMaxIndexMax8 offset 4 / InputAbsMinIndexMax9 offset 3, 280 samples; absmax "saturation" id: full
 *                     InputNew1, offset 3, value and index)
 *   *_no_idx        : same inputs and values, no index; absmax_no_idx saturation id: full InputNew1, offset 3
 *   mean            : Input2_q7 -> MeanVals2 (nb 15, 32, 47)
 *   power           : Input1_q7 -> PowerVals4_q31 (nb 15, 32, 47)
 *   mse             : InputNew1 + InputNew2 -> MSEVals10 (nb 15, 32, 47, long 100)
 *
 * ARM tolerances (StatsTestsQ7.cpp:8-:20): SNR_THRESHOLD 20 dB (:8), SNR_THRESHOLD_MSE 20 dB (:9), ABS_ERROR_Q7 20 LSB (:17), ABS_ERROR_Q31
 * 2^15 (:18), ABS_ERROR_Q7_MSE 6 LSB (:20); mean uses SNR 5 dB (:226) and 20 LSB. max / min / absmax / absmin values and all indexes are
 * compared EXACT (ASSERT_EQ; the index cast to int16_t as in ARM). Per sample bounds via dsp::withinLsb, SNR via dsp::snr (double).
 *
 * DEF tests (C3): the reference is evaluated in double / exact integers inside this file, never with a library function.
 *   Domain: the q7 domain is tiny, so the sweeps are EXHAUSTIVE: every single sample -128..127 (all functions), every pair (a, b) of
 *   256 x 256 values (max, min, absmax, absmin, mean with n = 2; mse with n = 1 on all pairs), the extremum planted at every position
 *   of blocks of 1..40 samples (and of a 1000 sample block incl. the index range above 255), plus random blocks of 1..40 samples.
 *   absmax / absmin: |-128| saturates to 127 (ARM's reference pattern AbsMaxVals8 has the saturation test, StatsTestsQ7 setUp ids 21 / 39 / 40).
 *   max / min / absmax / absmin: value exact; index exact when the extremum is unique, for ties see below.
 *   mean   : DOC (arm_mean_q7.c:36-:41) "accumulated in a 32-bit accumulator in 25.7 format ... no risk of internal overflow ...
 *            Finally, the accumulator is truncated to yield a result of 1.7 format". For a non-negative sum "truncated" is floor(sum / n)
 *            (exact). For a negative sum floor and truncation toward zero differ and the DOC does not say: general tests accept both
 *            (|result - sum/n| < 1), the toward-zero behaviour is a `characterization:`.
 *   power  : DOC (arm_power_q7.c:36-:43) "Intermediate multiplication yields a 2.14 format, and this result is added without
 *            saturation to an accumulator in 18.14 format ... 17 guard bits ... no risk of overflow, full precision preserved": the result
 *            is EXACTLY the integer sum of a_i^2 (a * 2^-7 squared is a^2 * 2^-14). Overflow boundary: n = 131071 samples of -128 gives
 *            131071 * 16384 = 2147467264 < 2^31 and is exact.
 *   mse    : DOC (arm_mse_q7.c:37-:43) gives no scaling. Hand derivation of the q7 result format from ARM's reference MSEVals10_q7
 *            (C1 passes within 6 LSB) and the source's 1.7 output: mse_q7 = 128 * mean(((a - b) / 128)^2) = mean((a-b)^2) / 128, saturating at
 *            127 (the true mse can reach 4.0 which a q7 cannot hold; DOC silent about saturation, listed in the report). General bound
 *            (rounding unspecified): the halved inputs a>>1 = a/2 - e, e in {0, 1/2} give d = D + delta, D = (a-b)/2, |delta| <= 1/2, so
 *            |d^2 - D^2| <= |D| + 1/4; with E = mean|D| + 1/4: S/n is within E of mean D^2; the integer division by n and the final
 *            scaling by 1/32 lose less than 1 + 1 (truncation); so  T - (E+1)/32 - 1 <= r <= T + E/32  (T = mean D^2 / 32 = DEF), both
 *            clamped to 127. For inputs with a and b EVEN the halves are exact (delta = 0), the only loss is the final truncation, and the
 *            result is EXACTLY floor(sum D^2 / (32 n)) clamped to 127: tested as `characterization:` (truncation is not documented).
 *
 * C5: q7 against the f32 library on the same data x/128: max / min / absmax / absmin (value * 128 exact, index when unique), mean
 * (|r - 128 m| < 1: truncation), power (n <= 64: all partial sums are multiples of 2^-14 below 2^6, exact in binary32, so
 * round(128^2 * f32 power) == q7 power exactly), mse (bound above with T = 128 * f32 mse).
 *
 * Ties (index rule): DOC says nothing. The source uses strict comparisons (first occurrence wins, like ARM's numpy-generated patterns):
 * `characterization:` tests, listed in the report. Not tested (NEEDS REVIEW, lead decision): blockSize 0 for the max / min / absmax /
 * absmin family (pSrc[0] is read unconditionally and ((0-1)>>2) loop passes follow: out of bounds) and for arm_mean_q7 / arm_mse_q7
 * (integer division by blockSize 0). arm_power_q7 with blockSize 0 returns 0 (characterization).
 *
 * NEEDS REVIEW (not tested, lead decisions; probes by the reviewer):
 *  - arm_mean_q7 accumulator wrap: DOC claims 25.7 format and no risk of internal overflow, but n = 16777217 samples of -128 gives 127
 *    and n = 16909321 samples of 127 gives -126 (32 bit signed sum wraps; the exact boundary n = 16777216 is tested).
 *  - arm_mse_q7.c:131 signed overflow of the 32 bit sum for n >= 133143 with full-scale differences (the unsigned division hides it up to
 *    n = 266287; n = 266288 returns 127 and n = 300000 returns 56 where 127 is expected).
 *  - arm_power_q7.c:146 signed overflow at n = 131072 samples of -128 (gives INT_MIN); the test stops at 131071.
 *
 * KNOWN ISSUE (undefined behaviour): B-DSQ7-1 arm_mean_q7, unrolled loop (arm_mean_q7.c:119-:121) 'sum += ((in << 24U) >> 24U)' (also
 * << 16U, << 8U) shifts a q31 (int) left so that bits move into or past the sign bit (and in may be negative). The result on the host is
 * the intended one (two's complement). UBSan reports first the host-only branch of read_q7x4 (arm_math_memory.h:178,
 * '(pQ7[3] & 0xFF) << 24' into the sign bit) and then arm_mean_q7.c:119-:121. All mean tests with blockSize >= 4 are KNOWN_ISSUE_UB_HIDDEN (skipped in sanitizer builds).
 *
 * Host vs target (C6): the host runs the portable C path (ARM_MATH_LOOPUNROLL defined, no ARM_MATH_DSP, no MVE): arm_absmax_q7 /
 * arm_absmin_q7 (and no_idx) use the 'saturating negate' ternary of the #else branch, on the target with ARM_MATH_DSP __QSUB8(0, x).
 * Both give 127 for -128. On the target with ARM_MATH_DSP arm_power_q7 uses the __SXTB16 / __SMLAD path (arm_power_q7.c:126-:134) and
 * absmax / absmin q7 use __QSUB8 with the 'index + k' unrolled loops; none of these paths run on the host. The mean / mse arithmetic is the same on both (32 bit integer sums, no floats, no 64 bit types); 'x >> 1' and
 * '(in << n) >> 24' rely on arithmetic right shift of negative values (implementation defined in C, arithmetic on gcc / clang for host
 * and target). The C5 tests use f32 values that are exact (x / 128) so FMA contraction cannot matter.
 */
#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math.h"

namespace {

const char *const kDir = "Stats/StatsQ7/";

// ARM tolerances (StatsTestsQ7.cpp:8-:20)
const double kSnr = 20.0;
const double kSnrMean = 5.0;
const double kSnrMse = 20.0;
const int64_t kAbsQ7 = 20;
const int64_t kAbsQ31 = 1 << 15;
const int64_t kAbsMse = 6;

typedef std::vector<int8_t> V;

V pat(const std::string &name) { return dsp::loadPattern(kDir + name).q7(); }
std::vector<int16_t> patIdx(const std::string &name) { return dsp::loadPattern(kDir + name).q15(); }
std::vector<int32_t> patQ31(const std::string &name) { return dsp::loadPattern(kDir + name).q31(); }

// Deterministic generator (LCG, Numerical Recipes constants). One call per statement.
struct Rng {
	uint32_t s;
	explicit Rng(uint32_t seed) : s(seed) {}
	uint32_t next() { s = s * 1664525u + 1013904223u; return s; }
	int range(int lo, int hi) {                        // inclusive
		const uint32_t v = next() >> 8;
		return lo + static_cast<int>(v % static_cast<uint32_t>(hi - lo + 1));
	}
};

V randV(Rng &r, size_t n, int lo, int hi) {
	V v(n);
	for (size_t i = 0; i < n; i++) v[i] = static_cast<int8_t>(r.range(lo, hi));
	return v;
}

std::string dump(const V &x) {
	std::ostringstream o;
	o << "n=" << x.size() << " [";
	for (size_t i = 0; i < x.size() && i < 24; i++) o << (i ? "," : "") << static_cast<int>(x[i]);
	if (x.size() > 24) o << ",...";
	o << "]";
	return o.str();
}

// Collects failures of a sweep so that one CHECK reports the count and the first case
struct Fails {
	size_t count = 0;
	std::string first;
	void add(const std::string &s) { if (!count) first = s; count++; }
};
#define CHECK_NOFAIL(f) CHECK_MESSAGE((f).count == 0, "failures: " << (f).count << ", first: " << (f).first)

// ARM's accuracy checks for one scalar result
template <class T>
void checkArmScalar(T ref, T out, double snrThr, int64_t lsb) {
	CHECK(dsp::snr(std::vector<T>{ref}, std::vector<T>{out}) >= snrThr);
	auto c = dsp::withinLsb(std::vector<T>{ref}, std::vector<T>{out}, lsb);
	CHECK_MESSAGE(c, c.message);
}

// ---------------------------------------------------------------------------------------------------------------
// function tables
// ---------------------------------------------------------------------------------------------------------------
enum Mode { MAX = 0, MIN = 1, ABSMAX = 2, ABSMIN = 3 };
typedef void (*IdxFn)(const q7_t *, uint32_t, q7_t *, uint32_t *);
typedef void (*NoIdxFn)(const q7_t *, uint32_t, q7_t *);
typedef void (*F32IdxFn)(const float *, uint32_t, float *, uint32_t *);

const IdxFn kIdx[4] = {arm_max_q7, arm_min_q7, arm_absmax_q7, arm_absmin_q7};
const NoIdxFn kNoIdx[4] = {arm_max_no_idx_q7, arm_min_no_idx_q7, arm_absmax_no_idx_q7, arm_absmin_no_idx_q7};
const F32IdxFn kIdxF32[4] = {arm_max_f32, arm_min_f32, arm_absmax_f32, arm_absmin_f32};
const char *const kName[4] = {"max", "min", "absmax", "absmin"};

int absSat(int x) { return std::min(std::abs(x), 127); }   // |-128| saturates to 127 (q7 cannot hold 128)

int keyOf(Mode m, int x) {
	switch (m) {
	case MAX: return x;
	case MIN: return -x;
	case ABSMAX: return absSat(x);
	default: return -absSat(x);
	}
}
int valueOf(Mode m, int x) { return (m == ABSMAX || m == ABSMIN) ? absSat(x) : x; }

struct Ext { int value; size_t index; int key; bool unique; };
// DEF: first occurrence of the extreme key (abs variants return the saturated absolute value)
Ext defExt(Mode m, const V &x) {
	size_t bi = 0;
	for (size_t i = 1; i < x.size(); i++)
		if (keyOf(m, x[i]) > keyOf(m, x[bi])) bi = i;
	const int key = keyOf(m, x[bi]);
	int cnt = 0;
	for (size_t i = 0; i < x.size(); i++) cnt += keyOf(m, x[i]) == key;
	return {valueOf(m, x[bi]), bi, key, cnt == 1};
}

// Runs the idx and the no_idx function and compares with DEF. Value exact for both. Index: exact if the extremum is unique,
// otherwise it must point at an element holding the extreme key (the first-occurrence rule is the characterization test).
void checkExt(Mode m, const V &x, Fails &f) {
	const Ext e = defExt(m, x);
	q7_t r = 0x55, r2 = 0x55;
	uint32_t idx = 0xFFFFFFFFu;
	kIdx[m](x.data(), static_cast<uint32_t>(x.size()), &r, &idx);
	kNoIdx[m](x.data(), static_cast<uint32_t>(x.size()), &r2);
	std::ostringstream o;
	o << kName[m] << " " << dump(x) << ": ";
	if (r != e.value) { o << "value " << static_cast<int>(r) << " expected " << e.value; f.add(o.str()); return; }
	if (r2 != e.value) { o << "no_idx value " << static_cast<int>(r2) << " expected " << e.value; f.add(o.str()); return; }
	if (idx >= x.size()) { o << "index " << idx << " out of range"; f.add(o.str()); return; }
	if (e.unique ? idx != e.index : keyOf(m, x[idx]) != e.key) { o << "index " << idx << " expected " << e.index << (e.unique ? " (unique)" : " (tie)"); f.add(o.str()); }
}

// ---------------------------------------------------------------------------------------------------------------
// C1 helpers: ARM's tests
// ---------------------------------------------------------------------------------------------------------------
// test_max_q7 / test_absmax_q7 / test_min_q7 / test_absmin_q7: exact value and exact index (nb 0 = the whole pattern)
void armIdx(Mode m, const char *input, const char *vals, const char *idxs, size_t nb, size_t refOffset) {
	V in = pat(input);
	const V ref = pat(vals);
	const std::vector<int16_t> refInd = patIdx(idxs);
	if (nb) { REQUIRE(in.size() >= nb); in.resize(nb); }
	REQUIRE(ref.size() > refOffset);
	REQUIRE(refInd.size() > refOffset);
	q7_t result = 0x55;
	uint32_t indexval = 0xFFFFFFFFu;
	kIdx[m](in.data(), static_cast<uint32_t>(in.size()), &result, &indexval);
	CHECK(result == ref[refOffset]);
	CHECK(static_cast<int16_t>(indexval) == refInd[refOffset]);
}

void armNoIdx(Mode m, const char *input, const char *vals, size_t nb, size_t refOffset) {
	V in = pat(input);
	const V ref = pat(vals);
	if (nb) { REQUIRE(in.size() >= nb); in.resize(nb); }
	REQUIRE(ref.size() > refOffset);
	q7_t result = 0x55;
	kNoIdx[m](in.data(), static_cast<uint32_t>(in.size()), &result);
	CHECK(result == ref[refOffset]);
}

void armMean(size_t nb, size_t refOffset) {
	V in = pat("Input2_q7.txt");
	const V ref = pat("MeanVals2_q7.txt");
	REQUIRE(in.size() >= nb);
	REQUIRE(ref.size() > refOffset);
	in.resize(nb);
	q7_t result = 0x55;
	arm_mean_q7(in.data(), static_cast<uint32_t>(nb), &result);
	checkArmScalar<int8_t>(ref[refOffset], result, kSnrMean, kAbsQ7);
}

void armPower(size_t nb, size_t refOffset) {
	V in = pat("Input1_q7.txt");
	const std::vector<int32_t> ref = patQ31("PowerVals4_q31.txt");
	REQUIRE(in.size() >= nb);
	REQUIRE(ref.size() > refOffset);
	in.resize(nb);
	q31_t result = 0x55555555;
	arm_power_q7(in.data(), static_cast<uint32_t>(nb), &result);
	checkArmScalar<int32_t>(ref[refOffset], result, kSnr, kAbsQ31);
}

void armMse(size_t nb, size_t refOffset) {
	V a = pat("InputNew1_q7.txt"), b = pat("InputNew2_q7.txt");
	const V ref = pat("MSEVals10_q7.txt");
	REQUIRE(a.size() >= nb);
	REQUIRE(b.size() >= nb);
	REQUIRE(ref.size() > refOffset);
	a.resize(nb);
	b.resize(nb);
	q7_t result = 0x55;
	arm_mse_q7(a.data(), b.data(), static_cast<uint32_t>(nb), &result);
	checkArmScalar<int8_t>(ref[refOffset], result, kSnrMse, kAbsMse);
}

// ---------------------------------------------------------------------------------------------------------------
// DEF helpers
// ---------------------------------------------------------------------------------------------------------------
long sumOf(const V &x) { long s = 0; for (int8_t v : x) s += v; return s; }

// mean: DEF check, see the header. Non-negative sum: exactly floor(sum/n). Negative sum: within one LSB of the true mean.
bool meanOk(const V &x, int r) {
	const long s = sumOf(x), n = static_cast<long>(x.size());
	if (s >= 0) return r == s / n;
	const long d = std::labs(static_cast<long>(r) * n - s);   // |r - mean| * n
	return d < n;
}

long defPower(const V &x) { long s = 0; for (int8_t v : x) s += static_cast<long>(v) * v; return s; }

// mse DEF: T = mean((a-b)^2) / 128 and the general bound derived in the header. Returns "" if ok.
std::string mseBound(const V &a, const V &b, int r, double T) {
	double meanAbsD = 0;
	for (size_t i = 0; i < a.size(); i++) meanAbsD += std::fabs((static_cast<int>(a[i]) - b[i]) / 2.0);
	meanAbsD /= static_cast<double>(a.size());
	const double E = meanAbsD + 0.25;
	const double lo = std::min(127.0, T - (E + 1.0) / 32.0 - 1.0);
	const double hi = std::min(127.0, T + E / 32.0);
	if (r >= lo && r <= hi) return "";
	std::ostringstream o;
	o << "mse of " << dump(a) << " and " << dump(b) << ": result " << r << " outside [" << lo << ", " << hi << "] (T = " << T << ")";
	return o.str();
}
double defMseT(const V &a, const V &b) {
	double s = 0;
	for (size_t i = 0; i < a.size(); i++) { const double d = static_cast<double>(a[i]) - b[i]; s += d * d; }
	return s / static_cast<double>(a.size()) / 128.0;
}

const uint32_t kSizes[] = {1, 2, 3, 4, 5, 9};   // 4 is one full unrolled group (blockSize-1 = 3 is tail only, 5 and 9 add groups)

}  // namespace

#define UB_MEAN "B-DSQ7-1: arm_mean_q7 (arm_mean_q7.c:119-:121) in the unrolled loop evaluates '(in << 24U) >> 24U', '(in << 16U) >> 24U', '(in << 8U) >> 24U' on a q31 (int): left shift into / past the sign bit and of negative values is undefined in C; the intended two's complement result occurs on the host"

TEST_SUITE("DSP Statistics q7") {

// ===============================================================================================================
// C1: ARM's test ids
// ===============================================================================================================

TEST_CASE("[C1 arm_max_q7] max and index of the first 15 samples (ARM nb=15) equal ARM's reference exactly") { armIdx(MAX, "Input1_q7.txt", "MaxVals1_q7.txt", "MaxIndexes1_s16.txt", 15, 0); }
TEST_CASE("[C1 arm_max_q7] max and index of the first 32 samples (ARM nb=16n) equal ARM's reference exactly") { armIdx(MAX, "Input1_q7.txt", "MaxVals1_q7.txt", "MaxIndexes1_s16.txt", 32, 1); }
TEST_CASE("[C1 arm_max_q7] max and index of the first 47 samples (ARM nb=16n+1) equal ARM's reference exactly") { armIdx(MAX, "Input1_q7.txt", "MaxVals1_q7.txt", "MaxIndexes1_s16.txt", 47, 2); }
TEST_CASE("[C1 arm_max_q7] max and index of 280 samples with the maximum above index 255 (ARM big index) equal ARM's reference exactly") { armIdx(MAX, "InputMaxIndexMax1_q7.txt", "MaxVals1_q7.txt", "MaxIndexes1_s16.txt", 280, 3); }
TEST_CASE("[C1 arm_min_q7] min and index of the first 15 samples (ARM nb=15) equal ARM's reference exactly") { armIdx(MIN, "Input1_q7.txt", "MinVals3_q7.txt", "MinIndexes3_s16.txt", 15, 0); }
TEST_CASE("[C1 arm_min_q7] min and index of the first 32 samples (ARM nb=16n) equal ARM's reference exactly") { armIdx(MIN, "Input1_q7.txt", "MinVals3_q7.txt", "MinIndexes3_s16.txt", 32, 1); }
TEST_CASE("[C1 arm_min_q7] min and index of the first 47 samples (ARM nb=16n+1) equal ARM's reference exactly") { armIdx(MIN, "Input1_q7.txt", "MinVals3_q7.txt", "MinIndexes3_s16.txt", 47, 2); }
TEST_CASE("[C1 arm_min_q7] min and index of 280 samples with the minimum above index 255 (ARM big index) equal ARM's reference exactly") { armIdx(MIN, "InputMinIndexMax3_q7.txt", "MinVals3_q7.txt", "MinIndexes3_s16.txt", 280, 3); }
TEST_CASE("[C1 arm_absmax_q7] absmax and index of the first 15 samples (ARM nb=15) equal ARM's reference exactly") { armIdx(ABSMAX, "InputNew1_q7.txt", "AbsMaxVals8_q7.txt", "AbsMaxIndexes8_s16.txt", 15, 0); }
TEST_CASE("[C1 arm_absmax_q7] absmax and index of the first 32 samples (ARM nb=16n) equal ARM's reference exactly") { armIdx(ABSMAX, "InputNew1_q7.txt", "AbsMaxVals8_q7.txt", "AbsMaxIndexes8_s16.txt", 32, 1); }
TEST_CASE("[C1 arm_absmax_q7] absmax and index of the first 47 samples (ARM nb=16n+1) equal ARM's reference exactly") { armIdx(ABSMAX, "InputNew1_q7.txt", "AbsMaxVals8_q7.txt", "AbsMaxIndexes8_s16.txt", 47, 2); }
TEST_CASE("[C1 arm_absmax_q7] absmax and index of 280 samples with the maximum above index 255 (ARM big index) equal ARM's reference exactly") { armIdx(ABSMAX, "InputAbsMaxIndexMax8_q7.txt", "AbsMaxVals8_q7.txt", "AbsMaxIndexes8_s16.txt", 280, 4); }
TEST_CASE("[C1 arm_absmax_q7] absmax and index of the whole InputNew1 pattern (ARM saturation id) equal ARM's reference exactly") { armIdx(ABSMAX, "InputNew1_q7.txt", "AbsMaxVals8_q7.txt", "AbsMaxIndexes8_s16.txt", 0, 3); }
TEST_CASE("[C1 arm_absmin_q7] absmin and index of the first 15 samples (ARM nb=15) equal ARM's reference exactly") { armIdx(ABSMIN, "InputNew1_q7.txt", "AbsMinVals9_q7.txt", "AbsMinIndexes9_s16.txt", 15, 0); }
TEST_CASE("[C1 arm_absmin_q7] absmin and index of the first 32 samples (ARM nb=16n) equal ARM's reference exactly") { armIdx(ABSMIN, "InputNew1_q7.txt", "AbsMinVals9_q7.txt", "AbsMinIndexes9_s16.txt", 32, 1); }
TEST_CASE("[C1 arm_absmin_q7] absmin and index of the first 47 samples (ARM nb=16n+1) equal ARM's reference exactly") { armIdx(ABSMIN, "InputNew1_q7.txt", "AbsMinVals9_q7.txt", "AbsMinIndexes9_s16.txt", 47, 2); }
TEST_CASE("[C1 arm_absmin_q7] absmin and index of 280 samples with the minimum above index 255 (ARM big index) equal ARM's reference exactly") { armIdx(ABSMIN, "InputAbsMinIndexMax9_q7.txt", "AbsMinVals9_q7.txt", "AbsMinIndexes9_s16.txt", 280, 3); }

TEST_CASE("[C1 arm_max_no_idx_q7] max of the first 15 samples (ARM nb=15) equals ARM's reference exactly") { armNoIdx(MAX, "Input1_q7.txt", "MaxVals1_q7.txt", 15, 0); }
TEST_CASE("[C1 arm_max_no_idx_q7] max of the first 32 samples (ARM nb=16n) equals ARM's reference exactly") { armNoIdx(MAX, "Input1_q7.txt", "MaxVals1_q7.txt", 32, 1); }
TEST_CASE("[C1 arm_max_no_idx_q7] max of the first 47 samples (ARM nb=16n+1) equals ARM's reference exactly") { armNoIdx(MAX, "Input1_q7.txt", "MaxVals1_q7.txt", 47, 2); }
TEST_CASE("[C1 arm_min_no_idx_q7] min of the first 15 samples (ARM nb=15) equals ARM's reference exactly") { armNoIdx(MIN, "Input1_q7.txt", "MinVals3_q7.txt", 15, 0); }
TEST_CASE("[C1 arm_min_no_idx_q7] min of the first 32 samples (ARM nb=16n) equals ARM's reference exactly") { armNoIdx(MIN, "Input1_q7.txt", "MinVals3_q7.txt", 32, 1); }
TEST_CASE("[C1 arm_min_no_idx_q7] min of the first 47 samples (ARM nb=16n+1) equals ARM's reference exactly") { armNoIdx(MIN, "Input1_q7.txt", "MinVals3_q7.txt", 47, 2); }
TEST_CASE("[C1 arm_absmax_no_idx_q7] absmax of the first 15 samples (ARM nb=15) equals ARM's reference exactly") { armNoIdx(ABSMAX, "InputNew1_q7.txt", "AbsMaxVals8_q7.txt", 15, 0); }
TEST_CASE("[C1 arm_absmax_no_idx_q7] absmax of the first 32 samples (ARM nb=16n) equals ARM's reference exactly") { armNoIdx(ABSMAX, "InputNew1_q7.txt", "AbsMaxVals8_q7.txt", 32, 1); }
TEST_CASE("[C1 arm_absmax_no_idx_q7] absmax of the first 47 samples (ARM nb=16n+1) equals ARM's reference exactly") { armNoIdx(ABSMAX, "InputNew1_q7.txt", "AbsMaxVals8_q7.txt", 47, 2); }
TEST_CASE("[C1 arm_absmax_no_idx_q7] absmax of the whole InputNew1 pattern (ARM saturation id) equals ARM's reference exactly") { armNoIdx(ABSMAX, "InputNew1_q7.txt", "AbsMaxVals8_q7.txt", 0, 3); }
TEST_CASE("[C1 arm_absmin_no_idx_q7] absmin of the first 15 samples (ARM nb=15) equals ARM's reference exactly") { armNoIdx(ABSMIN, "InputNew1_q7.txt", "AbsMinVals9_q7.txt", 15, 0); }
TEST_CASE("[C1 arm_absmin_no_idx_q7] absmin of the first 32 samples (ARM nb=16n) equals ARM's reference exactly") { armNoIdx(ABSMIN, "InputNew1_q7.txt", "AbsMinVals9_q7.txt", 32, 1); }
TEST_CASE("[C1 arm_absmin_no_idx_q7] absmin of the first 47 samples (ARM nb=16n+1) equals ARM's reference exactly") { armNoIdx(ABSMIN, "InputNew1_q7.txt", "AbsMinVals9_q7.txt", 47, 2); }

TEST_CASE("[C1 arm_mean_q7] mean of the first 15 samples (ARM nb=15) matches ARM's reference within 5 dB and 20 LSB" * KNOWN_ISSUE_UB_HIDDEN(UB_MEAN)) { armMean(15, 0); }
TEST_CASE("[C1 arm_mean_q7] mean of the first 32 samples (ARM nb=16n) matches ARM's reference within 5 dB and 20 LSB" * KNOWN_ISSUE_UB_HIDDEN(UB_MEAN)) { armMean(32, 1); }
TEST_CASE("[C1 arm_mean_q7] mean of the first 47 samples (ARM nb=16n+1) matches ARM's reference within 5 dB and 20 LSB" * KNOWN_ISSUE_UB_HIDDEN(UB_MEAN)) { armMean(47, 2); }

TEST_CASE("[C1 arm_power_q7] power of the first 15 samples (ARM nb=15) matches ARM's reference within 20 dB and 2^15") { armPower(15, 0); }
TEST_CASE("[C1 arm_power_q7] power of the first 32 samples (ARM nb=16n) matches ARM's reference within 20 dB and 2^15") { armPower(32, 1); }
TEST_CASE("[C1 arm_power_q7] power of the first 47 samples (ARM nb=16n+1) matches ARM's reference within 20 dB and 2^15") { armPower(47, 2); }

TEST_CASE("[C1 arm_mse_q7] mse of the first 15 samples (ARM nb=15) matches ARM's reference within 20 dB and 6 LSB") { armMse(15, 0); }
TEST_CASE("[C1 arm_mse_q7] mse of the first 32 samples (ARM nb=16n) matches ARM's reference within 20 dB and 6 LSB") { armMse(32, 1); }
TEST_CASE("[C1 arm_mse_q7] mse of the first 47 samples (ARM nb=16n+1) matches ARM's reference within 20 dB and 6 LSB") { armMse(47, 2); }
TEST_CASE("[C1 arm_mse_q7] mse of the first 100 samples (ARM long) matches ARM's reference within 20 dB and 6 LSB") { armMse(100, 3); }

// ===============================================================================================================
// C2: nothing written beyond the result / index, inputs untouched
// ===============================================================================================================

TEST_CASE("[C2 arm_max_q7] value and index are written to exactly one element each for block sizes 1, 2, 3, 4, 5 and 9 and are correct") {
	for (int m = 0; m < 4; m++) {
		for (uint32_t n : kSizes) {
			V x(n);
			for (uint32_t i = 0; i < n; i++) x[i] = static_cast<int8_t>(10 + 3 * static_cast<int>(i % 5));   // increasing then repeating: no -51 (poison)
			x[n / 2] = (m == MIN || m == ABSMIN) ? static_cast<int8_t>(-100 + (m == ABSMIN ? 99 : 0)) : static_cast<int8_t>(100);
			const V before = x;
			const Ext e = defExt(static_cast<Mode>(m), x);
			dsp::GuardedBuffer<q7_t> val(1);
			dsp::GuardedBuffer<uint32_t> idx(1);
			kIdx[m](x.data(), n, val.data(), idx.data());
			INFO(kName[m] << " " << dump(before));
			CHECK(val.output()[0] == e.value);
			CHECK(idx.output()[0] == e.index);
			CHECK_MESSAGE(val.guardIntact(), "value guard damaged at " << val.firstDamaged());
			CHECK_MESSAGE(idx.guardIntact(), "index guard damaged at " << idx.firstDamaged());
			CHECK(x == before);
			dsp::GuardedBuffer<q7_t> val2(1);
			kNoIdx[m](x.data(), n, val2.data());
			CHECK(val2.output()[0] == e.value);
			CHECK_MESSAGE(val2.guardIntact(), "no_idx value guard damaged at " << val2.firstDamaged());
			CHECK(x == before);
		}
	}
}

TEST_CASE("[C2 arm_mean_q7] the mean of 1, 2 and 3 samples is written to exactly one element and the input is untouched") {
	for (uint32_t n : {1u, 2u, 3u}) {
		V x(n);
		for (uint32_t i = 0; i < n; i++) x[i] = static_cast<int8_t>(30 + 10 * static_cast<int>(i));
		const V before = x;
		dsp::GuardedBuffer<q7_t> out(1);
		arm_mean_q7(x.data(), n, out.data());
		INFO(dump(before));
		CHECK(meanOk(before, out.output()[0]));
		CHECK_MESSAGE(out.guardIntact(), "guard damaged at " << out.firstDamaged());
		CHECK(x == before);
	}
}

TEST_CASE("[C2 arm_mean_q7] the mean of 4, 5 and 9 samples is written to exactly one element and the input is untouched" * KNOWN_ISSUE_UB_HIDDEN(UB_MEAN)) {
	for (uint32_t n : {4u, 5u, 9u}) {
		V x(n);
		for (uint32_t i = 0; i < n; i++) x[i] = static_cast<int8_t>(30 + 10 * static_cast<int>(i));
		const V before = x;
		dsp::GuardedBuffer<q7_t> out(1);
		arm_mean_q7(x.data(), n, out.data());
		INFO(dump(before));
		CHECK(meanOk(before, out.output()[0]));
		CHECK_MESSAGE(out.guardIntact(), "guard damaged at " << out.firstDamaged());
		CHECK(x == before);
	}
}

TEST_CASE("[C2 arm_power_q7] the power is written to exactly one 32 bit word for block sizes 1, 2, 3, 4, 5 and 9 and the input is untouched") {
	for (uint32_t n : kSizes) {
		V x(n);
		for (uint32_t i = 0; i < n; i++) x[i] = static_cast<int8_t>(20 + 7 * static_cast<int>(i));
		const V before = x;
		dsp::GuardedBuffer<q31_t> out(1);
		arm_power_q7(x.data(), n, out.data());
		INFO(dump(before));
		CHECK(out.output()[0] == defPower(before));
		CHECK_MESSAGE(out.guardIntact(), "guard damaged at " << out.firstDamaged());
		CHECK(x == before);
	}
}

TEST_CASE("[C2 arm_mse_q7] the mse is written to exactly one element for block sizes 1, 2, 3, 4, 5 and 9 and neither input is changed") {
	for (uint32_t n : kSizes) {
		V a(n), b(n);
		for (uint32_t i = 0; i < n; i++) {
			a[i] = static_cast<int8_t>(40 + 2 * static_cast<int>(i));
			b[i] = static_cast<int8_t>(10 + 2 * static_cast<int>(i));
		}
		const V ba = a, bb = b;
		dsp::GuardedBuffer<q7_t> out(1);
		arm_mse_q7(a.data(), b.data(), n, out.data());
		// a - b = 30 for every sample: D = 15, mean D^2 = 225, 225 / 32 = 7.03 -> exact 7 (even inputs, see the header)
		CHECK(out.output()[0] == 7);
		CHECK_MESSAGE(out.guardIntact(), "guard damaged at " << out.firstDamaged());
		CHECK(a == ba);
		CHECK(b == bb);
	}
}

// ===============================================================================================================
// C3: max / min / absmax / absmin (and no_idx) against DEF
// ===============================================================================================================

static void sweepSingle(Mode m) {
	Fails f;
	for (int v = -128; v <= 127; v++) {
		V x(1, static_cast<int8_t>(v));
		checkExt(m, x, f);
		q7_t r = 0x55; uint32_t idx = 77;
		kIdx[m](x.data(), 1, &r, &idx);
		if (idx != 0) f.add(std::string(kName[m]) + ": index of the single sample " + std::to_string(v) + " is " + std::to_string(idx));
	}
	CHECK_NOFAIL(f);
}
static void sweepPairs(Mode m) {
	Fails f;
	for (int a = -128; a <= 127; a++)
		for (int b = -128; b <= 127; b++) {
			V x(2);
			x[0] = static_cast<int8_t>(a);
			x[1] = static_cast<int8_t>(b);
			checkExt(m, x, f);
		}
	CHECK_NOFAIL(f);
}
// A background that cannot tie with the planted value, the planted extremum at every position of blocks of 1..40 samples
static void sweepPlanted(Mode m) {
	Fails f;
	Rng r(1000u + static_cast<uint32_t>(m));
	std::vector<size_t> sizes;
	for (size_t n = 1; n <= 40; n++) sizes.push_back(n);
	sizes.push_back(1000);
	for (size_t n : sizes) {
		std::vector<size_t> positions;
		if (n <= 40) for (size_t p = 0; p < n; p++) positions.push_back(p);
		else positions = {0, 1, 254, 255, 256, 257, 511, 998, 999};
		for (size_t p : positions) {
			V x(n);
			for (size_t i = 0; i < n; i++) {
				int v;
				switch (m) {
				case MAX: v = r.range(-128, 10); break;
				case MIN: v = r.range(-10, 127); break;
				case ABSMAX: v = r.range(-10, 10); break;
				default: { const int mag = r.range(20, 100); v = (r.next() >> 31) ? -mag : mag; }
				}
				x[i] = static_cast<int8_t>(v);
			}
			int ext;
			switch (m) {
			case MAX: ext = 100; break;
			case MIN: ext = -100; break;
			case ABSMAX: ext = (p % 4 == 0) ? -128 : ((p % 2) ? 100 : -100); break;   // incl. the saturating -128
			default: ext = (p % 2) ? 3 : -3; break;
			}
			x[p] = static_cast<int8_t>(ext);
			const Ext e = defExt(m, x);
			if (!e.unique || e.index != p) { f.add("test data not unique"); continue; }
			checkExt(m, x, f);
		}
	}
	CHECK_NOFAIL(f);
}
static void sweepRandom(Mode m) {
	Fails f;
	Rng r(2000u + static_cast<uint32_t>(m));
	for (size_t n = 1; n <= 40; n++)
		for (int k = 0; k < 300; k++) checkExt(m, randV(r, n, -128, 127), f);
	CHECK_NOFAIL(f);
}

TEST_CASE("[C3 arm_max_q7] max of every single sample -128..127 is the sample and its index is 0 (arm_max_no_idx_q7 too)") { sweepSingle(MAX); }
TEST_CASE("[C3 arm_min_q7] min of every single sample -128..127 is the sample and its index is 0 (arm_min_no_idx_q7 too)") { sweepSingle(MIN); }
TEST_CASE("[C3 arm_absmax_q7] absmax of every single sample -128..127 is its saturated absolute value and its index is 0 (arm_absmax_no_idx_q7 too)") { sweepSingle(ABSMAX); }
TEST_CASE("[C3 arm_absmin_q7] absmin of every single sample -128..127 is its saturated absolute value and its index is 0 (arm_absmin_no_idx_q7 too)") { sweepSingle(ABSMIN); }

TEST_CASE("[C3 arm_max_q7] max, index (unique case) of all 65536 pairs match the definition (arm_max_no_idx_q7 too)") { sweepPairs(MAX); }
TEST_CASE("[C3 arm_min_q7] min, index (unique case) of all 65536 pairs match the definition (arm_min_no_idx_q7 too)") { sweepPairs(MIN); }
TEST_CASE("[C3 arm_absmax_q7] absmax, index (unique case) of all 65536 pairs match the definition (arm_absmax_no_idx_q7 too)") { sweepPairs(ABSMAX); }
TEST_CASE("[C3 arm_absmin_q7] absmin, index (unique case) of all 65536 pairs match the definition (arm_absmin_no_idx_q7 too)") { sweepPairs(ABSMIN); }

TEST_CASE("[C3 arm_max_q7] a unique maximum at every position of blocks of 1..40 and 1000 samples is found with the right index (arm_max_no_idx_q7 too)") { sweepPlanted(MAX); }
TEST_CASE("[C3 arm_min_q7] a unique minimum at every position of blocks of 1..40 and 1000 samples is found with the right index (arm_min_no_idx_q7 too)") { sweepPlanted(MIN); }
TEST_CASE("[C3 arm_absmax_q7] a unique absolute maximum (incl. -128) at every position of blocks of 1..40 and 1000 samples is found with the right index (arm_absmax_no_idx_q7 too)") { sweepPlanted(ABSMAX); }
TEST_CASE("[C3 arm_absmin_q7] a unique absolute minimum at every position of blocks of 1..40 and 1000 samples is found with the right index (arm_absmin_no_idx_q7 too)") { sweepPlanted(ABSMIN); }

TEST_CASE("[C3 arm_max_q7] max of 12000 random blocks of 1..40 samples matches the definition (arm_max_no_idx_q7 too)") { sweepRandom(MAX); }
TEST_CASE("[C3 arm_min_q7] min of 12000 random blocks of 1..40 samples matches the definition (arm_min_no_idx_q7 too)") { sweepRandom(MIN); }
TEST_CASE("[C3 arm_absmax_q7] absmax of 12000 random blocks of 1..40 samples matches the definition (arm_absmax_no_idx_q7 too)") { sweepRandom(ABSMAX); }
TEST_CASE("[C3 arm_absmin_q7] absmin of 12000 random blocks of 1..40 samples matches the definition (arm_absmin_no_idx_q7 too)") { sweepRandom(ABSMIN); }

// ---------------------------------------------------------------------------------------------------------------
// C4: all-equal data, saturation, ties
// ---------------------------------------------------------------------------------------------------------------

static void allEqual(Mode m) {
	Fails f;
	for (int v = -128; v <= 127; v++)
		for (size_t n : {2u, 3u, 4u, 5u, 9u}) {
			V x(n, static_cast<int8_t>(v));
			checkExt(m, x, f);   // all keys tie: value exact, index only has to point at an element
		}
	CHECK_NOFAIL(f);
}
TEST_CASE("[C4 arm_max_q7] max of a block of equal samples of every value is that value (arm_max_no_idx_q7 too)") { allEqual(MAX); }
TEST_CASE("[C4 arm_min_q7] min of a block of equal samples of every value is that value (arm_min_no_idx_q7 too)") { allEqual(MIN); }
TEST_CASE("[C4 arm_absmax_q7] absmax of a block of equal samples of every value is its saturated absolute value (arm_absmax_no_idx_q7 too)") { allEqual(ABSMAX); }
TEST_CASE("[C4 arm_absmin_q7] absmin of a block of equal samples of every value is its saturated absolute value (arm_absmin_no_idx_q7 too)") { allEqual(ABSMIN); }

TEST_CASE("[C4 arm_absmax_q7] absmax of -128 saturates to 127 instead of wrapping to -128 (all four abs functions)") {
	// DEF: |-128| = 128 is not representable in q7, ARM's reference pattern AbsMaxVals8 (saturation id) expects 127
	const V x = {-128, 5, -7, 1, 0};
	q7_t r = 0; uint32_t idx = 9;
	arm_absmax_q7(x.data(), 5, &r, &idx);
	CHECK(r == 127);
	CHECK(idx == 0);
	r = 0;
	arm_absmax_no_idx_q7(x.data(), 5, &r);
	CHECK(r == 127);
	const V y = {-128, -128, -128, -128, -128};
	arm_absmin_q7(y.data(), 5, &r, &idx);
	CHECK(r == 127);
	r = 0;
	arm_absmin_no_idx_q7(y.data(), 5, &r);
	CHECK(r == 127);
}

TEST_CASE("[C4 arm_max_q7] characterization: with equal maxima the first occurrence wins (positions 1 and 8 of 9 samples: second unrolled group, last lane)") {
	// DOC silent; source compares with strict '<' ; numpy argmax (ARM's generator) also returns the first
	const V x = {1, 9, 3, 4, 5, 6, 7, 8, 9};   // n = 9, maxima at 1 and 8
	q7_t r = 0; uint32_t idx = 77;
	arm_max_q7(x.data(), 9, &r, &idx);
	CHECK(r == 9);
	CHECK(idx == 1);
}
TEST_CASE("[C4 arm_min_q7] characterization: with equal minima the first occurrence wins (positions 1 and 8 of 9 samples: second unrolled group, last lane)") {
	const V x = {5, -9, 3, 4, 5, 6, 7, 8, -9};
	q7_t r = 0; uint32_t idx = 77;
	arm_min_q7(x.data(), 9, &r, &idx);
	CHECK(r == -9);
	CHECK(idx == 1);
}
TEST_CASE("[C4 arm_absmax_q7] characterization: with equal absolute maxima (even of opposite sign) the first occurrence wins") {
	const V x = {1, -50, 3, 4, 5, 6, 7, 8, 50};
	q7_t r = 0; uint32_t idx = 77;
	arm_absmax_q7(x.data(), 9, &r, &idx);
	CHECK(r == 50);
	CHECK(idx == 1);
	const V y = {1, 127, 3, 4, 5, 6, 7, 8, -128};   // 127 and the saturated -128 tie
	arm_absmax_q7(y.data(), 9, &r, &idx);
	CHECK(r == 127);
	CHECK(idx == 1);
}
TEST_CASE("[C4 arm_absmin_q7] characterization: with equal absolute minima (even of opposite sign) the first occurrence wins") {
	const V x = {90, -2, 90, 90, 90, 90, 90, 90, 2};
	q7_t r = 0; uint32_t idx = 77;
	arm_absmin_q7(x.data(), 9, &r, &idx);
	CHECK(r == 2);
	CHECK(idx == 1);
}

// Ties: DOC silent; the library uses strict comparisons, so the FIRST of two equal extrema wins. All pairs of positions (p < q) of blocks
// of 2..13 samples (covers every lane of the unroll by 4 and the tail); the background is strictly worse than the extremum.
static void tieTest(Mode m) {
	const bool isMax = (m == MAX || m == ABSMAX), absolute = (m == ABSMAX || m == ABSMIN);
	Fails f;
	for (size_t n = 2; n <= 13; n++)
		for (size_t p = 0; p + 1 < n; p++)
			for (size_t q = p + 1; q < n; q++) {
				V x(n, static_cast<int8_t>(isMax ? 1 : 9));
				const int ext = isMax ? 9 : 1;
				x[p] = static_cast<int8_t>(ext);
				x[q] = static_cast<int8_t>(absolute ? -ext : ext);
				q7_t r = 0x55; uint32_t idx = 0xFFFFFFFFu;
				kIdx[m](x.data(), static_cast<uint32_t>(n), &r, &idx);
				if (r != ext || idx != p) f.add(dump(x) + " first " + std::to_string(p) + " second " + std::to_string(q) + " -> value " + std::to_string(static_cast<int>(r)) + " index " + std::to_string(idx));
			}
	CHECK_NOFAIL(f);
}
TEST_CASE("[C4 arm_max_q7] characterization: of two equal maxima at any positions p < q of blocks of 2..13 samples the first index is returned") { tieTest(MAX); }
TEST_CASE("[C4 arm_min_q7] characterization: of two equal minima at any positions p < q of blocks of 2..13 samples the first index is returned") { tieTest(MIN); }
TEST_CASE("[C4 arm_absmax_q7] characterization: of two equal maxima of |x| (opposite sign) at any positions p < q of blocks of 2..13 samples the first index is returned") { tieTest(ABSMAX); }
TEST_CASE("[C4 arm_absmin_q7] characterization: of two equal minima of |x| (opposite sign) at any positions p < q of blocks of 2..13 samples the first index is returned") { tieTest(ABSMIN); }

// ===============================================================================================================
// C3 / C4: mean
// ===============================================================================================================

TEST_CASE("[C3 arm_mean_q7] the mean of every single sample -128..127 is the sample") {
	for (int v = -128; v <= 127; v++) {
		const V x(1, static_cast<int8_t>(v));
		q7_t r = 0x55;
		arm_mean_q7(x.data(), 1, &r);
		INFO("sample " << v);
		CHECK(r == v);
	}
}

TEST_CASE("[C3 arm_mean_q7] the mean of all 65536 pairs is the sum over 2 truncated (exact for non-negative sums, within one LSB otherwise)") {
	Fails f;
	for (int a = -128; a <= 127; a++)
		for (int b = -128; b <= 127; b++) {
			const V x = {static_cast<int8_t>(a), static_cast<int8_t>(b)};
			q7_t r = 0x55;
			arm_mean_q7(x.data(), 2, &r);
			if (!meanOk(x, r)) f.add(dump(x) + " -> " + std::to_string(static_cast<int>(r)));
		}
	CHECK_NOFAIL(f);
}

TEST_CASE("[C3 arm_mean_q7] the mean of 3000 random blocks of 1, 2 and 3 samples is the sum over n truncated") {
	Rng r(3001);
	Fails f;
	for (size_t n = 1; n <= 3; n++)
		for (int k = 0; k < 1000; k++) {
			const V x = randV(r, n, -128, 127);
			q7_t out = 0x55;
			arm_mean_q7(x.data(), static_cast<uint32_t>(n), &out);
			if (!meanOk(x, out)) f.add(dump(x) + " -> " + std::to_string(static_cast<int>(out)));
		}
	CHECK_NOFAIL(f);
}

TEST_CASE("[C3 arm_mean_q7] the mean of 14000 random blocks of 4..40 samples is the sum over n truncated" * KNOWN_ISSUE_UB_HIDDEN(UB_MEAN)) {
	Rng r(3002);
	Fails f;
	for (size_t n = 4; n <= 40; n++)
		for (int k = 0; k < 400; k++) {
			const V x = randV(r, n, -128, 127);
			q7_t out = 0x55;
			arm_mean_q7(x.data(), static_cast<uint32_t>(n), &out);
			if (!meanOk(x, out)) f.add(dump(x) + " -> " + std::to_string(static_cast<int>(out)));
		}
	CHECK_NOFAIL(f);
}

TEST_CASE("[C3 arm_mean_q7] the mean of non-negative samples of blocks of 4..40 is exactly the floor of sum / n" * KNOWN_ISSUE_UB_HIDDEN(UB_MEAN)) {
	Rng r(3003);
	Fails f;
	for (size_t n = 4; n <= 40; n++)
		for (int k = 0; k < 100; k++) {
			const V x = randV(r, n, 0, 127);
			q7_t out = 0x55;
			arm_mean_q7(x.data(), static_cast<uint32_t>(n), &out);
			if (out != sumOf(x) / static_cast<long>(n)) f.add(dump(x) + " -> " + std::to_string(static_cast<int>(out)));
		}
	CHECK_NOFAIL(f);
}

TEST_CASE("[C4 arm_mean_q7] the mean of equal samples of every value in blocks of 1000 and 65536 samples is that value (no accumulator overflow)" * KNOWN_ISSUE_UB_HIDDEN(UB_MEAN)) {
	// DOC: 25.7 accumulator, no risk of overflow. 65536 * -128 = -2^23, far inside 32 bit.
	Fails f;
	for (int v = -128; v <= 127; v++)
		for (size_t n : {1000u, 65536u}) {
			const V x(n, static_cast<int8_t>(v));
			q7_t out = 0x55;
			arm_mean_q7(x.data(), static_cast<uint32_t>(n), &out);
			if (out != v) f.add("value " + std::to_string(v) + " n " + std::to_string(n) + " -> " + std::to_string(static_cast<int>(out)));
		}
	CHECK_NOFAIL(f);
}

TEST_CASE("[C4 arm_mean_q7] the mean of alternating -128 and 127 over 1001 samples is within one LSB of the true mean -628/1001" * KNOWN_ISSUE_UB_HIDDEN(UB_MEAN)) {
	// 501 samples of -128 and 500 of 127: sum = -64128 + 63500 = -628, mean -0.627
	V x(1001);
	for (size_t i = 0; i < x.size(); i++) x[i] = (i % 2 == 0) ? int8_t(-128) : int8_t(127);
	REQUIRE(sumOf(x) == -628);
	q7_t out = 0x55;
	arm_mean_q7(x.data(), 1001, &out);
	CHECK(meanOk(x, out));
}

TEST_CASE("[C4 arm_mean_q7] the mean of 16777216 samples of -128 is -128 and of 127 is 127 (exact 32 bit accumulator boundary 2^24 * 128 = 2^31)" * KNOWN_ISSUE_UB_HIDDEN(UB_MEAN)) {
	// sum -2^31 and 127 * 2^24 = 2130706432 both fit; one more sample of -128 (n = 16777217) wraps (see the header, NEEDS REVIEW)
	V x(16777216, static_cast<int8_t>(-128));
	q7_t out = 0x55;
	arm_mean_q7(x.data(), 16777216u, &out);
	CHECK(out == -128);
	std::fill(x.begin(), x.end(), static_cast<int8_t>(127));
	arm_mean_q7(x.data(), 16777216u, &out);
	CHECK(out == 127);
}

TEST_CASE("[C4 arm_mean_q7] characterization: a negative non-integer mean is truncated toward zero") {
	// DOC "truncated" is silent about the direction; the source divides with C '/' (toward zero). {-3, 0}: -1.5 -> -1; {-1, -1, 0}: -0.667 -> 0
	q7_t r = 0x55;
	const V a = {-3, 0};
	arm_mean_q7(a.data(), 2, &r);
	CHECK(r == -1);
	const V b = {-1, -1, 0};
	arm_mean_q7(b.data(), 3, &r);
	CHECK(r == 0);
}

// ===============================================================================================================
// C3 / C4: power
// ===============================================================================================================

TEST_CASE("[C3 arm_power_q7] the power of every single sample -128..127 is the square of the sample") {
	// hand check of the format: 64 (0.5) -> 0.25 in 18.14 = 4096 = 64^2
	for (int v = -128; v <= 127; v++) {
		const V x(1, static_cast<int8_t>(v));
		q31_t r = 0x55555555;
		arm_power_q7(x.data(), 1, &r);
		INFO("sample " << v);
		CHECK(r == v * v);
	}
}

TEST_CASE("[C3 arm_power_q7] the power of all 65536 pairs is the exact sum of the two squares") {
	Fails f;
	for (int a = -128; a <= 127; a++)
		for (int b = -128; b <= 127; b++) {
			const V x = {static_cast<int8_t>(a), static_cast<int8_t>(b)};
			q31_t r = 0x55555555;
			arm_power_q7(x.data(), 2, &r);
			if (r != a * a + b * b) f.add(dump(x) + " -> " + std::to_string(r));
		}
	CHECK_NOFAIL(f);
}

TEST_CASE("[C3 arm_power_q7] the power of 12000 random blocks of 1..40 samples is the exact sum of squares") {
	Rng r(4001);
	Fails f;
	for (size_t n = 1; n <= 40; n++)
		for (int k = 0; k < 300; k++) {
			const V x = randV(r, n, -128, 127);
			q31_t out = 0x55555555;
			arm_power_q7(x.data(), static_cast<uint32_t>(n), &out);
			if (out != defPower(x)) f.add(dump(x) + " -> " + std::to_string(out));
		}
	CHECK_NOFAIL(f);
}

TEST_CASE("[C4 arm_power_q7] the power of 131071 samples of -128 is exactly 131071 * 16384 (17 guard bits of the accumulator, no overflow)") {
	// DOC: "17 guard bits in the accumulator, there is no risk of overflow": 2^31 / 2^14 = 131072 samples, one fewer fits: 2147467264
	const V x(131071, static_cast<int8_t>(-128));
	q31_t r = 0;
	arm_power_q7(x.data(), 131071, &r);
	CHECK(r == 2147467264);
}

TEST_CASE("[C4 arm_power_q7] the power of {127, -128} is 16129 + 16384 (hand derivation)") {
	const V x = {127, -128};
	q31_t r = 0;
	arm_power_q7(x.data(), 2, &r);
	CHECK(r == 32513);
}

TEST_CASE("[C4 arm_power_q7] characterization: the power of an empty block (blockSize 0) is 0") {
	// DOC silent; an empty sum is 0 and the source reads no sample. No sample is read; a one element buffer is passed anyway.
	const V x(1, 55);
	q31_t r = 0x55555555;
	arm_power_q7(x.data(), 0, &r);
	CHECK(r == 0);
}

// ===============================================================================================================
// C3 / C4: mse
// ===============================================================================================================

TEST_CASE("[C3 arm_mse_q7] the mse of all 65536 single-sample pairs is within the derived bound of mean((a-b)^2) / 128") {
	Fails f;
	for (int a = -128; a <= 127; a++)
		for (int b = -128; b <= 127; b++) {
			const V x(1, static_cast<int8_t>(a)), y(1, static_cast<int8_t>(b));
			q7_t r = 0x55;
			arm_mse_q7(x.data(), y.data(), 1, &r);
			const std::string why = mseBound(x, y, r, defMseT(x, y));
			if (!why.empty()) f.add(why);
		}
	CHECK_NOFAIL(f);
}

TEST_CASE("[C3 arm_mse_q7] the mse of 12000 random block pairs of 1..40 samples is within the derived bound of mean((a-b)^2) / 128") {
	Rng r(5001);
	Fails f;
	for (size_t n = 1; n <= 40; n++)
		for (int k = 0; k < 300; k++) {
			const V a = randV(r, n, -128, 127);
			const V b = randV(r, n, -128, 127);
			q7_t out = 0x55;
			arm_mse_q7(a.data(), b.data(), static_cast<uint32_t>(n), &out);
			const std::string why = mseBound(a, b, out, defMseT(a, b));
			if (!why.empty()) f.add(why);
		}
	CHECK_NOFAIL(f);
}

TEST_CASE("[C3 arm_mse_q7] the mse of block pairs with small differences (|a-b| <= 40) of 1..40 samples is within the derived bound") {
	// the saturation at 127 hides the bound for large differences; small differences test the scale itself
	Rng r(5002);
	Fails f;
	for (size_t n = 1; n <= 40; n++)
		for (int k = 0; k < 300; k++) {
			const V a = randV(r, n, -80, 80);
			V b(n);
			for (size_t i = 0; i < n; i++) b[i] = static_cast<int8_t>(a[i] + r.range(-40, 40));
			q7_t out = 0x55;
			arm_mse_q7(a.data(), b.data(), static_cast<uint32_t>(n), &out);
			const std::string why = mseBound(a, b, out, defMseT(a, b));
			if (!why.empty()) f.add(why);
		}
	CHECK_NOFAIL(f);
}

TEST_CASE("[C4 arm_mse_q7] identical blocks give an mse of exactly 0") {
	Rng r(5003);
	for (size_t n = 1; n <= 40; n++) {
		const V a = randV(r, n, -128, 127);
		q7_t out = 0x55;
		arm_mse_q7(a.data(), a.data(), static_cast<uint32_t>(n), &out);
		INFO(dump(a));
		CHECK(out == 0);
	}
}

TEST_CASE("[C4 arm_mse_q7] characterization: with all samples even the mse is exactly floor(sum((a-b)/2)^2 / (32 n)) clamped to 127 (all 128 x 128 even pairs)") {
	// a = 2x, b = 2y: halves exact, D = x - y, result = floor(D^2 / 32) (n = 1). The truncation of the result is not documented.
	Fails f;
	for (int a = -128; a <= 126; a += 2)
		for (int b = -128; b <= 126; b += 2) {
			const V x(1, static_cast<int8_t>(a)), y(1, static_cast<int8_t>(b));
			q7_t r = 0x55;
			arm_mse_q7(x.data(), y.data(), 1, &r);
			const int D = (a - b) / 2;
			const int expected = std::min(127, (D * D) / 32);
			if (r != expected) f.add("a " + std::to_string(a) + " b " + std::to_string(b) + " -> " + std::to_string(static_cast<int>(r)) + " expected " + std::to_string(expected));
		}
	CHECK_NOFAIL(f);
}

TEST_CASE("[C4 arm_mse_q7] characterization: with all samples even the mse of blocks of 1..40 samples is exactly floor(sum D^2 / (32 n)) clamped to 127") {
	Rng r(5004);
	Fails f;
	for (size_t n = 1; n <= 40; n++)
		for (int k = 0; k < 300; k++) {
			V a(n), b(n);
			for (size_t i = 0; i < n; i++) {
				a[i] = static_cast<int8_t>(2 * r.range(-64, 63));
				b[i] = static_cast<int8_t>(a[i] - 2 * r.range(-20, 20));   // small differences: results below the saturation
			}
			long S = 0;
			for (size_t i = 0; i < n; i++) { const long D = (a[i] - b[i]) / 2; S += D * D; }
			const long expected = std::min(127L, S / (32L * static_cast<long>(n)));
			q7_t out = 0x55;
			arm_mse_q7(a.data(), b.data(), static_cast<uint32_t>(n), &out);
			if (out != expected) f.add(dump(a) + " / " + dump(b) + " -> " + std::to_string(static_cast<int>(out)) + " expected " + std::to_string(expected));
		}
	CHECK_NOFAIL(f);
}

TEST_CASE("[C4 arm_mse_q7] the mse saturates at 127 when the true mean square error (up to 4.0) exceeds the q7 range (blocks of 1..9 samples)") {
	// 127 vs -128: mean((a-b)^2)/128 = 255^2/128 = 508 >> 127; and -128 vs 127; and 100 vs -100: 200^2/128 = 312
	for (size_t n = 1; n <= 9; n++) {
		const V a(n, 127), b(n, -128);
		q7_t r = 0;
		arm_mse_q7(a.data(), b.data(), static_cast<uint32_t>(n), &r);
		INFO("n " << n);
		CHECK(r == 127);
		arm_mse_q7(b.data(), a.data(), static_cast<uint32_t>(n), &r);
		CHECK(r == 127);
		const V c(n, 100), d(n, -100);
		arm_mse_q7(c.data(), d.data(), static_cast<uint32_t>(n), &r);
		CHECK(r == 127);
	}
}

// ===============================================================================================================
// C5: q7 against the f32 library on x / 128
// ===============================================================================================================

static std::vector<float> toF32(const V &x) {
	std::vector<float> f(x.size());
	for (size_t i = 0; i < x.size(); i++) f[i] = static_cast<float>(x[i]) / 128.0f;   // exact
	return f;
}

static void c5Ext(Mode m) {
	Fails f;
	Rng r(6000u + static_cast<uint32_t>(m));
	for (size_t n = 1; n <= 40; n++)
		for (int k = 0; k < 100; k++) {
			// -128 is excluded for the abs functions: |-128| is 1.0 in f32 but saturates in q7 (the keys would differ)
			const bool abs = (m == ABSMAX || m == ABSMIN);
			const V x = randV(r, n, abs ? -127 : -128, 127);
			const std::vector<float> xf = toF32(x);
			float fv = 0; uint32_t fi = 99;
			kIdxF32[m](xf.data(), static_cast<uint32_t>(n), &fv, &fi);
			q7_t qv = 0; uint32_t qi = 99;
			kIdx[m](x.data(), static_cast<uint32_t>(n), &qv, &qi);
			const Ext e = defExt(m, x);
			if (qv != static_cast<int>(fv * 128.0f)) f.add(dump(x) + ": value " + std::to_string(static_cast<int>(qv)) + " vs f32 " + std::to_string(fv * 128.0f));
			else if (e.unique && qi != fi) f.add(dump(x) + ": index " + std::to_string(qi) + " vs f32 " + std::to_string(fi));
		}
	CHECK_NOFAIL(f);
}
TEST_CASE("[C5 arm_max_q7] value (times 128) and unique index agree with arm_max_f32 on x/128") { c5Ext(MAX); }
TEST_CASE("[C5 arm_min_q7] value (times 128) and unique index agree with arm_min_f32 on x/128") { c5Ext(MIN); }
TEST_CASE("[C5 arm_absmax_q7] value (times 128) and unique index agree with arm_absmax_f32 on x/128 (without -128)") { c5Ext(ABSMAX); }
TEST_CASE("[C5 arm_absmin_q7] value (times 128) and unique index agree with arm_absmin_f32 on x/128 (without -128)") { c5Ext(ABSMIN); }

TEST_CASE("[C5 arm_mean_q7] the mean agrees with arm_mean_f32 on x/128 within one LSB (truncation)" * KNOWN_ISSUE_UB_HIDDEN(UB_MEAN)) {
	Rng r(6100);
	Fails f;
	for (size_t n = 1; n <= 40; n++)
		for (int k = 0; k < 100; k++) {
			const V x = randV(r, n, -128, 127);
			const std::vector<float> xf = toF32(x);
			float fm = 0;
			arm_mean_f32(xf.data(), static_cast<uint32_t>(n), &fm);
			q7_t qm = 0x55;
			arm_mean_q7(x.data(), static_cast<uint32_t>(n), &qm);
			// truncation error < 1 LSB; the f32 mean carries ~1e-5 relative error (sum of exact dyadic values, one division)
			if (std::fabs(static_cast<double>(qm) - static_cast<double>(fm) * 128.0) >= 1.0 + 1e-3) f.add(dump(x) + ": " + std::to_string(static_cast<int>(qm)) + " vs " + std::to_string(fm * 128.0f));
		}
	CHECK_NOFAIL(f);
}

TEST_CASE("[C5 arm_power_q7] the power equals the arm_power_f32 result on x/128 times 2^14 exactly (n <= 64)") {
	// terms k^2 / 2^14 <= 1 and partial sums < 64 are multiples of 2^-14 below 2^6: exact in binary32
	Rng r(6200);
	Fails f;
	for (size_t n = 1; n <= 64; n++)
		for (int k = 0; k < 50; k++) {
			const V x = randV(r, n, -128, 127);
			const std::vector<float> xf = toF32(x);
			float fp = 0;
			arm_power_f32(xf.data(), static_cast<uint32_t>(n), &fp);
			q31_t qp = 0;
			arm_power_q7(x.data(), static_cast<uint32_t>(n), &qp);
			if (static_cast<double>(qp) != static_cast<double>(fp) * 16384.0) f.add(dump(x) + ": " + std::to_string(qp) + " vs " + std::to_string(fp * 16384.0f));
		}
	CHECK_NOFAIL(f);
}

TEST_CASE("[C5 arm_mse_q7] the mse agrees with 128 * arm_mse_f32 on x/128 within the derived bound") {
	Rng r(6300);
	Fails f;
	for (size_t n = 1; n <= 40; n++)
		for (int k = 0; k < 100; k++) {
			const V a = randV(r, n, -128, 127);
			V b(n);
			for (size_t i = 0; i < n; i++) b[i] = static_cast<int8_t>(std::max(-128, std::min(127, a[i] + r.range(-60, 60))));
			const std::vector<float> af = toF32(a), bf = toF32(b);
			float fm = 0;
			arm_mse_f32(af.data(), bf.data(), static_cast<uint32_t>(n), &fm);
			q7_t qm = 0x55;
			arm_mse_q7(a.data(), b.data(), static_cast<uint32_t>(n), &qm);
			const std::string why = mseBound(a, b, qm, static_cast<double>(fm) * 128.0);
			if (!why.empty()) f.add(why);
		}
	CHECK_NOFAIL(f);
}

}  // TEST_SUITE
