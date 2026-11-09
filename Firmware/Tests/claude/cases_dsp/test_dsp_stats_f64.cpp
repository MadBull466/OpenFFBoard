/*
 * Statistics f64 (work order SF64, requirements C1-C6 of Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md).
 *
 * Units covered (Source/StatisticsFunctions/, all 16 f64 files that exist): arm_max_f64, arm_min_f64, arm_absmax_f64,
 * arm_absmin_f64 (value + index), arm_max_no_idx_f64, arm_min_no_idx_f64, arm_absmax_no_idx_f64, arm_absmin_no_idx_f64,
 * arm_accumulate_f64, arm_mean_f64, arm_power_f64, arm_var_f64, arm_std_f64, arm_mse_f64, arm_entropy_f64,
 * arm_kullback_leibler_f64.
 * NOT existing for f64: arm_rms_f64, arm_logsumexp_f64 and arm_logsumexp_dot_prod_f64 (there is no source file and
 * statistics_functions.h declares none). ARM's suite lists their ids as disabled{...} (desc.txt:84-86 test_rms_f64,
 * desc.txt:96 and :98 logsumexp, :102 long rms) and their test bodies are commented out in StatsTestsF64.cpp
 * (rms :244-266, logsumexp :318-334, logsumexp_dot_prod :355-374), so there is nothing to run for them.
 *
 * Pattern files (Testing/Patterns/DSP/Stats/StatsF64/, suite "Statistics Tests F64", desc.txt:8-140): Input1 (max, min,
 * power, std, var, min/max_no_idx), Input2 (mean), InputNew1 / InputNew2 (absmax, absmin, mse), MaxVals1 / MaxIndexes1,
 * MinVals3 / MinIndexes3, MeanVals2, PowerVals4, StdVals6, VarVals7, AbsMaxVals26 / AbsMaxIndexes26,
 * AbsMinVals27 / AbsMinIndexes27, MSEVals28, Input22 / Dims22 / RefEntropy22, InputA24 / InputB24 / Dims24 / RefKL24,
 * InputAccumulate1 / InputAccumulateConfig1 / RefAccumulate1.
 * ARM ids (desc.txt:60-120): nb = 2, 2n (4 samples), 2n+1 (5 samples) with refOffset 0, 1, 2 for max, mean, min, power,
 * std, var, max_no_idx, absmax, absmin, min_no_idx, absmax_no_idx, absmin_no_idx, mse; "long" (100 samples, refOffset 3)
 * for mean, std, var, mse; test_std_stability_f64 (the stability case); entropy, kullback_leibler and accumulate run
 * all patterns of the dims file. The setUp bodies are StatsTestsF64.cpp:480-1290.
 * ARM tolerances (StatsTestsF64.cpp:7-14, Error.h:170-176, Error.cpp:198-216, :857-871): index and min/max values:
 * ASSERT_EQ (exact, the index through an int16_t cast); mean, power, std, var, mse, entropy, KL, accumulate:
 * ASSERT_SNR >= 300 dB and ASSERT_REL_ERROR <= 4.0e-15 (|a-b| / ((|a|+|b|)/2), skipped when the mean is 0); the stability
 * case |5.47723 - std| < 1.0e-4. Used unchanged (L4); the per-sample check is stricter than ARM only in that NaN fails.
 *
 * C2: result (and index) in GuardedBuffers (the guard check is a check of what the LIBRARY writes behind them); the input
 * of the DEF tests is followed by poison values (1e300, -1e300, 0, 0) so a read beyond blockSize changes max, min, abs
 * max, abs min and every sum. C3: DEF in double inside the test (std::max_element / min_element, Neumaier sum, std::log).
 * Strided sweeps over the whole exponent range of binary64 (denormals to DBL_MAX) for the 8 max/min units.
 *
 * Bounds (all derived, none looser than needed; u = 2^-53 = 1.11e-16):
 *  - exact tests: values that are multiples of 2^-20 (|x| <= 1), so every partial sum of up to 33 terms is exact in a
 *    53 bit mantissa (|sum| < 2^6, resolution 2^-20: 26 bits). accumulate, mean (one correctly rounded division), power
 *    (squares are multiples of 2^-40, sum 2^-40 .. 2^6: 46 bits) and mse (differences exact, same argument; one
 *    correctly rounded division) must therefore equal the integer computation EXACTLY. var with n a power of two: the
 *    mean is exact and so are the deviations, but the squares (multiples of 2^-2(20+log2 n), up to 4) can need up to 54
 *    bits, so the sum may round: that test allows n u relative (the integer DEF converted once, the library's sequence of
 *    roundings; the bound n u is generous for the worst case chain of n-1 additions). std is sqrt(var) of the library: equality with std::sqrt.
 *  - general data: the library sums sequentially, in any order the error of a sum of n terms is <= (n-1) u sum|x|; the
 *    DEF (Neumaier compensated sum) has an error of about 2 u |sum|, so |lib - DEF| <= (n+1) u sum|x| for accumulate
 *    (mean: divided by n, plus the rounding of the division and of the DEF: (n+2) u sum|x| / n). power / mse: every term is positive, one
 *    rounding per product (mse also one for the difference), so the relative bound is (n+4) u (n+4 absorbs the
 *    compensated DEF). var: first order effect of the rounding error of the mean on sum((x-m)^2) is 2 d sum(x-m) = 0,
 *    the second order term n d^2 is < 1e-20 relative here, so (n+4) u relative as well; std: sqrt halves the relative
 *    error and adds u, so the same bound is conservative. entropy: all terms -p ln p share their sign, bound (n+4) u
 *    sum|p ln p| (the log is the same libm call on both sides). KL: (n+4) u sum|p_A ln(p_B/p_A)| (the sign of the terms is
 *    mixed; the ratio is the same division on both sides).
 *  - C5 (f64 against f32 of the same input, inputs exactly representable in float): the f32 result has an error
 *    <= (n+4) u32 sum|.| with u32 = 2^-24 = 5.96e-8 (6.0e-8 is used); the f64 result error is negligible (1e-16).
 *    entropy f32: logf <= 1 ulp, terms same sign, (n+6) u32 sum|p ln p|. KL f32: per term the quotient rounds
 *    (abs error u32 in the log), logf adds 2 u32 |ln q|, the product u32: u32 (2 + 3|ln q|) pA, plus (n) u32 sum|t| for the sum.
 *
 * Host vs target (C6): the host runs the portable C path (no ARM_MATH_DSP, no MVE, no NEON), where the f64 functions are
 * plain sequential loops (arm_mse_f64 is unrolled by 2 with the same order); on the target (Cortex-M4F, soft double
 * from newlib) the same C code runs, no FMA contraction exists for soft double, no float to integer conversion, no signed
 * shift. The library log() is the host glibc one here and newlib on target (both <= 1 ulp, the last bit can differ), which
 * is why the entropy / KL bounds are in ulp-style relative terms and not bit exact. The exact (dyadic) tests do not
 * depend on the summation order. On an ARM_MATH_NEON build (aarch64) mean / accumulate / mse sum in pairs: the
 * general-data bounds above hold for every order.
 *
 * Suspected vendor defects: arm_max_no_idx_f64 starts from -DBL_MAX and arm_min_no_idx_f64 from +DBL_MAX, so a vector that
 * only holds -inf (max) or +inf (min) returns -DBL_MAX / +DBL_MAX instead of -inf / +inf (KNOWN_ISSUE B-DSF64-1/-2).
 */
#include "doctest.h"

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace {

using IdxFn = void (*)(const float64_t *, uint32_t, float64_t *, uint32_t *);
using ScFn = void (*)(const float64_t *, uint32_t, float64_t *);
using Sc2Fn = void (*)(const float64_t *, const float64_t *, uint32_t, float64_t *);
using IdxFn32 = void (*)(const float32_t *, uint32_t, float32_t *, uint32_t *);
using ScFn32 = void (*)(const float32_t *, uint32_t, float32_t *);
using Sc2Fn32 = void (*)(const float32_t *, const float32_t *, uint32_t, float32_t *);

constexpr double kSnr = 300.0;       // StatsTestsF64.cpp:7
constexpr double kRel = 4.0e-15;     // StatsTestsF64.cpp:14
constexpr double kU = 1.12e-16;      // 2^-53 rounded up
constexpr double kU32 = 6.0e-8;      // 2^-24 rounded up
const double kInf = std::numeric_limits<double>::infinity();

std::vector<double> pat(const char *name) {
	return dsp::loadPattern(std::string("Stats/StatsF64/") + name).f64();
}
std::vector<int16_t> patS16(const char *name) {
	return dsp::loadPattern(std::string("Stats/StatsF64/") + name).q15();
}

// ARM's Client::assert_relative_error (Error.cpp:198-216); NaN fails here (ARM would let it pass)
bool armRelOk(double a, double b, double thr) {
	if (std::isnan(a) || std::isnan(b)) return false;
	const double average = (std::fabs(a) + std::fabs(b)) / 2.0;
	if (average == 0.0) return true;
	return std::fabs(a - b) / average <= thr;
}

// ARM's pair ASSERT_SNR + ASSERT_REL_ERROR for a scalar result
void armScalarCheck(double got, double ref) {
	CHECK(dsp::snr(std::vector<double>{ref}, std::vector<double>{got}) >= kSnr);
	CHECK_MESSAGE(armRelOk(got, ref, kRel), "got " << got << " ref " << ref);
}
// same for a vector of results
void armVectorCheck(const std::vector<double> &ref, const std::vector<double> &got) {
	REQUIRE(ref.size() == got.size());
	CHECK(dsp::snr(ref, got) >= kSnr);
	for (size_t i = 0; i < ref.size(); i++) CHECK_MESSAGE(armRelOk(got[i], ref[i], kRel), "sample " << i << " got " << got[i] << " ref " << ref[i]);
}

// ---- deterministic data -----------------------------------------------------------------------------------------
struct Rng {
	uint64_t s;
	explicit Rng(uint64_t seed) : s(seed) {}
	uint64_t next() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
	double uni() { const uint64_t r = next(); return static_cast<double>(r >> 11) * (2.0 / 9007199254740992.0) - 1.0; }   // [-1, 1)
	int range(int lo, int hi) { const uint64_t r = next(); return lo + static_cast<int>(r % static_cast<uint64_t>(hi - lo + 1)); }
};

std::vector<double> randomVec(Rng &g, size_t n, double scale) {
	std::vector<double> v;
	for (size_t i = 0; i < n; i++) { const double x = g.uni(); v.push_back(x * scale); }
	return v;
}
// multiples of 2^-20 in [-1, 1]; k holds the integers
std::vector<int64_t> dyadicInts(Rng &g, size_t n) {
	std::vector<int64_t> k;
	for (size_t i = 0; i < n; i++) { const int v = g.range(-(1 << 20), 1 << 20); k.push_back(v); }
	return k;
}
std::vector<double> dyadicVec(const std::vector<int64_t> &k) {
	std::vector<double> v;
	for (int64_t x : k) v.push_back(std::ldexp(static_cast<double>(x), -20));
	return v;
}
// poison behind the data: a read beyond blockSize changes max, min, absmax, absmin and any sum
std::vector<double> withTail(const std::vector<double> &v) {
	std::vector<double> w = v;
	w.push_back(1e300); w.push_back(-1e300); w.push_back(0.0); w.push_back(0.0);
	return w;
}
const double *ptr(const std::vector<double> &w) { return w.data(); }

double neumaier(const std::vector<double> &v) {
	double sum = 0.0, c = 0.0;
	for (double x : v) {
		const double t = sum + x;
		if (std::fabs(sum) >= std::fabs(x)) c += (sum - t) + x; else c += (x - t) + sum;
		sum = t;
	}
	return sum + c;
}
double sumAbs(const std::vector<double> &v) { double s = 0.0; for (double x : v) s += std::fabs(x); return s; }
double defMean(const std::vector<double> &v) { return neumaier(v) / static_cast<double>(v.size()); }
double defPower(const std::vector<double> &v) { std::vector<double> t; for (double x : v) t.push_back(x * x); return neumaier(t); }
double defVar(const std::vector<double> &v) {
	const double m = defMean(v);
	std::vector<double> t;
	for (double x : v) { const double d = x - m; t.push_back(d * d); }
	return neumaier(t) / static_cast<double>(v.size() - 1);
}
double defMse(const std::vector<double> &a, const std::vector<double> &b) {
	std::vector<double> t;
	for (size_t i = 0; i < a.size(); i++) { const double d = a[i] - b[i]; t.push_back(d * d); }
	return neumaier(t) / static_cast<double>(a.size());
}

// ---- units ------------------------------------------------------------------------------------------------------
struct IdxUnit { IdxFn fn; bool absolute; bool maximum; };
struct NoIdxUnit { ScFn fn; bool absolute; bool maximum; };
const IdxUnit kMax{arm_max_f64, false, true}, kMin{arm_min_f64, false, false};
const IdxUnit kAbsMax{arm_absmax_f64, true, true}, kAbsMin{arm_absmin_f64, true, false};
const NoIdxUnit kMaxN{arm_max_no_idx_f64, false, true}, kMinN{arm_min_no_idx_f64, false, false};
const NoIdxUnit kAbsMaxN{arm_absmax_no_idx_f64, true, true}, kAbsMinN{arm_absmin_no_idx_f64, true, false};

// DEF: std::max_element / min_element return the FIRST extreme element
void defIdx(const std::vector<double> &v, bool absolute, bool maximum, double &val, uint32_t &idx) {
	std::vector<double> w = v;
	if (absolute) for (double &x : w) x = std::fabs(x);
	const auto it = maximum ? std::max_element(w.begin(), w.end()) : std::min_element(w.begin(), w.end());
	val = *it;
	idx = static_cast<uint32_t>(it - w.begin());
}

void checkIdx(const IdxUnit &u, const std::vector<double> &v) {
	double wantV = 0.0; uint32_t wantI = 0;
	defIdx(v, u.absolute, u.maximum, wantV, wantI);
	const std::vector<double> in = withTail(v);
	dsp::GuardedBuffer<double> r(1);
	dsp::GuardedBuffer<uint32_t> ix(1);
	u.fn(ptr(in), static_cast<uint32_t>(v.size()), r.data(), ix.data());
	CAPTURE(v.size());
	CHECK(r.guardIntact());
	CHECK(ix.guardIntact());
	CHECK(r.output()[0] == wantV);
	CHECK(ix.output()[0] == wantI);
}
void checkNoIdx(const NoIdxUnit &u, const std::vector<double> &v) {
	double wantV = 0.0; uint32_t wantI = 0;
	defIdx(v, u.absolute, u.maximum, wantV, wantI);
	const std::vector<double> in = withTail(v);
	dsp::GuardedBuffer<double> r(1);
	u.fn(ptr(in), static_cast<uint32_t>(v.size()), r.data());
	CAPTURE(v.size());
	CHECK(r.guardIntact());
	CHECK(r.output()[0] == wantV);
}

// ---- C1 runners ---------------------------------------------------------------------------------------------------
void armIdx(IdxFn fn, const char *in, const char *vals, const char *inds, size_t nb, size_t off) {
	const std::vector<double> x = pat(in), rv = pat(vals);
	const std::vector<int16_t> ri = patS16(inds);
	REQUIRE(x.size() >= nb);
	REQUIRE(rv.size() > off);
	REQUIRE(ri.size() > off);
	double res = 0.0; uint32_t idx = 0;
	fn(x.data(), static_cast<uint32_t>(nb), &res, &idx);
	CHECK(res == rv[off]);                              // ASSERT_EQ(result, refp[refOffset])
	CHECK(static_cast<int16_t>(idx) == ri[off]);        // ASSERT_EQ((int16_t)indexval, refind[refOffset])
}
void armNoIdx(ScFn fn, const char *in, const char *vals, size_t nb, size_t off) {
	const std::vector<double> x = pat(in), rv = pat(vals);
	REQUIRE(x.size() >= nb);
	REQUIRE(rv.size() > off);
	double res = 0.0;
	fn(x.data(), static_cast<uint32_t>(nb), &res);
	CHECK(res == rv[off]);
}
void armScalar(ScFn fn, const char *in, const char *vals, size_t nb, size_t off) {
	const std::vector<double> x = pat(in), rv = pat(vals);
	REQUIRE(x.size() >= nb);
	REQUIRE(rv.size() > off);
	double res = 0.0;
	fn(x.data(), static_cast<uint32_t>(nb), &res);
	armScalarCheck(res, rv[off]);
}
void armMse(size_t nb, size_t off) {
	const std::vector<double> a = pat("InputNew1_f64.txt"), b = pat("InputNew2_f64.txt"), rv = pat("MSEVals28_f64.txt");
	REQUIRE(a.size() >= nb);
	REQUIRE(b.size() >= nb);
	REQUIRE(rv.size() > off);
	double res = 0.0;
	arm_mse_f64(a.data(), b.data(), static_cast<uint32_t>(nb), &res);
	armScalarCheck(res, rv[off]);
}

bool poisoned(double v) {
	unsigned char b[sizeof v];
	std::memcpy(b, &v, sizeof v);
	for (unsigned char c : b) if (c != 0xCD) return false;
	return true;
}
// ---- guard test of a scalar function -----------------------------------------------------------------------------
void guardScalar(ScFn fn) {
	const std::vector<double> in = {0.5, -0.25, 2.0, 1.0, -3.0, 0.125, 4.0, 0.75, -0.5};
	for (uint32_t n : {1u, 2u, 3u, 4u, 5u, 9u}) {
		dsp::GuardedBuffer<double> r(1);
		fn(in.data(), n, r.data());
		CAPTURE(n);
		CHECK(r.guardIntact());
		CHECK_FALSE(poisoned(r.output()[0]));   // the result was written (GuardedBuffer pre-fills 0xCD bytes)
	}
}
void guardIdx(IdxFn fn) {
	const std::vector<double> in = {0.5, -0.25, 2.0, 1.0, -3.0, 0.125, 4.0, 0.75, -0.5};
	for (uint32_t n : {1u, 2u, 3u, 4u, 5u, 9u}) {
		dsp::GuardedBuffer<double> r(1);
		dsp::GuardedBuffer<uint32_t> ix(1);
		fn(in.data(), n, r.data(), ix.data());
		CAPTURE(n);
		CHECK(r.guardIntact());
		CHECK(ix.guardIntact());
		CHECK(ix.output()[0] < n);
	}
}

// ---- sweep data over the whole exponent range ---------------------------------------------------------------------
// One value per binary exponent step of 3 from the smallest denormal (2^-1074) to 2^1023, mantissa 1 + (k % 7) / 8,
// alternating sign, plus DBL_MAX and 0: all magnitudes are different, so max / min have a unique answer. The order is
// shuffled with a fixed generator so the extreme lands on different positions of the 9 sample windows.
std::vector<double> sweepData() {
	std::vector<double> s;
	int k = 0;
	for (int e = -1074; e <= 1023; e += 3, k++) {
		const double m = 1.0 + static_cast<double>(k % 7) / 8.0;
		const double x = std::ldexp(m, e);
		if (std::isfinite(x) && x != 0.0) s.push_back((k % 2) ? -x : x);
	}
	s.push_back(DBL_MAX);
	s.push_back(0.0);
	Rng g(0x9E3779B97F4A7C15ULL);
	for (size_t i = s.size() - 1; i > 0; i--) {
		const uint64_t r = g.next();
		std::swap(s[i], s[static_cast<size_t>(r % (i + 1))]);
	}
	return s;
}

}  // namespace

TEST_SUITE("DSP Statistics f64") {

// ===============================================================================================================
// C1: ARM's test ids
// ===============================================================================================================
// ---- arm_max_f64 (test_max_f64, StatsTestsF64.cpp:16-40, ids TEST_MAX_F64_1..3)
TEST_CASE("[C1 arm_max_f64] nb=2 maximum and index match the reference (test_max_f64)") { armIdx(arm_max_f64, "Input1_f64.txt", "MaxVals1_f64.txt", "MaxIndexes1_s16.txt", 2, 0); }
TEST_CASE("[C1 arm_max_f64] nb=2n (4) maximum and index match the reference (test_max_f64)") { armIdx(arm_max_f64, "Input1_f64.txt", "MaxVals1_f64.txt", "MaxIndexes1_s16.txt", 4, 1); }
TEST_CASE("[C1 arm_max_f64] nb=2n+1 (5) maximum and index match the reference (test_max_f64)") { armIdx(arm_max_f64, "Input1_f64.txt", "MaxVals1_f64.txt", "MaxIndexes1_s16.txt", 5, 2); }
// ---- arm_min_f64
TEST_CASE("[C1 arm_min_f64] nb=2 minimum and index match the reference (test_min_f64)") { armIdx(arm_min_f64, "Input1_f64.txt", "MinVals3_f64.txt", "MinIndexes3_s16.txt", 2, 0); }
TEST_CASE("[C1 arm_min_f64] nb=2n (4) minimum and index match the reference (test_min_f64)") { armIdx(arm_min_f64, "Input1_f64.txt", "MinVals3_f64.txt", "MinIndexes3_s16.txt", 4, 1); }
TEST_CASE("[C1 arm_min_f64] nb=2n+1 (5) minimum and index match the reference (test_min_f64)") { armIdx(arm_min_f64, "Input1_f64.txt", "MinVals3_f64.txt", "MinIndexes3_s16.txt", 5, 2); }
// ---- arm_absmax_f64 (InputNew1)
TEST_CASE("[C1 arm_absmax_f64] nb=2 maximum of the absolute values and index match the reference (test_absmax_f64)") { armIdx(arm_absmax_f64, "InputNew1_f64.txt", "AbsMaxVals26_f64.txt", "AbsMaxIndexes26_s16.txt", 2, 0); }
TEST_CASE("[C1 arm_absmax_f64] nb=2n (4) maximum of the absolute values and index match the reference (test_absmax_f64)") { armIdx(arm_absmax_f64, "InputNew1_f64.txt", "AbsMaxVals26_f64.txt", "AbsMaxIndexes26_s16.txt", 4, 1); }
TEST_CASE("[C1 arm_absmax_f64] nb=2n+1 (5) maximum of the absolute values and index match the reference (test_absmax_f64)") { armIdx(arm_absmax_f64, "InputNew1_f64.txt", "AbsMaxVals26_f64.txt", "AbsMaxIndexes26_s16.txt", 5, 2); }
// ---- arm_absmin_f64
TEST_CASE("[C1 arm_absmin_f64] nb=2 minimum of the absolute values and index match the reference (test_absmin_f64)") { armIdx(arm_absmin_f64, "InputNew1_f64.txt", "AbsMinVals27_f64.txt", "AbsMinIndexes27_s16.txt", 2, 0); }
TEST_CASE("[C1 arm_absmin_f64] nb=2n (4) minimum of the absolute values and index match the reference (test_absmin_f64)") { armIdx(arm_absmin_f64, "InputNew1_f64.txt", "AbsMinVals27_f64.txt", "AbsMinIndexes27_s16.txt", 4, 1); }
TEST_CASE("[C1 arm_absmin_f64] nb=2n+1 (5) minimum of the absolute values and index match the reference (test_absmin_f64)") { armIdx(arm_absmin_f64, "InputNew1_f64.txt", "AbsMinVals27_f64.txt", "AbsMinIndexes27_s16.txt", 5, 2); }
// ---- no_idx variants
TEST_CASE("[C1 arm_max_no_idx_f64] nb=2 maximum matches the reference (test_max_no_idx_f64)") { armNoIdx(arm_max_no_idx_f64, "Input1_f64.txt", "MaxVals1_f64.txt", 2, 0); }
TEST_CASE("[C1 arm_max_no_idx_f64] nb=2n (4) maximum matches the reference (test_max_no_idx_f64)") { armNoIdx(arm_max_no_idx_f64, "Input1_f64.txt", "MaxVals1_f64.txt", 4, 1); }
TEST_CASE("[C1 arm_max_no_idx_f64] nb=2n+1 (5) maximum matches the reference (test_max_no_idx_f64)") { armNoIdx(arm_max_no_idx_f64, "Input1_f64.txt", "MaxVals1_f64.txt", 5, 2); }
TEST_CASE("[C1 arm_min_no_idx_f64] nb=2 minimum matches the reference (test_min_no_idx_f64)") { armNoIdx(arm_min_no_idx_f64, "Input1_f64.txt", "MinVals3_f64.txt", 2, 0); }
TEST_CASE("[C1 arm_min_no_idx_f64] nb=2n (4) minimum matches the reference (test_min_no_idx_f64)") { armNoIdx(arm_min_no_idx_f64, "Input1_f64.txt", "MinVals3_f64.txt", 4, 1); }
TEST_CASE("[C1 arm_min_no_idx_f64] nb=2n+1 (5) minimum matches the reference (test_min_no_idx_f64)") { armNoIdx(arm_min_no_idx_f64, "Input1_f64.txt", "MinVals3_f64.txt", 5, 2); }
TEST_CASE("[C1 arm_absmax_no_idx_f64] nb=2 maximum of the absolute values matches the reference (test_absmax_no_idx_f64)") { armNoIdx(arm_absmax_no_idx_f64, "InputNew1_f64.txt", "AbsMaxVals26_f64.txt", 2, 0); }
TEST_CASE("[C1 arm_absmax_no_idx_f64] nb=2n (4) maximum of the absolute values matches the reference (test_absmax_no_idx_f64)") { armNoIdx(arm_absmax_no_idx_f64, "InputNew1_f64.txt", "AbsMaxVals26_f64.txt", 4, 1); }
TEST_CASE("[C1 arm_absmax_no_idx_f64] nb=2n+1 (5) maximum of the absolute values matches the reference (test_absmax_no_idx_f64)") { armNoIdx(arm_absmax_no_idx_f64, "InputNew1_f64.txt", "AbsMaxVals26_f64.txt", 5, 2); }
TEST_CASE("[C1 arm_absmin_no_idx_f64] nb=2 minimum of the absolute values matches the reference (test_absmin_no_idx_f64)") { armNoIdx(arm_absmin_no_idx_f64, "InputNew1_f64.txt", "AbsMinVals27_f64.txt", 2, 0); }
TEST_CASE("[C1 arm_absmin_no_idx_f64] nb=2n (4) minimum of the absolute values matches the reference (test_absmin_no_idx_f64)") { armNoIdx(arm_absmin_no_idx_f64, "InputNew1_f64.txt", "AbsMinVals27_f64.txt", 4, 1); }
TEST_CASE("[C1 arm_absmin_no_idx_f64] nb=2n+1 (5) minimum of the absolute values matches the reference (test_absmin_no_idx_f64)") { armNoIdx(arm_absmin_no_idx_f64, "InputNew1_f64.txt", "AbsMinVals27_f64.txt", 5, 2); }
// ---- arm_mean_f64 (Input2, MeanVals2; ids 4..6 and long 29: 100 samples, refOffset 3)
TEST_CASE("[C1 arm_mean_f64] nb=2 mean matches the reference (test_mean_f64)") { armScalar(arm_mean_f64, "Input2_f64.txt", "MeanVals2_f64.txt", 2, 0); }
TEST_CASE("[C1 arm_mean_f64] nb=2n (4) mean matches the reference (test_mean_f64)") { armScalar(arm_mean_f64, "Input2_f64.txt", "MeanVals2_f64.txt", 4, 1); }
TEST_CASE("[C1 arm_mean_f64] nb=2n+1 (5) mean matches the reference (test_mean_f64)") { armScalar(arm_mean_f64, "Input2_f64.txt", "MeanVals2_f64.txt", 5, 2); }
TEST_CASE("[C1 arm_mean_f64] long (100 samples) mean matches the reference (test_mean_f64 long)") { armScalar(arm_mean_f64, "Input2_f64.txt", "MeanVals2_f64.txt", 100, 3); }
// ---- arm_power_f64
TEST_CASE("[C1 arm_power_f64] nb=2 power matches the reference (test_power_f64)") { armScalar(arm_power_f64, "Input1_f64.txt", "PowerVals4_f64.txt", 2, 0); }
TEST_CASE("[C1 arm_power_f64] nb=2n (4) power matches the reference (test_power_f64)") { armScalar(arm_power_f64, "Input1_f64.txt", "PowerVals4_f64.txt", 4, 1); }
TEST_CASE("[C1 arm_power_f64] nb=2n+1 (5) power matches the reference (test_power_f64)") { armScalar(arm_power_f64, "Input1_f64.txt", "PowerVals4_f64.txt", 5, 2); }
// ---- arm_std_f64
TEST_CASE("[C1 arm_std_f64] nb=2 standard deviation matches the reference (test_std_f64)") { armScalar(arm_std_f64, "Input1_f64.txt", "StdVals6_f64.txt", 2, 0); }
TEST_CASE("[C1 arm_std_f64] nb=2n (4) standard deviation matches the reference (test_std_f64)") { armScalar(arm_std_f64, "Input1_f64.txt", "StdVals6_f64.txt", 4, 1); }
TEST_CASE("[C1 arm_std_f64] nb=2n+1 (5) standard deviation matches the reference (test_std_f64)") { armScalar(arm_std_f64, "Input1_f64.txt", "StdVals6_f64.txt", 5, 2); }
TEST_CASE("[C1 arm_std_f64] long (100 samples) standard deviation matches the reference (test_std_f64 long)") { armScalar(arm_std_f64, "Input1_f64.txt", "StdVals6_f64.txt", 100, 3); }
// ---- arm_var_f64
TEST_CASE("[C1 arm_var_f64] nb=2 variance matches the reference (test_var_f64)") { armScalar(arm_var_f64, "Input1_f64.txt", "VarVals7_f64.txt", 2, 0); }
TEST_CASE("[C1 arm_var_f64] nb=2n (4) variance matches the reference (test_var_f64)") { armScalar(arm_var_f64, "Input1_f64.txt", "VarVals7_f64.txt", 4, 1); }
TEST_CASE("[C1 arm_var_f64] nb=2n+1 (5) variance matches the reference (test_var_f64)") { armScalar(arm_var_f64, "Input1_f64.txt", "VarVals7_f64.txt", 5, 2); }
TEST_CASE("[C1 arm_var_f64] long (100 samples) variance matches the reference (test_var_f64 long)") { armScalar(arm_var_f64, "Input1_f64.txt", "VarVals7_f64.txt", 100, 3); }
// ---- arm_std_f64 stability (StatsTestsF64.cpp:312-349, test_std_stability_f64)
TEST_CASE("[C1 arm_std_f64] standard deviation of 4 values with a large offset stays positive and close to 5.47723 (test_std_stability_f64)") {
	double in[4] = {4.0, 7.0, 13.0, 16.0};
	for (double &x : in) x += 3.0e4;
	double result = -1.0;
	arm_std_f64(in, 4, &result);
	CHECK(std::fabs(5.47723 - result) < 1.0e-4);   // ARM's bound
}
// ---- arm_mse_f64 (InputNew1 / InputNew2, MSEVals28; ids 49..52)
TEST_CASE("[C1 arm_mse_f64] nb=2 mean square error matches the reference (test_mse_f64)") { armMse(2, 0); }
TEST_CASE("[C1 arm_mse_f64] nb=2n (4) mean square error matches the reference (test_mse_f64)") { armMse(4, 1); }
TEST_CASE("[C1 arm_mse_f64] nb=2n+1 (5) mean square error matches the reference (test_mse_f64)") { armMse(5, 2); }
TEST_CASE("[C1 arm_mse_f64] long (100 samples) mean square error matches the reference (test_mse_f64 long)") { armMse(100, 3); }
// ---- arm_entropy_f64 (StatsTestsF64.cpp:351-370)
TEST_CASE("[C1 arm_entropy_f64] the entropy of every pattern of ARM's input matches the reference (test_entropy_f64)") {
	const std::vector<double> in = pat("Input22_f64.txt"), ref = pat("RefEntropy22_f64.txt");
	const std::vector<int16_t> dims = patS16("Dims22_s16.txt");
	REQUIRE(dims.size() > 0);
	REQUIRE(static_cast<size_t>(dims[0]) == ref.size());
	std::vector<double> out;
	size_t pos = 0;
	for (int i = 0; i < dims[0]; i++) {
		REQUIRE(static_cast<size_t>(dims[i + 1]) + pos <= in.size());
		out.push_back(arm_entropy_f64(in.data() + pos, static_cast<uint32_t>(dims[i + 1])));
		pos += static_cast<size_t>(dims[i + 1]);
	}
	armVectorCheck(ref, out);
}
// ---- arm_kullback_leibler_f64 (StatsTestsF64.cpp:392-412)
TEST_CASE("[C1 arm_kullback_leibler_f64] the divergence of every pattern pair of ARM's input matches the reference (test_kullback_leibler_f64)") {
	const std::vector<double> a = pat("InputA24_f64.txt"), b = pat("InputB24_f64.txt"), ref = pat("RefKL24_f64.txt");
	const std::vector<int16_t> dims = patS16("Dims24_s16.txt");
	REQUIRE(dims.size() > 0);
	REQUIRE(static_cast<size_t>(dims[0]) == ref.size());
	std::vector<double> out;
	size_t pos = 0;
	for (int i = 0; i < dims[0]; i++) {
		const size_t len = static_cast<size_t>(dims[i + 1]);
		REQUIRE(pos + len <= a.size());
		REQUIRE(pos + len <= b.size());
		out.push_back(arm_kullback_leibler_f64(a.data() + pos, b.data() + pos, static_cast<uint32_t>(len)));
		pos += len;
	}
	armVectorCheck(ref, out);
}
// ---- arm_accumulate_f64 (test_accumulate_f64 "combined", StatsTestsF64.cpp:437-460)
TEST_CASE("[C1 arm_accumulate_f64] the sum of every pattern of ARM's input matches the reference (test_accumulate_f64)") {
	const std::vector<double> in = pat("InputAccumulate1_f64.txt"), ref = pat("RefAccumulate1_f64.txt");
	const std::vector<int16_t> dims = patS16("InputAccumulateConfig1_s16.txt");
	REQUIRE(dims.size() > 0);
	REQUIRE(static_cast<size_t>(dims[0]) == ref.size());
	std::vector<double> out;
	for (int i = 0; i < dims[0]; i++) {
		const size_t len = static_cast<size_t>(dims[i + 1]);
		REQUIRE(len <= in.size());
		double r = 0.0;
		// ARM's test never advances its input pointer (StatsTestsF64.cpp:449-453): every pattern sums the FIRST len samples
		arm_accumulate_f64(in.data(), static_cast<uint32_t>(len), &r);
		out.push_back(r);
	}
	armVectorCheck(ref, out);
}

// ===============================================================================================================
// C2: what the functions write (guard words behind the result / index)
// ===============================================================================================================
TEST_CASE("[C2 arm_max_f64] writes exactly one result and one index for block sizes 1, 2, 3, 4, 5 and 9") { guardIdx(arm_max_f64); }
TEST_CASE("[C2 arm_min_f64] writes exactly one result and one index for block sizes 1, 2, 3, 4, 5 and 9") { guardIdx(arm_min_f64); }
TEST_CASE("[C2 arm_absmax_f64] writes exactly one result and one index for block sizes 1, 2, 3, 4, 5 and 9") { guardIdx(arm_absmax_f64); }
TEST_CASE("[C2 arm_absmin_f64] writes exactly one result and one index for block sizes 1, 2, 3, 4, 5 and 9") { guardIdx(arm_absmin_f64); }
TEST_CASE("[C2 arm_max_no_idx_f64] writes exactly one result for block sizes 1, 2, 3, 4, 5 and 9") { guardScalar(arm_max_no_idx_f64); }
TEST_CASE("[C2 arm_min_no_idx_f64] writes exactly one result for block sizes 1, 2, 3, 4, 5 and 9") { guardScalar(arm_min_no_idx_f64); }
TEST_CASE("[C2 arm_absmax_no_idx_f64] writes exactly one result for block sizes 1, 2, 3, 4, 5 and 9") { guardScalar(arm_absmax_no_idx_f64); }
TEST_CASE("[C2 arm_absmin_no_idx_f64] writes exactly one result for block sizes 1, 2, 3, 4, 5 and 9") { guardScalar(arm_absmin_no_idx_f64); }
TEST_CASE("[C2 arm_accumulate_f64] writes exactly one result for block sizes 1, 2, 3, 4, 5 and 9") { guardScalar(arm_accumulate_f64); }
TEST_CASE("[C2 arm_mean_f64] writes exactly one result for block sizes 1, 2, 3, 4, 5 and 9") { guardScalar(arm_mean_f64); }
TEST_CASE("[C2 arm_power_f64] writes exactly one result for block sizes 1, 2, 3, 4, 5 and 9") { guardScalar(arm_power_f64); }
TEST_CASE("[C2 arm_var_f64] writes exactly one result for block sizes 1, 2, 3, 4, 5 and 9") { guardScalar(arm_var_f64); }
TEST_CASE("[C2 arm_std_f64] writes exactly one result for block sizes 1, 2, 3, 4, 5 and 9") { guardScalar(arm_std_f64); }
TEST_CASE("[C2 arm_mse_f64] writes exactly one result for block sizes 1, 2, 3, 4, 5 and 9") {
	const std::vector<double> a = {0.5, -0.25, 2.0, 1.0, -3.0, 0.125, 4.0, 0.75, -0.5};
	const std::vector<double> b = {0.0, 0.25, 1.0, 1.5, -2.0, 0.375, 3.0, 0.25, 0.5};
	for (uint32_t n : {1u, 2u, 3u, 4u, 5u, 9u}) {
		dsp::GuardedBuffer<double> r(1);
		arm_mse_f64(a.data(), b.data(), n, r.data());
		CAPTURE(n);
		CHECK(r.guardIntact());
		CHECK_FALSE(poisoned(r.output()[0]));   // the result was written (GuardedBuffer pre-fills 0xCD bytes)
	}
}

// ===============================================================================================================
// C3: DEF (double) for max / min / absmax / absmin with and without index
// ===============================================================================================================
#define FFB_IDX_UNITS(MACRO) \
	MACRO("arm_max_f64", kMax) MACRO("arm_min_f64", kMin) MACRO("arm_absmax_f64", kAbsMax) MACRO("arm_absmin_f64", kAbsMin)
#define FFB_NOIDX_UNITS(MACRO) \
	MACRO("arm_max_no_idx_f64", kMaxN) MACRO("arm_min_no_idx_f64", kMinN) MACRO("arm_absmax_no_idx_f64", kAbsMaxN) MACRO("arm_absmin_no_idx_f64", kAbsMinN)

#define IDX_RANDOM(NAME, U) \
	TEST_CASE("[C3 " NAME "] random data of 1 to 33 samples gives the extreme value and its index (DEF), nothing beyond blockSize is read") { \
		Rng g(0x1234ABCDULL); \
		for (size_t n = 1; n <= 33; n++) { const std::vector<double> v = randomVec(g, n, 1000.0); checkIdx(U, v); } \
	}
FFB_IDX_UNITS(IDX_RANDOM)
#define NOIDX_RANDOM(NAME, U) \
	TEST_CASE("[C3 " NAME "] random data of 1 to 33 samples gives the extreme value (DEF), nothing beyond blockSize is read") { \
		Rng g(0x1234ABCDULL); \
		for (size_t n = 1; n <= 33; n++) { const std::vector<double> v = randomVec(g, n, 1000.0); checkNoIdx(U, v); } \
	}
FFB_NOIDX_UNITS(NOIDX_RANDOM)

#define IDX_POSITION(NAME, U) \
	TEST_CASE("[C3 " NAME "] a unique extreme at each position of blocks of 1 to 17 samples is found with the right index") { \
		/* base 0.5 + 0.01 i: positive, increasing; the extreme is 100 (max), -100 (min, absmax) or -0.0005 (absmin) */ \
		const double ext = (U).maximum ? ((U).absolute ? -100.0 : 100.0) : ((U).absolute ? -0.0005 : -100.0); \
		for (size_t n = 1; n <= 17; n++) for (size_t p = 0; p < n; p++) { \
			std::vector<double> v; \
			for (size_t i = 0; i < n; i++) v.push_back(0.5 + 0.01 * static_cast<double>(i)); \
			v[p] = ext; \
			CAPTURE(n); CAPTURE(p); \
			double r = 0.0; uint32_t ix = 99; \
			(U).fn(v.data(), static_cast<uint32_t>(n), &r, &ix); \
			CHECK(ix == p); \
			CHECK(r == ((U).absolute ? std::fabs(ext) : ext)); \
		} \
	}
FFB_IDX_UNITS(IDX_POSITION)

#define IDX_SWEEP(NAME, U) \
	TEST_CASE("[C3 " NAME "] a sweep over every third binary exponent from 2^-1074 to DBL_MAX in windows of 1 to 9 samples matches the DEF") { \
		const std::vector<double> s = sweepData(); \
		for (size_t w = 1; w <= 9; w++) for (size_t start = 0; start + w <= s.size(); start += w) { \
			const std::vector<double> v(s.begin() + static_cast<std::ptrdiff_t>(start), s.begin() + static_cast<std::ptrdiff_t>(start + w)); \
			checkIdx(U, v); \
		} \
	}
FFB_IDX_UNITS(IDX_SWEEP)
#define NOIDX_SWEEP(NAME, U) \
	TEST_CASE("[C3 " NAME "] a sweep over every third binary exponent from 2^-1074 to DBL_MAX in windows of 1 to 9 samples matches the DEF") { \
		const std::vector<double> s = sweepData(); \
		for (size_t w = 1; w <= 9; w++) for (size_t start = 0; start + w <= s.size(); start += w) { \
			const std::vector<double> v(s.begin() + static_cast<std::ptrdiff_t>(start), s.begin() + static_cast<std::ptrdiff_t>(start + w)); \
			checkNoIdx(U, v); \
		} \
	}
FFB_NOIDX_UNITS(NOIDX_SWEEP)

#define IDX_CONST(NAME, U) \
	TEST_CASE("[C3 " NAME "] constant data of every block size 1 to 33 gives the constant at index 0") { \
		for (double c : {3.5, -3.5, 0.0}) for (uint32_t n = 1; n <= 33; n++) { \
			const std::vector<double> v(n, c); \
			CAPTURE(n); CAPTURE(c); \
			double r = -1.0; uint32_t ix = 99; \
			(U).fn(v.data(), n, &r, &ix); \
			CHECK(r == ((U).absolute ? std::fabs(c) : c)); \
			CHECK(ix == 0u); \
		} \
	}
FFB_IDX_UNITS(IDX_CONST)

#define NOIDX_CONST(NAME, U) \
	TEST_CASE("[C3 " NAME "] constant data of every block size 1 to 33 gives the constant") { \
		for (double c : {3.5, -3.5, 0.0}) for (uint32_t n = 1; n <= 33; n++) { \
			const std::vector<double> v(n, c); \
			CAPTURE(n); CAPTURE(c); \
			double r = -1.0; \
			(U).fn(v.data(), n, &r); \
			CHECK(r == ((U).absolute ? std::fabs(c) : c)); \
		} \
	}
FFB_NOIDX_UNITS(NOIDX_CONST)

#define NOIDX_NEGATIVE(NAME, U) \
	TEST_CASE("[C3 " NAME "] data that is all negative gives the right extreme (the initial value is not mixed in)") { \
		const std::vector<double> v = {-5.0, -2.0, -9.0, -3.0, -7.0}; \
		double r = 0.0; \
		(U).fn(v.data(), 5, &r); \
		/* hand: max -2, min -9, absmax 9, absmin 2 */ \
		const double want = (U).absolute ? ((U).maximum ? 9.0 : 2.0) : ((U).maximum ? -2.0 : -9.0); \
		CHECK(r == want); \
	}
FFB_NOIDX_UNITS(NOIDX_NEGATIVE)

#define IDX_NEGATIVE(NAME, U) \
	TEST_CASE("[C3 " NAME "] data that is all negative gives the right extreme and index") { \
		const std::vector<double> v = {-5.0, -2.0, -9.0, -3.0, -7.0}; \
		double r = 0.0; uint32_t ix = 99; \
		(U).fn(v.data(), 5, &r, &ix); \
		/* hand: max -2 at 1, min -9 at 2, absmax 9 at 2, absmin 2 at 1 */ \
		const double want = (U).absolute ? ((U).maximum ? 9.0 : 2.0) : ((U).maximum ? -2.0 : -9.0); \
		const uint32_t wantI = (U).absolute ? ((U).maximum ? 2u : 1u) : ((U).maximum ? 1u : 2u); \
		CHECK(r == want); \
		CHECK(ix == wantI); \
	}
FFB_IDX_UNITS(IDX_NEGATIVE)

// Ties: DOC (statistics_functions.h and the file headers) says nothing about equal values. The code compares with a
// strict < / >, so the first of equal extremes is reported. Equal to std::max_element / min_element (first element).
#define IDX_TIES(NAME, U) \
	TEST_CASE("characterization: [C3 " NAME "] equal extreme values report the first index (DOC is silent about ties)") { \
		/* hand: max/min of {1,3,3,2}: 3 at 1; {3,1,1,2}: 1 at 1; abs ties: {-3,3} -> index 0, {3,-3} -> index 0 */ \
		{ const std::vector<double> v = {1.0, 3.0, 3.0, 2.0}; if ((U).maximum && !(U).absolute) { double r; uint32_t ix = 9; (U).fn(v.data(), 4, &r, &ix); CHECK(ix == 1u); } } \
		{ const std::vector<double> v = {3.0, 1.0, 1.0, 2.0}; if (!(U).maximum && !(U).absolute) { double r; uint32_t ix = 9; (U).fn(v.data(), 4, &r, &ix); CHECK(ix == 1u); } } \
		if ((U).absolute) for (const std::vector<double> &v : {std::vector<double>{-3.0, 3.0}, std::vector<double>{3.0, -3.0}}) { \
			const std::vector<double> w = {v[0], v[1]}; \
			double r; uint32_t ix = 9; \
			if ((U).maximum) { (U).fn(w.data(), 2, &r, &ix); CHECK(ix == 0u); CHECK(r == 3.0); } \
			else { (U).fn(w.data(), 2, &r, &ix); CHECK(ix == 0u); CHECK(r == 3.0); } \
		} \
		Rng g(0x77ULL); \
		for (size_t n = 1; n <= 33; n++) { \
			std::vector<double> v; \
			for (size_t i = 0; i < n; i++) { const int k = g.range(-3, 3); v.push_back(static_cast<double>(k)); } \
			checkIdx(U, v); \
		} \
	}
FFB_IDX_UNITS(IDX_TIES)

TEST_CASE("[C3 arm_max_f64 arm_min_f64 arm_absmax_f64 arm_absmin_f64] block sizes 1 and 2 with hand-derived results") {
	double r = 0.0; uint32_t ix = 9;
	const double a[1] = {-4.0};
	arm_max_f64(a, 1, &r, &ix); CHECK(r == -4.0); CHECK(ix == 0u);
	arm_min_f64(a, 1, &r, &ix); CHECK(r == -4.0); CHECK(ix == 0u);
	arm_absmax_f64(a, 1, &r, &ix); CHECK(r == 4.0); CHECK(ix == 0u);
	arm_absmin_f64(a, 1, &r, &ix); CHECK(r == 4.0); CHECK(ix == 0u);
	const double b[2] = {2.0, -3.0};   // max 2 at 0, min -3 at 1, absmax 3 at 1, absmin 2 at 0
	arm_max_f64(b, 2, &r, &ix); CHECK(r == 2.0); CHECK(ix == 0u);
	arm_min_f64(b, 2, &r, &ix); CHECK(r == -3.0); CHECK(ix == 1u);
	arm_absmax_f64(b, 2, &r, &ix); CHECK(r == 3.0); CHECK(ix == 1u);
	arm_absmin_f64(b, 2, &r, &ix); CHECK(r == 2.0); CHECK(ix == 0u);
	const double c[2] = {-3.0, 2.0};   // max 2 at 1, min -3 at 0, absmax 3 at 0, absmin 2 at 1
	arm_max_f64(c, 2, &r, &ix); CHECK(r == 2.0); CHECK(ix == 1u);
	arm_min_f64(c, 2, &r, &ix); CHECK(r == -3.0); CHECK(ix == 0u);
	arm_absmax_f64(c, 2, &r, &ix); CHECK(r == 3.0); CHECK(ix == 0u);
	arm_absmin_f64(c, 2, &r, &ix); CHECK(r == 2.0); CHECK(ix == 1u);
}
TEST_CASE("[C3 arm_max_no_idx_f64 arm_min_no_idx_f64 arm_absmax_no_idx_f64 arm_absmin_no_idx_f64] block sizes 1 and 2 with hand-derived results") {
	double r = 0.0;
	const double a[1] = {-4.0};
	arm_max_no_idx_f64(a, 1, &r); CHECK(r == -4.0);
	arm_min_no_idx_f64(a, 1, &r); CHECK(r == -4.0);
	arm_absmax_no_idx_f64(a, 1, &r); CHECK(r == 4.0);
	arm_absmin_no_idx_f64(a, 1, &r); CHECK(r == 4.0);
	const double b[2] = {2.0, -3.0};
	arm_max_no_idx_f64(b, 2, &r); CHECK(r == 2.0);
	arm_min_no_idx_f64(b, 2, &r); CHECK(r == -3.0);
	arm_absmax_no_idx_f64(b, 2, &r); CHECK(r == 3.0);
	arm_absmin_no_idx_f64(b, 2, &r); CHECK(r == 2.0);
}

// ===============================================================================================================
// C4: infinities and signed zero in the max / min family
// ===============================================================================================================
TEST_CASE("[C4 arm_max_f64 arm_min_f64 arm_absmax_f64 arm_absmin_f64] infinities are found as extremes with their index") {
	const double v[4] = {1.0, -kInf, 5.0, kInf};
	double r = 0.0; uint32_t ix = 9;
	arm_max_f64(v, 4, &r, &ix); CHECK(r == kInf); CHECK(ix == 3u);
	arm_min_f64(v, 4, &r, &ix); CHECK(r == -kInf); CHECK(ix == 1u);
	arm_absmax_f64(v, 4, &r, &ix); CHECK(r == kInf); CHECK(ix == 1u);   // |-inf| = |inf|, first one
	arm_absmin_f64(v, 4, &r, &ix); CHECK(r == 1.0); CHECK(ix == 0u);
}
TEST_CASE("[C4 arm_max_f64 arm_min_f64] a vector of only -inf (max) or +inf (min) returns that infinity with index 0") {
	const double lo[3] = {-kInf, -kInf, -kInf}, hi[3] = {kInf, kInf, kInf};
	double r = 0.0; uint32_t ix = 9;
	arm_max_f64(lo, 3, &r, &ix); CHECK(r == -kInf); CHECK(ix == 0u);
	arm_min_f64(hi, 3, &r, &ix); CHECK(r == kInf); CHECK(ix == 0u);
}
TEST_CASE("[C4 arm_absmin_f64 arm_absmin_no_idx_f64] the absolute minimum of -0.0 and +0.0 is +0.0 (sign bit cleared)") {
	const double v[2] = {-0.0, 0.0};
	double r = -1.0; uint32_t ix = 9;
	arm_absmin_f64(v, 2, &r, &ix); CHECK(r == 0.0); CHECK_FALSE(std::signbit(r)); CHECK(ix == 0u);
	r = -1.0;
	arm_absmin_no_idx_f64(v, 2, &r); CHECK(r == 0.0); CHECK_FALSE(std::signbit(r));
}
TEST_CASE("[C4 arm_max_no_idx_f64] a vector holding only -DBL_MAX returns -DBL_MAX") {
	const double v[3] = {-DBL_MAX, -DBL_MAX, -DBL_MAX};
	double r = 0.0;
	arm_max_no_idx_f64(v, 3, &r);
	CHECK(r == -DBL_MAX);
}
TEST_CASE("[C4 arm_min_no_idx_f64] a vector holding only DBL_MAX returns DBL_MAX") {
	const double v[3] = {DBL_MAX, DBL_MAX, DBL_MAX};
	double r = 0.0;
	arm_min_no_idx_f64(v, 3, &r);
	CHECK(r == DBL_MAX);
}
TEST_CASE("[C4 arm_max_no_idx_f64] the maximum of a vector holding only -inf is -inf" * KNOWN_ISSUE("B-DSF64-1: arm_max_no_idx_f64.c starts from F64_MIN = -DBL_MAX (not from the first sample, not from -inf), so {-inf, -inf} returns -DBL_MAX; arm_max_f64 and arm_absmax_no_idx_f64 start from the first sample and return -inf / inf. DOC: 'Maximum value of a floating-point vector'; DEF std::max_element = -inf")) {
	const double v[3] = {-kInf, -kInf, -kInf};
	double r = 0.0;
	arm_max_no_idx_f64(v, 3, &r);
	CHECK(r == -kInf);
}
TEST_CASE("[C4 arm_min_no_idx_f64] the minimum of a vector holding only +inf is +inf" * KNOWN_ISSUE("B-DSF64-2: arm_min_no_idx_f64.c starts from F64_MAX = DBL_MAX (not from the first sample), so {+inf, +inf} returns DBL_MAX; arm_min_f64 returns +inf. DOC: 'Maximum value of a floating-point vector' (copy-paste text, sic) for the minimum; DEF std::min_element = +inf")) {
	const double v[3] = {kInf, kInf, kInf};
	double r = 0.0;
	arm_min_no_idx_f64(v, 3, &r);
	CHECK(r == kInf);
}

// ===============================================================================================================
// C3 / C4: accumulate, mean, power, mse, var, std
// ===============================================================================================================
TEST_CASE("[C3 arm_accumulate_f64] sums of multiples of 2^-20 of 1 to 33 samples are exact (integer DEF)") {
	Rng g(0xACC001ULL);
	for (size_t n = 1; n <= 33; n++) {
		const std::vector<int64_t> k = dyadicInts(g, n);
		int64_t s = 0; for (int64_t x : k) s += x;
		const std::vector<double> in = withTail(dyadicVec(k));
		double r = 0.0;
		arm_accumulate_f64(ptr(in), static_cast<uint32_t>(n), &r);
		CAPTURE(n);
		CHECK(r == std::ldexp(static_cast<double>(s), -20));
	}
}
TEST_CASE("[C3 arm_mean_f64] means of multiples of 2^-20 of 1 to 33 samples are the exact sum divided once (integer DEF)") {
	Rng g(0xACC002ULL);
	for (size_t n = 1; n <= 33; n++) {
		const std::vector<int64_t> k = dyadicInts(g, n);
		int64_t s = 0; for (int64_t x : k) s += x;
		const std::vector<double> in = withTail(dyadicVec(k));
		double r = 0.0;
		arm_mean_f64(ptr(in), static_cast<uint32_t>(n), &r);
		CAPTURE(n);
		CHECK(r == std::ldexp(static_cast<double>(s), -20) / static_cast<double>(n));
	}
}
TEST_CASE("[C3 arm_power_f64] power of multiples of 2^-20 of 1 to 33 samples is the exact sum of squares (integer DEF)") {
	Rng g(0xACC003ULL);
	for (size_t n = 1; n <= 33; n++) {
		const std::vector<int64_t> k = dyadicInts(g, n);
		int64_t s = 0; for (int64_t x : k) s += x * x;     // < 33 * 2^40 fits
		const std::vector<double> in = withTail(dyadicVec(k));
		double r = 0.0;
		arm_power_f64(ptr(in), static_cast<uint32_t>(n), &r);
		CAPTURE(n);
		CHECK(r == std::ldexp(static_cast<double>(s), -40));
	}
}
TEST_CASE("[C3 arm_mse_f64] mean square error of multiples of 2^-20 of 1 to 33 samples is the exact sum divided once (integer DEF)") {
	Rng g(0xACC004ULL);
	for (size_t n = 1; n <= 33; n++) {
		const std::vector<int64_t> ka = dyadicInts(g, n);
		const std::vector<int64_t> kb = dyadicInts(g, n);
		int64_t s = 0; for (size_t i = 0; i < n; i++) { const int64_t d = ka[i] - kb[i]; s += d * d; }   // d < 2^21, d^2 < 2^42, 33 terms < 2^48
		const std::vector<double> a = withTail(dyadicVec(ka)), b = withTail(dyadicVec(kb));
		double r = 0.0;
		arm_mse_f64(ptr(a), ptr(b), static_cast<uint32_t>(n), &r);
		CAPTURE(n);
		CHECK(r == std::ldexp(static_cast<double>(s), -40) / static_cast<double>(n));
	}
}
TEST_CASE("[C3 arm_var_f64] variance of multiples of 2^-20 with n a power of two is exact (integer DEF, divisor n-1)") {
	Rng g(0xACC005ULL);
	for (size_t n : {2u, 4u, 8u, 16u, 32u}) {
		const std::vector<int64_t> k = dyadicInts(g, n);
		// deviations in integer units of 2^-(20+log2 n): d_i = n k_i - sum k (exact, |d_i| < 2^26, d_i^2 < 2^52,
		// the sum of up to 32 squares < 2^57 fits int64)
		int64_t s = 0; for (int64_t x : k) s += x;
		int64_t ss = 0; for (int64_t x : k) { const int64_t d = static_cast<int64_t>(n) * x - s; ss += d * d; }
		int lg = 0; while ((static_cast<size_t>(1) << lg) < n) lg++;
		const double want = std::ldexp(static_cast<double>(ss), -2 * (20 + lg)) / static_cast<double>(n - 1);
		const std::vector<double> in = withTail(dyadicVec(k));
		double r = 0.0;
		arm_var_f64(ptr(in), static_cast<uint32_t>(n), &r);
		CAPTURE(n);
		// 'want' is rounded once (ss > 2^53 possible) and the library sums n squares in double: n u relative (worst case of n-1 half-ulp roundings plus one for 'want')
		CHECK(std::fabs(r - want) <= static_cast<double>(n) * kU * want);
	}
}
TEST_CASE("[C3 arm_std_f64] standard deviation of multiples of 2^-20 with n a power of two is the square root of the variance") {
	Rng g(0xACC006ULL);
	for (size_t n : {2u, 4u, 8u, 16u, 32u}) {
		const std::vector<int64_t> k = dyadicInts(g, n);
		const std::vector<double> in = withTail(dyadicVec(k));
		double v = 0.0, s = 0.0;
		arm_var_f64(ptr(in), static_cast<uint32_t>(n), &v);
		arm_std_f64(ptr(in), static_cast<uint32_t>(n), &s);
		CAPTURE(n);
		CHECK(s == std::sqrt(v));        // arm_std = sqrt(arm_var): the same value, sqrt correctly rounded
	}
}

TEST_CASE("[C3 arm_accumulate_f64] random data of 1 to 33 samples is within (n+1) u sum|x| of the compensated DEF") {
	Rng g(0xBEEF01ULL);
	for (size_t n = 1; n <= 33; n++) {
		const std::vector<double> v = randomVec(g, n, 1e3);
		const std::vector<double> in = withTail(v);
		double r = 0.0;
		arm_accumulate_f64(ptr(in), static_cast<uint32_t>(n), &r);
		CAPTURE(n);
		CHECK(std::fabs(r - neumaier(v)) <= static_cast<double>(n + 1) * kU * sumAbs(v));
	}
}
TEST_CASE("[C3 arm_mean_f64] random data of 1 to 33 samples is within (n+2) u sum|x| / n of the compensated DEF") {
	Rng g(0xBEEF02ULL);
	for (size_t n = 1; n <= 33; n++) {
		const std::vector<double> v = randomVec(g, n, 1e3);
		const std::vector<double> in = withTail(v);
		double r = 0.0;
		arm_mean_f64(ptr(in), static_cast<uint32_t>(n), &r);
		CAPTURE(n);
		CHECK(std::fabs(r - defMean(v)) <= static_cast<double>(n + 2) * kU * sumAbs(v) / static_cast<double>(n));
	}
}
TEST_CASE("[C3 arm_power_f64] random data of 1 to 33 samples is within (n+4) u of the DEF, relative") {
	Rng g(0xBEEF03ULL);
	for (size_t n = 1; n <= 33; n++) {
		const std::vector<double> v = randomVec(g, n, 1e3);
		const std::vector<double> in = withTail(v);
		double r = 0.0;
		arm_power_f64(ptr(in), static_cast<uint32_t>(n), &r);
		CAPTURE(n);
		const double want = defPower(v);
		CHECK(std::fabs(r - want) <= static_cast<double>(n + 4) * kU * want);
	}
}
TEST_CASE("[C3 arm_mse_f64] random data of 1 to 33 samples is within (n+4) u of the DEF, relative") {
	Rng g(0xBEEF04ULL);
	for (size_t n = 1; n <= 33; n++) {
		const std::vector<double> a = randomVec(g, n, 1e3);
		const std::vector<double> b = randomVec(g, n, 1e3);
		const std::vector<double> ia = withTail(a), ib = withTail(b);
		double r = 0.0;
		arm_mse_f64(ptr(ia), ptr(ib), static_cast<uint32_t>(n), &r);
		CAPTURE(n);
		const double want = defMse(a, b);
		CHECK(std::fabs(r - want) <= static_cast<double>(n + 4) * kU * want);
	}
}
TEST_CASE("[C3 arm_var_f64] random data of 2 to 33 samples is within (n+4) u of the two-pass DEF, relative (divisor n-1)") {
	Rng g(0xBEEF05ULL);
	for (size_t n = 2; n <= 33; n++) {
		const std::vector<double> v = randomVec(g, n, 1e3);
		const std::vector<double> in = withTail(v);
		double r = 0.0;
		arm_var_f64(ptr(in), static_cast<uint32_t>(n), &r);
		CAPTURE(n);
		const double want = defVar(v);
		CHECK(std::fabs(r - want) <= static_cast<double>(n + 4) * kU * want);
	}
}
TEST_CASE("[C3 arm_std_f64] random data of 2 to 33 samples is within (n+4) u of the DEF, relative") {
	Rng g(0xBEEF06ULL);
	for (size_t n = 2; n <= 33; n++) {
		const std::vector<double> v = randomVec(g, n, 1e3);
		const std::vector<double> in = withTail(v);
		double r = 0.0;
		arm_std_f64(ptr(in), static_cast<uint32_t>(n), &r);
		CAPTURE(n);
		const double want = std::sqrt(defVar(v));
		CHECK(std::fabs(r - want) <= static_cast<double>(n + 4) * kU * want);
	}
}

TEST_CASE("[C3 arm_var_f64] the variance of 1, 2, 3, 4, 5 is 2.5 (divisor n-1, not n)") {
	// hand: mean 3, deviations -2 -1 0 1 2, squares sum 10, 10 / 4 = 2.5 (biased divisor n would give 2)
	const double v[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
	double r = 0.0;
	arm_var_f64(v, 5, &r);
	CHECK(r == 2.5);
}
TEST_CASE("[C3 arm_std_f64] the standard deviation of 1, 2, 3, 4, 5 is sqrt(2.5)") {
	const double v[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
	double r = 0.0;
	arm_std_f64(v, 5, &r);
	CHECK(std::fabs(r - 1.5811388300841898) <= 2.0 * kU * 1.5811388300841898);   // sqrt(2.5) = 1.5811388300841898
}
TEST_CASE("[C3 arm_var_f64 arm_std_f64] negative data: the variance of -1, -2, -3, -4, -5 is 2.5 as well and the standard deviation sqrt(2.5)") {
	const double v[5] = {-1.0, -2.0, -3.0, -4.0, -5.0};
	double var = 0.0, sd = 0.0;
	arm_var_f64(v, 5, &var);
	arm_std_f64(v, 5, &sd);
	CHECK(var == 2.5);
	CHECK(std::fabs(sd - 1.5811388300841898) <= 2.0 * kU * 1.5811388300841898);   // sqrt(2.5)
}
TEST_CASE("[C3 arm_var_f64] a large offset leaves the variance of 4, 7, 13, 16 at exactly 30 (two-pass algorithm)") {
	// hand: mean 10, deviations -6 -3 3 6, squares 36+9+9+36 = 90, 90 / 3 = 30. Offset 1e8: every sum is an integer < 2^53, exact.
	double v[4] = {4.0, 7.0, 13.0, 16.0};
	for (double &x : v) x += 1.0e8;
	double r = 0.0;
	arm_var_f64(v, 4, &r);
	CHECK(r == 30.0);
}
TEST_CASE("[C3 arm_mean_f64] the mean of 1, 2, 3, 4, 5 is 3 and of 0.5, 0.25, -1 is -0.25/3") {
	const double a[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
	const double b[3] = {0.5, 0.25, -1.0};     // sum -0.25 exact, divided by 3
	double r = 0.0;
	arm_mean_f64(a, 5, &r); CHECK(r == 3.0);
	arm_mean_f64(b, 3, &r); CHECK(r == -0.25 / 3.0);
}
TEST_CASE("[C3 arm_accumulate_f64] the sum of 0.5, 0.25, -1 is -0.25 and of one sample is the sample") {
	const double b[3] = {0.5, 0.25, -1.0};
	double r = 0.0;
	arm_accumulate_f64(b, 3, &r); CHECK(r == -0.25);
	arm_accumulate_f64(b + 2, 1, &r); CHECK(r == -1.0);
}
TEST_CASE("[C3 arm_power_f64] the power of 3, 4 is 25 and of 1, 2, 3 is 14, of negative values the same as of positive") {
	const double a[2] = {3.0, 4.0}, b[3] = {1.0, 2.0, 3.0}, c[3] = {-1.0, -2.0, -3.0};
	double r = 0.0;
	arm_power_f64(a, 2, &r); CHECK(r == 25.0);
	arm_power_f64(b, 3, &r); CHECK(r == 14.0);
	arm_power_f64(c, 3, &r); CHECK(r == 14.0);
}
TEST_CASE("[C3 arm_mse_f64] the mean square error of {1,2,3} and {2,2,5} is 5/3 and of identical vectors 0") {
	// hand: differences -1 0 -2, squares 1 0 4, sum 5, / 3
	const double a[3] = {1.0, 2.0, 3.0}, b[3] = {2.0, 2.0, 5.0};
	double r = -1.0;
	arm_mse_f64(a, b, 3, &r); CHECK(r == 5.0 / 3.0);
	arm_mse_f64(a, a, 3, &r); CHECK(r == 0.0);
}
TEST_CASE("[C3 arm_mean_f64 arm_accumulate_f64 arm_power_f64 arm_var_f64 arm_std_f64] constant data: sum, mean, power exact, variance and deviation zero") {
	// c = 0.5 and 3.0, n = 7: n * c is exact (3.5, 21), mean exact, deviations 0
	for (double c : {0.5, -0.5, 3.0}) {
		const std::vector<double> v(7, c);
		double sum = 0, mean = 0, pw = 0, var = -1, sd = -1;
		arm_accumulate_f64(v.data(), 7, &sum);
		arm_mean_f64(v.data(), 7, &mean);
		arm_power_f64(v.data(), 7, &pw);
		arm_var_f64(v.data(), 7, &var);
		arm_std_f64(v.data(), 7, &sd);
		CAPTURE(c);
		CHECK(sum == 7.0 * c);
		CHECK(mean == c);
		CHECK(pw == 7.0 * c * c);
		CHECK(var == 0.0);
		CHECK(sd == 0.0);
	}
}
TEST_CASE("[C3 arm_mean_f64] the mean of constant 0.1 (n = 1 to 33) is within (n+2) u sum|x| / n of 0.1") {
	for (uint32_t n = 1; n <= 33; n++) {
		const std::vector<double> v(n, 0.1);
		double r = 0.0;
		arm_mean_f64(v.data(), n, &r);
		CAPTURE(n);
		CHECK(std::fabs(r - 0.1) <= static_cast<double>(n + 2) * kU * 0.1);
	}
}

TEST_CASE("characterization: [C4 arm_var_f64] a block of one sample has variance 0 and a block of zero samples too (no division by n-1 = 0)") {
	// DOC is silent; the code has 'if (blockSize <= 1U) { *pResult = 0; return; }'. Well defined, not suspicious.
	const double v[1] = {42.0};
	double r = -1.0;
	arm_var_f64(v, 1, &r);
	CHECK(r == 0.0);
	r = -1.0;
	arm_var_f64(v, 0, &r);
	CHECK(r == 0.0);
}
TEST_CASE("characterization: [C4 arm_std_f64] a block of one sample has standard deviation 0 and a block of zero samples too") {
	const double v[1] = {42.0};
	double r = -1.0;
	arm_std_f64(v, 1, &r);
	CHECK(r == 0.0);
	r = -1.0;
	arm_std_f64(v, 0, &r);
	CHECK(r == 0.0);
}
TEST_CASE("[C4 arm_mean_f64 arm_accumulate_f64 arm_power_f64] block size 1 returns the sample, the sample, its square") {
	const double v[1] = {-7.5};
	double r = 0.0;
	arm_mean_f64(v, 1, &r); CHECK(r == -7.5);
	arm_accumulate_f64(v, 1, &r); CHECK(r == -7.5);
	arm_power_f64(v, 1, &r); CHECK(r == 56.25);
}
TEST_CASE("[C4 arm_mse_f64] block size 1 returns the square of the difference") {
	const double a[1] = {1.5}, b[1] = {-0.5};
	double r = 0.0;
	arm_mse_f64(a, b, 1, &r);
	CHECK(r == 4.0);
}
TEST_CASE("characterization: [C4 arm_power_f64 arm_accumulate_f64] results that exceed the range are +inf (IEEE overflow, no wrap)") {
	const double v[2] = {1e200, 1e200};
	double r = 0.0;
	arm_power_f64(v, 2, &r); CHECK(r == kInf);          // 1e400 does not fit
	const double w[2] = {DBL_MAX, DBL_MAX};
	arm_accumulate_f64(w, 2, &r); CHECK(r == kInf);
}
TEST_CASE("characterization: [C4 arm_power_f64] squares of tiny values underflow to 0 without error") {
	const double v[3] = {1e-200, -1e-200, 1e-160};   // squares 1e-400 (0), 1e-400 (0), 1e-320 (denormal, > 0)
	double r = -1.0;
	arm_power_f64(v, 3, &r);
	CHECK(r >= 0.0);
	CHECK(r < 1e-300);
}

// ===============================================================================================================
// C3 / C4: entropy and Kullback-Leibler
// ===============================================================================================================
TEST_CASE("[C3 arm_entropy_f64] the entropy of a uniform distribution of 4 and 8 outcomes is ln 4 and ln 8") {
	// hand: -n * (1/n) ln(1/n) = ln n; ln 4 = 1.3862943611198906, ln 8 = 2.0794415416798357
	const double u4[4] = {0.25, 0.25, 0.25, 0.25};
	const double u8[8] = {0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125};
	CHECK(std::fabs(arm_entropy_f64(u4, 4) - 1.3862943611198906) <= 4.0 * kU * 1.3862943611198906);
	CHECK(std::fabs(arm_entropy_f64(u8, 8) - 2.0794415416798357) <= 4.0 * kU * 2.0794415416798357);
}
TEST_CASE("[C4 arm_entropy_f64] a certain outcome (p = 1) has entropy 0; one sample 0.5 has entropy ln(2)/2") {
	const double one[1] = {1.0};
	CHECK(arm_entropy_f64(one, 1) == 0.0);
	const double half[1] = {0.5};      // hand: -0.5 ln 0.5 = ln 2 / 2 = 0.34657359027997264
	CHECK(std::fabs(arm_entropy_f64(half, 1) - 0.34657359027997264) <= 4.0 * kU * 0.34657359027997264);
}
TEST_CASE("[C3 arm_entropy_f64] random distributions of 1 to 33 outcomes are within (n+4) u sum|p ln p| of the DEF, nothing beyond blockSize is read") {
	Rng g(0xE17101ULL);
	for (size_t n = 1; n <= 33; n++) {
		std::vector<double> p;
		for (size_t i = 0; i < n; i++) { const double x = g.uni(); p.push_back(0.02 + 0.98 * std::fabs(x)); }   // (0.02, 1)
		double def = 0.0, bound = 0.0;
		for (double x : p) { def -= x * std::log(x); bound += std::fabs(x * std::log(x)); }
		bound *= static_cast<double>(n + 4) * kU;
		const std::vector<double> in = withTail(p);
		const double r = arm_entropy_f64(ptr(in), static_cast<uint32_t>(n));
		CAPTURE(n);
		CHECK(std::fabs(r - def) <= bound);
	}
}
TEST_CASE("[C3 arm_kullback_leibler_f64] D(A||B) of {0.5,0.5} and {0.25,0.75} is ln 2 - ln(3)/2") {
	// hand: D = -sum pA ln(pB/pA) = -(0.5 ln(0.25/0.5) + 0.5 ln(0.75/0.5)) = 0.5 ln 2 - 0.5 ln(1.5)
	//     = 0.5 (ln 2 - ln 3 + ln 2) = ln 2 - ln(3)/2 = 0.6931471805599453 - 0.5493061443340549 = 0.1438410362258904
	const double a[2] = {0.5, 0.5}, b[2] = {0.25, 0.75};
	CHECK(std::fabs(arm_kullback_leibler_f64(a, b, 2) - 0.1438410362258904) <= 4.0e-15);
}
TEST_CASE("[C4 arm_kullback_leibler_f64] the divergence of a distribution from itself is 0") {
	const double a[4] = {0.1, 0.2, 0.3, 0.4};
	CHECK(arm_kullback_leibler_f64(a, a, 4) == 0.0);       // log(pA/pA) = log(1) = 0 exactly
}
TEST_CASE("[C3 arm_kullback_leibler_f64] random distribution pairs of 1 to 33 outcomes are within (n+4) u sum|t| of the DEF and the sum of terms is not below -bound") {
	Rng g(0xE17102ULL);
	for (size_t n = 1; n <= 33; n++) {
		std::vector<double> a, b;
		for (size_t i = 0; i < n; i++) { const double x = g.uni(); a.push_back(0.02 + 0.98 * std::fabs(x)); }
		for (size_t i = 0; i < n; i++) { const double x = g.uni(); b.push_back(0.02 + 0.98 * std::fabs(x)); }
		double def = 0.0, bound = 0.0;
		for (size_t i = 0; i < n; i++) { const double t = a[i] * std::log(b[i] / a[i]); def -= t; bound += std::fabs(t); }
		bound *= static_cast<double>(n + 4) * kU;
		const std::vector<double> ia = withTail(a), ib = withTail(b);
		const double r = arm_kullback_leibler_f64(ptr(ia), ptr(ib), static_cast<uint32_t>(n));
		CAPTURE(n);
		CHECK(std::fabs(r - def) <= bound);
	}
}

TEST_CASE("characterization: [C4 arm_entropy_f64] a zero probability makes the entropy NaN (0 * log(0) = 0 * -inf), the convention 0 ln 0 = 0 is not applied") {
	// NEEDS REVIEW: DOC says only '-Sum(p ln p)'; mathematically lim p ln p = 0, so {0.5, 0.5, 0} would have entropy ln 2.
	const double p[3] = {0.5, 0.5, 0.0};
	CHECK(std::isnan(arm_entropy_f64(p, 3)));
}
TEST_CASE("characterization: [C4 arm_kullback_leibler_f64] a zero in A makes the divergence NaN (0 * log(pB / 0) = 0 * inf)") {
	// NEEDS REVIEW: DOC says only 'Divergence D(A || B)'; the convention 0 ln(0/q) = 0 is not applied.
	const double a[2] = {0.0, 1.0}, b[2] = {0.5, 1.0};
	CHECK(std::isnan(arm_kullback_leibler_f64(a, b, 2)));
}

// ===============================================================================================================
// C5: f64 against f32 of the same (float representable) input
// ===============================================================================================================
namespace {
struct FData { std::vector<float> f; std::vector<double> d; };
FData floatData(Rng &g, size_t n, double scale) {
	FData r;
	for (size_t i = 0; i < n; i++) { const double x = g.uni(); const float f = static_cast<float>(x * scale); r.f.push_back(f); r.d.push_back(static_cast<double>(f)); }
	return r;
}
void c5Idx(IdxFn fn64, IdxFn32 fn32, uint64_t seed) {
	Rng g(seed);
	for (size_t n = 1; n <= 33; n++) {
		const FData x = floatData(g, n, 100.0);
		double r64 = 0.0; float r32 = 0.0f; uint32_t i64 = 99, i32 = 98;
		fn64(x.d.data(), static_cast<uint32_t>(n), &r64, &i64);
		fn32(x.f.data(), static_cast<uint32_t>(n), &r32, &i32);
		CAPTURE(n);
		CHECK(r64 == static_cast<double>(r32));      // the float inputs are exact in double: max / min are exact in both
		CHECK(i64 == i32);
	}
}
void c5NoIdx(ScFn fn64, ScFn32 fn32, uint64_t seed) {
	Rng g(seed);
	for (size_t n = 1; n <= 33; n++) {
		const FData x = floatData(g, n, 100.0);
		double r64 = 0.0; float r32 = 0.0f;
		fn64(x.d.data(), static_cast<uint32_t>(n), &r64);
		fn32(x.f.data(), static_cast<uint32_t>(n), &r32);
		CAPTURE(n);
		CHECK(r64 == static_cast<double>(r32));
	}
}
}  // namespace

TEST_CASE("[C5 arm_max_f64] agrees exactly with arm_max_f32 (value and index) on float representable data") { c5Idx(arm_max_f64, arm_max_f32, 0xC501ULL); }
TEST_CASE("[C5 arm_min_f64] agrees exactly with arm_min_f32 (value and index) on float representable data") { c5Idx(arm_min_f64, arm_min_f32, 0xC502ULL); }
TEST_CASE("[C5 arm_absmax_f64] agrees exactly with arm_absmax_f32 (value and index) on float representable data") { c5Idx(arm_absmax_f64, arm_absmax_f32, 0xC503ULL); }
TEST_CASE("[C5 arm_absmin_f64] agrees exactly with arm_absmin_f32 (value and index) on float representable data") { c5Idx(arm_absmin_f64, arm_absmin_f32, 0xC504ULL); }
TEST_CASE("[C5 arm_max_no_idx_f64] agrees exactly with arm_max_no_idx_f32 on float representable data") { c5NoIdx(arm_max_no_idx_f64, arm_max_no_idx_f32, 0xC505ULL); }
TEST_CASE("[C5 arm_min_no_idx_f64] agrees exactly with arm_min_no_idx_f32 on float representable data") { c5NoIdx(arm_min_no_idx_f64, arm_min_no_idx_f32, 0xC506ULL); }
TEST_CASE("[C5 arm_absmax_no_idx_f64] agrees exactly with arm_absmax_no_idx_f32 on float representable data") { c5NoIdx(arm_absmax_no_idx_f64, arm_absmax_no_idx_f32, 0xC507ULL); }
TEST_CASE("[C5 arm_absmin_no_idx_f64] agrees exactly with arm_absmin_no_idx_f32 on float representable data") { c5NoIdx(arm_absmin_no_idx_f64, arm_absmin_no_idx_f32, 0xC508ULL); }

TEST_CASE("[C5 arm_accumulate_f64] agrees with arm_accumulate_f32 within (n+4) u32 sum|x|") {
	Rng g(0xC509ULL);
	for (size_t n = 1; n <= 33; n++) {
		const FData x = floatData(g, n, 100.0);
		double r64 = 0.0; float r32 = 0.0f;
		arm_accumulate_f64(x.d.data(), static_cast<uint32_t>(n), &r64);
		arm_accumulate_f32(x.f.data(), static_cast<uint32_t>(n), &r32);
		CAPTURE(n);
		CHECK(std::fabs(r64 - static_cast<double>(r32)) <= static_cast<double>(n + 4) * kU32 * sumAbs(x.d));
	}
}
TEST_CASE("[C5 arm_mean_f64] agrees with arm_mean_f32 within (n+4) u32 sum|x| / n") {
	Rng g(0xC50AULL);
	for (size_t n = 1; n <= 33; n++) {
		const FData x = floatData(g, n, 100.0);
		double r64 = 0.0; float r32 = 0.0f;
		arm_mean_f64(x.d.data(), static_cast<uint32_t>(n), &r64);
		arm_mean_f32(x.f.data(), static_cast<uint32_t>(n), &r32);
		CAPTURE(n);
		CHECK(std::fabs(r64 - static_cast<double>(r32)) <= static_cast<double>(n + 4) * kU32 * sumAbs(x.d) / static_cast<double>(n));
	}
}
TEST_CASE("[C5 arm_power_f64] agrees with arm_power_f32 within (n+4) u32 relative") {
	Rng g(0xC50BULL);
	for (size_t n = 1; n <= 33; n++) {
		const FData x = floatData(g, n, 100.0);
		double r64 = 0.0; float r32 = 0.0f;
		arm_power_f64(x.d.data(), static_cast<uint32_t>(n), &r64);
		arm_power_f32(x.f.data(), static_cast<uint32_t>(n), &r32);
		CAPTURE(n);
		CHECK(std::fabs(r64 - static_cast<double>(r32)) <= static_cast<double>(n + 4) * kU32 * r64);
	}
}
TEST_CASE("[C5 arm_var_f64] agrees with arm_var_f32 within (n+4) u32 relative") {
	Rng g(0xC50CULL);
	for (size_t n = 2; n <= 33; n++) {
		const FData x = floatData(g, n, 100.0);
		double r64 = 0.0; float r32 = 0.0f;
		arm_var_f64(x.d.data(), static_cast<uint32_t>(n), &r64);
		arm_var_f32(x.f.data(), static_cast<uint32_t>(n), &r32);
		CAPTURE(n);
		CHECK(std::fabs(r64 - static_cast<double>(r32)) <= static_cast<double>(n + 4) * kU32 * r64);
	}
}
TEST_CASE("[C5 arm_std_f64] agrees with arm_std_f32 within (n+4) u32 relative") {
	Rng g(0xC50DULL);
	for (size_t n = 2; n <= 33; n++) {
		const FData x = floatData(g, n, 100.0);
		double r64 = 0.0; float r32 = 0.0f;
		arm_std_f64(x.d.data(), static_cast<uint32_t>(n), &r64);
		arm_std_f32(x.f.data(), static_cast<uint32_t>(n), &r32);
		CAPTURE(n);
		CHECK(std::fabs(r64 - static_cast<double>(r32)) <= static_cast<double>(n + 4) * kU32 * r64);
	}
}
TEST_CASE("[C5 arm_mse_f64] agrees with arm_mse_f32 within (n+4) u32 relative") {
	Rng g(0xC50EULL);
	for (size_t n = 1; n <= 33; n++) {
		const FData a = floatData(g, n, 100.0);
		const FData b = floatData(g, n, 100.0);
		double r64 = 0.0; float r32 = 0.0f;
		arm_mse_f64(a.d.data(), b.d.data(), static_cast<uint32_t>(n), &r64);
		arm_mse_f32(a.f.data(), b.f.data(), static_cast<uint32_t>(n), &r32);
		CAPTURE(n);
		CHECK(std::fabs(r64 - static_cast<double>(r32)) <= static_cast<double>(n + 4) * kU32 * r64);
	}
}
TEST_CASE("[C5 arm_entropy_f64] agrees with arm_entropy_f32 within (n+6) u32 sum|p ln p|") {
	Rng g(0xC50FULL);
	for (size_t n = 1; n <= 33; n++) {
		FData p;
		for (size_t i = 0; i < n; i++) { const double x = g.uni(); const float f = static_cast<float>(0.02 + 0.98 * std::fabs(x)); p.f.push_back(f); p.d.push_back(static_cast<double>(f)); }
		double bound = 0.0;
		for (double x : p.d) bound += std::fabs(x * std::log(x));
		bound *= static_cast<double>(n + 6) * kU32;
		const double r64 = arm_entropy_f64(p.d.data(), static_cast<uint32_t>(n));
		const float r32 = arm_entropy_f32(p.f.data(), static_cast<uint32_t>(n));
		CAPTURE(n);
		CHECK(std::fabs(r64 - static_cast<double>(r32)) <= bound);
	}
}
TEST_CASE("[C5 arm_kullback_leibler_f64] agrees with arm_kullback_leibler_f32 within the derived bound") {
	Rng g(0xC510ULL);
	for (size_t n = 1; n <= 33; n++) {
		FData a, b;
		for (size_t i = 0; i < n; i++) { const double x = g.uni(); const float f = static_cast<float>(0.05 + 0.95 * std::fabs(x)); a.f.push_back(f); a.d.push_back(static_cast<double>(f)); }
		for (size_t i = 0; i < n; i++) { const double x = g.uni(); const float f = static_cast<float>(0.05 + 0.95 * std::fabs(x)); b.f.push_back(f); b.d.push_back(static_cast<double>(f)); }
		// bound = u32 * (sum pA (2 + 3 |ln q|) + n * sum |t|), t = pA ln q, q = pB / pA
		double s1 = 0.0, s2 = 0.0;
		for (size_t i = 0; i < n; i++) { const double lq = std::log(b.d[i] / a.d[i]); s1 += a.d[i] * (2.0 + 3.0 * std::fabs(lq)); s2 += std::fabs(a.d[i] * lq); }
		const double bound = kU32 * (s1 + static_cast<double>(n) * s2);
		const double r64 = arm_kullback_leibler_f64(a.d.data(), b.d.data(), static_cast<uint32_t>(n));
		const float r32 = arm_kullback_leibler_f32(a.f.data(), b.f.data(), static_cast<uint32_t>(n));
		CAPTURE(n);
		CHECK(std::fabs(r64 - static_cast<double>(r32)) <= bound);
	}
}

}  // TEST_SUITE
