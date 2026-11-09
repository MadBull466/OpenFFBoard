/*
 * Statistics f16 (phase P11, group G3, work order SF16): arm_absmax_f16, arm_absmin_f16, arm_max_f16, arm_min_f16 (with
 * index), arm_absmax_no_idx_f16, arm_absmin_no_idx_f16, arm_max_no_idx_f16, arm_min_no_idx_f16, arm_accumulate_f16,
 * arm_mean_f16, arm_mse_f16, arm_power_f16, arm_rms_f16, arm_std_f16, arm_var_f16, arm_entropy_f16,
 * arm_kullback_leibler_f16, arm_logsumexp_f16, arm_logsumexp_dot_prod_f16 (19 units; all f16 files of
 * Source/StatisticsFunctions). Requirements of Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md: C1 (ARM's own test
 * ids), C2 (output arguments and the temporary buffer of logsumexp_dot_prod), C3 (DEF in double with derived bounds),
 * C4 (block size 1 and 2, equal data, hand-derived exact cases), C5 (agreement with the f32 function), C6 (host notes).
 *
 * Compiled with -D__ARM_FP16_FORMAT_IEEE=1 -D__fp16=_Float16 (file name ends in _f16.cpp, lead decision L2).
 *
 * Pattern files (Libraries/CMSIS-DSP/Testing/Patterns/DSP/Stats/StatsF16/) as wired in Testing/desc_f16.txt (suite
 * "Statistics Tests F16", lines 12-120) and Testing/Source/Tests/StatsTestsF16.cpp (test bodies lines 12-330, setUp from
 * line 380). desc_f16.txt labels the sizes "nb=7", "nb=8n", "nb=8n+1"; the real sizes are 7, 16 and 23 (reload(.., n),
 * refOffset 0, 1, 2) and "long" is 100 samples (refOffset 3):
 *   max / min (+ index)         Input1_f16 (300) -> MaxVals1 / MinVals3, MaxIndexes1 / MinIndexes3 (s16)   ids 1-3, 7-9
 *   max_no_idx / min_no_idx     Input1_f16 -> MaxVals1 / MinVals3                                          ids 26-28, 40-42
 *   absmax / absmin (+ index)   InputNew1_f16 (300) -> AbsMaxVals26 / AbsMinVals27, AbsMaxIndexes26 / AbsMinIndexes27
 *                                                                                                          ids 34-39
 *   absmax_no_idx / absmin_no_idx InputNew1_f16 -> AbsMaxVals26 / AbsMinVals27                             ids 43-48
 *   mean   Input2_f16 -> MeanVals2 (ids 4-6, long 29); power Input1 -> PowerVals4 (ids 10-12);
 *   rms    Input1 -> RmsVals5 (ids 13-15, long 30); std Input1 -> StdVals6 (ids 16-18, long 31, stability id 33 has no
 *   pattern); var Input1 -> VarVals7 (ids 19-21, long 32)
 *   mse    InputNew1 / InputNew2 -> MSEVals28 (ids 49-51, long 52)
 *   accumulate InputAccumulate1 + InputAccumulateConfig1 -> RefAccumulate1 (id 53; ARM does not advance the input pointer
 *          between the 4 blocks, so every block sums the first dims[i+1] samples: the test does the same)
 *   entropy Input22 + Dims22 -> RefEntropy22 (id 22); logsumexp Input23 + Dims23 -> RefLogSumExp23 (id 23);
 *   kullback_leibler InputA24 / InputB24 + Dims24 -> RefKL24 (id 24); logsumexp_dot_prod InputA25 / InputB25 + Dims25 ->
 *   RefLogSumExpDot25 (id 25, temp buffer of 12 samples)
 * All ids that exist are enabled in desc_f16.txt; none is commented out. (id 33 = test_std_stability_f16.)
 *
 * ARM tolerances (StatsTestsF16.cpp:7-19): SNR_THRESHOLD 48 dB (:7), SNR_KULLBACK_THRESHOLD 40 dB (:8), REL_ERROR 6.0e-3
 *   (:15), REL_ERROR_ACCUMULATE 7.0e-3 (:16), REL_KULLBACK_ERROR 5.0e-3 (:18), ABS_KULLBACK_ERROR 1.0e-2 (:19). The
 *   max / min / absmax / absmin values and indexes are compared EXACTLY (ASSERT_EQ, :42-43 and following). ASSERT_REL_ERROR
 *   is |a-b| / ((|a|+|b|)/2) <= threshold (Testing/FrameworkSource/Error.cpp:239-252, symmetric relative error);
 *   the kullback_leibler test uses ASSERT_CLOSE_ERROR (|out-ref| <= abs + rel*|ref|). The scalar ASSERT_SNR is 10 log10(ref^2 /
 *   (ref-out)^2), i.e. a relative error below 3.98e-3 for 48 dB. Used unchanged (L4).
 *   test_std_stability_f16 (:318-ff) asserts |5.47723 - std| < 0.32.
 *
 * Not tested, NEEDS REVIEW (lead decisions): blockSize 0 of mean / logsumexp / entropy / KL (0/0 NaN or reads before the
 * data; var / std with 0 or 1 samples are characterized and return 0); f16 accumulation overflow of mean (the mean of
 * {60000, 60000} is +inf although representable; accumulate / power overflow to +-inf are characterized).
 *
 * NEEDS REVIEW (f16 overflow / stagnation where the true result is representable; NOT pinned by any test, found by a reviewer
 * probe): mean{30000 x4} = +inf, var{30000 x4} = +inf (true 0), std = +inf, rms{200,200} = +inf (true 200); mean of ones is 1 at
 * n = 2048, 0.5 at n = 4096 and 0 at n = 70000 because (_Float16)blockSize = +inf; accumulate of ones stops growing at 2048;
 * logsumexp of zeros stagnates at 7.625 for n >= 2048. The gam(n) bounds below need n*u < 1, i.e. n < 2048; the random tests
 * stop at n = 33 and the constant-data tests at n = 100. tolLse is loose for small n (a 1.002 gain on the log or exp argument
 * survives the bound); the exact tests (hand values, single-sample sweeps) are the tight ones.
 *
 * Host vs target (C6): the host runs the portable C paths with _Float16 emulated by the compiler (x86-64 without
 * AVX512-FP16: operations are done in float and rounded back to binary16, at the latest at every store; with the default
 * -fexcess-precision=fast the compiler may keep float intermediates inside an expression; the tests also pass with standard
 * precision, with FMA contraction and with clang, because the bounds assume the maximum number of roundings). The C code accumulates in f16
 * (_Float16 sum) in all of mean, power, rms, var, mse, accumulate and logsumexp; the unrolled loops (4 for mean, power,
 * rms, var, accumulate; 8 for mse) keep ONE running sum in order, so the order of additions equals the plain loop on the
 * host. On a Cortex-M55 (MVE) the sums are lane sums (vecAddAcrossF16Mve) with fused multiply-add, so the last bits differ;
 * the bounds below are first order bounds for ANY order of at most n additions and any count of roundings up to the one
 * assumed, so they hold for both. expf / logf come from libm (the library calls them in float and converts to f16): on
 * the target newlib-nano, results differ in the last float bit, irrelevant after rounding to f16 within the bounds. The
 * firmware ships no f16 code (the Makefile excludes %_f16.c); these tests only protect the library as such.
 *
 * Error model used for every bound (Higham): u = 2^-11 is the unit roundoff of binary16 (round to nearest); a sum of k
 * terms that is rounded after every addition has relative error <= gam(k) = k*u / (1 - k*u) of the sum of the absolute
 * terms. Products, quotients, differences and the f16 cast of a float result each add one factor (1+d), |d| <= u. The
 * bounds assume that no intermediate value leaves the normal range [2^-14, 65504] except where an absolute term is added
 * (2^-25 per product / quotient in the subnormal range).
 */
#include "doctest.h"

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math_f16.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <cstring>
#include <string>
#include <vector>

#ifndef ARM_FLOAT16_SUPPORTED
#error "ARM_FLOAT16_SUPPORTED is not defined: the f16 compile definitions are missing for this file"
#endif

namespace {

typedef float16_t F;

const double kSnr = 48.0;                 // ARM SNR_THRESHOLD (StatsTestsF16.cpp:7)
const double kSnrKl = 40.0;               // ARM SNR_KULLBACK_THRESHOLD (:8)
const double kRel = 6.0e-3;               // ARM REL_ERROR (:15)
const double kRelAcc = 7.0e-3;            // ARM REL_ERROR_ACCUMULATE (:16)
const double kRelKl = 5.0e-3;             // ARM REL_KULLBACK_ERROR (:18)
const double kAbsKl = 1.0e-2;             // ARM ABS_KULLBACK_ERROR (:19)
const double kU = 4.8828125e-4;           // 2^-11, unit roundoff of binary16
const double kSub = 3.0e-8;               // 2^-25 = 2.98e-8, absolute error of a product in the subnormal range
const double kInf = HUGE_VAL;

double gam(double k) { return k * kU / (1.0 - k * kU); }

typedef void (*IdxFn)(const F *, uint32_t, F *, uint32_t *);
typedef void (*Fn1)(const F *, uint32_t, F *);
typedef void (*Fn2)(const F *, const F *, uint32_t, F *);

F fromBits(uint16_t b) {
	F h;
	std::memcpy(&h, &b, sizeof h);
	return h;
}

uint16_t bitsOf(F h) {
	uint16_t b;
	std::memcpy(&b, &h, sizeof b);
	return b;
}

F h16(double d) { return static_cast<F>(d); }
double dbl(F h) { return static_cast<double>(h); }

std::vector<F> toF16(const std::vector<uint16_t> &bits) {
	std::vector<F> v(bits.size());
	for (size_t i = 0; i < bits.size(); i++) v[i] = fromBits(bits[i]);
	return v;
}

std::vector<F> load(const char *name) {
	return toF16(dsp::loadPattern(std::string("Stats/StatsF16/") + name).f16Bits());
}

std::vector<int16_t> loadS16(const char *name) {
	return dsp::loadPattern(std::string("Stats/StatsF16/") + name).q15();
}

std::vector<F> head(const std::vector<F> &v, size_t n) { return std::vector<F>(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(n)); }

std::vector<F> make(std::initializer_list<double> values) {
	std::vector<F> v;
	for (double d : values) v.push_back(h16(d));
	return v;
}

std::vector<float> toFloat(const std::vector<F> &v) {
	std::vector<float> o(v.size());
	for (size_t i = 0; i < v.size(); i++) o[i] = static_cast<float>(v[i]);
	return o;
}

// NaN-pattern sentinel in the output variables, so "never written" is detectable
const uint16_t kSentinelBits = 0x7E01;
const uint32_t kSentinelIdx = 0xDEADBEEFu;

// Deterministic LCG; every call is its own statement (lesson 1)
struct Rng {
	uint32_t s;
	explicit Rng(uint32_t seed) : s(seed * 2654435761u + 12345u) {}
	double next() {  // uniform in [0, 1)
		s = s * 1664525u + 1013904223u;
		return static_cast<double>(s >> 8) / 16777216.0;
	}
	uint32_t below(uint32_t n) {
		double r = next();
		return static_cast<uint32_t>(r * n);
	}
};

std::vector<F> randVec(size_t n, uint32_t seed, double lo, double hi) {
	Rng rng(seed);
	std::vector<F> v(n);
	for (size_t i = 0; i < n; i++) {
		double r = rng.next();
		v[i] = h16(lo + (hi - lo) * r);
	}
	return v;
}

// n values with pairwise different magnitudes (0.25 * a permutation of 0..n-1, so all exact in f16 for n <= 2048),
// random signs if wantSign. No ties in value or in absolute value.
std::vector<F> distinctVec(size_t n, uint32_t seed, bool wantSign) {
	Rng rng(seed);
	std::vector<uint32_t> perm(n);
	for (size_t i = 0; i < n; i++) perm[i] = static_cast<uint32_t>(i);
	for (size_t i = n; i > 1; i--) {
		uint32_t j = rng.below(static_cast<uint32_t>(i));
		std::swap(perm[i - 1], perm[j]);
	}
	std::vector<F> v(n);
	for (size_t i = 0; i < n; i++) {
		double m = 0.25 * perm[i];
		uint32_t sg = rng.below(2);
		v[i] = h16((wantSign && sg == 1u) ? -m : m);
	}
	return v;
}

// Sizes around the unroll boundaries of 4 and 8 (and the scalar tail)
const size_t kSizes[] = {1, 2, 3, 4, 5, 7, 8, 9, 16, 17, 33};

double sumAbs(const std::vector<F> &x) {
	double s = 0;
	for (F v : x) s += std::fabs(dbl(v));
	return s;
}

bool within(double out, double exact, double tol) { return std::fabs(out - exact) <= tol; }

// ARM's relative error (Error.cpp:239): |a-b| / ((|a|+|b|)/2); 0 if both are zero
double armRel(F a, F b) {
	double da = dbl(a), db = dbl(b);
	double avg = (std::fabs(da) + std::fabs(db)) / 2.0;
	if (avg == 0.0) return 0.0;
	return std::fabs(da - db) / avg;
}

// ARM's scalar check: ASSERT_SNR(result, ref, 48) and ASSERT_REL_ERROR(result, ref, rel)
void checkArmScalar(F out, F ref, double snrThr, double rel) {
	std::vector<F> o(1, out), r(1, ref);
	CHECK(dsp::snr(r, o) >= snrThr);
	CHECK(armRel(out, ref) <= rel);
}

// ARM's vector check: ASSERT_SNR(ref, output, snr) and ASSERT_REL_ERROR per sample
void checkArmVector(const std::vector<F> &out, const std::vector<F> &ref, double snrThr, double rel) {
	REQUIRE(out.size() == ref.size());
	CHECK(dsp::snr(ref, out) >= snrThr);
	for (size_t i = 0; i < ref.size(); i++) {
		INFO("sample " << i);
		CHECK(armRel(out[i], ref[i]) <= rel);
	}
}

struct IdxResult {
	F value;
	uint32_t index;
};

// The value and index outputs live in guarded buffers (guard words behind them, poisoned before the call): a write past the
// output and a missing write are both detected (a failing check is only raised on damage, to keep the sweeps fast)
IdxResult callIdx(IdxFn fn, const std::vector<F> &x) {
	dsp::GuardedBuffer<F> v(1);
	dsp::GuardedBuffer<uint32_t> ix(1);
	fn(x.data(), static_cast<uint32_t>(x.size()), v.data(), ix.data());
	if (!v.guardIntact()) FAIL_CHECK("value output guard damaged");
	if (!ix.guardIntact()) FAIL_CHECK("index output guard damaged");
	return {v.output()[0], ix.output()[0]};
}

F call1(Fn1 fn, const std::vector<F> &x) {
	dsp::GuardedBuffer<F> v(1);
	fn(x.data(), static_cast<uint32_t>(x.size()), v.data());
	if (!v.guardIntact()) FAIL_CHECK("result output guard damaged");
	return v.output()[0];
}

F call2(Fn2 fn, const std::vector<F> &a, const std::vector<F> &b) {
	dsp::GuardedBuffer<F> v(1);
	fn(a.data(), b.data(), static_cast<uint32_t>(a.size()), v.data());
	if (!v.guardIntact()) FAIL_CHECK("result output guard damaged");
	return v.output()[0];
}

// DEF of the extrema: key = x or |x|; best = max or min of the key; on a tie the FIRST index (what the strict comparison
// of the library does; DOC is silent about ties, see the characterization tests)
struct ExtRef {
	double value;
	uint32_t index;
};

ExtRef defExt(const std::vector<F> &x, bool isMax, bool absolute) {
	double best = absolute ? std::fabs(dbl(x[0])) : dbl(x[0]);
	uint32_t bi = 0;
	for (size_t i = 1; i < x.size(); i++) {
		double k = absolute ? std::fabs(dbl(x[i])) : dbl(x[i]);
		if (isMax ? (k > best) : (k < best)) {
			best = k;
			bi = static_cast<uint32_t>(i);
		}
	}
	return {best, bi};
}

// all non-NaN binary16 patterns
bool isNan16(uint32_t b) { return (b & 0x7C00u) == 0x7C00u && (b & 0x03FFu) != 0; }

// strided list of patterns for the pair sweeps: every 251st pattern plus the special ones
std::vector<F> sweepValues() {
	std::vector<F> v;
	for (uint32_t b = 0; b < 65536u; b += 251u)
		if (!isNan16(b)) v.push_back(fromBits(static_cast<uint16_t>(b)));
	const uint16_t special[] = {0x0000, 0x8000, 0x0001, 0x8001, 0x03FF, 0x0400, 0x3C00, 0xBC00, 0x7BFF, 0xFBFF, 0x7C00, 0xFC00};
	for (uint16_t b : special) v.push_back(fromBits(b));
	return v;
}

// ---- bounds (absolute half widths around the exact result in double) ----------------------------------------------

// mean = sum / n: n-1 additions and one division, n roundings: gam(n) * sum|x| / n
double tolMean(const std::vector<F> &x) { return gam(static_cast<double>(x.size())) * sumAbs(x) / static_cast<double>(x.size()); }

// accumulate: n-1 additions (the first adds to 0, exact): gam(n) * sum|x|
double tolAccumulate(const std::vector<F> &x) { return gam(static_cast<double>(x.size())) * sumAbs(x); }

// power = sum of fl(x*x): each term has 1 product rounding and up to n addition roundings
double tolPower(const std::vector<F> &x) {
	double s2 = 0;
	for (F v : x) s2 += dbl(v) * dbl(v);
	double n = static_cast<double>(x.size());
	return gam(n + 1) * s2 + n * kSub;
}

struct Range {
	double lo, hi;
};

bool inRange(double v, const Range &r) { return v >= r.lo && v <= r.hi; }

// variance: see tolVar below
Range rangeVar(const std::vector<F> &x, double *exactOut) {
	double n = static_cast<double>(x.size());
	double sum = 0;
	for (F v : x) sum += dbl(v);
	double mu = sum / n, ss = 0;
	for (F v : x) ss += (dbl(v) - mu) * (dbl(v) - mu);
	double var = ss / (n - 1.0);
	*exactOut = var;
	// computed mean m = mu + e with |e| <= gam(n) sum|x| / n. Computed terms fl(fl(x-m)^2) have relative error
	// <= gam(3) (difference once, in the square twice via (1+d)^2, then the product rounding), added with up to n-1
	// roundings, divided by the exact (n-1) with one more rounding: relative error <= gam(n+4) overall.
	// sum (x-m)^2 = (n-1) var + n e^2 (the first order term cancels because sum (x - mu) = 0).
	double e = gam(n) * sumAbs(x) / n;
	double g = gam(n + 4);
	double up = (1.0 + g) * (var + n * e * e / (n - 1.0)) + kSub;
	double lo = (1.0 - g) * var - kSub;
	return {lo < 0 ? 0 : lo, up};
}

Range rangeStd(const std::vector<F> &x, double *exactOut) {
	double var;
	Range v = rangeVar(x, &var);
	*exactOut = std::sqrt(var);
	// sqrt is evaluated in float (error 6e-8, negligible) and cast to f16 (u)
	return {std::sqrt(v.lo) * (1.0 - kU), std::sqrt(v.hi) * (1.0 + kU) + kSub};
}

Range rangeRms(const std::vector<F> &x, double *exactOut) {
	double n = static_cast<double>(x.size());
	double s2 = 0;
	for (F v : x) s2 += dbl(v) * dbl(v);
	double p = s2 / n;
	*exactOut = std::sqrt(p);
	// sum of n products: gam(n+1); the division by n one more rounding: gam(n+2) on the mean square
	double g = gam(n + 2);
	double lo = (1.0 - g) * p - kSub, hi = (1.0 + g) * p + kSub;
	return {std::sqrt(lo < 0 ? 0 : lo) * (1.0 - kU), std::sqrt(hi) * (1.0 + kU) + kSub};
}

// mse: d = fl(a-b) (1 rounding), fl(d*d) (2 more through the square of d plus the product rounding), n-1 additions,
// division: relative error <= gam(n+4) of sum (a-b)^2 / n
double tolMse(const std::vector<F> &a, const std::vector<F> &b, double *exactOut) {
	double n = static_cast<double>(a.size()), s = 0;
	for (size_t i = 0; i < a.size(); i++) {
		double d = dbl(a[i]) - dbl(b[i]);
		s += d * d;
	}
	*exactOut = s / n;
	return gam(n + 4) * s / n + kSub;
}

// entropy -sum p ln p: ln p rounded to f16 (u), product (u), n-1 additions; logf float error negligible (1e-7 abs)
double tolEntropy(const std::vector<F> &p, double *exactOut) {
	double n = static_cast<double>(p.size()), e = 0, sabs = 0;
	for (F v : p) {
		double t = dbl(v) * std::log(dbl(v));
		e -= t;
		sabs += std::fabs(t);
	}
	*exactOut = e;
	return gam(n + 2) * sabs + n * kSub + 2.0e-7 * n;
}

// kullback_leibler -sum pA ln(pB/pA): ratio in float (6e-8), logf, cast to f16 (u), product (u), additions
double tolKl(const std::vector<F> &a, const std::vector<F> &b, double *exactOut) {
	double n = static_cast<double>(a.size()), e = 0, sabs = 0, sa = 0;
	for (size_t i = 0; i < a.size(); i++) {
		double t = dbl(a[i]) * std::log(dbl(b[i]) / dbl(a[i]));
		e -= t;
		sabs += std::fabs(t);
		sa += dbl(a[i]);
	}
	*exactOut = e;
	return gam(n + 2) * sabs + 2.0e-7 * sa + n * kSub;
}

// logsumexp = M + fl(ln S): M exact (maximum), d_i = fl(x_i - M) (u*|d_i| in the exponent), e_i = fl(exp(d_i)) (u), S
// = fl(sum e_i): relative error of S <= sum_i e^{d_i} (|d_i| u + u) / S + gam(n) (+ 3e-8 per term for exp results in
// the subnormal range); ln S is changed by at most -ln(1 - eps) (eps = that error); fl16(ln S) adds u * ln S; the sum
// M + fl(ln S) rounds once more (u * |result|). The input values are exact f16.
double tolLse(const std::vector<F> &x, double *exactOut) {
	double n = static_cast<double>(x.size()), m = dbl(x[0]);
	for (F v : x) m = std::max(m, dbl(v));
	double s = 0, w = 0;
	for (F v : x) {
		double d = dbl(v) - m, ed = std::exp(d);
		s += ed;
		w += ed * (std::fabs(d) * 1.01 * kU + kU);
	}
	double ln_s = std::log(s), exact = m + ln_s;
	*exactOut = exact;
	double eps = w / s + gam(n) + n * kSub;
	double errLog = -std::log1p(-eps);
	return 1.001 * (errLog + kU * (ln_s + errLog) + kU * (std::fabs(exact) + errLog)) + 1.0e-7;
}

// shared check of a scalar against exact +- tol
#define CHECK_NEAR_F16(out, exact, tol) \
	do { \
		INFO("out " << dbl(out) << " exact " << (exact) << " tol " << (tol)); \
		CHECK(within(dbl(out), (exact), (tol))); \
	} while (0)

// C1 helper: ARM scalar test on the first n samples, ref at offset
void c1Scalar(Fn1 fn, const char *inName, const char *refName, size_t n, size_t off, double rel) {
	std::vector<F> in = load(inName), ref = load(refName);
	REQUIRE(in.size() >= n);
	REQUIRE(ref.size() > off);
	INFO("block size " << n);
	F r = call1(fn, head(in, n));
	checkArmScalar(r, ref[off], kSnr, rel);
}

}  // namespace

TEST_SUITE("DSP Statistics F16") {

// ===============================================================================================================
// C1: ARM's test ids
// ===============================================================================================================

TEST_CASE("[C1 arm_max_f16] ARM test_max_f16 (ids 1-3, 7/16/23 samples): value and index equal the reference exactly") {
	std::vector<F> in = load("Input1_f16.txt"), rv = load("MaxVals1_f16.txt");
	std::vector<int16_t> ri = loadS16("MaxIndexes1_s16.txt");
	const size_t sizes[3] = {7, 16, 23};
	for (size_t k = 0; k < 3; k++) {
		INFO("block size " << sizes[k]);
		IdxResult r = callIdx(arm_max_f16, head(in, sizes[k]));
		CHECK(dbl(r.value) == dbl(rv[k]));
		CHECK(static_cast<int16_t>(r.index) == ri[k]);
	}
}

TEST_CASE("[C1 arm_min_f16] ARM test_min_f16 (ids 7-9, 7/16/23 samples): value and index equal the reference exactly") {
	std::vector<F> in = load("Input1_f16.txt"), rv = load("MinVals3_f16.txt");
	std::vector<int16_t> ri = loadS16("MinIndexes3_s16.txt");
	const size_t sizes[3] = {7, 16, 23};
	for (size_t k = 0; k < 3; k++) {
		INFO("block size " << sizes[k]);
		IdxResult r = callIdx(arm_min_f16, head(in, sizes[k]));
		CHECK(dbl(r.value) == dbl(rv[k]));
		CHECK(static_cast<int16_t>(r.index) == ri[k]);
	}
}

TEST_CASE("[C1 arm_absmax_f16] ARM test_absmax_f16 (ids 34-36, 7/16/23 samples): value and index equal the reference exactly") {
	std::vector<F> in = load("InputNew1_f16.txt"), rv = load("AbsMaxVals26_f16.txt");
	std::vector<int16_t> ri = loadS16("AbsMaxIndexes26_s16.txt");
	const size_t sizes[3] = {7, 16, 23};
	for (size_t k = 0; k < 3; k++) {
		INFO("block size " << sizes[k]);
		IdxResult r = callIdx(arm_absmax_f16, head(in, sizes[k]));
		CHECK(dbl(r.value) == dbl(rv[k]));
		CHECK(static_cast<int16_t>(r.index) == ri[k]);
	}
}

TEST_CASE("[C1 arm_absmin_f16] ARM test_absmin_f16 (ids 37-39, 7/16/23 samples): value and index equal the reference exactly") {
	std::vector<F> in = load("InputNew1_f16.txt"), rv = load("AbsMinVals27_f16.txt");
	std::vector<int16_t> ri = loadS16("AbsMinIndexes27_s16.txt");
	const size_t sizes[3] = {7, 16, 23};
	for (size_t k = 0; k < 3; k++) {
		INFO("block size " << sizes[k]);
		IdxResult r = callIdx(arm_absmin_f16, head(in, sizes[k]));
		CHECK(dbl(r.value) == dbl(rv[k]));
		CHECK(static_cast<int16_t>(r.index) == ri[k]);
	}
}

static void c1NoIdx(Fn1 fn, const char *inName, const char *refName) {
	std::vector<F> in = load(inName), rv = load(refName);
	const size_t sizes[3] = {7, 16, 23};
	for (size_t k = 0; k < 3; k++) {
		INFO("block size " << sizes[k]);
		CHECK(dbl(call1(fn, head(in, sizes[k]))) == dbl(rv[k]));
	}
}

TEST_CASE("[C1 arm_max_no_idx_f16] ARM test_max_no_idx_f16 (ids 26-28): the value equals the reference exactly") {
	c1NoIdx(arm_max_no_idx_f16, "Input1_f16.txt", "MaxVals1_f16.txt");
}

TEST_CASE("[C1 arm_min_no_idx_f16] ARM test_min_no_idx_f16 (ids 40-42): the value equals the reference exactly") {
	c1NoIdx(arm_min_no_idx_f16, "Input1_f16.txt", "MinVals3_f16.txt");
}

TEST_CASE("[C1 arm_absmax_no_idx_f16] ARM test_absmax_no_idx_f16 (ids 43-45): the value equals the reference exactly") {
	c1NoIdx(arm_absmax_no_idx_f16, "InputNew1_f16.txt", "AbsMaxVals26_f16.txt");
}

TEST_CASE("[C1 arm_absmin_no_idx_f16] ARM test_absmin_no_idx_f16 (ids 46-48): the value equals the reference exactly") {
	c1NoIdx(arm_absmin_no_idx_f16, "InputNew1_f16.txt", "AbsMinVals27_f16.txt");
}

TEST_CASE("[C1 arm_mean_f16] ARM test_mean_f16 (ids 4-6, 7/16/23 samples): SNR >= 48 dB and relative error <= 6e-3") {
	c1Scalar(arm_mean_f16, "Input2_f16.txt", "MeanVals2_f16.txt", 7, 0, kRel);
	c1Scalar(arm_mean_f16, "Input2_f16.txt", "MeanVals2_f16.txt", 16, 1, kRel);
	c1Scalar(arm_mean_f16, "Input2_f16.txt", "MeanVals2_f16.txt", 23, 2, kRel);
}

TEST_CASE("[C1 arm_mean_f16] ARM test_mean_f16 long (id 29, 100 samples): SNR >= 48 dB and relative error <= 6e-3") {
	c1Scalar(arm_mean_f16, "Input2_f16.txt", "MeanVals2_f16.txt", 100, 3, kRel);
}

TEST_CASE("[C1 arm_power_f16] ARM test_power_f16 (ids 10-12, 7/16/23 samples): SNR >= 48 dB and relative error <= 6e-3") {
	c1Scalar(arm_power_f16, "Input1_f16.txt", "PowerVals4_f16.txt", 7, 0, kRel);
	c1Scalar(arm_power_f16, "Input1_f16.txt", "PowerVals4_f16.txt", 16, 1, kRel);
	c1Scalar(arm_power_f16, "Input1_f16.txt", "PowerVals4_f16.txt", 23, 2, kRel);
}

TEST_CASE("[C1 arm_rms_f16] ARM test_rms_f16 (ids 13-15, 7/16/23 samples): SNR >= 48 dB and relative error <= 6e-3") {
	c1Scalar(arm_rms_f16, "Input1_f16.txt", "RmsVals5_f16.txt", 7, 0, kRel);
	c1Scalar(arm_rms_f16, "Input1_f16.txt", "RmsVals5_f16.txt", 16, 1, kRel);
	c1Scalar(arm_rms_f16, "Input1_f16.txt", "RmsVals5_f16.txt", 23, 2, kRel);
}

TEST_CASE("[C1 arm_rms_f16] ARM test_rms_f16 long (id 30, 100 samples): SNR >= 48 dB and relative error <= 6e-3") {
	c1Scalar(arm_rms_f16, "Input1_f16.txt", "RmsVals5_f16.txt", 100, 3, kRel);
}

TEST_CASE("[C1 arm_std_f16] ARM test_std_f16 (ids 16-18, 7/16/23 samples): SNR >= 48 dB and relative error <= 6e-3") {
	c1Scalar(arm_std_f16, "Input1_f16.txt", "StdVals6_f16.txt", 7, 0, kRel);
	c1Scalar(arm_std_f16, "Input1_f16.txt", "StdVals6_f16.txt", 16, 1, kRel);
	c1Scalar(arm_std_f16, "Input1_f16.txt", "StdVals6_f16.txt", 23, 2, kRel);
}

TEST_CASE("[C1 arm_std_f16] ARM test_std_f16 long (id 31, 100 samples): SNR >= 48 dB and relative error <= 6e-3") {
	c1Scalar(arm_std_f16, "Input1_f16.txt", "StdVals6_f16.txt", 100, 3, kRel);
}

TEST_CASE("[C1 arm_std_f16] ARM test_std_stability_f16 (id 33): a large offset does not give a negative variance, |std - 5.47723| < 0.32") {
	// ARM: in = {4, 7, 13, 16} + 3000 in float, stored as f16 (spacing 2 in [2048, 4096))
	F in[4] = {h16(4.0), h16(7.0), h16(13.0), h16(16.0)};
	for (int i = 0; i < 4; i++) in[i] = h16(static_cast<float>(in[i]) + 3.0e3f);
	F r = fromBits(kSentinelBits);
	arm_std_f16(in, 4, &r);
	CHECK(std::fabs(5.47723f - static_cast<float>(r)) < 0.32f);
}

TEST_CASE("[C1 arm_var_f16] ARM test_var_f16 (ids 19-21, 7/16/23 samples): SNR >= 48 dB and relative error <= 6e-3") {
	c1Scalar(arm_var_f16, "Input1_f16.txt", "VarVals7_f16.txt", 7, 0, kRel);
	c1Scalar(arm_var_f16, "Input1_f16.txt", "VarVals7_f16.txt", 16, 1, kRel);
	c1Scalar(arm_var_f16, "Input1_f16.txt", "VarVals7_f16.txt", 23, 2, kRel);
}

TEST_CASE("[C1 arm_var_f16] ARM test_var_f16 long (id 32, 100 samples): SNR >= 48 dB and relative error <= 6e-3") {
	c1Scalar(arm_var_f16, "Input1_f16.txt", "VarVals7_f16.txt", 100, 3, kRel);
}

static void c1Mse(size_t n, size_t off) {
	std::vector<F> a = load("InputNew1_f16.txt"), b = load("InputNew2_f16.txt"), ref = load("MSEVals28_f16.txt");
	INFO("block size " << n);
	F r = call2(arm_mse_f16, head(a, n), head(b, n));
	checkArmScalar(r, ref[off], kSnr, kRel);
}

TEST_CASE("[C1 arm_mse_f16] ARM test_mse_f16 (ids 49-51, 7/16/23 samples): SNR >= 48 dB and relative error <= 6e-3") {
	c1Mse(7, 0);
	c1Mse(16, 1);
	c1Mse(23, 2);
}

TEST_CASE("[C1 arm_mse_f16] ARM test_mse_f16 long (id 52, 100 samples): SNR >= 48 dB and relative error <= 6e-3") { c1Mse(100, 3); }

TEST_CASE("[C1 arm_accumulate_f16] ARM test_accumulate_f16 (id 53): the 4 sums match the reference within SNR 48 dB and relative error 7e-3") {
	std::vector<F> in = load("InputAccumulate1_f16.txt"), ref = load("RefAccumulate1_f16.txt");
	std::vector<int16_t> dims = loadS16("InputAccumulateConfig1_s16.txt");
	REQUIRE(dims.size() >= 1);
	REQUIRE(static_cast<size_t>(dims[0]) == ref.size());
	std::vector<F> out(ref.size(), fromBits(kSentinelBits));
	for (size_t i = 0; i < ref.size(); i++) {
		REQUIRE(static_cast<size_t>(dims[i + 1]) <= in.size());
		// ARM does not advance the input pointer: every block is the start of the input
		arm_accumulate_f16(in.data(), static_cast<uint32_t>(dims[i + 1]), &out[i]);
	}
	checkArmVector(out, ref, kSnr, kRelAcc);
}

TEST_CASE("[C1 arm_entropy_f16] ARM test_entropy_f16 (id 22): the 10 entropies match the reference within SNR 48 dB and relative error 6e-3") {
	std::vector<F> in = load("Input22_f16.txt"), ref = load("RefEntropy22_f16.txt");
	std::vector<int16_t> dims = loadS16("Dims22_s16.txt");
	REQUIRE(static_cast<size_t>(dims[0]) == ref.size());
	std::vector<F> out(ref.size());
	size_t pos = 0;
	for (size_t i = 0; i < ref.size(); i++) {
		size_t len = static_cast<size_t>(dims[i + 1]);
		REQUIRE(pos + len <= in.size());
		out[i] = arm_entropy_f16(in.data() + pos, static_cast<uint32_t>(len));
		pos += len;
	}
	checkArmVector(out, ref, kSnr, kRel);
}

TEST_CASE("[C1 arm_logsumexp_f16] ARM test_logsumexp_f16 (id 23): the 10 results match the reference within SNR 48 dB and relative error 6e-3") {
	std::vector<F> in = load("Input23_f16.txt"), ref = load("RefLogSumExp23_f16.txt");
	std::vector<int16_t> dims = loadS16("Dims23_s16.txt");
	REQUIRE(static_cast<size_t>(dims[0]) == ref.size());
	std::vector<F> out(ref.size());
	size_t pos = 0;
	for (size_t i = 0; i < ref.size(); i++) {
		size_t len = static_cast<size_t>(dims[i + 1]);
		REQUIRE(pos + len <= in.size());
		out[i] = arm_logsumexp_f16(in.data() + pos, static_cast<uint32_t>(len));
		pos += len;
	}
	checkArmVector(out, ref, kSnr, kRel);
}

TEST_CASE("[C1 arm_kullback_leibler_f16] ARM test_kullback_leibler_f16 (id 24): the 10 divergences match the reference within SNR 40 dB and 1e-2 abs + 5e-3 rel") {
	std::vector<F> a = load("InputA24_f16.txt"), b = load("InputB24_f16.txt"), ref = load("RefKL24_f16.txt");
	std::vector<int16_t> dims = loadS16("Dims24_s16.txt");
	REQUIRE(static_cast<size_t>(dims[0]) == ref.size());
	std::vector<F> out(ref.size());
	size_t pos = 0;
	for (size_t i = 0; i < ref.size(); i++) {
		size_t len = static_cast<size_t>(dims[i + 1]);
		REQUIRE(pos + len <= a.size());
		out[i] = arm_kullback_leibler_f16(a.data() + pos, b.data() + pos, static_cast<uint32_t>(len));
		pos += len;
	}
	CHECK(dsp::snr(ref, out) >= kSnrKl);
	auto c = dsp::closeAbsRel(ref, out, kAbsKl, kRelKl);
	CHECK_MESSAGE(c, c.message);
}

TEST_CASE("[C1 arm_logsumexp_dot_prod_f16] ARM test_logsumexp_dot_prod_f16 (id 25): the 10 results match the reference within SNR 48 dB and relative error 6e-3") {
	std::vector<F> a = load("InputA25_f16.txt"), b = load("InputB25_f16.txt"), ref = load("RefLogSumExpDot25_f16.txt");
	std::vector<int16_t> dims = loadS16("Dims25_s16.txt");
	REQUIRE(static_cast<size_t>(dims[0]) == ref.size());
	std::vector<F> out(ref.size());
	dsp::GuardedBuffer<F> tmp(12);  // ARM: Temp output of 12 samples (the largest block is 12)
	size_t pos = 0;
	for (size_t i = 0; i < ref.size(); i++) {
		size_t len = static_cast<size_t>(dims[i + 1]);
		REQUIRE(pos + len <= a.size());
		REQUIRE(len <= 12);
		out[i] = arm_logsumexp_dot_prod_f16(a.data() + pos, b.data() + pos, static_cast<uint32_t>(len), tmp.data());
		pos += len;
	}
	checkArmVector(out, ref, kSnr, kRel);
	CHECK(tmp.guardIntact());
}

// ===============================================================================================================
// C2: output arguments. Scalar outputs are sentinel-filled before the call and must be written; the temporary buffer of
// arm_logsumexp_dot_prod_f16 (written by the library, length blockSize) is a GuardedBuffer.
// ===============================================================================================================

TEST_CASE("[C2 arm_logsumexp_dot_prod_f16] the temporary buffer holds a[i]+b[i] for blockSize 1, 3, 4, 5, 9 and the guard behind it is intact") {
	// values with a small exponent spread, so the float sum inside arm_add_f16 is exact and the f16 sum is one rounding
	std::vector<F> a = make({0.5, -1.25, 2.0, 0.75, 1.5, -0.5, 3.0, 0.125, -2.5});
	std::vector<F> b = make({1.0, 0.5, -0.75, 2.25, 0.125, 1.75, -1.0, 0.5, 0.25});
	for (size_t n : {1u, 3u, 4u, 5u, 9u}) {
		INFO("block size " << n);
		dsp::GuardedBuffer<F> tmp(n);
		F r = arm_logsumexp_dot_prod_f16(a.data(), b.data(), static_cast<uint32_t>(n), tmp.data());
		std::vector<F> out = tmp.output();
		for (size_t i = 0; i < n; i++) {
			INFO("sample " << i);
			// all sums are exactly representable (multiples of 1/8 below 8)
			CHECK(dbl(out[i]) == dbl(a[i]) + dbl(b[i]));
		}
		CHECK(tmp.guardIntact());
		CHECK_FALSE(std::isnan(dbl(r)));
	}
}

TEST_CASE("[C2 arm_max_f16] value and index outputs are written for every block size from 1 to 20") {
	for (size_t n = 1; n <= 20; n++) {
		INFO("block size " << n);
		std::vector<F> x = distinctVec(n, static_cast<uint32_t>(n), true);
		IdxResult r = callIdx(arm_max_f16, x);
		ExtRef d = defExt(x, true, false);
		CHECK(dbl(r.value) == d.value);
		CHECK(r.index == d.index);
	}
}

// ===============================================================================================================
// C3: DEF against double, extrema family (distinct values; ties are in the C4 block)
// ===============================================================================================================

static void defExtRandom(IdxFn fn, bool isMax, bool absolute) {
	for (size_t n = 1; n <= 40; n++) {
		for (uint32_t seed = 0; seed < 3; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> x = distinctVec(n, static_cast<uint32_t>(n * 7 + seed), true);
			IdxResult r = callIdx(fn, x);
			ExtRef d = defExt(x, isMax, absolute);
			CHECK(dbl(r.value) == d.value);
			CHECK(r.index == d.index);
		}
	}
}

TEST_CASE("[C3 arm_max_f16] the maximum and its index equal the double reference for 1 to 40 distinct values") { defExtRandom(arm_max_f16, true, false); }
TEST_CASE("[C3 arm_min_f16] the minimum and its index equal the double reference for 1 to 40 distinct values") { defExtRandom(arm_min_f16, false, false); }
TEST_CASE("[C3 arm_absmax_f16] the maximum of |x| and its index equal the double reference for 1 to 40 values of distinct magnitude") {
	defExtRandom(arm_absmax_f16, true, true);
}
TEST_CASE("[C3 arm_absmin_f16] the minimum of |x| and its index equal the double reference for 1 to 40 values of distinct magnitude") {
	defExtRandom(arm_absmin_f16, false, true);
}

static void defExtNoIdx(Fn1 fn, bool isMax, bool absolute) {
	for (size_t n = 1; n <= 40; n++) {
		for (uint32_t seed = 0; seed < 3; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> x = distinctVec(n, static_cast<uint32_t>(n * 11 + seed), true);
			CHECK(dbl(call1(fn, x)) == defExt(x, isMax, absolute).value);
		}
	}
}

TEST_CASE("[C3 arm_max_no_idx_f16] the maximum equals the double reference for 1 to 40 values") { defExtNoIdx(arm_max_no_idx_f16, true, false); }
TEST_CASE("[C3 arm_min_no_idx_f16] the minimum equals the double reference for 1 to 40 values") { defExtNoIdx(arm_min_no_idx_f16, false, false); }
TEST_CASE("[C3 arm_absmax_no_idx_f16] the maximum of |x| equals the double reference for 1 to 40 values") { defExtNoIdx(arm_absmax_no_idx_f16, true, true); }
TEST_CASE("[C3 arm_absmin_no_idx_f16] the minimum of |x| equals the double reference for 1 to 40 values") { defExtNoIdx(arm_absmin_no_idx_f16, false, true); }

// Exhaustive single-sample domain: every non-NaN binary16 pattern (63488 values) as a block of one sample
static void exhaustiveSingle(IdxFn fn, Fn1 fnNoIdx, bool isMax, bool absolute) {
	size_t bad = 0, badIdx = 0, badNo = 0;
	uint32_t firstBad = 0xFFFFFFFFu;
	for (uint32_t b = 0; b < 65536u; b++) {
		if (isNan16(b)) continue;
		std::vector<F> x(1, fromBits(static_cast<uint16_t>(b)));
		double expect = absolute ? std::fabs(dbl(x[0])) : dbl(x[0]);
		IdxResult r = callIdx(fn, x);
		if (!(dbl(r.value) == expect)) {
			bad++;
			if (firstBad == 0xFFFFFFFFu) firstBad = b;
		}
		if (r.index != 0) badIdx++;
		// the no_idx variants start from the largest finite value (B-DSF16-1), so the saturating infinity is tested separately
		bool satInf = std::isinf(expect) && (absolute ? false : (isMax ? expect < 0 : expect > 0));
		if (!satInf && !(dbl(call1(fnNoIdx, x)) == expect)) badNo++;
	}
	INFO("first bad pattern 0x" << std::hex << firstBad);
	CHECK(bad == 0);
	CHECK(badIdx == 0);
	CHECK(badNo == 0);
}

TEST_CASE("[C3 arm_max_f16] a block of one sample returns that sample at index 0 for every non-NaN pattern (also arm_max_no_idx_f16)") {
	exhaustiveSingle(arm_max_f16, arm_max_no_idx_f16, true, false);
}
TEST_CASE("[C3 arm_min_f16] a block of one sample returns that sample at index 0 for every non-NaN pattern (also arm_min_no_idx_f16)") {
	exhaustiveSingle(arm_min_f16, arm_min_no_idx_f16, false, false);
}
TEST_CASE("[C3 arm_absmax_f16] a block of one sample returns |x| at index 0 for every non-NaN pattern (also arm_absmax_no_idx_f16)") {
	exhaustiveSingle(arm_absmax_f16, arm_absmax_no_idx_f16, true, true);
}
TEST_CASE("[C3 arm_absmin_f16] a block of one sample returns |x| at index 0 for every non-NaN pattern (also arm_absmin_no_idx_f16)") {
	exhaustiveSingle(arm_absmin_f16, arm_absmin_no_idx_f16, false, true);
}

// Strided pair sweep over the whole range of the type (infinities, zeros of both signs, subnormals, largest values)
static void pairSweep(IdxFn fn, Fn1 fnNoIdx, bool isMax, bool absolute) {
	std::vector<F> vals = sweepValues();
	size_t bad = 0, badNo = 0;
	std::string first;
	for (F x : vals) {
		for (F y : vals) {
			std::vector<F> p(2);
			p[0] = x;
			p[1] = y;
			IdxResult r = callIdx(fn, p);
			ExtRef d = defExt(p, isMax, absolute);
			// the first-index rule for equal keys is a characterization (C4 tie tests), so ties are excluded from the index check
			bool tie = absolute ? (std::fabs(dbl(x)) == std::fabs(dbl(y))) : (dbl(x) == dbl(y));
			if (!(dbl(r.value) == d.value) || (!tie && r.index != d.index)) {
				if (bad == 0) {
					char buf[64];
					std::snprintf(buf, sizeof buf, "pair 0x%04X 0x%04X", static_cast<unsigned>(bitsOf(x)), static_cast<unsigned>(bitsOf(y)));
					first = buf;
				}
				bad++;
			}
			bool satInf = std::isinf(d.value) && (absolute ? false : (isMax ? d.value < 0 : d.value > 0));  // B-DSF16-1
			if (!satInf && !(dbl(call1(fnNoIdx, p)) == d.value)) badNo++;
		}
	}
	INFO("first bad: " << first);
	CHECK(bad == 0);
	CHECK(badNo == 0);
}

TEST_CASE("[C3 arm_max_f16] the maximum and index of every pair of a strided full-range sweep equal the double reference (also no_idx)") {
	pairSweep(arm_max_f16, arm_max_no_idx_f16, true, false);
}
TEST_CASE("[C3 arm_min_f16] the minimum and index of every pair of a strided full-range sweep equal the double reference (also no_idx)") {
	pairSweep(arm_min_f16, arm_min_no_idx_f16, false, false);
}
TEST_CASE("[C3 arm_absmax_f16] the |x| maximum and index of every pair of a strided full-range sweep equal the double reference (also no_idx)") {
	pairSweep(arm_absmax_f16, arm_absmax_no_idx_f16, true, true);
}
TEST_CASE("[C3 arm_absmin_f16] the |x| minimum and index of every pair of a strided full-range sweep equal the double reference (also no_idx)") {
	pairSweep(arm_absmin_f16, arm_absmin_no_idx_f16, false, true);
}

TEST_CASE("[C3 arm_max_no_idx_f16] the maximum of {-65504} and of {-65504, -65504, -65504} is -65504 (companion of B-DSF16-1)") {
	CHECK(dbl(call1(arm_max_no_idx_f16, make({-65504.0}))) == -65504.0);
	CHECK(dbl(call1(arm_max_no_idx_f16, make({-65504.0, -65504.0, -65504.0}))) == -65504.0);
}

TEST_CASE("[C3 arm_min_no_idx_f16] the minimum of {65504} and of {65504, 65504, 65504} is 65504 (companion of B-DSF16-2)") {
	CHECK(dbl(call1(arm_min_no_idx_f16, make({65504.0}))) == 65504.0);
	CHECK(dbl(call1(arm_min_no_idx_f16, make({65504.0, 65504.0, 65504.0}))) == 65504.0);
}

TEST_CASE("[C3 arm_max_no_idx_f16] the maximum of {-infinity} is -infinity" *
		  KNOWN_ISSUE("B-DSF16-1: arm_max_no_idx_f16 (arm_max_no_idx_f16.c, scalar path) initialises the running maximum with F16_MIN = -65504 "
			  "and compares with '<', so a block whose maximum is -inf returns -65504; DEF: max of {-inf} = -inf")) {
	std::vector<F> x(1, fromBits(0xFC00));
	CHECK(dbl(call1(arm_max_no_idx_f16, x)) == -kInf);
	std::vector<F> y(3, fromBits(0xFC00));
	CHECK(dbl(call1(arm_max_no_idx_f16, y)) == -kInf);
}

TEST_CASE("[C3 arm_min_no_idx_f16] the minimum of {+infinity} is +infinity" *
		  KNOWN_ISSUE("B-DSF16-2: arm_min_no_idx_f16 (arm_min_no_idx_f16.c, scalar path) initialises the running minimum with F16_MAX = 65504 "
			  "and compares with '>', so a block whose minimum is +inf returns 65504; DEF: min of {+inf} = +inf")) {
	std::vector<F> x(1, fromBits(0x7C00));
	CHECK(dbl(call1(arm_min_no_idx_f16, x)) == kInf);
	std::vector<F> y(3, fromBits(0x7C00));
	CHECK(dbl(call1(arm_min_no_idx_f16, y)) == kInf);
}

TEST_CASE("[C3 arm_absmax_f16] the absolute value of negative zero is positive zero") {
	// DEF: |-0| = +0. DOC: "Maximum value of absolute values". The result bit pattern must be +0 (0x0000)
	std::vector<F> x(1, fromBits(0x8000));
	CHECK(bitsOf(callIdx(arm_absmax_f16, x).value) == 0x0000);
	CHECK(bitsOf(callIdx(arm_absmin_f16, x).value) == 0x0000);
	CHECK(bitsOf(call1(arm_absmax_no_idx_f16, x)) == 0x0000);
	CHECK(bitsOf(call1(arm_absmin_no_idx_f16, x)) == 0x0000);
}

// ---- sums --------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_mean_f16] the mean of random data is within gam(n) * sum|x| / n of the double mean (n = 1 to 33)") {
	// Bound: n-1 additions and one division in f16: n roundings -> gam(n) * sum|x| / n (see header)
	for (size_t n : kSizes) {
		for (uint32_t seed = 0; seed < 4; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> x = randVec(n, static_cast<uint32_t>(n * 5 + seed), -1.0, 3.0);
			double sum = 0;
			for (F v : x) sum += dbl(v);
			CHECK_NEAR_F16(call1(arm_mean_f16, x), sum / static_cast<double>(n), tolMean(x));
		}
	}
}

TEST_CASE("[C3 arm_accumulate_f16] the sum of random data is within gam(n) * sum|x| of the double sum (n = 1 to 33)") {
	for (size_t n : kSizes) {
		for (uint32_t seed = 0; seed < 4; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> x = randVec(n, static_cast<uint32_t>(n * 5 + seed), -1.0, 3.0);
			double sum = 0;
			for (F v : x) sum += dbl(v);
			CHECK_NEAR_F16(call1(arm_accumulate_f16, x), sum, tolAccumulate(x));
		}
	}
}

TEST_CASE("[C3 arm_power_f16] the power (sum of squares) of random data is within gam(n+1) * sum x^2 of the double sum (n = 1 to 33)") {
	for (size_t n : kSizes) {
		for (uint32_t seed = 0; seed < 4; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> x = randVec(n, static_cast<uint32_t>(n * 5 + seed), -2.0, 2.0);
			double s2 = 0;
			for (F v : x) s2 += dbl(v) * dbl(v);
			CHECK_NEAR_F16(call1(arm_power_f16, x), s2, tolPower(x));
		}
	}
}

TEST_CASE("[C3 arm_rms_f16] the rms of random data is inside the interval derived from the f16 accumulation (n = 1 to 33)") {
	for (size_t n : kSizes) {
		for (uint32_t seed = 0; seed < 4; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> x = randVec(n, static_cast<uint32_t>(n * 5 + seed), -2.0, 2.0);
			double exact;
			Range r = rangeRms(x, &exact);
			F out = call1(arm_rms_f16, x);
			INFO("out " << dbl(out) << " exact " << exact << " range [" << r.lo << ", " << r.hi << "]");
			CHECK(inRange(dbl(out), r));
		}
	}
}

TEST_CASE("[C3 arm_var_f16] the unbiased variance (n-1) of random data is inside the interval derived from the f16 accumulation (n = 2 to 33)") {
	for (size_t n : kSizes) {
		if (n < 2) continue;
		for (uint32_t seed = 0; seed < 4; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> x = randVec(n, static_cast<uint32_t>(n * 5 + seed), -2.0, 2.0);
			double exact;
			Range r = rangeVar(x, &exact);
			F out = call1(arm_var_f16, x);
			INFO("out " << dbl(out) << " exact " << exact << " range [" << r.lo << ", " << r.hi << "]");
			CHECK(inRange(dbl(out), r));
		}
	}
}

TEST_CASE("[C3 arm_std_f16] the standard deviation of random data is inside the interval derived from the f16 accumulation (n = 2 to 33)") {
	for (size_t n : kSizes) {
		if (n < 2) continue;
		for (uint32_t seed = 0; seed < 4; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> x = randVec(n, static_cast<uint32_t>(n * 5 + seed), -2.0, 2.0);
			double exact;
			Range r = rangeStd(x, &exact);
			F out = call1(arm_std_f16, x);
			INFO("out " << dbl(out) << " exact " << exact << " range [" << r.lo << ", " << r.hi << "]");
			CHECK(inRange(dbl(out), r));
		}
	}
}

TEST_CASE("[C3 arm_mse_f16] the mean square error of random data is within gam(n+4) * sum (a-b)^2 / n of the double value (n = 1 to 33)") {
	// 8-fold unrolled in the library: sizes 8, 9, 16, 17, 33 cross the unroll boundaries
	for (size_t n : kSizes) {
		for (uint32_t seed = 0; seed < 4; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> a = randVec(n, static_cast<uint32_t>(n * 5 + seed), -2.0, 2.0);
			std::vector<F> b = randVec(n, static_cast<uint32_t>(n * 5 + seed + 1000), -2.0, 2.0);
			double exact;
			double tol = tolMse(a, b, &exact);
			CHECK_NEAR_F16(call2(arm_mse_f16, a, b), exact, tol);
		}
	}
}

TEST_CASE("[C3 arm_entropy_f16] the entropy of random probabilities is within the derived bound of -sum p ln p (n = 1 to 33)") {
	for (size_t n : kSizes) {
		for (uint32_t seed = 0; seed < 4; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> p = randVec(n, static_cast<uint32_t>(n * 5 + seed), 0.02, 1.0);
			double exact;
			double tol = tolEntropy(p, &exact);
			F out = arm_entropy_f16(p.data(), static_cast<uint32_t>(n));
			CHECK_NEAR_F16(out, exact, tol);
		}
	}
}

TEST_CASE("[C3 arm_kullback_leibler_f16] the divergence of random distributions is within the derived bound of -sum pA ln(pB/pA) (n = 1 to 33)") {
	for (size_t n : kSizes) {
		for (uint32_t seed = 0; seed < 4; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> a = randVec(n, static_cast<uint32_t>(n * 5 + seed), 0.05, 1.0);
			std::vector<F> b = randVec(n, static_cast<uint32_t>(n * 5 + seed + 1000), 0.05, 1.0);
			double exact;
			double tol = tolKl(a, b, &exact);
			F out = arm_kullback_leibler_f16(a.data(), b.data(), static_cast<uint32_t>(n));
			CHECK_NEAR_F16(out, exact, tol);
		}
	}
}

TEST_CASE("[C3 arm_logsumexp_f16] logsumexp of random data is within the derived bound of ln(sum exp(x)) (n = 1 to 33)") {
	for (size_t n : kSizes) {
		for (uint32_t seed = 0; seed < 4; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> x = randVec(n, static_cast<uint32_t>(n * 5 + seed), -5.0, 5.0);
			double exact;
			double tol = tolLse(x, &exact);
			F out = arm_logsumexp_f16(x.data(), static_cast<uint32_t>(n));
			CHECK_NEAR_F16(out, exact, tol);
		}
	}
}

TEST_CASE("[C3 arm_logsumexp_dot_prod_f16] the result is within the derived bound of ln(sum exp(a_i + b_i)) (n = 1 to 33)") {
	// the f16 sums t = fl(a+b) differ from the exact sums by <= u*|a+b|; logsumexp is 1-Lipschitz in the max norm, so the
	// bound of logsumexp(t) is widened by u * max|a+b|
	for (size_t n : kSizes) {
		for (uint32_t seed = 0; seed < 4; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> a = randVec(n, static_cast<uint32_t>(n * 5 + seed), -3.0, 3.0);
			std::vector<F> b = randVec(n, static_cast<uint32_t>(n * 5 + seed + 1000), -3.0, 3.0);
			std::vector<F> t(n);
			std::vector<double> s(n);
			double smax = 0;
			for (size_t i = 0; i < n; i++) {
				s[i] = dbl(a[i]) + dbl(b[i]);
				t[i] = h16(s[i]);
				smax = std::max(smax, std::fabs(s[i]));
			}
			double m = *std::max_element(s.begin(), s.end()), acc = 0;
			for (size_t i = 0; i < n; i++) acc += std::exp(s[i] - m);
			double exact = m + std::log(acc), ignored;
			double tol = tolLse(t, &ignored) + kU * smax;
			dsp::GuardedBuffer<F> tmp(n);
			F out = arm_logsumexp_dot_prod_f16(a.data(), b.data(), static_cast<uint32_t>(n), tmp.data());
			CHECK_NEAR_F16(out, exact, tol);
		}
	}
}

// Exhaustive single-sample domain of the sums: every non-NaN pattern as a block of one sample
TEST_CASE("[C3 arm_mean_f16] the mean of one sample is that sample for every non-NaN pattern (also arm_accumulate_f16)") {
	// mean: fl(0 + x) / 1 = x; accumulate: fl(0 + x) = x. -0 gives +0 (0 + -0 = +0), equal in value.
	size_t bad = 0, badAcc = 0;
	for (uint32_t b = 0; b < 65536u; b++) {
		if (isNan16(b)) continue;
		std::vector<F> x(1, fromBits(static_cast<uint16_t>(b)));
		if (!(dbl(call1(arm_mean_f16, x)) == dbl(x[0]))) bad++;
		if (!(dbl(call1(arm_accumulate_f16, x)) == dbl(x[0]))) badAcc++;
	}
	CHECK(bad == 0);
	CHECK(badAcc == 0);
}

TEST_CASE("[C3 arm_power_f16] the power of one sample is x*x rounded once to binary16 for every non-NaN pattern") {
	// x has 11 significant bits, so x*x (22 bits) is exact in double; the library rounds once (0 + fl(x*x)).
	// DEF: the correctly rounded conversion of the exact square (overflow -> +inf).
	size_t bad = 0;
	uint32_t first = 0xFFFFFFFFu;
	for (uint32_t b = 0; b < 65536u; b++) {
		if (isNan16(b)) continue;
		std::vector<F> x(1, fromBits(static_cast<uint16_t>(b)));
		double expect = dbl(h16(dbl(x[0]) * dbl(x[0])));
		if (!(dbl(call1(arm_power_f16, x)) == expect)) {
			bad++;
			if (first == 0xFFFFFFFFu) first = b;
		}
	}
	INFO("first bad pattern 0x" << std::hex << first);
	CHECK(bad == 0);
}

TEST_CASE("[C3 arm_logsumexp_f16] logsumexp of one finite sample is that sample for every finite pattern") {
	// DEF: x + ln(exp(x - x)) = x + ln(1) = x
	size_t bad = 0;
	uint32_t first = 0xFFFFFFFFu;
	for (uint32_t b = 0; b < 65536u; b++) {
		if (isNan16(b) || (b & 0x7C00u) == 0x7C00u) continue;
		F x = fromBits(static_cast<uint16_t>(b));
		F r = arm_logsumexp_f16(&x, 1);
		if (!(dbl(r) == dbl(x))) {
			bad++;
			if (first == 0xFFFFFFFFu) first = b;
		}
	}
	INFO("first bad pattern 0x" << std::hex << first);
	CHECK(bad == 0);
}

TEST_CASE("[C3 arm_entropy_f16] the entropy of one probability p is within 2u of -p ln p for every positive finite p up to 100") {
	// one product p * fl(ln p): (1+u)^2 plus the underflow term; the sum 0 + t is exact. Largest |p ln p| = 100 * 4.6 < 65504.
	size_t bad = 0;
	uint32_t first = 0xFFFFFFFFu;
	for (uint32_t b = 1; b < 0x5640u; b++) {  // 0x5640 = 100.0
		F p = fromBits(static_cast<uint16_t>(b));
		double exact = -dbl(p) * std::log(dbl(p));
		double tol = 2.01 * kU * std::fabs(exact) + 2.0 * kSub + 2.0e-7 * dbl(p);
		F r = arm_entropy_f16(&p, 1);
		if (!within(dbl(r), exact, tol)) {
			bad++;
			if (first == 0xFFFFFFFFu) first = b;
		}
	}
	INFO("first bad pattern 0x" << std::hex << first);
	CHECK(bad == 0);
}

TEST_CASE("[C3 arm_mse_f16] the mean square error of one sample pair is within 4u of (a-b)^2 on a strided full-range sweep") {
	// fl(a-b) (u), fl(d*d) (1+u)^2 -> three roundings; 0 + t exact; /1 exact. Restricted to |a-b| < 250 (no overflow).
	std::vector<F> vals = sweepValues();
	size_t bad = 0, tested = 0;
	for (F a : vals) {
		for (F b : vals) {
			double d = dbl(a) - dbl(b);
			if (!(std::fabs(d) < 250.0)) continue;
			tested++;
			std::vector<F> x(1, a), y(1, b);
			double exact = d * d;
			double tol = 3.05 * kU * exact + 2.0 * kSub;
			if (!within(dbl(call2(arm_mse_f16, x, y)), exact, tol)) bad++;
		}
	}
	CHECK(tested > 1000);
	CHECK(bad == 0);
}

// ===============================================================================================================
// C4: block sizes 1 and 2, equal data, hand-derived exact results, documented edge behaviour
// ===============================================================================================================

TEST_CASE("[C4 arm_mean_f16] hand-derived: the mean of 1..9 is exactly 5 and of {1,2} exactly 1.5") {
	// 1+..+9 = 45 (all partial sums are integers below 2048, exact in f16), 45 / 9 = 5 exactly; {1,2}: 3/2 = 1.5 exactly
	CHECK(dbl(call1(arm_mean_f16, make({1, 2, 3, 4, 5, 6, 7, 8, 9}))) == 5.0);
	CHECK(dbl(call1(arm_mean_f16, make({1, 2}))) == 1.5);
}

TEST_CASE("[C4 arm_accumulate_f16] hand-derived: the sum of 1..9 is exactly 45 and of {1,2} exactly 3") {
	CHECK(dbl(call1(arm_accumulate_f16, make({1, 2, 3, 4, 5, 6, 7, 8, 9}))) == 45.0);
	CHECK(dbl(call1(arm_accumulate_f16, make({1, 2}))) == 3.0);
}

TEST_CASE("[C4 arm_power_f16] hand-derived: the power of 1..9 is exactly 285 and of {3,4} exactly 25") {
	// 1+4+9+...+81 = 285 (integers, exact); 9 + 16 = 25
	CHECK(dbl(call1(arm_power_f16, make({1, 2, 3, 4, 5, 6, 7, 8, 9}))) == 285.0);
	CHECK(dbl(call1(arm_power_f16, make({3, 4}))) == 25.0);
}

TEST_CASE("[C4 arm_rms_f16] hand-derived: the rms of {3,4} is sqrt(12.5) within one binary16 rounding and of {-5,5,-5,5} exactly 5") {
	// {3,4}: (9+16)/2 = 12.5 exact, sqrt = 3.5355339; the f16 result is the nearest binary16 value (spacing 2^-9 in [2,4)),
	// error <= u * 3.54 = 1.7e-3. {-5,5,-5,5}: sum 100, /4 = 25, sqrt = 5 exactly (sign of the data irrelevant).
	CHECK_NEAR_F16(call1(arm_rms_f16, make({3, 4})), std::sqrt(12.5), kU * 3.54);
	CHECK(dbl(call1(arm_rms_f16, make({-5, 5, -5, 5}))) == 5.0);
}

TEST_CASE("[C4 arm_var_f16] hand-derived: the unbiased variance of 1..9 is exactly 7.5 and of {0,2} exactly 2") {
	// 1..9: mean 5, squares of the deviations 16+9+4+1+0+1+4+9+16 = 60, / (n-1 = 8) = 7.5 (exact in f16).
	// {0,2}: mean 1, deviations +-1, sum 2, / (n-1 = 1) = 2. The population variance (/n) would be 6.667 and 1.
	CHECK(dbl(call1(arm_var_f16, make({1, 2, 3, 4, 5, 6, 7, 8, 9}))) == 7.5);
	CHECK(dbl(call1(arm_var_f16, make({0, 2}))) == 2.0);
}

TEST_CASE("[C4 arm_std_f16] hand-derived: the standard deviation of 1..9 is sqrt(7.5) and of {0,2} is sqrt(2) within one rounding") {
	// var 7.5 and 2 are exact (see arm_var_f16), sqrt in float then one cast to f16: error <= u * result
	CHECK_NEAR_F16(call1(arm_std_f16, make({1, 2, 3, 4, 5, 6, 7, 8, 9})), std::sqrt(7.5), kU * 2.74);
	CHECK_NEAR_F16(call1(arm_std_f16, make({0, 2})), std::sqrt(2.0), kU * 1.42);
}

TEST_CASE("[C4 arm_mse_f16] hand-derived: the mse of {1..8} and {2,4,..,16} is exactly 25.5 and of {1,2} and {3,5} exactly 6.5") {
	// differences -1..-8, squares 1+4+9+16+25+36+49+64 = 204, / 8 = 25.5 (exact). {1,2}-{3,5}: 4 + 9 = 13, / 2 = 6.5
	CHECK(dbl(call2(arm_mse_f16, make({1, 2, 3, 4, 5, 6, 7, 8}), make({2, 4, 6, 8, 10, 12, 14, 16}))) == 25.5);
	CHECK(dbl(call2(arm_mse_f16, make({1, 2}), make({3, 5}))) == 6.5);
}

TEST_CASE("[C4 arm_mse_f16] the mse of two equal vectors is exactly 0") {
	std::vector<F> x = randVec(17, 3, -2.0, 2.0);
	CHECK(dbl(call2(arm_mse_f16, x, x)) == 0.0);
}

TEST_CASE("[C4 arm_mean_f16] constant data: the mean, rms, power and accumulate of n copies of 3 are exact for n = 1 to 100") {
	// n * 3 <= 300 and n * 9 <= 900 are integers below 2048 (exact partial sums); 3n / n = 3, 9n / n = 9, sqrt(9) = 3
	for (size_t n : {1u, 2u, 3u, 4u, 5u, 8u, 9u, 33u, 100u}) {
		INFO("block size " << n);
		std::vector<F> x(n, h16(3.0));
		CHECK(dbl(call1(arm_mean_f16, x)) == 3.0);
		CHECK(dbl(call1(arm_rms_f16, x)) == 3.0);
		CHECK(dbl(call1(arm_power_f16, x)) == 9.0 * static_cast<double>(n));
		CHECK(dbl(call1(arm_accumulate_f16, x)) == 3.0 * static_cast<double>(n));
	}
}

TEST_CASE("[C4 arm_var_f16] constant data: the variance and the standard deviation of n >= 2 copies of 3 are exactly 0") {
	// the mean is exactly 3 (see above), all deviations are 0
	for (size_t n : {2u, 3u, 4u, 5u, 8u, 9u, 33u, 100u}) {
		INFO("block size " << n);
		std::vector<F> x(n, h16(3.0));
		CHECK(dbl(call1(arm_var_f16, x)) == 0.0);
		CHECK(dbl(call1(arm_std_f16, x)) == 0.0);
	}
}

TEST_CASE("[C4 arm_var_f16] characterization: the variance and standard deviation of 0 or 1 sample are 0 (the unbiased estimator would divide by zero)") {
	// Source arm_var_f16.c: "if (blockSize <= 1U) { *pResult = 0; return; }". DOC is silent. 0 is the only finite sensible
	// value; recorded as characterization (NEEDS REVIEW). For blockSize 0 the pointer is never dereferenced.
	F x = h16(3.5);
	F r = fromBits(kSentinelBits);
	arm_var_f16(&x, 1, &r);
	CHECK(dbl(r) == 0.0);
	r = fromBits(kSentinelBits);
	arm_std_f16(&x, 1, &r);
	CHECK(dbl(r) == 0.0);
	r = fromBits(kSentinelBits);
	arm_var_f16(&x, 0, &r);
	CHECK(dbl(r) == 0.0);
	r = fromBits(kSentinelBits);
	arm_std_f16(&x, 0, &r);
	CHECK(dbl(r) == 0.0);
}

TEST_CASE("[C4 arm_max_f16] a block of two returns the larger element and its index") {
	CHECK(dbl(callIdx(arm_max_f16, make({1, 2})).value) == 2.0);
	CHECK(callIdx(arm_max_f16, make({1, 2})).index == 1u);
	CHECK(dbl(callIdx(arm_max_f16, make({2, 1})).value) == 2.0);
	CHECK(callIdx(arm_max_f16, make({2, 1})).index == 0u);
}

TEST_CASE("[C4 arm_min_f16] a block of two returns the smaller element and its index") {
	CHECK(dbl(callIdx(arm_min_f16, make({1, 2})).value) == 1.0);
	CHECK(callIdx(arm_min_f16, make({1, 2})).index == 0u);
	CHECK(dbl(callIdx(arm_min_f16, make({2, 1})).value) == 1.0);
	CHECK(callIdx(arm_min_f16, make({2, 1})).index == 1u);
}

TEST_CASE("[C4 arm_absmax_f16] the maximum of |x| of {-3, 2} is 3 (positive) at index 0, of {2, -3} at index 1") {
	CHECK(dbl(callIdx(arm_absmax_f16, make({-3, 2})).value) == 3.0);
	CHECK(callIdx(arm_absmax_f16, make({-3, 2})).index == 0u);
	CHECK(dbl(callIdx(arm_absmax_f16, make({2, -3})).value) == 3.0);
	CHECK(callIdx(arm_absmax_f16, make({2, -3})).index == 1u);
}

TEST_CASE("[C4 arm_absmin_f16] the minimum of |x| of {-1, 2} is 1 (positive) at index 0, of {2, -1} at index 1") {
	CHECK(dbl(callIdx(arm_absmin_f16, make({-1, 2})).value) == 1.0);
	CHECK(callIdx(arm_absmin_f16, make({-1, 2})).index == 0u);
	CHECK(dbl(callIdx(arm_absmin_f16, make({2, -1})).value) == 1.0);
	CHECK(callIdx(arm_absmin_f16, make({2, -1})).index == 1u);
}

// Ties: DOC does not say which index is returned. The library uses a strict comparison, so the FIRST of equal extrema wins.
// Checked at every position of the unroll (blocks of 4) for all sizes up to 13.
static void tieTest(IdxFn fn, bool isMax, bool absolute) {
	for (size_t n = 2; n <= 13; n++) {
		for (size_t p = 0; p + 1 < n; p++) {
			for (size_t q = p + 1; q < n; q++) {
				// background strictly worse than the extremum: for max/min of the key, 1 vs 9 (abs: use -1 and 9/-9)
				std::vector<F> x(n, h16(isMax ? 1.0 : 9.0));
				double ext = isMax ? 9.0 : 1.0;
				x[p] = h16(ext);
				// second equal extremum; for the absolute variants use the opposite sign
				x[q] = h16(absolute ? -ext : ext);
				INFO("n " << n << " first " << p << " second " << q);
				IdxResult r = callIdx(fn, x);
				CHECK(dbl(r.value) == ext);
				CHECK(r.index == static_cast<uint32_t>(p));
			}
		}
	}
}

TEST_CASE("[C4 arm_max_f16] characterization: of equal maxima the first index is returned (all positions up to 13 samples)") { tieTest(arm_max_f16, true, false); }
TEST_CASE("[C4 arm_min_f16] characterization: of equal minima the first index is returned (all positions up to 13 samples)") { tieTest(arm_min_f16, false, false); }
TEST_CASE("[C4 arm_absmax_f16] characterization: of equal maxima of |x| (also of opposite sign) the first index is returned (up to 13 samples)") {
	tieTest(arm_absmax_f16, true, true);
}
TEST_CASE("[C4 arm_absmin_f16] characterization: of equal minima of |x| (also of opposite sign) the first index is returned (up to 13 samples)") {
	tieTest(arm_absmin_f16, false, true);
}

TEST_CASE("[C4 arm_max_f16] all-equal data: the value is that value and the index is 0") {
	std::vector<F> x(9, h16(2.5));
	IdxResult r = callIdx(arm_max_f16, x);
	CHECK(dbl(r.value) == 2.5);
	CHECK(r.index == 0u);
	r = callIdx(arm_min_f16, x);
	CHECK(dbl(r.value) == 2.5);
	CHECK(r.index == 0u);
	CHECK(dbl(call1(arm_max_no_idx_f16, x)) == 2.5);
	CHECK(dbl(call1(arm_min_no_idx_f16, x)) == 2.5);
	CHECK(dbl(call1(arm_absmax_no_idx_f16, x)) == 2.5);
	CHECK(dbl(call1(arm_absmin_no_idx_f16, x)) == 2.5);
}

TEST_CASE("[C4 arm_max_f16] the extremes of the type: +-65504 and +-infinity are handled") {
	std::vector<F> x = make({0.0, 65504.0, -65504.0, 1.0});
	CHECK(dbl(callIdx(arm_max_f16, x).value) == 65504.0);
	CHECK(callIdx(arm_max_f16, x).index == 1u);
	CHECK(dbl(callIdx(arm_min_f16, x).value) == -65504.0);
	CHECK(callIdx(arm_min_f16, x).index == 2u);
	std::vector<F> y = make({1.0, kInf, -kInf, 0.0});
	CHECK(dbl(callIdx(arm_max_f16, y).value) == kInf);
	CHECK(callIdx(arm_max_f16, y).index == 1u);
	CHECK(dbl(callIdx(arm_min_f16, y).value) == -kInf);
	CHECK(callIdx(arm_min_f16, y).index == 2u);
	CHECK(dbl(callIdx(arm_absmax_f16, make({1.0, -kInf, 2.0})).value) == kInf);
	CHECK(callIdx(arm_absmax_f16, make({1.0, -kInf, 2.0})).index == 1u);
	CHECK(dbl(call1(arm_absmax_no_idx_f16, make({1.0, -kInf, 2.0}))) == kInf);
}

TEST_CASE("[C4 arm_entropy_f16] hand-derived: the entropy of {0.5, 0.5} is ln 2 and of four times 0.25 is ln 4 within the derived bound") {
	// -(2 * 0.5 * ln 0.5) = ln 2 = 0.693147; -(4 * 0.25 * ln 0.25) = ln 4 = 1.386294
	std::vector<F> a = make({0.5, 0.5}), b = make({0.25, 0.25, 0.25, 0.25});
	double e;
	double ta = tolEntropy(a, &e);
	CHECK_NEAR_F16(arm_entropy_f16(a.data(), 2), std::log(2.0), ta);
	double tb = tolEntropy(b, &e);
	CHECK_NEAR_F16(arm_entropy_f16(b.data(), 4), std::log(4.0), tb);
}

TEST_CASE("[C4 arm_entropy_f16] a certain event (p = 1) has entropy 0") {
	// -(1 * ln 1) = 0, and ln 1 = 0 exactly in libm
	F p = h16(1.0);
	CHECK(dbl(arm_entropy_f16(&p, 1)) == 0.0);
}

TEST_CASE("[C4 arm_kullback_leibler_f16] the divergence of a distribution from itself is exactly 0") {
	// D(A||A) = -sum pA ln(pA/pA) = -sum pA * ln 1 = 0, ln 1 = 0 exactly
	std::vector<F> a = randVec(9, 5, 0.05, 1.0);
	CHECK(dbl(arm_kullback_leibler_f16(a.data(), a.data(), 9)) == 0.0);
}

TEST_CASE("[C4 arm_kullback_leibler_f16] hand-derived: D({0.5,0.5} || {0.25,0.75}) = 0.5 ln(0.5) + 0.5 ln(0.5/0.75)... = 0.5*ln(2) + 0.5*ln(2/3)") {
	// D = -sum pA ln(pB/pA) = -(0.5 ln(0.25/0.5) + 0.5 ln(0.75/0.5)) = -(0.5 * (-0.693147) + 0.5 * 0.405465) = 0.143841
	std::vector<F> a = make({0.5, 0.5}), b = make({0.25, 0.75});
	double e;
	double tol = tolKl(a, b, &e);
	CHECK(within(e, 0.5 * std::log(2.0) - 0.5 * std::log(1.5), 1e-12));
	CHECK_NEAR_F16(arm_kullback_leibler_f16(a.data(), b.data(), 2), 0.14384103622589045, tol);
}

TEST_CASE("[C4 arm_logsumexp_f16] hand-derived: logsumexp of two equal values c is c + ln 2 within the derived bound") {
	// ln(e^c + e^c) = c + ln 2. c = 0: 0.693147; c = 1000: exp(1000) overflows binary16 and float, so only the
	// max-shifted algorithm gives 1000.69 (f16 spacing at 1000 is 0.5)
	for (double c : {0.0, 1.0, -3.0, 1000.0}) {
		INFO("c " << c);
		std::vector<F> x(2, h16(c));
		double exact;
		double tol = tolLse(x, &exact);
		CHECK(within(exact, c + std::log(2.0), 1e-9));
		CHECK_NEAR_F16(arm_logsumexp_f16(x.data(), 2), c + std::log(2.0), tol);
	}
}

TEST_CASE("[C4 arm_logsumexp_f16] a very negative value among large ones does not change the result (no underflow to -inf, no NaN)") {
	// {-60, 10}: exp(-70) is far below the binary16 range (flushes to 0), the result is 10 + ln(1 + 0) = 10 exactly
	std::vector<F> x = make({-60, 10});
	CHECK(dbl(arm_logsumexp_f16(x.data(), 2)) == 10.0);
}

TEST_CASE("[C4 arm_logsumexp_dot_prod_f16] hand-derived: a = {0, 0}, b = {0, 0} gives ln 2") {
	std::vector<F> a = make({0, 0}), b = make({0, 0});
	dsp::GuardedBuffer<F> tmp(2);
	F r = arm_logsumexp_dot_prod_f16(a.data(), b.data(), 2, tmp.data());
	std::vector<F> x = make({0, 0});
	double e;
	double tol = tolLse(x, &e);
	CHECK_NEAR_F16(r, std::log(2.0), tol);
	CHECK(tmp.guardIntact());
}

TEST_CASE("[C4 arm_entropy_f16] characterization: a zero probability gives NaN because 0*logf(0) = 0*(-inf) (DOC is silent about 0 ln 0)") {
	// NEEDS REVIEW: the usual convention is 0 ln 0 = 0
	std::vector<F> p = make({0.5, 0.0, 0.5});
	CHECK(std::isnan(dbl(arm_entropy_f16(p.data(), 3))));
}

TEST_CASE("[C4 arm_kullback_leibler_f16] characterization: a zero in distribution A gives NaN on the portable path") {
	// pA * logf(pB / pA) = 0 * logf(inf) = 0 * inf = NaN. NEEDS REVIEW
	std::vector<F> a = make({0.5, 0.0, 0.5}), b = make({0.3, 0.4, 0.3});
	CHECK(std::isnan(dbl(arm_kullback_leibler_f16(a.data(), b.data(), 3))));
}

TEST_CASE("[C4 arm_accumulate_f16] characterization: the sum 40000 + 40000 exceeds the binary16 range and is +infinity (IEEE overflow)") {
	// DOC is silent about overflow. binary16 round-to-nearest overflows to +inf (IEEE 754): 80000 > 65504 + half ulp
	CHECK(dbl(call1(arm_accumulate_f16, make({40000, 40000}))) == kInf);
	CHECK(dbl(call1(arm_accumulate_f16, make({-40000, -40000}))) == -kInf);
}

TEST_CASE("[C4 arm_power_f16] characterization: the power of {300} exceeds the binary16 range and is +infinity (IEEE overflow)") {
	// 300^2 = 90000 > 65520
	CHECK(dbl(call1(arm_power_f16, make({300}))) == kInf);
}


TEST_CASE("[C4 arm_logsumexp_f16] logsumexp of {-inf, 0} is 0 (companion of B-DSF16-3)") {
	// max = 0, exp(-inf - 0) = 0, 0 + ln(1) = 0
	std::vector<F> x = make({-kInf, 0.0});
	CHECK(dbl(arm_logsumexp_f16(x.data(), 2)) == 0.0);
}

TEST_CASE("[C4 arm_logsumexp_f16] logsumexp of {-inf, -inf} is -inf" *
		  KNOWN_ISSUE("B-DSF16-3: arm_logsumexp_f16.c: the maximum is -inf, so the argument of expf is -inf - -inf = NaN and the result is NaN (math: -inf)")) {
	std::vector<F> x = make({-kInf, -kInf});
	CHECK(dbl(arm_logsumexp_f16(x.data(), 2)) == -kInf);
}

TEST_CASE("[C4 arm_logsumexp_f16] logsumexp of {+inf} and of {+inf, +inf} is +inf" *
		  KNOWN_ISSUE("B-DSF16-3: arm_logsumexp_f16.c: the maximum is +inf, so the argument of expf is inf - inf = NaN and the result is NaN (math: +inf)")) {
	std::vector<F> x = make({kInf});
	CHECK(dbl(arm_logsumexp_f16(x.data(), 1)) == kInf);
	std::vector<F> y = make({kInf, kInf});
	CHECK(dbl(arm_logsumexp_f16(y.data(), 2)) == kInf);
}

TEST_CASE("[C4 arm_var_f16] the variance of {1, NaN, 3} is NaN (companion of B-DSF16-4 and B-DSF16-5)") {
	std::vector<F> x = make({1.0, std::numeric_limits<double>::quiet_NaN(), 3.0});
	CHECK(std::isnan(dbl(call1(arm_var_f16, x))));
}

TEST_CASE("[C4 arm_std_f16] the standard deviation of {1, NaN, 3} is NaN" *
		  KNOWN_ISSUE("B-DSF16-4: arm_std_f16.c:61 passes the NaN variance to arm_sqrt_f16 -> arm_sqrt_f32, whose guard 'in >= 0.0f' fails and writes 0, so std is 0 where var is NaN")) {
	std::vector<F> x = make({1.0, std::numeric_limits<double>::quiet_NaN(), 3.0});
	CHECK(std::isnan(dbl(call1(arm_std_f16, x))));
}

TEST_CASE("[C4 arm_rms_f16] the rms of {1, NaN, 3} is NaN" *
		  KNOWN_ISSUE("B-DSF16-5: arm_rms_f16.c:129 passes the NaN mean square to arm_sqrt_f16 -> arm_sqrt_f32, whose guard 'in >= 0.0f' fails and writes 0, so rms is 0 for NaN input")) {
	std::vector<F> x = make({1.0, std::numeric_limits<double>::quiet_NaN(), 3.0});
	CHECK(std::isnan(dbl(call1(arm_rms_f16, x))));
}

// ===============================================================================================================
// C5: agreement with the f32 function of the same input (the f16 values are converted to float exactly)
// ===============================================================================================================

// extrema: both types see the same values (f16 -> f32 conversion is exact), so value and index are identical, ties
// included (heavy ties: values from a set of 5 numbers)
static std::vector<F> tieHeavy(size_t n, uint32_t seed) {
	Rng rng(seed);
	std::vector<F> v(n);
	for (size_t i = 0; i < n; i++) {
		uint32_t k = rng.below(5);
		v[i] = h16(static_cast<double>(k) - 2.0);
	}
	return v;
}

typedef void (*IdxFn32)(const float32_t *, uint32_t, float32_t *, uint32_t *);
typedef void (*Fn32)(const float32_t *, uint32_t, float32_t *);

static void c5Idx(IdxFn f16fn, IdxFn32 f32fn) {
	for (size_t n = 1; n <= 40; n++) {
		for (uint32_t seed = 0; seed < 5; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> x = tieHeavy(n, static_cast<uint32_t>(n * 13 + seed));
			std::vector<float> xf = toFloat(x);
			IdxResult r = callIdx(f16fn, x);
			float32_t v = 0;
			uint32_t idx = kSentinelIdx;
			f32fn(xf.data(), static_cast<uint32_t>(n), &v, &idx);
			CHECK(dbl(r.value) == static_cast<double>(v));
			CHECK(r.index == idx);
		}
	}
}

static void c5NoIdx(Fn1 f16fn, Fn32 f32fn) {
	for (size_t n = 1; n <= 40; n++) {
		for (uint32_t seed = 0; seed < 5; seed++) {
			INFO("block size " << n << " seed " << seed);
			std::vector<F> x = tieHeavy(n, static_cast<uint32_t>(n * 13 + seed));
			std::vector<float> xf = toFloat(x);
			float32_t v = 0;
			f32fn(xf.data(), static_cast<uint32_t>(n), &v);
			CHECK(dbl(call1(f16fn, x)) == static_cast<double>(v));
		}
	}
}

TEST_CASE("[C5 arm_max_f16] value and index equal those of arm_max_f32 on tie-heavy data (1 to 40 samples)") { c5Idx(arm_max_f16, arm_max_f32); }
TEST_CASE("[C5 arm_min_f16] value and index equal those of arm_min_f32 on tie-heavy data (1 to 40 samples)") { c5Idx(arm_min_f16, arm_min_f32); }
TEST_CASE("[C5 arm_absmax_f16] value and index equal those of arm_absmax_f32 on tie-heavy data (1 to 40 samples)") { c5Idx(arm_absmax_f16, arm_absmax_f32); }
TEST_CASE("[C5 arm_absmin_f16] value and index equal those of arm_absmin_f32 on tie-heavy data (1 to 40 samples)") { c5Idx(arm_absmin_f16, arm_absmin_f32); }
TEST_CASE("[C5 arm_max_no_idx_f16] the value equals that of arm_max_no_idx_f32 on tie-heavy data (1 to 40 samples)") { c5NoIdx(arm_max_no_idx_f16, arm_max_no_idx_f32); }
TEST_CASE("[C5 arm_min_no_idx_f16] the value equals that of arm_min_no_idx_f32 on tie-heavy data (1 to 40 samples)") { c5NoIdx(arm_min_no_idx_f16, arm_min_no_idx_f32); }
TEST_CASE("[C5 arm_absmax_no_idx_f16] the value equals that of arm_absmax_no_idx_f32 on tie-heavy data (1 to 40 samples)") {
	c5NoIdx(arm_absmax_no_idx_f16, arm_absmax_no_idx_f32);
}
TEST_CASE("[C5 arm_absmin_no_idx_f16] the value equals that of arm_absmin_no_idx_f32 on tie-heavy data (1 to 40 samples)") {
	c5NoIdx(arm_absmin_no_idx_f16, arm_absmin_no_idx_f32);
}

// Sums: the f32 library result (f32 accumulation, relative error <= gam32(n) ~ n * 6e-8 <= 2e-6 for n <= 33 on the
// absolute sum) is within 1e-5 * (sum |terms|) of the exact value; the f16 result is within the tolerance of the C3
// test. They therefore differ by at most tol + 1e-5 * scale, scale = 1 + |f32 result|.
#define CHECK_VS_F32(out, r32, tol) \
	do { \
		INFO("f16 " << dbl(out) << " f32 " << (r32) << " tol " << (tol)); \
		CHECK(within(dbl(out), (r32), (tol) + 1.0e-5 * (1.0 + std::fabs(r32)))); \
	} while (0)

TEST_CASE("[C5 arm_mean_f16] the f16 mean agrees with arm_mean_f32 of the same input within the f16 accumulation bound") {
	for (size_t n : kSizes) {
		INFO("block size " << n);
		std::vector<F> x = randVec(n, static_cast<uint32_t>(n + 70), -1.0, 3.0);
		std::vector<float> xf = toFloat(x);
		float32_t r32 = 0;
		arm_mean_f32(xf.data(), static_cast<uint32_t>(n), &r32);
		CHECK_VS_F32(call1(arm_mean_f16, x), static_cast<double>(r32), tolMean(x));
	}
}

TEST_CASE("[C5 arm_accumulate_f16] the f16 sum agrees with arm_accumulate_f32 of the same input within the f16 accumulation bound") {
	for (size_t n : kSizes) {
		INFO("block size " << n);
		std::vector<F> x = randVec(n, static_cast<uint32_t>(n + 70), -1.0, 3.0);
		std::vector<float> xf = toFloat(x);
		float32_t r32 = 0;
		arm_accumulate_f32(xf.data(), static_cast<uint32_t>(n), &r32);
		CHECK_VS_F32(call1(arm_accumulate_f16, x), static_cast<double>(r32), tolAccumulate(x));
	}
}

TEST_CASE("[C5 arm_power_f16] the f16 power agrees with arm_power_f32 of the same input within the f16 accumulation bound") {
	for (size_t n : kSizes) {
		INFO("block size " << n);
		std::vector<F> x = randVec(n, static_cast<uint32_t>(n + 70), -2.0, 2.0);
		std::vector<float> xf = toFloat(x);
		float32_t r32 = 0;
		arm_power_f32(xf.data(), static_cast<uint32_t>(n), &r32);
		CHECK_VS_F32(call1(arm_power_f16, x), static_cast<double>(r32), tolPower(x));
	}
}

TEST_CASE("[C5 arm_rms_f16] the f16 rms agrees with arm_rms_f32 of the same input within the f16 accumulation bound") {
	for (size_t n : kSizes) {
		INFO("block size " << n);
		std::vector<F> x = randVec(n, static_cast<uint32_t>(n + 70), -2.0, 2.0);
		std::vector<float> xf = toFloat(x);
		float32_t r32 = 0;
		arm_rms_f32(xf.data(), static_cast<uint32_t>(n), &r32);
		double exact;
		Range r = rangeRms(x, &exact);
		double tol = std::max(r.hi - exact, exact - r.lo);
		CHECK_VS_F32(call1(arm_rms_f16, x), static_cast<double>(r32), tol);
	}
}

TEST_CASE("[C5 arm_var_f16] the f16 variance agrees with arm_var_f32 of the same input within the f16 accumulation bound") {
	for (size_t n : kSizes) {
		if (n < 2) continue;
		INFO("block size " << n);
		std::vector<F> x = randVec(n, static_cast<uint32_t>(n + 70), -2.0, 2.0);
		std::vector<float> xf = toFloat(x);
		float32_t r32 = 0;
		arm_var_f32(xf.data(), static_cast<uint32_t>(n), &r32);
		double exact;
		Range r = rangeVar(x, &exact);
		double tol = std::max(r.hi - exact, exact - r.lo);
		CHECK_VS_F32(call1(arm_var_f16, x), static_cast<double>(r32), tol);
	}
}

TEST_CASE("[C5 arm_std_f16] the f16 standard deviation agrees with arm_std_f32 of the same input within the f16 accumulation bound") {
	for (size_t n : kSizes) {
		if (n < 2) continue;
		INFO("block size " << n);
		std::vector<F> x = randVec(n, static_cast<uint32_t>(n + 70), -2.0, 2.0);
		std::vector<float> xf = toFloat(x);
		float32_t r32 = 0;
		arm_std_f32(xf.data(), static_cast<uint32_t>(n), &r32);
		double exact;
		Range r = rangeStd(x, &exact);
		double tol = std::max(r.hi - exact, exact - r.lo);
		CHECK_VS_F32(call1(arm_std_f16, x), static_cast<double>(r32), tol);
	}
}

TEST_CASE("[C5 arm_mse_f16] the f16 mean square error agrees with arm_mse_f32 of the same input within the f16 accumulation bound") {
	for (size_t n : kSizes) {
		INFO("block size " << n);
		std::vector<F> a = randVec(n, static_cast<uint32_t>(n + 70), -2.0, 2.0);
		std::vector<F> b = randVec(n, static_cast<uint32_t>(n + 1070), -2.0, 2.0);
		std::vector<float> af = toFloat(a), bf = toFloat(b);
		float32_t r32 = 0;
		arm_mse_f32(af.data(), bf.data(), static_cast<uint32_t>(n), &r32);
		double exact;
		CHECK_VS_F32(call2(arm_mse_f16, a, b), static_cast<double>(r32), tolMse(a, b, &exact));
	}
}

TEST_CASE("[C5 arm_entropy_f16] the f16 entropy agrees with arm_entropy_f32 of the same input within the f16 bound") {
	for (size_t n : kSizes) {
		INFO("block size " << n);
		std::vector<F> p = randVec(n, static_cast<uint32_t>(n + 70), 0.02, 1.0);
		std::vector<float> pf = toFloat(p);
		float32_t r32 = arm_entropy_f32(pf.data(), static_cast<uint32_t>(n));
		double exact;
		double tol = tolEntropy(p, &exact);
		CHECK_VS_F32(arm_entropy_f16(p.data(), static_cast<uint32_t>(n)), static_cast<double>(r32), tol);
	}
}

TEST_CASE("[C5 arm_kullback_leibler_f16] the f16 divergence agrees with arm_kullback_leibler_f32 of the same input within the f16 bound") {
	for (size_t n : kSizes) {
		INFO("block size " << n);
		std::vector<F> a = randVec(n, static_cast<uint32_t>(n + 70), 0.05, 1.0);
		std::vector<F> b = randVec(n, static_cast<uint32_t>(n + 1070), 0.05, 1.0);
		std::vector<float> af = toFloat(a), bf = toFloat(b);
		float32_t r32 = arm_kullback_leibler_f32(af.data(), bf.data(), static_cast<uint32_t>(n));
		double exact;
		double tol = tolKl(a, b, &exact);
		CHECK_VS_F32(arm_kullback_leibler_f16(a.data(), b.data(), static_cast<uint32_t>(n)), static_cast<double>(r32), tol);
	}
}

TEST_CASE("[C5 arm_logsumexp_f16] the f16 logsumexp agrees with arm_logsumexp_f32 of the same input within the f16 bound") {
	for (size_t n : kSizes) {
		INFO("block size " << n);
		std::vector<F> x = randVec(n, static_cast<uint32_t>(n + 70), -5.0, 5.0);
		std::vector<float> xf = toFloat(x);
		float32_t r32 = arm_logsumexp_f32(xf.data(), static_cast<uint32_t>(n));
		double exact;
		double tol = tolLse(x, &exact);
		CHECK_VS_F32(arm_logsumexp_f16(x.data(), static_cast<uint32_t>(n)), static_cast<double>(r32), tol);
	}
}

TEST_CASE("[C5 arm_logsumexp_dot_prod_f16] the f16 result agrees with arm_logsumexp_dot_prod_f32 of the same input within the f16 bound") {
	for (size_t n : kSizes) {
		INFO("block size " << n);
		std::vector<F> a = randVec(n, static_cast<uint32_t>(n + 70), -3.0, 3.0);
		std::vector<F> b = randVec(n, static_cast<uint32_t>(n + 1070), -3.0, 3.0);
		std::vector<float> af = toFloat(a), bf = toFloat(b), tmpf(n);
		float32_t r32 = arm_logsumexp_dot_prod_f32(af.data(), bf.data(), static_cast<uint32_t>(n), tmpf.data());
		std::vector<F> t(n);
		double smax = 0;
		for (size_t i = 0; i < n; i++) {
			double s = dbl(a[i]) + dbl(b[i]);
			t[i] = h16(s);
			smax = std::max(smax, std::fabs(s));
		}
		double ignored;
		double tol = tolLse(t, &ignored) + kU * smax;
		dsp::GuardedBuffer<F> tmp(n);
		F out = arm_logsumexp_dot_prod_f16(a.data(), b.data(), static_cast<uint32_t>(n), tmp.data());
		CHECK_VS_F32(out, static_cast<double>(r32), tol);
	}
}

}  // TEST_SUITE
