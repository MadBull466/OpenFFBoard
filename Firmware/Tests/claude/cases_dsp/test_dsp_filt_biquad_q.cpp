/*
 * CMSIS-DSP Filtering, biquad cascades q31 / q15 (work order FBQ, phase P11 group G4). Requirement ids: Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * Units covered: arm_biquad_cascade_df1_q31 + _init_q31, arm_biquad_cascade_df1_fast_q31, arm_biquad_cas_df1_32x64_q31 + _init_q31,
 * arm_biquad_cascade_df1_q15 + _init_q15, arm_biquad_cascade_df1_fast_q15 (the fast variants share the init functions of the std ones).
 *
 * ARM ids (Testing/desc.txt:3296-:3330, tests Testing/Source/Tests/BIQUADQ31.cpp and BIQUADQ15.cpp): three ids, all run here, none disabled:
 *   BIQUAD Q31 arm_biquad_cascade_df1      BIQUADQ31.cpp:22  (3 stages, postShift 2, two blocks of 50): SNR_THRESHOLD 115 (:6), ABS_ERROR_Q31 1000 (:8)
 *   BIQUAD Q31 arm_biquad_cascade_df1_32x64 BIQUADQ31.cpp:83 (sample by sample, inner tail of 4 words zero): SNR_32x64_THRESHOLD 140 (:10), ABS_32x64_ERROR_Q31 25 (:12)
 *   BIQUAD Q15 arm_biquad_cascade_df1      BIQUADQ15.cpp:16: SNR_THRESHOLD 30 (:12), ABS_ERROR_Q15 500 (:14)
 * Patterns: Filtering/BIQUAD/BIQUADQ31/ and BIQUADQ15/ (BiquadInput1, BiquadCoefs1, BiquadOutput1). ARM has NO ids for arm_biquad_cascade_df1_fast_q31 /
 * _fast_q15, and the "AllBiquad*" random patterns of the q types (configs of 3 / 8 / 11 resp. 7 / 16 / 23 stages and blocks) are generated but used by no
 * q test: they are run here against the double DEF with a derived bound (C5), for all five functions.
 *
 * Scaling (derived from the code and DOC; DEF = difference equation y = b0 x + b1 x1 + b2 x2 + a1 y1 + a2 y2 with PLUS signs for a1, a2 (DOC of the 32x64
 * function: "Pay careful attention to the sign of the feedback coefficients")):
 *   q31 std : acc = sum of five 64-bit products (2.62), out = low 32 bits of floor(acc / 2^(31 - postShift)) (code: acc >> lShift, or the unrolled
 *             (acc_l >> lShift | acc_h << uShift)). Real coefficient = c / 2^(31 - postShift). DOC: "if the accumulator result overflows it wraps around
 *             rather than clip", "truncated ... by discarding the low 32 bits" = floor (arithmetic shift). Error against the real DEF: each stage injects an
 *             error in (-1, 0] LSB into its own recursion.
 *   q31 fast: each product is rounded to 2.30: floor((p + 2^31) / 2^32) (SMMULR / SMMLAR), five of them are added, out = acc << (postShift + 1). Error per
 *             stage in (-2.5, 2.5] * 2^(postShift + 1) LSB (five roundings of at most 1/2 acc unit; one acc unit = 2^(postShift + 1) output LSB).
 *   q31 32x64: state y is 1.63 (int64), acc = 3 exact products + two floor(Y a / 2^32) (mult32x64), Y = acc << (postShift + 1), out as the std one. The
 *             state keeps the fraction, so the error inside the recursion is 2 * 2^(postShift + 1) / 2^32 LSB and the final floor adds (-1, 0] LSB that is
 *             not fed back (passes only through later stages).
 *   q15 std : acc = sum of five 2.30 products (int64), out = SSAT16(acc >> (15 - postShift)) (floor, saturate). DOC: "saturated to 1.15". Coefficients are
 *             6 per stage {b0, 0, b1, b2, a1, a2} (init DOC; the struct comment "5 * numStages" is wrong).
 *   q15 fast: the same with a 32-bit accumulator (wraps on overflow, DOC), so with |sum| < 2^31 it is bit identical to the std one.
 * Error bound of the DEF tests (comment of errBound()): rounding errors e[n] of stage k enter ypred_k[n] = FF + a1 ypred_k[n-1] + a2 ypred_k[n-2] + e[n]
 * (the state IS the quantised output), so the output error is sum_m e_k[m] g_k[n-m] with g_k the impulse response of 1/A_k followed by the later stages H_j,
 * and |error| <= sum_k fb_k ||g_k||_1 (+ ff_k times the L1 norm of the later stages only, for the 32x64 output floor). The norms are computed numerically
 * (4096 samples; poles of the test filters have radius <= 0.8, so the tail is below 1e-300). If all coefficients are >= 0 the g are >= 0, so the floor
 * errors are ONE-SIDED: -bound <= (out - DEF) <= 0 (checked for the floor variants). Test filters: real coefficients k / 1024 (exactly representable in q15
 * and float), b scaled so that the L1 norm of each stage H is <= 0.9 and inputs are at most half scale, so no stage saturates or wraps (asserted on the DEF).
 *
 * Oracles. The EXACT integer models of the fast functions (rounding of the 2.30 products by SMMULR / SMMLAR, 32-bit accumulator of fast q15) and of the
 * q31 wrap are derived FROM THE LIBRARY CODE: the fast_q31 DOC says the products are "truncated to 2.30 format" while the code (and the SMMULR / SMMLAR
 * it emulates) rounds, so that DOC sentence is outdated (NEEDS REVIEW). The tests that compare the fast functions bit for bit with those models are
 * therefore named `characterization:`. The PRIMARY oracle of the fast q31 function is the double-DEF test with the derived bound
 * (5 roundings of at most 1/2 acc unit per stage), which holds under either wording (truncation would give (-5, 0], within the same bound).
 * The std q31, 32x64 and std q15 models follow DOC ("truncated ... by discarding the low 32 bits" = floor, "saturated to 1.15").
 * Fast q31 input range: DOC "the input signal must be scaled down by two bits and lie in the range [-0.25 +0.25)": all fast q31 tests use inputs
 * strictly inside it (Fast31::inAmp, Fast31::inLimit) and the ARM / AllBiquad data are halved until they are.
 *
 * NEEDS REVIEW (fast q31 at the ends of the range, NOT asserted either way): with the identity filter (b0 = 0x40000000, postShift 1) the input 0x7FFFFFFF
 * (and -0x7FFFFFFF) returns 0x80000000 on the host (std and 32x64 return the input). DOC of fast_q31: "Finally, the accumulator is saturated and converted
 * to a 1.31 result." and "The fast version has the same overflow behavior as the standard version", and the input must lie in [-0.25, +0.25); full scale
 * input violates that precondition and `acc << shift` (fast_q31.c:130, :156, :182, :214, :260) overflows (undefined behaviour).
 *
 * Not tested (NEEDS REVIEW): numStages == 0 (do { } while (--stage) runs 2^32 stages: out-of-bounds), postShift outside 0..14 (shift by >= 32 / negative
 * is undefined; q15 acc >> (15 - p) with p > 15), non-zero pad coefficient of the q15 coefficient layout (ignored by the portable std path, multiplied
 * by the next input in the SIMD path of the fast variant), in-place operation (pSrc == pDst: not documented).
 *
 * Host vs target (C6): the host runs the portable C path (no ARM_MATH_DSP / MVE); the q15 functions on the target use SMUAD / SMLALD on packed pairs and the
 * q31 std function the Helium kernel, whose exact rounding is the same by DOC but not tested here. `acc << shift` / `acc_h << uShift` on negative values
 * and signed overflow of the 32-bit SMLAD sums are undefined behaviour in C; GCC / clang wrap as the target does. Host int64 products are exact; nothing
 * depends on float -> int conversions.
 */
#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math.h"

namespace {

typedef int64_t i64;
typedef uint64_t u64;

const char *const kDir31 = "Filtering/BIQUAD/BIQUADQ31/";
const char *const kDir15 = "Filtering/BIQUAD/BIQUADQ15/";

// floor(v / 2^k), k in 0..63, with right shifts of non-negative values only
i64 flr(i64 v, int k) { return v >= 0 ? (v >> k) : ~((~v) >> k); }
int32_t wrap32(u64 v) { return static_cast<int32_t>(static_cast<uint32_t>(v)); }
i64 clamp15(i64 v) { return v > 32767 ? 32767 : (v < -32768 ? -32768 : v); }

struct Stage { i64 b0, b1, b2, a1, a2; };
struct SS { i64 x1 = 0, x2 = 0, y1 = 0, y2 = 0; };
struct K { int b0, b1, b2, a1, a2; };            // real coefficient * 1024
struct RStage { double b0, b1, b2, a1, a2; };

// Deterministic generator (LCG, Numerical Recipes constants). One call per statement.
struct Rng {
	uint32_t s;
	explicit Rng(uint32_t seed) : s(seed) {}
	uint32_t next() { s = s * 1664525u + 1013904223u; return s; }
	// uniform integer in [-m, m], m <= 2^30
	i64 sym(i64 m) {
		const uint32_t v = next() >> 1;
		return static_cast<i64>(v % static_cast<uint32_t>(2 * m + 1)) - m;
	}
};

// ---------------------------------------------------------------------------------------------------------------
// exact integer models (DEF in integers, scaling derived in the header comment)
// ---------------------------------------------------------------------------------------------------------------
u64 sum5(const Stage &s, const SS &st, i64 x) {
	return static_cast<u64>(s.b0 * x) + static_cast<u64>(s.b1 * st.x1) + static_cast<u64>(s.b2 * st.x2) + static_cast<u64>(s.a1 * st.y1) +
	       static_cast<u64>(s.a2 * st.y2);
}
void push(SS &st, i64 x, i64 y) { st.x2 = st.x1; st.x1 = x; st.y2 = st.y1; st.y1 = y; }
std::vector<int32_t> pack5(const std::vector<Stage> &st) {
	std::vector<int32_t> v;
	for (const Stage &s : st) {
		v.push_back(static_cast<int32_t>(s.b0)); v.push_back(static_cast<int32_t>(s.b1)); v.push_back(static_cast<int32_t>(s.b2));
		v.push_back(static_cast<int32_t>(s.a1)); v.push_back(static_cast<int32_t>(s.a2));
	}
	return v;
}

struct Std31 {
	typedef int32_t Sample; typedef int32_t St; typedef arm_biquad_casd_df1_inst_q31 Inst;
	static const int W = 31; static const int kMaxP = 5;
	static std::vector<int32_t> pack(const std::vector<Stage> &s) { return pack5(s); }
	static void init(Inst *i, unsigned n, const int32_t *c, St *st, int p) { arm_biquad_cascade_df1_init_q31(i, static_cast<uint8_t>(n), c, st, static_cast<int8_t>(p)); }
	static void run(const Inst *i, const int32_t *in, int32_t *out, uint32_t n) { arm_biquad_cascade_df1_q31(i, in, out, n); }
	static i64 step(const Stage &s, int p, SS &st, i64 x) {
		const i64 acc = static_cast<i64>(sum5(s, st, x));
		const i64 y = wrap32(static_cast<u64>(flr(acc, 31 - p)));
		push(st, x, y);
		return y;
	}
	static double fb(int) { return 1.0; }
	static double ff(int) { return 0.0; }
	static i64 outOfState(i64 s) { return s; }
	static i64 rawMax() { return i64(1) << 29; }
	static i64 inAmp() { return i64(1) << 30; }
	static i64 wideAmp() { return i64(1) << 30; }
	static i64 inLimit() { return i64(1) << 31; }
	static void checkFields(const Inst &i, unsigned n, const int32_t *c, const St *st, int p) {
		CHECK(i.numStages == n); CHECK(i.pCoeffs == c); CHECK(i.pState == st); CHECK(static_cast<int>(i.postShift) == p);
	}
};
struct Fast31 : Std31 {
	static void run(const Inst *i, const int32_t *in, int32_t *out, uint32_t n) { arm_biquad_cascade_df1_fast_q31(i, in, out, n); }
	static i64 rnd(i64 prod) { return flr(prod + (i64(1) << 31), 32); }   // SMMULR / SMMLAR rounding
	static i64 step(const Stage &s, int p, SS &st, i64 x) {
		const u64 a = static_cast<u64>(rnd(s.b0 * x)) + static_cast<u64>(rnd(s.b1 * st.x1)) + static_cast<u64>(rnd(s.b2 * st.x2)) +
		              static_cast<u64>(rnd(s.a1 * st.y1)) + static_cast<u64>(rnd(s.a2 * st.y2));
		const int32_t acc = wrap32(a);
		const i64 y = wrap32(static_cast<u64>(static_cast<uint32_t>(acc) << (p + 1)));
		push(st, x, y);
		return y;
	}
	static double fb(int p) { return 2.5 * std::ldexp(1.0, p + 1); }
	static i64 rawMax() { return i64(1) << 24; }
	// DOC: "the input signal must be scaled down by two bits and lie in the range [-0.25 +0.25)": every fast q31 test stays inside
	static i64 inAmp() { return (i64(1) << 29) - 256; }
	static i64 wideAmp() { return (i64(1) << 29) - 256; }
	static i64 inLimit() { return i64(1) << 29; }
};
struct X64 {
	typedef int32_t Sample; typedef int64_t St; typedef arm_biquad_cas_df1_32x64_ins_q31 Inst;
	static const int W = 31; static const int kMaxP = 1;
	static std::vector<int32_t> pack(const std::vector<Stage> &s) { return pack5(s); }
	static void init(Inst *i, unsigned n, const int32_t *c, St *st, int p) { arm_biquad_cas_df1_32x64_init_q31(i, static_cast<uint8_t>(n), c, st, static_cast<uint8_t>(p)); }
	static void run(const Inst *i, const int32_t *in, int32_t *out, uint32_t n) { arm_biquad_cas_df1_32x64_q31(i, in, out, n); }
	// floor(Y a / 2^32) = floor(Ylo a / 2^32) + (Y >> 32) a with Y = hi 2^32 + lo, 0 <= lo < 2^32 (mult32x64)
	static i64 mult(i64 Y, i64 a) { return flr((Y & 0xFFFFFFFFLL) * a, 32) + flr(Y, 32) * a; }
	static i64 step(const Stage &s, int p, SS &st, i64 x) {
		const u64 a = static_cast<u64>(s.b0 * x) + static_cast<u64>(s.b1 * st.x1) + static_cast<u64>(s.b2 * st.x2) + static_cast<u64>(mult(st.y1, s.a1)) +
		              static_cast<u64>(mult(st.y2, s.a2));
		const i64 acc = static_cast<i64>(a);
		const i64 y = wrap32(static_cast<u64>(flr(acc, 31 - p)));
		const i64 Y = static_cast<i64>(a << (p + 1));
		st.x2 = st.x1; st.x1 = x; st.y2 = st.y1; st.y1 = Y;
		return y;
	}
	static double fb(int p) { return std::ldexp(1.0, p + 2) / 4294967296.0; }
	static double ff(int) { return 1.0; }
	static i64 outOfState(i64 s) { return flr(s, 32); }
	static i64 rawMax() { return i64(1) << 26; }
	static i64 inAmp() { return i64(1) << 30; }
	static i64 wideAmp() { return i64(1) << 30; }
	static i64 inLimit() { return i64(1) << 31; }
	static void checkFields(const Inst &i, unsigned n, const int32_t *c, const St *st, int p) {
		CHECK(i.numStages == n); CHECK(i.pCoeffs == c); CHECK(i.pState == st); CHECK(static_cast<int>(i.postShift) == p);
	}
};
struct Std15 {
	typedef int16_t Sample; typedef int16_t St; typedef arm_biquad_casd_df1_inst_q15 Inst;
	static const int W = 15; static const int kMaxP = 5;
	static std::vector<int16_t> pack(const std::vector<Stage> &s) {
		std::vector<int16_t> v;
		for (const Stage &t : s) {
			v.push_back(static_cast<int16_t>(t.b0)); v.push_back(0); v.push_back(static_cast<int16_t>(t.b1)); v.push_back(static_cast<int16_t>(t.b2));
			v.push_back(static_cast<int16_t>(t.a1)); v.push_back(static_cast<int16_t>(t.a2));
		}
		return v;
	}
	static void init(Inst *i, unsigned n, const int16_t *c, St *st, int p) { arm_biquad_cascade_df1_init_q15(i, static_cast<uint8_t>(n), c, st, static_cast<int8_t>(p)); }
	static void run(const Inst *i, const int16_t *in, int16_t *out, uint32_t n) { arm_biquad_cascade_df1_q15(i, in, out, n); }
	static i64 step(const Stage &s, int p, SS &st, i64 x) {
		const i64 y = clamp15(flr(static_cast<i64>(sum5(s, st, x)), 15 - p));
		push(st, x, y);
		return y;
	}
	static double fb(int) { return 1.0; }
	static double ff(int) { return 0.0; }
	static i64 outOfState(i64 s) { return s; }
	static i64 rawMax() { return 32767; }
	static i64 inAmp() { return i64(1) << 14; }
	static i64 wideAmp() { return 32767; }
	static i64 inLimit() { return i64(1) << 16; }   // no limit beyond the type
	static void checkFields(const Inst &i, unsigned n, const int16_t *c, const St *st, int p) {
		CHECK(static_cast<unsigned>(i.numStages) == n); CHECK(i.pCoeffs == c); CHECK(i.pState == st); CHECK(static_cast<int>(i.postShift) == p);
	}
};
struct Fast15 : Std15 {
	static void run(const Inst *i, const int16_t *in, int16_t *out, uint32_t n) { arm_biquad_cascade_df1_fast_q15(i, in, out, n); }
	static i64 step(const Stage &s, int p, SS &st, i64 x) {
		const i64 acc = wrap32(sum5(s, st, x));   // 32-bit accumulator
		const i64 y = clamp15(flr(acc, 15 - p));
		push(st, x, y);
		return y;
	}
	static i64 rawMax() { return 1 << 13; }
};

// ---------------------------------------------------------------------------------------------------------------
// instance wrapper with an exactly sized state buffer in its own heap allocation (no guard words: the sanitizer build catches an overrun)
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
class Biq {
public:
	typedef typename Tr::Sample T;
	typedef typename Tr::St S;
	Biq(const std::vector<Stage> &stages, int p) : n_(static_cast<unsigned>(stages.size())), p_(p), coefs_(Tr::pack(stages)), state_(new S[4u * stages.size()]) {
		Tr::init(&inst_, n_, coefs_.data(), state_.get(), p_);
	}
	void reinit() { Tr::init(&inst_, n_, coefs_.data(), state_.get(), p_); }
	void run(const T *in, T *out, size_t n) { Tr::run(&inst_, in, out, static_cast<uint32_t>(n)); }
	std::vector<T> process(const std::vector<T> &in) {
		const T dummy = 0;
		dsp::GuardedBuffer<T> out(in.size());
		run(in.empty() ? &dummy : in.data(), out.data(), in.size());
		CHECK(out.guardIntact());
		return out.output();
	}
	// consecutive blocks, sizes taken cyclically
	std::vector<T> processBlocks(const std::vector<T> &in, const std::vector<size_t> &sizes) {
		std::vector<T> res;
		size_t pos = 0, k = 0;
		while (pos < in.size()) {
			const size_t n = std::min(sizes[k++ % sizes.size()], in.size() - pos);
			const std::vector<T> blk(in.begin() + static_cast<long>(pos), in.begin() + static_cast<long>(pos + n));
			const std::vector<T> o = process(blk);
			res.insert(res.end(), o.begin(), o.end());
			pos += n;
		}
		return res;
	}
	std::vector<S> state() const { return std::vector<S>(state_.get(), state_.get() + 4u * n_); }
	const std::vector<T> &coefs() const { return coefs_; }
	typename Tr::Inst inst_;
private:
	unsigned n_;
	int p_;
	std::vector<T> coefs_;
	std::unique_ptr<S[]> state_;   // exactly 4 * numStages elements in its own heap block: ASan sees a write behind the last one
};

template <class Tr>
struct Model {
	std::vector<Stage> stages;
	int p;
	std::vector<SS> st;
	std::vector<std::vector<i64>> in, out;   // per stage, of the last run
	Model(const std::vector<Stage> &s, int p_) : stages(s), p(p_), st(s.size()) {}
	std::vector<i64> run(const std::vector<i64> &x) {
		in.assign(stages.size(), std::vector<i64>());
		out.assign(stages.size(), std::vector<i64>());
		std::vector<i64> cur = x;
		for (size_t k = 0; k < stages.size(); k++) {
			in[k] = cur;
			std::vector<i64> y(cur.size());
			for (size_t n = 0; n < cur.size(); n++) y[n] = Tr::step(stages[k], p, st[k], cur[n]);
			out[k] = y;
			cur = y;
		}
		return cur;
	}
};

template <class T>
std::vector<i64> toI(const std::vector<T> &v) { return std::vector<i64>(v.begin(), v.end()); }
template <class T>
std::vector<T> fromI(const std::vector<i64> &v) {
	std::vector<T> r;
	for (i64 x : v) r.push_back(static_cast<T>(x));
	return r;
}

template <class Tr>
void expectOut(const std::vector<typename Tr::Sample> &got, const std::vector<i64> &want) {
	auto c = dsp::exactEqual(want, got);
	CHECK_MESSAGE(c, c.message);
}
template <class Tr>
void expectState(const Biq<Tr> &f, const Model<Tr> &m) {
	const std::vector<typename Tr::St> s = f.state();
	REQUIRE(s.size() == 4 * m.st.size());
	for (size_t k = 0; k < m.st.size(); k++) {
		INFO("stage " << k);
		CHECK(static_cast<i64>(s[4 * k + 0]) == m.st[k].x1);
		CHECK(static_cast<i64>(s[4 * k + 1]) == m.st[k].x2);
		CHECK(static_cast<i64>(s[4 * k + 2]) == m.st[k].y1);
		CHECK(static_cast<i64>(s[4 * k + 3]) == m.st[k].y2);
	}
}

// ---------------------------------------------------------------------------------------------------------------
// test data
// ---------------------------------------------------------------------------------------------------------------
// L1 norm of the impulse response of: feedback == true: 1 / A_from followed by the full stages from + 1 ...; feedback == false: the full stages
// from, from + 1, ... (identity if from == size)
double impulseL1(const std::vector<RStage> &st, size_t from, bool feedback) {
	const size_t N = 4096;
	std::vector<double> h(N, 0.0);
	h[0] = 1.0;
	for (size_t j = from; j < st.size(); j++) {
		const RStage &s = st[j];
		const bool allPole = feedback && (j == from);
		std::vector<double> y(N, 0.0);
		for (size_t n = 0; n < N; n++) {
			double v = allPole ? h[n] : (s.b0 * h[n] + (n > 0 ? s.b1 * h[n - 1] : 0.0) + (n > 1 ? s.b2 * h[n - 2] : 0.0));
			v += (n > 0 ? s.a1 * y[n - 1] : 0.0) + (n > 1 ? s.a2 * y[n - 2] : 0.0);
			y[n] = v;
		}
		h = y;
	}
	double sum = 0;
	for (double v : h) sum += std::fabs(v);
	return sum;
}
// sum_k fb ||g_k||_1 + ff ||H_{k+1..}||_1, see header
double errBound(const std::vector<RStage> &st, double fb, double ff) {
	double b = 0;
	for (size_t k = 0; k < st.size(); k++) {
		b += fb * impulseL1(st, k, true);
		if (ff != 0.0) b += ff * impulseL1(st, k + 1, false);
	}
	return b;
}
// stage whose from == size: identity (impulseL1 of an empty range is 1)

struct DefOut { std::vector<double> y; std::vector<double> maxIn, maxOut; };
DefOut defRun(const std::vector<RStage> &st, const std::vector<double> &in) {
	DefOut r;
	std::vector<double> x = in;
	for (const RStage &s : st) {
		double x1 = 0, x2 = 0, y1 = 0, y2 = 0, mi = 0, mo = 0;
		std::vector<double> y(x.size());
		for (size_t n = 0; n < x.size(); n++) {
			y[n] = s.b0 * x[n] + s.b1 * x1 + s.b2 * x2 + s.a1 * y1 + s.a2 * y2;
			x2 = x1; x1 = x[n]; y2 = y1; y1 = y[n];
			mi = std::max(mi, std::fabs(x[n]));
			mo = std::max(mo, std::fabs(y[n]));
		}
		r.maxIn.push_back(mi);
		r.maxOut.push_back(mo);
		x = y;
	}
	r.y = x;
	return r;
}
std::vector<RStage> realOf(const std::vector<Stage> &st, int sh) {
	std::vector<RStage> r;
	for (const Stage &s : st)
		r.push_back({std::ldexp(double(s.b0), -sh), std::ldexp(double(s.b1), -sh), std::ldexp(double(s.b2), -sh), std::ldexp(double(s.a1), -sh),
		             std::ldexp(double(s.a2), -sh)});
	return r;
}
std::vector<RStage> realOfK(const std::vector<K> &ks) {
	std::vector<RStage> r;
	for (const K &k : ks) r.push_back({k.b0 / 1024.0, k.b1 / 1024.0, k.b2 / 1024.0, k.a1 / 1024.0, k.a2 / 1024.0});
	return r;
}

// stable stage with real coefficients k / 1024. Poles: |a2| <= 0.5, |a1| <= 0.6 (1 - |a2|) < 1 - a2 (triangle |a1| < 1 - a2), radius <= 0.8.
// b scaled down so that the L1 norm of the stage's impulse response is <= 0.9.
K genK(Rng &r, bool nonneg) {
	K k;
	k.a2 = static_cast<int>(nonneg ? (r.sym(128) + 128) : r.sym(256));
	const i64 a1max = (6 * (1024 - std::abs(k.a2))) / 10;
	k.a1 = static_cast<int>(nonneg ? (r.sym(a1max / 2) + a1max / 2) : r.sym(a1max));
	k.b0 = static_cast<int>(nonneg ? (r.sym(256) + 256) : r.sym(512));
	k.b1 = static_cast<int>(nonneg ? (r.sym(256) + 256) : r.sym(512));
	k.b2 = static_cast<int>(nonneg ? (r.sym(256) + 256) : r.sym(512));
	if (k.b0 == 0 && k.b1 == 0 && k.b2 == 0) k.b0 = 256;
	// L1 of H = B / A: simulate
	std::vector<double> h(2048, 0.0);
	const double ra1 = k.a1 / 1024.0, ra2 = k.a2 / 1024.0;
	for (size_t n = 0; n < h.size(); n++) {
		double v = (n == 0 ? k.b0 : (n == 1 ? k.b1 : (n == 2 ? k.b2 : 0))) / 1024.0;
		v += (n > 0 ? ra1 * h[n - 1] : 0.0) + (n > 1 ? ra2 * h[n - 2] : 0.0);
		h[n] = v;
	}
	double s = 0;
	for (double v : h) s += std::fabs(v);
	if (s > 0.9) {
		const double f = 0.9 / s;
		k.b0 = static_cast<int>(k.b0 * f); k.b1 = static_cast<int>(k.b1 * f); k.b2 = static_cast<int>(k.b2 * f);   // truncation toward 0
		if (k.b0 == 0 && k.b1 == 0 && k.b2 == 0) k.b0 = 1;
	}
	return k;
}
std::vector<K> genKs(Rng &r, size_t n, bool nonneg) {
	std::vector<K> v;
	for (size_t i = 0; i < n; i++) v.push_back(genK(r, nonneg));
	return v;
}
// k / 1024 -> c = k * 2^(W - p - 10), so that c / 2^(W - p) = k / 1024
template <class Tr>
std::vector<Stage> toStages(const std::vector<K> &ks, int p) {
	const i64 m = i64(1) << (Tr::W - p - 10);
	std::vector<Stage> r;
	for (const K &k : ks) r.push_back({k.b0 * m, k.b1 * m, k.b2 * m, k.a1 * m, k.a2 * m});
	return r;
}
// random samples in [-amp, amp], multiples of mult
template <class Tr>
std::vector<typename Tr::Sample> randIn(Rng &r, size_t n, i64 amp, i64 mult = 1) {
	std::vector<typename Tr::Sample> v(n);
	for (size_t i = 0; i < n; i++) {
		const i64 x = r.sym(amp / mult) * mult;
		v[i] = static_cast<typename Tr::Sample>(x);
	}
	return v;
}
template <class Tr>
i64 halfScale() { return Tr::inAmp(); }   // half scale (quarter scale for the fast q31 function, see Fast31::inAmp)

// ---------------------------------------------------------------------------------------------------------------
// DEF check against the double difference equation
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
void checkDef(const std::vector<Stage> &stages, int p, const std::vector<typename Tr::Sample> &in, const std::vector<typename Tr::Sample> &got,
              bool oneSided) {
	const std::vector<RStage> rs = realOf(stages, Tr::W - p);
	std::vector<double> x(in.begin(), in.end());
	const DefOut d = defRun(rs, x);
	const double bound = errBound(rs, Tr::fb(p), Tr::ff(p)) + 0.01;
	const double full = std::ldexp(1.0, Tr::W);
	for (size_t k = 0; k < d.maxOut.size(); k++) REQUIRE(d.maxOut[k] + bound < 0.9 * full);   // the DEF itself neither saturates nor wraps
	REQUIRE(got.size() == in.size());
	size_t bad = in.size();
	double worst = 0;
	for (size_t n = 0; n < got.size(); n++) {
		const double e = static_cast<double>(got[n]) - d.y[n];
		const bool ok = oneSided ? (e <= 0.01 && e >= -bound) : (std::fabs(e) <= bound);
		if (std::fabs(e) > std::fabs(worst)) worst = e;
		if (!ok && bad == in.size()) bad = n;
	}
	INFO("bound " << bound << " LSB, worst error " << worst << " LSB, first violation at sample " << bad);
	CHECK(bad == in.size());
}

// ===============================================================================================================
// test bodies (templates over the five functions)
// ===============================================================================================================
// C3: bit exact against the integer model, random stable filters
template <class Tr>
void exactStable() {
	Rng r(0xB10C0001u);
	const unsigned counts[] = {1, 2, 3, 4, 5, 7, 9};
	const size_t sizes[] = {13, 64};
	for (unsigned n : counts)
		for (int p : {0, 2, Tr::kMaxP}) {
			const std::vector<Stage> st = toStages<Tr>(genKs(r, n, false), p);
			for (size_t bs : sizes) {
				INFO("stages " << n << " postShift " << p << " block " << bs);
				const std::vector<typename Tr::Sample> in = randIn<Tr>(r, bs, halfScale<Tr>());
				Biq<Tr> f(st, p);
				Model<Tr> m(st, p);
				const std::vector<typename Tr::Sample> out = f.process(in);
				expectOut<Tr>(out, m.run(toI(in)));
				expectState(f, m);
			}
		}
}

// C3: bit exact against the integer model, unstable full range coefficients (saturation of q15, wrap of q31)
template <class Tr>
void exactWide() {
	Rng r(0xB10C0002u);
	for (int rep = 0; rep < 24; rep++) {
		const int p = rep % 2;
		std::vector<Stage> st;
		const size_t n = 1 + static_cast<size_t>(rep % 3);
		for (size_t k = 0; k < n; k++) {
			const i64 m = Tr::rawMax();
			Stage s;
			s.b0 = r.sym(m); s.b1 = r.sym(m); s.b2 = r.sym(m); s.a1 = r.sym(m); s.a2 = r.sym(m);
			st.push_back(s);
		}
		std::vector<typename Tr::Sample> in(37);
		for (auto &v : in) v = static_cast<typename Tr::Sample>(r.sym(Tr::wideAmp()));
		INFO("rep " << rep);
		Biq<Tr> f(st, p);
		Model<Tr> m(st, p);
		expectOut<Tr>(f.process(in), m.run(toI(in)));
		expectState(f, m);
	}
}

// C3: against the double DEF within the derived bound
template <class Tr>
void defBound() {
	Rng r(0xB10C0003u);
	const unsigned counts[] = {1, 2, 3, 5, 9};
	for (unsigned n : counts)
		for (int p : {0, 2})
			for (int rep = 0; rep < 2; rep++) {
				INFO("stages " << n << " postShift " << p << " rep " << rep);
				const std::vector<Stage> st = toStages<Tr>(genKs(r, n, false), p);
				const std::vector<typename Tr::Sample> in = randIn<Tr>(r, 96, halfScale<Tr>());
				Biq<Tr> f(st, p);
				checkDef<Tr>(st, p, in, f.process(in), false);
			}
}

// C3: floor error is one sided (all coefficients >= 0)
template <class Tr>
void oneSided() {
	Rng r(0xB10C0004u);
	for (unsigned n : {1u, 2u, 4u, 9u})
		for (int p : {0, 3}) {
			INFO("stages " << n << " postShift " << p);
			const std::vector<Stage> st = toStages<Tr>(genKs(r, n, true), p);
			const std::vector<typename Tr::Sample> in = randIn<Tr>(r, 200, halfScale<Tr>());
			Biq<Tr> f(st, p);
			checkDef<Tr>(st, p, in, f.process(in), true);
		}
}

// C4: init stores the arguments, zeroes the exactly sized state and keeps the coefficients
template <class Tr>
void initFields() {
	Rng r(0xB10C0005u);
	for (unsigned n = 1; n <= 9; n++)
		for (int p : {0, 2}) {
			INFO("stages " << n << " postShift " << p);
			const std::vector<Stage> st = toStages<Tr>(genKs(r, n, false), p);
			const std::vector<typename Tr::Sample> c = Tr::pack(st);
			dsp::GuardedBuffer<typename Tr::St> state(4 * n);   // poisoned with 0xCD, guard behind it
			typename Tr::Inst inst;
			Tr::init(&inst, n, c.data(), state.data(), p);
			Tr::checkFields(inst, n, c.data(), state.data(), p);
			const std::vector<typename Tr::St> s = state.output();
			for (auto v : s) REQUIRE(v == 0);
			CHECK(state.guardIntact());
			CHECK(c == Tr::pack(st));
		}
}

// C4: re-init resets the state
template <class Tr>
void reinitResets() {
	Rng r(0xB10C0006u);
	const std::vector<Stage> st = toStages<Tr>(genKs(r, 3, false), 2);
	const std::vector<typename Tr::Sample> in = randIn<Tr>(r, 30, halfScale<Tr>());
	Biq<Tr> f(st, 2);
	const std::vector<typename Tr::Sample> first = f.process(in);
	(void)f.process(in);
	f.reinit();
	const std::vector<typename Tr::St> s = f.state();
	for (auto v : s) CHECK(v == 0);
	CHECK(f.process(in) == first);
}

// C4: two instances are independent
template <class Tr>
void independent() {
	Rng r(0xB10C0007u);
	const std::vector<Stage> st = toStages<Tr>(genKs(r, 3, false), 1);
	const std::vector<typename Tr::Sample> a = randIn<Tr>(r, 40, halfScale<Tr>());
	const std::vector<typename Tr::Sample> b = randIn<Tr>(r, 40, halfScale<Tr>());
	Biq<Tr> ra(st, 1), rb(st, 1);
	const std::vector<typename Tr::Sample> refA = ra.process(a), refB = rb.process(b);
	Biq<Tr> fa(st, 1), fb(st, 1);
	std::vector<typename Tr::Sample> oa, ob;
	for (size_t pos = 0; pos < 40; pos += 5) {
		const std::vector<typename Tr::Sample> ba(a.begin() + static_cast<long>(pos), a.begin() + static_cast<long>(pos + 5));
		const std::vector<typename Tr::Sample> bb(b.begin() + static_cast<long>(pos), b.begin() + static_cast<long>(pos + 5));
		const auto x = fa.process(ba);
		const auto y = fb.process(bb);
		oa.insert(oa.end(), x.begin(), x.end());
		ob.insert(ob.end(), y.begin(), y.end());
	}
	CHECK(oa == refA);
	CHECK(ob == refB);
}

// C4: consecutive blocks give the output and state of one long block
template <class Tr>
void continuity() {
	Rng r(0xB10C0008u);
	for (unsigned n : {1u, 2u, 3u, 5u, 9u}) {
		const std::vector<Stage> st = toStages<Tr>(genKs(r, n, false), 2);
		const std::vector<typename Tr::Sample> in = randIn<Tr>(r, 71, halfScale<Tr>());
		Biq<Tr> whole(st, 2);
		const std::vector<typename Tr::Sample> ref = whole.process(in);
		const std::vector<std::vector<size_t>> splits = {{1}, {2}, {3}, {4}, {5}, {7}, {1, 2, 3, 4, 5, 7}, {9, 1}};
		for (const auto &sp : splits) {
			INFO("stages " << n << " first block " << sp[0] << " (" << sp.size() << " sizes)");
			Biq<Tr> f(st, 2);
			CHECK(f.processBlocks(in, sp) == ref);
			CHECK(f.state() == whole.state());
		}
	}
}

// C4 / C2: block size 0 changes nothing
template <class Tr>
void emptyBlock() {
	Rng r(0xB10C0009u);
	const std::vector<Stage> st = toStages<Tr>(genKs(r, 2, false), 2);
	const std::vector<typename Tr::Sample> in = randIn<Tr>(r, 20, halfScale<Tr>());
	Biq<Tr> whole(st, 2);
	const std::vector<typename Tr::Sample> ref = whole.process(in);
	Biq<Tr> f(st, 2);
	const std::vector<typename Tr::Sample> head(in.begin(), in.begin() + 7), tail(in.begin() + 7, in.end());
	std::vector<typename Tr::Sample> o = f.process(head);
	const auto s0 = f.state();
	CHECK(f.process(std::vector<typename Tr::Sample>()).empty());
	CHECK(f.state() == s0);
	const auto t = f.process(tail);
	o.insert(o.end(), t.begin(), t.end());
	CHECK(o == ref);
}

// C2: output guard and exact output length for block sizes around the unroll factor
template <class Tr>
void outputGuard() {
	Rng r(0xB10C000Au);
	const std::vector<Stage> st = toStages<Tr>(genKs(r, 3, false), 1);
	for (size_t bs : {1, 2, 3, 4, 5, 6, 7, 8, 9}) {
		INFO("block " << bs);
		const std::vector<typename Tr::Sample> in = randIn<Tr>(r, bs, halfScale<Tr>());
		Biq<Tr> f(st, 1);
		Model<Tr> m(st, 1);
		expectOut<Tr>(f.process(in), m.run(toI(in)));   // process() checks the output and state guards
	}
}

// C2: the 4 * numStages state words are the only ones written (guard words behind a state buffer of exactly that size). The other tests use a
// tight heap block for the state instead (Biq), where ASan sees an overrun; the guard words here are visible without a sanitizer.
template <class Tr>
void stateGuard() {
	Rng r(0xB10C000Du);
	for (unsigned n : {1u, 3u, 9u}) {
		const std::vector<Stage> st = toStages<Tr>(genKs(r, n, false), 1);
		const std::vector<typename Tr::Sample> c = Tr::pack(st);
		dsp::GuardedBuffer<typename Tr::St> state(4 * n);
		typename Tr::Inst inst;
		Tr::init(&inst, n, c.data(), state.data(), 1);
		for (size_t bs : {1, 4, 5, 9}) {
			const std::vector<typename Tr::Sample> in = randIn<Tr>(r, bs, halfScale<Tr>());
			dsp::GuardedBuffer<typename Tr::Sample> out(bs);
			Tr::run(&inst, in.data(), out.data(), static_cast<uint32_t>(bs));
			INFO("stages " << n << " block " << bs);
			CHECK(state.guardIntact());
			CHECK(out.guardIntact());
		}
	}
}

// C4: state layout of the DOC {x[n-1], x[n-2], y[n-1], y[n-2]} per stage
template <class Tr>
void stateLayout() {
	Rng r(0xB10C000Bu);
	const std::vector<Stage> st = toStages<Tr>(genKs(r, 2, false), 2);
	const std::vector<typename Tr::Sample> in = randIn<Tr>(r, 9, halfScale<Tr>());
	Biq<Tr> f(st, 2);
	Model<Tr> m(st, 2);
	f.process(in);
	m.run(toI(in));
	const std::vector<typename Tr::St> s = f.state();
	for (size_t k = 0; k < 2; k++) {
		INFO("stage " << k);
		const auto &xi = m.in[k];
		const auto &yo = m.out[k];
		CHECK(static_cast<i64>(s[4 * k + 0]) == xi[8]);
		CHECK(static_cast<i64>(s[4 * k + 1]) == xi[7]);
		CHECK(Tr::outOfState(static_cast<i64>(s[4 * k + 2])) == yo[8]);
		CHECK(Tr::outOfState(static_cast<i64>(s[4 * k + 3])) == yo[7]);
	}
}

// C3: stage 1 delays by one sample, stage 2 by two samples: an impulse A appears at index 3 (hand derivation: real coefficient 1.0 is c = 2^(W - p),
// p = 1; acc = 2^(W-1) A, out = acc >> (W - 1) = A exactly; the fast q31 products are exact too: 2^30 A / 2^32 = A / 4, A multiple of 4, << 2)
template <class Tr>
void delayCascade() {
	const i64 one = i64(1) << (Tr::W - 1);
	const std::vector<Stage> st = {{0, one, 0, 0, 0}, {0, 0, one, 0, 0}};
	const i64 A = i64(1) << (Tr::W - 3);
	std::vector<typename Tr::Sample> in(12, 0);
	in[0] = static_cast<typename Tr::Sample>(A);
	Biq<Tr> f(st, 1);
	std::vector<typename Tr::Sample> want(12, 0);
	want[3] = static_cast<typename Tr::Sample>(A);
	CHECK(f.process(in) == want);
}

// C3: one pole b0 = 0.5, a1 = 0.5 (c = 2^(W-2) at p = 1), impulse x0 = 2^(W-3) = 0.125 FS (inside the fast q31 input range): y[0] = 0.5 x0 = 2^(W-4),
// y[n] = 0.5 y[n-1] = 2^(W-4-n), exact while the halving is exact (n <= W-4; q31 fast: the rounded products 2^(25-n) are exact for n <= 25, 24 shown)
template <class Tr>
void onePole(size_t exactCount) {
	const i64 half = i64(1) << (Tr::W - 2);
	const std::vector<Stage> st = {{half, 0, 0, half, 0}};
	std::vector<typename Tr::Sample> in(exactCount + 2, 0);
	in[0] = static_cast<typename Tr::Sample>(i64(1) << (Tr::W - 3));
	Biq<Tr> f(st, 1);
	const std::vector<typename Tr::Sample> out = f.process(in);
	for (size_t n = 0; n < exactCount; n++) {
		INFO("n " << n);
		CHECK(static_cast<i64>(out[n]) == (i64(1) << (Tr::W - 4 - static_cast<int>(n))));
	}
}

// C3: step response of y[n] = 0.25 x + 0.5 y[n-1] with x = 2^s (hand derivation in the cases below)
template <class Tr>
void stepResponse(int s, size_t count) {
	const i64 b = i64(1) << (Tr::W - 3);      // 0.25 at p = 1: c = 0.25 * 2^(W-1)
	const i64 a = i64(1) << (Tr::W - 2);      // 0.5
	const std::vector<Stage> st = {{b, 0, 0, a, 0}};
	std::vector<typename Tr::Sample> in(count, static_cast<typename Tr::Sample>(i64(1) << s));
	Biq<Tr> f(st, 1);
	const std::vector<typename Tr::Sample> out = f.process(in);
	// ideal: y[n] = 2^(s-1) (1 - 2^-(n+1)) = 2^(s-1) - 2^(s-2-n); floor makes the fixed point 2^(s-1) - 1 once s-2-n < 0 (see cases)
	for (size_t n = 0; n < count; n++) {
		INFO("n " << n);
		const i64 ideal = (n + 2 <= static_cast<size_t>(s)) ? (i64(1) << (s - 1)) - (i64(1) << (s - 2 - static_cast<int>(n))) : (i64(1) << (s - 1)) - 1;
		CHECK(static_cast<i64>(out[n]) == ideal);
	}
}

// C3: floor, not truncation toward zero: single tap 0.5 (p = 1) of +1 is 0, of -1 is -1 (acc = +-2^(W-2) >> (W-1))
template <class Tr>
void floorOfOne() {
	const i64 half = i64(1) << (Tr::W - 2);
	const std::vector<Stage> st = {{half, 0, 0, 0, 0}};
	Biq<Tr> f(st, 1);
	const std::vector<typename Tr::Sample> out = f.process({1, -1, 2, -2, 3, -3});
	// floor(0.5), floor(-0.5), 1, -1, floor(1.5) = 1, floor(-1.5) = -2
	CHECK(out == std::vector<typename Tr::Sample>({0, -1, 1, -1, 1, -2}));
}

// C3: strided full range sweep of a single tap. q31: y = floor(c x / 2^(31 - p)); the one-sided floor error is checked in exact int64 arithmetic
// (c x fits: |c| < 2^31, |x| <= 2^31): error strictly in (-1, 0]. q15: every input value, the integer DEF
// clamp(floor(c x / 2^(15 - p))) exactly.
template <class Tr>
void sweepSingleTap(i64 c, int p) {
	const std::vector<Stage> st = {{c, 0, 0, 0, 0}};
	std::vector<typename Tr::Sample> in;
	const i64 lo = -(i64(1) << (Tr::W - 1)), span = i64(1) << Tr::W;
	const i64 stride = (Tr::W == 31) ? 65521 : 1;
	for (i64 x = lo; x < lo + span; x += stride) in.push_back(static_cast<typename Tr::Sample>(x));
	in.push_back(static_cast<typename Tr::Sample>(lo + span - 1));
	Biq<Tr> f(st, p);
	const std::vector<typename Tr::Sample> out = f.process(in);
	size_t bad = in.size();
	for (size_t n = 0; n < in.size() && bad == in.size(); n++) {
		bool ok;
		if (Tr::W == 15) {
			ok = static_cast<i64>(out[n]) == clamp15(flr(c * static_cast<i64>(in[n]), 15 - p));
		} else {
			// exact integers: r = c x (|r| <= 2^62), k = 31 - p; floor error in (-1, 0] means out 2^k <= r < (out + 1) 2^k
			const i64 r = c * static_cast<i64>(in[n]);
			const i64 o = static_cast<i64>(out[n]);
			ok = (o * (i64(1) << (31 - p)) <= r) && (r < (o + 1) * (i64(1) << (31 - p)));
		}
		if (!ok) bad = n;
	}
	INFO("first violation at sample " << bad << " x = " << (bad < in.size() ? static_cast<double>(in[bad]) : 0.0));
	CHECK(bad == in.size());
}

// C5: against arm_biquad_cascade_df1_f32 on identical data. Bound = own bound + f32 bound. The f32 filter makes at most 9 roundings of relative size
// u = 2^-24 per output, each of at most S_k = (|b0|+|b1|+|b2|) max|x_k| + (|a1|+|a2|) max|y_k| (any partial sum is <= S_k); they enter the recursion
// like the integer rounding errors, so the f32 contribution is sum_k 9 u S_k ||g_k||_1 (in LSB). Coefficients (k / 1024) and inputs (multiples of
// 256 for q31: 23 significant bits) are exact in float.
template <class Tr>
void againstF32() {
	Rng r(0xB10C000Cu);
	const i64 mult = (Tr::W == 31) ? 256 : 1;
	for (unsigned n : {1u, 3u, 6u}) {
		const int p = 2;
		const std::vector<K> ks = genKs(r, n, false);
		const std::vector<Stage> st = toStages<Tr>(ks, p);
		const std::vector<typename Tr::Sample> in = randIn<Tr>(r, 80, halfScale<Tr>(), mult);
		Biq<Tr> f(st, p);
		const std::vector<typename Tr::Sample> got = f.process(in);

		std::vector<float> coefs, fin, fout(in.size());
		for (const K &k : ks) {
			coefs.push_back(static_cast<float>(k.b0 / 1024.0)); coefs.push_back(static_cast<float>(k.b1 / 1024.0)); coefs.push_back(static_cast<float>(k.b2 / 1024.0));
			coefs.push_back(static_cast<float>(k.a1 / 1024.0)); coefs.push_back(static_cast<float>(k.a2 / 1024.0));
		}
		const double full = std::ldexp(1.0, Tr::W);
		for (auto v : in) fin.push_back(static_cast<float>(static_cast<double>(v) / full));
		std::vector<float> state(4 * n, 0.0f);
		arm_biquad_casd_df1_inst_f32 S;
		arm_biquad_cascade_df1_init_f32(&S, static_cast<uint8_t>(n), coefs.data(), state.data());
		arm_biquad_cascade_df1_f32(&S, fin.data(), fout.data(), static_cast<uint32_t>(in.size()));

		const std::vector<RStage> rs = realOfK(ks);
		std::vector<double> x(in.begin(), in.end());
		const DefOut d = defRun(rs, x);
		double bound = errBound(rs, Tr::fb(p), Tr::ff(p)) + 0.01;
		const double u = std::ldexp(1.0, -24);
		for (size_t k = 0; k < n; k++) {
			const double S_k = (std::fabs(rs[k].b0) + std::fabs(rs[k].b1) + std::fabs(rs[k].b2)) * d.maxIn[k] + (std::fabs(rs[k].a1) + std::fabs(rs[k].a2)) * d.maxOut[k];
			bound += 9.0 * u * S_k * impulseL1(rs, k, true);   // S_k is in LSB already
			REQUIRE(d.maxOut[k] < 0.9 * full);
		}
		size_t bad = in.size();
		for (size_t i = 0; i < in.size(); i++)
			if (!(std::fabs(static_cast<double>(got[i]) - static_cast<double>(fout[i]) * full) <= bound)) { bad = i; break; }
		INFO("stages " << n << " bound " << bound << " LSB, first violation " << bad);
		CHECK(bad == in.size());
	}
}

// ---------------------------------------------------------------------------------------------------------------
// ARM pattern helpers
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
std::vector<Stage> stagesFromPattern(const std::vector<typename Tr::Sample> &c, size_t off, size_t n) {
	const size_t stride = (Tr::W == 31) ? 5 : 6;
	std::vector<Stage> st;
	for (size_t k = 0; k < n; k++) {
		const typename Tr::Sample *q = c.data() + off + k * stride;
		if (stride == 5) st.push_back({q[0], q[1], q[2], q[3], q[4]});
		else st.push_back({q[0], q[2], q[3], q[4], q[5]});   // q[1] is the pad
	}
	return st;
}

// C5: ARM's reference data (BiquadInput1 / Coefs1, 3 stages, postShift 2, 100 samples) through the function, against the double DEF with the derived
// bound (poles of that pattern reach radius 0.98, so the bound is large but derived; the ARM bounds are in the C1 tests)
template <class Tr>
void armDataVsDef(const char *dir, const char *suffix) {
	const std::string d = dir;
	const dsp::Pattern pi = dsp::loadPattern(d + "BiquadInput1_" + suffix + ".txt");
	const dsp::Pattern pc = dsp::loadPattern(d + "BiquadCoefs1_" + suffix + ".txt");
	std::vector<typename Tr::Sample> in, c;
	if (Tr::W == 31) { for (auto v : pi.q31()) in.push_back(static_cast<typename Tr::Sample>(v)); for (auto v : pc.q31()) c.push_back(static_cast<typename Tr::Sample>(v)); }
	else { for (auto v : pi.q15()) in.push_back(static_cast<typename Tr::Sample>(v)); for (auto v : pc.q15()) c.push_back(static_cast<typename Tr::Sample>(v)); }
	const std::vector<Stage> st = stagesFromPattern<Tr>(c, 0, 3);
	// ARM's input reaches exactly +0.25 FS (2^29), which is outside the half open DOC range [-0.25, +0.25) of the fast q31 function: halve it then
	for (;;) {
		bool inRange = true;
		for (auto v : in) inRange = inRange && (std::llabs(static_cast<long long>(v)) < Tr::inLimit());
		if (inRange) break;
		for (auto &v : in) v = static_cast<typename Tr::Sample>(v / 2);
	}
	Biq<Tr> f(st, 2);
	const std::vector<typename Tr::Sample> out = f.process(in);
	const std::vector<RStage> rs = realOf(st, Tr::W - 2);
	const DefOut ref = defRun(rs, std::vector<double>(in.begin(), in.end()));
	const double bound = errBound(rs, Tr::fb(2), Tr::ff(2)) + 0.01;
	size_t bad = in.size();
	for (size_t i = 0; i < in.size(); i++)
		if (!(std::fabs(static_cast<double>(out[i]) - ref.y[i]) <= bound)) { bad = i; break; }
	INFO("bound " << bound << " LSB, first violation " << bad);
	CHECK(bad == in.size());
}

// C5: ARM's "AllBiquad" random patterns (unused by ARM's q tests) against the double DEF. Configs are (numStages, blockSize) pairs.
template <class Tr>
void allPatternsVsDef(const char *dir, const char *suffix) {
	const std::string d = dir;
	const std::vector<int16_t> cfg = dsp::loadPattern(d + "AllBiquadConfigs2_s16.txt").q15();
	std::vector<typename Tr::Sample> in, c;
	const dsp::Pattern pi = dsp::loadPattern(d + "AllBiquadInputs2_" + suffix + ".txt");
	const dsp::Pattern pc = dsp::loadPattern(d + "AllBiquadCoefs2_" + suffix + ".txt");
	if (Tr::W == 31) { for (auto v : pi.q31()) in.push_back(static_cast<typename Tr::Sample>(v)); for (auto v : pc.q31()) c.push_back(static_cast<typename Tr::Sample>(v)); }
	else { for (auto v : pi.q15()) in.push_back(static_cast<typename Tr::Sample>(v)); for (auto v : pc.q15()) c.push_back(static_cast<typename Tr::Sample>(v)); }
	const size_t stride = (Tr::W == 31) ? 5 : 6;
	size_t io = 0, co = 0;
	REQUIRE(cfg.size() % 2 == 0);
	for (size_t k = 0; k + 1 < cfg.size(); k += 2) {
		const size_t n = static_cast<size_t>(cfg[k]), b = static_cast<size_t>(cfg[k + 1]);
		REQUIRE(io + b <= in.size());
		REQUIRE(co + n * stride <= c.size());
		const std::vector<Stage> st = stagesFromPattern<Tr>(c, co, n);
		std::vector<typename Tr::Sample> x(in.begin() + static_cast<long>(io), in.begin() + static_cast<long>(io + b));
		const std::vector<RStage> rs = realOf(st, Tr::W - 2);
		// the random pattern filters can have a gain above 1: halve the input until the DEF stays below 90 percent of full scale in every stage
		// (a test choice, so that neither saturation nor wrap is involved)
		int halvings = 0;
		DefOut ref;
		for (;; halvings++) {
			ref = defRun(rs, std::vector<double>(x.begin(), x.end()));
			bool inRange = true;
			for (double m : ref.maxOut) inRange = inRange && (m < 0.9 * std::ldexp(1.0, Tr::W));
			for (auto v : x) inRange = inRange && (std::llabs(static_cast<long long>(v)) < Tr::inLimit());   // fast q31: DOC input range [-0.25, 0.25)
			if (inRange || halvings >= 14) break;
			for (auto &v : x) v = static_cast<typename Tr::Sample>(v / 2);
		}
		INFO("config " << k / 2 << ": " << n << " stages, " << b << " samples, input halved " << halvings << " times");
		REQUIRE(halvings < 14);
		Biq<Tr> f(st, 2);
		const std::vector<typename Tr::Sample> out = f.process(x);
		const double bound = errBound(rs, Tr::fb(2), Tr::ff(2)) + 0.01;
		size_t bad = x.size();
		for (size_t i = 0; i < x.size(); i++)
			if (!(std::fabs(static_cast<double>(out[i]) - ref.y[i]) <= bound)) { bad = i; break; }
		INFO("bound " << bound << " LSB, first violation " << bad);
		CHECK(bad == x.size());
		io += b;
		co += n * stride;
	}
}

}  // namespace


// Undefined behaviour reached by ordinary data (UBSan stops at the first finding; the intended result occurs on the host, so the cases are
// KNOWN_ISSUE_UB_HIDDEN: plain passing cases normally, skipped in sanitizer builds). Evidence: UBSan run of every case with --no-skip.
#define UB_S31 KNOWN_ISSUE_UB_HIDDEN("B-DFBQ-1: arm_biquad_cascade_df1_q31.c:384, :400, :416, :432 `acc_h << uShift` shifts a negative q31 left in the unrolled loop (blocks of 4 or more samples)")
#define UB_F31 KNOWN_ISSUE_UB_HIDDEN("B-DFBQ-2: arm_biquad_cascade_df1_fast_q31.c:130, :156, :182, :214, :260 `acc << shift` and the macro multAcc_32x32_keep32_R (dsp/none.h:186, `((q63_t) a) << 32`, reached from e.g. :121, :144, :147, :248) shift a negative q31 left")
#define UB_X64 KNOWN_ISSUE_UB_HIDDEN("B-DFBQ-3: arm_biquad_cascade_df1_32x64_q31.c e.g. :248, :257, :594, :603, :629, :638, :665, :674, :699, :708, :768, :777 `acc << shift` / `acc_h << uShift` shift a negative value left (and overflow int64 for accumulators above 2^62)")
#define UB_F15 KNOWN_ISSUE_UB_HIDDEN("B-DFBQ-4: arm_math_memory.h:105 `pQ15[1] << 16` (read_q15x2 without __ARM_FEATURE_UNALIGNED) and dsp/none.h e.g. :460, :531 `(q31_t)x << 16` (__SMLAD / __SMUAD emulation) shift negative or too large values left")

#define ALL5(REQ, STMT, FN) \
	TEST_CASE("[" REQ " arm_biquad_cascade_df1_q31] " STMT * UB_S31) { FN<Std31>(); } \
	TEST_CASE("[" REQ " arm_biquad_cascade_df1_fast_q31] " STMT * UB_F31) { FN<Fast31>(); } \
	TEST_CASE("[" REQ " arm_biquad_cas_df1_32x64_q31] " STMT * UB_X64) { FN<X64>(); } \
	TEST_CASE("[" REQ " arm_biquad_cascade_df1_q15] " STMT) { FN<Std15>(); } \
	TEST_CASE("[" REQ " arm_biquad_cascade_df1_fast_q15] " STMT * UB_F15) { FN<Fast15>(); }
// the fast variants are compared bit for bit with a model derived from the library code (see header): named characterization
#define ALL5_FASTCH(REQ, STMT, FN) \
	TEST_CASE("[" REQ " arm_biquad_cascade_df1_q31] " STMT * UB_S31) { FN<Std31>(); } \
	TEST_CASE("characterization: [" REQ " arm_biquad_cascade_df1_fast_q31] " STMT * UB_F31) { FN<Fast31>(); } \
	TEST_CASE("[" REQ " arm_biquad_cas_df1_32x64_q31] " STMT * UB_X64) { FN<X64>(); } \
	TEST_CASE("[" REQ " arm_biquad_cascade_df1_q15] " STMT) { FN<Std15>(); } \
	TEST_CASE("characterization: [" REQ " arm_biquad_cascade_df1_fast_q15] " STMT * UB_F15) { FN<Fast15>(); }
// DOC is silent: all five characterization
#define ALL5_CH(REQ, STMT, FN) \
	TEST_CASE("characterization: [" REQ " arm_biquad_cascade_df1_q31] " STMT * UB_S31) { FN<Std31>(); } \
	TEST_CASE("characterization: [" REQ " arm_biquad_cascade_df1_fast_q31] " STMT * UB_F31) { FN<Fast31>(); } \
	TEST_CASE("characterization: [" REQ " arm_biquad_cas_df1_32x64_q31] " STMT * UB_X64) { FN<X64>(); } \
	TEST_CASE("characterization: [" REQ " arm_biquad_cascade_df1_q15] " STMT) { FN<Std15>(); } \
	TEST_CASE("characterization: [" REQ " arm_biquad_cascade_df1_fast_q15] " STMT * UB_F15) { FN<Fast15>(); }
#define ALL5_CLEAN(REQ, STMT, FN) \
	TEST_CASE("[" REQ " arm_biquad_cascade_df1_q31] " STMT) { FN<Std31>(); } \
	TEST_CASE("[" REQ " arm_biquad_cascade_df1_fast_q31] " STMT) { FN<Fast31>(); } \
	TEST_CASE("[" REQ " arm_biquad_cas_df1_32x64_q31] " STMT) { FN<X64>(); } \
	TEST_CASE("[" REQ " arm_biquad_cascade_df1_q15] " STMT) { FN<Std15>(); } \
	TEST_CASE("[" REQ " arm_biquad_cascade_df1_fast_q15] " STMT) { FN<Fast15>(); }

TEST_SUITE("DSP Filtering biquad q") {

// ===============================================================================================================
// C1: ARM's test ids
// ===============================================================================================================
TEST_CASE("[C1 arm_biquad_cascade_df1_q31] ARM's 3-stage reference (two blocks of 50, postShift 2) is matched within 115 dB SNR and 1000 LSB" * UB_S31) {
	const std::vector<int32_t> in = dsp::loadPattern(std::string(kDir31) + "BiquadInput1_q31.txt").q31();
	const std::vector<int32_t> c = dsp::loadPattern(std::string(kDir31) + "BiquadCoefs1_q31.txt").q31();
	const std::vector<int32_t> ref = dsp::loadPattern(std::string(kDir31) + "BiquadOutput1_q31.txt").q31();
	REQUIRE(c.size() == 15);
	REQUIRE(in.size() == ref.size());
	dsp::GuardedBuffer<int32_t> state(12);
	arm_biquad_casd_df1_inst_q31 S;
	arm_biquad_cascade_df1_init_q31(&S, 3, c.data(), state.data(), 2);
	dsp::GuardedBuffer<int32_t> out(ref.size());
	const size_t bs = in.size() >> 1;
	arm_biquad_cascade_df1_q31(&S, in.data(), out.data(), static_cast<uint32_t>(bs));
	arm_biquad_cascade_df1_q31(&S, in.data() + bs, out.data() + bs, static_cast<uint32_t>(bs));
	CHECK(out.guardIntact());
	CHECK(state.guardIntact());
	const std::vector<int32_t> o = out.output();
	CHECK(dsp::snr(ref, o) >= 115.0);
	auto k = dsp::withinLsb(ref, o, 1000);
	CHECK_MESSAGE(k, k.message);
}

TEST_CASE("[C1 arm_biquad_cas_df1_32x64_q31] ARM's 3-stage reference processed sample by sample is matched within 140 dB SNR and 25 LSB and writes nothing behind the output" * UB_X64) {
	const std::vector<int32_t> in = dsp::loadPattern(std::string(kDir31) + "BiquadInput1_q31.txt").q31();
	const std::vector<int32_t> c = dsp::loadPattern(std::string(kDir31) + "BiquadCoefs1_q31.txt").q31();
	const std::vector<int32_t> ref = dsp::loadPattern(std::string(kDir31) + "BiquadOutput1_q31.txt").q31();
	dsp::GuardedBuffer<int64_t> state(12);
	arm_biquad_cas_df1_32x64_ins_q31 S;
	arm_biquad_cas_df1_32x64_init_q31(&S, 3, c.data(), state.data(), 2);
	std::vector<int32_t> out(ref.size() + 4, 0);   // ARM's checkInnerTail: the 4 words behind the written part stay 0
	for (size_t k = 0; k < in.size(); k++) {
		arm_biquad_cas_df1_32x64_q31(&S, in.data() + k, out.data() + k, 1);
		for (size_t t = 1; t <= 4; t++) CHECK(out[k + t] == 0);   // ARM checkInnerTail: the 4 words behind the sample just written (the next call writes out[k+1])
	}
	CHECK(out[ref.size()] == 0);
	CHECK(state.guardIntact());
	out.resize(ref.size());
	CHECK(dsp::snr(ref, out) >= 140.0);
	auto k = dsp::withinLsb(ref, out, 25);
	CHECK_MESSAGE(k, k.message);
}

TEST_CASE("[C1 arm_biquad_cascade_df1_q15] ARM's 3-stage reference (two blocks of 50, postShift 2) is matched within 30 dB SNR and 500 LSB") {
	const std::vector<int16_t> in = dsp::loadPattern(std::string(kDir15) + "BiquadInput1_q15.txt").q15();
	const std::vector<int16_t> c = dsp::loadPattern(std::string(kDir15) + "BiquadCoefs1_q15.txt").q15();
	const std::vector<int16_t> ref = dsp::loadPattern(std::string(kDir15) + "BiquadOutput1_q15.txt").q15();
	REQUIRE(c.size() == 18);
	REQUIRE(in.size() == ref.size());
	for (size_t k = 0; k < 3; k++) CHECK(c[6 * k + 1] == 0);   // the pad coefficient of the DOC layout is 0
	dsp::GuardedBuffer<int16_t> state(12);
	arm_biquad_casd_df1_inst_q15 S;
	arm_biquad_cascade_df1_init_q15(&S, 3, c.data(), state.data(), 2);
	dsp::GuardedBuffer<int16_t> out(ref.size());
	const size_t bs = in.size() >> 1;
	arm_biquad_cascade_df1_q15(&S, in.data(), out.data(), static_cast<uint32_t>(bs));
	arm_biquad_cascade_df1_q15(&S, in.data() + bs, out.data() + bs, static_cast<uint32_t>(bs));
	CHECK(out.guardIntact());
	CHECK(state.guardIntact());
	const std::vector<int16_t> o = out.output();
	CHECK(dsp::snr(ref, o) >= 30.0);
	auto k = dsp::withinLsb(ref, o, 500);
	CHECK_MESSAGE(k, k.message);
}

// ===============================================================================================================
// C2 / C4: init, state, blocks
// ===============================================================================================================
ALL5_CLEAN("C4", "init stores numStages, pCoeffs, pState and postShift, zeroes exactly 4 * numStages state words (1 to 9 stages) and leaves the coefficients", initFields)
ALL5("C2", "state buffers of exactly 4 * numStages words keep their guard words behind them over blocks of 1, 4, 5 and 9 samples (1, 3 and 9 stages)", stateGuard)
ALL5("C4", "re-init clears the state so that the same input gives the same output again", reinitResets)
ALL5("C4", "two instances with the same coefficients do not influence each other", independent)
ALL5("C4", "consecutive blocks of 1, 2, 3, 4, 5, 7 and mixed sizes (1, 2, 3, 5 and 9 stages) give the same output and state as one block", continuity)
ALL5_CH("C4", "a block of size 0 changes neither the state nor the following output and writes no output (DOC silent)", emptyBlock)
ALL5_FASTCH("C4", "the state array holds {x[n-1], x[n-2], y[n-1], y[n-2]} of each stage", stateLayout)
ALL5_FASTCH("C2", "block sizes 1 to 9 write exactly blockSize samples (guard words behind the output intact) and match the integer model", outputGuard)

// ===============================================================================================================
// C3: difference equation
// ===============================================================================================================
ALL5_FASTCH("C3", "random stable cascades of 1, 2, 3, 4, 5, 7 and 9 stages, block sizes 13 and 64, postShift 0, 2 and the maximum match the integer model bit for bit, including the state", exactStable)
ALL5_FASTCH("C3", "unstable full range coefficients (saturation of q15, wrap of q31) match the integer model bit for bit", exactWide)
ALL5("C3", "output stays within the derived error bound of the double difference equation (1, 2, 3, 5 and 9 stages)", defBound)

TEST_CASE("[C3 arm_biquad_cascade_df1_q31] with non-negative coefficients the floor error against the double DEF lies in [-bound, 0]" * UB_S31) { oneSided<Std31>(); }
TEST_CASE("[C3 arm_biquad_cas_df1_32x64_q31] with non-negative coefficients the floor error against the double DEF lies in [-bound, 0]" * UB_X64) { oneSided<X64>(); }
TEST_CASE("[C3 arm_biquad_cascade_df1_q15] with non-negative coefficients the floor error against the double DEF lies in [-bound, 0]") { oneSided<Std15>(); }
TEST_CASE("[C3 arm_biquad_cascade_df1_fast_q15] with non-negative coefficients the floor error against the double DEF lies in [-bound, 0]" * UB_F15) { oneSided<Fast15>(); }

TEST_CASE("[C3 arm_biquad_cascade_df1_q31] stage 1 delaying by one and stage 2 by two samples move an impulse to index 3 exactly") { delayCascade<Std31>(); }
TEST_CASE("[C3 arm_biquad_cascade_df1_fast_q31] stage 1 delaying by one and stage 2 by two samples move an impulse to index 3 exactly") { delayCascade<Fast31>(); }
TEST_CASE("[C3 arm_biquad_cas_df1_32x64_q31] stage 1 delaying by one and stage 2 by two samples move an impulse to index 3 exactly") { delayCascade<X64>(); }
TEST_CASE("[C3 arm_biquad_cascade_df1_q15] stage 1 delaying by one and stage 2 by two samples move an impulse to index 3 exactly") { delayCascade<Std15>(); }
TEST_CASE("[C3 arm_biquad_cascade_df1_fast_q15] stage 1 delaying by one and stage 2 by two samples move an impulse to index 3 exactly" * UB_F15) { delayCascade<Fast15>(); }

TEST_CASE("[C3 arm_biquad_cascade_df1_q31] impulse response of the one-pole stage b0 = a1 = 0.5 halves exactly (2^(27-n) for n = 0..27)") { onePole<Std31>(28); }
TEST_CASE("[C3 arm_biquad_cas_df1_32x64_q31] impulse response of the one-pole stage b0 = a1 = 0.5 halves exactly (2^(27-n) for n = 0..27)") { onePole<X64>(28); }
TEST_CASE("[C3 arm_biquad_cascade_df1_fast_q31] impulse response of the one-pole stage b0 = a1 = 0.5 halves exactly for n = 0..24 (the rounded products are still exact)") { onePole<Fast31>(25); }
TEST_CASE("[C3 arm_biquad_cascade_df1_q15] impulse response of the one-pole stage b0 = a1 = 0.5 halves exactly (2^(11-n) for n = 0..11)") { onePole<Std15>(12); }
TEST_CASE("[C3 arm_biquad_cascade_df1_fast_q15] impulse response of the one-pole stage b0 = a1 = 0.5 halves exactly (2^(11-n) for n = 0..11)" * UB_F15) { onePole<Fast15>(12); }

// s = 20: y[n] = 2^19 - 2^(18-n) for n <= 18 (all integers), then floor(2^18 + 0.5 (2^19 - 1)) = 2^19 - 1 stays (fixed point one LSB below the ideal 2^19)
TEST_CASE("[C3 arm_biquad_cascade_df1_q31] step response of y = 0.25 x + 0.5 y[n-1] with x = 2^20 reaches the floor fixed point 2^19 - 1") { stepResponse<Std31>(20, 30); }
TEST_CASE("[C3 arm_biquad_cas_df1_32x64_q31] step response of y = 0.25 x + 0.5 y[n-1] with x = 2^20 reaches 2^19 - 1 (floor of the exact 2^19 - 2^(18-n))") { stepResponse<X64>(20, 30); }
// q15 s = 6: y[n] = 2^5 - 2^(4-n) for n <= 4 (16, 24, 28, 30, 31), then 31
TEST_CASE("[C3 arm_biquad_cascade_df1_q15] step response of y = 0.25 x + 0.5 y[n-1] with x = 64 reaches the floor fixed point 31") { stepResponse<Std15>(6, 12); }
TEST_CASE("[C3 arm_biquad_cascade_df1_fast_q15] step response of y = 0.25 x + 0.5 y[n-1] with x = 64 reaches the floor fixed point 31" * UB_F15) { stepResponse<Fast15>(6, 12); }

TEST_CASE("[C3 arm_biquad_cascade_df1_q31] the single tap 0.5 floors +-1, +-2 and +-3 (0, -1, 1, -1, 1, -2), not toward zero" * UB_S31) { floorOfOne<Std31>(); }
TEST_CASE("[C3 arm_biquad_cas_df1_32x64_q31] the single tap 0.5 floors +-1, +-2 and +-3 (0, -1, 1, -1, 1, -2), not toward zero" * UB_X64) { floorOfOne<X64>(); }
TEST_CASE("[C3 arm_biquad_cascade_df1_q15] the single tap 0.5 floors +-1, +-2 and +-3 (0, -1, 1, -1, 1, -2), not toward zero") { floorOfOne<Std15>(); }
TEST_CASE("[C3 arm_biquad_cascade_df1_fast_q15] the single tap 0.5 floors +-1, +-2 and +-3 (0, -1, 1, -1, 1, -2), not toward zero" * UB_F15) { floorOfOne<Fast15>(); }

// sweeps. q31: coefficients 0x7FFFFFFF (real 1 - 2^-31 at p = 0) and -0x3FFFFFFF / p = 1, stride 65521 over the whole int32 range plus INT_MAX.
// q15: every input value (exhaustive). For q15 the DEF is clamp(floor(c x / 2^(15-p))).
TEST_CASE("[C3 arm_biquad_cascade_df1_q31] single tap 0x7FFFFFFF over the strided full int32 range is floor(c x / 2^31): error in (-1, 0]" * UB_S31) { sweepSingleTap<Std31>(0x7FFFFFFFLL, 0); }
TEST_CASE("[C3 arm_biquad_cascade_df1_q31] single tap -0x3FFFFFFF at postShift 1 over the strided full int32 range has error in (-1, 0]" * UB_S31) { sweepSingleTap<Std31>(-0x3FFFFFFFLL, 1); }
TEST_CASE("[C3 arm_biquad_cas_df1_32x64_q31] single tap 0x7FFFFFFF over the strided full int32 range is floor(c x / 2^31): error in (-1, 0]" * UB_X64) { sweepSingleTap<X64>(0x7FFFFFFFLL, 0); }
TEST_CASE("[C3 arm_biquad_cas_df1_32x64_q31] single tap -0x3FFFFFFF at postShift 1 over the strided full int32 range has error in (-1, 0]" * UB_X64) { sweepSingleTap<X64>(-0x3FFFFFFFLL, 1); }
TEST_CASE("[C3 arm_biquad_cascade_df1_fast_q31] single tap 0x20000001 at postShift 1 over inputs in [-0.25, 0.25) FS is within 2^(p+1)/2 LSB of the real product" * UB_F31) {
	// inputs inside the documented [-0.25, +0.25) range, real gain 0.5: no wrap of the shifted accumulator
	const std::vector<Stage> st = {{0x20000001LL, 0, 0, 0, 0}};
	std::vector<int32_t> in;
	for (i64 x = -(i64(1) << 29); x < (i64(1) << 29); x += 32749) in.push_back(static_cast<int32_t>(x));   // |x| < 0.25 FS (DOC precondition of the fast function)
	Biq<Fast31> f(st, 1);
	const std::vector<int32_t> out = f.process(in);
	size_t bad = in.size();
	for (size_t n = 0; n < in.size(); n++) {
		const double ideal = std::ldexp(536870913.0 * in[n], -30);
		if (!(std::fabs(out[n] - ideal) <= 2.0 + 1e-3)) { bad = n; break; }
	}
	CHECK(bad == in.size());
}
TEST_CASE("[C3 arm_biquad_cascade_df1_q15] every input value through the single tap 0x7FFF at postShift 0 is floor(c x / 2^15)") { sweepSingleTap<Std15>(0x7FFF, 0); }
TEST_CASE("[C3 arm_biquad_cascade_df1_q15] every input value through the single tap -0x5555 at postShift 2 is the saturated floor") { sweepSingleTap<Std15>(-0x5555, 2); }
TEST_CASE("[C3 arm_biquad_cascade_df1_fast_q15] every input value through the single tap 0x7FFF at postShift 0 is floor(c x / 2^15)" * UB_F15) { sweepSingleTap<Fast15>(0x7FFF, 0); }
TEST_CASE("[C3 arm_biquad_cascade_df1_fast_q15] every input value through the single tap -0x5555 at postShift 2 is the saturated floor" * UB_F15) { sweepSingleTap<Fast15>(-0x5555, 2); }

// saturation boundary of q15 (DOC of the std function: "the result is saturated to 1.15 format"). Gain g = 0x4001 / 2^14 = 1 + 2^-14 at p = 1:
// y = x + floor(x / 2^14): x = 32766 -> 32767 (exactly at the limit), 32767 -> 32768 saturates to 32767, -32766 -> -32768 (exactly), -32767 -> -32769 and
// -32768 -> -32770 saturate to -32768; and identity (0x4000) is exact for every x including both ends.
template <class Tr>
void q15Boundary() {
	{
		const std::vector<Stage> st = {{0x4001, 0, 0, 0, 0}};
		Biq<Tr> f(st, 1);
		const std::vector<int16_t> out = f.process({32766, 32767, -32766, -32767, -32768, 16383, 16384});
		// 16383 + floor(0.99994) = 16383, 16384 + 1 = 16385
		CHECK(out == std::vector<int16_t>({32767, 32767, -32768, -32768, -32768, 16383, 16385}));
	}
	{
		const std::vector<Stage> st = {{0x4000, 0, 0, 0, 0}};
		Biq<Tr> f(st, 1);
		std::vector<int16_t> in;
		for (int x = -32768; x <= 32767; x++) in.push_back(static_cast<int16_t>(x));
		CHECK(f.process(in) == in);
	}
}
TEST_CASE("[C3 arm_biquad_cascade_df1_q15] gain 1 + 2^-14 saturates at both ends of the q15 range exactly where derived and gain 1 is the identity for every input") { q15Boundary<Std15>(); }
TEST_CASE("[C3 arm_biquad_cascade_df1_fast_q15] gain 1 + 2^-14 saturates at both ends of the q15 range exactly where derived and gain 1 is the identity for every input" * UB_F15) { q15Boundary<Fast15>(); }

// q31 wrap, DOC "if the accumulator result overflows it wraps around rather than clip". p = 0, b0 = b1 = 0x7FFFFFFF, x = 0x7FFFFFFF twice (block of 2:
// the tail path): y0: acc = (2^31-1)^2 = 2^62 - 2^32 + 1, >> 31 = 2^31 - 2 (0x7FFFFFFE). y1: acc = 2 (2^31-1)^2 = 2^63 - 2^33 + 2 (fits int64),
// >> 31 = 2^32 - 4 whose low 32 bits are 0xFFFFFFFC = -4 (an overflowed result wraps to a small NEGATIVE value, it does not clip to 0x7FFFFFFF).
template <class Tr>
void q31Wrap() {
	const std::vector<Stage> st = {{0x7FFFFFFFLL, 0x7FFFFFFFLL, 0, 0, 0}};
	Biq<Tr> f(st, 0);
	CHECK(f.process({0x7FFFFFFF, 0x7FFFFFFF}) == std::vector<int32_t>({0x7FFFFFFE, -4}));
}
TEST_CASE("[C4 arm_biquad_cascade_df1_q31] an accumulator above 2^62 wraps around instead of clipping (0x7FFFFFFE, then -4)") { q31Wrap<Std31>(); }
TEST_CASE("[C4 arm_biquad_cas_df1_32x64_q31] an accumulator above 2^62 wraps around instead of clipping (0x7FFFFFFE, then -4)" * UB_X64) { q31Wrap<X64>(); }

// ===============================================================================================================
// C5: agreement with f32 and with ARM's other data
// ===============================================================================================================
ALL5("C5", "agrees with arm_biquad_cascade_df1_f32 on identical data within the derived own + f32 rounding bound", againstF32)

TEST_CASE("[C5 arm_biquad_cascade_df1_q31] ARM's reference input and coefficients stay within the derived bound of the double DEF" * UB_S31) { armDataVsDef<Std31>(kDir31, "q31"); }
TEST_CASE("[C5 arm_biquad_cascade_df1_fast_q31] ARM's reference input and coefficients stay within the derived bound of the double DEF" * UB_F31) { armDataVsDef<Fast31>(kDir31, "q31"); }
TEST_CASE("[C5 arm_biquad_cas_df1_32x64_q31] ARM's reference input and coefficients stay within the derived bound of the double DEF" * UB_X64) { armDataVsDef<X64>(kDir31, "q31"); }
TEST_CASE("[C5 arm_biquad_cascade_df1_q15] ARM's reference input and coefficients stay within the derived bound of the double DEF") { armDataVsDef<Std15>(kDir15, "q15"); }
TEST_CASE("[C5 arm_biquad_cascade_df1_fast_q15] ARM's reference input and coefficients stay within the derived bound of the double DEF" * UB_F15) { armDataVsDef<Fast15>(kDir15, "q15"); }

TEST_CASE("[C5 arm_biquad_cascade_df1_q31] ARM's unused AllBiquad random patterns (9 configs) stay within the derived bound of the double DEF" * UB_S31) { allPatternsVsDef<Std31>(kDir31, "q31"); }
TEST_CASE("[C5 arm_biquad_cascade_df1_fast_q31] ARM's unused AllBiquad random patterns (9 configs) stay within the derived bound of the double DEF" * UB_F31) { allPatternsVsDef<Fast31>(kDir31, "q31"); }
TEST_CASE("[C5 arm_biquad_cas_df1_32x64_q31] ARM's unused AllBiquad random patterns (9 configs) stay within the derived bound of the double DEF" * UB_X64) { allPatternsVsDef<X64>(kDir31, "q31"); }
TEST_CASE("[C5 arm_biquad_cascade_df1_q15] ARM's unused AllBiquad random patterns (9 configs) stay within the derived bound of the double DEF") { allPatternsVsDef<Std15>(kDir15, "q15"); }
TEST_CASE("[C5 arm_biquad_cascade_df1_fast_q15] ARM's unused AllBiquad random patterns (9 configs) stay within the derived bound of the double DEF" * UB_F15) { allPatternsVsDef<Fast15>(kDir15, "q15"); }

}  // TEST_SUITE
