/*
 * CMSIS-DSP Filtering, decimation and interpolation, fixed point (work order DECQ, phase P11 group G4):
 *   arm_fir_decimate_q31, arm_fir_decimate_fast_q31, arm_fir_decimate_init_q31,
 *   arm_fir_decimate_q15, arm_fir_decimate_fast_q15, arm_fir_decimate_init_q15,
 *   arm_fir_interpolate_q31, arm_fir_interpolate_init_q31, arm_fir_interpolate_q15, arm_fir_interpolate_init_q15.
 * Requirement ids: Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * ARM tests (C1): Libraries/CMSIS-DSP/Testing/Source/Tests/DECIMQ31.cpp and DECIMQ15.cpp, suites "DECIM Q31" / "DECIM Q15"
 * (Testing/desc.txt:2222-2245 and :2246-2269). Ids: test_fir_decimate_q31 (DECIMQ31.cpp:19-70), test_fir_interpolate_q31 (:74-125),
 * test_fir_decimate_q15 (DECIMQ15.cpp:19-67), test_fir_interpolate_q15 (:71-122). Nothing is commented out or disabled; there is
 * NO ARM test of the fast variants (arm_fir_decimate_fast_q31 / _q15): they are covered by the DEF only (C3 / C5).
 * Pattern files (Testing/Patterns/DSP/Filtering/DECIM/DECIMQ31/ and DECIMQ15/):
 *   decimate:    Configs2_u32 (4 words per config: M, numTaps, blockSize, refsize = blockSize / M; 72 configs), Input2 (the
 *                blockSize inputs of all configs concatenated), Coefs2 (the numTaps coefficients of all configs concatenated),
 *                Reference2 (all outputs concatenated);
 *   interpolate: Configs3_u32 (L, numTaps, blockSize, refsize = blockSize * L; 36 configs), Input3, Coefs3, Reference3.
 * For every config ARM calls init(numTaps, M or L, coefs, state, blockSize) and ONE processing call of blockSize samples, state
 * buffer re-initialised for each config, output blocks concatenated, then ASSERT_EMPTY_TAIL, ASSERT_SNR, ASSERT_NEAR_EQ.
 * Same here, with the state buffer of exactly the DOC length and a guard behind every output block.
 * ARM tolerances: q31 SNR_THRESHOLD 100 (DECIMQ31.cpp:5), ABS_ERROR_Q31 2 LSB (:13); q15 SNR_THRESHOLD 70 (DECIMQ15.cpp:5),
 * ABS_ERROR_Q15 5 LSB (:13). Used unchanged (L4). The reference patterns are generated in double precision (comment at :7-:11).
 *
 * DOC and the exact arithmetic (derived from the DOC, then confirmed against the pattern data bit for bit):
 *  - Coefficients are stored time reversed: c[T-1] = b[0] weights the newest sample (arm_fir_decimate_init_q31.c:55-60,
 *    arm_fir_interpolate_init_q31.c:57-61). Decimate output j (j = 0 .. B/M-1) is  y[j] = sum_{i=0}^{T-1} c[i] * X[j*M - (T-1) + i],
 *    X = the input stream including the history (zero before the first sample): output j uses the FIRST sample of its group of M
 *    as the newest one. Interpolate (zero stuffing by L, polyphase length P = T/L): output nL+r (r = 0..L-1) is
 *    y[nL+r] = sum_{m=0}^{P-1} c[L-1-r+m*L] * x[n-(P-1)+m]; with b[i] = c[T-1-i] this is sum_k b[kL+r] x[n-k] = the zero stuffed
 *    input filtered by b. Output counts: B/M and B*L.
 *  - q31 standard (decimate :48-55, interpolate :48-56): 64 bit accumulator in 2.62 format of the full products (31 + 31 fractional
 *    bits), "truncated to 1.32 format and then saturated to 1.31 format": V = floor(S / 2^31) with S = sum c*x (exact), result =
 *    clamp(V, -2^31, 2^31-1). floor = arithmetic shift, so the error against the ideal S/2^31 is one-sided: 0 <= S/2^31 - V < 1.
 *    The exact integer S is formed here without a 128 bit type: c = ch*2^16 + cl (ch = c >> 16, cl = c & 0xFFFF), hi = sum x*ch,
 *    lo = sum x*cl (both exact in int64 for the tap counts used), floor(S/2^31) = (hi + (lo >> 16)) >> 15 (nested floor division).
 *  - q31 fast (arm_fir_decimate_fast_q31.c:48-58): each product is truncated to 2.30: floor(c*x / 2^32), summed in 32 bit, the
 *    result is acc * 2 (1.31). So V_fast = 2 * sum floor(c*x/2^32): one-sided as well: S/2^31 - 2T < V_fast <= S/2^31 (every
 *    floor loses < 1 unit of 2^-32, i.e. < 2 units of the result), and V_fast <= V_standard < V_fast + 2T.
 *  - q15 standard (decimate :48-56, interpolate :48-56): 64 bit accumulator in 34.30 of the exact products (2.30 each),
 *    "truncated to 34.15 format by discarding low 15 bits. Lastly, the accumulator is saturated to yield a result in 1.15 format":
 *    V = floor(S / 2^15), result = clamp(V, -32768, 32767). Error against S/2^15: 0 <= S/2^15 - V < 1.
 *  - q15 fast (arm_fir_decimate_fast_q15.c:48-55): 32 bit accumulator in 2.30, "if the accumulator result overflows it wraps
 *    around and distorts the result", then floor(acc/2^15) saturated. While |S| < 2^31 it is identical to the standard function. The
 *    test data keeps |S| below 2^31 (amplitude limit amp(): |x| <= 65535/T for the fast variant); the wrap itself is not
 *    tested: the host code (no ARM_MATH_DSP) does "acc0 += x0 * c0" in int, a signed overflow (UB), and the DOC gives no value.
 *  The standard q31 functions and the fast q31 function do NOT saturate the final narrowing: they cast (q31_t)(acc >> 31) /
 *  (q31_t)(acc << 1), which wraps. The DOC sentence for THIS step says "saturated": decimate_q31.c:55 and interpolate_q31.c:56
 *  "After all multiply-accumulates are performed, the 2.62 accumulator is truncated to 1.32 format and then saturated to 1.31 format.",
 *  fast_q31.c:~51 "Finally, the accumulator is saturated and converted to a 1.31 result.". The other DOC sentence ("if the accumulator
 *  result overflows it wraps around rather than clip", :53) is about the int64 accumulator, which does not overflow in these cases (the
 *  true result is only out of the 1.31 range). The tests that reach the narrowing with an out of range result are therefore
 *  `characterization:` tests (values hand derived in the comments): NEEDS REVIEW, "DOC says saturated, library wraps the narrowing;
 *  suspected DOC/vendor defect, not decided". Earlier defect ids B-DDECQ-1..3 were dropped (lead decision); ids -4 and -5 stay (the lead
 *  renumbers). A real accumulator wrap (int64 / the 32 bit q15 fast accumulator) is not tested: signed overflow, UB, no DOC value.
 *  The negative fast q31 wrap case is not tested either (it left shifts a negative value, B-DDECQ-4). The 4-way unrolled code is pinned by
 *  9-output calls (two groups of 4 and a tail) with M / L = 1, 2, 3, 5. Random test data of the q31 functions is
 *  scaled to |x| <= (2^31-1)/T so that the result never leaves the range (|V| <= T*|x|*|c|/2^31 < 2^31).
 *
 * Sanitizer coverage gap (hidden UB, cases skipped in sanitizer builds): arm_fir_decimate_fast_q31 with negative data (B-DDECQ-4:
 * 13 cases) and arm_fir_interpolate_q15 with phaseLength >= 5 and negative state samples (B-DDECQ-5: 10 cases) have no sanitizer
 * coverage for negative data; the non-negative variants of random / continuity cases and phaseLength < 5 cases stay active there.
 *
 * State overrun visibility: every run through doRun() is done twice, with a GuardedBuffer state (guards behind) and with a tight
 * `new S[n]` state (no slack, n = DOC length): ASan reports a write one past the end or before the start; outputs must be equal.
 *
 * DEF in double (C5): D = sum (double c * double x) / 2^fb in double. Error of D: every product has relative error 2^-53 (|c*x| <=
 * 2^62, so at most 2^62*2^-53 = 2^9 absolute, 2^-22 LSB of the q31 result after dividing by 2^31), every addition rounds a partial
 * sum of magnitude <= T*2^31 LSB by at most T*2^31*2^-53 = T*2^-22 LSB: |D - S/2^fb| <= 2T * 2^-22 LSB < T * 2^-21 (q15: exact).
 * eps = T * 2^-21 is used. The tests then assert  out <= D + eps  and  out > D - slack - eps  (slack 1, or 2T for the q31 fast
 * variant), i.e. truncation is asserted one-sided; a round-to-nearest implementation would fail the upper bound.
 *
 * State length formulas (DOC): decimate numTaps + blockSize - 1 (arm_fir_decimate_init_q31.c:62, q15 :62), interpolate
 * (numTaps/L) + blockSize - 1 (arm_fir_interpolate_init_q31.c:63, q15 :64). Every test allocates exactly this (GuardedBuffer, 8
 * guard words behind): sanitizer builds catch an overrun. Init errors: decimate returns ARM_MATH_LENGTH_ERROR when blockSize is
 * not a multiple of M (DOC "- ARM_MATH_LENGTH_ERROR : blockSize is not a multiple of M", init_q31.c:53). It has NO error for numTaps
 * not a multiple of M (accepted). Interpolate: DOC "- ARM_MATH_ARGUMENT_ERROR : filter length numTaps is not a multiple of the
 * interpolation factor L" (init_q31.c:53) but the code returns ARM_MATH_LENGTH_ERROR (init_q31.c:80-81): DOC defect (lead decision),
 * characterization tests assert ARM_MATH_LENGTH_ERROR, NEEDS REVIEW.
 *
 * Host vs target (C6): the host runs the portable C path (no ARM_MATH_DSP, no MVE); the target Cortex-M4 uses SMLALD / SMLAD
 * kernels with the same integer arithmetic (the q15 results are identical, the q31 kernels have no DSP variant). Fixed width types
 * only; right shifts of negative values are arithmetic (GCC and ARM GCC, implementation-defined in C); (q31_t) of an out of range
 * int64 wraps on both. The q31 fast kernel left shifts negative values (see the UBSan notes of the B-DDECQ cases).
 *
 * Not tested, NEEDS REVIEW: blockSize 0 (DOC silent; the init arithmetic numTaps + blockSize - 1 and the kernels are unspecified),
 * numTaps 0, a processing blockSize that is not a multiple of M (undefined: the output count is blockSize/M and the state would
 * be advanced by the wrong amount), an input block larger than the blockSize given to init (state buffer too small).
 */
#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math.h"

namespace {

// Deterministic xorshift32; every call is its own statement (lesson: never two PRNG calls in one expression)
struct Rng {
	uint32_t s;
	explicit Rng(uint32_t seed) : s(seed * 2654435761u + 0x9E3779B9u) {
		if (s == 0) s = 1;
	}
	uint32_t next() {
		s ^= s << 13;
		s ^= s >> 17;
		s ^= s << 5;
		return s;
	}
};

// uniform in [lo, hi] (hi - lo + 1 <= 2^32)
int64_t randIn(Rng &rng, int64_t lo, int64_t hi) {
	const uint32_t r = rng.next();
	return lo + static_cast<int64_t>(static_cast<uint64_t>(r) % static_cast<uint64_t>(hi - lo + 1));
}

// ---------------------------------------------------------------------------------------------------------------
// Arithmetic descriptions (the exact arithmetic of the DOC, see the header)
// ---------------------------------------------------------------------------------------------------------------
struct Q31Std {
	typedef int32_t S;
	static const char *sfx() { return "q31"; }
	static constexpr int fb = 31;
	static constexpr int64_t minV = INT32_MIN, maxV = INT32_MAX;
	static constexpr bool isFast = false;
	// floor(S / 2^31), S = sum c*x exact (hi/lo split, see header)
	static int64_t reduce(const std::vector<S> &w, const std::vector<S> &c) {
		int64_t hi = 0, lo = 0;
		for (size_t i = 0; i < w.size(); i++) {
			const int64_t x = w[i];
			const int64_t ch = static_cast<int64_t>(c[i]) >> 16;
			const int64_t cl = static_cast<int64_t>(c[i]) & 0xFFFF;
			hi += x * ch;
			lo += x * cl;
		}
		return (hi + (lo >> 16)) >> 15;
	}
	static int64_t direct1(S x, S c) { return (static_cast<int64_t>(x) * c) >> 31; }  // |x*c| <= 2^62 fits
	static double slack(size_t) { return 1.0; }
	static int64_t amp(size_t n) { return INT32_MAX / static_cast<int64_t>(n); }
};

struct Q31Fast : Q31Std {
	static constexpr bool isFast = true;
	// 2 * sum floor(c*x / 2^32)
	static int64_t reduce(const std::vector<S> &w, const std::vector<S> &c) {
		int64_t acc = 0;
		for (size_t i = 0; i < w.size(); i++) acc += (static_cast<int64_t>(w[i]) * c[i]) >> 32;
		return 2 * acc;
	}
	static int64_t direct1(S x, S c) { return 2 * ((static_cast<int64_t>(x) * c) >> 32); }
	static double slack(size_t n) { return 2.0 * static_cast<double>(n); }
};

struct Q15Std {
	typedef int16_t S;
	static const char *sfx() { return "q15"; }
	static constexpr int fb = 15;
	static constexpr int64_t minV = INT16_MIN, maxV = INT16_MAX;
	static int64_t reduce(const std::vector<S> &w, const std::vector<S> &c) {
		int64_t s = 0;
		for (size_t i = 0; i < w.size(); i++) s += static_cast<int64_t>(w[i]) * c[i];
		return s >> 15;
	}
	static int64_t direct1(S x, S c) { return (static_cast<int32_t>(x) * static_cast<int32_t>(c)) >> 15; }
	static double slack(size_t) { return 1.0; }
	static int64_t amp(size_t) { return 32768; }
};

struct Q15Fast : Q15Std {
	// |S| = |sum c*x| <= n * amp * 32768 < 2^31  <=>  amp < 65536 / n
	static int64_t amp(size_t n) { return std::min<int64_t>(32768, 65535 / static_cast<int64_t>(n)); }
};

template <class A>
int64_t satV(int64_t v) {
	return v < A::minV ? A::minV : (v > A::maxV ? A::maxV : v);
}

// ---------------------------------------------------------------------------------------------------------------
// Function sets. F is the decimation factor M or the interpolation factor L; t is the number of taps per phase (decimate:
// T = t, interpolate: T = t * L, phaseLength = t).
// ---------------------------------------------------------------------------------------------------------------
template <class A>
struct DecimBase : A {
	typedef typename A::S S;
	static constexpr bool isDecim = true;
	static size_t taps(size_t t, unsigned) { return t; }
	static size_t winLen(size_t T, unsigned) { return T; }
	static size_t outCount(size_t B, unsigned F) { return B / F; }
	static size_t stateLen(size_t T, unsigned, size_t B) { return T + B - 1; }
	static size_t chunk(size_t k, unsigned F) { return k * F; }  // a block of k outputs
	static const char *patId() { return "2"; }
	// window and aligned coefficients of output o (see header)
	static void win(const std::vector<S> &c, const std::vector<S> &x, unsigned M, size_t o, std::vector<S> &w, std::vector<S> &cc) {
		const long T = static_cast<long>(c.size());
		w.assign(c.size(), 0);
		cc = c;
		for (long i = 0; i < T; i++) {
			const long idx = static_cast<long>(o) * M - (T - 1) + i;
			if (idx >= 0 && idx < static_cast<long>(x.size())) w[static_cast<size_t>(i)] = x[static_cast<size_t>(idx)];
		}
	}
};

template <class A>
struct InterpBase : A {
	typedef typename A::S S;
	static constexpr bool isDecim = false;
	static size_t taps(size_t t, unsigned F) { return t * F; }
	static size_t winLen(size_t T, unsigned F) { return T / F; }
	static size_t outCount(size_t B, unsigned F) { return B * F; }
	static size_t stateLen(size_t T, unsigned F, size_t B) { return T / F + B - 1; }
	static size_t chunk(size_t k, unsigned) { return k; }
	static const char *patId() { return "3"; }
	static void win(const std::vector<S> &c, const std::vector<S> &x, unsigned L, size_t o, std::vector<S> &w, std::vector<S> &cc) {
		const long P = static_cast<long>(c.size() / L);
		const long n = static_cast<long>(o / L), r = static_cast<long>(o % L);
		w.assign(static_cast<size_t>(P), 0);
		cc.assign(static_cast<size_t>(P), 0);
		for (long m = 0; m < P; m++) {
			const long idx = n - (P - 1) + m;
			if (idx >= 0) w[static_cast<size_t>(m)] = x[static_cast<size_t>(idx)];
			cc[static_cast<size_t>(m)] = c[static_cast<size_t>(L - 1 - r + m * L)];
		}
	}
};

struct DecQ31 : DecimBase<Q31Std> {
	typedef arm_fir_decimate_instance_q31 Inst;
	static arm_status init(Inst *i, uint16_t T, unsigned F, const S *c, S *st, uint32_t B) {
		return arm_fir_decimate_init_q31(i, T, static_cast<uint8_t>(F), c, st, B);
	}
	static void run(const Inst *i, const S *x, S *y, uint32_t B) { arm_fir_decimate_q31(i, x, y, B); }
	static bool fields(const Inst &i, uint16_t T, unsigned F, const S *c, S *st) {
		return i.numTaps == T && i.M == F && i.pCoeffs == c && i.pState == st;
	}
};
struct DecQ31Fast : DecimBase<Q31Fast> {
	typedef arm_fir_decimate_instance_q31 Inst;
	static arm_status init(Inst *i, uint16_t T, unsigned F, const S *c, S *st, uint32_t B) {
		return arm_fir_decimate_init_q31(i, T, static_cast<uint8_t>(F), c, st, B);
	}
	static void run(const Inst *i, const S *x, S *y, uint32_t B) { arm_fir_decimate_fast_q31(i, x, y, B); }
	static bool fields(const Inst &i, uint16_t T, unsigned F, const S *c, S *st) {
		return i.numTaps == T && i.M == F && i.pCoeffs == c && i.pState == st;
	}
};
struct DecQ15 : DecimBase<Q15Std> {
	typedef arm_fir_decimate_instance_q15 Inst;
	static arm_status init(Inst *i, uint16_t T, unsigned F, const S *c, S *st, uint32_t B) {
		return arm_fir_decimate_init_q15(i, T, static_cast<uint8_t>(F), c, st, B);
	}
	static void run(const Inst *i, const S *x, S *y, uint32_t B) { arm_fir_decimate_q15(i, x, y, B); }
	static bool fields(const Inst &i, uint16_t T, unsigned F, const S *c, S *st) {
		return i.numTaps == T && i.M == F && i.pCoeffs == c && i.pState == st;
	}
};
struct DecQ15Fast : DecimBase<Q15Fast> {
	typedef arm_fir_decimate_instance_q15 Inst;
	static arm_status init(Inst *i, uint16_t T, unsigned F, const S *c, S *st, uint32_t B) {
		return arm_fir_decimate_init_q15(i, T, static_cast<uint8_t>(F), c, st, B);
	}
	static void run(const Inst *i, const S *x, S *y, uint32_t B) { arm_fir_decimate_fast_q15(i, x, y, B); }
	static bool fields(const Inst &i, uint16_t T, unsigned F, const S *c, S *st) {
		return i.numTaps == T && i.M == F && i.pCoeffs == c && i.pState == st;
	}
};
struct IntQ31 : InterpBase<Q31Std> {
	typedef arm_fir_interpolate_instance_q31 Inst;
	static arm_status init(Inst *i, uint16_t T, unsigned F, const S *c, S *st, uint32_t B) {
		return arm_fir_interpolate_init_q31(i, static_cast<uint8_t>(F), T, c, st, B);
	}
	static void run(const Inst *i, const S *x, S *y, uint32_t B) { arm_fir_interpolate_q31(i, x, y, B); }
	static bool fields(const Inst &i, uint16_t T, unsigned F, const S *c, S *st) {
		return i.L == F && i.phaseLength == T / F && i.pCoeffs == c && i.pState == st;
	}
};
struct IntQ15 : InterpBase<Q15Std> {
	typedef arm_fir_interpolate_instance_q15 Inst;
	static arm_status init(Inst *i, uint16_t T, unsigned F, const S *c, S *st, uint32_t B) {
		return arm_fir_interpolate_init_q15(i, static_cast<uint8_t>(F), T, c, st, B);
	}
	static void run(const Inst *i, const S *x, S *y, uint32_t B) { arm_fir_interpolate_q15(i, x, y, B); }
	static bool fields(const Inst &i, uint16_t T, unsigned F, const S *c, S *st) {
		return i.L == F && i.phaseLength == T / F && i.pCoeffs == c && i.pState == st;
	}
};

template <class Tr>
std::string patDir() {
	return std::string("Filtering/DECIM/DECIM") + (Tr::fb == 31 ? "Q31/" : "Q15/");
}

template <class Tr>
std::vector<typename Tr::S> loadVec(const std::string &name) {
	const dsp::Pattern p = dsp::loadPattern(patDir<Tr>() + name + "_" + Tr::sfx() + ".txt");
	std::vector<typename Tr::S> v;
	if (Tr::fb == 31) {
		const std::vector<int32_t> t = p.q31();
		v.assign(t.begin(), t.end());
	} else {
		const std::vector<int16_t> t = p.q15();
		v.assign(t.begin(), t.end());
	}
	return v;
}

// ---------------------------------------------------------------------------------------------------------------
// DEF and runners
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
std::vector<typename Tr::S> defOut(const std::vector<typename Tr::S> &c, const std::vector<typename Tr::S> &x, unsigned F) {
	typedef typename Tr::S S;
	const size_t total = Tr::outCount(x.size(), F);
	std::vector<S> y(total);
	std::vector<S> w, cc;
	for (size_t o = 0; o < total; o++) {
		Tr::win(c, x, F, o, w, cc);
		y[o] = static_cast<S>(satV<Tr>(Tr::reduce(w, cc)));
	}
	return y;
}

// D = sum (double c * double x) / 2^fb, in double (error bound in the header)
template <class Tr>
std::vector<double> defDouble(const std::vector<typename Tr::S> &c, const std::vector<typename Tr::S> &x, unsigned F) {
	typedef typename Tr::S S;
	const size_t total = Tr::outCount(x.size(), F);
	std::vector<double> d(total);
	std::vector<S> w, cc;
	for (size_t o = 0; o < total; o++) {
		Tr::win(c, x, F, o, w, cc);
		double acc = 0.0;
		for (size_t i = 0; i < w.size(); i++) acc += static_cast<double>(w[i]) * static_cast<double>(cc[i]);
		d[o] = std::ldexp(acc, -Tr::fb);
	}
	return d;
}

template <class Tr>
struct RunRes {
	std::vector<typename Tr::S> out;
	bool guards = true;  // every output guard and the exact-size state guard intact
};

// One pass over x with the given state buffer (stateLen elements). Returns false if init failed or an output guard is damaged.
template <class Tr>
bool runPass(const std::vector<typename Tr::S> &c, const std::vector<typename Tr::S> &x, unsigned F, const std::vector<size_t> &chunks,
             size_t maxChunk, typename Tr::S *state, std::vector<typename Tr::S> &out) {
	typedef typename Tr::S S;
	typename Tr::Inst inst;
	const arm_status st = Tr::init(&inst, static_cast<uint16_t>(c.size()), F, c.data(), state, static_cast<uint32_t>(maxChunk));
	if (st != ARM_MATH_SUCCESS) return false;
	bool ok = true;
	size_t pos = 0, ci = 0;
	while (pos < x.size()) {
		size_t b = chunks[ci++ % chunks.size()];
		if (b > x.size() - pos) b = x.size() - pos;
		dsp::GuardedBuffer<S> o(Tr::outCount(b, F));
		Tr::run(&inst, x.data() + pos, o.data(), static_cast<uint32_t>(b));
		if (!o.guardIntact()) ok = false;
		const std::vector<S> v = o.output();
		out.insert(out.end(), v.begin(), v.end());
		pos += b;
	}
	return ok;
}

// Processes x in consecutive calls of the sizes in `chunks` (cycled, last one truncated). The state buffer has exactly the DOC
// length for the largest chunk, which is also the blockSize given to init. Two passes: (1) a GuardedBuffer state (8 guard words
// behind, damage is reported in .guards), (2) a tight heap state (`new S[n]`, no slack, filled with a pattern first): ASan sees a
// write one element past the end or before the start, and the output must equal pass 1.
template <class Tr>
RunRes<Tr> doRun(const std::vector<typename Tr::S> &c, const std::vector<typename Tr::S> &x, unsigned F, const std::vector<size_t> &chunks) {
	typedef typename Tr::S S;
	RunRes<Tr> r;
	size_t maxChunk = 0;
	for (size_t k : chunks) maxChunk = std::max(maxChunk, k);
	const size_t n = Tr::stateLen(c.size(), F, maxChunk);
	dsp::GuardedBuffer<S> state(n);
	if (!runPass<Tr>(c, x, F, chunks, maxChunk, state.data(), r.out)) r.guards = false;
	if (!state.guardIntact()) r.guards = false;
	std::vector<S> out2;
	{
		std::unique_ptr<S[]> tight(new S[n]);
		std::memset(static_cast<void *>(tight.get()), 0x5A, n * sizeof(S));
		if (!runPass<Tr>(c, x, F, chunks, maxChunk, tight.get(), out2)) r.guards = false;
	}
	if (out2 != r.out) r.guards = false;
	return r;
}

template <class Tr>
std::vector<typename Tr::S> genVec(size_t n, uint32_t seed, int64_t amp, bool nonNeg = false) {
	typedef typename Tr::S S;
	Rng rng(seed);
	const int64_t lo = nonNeg ? 0 : std::max<int64_t>(-amp, Tr::minV), hi = std::min<int64_t>(amp, Tr::maxV);
	std::vector<S> v(n);
	for (size_t i = 0; i < n; i++) v[i] = static_cast<S>(randIn(rng, lo, hi));
	return v;
}

struct Cfg {
	unsigned F;
	size_t T;
	size_t n;  // input length used for the random tests: 13 outputs (decimate) / 13 inputs (interpolate): unroll by 4 plus tail
};

template <class Tr>
std::vector<Cfg> allCfgs() {
	std::vector<Cfg> v;
	const size_t ts[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 17, 25};
	for (unsigned F = 1; F <= 5; F++)
		for (size_t t : ts) {
			if (!Tr::isDecim && t > 12) continue;  // interpolate: phase lengths 1..9, 12
			v.push_back({F, Tr::taps(t, F), Tr::isDecim ? 13u * F : 13u});
		}
	return v;
}

// ---------------------------------------------------------------------------------------------------------------
// C1: ARM's pattern test
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
void armPatternTest(double snrThr, int64_t lsb) {
	typedef typename Tr::S S;
	const std::string id = Tr::patId();
	const std::vector<S> input = loadVec<Tr>("Input" + id);
	const std::vector<S> coefs = loadVec<Tr>("Coefs" + id);
	const std::vector<S> ref = loadVec<Tr>("Reference" + id);
	const std::vector<int32_t> cfg = dsp::loadPattern(patDir<Tr>() + "Configs" + id + "_u32.txt").q31();
	REQUIRE(cfg.size() % 4 == 0);
	std::vector<S> all;
	size_t ip = 0, cp = 0;
	bool guards = true, fields = true, sizes = true, ok = true;
	for (size_t i = 0; i + 3 < cfg.size(); i += 4) {
		const unsigned F = static_cast<unsigned>(cfg[i]);
		const size_t T = static_cast<size_t>(cfg[i + 1]), B = static_cast<size_t>(cfg[i + 2]), R = static_cast<size_t>(cfg[i + 3]);
		REQUIRE(ip + B <= input.size());
		REQUIRE(cp + T <= coefs.size());
		sizes = sizes && R == Tr::outCount(B, F);
		dsp::GuardedBuffer<S> state(Tr::stateLen(T, F, B));
		typename Tr::Inst inst;
		const arm_status st = Tr::init(&inst, static_cast<uint16_t>(T), F, coefs.data() + cp, state.data(), static_cast<uint32_t>(B));
		ok = ok && st == ARM_MATH_SUCCESS;  // ARM: ASSERT_TRUE(status == ARM_MATH_SUCCESS)
		fields = fields && Tr::fields(inst, static_cast<uint16_t>(T), F, coefs.data() + cp, state.data());
		dsp::GuardedBuffer<S> out(R);
		Tr::run(&inst, input.data() + ip, out.data(), static_cast<uint32_t>(B));
		guards = guards && out.guardIntact() && state.guardIntact();
		const std::vector<S> o = out.output();
		all.insert(all.end(), o.begin(), o.end());
		ip += B;
		cp += T;
	}
	CHECK(ok);
	CHECK(sizes);
	CHECK(fields);
	CHECK(guards);
	CHECK(ip == input.size());
	CHECK(cp == coefs.size());
	REQUIRE(all.size() == ref.size());
	CHECK(dsp::snr(ref, all) >= snrThr);
	auto c = dsp::withinLsb(ref, all, lsb);
	CHECK_MESSAGE(c, c.message);
}

// The library output on ARM's pattern inputs equals the exact integer DEF of this file bit for bit (checks the orientation and the
// phase convention of the DEF against ARM's data). Works for the fast variants as well (they have no ARM test).
template <class Tr>
void patternEqualsDefTest() {
	typedef typename Tr::S S;
	const std::string id = Tr::patId();
	const std::vector<S> input = loadVec<Tr>("Input" + id);
	const std::vector<S> coefs = loadVec<Tr>("Coefs" + id);
	const std::vector<int32_t> cfg = dsp::loadPattern(patDir<Tr>() + "Configs" + id + "_u32.txt").q31();
	size_t ip = 0, cp = 0, bad = 0;
	std::string first;
	for (size_t i = 0; i + 3 < cfg.size(); i += 4) {
		const unsigned F = static_cast<unsigned>(cfg[i]);
		const size_t T = static_cast<size_t>(cfg[i + 1]), B = static_cast<size_t>(cfg[i + 2]);
		const std::vector<S> c(coefs.begin() + static_cast<long>(cp), coefs.begin() + static_cast<long>(cp + T));
		const std::vector<S> x(input.begin() + static_cast<long>(ip), input.begin() + static_cast<long>(ip + B));
		const RunRes<Tr> r = doRun<Tr>(c, x, F, {B});
		const std::vector<S> d = defOut<Tr>(c, x, F);
		if (!(r.guards && dsp::exactEqual(d, r.out))) {
			if (bad == 0) first = "config " + std::to_string(i / 4) + ": " + dsp::exactEqual(d, r.out).message;
			bad++;
		}
		ip += B;
		cp += T;
	}
	CHECK_MESSAGE(bad == 0, bad << " configs differ from the DEF, first: " << first);
}

// The fast variant on ARM's pattern inputs relative to the standard function: q31 fast: 0 <= standard - fast < 2T (one-sided,
// header), q15 fast: identical to standard (the pattern data does not overflow the 32 bit accumulator, checked by equality).
template <class Fast, class Std>
void fastVsStdOnPattern() {
	typedef typename Std::S S;
	const std::string id = Std::patId();
	const std::vector<S> input = loadVec<Std>("Input" + id);
	const std::vector<S> coefs = loadVec<Std>("Coefs" + id);
	const std::vector<int32_t> cfg = dsp::loadPattern(patDir<Std>() + "Configs" + id + "_u32.txt").q31();
	size_t ip = 0, cp = 0, bad = 0;
	std::string first;
	for (size_t i = 0; i + 3 < cfg.size(); i += 4) {
		const unsigned F = static_cast<unsigned>(cfg[i]);
		const size_t T = static_cast<size_t>(cfg[i + 1]), B = static_cast<size_t>(cfg[i + 2]);
		const std::vector<S> c(coefs.begin() + static_cast<long>(cp), coefs.begin() + static_cast<long>(cp + T));
		const std::vector<S> x(input.begin() + static_cast<long>(ip), input.begin() + static_cast<long>(ip + B));
		const RunRes<Std> rs = doRun<Std>(c, x, F, {B});
		const RunRes<Fast> rf = doRun<Fast>(c, x, F, {B});
		const int64_t maxDiff = Fast::fb == 31 ? static_cast<int64_t>(2 * T - 1) : 0;
		REQUIRE(rs.out.size() == rf.out.size());
		for (size_t n = 0; n < rs.out.size(); n++) {
			const int64_t d = static_cast<int64_t>(rs.out[n]) - static_cast<int64_t>(rf.out[n]);
			if (d < 0 || d > maxDiff) {
				if (bad == 0) first = "config " + std::to_string(i / 4) + " sample " + std::to_string(n) + ": standard - fast = " + std::to_string(d);
				bad++;
			}
		}
		ip += B;
		cp += T;
	}
	CHECK_MESSAGE(bad == 0, bad << " samples outside 0 <= standard - fast <= " << (Fast::fb == 31 ? "2T-1" : "0") << ", first: " << first);
}

// ---------------------------------------------------------------------------------------------------------------
// C3 / C5 random data
// ---------------------------------------------------------------------------------------------------------------
// Bit exact against the integer DEF, every F and tap count of allCfgs() (output counts 13: unroll by 4 and tail 1)
template <class Tr>
void randomExactTest(bool nonNeg = false) {
	typedef typename Tr::S S;
	size_t bad = 0;
	std::string first;
	for (const Cfg &g : allCfgs<Tr>()) {
		const uint32_t seed = static_cast<uint32_t>(g.F * 1000 + g.T);
		const std::vector<S> c = genVec<Tr>(g.T, seed, Tr::maxV, nonNeg);
		const std::vector<S> x = genVec<Tr>(g.n, seed + 7, Tr::amp(Tr::winLen(g.T, g.F)), nonNeg);
		const RunRes<Tr> r = doRun<Tr>(c, x, g.F, {g.n});
		const std::vector<S> d = defOut<Tr>(c, x, g.F);
		auto chk = dsp::exactEqual(d, r.out);
		if (!(chk && r.guards)) {
			if (bad == 0) first = "F " + std::to_string(g.F) + " T " + std::to_string(g.T) + ": " + chk.message;
			bad++;
		}
	}
	CHECK_MESSAGE(bad == 0, bad << " configs differ from the DEF, first: " << first);
}

// One-sided floor bound against the double DEF (header): out <= D + eps and out > D - slack - eps (D clamped to the type range)
template <class Tr>
void randomDoubleTest(bool nonNeg = false) {
	typedef typename Tr::S S;
	size_t bad = 0;
	std::string first;
	for (const Cfg &g : allCfgs<Tr>()) {
		const uint32_t seed = static_cast<uint32_t>(g.F * 1000 + g.T + 500);
		const std::vector<S> c = genVec<Tr>(g.T, seed, Tr::maxV, nonNeg);
		const std::vector<S> x = genVec<Tr>(g.n, seed + 7, Tr::amp(Tr::winLen(g.T, g.F)), nonNeg);
		const RunRes<Tr> r = doRun<Tr>(c, x, g.F, {g.n});
		const std::vector<double> d = defDouble<Tr>(c, x, g.F);
		const double nTaps = static_cast<double>(Tr::winLen(g.T, g.F));
		const double eps = nTaps * std::ldexp(1.0, -21);
		REQUIRE(r.out.size() == d.size());
		for (size_t n = 0; n < d.size(); n++) {
			const double dc = std::min<double>(std::max<double>(d[n], static_cast<double>(Tr::minV)), static_cast<double>(Tr::maxV));
			const double o = static_cast<double>(r.out[n]);
			if (!(o <= dc + eps && o > dc - Tr::slack(Tr::winLen(g.T, g.F)) - eps)) {
				if (bad == 0)
					first = "F " + std::to_string(g.F) + " T " + std::to_string(g.T) + " n " + std::to_string(n) + ": out " + std::to_string(o) + " D " + std::to_string(dc);
				bad++;
			}
		}
	}
	CHECK_MESSAGE(bad == 0, bad << " outputs outside (D - slack - eps, D + eps], first: " << first);
}

// ---------------------------------------------------------------------------------------------------------------
// C3: impulse and step response with hand derived values
// ---------------------------------------------------------------------------------------------------------------
// unit: coefficient granularity so that c is a multiple of 4: then half*c is exact in every variant (q31: floor(2^30*c/2^32)*2 =
// c/2, q15: 2^14*c >> 15 = c/2).
template <class Tr>
int64_t unitOf() {
	return Tr::fb == 31 ? (1 << 20) : (1 << 8);
}
template <class Tr>
typename Tr::S halfScale() {
	return static_cast<typename Tr::S>(Tr::fb == 31 ? 0x40000000 : 0x4000);  // 0.5
}
template <class Tr>
std::vector<typename Tr::S> alternatingCoefs(size_t T, int mode) {
	std::vector<typename Tr::S> c(T);
	for (size_t i = 0; i < T; i++) {
		const bool neg = mode == 0 ? (i % 2 == 1) : (i % 3 == 1);
		c[i] = static_cast<typename Tr::S>((neg ? -1 : 1) * static_cast<int64_t>(i + 1) * unitOf<Tr>());
	}
	return c;
}

// x = {0.5, 0, 0, ...}; b[i] = c[T-1-i]. Decimate: y[j] = b[jM] * 0.5 = c[T-1-jM] / 2 for jM < T, else 0 (y[j] = sum_i b[i] x[jM-i],
// only i = jM hits the impulse). Interpolate: y[m] = b[m] * 0.5 for m < T else 0 (m = kL+r, y[nL+r] = sum_k b[kL+r] x[n-k], n = k).
template <class Tr>
void impulseTest() {
	typedef typename Tr::S S;
	for (unsigned F = 1; F <= 4; F++)
		for (size_t t = 1; t <= 12; t++) {
			const size_t T = Tr::taps(t, F);
			const std::vector<S> c = alternatingCoefs<Tr>(T, 0);
			const size_t nOut = Tr::isDecim ? T + 3 : (t + 3) * F;       // outputs
			const size_t nIn = Tr::isDecim ? nOut * F : t + 3;           // inputs
			std::vector<S> x(nIn, 0);
			x[0] = halfScale<Tr>();
			const RunRes<Tr> r = doRun<Tr>(c, x, F, {nIn});
			REQUIRE(r.out.size() == nOut);
			std::vector<S> expect(nOut, 0);
			for (size_t j = 0; j < nOut; j++) {
				const size_t m = Tr::isDecim ? j * F : j;  // index into b
				if (m < T) expect[j] = static_cast<S>(static_cast<int64_t>(c[T - 1 - m]) / 2);
			}
			auto chk = dsp::exactEqual(expect, r.out);
			CHECK_MESSAGE(chk, "F " << F << " taps " << T << ": " << chk.message);
			CHECK(r.guards);
		}
}

// x = 0.5 constant. Decimate: y[j] = 0.5 * sum_{i=0}^{min(jM,T-1)} b[i]. Interpolate: y[nL+r] = 0.5 * sum_{k=0}^{min(n,P-1)} b[kL+r].
template <class Tr>
void stepTest() {
	typedef typename Tr::S S;
	for (unsigned F = 1; F <= 4; F++)
		for (size_t t = 1; t <= 12; t++) {
			const size_t T = Tr::taps(t, F);
			const std::vector<S> c = alternatingCoefs<Tr>(T, 1);
			const size_t nOut = Tr::isDecim ? T + 3 : (t + 3) * F;
			const size_t nIn = Tr::isDecim ? nOut * F : t + 3;
			const std::vector<S> x(nIn, halfScale<Tr>());
			const RunRes<Tr> r = doRun<Tr>(c, x, F, {nIn});
			REQUIRE(r.out.size() == nOut);
			std::vector<S> expect(nOut, 0);
			for (size_t j = 0; j < nOut; j++) {
				int64_t sum = 0;
				if (Tr::isDecim) {
					const size_t hiI = std::min(j * F, T - 1);
					for (size_t i = 0; i <= hiI; i++) sum += c[T - 1 - i];
				} else {
					const size_t n = j / F, rr = j % F;
					const size_t hiK = std::min(n, t - 1);
					for (size_t k = 0; k <= hiK; k++) sum += c[T - 1 - (k * F + rr)];
				}
				expect[j] = static_cast<S>(sum / 2);
			}
			auto chk = dsp::exactEqual(expect, r.out);
			CHECK_MESSAGE(chk, "F " << F << " taps " << T << ": " << chk.message);
		}
}

// ---------------------------------------------------------------------------------------------------------------
// C2 / C4
// ---------------------------------------------------------------------------------------------------------------
// A call of k outputs (k = 1..9: unroll by 4 and every tail) writes exactly the DOC output count: guard behind the output and
// behind the exact-size state intact, every output equal to the DEF (an unwritten sample would still hold the poison)
template <class Tr>
void writesExactlyTest() {
	typedef typename Tr::S S;
	const unsigned Fs[] = {1, 2, 3, 5};
	const size_t ts[] = {1, 3, 4, 5, 9};
	for (unsigned F : Fs)
		for (size_t t : ts)
			for (size_t k = 1; k <= 9; k++) {
				const size_t T = Tr::taps(t, F);
				const std::vector<S> c = genVec<Tr>(T, static_cast<uint32_t>(F * 100 + t), Tr::maxV);
				const size_t B = Tr::isDecim ? k * F : k;
				const std::vector<S> x = genVec<Tr>(B, static_cast<uint32_t>(k + 31 * t), Tr::amp(Tr::winLen(T, F)));
				const RunRes<Tr> r = doRun<Tr>(c, x, F, {B});
				auto chk = dsp::exactEqual(defOut<Tr>(c, x, F), r.out);
				CHECK_MESSAGE(chk, "F " << F << " taps " << T << " k " << k << ": " << chk.message);
				CHECK_MESSAGE(r.guards, "F " << F << " taps " << T << " k " << k << ": guard damaged");
			}
}

// consecutive blocks give the output of one long block (chunk sizes in blocks of outputs / inputs: 1, 2, 3, 4, 5, 7 and a mix)
template <class Tr>
void continuityTest(bool nonNeg = false) {
	typedef typename Tr::S S;
	const std::vector<std::vector<size_t>> lists = {{1}, {2}, {3}, {4}, {5}, {7}, {1, 3, 2, 7, 5, 4}};
	const unsigned Fs[] = {1, 2, 3, 5};
	const size_t ts[] = {1, 4, 5, 9};
	for (unsigned F : Fs)
		for (size_t t : ts) {
			const size_t T = Tr::taps(t, F);
			const std::vector<S> c = genVec<Tr>(T, static_cast<uint32_t>(F * 77 + t), Tr::maxV, nonNeg);
			const size_t N = Tr::chunk(37, F);
			const std::vector<S> x = genVec<Tr>(N, static_cast<uint32_t>(F + t * 13), Tr::amp(Tr::winLen(T, F)), nonNeg);
			const RunRes<Tr> whole = doRun<Tr>(c, x, F, {N});
			auto d = dsp::exactEqual(defOut<Tr>(c, x, F), whole.out);
			CHECK_MESSAGE(d, "F " << F << " taps " << T << " long block: " << d.message);
			for (const std::vector<size_t> &l : lists) {
				std::vector<size_t> ch;
				for (size_t k : l) ch.push_back(Tr::chunk(k, F));
				const RunRes<Tr> parts = doRun<Tr>(c, x, F, ch);
				auto chk = dsp::exactEqual(whole.out, parts.out);
				CHECK_MESSAGE(chk, "F " << F << " taps " << T << " blocks starting " << l[0] << ": " << chk.message);
				CHECK(parts.guards);
			}
		}
}

// two instances with different coefficients, factors and states, called alternately, do not influence each other
template <class Tr>
void independentInstancesTest() {
	typedef typename Tr::S S;
	const unsigned F1 = 2, F2 = 3;
	const size_t T1 = Tr::taps(5, F1), T2 = Tr::taps(4, F2);
	const std::vector<S> c1 = genVec<Tr>(T1, 11, Tr::maxV), c2 = genVec<Tr>(T2, 12, Tr::maxV);
	const size_t b1 = Tr::chunk(4, F1), b2 = Tr::chunk(4, F2);
	const size_t rounds = 6;
	const std::vector<S> x1 = genVec<Tr>(b1 * rounds, 13, Tr::amp(Tr::winLen(T1, F1)));
	const std::vector<S> x2 = genVec<Tr>(b2 * rounds, 14, Tr::amp(Tr::winLen(T2, F2)));
	dsp::GuardedBuffer<S> s1(Tr::stateLen(T1, F1, b1)), s2(Tr::stateLen(T2, F2, b2));
	typename Tr::Inst i1, i2;
	Tr::init(&i1, static_cast<uint16_t>(T1), F1, c1.data(), s1.data(), static_cast<uint32_t>(b1));
	Tr::init(&i2, static_cast<uint16_t>(T2), F2, c2.data(), s2.data(), static_cast<uint32_t>(b2));
	std::vector<S> y1, y2;
	for (size_t r = 0; r < rounds; r++) {
		dsp::GuardedBuffer<S> o1(Tr::outCount(b1, F1)), o2(Tr::outCount(b2, F2));
		Tr::run(&i1, x1.data() + r * b1, o1.data(), static_cast<uint32_t>(b1));
		Tr::run(&i2, x2.data() + r * b2, o2.data(), static_cast<uint32_t>(b2));
		CHECK(o1.guardIntact());
		CHECK(o2.guardIntact());
		const std::vector<S> a = o1.output(), b = o2.output();
		y1.insert(y1.end(), a.begin(), a.end());
		y2.insert(y2.end(), b.begin(), b.end());
	}
	auto k1 = dsp::exactEqual(defOut<Tr>(c1, x1, F1), y1);
	auto k2 = dsp::exactEqual(defOut<Tr>(c2, x2, F2), y2);
	CHECK_MESSAGE(k1, k1.message);
	CHECK_MESSAGE(k2, k2.message);
	CHECK(s1.guardIntact());
	CHECK(s2.guardIntact());
}

// init again clears the history left by a previous run (the state buffer is first filled with a pattern): the run repeats
template <class Tr>
void reinitTest() {
	typedef typename Tr::S S;
	const unsigned F = 2;
	const size_t T = Tr::taps(6, F);
	const std::vector<S> c = genVec<Tr>(T, 21, Tr::maxV);
	const size_t B = Tr::chunk(5, F);
	const std::vector<S> x = genVec<Tr>(B, 22, Tr::amp(Tr::winLen(T, F)));
	dsp::GuardedBuffer<S> state(Tr::stateLen(T, F, B));
	typename Tr::Inst inst;
	Tr::init(&inst, static_cast<uint16_t>(T), F, c.data(), state.data(), static_cast<uint32_t>(B));
	dsp::GuardedBuffer<S> o1(Tr::outCount(B, F)), o2(Tr::outCount(B, F)), o3(Tr::outCount(B, F));
	Tr::run(&inst, x.data(), o1.data(), static_cast<uint32_t>(B));
	Tr::run(&inst, x.data(), o2.data(), static_cast<uint32_t>(B));  // second call carries the history of the first
	std::memset(static_cast<void *>(state.data()), 0x5A, state.size() * sizeof(S));
	Tr::init(&inst, static_cast<uint16_t>(T), F, c.data(), state.data(), static_cast<uint32_t>(B));
	Tr::run(&inst, x.data(), o3.data(), static_cast<uint32_t>(B));
	CHECK(o1.output() != o2.output());  // sanity: the history changes the output (random data, 6 phases of taps)
	auto chk = dsp::exactEqual(o1.output(), o3.output());
	CHECK_MESSAGE(chk, chk.message);
	CHECK(state.guardIntact());
}

// init stores the fields and zeroes exactly the DOC state length (poisoned by the GuardedBuffer: a missing clear or a clear too
// short stays visible, a clear too long damages the guard)
template <class Tr>
void initTest() {
	typedef typename Tr::S S;
	const unsigned Fs[] = {1, 2, 3, 4};
	const size_t ts[] = {1, 2, 3, 4, 5, 9, 12};
	const size_t ks[] = {1, 2, 5};
	for (unsigned F : Fs)
		for (size_t t : ts)
			for (size_t k : ks) {
				const size_t T = Tr::taps(t, F);
				const size_t B = Tr::chunk(k, F);
				const std::vector<S> c(T, static_cast<S>(1));
				dsp::GuardedBuffer<S> st(Tr::stateLen(T, F, B));
				typename Tr::Inst inst;
				std::memset(static_cast<void *>(&inst), 0xEE, sizeof inst);
				const arm_status s = Tr::init(&inst, static_cast<uint16_t>(T), F, c.data(), st.data(), static_cast<uint32_t>(B));
				CHECK(s == ARM_MATH_SUCCESS);
				CHECK_MESSAGE(Tr::fields(inst, static_cast<uint16_t>(T), F, c.data(), st.data()), "F " << F << " taps " << T);
				const std::vector<S> o = st.output();
				bool zero = true;
				for (S v : o) zero = zero && v == 0;
				CHECK_MESSAGE(zero, "state not cleared: F " << F << " taps " << T << " block " << B);
				CHECK_MESSAGE(st.guardIntact(), "state guard damaged: F " << F << " taps " << T << " block " << B);
			}
}

template <class Tr>
void constInputsTest() {
	typedef typename Tr::S S;
	const unsigned F = 3;
	const size_t T = Tr::taps(7, F);
	const std::vector<S> c = genVec<Tr>(T, 41, Tr::maxV);
	const std::vector<S> x = genVec<Tr>(Tr::chunk(9, F), 42, Tr::amp(Tr::winLen(T, F)));
	const std::vector<S> c0 = c, x0 = x;
	doRun<Tr>(c, x, F, {x.size()});
	CHECK(c == c0);
	CHECK(x == x0);
}

// ---------------------------------------------------------------------------------------------------------------
// C4: init argument errors
// ---------------------------------------------------------------------------------------------------------------
// DOC (arm_fir_decimate_init_q31.c:53): "ARM_MATH_LENGTH_ERROR : blockSize is not a multiple of M"; nothing about numTaps
template <class Tr>
void decimInitErrorsTest() {
	typedef typename Tr::S S;
	struct Case {
		uint16_t T;
		unsigned M;
		uint32_t B;
		bool ok;
	};
	const Case cases[] = {
	    {5, 3, 10, false}, {5, 3, 11, false}, {5, 3, 3, true},  {5, 3, 9, true},  {5, 3, 12, true}, {1, 1, 7, true},
	    {5, 2, 3, false},  {5, 2, 2, true},   {2, 4, 4, true},  {1, 4, 8, true},  {3, 255, 255, true}, {3, 255, 256, false},
	    {7, 2, 1, false},
	};
	for (const Case &k : cases) {
		std::vector<S> c(k.T, static_cast<S>(1));
		std::vector<S> st(static_cast<size_t>(k.T) + k.B, static_cast<S>(7));
		typename Tr::Inst inst;
		const arm_status s = Tr::init(&inst, k.T, k.M, c.data(), st.data(), k.B);
		INFO("numTaps " << k.T << " M " << k.M << " blockSize " << k.B);
		if (k.ok)
			CHECK(s == ARM_MATH_SUCCESS);
		else
			CHECK(s == ARM_MATH_LENGTH_ERROR);
	}
}

// DOC is silent about the state of the instance and of the buffer after an error: characterization (well defined: the init has an
// if / else, nothing is written in the error branch)
template <class Tr>
void decimInitErrorLeavesEverythingTest() {
	typedef typename Tr::S S;
	std::vector<S> c(5, static_cast<S>(1));
	std::vector<S> st(5 + 10, static_cast<S>(7));
	const std::vector<S> st0 = st;
	typename Tr::Inst inst;
	std::memset(static_cast<void *>(&inst), 0xEE, sizeof inst);
	unsigned char before[sizeof inst];
	std::memcpy(before, &inst, sizeof inst);
	const arm_status s = Tr::init(&inst, 5, 3, c.data(), st.data(), 10);  // 10 % 3 != 0
	CHECK(s != ARM_MATH_SUCCESS);
	CHECK(std::memcmp(before, &inst, sizeof inst) == 0);
	CHECK(st == st0);
}

// DOC (arm_fir_interpolate_init_q31.c:53): "ARM_MATH_ARGUMENT_ERROR : filter length numTaps is not a multiple of the interpolation
// factor L" (@return list). The code returns ARM_MATH_LENGTH_ERROR (:80-81). Lead decision: DOC defect, characterization asserting
// ARM_MATH_LENGTH_ERROR, NEEDS REVIEW.
template <class Tr>
void interpInitErrorsTest(bool exactCode) {
	typedef typename Tr::S S;
	struct Case {
		uint16_t T;
		unsigned L;
		bool ok;
	};
	const Case cases[] = {{5, 3, false}, {7, 2, false}, {1, 2, false}, {3, 3, true}, {6, 3, true}, {9, 1, true},
	                      {1, 1, true},  {8, 4, true},  {9, 4, false}, {255, 255, true}, {254, 255, false}};
	for (const Case &k : cases) {
		std::vector<S> c(k.T, static_cast<S>(1));
		std::vector<S> st(static_cast<size_t>(k.T) + 8, static_cast<S>(7));
		typename Tr::Inst inst;
		const arm_status s = Tr::init(&inst, k.T, k.L, c.data(), st.data(), 4);
		INFO("numTaps " << k.T << " L " << k.L);
		if (k.ok)
			CHECK(s == ARM_MATH_SUCCESS);
		else
		{
			if (exactCode)
				CHECK(s == ARM_MATH_LENGTH_ERROR);  // what the code returns; the DOC @return says ARM_MATH_ARGUMENT_ERROR (NEEDS REVIEW)
			else
				CHECK(s != ARM_MATH_SUCCESS);  // DOC conforming: any error code
		}
	}
}

template <class Tr>
void interpInitErrorLeavesEverythingTest() {
	typedef typename Tr::S S;
	std::vector<S> c(5, static_cast<S>(1));
	std::vector<S> st(5 + 4, static_cast<S>(7));
	const std::vector<S> st0 = st;
	typename Tr::Inst inst;
	std::memset(static_cast<void *>(&inst), 0xEE, sizeof inst);
	unsigned char before[sizeof inst];
	std::memcpy(before, &inst, sizeof inst);
	const arm_status s = Tr::init(&inst, 5, 3, c.data(), st.data(), 4);  // 5 % 3 != 0
	CHECK(s != ARM_MATH_SUCCESS);
	CHECK(std::memcmp(before, &inst, sizeof inst) == 0);
	CHECK(st == st0);
}

// ---------------------------------------------------------------------------------------------------------------
// C4: saturation and boundary values (F = 1: a plain filter; the output at index T-1 has the complete window; x all the same)
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
typename Tr::S lastOut(const std::vector<typename Tr::S> &c, typename Tr::S xv) {
	const std::vector<typename Tr::S> x(c.size(), xv);
	const RunRes<Tr> r = doRun<Tr>(c, x, 1, {x.size()});
	return r.out.back();
}

// q15 hand derived (the 2.30 products are exact; result = clamp(floor(S / 2^15))):
//  A: T=1, x = c = -32768: S = 2^30, V = 32768 -> 32767
//  B: T=2, x = -32768, c = 32767: S = -2 * 32768 * 32767 = -2147418112, V = -65534 -> -32768
//  C: T=2, x = 30000, c = 32767: S = 2 * 983010000 = 1966020000 (< 2^31: no 32 bit wrap), V = 59999 -> 32767
//  D (standard only, 64 bit accumulator): T=8, x = c = -32768: S = 2^33 (does not fit 32 bit), V = 2^18 -> 32767
//  E (standard only): T=8, x = -32768, c = 32767: S = -8*32768*32767, V = -262136 -> -32768
template <class Tr>
void q15SaturationTest(bool extended) {
	typedef typename Tr::S S;
	CHECK(lastOut<Tr>({static_cast<S>(-32768)}, static_cast<S>(-32768)) == 32767);
	CHECK(lastOut<Tr>({32767, 32767}, static_cast<S>(-32768)) == -32768);
	CHECK(lastOut<Tr>({32767, 32767}, static_cast<S>(30000)) == 32767);
	if (extended) {
		CHECK(lastOut<Tr>(std::vector<S>(8, static_cast<S>(-32768)), static_cast<S>(-32768)) == 32767);
		CHECK(lastOut<Tr>(std::vector<S>(8, static_cast<S>(32767)), static_cast<S>(-32768)) == -32768);
	}
}

// q15 values that must NOT saturate: the extreme results of one product:
//  x = -32768, c = 32767: S = -1073709056, V = -32767 (exactly: S/32768 = -32767);  x = c = 32767: S = 1073676289, V = 32766
template <class Tr>
void q15NoSaturationTest() {
	typedef typename Tr::S S;
	CHECK(lastOut<Tr>({32767}, static_cast<S>(-32768)) == -32767);
	CHECK(lastOut<Tr>({32767}, static_cast<S>(32767)) == 32766);
	CHECK(lastOut<Tr>({static_cast<S>(-32768)}, static_cast<S>(32767)) == -32767);
	CHECK(lastOut<Tr>({0}, static_cast<S>(-32768)) == 0);
}

// q31 products that stay in range, standard: x = INT32_MAX, c = INT32_MAX: S = (2^31-1)^2 = 2^62 - 2^32 + 1, V = floor(S/2^31) = 2^31 - 2;
// x = INT32_MIN, c = INT32_MAX: S = -2^31 (2^31-1), V = -(2^31-1) = INT32_MIN + 1.
// fast: floor(S / 2^32) * 2: (2^62 - 2^32 + 1) / 2^32 = 2^30 - 1 + 2^-32 -> 2^30 - 1 -> * 2 = 2^31 - 2;  -2^31 (2^31-1) / 2^32 = -(2^30) + 0.5
// -> floor = -2^30 -> * 2 = INT32_MIN (one LSB lower than the standard result: the truncation of the product, -2T < error)
template <class Tr>
void q31BoundaryTest() {
	typedef typename Tr::S S;
	CHECK(lastOut<Tr>({INT32_MAX}, static_cast<S>(INT32_MAX)) == INT32_MAX - 1);
	if (Tr::isFast)
		CHECK(lastOut<Tr>({INT32_MAX}, static_cast<S>(INT32_MIN)) == INT32_MIN);
	else
		CHECK(lastOut<Tr>({INT32_MAX}, static_cast<S>(INT32_MIN)) == INT32_MIN + 1);
	CHECK(lastOut<Tr>({0}, static_cast<S>(INT32_MIN)) == 0);
}

// the part of the boundary test without a negative accumulator (no left shift of a negative value in the fast kernel)
template <class Tr>
void q31BoundaryPositiveTest() {
	typedef typename Tr::S S;
	CHECK(lastOut<Tr>({INT32_MAX}, static_cast<S>(INT32_MAX)) == INT32_MAX - 1);
	CHECK(lastOut<Tr>({0}, static_cast<S>(INT32_MIN)) == 0);
	CHECK(lastOut<Tr>({INT32_MAX}, static_cast<S>(0)) == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// C3: sweeps over the input range
// ---------------------------------------------------------------------------------------------------------------
// One tap, F = 1, every q15 input value (-32768..32767 ascending, one block of 65536) with a set of coefficients, against the
// direct formula clamp(x*c >> 15) (hand formula, independent of reduce()).
template <class Tr>
void sweepOneTapQ15() {
	typedef typename Tr::S S;
	const int16_t cs[] = {0, 1, -1, 32767, -32768, 16384, 12345, -23456, -1000};
	std::vector<S> x(65536);
	for (size_t n = 0; n < x.size(); n++) x[n] = static_cast<S>(static_cast<int32_t>(n) - 32768);
	for (int16_t cv : cs) {
		const std::vector<S> c(1, static_cast<S>(cv));
		const RunRes<Tr> r = doRun<Tr>(c, x, 1, {x.size()});
		REQUIRE(r.out.size() == x.size());
		size_t bad = 0;
		for (size_t n = 0; n < x.size(); n++) {
			const int64_t e = satV<Tr>(Tr::direct1(x[n], static_cast<S>(cv)));
			if (r.out[n] != e) bad++;
		}
		CHECK_MESSAGE(bad == 0, bad << " of 65536 outputs wrong for c = " << cv);
		CHECK(r.guards);
	}
}

// Interpolate with L = 2 and one tap per phase (T = 2, P = 1): y[2n] = c[1]*x[n], y[2n+1] = c[0]*x[n] (derived from the header formula
// with P = 1), every q15 input value.
template <class Tr>
void sweepInterpL2Q15() {
	typedef typename Tr::S S;
	std::vector<S> x(65536);
	for (size_t n = 0; n < x.size(); n++) x[n] = static_cast<S>(static_cast<int32_t>(n) - 32768);
	const std::vector<S> c = {static_cast<S>(-12345), static_cast<S>(31000)};
	const RunRes<Tr> r = doRun<Tr>(c, x, 2, {x.size()});
	REQUIRE(r.out.size() == 2 * x.size());
	size_t bad = 0;
	for (size_t n = 0; n < x.size(); n++) {
		if (r.out[2 * n] != satV<Tr>(Tr::direct1(x[n], c[1]))) bad++;
		if (r.out[2 * n + 1] != satV<Tr>(Tr::direct1(x[n], c[0]))) bad++;
	}
	CHECK_MESSAGE(bad == 0, bad << " outputs wrong");
	CHECK(r.guards);
}

// Full-range permutation sweep: every q15 value appears once as the newest sample (index n*7919 mod 65536 is a bijection, 7919 is
// odd) for T = 1..maxT taps, F = 1, against the integer DEF
template <class Tr>
void sweepPermutedQ15(size_t minT, size_t maxT) {
	typedef typename Tr::S S;
	std::vector<S> x(65536);
	for (size_t n = 0; n < x.size(); n++) x[n] = static_cast<S>(static_cast<int32_t>((n * 7919u) % 65536u) - 32768);
	for (size_t T = minT; T <= maxT; T++) {
		const std::vector<S> c = genVec<Tr>(T, static_cast<uint32_t>(900 + T), Tr::maxV);
		const RunRes<Tr> r = doRun<Tr>(c, x, 1, {x.size()});
		auto chk = dsp::exactEqual(defOut<Tr>(c, x, 1), r.out);
		CHECK_MESSAGE(chk, "taps " << T << ": " << chk.message);
		CHECK(r.guards);
	}
}

// q31: strided sweep x_n = INT32_MIN + 1 + n * 65521 (n < 65534, about 2^32 span, 65521 odd), one tap, coefficients from a set that
// includes the extremes except where the product would reach 2^62 with x = INT32_MIN (x never is INT32_MIN here), against the direct
// formula
template <class Tr>
void sweepOneTapQ31(bool nonNegOnly = false) {
	typedef typename Tr::S S;
	const int32_t cs[] = {0, 1, -1, INT32_MAX, INT32_MIN, 0x40000000, 0x12345678, -0x0BCDEF01, -1000};
	std::vector<S> x(65534);
	for (size_t n = 0; n < x.size(); n++) x[n] = static_cast<S>(static_cast<int64_t>(INT32_MIN) + 1 + static_cast<int64_t>(n) * 65521);
	REQUIRE(x.back() > 0);
	if (nonNegOnly) x.erase(std::remove_if(x.begin(), x.end(), [](S v) { return v < 0; }), x.end());
	for (int32_t cv : cs) {
		if (nonNegOnly && cv < 0) continue;
		const std::vector<S> c(1, static_cast<S>(cv));
		const RunRes<Tr> r = doRun<Tr>(c, x, 1, {x.size()});
		REQUIRE(r.out.size() == x.size());
		size_t bad = 0;
		for (size_t n = 0; n < x.size(); n++) {
			const int64_t e = satV<Tr>(Tr::direct1(x[n], static_cast<S>(cv)));
			if (r.out[n] != e) bad++;
		}
		CHECK_MESSAGE(bad == 0, bad << " of " << x.size() << " outputs wrong for c = " << cv);
	}
}

// ---------------------------------------------------------------------------------------------------------------
// characterization: the narrowing of out-of-range results in the 4-way unrolled code (9 outputs per call: two groups of 4 and a tail of
// 1, M / L = 1, 2, 3, 5). NEEDS REVIEW: DOC says "saturated", the library wraps (see the header).
// ---------------------------------------------------------------------------------------------------------------
// Taps and input all INT32_MIN, one tap per phase: every output is the product +1.0 = 2^62 / 2^31 = 2^31 (fast: acc 2^30, << 1), which
// leaves the narrowing as INT32_MIN at every output position 0..8.
template <class Tr>
void wrapPositiveUnrolledTest() {
	typedef typename Tr::S S;
	const unsigned Fs[] = {1, 2, 3, 5};
	for (unsigned F : Fs) {
		const size_t T = Tr::taps(1, F);
		const std::vector<S> c(T, INT32_MIN);
		const std::vector<S> x(Tr::isDecim ? 9 * F : 9, INT32_MIN);
		const RunRes<Tr> r = doRun<Tr>(c, x, F, {x.size()});
		REQUIRE(r.out.size() == 9 * (Tr::isDecim ? 1 : F));
		bool all = true;
		for (S v : r.out) all = all && v == INT32_MIN;
		CHECK_MESSAGE(all, "F " << F);
		CHECK(r.guards);
	}
}

// Two taps (decimate) / two taps per phase (interpolate) of INT32_MAX, input all INT32_MIN (standard function only: the fast kernel
// would shift a negative accumulator, B-DDECQ-4). A window with both samples: 2 * (-2^31)(2^31-1) / 2^31 = -(2^32-2), +2 after the
// wrapping narrowing. The first window holds one sample only (the other is the zero history): -(2^31)(2^31-1)/2^31 = -(2^31-1) =
// INT32_MIN + 1 (in range). Decimate: output 0 has one sample, outputs 1..8 two (output j uses x[jM-1] and x[jM]). Interpolate: outputs
// 0..L-1 (n = 0) have one sample, the others two.
template <class Tr>
void wrapNegativeUnrolledTest() {
	typedef typename Tr::S S;
	const unsigned Fs[] = {1, 2, 3, 5};
	for (unsigned F : Fs) {
		const size_t T = Tr::taps(2, F);
		const std::vector<S> c(T, INT32_MAX);
		const std::vector<S> x(Tr::isDecim ? 9 * F : 9, INT32_MIN);
		const RunRes<Tr> r = doRun<Tr>(c, x, F, {x.size()});
		const size_t nOut = 9 * (Tr::isDecim ? 1 : F);
		REQUIRE(r.out.size() == nOut);
		const size_t firstWindow = Tr::isDecim ? 1 : F;
		bool all = true;
		for (size_t o = 0; o < nOut; o++) all = all && r.out[o] == (o < firstWindow ? INT32_MIN + 1 : 2);
		CHECK_MESSAGE(all, "F " << F);
		CHECK(r.guards);
	}
}

}  // namespace

// ===============================================================================================================
// arm_fir_decimate_q31
// ===============================================================================================================
TEST_SUITE("DSP Filtering decimation q31") {

TEST_CASE("[C1 arm_fir_decimate_q31] ARM's pattern test (72 configs, one block each) reaches SNR 100 and 2 LSB") {
	armPatternTest<DecQ31>(100.0, 2);  // DECIMQ31.cpp:5, :13
}
TEST_CASE("[C3 arm_fir_decimate_q31] the output on ARM's pattern inputs equals the integer DEF bit for bit") {
	patternEqualsDefTest<DecQ31>();
}
TEST_CASE("[C3 arm_fir_decimate_q31] random data, M 1..5 and 1..25 taps equals the integer DEF bit for bit") { randomExactTest<DecQ31>(); }
TEST_CASE("[C5 arm_fir_decimate_q31] random data is within one LSB below and nothing above the double DEF (floor)") { randomDoubleTest<DecQ31>(); }
TEST_CASE("[C3 arm_fir_decimate_q31] the impulse response is the coefficients in reverse order times 0.5, every M-th one") { impulseTest<DecQ31>(); }
TEST_CASE("[C3 arm_fir_decimate_q31] the step response is the running sum of the reversed coefficients times 0.5") { stepTest<DecQ31>(); }
TEST_CASE("[C2 arm_fir_decimate_q31] a call of 1..9 outputs writes blockSize/M samples and nothing behind them or behind the exact size state") {
	writesExactlyTest<DecQ31>();
}
TEST_CASE("[C4 arm_fir_decimate_q31] consecutive blocks of 1, 2, 3, 4, 5, 7 and mixed outputs carry the state like one long block") {
	continuityTest<DecQ31>();
}
TEST_CASE("[C4 arm_fir_decimate_q31] two instances do not influence each other") { independentInstancesTest<DecQ31>(); }
TEST_CASE("[C4 arm_fir_decimate_q31] init again clears the history and a garbage state") { reinitTest<DecQ31>(); }
TEST_CASE("[C4 arm_fir_decimate_init_q31] init stores numTaps, M, pCoeffs, pState and zeroes exactly numTaps + blockSize - 1 elements") {
	initTest<DecQ31>();
}
TEST_CASE("[C4 arm_fir_decimate_init_q31] a block size that is not a multiple of M returns ARM_MATH_LENGTH_ERROR, numTaps is not checked") {
	decimInitErrorsTest<DecQ31>();
}
TEST_CASE("characterization: [C4 arm_fir_decimate_init_q31] the error return leaves the instance and the state buffer untouched") {
	decimInitErrorLeavesEverythingTest<DecQ31>();
}
TEST_CASE("[C4 arm_fir_decimate_q31] the coefficient and input arrays are not modified") { constInputsTest<DecQ31>(); }
TEST_CASE("[C4 arm_fir_decimate_q31] results next to full scale stay in range: (2^31-1)^2 gives 2^31-2 and INT32_MIN*(2^31-1) gives INT32_MIN+1") {
	q31BoundaryTest<DecQ31>();
}
TEST_CASE("[C3 arm_fir_decimate_q31] a one tap filter over a strided sweep of the whole q31 range equals clamp(x*c >> 31)") { sweepOneTapQ31<DecQ31>(); }
// NEEDS REVIEW (characterization): DOC says saturated, library wraps the final narrowing; suspected DOC/vendor defect, not decided.
// DOC of the narrowing step: arm_fir_decimate_q31.c:55 "After all multiply-accumulates are performed, the 2.62 accumulator is truncated to 1.32 format and then saturated to 1.31 format."
// (The DOC sentence "wraps around rather than clip" is about the int64 accumulator, which does not overflow here.)
TEST_CASE("characterization: [C4 arm_fir_decimate_q31] +1.0 (INT32_MIN * INT32_MIN) leaves the narrowing as 0x80000000, the DOC says saturated (NEEDS REVIEW)") {
	const std::vector<int32_t> c = {INT32_MIN};
	const int32_t r = lastOut<DecQ31>(c, INT32_MIN);
	CHECK(r == INT32_MIN);  // V = 2^62 / 2^31 = 2^31 (fast: acc 2^30 in 2.30, acc << 1 = 2^31), (q31_t) wraps to INT32_MIN
}
// NEEDS REVIEW (characterization): DOC says saturated, library wraps the final narrowing; suspected DOC/vendor defect, not decided.
// DOC of the narrowing step: arm_fir_decimate_q31.c:55 "After all multiply-accumulates are performed, the 2.62 accumulator is truncated to 1.32 format and then saturated to 1.31 format."
// (The DOC sentence "wraps around rather than clip" is about the int64 accumulator, which does not overflow here.)
TEST_CASE("characterization: [C4 arm_fir_decimate_q31] -(2^32-2) LSB leaves the narrowing as +2, the DOC says saturated (NEEDS REVIEW)") {
	const std::vector<int32_t> c = {INT32_MAX, INT32_MAX};
	const int32_t r = lastOut<DecQ31>(c, INT32_MIN);
	CHECK(r == 2);  // V = 2*(-2^31)(2^31-1)/2^31 = -(2^32-2) = +2 mod 2^32
}
TEST_CASE("characterization: [C4 arm_fir_decimate_q31] +1.0 at every output of a 9 output call (unrolled by 4 plus tail), M or L 1, 2, 3, 5, leaves the narrowing as 0x80000000, the DOC says saturated (NEEDS REVIEW)") { wrapPositiveUnrolledTest<DecQ31>(); }
TEST_CASE("characterization: [C4 arm_fir_decimate_q31] -(2^32-2) LSB at every full window of a 9 output call, M or L 1, 2, 3, 5, leaves the narrowing as +2, the DOC says saturated (NEEDS REVIEW)") { wrapNegativeUnrolledTest<DecQ31>(); }

}  // TEST_SUITE

// arm_fir_decimate_fast_q31.c left shifts a negative accumulator: ((q63_t) acc << 32) at :140-:328 and (q31_t)(acc0 << 1) at :223-:226 and
// :339. UB in C (UBSan: "left shift of negative value", hits at :158, :172, :223-:226 and :339; this stops a sanitizer run), the
// host result is the intended one. Every case of the fast q31 kernel whose data can make an accumulator negative is marked; the cases
// "with non-negative data" below never shift a negative value and stay active in sanitizer builds.
#define UB_Q31F KNOWN_ISSUE_UB_HIDDEN("B-DDECQ-4: arm_fir_decimate_fast_q31.c shifts negative values left ((q63_t) acc0 << 32 at :140-:328 (hit at :158 and :172), acc0 << 1 at :223-:226 and :339), undefined in C; UBSan reports it as soon as an accumulator is negative; the host result equals the intended one")

// ===============================================================================================================
// arm_fir_decimate_fast_q31
// ===============================================================================================================
TEST_SUITE("DSP Filtering decimation q31") {

TEST_CASE("[C3 arm_fir_decimate_fast_q31] the output on ARM's pattern inputs equals the integer DEF bit for bit" * UB_Q31F) {
	patternEqualsDefTest<DecQ31Fast>();
}
TEST_CASE("[C5 arm_fir_decimate_fast_q31] on ARM's pattern inputs the result is 0 to 2T-1 LSB below the standard function" * UB_Q31F) {
	fastVsStdOnPattern<DecQ31Fast, DecQ31>();
}
TEST_CASE("[C3 arm_fir_decimate_fast_q31] random data, M 1..5 and 1..25 taps equals the integer DEF (2 * sum of truncated products) bit for bit" * UB_Q31F) {
	randomExactTest<DecQ31Fast>();
}
TEST_CASE("[C5 arm_fir_decimate_fast_q31] random data is within 2T LSB below and nothing above the double DEF" * UB_Q31F) { randomDoubleTest<DecQ31Fast>(); }
TEST_CASE("[C3 arm_fir_decimate_fast_q31] the impulse response is the coefficients in reverse order times 0.5, every M-th one" * UB_Q31F) {
	impulseTest<DecQ31Fast>();
}
TEST_CASE("[C3 arm_fir_decimate_fast_q31] the step response is the running sum of the reversed coefficients times 0.5" * UB_Q31F) { stepTest<DecQ31Fast>(); }
TEST_CASE("[C2 arm_fir_decimate_fast_q31] a call of 1..9 outputs writes blockSize/M samples and nothing behind them or behind the exact size state" * UB_Q31F) {
	writesExactlyTest<DecQ31Fast>();
}
TEST_CASE("[C4 arm_fir_decimate_fast_q31] consecutive blocks of 1, 2, 3, 4, 5, 7 and mixed outputs carry the state like one long block" * UB_Q31F) {
	continuityTest<DecQ31Fast>();
}
TEST_CASE("[C4 arm_fir_decimate_fast_q31] two instances do not influence each other" * UB_Q31F) { independentInstancesTest<DecQ31Fast>(); }
TEST_CASE("[C4 arm_fir_decimate_fast_q31] init again clears the history and a garbage state" * UB_Q31F) { reinitTest<DecQ31Fast>(); }
TEST_CASE("[C4 arm_fir_decimate_fast_q31] the coefficient and input arrays are not modified" * UB_Q31F) { constInputsTest<DecQ31Fast>(); }
TEST_CASE("[C4 arm_fir_decimate_fast_q31] results next to full scale: (2^31-1)^2 gives 2^31-2, INT32_MIN*(2^31-1) gives INT32_MIN" * UB_Q31F) {
	q31BoundaryTest<DecQ31Fast>();
}
TEST_CASE("[C3 arm_fir_decimate_fast_q31] a one tap filter over a strided sweep of the whole q31 range equals 2*floor(x*c / 2^32)" * UB_Q31F) {
	sweepOneTapQ31<DecQ31Fast>();
}
// NEEDS REVIEW (characterization): DOC says saturated, library wraps the final narrowing; suspected DOC/vendor defect, not decided.
// DOC of the narrowing step: arm_fir_decimate_fast_q31.c:~51 "Finally, the accumulator is saturated and converted to a 1.31 result."
// (The DOC sentence "wraps around rather than clip" is about the int64 accumulator, which does not overflow here.)
TEST_CASE("characterization: [C4 arm_fir_decimate_fast_q31] +1.0 (INT32_MIN * INT32_MIN) leaves the narrowing as 0x80000000, the DOC says saturated (NEEDS REVIEW)") {
	const std::vector<int32_t> c = {INT32_MIN};
	const int32_t r = lastOut<DecQ31Fast>(c, INT32_MIN);
	CHECK(r == INT32_MIN);  // V = 2^62 / 2^31 = 2^31 (fast: acc 2^30 in 2.30, acc << 1 = 2^31), (q31_t) wraps to INT32_MIN
}
TEST_CASE("characterization: [C4 arm_fir_decimate_fast_q31] +1.0 at every output of a 9 output call (unrolled by 4 plus tail), M or L 1, 2, 3, 5, leaves the narrowing as 0x80000000, the DOC says saturated (NEEDS REVIEW)") { wrapPositiveUnrolledTest<DecQ31Fast>(); }

TEST_CASE("[C3 arm_fir_decimate_fast_q31] random non-negative data, M 1..5 and 1..25 taps equals the integer DEF bit for bit (no negative accumulator)") {
	randomExactTest<DecQ31Fast>(true);
}
TEST_CASE("[C5 arm_fir_decimate_fast_q31] random non-negative data is within 2T LSB below and nothing above the double DEF (no negative accumulator)") {
	randomDoubleTest<DecQ31Fast>(true);
}
TEST_CASE("[C4 arm_fir_decimate_fast_q31] consecutive blocks of non-negative data carry the state like one long block (no negative accumulator)") {
	continuityTest<DecQ31Fast>(true);
}
TEST_CASE("[C4 arm_fir_decimate_fast_q31] (2^31-1)^2 gives 2^31-2 and a zero input gives zero") { q31BoundaryPositiveTest<DecQ31Fast>(); }
TEST_CASE("[C3 arm_fir_decimate_fast_q31] a one tap filter over the strided non-negative half of the q31 range equals 2*floor(x*c / 2^32)") {
	sweepOneTapQ31<DecQ31Fast>(true);
}

}  // TEST_SUITE

// ===============================================================================================================
// arm_fir_decimate_q15 and arm_fir_decimate_fast_q15
// ===============================================================================================================
TEST_SUITE("DSP Filtering decimation q15") {

TEST_CASE("[C1 arm_fir_decimate_q15] ARM's pattern test (72 configs, one block each) reaches SNR 70 and 5 LSB") {
	armPatternTest<DecQ15>(70.0, 5);  // DECIMQ15.cpp:5, :13
}
TEST_CASE("[C3 arm_fir_decimate_q15] the output on ARM's pattern inputs equals the integer DEF bit for bit") { patternEqualsDefTest<DecQ15>(); }
TEST_CASE("[C3 arm_fir_decimate_q15] random full scale data, M 1..5 and 1..25 taps equals the integer DEF (floor, saturated) bit for bit") {
	randomExactTest<DecQ15>();
}
TEST_CASE("[C5 arm_fir_decimate_q15] random full scale data is within one LSB below and nothing above the double DEF (floor)") { randomDoubleTest<DecQ15>(); }
TEST_CASE("[C3 arm_fir_decimate_q15] the impulse response is the coefficients in reverse order times 0.5, every M-th one") { impulseTest<DecQ15>(); }
TEST_CASE("[C3 arm_fir_decimate_q15] the step response is the running sum of the reversed coefficients times 0.5") { stepTest<DecQ15>(); }
TEST_CASE("[C2 arm_fir_decimate_q15] a call of 1..9 outputs writes blockSize/M samples and nothing behind them or behind the exact size state") {
	writesExactlyTest<DecQ15>();
}
TEST_CASE("[C4 arm_fir_decimate_q15] consecutive blocks of 1, 2, 3, 4, 5, 7 and mixed outputs carry the state like one long block") {
	continuityTest<DecQ15>();
}
TEST_CASE("[C4 arm_fir_decimate_q15] two instances do not influence each other") { independentInstancesTest<DecQ15>(); }
TEST_CASE("[C4 arm_fir_decimate_q15] init again clears the history and a garbage state") { reinitTest<DecQ15>(); }
TEST_CASE("[C4 arm_fir_decimate_init_q15] init stores numTaps, M, pCoeffs, pState and zeroes exactly numTaps + blockSize - 1 elements") {
	initTest<DecQ15>();
}
TEST_CASE("[C4 arm_fir_decimate_init_q15] a block size that is not a multiple of M returns ARM_MATH_LENGTH_ERROR, numTaps is not checked") {
	decimInitErrorsTest<DecQ15>();
}
TEST_CASE("characterization: [C4 arm_fir_decimate_init_q15] the error return leaves the instance and the state buffer untouched") {
	decimInitErrorLeavesEverythingTest<DecQ15>();
}
TEST_CASE("[C4 arm_fir_decimate_q15] the coefficient and input arrays are not modified") { constInputsTest<DecQ15>(); }
TEST_CASE("[C4 arm_fir_decimate_q15] results saturate to 32767 / -32768, also when the sum exceeds 32 bit (64 bit accumulator)") {
	q15SaturationTest<DecQ15>(true);
}
TEST_CASE("[C4 arm_fir_decimate_q15] the extreme single product results -32767 and 32766 do not saturate") { q15NoSaturationTest<DecQ15>(); }
TEST_CASE("[C3 arm_fir_decimate_q15] a one tap filter over every q15 input value equals clamp(x*c >> 15) for 9 coefficients") { sweepOneTapQ15<DecQ15>(); }
TEST_CASE("[C3 arm_fir_decimate_q15] 1 to 5 taps over a permutation of the whole q15 range equal the integer DEF") { sweepPermutedQ15<DecQ15>(1, 5); }

TEST_CASE("[C3 arm_fir_decimate_fast_q15] the output on ARM's pattern inputs equals the integer DEF bit for bit") { patternEqualsDefTest<DecQ15Fast>(); }
TEST_CASE("[C5 arm_fir_decimate_fast_q15] on ARM's pattern inputs the result equals the standard function") { fastVsStdOnPattern<DecQ15Fast, DecQ15>(); }
TEST_CASE("[C3 arm_fir_decimate_fast_q15] random data scaled below 32 bit accumulator overflow, M 1..5 and 1..25 taps equals the integer DEF bit for bit") {
	randomExactTest<DecQ15Fast>();
}
TEST_CASE("[C5 arm_fir_decimate_fast_q15] random data is within one LSB below and nothing above the double DEF (floor)") { randomDoubleTest<DecQ15Fast>(); }
TEST_CASE("[C3 arm_fir_decimate_fast_q15] the impulse response is the coefficients in reverse order times 0.5, every M-th one") {
	impulseTest<DecQ15Fast>();
}
TEST_CASE("[C3 arm_fir_decimate_fast_q15] the step response is the running sum of the reversed coefficients times 0.5") { stepTest<DecQ15Fast>(); }
TEST_CASE("[C2 arm_fir_decimate_fast_q15] a call of 1..9 outputs writes blockSize/M samples and nothing behind them or behind the exact size state") {
	writesExactlyTest<DecQ15Fast>();
}
TEST_CASE("[C4 arm_fir_decimate_fast_q15] consecutive blocks of 1, 2, 3, 4, 5, 7 and mixed outputs carry the state like one long block") {
	continuityTest<DecQ15Fast>();
}
TEST_CASE("[C4 arm_fir_decimate_fast_q15] two instances do not influence each other") { independentInstancesTest<DecQ15Fast>(); }
TEST_CASE("[C4 arm_fir_decimate_fast_q15] init again clears the history and a garbage state") { reinitTest<DecQ15Fast>(); }
TEST_CASE("[C4 arm_fir_decimate_fast_q15] the coefficient and input arrays are not modified") { constInputsTest<DecQ15Fast>(); }
TEST_CASE("[C4 arm_fir_decimate_fast_q15] results saturate to 32767 / -32768 while the 32 bit accumulator does not overflow") {
	q15SaturationTest<DecQ15Fast>(false);
}
TEST_CASE("[C4 arm_fir_decimate_fast_q15] the extreme single product results -32767 and 32766 do not saturate") { q15NoSaturationTest<DecQ15Fast>(); }
TEST_CASE("[C3 arm_fir_decimate_fast_q15] a one tap filter over every q15 input value equals clamp(x*c >> 15) for 9 coefficients") {
	sweepOneTapQ15<DecQ15Fast>();
}
TEST_CASE("[C3 arm_fir_decimate_fast_q15] one tap over a permutation of the whole q15 range equals the integer DEF") { sweepPermutedQ15<DecQ15Fast>(1, 1); }

}  // TEST_SUITE

// ===============================================================================================================
// arm_fir_interpolate_q31
// ===============================================================================================================
TEST_SUITE("DSP Filtering interpolation q31") {

TEST_CASE("[C1 arm_fir_interpolate_q31] ARM's pattern test (36 configs, one block each) reaches SNR 100 and 2 LSB") {
	armPatternTest<IntQ31>(100.0, 2);  // DECIMQ31.cpp:5, :13
}
TEST_CASE("[C3 arm_fir_interpolate_q31] the output on ARM's pattern inputs equals the integer DEF bit for bit") { patternEqualsDefTest<IntQ31>(); }
TEST_CASE("[C3 arm_fir_interpolate_q31] random data, L 1..5 and phase lengths 1..12 equals the integer DEF bit for bit") { randomExactTest<IntQ31>(); }
TEST_CASE("[C5 arm_fir_interpolate_q31] random data is within one LSB below and nothing above the double DEF (floor)") { randomDoubleTest<IntQ31>(); }
TEST_CASE("[C3 arm_fir_interpolate_q31] the impulse response is the coefficients in reverse order times 0.5") { impulseTest<IntQ31>(); }
TEST_CASE("[C3 arm_fir_interpolate_q31] the step response of each phase is the running sum of its reversed coefficients times 0.5") { stepTest<IntQ31>(); }
TEST_CASE("[C2 arm_fir_interpolate_q31] a call of 1..9 inputs writes blockSize*L samples and nothing behind them or behind the exact size state") {
	writesExactlyTest<IntQ31>();
}
TEST_CASE("[C4 arm_fir_interpolate_q31] consecutive blocks of 1, 2, 3, 4, 5, 7 and mixed inputs carry the state like one long block") {
	continuityTest<IntQ31>();
}
TEST_CASE("[C4 arm_fir_interpolate_q31] two instances do not influence each other") { independentInstancesTest<IntQ31>(); }
TEST_CASE("[C4 arm_fir_interpolate_q31] init again clears the history and a garbage state") { reinitTest<IntQ31>(); }
TEST_CASE("[C4 arm_fir_interpolate_init_q31] init stores L, phaseLength, pCoeffs, pState and zeroes exactly numTaps/L + blockSize - 1 elements") {
	initTest<IntQ31>();
}
TEST_CASE("characterization: [C4 arm_fir_interpolate_init_q31] a numTaps that is not a multiple of L returns ARM_MATH_LENGTH_ERROR (DOC says ARM_MATH_ARGUMENT_ERROR), other cases succeed") {
	interpInitErrorsTest<IntQ31>(true);
}
TEST_CASE("[C4 arm_fir_interpolate_init_q31] a numTaps that is not a multiple of L is rejected with a return value other than ARM_MATH_SUCCESS") {
	interpInitErrorsTest<IntQ31>(false);
}
TEST_CASE("characterization: [C4 arm_fir_interpolate_init_q31] the error return leaves the instance and the state buffer untouched") {
	interpInitErrorLeavesEverythingTest<IntQ31>();
}
TEST_CASE("[C4 arm_fir_interpolate_q31] the coefficient and input arrays are not modified") { constInputsTest<IntQ31>(); }
TEST_CASE("[C4 arm_fir_interpolate_q31] results next to full scale stay in range: (2^31-1)^2 gives 2^31-2 and INT32_MIN*(2^31-1) gives INT32_MIN+1") {
	q31BoundaryTest<IntQ31>();
}
TEST_CASE("[C3 arm_fir_interpolate_q31] a one tap filter over a strided sweep of the whole q31 range equals clamp(x*c >> 31)") { sweepOneTapQ31<IntQ31>(); }
// NEEDS REVIEW (characterization): DOC says saturated, library wraps the final narrowing; suspected DOC/vendor defect, not decided.
// DOC of the narrowing step: arm_fir_interpolate_q31.c:56 "After all multiply-accumulates are performed, the 2.62 accumulator is truncated to 1.32 format and then saturated to 1.31 format."
// (The DOC sentence "wraps around rather than clip" is about the int64 accumulator, which does not overflow here.)
TEST_CASE("characterization: [C4 arm_fir_interpolate_q31] +1.0 (INT32_MIN * INT32_MIN) leaves the narrowing as 0x80000000, the DOC says saturated (NEEDS REVIEW)") {
	const std::vector<int32_t> c = {INT32_MIN};
	const int32_t r = lastOut<IntQ31>(c, INT32_MIN);
	CHECK(r == INT32_MIN);  // V = 2^62 / 2^31 = 2^31 (fast: acc 2^30 in 2.30, acc << 1 = 2^31), (q31_t) wraps to INT32_MIN
}
// NEEDS REVIEW (characterization): DOC says saturated, library wraps the final narrowing; suspected DOC/vendor defect, not decided.
// DOC of the narrowing step: arm_fir_interpolate_q31.c:56 "After all multiply-accumulates are performed, the 2.62 accumulator is truncated to 1.32 format and then saturated to 1.31 format."
// (The DOC sentence "wraps around rather than clip" is about the int64 accumulator, which does not overflow here.)
TEST_CASE("characterization: [C4 arm_fir_interpolate_q31] -(2^32-2) LSB leaves the narrowing as +2, the DOC says saturated (NEEDS REVIEW)") {
	const std::vector<int32_t> c = {INT32_MAX, INT32_MAX};
	const int32_t r = lastOut<IntQ31>(c, INT32_MIN);
	CHECK(r == 2);  // V = 2*(-2^31)(2^31-1)/2^31 = -(2^32-2) = +2 mod 2^32
}
TEST_CASE("characterization: [C4 arm_fir_interpolate_q31] +1.0 at every output of a 9 output call (unrolled by 4 plus tail), M or L 1, 2, 3, 5, leaves the narrowing as 0x80000000, the DOC says saturated (NEEDS REVIEW)") { wrapPositiveUnrolledTest<IntQ31>(); }
TEST_CASE("characterization: [C4 arm_fir_interpolate_q31] -(2^32-2) LSB at every full window of a 9 output call, M or L 1, 2, 3, 5, leaves the narrowing as +2, the DOC says saturated (NEEDS REVIEW)") { wrapNegativeUnrolledTest<IntQ31>(); }

}  // TEST_SUITE

// ===============================================================================================================
// arm_fir_interpolate_q15.c copies the last phaseLength-1 state samples with read_q15x2_ia / write_q15x2_ia (:657-:658, used when
// phaseLength >= 5) and the host version of read_q15x2 (Include/arm_math_memory.h:105, no __ARM_FEATURE_UNALIGNED) computes
// (pQ15[1] << 16): a left shift of a negative value, UB in C (UBSan: "left shift of negative value -32768"; this stops a sanitizer run).
// The host result is the intended one. Every case that can copy a negative sample with phaseLength >= 5 is marked; the cases "with
// non-negative data" and the ones with phaseLength < 5 stay active in sanitizer builds.
#define UB_Q15I KNOWN_ISSUE_UB_HIDDEN("B-DDECQ-5: arm_fir_interpolate_q15.c:657-658 copies the state with read_q15x2_ia, which shifts a negative q15 left (pQ15[1] << 16, arm_math_memory.h:105), undefined in C; UBSan reports it for phaseLength >= 5 and a negative state sample; the host result equals the intended one")

// arm_fir_interpolate_q15
// ===============================================================================================================
TEST_SUITE("DSP Filtering interpolation q15") {

TEST_CASE("[C1 arm_fir_interpolate_q15] ARM's pattern test (36 configs, one block each) reaches SNR 70 and 5 LSB" * UB_Q15I) {
	armPatternTest<IntQ15>(70.0, 5);  // DECIMQ15.cpp:5, :13
}
TEST_CASE("[C3 arm_fir_interpolate_q15] the output on ARM's pattern inputs equals the integer DEF bit for bit" * UB_Q15I) { patternEqualsDefTest<IntQ15>(); }
TEST_CASE("[C3 arm_fir_interpolate_q15] random full scale data, L 1..5 and phase lengths 1..12 equals the integer DEF (floor, saturated) bit for bit" * UB_Q15I) {
	randomExactTest<IntQ15>();
}
TEST_CASE("[C5 arm_fir_interpolate_q15] random full scale data is within one LSB below and nothing above the double DEF (floor)" * UB_Q15I) { randomDoubleTest<IntQ15>(); }
TEST_CASE("[C3 arm_fir_interpolate_q15] the impulse response is the coefficients in reverse order times 0.5") { impulseTest<IntQ15>(); }
TEST_CASE("[C3 arm_fir_interpolate_q15] the step response of each phase is the running sum of its reversed coefficients times 0.5") { stepTest<IntQ15>(); }
TEST_CASE("[C2 arm_fir_interpolate_q15] a call of 1..9 inputs writes blockSize*L samples and nothing behind them or behind the exact size state" * UB_Q15I) {
	writesExactlyTest<IntQ15>();
}
TEST_CASE("[C4 arm_fir_interpolate_q15] consecutive blocks of 1, 2, 3, 4, 5, 7 and mixed inputs carry the state like one long block" * UB_Q15I) {
	continuityTest<IntQ15>();
}
TEST_CASE("[C4 arm_fir_interpolate_q15] two instances do not influence each other" * UB_Q15I) { independentInstancesTest<IntQ15>(); }
TEST_CASE("[C4 arm_fir_interpolate_q15] init again clears the history and a garbage state" * UB_Q15I) { reinitTest<IntQ15>(); }
TEST_CASE("[C4 arm_fir_interpolate_init_q15] init stores L, phaseLength, pCoeffs, pState and zeroes exactly numTaps/L + blockSize - 1 elements") {
	initTest<IntQ15>();
}
TEST_CASE("characterization: [C4 arm_fir_interpolate_init_q15] a numTaps that is not a multiple of L returns ARM_MATH_LENGTH_ERROR (DOC says ARM_MATH_ARGUMENT_ERROR), other cases succeed") {
	interpInitErrorsTest<IntQ15>(true);
}
TEST_CASE("[C4 arm_fir_interpolate_init_q15] a numTaps that is not a multiple of L is rejected with a return value other than ARM_MATH_SUCCESS") {
	interpInitErrorsTest<IntQ15>(false);
}
TEST_CASE("characterization: [C4 arm_fir_interpolate_init_q15] the error return leaves the instance and the state buffer untouched") {
	interpInitErrorLeavesEverythingTest<IntQ15>();
}
TEST_CASE("[C4 arm_fir_interpolate_q15] the coefficient and input arrays are not modified" * UB_Q15I) { constInputsTest<IntQ15>(); }
TEST_CASE("[C4 arm_fir_interpolate_q15] results saturate to 32767 / -32768 for 1 and 2 taps") { q15SaturationTest<IntQ15>(false); }
TEST_CASE("[C4 arm_fir_interpolate_q15] results saturate to 32767 / -32768 also when the sum exceeds 32 bit (8 taps, 64 bit accumulator)" * UB_Q15I) {
	q15SaturationTest<IntQ15>(true);
}
TEST_CASE("[C4 arm_fir_interpolate_q15] the extreme single product results -32767 and 32766 do not saturate") { q15NoSaturationTest<IntQ15>(); }
TEST_CASE("[C3 arm_fir_interpolate_q15] a one tap filter (L = 1) over every q15 input value equals clamp(x*c >> 15) for 9 coefficients") {
	sweepOneTapQ15<IntQ15>();
}
TEST_CASE("[C3 arm_fir_interpolate_q15] L = 2 with one tap per phase over every q15 input value equals the direct formula") { sweepInterpL2Q15<IntQ15>(); }
TEST_CASE("[C3 arm_fir_interpolate_q15] 1 to 4 taps (L = 1) over a permutation of the whole q15 range equal the integer DEF") { sweepPermutedQ15<IntQ15>(1, 4); }
TEST_CASE("[C3 arm_fir_interpolate_q15] 5 taps (L = 1) over a permutation of the whole q15 range equal the integer DEF (the last four state samples of the sweep are positive: no UB)") { sweepPermutedQ15<IntQ15>(5, 5); }
TEST_CASE("[C3 arm_fir_interpolate_q15] random non-negative data, L 1..5 and phase lengths 1..12 equals the integer DEF bit for bit (no negative state sample)") {
	randomExactTest<IntQ15>(true);
}
TEST_CASE("[C5 arm_fir_interpolate_q15] random non-negative data is within one LSB below and nothing above the double DEF (no negative state sample)") {
	randomDoubleTest<IntQ15>(true);
}
TEST_CASE("[C4 arm_fir_interpolate_q15] consecutive blocks of non-negative data carry the state like one long block (no negative state sample)") {
	continuityTest<IntQ15>(true);
}

}  // TEST_SUITE
