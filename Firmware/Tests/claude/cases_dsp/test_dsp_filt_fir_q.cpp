/*
 * CMSIS-DSP Filtering, FIR fixed point (work order FIRQ, phase P11 group G4): arm_fir_q31 / arm_fir_q15 / arm_fir_q7,
 * arm_fir_fast_q31 / arm_fir_fast_q15 and their init functions arm_fir_init_q31 / _q15 / _q7 (fast variants share the init).
 * Requirement ids: Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * Pattern files (Libraries/CMSIS-DSP/Testing/Patterns/DSP/Filtering/FIR/FIRQ31/, FIRQ15/, FIRQ7/), suites "FIR Q31", "FIR Q15",
 * "FIR Q7" (Testing/desc.txt:3187-3239), ARM test functions Testing/Source/Tests/FIRQ31.cpp, FIRQ15.cpp, FIRQ7.cpp (test_fir_q31
 * etc., :24-:125). ids: arm_fir_q31, arm_fir_q15, arm_fir_q7 (1 test each). ARM has NO id for arm_fir_fast_q31 / arm_fir_fast_q15
 * (desc.txt lists only the three); nothing is commented out or disabled. Each test uses FirInput1 (input: the 46 values of
 * x = normalize(sin + noise) / 30, "to avoid saturation", FIR.py), FirCoefs1 (the coefficients of ALL configs, concatenated, time
 * reversed, python value k/(numTaps+2) for k = 1..numTaps), FirConfigs1_s16 (pairs blockSize, numTaps), FirRefs1 (scipy lfilter
 * reference, 2*blockSize values per config). Configs (FIR.py:64-72): q31 blocks {1,2,3,8,9,10,11,16,23} x taps {1,2,3,4,5,6,7,8,
 * 11,16,23,25}; q15 blocks {1,2,3,12,13,14,15} x taps {2,3,4,5,6,7,8,11,25} with every ODD numTaps increased by one and a zero
 * coefficient appended (the config file holds the padded numTaps); q7 blocks {1,2,3,20,21,22,23} x taps {1,2,3,4,5,6,7,8,11,25}.
 * For every config the filter is initialised and 2*blockSize inputs are filtered in TWO calls of blockSize (state continuity).
 * ARM state buffers are larger (q31 141, q15 123, q7 47 elements); here the state buffer has the length of the DOC (below).
 * ARM tolerances (cited, used unchanged, L4): q31 SNR_THRESHOLD 100 (FIRQ31.cpp:5), ABS_ERROR_Q31 2 (:7); q15 SNR 59
 * (FIRQ15.cpp:5), ABS_ERROR_Q15 2 (:7); q7 SNR 10 (FIRQ7.cpp:5), ABS_ERROR_Q7 2 (:7). ASSERT_NEAR_EQ = |out - ref| <= 2 per
 * sample (withinLsb), ASSERT_SNR = dsp::snr (double). ARM also checks 4 zeros behind each block (checkInnerTail) and the tail
 * of the whole output: here a guard behind every block (GuardedBuffer).
 *
 * DOC (arm_fir_q31.c:51-56, arm_fir_q15.c, arm_fir_q7.c, arm_fir_fast_q31.c, arm_fir_fast_q15.c, arm_fir_init_*.c):
 *   y[n] = b[0]*x[n] + ... + b[T-1]*x[n-T+1]; coefficients time reversed: pCoeffs = {b[T-1], ..., b[0]}; so with c = pCoeffs the
 *   exact sum is acc[n] = sum_k c[k] * x[n-T+1+k] (x[m] = 0 for m < 0; "init zeros the state"). Scaling (derived below):
 *   q31: 64 bit accumulator, 1.31 x 1.31 = 2.62, "single guard bit", result = acc >> 31 (arithmetic shift = floor);
 *   q15: 64 bit accumulator 34.30, "truncated to 34.15 by discarding the low 15 bits ... saturated to 1.15": y = sat16(floor(acc / 2^15));
 *   q7: 32 bit accumulator 18.14, "converted to 18.7 by discarding the low 7 bits. Finally truncated to 1.7": y = floor(acc / 2^7)
 *       (the code saturates with __SSAT, the DOC word is "truncated", see Lead decisions);
 *   fast q31: each 1.31 x 1.31 product is rounded to 32 bit ("multAcc_32x32_keep32_R" = SMMLAR: (a*2^32 + c*x + 2^31) >> 32, so
 *       a += floor((c*x + 2^31) / 2^32)), summed in a 2.30 q31 accumulator, result = acc << 1;
 *   fast q15: 32 bit accumulator 2.30, y = sat16(acc >> 15), wraps if the accumulator overflows (DOC).
 *   State lengths (init DOCs): q31 and q7 numTaps + blockSize - 1; q15 numTaps + blockSize "when ARM_MATH_DSP is defined",
 *   numTaps + blockSize - 1 otherwise. q15: "numTaps must be even and greater than or equal to 4".
 * The DEF is exact integer arithmetic (__int128, GCC / clang / MinGW-w64 have it): acc = sum c*x and, for the fast q31, the sum of
 * the rounded products. Results are compared EXACTLY to the DEF model above (no tolerance needed: truncation is deterministic), the
 * floor error is additionally asserted one-sided (0 <= acc - y*2^shift < 2^shift) independently of the model, and the fast q31
 * rounding error with the bound derived in the test.
 * Overflow-free data: the random tests use |c| <= 2^b / T (b = 30 for q31, 15 for q15, 7 for q7; the full range for x), so that
 * sum |c*x| <= 2^(2b) and no accumulator or result overflows (q31: |acc| <= 2^61, the fast q31 sum of rounded terms < 2^30 + T/2).
 *
 * Host vs target (C6): the host runs the portable C path (no ARM_MATH_DSP, no MVE: the DSP-style code with the C intrinsics of
 * dsp/none.h, ARM_MATH_LOOPUNROLL); a Cortex-M4 build defines ARM_MATH_DSP, which only changes init for q15 (odd numTaps rejected,
 * state numTaps + blockSize) and uses SMLALD/SMLAD instead of the C versions; the results are the same when nothing overflows.
 * q15 / fast q15 / fast q31 undefined behaviour on the host (UBSan, -fno-sanitize-recover aborts the run), so the test cases that
 * reach it are KNOWN_ISSUE_UB_HIDDEN (intended result on the host, skipped in sanitizer builds; the findings are listed by the id):
 *   B-DFIRQ-1  q15 and fast q15 with blockSize >= 4 (the unrolled path): arm_math_memory.h:105 read_q15x2 "(pQ15[1] << 16)" shifts a
 *              negative int left, dsp/none.h:460 (__SMLAD) and :503 (__SMLALD) shift a packed pair "(q31_t)x << 16" left (a value above 2^15
 *              does not fit): "left shift of negative value" / "cannot be represented in type 'int'". Blocks of 1..3 use only the scalar
 *              tail loop and are clean.
 *   B-DFIRQ-2  fast q31 with a negative accumulator: arm_fir_fast_q31.c:222, :223 and :269 "acc0 << 1", and the macro
 *              multAcc_32x32_keep32_R (dsp/none.h:185, call sites such as arm_fir_fast_q31.c:160) "((q63_t) a) << 32" shift a negative
 *              q31 / q63 left (undefined in C). Non-negative data are clean.
 *              (also: a 2.30 accumulator above 2^30 makes "acc0 << 1" overflow int: the tests never feed such data to the fast q31)
 *   B-DFIRQ-3  arm_fir_q15 / arm_fir_fast_q15 with blockSize >= 4 read 2 bytes behind the state buffer of numTaps + blockSize - 1
 *              (the length the DOC gives for builds without ARM_MATH_DSP, i.e. this host build): the second read_q15x2 pair of the
 *              4 taps unrolled loop reads (n+T, n+T+1) while the last needed sample is n+T-1; the pair's upper half is never used (results
 *              are unaffected). Evidence (ASan only, separate probe, T = 4, B = 4, 7 element heap state): "READ of size 2 ... 0 bytes after
 *              14-byte region" at arm_math_memory.h:105 called from arm_fir_q15.c:568. The main tests therefore give the state
 *              numTaps + blockSize (the DSP length of the DOC); UBSan stops first in the unrolled path (B-DFIRQ-1), so the exact length test
 *              is KNOWN_ISSUE_UB_HIDDEN.
 *   B-DFIRQ-4  arm_fir_init_q15 without ARM_MATH_DSP accepts an odd numTaps. DOC (arm_fir_init_q15.c @return): "ARM_MATH_ARGUMENT_ERROR :
 *              numTaps is not greater than or equal to 4 and even".
 *
 * NEEDS REVIEW (no test or characterization only; details in the report):
 *   - numTaps = 2 for q15: the code accepts it (even), the DOC says "greater than or equal to 4", ARM's own FIR q15 configs use numTaps = 2
 *     (FIR.py:67: taps start at 2). No test of its rejection; it is run like ARM does.
 *   - q31: a result above the 1.31 range wraps (DOC arm_fir_q31.c:53-54 "wraps around rather than clip") but the last DOC sentence (:56) says
 *     "saturated to 1.31 format": contradiction; test is `characterization:` (q31WrapTest).
 *   - fast q31: the DOC says "truncated to 2.30 format" for each product and "the accumulator is saturated"; the code ROUNDS each product
 *     (SMMLAR, +2^31) and does not saturate (acc0 << 1, wraps / undefined): the tests that compare the fast q31 with the exact model of the
 *     rounding and the rounding tie are `characterization:`; overflow of the fast q31 is not tested.
 *   - q7: DOC "truncated to 1.7", code saturates (`characterization:`).
 * Host compiler: the DEF uses __int128, so a 64 bit host compiler is required (GCC / clang on a 64 bit target, MinGW-w64 x86_64; not a 32 bit
 * MinGW). On a build with ARM_MATH_DSP defined the exact length test exactStateTest (B-DFIRQ-3) is compiled out.
 * The 32 bit int overflow of the fast q15 / fast q31 accumulators ("wraps", DOC) is undefined in C and not tested (NEEDS REVIEW).
 *
 * Not tested, NEEDS REVIEW: blockSize 0 (kernel arithmetic unspecified), numTaps 0, numTaps 1 / odd for q15 (outside the
 * contract), the wrap-around of the fast accumulators, results that overflow the q31 / fast q31 range (DOC contradicts itself:
 * see Lead decisions in the report).
 */
#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math.h"

namespace {

typedef __int128 i128;

// ---------------------------------------------------------------------------------------------------------------
// Deterministic data. Every call is its own statement (lesson: never two PRNG calls in one expression).
// ---------------------------------------------------------------------------------------------------------------
struct Rng {
	uint64_t s;
	explicit Rng(uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ull + 0x1234567ull) {
		if (s == 0) s = 1;
	}
	uint32_t next32() {  // xorshift64, high half
		s ^= s << 13;
		s ^= s >> 7;
		s ^= s << 17;
		return static_cast<uint32_t>(s >> 32);
	}
	// uniform integer in [lo, hi] (span <= 2^32)
	int64_t range(int64_t lo, int64_t hi) {
		const uint64_t span = static_cast<uint64_t>(hi - lo) + 1u;
		const uint32_t r = next32();
		return lo + static_cast<int64_t>((static_cast<uint64_t>(r) * span) >> 32);
	}
};

// ---------------------------------------------------------------------------------------------------------------
// Variants: which block sizes and which data signs a test case may use (the UB findings of the header)
// ---------------------------------------------------------------------------------------------------------------
struct Var {
	size_t lo, hi;  // block size range of the plans / blocks
	bool neg;       // negative data allowed
};
const size_t kBlockMaster[] = {1, 2, 3, 4, 5, 7, 8, 9, 16, 17};
std::vector<size_t> blocksOf(const Var &v) {
	std::vector<size_t> r;
	for (size_t b : kBlockMaster)
		if (b >= v.lo && b <= v.hi) r.push_back(b);
	return r;
}

// a block size inside the variant's range, as near as possible to pref
size_t pickBlock(const Var &v, size_t pref) { return std::min(std::max(pref, v.lo), v.hi); }

// ---------------------------------------------------------------------------------------------------------------
// Type traits. out() is the DEF model of the output from the exact sums, usable() tells which samples have a well defined
// intended result (no overflow of a wrapping accumulator or of the q31 result, no q7 clipping: DOC "truncated" is ambiguous).
// ---------------------------------------------------------------------------------------------------------------
const i128 kOne = 1;

struct Q31T {
	typedef int32_t S;
	typedef arm_fir_instance_q31 Inst;
	static constexpr int bits = 31;      // fraction bits of the type = the right shift
	static constexpr bool fast = false;
	static const char *sfx() { return "q31"; }
	static std::string dir() { return "Filtering/FIR/FIRQ31/"; }
	static i128 smin() { return -(kOne << 31); }
	static i128 smax() { return (kOne << 31) - 1; }
	static size_t procLen(size_t T, size_t B) { return T + B - 1; }
	static size_t initLen(size_t T, size_t B) { return T + B - 1; }
	static int init(Inst *i, uint16_t t, const S *c, S *st, uint32_t b) { arm_fir_init_q31(i, t, c, st, b); return 0; }
	static void run(const Inst *i, const S *x, S *y, uint32_t b) { arm_fir_q31(i, x, y, b); }
	static i128 out(i128 acc, i128) { return acc >> 31; }
	static bool usable(i128 acc, i128 accR) { const i128 y = out(acc, accR); return y >= smin() && y <= smax(); }
	static int64_t cLimit(size_t T) { return (int64_t(1) << 30) / static_cast<int64_t>(T); }
	static std::vector<S> pat(const std::string &n) { return dsp::loadPattern(dir() + n).q31(); }
	static Var cleanVar() { return {1, 40, true}; }
	static std::vector<size_t> taps() { return {1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 17, 25, 33}; }
};

struct FQ31T : Q31T {
	static constexpr bool fast = true;
	static const char *sfx() { return "q31"; }
	static void run(const Inst *i, const S *x, S *y, uint32_t b) { arm_fir_fast_q31(i, x, y, b); }
	static i128 out(i128, i128 accR) { return accR * 2; }
	// the 2.30 accumulator and its left shift must not overflow (the wrap is undefined behaviour and not tested)
	static bool usable(i128, i128 accR) { return accR >= -(kOne << 30) && accR <= (kOne << 30) - 1; }
	static Var cleanVar() { return {1, 40, false}; }   // non-negative data: no negative left shift (B-DFIRQ-2)
	static Var hiddenVar() { return {1, 40, true}; }   // signed data
};

struct Q15T {
	typedef int16_t S;
	typedef arm_fir_instance_q15 Inst;
	static constexpr int bits = 15;
	static constexpr bool fast = false;
	static const char *sfx() { return "q15"; }
	static std::string dir() { return "Filtering/FIR/FIRQ15/"; }
	static i128 smin() { return -(kOne << 15); }
	static i128 smax() { return (kOne << 15) - 1; }
	// DOC arm_fir_init_q15.c: numTaps + blockSize with ARM_MATH_DSP, numTaps + blockSize - 1 without. The kernel reads one element more
	// than the second length (B-DFIRQ-3), so the processing tests use the first.
	static size_t procLen(size_t T, size_t B) { return T + B; }
	static size_t initLen(size_t T, size_t B) {
#if defined(ARM_MATH_DSP)
		return T + B;
#else
		return T + B - 1;
#endif
	}
	static int init(Inst *i, uint16_t t, const S *c, S *st, uint32_t b) { return static_cast<int>(arm_fir_init_q15(i, t, c, st, b)); }
	static void run(const Inst *i, const S *x, S *y, uint32_t b) { arm_fir_q15(i, x, y, b); }
	static i128 sat(i128 v) { return v < smin() ? smin() : (v > smax() ? smax() : v); }
	static i128 out(i128 acc, i128) { return sat(acc >> 15); }
	static bool usable(i128, i128) { return true; }
	static int64_t cLimit(size_t T) { return (int64_t(1) << 15) / static_cast<int64_t>(T); }
	static std::vector<S> pat(const std::string &n) { return dsp::loadPattern(dir() + n).q15(); }
	static Var cleanVar() { return {1, 3, true}; }    // scalar tail loop only
	static Var hiddenVar() { return {4, 40, true}; }  // unrolled path (B-DFIRQ-1, B-DFIRQ-3)
	static std::vector<size_t> taps() { return {2, 4, 6, 8, 10, 12, 18, 26}; }
};

struct FQ15T : Q15T {
	static constexpr bool fast = true;
	static const char *sfx() { return "q15"; }
	static void run(const Inst *i, const S *x, S *y, uint32_t b) { arm_fir_fast_q15(i, x, y, b); }
	// the 32 bit accumulator wraps otherwise (undefined behaviour, not tested)
	static bool usable(i128 acc, i128) { return acc >= -(kOne << 31) && acc <= (kOne << 31) - 1; }
};

struct Q7T {
	typedef int8_t S;
	typedef arm_fir_instance_q7 Inst;
	static constexpr int bits = 7;
	static constexpr bool fast = false;
	static const char *sfx() { return "q7"; }
	static std::string dir() { return "Filtering/FIR/FIRQ7/"; }
	static i128 smin() { return -(kOne << 7); }
	static i128 smax() { return (kOne << 7) - 1; }
	static size_t procLen(size_t T, size_t B) { return T + B - 1; }
	static size_t initLen(size_t T, size_t B) { return T + B - 1; }
	static int init(Inst *i, uint16_t t, const S *c, S *st, uint32_t b) { arm_fir_init_q7(i, t, c, st, b); return 0; }
	static void run(const Inst *i, const S *x, S *y, uint32_t b) { arm_fir_q7(i, x, y, b); }
	static i128 sat(i128 v) { return v < smin() ? smin() : (v > smax() ? smax() : v); }
	static i128 out(i128 acc, i128) { return sat(acc >> 7); }
	// DOC says "truncated to 1.7", the code saturates: results that need clipping are not compared by the general tests
	static bool usable(i128 acc, i128) { const i128 y = acc >> 7; return y >= smin() && y <= smax(); }
	static int64_t cLimit(size_t T) { return (int64_t(1) << 7) / static_cast<int64_t>(T); }
	static std::vector<S> pat(const std::string &n) { return dsp::loadPattern(dir() + n).q7(); }
	static Var cleanVar() { return {1, 40, true}; }
	static std::vector<size_t> taps() { return {1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 17, 25, 33}; }
};

// Q31T::hiddenVar does not exist (those types have no hidden variant); the macros below only use it for the split types.

// ---------------------------------------------------------------------------------------------------------------
// DEF and runner
// ---------------------------------------------------------------------------------------------------------------
struct Def {
	std::vector<i128> acc, accR;
};

// acc[n] = sum_k c[k] x[n-T+1+k]; accR[n] = sum of the rounded products floor((c x + 2^31) / 2^32)
template <class Tr>
Def defFir(const std::vector<typename Tr::S> &c, const std::vector<typename Tr::S> &x) {
	const long T = static_cast<long>(c.size());
	Def d;
	d.acc.assign(x.size(), 0);
	d.accR.assign(x.size(), 0);
	for (long n = 0; n < static_cast<long>(x.size()); n++) {
		i128 a = 0, r = 0;
		for (long k = 0; k < T; k++) {
			const long m = n - T + 1 + k;
			if (m < 0) continue;
			const i128 p = static_cast<i128>(c[static_cast<size_t>(k)]) * static_cast<i128>(x[static_cast<size_t>(m)]);
			a += p;
			r += (p + (kOne << 31)) >> 32;
		}
		d.acc[static_cast<size_t>(n)] = a;
		d.accR[static_cast<size_t>(n)] = r;
	}
	return d;
}

template <class Tr>
std::vector<typename Tr::S> makeVec(Rng &rng, size_t n, int64_t lo, int64_t hi) {
	std::vector<typename Tr::S> v(n);
	for (size_t i = 0; i < n; i++) {
		const int64_t r = rng.range(lo, hi);
		v[i] = static_cast<typename Tr::S>(r);
	}
	return v;
}
// coefficients limited so that nothing overflows (header); the input uses the full range of the type
template <class Tr>
std::vector<typename Tr::S> makeCoefs(Rng &rng, size_t T, const Var &v) {
	int64_t lim = Tr::cLimit(T) < 1 ? 1 : Tr::cLimit(T);
	if (lim > static_cast<int64_t>(Tr::smax())) lim = static_cast<int64_t>(Tr::smax());
	return makeVec<Tr>(rng, T, v.neg ? -lim : 0, lim);
}
template <class Tr>
std::vector<typename Tr::S> makeInput(Rng &rng, size_t n, const Var &v) {
	return makeVec<Tr>(rng, n, v.neg ? static_cast<int64_t>(Tr::smin()) : 0, static_cast<int64_t>(Tr::smax()));
}

template <class Tr>
struct RunResult {
	std::vector<typename Tr::S> out;
	bool guardsOk = true;   // every output block guard and the state guard intact
	std::vector<bool> written;  // output element differs from the guard buffer's poison (or equals it by chance: see below)
};

template <class Tr>
typename Tr::S poisonValue() {
	unsigned char raw[sizeof(typename Tr::S)];
	std::memset(raw, 0xCD, sizeof raw);
	typename Tr::S v;
	std::memcpy(&v, raw, sizeof v);
	return v;
}

// Filters x with taps c in consecutive calls of the sizes in `chunks` (cycled, last one truncated). The state buffer has exactly
// Tr::procLen(numTaps, maxChunk) elements and is initialised with blockSize = maxChunk. By default it is a tight heap allocation of
// exactly that length (poisoned first, no slack, no guard words in the same allocation), so that ASan sees any access past its end;
// with guardedState the buffer is a GuardedBuffer and the guard words behind it are checked (a write past the end that stays inside
// the guard words is invisible to ASan, that is what the guard checks).
template <class Tr>
RunResult<Tr> firRun(const std::vector<typename Tr::S> &c, const std::vector<typename Tr::S> &x, const std::vector<size_t> &chunks,
                     bool guardedState = false) {
	typedef typename Tr::S S;
	RunResult<Tr> r;
	size_t maxChunk = 0;
	for (size_t k : chunks) maxChunk = k > maxChunk ? k : maxChunk;
	const size_t stateLen = Tr::procLen(c.size(), maxChunk);
	dsp::GuardedBuffer<S> state(guardedState ? stateLen : 0);
	std::unique_ptr<S[]> tight(new S[stateLen]);
	std::memset(static_cast<void *>(tight.get()), 0xCD, stateLen * sizeof(S));
	S *const statePtr = guardedState ? state.data() : tight.get();
	typename Tr::Inst inst;
	Tr::init(&inst, static_cast<uint16_t>(c.size()), c.data(), statePtr, static_cast<uint32_t>(maxChunk));
	size_t pos = 0, ci = 0;
	while (pos < x.size()) {
		size_t b = chunks[ci++ % chunks.size()];
		if (b > x.size() - pos) b = x.size() - pos;
		dsp::GuardedBuffer<S> out(b);
		Tr::run(&inst, x.data() + pos, out.data(), static_cast<uint32_t>(b));
		if (!out.guardIntact()) r.guardsOk = false;
		const std::vector<S> o = out.output();
		const S poison = poisonValue<Tr>();
		for (size_t i = 0; i < b; i++) {
			r.out.push_back(o[i]);
			r.written.push_back(std::memcmp(&o[i], &poison, sizeof(S)) != 0);
		}
		pos += b;
	}
	if (guardedState && !state.guardIntact()) r.guardsOk = false;
	return r;
}

struct Cmp {
	size_t bad = 0, usedSamples = 0, skipped = 0;
	std::string first;
	bool guardsOk = true;
};

// runs the filter and compares every usable output EXACTLY with the model
template <class Tr>
Cmp compareToModel(const std::vector<typename Tr::S> &c, const std::vector<typename Tr::S> &x, const std::vector<size_t> &chunks,
                   const std::string &tag) {
	Cmp r;
	const RunResult<Tr> run = firRun<Tr>(c, x, chunks);
	r.guardsOk = run.guardsOk;
	REQUIRE(run.out.size() == x.size());
	const Def d = defFir<Tr>(c, x);
	for (size_t n = 0; n < x.size(); n++) {
		if (!Tr::usable(d.acc[n], d.accR[n])) {
			r.skipped++;
			continue;
		}
		r.usedSamples++;
		if (static_cast<i128>(run.out[n]) != Tr::out(d.acc[n], d.accR[n])) {
			if (r.bad == 0)
				r.first = tag + " n " + std::to_string(n) + " out " + std::to_string(static_cast<long long>(run.out[n])) + " model " +
				          std::to_string(static_cast<long long>(Tr::out(d.acc[n], d.accR[n])));
			r.bad++;
		}
	}
	return r;
}

// ---------------------------------------------------------------------------------------------------------------
// C1: ARM's test and the ARM reference against the DEF
// ---------------------------------------------------------------------------------------------------------------
// maxBlock: configs with a larger blockSize are skipped (the unrolled path of the q15 filters is the hidden variant)
template <class Tr>
void armPatternTest(double snrThr, int64_t absLsb, size_t maxBlock, bool useSnr) {
	typedef typename Tr::S S;
	const std::vector<S> input = Tr::pat(std::string("FirInput1_") + Tr::sfx() + ".txt");
	const std::vector<S> coefs = Tr::pat(std::string("FirCoefs1_") + Tr::sfx() + ".txt");
	const std::vector<S> ref = Tr::pat(std::string("FirRefs1_") + Tr::sfx() + ".txt");
	const std::vector<int16_t> cfg = dsp::loadPattern(Tr::dir() + "FirConfigs1_s16.txt").q15();
	std::vector<S> all, expect;
	size_t cp = 0, rp = 0, nCfg = 0;
	bool guards = true, fields = true, status = true;
	for (size_t i = 0; i + 1 < cfg.size(); i += 2) {
		const size_t B = static_cast<size_t>(cfg[i]), T = static_cast<size_t>(cfg[i + 1]);
		REQUIRE(cp + T <= coefs.size());
		REQUIRE(2 * B <= input.size());
		REQUIRE(rp + 2 * B <= ref.size());
		if (B <= maxBlock) {
			// tight heap state of exactly the length used here (no slack: ARM's own test uses a larger buffer), poisoned
			std::unique_ptr<S[]> state(new S[Tr::procLen(T, B)]);
			std::memset(static_cast<void *>(state.get()), 0xCD, Tr::procLen(T, B) * sizeof(S));
			typename Tr::Inst inst;
			status = status && Tr::init(&inst, static_cast<uint16_t>(T), coefs.data() + cp, state.get(), static_cast<uint32_t>(B)) == 0;
			fields = fields && inst.numTaps == T && inst.pCoeffs == coefs.data() + cp && inst.pState == state.get();
			for (size_t blk = 0; blk < 2; blk++) {
				dsp::GuardedBuffer<S> out(B);
				Tr::run(&inst, input.data() + blk * B, out.data(), static_cast<uint32_t>(B));
				guards = guards && out.guardIntact();
				const std::vector<S> o = out.output();
				all.insert(all.end(), o.begin(), o.end());
			}
			expect.insert(expect.end(), ref.begin() + static_cast<long>(rp), ref.begin() + static_cast<long>(rp + 2 * B));
			nCfg++;
		}
		cp += T;
		rp += 2 * B;
	}
	CHECK(nCfg > 0);
	CHECK(status);
	CHECK(fields);
	CHECK(guards);
	CHECK(cp == coefs.size());
	CHECK(rp == ref.size());
	REQUIRE(all.size() == expect.size());
	if (useSnr) CHECK(dsp::snr(expect, all) >= snrThr);
	auto c = dsp::withinLsb(expect, all, absLsb);
	CHECK_MESSAGE(c, c.message);
}

// ARM's reference against the DEF with ARM's absolute tolerance: checks the coefficient order convention of the DEF
template <class Tr>
void patternIsDefTest(int64_t absLsb) {
	typedef typename Tr::S S;
	const std::vector<S> input = Tr::pat(std::string("FirInput1_") + Tr::sfx() + ".txt");
	const std::vector<S> coefs = Tr::pat(std::string("FirCoefs1_") + Tr::sfx() + ".txt");
	const std::vector<S> ref = Tr::pat(std::string("FirRefs1_") + Tr::sfx() + ".txt");
	const std::vector<int16_t> cfg = dsp::loadPattern(Tr::dir() + "FirConfigs1_s16.txt").q15();
	size_t cp = 0, rp = 0, bad = 0, nonzero = 0;
	std::string first;
	for (size_t i = 0; i + 1 < cfg.size(); i += 2) {
		const size_t B = static_cast<size_t>(cfg[i]), T = static_cast<size_t>(cfg[i + 1]);
		const std::vector<S> c(coefs.begin() + static_cast<long>(cp), coefs.begin() + static_cast<long>(cp + T));
		const std::vector<S> x(input.begin(), input.begin() + static_cast<long>(2 * B));
		const Def d = defFir<Tr>(c, x);
		for (size_t n = 0; n < 2 * B; n++) {
			const i128 m = Tr::out(d.acc[n], d.accR[n]);
			const i128 e = m > static_cast<i128>(ref[rp + n]) ? m - ref[rp + n] : static_cast<i128>(ref[rp + n]) - m;
			if (ref[rp + n] != 0) nonzero++;
			if (e > absLsb) {
				if (bad == 0) first = "config " + std::to_string(i / 2) + " sample " + std::to_string(n);
				bad++;
			}
		}
		cp += T;
		rp += 2 * B;
	}
	CHECK(nonzero > 100);
	CHECK_MESSAGE(bad == 0, bad << " samples differ from the DEF by more than " << absLsb << " LSB, first at " << first);
}

// The fast variants have no ARM id: ARM's patterns with the bound derived from the DEF.
// fast q15: identical to the standard filter as long as the accumulator does not overflow (the input is scaled /30): ARM's bounds.
// fast q31: each product is rounded (error <= 1/2 of the 2.30 LSB), the result is 2 * sum: |y - acc/2^31| <= T LSB of q31 (+1 floor of the
// reference model, + ARM's 2 LSB between pattern and model): |y - ref| <= T + 1 + 2.
template <class Tr>
void fastPatternTest(size_t maxBlock) {
	typedef typename Tr::S S;
	const std::vector<S> input = Tr::pat(std::string("FirInput1_") + Tr::sfx() + ".txt");
	const std::vector<S> coefs = Tr::pat(std::string("FirCoefs1_") + Tr::sfx() + ".txt");
	const std::vector<S> ref = Tr::pat(std::string("FirRefs1_") + Tr::sfx() + ".txt");
	const std::vector<int16_t> cfg = dsp::loadPattern(Tr::dir() + "FirConfigs1_s16.txt").q15();
	size_t cp = 0, rp = 0, bad = 0, n = 0;
	std::string first;
	bool guards = true;
	for (size_t i = 0; i + 1 < cfg.size(); i += 2) {
		const size_t B = static_cast<size_t>(cfg[i]), T = static_cast<size_t>(cfg[i + 1]);
		if (B <= maxBlock) {
			const std::vector<S> c(coefs.begin() + static_cast<long>(cp), coefs.begin() + static_cast<long>(cp + T));
			const std::vector<S> x(input.begin(), input.begin() + static_cast<long>(2 * B));
			const RunResult<Tr> r = firRun<Tr>(c, x, {B});
			guards = guards && r.guardsOk;
			REQUIRE(r.out.size() == 2 * B);
			const int64_t bound = Tr::bits == 31 ? static_cast<int64_t>(T) + 1 + 2 : 2;
			for (size_t k = 0; k < 2 * B; k++) {
				const int64_t e = std::llabs(static_cast<int64_t>(r.out[k]) - static_cast<int64_t>(ref[rp + k]));
				n++;
				if (e > bound) {
					if (bad == 0) first = "config " + std::to_string(i / 2) + " sample " + std::to_string(k);
					bad++;
				}
			}
		}
		cp += T;
		rp += 2 * B;
	}
	CHECK(n > 100);
	CHECK(guards);
	CHECK_MESSAGE(bad == 0, bad << " samples outside the bound, first at " << first);
}

// ---------------------------------------------------------------------------------------------------------------
// C3 / C4
// ---------------------------------------------------------------------------------------------------------------
// unit of the exact tests: even for all, multiple of 4 where it is used by the fast q31 (header of impulse / step test)
template <class Tr>
int64_t unit() {
	return Tr::bits == 31 ? (int64_t(1) << 24) : (Tr::bits == 15 ? (int64_t(1) << 8) : 2);
}
// half of the full scale: x0 = 2^(bits-1): c * x0 / 2^bits = c / 2
template <class Tr>
typename Tr::S half() {
	return static_cast<typename Tr::S>(int64_t(1) << (Tr::bits - 1));
}

// Impulse response: x = {0.5, 0, ...}: y[m] = floor(b[m] * 0.5) = c[T-1-m] / 2 (the coefficients are even; multiples of 4 for the fast
// q31, whose product c*2^30 / 2^32 = c/4 is then exact, so that rounding does not matter)
template <class Tr>
void impulseTest(const Var &v) {
	typedef typename Tr::S S;
	size_t checked = 0;
	for (size_t T : Tr::taps()) {
		if (T > 12) continue;
		std::vector<S> c(T);
		for (size_t k = 0; k < T; k++) {
			const int64_t sign = (v.neg && (k % 2)) ? -1 : 1;
			c[k] = static_cast<S>(sign * static_cast<int64_t>(k + 1) * unit<Tr>());
		}
		std::vector<S> x(T + 5, 0);
		x[0] = half<Tr>();
		for (size_t pref : {T + 5, size_t(1), size_t(3)}) {
			const size_t B = pickBlock(v, pref);
			const RunResult<Tr> r = firRun<Tr>(c, x, {B});
			std::vector<S> expect(T + 5, 0);
			for (size_t m = 0; m < T; m++) expect[m] = static_cast<S>(c[T - 1 - m] / 2);
			auto chk = dsp::exactEqual(expect, r.out);
			CHECK_MESSAGE(chk, "numTaps " << T << " block " << B << ": " << chk.message);
			CHECK(r.guardsOk);
			checked++;
		}
	}
	CHECK(checked > 0);
}

// Step response: x = 0.5: y[m] = sum_{j=T-1-m}^{T-1} c[j] / 2 for m < T, then the sum of all / 2
template <class Tr>
void stepTest(const Var &v) {
	typedef typename Tr::S S;
	size_t checked = 0;
	for (size_t T : Tr::taps()) {
		if (T > 12) continue;
		const size_t B = T + 5, blk = pickBlock(v, B);
		std::vector<S> c(T);
		for (size_t k = 0; k < T; k++) {
			const int64_t sign = (v.neg && (k % 3 == 1)) ? -1 : 1;
			c[k] = static_cast<S>(sign * static_cast<int64_t>(k + 1) * unit<Tr>());
		}
		std::vector<S> x(B, half<Tr>());
		const RunResult<Tr> r = firRun<Tr>(c, x, {blk});
		std::vector<S> expect(B);
		for (size_t m = 0; m < B; m++) {
			int64_t acc = 0;
			const size_t lo = (m < T) ? (T - 1 - m) : 0;
			for (size_t j = lo; j < T; j++) acc += c[j];
			expect[m] = static_cast<S>(acc / 2);  // acc is even: exact
		}
		auto chk = dsp::exactEqual(expect, r.out);
		CHECK_MESSAGE(chk, "numTaps " << T << ": " << chk.message);
		checked++;
	}
	CHECK(checked > 0);
}

// Random data, one call per block size from a zero state, EXACTLY the DEF model
template <class Tr>
void randomVsModelTest(const Var &v) {
	uint64_t seed = 1;
	size_t bad = 0, used = 0;
	std::string first;
	for (size_t T : Tr::taps()) {
		for (size_t B : blocksOf(v)) {
			for (int rep = 0; rep < 8; rep++) {
				Rng rng(seed++);
				const std::vector<typename Tr::S> c = makeCoefs<Tr>(rng, T, v);
				const std::vector<typename Tr::S> x = makeInput<Tr>(rng, B, v);
				const Cmp r = compareToModel<Tr>(c, x, {B}, "taps " + std::to_string(T) + " block " + std::to_string(B));
				CHECK_MESSAGE(r.guardsOk, "taps " << T << " block " << B << ": guard damaged");
				if (r.bad && !bad) first = r.first;
				bad += r.bad;
				used += r.usedSamples;
			}
		}
	}
	CHECK(used > 100);
	CHECK_MESSAGE(bad == 0, bad << " outputs differ from the exact model, first at " << first);
}

// Full range random data (the filter clips): only for the types whose DOC says saturation and whose model clips (q15, fast q15 without
// accumulator overflow).
template <class Tr>
void randomFullRangeTest(const Var &v) {
	typedef typename Tr::S S;
	uint64_t seed = 300;
	size_t bad = 0, used = 0, clipped = 0;
	std::string first;
	for (size_t T : Tr::taps()) {
		if (T > 9) continue;
		for (size_t B : blocksOf(v)) {
			for (int rep = 0; rep < 12; rep++) {
				Rng rng(seed++);
				std::vector<S> c, x;
				if (!Tr::fast && rep % 2 == 0) {  // uniform over the whole range of the type
					c = makeVec<Tr>(rng, T, static_cast<int64_t>(Tr::smin()), static_cast<int64_t>(Tr::smax()));
					x = makeVec<Tr>(rng, B, static_cast<int64_t>(Tr::smin()), static_cast<int64_t>(Tr::smax()));
				} else {
					// constructed: for the fast filter sum |c x| <= T * lim * 2^15 < 2^31 (the 32 bit accumulator cannot wrap) with lim = 65535 / T - 1; coefficients in
					// [lim/2, lim] and inputs of one sign (per block) with magnitude 2^14 .. 2^15 - 1 reach |acc| > 2^30 (clipping)
					const int64_t lim = Tr::fast ? std::min<int64_t>(32767, 65535 / static_cast<int64_t>(T) - 1) : 32767;
					c = makeVec<Tr>(rng, T, lim / 2, lim);
					x = makeVec<Tr>(rng, B, 16384, 32767);
					const int64_t sgn = rng.range(0, 1);
					if (sgn) for (S &e : x) e = static_cast<S>(-e);
				}
				const Cmp r = compareToModel<Tr>(c, x, {B}, "taps " + std::to_string(T) + " block " + std::to_string(B));
				const Def d = defFir<Tr>(c, x);
				for (size_t n = 0; n < B; n++) {
					const i128 y = d.acc[n] >> Tr::bits;
					if (y < Tr::smin() || y > Tr::smax()) clipped++;
				}
				CHECK(r.guardsOk);
				if (r.bad && !bad) first = r.first;
				bad += r.bad;
				used += r.usedSamples;
			}
		}
	}
	CHECK(used > 100);
	CHECK(clipped > 5);  // the data really reach the saturation
	CHECK_MESSAGE(bad == 0, bad << " outputs differ from the exact model, first at " << first);
}

// Floor error one-sided (independent of the model): the standard filters truncate the 2^bits fraction: 0 <= acc - y * 2^bits < 2^bits
// for results that are not clipped. For the fast q31 (product rounding, then 2 * sum): |y * 2^31 - acc| <= T * 2^31: each product is
// rounded to the nearest multiple of 2^32 (error <= 2^31), T products, the sum is doubled in the 2.30 -> 1.31 conversion:
// |2 * accR - acc / 2^31| <= 2 * T * 1/2 = T.
template <class Tr>
void errorBoundTest(const Var &v) {
	uint64_t seed = 700;
	size_t bad = 0, used = 0;
	std::string first;
	for (size_t T : Tr::taps()) {
		for (size_t B : {size_t(9), size_t(8), size_t(3), size_t(3), size_t(3), size_t(3), size_t(3), size_t(3)}) {
			if (B < v.lo || B > v.hi) continue;
			Rng rng(seed++);
			const std::vector<typename Tr::S> c = makeCoefs<Tr>(rng, T, v);
			const std::vector<typename Tr::S> x = makeInput<Tr>(rng, B, v);
			const RunResult<Tr> r = firRun<Tr>(c, x, {B});
			const Def d = defFir<Tr>(c, x);
			for (size_t n = 0; n < B; n++) {
				if (!Tr::usable(d.acc[n], d.accR[n])) continue;
				if (!(Tr::fast && Tr::bits == 31) && ((d.acc[n] >> Tr::bits) < Tr::smin() || (d.acc[n] >> Tr::bits) > Tr::smax())) continue;  // clipped
				const i128 y = r.out[n];
				bool ok;
				if (Tr::fast && Tr::bits == 31) {
					const i128 e = y * (kOne << 31) - d.acc[n];
					ok = (e < 0 ? -e : e) <= static_cast<i128>(T) * (kOne << 31);
				} else {
					const i128 e = d.acc[n] - y * (kOne << Tr::bits);
					ok = e >= 0 && e < (kOne << Tr::bits);
				}
				used++;
				if (!ok) {
					if (bad == 0) first = "taps " + std::to_string(T) + " block " + std::to_string(B) + " n " + std::to_string(n);
					bad++;
				}
			}
		}
	}
	CHECK(used > 40);
	CHECK_MESSAGE(bad == 0, bad << " outputs outside the error bound, first at " << first);
}

// C5: the result agrees with the double computation of the converted input within the resolution of the type: with u = 2^-bits,
// the double sum of (c u)(x u) = acc u^2 is exact in double for q7 / q15 (products of 7 / 15 bit values, sums < 2^53) and has an error of
// at most T * 2^30 * 2^-53 < 2^-17 LSB for q31 (T <= 33; the bound gets 2^-16 LSB slack, negligible against 1 LSB). Standard filters: 0 <= exact - y < 1 LSB (floor),
// fast q31: |exact - y| <= T LSB.
template <class Tr>
void doubleAgreementTest(const Var &v) {
	uint64_t seed = 1100;
	size_t bad = 0, used = 0;
	const double scale = std::ldexp(1.0, Tr::bits);
	for (size_t T : Tr::taps()) {
		const size_t B = v.hi >= 9 ? 9 : 3;
		if (B < v.lo) continue;
		Rng rng(seed++);
		const std::vector<typename Tr::S> c = makeCoefs<Tr>(rng, T, v);
		const std::vector<typename Tr::S> x = makeInput<Tr>(rng, B, v);
		const RunResult<Tr> r = firRun<Tr>(c, x, {B});
		for (size_t n = 0; n < B; n++) {
			double exact = 0.0;  // sum (c/2^b) * (x/2^b), then * 2^b = in units of the output LSB
			bool clip = false;
			for (size_t k = 0; k < T; k++) {
				const long m = static_cast<long>(n) - static_cast<long>(T) + 1 + static_cast<long>(k);
				if (m < 0) continue;
				exact += (static_cast<double>(c[k]) / scale) * (static_cast<double>(x[static_cast<size_t>(m)]) / scale) * scale;
			}
			if (exact >= scale || exact < -scale) clip = true;
			if (clip) continue;
			const double y = static_cast<double>(r.out[n]);
			const double slack = std::ldexp(1.0, -16);
			const bool ok = (Tr::fast && Tr::bits == 31) ? std::fabs(exact - y) <= static_cast<double>(T) + slack
			                                              : (exact - y >= -slack && exact - y < 1.0 + slack);
			used++;
			if (!ok) bad++;
		}
	}
	CHECK(used > 20);
	CHECK(bad == 0);
}

// Saturation and the guard bit (q15 and q7 documented; q31: intermediate sums beyond 2^62)
void q15SaturationTest() {
	// y = sat16(floor(acc / 2^15)); numTaps 2 (the scalar tail loop is used for blocks 1..3).
	// (a) 2 * 32767 * 32767 = 2147352578 -> >> 15 = 65531 -> 32767; negative: 2 * 32767 * -32768 = -2147418112 -> -65534 -> -32768
	{
		const int16_t c[2] = {32767, 32767};
		const std::vector<int16_t> cv(c, c + 2);
		const std::vector<int16_t> xp(6, 32767), xn(6, -32768);
		const RunResult<Q15T> rp = firRun<Q15T>(cv, xp, {3});
		const RunResult<Q15T> rn = firRun<Q15T>(cv, xn, {3});
		// first output: only c[1]*x: 32767*32767 >> 15 = 32766 (floor of 32765.99...: 1073676289 / 32768 = 32766.00003 -> 32766)
		CHECK(rp.out[0] == 32766);
		for (size_t n = 1; n < 6; n++) CHECK(rp.out[n] == 32767);
		// first output: 32767 * -32768 >> 15 = -32767 exactly
		CHECK(rn.out[0] == -32767);
		for (size_t n = 1; n < 6; n++) CHECK(rn.out[n] == -32768);
	}
	// (b) (-1)*(-1): c = {-32768, 0} (c[1] = 0 weights the newest sample, c[0] = -32768 the oldest), x = -32768: the product
	// 2^30 >> 15 = 32768 needs clipping to 32767
	{
		const std::vector<int16_t> cv = {-32768, 0};
		const std::vector<int16_t> x(6, -32768);
		const RunResult<Q15T> r = firRun<Q15T>(cv, x, {3});
		CHECK(r.out[0] == 0);  // history is zero
		for (size_t n = 1; n < 6; n++) CHECK(r.out[n] == 32767);
	}
}

void q15FastSaturationTest() {
	// 32 bit accumulator, 2 * 32767 * 32767 = 2147352578 < 2^31 (no wrap): >> 15 = 65531 -> clipped to 32767; negative -2147418112 -> -32768
	const std::vector<int16_t> cv = {32767, 32767};
	const std::vector<int16_t> xp(6, 32767), xn(6, -32768);
	const RunResult<FQ15T> rp = firRun<FQ15T>(cv, xp, {3});
	const RunResult<FQ15T> rn = firRun<FQ15T>(cv, xn, {3});
	CHECK(rp.out[0] == 32766);
	for (size_t n = 1; n < 6; n++) CHECK(rp.out[n] == 32767);
	CHECK(rn.out[0] == -32767);
	for (size_t n = 1; n < 6; n++) CHECK(rn.out[n] == -32768);
}

// q31 extremes without result overflow: max * max = (2^31-1)^2 = 2^62 - 2^32 + 1 -> >> 31 = 2^31 - 2 = 0x7FFFFFFE (floor);
// min * max = -2^31 (2^31 - 1) = -(2^62 - 2^31) -> >> 31 = -(2^31 - 1) = 0x80000001 (exact, no remainder)
void q31ExtremesTest() {
	const std::vector<int32_t> c1 = {0x7FFFFFFF};
	const RunResult<Q31T> a = firRun<Q31T>(c1, std::vector<int32_t>(4, 0x7FFFFFFF), {4});
	for (size_t n = 0; n < 4; n++) CHECK(a.out[n] == 0x7FFFFFFE);
	const RunResult<Q31T> b = firRun<Q31T>(c1, std::vector<int32_t>(4, INT32_MIN), {4});
	// 0x7FFFFFFF * -2^31 >> 31 = -(2^31 - 1)
	for (size_t n = 0; n < 4; n++) CHECK(b.out[n] == -0x7FFFFFFF);
	const std::vector<int32_t> c2 = {INT32_MIN};
	const RunResult<Q31T> d = firRun<Q31T>(c2, std::vector<int32_t>(4, 0x7FFFFFFF), {4});
	for (size_t n = 0; n < 4; n++) CHECK(d.out[n] == -0x7FFFFFFF);
}

// Wrap of the q31 result. DOC arm_fir_q31.c:53-54: "Thus, if the accumulator result overflows it wraps around rather than clip."; the
// last DOC sentence (:56) says "the 2.62 accumulator is right shifted by 31 bits and saturated to 1.31 format", which contradicts the first
// (NEEDS REVIEW, lead decision: the first sentence is taken as intended). Hand derivation:
//  (a) x = c = -2^31: acc = 2^62, acc >> 31 = 2^31 = 0x80000000 -> (q31_t) wraps to -2^31 = 0x80000000.
//  (b) c = {A, A}, x constant A = 2^31 - 1: n >= 1: acc = 2 A^2 = 2^63 - 2^33 + 2 (fits q63), >> 31 = 2^32 - 4 (floor, the +2 is lost)
//      -> wraps to -4 (0xFFFFFFFC); n = 0 has one product only: A^2 >> 31 = 2^31 - 2 = 0x7FFFFFFE, in range.
void q31WrapTest() {
	const RunResult<Q31T> a = firRun<Q31T>({INT32_MIN}, std::vector<int32_t>(4, INT32_MIN), {4});
	for (size_t n = 0; n < 4; n++) CHECK(a.out[n] == INT32_MIN);
	const RunResult<Q31T> b = firRun<Q31T>({0x7FFFFFFF, 0x7FFFFFFF}, std::vector<int32_t>(6, 0x7FFFFFFF), {3});
	CHECK(b.out[0] == 0x7FFFFFFE);
	for (size_t n = 1; n < 6; n++) CHECK(b.out[n] == -4);
}

// Rounding tie of the fast q31 (characterization: DOC says "truncated to 2.30", the macro multAcc_32x32_keep32_R is SMMLAR and adds 2^31:
// a = (a * 2^32 + c*x + 2^31) >> 32, round half up). c = 2^30, T = 1: c*x = x * 2^30.
//  x = 2:  c*x = 2^31 (a tie), (2^31 + 2^31) >> 32 = 1 (truncation would give 0), y = acc << 1 = 2
//  x = 6:  c*x = 3 * 2^31, (3 * 2^31 + 2^31) >> 32 = 2, y = 4;  x = 10: 5 * 2^31, (5 * 2^31 + 2^31) >> 32 = 3, y = 6
//  x = 1:  c*x = 2^30 (below the tie), (2^30 + 2^31) >> 32 = 0, y = 0;  x = 3: 3 * 2^30: (3*2^30 + 2^31) >> 32 = 1, y = 2 (above the tie)
//  x = -2: c*x = -2^31, (-2^31 + 2^31) >> 32 = 0 (half up rounds toward +infinity), y = 0 (the accumulator stays 0: no negative shift)
void fastQ31RoundingTieTest() {
	const std::vector<int32_t> c = {0x40000000};
	const std::vector<int32_t> x = {2, 6, 10, 1, 3, -2, 2, 6};
	const std::vector<int32_t> expect = {2, 4, 6, 0, 2, 0, 2, 4};
	for (size_t B : {size_t(8), size_t(4), size_t(1)}) {
		const RunResult<FQ31T> r = firRun<FQ31T>(c, x, {B});
		auto chk = dsp::exactEqual(expect, r.out);
		CHECK_MESSAGE(chk, "block " << B << ": " << chk.message);
	}
}

// Guard bit: c = {A, A, -A, -A}, x constant A = 2^31-1: partial sums 2P exceed 2^62 (P = A^2 = 2^62 - 2^32 + 1, 2P = 2^63 - 2^33 + 2 < 2^63 fits
// the 64 bit accumulator) and the total is exactly 0 once all four taps see the constant (n >= 3); n = 0..2 have fewer taps: DEF.
void q31GuardBitTest() {
	const int32_t A = 0x7FFFFFFF;
	const std::vector<int32_t> c = {A, A, -A, -A};
	const std::vector<int32_t> x(12, A);
	for (size_t B : {size_t(12), size_t(5), size_t(1)}) {
		const Cmp r = compareToModel<Q31T>(c, x, {B}, "block " + std::to_string(B));
		CHECK(r.bad == 0);
		CHECK_MESSAGE(r.first.empty(), r.first);
		const RunResult<Q31T> run = firRun<Q31T>(c, x, {B});
		for (size_t n = 3; n < 12; n++) CHECK(run.out[n] == 0);
	}
}

// ---------------------------------------------------------------------------------------------------------------
// C2 / C4
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
void writesExactlyBlockTest(const Var &v) {
	uint64_t seed = 500;
	size_t checked = 0;
	for (size_t T : Tr::taps()) {
		if (T != 1 && T != 2 && T != 4 && T != 5 && T != 6 && T != 9 && T != 10) continue;
		for (size_t B : blocksOf(v)) {
			Rng rng(seed++);
			const std::vector<typename Tr::S> c = makeCoefs<Tr>(rng, T, v);
			const std::vector<typename Tr::S> x = makeInput<Tr>(rng, B, v);
			const RunResult<Tr> r = firRun<Tr>(c, x, {B}, true);
			const Def d = defFir<Tr>(c, x);
			CHECK_MESSAGE(r.guardsOk, "taps " << T << " block " << B << ": guard behind the output or the state damaged");
			size_t unwritten = 0;
			for (size_t n = 0; n < B; n++)
				if (!r.written[n] && !(Tr::usable(d.acc[n], d.accR[n]) && Tr::out(d.acc[n], d.accR[n]) == static_cast<i128>(poisonValue<Tr>()))) unwritten++;
			CHECK_MESSAGE(unwritten == 0, "taps " << T << " block " << B << ": " << unwritten << " output elements not written");
			checked++;
		}
	}
	CHECK(checked > 5);
}

// Consecutive blocks of several sizes carry the state: the result equals the model of the whole stream
template <class Tr>
void continuityTest(const Var &v) {
	const std::vector<std::vector<size_t>> plans = {{1}, {2}, {3}, {1, 2, 3, 2}, {4}, {5}, {7}, {4, 5, 7}, {1, 2, 3, 4, 5, 7},
	                                               {8, 9, 16}, {9, 4, 17, 5}, {40}};
	uint64_t seed = 900;
	size_t bad = 0, plansRun = 0;
	std::string first;
	const std::vector<size_t> taps = Tr::taps();
	for (size_t T : taps) {
		if (T > 17) continue;
		Rng rng(seed++);
		const std::vector<typename Tr::S> c = makeCoefs<Tr>(rng, T, v);
		const std::vector<typename Tr::S> x = makeInput<Tr>(rng, 40, v);
		for (size_t p = 0; p < plans.size(); p++) {
			bool inRange = true;
			for (size_t k : plans[p]) inRange = inRange && k >= v.lo && k <= v.hi;
			if (!inRange) continue;
			const Cmp r = compareToModel<Tr>(c, x, plans[p], "taps " + std::to_string(T) + " plan " + std::to_string(p));
			CHECK(r.guardsOk);
			if (r.bad && !bad) first = r.first;
			bad += r.bad;
			plansRun++;
		}
	}
	CHECK(plansRun > 10);
	CHECK_MESSAGE(bad == 0, bad << " outputs differ from the model of the whole stream, first at " << first);
}

// Two instances with different coefficients, interleaved calls: each equals its own model
template <class Tr>
void independentInstancesTest(const Var &v) {
	typedef typename Tr::S S;
	const size_t Ta = Tr::taps()[2], Tb = Tr::taps()[5];  // q31 / q7: 3 and 6; q15: 6 and 12
	const size_t B = pickBlock(v, 3), blocks = 6;
	Rng rng(31);
	const std::vector<S> ca = makeCoefs<Tr>(rng, Ta, v);
	const std::vector<S> cb = makeCoefs<Tr>(rng, Tb, v);
	const std::vector<S> xa = makeInput<Tr>(rng, B * blocks, v);
	const std::vector<S> xb = makeInput<Tr>(rng, B * blocks, v);
	dsp::GuardedBuffer<S> sa(Tr::procLen(Ta, B)), sb(Tr::procLen(Tb, B));
	typename Tr::Inst ia, ib;
	Tr::init(&ia, static_cast<uint16_t>(Ta), ca.data(), sa.data(), static_cast<uint32_t>(B));
	Tr::init(&ib, static_cast<uint16_t>(Tb), cb.data(), sb.data(), static_cast<uint32_t>(B));
	std::vector<S> oa, ob;
	for (size_t k = 0; k < blocks; k++) {
		std::vector<S> pa(B), pb(B);
		Tr::run(&ia, xa.data() + k * B, pa.data(), static_cast<uint32_t>(B));
		Tr::run(&ib, xb.data() + k * B, pb.data(), static_cast<uint32_t>(B));
		oa.insert(oa.end(), pa.begin(), pa.end());
		ob.insert(ob.end(), pb.begin(), pb.end());
	}
	const Def da = defFir<Tr>(ca, xa), db = defFir<Tr>(cb, xb);
	size_t bad = 0, used = 0;
	for (size_t n = 0; n < oa.size(); n++) {
		if (Tr::usable(da.acc[n], da.accR[n])) {
			used++;
			if (static_cast<i128>(oa[n]) != Tr::out(da.acc[n], da.accR[n])) bad++;
		}
		if (Tr::usable(db.acc[n], db.accR[n])) {
			used++;
			if (static_cast<i128>(ob[n]) != Tr::out(db.acc[n], db.accR[n])) bad++;
		}
	}
	CHECK(used > 20);
	CHECK(bad == 0);
	CHECK(sa.guardIntact());
	CHECK(sb.guardIntact());
}

// Re-init with a poisoned state buffer resets the history: the second run is bit identical to the first
template <class Tr>
void reinitTest(const Var &v) {
	typedef typename Tr::S S;
	const size_t T = Tr::taps()[3], B = pickBlock(v, 5);
	Rng rng(41);
	const std::vector<S> c = makeCoefs<Tr>(rng, T, v);
	const std::vector<S> x = makeInput<Tr>(rng, 3 * B, v);
	dsp::GuardedBuffer<S> state(Tr::procLen(T, B));
	typename Tr::Inst inst;
	Tr::init(&inst, static_cast<uint16_t>(T), c.data(), state.data(), static_cast<uint32_t>(B));
	std::vector<S> first(3 * B), second(3 * B);
	for (size_t k = 0; k < 3; k++) Tr::run(&inst, x.data() + k * B, first.data() + k * B, static_cast<uint32_t>(B));
	// the state now holds the last T-1 inputs: re-init must clear it even if the buffer is full of garbage
	std::memset(static_cast<void *>(state.data()), 0xCD, Tr::initLen(T, B) * sizeof(S));
	Tr::init(&inst, static_cast<uint16_t>(T), c.data(), state.data(), static_cast<uint32_t>(B));
	for (size_t k = 0; k < 3; k++) Tr::run(&inst, x.data() + k * B, second.data() + k * B, static_cast<uint32_t>(B));
	CHECK(std::memcmp(first.data(), second.data(), first.size() * sizeof(S)) == 0);
	CHECK(state.guardIntact());
}

// Init: fields hold the arguments, the state buffer (the documented length) is zero, nothing behind it changes, status success
template <class Tr>
void initTest() {
	typedef typename Tr::S S;
	const size_t cfg[][2] = {{4, 4}, {4, 1}, {2, 9}, {6, 3}, {8, 8}, {12, 17}, {26, 23}, {1, 1}, {1, 9}, {9, 1}, {5, 3}, {25, 23}};
	size_t tested = 0;
	for (const auto &t : cfg) {
		const size_t T = t[0], B = t[1];
		if (Tr::bits == 15 && (T % 2)) continue;  // odd numTaps: rejected / out of the contract (own test)
		Rng rng(61);
		const std::vector<S> c = makeCoefs<Tr>(rng, T, Tr::cleanVar());
		dsp::GuardedBuffer<S> state(Tr::initLen(T, B));  // poisoned with 0xCD by the buffer
		typename Tr::Inst inst;
		std::memset(static_cast<void *>(&inst), 0x5A, sizeof inst);
		const int st = Tr::init(&inst, static_cast<uint16_t>(T), c.data(), state.data(), static_cast<uint32_t>(B));
		CHECK(st == 0);  // ARM_MATH_SUCCESS (q15), void for the others
		CHECK(inst.numTaps == T);
		CHECK(inst.pCoeffs == c.data());
		CHECK(inst.pState == state.data());
		bool zero = true;
		const std::vector<S> sv = state.output();
		for (const S &val : sv) {
			unsigned char raw[sizeof(S)];
			std::memcpy(raw, &val, sizeof(S));
			for (unsigned char byte : raw) zero = zero && byte == 0;
		}
		CHECK_MESSAGE(zero, "taps " << T << " block " << B << ": state not zeroed");
		CHECK_MESSAGE(state.guardIntact(), "taps " << T << " block " << B << ": init wrote behind the state buffer");
		tested++;
	}
	CHECK(tested >= 7);
}

// The coefficients and the input are not modified by the filter
template <class Tr>
void constInputsTest(const Var &v) {
	typedef typename Tr::S S;
	const size_t T = Tr::taps()[4];
	Rng rng(71);
	const std::vector<S> c = makeCoefs<Tr>(rng, T, v);
	const std::vector<S> x = makeInput<Tr>(rng, 23, v);
	std::vector<S> c2(c), x2(x);
	const std::vector<size_t> plan = v.lo >= 4 ? std::vector<size_t>{5, 4, 7} : std::vector<size_t>{3, 1, 2};
	const RunResult<Tr> r = firRun<Tr>(c2, x2, plan);
	CHECK(r.out.size() == 23);
	CHECK(std::memcmp(c.data(), c2.data(), c.size() * sizeof(S)) == 0);
	CHECK(std::memcmp(x.data(), x2.data(), x.size() * sizeof(S)) == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// Sweeps
// ---------------------------------------------------------------------------------------------------------------
// q7, one tap, ALL 256 x 256 (coefficient, input) pairs: y = floor(c * x / 128); the single pair that needs clipping (-128, -128) is
// excluded by usable() (own characterization test).
void q7OneTapExhaustiveTest() {
	size_t bad = 0, used = 0;
	std::vector<int8_t> x(256);
	for (int i = 0; i < 256; i++) x[static_cast<size_t>(i)] = static_cast<int8_t>(i - 128);
	for (int c0 = -128; c0 < 128; c0++) {
		const std::vector<int8_t> c = {static_cast<int8_t>(c0)};
		const RunResult<Q7T> r = firRun<Q7T>(c, x, {256});
		CHECK(r.guardsOk);
		for (int i = 0; i < 256; i++) {
			const int acc = c0 * (i - 128);
			if ((acc >> 7) > 127) continue;
			used++;
			if (r.out[static_cast<size_t>(i)] != (acc >> 7)) bad++;  // acc >> 7: arithmetic shift = floor, the DEF
		}
	}
	CHECK(used == 65536 - 1);
	CHECK(bad == 0);
}

// Two taps over a stream that contains EVERY pair (a, b) of input values at (x[2i], x[2i+1]); the odd outputs then see all pairs. The
// coefficient pairs keep |c0 a + c1 b| < 2^14 (no clipping): |c0| + |c1| <= 127.
template <class Tr>
void twoTapPairStream(const std::vector<std::pair<int, int>> &coefs, size_t block, const Var &v) {
	typedef typename Tr::S S;
	const size_t N = Tr::bits == 7 ? 256 * 256 : 65536;
	std::vector<S> x(2 * N);
	if (Tr::bits == 7) {
		for (size_t i = 0; i < N; i++) {
			x[2 * i] = static_cast<S>(static_cast<int>(i >> 8) - 128);
			x[2 * i + 1] = static_cast<S>(static_cast<int>(i & 255) - 128);
		}
	} else {
		// 16 bit: a_i walks all 65536 values, b_i = a_{i*K} (K odd: a permutation): every value is seen at both positions
		for (size_t i = 0; i < N; i++) {
			x[2 * i] = static_cast<S>(static_cast<int>(i) - 32768);
			x[2 * i + 1] = static_cast<S>(static_cast<int>((i * 40503u) & 0xFFFFu) - 32768);
		}
	}
	(void)v;
	for (const auto &cp : coefs) {
		const std::vector<S> c = {static_cast<S>(cp.first), static_cast<S>(cp.second)};
		const Cmp r = compareToModel<Tr>(c, x, {block}, "coefs " + std::to_string(cp.first) + "," + std::to_string(cp.second));
		CHECK(r.guardsOk);
		CHECK_MESSAGE(r.bad == 0, r.bad << " outputs differ from the model, first " << r.first);
		CHECK(r.usedSamples > x.size() * 9 / 10);
	}
}

// q31: strided sweep of the whole input range, 1 and 2 taps; samples whose result would overflow the q31 range (and for the fast variant the
// 2.30 accumulator) are skipped by usable(); the signed variants include the extreme values
template <class Tr>
void q31SweepTest(const Var &v) {
	typedef int32_t S;
	const uint32_t stride = 40961u;  // odd: ~104,857 values over the 32 bit range
	std::vector<S> x;
	for (uint64_t bits = 0; bits < 0x100000000ull; bits += stride) {
		int32_t s = static_cast<int32_t>(static_cast<uint32_t>(bits));
		if (!v.neg) s &= 0x7FFFFFFF;
		x.push_back(s);
	}
	x.push_back(v.neg ? INT32_MIN : 0);
	x.push_back(INT32_MAX);
	x.push_back(0);
	x.push_back(v.neg ? -1 : 1);
	// (the sets whose sum of |c| reaches 2^31 are skipped for the fast filter, see below; {0x7FFFFFFF} has sum 2^31 - 1 and is kept)
	std::vector<std::vector<S>> coefSets = {{0x7FFFFFFF}, {0x40000000}, {1}, {0x12345678, 0x2345678}, {0x7FFFFFFF, 0x7FFFFFFF}, {0x20000000, 0x20000000}};
	if (v.neg) {
		coefSets.push_back({INT32_MIN});
		coefSets.push_back({-0x12345678});
		coefSets.push_back({0x7FFFFFFF, INT32_MIN});
		coefSets.push_back({-0x20000000, 0x30000000});
	}
	for (const auto &c : coefSets) {
		if (Tr::fast) {
			// the fast filter must not be run on data that overflows its 2.30 accumulator (undefined behaviour of the left shift): the bound
			// |acc| <= sum|c| * 2^31 / 2^32 + T/2 < 2^30 needs sum |c| < 2^31 - T (and c != -2^31: (-2^31)^2 = 2^62 -> acc = 2^30)
			int64_t sumAbs = 0;
			for (S e : c) sumAbs += e < 0 ? -static_cast<int64_t>(e) : static_cast<int64_t>(e);
			if (sumAbs > (int64_t(1) << 31) - 1) continue;
		}
		const Cmp r = compareToModel<Tr>(c, x, {4096}, "coefs " + std::to_string(c[0]));
		CHECK(r.guardsOk);
		CHECK_MESSAGE(r.bad == 0, r.bad << " outputs differ from the model, first " << r.first);
		CHECK(r.usedSamples > x.size() / 4);
	}
}

}  // namespace

// ===============================================================================================================
// Test cases
// ===============================================================================================================
// B-DFIRQ ids (header): the lead renumbers them
#define UB_Q15 KNOWN_ISSUE_UB_HIDDEN("B-DFIRQ-1: dsp/none.h __SMLALD/__SMLAD/__PKHBT and read_q15x2 left shift signed values (unrolled path, blockSize >= 4); B-DFIRQ-3 reads 2 bytes behind numTaps + blockSize - 1 state elements")
#define UB_FQ31 KNOWN_ISSUE_UB_HIDDEN("B-DFIRQ-2: arm_fir_fast_q31.c acc0 << 1 and dsp/none.h multAcc_32x32_keep32_R shift a negative value left")

// Two cases of one function: the clean variant and the variant that reaches the undefined behaviour
#define CASE2(REQ, FN, WHAT, FUNC, TR, TAGC, TAGH, MARK)                                                   \
	TEST_CASE("[" REQ " " FN "] " WHAT TAGC) { FUNC<TR>(TR::cleanVar()); }                                 \
	TEST_CASE("[" REQ " " FN "] " WHAT TAGH * MARK) { FUNC<TR>(TR::hiddenVar()); }
#define CASE2C(REQ, FN, WHAT, FUNC, TR, TAGC, TAGH, MARK)                                                  \
	TEST_CASE("characterization: [" REQ " " FN "] " WHAT TAGC) { FUNC<TR>(TR::cleanVar()); }               \
	TEST_CASE("characterization: [" REQ " " FN "] " WHAT TAGH * MARK) { FUNC<TR>(TR::hiddenVar()); }
#define CASE1(REQ, FN, WHAT, FUNC, TR) TEST_CASE("[" REQ " " FN "] " WHAT) { FUNC<TR>(TR::cleanVar()); }

#if defined(ARM_MATH_DSP)
#define B_DFIRQ_4 doctest::description("odd numTaps rejected (ARM_MATH_DSP build)")
#else
#define B_DFIRQ_4 KNOWN_ISSUE("B-DFIRQ-4: without ARM_MATH_DSP (this host build) arm_fir_init_q15 accepts an odd numTaps and returns ARM_MATH_SUCCESS; DOC @return: ARM_MATH_ARGUMENT_ERROR : numTaps is not greater than or equal to 4 and even; the kernel then drops the last tap")
#endif

#define TAG_B13 ", blocks of 1 to 3 samples"
#define TAG_B4 ", blocks of 4 or more samples"
#define TAG_NONNEG ", non-negative data"
#define TAG_SIGNED ", signed data"

TEST_SUITE("DSP Filtering q31") {

// ---------------------------------------------------------------------------------------------------------------
// arm_fir_q31
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_fir_q31] ARM's pattern test (108 configs, two blocks each) reaches SNR 100 and 2 LSB") { armPatternTest<Q31T>(100.0, 2, 100, true); }  // FIRQ31.cpp:5, :7
TEST_CASE("[C3 arm_fir_q31] ARM's reference pattern equals the difference equation of the DOC within ARM's 2 LSB") { patternIsDefTest<Q31T>(2); }
TEST_CASE("[C3 arm_fir_q31] the impulse response is the coefficients in reverse order times 0.5, for 1 to 12 taps") { impulseTest<Q31T>(Q31T::cleanVar()); }
TEST_CASE("[C3 arm_fir_q31] the step response is the running sum of the coefficients times 0.5") { stepTest<Q31T>(Q31T::cleanVar()); }
TEST_CASE("[C3 arm_fir_q31] random data of every block size 1..17 and 1..33 taps equals the exact model acc >> 31") { randomVsModelTest<Q31T>(Q31T::cleanVar()); }
TEST_CASE("[C3 arm_fir_q31] the output is the floor of the exact sum: 0 <= acc - y * 2^31 < 2^31") { errorBoundTest<Q31T>(Q31T::cleanVar()); }
TEST_CASE("[C5 arm_fir_q31] the result agrees with the double computation of the converted input to one LSB below it") { doubleAgreementTest<Q31T>(Q31T::cleanVar()); }
TEST_CASE("[C3 arm_fir_q31] products of the extreme values that do not overflow give the exact floor results") { q31ExtremesTest(); }
TEST_CASE("[C3 arm_fir_q31] the 64 bit accumulator holds partial sums above 2^62 (one guard bit) and ends exactly at zero") { q31GuardBitTest(); }
TEST_CASE("characterization: [C4 arm_fir_q31] a result above the 1.31 range wraps around (DOC: wraps rather than clip; the DOC sentence 'saturated to 1.31' contradicts it, NEEDS REVIEW)") { q31WrapTest(); }
TEST_CASE("[C2 arm_fir_q31] a call writes exactly blockSize outputs and nothing behind them or behind the exact size state") { writesExactlyBlockTest<Q31T>(Q31T::cleanVar()); }
TEST_CASE("[C4 arm_fir_q31] consecutive blocks of sizes 1, 2, 3, 4, 5, 7 and mixed carry the state like one long block") { continuityTest<Q31T>(Q31T::cleanVar()); }
TEST_CASE("[C4 arm_fir_q31] two instances with different coefficients do not influence each other") { independentInstancesTest<Q31T>(Q31T::cleanVar()); }
TEST_CASE("[C4 arm_fir_q31] init again clears a garbage state: the run repeats bit for bit") { reinitTest<Q31T>(Q31T::cleanVar()); }
TEST_CASE("[C4 arm_fir_init_q31] init stores numTaps, pCoeffs, pState and zeroes exactly numTaps + blockSize - 1 elements") { initTest<Q31T>(); }
TEST_CASE("[C4 arm_fir_q31] the coefficient and input arrays are not modified") { constInputsTest<Q31T>(Q31T::cleanVar()); }
TEST_CASE("[C3 arm_fir_q31] one and two tap filters equal the model over a strided sweep of the whole input range") { q31SweepTest<Q31T>(Q31T::cleanVar()); }

// ---------------------------------------------------------------------------------------------------------------
// arm_fir_fast_q31 (no ARM id: ARM's patterns with a derived bound)
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("characterization: [C3 arm_fir_fast_q31] a product exactly half way between two 2.30 values is rounded up (SMMLAR), not truncated") { fastQ31RoundingTieTest(); }
TEST_CASE("[C3 arm_fir_fast_q31] ARM's FIR pattern is reproduced within numTaps + 3 LSB" * UB_FQ31) { fastPatternTest<FQ31T>(100); }
CASE2C("C3", "arm_fir_fast_q31", "random data of every block size and 1..33 taps equals the exact model 2 * sum(rounded products)", randomVsModelTest, FQ31T, TAG_NONNEG, TAG_SIGNED, UB_FQ31)
CASE2("C3", "arm_fir_fast_q31", "the result is within numTaps LSB of the exact sum", errorBoundTest, FQ31T, TAG_NONNEG, TAG_SIGNED, UB_FQ31)
CASE2("C5", "arm_fir_fast_q31", "the result agrees with the double computation of the converted input within numTaps LSB", doubleAgreementTest, FQ31T, TAG_NONNEG, TAG_SIGNED, UB_FQ31)
CASE2("C3", "arm_fir_fast_q31", "the impulse response is the coefficients in reverse order times 0.5, for 1 to 12 taps", impulseTest, FQ31T, TAG_NONNEG, TAG_SIGNED, UB_FQ31)
CASE2("C3", "arm_fir_fast_q31", "the step response is the running sum of the coefficients times 0.5", stepTest, FQ31T, TAG_NONNEG, TAG_SIGNED, UB_FQ31)
CASE2C("C2", "arm_fir_fast_q31", "a call writes exactly blockSize outputs and nothing behind them or behind the exact size state", writesExactlyBlockTest, FQ31T, TAG_NONNEG, TAG_SIGNED, UB_FQ31)
CASE2C("C4", "arm_fir_fast_q31", "consecutive blocks of mixed sizes carry the state like one long block", continuityTest, FQ31T, TAG_NONNEG, TAG_SIGNED, UB_FQ31)
CASE2C("C4", "arm_fir_fast_q31", "two instances with different coefficients do not influence each other", independentInstancesTest, FQ31T, TAG_NONNEG, TAG_SIGNED, UB_FQ31)
CASE2("C4", "arm_fir_fast_q31", "init again clears a garbage state: the run repeats bit for bit", reinitTest, FQ31T, TAG_NONNEG, TAG_SIGNED, UB_FQ31)
CASE2("C4", "arm_fir_fast_q31", "the coefficient and input arrays are not modified", constInputsTest, FQ31T, TAG_NONNEG, TAG_SIGNED, UB_FQ31)
CASE2C("C3", "arm_fir_fast_q31", "one and two tap filters equal the model over a strided sweep of the input range", q31SweepTest, FQ31T, TAG_NONNEG, TAG_SIGNED, UB_FQ31)

}  // TEST_SUITE

TEST_SUITE("DSP Filtering q15") {

// ---------------------------------------------------------------------------------------------------------------
// arm_fir_q15
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_fir_q15] ARM's configs with blockSize 1 to 3 are within ARM's 2 LSB (the SNR 59 of the full run is not asserted on this subset)") { armPatternTest<Q15T>(59.0, 2, 3, false); }  // subset: SNR not asserted, measured 55.1 dB over these configs only (small signal, quantisation noise of the 2 LSB reference); ARM's 59 dB is asserted by the full run below  // FIRQ15.cpp:7
TEST_CASE("[C1 arm_fir_q15] ARM's pattern test (63 configs, two blocks each) reaches SNR 59 and 2 LSB" * UB_Q15) { armPatternTest<Q15T>(59.0, 2, 100, true); }  // FIRQ15.cpp:5, :7
TEST_CASE("[C3 arm_fir_q15] ARM's reference pattern equals the difference equation of the DOC within ARM's 2 LSB") { patternIsDefTest<Q15T>(2); }
CASE2("C3", "arm_fir_q15", "the impulse response is the coefficients in reverse order times 0.5, for 2 to 12 taps", impulseTest, Q15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C3", "arm_fir_q15", "the step response is the running sum of the coefficients times 0.5", stepTest, Q15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C3", "arm_fir_q15", "random data of every block size and 2..26 taps equals the exact model sat16(acc >> 15)", randomVsModelTest, Q15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C3", "arm_fir_q15", "full range random data saturates to the 16 bit range exactly like the model", randomFullRangeTest, Q15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C3", "arm_fir_q15", "the output is the floor of the exact sum: 0 <= acc - y * 2^15 < 2^15", errorBoundTest, Q15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C5", "arm_fir_q15", "the result agrees with the double computation of the converted input to one LSB below it", doubleAgreementTest, Q15T, TAG_B13, TAG_B4, UB_Q15)
TEST_CASE("[C3 arm_fir_q15] sums above the 16 bit range saturate to 32767 and -32768 and (-1) * (-1) saturates to 32767") { q15SaturationTest(); }
CASE2("C2", "arm_fir_q15", "a call writes exactly blockSize outputs and nothing behind them or behind the state of numTaps + blockSize", writesExactlyBlockTest, Q15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C4", "arm_fir_q15", "consecutive blocks of mixed sizes carry the state like one long block", continuityTest, Q15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C4", "arm_fir_q15", "two instances with different coefficients do not influence each other", independentInstancesTest, Q15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C4", "arm_fir_q15", "init again clears a garbage state: the run repeats bit for bit", reinitTest, Q15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C4", "arm_fir_q15", "the coefficient and input arrays are not modified", constInputsTest, Q15T, TAG_B13, TAG_B4, UB_Q15)
TEST_CASE("[C4 arm_fir_init_q15] init stores numTaps, pCoeffs, pState, returns ARM_MATH_SUCCESS for even numTaps and zeroes the documented number of elements") { initTest<Q15T>(); }

TEST_CASE("[C3 arm_fir_q15] a two tap filter equals the model for every input value at both positions, blocks of 3") {
	twoTapPairStream<Q15T>({{16383, 16383}, {-16384, 16383}, {16383, -16384}, {32767, 0}, {0, -32768}, {1, -1}, {12345, -23456}, {32767, 32767}, {32767, -32768}}, 3, Q15T::cleanVar());
}
TEST_CASE("[C3 arm_fir_q15] a two tap filter equals the model for every input value at both positions, blocks of 64" * UB_Q15) {
	twoTapPairStream<Q15T>({{16383, 16383}, {-16384, 16383}, {16383, -16384}, {32767, 0}, {0, -32768}, {1, -1}, {12345, -23456}, {32767, 32767}, {32767, -32768}}, 64, Q15T::hiddenVar());
}
TEST_CASE("[C3 arm_fir_q15] two coefficients -32768 with inputs -32768 saturate to 32767") {
	// c = {-32768, -32768}, x = -32768: one product 2^30 >> 15 = 32768 -> 32767; two products 2^31 >> 15 = 65536 -> 32767
	const std::vector<int16_t> c = {-32768, -32768}, x(9, -32768);
	const RunResult<Q15T> r = firRun<Q15T>(c, x, {3});
	CHECK(r.out[0] == 32767);  // single history: 2^30 >> 15 = 32768 clipped
	for (size_t n = 1; n < 9; n++) CHECK(r.out[n] == 32767);  // 2^31 >> 15 = 65536 clipped
}

TEST_CASE("[C4 arm_fir_init_q15] an odd numTaps is rejected with ARM_MATH_ARGUMENT_ERROR and the instance is not modified" * B_DFIRQ_4) {
	const std::vector<int16_t> c = {1, 2, 3};
	std::vector<int16_t> state(3 + 4);
	arm_fir_instance_q15 inst;
	std::memset(&inst, 0x5A, sizeof inst);
	arm_fir_instance_q15 before = inst;
	CHECK(arm_fir_init_q15(&inst, 3, c.data(), state.data(), 4) == ARM_MATH_ARGUMENT_ERROR);
	CHECK(std::memcmp(&inst, &before, sizeof inst) == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// arm_fir_fast_q15 (no ARM id)
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C3 arm_fir_fast_q15] ARM's FIR pattern is reproduced within ARM's 2 LSB, configs with blockSize 1 to 3") { fastPatternTest<FQ15T>(3); }
TEST_CASE("[C3 arm_fir_fast_q15] ARM's FIR pattern is reproduced within ARM's 2 LSB, all configs" * UB_Q15) { fastPatternTest<FQ15T>(100); }
CASE2("C3", "arm_fir_fast_q15", "random data of every block size and 2..26 taps equals the exact model sat16(acc >> 15)", randomVsModelTest, FQ15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C3", "arm_fir_fast_q15", "the output is the floor of the exact sum: 0 <= acc - y * 2^15 < 2^15", errorBoundTest, FQ15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C5", "arm_fir_fast_q15", "the result agrees with the double computation of the converted input to one LSB below it", doubleAgreementTest, FQ15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C3", "arm_fir_fast_q15", "the impulse response is the coefficients in reverse order times 0.5, for 2 to 12 taps", impulseTest, FQ15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C3", "arm_fir_fast_q15", "the step response is the running sum of the coefficients times 0.5", stepTest, FQ15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C3", "arm_fir_fast_q15", "full range random data with a non-wrapping accumulator saturates like the model", randomFullRangeTest, FQ15T, TAG_B13, TAG_B4, UB_Q15)
TEST_CASE("[C3 arm_fir_fast_q15] sums above the 16 bit range saturate to 32767 and -32768 while the 32 bit accumulator does not wrap") { q15FastSaturationTest(); }
CASE2("C2", "arm_fir_fast_q15", "a call writes exactly blockSize outputs and nothing behind them or behind the state of numTaps + blockSize", writesExactlyBlockTest, FQ15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C4", "arm_fir_fast_q15", "consecutive blocks of mixed sizes carry the state like one long block", continuityTest, FQ15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C4", "arm_fir_fast_q15", "two instances with different coefficients do not influence each other", independentInstancesTest, FQ15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C4", "arm_fir_fast_q15", "init again clears a garbage state: the run repeats bit for bit", reinitTest, FQ15T, TAG_B13, TAG_B4, UB_Q15)
CASE2("C4", "arm_fir_fast_q15", "the coefficient and input arrays are not modified", constInputsTest, FQ15T, TAG_B13, TAG_B4, UB_Q15)
TEST_CASE("[C3 arm_fir_fast_q15] a two tap filter equals the model for every input value at both positions, blocks of 3") {
	twoTapPairStream<FQ15T>({{16383, 16383}, {-16384, 16383}, {16383, -16384}, {32767, 0}, {0, -32768}, {1, -1}, {12345, -23456}, {32767, 32767}, {32767, -32768}}, 3, FQ15T::cleanVar());
}
TEST_CASE("[C3 arm_fir_fast_q15] a two tap filter equals the model for every input value at both positions, blocks of 64" * UB_Q15) {
	twoTapPairStream<FQ15T>({{16383, 16383}, {-16384, 16383}, {16383, -16384}, {32767, 0}, {0, -32768}, {1, -1}, {12345, -23456}, {32767, 32767}, {32767, -32768}}, 64, FQ15T::hiddenVar());
}

// B-DFIRQ-3: the DOC length of the state buffer for builds without ARM_MATH_DSP is numTaps + blockSize - 1 (this host build); the unrolled kernel
// reads one q15 element behind it. The intended result (no read behind the state) is what the filter returns on the host.
#if !defined(ARM_MATH_DSP)
template <class Tr>
void exactStateTest() {
	Rng rng(9);
	const Var v = Q15T::cleanVar();
	for (size_t T : {size_t(4), size_t(6)}) {
		const std::vector<int16_t> c = makeCoefs<Tr>(rng, T, v);
		const std::vector<int16_t> x = makeInput<Tr>(rng, 8, v);
		dsp::GuardedBuffer<int16_t> state(T + 4 - 1);  // DOC: numTaps + blockSize - 1
		typename Tr::Inst inst;
		Tr::init(&inst, static_cast<uint16_t>(T), c.data(), state.data(), 4);
		std::vector<int16_t> y(8);
		Tr::run(&inst, x.data(), y.data(), 4);
		Tr::run(&inst, x.data() + 4, y.data() + 4, 4);
		const Def d = defFir<Tr>(c, x);
		for (size_t n = 0; n < 8; n++) CHECK(static_cast<i128>(y[n]) == Tr::out(d.acc[n], d.accR[n]));
		CHECK(state.guardIntact());
	}
}
TEST_CASE("[C2 arm_fir_q15] without ARM_MATH_DSP a state buffer of the DOC length numTaps + blockSize - 1 is enough: the unrolled kernel reads nothing behind it"
          * KNOWN_ISSUE_UB_HIDDEN("B-DFIRQ-3: the portable path of arm_fir_q15.c needs numTaps + blockSize state elements (reads 2 bytes behind numTaps + blockSize - 1 for blockSize >= 4) but the init DOC gives numTaps + blockSize - 1 without ARM_MATH_DSP")) { exactStateTest<Q15T>(); }
TEST_CASE("[C2 arm_fir_fast_q15] without ARM_MATH_DSP a state buffer of the DOC length numTaps + blockSize - 1 is enough: the unrolled kernel reads nothing behind it"
          * KNOWN_ISSUE_UB_HIDDEN("B-DFIRQ-3: the portable path of arm_fir_fast_q15.c needs numTaps + blockSize state elements (reads 2 bytes behind numTaps + blockSize - 1 for blockSize >= 4) but the init DOC gives numTaps + blockSize - 1 without ARM_MATH_DSP")) { exactStateTest<FQ15T>(); }
#endif

}  // TEST_SUITE

TEST_SUITE("DSP Filtering q7") {

// ---------------------------------------------------------------------------------------------------------------
// arm_fir_q7
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_fir_q7] ARM's pattern test (70 configs, two blocks each) reaches SNR 10 and 2 LSB") { armPatternTest<Q7T>(10.0, 2, 100, true); }  // FIRQ7.cpp:5, :7
TEST_CASE("[C3 arm_fir_q7] ARM's reference pattern equals the difference equation of the DOC within ARM's 2 LSB") { patternIsDefTest<Q7T>(2); }
TEST_CASE("[C3 arm_fir_q7] the impulse response is the coefficients in reverse order times 0.5, for 1 to 12 taps") { impulseTest<Q7T>(Q7T::cleanVar()); }
TEST_CASE("[C3 arm_fir_q7] the step response is the running sum of the coefficients times 0.5") { stepTest<Q7T>(Q7T::cleanVar()); }
TEST_CASE("[C3 arm_fir_q7] random data of every block size 1..17 and 1..33 taps equals the exact model acc >> 7") { randomVsModelTest<Q7T>(Q7T::cleanVar()); }
TEST_CASE("[C3 arm_fir_q7] the output is the floor of the exact sum: 0 <= acc - y * 2^7 < 2^7") { errorBoundTest<Q7T>(Q7T::cleanVar()); }
TEST_CASE("[C5 arm_fir_q7] the result agrees with the double computation of the converted input to one LSB below it") { doubleAgreementTest<Q7T>(Q7T::cleanVar()); }
TEST_CASE("[C2 arm_fir_q7] a call writes exactly blockSize outputs and nothing behind them or behind the exact size state") { writesExactlyBlockTest<Q7T>(Q7T::cleanVar()); }
TEST_CASE("[C4 arm_fir_q7] consecutive blocks of sizes 1, 2, 3, 4, 5, 7 and mixed carry the state like one long block") { continuityTest<Q7T>(Q7T::cleanVar()); }
TEST_CASE("[C4 arm_fir_q7] two instances with different coefficients do not influence each other") { independentInstancesTest<Q7T>(Q7T::cleanVar()); }
TEST_CASE("[C4 arm_fir_q7] init again clears a garbage state: the run repeats bit for bit") { reinitTest<Q7T>(Q7T::cleanVar()); }
TEST_CASE("[C4 arm_fir_init_q7] init stores numTaps, pCoeffs, pState and zeroes exactly numTaps + blockSize - 1 elements") { initTest<Q7T>(); }
TEST_CASE("[C4 arm_fir_q7] the coefficient and input arrays are not modified") { constInputsTest<Q7T>(Q7T::cleanVar()); }
TEST_CASE("[C3 arm_fir_q7] a one tap filter equals floor(c * x / 128) for all 65535 non-clipping (c, x) pairs") { q7OneTapExhaustiveTest(); }
TEST_CASE("[C3 arm_fir_q7] a two tap filter equals the model for every pair of input values (all 65536 pairs) and 8 coefficient pairs") {
	twoTapPairStream<Q7T>({{63, 63}, {-64, 63}, {63, -64}, {1, -1}, {127, 0}, {0, -128}, {77, -45}, {-50, -77}}, 1021, Q7T::cleanVar());
}

// DOC: "the result is truncated to 1.7 format" ; the code saturates (__SSAT(acc >> 7, 8)). Saturation is what the other q functions do, the DOC wording is
// ambiguous: characterization (listed for the lead).
TEST_CASE("characterization: [C4 arm_fir_q7] a result that does not fit 1.7 is clipped to 127 / -128 (the DOC says truncated), blocks of 1 to 8") {
	// (-128) * (-128) = 16384 >> 7 = 128 -> 127;  2 * 127 * 127 = 32258 >> 7 = 252 -> 127;  2 * 127 * -128 = -32512 >> 7 = -254 -> -128.
	// Block sizes 1, 2, 3 (scalar loop), 4, 8 (unrolled by 4), 5, 7 (unrolled + tail).
	for (size_t B : {size_t(1), size_t(2), size_t(3), size_t(4), size_t(5), size_t(7), size_t(8)}) {
		const RunResult<Q7T> a = firRun<Q7T>({-128}, std::vector<int8_t>(8, -128), {B});
		for (size_t n = 0; n < 8; n++) CHECK_MESSAGE(a.out[n] == 127, "block " << B << " n " << n);
		const RunResult<Q7T> b = firRun<Q7T>({127, 127}, std::vector<int8_t>(8, 127), {B});
		CHECK(b.out[0] == 126);  // one history sample: 127 * 127 = 16129 >> 7 = 126, in range
		for (size_t n = 1; n < 8; n++) CHECK_MESSAGE(b.out[n] == 127, "block " << B << " n " << n);
		const RunResult<Q7T> d = firRun<Q7T>({127, 127}, std::vector<int8_t>(8, -128), {B});
		CHECK(d.out[0] == -127);  // 127 * -128 = -16256 >> 7 = -127, in range
		for (size_t n = 1; n < 8; n++) CHECK_MESSAGE(d.out[n] == -128, "block " << B << " n " << n);
		// positive clip with mixed signs: c = {-128, 127}, x = (-128, 127): -128 * -128 + 127 * 127 = 16384 + 16129 = 32513 >> 7 = 254 -> 127
		const RunResult<Q7T> e = firRun<Q7T>({-128, 127}, std::vector<int8_t>{-128, 127, -128, 127, -128, 127, -128, 127}, {B});
		// n >= 1: c[0] * x[n-1] + c[1] * x[n]: n odd: (-128)(-128) + 127 * 127 = 254 -> 127; n even: (-128)(127) + 127 * (-128) = -32512 -> -254 -> -128
		for (size_t n = 1; n < 8; n++) CHECK_MESSAGE(e.out[n] == ((n % 2) ? 127 : -128), "block " << B << " n " << n);
	}
}

// Saturation of the q7 filter over ALL pairs of input values, for coefficient pairs that clip: the output is SSAT(acc >> 7, 8) of the exact sum
// (characterization: the DOC says "truncated"). Block sizes of the scalar loop (1, 2, 3), the unrolled loop (4, 8) and both (5, 7).
TEST_CASE("characterization: [C3 arm_fir_q7] a two tap filter clips every pair of input values to 8 bits like SSAT(acc >> 7), blocks of 1 to 8") {
	std::vector<int8_t> x(2 * 65536);
	for (size_t i = 0; i < 65536; i++) {
		x[2 * i] = static_cast<int8_t>(static_cast<int>(i >> 8) - 128);
		x[2 * i + 1] = static_cast<int8_t>(static_cast<int>(i & 255) - 128);
	}
	const std::pair<int, int> coefs[] = {{127, 127}, {-128, -128}, {127, -128}, {-128, 127}, {-100, -100}, {0, -128}};
	for (const auto &cp : coefs) {
		const std::vector<int8_t> c = {static_cast<int8_t>(cp.first), static_cast<int8_t>(cp.second)};
		const Def d = defFir<Q7T>(c, x);
		size_t clips = 0;
		for (size_t n = 0; n < x.size(); n++) {
			const i128 y = d.acc[n] >> 7;
			if (y < -128 || y > 127) clips++;
		}
		CHECK(clips >= 256);
		for (size_t B : {size_t(1), size_t(2), size_t(3), size_t(4), size_t(5), size_t(7), size_t(8)}) {
			const RunResult<Q7T> r = firRun<Q7T>(c, x, {B});
			size_t bad = 0;
			for (size_t n = 0; n < x.size(); n++)
				if (static_cast<i128>(r.out[n]) != Q7T::out(d.acc[n], d.accR[n])) bad++;
			CHECK_MESSAGE(bad == 0, "coefs " << cp.first << "," << cp.second << " block " << B << ": " << bad << " outputs differ from SSAT(acc >> 7)");
		}
	}
}

TEST_CASE("characterization: [C3 arm_fir_q7] the one tap filter clips the pair (-128, -128) to 127 in the scalar loop and the unrolled loop") {
	for (size_t B : {size_t(1), size_t(2), size_t(3), size_t(4), size_t(5), size_t(256)}) {
		const std::vector<int8_t> x(256, -128);
		const RunResult<Q7T> r = firRun<Q7T>({-128}, x, {B});
		for (size_t n = 0; n < 256; n++) CHECK_MESSAGE(r.out[n] == 127, "block " << B << " n " << n);  // 16384 >> 7 = 128 -> 127
	}
}

}  // TEST_SUITE
