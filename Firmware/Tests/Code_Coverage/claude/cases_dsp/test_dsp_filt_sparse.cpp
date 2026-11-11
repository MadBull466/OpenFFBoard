/*
 * CMSIS-DSP Filtering, sparse FIR (work order SPAR, phase P11 group G5): arm_fir_sparse_f32 / _q31 / _q15 / _q7 and their init
 * functions arm_fir_sparse_init_f32 / _q31 / _q15 / _q7 (Source/FilteringFunctions/arm_fir_sparse_*.c).
 * Requirement ids: Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * ARM tests (C1): NONE. The suites "MISC F32 / Q31 / Q15 / Q7" of Testing/desc.txt (:2271-3146) and desc_f16.txt cover only
 * levinson_durbin, correlate, conv and conv_partial; no ARM pattern exists for fir_sparse (the group is "@deprecated: no more tested nor
 * maintained", arm_fir_sparse_f32.c:36). So no pattern files are used and every test is DEF (C3) / DOC (C4), derived below.
 *
 * DOC (arm_fir_sparse_f32.c:44-83, arm_fir_sparse_{q31,q15,q7}.c, arm_fir_sparse_init_*.c, filtering_functions.h:2036-2083):
 *   y[n] = b[0]*x[n-pTapDelay[0]] + b[1]*x[n-pTapDelay[1]] + ... + b[numTaps-1]*x[n-pTapDelay[numTaps-1]]
 *   "pCoeffs points to a coefficient array of size numTaps; pTapDelay points to an array of nonzero indices and is also of size numTaps;
 *   pState points to a state array of size maxDelay + blockSize, where maxDelay is the largest offset value that is ever used in the
 *   pTapDelay array. Some of the processing functions also require temporary working buffers." Scratch sizes (the @param lines of each
 *   function): f32 and q31: pScratchIn of blockSize elements; q15 and q7: pScratchIn of blockSize elements (q15_t / q7_t) and pScratchOut
 *   of blockSize q31_t elements. The tests use EXACTLY these sizes, the state exactly maxDelay + blockSize, the coefficient / delay arrays
 *   exactly numTaps and the input block exactly blockSize elements, each as its own tight heap allocation (the sanitizer builds see a
 *   one-element overrun). The init "Sets the values of the internal structure fields. Zeros out the values in the state buffer"
 *   ("size is always maxDelay + blockSize", init DOC); stateIndex is set to 0. The init functions return void and document no error.
 *   The "stateIndex ... Points to the oldest sample in the state buffer" (instance structure): after k calls of blockSize samples
 *   stateIndex = (k * blockSize) mod (maxDelay + blockSize), and the state buffer is the circular store of the last
 *   maxDelay + blockSize inputs, the oldest at stateIndex. The manual initialisation of the DOC ("arm_fir_sparse_instance_f32 S =
 *   {numTaps, 0, pState, pCoeffs, maxDelay, pTapDelay}") is tested too (field order).
 *   Delays: a tap delay d has 0 <= d <= maxDelay (d = 0 is allowed, repeated and unsorted delays are not forbidden by the DOC: a repeated
 *   delay adds the coefficients). x[m] = 0 for m < 0 (zeroed state).
 *   The kernel uses delaySize = maxDelay + blockSize of the CALL as the circular buffer length. A constant block size equal to the init
 *   block size is the documented use (the main tests); a constant block size smaller than the init block size also works (probe by the
 *   reviewer, plain test smallerBlockTest). State continuity is tested with uniform partitions (N calls of B samples; and the same
 *   stream as ONE call of N*B samples with an instance initialised for N*B). Only VARYING the block size between calls of one
 *   instance breaks the circular indices (alternating 3 and 5: wrong results); that is not tested (NEEDS REVIEW 1).
 *   Delay positions: the DOC phrase "pTapDelay points to an array of nonzero indices" is read as the delay positions of the taps (plausible
 *   reading, the DOC is ambiguous: the algorithm formula y[n] = b[k] * x[n - pTapDelay[k]] allows a delay of 0, and every plan here
 *   uses delay 0 for tap 0 in the Asc / Zero / Slack modes and in the hand derived tests; the Random modes put maxDelay on tap 0).
 *
 * Scaling (derived from the code and the DOC; xq = x * 2^b, cq = c * 2^b, b = fraction bits of the type, y = sum c x):
 *   f32: y = sum fl(c x) accumulated in float, in tap order (first the product of tap 0, then "+= product" for tap 1 ...).
 *   q31 (b = 31): per tap  t_k = (q63(c_k) * x[n-d_k]) >> 32 (arithmetic shift = floor(c x / 2^32), the DOC "truncated to 2.30 format"),
 *       accumulated in 2.30 ("single guard bit"), then the whole block is doubled: y = 2 * sum_k floor(c_k x_k / 2^32) (a 1.31 result).
 *       Exact error: sum c x - y*2^31 = sum_k (c_k x_k mod 2^32) in [0, numTaps * 2^32), i.e. y is the floor with a one-sided error
 *       0 <= ideal - y < 2 * numTaps LSB (1 LSB = 2^-31).
 *       Valid while the final sum of the floors is within [-2^30, 2^30 - 1]: then no accumulator overflows and the doubling fits.
 *       DOC: "If the accumulator result overflows, it wraps around rather than saturate": not tested (signed overflow, UB).
 *   q15 (b = 15): scratch q31 accumulator acc = sum c_k x_k (1.15 x 1.15 = 2.30, exact), y = sat16(acc >> 15) (arithmetic shift =
 *       floor(acc / 2^15), "truncated to 2.15 format and then saturated to 1.15"). The accumulator overflow ("wrap") is signed overflow
 *       and not tested: the data keep sum |c x| <= 2^31 - 1 (random data: |c| <= floor(32767 / numTaps), so sum |c x| <= 32767 * 32768).
 *   q7 (b = 7): acc = sum c_k x_k (18.14, exact in 32 bit), y = acc >> 7 (floor) "and finally the result is truncated to 1.7": the code
 *       saturates with __SSAT(.., 8) instead; for results in [-128, 127] both readings agree (plain tests, data chosen with
 *       sum |c| <= 127 so that |y| <= 127), the saturation itself is `characterization:` (NEEDS REVIEW 2).
 *   The DEF is exact integer arithmetic (__int128: GCC / clang / MinGW-w64 on a 64 bit target) for q types, long double for f32 with
 *   the Higham bound (below). Nothing is copied from the library output.
 *
 * f32 bound (Higham, dot product, every output is a sum of numTaps rounded products accumulated in order; holds with and without FMA):
 *   |y - ydef| <= gam(T) * S,   S = sum_k |c_k * x[n-d_k]|,  gam(k) = k u / (1 - k u), u = 2^-24;
 *   plus the error of the DEF itself, gam_ld(T) * S with u_ld = numeric_limits<long double>::epsilon() / 2 (2^-64 on x86, 2^-53 where
 *   long double is double: the bound stays valid). No magic number. Exact (dyadic) data use bound 0.
 *   (Mutation resolution: a gain error below about 2*T*u*S per output is not detected by the random tests; the dyadic exact
 *   tests and the hand derived impulse / step tests are tight.)
 *
 * Host vs target (C6): portable C path (no ARM_MATH_DSP / MVE); ARM_MATH_LOOPUNROLL unrolls the block loops by 4: block sizes
 * 1..9, 16 and 17 are used. __int128 needs a 64 bit host. The target may contract a*b+c into VFMA: covered by the f32 bound; the dyadic
 * tests are exact in both cases. The q31 doubling "*pOut << 1" and the q15 / q7 helper macros: see the UB list below. Struct layout:
 * the manual instance initialisation uses the declared field order (uint16 numTaps, uint16 stateIndex, pointer, pointer, uint16,
 * pointer); layout padding differs between host and target but the aggregate initialisation does not depend on it.
 *
 * Crash found (skipped cases "KNOWN ISSUE (crash)", one per type):
 *   B-DSPAR-4  numTaps = 1 crashes (SIGSEGV): the kernels compute "tapCnt = numTaps - 2U" (arm_fir_sparse_f32.c, _q31.c, _q15.c:173, _q7.c:174)
 *              as an unsigned loop count (0xFFFFFFFF iterations reading beyond pCoeffs / pTapDelay). The DOC states no minimum numTaps.
 *              The skipped cases assert the intended result y[n] = c * x[n - d].
 *
 * Undefined behaviour found by UBSan (--no-skip, one process per case):
 *   B-DSPAR-1  arm_fir_sparse_q31.c:329, :331, :333, :335 (unrolled) and :354 (scalar tail), "in = *pOut << 1": the final doubling shifts a
 *              negative q31 left (every blockSize, any negative output). The plain q31 tests use non-negative data only.
 *   B-DSPAR-2  arm_fir_sparse_q15.c:309 and :318 (unrolled output packing, blockSize >= 4): __PKHBT (dsp/none.h:212) shifts negative
 *              values left ("(int32_t)(ARG1) << 0", "(ARG2) << 16"). blockSize 1..3 (scalar tail) is clean with negative data.
 *   B-DSPAR-3  arm_fir_sparse_q7.c:321 (unrolled output packing, blockSize >= 4): __PACKq7 (dsp/none.h:222) shifts negative values left.
 *              Results are the intended ones on the host. The plain q15 / q7 tests use signed data up to blockSize 3 and non-negative
 *              data above (Var::maxNegB); the "(signed data ...)" cases are KNOWN_ISSUE_UB_HIDDEN (intended result, skipped in sanitizer
 *              builds). No other finding (ASan / UBSan clean for all other cases, run one process per case with --no-skip).
 *
 * NEEDS REVIEW (no test, or characterization only; details in the report):
 *   1. Varying the blockSize between calls of one instance (e.g. alternating 3 and 5): delaySize changes with it and the circular
 *      buffer indices become inconsistent (a block larger than the init block also overruns the state). DOC: init "blockSize =
 *      number of samples that will be processed per block"; not tested. A constant smaller block size is tested.
 *   2. q7: DOC "truncated to 1.7", code saturates (SSAT): `characterization:` test; q31 / q15 accumulator wrap: UB, not tested.
 *   3. numTaps = 0 and blockSize = 0 and a tap delay above maxDelay or negative: not
 *      tested (DOC silent, the kernel would read outside the circular buffer).
 *   4. Instance structure comment (filtering_functions.h:2053, :2066, :2079) says the state array has length maxDelay+blockSize-1;
 *      the init DOC and the init code (memset of maxDelay + blockSize elements) and the kernel (delaySize = maxDelay + blockSize)
 *      use maxDelay + blockSize. The tests use maxDelay + blockSize (documentation inconsistency).
 */
#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math.h"

namespace {

typedef __int128 i128;
const i128 kOne = 1;

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

// data variants: neg = signed data allowed, but only for plans with blockSize <= maxNegB (larger blocks get non-negative data);
// dy = dyadic data for f32 (exact arithmetic).
// The reason for maxNegB (UBSan, see the UB list in the header): q31 shifts a negative sum left in every path (maxNegB 0), q15 and q7
// pack the outputs of the unrolled path (blockSize >= 4) with shifts of negative values (maxNegB 3 for the plain tests; the
// hidden variants use signed data for every block size).
struct Var {
	bool neg;
	bool dy;
	size_t maxNegB;
};
const Var VF{true, false, 1000};     // f32
const Var VDY{true, true, 1000};     // f32 dyadic
const Var VQ31{false, false, 0};     // q31: non-negative data
const Var VQ31S{true, false, 1000};  // q31: signed data (hidden UB)
const Var VQ{true, false, 3};        // q15 / q7: signed data up to blockSize 3, non-negative above
const Var VQNN{false, false, 0};     // q15 / q7: non-negative data
const Var VQS{true, false, 1000};    // q15 / q7: signed data for every block size (hidden UB)

// ---------------------------------------------------------------------------------------------------------------
// Type traits. expect() is the DEF of output sample n of the stream x for the taps (c, d); ok = false when the intended result is
// undefined (accumulator overflow) or ambiguous (q7 above the range: DOC "truncated", code saturates).
// ---------------------------------------------------------------------------------------------------------------
double gam(double k, double u) { return k * u / (1.0 - k * u); }

struct F32T {
	typedef float S;
	typedef float W;  // unused scratch out
	typedef arm_fir_sparse_instance_f32 Inst;
	static constexpr bool needOut = false;
	static const char *fn() { return "arm_fir_sparse_f32"; }
	static void init(Inst *i, uint16_t t, const S *c, S *st, int32_t *d, uint16_t md, uint32_t b) { arm_fir_sparse_init_f32(i, t, c, st, d, md, b); }
	static void run(Inst *i, const S *x, S *y, S *si, W *, uint32_t b) { arm_fir_sparse_f32(i, x, y, si, b); }
	static Inst manual(uint16_t t, S *st, const S *c, uint16_t md, int32_t *d) { Inst i = {t, 0, st, c, md, d}; return i; }
	static long double expect(const std::vector<S> &c, const std::vector<int32_t> &d, const std::vector<S> &x, size_t n, bool &ok,
							  long double &tol, bool exact) {
		long double acc = 0, sabs = 0;
		for (size_t k = 0; k < c.size(); k++) {
			const int64_t m = static_cast<int64_t>(n) - d[k];
			if (m < 0) continue;
			const long double p = static_cast<long double>(c[k]) * static_cast<long double>(x[static_cast<size_t>(m)]);
			acc += p;
			sabs += std::fabs(p);
		}
		const double uld = std::numeric_limits<long double>::epsilon() / 2;
		tol = exact ? 0.0L : static_cast<long double>(gam(static_cast<double>(c.size()), 5.9604644775390625e-8) + gam(static_cast<double>(c.size()), uld)) * sabs;
		ok = true;
		return acc;
	}
	static S gen(Rng &r, const Var &v, double lo, double hi) {
		const uint32_t u = r.next32();
		const double f = lo + (hi - lo) * (static_cast<double>(u >> 8) / 16777216.0);
		return static_cast<S>(v.dy ? std::floor(f * 16.0) / 16.0 : f);
	}
	static S genX(Rng &r, const Var &v, size_t) { return gen(r, v, -1.0, 1.0); }
	static S genC(Rng &r, const Var &v, size_t) { return gen(r, v, -1.0, 1.0); }
};

struct Q31T {
	typedef int32_t S;
	typedef int32_t W;
	typedef arm_fir_sparse_instance_q31 Inst;
	static constexpr bool needOut = false;
	static const char *fn() { return "arm_fir_sparse_q31"; }
	static void init(Inst *i, uint16_t t, const S *c, S *st, int32_t *d, uint16_t md, uint32_t b) { arm_fir_sparse_init_q31(i, t, c, st, d, md, b); }
	static void run(Inst *i, const S *x, S *y, S *si, W *, uint32_t b) { arm_fir_sparse_q31(i, x, y, si, b); }
	static Inst manual(uint16_t t, S *st, const S *c, uint16_t md, int32_t *d) { Inst i = {t, 0, st, c, md, d}; return i; }
	// y = 2 * sum floor(c x / 2^32), valid for a sum of floors in [-2^30, 2^30 - 1]
	static long double expect(const std::vector<S> &c, const std::vector<int32_t> &d, const std::vector<S> &x, size_t n, bool &ok,
							  long double &tol, bool) {
		i128 sf = 0;
		for (size_t k = 0; k < c.size(); k++) {
			const int64_t m = static_cast<int64_t>(n) - d[k];
			if (m < 0) continue;
			sf += (static_cast<i128>(c[k]) * x[static_cast<size_t>(m)]) >> 32;  // floor (arithmetic shift of the __int128)
		}
		ok = sf >= -(kOne << 30) && sf <= (kOne << 30) - 1;
		tol = 0;
		return static_cast<long double>(2 * sf);
	}
	// 0 <= sum c x - y 2^31 < numTaps 2^32 (exact), see the header
	static bool floorErr(const std::vector<S> &c, const std::vector<int32_t> &d, const std::vector<S> &x, size_t n, S y) {
		i128 acc = 0;
		for (size_t k = 0; k < c.size(); k++) {
			const int64_t m = static_cast<int64_t>(n) - d[k];
			if (m >= 0) acc += static_cast<i128>(c[k]) * x[static_cast<size_t>(m)];
		}
		const i128 e = acc - (static_cast<i128>(y) << 31);
		return e >= 0 && e < static_cast<i128>(c.size()) * (kOne << 32);
	}
	static S genX(Rng &r, const Var &v, size_t) { return static_cast<S>(r.range(v.neg ? -(int64_t(1) << 31) : 0, (int64_t(1) << 31) - 1)); }
	static S genC(Rng &r, const Var &v, size_t T) {
		const int64_t cm = (int64_t(1) << 30) / static_cast<int64_t>(T);
		return static_cast<S>(r.range(v.neg ? -cm : 0, cm));
	}
};

struct Q15T {
	typedef int16_t S;
	typedef int32_t W;
	typedef arm_fir_sparse_instance_q15 Inst;
	static constexpr bool needOut = true;
	static const char *fn() { return "arm_fir_sparse_q15"; }
	static void init(Inst *i, uint16_t t, const S *c, S *st, int32_t *d, uint16_t md, uint32_t b) { arm_fir_sparse_init_q15(i, t, c, st, d, md, b); }
	static void run(Inst *i, const S *x, S *y, S *si, W *so, uint32_t b) { arm_fir_sparse_q15(i, x, y, si, so, b); }
	static Inst manual(uint16_t t, S *st, const S *c, uint16_t md, int32_t *d) { Inst i = {t, 0, st, c, md, d}; return i; }
	static long double expect(const std::vector<S> &c, const std::vector<int32_t> &d, const std::vector<S> &x, size_t n, bool &ok,
							  long double &tol, bool) {
		i128 acc = 0, sabs = 0;
		for (size_t k = 0; k < c.size(); k++) {
			const int64_t m = static_cast<int64_t>(n) - d[k];
			if (m < 0) continue;
			const i128 p = static_cast<i128>(c[k]) * x[static_cast<size_t>(m)];
			acc += p;
			sabs += p < 0 ? -p : p;
		}
		ok = sabs <= (kOne << 31) - 1;  // no int32 accumulator overflow
		tol = 0;
		i128 y = acc >> 15;  // floor
		y = y < -32768 ? -32768 : (y > 32767 ? 32767 : y);
		return static_cast<long double>(y);
	}
	// 0 <= acc - y 2^15 < 2^15 where y is not saturated (floor error, exact)
	static bool floorErr(const std::vector<S> &c, const std::vector<int32_t> &d, const std::vector<S> &x, size_t n, S y) {
		i128 acc = 0;
		for (size_t k = 0; k < c.size(); k++) {
			const int64_t m = static_cast<int64_t>(n) - d[k];
			if (m >= 0) acc += static_cast<i128>(c[k]) * x[static_cast<size_t>(m)];
		}
		const i128 e = acc - (static_cast<i128>(y) << 15);
		if (y == 32767 || y == -32768) return true;  // saturated: not a floor of the exact value
		return e >= 0 && e < (kOne << 15);
	}
	static S genX(Rng &r, const Var &v, size_t) { return static_cast<S>(r.range(v.neg ? -32768 : 0, 32767)); }
	static S genC(Rng &r, const Var &v, size_t T) {
		const int64_t cm = 32767 / static_cast<int64_t>(T);
		return static_cast<S>(r.range(v.neg ? -cm : 0, cm));
	}
};

struct Q7T {
	typedef int8_t S;
	typedef int32_t W;
	typedef arm_fir_sparse_instance_q7 Inst;
	static constexpr bool needOut = true;
	static const char *fn() { return "arm_fir_sparse_q7"; }
	static void init(Inst *i, uint16_t t, const S *c, S *st, int32_t *d, uint16_t md, uint32_t b) { arm_fir_sparse_init_q7(i, t, c, st, d, md, b); }
	static void run(Inst *i, const S *x, S *y, S *si, W *so, uint32_t b) { arm_fir_sparse_q7(i, x, y, si, so, b); }
	static Inst manual(uint16_t t, S *st, const S *c, uint16_t md, int32_t *d) { Inst i = {t, 0, st, c, md, d}; return i; }
	// y = floor(acc / 128); ok only for results inside [-128, 127] (the saturation is characterized separately)
	static long double expect(const std::vector<S> &c, const std::vector<int32_t> &d, const std::vector<S> &x, size_t n, bool &ok,
							  long double &tol, bool) {
		i128 acc = 0;
		for (size_t k = 0; k < c.size(); k++) {
			const int64_t m = static_cast<int64_t>(n) - d[k];
			if (m >= 0) acc += static_cast<i128>(c[k]) * x[static_cast<size_t>(m)];
		}
		const i128 y = acc >> 7;
		ok = y >= -128 && y <= 127;
		tol = 0;
		return static_cast<long double>(y);
	}
	static bool floorErr(const std::vector<S> &c, const std::vector<int32_t> &d, const std::vector<S> &x, size_t n, S y) {
		i128 acc = 0;
		for (size_t k = 0; k < c.size(); k++) {
			const int64_t m = static_cast<int64_t>(n) - d[k];
			if (m >= 0) acc += static_cast<i128>(c[k]) * x[static_cast<size_t>(m)];
		}
		const i128 e = acc - (static_cast<i128>(y) << 7);
		return e >= 0 && e < (kOne << 7);
	}
	static S genX(Rng &r, const Var &v, size_t) { return static_cast<S>(r.range(v.neg ? -128 : 0, 127)); }
	static S genC(Rng &r, const Var &v, size_t T) {
		const int64_t cm = std::max<int64_t>(1, 127 / static_cast<int64_t>(T));  // numTaps * cm <= 127: |y| <= 127
		return static_cast<S>(r.range(v.neg ? -cm : 0, cm));
	}
};

// ---------------------------------------------------------------------------------------------------------------
// Rig: instance with tight allocations (state maxDelay + blockSize, scratch exactly the DOC size, arrays of numTaps).
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
struct Rig {
	typedef typename Tr::S S;
	typedef typename Tr::W W;
	size_t T, B;
	uint16_t md;
	std::unique_ptr<S[]> coef, state, sIn;
	std::unique_ptr<int32_t[]> delay;
	std::unique_ptr<W[]> sOut;
	typename Tr::Inst inst;

	Rig(const std::vector<S> &c, const std::vector<int32_t> &d, uint16_t maxDelay, size_t blockSize, bool manual = false)
		: T(c.size()), B(blockSize), md(maxDelay), coef(new S[c.size()]), state(new S[maxDelay + blockSize]), sIn(new S[blockSize]),
		  delay(new int32_t[c.size()]), sOut(new W[Tr::needOut ? blockSize : 1]) {
		std::memcpy(static_cast<void *>(coef.get()), c.data(), T * sizeof(S));
		std::memcpy(static_cast<void *>(delay.get()), d.data(), T * sizeof(int32_t));
		std::memset(static_cast<void *>(&inst), 0xCD, sizeof inst);
		std::memset(static_cast<void *>(state.get()), 0xCD, (md + B) * sizeof(S));  // poison: the init must clear it
		const uint16_t t16 = static_cast<uint16_t>(T);
		if (manual) {
			std::memset(static_cast<void *>(state.get()), 0, (md + B) * sizeof(S));  // DOC: set the state to zero before static init
			inst = Tr::manual(t16, state.get(), coef.get(), md, delay.get());
		} else {
			Tr::init(&inst, t16, coef.get(), state.get(), delay.get(), md, static_cast<uint32_t>(B));
		}
		REQUIRE(inst.numTaps == t16);
		REQUIRE(inst.pCoeffs == coef.get());
		REQUIRE(inst.pTapDelay == delay.get());
		REQUIRE(inst.maxDelay == md);
		REQUIRE(inst.pState == state.get());
		REQUIRE(inst.stateIndex == 0);
	}
	size_t rb = 0;         // block size of the calls (the init block size unless a test changes it)
	bool inputOk = true;   // false once a call modified its (const) input block
	void run(const S *x, S *y) {
		if (rb == 0) rb = B;
		std::unique_ptr<S[]> xb(new S[rb]);  // tight input block: the kernel works on THIS buffer, compared with a copy afterwards
		std::memcpy(static_cast<void *>(xb.get()), x, rb * sizeof(S));
		const std::vector<S> before(xb.get(), xb.get() + rb);
		Tr::run(&inst, xb.get(), y, sIn.get(), sOut.get(), static_cast<uint32_t>(rb));
		if (std::memcmp(before.data(), xb.get(), rb * sizeof(S)) != 0) inputOk = false;
	}
};

template <class Tr>
struct Plan {
	std::vector<typename Tr::S> c, x;
	std::vector<int32_t> d;
	uint16_t maxDelay = 0;
	size_t B = 1, calls = 1;
	bool exact = false;
	bool manual = false;
};

template <class Tr>
struct RunOut {
	std::vector<typename Tr::S> y;
	bool guardOk = true;
	bool inputOk = true;
};

template <class Tr>
RunOut<Tr> runPlan(const Plan<Tr> &p) {
	typedef typename Tr::S S;
	RunOut<Tr> r;
	Rig<Tr> rig(p.c, p.d, p.maxDelay, p.B, p.manual);
	const std::vector<S> &xin = p.x;
	for (size_t j = 0; j < p.calls; j++) {
		dsp::GuardedBuffer<S> out(p.B);
		rig.run(xin.data() + j * p.B, out.data());
		r.guardOk = r.guardOk && out.guardIntact();
		const std::vector<S> o = out.output();
		r.y.insert(r.y.end(), o.begin(), o.end());
	}
	r.inputOk = rig.inputOk;
	return r;
}

// compares the whole stream with the DEF; returns the first difference ("" = fine); skipped counts the samples without a defined result
template <class Tr>
std::string verify(const std::vector<typename Tr::S> &c, const std::vector<int32_t> &d, const std::vector<typename Tr::S> &x,
				   const std::vector<typename Tr::S> &y, bool exact, size_t &skipped) {
	skipped = 0;
	for (size_t n = 0; n < y.size(); n++) {
		bool ok = true;
		long double tol = 0;
		const long double e = Tr::expect(c, d, x, n, ok, tol, exact);
		if (!ok) {
			skipped++;
			continue;
		}
		const long double diff = std::fabs(static_cast<long double>(y[n]) - e);
		if (!(diff <= tol)) {
			return "sample " + std::to_string(n) + ": out " + std::to_string(static_cast<double>(y[n])) + ", DEF " +
				   std::to_string(static_cast<double>(e)) + ", tolerance " + std::to_string(static_cast<double>(tol));
		}
	}
	return "";
}

template <class Tr>
void checkPlan(const Plan<Tr> &p, bool allowSkip = false) {
	const RunOut<Tr> r = runPlan(p);
	size_t skipped = 0;
	const std::string msg = verify<Tr>(p.c, p.d, p.x, r.y, p.exact, skipped);
	CHECK_MESSAGE(msg.empty(), msg);
	CHECK(r.guardOk);
	CHECK(r.inputOk);
	if (!allowSkip) CHECK(skipped == 0);
}

enum class Mode { Asc, Desc, Random, Dup, Zero, Slack };

template <class Tr>
Plan<Tr> makePlan(const Var &vin, Mode mode, size_t T, size_t B, int32_t gap, uint64_t seed) {
	Rng rng(seed);
	Var v = vin;
	v.neg = vin.neg && B <= vin.maxNegB;
	Plan<Tr> p;
	p.B = B;
	p.exact = v.dy;
	int32_t md = static_cast<int32_t>(T - 1) * gap;
	p.d.resize(T);
	for (size_t k = 0; k < T; k++) p.d[k] = static_cast<int32_t>(k) * gap;
	switch (mode) {
	case Mode::Asc:
		break;
	case Mode::Desc:
		std::reverse(p.d.begin(), p.d.end());
		break;
	case Mode::Random:
	case Mode::Dup:
		for (size_t k = 0; k < T; k++) p.d[k] = static_cast<int32_t>(rng.range(0, md));
		p.d[0] = md;  // the largest delay is used
		if (mode == Mode::Dup) p.d[1] = p.d[0];  // a repeated delay
		break;
	case Mode::Zero:
		std::fill(p.d.begin(), p.d.end(), 0);
		md = 0;
		break;
	case Mode::Slack:
		md += 9;  // maxDelay above every used delay
		break;
	}
	p.maxDelay = static_cast<uint16_t>(md);
	const size_t L = static_cast<size_t>(md) + B;
	p.calls = (2 * L) / B + 3;  // the circular buffer wraps at least twice
	p.c.resize(T);
	for (size_t k = 0; k < T; k++) p.c[k] = Tr::genC(rng, v, T);
	p.x.resize(p.calls * B);
	for (size_t n = 0; n < p.x.size(); n++) p.x[n] = Tr::genX(rng, v, n);
	return p;
}

// ---------------------------------------------------------------------------------------------------------------
// Test bodies
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
void modeTest(Mode mode, const Var &v) {
	static const size_t Ts[] = {2, 3, 4, 5, 7, 9};
	static const size_t Bs[] = {1, 2, 3, 4, 5, 8, 9};
	static const int32_t gaps[] = {1, 3};
	uint64_t seed = 1;
	for (size_t T : Ts)
		for (size_t B : Bs)
			for (int32_t g : gaps) {
				INFO("numTaps " << T << ", blockSize " << B << ", gap " << g);
				checkPlan<Tr>(makePlan<Tr>(v, mode, T, B, g, seed++));
			}
}

// numTaps 2 .. 9 against every blockSize 1 .. 9 (loop unroll tails of the block loops)
template <class Tr>
void allSizesTest(const Var &v) {
	uint64_t seed = 500;
	for (size_t T = 2; T <= 9; T++)
		for (size_t B = 1; B <= 9; B++) {
			INFO("numTaps " << T << ", blockSize " << B);
			checkPlan<Tr>(makePlan<Tr>(v, Mode::Random, T, B, 2, seed++));
		}
}

template <class Tr>
void bigBlockTest(const Var &v) {
	uint64_t seed = 900;
	for (size_t B : {16u, 17u, 33u, 64u}) {
		INFO("blockSize " << B);
		checkPlan<Tr>(makePlan<Tr>(v, Mode::Random, 6, B, 5, seed++));
	}
}

template <class Tr>
void explicitTest(const std::vector<typename Tr::S> &c, const std::vector<int32_t> &d, uint16_t md, const std::vector<typename Tr::S> &x,
				  const std::vector<typename Tr::S> &yExpected) {
	Plan<Tr> p;
	p.c = c;
	p.d = d;
	p.maxDelay = md;
	p.B = 3;  // scalar tail loops only (negative outputs of q15 / q7 are clean there)
	p.calls = x.size() / 3;
	p.x = x;
	const RunOut<Tr> r = runPlan(p);
	REQUIRE(r.y.size() == yExpected.size());
	for (size_t n = 0; n < yExpected.size(); n++) {
		INFO("sample " << n);
		CHECK(static_cast<long double>(r.y[n]) == static_cast<long double>(yExpected[n]));
	}
	CHECK(r.guardOk);
}

// one long call of N*B samples equals N calls of B samples (both against the DEF; integer types: equal bit by bit, exact model)
template <class Tr>
void continuityTest(const Var &v) {
	typedef typename Tr::S S;
	uint64_t seed = 2000;
	for (size_t B : {1u, 2u, 3u, 4u, 5u, 7u, 9u}) {
		INFO("blockSize " << B);
		Plan<Tr> p = makePlan<Tr>(v, Mode::Random, 5, B, 3, seed++);
		Plan<Tr> q = p;
		q.B = p.calls * p.B;
		q.calls = 1;
		const RunOut<Tr> a = runPlan(p), b = runPlan(q);
		size_t s1 = 0, s2 = 0;
		const std::string m1 = verify<Tr>(p.c, p.d, p.x, a.y, p.exact, s1);
		const std::string m2 = verify<Tr>(p.c, p.d, p.x, b.y, p.exact, s2);
		CHECK_MESSAGE(m1.empty(), m1);
		CHECK_MESSAGE(m2.empty(), m2);
		CHECK(s1 == 0);
		CHECK(a.guardOk);
		CHECK(b.guardOk);
		if (!std::is_floating_point<S>::value) CHECK(a.y == b.y);
	}
}

// the state buffer is the circular store of the last maxDelay + blockSize inputs, the oldest at stateIndex
template <class Tr>
void stateLayoutTest(const Var &v) {
	typedef typename Tr::S S;
	uint64_t seed = 3000;
	for (size_t B : {1u, 3u, 4u, 5u}) {
		Plan<Tr> p = makePlan<Tr>(v, Mode::Random, 4, B, 3, seed++);
		Rig<Tr> rig(p.c, p.d, p.maxDelay, p.B);
		const size_t L = static_cast<size_t>(p.maxDelay) + B;
		for (size_t j = 0; j < p.calls; j++) {
			dsp::GuardedBuffer<S> out(B);
			rig.run(p.x.data() + j * B, out.data());
			INFO("blockSize " << B << ", after call " << j + 1);
			CHECK(rig.inst.stateIndex == ((j + 1) * B) % L);
		}
		const size_t N = p.calls * B;
		for (size_t i = 0; i < L; i++) {
			const int64_t xi = static_cast<int64_t>(N) - static_cast<int64_t>(L) + static_cast<int64_t>(i);  // the i-th oldest sample
			const S expected = xi >= 0 ? p.x[static_cast<size_t>(xi)] : S(0);
			INFO("blockSize " << B << ", oldest + " << i);
			CHECK(rig.state[(rig.inst.stateIndex + i) % L] == expected);
		}
	}
}

template <class Tr>
void initFieldsTest(const Var &v) {
	typedef typename Tr::S S;
	const size_t T = 4, B = 5;
	const uint16_t md = 7;
	Plan<Tr> p = makePlan<Tr>(v, Mode::Random, T, B, 2, 77);
	std::vector<S> cc = p.c;
	std::vector<int32_t> dd = p.d;
	REQUIRE(p.maxDelay <= md);
	dsp::GuardedBuffer<S> state(static_cast<size_t>(md) + B);  // exact length with a guard behind it
	std::memset(static_cast<void *>(state.data()), 0xCD, (md + B) * sizeof(S));
	typename Tr::Inst inst;
	std::memset(static_cast<void *>(&inst), 0xCD, sizeof inst);
	Tr::init(&inst, static_cast<uint16_t>(T), cc.data(), state.data(), dd.data(), md, B);
	CHECK(inst.numTaps == T);
	CHECK(inst.pCoeffs == cc.data());
	CHECK(inst.pTapDelay == dd.data());
	CHECK(inst.maxDelay == md);
	CHECK(inst.pState == state.data());
	CHECK(inst.stateIndex == 0);
	bool zero = true;
	for (size_t i = 0; i < static_cast<size_t>(md) + B; i++) zero = zero && state.data()[i] == S(0);
	CHECK(zero);
	CHECK(state.guardIntact());
	CHECK(cc == p.c);
	CHECK(dd == p.d);
}

template <class Tr>
void reinitTest(const Var &v) {
	typedef typename Tr::S S;
	Plan<Tr> p = makePlan<Tr>(v, Mode::Random, 5, 4, 3, 88);
	Rig<Tr> rig(p.c, p.d, p.maxDelay, p.B);
	std::vector<S> first, second;
	for (size_t j = 0; j < p.calls; j++) {
		dsp::GuardedBuffer<S> out(p.B);
		rig.run(p.x.data() + j * p.B, out.data());
		const std::vector<S> o = out.output();
		first.insert(first.end(), o.begin(), o.end());
	}
	// re-init the same instance and state buffer after the state is full of data
	Tr::init(&rig.inst, static_cast<uint16_t>(p.c.size()), rig.coef.get(), rig.state.get(), rig.delay.get(), p.maxDelay, static_cast<uint32_t>(p.B));
	CHECK(rig.inst.stateIndex == 0);
	for (size_t j = 0; j < p.calls; j++) {
		dsp::GuardedBuffer<S> out(p.B);
		rig.run(p.x.data() + j * p.B, out.data());
		const std::vector<S> o = out.output();
		second.insert(second.end(), o.begin(), o.end());
	}
	size_t sk = 0;
	const std::string m = verify<Tr>(p.c, p.d, p.x, second, p.exact, sk);
	CHECK_MESSAGE(m.empty(), m);
	CHECK(first == second);  // identical partition and data: bit identical
}

template <class Tr>
void independentTest(const Var &v) {
	typedef typename Tr::S S;
	// two instances with different taps and different data, calls interleaved
	Plan<Tr> a = makePlan<Tr>(v, Mode::Random, 4, 3, 2, 101);
	Plan<Tr> b = makePlan<Tr>(v, Mode::Asc, 6, 3, 1, 202);
	const size_t calls = std::min(a.calls, b.calls);
	Rig<Tr> ra(a.c, a.d, a.maxDelay, a.B), rb(b.c, b.d, b.maxDelay, b.B);
	std::vector<S> ya, yb;
	for (size_t j = 0; j < calls; j++) {
		dsp::GuardedBuffer<S> oa(a.B), ob(b.B);
		ra.run(a.x.data() + j * a.B, oa.data());
		rb.run(b.x.data() + j * b.B, ob.data());
		CHECK(oa.guardIntact());
		CHECK(ob.guardIntact());
		const std::vector<S> va = oa.output(), vb = ob.output();
		ya.insert(ya.end(), va.begin(), va.end());
		yb.insert(yb.end(), vb.begin(), vb.end());
	}
	size_t sk = 0;
	const std::string ma = verify<Tr>(a.c, a.d, a.x, ya, a.exact, sk);
	const std::string mb = verify<Tr>(b.c, b.d, b.x, yb, b.exact, sk);
	CHECK_MESSAGE(ma.empty(), ma);
	CHECK_MESSAGE(mb.empty(), mb);
}

// DOC: "Coefficient and offset arrays may be shared among several instances while state variable arrays cannot be shared"
template <class Tr>
void sharedArraysTest(const Var &v) {
	typedef typename Tr::S S;
	Plan<Tr> a = makePlan<Tr>(v, Mode::Random, 5, 4, 2, 303);
	Plan<Tr> b = makePlan<Tr>(v, Mode::Random, 5, 4, 2, 404);  // different data, same taps below
	b.c = a.c;
	b.d = a.d;
	b.maxDelay = a.maxDelay;
	b.calls = a.calls;
	b.x.resize(b.calls * b.B);
	Rig<Tr> ra(a.c, a.d, a.maxDelay, a.B);
	std::unique_ptr<S[]> stateB(new S[a.maxDelay + a.B]), sIn(new S[a.B]);
	std::unique_ptr<typename Tr::W[]> sOut(new typename Tr::W[Tr::needOut ? a.B : 1]);
	typename Tr::Inst ib;
	Tr::init(&ib, static_cast<uint16_t>(a.c.size()), ra.coef.get(), stateB.get(), ra.delay.get(), a.maxDelay, static_cast<uint32_t>(a.B));
	std::vector<S> ya, yb;
	for (size_t j = 0; j < a.calls; j++) {
		dsp::GuardedBuffer<S> oa(a.B), ob(b.B);
		ra.run(a.x.data() + j * a.B, oa.data());
		std::unique_ptr<S[]> xb(new S[b.B]);
		std::memcpy(static_cast<void *>(xb.get()), b.x.data() + j * b.B, b.B * sizeof(S));
		Tr::run(&ib, xb.get(), ob.data(), sIn.get(), sOut.get(), static_cast<uint32_t>(b.B));
		const std::vector<S> va = oa.output(), vb = ob.output();
		ya.insert(ya.end(), va.begin(), va.end());
		yb.insert(yb.end(), vb.begin(), vb.end());
	}
	size_t sk = 0;
	const std::string ma = verify<Tr>(a.c, a.d, a.x, ya, a.exact, sk);
	const std::string mb = verify<Tr>(b.c, b.d, b.x, yb, b.exact, sk);
	CHECK_MESSAGE(ma.empty(), ma);
	CHECK_MESSAGE(mb.empty(), mb);
}

template <class Tr>
void manualInitTest(const Var &v) {
	Plan<Tr> p = makePlan<Tr>(v, Mode::Random, 5, 4, 3, 505);
	p.manual = true;
	checkPlan<Tr>(p);
}

// the output guard behind the blockSize outputs and the input block stay untouched (every call of checkPlan checks both; this one
// adds an odd block size with a long delay line)
template <class Tr>
void guardTest(const Var &v) {
	for (size_t B : {1u, 2u, 3u, 5u, 7u}) {
		Plan<Tr> p = makePlan<Tr>(v, Mode::Slack, 3, B, 4, 600 + B);
		const RunOut<Tr> r = runPlan(p);
		INFO("blockSize " << B);
		CHECK(r.guardOk);
		CHECK(r.inputOk);
		CHECK(r.y.size() == p.x.size());
	}
}

// floor error, one-sided and independent of the DEF model of the data path (C5: the real number result sum c x scaled to 1.b)
template <class Tr>
void floorErrTest(const Var &v) {
	uint64_t seed = 4000;
	for (size_t T : {2u, 3u, 5u, 9u}) {
		Plan<Tr> p = makePlan<Tr>(v, Mode::Random, T, 3, 2, seed++);
		const RunOut<Tr> r = runPlan(p);
		bool all = true;
		size_t bad = 0;
		for (size_t n = 0; n < r.y.size(); n++)
			if (!Tr::floorErr(p.c, p.d, p.x, n, r.y[n])) {
				all = false;
				bad = n;
				break;
			}
		INFO("numTaps " << T << ", first sample outside the one-sided floor error: " << bad);
		CHECK(all);
	}
}

// ---- f32 only ----
void f32DyadicTest() {
	Var v = VDY;
	uint64_t seed = 7000;
	for (size_t T : {2u, 3u, 6u})
		for (size_t B : {1u, 4u, 5u}) {
			INFO("numTaps " << T << ", blockSize " << B);
			checkPlan<F32T>(makePlan<F32T>(v, Mode::Random, T, B, 2, seed++));
		}
}

// ---- sweeps ----
// q7: every input value and every coefficient (signed: all 256 x 256; non-negative: 128 x 128), one tap non zero. The only product
// outside the 1.7 range is (-128) * (-128) (saturation: characterized separately), so exactly that sample is skipped.
void q7SweepTest(bool sgn, size_t B) {
	Plan<Q7T> p;
	p.d = {0, 3};
	p.maxDelay = 3;
	p.B = B;
	for (int i = sgn ? -128 : 0; i <= 127; i++) p.x.push_back(static_cast<int8_t>(i));
	while (p.x.size() % B) p.x.push_back(0);
	p.calls = p.x.size() / B;
	for (int c0 = sgn ? -128 : 0; c0 <= 127; c0++) {
		p.c = {static_cast<int8_t>(c0), 0};
		const RunOut<Q7T> r = runPlan(p);
		size_t sk = 0;
		const std::string m = verify<Q7T>(p.c, p.d, p.x, r.y, false, sk);
		INFO("coefficient " << c0);
		CHECK_MESSAGE(m.empty(), m);
		CHECK(r.guardOk);
		CHECK(sk == ((sgn && c0 == -128) ? 1u : 0u));
	}
}

// q15: every input value (65536 signed or 32768 non-negative) with coefficients spread over the range (step cstep, incl. -32768
// when signed); the saturating product (-32768)^2 is part of the model (DOC: saturated to 1.15)
void q15SweepTest(bool sgn, size_t B, int cstep) {
	Plan<Q15T> p;
	p.d = {1, 0};
	p.maxDelay = 1;
	p.B = B;
	for (int i = sgn ? -32768 : 0; i <= 32767; i++) p.x.push_back(static_cast<int16_t>(i));
	while (p.x.size() % B) p.x.push_back(0);
	p.calls = p.x.size() / B;
	for (int c0 = sgn ? -32768 : 0; c0 <= 32767; c0 += cstep) {
		p.c = {static_cast<int16_t>(c0), 0};
		const RunOut<Q15T> r = runPlan(p);
		size_t sk = 0;
		const std::string m = verify<Q15T>(p.c, p.d, p.x, r.y, false, sk);
		INFO("coefficient " << c0);
		CHECK_MESSAGE(m.empty(), m);
		CHECK(r.guardOk);
		CHECK(sk == 0);
	}
}

// q31 strided sweep over the 1.31 range (non negative, or signed for the hidden variant), one tap non zero
void q31SweepTest(const Var &v) {
	const int64_t stride = 1048573;  // prime: 4096 values over 2^32
	Plan<Q31T> p;
	p.d = {2, 0};
	p.maxDelay = 2;
	p.B = 64;
	p.calls = 64;  // 4096 samples
	for (size_t i = 0; i < 4096; i++) {
		const int64_t base = v.neg ? -(int64_t(1) << 31) : 0;
		const int64_t span = v.neg ? (int64_t(1) << 32) : (int64_t(1) << 31);
		const int64_t xv = base + (static_cast<int64_t>(i) * stride) % span;
		p.x.push_back(static_cast<int32_t>(xv));
	}
	const int32_t cs[] = {0, 1, 3, 65535, 65536, 0x12345678, 0x40000000, 0x7FFFFFFE, 0x7FFFFFFF};
	size_t skippedTotal = 0;
	for (int32_t c0 : cs) {
		for (int sign = 0; sign < (v.neg ? 2 : 1); sign++) {
			const int32_t cv = sign ? -c0 : c0;
			p.c = {cv, 0};
			const RunOut<Q31T> r = runPlan(p);
			size_t sk = 0;
			const std::string m = verify<Q31T>(p.c, p.d, p.x, r.y, false, sk);
			skippedTotal += sk;
			INFO("coefficient " << cv);
			CHECK_MESSAGE(m.empty(), m);
			CHECK(r.guardOk);
		}
	}
	if (!v.neg) CHECK(skippedTotal == 0);  // one tap, non negative: sum of floors < 2^30 never leaves the range
}

// f32: strided bit pattern sweep of x and c (biased exponent 70..185, so that no product leaves the normal range); one tap, so the
// result is the single rounding of the exact product: bit exact against the DEF in double
void f32SweepTest() {
	Plan<F32T> p;
	p.d = {0, 1};
	p.maxDelay = 1;
	p.B = 64;
	p.calls = 64;
	uint32_t bits = 0x12345u;
	auto next = [&bits](float &out) {
		for (;;) {
			bits += 4093u * 1013u;
			const int e = static_cast<int>((bits >> 23) & 0xFF);
			if (e >= 70 && e <= 185) {
				std::memcpy(&out, &bits, sizeof out);
				return;
			}
		}
	};
	for (size_t i = 0; i < p.B * p.calls; i++) {
		float f = 0;
		next(f);
		p.x.push_back(f);
	}
	for (int t = 0; t < 8; t++) {
		float c = 0;
		next(c);
		p.c = {c, 0.0f};
		const RunOut<F32T> r = runPlan(p);
		for (size_t n = 0; n < r.y.size(); n++) {
			const float expected = static_cast<float>(static_cast<double>(c) * static_cast<double>(p.x[n]));
			INFO("coefficient " << c << ", sample " << n);
			if (!(r.y[n] == expected)) {
				CHECK(r.y[n] == expected);
				break;
			}
		}
		CHECK(r.guardOk);
	}
}

// ---- saturation / boundary ----
// q31: the largest product is (2^31-1)^2 = 2^62 - 2^32 + 1; >> 32 = 2^30 - 1 (floor); doubled = 2^31 - 2
void q31BoundaryTest() {
	Plan<Q31T> p;
	p.c = {2147483647, 0};
	p.d = {0, 1};
	p.maxDelay = 1;
	p.B = 4;
	p.calls = 1;
	p.x = {2147483647, 2147483647, 0, 1};
	// y[0] = 2^31 - 2; y[1] = same (tap 0 only: x[1]); y[2] = 0; y[3] = 2*floor((2^31-1) / 2^32) = 0
	const RunOut<Q31T> r = runPlan(p);
	CHECK(r.y[0] == 2147483646);
	CHECK(r.y[1] == 2147483646);
	CHECK(r.y[2] == 0);
	CHECK(r.y[3] == 0);
	CHECK(r.guardOk);
}

// q15: DOC "truncated to 2.15 format and then saturated to 1.15 format"
//   (-32768) * (-32768) = 2^30; >> 15 = 32768, saturated to 32767.
//   two taps, x = -32768: 2 * (32767 * -32768) = -2^31 + 65536 (fits int32); >> 15 = -65534, saturated to -32768.
//   (-32768) * (-32768) + (-16384) * (-32768) = 2^30 + 2^29 = 1610612736 < 2^31; >> 15 = 49152, saturated to 32767.
void q15SaturationTest() {
	{
		Plan<Q15T> p;
		p.c = {-32768, 0};
		p.d = {0, 1};
		p.maxDelay = 1;
		p.B = 2;
		p.calls = 1;
		p.x = {-32768, -32768};
		const RunOut<Q15T> r = runPlan(p);
		CHECK(r.y[0] == 32767);
		CHECK(r.y[1] == 32767);
		CHECK(r.guardOk);
	}
	{
		Plan<Q15T> p;
		p.c = {32767, 32767};
		p.d = {0, 1};
		p.maxDelay = 1;
		p.B = 2;
		p.calls = 1;
		p.x = {-32768, -32768};
		const RunOut<Q15T> r = runPlan(p);
		CHECK(r.y[0] == -32767);  // one tap: 32767 * -32768 = -1073709056 = -32767 * 2^15
		CHECK(r.y[1] == -32768);
		CHECK(r.guardOk);
	}
	{
		Plan<Q15T> p;
		p.c = {-32768, -16384};
		p.d = {0, 1};
		p.maxDelay = 1;
		p.B = 2;
		p.calls = 1;
		p.x = {-32768, -32768};
		const RunOut<Q15T> r = runPlan(p);
		CHECK(r.y[0] == 32767);  // 2^30 >> 15 = 32768, saturated
		CHECK(r.y[1] == 32767);  // (2^30 + 2^29) >> 15 = 49152, saturated
		CHECK(r.guardOk);
	}
}

// q7: DOC "converted to 18.7 format by discarding the low 7 bits. Finally, the result is truncated to 1.7 format" but the code
// saturates (__SSAT(acc >> 7, 8)): characterization, see NEEDS REVIEW 2.
//   (-128) * (-128) = 2^14; >> 7 = 128 -> 127.   x = -128, c = {127, 127}: acc = -2 * 127 * 128 = -32512; >> 7 = -254 -> -128.
void q7SaturationTest() {
	{
		Plan<Q7T> p;
		p.c = {-128, 0};
		p.d = {0, 1};
		p.maxDelay = 1;
		p.B = 2;
		p.calls = 1;
		p.x = {-128, -128};
		const RunOut<Q7T> r = runPlan(p);
		CHECK(r.y[0] == 127);
		CHECK(r.y[1] == 127);
		CHECK(r.guardOk);
	}
	{
		Plan<Q7T> p;
		p.c = {127, 127};
		p.d = {0, 1};
		p.maxDelay = 1;
		p.B = 2;
		p.calls = 1;
		p.x = {-128, -128};
		const RunOut<Q7T> r = runPlan(p);
		CHECK(r.y[0] == -127);  // one tap: 127 * -128 = -16256; >> 7 = -127
		CHECK(r.y[1] == -128);
		CHECK(r.guardOk);
	}
}

// q7 saturation inside the UNROLLED loop (blockSize 8: every sample passes in1..in4 of __SSAT; outputs are non-negative, so the packing
// shifts of B-DSPAR-3 are not reached): x = 127 at both delays, c = {127, 127}: y[0] = floor(127*127 / 128) = 126 (one tap, x[-1] = 0);
// y[n >= 1] = floor(2*16129 / 128) = 252, saturated to 127 (DOC: "truncated", see NEEDS REVIEW 2).
void q7UnrolledSaturationTest() {
	Plan<Q7T> p;
	p.c = {127, 127};
	p.d = {0, 1};
	p.maxDelay = 1;
	p.B = 8;
	p.calls = 2;
	p.x = std::vector<int8_t>(16, 127);
	const RunOut<Q7T> r = runPlan(p);
	CHECK(r.y[0] == 126);
	for (size_t n = 1; n < 16; n++) {
		INFO("sample " << n);
		CHECK(r.y[n] == 127);
	}
	CHECK(r.guardOk);
}

// a constant block size smaller than the init block size (state of maxDelay + init blockSize, kernel circle of maxDelay + call size)
template <class Tr>
void smallerBlockTest(const Var &v) {
	typedef typename Tr::S S;
	uint64_t seed = 8000;
	for (size_t cb : {1u, 2u, 3u, 5u, 7u}) {
		Plan<Tr> p = makePlan<Tr>(v, Mode::Random, 5, cb, 3, seed++);
		Rig<Tr> rig(p.c, p.d, p.maxDelay, 8);  // init for blockSize 8
		rig.rb = cb;
		std::vector<S> y;
		for (size_t j = 0; j < p.calls; j++) {
			dsp::GuardedBuffer<S> out(cb);
			rig.run(p.x.data() + j * cb, out.data());
			CHECK(out.guardIntact());
			const std::vector<S> o = out.output();
			y.insert(y.end(), o.begin(), o.end());
		}
		size_t sk = 0;
		const std::string m = verify<Tr>(p.c, p.d, p.x, y, p.exact, sk);
		INFO("call blockSize " << cb);
		CHECK_MESSAGE(m.empty(), m);
		CHECK(rig.inputOk);
	}
}

// numTaps 1 (skipped: crash, B-DSPAR-4); intended result y[n] = c * x[n - d]
template <class Tr>
void oneTapTest(const Var &v) {
	checkPlan<Tr>(makePlan<Tr>(v, Mode::Asc, 1, 4, 1, 9100));
}

// ---- hand derived impulse and step responses (delays {0, 2, 5}, maxDelay 5, blockSize 3, 4 calls) ----
// impulse x = (v, 0, 0, ...): y[d_k] = c_k * v (scaled), all other samples 0.
template <class Tr>
std::vector<typename Tr::S> unitVec(size_t n, typename Tr::S first) {
	std::vector<typename Tr::S> v(n, typename Tr::S(0));
	v[0] = first;
	return v;
}
template <class S>
std::vector<S> at(size_t n, S a, S b, S c) {  // zero except samples 0, 2, 5
	std::vector<S> v(n, S(0));
	v[0] = a;
	v[2] = b;
	v[5] = c;
	return v;
}
template <class S>
std::vector<S> stepY(S a, S ab, S abc) {  // 2 x a, 3 x ab, 7 x abc
	std::vector<S> v;
	for (int i = 0; i < 2; i++) v.push_back(a);
	for (int i = 0; i < 3; i++) v.push_back(ab);
	for (int i = 0; i < 7; i++) v.push_back(abc);
	return v;
}

}  // namespace

#define UB_Q31 KNOWN_ISSUE_UB_HIDDEN("B-DSPAR-1: arm_fir_sparse_q31.c:329,331,333,335,354 'in = *pOut << 1' shifts a negative value left (every blockSize)")
#define UB_Q15 KNOWN_ISSUE_UB_HIDDEN("B-DSPAR-2: arm_fir_sparse_q15.c:309,318 (blockSize >= 4) __PKHBT of dsp/none.h:212 shifts negative values left")
#define UB_Q7 KNOWN_ISSUE_UB_HIDDEN("B-DSPAR-3: arm_fir_sparse_q7.c:321 (blockSize >= 4) __PACKq7 of dsp/none.h:222 shifts negative values left")

TEST_SUITE("DSP Filtering f32") {

TEST_CASE("[C3 arm_fir_sparse_f32] impulse response puts each coefficient at its tap delay (c = 0.5, -0.25, 2 at delays 0, 2, 5)") {
	explicitTest<F32T>({0.5f, -0.25f, 2.0f}, {0, 2, 5}, 5, unitVec<F32T>(12, 1.0f), at<float>(12, 0.5f, -0.25f, 2.0f));
}
TEST_CASE("[C3 arm_fir_sparse_f32] step response is the running sum of the coefficients whose delay has elapsed (1, 2, 4 at delays 0, 2, 5)") {
	explicitTest<F32T>({1.0f, 2.0f, 4.0f}, {0, 2, 5}, 5, std::vector<float>(12, 1.0f), stepY<float>(1.0f, 3.0f, 7.0f));
}
TEST_CASE("[C3 arm_fir_sparse_f32] ascending distinct delays match the double DEF (numTaps 2, 3, 4, 5, 7, 9; blockSize 1, 2, 3, 4, 5, 8, 9)") { modeTest<F32T>(Mode::Asc, VF); }
TEST_CASE("[C3 arm_fir_sparse_f32] descending delays (unsorted pTapDelay) match the double DEF") { modeTest<F32T>(Mode::Desc, VF); }
TEST_CASE("[C3 arm_fir_sparse_f32] random delays including the largest maxDelay and delays above the block size match the double DEF") { modeTest<F32T>(Mode::Random, VF); }
TEST_CASE("[C3 arm_fir_sparse_f32] two taps with the same delay add their coefficients") { modeTest<F32T>(Mode::Dup, VF); }
TEST_CASE("[C3 arm_fir_sparse_f32] all delays zero with maxDelay 0 is a gain of the coefficient sum") { modeTest<F32T>(Mode::Zero, VF); }
TEST_CASE("[C3 arm_fir_sparse_f32] a maxDelay above every used delay changes nothing") { modeTest<F32T>(Mode::Slack, VF); }
TEST_CASE("[C4 arm_fir_sparse_f32] numTaps 2..9 with every blockSize 1..9 match the DEF (unroll tails)") { allSizesTest<F32T>(VF); }
TEST_CASE("[C4 arm_fir_sparse_f32] blockSize 16, 17, 33 and 64 match the DEF") { bigBlockTest<F32T>(VF); }
TEST_CASE("[C3 arm_fir_sparse_f32] dyadic data give exactly the DEF (bound 0, fused or not)") { f32DyadicTest(); }
TEST_CASE("[C3 arm_fir_sparse_f32] one tap sweep over strided bit patterns is bit exact against the rounded double product") { f32SweepTest(); }
TEST_CASE("[C4 arm_fir_sparse_f32] one call of N*B samples equals N calls of B samples (blockSize 1, 2, 3, 4, 5, 7, 9)") { continuityTest<F32T>(VF); }
TEST_CASE("[C4 arm_fir_sparse_f32] state index and circular state buffer hold the last maxDelay + blockSize inputs (blockSize 1, 3, 4, 5)") { stateLayoutTest<F32T>(VF); }
TEST_CASE("[C4 arm_fir_sparse_f32] init stores the arguments, clears a poisoned state of exactly maxDelay + blockSize and resets stateIndex") { initFieldsTest<F32T>(VF); }
TEST_CASE("[C4 arm_fir_sparse_f32] re-init resets the state of a used instance") { reinitTest<F32T>(VF); }
TEST_CASE("[C4 arm_fir_sparse_f32] two instances with interleaved calls are independent") { independentTest<F32T>(VF); }
TEST_CASE("[C4 arm_fir_sparse_f32] two instances may share the coefficient and delay arrays") { sharedArraysTest<F32T>(VF); }
TEST_CASE("[C4 arm_fir_sparse_f32] an instance initialised by hand as in the DOC works") { manualInitTest<F32T>(VF); }
TEST_CASE("[C2 arm_fir_sparse_f32] output guard stays intact and the input block is not modified (blockSize 1, 2, 3, 5, 7, tight scratch and state)") { guardTest<F32T>(VF); }
TEST_CASE("[C4 arm_fir_sparse_f32] a constant blockSize smaller than the init blockSize (1, 2, 3, 5, 7 for init 8) equals the DEF") { smallerBlockTest<F32T>(VF); }
TEST_CASE("[C4 arm_fir_sparse_f32] a filter with numTaps 1 gives y[n] = c * x[n - d]" * doctest::description("KNOWN ISSUE (crash): B-DSPAR-4: numTaps 1 gives tapCnt = numTaps - 2U = 0xFFFFFFFF and the kernel reads beyond the arrays (SIGSEGV)") * doctest::skip()) { oneTapTest<F32T>(VF); }

}  // TEST_SUITE

TEST_SUITE("DSP Filtering q31") {

TEST_CASE("[C3 arm_fir_sparse_q31] impulse response puts each coefficient (halved, floor 2.30 then doubled) at its tap delay") {
	// x = 2^30 (0.5): y = 2 * floor(c * 2^30 / 2^32) = 2 * floor(c / 4): c = 2^28 -> 2^27, 2^30 -> 2^29, 2^31 - 1 -> 2 * (2^29 - 1)
	explicitTest<Q31T>({268435456, 1073741824, 2147483647}, {0, 2, 5}, 5, unitVec<Q31T>(12, 1073741824), at<int32_t>(12, 134217728, 536870912, 1073741822));
}
TEST_CASE("[C3 arm_fir_sparse_q31] step response is the running sum of the halved coefficients whose delay has elapsed") {
	// x = 2^29 (0.25): y_k = 2 * floor(c_k / 8): 2^28, 2^27, 2^26
	explicitTest<Q31T>({1073741824, 536870912, 268435456}, {0, 2, 5}, 5, std::vector<int32_t>(12, 536870912),
					   stepY<int32_t>(268435456, 402653184, 469762048));
}
TEST_CASE("[C3 arm_fir_sparse_q31] the largest product gives 2^31 - 2") { q31BoundaryTest(); }
TEST_CASE("[C3 arm_fir_sparse_q31] ascending distinct delays equal the exact model (non negative data)") { modeTest<Q31T>(Mode::Asc, VQ31); }
TEST_CASE("[C3 arm_fir_sparse_q31] ascending distinct delays equal the exact model (signed data)" * UB_Q31) { modeTest<Q31T>(Mode::Asc, VQ31S); }
TEST_CASE("[C3 arm_fir_sparse_q31] descending delays equal the exact model (non negative data)") { modeTest<Q31T>(Mode::Desc, VQ31); }
TEST_CASE("[C3 arm_fir_sparse_q31] random delays equal the exact model (non negative data)") { modeTest<Q31T>(Mode::Random, VQ31); }
TEST_CASE("[C3 arm_fir_sparse_q31] random delays equal the exact model (signed data)" * UB_Q31) { modeTest<Q31T>(Mode::Random, VQ31S); }
TEST_CASE("[C3 arm_fir_sparse_q31] two taps with the same delay add their coefficients (non negative data)") { modeTest<Q31T>(Mode::Dup, VQ31); }
TEST_CASE("[C3 arm_fir_sparse_q31] all delays zero with maxDelay 0 equals the exact model (non negative data)") { modeTest<Q31T>(Mode::Zero, VQ31); }
TEST_CASE("[C3 arm_fir_sparse_q31] a maxDelay above every used delay changes nothing (non negative data)") { modeTest<Q31T>(Mode::Slack, VQ31); }
TEST_CASE("[C4 arm_fir_sparse_q31] numTaps 2..9 with every blockSize 1..9 equal the exact model (non negative data)") { allSizesTest<Q31T>(VQ31); }
TEST_CASE("[C4 arm_fir_sparse_q31] blockSize 16, 17, 33 and 64 equal the exact model (non negative data)") { bigBlockTest<Q31T>(VQ31); }
TEST_CASE("[C5 arm_fir_sparse_q31] result is the floor of the real sum with error in [0, 2 numTaps) LSB (signed data)" * UB_Q31) { floorErrTest<Q31T>(VQ31S); }
TEST_CASE("[C4 arm_fir_sparse_q31] one call of N*B samples equals N calls of B samples (signed data)" * UB_Q31) { continuityTest<Q31T>(VQ31S); }
TEST_CASE("[C3 arm_fir_sparse_q31] one tap strided sweep of the whole non negative 1.31 range equals the exact model") { q31SweepTest(VQ31); }
TEST_CASE("[C3 arm_fir_sparse_q31] one tap strided sweep of the whole signed 1.31 range equals the exact model" * UB_Q31) { q31SweepTest(VQ31S); }
TEST_CASE("[C5 arm_fir_sparse_q31] result is the floor of the real sum with error in [0, 2 numTaps) LSB (non negative data)") { floorErrTest<Q31T>(VQ31); }
TEST_CASE("[C4 arm_fir_sparse_q31] one call of N*B samples equals N calls of B samples (non negative data)") { continuityTest<Q31T>(VQ31); }
TEST_CASE("[C4 arm_fir_sparse_q31] state index and circular state buffer hold the last maxDelay + blockSize inputs") { stateLayoutTest<Q31T>(VQ31); }
TEST_CASE("[C4 arm_fir_sparse_q31] init stores the arguments, clears a poisoned state of exactly maxDelay + blockSize and resets stateIndex") { initFieldsTest<Q31T>(VQ31); }
TEST_CASE("[C4 arm_fir_sparse_q31] re-init resets the state of a used instance (non negative data)") { reinitTest<Q31T>(VQ31); }
TEST_CASE("[C4 arm_fir_sparse_q31] two instances with interleaved calls are independent (non negative data)") { independentTest<Q31T>(VQ31); }
TEST_CASE("[C4 arm_fir_sparse_q31] two instances may share the coefficient and delay arrays (non negative data)") { sharedArraysTest<Q31T>(VQ31); }
TEST_CASE("[C4 arm_fir_sparse_q31] an instance initialised by hand as in the DOC works (non negative data)") { manualInitTest<Q31T>(VQ31); }
TEST_CASE("[C2 arm_fir_sparse_q31] output guard stays intact and the input block is not modified (non negative data, tight scratch and state)") { guardTest<Q31T>(VQ31); }
TEST_CASE("[C4 arm_fir_sparse_q31] a constant blockSize smaller than the init blockSize (1, 2, 3, 5, 7 for init 8) equals the DEF (non negative data)") { smallerBlockTest<Q31T>(VQ31); }
TEST_CASE("[C4 arm_fir_sparse_q31] a filter with numTaps 1 gives y[n] = c * x[n - d]" * doctest::description("KNOWN ISSUE (crash): B-DSPAR-4: numTaps 1 gives tapCnt = numTaps - 2U = 0xFFFFFFFF and the kernel reads beyond the arrays (SIGSEGV)") * doctest::skip()) { oneTapTest<Q31T>(VQ31); }

}  // TEST_SUITE

TEST_SUITE("DSP Filtering q15") {

TEST_CASE("[C3 arm_fir_sparse_q15] impulse response puts each coefficient (halved, floor) at its tap delay") {
	// x = 16384 (0.5): y = floor(c * 16384 / 2^15) = floor(c / 2): 8192 -> 4096, -16384 -> -8192, 32767 -> 16383
	explicitTest<Q15T>({8192, -16384, 32767}, {0, 2, 5}, 5, unitVec<Q15T>(12, 16384), at<int16_t>(12, 4096, -8192, 16383));
}
TEST_CASE("[C3 arm_fir_sparse_q15] step response is the running sum of the quartered coefficients whose delay has elapsed") {
	// x = 8192 (0.25): y_k = c_k / 4: 4096, 2048, -1024
	explicitTest<Q15T>({16384, 8192, -4096}, {0, 2, 5}, 5, std::vector<int16_t>(12, 8192), stepY<int16_t>(4096, 6144, 5120));
}
TEST_CASE("[C3 arm_fir_sparse_q15] results saturate to 1.15 in both directions") { q15SaturationTest(); }
TEST_CASE("[C3 arm_fir_sparse_q15] ascending distinct delays equal the exact model (signed data for blockSize <= 3, non negative above)") { modeTest<Q15T>(Mode::Asc, VQ); }
TEST_CASE("[C3 arm_fir_sparse_q15] descending delays equal the exact model (signed data for blockSize <= 3, non negative above)") { modeTest<Q15T>(Mode::Desc, VQ); }
TEST_CASE("[C3 arm_fir_sparse_q15] random delays equal the exact model (signed data for blockSize <= 3, non negative above)") { modeTest<Q15T>(Mode::Random, VQ); }
TEST_CASE("[C3 arm_fir_sparse_q15] two taps with the same delay add their coefficients (signed data for blockSize <= 3, non negative above)") { modeTest<Q15T>(Mode::Dup, VQ); }
TEST_CASE("[C3 arm_fir_sparse_q15] all delays zero with maxDelay 0 equals the exact model (signed data for blockSize <= 3, non negative above)") { modeTest<Q15T>(Mode::Zero, VQ); }
TEST_CASE("[C3 arm_fir_sparse_q15] a maxDelay above every used delay changes nothing (signed data for blockSize <= 3, non negative above)") { modeTest<Q15T>(Mode::Slack, VQ); }
TEST_CASE("[C4 arm_fir_sparse_q15] numTaps 2..9 with every blockSize 1..9 equal the exact model (signed data for blockSize <= 3, non negative above)") { allSizesTest<Q15T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q15] blockSize 16, 17, 33 and 64 equal the exact model (non negative data)") { bigBlockTest<Q15T>(VQ); }
TEST_CASE("[C3 arm_fir_sparse_q15] one tap sweep of all 65536 inputs with 6 coefficients equals the exact model (blockSize 3, signed)") { q15SweepTest(true, 3, 10923); }
TEST_CASE("[C3 arm_fir_sparse_q15] one tap sweep of all 32768 non negative inputs with 24 coefficients equals the exact model (blockSize 4096)") { q15SweepTest(false, 4096, 1366); }
TEST_CASE("[C3 arm_fir_sparse_q15] one tap sweep of all 65536 inputs with 24 coefficients equals the exact model (blockSize 4096, signed)" * UB_Q15) { q15SweepTest(true, 4096, 2731); }
TEST_CASE("[C3 arm_fir_sparse_q15] random delays equal the exact model (signed data, every blockSize)" * UB_Q15) { modeTest<Q15T>(Mode::Random, VQS); }
TEST_CASE("[C4 arm_fir_sparse_q15] numTaps 2..9 with every blockSize 1..9 equal the exact model (signed data)" * UB_Q15) { allSizesTest<Q15T>(VQS); }
TEST_CASE("[C4 arm_fir_sparse_q15] blockSize 16, 17, 33 and 64 equal the exact model (signed data)" * UB_Q15) { bigBlockTest<Q15T>(VQS); }
TEST_CASE("[C4 arm_fir_sparse_q15] one call of N*B samples equals N calls of B samples (signed data)" * UB_Q15) { continuityTest<Q15T>(VQS); }
TEST_CASE("[C5 arm_fir_sparse_q15] result is the floor of the real sum with error in [0, 1) LSB (signed data, blockSize 3)") { floorErrTest<Q15T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q15] one call of N*B samples equals N calls of B samples (non negative data)") { continuityTest<Q15T>(VQNN); }
TEST_CASE("[C4 arm_fir_sparse_q15] state index and circular state buffer hold the last maxDelay + blockSize inputs") { stateLayoutTest<Q15T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q15] init stores the arguments, clears a poisoned state of exactly maxDelay + blockSize and resets stateIndex") { initFieldsTest<Q15T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q15] re-init resets the state of a used instance") { reinitTest<Q15T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q15] two instances with interleaved calls are independent") { independentTest<Q15T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q15] two instances may share the coefficient and delay arrays") { sharedArraysTest<Q15T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q15] an instance initialised by hand as in the DOC works") { manualInitTest<Q15T>(VQ); }
TEST_CASE("[C2 arm_fir_sparse_q15] output guard stays intact and the input block is not modified (tight scratch buffers and state)") { guardTest<Q15T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q15] a constant blockSize smaller than the init blockSize (1, 2, 3, 5, 7 for init 8) equals the DEF") { smallerBlockTest<Q15T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q15] a filter with numTaps 1 gives y[n] = c * x[n - d]" * doctest::description("KNOWN ISSUE (crash): B-DSPAR-4: numTaps 1 gives tapCnt = numTaps - 2U = 0xFFFFFFFF and the kernel reads beyond the arrays (SIGSEGV)") * doctest::skip()) { oneTapTest<Q15T>(VQ); }

}  // TEST_SUITE

TEST_SUITE("DSP Filtering q7") {

TEST_CASE("[C3 arm_fir_sparse_q7] impulse response puts each coefficient (halved, floor) at its tap delay") {
	// x = 64 (0.5): y = floor(c * 64 / 128) = floor(c / 2): 32 -> 16, -64 -> -32, 127 -> 63
	explicitTest<Q7T>({32, -64, 127}, {0, 2, 5}, 5, unitVec<Q7T>(12, 64), at<int8_t>(12, 16, -32, 63));
}
TEST_CASE("[C3 arm_fir_sparse_q7] step response is the running sum of the quartered coefficients whose delay has elapsed") {
	// x = 32 (0.25): y_k = c_k / 4: 16, 8, -4
	explicitTest<Q7T>({64, 32, -16}, {0, 2, 5}, 5, std::vector<int8_t>(12, 32), stepY<int8_t>(16, 24, 20));
}
TEST_CASE("characterization: [C4 arm_fir_sparse_q7] results beyond the 1.7 range saturate (DOC says truncated)") { q7SaturationTest(); }
TEST_CASE("characterization: [C4 arm_fir_sparse_q7] results beyond the 1.7 range saturate in the unrolled loop (blockSize 8, non negative outputs)") { q7UnrolledSaturationTest(); }
TEST_CASE("[C3 arm_fir_sparse_q7] ascending distinct delays equal the exact model (signed data for blockSize <= 3, non negative above)") { modeTest<Q7T>(Mode::Asc, VQ); }
TEST_CASE("[C3 arm_fir_sparse_q7] descending delays equal the exact model (signed data for blockSize <= 3, non negative above)") { modeTest<Q7T>(Mode::Desc, VQ); }
TEST_CASE("[C3 arm_fir_sparse_q7] random delays equal the exact model (signed data for blockSize <= 3, non negative above)") { modeTest<Q7T>(Mode::Random, VQ); }
TEST_CASE("[C3 arm_fir_sparse_q7] two taps with the same delay add their coefficients (signed data for blockSize <= 3, non negative above)") { modeTest<Q7T>(Mode::Dup, VQ); }
TEST_CASE("[C3 arm_fir_sparse_q7] all delays zero with maxDelay 0 equals the exact model (signed data for blockSize <= 3, non negative above)") { modeTest<Q7T>(Mode::Zero, VQ); }
TEST_CASE("[C3 arm_fir_sparse_q7] a maxDelay above every used delay changes nothing (signed data for blockSize <= 3, non negative above)") { modeTest<Q7T>(Mode::Slack, VQ); }
TEST_CASE("[C4 arm_fir_sparse_q7] numTaps 2..9 with every blockSize 1..9 equal the exact model (signed data for blockSize <= 3, non negative above)") { allSizesTest<Q7T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q7] blockSize 16, 17, 33 and 64 equal the exact model (non negative data)") { bigBlockTest<Q7T>(VQ); }
TEST_CASE("[C3 arm_fir_sparse_q7] one tap sweep of all 256 inputs with all 256 coefficients equals the exact model (blockSize 3, signed)") { q7SweepTest(true, 3); }
TEST_CASE("[C3 arm_fir_sparse_q7] one tap sweep of all 128 non negative inputs with all 128 non negative coefficients equals the exact model (blockSize 16)") { q7SweepTest(false, 16); }
TEST_CASE("[C3 arm_fir_sparse_q7] one tap sweep of all 256 inputs with all 256 coefficients equals the exact model (blockSize 16, signed)" * UB_Q7) { q7SweepTest(true, 16); }
TEST_CASE("[C3 arm_fir_sparse_q7] random delays equal the exact model (signed data, every blockSize)" * UB_Q7) { modeTest<Q7T>(Mode::Random, VQS); }
TEST_CASE("[C4 arm_fir_sparse_q7] numTaps 2..9 with every blockSize 1..9 equal the exact model (signed data)" * UB_Q7) { allSizesTest<Q7T>(VQS); }
TEST_CASE("[C4 arm_fir_sparse_q7] blockSize 16, 17, 33 and 64 equal the exact model (signed data)" * UB_Q7) { bigBlockTest<Q7T>(VQS); }
TEST_CASE("[C4 arm_fir_sparse_q7] one call of N*B samples equals N calls of B samples (signed data)" * UB_Q7) { continuityTest<Q7T>(VQS); }
TEST_CASE("[C5 arm_fir_sparse_q7] result is the floor of the real sum with error in [0, 1) LSB (signed data, blockSize 3)") { floorErrTest<Q7T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q7] one call of N*B samples equals N calls of B samples (non negative data)") { continuityTest<Q7T>(VQNN); }
TEST_CASE("[C4 arm_fir_sparse_q7] state index and circular state buffer hold the last maxDelay + blockSize inputs") { stateLayoutTest<Q7T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q7] init stores the arguments, clears a poisoned state of exactly maxDelay + blockSize and resets stateIndex") { initFieldsTest<Q7T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q7] re-init resets the state of a used instance") { reinitTest<Q7T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q7] two instances with interleaved calls are independent") { independentTest<Q7T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q7] two instances may share the coefficient and delay arrays") { sharedArraysTest<Q7T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q7] an instance initialised by hand as in the DOC works") { manualInitTest<Q7T>(VQ); }
TEST_CASE("[C2 arm_fir_sparse_q7] output guard stays intact and the input block is not modified (tight scratch buffers and state)") { guardTest<Q7T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q7] a constant blockSize smaller than the init blockSize (1, 2, 3, 5, 7 for init 8) equals the DEF") { smallerBlockTest<Q7T>(VQ); }
TEST_CASE("[C4 arm_fir_sparse_q7] a filter with numTaps 1 gives y[n] = c * x[n - d]" * doctest::description("KNOWN ISSUE (crash): B-DSPAR-4: numTaps 1 gives tapCnt = numTaps - 2U = 0xFFFFFFFF and the kernel reads beyond the arrays (SIGSEGV)") * doctest::skip()) { oneTapTest<Q7T>(VQ); }

}  // TEST_SUITE
