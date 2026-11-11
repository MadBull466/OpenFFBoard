/*
 * CMSIS-DSP Filtering, q31 convolution / correlation / Levinson-Durbin (work order CVQ31, phase P11 group G5):
 * arm_conv_q31, arm_conv_fast_q31, arm_conv_partial_q31, arm_conv_partial_fast_q31, arm_correlate_q31, arm_correlate_fast_q31,
 * arm_levinson_durbin_q31. Requirement ids: Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * Pattern files: Libraries/CMSIS-DSP/Testing/Patterns/DSP/Filtering/MISC/MISCQ31/, suite "MISC Q31" (Testing/desc.txt:2673-2790 and the
 * Functions list up to :2830), ARM test functions Testing/Source/Tests/MISCQ31.cpp. ARM ids (60 + 6 + 3 test cases):
 *   arm_correlate_q31   nba {4,5,6,9,10,11,12,13} x nbb {1,2,3,8,11}   (REF1..REF40, InputsA1/B1, MISCQ31.cpp:48-61, setUp :138-456)
 *   arm_conv_q31        the same 40 pairs                              (REF41..REF80, InputsA1/B1, :63-77)
 *   arm_levinson_durbin_q31 n = 3, 8, 11 (InputPhi81/82/83, LDErrors81, REF81..REF83, :28-46, setUp :778-809)
 *   arm_conv_partial_q31 / arm_conv_partial_fast_q31 nba = 6, nbb = 8, first = 3, 9, 7, numPoints = 4 (InputsA2/B2, REF84..REF86, :83-127)
 * ARM has NO id for arm_conv_fast_q31 and arm_correlate_fast_q31 (desc.txt lists only the ones above); nothing is disabled or commented out
 * in this suite (the "disabled" ids of the charter belong to the other types).
 * ARM tolerances (used unchanged, L4): SNR_THRESHOLD 100 (MISCQ31.cpp:8), ABS_ERROR_Q31 2 (:15), ABS_ERROR_FAST_Q31 11 (:17, partial fast),
 * ABS_LD_ERROR 30 (:25, Levinson-Durbin coefficients AND error). ASSERT_NEAR_EQ = |out - ref| <= bound per sample (withinLsb), ASSERT_SNR =
 * dsp::snr. ARM's checks ASSERT_EMPTY_TAIL (conv, levinson) are replaced by a guard behind every output (GuardedBuffer). ARM's partial tests
 * copy the window out of a buffer that holds first + numPoints values; here the buffer has exactly first + numPoints elements.
 *
 * DOC (arm_conv_q31.c, arm_correlate_q31.c, arm_conv_partial_q31.c, arm_levinson_durbin_q31.c):
 *   conv:      y[n] = sum_k a[k] b[n-k], length srcALen + srcBLen - 1; the function swaps the arguments if srcALen < srcBLen (no precondition).
 *   correlate: "Length 2 * max(srcALen, srcBLen) - 1". Exact layout, derived from the stages of the code and the comments in it (hand
 *              derivation, examples in the tests): r[n] = sum_k a[k+n] b[k] for the lags n = -(srcBLen-1) .. srcALen-1 (zero outside),
 *              out[p] = r[p - (M-1)], M = max(srcALen, srcBLen), p = 0 .. 2M-2. The nonzero lags are p = M-srcBLen .. M+srcALen-2: for
 *              srcALen >= srcBLen the first srcALen-srcBLen outputs are padding, for srcALen < srcBLen the last srcBLen-srcALen are.
 *              The source comments say "zeroes ... included in the output" but the code never WRITES the padding (it only moves the output
 *              pointer); the f32 DOC says "pDst should be initialized to all zeros", the q31 DOC does not. ARM's output buffers are zero.
 *              The tests below zero the output first (the f32 rule); what the function does to the padding is characterization.
 *   q31 std:   64 bit accumulator 2.62, "no saturation on intermediate additions ... wraps", result = acc >> 31 (arithmetic shift, floor);
 *              "right shifted by 31 bits and saturated to 1.31 format" (DOC, same function: see NEEDS REVIEW 1).
 *   q31 fast:  "The result of each 1.31 x 1.31 multiplication is truncated to 2.30 format": the code is sum += floor(a*b / 2^32) per product
 *              (sum = (q31)(((q63)sum << 32) + a*b >> 32), exact: sum << 32 is a multiple of 2^32) and the output is sum << 1. This is the
 *              DOC ("truncated" = floor), so the fast model is a plain DEF here (unlike arm_fir_fast_q31, which rounds with SMMLAR).
 *              Floor error of one product: 0 <= a*b/2^32 - floor < 1, so for L = min(srcALen, srcBLen) terms: 0 <= acc/2^31 - y < 2 L.
 *              The accumulator is 2.30 in 32 bits: "the accumulator is saturated" - the code does not saturate (sum << 1 wraps / is undefined).
 *   partial:   window (firstIndex, numPoints) of the full convolution written to pDst[firstIndex .. firstIndex+numPoints-1];
 *              "ARM_MATH_ARGUMENT_ERROR : requested subset is not in the range [0 srcALen+srcBLen-2]" when firstIndex + numPoints > srcALen+srcBLen-1.
 *   Levinson-Durbin: phi = autocovariance starting at lag 0 (nbCoefs + 1 values), a = AR coefficients (prediction x^[n] = sum a[i] x[n-1-i],
 *              i.e. a solves the Yule-Walker system sum_i a[i] phi[|j-i|] = phi[j+1]), err = prediction error variance
 *              phi[0] - sum a[i] phi[i+1]. Hand derived cases in the tests.
 *
 * Overflow-free data: the shorter operand (L = min length) has |s| <= 2^30 / L, the longer one the full range, so every |acc| <= L * 2^31 * 2^30 / L
 * = 2^61 (std result inside 1.31) and for the fast variant (non-negative data) the 2.30 sum stays below 2^29 + ... < 2^30.
 *
 * Host vs target (C6): the host runs the portable C path (no ARM_MATH_DSP, no MVE) with ARM_MATH_LOOPUNROLL. The DEF uses __int128, so a 64 bit
 * host compiler is required (GCC / clang on a 64 bit target, MinGW-w64 x86_64). `sum << 1`, `(q63_t) sum << 32` of a negative accumulator
 * (fast variants) and the final narrowing of a wrapped result are the places where host and target could differ: B-DCVQ31-1 below.
 *
 * Undefined behaviour (UBSan, run with --no-skip; the test cases that reach it are KNOWN_ISSUE_UB_HIDDEN: intended result on the host, skipped
 * in sanitizer builds), ONE id per defect:
 *   B-DCVQ31-1 the fast variants (arm_conv_fast_q31.c, arm_conv_partial_fast_q31.c, arm_correlate_fast_q31.c) shift a NEGATIVE accumulator left:
 *              "((q63_t) sum << 32)" in every MAC (e.g. arm_conv_fast_q31.c:162, :268) and "sum << 1" for the output (:196); left shift of a
 *              negative value is undefined in C. Non-negative data are clean. (Also: a 2.30 sum >= 2^30 makes "sum << 1" overflow int; not tested.)
 *   B-DCVQ31-2 (wrong result AND out of bounds read, not hidden: KNOWN_ISSUE_UB) arm_conv_partial_fast_q31 with firstIndex > max(srcALen, srcBLen):
 *              stage 3 starts with count = srcBLen - 1 (arm_conv_partial_fast_q31.c:529) although only srcALen + srcBLen - 1 - firstIndex products
 *              exist, so it multiplies samples behind the end of the longer input (ASan: heap-buffer-overflow READ, :597). ARM's own window
 *              first = 9 (6 x 8) is one of these; it passes ARM's check only because ARM's input buffers hold zeros behind the data.
 *              The standard arm_conv_partial_q31 has the same stage code under ARM_MATH_DSP (line 70) but the host build runs its generic loop.
 *   B-DCVQ31-3 arm_levinson_durbin_q31.c:89 "r << (shift + 2)" (divide) shifts a negative value left (UBSan), reached by ARM's own patterns.
 *
 * Host-only coverage: with ARM_MATH_DSP undefined (the host) arm_conv_partial_q31 runs the generic loop of arm_conv_partial_q31.c:592 (one
 * output at a time), NOT the unrolled three-stage kernel; the unroll / stage claims of the other kernels do not apply to the standard partial
 * convolution here. arm_conv_partial_fast_q31 has no generic loop and always runs the stage code.
 * The whole file needs __int128 (guard below: it compiles to nothing on a 32 bit host, like the f16 files without _Float16).
 *
 * NEEDS REVIEW (characterization or untested; details in the report):
 *   1. DOC contradiction of arm_conv_q31 / arm_correlate_q31 / arm_conv_partial_q31: "if the accumulator overflows it wraps around" and "right
 *      shifted by 31 bits and saturated to 1.31 format". Code: (q31_t)(acc >> 31) wraps. The only single-term case that does not fit is
 *      (-2^31) * (-2^31) = 2^62 -> 2^31: the code gives -2^31, the DOC "saturated" would give 2^31 - 1. `characterization:` tests.
 *      Argument for the lead: (-2^31) * (-2^31) = 2^62 does not overflow the 64 bit accumulator (2.62 format, 1 guard bit), so the sentence
 *      about wrapping accumulators does not apply to this case and only "saturated to 1.31" governs the narrowing: suspected DOC / vendor
 *      defect (the wrap is what the code does), undecided; the test stays `characterization:`.
 *   2. arm_correlate_*: the padding outputs are never written (see DOC above): `characterization:`; q31 DOC silent about zero initialisation.
 *   3. fast variants: result outside the 2.30 range (sum >= 2^30) overflows the int left shift; DOC says "saturated": not tested.
 *   4. srcALen = 0 / srcBLen = 0, numPoints = 0 for partial, firstIndex + numPoints overflowing 32 bits: kernel arithmetic unspecified, not tested.
 *   5. arm_levinson_durbin_q31: nbCoefs = 0 (a[0] is written unconditionally), phi[0] <= 0 (divide by zero / negative): outside the contract, not tested.
 *   6. arm_conv_partial_*: the DOC says only that pDst receives the output; the placement at pDst[firstIndex ..] (code and ARM's test) and "nothing
 *      before the window is written" are `characterization:` tests.
 *   7. arm_conv_partial_q31 with ARM_MATH_DSP (target): its stage code has the same defect as B-DCVQ31-2 (count = srcBLen - 1 not reduced for
 *      firstIndex > max length). Measured only in a scratch build with ARM_MATH_DSP set for that file; not run in the suite (no test).
 *   8. Levinson-Durbin tolerance: ARM's 30 LSB (charter L4) is kept but NOT derived (the accuracy of arm_divide_q15 is undocumented). Measured
 *      deviations reach 8-11 LSB on the hand derived cases and up to 53 LSB on other random seeds, so the committed seeds pass against 30 by luck.
 *      Mutants "err +4 / +16 LSB" and "a[last] +8 LSB" survive these tests (the bound is too loose to see them). Not widened.
 *   9. ARM's partial fast id first = 9 (6 x 8) reads past srcA (B-DCVQ31-2); covered only by the beyond-window sweep (KNOWN_ISSUE_UB).
 */
#include "doctest.h"

// the DEF needs __int128: nothing is compiled on a 32 bit host
#if defined(__SIZEOF_INT128__)

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
const i128 kOne = 1;

struct Rng {
	uint64_t s;
	explicit Rng(uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ull + 0x1234567ull) {
		if (s == 0) s = 1;
	}
	uint32_t next32() {  // xorshift64, high half; every call is its own statement
		s ^= s << 13;
		s ^= s >> 7;
		s ^= s << 17;
		return static_cast<uint32_t>(s >> 32);
	}
	int64_t range(int64_t lo, int64_t hi) {  // uniform in [lo, hi], span <= 2^32
		const uint64_t span = static_cast<uint64_t>(hi - lo) + 1u;
		const uint32_t r = next32();
		return lo + static_cast<int64_t>((static_cast<uint64_t>(r) * span) >> 32);
	}
};

typedef std::vector<int32_t> V;

std::string pdir() { return "Filtering/MISC/MISCQ31/"; }
V pat(const std::string &n) { return dsp::loadPattern(pdir() + n).q31(); }
V firstN(const V &v, size_t n) {
	REQUIRE(n <= v.size());
	return V(v.begin(), v.begin() + static_cast<long>(n));
}

// ---------------------------------------------------------------------------------------------------------------
// The four convolution / correlation kernels
// ---------------------------------------------------------------------------------------------------------------
enum Kind { CONV, CONVF, CORR, CORRF };
bool isFast(Kind k) { return k == CONVF || k == CORRF; }
bool isCorr(Kind k) { return k == CORR || k == CORRF; }
size_t outLenOf(Kind k, size_t A, size_t B) { return isCorr(k) ? 2 * std::max(A, B) - 1 : A + B - 1; }
void callKind(Kind k, const q31_t *a, size_t A, const q31_t *b, size_t B, q31_t *o) {
	const uint32_t la = static_cast<uint32_t>(A), lb = static_cast<uint32_t>(B);
	switch (k) {
		case CONV: arm_conv_q31(a, la, b, lb, o); break;
		case CONVF: arm_conv_fast_q31(a, la, b, lb, o); break;
		case CORR: arm_correlate_q31(a, la, b, lb, o); break;
		case CORRF: arm_correlate_fast_q31(a, la, b, lb, o); break;
	}
}

// DEF: exact integer model. acc = sum of products, accf = sum of floor(product / 2^32), valid = a lag that is computed (not padding)
struct Model {
	std::vector<i128> acc, accf, out;
	std::vector<char> valid;
};
i128 floorDiv32(i128 p) { return p >> 32; }  // arithmetic shift of __int128 = floor (GCC / clang)
Model defModel(Kind k, const V &a, const V &b) {
	const long A = static_cast<long>(a.size()), B = static_cast<long>(b.size());
	const long n = static_cast<long>(outLenOf(k, a.size(), b.size()));
	const long M = std::max(A, B);
	Model m;
	m.acc.assign(static_cast<size_t>(n), 0);
	m.accf.assign(static_cast<size_t>(n), 0);
	m.out.assign(static_cast<size_t>(n), 0);
	m.valid.assign(static_cast<size_t>(n), 1);
	for (long p = 0; p < n; p++) {
		i128 s = 0, sf = 0;
		if (!isCorr(k)) {
			for (long i = 0; i < A; i++) {
				const long j = p - i;
				if (j < 0 || j >= B) continue;
				const i128 pr = static_cast<i128>(a[static_cast<size_t>(i)]) * b[static_cast<size_t>(j)];
				s += pr;
				sf += floorDiv32(pr);
			}
		} else {
			const long lag = p - (M - 1);  // r[lag] = sum_k a[k+lag] b[k]
			if (lag < -(B - 1) || lag > A - 1) {
				m.valid[static_cast<size_t>(p)] = 0;
			} else {
				for (long q = 0; q < B; q++) {
					const long i = q + lag;
					if (i < 0 || i >= A) continue;
					const i128 pr = static_cast<i128>(a[static_cast<size_t>(i)]) * b[static_cast<size_t>(q)];
					s += pr;
					sf += floorDiv32(pr);
				}
			}
		}
		m.acc[static_cast<size_t>(p)] = s;
		m.accf[static_cast<size_t>(p)] = sf;
		m.out[static_cast<size_t>(p)] = isFast(k) ? sf * 2 : (s >> 31);
	}
	return m;
}
// the intended result exists (no overflow of the result / of the 2.30 accumulator)
bool usable(Kind k, const Model &m, size_t i) {
	if (!m.valid[i]) return true;
	if (isFast(k)) return m.accf[i] >= -(kOne << 30) && m.accf[i] <= (kOne << 30) - 1;
	return m.out[i] >= -(kOne << 31) && m.out[i] <= (kOne << 31) - 1;
}

// Data: the shorter operand limited to |s| <= 2^30 / L, the longer one full range (header). neg = false: non-negative data.
struct Data {
	V a, b;
};
Data makeData(Rng &rng, size_t A, size_t B, bool neg) {
	const size_t L = std::min(A, B);
	const int64_t lim = (int64_t(1) << 30) / static_cast<int64_t>(L);
	Data d;
	d.a.resize(A);
	d.b.resize(B);
	const bool aShort = A <= B;
	for (size_t i = 0; i < A; i++) {
		const int64_t hi = aShort ? lim : 0x7FFFFFFFll;
		const int64_t lo = neg ? (aShort ? -lim : -0x80000000ll) : 0;
		const int64_t r = rng.range(lo, hi);
		d.a[i] = static_cast<int32_t>(r);
	}
	for (size_t i = 0; i < B; i++) {
		const int64_t hi = aShort ? 0x7FFFFFFFll : lim;
		const int64_t lo = neg ? (aShort ? -0x80000000ll : -lim) : 0;
		const int64_t r = rng.range(lo, hi);
		d.b[i] = static_cast<int32_t>(r);
	}
	return d;
}

int32_t poison32() {
	unsigned char raw[4];
	std::memset(raw, 0xCD, 4);
	int32_t v;
	std::memcpy(&v, raw, 4);
	return v;
}

// ---------------------------------------------------------------------------------------------------------------
// Runners: one run in a GuardedBuffer (guard check), one in tight heap allocations of exactly the sizes (ASan sees any overrun);
// zeroInit: the output is zero before the call (the f32 DOC rule, ARM's output buffers), else it holds the poison pattern
// ---------------------------------------------------------------------------------------------------------------
struct Run {
	V out;
	bool guardOk = true;
	bool tightSame = true;
};
Run runKind(Kind k, const V &a, const V &b, bool zeroInit) {
	Run r;
	const size_t n = outLenOf(k, a.size(), b.size());
	dsp::GuardedBuffer<int32_t> o(n);
	if (zeroInit) std::memset(o.data(), 0, n * sizeof(int32_t));
	callKind(k, a.data(), a.size(), b.data(), b.size(), o.data());
	r.guardOk = o.guardIntact();
	r.out = o.output();
	std::unique_ptr<int32_t[]> ta(new int32_t[a.size()]), tb(new int32_t[b.size()]), to(new int32_t[n]);
	std::memcpy(ta.get(), a.data(), a.size() * 4);
	std::memcpy(tb.get(), b.data(), b.size() * 4);
	for (size_t i = 0; i < n; i++) to[i] = zeroInit ? 0 : poison32();
	callKind(k, ta.get(), a.size(), tb.get(), b.size(), to.get());
	r.tightSame = std::memcmp(to.get(), r.out.data(), n * 4) == 0;
	return r;
}

struct Cmp {
	size_t bad = 0, used = 0, skipped = 0;
	std::string first;
	bool guardsOk = true;
};
// exact comparison with the model of every usable output; padding of the correlation must be 0 (zero initialised output)
void compareExact(Kind k, const V &a, const V &b, const std::string &tag, Cmp &c) {
	const Run r = runKind(k, a, b, true);
	const Model m = defModel(k, a, b);
	REQUIRE(r.out.size() == m.out.size());
	if (!r.guardOk || !r.tightSame) c.guardsOk = false;
	for (size_t i = 0; i < m.out.size(); i++) {
		if (!usable(k, m, i)) {
			c.skipped++;
			continue;
		}
		c.used++;
		const i128 want = m.valid[i] ? m.out[i] : 0;
		if (static_cast<i128>(r.out[i]) != want) {
			if (c.bad == 0)
				c.first = tag + " out[" + std::to_string(i) + "] = " + std::to_string(r.out[i]) + ", model " + std::to_string(static_cast<long long>(want));
			c.bad++;
		}
	}
}

// ---------------------------------------------------------------------------------------------------------------
// Tests of the four kernels
// ---------------------------------------------------------------------------------------------------------------
// C1: ARM's patterns. Standard variants: ARM's SNR and 2 LSB. Fast variants (no ARM id): the bound derived in the header, 2 L + 2
// (floor error < 2 L, ARM's reference within 2 LSB of the exact value), SNR not asserted (ARM has no threshold for them).
void armPatternTest(Kind k, bool neg) {
	const V A1 = pat("InputsA1_q31.txt"), B1 = pat("InputsB1_q31.txt");
	const size_t nbas[] = {4, 5, 6, 9, 10, 11, 12, 13}, nbbs[] = {1, 2, 3, 8, 11};
	int id = isCorr(k) ? 1 : 41;
	size_t ran = 0, bad = 0, skipped = 0;
	std::string first;
	bool guards = true, snrOk = true;
	for (size_t na : nbas)
		for (size_t nb : nbbs) {
			const V ref = pat("Reference" + std::to_string(id++) + "_q31.txt");
			const V a = firstN(A1, na), b = firstN(B1, nb);
			REQUIRE(ref.size() == outLenOf(k, na, nb));
			if (isFast(k)) {
				const Model m = defModel(k, a, b);
				bool ok = true;
				for (size_t i = 0; i < m.out.size(); i++) ok = ok && usable(k, m, i);
				if (!ok) {
					skipped++;
					continue;
				}
				// signed ARM data: only the hidden variant runs it (negative accumulators), the clean one has no non-negative ARM data
				if (!neg) {
					bool allPos = true;
					for (int32_t v : a) allPos = allPos && v >= 0;
					for (int32_t v : b) allPos = allPos && v >= 0;
					if (!allPos) {
						skipped++;
						continue;
					}
				}
			}
			const Run r = runKind(k, a, b, true);
			ran++;
			guards = guards && r.guardOk && r.tightSame;
			const int64_t bound = isFast(k) ? 2 * static_cast<int64_t>(std::min(na, nb)) + 2 : 2;  // MISCQ31.cpp:15
			const auto c = dsp::withinLsb(ref, r.out, bound);
			if (!c) {
				if (bad == 0) first = "nba " + std::to_string(na) + " nbb " + std::to_string(nb) + ": " + c.message;
				bad++;
			}
			if (!isFast(k) && !(dsp::snr(ref, r.out) >= 100.0)) snrOk = false;  // MISCQ31.cpp:8
		}
	CHECK(ran > 0);
	CHECK(guards);
	CHECK(snrOk);
	CHECK_MESSAGE(bad == 0, bad << " ARM configs outside the bound, first " << first);
	if (!isFast(k)) CHECK(skipped == 0);
}

// C3: random data of all length pairs 1..13 (both argument orders, unroll tails of 3 and 4), exact against the model
void defPairsTest(Kind k, bool neg) {
	Rng rng(1000 + static_cast<uint64_t>(k) * 17 + (neg ? 1 : 0));
	Cmp c;
	for (size_t A = 1; A <= 13; A++)
		for (size_t B = 1; B <= 13; B++)
			for (int rep = 0; rep < 3; rep++) {
				const Data d = makeData(rng, A, B, neg);
				compareExact(k, d.a, d.b, "A " + std::to_string(A) + " B " + std::to_string(B), c);
			}
	CHECK(c.guardsOk);
	CHECK_MESSAGE(c.bad == 0, c.bad << " outputs differ from the model, first " << c.first);
	CHECK(c.used >= 6000);
}
// longer sequences: the stage 2 loops (unrolled by 3 for the standard, by 4 for the fast variants) with tails
void defLongTest(Kind k, bool neg) {
	Rng rng(2000 + static_cast<uint64_t>(k) * 17 + (neg ? 1 : 0));
	Cmp c;
	const size_t As[] = {17, 33, 64}, Bs[] = {1, 2, 3, 4, 5, 7, 12, 16, 17, 25, 33};
	for (size_t A : As)
		for (size_t B : Bs) {
			const Data d = makeData(rng, A, B, neg);
			compareExact(k, d.a, d.b, "A " + std::to_string(A) + " B " + std::to_string(B), c);
			compareExact(k, d.b, d.a, "A " + std::to_string(B) + " B " + std::to_string(A), c);
		}
	CHECK(c.guardsOk);
	CHECK_MESSAGE(c.bad == 0, c.bad << " outputs differ from the model, first " << c.first);
}

// C3: hand derived: a = [1,2,3] u, b = [4,5] u with u = 2^28: products ij 2^56, >> 31 = ij 2^25 (fast: floor(ij 2^56 / 2^32) = ij 2^24, times 2)
//  conv  [1,2,3]*[4,5] = [4, 13, 22, 15];  correlation r[n] = sum a[k+n] b[k], lags -1..2: [5, 14, 23, 12]
//  A = 3 > B = 2: M = 3, out[p] = r[p-2]: [0 (padding), 5, 14, 23, 12];  A = 2 < B = 3 (a = [4,5], b = [1,2,3]): lags -2..1: r = [12, 23, 14, 5],
//  out = [12, 23, 14, 5, 0 (padding)].
void handTest(Kind k) {
	const int32_t u = 1 << 28, e = 1 << 25;
	const V x = {1 * u, 2 * u, 3 * u}, y = {4 * u, 5 * u};
	std::vector<int64_t> want1, want2;
	if (!isCorr(k)) {
		want1 = {4, 13, 22, 15};
		want2 = {4, 13, 22, 15};
	} else {
		want1 = {0, 5, 14, 23, 12};
		want2 = {12, 23, 14, 5, 0};
	}
	const Run r1 = runKind(k, x, y, true);
	REQUIRE(r1.out.size() == want1.size());
	for (size_t i = 0; i < want1.size(); i++) CHECK_MESSAGE(r1.out[i] == want1[i] * e, "A > B, out[" << i << "]");
	const Run r2 = runKind(k, y, x, true);
	REQUIRE(r2.out.size() == want2.size());
	// conv is commutative; for the correlation swapped arguments give the reversed lag order (hand derivation above, b = x, a = y)
	for (size_t i = 0; i < want2.size(); i++) CHECK_MESSAGE(r2.out[i] == want2[i] * e, "A < B, out[" << i << "]");
	CHECK(r1.guardOk);
	CHECK(r2.guardOk);
	CHECK(r1.tightSame);
	CHECK(r2.tightSame);
}

// C3: a one tap kernel: conv(a, {2^30}) = floor(a / 2) (a * 2^30 / 2^31). Fast: product floor(a 2^30 / 2^32) = floor(a/4), times 2: multiples of 4 -> a / 2
void halfTest(Kind k) {
	Rng rng(3000 + static_cast<uint64_t>(k));
	for (size_t n : {size_t(1), size_t(2), size_t(3), size_t(4), size_t(5), size_t(7), size_t(9), size_t(17)}) {
		V a(n);
		for (size_t i = 0; i < n; i++) {
			const int64_t r = rng.range(isFast(k) ? 0 : -0x80000000ll, 0x7FFFFFFFll);
			a[i] = static_cast<int32_t>(isFast(k) ? (r & ~int64_t(3)) : r);
		}
		const V b = {1 << 30};
		const Run r = runKind(k, a, b, true);
		REQUIRE(r.out.size() == outLenOf(k, n, 1));
		for (size_t i = 0; i < n; i++) {
			// a one element b: conv and correlation (M = n, lag 0 at p = n-1: the padding is not part of the nonzero lags for B = 1:
			// lags -(B-1)..A-1 = 0..n-1, p = lag + n - 1 >= n - 1: out has length 2n-1, the first n-1 are padding)
			const size_t p = isCorr(k) ? (n - 1 + i) : i;
			if (isCorr(k) && p >= r.out.size()) continue;
			const int64_t want = isFast(k) ? a[i] / 2 : (static_cast<int64_t>(a[i]) >> 1);
			CHECK_MESSAGE(r.out[p] == want, "n " << n << " i " << i);
		}
		if (isCorr(k))
			for (size_t p = 0; p + 1 < n; p++) CHECK(r.out[p] == 0);
	}
}

// C3: extremes with a single product: max * max = (2^31-1)^2 = 2^62 - 2^32 + 1 -> >> 31 = 2^31 - 2 ; min * max = -2^62 + 2^31 -> -2^31 + 1 (both exact
// in 1.31). The fast variant has the products floor(.. / 2^32): (2^62 - 2^32 + 1) >> 32 = 2^30 - 1 -> 2^31 - 2 (the same), and floor(-2^62 + 2^31 / 2^32)
// = -2^30 -> -2^31 (a floor error of 1 LSB below the exact -2^31 + 1).
void extremeTest(Kind k) {
	const V mx = {0x7FFFFFFF}, mn = {static_cast<int32_t>(0x80000000u)};
	const Run r1 = runKind(k, mx, mx, true);
	REQUIRE(r1.out.size() == 1);
	CHECK(r1.out[0] == 0x7FFFFFFE);
	if (!isFast(k)) {
		const Run r2 = runKind(k, mn, mx, true);
		REQUIRE(r2.out.size() == 1);
		CHECK(r2.out[0] == static_cast<int32_t>(0x80000001u));
	}
}

// the wrap of the one single term that does not fit: NEEDS REVIEW 1
void wrapCharacterization(Kind k) {
	const V mn = {static_cast<int32_t>(0x80000000u)};
	const Run r = runKind(k, mn, mn, true);
	REQUIRE(r.out.size() == 1);
	CHECK(r.out[0] == static_cast<int32_t>(0x80000000u));  // 2^31 narrowed to q31 (the DOC "saturated" would give 0x7FFFFFFF)
}

// C5: against a double computation of the converted input (x / 2^31): the double sum of products is exact to 2^-53 relative,
// per product error <= 2^62 * 2^-53 = 2^9 in units of 2^-62, i.e. L 2^-22 LSB (L terms): eps = L * 2^-21.
// Standard: 0 <= d - y < 1 (floor). Fast: 0 <= d - y < 2 L (floor of every product, times 2).
void doubleAgreementTest(Kind k, bool neg) {
	Rng rng(4000 + static_cast<uint64_t>(k) * 17 + (neg ? 1 : 0));
	size_t bad = 0, n = 0;
	std::string first;
	for (size_t A : {size_t(1), size_t(3), size_t(4), size_t(8), size_t(13)})
		for (size_t B : {size_t(2), size_t(5), size_t(9), size_t(12)}) {
			const Data d = makeData(rng, A, B, neg);
			const Run r = runKind(k, d.a, d.b, true);
			const Model m = defModel(k, d.a, d.b);
			const size_t L = std::min(A, B);
			const double eps = static_cast<double>(L) * std::ldexp(1.0, -21);
			for (size_t i = 0; i < m.out.size(); i++) {
				if (!m.valid[i] || !usable(k, m, i)) continue;
				const long M = static_cast<long>(std::max(A, B));
				const long lag = static_cast<long>(i) - (M - 1);
				double s = 0;
				if (!isCorr(k)) {
					for (size_t q = 0; q < A; q++) {
						if (i < q || i - q >= B) continue;
						s += (static_cast<double>(d.a[q]) / 2147483648.0) * (static_cast<double>(d.b[i - q]) / 2147483648.0);
					}
				} else {
					for (size_t q = 0; q < B; q++) {
						const long j = static_cast<long>(q) + lag;
						if (j < 0 || j >= static_cast<long>(A)) continue;
						s += (static_cast<double>(d.a[static_cast<size_t>(j)]) / 2147483648.0) * (static_cast<double>(d.b[q]) / 2147483648.0);
					}
				}
				const double ds = s * 2147483648.0 - static_cast<double>(r.out[i]);  // d - y in LSB
				const double hi = isFast(k) ? 2.0 * static_cast<double>(L) + eps : 1.0 + eps;
				n++;
				if (!(ds > -eps && ds < hi)) {
					if (bad == 0) first = "A " + std::to_string(A) + " B " + std::to_string(B) + " out[" + std::to_string(i) + "] d-y = " + std::to_string(ds);
					bad++;
				}
			}
		}
	CHECK(n > 200);
	CHECK_MESSAGE(bad == 0, bad << " samples outside [0, bound), first " << first);
}

// C2: the inputs are not modified; the padding of the correlation is not touched (characterization, NEEDS REVIEW 2)
void inputsConstTest(Kind k) {
	Rng rng(5000 + static_cast<uint64_t>(k));
	const Data d = makeData(rng, 9, 5, false);
	V a = d.a, b = d.b;
	const Run r = runKind(k, a, b, true);
	CHECK(a == d.a);
	CHECK(b == d.b);
	CHECK(r.guardOk);
	// repeat: same inputs -> same output (no state)
	const Run r2 = runKind(k, a, b, true);
	CHECK(r.out == r2.out);
}
void paddingCharacterization(Kind k) {
	// srcALen = 6 > srcBLen = 2: padding = first 4 outputs; srcALen = 2 < srcBLen = 6: the last 4 stay untouched (poison)
	Rng rng(6000 + static_cast<uint64_t>(k));
	const Data d1 = makeData(rng, 6, 2, false);
	const Run r1 = runKind(k, d1.a, d1.b, false);
	for (size_t p = 0; p < 4; p++) CHECK_MESSAGE(r1.out[p] == poison32(), "A > B padding out[" << p << "]");
	const Data d2 = makeData(rng, 2, 6, false);
	const Run r2 = runKind(k, d2.a, d2.b, false);
	for (size_t p = 7; p < 11; p++) CHECK_MESSAGE(r2.out[p] == poison32(), "A < B padding out[" << p << "]");
	CHECK(r1.guardOk);
	CHECK(r2.guardOk);
}

// ---------------------------------------------------------------------------------------------------------------
// Partial convolution
// ---------------------------------------------------------------------------------------------------------------
struct PRun {
	arm_status st = ARM_MATH_SUCCESS;
	V window;
	bool guardOk = true, prefixUntouched = true, tightSame = true;
	V all;
};
PRun runPartial(bool fast, const V &a, const V &b, uint32_t first, uint32_t num) {
	PRun r;
	const size_t n = static_cast<size_t>(first) + num;
	dsp::GuardedBuffer<int32_t> o(n);
	r.st = (fast ? arm_conv_partial_fast_q31 : arm_conv_partial_q31)(a.data(), static_cast<uint32_t>(a.size()), b.data(), static_cast<uint32_t>(b.size()), o.data(), first, num);
	r.guardOk = o.guardIntact();
	r.all = o.output();
	for (uint32_t i = 0; i < first; i++) r.prefixUntouched = r.prefixUntouched && r.all[i] == poison32();
	r.window.assign(r.all.begin() + first, r.all.end());
	std::unique_ptr<int32_t[]> ta(new int32_t[a.size()]), tb(new int32_t[b.size()]), to(new int32_t[n]);
	std::memcpy(ta.get(), a.data(), a.size() * 4);
	std::memcpy(tb.get(), b.data(), b.size() * 4);
	for (size_t i = 0; i < n; i++) to[i] = poison32();
	(fast ? arm_conv_partial_fast_q31 : arm_conv_partial_q31)(ta.get(), static_cast<uint32_t>(a.size()), tb.get(), static_cast<uint32_t>(b.size()), to.get(), first, num);
	r.tightSame = std::memcmp(to.get(), r.all.data(), n * 4) == 0;
	return r;
}

// C1: ARM's three windows (nba 6, nbb 8, numPoints 4, first 3 / 9 / 7), ARM bound 2 (std) or 11 (fast, MISCQ31.cpp:17) and SNR 100
void armPartialTest(bool fast, const std::vector<int> &which) {
	const V A2 = pat("InputsA2_q31.txt"), B2 = pat("InputsB2_q31.txt");
	const uint32_t firsts[] = {3, 9, 7};
	const int ids[] = {84, 85, 86};
	for (int t : which) {
		const V ref = pat("Reference" + std::to_string(ids[t]) + "_q31.txt");
		const V a = firstN(A2, 6), b = firstN(B2, 8);
		const PRun r = runPartial(fast, a, b, firsts[t], 4);
		CHECK_MESSAGE(r.st == ARM_MATH_SUCCESS, "first " << firsts[t]);
		const auto c = dsp::withinLsb(ref, r.window, fast ? 11 : 2);
		CHECK_MESSAGE(c, "first " << firsts[t] << ": " << c.message);
		CHECK_MESSAGE(dsp::snr(ref, r.window) >= 100.0, "first " << firsts[t]);
		CHECK(r.guardOk);
		CHECK(r.tightSame);
	}
}

// C3: every window (first, numPoints >= 1) of every length pair 1..9 (both orders) equals the window of the model
// beyond = false: windows with firstIndex <= max(srcALen, srcBLen); true: firstIndex > max (B-DCVQ31-2)
void partialSweepTest(bool fast, bool neg, bool beyond = false) {
	Rng rng(7000 + (fast ? 1 : 0) + (neg ? 2 : 0));
	size_t skippedWindows = 0;
	size_t bad = 0, runs = 0;
	std::string first, flags;
	bool guards = true, status = true;
	for (size_t A = 1; A <= 9; A++)
		for (size_t B = 1; B <= 9; B++) {
			const Data d = makeData(rng, A, B, neg);
			const Model m = defModel(fast ? CONVF : CONV, d.a, d.b);
			const uint32_t L = static_cast<uint32_t>(A + B - 1);
			for (uint32_t f = 0; f < L; f++)
				for (uint32_t num = 1; f + num <= L; num++) {
					if ((f > std::max(A, B)) != beyond) {
						skippedWindows++;
						continue;
					}
					const PRun r = runPartial(fast, d.a, d.b, f, num);
					runs++;
					if ((!r.guardOk || !r.tightSame || r.st != ARM_MATH_SUCCESS) && flags.empty())
						flags = "A " + std::to_string(A) + " B " + std::to_string(B) + " first " + std::to_string(f) + " num " + std::to_string(num) + " guard " + std::to_string(r.guardOk) + " tightSame " + std::to_string(r.tightSame) + " status " + std::to_string(static_cast<int>(r.st));
					guards = guards && r.guardOk && r.tightSame;
					status = status && r.st == ARM_MATH_SUCCESS;
					for (uint32_t i = 0; i < num; i++)
						if (static_cast<i128>(r.window[i]) != m.out[f + i]) {
							if (bad == 0) first = "A " + std::to_string(A) + " B " + std::to_string(B) + " first " + std::to_string(f) + " num " + std::to_string(num) + " i " + std::to_string(i) + " out " + std::to_string(r.window[i]) + " model " + std::to_string(static_cast<long long>(m.out[f + i]));
							bad++;
						}
				}
		}
	CHECK(runs > (beyond ? 100u : 2000u));
	CHECK_MESSAGE(bad == 0, bad << " samples differ from the model, first " << first);
	(void)skippedWindows;
	CHECK_MESSAGE(status, flags);
	CHECK_MESSAGE(guards, flags);
}

// C4: DOC: ARGUMENT_ERROR when the window is not in [0, srcALen+srcBLen-2]; the last valid window succeeds
void partialErrorTest(bool fast) {
	Rng rng(8000 + (fast ? 1 : 0));
	const Data d = makeData(rng, 6, 8, false);  // full length 13
	const V &a = d.a, &b = d.b;
	const PRun okLast = runPartial(fast, a, b, 8, 5);  // 8 + 5 = 13: ok (a first index above 8 is B-DCVQ31-2)
	CHECK(okLast.st == ARM_MATH_SUCCESS);
	const PRun okAll = runPartial(fast, a, b, 0, 13);
	CHECK(okAll.st == ARM_MATH_SUCCESS);
	const PRun e1 = runPartial(fast, a, b, 9, 5);  // 14 > 13
	CHECK(e1.st == ARM_MATH_ARGUMENT_ERROR);
	const PRun e2 = runPartial(fast, a, b, 13, 1);
	CHECK(e2.st == ARM_MATH_ARGUMENT_ERROR);
	const PRun e3 = runPartial(fast, a, b, 0, 14);
	CHECK(e3.st == ARM_MATH_ARGUMENT_ERROR);
	const PRun e4 = runPartial(fast, a, b, 20, 3);
	CHECK(e4.st == ARM_MATH_ARGUMENT_ERROR);
	CHECK(e1.guardOk);
	CHECK(e3.guardOk);
}
// DOC silent about the output on error; the code returns before writing: characterization
void partialErrorWritesNothing(bool fast) {
	Rng rng(8100 + (fast ? 1 : 0));
	const Data d = makeData(rng, 6, 8, false);
	for (auto fn : {std::make_pair(9u, 5u), std::make_pair(0u, 14u), std::make_pair(13u, 1u)}) {
		const PRun e = runPartial(fast, d.a, d.b, fn.first, fn.second);
		CHECK(e.st == ARM_MATH_ARGUMENT_ERROR);
		bool untouched = true;
		for (int32_t v : e.all) untouched = untouched && v == poison32();
		CHECK_MESSAGE(untouched, "first " << fn.first << " num " << fn.second);
	}
}

// DOC says only "pDst points to the location where the output result is written"; code and ARM's test place the window at pDst[firstIndex ..]
// and write nothing before it: characterization (NEEDS REVIEW 6)
void partialPlacementCharacterization(bool fast) {
	Rng rng(8200 + (fast ? 1 : 0));
	const Data d = makeData(rng, 6, 8, false);
	const Model m = defModel(fast ? CONVF : CONV, d.a, d.b);
	for (uint32_t f : {0u, 3u, 7u, 8u}) {
		const PRun r = runPartial(fast, d.a, d.b, f, 4);
		CHECK_MESSAGE(r.prefixUntouched, "first " << f);
		for (uint32_t i = 0; i < 4; i++) CHECK_MESSAGE(static_cast<i128>(r.all[f + i]) == m.out[f + i], "first " << f << " i " << i);
	}
}

// the partial result with the full window equals the full convolution (bit for bit): C3 consistency
void partialEqualsFullTest(bool fast, bool neg) {
	Rng rng(9000 + (fast ? 1 : 0) + (neg ? 2 : 0));
	bool same = true;
	for (size_t A : {size_t(5), size_t(8), size_t(13)})
		for (size_t B : {size_t(3), size_t(8), size_t(11)}) {
			const Data d = makeData(rng, A, B, neg);
			const Run full = runKind(fast ? CONVF : CONV, d.a, d.b, false);
			const PRun p = runPartial(fast, d.a, d.b, 0, static_cast<uint32_t>(A + B - 1));
			same = same && p.window == full.out;
		}
	CHECK(same);
}

// the partial window agrees with the double computation: floor error as above
void partialSaturationCharacterization() {
	const V mn = {static_cast<int32_t>(0x80000000u)};
	const PRun r = runPartial(false, mn, mn, 0, 1);
	CHECK(r.st == ARM_MATH_SUCCESS);
	REQUIRE(r.window.size() == 1);
	CHECK(r.window[0] == static_cast<int32_t>(0x80000000u));
}

// ---------------------------------------------------------------------------------------------------------------
// Levinson-Durbin
// ---------------------------------------------------------------------------------------------------------------
struct LdOut {
	V a;
	int32_t err = 0;
	bool guardOk = true;
};
LdOut runLd(const V &phi, int n) {
	LdOut r;
	REQUIRE(phi.size() == static_cast<size_t>(n) + 1);
	std::unique_ptr<int32_t[]> tphi(new int32_t[phi.size()]);
	std::memcpy(tphi.get(), phi.data(), phi.size() * 4);
	dsp::GuardedBuffer<int32_t> a(static_cast<size_t>(n));
	q31_t e = poison32();
	arm_levinson_durbin_q31(tphi.get(), a.data(), &e, n);
	r.guardOk = a.guardIntact();
	r.a = a.output();
	r.err = e;
	// tight heap output of exactly n coefficients: the same result
	std::unique_ptr<int32_t[]> ta(new int32_t[static_cast<size_t>(n)]);
	q31_t e2 = poison32();
	arm_levinson_durbin_q31(tphi.get(), ta.get(), &e2, n);
	if (std::memcmp(ta.get(), r.a.data(), static_cast<size_t>(n) * 4) != 0 || e2 != e) r.guardOk = false;
	return r;
}
// DEF: double Levinson-Durbin of the Yule-Walker system (the algorithm of the comments in the library source)
void defLd(const std::vector<double> &phi, int n, std::vector<double> &a, double &e) {
	a.assign(static_cast<size_t>(n), 0.0);
	a[0] = phi[1] / phi[0];
	e = phi[0] - phi[1] * a[0];
	for (int p = 1; p < n; p++) {
		double sa = 0, sb = 0;
		for (int i = 0; i < p; i++) {
			sa += a[static_cast<size_t>(i)] * phi[static_cast<size_t>(p - i)];
			sb += a[static_cast<size_t>(i)] * phi[static_cast<size_t>(i + 1)];
		}
		const double k = (phi[static_cast<size_t>(p + 1)] - sa) / (phi[0] - sb);
		std::vector<double> na = a;
		for (int i = 0; i < p; i++) na[static_cast<size_t>(i)] = a[static_cast<size_t>(i)] - k * a[static_cast<size_t>(p - 1 - i)];
		na[static_cast<size_t>(p)] = k;
		a = na;
		e = e * (1 - k * k);
	}
}
int32_t q31of(double v) { return static_cast<int32_t>(std::lrint(v * 2147483648.0)); }

void armLdTest() {
	const int ns[] = {3, 8, 11};
	const char *phis[] = {"InputPhi81_q31.txt", "InputPhi82_q31.txt", "InputPhi83_q31.txt"};
	const V errs = pat("LDErrors81_q31.txt");
	for (int t = 0; t < 3; t++) {
		const V phi = pat(phis[t]);
		const V ref = pat("Reference" + std::to_string(81 + t) + "_q31.txt");
		const LdOut r = runLd(phi, ns[t]);
		CHECK_MESSAGE(r.guardOk, "n " << ns[t]);
		const auto c = dsp::withinLsb(ref, r.a, 30);  // ABS_LD_ERROR, MISCQ31.cpp:25
		CHECK_MESSAGE(c, "n " << ns[t] << ": " << c.message);
		CHECK_MESSAGE(dsp::snr(ref, r.a) >= 100.0, "n " << ns[t]);  // MISCQ31.cpp:8
		REQUIRE(errs.size() == 3);
		CHECK_MESSAGE(std::llabs(static_cast<long long>(r.err) - errs[static_cast<size_t>(t)]) <= 30, "n " << ns[t] << " error " << r.err << " ref " << errs[static_cast<size_t>(t)]);
	}
}

// hand derived (header): tolerance 30 LSB = ARM's ABS_LD_ERROR (the library's divide uses a Q15 inverse with one Newton step and every
// 1.31 x 1.31 product floors; the 30 LSB of ARM is used because the accuracy of arm_divide_q15 is not specified in the DOC)
const int64_t kLdTol = 30;
void ldNear(const LdOut &r, const std::vector<double> &a, double e) {
	CHECK(r.guardOk);
	REQUIRE(r.a.size() == a.size());
	for (size_t i = 0; i < a.size(); i++) CHECK_MESSAGE(std::llabs(static_cast<long long>(r.a[i]) - q31of(a[i])) <= kLdTol, "a[" << i << "] = " << r.a[i] << " want " << q31of(a[i]));
	CHECK_MESSAGE(std::llabs(static_cast<long long>(r.err) - q31of(e)) <= kLdTol, "err = " << r.err << " want " << q31of(e));
}

}  // namespace

// ===============================================================================================================
// Test cases
// ===============================================================================================================
#define UB_FAST KNOWN_ISSUE_UB_HIDDEN("B-DCVQ31-1: the fast kernels shift a negative accumulator left ((q63_t) sum << 32 in every MAC, sum << 1 for the output)")

#define UB_LD KNOWN_ISSUE_UB_HIDDEN("B-DCVQ31-3: arm_levinson_durbin_q31.c:89 (divide) r << (shift + 2) shifts a negative value left when the quotient is negative")

#define KIND_TESTS(KIND, FN, EXACT_NAME)                                                                                                                  \
	TEST_CASE("[C3 " FN "] " EXACT_NAME ", all 169 length pairs 1 to 13 and both argument orders, random data, non-negative or full range") { defPairsTest(KIND, !isFast(KIND)); } \
	TEST_CASE("[C3 " FN "] longer inputs (17, 33, 64 against 1 to 33, both argument orders) equal the exact model: stage 2 unroll tails") { defLongTest(KIND, !isFast(KIND)); } \
	TEST_CASE("[C3 " FN "] the hand derived sequences [1,2,3] and [4,5] (units of 2^28) give the derived result in both argument orders") { handTest(KIND); } \
	TEST_CASE("[C3 " FN "] a one element sequence 2^30 halves the other (floor), for lengths 1, 2, 3, 4, 5, 7, 9, 17") { halfTest(KIND); } \
	TEST_CASE("[C3 " FN "] the product of the two largest values (2^31-1)^2 gives 2^31 - 2 exactly") { extremeTest(KIND); } \
	TEST_CASE("[C5 " FN "] the result agrees with the double computation of the converted input: one-sided floor error with the derived bound") { doubleAgreementTest(KIND, !isFast(KIND)); } \
	TEST_CASE("[C2 " FN "] the input sequences are not modified and a repeated call gives the same output") { inputsConstTest(KIND); }

TEST_SUITE("DSP Filtering q31") {

// ---------------------------------------------------------------------------------------------------------------
// arm_conv_q31
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_conv_q31] ARM's 40 pattern tests (nba 4..13 x nbb 1,2,3,8,11) reach SNR 100 and 2 LSB, guard behind the output intact") { armPatternTest(CONV, true); }
KIND_TESTS(CONV, "arm_conv_q31", "the output equals the exact model acc >> 31")
TEST_CASE("characterization: [C4 arm_conv_q31] (-2^31) * (-2^31) = 2^31 wraps to -2^31 (the DOC also says saturated to 1.31)") { wrapCharacterization(CONV); }

// ---------------------------------------------------------------------------------------------------------------
// arm_conv_fast_q31
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_conv_fast_q31] ARM's signed patterns of conv (no ARM id) stay within 2 L + 2 LSB of ARM's reference" * UB_FAST) { armPatternTest(CONVF, true); }
KIND_TESTS(CONVF, "arm_conv_fast_q31", "with non-negative data the output equals the exact model 2 * sum floor(a b / 2^32)")
TEST_CASE("[C3 arm_conv_fast_q31] signed data of all 169 length pairs 1 to 13 equal the exact model 2 * sum floor(a b / 2^32)" * UB_FAST) { defPairsTest(CONVF, true); }
TEST_CASE("[C5 arm_conv_fast_q31] signed data agree with the double computation: 0 <= d - y < 2 L" * UB_FAST) { doubleAgreementTest(CONVF, true); }

// ---------------------------------------------------------------------------------------------------------------
// arm_correlate_q31
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_correlate_q31] ARM's 40 pattern tests (nba 4..13 x nbb 1,2,3,8,11, zeroed output) reach SNR 100 and 2 LSB, guard intact") { armPatternTest(CORR, true); }
KIND_TESTS(CORR, "arm_correlate_q31", "the output equals out[p] = r[p - (M-1)] of the exact model, padding stays 0")
TEST_CASE("characterization: [C4 arm_correlate_q31] (-2^31) * (-2^31) = 2^31 wraps to -2^31 (the DOC also says saturated to 1.31)") { wrapCharacterization(CORR); }
TEST_CASE("characterization: [C2 arm_correlate_q31] the srcALen - srcBLen padding outputs are never written (6 against 2 and 2 against 6)") { paddingCharacterization(CORR); }

// ---------------------------------------------------------------------------------------------------------------
// arm_correlate_fast_q31
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_correlate_fast_q31] ARM's signed patterns of correlate (no ARM id) stay within 2 L + 2 LSB of ARM's reference" * UB_FAST) { armPatternTest(CORRF, true); }
KIND_TESTS(CORRF, "arm_correlate_fast_q31", "with non-negative data the output equals the exact model 2 * sum floor(a b / 2^32), padding stays 0")
TEST_CASE("[C3 arm_correlate_fast_q31] signed data of all 169 length pairs 1 to 13 equal the exact model" * UB_FAST) { defPairsTest(CORRF, true); }
TEST_CASE("[C5 arm_correlate_fast_q31] signed data agree with the double computation: 0 <= d - y < 2 L" * UB_FAST) { doubleAgreementTest(CORRF, true); }
TEST_CASE("characterization: [C2 arm_correlate_fast_q31] the srcALen - srcBLen padding outputs are never written (6 against 2 and 2 against 6)") { paddingCharacterization(CORRF); }

// ---------------------------------------------------------------------------------------------------------------
// arm_conv_partial_q31 / arm_conv_partial_fast_q31
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_conv_partial_q31] ARM's three windows (6 x 8, first 3, 9, 7, 4 points) reach SNR 100 and 2 LSB, ") { armPartialTest(false, {0, 1, 2}); }
TEST_CASE("[C3 arm_conv_partial_q31] every window with firstIndex <= max(srcALen, srcBLen) of every length pair 1 to 9 (both orders) equals the window of the exact convolution") { partialSweepTest(false, true); }
TEST_CASE("[C3 arm_conv_partial_q31] the window 0 .. srcALen+srcBLen-2 equals arm_conv_q31 bit for bit") { partialEqualsFullTest(false, true); }
TEST_CASE("[C4 arm_conv_partial_q31] a window outside [0, srcALen+srcBLen-2] returns ARM_MATH_ARGUMENT_ERROR, the last valid windows succeed") { partialErrorTest(false); }
TEST_CASE("characterization: [C2 arm_conv_partial_q31] the window is written at pDst[firstIndex ..] and nothing before it (first 0, 3, 7, 8)") { partialPlacementCharacterization(false); }
TEST_CASE("characterization: [C4 arm_conv_partial_q31] on ARM_MATH_ARGUMENT_ERROR nothing is written to the output") { partialErrorWritesNothing(false); }
TEST_CASE("characterization: [C4 arm_conv_partial_q31] (-2^31) * (-2^31) wraps to -2^31 (the DOC also says saturated to 1.31)") { partialSaturationCharacterization(); }

TEST_CASE("[C1 arm_conv_partial_fast_q31] ARM's windows first = 3 and 7 (6 x 8, 4 points) reach SNR 100 and 11 LSB, " * UB_FAST) { armPartialTest(true, {0, 2}); }
// ARM's window first = 9 (6 x 8) reads past srcA (B-DCVQ31-2): it is NOT run as a C1 case (its only failing assertion would depend on what the
// allocator keeps behind the buffer). The ARM id is covered only through the beyond-window sweep below (value check first).
TEST_CASE("[C3 arm_conv_partial_fast_q31] windows with firstIndex > max(srcALen, srcBLen) equal the exact model, non-negative data"
          * KNOWN_ISSUE_UB("B-DCVQ31-2: arm_conv_partial_fast_q31.c:529 count = srcBLen - 1 is not reduced for firstIndex > srcALen: stage 3 multiplies samples behind the end of the longer input, e.g. 3 x 4 first 5: two products instead of one")) { partialSweepTest(true, false, true); }
TEST_CASE("[C3 arm_conv_partial_q31] windows with firstIndex > max(srcALen, srcBLen) equal the exact model (host: generic loop, see NEEDS REVIEW 7)") { partialSweepTest(false, true, true); }
TEST_CASE("[C3 arm_conv_partial_fast_q31] every window with firstIndex <= max(srcALen, srcBLen) of every length pair 1 to 9 equals the exact model with non-negative data") { partialSweepTest(true, false); }
TEST_CASE("[C3 arm_conv_partial_fast_q31] every window with firstIndex <= max(srcALen, srcBLen) of every length pair 1 to 9 equals the exact model with signed data" * UB_FAST) { partialSweepTest(true, true); }
TEST_CASE("[C3 arm_conv_partial_fast_q31] the window 0 .. srcALen+srcBLen-2 equals arm_conv_fast_q31 bit for bit, non-negative data") { partialEqualsFullTest(true, false); }
TEST_CASE("[C4 arm_conv_partial_fast_q31] a window outside [0, srcALen+srcBLen-2] returns ARM_MATH_ARGUMENT_ERROR, the last valid windows succeed") { partialErrorTest(true); }
TEST_CASE("characterization: [C2 arm_conv_partial_fast_q31] the window is written at pDst[firstIndex ..] and nothing before it (first 0, 3, 7, 8, non-negative data)") { partialPlacementCharacterization(true); }
TEST_CASE("characterization: [C4 arm_conv_partial_fast_q31] on ARM_MATH_ARGUMENT_ERROR nothing is written to the output") { partialErrorWritesNothing(true); }

// ---------------------------------------------------------------------------------------------------------------
// arm_levinson_durbin_q31
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_levinson_durbin_q31] ARM's three tests (n = 3, 8, 11) reach SNR 100 and 30 LSB on the coefficients and 30 LSB on the error" * UB_LD) { armLdTest(); }

// n = 1: phi = [0.5, 0.25]: a0 = phi1 / phi0 = 0.5, err = phi0 - phi1 a0 = 0.5 - 0.125 = 0.375
TEST_CASE("[C3 arm_levinson_durbin_q31] n = 1 with phi = [0.5, 0.25] gives a = [0.5] and error 0.375 (hand derived)") {
	ldNear(runLd({q31of(0.5), q31of(0.25)}, 1), {0.5}, 0.375);
}
// n = 2: phi = [0.5, 0.25, 0]. a0 = 0.5, e = 0.375; p = 1: suma = sumb = a0 phi1 = 0.125, k = (phi2 - suma) / (phi0 - sumb) = -0.125 / 0.375 = -1/3,
// a0' = a0 - k a0 = 2/3, a1 = k = -1/3, e = 0.375 (1 - 1/9) = 1/3. Check of Yule-Walker: 0.5 (2/3) + 0.25 (-1/3) = 0.25 = phi1; 0.25 (2/3) + 0.5 (-1/3) = 0 = phi2.
TEST_CASE("[C3 arm_levinson_durbin_q31] n = 2 with phi = [0.5, 0.25, 0] gives a = [2/3, -1/3] and error 1/3 (hand derived, Yule-Walker checked)" * UB_LD) {
	ldNear(runLd({q31of(0.5), q31of(0.25), 0}, 2), {2.0 / 3.0, -1.0 / 3.0}, 1.0 / 3.0);
}
// AR(1): phi[k] = 0.75 * 0.5^k = [0.75, 0.375, 0.1875, 0.09375]: a = [0.5, 0, 0], err = phi0 (1 - 0.25) = 0.5625 (k2 = k3 = 0)
TEST_CASE("[C3 arm_levinson_durbin_q31] n = 3 with the autocovariance of an AR(1) process (0.75 * 0.5^k) gives a = [0.5, 0, 0] and error 0.5625") {
	ldNear(runLd({q31of(0.75), q31of(0.375), q31of(0.1875), q31of(0.09375)}, 3), {0.5, 0, 0}, 0.5625);
}
// MA(3) autocovariances (phi[k] = sum h[n] h[n+k], zero beyond lag 3: positive definite), n = 1..8 against the double algorithm
TEST_CASE("[C3 arm_levinson_durbin_q31] random positive definite autocovariances (MA(3), n = 1 to 8) agree with the double algorithm within 30 LSB" * UB_LD) {
	Rng rng(11000);
	size_t used = 0;
	for (int rep = 0; rep < 12; rep++) {
		double h[4];
		h[0] = 0.5;
		for (int i = 1; i < 4; i++) {
			const int64_t r = rng.range(-300, 300);
			h[i] = static_cast<double>(r) / 1000.0;
		}
		for (int n = 1; n <= 8; n++) {
			std::vector<double> phi(static_cast<size_t>(n) + 1, 0.0);
			for (int k = 0; k <= std::min(n, 3); k++)
				for (int i = 0; i + k < 4; i++) phi[static_cast<size_t>(k)] += h[i] * h[i + k];
			V q(phi.size());
			std::vector<double> pq(phi.size());
			for (size_t i = 0; i < phi.size(); i++) {
				q[i] = q31of(phi[i]);
				pq[i] = static_cast<double>(q[i]) / 2147483648.0;
			}
			std::vector<double> a;
			double e = 0;
			defLd(pq, n, a, e);
			bool inRange = true;
			for (double v : a) inRange = inRange && std::fabs(v) < 0.95;
			if (!inRange) continue;
			used++;
			ldNear(runLd(q, n), a, e);
		}
	}
	CHECK(used > 60);
}
}  // TEST_SUITE

#endif  // __SIZEOF_INT128__
