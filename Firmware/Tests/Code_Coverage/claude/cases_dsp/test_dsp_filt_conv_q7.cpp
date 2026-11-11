/*
 * CMSIS-DSP Filtering, convolution / correlation q7 (work order CVQ7, phase P11 group G5): arm_conv_q7, arm_conv_opt_q7,
 * arm_conv_partial_q7, arm_conv_partial_opt_q7, arm_correlate_q7, arm_correlate_opt_q7.
 * Requirement ids: Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * Pattern files (Libraries/CMSIS-DSP/Testing/Patterns/DSP/Filtering/MISC/MISCQ7/), suite "MISC Q7" (Testing/desc.txt:3013-3146), ARM
 * test class Testing/Source/Tests/MISCQ7.cpp. ARM ids (56, none disabled): arm_correlate_q7 x25 (nba x nbb = {30,31,32,33,48} x
 * {31,32,33,34,49}, refs Reference1..25), arm_conv_q7 x25 (same sizes, Reference26..50), arm_conv_partial_q7 x3 and
 * arm_conv_partial_opt_q7 x3 (nba 6, nbb 8, first 3 / 9 / 7, NBPOINTS 4, inputs InputsA2 / InputsB2, Reference54 / 55 / 56 =
 * the window [first, first+4) of the full convolution). Inputs are the first nba / nbb values of InputsA1 / InputsB1 (128 values).
 * ARM has NO id for arm_conv_opt_q7 and arm_correlate_opt_q7: here they are run on the same 25 configs with the same tolerances.
 * ARM tolerances (used unchanged, L4): SNR_THRESHOLD 15 (MISCQ7.cpp:8), ABS_ERROR_Q7 5 and ABS_ERROR_FAST_Q7 5 (:15, :16).
 * ASSERT_NEAR_EQ = |out - ref| <= 5 per sample (withinLsb), ASSERT_SNR = dsp::snr (double). ARM's correlate test runs on an output
 * buffer that is zero (the library does not write the zero padding, see characterization below), its conv test also checks the tail
 * (here: a guard behind every output, GuardedBuffer).
 * ARM reference of the correlation (Convolutions.py): r = np.correlate(a, b, "full") i.e. r[i] = sum_n a[n + i - (B-1)] b[n],
 * i = 0 .. A+B-2; if A > B then A-B zeros are put BEFORE, if A < B then B-A zeros AFTER (total length 2 max(A,B) - 1, the DOC length).
 *
 * DOC (arm_conv_q7.c, arm_conv_opt_q7.c, arm_correlate_q7.c, arm_correlate_opt_q7.c; arm_conv_partial*_q7.c):
 *   conv: y[n] = sum_k a[k] b[n-k], length A + B - 1. 32 bit accumulator 18.14 (products 2.14), "truncated to 18.7 format by discarding
 *   the low 7 bits and then saturated to 1.7 format": y = sat8(floor(acc / 2^7)). Same wording for the correlations (output length
 *   2 max(A,B) - 1). Scratch of the opt variants (q15): pScratch1 max(A,B) + 2 min(A,B) - 2 elements, pScratch2 min(A,B) elements: every
 *   test allocates exactly this on the heap (new[]), and the inputs are tight heap copies too, so ASan sees every overrun.
 *   partial: "firstIndex is the first output sample to start with, numPoints the number of output points", return
 *   ARM_MATH_ARGUMENT_ERROR "requested subset is not in the range [0 srcALen+srcBLen-2]" (the code: firstIndex + numPoints >
 *   A + B - 1). The DOC does not say where the output is written; ARM's test (MISCQ7.cpp:79) reads outp[first], and the code writes
 *   pDst[firstIndex .. firstIndex + numPoints - 1] (so pDst has the layout of the full result). The tests follow ARM's test. The
 *   partial DOC has no "Scaling" paragraph.
 * DEF: exact integer model (int64: |acc| <= 2^31 for the sizes used), floor division by 128 and saturation to [-128, 127].
 *
 * Host vs target (C6): the host runs the portable C path (no ARM_MATH_DSP, no MVE; ARM_MATH_LOOPUNROLL on). The host build of
 * arm_conv_partial_q7 therefore compiles the "alternate version for CM0_FAMILY" (arm_conv_partial_q7.c:711-756, a plain double loop
 * on pDst[i]), while a Cortex-M4 build (ARM_MATH_DSP) uses the unrolled version (:70-709), which the host tests do NOT cover.
 * conv / correlate (+opt) use the DSP style kernels with the C intrinsics of dsp/none.h on the host and __SMLAD etc. on the target:
 * results are equal when nothing overflows. Left shifts of negative values (UB in C, defined result on ARM and x86) are listed per
 * test: see UB notes below.
 *
 * Only the non ARM_MATH_DSP code of arm_conv_q7.c and arm_correlate_q7.c is compiled on the host (the `#if (1)` DSP style block with the C
 * intrinsics of dsp/none.h); the unrolled ARM_MATH_DSP / Cortex-M4 target code (SMLAD intrinsics) and the MVE code are NOT covered by these tests.
 *
 * Mutation limits: the unrolled accumulator wiring of the plain kernels (e.g. conv_q7 unrolled acc3 store, correlate_q7 unrolled acc1 store) is
 * only observable with data that makes dsp/none.h:460 shift a packed pair with a non zero upper half (B-DCVQ7-1): no UB free input reaches those
 * stores with non zero results, so the exact-model cases are KNOWN_ISSUE_UB_HIDDEN (skipped in sanitizer builds).
 *
 * NEEDS REVIEW / not tested:
 *   - arm_correlate_*: DOC length "2 max(A,B) - 1" but the (A-B) leading zeros (A > B) or (B-A) trailing zeros (A < B) are NOT written by
 *     the library (the code skips them: pOut += j). characterization test below; the DOC does not say whether they are written.
 *   - arm_conv_partial_q7 / _opt_q7 with numPoints = 0: the host (CM0 style) loop "i <= firstIndex + numPoints - 1" wraps for firstIndex = 0
 *     (unsigned) and would run away: not called (needs a lead decision; DOC silent).
 *   - firstIndex + numPoints overflowing uint32_t passes the range check: not called.
 *   - blockSize / length 0 for conv / correlate: kernel arithmetic (srcBLen - 1U) wraps: not called.
 *   - partial DOC silent on saturation: characterization test.
 *   - arm_conv_partial_q7 unrolled version (Cortex-M4 build) is not reachable on the host.
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

typedef std::vector<int8_t> V8;
typedef std::vector<int64_t> V64;

// ---------------------------------------------------------------------------------------------------------------
// Deterministic data. Every call is its own statement (lesson: never two PRNG calls in one expression).
// ---------------------------------------------------------------------------------------------------------------
struct Rng {
	uint64_t s;
	explicit Rng(uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ull + 0x1234567ull) {
		if (s == 0) s = 1;
	}
	uint32_t next32() {
		s ^= s << 13;
		s ^= s >> 7;
		s ^= s << 17;
		return static_cast<uint32_t>(s >> 32);
	}
	int64_t range(int64_t lo, int64_t hi) {
		const uint64_t span = static_cast<uint64_t>(hi - lo) + 1u;
		const uint32_t r = next32();
		return lo + static_cast<int64_t>((static_cast<uint64_t>(r) * span) >> 32);
	}
};

V8 randVec(Rng &rng, size_t n, int lo, int hi) {
	V8 v(n);
	for (size_t i = 0; i < n; i++) {
		const int64_t x = rng.range(lo, hi);
		v[i] = static_cast<int8_t>(x);
	}
	return v;
}

// ---------------------------------------------------------------------------------------------------------------
// DEF
// ---------------------------------------------------------------------------------------------------------------
enum Op { CONV, CORR };
struct Fn {
	const char *name;
	Op op;
	bool opt;
};
const Fn kConv = {"arm_conv_q7", CONV, false};
const Fn kConvOpt = {"arm_conv_opt_q7", CONV, true};
const Fn kCorr = {"arm_correlate_q7", CORR, false};
const Fn kCorrOpt = {"arm_correlate_opt_q7", CORR, true};

size_t fullLen(Op op, size_t A, size_t B) { return op == CONV ? A + B - 1 : 2 * std::max(A, B) - 1; }

int64_t floorDiv128(int64_t v) {
	int64_t q = v / 128;  // truncates toward zero
	if (v % 128 < 0) q--;
	return q;
}
int8_t sat8(int64_t acc) {
	int64_t y = floorDiv128(acc);
	if (y > 127) y = 127;
	if (y < -128) y = -128;
	return static_cast<int8_t>(y);
}

// y[n] = sum a[k] b[n-k]
V64 convAcc(const V8 &a, const V8 &b) {
	V64 r(a.size() + b.size() - 1, 0);
	for (size_t i = 0; i < a.size(); i++)
		for (size_t j = 0; j < b.size(); j++) r[i + j] += static_cast<int64_t>(a[i]) * b[j];
	return r;
}

// np.correlate(a, b, "full")[i] = sum_n a[n + i - (B-1)] b[n], then zero padding (ARM's Convolutions.py): A > B zeros before, A < B after
V64 corrAcc(const V8 &a, const V8 &b) {
	const int64_t A = static_cast<int64_t>(a.size()), B = static_cast<int64_t>(b.size());
	V64 full(static_cast<size_t>(A + B - 1), 0);
	for (int64_t i = 0; i < A + B - 1; i++)
		for (int64_t n = 0; n < B; n++) {
			const int64_t k = n + i - (B - 1);
			if (k >= 0 && k < A) full[static_cast<size_t>(i)] += static_cast<int64_t>(a[static_cast<size_t>(k)]) * b[static_cast<size_t>(n)];
		}
	V64 r;
	if (A > B) r.assign(static_cast<size_t>(A - B), 0);
	r.insert(r.end(), full.begin(), full.end());
	if (A < B) r.insert(r.end(), static_cast<size_t>(B - A), 0);
	return r;
}

V64 modelAcc(Op op, const V8 &a, const V8 &b) { return op == CONV ? convAcc(a, b) : corrAcc(a, b); }
V8 modelOut(Op op, const V8 &a, const V8 &b) {
	const V64 acc = modelAcc(op, a, b);
	V8 y(acc.size());
	for (size_t i = 0; i < acc.size(); i++) y[i] = sat8(acc[i]);
	return y;
}

// ---------------------------------------------------------------------------------------------------------------
// Runners: tight heap inputs, tight scratch, guarded output
// ---------------------------------------------------------------------------------------------------------------
struct Run {
	V8 out;
	bool guardOk = true;
	bool inputsOk = true;
};

size_t scratch1Len(size_t A, size_t B) { return std::max(A, B) + 2 * std::min(A, B) - 2; }
size_t scratch2Len(size_t A, size_t B) { return std::min(A, B); }

void callFull(const Fn &f, const int8_t *a, size_t A, const int8_t *b, size_t B, int8_t *out, int16_t *s1, int16_t *s2) {
	if (f.op == CONV) {
		if (f.opt) arm_conv_opt_q7(a, static_cast<uint32_t>(A), b, static_cast<uint32_t>(B), out, s1, s2);
		else arm_conv_q7(a, static_cast<uint32_t>(A), b, static_cast<uint32_t>(B), out);
	} else {
		if (f.opt) arm_correlate_opt_q7(a, static_cast<uint32_t>(A), b, static_cast<uint32_t>(B), out, s1, s2);
		else arm_correlate_q7(a, static_cast<uint32_t>(A), b, static_cast<uint32_t>(B), out);
	}
}

// zeroOut: output part zero before the call (like ARM's output buffer); else poisoned by GuardedBuffer (0xCD)
Run runFull(const Fn &f, const V8 &a, const V8 &b, bool zeroOut = true, bool tightOut = false) {
	const size_t A = a.size(), B = b.size();
	Run r;
	const size_t n = fullLen(f.op, A, B);
	dsp::GuardedBuffer<int8_t> g(n);
	if (zeroOut) std::memset(g.data(), 0, n);
	std::unique_ptr<int8_t[]> pa(new int8_t[A]), pb(new int8_t[B]);
	std::copy(a.begin(), a.end(), pa.get());
	std::copy(b.begin(), b.end(), pb.get());
	std::unique_ptr<int16_t[]> s1, s2;
	if (f.opt) {
		s1.reset(new int16_t[scratch1Len(A, B)]);  // exactly the DOC size
		s2.reset(new int16_t[scratch2Len(A, B)]);
		for (size_t i = 0; i < scratch1Len(A, B); i++) s1[i] = 0x5A5A;
		for (size_t i = 0; i < scratch2Len(A, B); i++) s2[i] = 0x5A5A;
	}
	if (tightOut) {
		// exact size heap output WITHOUT guard words: ASan sees every write past the DOC length (the guard words of GuardedBuffer share
		// the allocation of the output and hide small overruns)
		std::unique_ptr<int8_t[]> t(new int8_t[n]);
		std::memset(t.get(), zeroOut ? 0 : 0xCD, n);
		callFull(f, pa.get(), A, pb.get(), B, t.get(), s1.get(), s2.get());
		r.out.assign(t.get(), t.get() + n);
	} else {
		callFull(f, pa.get(), A, pb.get(), B, g.data(), s1.get(), s2.get());
		r.out = g.output();
		r.guardOk = g.guardIntact();
	}
	r.inputsOk = std::equal(a.begin(), a.end(), pa.get()) && std::equal(b.begin(), b.end(), pb.get());
	return r;
}

struct PRun {
	arm_status st = ARM_MATH_SUCCESS;
	V8 out;  // first + numPoints elements (layout of the full result)
	bool guardOk = true;
};

PRun runPartial(bool opt, const V8 &a, const V8 &b, size_t first, size_t num, size_t outLen, bool tightOut = false) {
	const size_t A = a.size(), B = b.size();
	PRun r;
	dsp::GuardedBuffer<int8_t> g(outLen);
	std::unique_ptr<int8_t[]> tight;
	if (tightOut) {
		tight.reset(new int8_t[outLen]);  // exact size, no guard words (see runFull)
		std::memset(tight.get(), 0xCD, outLen);
	}
	int8_t *dst = tightOut ? tight.get() : g.data();
	std::unique_ptr<int8_t[]> pa(new int8_t[A]), pb(new int8_t[B]);
	std::copy(a.begin(), a.end(), pa.get());
	std::copy(b.begin(), b.end(), pb.get());
	std::unique_ptr<int16_t[]> s1, s2;
	if (opt) {
		s1.reset(new int16_t[scratch1Len(A, B)]);
		s2.reset(new int16_t[scratch2Len(A, B)]);
		for (size_t i = 0; i < scratch1Len(A, B); i++) s1[i] = 0x5A5A;
		for (size_t i = 0; i < scratch2Len(A, B); i++) s2[i] = 0x5A5A;
		r.st = arm_conv_partial_opt_q7(pa.get(), static_cast<uint32_t>(A), pb.get(), static_cast<uint32_t>(B), dst,
		                               static_cast<uint32_t>(first), static_cast<uint32_t>(num), s1.get(), s2.get());
	} else {
		r.st = arm_conv_partial_q7(pa.get(), static_cast<uint32_t>(A), pb.get(), static_cast<uint32_t>(B), dst,
		                           static_cast<uint32_t>(first), static_cast<uint32_t>(num));
	}
	if (tightOut) {
		r.out.assign(tight.get(), tight.get() + outLen);
	} else {
		r.out = g.output();
		r.guardOk = g.guardIntact();
	}
	return r;
}

std::string cfg(size_t A, size_t B) { return "A=" + std::to_string(A) + " B=" + std::to_string(B); }

V8 pat(const std::string &n) { return dsp::loadPattern("Filtering/MISC/MISCQ7/" + n).q7(); }
V8 head(const V8 &v, size_t n) { return V8(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(n)); }

const size_t kArmA[] = {30, 31, 32, 33, 48};
const size_t kArmB[] = {31, 32, 33, 34, 49};

// largest |x| so that min(A,B) products of |x| * |x| stay below 2^14 (no clipping, no 8 bit overflow)
int cleanLim(size_t A, size_t B) {
	const size_t m = std::min(A, B);
	int lim = 127;
	while (static_cast<size_t>(lim) * static_cast<size_t>(lim) * m >= 16384u) lim--;
	return lim;
}

// Sizes whose execution is free of undefined behaviour on the host (found with UBSan --no-skip, see the header): the DSP style
// kernels shift packed pairs / negative values left once the unrolled path (4 samples) is reached.
//   plain variants: the unrolled loops need a count >= 4, i.e. min(A, B) >= 4;
//   opt variants: SMLAD on packed pairs needs min(A, B) >= 2 (any data), the 4 output block (read_q15x2 of negative values, __PACKq7 of
//   negative outputs) needs A + B - 1 >= 4 with negative data.
bool isClean(const Fn &f, size_t A, size_t B, bool neg) {
	const size_t mn = std::min(A, B);
	if (!f.opt) return mn <= 3;
	if (neg) return A + B - 1 < 4 && mn == 1;
	return mn == 1;
}

// ---------------------------------------------------------------------------------------------------------------
// C1: ARM pattern tests
// ---------------------------------------------------------------------------------------------------------------
void armFullPatternTest(const Fn &f) {
	const V8 ia = pat("InputsA1_q7.txt"), ib = pat("InputsB1_q7.txt");
	REQUIRE(ia.size() == 128);
	REQUIRE(ib.size() == 128);
	int id = f.op == CORR ? 1 : 26;
	for (size_t A : kArmA)
		for (size_t B : kArmB) {
			const V8 ref = pat("Reference" + std::to_string(id) + "_q7.txt");
			INFO(cfg(A, B) << " Reference" << id);
			const Run r = runFull(f, head(ia, A), head(ib, B));
			REQUIRE(r.out.size() == ref.size());
			CHECK(r.guardOk);
			const double snr = dsp::snr(ref, r.out);
			CHECK_MESSAGE(snr >= 15.0, "SNR " << snr);  // MISCQ7.cpp:8
			const dsp::Check c = dsp::withinLsb(ref, r.out, 5);  // MISCQ7.cpp:15
			CHECK_MESSAGE(c, c.message);
			id++;
		}
}

void armPartialPatternTest(bool opt) {
	const V8 ia = pat("InputsA2_q7.txt"), ib = pat("InputsB2_q7.txt");
	REQUIRE(ia.size() == 128);
	const size_t firsts[3] = {3, 9, 7};
	const int refIds[3] = {54, 55, 56};
	for (int t = 0; t < 3; t++) {
		const size_t first = firsts[t];
		const V8 ref = pat("Reference" + std::to_string(refIds[t]) + "_q7.txt");
		REQUIRE(ref.size() == 4);
		INFO("first " << first);
		const PRun r = runPartial(opt, head(ia, 6), head(ib, 8), first, 4, first + 4);
		CHECK(r.st == ARM_MATH_SUCCESS);
		CHECK(r.guardOk);
		const V8 win(r.out.begin() + static_cast<std::ptrdiff_t>(first), r.out.end());
		const double snr = dsp::snr(ref, win);
		CHECK_MESSAGE(snr >= 15.0, "SNR " << snr);
		const dsp::Check c = dsp::withinLsb(ref, win, 5);  // :15 / :16
		CHECK_MESSAGE(c, c.message);
	}
}

// ---------------------------------------------------------------------------------------------------------------
// C3 / C2: exact model
// ---------------------------------------------------------------------------------------------------------------
void compareModel(const Fn &f, const V8 &a, const V8 &b, const std::string &what) {
	const Run r = runFull(f, a, b);
	const V8 want = modelOut(f.op, a, b);
	CHECK_MESSAGE(r.guardOk, what << ": guard damaged");
	CHECK_MESSAGE(r.inputsOk, what << ": input modified");
	REQUIRE(r.out.size() == want.size());
	const dsp::Check c = dsp::exactEqual(want, r.out);
	CHECK_MESSAGE(c, what << ": " << c.message);
}

// all lengths 1..9 x 1..9 (A >= B, A < B, the unroll tails), full range data (saturation included: DOC "saturated")
void sizesSweepTest(const Fn &f, bool allowNeg, bool cleanOnly = false) {
	for (uint64_t seed = 1; seed <= 3; seed++) {
		Rng rng(seed + 100);
		for (size_t A = 1; A <= 9; A++)
			for (size_t B = 1; B <= 9; B++) {
				const V8 a = randVec(rng, A, allowNeg ? -128 : 0, 127);
				const V8 b = randVec(rng, B, allowNeg ? -128 : 0, 127);
				if (cleanOnly && !isClean(f, A, B, allowNeg)) continue;
				compareModel(f, a, b, cfg(A, B) + " seed " + std::to_string(seed));
			}
	}
}

void bigSizesTest(const Fn &f, bool allowNeg) {
	const size_t lens[] = {10, 11, 12, 13, 15, 16, 17, 20, 31, 32, 33, 48, 49, 64};
	Rng rng(7);
	for (size_t A : lens)
		for (size_t B : lens) {
			const V8 a = randVec(rng, A, allowNeg ? -128 : 0, 127);
			const V8 b = randVec(rng, B, allowNeg ? -128 : 0, 127);
			compareModel(f, a, b, cfg(A, B));
		}
}

// DOC "no risk of overflow": longest input 4096 with large values still equals the 64 bit model
void floorErrorTest(const Fn &f, bool allowNeg, bool cleanOnly = false) {
	Rng rng(11);
	for (size_t A = 1; A <= 20; A++)
		for (size_t B = 1; B <= 20; B++) {
			const int lim = cleanLim(A, B);
			const V8 a = randVec(rng, A, allowNeg ? -lim : 0, lim);
			const V8 b = randVec(rng, B, allowNeg ? -lim : 0, lim);
			if (cleanOnly && !isClean(f, A, B, allowNeg)) continue;
			const Run r = runFull(f, a, b);
			// independent double: exact (|terms| < 2^14, sums < 2^20)
			std::vector<double> d(r.out.size(), 0.0);
			const std::vector<int64_t> acc = modelAcc(f.op, a, b);
			for (size_t i = 0; i < acc.size(); i++) d[i] = static_cast<double>(acc[i]) / 128.0;
			for (size_t i = 0; i < d.size(); i++) {
				// floor: 0 <= acc/128 - y < 1 (strict)
				const double e = d[i] - static_cast<double>(r.out[i]);
				CHECK_MESSAGE(e >= 0.0, cfg(A, B) << " i " << i << " error " << e);
				CHECK_MESSAGE(e < 1.0, cfg(A, B) << " i " << i << " error " << e);
			}
		}
}

// hand values: a = {64} (0.5), b = x: y = floor(64 x / 128) = floor(x / 2)
// x = {1,-1,2,-2,100,-100,127,-128} -> {0,-1,1,-1,50,-50,63,-64}
void impulseTest(const Fn &f, size_t L) {
	const V8 x = head({1, -1, 2, -2, 100, -100, 127, -128}, L);
	const V8 h = head({0, -1, 1, -1, 50, -50, 63, -64}, L);
	const V8 one = {64};
	if (f.op == CONV) {
		const Run r1 = runFull(f, one, x);
		CHECK(r1.guardOk);
		CHECK(r1.out == h);
		const Run r2 = runFull(f, x, one);
		CHECK(r2.guardOk);
		CHECK(r2.out == h);
	} else {
		// a = {64}, b = x (A < B): r[i] = 64 b[L-1-i] for i < L then L-1 zeros
		V8 want;
		for (size_t i = 0; i < L; i++) want.push_back(h[L - 1 - i]);
		want.insert(want.end(), L - 1, 0);
		const Run r1 = runFull(f, one, x);
		CHECK(r1.guardOk);
		CHECK(r1.out == want);
		// a = x, b = {64} (A > B): L-1 zeros, then r[i'] = a[i'] 64 / 128
		V8 want2(L - 1, 0);
		want2.insert(want2.end(), h.begin(), h.end());
		const Run r2 = runFull(f, x, one);
		CHECK(r2.guardOk);
		CHECK(r2.out == want2);
	}
}

// all 65536 (a, b) pairs, one value each: y = sat8(floor(a b / 128)); the pair (-128, -128) gives 128 -> 127 ("saturated")
void oneTapExhaustiveTest(const Fn &f) {
	int8_t s1[1], s2[1], out[1];
	size_t bad = 0;
	int firstA = 0, firstB = 0;
	for (int x = -128; x <= 127; x++)
		for (int y = -128; y <= 127; y++) {
			const int8_t a = static_cast<int8_t>(x), b = static_cast<int8_t>(y);
			int16_t t1[1] = {0}, t2[1] = {0};
			out[0] = 0x55;
			callFull(f, &a, 1, &b, 1, out, t1, t2);
			const int8_t want = sat8(static_cast<int64_t>(x) * y);
			if (out[0] != want) {
				if (bad == 0) {
					firstA = x;
					firstB = y;
				}
				bad++;
			}
		}
	(void)s1;
	(void)s2;
	CHECK_MESSAGE(bad == 0, bad << " pairs differ, first (" << firstA << ", " << firstB << ")");
}

// two values x coefficient c: all 65536 (x0, x1) pairs, A = 2 with B = 1 and A = 1 with B = 2, for three coefficients
void twoTapExhaustiveTest(const Fn &f) {
	const int cs[] = {127, -128, -37};
	for (int c : cs) {
		size_t bad = 0;
		int fx0 = 0, fx1 = 0;
		for (int x0 = -128; x0 <= 127; x0++)
			for (int x1 = -128; x1 <= 127; x1++) {
				const int8_t x[2] = {static_cast<int8_t>(x0), static_cast<int8_t>(x1)};
				const int8_t cc[1] = {static_cast<int8_t>(c)};
				int16_t t1[4] = {0, 0, 0, 0}, t2[2] = {0, 0};
				for (int swap = 0; swap < 2; swap++) {
					int8_t out[3] = {0, 0, 0};
					if (swap == 0) callFull(f, x, 2, cc, 1, out, t1, t2);
					else callFull(f, cc, 1, x, 2, out, t1, t2);
					V8 a, b;
					if (swap == 0) {
						a = V8(x, x + 2);
						b = V8(cc, cc + 1);
					} else {
						a = V8(cc, cc + 1);
						b = V8(x, x + 2);
					}
					const V8 want = modelOut(f.op, a, b);
					for (size_t i = 0; i < want.size(); i++)
						if (out[i] != want[i]) {
							if (bad == 0) {
								fx0 = x0;
								fx1 = x1;
							}
							bad++;
						}
				}
			}
		CHECK_MESSAGE(bad == 0, "c " << c << ": " << bad << " outputs differ, first (" << fx0 << ", " << fx1 << ")");
	}
}

// DOC "saturated to 1.7": all inputs -128 -> every product 16384 -> every nonzero output 127;
// a = 127, b = -128: product -16256 = -127 * 128: first output -127, k terms: -127 k -> -128 for k >= 2
void saturationTest(const Fn &f, bool cleanOnly = false) {
	for (size_t A = 1; A <= 9; A++)
		for (size_t B = 1; B <= 9; B++) {
			if (cleanOnly && !isClean(f, A, B, true)) continue;
			INFO(cfg(A, B));
			{
				const Run r = runFull(f, V8(A, -128), V8(B, -128));
				CHECK(r.guardOk);
				const V64 acc = modelAcc(f.op, V8(A, -128), V8(B, -128));
				for (size_t i = 0; i < acc.size(); i++) {
					const int8_t want = acc[i] == 0 ? 0 : 127;  // padding samples are 0
					CHECK_MESSAGE(r.out[i] == want, "i " << i);
				}
			}
			{
				const Run r = runFull(f, V8(A, 127), V8(B, -128));
				CHECK(r.guardOk);
				const V64 acc = modelAcc(f.op, V8(A, 127), V8(B, -128));
				for (size_t i = 0; i < acc.size(); i++) {
					if (acc[i] == 0) {
						CHECK_MESSAGE(r.out[i] == 0, "i " << i);
					} else {
						// acc = -16256 k: k = 1 -> -127, k >= 2 -> -128 (hand derived)
						const int64_t k = -acc[i] / 16256;
						const int8_t want = k == 1 ? -127 : -128;
						CHECK_MESSAGE(r.out[i] == want, "i " << i << " k " << k);
					}
				}
			}
		}
}

// conv is commutative: the integer sums are identical, so bitwise equal outputs for (a, b) and (b, a)
void commutativeTest(const Fn &f, bool allowNeg, bool cleanOnly = false) {
	Rng rng(21);
	for (size_t A = 1; A <= 12; A++)
		for (size_t B = 1; B <= 12; B++) {
			const V8 a = randVec(rng, A, allowNeg ? -128 : 0, 127);
			const V8 b = randVec(rng, B, allowNeg ? -128 : 0, 127);
			if (cleanOnly && !isClean(f, A, B, allowNeg)) continue;
			const Run r1 = runFull(f, a, b), r2 = runFull(f, b, a);
			CHECK_MESSAGE(r1.out == r2.out, cfg(A, B));
		}
}

// correlate(a, b) is the reverse of correlate(b, a) over the whole 2 max - 1 buffer (the code's own comment, same integer sums)
void reverseTest(const Fn &f, bool allowNeg, bool cleanOnly = false) {
	Rng rng(22);
	for (size_t A = 1; A <= 12; A++)
		for (size_t B = 1; B <= 12; B++) {
			const V8 a = randVec(rng, A, allowNeg ? -128 : 0, 127);
			const V8 b = randVec(rng, B, allowNeg ? -128 : 0, 127);
			if (cleanOnly && !isClean(f, A, B, allowNeg)) continue;
			const Run r1 = runFull(f, a, b), r2 = runFull(f, b, a);
			V8 rev(r2.out.rbegin(), r2.out.rend());
			CHECK_MESSAGE(r1.out == rev, cfg(A, B));
		}
}

// opt variant equals the plain variant bitwise (identical integer computation)
void optEqualsPlainTest(Op op, bool allowNeg) {
	Rng rng(31);
	const Fn &p = op == CONV ? kConv : kCorr;
	const Fn &o = op == CONV ? kConvOpt : kCorrOpt;
	for (size_t A = 1; A <= 40; A += (A < 12 ? 1 : 7))
		for (size_t B = 1; B <= 40; B += (B < 12 ? 1 : 5)) {
			const V8 a = randVec(rng, A, allowNeg ? -128 : 0, 127);
			const V8 b = randVec(rng, B, allowNeg ? -128 : 0, 127);
			const Run r1 = runFull(p, a, b), r2 = runFull(o, a, b);
			CHECK_MESSAGE(r1.out == r2.out, cfg(A, B));
		}
}

void repeatTest(const Fn &f, bool allowNeg, size_t A = 13, size_t B = 6) {
	Rng rng(41);
	const V8 a = randVec(rng, A, allowNeg ? -128 : 0, 127);
	const V8 b = randVec(rng, B, allowNeg ? -128 : 0, 127);
	const Run r1 = runFull(f, a, b), r2 = runFull(f, a, b);
	CHECK(r1.out == r2.out);
	CHECK(r1.inputsOk);
}

// ---------------------------------------------------------------------------------------------------------------
// Partial convolution
// ---------------------------------------------------------------------------------------------------------------
// every window [first, first + num) of the full result, A, B in 1..9, non clipping data (the DOC has no scaling paragraph)
void partialWindowsTest(bool opt, bool allowNeg, bool cleanOnly = false) {
	Rng rng(51);
	for (size_t A = 1; A <= 9; A++)
		for (size_t B = 1; B <= 9; B++) {
			const int lim = cleanLim(A, B);
			const V8 a = randVec(rng, A, allowNeg ? -lim : 0, lim);
			const V8 b = randVec(rng, B, allowNeg ? -lim : 0, lim);
			const V8 full = modelOut(CONV, a, b);
			const size_t L = A + B - 1;
			for (size_t first = 0; first < L; first++)
				for (size_t num = 1; first + num <= L; num++) {
					if (cleanOnly && opt && !(std::min(A, B) == 1 && (num < 4 || !allowNeg))) continue;
					const PRun r = runPartial(opt, a, b, first, num, first + num);
					INFO(cfg(A, B) << " first " << first << " num " << num);
					REQUIRE(r.st == ARM_MATH_SUCCESS);
					CHECK(r.guardOk);
					bool ok = true;
					for (size_t i = first; i < first + num; i++) ok = ok && r.out[i] == full[i];
					CHECK(ok);
					// nothing before the window is written (poison 0xCD = -51)
					bool untouched = true;
					for (size_t i = 0; i < first; i++) untouched = untouched && r.out[i] == static_cast<int8_t>(0xCD);
					CHECK(untouched);
				}
		}
}

void partialBigTest(bool opt, bool allowNeg) {
	Rng rng(52);
	const size_t lens[] = {10, 15, 16, 17, 33, 48};
	for (size_t A : lens)
		for (size_t B : lens) {
			const int lim = cleanLim(A, B);
			const V8 a = randVec(rng, A, allowNeg ? -lim : 0, lim);
			const V8 b = randVec(rng, B, allowNeg ? -lim : 0, lim);
			const V8 full = modelOut(CONV, a, b);
			const size_t L = A + B - 1;
			for (int t = 0; t < 12; t++) {
				const size_t first = static_cast<size_t>(rng.range(0, static_cast<int64_t>(L) - 1));
				const size_t num = static_cast<size_t>(rng.range(1, static_cast<int64_t>(L - first)));
				const PRun r = runPartial(opt, a, b, first, num, first + num);
				INFO(cfg(A, B) << " first " << first << " num " << num);
				REQUIRE(r.st == ARM_MATH_SUCCESS);
				CHECK(r.guardOk);
				bool ok = true;
				for (size_t i = first; i < first + num; i++) ok = ok && r.out[i] == full[i];
				CHECK(ok);
			}
		}
}

// DOC: ARM_MATH_ARGUMENT_ERROR "requested subset is not in the range [0 srcALen+srcBLen-2]"; nothing is written
void partialErrorTest(bool opt) {
	const V8 a = {1, 2, 3, 4, 5, 6}, b = {7, 8, 9, 10, 11, 12, 13, 14};  // A + B - 1 = 13
	struct Q { size_t first, num; };
	const Q qs[] = {{0, 14}, {13, 1}, {12, 2}, {7, 7}, {3, 11}, {9, 5}, {14, 0}};
	for (const Q &q : qs) {
		INFO("first " << q.first << " num " << q.num);
		const PRun r = runPartial(opt, a, b, q.first, q.num, 16);
		if (q.first + q.num > 13) {
			CHECK(r.st == ARM_MATH_ARGUMENT_ERROR);
			bool untouched = true;
			for (size_t i = 0; i < 16; i++) untouched = untouched && r.out[i] == static_cast<int8_t>(0xCD);
			CHECK(untouched);
		}
		CHECK(r.guardOk);
	}
}

// the last valid windows succeed (b has one value so that the opt variant is free of undefined behaviour with non negative data)
void partialBoundaryTest(bool opt) {
	const V8 a = {1, 2, 3, 4, 5, 6}, b = {100};  // A + B - 1 = 6, y = floor(100 x / 128): {0,1,2,3,3,4}
	const V8 want = {0, 1, 2, 3, 3, 4};
	struct Q { size_t first, num; };
	const Q qs[] = {{0, 6}, {5, 1}, {2, 4}, {4, 2}};
	for (const Q &q : qs) {
		INFO("first " << q.first << " num " << q.num);
		const PRun r = runPartial(opt, a, b, q.first, q.num, q.first + q.num);
		CHECK(r.st == ARM_MATH_SUCCESS);
		CHECK(r.guardOk);
		for (size_t i = q.first; i < q.first + q.num; i++) CHECK_MESSAGE(r.out[i] == want[i], "i " << i);
	}
}

// sizes at which the opt variants read 2 bytes behind the DOC sized scratch1 (B-DCVQ7-5): min(A,B) a multiple of 4 and the other length 1 mod 4
bool overreadSize(size_t A, size_t B) {
	const size_t mn = std::min(A, B), mx = std::max(A, B);
	return mn % 4 == 0 && mx % 4 == 1;
}

// C2: exact size heap output (no guard words), UB free sizes, random data (negative and non negative), full results
void tightOutputTest(const Fn &f) {
	Rng rng(61);
	for (int neg = 0; neg < 2; neg++)
		for (size_t A = 1; A <= 9; A++)
			for (size_t B = 1; B <= 9; B++) {
				const V8 a = randVec(rng, A, neg ? -128 : 0, 127);
				const V8 b = randVec(rng, B, neg ? -128 : 0, 127);
				if (!isClean(f, A, B, neg != 0)) continue;
				const Run r = runFull(f, a, b, true, true);
				const V8 want = modelOut(f.op, a, b);
				REQUIRE(r.out.size() == want.size());
				const dsp::Check c = dsp::exactEqual(want, r.out);
				CHECK_MESSAGE(c, cfg(A, B) << " neg " << neg << ": " << c.message);
			}
}

// C2: all-zero data reaches the unrolled paths of every kernel without undefined behaviour (zero packed pairs shift to zero): every output is 0
// and tight inputs, scratch and (second run) a tight output are never overrun. It cannot detect wrong accumulator wiring (all sums are 0).
void zeroDataTest(const Fn &f) {
	const size_t lens[] = {4, 5, 7, 8, 9, 12, 13, 16, 17, 33};
	for (size_t A : lens)
		for (size_t B : lens) {
			if (f.opt && overreadSize(A, B)) continue;
			const V8 a(A, 0), b(B, 0);
			for (int tight = 0; tight < 2; tight++) {
				const Run r = runFull(f, a, b, true, tight != 0);
				INFO(cfg(A, B) << " tight " << tight);
				if (!tight) CHECK(r.guardOk);
				CHECK(r.out == V8(fullLen(f.op, A, B), 0));
			}
		}
}

void partialTightTest(bool opt) {
	Rng rng(62);
	for (int neg = 0; neg < 2; neg++)
		for (size_t A = 1; A <= 9; A++)
			for (size_t B = 1; B <= 9; B++) {
				const int lim = cleanLim(A, B);
				const V8 a = randVec(rng, A, neg ? -lim : 0, lim);
				const V8 b = randVec(rng, B, neg ? -lim : 0, lim);
				const V8 full = modelOut(CONV, a, b);
				const size_t L = A + B - 1;
				for (size_t first = 0; first < L; first += 2)
					for (size_t num = 1; first + num <= L; num += 2) {
						if (opt && !(std::min(A, B) == 1 && (num < 4 || !neg))) continue;
						const PRun r = runPartial(opt, a, b, first, num, first + num, true);
						INFO(cfg(A, B) << " first " << first << " num " << num << " neg " << neg);
						REQUIRE(r.st == ARM_MATH_SUCCESS);
						bool ok = true;
						for (size_t i = first; i < first + num; i++) ok = ok && r.out[i] == full[i];
						CHECK(ok);
					}
			}
}

void partialZeroDataTest(bool opt) {
	const size_t lens[] = {4, 5, 8, 9, 12, 13, 17};
	for (size_t A : lens)
		for (size_t B : lens) {
			if (opt && overreadSize(A, B)) continue;
			const V8 a(A, 0), b(B, 0);
			const size_t L = A + B - 1;
			const size_t firsts[3] = {0, L - 1, L / 3};
			for (size_t first : firsts) {
				const size_t num = std::min<size_t>(L - first, first == L - 1 ? 1 : 6);
				for (int tight = 0; tight < 2; tight++) {
					const PRun r = runPartial(opt, a, b, first, num, first + num, tight != 0);
					INFO(cfg(A, B) << " first " << first << " num " << num << " tight " << tight);
					REQUIRE(r.st == ARM_MATH_SUCCESS);
					if (!tight) CHECK(r.guardOk);
					bool ok = true;
					for (size_t i = first; i < first + num; i++) ok = ok && r.out[i] == 0;
					CHECK(ok);
				}
			}
		}
}

// B-DCVQ7-5: opt variants, DOC sized scratch1 (max + 2 min - 2), the sizes of overreadSize(): the intended result is the correct output
void scratchOverreadTest(const Fn &f) {
	const size_t pairs[][2] = {{4, 5}, {5, 4}, {8, 9}, {9, 8}, {12, 13}, {13, 12}, {16, 17}, {17, 16}};
	Rng rng(71);
	// all-zero data first: free of other undefined behaviour, so ASan reports the over-read (B-DCVQ7-5) before UBSan can stop the run
	for (const auto &p : pairs) compareModel(f, V8(p[0], 0), V8(p[1], 0), cfg(p[0], p[1]) + " zero data");
	for (const auto &p : pairs) {
		const V8 a = randVec(rng, p[0], 0, 127);
		const V8 b = randVec(rng, p[1], 0, 127);
		compareModel(f, a, b, cfg(p[0], p[1]));
	}
}

void partialScratchOverreadTest() {
	const size_t pairs[][2] = {{4, 5}, {5, 4}, {8, 9}, {9, 8}, {12, 13}, {13, 12}, {16, 17}, {17, 16}};
	Rng rng(72);
	for (const auto &p : pairs) {  // zero data first (see scratchOverreadTest)
		const PRun r = runPartial(true, V8(p[0], 0), V8(p[1], 0), 0, p[0] + p[1] - 1, p[0] + p[1] - 1, true);
		INFO(cfg(p[0], p[1]) << " zero data");
		REQUIRE(r.st == ARM_MATH_SUCCESS);
		CHECK(r.out == V8(p[0] + p[1] - 1, 0));
	}
	for (const auto &p : pairs) {
		const size_t A = p[0], B = p[1];
		const int lim = cleanLim(A, B);
		const V8 a = randVec(rng, A, 0, lim);
		const V8 b = randVec(rng, B, 0, lim);
		const V8 full = modelOut(CONV, a, b);
		const PRun r = runPartial(true, a, b, 0, A + B - 1, A + B - 1, true);
		INFO(cfg(A, B));
		REQUIRE(r.st == ARM_MATH_SUCCESS);
		CHECK(r.out == full);
	}
}


}  // namespace

TEST_SUITE("DSP Filtering q7") {

// Undefined behaviour of the portable path (UBSan --no-skip, intended result on the host; skipped in sanitizer builds). One id per defect:
//   B-DCVQ7-1  dsp/none.h:460 (__SMLAD, columns 36 and 63): "(q31_t)x << 16" shifts a packed pair (value above 2^15) left: "cannot be
//              represented in type 'int'"; reached by every kernel once a SMLAD on non zero data runs (also with non negative data).
//   B-DCVQ7-2  arm_conv_q7.c:331, :336, :345, :633 ... and arm_correlate_q7.c:403, :713 ...: "((q31_t) in2 << 16U)" with a negative q15
//              in2 (unrolled loops, count >= 4): left shift of a negative value.
//   B-DCVQ7-3  Include/arm_math_memory.h:105 (read_q15x2, "pQ15[1] << 16"): left shift of a negative value; opt variants with negative data.
//   B-DCVQ7-4  UBSan reports "left shift of negative value" at the call sites arm_conv_opt_q7.c:305 and arm_conv_partial_opt_q7.c:324
//              (write_q7x4_ia (&pOut, __PACKq7(out0, out1, out2, out3)); the macro dsp/none.h:222-230 "(int32_t)(v) << 8/16/24" is the cause
//              but is not reported itself; arm_correlate_opt_q7.c uses the same macro): negative outputs, opt variants with 4 or more outputs.
//   B-DCVQ7-5  (ASan, undocumented defect) the opt variants read 2 bytes behind the DOC sized scratch1 (max(A,B) + 2 min(A,B) - 2 q15 elements):
//              read_q15x2 (arm_math_memory.h:105) called from arm_conv_opt_q7.c:263, arm_correlate_opt_q7.c:288 and arm_conv_partial_opt_q7.c:282,
//              when min(A,B) is a multiple of 4 and the other length is 1 mod 4: (4,5), (8,9), (12,13), (16,17) in both orders. The upper half of
//              the pair is never used, so the results are correct. Evidence: ASan --no-skip on the tight scratch of the dedicated cases below
//              ("READ of size 2 ... bytes after ... region" at arm_math_memory.h:105).
#define UB_PLAIN KNOWN_ISSUE_UB_HIDDEN("B-DCVQ7-1: dsp/none.h:460 __SMLAD shifts a packed pair left; B-DCVQ7-2: unrolled loop '(q31_t) in2 << 16U' of a negative value")
#define UB_OVERREAD KNOWN_ISSUE_UB_HIDDEN("B-DCVQ7-5: the opt variant reads 2 bytes behind the DOC sized pScratch1 (read_q15x2, arm_math_memory.h:105) when min(A, B) is a multiple of 4 and the other length is 1 mod 4")
#define UB_OPT KNOWN_ISSUE_UB_HIDDEN("B-DCVQ7-1: dsp/none.h:460 __SMLAD shifts a packed pair left; B-DCVQ7-3: arm_math_memory.h:105 read_q15x2; B-DCVQ7-4: dsp/none.h:222 __PACKq7 shifts negative outputs left")

#define CONV_CASES(F, N, UBD, CLEAN)                                                                                                  \
	TEST_CASE("[C1 " N "] ARM's pattern test configs (nba x nbb = {30..33,48} x {31..34,49}, 25 ids) reach SNR 15 and 5 LSB" * UBD) { armFullPatternTest(F); } \
	TEST_CASE("[C3 " N "] random data of lengths 1..9 x 1..9 (3 seeds, both argument orders) equals the exact model sat8(acc >> 7)" * UBD) { sizesSweepTest(F, true); } \
	TEST_CASE("[C3 " N "] random non negative data of lengths 1..9 x 1..9 equals the exact model" * UBD) { sizesSweepTest(F, false); } \
	TEST_CASE("[C3 " N "] UB free subset (" CLEAN ") of the 1..9 x 1..9 sweep equals the exact model for negative and non negative data") { sizesSweepTest(F, true, true); sizesSweepTest(F, false, true); } \
	TEST_CASE("[C3 " N "] random data of the lengths 10, 11, 12, 13, 15, 16, 17, 20, 31, 32, 33, 48, 49, 64 (all pairs) equals the exact model" * UBD) { bigSizesTest(F, true); } \
	TEST_CASE("[C3 " N "] random non negative data of the same long lengths equals the exact model" * UBD) { bigSizesTest(F, false); } \
	TEST_CASE("[C5 " N "] the result is the floor of the double sum / 128: 0 <= acc / 128 - y < 1 for non clipping data, lengths 1..20" * UBD) { floorErrorTest(F, true); } \
	TEST_CASE("[C5 " N "] the floor error is in [0, 1) for non clipping data of the UB free subset (" CLEAN "), lengths 1..20") { floorErrorTest(F, true, true); floorErrorTest(F, false, true); } \
	TEST_CASE("[C3 " N "] one value x one value equals sat8(floor(a b / 128)) for all 65536 pairs") { oneTapExhaustiveTest(F); } \
	TEST_CASE("[C3 " N "] two values x one coefficient (127, -128, -37) equals the model for all 65536 pairs in both argument orders") { twoTapExhaustiveTest(F); } \
	TEST_CASE("[C4 " N "] results beyond the 1.7 range are saturated to 127 / -128 (all -128, and 127 x -128), lengths 1..9" * UBD) { saturationTest(F); } \
	TEST_CASE("[C4 " N "] results beyond the 1.7 range are saturated to 127 / -128 for the UB free subset (" CLEAN ") of lengths 1..9") { saturationTest(F, true); } \
	TEST_CASE("[C4 " N "] swapping the arguments gives the same result (conv) or the reversed buffer (correlate), lengths 1..12" * UBD) { \
		if (F.op == CONV) commutativeTest(F, true); else reverseTest(F, true); }                                                        \
	TEST_CASE("[C4 " N "] swapping the arguments gives the same result (conv) or the reversed buffer (correlate) for the UB free subset (" CLEAN ")") { \
		if (F.op == CONV) { commutativeTest(F, true, true); commutativeTest(F, false, true); } else { reverseTest(F, true, true); reverseTest(F, false, true); } } \
	TEST_CASE("[C4 " N "] a second identical call (lengths 13 and 6) gives the same output and the inputs are not modified" * UBD) { repeatTest(F, true); } \
	TEST_CASE("[C4 " N "] a second identical call (lengths 3 and 1) gives the same output and the inputs are not modified") { repeatTest(F, true, 3, 1); }

CONV_CASES(kConv, "arm_conv_q7", UB_PLAIN, "min(A, B) <= 3")
CONV_CASES(kConvOpt, "arm_conv_opt_q7", UB_OPT, "min(A, B) = 1, and A + B - 1 < 4 for negative data")
CONV_CASES(kCorr, "arm_correlate_q7", UB_PLAIN, "min(A, B) <= 3")
CONV_CASES(kCorrOpt, "arm_correlate_opt_q7", UB_OPT, "min(A, B) = 1, and A + B - 1 < 4 for negative data")

// hand derived impulse response (a = {64}: 0.5), 8 values; the plain variants are UB free (count 1), the opt variants only for 2 values
TEST_CASE("[C3 arm_conv_q7] a single 0.5 (64) against 8 hand picked values gives floor(x / 2) in both argument orders") { impulseTest(kConv, 8); }
TEST_CASE("[C3 arm_correlate_q7] a single 0.5 (64) against 8 hand picked values gives floor(x / 2), reversed, in both argument orders") { impulseTest(kCorr, 8); }
TEST_CASE("[C3 arm_conv_opt_q7] a single 0.5 (64) against 8 hand picked values gives floor(x / 2) in both argument orders" * UB_OPT) { impulseTest(kConvOpt, 8); }
TEST_CASE("[C3 arm_correlate_opt_q7] a single 0.5 (64) against 8 hand picked values gives floor(x / 2), reversed, in both argument orders" * UB_OPT) { impulseTest(kCorrOpt, 8); }
TEST_CASE("[C3 arm_conv_opt_q7] a single 0.5 (64) against the first 2 hand picked values gives floor(x / 2) in both argument orders") { impulseTest(kConvOpt, 2); }
TEST_CASE("[C3 arm_correlate_opt_q7] a single 0.5 (64) against the first 2 hand picked values gives floor(x / 2), reversed, in both argument orders") { impulseTest(kCorrOpt, 2); }

TEST_CASE("[C3 arm_conv_opt_q7] the output is bit identical to arm_conv_q7 for lengths 1..12 and sparse 13..40" * UB_OPT) { optEqualsPlainTest(CONV, true); }
TEST_CASE("[C3 arm_correlate_opt_q7] the output is bit identical to arm_correlate_q7 for lengths 1..12 and sparse 13..40" * UB_OPT) { optEqualsPlainTest(CORR, true); }

// ---- C2: exact size heap outputs, all-zero unrolled-path data, B-DCVQ7-5
#define TIGHT_CASES(F, N, CLEAN) \
	TEST_CASE("[C2 " N "] an exact size heap output (no guard words) holds the model values for the UB free sizes (" CLEAN ") of lengths 1..9, negative and non negative data") { tightOutputTest(F); } \
	TEST_CASE("[C2 " N "] all-zero data of lengths {4,5,7,8,9,12,13,16,17,33} (all pairs) gives zeros with guarded and with exact size outputs and tight scratch") { zeroDataTest(F); }
TIGHT_CASES(kConv, "arm_conv_q7", "min(A, B) <= 3")
TIGHT_CASES(kConvOpt, "arm_conv_opt_q7", "min(A, B) = 1")
TIGHT_CASES(kCorr, "arm_correlate_q7", "min(A, B) <= 3")
TIGHT_CASES(kCorrOpt, "arm_correlate_opt_q7", "min(A, B) = 1")

TEST_CASE("[C2 arm_conv_partial_q7] an exact size heap output holds the model values for windows of lengths 1..9 x 1..9") { partialTightTest(false); }
TEST_CASE("[C2 arm_conv_partial_opt_q7] an exact size heap output holds the model values for the UB free windows (min(A, B) = 1) of lengths 1..9") { partialTightTest(true); }
TEST_CASE("[C2 arm_conv_partial_q7] all-zero data of lengths {4,5,8,9,12,13,17} (all pairs, 3 windows) gives zeros with guarded and exact size outputs") { partialZeroDataTest(false); }
TEST_CASE("[C2 arm_conv_partial_opt_q7] all-zero data of lengths {4,5,8,9,12,13,17} (all pairs, 3 windows) gives zeros with guarded and exact size outputs") { partialZeroDataTest(true); }

TEST_CASE("[C2 arm_conv_opt_q7] DOC sized scratch1 is enough for (A, B) = (4,5), (8,9), (12,13), (16,17) in both orders" * UB_OVERREAD) { scratchOverreadTest(kConvOpt); }
TEST_CASE("[C2 arm_correlate_opt_q7] DOC sized scratch1 is enough for (A, B) = (4,5), (8,9), (12,13), (16,17) in both orders" * UB_OVERREAD) { scratchOverreadTest(kCorrOpt); }
TEST_CASE("[C2 arm_conv_partial_opt_q7] DOC sized scratch1 is enough for (A, B) = (4,5), (8,9), (12,13), (16,17) in both orders, full window" * UB_OVERREAD) { partialScratchOverreadTest(); }

// ---- correlate padding (characterization). a has 7 values, b has 1: 6 padding samples; (a, b): 6 leading, (b, a): 6 trailing.
TEST_CASE("characterization: [C4 arm_correlate_q7] the (A - B) leading (A > B) or (B - A) trailing padding samples are not written") {
	// DOC (arm_correlate_q7.c): "pDst ... Length 2 * max(srcALen, srcBLen) - 1"; the code skips the padding (pOut += j). NEEDS REVIEW.
	const V8 a = {1, 2, 3, 4, 5, 6, 7}, b = {9};
	for (int swap = 0; swap < 2; swap++) {
		const V8 &x = swap ? b : a, &y = swap ? a : b;
		const Run r = runFull(kCorr, x, y, false);
		CHECK(r.guardOk);
		for (size_t i = 0; i < 6; i++) {
			const size_t idx = swap ? r.out.size() - 1 - i : i;
			CHECK_MESSAGE(r.out[idx] == static_cast<int8_t>(0xCD), "swap " << swap << " idx " << idx);
		}
	}
}

TEST_CASE("characterization: [C4 arm_correlate_opt_q7] the (A - B) leading (A > B) or (B - A) trailing padding samples are not written") {
	const V8 a = {1, 2, 3, 4, 5, 6, 7}, b = {9};
	for (int swap = 0; swap < 2; swap++) {
		const V8 &x = swap ? b : a, &y = swap ? a : b;
		const Run r = runFull(kCorrOpt, x, y, false);
		CHECK(r.guardOk);
		for (size_t i = 0; i < 6; i++) {
			const size_t idx = swap ? r.out.size() - 1 - i : i;
			CHECK_MESSAGE(r.out[idx] == static_cast<int8_t>(0xCD), "swap " << swap << " idx " << idx);
		}
	}
}

// ---- partial
TEST_CASE("[C1 arm_conv_partial_q7] ARM's pattern test (nba 6, nbb 8, first 3, 9, 7, 4 points) reaches SNR 15 and 5 LSB") { armPartialPatternTest(false); }
TEST_CASE("[C1 arm_conv_partial_opt_q7] ARM's pattern test (nba 6, nbb 8, first 3, 9, 7, 4 points) reaches SNR 15 and 5 LSB" * UB_OPT) { armPartialPatternTest(true); }
TEST_CASE("[C3 arm_conv_partial_q7] every window of the full convolution, lengths 1..9 x 1..9, equals the model and nothing before it is written") { partialWindowsTest(false, true); }
TEST_CASE("[C3 arm_conv_partial_opt_q7] every window of the full convolution, lengths 1..9 x 1..9, equals the model and nothing before it is written" * UB_OPT) { partialWindowsTest(true, true); }
TEST_CASE("[C3 arm_conv_partial_opt_q7] every window of the 1..9 x 1..9 sweep with min(A, B) = 1 (windows below 4 points for negative data) equals the model") { partialWindowsTest(true, true, true); partialWindowsTest(true, false, true); }
TEST_CASE("[C3 arm_conv_partial_q7] random windows of lengths 10, 15, 16, 17, 33, 48 (all pairs, 12 windows each) equal the model") { partialBigTest(false, true); }
TEST_CASE("[C3 arm_conv_partial_opt_q7] random windows of lengths 10, 15, 16, 17, 33, 48 (all pairs, 12 windows each) equal the model" * UB_OPT) { partialBigTest(true, true); }
TEST_CASE("[C4 arm_conv_partial_q7] windows beyond A + B - 2 (A 6, B 8) return ARM_MATH_ARGUMENT_ERROR and write nothing") { partialErrorTest(false); }
TEST_CASE("[C4 arm_conv_partial_opt_q7] windows beyond A + B - 2 (A 6, B 8) return ARM_MATH_ARGUMENT_ERROR and write nothing") { partialErrorTest(true); }
TEST_CASE("[C4 arm_conv_partial_q7] the last valid windows (A 6, B 1: 0+6, 5+1, 2+4, 4+2) succeed with the model values") { partialBoundaryTest(false); }
TEST_CASE("[C4 arm_conv_partial_opt_q7] the last valid windows (A 6, B 1: 0+6, 5+1, 2+4, 4+2) succeed with the model values") { partialBoundaryTest(true); }

TEST_CASE("characterization: [C4 arm_conv_partial_q7] a result beyond the 1.7 range is saturated like the full convolution (the DOC has no scaling paragraph)") {
	const V8 a(6, -128), b(1, -128);  // full result: 127 everywhere (16384 >> 7 = 128 -> 127)
	const PRun r = runPartial(false, a, b, 2, 3, 5);
	CHECK(r.st == ARM_MATH_SUCCESS);
	for (size_t i = 2; i < 5; i++) CHECK(r.out[i] == 127);
}
TEST_CASE("characterization: [C4 arm_conv_partial_opt_q7] a result beyond the 1.7 range is saturated like the full convolution (the DOC has no scaling paragraph)") {
	const V8 a(6, -128), b(1, -128);
	const PRun r = runPartial(true, a, b, 2, 3, 5);
	CHECK(r.st == ARM_MATH_SUCCESS);
	for (size_t i = 2; i < 5; i++) CHECK(r.out[i] == 127);
}

}  // TEST_SUITE
