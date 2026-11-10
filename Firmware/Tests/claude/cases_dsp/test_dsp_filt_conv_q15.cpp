/*
 * CMSIS-DSP Filtering, convolution and correlation, q15 (work order CVQ15, phase P11 group G5):
 *   arm_conv_q15, arm_conv_fast_q15, arm_conv_opt_q15, arm_conv_fast_opt_q15,
 *   arm_conv_partial_q15, arm_conv_partial_fast_q15, arm_conv_partial_opt_q15, arm_conv_partial_fast_opt_q15,
 *   arm_correlate_q15, arm_correlate_fast_q15, arm_correlate_opt_q15, arm_correlate_fast_opt_q15.
 * Requirement ids: Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * ARM tests (C1): Libraries/CMSIS-DSP/Testing/Source/Tests/MISCQ15.cpp, suite "MISC Q15" (Testing/desc.txt:2875-3010). Ids:
 *   test_correlate_q15 (MISCQ15.cpp:20-33, 25 configs nba 14/15/16/17/32 x nbb 15/16/17/18/33, references 1..25),
 *   test_conv_q15 (:35-53, same 25 configs, references 26..50),
 *   test_conv_partial_q15 (:55-76), _fast_q15 (:78-99), _opt_q15 (:101-128), _fast_opt_q15 (:130-157): nba 6, nbb 8,
 *   firstIndex 3 / 9 / 7, NBPOINTS 4 (:23-25), references 54..56, inputs InputsA2 / InputsB2.
 *   No id is disabled or commented out. ARM has NO test of arm_conv_fast_q15, arm_conv_opt_q15, arm_conv_fast_opt_q15 and of the
 *   three arm_correlate_{fast,opt,fast_opt}_q15: they run on ARM's pattern inputs against the integer DEF (C3), not against ARM's
 *   (double precision, generated with np.convolve / np.correlate, Testing/PatternGeneration/Convolutions.py:62, :77, :131) references.
 * Pattern files (Testing/Patterns/DSP/Filtering/MISC/MISCQ15/): InputsA1 / InputsB1 (conv, correlate; the first nba / nbb values
 * are used, MISCQ15.cpp:~600), InputsA2 / InputsB2 (partial), Reference1..50, Reference54..56 (one block per config).
 * ARM tolerances: SNR_THRESHOLD 65 (:8), ABS_ERROR_Q15 10 LSB (:15, correlate, conv, partial std), ABS_ERROR_FAST_Q15 20 LSB (:17,
 * partial fast, opt and fast_opt). Used unchanged (L4). ARM's correlate and conv tests compare the whole output; the correlate reference
 * contains zeros in the pad (2*max-1 samples), so the C1 correlate output buffer is pre-filled with zeros like ARM's output array.
 *
 * Definitions (DEF, exact integer model, derived from the DOC and confirmed against ARM's patterns):
 *  - conv:      full[n] = sum_k a[k] b[n-k], n = 0 .. srcALen+srcBLen-2 (DOC "Length srcALen+srcBLen-1"). The functions swap the
 *               arguments internally when srcALen < srcBLen: both orders are tested.
 *  - partial:   the same values for n = firstIndex .. firstIndex+numPoints-1, written at pDst[firstIndex ...] (arm_conv_partial_q15.c:140
 *               "pOut = pDst + firstIndex"; ARM reads outp[first], MISCQ15.cpp:67). DOC: "ARM_MATH_ARGUMENT_ERROR : requested subset is not in
 *               the range [0 srcALen+srcBLen-2]", code (:87) firstIndex + numPoints > srcALen + srcBLen - 1. No output is written then.
 *  - correlate: DOC "Length 2 * max(srcALen, srcBLen) - 1". r[i] = sum_{n=0}^{B-1} a[n+i-(B-1)] b[n] (terms with an index outside
 *               a are zero), i = 0 .. A+B-2 (np.correlate(a, b, "full")). With A >= B the A-B zeros of the pad come FIRST, with A < B the
 *               B-A pad zeros come LAST (Convolutions.py:63-70 and the code comment in arm_correlate_q15.c: "(srcALen - srcBLen) zeroes
 *               has to included in the starting of the output buffer").
 *  - Scaling (DOC of the std / opt functions): 64 bit accumulator in 34.30 of the exact products, "truncated to 34.15 format by
 *    discarding the low 15 bits and then saturated to 1.15 format": result = clamp(floor(S / 2^15), -32768, 32767), S = exact sum.
 *    The floor error against S/2^15 is one-sided: 0 <= S/2^15 - result < 1 (tested against an independent double computation, C5).
 *  - fast / fast_opt: 32 bit accumulator in 2.30, "if the accumulator overflows it wraps around and distorts the result", "Scale down the
 *    inputs by log2(min(srcALen, srcBLen))". The tests keep every window sum below 2^30 in magnitude: random data are limited to
 *    |x| <= amp with min(A,B) * amp^2 < 2^30 (amp(n) below), so the result equals the std function and floor(S/2^15) fits 1.15 without
 *    saturation. The DOC says the 2.30 accumulator "is right shifted by 15 bits and then saturated to 1.15 format": the three
 *    NON-opt fast functions (arm_conv_fast_q15.c:172 etc., arm_conv_partial_fast_q15, arm_correlate_fast_q15.c:218 etc.) only cast
 *    "(q15_t)(sum >> 15)" with no saturation (the _fast_opt functions use __SSAT) -> B-DCVQ15-1 below. A real 32 bit overflow is not
 *    tested (signed overflow, UB, no DOC value).
 *
 * Scratch buffers (DOC): opt / fast_opt convolution: pScratch1 max(srcALen, srcBLen) + 2*min(srcALen, srcBLen) - 2, pScratch2
 * min(srcALen, srcBLen); the correlate opt functions take ONE scratch of the pScratch1 size. Every call uses tight `new q15_t[n]` scratch
 * buffers of exactly this size, filled with a poison pattern before the call. Every call is done twice: with tight heap output
 * (`new q15_t[len]`, ASan sees an overrun) and with a dsp::GuardedBuffer (guard intact, nothing written outside the window); the
 * two outputs must be equal. Inputs are exact-size std::vector copies and must be unchanged afterwards.
 *
 * Host vs target (C6): the host runs the portable C path (no ARM_MATH_DSP, ARM_MATH_LOOPUNROLL set). arm_conv_q15.c, arm_correlate_q15.c and
 * arm_conv_partial_q15.c are `#if defined(ARM_MATH_DSP)` ... `#else` plain loops: the host runs ONLY the `#else` loops, the Cortex-M4
 * target runs different (SMLALD / SMLAD) code, so these tests say nothing about the target code of those three functions. The results of
 * the other functions are the same integer arithmetic on both while nothing overflows, but the host runs the portable emulation of the SIMD
 * instructions (dsp/none.h) which is not the target code. Fixed width types only; right shifts of negative values are arithmetic (GCC / ARM
 * GCC, implementation-defined in C). The unsaturated "(q15_t)" casts of the fast functions wrap on both. The portable SIMD emulation
 * (arm_math_memory.h:105 read_q15x2, dsp/none.h) shifts negative values left (UBSan, existing finding V64): see B-DCVQ15-5..7.
 *
 * NEEDS REVIEW (reviewer measurement, no test added): forcing the ARM_MATH_DSP branch of arm_conv_q15.c / arm_conv_partial_q15.c /
 * arm_correlate_q15.c in a scratch build makes 7 of the plain cases fail: partial_q15 A=9 B=3 first=10 and A=40 B=10 first=48
 * (the B-DCVQ15-4 count defect, which exists in the DSP branch of arm_conv_partial_q15.c too), the two tap A=256 B=2 conv / partial giving
 * 8192 where the DEF says 8191 (the B-DCVQ15-3 sign extension +1, real on the target) and the three satAccumulator64 cases
 * (B-DCVQ15-2: an emulation artefact only). These defects are therefore not host-only for the std functions.
 *
 * Not tested, NEEDS REVIEW: srcALen / srcBLen 0 (DOC silent, kernels read / loop unspecified), numPoints 0 of the partial functions
 * (DOC silent), firstIndex + numPoints overflowing 32 bit (wraps past the range check; not DOC-specified), overlapping input and output
 * buffers, a real 32 bit accumulator overflow of the fast functions.
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
int64_t randIn(Rng &rng, int64_t lo, int64_t hi) {
	const uint32_t r = rng.next();
	return lo + static_cast<int64_t>(static_cast<uint64_t>(r) % static_cast<uint64_t>(hi - lo + 1));
}

enum Kind { CONV, PART, CORR };
struct Fn {
	const char *name;
	Kind kind;
	bool opt;   // takes scratch buffers
	bool fast;  // 32 bit accumulator
	bool sat;   // saturates the final narrowing
};
const Fn kFn[12] = {
	{"arm_conv_q15", CONV, false, false, true},
	{"arm_conv_fast_q15", CONV, false, true, false},
	{"arm_conv_opt_q15", CONV, true, false, true},
	{"arm_conv_fast_opt_q15", CONV, true, true, true},
	{"arm_conv_partial_q15", PART, false, false, true},
	{"arm_conv_partial_fast_q15", PART, false, true, false},
	{"arm_conv_partial_opt_q15", PART, true, false, true},
	{"arm_conv_partial_fast_opt_q15", PART, true, true, true},
	{"arm_correlate_q15", CORR, false, false, true},
	{"arm_correlate_fast_q15", CORR, false, true, false},
	{"arm_correlate_opt_q15", CORR, true, false, true},
	{"arm_correlate_fast_opt_q15", CORR, true, true, true},
};

typedef std::vector<int16_t> V;
const int16_t kPoison = static_cast<int16_t>(0xCDCD);  // GuardedBuffer poisons the output with 0xCD bytes

size_t outLen(const Fn &f, size_t A, size_t B) {
	return f.kind == CORR ? 2 * std::max(A, B) - 1 : A + B - 1;
}

arm_status exec(int id, const q15_t *a, uint32_t la, const q15_t *b, uint32_t lb, q15_t *d, uint32_t first, uint32_t num,
                q15_t *s1, q15_t *s2) {
	switch (id) {
	case 0: arm_conv_q15(a, la, b, lb, d); break;
	case 1: arm_conv_fast_q15(a, la, b, lb, d); break;
	case 2: arm_conv_opt_q15(a, la, b, lb, d, s1, s2); break;
	case 3: arm_conv_fast_opt_q15(a, la, b, lb, d, s1, s2); break;
	case 4: return arm_conv_partial_q15(a, la, b, lb, d, first, num);
	case 5: return arm_conv_partial_fast_q15(a, la, b, lb, d, first, num);
	case 6: return arm_conv_partial_opt_q15(a, la, b, lb, d, first, num, s1, s2);
	case 7: return arm_conv_partial_fast_opt_q15(a, la, b, lb, d, first, num, s1, s2);
	case 8: arm_correlate_q15(a, la, b, lb, d); break;
	case 9: arm_correlate_fast_q15(a, la, b, lb, d); break;
	case 10: arm_correlate_opt_q15(a, la, b, lb, d, s1); break;
	case 11: arm_correlate_fast_opt_q15(a, la, b, lb, d, s1); break;
	}
	return ARM_MATH_SUCCESS;
}

struct Run {
	arm_status st = ARM_MATH_SUCCESS;
	V out;
	bool guardOk = true, same = true, inputsOk = true;
};

// first / num are only used by the partial functions. fill = initial content of the whole output.
// poisonIn: the input copies get one extra element (value 16384) behind them that the function must not use (the library is told the
// real length); a function that over-reads then gets a deterministic value instead of stale heap content.
Run runFn(int id, const V &av, const V &bv, uint32_t first, uint32_t num, int16_t fill, bool poisonIn = false) {
	const Fn &f = kFn[id];
	const V a0 = av, b0 = bv;
	V a(av), b(bv);  // exact size copies (+ the poison element)
	if (poisonIn) {
		a.push_back(16384);
		b.push_back(16384);
	}
	const size_t A = av.size(), B = bv.size(), len = outLen(f, A, B);
	const size_t mn = std::min(A, B), mx = std::max(A, B);
	Run r;
	// tight heap output and tight scratch (no slack): ASan sees an overrun
	std::unique_ptr<q15_t[]> tight(new q15_t[len]);
	std::unique_ptr<q15_t[]> s1(new q15_t[mx + 2 * mn - 2]);
	std::unique_ptr<q15_t[]> s2(new q15_t[mn]);
	std::fill(tight.get(), tight.get() + len, fill);
	std::fill(s1.get(), s1.get() + (mx + 2 * mn - 2), static_cast<q15_t>(0x5A5A));
	std::fill(s2.get(), s2.get() + mn, static_cast<q15_t>(0x5A5A));
	const arm_status st1 = exec(id, a.data(), A, b.data(), B, tight.get(), first, num, f.opt ? s1.get() : nullptr, f.opt ? s2.get() : nullptr);
	dsp::GuardedBuffer<int16_t> g(len);
	std::fill(g.data(), g.data() + len, fill);
	std::fill(s1.get(), s1.get() + (mx + 2 * mn - 2), static_cast<q15_t>(0x5A5A));
	std::fill(s2.get(), s2.get() + mn, static_cast<q15_t>(0x5A5A));
	r.st = exec(id, a.data(), A, b.data(), B, g.data(), first, num, f.opt ? s1.get() : nullptr, f.opt ? s2.get() : nullptr);
	r.out = g.output();
	r.guardOk = g.guardIntact();
	r.same = (st1 == r.st) && std::equal(r.out.begin(), r.out.end(), tight.get());
	r.inputsOk = std::equal(a0.begin(), a0.end(), a.begin()) && std::equal(b0.begin(), b0.end(), b.begin()) && (!poisonIn || (a.back() == 16384 && b.back() == 16384));
	return r;
}

// ---------------------------------------------------------------------------------------------------------------------
// Models
// ---------------------------------------------------------------------------------------------------------------------
// exact sums, index i = 0 .. A+B-2
std::vector<int64_t> convS(const V &a, const V &b) {
	std::vector<int64_t> s(a.size() + b.size() - 1, 0);
	for (size_t i = 0; i < a.size(); i++)
		for (size_t j = 0; j < b.size(); j++) s[i + j] += static_cast<int64_t>(a[i]) * b[j];
	return s;
}
std::vector<int64_t> corrS(const V &a, const V &b) {
	const int64_t A = a.size(), B = b.size();
	std::vector<int64_t> s(A + B - 1, 0);
	for (int64_t i = 0; i < A + B - 1; i++)
		for (int64_t n = 0; n < B; n++) {
			const int64_t k = n + i - (B - 1);
			if (k >= 0 && k < A) s[i] += static_cast<int64_t>(a[k]) * b[n];
		}
	return s;
}
int64_t clampQ15(int64_t v) { return v < INT16_MIN ? INT16_MIN : (v > INT16_MAX ? INT16_MAX : v); }

// amplitude limit of the fast functions: n * amp^2 < 2^30
int64_t ampFor(size_t n) {
	int64_t amp = static_cast<int64_t>(std::sqrt(static_cast<double>((1LL << 30) - 1) / static_cast<double>(n)));
	if (amp > 32767) amp = 32767;
	while (static_cast<int64_t>(n) * amp * amp >= (1LL << 30)) amp--;
	return amp;
}
V genVec(Rng &rng, size_t n, bool limited, size_t minLen) {
	V v(n);
	const int64_t amp = limited ? ampFor(minLen) : 32767;
	for (auto &x : v) x = static_cast<int16_t>(randIn(rng, limited ? -amp : -32768, amp));
	return v;
}

struct Region {
	size_t lo, hi;  // [lo, hi): the defined part of the output
	V exp;          // expected values, full output length (outside the region: unused)
};
// Expected output. The fast functions without saturation must stay in the 1.15 range: *inRange tells the caller.
Region expected(const Fn &f, const V &a, const V &b, uint32_t first, uint32_t num, bool *inRange) {
	const size_t A = a.size(), B = b.size();
	Region r;
	r.exp.assign(outLen(f, A, B), 0);
	std::vector<int64_t> s = f.kind == CORR ? corrS(a, b) : convS(a, b);
	size_t off = 0;
	r.lo = 0;
	r.hi = A + B - 1;
	if (f.kind == CORR) {
		off = A > B ? A - B : 0;
		r.lo = off;
		r.hi = off + A + B - 1;
	}
	if (f.kind == PART) {
		r.lo = first;
		r.hi = first + num;
	}
	if (inRange) *inRange = true;
	for (size_t i = r.lo; i < r.hi; i++) {
		const int64_t S = f.kind == CORR ? s[i - off] : s[i];
		const int64_t v = S >> 15;  // floor (arithmetic shift)
		if (!f.sat && (v < INT16_MIN || v > INT16_MAX) && inRange) *inRange = false;
		r.exp[i] = static_cast<int16_t>(f.sat ? clampQ15(v) : v);
	}
	return r;
}

// Runs one call and compares with the model; returns "" or the first problem. Outside the window nothing may be written
// (conv, partial); the pad of the correlate output is not checked here (see the pad test).
// The three non-opt fast functions finish the odd taps of every sum with __SMLAD(*px++, *py--, sum) on q15 values (arm_conv_fast_q15.c:165,
// :224, :608, :644; arm_conv_partial_fast_q15.c:180, :239, :640, :679; arm_correlate_fast_q15.c:211, :590): a negative q15 promoted to
// uint32 has 0xFFFF in the upper half, which __SMLAD multiplies as well: every tail product of two negative samples adds +1 (2^-30) to
// the accumulator (B-DCVQ15-3). Up to 3 tail products per sum, so floor(S'/2^15) is E or E + 1. Their data tests therefore assert
// 0 <= out - E <= 1 (E = the exact DEF); the exact result is asserted by the KNOWN_ISSUE cases.
bool smladTail(int id) { return id == 1 || id == 5 || id == 9; }

std::string verify(int id, const V &a, const V &b, uint32_t first, uint32_t num, int16_t fill = kPoison, bool skipOutOfRange = false, bool poisonIn = false, bool exact = false) {
	const Fn &f = kFn[id];
	const std::string ctx = std::string(f.name) + " A=" + std::to_string(a.size()) + " B=" + std::to_string(b.size()) +
	                        " first=" + std::to_string(first) + " num=" + std::to_string(num) + ": ";
	const Run r = runFn(id, a, b, first, num, fill, poisonIn);
	if (r.st != ARM_MATH_SUCCESS) return ctx + "status " + std::to_string(static_cast<int>(r.st));
	if (!r.guardOk) return ctx + "guard damaged";
	if (!r.same) return ctx + "tight heap output differs from the guarded output";
	if (!r.inputsOk) return ctx + "an input was modified";
	bool inRange = true;
	const Region e = expected(f, a, b, first, num, &inRange);
	if (!inRange) return skipOutOfRange ? std::string() : ctx + "TEST PRECONDITION: result outside 1.15 for an unsaturated function";
	for (size_t i = 0; i < r.out.size(); i++) {
		if (i >= e.lo && i < e.hi) {
			const int d = r.out[i] - e.exp[i];
			if ((smladTail(id) && !exact) ? (d < 0 || d > 1) : d != 0)
				return ctx + "out[" + std::to_string(i) + "] = " + std::to_string(r.out[i]) + ", expected " + std::to_string(e.exp[i]);
		} else if (f.kind != CORR && r.out[i] != fill) {
			return ctx + "out[" + std::to_string(i) + "] outside the window was written";
		}
	}
	return "";
}

std::vector<std::pair<uint32_t, uint32_t> > windows(size_t L, bool all) {
	std::vector<std::pair<uint32_t, uint32_t> > w;
	if (all) {
		for (uint32_t f = 0; f < L; f++)
			for (uint32_t n = 1; f + n <= L; n++) w.push_back({f, n});
	} else {
		const uint32_t firsts[] = {0, 1, 2, 3, 4, 5, static_cast<uint32_t>(L / 2), static_cast<uint32_t>(L - 5), static_cast<uint32_t>(L - 1)};
		const uint32_t nums[] = {1, 2, 3, 4, 5, 7, 8, 9};
		for (uint32_t f : firsts) {
			if (f >= L) continue;
			for (uint32_t n : nums)
				if (f + n <= L) w.push_back({f, n});
			w.push_back({f, static_cast<uint32_t>(L - f)});
		}
	}
	return w;
}

// arm_conv_partial_fast_q15 (B-DCVQ15-4): stage 3 never reduces count = srcBLen - 1 when firstIndex > srcALen (srcALen = the longer input,
// arm_conv_partial_fast_q15.c:~580), so windows with firstIndex > max(srcALen, srcBLen) read x[srcALen ...] past the longer input. The
// plain tests only use windows with firstIndex <= max(srcALen, srcBLen); the others are the KNOWN_ISSUE cases (input with a poisoned
// extra element behind it).
bool windowOk(int id, bool full, size_t A, size_t B, uint32_t first) { return id != 5 || full || first <= std::max(A, B); }

// all lengths in la x lb (both orders occur), random data
void randomLengths(int id, const std::vector<size_t> &la, const std::vector<size_t> &lb, bool allWin, uint32_t seed, bool full = false) {
	const Fn &f = kFn[id];
	Rng rng(seed + static_cast<uint32_t>(id) * 977u);
	std::string err;
	int runs = 0;
	for (size_t A : la)
		for (size_t B : lb) {
			const size_t mn = std::min(A, B);
			const V a = genVec(rng, A, f.fast, mn);
			const V b = genVec(rng, B, f.fast, mn);
			if (f.kind == PART) {
				for (auto w : windows(A + B - 1, allWin)) {
					if (!windowOk(id, full, A, B, w.first)) continue;
					err = verify(id, a, b, w.first, w.second);
					runs++;
					if (!err.empty()) break;
				}
			} else {
				err = verify(id, a, b, 0, 0);
				runs++;
			}
			if (!err.empty()) break;
		}
	CHECK_MESSAGE(err.empty(), err);
	CHECK(runs > 0);
}

const std::string kDir = "Filtering/MISC/MISCQ15/";
V loadQ15(const std::string &name) { return dsp::loadPattern(kDir + name + "_q15.txt").q15(); }

const size_t kNba[5] = {14, 15, 16, 17, 32};
const size_t kNbb[5] = {15, 16, 17, 18, 33};

// ARM's 25 conv / correlate configs. armTol: compare with ARM's reference (SNR 65, 10 LSB); else integer model.
void armConfigs(int id, bool armRef) {
	const Fn &f = kFn[id];
	const V ia = loadQ15("InputsA1"), ib = loadQ15("InputsB1");
	REQUIRE(ia.size() >= 32);
	REQUIRE(ib.size() >= 33);
	std::string err;
	int k = 0;
	for (size_t na : kNba)
		for (size_t nb : kNbb) {
			k++;
			const V a(ia.begin(), ia.begin() + na), b(ib.begin(), ib.begin() + nb);
			if (armRef) {
				const V ref = loadQ15("Reference" + std::to_string((f.kind == CORR ? 0 : 25) + k));
				REQUIRE(ref.size() == outLen(f, na, nb));
				const Run r = runFn(id, a, b, 0, 0, 0);  // pre-zeroed output like ARM's output array
				const std::string c = std::string(f.name) + " nba=" + std::to_string(na) + " nbb=" + std::to_string(nb) + ": ";
				if (!r.guardOk) err = c + "guard damaged";
				else if (!r.same) err = c + "tight output differs";
				else {
					const double s = dsp::snr(ref, r.out);
					const dsp::Check lc = dsp::withinLsb(ref, r.out, 10);
					if (!(s >= 65.0)) err = c + "SNR " + std::to_string(s) + " < 65";
					else if (!lc) err = c + lc.message;
				}
			} else {
				err = verify(id, a, b, 0, 0, kPoison, !f.sat);  // (unsaturated functions: configs whose result leaves 1.15 are skipped)
			}
			if (!err.empty()) break;
		}
	CHECK_MESSAGE(err.empty(), err);
}

void armPartial(int id, bool armRef, bool full = false, bool poisonIn = false) {
	const Fn &f = kFn[id];
	const V ia = loadQ15("InputsA2"), ib = loadQ15("InputsB2");
	REQUIRE(ia.size() >= 6);
	REQUIRE(ib.size() >= 8);
	const V a(ia.begin(), ia.begin() + 6), b(ib.begin(), ib.begin() + 8);
	const uint32_t firsts[3] = {3, 9, 7};
	std::string err;
	for (int k = 0; k < 3 && err.empty(); k++) {
		if (!windowOk(id, full, 6, 8, firsts[k])) continue;
		if (poisonIn && windowOk(id, false, 6, 8, firsts[k])) continue;  // poisoned runs: only the windows beyond max(A, B)
		if (armRef) {
			const V ref = loadQ15("Reference" + std::to_string(54 + k));
			REQUIRE(ref.size() == 4);
			const Run r = runFn(id, a, b, firsts[k], 4, 0, poisonIn);
			const std::string c = std::string(f.name) + " first=" + std::to_string(firsts[k]) + ": ";
			if (r.st != ARM_MATH_SUCCESS) err = c + "status";
			else if (!r.guardOk || !r.same) err = c + "guard / tight output";
			else {
				const V win(r.out.begin() + firsts[k], r.out.begin() + firsts[k] + 4);
				const int64_t lsb = id == 4 ? 10 : 20;  // ABS_ERROR_Q15 / ABS_ERROR_FAST_Q15 (:15, :17); opt uses the fast bound (:127)
				const double s = dsp::snr(ref, win);
				const dsp::Check lc = dsp::withinLsb(ref, win, lsb);
				if (!(s >= 65.0)) err = c + "SNR " + std::to_string(s) + " < 65";
				else if (!lc) err = c + lc.message;
			}
		} else {
			err = verify(id, a, b, firsts[k], 4, kPoison, false, poisonIn);
		}
	}
	CHECK_MESSAGE(err.empty(), err);
}

// Hand derived example (units 2^-15): a = {16384, -16384, 8192} (0.5, -0.5, 0.25), b = {16384, 16384} (0.5, 0.5).
// 16384 * 16384 = 2^28 -> >> 15 = 8192; 8192 * 16384 = 2^27 -> 4096.
//   conv n0 = a0 b0 = 2^28 -> 8192;   n1 = a0 b1 + a1 b0 = 2^28 - 2^28 = 0;
//        n2 = a1 b1 + a2 b0 = -2^28 + 2^27 = -2^27 -> -4096;   n3 = a2 b1 = 2^27 -> 4096        conv = {8192, 0, -4096, 4096}
//   r[i] = sum_n a[n+i-1] b[n]: r0 = a0 b1 = 2^28 -> 8192; r1 = a0 b0 + a1 b1 = 0; r2 = a1 b0 + a2 b1 = -2^27 -> -4096;
//        r3 = a2 b0 = 2^27 -> 4096. A=3 > B=2: output = {0 (pad), 8192, 0, -4096, 4096}
//   swapped arguments (A=2, B=3, a = {16384, 16384}, b = {16384, -16384, 8192}): the reversed sequence {4096, -4096, 0, 8192}
//        (r0 = a0 b2 = 2^27 -> 4096; r1 = a0 b1 + a1 b2 = -2^28 + 2^27 -> -4096; r2 = a0 b0 + a1 b1 = 0; r3 = a1 b0 -> 8192),
//        output = {4096, -4096, 0, 8192, 0 (pad)}.   conv with swapped arguments is the same conv.
void handExample(int id) {
	const Fn &f = kFn[id];
	const V x = {16384, -16384, 8192}, y = {16384, 16384};
	std::string err;
	if (f.kind == CONV) {
		const V exp = {8192, 0, -4096, 4096};
		for (int order = 0; order < 2 && err.empty(); order++) {
			const Run r = runFn(id, order ? y : x, order ? x : y, 0, 0, kPoison);
			if (r.out != exp || r.st != ARM_MATH_SUCCESS || !r.guardOk) err = "conv order " + std::to_string(order) + " wrong";
		}
	} else if (f.kind == PART) {
		const V exp = {8192, 0, -4096, 4096};
		for (int order = 0; order < 2 && err.empty(); order++) {
			const Run r = runFn(id, order ? y : x, order ? x : y, 1, 2, kPoison);  // n = 1, 2 -> {0, -4096}
			if (r.st != ARM_MATH_SUCCESS || !r.guardOk || r.out.size() != 4 || r.out[0] != kPoison || r.out[1] != exp[1] ||
			    r.out[2] != exp[2] || r.out[3] != kPoison)
				err = "partial order " + std::to_string(order) + " wrong";
		}
	} else {
		const V e1 = {0, 8192, 0, -4096, 4096}, e2 = {4096, -4096, 0, 8192, 0};
		const Run r1 = runFn(id, x, y, 0, 0, 0);
		// A > B
		if (r1.out != e1 || !r1.guardOk) err = "correlate A > B wrong";
		const V ya = {16384, 16384}, xb = {16384, -16384, 8192};
		const Run r3 = runFn(id, ya, xb, 0, 0, 0);
		if (err.empty() && (r3.out != e2 || !r3.guardOk)) err = "correlate A < B wrong";
	}
	CHECK_MESSAGE(err.empty(), err);
}

V sweepX(int stride, int phase) {
	V x;
	for (int i = phase; i < 65536; i += stride) x.push_back(static_cast<int16_t>(i - 32768));
	return x;
}
// The std functions of the portable build are O(n^2) (arm_conv_q15.c:850-866), so "every x" is processed in blocks of 256 values.
std::string sweepBlocks(int id, const V &x, const V &taps) {
	const Fn &f = kFn[id];
	std::string err;
	for (size_t s = 0; s < x.size() && err.empty(); s += 256) {
		const V blk(x.begin() + s, x.begin() + std::min(x.size(), s + 256));
		uint32_t n = f.kind == PART ? static_cast<uint32_t>(blk.size() + taps.size() - 1) : 0;
		err = verify(id, blk, taps, 0, n);
		if (err.empty()) err = verify(id, taps, blk, 0, n);
	}
	return err;
}
// one tap filters, both argument orders. Gains 32767, -32767 and (not fast) -32768 see EVERY x; 10 more gains see every 251st x.
void sweepOneTap(int id) {
	const Fn &f = kFn[id];
	const V xAll = sweepX(1, 0);
	std::string err;
	std::vector<int16_t> full = {32767, -32767};
	if (!f.fast) full.push_back(INT16_MIN);
	const int16_t some[] = {-32000, -16384, -12345, -3, -1, 0, 1, 5, 16384, 25000};
	std::vector<int16_t> cs = full;
	cs.insert(cs.end(), std::begin(some), std::end(some));
	for (size_t k = 0; k < cs.size() && err.empty(); k++) {
		const V x = k < full.size() ? xAll : sweepX(251, static_cast<int>(k));
		err = sweepBlocks(id, x, V{cs[k]});
	}
	CHECK_MESSAGE(err.empty(), err);
}
// two tap filters with |c0| + |c1| <= 32767 (so even the fast functions stay below 2^30), both orders: the first two pairs see every x,
// the other five every 257th x
void sweepTwoTap(int id) {
	const V xAll = sweepX(1, 0);
	const int16_t pairs[][2] = {{16384, -16383}, {32767, 0}, {-20000, 12767}, {1, -1}, {12345, -20422}, {0, -32767}, {-1, -2}};
	std::string err;
	for (int k = 0; k < 7 && err.empty(); k++) {
		const V x = k < 2 ? xAll : sweepX(257, k);
		err = sweepBlocks(id, x, V{pairs[k][0], pairs[k][1]});
	}
	CHECK_MESSAGE(err.empty(), err);
}

V nonNeg(const V &v) {
	V r(v);
	for (auto &x : r) x = static_cast<int16_t>(x < 0 ? (x == INT16_MIN ? INT16_MAX : -x) : x);
	return r;
}
// Exact result for the non-opt fast functions on NON-NEGATIVE data: the sign extension error of B-DCVQ15-3 needs two negative samples, so
// the result must be exactly floor(S / 2^15) (kills any rounding of the narrowing). Lengths 1..9, 12, 13, 16, 17, 33.
void nonNegativeExact(int id) {
	const Fn &f = kFn[id];
	Rng rng(0x70A0u + static_cast<uint32_t>(id));
	const size_t lens[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 13, 16, 17, 33};
	std::string err;
	for (size_t A : lens)
		for (size_t B : lens) {
			const size_t mn = std::min(A, B);
			const V a = nonNeg(genVec(rng, A, true, mn)), b = nonNeg(genVec(rng, B, true, mn));
			if (f.kind == PART) {
				for (auto w : windows(A + B - 1, false)) {
					if (!windowOk(id, false, A, B, w.first)) continue;
					err = verify(id, a, b, w.first, w.second, kPoison, false, false, true);
					if (!err.empty()) break;
				}
			} else {
				err = verify(id, a, b, 0, 0, kPoison, false, false, true);
			}
			if (!err.empty()) break;
		}
	CHECK_MESSAGE(err.empty(), err);
}

// characterization (NEEDS REVIEW): exact model of arm_conv_fast_q15 including the B-DCVQ15-3 error, derived from the source
// (arm_conv_fast_q15.c:108-232 stage 1, :470-490 stage 2, :555-650 stage 3). x = the longer input (A >= B: a), y = the shorter, n = output.
//  stage 1 (n < |y| - 1): count = n + 1 products x[i] y[n-i], i = 0..n; count < 4: all through __SMLAD; else the first 4*(count>>2) products
//          through the correct SIMD pairs and the LAST count%4 through the scalar __SMLAD.
//  stage 2: plain `sum += x*y`, exact.
//  stage 3 (t = n - |x| = 0..|y|-2, count = |y| - 1 - t): the first (|y|-1)>>2 outputs: last count%4 products scalar __SMLAD; the remaining
//          outputs (second part of the stage): all count products scalar __SMLAD.
//  each scalar __SMLAD product of two negative samples adds +1 (the upper halves 0xFFFF * 0xFFFF as signed -1 * -1) to the accumulator.
//  out = floor((S + e) / 2^15). Data in [-3, 3], so that S + e crosses the floor boundary (S in [-3*3*9, 81]; S + e < 0 -> -1).
void convFastModelTest() {
	Rng rng(0x5A5Au);
	std::string err;
	const size_t lens[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 13, 16, 17};
	for (size_t A : lens)
		for (size_t B : lens) {
			V a(A), b(B);
			for (auto &v : a) v = static_cast<int16_t>(randIn(rng, -3, 3));
			for (auto &v : b) v = static_cast<int16_t>(randIn(rng, -3, 3));
			const Run r = runFn(1, a, b, 0, 0, kPoison);
			const V &x = A >= B ? a : b;
			const V &y = A >= B ? b : a;
			const long X = static_cast<long>(x.size()), Y = static_cast<long>(y.size());
			V exp(static_cast<size_t>(X + Y - 1));
			for (long n = 0; n < X + Y - 1; n++) {
				const long lo = std::max(0L, n - Y + 1), hi = std::min(n, X - 1), cnt = hi - lo + 1;
				long tail = 0;
				if (n < Y - 1) tail = cnt < 4 ? cnt : cnt % 4;
				else if (n >= X) {
					const long t = n - X;
					tail = t < ((Y - 1) >> 2) ? cnt % 4 : cnt;
				}
				int64_t S = 0, e = 0;
				for (long i = lo; i <= hi; i++) {
					S += static_cast<int64_t>(x[i]) * y[n - i];
					if (i > hi - tail && x[i] < 0 && y[n - i] < 0) e++;
				}
				exp[n] = static_cast<int16_t>((S + e) >> 15);
			}
			if (r.out != exp) {
				err = "A=" + std::to_string(A) + " B=" + std::to_string(B);
				for (size_t i = 0; i < exp.size() && i < r.out.size(); i++)
					if (r.out[i] != exp[i]) { err += " out[" + std::to_string(i) + "]=" + std::to_string(r.out[i]) + " model " + std::to_string(exp[i]); break; }
				break;
			}
		}
	CHECK_MESSAGE(err.empty(), err);
}

// B-DCVQ15-8: min(A, B) a multiple of 4 and the other length 1 mod 4 ((4,5), (8,9), (12,13), (16,17), both orders): the opt functions read 2
// bytes past the DOC sized scratch1 (read_q15x2, arm_math_memory.h:105), the result is correct on the host
void scratchOverread(int id) {
	const Fn &f = kFn[id];
	Rng rng(0x5C4Au + static_cast<uint32_t>(id));
	const size_t pairs[][2] = {{4, 5}, {8, 9}, {12, 13}, {16, 17}};
	std::string err;
	for (auto &p : pairs)
		for (int order = 0; order < 2 && err.empty(); order++) {
			const size_t A = p[order], B = p[1 - order];
			const size_t mn = std::min(A, B);
			const V a = genVec(rng, A, f.fast, mn), b = genVec(rng, B, f.fast, mn);
			err = verify(id, a, b, 0, f.kind == PART ? static_cast<uint32_t>(A + B - 1) : 0);
		}
	CHECK_MESSAGE(err.empty(), err);
}

// characterization of the wrap of the unsaturated narrowing (conv_partial_fast: its DOC has no "saturated to 1.15" sentence)
void wrapChar(int id, bool neg) {
	const V a = neg ? V{INT16_MIN, INT16_MIN} : V{INT16_MIN}, b = neg ? V{INT16_MAX, INT16_MAX} : V{INT16_MIN};
	const Run r = runFn(id, a, b, neg ? 1 : 0, 1, kPoison);
	REQUIRE(r.st == ARM_MATH_SUCCESS);
	if (neg) {
		REQUIRE(r.out.size() == 3);
		CHECK(r.out[1] == 2);  // floor(-2147418112 / 2^15) = -65534 = 0xFFFF0002, (q15_t) -> 2
	} else {
		REQUIRE(r.out.size() == 1);
		CHECK(r.out[0] == INT16_MIN);  // 2^30 >> 15 = 32768 = 0x8000 -> (q15_t) -32768
	}
}

// B-DCVQ15-4: every window of arm_conv_partial_fast_q15 with firstIndex > max(srcALen, srcBLen), lengths 1..9 x 1..9, both inputs with a poisoned
// (16384) element behind them: the DEF does not depend on it
void partialBeyond() {
	Rng rng(0xBE70u);
	std::string err;
	int runs = 0;
	for (size_t A = 1; A <= 9 && err.empty(); A++)
		for (size_t B = 1; B <= 9 && err.empty(); B++) {
			const size_t mn = std::min(A, B), L = A + B - 1;
			const V a = genVec(rng, A, true, mn), b = genVec(rng, B, true, mn);
			for (uint32_t first = static_cast<uint32_t>(std::max(A, B)) + 1; first < L && err.empty(); first++)
				for (uint32_t n = 1; first + n <= L && err.empty(); n++) {
					err = verify(5, a, b, first, n, kPoison, false, true);
					runs++;
				}
		}
	CHECK_MESSAGE(err.empty(), err);
	CHECK(runs > 0);
}

// C5: independent double computation, floor error one-sided: 0 <= D - out < 1 where D is in range
void doubleTest(int id) {
	const Fn &f = kFn[id];
	Rng rng(0xD0B1Eu + static_cast<uint32_t>(id));
	const size_t lens[] = {1, 2, 3, 4, 5, 8, 9, 17, 33};
	std::string err;
	for (size_t A : lens)
		for (size_t B : lens) {
			const V a = genVec(rng, A, f.fast, std::min(A, B)), b = genVec(rng, B, f.fast, std::min(A, B));
			const uint32_t num = static_cast<uint32_t>(A + B - 1), first = 0;
			const Run r = runFn(id, a, b, first, f.kind == PART ? num : 0, kPoison);
			// D[i] in double: every product |a b| <= 2^30 and the sums stay below 2^53, so doubles are exact; D = S / 2^15 (exact).
			const size_t off = f.kind == CORR && A > B ? A - B : 0;
			for (size_t i = 0; i < (f.kind == PART ? num : A + B - 1) && err.empty(); i++) {
				double S = 0;
				if (f.kind == CORR) {
					for (size_t n = 0; n < B; n++) {
						const long k = static_cast<long>(n + i) - static_cast<long>(B - 1);
						if (k >= 0 && k < static_cast<long>(A)) S += static_cast<double>(a[k]) * static_cast<double>(b[n]);
					}
				} else {
					for (size_t j = 0; j < B; j++)
						if (i >= j && i - j < A) S += static_cast<double>(a[i - j]) * static_cast<double>(b[j]);
				}
				const double D = S / 32768.0;
				const double o = r.out[off + i];
				if (D > -32768.0 && D < 32767.0) {
					// the non-opt fast functions: up to +1 (B-DCVQ15-3), so o may exceed D by less than 1 LSB: o <= D + 1 and o > D - 1
					if (!(smladTail(id) ? (o <= D + 1.0 && o > D - 1.0) : (o <= D && o > D - 1.0)))
						err = std::string(f.name) + " A=" + std::to_string(A) + " B=" + std::to_string(B) + " i=" + std::to_string(i) +
						      ": out " + std::to_string(o) + " D " + std::to_string(D);
				}
			}
			if (!err.empty()) break;
		}
	CHECK_MESSAGE(err.empty(), err);
}

// Saturation of the functions that saturate. a = b-values -32768 * -32768: S = 2^30 -> floor(S / 2^15) = 32768 -> 32767.
void satPositive(int id) {
	const Fn &f = kFn[id];
	const V a = {INT16_MIN}, b = {INT16_MIN};
	const Run r = runFn(id, a, b, 0, f.kind == PART ? 1 : 0, kPoison);
	REQUIRE(r.st == ARM_MATH_SUCCESS);
	REQUIRE(r.out.size() == 1);
	CHECK(r.out[0] == INT16_MAX);
}
// a = {-32768, -32768}, b = {32767, 32767}: window sum at the middle sample = 2 * (-32768 * 32767) = -2147418112 (inside int32),
// floor(S / 2^15) = -65534 -> saturates to -32768 (conv index 1, correlate lag 0 = index 1 (A = B = 2))
void satNegative(int id) {
	const Fn &f = kFn[id];
	const V a = {INT16_MIN, INT16_MIN}, b = {INT16_MAX, INT16_MAX};
	const Run r = runFn(id, a, b, 1, f.kind == PART ? 1 : 0, kPoison);
	REQUIRE(r.st == ARM_MATH_SUCCESS);
	REQUIRE(r.out.size() == 3);
	CHECK(r.out[1] == INT16_MIN);
}
// 64 bit accumulator (std and opt): all -32768, A = B = n: every output sum is k * 2^30 (k >= 1 products): saturates to 32767 at every
// output, also where a 32 bit accumulator would have wrapped (n >= 2 gives 2^31).
void satAccumulator64(int id) {
	const Fn &f = kFn[id];
	std::string err;
	const size_t ns[] = {2, 3, 4, 5, 9, 17};
	for (size_t n : ns) {
		const V a(n, INT16_MIN), b(n, INT16_MIN);
		const Run r = runFn(id, a, b, 0, f.kind == PART ? static_cast<uint32_t>(2 * n - 1) : 0, kPoison);
		if (r.st != ARM_MATH_SUCCESS) err = "status";
		for (size_t i = 0; i < r.out.size() && err.empty(); i++)
			if (r.out[i] != INT16_MAX) err = std::string(f.name) + " n=" + std::to_string(n) + " out[" + std::to_string(i) + "] = " + std::to_string(r.out[i]);
		if (!err.empty()) break;
	}
	CHECK_MESSAGE(err.empty(), err);
}

// partial sums of the 64 bit accumulator beyond 2^31 that cancel again: a = n * 20000, b = n/2 times +20000 then -20000 (every pair sum
// 2 * 4e8 = 8e8 < 2^31), the exact DEF decides (a 32 bit accumulator would wrap after 6 products)
void accumulatorCancel(int id) {
	std::string err;
	const size_t ns[] = {12, 16, 17};
	for (size_t n : ns) {
		const V a(n, 20000);
		V b(n, 20000);
		for (size_t i = n / 2; i < n; i++) b[i] = -20000;
		const uint32_t num = static_cast<uint32_t>(2 * n - 1);
		err = verify(id, a, b, 0, kFn[id].kind == PART ? num : 0);
		if (err.empty()) err = verify(id, b, a, 0, kFn[id].kind == PART ? num : 0);
		if (!err.empty()) break;
	}
	CHECK_MESSAGE(err.empty(), err);
}
// B-DCVQ15-3: a = {-1, -1}, b = {-32767, -32767}: S = {32767, 65534, 32767} = {2^15 - 1, 2^16 - 2, 2^15 - 1}, floor(S / 2^15) = {0, 1, 0}
// exactly (correlate: the same three sums, lags -1, 0, 1). Each end sum is a single product; +1 on it would give 1 (or 2^16 - 1 -> 1).
// partial_fast: window 0..1 only (B-DCVQ15-4).
void smladSign(int id) {
	const V a = {-1, -1}, b = {-32767, -32767};
	const bool part = kFn[id].kind == PART;
	const Run r = runFn(id, a, b, 0, part ? (id == 5 ? 2 : 3) : 0, kPoison);
	REQUIRE(r.st == ARM_MATH_SUCCESS);
	REQUIRE(r.out.size() == 3);
	CHECK(r.out[0] == 0);
	CHECK(r.out[1] == 1);
	if (id != 5) CHECK(r.out[2] == 0);
}

// valid = false: windows beyond the range return ARM_MATH_ARGUMENT_ERROR; valid = true: windows inside the range succeed (the status is
// all that is checked). arm_conv_partial_fast_q15 only gets windows with firstIndex <= max(srcALen, srcBLen) (B-DCVQ15-4).
void partialErrors(int id, bool valid) {
	const V a = {1000, -2000, 3000, -4000, 5000, -6000};
	const V b = {1, 2, 3, 4, 5, 6, 7, 8};
	const size_t L = a.size() + b.size() - 1;  // 13
	REQUIRE(L == 13);
	struct C { uint32_t first, num; };
	const C bad[] = {{13, 1}, {9, 5}, {0, 14}, {7, 7}, {1000, 1}, {12, 2}};
	const C good[] = {{0, 13}, {12, 1}, {7, 6}, {0, 1}, {3, 6}};
	const C goodFast[] = {{0, 9}, {3, 6}, {0, 1}, {8, 1}};
	std::string err;
	for (int order = 0; order < 2 && err.empty(); order++) {
		if (!valid) {
			for (const C &c : bad) {
				const Run r = runFn(id, order ? b : a, order ? a : b, c.first, c.num, kPoison);
				if (r.st != ARM_MATH_ARGUMENT_ERROR) err = "invalid window not ARM_MATH_ARGUMENT_ERROR first=" + std::to_string(c.first) + " num=" + std::to_string(c.num);
			}
		} else if (id == 5) {
			for (const C &c : goodFast) {
				const Run r = runFn(id, order ? b : a, order ? a : b, c.first, c.num, kPoison);
				if (r.st != ARM_MATH_SUCCESS) err = "valid window rejected first=" + std::to_string(c.first) + " num=" + std::to_string(c.num);
			}
		} else {
			for (const C &c : good) {
				const Run r = runFn(id, order ? b : a, order ? a : b, c.first, c.num, kPoison);
				if (r.st != ARM_MATH_SUCCESS) err = "valid window rejected first=" + std::to_string(c.first) + " num=" + std::to_string(c.num);
			}
		}
	}
	CHECK_MESSAGE(err.empty(), err);
}
void partialErrorWritesNothing(int id) {
	const V a = {1000, -2000, 3000, -4000, 5000, -6000};
	const V b = {1, 2, 3, 4, 5, 6, 7, 8};
	std::string err;
	const uint32_t firsts[] = {13, 9, 0, 1000};
	const uint32_t nums[] = {1, 5, 14, 1};
	for (int k = 0; k < 4 && err.empty(); k++) {
		const Run r = runFn(id, a, b, firsts[k], nums[k], kPoison);
		if (r.st != ARM_MATH_ARGUMENT_ERROR) err = "status";
		for (size_t i = 0; i < r.out.size() && err.empty(); i++)
			if (r.out[i] != kPoison) err = "out[" + std::to_string(i) + "] written on error";
		if (err.empty() && !r.guardOk) err = "guard";
	}
	CHECK_MESSAGE(err.empty(), err);
}

// pad of the correlate output: the DOC gives only the length 2*max-1; the code comment says zeros are included in the output; the code
// never writes the pad (A > B: the first A-B samples, A < B: the last B-A samples), ARM's test pre-zeroes its output.
void padTest(int id) {
	const V a1 = {100, 200, 300, 400, 500, 600, 700}, b1 = {1, 2, 3};   // A > B: 4 leading pad samples
	const V a2 = {1, 2, 3}, b2 = {100, 200, 300, 400, 500, 600, 700};   // A < B: 4 trailing pad samples
	const Run r1 = runFn(id, a1, b1, 0, 0, kPoison), r2 = runFn(id, a2, b2, 0, 0, kPoison);
	REQUIRE(r1.out.size() == 13);
	REQUIRE(r2.out.size() == 13);
	bool kept1 = true, kept2 = true;
	for (int i = 0; i < 4; i++) {
		kept1 = kept1 && r1.out[i] == kPoison;
		kept2 = kept2 && r2.out[9 + i] == kPoison;
	}
	CHECK(kept1);
	CHECK(kept2);
}

// ---------------------------------------------------------------------------------------------------------------------
std::vector<size_t> seq(size_t lo, size_t hi) {
	std::vector<size_t> v;
	for (size_t i = lo; i <= hi; i++) v.push_back(i);
	return v;
}
const size_t kLongArr[] = {10, 12, 13, 15, 16, 17, 31, 32, 33, 40};

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------
// Defects (ids are renumbered by the lead):
// B-DCVQ15-1  the three non-opt fast functions do not saturate (DOC: "The 2.30 accumulator is right shifted by 15 bits and then saturated to
//             1.15 format"): arm_conv_fast_q15.c:172 / :231 / :431-:435 / :489 / :526 / :615, arm_conv_partial_fast_q15.c:454-:458 (and the
//             scalar tails), arm_correlate_fast_q15.c:218 / :416-:426 / :480 / :519 / :597 do "(q15_t)(sum >> 15)": 2^30 >> 15 = 32768 wraps
//             to -32768, -65534 wraps to 2 (the _fast_opt functions use __SSAT).
// B-DCVQ15-2  PORTABLE EMULATION ONLY, firmware unaffected (Cortex-M SMLALD accumulates in 64 bits): dsp/none.h:503 __SMLALD and
//             __SMLALDX (:518 / :519) add the two int products before widening:
//             (-32768 * -32768) * 2 = 2^31 overflows int (signed overflow, UB; UBSan) and the 64 bit accumulator receives -2^31. DOC of the std /
//             opt functions: "64-bit accumulator ... 33 guard bits and there is no risk of overflow". Seen for 2 products of -1.0 * -1.0 in a
//             unrolled pair (arm_conv_opt_q15, arm_conv_partial_opt_q15, arm_correlate_opt_q15: n >= 4), result -32768 instead of 32767.
// B-DCVQ15-3  see smladTail(): +1 (2^-30) per tail product of two negative samples in the non-opt fast functions (real on the target too).
//             Exact checks: non-negative data must give exactly floor(S / 2^15) (all three); conv_fast has an exact model of the error
//             (characterization, derived from the source, convFastModelTest). The partial_fast and correlate_fast error is NOT modelled
//             exactly (their stage / tail structure differs): only the 0..+1 bound and the non-negative exact test.
// B-DCVQ15-8  the opt functions read 2 bytes past the DOC sized scratch1 (see UB_SCR, scratchOverread): KNOWN_ISSUE_UB_HIDDEN cases.
// B-DCVQ15-4  arm_conv_partial_fast_q15.c (stage 3, ~:580, read at :679): when firstIndex > srcALen (srcALen = the longer input) count =
//             srcBLen - 1 is never reduced, so the outputs read x[srcALen ...] past the longer input (ASan heap-buffer-overflow, A = 9, B = 3,
//             first = 10, 1 point) and use it. The plain tests use windows with firstIndex <= max(A, B) (the reviewer's examples A = 3, B = 3,
//             first = 4 and A = 4, B = 4, first = 5 depend on the over-read element); ARM's pattern id first = 9 (> max(6, 8)) is covered only
//             by a KNOWN_ISSUE case whose input is followed by a poisoned element (16384), so its result does not depend on the heap.
// ---------------------------------------------------------------------------------------------------------------------
#ifdef HOSTTEST_SANITIZERS
constexpr bool kSan = true;
#else
constexpr bool kSan = false;
#endif
// Sanitizer coverage gap (hidden undefined behaviour). UBSan with --no-skip, one process per case, reported for these cases (kind x function id
// bitmask; bit n = function n of kFn). All three defects are in the portable emulation of the q15 SIMD kernels and cannot be avoided with
// any data that reaches the SIMD paths:
//  B-DCVQ15-5  arm_math_memory.h:105 read_q15x2: "(pQ15[1] << 16)" shifts a negative value left
//  B-DCVQ15-6  dsp/none.h:460 __SMLAD (and the __SMLADX family) and :503 __SMLALD: "((q31_t)x << 16)" shifts a packed pair or a negative value
//              left; for a pair whose upper half is positive the shift overflows int ("left shift of 327739536 by 16 places")
//  B-DCVQ15-7  the __PKHBT call sites with a negative operand shifted left: arm_conv_opt_q15.c:291, arm_conv_fast_opt_q15.c:294-295,
//              arm_conv_partial_opt_q15.c:306
// The cases that were clean in the sanitizer run (arm_conv_q15, arm_conv_partial_q15 and arm_correlate_q15 on most data: their portable
// std paths are plain loops) stay active in sanitizer builds.
enum UbKind { K_C1P, K_ARMDEF, K_RSMALL, K_RLONG, K_HAND, K_ONE, K_TWO, K_DOUBLE, K_SATN, K_CANCEL, K_PVALID, K_NKINDS };
#define M(n) (1u << (n))
constexpr unsigned kAllSimd = M(1) | M(2) | M(3) | M(5) | M(6) | M(7) | M(9) | M(10) | M(11);
constexpr unsigned kUbMask[K_NKINDS] = {
	M(5) | M(6) | M(7),                      // K_C1P
	kAllSimd, kAllSimd, kAllSimd,            // K_ARMDEF, K_RSMALL, K_RLONG
	M(2) | M(3) | M(6) | M(7),               // K_HAND
	M(2) | M(3) | M(6) | M(7) | M(10) | M(11),  // K_ONE
	kAllSimd, kAllSimd,                      // K_TWO, K_DOUBLE
	M(6) | M(7),                             // K_SATN
	M(2) | M(6) | M(10),                     // K_CANCEL
	M(5) | M(6) | M(7),                      // K_PVALID
};
constexpr bool ubHit(int kind, int id) { return ((kUbMask[kind] >> id) & 1u) != 0; }
#define UBTEXT "B-DCVQ15-5 (arm_math_memory.h:105 read_q15x2 shifts a negative left), B-DCVQ15-6 (dsp/none.h:460 / :503 __SMLAD / __SMLALD shift packed pairs left), B-DCVQ15-7 (__PKHBT call sites in the opt functions)"
#define UBH(KIND, ID) \
	doctest::description(ubHit(KIND, ID) ? "KNOWN ISSUE (undefined behaviour, intended result on the host): " UBTEXT : "") * doctest::skip(kSan && ubHit(KIND, ID))

#define ALLFN(X)                                                                                                      \
	X(0, "arm_conv_q15", "")                                                                                          \
	X(1, "arm_conv_fast_q15", " (within 0..+1 LSB above the DEF, B-DCVQ15-3)")                                        \
	X(2, "arm_conv_opt_q15", "")                                                                                      \
	X(3, "arm_conv_fast_opt_q15", "")                                                                                 \
	X(4, "arm_conv_partial_q15", "")                                                                                  \
	X(5, "arm_conv_partial_fast_q15", " (within 0..+1 LSB above the DEF, B-DCVQ15-3; only windows with firstIndex <= max(srcALen, srcBLen), B-DCVQ15-4)") \
	X(6, "arm_conv_partial_opt_q15", "")                                                                              \
	X(7, "arm_conv_partial_fast_opt_q15", "")                                                                         \
	X(8, "arm_correlate_q15", "")                                                                                     \
	X(9, "arm_correlate_fast_q15", " (within 0..+1 LSB above the DEF, B-DCVQ15-3)")                                   \
	X(10, "arm_correlate_opt_q15", "")                                                                                \
	X(11, "arm_correlate_fast_opt_q15", "")

#define ALLPART(X)                                            \
	X(4, "arm_conv_partial_q15")                              \
	X(5, "arm_conv_partial_fast_q15")                         \
	X(6, "arm_conv_partial_opt_q15")                          \
	X(7, "arm_conv_partial_fast_opt_q15")

#define STD64FN(X)                                            \
	X(0, "arm_conv_q15")                                      \
	X(4, "arm_conv_partial_q15")                              \
	X(8, "arm_correlate_q15")

#define OPT64FN(X)                                            \
	X(2, "arm_conv_opt_q15")                                  \
	X(6, "arm_conv_partial_opt_q15")                          \
	X(10, "arm_correlate_opt_q15")

#define SATFN(X)                                              \
	X(0, "arm_conv_q15")                                      \
	X(2, "arm_conv_opt_q15")                                  \
	X(3, "arm_conv_fast_opt_q15")                             \
	X(4, "arm_conv_partial_q15")                              \
	X(6, "arm_conv_partial_opt_q15")                          \
	X(7, "arm_conv_partial_fast_opt_q15")                     \
	X(8, "arm_correlate_q15")                                 \
	X(10, "arm_correlate_opt_q15")                            \
	X(11, "arm_correlate_fast_opt_q15")

#define NOSATFN(X)                                            \
	X(1, "arm_conv_fast_q15")                                 \
	X(5, "arm_conv_partial_fast_q15")                         \
	X(9, "arm_correlate_fast_q15")

#define NOSATUBFN(X)                                          \
	X(1, "arm_conv_fast_q15")                                 \
	X(9, "arm_correlate_fast_q15")

#define UB_SCR "B-DCVQ15-8: the opt functions read 2 bytes past the DOC sized scratch1 via read_q15x2 (arm_math_memory.h:105, called from arm_conv_opt_q15.c:252, arm_correlate_opt_q15.c:228, arm_conv_fast_opt_q15.c:256, arm_correlate_fast_opt_q15.c:233, arm_conv_partial_opt_q15.c:268, arm_conv_partial_fast_opt_q15.c:267) when min(A, B) is a multiple of 4 and the other length is 1 mod 4"
#define UB_PF "B-DCVQ15-4: arm_conv_partial_fast_q15.c stage 3 does not reduce count = srcBLen - 1 when firstIndex > srcALen: it reads x[srcALen ...] past the longer input (the test input has a poisoned element there) and the result depends on it"
#define UB_OPT64 "B-DCVQ15-2 (portable emulation only, firmware unaffected): dsp/none.h:503 __SMLALD / :518-:519 __SMLALDX add two products of -32768 * -32768 in int (2^31 overflows, UB) before widening, the 64 bit accumulator gets -2^31"

TEST_SUITE("DSP Filtering q15 convolution") {

// C1: ARM's pattern tests (conv std, correlate std, the four partial functions)
TEST_CASE("[C1 arm_conv_q15] ARM's pattern test (25 configs nba 14..32 x nbb 15..33) reaches SNR 65 and 10 LSB") { armConfigs(0, true); }
TEST_CASE("[C1 arm_correlate_q15] ARM's pattern test (25 configs nba 14..32 x nbb 15..33, 2*max-1 samples with pad) reaches SNR 65 and 10 LSB") { armConfigs(8, true); }
TEST_CASE("[C1 arm_conv_partial_q15] ARM's pattern test (nba 6, nbb 8, first 3, 9, 7, 4 points) reaches SNR 65 and 10 LSB") { armPartial(4, true); }
TEST_CASE("[C1 arm_conv_partial_fast_q15] ARM's pattern test, first = 3 and 7 (nba 6, nbb 8, 4 points) reaches SNR 65 and 20 LSB" * UBH(K_C1P, 5)) { armPartial(5, true); }
// ARM's id first = 9 is covered only here (9 > max(6, 8), over-read of the longer input, poisoned element behind it)
TEST_CASE("[C1 arm_conv_partial_fast_q15] ARM's pattern test, first = 9 (nba 6, nbb 8, 4 points), input followed by a poisoned element, reaches SNR 65 and 20 LSB" * KNOWN_ISSUE_UB(UB_PF)) { armPartial(5, true, true, true); }
TEST_CASE("[C1 arm_conv_partial_opt_q15] ARM's pattern test (nba 6, nbb 8, first 3, 9, 7, 4 points) reaches SNR 65 and 20 LSB" * UBH(K_C1P, 6)) { armPartial(6, true); }
TEST_CASE("[C1 arm_conv_partial_fast_opt_q15] ARM's pattern test (nba 6, nbb 8, first 3, 9, 7, 4 points) reaches SNR 65 and 20 LSB" * UBH(K_C1P, 7)) { armPartial(7, true); }

// the same inputs against the integer DEF for every function, incl. the ones ARM does not test
#define T_ARMDEF(ID, NAME, EXTRA) \
	TEST_CASE("[C3 " NAME "] on ARM's pattern inputs (conv / correlate: 25 configs; partial: first 3, 9, 7 of nba 6 x nbb 8) the output equals the integer DEF" EXTRA * UBH(K_ARMDEF, ID)) { \
		if (kFn[ID].kind == PART) armPartial(ID, false); else armConfigs(ID, false); \
	}
ALLFN(T_ARMDEF)

// C3: random data, lengths 1..9 x 1..9 (every pair: both argument orders, unroll tails); partial: every valid window
#define T_RANDSMALL(ID, NAME, EXTRA) \
	TEST_CASE("[C3 " NAME "] random data, lengths 1..9 x 1..9 (all windows for partial; fast variants limited to n*amp^2 < 2^30) equal the integer DEF" EXTRA * UBH(K_RSMALL, ID)) { \
		randomLengths(ID, seq(1, 9), seq(1, 9), true, 0x1001u); \
	}
ALLFN(T_RANDSMALL)

#define T_RANDLONG(ID, NAME, EXTRA) \
	TEST_CASE("[C3 " NAME "] random data, lengths {10, 12, 13, 15, 16, 17, 31, 32, 33, 40} squared (windows 0..L-1 sampled for partial) equal the integer DEF" EXTRA * UBH(K_RLONG, ID)) { \
		randomLengths(ID, std::vector<size_t>(std::begin(kLongArr), std::end(kLongArr)), std::vector<size_t>(std::begin(kLongArr), std::end(kLongArr)), false, 0x2002u); \
	}
ALLFN(T_RANDLONG)

#define T_HAND(ID, NAME, EXTRA) \
	TEST_CASE("[C3 " NAME "] the hand derived example {0.5, -0.5, 0.25} with {0.5, 0.5} gives {8192, 0, -4096, 4096} (conv) or its correlation, in both argument orders" * UBH(K_HAND, ID)) { handExample(ID); }
ALLFN(T_HAND)

#define T_ONE(ID, NAME, EXTRA) \
	TEST_CASE("[C3 " NAME "] a one tap filter (3 gains over every x from -32768 to 32767 and 10 over every 251st, both argument orders) equals clamp(floor(x*c / 2^15))" EXTRA * UBH(K_ONE, ID)) { sweepOneTap(ID); }
ALLFN(T_ONE)

#define T_TWO(ID, NAME, EXTRA) \
	TEST_CASE("[C3 " NAME "] a two tap filter (2 coefficient pairs over every x and 5 over every 257th x, both argument orders) equals the integer DEF" EXTRA * UBH(K_TWO, ID)) { sweepTwoTap(ID); }
ALLFN(T_TWO)

#define T_DOUBLE(ID, NAME, EXTRA) \
	TEST_CASE("[C5 " NAME "] random data, lengths {1, 2, 3, 4, 5, 8, 9, 17, 33} squared, is within one LSB below and nothing above the double DEF (floor)" EXTRA * UBH(K_DOUBLE, ID)) { doubleTest(ID); }
ALLFN(T_DOUBLE)

// partial_fast, windows with firstIndex > max(srcALen, srcBLen) (B-DCVQ15-4): the input has a poisoned element behind it
TEST_CASE("[C3 arm_conv_partial_fast_q15] ARM's pattern inputs with first = 9 (> max(6, 8)), input followed by a poisoned element, equal the integer DEF" * KNOWN_ISSUE_UB(UB_PF)) { armPartial(5, false, true, true); }
TEST_CASE("[C3 arm_conv_partial_fast_q15] random data, lengths 1..9 x 1..9, every window with firstIndex > max(srcALen, srcBLen), input followed by a poisoned element, equal the integer DEF" * KNOWN_ISSUE_UB(UB_PF)) { partialBeyond(); }

// C4 saturation
#define T_SATP(ID, NAME) \
	TEST_CASE("[C4 " NAME "] -1.0 * -1.0 (2^30) saturates to 32767") { satPositive(ID); }
SATFN(T_SATP)
#define T_SATN(ID, NAME) \
	TEST_CASE("[C4 " NAME "] a sum of -65534 LSB (2 * -32768 * 32767 >> 15) saturates to -32768" * UBH(K_SATN, ID)) { satNegative(ID); }
SATFN(T_SATN)
#define T_SAT64(ID, NAME) \
	TEST_CASE("[C4 " NAME "] all -32768 inputs of 2, 3, 4, 5, 9 and 17 taps saturate to 32767 at every output (64 bit accumulator)") { satAccumulator64(ID); }
STD64FN(T_SAT64)
#define T_SAT64U(ID, NAME) \
	TEST_CASE("[C4 " NAME "] all -32768 inputs of 2, 3, 4, 5, 9 and 17 taps saturate to 32767 at every output (64 bit accumulator)" * KNOWN_ISSUE_UB(UB_OPT64)) { satAccumulator64(ID); }
OPT64FN(T_SAT64U)
#define T_CANCEL(ID, NAME) \
	TEST_CASE("[C4 " NAME "] partial sums beyond 2^31 that cancel (12, 16 and 17 taps of 20000, +-20000 coefficients, both orders) equal the DEF: 64 bit accumulator" * UBH(K_CANCEL, ID)) { accumulatorCancel(ID); }
STD64FN(T_CANCEL)
OPT64FN(T_CANCEL)

// B-DCVQ15-1
#define T_NOSATP(ID, NAME) \
	TEST_CASE("[C4 " NAME "] -1.0 * -1.0 (2^30) saturates to 32767 as the DOC says" * KNOWN_ISSUE("B-DCVQ15-1: DOC says the 2.30 accumulator is right shifted by 15 and saturated to 1.15, the code casts (q15_t)(sum >> 15): 2^30 >> 15 = 32768 wraps to -32768")) { satPositive(ID); }
NOSATUBFN(T_NOSATP)
TEST_CASE("characterization: [C4 arm_conv_partial_fast_q15] -1.0 * -1.0 (2^30) wraps to -32768 (the DOC of this function has no saturation sentence; NEEDS REVIEW)") { wrapChar(5, false); }
TEST_CASE("characterization: [C4 arm_conv_partial_fast_q15] a sum of -65534 LSB wraps to 2 (the DOC of this function has no saturation sentence; NEEDS REVIEW)") { wrapChar(5, true); }
#define T_NOSATNU(ID, NAME) \
	TEST_CASE("[C4 " NAME "] a sum of -65534 LSB saturates to -32768 as the DOC says" * KNOWN_ISSUE_UB("B-DCVQ15-1: DOC says saturated to 1.15, the code casts (q15_t)(sum >> 15): -65534 wraps to 2; the run also hits " UBTEXT)) { satNegative(ID); }
NOSATUBFN(T_NOSATNU)

// B-DCVQ15-3
#define T_SIGN(ID, NAME) \
	TEST_CASE("[C3 " NAME "] sums 32767, 65534, 32767 (-1 and -32767 data, 2 taps) give floor(S / 2^15) = 0, 1, 0 exactly" * KNOWN_ISSUE_UB("B-DCVQ15-3 (also hits " UBTEXT "): the odd tap __SMLAD(*px++, *py--, sum) multiplies the sign extended upper halves too and adds +1: S = 32768, result 1")) { smladSign(ID); }
NOSATFN(T_SIGN)

// exact result on non-negative data (all three) and the exact model of the sign extension error (conv_fast)
#define T_NONNEG(ID, NAME) \
	TEST_CASE("[C3 " NAME "] random non-negative data (lengths 1..9, 12, 13, 16, 17, 33 squared) give exactly floor(S / 2^15), no tolerance" * UBH(K_RSMALL, ID)) { nonNegativeExact(ID); }
NOSATFN(T_NONNEG)
TEST_CASE("characterization: [C3 arm_conv_fast_q15] data in [-3, 3] (1..9, 12, 13, 16, 17 taps squared) give exactly floor((S + e) / 2^15), e = tail products of two negative samples (NEEDS REVIEW, B-DCVQ15-3)" * UBH(K_RSMALL, 1)) { convFastModelTest(); }

// B-DCVQ15-8
#define T_SCRATCH(ID, NAME) \
	TEST_CASE("[C2 " NAME "] lengths (4,5), (8,9), (12,13), (16,17) in both orders with scratch1 of exactly the DOC size give the DEF result" * KNOWN_ISSUE_UB_HIDDEN(UB_SCR)) { scratchOverread(ID); }
#define OPTFN(X) X(2, "arm_conv_opt_q15") X(3, "arm_conv_fast_opt_q15") X(6, "arm_conv_partial_opt_q15") X(7, "arm_conv_partial_fast_opt_q15") X(10, "arm_correlate_opt_q15") X(11, "arm_correlate_fast_opt_q15")
OPTFN(T_SCRATCH)

// C4 partial argument range
#define T_PERR(ID, NAME) \
	TEST_CASE("[C4 " NAME "] a window with firstIndex + numPoints > srcALen + srcBLen - 1 (nba 6, nbb 8: 13+1, 9+5, 0+14, 7+7, 1000+1, 12+2) returns ARM_MATH_ARGUMENT_ERROR") { partialErrors(ID, false); }
ALLPART(T_PERR)
#define T_PVALID(ID, NAME) \
	TEST_CASE("[C4 " NAME "] windows inside [0, srcALen + srcBLen - 2] return ARM_MATH_SUCCESS (partial_fast: only windows with firstIndex <= max(srcALen, srcBLen), B-DCVQ15-4)" * UBH(K_PVALID, ID)) { partialErrors(ID, true); }
ALLPART(T_PVALID)
#define T_PERRW(ID, NAME) \
	TEST_CASE("characterization: [C4 " NAME "] an out of range window writes nothing to the output (DOC silent)") { partialErrorWritesNothing(ID); }
ALLPART(T_PERRW)

// pad of the correlate output
#define T_PAD(ID, NAME) \
	TEST_CASE("characterization: [C4 " NAME "] the pad samples (A=7, B=3 leading, A=3, B=7 trailing) of a poisoned output buffer are not written (DOC gives only the length; NEEDS REVIEW)") { padTest(ID); }
T_PAD(8, "arm_correlate_q15")
T_PAD(9, "arm_correlate_fast_q15")
T_PAD(10, "arm_correlate_opt_q15")
T_PAD(11, "arm_correlate_fast_opt_q15")

}  // TEST_SUITE
