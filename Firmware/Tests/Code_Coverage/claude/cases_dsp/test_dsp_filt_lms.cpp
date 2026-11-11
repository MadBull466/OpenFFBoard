/*
 * CMSIS-DSP Filtering, LMS and normalised LMS (work order LMS, phase P11 group G5): arm_lms_f32 / arm_lms_q15 / arm_lms_q31,
 * arm_lms_norm_f32 / arm_lms_norm_q15 / arm_lms_norm_q31, their init functions, and the inline helpers arm_recip_q31 (charter L7:
 * tested directly and through arm_lms_norm_q31; Include/dsp/utils.h:67) and arm_recip_q15 (direct, it is what arm_lms_norm_q15 uses).
 * Requirement ids: Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * ARM has NO test for any of these functions (Testing/desc.txt and desc_f16.txt list no LMS suite, the MISC suites cover only
 * levinson_durbin, correlate, conv and conv_partial), so there are no pattern files and no C1 tests. Everything is DEF (C3), derived from
 * the DOC of arm_lms_f32.c (group LMS), arm_lms_norm_f32.c (group LMS_NORM), arm_lms_q15.c, arm_lms_q31.c, arm_lms_norm_q15.c,
 * arm_lms_norm_q31.c and the init functions, and from the source where the DOC is silent (stated where it is used).
 *
 * DOC (arm_lms_f32.c, :40-110 "Algorithm"):  y[n] = b[0] x[n] + ... + b[T-1] x[n-T+1];  e[n] = d[n] - y[n];
 *   b[k] += e[n] mu x[n-k]  (k = 0 .. T-1, updated sample by sample); pCoeffs is stored TIME REVERSED {b[T-1], ..., b[0]}, so with
 *   c = pCoeffs the sum is y[n] = sum_k c[k] x[n-T+1+k] and the update is c[k] += mu e[n] x[n-T+1+k]. The state array has T + blockSize - 1
 *   elements (blockSize = the block size of the calls), "the state variables are updated after each block". The init functions set the
 *   fields and "zero out the values in the state buffer"; they return void (no argument errors exist).
 *   Normalised LMS (arm_lms_norm_f32.c): E = x[n]^2 + ... + x[n-T+1]^2 (the energy of the T newest samples including x[n]) and
 *   b[k] += e[n] (mu / E) x[n-k]. The code adds a regularisation constant to E (float: 0.000000119209289f in the source comment
 *   "epsilon value"; q31: DELTA_Q31 = 0x100 LSB; q15: DELTA_Q15 = 5 LSB, filtering_functions.h:46-47); it is NOT in the DOC, so the DEF
 *   uses mu / E and every bound below has the term delta / E.
 *   Fixed point (DOC arm_lms_q15.c, arm_lms_q31.c): q15: "multiplications yield a 2.30 result ... accumulated in a 64-bit accumulator in
 *   34.30 format ... truncated to 34.15 by discarding the low 15 bits ... saturated to yield a result in 1.15 format", filter coefficient
 *   updates "are saturated"; q31: "2.62" accumulator, "shifted and saturated to 1.31" (and "wraps around rather than clips" for an
 *   accumulator overflow: contradiction, see NEEDS REVIEW). postShift: "shifts the result by postShift bits, scaling the coefficients by
 *   2^postShift": y = 2^ps * sum c x.
 *
 * Formats derived here (a = Q value / 2^fb, fb = 15 or 31; all x, c, d, y, e in 1.fb):
 *   q15: acc = sum c x (34.30, exact); y = floor(acc / 2^(15-ps)) (arithmetic shift = floor, DOC "truncated"), exact model, no rounding.
 *        e = d - y (plain subtraction, exact when in range).
 *        update (the DOC says only "saturated"): alpha = floor(e mu / 2^15); c += floor(alpha x / 2^15); the double DEF is c += mu e x:
 *        two floors, |alpha error| < 1 LSB times |x| / 2^15 < 1, |second floor| < 1: |lib - DEF| < 2 LSB (strict).
 *   q31: acc 2.62; y = acc >> (31-ps) (floor); alpha = floor(e mu / 2^31); coef = floor(alpha x / 2^32); c += 2 coef, i.e. the double
 *        DEF c += mu e x is met within |alpha error| |x| / 2^31 + 2 |floor error| < 3 LSB (strict).
 *   q15 norm: energy (1.15, LSBs of 2^-15) E_q = E_q - floor(x0^2 / 2^15) + floor(x^2 / 2^15) per sample: the error against the exact
 *        energy is within (-1, +1) LSB per sample, so |E_q - E| < n LSB after n samples (the test also checks the instance field
 *        S->energy against this derived bound). w = sat(floor(errorXmu * recip(E_q + 5) / 2^(15-ps_r))) with errorXmu = floor(e mu / 2^15).
 *        Double DEF: w = e mu / E; |w_lib - w_DEF| <= 1/E (errorXmu floor, in w LSBs) + 1 (final floor) + |w| (rR + errE/E + delta/E)
 *        with rR = relative accuracy of arm_recip_q15 and errE = n LSB; then c += floor(w x / 2^15): error <= |w error| |x| / 2^15 + 1.
 *   q31 norm: energy (1.31) E_q = floor(E_q - x0^2 / 2^31) then floor(. + x^2 / 2^31): error in (-2, 0] LSB per sample (one-sided);
 *        w = clip(floor(errorXmu * recip(E_q + 256) / 2^(31-ps_r))), errorXmu = floor(e mu / 2^31); coefficient += 2 floor(w x / 2^32):
 *        |lib - DEF| <= |w error| |x| / 2^31 + 2, |w error| as above with errE = 2n LSB.
 *   arm_recip_q31(in, &out, table): normalises in to xn = (in << clz(in)-1) / 2^31 in [0.5, 1), starts from the 64 entry table
 *        (the bin index is the 6 bits below the top bit: the bin is 2^-7 wide and the table holds the value at the bin centre, so the
 *        start residual r = |1 - 2 xn out| <= 2^-8 / 0.5 = 2^-7), runs 2 Newton steps (r -> r^2: 2^-14, 2^-28) and returns the
 *        shift clz(in): real 1/in_real = out / 2^31 * 2^ret (in_real = in / 2^31). Rounding per step: the floor of tempVal (2^-31 abs) and
 *        of the product (2^-31 abs) each give at most 2^-30 relative (out >= 0.5), so the result is within r^2 + 2 rounding terms
 *        = (2^-14)^2 + 2^-28 = 2^-28 + 2^-28 relative at the worst point, i.e. 2^-27 (the residual squared is the 2^-28 term; the true sum with the
 *        rounding of the first step squared is ~2^-28 + 2^-29 in practice; rR31 = 2^-27 keeps the full worst case). arm_recip_q15: floors of 2^-15 abs give 2^-14 relative each, per step
 *        2^-13, so rR15 = 2^-13 plus the squared residual 2^-24 (rR15 = 2^-13 (1 + 2^-10)).
 *
 * Test method for the stateful kernels: TEACHER FORCING. The sample is processed one at a time (blockSize 1, state buffer of exactly T
 * elements), the coefficients that the library holds BEFORE the sample are read from pCoeffs, and the output, the error and the new coefficients
 * are compared with the DEF applied to exactly those coefficients: the bounds are local (one step) and derived, there is no accumulated
 * drift. Float: y within gamma_T sum|c x| (gamma_T = T u / (1 - T u), u = 2^-24), e is the float subtraction d - y exactly,
 * coefficient update within 4 u (|c| + |delta|) + |delta| (running-energy error + delta / E) for the normalised filter. q: y and e EXACT
 * (the DOC says truncation, the model is deterministic), the update within the strict derived bounds above. Over a long run: the sample
 * by sample results of the partitions are compared with the DEF with the accumulated bound (float: the LMS error recursion
 * delta' = (I - mu x x^T / s) delta + local errors is a contraction in the 2-norm for mu ||x||^2 / s <= 2 (s = 1 plain, s = E normalised, E = ||x||^2),
 * so the local errors add: total <= N stepB); q: the partitions are bit identical to one long block (pure integer arithmetic, the same
 * operations in the same order) and the final coefficient vector is within N sqrt(T) (local bound + 1 LSB for the effect of the y floor) of the DEF.
 *
 * Data: x in +-[0.25, 0.3] (so E in [0.0625, 0.81] for T <= 9: below 1 (the q energy never saturates) and above delta / E noise), mu = 0.1 (float) /
 * 0.25 (q, plain) / 0.5 (norm float) / 0.125 (norm q: keeps the converging non-negative runs long) (mu ||x||^2 / s < 2: stable); deterministic xorshift generator, every call its own statement.
 *
 * Host vs target (C6): the DEF uses __int128, so a 64 bit host compiler is required (GCC / clang on a 64 bit target, MinGW-w64 x86_64). The host runs the portable C path (no ARM_MATH_DSP: DSP intrinsics emulated by dsp/none.h, ARM_MATH_LOOPUNROLL: the unrolled
 * loops by 4). The Cortex-M4 uses SMLALD / the real intrinsics: results are the same where nothing overflows. float: the DEF is in double;
 * a target with VMLA / FMA contraction (and the host with -ffp-contract) changes the roundings inside the derived bounds (the bound has the
 * margin 4 u instead of 3 u for the three roundings). Undefined behaviour of the library on the host (UBSan, -fno-sanitize-recover aborts
 * the run: those tests are KNOWN_ISSUE_UB_HIDDEN = intended result on the host, skipped in the sanitizer builds; ids B-DLMS-n, the lead renumbers):
 *   B-DLMS-1  "coef << 1U" shifts a negative value left (every coefficient decrease): arm_lms_q31.c:194-228, arm_lms_norm_q31.c:213-247.
 *   B-DLMS-2  "acc_h << uShift" shifts a negative upper accumulator half left (every negative accumulator): arm_lms_q31.c:162,
 *             arm_lms_norm_q31.c:176, arm_lms_q15.c:150, arm_lms_norm_q15.c:168.
 *   B-DLMS-4  arm_lms_norm_q31.c:120 "(energy << 32)": after the first line of the energy update the energy is floor(E - x0^2 / 2^31), which is -1
 *             whenever the floor loses a bit and E - x0^2 is 0 (numTaps 1: every sample after the first; for numTaps >= 2 the remainder is positive and the
 *             value stays >= 0), and the second line shifts it left (negative value).
 *   B-DLMS-3  q15 (lms and norm) with numTaps >= 4: dsp/none.h:503 (__SMLALD "(q31_t)x << 16") shifts a packed pair left: values above 2^15 do not
 *             fit an int (the ONLY line hit in the runs; dsp/none.h:460 __SMLAD and arm_math_memory.h:105 read_q15x2 have the same pattern but are NOT
 *             confirmed by a run: UBSan stops at :503 first).
 *   B-DLMS-5  arm_lms_q15.c:147-153 and arm_lms_norm_q15.c:165-171 keep only a 32 bit window of the 34.30 accumulator (acc_l >> lShift | acc_h << uShift,
 *             then __SSAT), so a result that needs more than 32 bits after the shift wraps where the DOC says "saturated" (norm: also "no risk of internal
 *             overflow"). T = 5, ps = 14, c = x = 32767: y = -32768 (intended 32767). Reached only with numTaps >= 4 (UB path B-DLMS-3 first), so the cases are
 *             KNOWN_ISSUE_UB (should_fail on the host: the host result is wrong, not only undefined; skipped in sanitizer builds).
 *   B-DLMS-7  arm_lms_norm_q15.c:182: the energy saturates at 32767 (T = 1, x = 32767 gives 32766, any larger window sum clips) and "(q15_t) energy + DELTA_Q15"
 *             (32771) is converted to q15 = -32765 for arm_recip_q15, documented for positive input only: utils.h:139 "in << signBits" shifts a negative
 *             value left (UBSan, run with the case). Host result: the coefficient falls (-1417) instead of rising: KNOWN_ISSUE_UB.
 *   B-DLMS-6  dsp/none.h:503 __SMLALD emulation: ((x lo * y lo) + (x hi * y hi)) is computed in int before the widening to 64 bit, so T = 4, ps = 0,
 *             c = x = -32768 (2^30 + 2^30) overflows and y = -32768 (intended 32767). The target SMLALD accumulates in 64 bit: a HOST EMULATION artefact
 *             (same defect as B-DCVQ15-2 of the conv q15 file); KNOWN_ISSUE_UB like B-DLMS-5.
 *   (lines found with UBSan --no-skip, one process per case)
 *
 * Comparison notes: the q partitions are compared BITWISE with one long block (pure integer arithmetic, the same operations in the same order: justified).
 * The norm q31 energy test tolerates a systematic -1 LSB per step (the derived one-sided bound is (-2n, 0]; a mutant with that drift survives by design).
 * The plain q update is checked with the derived bound (primary oracle) and, in the `characterization:` cases, with the exact floor model of the source
 * (the DOC gives no rounding rule for the update): NEEDS REVIEW.
 *
 * NEEDS REVIEW (not tested or characterization only; details in the report):
 *   - q31 lms / norm: accumulator results that need more than 1.31 after the shift: DOC says both "wraps around rather than clips" and
 *     "shifted and saturated to 1.31": contradiction, not tested.
 *   - q15 results beyond the 32 bit window of the shift code: not asserted as plain.
 *   - blockSize 0 (the init formula numTaps + blockSize - 1 with blockSize 0 and the kernels with numTaps 0 / blockSize 0 are outside the contract), numTaps 0: not tested.
 *   - arm_recip_q31 / arm_recip_q15 with in = 0 or negative ("should not be used with negative values"): not tested.
 *   - the regularisation constants of the normalised filters are not in the DOC (above).
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
#include "arm_common_tables.h"

namespace {

typedef __int128 i128;

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
	double unit() {  // [0, 1)
		const uint32_t r = next32();
		return static_cast<double>(r) / 4294967296.0;
	}
	double range(double lo, double hi) {
		const double u = unit();
		return lo + (hi - lo) * u;
	}
	bool coin() { return (next32() >> 31) != 0; }
};

const double kU = 5.9604644775390625e-8;           // float unit roundoff 2^-24
const double kNlmsEps = 0.000000119209289;           // source constant of arm_lms_norm_f32 (not in the DOC)
const double kRecipRel31 = 7.450580596923828e-9;     // 2^-27
const double kRecipRel15 = 1.220703125e-4 * (1.0 + 1.0 / 1024.0);  // 2^-13 (1 + 2^-10)

double gammaT(size_t T) { return (T * kU) / (1.0 - T * kU); }

// ---------------------------------------------------------------------------------------------------------------
// Traits of the six filters
// ---------------------------------------------------------------------------------------------------------------
struct TrF32 {
	typedef float S;
	typedef arm_lms_instance_f32 Inst;
	static constexpr int fb = 0;
	static constexpr bool isQ = false, norm = false, hasPs = false;
	static const char *nm() { return "arm_lms_f32"; }
	static void init(Inst *i, size_t T, S *c, S *st, S mu, size_t B, unsigned) { arm_lms_init_f32(i, static_cast<uint16_t>(T), c, st, mu, static_cast<uint32_t>(B)); }
	static void run(Inst *i, const S *x, S *d, S *y, S *e, size_t n) { arm_lms_f32(i, x, d, y, e, static_cast<uint32_t>(n)); }
};
struct TrQ15 {
	typedef int16_t S;
	typedef arm_lms_instance_q15 Inst;
	static constexpr int fb = 15;
	static constexpr bool isQ = true, norm = false, hasPs = true;
	static const char *nm() { return "arm_lms_q15"; }
	static void init(Inst *i, size_t T, S *c, S *st, S mu, size_t B, unsigned ps) { arm_lms_init_q15(i, static_cast<uint16_t>(T), c, st, mu, static_cast<uint32_t>(B), ps); }
	static void run(Inst *i, const S *x, S *d, S *y, S *e, size_t n) { arm_lms_q15(i, x, d, y, e, static_cast<uint32_t>(n)); }
};
struct TrQ31 {
	typedef int32_t S;
	typedef arm_lms_instance_q31 Inst;
	static constexpr int fb = 31;
	static constexpr bool isQ = true, norm = false, hasPs = true;
	static const char *nm() { return "arm_lms_q31"; }
	static void init(Inst *i, size_t T, S *c, S *st, S mu, size_t B, unsigned ps) { arm_lms_init_q31(i, static_cast<uint16_t>(T), c, st, mu, static_cast<uint32_t>(B), ps); }
	static void run(Inst *i, const S *x, S *d, S *y, S *e, size_t n) { arm_lms_q31(i, x, d, y, e, static_cast<uint32_t>(n)); }
};
struct TrNF32 {
	typedef float S;
	typedef arm_lms_norm_instance_f32 Inst;
	static constexpr int fb = 0;
	static constexpr bool isQ = false, norm = true, hasPs = false;
	static const char *nm() { return "arm_lms_norm_f32"; }
	static void init(Inst *i, size_t T, S *c, S *st, S mu, size_t B, unsigned) { arm_lms_norm_init_f32(i, static_cast<uint16_t>(T), c, st, mu, static_cast<uint32_t>(B)); }
	static void run(Inst *i, const S *x, S *d, S *y, S *e, size_t n) { arm_lms_norm_f32(i, x, d, y, e, static_cast<uint32_t>(n)); }
};
struct TrNQ15 {
	typedef int16_t S;
	typedef arm_lms_norm_instance_q15 Inst;
	static constexpr int fb = 15;
	static constexpr bool isQ = true, norm = true, hasPs = true;
	static const char *nm() { return "arm_lms_norm_q15"; }
	static void init(Inst *i, size_t T, S *c, S *st, S mu, size_t B, unsigned ps) { arm_lms_norm_init_q15(i, static_cast<uint16_t>(T), c, st, mu, static_cast<uint32_t>(B), static_cast<uint8_t>(ps)); }
	static void run(Inst *i, const S *x, S *d, S *y, S *e, size_t n) { arm_lms_norm_q15(i, x, d, y, e, static_cast<uint32_t>(n)); }
};
struct TrNQ31 {
	typedef int32_t S;
	typedef arm_lms_norm_instance_q31 Inst;
	static constexpr int fb = 31;
	static constexpr bool isQ = true, norm = true, hasPs = true;
	static const char *nm() { return "arm_lms_norm_q31"; }
	static void init(Inst *i, size_t T, S *c, S *st, S mu, size_t B, unsigned ps) { arm_lms_norm_init_q31(i, static_cast<uint16_t>(T), c, st, mu, static_cast<uint32_t>(B), static_cast<uint8_t>(ps)); }
	static void run(Inst *i, const S *x, S *d, S *y, S *e, size_t n) { arm_lms_norm_q31(i, x, d, y, e, static_cast<uint32_t>(n)); }
};

template <class Tr> double toReal(typename Tr::S v) {
	if (Tr::isQ) return std::ldexp(static_cast<double>(v), -Tr::fb);
	return static_cast<double>(v);
}
template <class Tr> typename Tr::S fromReal(double r) {
	typedef typename Tr::S S;
	if constexpr (Tr::isQ) {
		const double v = std::nearbyint(std::ldexp(r, Tr::fb));
		const double hi = std::ldexp(1.0, Tr::fb) - 1.0, lo = -std::ldexp(1.0, Tr::fb);
		return static_cast<S>(std::min(hi, std::max(lo, v)));
	} else {
		return static_cast<S>(r);
	}
}
template <class Tr> double lsb() { return Tr::isQ ? std::ldexp(1.0, -Tr::fb) : 0.0; }

// ---------------------------------------------------------------------------------------------------------------
// Scenario data
// ---------------------------------------------------------------------------------------------------------------
template <class Tr> struct Data {
	size_t T = 0;
	unsigned ps = 0;
	typename Tr::S mu = 0;
	std::vector<typename Tr::S> c0, x, d;
};

// mu: plain 0.1 (float) / 0.25 (q); normalised 0.5 (float) / 0.125 (q, so that the non-negative runs stay long enough: the error of a converging run
// decays by about (1 - mu) per sample and must stay above a few LSB)
template <class Tr> typename Tr::S defaultMu() {
	if (Tr::norm) return fromReal<Tr>(Tr::isQ ? 0.125 : 0.5);
	return fromReal<Tr>(Tr::isQ ? 0.25 : 0.1);
}
template <class Tr> size_t cleanLength(const Data<Tr> &dt);

// clean = every value of the DEF trajectory is non-negative: x > 0, d from a plant h >= c0 (so e >= 0), accumulators >= 0 and nothing needs a
// left shift of a negative value in the library (B-DLMS-1/2). Otherwise signed data.
template <class Tr> Data<Tr> makeData(size_t T, size_t N, uint64_t seed, bool clean, unsigned ps) {
	typedef typename Tr::S S;
	Data<Tr> r;
	r.T = T;
	r.ps = ps;
	r.mu = defaultMu<Tr>();
	Rng g(seed * 131 + T);
	std::vector<double> h(T);
	for (size_t k = 0; k < T; k++) {
		if (clean) {
			const double a = g.range(0.02, 0.04);
			const double gap = g.range(0.03, 0.06);
			r.c0.push_back(fromReal<Tr>(a));
			h[k] = toReal<Tr>(r.c0.back()) + gap;
		} else {
			const double a = g.range(-0.05, 0.05);
			r.c0.push_back(fromReal<Tr>(a));
			h[k] = 0.0;
		}
	}
	std::vector<double> xr(T - 1, 0.0);
	for (size_t n = 0; n < N; n++) {
		double v = g.range(0.25, 0.3);
		const bool neg = g.coin();
		if (!clean && neg) v = -v;
		const S xs = fromReal<Tr>(v);
		r.x.push_back(xs);
		xr.push_back(toReal<Tr>(xs));
	}
	for (size_t n = 0; n < N; n++) {
		double v;
		if (clean) {
			double s = 0;
			for (size_t k = 0; k < T; k++) s += h[k] * xr[n + k];
			v = s * std::ldexp(1.0, static_cast<int>(ps));
		} else {
			v = g.range(-0.06, 0.06);
		}
		r.d.push_back(fromReal<Tr>(v));
	}
	if (clean && Tr::isQ) {
		// non-negative run: the converging error must stay >= 24 LSB (the floors of the library are at most a few LSB, header), so the run is cut
		// at the first sample where the DEF error falls below that; at least 10 samples must remain (test design)
		const size_t m = cleanLength<Tr>(r);
		REQUIRE_MESSAGE(m >= 10, "test design: the non-negative run is shorter than 10 samples (T=" << T << " ps=" << ps << ")");
		r.x.resize(m);
		r.d.resize(m);
	}
	return r;
}

// ---------------------------------------------------------------------------------------------------------------
// Instance owner: tight heap coefficient and state arrays (no slack: ASan sees every overrun), guarded outputs
// ---------------------------------------------------------------------------------------------------------------
template <class Tr> struct Filt {
	typedef typename Tr::S S;
	typename Tr::Inst inst;
	std::unique_ptr<S[]> c, st;
	size_t T, B;
	Filt(size_t T_, size_t B_, const std::vector<S> &c0, S mu, unsigned ps) : c(new S[T_]), st(new S[T_ + B_ - 1]), T(T_), B(B_) {
		std::memcpy(c.get(), c0.data(), T * sizeof(S));
		std::memset(static_cast<void *>(st.get()), 0xCD, (T + B - 1) * sizeof(S));
		std::memset(static_cast<void *>(&inst), 0xCD, sizeof(inst));
		Tr::init(&inst, T, c.get(), st.get(), mu, B, ps);
	}
	// processes x[off .. off+n) with d[off ..) (n <= B); appends to y and e
	void run(const std::vector<S> &x, const std::vector<S> &d, size_t off, size_t n, std::vector<S> &y, std::vector<S> &e) {
		std::vector<S> in(x.begin() + off, x.begin() + off + n);
		std::vector<S> ref(d.begin() + off, d.begin() + off + n);
		const std::vector<S> refCopy = ref;
		dsp::GuardedBuffer<S> gy(n), ge(n);
		Tr::run(&inst, in.data(), ref.data(), gy.data(), ge.data(), n);
		CHECK_MESSAGE(gy.guardIntact(), "guard behind the output damaged");
		CHECK_MESSAGE(ge.guardIntact(), "guard behind the error output damaged");
		CHECK_MESSAGE(ref == refCopy, "pRef was modified");
		const std::vector<S> oy = gy.output(), oe = ge.output();
		y.insert(y.end(), oy.begin(), oy.end());
		e.insert(e.end(), oe.begin(), oe.end());
	}
	std::vector<S> coeffs() const { return std::vector<S>(c.get(), c.get() + T); }
};

// ---------------------------------------------------------------------------------------------------------------
// Teacher forcing (one sample per call, checks against the DEF applied to the library's own coefficients)
// ---------------------------------------------------------------------------------------------------------------
template <class Tr> struct Teacher {
	typedef typename Tr::S S;
	size_t T;
	unsigned ps;
	S mu;
	bool clean;
	bool exact = false;  // characterization: also assert the exact floor model of the plain update (header)
	Filt<Tr> f;
	std::vector<S> win;  // the T newest inputs, oldest first
	size_t n = 0;
	double maxAbsC = 0, maxDelta = 0, emax = 0, emin = 1e30;
	Teacher(size_t T_, const std::vector<S> &c0, S mu_, unsigned ps_, bool clean_) : T(T_), ps(ps_), mu(mu_), clean(clean_), f(T_, 1, c0, mu_, ps_), win(T_, 0) {}

	void step(S xs, S ds) {
		win.erase(win.begin());
		win.push_back(xs);
		n++;
		std::vector<S> cp = f.coeffs();
		std::vector<S> y, e;
		f.run(std::vector<S>(1, xs), std::vector<S>(1, ds), 0, 1, y, e);
		const std::vector<S> cn = f.coeffs();
		double E = 0;
		for (size_t k = 0; k < T; k++) {
			const double xv = toReal<Tr>(win[k]);
			E += xv * xv;
		}
		emax = std::max(emax, E);
		emin = std::min(emin, E);
		if constexpr (Tr::isQ) stepQ(cp, cn, y[0], e[0], ds, E);
		else stepF(cp, cn, y[0], e[0], ds, E);
	}

	void stepF(const std::vector<float> &cp, const std::vector<float> &cn, float y, float e, float d, double E) {
		double ydef = 0, sabs = 0;
		for (size_t k = 0; k < T; k++) {
			const double t = static_cast<double>(cp[k]) * static_cast<double>(win[k]);
			ydef += t;
			sabs += std::fabs(t);
		}
		const double ybound = gammaT(T) * sabs;
		CHECK_MESSAGE(std::fabs(static_cast<double>(y) - ydef) <= ybound, "n=" << n << " y " << y << " DEF " << ydef << " bound " << ybound);
		const float eExpect = d - y;  // float subtraction, exact IEEE operation
		CHECK_MESSAGE(e == eExpect, "n=" << n << " e " << e << " expected d - y = " << eExpect);
		const double k_ = Tr::norm ? static_cast<double>(mu) / E : static_cast<double>(mu);
		// norm: running energy error of the library: n steps, each 2 float additions and 2 products of magnitude <= emax: 4 u emax per step
		const double relW = Tr::norm ? (static_cast<double>(n) * 4.0 * kU * std::max(emax, 1.0e-30) / E + kNlmsEps / E + 3.0 * kU) : 0.0;
		for (size_t k = 0; k < T; k++) {
			const double delta = k_ * static_cast<double>(e) * static_cast<double>(win[k]);
			const double bound = 4.0 * kU * (std::fabs(static_cast<double>(cp[k])) + std::fabs(delta)) + std::fabs(delta) * relW;
			const double got = static_cast<double>(cn[k]) - static_cast<double>(cp[k]);
			CHECK_MESSAGE(std::fabs(static_cast<double>(cn[k]) - (static_cast<double>(cp[k]) + delta)) <= bound,
			              "n=" << n << " tap " << k << " new " << cn[k] << " prev " << cp[k] << " delta lib " << got << " DEF " << delta << " bound " << bound);
			maxAbsC = std::max(maxAbsC, std::fabs(static_cast<double>(cn[k])));
			maxDelta = std::max(maxDelta, std::fabs(delta));
		}
	}

	template <class Q> static i128 satq(i128 v) {
		const i128 hi = (static_cast<i128>(1) << Q::fb) - 1, lo = -(static_cast<i128>(1) << Q::fb);
		return v < lo ? lo : (v > hi ? hi : v);
	}

	void stepQ(const std::vector<S> &cp, const std::vector<S> &cn, S y, S e, S d, double E) {
		const double fbScale = std::ldexp(1.0, Tr::fb);
		i128 acc = 0;
		for (size_t k = 0; k < T; k++) acc += static_cast<i128>(cp[k]) * static_cast<i128>(win[k]);
		const i128 yi = acc >> (Tr::fb - static_cast<int>(ps));
		const i128 hi = (static_cast<i128>(1) << Tr::fb) - 1, lo = -(static_cast<i128>(1) << Tr::fb);
		REQUIRE_MESSAGE((yi >= lo && yi <= hi), "test design: y out of range at n=" << n);
		if (clean) REQUIRE_MESSAGE(acc >= 0, "test design: negative accumulator at n=" << n);
		CHECK_MESSAGE(static_cast<i128>(y) == yi, "n=" << n << " y " << y << " exact floor(acc / 2^(fb-ps)) " << static_cast<long long>(yi));
		const i128 ei = static_cast<i128>(d) - yi;
		REQUIRE_MESSAGE((ei >= lo && ei <= hi), "test design: e out of range at n=" << n);
		if (clean) REQUIRE_MESSAGE(ei >= 0, "test design: negative error at n=" << n);
		CHECK_MESSAGE(static_cast<i128>(e) == ei, "n=" << n << " e " << e << " expected d - y " << static_cast<long long>(ei));
		// C5: the double DEF y (in LSBs) is above the floor result by less than one LSB (one sided floor error, strictly < 1)
		const double ydefLsb = std::ldexp(static_cast<double>(acc), -(Tr::fb - static_cast<int>(ps)));
		const double fl = ydefLsb - static_cast<double>(y);
		CHECK_MESSAGE((fl >= 0.0 && fl < 1.0), "n=" << n << " floor error " << fl);
		// energy field and update
		double wDefLsb = 0, wErr = 0;
		double deltaBoundFactor = 0, deltaConst = 0;
		if constexpr (Tr::norm) {
			const double dl = Tr::fb == 31 ? 256.0 / fbScale : 5.0 / fbScale;  // regularisation of the code, not in the DOC
			const double rR = Tr::fb == 31 ? kRecipRel31 : kRecipRel15;
			const double errE = Tr::fb == 31 ? 2.0 * static_cast<double>(n) / fbScale : static_cast<double>(n) / fbScale;
			// energy field of the instance (derived one sided / symmetric bounds, header)
			const double Elsb = E * fbScale;
			const double lib = static_cast<double>(energyField());
			if (Tr::fb == 31) CHECK_MESSAGE((lib <= Elsb + 1e-3 && lib > Elsb - 2.0 * static_cast<double>(n) - 1e-3), "n=" << n << " energy " << lib << " exact " << Elsb);
			else CHECK_MESSAGE(std::fabs(lib - Elsb) < static_cast<double>(n) + 1e-3, "n=" << n << " energy " << lib << " exact " << Elsb);
			wDefLsb = static_cast<double>(e) * static_cast<double>(mu) / (fbScale * E);
			wErr = 1.0 / E + 1.0 + std::fabs(wDefLsb) * (rR + errE / E + dl / E) * 1.0001;
			deltaBoundFactor = wErr / fbScale;
			deltaConst = Tr::fb == 31 ? 2.0 : 1.0;
		}
		for (size_t k = 0; k < T; k++) {
			const double xv = static_cast<double>(win[k]);
			double defLsb, bound;
			if constexpr (Tr::norm) {
				defLsb = wDefLsb * xv / fbScale;
				bound = deltaBoundFactor * std::fabs(xv) + deltaConst;
			} else {
				defLsb = static_cast<double>(mu) * static_cast<double>(e) * xv / std::ldexp(1.0, 2 * Tr::fb);
				bound = Tr::fb == 31 ? 3.0 : 2.0;  // strict
			}
			const i128 cnew = static_cast<i128>(cn[k]);
			const i128 cexp = static_cast<i128>(cp[k]);
			if constexpr (!Tr::norm) {
				if (exact) {
					// q15: alpha = floor(e mu / 2^15), c += floor(alpha x / 2^15); q31: alpha = floor(e mu / 2^31), c += 2 floor(alpha x / 2^32) (source, DOC silent)
					const i128 alpha = (static_cast<i128>(e) * static_cast<i128>(mu)) >> Tr::fb;
					const i128 dl = Tr::fb == 15 ? ((alpha * static_cast<i128>(win[k])) >> 15) : (((alpha * static_cast<i128>(win[k])) >> 32) * 2);
					CHECK_MESSAGE(cnew == satq<Tr>(cexp + dl), "n=" << n << " tap " << k << " exact update model " << static_cast<long long>(dl) << " lib " << static_cast<long long>(cnew - cexp));
				}
			}
			REQUIRE_MESSAGE((cexp + static_cast<i128>(std::llround(defLsb)) > lo + 4 && cexp + static_cast<i128>(std::llround(defLsb)) < hi - 4),
			                "test design: coefficient saturation at n=" << n);
			const double diff = static_cast<double>(cnew - cexp) - defLsb;
			if constexpr (Tr::norm) CHECK_MESSAGE(std::fabs(diff) <= bound, "n=" << n << " tap " << k << " update lib " << static_cast<long long>(cnew - cexp) << " DEF " << defLsb << " bound " << bound);
			else CHECK_MESSAGE(std::fabs(diff) < bound, "n=" << n << " tap " << k << " update lib " << static_cast<long long>(cnew - cexp) << " DEF " << defLsb << " bound " << bound);
		}
		(void)wDefLsb;
	}
	double energyField() const {
		if constexpr (Tr::norm && Tr::isQ) return static_cast<double>(f.inst.energy);
		return 0.0;
	}
};

template <class Tr> void runTeacher(size_t T, size_t N, uint64_t seed, bool clean, unsigned ps, bool exact = false) {
	const Data<Tr> dt = makeData<Tr>(T, N, seed, clean, ps);
	INFO("T=" << T << " ps=" << ps << " clean=" << clean << " N=" << N);
	Teacher<Tr> t(T, dt.c0, dt.mu, ps, clean);
	t.exact = exact;
	for (size_t n = 0; n < dt.x.size(); n++) t.step(dt.x[n], dt.d[n]);
}

template <class Tr> void teacherAll(const std::vector<size_t> &Ts, bool clean, const std::vector<unsigned> &pss, bool exact = false) {
	for (size_t T : Ts) {
		if constexpr (Tr::hasPs) {
			for (unsigned ps : pss) runTeacher<Tr>(T, 48, 7 + T, clean, ps, exact);
		} else {
			(void)pss;
			runTeacher<Tr>(T, 48, 7 + T, clean, 0, exact);
		}
	}
}

// postShift sweep: mu = 0 (the coefficients stay), non-negative data, c scaled so that y = 2^ps sum c x <= 0.9 (no saturation, no window wrap), d = 0.99 (e >= 0);
// every sample is checked against the exact floor model, the energy field and the (zero) update.
template <class Tr> void psSweep(const std::vector<size_t> &Ts, const std::vector<unsigned> &pss) {
	typedef typename Tr::S S;
	for (size_t T : Ts) {
		for (unsigned ps : pss) {
			Rng g(300 + T * 16 + ps);
			std::vector<S> c0;
			for (size_t k = 0; k < T; k++) {
				const double u = g.range(0.3, 1.0);
				c0.push_back(fromReal<Tr>(std::min(0.8, u * 0.9 / (static_cast<double>(T) * 0.3 * std::ldexp(1.0, static_cast<int>(ps))))));
			}
			INFO("T=" << T << " ps=" << ps);
			Teacher<Tr> t(T, c0, fromReal<Tr>(0.0), ps, true);
			for (size_t n = 0; n < 20; n++) {
				const double xv = g.range(0.25, 0.3);
				t.step(fromReal<Tr>(xv), fromReal<Tr>(0.99));
			}
		}
	}
}

// Window wrap / emulation cases (header B-DLMS-5, B-DLMS-6): mu = 0, T samples of x with the coefficients all cval, d = 0; the last output (full window)
// is checked against the saturated result of the DOC. y is asserted first, so it is the first failing assertion.
template <class Tr> void windowCase(size_t T, unsigned ps, int cval, int xval, int expectY, bool checkE) {
	typedef typename Tr::S S;
	Filt<Tr> f(T, T, std::vector<S>(T, static_cast<S>(cval)), fromReal<Tr>(0.0), ps);
	std::vector<S> y, e;
	f.run(std::vector<S>(T, static_cast<S>(xval)), std::vector<S>(T, 0), 0, T, y, e);
	CHECK_MESSAGE(y.back() == expectY, "T=" << T << " ps=" << ps << " c=" << cval << " x=" << xval << " y " << y.back() << " intended " << expectY);
	if (checkE) CHECK(e.back() == -expectY);
}
// plain variants inside the 32 bit window (T <= 3, scalar path, positive accumulator): y saturates at 32767 (DOC "saturated")
// x = 3000 (not full scale: a norm filter whose energy reaches full scale hits B-DLMS-7): acc = 3 * 32767 * 3000 = 2.9e8, acc >> (15 - ps) > 32767 for ps >= 5
template <class Tr> void saturateInside() {
	for (unsigned ps : {5u, 7u, 14u}) windowCase<Tr>(3, ps, 32767, 3000, 32767, true);
}
// norm q15 whose energy reaches full scale: T = 1, x = 32767: E = floor(32767^2 / 2^15) = 32766, the code evaluates (q15_t) energy + DELTA_Q15 = 32771 and passes it as q15 (wraps to
// -32765: arm_recip_q15 is documented for positive input only). Intended: the weight has the sign of e, so with e > 0 and x > 0 the coefficient rises.
template <class Tr> void energyFullScale() {
	typedef typename Tr::S S;
	Filt<Tr> f(1, 1, std::vector<S>(1, 0), static_cast<S>(16384), 0);
	std::vector<S> y, e;
	f.run(std::vector<S>(1, 32767), std::vector<S>(1, 16000), 0, 1, y, e);
	CHECK(e[0] == 16000);
	CHECK_MESSAGE(f.coeffs()[0] > 0, "coefficient " << f.coeffs()[0] << " after an update with e = 16000, x = 32767");
}
// zero filter, T >= 4: nothing for the pair intrinsics to overflow on (all packed values 0): only the output guards, the state and the count are exercised
template <class Tr> void zeroFilterOutputs() {
	typedef typename Tr::S S;
	for (size_t T = 4; T <= 9; T++) {
		for (size_t n = 1; n <= 9; n++) {
			Filt<Tr> f(T, 9, std::vector<S>(T, 0), fromReal<Tr>(0.25), 0);
			std::vector<S> x(n, 0), d(n, 1000), y, e;
			f.run(x, d, 0, n, y, e);
			for (size_t i = 0; i < n; i++) {
				CHECK_MESSAGE(y[i] == 0, "T=" << T << " n=" << n << " y[" << i << "]");
				CHECK_MESSAGE(e[i] == 1000, "T=" << T << " n=" << n << " e[" << i << "]");
			}
		}
	}
}

// ---------------------------------------------------------------------------------------------------------------
// Double DEF of a complete run (for partitions and final coefficients)
// ---------------------------------------------------------------------------------------------------------------
struct DefRun {
	std::vector<double> y, e, c;
	double Bmax = 0, Dmax = 0, xmax = 0, Emin = 1e30, Emax = 0;
};
template <class Tr> DefRun defRun(const Data<Tr> &dt) {
	DefRun r;
	const size_t T = dt.T;
	std::vector<double> c(T), w(T, 0.0);
	for (size_t k = 0; k < T; k++) c[k] = toReal<Tr>(dt.c0[k]);
	const double mu = toReal<Tr>(dt.mu);
	const double g = std::ldexp(1.0, static_cast<int>(dt.ps));
	for (size_t n = 0; n < dt.x.size(); n++) {
		w.erase(w.begin());
		w.push_back(toReal<Tr>(dt.x[n]));
		double y = 0, E = 0;
		for (size_t k = 0; k < T; k++) {
			y += c[k] * w[k];
			E += w[k] * w[k];
		}
		y *= g;
		const double e = toReal<Tr>(dt.d[n]) - y;
		r.y.push_back(y);
		r.e.push_back(e);
		const double s = Tr::norm ? mu / E : mu;
		for (size_t k = 0; k < T; k++) {
			const double dl = s * e * w[k];
			c[k] += dl;
			r.Bmax = std::max(r.Bmax, std::fabs(c[k]));
			r.Dmax = std::max(r.Dmax, std::fabs(dl));
			r.xmax = std::max(r.xmax, std::fabs(w[k]));
		}
		r.Emin = std::min(r.Emin, E);
		r.Emax = std::max(r.Emax, E);
	}
	r.c = c;
	return r;
}

template <class Tr> size_t cleanLength(const Data<Tr> &dt) {
	const DefRun r = defRun<Tr>(dt);
	const double lim = 24.0 * lsb<Tr>();
	size_t m = 0;
	while (m < r.e.size() && r.e[m] >= lim) m++;
	return m;
}

// Accumulated float bound (header: contraction argument, local errors add): returns {coefficient bound (2-norm), y bound}
template <class Tr> std::pair<double, double> floatAccum(const Data<Tr> &dt, const DefRun &r) {
	const double T = static_cast<double>(dt.T), N = static_cast<double>(dt.x.size());
	const double mu = toReal<Tr>(dt.mu);
	const double relW = Tr::norm ? (N * 4.0 * kU * r.Emax / r.Emin + kNlmsEps / r.Emin + 3.0 * kU) : 0.0;
	const double yLocal = gammaT(dt.T) * T * r.Bmax * r.xmax;
	const double inj = (Tr::norm ? mu / std::sqrt(r.Emin) : mu * std::sqrt(T) * r.xmax) * yLocal;
	const double stepB = std::sqrt(T) * (4.0 * kU * (r.Bmax + r.Dmax) + r.Dmax * relW) + inj;
	const double cB = N * stepB;
	const double xnorm = std::sqrt(r.Emax);
	return {cB, xnorm * cB + yLocal + kU * (r.Bmax * T * r.xmax + 1.0)};
}

std::vector<size_t> partition(size_t N, const std::vector<size_t> &pat) {
	std::vector<size_t> p;
	size_t sum = 0, i = 0;
	while (sum < N) {
		const size_t b = std::min(pat[i % pat.size()], N - sum);
		p.push_back(b);
		sum += b;
		i++;
	}
	return p;
}

template <class Tr> void continuity(const std::vector<size_t> &Ts, bool clean, unsigned ps) {
	typedef typename Tr::S S;
	for (size_t T : Ts) {
		const Data<Tr> dt = makeData<Tr>(T, 40, 21 + T, clean, ps);
		const size_t N = dt.x.size();
		const std::vector<std::vector<size_t>> pats = {{N}, {1}, {2}, {3}, {4}, {5}, {7}, {1, 2, 3, 4, 5, 7}, {9, 1, 4}};
		const DefRun def = defRun<Tr>(dt);
		std::vector<S> y0, e0, c0;
		for (size_t pi = 0; pi < pats.size(); pi++) {
			const std::vector<size_t> p = partition(N, pats[pi]);
			const size_t B = *std::max_element(p.begin(), p.end());
			Filt<Tr> f(T, B, dt.c0, dt.mu, ps);
			std::vector<S> y, e;
			size_t off = 0;
			for (size_t b : p) {
				f.run(dt.x, dt.d, off, b, y, e);
				off += b;
			}
			REQUIRE(y.size() == N);
			if constexpr (Tr::isQ) {
				if (pi == 0) {
					y0 = y;
					e0 = e;
					c0 = f.coeffs();
				} else {
					CHECK_MESSAGE(y == y0, "T=" << T << " partition " << pi << ": output differs from one long block");
					CHECK_MESSAGE(e == e0, "T=" << T << " partition " << pi << ": error differs from one long block");
					CHECK_MESSAGE(f.coeffs() == c0, "T=" << T << " partition " << pi << ": coefficients differ from one long block");
				}
				// y against the double DEF, one-sided floor error accumulated with the coefficient error (header): N sqrt(T) (local + 1 LSB) LSBs
				if (!Tr::norm) {
					const double loc = (Tr::fb == 31 ? 3.0 : 2.0) + 1.0;
					const double cBound = static_cast<double>(N) * std::sqrt(static_cast<double>(T)) * loc;
					const std::vector<S> cf = f.coeffs();
					for (size_t k = 0; k < T; k++) {
						const double cdef = std::ldexp(def.c[k], Tr::fb);
						CHECK_MESSAGE(std::fabs(static_cast<double>(cf[k]) - cdef) <= cBound, "T=" << T << " partition " << pi << " tap " << k << " lib " << cf[k] << " DEF " << cdef << " bound " << cBound);
					}
				}
			} else {
				const std::pair<double, double> bnd = floatAccum<Tr>(dt, def);
				const std::vector<S> cf = f.coeffs();
				for (size_t k = 0; k < T; k++) CHECK_MESSAGE(std::fabs(static_cast<double>(cf[k]) - def.c[k]) <= bnd.first, "T=" << T << " partition " << pi << " tap " << k << " lib " << cf[k] << " DEF " << def.c[k] << " bound " << bnd.first);
				for (size_t n = 0; n < N; n++) {
					CHECK_MESSAGE(std::fabs(static_cast<double>(y[n]) - def.y[n]) <= bnd.second, "T=" << T << " partition " << pi << " n=" << n << " y " << y[n] << " DEF " << def.y[n] << " bound " << bnd.second);
					CHECK_MESSAGE(std::fabs(static_cast<double>(e[n]) - def.e[n]) <= bnd.second, "T=" << T << " partition " << pi << " n=" << n << " e " << e[n] << " DEF " << def.e[n] << " bound " << bnd.second);
				}
			}
		}
	}
}

// two instances with different data run interleaved: each equals its own isolated run (bitwise: identical call sequences)
template <class Tr> void twoInstances(const std::vector<size_t> &Ts, bool clean, unsigned ps) {
	typedef typename Tr::S S;
	const size_t B = 4;
	for (size_t T : Ts) {
		const Data<Tr> a = makeData<Tr>(T, 24, 5 + T, clean, ps), b = makeData<Tr>(T, 24, 55 + T, clean, ps);
		const size_t N = std::min(a.x.size(), b.x.size()) / B * B;  // whole blocks of B
		std::vector<S> ya, ea, yb, eb, ra, rea, rb, reb;
		{
			Filt<Tr> f(T, B, a.c0, a.mu, ps);
			for (size_t off = 0; off < N; off += B) f.run(a.x, a.d, off, B, ra, rea);
		}
		{
			Filt<Tr> f(T, B, b.c0, b.mu, ps);
			for (size_t off = 0; off < N; off += B) f.run(b.x, b.d, off, B, rb, reb);
		}
		Filt<Tr> fa(T, B, a.c0, a.mu, ps), fb(T, B, b.c0, b.mu, ps);
		for (size_t off = 0; off < N; off += B) {
			fa.run(a.x, a.d, off, B, ya, ea);
			fb.run(b.x, b.d, off, B, yb, eb);
		}
		CHECK_MESSAGE(ya == ra, "T=" << T << ": instance A output changed by instance B");
		CHECK_MESSAGE(ea == rea, "T=" << T << ": instance A error changed by instance B");
		CHECK_MESSAGE(yb == rb, "T=" << T << ": instance B output changed by instance A");
		CHECK_MESSAGE(eb == reb, "T=" << T << ": instance B error changed by instance A");
		CHECK(ya != yb);
	}
}

// re-init: after processing, init again with the same coefficients: the run repeats bitwise (state, energy and x0 are reset)
template <class Tr> void reinit(const std::vector<size_t> &Ts, bool clean, unsigned ps) {
	typedef typename Tr::S S;
	const size_t B = 5;
	for (size_t T : Ts) {
		const Data<Tr> a = makeData<Tr>(T, 20, 9 + T, clean, ps);
		const size_t N = a.x.size() / B * B;
		Filt<Tr> f(T, B, a.c0, a.mu, ps);
		std::vector<S> y1, e1, y2, e2;
		for (size_t off = 0; off < N; off += B) f.run(a.x, a.d, off, B, y1, e1);
		// re-init into the same arrays: coefficients restored by the caller, state poisoned first
		std::memcpy(f.c.get(), a.c0.data(), T * sizeof(S));
		std::memset(static_cast<void *>(f.st.get()), 0xCD, (T + B - 1) * sizeof(S));
		Tr::init(&f.inst, T, f.c.get(), f.st.get(), a.mu, B, ps);
		for (size_t off = 0; off < N; off += B) f.run(a.x, a.d, off, B, y2, e2);
		CHECK_MESSAGE(y1 == y2, "T=" << T << ": output after re-init differs from the first run");
		CHECK_MESSAGE(e1 == e2, "T=" << T << ": error after re-init differs from the first run");
		CHECK_MESSAGE(f.coeffs() != a.c0, "T=" << T << ": the run did not adapt (test design)");
	}
}

// init: fields hold the arguments, the (poisoned, tight) state is zeroed, nothing written behind T + B - 1 elements
template <class Tr> void initTest() {
	typedef typename Tr::S S;
	for (size_t T : {size_t(1), size_t(2), size_t(3), size_t(4), size_t(5), size_t(9)}) {
		for (size_t B : {size_t(1), size_t(2), size_t(8)}) {
			for (unsigned ps : {0u, 3u}) {
				std::vector<S> c0(T);
				for (size_t k = 0; k < T; k++) c0[k] = static_cast<S>(k + 1);
				const S mu = defaultMu<Tr>();
				Filt<Tr> f(T, B, c0, mu, ps);
				INFO("T=" << T << " B=" << B << " ps=" << ps);
				REQUIRE(f.inst.numTaps == T);
				REQUIRE(f.inst.pCoeffs == f.c.get());
				REQUIRE(f.inst.pState == f.st.get());
				CHECK(f.inst.mu == mu);
				if constexpr (Tr::hasPs) CHECK(static_cast<unsigned>(f.inst.postShift) == ps);
				if constexpr (Tr::norm) {
					CHECK(f.inst.energy == 0);
					CHECK(f.inst.x0 == 0);
					if constexpr (Tr::fb == 31) CHECK(f.inst.recipTable == armRecipTableQ31);
					if constexpr (Tr::fb == 15) CHECK(f.inst.recipTable == armRecipTableQ15);
				}
				bool allZero = true;
				for (size_t i = 0; i < T + B - 1; i++)
					if (f.st[i] != 0) allZero = false;
				CHECK_MESSAGE(allZero, "state buffer of T + B - 1 elements not zeroed");
				CHECK_MESSAGE(f.coeffs() == c0, "init changed the coefficients");
			}
		}
	}
}

// guard words behind the state: a block smaller than B and the largest block write nothing past T + B - 1 (the ASan builds see heap overruns)
template <class Tr> void outputTest(const std::vector<size_t> &Ts, bool clean) {
	typedef typename Tr::S S;
	for (size_t T : Ts) {
		const Data<Tr> a = makeData<Tr>(T, 10, 3 + T, clean, 0);
		REQUIRE(a.x.size() >= 9);
		for (size_t n = 1; n <= 9; n++) {
			Filt<Tr> f(T, 9, a.c0, a.mu, 0);
			std::vector<S> y, e;
			f.run(a.x, a.d, 0, n, y, e);
			CHECK_MESSAGE(y.size() == n, "T=" << T << " n=" << n);
		}
	}
}

// property of the LMS recursion (independent of the implementation): for d = h * x (a plant with the same T taps, zero start) the squared distance
// ||c - h||^2 never increases: ||d'||^2 = ||d||^2 - (2 mu - mu^2 ||x||^2 / s) e^2 / s (s = 1 plain, E normalised), with 0 < mu ||x||^2 / s < 2 (mu = 0.1 plain,
// ||x||^2 <= 0.25 T; norm mu = 0.5). Float slack per step: the local bound of the update (4 u (|c| + |delta|), plus the normalised relative terms).
template <class Tr> void identification(size_t T) {
	typedef typename Tr::S S;
	const size_t N = 300;
	Rng g(900 + T);
	std::vector<S> h(T), c0(T, 0), x(N), d(N);
	std::vector<double> hr(T), xr(T - 1, 0.0);
	for (size_t k = 0; k < T; k++) {
		h[k] = fromReal<Tr>(g.range(-0.5, 0.5));
		hr[k] = toReal<Tr>(h[k]);
	}
	for (size_t n = 0; n < N; n++) {
		x[n] = fromReal<Tr>(g.range(-0.5, 0.5));
		xr.push_back(toReal<Tr>(x[n]));
	}
	for (size_t n = 0; n < N; n++) {
		double s = 0;
		for (size_t k = 0; k < T; k++) s += hr[k] * xr[n + k];
		d[n] = fromReal<Tr>(s);
	}
	Teacher<Tr> t(T, c0, Tr::norm ? fromReal<Tr>(0.5) : fromReal<Tr>(0.1), 0, false);
	auto dist = [&](const std::vector<S> &c) {
		double s = 0;
		for (size_t k = 0; k < T; k++) {
			const double q = toReal<Tr>(c[k]) - hr[k];
			s += q * q;
		}
		return std::sqrt(s);
	};
	double prev = dist(t.f.coeffs());
	const double first = prev;
	double worstRise = 0;
	for (size_t n = 0; n < N; n++) {
		t.step(x[n], d[n]);  // also checks the step against the DEF
		const double cur = dist(t.f.coeffs());
		worstRise = std::max(worstRise, cur - prev);
		prev = cur;
	}
	// slack per step: the largest local coefficient error of the teacher bound (4 u (|c| + |delta|) per tap, T taps) plus the relative normalised terms
	const double Eref = std::max(t.emax, 1e-30);
	const double slack = std::sqrt(static_cast<double>(T)) * (4.0 * kU * (t.maxAbsC + t.maxDelta) + t.maxDelta * (Tr::norm ? (static_cast<double>(N) * 4.0 * kU * Eref / std::max(t.emin, 1e-30) + kNlmsEps / std::max(t.emin, 1e-30) + 3.0 * kU) : 0.0));
	CHECK_MESSAGE(worstRise <= slack, "T=" << T << ": the distance to the plant rose by " << worstRise << " in one step, allowed " << slack);
	CHECK_MESSAGE(prev < first, "T=" << T << ": final distance " << prev << " not below the start " << first);
}

// ---------------------------------------------------------------------------------------------------------------
// Hand derived sequences (all values dyadic: exact in float and in q15 / q31), real values:
//   T = 2, c = {0.25 (oldest tap), 0.5}, mu = 0.5, ps = 0
//   n=1: x=0.25, d=0.25: window [0, 0.25]: y = 0.5 * 0.25 = 0.125; e = 0.125; mu e = 0.0625; c0 += 0.0625 * 0 = 0.25; c1 += 0.0625 * 0.25 = 0.015625 -> 0.515625
//   n=2: x=0.5,  d=0.5:  window [0.25, 0.5]: y = 0.25 * 0.25 + 0.515625 * 0.5 = 0.0625 + 0.2578125 = 0.3203125; e = 0.1796875; mu e = 0.08984375;
//        c0 += 0.08984375 * 0.25 = 0.0224609375 -> 0.2724609375; c1 += 0.08984375 * 0.5 = 0.044921875 -> 0.560546875
// (Q15: 0.1796875 = 5888 / 2^15, mu = 16384: alpha = 2944 = 0.08984375 exactly, alpha * x(8192) >> 15 = 736 = 0.0224609375: no floor loses a bit.)
// ---------------------------------------------------------------------------------------------------------------
template <class Tr> void handPlain() {
	typedef typename Tr::S S;
	const std::vector<S> c0 = {fromReal<Tr>(0.25), fromReal<Tr>(0.5)};
	const std::vector<S> x = {fromReal<Tr>(0.25), fromReal<Tr>(0.5)}, d = {fromReal<Tr>(0.25), fromReal<Tr>(0.5)};
	Filt<Tr> f(2, 2, c0, fromReal<Tr>(0.5), 0);
	std::vector<S> y, e;
	f.run(x, d, 0, 2, y, e);
	CHECK(toReal<Tr>(y[0]) == 0.125);
	CHECK(toReal<Tr>(y[1]) == 0.3203125);
	CHECK(toReal<Tr>(e[0]) == 0.125);
	CHECK(toReal<Tr>(e[1]) == 0.1796875);
	const std::vector<S> c = f.coeffs();
	CHECK(toReal<Tr>(c[0]) == 0.2724609375);
	CHECK(toReal<Tr>(c[1]) == 0.560546875);
}

// Normalised, T = 1, c = 0.5, mu = 0.5, x = 0.5, d = 0.5: E = 0.25, y = 0.25 (exact), e = 0.25, w = mu e / E = 0.5, c += w x = 0.25 -> 0.75.
// Second sample x = 0.25, d = 0.5 (T = 1: E = x^2 = 0.0625, the old sample leaves the window): y = 0.75 * 0.25 = 0.1875, e = 0.3125, w = 0.5 * 0.3125 / 0.0625 = 2.5
// (not representable in q), so only the first sample is hand checked and the second step is checked through y, e and the energy field.
template <class Tr> void handNorm() {
	typedef typename Tr::S S;
	Filt<Tr> f(1, 1, {fromReal<Tr>(0.5)}, fromReal<Tr>(0.5), 0);
	std::vector<S> y, e;
	f.run({fromReal<Tr>(0.5)}, {fromReal<Tr>(0.5)}, 0, 1, y, e);
	CHECK(toReal<Tr>(y[0]) == 0.25);
	CHECK(toReal<Tr>(e[0]) == 0.25);
	// delta / E and the arithmetic of the formats (header): float eps / E = 4.8e-7 relative, q31 delta / E = 2^-21, q15 5 / 2^15 / 0.25; the bound is
	// derived per format: relative error of the update <= rel + recip + floors
	double rel;
	double absB;
	if (Tr::isQ) {
		const double dl = Tr::fb == 31 ? 256.0 : 5.0;
		const double rR = Tr::fb == 31 ? kRecipRel31 : kRecipRel15;
		rel = dl / std::ldexp(1.0, Tr::fb) / 0.25 + rR;
		absB = 4.0 * lsb<Tr>();  // floors (1/E + 1 + 2 LSB of the header, times |x| < 1)
		absB = (1.0 / 0.25 + 1.0 + 2.0) * lsb<Tr>();
	} else {
		rel = kNlmsEps / 0.25 + 6.0 * kU;
		absB = 0.0;
	}
	const double got = toReal<Tr>(f.coeffs()[0]);
	CHECK_MESSAGE(std::fabs(got - 0.75) <= rel * 0.25 + absB, "coefficient " << got << " expected 0.75 within " << rel * 0.25 + absB);
}

// ---------------------------------------------------------------------------------------------------------------
// Saturation of the coefficient update (DOC arm_lms_q15.c / arm_lms_q31.c: "the updation of filter coefficients are saturted") and of the q15 output
// (DOC arm_lms_q15.c: "the accumulator is saturated to yield a result in 1.15 format")
// ---------------------------------------------------------------------------------------------------------------
template <class Tr> void coefSaturatePos() {
	typedef typename Tr::S S;
	// T = 1, c = 0.9, x = 0.9, d = 0.95 (e = 0.95 - 0.81 = 0.14), mu = 0.9: update 0.9 * 0.14 * 0.9 = 0.113 -> 1.013 > 1: saturates at the maximum value
	const S big = fromReal<Tr>(0.9);
	Filt<Tr> f(1, 1, {big}, fromReal<Tr>(0.9), 0);
	std::vector<S> y, e;
	f.run({big}, {fromReal<Tr>(0.95)}, 0, 1, y, e);
	const S mx = static_cast<S>((static_cast<i128>(1) << Tr::fb) - 1);
	CHECK(f.coeffs()[0] == mx);
	// the next sample processes the saturated coefficient
	f.run({big}, {fromReal<Tr>(0.95)}, 0, 1, y, e);
	CHECK(f.coeffs()[0] == mx);
	const i128 acc = static_cast<i128>(mx) * static_cast<i128>(big);
	CHECK(static_cast<i128>(y[1]) == (acc >> Tr::fb));
}
template <class Tr> void coefSaturateNeg() {
	typedef typename Tr::S S;
	// T = 1, c = -0.9, x = 0.9, d = -0.95: y = -0.81, e = -0.14, update 0.9 * (-0.14) * 0.9 = -0.113 -> -1.013 < -1: saturates at the minimum value
	const S big = fromReal<Tr>(0.9);
	Filt<Tr> f(1, 1, {fromReal<Tr>(-0.9)}, fromReal<Tr>(0.9), 0);
	std::vector<S> y, e;
	f.run({big}, {fromReal<Tr>(-0.95)}, 0, 1, y, e);
	const S mn = static_cast<S>(-(static_cast<i128>(1) << Tr::fb));
	CHECK(f.coeffs()[0] == mn);
}
template <class Tr> void outSaturate() {
	typedef typename Tr::S S;
	// q15: T = 2 (scalar path), c = {0.9, 0.9}, window [0.9, 0.9] ... x = 0.9, 0.9: y = 2^ps * 1.62 (ps=0) saturates at 0x7FFF; e = d - y
	const S c = fromReal<Tr>(0.9);
	Filt<Tr> f(2, 2, {c, c}, fromReal<Tr>(0.0), 0);
	std::vector<S> y, e;
	f.run({c, c}, {fromReal<Tr>(0.5), fromReal<Tr>(0.5)}, 0, 2, y, e);
	// n=0: window [0, 0.9]: acc = 0.81 (in range); n=1: window [0.9, 0.9]: 1.62 -> saturates
	const S mx = static_cast<S>((static_cast<i128>(1) << Tr::fb) - 1);
	CHECK(static_cast<i128>(y[0]) == ((static_cast<i128>(c) * c) >> Tr::fb));
	CHECK(y[1] == mx);
	CHECK(static_cast<i128>(e[1]) == static_cast<i128>(fromReal<Tr>(0.5)) - mx);
}

// ---------------------------------------------------------------------------------------------------------------
// arm_recip_q31 / arm_recip_q15 direct
// ---------------------------------------------------------------------------------------------------------------
double recipReal31(q31_t in, q31_t out, uint32_t ret) { return std::ldexp(static_cast<double>(out), static_cast<int>(ret) - 31); }

void recipCheck31(q31_t in, size_t &bad, double &worst) {
	q31_t out = 0x55555555;
	const uint32_t ret = arm_recip_q31(in, &out, armRecipTableQ31);
	const uint32_t clz = static_cast<uint32_t>(__builtin_clz(static_cast<uint32_t>(in)));
	const double ideal = 2147483648.0 / static_cast<double>(in);  // 1 / (in / 2^31)
	const double got = recipReal31(in, out, ret);
	const double rel = std::fabs(got - ideal) / ideal;
	worst = std::max(worst, rel);
	if (ret != clz || rel > kRecipRel31) bad++;
}

}  // namespace

TEST_SUITE("DSP Filtering LMS") {

// ======================= arm_recip_q31 / arm_recip_q15 =======================
TEST_CASE("[C3 arm_recip_q31] hand derived values: 0.5 -> 1/0.5 = 2, 1 LSB -> 2^31, the shift is clz(in)") {
	// in = 0x40000000 = 0.5: xn = 0.5, 1/(2 xn) = 1.0 (clips to 0x7FFFFFFF in the 1.31 result), ret = clz = 1: value = 1.0 * 2^1 = 2 = 1 / 0.5.
	q31_t out = 0;
	uint32_t ret = arm_recip_q31(0x40000000, &out, armRecipTableQ31);
	CHECK(ret == 1u);
	CHECK(std::fabs(recipReal31(0x40000000, out, ret) - 2.0) <= 2.0 * kRecipRel31);
	// in = 0x7FFFFFFF: xn = 1 - 2^-31, 1/(2 xn) = 0.5 (+): out ~ 0x40000000, ret 1, value 1.0
	ret = arm_recip_q31(0x7FFFFFFF, &out, armRecipTableQ31);
	CHECK(ret == 1u);
	CHECK(std::fabs(recipReal31(0x7FFFFFFF, out, ret) - 1.0) <= 2.0 * kRecipRel31);
	// in = 1 (the smallest positive value): 1 / 2^-31 = 2^31: ret = 31, out ~ 1.0 (clipped)
	ret = arm_recip_q31(1, &out, armRecipTableQ31);
	CHECK(ret == 31u);
	CHECK(std::fabs(recipReal31(1, out, ret) - 2147483648.0) <= 2147483648.0 * kRecipRel31);
}

TEST_CASE("[C3 arm_recip_q31] the shift is clz(in) and the reciprocal is within 2^-27 relative for powers of two and their neighbours") {
	size_t bad = 0;
	double worst = 0;
	for (int k = 0; k <= 30; k++) {
		const int64_t p = static_cast<int64_t>(1) << k;
		for (int64_t dlt : {int64_t(-1), int64_t(0), int64_t(1), p / 3, p - 1, p + p / 2}) {
			const int64_t v = p + dlt;
			if (v < 1 || v > 0x7FFFFFFF) continue;
			recipCheck31(static_cast<q31_t>(v), bad, worst);
		}
	}
	CHECK_MESSAGE(bad == 0, bad << " inputs outside the bound, worst relative error " << worst << " bound " << kRecipRel31);
}

TEST_CASE("[C3 arm_recip_q31] the reciprocal is within 2^-27 relative over a strided sweep of the whole positive range and around the table bin edges") {
	size_t bad = 0, n = 0;
	double worst = 0;
	Rng g(77);
	for (int i = 0; i < 20000; i++) {
		const uint32_t r = g.next32();
		const q31_t v = static_cast<q31_t>(std::max<uint32_t>(1, r >> 1));
		recipCheck31(v, bad, worst);
		n++;
	}
	// bin edges: the top 7 bits of the normalised value change every 2^24 (the table index): the values on both sides of each edge, for several scales
	for (int sh = 0; sh < 24; sh += 5) {
		for (uint32_t b = 64; b < 128; b++) {
			for (int dlt : {-1, 0, 1}) {
				const int64_t v = ((static_cast<int64_t>(b) << 24) + dlt) >> sh;
				if (v < 1 || v > 0x7FFFFFFF) continue;
				recipCheck31(static_cast<q31_t>(v), bad, worst);
				n++;
			}
		}
	}
	CHECK_MESSAGE(bad == 0, bad << " of " << n << " inputs outside the bound, worst relative error " << worst << " bound " << kRecipRel31);
}

TEST_CASE("[C3 arm_recip_q31] the result of a larger input never exceeds the result of a smaller one by more than the accuracy bound (monotonic reciprocal)") {
	// 1/x decreases with x: in real units out_real(in) = out 2^(ret-31): compare neighbours of a geometric sweep
	double prev = 1e300;
	size_t bad = 0;
	for (int64_t v = 1; v <= 0x7FFFFFFF; v += std::max<int64_t>(1, v / 97)) {
		q31_t out;
		const uint32_t ret = arm_recip_q31(static_cast<q31_t>(v), &out, armRecipTableQ31);
		const double r = recipReal31(static_cast<q31_t>(v), out, ret);
		if (r > prev * (1.0 + 2.0 * kRecipRel31)) bad++;
		prev = r;
	}
	CHECK(bad == 0);
}

TEST_CASE("[C3 arm_recip_q15] the shift is clz(in) - 16 and the reciprocal is within 2^-13 relative for every input in [1, 32767] that is not a power of two") {
	size_t bad = 0;
	double worst = 0;
	for (int32_t v = 1; v <= 32767; v++) {
		if ((v & (v - 1)) == 0) continue;  // power of two: the exact result 1.0 does not fit 1.15 (separate case)
		q15_t out = 0x5555;
		const uint32_t ret = arm_recip_q15(static_cast<q15_t>(v), &out, armRecipTableQ15);
		const uint32_t sb = static_cast<uint32_t>(__builtin_clz(static_cast<uint32_t>(v))) - 17u;  // header: ret = signBits + 1
		const double ideal = 32768.0 / v;
		const double got = std::ldexp(static_cast<double>(out), static_cast<int>(ret) - 15);
		const double rel = std::fabs(got - ideal) / ideal;
		worst = std::max(worst, rel);
		if (ret != sb + 1 || rel > kRecipRel15) bad++;
	}
	CHECK_MESSAGE(bad == 0, bad << " inputs outside the bound, worst relative error " << worst << " bound " << kRecipRel15);
}

TEST_CASE("[C4 arm_recip_q15] a power of two input gives 1/in = 2^k: the exact 1.0 of the 1.15 mantissa is represented without sign change") {
	// in = 2^m: xn = 0.5, 1 / (2 xn) = 1.0, which is 32768 in 1.15: not representable; the correct (saturated) result is 0x7FFF (value 1 - 2^-15),
	// within the relative bound; a result that wraps to -32768 is a defect (the clip is commented out in utils.h)
	size_t bad = 0;
	for (int m = 0; m <= 14; m++) {
		q15_t out = 0x5555;
		const uint32_t ret = arm_recip_q15(static_cast<q15_t>(1 << m), &out, armRecipTableQ15);
		const double got = std::ldexp(static_cast<double>(out), static_cast<int>(ret) - 15);
		const double ideal = 32768.0 / static_cast<double>(1 << m);
		if (std::fabs(got - ideal) / ideal > kRecipRel15) bad++;
	}
	CHECK_MESSAGE(bad == 0, bad << " of 15 powers of two outside the bound");
}

// ======================= arm_lms_f32 =======================
#define LMS_REQ_COMMON(TR, FN)                                                                                           \
	TEST_CASE("[C4 " FN "] init stores numTaps, pointers, mu (and postShift, energy, x0, recipTable) and zeroes the tight state buffer of numTaps + blockSize - 1") { initTest<TR>(); }

LMS_REQ_COMMON(TrF32, "arm_lms_init_f32")
LMS_REQ_COMMON(TrQ15, "arm_lms_init_q15")
LMS_REQ_COMMON(TrQ31, "arm_lms_init_q31")
LMS_REQ_COMMON(TrNF32, "arm_lms_norm_init_f32")
LMS_REQ_COMMON(TrNQ15, "arm_lms_norm_init_q15")
LMS_REQ_COMMON(TrNQ31, "arm_lms_norm_init_q31")

const std::vector<size_t> kAllT = {1, 2, 3, 4, 5, 6, 7, 8, 9};
const std::vector<size_t> kT13 = {1, 2, 3};
const std::vector<size_t> kT49 = {4, 5, 6, 7, 8, 9};
const std::vector<size_t> kT29 = {2, 3, 4, 5, 6, 7, 8, 9};
const std::vector<size_t> kT1 = {1};

TEST_CASE("[C3 arm_lms_f32] the hand derived two tap sequence is exact") { handPlain<TrF32>(); }
TEST_CASE("[C3 arm_lms_q15] the hand derived two tap sequence is exact (Q15 values)") { handPlain<TrQ15>(); }
TEST_CASE("[C3 arm_lms_q31] the hand derived two tap sequence is exact (Q31 values)") { handPlain<TrQ31>(); }
TEST_CASE("[C3 arm_lms_norm_f32] the first sample of the hand derived one tap sequence gives y = 0.25, e = 0.25 and the update 0.25") { handNorm<TrNF32>(); }
TEST_CASE("[C3 arm_lms_norm_q15] the first sample of the hand derived one tap sequence gives y = 0.25, e = 0.25 and the update 0.25") { handNorm<TrNQ15>(); }
TEST_CASE("[C3 arm_lms_norm_q31] the first sample of the hand derived one tap sequence gives y = 0.25, e = 0.25 and the update 0.25") { handNorm<TrNQ31>(); }

// ---- C3 / C5 teacher forced DEF, per sample ----
TEST_CASE("[C3 arm_lms_f32] sample by sample output, error and coefficient update match the DEF for numTaps 1 to 9") { teacherAll<TrF32>(kAllT, false, {0}); }
TEST_CASE("[C3 arm_lms_norm_f32] sample by sample output, error and normalised coefficient update match the DEF for numTaps 1 to 9") { teacherAll<TrNF32>(kAllT, false, {0}); }

TEST_CASE("[C3 arm_lms_q31] with non-negative data the output and error are the exact floor model and the update is within 3 LSB of the DEF, numTaps 1 to 9, postShift 0 and 1") {
	teacherAll<TrQ31>(kAllT, true, {0, 1});
}
TEST_CASE("[C3 arm_lms_q31] with signed data the output and error are the exact floor model and the update is within 3 LSB of the DEF, numTaps 1 to 9, postShift 0 and 1" *
          KNOWN_ISSUE_UB_HIDDEN("B-DLMS-1/2: arm_lms_q31.c shifts a negative value left (coef << 1U, acc_h << uShift)")) {
	teacherAll<TrQ31>(kAllT, false, {0, 1});
}
TEST_CASE("[C3 arm_lms_norm_q31] with non-negative data the output, error, energy and the normalised update match the DEF within the derived bounds, numTaps 2 to 9, postShift 0 and 1") {
	teacherAll<TrNQ31>(kT29, true, {0, 1});
}
TEST_CASE("[C3 arm_lms_norm_q31] with non-negative data the output, error, energy and the normalised update match the DEF within the derived bounds, numTaps 1, postShift 0 and 1" *
          KNOWN_ISSUE_UB_HIDDEN("B-DLMS-4: arm_lms_norm_q31.c:120 shifts the energy left after the floor made it -1 (numTaps 1: E - x0^2 < 0 by the truncation)")) {
	teacherAll<TrNQ31>(kT1, true, {0, 1});
}
TEST_CASE("[C3 arm_lms_norm_q31] with signed data the output, error, energy and the normalised update match the DEF within the derived bounds, numTaps 1 to 9, postShift 0 and 1" *
          KNOWN_ISSUE_UB_HIDDEN("B-DLMS-1/2: arm_lms_norm_q31.c shifts a negative value left (coef << 1U, acc_h << uShift)")) {
	teacherAll<TrNQ31>(kAllT, false, {0, 1});
}

TEST_CASE("[C3 arm_lms_q15] with non-negative data the output and error are the exact floor model and the update is within 2 LSB of the DEF, numTaps 1 to 3 (scalar path), postShift 0 and 1") {
	teacherAll<TrQ15>(kT13, true, {0, 1});
}
TEST_CASE("[C3 arm_lms_q15] with non-negative data the output and error are the exact floor model and the update is within 2 LSB of the DEF, numTaps 4 to 9 (unrolled path), postShift 0 and 1" *
          KNOWN_ISSUE_UB_HIDDEN("B-DLMS-3: dsp/none.h __SMLALD / read_q15x2 shift packed pairs left in the unrolled path (numTaps >= 4)")) {
	teacherAll<TrQ15>(kT49, true, {0, 1});
}
TEST_CASE("[C3 arm_lms_q15] with signed data the output and error are the exact floor model and the update is within 2 LSB of the DEF, numTaps 1 to 9, postShift 0 and 1" *
          KNOWN_ISSUE_UB_HIDDEN("B-DLMS-2/3: arm_lms_q15.c acc_h << uShift of a negative upper half, and the SMLALD pair shifts of the unrolled path")) {
	teacherAll<TrQ15>(kAllT, false, {0, 1});
}
TEST_CASE("[C3 arm_lms_norm_q15] with non-negative data the output, error, energy and the normalised update match the DEF within the derived bounds, numTaps 1 to 3 (scalar path), postShift 0 and 1") {
	teacherAll<TrNQ15>(kT13, true, {0, 1});
}
TEST_CASE("[C3 arm_lms_norm_q15] with non-negative data the output, error, energy and the normalised update match the DEF within the derived bounds, numTaps 4 to 9 (unrolled path), postShift 0 and 1" *
          KNOWN_ISSUE_UB_HIDDEN("B-DLMS-3: dsp/none.h __SMLALD / read_q15x2 shift packed pairs left in the unrolled path (numTaps >= 4)")) {
	teacherAll<TrNQ15>(kT49, true, {0, 1});
}
TEST_CASE("[C3 arm_lms_norm_q15] with signed data the output, error, energy and the normalised update match the DEF within the derived bounds, numTaps 1 to 9, postShift 0 and 1" *
          KNOWN_ISSUE_UB_HIDDEN("B-DLMS-2/3: arm_lms_norm_q15.c acc_h << uShift of a negative upper half, and the SMLALD pair shifts of the unrolled path")) {
	teacherAll<TrNQ15>(kAllT, false, {0, 1});
}

// ---- postShift sweep, exact update model, window wrap ----
TEST_CASE("[C3 arm_lms_q31] with mu = 0 and non-negative data the output is the exact floor of acc / 2^(31-ps) for postShift 0, 1, 2, 3, 5, 7 and numTaps 1 to 9") {
	psSweep<TrQ31>(kAllT, {0, 1, 2, 3, 5, 7});
}
TEST_CASE("[C3 arm_lms_norm_q31] with mu = 0 and non-negative data the output is the exact floor of acc / 2^(31-ps) and the energy follows its bound for postShift 0, 1, 2, 3, 5, 7 and numTaps 2 to 9") {
	psSweep<TrNQ31>(kT29, {0, 1, 2, 3, 5, 7});
}
TEST_CASE("[C3 arm_lms_q15] with mu = 0 and non-negative data the output is the exact floor of acc / 2^(15-ps) for postShift 0, 1, 2, 3, 5, 7 and numTaps 1 to 3") {
	psSweep<TrQ15>(kT13, {0, 1, 2, 3, 5, 7});
}
TEST_CASE("[C3 arm_lms_q15] with mu = 0 and non-negative data the output is the exact floor of acc / 2^(15-ps) for postShift 0, 1, 2, 3, 5, 7 and numTaps 4 to 9" *
          KNOWN_ISSUE_UB_HIDDEN("B-DLMS-3: dsp/none.h __SMLALD / read_q15x2 shift packed pairs left in the unrolled path (numTaps >= 4)")) {
	psSweep<TrQ15>(kT49, {0, 1, 2, 3, 5, 7});
}
TEST_CASE("[C3 arm_lms_norm_q15] with mu = 0 and non-negative data the output is the exact floor of acc / 2^(15-ps) and the energy follows its bound for postShift 0, 1, 2, 3, 5, 7 and numTaps 1 to 3") {
	psSweep<TrNQ15>(kT13, {0, 1, 2, 3, 5, 7});
}
TEST_CASE("[C3 arm_lms_norm_q15] with mu = 0 and non-negative data the output is the exact floor of acc / 2^(15-ps) and the energy follows its bound for postShift 0, 1, 2, 3, 5, 7 and numTaps 4 to 9" *
          KNOWN_ISSUE_UB_HIDDEN("B-DLMS-3: dsp/none.h __SMLALD / read_q15x2 shift packed pairs left in the unrolled path (numTaps >= 4)")) {
	psSweep<TrNQ15>(kT49, {0, 1, 2, 3, 5, 7});
}
TEST_CASE("characterization: [C3 arm_lms_q15] with non-negative data the plain update is exactly c += floor(floor(e mu / 2^15) x / 2^15), numTaps 1 to 3 (detects a gain error of 2^-9)") {
	teacherAll<TrQ15>(kT13, true, {0, 1}, true);
}
TEST_CASE("characterization: [C3 arm_lms_q31] with non-negative data the plain update is exactly c += 2 floor(floor(e mu / 2^31) x / 2^32), numTaps 1 to 9") {
	teacherAll<TrQ31>(kAllT, true, {0, 1}, true);
}
TEST_CASE("[C4 arm_lms_q15] an accumulator that needs more than 32 bits after the postShift still saturates, numTaps 3, postShift 1, 5, 7, 14") { saturateInside<TrQ15>(); }
TEST_CASE("[C4 arm_lms_norm_q15] an accumulator that needs more than 16 bits after the postShift saturates, numTaps 3, postShift 1, 5, 7, 14") { saturateInside<TrNQ15>(); }
TEST_CASE("[C4 arm_lms_q15] numTaps 5, postShift 14, all coefficients and inputs 32767: the output saturates at 32767 (the 32 bit window of the accumulator wraps)" *
          KNOWN_ISSUE_UB("B-DLMS-5: arm_lms_q15.c:147-153 keeps a 32 bit window of the 34.30 accumulator (5 * 32767^2 >> 1 = 2.68e9 > 2^31): y = -32768, DOC: saturated to 1.15")) {
	windowCase<TrQ15>(5, 14, 32767, 32767, 32767, true);
}
TEST_CASE("[C4 arm_lms_q15] numTaps 5, postShift 14, coefficients 32767 and inputs -32768: the output saturates at -32768 (the 32 bit window of the accumulator wraps)" *
          KNOWN_ISSUE_UB("B-DLMS-5: arm_lms_q15.c:147-153 32 bit window of the accumulator: y = +32767, DOC: saturated to 1.15")) {
	windowCase<TrQ15>(5, 14, 32767, -32768, -32768, false);
}
TEST_CASE("[C4 arm_lms_norm_q15] numTaps 5, postShift 14, all coefficients and inputs 32767: the output saturates at 32767 and the error is -32767 (the 32 bit window wraps)" *
          KNOWN_ISSUE_UB("B-DLMS-5: arm_lms_norm_q15.c:165-171 32 bit window of the 34.30 accumulator: y = e = -32768, DOC: saturated to 1.15 (and 'no risk of internal overflow')")) {
	windowCase<TrNQ15>(5, 14, 32767, 32767, 32767, true);
}
TEST_CASE("[C4 arm_lms_norm_q15] numTaps 5, postShift 14, coefficients 32767 and inputs -32768: the output saturates at -32768 (the 32 bit window wraps)" *
          KNOWN_ISSUE_UB("B-DLMS-5: arm_lms_norm_q15.c:165-171 32 bit window of the accumulator: y = +32767, DOC: saturated to 1.15")) {
	windowCase<TrNQ15>(5, 14, 32767, -32768, -32768, false);
}
TEST_CASE("[C4 arm_lms_q15] numTaps 4, postShift 0, c = x = -32768: the accumulator 2^32 saturates the output at 32767 (host emulation of SMLALD adds two int products before widening)" *
          KNOWN_ISSUE_UB("B-DLMS-6: dsp/none.h:503 __SMLALD emulation sums two int products (2^30 + 2^30 overflows int) before widening to 64 bit: y = -32768; the target SMLALD accumulates in 64 bit, so this is a host emulation artefact (same as B-DCVQ15-2)")) {
	windowCase<TrQ15>(4, 0, -32768, -32768, 32767, false);
}
TEST_CASE("[C4 arm_lms_norm_q15] a full scale input raises the coefficient for a positive error (energy + DELTA_Q15 must not wrap negative)" *
          KNOWN_ISSUE_UB("B-DLMS-7: arm_lms_norm_q15.c:182 passes (q15_t) energy + DELTA_Q15 = 32771 to arm_recip_q15 as q15 (-32765): utils.h:139 shifts a negative value left; the coefficient falls to -1417 instead of rising")) { energyFullScale<TrNQ15>(); }
TEST_CASE("[C2 arm_lms_q15] a zero filter of 4 to 9 taps writes exactly n outputs and errors for blocks of 1 to 9 (guards intact, y = 0, e = d)") { zeroFilterOutputs<TrQ15>(); }
TEST_CASE("[C2 arm_lms_norm_q15] a zero filter of 4 to 9 taps writes exactly n outputs and errors for blocks of 1 to 9 (guards intact, y = 0, e = d)") { zeroFilterOutputs<TrNQ15>(); }

// ---- C4 state continuity: one long block == partitions (1,2,3,4,5,7 ... samples), compared with the DEF ----
TEST_CASE("[C4 arm_lms_f32] one block of 40 samples and the partitions 1, 2, 3, 4, 5, 7, 1-2-3-4-5-7 and 9-1-4 agree with the DEF within the accumulated bound, numTaps 1 to 9") { continuity<TrF32>(kAllT, false, 0); }
TEST_CASE("[C4 arm_lms_norm_f32] one block of 40 samples and the partitions 1, 2, 3, 4, 5, 7, 1-2-3-4-5-7 and 9-1-4 agree with the DEF within the accumulated bound, numTaps 1 to 9") { continuity<TrNF32>(kAllT, false, 0); }
TEST_CASE("[C4 arm_lms_q31] with non-negative data the partitions of 40 samples (1, 2, 3, 4, 5, 7, 1-2-3-4-5-7, 9-1-4) are bit identical to one block and the coefficients are within the derived bound of the DEF, numTaps 1 to 9") { continuity<TrQ31>(kAllT, true, 0); }
TEST_CASE("[C4 arm_lms_norm_q31] with non-negative data the partitions of 40 samples (1, 2, 3, 4, 5, 7, 1-2-3-4-5-7, 9-1-4) are bit identical to one block, numTaps 2 to 9") { continuity<TrNQ31>(kT29, true, 0); }
TEST_CASE("[C4 arm_lms_norm_q31] with non-negative data the partitions of 40 samples (1, 2, 3, 4, 5, 7, 1-2-3-4-5-7, 9-1-4) are bit identical to one block, numTaps 1" *
          KNOWN_ISSUE_UB_HIDDEN("B-DLMS-4: arm_lms_norm_q31.c:120 shifts the energy left after the floor made it -1 (numTaps 1)")) { continuity<TrNQ31>(kT1, true, 0); }
TEST_CASE("[C4 arm_lms_q15] with non-negative data the partitions of 40 samples (1, 2, 3, 4, 5, 7, 1-2-3-4-5-7, 9-1-4) are bit identical to one block and the coefficients are within the derived bound of the DEF, numTaps 1 to 3") { continuity<TrQ15>(kT13, true, 0); }
TEST_CASE("[C4 arm_lms_q15] with non-negative data the partitions of 40 samples (1, 2, 3, 4, 5, 7, 1-2-3-4-5-7, 9-1-4) are bit identical to one block and the coefficients are within the derived bound of the DEF, numTaps 4 to 9" *
          KNOWN_ISSUE_UB_HIDDEN("B-DLMS-3: dsp/none.h __SMLALD / read_q15x2 shift packed pairs left in the unrolled path (numTaps >= 4)")) { continuity<TrQ15>(kT49, true, 0); }
TEST_CASE("[C4 arm_lms_norm_q15] with non-negative data the partitions of 40 samples (1, 2, 3, 4, 5, 7, 1-2-3-4-5-7, 9-1-4) are bit identical to one block, numTaps 1 to 3") { continuity<TrNQ15>(kT13, true, 0); }
TEST_CASE("[C4 arm_lms_norm_q15] with non-negative data the partitions of 40 samples (1, 2, 3, 4, 5, 7, 1-2-3-4-5-7, 9-1-4) are bit identical to one block, numTaps 4 to 9" *
          KNOWN_ISSUE_UB_HIDDEN("B-DLMS-3: dsp/none.h __SMLALD / read_q15x2 shift packed pairs left in the unrolled path (numTaps >= 4)")) { continuity<TrNQ15>(kT49, true, 0); }

// ---- C4 two instances independent, re-init resets ----
TEST_CASE("[C4 arm_lms_f32] two instances processed interleaved give the results of their isolated runs, numTaps 1 to 9") { twoInstances<TrF32>(kAllT, false, 0); }
TEST_CASE("[C4 arm_lms_norm_f32] two instances processed interleaved give the results of their isolated runs, numTaps 1 to 9") { twoInstances<TrNF32>(kAllT, false, 0); }
TEST_CASE("[C4 arm_lms_q31] two instances processed interleaved give the results of their isolated runs, non-negative data, numTaps 1 to 9") { twoInstances<TrQ31>(kAllT, true, 0); }
TEST_CASE("[C4 arm_lms_norm_q31] two instances processed interleaved give the results of their isolated runs, non-negative data, numTaps 2 to 9") { twoInstances<TrNQ31>(kT29, true, 0); }
TEST_CASE("[C4 arm_lms_q15] two instances processed interleaved give the results of their isolated runs, non-negative data, numTaps 1 to 3") { twoInstances<TrQ15>(kT13, true, 0); }
TEST_CASE("[C4 arm_lms_norm_q15] two instances processed interleaved give the results of their isolated runs, non-negative data, numTaps 1 to 3") { twoInstances<TrNQ15>(kT13, true, 0); }

TEST_CASE("[C4 arm_lms_f32] re-initialising an instance resets the state: the repeated run is bit identical, numTaps 1 to 9") { reinit<TrF32>(kAllT, false, 0); }
TEST_CASE("[C4 arm_lms_norm_f32] re-initialising an instance resets the state, the energy and x0: the repeated run is bit identical, numTaps 1 to 9") { reinit<TrNF32>(kAllT, false, 0); }
TEST_CASE("[C4 arm_lms_q31] re-initialising an instance resets the state: the repeated run is bit identical, non-negative data, numTaps 1 to 9") { reinit<TrQ31>(kAllT, true, 0); }
TEST_CASE("[C4 arm_lms_norm_q31] re-initialising an instance resets the state, the energy and x0: the repeated run is bit identical, non-negative data, numTaps 2 to 9") { reinit<TrNQ31>(kT29, true, 0); }
TEST_CASE("[C4 arm_lms_q15] re-initialising an instance resets the state: the repeated run is bit identical, non-negative data, numTaps 1 to 3") { reinit<TrQ15>(kT13, true, 0); }
TEST_CASE("[C4 arm_lms_norm_q15] re-initialising an instance resets the state, the energy and x0: the repeated run is bit identical, non-negative data, numTaps 1 to 3") { reinit<TrNQ15>(kT13, true, 0); }

// ---- C2 output guards for blocks of 1 to 9 with a state sized for 9 ----
TEST_CASE("[C2 arm_lms_f32] a block of 1 to 9 samples writes exactly that many outputs and errors (guards intact, pRef unchanged), numTaps 1 to 9") { outputTest<TrF32>(kAllT, false); }
TEST_CASE("[C2 arm_lms_norm_f32] a block of 1 to 9 samples writes exactly that many outputs and errors (guards intact, pRef unchanged), numTaps 1 to 9") { outputTest<TrNF32>(kAllT, false); }
TEST_CASE("[C2 arm_lms_q31] a block of 1 to 9 samples writes exactly that many outputs and errors (guards intact, pRef unchanged), non-negative data, numTaps 1 to 9") { outputTest<TrQ31>(kAllT, true); }
TEST_CASE("[C2 arm_lms_norm_q31] a block of 1 to 9 samples writes exactly that many outputs and errors (guards intact, pRef unchanged), non-negative data, numTaps 2 to 9") { outputTest<TrNQ31>(kT29, true); }
TEST_CASE("[C2 arm_lms_q15] a block of 1 to 9 samples writes exactly that many outputs and errors (guards intact, pRef unchanged), non-negative data, numTaps 1 to 3") { outputTest<TrQ15>(kT13, true); }
TEST_CASE("[C2 arm_lms_norm_q15] a block of 1 to 9 samples writes exactly that many outputs and errors (guards intact, pRef unchanged), non-negative data, numTaps 1 to 3") { outputTest<TrNQ15>(kT13, true); }

// ---- LMS property: the distance to the plant never grows ----
TEST_CASE("[C3 arm_lms_f32] identifying a plant of 1 to 9 taps never increases the distance of the coefficients to the plant (LMS recursion invariant) and ends below the start") {
	for (size_t T : kAllT) identification<TrF32>(T);
}
TEST_CASE("[C3 arm_lms_norm_f32] identifying a plant of 1 to 9 taps never increases the distance of the coefficients to the plant (NLMS recursion invariant) and ends below the start") {
	for (size_t T : kAllT) identification<TrNF32>(T);
}

// ---- Saturation ----
TEST_CASE("[C4 arm_lms_q15] a coefficient update above 1 - 2^-15 saturates at 32767 and the saturated coefficient is used by the next sample") { coefSaturatePos<TrQ15>(); }
TEST_CASE("[C4 arm_lms_q15] a coefficient update below -1 saturates at -32768" *
          KNOWN_ISSUE_UB_HIDDEN("B-DLMS-2: arm_lms_q15.c acc_h << uShift of a negative upper half (negative accumulator)")) { coefSaturateNeg<TrQ15>(); }
TEST_CASE("[C4 arm_lms_q31] a coefficient update above 1 - 2^-31 saturates at the maximum value and the saturated coefficient is used by the next sample") { coefSaturatePos<TrQ31>(); }
TEST_CASE("[C4 arm_lms_q31] a coefficient update below -1 saturates at the minimum value" *
          KNOWN_ISSUE_UB_HIDDEN("B-DLMS-2: arm_lms_q31.c:162 acc_h << uShift of a negative upper half (negative accumulator); the update also hits B-DLMS-1")) { coefSaturateNeg<TrQ31>(); }
TEST_CASE("[C4 arm_lms_q15] an accumulator above 1.15 saturates the output at 32767 and the error is d - 32767") { outSaturate<TrQ15>(); }

}  // TEST_SUITE
