/*
 * CMSIS-DSP Filtering, lattice filters (work order LATT, phase P11 group G5): arm_fir_lattice_{f32,q15,q31} + arm_fir_lattice_init_*,
 * arm_iir_lattice_{f32,q15,q31} + arm_iir_lattice_init_*. Requirement ids: Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * ARM tests: NONE. ARM's suites MISC F64 / F32 / F16 / Q31 / Q15 / Q7 (Testing/desc.txt:2271-3146) cover only levinson_durbin,
 * correlate, conv and conv_partial; fir_lattice and iir_lattice have no pattern files and no test id (C1 has nothing to cover). The
 * FIR lattice DOC even says "@deprecated Those functions are no more tested nor maintained" (arm_fir_lattice_f32.c). Everything
 * here is DEF (C3, from the DOC and the source) and C2 / C4 / C5. No pattern file is used.
 *
 * DOC, FIR lattice (arm_fir_lattice_f32.c:53-74 group text; the same equations for q15 / q31):
 *     f0[n] = g0[n] = x[n];  fm[n] = fm-1[n] + km * gm-1[n-1];  gm[n] = km * fm-1[n] + gm-1[n-1] (m = 1..M);  y[n] = fM[n]
 *   pCoeffs = {k1, ..., kM}; "pState points to a state array of size numStages. The state variables (g values) hold previous inputs and
 *   are stored in the following order {g0[n], g1[n], g2[n] ... gM-1[n]}". Init: "Sets the values of the internal structure fields.
 *   Zeros out the values in the state buffer"; the init functions return void: there is no documented argument error.
 *   Fixed point (arm_fir_lattice_q15.c): no scaling text; the code computes P(a,k) = (a*k) >> 15 (floor), f = sat16(P + f), g = sat16(P + g)
 *   (__SSAT). q31 (arm_fir_lattice_q31.c:56-57 "In order to avoid overflows the input signal must be scaled down by 2*log2(numStages) bits"):
 *   P(a,k) = ((q63)a * k >> 32) << 1 (floor of the product to a multiple of 2 LSB), f = P + f, no saturation: a sum outside 1.31 wraps (signed
 *   overflow, undefined behaviour); the tests keep every f and g inside the range (the model checks it and the test REQUIREs it).
 *
 * DOC, IIR lattice (arm_iir_lattice_f32.c:50-122; same equations for q15 / q31):
 *     fN(n) = x(n);  fm-1(n) = fm(n) - km * gm-1(n-1);  gm(n) = km * fm-1(n) + gm-1(n-1) (m = N..1);  y(n) = vN gN(n) + ... + v0 g0(n)
 *   pkCoeffs = {kN, ..., k1} (time reversed), pvCoeffs = {vN, ..., v0} (numStages + 1 values), g0(n) = f0(n). "pState points to a state
 *   array of size numStages + blockSize" (the init takes blockSize). Hand reading of the code (all three types): the state array is a
 *   sliding window: sample n of a call works on state[n .. n+N]; at the end of the call the last N elements (state[blockSize ..
 *   blockSize+N-1]) are copied to state[0..N-1], so after a call state[i] = g_{N-1-i}(last sample), i = 0..N-1; a block must not be longer
 *   than the blockSize given to init (it would write behind the state). q15 (arm_iir_lattice_q15.c:50-57): "internal 64-bit accumulator ...
 *   multiplications yield a 2.30 result ... accumulated in 34.30 ... truncated to 34.15 by discarding low 15 bits. Lastly, the accumulator
 *   is saturated to yield a result in 1.15 format": y = sat16(floor(sum v*g / 2^15)). The host (portable, no ARM_MATH_DSP) code computes
 *   f = sat16(f - ((g*k) >> 15)), g = sat16(((f*k) >> 15) + g). q31 (arm_iir_lattice_q31.c:50-55): 2.62 accumulator, "if the accumulator
 *   result overflows it wraps around rather than clip" and "the 2.62 accumulator is saturated to 1.32 format and then truncated to 1.31":
 *   contradictory (the portable code does (q31_t)(acc >> 31): an out of range value is an implementation defined wrap): NOT tested; intermediate f, g:
 *   clip_q63_to_q31(f - ((q63)g*k >> 31)) etc.
 *   The init functions return void: no documented argument error.
 *
 * Models (all derived by hand from the equations above):
 *   Float / DEF: double. A "node" carries value v, abs-model a (the same recursion with |k|, |v|, |x| and positive sums) and e, a first order
 *   bound of the error of the type: every float multiplication and addition adds rel = 2^-24 * (abs-model value of the result), every floor product
 *   adds its absolute error (q15: < 1 LSB = 2^-15 in real units; q31 FIR product: floor to a multiple of 2 LSB, < 2 LSB; q31 IIR product < 1 LSB;
 *   the final floor of the IIR output < 1 LSB), errors propagate with |k|, |v| gains (an upper bound by monotonicity of the abs system).
 *   This holds for FIR and IIR, including the feedback of the IIR (the error of an old rounding is carried through the state). The test bound is
 *   1.01 * e (second order terms: relative O(n u) < 1e-3) + (8N+8) 2^-53 * a (the rounding of the double DEF itself, <= 8N+4 operations).
 *   Integer / LIB: exact integer models of the code (floor products, __SSAT, clip): these are the `characterization:` tests (the DOC does not
 *   say how products are rounded and for q15 FIR it says nothing about saturation); they REQUIRE that no accumulator overflow happens.
 *
 * Block structure (the units of the code): the FIR kernels unroll samples by 4 and stages by 4 (blockSize / numStages 1..9, 12, 17 and block
 * sizes 1, 2, 3, 4, 5, 7, 9, 16, 40 cover the unroll and its tails); ARM_MATH_LOOPUNROLL is defined in this build, so arm_iir_lattice_f32 also
 * runs its 4-stage unrolled path; only the q15 / q31 IIR kernels run the non-unrolled branch here (their unrolling is inside ARM_MATH_DSP). Sizes 1..9 are run.
 * Partitions with the same first block share their random data (the seed uses the first block): harmless.
 *
 * Data: FIR: |k| <= 1/(2M) (real), |x| <= 0.25 (q types): the sum |f| + |g| grows at most by (1 + |k|) per stage, so |f_M| <= 2 * 0.25 * (1+1/(2M))^M
 * <= 0.5 * 1.65 = 0.83 (1.65 >= e^0.5 >= (1+1/(2M))^M): never leaves 1.15 / 1.31 (the model checks). IIR: |k| <= 0.05 (q) / 0.1 (float; keeps the abs-model error gain of the feedback small, so the bound stays below the signal), |x| <= 2^-4 (q), |v| <= 0.5 (q);
 * the model checks that no intermediate clips, no accumulator overflows and the output is in range.
 *
 * Host vs target (C6): the host build runs the portable C path (no ARM_MATH_DSP, ARM_MATH_LOOPUNROLL). A Cortex-M4 build defines ARM_MATH_DSP: the IIR q15 / q31
 * kernels then take another path (QADD / QSUB saturating intermediates, SMLALD packing, the state copy loops of numStages - 1 elements without
 * ARM_MATH_LOOPUNROLL); results agree when nothing saturates. Float: FMA contraction (VFMA on the target) changes the rounding but the bounds above
 * hold for any single rounding per operation. The models use __int128 / int64: a 64 bit host compiler is required.
 *
 * Undefined behaviour found with UBSan (--no-skip), ids B-DLATT-n (the lead renumbers):
 *   B-DLATT-1  arm_fir_lattice_q31.c:119 "gnext0 = (gnext0 << 1U) + gcurr0" (and the same pattern at :113, :130 and in the scalar stage code) shifts a
 *              negative q31 left (undefined in C), reached with ordinary negative data inside the documented range. The plain tests use non-negative
 *              data and coefficients; the signed variants are KNOWN_ISSUE_UB_HIDDEN (intended result on the host). A different case, signed integer
 *              overflow (a sum outside 1.31, DOC: input must be scaled down), is only avoided by the data limits of the tests (the sweep skips such runs).
 *   B-DLATT-2  arm_fir_lattice_q15 with blockSize >= 4 (unrolled path) and a negative f value: the output store write_q15x2_ia(__PKHBT(fcurr0, fcurr1, 16))
 *              (arm_fir_lattice_q15.c:361-362, dsp/none.h:212 __PKHBT) shifts a negative value left; the same defect as B-DSPAR-2 and B-DFBQ-4 (the lead
 *              renumbers it into one bug). Blocks of 1..3 and non-negative data are clean; the signed variants with blocks of 4 or more are
 *              KNOWN_ISSUE_UB_HIDDEN. The IIR lattice kernels (portable path) are UBSan clean.
 *
 * NEEDS REVIEW (not tested or characterization only):
 *   - numStages 0 and blockSize 0: the FIR lattice reads pCoeffs[0] / pState[0] regardless of numStages (out of bounds for numStages 0); for the IIR
 *     the portable q kernels return a sum without the v0 g0 term of the input for numStages 0 (fnext is initialised to 0): not tested.
 *   - FIR lattice q15 saturation of f / g: DOC silent, the code clips (characterization). FIR lattice q31 overflow: DOC "scaled down", wraps (not tested).
 *   - IIR lattice q31 output overflow: DOC contradicts itself (see above), not tested. IIR q15: the DOC says the output is saturated: plain test.
 *   - Product rounding (floor) of all fixed point kernels is not stated in the DOC: the bound tests use "< 1 LSB per product" (derived from the code
 *     and valid for any truncating or rounding rule within one LSB), the exact model tests are characterization.
 *   - The IIR f32 "Algorithm" text has the order of the state (g values) only through the figure; the state layout was derived from the code.
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
	int64_t range(int64_t lo, int64_t hi) {  // inclusive, span <= 2^32
		const uint64_t span = static_cast<uint64_t>(hi - lo) + 1u;
		const uint32_t r = next32();
		return lo + static_cast<int64_t>((static_cast<uint64_t>(r) * span) >> 32);
	}
	double uni(double lo, double hi) {
		const int64_t r = range(0, 1000000);
		return lo + (hi - lo) * static_cast<double>(r) / 1000000.0;
	}
};

const double kU = 0x1p-24;  // float unit roundoff

// ---------------------------------------------------------------------------------------------------------------
// Error model and the double DEF (value, abs-model, error bound)
// ---------------------------------------------------------------------------------------------------------------
struct ErrModel {
	double rel, absProd, absOut;
};
struct Node {
	double v = 0, a = 0, e = 0;
};
Node mulN(const Node &n, double k, const ErrModel &m, bool roundsProduct) {
	Node r;
	r.v = n.v * k;
	r.a = n.a * std::fabs(k);
	r.e = n.e * std::fabs(k) + m.rel * r.a + (roundsProduct ? m.absProd : 0.0);
	return r;
}
Node addN(const Node &x, const Node &y, const ErrModel &m, double sign) {
	Node r;
	r.v = x.v + sign * y.v;
	r.a = x.a + y.a;
	r.e = x.e + y.e + m.rel * r.a;
	return r;
}
double tolOf(const Node &n, size_t stages) { return 1.01 * n.e + static_cast<double>(8 * stages + 8) * 0x1p-53 * n.a; }

// FIR lattice DEF: G[i] = g_i[n-1]
struct FirSim {
	std::vector<double> k;
	ErrModel m;
	std::vector<Node> G;
	FirSim(const std::vector<double> &kk, const ErrModel &mm) : k(kk), m(mm), G(kk.size()) {}
	Node step(double x) {
		Node f;
		f.v = x;
		f.a = std::fabs(x);
		Node gc = f;  // g_{m-1}[n]
		for (size_t i = 0; i < k.size(); i++) {
			const Node Gold = G[i];
			const Node p = mulN(Gold, k[i], m, true);
			const Node fn = addN(f, p, m, 1.0);
			const Node q = mulN(f, k[i], m, true);
			const Node gn = addN(q, Gold, m, 1.0);
			G[i] = gc;
			gc = gn;
			f = fn;
		}
		return f;
	}
};

// IIR lattice DEF: k[i] = k_{N-i}, v[i] = v_{N-i}, G[m] = g_m(n-1)
struct IirSim {
	std::vector<double> k, v;
	ErrModel m;
	std::vector<Node> G;
	IirSim(const std::vector<double> &kk, const std::vector<double> &vv, const ErrModel &mm) : k(kk), v(vv), m(mm), G(kk.size()) {}
	Node step(double x) {
		const size_t N = k.size();
		Node f;
		f.v = x;
		f.a = std::fabs(x);
		Node acc;
		std::vector<Node> nG(N);
		for (size_t i = 0; i < N; i++) {
			const size_t mm = N - i;
			const Node Gm1 = G[mm - 1];
			const Node p = mulN(Gm1, k[i], m, true);
			const Node fn = addN(f, p, m, -1.0);
			const Node q = mulN(fn, k[i], m, true);
			const Node gm = addN(Gm1, q, m, 1.0);
			acc = addN(acc, mulN(gm, v[i], m, false), m, 1.0);
			if (mm < N) nG[mm] = gm;
			f = fn;
		}
		acc = addN(acc, mulN(f, v[N], m, false), m, 1.0);
		nG[0] = f;
		G = nG;
		acc.e += m.absOut;
		return acc;
	}
};

// ---------------------------------------------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------------------------------------------
int64_t sat16(int64_t v, bool &clipped) {
	if (v > 32767) { clipped = true; return 32767; }
	if (v < -32768) { clipped = true; return -32768; }
	return v;
}
int64_t clip32(int64_t v, bool &clipped) {
	if (v > INT32_MAX) { clipped = true; return INT32_MAX; }
	if (v < INT32_MIN) { clipped = true; return INT32_MIN; }
	return v;
}

struct F32T {
	typedef float S;
	typedef arm_fir_lattice_instance_f32 FI;
	typedef arm_iir_lattice_instance_f32 II;
	static constexpr int bits = 0;
	static constexpr bool isF = true;
	static const char *sfx() { return "f32"; }
	static void fInit(FI *i, size_t m, S *k, S *st) { arm_fir_lattice_init_f32(i, static_cast<uint16_t>(m), k, st); }
	static void fRun(const FI *i, const S *x, S *y, size_t b) { arm_fir_lattice_f32(i, x, y, static_cast<uint32_t>(b)); }
	static void iInit(II *i, size_t m, S *k, S *v, S *st, size_t b) { arm_iir_lattice_init_f32(i, static_cast<uint16_t>(m), k, v, st, static_cast<uint32_t>(b)); }
	static void iRun(const II *i, const S *x, S *y, size_t b) { arm_iir_lattice_f32(i, x, y, static_cast<uint32_t>(b)); }
	static ErrModel fErr() { return {kU, 0, 0}; }
	static ErrModel iErr() { return {kU, 0, 0}; }
	// no integer model
	static int64_t fProd(int64_t, int64_t) { return 0; }
	static int64_t fFix(int64_t v, bool &, bool &) { return v; }
	static int64_t iProd(int64_t, int64_t) { return 0; }
	static int64_t iFix(int64_t v, bool &, bool &) { return v; }
	static int64_t iOut(i128 a, bool &, bool &) { return static_cast<int64_t>(a); }
};
struct Q15T {
	typedef int16_t S;
	typedef arm_fir_lattice_instance_q15 FI;
	typedef arm_iir_lattice_instance_q15 II;
	static constexpr int bits = 15;
	static constexpr bool isF = false;
	static const char *sfx() { return "q15"; }
	static void fInit(FI *i, size_t m, S *k, S *st) { arm_fir_lattice_init_q15(i, static_cast<uint16_t>(m), k, st); }
	static void fRun(const FI *i, const S *x, S *y, size_t b) { arm_fir_lattice_q15(i, x, y, static_cast<uint32_t>(b)); }
	static void iInit(II *i, size_t m, S *k, S *v, S *st, size_t b) { arm_iir_lattice_init_q15(i, static_cast<uint16_t>(m), k, v, st, static_cast<uint32_t>(b)); }
	static void iRun(const II *i, const S *x, S *y, size_t b) { arm_iir_lattice_q15(i, x, y, static_cast<uint32_t>(b)); }
	static ErrModel fErr() { return {0, 0x1p-15, 0}; }
	static ErrModel iErr() { return {0, 0x1p-15, 0x1p-15}; }
	static int64_t fProd(int64_t a, int64_t k) { return (a * k) >> 15; }
	static int64_t fFix(int64_t v, bool &, bool &clipped) { return sat16(v, clipped); }
	static int64_t iProd(int64_t a, int64_t k) { return (a * k) >> 15; }
	static int64_t iFix(int64_t v, bool &, bool &clipped) { return sat16(v, clipped); }
	static int64_t iOut(i128 a, bool &flag, bool &clipped) {
		if (a > (static_cast<i128>(1) << 62) || a < -(static_cast<i128>(1) << 62)) flag = true;
		return sat16(static_cast<int64_t>(a >> 15), clipped);
	}
};
struct Q31T {
	typedef int32_t S;
	typedef arm_fir_lattice_instance_q31 FI;
	typedef arm_iir_lattice_instance_q31 II;
	static constexpr int bits = 31;
	static constexpr bool isF = false;
	static const char *sfx() { return "q31"; }
	static void fInit(FI *i, size_t m, S *k, S *st) { arm_fir_lattice_init_q31(i, static_cast<uint16_t>(m), k, st); }
	static void fRun(const FI *i, const S *x, S *y, size_t b) { arm_fir_lattice_q31(i, x, y, static_cast<uint32_t>(b)); }
	static void iInit(II *i, size_t m, S *k, S *v, S *st, size_t b) { arm_iir_lattice_init_q31(i, static_cast<uint16_t>(m), k, v, st, static_cast<uint32_t>(b)); }
	static void iRun(const II *i, const S *x, S *y, size_t b) { arm_iir_lattice_q31(i, x, y, static_cast<uint32_t>(b)); }
	static ErrModel fErr() { return {0, 2 * 0x1p-31, 0}; }
	static ErrModel iErr() { return {0, 0x1p-31, 0x1p-31}; }
	static int64_t fProd(int64_t a, int64_t k) { return 2 * ((a * k) >> 32); }
	static int64_t fFix(int64_t v, bool &flag, bool &) {
		if (v > INT32_MAX || v < INT32_MIN) flag = true;  // wraps (undefined): premise violation
		return v;
	}
	static int64_t iProd(int64_t a, int64_t k) { return (a * k) >> 31; }
	static int64_t iFix(int64_t v, bool &, bool &clipped) { return clip32(v, clipped); }
	static int64_t iOut(i128 a, bool &flag, bool &) {
		if (a > (static_cast<i128>(1) << 62) || a < -(static_cast<i128>(1) << 62)) flag = true;
		const int64_t y = static_cast<int64_t>(a >> 31);
		if (y > INT32_MAX || y < INT32_MIN) flag = true;  // implementation defined wrap: not tested
		return y;
	}
};

constexpr int shiftDown(int bits, int d) { return bits > d ? bits - d : 0; }  // float instantiations never use it

template <class Tr>
double real(typename Tr::S v) {
	return std::ldexp(static_cast<double>(v), -Tr::bits);
}
template <class Tr>
std::vector<double> realVec(const std::vector<typename Tr::S> &v) {
	std::vector<double> r(v.size());
	for (size_t i = 0; i < v.size(); i++) r[i] = real<Tr>(v[i]);
	return r;
}
template <class Tr>
typename Tr::S from(double r) {  // exact for the dyadic values of the hand derivations
	if (Tr::isF) return static_cast<typename Tr::S>(r);
	return static_cast<typename Tr::S>(std::llround(std::ldexp(r, Tr::bits)));
}

// ---------------------------------------------------------------------------------------------------------------
// Integer models of the code (characterization)
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
struct IntFir {
	std::vector<int64_t> k, G;
	bool flag = false, clipped = false;
	explicit IntFir(const std::vector<typename Tr::S> &kk) : k(kk.begin(), kk.end()), G(kk.size(), 0) {}
	int64_t step(int64_t x) {
		int64_t f = x, gc = x;
		for (size_t i = 0; i < k.size(); i++) {
			const int64_t Gold = G[i];
			const int64_t fn = Tr::fFix(Tr::fProd(Gold, k[i]) + f, flag, clipped);
			const int64_t gn = Tr::fFix(Tr::fProd(f, k[i]) + Gold, flag, clipped);
			G[i] = gc;
			gc = gn;
			f = fn;
		}
		return f;
	}
};
template <class Tr>
struct IntIir {
	std::vector<int64_t> k, v, G;
	bool flag = false, clipped = false;
	IntIir(const std::vector<typename Tr::S> &kk, const std::vector<typename Tr::S> &vv)
	    : k(kk.begin(), kk.end()), v(vv.begin(), vv.end()), G(kk.size(), 0) {}
	int64_t step(int64_t x) {
		const size_t N = k.size();
		int64_t f = x;
		i128 acc = 0;
		std::vector<int64_t> nG(N);
		for (size_t i = 0; i < N; i++) {
			const size_t mm = N - i;
			const int64_t Gm1 = G[mm - 1];
			const int64_t fn = Tr::iFix(f - Tr::iProd(Gm1, k[i]), flag, clipped);
			const int64_t gm = Tr::iFix(Tr::iProd(fn, k[i]) + Gm1, flag, clipped);
			acc += static_cast<i128>(gm) * v[i];
			if (mm < N) nG[mm] = gm;
			f = fn;
		}
		acc += static_cast<i128>(f) * v[N];
		nG[0] = f;
		G = nG;
		return Tr::iOut(acc, flag, clipped);
	}
};

// ---------------------------------------------------------------------------------------------------------------
// Data
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
std::vector<typename Tr::S> randS(Rng &rng, size_t n, double lo, double hi) {  // real range, rounded to the type
	std::vector<typename Tr::S> v(n);
	for (size_t i = 0; i < n; i++) {
		const double r = rng.uni(lo, hi);
		v[i] = from<Tr>(r);
	}
	return v;
}
template <class Tr>
std::vector<typename Tr::S> randInt(Rng &rng, size_t n, int64_t lo, int64_t hi) {  // integer range (q types)
	std::vector<typename Tr::S> v(n);
	for (size_t i = 0; i < n; i++) {
		const int64_t r = rng.range(lo, hi);
		v[i] = static_cast<typename Tr::S>(r);
	}
	return v;
}
// FIR data (header): |k| <= 1/(2M), |x| <= 0.25 for the q types, float k in +-0.9, x in +-1; neg false = non-negative values only
template <class Tr>
std::vector<typename Tr::S> firK(Rng &rng, size_t M, bool neg) {
	if (Tr::isF) return randS<Tr>(rng, M, neg ? -0.9 : 0.0, 0.9);
	const int64_t lim = (int64_t(1) << Tr::bits) / static_cast<int64_t>(2 * M);
	return randInt<Tr>(rng, M, neg ? -lim : 0, lim);
}
template <class Tr>
std::vector<typename Tr::S> firX(Rng &rng, size_t n, bool neg) {
	if (Tr::isF) return randS<Tr>(rng, n, neg ? -1.0 : 0.0, 1.0);
	const int64_t lim = int64_t(1) << shiftDown(Tr::bits, 2);
	return randInt<Tr>(rng, n, neg ? -lim : 0, lim);
}
// IIR data (header)
template <class Tr>
std::vector<typename Tr::S> iirK(Rng &rng, size_t N) {
	if (Tr::isF) return randS<Tr>(rng, N, -0.1, 0.1);
	const int64_t lim = (int64_t(1) << Tr::bits) / 20;
	return randInt<Tr>(rng, N, -lim, lim);
}
template <class Tr>
std::vector<typename Tr::S> iirV(Rng &rng, size_t N) {  // N + 1 values
	if (Tr::isF) return randS<Tr>(rng, N + 1, -1.0, 1.0);
	(void)N;
	const int64_t lim = int64_t(1) << shiftDown(Tr::bits, 1);
	return randInt<Tr>(rng, N + 1, -lim, lim);
}
template <class Tr>
std::vector<typename Tr::S> iirX(Rng &rng, size_t n) {
	if (Tr::isF) return randS<Tr>(rng, n, -1.0, 1.0);
	const int64_t lim = int64_t(1) << shiftDown(Tr::bits, 4);
	return randInt<Tr>(rng, n, -lim, lim);
}

const size_t kStages[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 17};
typedef std::vector<size_t> Chunks;
const size_t kLong = 40;  // samples of the random tests
// Mode: which data signs and which block sizes a test may use (UB findings of the header: B-DLATT-2 needs signed data with blocks of 4 or more)
struct Mode {
	bool neg;       // signed data
	size_t minB, maxB;  // admitted partitions: the largest listed chunk lies in [minB, maxB]
};
const Mode kAll = {true, 1, 1000}, kNonNeg = {false, 1, 1000}, kSmall = {true, 1, 3}, kLarge = {true, 4, 1000};
bool admits(const Mode &m, const Chunks &c) {
	size_t mx = 0;
	for (size_t b : c) mx = std::max(mx, b);
	return mx >= m.minB && mx <= m.maxB;
}
std::vector<Chunks> filtered(const Mode &m, const std::vector<Chunks> &all) {
	std::vector<Chunks> r;
	for (const Chunks &c : all)
		if (admits(m, c)) r.push_back(c);
	return r;
}
std::vector<Chunks> partitions(const Mode &m) {
	return filtered(m, {{kLong}, {1}, {2}, {3}, {4}, {5, 1, 7}, {9}, {2, 16, 3}});
}

template <class S>
void poison(S *p, size_t n) {
	std::memset(static_cast<void *>(p), 0xCD, n * sizeof(S));
}

// ---------------------------------------------------------------------------------------------------------------
// Runners. The state is a tight heap allocation of exactly the DOC length (poisoned first, no slack, so ASan sees any access
// past its end) unless guardedState: then a GuardedBuffer whose guard words are checked.
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
struct Run {
	std::vector<typename Tr::S> y, state;  // state: first N elements after the last call
	bool guards = true;                    // output guards (and state guard if guardedState)
	bool coefsOk = true;                   // coefficient arrays unchanged
	bool fieldsOk = true;
};

template <class Tr>
Run<Tr> runFir(const std::vector<typename Tr::S> &kIn, const std::vector<typename Tr::S> &x, const Chunks &chunks, bool guardedState = false) {
	typedef typename Tr::S S;
	Run<Tr> r;
	std::vector<S> k = kIn;
	const size_t M = k.size();
	dsp::GuardedBuffer<S> gstate(guardedState ? M : 0);
	std::unique_ptr<S[]> tight(new S[M]);
	poison(tight.get(), M);
	S *st = guardedState ? gstate.data() : tight.get();
	typename Tr::FI inst;
	Tr::fInit(&inst, M, k.data(), st);
	r.fieldsOk = inst.numStages == M && inst.pCoeffs == k.data() && inst.pState == st;
	REQUIRE(r.fieldsOk);
	size_t pos = 0, ci = 0;
	while (pos < x.size()) {
		size_t b = chunks[ci++ % chunks.size()];
		if (b > x.size() - pos) b = x.size() - pos;
		dsp::GuardedBuffer<S> out(b);
		Tr::fRun(&inst, x.data() + pos, out.data(), b);
		if (!out.guardIntact()) r.guards = false;
		const std::vector<S> o = out.output();
		r.y.insert(r.y.end(), o.begin(), o.end());
		pos += b;
	}
	if (guardedState && !gstate.guardIntact()) r.guards = false;
	r.state.assign(st, st + M);
	r.coefsOk = k == kIn;
	return r;
}

template <class Tr>
Run<Tr> runIir(const std::vector<typename Tr::S> &kIn, const std::vector<typename Tr::S> &vIn, const std::vector<typename Tr::S> &x,
               const Chunks &chunks, bool guardedState = false) {
	typedef typename Tr::S S;
	Run<Tr> r;
	std::vector<S> k = kIn, v = vIn;
	const size_t N = k.size();
	size_t maxChunk = 0;
	for (size_t c : chunks) maxChunk = std::max(maxChunk, c);
	const size_t stateLen = N + maxChunk;
	dsp::GuardedBuffer<S> gstate(guardedState ? stateLen : 0);
	std::unique_ptr<S[]> tight(new S[stateLen]);
	poison(tight.get(), stateLen);
	S *st = guardedState ? gstate.data() : tight.get();
	typename Tr::II inst;
	Tr::iInit(&inst, N, k.data(), v.data(), st, maxChunk);
	r.fieldsOk = inst.numStages == N && inst.pkCoeffs == k.data() && inst.pvCoeffs == v.data() && inst.pState == st;
	REQUIRE(r.fieldsOk);
	size_t pos = 0, ci = 0;
	while (pos < x.size()) {
		size_t b = chunks[ci++ % chunks.size()];
		if (b > x.size() - pos) b = x.size() - pos;
		dsp::GuardedBuffer<S> out(b);
		Tr::iRun(&inst, x.data() + pos, out.data(), b);
		if (!out.guardIntact()) r.guards = false;
		const std::vector<S> o = out.output();
		r.y.insert(r.y.end(), o.begin(), o.end());
		pos += b;
	}
	if (guardedState && !gstate.guardIntact()) r.guards = false;
	r.state.assign(st, st + N);
	r.coefsOk = k == kIn && v == vIn;
	return r;
}

// ---------------------------------------------------------------------------------------------------------------
// FIR lattice tests
// ---------------------------------------------------------------------------------------------------------------
// DEF check: output and the g state against the double DEF with the derived bound, plus the premise (model without clipping).
template <class Tr>
void firVsDef(const std::vector<typename Tr::S> &k, const std::vector<typename Tr::S> &x, const Chunks &chunks, const std::string &tag,
              bool guardedState = false) {
	if (!Tr::isF) {  // premise first: the kernel must not see data that overflows (undefined behaviour)
		IntFir<Tr> im(k);
		for (typename Tr::S xv : x) im.step(xv);
		REQUIRE_MESSAGE(!im.flag, tag << ": premise violated (overflow of an f / g sum)");
		REQUIRE_MESSAGE(!im.clipped, tag << ": premise violated (an f / g value saturates)");
	}
	const Run<Tr> r = runFir<Tr>(k, x, chunks, guardedState);
	CHECK_MESSAGE(r.guards, tag << ": a guard word was overwritten");
	CHECK_MESSAGE(r.coefsOk, tag << ": the coefficients were modified");
	REQUIRE(r.y.size() == x.size());
	FirSim sim(realVec<Tr>(k), Tr::fErr());
	size_t bad = 0;
	std::string first;
	for (size_t n = 0; n < x.size(); n++) {
		const Node o = sim.step(real<Tr>(x[n]));
		const double err = std::fabs(real<Tr>(r.y[n]) - o.v);
		if (!(err <= tolOf(o, k.size()))) {
			if (bad == 0) first = "n " + std::to_string(n) + " out " + std::to_string(real<Tr>(r.y[n])) + " DEF " + std::to_string(o.v) + " bound " + std::to_string(tolOf(o, k.size()));
			bad++;
		}
	}
	CHECK_MESSAGE(bad == 0, tag << ": " << bad << " outputs outside the bound, first " << first);
	size_t badState = 0;
	for (size_t i = 0; i < k.size(); i++)
		if (!(std::fabs(real<Tr>(r.state[i]) - sim.G[i].v) <= tolOf(sim.G[i], k.size()))) badState++;
	CHECK_MESSAGE(badState == 0, tag << ": " << badState << " state values (g_m[n] of the DOC) outside the bound");
}

template <class Tr>
void firRandomTest(const Mode &mode) {
	size_t cases = 0;
	for (size_t M : kStages) {
		for (const Chunks &c : partitions(mode)) {
			Rng rng(100 + M * 7 + c[0]);
			const std::vector<typename Tr::S> k = firK<Tr>(rng, M, mode.neg);
			const std::vector<typename Tr::S> x = firX<Tr>(rng, kLong, mode.neg);
			firVsDef<Tr>(k, x, c, "numStages " + std::to_string(M) + " first block " + std::to_string(c[0]));
			cases++;
		}
	}
	CHECK(cases == 11 * partitions(mode).size());
	CHECK(cases > 0);
}

// Hand derived impulse responses (header): x = 0.5 at n = 0.
//   M = 1: y = x + k1 x[n-1]: k1 = 0.5 -> {0.5, 0.25, 0, 0, ...}
//   M = 2: f2 = f1 + k2 g1[n-1], g1[n] = k1 x[n] + x[n-1] -> h = {1, k1 (1 + k2), k2}; k1 = 0.5, k2 = 0.25: h = {1, 0.625, 0.25}, times 0.5 =
//          {0.5, 0.3125, 0.125}
// All values are dyadic: the floor products of the fixed point kernels are exact (q15: 16384 * 8192 = 2^27 = 4096 * 2^15 etc.).
template <class Tr>
void firImpulseTest() {
	typedef typename Tr::S S;
	struct Case {
		std::vector<double> k, y;
	};
	const Case cases[] = {{{0.5}, {0.5, 0.25, 0, 0, 0, 0, 0, 0}}, {{0.5, 0.25}, {0.5, 0.3125, 0.125, 0, 0, 0, 0, 0}}};
	for (const Case &c : cases) {
		std::vector<S> k, expect, x(8, from<Tr>(0));
		for (double d : c.k) k.push_back(from<Tr>(d));
		for (double d : c.y) expect.push_back(from<Tr>(d));
		x[0] = from<Tr>(0.5);
		for (const Chunks &ch : {Chunks{8}, Chunks{1}, Chunks{3}}) {
			const Run<Tr> r = runFir<Tr>(k, x, ch);
			auto chk = dsp::exactEqual(expect, r.y);
			CHECK_MESSAGE(chk, "numStages " << k.size() << " first block " << ch[0] << ": " << chk.message);
			CHECK(r.guards);
		}
	}
}

// all reflection coefficients 0: y[n] = x[n] (f_m = f_{m-1}): exact for every type and every stage count of the unrolled code
template <class Tr>
void firPassThroughTest(const Mode &mode) {
	size_t checked = 0;
	const std::vector<Chunks> chunkSets = filtered(mode, {{24}, {1}, {2}, {3}, {5}, {4, 3}});
	for (size_t M : kStages) {
		Rng rng(300 + M);
		const std::vector<typename Tr::S> k(M, from<Tr>(0));
		std::vector<typename Tr::S> x;
		if (Tr::isF) x = randS<Tr>(rng, 24, mode.neg ? -1.0 : 0.0, 1.0);
		else x = randInt<Tr>(rng, 24, mode.neg ? -(int64_t(1) << (Tr::bits - 1)) : 0, (int64_t(1) << (Tr::bits - 1)) - 1);
		for (const Chunks &c : chunkSets) {
			const Run<Tr> r = runFir<Tr>(k, x, c);
			auto chk = dsp::exactEqual(x, r.y);
			CHECK_MESSAGE(chk, "numStages " << M << " first block " << c[0] << ": " << chk.message);
			CHECK(r.guards);
			checked++;
		}
	}
	CHECK(checked == 11 * chunkSets.size());
	CHECK(checked > 0);
}

// One long block against consecutive blocks. q: the integer arithmetic is the same operation sequence: bitwise equal. float: each
// partition is checked against the DEF bound (the compiler may contract differently in the scalar and unrolled code).
template <class Tr>
void firContinuityTest(const Mode &mode) {
	size_t runs = 0;
	const bool neg = mode.neg;
	const std::vector<Chunks> chunkSets = filtered(mode, {{1}, {2}, {3}, {4}, {5}, {7}, {1, 4, 3, 9, 2}});
	for (size_t M : {size_t(1), size_t(2), size_t(5), size_t(9)}) {
		Rng rng(500 + M);
		const std::vector<typename Tr::S> k = firK<Tr>(rng, M, neg);
		const std::vector<typename Tr::S> x = firX<Tr>(rng, kLong, neg);
		const Run<Tr> whole = runFir<Tr>(k, x, {mode.maxB <= 3 ? size_t(3) : kLong});
		for (const Chunks &c : chunkSets) {
			const std::string tag = "numStages " + std::to_string(M) + " first block " + std::to_string(c[0]);
			if (Tr::isF) {
				firVsDef<Tr>(k, x, c, tag);
			} else {
				const Run<Tr> part = runFir<Tr>(k, x, c);
				auto chk = dsp::exactEqual(whole.y, part.y);
				CHECK_MESSAGE(chk, tag << ": " << chk.message);
				CHECK_MESSAGE(whole.state == part.state, tag << ": final state differs");
			}
			runs++;
		}
	}
	CHECK(runs == 4 * chunkSets.size());
	CHECK(runs > 0);
}

template <class Tr>
void firTwoInstancesTest(bool neg) {
	typedef typename Tr::S S;
	Rng rng(700);
	const size_t M = 5;
	std::vector<S> kA = firK<Tr>(rng, M, neg), kB = firK<Tr>(rng, M, neg);
	const std::vector<S> xA = firX<Tr>(rng, 30, neg), xB = firX<Tr>(rng, 30, neg);
	const Run<Tr> refA = runFir<Tr>(kA, xA, {3});
	const Run<Tr> refB = runFir<Tr>(kB, xB, {3});
	std::unique_ptr<S[]> sA(new S[M]), sB(new S[M]);
	poison(sA.get(), M);
	poison(sB.get(), M);
	typename Tr::FI iA, iB;
	Tr::fInit(&iA, M, kA.data(), sA.get());
	Tr::fInit(&iB, M, kB.data(), sB.get());
	std::vector<S> yA, yB;
	for (size_t p = 0; p < 30; p += 3) {
		S oA[3], oB[3];
		Tr::fRun(&iA, xA.data() + p, oA, 3);
		Tr::fRun(&iB, xB.data() + p, oB, 3);
		yA.insert(yA.end(), oA, oA + 3);
		yB.insert(yB.end(), oB, oB + 3);
	}
	auto a = dsp::exactEqual(refA.y, yA);
	auto b = dsp::exactEqual(refB.y, yB);
	CHECK_MESSAGE(a, "instance A: " << a.message);
	CHECK_MESSAGE(b, "instance B: " << b.message);
	CHECK_MESSAGE(refA.y != refB.y, "the two filters must differ for the test to mean something");
}

// init again on the same buffer: the state is cleared (identical repeat of the run, bit for bit)
template <class Tr>
void firReinitTest(const Mode &mode) {
	typedef typename Tr::S S;
	Rng rng(800);
	const size_t M = 6, B = mode.maxB <= 3 ? 3 : 20;
	std::vector<S> k = firK<Tr>(rng, M, mode.neg);
	const std::vector<S> x = firX<Tr>(rng, B, mode.neg);
	std::unique_ptr<S[]> st(new S[M]);
	poison(st.get(), M);
	typename Tr::FI inst;
	Tr::fInit(&inst, M, k.data(), st.get());
	std::vector<S> first(B), second(B);
	Tr::fRun(&inst, x.data(), first.data(), B);
	bool nonZero = false;
	for (size_t i = 0; i < M; i++) nonZero = nonZero || st[i] != 0;
	CHECK_MESSAGE(nonZero, "the state must hold the history after the run");
	Tr::fInit(&inst, M, k.data(), st.get());
	Tr::fRun(&inst, x.data(), second.data(), B);
	auto chk = dsp::exactEqual(first, second);
	CHECK_MESSAGE(chk, chk.message);
}

// init: fields hold the arguments, exactly numStages elements of the state are zeroed (poison first), nothing else is written
template <class Tr>
void firInitTest() {
	typedef typename Tr::S S;
	size_t checked = 0;
	for (size_t M : kStages) {
		std::vector<S> k(M, from<Tr>(0.0));
		dsp::GuardedBuffer<S> st(M);  // output part poisoned with 0xCD, guard words behind it
		typename Tr::FI inst;
		Tr::fInit(&inst, M, k.data(), st.data());
		CHECK(inst.numStages == M);
		CHECK(inst.pCoeffs == k.data());
		CHECK(inst.pState == st.data());
		const std::vector<S> o = st.output();
		size_t nonZero = 0;
		for (S v : o) nonZero += (v != 0);
		CHECK_MESSAGE(nonZero == 0, "numStages " << M << ": " << nonZero << " of the numStages state elements not zeroed");
		CHECK_MESSAGE(st.guardIntact(), "numStages " << M << ": init wrote behind numStages elements");
		checked++;
	}
	CHECK(checked == 11);
}

// every output written exactly blockSize, nothing behind it, nothing behind the exact size state (guarded state)
template <class Tr>
void firGuardsTest(const Mode &mode) {
	size_t checked = 0;
	size_t blocks = 0;
	for (size_t b : {size_t(1), size_t(2), size_t(3), size_t(4), size_t(5), size_t(9)})
		if (b >= mode.minB && b <= mode.maxB) blocks++;
	for (size_t M : {size_t(1), size_t(4), size_t(5), size_t(9)}) {
		for (size_t b : {size_t(1), size_t(2), size_t(3), size_t(4), size_t(5), size_t(9)}) {
			if (b < mode.minB || b > mode.maxB) continue;
			Rng rng(900 + M * 11 + b);
			const std::vector<typename Tr::S> k = firK<Tr>(rng, M, mode.neg);
			const std::vector<typename Tr::S> x = firX<Tr>(rng, b + 6, mode.neg);
			firVsDef<Tr>(k, x, {b}, "numStages " + std::to_string(M) + " block " + std::to_string(b), true);
			checked++;
		}
	}
	CHECK(checked == 4 * blocks);
	CHECK(checked > 0);
}

// All the f / g values with the DEF checked for M = 1 over the whole range of x (q15 exhaustive over x[0], q31 strided): block of 4
// samples {a, b, c, d}: y[n] = x[n] + k1 * x[n-1] (g0[n-1] = x[n-1]). Results whose true value does not fit the range are skipped (the DOC is silent about
// saturation, the characterization test below covers it); exact = the floor model (characterization), else the bound |error| < 1 LSB (q15) / 2 LSB (q31) of the DEF
template <class Tr>
void firSweepOneStageTest(size_t blk, bool signedData, bool exact) {
	typedef typename Tr::S S;
	const int64_t lsbP = Tr::bits == 15 ? 1 : 2;
	std::vector<int64_t> ks;
	int64_t stride;
	int64_t lo, hi;
	if (Tr::bits == 15) {
		ks = signedData ? std::vector<int64_t>{16384, -16384, 32767, -32768, 1} : std::vector<int64_t>{16384, 32767, 1};
		stride = 1;
		lo = signedData ? -32768 : 0;
		hi = 32767;
	} else {
		ks = {int64_t(1) << 30, 1234567890, INT32_MAX};  // non-negative: no negative left shift (B-DLATT-1)
		stride = 40009;
		lo = 0;
		hi = INT32_MAX;
	}
	const int64_t b = Tr::bits == 15 ? 12345 : 123456789, c = Tr::bits == 15 ? (signedData ? -31000 : 31000) : 1000, d = Tr::bits == 15 ? 777 : 777777;
	size_t tested = 0, bad = 0;
	int64_t firstBad = 0;
	for (int64_t k : ks) {
		const S kk = static_cast<S>(k);
		const std::vector<S> kv(1, kk);
		for (int64_t a = lo; a <= hi; a += stride) {
			const S xs[4] = {static_cast<S>(a), static_cast<S>(b), static_cast<S>(c), static_cast<S>(d)};
			if (Tr::bits == 31) {  // a q31 sum outside the range is signed overflow in the kernel (DOC: input must be scaled down): not run
				bool fits = true;
				const int64_t xq[4] = {a, b, c, d};
				for (int n = 0; n < 4; n++) {
					const int64_t sum = xq[n] + Tr::fProd(n ? xq[n - 1] : 0, k);
					const int64_t gsum = Tr::fProd(xq[n], k) + (n ? xq[n - 1] : 0);  // g1[n] = k f0[n] + g0[n-1]
					fits = fits && sum <= INT32_MAX && sum >= INT32_MIN && gsum <= INT32_MAX && gsum >= INT32_MIN;
				}
				if (!fits) continue;
			}
			S st = 0;
			typename Tr::FI inst;
			Tr::fInit(&inst, 1, const_cast<S *>(kv.data()), &st);
			S y[4] = {0, 0, 0, 0};
			Tr::fRun(&inst, xs, y, blk);
			const int64_t xv[4] = {a, b, c, d};
			for (size_t n = 0; n < blk; n++) {
				const int64_t prev = n ? xv[n - 1] : 0;
				const int64_t P = Tr::fProd(prev, k);
				const int64_t v = xv[n] + P;
				if (v > (int64_t(1) << Tr::bits) - 1 || v < -(int64_t(1) << Tr::bits)) continue;  // saturation: not asserted here
				if (exact) {  // characterization: the floor model of the code
					if (static_cast<int64_t>(y[n]) != v) {
						if (bad == 0) firstBad = a;
						bad++;
					}
				} else {  // DEF: |y - (x[n] + k x[n-1] / 2^bits)| < lsbP LSB (the DOC does not say how the product is rounded)
					const i128 diff = (static_cast<i128>(xv[n]) << Tr::bits) + static_cast<i128>(prev) * k - (static_cast<i128>(y[n]) << Tr::bits);
					const i128 ad = diff < 0 ? -diff : diff;
					if (!(ad < (static_cast<i128>(lsbP) << Tr::bits))) {
						if (bad == 0) firstBad = a;
						bad++;
					}
				}
				tested++;
			}
		}
	}
	CHECK(tested > 100000);
	CHECK_MESSAGE(bad == 0, bad << " outputs differ from x[n] + floor(k x[n-1]), first at x[0] = " << firstBad);
}

// characterization: the integer model of the code, bitwise, including the g state after each call
template <class Tr>
void firExactModelTest(bool full, const Mode &mode) {
	typedef typename Tr::S S;
	size_t runs = 0, clippedRuns = 0;
	for (size_t M : kStages) {
		for (const Chunks &c : partitions(mode)) {
			Rng rng(1100 + M * 5 + c[0]);
			std::vector<S> k, x;
			if (full) {  // full range (q15 only)
				k = randInt<Tr>(rng, M, -(int64_t(1) << Tr::bits), (int64_t(1) << Tr::bits) - 1);
				x = randInt<Tr>(rng, kLong, -(int64_t(1) << Tr::bits), (int64_t(1) << Tr::bits) - 1);
			} else {
				k = firK<Tr>(rng, M, false);
				x = firX<Tr>(rng, kLong, false);
			}
			IntFir<Tr> im(k);
			std::vector<S> expect;
			for (S xv : x) expect.push_back(static_cast<S>(im.step(xv)));
			REQUIRE(!im.flag);
			const Run<Tr> r = runFir<Tr>(k, x, c);
			auto chk = dsp::exactEqual(expect, r.y);
			CHECK_MESSAGE(chk, "numStages " << M << " first block " << c[0] << ": " << chk.message);
			std::vector<S> es;
			for (int64_t g : im.G) es.push_back(static_cast<S>(g));
			CHECK_MESSAGE(es == r.state, "numStages " << M << " first block " << c[0] << ": state differs from g_m[n]");
			CHECK(r.guards);
			runs++;
			if (im.clipped) clippedRuns++;
		}
	}
	CHECK(runs == 11 * partitions(mode).size());
	if (full) CHECK_MESSAGE(clippedRuns * 2 > runs, "the full range data must saturate in most runs: " << clippedRuns << " of " << runs);
	else CHECK(clippedRuns == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// IIR lattice tests
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
void iirVsDef(const std::vector<typename Tr::S> &k, const std::vector<typename Tr::S> &v, const std::vector<typename Tr::S> &x, const Chunks &chunks,
              const std::string &tag, bool guardedState = false) {
	if (!Tr::isF) {  // premise first (see firVsDef)
		IntIir<Tr> im(k, v);
		for (typename Tr::S xv : x) im.step(xv);
		REQUIRE_MESSAGE(!im.flag, tag << ": premise violated (overflow of the accumulator or the output)");
		REQUIRE_MESSAGE(!im.clipped, tag << ": premise violated (an f / g value or the output saturates)");
	}
	const Run<Tr> r = runIir<Tr>(k, v, x, chunks, guardedState);
	CHECK_MESSAGE(r.guards, tag << ": a guard word was overwritten");
	CHECK_MESSAGE(r.coefsOk, tag << ": the coefficients were modified");
	REQUIRE(r.y.size() == x.size());
	IirSim sim(realVec<Tr>(k), realVec<Tr>(v), Tr::iErr());
	size_t bad = 0;
	double worst = 0, ymax = 0;
	std::string first;
	for (size_t n = 0; n < x.size(); n++) {
		const Node o = sim.step(real<Tr>(x[n]));
		const double err = std::fabs(real<Tr>(r.y[n]) - o.v);
		ymax = std::max(ymax, std::fabs(o.v));
		worst = std::max(worst, tolOf(o, k.size()));
		if (!(err <= tolOf(o, k.size()))) {
			if (bad == 0) first = "n " + std::to_string(n) + " out " + std::to_string(real<Tr>(r.y[n])) + " DEF " + std::to_string(o.v) + " bound " + std::to_string(tolOf(o, k.size()));
			bad++;
		}
	}
	CHECK_MESSAGE(bad == 0, tag << ": " << bad << " outputs outside the bound, first " << first);
	size_t badState = 0;
	const size_t N = k.size();
	for (size_t i = 0; i < N; i++)  // state[i] = g_{N-1-i}(last)
		if (!(std::fabs(real<Tr>(r.state[i]) - sim.G[N - 1 - i].v) <= tolOf(sim.G[N - 1 - i], N))) badState++;
	CHECK_MESSAGE(badState == 0, tag << ": " << badState << " state values outside the bound");
	// the bound must be informative (not larger than the signal): a failing bound would hide a wrong kernel
	const bool informative = worst < 0.25 * ymax || ymax == 0;  // measured ratios reach 0.08
	CHECK_MESSAGE(informative, tag << ": bound " << worst << " is not below a quarter of the signal " << ymax);
}

template <class Tr>
void iirRandomTest() {
	size_t cases = 0;
	for (size_t N : kStages) {
		for (const Chunks &c : partitions(kAll)) {
			Rng rng(2100 + N * 7 + c.size());
			const std::vector<typename Tr::S> k = iirK<Tr>(rng, N);
			const std::vector<typename Tr::S> v = iirV<Tr>(rng, N);
			const std::vector<typename Tr::S> x = iirX<Tr>(rng, kLong);
			iirVsDef<Tr>(k, v, x, c, "numStages " + std::to_string(N) + " first block " + std::to_string(c[0]));
			cases++;
		}
	}
	CHECK(cases == 11 * partitions(kAll).size());
}

// Hand derived, N = 1, k1 = 0.5, v1 = 0.5, v0 = 0.25 (pk = {k1}, pv = {v1, v0}), x = 0.5 at n = 0:
//   f0[n] = x[n] - k1 g0[n-1], g0[n] = f0[n], g1[n] = g0[n-1] + k1 f0[n], y[n] = v1 g1[n] + v0 g0[n]:
//   f0 = 0.5 * (-0.5)^n = {0.5, -0.25, 0.125, -0.0625, 0.03125}; g1 = {0.25, 0.375, -0.1875, 0.09375, -0.046875}
//   y = {0.5*0.25 + 0.25*0.5 = 0.25, 0.1875 - 0.0625 = 0.125, -0.09375 + 0.03125 = -0.0625, 0.046875 - 0.015625 = 0.03125,
//        -0.0234375 + 0.0078125 = -0.015625}
// All dyadic: floor products and the accumulator are exact in q15 and q31.
template <class Tr>
void iirImpulseTest() {
	typedef typename Tr::S S;
	const std::vector<S> k = {from<Tr>(0.5)};
	const std::vector<S> v = {from<Tr>(0.5), from<Tr>(0.25)};
	std::vector<S> x(5, from<Tr>(0.0));
	x[0] = from<Tr>(0.5);
	const std::vector<S> expect = {from<Tr>(0.25), from<Tr>(0.125), from<Tr>(-0.0625), from<Tr>(0.03125), from<Tr>(-0.015625)};
	for (const Chunks &c : {Chunks{5}, Chunks{1}, Chunks{2}}) {
		const Run<Tr> r = runIir<Tr>(k, v, x, c);
		auto chk = dsp::exactEqual(expect, r.y);
		CHECK_MESSAGE(chk, "first block " << c[0] << ": " << chk.message);
		CHECK(r.guards);
	}
}

// all reflection coefficients 0: g_m(n) = x[n-m], y[n] = sum_m v_m x[n-m]: the impulse response of amplitude 0.5 is 0.5 * v_m at n = m.
// pv[i] = (i+1)/32 -> v_m = pv[N-m]; y[m] = (N-m+1)/64 (exact in every type), for 1..9 stages (the unrolled tails of the DSP path included)
template <class Tr>
void iirZeroKImpulseTest() {
	typedef typename Tr::S S;
	size_t checked = 0;
	for (size_t N : kStages) {
		const std::vector<S> k(N, from<Tr>(0.0));
		std::vector<S> v(N + 1), expect(N + 4, from<Tr>(0.0)), x(N + 4, from<Tr>(0.0));
		for (size_t i = 0; i <= N; i++) v[i] = from<Tr>(static_cast<double>(i + 1) / 32.0);
		x[0] = from<Tr>(0.5);
		for (size_t m = 0; m <= N; m++) expect[m] = from<Tr>(static_cast<double>(N - m + 1) / 64.0);
		for (const Chunks &c : {Chunks{N + 4}, Chunks{1}, Chunks{3}}) {
			const Run<Tr> r = runIir<Tr>(k, v, x, c);
			auto chk = dsp::exactEqual(expect, r.y);
			CHECK_MESSAGE(chk, "numStages " << N << " first block " << c[0] << ": " << chk.message);
			CHECK(r.guards);
			checked++;
		}
	}
	CHECK(checked == 33);
}

template <class Tr>
void iirContinuityTest() {
	size_t runs = 0;
	for (size_t N : {size_t(1), size_t(2), size_t(5), size_t(9)}) {
		Rng rng(2500 + N);
		const std::vector<typename Tr::S> k = iirK<Tr>(rng, N);
		const std::vector<typename Tr::S> v = iirV<Tr>(rng, N);
		const std::vector<typename Tr::S> x = iirX<Tr>(rng, kLong);
		const Run<Tr> whole = runIir<Tr>(k, v, x, {kLong});
		for (const Chunks &c : {Chunks{1}, Chunks{2}, Chunks{3}, Chunks{4}, Chunks{5}, Chunks{7}, Chunks{1, 4, 3, 9, 2}}) {
			const std::string tag = "numStages " + std::to_string(N) + " first block " + std::to_string(c[0]);
			if (Tr::isF) {
				iirVsDef<Tr>(k, v, x, c, tag);
			} else {
				const Run<Tr> part = runIir<Tr>(k, v, x, c);
				auto chk = dsp::exactEqual(whole.y, part.y);
				CHECK_MESSAGE(chk, tag << ": " << chk.message);
				CHECK_MESSAGE(whole.state == part.state, tag << ": final state differs");
			}
			runs++;
		}
	}
	CHECK(runs == 28);
}

template <class Tr>
void iirTwoInstancesTest() {
	typedef typename Tr::S S;
	Rng rng(2700);
	const size_t N = 5;
	std::vector<S> kA = iirK<Tr>(rng, N), vA = iirV<Tr>(rng, N), kB = iirK<Tr>(rng, N), vB = iirV<Tr>(rng, N);
	const std::vector<S> xA = iirX<Tr>(rng, 30), xB = iirX<Tr>(rng, 30);
	const Run<Tr> refA = runIir<Tr>(kA, vA, xA, {3});
	const Run<Tr> refB = runIir<Tr>(kB, vB, xB, {3});
	std::unique_ptr<S[]> sA(new S[N + 3]), sB(new S[N + 3]);
	poison(sA.get(), N + 3);
	poison(sB.get(), N + 3);
	typename Tr::II iA, iB;
	Tr::iInit(&iA, N, kA.data(), vA.data(), sA.get(), 3);
	Tr::iInit(&iB, N, kB.data(), vB.data(), sB.get(), 3);
	std::vector<S> yA, yB;
	for (size_t p = 0; p < 30; p += 3) {
		S oA[3], oB[3];
		Tr::iRun(&iA, xA.data() + p, oA, 3);
		Tr::iRun(&iB, xB.data() + p, oB, 3);
		yA.insert(yA.end(), oA, oA + 3);
		yB.insert(yB.end(), oB, oB + 3);
	}
	auto a = dsp::exactEqual(refA.y, yA);
	auto b = dsp::exactEqual(refB.y, yB);
	CHECK_MESSAGE(a, "instance A: " << a.message);
	CHECK_MESSAGE(b, "instance B: " << b.message);
	CHECK_MESSAGE(refA.y != refB.y, "the two filters must differ for the test to mean something");
}

template <class Tr>
void iirReinitTest() {
	typedef typename Tr::S S;
	Rng rng(2800);
	const size_t N = 6, B = 20;
	std::vector<S> k = iirK<Tr>(rng, N), v = iirV<Tr>(rng, N);
	const std::vector<S> x = iirX<Tr>(rng, B);
	std::unique_ptr<S[]> st(new S[N + B]);
	poison(st.get(), N + B);
	typename Tr::II inst;
	Tr::iInit(&inst, N, k.data(), v.data(), st.get(), B);
	std::vector<S> first(B), second(B);
	Tr::iRun(&inst, x.data(), first.data(), B);
	bool nonZero = false;
	for (size_t i = 0; i < N; i++) nonZero = nonZero || st[i] != 0;
	CHECK_MESSAGE(nonZero, "the state must hold the history after the run");
	Tr::iInit(&inst, N, k.data(), v.data(), st.get(), B);
	Tr::iRun(&inst, x.data(), second.data(), B);
	auto chk = dsp::exactEqual(first, second);
	CHECK_MESSAGE(chk, chk.message);
}

// init: the fields hold the arguments, exactly numStages + blockSize state elements are zeroed (poison first), nothing behind them is written
template <class Tr>
void iirInitTest() {
	typedef typename Tr::S S;
	size_t checked = 0;
	for (size_t N : kStages) {
		for (size_t B : {size_t(1), size_t(7)}) {
			std::vector<S> k(N, from<Tr>(0.0)), v(N + 1, from<Tr>(0.0));
			dsp::GuardedBuffer<S> st(N + B);
			typename Tr::II inst;
			Tr::iInit(&inst, N, k.data(), v.data(), st.data(), B);
			CHECK(inst.numStages == N);
			CHECK(inst.pkCoeffs == k.data());
			CHECK(inst.pvCoeffs == v.data());
			CHECK(inst.pState == st.data());
			const std::vector<S> o = st.output();
			size_t nonZero = 0;
			for (S e : o) nonZero += (e != 0);
			CHECK_MESSAGE(nonZero == 0, "numStages " << N << " blockSize " << B << ": " << nonZero << " state elements not zeroed");
			CHECK_MESSAGE(st.guardIntact(), "numStages " << N << " blockSize " << B << ": init wrote behind numStages + blockSize elements");
			checked++;
		}
	}
	CHECK(checked == 22);
}

template <class Tr>
void iirGuardsTest() {
	size_t checked = 0;
	for (size_t N : {size_t(1), size_t(4), size_t(5), size_t(9)}) {
		for (size_t b : {size_t(1), size_t(2), size_t(3), size_t(4), size_t(5), size_t(9)}) {
			Rng rng(2900 + N * 11 + b);
			const std::vector<typename Tr::S> k = iirK<Tr>(rng, N);
			const std::vector<typename Tr::S> v = iirV<Tr>(rng, N);
			const std::vector<typename Tr::S> x = iirX<Tr>(rng, b + 6);
			iirVsDef<Tr>(k, v, x, {b}, "numStages " + std::to_string(N) + " block " + std::to_string(b), true);
			checked++;
		}
	}
	CHECK(checked == 24);
}

// N = 1, k = 0: y[n] = v1 x[n-1] + v0 x[n]. q15 exhaustive over x[0] (x[1] fixed), q31 strided; the DOC says the accumulator is saturated to 1.15
// (q15): expected sat16(floor(acc / 2^15)) in every case incl. saturated ones. q31: |acc| <= 2^30 2^31 + 2^29 2^31 < 2^62 and |y| <= 1.5 * 2^30: in range.
// The floor is asserted one-sided independently: 0 <= acc - y 2^bits < 2^bits where y is not saturated.
template <class Tr>
void iirSweepTest() {
	typedef typename Tr::S S;
	const int64_t v0 = Tr::bits == 15 ? 30000 : (int64_t(1) << 30);
	const int64_t v1 = Tr::bits == 15 ? -30000 : -(int64_t(1) << 29);
	const std::vector<S> k(1, static_cast<S>(0));
	const std::vector<S> v = {static_cast<S>(v1), static_cast<S>(v0)};
	const int64_t stride = Tr::bits == 15 ? 1 : 40009;
	const int64_t lo = -(int64_t(1) << Tr::bits), hi = (int64_t(1) << Tr::bits) - 1;
	size_t tested = 0, bad = 0, saturated = 0;
	int64_t firstBad = 0;
	for (int64_t a = lo; a <= hi; a += stride) {
		for (int64_t b : {int64_t(12345), int64_t(-31000)}) {
			const S xs[2] = {static_cast<S>(a), static_cast<S>(b * (Tr::bits == 15 ? 1 : 65536))};
			const int64_t xb = xs[1];
			S st2[3] = {0, 0, 0};
			typename Tr::II inst;
			Tr::iInit(&inst, 1, const_cast<S *>(k.data()), const_cast<S *>(v.data()), st2, 2);
			S y[2];
			Tr::iRun(&inst, xs, y, 2);
			const i128 acc0 = static_cast<i128>(v0) * a;
			const i128 acc1 = static_cast<i128>(v1) * a + static_cast<i128>(v0) * xb;
			const i128 accs[2] = {acc0, acc1};
			for (int n = 0; n < 2; n++) {
				const i128 fl = accs[n] >> Tr::bits;
				const i128 smax = (static_cast<i128>(1) << Tr::bits) - 1, smin = -(static_cast<i128>(1) << Tr::bits);
				const i128 want = fl > smax ? smax : (fl < smin ? smin : fl);
				if (fl != want) saturated++;
				bool ok = static_cast<i128>(y[n]) == want;
				if (fl == want) {  // floor error one-sided and strict
					const i128 diff = accs[n] - (static_cast<i128>(y[n]) << Tr::bits);
					ok = ok && diff >= 0 && diff < (static_cast<i128>(1) << Tr::bits);
				}
				if (!ok) {
					if (bad == 0) firstBad = a;
					bad++;
				}
				tested++;
			}
		}
	}
	CHECK(tested > 100000);
	if (Tr::bits == 15) CHECK_MESSAGE(saturated > 0, "the sweep must reach the saturation of the output");
	CHECK_MESSAGE(bad == 0, bad << " outputs differ from the DEF, first at x[0] = " << firstBad);
}

// q15 IIR output saturation (DOC "the accumulator is saturated to yield a result in 1.15 format"): k = 0, v1 = v0 = 29491 (0.9), x = +-29491:
// the intermediate f / g stay in range (k = 0: g1 = x[n-1]). Hand: p = 29491^2 = 869719081; p / 2^15 = 26541.7: y[0] = floor(p / 2^15) = 26541 (only v0 x[0],
// g1 = 0), y[1] = y[2] = floor(2 p / 2^15) = 53083 > 32767 -> 32767. Negative data: y[0] = floor(-p / 2^15) = -26542, then -32768.
void iirQ15OutputSaturationTest() {
	const std::vector<int16_t> k = {0};
	const std::vector<int16_t> v = {29491, 29491};
	const std::vector<int16_t> xp = {29491, 29491, 29491};
	const std::vector<int16_t> xn = {-29491, -29491, -29491};
	const Run<Q15T> rp = runIir<Q15T>(k, v, xp, {3});
	const Run<Q15T> rn = runIir<Q15T>(k, v, xn, {3});
	const int64_t p = static_cast<int64_t>(29491) * 29491;
	const int64_t y0 = p >> 15, y1 = (2 * p) >> 15;
	REQUIRE(p == 869719081);
	REQUIRE(y0 == 26541);
	REQUIRE(y1 == 53083);
	CHECK(rp.y[0] == y0);
	CHECK(rp.y[1] == 32767);
	CHECK(rp.y[2] == 32767);
	CHECK(rn.y[0] == -y0 - 1);  // floor of -p / 2^15 (p not a multiple of 2^15)
	CHECK(rn.y[1] == -32768);
	CHECK(rn.y[2] == -32768);
}

// characterization: the integer model of the code, bitwise, incl. the final state[i] = g_{N-1-i}; full: x and k over the full range (the
// intermediate f / g clip, output clips for q15), v small so that the output / accumulator stay in range for q31
template <class Tr>
void iirExactModelTest(bool full) {
	typedef typename Tr::S S;
	size_t runs = 0, clippedRuns = 0;
	for (size_t N : kStages) {
		for (const Chunks &c : partitions(kAll)) {
			Rng rng(3100 + N * 5 + c.size());
			std::vector<S> k, v, x;
			if (full) {
				const int64_t kmax = (int64_t(1) << Tr::bits) - 1;
				k = randInt<Tr>(rng, N, -kmax, kmax);  // never the most negative value (the product (-1) * (-1) wraps in q31)
				x = randInt<Tr>(rng, kLong, -kmax - 1, kmax);
				const int64_t vl = (int64_t(1) << Tr::bits) / static_cast<int64_t>(2 * (N + 1));  // |y| <= sum |v| |g| < 2^bits / 2
				v = randInt<Tr>(rng, N + 1, -vl, vl);
			} else {
				k = iirK<Tr>(rng, N);
				v = iirV<Tr>(rng, N);
				x = iirX<Tr>(rng, kLong);
			}
			IntIir<Tr> im(k, v);
			std::vector<S> expect;
			for (S xv : x) expect.push_back(static_cast<S>(im.step(xv)));
			REQUIRE(!im.flag);
			const Run<Tr> r = runIir<Tr>(k, v, x, c);
			auto chk = dsp::exactEqual(expect, r.y);
			CHECK_MESSAGE(chk, "numStages " << N << " first block " << c[0] << ": " << chk.message);
			std::vector<S> es;
			for (size_t i = 0; i < N; i++) es.push_back(static_cast<S>(im.G[N - 1 - i]));
			CHECK_MESSAGE(es == r.state, "numStages " << N << " first block " << c[0] << ": state differs");
			CHECK(r.guards);
			runs++;
			if (im.clipped) clippedRuns++;
		}
	}
	CHECK(runs == 11 * partitions(kAll).size());
	if (full) CHECK_MESSAGE(clippedRuns > 30, "the full range data must clip in most runs: " << clippedRuns);
	else CHECK(clippedRuns == 0);
}

// Hand derived, N = 2, pk = {k2, k1} = {0.25, 0.5}, pv = {v2, v1, v0} = {0.25, 0.5, 0.25}, x = 0.5 at n = 0, G0 = g0(n-1), G1 = g1(n-1) (DOC equations):
//   f1 = x - k2 G1; g2 = G1 + k2 f1; f0 = f1 - k1 G0; g1 = G0 + k1 f0; g0 = f0; y = v2 g2 + v1 g1 + v0 g0
//   n = 0: G = (0, 0): f1 = 0.5, g2 = 0.125, f0 = 0.5, g1 = 0.25, g0 = 0.5, y = 0.03125 + 0.125 + 0.125 = 0.28125
//   n = 1: G = (0.5, 0.25): f1 = -0.0625, g2 = 0.25 - 0.015625 = 0.234375, f0 = -0.0625 - 0.25 = -0.3125, g1 = 0.5 - 0.15625 = 0.34375,
//          y = 0.05859375 + 0.171875 - 0.078125 = 0.15234375
//   n = 2: G = (-0.3125, 0.34375): f1 = -0.0859375, g2 = 0.34375 - 0.021484375 = 0.322265625, f0 = -0.0859375 + 0.15625 = 0.0703125,
//          g1 = -0.3125 + 0.03515625 = -0.27734375, y = 0.08056640625 - 0.138671875 + 0.017578125 = -0.04052734375
// All products are dyadic with few bits (every f / g is a multiple of 2^-12): the floor products are exact in q15 and q31.
template <class Tr>
void iirTwoStageImpulseTest() {
	typedef typename Tr::S S;
	const std::vector<S> k = {from<Tr>(0.25), from<Tr>(0.5)};
	const std::vector<S> v = {from<Tr>(0.25), from<Tr>(0.5), from<Tr>(0.25)};
	std::vector<S> x(3, from<Tr>(0.0));
	x[0] = from<Tr>(0.5);
	const std::vector<S> expect = {from<Tr>(0.28125), from<Tr>(0.15234375), from<Tr>(-0.04052734375)};
	for (const Chunks &c : {Chunks{3}, Chunks{1}, Chunks{2}}) {
		const Run<Tr> r = runIir<Tr>(k, v, x, c);
		auto chk = dsp::exactEqual(expect, r.y);
		CHECK_MESSAGE(chk, "first block " << c[0] << ": " << chk.message);
		CHECK(r.guards);
	}
}

// the DOC allows coefficient arrays shared among instances: two instances on the same arrays and different inputs give the single instance results
template <class Tr>
void firSharedCoefsTest(bool neg) {
	typedef typename Tr::S S;
	Rng rng(1300);
	const size_t M = 5;
	std::vector<S> k = firK<Tr>(rng, M, neg);
	const std::vector<S> xA = firX<Tr>(rng, 30, neg), xB = firX<Tr>(rng, 30, neg);
	const Run<Tr> refA = runFir<Tr>(k, xA, {3});
	const Run<Tr> refB = runFir<Tr>(k, xB, {3});
	std::unique_ptr<S[]> sA(new S[M]), sB(new S[M]);
	poison(sA.get(), M);
	poison(sB.get(), M);
	typename Tr::FI iA, iB;
	Tr::fInit(&iA, M, k.data(), sA.get());
	Tr::fInit(&iB, M, k.data(), sB.get());
	std::vector<S> yA, yB;
	for (size_t p = 0; p < 30; p += 3) {
		S oA[3], oB[3];
		Tr::fRun(&iA, xA.data() + p, oA, 3);
		Tr::fRun(&iB, xB.data() + p, oB, 3);
		yA.insert(yA.end(), oA, oA + 3);
		yB.insert(yB.end(), oB, oB + 3);
	}
	CHECK(iA.pCoeffs == iB.pCoeffs);
	CHECK(yA == refA.y);
	CHECK(yB == refB.y);
	CHECK(yA != yB);
}
template <class Tr>
void iirSharedCoefsTest() {
	typedef typename Tr::S S;
	Rng rng(3300);
	const size_t N = 5;
	std::vector<S> k = iirK<Tr>(rng, N), v = iirV<Tr>(rng, N);
	const std::vector<S> xA = iirX<Tr>(rng, 30), xB = iirX<Tr>(rng, 30);
	const Run<Tr> refA = runIir<Tr>(k, v, xA, {3});
	const Run<Tr> refB = runIir<Tr>(k, v, xB, {3});
	std::unique_ptr<S[]> sA(new S[N + 3]), sB(new S[N + 3]);
	poison(sA.get(), N + 3);
	poison(sB.get(), N + 3);
	typename Tr::II iA, iB;
	Tr::iInit(&iA, N, k.data(), v.data(), sA.get(), 3);
	Tr::iInit(&iB, N, k.data(), v.data(), sB.get(), 3);
	std::vector<S> yA, yB;
	for (size_t p = 0; p < 30; p += 3) {
		S oA[3], oB[3];
		Tr::iRun(&iA, xA.data() + p, oA, 3);
		Tr::iRun(&iB, xB.data() + p, oB, 3);
		yA.insert(yA.end(), oA, oA + 3);
		yB.insert(yB.end(), oB, oB + 3);
	}
	CHECK(iA.pkCoeffs == iB.pkCoeffs);
	CHECK(iA.pvCoeffs == iB.pvCoeffs);
	CHECK(yA == refA.y);
	CHECK(yB == refB.y);
	CHECK(yA != yB);
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------
// FIR lattice
// ---------------------------------------------------------------------------------------------------------------
#define UB_FQ31 KNOWN_ISSUE_UB_HIDDEN("B-DLATT-1: arm_fir_lattice_q31.c shifts a negative q31 product left ((fnext0 << 1U) + fcurr0)")
#define UB_FQ15 KNOWN_ISSUE_UB_HIDDEN("B-DLATT-2: arm_fir_lattice_q15.c blockSize >= 4 stores the outputs with __PKHBT (dsp/none.h) which shifts a negative q15 left")

TEST_CASE("[C3 arm_fir_lattice_f32] the impulse response of a 1 and a 2 stage filter equals the hand derived h = {1, k1 (1 + k2), k2}") { firImpulseTest<F32T>(); }
TEST_CASE("[C3 arm_fir_lattice_q15] the impulse response of a 1 and a 2 stage filter equals the hand derived h = {1, k1 (1 + k2), k2}") { firImpulseTest<Q15T>(); }
TEST_CASE("[C3 arm_fir_lattice_q31] the impulse response of a 1 and a 2 stage filter equals the hand derived h = {1, k1 (1 + k2), k2}") { firImpulseTest<Q31T>(); }

TEST_CASE("[C3 arm_fir_lattice_f32] with all reflection coefficients 0 the output equals the input for 1..9, 12, 17 stages and blocks of 24, 1, 2, 3, 5, 4+3") { firPassThroughTest<F32T>(kAll); }
TEST_CASE("[C3 arm_fir_lattice_q15] with all reflection coefficients 0 the output equals the non-negative input for 1..9, 12, 17 stages and blocks of 24, 1, 2, 3, 5, 4+3") { firPassThroughTest<Q15T>(kNonNeg); }
TEST_CASE("[C3 arm_fir_lattice_q15] with all reflection coefficients 0 the output equals the signed input for 1..9, 12, 17 stages and blocks of 1, 2, 3") { firPassThroughTest<Q15T>(kSmall); }
TEST_CASE("[C3 arm_fir_lattice_q15] with all reflection coefficients 0 the output equals the signed input for 1..9, 12, 17 stages and blocks of 24, 5, 4+3" * UB_FQ15) { firPassThroughTest<Q15T>(kLarge); }
TEST_CASE("[C3 arm_fir_lattice_q31] with all reflection coefficients 0 the output equals the signed input for 1..9, 12, 17 stages and blocks of 24, 1, 2, 3, 5, 4+3") { firPassThroughTest<Q31T>(kAll); }

TEST_CASE("[C3 arm_fir_lattice_f32] 1..9, 12 and 17 stages with random data are within the derived n*eps bound of the double DEF, output and g state, 8 block partitions of 40 samples") { firRandomTest<F32T>(kAll); }
TEST_CASE("[C3 C5 arm_fir_lattice_q15] 1..9, 12 and 17 stages with non-negative data are within the derived floor bound of the double DEF, output and g state, 8 block partitions of 40 samples") { firRandomTest<Q15T>(kNonNeg); }
TEST_CASE("[C3 C5 arm_fir_lattice_q15] 1..9, 12 and 17 stages with signed data are within the derived floor bound of the double DEF, output and g state, blocks of 1, 2, 3") { firRandomTest<Q15T>(kSmall); }
TEST_CASE("[C3 C5 arm_fir_lattice_q15] 1..9, 12 and 17 stages with signed data are within the derived floor bound of the double DEF, output and g state, blocks of 4 and more (40, 4, 5+1+7, 9, 2+16+3)" * UB_FQ15) { firRandomTest<Q15T>(kLarge); }
TEST_CASE("[C3 C5 arm_fir_lattice_q31] 1..9, 12 and 17 stages with non-negative data are within the derived floor bound of the double DEF, output and g state, 8 block partitions of 40 samples") { firRandomTest<Q31T>(kNonNeg); }
TEST_CASE("[C3 C5 arm_fir_lattice_q31] 1..9, 12 and 17 stages with signed data are within the derived floor bound of the double DEF, 8 block partitions of 40 samples" * UB_FQ31) { firRandomTest<Q31T>(kAll); }

TEST_CASE("[C4 arm_fir_lattice_f32] blocks of 1, 2, 3, 4, 5, 7 and 1+4+3+9+2 samples carry the state like one long block (each within the DEF bound), 1, 2, 5, 9 stages") { firContinuityTest<F32T>(kAll); }
TEST_CASE("[C4 arm_fir_lattice_q15] blocks of 1, 2, 3, 4, 5, 7 and 1+4+3+9+2 samples give bit for bit the output and state of one block of 40, non-negative data, 1, 2, 5, 9 stages") { firContinuityTest<Q15T>(kNonNeg); }
TEST_CASE("[C4 arm_fir_lattice_q15] blocks of 1, 2 and 3 samples give bit for bit the output and state of blocks of 3, signed data, 1, 2, 5, 9 stages") { firContinuityTest<Q15T>(kSmall); }
TEST_CASE("[C4 arm_fir_lattice_q15] blocks of 4, 5, 7 and 1+4+3+9+2 samples give bit for bit the output and state of one block of 40, signed data, 1, 2, 5, 9 stages" * UB_FQ15) { firContinuityTest<Q15T>(kLarge); }
TEST_CASE("[C4 arm_fir_lattice_q31] blocks of 1, 2, 3, 4, 5, 7 and 1+4+3+9+2 samples give bit for bit the output and state of one block of 40, non-negative data, 1, 2, 5, 9 stages") { firContinuityTest<Q31T>(kNonNeg); }
TEST_CASE("[C4 arm_fir_lattice_q31] blocks of 1, 2, 3, 4, 5, 7 and 1+4+3+9+2 samples give bit for bit the output and state of one block of 40, signed data" * UB_FQ31) { firContinuityTest<Q31T>(kAll); }

TEST_CASE("[C4 arm_fir_lattice_f32] two instances with different coefficients, called alternately with blocks of 3, do not influence each other") { firTwoInstancesTest<F32T>(true); }
TEST_CASE("[C4 arm_fir_lattice_q15] two instances with different coefficients, called alternately with blocks of 3, do not influence each other (signed data)") { firTwoInstancesTest<Q15T>(true); }
TEST_CASE("[C4 arm_fir_lattice_q31] two instances with different coefficients, called alternately with blocks of 3, do not influence each other (non-negative data)") { firTwoInstancesTest<Q31T>(false); }

TEST_CASE("[C4 arm_fir_lattice_f32] init again on the same state buffer clears the history: the run of 20 samples repeats bit for bit") { firReinitTest<F32T>(kAll); }
TEST_CASE("[C4 arm_fir_lattice_q15] init again on the same state buffer clears the history: the run of 20 non-negative samples repeats bit for bit") { firReinitTest<Q15T>(kNonNeg); }
TEST_CASE("[C4 arm_fir_lattice_q15] init again on the same state buffer clears the history: the run of 3 signed samples repeats bit for bit") { firReinitTest<Q15T>(kSmall); }
TEST_CASE("[C4 arm_fir_lattice_q31] init again on the same state buffer clears the history: the run of 20 non-negative samples repeats bit for bit") { firReinitTest<Q31T>(kNonNeg); }

TEST_CASE("[C4 arm_fir_lattice_init_f32] init stores numStages, pCoeffs, pState and zeroes exactly numStages state elements for 1..9, 12, 17 stages") { firInitTest<F32T>(); }
TEST_CASE("[C4 arm_fir_lattice_init_q15] init stores numStages, pCoeffs, pState and zeroes exactly numStages state elements for 1..9, 12, 17 stages") { firInitTest<Q15T>(); }
TEST_CASE("[C4 arm_fir_lattice_init_q31] init stores numStages, pCoeffs, pState and zeroes exactly numStages state elements for 1..9, 12, 17 stages") { firInitTest<Q31T>(); }

TEST_CASE("[C2 arm_fir_lattice_f32] blocks of 1, 2, 3, 4, 5, 9 samples with 1, 4, 5, 9 stages write exactly blockSize outputs and nothing behind the exact size (numStages) state") { firGuardsTest<F32T>(kAll); }
TEST_CASE("[C2 arm_fir_lattice_q15] blocks of 1, 2, 3, 4, 5, 9 samples with 1, 4, 5, 9 stages and non-negative data write exactly blockSize outputs and nothing behind the exact size (numStages) state") { firGuardsTest<Q15T>(kNonNeg); }
TEST_CASE("[C2 arm_fir_lattice_q15] blocks of 1, 2, 3 samples with 1, 4, 5, 9 stages and signed data write exactly blockSize outputs and nothing behind the exact size (numStages) state") { firGuardsTest<Q15T>(kSmall); }
TEST_CASE("[C2 arm_fir_lattice_q15] blocks of 4, 5, 9 samples with 1, 4, 5, 9 stages and signed data write exactly blockSize outputs and nothing behind the exact size (numStages) state" * UB_FQ15) { firGuardsTest<Q15T>(kLarge); }
TEST_CASE("[C2 arm_fir_lattice_q31] blocks of 1, 2, 3, 4, 5, 9 samples with 1, 4, 5, 9 stages and non-negative data write exactly blockSize outputs and nothing behind the exact size (numStages) state") { firGuardsTest<Q31T>(kNonNeg); }

TEST_CASE("characterization: [C3 arm_fir_lattice_q15] one stage, blocks of 3: y[n] = x[n] + floor(k x[n-1] / 2^15) for every x[0] in 1.15 (non-saturating results), 5 coefficients, exact floor model") { firSweepOneStageTest<Q15T>(3, true, true); }
TEST_CASE("[C3 arm_fir_lattice_q15] one stage, blocks of 3: y[n] = x[n] + floor(k x[n-1] / 2^15) for every x[0] in 1.15 (non-saturating results), 5 coefficients, within 1 LSB (q31: 2 LSB) of the real product") { firSweepOneStageTest<Q15T>(3, true, false); }
TEST_CASE("characterization: [C3 arm_fir_lattice_q15] one stage, blocks of 4: y[n] = x[n] + floor(k x[n-1] / 2^15) for every non-negative x[0], non-negative data, 3 coefficients, exact floor model") { firSweepOneStageTest<Q15T>(4, false, true); }
TEST_CASE("[C3 arm_fir_lattice_q15] one stage, blocks of 4: y[n] = x[n] + floor(k x[n-1] / 2^15) for every non-negative x[0], non-negative data, 3 coefficients, within 1 LSB (q31: 2 LSB) of the real product") { firSweepOneStageTest<Q15T>(4, false, false); }
TEST_CASE("characterization: [C3 arm_fir_lattice_q15] one stage, blocks of 4: y[n] = x[n] + floor(k x[n-1] / 2^15) for every x[0] in 1.15 (non-saturating results), 5 coefficients, exact floor model" * UB_FQ15) { firSweepOneStageTest<Q15T>(4, true, true); }
TEST_CASE("[C3 arm_fir_lattice_q15] one stage, blocks of 4: y[n] = x[n] + floor(k x[n-1] / 2^15) for every x[0] in 1.15 (non-saturating results), 5 coefficients, within 1 LSB (q31: 2 LSB) of the real product" * UB_FQ15) { firSweepOneStageTest<Q15T>(4, true, false); }
TEST_CASE("characterization: [C3 arm_fir_lattice_q31] one stage, block of 4: y[n] = x[n] + 2 floor(k x[n-1] / 2^32) for x[0] over the non-negative range in steps of 40009, 3 coefficients, exact floor model") { firSweepOneStageTest<Q31T>(4, false, true); }
TEST_CASE("[C3 arm_fir_lattice_q31] one stage, block of 4: y[n] = x[n] + 2 floor(k x[n-1] / 2^32) for x[0] over the non-negative range in steps of 40009, 3 coefficients, within 1 LSB (q31: 2 LSB) of the real product") { firSweepOneStageTest<Q31T>(4, false, false); }

TEST_CASE("characterization: [C3 arm_fir_lattice_q15] the output and the g state equal the integer model with floor products (DOC silent), non-negative data, 1..9, 12 and 17 stages, 8 partitions") { firExactModelTest<Q15T>(false, kNonNeg); }
TEST_CASE("characterization: [C4 arm_fir_lattice_q15] full range data, blocks of 1, 2, 3: every f and g is clipped to 1.15 as the model with __SSAT says (DOC silent about saturation)") { firExactModelTest<Q15T>(true, kSmall); }
TEST_CASE("characterization: [C4 arm_fir_lattice_q15] full range data, blocks of 4 and more: every f and g is clipped to 1.15 as the model with __SSAT says (DOC silent about saturation)" * UB_FQ15) { firExactModelTest<Q15T>(true, kLarge); }
TEST_CASE("characterization: [C3 arm_fir_lattice_q31] the output and the g state equal the integer model with 2 * floor(a k / 2^32) products (DOC silent), non-negative data, 1..9, 12 and 17 stages, 8 partitions") { firExactModelTest<Q31T>(false, kNonNeg); }

// ---------------------------------------------------------------------------------------------------------------
// IIR lattice
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C3 arm_iir_lattice_f32] the impulse response of a 1 stage filter equals the hand derived {0.25, 0.125, -0.0625, 0.03125, -0.015625}") { iirImpulseTest<F32T>(); }
TEST_CASE("[C3 arm_iir_lattice_q15] the impulse response of a 1 stage filter equals the hand derived {0.25, 0.125, -0.0625, 0.03125, -0.015625}") { iirImpulseTest<Q15T>(); }
TEST_CASE("[C3 arm_iir_lattice_q31] the impulse response of a 1 stage filter equals the hand derived {0.25, 0.125, -0.0625, 0.03125, -0.015625}") { iirImpulseTest<Q31T>(); }

TEST_CASE("[C3 arm_iir_lattice_f32] with all reflection coefficients 0 the impulse response is the ladder coefficients in reverse order, for 1..9, 12 and 17 stages") { iirZeroKImpulseTest<F32T>(); }
TEST_CASE("[C3 arm_iir_lattice_q15] with all reflection coefficients 0 the impulse response is the ladder coefficients in reverse order, for 1..9, 12 and 17 stages") { iirZeroKImpulseTest<Q15T>(); }
TEST_CASE("[C3 arm_iir_lattice_q31] with all reflection coefficients 0 the impulse response is the ladder coefficients in reverse order, for 1..9, 12 and 17 stages") { iirZeroKImpulseTest<Q31T>(); }

TEST_CASE("[C3 arm_iir_lattice_f32] 1..9, 12 and 17 stages with random data are within the derived error bound (feedback included) of the double DEF, output and state, 8 block partitions of 40 samples") { iirRandomTest<F32T>(); }
TEST_CASE("[C3 C5 arm_iir_lattice_q15] 1..9, 12 and 17 stages with random data are within the derived floor bound of the double DEF, output and state, 8 block partitions of 40 samples") { iirRandomTest<Q15T>(); }
TEST_CASE("[C3 C5 arm_iir_lattice_q31] 1..9, 12 and 17 stages with random data are within the derived floor bound of the double DEF, output and state, 8 block partitions of 40 samples") { iirRandomTest<Q31T>(); }

TEST_CASE("[C4 arm_iir_lattice_f32] blocks of 1, 2, 3, 4, 5, 7 and 1+4+3+9+2 samples carry the state like one long block (each within the DEF bound), 1, 2, 5, 9 stages") { iirContinuityTest<F32T>(); }
TEST_CASE("[C4 arm_iir_lattice_q15] blocks of 1, 2, 3, 4, 5, 7 and 1+4+3+9+2 samples give bit for bit the output and state of one long block, 1, 2, 5, 9 stages") { iirContinuityTest<Q15T>(); }
TEST_CASE("[C4 arm_iir_lattice_q31] blocks of 1, 2, 3, 4, 5, 7 and 1+4+3+9+2 samples give bit for bit the output and state of one long block, 1, 2, 5, 9 stages") { iirContinuityTest<Q31T>(); }

TEST_CASE("[C4 arm_iir_lattice_f32] two instances with different coefficients, called alternately, do not influence each other") { iirTwoInstancesTest<F32T>(); }
TEST_CASE("[C4 arm_iir_lattice_q15] two instances with different coefficients, called alternately, do not influence each other") { iirTwoInstancesTest<Q15T>(); }
TEST_CASE("[C4 arm_iir_lattice_q31] two instances with different coefficients, called alternately, do not influence each other") { iirTwoInstancesTest<Q31T>(); }

TEST_CASE("[C4 arm_iir_lattice_f32] init again on the same state buffer clears the history: the run repeats bit for bit") { iirReinitTest<F32T>(); }
TEST_CASE("[C4 arm_iir_lattice_q15] init again on the same state buffer clears the history: the run repeats bit for bit") { iirReinitTest<Q15T>(); }
TEST_CASE("[C4 arm_iir_lattice_q31] init again on the same state buffer clears the history: the run repeats bit for bit") { iirReinitTest<Q31T>(); }

TEST_CASE("[C4 arm_iir_lattice_init_f32] init stores numStages, both coefficient pointers, pState and zeroes exactly numStages + blockSize elements (1..9, 12, 17 stages, blockSize 1 and 7)") { iirInitTest<F32T>(); }
TEST_CASE("[C4 arm_iir_lattice_init_q15] init stores numStages, both coefficient pointers, pState and zeroes exactly numStages + blockSize elements (1..9, 12, 17 stages, blockSize 1 and 7)") { iirInitTest<Q15T>(); }
TEST_CASE("[C4 arm_iir_lattice_init_q31] init stores numStages, both coefficient pointers, pState and zeroes exactly numStages + blockSize elements (1..9, 12, 17 stages, blockSize 1 and 7)") { iirInitTest<Q31T>(); }

TEST_CASE("[C2 arm_iir_lattice_f32] blocks of 1, 2, 3, 4, 5, 9 samples with 1, 4, 5, 9 stages write exactly blockSize outputs and nothing behind the exact size (numStages + blockSize) state") { iirGuardsTest<F32T>(); }
TEST_CASE("[C2 arm_iir_lattice_q15] blocks of 1, 2, 3, 4, 5, 9 samples with 1, 4, 5, 9 stages write exactly blockSize outputs and nothing behind the exact size (numStages + blockSize) state") { iirGuardsTest<Q15T>(); }
TEST_CASE("[C2 arm_iir_lattice_q31] blocks of 1, 2, 3, 4, 5, 9 samples with 1, 4, 5, 9 stages write exactly blockSize outputs and nothing behind the exact size (numStages + blockSize) state") { iirGuardsTest<Q31T>(); }

TEST_CASE("[C3 arm_iir_lattice_q15] one stage with k = 0: y[n] = sat16(floor((v1 x[n-1] + v0 x[n]) / 2^15)) for every x[0] in 1.15 and two x[1], floor error in [0, 1) LSB") { iirSweepTest<Q15T>(); }
TEST_CASE("[C3 arm_iir_lattice_q31] one stage with k = 0: y[n] = floor((v1 x[n-1] + v0 x[n]) / 2^31) for x[0] over the full range in steps of 40009 and two x[1], floor error in [0, 1) LSB") { iirSweepTest<Q31T>(); }
TEST_CASE("[C4 arm_iir_lattice_q15] a sum above +1 or below -1 at the output is saturated to 32767 / -32768 (DOC: the accumulator is saturated to 1.15)") { iirQ15OutputSaturationTest(); }

TEST_CASE("characterization: [C3 arm_iir_lattice_q15] output and state equal the integer model with floor products (DOC silent about the product rounding), 1..9, 12 and 17 stages, 7 partitions") { iirExactModelTest<Q15T>(false); }
TEST_CASE("characterization: [C4 arm_iir_lattice_q15] full range data: every f and g is clipped to 1.15 as the model with __SSAT says (DOC silent about intermediate saturation)") { iirExactModelTest<Q15T>(true); }
TEST_CASE("characterization: [C3 arm_iir_lattice_q31] output and state equal the integer model with floor products (DOC silent about the product rounding), 1..9, 12 and 17 stages, 7 partitions") { iirExactModelTest<Q31T>(false); }
TEST_CASE("characterization: [C4 arm_iir_lattice_q31] full range data: every f and g is clipped to 1.31 as the model with clip_q63_to_q31 says (DOC silent about intermediate saturation)") { iirExactModelTest<Q31T>(true); }

TEST_CASE("[C3 arm_iir_lattice_f32] the impulse response of a 2 stage filter with k1 = 0.5, k2 = 0.25 equals the hand derived {0.28125, 0.15234375, -0.04052734375}") { iirTwoStageImpulseTest<F32T>(); }
TEST_CASE("[C3 arm_iir_lattice_q15] the impulse response of a 2 stage filter with k1 = 0.5, k2 = 0.25 equals the hand derived {0.28125, 0.15234375, -0.04052734375}") { iirTwoStageImpulseTest<Q15T>(); }
TEST_CASE("[C3 arm_iir_lattice_q31] the impulse response of a 2 stage filter with k1 = 0.5, k2 = 0.25 equals the hand derived {0.28125, 0.15234375, -0.04052734375}") { iirTwoStageImpulseTest<Q31T>(); }
TEST_CASE("[C4 arm_fir_lattice_f32] two instances sharing one coefficient array (allowed by the DOC) give the single instance results") { firSharedCoefsTest<F32T>(true); }
TEST_CASE("[C4 arm_fir_lattice_q15] two instances sharing one coefficient array (allowed by the DOC) give the single instance results (signed data, blocks of 3)") { firSharedCoefsTest<Q15T>(true); }
TEST_CASE("[C4 arm_fir_lattice_q31] two instances sharing one coefficient array (allowed by the DOC) give the single instance results (non-negative data)") { firSharedCoefsTest<Q31T>(false); }
TEST_CASE("[C4 arm_iir_lattice_f32] two instances sharing the coefficient arrays (allowed by the DOC) give the single instance results") { iirSharedCoefsTest<F32T>(); }
TEST_CASE("[C4 arm_iir_lattice_q15] two instances sharing the coefficient arrays (allowed by the DOC) give the single instance results") { iirSharedCoefsTest<Q15T>(); }
TEST_CASE("[C4 arm_iir_lattice_q31] two instances sharing the coefficient arrays (allowed by the DOC) give the single instance results") { iirSharedCoefsTest<Q31T>(); }
