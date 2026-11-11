/*
 * CMSIS-DSP Filtering, FIR f16 (work order FIRF, phase P11 group G4): arm_fir_f16 / arm_fir_init_f16 (f32 and f64 are in
 * test_dsp_filt_fir_float.cpp). Requirement ids: Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 * Compiled with -D__ARM_FP16_FORMAT_IEEE=1 -D__fp16=_Float16 (file name ends in _f16.cpp, lead decision L2).
 *
 * Pattern files (Libraries/CMSIS-DSP/Testing/Patterns/DSP/Filtering/FIR/FIRF16/), suite "FIR F16" (Testing/desc_f16.txt:727-:745),
 * ARM test function test_fir_f16 (Testing/Source/Tests/FIRF16.cpp:30-:135); the only id (arm_fir_f16) is enabled. Inputs
 * FirInput1_f16, FirCoefs1_f16 (coefficients of all configs, concatenated), FirConfigs1_s16 (pairs blockSize, numTaps: blockSize
 * in {1,2,3,12,13,14,15} x numTaps in {2..8,11,25} = 63 configs, not the f32 / f64 set), reference FirRefs1_f16 (2*blockSize outputs per
 * config). For every config: init(numTaps, coefs, state, blockSize), then the first 2*blockSize inputs in TWO calls of
 * blockSize. Same here with a state of exactly numTaps + blockSize - 1 elements (the scalar layout; the Helium layout of
 * the DOC needs more and is not the one compiled) and a guard behind every output block (ARM: checkInnerTail,
 * ASSERT_EMPTY_TAIL).
 * ARM tolerances: SNR_THRESHOLD 60 (FIRF16.cpp:5), REL_ERROR 2.0e-2 (:13); ASSERT_REL_ERROR is
 * |a-b| / ((|a|+|b|)/2) <= thr, nothing compared if both are 0 (FrameworkSource/Error.cpp, the float16_t overload computes in
 * float); armRel() does the same in double and also fails on NaN. Used unchanged (L4).
 *
 * DOC (arm_fir_f16.c, arm_fir_init_f16.c): y[n] = b[0]*x[n] + ... + b[T-1]*x[n-T+1]; "pCoeffs ... coefficients stored in
 * time reversed order {b[T-1], ..., b[0]}"; "pState is of length numTaps+blockSize-1 samples (except for Helium)"; init
 * "Sets the values of the internal structure fields. Zeros out the values in the state buffer." DEF for c = pCoeffs:
 *   y[n] = sum_k c[k] * x[n-T+1+k] (x[m] = 0 for m < 0), in long double (the f16 values are exact in it).
 *
 * Error bounds (Higham, dot product): u = 2^-11 (binary16 unit roundoff), gam(k) = k*u/(1-k*u). The portable code
 * accumulates in _Float16 (acc0 ... acc7 are _Float16): 
 *   |y - ydef| <= gam(T) * S + T * 2^-25 + gam_ld(T) * S,   S = sum_k |c[k] x[n-T+1+k]|,
 * 2^-25 = absolute error of one product in the subnormal range, gam_ld (u_ld = numeric_limits<long double>::epsilon()/2) = the long double DEF itself. Valid for any order of
 * accumulation, with or without FMA, with or without float excess precision (fewer roundings). Resolution (by design): the smallest relative gain error a bound can detect is about gam(T) ~ T*2^-11 of S per output
 * (4.9e-4 for T=1, 1.6e-2 for T=33); the exact impulse / step tests and the exhaustive one tap sweep are the tight ones.
 * The tests keep T*u < 1
 * (T <= 33: 33 * 4.9e-4 = 0.016).
 * The ARM reference pattern was computed from the UNROUNDED data (coefficients and inputs rounded to f16 afterwards: 2*u*S),
 * in double (gam_64) and rounded to f16 (u*|y|): bound for the pattern-vs-DEF test.
 *
 * Host vs target (C6): _Float16 is emulated by the compiler on x86-64 (operations in float, rounded back at the latest at every
 * store; with -fexcess-precision=fast intermediates may stay in float), on a Cortex-M55 (MVE) the kernel differs (lane sums
 * with FMA): the bounds above hold for all of them. The firmware ships no f16 code (the Makefile excludes %_f16.c); these
 * tests only protect the library as such. The exhaustive sweeps cover all 65536 binary16 bit patterns (one tap) and the
 * normal window 2^-10 .. 2^10 (two taps); Cortex-M f16 subnormal handling (FZ) is not covered.
 *
 * Not tested, NEEDS REVIEW: blockSize 0 (DOC silent); numTaps 0.
 */
#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math.h"
#include "arm_math_f16.h"

#ifndef ARM_FLOAT16_SUPPORTED
#error "ARM_FLOAT16_SUPPORTED is not defined: the f16 compile definitions are missing for this file"
#endif

namespace {

const double kU64 = 1.1102230246251565e-16;  // 2^-53

struct F16 {
	typedef float16_t S;
	typedef arm_fir_instance_f16 Inst;
	static const char *name() { return "f16"; }
	static constexpr double u = 4.8828125e-4;    // 2^-11
	static constexpr double sub = 2.9802322387695312e-8;  // 2^-25
	static void init(Inst *i, uint16_t t, const S *c, S *st, uint32_t b) { arm_fir_init_f16(i, t, c, st, b); }
	static void run(const Inst *i, const S *x, S *y, uint32_t b) { arm_fir_f16(i, x, y, b); }
	static S from(double d) { return static_cast<float16_t>(d); }
	static double to(S v) { return static_cast<double>(v); }
	static constexpr size_t nCfg = 126;  // 2 values per config
	static std::string dir() { return "Filtering/FIR/FIRF16/"; }
	static std::vector<S> pat(const std::string &n) {
		const std::vector<uint16_t> bits = dsp::loadPattern(dir() + n).f16Bits();
		std::vector<S> v(bits.size());
		for (size_t i = 0; i < bits.size(); i++) std::memcpy(&v[i], &bits[i], sizeof(S));
		return v;
	}
};

double gam(double k, double u) { return k * u / (1.0 - k * u); }

// Deterministic LCG; every call is its own statement (lesson: never two PRNG calls in one expression)
struct Rng {
	uint32_t s;
	explicit Rng(uint32_t seed) : s(seed * 2654435761u + 12345u) {}
	double next() {  // uniform in [0, 1)
		s = s * 1664525u + 1013904223u;
		return static_cast<double>(s >> 8) / 16777216.0;
	}
};

template <class Tr>
std::vector<typename Tr::S> randVec(size_t n, uint32_t seed, double lo, double hi) {
	Rng rng(seed);
	std::vector<typename Tr::S> v(n);
	for (size_t i = 0; i < n; i++) {
		double r = rng.next();
		v[i] = Tr::from(lo + (hi - lo) * r);
	}
	return v;
}

// ARM's ASSERT_REL_ERROR (Error.cpp:205-:220), plus NaN fails
template <class A, class B>
dsp::Check armRel(const std::vector<A> &ref, const std::vector<B> &out, double thr) {
	if (ref.size() != out.size()) return dsp::detail::lengthMismatch(ref.size(), out.size());
	for (size_t i = 0; i < ref.size(); i++) {
		const double a = static_cast<double>(out[i]), b = static_cast<double>(ref[i]);
		if (std::isnan(a) || std::isnan(b)) return dsp::detail::fail("relative error", i, a, b, thr);
		const double avg = (std::fabs(a) + std::fabs(b)) / 2.0;
		if (avg != 0.0 && std::fabs(a - b) / avg > thr) return dsp::detail::fail("relative error", i, a, b, thr);
	}
	return dsp::Check();
}

// DEF (long double): y[n] = sum_k c[k] * x[n-T+1+k]; S[n] = sum_k |c[k] * x[n-T+1+k]|
template <class Tr>
void defFir(const std::vector<typename Tr::S> &c, const std::vector<typename Tr::S> &x, std::vector<double> &y,
            std::vector<double> &sabs) {
	const long T = static_cast<long>(c.size());
	y.assign(x.size(), 0.0);
	sabs.assign(x.size(), 0.0);
	for (long n = 0; n < static_cast<long>(x.size()); n++) {
		long double acc = 0.0L, sa = 0.0L;
		for (long k = 0; k < T; k++) {
			const long m = n - T + 1 + k;
			if (m < 0) continue;
			const long double p = static_cast<long double>(Tr::to(c[static_cast<size_t>(k)])) *
			                      static_cast<long double>(Tr::to(x[static_cast<size_t>(m)]));
			acc += p;
			sa += (p < 0 ? -p : p);
		}
		y[static_cast<size_t>(n)] = static_cast<double>(acc);
		sabs[static_cast<size_t>(n)] = static_cast<double>(sa);
	}
}

// bound of the header: gam(T)*S + T*sub + gam_ld(T)*S
template <class Tr>
double bound(size_t T, double sabs) {
	return gam(static_cast<double>(T), Tr::u) * sabs + static_cast<double>(T) * Tr::sub + gam(static_cast<double>(T), std::numeric_limits<long double>::epsilon() / 2) * sabs;
}

template <class Tr>
struct RunResult {
	std::vector<typename Tr::S> out;
	bool guardsOk = true;     // every output block guard and the state guard intact
	bool noneLeftPoison = true;
};

// Filters x with taps c in consecutive calls of the sizes in `chunks` (cycled, last one truncated). The state buffer has
// exactly numTaps + maxChunk - 1 elements and is initialised with blockSize = maxChunk.
template <class Tr>
RunResult<Tr> firRun(const std::vector<typename Tr::S> &c, const std::vector<typename Tr::S> &x, const std::vector<size_t> &chunks,
                     bool guarded = false) {
	typedef typename Tr::S S;
	RunResult<Tr> r;
	size_t maxChunk = 0;
	for (size_t k : chunks) maxChunk = k > maxChunk ? k : maxChunk;
	// Default: the state is its own heap allocation of EXACTLY numTaps + maxChunk - 1 elements (no guard slack inside the same
	// allocation, so ASan sees a write behind it); guarded = true uses a GuardedBuffer instead (guard words test, not ASan).
	const size_t nState = c.size() + maxChunk - 1;
	dsp::GuardedBuffer<S> state(guarded ? nState : 1);
	std::vector<S> tight(guarded ? 1 : nState);
	S *stp = guarded ? state.data() : tight.data();
	if (!guarded) std::memset(static_cast<void *>(stp), 0xCD, nState * sizeof(S));  // poison: a missing clear is visible
	typename Tr::Inst inst;
	Tr::init(&inst, static_cast<uint16_t>(c.size()), c.data(), stp, static_cast<uint32_t>(maxChunk));
	size_t pos = 0, ci = 0;
	while (pos < x.size()) {
		size_t b = chunks[ci++ % chunks.size()];
		if (b > x.size() - pos) b = x.size() - pos;
		dsp::GuardedBuffer<S> out(b);
		Tr::run(&inst, x.data() + pos, out.data(), static_cast<uint32_t>(b));
		if (!out.guardIntact()) r.guardsOk = false;
		const std::vector<S> o = out.output();
		for (size_t i = 0; i < b; i++) {
			unsigned char raw[sizeof(S)];
			std::memcpy(raw, &o[i], sizeof(S));
			bool poison = true;
			for (unsigned char byte : raw) poison = poison && (byte == 0xCD);
			if (poison) r.noneLeftPoison = false;
			r.out.push_back(o[i]);
		}
		pos += b;
	}
	if (guarded && !state.guardIntact()) r.guardsOk = false;
	return r;
}

// ---------------------------------------------------------------------------------------------------------------
// C1: ARM's test
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
void armPatternTest(double snrThr, double relThr) {
	typedef typename Tr::S S;
	const char *sfx = Tr::name();
	const std::vector<S> input = Tr::pat(std::string("FirInput1_") + sfx + ".txt");
	const std::vector<S> coefs = Tr::pat(std::string("FirCoefs1_") + sfx + ".txt");
	const std::vector<S> ref = Tr::pat(std::string("FirRefs1_") + sfx + ".txt");
	const std::vector<int16_t> cfg = dsp::loadPattern(Tr::dir() + "FirConfigs1_s16.txt").q15();
	const size_t nCfg = Tr::nCfg;
	REQUIRE(cfg.size() == nCfg);
	std::vector<S> all;
	size_t cp = 0;
	bool guards = true, fields = true;
	for (size_t i = 0; i + 1 < cfg.size(); i += 2) {
		const size_t B = static_cast<size_t>(cfg[i]), T = static_cast<size_t>(cfg[i + 1]);
		REQUIRE(cp + T <= coefs.size());
		REQUIRE(2 * B <= input.size());
		std::vector<S> state(T + B - 1, Tr::from(0.0));  // exact size heap allocation (ASan sees an overrun)
		std::memset(static_cast<void *>(state.data()), 0xCD, state.size() * sizeof(S));
		typename Tr::Inst inst;
		Tr::init(&inst, static_cast<uint16_t>(T), coefs.data() + cp, state.data(), static_cast<uint32_t>(B));
		fields = fields && inst.numTaps == T && inst.pCoeffs == coefs.data() + cp && inst.pState == state.data();
		for (size_t blk = 0; blk < 2; blk++) {
			dsp::GuardedBuffer<S> out(B);
			Tr::run(&inst, input.data() + blk * B, out.data(), static_cast<uint32_t>(B));
			guards = guards && out.guardIntact();
			const std::vector<S> o = out.output();
			all.insert(all.end(), o.begin(), o.end());
		}
		cp += T;
	}
	CHECK(fields);
	CHECK(guards);
	CHECK(cp == coefs.size());
	REQUIRE(all.size() == ref.size());
	CHECK(dsp::snr(ref, all) >= snrThr);
	auto c = armRel(ref, all, relThr);
	CHECK_MESSAGE(c, c.message);
}

// ARM reference pattern against the DEF (checks the coefficient order convention of the DEF)
template <class Tr>
void patternIsDefTest() {
	typedef typename Tr::S S;
	const char *sfx = Tr::name();
	const std::vector<S> input = Tr::pat(std::string("FirInput1_") + sfx + ".txt");
	const std::vector<S> coefs = Tr::pat(std::string("FirCoefs1_") + sfx + ".txt");
	const std::vector<S> ref = Tr::pat(std::string("FirRefs1_") + sfx + ".txt");
	const std::vector<int16_t> cfg = dsp::loadPattern(Tr::dir() + "FirConfigs1_s16.txt").q15();
	size_t cp = 0, rp = 0;
	size_t bad = 0;
	std::string first;
	for (size_t i = 0; i + 1 < cfg.size(); i += 2) {
		const size_t B = static_cast<size_t>(cfg[i]), T = static_cast<size_t>(cfg[i + 1]);
		const std::vector<S> c(coefs.begin() + static_cast<long>(cp), coefs.begin() + static_cast<long>(cp + T));
		const std::vector<S> x(input.begin(), input.begin() + static_cast<long>(2 * B));
		std::vector<double> y, sa;
		defFir<Tr>(c, x, y, sa);
		for (size_t n = 0; n < 2 * B; n++) {
			// the reference is a double result of the UNROUNDED coefficients and inputs (each rounded to the type afterwards:
			// 2*u*S) rounded to the type itself (u*|y|); the script's double sum has error gam(T)*S at
			// 2^-53, my long double DEF about the same at most: 2*gam_64(T)*S
			const double b = Tr::u * std::fabs(y[n]) + 2.0 * Tr::u * sa[n] + 2.0 * gam(static_cast<double>(T), kU64) * sa[n];
			const double e = std::fabs(Tr::to(ref[rp + n]) - y[n]);
			if (!(e <= b)) {
				if (bad == 0) first = "config " + std::to_string(i / 2) + " sample " + std::to_string(n);
				bad++;
			}
		}
		cp += T;
		rp += 2 * B;
	}
	CHECK_MESSAGE(bad == 0, bad << " samples differ from the DEF, first at " << first);
}

// ---------------------------------------------------------------------------------------------------------------
// C3 / C4
// ---------------------------------------------------------------------------------------------------------------
const size_t kTaps[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 17, 24, 25, 32, 33};
const size_t kBlocks[] = {1, 2, 3, 4, 5, 7, 8, 9, 16, 17};

// Impulse response: x = {1, 0, ...}: y[m] = b[m] = c[T-1-m] for m < T, then 0. Dyadic coefficients: exact.
template <class Tr>
void impulseTest() {
	typedef typename Tr::S S;
	for (size_t T = 1; T <= 12; T++) {
		std::vector<S> c(T);
		for (size_t k = 0; k < T; k++) c[k] = Tr::from(((k % 2) ? -1.0 : 1.0) * static_cast<double>(k + 1) / 16.0);
		std::vector<S> x(T + 5, Tr::from(0.0));
		x[0] = Tr::from(1.0);
		const RunResult<Tr> r = firRun<Tr>(c, x, {T + 5});
		std::vector<S> expect(T + 5, Tr::from(0.0));
		for (size_t m = 0; m < T; m++) expect[m] = c[T - 1 - m];
		auto chk = dsp::exactEqual(expect, r.out);
		CHECK_MESSAGE(chk, "numTaps " << T << ": " << chk.message);
		CHECK(r.guardsOk);
	}
}

// Step response: x = 1: y[m] = b[0] + ... + b[min(m, T-1)] = sum_{j=T-1-m}^{T-1} c[j] for m < T, then the sum of all.
template <class Tr>
void stepTest() {
	typedef typename Tr::S S;
	for (size_t T = 1; T <= 12; T++) {
		std::vector<S> c(T);
		for (size_t k = 0; k < T; k++) c[k] = Tr::from(((k % 3 == 1) ? -1.0 : 1.0) * static_cast<double>(k + 1) / 16.0);
		std::vector<S> x(T + 5, Tr::from(1.0));
		const RunResult<Tr> r = firRun<Tr>(c, x, {T + 5});
		std::vector<S> expect(T + 5);
		for (size_t m = 0; m < T + 5; m++) {
			double acc = 0.0;
			const size_t lo = (m < T) ? (T - 1 - m) : 0;
			for (size_t j = lo; j < T; j++) acc += Tr::to(c[j]);  // dyadic, exact in double
			expect[m] = Tr::from(acc);
		}
		auto chk = dsp::exactEqual(expect, r.out);
		CHECK_MESSAGE(chk, "numTaps " << T << ": " << chk.message);
	}
}

// Random data, one call per block size from a zero state, against the DEF with the bound of the header
template <class Tr>
void randomVsDefTest() {
	typedef typename Tr::S S;
	uint32_t seed = 1;
	size_t bad = 0;
	std::string first;
	for (size_t T : kTaps) {
		for (size_t B : kBlocks) {
			const std::vector<S> c = randVec<Tr>(T, seed++, -1.0, 1.0);
			const std::vector<S> x = randVec<Tr>(B, seed++, -1.0, 1.0);
			std::vector<double> y, sa;
			defFir<Tr>(c, x, y, sa);
			const RunResult<Tr> r = firRun<Tr>(c, x, {B});
			REQUIRE(r.out.size() == B);
			for (size_t n = 0; n < B; n++) {
				const double e = std::fabs(Tr::to(r.out[n]) - y[n]);
				if (!(e <= bound<Tr>(T, sa[n]))) {
					if (bad == 0) first = "taps " + std::to_string(T) + " block " + std::to_string(B) + " n " + std::to_string(n);
					bad++;
				}
			}
		}
	}
	CHECK_MESSAGE(bad == 0, bad << " outputs outside the bound, first at " << first);
}

template <class Tr>
void writesExactlyBlockTest() {
	typedef typename Tr::S S;
	uint32_t seed = 500;
	for (size_t T : {1u, 4u, 5u, 8u, 9u}) {
		for (size_t B : kBlocks) {
			const std::vector<S> c = randVec<Tr>(T, seed++, -0.5, 0.5);
			const std::vector<S> x = randVec<Tr>(B, seed++, -0.5, 0.5);
			const RunResult<Tr> r = firRun<Tr>(c, x, {B}, true);
			CHECK_MESSAGE(r.guardsOk, "taps " << T << " block " << B << ": guard behind the output or the state damaged");
			CHECK_MESSAGE(r.noneLeftPoison, "taps " << T << " block " << B << ": an output element was not written");
		}
	}
}

// Consecutive blocks of several sizes carry the state: result agrees with the DEF of the whole stream
template <class Tr>
void continuityTest() {
	typedef typename Tr::S S;
	const std::vector<std::vector<size_t>> plans = {{1}, {2}, {3}, {4}, {5}, {7}, {1, 2, 3, 4, 5, 7}, {8, 9, 16}, {9, 1, 17, 2}, {40}};
	uint32_t seed = 900;
	size_t bad = 0;
	std::string first;
	for (size_t T : {1u, 2u, 3u, 4u, 5u, 8u, 9u, 12u, 17u, 24u, 32u}) {
		const std::vector<S> c = randVec<Tr>(T, seed++, -1.0, 1.0);
		const std::vector<S> x = randVec<Tr>(40, seed++, -1.0, 1.0);
		std::vector<double> y, sa;
		defFir<Tr>(c, x, y, sa);
		for (size_t p = 0; p < plans.size(); p++) {
			const RunResult<Tr> r = firRun<Tr>(c, x, plans[p]);
			REQUIRE(r.out.size() == x.size());
			CHECK(r.guardsOk);
			for (size_t n = 0; n < x.size(); n++) {
				const double e = std::fabs(Tr::to(r.out[n]) - y[n]);
				if (!(e <= bound<Tr>(T, sa[n]))) {
					if (bad == 0) first = "taps " + std::to_string(T) + " plan " + std::to_string(p) + " n " + std::to_string(n);
					bad++;
				}
			}
		}
	}
	CHECK_MESSAGE(bad == 0, bad << " outputs outside the bound, first at " << first);
}

// Two instances with different coefficients, interleaved calls: each equals its own DEF
template <class Tr>
void independentInstancesTest() {
	typedef typename Tr::S S;
	const size_t Ta = 5, Tb = 9, B = 7, blocks = 6;
	const std::vector<S> ca = randVec<Tr>(Ta, 31, -1.0, 1.0), cb = randVec<Tr>(Tb, 32, -1.0, 1.0);
	const std::vector<S> xa = randVec<Tr>(B * blocks, 33, -1.0, 1.0), xb = randVec<Tr>(B * blocks, 34, -1.0, 1.0);
	std::vector<S> sa(Ta + B - 1), sb(Tb + B - 1);  // exact size heap allocations
	typename Tr::Inst ia, ib;
	Tr::init(&ia, Ta, ca.data(), sa.data(), B);
	Tr::init(&ib, Tb, cb.data(), sb.data(), B);
	std::vector<S> oa, ob;
	for (size_t k = 0; k < blocks; k++) {
		std::vector<S> pa(B), pb(B);
		Tr::run(&ia, xa.data() + k * B, pa.data(), B);
		Tr::run(&ib, xb.data() + k * B, pb.data(), B);
		oa.insert(oa.end(), pa.begin(), pa.end());
		ob.insert(ob.end(), pb.begin(), pb.end());
	}
	std::vector<double> ya, yb, sca, scb;
	defFir<Tr>(ca, xa, ya, sca);
	defFir<Tr>(cb, xb, yb, scb);
	size_t bad = 0;
	for (size_t n = 0; n < oa.size(); n++) {
		if (!(std::fabs(Tr::to(oa[n]) - ya[n]) <= bound<Tr>(Ta, sca[n]))) bad++;
		if (!(std::fabs(Tr::to(ob[n]) - yb[n]) <= bound<Tr>(Tb, scb[n]))) bad++;
	}
	CHECK(bad == 0);
}

// Re-init with a poisoned state buffer resets the history: the second run is bit identical to the first
template <class Tr>
void reinitTest() {
	typedef typename Tr::S S;
	const size_t T = 6, B = 5;
	const std::vector<S> c = randVec<Tr>(T, 41, -1.0, 1.0);
	const std::vector<S> x = randVec<Tr>(3 * B, 42, -1.0, 1.0);
	dsp::GuardedBuffer<S> state(T + B - 1);
	typename Tr::Inst inst;
	Tr::init(&inst, T, c.data(), state.data(), B);
	std::vector<S> first(3 * B), second(3 * B);
	for (size_t k = 0; k < 3; k++) Tr::run(&inst, x.data() + k * B, first.data() + k * B, B);
	// the state now holds the last T-1 inputs: re-init must clear it even if the buffer is full of garbage
	std::memset(static_cast<void *>(state.data()), 0xCD, (T + B - 1) * sizeof(S));
	Tr::init(&inst, T, c.data(), state.data(), B);
	for (size_t k = 0; k < 3; k++) Tr::run(&inst, x.data() + k * B, second.data() + k * B, B);
	CHECK(std::memcmp(first.data(), second.data(), first.size() * sizeof(S)) == 0);
	CHECK(state.guardIntact());
}

// Init: fields hold the arguments, the state buffer (numTaps + blockSize - 1 elements) is zero, nothing behind it changes
template <class Tr>
void initTest() {
	typedef typename Tr::S S;
	const size_t cfg[][2] = {{1, 1}, {1, 9}, {9, 1}, {4, 4}, {8, 8}, {25, 23}, {5, 3}};
	for (const auto &t : cfg) {
		const size_t T = t[0], B = t[1];
		const std::vector<S> c = randVec<Tr>(T, 61, -1.0, 1.0);
		dsp::GuardedBuffer<S> state(T + B - 1);  // poisoned with 0xCD by the buffer
		typename Tr::Inst inst;
		std::memset(&inst, 0x5A, sizeof inst);
		Tr::init(&inst, static_cast<uint16_t>(T), c.data(), state.data(), static_cast<uint32_t>(B));
		CHECK(inst.numTaps == T);
		CHECK(inst.pCoeffs == c.data());
		CHECK(inst.pState == state.data());
		bool zero = true;
		const std::vector<S> st = state.output();
		for (const S &v : st) {
			unsigned char raw[sizeof(S)];
			std::memcpy(raw, &v, sizeof(S));
			for (unsigned char byte : raw) zero = zero && byte == 0;
		}
		CHECK_MESSAGE(zero, "taps " << T << " block " << B << ": state not zeroed");
		CHECK_MESSAGE(state.guardIntact(), "taps " << T << " block " << B << ": init wrote behind the state buffer");
	}
}

// The coefficients and the input are not modified by the filter
template <class Tr>
void constInputsTest() {
	typedef typename Tr::S S;
	const std::vector<S> c = randVec<Tr>(9, 71, -1.0, 1.0);
	const std::vector<S> x = randVec<Tr>(23, 72, -1.0, 1.0);
	std::vector<S> c2(c), x2(x);
	const RunResult<Tr> r = firRun<Tr>(c2, x2, {5, 3, 7});
	CHECK(r.out.size() == 23);
	CHECK(std::memcmp(c.data(), c2.data(), c.size() * sizeof(S)) == 0);
	CHECK(std::memcmp(x.data(), x2.data(), x.size() * sizeof(S)) == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// Sweeps
// ---------------------------------------------------------------------------------------------------------------
typedef F16::S H;

H fromBits(uint16_t b) {
	H h;
	std::memcpy(&h, &b, sizeof h);
	return h;
}

// One tap, every non NaN binary16 bit pattern (all exponents, both signs, subnormals, infinities): y = c0 * x. The product of
// two binary16 values has at most 22 significant bits, exact in double, so (f16)(double product) is the correctly rounded
// IEEE product; accumulator start 0 changes nothing (zero signs compare equal).
void sweepOneTap() {
	const H c0 = F16::from(0.7);
	const H cArr[1] = {c0};
	const size_t N = 65536;
	std::vector<H> x, y(N), state;
	for (uint32_t b = 0; b < 65536u; b++) {
		const H h = fromBits(static_cast<uint16_t>(b));
		if (!std::isnan(static_cast<double>(h))) x.push_back(h);
	}
	const size_t n = x.size();
	y.resize(n);
	state.resize(n);  // numTaps + blockSize - 1
	arm_fir_instance_f16 inst;
	arm_fir_init_f16(&inst, 1, cArr, state.data(), static_cast<uint32_t>(n));
	arm_fir_f16(&inst, x.data(), y.data(), static_cast<uint32_t>(n));
	size_t bad = 0;
	uint16_t firstBad = 0;
	for (size_t i = 0; i < n; i++) {
		const H e = static_cast<H>(static_cast<double>(c0) * static_cast<double>(x[i]));
		if (!(y[i] == e)) {
			if (bad == 0) std::memcpy(&firstBad, &x[i], sizeof firstBad);
			bad++;
		}
	}
	CHECK(n == 63490);  // 65536 - 2046 NaN patterns (2 * 1023)
	CHECK_MESSAGE(bad == 0, bad << " products differ, first input bits 0x" << std::hex << firstBad);
}

// Two taps over every pattern of the window biased exponent 5..25 (2^-10 .. 2^10; no product or sum leaves the normal range
// [2^-14, 65504]), in increasing bit order: y[n] = c0*x[n-1] + c1*x[n] against the DEF with the bound of the header (T = 2).
void sweepTwoTap() {
	const H c[2] = {F16::from(0.7), F16::from(-1.3)};
	const std::vector<H> cv(c, c + 2);
	std::vector<H> x;
	for (uint32_t b = 0; b < 65536u; b++) {
		const uint32_t e = (b >> 10) & 0x1F;
		if (e >= 5 && e <= 25) x.push_back(fromBits(static_cast<uint16_t>(b)));
	}
	const size_t n = x.size();
	std::vector<H> y(n), state(n + 1);
	arm_fir_instance_f16 inst;
	arm_fir_init_f16(&inst, 2, c, state.data(), static_cast<uint32_t>(n));
	arm_fir_f16(&inst, x.data(), y.data(), static_cast<uint32_t>(n));
	std::vector<double> yd, sa;
	defFir<F16>(cv, x, yd, sa);
	size_t bad = 0;
	for (size_t i = 0; i < n; i++)
		if (!(std::fabs(F16::to(y[i]) - yd[i]) <= bound<F16>(2, sa[i]))) bad++;
	CHECK(n > 40000);
	CHECK_MESSAGE(bad == 0, bad << " outputs outside the bound");
}

}  // namespace

TEST_CASE("[C1 arm_fir_f16] ARM's pattern test (63 configs, two blocks each) reaches SNR 60 and relative error 2e-2") {
	armPatternTest<F16>(60.0, 2.0e-2);  // FIRF16.cpp:5, :13
}
TEST_CASE("[C3 arm_fir_f16] ARM's reference pattern equals the difference equation of the DOC (c[T-1] weights the newest sample)") {
	patternIsDefTest<F16>();
}
TEST_CASE("[C3 arm_fir_f16] the impulse response is the coefficients in reverse order, for 1 to 12 taps") { impulseTest<F16>(); }
TEST_CASE("[C3 arm_fir_f16] the step response is the running sum of b[0..m] and then the sum of all coefficients") { stepTest<F16>(); }
TEST_CASE("[C3 arm_fir_f16] random data of every block size 1..17 and 1..33 taps is within n*eps*sum|terms| of the DEF") { randomVsDefTest<F16>(); }
TEST_CASE("[C2 arm_fir_f16] a call writes exactly blockSize outputs and nothing behind them or behind the exact size state") { writesExactlyBlockTest<F16>(); }
TEST_CASE("[C4 arm_fir_f16] consecutive blocks of sizes 1, 2, 3, 4, 5, 7 and mixed carry the state like one long block") { continuityTest<F16>(); }
TEST_CASE("[C4 arm_fir_f16] two instances with different coefficients do not influence each other") { independentInstancesTest<F16>(); }
TEST_CASE("[C4 arm_fir_f16] init again clears a garbage state: the run repeats bit for bit") { reinitTest<F16>(); }
TEST_CASE("[C4 arm_fir_init_f16] init stores numTaps, pCoeffs, pState and zeroes exactly numTaps + blockSize - 1 elements") { initTest<F16>(); }
TEST_CASE("[C4 arm_fir_f16] the coefficient and input arrays are not modified") { constInputsTest<F16>(); }
TEST_CASE("[C3 arm_fir_f16] a one tap filter returns the correctly rounded product for all 63490 non NaN binary16 inputs") { sweepOneTap(); }
TEST_CASE("[C3 arm_fir_f16] a two tap filter is within the bound of the DEF for every input of 2^-10 .. 2^10") { sweepTwoTap(); }

TEST_CASE("[C5 arm_fir_f16] the f16 result agrees with arm_fir_f32 on the same (f16 exact) data within the f16 bound") {
	uint32_t seed = 3000;
	size_t bad = 0;
	for (size_t T : kTaps) {
		const std::vector<H> c = randVec<F16>(T, seed++, -1.0, 1.0);
		const std::vector<H> x = randVec<F16>(30, seed++, -1.0, 1.0);
		const std::vector<float> c32(c.begin(), c.end()), x32(x.begin(), x.end());
		std::vector<float> state(T + 29), y32(30);
		arm_fir_instance_f32 inst;
		arm_fir_init_f32(&inst, static_cast<uint16_t>(T), c32.data(), state.data(), 30);
		arm_fir_f32(&inst, x32.data(), y32.data(), 30);
		const RunResult<F16> r16 = firRun<F16>(c, x, {30});
		std::vector<double> y, sa;
		defFir<F16>(c, x, y, sa);
		for (size_t n = 0; n < 30; n++) {
			// f32 is within gam_32(T)*S of the DEF (negligible next to f16 but added), f16 within the f16 bound
			const double b = bound<F16>(T, sa[n]) + gam(static_cast<double>(T), 5.9604644775390625e-8) * sa[n];
			if (!(std::fabs(F16::to(r16.out[n]) - static_cast<double>(y32[n])) <= b)) bad++;
		}
	}
	CHECK(bad == 0);
}
