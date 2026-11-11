/*
 * CMSIS-DSP Filtering, FIR float (work order FIRF, phase P11 group G4): arm_fir_f32 / arm_fir_init_f32 and arm_fir_f64 /
 * arm_fir_init_f64 (the f16 FIR is in test_dsp_filt_fir_f16.cpp). Requirement ids: Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * Pattern files (Libraries/CMSIS-DSP/Testing/Patterns/DSP/Filtering/FIR/FIRF32/ and FIRF64/), suites "FIR F32" and "FIR F64"
 * (Testing/desc.txt:3151-3185), ARM test functions Testing/Source/Tests/FIRF32.cpp (test_fir_f32, :30-:127) and FIRF64.cpp
 * (test_fir_f64). ids: arm_fir_f32 (1 test), arm_fir_f64 (1 test); nothing is commented out or disabled. Both use
 *   FirInput1 (input), FirCoefs1 (the coefficients of ALL configs, concatenated), FirConfigs1_s16 (pairs blockSize, numTaps:
 *   blockSize in {1,2,3,8,9,10,11,16,23} x numTaps in {1,2,3,4,5,6,7,8,11,16,23,25} = 108 configs) -> FirRefs1 (2*blockSize outputs
 *   per config, concatenated). For every config the filter is initialised with (numTaps, coefs, state, blockSize) and the
 *   first 2*blockSize inputs are filtered in TWO calls of blockSize (state continuity). Same here, with a state buffer of
 *   exactly numTaps + blockSize - 1 elements (the scalar / non Helium layout) and a guard behind every output block (ARM
 *   checks 4 zeros behind each block, checkInnerTail, and ASSERT_EMPTY_TAIL).
 * ARM tolerances: f32 SNR_THRESHOLD 120 (FIRF32.cpp:5), REL_ERROR 3.0e-5 (:13); f64 SNR_THRESHOLD 310 (FIRF64.cpp:5),
 * REL_ERROR 5.0e-14 (:13). ASSERT_REL_ERROR is |a-b| / ((|a|+|b|)/2) <= thr, nothing compared if both are 0
 * (FrameworkSource/Error.cpp:205-:220); armRel() below does the same and also fails on NaN. Used unchanged (L4).
 *
 * DOC (arm_fir_f32.c:42-60, arm_fir_init_f32.c): y[n] = b[0]*x[n] + b[1]*x[n-1] + ... + b[numTaps-1]*x[n-numTaps+1];
 * "pCoeffs points to a coefficient array of size numTaps. Coefficients are stored in time reversed order
 * {b[numTaps-1], ..., b[1], b[0]}"; "pState points to a state array of size numTaps + blockSize - 1"; init "Sets the values of
 * the internal structure fields. Zeros out the values in the state buffer." So for the array c = pCoeffs, DEF:
 *   y[n] = sum_{k=0}^{T-1} c[k] * x[n-T+1+k]  (x[m] = 0 for m < 0), i.e. c[T-1] multiplies the newest sample.
 * The DEF is evaluated in long double. The ARM reference patterns are checked against this DEF too (convention check).
 *
 * Error bounds (Higham, dot product): u = 2^-24 (f32) or 2^-53 (f64) is the unit roundoff; gam(k) = k*u/(1-k*u). Every
 * output is a sum of T rounded products accumulated in order (in any order and with or without FMA the bound holds):
 *   |y - ydef| <= gam(T) * S + T * sub,   S = sum_k |c[k] * x[n-T+1+k]|,
 * sub = the smallest subnormal / 2 (absolute error of a product in the subnormal range; zero for the data used here
 * except the sweeps), plus the error of the DEF itself, gam_ld(T) * S with u_ld = numeric_limits<long double>::epsilon()/2
 * (2^-64 on x86, 2^-53 where long double is double, e.g. MSVC / macOS arm64: the bound then doubles the f64 allowance but
 * stays valid, since DEF and library are both bounded against the exact value). No magic number.
 * Resolution of the bounds (by design): the smallest relative gain error a bound can detect is about gam(T)+gam_ld(T)
 * ~ 2*T*u of S per output (f32: T=1 -> 1.2e-7, T=33 -> 3.9e-6; f64 about 2.2e-16*T), so a mutant that perturbs a
 * coefficient or product by less than that is invisible to the random / sweep tests; the exact (dyadic) impulse and step tests
 * and the bit exact one tap sweeps are the tight ones.
 *
 * Host vs target (C6): the host runs the portable C path (no ARM_MATH_DSP / MVE / Neon); the target (-O2) may contract
 * a*b+c into VFMA (-ffp-contract=fast): the bound above holds for fused and unfused. The "exact" tests use dyadic data
 * (multiples of 1/16, small), for which every product and sum is exactly representable, so fused or not gives the same
 * result. The sweeps include subnormal inputs (a Cortex-M4F with FPSCR.FZ set would flush them; the reset value of FZ is 0).
 * The FIR kernels unroll the block by 8 (f32 and f64 portable code) and the taps by 8 (f64) / 4 (f32): block sizes and
 * numTaps 1..9, 12, 17, 25 and 33 are used.
 *
 * Not tested, NEEDS REVIEW: blockSize 0 (DOC silent; the kernel and the init arithmetic for 0 are unspecified); numTaps 0.
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

namespace {

const double kU64 = 1.1102230246251565e-16;  // 2^-53

struct F32 {
	typedef float S;
	typedef arm_fir_instance_f32 Inst;
	static const char *name() { return "f32"; }
	static constexpr double u = 5.9604644775390625e-8;  // 2^-24
	static constexpr double sub = 7.006492321624085e-46;  // 2^-150
	static void init(Inst *i, uint16_t t, const S *c, S *st, uint32_t b) { arm_fir_init_f32(i, t, c, st, b); }
	static void run(const Inst *i, const S *x, S *y, uint32_t b) { arm_fir_f32(i, x, y, b); }
	static S from(double d) { return static_cast<float>(d); }
	static double to(S v) { return static_cast<double>(v); }
	static constexpr size_t nCfg = 216;  // 2 values per config
	static std::string dir() { return "Filtering/FIR/FIRF32/"; }
	static std::vector<S> pat(const std::string &n) { return dsp::loadPattern(dir() + n).f32(); }
	// strided bit pattern sweep window: biased exponent 70..185 (2^-57 .. 2^58), so that no product leaves the normal range
	static constexpr uint64_t stride = 4093ull;
	static constexpr uint64_t span = 0x100000000ull;
	static bool pick(uint64_t bits, S &v) {
		const uint32_t b = static_cast<uint32_t>(bits);
		std::memcpy(&v, &b, sizeof v);
		const int e = static_cast<int>((b >> 23) & 0xFF);
		return e >= 70 && e <= 185;
	}
};

struct F64 {
	typedef double S;
	typedef arm_fir_instance_f64 Inst;
	static const char *name() { return "f64"; }
	static constexpr double u = 1.1102230246251565e-16;  // 2^-53
	static constexpr double sub = 2.4703282292062328e-324;  // 2^-1075 (rounds to 0 as a double constant, kept for the formula)
	static void init(Inst *i, uint16_t t, const S *c, S *st, uint32_t b) { arm_fir_init_f64(i, t, c, st, b); }
	static void run(const Inst *i, const S *x, S *y, uint32_t b) { arm_fir_f64(i, x, y, b); }
	static S from(double d) { return d; }
	static double to(S v) { return v; }
	static constexpr size_t nCfg = 216;
	static std::string dir() { return "Filtering/FIR/FIRF64/"; }
	static std::vector<S> pat(const std::string &n) { return dsp::loadPattern(dir() + n).f64(); }
	// biased exponent 600..1400 (2^-423 .. 2^377); stride odd, ~1.05e6 patterns over the 64 bit range (wraps at 2^64)
	static constexpr uint64_t stride = 17592186044417ull;
	static constexpr uint64_t span = 0;  // unused: the sweep is limited by the pattern count
	static bool pick(uint64_t bits, S &v) {
		std::memcpy(&v, &bits, sizeof v);
		const int e = static_cast<int>((bits >> 52) & 0x7FF);
		return e >= 600 && e <= 1400;
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
// One tap: y[n] = c0 * x[n]. The product of two floats is exact in double (24+24 bits), so (S)(double product) is the
// correctly rounded product that IEEE multiplication gives; adding the accumulator start 0 changes nothing (zero sign
// compares equal). Every non NaN input bit pattern (strided over the whole 32 bit range, i.e. all exponents, both signs,
// subnormals, infinities) must give exactly that.
void sweepOneTapF32() {
	const float c0 = 0.7f;
	const float cArr[1] = {c0};
	const uint32_t stride = 4093u;  // prime: ~1.05e6 patterns
	const size_t BLK = 4096;
	std::vector<float> x(BLK), y(BLK);
	std::vector<float> state(BLK);  // numTaps + blockSize - 1 = 1 + 4096 - 1
	arm_fir_instance_f32 inst;
	arm_fir_init_f32(&inst, 1, cArr, state.data(), BLK);
	size_t n = 0, bad = 0, tested = 0;
	uint32_t firstBad = 0;
	uint64_t bits = 0;
	for (; bits < 0x100000000ull; bits += stride) {
		const uint32_t b = static_cast<uint32_t>(bits);
		float f;
		std::memcpy(&f, &b, sizeof f);
		if (std::isnan(f)) continue;
		x[n++] = f;
		if (n == BLK) {
			arm_fir_f32(&inst, x.data(), y.data(), BLK);
			for (size_t i = 0; i < BLK; i++) {
				const float e = static_cast<float>(static_cast<double>(c0) * static_cast<double>(x[i]));
				if (!(y[i] == e)) {
					if (bad == 0) std::memcpy(&firstBad, &x[i], sizeof firstBad);
					bad++;
				}
			}
			tested += BLK;
			n = 0;
		}
	}
	CHECK(tested > 1000000);
	CHECK_MESSAGE(bad == 0, bad << " products differ, first input bits 0x" << std::hex << firstBad);
}

void sweepOneTapF64() {
	const double c0 = 0.7;
	const double cArr[1] = {c0};
	const uint64_t stride = 17592186044417ull;  // odd: ~1.05e6 patterns over the 64 bit range
	const size_t BLK = 4096;
	std::vector<double> x(BLK), y(BLK), state(BLK);
	arm_fir_instance_f64 inst;
	arm_fir_init_f64(&inst, 1, cArr, state.data(), BLK);
	size_t n = 0, bad = 0, tested = 0;
	uint64_t firstBad = 0;
	uint64_t bits = 0;
	for (uint64_t k = 0; k < 1100000ull; k++, bits += stride) {
		double f;
		std::memcpy(&f, &bits, sizeof f);
		if (std::isnan(f)) continue;
		x[n++] = f;
		if (n == BLK) {
			arm_fir_f64(&inst, x.data(), y.data(), BLK);
			for (size_t i = 0; i < BLK; i++) {
				volatile double e = c0 * x[i];  // IEEE double product (the DEF of a one tap filter), no fusion possible
				if (!(y[i] == e)) {
					if (bad == 0) std::memcpy(&firstBad, &x[i], sizeof firstBad);
					bad++;
				}
			}
			tested += BLK;
			n = 0;
		}
	}
	CHECK(tested > 1000000);
	CHECK_MESSAGE(bad == 0, bad << " products differ, first input bits 0x" << std::hex << firstBad);
}

// Two taps over the range of normal inputs of the type (biased exponent window, so that no product leaves the normal
// range): y[n] = c0*x[n-1] + c1*x[n] against the long double DEF with the bound of the header (T = 2).
template <class Tr>
void sweepTwoTapTest() {
	typedef typename Tr::S S;
	const S c[2] = {Tr::from(0.7), Tr::from(-1.3)};
	const std::vector<S> cv(c, c + 2);
	const size_t BLK = 4096;
	std::vector<S> x(BLK), y(BLK), state(BLK + 1);
	typename Tr::Inst inst;
	Tr::init(&inst, 2, c, state.data(), BLK);
	size_t n = 0, bad = 0, tested = 0;
	uint64_t bits = 0;
	for (uint64_t count = 0; count < 1100000ull; count++, bits += Tr::stride) {
		if (sizeof(S) == 4 && bits >= Tr::span) break;
		S v;
		if (!Tr::pick(bits, v)) continue;
		x[n++] = v;
		if (n == BLK) {
			Tr::run(&inst, x.data(), y.data(), BLK);
			std::vector<double> yd, sa;
			defFir<Tr>(cv, x, yd, sa);
			// element 0 of the block continues the previous block (history of the previous call): the DEF restarts
			// from zero history, so skip it here; every other element only uses the block
			for (size_t i = 1; i < BLK; i++) {
				if (!(std::fabs(Tr::to(y[i]) - yd[i]) <= bound<Tr>(2, sa[i]))) bad++;
			}
			tested += BLK;
			n = 0;
		}
	}
	CHECK(tested >= 4096 * 20);
	CHECK_MESSAGE(bad == 0, bad << " outputs outside the bound");
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------
// arm_fir_f32
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_fir_f32] ARM's pattern test (108 configs, two blocks each) reaches SNR 120 and relative error 3e-5") {
	armPatternTest<F32>(120.0, 3.0e-5);  // FIRF32.cpp:5, :13
}
TEST_CASE("[C3 arm_fir_f32] ARM's reference pattern equals the difference equation of the DOC (c[T-1] weights the newest sample)") {
	patternIsDefTest<F32>();
}
TEST_CASE("[C3 arm_fir_f32] the impulse response is the coefficients in reverse order, for 1 to 12 taps") { impulseTest<F32>(); }
TEST_CASE("[C3 arm_fir_f32] the step response is the running sum of b[0..m] and then the sum of all coefficients") { stepTest<F32>(); }
TEST_CASE("[C3 arm_fir_f32] random data of every block size 1..17 and 1..33 taps is within n*eps*sum|terms| of the DEF") { randomVsDefTest<F32>(); }
TEST_CASE("[C2 arm_fir_f32] a call writes exactly blockSize outputs and nothing behind them or behind the exact size state") { writesExactlyBlockTest<F32>(); }
TEST_CASE("[C4 arm_fir_f32] consecutive blocks of sizes 1, 2, 3, 4, 5, 7 and mixed carry the state like one long block") { continuityTest<F32>(); }
TEST_CASE("[C4 arm_fir_f32] two instances with different coefficients do not influence each other") { independentInstancesTest<F32>(); }
TEST_CASE("[C4 arm_fir_f32] init again clears a garbage state: the run repeats bit for bit") { reinitTest<F32>(); }
TEST_CASE("[C4 arm_fir_init_f32] init stores numTaps, pCoeffs, pState and zeroes exactly numTaps + blockSize - 1 elements") { initTest<F32>(); }
TEST_CASE("[C4 arm_fir_f32] the coefficient and input arrays are not modified") { constInputsTest<F32>(); }
TEST_CASE("[C3 arm_fir_f32] a one tap filter returns the correctly rounded product for strided inputs over the whole float range") { sweepOneTapF32(); }
TEST_CASE("[C3 arm_fir_f32] a two tap filter is within the bound of the DEF over 2^-57 .. 2^58, strided") { sweepTwoTapTest<F32>(); }

TEST_CASE("[C5 arm_fir_f32] the f32 result agrees with arm_fir_f64 on the same data within the f32 bound") {
	uint32_t seed = 2000;
	size_t bad = 0;
	for (size_t T : kTaps) {
		const std::vector<float> c = randVec<F32>(T, seed++, -1.0, 1.0);
		const std::vector<float> x = randVec<F32>(30, seed++, -1.0, 1.0);
		const std::vector<double> c64(c.begin(), c.end()), x64(x.begin(), x.end());
		const RunResult<F32> r32 = firRun<F32>(c, x, {30});
		const RunResult<F64> r64 = firRun<F64>(c64, x64, {30});
		std::vector<double> y, sa;
		defFir<F32>(c, x, y, sa);
		for (size_t n = 0; n < 30; n++) {
			// both within their own bound of the DEF (f64 bound is negligible), so within the sum of the two of each other
			const double b = bound<F32>(T, sa[n]) + bound<F64>(T, sa[n]);
			if (!(std::fabs(static_cast<double>(r32.out[n]) - r64.out[n]) <= b)) bad++;
		}
	}
	CHECK(bad == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// arm_fir_f64
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_fir_f64] ARM's pattern test (108 configs, two blocks each) reaches SNR 310 and relative error 5e-14") {
	armPatternTest<F64>(310.0, 5.0e-14);  // FIRF64.cpp:5, :13
}
TEST_CASE("[C3 arm_fir_f64] ARM's reference pattern equals the difference equation of the DOC (c[T-1] weights the newest sample)") {
	patternIsDefTest<F64>();
}
TEST_CASE("[C3 arm_fir_f64] the impulse response is the coefficients in reverse order, for 1 to 12 taps") { impulseTest<F64>(); }
TEST_CASE("[C3 arm_fir_f64] the step response is the running sum of b[0..m] and then the sum of all coefficients") { stepTest<F64>(); }
TEST_CASE("[C3 arm_fir_f64] random data of every block size 1..17 and 1..33 taps is within n*eps*sum|terms| of the DEF") { randomVsDefTest<F64>(); }
TEST_CASE("[C2 arm_fir_f64] a call writes exactly blockSize outputs and nothing behind them or behind the exact size state") { writesExactlyBlockTest<F64>(); }
TEST_CASE("[C4 arm_fir_f64] consecutive blocks of sizes 1, 2, 3, 4, 5, 7 and mixed carry the state like one long block") { continuityTest<F64>(); }
TEST_CASE("[C4 arm_fir_f64] two instances with different coefficients do not influence each other") { independentInstancesTest<F64>(); }
TEST_CASE("[C4 arm_fir_f64] init again clears a garbage state: the run repeats bit for bit") { reinitTest<F64>(); }
TEST_CASE("[C4 arm_fir_init_f64] init stores numTaps, pCoeffs, pState and zeroes exactly numTaps + blockSize - 1 elements") { initTest<F64>(); }
TEST_CASE("[C4 arm_fir_f64] the coefficient and input arrays are not modified") { constInputsTest<F64>(); }
TEST_CASE("[C3 arm_fir_f64] a one tap filter returns the IEEE product for strided inputs over the whole double range") { sweepOneTapF64(); }
TEST_CASE("[C3 arm_fir_f64] a two tap filter is within the bound of the DEF over 2^-423 .. 2^377, strided") { sweepTwoTapTest<F64>(); }
