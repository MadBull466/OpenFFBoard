/*
 * CMSIS-DSP Filtering, correlation / Levinson-Durbin f16 (work order CVF, phase P11 group G5): arm_correlate_f16 and
 * arm_levinson_durbin_f16 (f32 / f64 units are in test_dsp_filt_conv_float.cpp). Requirement ids:
 * Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md. The definitions, DOC quotes and conventions (output index i of
 * correlate is lag i-(M-1), M = max(na, nb), zero initialised output; Yule-Walker definition of Levinson-Durbin) are the
 * ones of test_dsp_filt_conv_float.cpp.
 *
 * ARM tests: suite "MISC F16" (Testing/desc_f16.txt:566-717, class Testing/Source/Tests/MISCF16.cpp), patterns
 * Libraries/CMSIS-DSP/Testing/Patterns/DSP/Filtering/MISC/MISCF16/. ids: arm_correlate_f16 (40: nba in {4,5,6,9,10,11,12,13} x
 * nbb in {1,2,3,8,11}, ref ids 1-40, inputs the first nba / nbb samples of InputsA1 / InputsB1), arm_levinson_durbin_f16
 * (n = 7, 16, 23; phi InputPhi81/82/83, errors LDErrors81[0..2], ref ids 81-83). MISCF16.cpp also contains arm_conv_f16 but the
 * library has no such function and desc_f16.txt does not list it. Nothing is disabled.
 * ARM tolerances: SNR_THRESHOLD 60 (MISCF16.cpp:6), REL_ERROR 1e-4 / ABS_ERROR 1e-3 (:13-:14); Levinson-Durbin SNR_LD_THRESHOLD
 * 52 (:21), REL_LD_ERROR / ABS_LD_ERROR 1e-3 (:22-:23); err is checked with the LD abs / rel values. Used unchanged (L4).
 *
 * Error bound (random tests): |y - ydef| <= gam(T) S + T sub with u = 2^-11, sub = 2^-25 (smallest f16 subnormal / 2), T =
 * min(na, nb) (Higham dot product; the f16 kernel accumulates in f16; the long double DEF error is negligible but added).
 * Data in [-1, 1] keeps every sum far below the f16 maximum 65504. Levinson-Durbin random tests: the same conditioning bound as
 * in the float file, n * kappa_inf * u * max(1, |a|_inf) with u = 2^-11, restricted to n <= 7 and mild AR(1) mixtures.
 *
 * Host vs target (C6): _Float16 is emulated on x86-64 (operations in float, rounded at every store at the latest; with
 * -fexcess-precision=fast intermediates may stay in float); on a Cortex-M55 the kernel uses MVE lanes with different summation
 * order: the bounds hold for all. CONSEQUENCE: on the host (ARM_MATH_DSP and MVE
 * undefined) arm_correlate_f16 (arm_correlate_f16.c:1025) runs the generic loop, not the unrolled / MVE kernels of a
 * Cortex-M target: the length sweeps (1-9, 12, 13, 17) test the generic loop and do not exercise the unroll tails of target
 * kernels. The firmware itself ships no f16 code (Makefile excludes %_f16.c); these tests protect the
 * library only.
 * Not tested, NEEDS REVIEW: srcALen / srcBLen / nbCoefs 0; phi[0] == 0; aliasing.
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

struct F16 {
	typedef float16_t S;
	static constexpr double u = 4.8828125e-4;             // 2^-11
	static constexpr double sub = 2.9802322387695312e-8;  // 2^-25
	static S from(double d) { return static_cast<float16_t>(d); }
	static double to(S v) { return static_cast<double>(v); }
	static std::string dir() { return "Filtering/MISC/MISCF16/"; }
	static std::vector<S> pat(const std::string &n) {
		const std::vector<uint16_t> bits = dsp::loadPattern(dir() + n).f16Bits();
		std::vector<S> v(bits.size());
		for (size_t i = 0; i < bits.size(); i++) std::memcpy(&v[i], &bits[i], sizeof(S));
		return v;
	}
};

typedef std::vector<float16_t> V;

double gam(double k, double u) { return k * u / (1.0 - k * u); }
const double kLdU = std::numeric_limits<long double>::epsilon() / 2;

struct Rng {
	uint32_t s;
	explicit Rng(uint32_t seed) : s(seed * 2654435761u + 12345u) {}
	double next() {
		s = s * 1664525u + 1013904223u;
		return static_cast<double>(s >> 8) / 16777216.0;
	}
};

V randVec(size_t n, uint32_t seed, double lo, double hi) {
	Rng rng(seed);
	V v(n);
	for (size_t i = 0; i < n; i++) {
		double r = rng.next();
		v[i] = F16::from(lo + (hi - lo) * r);
	}
	return v;
}

// multiples of 1/4 in [-1, 1]: products are multiples of 1/16 and any sum of at most 9 of them has magnitude <= 9, i.e. at
// most 9*16 = 144 < 2^11 steps of 1/16: exact in f16 (lengths up to 9 only).
V dyadicVec(size_t n, uint32_t seed) {
	Rng rng(seed);
	V v(n);
	for (size_t i = 0; i < n; i++) {
		double r = rng.next();
		v[i] = F16::from(std::floor(r * 9.0) / 4.0 - 1.0);
	}
	return v;
}

V head(const V &v, size_t n) { return V(v.begin(), v.begin() + static_cast<long>(n)); }

double boundFor(size_t T, double sabs) {
	return gam(static_cast<double>(T), F16::u) * sabs + static_cast<double>(T) * F16::sub + gam(static_cast<double>(T), kLdU) * sabs;
}

void defCorr(const V &a, const V &b, std::vector<double> &y, std::vector<double> &sabs) {
	const long na = static_cast<long>(a.size()), nb = static_cast<long>(b.size());
	const long M = na > nb ? na : nb;
	y.assign(static_cast<size_t>(2 * M - 1), 0.0);
	sabs.assign(y.size(), 0.0);
	for (long i = 0; i < 2 * M - 1; i++) {
		const long l = i - (M - 1);
		long double acc = 0, sa = 0;
		for (long k = 0; k < nb; k++) {
			if (k + l < 0 || k + l >= na) continue;
			const long double p = static_cast<long double>(F16::to(a[static_cast<size_t>(k + l)])) * static_cast<long double>(F16::to(b[static_cast<size_t>(k)]));
			acc += p;
			sa += (p < 0 ? -p : p);
		}
		y[static_cast<size_t>(i)] = static_cast<double>(acc);
		sabs[static_cast<size_t>(i)] = static_cast<double>(sa);
	}
}

V runCorrG(const V &a, const V &b, bool &guard) {
	const size_t M = a.size() > b.size() ? a.size() : b.size();
	dsp::GuardedBuffer<float16_t> g(2 * M - 1);
	std::memset(static_cast<void *>(g.data()), 0, g.size() * sizeof(float16_t));
	arm_correlate_f16(a.data(), static_cast<uint32_t>(a.size()), b.data(), static_cast<uint32_t>(b.size()), g.data());
	guard = g.guardIntact();
	return g.output();
}

V runCorrT(const V &a, const V &b) {
	const size_t M = a.size() > b.size() ? a.size() : b.size();
	V o(2 * M - 1, F16::from(0.0));  // exact size
	arm_correlate_f16(a.data(), static_cast<uint32_t>(a.size()), b.data(), static_cast<uint32_t>(b.size()), o.data());
	return o;
}

const size_t kNa[] = {4, 5, 6, 9, 10, 11, 12, 13};
const size_t kNb[] = {1, 2, 3, 8, 11};
const size_t kLens[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 13, 17};

void armCorrelateTest() {
	const V A = F16::pat("InputsA1_f16.txt"), B = F16::pat("InputsB1_f16.txt");
	size_t id = 0;
	for (size_t na : kNa)
		for (size_t nb : kNb) {
			id++;
			const V ref = F16::pat("Reference" + std::to_string(id) + "_f16.txt");
			bool guard = false;
			const V out = runCorrG(head(A, na), head(B, nb), guard);
			CHECK_MESSAGE(guard, "guard damaged, nba " << na << " nbb " << nb);
			REQUIRE(ref.size() == out.size());
			CHECK_MESSAGE(dsp::snr(ref, out) >= 60.0, "SNR " << dsp::snr(ref, out) << " nba " << na << " nbb " << nb);
			auto c = dsp::closeAbsRel(ref, out, 1e-3, 1e-4);
			CHECK_MESSAGE(c, "nba " << na << " nbb " << nb << ": " << c.message);
		}
	CHECK(id == 40);
}

void patternIsDef() {
	const V A = F16::pat("InputsA1_f16.txt"), B = F16::pat("InputsB1_f16.txt");
	size_t id = 0, bad = 0;
	for (size_t na : kNa)
		for (size_t nb : kNb) {
			id++;
			const V ref = F16::pat("Reference" + std::to_string(id) + "_f16.txt");
			std::vector<double> y, sa;
			defCorr(head(A, na), head(B, nb), y, sa);
			REQUIRE(ref.size() == y.size());
			for (size_t i = 0; i < y.size(); i++) {
				// the reference: double result of the unrounded data rounded to f16: u|y| + 2 u S (inputs rounded), DEF error
				const double b = F16::u * std::fabs(y[i]) + 2.0 * F16::u * sa[i] + 2.0 * gam(static_cast<double>(na < nb ? na : nb), 1.1102230246251565e-16) * sa[i];
				if (!(std::fabs(F16::to(ref[i]) - y[i]) <= b)) bad++;
			}
		}
	CHECK_MESSAGE(bad == 0, bad << " reference samples differ from the DEF");
}

void handTest() {
	// see test_dsp_filt_conv_float.cpp corrHandTest for the derivations (small integers: exact in f16)
	struct Case { V a, b, e; };
	const Case cs[] = {
	    {{F16::from(1), F16::from(2), F16::from(3)}, {F16::from(0), F16::from(1)}, {F16::from(0), F16::from(1), F16::from(2), F16::from(3), F16::from(0)}},
	    {{F16::from(1), F16::from(2)}, {F16::from(1), F16::from(10), F16::from(100)}, {F16::from(100), F16::from(210), F16::from(21), F16::from(2), F16::from(0)}},
	    {{F16::from(1), F16::from(10), F16::from(100)}, {F16::from(1), F16::from(2)}, {F16::from(0), F16::from(2), F16::from(21), F16::from(210), F16::from(100)}}};
	for (const Case &c : cs) {
		bool guard = false;
		const V o = runCorrG(c.a, c.b, guard);
		auto r = dsp::exactEqual(c.e, o);
		CHECK_MESSAGE(r, r.message);
		CHECK(guard);
	}
}

void randomTest() {
	size_t bad = 0, guardBad = 0, tightDiff = 0, runs = 0;
	for (size_t na : kLens)
		for (size_t nb : kLens) {
			const V a = randVec(na, static_cast<uint32_t>(100 + na), -1.0, 1.0);
			const V b = randVec(nb, static_cast<uint32_t>(200 + nb), -1.0, 1.0);
			std::vector<double> y, sa;
			defCorr(a, b, y, sa);
			bool guard = false;
			const V o = runCorrG(a, b, guard);
			const V t = runCorrT(a, b);
			if (!guard) guardBad++;
			if (std::memcmp(o.data(), t.data(), o.size() * sizeof(float16_t)) != 0) tightDiff++;
			const size_t T = na < nb ? na : nb;
			REQUIRE(o.size() == y.size());
			for (size_t i = 0; i < y.size(); i++) {
				const double v = F16::to(o[i]);
				if (std::isnan(v) || !(std::fabs(v - y[i]) <= boundFor(T, sa[i]))) bad++;
			}
			runs++;
		}
	CHECK(runs == 144);
	CHECK_MESSAGE(bad == 0, bad << " outputs outside the bound (lengths 1-9, 12, 13, 17 both orders)");
	CHECK(guardBad == 0);
	CHECK(tightDiff == 0);
}

// multiples of 1/4: products multiples of 1/16, |sums| <= 9, so every result is a multiple of 1/16 below 16: at most 8 bits.
// Exact in f16 for lengths up to 9 (the sums stay below 9 in magnitude).
void dyadicExactTest() {
	size_t bad = 0;
	for (size_t na = 1; na <= 9; na++)
		for (size_t nb = 1; nb <= 9; nb++) {
			const V a = dyadicVec(na, static_cast<uint32_t>(300 + na));
			const V b = dyadicVec(nb, static_cast<uint32_t>(400 + nb));
			std::vector<double> y, sa;
			defCorr(a, b, y, sa);
			const V o = runCorrT(a, b);
			for (size_t i = 0; i < y.size(); i++)
				if (!(F16::to(o[i]) == y[i])) bad++;
		}
	CHECK_MESSAGE(bad == 0, bad << " outputs differ from the exact value (lengths 1-9, both orders)");
}

void swapTest() {
	size_t bad = 0;
	for (size_t na = 1; na <= 9; na++)
		for (size_t nb = 1; nb <= 9; nb++) {
			const V a = dyadicVec(na, static_cast<uint32_t>(500 + na));
			const V b = dyadicVec(nb, static_cast<uint32_t>(600 + nb));
			const V ab = runCorrT(a, b), ba = runCorrT(b, a);
			for (size_t i = 0; i < ab.size(); i++)
				if (!(ab[i] == ba[ab.size() - 1 - i])) bad++;
		}
	CHECK_MESSAGE(bad == 0, bad << " outputs differ (lengths 1-9, both orders)");
}

void constInputsTest() {
	const V a = randVec(9, 801, -1.0, 1.0), b = randVec(5, 802, -1.0, 1.0);
	const V a0 = a, b0 = b;
	(void)runCorrT(a, b);
	CHECK(std::memcmp(a.data(), a0.data(), a.size() * sizeof(float16_t)) == 0);
	CHECK(std::memcmp(b.data(), b0.data(), b.size() * sizeof(float16_t)) == 0);
}

// ---------------- Levinson-Durbin ----------------
void defLevinson(const std::vector<double> &phi, size_t n, std::vector<double> &a, double &err, double &kappa) {
	typedef long double LD;
	std::vector<std::vector<LD> > M(n, std::vector<LD>(2 * n + 1, 0.0L));
	for (size_t i = 0; i < n; i++) {
		for (size_t j = 0; j < n; j++) M[i][j] = phi[i > j ? i - j : j - i];
		M[i][n + i] = 1.0L;
		M[i][2 * n] = phi[i + 1];
	}
	LD normR = 0;
	for (size_t i = 0; i < n; i++) {
		LD s = 0;
		for (size_t j = 0; j < n; j++) s += std::fabs(M[i][j]);
		normR = s > normR ? s : normR;
	}
	for (size_t c = 0; c < n; c++) {
		size_t p = c;
		for (size_t r = c + 1; r < n; r++)
			if (std::fabs(M[r][c]) > std::fabs(M[p][c])) p = r;
		std::swap(M[c], M[p]);
		for (size_t r = 0; r < n; r++) {
			if (r == c) continue;
			const LD f = M[r][c] / M[c][c];
			for (size_t k = 0; k < 2 * n + 1; k++) M[r][k] -= f * M[c][k];
		}
	}
	a.assign(n, 0.0);
	LD normInv = 0, e = phi[0];
	for (size_t i = 0; i < n; i++) {
		a[i] = static_cast<double>(M[i][2 * n] / M[i][i]);
		e -= (M[i][2 * n] / M[i][i]) * phi[i + 1];
		LD s = 0;
		for (size_t j = 0; j < n; j++) s += std::fabs(M[i][n + j] / M[i][i]);
		normInv = s > normInv ? s : normInv;
	}
	err = static_cast<double>(e);
	kappa = static_cast<double>(normR * normInv);
}

// mild mixture: two AR(1) autocovariances with |r| < 0.6, weights sum to 1 (phi[0] = 1)
std::vector<double> mixPhi(size_t n, uint32_t seed) {
	Rng rng(seed);
	double r[2], w[2], ws = 0;
	for (int m = 0; m < 2; m++) {
		double x = rng.next();
		r[m] = 1.2 * x - 0.6;
		double y = rng.next();
		w[m] = 0.3 + y;
		ws += w[m];
	}
	std::vector<double> phi(n + 1, 0.0);
	for (size_t l = 0; l <= n; l++)
		for (int m = 0; m < 2; m++) phi[l] += (w[m] / ws) * std::pow(r[m], static_cast<double>(l));
	return phi;
}

struct LdRun {
	V a;
	float16_t err;
	bool guard;
};

LdRun runLd(const V &phi, size_t n) {
	LdRun r;
	dsp::GuardedBuffer<float16_t> g(n);
	r.err = F16::from(-123.0);
	arm_levinson_durbin_f16(phi.data(), g.data(), &r.err, static_cast<int>(n));
	r.guard = g.guardIntact();
	r.a = g.output();
	return r;
}

void armLevinsonTest() {
	const size_t ns[] = {7, 16, 23};
	const char *phiName[] = {"InputPhi81_f16.txt", "InputPhi82_f16.txt", "InputPhi83_f16.txt"};
	const V errs = F16::pat("LDErrors81_f16.txt");
	for (size_t t = 0; t < 3; t++) {
		const V phi = F16::pat(phiName[t]);
		const V ref = F16::pat("Reference" + std::to_string(81 + t) + "_f16.txt");
		REQUIRE(phi.size() == ns[t] + 1);
		REQUIRE(ref.size() == ns[t]);
		REQUIRE(errs.size() > t);
		const LdRun r = runLd(phi, ns[t]);
		CHECK_MESSAGE(r.guard, "guard damaged, n " << ns[t]);
		CHECK_MESSAGE(dsp::snr(ref, r.a) >= 52.0, "SNR " << dsp::snr(ref, r.a) << " n " << ns[t]);  // SNR_LD_THRESHOLD, MISCF16.cpp:21
		auto c = dsp::closeAbsRel(ref, r.a, 1e-3, 1e-3);                                             // :22-:23
		CHECK_MESSAGE(c, "n " << ns[t] << ": " << c.message);
		auto ce = dsp::closeAbsRel(V(1, errs[t]), V(1, r.err), 1e-3, 1e-3);
		CHECK_MESSAGE(ce, "err, n " << ns[t] << ": " << ce.message);
	}
}

void ldHandTest() {
	{  // phi = {2,1}: a = {1/2}, err = 3/2 (exact in f16)
		const LdRun r = runLd({F16::from(2), F16::from(1)}, 1);
		CHECK(r.guard);
		CHECK(F16::to(r.a[0]) == 0.5);
		CHECK(F16::to(r.err) == 1.5);
	}
	{  // phi = {1, 1/2, 1/4, 1/8}: a = {1/2, 0, 0}, err = 3/4 (all intermediates dyadic: exact)
		const LdRun r = runLd({F16::from(1), F16::from(0.5), F16::from(0.25), F16::from(0.125)}, 3);
		CHECK(r.guard);
		CHECK(F16::to(r.a[0]) == 0.5);
		CHECK(F16::to(r.a[1]) == 0.0);
		CHECK(F16::to(r.a[2]) == 0.0);
		CHECK(F16::to(r.err) == 0.75);
	}
	{  // phi = {4,2,2}: a = {1/3, 1/3}, err = 8/3; about 8 roundings of relative size u = 2^-11 (see the float file)
		const LdRun r = runLd({F16::from(4), F16::from(2), F16::from(2)}, 2);
		CHECK(r.guard);
		const double tol = 8.0 * F16::u;
		CHECK(std::fabs(F16::to(r.a[0]) - 1.0 / 3.0) <= tol / 3.0);
		CHECK(std::fabs(F16::to(r.a[1]) - 1.0 / 3.0) <= tol / 3.0);
		CHECK(std::fabs(F16::to(r.err) - 8.0 / 3.0) <= tol * 8.0 / 3.0);
	}
}

void ldRandomTest() {
	const size_t ns[] = {1, 2, 3, 4, 5, 6, 7};
	size_t bad = 0, guardBad = 0, runs = 0;
	for (size_t n : ns)
		for (uint32_t seed = 1; seed <= 6; seed++) {
			const std::vector<double> p = mixPhi(n, seed * 31 + static_cast<uint32_t>(n));
			V phi(p.size());
			for (size_t i = 0; i < p.size(); i++) phi[i] = F16::from(p[i]);
			std::vector<double> pr(p.size()), a;
			for (size_t i = 0; i < p.size(); i++) pr[i] = F16::to(phi[i]);
			double e = 0, kappa = 0;
			defLevinson(pr, n, a, e, kappa);
			double amax = 1;
			for (double v : a) amax = std::fabs(v) > amax ? std::fabs(v) : amax;
			const double bnd = static_cast<double>(n) * kappa * F16::u * amax;
			const LdRun r = runLd(phi, n);
			if (!r.guard) guardBad++;
			for (size_t i = 0; i < n; i++)
				if (!(std::fabs(F16::to(r.a[i]) - a[i]) <= bnd)) bad++;
			if (!(std::fabs(F16::to(r.err) - e) <= bnd * pr[0])) bad++;
			runs++;
		}
	CHECK(runs == 42);
	CHECK_MESSAGE(bad == 0, bad << " values outside the bound");
	CHECK(guardBad == 0);
}

void ldTightTest() {
	for (size_t n : {1u, 2u, 3u, 5u, 7u}) {
		const std::vector<double> p = mixPhi(n, 90 + static_cast<uint32_t>(n));
		V phi(p.size());  // exactly n + 1 values
		for (size_t i = 0; i < p.size(); i++) phi[i] = F16::from(p[i]);
		V a(n, F16::from(0.0));
		float16_t err = F16::from(0.0);
		arm_levinson_durbin_f16(phi.data(), a.data(), &err, static_cast<int>(n));
		const LdRun r = runLd(phi, n);
		CHECK(std::memcmp(a.data(), r.a.data(), n * sizeof(float16_t)) == 0);
		CHECK(std::memcmp(&err, &r.err, sizeof(float16_t)) == 0);
	}
}

}  // namespace

TEST_CASE("[C1 arm_correlate_f16] ARM's 40 pattern cases reach SNR 60, abs 1e-3 and rel 1e-4 with a zero guard behind the output") { armCorrelateTest(); }
TEST_CASE("[C3 arm_correlate_f16] ARM's 40 reference patterns equal the DEF (output index i is lag i-(M-1))") { patternIsDef(); }
TEST_CASE("[C3 arm_correlate_f16] hand derived cases for na > nb and na < nb, padding zero at the start / end") { handTest(); }
TEST_CASE("[C3 arm_correlate_f16] random data, all length pairs of 1-9, 12, 13, 17 are within the f16 bound of the DEF, guard intact") { randomTest(); }
TEST_CASE("[C5 arm_correlate_f16] exact dyadic data, all length pairs of 1-9, equals the exact result of the f32 / double DEF") { dyadicExactTest(); }
TEST_CASE("[C4 arm_correlate_f16] swapping the arguments reverses the output, for all length pairs of 1-9") { swapTest(); }
TEST_CASE("[C4 arm_correlate_f16] the input arrays are not modified") { constInputsTest(); }

TEST_CASE("[C1 arm_levinson_durbin_f16] ARM's cases n = 7, 16, 23 reach SNR 52, abs and rel 1e-3 for the coefficients and for err, guard intact") { armLevinsonTest(); }
TEST_CASE("[C3 arm_levinson_durbin_f16] hand derived Yule-Walker cases n = 1, 2, 3") { ldHandTest(); }
TEST_CASE("[C3 arm_levinson_durbin_f16] well conditioned random autocovariances (n 1-7) are within the conditioning bound of the DEF") { ldRandomTest(); }
TEST_CASE("[C2 arm_levinson_durbin_f16] exactly n coefficients are written (tight allocation, n 1, 2, 3, 5, 7)") { ldTightTest(); }
