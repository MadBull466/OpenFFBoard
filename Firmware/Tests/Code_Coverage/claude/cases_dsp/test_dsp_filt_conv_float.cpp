/*
 * CMSIS-DSP Filtering, convolution / correlation / Levinson-Durbin float (work order CVF, phase P11 group G5):
 * arm_conv_f32, arm_conv_partial_f32, arm_correlate_f32, arm_correlate_f64, arm_levinson_durbin_f32 (the f16 correlate and
 * Levinson-Durbin are in test_dsp_filt_conv_f16.cpp). Requirement ids: Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * ARM tests: suites "MISC F32" (Testing/desc.txt:2474-2672, class Testing/Source/Tests/MISCF32.cpp) and "MISC F64"
 * (desc.txt:2275-2473, MISCF64.cpp). Pattern files Libraries/CMSIS-DSP/Testing/Patterns/DSP/Filtering/MISC/MISCF32/ and MISCF64/.
 *   F32 ids: arm_correlate_f32 (40: nba in {4,5,6,9,10,11,12,13} x nbb in {1,2,3,8,11}, ref ids 1-40), arm_conv_f32 (40, same
 *   lengths, ref ids 41-80), arm_levinson_durbin_f32 (n = 3, 8, 11; phi InputPhi81/82/83, errors LDErrors81[0..2], ref ids 81-83),
 *   arm_conv_partial_f32 (nba 6, nbb 8, first = 3, 9, 7, NBPOINTS 4 (MISCF32.cpp:79), inputs InputsA2/B2, ref ids 84-86).
 *   Inputs: the first nba samples of InputsA1 and the first nbb of InputsB1 (the framework reloads with nba / nbb samples).
 *   F64 ids: arm_correlate_f64 (40, same lengths) is ENABLED in desc.txt. The ids arm_conv_f64, arm_levinson_durbin_f64 and
 *   arm_conv_partial_f64 are `disabled{...}` in desc.txt AND the three functions do not exist in the library (no arm_conv_f64.c,
 *   no arm_conv_partial_f64.c, no arm_levinson_durbin_f64.c; the F64 test bodies for levinson are commented out): they cannot
 *   be run, reported only.
 * ARM tolerances: f32 SNR_THRESHOLD 120 (MISCF32.cpp:8), REL_ERROR 1e-6 / ABS_ERROR 1e-5 (:15-:16), REL_LD_ERROR / ABS_LD_ERROR
 * 1e-6 (:23-:24). f64 SNR_THRESHOLD 310 (MISCF64.cpp:8), REL_ERROR / ABS_ERROR 2e-16 (:15-:16). ASSERT_CLOSE_ERROR is
 * |a-b| <= ABS + REL*|b| (dsp::closeAbsRel). Used unchanged (L4). ARM checks 4 zeros behind the output of correlate and
 * Levinson-Durbin (ASSERT_EMPTY_TAIL): a guard behind every output is checked here.
 *
 * DEF (double / long double; the ARM reference patterns are checked against it, so the conventions are the ARM ones):
 *   conv:      y[n] = sum_k a[k] b[n-k], n = 0 .. na+nb-2 (DOC arm_conv_f32 in filtering_functions.h:443-450: "Length
 *              srcALen+srcBLen-1").
 *   correlate: DOC "c[n] = sum_k a[k] b[k-n]", "length 2 * max(srcALen, srcBLen) - 1", "pDst should be initialized to all zeros
 *              before being used". With M = max(na, nb) the output index i = 0 .. 2M-2 holds the lag l = i - (M-1):
 *              out[i] = R[l] = sum_k a[k+l] b[k] (terms with k+l outside [0, na) or k outside [0, nb) are absent, so R[l] is 0
 *              outside l in [-(nb-1), na-1]: the zero padding is at the START of the output if na > nb and at the END if
 *              na < nb). Derived by comparing with the ARM patterns (checked by the "reference equals DEF" tests below).
 *              The unwritten padding entries are the documented zero initialisation: the tests zero the output first.
 *   partial conv: DOC (PartialConv): "The function computes the output in the range [firstIndex, ..., firstIndex+numPoints-1]",
 *              "The allowable range of output indices is [0 srcALen+srcBLen-2]. If the requested subset does not fall in this
 *              range then the functions return ARM_MATH_ARGUMENT_ERROR". DOC also says "The output array pDst contains
 *              numPoints values" but ARM's test (MISCF32.cpp:95) reads the window at pDst[firstIndex]: the window tests follow
 *              ARM (PAT), the "numPoints values at pDst[0..]" reading is not asserted (see the characterization test).
 *   Levinson-Durbin: phi = autocovariance, lag 0 first, nbCoefs + 1 values; a (nbCoefs values) solves the Yule-Walker system
 *              sum_j a[j] phi[|i-j|] = phi[i+1], i = 0 .. nbCoefs-1; err = phi[0] - sum_j a[j] phi[j+1] (prediction error).
 *              Hand cases: phi = {2,1}: a = {1/2}, err = 3/2. phi = {4,2,2}: [[4,2],[2,4]] a = [2,2] -> a = {1/3,1/3},
 *              err = 4 - 4/3 = 8/3. phi = {1,1/2,1/4,1/8} (AR(1), rho = 1/2): a = {1/2,0,0}, err = 3/4.
 *
 * Error bounds. Every correlate / conv output is a sum of at most T = min(na, nb) rounded products accumulated in some order:
 *   |y - ydef| <= gam(T) * S + T * sub + gam_ld(T) * S,  S = sum |a b|  (Higham, dot product; same derivation and the same
 * long double allowance as test_dsp_filt_fir_float.cpp; u = 2^-24 / 2^-53; holds fused or not). Dyadic data (multiples of 1/16)
 * is exact in both types: those tests compare exactly. Resolution: a perturbation below about 2*T*u*S per output is invisible
 * to the random tests; the exact tests are the tight ones.
 * Levinson-Durbin random tests: phi = a positive mixture of three AR(1) autocovariances (weights sum to 1, so phi[0] = 1; the
 * Toeplitz matrix R is symmetric positive definite and well conditioned). The forward error of the solution is bounded by the
 * first-order estimate  |a - adef|_inf <= n * kappa_inf(R) * u * max(1, |a|_inf)  (backward error O(n u) per Cybenko 1980,
 * times the condition number kappa computed in the test from R and its inverse in long double); the same bound is used for err
 * (phi[0] = 1). This is a conservative derived estimate, not a sharp bound; ARM's patterns (above) are the tight check.
 *
 * Host vs target (C6): the host runs the portable C path (no ARM_MATH_DSP / MVE); the target may contract a*b+c to VFMA: the
 * bounds hold for both. CONSEQUENCE: on the host (ARM_MATH_DSP undefined) arm_conv_f32 (arm_conv_f32.c:933), arm_correlate_f32
 * (:1015) and arm_conv_partial_f32 (:638) run the generic loop, NOT the 4-way unrolled kernels the Cortex-M4 target runs
 * (ARM_MATH_DSP + ARM_MATH_LOOPUNROLL). The length sweeps (1-9, 12, 13, 17) are therefore tests of the generic loop; the unroll
 * tails of the target kernels are not exercised by this file.
 * NEEDS REVIEW (measured only in a scratch build by the reviewer, with ARM_MATH_DSP set for this one file; no test added):
 * arm_conv_partial_f32 on the target path has the same stage-3 over-read defect as B-DCVQ31-2: window first = 9 (ARM's id 85)
 * returned 0.2324 against the reference -0.00268.
 * Not tested, NEEDS REVIEW: srcALen / srcBLen / numPoints / nbCoefs 0 (kernels read or write out of bounds or DOC silent);
 * phi[0] == 0 in Levinson-Durbin (division by zero, DOC silent); firstIndex + numPoints wrapping around 2^32 in the partial
 * convolution range check (unsigned overflow defeats the check, DOC silent); aliasing of pDst and the inputs.
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

struct F32 {
	typedef float S;
	static const char *name() { return "f32"; }
	static constexpr double u = 5.9604644775390625e-8;  // 2^-24
	static constexpr double sub = 7.006492321624085e-46;  // 2^-150
	static constexpr double snrThr = 120.0, absTol = 1e-5, relTol = 1e-6;  // MISCF32.cpp:8, :15-:16
	static S from(double d) { return static_cast<float>(d); }
	static double to(S v) { return static_cast<double>(v); }
	static std::string dir() { return "Filtering/MISC/MISCF32/"; }
	static std::vector<S> pat(const std::string &n) { return dsp::loadPattern(dir() + n).f32(); }
	static void corr(const S *a, uint32_t na, const S *b, uint32_t nb, S *o) { arm_correlate_f32(a, na, b, nb, o); }
};

struct F64 {
	typedef double S;
	static const char *name() { return "f64"; }
	static constexpr double u = 1.1102230246251565e-16;  // 2^-53
	static constexpr double sub = 2.4703282292062328e-324;
	static constexpr double snrThr = 310.0, absTol = 2e-16, relTol = 2e-16;  // MISCF64.cpp:8, :15-:16
	static S from(double d) { return d; }
	static double to(S v) { return v; }
	static std::string dir() { return "Filtering/MISC/MISCF64/"; }
	static std::vector<S> pat(const std::string &n) { return dsp::loadPattern(dir() + n).f64(); }
	static void corr(const S *a, uint32_t na, const S *b, uint32_t nb, S *o) { arm_correlate_f64(a, na, b, nb, o); }
};

double gam(double k, double u) { return k * u / (1.0 - k * u); }
const double kLdU = std::numeric_limits<long double>::epsilon() / 2;

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

// multiples of 1/16 in [-1, 1]: every product (1/256 multiples) and any sum of at most 17 of them is exact in f32 and f64
template <class Tr>
std::vector<typename Tr::S> dyadicVec(size_t n, uint32_t seed) {
	Rng rng(seed);
	std::vector<typename Tr::S> v(n);
	for (size_t i = 0; i < n; i++) {
		double r = rng.next();
		v[i] = Tr::from(std::floor(r * 33.0) / 16.0 - 1.0);
	}
	return v;
}

template <class Tr>
std::vector<typename Tr::S> head(const std::vector<typename Tr::S> &v, size_t n) {
	return std::vector<typename Tr::S>(v.begin(), v.begin() + static_cast<long>(n));
}

template <class Tr>
double boundFor(size_t T, double sabs) {
	return gam(static_cast<double>(T), Tr::u) * sabs + static_cast<double>(T) * Tr::sub + gam(static_cast<double>(T), kLdU) * sabs;
}

// DEF correlate (out size 2M-1) and conv (size na+nb-1); sabs[i] = sum |a b|
template <class Tr>
void defCorr(const std::vector<typename Tr::S> &a, const std::vector<typename Tr::S> &b, std::vector<double> &y, std::vector<double> &sabs) {
	const long na = static_cast<long>(a.size()), nb = static_cast<long>(b.size());
	const long M = na > nb ? na : nb;
	y.assign(static_cast<size_t>(2 * M - 1), 0.0);
	sabs.assign(y.size(), 0.0);
	for (long i = 0; i < 2 * M - 1; i++) {
		const long l = i - (M - 1);
		long double acc = 0, sa = 0;
		for (long k = 0; k < nb; k++) {
			if (k + l < 0 || k + l >= na) continue;
			const long double p = static_cast<long double>(Tr::to(a[static_cast<size_t>(k + l)])) * static_cast<long double>(Tr::to(b[static_cast<size_t>(k)]));
			acc += p;
			sa += (p < 0 ? -p : p);
		}
		y[static_cast<size_t>(i)] = static_cast<double>(acc);
		sabs[static_cast<size_t>(i)] = static_cast<double>(sa);
	}
}

template <class Tr>
void defConv(const std::vector<typename Tr::S> &a, const std::vector<typename Tr::S> &b, std::vector<double> &y, std::vector<double> &sabs) {
	const long na = static_cast<long>(a.size()), nb = static_cast<long>(b.size());
	y.assign(static_cast<size_t>(na + nb - 1), 0.0);
	sabs.assign(y.size(), 0.0);
	for (long n = 0; n < na + nb - 1; n++) {
		long double acc = 0, sa = 0;
		for (long k = 0; k < nb; k++) {
			const long m = n - k;
			if (m < 0 || m >= na) continue;
			const long double p = static_cast<long double>(Tr::to(a[static_cast<size_t>(m)])) * static_cast<long double>(Tr::to(b[static_cast<size_t>(k)]));
			acc += p;
			sa += (p < 0 ? -p : p);
		}
		y[static_cast<size_t>(n)] = static_cast<double>(acc);
		sabs[static_cast<size_t>(n)] = static_cast<double>(sa);
	}
}

// ---- runners: the output is zero filled first (documented), inputs are tight heap vectors ----
template <class Tr>
std::vector<typename Tr::S> runCorrG(const std::vector<typename Tr::S> &a, const std::vector<typename Tr::S> &b, bool &guard) {
	const size_t M = a.size() > b.size() ? a.size() : b.size();
	dsp::GuardedBuffer<typename Tr::S> g(2 * M - 1);
	std::memset(static_cast<void *>(g.data()), 0, g.size() * sizeof(typename Tr::S));
	Tr::corr(a.data(), static_cast<uint32_t>(a.size()), b.data(), static_cast<uint32_t>(b.size()), g.data());
	guard = g.guardIntact();
	return g.output();
}

template <class Tr>
std::vector<typename Tr::S> runCorrT(const std::vector<typename Tr::S> &a, const std::vector<typename Tr::S> &b) {
	const size_t M = a.size() > b.size() ? a.size() : b.size();
	std::vector<typename Tr::S> o(2 * M - 1, Tr::from(0.0));  // exact size: ASan sees any write behind it
	Tr::corr(a.data(), static_cast<uint32_t>(a.size()), b.data(), static_cast<uint32_t>(b.size()), o.data());
	return o;
}

std::vector<float> runConvG(const std::vector<float> &a, const std::vector<float> &b, bool &guard) {
	dsp::GuardedBuffer<float> g(a.size() + b.size() - 1);
	arm_conv_f32(a.data(), static_cast<uint32_t>(a.size()), b.data(), static_cast<uint32_t>(b.size()), g.data());
	guard = g.guardIntact();
	return g.output();
}

std::vector<float> runConvT(const std::vector<float> &a, const std::vector<float> &b) {
	std::vector<float> o(a.size() + b.size() - 1, 0.0f);
	arm_conv_f32(a.data(), static_cast<uint32_t>(a.size()), b.data(), static_cast<uint32_t>(b.size()), o.data());
	return o;
}

// compare with the DEF and its bound; returns the number of outputs outside
template <class Tr>
size_t outsideBound(const std::vector<typename Tr::S> &out, const std::vector<double> &y, const std::vector<double> &sabs, size_t T) {
	size_t bad = 0;
	if (out.size() != y.size()) return out.size() + 1;
	for (size_t i = 0; i < y.size(); i++) {
		const double o = Tr::to(out[i]);
		if (std::isnan(o) || !(std::fabs(o - y[i]) <= boundFor<Tr>(T, sabs[i]))) bad++;
	}
	return bad;
}

const size_t kNa[] = {4, 5, 6, 9, 10, 11, 12, 13};
const size_t kNb[] = {1, 2, 3, 8, 11};
const size_t kLens[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 13, 17};

// ---------------------------------------------------------------------------------------------------------------
// C1
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
void armCorrelateTest() {
	typedef typename Tr::S S;
	const std::string sfx = Tr::name();
	const std::vector<S> A = Tr::pat("InputsA1_" + sfx + ".txt");
	const std::vector<S> B = Tr::pat("InputsB1_" + sfx + ".txt");
	size_t id = 0;
	for (size_t na : kNa)
		for (size_t nb : kNb) {
			id++;
			const std::vector<S> ref = Tr::pat("Reference" + std::to_string(id) + "_" + sfx + ".txt");
			const std::vector<S> a = head<Tr>(A, na), b = head<Tr>(B, nb);
			bool guard = false;
			const std::vector<S> out = runCorrG<Tr>(a, b, guard);
			CHECK_MESSAGE(guard, "guard damaged, nba " << na << " nbb " << nb);
			REQUIRE(ref.size() == out.size());
			CHECK_MESSAGE(dsp::snr(ref, out) >= Tr::snrThr, "SNR " << dsp::snr(ref, out) << " nba " << na << " nbb " << nb);
			auto c = dsp::closeAbsRel(ref, out, Tr::absTol, Tr::relTol);
			CHECK_MESSAGE(c, "nba " << na << " nbb " << nb << ": " << c.message);
		}
	CHECK(id == 40);
}

void armConvTest() {
	const std::vector<float> A = F32::pat("InputsA1_f32.txt"), B = F32::pat("InputsB1_f32.txt");
	size_t id = 40;
	for (size_t na : kNa)
		for (size_t nb : kNb) {
			id++;
			const std::vector<float> ref = F32::pat("Reference" + std::to_string(id) + "_f32.txt");
			const std::vector<float> a = head<F32>(A, na), b = head<F32>(B, nb);
			bool guard = false;
			const std::vector<float> out = runConvG(a, b, guard);
			CHECK_MESSAGE(guard, "guard damaged, nba " << na << " nbb " << nb);
			REQUIRE(ref.size() == out.size());
			CHECK_MESSAGE(dsp::snr(ref, out) >= F32::snrThr, "SNR " << dsp::snr(ref, out) << " nba " << na << " nbb " << nb);
			auto c = dsp::closeAbsRel(ref, out, F32::absTol, F32::relTol);
			CHECK_MESSAGE(c, "nba " << na << " nbb " << nb << ": " << c.message);
		}
	CHECK(id == 80);
}

// ARM reference patterns against the DEF (convention check). The script rounds a double result to the type: u*|y| plus
// the DEF's own long double error.
template <class Tr>
void corrPatternIsDef() {
	typedef typename Tr::S S;
	const std::string sfx = Tr::name();
	const std::vector<S> A = Tr::pat("InputsA1_" + sfx + ".txt");
	const std::vector<S> B = Tr::pat("InputsB1_" + sfx + ".txt");
	size_t id = 0, bad = 0;
	for (size_t na : kNa)
		for (size_t nb : kNb) {
			id++;
			const std::vector<S> ref = Tr::pat("Reference" + std::to_string(id) + "_" + sfx + ".txt");
			std::vector<double> y, sa;
			defCorr<Tr>(head<Tr>(A, na), head<Tr>(B, nb), y, sa);
			REQUIRE(ref.size() == y.size());
			const size_t T = na < nb ? na : nb;
			for (size_t i = 0; i < y.size(); i++) {
				// ref = round_type(double sum of the unrounded-data products): u*|y| + 2*u*S (inputs rounded to the type)
				const double b = Tr::u * std::fabs(y[i]) + 2.0 * Tr::u * sa[i] + 2.0 * gam(static_cast<double>(T), 1.1102230246251565e-16) * sa[i];
				if (!(std::fabs(Tr::to(ref[i]) - y[i]) <= b)) bad++;
			}
		}
	CHECK_MESSAGE(bad == 0, bad << " reference samples differ from the DEF");
}

void convPatternIsDef() {
	const std::vector<float> A = F32::pat("InputsA1_f32.txt"), B = F32::pat("InputsB1_f32.txt");
	size_t id = 40, bad = 0;
	for (size_t na : kNa)
		for (size_t nb : kNb) {
			id++;
			const std::vector<float> ref = F32::pat("Reference" + std::to_string(id) + "_f32.txt");
			std::vector<double> y, sa;
			defConv<F32>(head<F32>(A, na), head<F32>(B, nb), y, sa);
			REQUIRE(ref.size() == y.size());
			const size_t T = na < nb ? na : nb;
			for (size_t i = 0; i < y.size(); i++) {
				const double b = F32::u * std::fabs(y[i]) + 2.0 * F32::u * sa[i] + 2.0 * gam(static_cast<double>(T), 1.1102230246251565e-16) * sa[i];
				if (!(std::fabs(F32::to(ref[i]) - y[i]) <= b)) bad++;
			}
		}
	CHECK_MESSAGE(bad == 0, bad << " reference samples differ from the DEF");
}

const size_t kPartFirst[] = {3, 9, 7};

void armPartialTest() {
	const std::vector<float> A = F32::pat("InputsA2_f32.txt"), B = F32::pat("InputsB2_f32.txt");
	for (size_t t = 0; t < 3; t++) {
		const size_t first = kPartFirst[t], num = 4;
		const std::vector<float> ref = F32::pat("Reference" + std::to_string(84 + t) + "_f32.txt");
		const std::vector<float> a = head<F32>(A, 6), b = head<F32>(B, 8);
		REQUIRE(ref.size() == num);
		dsp::GuardedBuffer<float> g(first + num);
		const arm_status st = arm_conv_partial_f32(a.data(), 6, b.data(), 8, g.data(), static_cast<uint32_t>(first), static_cast<uint32_t>(num));
		CHECK(st == ARM_MATH_SUCCESS);
		CHECK(g.guardIntact());
		const std::vector<float> o = g.output();
		const std::vector<float> win(o.begin() + static_cast<long>(first), o.end());
		CHECK_MESSAGE(dsp::snr(ref, win) >= F32::snrThr, "SNR " << dsp::snr(ref, win) << " first " << first);
		auto c = dsp::closeAbsRel(ref, win, F32::absTol, F32::relTol);
		CHECK_MESSAGE(c, "first " << first << ": " << c.message);
	}
}

// ---------------------------------------------------------------------------------------------------------------
// Levinson-Durbin helpers
// ---------------------------------------------------------------------------------------------------------------
// Solves R x = r for the symmetric Toeplitz R[i][j] = phi[|i-j|] in long double (Gaussian elimination, partial pivoting).
// Also returns kappa_inf(R) = |R|_inf * |R^-1|_inf.
void defLevinson(const std::vector<double> &phi, size_t n, std::vector<double> &a, double &err, double &kappa) {
	typedef long double LD;
	std::vector<std::vector<LD> > M(n, std::vector<LD>(2 * n + 1, 0.0L));
	for (size_t i = 0; i < n; i++) {
		for (size_t j = 0; j < n; j++) M[i][j] = phi[i > j ? i - j : j - i];
		M[i][n + i] = 1.0L;  // identity: gives the inverse
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

// phi[l] = sum_m w_m r_m^l (three AR(1) autocovariances, w sums to 1)
std::vector<double> mixPhi(size_t n, uint32_t seed) {
	Rng rng(seed);
	double r[3], w[3], ws = 0;
	for (int m = 0; m < 3; m++) {
		double x = rng.next();
		r[m] = 1.6 * x - 0.8;  // (-0.8, 0.8)
		double y = rng.next();
		w[m] = 0.2 + y;
		ws += w[m];
	}
	std::vector<double> phi(n + 1, 0.0);
	for (size_t l = 0; l <= n; l++)
		for (int m = 0; m < 3; m++) phi[l] += (w[m] / ws) * std::pow(r[m], static_cast<double>(l));
	return phi;
}

struct LdRun {
	std::vector<float> a;
	float err;
	bool guard;
};

LdRun runLd(const std::vector<float> &phi, size_t n) {
	LdRun r;
	dsp::GuardedBuffer<float> g(n);
	r.err = -12345.0f;
	arm_levinson_durbin_f32(phi.data(), g.data(), &r.err, static_cast<int>(n));
	r.guard = g.guardIntact();
	r.a = g.output();
	return r;
}

void armLevinsonTest() {
	const size_t ns[] = {3, 8, 11};
	const char *phiName[] = {"InputPhi81_f32.txt", "InputPhi82_f32.txt", "InputPhi83_f32.txt"};
	const std::vector<float> errs = F32::pat("LDErrors81_f32.txt");
	for (size_t t = 0; t < 3; t++) {
		const std::vector<float> phi = F32::pat(phiName[t]);
		const std::vector<float> ref = F32::pat("Reference" + std::to_string(81 + t) + "_f32.txt");
		REQUIRE(phi.size() == ns[t] + 1);
		REQUIRE(ref.size() == ns[t]);
		REQUIRE(errs.size() > t);
		const LdRun r = runLd(phi, ns[t]);
		CHECK_MESSAGE(r.guard, "guard damaged, n " << ns[t]);
		CHECK_MESSAGE(dsp::snr(ref, r.a) >= 120.0, "SNR " << dsp::snr(ref, r.a) << " n " << ns[t]);  // MISCF32.cpp:8
		auto c = dsp::closeAbsRel(ref, r.a, 1e-6, 1e-6);  // ABS_LD_ERROR, REL_LD_ERROR (:23-:24)
		CHECK_MESSAGE(c, "n " << ns[t] << ": " << c.message);
		auto ce = dsp::closeAbsRel(std::vector<float>(1, errs[t]), std::vector<float>(1, r.err), 1e-6, 1e-6);
		CHECK_MESSAGE(ce, "err, n " << ns[t] << ": " << ce.message);
	}
}

void ldPatternIsDef() {
	const size_t ns[] = {3, 8, 11};
	const char *phiName[] = {"InputPhi81_f32.txt", "InputPhi82_f32.txt", "InputPhi83_f32.txt"};
	const std::vector<float> errs = F32::pat("LDErrors81_f32.txt");
	for (size_t t = 0; t < 3; t++) {
		const std::vector<float> phi = F32::pat(phiName[t]);
		const std::vector<float> ref = F32::pat("Reference" + std::to_string(81 + t) + "_f32.txt");
		std::vector<double> p(phi.begin(), phi.end()), a;
		double e = 0, kappa = 0;
		defLevinson(p, ns[t], a, e, kappa);
		// the patterns come from a double computation rounded to f32 (u|a|); the data itself is rounded: kappa * u
		double amax = 1;
		for (double v : a) amax = std::fabs(v) > amax ? std::fabs(v) : amax;
		const double bnd = F32::u * amax + ns[t] * kappa * F32::u * amax;
		for (size_t i = 0; i < ns[t]; i++) CHECK_MESSAGE(std::fabs(static_cast<double>(ref[i]) - a[i]) <= bnd, "n " << ns[t] << " coefficient " << i);
		CHECK_MESSAGE(std::fabs(static_cast<double>(errs[t]) - e) <= bnd, "n " << ns[t] << " err");
	}
}

// ---------------------------------------------------------------------------------------------------------------
// C3 / C4 correlate
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
void corrHandTest() {
	typedef typename Tr::S S;
	// a = {1,2,3}, b = {0,1}: M = 3, R[l] = sum_k a[k+l] b[k] = a[1+l] -> R[-1]=1, R[0]=2, R[1]=3; out[i] = R[i-2]:
	// {R[-2], R[-1], R[0], R[1], R[2]} = {0, 1, 2, 3, 0} (padding at the end, na > nb)
	{
		const std::vector<S> a = {Tr::from(1), Tr::from(2), Tr::from(3)}, b = {Tr::from(0), Tr::from(1)};
		bool guard;
		const std::vector<S> out = runCorrG<Tr>(a, b, guard);
		const std::vector<S> exp = {Tr::from(0), Tr::from(1), Tr::from(2), Tr::from(3), Tr::from(0)};
		auto c = dsp::exactEqual(exp, out);
		CHECK_MESSAGE(c, c.message);
		CHECK(guard);
	}
	// a = {1,2}, b = {1,10,100}: M = 3, R[l] = sum_k a[k+l] b[k]; lags -2..1: R[-2] = a[0]b[2] = 100, R[-1] = a[0]b[1] + a[1]b[2]
	// = 10 + 200 = 210, R[0] = a[0]b[0] + a[1]b[1] = 1 + 20 = 21, R[1] = a[1]b[0] = 2, R[2] = 0; out[i] = R[i-2] =
	// {100, 210, 21, 2, 0}? i=0 -> l=-2: 100; i=1 -> 210; i=2 -> 21; i=3 -> 2; i=4 -> 0. (na < nb: padding at the end)
	{
		const std::vector<S> a = {Tr::from(1), Tr::from(2)}, b = {Tr::from(1), Tr::from(10), Tr::from(100)};
		bool guard;
		const std::vector<S> out = runCorrG<Tr>(a, b, guard);
		const std::vector<S> exp = {Tr::from(100), Tr::from(210), Tr::from(21), Tr::from(2), Tr::from(0)};
		auto c = dsp::exactEqual(exp, out);
		CHECK_MESSAGE(c, c.message);
		CHECK(guard);
	}
	// na > nb: a = {1,10,100}, b = {1,2}: R[-1] = a[0]b[1] = 2, R[0] = a[0]b[0] + a[1]b[1] = 1 + 20 = 21, R[1] = a[1]b[0] +
	// a[2]b[1] = 10 + 200 = 210, R[2] = a[2]b[0] = 100; out[i] = R[i-2], i = 0 is lag -2 (outside: 0): {0, 2, 21, 210, 100}
	{
		const std::vector<S> a = {Tr::from(1), Tr::from(10), Tr::from(100)}, b = {Tr::from(1), Tr::from(2)};
		bool guard;
		const std::vector<S> out = runCorrG<Tr>(a, b, guard);
		const std::vector<S> exp = {Tr::from(0), Tr::from(2), Tr::from(21), Tr::from(210), Tr::from(100)};
		auto c = dsp::exactEqual(exp, out);
		CHECK_MESSAGE(c, c.message);
		CHECK(guard);
	}
}

template <class Tr>
void corrRandomTest() {
	size_t bad = 0, guardBad = 0, tightDiff = 0, runs = 0;
	for (size_t na : kLens)
		for (size_t nb : kLens) {
			const std::vector<typename Tr::S> a = randVec<Tr>(na, static_cast<uint32_t>(100 + na), -1.0, 1.0);
			const std::vector<typename Tr::S> b = randVec<Tr>(nb, static_cast<uint32_t>(200 + nb), -1.0, 1.0);
			std::vector<double> y, sa;
			defCorr<Tr>(a, b, y, sa);
			bool guard = false;
			const std::vector<typename Tr::S> o = runCorrG<Tr>(a, b, guard);
			const std::vector<typename Tr::S> t = runCorrT<Tr>(a, b);
			if (!guard) guardBad++;
			if (std::memcmp(o.data(), t.data(), o.size() * sizeof(typename Tr::S)) != 0) tightDiff++;  // same call, same data
			bad += outsideBound<Tr>(o, y, sa, na < nb ? na : nb);
			runs++;
		}
	CHECK(runs == 144);
	CHECK_MESSAGE(bad == 0, bad << " outputs outside the bound (lengths 1-9, 12, 13, 17 both orders)");
	CHECK(guardBad == 0);
	CHECK(tightDiff == 0);
}

template <class Tr>
void corrDyadicExactTest() {
	size_t bad = 0;
	for (size_t na : kLens)
		for (size_t nb : kLens) {
			const std::vector<typename Tr::S> a = dyadicVec<Tr>(na, static_cast<uint32_t>(300 + na));
			const std::vector<typename Tr::S> b = dyadicVec<Tr>(nb, static_cast<uint32_t>(400 + nb));
			std::vector<double> y, sa;
			defCorr<Tr>(a, b, y, sa);
			const std::vector<typename Tr::S> o = runCorrT<Tr>(a, b);
			for (size_t i = 0; i < y.size(); i++)
				if (!(Tr::to(o[i]) == y[i])) bad++;
		}
	CHECK_MESSAGE(bad == 0, bad << " outputs differ from the exact value");
}

// correlate(b, a) is correlate(a, b) reversed (R_ba[l] = R_ab[-l]; out_ba[i] = R_ab[(M-1)-i] = out_ab[2M-2-i]); exact data
template <class Tr>
void corrSwapTest() {
	size_t bad = 0;
	for (size_t na : kLens)
		for (size_t nb : kLens) {
			const std::vector<typename Tr::S> a = dyadicVec<Tr>(na, static_cast<uint32_t>(500 + na));
			const std::vector<typename Tr::S> b = dyadicVec<Tr>(nb, static_cast<uint32_t>(600 + nb));
			const std::vector<typename Tr::S> ab = runCorrT<Tr>(a, b), ba = runCorrT<Tr>(b, a);
			for (size_t i = 0; i < ab.size(); i++)
				if (!(ab[i] == ba[ab.size() - 1 - i])) bad++;
		}
	CHECK_MESSAGE(bad == 0, bad << " outputs differ");
}

// autocorrelation: out is symmetric about the centre, centre = sum a^2 (exact data)
template <class Tr>
void corrAutoTest() {
	for (size_t n : kLens) {
		const std::vector<typename Tr::S> a = dyadicVec<Tr>(n, static_cast<uint32_t>(700 + n));
		const std::vector<typename Tr::S> o = runCorrT<Tr>(a, a);
		REQUIRE(o.size() == 2 * n - 1);
		long double ss = 0;
		for (size_t i = 0; i < n; i++) ss += static_cast<long double>(Tr::to(a[i])) * Tr::to(a[i]);
		CHECK_MESSAGE(Tr::to(o[n - 1]) == static_cast<double>(ss), "n " << n << " centre");
		bool sym = true;
		for (size_t i = 0; i < o.size(); i++) sym = sym && (o[i] == o[o.size() - 1 - i]);
		CHECK_MESSAGE(sym, "n " << n << " symmetric");
	}
}

template <class Tr>
void corrConstInputsTest() {
	const std::vector<typename Tr::S> a = randVec<Tr>(9, 801, -1.0, 1.0), b = randVec<Tr>(5, 802, -1.0, 1.0);
	const std::vector<typename Tr::S> a0 = a, b0 = b;
	(void)runCorrT<Tr>(a, b);
	CHECK(std::memcmp(a.data(), a0.data(), a.size() * sizeof(typename Tr::S)) == 0);
	CHECK(std::memcmp(b.data(), b0.data(), b.size() * sizeof(typename Tr::S)) == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// conv
// ---------------------------------------------------------------------------------------------------------------
void convHandTest() {
	// {1,2,3} * {0,1} = {0, 1, 2, 3}; {1,1,1} * {1,1,1} = {1,2,3,2,1}; {1,2} * {1,10,100} = {1, 12, 120, 200}
	struct Case { std::vector<float> a, b, e; };
	const Case cs[] = {{{1, 2, 3}, {0, 1}, {0, 1, 2, 3}}, {{1, 1, 1}, {1, 1, 1}, {1, 2, 3, 2, 1}}, {{1, 2}, {1, 10, 100}, {1, 12, 120, 200}}};
	for (const Case &c : cs) {
		bool guard = false;
		const std::vector<float> o = runConvG(c.a, c.b, guard);
		auto r = dsp::exactEqual(c.e, o);
		CHECK_MESSAGE(r, r.message);
		CHECK(guard);
	}
}

void convRandomTest() {
	size_t bad = 0, guardBad = 0, tightDiff = 0, runs = 0;
	for (size_t na : kLens)
		for (size_t nb : kLens) {
			const std::vector<float> a = randVec<F32>(na, static_cast<uint32_t>(1100 + na), -1.0, 1.0);
			const std::vector<float> b = randVec<F32>(nb, static_cast<uint32_t>(1200 + nb), -1.0, 1.0);
			std::vector<double> y, sa;
			defConv<F32>(a, b, y, sa);
			bool guard = false;
			const std::vector<float> o = runConvG(a, b, guard);
			const std::vector<float> t = runConvT(a, b);
			if (!guard) guardBad++;
			if (std::memcmp(o.data(), t.data(), o.size() * sizeof(float)) != 0) tightDiff++;
			bad += outsideBound<F32>(o, y, sa, na < nb ? na : nb);
			runs++;
		}
	CHECK(runs == 144);
	CHECK_MESSAGE(bad == 0, bad << " outputs outside the bound (lengths 1-9, 12, 13, 17 both orders)");
	CHECK(guardBad == 0);
	CHECK(tightDiff == 0);
}

void convDyadicExactTest() {
	size_t bad = 0, comm = 0;
	for (size_t na : kLens)
		for (size_t nb : kLens) {
			const std::vector<float> a = dyadicVec<F32>(na, static_cast<uint32_t>(1300 + na));
			const std::vector<float> b = dyadicVec<F32>(nb, static_cast<uint32_t>(1400 + nb));
			std::vector<double> y, sa;
			defConv<F32>(a, b, y, sa);
			const std::vector<float> ab = runConvT(a, b), ba = runConvT(b, a);
			for (size_t i = 0; i < y.size(); i++) {
				if (!(static_cast<double>(ab[i]) == y[i])) bad++;
				if (!(ab[i] == ba[i])) comm++;
			}
		}
	CHECK_MESSAGE(bad == 0, bad << " outputs differ from the exact value");
	CHECK_MESSAGE(comm == 0, comm << " outputs differ between conv(a,b) and conv(b,a)");
}

// delta at position p: out[p + j] = b[j], everything else 0 (exact)
void convDeltaTest() {
	for (size_t nb : kLens)
		for (size_t na : {1u, 2u, 5u, 9u}) {
			for (size_t p = 0; p < na; p++) {
				std::vector<float> a(na, 0.0f);
				a[p] = 1.0f;
				const std::vector<float> b = dyadicVec<F32>(nb, static_cast<uint32_t>(1500 + nb));
				const std::vector<float> o = runConvT(a, b);
				std::vector<float> e(na + nb - 1, 0.0f);
				for (size_t j = 0; j < nb; j++) e[p + j] = b[j];
				auto c = dsp::exactEqual(e, o);
				CHECK_MESSAGE(c, "na " << na << " p " << p << " nb " << nb << ": " << c.message);
			}
		}
}

void convConstInputsTest() {
	const std::vector<float> a = randVec<F32>(9, 1601, -1.0, 1.0), b = randVec<F32>(5, 1602, -1.0, 1.0);
	const std::vector<float> a0 = a, b0 = b;
	(void)runConvT(a, b);
	CHECK(std::memcmp(a.data(), a0.data(), a.size() * sizeof(float)) == 0);
	CHECK(std::memcmp(b.data(), b0.data(), b.size() * sizeof(float)) == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// partial conv
// ---------------------------------------------------------------------------------------------------------------
void partialAllWindowsTest() {
	size_t bad = 0, guardBad = 0, statusBad = 0, windows = 0, tightBad = 0;
	for (size_t na = 1; na <= 9; na++)
		for (size_t nb = 1; nb <= 9; nb++) {
			const std::vector<float> a = randVec<F32>(na, static_cast<uint32_t>(1700 + na), -1.0, 1.0);
			const std::vector<float> b = randVec<F32>(nb, static_cast<uint32_t>(1800 + nb), -1.0, 1.0);
			std::vector<double> y, sa;
			defConv<F32>(a, b, y, sa);
			const size_t L = na + nb - 1, T = na < nb ? na : nb;
			for (size_t first = 0; first < L; first++)
				for (size_t num = 1; first + num <= L; num++) {
					dsp::GuardedBuffer<float> g(first + num);  // the window lives at pDst[first ..]
					const arm_status st = arm_conv_partial_f32(a.data(), static_cast<uint32_t>(na), b.data(), static_cast<uint32_t>(nb), g.data(),
					                                            static_cast<uint32_t>(first), static_cast<uint32_t>(num));
					if (st != ARM_MATH_SUCCESS) statusBad++;
					if (!g.guardIntact()) guardBad++;
					const std::vector<float> o = g.output();
					for (size_t i = 0; i < num; i++) {
						const double v = static_cast<double>(o[first + i]);
						if (std::isnan(v) || !(std::fabs(v - y[first + i]) <= boundFor<F32>(T, sa[first + i]))) bad++;
					}
					// the same call on an exact size heap output (first + num elements, no slack: ASan sees any write behind it)
					std::vector<float> tight(first + num, 0.0f);
					const arm_status st2 = arm_conv_partial_f32(a.data(), static_cast<uint32_t>(na), b.data(), static_cast<uint32_t>(nb), tight.data(),
					                                             static_cast<uint32_t>(first), static_cast<uint32_t>(num));
					if (st2 != ARM_MATH_SUCCESS) statusBad++;
					for (size_t i = 0; i < num; i++)
						if (std::memcmp(&tight[first + i], &o[first + i], sizeof(float)) != 0) tightBad++;  // same call, same data
					windows++;
				}
		}
	CHECK(tightBad == 0);
	CHECK(windows == 4185);  // sum over na, nb = 1..9 of L(L+1)/2, L = na+nb-1 (number of windows with first+num <= L, num >= 1)
	CHECK_MESSAGE(statusBad == 0, statusBad << " windows returned an error");
	CHECK_MESSAGE(bad == 0, bad << " window samples outside the bound (na, nb 1-9, every window)");
	CHECK(guardBad == 0);
}

void partialRangeErrorTest() {
	// DOC: "If the requested subset does not fall in this range then the functions return ARM_MATH_ARGUMENT_ERROR"
	// range [0, na+nb-2]: first + num <= L = na + nb - 1 is allowed, L + 1 is not
	size_t bad = 0;
	for (size_t na = 1; na <= 9; na++)
		for (size_t nb = 1; nb <= 9; nb++) {
			const std::vector<float> a(na, 0.5f), b(nb, 0.25f);
			const size_t L = na + nb - 1;
			std::vector<float> out(L + 4, 0.0f);
			arm_status st = arm_conv_partial_f32(a.data(), static_cast<uint32_t>(na), b.data(), static_cast<uint32_t>(nb), out.data(), 0, static_cast<uint32_t>(L));
			if (st != ARM_MATH_SUCCESS) bad++;
			st = arm_conv_partial_f32(a.data(), static_cast<uint32_t>(na), b.data(), static_cast<uint32_t>(nb), out.data(), 0, static_cast<uint32_t>(L + 1));
			if (st != ARM_MATH_ARGUMENT_ERROR) bad++;
			st = arm_conv_partial_f32(a.data(), static_cast<uint32_t>(na), b.data(), static_cast<uint32_t>(nb), out.data(), static_cast<uint32_t>(L - 1), 2);
			if (st != ARM_MATH_ARGUMENT_ERROR) bad++;
			st = arm_conv_partial_f32(a.data(), static_cast<uint32_t>(na), b.data(), static_cast<uint32_t>(nb), out.data(), static_cast<uint32_t>(L), 1);
			if (st != ARM_MATH_ARGUMENT_ERROR) bad++;
			st = arm_conv_partial_f32(a.data(), static_cast<uint32_t>(na), b.data(), static_cast<uint32_t>(nb), out.data(), static_cast<uint32_t>(L - 1), 1);
			if (st != ARM_MATH_SUCCESS) bad++;
		}
	CHECK_MESSAGE(bad == 0, bad << " wrong status codes");
}

// characterization: DOC says "pDst contains numPoints values" and "[firstIndex ...]": the code writes at pDst[firstIndex] and
// leaves pDst[0 .. firstIndex-1] untouched; on ARGUMENT_ERROR it writes nothing. (NEEDS REVIEW: DOC wording)
void partialPlacementCharacterization() {
	const std::vector<float> a = {1, 2, 3, 4, 5, 6}, b = {1, 1, 1};
	std::vector<float> out(10, -777.0f);
	const arm_status st = arm_conv_partial_f32(a.data(), 6, b.data(), 3, out.data(), 3, 2);
	CHECK(st == ARM_MATH_SUCCESS);
	// full conv = {1,3,6,9,12,15,11,6}: indices 3, 4 = 9, 12 (exact integers)
	CHECK(out[3] == 9.0f);
	CHECK(out[4] == 12.0f);
	for (size_t i = 0; i < 10; i++)
		if (i != 3 && i != 4) CHECK_MESSAGE(out[i] == -777.0f, "index " << i << " was written");
	std::vector<float> e(10, -777.0f);
	CHECK(arm_conv_partial_f32(a.data(), 6, b.data(), 3, e.data(), 7, 2) == ARM_MATH_ARGUMENT_ERROR);
	for (float v : e) CHECK(v == -777.0f);
}

// ---------------------------------------------------------------------------------------------------------------
// Levinson-Durbin
// ---------------------------------------------------------------------------------------------------------------
void ldHandTest() {
	{  // phi = {2,1}: a = {1/2}, err = 3/2 (exact)
		const LdRun r = runLd({2.0f, 1.0f}, 1);
		CHECK(r.guard);
		CHECK(r.a[0] == 0.5f);
		CHECK(r.err == 1.5f);
	}
	{  // phi = {4,2,2}: a = {1/3, 1/3}, err = 8/3. Operations: k = (2 - 1)/(4 - 1) (1 rounding), a0 = 0.5 - k*0.5 (3 roundings,
	   // the subtraction loses at most a factor 1.5), e = 3*(1 - k*k): about 8 roundings at most: 8 u relative, with u = 2^-24
		const LdRun r = runLd({4.0f, 2.0f, 2.0f}, 2);
		CHECK(r.guard);
		const double tol = 8.0 * F32::u;
		CHECK(std::fabs(static_cast<double>(r.a[0]) - 1.0 / 3.0) <= tol / 3.0);
		CHECK(std::fabs(static_cast<double>(r.a[1]) - 1.0 / 3.0) <= tol / 3.0);
		CHECK(std::fabs(static_cast<double>(r.err) - 8.0 / 3.0) <= tol * 8.0 / 3.0);
	}
	{  // phi = {1, 1/2, 1/4, 1/8}: a = {1/2, 0, 0}, err = 3/4; every intermediate is dyadic, so exact
		const LdRun r = runLd({1.0f, 0.5f, 0.25f, 0.125f}, 3);
		CHECK(r.guard);
		CHECK(r.a[0] == 0.5f);
		CHECK(r.a[1] == 0.0f);
		CHECK(r.a[2] == 0.0f);
		CHECK(r.err == 0.75f);
	}
}

void ldRandomTest() {
	const size_t ns[] = {1, 2, 3, 4, 5, 6, 8, 11};
	size_t bad = 0, guardBad = 0, runs = 0;
	for (size_t n : ns)
		for (uint32_t seed = 1; seed <= 6; seed++) {
			const std::vector<double> p = mixPhi(n, seed * 31 + static_cast<uint32_t>(n));
			std::vector<float> phi(p.begin(), p.end());
			std::vector<double> pr(phi.begin(), phi.end()), a;  // the DEF sees the rounded phi
			double e = 0, kappa = 0;
			defLevinson(pr, n, a, e, kappa);
			double amax = 1;
			for (double v : a) amax = std::fabs(v) > amax ? std::fabs(v) : amax;
			const double bnd = static_cast<double>(n) * kappa * F32::u * amax;
			const LdRun r = runLd(phi, n);
			if (!r.guard) guardBad++;
			for (size_t i = 0; i < n; i++)
				if (!(std::fabs(static_cast<double>(r.a[i]) - a[i]) <= bnd)) bad++;
			if (!(std::fabs(static_cast<double>(r.err) - e) <= bnd * pr[0])) bad++;
			runs++;
		}
	CHECK(runs == 48);
	CHECK_MESSAGE(bad == 0, bad << " values outside the bound");
	CHECK(guardBad == 0);
}

// exact-size output (ASan sees an overrun of a tight heap allocation)
void ldTightTest() {
	for (size_t n : {1u, 2u, 3u, 4u, 5u, 7u, 8u, 9u}) {
		const std::vector<double> p = mixPhi(n, 90 + static_cast<uint32_t>(n));
		const std::vector<float> phi(p.begin(), p.end());  // exactly n + 1 values
		std::vector<float> a(n, 0.0f);
		float err = 0;
		arm_levinson_durbin_f32(phi.data(), a.data(), &err, static_cast<int>(n));
		const LdRun r = runLd(phi, n);
		CHECK(std::memcmp(a.data(), r.a.data(), n * sizeof(float)) == 0);
		CHECK(err == r.err);
	}
}

// the Yule-Walker residual of the result is small (property check independent of the DEF solver)
void ldResidualTest() {
	for (size_t n : {3u, 8u, 11u}) {
		const std::vector<double> p = mixPhi(n, 7 + static_cast<uint32_t>(n));
		const std::vector<float> phi(p.begin(), p.end());
		const LdRun r = runLd(phi, n);
		// Loose secondary check, bound derived from the Levinson error bound of the header: residual_i = (R a - rhs)_i =
		// (R (a - a_exact))_i, |residual|_inf <= |R|_inf * |a - a_exact|_inf <= n * (n * kappa * u * max(1, |a|_inf)) because
		// |phi[l]| <= phi[0] = 1 gives |R|_inf <= n (the products and the sum are evaluated in long double).
		std::vector<double> pr(phi.begin(), phi.end()), aDef;
		double eDef = 0, kappa = 0;
		defLevinson(pr, n, aDef, eDef, kappa);
		double amax = 1;
		for (double v : aDef) amax = std::fabs(v) > amax ? std::fabs(v) : amax;
		const double resBound = static_cast<double>(n) * static_cast<double>(n) * kappa * F32::u * amax;
		for (size_t i = 0; i < n; i++) {
			long double s = 0;
			for (size_t j = 0; j < n; j++) s += static_cast<long double>(r.a[j]) * phi[i > j ? i - j : j - i];
			CHECK_MESSAGE(std::fabs(static_cast<double>(s - phi[i + 1])) <= resBound, "n " << n << " row " << i);
		}
	}
}

// C5: f32 correlate agrees with f64 correlate on the same (f32 exact) data
void corrF32VsF64Test() {
	size_t bad = 0;
	for (size_t na : kLens)
		for (size_t nb : kLens) {
			const std::vector<float> a = randVec<F32>(na, static_cast<uint32_t>(900 + na), -1.0, 1.0);
			const std::vector<float> b = randVec<F32>(nb, static_cast<uint32_t>(950 + nb), -1.0, 1.0);
			const std::vector<double> a64(a.begin(), a.end()), b64(b.begin(), b.end());
			std::vector<double> y, sa;
			defCorr<F64>(a64, b64, y, sa);
			const std::vector<float> o32 = runCorrT<F32>(a, b);
			const std::vector<double> o64 = runCorrT<F64>(a64, b64);
			const size_t T = na < nb ? na : nb;
			for (size_t i = 0; i < y.size(); i++) {
				// |f32 - f64| <= f32 bound + f64 bound
				const double bnd = boundFor<F32>(T, sa[i]) + boundFor<F64>(T, sa[i]);
				if (!(std::fabs(static_cast<double>(o32[i]) - o64[i]) <= bnd)) bad++;
			}
		}
	CHECK_MESSAGE(bad == 0, bad << " samples differ by more than the sum of both bounds");
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_correlate_f32] ARM's 40 pattern cases reach SNR 120, abs 1e-5 and rel 1e-6 with a zero guard behind the output") { armCorrelateTest<F32>(); }
TEST_CASE("[C3 arm_correlate_f32] ARM's 40 reference patterns equal the DEF (output index i is lag i-(M-1))") { corrPatternIsDef<F32>(); }
TEST_CASE("[C3 arm_correlate_f32] hand derived cases for na > nb and na < nb, padding zero at the start / end") { corrHandTest<F32>(); }
TEST_CASE("[C3 arm_correlate_f32] random data, all length pairs of 1-9, 12, 13, 17 are within the bound of the DEF, guard intact") { corrRandomTest<F32>(); }
TEST_CASE("[C3 arm_correlate_f32] dyadic data, all length pairs of 1-9, 12, 13, 17 equal the exact result") { corrDyadicExactTest<F32>(); }
TEST_CASE("[C4 arm_correlate_f32] swapping the arguments reverses the output, for all length pairs of 1-9, 12, 13, 17") { corrSwapTest<F32>(); }
TEST_CASE("[C3 arm_correlate_f32] the autocorrelation is symmetric and its centre is the energy, for lengths 1-9, 12, 13, 17") { corrAutoTest<F32>(); }
TEST_CASE("[C4 arm_correlate_f32] the input arrays are not modified") { corrConstInputsTest<F32>(); }
TEST_CASE("[C5 arm_correlate_f32] the f32 result agrees with arm_correlate_f64 within the sum of both bounds") { corrF32VsF64Test(); }

TEST_CASE("[C1 arm_correlate_f64] ARM's 40 pattern cases reach SNR 310, abs 2e-16 and rel 2e-16") { armCorrelateTest<F64>(); }
TEST_CASE("[C3 arm_correlate_f64] ARM's 40 reference patterns equal the DEF (output index i is lag i-(M-1))") { corrPatternIsDef<F64>(); }
TEST_CASE("[C3 arm_correlate_f64] hand derived cases for na > nb and na < nb, padding zero at the start / end") { corrHandTest<F64>(); }
TEST_CASE("[C3 arm_correlate_f64] random data, all length pairs of 1-9, 12, 13, 17 are within the bound of the DEF, guard intact") { corrRandomTest<F64>(); }
TEST_CASE("[C3 arm_correlate_f64] dyadic data, all length pairs of 1-9, 12, 13, 17 equal the exact result") { corrDyadicExactTest<F64>(); }
TEST_CASE("[C4 arm_correlate_f64] swapping the arguments reverses the output, for all length pairs of 1-9, 12, 13, 17") { corrSwapTest<F64>(); }
TEST_CASE("[C3 arm_correlate_f64] the autocorrelation is symmetric and its centre is the energy, for lengths 1-9, 12, 13, 17") { corrAutoTest<F64>(); }
TEST_CASE("[C4 arm_correlate_f64] the input arrays are not modified") { corrConstInputsTest<F64>(); }

TEST_CASE("[C1 arm_conv_f32] ARM's 40 pattern cases reach SNR 120, abs 1e-5 and rel 1e-6") { armConvTest(); }
TEST_CASE("[C3 arm_conv_f32] ARM's 40 reference patterns equal the DEF y[n] = sum a[k] b[n-k]") { convPatternIsDef(); }
TEST_CASE("[C3 arm_conv_f32] hand derived products of small integer sequences") { convHandTest(); }
TEST_CASE("[C3 arm_conv_f32] random data, all length pairs of 1-9, 12, 13, 17 are within the bound of the DEF, guard intact") { convRandomTest(); }
TEST_CASE("[C3 arm_conv_f32] dyadic data equals the exact result and conv(a,b) equals conv(b,a), all length pairs of 1-9, 12, 13, 17") { convDyadicExactTest(); }
TEST_CASE("[C3 arm_conv_f32] a delta at position p shifts the other sequence by p (na 1, 2, 5, 9; nb 1-9, 12, 13, 17)") { convDeltaTest(); }
TEST_CASE("[C4 arm_conv_f32] the input arrays are not modified") { convConstInputsTest(); }

TEST_CASE("[C1 arm_conv_partial_f32] ARM's three windows (first 3, 9, 7; 4 points) return success and reach SNR 120, abs 1e-5, rel 1e-6") { armPartialTest(); }
TEST_CASE("[C3 arm_conv_partial_f32] every window of every length pair 1-9 equals the same part of the full convolution within the bound") { partialAllWindowsTest(); }
TEST_CASE("[C4 arm_conv_partial_f32] a window ending at srcALen+srcBLen-2 succeeds, one point further returns ARM_MATH_ARGUMENT_ERROR") { partialRangeErrorTest(); }
TEST_CASE("characterization: [C4 arm_conv_partial_f32] the window is written at pDst[firstIndex], nothing else is written, an error writes nothing (NEEDS REVIEW)") { partialPlacementCharacterization(); }

TEST_CASE("[C1 arm_levinson_durbin_f32] ARM's cases n = 3, 8, 11 reach SNR 120, abs and rel 1e-6 for the coefficients and for err, guard intact") { armLevinsonTest(); }
TEST_CASE("[C3 arm_levinson_durbin_f32] ARM's reference patterns equal the Yule-Walker solution of the DEF") { ldPatternIsDef(); }
TEST_CASE("[C3 arm_levinson_durbin_f32] hand derived Yule-Walker cases n = 1, 2, 3") { ldHandTest(); }
TEST_CASE("[C3 arm_levinson_durbin_f32] well conditioned random autocovariances (n 1-6, 8, 11) are within the conditioning bound of the DEF") { ldRandomTest(); }
TEST_CASE("[C2 arm_levinson_durbin_f32] exactly n coefficients are written (tight allocation, n 1-5, 7, 8, 9)") { ldTightTest(); }
TEST_CASE("[C3 arm_levinson_durbin_f32] the result satisfies the Yule-Walker equations (n 3, 8, 11) within the residual bound derived from the conditioning bound") { ldResidualTest(); }
