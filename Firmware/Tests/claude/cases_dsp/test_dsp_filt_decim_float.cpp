/*
 * CMSIS-DSP Filtering, decimation / interpolation float (work order DECF, phase P11 group G4): arm_fir_decimate_f32 /
 * arm_fir_decimate_init_f32, arm_fir_decimate_f64 / arm_fir_decimate_init_f64, arm_fir_interpolate_f32 /
 * arm_fir_interpolate_init_f32. Requirement ids: Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * Pattern files (Libraries/CMSIS-DSP/Testing/Patterns/DSP/Filtering/DECIM/DECIMF32/ and DECIMF64/), suites "DECIM F32" and
 * "DECIM F64" (Testing/desc.txt:2175-2220). ARM test functions Testing/Source/Tests/DECIMF32.cpp (test_fir_decimate_f32 :16,
 * test_fir_interpolate_f32 :72) and DECIMF64.cpp (test_fir_decimate_f64 :16).
 *   decimate  (f32 and f64): Input2 (input), Coefs2 (the coefficients of ALL configs, concatenated), Configs2_u32 (4 words per
 *             config: M, numTaps, blockSize, refSize = 72 configs) -> Reference2. Each config: init (status must be SUCCESS),
 *             ONE call with blockSize samples, refSize = blockSize/M outputs.
 *   interpolate (f32): Input3, Coefs3, Configs3_u32 (L, numTaps, blockSize, refSize = blockSize*L; 36 configs) -> Reference3.
 *   ARM sets the state buffer to 16 + 768 - 1 elements for every config; here a buffer of EXACTLY the DOC size is used
 *   (decimate numTaps + blockSize - 1, interpolate numTaps/L + blockSize - 1), with guards behind every output block (ARM
 *   checks ASSERT_EMPTY_TAIL on the output).
 *   NOT covered, disabled by ARM: arm_fir_interpolate_f64. DECIMF64.cpp:71-128 (test_fir_interpolate_f64) and :147-158 (its
 *   setUp case) are inside "#if 0", desc.txt:2193-2195 lists only test_fir_decimate_f64 for the f64 suite, and the library
 *   has no arm_fir_interpolate_f64 / init function (grep over Include/ and Source/): there is nothing to run.
 * ARM tolerances: f32 SNR_THRESHOLD 120 (DECIMF32.cpp:5), REL_ERROR 8.0e-4 (:13); f64 SNR_THRESHOLD 120 (DECIMF64.cpp:5),
 * REL_ERROR 8.0e-4 (:13), for decimate and interpolate. ASSERT_REL_ERROR is |a-b| / ((|a|+|b|)/2) <= thr, nothing compared
 * if both are 0 (FrameworkSource/Error.cpp:205-:220); armRel() does the same and also fails on NaN. Used unchanged (L4).
 * The ARM comment (DECIMF32.cpp:7-:12) says the references are generated "with a double precision computation".
 *
 * DOC (arm_fir_decimate_f32.c:33-100, arm_fir_decimate_init_f32.c): y[n] = b[0]*x[n] + b[1]*x[n-1] + ... + b[T-1]*x[n-T+1];
 * "pCoeffs points to a coefficient array of size numTaps. Coefficients are stored in time reversed order
 * {b[numTaps-1], ..., b[1], b[0]}"; "pDst points to an array of blockSize/M output values"; "blockSize must always be a
 * multiple of the decimation factor M"; "Instead of calculating all of the FIR filter outputs and discarding M-1 out of every
 * M, only the samples output by the decimator are computed"; "pState points to a state array of size numTaps + blockSize - 1".
 * Init: "Sets the values of the internal structure fields. Zeros out the values in the state buffer. Checks to make sure
 * that the size of the input is a multiple of the decimation factor." and "@return ARM_MATH_LENGTH_ERROR : blockSize is not a
 * multiple of M". For the array c = pCoeffs the DEF of the decimator is the FIR output at the times n = 0, M, 2M, ... of the
 * stream (x[m] = 0 for m < 0):
 *   y[m] = sum_{k=0}^{T-1} c[k] * x[M*m - T + 1 + k]
 * (c[T-1] weights x[M*m]). Whole blocks of the stream are multiples of M, so the phase of the decimation carries over
 * from block to block. The ARM reference patterns are checked against this DEF (convention check).
 *
 * DOC (arm_fir_interpolate_f32.c:35-119, arm_fir_interpolate_init_f32.c): the upsampler inserts L-1 zeros between the samples
 * and the FIR filter skips them; "pDst points to an array of blockSize*L output values"; polyphase y[n+i] = b[i]*x[n] +
 * b[L+i]*x[n-1] + ... + b[L*(P-1)+i]*x[n-P+1], P = phaseLength = numTaps/L; "numTaps must be a multiple of the interpolation
 * factor L"; "pState points to a state array of size blockSize + phaseLength - 1"; init "Sets the values of the internal
 * structure fields. Zeros out the values in the state buffer. Checks to make sure that the length of the filter is a
 * multiple of the interpolation factor." The DEF used here is the DOC's block diagram: zero stuffing u[L*m] = x[m], then the
 * FIR with b[t] = c[T-1-t]:  y[q] = sum_{t=0}^{T-1} b[t] * u[q-t]  (only P of the products are non zero).
 *
 * Error bounds (Higham, dot product): u = 2^-24 (f32) or 2^-53 (f64) is the unit roundoff; gam(k) = k*u/(1-k*u). Every
 * output is a sum of N rounded products (N = T for the decimator, N = P for the interpolator) accumulated in order; in any
 * order and with or without FMA the bound
 *   |y - ydef| <= gam(N) * S + N * sub,   S = sum |c * x| over the products,
 * holds (sub = smallest subnormal / 2: absolute error of a product in the subnormal range, zero for the data used here except
 * in the one-tap sweeps, which compare exact products), plus the error of the DEF itself, gam_ld(N) * S (long double,
 * 2^-64 per operation; on a platform where long double is double this is the bound of the double DEF), added to the bound.
 * No magic number.
 *
 * Host vs target (C6): the host runs the portable C path (ARM_MATH_LOOPUNROLL: outputs unrolled by 4, taps by 4; the
 * interpolator unrolls the input by 4 and the phase taps by 4); the target (-O2) may contract a*b+c into VFMA: the bound
 * holds for fused and unfused. The "exact" tests use dyadic data (multiples of 1/16, small), for which every product and sum is
 * exactly representable. The sweeps include subnormal inputs (a Cortex-M4F with FPSCR.FZ set would flush them; the reset
 * value is 0). The MVE / NEON code paths of the sources are not compiled on the host.
 *
 * Not tested, NEEDS REVIEW: blockSize 0 (DOC silent; the init arithmetic and kernel for 0 are unspecified), numTaps 0, M = 0
 * and L = 0 (division by zero in the init).
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
const double kULd = 5.4210108624275222e-20;  // 2^-64

struct DF32 {
	typedef float S;
	typedef arm_fir_decimate_instance_f32 Inst;
	static const char *name() { return "f32"; }
	static constexpr double u = 5.9604644775390625e-8;  // 2^-24
	static constexpr double sub = 7.006492321624085e-46;  // 2^-150
	static arm_status init(Inst *i, uint16_t t, uint8_t m, const S *c, S *st, uint32_t b) {
		return arm_fir_decimate_init_f32(i, t, m, c, st, b);
	}
	static void run(const Inst *i, const S *x, S *y, uint32_t b) { arm_fir_decimate_f32(i, x, y, b); }
	static S from(double d) { return static_cast<float>(d); }
	static double to(S v) { return static_cast<double>(v); }
	static std::string dir() { return "Filtering/DECIM/DECIMF32/"; }
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

struct DF64 {
	typedef double S;
	typedef arm_fir_decimate_instance_f64 Inst;
	static const char *name() { return "f64"; }
	static constexpr double u = 1.1102230246251565e-16;  // 2^-53
	static constexpr double sub = 2.4703282292062328e-324;  // 2^-1075 (rounds to 0 as a double constant, kept for the formula)
	static arm_status init(Inst *i, uint16_t t, uint8_t m, const S *c, S *st, uint32_t b) {
		return arm_fir_decimate_init_f64(i, t, m, c, st, b);
	}
	static void run(const Inst *i, const S *x, S *y, uint32_t b) { arm_fir_decimate_f64(i, x, y, b); }
	static S from(double d) { return d; }
	static double to(S v) { return v; }
	static std::string dir() { return "Filtering/DECIM/DECIMF64/"; }
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

template <class S>
bool isPoison(const S &v) {
	unsigned char raw[sizeof(S)];
	std::memcpy(raw, &v, sizeof(S));
	for (unsigned char byte : raw)
		if (byte != 0xCD) return false;
	return true;
}

template <class S>
bool isZeroBytes(const S &v) {
	unsigned char raw[sizeof(S)];
	std::memcpy(raw, &v, sizeof(S));
	for (unsigned char byte : raw)
		if (byte != 0) return false;
	return true;
}

// ---------------------------------------------------------------------------------------------------------------
// DEFs (long double)
// ---------------------------------------------------------------------------------------------------------------
// decimator: y[m] = sum_k c[k] * x[M*m - T + 1 + k]; S[m] = sum_k |c[k] * x[...]|
template <class Tr>
void defDecim(const std::vector<typename Tr::S> &c, size_t M, const std::vector<typename Tr::S> &x, std::vector<double> &y,
              std::vector<double> &sabs) {
	const long T = static_cast<long>(c.size());
	const size_t nOut = x.size() / M;
	y.assign(nOut, 0.0);
	sabs.assign(nOut, 0.0);
	for (size_t m = 0; m < nOut; m++) {
		long double acc = 0.0L, sa = 0.0L;
		for (long k = 0; k < T; k++) {
			const long idx = static_cast<long>(M * m) - T + 1 + k;
			if (idx < 0) continue;
			const long double p = static_cast<long double>(Tr::to(c[static_cast<size_t>(k)])) *
			                      static_cast<long double>(Tr::to(x[static_cast<size_t>(idx)]));
			acc += p;
			sa += (p < 0 ? -p : p);
		}
		y[m] = static_cast<double>(acc);
		sabs[m] = static_cast<double>(sa);
	}
}

// interpolator: zero stuffing u[L*m] = x[m], FIR with b[t] = c[T-1-t]: y[q] = sum_t b[t] * u[q-t]
template <class Tr>
void defInterp(const std::vector<typename Tr::S> &c, size_t L, const std::vector<typename Tr::S> &x, std::vector<double> &y,
               std::vector<double> &sabs) {
	const size_t T = c.size();
	y.assign(L * x.size(), 0.0);
	sabs.assign(L * x.size(), 0.0);
	for (size_t q = 0; q < L * x.size(); q++) {
		long double acc = 0.0L, sa = 0.0L;
		for (size_t t = q % L; t < T && t <= q; t += L) {  // u[q-t] is non zero only if (q - t) is a multiple of L
			const size_t m = (q - t) / L;
			const long double p = static_cast<long double>(Tr::to(c[T - 1 - t])) * static_cast<long double>(Tr::to(x[m]));
			acc += p;
			sa += (p < 0 ? -p : p);
		}
		y[q] = static_cast<double>(acc);
		sabs[q] = static_cast<double>(sa);
	}
}

// bound of the header: gam(N)*S + N*sub + gam_ld(N)*S
template <class Tr>
double bound(size_t N, double sabs) {
	return gam(static_cast<double>(N), Tr::u) * sabs + static_cast<double>(N) * Tr::sub + gam(static_cast<double>(N), kULd) * sabs;
}

// ---------------------------------------------------------------------------------------------------------------
// Runners: consecutive calls with exact size state and a guard behind every output block
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
struct RunResult {
	std::vector<typename Tr::S> out;
	bool guardsOk = true;
	bool noneLeftPoison = true;
};

// Value equality that also checks the sign of a zero result. The kernels add every product to an accumulator that starts at +0;
// in round-to-nearest (-0) + (+0) = +0 (also for a fused multiply-add), so an expected zero (a product of -0 or an underflow
// to either zero) must come out as +0. This is IEEE arithmetic of the DEF "sum starting from +0", not a statement of the DOC.
template <class S>
bool sameProduct(S y, S e) {
	if (!(y == e)) return false;
	return e != 0 || !std::signbit(y);
}

template <class S>
bool zerosArePositive(const std::vector<S> &expect, const std::vector<S> &out) {
	for (size_t i = 0; i < expect.size() && i < out.size(); i++)
		if (expect[i] == 0 && std::signbit(out[i])) return false;
	return true;
}

// The fields the kernels index with must hold the init arguments (guards the run-through tests against a crash on a stale instance)
template <class I, class C, class S>
bool decFieldsStored(const I &i, size_t T, size_t M, const C *c, const S *st) {
	return i.numTaps == T && i.M == M && i.pCoeffs == c && i.pState == st;
}
template <class I, class C, class S>
bool interpFieldsStored(const I &i, size_t L, size_t numTaps, const C *c, const S *st) {
	return i.L == L && i.phaseLength == numTaps / L && i.pCoeffs == c && i.pState == st;
}

// Decimator. x.size() is a multiple of M; chunk sizes are units * M (cycled, last one truncated). The state buffer has exactly
// numTaps + maxChunk - 1 elements and is initialised with blockSize = maxChunk.
// Every call is made twice: once on a tight heap state buffer (a std::vector of exactly that size: AddressSanitizer sees a kernel
// writing or reading one element past it) and once on a dsp::GuardedBuffer (guard words behind the state, visible in all
// builds, but in the same allocation so ASan cannot see them). The result is that of the tight run; guardsOk also requires
// both runs to give identical output.
template <class Tr>
RunResult<Tr> decRunOn(const std::vector<typename Tr::S> &c, size_t M, const std::vector<typename Tr::S> &x,
                       const std::vector<size_t> &units, typename Tr::S *stateMem, size_t maxChunk) {
	typedef typename Tr::S S;
	RunResult<Tr> r;
	typename Tr::Inst inst;
	const arm_status st = Tr::init(&inst, static_cast<uint16_t>(c.size()), static_cast<uint8_t>(M), c.data(), stateMem,
	                               static_cast<uint32_t>(maxChunk));
	REQUIRE(st == ARM_MATH_SUCCESS);
	REQUIRE(decFieldsStored(inst, c.size(), M, c.data(), stateMem));
	size_t pos = 0, ci = 0;
	while (pos < x.size()) {
		size_t b = units[ci++ % units.size()] * M;
		if (b > x.size() - pos) b = x.size() - pos;
		dsp::GuardedBuffer<S> out(b / M);
		Tr::run(&inst, x.data() + pos, out.data(), static_cast<uint32_t>(b));
		if (!out.guardIntact()) r.guardsOk = false;
		const std::vector<S> o = out.output();
		for (size_t i = 0; i < o.size(); i++) {
			if (isPoison(o[i])) r.noneLeftPoison = false;
			r.out.push_back(o[i]);
		}
		pos += b;
	}
	return r;
}

template <class Tr>
RunResult<Tr> decRun(const std::vector<typename Tr::S> &c, size_t M, const std::vector<typename Tr::S> &x,
                     const std::vector<size_t> &units) {
	typedef typename Tr::S S;
	size_t maxChunk = 0;
	for (size_t k : units) maxChunk = k * M > maxChunk ? k * M : maxChunk;
	const size_t n = c.size() + maxChunk - 1;
	std::vector<S> tight(n);
	std::memset(static_cast<void *>(tight.data()), 0xCD, n * sizeof(S));
	RunResult<Tr> r = decRunOn<Tr>(c, M, x, units, tight.data(), maxChunk);
	dsp::GuardedBuffer<S> state(n);
	const RunResult<Tr> g = decRunOn<Tr>(c, M, x, units, state.data(), maxChunk);
	if (!state.guardIntact() || !g.guardsOk) r.guardsOk = false;
	if (g.out.size() != r.out.size() || std::memcmp(g.out.data(), r.out.data(), r.out.size() * sizeof(S)) != 0) r.guardsOk = false;
	return r;
}

// Interpolator (float only). Chunk sizes are in input samples (cycled, last one truncated); state buffer exactly
// numTaps/L + maxChunk - 1 elements; tight heap buffer and guarded buffer as for the decimator.
RunResult<DF32> interpRunOn(const std::vector<float> &c, size_t L, const std::vector<float> &x, const std::vector<size_t> &chunks,
                            float *stateMem, size_t maxChunk) {
	RunResult<DF32> r;
	arm_fir_interpolate_instance_f32 inst;
	const arm_status st = arm_fir_interpolate_init_f32(&inst, static_cast<uint8_t>(L), static_cast<uint16_t>(c.size()), c.data(),
	                                                   stateMem, static_cast<uint32_t>(maxChunk));
	REQUIRE(st == ARM_MATH_SUCCESS);
	REQUIRE(interpFieldsStored(inst, L, c.size(), c.data(), stateMem));
	size_t pos = 0, ci = 0;
	while (pos < x.size()) {
		size_t b = chunks[ci++ % chunks.size()];
		if (b > x.size() - pos) b = x.size() - pos;
		dsp::GuardedBuffer<float> out(b * L);
		arm_fir_interpolate_f32(&inst, x.data() + pos, out.data(), static_cast<uint32_t>(b));
		if (!out.guardIntact()) r.guardsOk = false;
		const std::vector<float> o = out.output();
		for (size_t i = 0; i < o.size(); i++) {
			if (isPoison(o[i])) r.noneLeftPoison = false;
			r.out.push_back(o[i]);
		}
		pos += b;
	}
	return r;
}

RunResult<DF32> interpRun(const std::vector<float> &c, size_t L, const std::vector<float> &x, const std::vector<size_t> &chunks) {
	size_t maxChunk = 0;
	for (size_t k : chunks) maxChunk = k > maxChunk ? k : maxChunk;
	const size_t n = c.size() / L + maxChunk - 1;
	std::vector<float> tight(n);
	std::memset(static_cast<void *>(tight.data()), 0xCD, n * sizeof(float));
	RunResult<DF32> r = interpRunOn(c, L, x, chunks, tight.data(), maxChunk);
	dsp::GuardedBuffer<float> state(n);
	const RunResult<DF32> g = interpRunOn(c, L, x, chunks, state.data(), maxChunk);
	if (!state.guardIntact() || !g.guardsOk) r.guardsOk = false;
	if (g.out.size() != r.out.size() || std::memcmp(g.out.data(), r.out.data(), r.out.size() * sizeof(float)) != 0) r.guardsOk = false;
	return r;
}

const size_t kTaps[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 17, 25, 33};
const size_t kFactors[] = {1, 2, 3, 4, 5, 8};

// ---------------------------------------------------------------------------------------------------------------
// C1: ARM's tests
// ---------------------------------------------------------------------------------------------------------------
template <class Tr>
void armDecimTest(double snrThr, double relThr) {
	typedef typename Tr::S S;
	const char *sfx = Tr::name();
	const std::vector<S> input = Tr::pat(std::string("Input2_") + sfx + ".txt");
	const std::vector<S> coefs = Tr::pat(std::string("Coefs2_") + sfx + ".txt");
	const std::vector<S> ref = Tr::pat(std::string("Reference2_") + sfx + ".txt");
	const std::vector<int32_t> cfg = dsp::loadPattern(Tr::dir() + "Configs2_u32.txt").q31();
	REQUIRE(cfg.size() == 288);
	dsp::GuardedBuffer<S> out(ref.size());
	size_t cp = 0, ip = 0, op = 0;
	bool guards = true, fields = true, status = true;
	for (size_t i = 0; i + 3 < cfg.size(); i += 4) {
		const size_t M = static_cast<size_t>(cfg[i]), T = static_cast<size_t>(cfg[i + 1]);
		const size_t B = static_cast<size_t>(cfg[i + 2]), R = static_cast<size_t>(cfg[i + 3]);
		REQUIRE(R * M == B);
		REQUIRE(cp + T <= coefs.size());
		REQUIRE(ip + B <= input.size());
		REQUIRE(op + R <= ref.size());
		dsp::GuardedBuffer<S> state(T + B - 1);
		typename Tr::Inst inst;
		const bool initOk = Tr::init(&inst, static_cast<uint16_t>(T), static_cast<uint8_t>(M), coefs.data() + cp, state.data(),
		                             static_cast<uint32_t>(B)) == ARM_MATH_SUCCESS;
		status = status && initOk;
		const bool stored = decFieldsStored(inst, T, M, coefs.data() + cp, state.data());
		fields = fields && stored;
		if (!initOk || !stored) break;  // never run the kernel on an instance that init did not fill
		Tr::run(&inst, input.data() + ip, out.data() + op, static_cast<uint32_t>(B));
		guards = guards && state.guardIntact();
		ip += B;
		op += R;
		cp += T;
	}
	CHECK(status);
	CHECK(fields);
	CHECK(guards);
	CHECK(out.guardIntact());
	CHECK(cp == coefs.size());
	CHECK(ip == input.size());
	REQUIRE(op == ref.size());
	const std::vector<S> all = out.output();
	CHECK(dsp::snr(ref, all) >= snrThr);
	auto c = armRel(ref, all, relThr);
	CHECK_MESSAGE(c, c.message);
}

void armInterpTest(double snrThr, double relThr) {
	const std::vector<float> input = DF32::pat("Input3_f32.txt");
	const std::vector<float> coefs = DF32::pat("Coefs3_f32.txt");
	const std::vector<float> ref = DF32::pat("Reference3_f32.txt");
	const std::vector<int32_t> cfg = dsp::loadPattern(DF32::dir() + "Configs3_u32.txt").q31();
	REQUIRE(cfg.size() == 144);
	dsp::GuardedBuffer<float> out(ref.size());
	size_t cp = 0, ip = 0, op = 0;
	bool guards = true, fields = true, status = true;
	for (size_t i = 0; i + 3 < cfg.size(); i += 4) {
		const size_t L = static_cast<size_t>(cfg[i]), T = static_cast<size_t>(cfg[i + 1]);
		const size_t B = static_cast<size_t>(cfg[i + 2]), R = static_cast<size_t>(cfg[i + 3]);
		REQUIRE(R == B * L);
		REQUIRE(T % L == 0);
		REQUIRE(cp + T <= coefs.size());
		REQUIRE(ip + B <= input.size());
		REQUIRE(op + R <= ref.size());
		const size_t P = T / L;
		dsp::GuardedBuffer<float> state(P + B - 1);
		arm_fir_interpolate_instance_f32 inst;
		const bool initOk = arm_fir_interpolate_init_f32(&inst, static_cast<uint8_t>(L), static_cast<uint16_t>(T), coefs.data() + cp,
		                                                 state.data(), static_cast<uint32_t>(B)) == ARM_MATH_SUCCESS;
		status = status && initOk;
		const bool stored = interpFieldsStored(inst, L, T, coefs.data() + cp, state.data());
		fields = fields && stored;
		if (!initOk || !stored) break;  // never run the kernel on an instance that init did not fill
		arm_fir_interpolate_f32(&inst, input.data() + ip, out.data() + op, static_cast<uint32_t>(B));
		guards = guards && state.guardIntact();
		ip += B;
		op += R;
		cp += T;
	}
	CHECK(status);
	CHECK(fields);
	CHECK(guards);
	CHECK(out.guardIntact());
	CHECK(cp == coefs.size());
	CHECK(ip == input.size());
	REQUIRE(op == ref.size());
	const std::vector<float> all = out.output();
	CHECK(dsp::snr(ref, all) >= snrThr);
	auto c = armRel(ref, all, relThr);
	CHECK_MESSAGE(c, c.message);
}

// ARM reference pattern against the DEF (checks the coefficient order and the phase of the DEFs)
template <class Tr>
void decimPatternIsDefTest() {
	typedef typename Tr::S S;
	const char *sfx = Tr::name();
	const std::vector<S> input = Tr::pat(std::string("Input2_") + sfx + ".txt");
	const std::vector<S> coefs = Tr::pat(std::string("Coefs2_") + sfx + ".txt");
	const std::vector<S> ref = Tr::pat(std::string("Reference2_") + sfx + ".txt");
	const std::vector<int32_t> cfg = dsp::loadPattern(Tr::dir() + "Configs2_u32.txt").q31();
	size_t cp = 0, ip = 0, rp = 0, bad = 0;
	std::string first;
	for (size_t i = 0; i + 3 < cfg.size(); i += 4) {
		const size_t M = static_cast<size_t>(cfg[i]), T = static_cast<size_t>(cfg[i + 1]);
		const size_t B = static_cast<size_t>(cfg[i + 2]), R = static_cast<size_t>(cfg[i + 3]);
		const std::vector<S> c(coefs.begin() + static_cast<long>(cp), coefs.begin() + static_cast<long>(cp + T));
		const std::vector<S> x(input.begin() + static_cast<long>(ip), input.begin() + static_cast<long>(ip + B));
		std::vector<double> y, sa;
		defDecim<Tr>(c, M, x, y, sa);
		REQUIRE(y.size() == R);
		for (size_t n = 0; n < R; n++) {
			// the reference is a double result of the UNROUNDED coefficients and inputs (each rounded to the type afterwards:
			// 2*u*S) rounded to the type itself (u*|y|); the script's double sum has error gam(T)*S at 2^-53, my long double
			// DEF about the same at most: 2*gam_64(T)*S
			const double b = Tr::u * std::fabs(y[n]) + 2.0 * Tr::u * sa[n] + 2.0 * gam(static_cast<double>(T), kU64) * sa[n];
			const double e = std::fabs(Tr::to(ref[rp + n]) - y[n]);
			if (!(e <= b)) {
				if (bad == 0) first = "config " + std::to_string(i / 4) + " sample " + std::to_string(n);
				bad++;
			}
		}
		cp += T;
		ip += B;
		rp += R;
	}
	CHECK_MESSAGE(bad == 0, bad << " samples differ from the DEF, first at " << first);
}

void interpPatternIsDefTest() {
	const std::vector<float> input = DF32::pat("Input3_f32.txt");
	const std::vector<float> coefs = DF32::pat("Coefs3_f32.txt");
	const std::vector<float> ref = DF32::pat("Reference3_f32.txt");
	const std::vector<int32_t> cfg = dsp::loadPattern(DF32::dir() + "Configs3_u32.txt").q31();
	size_t cp = 0, ip = 0, rp = 0, bad = 0;
	std::string first;
	for (size_t i = 0; i + 3 < cfg.size(); i += 4) {
		const size_t L = static_cast<size_t>(cfg[i]), T = static_cast<size_t>(cfg[i + 1]);
		const size_t B = static_cast<size_t>(cfg[i + 2]), R = static_cast<size_t>(cfg[i + 3]);
		const std::vector<float> c(coefs.begin() + static_cast<long>(cp), coefs.begin() + static_cast<long>(cp + T));
		const std::vector<float> x(input.begin() + static_cast<long>(ip), input.begin() + static_cast<long>(ip + B));
		std::vector<double> y, sa;
		defInterp<DF32>(c, L, x, y, sa);
		REQUIRE(y.size() == R);
		for (size_t n = 0; n < R; n++) {
			const double b = DF32::u * std::fabs(y[n]) + 2.0 * DF32::u * sa[n] + 2.0 * gam(static_cast<double>(T / L), kU64) * sa[n];
			const double e = std::fabs(static_cast<double>(ref[rp + n]) - y[n]);
			if (!(e <= b)) {
				if (bad == 0) first = "config " + std::to_string(i / 4) + " sample " + std::to_string(n);
				bad++;
			}
		}
		cp += T;
		ip += B;
		rp += R;
	}
	CHECK_MESSAGE(bad == 0, bad << " samples differ from the DEF, first at " << first);
}

// ---------------------------------------------------------------------------------------------------------------
// C3: decimator
// ---------------------------------------------------------------------------------------------------------------
// Impulse of height 1 at stream index j: y[m] = sum_k c[k]*x[M*m-T+1+k] is non zero for the k with M*m-T+1+k = j, i.e.
// y[m] = c[T-1+j-M*m] if 0 <= T-1+j-M*m <= T-1, else 0. Dyadic coefficients: exact.
template <class Tr>
void decimImpulseTest() {
	typedef typename Tr::S S;
	for (size_t M : {1u, 2u, 3u, 4u}) {
		for (size_t T = 1; T <= 12; T++) {
			std::vector<S> c(T);
			for (size_t k = 0; k < T; k++) c[k] = Tr::from(((k % 2) ? -1.0 : 1.0) * static_cast<double>(k + 1) / 16.0);
			for (size_t j : {0u, 1u, 2u}) {
				const size_t nOut = T + j + 2;
				std::vector<S> x(M * nOut, Tr::from(0.0));
				x[j] = Tr::from(1.0);
				const RunResult<Tr> r = decRun<Tr>(c, M, x, {nOut});
				std::vector<S> expect(nOut, Tr::from(0.0));
				for (size_t m = 0; m < nOut; m++) {
					const long idx = static_cast<long>(T) - 1 + static_cast<long>(j) - static_cast<long>(M * m);
					if (idx >= 0 && idx <= static_cast<long>(T) - 1) expect[m] = c[static_cast<size_t>(idx)];
				}
				auto chk = dsp::exactEqual(expect, r.out);
				CHECK_MESSAGE(chk, "M " << M << " numTaps " << T << " impulse at " << j << ": " << chk.message);
				CHECK_MESSAGE(zerosArePositive(expect, r.out), "M " << M << " numTaps " << T << " impulse at " << j << ": a zero output is -0");
				CHECK(r.guardsOk);
			}
		}
	}
}

// Step x = 1: y[m] = sum_{k >= max(0, T-1-M*m)} c[k] (x[M*m-T+1+k] = 1 once the index is >= 0)
template <class Tr>
void decimStepTest() {
	typedef typename Tr::S S;
	for (size_t M : {1u, 2u, 3u, 4u}) {
		for (size_t T = 1; T <= 12; T++) {
			std::vector<S> c(T);
			for (size_t k = 0; k < T; k++) c[k] = Tr::from(((k % 3 == 1) ? -1.0 : 1.0) * static_cast<double>(k + 1) / 16.0);
			const size_t nOut = T + 3;
			std::vector<S> x(M * nOut, Tr::from(1.0));
			const RunResult<Tr> r = decRun<Tr>(c, M, x, {nOut});
			std::vector<S> expect(nOut);
			for (size_t m = 0; m < nOut; m++) {
				const long lo = static_cast<long>(T) - 1 - static_cast<long>(M * m);
				double acc = 0.0;
				for (size_t j = lo > 0 ? static_cast<size_t>(lo) : 0; j < T; j++) acc += Tr::to(c[j]);  // dyadic, exact in double
				expect[m] = Tr::from(acc);
			}
			auto chk = dsp::exactEqual(expect, r.out);
			CHECK_MESSAGE(chk, "M " << M << " numTaps " << T << ": " << chk.message);
		}
	}
}

// Random data, one call per block size (multiples of M) from a zero state, against the DEF with the bound of the header (N = T)
template <class Tr>
void decimRandomVsDefTest() {
	typedef typename Tr::S S;
	uint32_t seed = 1;
	size_t bad = 0;
	std::string first;
	for (size_t M : kFactors) {
		for (size_t T : kTaps) {
			for (size_t U : {1u, 2u, 3u, 4u, 5u, 7u, 9u}) {
				const size_t B = U * M;
				const std::vector<S> c = randVec<Tr>(T, seed++, -1.0, 1.0);
				const std::vector<S> x = randVec<Tr>(B, seed++, -1.0, 1.0);
				std::vector<double> y, sa;
				defDecim<Tr>(c, M, x, y, sa);
				const RunResult<Tr> r = decRun<Tr>(c, M, x, {U});
				REQUIRE(r.out.size() == U);
				for (size_t n = 0; n < U; n++) {
					const double e = std::fabs(Tr::to(r.out[n]) - y[n]);
					if (!(e <= bound<Tr>(T, sa[n]))) {
						if (bad == 0)
							first = "M " + std::to_string(M) + " taps " + std::to_string(T) + " block " + std::to_string(B) + " n " + std::to_string(n);
						bad++;
					}
				}
			}
		}
	}
	CHECK_MESSAGE(bad == 0, bad << " outputs outside the bound, first at " << first);
}

template <class Tr>
void decimWritesExactlyTest() {
	typedef typename Tr::S S;
	uint32_t seed = 500;
	for (size_t M : {1u, 2u, 3u, 5u}) {
		for (size_t T : {1u, 4u, 5u, 9u}) {
			for (size_t U : {1u, 2u, 3u, 4u, 5u, 7u, 9u}) {
				const std::vector<S> c = randVec<Tr>(T, seed++, -0.5, 0.5);
				const std::vector<S> x = randVec<Tr>(U * M, seed++, -0.5, 0.5);
				const RunResult<Tr> r = decRun<Tr>(c, M, x, {U});
				CHECK_MESSAGE(r.guardsOk, "M " << M << " taps " << T << " outputs " << U << ": guard behind the output or the state damaged");
				CHECK_MESSAGE(r.noneLeftPoison, "M " << M << " taps " << T << " outputs " << U << ": an output element was not written");
				CHECK(r.out.size() == U);
			}
		}
	}
}

// Consecutive blocks (multiples of M, from 1 output up, not multiples of the unroll factor 4) carry the state
template <class Tr>
void decimContinuityTest() {
	typedef typename Tr::S S;
	const std::vector<std::vector<size_t>> plans = {{1}, {2}, {3}, {5}, {1, 2, 3, 5}, {4, 1, 7}, {24}};
	uint32_t seed = 900;
	size_t bad = 0;
	std::string first;
	for (size_t M : {1u, 2u, 3u, 5u}) {
		for (size_t T : {1u, 2u, 3u, 4u, 5u, 8u, 9u, 12u, 17u}) {
			const std::vector<S> c = randVec<Tr>(T, seed++, -1.0, 1.0);
			const std::vector<S> x = randVec<Tr>(24 * M, seed++, -1.0, 1.0);
			std::vector<double> y, sa;
			defDecim<Tr>(c, M, x, y, sa);
			for (size_t p = 0; p < plans.size(); p++) {
				const RunResult<Tr> r = decRun<Tr>(c, M, x, plans[p]);
				REQUIRE(r.out.size() == 24);
				CHECK(r.guardsOk);
				for (size_t n = 0; n < 24; n++) {
					const double e = std::fabs(Tr::to(r.out[n]) - y[n]);
					if (!(e <= bound<Tr>(T, sa[n]))) {
						if (bad == 0)
							first = "M " + std::to_string(M) + " taps " + std::to_string(T) + " plan " + std::to_string(p) + " n " + std::to_string(n);
						bad++;
					}
				}
			}
		}
	}
	CHECK_MESSAGE(bad == 0, bad << " outputs outside the bound, first at " << first);
}

template <class Tr>
void decimIndependentInstancesTest() {
	typedef typename Tr::S S;
	const size_t Ta = 5, Tb = 9, Ma = 2, Mb = 3, B = 6, blocks = 6;
	const std::vector<S> ca = randVec<Tr>(Ta, 31, -1.0, 1.0), cb = randVec<Tr>(Tb, 32, -1.0, 1.0);
	const std::vector<S> xa = randVec<Tr>(B * blocks, 33, -1.0, 1.0), xb = randVec<Tr>(B * blocks, 34, -1.0, 1.0);
	dsp::GuardedBuffer<S> sa(Ta + B - 1), sb(Tb + B - 1);
	typename Tr::Inst ia, ib;
	REQUIRE(Tr::init(&ia, Ta, Ma, ca.data(), sa.data(), B) == ARM_MATH_SUCCESS);
	REQUIRE(decFieldsStored(ia, Ta, Ma, ca.data(), sa.data()));
	REQUIRE(Tr::init(&ib, Tb, Mb, cb.data(), sb.data(), B) == ARM_MATH_SUCCESS);
	REQUIRE(decFieldsStored(ib, Tb, Mb, cb.data(), sb.data()));
	std::vector<S> oa, ob;
	for (size_t k = 0; k < blocks; k++) {
		std::vector<S> pa(B / Ma), pb(B / Mb);
		Tr::run(&ia, xa.data() + k * B, pa.data(), B);
		Tr::run(&ib, xb.data() + k * B, pb.data(), B);
		oa.insert(oa.end(), pa.begin(), pa.end());
		ob.insert(ob.end(), pb.begin(), pb.end());
	}
	std::vector<double> ya, yb, sca, scb;
	defDecim<Tr>(ca, Ma, xa, ya, sca);
	defDecim<Tr>(cb, Mb, xb, yb, scb);
	REQUIRE(oa.size() == ya.size());
	REQUIRE(ob.size() == yb.size());
	size_t bad = 0;
	for (size_t n = 0; n < oa.size(); n++)
		if (!(std::fabs(Tr::to(oa[n]) - ya[n]) <= bound<Tr>(Ta, sca[n]))) bad++;
	for (size_t n = 0; n < ob.size(); n++)
		if (!(std::fabs(Tr::to(ob[n]) - yb[n]) <= bound<Tr>(Tb, scb[n]))) bad++;
	CHECK(bad == 0);
	CHECK(sa.guardIntact());
	CHECK(sb.guardIntact());
}

// Re-init with a poisoned state buffer resets the history: the second run is bit identical to the first
template <class Tr>
void decimReinitTest() {
	typedef typename Tr::S S;
	const size_t T = 6, M = 2, B = 10;
	const std::vector<S> c = randVec<Tr>(T, 41, -1.0, 1.0);
	const std::vector<S> x = randVec<Tr>(3 * B, 42, -1.0, 1.0);
	dsp::GuardedBuffer<S> state(T + B - 1);
	typename Tr::Inst inst;
	REQUIRE(Tr::init(&inst, T, M, c.data(), state.data(), B) == ARM_MATH_SUCCESS);
	REQUIRE(decFieldsStored(inst, T, M, c.data(), state.data()));
	std::vector<S> first(3 * B / M), second(3 * B / M);
	for (size_t k = 0; k < 3; k++) Tr::run(&inst, x.data() + k * B, first.data() + k * B / M, B);
	std::memset(static_cast<void *>(state.data()), 0xCD, (T + B - 1) * sizeof(S));
	REQUIRE(Tr::init(&inst, T, M, c.data(), state.data(), B) == ARM_MATH_SUCCESS);
	REQUIRE(decFieldsStored(inst, T, M, c.data(), state.data()));
	for (size_t k = 0; k < 3; k++) Tr::run(&inst, x.data() + k * B, second.data() + k * B / M, B);
	CHECK(std::memcmp(first.data(), second.data(), first.size() * sizeof(S)) == 0);
	CHECK(state.guardIntact());
}

template <class Tr>
void decimInitTest() {
	typedef typename Tr::S S;
	const size_t cfg[][3] = {{1, 1, 1}, {5, 2, 4}, {3, 3, 9}, {8, 4, 8}, {25, 5, 10}, {4, 2, 2}, {1, 5, 5}};  // T, M, B
	for (const auto &t : cfg) {
		const size_t T = t[0], M = t[1], B = t[2];
		const std::vector<S> c = randVec<Tr>(T, 61, -1.0, 1.0);
		dsp::GuardedBuffer<S> state(T + B - 1);  // poisoned with 0xCD by the buffer
		typename Tr::Inst inst;
		std::memset(&inst, 0x5A, sizeof inst);
		const arm_status st = Tr::init(&inst, static_cast<uint16_t>(T), static_cast<uint8_t>(M), c.data(), state.data(), static_cast<uint32_t>(B));
		CHECK(st == ARM_MATH_SUCCESS);
		CHECK(inst.numTaps == T);
		CHECK(inst.M == M);
		CHECK(inst.pCoeffs == c.data());
		CHECK(inst.pState == state.data());
		bool zero = true;
		for (const S &v : state.output()) zero = zero && isZeroBytes(v);
		CHECK_MESSAGE(zero, "taps " << T << " M " << M << " block " << B << ": state not zeroed");
		CHECK_MESSAGE(state.guardIntact(), "taps " << T << " M " << M << " block " << B << ": init wrote behind the state buffer");
	}
}

// DOC: "ARM_MATH_LENGTH_ERROR : blockSize is not a multiple of M"
template <class Tr>
void decimInitErrorTest() {
	typedef typename Tr::S S;
	const size_t cfg[][2] = {{2, 1}, {2, 3}, {3, 4}, {4, 6}, {5, 11}, {8, 12}};  // M, B
	for (const auto &t : cfg) {
		const size_t M = t[0], B = t[1];
		const std::vector<S> c = randVec<Tr>(4, 71, -1.0, 1.0);
		dsp::GuardedBuffer<S> state(4 + B - 1);
		typename Tr::Inst inst;
		CHECK_MESSAGE(Tr::init(&inst, 4, static_cast<uint8_t>(M), c.data(), state.data(), static_cast<uint32_t>(B)) == ARM_MATH_LENGTH_ERROR,
		              "M " << M << " blockSize " << B);
	}
}

// Not stated by the DOC: on the error the instance and the state buffer are left as they were (well defined, harmless)
template <class Tr>
void decimInitErrorUnmodifiedTest() {
	typedef typename Tr::S S;
	const std::vector<S> c = randVec<Tr>(4, 72, -1.0, 1.0);
	dsp::GuardedBuffer<S> state(4 + 7 - 1);
	typename Tr::Inst inst, before;
	std::memset(&inst, 0x5A, sizeof inst);
	std::memcpy(&before, &inst, sizeof inst);
	CHECK(Tr::init(&inst, 4, 2, c.data(), state.data(), 7) == ARM_MATH_LENGTH_ERROR);
	CHECK(std::memcmp(&inst, &before, sizeof inst) == 0);
	bool untouched = true;
	for (const S &v : state.output()) untouched = untouched && isPoison(v);
	CHECK(untouched);
	CHECK(state.guardIntact());
}

// DOC: only blockSize must be a multiple of M; numTaps is free (numTaps 5 with M 2 and 3)
template <class Tr>
void decimTapsNotMultipleOfMTest() {
	typedef typename Tr::S S;
	for (size_t M : {2u, 3u}) {
		const size_t T = 5, B = 6;
		const std::vector<S> c = randVec<Tr>(T, 81, -1.0, 1.0);
		const std::vector<S> x = randVec<Tr>(B, 82, -1.0, 1.0);
		std::vector<double> y, sa;
		defDecim<Tr>(c, M, x, y, sa);
		const RunResult<Tr> r = decRun<Tr>(c, M, x, {B / M});
		REQUIRE(r.out.size() == y.size());
		for (size_t n = 0; n < y.size(); n++) CHECK(std::fabs(Tr::to(r.out[n]) - y[n]) <= bound<Tr>(T, sa[n]));
	}
}

template <class Tr>
void decimConstInputsTest() {
	typedef typename Tr::S S;
	const std::vector<S> c = randVec<Tr>(9, 91, -1.0, 1.0);
	const std::vector<S> x = randVec<Tr>(24, 92, -1.0, 1.0);
	std::vector<S> c2(c), x2(x);
	const RunResult<Tr> r = decRun<Tr>(c2, 3, x2, {2, 1, 3});
	CHECK(r.out.size() == 8);
	CHECK(std::memcmp(c.data(), c2.data(), c.size() * sizeof(S)) == 0);
	CHECK(std::memcmp(x.data(), x2.data(), x.size() * sizeof(S)) == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// Decimator sweeps
// ---------------------------------------------------------------------------------------------------------------
// One tap, M = 3: y[m] = c0 * x[3m]. The product of two floats is exact in double (24+24 bits), so (float)(double product) is
// the correctly rounded product that IEEE multiplication gives. The patterns (strided over the whole 32 bit range: all
// exponents, both signs, subnormals, infinities; NaN skipped) fill every position of the block, the outputs test every third.
void sweepDecimOneTapF32() {
	const float c0 = 0.7f;
	const float cArr[1] = {c0};
	const size_t M = 3, OUT = 1024, BLK = M * OUT;
	std::vector<float> x(BLK), y(OUT), state(BLK);  // numTaps + blockSize - 1 = 1 + BLK - 1
	arm_fir_decimate_instance_f32 inst;
	REQUIRE(arm_fir_decimate_init_f32(&inst, 1, M, cArr, state.data(), BLK) == ARM_MATH_SUCCESS);
	REQUIRE(decFieldsStored(inst, 1, M, cArr, state.data()));
	size_t n = 0, bad = 0, tested = 0;
	uint32_t firstBad = 0;
	for (uint64_t bits = 0; bits < 0x100000000ull; bits += 1361ull) {
		const uint32_t b = static_cast<uint32_t>(bits);
		float f;
		std::memcpy(&f, &b, sizeof f);
		if (std::isnan(f)) continue;
		x[n++] = f;
		if (n == BLK) {
			arm_fir_decimate_f32(&inst, x.data(), y.data(), BLK);
			for (size_t m = 0; m < OUT; m++) {
				const float e = static_cast<float>(static_cast<double>(c0) * static_cast<double>(x[M * m]));
				if (!sameProduct(y[m], e)) {
					if (bad == 0) std::memcpy(&firstBad, &x[M * m], sizeof firstBad);
					bad++;
				}
			}
			tested += OUT;
			n = 0;
		}
	}
	CHECK(tested > 1000000);
	CHECK_MESSAGE(bad == 0, bad << " products differ, first input bits 0x" << std::hex << firstBad);
}

void sweepDecimOneTapF64() {
	const double c0 = 0.7;
	const double cArr[1] = {c0};
	const size_t M = 3, OUT = 1024, BLK = M * OUT;
	std::vector<double> x(BLK), y(OUT), state(BLK);
	arm_fir_decimate_instance_f64 inst;
	REQUIRE(arm_fir_decimate_init_f64(&inst, 1, M, cArr, state.data(), BLK) == ARM_MATH_SUCCESS);
	REQUIRE(decFieldsStored(inst, 1, M, cArr, state.data()));
	size_t n = 0, bad = 0, tested = 0;
	uint64_t firstBad = 0;
	uint64_t bits = 0;
	for (uint64_t k = 0; k < 3300000ull; k++, bits += 17592186044417ull) {  // odd stride, wraps over the 64 bit range
		double f;
		std::memcpy(&f, &bits, sizeof f);
		if (std::isnan(f)) continue;
		x[n++] = f;
		if (n == BLK) {
			arm_fir_decimate_f64(&inst, x.data(), y.data(), BLK);
			for (size_t m = 0; m < OUT; m++) {
				volatile double e = c0 * x[M * m];  // IEEE double product (the DEF of a one tap filter), no fusion possible
				if (!sameProduct(y[m], e)) {
					if (bad == 0) std::memcpy(&firstBad, &x[M * m], sizeof firstBad);
					bad++;
				}
			}
			tested += OUT;
			n = 0;
		}
	}
	CHECK(tested > 1000000);
	CHECK_MESSAGE(bad == 0, bad << " products differ, first input bits 0x" << std::hex << firstBad);
}

// Two taps, M = 2, over the range of normal inputs of the type (biased exponent window, so that no product leaves the normal
// range; no zero input, so the sign of a zero result is only reachable by exact cancellation and is not compared, the bound is):
// y[m] = c0*x[2m-1] + c1*x[2m] against the long double DEF with the bound of the header (N = 2).
template <class Tr>
void sweepDecimTwoTapTest() {
	typedef typename Tr::S S;
	const S c[2] = {Tr::from(0.7), Tr::from(-1.3)};
	const std::vector<S> cv(c, c + 2);
	const size_t M = 2, BLK = 4096;
	std::vector<S> x(BLK), y(BLK / M), state(BLK + 1);
	typename Tr::Inst inst;
	REQUIRE(Tr::init(&inst, 2, M, c, state.data(), BLK) == ARM_MATH_SUCCESS);
	REQUIRE(decFieldsStored(inst, 2, M, c, state.data()));
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
			defDecim<Tr>(cv, M, x, yd, sa);
			// output 0 continues the previous block (history of the previous call): the DEF restarts from a zero history, skip it
			for (size_t i = 1; i < BLK / M; i++)
				if (!(std::fabs(Tr::to(y[i]) - yd[i]) <= bound<Tr>(2, sa[i]))) bad++;
			tested += BLK;
			n = 0;
		}
	}
	CHECK(tested >= 4096 * 20);
	CHECK_MESSAGE(bad == 0, bad << " outputs outside the bound");
}

// ---------------------------------------------------------------------------------------------------------------
// C3 / C4: interpolator (f32)
// ---------------------------------------------------------------------------------------------------------------
const size_t kPhase[] = {1, 2, 3, 4, 5, 6, 9};
const size_t kInBlocks[] = {1, 2, 3, 4, 5, 7, 8, 9, 17};

// Impulse x = {1, 0, ...}: y[q] = b[q] = c[T-1-q] for q < T, then 0 (zero stuffing: u[0] = 1 is the only non zero sample)
void interpImpulseTest() {
	for (size_t L : {1u, 2u, 3u, 4u}) {
		for (size_t P = 1; P <= 9; P++) {
			const size_t T = L * P;
			std::vector<float> c(T);
			for (size_t k = 0; k < T; k++) c[k] = static_cast<float>(((k % 2) ? -1.0 : 1.0) * static_cast<double>(k + 1) / 16.0);
			const size_t N = P + 2;
			std::vector<float> x(N, 0.0f);
			x[0] = 1.0f;
			const RunResult<DF32> r = interpRun(c, L, x, {N});
			std::vector<float> expect(L * N, 0.0f);
			for (size_t q = 0; q < T; q++) expect[q] = c[T - 1 - q];
			auto chk = dsp::exactEqual(expect, r.out);
			CHECK_MESSAGE(chk, "L " << L << " phaseLength " << P << ": " << chk.message);
			CHECK_MESSAGE(zerosArePositive(expect, r.out), "L " << L << " phaseLength " << P << ": a zero output is -0");
			CHECK(r.guardsOk);
		}
	}
}

// Step x = 1: y[L*n+i] = sum_{j=0}^{min(n,P-1)} b[L*j+i], b[t] = c[T-1-t]
void interpStepTest() {
	for (size_t L : {1u, 2u, 3u, 4u}) {
		for (size_t P = 1; P <= 9; P++) {
			const size_t T = L * P;
			std::vector<float> c(T);
			for (size_t k = 0; k < T; k++) c[k] = static_cast<float>(((k % 3 == 1) ? -1.0 : 1.0) * static_cast<double>(k + 1) / 16.0);
			const size_t N = P + 3;
			std::vector<float> x(N, 1.0f);
			const RunResult<DF32> r = interpRun(c, L, x, {N});
			std::vector<float> expect(L * N);
			for (size_t n = 0; n < N; n++)
				for (size_t i = 0; i < L; i++) {
					double acc = 0.0;
					for (size_t j = 0; j <= (n < P - 1 ? n : P - 1); j++) acc += static_cast<double>(c[T - 1 - (L * j + i)]);
					expect[L * n + i] = static_cast<float>(acc);
				}
			auto chk = dsp::exactEqual(expect, r.out);
			CHECK_MESSAGE(chk, "L " << L << " phaseLength " << P << ": " << chk.message);
		}
	}
}

void interpRandomVsDefTest() {
	uint32_t seed = 1;
	size_t bad = 0;
	std::string first;
	for (size_t L : kFactors) {
		for (size_t P : kPhase) {
			for (size_t B : kInBlocks) {
				const std::vector<float> c = randVec<DF32>(L * P, seed++, -1.0, 1.0);
				const std::vector<float> x = randVec<DF32>(B, seed++, -1.0, 1.0);
				std::vector<double> y, sa;
				defInterp<DF32>(c, L, x, y, sa);
				const RunResult<DF32> r = interpRun(c, L, x, {B});
				REQUIRE(r.out.size() == L * B);
				for (size_t n = 0; n < L * B; n++) {
					const double e = std::fabs(static_cast<double>(r.out[n]) - y[n]);
					if (!(e <= bound<DF32>(P, sa[n]))) {
						if (bad == 0)
							first = "L " + std::to_string(L) + " P " + std::to_string(P) + " block " + std::to_string(B) + " n " + std::to_string(n);
						bad++;
					}
				}
			}
		}
	}
	CHECK_MESSAGE(bad == 0, bad << " outputs outside the bound, first at " << first);
}

void interpWritesExactlyTest() {
	uint32_t seed = 500;
	for (size_t L : {1u, 2u, 3u, 5u}) {
		for (size_t P : {1u, 2u, 4u, 5u}) {
			for (size_t B : kInBlocks) {
				const std::vector<float> c = randVec<DF32>(L * P, seed++, -0.5, 0.5);
				const std::vector<float> x = randVec<DF32>(B, seed++, -0.5, 0.5);
				const RunResult<DF32> r = interpRun(c, L, x, {B});
				CHECK_MESSAGE(r.guardsOk, "L " << L << " P " << P << " block " << B << ": guard behind the output or the state damaged");
				CHECK_MESSAGE(r.noneLeftPoison, "L " << L << " P " << P << " block " << B << ": an output element was not written");
				CHECK(r.out.size() == L * B);
			}
		}
	}
}

void interpContinuityTest() {
	const std::vector<std::vector<size_t>> plans = {{1}, {2}, {3}, {4}, {5}, {7}, {1, 2, 3, 4, 5, 7}, {8, 9, 16}, {9, 1, 17, 2}, {40}};
	uint32_t seed = 900;
	size_t bad = 0;
	std::string first;
	for (size_t L : {1u, 2u, 3u, 4u}) {
		for (size_t P : {1u, 2u, 3u, 4u, 5u, 9u}) {
			const std::vector<float> c = randVec<DF32>(L * P, seed++, -1.0, 1.0);
			const std::vector<float> x = randVec<DF32>(40, seed++, -1.0, 1.0);
			std::vector<double> y, sa;
			defInterp<DF32>(c, L, x, y, sa);
			for (size_t p = 0; p < plans.size(); p++) {
				const RunResult<DF32> r = interpRun(c, L, x, plans[p]);
				REQUIRE(r.out.size() == L * 40);
				CHECK(r.guardsOk);
				for (size_t n = 0; n < L * 40; n++) {
					const double e = std::fabs(static_cast<double>(r.out[n]) - y[n]);
					if (!(e <= bound<DF32>(P, sa[n]))) {
						if (bad == 0)
							first = "L " + std::to_string(L) + " P " + std::to_string(P) + " plan " + std::to_string(p) + " n " + std::to_string(n);
						bad++;
					}
				}
			}
		}
	}
	CHECK_MESSAGE(bad == 0, bad << " outputs outside the bound, first at " << first);
}

void interpIndependentInstancesTest() {
	const size_t La = 2, Pa = 3, Lb = 3, Pb = 4, B = 7, blocks = 6;
	const std::vector<float> ca = randVec<DF32>(La * Pa, 31, -1.0, 1.0), cb = randVec<DF32>(Lb * Pb, 32, -1.0, 1.0);
	const std::vector<float> xa = randVec<DF32>(B * blocks, 33, -1.0, 1.0), xb = randVec<DF32>(B * blocks, 34, -1.0, 1.0);
	dsp::GuardedBuffer<float> sa(Pa + B - 1), sb(Pb + B - 1);
	arm_fir_interpolate_instance_f32 ia, ib;
	REQUIRE(arm_fir_interpolate_init_f32(&ia, La, La * Pa, ca.data(), sa.data(), B) == ARM_MATH_SUCCESS);
	REQUIRE(interpFieldsStored(ia, La, La * Pa, ca.data(), sa.data()));
	REQUIRE(arm_fir_interpolate_init_f32(&ib, Lb, Lb * Pb, cb.data(), sb.data(), B) == ARM_MATH_SUCCESS);
	REQUIRE(interpFieldsStored(ib, Lb, Lb * Pb, cb.data(), sb.data()));
	std::vector<float> oa, ob;
	for (size_t k = 0; k < blocks; k++) {
		std::vector<float> pa(B * La), pb(B * Lb);
		arm_fir_interpolate_f32(&ia, xa.data() + k * B, pa.data(), B);
		arm_fir_interpolate_f32(&ib, xb.data() + k * B, pb.data(), B);
		oa.insert(oa.end(), pa.begin(), pa.end());
		ob.insert(ob.end(), pb.begin(), pb.end());
	}
	std::vector<double> ya, yb, sca, scb;
	defInterp<DF32>(ca, La, xa, ya, sca);
	defInterp<DF32>(cb, Lb, xb, yb, scb);
	size_t bad = 0;
	for (size_t n = 0; n < oa.size(); n++)
		if (!(std::fabs(static_cast<double>(oa[n]) - ya[n]) <= bound<DF32>(Pa, sca[n]))) bad++;
	for (size_t n = 0; n < ob.size(); n++)
		if (!(std::fabs(static_cast<double>(ob[n]) - yb[n]) <= bound<DF32>(Pb, scb[n]))) bad++;
	CHECK(bad == 0);
	CHECK(sa.guardIntact());
	CHECK(sb.guardIntact());
}

void interpReinitTest() {
	const size_t L = 3, P = 4, B = 5;
	const std::vector<float> c = randVec<DF32>(L * P, 41, -1.0, 1.0);
	const std::vector<float> x = randVec<DF32>(3 * B, 42, -1.0, 1.0);
	dsp::GuardedBuffer<float> state(P + B - 1);
	arm_fir_interpolate_instance_f32 inst;
	REQUIRE(arm_fir_interpolate_init_f32(&inst, L, L * P, c.data(), state.data(), B) == ARM_MATH_SUCCESS);
	REQUIRE(interpFieldsStored(inst, L, L * P, c.data(), state.data()));
	std::vector<float> first(3 * B * L), second(3 * B * L);
	for (size_t k = 0; k < 3; k++) arm_fir_interpolate_f32(&inst, x.data() + k * B, first.data() + k * B * L, B);
	std::memset(static_cast<void *>(state.data()), 0xCD, (P + B - 1) * sizeof(float));
	REQUIRE(arm_fir_interpolate_init_f32(&inst, L, L * P, c.data(), state.data(), B) == ARM_MATH_SUCCESS);
	REQUIRE(interpFieldsStored(inst, L, L * P, c.data(), state.data()));
	for (size_t k = 0; k < 3; k++) arm_fir_interpolate_f32(&inst, x.data() + k * B, second.data() + k * B * L, B);
	CHECK(std::memcmp(first.data(), second.data(), first.size() * sizeof(float)) == 0);
	CHECK(state.guardIntact());
}

void interpInitTest() {
	const size_t cfg[][3] = {{1, 1, 1}, {2, 3, 4}, {3, 4, 9}, {4, 2, 8}, {5, 5, 10}, {8, 1, 3}, {2, 9, 1}};  // L, P, B
	for (const auto &t : cfg) {
		const size_t L = t[0], P = t[1], B = t[2];
		const std::vector<float> c = randVec<DF32>(L * P, 61, -1.0, 1.0);
		dsp::GuardedBuffer<float> state(P + B - 1);  // poisoned with 0xCD by the buffer
		arm_fir_interpolate_instance_f32 inst;
		std::memset(&inst, 0x5A, sizeof inst);
		const arm_status st = arm_fir_interpolate_init_f32(&inst, static_cast<uint8_t>(L), static_cast<uint16_t>(L * P), c.data(), state.data(),
		                                                   static_cast<uint32_t>(B));
		CHECK(st == ARM_MATH_SUCCESS);
		CHECK(inst.L == L);
		CHECK(inst.phaseLength == P);
		CHECK(inst.pCoeffs == c.data());
		CHECK(inst.pState == state.data());
		bool zero = true;
		for (const float &v : state.output()) zero = zero && isZeroBytes(v);
		CHECK_MESSAGE(zero, "L " << L << " P " << P << " block " << B << ": state not zeroed");
		CHECK_MESSAGE(state.guardIntact(), "L " << L << " P " << P << " block " << B << ": init wrote behind the state buffer");
	}
}

// DOC (arm_fir_interpolate_init_f32.c, @return): "ARM_MATH_ARGUMENT_ERROR : filter length numTaps is not a multiple of the
// interpolation factor L". DOC says ARGUMENT_ERROR; the code and the decimate init return LENGTH_ERROR. NEEDS REVIEW (DOC defect
// or code defect, not decided). The test documents the code's value.
void interpInitErrorTest() {
	const size_t cfg[][2] = {{2, 3}, {2, 5}, {3, 4}, {4, 6}, {5, 11}, {3, 1}};  // L, numTaps
	for (const auto &t : cfg) {
		const size_t L = t[0], T = t[1];
		const std::vector<float> c = randVec<DF32>(T, 71, -1.0, 1.0);
		dsp::GuardedBuffer<float> state(T + 4);
		arm_fir_interpolate_instance_f32 inst;
		CHECK_MESSAGE(arm_fir_interpolate_init_f32(&inst, static_cast<uint8_t>(L), static_cast<uint16_t>(T), c.data(), state.data(), 4) ==
		                  ARM_MATH_LENGTH_ERROR,
		              "L " << L << " numTaps " << T);
	}
}

// Not stated by the DOC: on the error the instance and the state buffer are left as they were (well defined, harmless)
void interpInitErrorUnmodifiedTest() {
	const std::vector<float> c = randVec<DF32>(5, 72, -1.0, 1.0);
	dsp::GuardedBuffer<float> state(8);
	arm_fir_interpolate_instance_f32 inst, before;
	std::memset(&inst, 0x5A, sizeof inst);
	std::memcpy(&before, &inst, sizeof inst);
	CHECK(arm_fir_interpolate_init_f32(&inst, 2, 5, c.data(), state.data(), 4) != ARM_MATH_SUCCESS);
	CHECK(std::memcmp(&inst, &before, sizeof inst) == 0);
	bool untouched = true;
	for (const float &v : state.output()) untouched = untouched && isPoison(v);
	CHECK(untouched);
	CHECK(state.guardIntact());
}

void interpConstInputsTest() {
	const std::vector<float> c = randVec<DF32>(12, 91, -1.0, 1.0);
	const std::vector<float> x = randVec<DF32>(23, 92, -1.0, 1.0);
	std::vector<float> c2(c), x2(x);
	const RunResult<DF32> r = interpRun(c2, 3, x2, {5, 3, 7});
	CHECK(r.out.size() == 69);
	CHECK(std::memcmp(c.data(), c2.data(), c.size() * sizeof(float)) == 0);
	CHECK(std::memcmp(x.data(), x2.data(), x.size() * sizeof(float)) == 0);
}

// One tap per phase, L = 3 (numTaps 3, phaseLength 1): y[3n+i] = c[2-i] * x[n], correctly rounded products over the whole float range
void sweepInterpOneTap() {
	const float c[3] = {0.7f, -1.3f, 0.3f};
	const size_t L = 3, BLK = 2048;
	std::vector<float> x(BLK), y(BLK * L), state(BLK);  // phaseLength + blockSize - 1 = BLK
	arm_fir_interpolate_instance_f32 inst;
	REQUIRE(arm_fir_interpolate_init_f32(&inst, L, 3, c, state.data(), BLK) == ARM_MATH_SUCCESS);
	REQUIRE(interpFieldsStored(inst, L, 3, c, state.data()));
	size_t n = 0, bad = 0, tested = 0;
	uint32_t firstBad = 0;
	for (uint64_t bits = 0; bits < 0x100000000ull; bits += 4093ull) {
		const uint32_t b = static_cast<uint32_t>(bits);
		float f;
		std::memcpy(&f, &b, sizeof f);
		if (std::isnan(f)) continue;
		x[n++] = f;
		if (n == BLK) {
			arm_fir_interpolate_f32(&inst, x.data(), y.data(), BLK);
			for (size_t k = 0; k < BLK; k++)
				for (size_t i = 0; i < L; i++) {
					const float e = static_cast<float>(static_cast<double>(c[2 - i]) * static_cast<double>(x[k]));
					if (!sameProduct(y[L * k + i], e)) {
						if (bad == 0) std::memcpy(&firstBad, &x[k], sizeof firstBad);
						bad++;
					}
				}
			tested += BLK * L;
			n = 0;
		}
	}
	CHECK(tested > 3000000);
	CHECK_MESSAGE(bad == 0, bad << " products differ, first input bits 0x" << std::hex << firstBad);
}

// Two taps per phase, L = 2: against the DEF within the bound (N = 2) over 2^-57 .. 2^58
void sweepInterpTwoTap() {
	const std::vector<float> c = {0.7f, -1.3f, 0.45f, -0.85f};
	const size_t L = 2, BLK = 4096;
	std::vector<float> x(BLK), y(BLK * L), state(BLK + 1);
	arm_fir_interpolate_instance_f32 inst;
	REQUIRE(arm_fir_interpolate_init_f32(&inst, L, 4, c.data(), state.data(), BLK) == ARM_MATH_SUCCESS);
	REQUIRE(interpFieldsStored(inst, L, 4, c.data(), state.data()));
	size_t n = 0, bad = 0, tested = 0;
	for (uint64_t bits = 0; bits < DF32::span; bits += DF32::stride) {
		float v;
		if (!DF32::pick(bits, v)) continue;
		x[n++] = v;
		if (n == BLK) {
			arm_fir_interpolate_f32(&inst, x.data(), y.data(), BLK);
			std::vector<double> yd, sa;
			defInterp<DF32>(c, L, x, yd, sa);
			// the first input sample of a block (outputs 0 .. L-1) uses the history of the previous call: skipped
			for (size_t q = L; q < BLK * L; q++)
				if (!(std::fabs(static_cast<double>(y[q]) - yd[q]) <= bound<DF32>(2, sa[q]))) bad++;
			tested += BLK;
			n = 0;
		}
	}
	CHECK(tested >= 4096 * 20);
	CHECK_MESSAGE(bad == 0, bad << " outputs outside the bound");
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------
// arm_fir_decimate_f32
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_fir_decimate_f32] ARM's pattern test (72 configs) reaches SNR 120 and relative error 8e-4") {
	armDecimTest<DF32>(120.0, 8.0e-4);  // DECIMF32.cpp:5, :13
}
TEST_CASE("[C3 arm_fir_decimate_f32] ARM's reference pattern equals the difference equation of the DOC (y[m] is the FIR output at n = M*m)") {
	decimPatternIsDefTest<DF32>();
}
TEST_CASE("[C3 arm_fir_decimate_f32] the impulse response is the reversed coefficients sampled every M samples, for M 1..4 and 1 to 12 taps") { decimImpulseTest<DF32>(); }
TEST_CASE("[C3 arm_fir_decimate_f32] the step response is the partial sums of the coefficients sampled every M samples") { decimStepTest<DF32>(); }
TEST_CASE("[C3 arm_fir_decimate_f32] random data of M in {1,2,3,4,5,8}, 1..33 taps and 1..9 outputs is within n*eps*sum|terms| of the DEF") { decimRandomVsDefTest<DF32>(); }
TEST_CASE("[C2 arm_fir_decimate_f32] a call writes exactly blockSize/M outputs and nothing behind them or behind the exact size state") { decimWritesExactlyTest<DF32>(); }
TEST_CASE("[C4 arm_fir_decimate_f32] consecutive blocks of 1, 2, 3, 5 and mixed multiples of M carry the state like one long block") { decimContinuityTest<DF32>(); }
TEST_CASE("[C4 arm_fir_decimate_f32] two instances with different M and coefficients do not influence each other") { decimIndependentInstancesTest<DF32>(); }
TEST_CASE("[C4 arm_fir_decimate_f32] init again clears a garbage state: the run repeats bit for bit") { decimReinitTest<DF32>(); }
TEST_CASE("[C4 arm_fir_decimate_init_f32] init stores numTaps, M, pCoeffs, pState, returns success and zeroes exactly numTaps + blockSize - 1 elements") { decimInitTest<DF32>(); }
TEST_CASE("[C4 arm_fir_decimate_init_f32] init returns ARM_MATH_LENGTH_ERROR if blockSize is not a multiple of M") { decimInitErrorTest<DF32>(); }
TEST_CASE("[C4 arm_fir_decimate_init_f32] characterization: after the length error the instance and the state buffer are unchanged") { decimInitErrorUnmodifiedTest<DF32>(); }
TEST_CASE("[C4 arm_fir_decimate_f32] numTaps need not be a multiple of M (taps 5 with M 2 and 3 match the DEF)") { decimTapsNotMultipleOfMTest<DF32>(); }
TEST_CASE("[C4 arm_fir_decimate_f32] the coefficient and input arrays are not modified") { decimConstInputsTest<DF32>(); }
TEST_CASE("[C3 arm_fir_decimate_f32] a one tap decimator by 3 returns the correctly rounded product for strided inputs over the whole float range") { sweepDecimOneTapF32(); }
TEST_CASE("[C3 arm_fir_decimate_f32] a two tap decimator by 2 is within the bound of the DEF over 2^-57 .. 2^58, strided") { sweepDecimTwoTapTest<DF32>(); }

TEST_CASE("[C5 arm_fir_decimate_f32] the f32 result agrees with arm_fir_decimate_f64 on the same data within the f32 bound") {
	uint32_t seed = 2000;
	size_t bad = 0;
	for (size_t M : {1u, 3u}) {
		for (size_t T : kTaps) {
			const std::vector<float> c = randVec<DF32>(T, seed++, -1.0, 1.0);
			const std::vector<float> x = randVec<DF32>(30 * M, seed++, -1.0, 1.0);
			const std::vector<double> c64(c.begin(), c.end()), x64(x.begin(), x.end());
			const RunResult<DF32> r32 = decRun<DF32>(c, M, x, {30});
			const RunResult<DF64> r64 = decRun<DF64>(c64, M, x64, {30});
			std::vector<double> y, sa;
			defDecim<DF32>(c, M, x, y, sa);
			for (size_t n = 0; n < 30; n++) {
				// both within their own bound of the DEF, so within the sum of the two of each other
				const double b = bound<DF32>(T, sa[n]) + bound<DF64>(T, sa[n]);
				if (!(std::fabs(static_cast<double>(r32.out[n]) - r64.out[n]) <= b)) bad++;
			}
		}
	}
	CHECK(bad == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// arm_fir_decimate_f64
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_fir_decimate_f64] ARM's pattern test (72 configs) reaches SNR 120 and relative error 8e-4") {
	armDecimTest<DF64>(120.0, 8.0e-4);  // DECIMF64.cpp:5, :13
}
TEST_CASE("[C3 arm_fir_decimate_f64] ARM's reference pattern equals the difference equation of the DOC (y[m] is the FIR output at n = M*m)") {
	decimPatternIsDefTest<DF64>();
}
TEST_CASE("[C3 arm_fir_decimate_f64] the impulse response is the reversed coefficients sampled every M samples, for M 1..4 and 1 to 12 taps") { decimImpulseTest<DF64>(); }
TEST_CASE("[C3 arm_fir_decimate_f64] the step response is the partial sums of the coefficients sampled every M samples") { decimStepTest<DF64>(); }
TEST_CASE("[C3 arm_fir_decimate_f64] random data of M in {1,2,3,4,5,8}, 1..33 taps and 1..9 outputs is within n*eps*sum|terms| of the DEF") { decimRandomVsDefTest<DF64>(); }
TEST_CASE("[C2 arm_fir_decimate_f64] a call writes exactly blockSize/M outputs and nothing behind them or behind the exact size state") { decimWritesExactlyTest<DF64>(); }
TEST_CASE("[C4 arm_fir_decimate_f64] consecutive blocks of 1, 2, 3, 5 and mixed multiples of M carry the state like one long block") { decimContinuityTest<DF64>(); }
TEST_CASE("[C4 arm_fir_decimate_f64] two instances with different M and coefficients do not influence each other") { decimIndependentInstancesTest<DF64>(); }
TEST_CASE("[C4 arm_fir_decimate_f64] init again clears a garbage state: the run repeats bit for bit") { decimReinitTest<DF64>(); }
TEST_CASE("[C4 arm_fir_decimate_init_f64] init stores numTaps, M, pCoeffs, pState, returns success and zeroes exactly numTaps + blockSize - 1 elements") { decimInitTest<DF64>(); }
TEST_CASE("[C4 arm_fir_decimate_init_f64] init returns ARM_MATH_LENGTH_ERROR if blockSize is not a multiple of M") { decimInitErrorTest<DF64>(); }
TEST_CASE("[C4 arm_fir_decimate_init_f64] characterization: after the length error the instance and the state buffer are unchanged") { decimInitErrorUnmodifiedTest<DF64>(); }
TEST_CASE("[C4 arm_fir_decimate_f64] numTaps need not be a multiple of M (taps 5 with M 2 and 3 match the DEF)") { decimTapsNotMultipleOfMTest<DF64>(); }
TEST_CASE("[C4 arm_fir_decimate_f64] the coefficient and input arrays are not modified") { decimConstInputsTest<DF64>(); }
TEST_CASE("[C3 arm_fir_decimate_f64] a one tap decimator by 3 returns the IEEE product for strided inputs over the whole double range") { sweepDecimOneTapF64(); }
TEST_CASE("[C3 arm_fir_decimate_f64] a two tap decimator by 2 is within the bound of the DEF over 2^-423 .. 2^377, strided") { sweepDecimTwoTapTest<DF64>(); }

// ---------------------------------------------------------------------------------------------------------------
// arm_fir_interpolate_f32
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("[C1 arm_fir_interpolate_f32] ARM's pattern test (36 configs) reaches SNR 120 and relative error 8e-4") {
	armInterpTest(120.0, 8.0e-4);  // DECIMF32.cpp:5, :13
}
TEST_CASE("[C3 arm_fir_interpolate_f32] ARM's reference pattern equals the zero stuffing plus FIR of the DOC") { interpPatternIsDefTest(); }
TEST_CASE("[C3 arm_fir_interpolate_f32] the impulse response is the coefficients in reverse order, for L 1..4 and phase length 1 to 9") { interpImpulseTest(); }
TEST_CASE("[C3 arm_fir_interpolate_f32] the step response of each phase is the running sum of its coefficients") { interpStepTest(); }
TEST_CASE("[C3 arm_fir_interpolate_f32] random data of L in {1,2,3,4,5,8}, phase length 1..9 and blocks 1..17 is within n*eps*sum|terms| of the DEF") { interpRandomVsDefTest(); }
TEST_CASE("[C2 arm_fir_interpolate_f32] a call writes exactly blockSize*L outputs and nothing behind them or behind the exact size state") { interpWritesExactlyTest(); }
TEST_CASE("[C4 arm_fir_interpolate_f32] consecutive blocks of sizes 1, 2, 3, 4, 5, 7 and mixed carry the state like one long block") { interpContinuityTest(); }
TEST_CASE("[C4 arm_fir_interpolate_f32] two instances with different L and coefficients do not influence each other") { interpIndependentInstancesTest(); }
TEST_CASE("[C4 arm_fir_interpolate_f32] init again clears a garbage state: the run repeats bit for bit") { interpReinitTest(); }
TEST_CASE("[C4 arm_fir_interpolate_init_f32] init stores L, phaseLength = numTaps/L, pCoeffs, pState, returns success and zeroes exactly phaseLength + blockSize - 1 elements") { interpInitTest(); }
TEST_CASE("[C4 arm_fir_interpolate_init_f32] characterization: init returns ARM_MATH_LENGTH_ERROR if numTaps is not a multiple of L (DOC says ARGUMENT_ERROR)") { interpInitErrorTest(); }
TEST_CASE("[C4 arm_fir_interpolate_init_f32] characterization: after the error for numTaps not a multiple of L the instance and the state buffer are unchanged") { interpInitErrorUnmodifiedTest(); }
TEST_CASE("[C4 arm_fir_interpolate_f32] the coefficient and input arrays are not modified") { interpConstInputsTest(); }
TEST_CASE("[C3 arm_fir_interpolate_f32] a one tap per phase interpolator by 3 returns the correctly rounded products for strided inputs over the whole float range") { sweepInterpOneTap(); }
TEST_CASE("[C3 arm_fir_interpolate_f32] a two taps per phase interpolator by 2 is within the bound of the DEF over 2^-57 .. 2^58, strided") { sweepInterpTwoTap(); }
