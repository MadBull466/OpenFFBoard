/*
 * CMSIS-DSP Filtering, biquad cascades f32 (work order FB32, phase P11 group G4). Requirement ids:
 * Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * Units covered: arm_biquad_cascade_df1_f32 / _init_f32, arm_biquad_cascade_df2T_f32 / _init_f32,
 * arm_biquad_cascade_stereo_df2T_f32 / _init_f32 (Source/FilteringFunctions/arm_biquad_cascade_*_f32.c).
 *
 * Pattern files used (Libraries/CMSIS-DSP/Testing/Patterns/DSP/Filtering/BIQUAD/BIQUADF32/); ARM's test functions are in
 * Testing/Source/Tests/BIQUADF32.cpp, the suite "BIQUAD F32" is in Testing/desc.txt:3268 (functions at :3287-:3291):
 *   df1  ref   (BIQUADF32.cpp:14)  : BiquadInput1 (100) BiquadCoefs1 (15 = 3 stages) -> BiquadOutput1 (100)
 *                                    3 stages, blockSize 50, two consecutive calls (state continuity)
 *   df2T ref   (:94)               : the same patterns
 *   df1  rand  (:176)              : AllBiquadInputs2 (66) AllBiquadCoefs2 (330) AllBiquadConfigs2 (s16 pairs
 *                                    numStages, blockSize: stages {3,8,11} x blocks {3,8,11}) -> AllBiquadRefs2 (66)
 *   df2T rand  (:253)              : the same patterns
 *   stereo df2T rand (:335)        : AllBiquadStereoInputs2 (132 = interleaved) -> AllBiquadStereoRefs2 (132)
 * All 5 ARM test ids are covered. Nothing is disabled except the Neon / MVE code paths, which are not compiled on the
 * host (BIQUADF32.cpp:103-:117 and :280-:289 are "#if 0" Neon variants, arm_biquad_cascade_df1_mve_init_f32 only exists
 * with MVE). ARM's patterns are "generated with a double precision computation" (BIQUADF32.cpp:8-:12, scipy sosfilt).
 * ARM also dumps the df1 state after each block (debugstate, :55 and :65) but never asserts it: the state is tested here
 * against the DEF.
 *
 * ARM tolerances (BIQUADF32.cpp:5 and :13): SNR_THRESHOLD 94 dB and REL_ERROR 2.0e-4. ARM's ASSERT_REL_ERROR
 * (Testing/FrameworkSource/Error.cpp:213) is |a-b| / ((|a|+|b|)/2) <= 2e-4 per sample (skipped if both are 0); it is
 * reproduced exactly by armRel() below (not dsp::closeAbsRel, whose reference is |ref|), with the one difference that
 * NaN fails here (ARM's comparison would let it pass). The SNR is computed in double (dsp::snr).
 *
 * DEF (C3): the difference equation in double with ARM's sign convention (DOC arm_biquad_cascade_df1_f32.c:44-47:
 * y[n] = b0 x[n] + b1 x[n-1] + b2 x[n-2] + a1 y[n-1] + a2 y[n-2], the feedback coefficients are NOT negated), df2T as the
 * transposed recursion y = b0 x + d1, d1 = b1 x + a1 y + d2, d2 = b2 x + a2 y (arm_biquad_cascade_df2T_f32.c:81-83); both
 * have the same transfer function. Coefficient order {b0, b1, b2, a1, a2} per stage (init DOC).
 *
 * Tolerance of the DEF comparisons (derived, no magic number): a running first-order error analysis follows the float
 * algorithm sample by sample. Let u = 2^-24 (unit roundoff of binary32, round to nearest) and gamma_k = k u / (1 - k u).
 * A sum of k products fl(c1 t1 + ... + ck tk) evaluated left to right has an error of at most
 * gamma_k * sum|ci ti| plus the propagated error sum|ci| e(ti) of its operands (each term takes one rounding for the
 * product and at most k-1 for the additions). df1: k = 5 (five products), so
 *   e(y[n]) <= |b0| e(x[n]) + |b1| e(x[n-1]) + |b2| e(x[n-2]) + |a1| e(y[n-1]) + |a2| e(y[n-2]) + gamma_5 * S[n],
 *   S[n] = |b0 x[n]| + |b1 x[n-1]| + |b2 x[n-2]| + |a1 y[n-1]| + |a2 y[n-2]|.
 * df2T (code order: acc = b0 x + d1; d1 = (b1 x + d2) + a1 acc; d2 = b2 x + a2 acc): the output has 2 roundings per
 * term (gamma_2), d1 has up to 3 (gamma_3), d2 has 2 (gamma_2); the same bound is valid for the stereo kernel, which adds
 * in a different order but with the same counts. The input of stage 1 is exact (floats), the double DEF itself adds
 * 10 * 2^-53 * S per sample for its own rounding; second order terms (u^2) are neglected. The bound of a stage is the
 * input error bound of the next. The test fails if |out - DEF| exceeds the bound at any sample: any wrong
 * coefficient, sign, state slot or stage order is many orders of magnitude outside it (the tightness test below checks
 * that the bound stays below 1e-3 of the peak output for 1..4 stages).
 * The DEF filters are stable on purpose (see genCoefs: |a1| <= 0.5, |a2| <= 0.25, so |a1|+|a2| < 1 and the error
 * gain of a stage is at most 4), so the double DEF is well conditioned. The bound grows with the number of stages
 * (the tightness test covers 1..4 stages).
 *
 * Host vs target (C6): the host runs the portable C path (ARM_MATH_LOOPUNROLL, no ARM_MATH_DSP, no MVE / Neon); the
 * kernels unroll by 4 (df1), 16 (df2T) and 8 frames (stereo df2T), so block sizes 1..40 are swept. IEEE binary32 on
 * both. On the target a compiler may contract a*b+c into a fused multiply-add (differences of one rounding, inside the
 * bounds, which allow gamma_k); therefore no test compares two different block partitions bit for bit: they compare
 * each against the DEF bound, and use exact comparison only where the arithmetic is exact (dyadic hand values,
 * power-of-two scaling, a single product rounded once). The range sweeps rely on denormals being kept (host default; the
 * target's FPSCR flush-to-zero bit is off by default). DOC typo: the stereo init DOC text
 * (arm_biquad_cascade_stereo_df2T_init_f32.c:48) says the state has 2*numStages values, the header
 * (filtering_functions.h:1201) and the code (memset of 4*numStages) say 4*numStages: 4*numStages is used here.
 * Not tested (DOC silent, would run the loop 2^32 times): numStages == 0 (the do-while loops decrement an unsigned
 * counter from 0), listed as NEEDS REVIEW in the report.
 */
#include "doctest.h"

#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "DspPatterns.h"
#include "arm_math.h"

namespace {

const char *const kDir = "Filtering/BIQUAD/BIQUADF32/";

// ARM tolerances (BIQUADF32.cpp:5, :13)
const double kSnr = 94.0;
const double kRel = 2.0e-4;

const double kU = 1.0 / 16777216.0;   // 2^-24

double gam(int k) { return k * kU / (1.0 - k * kU); }

uint32_t bitsOf(float f) { uint32_t b; std::memcpy(&b, &f, sizeof b); return b; }
float floatOfBits(uint32_t b) { float f; std::memcpy(&f, &b, sizeof f); return f; }

// ARM ASSERT_REL_ERROR for float (Error.cpp:213): delta / average of the magnitudes, average computed in float.
std::string armRel(const std::vector<float> &ref, const std::vector<float> &out, double thr) {
	if (ref.size() != out.size()) return "length mismatch";
	for (size_t i = 0; i < ref.size(); i++) {
		float a = out[i], b = ref[i];
		double delta = static_cast<double>(std::fabs(a - b));
		double average = static_cast<double>((std::fabs(a) + std::fabs(b)) / 2.0f);
		bool bad = std::isnan(a) || std::isnan(b) || (average != 0.0 && delta / average > thr);
		if (bad) return "rel error at " + std::to_string(i) + ": out " + std::to_string(a) + " ref " + std::to_string(b);
	}
	return "";
}

// Buffer with 8 guard floats before and after the body; the body is poisoned with 0xCD bytes (so a missing clear or a
// missing write is visible), the guards are 0xA5 bytes. Used for state buffers of EXACTLY the documented size.
struct Padded {
	static const size_t kG = 8;
	std::vector<float> v;
	size_t n;
	explicit Padded(size_t count) : v(count + 2 * kG), n(count) {
		std::memset(static_cast<void *>(v.data()), 0xA5, v.size() * sizeof(float));
		if (n) std::memset(static_cast<void *>(v.data() + kG), 0xCD, n * sizeof(float));
	}
	float *body() { return v.data() + kG; }
	std::vector<float> contents() { return std::vector<float>(body(), body() + n); }
	bool guardsIntact() const {
		for (size_t i = 0; i < kG; i++)
			if (bitsOf(v[i]) != 0xA5A5A5A5u || bitsOf(v[kG + n + i]) != 0xA5A5A5A5u) return false;
		return true;
	}
	bool bodyAllZero() const {
		for (size_t i = 0; i < n; i++)
			if (bitsOf(v[kG + i]) != 0) return false;
		return true;
	}
};

enum Kind { DF1, DF2T, STEREO };
unsigned channels(Kind k) { return k == STEREO ? 2 : 1; }
// documented state lengths: 4*numStages (df1), 2*numStages (df2T), 4*numStages (stereo, header and code)
size_t stateLen(Kind k, unsigned ns) { return (k == DF2T ? 2u : 4u) * ns; }

// One filter instance of any of the three kinds with a state buffer of exactly the documented size.
struct Filt {
	Kind k;
	unsigned ns;
	const float *coefs;
	Padded state;                 // guarded variant (guards in the same allocation: only the guard check sees an overrun)
	std::vector<float> tight;     // tight variant: a fresh heap block of exactly the documented length, no slack,
	                              // so AddressSanitizer reports any access beyond the documented state length
	bool guarded;
	arm_biquad_casd_df1_inst_f32 s1;
	arm_biquad_cascade_df2T_instance_f32 s2;
	arm_biquad_cascade_stereo_df2T_instance_f32 s3;
	Filt(Kind kind, unsigned stages, const float *c, bool guardedState = false)
	    : k(kind), ns(stages), coefs(c), state(guardedState ? stateLen(kind, stages) : 0), tight(guardedState ? 0 : stateLen(kind, stages)),
	      guarded(guardedState) {
		if (!guarded) std::memset(static_cast<void *>(tight.data()), 0xCD, tight.size() * sizeof(float));   // poison
		std::memset(&s1, 0, sizeof s1);
		std::memset(&s2, 0, sizeof s2);
		std::memset(&s3, 0, sizeof s3);
		init();
	}
	void init() {
		if (k == DF1) arm_biquad_cascade_df1_init_f32(&s1, static_cast<uint8_t>(ns), coefs, st());
		else if (k == DF2T) arm_biquad_cascade_df2T_init_f32(&s2, static_cast<uint8_t>(ns), coefs, st());
		else arm_biquad_cascade_stereo_df2T_init_f32(&s3, static_cast<uint8_t>(ns), coefs, st());
	}
	void process(const float *in, float *out, uint32_t frames) {
		if (k == DF1) arm_biquad_cascade_df1_f32(&s1, in, out, frames);
		else if (k == DF2T) arm_biquad_cascade_df2T_f32(&s2, in, out, frames);
		else arm_biquad_cascade_stereo_df2T_f32(&s3, in, out, frames);
	}
	float *st() { return guarded ? state.body() : tight.data(); }
	std::vector<float> contents() { return guarded ? state.contents() : tight; }
	bool allZero() { for (float v : contents()) if (bitsOf(v) != 0) return false; return true; }
	bool guardsOk() const { return !guarded || state.guardsIntact(); }
};

// Processes `in` (frames * channels floats) in consecutive blocks whose sizes cycle through `chunks` (frames).
std::vector<float> runChunks(Filt &f, const std::vector<float> &in, const std::vector<uint32_t> &chunks) {
	unsigned ch = channels(f.k);
	size_t frames = in.size() / ch;
	std::vector<float> out(in.size());
	size_t pos = 0, ci = 0;
	while (pos < frames) {
		size_t n = chunks[ci % chunks.size()];
		ci++;
		if (n > frames - pos) n = frames - pos;
		f.process(in.data() + pos * ch, out.data() + pos * ch, static_cast<uint32_t>(n));
		pos += n;
	}
	return out;
}
std::vector<float> runOnce(Filt &f, const std::vector<float> &in) {
	return runChunks(f, in, {static_cast<uint32_t>(in.size() / channels(f.k))});
}

// ---------------------------------------------------------------------------------------------------------------
// DEF in double with the running error bound (see the header comment)
// ---------------------------------------------------------------------------------------------------------------
struct Ref {
	std::vector<double> y, e;     // output and its error bound (interleaved for stereo)
	std::vector<double> st, ste;  // final state (layout of the kernel) and its error bound
};

const double kD = 10.0 * (1.0 / 9007199254740992.0);   // 10 * 2^-53, rounding of the double DEF itself

Ref refDf1(const std::vector<float> &c, unsigned ns, const std::vector<float> &xin) {
	size_t n = xin.size();
	std::vector<double> x(xin.begin(), xin.end()), ex(n, 0.0);
	Ref r;
	for (unsigned s = 0; s < ns; s++) {
		double b0 = c[5 * s], b1 = c[5 * s + 1], b2 = c[5 * s + 2], a1 = c[5 * s + 3], a2 = c[5 * s + 4];
		std::vector<double> y(n), ey(n);
		double x1 = 0, x2 = 0, y1 = 0, y2 = 0, ex1 = 0, ex2 = 0, ey1 = 0, ey2 = 0;
		for (size_t i = 0; i < n; i++) {
			double S = std::fabs(b0 * x[i]) + std::fabs(b1 * x1) + std::fabs(b2 * x2) + std::fabs(a1 * y1) + std::fabs(a2 * y2);
			double v = b0 * x[i] + b1 * x1 + b2 * x2 + a1 * y1 + a2 * y2;
			double e = std::fabs(b0) * ex[i] + std::fabs(b1) * ex1 + std::fabs(b2) * ex2 + std::fabs(a1) * ey1 + std::fabs(a2) * ey2 +
			           (gam(5) + kD) * S;
			y[i] = v; ey[i] = e;
			x2 = x1; x1 = x[i]; ex2 = ex1; ex1 = ex[i];
			y2 = y1; y1 = v; ey2 = ey1; ey1 = e;
		}
		// state layout {x[n-1], x[n-2], y[n-1], y[n-2]}
		r.st.push_back(x1); r.st.push_back(x2); r.st.push_back(y1); r.st.push_back(y2);
		r.ste.push_back(ex1); r.ste.push_back(ex2); r.ste.push_back(ey1); r.ste.push_back(ey2);
		x = y; ex = ey;
	}
	r.y = x; r.e = ex;
	return r;
}

// df2T of one channel; state per stage {d1, d2}
Ref refDf2T(const std::vector<float> &c, unsigned ns, const std::vector<float> &xin) {
	size_t n = xin.size();
	std::vector<double> x(xin.begin(), xin.end()), ex(n, 0.0);
	Ref r;
	for (unsigned s = 0; s < ns; s++) {
		double b0 = c[5 * s], b1 = c[5 * s + 1], b2 = c[5 * s + 2], a1 = c[5 * s + 3], a2 = c[5 * s + 4];
		std::vector<double> y(n), ey(n);
		double d1 = 0, d2 = 0, e1 = 0, e2 = 0;
		for (size_t i = 0; i < n; i++) {
			double acc = b0 * x[i] + d1;
			double eacc = std::fabs(b0) * ex[i] + e1 + (gam(2) + kD) * (std::fabs(b0 * x[i]) + std::fabs(d1));
			double nd1 = b1 * x[i] + d2 + a1 * acc;
			double ne1 = std::fabs(b1) * ex[i] + e2 + std::fabs(a1) * eacc +
			             (gam(3) + kD) * (std::fabs(b1 * x[i]) + std::fabs(d2) + std::fabs(a1 * acc));
			double nd2 = b2 * x[i] + a2 * acc;
			double ne2 = std::fabs(b2) * ex[i] + std::fabs(a2) * eacc + (gam(2) + kD) * (std::fabs(b2 * x[i]) + std::fabs(a2 * acc));
			y[i] = acc; ey[i] = eacc;
			d1 = nd1; e1 = ne1; d2 = nd2; e2 = ne2;
		}
		r.st.push_back(d1); r.st.push_back(d2);
		r.ste.push_back(e1); r.ste.push_back(e2);
		x = y; ex = ey;
	}
	r.y = x; r.e = ex;
	return r;
}

Ref refStereo(const std::vector<float> &c, unsigned ns, const std::vector<float> &in) {
	size_t n = in.size() / 2;
	std::vector<float> a(n), b(n);
	for (size_t i = 0; i < n; i++) { a[i] = in[2 * i]; b[i] = in[2 * i + 1]; }
	Ref ra = refDf2T(c, ns, a), rb = refDf2T(c, ns, b), r;
	for (size_t i = 0; i < n; i++) {
		r.y.push_back(ra.y[i]); r.y.push_back(rb.y[i]);
		r.e.push_back(ra.e[i]); r.e.push_back(rb.e[i]);
	}
	// per stage {d1a, d2a, d1b, d2b}
	for (unsigned s = 0; s < ns; s++) {
		r.st.push_back(ra.st[2 * s]); r.st.push_back(ra.st[2 * s + 1]); r.st.push_back(rb.st[2 * s]); r.st.push_back(rb.st[2 * s + 1]);
		r.ste.push_back(ra.ste[2 * s]); r.ste.push_back(ra.ste[2 * s + 1]); r.ste.push_back(rb.ste[2 * s]); r.ste.push_back(rb.ste[2 * s + 1]);
	}
	return r;
}

Ref refFor(Kind k, const std::vector<float> &c, unsigned ns, const std::vector<float> &in) {
	return k == DF1 ? refDf1(c, ns, in) : (k == DF2T ? refDf2T(c, ns, in) : refStereo(c, ns, in));
}

// |got - ref| <= bound for every element; returns "" or a message
std::string withinBound(const std::vector<float> &got, const std::vector<double> &ref, const std::vector<double> &bound) {
	if (got.size() != ref.size()) return "length mismatch";
	for (size_t i = 0; i < got.size(); i++) {
		double d = std::fabs(static_cast<double>(got[i]) - ref[i]);
		if (!(d <= bound[i]))
			return "index " + std::to_string(i) + ": got " + std::to_string(got[i]) + " DEF " + std::to_string(ref[i]) +
			       " bound " + std::to_string(bound[i]);
	}
	return "";
}

// ---------------------------------------------------------------------------------------------------------------
// Test data
// ---------------------------------------------------------------------------------------------------------------
struct Rng {
	std::mt19937 g;
	explicit Rng(uint32_t seed) : g(seed) {}
	double u() { return static_cast<double>(g() >> 8) / 16777216.0; }   // [0, 1), ONE generator call per use
};

// Stable stages with |a1| <= 0.5 and |a2| <= 0.25 (ARM sign convention: plus). Stable because |a2| < 1 and
// |a1| < 1 - a2; |a1| + |a2| <= 0.75 < 1 also keeps the running error recursion of the DEF bound convergent (its
// error gain is at most 1/(1 - 0.75) = 4 per stage). Numerators in [-0.5, 0.5].
std::vector<float> genCoefs(Rng &rng, unsigned ns) {
	std::vector<float> c;
	for (unsigned s = 0; s < ns; s++) {
		double n0 = rng.u() - 0.5;
		double n1 = rng.u() - 0.5;
		double n2 = rng.u() - 0.5;
		double a1 = rng.u() - 0.5;
		double a2 = 0.5 * (rng.u() - 0.5);
		c.push_back(static_cast<float>(n0));
		c.push_back(static_cast<float>(n1));
		c.push_back(static_cast<float>(n2));
		c.push_back(static_cast<float>(a1));
		c.push_back(static_cast<float>(a2));
	}
	return c;
}
std::vector<float> genSignal(Rng &rng, size_t n) {
	std::vector<float> x;
	for (size_t i = 0; i < n; i++) x.push_back(static_cast<float>(2.0 * rng.u() - 1.0));
	return x;
}

// Hand-derivable stage used by several tests: b = {1, 2, 3}, a1 = 0.5, a2 = -0.25 (poles of radius 0.5, stable)
const float kHand[5] = {1.0f, 2.0f, 3.0f, 0.5f, -0.25f};
// Impulse response by hand (y[n] = 1*d + 2 x[n-1] + 3 x[n-2] + 0.5 y[n-1] - 0.25 y[n-2]):
//  h0 = 1; h1 = 2 + 0.5*1 = 2.5; h2 = 3 + 0.5*2.5 - 0.25*1 = 4; h3 = 0.5*4 - 0.25*2.5 = 1.375;
//  h4 = 0.5*1.375 - 0.25*4 = -0.3125; h5 = 0.5*(-0.3125) - 0.25*1.375 = -0.5; h6 = 0.5*(-0.5) - 0.25*(-0.3125) = -0.171875;
//  h7 = 0.5*(-0.171875) - 0.25*(-0.5) = 0.0390625. All dyadic with at most 12 fractional bits: exact in binary32.
const float kHandImpulse[8] = {1.0f, 2.5f, 4.0f, 1.375f, -0.3125f, -0.5f, -0.171875f, 0.0390625f};
// Step response = running sum of h (linear time invariant system): 1, 3.5, 7.5, 8.875, 8.5625, 8.0625, 7.890625, 7.9296875
// (also exact); the steady state is (b0+b1+b2)/(1-a1-a2) = 6 / 0.75 = 8.
const float kHandStep[8] = {1.0f, 3.5f, 7.5f, 8.875f, 8.5625f, 8.0625f, 7.890625f, 7.9296875f};

std::vector<float> impulse(Kind k, size_t frames) {
	std::vector<float> x(frames * channels(k), 0.0f);
	x[0] = 1.0f;   // channel a of the first frame
	return x;
}

}  // namespace

TEST_SUITE("DSP Filtering f32") {

// ---------------------------------------------------------------------------------------------------------------
// C1: ARM's tests
// ---------------------------------------------------------------------------------------------------------------

namespace {

// BIQUADF32.cpp:14 / :94: 3 stages, blockSize = nSamples/2, two consecutive calls on one instance
void armRefTest(Kind k) {
	auto in = dsp::loadPattern(std::string(kDir) + "BiquadInput1_f32.txt").f32();
	auto coefs = dsp::loadPattern(std::string(kDir) + "BiquadCoefs1_f32.txt").f32();
	auto ref = dsp::loadPattern(std::string(kDir) + "BiquadOutput1_f32.txt").f32();
	REQUIRE(in.size() == ref.size());
	REQUIRE(coefs.size() == 15);
	uint32_t blockSize = static_cast<uint32_t>(in.size() >> 1);
	Filt f(k, 3, coefs.data());
	dsp::GuardedBuffer<float> out(ref.size());
	f.process(in.data(), out.data(), blockSize);
	f.process(in.data() + blockSize, out.data() + blockSize, blockSize);
	CHECK(out.guardIntact());
	CHECK(f.guardsOk());
	std::vector<float> o = out.output();
	CHECK(dsp::snr(ref, o) >= kSnr);
	std::string m = armRel(ref, o, kRel);
	CHECK_MESSAGE(m.empty(), m);
}

// BIQUADF32.cpp:176 / :253 / :335: one filter per config pair (numStages, blockSize), consecutive coefficients / input
void armRandTest(Kind k) {
	const char *inName = k == STEREO ? "AllBiquadStereoInputs2_f32.txt" : "AllBiquadInputs2_f32.txt";
	const char *refName = k == STEREO ? "AllBiquadStereoRefs2_f32.txt" : "AllBiquadRefs2_f32.txt";
	auto in = dsp::loadPattern(std::string(kDir) + inName).f32();
	auto coefs = dsp::loadPattern(std::string(kDir) + "AllBiquadCoefs2_f32.txt").f32();
	auto cfg = dsp::loadPattern(std::string(kDir) + "AllBiquadConfigs2_s16.txt").q15();
	auto ref = dsp::loadPattern(std::string(kDir) + refName).f32();
	REQUIRE(in.size() == ref.size());
	REQUIRE(cfg.size() == 18);
	dsp::GuardedBuffer<float> out(ref.size());
	size_t ip = 0, cp = 0;
	bool statesOk = true;
	for (size_t i = 0; i < cfg.size(); i += 2) {
		unsigned ns = static_cast<unsigned>(cfg[i]);
		uint32_t bs = static_cast<uint32_t>(cfg[i + 1]);
		REQUIRE(cp + 5 * ns <= coefs.size());
		REQUIRE(ip + channels(k) * bs <= in.size());
		Filt f(k, ns, coefs.data() + cp);
		f.process(in.data() + ip, out.data() + ip, bs);
		if (!f.guardsOk()) statesOk = false;
		ip += channels(k) * bs;
		cp += 5 * ns;
	}
	CHECK(ip == in.size());
	CHECK(cp == coefs.size());
	CHECK(statesOk);
	CHECK(out.guardIntact());
	std::vector<float> o = out.output();
	CHECK(dsp::snr(ref, o) >= kSnr);
	std::string m = armRel(ref, o, kRel);
	CHECK_MESSAGE(m.empty(), m);
}

}  // namespace

TEST_CASE("[C1 arm_biquad_cascade_df1_f32] ARM's ref test (3 stages, 2 blocks of 50) matches the reference within 94 dB and 2e-4 relative") {
	armRefTest(DF1);
}
TEST_CASE("[C1 arm_biquad_cascade_df2T_f32] ARM's ref test (3 stages, 2 blocks of 50) matches the reference within 94 dB and 2e-4 relative") {
	armRefTest(DF2T);
}
TEST_CASE("[C1 arm_biquad_cascade_df1_f32] ARM's rand test (stages 3/8/11 x blocks 3/8/11) matches the reference within 94 dB and 2e-4 relative") {
	armRandTest(DF1);
}
TEST_CASE("[C1 arm_biquad_cascade_df2T_f32] ARM's rand test (stages 3/8/11 x blocks 3/8/11) matches the reference within 94 dB and 2e-4 relative") {
	armRandTest(DF2T);
}
TEST_CASE("[C1 arm_biquad_cascade_stereo_df2T_f32] ARM's stereo rand test (stages 3/8/11 x blocks 3/8/11) matches the reference within 94 dB and 2e-4 relative") {
	armRandTest(STEREO);
}

// ---------------------------------------------------------------------------------------------------------------
// C3: DEF (hand values, double DEF with derived bound)
// ---------------------------------------------------------------------------------------------------------------

namespace {

// Impulse / step through the hand stage for channel a (stereo: channel b stays 0, so it must stay exactly 0)
void handResponse(Kind k, bool step) {
	Filt f(k, 1, kHand);
	unsigned ch = channels(k);
	std::vector<float> in(8 * ch, 0.0f);
	if (step) { for (size_t i = 0; i < 8; i++) in[i * ch] = 1.0f; } else in[0] = 1.0f;
	std::vector<float> out = runOnce(f, in);
	const float *want = step ? kHandStep : kHandImpulse;
	for (size_t i = 0; i < 8; i++) {
		CAPTURE(i);
		CHECK(out[i * ch] == want[i]);
		if (ch == 2) CHECK(out[i * 2 + 1] == 0.0f);
	}
}

}  // namespace

TEST_CASE("[C3 arm_biquad_cascade_df1_f32] the impulse response of a hand stage is 1, 2.5, 4, 1.375, -0.3125, -0.5, -0.171875, 0.0390625 (plus sign of a1, a2)") {
	handResponse(DF1, false);
}
TEST_CASE("[C3 arm_biquad_cascade_df2T_f32] the impulse response of a hand stage is 1, 2.5, 4, 1.375, -0.3125, -0.5, -0.171875, 0.0390625 (plus sign of a1, a2)") {
	handResponse(DF2T, false);
}
TEST_CASE("[C3 arm_biquad_cascade_stereo_df2T_f32] the impulse response of a hand stage on channel a is 1, 2.5, 4, 1.375, -0.3125, -0.5, -0.171875, 0.0390625 and channel b stays 0") {
	handResponse(STEREO, false);
}
TEST_CASE("[C3 arm_biquad_cascade_df1_f32] the step response of the hand stage is 1, 3.5, 7.5, 8.875, 8.5625, 8.0625, 7.890625, 7.9296875") {
	handResponse(DF1, true);
}
TEST_CASE("[C3 arm_biquad_cascade_df2T_f32] the step response of the hand stage is 1, 3.5, 7.5, 8.875, 8.5625, 8.0625, 7.890625, 7.9296875") {
	handResponse(DF2T, true);
}
TEST_CASE("[C3 arm_biquad_cascade_stereo_df2T_f32] the step response of the hand stage on channel a is 1, 3.5, 7.5, 8.875, 8.5625, 8.0625, 7.890625, 7.9296875 and channel b stays 0") {
	handResponse(STEREO, true);
}

namespace {

// DC gain of the hand stage: (b0+b1+b2)/(1-a1-a2) = 6 / 0.75 = 8. Tolerance derived, not guessed: (1) the exact step
// response differs from 8 by the transient sum_{k>=n} h[k]; the poles have radius 0.5, |h[k]| <= 4 * 0.5^(k-2) holds for
// the hand impulse response (h2 = 4, then h decays by at least 0.5 per step on average), so after 59 samples the
// transient is below 4 * 0.5^57 * 2 = 6e-17: the double DEF equals 8 within 1e-12 (asserted). (2) The float output is
// within the running error bound of the double DEF (see header), asserted at the last sample with that bound.
void dcGain(Kind k) {
	unsigned ch = channels(k);
	std::vector<float> in(60 * ch, 1.0f);
	Filt f(k, 1, kHand);
	std::vector<float> out = runOnce(f, in);
	std::vector<float> c(kHand, kHand + 5);
	Ref r = refFor(k, c, 1, in);
	for (unsigned j = 0; j < ch; j++) {
		size_t i = 59 * ch + j;
		CAPTURE(j);
		CHECK(std::fabs(r.y[i] - 8.0) < 1e-12);
		CHECK(std::fabs(static_cast<double>(out[i]) - 8.0) <= r.e[i] + 1e-12);
	}
}

}  // namespace

TEST_CASE("[C3 arm_biquad_cascade_df1_f32] the step response of the hand stage settles at the DC gain 8 = (b0+b1+b2)/(1-a1-a2) within the derived bound") { dcGain(DF1); }
TEST_CASE("[C3 arm_biquad_cascade_df2T_f32] the step response of the hand stage settles at the DC gain 8 = (b0+b1+b2)/(1-a1-a2) within the derived bound") { dcGain(DF2T); }
TEST_CASE("[C3 arm_biquad_cascade_stereo_df2T_f32] the step response of the hand stage settles at the DC gain 8 on both channels within the derived bound") { dcGain(STEREO); }

namespace {

// Feedback-only stage: y[n] = x[n] + a1 y[n-1] or + a2 y[n-2] with 0.5: impulse response by hand
void feedbackOnly(Kind k, bool useA2) {
	float c[5] = {1.0f, 0.0f, 0.0f, useA2 ? 0.0f : 0.5f, useA2 ? 0.5f : 0.0f};
	Filt f(k, 1, c);
	std::vector<float> out = runOnce(f, impulse(k, 10));
	unsigned ch = channels(k);
	for (size_t i = 0; i < 10; i++) {
		// a1: h[n] = 0.5^n; a2: h[n] = 0.5^(n/2) for even n, 0 for odd n
		float want = useA2 ? ((i % 2 == 0) ? std::ldexp(1.0f, -static_cast<int>(i / 2)) : 0.0f) : std::ldexp(1.0f, -static_cast<int>(i));
		CAPTURE(i);
		CHECK(out[i * ch] == want);
		if (ch == 2) CHECK(out[i * 2 + 1] == 0.0f);
	}
}

}  // namespace

TEST_CASE("[C3 arm_biquad_cascade_df1_f32] a feedback stage with a1 = 0.5 has the impulse response 0.5^n") { feedbackOnly(DF1, false); }
TEST_CASE("[C3 arm_biquad_cascade_df2T_f32] a feedback stage with a1 = 0.5 has the impulse response 0.5^n") { feedbackOnly(DF2T, false); }
TEST_CASE("[C3 arm_biquad_cascade_stereo_df2T_f32] a feedback stage with a1 = 0.5 has the impulse response 0.5^n") { feedbackOnly(STEREO, false); }
TEST_CASE("[C3 arm_biquad_cascade_df1_f32] a feedback stage with a2 = 0.5 has the impulse response 1, 0, 0.5, 0, 0.25, ...") { feedbackOnly(DF1, true); }
TEST_CASE("[C3 arm_biquad_cascade_df2T_f32] a feedback stage with a2 = 0.5 has the impulse response 1, 0, 0.5, 0, 0.25, ...") { feedbackOnly(DF2T, true); }
TEST_CASE("[C3 arm_biquad_cascade_stereo_df2T_f32] a feedback stage with a2 = 0.5 has the impulse response 1, 0, 0.5, 0, 0.25, ...") { feedbackOnly(STEREO, true); }

namespace {

// Random stable cascades: ns = 1..9 stages (unroll tails and the stage loop), block sizes 1..40 frames (df1 unrolls by
// 4, df2T by 16, stereo by 8) against the double DEF within the derived bound.
void defSweep(Kind k) {
	Rng rng(0x5eed0000u + static_cast<uint32_t>(k));
	std::string first;
	for (unsigned ns = 1; ns <= 9 && first.empty(); ns++) {
		for (unsigned n = 1; n <= 40 && first.empty(); n++) {
			std::vector<float> c = genCoefs(rng, ns);
			std::vector<float> x = genSignal(rng, n * channels(k));
			Filt f(k, ns, c.data());
			std::vector<float> out = runOnce(f, x);
			Ref r = refFor(k, c, ns, x);
			first = withinBound(out, r.y, r.e);
			if (!first.empty()) first = "stages " + std::to_string(ns) + " frames " + std::to_string(n) + ": " + first;
		}
	}
	CHECK_MESSAGE(first.empty(), first);
}

}  // namespace

TEST_CASE("[C3 arm_biquad_cascade_df1_f32] random stable cascades of 1..9 stages and 1..40 samples match the double DEF within the derived bound") {
	defSweep(DF1);
}
TEST_CASE("[C3 arm_biquad_cascade_df2T_f32] random stable cascades of 1..9 stages and 1..40 samples match the double DEF within the derived bound") {
	defSweep(DF2T);
}
TEST_CASE("[C3 arm_biquad_cascade_stereo_df2T_f32] random stable cascades of 1..9 stages and 1..40 frames match the double DEF within the derived bound on both channels") {
	defSweep(STEREO);
}

TEST_CASE("[C3 arm_biquad_cascade_df1_f32] the derived error bound stays below 1e-3 of the peak output for 1..4 stages and the signals used (the bound is tight enough to catch a wrong coefficient)") {
	Rng rng(0x71ab7u);
	for (unsigned ns = 1; ns <= 4; ns++) {
		std::vector<float> c = genCoefs(rng, ns);
		std::vector<float> x = genSignal(rng, 40);
		Ref r = refDf1(c, ns, x);
		double peak = 0, emax = 0;
		for (size_t i = 0; i < r.y.size(); i++) { peak = std::fmax(peak, std::fabs(r.y[i])); emax = std::fmax(emax, r.e[i]); }
		CAPTURE(ns);
		CHECK(emax < 1e-3 * peak);
	}
}
TEST_CASE("[C3 arm_biquad_cascade_df2T_f32] the derived error bound stays below 1e-3 of the peak output for 1..4 stages and the signals used (the bound is tight enough to catch a wrong coefficient)") {
	Rng rng(0x71ab8u);
	for (unsigned ns = 1; ns <= 4; ns++) {
		std::vector<float> c = genCoefs(rng, ns);
		std::vector<float> x = genSignal(rng, 40);
		Ref r = refDf2T(c, ns, x);
		double peak = 0, emax = 0;
		for (size_t i = 0; i < r.y.size(); i++) { peak = std::fmax(peak, std::fabs(r.y[i])); emax = std::fmax(emax, r.e[i]); }
		CAPTURE(ns);
		CHECK(emax < 1e-3 * peak);
	}
}

namespace {

// Full-range strided sweep: a stage with ONE nonzero numerator coefficient c at position `pos` (0 = b0, 1 = b1, 2 = b2)
// is a delay of pos samples times c. The expected output is the double DEF fl_float(double(c) * double(x[n-pos])):
// the product of two binary32 numbers is exact in binary64 (48 bit significand), so the single rounding to float is
// the correctly rounded float product that the kernel computes (all other products are 0 and the sums are exact):
// result bit exact. Inputs: every 8191st bit pattern of the 2^32 patterns (about 524000 values: all exponents, both
// signs, normals and denormals); infinities, NaNs and values whose product overflows are replaced by 0 (expected 0).
void singleTapSweep(Kind k, int pos) {
	const float c = 0.7f;
	float coefs[5] = {0, 0, 0, 0, 0};
	coefs[pos] = c;
	unsigned ch = channels(k);
	std::vector<float> in;
	for (uint64_t i = 0; i < 4294967296ull; i += 8191) {
		float x = floatOfBits(static_cast<uint32_t>(i));
		// stereo: channel b gets the pattern of a different position of the sweep
		if (!std::isfinite(x) || std::fabs(static_cast<double>(c) * static_cast<double>(x)) > static_cast<double>(FLT_MAX)) x = 0.0f;
		in.push_back(x);
	}
	size_t frames = in.size();
	std::vector<float> sig(frames * ch);
	for (size_t i = 0; i < frames; i++) {
		sig[i * ch] = in[i];
		if (ch == 2) sig[i * 2 + 1] = in[(i * 7 + 3) % frames];
	}
	Filt f(k, 1, coefs);
	std::vector<float> out = runChunks(f, sig, {1021});   // blocks of 1021 frames: odd size, tails in every kernel
	size_t bad = 0, firstBad = 0;
	for (size_t i = 0; i < frames; i++) {
		for (unsigned j = 0; j < ch; j++) {
			float src = (i >= static_cast<size_t>(pos)) ? sig[(i - pos) * ch + j] : 0.0f;
			float want = static_cast<float>(static_cast<double>(c) * static_cast<double>(src));
			if (!(out[i * ch + j] == want)) { if (!bad) firstBad = i * ch + j; bad++; }
		}
	}
	CAPTURE(firstBad);
	CHECK(bad == 0);
	CHECK(f.guardsOk());
}

}  // namespace

TEST_CASE("[C3 arm_biquad_cascade_df1_f32] b0 alone multiplies every sample correctly rounded over the strided full float range") { singleTapSweep(DF1, 0); }
TEST_CASE("[C3 arm_biquad_cascade_df1_f32] b1 alone delays by one sample and multiplies correctly rounded over the strided full float range") { singleTapSweep(DF1, 1); }
TEST_CASE("[C3 arm_biquad_cascade_df1_f32] b2 alone delays by two samples and multiplies correctly rounded over the strided full float range") { singleTapSweep(DF1, 2); }
TEST_CASE("[C3 arm_biquad_cascade_df2T_f32] b0 alone multiplies every sample correctly rounded over the strided full float range") { singleTapSweep(DF2T, 0); }
TEST_CASE("[C3 arm_biquad_cascade_df2T_f32] b1 alone delays by one sample and multiplies correctly rounded over the strided full float range") { singleTapSweep(DF2T, 1); }
TEST_CASE("[C3 arm_biquad_cascade_df2T_f32] b2 alone delays by two samples and multiplies correctly rounded over the strided full float range") { singleTapSweep(DF2T, 2); }
TEST_CASE("[C3 arm_biquad_cascade_stereo_df2T_f32] b0 alone multiplies every sample of both channels correctly rounded over the strided full float range") { singleTapSweep(STEREO, 0); }
TEST_CASE("[C3 arm_biquad_cascade_stereo_df2T_f32] b1 alone delays by one frame and multiplies correctly rounded over the strided full float range") { singleTapSweep(STEREO, 1); }
TEST_CASE("[C3 arm_biquad_cascade_stereo_df2T_f32] b2 alone delays by two frames and multiplies correctly rounded over the strided full float range") { singleTapSweep(STEREO, 2); }

namespace {

// Scaling a signal by 2^k scales every intermediate value of every operation by 2^k exactly (rounding is relative), as
// long as nothing under- or overflows: a 3-stage random filter with a 37 sample random input (values -1..1; the preconditions are asserted in the test) stays
// inside the normal range of binary32. Bit-exact equality over k = -40..60 in steps of 5.
void powerOfTwoScaling(Kind k) {
	Rng rng(0x2a2a0000u + static_cast<uint32_t>(k));
	std::vector<float> c = genCoefs(rng, 3);
	std::vector<float> x = genSignal(rng, 37 * channels(k));
	Filt f0(k, 3, c.data());
	std::vector<float> base = runOnce(f0, x);
	// No underflow / overflow precondition (asserted): the smallest nonzero |x|, |output| and |coefficient| bound the
	// smallest product: 1e-12 * 1e-6 * 2^-40 (9e-13) = 9e-31, far above FLT_MIN (1.2e-38); largest value < 1e3 * 2^60 = 1e21
	double minX = 1e30, minY = 1e30, minC = 1e30, maxY = 0;
	for (float v : x) if (v != 0.0f) minX = std::fmin(minX, std::fabs(v));
	for (float v : base) { if (v != 0.0f) minY = std::fmin(minY, std::fabs(v)); maxY = std::fmax(maxY, std::fabs(v)); }
	for (float v : c) if (v != 0.0f) minC = std::fmin(minC, std::fabs(v));
	CHECK(minX > 1e-12);
	CHECK(minY > 1e-12);
	CHECK(minC > 1e-6);
	CHECK(maxY < 1e3);
	size_t bad = 0;
	int firstK = 0;
	for (int e = -40; e <= 60; e += 5) {
		std::vector<float> xs = x;
		for (float &v : xs) v = std::ldexp(v, e);
		Filt f(k, 3, c.data());
		std::vector<float> out = runOnce(f, xs);
		for (size_t i = 0; i < out.size(); i++)
			if (!(out[i] == std::ldexp(base[i], e))) { if (!bad) firstK = e; bad++; }
	}
	CAPTURE(firstK);
	CHECK(bad == 0);
}

}  // namespace

TEST_CASE("[C3 arm_biquad_cascade_df1_f32] scaling the input by 2^k scales the output exactly for k from -40 to 60") { powerOfTwoScaling(DF1); }
TEST_CASE("[C3 arm_biquad_cascade_df2T_f32] scaling the input by 2^k scales the output exactly for k from -40 to 60") { powerOfTwoScaling(DF2T); }
TEST_CASE("[C3 arm_biquad_cascade_stereo_df2T_f32] scaling the input by 2^k scales the output exactly for k from -40 to 60") { powerOfTwoScaling(STEREO); }

namespace {

// Stage s is a pure delay of (s % 3) samples (b0, b1 or b2 = 1), so a cascade of ns stages delays by sum(s % 3). This
// proves that every stage reads its own 5 coefficients (pointer advance of 5 per stage) and its own state slots.
void delayCascade(Kind k, unsigned ns) {
	std::vector<float> c(5 * ns, 0.0f);
	unsigned delay = 0;
	for (unsigned s = 0; s < ns; s++) { c[5 * s + (s % 3)] = 1.0f; delay += s % 3; }
	const size_t frames = 300;
	unsigned ch = channels(k);
	std::vector<float> x(frames * ch);
	for (size_t i = 0; i < frames; i++) {
		x[i * ch] = static_cast<float>(i + 1);
		if (ch == 2) x[i * 2 + 1] = static_cast<float>(-static_cast<int>(i + 1) * 2);
	}
	Filt f(k, ns, c.data());
	std::vector<float> out = runOnce(f, x);
	size_t bad = 0;
	for (size_t i = 0; i < frames; i++)
		for (unsigned j = 0; j < ch; j++) {
			float want = i >= delay ? x[(i - delay) * ch + j] : 0.0f;
			if (!(out[i * ch + j] == want)) bad++;
		}
	CAPTURE(ns);
	CAPTURE(delay);
	CHECK(bad == 0);
	CHECK(f.guardsOk());
}

}  // namespace

TEST_CASE("[C3 arm_biquad_cascade_df1_f32] cascades of 2, 3, 9 and 255 delay stages with distinct coefficients delay the signal by the sum of the stage delays") {
	for (unsigned ns : {2u, 3u, 9u, 255u}) delayCascade(DF1, ns);
}
TEST_CASE("[C3 arm_biquad_cascade_df2T_f32] cascades of 2, 3, 9 and 255 delay stages with distinct coefficients delay the signal by the sum of the stage delays") {
	for (unsigned ns : {2u, 3u, 9u, 255u}) delayCascade(DF2T, ns);
}
TEST_CASE("[C3 arm_biquad_cascade_stereo_df2T_f32] cascades of 2, 3, 9 and 255 delay stages with distinct coefficients delay both channels by the sum of the stage delays") {
	for (unsigned ns : {2u, 3u, 9u, 255u}) delayCascade(STEREO, ns);
}

// ---------------------------------------------------------------------------------------------------------------
// C4: state layout, init, continuity
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE("[C4 arm_biquad_cascade_df1_f32] two stages of hand data give the hand output and the state {x[n-1], x[n-2], y[n-1], y[n-2]} per stage") {
	// stage 1 {b0=2} : y = 2x; stage 2 {b0=1, a1=0.5}: y2[n] = u[n] + 0.5 y2[n-1]; input 1, 2, 3 (one block of 3 = tail path)
	// stage 1 out 2, 4, 6; stage 2 out 2, 4 + 1 = 5, 6 + 2.5 = 8.5
	// stage 1 state: x[n-1]=3, x[n-2]=2, y[n-1]=6, y[n-2]=4; stage 2: x[n-1]=6, x[n-2]=4, y[n-1]=8.5, y[n-2]=5
	const float c[10] = {2, 0, 0, 0, 0, 1, 0, 0, 0.5f, 0};
	const std::vector<float> x = {1, 2, 3};
	Filt f(DF1, 2, c);
	std::vector<float> out = runOnce(f, x);
	CHECK(out == std::vector<float>({2.0f, 5.0f, 8.5f}));
	CHECK(f.contents() == std::vector<float>({3, 2, 6, 4, 6, 4, 8.5f, 5}));
}

TEST_CASE("[C4 arm_biquad_cascade_df1_f32] a single-sample block leaves the state {x, 0, y, 0} (the earlier slots shift in zeros)") {
	// first sample 3 through {b0=2,...}: y = 6; state {x[n-1]=3, x[n-2]=0, y[n-1]=6, y[n-2]=0}
	const float c[5] = {2, 0, 0, 0, 0};
	Filt f(DF1, 1, c);
	float x = 3.0f, y = -1.0f;
	f.process(&x, &y, 1);
	CHECK(y == 6.0f);
	CHECK(f.contents() == std::vector<float>({3.0f, 0.0f, 6.0f, 0.0f}));
}

TEST_CASE("[C4 arm_biquad_cascade_df1_f32] a stage resumes from a given state with distinct coefficients per slot (hand values 18 and 15)") {
	// stage {b0=0, b1=1, b2=2, a1=0.5, a2=0.25}, state {x1=2, x2=4, y1=8, y2=16}, input 0, 0
	// y0 = 0 + 1*2 + 2*4 + 0.5*8 + 0.25*16 = 2 + 8 + 4 + 4 = 18; state {0, 2, 18, 8}
	// y1 = 0 + 1*0 + 2*2 + 0.5*18 + 0.25*8 = 4 + 9 + 2 = 15; state {0, 0, 15, 18}
	const float c[5] = {0, 1, 2, 0.5f, 0.25f};
	Filt f(DF1, 1, c);
	f.st()[0] = 2; f.st()[1] = 4; f.st()[2] = 8; f.st()[3] = 16;
	float x[2] = {0, 0}, y[2] = {-1, -1};
	f.process(x, y, 2);
	CHECK(y[0] == 18.0f);
	CHECK(y[1] == 15.0f);
	CHECK(f.contents() == std::vector<float>({0, 0, 15, 18}));
}

TEST_CASE("[C4 arm_biquad_cascade_df2T_f32] a stage resumes from a given state {d1, d2} with hand values 5 and 8.5") {
	// stage {b0=2, b1=1, b2=4, a1=0.5, a2=0.25}, state {d1=3, d2=5}, input 1, 0
	// y0 = 2*1 + 3 = 5; d1 = 1*1 + 5 + 0.5*5 = 8.5; d2 = 4*1 + 0.25*5 = 5.25
	// y1 = 2*0 + 8.5 = 8.5; d1 = 0 + 5.25 + 0.5*8.5 = 9.5; d2 = 0 + 0.25*8.5 = 2.125
	const float c[5] = {2, 1, 4, 0.5f, 0.25f};
	Filt f(DF2T, 1, c);
	f.st()[0] = 3; f.st()[1] = 5;
	float x[2] = {1, 0}, y[2] = {-1, -1};
	f.process(x, y, 2);
	CHECK(y[0] == 5.0f);
	CHECK(y[1] == 8.5f);
	CHECK(f.contents() == std::vector<float>({9.5f, 2.125f}));
}

TEST_CASE("[C4 arm_biquad_cascade_df2T_f32] two stages of hand data give the hand output and the state {d1, d2} per stage") {
	// stage 1 = hand stage, input impulse over 2 samples: y = 1, 2.5; d1 after sample 0 = 2*1 + 0.5*1 + 0 = 2.5, d2 = 3*1 - 0.25*1 = 2.75;
	//   sample 1 (x=0): y = 2.5; d1 = 0 + 2.75 + 0.5*2.5 = 4.0, d2 = 0 - 0.25*2.5 = -0.625  -> state {4, -0.625}
	// stage 2 {b0=1, b1=1}: y = u[n] + d1, d1 = u[n] (d2 = 0): u = 1, 2.5 -> y = 1, 2.5 + 1 = 3.5; state {2.5, 0}
	const float c[10] = {1, 2, 3, 0.5f, -0.25f, 1, 1, 0, 0, 0};
	Filt f(DF2T, 2, c);
	std::vector<float> out = runOnce(f, {1.0f, 0.0f});
	CHECK(out == std::vector<float>({1.0f, 3.5f}));
	CHECK(f.contents() == std::vector<float>({4.0f, -0.625f, 2.5f, 0.0f}));
}

TEST_CASE("[C4 arm_biquad_cascade_stereo_df2T_f32] a stage resumes from a given state {d1a, d2a, d1b, d2b} with interleaved hand values") {
	// stage {b0=2, b1=1, b2=4, a1=0.5, a2=0.25}. Channel a: state {3, 5}, input 1, 0 -> output 5, 8.5, state {9.5, 2.125} (as df2T above).
	// Channel b: state {d1=-1, d2=2}, input 2, 1:
	//   y0 = 2*2 - 1 = 3; d1 = 1*2 + 2 + 0.5*3 = 5.5; d2 = 4*2 + 0.25*3 = 8.75
	//   y1 = 2*1 + 5.5 = 7.5; d1 = 1*1 + 8.75 + 0.5*7.5 = 13.5; d2 = 4*1 + 0.25*7.5 = 5.875
	// interleaved output a0, b0, a1, b1 = 5, 3, 8.5, 7.5
	const float c[5] = {2, 1, 4, 0.5f, 0.25f};
	Filt f(STEREO, 1, c);
	f.st()[0] = 3; f.st()[1] = 5; f.st()[2] = -1; f.st()[3] = 2;
	float x[4] = {1, 2, 0, 1}, y[4] = {-1, -1, -1, -1};
	f.process(x, y, 2);
	CHECK(y[0] == 5.0f);
	CHECK(y[1] == 3.0f);
	CHECK(y[2] == 8.5f);
	CHECK(y[3] == 7.5f);
	CHECK(f.contents() == std::vector<float>({9.5f, 2.125f, 13.5f, 5.875f}));
}

TEST_CASE("[C4 arm_biquad_cascade_stereo_df2T_f32] channel a is the impulse response and channel b twice it, with the state {4, -0.625, 8, -1.25} after 2 frames") {
	// linear: channel b = 2 * channel a (impulse 2 instead of 1): outputs 1, 2 | 2.5, 5 ; states 2x: {4, -0.625} and {8, -1.25}
	Filt f(STEREO, 1, kHand);
	std::vector<float> out = runOnce(f, {1.0f, 2.0f, 0.0f, 0.0f});
	CHECK(out == std::vector<float>({1.0f, 2.0f, 2.5f, 5.0f}));
	CHECK(f.contents() == std::vector<float>({4.0f, -0.625f, 8.0f, -1.25f}));
}

namespace {

// State at the end of a block against the DEF state (layout of each kernel) within the derived bound
void stateVsDef(Kind k) {
	Rng rng(0x57a7e000u + static_cast<uint32_t>(k));
	std::string first;
	for (unsigned ns = 1; ns <= 4 && first.empty(); ns++) {
		for (unsigned n = 1; n <= 20 && first.empty(); n++) {
			std::vector<float> c = genCoefs(rng, ns);
			std::vector<float> x = genSignal(rng, n * channels(k));
			Filt f(k, ns, c.data());
			runOnce(f, x);
			Ref r = refFor(k, c, ns, x);
			first = withinBound(f.contents(), r.st, r.ste);
			if (!first.empty()) first = "stages " + std::to_string(ns) + " frames " + std::to_string(n) + ": " + first;
		}
	}
	CHECK_MESSAGE(first.empty(), first);
}

}  // namespace

TEST_CASE("[C4 arm_biquad_cascade_df1_f32] the state after random blocks of 1..20 samples equals the DEF state within the derived bound") { stateVsDef(DF1); }
TEST_CASE("[C4 arm_biquad_cascade_df2T_f32] the state after random blocks of 1..20 samples equals the DEF state within the derived bound") { stateVsDef(DF2T); }
TEST_CASE("[C4 arm_biquad_cascade_stereo_df2T_f32] the state after random blocks of 1..20 frames equals the DEF state within the derived bound") { stateVsDef(STEREO); }

namespace {

// Init DOC (arm_biquad_cascade_df1_init_f32.c:36-40 "Sets the values of the internal structure fields. Zeros out the
// values in the state buffer."): fields hold the arguments
void initFields(Kind k) {
	for (unsigned ns : {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 255u}) {
		Rng rng(ns);
		std::vector<float> c = genCoefs(rng, ns);
		Filt f(k, ns, c.data());
		CAPTURE(ns);
		if (k == DF1) {
			CHECK(f.s1.numStages == ns);
			CHECK(f.s1.pCoeffs == c.data());
			CHECK(f.s1.pState == f.st());
		} else if (k == DF2T) {
			CHECK(f.s2.numStages == ns);
			CHECK(f.s2.pCoeffs == c.data());
			CHECK(f.s2.pState == f.st());
		} else {
			CHECK(f.s3.numStages == ns);
			CHECK(f.s3.pCoeffs == c.data());
			CHECK(f.s3.pState == f.st());
		}
	}
}

// The state buffer has exactly the documented length (4 / 2 / 4 * numStages): init zeroes all of it and nothing
// around it; processing stays inside it (guards on both sides, sanitizer builds catch overruns of the exact size).
void initClearsExactState(Kind k) {
	for (unsigned ns : {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 255u}) {
		Rng rng(100 + ns);
		std::vector<float> c = genCoefs(rng, ns);
		Padded st(stateLen(k, ns));
		// Filt owns its state, so build the instance by hand here to see the buffer before and after init
		if (k == DF1) {
			arm_biquad_casd_df1_inst_f32 s;
			arm_biquad_cascade_df1_init_f32(&s, static_cast<uint8_t>(ns), c.data(), st.body());
		} else if (k == DF2T) {
			arm_biquad_cascade_df2T_instance_f32 s;
			arm_biquad_cascade_df2T_init_f32(&s, static_cast<uint8_t>(ns), c.data(), st.body());
		} else {
			arm_biquad_cascade_stereo_df2T_instance_f32 s;
			arm_biquad_cascade_stereo_df2T_init_f32(&s, static_cast<uint8_t>(ns), c.data(), st.body());
		}
		CAPTURE(ns);
		CHECK(st.bodyAllZero());
		CHECK(st.guardsIntact());
	}
}

void processStaysInState(Kind k) {
	for (unsigned ns : {1u, 4u, 9u, 255u}) {
		Rng rng(200 + ns);
		std::vector<float> c = genCoefs(rng, ns);
		std::vector<float> x = genSignal(rng, 19 * channels(k));
		Filt f(k, ns, c.data(), true);
		runChunks(f, x, {7, 5, 3, 4});
		CAPTURE(ns);
		CHECK(f.guardsOk());
	}
}

}  // namespace

TEST_CASE("[C4 arm_biquad_cascade_df1_init_f32] init stores numStages, the coefficient pointer and the state pointer for 1..9 and 255 stages") { initFields(DF1); }
TEST_CASE("[C4 arm_biquad_cascade_df2T_init_f32] init stores numStages, the coefficient pointer and the state pointer for 1..9 and 255 stages") { initFields(DF2T); }
TEST_CASE("[C4 arm_biquad_cascade_stereo_df2T_init_f32] init stores numStages, the coefficient pointer and the state pointer for 1..9 and 255 stages") { initFields(STEREO); }

TEST_CASE("[C4 arm_biquad_cascade_df1_init_f32] init zeroes the poisoned state of exactly 4*numStages floats and touches nothing around it") { initClearsExactState(DF1); }
TEST_CASE("[C4 arm_biquad_cascade_df2T_init_f32] init zeroes the poisoned state of exactly 2*numStages floats and touches nothing around it") { initClearsExactState(DF2T); }
TEST_CASE("[C4 arm_biquad_cascade_stereo_df2T_init_f32] init zeroes the poisoned state of exactly 4*numStages floats and touches nothing around it") { initClearsExactState(STEREO); }

TEST_CASE("[C4 arm_biquad_cascade_df1_f32] processing stays inside a state buffer of exactly 4*numStages floats") { processStaysInState(DF1); }
TEST_CASE("[C4 arm_biquad_cascade_df2T_f32] processing stays inside a state buffer of exactly 2*numStages floats") { processStaysInState(DF2T); }
TEST_CASE("[C4 arm_biquad_cascade_stereo_df2T_f32] processing stays inside a state buffer of exactly 4*numStages floats") { processStaysInState(STEREO); }

TEST_CASE("[C4 arm_biquad_cascade_df1_f32] a static instance {numStages, pState, pCoeffs} as in the DOC behaves like an initialised one") {
	// DOC (arm_biquad_cascade_df1_f32.c:92): arm_biquad_casd_df1_inst_f32 S1 = {numStages, pState, pCoeffs};
	Rng rng(7);
	std::vector<float> c = genCoefs(rng, 3);
	std::vector<float> x = genSignal(rng, 21);
	Padded st(12);
	std::memset(static_cast<void *>(st.body()), 0, 12 * sizeof(float));
	arm_biquad_casd_df1_inst_f32 S1 = {3, st.body(), c.data()};
	std::vector<float> out(21);
	arm_biquad_cascade_df1_f32(&S1, x.data(), out.data(), 21);
	Filt f(DF1, 3, c.data());
	CHECK(out == runOnce(f, x));
}

TEST_CASE("[C4 arm_biquad_cascade_df1_f32] processing leaves the coefficients and the input untouched") {
	Rng rng(8);
	std::vector<float> c = genCoefs(rng, 4), c0 = c;
	std::vector<float> x = genSignal(rng, 23), x0 = x;
	Filt f(DF1, 4, c.data());
	runOnce(f, x);
	CHECK(c == c0);
	CHECK(x == x0);
}
TEST_CASE("[C4 arm_biquad_cascade_df2T_f32] processing leaves the coefficients and the input untouched") {
	Rng rng(9);
	std::vector<float> c = genCoefs(rng, 4), c0 = c;
	std::vector<float> x = genSignal(rng, 23), x0 = x;
	Filt f(DF2T, 4, c.data());
	runOnce(f, x);
	CHECK(c == c0);
	CHECK(x == x0);
}
TEST_CASE("[C4 arm_biquad_cascade_stereo_df2T_f32] processing leaves the coefficients and the input untouched") {
	Rng rng(10);
	std::vector<float> c = genCoefs(rng, 4), c0 = c;
	std::vector<float> x = genSignal(rng, 46), x0 = x;
	Filt f(STEREO, 4, c.data());
	runOnce(f, x);
	CHECK(c == c0);
	CHECK(x == x0);
}

namespace {

// State continuity: the signal in blocks of 1, 2, 3, 4, 5, 7, 9, 15, 16, 17, 33 and a mixed sequence matches the double
// DEF within the derived bound, the final state too (the DEF of the whole signal is the oracle: no bit-exact
// comparison of different partitions, see the C6 note).
void continuity(Kind k) {
	Rng rng(0xc0471000u + static_cast<uint32_t>(k));
	const unsigned ns = 3;
	std::vector<float> c = genCoefs(rng, ns);
	std::vector<float> x = genSignal(rng, 64 * channels(k));
	Ref r = refFor(k, c, ns, x);
	const std::vector<std::vector<uint32_t>> parts = {{64}, {1}, {2}, {3}, {4}, {5}, {7}, {9}, {15}, {16}, {17}, {33}, {1, 2, 3, 4, 5, 7, 16, 17}};
	std::string first;
	for (size_t p = 0; p < parts.size() && first.empty(); p++) {
		Filt f(k, ns, c.data());
		std::vector<float> out = runChunks(f, x, parts[p]);
		first = withinBound(out, r.y, r.e);
		if (first.empty()) first = withinBound(f.contents(), r.st, r.ste);
		if (!first.empty()) first = "partition " + std::to_string(p) + ": " + first;
	}
	CHECK_MESSAGE(first.empty(), first);
}

}  // namespace

TEST_CASE("[C4 arm_biquad_cascade_df1_f32] one long block and consecutive blocks of 1, 2, 3, 4, 5, 7, 9, 15, 16, 17, 33 and mixed sizes all match the DEF within the bound, output and state") { continuity(DF1); }
TEST_CASE("[C4 arm_biquad_cascade_df2T_f32] one long block and consecutive blocks of 1, 2, 3, 4, 5, 7, 9, 15, 16, 17, 33 and mixed sizes all match the DEF within the bound, output and state") { continuity(DF2T); }
TEST_CASE("[C4 arm_biquad_cascade_stereo_df2T_f32] one long block and consecutive blocks of 1, 2, 3, 4, 5, 7, 9, 15, 16, 17, 33 and mixed frame counts all match the DEF within the bound, output and state") { continuity(STEREO); }

namespace {

// Two instances sharing one coefficient array but with separate state buffers, processed alternately with different
// signals, give the same bits as each instance alone with the same block partition (identical arithmetic).
void independentInstances(Kind k) {
	Rng rng(0x1d3e0000u + static_cast<uint32_t>(k));
	std::vector<float> c = genCoefs(rng, 3);
	std::vector<float> xa = genSignal(rng, 30 * channels(k));
	std::vector<float> xb = genSignal(rng, 30 * channels(k));
	const std::vector<uint32_t> chunks = {5, 3, 7};
	Filt aloneA(k, 3, c.data()), aloneB(k, 3, c.data());
	std::vector<float> wantA = runChunks(aloneA, xa, chunks), wantB = runChunks(aloneB, xb, chunks);
	Filt a(k, 3, c.data()), b(k, 3, c.data());
	unsigned ch = channels(k);
	std::vector<float> outA(xa.size()), outB(xb.size());
	size_t pos = 0, ci = 0;
	while (pos < 30) {
		size_t n = chunks[ci++ % chunks.size()];
		if (n > 30 - pos) n = 30 - pos;
		a.process(xa.data() + pos * ch, outA.data() + pos * ch, static_cast<uint32_t>(n));
		b.process(xb.data() + pos * ch, outB.data() + pos * ch, static_cast<uint32_t>(n));
		pos += n;
	}
	CHECK(outA == wantA);
	CHECK(outB == wantB);
}

// Re-init resets the state: the same signal after init again gives the same bits as the first run
void reinitResets(Kind k) {
	Rng rng(0x7e1a0000u + static_cast<uint32_t>(k));
	std::vector<float> c = genCoefs(rng, 3);
	std::vector<float> x = genSignal(rng, 25 * channels(k));
	std::vector<float> other = genSignal(rng, 11 * channels(k));
	Filt f(k, 3, c.data());
	std::vector<float> first = runOnce(f, x);
	runOnce(f, other);   // leaves a nonzero state
	CHECK_FALSE(f.allZero());
	f.init();
	CHECK(f.allZero());
	CHECK(runOnce(f, x) == first);
}

}  // namespace

TEST_CASE("[C4 arm_biquad_cascade_df1_f32] two instances with separate state buffers do not influence each other") { independentInstances(DF1); }
TEST_CASE("[C4 arm_biquad_cascade_df2T_f32] two instances with separate state buffers do not influence each other") { independentInstances(DF2T); }
TEST_CASE("[C4 arm_biquad_cascade_stereo_df2T_f32] two instances with separate state buffers do not influence each other") { independentInstances(STEREO); }

TEST_CASE("[C4 arm_biquad_cascade_df1_init_f32] calling init again clears the state and the same input gives the same output bits") { reinitResets(DF1); }
TEST_CASE("[C4 arm_biquad_cascade_df2T_init_f32] calling init again clears the state and the same input gives the same output bits") { reinitResets(DF2T); }
TEST_CASE("[C4 arm_biquad_cascade_stereo_df2T_init_f32] calling init again clears the state and the same input gives the same output bits") { reinitResets(STEREO); }

// ---------------------------------------------------------------------------------------------------------------
// C2: output guards
// ---------------------------------------------------------------------------------------------------------------

namespace {

void outputGuard(Kind k) {
	Rng rng(0x6a2d0000u + static_cast<uint32_t>(k));
	unsigned ch = channels(k);
	for (unsigned n = 1; n <= 20; n++) {
		std::vector<float> c = genCoefs(rng, 2);
		std::vector<float> x = genSignal(rng, n * ch);
		Filt f(k, 2, c.data());
		dsp::GuardedBuffer<float> out(n * ch);
		f.process(x.data(), out.data(), n);
		CAPTURE(n);
		CHECK(out.guardIntact());
		bool allWritten = true;
		for (float v : out.output())
			if (bitsOf(v) == 0xCDCDCDCDu) allWritten = false;
		CHECK(allWritten);
	}
}

}  // namespace

TEST_CASE("[C2 arm_biquad_cascade_df1_f32] blocks of 1..20 samples write every output sample and nothing beyond the block") { outputGuard(DF1); }
TEST_CASE("[C2 arm_biquad_cascade_df2T_f32] blocks of 1..20 samples write every output sample and nothing beyond the block") { outputGuard(DF2T); }
TEST_CASE("[C2 arm_biquad_cascade_stereo_df2T_f32] blocks of 1..20 frames write every output sample of both channels and nothing beyond the block") { outputGuard(STEREO); }

namespace {

// DOC is silent about blockSize 0; the code has no loop iteration for it, so nothing is read or written
void blockSizeZero(Kind k) {
	Rng rng(0xb10c0000u + static_cast<uint32_t>(k));
	std::vector<float> c = genCoefs(rng, 2);
	std::vector<float> x = genSignal(rng, 5 * channels(k));
	Filt f(k, 2, c.data());
	runOnce(f, x);
	std::vector<float> before = f.contents();
	dsp::GuardedBuffer<float> out(4);
	f.process(x.data(), out.data(), 0);
	CHECK(f.contents() == before);
	bool untouched = true;
	for (float v : out.output())
		if (bitsOf(v) != 0xCDCDCDCDu) untouched = false;
	CHECK(untouched);
	CHECK(out.guardIntact());
}

}  // namespace

TEST_CASE("characterization: [C4 arm_biquad_cascade_df1_f32] a block of size 0 leaves the state and the output buffer unchanged") { blockSizeZero(DF1); }
TEST_CASE("characterization: [C4 arm_biquad_cascade_df2T_f32] a block of size 0 leaves the state and the output buffer unchanged") { blockSizeZero(DF2T); }
TEST_CASE("characterization: [C4 arm_biquad_cascade_stereo_df2T_f32] a block of size 0 leaves the state and the output buffer unchanged") { blockSizeZero(STEREO); }

}  // TEST_SUITE
