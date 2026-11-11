/*
 * Filtering, biquad f16 (phase P11, group G4, work order FB6416): arm_biquad_cascade_df1_f16 (+ init),
 * arm_biquad_cascade_df2T_f16 (+ init), arm_biquad_cascade_stereo_df2T_f16 (+ init) of Source/FilteringFunctions.
 * Requirements C1-C6 of Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md. Compiled with
 * -D__ARM_FP16_FORMAT_IEEE=1 -D__fp16=_Float16 (file name ends in _f16.cpp, lead decision L2).
 *
 * ARM's suite "BIQUAD F16" (Testing/desc_f16.txt:750-776, bodies Testing/Source/Tests/BIQUADF16.cpp), 5 ids, all enabled:
 *   1 df1 ref   (BIQUADF16.cpp:16-88)   BiquadInput1_f16 (100) + BiquadCoefs1_f16 (15) -> BiquadOutput1_f16 (100), 3 stages, 2 x 50
 *   2 df2T ref  (:90-151)               same patterns, 3 stages, 2 x 50
 *   3 df1 rand  (:153-227)              AllBiquadInputs2_f16 (138) + AllBiquadCoefs2_f16 (690) + AllBiquadConfigs2_s16 (18 =
 *                                       9 pairs numStages, blockSize) -> AllBiquadRefs2_f16 (138); one call per config
 *   4 df2T rand (:229-299)              same patterns
 *   5 stereo df2T rand (:301-370)       AllBiquadStereoInputs2_f16 (276) -> AllBiquadStereoRefs2_f16 (276), blockSize = frames,
 *                                       2 samples per frame (interleaved)
 *   (patterns in Testing/Patterns/DSP/Filtering/BIQUAD/BIQUADF16/). The MVE variants of ARM's test (#if ARM_MATH_MVE_FLOAT16)
 *   are not built on the host; ARM's state buffer is 128 elements, here it is EXACTLY 4/2/4 * numStages.
 * ARM tolerances (BIQUADF16.cpp:5, :13, :14): SNR_THRESHOLD 27 dB, REL_ERROR 5.0e-2, ABS_ERROR 1.0e-1, per sample
 * ASSERT_CLOSE_ERROR |out-ref| <= 0.1 + 0.05 |ref| (Error.cpp:448), ASSERT_EMPTY_TAIL. Reference = double computation
 * (:7-12). Used unchanged (L4).
 *
 * Convention (DOC, same as f64/f32): y[n] = b0 x[n] + b1 x[n-1] + b2 x[n-2] + a1 y[n-1] + a2 y[n-2] (df1; a1, a2 with a
 * PLUS sign); df2T: y = b0 x + d1; d1 = b1 x + a1 y + d2; d2 = b2 x + a2 y. Coefficients {b0,b1,b2,a1,a2} per stage.
 * State: df1 {x[n-1], x[n-2], y[n-1], y[n-2]} per stage (4*numStages, init DOC); df2T {d1, d2} per stage (2*numStages);
 * stereo df2T 4*numStages (header filtering_functions_f16.h:147, code memset). DOC DEFECT: the DOC text of
 * arm_biquad_cascade_stereo_df2T_init_f16 says "The state array has a total length of 2*numStages values"; the header and the code
 * (and the f32 original) use 4*numStages. The tests use 4*numStages (two channels x two states): reported to the lead.
 * The DOC does not give the order of the 4 stereo state values: the layout test is a characterization.
 *
 * Running error bound (derivation). u = 2^-11 (binary16, round to nearest). A term of a sum with k roundings carries a
 * relative error <= gam(k) = k u / (1 - k u). With stage input value x (bound Ex) and the double DEF values/bounds of
 * the previous samples (E of each state), the error of the library value is, to first order (|v|+E = the library's own value):
 *   df1: y = b0 x + b1 x1 + b2 x2 + a1 y1 + a2 y2 (5 products, 4 sums in sequence: 5 roundings per term)
 *        Ey = gam(5) (|b0|(|x|+Ex) + |b1|(|x1|+Ex1) + |b2|(|x2|+Ex2) + |a1|(|y1|+Ey1) + |a2|(|y2|+Ey2))
 *             + |b0|Ex + |b1|Ex1 + |b2|Ex2 + |a1|Ey1 + |a2|Ey2 + F
 *   df2T / stereo (per channel): y = b0 x + d1               Ey  = gam(2)(|b0|(|x|+Ex) + |d1|+Ed1) + |b0|Ex + Ed1 + F
 *        d1 = b1 x + a1 y + d2    Ed1' = gam(4)(|b1|(|x|+Ex) + |a1|(|y|+Ey) + |d2|+Ed2) + |b1|Ex + |a1|Ey + Ed2 + F
 *        d2 = b2 x + a2 y         Ed2' = gam(3)(|b2|(|x|+Ex) + |a2|(|y|+Ey)) + |b2|Ex + |a2|Ey + F
 *   F = 5 * 2^-25: absolute error of the (up to 5) products in the subnormal range (half ulp of 2^-24 each; sums of
 *   subnormals are exact). Computing in float and rounding later (excess precision, FMA) has fewer roundings, so the bound covers it. The
 *   stage output bound is the input bound of the next stage; the double DEF itself contributes about 2^-42 of the bound
 *   (factor 1.0001 used). Data: |a1| <= 0.2, |a2| <= 0.15, |b| <= 0.3, inputs in [-1, 1], all rounded to binary16 first, so the
 *   bound recursion contracts (gain 1/(1-0.35) per stage). Sweeps use |x| <= 30000 and |b|, |a| <= 0.6 so that no overflow of binary16 (65504) can occur.
 *
 * Host vs target (C6): the host compiles _Float16 arithmetic through float (x86-64 without AVX512-FP16) and may keep float
 * intermediates inside and across statements (-fexcess-precision=fast); the target (Cortex-M55 MVE or soft emulation)
 * rounds differently in the last bit, and MVE sums lanes with fused multiply-add. The exact tests use data whose every
 * product and sum is exactly representable (integers < 2048, dyadic hand values), the others use the bound above, so all hold for any
 * of these. Block partitions (split / mixed blocks run through the unrolled and the tail code) are compared with the DEF bound, not bitwise: excess
 * precision kept in the state variables or FMA contraction could differ per code path in the last bit. Only runs with identical
 * partitions (two instances, re-init, tight versus guarded state) are compared bitwise. The state tests use, besides the guarded
 * buffer, a tight heap buffer without slack so that ASan sees a write past the state. The firmware ships no f16 code (the Makefile excludes %_f16.c); these tests protect the library as such.
 * numStages = 0 is not tested (the do/while loops would run 2^32 stages): NEEDS REVIEW.
 */
#include "doctest.h"

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math_f16.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#ifndef ARM_FLOAT16_SUPPORTED
#error "ARM_FLOAT16_SUPPORTED is not defined: the f16 compile definitions are missing for this file"
#endif

namespace {

typedef float16_t F;

constexpr double kSnr = 27.0;        // BIQUADF16.cpp:5
constexpr double kRelArm = 5.0e-2;   // :13
constexpr double kAbsArm = 1.0e-1;   // :14
constexpr double kU = 4.8828125e-4;  // 2^-11
constexpr double kU32 = 5.9604644775390625e-8;  // 2^-24
constexpr double kFloor16 = 5.0 * 2.98023223876953125e-8;   // 5 * 2^-25
constexpr double kDefFactor = 1.0001;

double gam(double k, double u) { return k * u / (1.0 - k * u); }
F h16(double d) { return static_cast<F>(d); }
double rnd(double d) { return static_cast<double>(h16(d)); }
F fromBits(uint16_t b) { F h; std::memcpy(&h, &b, sizeof h); return h; }

struct Rng {
	uint64_t s;
	explicit Rng(uint64_t seed) : s(seed) {}
	uint64_t next() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
	double uni() { const uint64_t r = next(); return static_cast<double>(r >> 11) * (2.0 / 9007199254740992.0) - 1.0; }   // [-1, 1)
	int range(int lo, int hi) { const uint64_t r = next(); return lo + static_cast<int>(r % static_cast<uint64_t>(hi - lo + 1)); }
};

struct Coef { double b0, b1, b2, a1, a2; };   // always exactly representable in binary16

std::vector<Coef> randomStages(Rng &g, int ns) {
	std::vector<Coef> st;
	for (int i = 0; i < ns; i++) {
		Coef c;
		double v = g.uni(); c.b0 = rnd(0.3 * v);
		v = g.uni(); c.b1 = rnd(0.3 * v);
		v = g.uni(); c.b2 = rnd(0.3 * v);
		v = g.uni(); c.a1 = rnd(0.2 * v);
		v = g.uni(); c.a2 = rnd(0.15 * v);
		st.push_back(c);
	}
	return st;
}
std::vector<double> randomFrames(Rng &g, size_t n, int ch) {
	std::vector<double> x;
	for (size_t i = 0; i < n * static_cast<size_t>(ch); i++) { const double v = g.uni(); x.push_back(rnd(v)); }
	return x;
}
std::vector<double> flattenD(const std::vector<Coef> &st) {
	std::vector<double> f;
	for (const Coef &c : st) { f.push_back(c.b0); f.push_back(c.b1); f.push_back(c.b2); f.push_back(c.a1); f.push_back(c.a2); }
	return f;
}
std::vector<F> toF(const std::vector<double> &d) { std::vector<F> f; for (double v : d) f.push_back(h16(v)); return f; }
std::vector<double> toD(const F *p, size_t n) { std::vector<double> d; for (size_t i = 0; i < n; i++) d.push_back(static_cast<double>(p[i])); return d; }

// ---- DEF in double with the running error bound ----------------------------------------------------------------------
struct Model {
	std::vector<double> y, e;
	double maxSum = 0.0;
};

Model modelDf1(const std::vector<Coef> &st, const std::vector<double> &x, double u, double fl) {
	Model m;
	m.y = x;
	m.e.assign(x.size(), 0.0);
	const double g5 = gam(5, u);
	for (const Coef &c : st) {
		double X1 = 0, X2 = 0, Y1 = 0, Y2 = 0, eX1 = 0, eX2 = 0, eY1 = 0, eY2 = 0;
		for (size_t i = 0; i < x.size(); i++) {
			const double xi = m.y[i], ex = m.e[i];
			const double y = c.b0 * xi + c.b1 * X1 + c.b2 * X2 + c.a1 * Y1 + c.a2 * Y2;
			const double s = std::fabs(c.b0) * (std::fabs(xi) + ex) + std::fabs(c.b1) * (std::fabs(X1) + eX1) + std::fabs(c.b2) * (std::fabs(X2) + eX2)
				+ std::fabs(c.a1) * (std::fabs(Y1) + eY1) + std::fabs(c.a2) * (std::fabs(Y2) + eY2);
			const double Ey = g5 * s + std::fabs(c.b0) * ex + std::fabs(c.b1) * eX1 + std::fabs(c.b2) * eX2 + std::fabs(c.a1) * eY1 + std::fabs(c.a2) * eY2 + fl;
			m.maxSum = std::max(m.maxSum, s);
			X2 = X1; eX2 = eX1; X1 = xi; eX1 = ex;
			Y2 = Y1; eY2 = eY1; Y1 = y; eY1 = Ey;
			m.y[i] = y; m.e[i] = Ey;
		}
	}
	return m;
}

Model modelDf2T(const std::vector<Coef> &st, const std::vector<double> &x, double u, double fl) {
	Model m;
	m.y = x;
	m.e.assign(x.size(), 0.0);
	const double g2 = gam(2, u), g3 = gam(3, u), g4 = gam(4, u);
	for (const Coef &c : st) {
		double d1 = 0, d2 = 0, E1 = 0, E2 = 0;
		for (size_t i = 0; i < x.size(); i++) {
			const double xi = m.y[i], ex = m.e[i], xa = std::fabs(xi) + ex;
			const double y = c.b0 * xi + d1;
			const double s0 = std::fabs(c.b0) * xa + std::fabs(d1) + E1;
			const double Ey = g2 * s0 + std::fabs(c.b0) * ex + E1 + fl;
			const double d1n = c.b1 * xi + c.a1 * y + d2;
			const double s1 = std::fabs(c.b1) * xa + std::fabs(c.a1) * (std::fabs(y) + Ey) + std::fabs(d2) + E2;
			const double E1n = g4 * s1 + std::fabs(c.b1) * ex + std::fabs(c.a1) * Ey + E2 + fl;
			const double d2n = c.b2 * xi + c.a2 * y;
			const double s2 = std::fabs(c.b2) * xa + std::fabs(c.a2) * (std::fabs(y) + Ey);
			const double E2n = g3 * s2 + std::fabs(c.b2) * ex + std::fabs(c.a2) * Ey + fl;
			m.maxSum = std::max(m.maxSum, std::max(s0, std::max(s1, s2)));
			d1 = d1n; d2 = d2n; E1 = E1n; E2 = E2n;
			m.y[i] = y; m.e[i] = Ey;
		}
	}
	return m;
}

Model modelStereo(const std::vector<Coef> &st, const std::vector<double> &x, double u, double fl) {
	std::vector<double> a, b;
	for (size_t i = 0; i + 1 < x.size(); i += 2) { a.push_back(x[i]); b.push_back(x[i + 1]); }
	const Model ma = modelDf2T(st, a, u, fl), mb = modelDf2T(st, b, u, fl);
	Model m;
	for (size_t i = 0; i < a.size(); i++) {
		m.y.push_back(ma.y[i]); m.y.push_back(mb.y[i]);
		m.e.push_back(ma.e[i]); m.e.push_back(mb.e[i]);
	}
	m.maxSum = std::max(ma.maxSum, mb.maxSum);
	return m;
}

std::string boundFailure(const std::vector<double> &lib, const Model &m, double factor, double extra = 0.0) {
	if (lib.size() != m.y.size()) return "length mismatch";
	for (size_t i = 0; i < lib.size(); i++) {
		const double err = std::fabs(lib[i] - m.y[i]);
		if (std::isnan(lib[i]) || !(err <= factor * m.e[i] + extra)) {
			char buf[160];
			std::snprintf(buf, sizeof buf, "sample %zu: lib %.9g def %.9g |diff| %.3g bound %.3g", i, lib[i], m.y[i], err, factor * m.e[i] + extra);
			return buf;
		}
	}
	return std::string();
}

// ---- the three forms ---------------------------------------------------------------------------------------------------
struct Df1 {
	typedef arm_biquad_casd_df1_inst_f16 Inst;
	static constexpr int ch = 1;
	static constexpr size_t stMul = 4;
	static void init(Inst *S, uint8_t ns, const F *c, F *st) { arm_biquad_cascade_df1_init_f16(S, ns, c, st); }
	static void run(const Inst *S, const F *in, F *out, uint32_t n) { arm_biquad_cascade_df1_f16(S, in, out, n); }
	static Model model(const std::vector<Coef> &st, const std::vector<double> &x, double u, double fl) { return modelDf1(st, x, u, fl); }
	static std::vector<float> runF32(const std::vector<Coef> &st, const std::vector<double> &x, size_t frames) {
		std::vector<float> c, in(x.begin(), x.end()), out(frames), state(4 * st.size());
		for (double v : flattenD(st)) c.push_back(static_cast<float>(v));
		arm_biquad_casd_df1_inst_f32 S;
		arm_biquad_cascade_df1_init_f32(&S, static_cast<uint8_t>(st.size()), c.data(), state.data());
		arm_biquad_cascade_df1_f32(&S, in.data(), out.data(), static_cast<uint32_t>(frames));
		return out;
	}
	static const char *inFile() { return "AllBiquadInputs2_f16.txt"; }
	static const char *refFile() { return "AllBiquadRefs2_f16.txt"; }
};
struct Df2T {
	typedef arm_biquad_cascade_df2T_instance_f16 Inst;
	static constexpr int ch = 1;
	static constexpr size_t stMul = 2;
	static void init(Inst *S, uint8_t ns, const F *c, F *st) { arm_biquad_cascade_df2T_init_f16(S, ns, c, st); }
	static void run(const Inst *S, const F *in, F *out, uint32_t n) { arm_biquad_cascade_df2T_f16(S, in, out, n); }
	static Model model(const std::vector<Coef> &st, const std::vector<double> &x, double u, double fl) { return modelDf2T(st, x, u, fl); }
	static std::vector<float> runF32(const std::vector<Coef> &st, const std::vector<double> &x, size_t frames) {
		std::vector<float> c, in(x.begin(), x.end()), out(frames), state(2 * st.size());
		for (double v : flattenD(st)) c.push_back(static_cast<float>(v));
		arm_biquad_cascade_df2T_instance_f32 S;
		arm_biquad_cascade_df2T_init_f32(&S, static_cast<uint8_t>(st.size()), c.data(), state.data());
		arm_biquad_cascade_df2T_f32(&S, in.data(), out.data(), static_cast<uint32_t>(frames));
		return out;
	}
	static const char *inFile() { return "AllBiquadInputs2_f16.txt"; }
	static const char *refFile() { return "AllBiquadRefs2_f16.txt"; }
};
struct Stereo {
	typedef arm_biquad_cascade_stereo_df2T_instance_f16 Inst;
	static constexpr int ch = 2;
	static constexpr size_t stMul = 4;
	static void init(Inst *S, uint8_t ns, const F *c, F *st) { arm_biquad_cascade_stereo_df2T_init_f16(S, ns, c, st); }
	static void run(const Inst *S, const F *in, F *out, uint32_t n) { arm_biquad_cascade_stereo_df2T_f16(S, in, out, n); }
	static Model model(const std::vector<Coef> &st, const std::vector<double> &x, double u, double fl) { return modelStereo(st, x, u, fl); }
	static std::vector<float> runF32(const std::vector<Coef> &st, const std::vector<double> &x, size_t frames) {
		std::vector<float> c, in(x.begin(), x.end()), out(2 * frames), state(4 * st.size());
		for (double v : flattenD(st)) c.push_back(static_cast<float>(v));
		arm_biquad_cascade_stereo_df2T_instance_f32 S;
		arm_biquad_cascade_stereo_df2T_init_f32(&S, static_cast<uint8_t>(st.size()), c.data(), state.data());
		arm_biquad_cascade_stereo_df2T_f32(&S, in.data(), out.data(), static_cast<uint32_t>(frames));
		return out;
	}
	static const char *inFile() { return "AllBiquadStereoInputs2_f16.txt"; }
	static const char *refFile() { return "AllBiquadStereoRefs2_f16.txt"; }
};

// library side: instance with exact-size guarded state; input and output of "frames" frames (ch samples each)
template <class Form>
struct Dut {
	std::vector<F> coefs;
	dsp::GuardedBuffer<F> state;
	typename Form::Inst S;
	bool outGuard = true;
	size_t ns;
	// tightState: a fresh heap block of exactly stMul*numStages elements (no guard words, no slack): ASan sees any write beyond it
	// (GuardedBuffer keeps its guard words in the SAME allocation, which ASan cannot see)
	std::unique_ptr<F[]> tight;
	explicit Dut(const std::vector<Coef> &st, bool tightState = false) : coefs(toF(flattenD(st))), state(tightState ? 0 : Form::stMul * st.size()), ns(st.size()) {
		if (tightState) tight.reset(new F[Form::stMul * st.size()]);
		Form::init(&S, static_cast<uint8_t>(st.size()), coefs.data(), sp());
	}
	F *sp() { return tight ? tight.get() : state.data(); }
	void reinit() { Form::init(&S, static_cast<uint8_t>(ns), coefs.data(), sp()); }
	std::vector<double> run(const double *in, size_t frames) {
		const size_t n = frames * Form::ch;
		const std::vector<F> fin = toF(std::vector<double>(in, in + n));   // exact size: ASan sees a read beyond the block
		dsp::GuardedBuffer<F> out(n);
		Form::run(&S, fin.data(), out.data(), static_cast<uint32_t>(frames));
		outGuard = outGuard && out.guardIntact();
		return toD(out.data(), n);
	}
	std::vector<double> run(const std::vector<double> &in) { return run(in.data(), in.size() / Form::ch); }
	std::vector<double> stateVals() { return toD(sp(), Form::stMul * ns); }
	bool stateGuard() const { return tight ? true : state.guardIntact(); }
};

const size_t kBlockSizes[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 15, 16, 17, 31, 32, 33};

// ---- C1 ---------------------------------------------------------------------------------------------------------------
std::vector<uint16_t> patBits(const char *name) { return dsp::loadPattern(std::string("Filtering/BIQUAD/BIQUADF16/") + name).f16Bits(); }
std::vector<float> patF(const char *name) { return dsp::loadPattern(std::string("Filtering/BIQUAD/BIQUADF16/") + name).f16AsFloat(); }
std::vector<F> asF(const std::vector<uint16_t> &b) { std::vector<F> f; for (uint16_t v : b) f.push_back(fromBits(v)); return f; }

void armCheck(const std::vector<float> &ref, const std::vector<float> &got) {
	REQUIRE(ref.size() == got.size());
	CHECK_MESSAGE(dsp::snr(ref, got) >= kSnr, "snr " << dsp::snr(ref, got));
	const dsp::Check c = dsp::closeAbsRel(ref, got, kAbsArm, kRelArm);
	CHECK_MESSAGE(c, c.message);
}
std::vector<float> asFloat(const dsp::GuardedBuffer<F> &b) {
	std::vector<float> v;
	const F *p = const_cast<dsp::GuardedBuffer<F> &>(b).data();
	for (size_t i = 0; i < b.size(); i++) v.push_back(static_cast<float>(p[i]));
	return v;
}

template <class Form>
void armRef() {
	const std::vector<F> in = asF(patBits("BiquadInput1_f16.txt")), co = asF(patBits("BiquadCoefs1_f16.txt"));
	const std::vector<float> ref = patF("BiquadOutput1_f16.txt");
	REQUIRE(in.size() == 100);
	REQUIRE(co.size() == 15);
	REQUIRE(ref.size() == 100);
	dsp::GuardedBuffer<F> state(3 * Form::stMul), out(100);
	typename Form::Inst S;
	Form::init(&S, 3, co.data(), state.data());
	Form::run(&S, in.data(), out.data(), 50);
	Form::run(&S, in.data() + 50, out.data() + 50, 50);
	CHECK(out.guardIntact());
	CHECK(state.guardIntact());
	armCheck(ref, asFloat(out));
}

template <class Form>
void armRand() {
	const std::vector<F> in = asF(patBits(Form::inFile())), co = asF(patBits("AllBiquadCoefs2_f16.txt"));
	const std::vector<float> ref = patF(Form::refFile());
	const std::vector<int16_t> cfg = dsp::loadPattern("Filtering/BIQUAD/BIQUADF16/AllBiquadConfigs2_s16.txt").q15();
	REQUIRE(cfg.size() == 18);
	REQUIRE(in.size() == (Form::ch == 2 ? 276u : 138u));
	REQUIRE(ref.size() == in.size());
	REQUIRE(co.size() == 690);
	dsp::GuardedBuffer<F> out(ref.size());
	size_t inOff = 0, coOff = 0;
	bool stateGuards = true;
	for (size_t i = 0; i < cfg.size(); i += 2) {
		const size_t ns = static_cast<size_t>(cfg[i]), bs = static_cast<size_t>(cfg[i + 1]);
		REQUIRE(coOff + 5 * ns <= co.size());
		REQUIRE(inOff + bs * Form::ch <= in.size());
		dsp::GuardedBuffer<F> state(Form::stMul * ns);
		typename Form::Inst S;
		Form::init(&S, static_cast<uint8_t>(ns), co.data() + coOff, state.data());
		Form::run(&S, in.data() + inOff, out.data() + inOff, static_cast<uint32_t>(bs));
		stateGuards = stateGuards && state.guardIntact();
		coOff += 5 * ns;
		inOff += bs * Form::ch;
	}
	CHECK(inOff == in.size());
	CHECK(coOff == 690);
	CHECK(out.guardIntact());
	CHECK(stateGuards);
	armCheck(ref, asFloat(out));
}

// ---- C2 ---------------------------------------------------------------------------------------------------------------
template <class Form>
void guardsAllSizes() {
	Rng g(11);
	bool outOk = true, stOk = true;
	for (int ns = 1; ns <= 9; ns++)
		for (size_t bs : kBlockSizes) {
			const std::vector<Coef> st = randomStages(g, ns);
			Dut<Form> d(st), t(st, true);   // t: tight heap state, same data and partition: bit-identical output
			const std::vector<double> in = randomFrames(g, bs, Form::ch);
			const std::vector<double> out = d.run(in);
			outOk = outOk && d.outGuard && out.size() == bs * Form::ch && t.run(in) == out;
			stOk = stOk && d.stateGuard() && t.stateVals() == d.stateVals();
		}
	CHECK(outOk);
	CHECK(stOk);
}

template <class Form>
void blockZero() {
	Rng g(12);
	Dut<Form> d(randomStages(g, 3));
	const std::vector<double> in = randomFrames(g, 5, Form::ch);
	d.run(in);
	const std::vector<double> before = d.stateVals();
	dsp::GuardedBuffer<F> out(4);
	const std::vector<F> fin = toF(in);
	Form::run(&d.S, fin.data(), out.data(), 0);
	CHECK(out.guardIntact());
	unsigned char poison[4 * sizeof(F)];
	std::memset(poison, 0xCD, sizeof poison);
	CHECK(std::memcmp(out.data(), poison, sizeof poison) == 0);   // nothing written
	CHECK(d.stateVals() == before);
}

// ---- C3 ---------------------------------------------------------------------------------------------------------------
template <class Form>
void defRandom() {
	Rng g(21);
	std::string fail;
	for (int ns = 1; ns <= 9 && fail.empty(); ns++)
		for (size_t bs : kBlockSizes) {
			const std::vector<Coef> st = randomStages(g, ns);
			const std::vector<double> in = randomFrames(g, bs, Form::ch);
			Dut<Form> d(st, true);
			const std::vector<double> lib = d.run(in);
			fail = boundFailure(lib, Form::model(st, in, kU, kFloor16), kDefFactor);
			if (!fail.empty()) { fail = "stages " + std::to_string(ns) + " block " + std::to_string(bs) + " " + fail; break; }
		}
	CHECK_MESSAGE(fail.empty(), fail);
}

template <class Form>
void boundInformative() {
	// a bound that grew without limit would make the DEF test meaningless: 9 stages, 64 frames, inputs in [-1, 1]
	Rng g(22);
	const std::vector<Coef> st = randomStages(g, 9);
	const std::vector<double> in = randomFrames(g, 64, Form::ch);
	const Model m = Form::model(st, in, kU, kFloor16);
	double worst = 0.0;
	for (double e : m.e) worst = std::max(worst, e);
	CHECK_MESSAGE(worst < 0.01, "largest bound " << worst);
}

// impulse (channel 0) and step (channel 1 of the stereo filter; a second run for the mono ones), b = (1, 2, 3), a = (0.5, -0.25):
// impulse 1, 2.5, 4, 1.375, -0.3125 ; step 1, 3.5, 7.5, 8.875, 8.5625 (derivations in the test names' comments below)
template <class Form>
void impulseAndStep() {
	// hand derivation (y[n] = b0 x + b1 x[n-1] + b2 x[n-2] + a1 y[n-1] + a2 y[n-2]):
	//  impulse: y0 = 1; y1 = 2 + 0.5*1 = 2.5; y2 = 3 + 0.5*2.5 - 0.25*1 = 4; y3 = 0.5*4 - 0.25*2.5 = 1.375; y4 = 0.5*1.375 - 0.25*4 = -0.3125
	//  step:    y0 = 1; y1 = 3 + 0.5 = 3.5; y2 = 6 + 0.5*3.5 - 0.25*1 = 7.5; y3 = 6 + 0.5*7.5 - 0.25*3.5 = 8.875; y4 = 6 + 0.5*8.875 - 0.25*7.5 = 8.5625
	//  every value has at most 8 significant bits: exact in binary16, so every operation is exact
	const std::vector<Coef> st = {{1, 2, 3, 0.5, -0.25}};
	const std::vector<double> imp = {1, 0, 0, 0, 0}, stp = {1, 1, 1, 1, 1};
	const std::vector<double> impRef = {1, 2.5, 4, 1.375, -0.3125}, stpRef = {1, 3.5, 7.5, 8.875, 8.5625};
	if (Form::ch == 1) {
		Dut<Form> a(st), b(st);
		CHECK(a.run(imp) == impRef);
		CHECK(b.run(stp) == stpRef);
	} else {
		std::vector<double> in, expect;
		for (size_t i = 0; i < 5; i++) { in.push_back(imp[i]); in.push_back(stp[i]); expect.push_back(impRef[i]); expect.push_back(stpRef[i]); }
		Dut<Form> d(st);
		CHECK(d.run(in) == expect);
	}
}

template <class Form>
void cascadeDelay() {
	// stage 1 gain 2, stage 2 one sample delay: y[n] = 2 x[n-1]
	const std::vector<Coef> st = {{2, 0, 0, 0, 0}, {0, 1, 0, 0, 0}};
	const std::vector<double> x = {1, -3, 5, 0.5, 7, 0};
	const std::vector<double> y = {0, 2, -6, 10, 1, 14};
	std::vector<double> in, expect;
	for (size_t i = 0; i < x.size(); i++) {
		in.push_back(x[i]); expect.push_back(y[i]);
		if (Form::ch == 2) { in.push_back(-x[i]); expect.push_back(-y[i]); }
	}
	Dut<Form> d(st);
	CHECK(d.run(in) == expect);
}

template <class Form>
void exactIntegers() {
	// integers < 2048 and integer coefficients: every product and partial sum is exact in binary16 (11 bit significand), so
	// the library must equal the exact double DEF. Retry (deterministic) until the largest sum of absolute terms is < 2048.
	Rng g(31);
	std::vector<Coef> st;
	std::vector<double> in;
	Model m;
	for (int attempt = 0; attempt < 500; attempt++) {
		st.clear();
		for (int s = 0; s < 2; s++) {
			Coef c;
			c.b0 = g.range(-2, 2); c.b1 = g.range(-2, 2); c.b2 = g.range(-2, 2); c.a1 = g.range(-1, 1); c.a2 = g.range(-1, 1);
			st.push_back(c);
		}
		in.clear();
		for (int i = 0; i < 9 * Form::ch; i++) in.push_back(g.range(-3, 3));
		m = Form::model(st, in, 0.0, 0.0);
		if (m.maxSum < 2048.0) break;
	}
	REQUIRE(m.maxSum < 2048.0);
	Dut<Form> d(st);
	CHECK(d.run(in) == m.y);
}

// exhaustive: every binary16 value (except NaN) as the first sample of a fresh filter; y0 = round(b0 * x) exactly
template <class Form>
void exhaustiveFirstSample() {
	const double b0 = rnd(-0.6);
	const std::vector<Coef> st = {{b0, 0.5, 0.25, 0.125, -0.0625}};
	Dut<Form> d(st);
	size_t bad = 0, total = 0;
	double firstBad = 0;
	for (uint32_t i = 0; i < 65536; i++) {
		const double x = static_cast<double>(fromBits(static_cast<uint16_t>(i)));
		const double xb = static_cast<double>(fromBits(static_cast<uint16_t>((i * 7u + 3u) & 0xFFFFu)));
		if (std::isnan(x) || std::isnan(xb)) continue;
		d.reinit();
		std::vector<double> in = {x};
		if (Form::ch == 2) in.push_back(xb);
		const std::vector<double> y = d.run(in);
		// the product of two binary16 values has at most 22 bits: exact in double, so one correct rounding is the DEF
		bool ok = y[0] == rnd(b0 * x);
		if (Form::ch == 2) ok = ok && y[1] == rnd(b0 * xb);
		total++;
		if (!ok) { if (!bad) firstBad = x; bad++; }
	}
	CHECK_MESSAGE(bad == 0, bad << " of " << total << " differ, first x " << firstBad);
}

// strided pairs of binary16 values (|x| <= 30000) through a one stage filter, bound of the model
template <class Form>
void stridedPairs() {
	const std::vector<Coef> st = {{0.5, 0.25, -0.125, 0.125, -0.0625}};
	size_t bad = 0, total = 0;
	std::string first;
	Dut<Form> d(st);
	for (uint32_t i = 0; i < 0x7800; i += 509)
		for (uint32_t j = 0; j < 0x7800; j += 521)
			for (uint32_t sg = 0; sg < 4; sg++) {
				const double s0 = (sg & 1) ? -1.0 : 1.0, s1 = (sg & 2) ? -1.0 : 1.0;
				const double x0 = s0 * static_cast<double>(fromBits(static_cast<uint16_t>(i)));
				const double x1 = s1 * static_cast<double>(fromBits(static_cast<uint16_t>(j)));
				std::vector<double> in;
				if (Form::ch == 1) { in.push_back(x0); in.push_back(x1); }
				else { in.push_back(x0); in.push_back(x1); in.push_back(x1); in.push_back(x0); }   // frames (x0, x1), (x1, x0)
				d.reinit();
				const std::vector<double> lib = d.run(in);
				const std::string f = boundFailure(lib, Form::model(st, in, kU, kFloor16), kDefFactor);
				total++;
				if (!f.empty()) { if (!bad) first = f; bad++; }
			}
	CHECK_MESSAGE(bad == 0, bad << " of " << total << " outside the bound, first " << first);
}

// ---- C4 ---------------------------------------------------------------------------------------------------------------
template <class Form>
void initFields() {
	for (int ns = 1; ns <= 9; ns++) {
		Rng g(40 + ns);
		const std::vector<F> co = toF(flattenD(randomStages(g, ns)));
		dsp::GuardedBuffer<F> state(Form::stMul * static_cast<size_t>(ns));
		typename Form::Inst S;
		std::memset(static_cast<void *>(&S), 0xAB, sizeof S);
		Form::init(&S, static_cast<uint8_t>(ns), co.data(), state.data());
		CHECK(S.numStages == static_cast<unsigned>(ns));
		CHECK(S.pCoeffs == co.data());
		CHECK(S.pState == state.data());
	}
}

template <class Form>
void initZeroes() {
	for (int ns = 1; ns <= 9; ns++) {
		Rng g(50 + ns);
		const std::vector<F> co = toF(flattenD(randomStages(g, ns)));
		dsp::GuardedBuffer<F> state(Form::stMul * static_cast<size_t>(ns));   // poisoned with 0xCD bytes by the constructor
		typename Form::Inst S;
		Form::init(&S, static_cast<uint8_t>(ns), co.data(), state.data());
		bool zero = true;
		for (size_t i = 0; i < state.size(); i++) zero = zero && static_cast<double>(state.data()[i]) == 0.0;
		CHECK_MESSAGE(zero, "numStages " << ns);
		CHECK_MESSAGE(state.guardIntact(), "numStages " << ns);
	}
}

template <class Form>
void reinitResets() {
	Rng g(61);
	const std::vector<Coef> st = randomStages(g, 3);
	const std::vector<double> in = randomFrames(g, 20, Form::ch);
	Dut<Form> d(st);
	const std::vector<double> first = d.run(in);
	const std::vector<double> second = d.run(in);
	CHECK(second != first);   // the state is kept between calls (guards this test)
	d.reinit();
	CHECK(d.run(in) == first);
}

template <class Form>
void coefsUntouched() {
	Rng g(66);
	const std::vector<Coef> st = randomStages(g, 5);
	const std::vector<double> in = randomFrames(g, 33, Form::ch);
	Dut<Form> d(st);
	const std::vector<F> copy = d.coefs;
	d.run(in);
	CHECK(std::memcmp(copy.data(), d.coefs.data(), copy.size() * sizeof(F)) == 0);
}

template <class Form>
void splitBlocks() {
	Rng g(63);
	const std::vector<Coef> st = randomStages(g, 4);
	const std::vector<double> in = randomFrames(g, 100, Form::ch);
	// different partitions run through the unrolled and the tail code: compared with the DEF bound, not bitwise
	const Model m = Form::model(st, in, kU, kFloor16);
	const size_t splits[] = {1, 2, 3, 4, 5, 7, 8, 9, 16, 17};
	for (size_t sp : splits) {
		Dut<Form> d(st, true);
		std::vector<double> got;
		for (size_t pos = 0; pos < 100; pos += sp) {
			const size_t n = std::min<size_t>(sp, 100 - pos);
			const std::vector<double> part = d.run(in.data() + pos * Form::ch, n);
			got.insert(got.end(), part.begin(), part.end());
		}
		const std::string f = boundFailure(got, m, kDefFactor);
		CHECK_MESSAGE(f.empty(), "block size " << sp << " " << f);
	}
}

template <class Form>
void mixedBlocks() {
	Rng g(64);
	const std::vector<Coef> st = randomStages(g, 2);
	const std::vector<double> in = randomFrames(g, 38, Form::ch);
	const Model m = Form::model(st, in, kU, kFloor16);
	const size_t sizes[] = {3, 1, 16, 5, 2, 7, 4};
	Dut<Form> d(st, true);
	std::vector<double> got;
	size_t pos = 0;
	for (size_t n : sizes) {
		const std::vector<double> part = d.run(in.data() + pos * Form::ch, n);
		got.insert(got.end(), part.begin(), part.end());
		pos += n;
	}
	REQUIRE(pos == 38);
	const std::string f = boundFailure(got, m, kDefFactor);
	CHECK_MESSAGE(f.empty(), f);
}

template <class Form>
void twoInstances() {
	Rng g(65);
	const std::vector<Coef> st = randomStages(g, 3);
	const std::vector<double> inA = randomFrames(g, 24, Form::ch), inB = randomFrames(g, 24, Form::ch);
	Dut<Form> soloA(st), soloB(st);
	std::vector<double> refA, refB;   // same partition (blocks of 6) as the interleaved runs: bit-identical results expected
	for (size_t pos = 0; pos < 24; pos += 6) {
		const std::vector<double> pa = soloA.run(inA.data() + pos * Form::ch, 6), pb = soloB.run(inB.data() + pos * Form::ch, 6);
		refA.insert(refA.end(), pa.begin(), pa.end());
		refB.insert(refB.end(), pb.begin(), pb.end());
	}
	Dut<Form> a(st), b(st);   // equal coefficient VALUES in two arrays; the shared-array case is below
	std::vector<double> gotA, gotB;
	for (size_t pos = 0; pos < 24; pos += 6) {
		const std::vector<double> pa = a.run(inA.data() + pos * Form::ch, 6);
		gotA.insert(gotA.end(), pa.begin(), pa.end());
		const std::vector<double> pb = b.run(inB.data() + pos * Form::ch, 6);
		gotB.insert(gotB.end(), pb.begin(), pb.end());
	}
	CHECK(gotA == refA);
	CHECK(gotB == refB);
	// two instances on ONE coefficient array (DOC: "coefficient arrays may be shared among several instances")
	dsp::GuardedBuffer<F> stateB(Form::stMul * 3);
	typename Form::Inst SB;
	Form::init(&SB, 3, a.coefs.data(), stateB.data());
	a.reinit();
	std::vector<double> gA, gB;
	for (size_t pos = 0; pos < 24; pos += 6) {
		const std::vector<double> pa = a.run(inA.data() + pos * Form::ch, 6);
		gA.insert(gA.end(), pa.begin(), pa.end());
		const std::vector<F> fin = toF(std::vector<double>(inB.data() + pos * Form::ch, inB.data() + (pos + 6) * Form::ch));
		dsp::GuardedBuffer<F> ob(6 * Form::ch);
		Form::run(&SB, fin.data(), ob.data(), 6);
		const std::vector<double> pb = toD(ob.data(), 6 * Form::ch);
		gB.insert(gB.end(), pb.begin(), pb.end());
	}
	CHECK(gA == refA);
	CHECK(gB == refB);
}

template <class Form>
void identityStages() {
	Rng g(67);
	for (int ns = 1; ns <= 9; ns++) {
		const std::vector<Coef> st(static_cast<size_t>(ns), Coef{1, 0, 0, 0, 0});
		const std::vector<double> in = randomFrames(g, 19, Form::ch);
		Dut<Form> d(st);
		CHECK_MESSAGE(d.run(in) == in, "stages " << ns);
	}
}

// ---- C5 ---------------------------------------------------------------------------------------------------------------
template <class Form>
void agreesWithF32() {
	// both sides see the identical (binary16 exact) coefficients and inputs; |f16 - def| <= E16 and |f32 - def| <= E32
	Rng g(70);
	std::string fail;
	for (int ns = 1; ns <= 5 && fail.empty(); ns++)
		for (size_t bs : {1, 4, 5, 9, 17, 40}) {
			const std::vector<Coef> st = randomStages(g, ns);
			const std::vector<double> in = randomFrames(g, bs, Form::ch);
			Dut<Form> d(st);
			const std::vector<double> lib16 = d.run(in);
			const std::vector<float> lib32 = Form::runF32(st, in, bs);
			const Model m16 = Form::model(st, in, kU, kFloor16), m32 = Form::model(st, in, kU32, 0.0);
			for (size_t i = 0; i < lib16.size(); i++) {
				const double diff = std::fabs(lib16[i] - static_cast<double>(lib32[i]));
				const double bound = kDefFactor * m16.e[i] + m32.e[i] * 1.0001;
				if (!(diff <= bound)) {
					fail = "stages " + std::to_string(ns) + " block " + std::to_string(bs) + " sample " + std::to_string(i) + ": f16 " + std::to_string(lib16[i]) + " f32 " + std::to_string(lib32[i]);
					break;
				}
			}
			if (!fail.empty()) break;
		}
	CHECK_MESSAGE(fail.empty(), fail);
}

#define FORM_CASES(REQ, TEXT, FN)                                                         \
	TEST_CASE("[" REQ " arm_biquad_cascade_df1_f16] " TEXT) { FN<Df1>(); }                \
	TEST_CASE("[" REQ " arm_biquad_cascade_df2T_f16] " TEXT) { FN<Df2T>(); }              \
	TEST_CASE("[" REQ " arm_biquad_cascade_stereo_df2T_f16] " TEXT) { FN<Stereo>(); }

#define INIT_CASES(REQ, TEXT, FN)                                                         \
	TEST_CASE("[" REQ " arm_biquad_cascade_df1_init_f16] " TEXT) { FN<Df1>(); }           \
	TEST_CASE("[" REQ " arm_biquad_cascade_df2T_init_f16] " TEXT) { FN<Df2T>(); }         \
	TEST_CASE("[" REQ " arm_biquad_cascade_stereo_df2T_init_f16] " TEXT) { FN<Stereo>(); }

}  // namespace

TEST_SUITE("DSP Filtering f16") {

// ======================================================= C1 =========================================================
TEST_CASE("[C1 arm_biquad_cascade_df1_f16] ARM id 1: the 3 stage filter over two blocks of 50 matches the reference (test_biquad_cascade_df1_ref)") { armRef<Df1>(); }
TEST_CASE("[C1 arm_biquad_cascade_df2T_f16] ARM id 2: the 3 stage filter over two blocks of 50 matches the reference (test_biquad_cascade_df2T_ref)") { armRef<Df2T>(); }
TEST_CASE("[C1 arm_biquad_cascade_df1_f16] ARM id 3: the 9 random configurations match the references (test_biquad_cascade_df1_rand)") { armRand<Df1>(); }
TEST_CASE("[C1 arm_biquad_cascade_df2T_f16] ARM id 4: the 9 random configurations match the references (test_biquad_cascade_df2T_rand)") { armRand<Df2T>(); }
TEST_CASE("[C1 arm_biquad_cascade_stereo_df2T_f16] ARM id 5: the 9 random stereo configurations match the references (test_biquad_cascade_stereo_df2T_rand)") { armRand<Stereo>(); }

// ======================================================= C2 =========================================================
FORM_CASES("C2", "1 to 9 stages and block sizes 1 to 33 write only the output block and only the exact-size state buffer", guardsAllSizes)
FORM_CASES("C2", "characterization: block size 0 leaves the output and the state unchanged (DOC is silent)", blockZero)

// ======================================================= C3 =========================================================
FORM_CASES("C3", "1 to 9 stages and block sizes 1 to 33 agree with the double DEF within the running error bound", defRandom)
FORM_CASES("C3", "the error bound is informative: below 0.01 for a 9 stage cascade of inputs in [-1, 1]", boundInformative)
FORM_CASES("C3", "the impulse response 1, 2.5, 4, 1.375, -0.3125 and the step response 1, 3.5, 7.5, 8.875, 8.5625 of b = (1, 2, 3), a = (0.5, -0.25) are exact", impulseAndStep)
FORM_CASES("C3", "a gain 2 stage followed by a delay stage gives 2 x[n-1] (cascade order of the stages)", cascadeDelay)
FORM_CASES("C3", "integer data and coefficients (2 stages, 9 frames) give exactly the integer DEF result", exactIntegers)
FORM_CASES("C3", "all non-NaN binary16 values as first sample give exactly round(b0 * x) (exhaustive sweep)", exhaustiveFirstSample)
FORM_CASES("C3", "a strided sweep over pairs of binary16 values up to 30000 stays within the running error bound", stridedPairs)

// ======================================================= C4 =========================================================
INIT_CASES("C4", "init stores numStages, pCoeffs and pState", initFields)
INIT_CASES("C4", "init zeroes exactly the state elements of the DOC length (poisoned buffer, guard words intact)", initZeroes)
FORM_CASES("C4", "initialising again resets the state, without init the state is kept between calls", reinitResets)
FORM_CASES("C4", "processing does not modify the coefficients", coefsUntouched)
FORM_CASES("C4", "the same samples split into blocks of 1, 2, 3, 4, 5, 7, 8, 9, 16 and 17 agree with the double DEF within the running error bound", splitBlocks)
FORM_CASES("C4", "mixed block sizes (3, 1, 16, 5, 2, 7, 4) agree with the double DEF within the running error bound", mixedBlocks)
FORM_CASES("C4", "two instances (own and shared coefficient arrays) are independent when their calls are interleaved", twoInstances)
FORM_CASES("C4", "stages with b = (1, 0, 0) and a = (0, 0) are the identity for 1 to 9 stages", identityStages)

TEST_CASE("[C4 arm_biquad_cascade_df1_f16] the state after one impulse sample is {x1, x2, y1, y2} = {1, 0, 1, 0}, after a zero sample {0, 1, 2.5, 1}") {
	// DOC (init): state {x[n-1], x[n-2], y[n-1], y[n-2]} per stage; b = (1, 2, 3), a = (0.5, -0.25): y0 = 1; then x = 0: y1 = 2 + 0.5 = 2.5
	const std::vector<Coef> st = {{1, 2, 3, 0.5, -0.25}};
	Dut<Df1> d(st);
	d.run(std::vector<double>{1});
	CHECK(d.stateVals() == std::vector<double>{1, 0, 1, 0});
	d.run(std::vector<double>{0});
	CHECK(d.stateVals() == std::vector<double>{0, 1, 2.5, 1});
}

TEST_CASE("[C4 arm_biquad_cascade_df1_f16] the state of stage 2 follows the 4 values of stage 1") {
	// stage 1 as above (state {1, 0, 1, 0}, output 1); stage 2 b = (1, 1, 0), no feedback: input 1, output 1, state {1, 0, 1, 0}
	const std::vector<Coef> st = {{1, 2, 3, 0.5, -0.25}, {1, 1, 0, 0, 0}};
	Dut<Df1> d(st);
	d.run(std::vector<double>{1});
	CHECK(d.stateVals() == std::vector<double>{1, 0, 1, 0, 1, 0, 1, 0});
}

TEST_CASE("[C4 arm_biquad_cascade_df2T_f16] the state after one impulse sample is {d1, d2} = {2.5, 2.75}, with a second stage {2.5, 2.75, 1, 0}") {
	// DOC: state {d11, d12, d21, d22}; y0 = 1; d1 = b1 + a1 y0 = 2.5; d2 = b2 + a2 y0 = 2.75; stage 2 b = (1, 1, 0): y = 1, d1 = 1, d2 = 0
	Dut<Df2T> one({{1, 2, 3, 0.5, -0.25}});
	one.run(std::vector<double>{1});
	CHECK(one.stateVals() == std::vector<double>{2.5, 2.75});
	Dut<Df2T> two({{1, 2, 3, 0.5, -0.25}, {1, 1, 0, 0, 0}});
	two.run(std::vector<double>{1});
	CHECK(two.stateVals() == std::vector<double>{2.5, 2.75, 1, 0});
}

TEST_CASE("[C4 arm_biquad_cascade_stereo_df2T_f16] characterization: the state layout of one stage is {d1a, d2a, d1b, d2b} (DOC gives no order)") {
	// impulse in channel a only, channel b = 0: d1a = 2.5, d2a = 2.75 (as the mono filter), d1b = d2b = 0
	Dut<Stereo> d({{1, 2, 3, 0.5, -0.25}});
	d.run(std::vector<double>{1, 0});
	CHECK(d.stateVals() == std::vector<double>{2.5, 2.75, 0, 0});
}

TEST_CASE("[C4 arm_biquad_cascade_stereo_df2T_f16] a signal in channel a leaves channel b exactly zero and the reverse") {
	Rng g(80);
	const std::vector<Coef> st = randomStages(g, 3);
	const std::vector<double> sig = randomFrames(g, 20, 1);
	std::vector<double> inA, inB;
	for (double v : sig) { inA.push_back(v); inA.push_back(0.0); inB.push_back(0.0); inB.push_back(v); }
	Dut<Stereo> da(st), db(st);
	const std::vector<double> oa = da.run(inA), ob = db.run(inB);
	bool zeroB = true, zeroA = true, same = true;
	for (size_t i = 0; i < sig.size(); i++) {
		zeroB = zeroB && oa[2 * i + 1] == 0.0;
		zeroA = zeroA && ob[2 * i] == 0.0;
		same = same && oa[2 * i] == ob[2 * i + 1];
	}
	CHECK(zeroB);
	CHECK(zeroA);
	CHECK(same);   // both channels use the same arithmetic
}

// ======================================================= C5 =========================================================
FORM_CASES("C5", "the f16 output agrees with the f32 function of the same input within both running error bounds", agreesWithF32)

}  // TEST_SUITE
