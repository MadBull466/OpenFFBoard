/*
 * Filtering, biquad f64 (phase P11, group G4, work order FB6416): arm_biquad_cascade_df2T_f64 and
 * arm_biquad_cascade_df2T_init_f64 (Source/FilteringFunctions). These are the only f64 biquad functions of the library
 * (there is no df1 or stereo f64). Requirements C1-C6 of Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md.
 *
 * ARM's suite "BIQUAD F64" (Testing/desc.txt:3246-3266, test bodies Testing/Source/Tests/BIQUADF64.cpp) has two ids, both
 * enabled (none is commented out or disabled):
 *   1 test_biquad_cascade_df2T_ref   BiquadInput1_f64 (100) + BiquadCoefs1_f64 (3 stages = 15) -> BiquadOutput1_f64 (100);
 *                                    init with 3 stages, two calls of blockSize 50 (BIQUADF64.cpp:15-75)
 *   2 test_biquad_cascade_df2T_rand  AllBiquadInputs2_f64 (30) + AllBiquadCoefs2_f64 (150) + AllBiquadConfigs2_s16 (18 =
 *                                    9 pairs numStages, blockSize: (1,1) (1,4) (1,5) (4,1) (4,4) (4,5) (5,1) (5,4) (5,5))
 *                                    -> AllBiquadRefs2_f64 (30); every config is a re-init with its own coefficients and ONE
 *                                    call of blockSize (BIQUADF64.cpp:79-148; the "2 blocks" comment there is stale)
 *   (patterns in Testing/Patterns/DSP/Filtering/BIQUAD/BIQUADF64/)
 * ARM tolerances (BIQUADF64.cpp:5 and :13): SNR_THRESHOLD 98 dB, REL_ERROR 1.2e-3, REL_ERROR is ARM's symmetric relative
 * error |a-b| / ((|a|+|b|)/2) (Testing/FrameworkSource/Error.cpp:198-212); ASSERT_EMPTY_TAIL on the output. The reference is
 * a double computation (BIQUADF64.cpp:7-12). Used unchanged (L4); the per-sample check is stricter than ARM only in that NaN
 * fails. ARM's state buffer is 128 elements; here it is EXACTLY 2*numStages (the DOC length) inside a guarded buffer.
 *
 * Convention (DOC of arm_biquad_cascade_df2T_f64.c): y[n] = b0 x[n] + d1; d1 = b1 x[n] + a1 y[n] + d2;
 * d2 = b2 x[n] + a2 y[n]; coefficients {b0, b1, b2, a1, a2} per stage, a1 and a2 enter with a PLUS sign (design tools
 * that flip the sign must negate them); state {d11, d12, d21, d22, ...} (2 values per stage, 2*numStages in all); the
 * output of a stage is the input of the next one; init "sets the instance fields, zeros the state buffer"; "the
 * coefficients are untouched".
 *
 * C2 exact-size state and output guards (plus a tight heap state buffer without slack in the C2, C3 and split tests, so ASan sees
 *    a write past the state), block size 0 (characterization: DOC silent), input read exactly blockSize samples (exact-size vectors).
 * C3 DEF in double with a RUNNING ERROR BOUND (see below); hand-derived impulse and step responses; exact integer data;
 *    strided exponent sweep over the whole normal range.
 * C4 init fields and state clear (poisoned buffer), re-init resets the state, continuity over block splits (1 2 3 4 5 7 ...
 *    not multiples of the unroll 16), two instances independent, coefficients untouched.
 * C5 agreement with arm_biquad_cascade_df2T_f32 (the f32 function of the same input, same coefficients).
 *
 * Running error bound (derivation). u = 2^-53. For one sample of one stage, every term of an evaluated sum with k
 * roundings carries a relative error <= gam(k) = k u / (1 - k u) (Higham). With x the stage input (value x, error bound
 * Ex), the double DEF values y, d1, d2 and their bounds Ey, Ed1, Ed2 (from the previous samples), the error of the
 * library value is, to first order:
 *   y  = b0 x + d1:            Ey'  = gam(2) (|b0|(|x|+Ex) + |d1|+Ed1) + |b0| Ex + Ed1
 *   d1 = b1 x + a1 y + d2:     Ed1' = gam(4) (|b1|(|x|+Ex) + |a1|(|y|+Ey) + |d2|+Ed2) + |b1| Ex + |a1| Ey + Ed2
 *   d2 = b2 x + a2 y:          Ed2' = gam(3) (|b2|(|x|+Ex) + |a2|(|y|+Ey)) + |b2| Ex + |a2| Ey
 * (2, 3 and 4 are the real counts of products and additions per term plus one spare for the store; the |.|+E terms are
 * the library's own values, so the bound is rigorous at first order and the second order terms are < 1e-14 relative).
 * The stage output bound Ey' is the input bound of the next stage. A product contracted into an FMA has fewer roundings,
 * so the bound also covers FMA contraction. The DEF itself is evaluated in double with the same error model, so the
 * comparison tolerance is 2 E (library error + DEF error). The data are chosen so that |a1|+|a2| <= 0.35 and |b| <= 0.3: the
 * bound recursion contracts (gain 1/(1-0.35) = 1.54 per stage) and the bound stays informative (about 1e-15 relative).
 *
 * Host vs target (C6): the host runs the portable C path (the NEON aarch64 variant would sum in a different order but is
 * covered by the same bound); the Cortex-M4F target computes double in software (newlib, no FMA contraction, IEEE
 * round to nearest), the same C code with the same results; no float to integer conversion, no shifts, no struct
 * packing involved (the instance has uint8_t, then two pointers: 12 bytes on target, 24 on the host; only its fields are
 * tested). The exact tests do not depend on contraction; the block-partition tests (different partitions run through the unrolled
 * and the tail code) use the DEF bound, not bit equality, because FMA contraction or excess precision could differ per code path; only runs
 * with identical partitions (two instances, re-init) are compared bitwise.
 * numStages = 0 is not tested (the do/while loop would run 2^32 stages): NEEDS REVIEW, the DOC does not exclude it.
 */
#include "doctest.h"

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr double kSnr = 98.0;         // BIQUADF64.cpp:5
constexpr double kRel = 1.2e-3;       // BIQUADF64.cpp:13
constexpr double kU = 1.1102230246251565e-16;   // 2^-53
constexpr double kU32 = 5.9604644775390625e-8;  // 2^-24

double gam(double k, double u) { return k * u / (1.0 - k * u); }

std::vector<double> pat(const char *name) {
	return dsp::loadPattern(std::string("Filtering/BIQUAD/BIQUADF64/") + name).f64();
}

// ARM's Client::assert_relative_error (Error.cpp:198-212); NaN fails here (ARM would let it pass)
bool armRelOk(double a, double b, double thr) {
	if (std::isnan(a) || std::isnan(b)) return false;
	const double average = (std::fabs(a) + std::fabs(b)) / 2.0;
	if (average == 0.0) return true;
	return std::fabs(a - b) / average <= thr;
}

struct Rng {
	uint64_t s;
	explicit Rng(uint64_t seed) : s(seed) {}
	uint64_t next() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
	double uni() { const uint64_t r = next(); return static_cast<double>(r >> 11) * (2.0 / 9007199254740992.0) - 1.0; }   // [-1, 1)
	int range(int lo, int hi) { const uint64_t r = next(); return lo + static_cast<int>(r % static_cast<uint64_t>(hi - lo + 1)); }
};

struct Coef { double b0, b1, b2, a1, a2; };

// stable stages with |a1| <= 0.2, |a2| <= 0.15, |b| <= 0.3
std::vector<Coef> randomStages(Rng &g, int ns) {
	std::vector<Coef> st;
	for (int i = 0; i < ns; i++) {
		Coef c;
		c.b0 = 0.3 * g.uni();
		c.b1 = 0.3 * g.uni();
		c.b2 = 0.3 * g.uni();
		c.a1 = 0.2 * g.uni();
		c.a2 = 0.15 * g.uni();
		st.push_back(c);
	}
	return st;
}
std::vector<double> randomSignal(Rng &g, size_t n) {
	std::vector<double> x;
	for (size_t i = 0; i < n; i++) { const double v = g.uni(); x.push_back(v); }
	return x;
}
std::vector<double> flatten(const std::vector<Coef> &st) {
	std::vector<double> f;
	for (const Coef &c : st) { f.push_back(c.b0); f.push_back(c.b1); f.push_back(c.b2); f.push_back(c.a1); f.push_back(c.a2); }
	return f;
}

// ---- DEF in double with the running error bound (derivation in the header) --------------------------------------------
struct Model {
	std::vector<double> y, e;   // output and error bound per sample
	double maxSum = 0.0;        // largest sum of absolute terms seen (for the exact tests)
};

Model modelDf2T(const std::vector<Coef> &st, const std::vector<double> &x, double u) {
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
			const double Ey = g2 * s0 + std::fabs(c.b0) * ex + E1;
			const double d1n = c.b1 * xi + c.a1 * y + d2;
			const double s1 = std::fabs(c.b1) * xa + std::fabs(c.a1) * (std::fabs(y) + Ey) + std::fabs(d2) + E2;
			const double E1n = g4 * s1 + std::fabs(c.b1) * ex + std::fabs(c.a1) * Ey + E2;
			const double d2n = c.b2 * xi + c.a2 * y;
			const double s2 = std::fabs(c.b2) * xa + std::fabs(c.a2) * (std::fabs(y) + Ey);
			const double E2n = g3 * s2 + std::fabs(c.b2) * ex + std::fabs(c.a2) * Ey;
			m.maxSum = std::max(m.maxSum, std::max(s0, std::max(s1, s2)));
			d1 = d1n; d2 = d2n; E1 = E1n; E2 = E2n;
			m.y[i] = y; m.e[i] = Ey;
		}
	}
	return m;
}

// first sample outside |lib - def| <= factor * bound, as text (empty if none)
std::string boundFailure(const std::vector<double> &lib, const Model &m, double factor) {
	if (lib.size() != m.y.size()) return "length mismatch";
	for (size_t i = 0; i < lib.size(); i++) {
		const double err = std::fabs(lib[i] - m.y[i]);
		if (std::isnan(lib[i]) || !(err <= factor * m.e[i])) {
			char buf[160];
			std::snprintf(buf, sizeof buf, "sample %zu: lib %.17g def %.17g |diff| %.3g bound %.3g", i, lib[i], m.y[i], err, factor * m.e[i]);
			return buf;
		}
	}
	return std::string();
}

// ---- library side: instance with an exact-size guarded state buffer --------------------------------------------------
struct Dut {
	std::vector<double> coefs;
	dsp::GuardedBuffer<double> state;
	arm_biquad_cascade_df2T_instance_f64 S;
	bool outGuard = true;
	// tightState: the state buffer is a fresh heap block of exactly 2*numStages elements (no guard words, no slack), so ASan
	// sees any write beyond it (GuardedBuffer keeps its guard words in the SAME allocation: ASan cannot see those)
	std::unique_ptr<double[]> tight;
	size_t ns;
	explicit Dut(const std::vector<Coef> &st, bool tightState = false) : coefs(flatten(st)), state(tightState ? 0 : 2 * st.size()), ns(st.size()) {
		if (tightState) tight.reset(new double[2 * st.size()]);
		arm_biquad_cascade_df2T_init_f64(&S, static_cast<uint8_t>(st.size()), coefs.data(), sp());
	}
	double *sp() { return tight ? tight.get() : state.data(); }
	std::vector<double> stateVals() { return std::vector<double>(sp(), sp() + 2 * ns); }
	// processes n samples starting at in; the output has its own guard words
	std::vector<double> run(const double *in, size_t n) {
		dsp::GuardedBuffer<double> out(n);
		arm_biquad_cascade_df2T_f64(&S, in, out.data(), static_cast<uint32_t>(n));
		outGuard = outGuard && out.guardIntact();
		return out.output();
	}
	std::vector<double> run(const std::vector<double> &in) { return run(in.data(), in.size()); }
	bool stateGuard() const { return tight ? true : state.guardIntact(); }
};

// ---- C1 -------------------------------------------------------------------------------------------------------------
void armCheck(const std::vector<double> &ref, const std::vector<double> &got) {
	REQUIRE(ref.size() == got.size());
	CHECK_MESSAGE(dsp::snr(ref, got) >= kSnr, "snr " << dsp::snr(ref, got));
	for (size_t i = 0; i < ref.size(); i++) CHECK_MESSAGE(armRelOk(got[i], ref[i], kRel), "sample " << i << " got " << got[i] << " ref " << ref[i]);
}

void armRef() {
	const std::vector<double> in = pat("BiquadInput1_f64.txt"), co = pat("BiquadCoefs1_f64.txt"), ref = pat("BiquadOutput1_f64.txt");
	REQUIRE(in.size() == 100);
	REQUIRE(co.size() == 15);
	REQUIRE(ref.size() == 100);
	dsp::GuardedBuffer<double> state(6), out(100);
	arm_biquad_cascade_df2T_instance_f64 S;
	arm_biquad_cascade_df2T_init_f64(&S, 3, co.data(), state.data());
	arm_biquad_cascade_df2T_f64(&S, in.data(), out.data(), 50);
	arm_biquad_cascade_df2T_f64(&S, in.data() + 50, out.data() + 50, 50);
	CHECK(out.guardIntact());
	CHECK(state.guardIntact());
	armCheck(ref, out.output());
}

void armRand() {
	const std::vector<double> in = pat("AllBiquadInputs2_f64.txt"), co = pat("AllBiquadCoefs2_f64.txt"), ref = pat("AllBiquadRefs2_f64.txt");
	const std::vector<int16_t> cfg = dsp::loadPattern("Filtering/BIQUAD/BIQUADF64/AllBiquadConfigs2_s16.txt").q15();
	REQUIRE(cfg.size() == 18);
	REQUIRE(in.size() == 30);
	REQUIRE(ref.size() == 30);
	REQUIRE(co.size() == 150);
	dsp::GuardedBuffer<double> out(ref.size());
	size_t inOff = 0, coOff = 0;
	bool stateGuards = true;
	for (size_t i = 0; i < cfg.size(); i += 2) {
		const int ns = cfg[i], bs = cfg[i + 1];
		REQUIRE(coOff + 5 * static_cast<size_t>(ns) <= co.size());
		REQUIRE(inOff + static_cast<size_t>(bs) <= in.size());
		dsp::GuardedBuffer<double> state(2 * static_cast<size_t>(ns));
		arm_biquad_cascade_df2T_instance_f64 S;
		arm_biquad_cascade_df2T_init_f64(&S, static_cast<uint8_t>(ns), co.data() + coOff, state.data());
		arm_biquad_cascade_df2T_f64(&S, in.data() + inOff, out.data() + inOff, static_cast<uint32_t>(bs));
		stateGuards = stateGuards && state.guardIntact();
		coOff += 5 * static_cast<size_t>(ns);
		inOff += static_cast<size_t>(bs);
	}
	CHECK(inOff == 30);
	CHECK(coOff == 150);
	CHECK(out.guardIntact());
	CHECK(stateGuards);
	armCheck(ref, out.output());
}

// ---- helpers for the other tests ---------------------------------------------------------------------------------
const size_t kBlockSizes[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 15, 16, 17, 31, 32, 33};

}  // namespace

TEST_SUITE("DSP Filtering f64") {

// ======================================================= C1 =========================================================
TEST_CASE("[C1 arm_biquad_cascade_df2T_f64] ARM id 1: the 3 stage filter over two blocks of 50 matches the reference (test_biquad_cascade_df2T_ref)") { armRef(); }
TEST_CASE("[C1 arm_biquad_cascade_df2T_f64] ARM id 2: the 9 random configurations (1, 4, 5 stages; blocks 1, 4, 5) match the references (test_biquad_cascade_df2T_rand)") { armRand(); }

// ======================================================= C2 =========================================================
TEST_CASE("[C2 arm_biquad_cascade_df2T_f64] 1 to 9 stages and block sizes 1 to 33 write only the output block and only the 2*numStages state elements") {
	Rng g(11);
	bool outOk = true, stOk = true;
	for (int ns = 1; ns <= 9; ns++)
		for (size_t bs : kBlockSizes) {
			const std::vector<Coef> st = randomStages(g, ns);
			Dut d(st), t(st, true);   // t: tight heap state, same data: identical partition, so bit-identical output
			const std::vector<double> in = randomSignal(g, bs);   // exact size: a read beyond blockSize is an ASan finding
			const std::vector<double> out = d.run(in);
			outOk = outOk && d.outGuard && out.size() == bs && t.run(in) == out;
			stOk = stOk && d.stateGuard() && t.stateVals() == d.stateVals();
		}
	CHECK(outOk);
	CHECK(stOk);
}

TEST_CASE("[C2 arm_biquad_cascade_df2T_f64] characterization: block size 0 leaves the output and the state unchanged (DOC is silent)") {
	Rng g(12);
	Dut d(randomStages(g, 3));
	const std::vector<double> in = randomSignal(g, 5);
	d.run(in);
	const std::vector<double> before = d.stateVals();
	dsp::GuardedBuffer<double> out(4);
	arm_biquad_cascade_df2T_f64(&d.S, in.data(), out.data(), 0);
	CHECK(out.guardIntact());
	const std::vector<double> poison(4, [] { double p; unsigned char b[sizeof p]; std::memset(b, 0xCD, sizeof b); std::memcpy(&p, b, sizeof p); return p; }());
	// the output buffer still holds the 0xCD poison bytes (compared bytewise, the pattern is a NaN)
	std::vector<double> o = out.output();
	CHECK(std::memcmp(o.data(), poison.data(), 4 * sizeof(double)) == 0);
	CHECK(d.stateVals() == before);
}

// ======================================================= C3 =========================================================
TEST_CASE("[C3 arm_biquad_cascade_df2T_f64] 1 to 9 stages and block sizes 1 to 33 agree with the double DEF within twice the running error bound") {
	Rng g(21);
	std::string fail;
	for (int ns = 1; ns <= 9 && fail.empty(); ns++)
		for (size_t bs : kBlockSizes) {
			const std::vector<Coef> st = randomStages(g, ns);
			const std::vector<double> in = randomSignal(g, bs);
			Dut d(st, true);
			const std::vector<double> lib = d.run(in);
			const Model m = modelDf2T(st, in, kU);
			fail = boundFailure(lib, m, 2.0);
			if (!fail.empty()) { fail = "stages " + std::to_string(ns) + " block " + std::to_string(bs) + " " + fail; break; }
		}
	CHECK_MESSAGE(fail.empty(), fail);
}

TEST_CASE("[C3 arm_biquad_cascade_df2T_f64] the error bound is informative: it stays below 1e-13 of the signal for a 9 stage cascade") {
	// guards the guard: a bound that grew without limit would make the DEF test above meaningless
	Rng g(22);
	const std::vector<Coef> st = randomStages(g, 9);
	const std::vector<double> in = randomSignal(g, 64);
	const Model m = modelDf2T(st, in, kU);
	double worst = 0.0;
	for (double e : m.e) worst = std::max(worst, e);
	CHECK_MESSAGE(worst < 1e-13, "largest bound " << worst);
}

TEST_CASE("[C3 arm_biquad_cascade_df2T_f64] the impulse response of b = (1, 2, 3), a = (0.5, -0.25) is 1, 2.5, 4, 1.375, -0.3125 exactly") {
	// hand derivation (y[n] = b0 x + b1 x[n-1] + b2 x[n-2] + a1 y[n-1] + a2 y[n-2], DOC sign convention):
	// y0 = 1; y1 = 2 + 0.5*1 = 2.5; y2 = 3 + 0.5*2.5 - 0.25*1 = 4; y3 = 0.5*4 - 0.25*2.5 = 1.375; y4 = 0.5*1.375 - 0.25*4 = -0.3125
	// all values are dyadic with few bits, so every operation is exact
	const std::vector<Coef> st = {{1, 2, 3, 0.5, -0.25}};
	Dut d(st);
	const std::vector<double> in = {1, 0, 0, 0, 0};
	CHECK(d.run(in) == std::vector<double>{1, 2.5, 4, 1.375, -0.3125});
}

TEST_CASE("[C3 arm_biquad_cascade_df2T_f64] the step response of b = (1, 2, 3), a = (0.5, -0.25) is 1, 3.5, 7.5, 8.875, 8.5625 exactly") {
	// hand derivation: y0 = 1; y1 = (1+2) + 0.5*1 = 3.5; y2 = 6 + 0.5*3.5 - 0.25*1 = 7.5; y3 = 6 + 0.5*7.5 - 0.25*3.5 = 8.875;
	// y4 = 6 + 0.5*8.875 - 0.25*7.5 = 8.5625 (the limit is the DC gain 6 / (1 - 0.5 + 0.25) = 8)
	const std::vector<Coef> st = {{1, 2, 3, 0.5, -0.25}};
	Dut d(st);
	const std::vector<double> in = {1, 1, 1, 1, 1};
	CHECK(d.run(in) == std::vector<double>{1, 3.5, 7.5, 8.875, 8.5625});
}

TEST_CASE("[C3 arm_biquad_cascade_df2T_f64] a two stage cascade feeds the output of stage 1 into stage 2: gain 2 followed by a one sample delay gives 2 x[n-1]") {
	const std::vector<Coef> st = {{2, 0, 0, 0, 0}, {0, 1, 0, 0, 0}};
	Dut d(st);
	const std::vector<double> in = {1, -3, 5, 0.5, 7, 0};
	CHECK(d.run(in) == std::vector<double>{0, 2, -6, 10, 1, 14});
}

TEST_CASE("[C3 arm_biquad_cascade_df2T_f64] integer data and coefficients (2 stages, 9 samples) give exactly the integer DEF result") {
	// all products and sums are integers far below 2^53: the double DEF is exact, so the library must be exact
	Rng g(31);
	std::vector<Coef> st;
	std::vector<double> in;
	Model m;
	for (int attempt = 0; attempt < 200; attempt++) {
		st.clear();
		for (int s = 0; s < 2; s++) {
			Coef c;
			c.b0 = g.range(-2, 2); c.b1 = g.range(-2, 2); c.b2 = g.range(-2, 2); c.a1 = g.range(-1, 1); c.a2 = g.range(-1, 1);
			st.push_back(c);
		}
		in.clear();
		for (int i = 0; i < 9; i++) in.push_back(g.range(-3, 3));
		m = modelDf2T(st, in, 0.0);
		if (m.maxSum < 1e6) break;
	}
	REQUIRE(m.maxSum < 1e6);
	Dut d(st);
	CHECK(d.run(in) == m.y);
}

TEST_CASE("[C3 arm_biquad_cascade_df2T_f64] a strided sweep over the exponent range (2^-400 to 2^400) of a one sample filter equals b0 * x exactly") {
	// y[0] = b0 * x + 0 is a single correctly rounded product, so the library must equal the double product exactly.
	// sweep: sign, 8 mantissas, exponent -400..400 in steps of 7; b0 in {1, -0.75, 0.3} (|b0| <= 1: no overflow)
	const double b0s[] = {1.0, -0.75, 0.3};
	const double mant[] = {1.0, 1.0000000000000002, 1.25, 1.4142135623730951, 1.5, 1.7320508075688772, 1.9999999999999998, 1.1};
	size_t bad = 0, total = 0;
	double firstBad = 0;
	for (double b0 : b0s) {
		const std::vector<Coef> st = {{b0, 0.5, 0.25, 0.1, 0.05}};
		for (int e = -400; e <= 400; e += 7)
			for (double mt : mant)
				for (double sg : {1.0, -1.0}) {
					const double x = sg * std::ldexp(mt, e);
					Dut d(st);
					const std::vector<double> y = d.run(&x, 1);
					total++;
					if (!(y[0] == b0 * x)) { if (!bad) firstBad = x; bad++; }
				}
	}
	CHECK_MESSAGE(bad == 0, bad << " of " << total << " differ, first x " << firstBad);
}

TEST_CASE("[C3 arm_biquad_cascade_df2T_f64] a strided sweep over pairs of inputs with different exponents stays within the running error bound") {
	// two samples: the second output carries d1 of the first (absorption when the exponents differ widely)
	const std::vector<Coef> st = {{0.5, 0.25, -0.125, 0.125, -0.0625}};
	size_t bad = 0, total = 0;
	std::string first;
	for (int e0 = -300; e0 <= 300; e0 += 37)
		for (int e1 = -300; e1 <= 300; e1 += 41)
			for (double s0 : {1.0, -1.0})
				for (double s1 : {1.0, -1.0}) {
					const std::vector<double> in = {s0 * std::ldexp(1.3, e0), s1 * std::ldexp(1.7, e1)};
					Dut d(st);
					const std::vector<double> lib = d.run(in);
					const Model m = modelDf2T(st, in, kU);
					const std::string f = boundFailure(lib, m, 2.0);
					total++;
					if (!f.empty()) { if (!bad) first = f; bad++; }
				}
	CHECK_MESSAGE(bad == 0, bad << " of " << total << " outside the bound, first " << first);
}

// ======================================================= C4 =========================================================
TEST_CASE("[C4 arm_biquad_cascade_df2T_init_f64] init stores numStages, pCoeffs and pState") {
	for (int ns = 1; ns <= 9; ns++) {
		Rng g(40 + ns);
		const std::vector<double> co = flatten(randomStages(g, ns));
		dsp::GuardedBuffer<double> state(2 * static_cast<size_t>(ns));
		arm_biquad_cascade_df2T_instance_f64 S;
		std::memset(&S, 0xAB, sizeof S);
		arm_biquad_cascade_df2T_init_f64(&S, static_cast<uint8_t>(ns), co.data(), state.data());
		CHECK(S.numStages == ns);
		CHECK(S.pCoeffs == co.data());
		CHECK(S.pState == state.data());
	}
}

TEST_CASE("[C4 arm_biquad_cascade_df2T_init_f64] init zeroes exactly the 2*numStages state elements (poisoned buffer, guard words intact)") {
	// DOC: "Zeros out the values in the state buffer"; the state array has a total length of 2*numStages values
	for (int ns = 1; ns <= 9; ns++) {
		Rng g(50 + ns);
		const std::vector<double> co = flatten(randomStages(g, ns));
		dsp::GuardedBuffer<double> state(2 * static_cast<size_t>(ns));   // constructor poisons the elements with 0xCD bytes
		arm_biquad_cascade_df2T_instance_f64 S;
		arm_biquad_cascade_df2T_init_f64(&S, static_cast<uint8_t>(ns), co.data(), state.data());
		const std::vector<double> o = state.output();
		bool zero = true;
		for (double v : o) zero = zero && v == 0.0;
		CHECK_MESSAGE(zero, "numStages " << ns);
		CHECK_MESSAGE(state.guardIntact(), "numStages " << ns);
	}
}

TEST_CASE("[C4 arm_biquad_cascade_df2T_init_f64] init does not modify the coefficients") {
	Rng g(60);
	const std::vector<Coef> st = randomStages(g, 4);
	const std::vector<double> co = flatten(st);
	const std::vector<double> copy = co;
	Dut d(st);
	CHECK(co == copy);
	CHECK(d.coefs == copy);
}

TEST_CASE("[C4 arm_biquad_cascade_df2T_f64] initialising again resets the state: the same input gives the same output") {
	Rng g(61);
	const std::vector<Coef> st = randomStages(g, 3);
	const std::vector<double> in = randomSignal(g, 20);
	Dut d(st);
	const std::vector<double> first = d.run(in);
	d.run(in);   // state is now not zero
	arm_biquad_cascade_df2T_init_f64(&d.S, 3, d.coefs.data(), d.sp());
	CHECK(d.run(in) == first);
}

TEST_CASE("[C4 arm_biquad_cascade_df2T_f64] the output without re-init differs from the first block (the state is kept between calls)") {
	// guards the re-init test above: the second call without init must NOT repeat the first output
	Rng g(62);
	const std::vector<Coef> st = randomStages(g, 3);
	const std::vector<double> in = randomSignal(g, 20);
	Dut d(st);
	const std::vector<double> first = d.run(in);
	CHECK(d.run(in) != first);
}

TEST_CASE("[C4 arm_biquad_cascade_df2T_f64] the state layout after one sample of a one stage filter is {d1, d2} = {2.5, 2.75}") {
	// hand derivation (impulse, b = (1, 2, 3), a = (0.5, -0.25)): y0 = 1; d1 = b1*1 + a1*y0 + 0 = 2.5; d2 = b2*1 + a2*y0 = 2.75
	const std::vector<Coef> st = {{1, 2, 3, 0.5, -0.25}};
	Dut d(st);
	const double x = 1.0;
	d.run(&x, 1);
	CHECK(d.stateVals() == std::vector<double>{2.5, 2.75});
}

TEST_CASE("[C4 arm_biquad_cascade_df2T_f64] the state of stage 2 follows the state of stage 1: {d11, d12, d21, d22}") {
	// stage 1 as above (state {2.5, 2.75} after the impulse sample, output 1); stage 2 b = (1, 1, 0), no feedback:
	// y = 1*1 + 0 = 1; d1 = b1 * 1 + a1 y + d2 = 1; d2 = b2 * 1 + a2 y = 0
	const std::vector<Coef> st = {{1, 2, 3, 0.5, -0.25}, {1, 1, 0, 0, 0}};
	Dut d(st);
	const double x = 1.0;
	d.run(&x, 1);
	CHECK(d.stateVals() == std::vector<double>{2.5, 2.75, 1.0, 0.0});
}

TEST_CASE("[C4 arm_biquad_cascade_df2T_f64] the same samples split into blocks of 1, 2, 3, 4, 5, 7, 16 and 17 agree with the double DEF within twice the running error bound") {
	Rng g(63);
	const std::vector<Coef> st = randomStages(g, 4);
	const std::vector<double> in = randomSignal(g, 100);
	// different partitions run through the unrolled (16) and the tail code: compared with the DEF bound, not bitwise
	// (FMA contraction or excess precision could differ per code path on some targets)
	const Model m = modelDf2T(st, in, kU);
	const size_t splits[] = {1, 2, 3, 4, 5, 7, 16, 17};
	for (size_t sp : splits) {
		Dut d(st, true);
		std::vector<double> got;
		for (size_t pos = 0; pos < in.size(); pos += sp) {
			const size_t n = std::min(sp, in.size() - pos);
			const std::vector<double> part = d.run(in.data() + pos, n);
			got.insert(got.end(), part.begin(), part.end());
		}
		const std::string f = boundFailure(got, m, 2.0);
		CHECK_MESSAGE(f.empty(), "block size " << sp << " " << f);
	}
}

TEST_CASE("[C4 arm_biquad_cascade_df2T_f64] mixed block sizes (3, 1, 16, 5, 2, 7, 4) agree with the double DEF within twice the running error bound") {
	Rng g(64);
	const std::vector<Coef> st = randomStages(g, 2);
	const std::vector<double> in = randomSignal(g, 38);
	const Model m = modelDf2T(st, in, kU);
	const size_t sizes[] = {3, 1, 16, 5, 2, 7, 4};
	Dut d(st, true);
	std::vector<double> got;
	size_t pos = 0;
	for (size_t n : sizes) {
		const std::vector<double> part = d.run(in.data() + pos, n);
		got.insert(got.end(), part.begin(), part.end());
		pos += n;
	}
	REQUIRE(pos == in.size());
	const std::string f = boundFailure(got, m, 2.0);
	CHECK_MESSAGE(f.empty(), f);
}

TEST_CASE("[C4 arm_biquad_cascade_df2T_f64] two instances sharing one coefficient array are independent when their calls are interleaved") {
	Rng g(65);
	const std::vector<Coef> st = randomStages(g, 3);
	const std::vector<double> inA = randomSignal(g, 24), inB = randomSignal(g, 24);
	Dut soloA(st), soloB(st);
	std::vector<double> refA, refB;   // same partition (blocks of 6) as the interleaved run: bit-identical results expected
	for (size_t pos = 0; pos < 24; pos += 6) {
		const std::vector<double> pa = soloA.run(inA.data() + pos, 6), pb = soloB.run(inB.data() + pos, 6);
		refA.insert(refA.end(), pa.begin(), pa.end());
		refB.insert(refB.end(), pb.begin(), pb.end());
	}
	Dut a(st);
	dsp::GuardedBuffer<double> stateB(6);
	arm_biquad_cascade_df2T_instance_f64 SB;
	arm_biquad_cascade_df2T_init_f64(&SB, 3, a.coefs.data(), stateB.data());   // same coefficient array as a
	std::vector<double> gotA, gotB;
	for (size_t pos = 0; pos < 24; pos += 6) {
		const std::vector<double> pa = a.run(inA.data() + pos, 6);
		gotA.insert(gotA.end(), pa.begin(), pa.end());
		dsp::GuardedBuffer<double> ob(6);
		arm_biquad_cascade_df2T_f64(&SB, inB.data() + pos, ob.data(), 6);
		const std::vector<double> pb = ob.output();
		gotB.insert(gotB.end(), pb.begin(), pb.end());
	}
	CHECK(gotA == refA);
	CHECK(gotB == refB);
}

TEST_CASE("[C4 arm_biquad_cascade_df2T_f64] processing does not modify the coefficients or the input") {
	Rng g(66);
	const std::vector<Coef> st = randomStages(g, 5);
	const std::vector<double> in = randomSignal(g, 33);
	const std::vector<double> inCopy = in;
	Dut d(st);
	const std::vector<double> coCopy = d.coefs;
	d.run(in);
	CHECK(d.coefs == coCopy);
	CHECK(in == inCopy);
}

TEST_CASE("[C4 arm_biquad_cascade_df2T_f64] a stage with b = (1, 0, 0) and a = (0, 0) is the identity for 1 to 9 stages") {
	Rng g(67);
	for (int ns = 1; ns <= 9; ns++) {
		const std::vector<Coef> st(static_cast<size_t>(ns), Coef{1, 0, 0, 0, 0});
		const std::vector<double> in = randomSignal(g, 19);
		Dut d(st);
		CHECK_MESSAGE(d.run(in) == in, "stages " << ns);
	}
}

// ======================================================= C5 =========================================================
TEST_CASE("[C5 arm_biquad_cascade_df2T_f64] the f64 output agrees with arm_biquad_cascade_df2T_f32 of the same input within both running error bounds") {
	// f32 side: u32 = 2^-24 bound (same derivation); the f64 library side is covered by the 1e-15 level bound.
	// inputs and coefficients are rounded to float first so both functions see identical values
	Rng g(70);
	std::string fail;
	for (int ns = 1; ns <= 5 && fail.empty(); ns++)
		for (size_t bs : {1, 4, 5, 9, 17, 40}) {
			std::vector<Coef> st = randomStages(g, ns);
			for (Coef &c : st) {
				c.b0 = static_cast<float>(c.b0); c.b1 = static_cast<float>(c.b1); c.b2 = static_cast<float>(c.b2);
				c.a1 = static_cast<float>(c.a1); c.a2 = static_cast<float>(c.a2);
			}
			std::vector<double> in = randomSignal(g, bs);
			for (double &v : in) v = static_cast<float>(v);
			Dut d(st);
			const std::vector<double> lib64 = d.run(in);
			const std::vector<double> c64 = flatten(st);
			std::vector<float> c32(c64.begin(), c64.end()), in32(in.begin(), in.end()), out32(bs), state32(2 * static_cast<size_t>(ns));
			arm_biquad_cascade_df2T_instance_f32 S32;
			arm_biquad_cascade_df2T_init_f32(&S32, static_cast<uint8_t>(ns), c32.data(), state32.data());
			arm_biquad_cascade_df2T_f32(&S32, in32.data(), out32.data(), static_cast<uint32_t>(bs));
			const Model m32 = modelDf2T(st, in, kU32);
			const Model m64 = modelDf2T(st, in, kU);
			for (size_t i = 0; i < bs; i++) {
				const double diff = std::fabs(static_cast<double>(out32[i]) - lib64[i]);
				const double bound = m32.e[i] + 2.0 * m64.e[i];
				if (!(diff <= bound)) {
					fail = "stages " + std::to_string(ns) + " block " + std::to_string(bs) + " sample " + std::to_string(i) + ": f32 " + std::to_string(out32[i]) + " f64 " + std::to_string(lib64[i]);
					break;
				}
			}
			if (!fail.empty()) break;
		}
	CHECK_MESSAGE(fail.empty(), fail);
}

}  // TEST_SUITE
