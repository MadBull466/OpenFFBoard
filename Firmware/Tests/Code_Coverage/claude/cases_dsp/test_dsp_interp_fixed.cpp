/*
 * Interpolation q31 / q15 / q7 (work order IQ, requirements C1-C6 of Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md).
 *
 * Units covered: arm_linear_interp_q31 / _q15 / _q7 (Source/InterpolationFunctions/arm_linear_interp_q*.c) and
 * arm_bilinear_interp_q31 / _q15 / _q7 (arm_bilinear_interp_q*.c).
 *
 * ARM ids covered (Testing/desc.txt:887-951, suites "Interpolation Tests Q31 / Q15 / Q7"): test_linear_interp_q31,
 * test_bilinear_interp_q31, test_linear_interp_q15, test_bilinear_interp_q15, test_linear_interp_q7,
 * test_bilinear_interp_q7. None is disabled or commented out, all six are run. The pattern files InputX3..5, InputY3..5,
 * OutputX3..5 and Reference3..5 of the folders are not referenced by desc.txt for these suites and are not used.
 * Pattern files (Testing/Patterns/DSP/Interpolation/InterpolationQ31 / Q15 / Q7/): Input1_q31.txt (42 x values in 12.20),
 * YVals1_<t>.txt (41 table values), Reference1_<t>.txt (42); Input2_q31.txt (150 (X, Y) pairs), Config2_s16.txt
 * (numCols = 7, numRows = 8), YVals2_<t>.txt (56 values, row major), Reference2_<t>.txt (150).
 * ARM tolerances (InterpolationTestsQ31.cpp:5,13; Q15.cpp:5,13; Q7.cpp:5,13), used unchanged (L4): SNR threshold
 *   100 dB (q31), 70 dB (q15), 20 dB (q7); per sample ABS_ERROR_Q31 2000 LSB, ABS_ERROR_Q15 2 LSB, ABS_ERROR_Q7 2 LSB.
 *   ARM's setUp loads the whole files (MAX_NB_SAMPLES); the function is called once per sample (nValues = 41; bilinear
 *   numRows = Config[1], numCols = Config[0]); ASSERT_EMPTY_TAIL.
 *
 * The GuardedBuffer guard in C1 and C2 only mirrors ARM's ASSERT_EMPTY_TAIL: the library returns by value, so the guard
 *   checks the test's own buffer, not the library.
 * C2: all six functions return their result by value, there is no output pointer: the "output" is a GuardedBuffer filled
 *   one sample per call for block sizes 0, 1, 3, 4, 5, 9 (every sample against the DEF, guard intact, size 0 writes
 *   nothing). The tables are std::vectors of exactly the table size, so a read past the table is caught by ASan.
 *
 * C3 (DEF, evaluated in double in the test): x is 12.20: index i = x >> 20, fraction f = (x & 0xFFFFF) / 2^20.
 *   linear: y = t[i] + f * (t[i+1] - t[i]); bilinear: X, Y give column c and row r (table element (c, r) is
 *   t[c + r * numCols], header text of arm_bilinear_interp_f32.c), result is the four-corner formula of that header text.
 *   Error bounds, derived from the code:
 *   linear q15 / q7: y = (y0 * (M - f) + y1 * f) >> 20 with M = 0xFFFFF = 2^20 - 1 (not 2^20) = floor(DEF - y0 / 2^20)
 *     exactly, so (result - DEF) is in (-1 - y0 / 2^20, -y0 / 2^20]. This is per sample and tighter than ARM's 2 LSB.
 *     At a knot the positive value y0 is returned as y0 - 1 (M = 0xFFFFF): NEEDS REVIEW note, lead decision: not a
 *     bug, within the bound and ARM's 2 LSB, kept as is.
 *   linear q31: fract = f << 11 (31 bits), y = floor(y0 * (M31 - fract) / 2^32) + floor(y1 * fract / 2^32) with
 *     M31 = 0x7FFFFFFF, result 2 * y. y0 * (M31 - fract) + y1 * fract = 2^31 * DEF - y0, so the exact value is
 *     DEF - y0 / 2^31, the two floors lose < 1 each (= < 2 in y, < 4 in the result): (result - DEF) is in
 *     (-4 - y0 / 2^31, -y0 / 2^31].
 *   bilinear q7: all products are exact integers, only the last >> 40 truncates; the weights sum to M^2 = 2^40 - 2^21 + 1
 *     instead of 2^40, which changes the value by <= 2 * 2^-20 * 128 = 2.4e-4 LSB: (result - DEF) in [-1.001, 0.001].
 *   bilinear q15: the intermediate x * (M - xf) >> 4 truncates < 1 unit of 2^-16 LSB per term (4 terms: < 6.2e-5 LSB),
 *     weight deficit <= 2 * 2^-20 * 32768 = 0.0625 LSB, final >> 36 truncates < 1 LSB: [-1.07, 0.07].
 *   bilinear q31: each term is out = floor(x * a / 2) (a = (M31 - xfract) / 2^31, error < 1) then floor(out * b / 2) (error
 *     < 1, the first error is scaled by b / 2 <= 0.5): per term in acc units (-1.5, 0], four terms (-6, 0], the result is
 *     acc << 2: (-24, 0]. Weight deficit: sum w_i * v_i is DEF minus 2^-31 * (2 v00 ...) so |.| <= 2 * 2^-31 * 2^31 = 2 LSB.
 *     Total [-26.5, 2.5].
 *   The sweeps cover the input range at least once strided (lesson 2): linear over the whole int32 range with a stride of
 *   1000003 plus a dense stride (509) around the table, bilinear a dense stride (98317) around the matrix and the extremes.
 *
 * C4 (DOC): linear (arm_linear_interp_f32.c group text, valid for all linear types): "if x is outside of the table
 *   boundary, Linear interpolation returns first value of the table if x is below input range and returns last value of
 *   table if x is above range." bilinear (arm_bilinear_interp_f32.c group text and the comment in arm_bilinear_interp_q*.c):
 *   "if (x,y) are outside of the table boundary, Bilinear interpolation returns zero output."
 *   The exact last column / last row line (X = (numCols - 1) << 20 returns 0 although it is on the table) is a
 *   characterization test (NEEDS REVIEW); the sweeps skip it. Not tested (NEEDS REVIEW): nValues = 0,
 *   matrices with numCols or numRows below 2, tables above 2048 entries.
 *
 * C5: q result against the f32 function of the same input (arm_linear_interp_f32 with x1 = 0, xSpacing = 1;
 *   arm_bilinear_interp_f32). The f32 table holds the integer LSB values (exact in float: the q31 table values are
 *   multiples of 128 below 2^31, i.e. at most 24 significant bits), x / 2^20 is exact in float because all x stay below
 *   2^24 (tables of 9 entries, matrix 4 x 5). So the f32 result is in LSB of the q type. f32 error: 8 ulp (linear: y1 - y0,
 *   product, sum, the fractional part, each at most one ulp of max(|y0|, |y1|)) and 42 ulp (bilinear, counted as the
 *   f16 file does: 4 + 9 + 12 + 17 rounding errors of the differences, the weights, the products and the sums, each
 *   at most 2^-24 relative to max|v|, with the error of the difference terms multiplied by weights <= 1) of 2^-24 * max|v|. The q value must be in [lo - fErr, hi + fErr] of the f32 value. Bilinear compares only
 *   X, Y >= 0: for -1 < X < 0 the f32 function truncates (int)X to 0 and interpolates, the q functions return 0 (see report).
 *
 * Host vs target (C6): the host runs the portable C path. No ARM_MATH_DSP intrinsics are used in these files. q63
 * arithmetic is used for the products (long long on both). Right shifts of negative values (>> 20, >> 32, >> 36, >> 40)
 * are implementation-defined in C (arithmetic on GCC, clang and ARM GCC). LEFT shifts of negative values are undefined
 * (B-DIQ-1: arm_linear_interp_q31.c:97 'y << 1U', B-DIQ-2: arm_bilinear_interp_q31.c:114 'acc << 2'): every q31 test
 * that can produce a negative result is KNOWN_ISSUE_UB_HIDDEN; each has a positive-only twin that runs under UBSan.
 * int is 32 bit on host and target, (q31_t) casts of 0xFFF00000 are done by the library.
 */
#include "doctest.h"

#include "DspPatterns.h"
#include "TestHelpers.h"
#include "arm_math.h"

#include <climits>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#define B_DIQ_1                                                                                                          \
	"B-DIQ-1: arm_linear_interp_q31.c:97 'return (y << 1U)' shifts a negative q31 value left for every negative result: " \
	"undefined behaviour in C (UBSan: 'left shift of negative value'). The intended result occurs on the host; skipped "  \
	"in sanitizer builds"
#define B_DIQ_2                                                                                                          \
	"B-DIQ-2: arm_bilinear_interp_q31.c:114 'return ((q31_t)(acc << 2))' shifts a negative value left for every negative " \
	"result: undefined behaviour in C (UBSan: 'left shift of negative value'). The intended result occurs on the host; " \
	"skipped in sanitizer builds"

namespace {

constexpr int kFrac = 20;
constexpr double kOne = 1048576.0;  // 2^20

// ------------------------------------------------------------------------------------------------------------------
// Type traits: function pointers, ARM tolerances, derived DEF bounds, test tables
// ------------------------------------------------------------------------------------------------------------------

struct TQ31 {
	typedef q31_t T;
	static std::string name() { return "q31"; }
	static std::string dir() { return "Interpolation/InterpolationQ31/"; }
	static std::vector<T> load(const std::string &f) { return dsp::loadPattern(dir() + f).q31(); }
	static constexpr double armSnr = 100.0;
	static constexpr int64_t armLsb = 2000;
	static T linear(const T *p, q31_t x, uint32_t n) { return arm_linear_interp_q31(p, x, n); }
	static T bilinear(const T *d, uint16_t rows, uint16_t cols, q31_t X, q31_t Y) {
		arm_bilinear_interp_instance_q31 S;
		S.numRows = rows;
		S.numCols = cols;
		S.pData = d;
		return arm_bilinear_interp_q31(&S, X, Y);
	}
	static void linBounds(double y0, double &lo, double &hi) {
		lo = -4.0 - y0 / 2147483648.0;
		hi = -y0 / 2147483648.0;
	}
	static void biBounds(double &lo, double &hi) {
		lo = -26.5;
		hi = 2.5;
	}
	static std::vector<T> mixed() { return {INT32_MIN, -1500000000, 0, 7, 1073741824, INT32_MAX, -1, 123456789, -987654321}; }
	static std::vector<T> positive() { return {0, 1, 1000, 500000000, INT32_MAX, 1073741824, 5, 2000000000, INT32_MAX}; }
	// multiples of 128, so exact in float (at most 24 significant bits); positive: no negative result, no UB
	static std::vector<T> smooth() { return {0, 131072, 262144000, 384000000, 600000000, 800000000, 1000000000, 1500000000, 1000000000}; }
};

struct TQ15 {
	typedef q15_t T;
	static std::string name() { return "q15"; }
	static std::string dir() { return "Interpolation/InterpolationQ15/"; }
	static std::vector<T> load(const std::string &f) { return dsp::loadPattern(dir() + f).q15(); }
	static constexpr double armSnr = 70.0;
	static constexpr int64_t armLsb = 2;
	static T linear(const T *p, q31_t x, uint32_t n) { return arm_linear_interp_q15(p, x, n); }
	static T bilinear(const T *d, uint16_t rows, uint16_t cols, q31_t X, q31_t Y) {
		arm_bilinear_interp_instance_q15 S;
		S.numRows = rows;
		S.numCols = cols;
		S.pData = d;
		return arm_bilinear_interp_q15(&S, X, Y);
	}
	static void linBounds(double y0, double &lo, double &hi) {
		lo = -1.0 - y0 / kOne;
		hi = -y0 / kOne;
	}
	static void biBounds(double &lo, double &hi) {
		lo = -1.07;
		hi = 0.07;
	}
	static std::vector<T> mixed() { return {-32768, -20000, 0, 7, 16384, 32767, -1, 12345, -9876}; }
	static std::vector<T> positive() { return {0, 1, 1000, 5000, 32767, 16384, 5, 30000, 32767}; }
	static std::vector<T> smooth() { return mixed(); }
};

struct TQ7 {
	typedef q7_t T;
	static std::string name() { return "q7"; }
	static std::string dir() { return "Interpolation/InterpolationQ7/"; }
	static std::vector<T> load(const std::string &f) { return dsp::loadPattern(dir() + f).q7(); }
	static constexpr double armSnr = 20.0;
	static constexpr int64_t armLsb = 2;
	static T linear(const T *p, q31_t x, uint32_t n) { return arm_linear_interp_q7(p, x, n); }
	static T bilinear(const T *d, uint16_t rows, uint16_t cols, q31_t X, q31_t Y) {
		arm_bilinear_interp_instance_q7 S;
		S.numRows = rows;
		S.numCols = cols;
		S.pData = d;
		return arm_bilinear_interp_q7(&S, X, Y);
	}
	static void linBounds(double y0, double &lo, double &hi) {
		lo = -1.0 - y0 / kOne;
		hi = -y0 / kOne;
	}
	static void biBounds(double &lo, double &hi) {
		lo = -1.001;
		hi = 0.001;
	}
	static std::vector<T> mixed() { return {-128, -100, 0, 7, 64, 127, -1, 45, -98}; }
	static std::vector<T> positive() { return {0, 1, 10, 50, 127, 64, 5, 100, 127}; }
	static std::vector<T> smooth() { return mixed(); }
};

// Matrix of rows x cols values taken from a 9 value list (the stride 5 is coprime to 9, so every list value occurs)
template <class T>
std::vector<T> matrixFrom(const std::vector<T> &list, size_t rows, size_t cols) {
	std::vector<T> m;
	for (size_t k = 0; k < rows * cols; k++) m.push_back(list[(k * 5 + 2) % list.size()]);
	return m;
}

// ------------------------------------------------------------------------------------------------------------------
// DEF
// ------------------------------------------------------------------------------------------------------------------

struct Exp {
	bool skip = false;   // ambiguous point (bilinear boundary line), not asserted
	bool exact = false;  // result must equal def exactly
	double def = 0, lo = 0, hi = 0;
};

template <class Tr>
Exp linearExp(const std::vector<typename Tr::T> &t, int64_t x) {
	Exp e;
	const int64_t n = (int64_t)t.size();
	if (x < 0) {  // DOC: below the range returns the first value
		e.exact = true;
		e.def = (double)t[0];
		return e;
	}
	const int64_t idx = x >> kFrac;
	if (idx >= n - 1) {  // DOC: above the range returns the last value
		e.exact = true;
		e.def = (double)t[(size_t)n - 1];
		return e;
	}
	const double frac = (double)(x & 0xFFFFF) / kOne;
	e.def = (double)t[(size_t)idx] + frac * ((double)t[(size_t)idx + 1] - (double)t[(size_t)idx]);
	Tr::linBounds((double)t[(size_t)idx], e.lo, e.hi);
	return e;
}

// X selects the column, Y the row (table element (c, r) is t[c + r * cols])
template <class Tr>
Exp bilinearExp(const std::vector<typename Tr::T> &t, int64_t rows, int64_t cols, int64_t X, int64_t Y) {
	Exp e;
	const int64_t c = (int64_t)std::floor((double)X / kOne), r = (int64_t)std::floor((double)Y / kOne);
	// the lines X = (cols - 1) << 20 and Y = (rows - 1) << 20 are on the table, but the library returns 0 there: DOC
	// ambiguous, not asserted (lead decision)
	if ((X == ((cols - 1) << kFrac) && Y >= 0 && Y <= ((rows - 1) << kFrac)) ||
	    (Y == ((rows - 1) << kFrac) && X >= 0 && X <= ((cols - 1) << kFrac))) {
		e.skip = true;
		return e;
	}
	if (c < 0 || c > cols - 2 || r < 0 || r > rows - 2) {  // DOC: outside returns zero
		e.exact = true;
		e.def = 0.0;
		return e;
	}
	const double xf = (double)(X - (c << kFrac)) / kOne, yf = (double)(Y - (r << kFrac)) / kOne;
	const double v00 = (double)t[(size_t)(c + r * cols)], v01 = (double)t[(size_t)(c + 1 + r * cols)];
	const double v10 = (double)t[(size_t)(c + (r + 1) * cols)], v11 = (double)t[(size_t)(c + 1 + (r + 1) * cols)];
	e.def = v00 * (1 - xf) * (1 - yf) + v01 * xf * (1 - yf) + v10 * (1 - xf) * yf + v11 * xf * yf;
	Tr::biBounds(e.lo, e.hi);
	return e;
}

// "" if got is as expected, else a message
std::string verdict(const Exp &e, double got, double extra, const char *what, int64_t x, int64_t y) {
	if (e.skip) return "";
	const double lo = e.exact ? 0.0 : e.lo - extra - 1e-6, hi = e.exact ? 0.0 : e.hi + extra + 1e-6;
	const double d = got - e.def;
	if (d >= lo && d <= hi) return "";
	std::ostringstream s;
	s << what << " x=" << x << " y=" << y << " got=" << got << " DEF=" << e.def << " diff=" << d << " allowed [" << lo << ", " << hi << "]";
	return s.str();
}

template <class Tr>
std::string linearCheck(const std::vector<typename Tr::T> &t, int64_t x) {
	const Exp e = linearExp<Tr>(t, x);
	return verdict(e, (double)Tr::linear(t.data(), (q31_t)x, (uint32_t)t.size()), 0.0, "linear", x, 0);
}

template <class Tr>
std::string bilinearCheck(const std::vector<typename Tr::T> &t, int rows, int cols, int64_t X, int64_t Y) {
	const Exp e = bilinearExp<Tr>(t, rows, cols, X, Y);
	return verdict(e, (double)Tr::bilinear(t.data(), (uint16_t)rows, (uint16_t)cols, (q31_t)X, (q31_t)Y), 0.0, "bilinear", X, Y);
}

// Sweeps; return the first failure message
template <class Tr>
std::string sweepLinear(const std::vector<typename Tr::T> &t) {
	std::string first;
	auto one = [&](int64_t x) {
		if (!first.empty()) return;
		first = linearCheck<Tr>(t, x);
	};
	const int64_t n = (int64_t)t.size();
	for (int64_t x = -(2 << kFrac); x <= ((n + 2) << kFrac); x += 509) one(x);                // dense around the table
	for (int64_t k = -1; k <= n + 1; k++)                                                      // knots and their neighbours
		for (int64_t d = -1; d <= 1; d++) one((k << kFrac) + d);
	for (int64_t x = INT32_MIN; x < INT32_MAX - 1000003; x += 1000003) one(x);                 // the whole int32 range
	one(INT32_MIN);
	one(INT32_MAX);
	return first;
}

template <class Tr>
std::string sweepBilinear(const std::vector<typename Tr::T> &t, int rows, int cols) {
	std::string first;
	auto one = [&](int64_t X, int64_t Y) {
		if (!first.empty()) return;
		first = bilinearCheck<Tr>(t, rows, cols, X, Y);
	};
	for (int64_t X = -(1 << kFrac); X <= ((int64_t)(cols + 1) << kFrac); X += 98317)
		for (int64_t Y = -(1 << kFrac); Y <= ((int64_t)(rows + 1) << kFrac); Y += 98317) one(X, Y);
	for (int64_t kx = -1; kx <= cols + 1; kx++)
		for (int64_t ky = -1; ky <= rows + 1; ky++)
			for (int64_t dx = -1; dx <= 1; dx++)
				for (int64_t dy = -1; dy <= 1; dy++) one((kx << kFrac) + dx, (ky << kFrac) + dy);
	const int64_t ext[] = {INT32_MIN, -1, 0, (int64_t)(1 << kFrac) + 12345, INT32_MAX};
	for (int64_t X : ext)
		for (int64_t Y : ext) one(X, Y);
	return first;
}

// ------------------------------------------------------------------------------------------------------------------
// Test bodies
// ------------------------------------------------------------------------------------------------------------------

template <class Tr>
void armLinear() {
	std::vector<int32_t> in = dsp::loadPattern(Tr::dir() + "Input1_q31.txt").q31();
	std::vector<typename Tr::T> y = Tr::load("YVals1_" + Tr::name() + ".txt");
	std::vector<typename Tr::T> ref = Tr::load("Reference1_" + Tr::name() + ".txt");
	REQUIRE(in.size() == ref.size());
	dsp::GuardedBuffer<typename Tr::T> out(ref.size());
	for (size_t i = 0; i < in.size(); i++) out.data()[i] = Tr::linear(y.data(), in[i], (uint32_t)y.size());
	std::vector<typename Tr::T> o = out.output();
	CHECK(dsp::snr(ref, o) >= Tr::armSnr);
	auto c = dsp::withinLsb(ref, o, Tr::armLsb);
	CHECK_MESSAGE(c, c.message);
	CHECK(out.guardIntact());
}

template <class Tr>
void armBilinear() {
	std::vector<int32_t> in = dsp::loadPattern(Tr::dir() + "Input2_q31.txt").q31();
	std::vector<int16_t> cfg = dsp::loadPattern(Tr::dir() + "Config2_s16.txt").q15();
	std::vector<typename Tr::T> y = Tr::load("YVals2_" + Tr::name() + ".txt");
	std::vector<typename Tr::T> ref = Tr::load("Reference2_" + Tr::name() + ".txt");
	REQUIRE(cfg.size() >= 2);
	REQUIRE(in.size() == 2 * ref.size());
	const uint16_t numRows = (uint16_t)cfg[1], numCols = (uint16_t)cfg[0];  // InterpolationTests<T>.cpp: SBI.numRows = pConfig[1]
	REQUIRE(y.size() == (size_t)numRows * numCols);
	dsp::GuardedBuffer<typename Tr::T> out(ref.size());
	for (size_t i = 0; i < ref.size(); i++) out.data()[i] = Tr::bilinear(y.data(), numRows, numCols, in[2 * i], in[2 * i + 1]);
	std::vector<typename Tr::T> o = out.output();
	CHECK(dsp::snr(ref, o) >= Tr::armSnr);
	auto c = dsp::withinLsb(ref, o, Tr::armLsb);
	CHECK_MESSAGE(c, c.message);
	CHECK(out.guardIntact());
}

template <class Tr>
void guardLinear(const std::vector<typename Tr::T> &t) {
	const int64_t xs[9] = {0, (1 << kFrac) + 5, (3 << kFrac) + 777777, 5 << kFrac, -7, 20 << kFrac, 1, (7 << kFrac) + 1, (8 << kFrac) - 1};
	const uint32_t sizes[] = {0, 1, 3, 4, 5, 9};
	for (uint32_t n : sizes) {
		CAPTURE(n);
		dsp::GuardedBuffer<typename Tr::T> out(n);
		for (uint32_t i = 0; i < n; i++) out.data()[i] = Tr::linear(t.data(), (q31_t)xs[i], (uint32_t)t.size());
		std::vector<typename Tr::T> o = out.output();
		REQUIRE(o.size() == n);
		for (uint32_t i = 0; i < n; i++) {
			const std::string m = verdict(linearExp<Tr>(t, xs[i]), (double)o[i], 0.0, "linear", xs[i], 0);
			CHECK_MESSAGE(m.empty(), m);
		}
		CHECK(out.guardIntact());
	}
}

template <class Tr>
void guardBilinear(const std::vector<typename Tr::T> &t) {
	const int rows = 4, cols = 5;
	const int64_t xs[9] = {0, (1 << kFrac) + 5, (3 << kFrac) + 777777, 2 << kFrac, -7, 20 << kFrac, 1, (3 << kFrac) + 1, (4 << kFrac) - 1};
	const int64_t ys[9] = {0, 5, (2 << kFrac) + 31, 1 << kFrac, 3, 1, -(1 << kFrac), (2 << kFrac) + 9, (3 << kFrac) - 1};
	const uint32_t sizes[] = {0, 1, 3, 4, 5, 9};
	for (uint32_t n : sizes) {
		CAPTURE(n);
		dsp::GuardedBuffer<typename Tr::T> out(n);
		for (uint32_t i = 0; i < n; i++) out.data()[i] = Tr::bilinear(t.data(), rows, cols, (q31_t)xs[i], (q31_t)ys[i]);
		std::vector<typename Tr::T> o = out.output();
		REQUIRE(o.size() == n);
		for (uint32_t i = 0; i < n; i++) {
			const std::string m = verdict(bilinearExp<Tr>(t, rows, cols, xs[i], ys[i]), (double)o[i], 0.0, "bilinear", xs[i], ys[i]);
			CHECK_MESSAGE(m.empty(), m);
		}
		CHECK(out.guardIntact());
	}
}

template <class Tr>
void linearSweepTest(const std::vector<typename Tr::T> &t) {
	const std::string m = sweepLinear<Tr>(t);
	CHECK_MESSAGE(m.empty(), m);
}

template <class Tr>
void bilinearSweepTest(const std::vector<typename Tr::T> &t) {
	const std::string m = sweepBilinear<Tr>(t, 4, 5);
	CHECK_MESSAGE(m.empty(), m);
}

// DOC: below the input range the first value of the table is returned (exactly, whatever the fraction bits)
template <class Tr>
void linearBelowRange() {
	const std::vector<typename Tr::T> t = Tr::mixed();
	const int64_t xs[] = {-1, -(1 << kFrac), -(1 << kFrac) - 1, -12345678, INT32_MIN};
	for (int64_t x : xs) {
		CAPTURE(x);
		CHECK(Tr::linear(t.data(), (q31_t)x, (uint32_t)t.size()) == t[0]);
	}
}

// DOC: above the input range the last value of the table is returned; the last knot itself is "not below" the range
template <class Tr>
void linearAboveRange() {
	const std::vector<typename Tr::T> t = Tr::mixed();
	const int64_t n = (int64_t)t.size();
	const int64_t xs[] = {(n - 1) << kFrac, ((n - 1) << kFrac) + 1, ((n - 1) << kFrac) + 0xFFFFF, n << kFrac, 2047 << kFrac, INT32_MAX};
	for (int64_t x : xs) {
		CAPTURE(x);
		CHECK(Tr::linear(t.data(), (q31_t)x, (uint32_t)t.size()) == t[(size_t)n - 1]);
	}
}

// first interval [0, 1) and last interval [n-2, n-1) of a 9 point table (positive values: no negative q31 result)
template <class Tr>
void linearEndIntervals() {
	const std::vector<typename Tr::T> t = Tr::positive();
	const int64_t n = (int64_t)t.size();
	const int64_t xs[] = {0,
	                      1,
	                      1 << (kFrac - 1),
	                      (1 << kFrac) - 1,
	                      ((n - 2) << kFrac),
	                      ((n - 2) << kFrac) + 1,
	                      ((n - 2) << kFrac) + (1 << (kFrac - 1)),
	                      ((n - 1) << kFrac) - 1};
	for (int64_t x : xs) {
		const std::string m = linearCheck<Tr>(t, x);
		CHECK_MESSAGE(m.empty(), m);
	}
}

// the smallest table that interpolates (2 entries)
template <class Tr>
void linearTwoEntries() {
	const std::vector<typename Tr::T> full = Tr::positive();
	const std::vector<typename Tr::T> t = {full[1], full[4]};
	const int64_t xs[] = {0, 1, 1 << (kFrac - 1), (1 << kFrac) - 1, 1 << kFrac, (1 << kFrac) + 1, INT32_MAX, -1};
	for (int64_t x : xs) {
		const std::string m = linearCheck<Tr>(t, x);
		CHECK_MESSAGE(m.empty(), m);
	}
}

// a single entry: first == last, DOC gives that value for x below and above
template <class Tr>
void linearOneEntry() {
	const std::vector<typename Tr::T> t = {Tr::mixed()[3]};
	const int64_t xs[] = {INT32_MIN, -1, 0, 1, 1 << kFrac, INT32_MAX};
	for (int64_t x : xs) {
		CAPTURE(x);
		CHECK(Tr::linear(t.data(), (q31_t)x, 1) == t[0]);
	}
}

// DOC: outside the table the result is 0 (matrix with non zero values, 4 rows x 5 columns)
template <class Tr>
void bilinearOutside() {
	const int rows = 4, cols = 5;
	const std::vector<typename Tr::T> t = matrixFrom(Tr::positive(), rows, cols);
	const int64_t inX = 1 << (kFrac - 1) , inY = (1 << kFrac) + 77;  // inside for both
	const int64_t outs[] = {-1, -(1 << kFrac), INT32_MIN, INT32_MAX, 100 << kFrac};
	for (int64_t o : outs) {
		CAPTURE(o);
		CHECK(Tr::bilinear(t.data(), rows, cols, (q31_t)o, (q31_t)inY) == 0);  // X outside
		CHECK(Tr::bilinear(t.data(), rows, cols, (q31_t)inX, (q31_t)o) == 0);  // Y outside
		CHECK(Tr::bilinear(t.data(), rows, cols, (q31_t)o, (q31_t)o) == 0);
	}
	// just beyond the last column / row line (the line itself is a lead decision) and the first row beyond it
	const int64_t justBeyondX = ((int64_t)(cols - 1) << kFrac) + 1, justBeyondY = ((int64_t)(rows - 1) << kFrac) + 1;
	CHECK(Tr::bilinear(t.data(), rows, cols, (q31_t)justBeyondX, (q31_t)inY) == 0);
	CHECK(Tr::bilinear(t.data(), rows, cols, (q31_t)inX, (q31_t)justBeyondY) == 0);
	CHECK(Tr::bilinear(t.data(), rows, cols, (q31_t)(cols << kFrac), (q31_t)inY) == 0);
	CHECK(Tr::bilinear(t.data(), rows, cols, (q31_t)inX, (q31_t)(rows << kFrac)) == 0);
}

// The minimum matrix 2 x 2: one cell, corners and the middle follow the DEF
template <class Tr>
void bilinearTwoByTwo() {
	const std::vector<typename Tr::T> list = Tr::positive();
	const std::vector<typename Tr::T> t = {list[1], list[4], list[5], list[3]};
	const int64_t last = (1 << kFrac) - 1;
	const int64_t pts[][2] = {{0, 0}, {last, 0}, {0, last}, {last, last}, {1 << (kFrac - 1), 1 << (kFrac - 1)}, {1, 1}, {123456, 987654}};
	for (const auto &p : pts) {
		const std::string m = bilinearCheck<Tr>(t, 2, 2, p[0], p[1]);
		CHECK_MESSAGE(m.empty(), m);
	}
}

// first cell (0, 0) and last cell (cols-2, rows-2) of a 4 x 5 matrix, orientation X = column, Y = row
template <class Tr>
void bilinearEndCells() {
	const int rows = 4, cols = 5;
	const std::vector<typename Tr::T> t = matrixFrom(Tr::positive(), rows, cols);
	const int64_t lx = (int64_t)(cols - 2) << kFrac, ly = (int64_t)(rows - 2) << kFrac;
	const int64_t pts[][2] = {{0, 0},
	                          {1, 1},
	                          {1 << (kFrac - 1), 1 << (kFrac - 1)},
	                          {(1 << kFrac) - 1, (1 << kFrac) - 1},
	                          {lx, ly},
	                          {lx + 1, ly + 1},
	                          {lx + (1 << (kFrac - 1)), ly + (1 << (kFrac - 1))},
	                          {lx + (1 << kFrac) - 1, ly + (1 << kFrac) - 1},
	                          {lx + (1 << kFrac) - 1, 0},
	                          {0, ly + (1 << kFrac) - 1}};
	for (const auto &p : pts) {
		const std::string m = bilinearCheck<Tr>(t, rows, cols, p[0], p[1]);
		CHECK_MESSAGE(m.empty(), m);
	}
}

// characterization (NEEDS REVIEW): the line X = (numCols-1)<<20 or Y = (numRows-1)<<20 is on the table but returns 0
template <class Tr>
void bilinearLastLine() {
	const std::vector<typename Tr::T> list = Tr::positive();
	const std::vector<typename Tr::T> t = {list[1], list[4], list[5], list[3]};  // 2 x 2, non zero values
	const int64_t one = 1 << kFrac, half = 1 << (kFrac - 1);
	CHECK(Tr::bilinear(t.data(), 2, 2, (q31_t)one, 0) == 0);
	CHECK(Tr::bilinear(t.data(), 2, 2, (q31_t)one, (q31_t)half) == 0);
	CHECK(Tr::bilinear(t.data(), 2, 2, (q31_t)one, (q31_t)one) == 0);
	CHECK(Tr::bilinear(t.data(), 2, 2, 0, (q31_t)one) == 0);
	CHECK(Tr::bilinear(t.data(), 2, 2, (q31_t)half, (q31_t)one) == 0);
}

// C5 linear
template <class Tr>
void linearVsF32() {
	const std::vector<typename Tr::T> t = Tr::smooth();
	std::vector<float> tf;
	for (auto v : t) {
		tf.push_back((float)v);
		REQUIRE((double)tf.back() == (double)v);  // exact in float
	}
	arm_linear_interp_instance_f32 S;
	S.nValues = (uint32_t)t.size();
	S.x1 = 0.0f;
	S.xSpacing = 1.0f;
	S.pYData = tf.data();
	std::string first;
	for (int64_t x = -(2 << kFrac); x <= ((int64_t)(t.size() + 2) << kFrac) && first.empty(); x += 7919) {
		const float xf = (float)x / 1048576.0f;
		const float f = arm_linear_interp_f32(&S, xf);
		const Exp e = linearExp<Tr>(t, x);
		double maxabs = 0;
		if (x >= 0 && (x >> kFrac) < (int64_t)t.size() - 1)
			maxabs = std::fmax(std::fabs((double)t[(size_t)(x >> kFrac)]), std::fabs((double)t[(size_t)(x >> kFrac) + 1]));
		const double fErr = e.exact ? 0.0 : 8.0 * std::ldexp(maxabs, -24);
		// q result against the f32 result: replace DEF by the f32 value
		Exp g = e;
		g.def = (double)f;
		const std::string m = verdict(g, (double)Tr::linear(t.data(), (q31_t)x, (uint32_t)t.size()), fErr, "linear vs f32", x, 0);
		if (!m.empty()) first = m;
	}
	CHECK_MESSAGE(first.empty(), first);
}

// C5 bilinear
template <class Tr>
void bilinearVsF32() {
	const int rows = 4, cols = 5;
	const std::vector<typename Tr::T> t = matrixFrom(Tr::smooth(), rows, cols);
	std::vector<float> tf;
	double maxabs = 0;
	for (auto v : t) {
		tf.push_back((float)v);
		REQUIRE((double)tf.back() == (double)v);
		maxabs = std::fmax(maxabs, std::fabs((double)v));
	}
	arm_bilinear_interp_instance_f32 S;
	S.numRows = (uint16_t)rows;
	S.numCols = (uint16_t)cols;
	S.pData = tf.data();
	std::string first;
	for (int64_t X = 0; X <= ((int64_t)(cols + 1) << kFrac) && first.empty(); X += 98317)
		for (int64_t Y = 0; Y <= ((int64_t)(rows + 1) << kFrac) && first.empty(); Y += 98317) {
			const float f = arm_bilinear_interp_f32(&S, (float)X / 1048576.0f, (float)Y / 1048576.0f);
			Exp g = bilinearExp<Tr>(t, rows, cols, X, Y);
			g.def = (double)f;
			const double fErr = g.exact ? 0.0 : 42.0 * std::ldexp(maxabs, -24);
			const std::string m = verdict(g, (double)Tr::bilinear(t.data(), (uint16_t)rows, (uint16_t)cols, (q31_t)X, (q31_t)Y), fErr,
			                              "bilinear vs f32", X, Y);
			if (!m.empty()) first = m;
		}
	CHECK_MESSAGE(first.empty(), first);
}

}  // namespace

TEST_SUITE("DSP Interpolation q31 q15 q7") {

// ------------------------------------------------------------------------------------------------------------------
// C1: ARM's ids
// ------------------------------------------------------------------------------------------------------------------

TEST_CASE("[C1 arm_linear_interp_q31] the output for ARM's 42 inputs has SNR >= 100 dB and is within 2000 LSB of the reference" *
          KNOWN_ISSUE_UB_HIDDEN(B_DIQ_1)) {
	armLinear<TQ31>();
}
TEST_CASE("[C1 arm_linear_interp_q15] the output for ARM's 42 inputs has SNR >= 70 dB and is within 2 LSB of the reference") {
	armLinear<TQ15>();
}
TEST_CASE("[C1 arm_linear_interp_q7] the output for ARM's 42 inputs has SNR >= 20 dB and is within 2 LSB of the reference") {
	armLinear<TQ7>();
}
TEST_CASE("[C1 arm_bilinear_interp_q31] the output for ARM's 150 (X, Y) pairs has SNR >= 100 dB and is within 2000 LSB of the reference" *
          KNOWN_ISSUE_UB_HIDDEN(B_DIQ_2)) {
	armBilinear<TQ31>();
}
TEST_CASE("[C1 arm_bilinear_interp_q15] the output for ARM's 150 (X, Y) pairs has SNR >= 70 dB and is within 2 LSB of the reference") {
	armBilinear<TQ15>();
}
TEST_CASE("[C1 arm_bilinear_interp_q7] the output for ARM's 150 (X, Y) pairs has SNR >= 20 dB and is within 2 LSB of the reference") {
	armBilinear<TQ7>();
}

// ------------------------------------------------------------------------------------------------------------------
// C2: one sample per call into a guarded buffer, block sizes 0, 1, 3, 4, 5, 9
// ------------------------------------------------------------------------------------------------------------------

TEST_CASE("[C2 arm_linear_interp_q31] every sample of blocks of 0, 1, 3, 4, 5 and 9 calls is correct" *
          KNOWN_ISSUE_UB_HIDDEN(B_DIQ_1)) {
	guardLinear<TQ31>(TQ31::mixed());
}
TEST_CASE("[C2 arm_linear_interp_q31] positive table: every sample of blocks of 0, 1, 3, 4, 5 and 9 calls is correct") {
	guardLinear<TQ31>(TQ31::positive());
}
TEST_CASE("[C2 arm_linear_interp_q15] every sample of blocks of 0, 1, 3, 4, 5 and 9 calls is correct") {
	guardLinear<TQ15>(TQ15::mixed());
}
TEST_CASE("[C2 arm_linear_interp_q7] every sample of blocks of 0, 1, 3, 4, 5 and 9 calls is correct") {
	guardLinear<TQ7>(TQ7::mixed());
}
TEST_CASE("[C2 arm_bilinear_interp_q31] every sample of blocks of 0, 1, 3, 4, 5 and 9 calls is correct" *
          KNOWN_ISSUE_UB_HIDDEN(B_DIQ_2)) {
	guardBilinear<TQ31>(matrixFrom(TQ31::mixed(), 4, 5));
}
TEST_CASE("[C2 arm_bilinear_interp_q31] positive matrix: every sample of blocks of 0, 1, 3, 4, 5 and 9 calls is correct") {
	guardBilinear<TQ31>(matrixFrom(TQ31::positive(), 4, 5));
}
TEST_CASE("[C2 arm_bilinear_interp_q15] every sample of blocks of 0, 1, 3, 4, 5 and 9 calls is correct") {
	guardBilinear<TQ15>(matrixFrom(TQ15::mixed(), 4, 5));
}
TEST_CASE("[C2 arm_bilinear_interp_q7] every sample of blocks of 0, 1, 3, 4, 5 and 9 calls is correct") {
	guardBilinear<TQ7>(matrixFrom(TQ7::mixed(), 4, 5));
}

// ------------------------------------------------------------------------------------------------------------------
// C3: DEF sweeps (bounds derived in the header comment)
// ------------------------------------------------------------------------------------------------------------------

TEST_CASE("[C3 arm_linear_interp_q31] over the whole int32 input range the result is within the derived bound (-4 - y0/2^31, -y0/2^31] of the linear interpolation" *
          KNOWN_ISSUE_UB_HIDDEN(B_DIQ_1)) {
	linearSweepTest<TQ31>(TQ31::mixed());
}
TEST_CASE("[C3 arm_linear_interp_q31] positive table: over the whole int32 input range the result is within the derived bound of the linear interpolation") {
	linearSweepTest<TQ31>(TQ31::positive());
}
TEST_CASE("[C3 arm_linear_interp_q15] over the whole int32 input range the result is within the derived bound (-1 - y0/2^20, -y0/2^20] of the linear interpolation") {
	linearSweepTest<TQ15>(TQ15::mixed());
}
TEST_CASE("[C3 arm_linear_interp_q7] over the whole int32 input range the result is within the derived bound (-1 - y0/2^20, -y0/2^20] of the linear interpolation") {
	linearSweepTest<TQ7>(TQ7::mixed());
}
TEST_CASE("[C3 arm_bilinear_interp_q31] over a 4 x 5 matrix and beyond it the result is within the derived [-26.5, 2.5] LSB of the bilinear formula" *
          KNOWN_ISSUE_UB_HIDDEN(B_DIQ_2)) {
	bilinearSweepTest<TQ31>(matrixFrom(TQ31::mixed(), 4, 5));
}
TEST_CASE("[C3 arm_bilinear_interp_q31] positive matrix: over a 4 x 5 matrix and beyond it the result is within the derived [-26.5, 2.5] LSB of the bilinear formula") {
	bilinearSweepTest<TQ31>(matrixFrom(TQ31::positive(), 4, 5));
}
TEST_CASE("[C3 arm_bilinear_interp_q15] over a 4 x 5 matrix and beyond it the result is within the derived [-1.07, 0.07] LSB of the bilinear formula") {
	bilinearSweepTest<TQ15>(matrixFrom(TQ15::mixed(), 4, 5));
}
TEST_CASE("[C3 arm_bilinear_interp_q7] over a 4 x 5 matrix and beyond it the result is within the derived [-1.001, 0.001] LSB of the bilinear formula") {
	bilinearSweepTest<TQ7>(matrixFrom(TQ7::mixed(), 4, 5));
}

// ------------------------------------------------------------------------------------------------------------------
// C4: documented edge behaviour
// ------------------------------------------------------------------------------------------------------------------

TEST_CASE("[C4 arm_linear_interp_q31] an input below the range returns the first table value") {
	linearBelowRange<TQ31>();
}
TEST_CASE("[C4 arm_linear_interp_q15] an input below the range returns the first table value") {
	linearBelowRange<TQ15>();
}
TEST_CASE("[C4 arm_linear_interp_q7] an input below the range returns the first table value") {
	linearBelowRange<TQ7>();
}
TEST_CASE("[C4 arm_linear_interp_q31] an input at the last knot or above the range returns the last table value") {
	linearAboveRange<TQ31>();
}
TEST_CASE("[C4 arm_linear_interp_q15] an input at the last knot or above the range returns the last table value") {
	linearAboveRange<TQ15>();
}
TEST_CASE("[C4 arm_linear_interp_q7] an input at the last knot or above the range returns the last table value") {
	linearAboveRange<TQ7>();
}
TEST_CASE("[C4 arm_linear_interp_q31] the first and the last interval of the table interpolate within the derived bound") {
	linearEndIntervals<TQ31>();
}
TEST_CASE("[C4 arm_linear_interp_q15] the first and the last interval of the table interpolate within the derived bound") {
	linearEndIntervals<TQ15>();
}
TEST_CASE("[C4 arm_linear_interp_q7] the first and the last interval of the table interpolate within the derived bound") {
	linearEndIntervals<TQ7>();
}
TEST_CASE("[C4 arm_linear_interp_q31] a table of 2 entries interpolates its interval and clamps outside") {
	linearTwoEntries<TQ31>();
}
TEST_CASE("[C4 arm_linear_interp_q15] a table of 2 entries interpolates its interval and clamps outside") {
	linearTwoEntries<TQ15>();
}
TEST_CASE("[C4 arm_linear_interp_q7] a table of 2 entries interpolates its interval and clamps outside") {
	linearTwoEntries<TQ7>();
}
TEST_CASE("[C4 arm_linear_interp_q31] a table of 1 entry returns that entry for every input") {
	linearOneEntry<TQ31>();
}
TEST_CASE("[C4 arm_linear_interp_q15] a table of 1 entry returns that entry for every input") {
	linearOneEntry<TQ15>();
}
TEST_CASE("[C4 arm_linear_interp_q7] a table of 1 entry returns that entry for every input") {
	linearOneEntry<TQ7>();
}

TEST_CASE("[C4 arm_bilinear_interp_q31] a point outside the table returns 0") {
	bilinearOutside<TQ31>();
}
TEST_CASE("[C4 arm_bilinear_interp_q15] a point outside the table returns 0") {
	bilinearOutside<TQ15>();
}
TEST_CASE("[C4 arm_bilinear_interp_q7] a point outside the table returns 0") {
	bilinearOutside<TQ7>();
}
TEST_CASE("[C4 arm_bilinear_interp_q31] a 2 x 2 matrix interpolates its one cell within the derived bound") {
	bilinearTwoByTwo<TQ31>();
}
TEST_CASE("[C4 arm_bilinear_interp_q15] a 2 x 2 matrix interpolates its one cell within the derived bound") {
	bilinearTwoByTwo<TQ15>();
}
TEST_CASE("[C4 arm_bilinear_interp_q7] a 2 x 2 matrix interpolates its one cell within the derived bound") {
	bilinearTwoByTwo<TQ7>();
}
TEST_CASE("[C4 arm_bilinear_interp_q31] the first and the last cell of a 4 x 5 matrix interpolate within the derived bound with X as column and Y as row") {
	bilinearEndCells<TQ31>();
}
TEST_CASE("[C4 arm_bilinear_interp_q15] the first and the last cell of a 4 x 5 matrix interpolate within the derived bound with X as column and Y as row") {
	bilinearEndCells<TQ15>();
}
TEST_CASE("[C4 arm_bilinear_interp_q7] the first and the last cell of a 4 x 5 matrix interpolate within the derived bound with X as column and Y as row") {
	bilinearEndCells<TQ7>();
}

TEST_CASE("characterization: [C4 arm_bilinear_interp_q31] a coordinate on the last column or last row returns zero (NEEDS REVIEW)") {
	bilinearLastLine<TQ31>();
}
TEST_CASE("characterization: [C4 arm_bilinear_interp_q15] a coordinate on the last column or last row returns zero (NEEDS REVIEW)") {
	bilinearLastLine<TQ15>();
}
TEST_CASE("characterization: [C4 arm_bilinear_interp_q7] a coordinate on the last column or last row returns zero (NEEDS REVIEW)") {
	bilinearLastLine<TQ7>();
}

// ------------------------------------------------------------------------------------------------------------------
// C5: against the f32 functions of the same input
// ------------------------------------------------------------------------------------------------------------------

TEST_CASE("[C5 arm_linear_interp_q31] the q31 result agrees with arm_linear_interp_f32 within the derived bound plus the f32 error") {
	linearVsF32<TQ31>();
}
TEST_CASE("[C5 arm_linear_interp_q15] the q15 result agrees with arm_linear_interp_f32 within the derived bound plus the f32 error") {
	linearVsF32<TQ15>();
}
TEST_CASE("[C5 arm_linear_interp_q7] the q7 result agrees with arm_linear_interp_f32 within the derived bound plus the f32 error") {
	linearVsF32<TQ7>();
}
TEST_CASE("[C5 arm_bilinear_interp_q31] the q31 result agrees with arm_bilinear_interp_f32 within the derived bound plus the f32 error") {
	bilinearVsF32<TQ31>();
}
TEST_CASE("[C5 arm_bilinear_interp_q15] the q15 result agrees with arm_bilinear_interp_f32 within the derived bound plus the f32 error") {
	bilinearVsF32<TQ15>();
}
TEST_CASE("[C5 arm_bilinear_interp_q7] the q7 result agrees with arm_bilinear_interp_f32 within the derived bound plus the f32 error") {
	bilinearVsF32<TQ7>();
}

}
