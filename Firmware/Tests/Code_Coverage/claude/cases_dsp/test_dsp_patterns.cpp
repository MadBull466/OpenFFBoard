/*
 * Self-tests of the pattern loader and the comparison helpers of ffb_dsp_tests (support/DspPatterns.h). No CMSIS-DSP
 * function is under test here.
 *
 * Real pattern files used (Libraries/CMSIS-DSP/Testing/Patterns/DSP, the values are read from their "//" comment
 * lines and checked against hand derivations; the loader itself never reads the comments):
 *   FastMath/FastMathF32/Angles1_f32.txt   'W', 23 values. // 0.785398 = 0x3f490fdb: sign 0, exponent 0x7e = 2^-1,
 *                                          mantissa 0x490fdb -> 1.5707964 * 0.5 = 0.78539816 = float(pi/4)
 *   FastMath/FastMathF16/Angles1_f16.txt   'H', 23 values. // 0.785398 = 0x3a48: exponent 14 -> 2^-1, mantissa
 *                                          0x248/1024 = 0.5703125 -> 1.5703125 * 0.5 = 0.78515625;
 *                                          // 1.570796 = 0x3e48 -> 2^0 * 1.5703125; // 2.356194 = 0x40b6: 2^1 *
 *                                          (1 + 0xb6/1024) = 2.35546875
 *   FastMath/FastMathF64/Angles1_f64.txt   'D', 23 values. // 1.570796 = 0x3ff921fb54442d18 = pi/2 (double)
 *   FastMath/FastMathQ15/Angles1_q15.txt   'H': // 0.125 = 0x1000 = 0.125 * 32768; last // 1.0 = 0x7FFF (saturated)
 *   FastMath/FastMathQ31/Angles1_q31.txt   'W': // 0.125 = 0x10000000 = 0.125 * 2^31
 *   Interpolation/InterpolationQ7/InputX3_q7.txt 'B', 4 values: 0x00, 0x7F, 0x7F, 0x7F
 *   FastMath/FastMathQ63/DivDenInput1_s64.txt 'D', 12 values: // 9223372036854775807 = 0x7FFFFFFFFFFFFFFF,
 *                                          // -9223372036854775808 = 0x8000000000000000
 *   FastMath/FastMathQ63/RefNorm64To32_Norms1_s16.txt 'H', 4 values: // -30 = 0xFFE2, // -1 = 0xFFFF, // 1 = 0x0001
 * Temporary files (CRLF, comments, malformed) are written into the current directory (the build directory under
 * ctest) and removed again.
 *
 * Failure path of loadPattern(): it ends the test with doctest FAIL, which cannot be asserted from inside a test; the
 * non-failing loadPatternFile() carries the same error text and is tested instead.
 */
#include "doctest.h"

#include <climits>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>

#include "DspPatterns.h"

namespace {

// Writes a file in binary mode (so the bytes are exactly the given text) and removes it again
struct TempFile {
	std::string path;
	TempFile(const std::string &name, const std::string &content) : path(name) {
		std::ofstream f(path.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
		f << content;
	}
	~TempFile() { std::remove(path.c_str()); }
};

}  // namespace

TEST_SUITE("DspPatterns") {

TEST_CASE("[loader] a 'W' file gives the count and the float values by bit pattern") {
	dsp::Pattern p = dsp::loadPattern("FastMath/FastMathF32/Angles1_f32.txt");
	CHECK(p.type == 'W');
	CHECK(p.size() == 23);
	std::vector<float> v = p.f32();
	REQUIRE(v.size() == 23);
	CHECK(v[0] == 0.0f);                       // "0x0" (minimal digits for zero)
	CHECK(v[1] == 0.78539816339744831f);       // 0x3f490fdb = float(pi/4), see header comment
	CHECK(p.raw[1] == 0x3f490fdbu);
}

TEST_CASE("[loader] an 'H' float file gives the binary16 bit patterns and their exact float values") {
	dsp::Pattern p = dsp::loadPattern("FastMath/FastMathF16/Angles1_f16.txt");
	CHECK(p.type == 'H');
	std::vector<uint16_t> bits = p.f16Bits();
	REQUIRE(bits.size() == 23);
	CHECK(bits[0] == 0x0000);
	CHECK(bits[1] == 0x3a48);
	std::vector<float> f = p.f16AsFloat();
	CHECK(f[1] == 0.78515625f);   // hand derivation in the file header
	CHECK(f[2] == 1.5703125f);
	CHECK(f[3] == 2.35546875f);
}

TEST_CASE("[loader] a 'D' file gives the doubles by bit pattern") {
	dsp::Pattern p = dsp::loadPattern("FastMath/FastMathF64/Angles1_f64.txt");
	CHECK(p.type == 'D');
	std::vector<double> v = p.f64();
	REQUIRE(v.size() == 23);
	CHECK(v[0] == 0.0);
	CHECK(v[2] == 1.57079632679489661923);  // pi/2 as a double
}

TEST_CASE("[loader] q15 and q31 values are two's complement integers") {
	std::vector<int16_t> q15 = dsp::loadPattern("FastMath/FastMathQ15/Angles1_q15.txt").q15();
	REQUIRE(q15.size() == 23);
	CHECK(q15[0] == 0);
	CHECK(q15[1] == 0x1000);
	CHECK(q15[22] == 0x7FFF);
	std::vector<int32_t> q31 = dsp::loadPattern("FastMath/FastMathQ31/Angles1_q31.txt").q31();
	REQUIRE(q31.size() == 23);
	CHECK(q31[1] == 0x10000000);
	// negative numbers: FFE2 / FFFF are -30 / -1 (// -30 and // -1 in the file)
	std::vector<int16_t> s16 = dsp::loadPattern("FastMath/FastMathQ63/RefNorm64To32_Norms1_s16.txt").q15();
	REQUIRE(s16.size() == 4);
	CHECK(s16[0] == -30);
	CHECK(s16[1] == -1);
	CHECK(s16[2] == 1);
}

TEST_CASE("[loader] q7 values come from a 'B' file") {
	dsp::Pattern p = dsp::loadPattern("Interpolation/InterpolationQ7/InputX3_q7.txt");
	CHECK(p.type == 'B');
	std::vector<int8_t> v = p.q7();
	REQUIRE(v.size() == 4);
	CHECK(v[0] == 0);
	CHECK(v[1] == 127);
	CHECK(v[3] == 127);
}

TEST_CASE("[loader] 64 bit integer values are read signed and unsigned") {
	dsp::Pattern p = dsp::loadPattern("FastMath/FastMathQ63/DivDenInput1_s64.txt");
	CHECK(p.type == 'D');
	REQUIRE(p.size() == 12);
	CHECK(p.s64()[0] == INT64_MAX);
	CHECK(p.s64()[1] == INT64_MIN);
	CHECK(p.u64()[1] == 0x8000000000000000ull);
}

TEST_CASE("[loader] comment lines are never parsed, blank lines are skipped, hex digits are case insensitive") {
	// The comments contain valid-looking hex ("0xDEAD") that must not become values
	TempFile f("ffb_dsp_selftest_comments.txt",
		"H\n3\n// 0xDEAD is a comment\n0x00aB\n\n// 0xBEEF\n0xCdEf\n// 7\n0x7\n");
	dsp::Pattern p = dsp::loadPatternFile(f.path);
	REQUIRE_MESSAGE(p.ok(), p.error);
	CHECK(p.type == 'H');
	REQUIRE(p.size() == 3);
	CHECK(p.raw[0] == 0x00ab);
	CHECK(p.raw[1] == 0xcdef);
	CHECK(p.raw[2] == 0x7);
}

TEST_CASE("[loader] a file with CRLF line endings gives the same values as one with LF") {
	const std::string lf = "W\n2\n// 1\n0x3f800000\n// -2\n0xC0000000\n";
	std::string crlf;
	for (char c : lf) { if (c == '\n') crlf += '\r'; crlf += c; }
	TempFile a("ffb_dsp_selftest_lf.txt", lf), b("ffb_dsp_selftest_crlf.txt", crlf);
	dsp::Pattern pa = dsp::loadPatternFile(a.path), pb = dsp::loadPatternFile(b.path);
	REQUIRE_MESSAGE(pa.ok(), pa.error);
	REQUIRE_MESSAGE(pb.ok(), pb.error);
	CHECK(pa.raw == pb.raw);
	CHECK(pb.f32()[0] == 1.0f);
	CHECK(pb.f32()[1] == -2.0f);
	// a file without a final line ending
	TempFile c("ffb_dsp_selftest_noeol.txt", "B\n1\n0x80");
	dsp::Pattern pc = dsp::loadPatternFile(c.path);
	REQUIRE_MESSAGE(pc.ok(), pc.error);
	CHECK(pc.q7()[0] == -128);
}

TEST_CASE("[loader] a missing file returns an error instead of values") {
	dsp::Pattern p = dsp::loadPatternFile(dsp::patternDir() + "/FastMath/FastMathF32/DoesNotExist.txt");
	CHECK_FALSE(p.ok());
	CHECK(p.error.find("cannot open") != std::string::npos);
	CHECK(p.size() == 0);
}

TEST_CASE("[loader] the pattern directory is an absolute path with forward slashes") {
	std::string d = dsp::patternDir();
	CHECK_FALSE(d.empty());
	CHECK(d.find('\\') == std::string::npos);
	CHECK((d[0] == '/' || (d.size() > 2 && d[1] == ':')));
}

TEST_CASE("[loader] malformed files are rejected with a message naming the line") {
	struct Case { const char *name; const char *text; const char *expect; };
	const Case cases[] = {
		{"count larger than the values", "W\n3\n0x1\n0x2\n", "header says 3 values, file has 2"},
		{"count smaller than the values", "W\n1\n0x1\n0x2\n", "header says 1 values, file has 2"},
		{"unknown type letter", "X\n1\n0x1\n", "type letter"},
		{"value without 0x", "W\n1\n12\n", "line 3"},
		{"value too wide for the type", "H\n1\n0x12345\n", "at most 4 digits"},
		{"non hex digit", "B\n1\n0xZZ\n", "line 3"},
		{"no count", "W\n", "no value count"},
		{"empty file", "", "empty file"},
	};
	for (const Case &c : cases) {
		CAPTURE(c.name);
		TempFile f("ffb_dsp_selftest_bad.txt", c.text);
		dsp::Pattern p = dsp::loadPatternFile(f.path);
		CHECK_FALSE(p.ok());
		CHECK_MESSAGE(p.error.find(c.expect) != std::string::npos, p.error);
		CHECK(p.size() == 0);
	}
}

TEST_CASE("[loader] binary16 conversion covers zero, subnormals, normals, infinity and NaN") {
	CHECK(dsp::f16ToFloat(0x0000) == 0.0f);
	CHECK(std::signbit(dsp::f16ToFloat(0x8000)));
	CHECK(dsp::f16ToFloat(0x3c00) == 1.0f);
	CHECK(dsp::f16ToFloat(0xc000) == -2.0f);
	CHECK(dsp::f16ToFloat(0x7bff) == 65504.0f);                // largest finite binary16
	CHECK(dsp::f16ToFloat(0x0001) == std::ldexp(1.0f, -24));   // smallest subnormal 2^-24
	CHECK(dsp::f16ToFloat(0x03ff) == 1023.0f * std::ldexp(1.0f, -24));
	CHECK(dsp::f16ToFloat(0x0400) == std::ldexp(1.0f, -14));   // smallest normal 2^-14
	CHECK(std::isinf(dsp::f16ToFloat(0x7c00)));
	CHECK(dsp::f16ToFloat(0xfc00) < 0);
	CHECK(std::isnan(dsp::f16ToFloat(0x7e00)));
}

TEST_CASE("[compare] snr of identical vectors is 100000 and of a known error matches the formula") {
	std::vector<double> ref = {1.0, 1.0};
	CHECK(dsp::snr(ref, ref) == 100000.0);
	// signal energy 2, error energy 0.1^2 = 0.01 -> 10 log10(200) = 23.0103 dB
	std::vector<double> out = {1.0, 0.9};
	CHECK(dsp::snr(ref, out) == doctest::Approx(10.0 * std::log10(200.0)).epsilon(1e-12));
	// mixed element types are converted to double
	std::vector<float> outf = {1.0f, 0.5f};
	CHECK(dsp::snr(ref, outf) == doctest::Approx(10.0 * std::log10(2.0 / 0.25)).epsilon(1e-12));  // 9.0309 dB
}

TEST_CASE("[compare] snr is 0 for NaN, a length mismatch, an empty vector and a zero reference with zero output") {
	std::vector<double> ref = {1.0, 2.0};
	CHECK(dsp::snr(ref, std::vector<double>{1.0, std::nan("")}) == 0.0);
	CHECK(dsp::snr(std::vector<double>{std::nan(""), 1.0}, ref) == 0.0);
	CHECK(dsp::snr(ref, std::vector<double>{1.0}) == 0.0);
	CHECK(dsp::snr(std::vector<double>{}, std::vector<double>{}) == 0.0);
	CHECK(dsp::snr(std::vector<double>{0.0}, std::vector<double>{0.0}) == 0.0);  // 0/0 = NaN, like arm_snr_f32
}

TEST_CASE("[compare] closeAbsRel uses abs + rel*|ref| as the inclusive bound and reports the first bad sample") {
	// ref 4, bound 0.25 + 0.0625*4 = 0.5 (all exactly representable)
	std::vector<double> ref = {4.0, 4.0, 4.0};
	CHECK(dsp::closeAbsRel(ref, std::vector<double>{4.5, 3.5, 4.0}, 0.25, 0.0625));
	dsp::Check c = dsp::closeAbsRel(ref, std::vector<double>{4.0, 4.0, 4.5625}, 0.25, 0.0625);
	CHECK_FALSE(c);
	CHECK(c.index == 2);
	CHECK(c.message.find("sample 2") != std::string::npos);
	// negative reference: the bound uses |ref|
	CHECK(dsp::closeAbsRel(std::vector<double>{-4.0}, std::vector<double>{-4.5}, 0.25, 0.0625));
}

TEST_CASE("[compare] closeAbsRel fails for NaN and for different lengths") {
	std::vector<double> ref = {1.0, 2.0};
	CHECK_FALSE(dsp::closeAbsRel(ref, std::vector<double>{1.0, std::nan("")}, 1.0, 1.0));
	CHECK_FALSE(dsp::closeAbsRel(std::vector<double>{std::nan("")}, std::vector<double>{1.0}, 1.0, 1.0));
	dsp::Check c = dsp::closeAbsRel(ref, std::vector<double>{1.0}, 1.0, 1.0);
	CHECK_FALSE(c);
	CHECK(c.message.find("length mismatch") != std::string::npos);
}

TEST_CASE("[compare] withinLsb is inclusive, symmetric and safe at the extremes of the integer range") {
	std::vector<int32_t> ref = {0, 10, -5};
	CHECK(dsp::withinLsb(ref, std::vector<int32_t>{1, 8, -5}, 2));
	dsp::Check c = dsp::withinLsb(ref, std::vector<int32_t>{1, 8, -5}, 1);
	CHECK_FALSE(c);
	CHECK(c.index == 1);
	// the difference of INT32_MIN and INT32_MAX does not overflow
	CHECK_FALSE(dsp::withinLsb(std::vector<int32_t>{INT32_MIN}, std::vector<int32_t>{INT32_MAX}, 1000));
	CHECK(dsp::withinLsb(std::vector<int32_t>{INT32_MIN}, std::vector<int32_t>{INT32_MIN + 1}, 1));
	// different integer types, lsb 0 = exact
	CHECK(dsp::withinLsb(std::vector<int8_t>{-128, 127}, std::vector<int16_t>{-128, 127}, 0));
	CHECK_FALSE(dsp::withinLsb(std::vector<int8_t>{1}, std::vector<int8_t>{2}, 0));
	CHECK_FALSE(dsp::withinLsb(std::vector<int8_t>{1}, std::vector<int8_t>{1, 1}, 5));
}

TEST_CASE("[compare] exactEqual compares values and lengths") {
	CHECK(dsp::exactEqual(std::vector<int>{1, 2, 3}, std::vector<int>{1, 2, 3}));
	dsp::Check c = dsp::exactEqual(std::vector<int>{1, 2, 3}, std::vector<int>{1, 2, 4});
	CHECK_FALSE(c);
	CHECK(c.index == 2);
	CHECK_FALSE(dsp::exactEqual(std::vector<int>{1}, std::vector<int>{1, 2}));
	CHECK_FALSE(dsp::exactEqual(std::vector<float>{std::nanf("")}, std::vector<float>{std::nanf("")}));
}

TEST_CASE("[compare] a guarded buffer keeps its guard words if only the output is written") {
	dsp::GuardedBuffer<int16_t> b(4, 8);
	CHECK(b.size() == 4);
	CHECK(b.guardWords() == 8);
	for (size_t i = 0; i < b.size(); i++) b.data()[i] = static_cast<int16_t>(100 + i);
	CHECK(b.guardIntact());
	CHECK(b.firstDamaged() == 8);
	CHECK(b.output() == std::vector<int16_t>{100, 101, 102, 103});
}

TEST_CASE("[compare] a guarded buffer reports the first guard word that was written") {
	dsp::GuardedBuffer<float> first(3, 4);
	first.data()[3] = 1.0f;              // one past the output
	CHECK_FALSE(first.guardIntact());
	CHECK(first.firstDamaged() == 0);
	dsp::GuardedBuffer<uint32_t> last(3, 4);
	last.data()[3 + 3] = 0u;             // the last guard word
	CHECK_FALSE(last.guardIntact());
	CHECK(last.firstDamaged() == 3);
	dsp::GuardedBuffer<int8_t> bytes(2, 3);
	bytes.data()[2 + 1] = 0;             // a byte buffer: second guard byte
	CHECK(bytes.firstDamaged() == 1);
}

TEST_CASE("[compare] the output part of a guarded buffer is poisoned so a function that writes nothing is visible") {
	dsp::GuardedBuffer<uint16_t> b(3, 2);
	CHECK(b.output() == std::vector<uint16_t>{0xCDCD, 0xCDCD, 0xCDCD});
	dsp::GuardedBuffer<int> empty(0, 2);   // zero length output is allowed
	CHECK(empty.output().empty());
	CHECK(empty.guardIntact());
}

}
