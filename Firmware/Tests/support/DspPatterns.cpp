/* Pattern loader and comparison helpers of ffb_dsp_tests, see DspPatterns.h */
#include "DspPatterns.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <sstream>

#include "doctest.h"

#ifndef FFB_CMSIS_PATTERN_DIR
#error "FFB_CMSIS_PATTERN_DIR must be defined by CMake (absolute path of Libraries/CMSIS-DSP/Testing/Patterns/DSP)"
#endif

namespace dsp {

std::string patternDir() { return FFB_CMSIS_PATTERN_DIR; }

namespace {

std::string trim(const std::string &s) {
	size_t a = 0, b = s.size();
	while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) a++;
	while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r' || s[b - 1] == '\n')) b--;
	return s.substr(a, b - a);
}

unsigned digitsOf(char type) {
	switch (type) {
	case 'W': return 8;
	case 'H': return 4;
	case 'B': return 2;
	case 'D': return 16;
	}
	return 0;
}

// "0x1F" -> 31. false if the token is not 0x followed by 1..maxDigits hex digits
bool parseHex(const std::string &tok, unsigned maxDigits, uint64_t &out) {
	if (tok.size() < 3 || tok[0] != '0' || (tok[1] != 'x' && tok[1] != 'X')) return false;
	if (tok.size() - 2 > maxDigits) return false;
	uint64_t v = 0;
	for (size_t i = 2; i < tok.size(); i++) {
		char c = tok[i];
		unsigned d;
		if (c >= '0' && c <= '9') d = static_cast<unsigned>(c - '0');
		else if (c >= 'a' && c <= 'f') d = static_cast<unsigned>(c - 'a' + 10);
		else if (c >= 'A' && c <= 'F') d = static_cast<unsigned>(c - 'A' + 10);
		else return false;
		v = (v << 4) | d;
	}
	out = v;
	return true;
}

void requireType(const Pattern &p, char want, const char *what) {
	if (!p.ok()) FAIL("pattern " << p.path << " was not loaded: " << p.error);
	if (p.type != want)
		FAIL("pattern " << p.path << " has type letter '" << p.type << "', " << what << " needs '" << want << "'");
}

template <class T, class U>
T bitsTo(U u) {
	T t;
	static_assert(sizeof(T) == sizeof(U), "size");
	std::memcpy(&t, &u, sizeof t);
	return t;
}

}  // namespace

float f16ToFloat(uint16_t h) {
	uint32_t sign = static_cast<uint32_t>(h >> 15) << 31;
	uint32_t exp = (h >> 10) & 0x1F;
	uint32_t man = h & 0x3FF;
	uint32_t bits;
	if (exp == 0) {
		if (man == 0) {
			bits = sign;  // +-0
		} else {  // subnormal: normalise
			int e = -1;
			do { e++; man <<= 1; } while (!(man & 0x400));
			bits = sign | (static_cast<uint32_t>(127 - 15 - e) << 23) | ((man & 0x3FF) << 13);
		}
	} else if (exp == 31) {
		bits = sign | 0x7F800000u | (man << 13);  // inf / NaN
	} else {
		bits = sign | ((exp + 127 - 15) << 23) | (man << 13);
	}
	return bitsTo<float>(bits);
}

Pattern loadPatternFile(const std::string &path) {
	Pattern p;
	p.path = path;
	std::ifstream f(path.c_str(), std::ios::in | std::ios::binary);  // binary: CRLF checkouts do not matter
	if (!f) {
		p.error = "cannot open file";
		return p;
	}
	std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
	std::istringstream in(content);
	std::string line;
	long count = -1;
	unsigned digits = 0;
	size_t lineNo = 0;
	auto err = [&](const std::string &m) { p.error = "line " + std::to_string(lineNo) + ": " + m; p.raw.clear(); return p; };
	while (std::getline(in, line)) {
		lineNo++;
		line = trim(line);
		if (line.empty() || line.compare(0, 2, "//") == 0) continue;
		if (p.type == 0) {
			if (line.size() != 1 || digitsOf(line[0]) == 0) return err("expected type letter W, H, B or D, found '" + line + "'");
			p.type = line[0];
			digits = digitsOf(p.type);
		} else if (count < 0) {
			char *end = nullptr;
			long v = std::strtol(line.c_str(), &end, 10);
			if (end == line.c_str() || *end != 0 || v < 0) return err("expected the value count, found '" + line + "'");
			count = v;
			p.raw.reserve(static_cast<size_t>(count));
		} else {
			uint64_t v;
			if (!parseHex(line, digits, v)) return err("expected 0x<hex> with at most " + std::to_string(digits) + " digits for type '" + std::string(1, p.type) + "', found '" + line + "'");
			p.raw.push_back(v);
		}
	}
	if (p.type == 0) return err("empty file (no type letter)");
	if (count < 0) return err("no value count");
	if (static_cast<long>(p.raw.size()) != count) {
		return err("header says " + std::to_string(count) + " values, file has " + std::to_string(p.raw.size()));
	}
	return p;
}

Pattern loadPattern(const std::string &relativePath) {
	Pattern p = loadPatternFile(patternDir() + "/" + relativePath);
	if (!p.ok())
		FAIL("CMSIS-DSP pattern file " << p.path << ": " << p.error
			<< " (FFB_CMSIS_PATTERN_DIR=" << patternDir() << "; is the submodule Firmware/Libraries/CMSIS-DSP checked out with Testing/Patterns?)");
	return p;
}

std::vector<double> Pattern::f64() const {
	requireType(*this, 'D', "f64");
	std::vector<double> v;
	for (uint64_t r : raw) v.push_back(bitsTo<double>(r));
	return v;
}
std::vector<float> Pattern::f32() const {
	requireType(*this, 'W', "f32");
	std::vector<float> v;
	for (uint64_t r : raw) v.push_back(bitsTo<float>(static_cast<uint32_t>(r)));
	return v;
}
std::vector<uint16_t> Pattern::f16Bits() const {
	requireType(*this, 'H', "f16");
	std::vector<uint16_t> v;
	for (uint64_t r : raw) v.push_back(static_cast<uint16_t>(r));
	return v;
}
std::vector<float> Pattern::f16AsFloat() const {
	std::vector<float> v;
	for (uint16_t b : f16Bits()) v.push_back(f16ToFloat(b));
	return v;
}
std::vector<int32_t> Pattern::q31() const {
	requireType(*this, 'W', "q31");
	std::vector<int32_t> v;
	for (uint64_t r : raw) v.push_back(static_cast<int32_t>(static_cast<uint32_t>(r)));
	return v;
}
std::vector<int16_t> Pattern::q15() const {
	requireType(*this, 'H', "q15");
	std::vector<int16_t> v;
	for (uint64_t r : raw) v.push_back(static_cast<int16_t>(static_cast<uint16_t>(r)));
	return v;
}
std::vector<int8_t> Pattern::q7() const {
	requireType(*this, 'B', "q7");
	std::vector<int8_t> v;
	for (uint64_t r : raw) v.push_back(static_cast<int8_t>(static_cast<uint8_t>(r)));
	return v;
}
std::vector<int64_t> Pattern::s64() const {
	requireType(*this, 'D', "s64");
	std::vector<int64_t> v;
	for (uint64_t r : raw) v.push_back(static_cast<int64_t>(r));
	return v;
}
std::vector<uint64_t> Pattern::u64() const {
	requireType(*this, 'D', "u64");
	return raw;
}

namespace detail {
std::string describe(const char *what, size_t index, double value, double reference, double bound) {
	char buf[200];
	std::snprintf(buf, sizeof buf, "%s at sample %zu: value %.17g, reference %.17g, bound %.17g", what, index, value, reference, bound);
	return buf;
}
Check fail(const char *what, size_t index, double value, double reference, double bound) {
	Check c;
	c.ok = false;
	c.index = index;
	c.message = describe(what, index, value, reference, bound);
	return c;
}
Check lengthMismatch(size_t a, size_t b) {
	Check c;
	c.ok = false;
	c.message = "length mismatch: reference " + std::to_string(a) + " samples, output " + std::to_string(b);
	return c;
}
}  // namespace detail

double snr(const double *ref, const double *out, size_t n) {
	double energySignal = 0.0, energyError = 0.0;
	for (size_t i = 0; i < n; i++) {
		if (std::isnan(ref[i]) || std::isnan(out[i])) return 0.0;
		energySignal += ref[i] * ref[i];
		double d = ref[i] - out[i];
		energyError += d * d;
	}
	if (std::isnan(energyError)) return 0.0;
	double s = 10.0 * std::log10(energySignal / energyError);
	if (std::isnan(s)) return 0.0;
	if (std::isinf(s)) return s > 0 ? 100000.0 : -100000.0;
	return s;
}

}  // namespace dsp
