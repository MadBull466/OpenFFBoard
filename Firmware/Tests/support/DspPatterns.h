/*
 * DspPatterns: loader for ARM's CMSIS-DSP reference patterns and the comparison helpers of ffb_dsp_tests
 * (phase P11, Tests/dsp/CMSIS_DSP_REQUIREMENTS.md, section "Infrastructure").
 *
 * Pattern files: Libraries/CMSIS-DSP/Testing/Patterns/DSP/<Module>/<Suite>/<name>.txt, read at RUN TIME from
 * FFB_CMSIS_PATTERN_DIR (CMake define: absolute path, forward slashes). Nothing is copied under Firmware/Tests.
 * Format (written by Testing/PatternGeneration/Tools.py, _writeVector*):
 *     <type letter>          W = 32 bit word, H = 16 bit half word, B = 8 bit byte, D = 64 bit double word
 *     <count>                number of values (decimal)
 *     // <readable value>    comment line before each value: NEVER parsed (it is rounded and lossy)
 *     0x<hex>                the value (raw IEEE bit pattern for float types, two's complement for q types)
 * The loader skips blank and "//" lines, strips CR (CRLF checkouts), reads in binary mode, checks that the count in
 * the header matches the number of values and that every value fits the width of the type letter.
 *
 * A missing or malformed file FAILS the test (loadPattern); it is never a skip, because CI checks out the submodule.
 * loadPatternFile() is the non-failing variant that returns the error text (used by the loader's own tests).
 *
 * Comparison helpers (scheme of ARM's framework, Testing/FrameworkSource/Error.cpp; thresholds are the caller's):
 *   snr()           10 log10(sum ref^2 / sum (ref-out)^2) in double. NaN anywhere -> 0 (so any threshold fails),
 *                   zero error -> 100000, like arm_snr_f32. A length mismatch returns 0.
 *   closeAbsRel()   per sample |out-ref| <= abs + rel*|ref| (ARM ASSERT_CLOSE_ERROR). Stricter than ARM: NaN fails.
 *   withinLsb()     per sample |out-ref| <= lsb for integers (ARM ASSERT_NEAR_EQ); lsb = 0 is exact.
 *   exactEqual()    all samples equal (ARM ASSERT_EQ; min/max values, indexes).
 *   GuardedBuffer   output buffer with guard words behind it (ARM ASSERT_EMPTY_TAIL).
 * The checks return a Check, which converts to bool and carries a message with index, value, reference and bound:
 *     auto c = dsp::closeAbsRel(ref, out, 1e-5, 1e-6);  CHECK_MESSAGE(c, c.message);
 */
#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

namespace dsp {

// Directory of the pattern files (FFB_CMSIS_PATTERN_DIR, without trailing slash)
std::string patternDir();

// Parsed pattern file. type is 'W', 'H', 'B' or 'D'; raw holds the values zero extended to 64 bits.
struct Pattern {
	std::string path;
	std::string error;           // empty if the file was loaded
	char type = 0;
	std::vector<uint64_t> raw;

	bool ok() const { return error.empty(); }
	size_t size() const { return raw.size(); }

	// Typed copies. A wrong type letter fails the test (doctest FAIL) with the file name in the message.
	std::vector<double> f64() const;       // 'D': IEEE binary64 bit pattern
	std::vector<float> f32() const;        // 'W': IEEE binary32 bit pattern
	std::vector<uint16_t> f16Bits() const; // 'H': IEEE binary16 bit pattern (the raw words)
	std::vector<float> f16AsFloat() const; // 'H': binary16 converted to float (exact), needs no _Float16 support
	std::vector<int32_t> q31() const;      // 'W': two's complement (also s32 / u32 files read as signed)
	std::vector<int16_t> q15() const;      // 'H': two's complement (also the s16 config files)
	std::vector<int8_t> q7() const;        // 'B': two's complement
	std::vector<int64_t> s64() const;      // 'D': two's complement (Q63 patterns)
	std::vector<uint64_t> u64() const;     // 'D'
};

// Reads a pattern file by path. Never fails the test: on any problem Pattern::error describes it.
Pattern loadPatternFile(const std::string &path);
// Reads patternDir()/relativePath (e.g. "FastMath/FastMathF32/Sin1_f32.txt"). Fails the test if it cannot be read.
Pattern loadPattern(const std::string &relativePath);

// Binary16 -> float, exact (zero, subnormals, normals, infinity, NaN)
float f16ToFloat(uint16_t bits);

// ---------------------------------------------------------------------------------------------------------------
// Comparison helpers
// ---------------------------------------------------------------------------------------------------------------
struct Check {
	bool ok = true;
	size_t index = 0;        // first offending sample
	std::string message;     // empty if ok
	explicit operator bool() const { return ok; }
};

namespace detail {
std::string describe(const char *what, size_t index, double value, double reference, double bound);
Check fail(const char *what, size_t index, double value, double reference, double bound);
Check lengthMismatch(size_t a, size_t b);
}

double snr(const double *ref, const double *out, size_t n);

template <class A, class B>
double snr(const std::vector<A> &ref, const std::vector<B> &out) {
	if (ref.size() != out.size() || ref.empty()) return 0.0;
	std::vector<double> r(ref.begin(), ref.end()), o(out.begin(), out.end());
	return snr(r.data(), o.data(), r.size());
}

// |out - ref| <= absTol + relTol*|ref| for every sample; NaN in either fails
template <class A, class B>
Check closeAbsRel(const std::vector<A> &ref, const std::vector<B> &out, double absTol, double relTol) {
	if (ref.size() != out.size()) return detail::lengthMismatch(ref.size(), out.size());
	for (size_t i = 0; i < ref.size(); i++) {
		double r = static_cast<double>(ref[i]), o = static_cast<double>(out[i]);
		double bound = absTol + relTol * std::fabs(r);
		if (std::isnan(r) || std::isnan(o) || !(std::fabs(o - r) <= bound))
			return detail::fail("close error", i, o, r, bound);
	}
	return Check();
}

// |out - ref| <= lsb for integer samples (lsb 0 = exact)
template <class A, class B>
Check withinLsb(const std::vector<A> &ref, const std::vector<B> &out, int64_t lsb) {
	static_assert(std::is_integral<A>::value && std::is_integral<B>::value, "withinLsb is for integer types");
	if (ref.size() != out.size()) return detail::lengthMismatch(ref.size(), out.size());
	for (size_t i = 0; i < ref.size(); i++) {
		int64_t d = static_cast<int64_t>(out[i]) - static_cast<int64_t>(ref[i]);
		if (d < 0) d = -d;
		if (d > lsb) return detail::fail("integer error", i, static_cast<double>(out[i]), static_cast<double>(ref[i]), static_cast<double>(lsb));
	}
	return Check();
}

// Exactly equal (integers; floats compare by value, NaN never equals)
template <class A, class B>
Check exactEqual(const std::vector<A> &ref, const std::vector<B> &out) {
	if (ref.size() != out.size()) return detail::lengthMismatch(ref.size(), out.size());
	for (size_t i = 0; i < ref.size(); i++)
		if (!(ref[i] == out[i]))
			return detail::fail("not equal", i, static_cast<double>(out[i]), static_cast<double>(ref[i]), 0.0);
	return Check();
}

// Output buffer of n elements followed by guardWords guard elements (filled with 0xA5 bytes). The output part is
// poisoned with 0xCD bytes, so "never written" is detectable too. T must be trivially copyable.
template <class T>
class GuardedBuffer {
public:
	explicit GuardedBuffer(size_t n, size_t guardWords = 8) : n_(n), guard_(guardWords), store_(n + guardWords) {
		static_assert(std::is_trivially_copyable<T>::value, "GuardedBuffer needs a trivially copyable type");
		if (n_) std::memset(static_cast<void *>(store_.data()), 0xCD, n_ * sizeof(T));
		std::memset(static_cast<void *>(store_.data() + n_), 0xA5, guard_ * sizeof(T));
	}
	T *data() { return store_.data(); }
	size_t size() const { return n_; }
	size_t guardWords() const { return guard_; }
	// copy of the output part
	std::vector<T> output() const { return std::vector<T>(store_.begin(), store_.begin() + n_); }
	// true if no guard element was changed. firstDamaged() returns its index relative to the end of the output
	bool guardIntact() const { return firstDamaged() == guard_; }
	size_t firstDamaged() const {
		const unsigned char *g = reinterpret_cast<const unsigned char *>(store_.data() + n_);
		for (size_t i = 0; i < guard_ * sizeof(T); i++)
			if (g[i] != 0xA5) return i / sizeof(T);
		return guard_;
	}
private:
	size_t n_, guard_;
	std::vector<T> store_;
};

}  // namespace dsp
