/*
 * BasicMath helpers of work order FF64: arm_abs_q15 and arm_abs_q31 (C2 and C4, beyond the smoke tests of
 * test_dsp_smoke.cpp). arm_add_f32 / arm_add_f16 belong to the f32 / f16 work orders.
 *
 * ARM tests these only in the BasicMath suites of other work orders; this file covers the helper behaviour the
 * firmware relies on. No ARM pattern is used, the expected values are DEF (saturating |x|, computed in int64):
 *   DOC (Source/BasicMathFunctions/arm_abs_q15.c:44-46): "The function uses saturating arithmetic. The Q15 value -1
 *   (0x8000) will be saturated to the maximum allowable positive value 0x7FFF." and arm_abs_q31.c the same with
 *   0x80000000 -> 0x7FFFFFFF. Every other value gives |x| exactly, 0 gives 0.
 * The loops are unrolled in groups of 4 (ARM_MATH_LOOPUNROLL is defined in this build) with a scalar tail of
 * blockSize % 4, so the tests use block sizes 0, 1, 3, 4, 5, 8, 9 and put the most negative value at every position.
 *
 * Host vs target (C6): the host runs the portable branch (`in > 0 ? in : (in == MIN ? MAX : -in)`); the Cortex-M4 branch
 * (ARM_MATH_DSP) uses the saturating __QSUB16 / __QSUB instructions. Both give the same results by DOC. The portable
 * branch needs no signed overflow because the minimum value is handled before the negation.
 * Not covered: the Helium (MVE) and NEON branches (not compiled here).
 */
#include "doctest.h"

#include "DspPatterns.h"
#include "arm_math.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

namespace {

// DEF: saturating absolute value
template <class T>
T satAbs(T x) {
	int64_t v = x;
	if (v < 0) v = -v;
	const int64_t mx = std::numeric_limits<T>::max();
	return static_cast<T>(std::min(v, mx));
}

template <class T>
std::vector<T> poolOf();
template <>
std::vector<q15_t> poolOf<q15_t>() {
	return {0, 1, -1, 5, -5, 0x7FFF, -0x7FFF, (q15_t)0x8000, 1234, -1234, 0x4000, -0x4000, 2, -2, 0x7FFE, (q15_t)-0x7FFE};
}
template <>
std::vector<q31_t> poolOf<q31_t>() {
	return {0, 1, -1, 5, -5, INT32_MAX, -INT32_MAX, INT32_MIN, 123456789, -123456789, 0x40000000, -0x40000000, 2, -2,
	        INT32_MAX - 1, INT32_MIN + 2};
}

template <class T, class Fn>
void checkBlock(Fn fn, const std::vector<T> &in, size_t n) {
	std::vector<T> src(in.begin(), in.begin() + static_cast<std::ptrdiff_t>(n));
	std::vector<T> ref;
	for (T x : src) ref.push_back(satAbs(x));
	dsp::GuardedBuffer<T> out(n);
	const T dummy = 1;
	fn(n ? src.data() : &dummy, out.data(), static_cast<uint32_t>(n));
	CHECK(out.guardIntact());
	auto c = dsp::exactEqual(ref, out.output());
	CHECK_MESSAGE(c, c.message);
}

// the most negative value at position pos of a block of n, other samples -3
template <class T, class Fn>
void checkMinAt(Fn fn, size_t n, size_t pos) {
	std::vector<T> src(n, static_cast<T>(-3));
	src[pos] = std::numeric_limits<T>::min();
	std::vector<T> ref(n, static_cast<T>(3));
	ref[pos] = std::numeric_limits<T>::max();
	dsp::GuardedBuffer<T> out(n);
	fn(src.data(), out.data(), static_cast<uint32_t>(n));
	CHECK(out.guardIntact());
	auto c = dsp::exactEqual(ref, out.output());
	CHECK_MESSAGE(c, c.message);
}

}  // namespace

TEST_SUITE("DSP BasicMath helpers") {

TEST_CASE("[C2 arm_abs_q15] block sizes 0, 1, 3, 4, 5, 8, 9 and 16 give the saturated magnitude of every sample and write nothing else") {
	for (size_t n : {0u, 1u, 3u, 4u, 5u, 8u, 9u, 16u}) {
		CAPTURE(n);
		checkBlock<q15_t>(arm_abs_q15, poolOf<q15_t>(), n);
	}
}
TEST_CASE("[C2 arm_abs_q31] block sizes 0, 1, 3, 4, 5, 8, 9 and 16 give the saturated magnitude of every sample and write nothing else") {
	for (size_t n : {0u, 1u, 3u, 4u, 5u, 8u, 9u, 16u}) {
		CAPTURE(n);
		checkBlock<q31_t>(arm_abs_q31, poolOf<q31_t>(), n);
	}
}
TEST_CASE("[C2 arm_abs_q15] block size 0 does not write the output") {
	const q15_t in[1] = {-4};
	dsp::GuardedBuffer<q15_t> out(3);
	arm_abs_q15(in, out.data(), 0);
	CHECK(out.output() == std::vector<q15_t>(3, static_cast<q15_t>(0xCDCD)));
	CHECK(out.guardIntact());
}
TEST_CASE("[C2 arm_abs_q31] block size 0 does not write the output") {
	const q31_t in[1] = {-4};
	dsp::GuardedBuffer<q31_t> out(3);
	arm_abs_q31(in, out.data(), 0);
	CHECK(out.output() == std::vector<q31_t>(3, static_cast<q31_t>(0xCDCDCDCDu)));
	CHECK(out.guardIntact());
}

TEST_CASE("[C4 arm_abs_q15] 0x8000 (-1.0) saturates to 0x7FFF at every position of the unrolled loop and of the tail") {
	// block size 9 = two unrolled groups of 4 (positions 0-7) plus a tail of 1 (position 8)
	for (size_t pos = 0; pos < 9; pos++) {
		CAPTURE(pos);
		checkMinAt<q15_t>(arm_abs_q15, 9, pos);
	}
}
TEST_CASE("[C4 arm_abs_q31] -1 (0x80000000) saturates to 0x7FFFFFFF at every position of the unrolled loop and of the tail") {
	for (size_t pos = 0; pos < 9; pos++) {
		CAPTURE(pos);
		checkMinAt<q31_t>(arm_abs_q31, 9, pos);
	}
}
TEST_CASE("[C4 arm_abs_q15] -32767 gives 32767 exactly (no saturation needed) and 0x7FFF stays 0x7FFF") {
	const q15_t in[4] = {-32767, 0x7FFF, 0, -0};
	q15_t out[4] = {1, 1, 1, 1};
	arm_abs_q15(in, out, 4);
	CHECK(out[0] == 32767);
	CHECK(out[1] == 0x7FFF);
	CHECK(out[2] == 0);
	CHECK(out[3] == 0);
}
TEST_CASE("[C4 arm_abs_q31] INT32_MIN+1 gives INT32_MAX exactly (no saturation needed) and zero stays zero") {
	const q31_t in[4] = {INT32_MIN + 1, INT32_MAX, 0, -0};
	q31_t out[4] = {1, 1, 1, 1};
	arm_abs_q31(in, out, 4);
	CHECK(out[0] == INT32_MAX);
	CHECK(out[1] == INT32_MAX);
	CHECK(out[2] == 0);
	CHECK(out[3] == 0);
}
TEST_CASE("[C4 arm_abs_q15] the result equals the saturated absolute value for all 65536 input values") {
	// DEF: saturating |x| = min(|x|, 32767); blocks of 64 so the unrolled loop is used
	std::vector<q15_t> src(65536), out(65536);
	for (int i = 0; i < 65536; i++) src[i] = static_cast<q15_t>(i - 32768);
	for (size_t off = 0; off < src.size(); off += 64) arm_abs_q15(&src[off], &out[off], 64);
	size_t bad = 0, first = 0;
	for (size_t i = 0; i < src.size(); i++)
		if (out[i] != satAbs(src[i])) { if (!bad) first = i; bad++; }
	CAPTURE(first);
	CHECK(bad == 0);
}

}
