/*
 * Smoke tests of ffb_dsp_tests: the CMSIS-DSP libraries (ffb_cmsis_dsp) link and run in this executable, one plain
 * test per data type family. The f16 family is in test_dsp_smoke_f16.cpp (needs the f16 compile definitions).
 * Expected values are hand derived (the requirement ids are those of dsp/CMSIS_DSP_REQUIREMENTS.md):
 *  - arm_abs_q15: DOC in arm_abs_q15.c: "The Q15 value -1 (0x8000) will be saturated to the maximum allowable
 *    positive value 0x7FFF". abs(-1) -> 1, abs(5) -> 5, abs(0x7FFF) -> 0x7FFF.
 *  - arm_add_f32: element wise sum, all operands and results are exactly representable in binary32.
 * Host vs target: the host runs the portable C path (no ARM_MATH_DSP intrinsics).
 */
#include "doctest.h"

#include "DspPatterns.h"
#include "arm_math.h"

TEST_SUITE("DspSmoke") {

TEST_CASE("[C4 arm_abs_q15] the most negative q15 value saturates to 0x7FFF and the other values give their magnitude") {
	// The block size 5 includes the scalar tail of the unrolled loop (ARM_MATH_LOOPUNROLL works on groups of 4)
	const q15_t in[5] = {(q15_t)0x8000, 0x7FFF, -1, 5, (q15_t)-32767};
	dsp::GuardedBuffer<q15_t> out(5);
	arm_abs_q15(in, out.data(), 5);
	CHECK(out.output() == std::vector<q15_t>{0x7FFF, 0x7FFF, 1, 5, 32767});
	CHECK(out.guardIntact());
}

TEST_CASE("[C3 arm_add_f32] the sum of two f32 vectors is exact for exactly representable values") {
	const float a[5] = {1.5f, -2.0f, 0.25f, 4.0f, 1024.0f};
	const float b[5] = {2.5f, 0.5f, -0.25f, -4.0f, 0.0625f};
	dsp::GuardedBuffer<float> out(5);
	arm_add_f32(a, b, out.data(), 5);
	CHECK(out.output() == std::vector<float>{4.0f, -1.5f, 0.0f, 0.0f, 1024.0625f});
	CHECK(out.guardIntact());
}

TEST_CASE("[C4 arm_abs_q31] the most negative q31 value saturates to 0x7FFFFFFF") {
	// DOC in arm_abs_q31.c: the Q31 value -1 (0x80000000) is saturated to the maximum positive value 0x7FFFFFFF
	const q31_t in[3] = {(q31_t)0x80000000, -7, 9};
	dsp::GuardedBuffer<q31_t> out(3);
	arm_abs_q31(in, out.data(), 3);
	CHECK(out.output() == std::vector<q31_t>{0x7FFFFFFF, 7, 9});
	CHECK(out.guardIntact());
}

TEST_CASE("[C4 CommonTables] the sine table the f32 functions use is linked in and has the documented size") {
	// FAST_MATH_TABLE_SIZE = 512 intervals, the table has 513 entries (arm_common_tables.h); sin(0) = 0, sin(pi/2) = 1
	CHECK(FAST_MATH_TABLE_SIZE == 512);
	CHECK(arm_sin_f32(0.0f) == 0.0f);
	CHECK(arm_sin_f32(1.57079632679f) == doctest::Approx(1.0).epsilon(1e-5));
}

}
