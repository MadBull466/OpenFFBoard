/*
 * f16 smoke test of ffb_dsp_tests. This file is compiled with -D__ARM_FP16_FORMAT_IEEE=1 -D__fp16=_Float16 (CMake,
 * files named test_dsp_*_f16.cpp; lead decision L2) and only if the compiler supports _Float16. CMSIS-DSP then defines
 * float16_t as _Float16 and ARM_FLOAT16_SUPPORTED.
 * arm_add_f16 on hand values: 1.5 + 2.5 = 4, -2 + 0.5 = -1.5, 0.25 - 0.25 = 0, 1024 + 0.5 = 1024.5 (not exactly
 * representable in binary16: spacing above 1024 is 1, so ties-to-even gives 1024), all others are exact.
 */
#include "doctest.h"

#include "DspPatterns.h"
#include "arm_math_f16.h"

#ifndef ARM_FLOAT16_SUPPORTED
#error "ARM_FLOAT16_SUPPORTED is not defined: the f16 compile definitions are missing for this file"
#endif

TEST_SUITE("DspSmoke") {

TEST_CASE("[C3 arm_add_f16] the sum of two f16 vectors matches the hand-derived binary16 values") {
	CHECK(sizeof(float16_t) == 2);
	const float16_t a[5] = {(float16_t)1.5f, (float16_t)-2.0f, (float16_t)0.25f, (float16_t)4.0f, (float16_t)1024.0f};
	const float16_t b[5] = {(float16_t)2.5f, (float16_t)0.5f, (float16_t)-0.25f, (float16_t)-4.0f, (float16_t)0.5f};
	dsp::GuardedBuffer<float16_t> out(5);
	arm_add_f16(a, b, out.data(), 5);
	std::vector<float16_t> o = out.output();
	CHECK((float)o[0] == 4.0f);
	CHECK((float)o[1] == -1.5f);
	CHECK((float)o[2] == 0.0f);
	CHECK((float)o[3] == 0.0f);
	CHECK((float)o[4] == 1024.0f);  // 1024.5 is a tie between 1024 and 1025 in binary16: round to even gives 1024
	CHECK(out.guardIntact());
}

TEST_CASE("[loader] a float16_t built from a pattern bit pattern equals the software conversion") {
	std::vector<uint16_t> bits = dsp::loadPattern("FastMath/FastMathF16/Angles1_f16.txt").f16Bits();
	REQUIRE(bits.size() == 23);
	for (size_t i = 0; i < bits.size(); i++) {
		float16_t h;
		static_assert(sizeof h == sizeof bits[i], "size");
		std::memcpy(&h, &bits[i], sizeof h);
		CHECK((float)h == dsp::f16ToFloat(bits[i]));
	}
}

}
