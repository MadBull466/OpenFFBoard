// CDX-014: finite results must be checked sample by sample, before any error reduction.
// Public CMSIS APIs only. Trigonometry is stateless; the biquad fixture owns/reset its state.
// Host portable C and target FPU rounding may differ; comparisons allow float roundoff.
#include "doctest.h"

#ifdef USE_DSP_FUNCTIONS
#include "arm_math.h"

#include <array>
#include <cmath>

namespace {
struct StableBiquadFixture {
	// CMSIS adds a1*y[n-1] + a2*y[n-2]. Poles of z²-.25z-.125 are .5 and -.25.
	std::array<float, 5> coefficients{0.5f, -0.125f, 0.0625f, 0.25f, 0.125f};
	std::array<float, 4> state{};
	arm_biquad_casd_df1_inst_f32 filter{};

	StableBiquadFixture() {
		arm_biquad_cascade_df1_init_f32(&filter, 1, coefficients.data(), state.data());
	}
};
}

TEST_SUITE("CodexDspFiniteOutputs") {
TEST_CASE("sine and cosine of finite small angles return finite approximations") {
	for (unsigned i = 0; i < 32; ++i) {
		const float angle = static_cast<float>(0.2 + 0.01 * i / 31.0);
		CAPTURE(i);
		CAPTURE(angle);
		const float sine = arm_sin_f32(angle);
		const float cosine = arm_cos_f32(angle);
		REQUIRE(std::isfinite(sine));
		REQUIRE(std::isfinite(cosine));
		// 512-interval linear interpolation: h²/8 < 1.89e-5; margin for float rounding.
		CHECK(std::fabs(static_cast<double>(sine) - std::sin(static_cast<double>(angle))) <= 2.1e-5);
		CHECK(std::fabs(static_cast<double>(cosine) - std::cos(static_cast<double>(angle))) <= 2.1e-5);
	}
}

TEST_CASE_FIXTURE(StableBiquadFixture, "a stable biquad returns finite samples across successive blocks") {
	const std::array<float, 26> input{
		.75f, -.25f, .125f, -.875f, .5f, .0625f, -.375f, .9375f, -.125f,
		.3125f, -.6875f, .1875f, .5625f, -.4375f, .8125f, -.0625f, .25f,
		-.75f, .375f, -.1875f, .6875f, -.3125f, .125f, -.5625f, .875f, -.5f};
	std::array<float, 26> output{};
	arm_biquad_cascade_df1_f32(&filter, input.data(), output.data(), 13);
	arm_biquad_cascade_df1_f32(&filter, input.data() + 13, output.data() + 13, 13);
	double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
	for (unsigned i = 0; i < output.size(); ++i) {
		CAPTURE(i);
		REQUIRE(std::isfinite(output[i]));
		// Independent DF1 equation, zero history. With |x|<=1, |y|<=.6875/(1-.375)=1.1.
		const double x = input[i];
		const double expected = .5*x - .125*x1 + .0625*x2 + .25*y1 + .125*y2;
		CHECK(std::fabs(static_cast<double>(output[i]) - expected) <= 2e-6);
		x2 = x1;
		x1 = x;
		y2 = y1;
		y1 = expected;
	}
}
}
#endif
