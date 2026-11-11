#include "doctest.h"
#include "../support/NumericAssertions.h"
#ifdef USE_DSP_FUNCTIONS
#include "arm_math.h"
#include <array>
#include <limits>

TEST_SUITE("CodexPidCloseout") {
// Pure public CMSIS functions, local state only. Execution covers portable C on
// the host; Cortex-M intrinsic/FPU rounding and timing remain unverified.
TEST_CASE("public PID follows the positional equation for a signed dyadic history") {
	arm_pid_instance_f32 pid{};
	constexpr double kp = 0.5, ki = 0.125, kd = 0.25;
	pid.Kp = kp;
	pid.Ki = ki;
	pid.Kd = kd;
	arm_pid_init_f32(&pid, 1);
	constexpr std::array<float, 9> errors{1.0f, -2.0f, 0.5f, 3.0f, -1.5f, 0.0f, -4.0f, 0.25f, 2.0f};
	double sum = 0.0, previous = 0.0;
	bool positive = false, negative = false;
	for (size_t sample = 0; sample < errors.size(); ++sample) {
		const double e = errors[sample];
		sum += e;
		// Independent positional equation, no CMSIS coefficients or state read.
		const double reference = kp * e + ki * sum + kd * (e - previous);
		previous = e;
		const float actual = arm_pid_f32(&pid, errors[sample]);
		INFO("sample=" << sample << " actual=" << actual << " reference=" << reference);
		// All inputs and gains are dyadic and bounded: exact binary arithmetic
		// is possible here; a small absolute allowance also permits rounding.
		CHECK(CodexNumeric::finiteWithin(actual, reference, 1e-6));
		CHECK_FALSE(CodexNumeric::finiteWithin(actual + 0.25, reference, 1e-6));
		CHECK_FALSE(CodexNumeric::finiteWithin(std::numeric_limits<double>::quiet_NaN(), reference, 1e-6));
		positive = positive || actual > 0.0f;
		negative = negative || actual < 0.0f;
	}
	CHECK(positive);
	CHECK(negative);
}
}
#endif
