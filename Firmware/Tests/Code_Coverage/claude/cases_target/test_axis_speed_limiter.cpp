/*
 * Speed limiter of the real Axis (Axis.cpp:919-967 applySpeedLimiterTorque) in the CMSIS-DSP build (the shipped
 * targets): integral windup of the arm_pid_f32 controller. The basic limiter behaviour (direction, inactive below the
 * limit, PI gains, restart of the integral) is in test_axis_torque.cpp; NEEDS REVIEW 32 covers the different scaling
 * of the two builds.
 *
 * With USE_DSP_FUNCTIONS the reduction is reductionAmount = arm_pid_f32(&speedLimiterPID, speedError) (Axis.cpp:945):
 * an incremental PI y[n] = y[n-1] + (Kp + Ki) e[n] - Kp e[n-1] with no output limit (controller_functions.h:287-303,
 * documented CMSIS behaviour). Axis clips only the returned value to [0, |torque|] (Axis.cpp:950); state[2] keeps the
 * unclipped output. During a long over-speed the stored output grows far beyond the torque. While the speed stays
 * above the limit the stored output falls only when the error falls (by Kp (e[n-1] - e[n]) - Ki e[n] per cycle,
 * once), so the torque stays fully cut until the reset below the limit (Axis.cpp:961-962): bang-bang at the limit.
 *
 * Without CMSIS-DSP (Axis.cpp:947-949) the I term is clipped to [0, power] (power 0x7fff here), so it also cannot
 * fall while the error is positive, but it is bounded by the power setting; this file is empty in DSP-off builds.
 *
 * Fixture: AxisFixture (real Axis, MockMotorDriver / MockEncoder), driven cycle by cycle like
 * EffectsCalculator::calculateEffects + AxesManager::updateTorque (as TorqueFixture in test_axis_torque.cpp).
 * Host vs target: float math, compared with tolerances.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#ifdef USE_DSP_FUNCTIONS

#include <cmath>

namespace {

constexpr uint32_t CPR = 1u << 24; //!< High resolution so that the speed is not quantised

struct SpeedLimiterFixture : AxisFixture {
	SpeedLimiterFixture() {
		setFullScale();
		axisCommand(Axis_commands::invert, CMDtype::set, 0);
		axisCommand(Axis_commands::axisdamper, CMDtype::set, 0);
		encoder().setCpr(CPR);
		axisCommand(Axis_commands::degrees, CMDtype::set, 7200); // far from the endstop while moving
		cycle(0);
		for (int i = 0; i < 1100; i++) cycle(0); // past the 1 s fade-in
	}

	void cycle(int32_t game) {
		HostPlatform::advanceMs(1);
		Axis &a = axis();
		a.prepareForUpdate();
		a.calculateMechanicalEffects(true);
		a.setFfbEffectTorque(game);
		if (!control.emergency) {
			a.updateDriveTorque();
		}
	}

	/** Moves the encoder at a constant speed (deg/s) for n 1 ms cycles with the game torque */
	void moveAt(double degPerS, uint32_t n, int32_t game) {
		for (uint32_t i = 0; i < n; i++) {
			posDeg += degPerS / 1000.0;
			encoder().setRawPosition(static_cast<int32_t>(std::lround(posDeg * CPR / 360.0)));
			cycle(game);
		}
	}

	int32_t torque() { return driver().lastTorque; }

	double posDeg = 0;
};

} // namespace

TEST_SUITE("AxisSpeedLimiter") {

// NEEDS REVIEW 158: arm_pid_f32 has no output limit or anti-windup and Axis.cpp:945-950 clips only the returned
// reduction, so state[2] keeps the unclipped output: after 1.5 s at 500 deg/s (limit 100) the stored reduction is
// about 0.33*400 + 1499*0.03*400 = 18,120 for a torque of 10,000. When the speed then falls to 110 deg/s (10 deg/s
// over the limit) the torque stays 0, whereas a limiter entering at 110 deg/s would cut only about 3 + 0.3 per
// cycle. Intended: clamp the controller output / state to the applied reduction (anti-windup), or is the full cut
// until the speed is below the limit (bang-bang) intended?
TEST_CASE_FIXTURE(SpeedLimiterFixture,
				  "characterization: after a long over-speed the speed limiter keeps the torque cut just above the "
				  "limit (integral windup, NEEDS REVIEW 158)") {
	moveAt(500, 300, 10000); // speed estimate settled before the limiter is switched on
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 100);
	moveAt(500, 1500, 10000);
	REQUIRE(torque() == 0); // fully cut during the over-speed
	moveAt(110, 100, 10000);
	CHECK(torque() == 0);
}

TEST_CASE_FIXTURE(SpeedLimiterFixture, "a speed limiter that starts just above the limit cuts only a little torque") {
	// reference for the case above: the same 100 cycles at 110 deg/s without the preceding over-speed
	moveAt(110, 300, 10000);
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 100);
	moveAt(110, 100, 10000);
	CHECK_NEAR(torque(), 10000 - (0.33 * 10 + 99 * 0.03 * 10), 3);
}

TEST_CASE_FIXTURE(SpeedLimiterFixture, "the speed limiter gives the full torque back once the speed is below the limit") {
	moveAt(500, 300, 10000);
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 100);
	moveAt(500, 1500, 10000);
	moveAt(50, 300, 10000);
	CHECK_NEAR(torque(), 10000, 1);
}

} // TEST_SUITE

#endif // USE_DSP_FUNCTIONS
