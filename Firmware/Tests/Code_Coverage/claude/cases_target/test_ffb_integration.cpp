/*
 * End to end tests of the torque path: PID reports from the simulated host (PidHost) -> HidFFB -> real
 * EffectsCalculator -> real Axis -> MockMotorDriver torque, with the position and speed coming from MockEncoder
 * through the real Axis metrics.
 *
 * Setup of every test (IntegrationFixture): power 0x7fff and effect ratio 255 (torque scaler and effect ratio 1.0,
 * see AxisFixture::setFullScale), internal axis damper 0 (default 30, it would add a speed dependent torque), and
 * 1100 FFB cycles at the center before the first PID report so the fade-in after the driver became ready is over.
 * Axes are inverted by default (Axis.h invertAxis = true). The direction enable bit is 0x04 (2 axis descriptor) like
 * FFBWheel::usbInit sets it; HidFFB's own 1 axis default is 0x02.
 *
 * Two kinds of expectations:
 * - PID / DirectInput semantics, independent of the firmware tuning: signs (a constant force and a spring at the same
 *   position push the same way, conditions oppose position / motion / acceleration, inversion does not change the
 *   physical direction of conditions), linearity, ratios (gains, power, effect ratio), timing (duration, start delay,
 *   period, ramp). The absolute sign of the constant force itself comes from the firmware (EffectsCalculator negates
 *   it); its consistency with the spring is checked by "a positive constant force in direction 90 degrees pushes the
 *   same way as a spring ...".
 * - Absolute magnitudes of conditions and endstop use the firmware tuning constants copied from EffectsCalculator.h
 *   (gains 64/64/254/127, scalers 16/4/1/2, INTERNAL_SCALER_*) and Axis.h (endstop gain 25, strength 127, idle spring
 *   127 * 35). They protect against unintended changes of the tuning, not against the PID specification.
 * Values are compared with tolerances (float arithmetic, CMSIS-DSP vs libm sin/cos).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "FFBWheel.h"

#include <cmath>
#include <vector>

namespace {

struct IntegrationFixture : AxisFixture {
	explicit IntegrationFixture(uint8_t axisCount = 1) : AxisFixture(axisCount) {
		for (uint8_t i = 0; i < axisCount; i++) {
			setFullScale(i);
			axisCommand(Axis_commands::axisdamper, CMDtype::set, 0, nullptr, 0, i);
		}
		run(1100); // fade-in over, FFB still off, all axes centered: torque 0
	}

	/** Sets a command value of axis i */
	void setAxis(Axis_commands cmd, int64_t val, uint8_t i = 0) { axisCommand(cmd, CMDtype::set, val, nullptr, 0, i); }

	/** Allocates, configures (direction 90 deg) and starts a constant force */
	uint8_t constantForce(int16_t magnitude, const PidHost::EffectParams &params = PidHost::EffectParams()) {
		uint8_t index = startEffect(FFB_EFFECT_CONSTANT, params);
		host.setConstantForce(index, magnitude);
		return index;
	}

	/**
	 * Condition effect like DirectInput sends it: Create New Effect, Set Condition block(s), Set Effect, Start.
	 * With one block the effect direction (90 deg, X) applies; with a block per axis both axes are enabled.
	 */
	uint8_t conditionEffect(uint8_t type, int16_t coefficient, uint16_t saturation = 0x7fff, int16_t coefficientY = 0, bool blockPerAxis = false) {
		uint8_t index = host.allocateEffect(type);
		host.setCondition(index, 0, 0, coefficient, coefficient, saturation, saturation, 0);
		PidHost::EffectParams params;
		if (blockPerAxis) {
			host.setCondition(index, 1, 0, coefficientY, coefficientY, saturation, saturation, 0);
			params.enableAxis = PidHost::AXIS_X | PidHost::AXIS_Y | PidHost::DIRECTION_ENABLE_2AXIS;
		}
		host.setEffect(index, type, params);
		host.startEffect(index);
		return index;
	}

	/** Encoder resolution of 0.0001 deg per count so the motion profiles below are not quantised */
	void fineEncoder(uint8_t i = 0) { encoder(i).setCpr(3600000); }
	static int32_t fineCounts(double degrees) { return (int32_t)std::lround(degrees * 10000.0); }

	/** Moves the encoder of axis i at a constant speed (deg/s) for `cycles` FFB cycles starting at `startDeg` */
	void moveAtSpeed(double startDeg, double degPerS, uint32_t cycles, uint8_t i = 0) {
		for (uint32_t n = 1; n <= cycles; n++) {
			encoder(i).setRawPosition(fineCounts(startDeg + degPerS * n / 1000.0));
			run(1);
		}
	}
};

struct TwoAxisIntegrationFixture : IntegrationFixture {
	TwoAxisIntegrationFixture() : IntegrationFixture(2) {}
};

/** Expected spring force before the axis (EffectsCalculator::calcConditionEffectForce, gain 64, scaler 16) */
double springForce(double coefficient, double posScaled) { return coefficient / 0x7fff * (65.0 / 256.0) * 16.0 * posScaled; }

} // namespace

TEST_SUITE("FFBIntegration") {

// ---------------------------------------------------------------------------
// Constant force: sign, direction, scaling, clipping
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(IntegrationFixture, "the setup leaves the axis centered without torque and FFB off") {
	CHECK(driver().lastTorque == 0);
	CHECK_FALSE(ffb.getFfbActive());
}

TEST_CASE_FIXTURE(IntegrationFixture, "a positive constant force in direction 90 degrees reaches the driver as positive torque with the default inversion") {
	constantForce(10000);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(10000).epsilon(0.0005));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a positive constant force reaches the driver as negative torque when the axis is not inverted") {
	setAxis(Axis_commands::invert, 0);
	constantForce(10000);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(-10000).epsilon(0.0005));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a negative constant force reaches the driver with the opposite sign of a positive one") {
	constantForce(-10000);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(-10000).epsilon(0.0005));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a constant force in direction 270 degrees reverses the torque") {
	PidHost::EffectParams params;
	params.directionX = 27000;
	constantForce(10000, params);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(-10000).epsilon(0.0005));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a new constant force magnitude reaches the driver in the next cycle") {
	uint8_t index = constantForce(10000);
	run(1);
	host.setConstantForce(index, 3000);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(3000).epsilon(0.001));
}

TEST_CASE_FIXTURE(IntegrationFixture, "the torque is scaled by the power of the axis") {
	setAxis(Axis_commands::power, 16384);
	constantForce(10000);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(10000.0 * 16384.0 / 32767.0).epsilon(0.001));
	CHECK(driver().powerLimit == 16384);
}

TEST_CASE_FIXTURE(IntegrationFixture, "the torque is scaled by the effect ratio of the axis") {
	setAxis(Axis_commands::fxratio, 128);
	constantForce(10000);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(10000.0 * 128.0 / 255.0).epsilon(0.001));
}

TEST_CASE_FIXTURE(IntegrationFixture, "the sum of two constant forces above full scale is clipped to the power") {
	setAxis(Axis_commands::power, 20000);
	constantForce(30000);
	constantForce(30000);
	run(1);
	CHECK(driver().lastTorque == 20000); // 60000 * 20000 / 32767 = 36622 before clipping
}

TEST_CASE_FIXTURE(IntegrationFixture, "two constant forces in opposite directions cancel each other") {
	constantForce(8000);
	constantForce(-8000);
	run(1);
	CHECK(std::abs(driver().lastTorque) <= 1);
}

TEST_CASE_FIXTURE(IntegrationFixture, "the gain of the effect scales its torque") {
	PidHost::EffectParams params;
	params.gain = 128;
	constantForce(10000, params);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(10000.0 * 128 / 255).epsilon(0.001));
}

TEST_CASE_FIXTURE(IntegrationFixture, "the device gain scales the torque of all effects") {
	constantForce(10000);
	host.deviceGain(128);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(10000.0 * 128 / 255).epsilon(0.001));
}

TEST_CASE_FIXTURE(IntegrationFixture, "the device gain and the effect gain multiply") {
	PidHost::EffectParams params;
	params.gain = 128;
	constantForce(10000, params);
	host.deviceGain(128);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(10000.0 * 128 / 255 * 128 / 255).epsilon(0.002));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a direction enable bit that matches the 1 axis descriptor gives the full force") {
	ffb.setDirectionEnableMask(PidHost::DIRECTION_ENABLE_1AXIS); // HidFFB default for 1 axis
	PidHost::EffectParams params;
	params.enableAxis = PidHost::AXIS_X | PidHost::DIRECTION_ENABLE_1AXIS;
	constantForce(10000, params);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(10000).epsilon(0.0005));
}

// ---------------------------------------------------------------------------
// Effect operation
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(IntegrationFixture, "a stopped effect no longer produces torque") {
	uint8_t index = constantForce(10000);
	run(1);
	REQUIRE(driver().lastTorque != 0);

	host.stopEffect(index);
	run(1);

	CHECK(driver().lastTorque == 0);
	CHECK(ffb.getFfbActive()); // only the effect was stopped
}

TEST_CASE_FIXTURE(IntegrationFixture, "a stopped effect produces torque again when it is restarted") {
	uint8_t index = constantForce(10000);
	host.stopEffect(index);
	run(1);
	REQUIRE(driver().lastTorque == 0);

	host.startEffect(index);
	run(1);

	CHECK(driver().lastTorque == doctest::Approx(10000).epsilon(0.0005));
}

TEST_CASE_FIXTURE(IntegrationFixture, "an allocated but not started effect produces no torque") {
	uint8_t index = host.allocateEffect(FFB_EFFECT_CONSTANT);
	host.setEffect(index, FFB_EFFECT_CONSTANT);
	host.setConstantForce(index, 10000);
	run(1);
	CHECK(driver().lastTorque == 0);
}

TEST_CASE_FIXTURE(IntegrationFixture, "starting an effect solo stops all other effects") {
	constantForce(10000);
	uint8_t second = constantForce(3000);
	run(1);
	REQUIRE(driver().lastTorque == doctest::Approx(13000).epsilon(0.001));

	host.effectOperation(second, PidHost::OP_START_SOLO);
	run(1);

	CHECK(driver().lastTorque == doctest::Approx(3000).epsilon(0.001));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a freed effect no longer produces torque") {
	uint8_t index = constantForce(10000);
	run(1);
	host.blockFree(index);
	run(1);
	CHECK(driver().lastTorque == 0);
}

TEST_CASE_FIXTURE(IntegrationFixture, "an effect with a duration stops producing torque after the duration") {
	PidHost::EffectParams params;
	params.duration = 100;
	constantForce(10000, params);
	run(100); // still on 100 ms after the start (expires when more than the duration has elapsed)
	CHECK(driver().lastTorque == doctest::Approx(10000).epsilon(0.0005));
	run(1); // 101 ms
	CHECK(driver().lastTorque == 0);
}

TEST_CASE_FIXTURE(IntegrationFixture, "an effect with a duration and a start delay produces torque only after the delay") {
	// Infinite effects ignore the start delay: existing KNOWN_ISSUE "start delay postpones an infinite effect"
	// (cases/test_effects_calculator.cpp)
	PidHost::EffectParams params;
	params.duration = 1000;
	params.startDelay = 50;
	constantForce(10000, params);
	run(49);
	CHECK(driver().lastTorque == 0);
	run(1); // exactly 50 ms
	CHECK(driver().lastTorque == doctest::Approx(10000).epsilon(0.0005));
}

// ---------------------------------------------------------------------------
// Device control and gain
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(IntegrationFixture, "stop all effects removes the effect torque") {
	// Whether FFB stays enabled is open: baseline KNOWN_ISSUE "stop all effects stops every running effect"
	constantForce(10000);
	run(1);
	host.deviceControl(PidHost::DC_STOP_ALL_EFFECTS);
	run(1);
	CHECK(driver().lastTorque == 0); // centered: no idle spring torque either
}

TEST_CASE_FIXTURE(IntegrationFixture, "pause removes the effect torque and continue restores it") {
	constantForce(10000);
	run(1);

	host.deviceControl(PidHost::DC_DEVICE_PAUSE);
	run(1);
	CHECK(driver().lastTorque == 0);

	host.deviceControl(PidHost::DC_DEVICE_CONTINUE);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(10000).epsilon(0.0005));
}

TEST_CASE_FIXTURE(IntegrationFixture, "disable actuators removes the effect torque and enable actuators restores it") {
	constantForce(10000);
	run(1);

	host.deviceControl(PidHost::DC_DISABLE_ACTUATORS);
	run(1);
	CHECK(driver().lastTorque == 0);

	host.deviceControl(PidHost::DC_ENABLE_ACTUATORS);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(10000).epsilon(0.0005));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a device reset deletes the effects so continue gives no torque") {
	constantForce(10000);
	run(1);

	host.deviceControl(PidHost::DC_DEVICE_RESET);
	host.deviceControl(PidHost::DC_DEVICE_CONTINUE);
	run(1);

	CHECK(ffb.getFfbActive());
	CHECK(driver().lastTorque == 0);
}

TEST_CASE_FIXTURE(IntegrationFixture, "while FFB is off the idle spring of the axis replaces the effects") {
	encoder().setDegrees(90.0f);
	constantForce(10000);
	host.deviceControl(PidHost::DC_DISABLE_ACTUATORS);
	run(1);
	// Idle spring strength 127: clip 127 * 35 = 4445, restoring (encoder positive -> negative torque)
	CHECK(driver().lastTorque == -4445);
}

TEST_CASE_FIXTURE(IntegrationFixture, "the idle spring is off while FFB is on") {
	encoder().setDegrees(90.0f);
	uint8_t index = constantForce(10000);
	host.stopEffect(index); // FFB stays on without effects
	run(1);
	CHECK(driver().lastTorque == 0);
}

// ---------------------------------------------------------------------------
// Periodic and ramp effects (time from the simulated clock)
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(IntegrationFixture, "a sine reaches its positive and negative peak at a quarter and three quarters of the period") {
	uint8_t index = startEffect(FFB_EFFECT_SINE);
	host.setPeriodic(index, 10000, 0, 0, 1000);

	run(250);
	CHECK(driver().lastTorque == doctest::Approx(10000).epsilon(0.002));
	run(500);
	CHECK(driver().lastTorque == doctest::Approx(-10000).epsilon(0.002));
}

TEST_CASE_FIXTURE(IntegrationFixture, "the offset of a periodic effect shifts the torque") {
	uint8_t index = startEffect(FFB_EFFECT_SINE);
	host.setPeriodic(index, 5000, 2000, 0, 1000);

	run(500); // sine at zero crossing
	CHECK(driver().lastTorque == doctest::Approx(2000).epsilon(0.01));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a square wave flips its sign between the two halves of the period") {
	// Which half comes first: baseline characterization "square starts with the negative half"
	uint8_t index = startEffect(FFB_EFFECT_SQUARE);
	host.setPeriodic(index, 10000, 0, 0, 1000);

	run(100); // first half
	int32_t first = driver().lastTorque;
	run(500); // second half
	int32_t second = driver().lastTorque;
	CHECK(std::abs(first) == doctest::Approx(10000).epsilon(0.0005));
	CHECK(second == doctest::Approx(-first).epsilon(0.0005));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a ramp moves the torque linearly from the start to the end level over the duration") {
	PidHost::EffectParams params;
	params.duration = 1000;
	uint8_t index = startEffect(FFB_EFFECT_RAMP, params);
	host.setRamp(index, -10000, 10000);

	run(250);
	CHECK(driver().lastTorque == doctest::Approx(-5000).epsilon(0.005));
	run(500);
	CHECK(driver().lastTorque == doctest::Approx(5000).epsilon(0.005));
	run(300); // expired
	CHECK(driver().lastTorque == 0);
}

// ---------------------------------------------------------------------------
// Condition effects: metrics from the encoder through the real axis
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(IntegrationFixture, "a spring pushes the axis back to the center with the default inversion") {
	encoder().setDegrees(90.0f); // pos_scaled_16b = -6553 (inverted)
	conditionEffect(FFB_EFFECT_SPRING, 10000);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(-springForce(10000, 6553)).epsilon(0.001));
	CHECK(driver().lastTorque < 0);
}

TEST_CASE_FIXTURE(IntegrationFixture, "a spring pushes the axis back to the center when the axis is not inverted") {
	setAxis(Axis_commands::invert, 0);
	encoder().setDegrees(90.0f);
	conditionEffect(FFB_EFFECT_SPRING, 10000);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(-springForce(10000, 6553)).epsilon(0.001));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a spring pushes back from the other side with the opposite torque") {
	encoder().setDegrees(-90.0f);
	conditionEffect(FFB_EFFECT_SPRING, 10000);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(springForce(10000, 6553)).epsilon(0.001));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a positive constant force in direction 90 degrees pushes the same way as a spring at a position on the positive X side") {
	// DirectInput: a positive force in direction 90 deg (east) pushes towards -X, like a centering spring at +X
	for (int inverted : {1, 0}) {
		CAPTURE(inverted);
		setAxis(Axis_commands::invert, inverted);
		encoder().setDegrees(inverted ? -90.0f : 90.0f); // +X as seen by the game (HID axis value)
		run(1);
		REQUIRE(axis().getLastScaledEnc() > 0);

		uint8_t spring = conditionEffect(FFB_EFFECT_SPRING, 10000);
		run(1);
		int32_t springTorque = driver().lastTorque;
		host.blockFree(spring);

		uint8_t cf = constantForce(10000);
		run(1);
		int32_t constantTorque = driver().lastTorque;
		host.blockFree(cf);

		REQUIRE(springTorque != 0);
		CHECK((springTorque > 0) == (constantTorque > 0));
		CHECK(constantTorque != 0);
	}
}

TEST_CASE_FIXTURE(IntegrationFixture, "a spring torque is limited by its saturation") {
	encoder().setDegrees(90.0f);
	conditionEffect(FFB_EFFECT_SPRING, 10000, 3000);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(-3000).epsilon(0.001));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a spring follows the encoder position") {
	conditionEffect(FFB_EFFECT_SPRING, 10000);
	encoder().setDegrees(45.0f);
	run(1);
	int32_t at45 = driver().lastTorque;
	encoder().setDegrees(90.0f);
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(2.0 * at45).epsilon(0.002));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a damper opposes the motion of the axis") {
	fineEncoder();
	conditionEffect(FFB_EFFECT_DAMPER, 0x7fff);
	moveAtSpeed(-15.0, 100.0, 300); // +100 deg/s

	// speed -100 deg/s (inverted) * 40 * gain (65/256) * scaler 4 = -4062.5; the effect opposes it, axis inverts
	CHECK(driver().lastTorque == doctest::Approx(-4062).epsilon(0.01));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a damper opposes the motion of the axis when the axis is not inverted") {
	setAxis(Axis_commands::invert, 0);
	fineEncoder();
	conditionEffect(FFB_EFFECT_DAMPER, 0x7fff);
	moveAtSpeed(-15.0, 100.0, 300);
	CHECK(driver().lastTorque == doctest::Approx(-4062).epsilon(0.01));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a damper gives no torque while the axis does not move") {
	encoder().setDegrees(90.0f);
	run(200); // speed filter settled
	conditionEffect(FFB_EFFECT_DAMPER, 0x7fff);
	run(100);
	CHECK(std::abs(driver().lastTorque) <= 1);
}

TEST_CASE_FIXTURE(IntegrationFixture, "a friction opposes the motion of the axis with its coefficient above the ramp-up speed") {
	fineEncoder();
	conditionEffect(FFB_EFFECT_FRICTION, 10000);
	moveAtSpeed(-45.0, 300.0, 300); // speed * 45 = 13500, above the ramp-up range of 25 % of 32767

	// ((254 + 1) * -10000) >> 8 = -9961, opposed, inverted
	CHECK(driver().lastTorque == doctest::Approx(-9961).epsilon(0.005));
}

TEST_CASE_FIXTURE(IntegrationFixture, "a friction reverses its torque with the direction of motion") {
	fineEncoder();
	conditionEffect(FFB_EFFECT_FRICTION, 10000);
	moveAtSpeed(45.0, -300.0, 300);
	CHECK(driver().lastTorque == doctest::Approx(9961).epsilon(0.005));
}

TEST_CASE_FIXTURE(IntegrationFixture, "an inertia opposes the acceleration of the axis") {
	encoder().setCpr(36000000); // 0.00001 deg per count
	conditionEffect(FFB_EFFECT_INERTIA, 0x7fff);
	const double accel = 2000.0; // deg/s^2
	for (uint32_t n = 1; n <= 350; n++) {
		double t = n / 1000.0;
		encoder().setRawPosition((int32_t)std::lround(0.5 * accel * t * t * 100000.0));
		run(1);
	}

	// accel -2000 (inverted) * 4 * gain (128/256) * scaler 2 = -8000; opposed, inverted
	CHECK(driver().lastTorque == doctest::Approx(-8000).epsilon(0.02));
}

TEST_CASE_FIXTURE(TwoAxisIntegrationFixture, "a spring with a condition block per axis acts on each axis with its own coefficient") {
	encoder(0).setDegrees(90.0f);
	encoder(1).setDegrees(90.0f);
	conditionEffect(FFB_EFFECT_SPRING, 10000, 0x7fff, 5000, true);
	run(1);
	CHECK(driver(0).lastTorque == doctest::Approx(-springForce(10000, 6553)).epsilon(0.001));
	CHECK(driver(1).lastTorque == doctest::Approx(-springForce(5000, 6553)).epsilon(0.002));
}

TEST_CASE_FIXTURE(TwoAxisIntegrationFixture, "a spring with a single condition block in direction 90 degrees acts only on the X axis") {
	encoder(0).setDegrees(90.0f);
	encoder(1).setDegrees(90.0f);
	conditionEffect(FFB_EFFECT_SPRING, 10000);
	run(1);
	CHECK(driver(0).lastTorque == doctest::Approx(-springForce(10000, 6553)).epsilon(0.001));
	CHECK(std::abs(driver(1).lastTorque) <= 1);
}

TEST_CASE_FIXTURE(TwoAxisIntegrationFixture, "a damper with a condition block per axis acts on the moving axis only") {
	fineEncoder(0);
	fineEncoder(1);
	conditionEffect(FFB_EFFECT_DAMPER, 0x7fff, 0x7fff, 0x7fff, true);
	moveAtSpeed(-15.0, 100.0, 300, 1); // only Y moves
	CHECK(std::abs(driver(0).lastTorque) <= 1);
	CHECK(driver(1).lastTorque == doctest::Approx(-4062).epsilon(0.01));
}

// ---------------------------------------------------------------------------
// Two axes: directions
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(TwoAxisIntegrationFixture, "a constant force in direction 0 degrees acts only on the Y axis") {
	PidHost::EffectParams params;
	params.enableAxis = PidHost::AXIS_X | PidHost::AXIS_Y | PidHost::DIRECTION_ENABLE_2AXIS;
	params.directionX = 0; // north: X magnitude sin(0) = 0, Y magnitude -cos(0) = -1
	constantForce(10000, params);
	run(1);
	CHECK(std::abs(driver(0).lastTorque) <= 1);
	CHECK(driver(1).lastTorque == doctest::Approx(-10000).epsilon(0.0005));
}

TEST_CASE_FIXTURE(TwoAxisIntegrationFixture, "a constant force in direction 90 degrees acts only on the X axis") {
	constantForce(10000);
	run(1);
	CHECK(driver(0).lastTorque == doctest::Approx(10000).epsilon(0.0005));
	CHECK(std::abs(driver(1).lastTorque) <= 1);
}

TEST_CASE_FIXTURE(TwoAxisIntegrationFixture, "the device gain applies to both axes") {
	PidHost::EffectParams params;
	params.enableAxis = PidHost::AXIS_X | PidHost::AXIS_Y | PidHost::DIRECTION_ENABLE_2AXIS;
	params.directionX = 13500;
	constantForce(10000, params);
	host.deviceGain(128);
	run(1);
	double expected = 10000.0 * std::sin(M_PI * 0.75) * 128.0 / 255.0;
	CHECK(driver(0).lastTorque == doctest::Approx(expected).epsilon(0.003));
	CHECK(driver(1).lastTorque == doctest::Approx(expected).epsilon(0.003));
}

// ---------------------------------------------------------------------------
// Endstop and effects
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(IntegrationFixture, "beyond the range the endstop pushes back even with the effect ratio 0") {
	setAxis(Axis_commands::fxratio, 0);
	constantForce(10000);
	fineEncoder();
	encoder().setRawPosition(fineCounts(455.0)); // 5 deg beyond the 900 deg range
	run(1);
	// (455 - 450) * endstop strength 127 * gain 25 = 15875 against the position, inverted
	CHECK(driver().lastTorque == doctest::Approx(-15875).epsilon(0.002));
}

TEST_CASE_FIXTURE(IntegrationFixture, "the effect ratio scales the effects but not the endstop") {
	setAxis(Axis_commands::fxratio, 128);
	constantForce(10000); // pushes towards the positive end
	fineEncoder();
	encoder().setRawPosition(fineCounts(455.0));
	run(1);
	CHECK(driver().lastTorque == doctest::Approx(-15875 + 10000.0 * 128 / 255).epsilon(0.002));
}

TEST_CASE_FIXTURE(IntegrationFixture, "the endstop torque is at most the power") {
	setAxis(Axis_commands::power, 10000);
	fineEncoder();
	encoder().setRawPosition(fineCounts(-500.0)); // 50 deg beyond the range: far more than full scale
	run(1);
	CHECK(driver().lastTorque <= 10000);
	CHECK(driver().lastTorque == doctest::Approx(10000).epsilon(0.001));
}

TEST_CASE_FIXTURE(IntegrationFixture, "within the range there is no endstop torque") {
	fineEncoder();
	encoder().setRawPosition(fineCounts(449.0));
	setAxis(Axis_commands::idlespring, 0);
	run(1);
	CHECK(driver().lastTorque == 0);
}

// ---------------------------------------------------------------------------
// Emergency stop in the torque path
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(IntegrationFixture, "no effect torque reaches the driver during an emergency stop") {
	constantForce(10000);
	run(1);
	axis().emergencyStop(false);
	driver().torqueHistory.clear();
	run(10);
	CHECK(driver().torqueHistory.empty());
	CHECK(driver().lastTorque == 0);
}

// ---------------------------------------------------------------------------
// Through a real main class
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(TargetFixture, "a constant force sent to an FFBWheel reaches the motor driver of its axis" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	wheel.usbInit(); // sets the direction enable bit of the 2 axis descriptor
	Axis *axis = dynamic_cast<Axis *>(CommandHandler::getHandlerFromClassName("axis", 0));
	REQUIRE(axis != nullptr);
	axis->driverChooser = ClassChooser<MotorDriver>(AxisFixture::driverRegistry());
	axis->encoderChooser = ClassChooser<Encoder>(AxisFixture::encoderRegistry());
	axis->setDrvType(MockMotorDriver::SELECTION_ID);
	axis->setEncType(MockEncoder::SELECTION_ID);
	MockMotorDriver *drv = dynamic_cast<MockMotorDriver *>(axis->getDriver());
	REQUIRE(drv != nullptr);
	std::vector<CommandReply> replies;
	ParsedCommand cmd;
	cmd.target = axis;
	cmd.type = CMDtype::set;
	cmd.cmdId = (uint32_t)Axis_commands::power;
	cmd.val = 0x7fff;
	axis->command(cmd, replies);
	cmd.cmdId = (uint32_t)Axis_commands::fxratio;
	cmd.val = 255;
	axis->command(cmd, replies);

	PidHost host;
	uint8_t index = host.allocateEffect(FFB_EFFECT_CONSTANT);
	REQUIRE(index != 0);
	host.setEffect(index, FFB_EFFECT_CONSTANT);
	host.startEffect(index);
	host.setConstantForce(index, 10000);

	HostRtos::runUntilBlocked(wheel); // starts TIM_FFB
	for (int i = 0; i < 1100; i++) {
		HostPlatform::advanceMs(1);
		REQUIRE(HostTarget::timerUpdate(&htim13));
		HostRtos::runUntilBlocked(wheel);
	}

	CHECK(drv->lastTorque == doctest::Approx(10000).epsilon(0.0005));
}

} // TEST_SUITE
