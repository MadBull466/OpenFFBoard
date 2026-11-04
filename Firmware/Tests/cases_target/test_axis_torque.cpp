/*
 * Torque path of the real Axis (Axis.cpp): Axis::prepareForUpdate, calculateMechanicalEffects, updateTorque /
 * updateDriveTorque and everything they combine: game torque (equalizer, expo, effect ratio), internal axis effects
 * (idle spring, damper, friction, inertia), endstop, power scaling, out of range shutdown, fade-in, speed limiter,
 * inversion, slew rate limiter, clipping, emergency stop, USB suspend / resume, hands-off detection and the
 * position / speed / acceleration metrics.
 *
 * Most cases drive one axis cycle by cycle the way EffectsCalculator::calculateEffects and
 * AxesManager::updateTorque do (TorqueFixture::cycle), so the game torque can be set exactly. Configuration is set
 * with the axis commands (tested in test_axis_config.cpp).
 *
 * Order of operations in Axis::updateTorque (Axis.cpp:846-905):
 *   game torque -> equalizer -> expo -> * effect ratio, + internal effects, + endstop (clipped to +-0x7fff),
 *   * power / 0x7fff, 0 when out of range, * fade-in multiplier, - speed limiter, inversion,
 *   slew rate limit against the previous output, clip to +-power.
 *
 * Host vs target: float math, compared with tolerances. Out-of-range float -> int conversions (ARM saturates,
 * x86-64 returns INT_MIN) are only reached by the KNOWN_ISSUE_UB case of getLastScaledEnc.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include <cmath>
#include <cstdlib>

namespace {

constexpr uint32_t CPR = 1u << 24; //!< High resolution so that speed and acceleration are not quantised

/**
 * One axis with full scale torque (power 0x7fff, effect ratio 255), not inverted, internal damper off and a
 * high resolution encoder. Every other setting is the firmware default.
 */
struct TorqueFixture : AxisFixture {
	TorqueFixture() {
		setFullScale();
		axisCommand(Axis_commands::invert, CMDtype::set, 0);
		axisCommand(Axis_commands::axisdamper, CMDtype::set, 0);
		encoder().setCpr(CPR);
	}

	/** Sets the encoder to an angle in degrees (encoder frame, before inversion and gear ratio) */
	void setDeg(double deg) {
		posDeg = deg;
		encoder().setRawPosition(static_cast<int32_t>(std::lround(deg * CPR / 360.0)));
	}

	/**
	 * One 1 kHz FFB cycle for the axis like EffectsCalculator::calculateEffects + AxesManager::updateTorque:
	 * prepareForUpdate, internal effects, game torque, then the drive torque unless the emergency stop is active.
	 */
	void cycle(int32_t game, bool ffbOn = true) {
		HostPlatform::advanceMs(1);
		Axis &a = axis();
		a.prepareForUpdate();
		a.calculateMechanicalEffects(ffbOn);
		a.setFfbEffectTorque(game);
		if (!control.emergency) {
			a.updateDriveTorque();
		}
	}

	void cycles(uint32_t n, int32_t game, bool ffbOn = true) {
		for (uint32_t i = 0; i < n; i++) {
			cycle(game, ffbOn);
		}
	}

	/** Moves the encoder at a constant speed (deg/s in the encoder frame) for n cycles of `dtMs` */
	void moveAt(double degPerS, uint32_t n, int32_t game, bool ffbOn = true, double dtMs = 1.0) {
		for (uint32_t i = 0; i < n; i++) {
			setDeg(posDeg + degPerS * dtMs / 1000.0);
			cycle(game, ffbOn);
		}
	}

	/** Runs past the 1 s fade-in that starts when the driver is ready (at rest, no torque) */
	void finishFadeIn() { cycles(1100, 0); }

	/** Uses a large rotation range so that a moving axis stays far from the endstop */
	void useLargeRange() {
		axisCommand(Axis_commands::degrees, CMDtype::set, 7200);
		cycle(0);
	}

	int32_t torque() { return driver().lastTorque; }
	uint32_t clipPulses() { return HostPlatform::leds().clipPulses; }

	double posDeg = 0;
};

/** Endstop torque magnitude: (distance beyond the end in degrees) * esgain * 25 (Axis.cpp:831-835) */
double endstopTorque(double beyondDeg, int esgain = 127) {
	return std::min(beyondDeg * esgain * 25.0, 32767.0);
}

/**
 * Driver whose emergency stop does not set the torque to zero by itself (like a driver that only disables its
 * output stage). Records for every turn() how many emergency stops had been requested before it.
 */
class EstopOnlyDriver : public MockMotorDriver {
public:
	static constexpr uint16_t SELECTION_ID = 61;
	void emergencyStop(bool reset = false) override {
		emergencyStopCalls++;
		emergency = !reset;
	}
	void turn(int16_t power) override {
		emergencyStopsBeforeTurn.push_back(emergencyStopCalls);
		MockMotorDriver::turn(power);
	}
	std::vector<uint32_t> emergencyStopsBeforeTurn; //!< parallel to torqueHistory
};

/** Selects EstopOnlyDriver (and the mock encoder again) on the axis of the fixture */
void selectEstopOnlyDriver(AxisFixture &f) {
	static const std::vector<class_entry<MotorDriver>> registry = {add_class<MotorDriver, MotorDriver>(0),
																   add_class<EstopOnlyDriver, MotorDriver>(EstopOnlyDriver::SELECTION_ID)};
	f.axis().driverChooser = ClassChooser<MotorDriver>(registry);
	f.axis().setDrvType(EstopOnlyDriver::SELECTION_ID);
	f.axis().setEncType(MockEncoder::SELECTION_ID);
	f.encoder().setCpr(CPR);
}

} // namespace

TEST_SUITE("Axis") {

// ---------------------------------------------------------------------------------------------------------------
// Power, effect ratio, inversion, clipping
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TorqueFixture, "at full power and full effect ratio the game torque reaches the driver unchanged") {
	finishFadeIn();
	cycle(10000);
	CHECK_NEAR(torque(), 10000, 1);
	CHECK(axis().getTorque() == torque());
}

TEST_CASE_FIXTURE(TorqueFixture, "the power scales the torque by power / 0x7fff and is passed to the driver as power limit") {
	axisCommand(Axis_commands::power, CMDtype::set, 16384);
	CHECK(driver().powerLimit == 16384);
	finishFadeIn();
	cycle(20000);
	CHECK_NEAR(torque(), 20000.0 * 16384.0 / 32767.0, 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "the effect ratio scales the game torque by ratio / 255") {
	axisCommand(Axis_commands::fxratio, CMDtype::set, 127);
	finishFadeIn();
	cycle(20000);
	CHECK_NEAR(torque(), 20000.0 * 127.0 / 255.0, 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "an effect ratio of 0 removes the game torque") {
	axisCommand(Axis_commands::fxratio, CMDtype::set, 0);
	finishFadeIn();
	cycle(20000);
	CHECK(torque() == 0);
}

TEST_CASE_FIXTURE(TorqueFixture, "an inverted axis sends the game torque to the driver with the opposite sign") {
	axisCommand(Axis_commands::invert, CMDtype::set, 1);
	finishFadeIn();
	cycle(10000);
	CHECK_NEAR(torque(), -10000, 1);
	CHECK(axis().getTorque() == torque());
}

TEST_CASE_FIXTURE(TorqueFixture, "the summed torque is clipped to +-0x7fff at full power") {
	finishFadeIn();
	setDeg(470); // endstop pushes negative with its maximum
	cycle(-30000);
	CHECK(torque() == -0x7fff);
}

TEST_CASE_FIXTURE(TorqueFixture, "the summed torque is clipped to the power") {
	axisCommand(Axis_commands::power, CMDtype::set, 20000);
	finishFadeIn();
	setDeg(470);
	cycle(-30000);
	CHECK(torque() == -20000);
}

TEST_CASE_FIXTURE(TorqueFixture, "the clip led pulses when the output is limited to the power") {
	axisCommand(Axis_commands::power, CMDtype::set, 20000);
	finishFadeIn();
	setDeg(470);
	uint32_t before = clipPulses();
	cycle(-30000);
	CHECK(clipPulses() > before);
}

TEST_CASE_FIXTURE(TorqueFixture, "the clip led pulses when the game torque reaches full scale") {
	finishFadeIn();
	uint32_t before = clipPulses();
	cycle(32767);
	CHECK(clipPulses() == before + 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "the clip led does not pulse for a torque within the limits") {
	finishFadeIn();
	uint32_t before = clipPulses();
	cycles(10, 32766);
	CHECK(clipPulses() == before);
}

TEST_CASE_FIXTURE(TorqueFixture, "the driver never receives a torque above 0x7fff or with the wrong sign when the power is above 0x7fff" *
		KNOWN_ISSUE("(bug 32) Axis.cpp:498-501, 892-894: power is not limited to 0x7fff (setPower takes any uint16_t), the torque is "
					"clipped to +-power and passed to MotorDriver::turn(int16_t): the torque of 40000 wraps to -25536 (reversed torque). "
					"The first CHECK fails")) {
	axisCommand(Axis_commands::power, CMDtype::set, 40000);
	finishFadeIn();
	cycle(32767);
	CHECK(torque() > 0);
	CHECK(torque() <= 0x7fff);
	CHECK(axis().getTorque() <= 0x7fff);
}

TEST_CASE_FIXTURE(TorqueFixture, "the driver gets a new torque only when it changed") {
	finishFadeIn();
	driver().torqueHistory.clear();
	cycles(5, 10000);
	CHECK(driver().torqueHistory.size() == 1);
	cycle(9000);
	CHECK(driver().torqueHistory.size() == 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "a driver that is not ready gets no torque") {
	driver().ready = false;
	driver().torqueHistory.clear();
	cycles(1100, 10000);
	CHECK(driver().torqueHistory.empty());
	CHECK(axis().getTorque() != 0); // calculated, but not sent
}

// ---------------------------------------------------------------------------------------------------------------
// Fade-in
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TorqueFixture, "the startup fade-in begins when the driver becomes ready and ramps up linearly within 1 s") {
	driver().ready = false;
	cycles(500, 10000); // not ready: no fade started yet
	driver().ready = true;
	cycle(10000);
	CHECK(torque() == 0);
	cycles(499, 10000);
	CHECK_NEAR(torque(), 4990, 20);
	cycles(510, 10000);
	CHECK_NEAR(torque(), 10000, 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "the fade-in duration follows the update rate") {
	axis().updateSamplerate(500);
	cycles(250, 10000); // 0.5 s at 500 Hz
	CHECK_NEAR(torque(), 5000, 50);
	cycles(260, 10000);
	CHECK_NEAR(torque(), 10000, 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "zeroing the position restarts the fade-in at 25 percent") {
	finishFadeIn();
	cycle(10000);
	axisCommand(Axis_commands::zeroenc, CMDtype::get);
	cycle(10000);
	CHECK_NEAR(torque(), 2500, 10);
	cycles(100, 10000);
	CHECK(torque() > 2500);
	CHECK(torque() < 10000);
}

TEST_CASE_FIXTURE(TorqueFixture, "after a USB suspend the driver is stopped with zero torque") {
	finishFadeIn();
	cycle(10000);
	axis().usbSuspend();
	CHECK(torque() == 0);
	CHECK_FALSE(driver().running);
}

TEST_CASE_FIXTURE(TorqueFixture, "after a USB resume the driver is started and the torque fades in again from zero") {
	finishFadeIn();
	cycle(10000);
	axis().usbSuspend();
	uint32_t starts = driver().startCalls;
	axis().usbResume();
	CHECK(driver().running);
	CHECK(driver().startCalls == starts + 1);
	cycle(10000);
	CHECK(torque() == 0);
	int32_t last = 0;
	for (int i = 0; i < 100; i++) {
		cycle(10000);
		REQUIRE(torque() >= last);
		last = torque();
	}
	CHECK(last > 0);
}

TEST_CASE_FIXTURE(TorqueFixture, "after a USB resume the torque is back to full within the documented fade time of 0.5 s" *
		KNOWN_ISSUE("(bug 30) Axis.cpp:875: the multiplier grows by fadeTime / samplerate per cycle, so startForceFadeIn(start, "
					"fadeTime) (documented in Axis.h:357-360 and :473 as duration in s) takes (1 - start) / fadeTime s: the default "
					"0.5 s fade of usbResume and of the emergency stop reset (start 0) takes 2 s, zeroing (start 0.25) 1.5 s. The first CHECK fails")) {
	finishFadeIn();
	axis().usbSuspend();
	axis().usbResume();
	cycles(550, 10000);
	CHECK_NEAR(torque(), 10000, 1);
}

// ---------------------------------------------------------------------------------------------------------------
// Emergency stop
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TorqueFixture, "an emergency stop sends zero torque before it stops the driver") {
	selectEstopOnlyDriver(*this);
	finishFadeIn();
	cycle(10000);
	auto &drv = dynamic_cast<EstopOnlyDriver &>(driver());
	drv.torqueHistory.clear();
	drv.emergencyStopsBeforeTurn.clear();
	axis().emergencyStop(false);
	REQUIRE(drv.torqueHistory.size() >= 1);
	CHECK(drv.torqueHistory.front() == 0);
	CHECK(drv.emergencyStopsBeforeTurn.front() == 0); // zero torque requested before the driver's emergency stop
	CHECK(drv.emergencyStopCalls == 1);
	CHECK(drv.emergency);
	CHECK(control.emergency);
}

TEST_CASE_FIXTURE(TorqueFixture, "an emergency stop sends zero torque also to a driver that does not stop the torque itself") {
	selectEstopOnlyDriver(*this);
	axisCommand(Axis_commands::slewrate, CMDtype::set, 0);
	finishFadeIn();
	cycle(10000);
	REQUIRE(torque() != 0);
	axis().emergencyStop(false);
	CHECK(torque() == 0);
	CHECK(driver().emergencyStopCalls == 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "resetting the emergency stop clears it and fades the torque in from zero") {
	finishFadeIn();
	cycle(10000);
	axis().emergencyStop(false);
	cycles(10, 10000); // no torque while stopped
	CHECK(torque() == 0);

	axis().emergencyStop(true);
	CHECK_FALSE(control.emergency);
	CHECK_FALSE(driver().emergency);
	CHECK(driver().running);
	cycle(10000);
	CHECK(torque() == 0);
	cycles(100, 10000);
	CHECK(torque() > 0);
	CHECK(torque() < 10000);
}

TEST_CASE_FIXTURE(TorqueFixture, "after an emergency stop reset the slew rate limiter starts from the zero torque of the stopped motor" *
		KNOWN_ISSUE("(bug 33) Axis.cpp:917-921: the slew rate limiter compares with metric.previous.torque, which keeps the last "
					"torque before the emergency stop (emergencyStop only calls drv->turn(0), Axis.cpp:618). After the reset "
					"the first torque jumps from 0 to the old torque - slewrate (9899 instead of <= 100), bypassing the "
					"fade-in. The first CHECK fails")) {
	axisCommand(Axis_commands::slewrate, CMDtype::set, 100);
	finishFadeIn();
	cycles(150, 10000);
	REQUIRE(torque() > 9900);
	axis().emergencyStop(false);
	cycles(10, 10000);
	axis().emergencyStop(true);

	cycle(10000);
	CHECK(std::abs(torque()) <= 100);
}

TEST_CASE_FIXTURE(TorqueFixture, "after a USB resume the slew rate limiter starts from the zero torque of the stopped motor" *
		KNOWN_ISSUE("(bug 33) Axis.cpp:917-921: same root cause as after an emergency stop reset. usbSuspend only calls drv->turn(0) "
					"and stopMotor (Axis.cpp:629-634); metric.previous.torque keeps the torque before the suspend. After the "
					"resume the first torque jumps from 0 to the old torque - slewrate (9899 with slew rate 100) although the "
					"fade-in restarts at 0. The first CHECK fails")) {
	axisCommand(Axis_commands::slewrate, CMDtype::set, 100);
	finishFadeIn();
	cycles(150, 10000);
	REQUIRE(torque() > 9900);
	axis().usbSuspend();
	REQUIRE(torque() == 0);
	axis().usbResume(); // short suspend: no FFB cycle in between (FFBHIDMain keeps updating while suspended)
	REQUIRE_ALL_HAL_MODELLED();

	cycle(10000);
	CHECK(std::abs(torque()) <= 100);
}

// ---------------------------------------------------------------------------------------------------------------
// Slew rate limiter
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TorqueFixture, "the slew rate limits the torque increase per update") {
	axisCommand(Axis_commands::slewrate, CMDtype::set, 100);
	finishFadeIn();
	cycle(10000);
	CHECK(torque() == 100);
	cycle(10000);
	CHECK(torque() == 200);
	cycles(98, 10000);
	CHECK_NEAR(torque(), 10000, 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "the slew rate limits the torque decrease per update") {
	axisCommand(Axis_commands::slewrate, CMDtype::set, 1000);
	finishFadeIn();
	cycles(20, 10000);
	int32_t before = torque();
	REQUIRE(before > 9900);
	cycle(-10000);
	CHECK(torque() == before - 1000);
}

TEST_CASE_FIXTURE(TorqueFixture, "the slew rate limits the inverted torque sent to the driver") {
	axisCommand(Axis_commands::invert, CMDtype::set, 1);
	axisCommand(Axis_commands::slewrate, CMDtype::set, 100);
	finishFadeIn();
	cycle(10000);
	CHECK(torque() == -100);
	cycle(10000);
	CHECK(torque() == -200);
}

TEST_CASE_FIXTURE(TorqueFixture, "a slew rate of 0 disables the limiter") {
	axisCommand(Axis_commands::slewrate, CMDtype::set, 0);
	finishFadeIn();
	cycle(10000);
	CHECK_NEAR(torque(), 10000, 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "the slew rate of a driver is the axis slew rate after the driver is selected") {
	CHECK(axisGet(Axis_commands::slewrate) == MAX_SLEW_RATE); // MockMotorDriver::getDrvSlewRate
	finishFadeIn();
	cycle(30000);
	CHECK_NEAR(torque(), 30000, 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "a driver slew rate calibration lowers the axis slew rate to the measured driver limit") {
	driver().supportsSlewCalibration = true;
	finishFadeIn();
	axisCommand(Axis_commands::calibrate_maxSlewRateDrv, CMDtype::get);
	cycles(5, 0); // calibration in progress: no change
	CHECK(axisGet(Axis_commands::slewrate) == MAX_SLEW_RATE);

	driver().slewRate = 300;
	driver().slewCalibrationInProgress = false;
	cycle(10000);
	CHECK(torque() == 300);
	cycle(10000);
	CHECK(torque() == 600);
	CHECK(axisGet(Axis_commands::maxSlewRateDrv) == 300);
}

TEST_CASE_FIXTURE(TorqueFixture, "a driver slew rate calibration keeps a lower axis slew rate") {
	axisCommand(Axis_commands::slewrate, CMDtype::set, 100);
	driver().supportsSlewCalibration = true;
	finishFadeIn();
	axisCommand(Axis_commands::calibrate_maxSlewRateDrv, CMDtype::get);
	driver().slewRate = 300;
	driver().slewCalibrationInProgress = false;
	cycle(10000);
	CHECK(torque() == 100);
}

// ---------------------------------------------------------------------------------------------------------------
// Endstop
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TorqueFixture, "within the rotation range there is no endstop torque") {
	finishFadeIn();
	setDeg(449);
	cycle(0);
	CHECK(torque() == 0);
	setDeg(-449);
	cycle(0);
	CHECK(torque() == 0);
}

TEST_CASE_FIXTURE(TorqueFixture, "beyond the positive end the endstop pushes back proportional to the distance and the endstop gain") {
	finishFadeIn();
	setDeg(455);
	cycle(0);
	CHECK_NEAR(torque(), -endstopTorque(5), 2);
	setDeg(460);
	cycle(0);
	CHECK_NEAR(torque(), -endstopTorque(10), 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "beyond the negative end the endstop pushes back in the positive direction") {
	finishFadeIn();
	setDeg(-460);
	cycle(0);
	CHECK_NEAR(torque(), endstopTorque(10), 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "the endstop pushes back towards the range also on an inverted axis") {
	axisCommand(Axis_commands::invert, CMDtype::set, 1);
	finishFadeIn();
	setDeg(460);
	cycle(0);
	CHECK_NEAR(torque(), -endstopTorque(10), 2);
	setDeg(-460);
	cycle(0);
	CHECK_NEAR(torque(), endstopTorque(10), 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "the endstop stiffness follows esgain") {
	axisCommand(Axis_commands::esgain, CMDtype::set, 50);
	finishFadeIn();
	setDeg(460);
	cycle(0);
	CHECK_NEAR(torque(), -endstopTorque(10, 50), 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "an endstop gain of 0 disables the endstop") {
	axisCommand(Axis_commands::esgain, CMDtype::set, 0);
	finishFadeIn();
	setDeg(460);
	cycle(0);
	CHECK(torque() == 0);
}

TEST_CASE_FIXTURE(TorqueFixture, "the endstop follows the configured rotation range") {
	axisCommand(Axis_commands::degrees, CMDtype::set, 540);
	finishFadeIn();
	setDeg(275);
	cycle(0);
	CHECK_NEAR(torque(), -endstopTorque(5), 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "the endstop torque is limited to 0x7fff before the game torque is added") {
	finishFadeIn();
	setDeg(470); // 20 deg beyond: 63500 unclipped
	cycle(30000);
	CHECK_NEAR(torque(), 30000 - 32767, 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "the endstop is not reduced by the effect ratio but by the power") {
	axisCommand(Axis_commands::fxratio, CMDtype::set, 0);
	axisCommand(Axis_commands::power, CMDtype::set, 16384);
	finishFadeIn();
	setDeg(460);
	cycle(10000);
	CHECK_NEAR(torque(), -endstopTorque(10) * 16384.0 / 32767.0, 2);
}

// ---------------------------------------------------------------------------------------------------------------
// Out of range shutdown
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TorqueFixture, "an axis beyond twice its half range stops the driver, outputs zero torque and reports an error") {
	finishFadeIn();
	cycle(10000);
	uint32_t errPulses = HostPlatform::leds().errPulses;
	setDeg(950); // scaled position > 0xffff
	cycle(10000);
	CHECK(driver().stopCalls == 1);
	CHECK(torque() == 0);
	CHECK(axis().getTorque() == 0);
	CHECK(errorsWithCode(ErrorCode::axisOutOfRange).size() == 1);
	CHECK(HostPlatform::leds().errPulses > errPulses);
}

TEST_CASE_FIXTURE(TorqueFixture, "the out of range error is added only once while the axis stays out of range") {
	finishFadeIn();
	setDeg(950);
	cycles(5, 0);
	CHECK(errorsWithCode(ErrorCode::axisOutOfRange).size() == 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "out of range torque stays off until the axis is back within the range") {
	finishFadeIn();
	setDeg(950);
	cycle(10000);
	setDeg(600); // between the end of the range and the shutdown limit
	cycle(10000);
	CHECK(axis().getTorque() == 0);
	setDeg(0);
	cycle(10000);
	CHECK_NEAR(axis().getTorque(), 10000, 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "clearing the out of range error restarts the driver") {
	finishFadeIn();
	setDeg(950);
	cycle(0);
	REQUIRE_FALSE(driver().running);
	ErrorHandler::clearError(Error(ErrorCode::axisOutOfRange, ErrorType::warning, "Axis out of bounds"));
	CHECK(driver().running);
}

TEST_CASE_FIXTURE(TorqueFixture, "a driver that is not ready is not stopped by an out of range position") {
	finishFadeIn();
	driver().ready = false;
	setDeg(950);
	cycle(0);
	CHECK(driver().stopCalls == 0);
	CHECK(errorsWithCode(ErrorCode::axisOutOfRange).empty());
}

// ---------------------------------------------------------------------------------------------------------------
// Position, range and gear ratio
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TorqueFixture, "a new rotation range is applied at the next update") {
	setDeg(90);
	axisCommand(Axis_commands::degrees, CMDtype::set, 360);
	CHECK(axisGet(Axis_commands::degrees) == 900);
	cycle(0);
	CHECK(axisGet(Axis_commands::degrees) == 360);
	CHECK_NEAR(metrics().pos_f, 0.5, 1e-4);
	CHECK_NEAR(metrics().pos_scaled_16b, 0xffff / 4, 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "a smaller rotation range waits until the position is within it") {
	axisCommand(Axis_commands::degrees, CMDtype::set, 1080);
	cycle(0);
	setDeg(500);
	cycle(0);
	axisCommand(Axis_commands::degrees, CMDtype::set, 900);
	cycle(0);
	CHECK(axisGet(Axis_commands::degrees) == 1080);
	setDeg(400);
	cycle(0);
	CHECK(axisGet(Axis_commands::degrees) == 900);
}

TEST_CASE_FIXTURE(TorqueFixture, "a rotation range of 0 returns to the previous range") {
	axisCommand(Axis_commands::degrees, CMDtype::set, 1080);
	cycle(0);
	axisCommand(Axis_commands::degrees, CMDtype::set, 0);
	cycle(0);
	CHECK(axisGet(Axis_commands::degrees) == 900);
}

TEST_CASE_FIXTURE(TorqueFixture, "the gear ratio multiplies the encoder angle by (numerator + 1) / (denominator + 1)") {
	axis().setGearRatio(1, 3); // 2 / 4
	setDeg(90);
	cycle(0);
	CHECK_NEAR(metrics().posDegrees, 45.0, 1e-3);
	axis().setGearRatio(3, 0); // 4 / 1
	setDeg(10);
	cycle(0);
	CHECK_NEAR(metrics().posDegrees, 40.0, 1e-3);
}

TEST_CASE_FIXTURE(TorqueFixture, "the endstop uses the axis angle after the gear ratio") {
	axis().setGearRatio(1, 0); // 2 / 1
	finishFadeIn();
	setDeg(230); // axis at 460 deg
	cycle(0);
	CHECK_NEAR(torque(), -endstopTorque(10), 3);
}

TEST_CASE_FIXTURE(TorqueFixture, "the scaled gamepad position is the position in the range times 0x7fffffff") {
	setDeg(225); // half of the positive half range of 900 deg
	cycle(0);
	CHECK_NEAR(axis().getLastScaledEnc(), 0.5 * 2147483647.0, 300);
	setDeg(-225);
	cycle(0);
	CHECK_NEAR(axis().getLastScaledEnc(), -0.5 * 2147483647.0, 300);
}

TEST_CASE_FIXTURE(TorqueFixture, "an axis beyond the end of its range reports the largest gamepad position" *
		KNOWN_ISSUE_UB("(bug 36) Axis.cpp:656: clip(pos_f * 0x7fffffff, ...) is done in float, where 0x7fffffff becomes 2^31; "
					   "for pos_f >= 1 the float 2^31 or more is converted to int32_t (undefined behaviour: ARM saturates to "
					   "0x7fffffff, x86-64 gives INT_MIN, i.e. the opposite end in the HID report). The first CHECK fails on the host")) {
	setDeg(500);
	cycle(0);
	CHECK(axis().getLastScaledEnc() > 0x7fff0000);
}

TEST_CASE_FIXTURE(TorqueFixture, "without an encoder the axis angle is 0") {
	CHECK(axis().getEncAngle(nullptr) == 0.0f);
}

TEST_CASE_FIXTURE(TorqueFixture, "characterization: scaling to a range of 0 degrees gives 0x7fff and 0.0") {
	// NEEDS REVIEW 33: the integer result is the end of the range (0x7fff) but the float result is the center (0.0).
	// Not reachable through setDegrees (0 restores the previous range); only through a degrees value of 0 in flash.
	auto scaled = axis().scaleEncValue(123.0f, 0);
	CHECK(scaled.first == 0x7fff);
	CHECK(scaled.second == 0.0f);
}

// ---------------------------------------------------------------------------------------------------------------
// Metrics: speed and acceleration
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TorqueFixture, "the speed metric converges to the rotation speed in deg/s") {
	useLargeRange();
	moveAt(100, 200, 0);
	CHECK_NEAR(metrics().speed, 100.0, 0.5);
	moveAt(-300, 200, 0);
	CHECK_NEAR(metrics().speed, -300.0, 1.0);
}

TEST_CASE_FIXTURE(TorqueFixture, "the speed metric is low pass filtered") {
	useLargeRange();
	moveAt(100, 1, 0);
	CHECK(metrics().speed > 0.0f);
	CHECK(metrics().speed < 20.0f);
}

TEST_CASE_FIXTURE(TorqueFixture, "the speed metric uses the update rate") {
	axis().updateSamplerate(500);
	useLargeRange();
	moveAt(100, 200, 0, true, 2.0); // 0.2 deg per update at 500 Hz
	CHECK_NEAR(metrics().speed, 100.0, 0.5);
}

TEST_CASE_FIXTURE(TorqueFixture, "the speed metric is in the inverted axis direction") {
	axisCommand(Axis_commands::invert, CMDtype::set, 1);
	useLargeRange();
	moveAt(100, 200, 0);
	CHECK_NEAR(metrics().speed, -100.0, 0.5);
}

TEST_CASE_FIXTURE(TorqueFixture, "the acceleration metric converges to the constant acceleration in deg/s2") {
	useLargeRange();
	double speed = 0;
	for (int i = 0; i < 300; i++) {
		speed += 1.0; // 1000 deg/s2
		setDeg(posDeg + speed / 1000.0);
		cycle(0);
	}
	CHECK_NEAR(metrics().accel, 1000.0, 50.0);
}

TEST_CASE_FIXTURE(TorqueFixture, "the acceleration metric is zero at a constant speed") {
	useLargeRange();
	moveAt(200, 300, 0);
	CHECK_NEAR(metrics().accel, 0.0, 10.0);
}

// ---------------------------------------------------------------------------------------------------------------
// Idle spring
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TorqueFixture, "with FFB off the idle spring pulls the axis to the center") {
	finishFadeIn();
	setDeg(10);
	cycle(0, false);
	// scaled position 0xffff / 900 * 10 = 728, scale 0.5 + 127 * 0.01
	CHECK_NEAR(torque(), -728 * 1.77, 2);
	setDeg(-10);
	cycle(0, false);
	CHECK_NEAR(torque(), 728 * 1.77, 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "the idle spring pulls to the center also on an inverted axis") {
	axisCommand(Axis_commands::invert, CMDtype::set, 1);
	finishFadeIn();
	setDeg(10);
	cycle(0, false);
	CHECK_NEAR(torque(), -728 * 1.77, 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "the idle spring force is limited to 35 times its strength") {
	finishFadeIn();
	setDeg(300);
	cycle(0, false);
	CHECK(torque() == -127 * 35);
	axisCommand(Axis_commands::idlespring, CMDtype::set, 255);
	cycle(0, false);
	CHECK(torque() == -255 * 35);
}

TEST_CASE_FIXTURE(TorqueFixture, "the idle spring stiffness grows with its strength") {
	axisCommand(Axis_commands::idlespring, CMDtype::set, 50);
	finishFadeIn();
	setDeg(10);
	cycle(0, false);
	CHECK_NEAR(torque(), -728 * 1.0, 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "an idle spring strength of 0 disables it") {
	axisCommand(Axis_commands::idlespring, CMDtype::set, 0);
	finishFadeIn();
	setDeg(100);
	cycle(0, false);
	CHECK(torque() == 0);
}

TEST_CASE_FIXTURE(TorqueFixture, "with FFB on there is no idle spring") {
	finishFadeIn();
	setDeg(100);
	cycle(0, true);
	CHECK(torque() == 0);
}

TEST_CASE_FIXTURE(TorqueFixture, "the idle spring is not reduced by the effect ratio") {
	axisCommand(Axis_commands::fxratio, CMDtype::set, 0);
	finishFadeIn();
	setDeg(10);
	cycle(0, false);
	CHECK_NEAR(torque(), -728 * 1.77, 2);
}

// ---------------------------------------------------------------------------------------------------------------
// Internal damper, friction, inertia
// ---------------------------------------------------------------------------------------------------------------

/** Damper torque at a constant speed: speed * strength * 40 * 0.7 / 255 (Axis.cpp:699, Axis.h:394) */
static double damperTorque(double speed, int strength) {
	return std::min(speed * strength * 40.0 * 0.7 / 255.0, 20000.0);
}

TEST_CASE_FIXTURE(AxisFixture, "an axis has an internal damper of strength 30 by default") {
	setFullScale();
	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	encoder().setCpr(CPR);
	axisCommand(Axis_commands::degrees, CMDtype::set, 7200);
	axisCommand(Axis_commands::idlespring, CMDtype::set, 0); // FFB is off: no idle spring
	run(1100);
	double pos = 0;
	for (int i = 0; i < 300; i++) {
		pos += 0.1; // 100 deg/s
		encoder().setRawPosition(static_cast<int32_t>(std::lround(pos * CPR / 360.0)));
		run(1);
	}
	CHECK(axisGet(Axis_commands::axisdamper) == 30);
	CHECK_NEAR(driver().lastTorque, -damperTorque(100, 30), 3);
}

TEST_CASE_FIXTURE(TorqueFixture, "the internal damper opposes the movement proportional to speed and strength") {
	axisCommand(Axis_commands::axisdamper, CMDtype::set, 100);
	useLargeRange();
	finishFadeIn();
	moveAt(100, 300, 0);
	CHECK_NEAR(torque(), -damperTorque(100, 100), 3);
	moveAt(-100, 300, 0);
	CHECK_NEAR(torque(), damperTorque(100, 100), 3);
}

TEST_CASE_FIXTURE(TorqueFixture, "the internal damper is added to the game torque") {
	axisCommand(Axis_commands::axisdamper, CMDtype::set, 100);
	useLargeRange();
	finishFadeIn();
	moveAt(100, 300, 5000);
	CHECK_NEAR(torque(), 5000 - damperTorque(100, 100), 3);
}

TEST_CASE_FIXTURE(TorqueFixture, "the internal damper torque is limited to 20000") {
	axisCommand(Axis_commands::axisdamper, CMDtype::set, 255);
	useLargeRange();
	finishFadeIn();
	moveAt(1000, 300, 0); // unclipped 28000
	CHECK_NEAR(torque(), -20000, 3);
}

TEST_CASE_FIXTURE(TorqueFixture, "the internal damper is not reduced by the effect ratio") {
	axisCommand(Axis_commands::axisdamper, CMDtype::set, 100);
	axisCommand(Axis_commands::fxratio, CMDtype::set, 0);
	useLargeRange();
	finishFadeIn();
	moveAt(100, 300, 5000);
	CHECK_NEAR(torque(), -damperTorque(100, 100), 3);
}

TEST_CASE_FIXTURE(TorqueFixture, "the internal damper starts from a reset filter when it is switched on again") {
	axisCommand(Axis_commands::axisdamper, CMDtype::set, 100);
	useLargeRange();
	finishFadeIn();
	moveAt(100, 300, 0);
	axisCommand(Axis_commands::axisdamper, CMDtype::set, 0);
	moveAt(100, 10, 0);
	CHECK(torque() == 0);
	axisCommand(Axis_commands::axisdamper, CMDtype::set, 100);
	moveAt(100, 1, 0);
	CHECK(std::abs(torque()) < damperTorque(100, 100) / 4); // filter starts at 0
}

/** Friction torque magnitude (Axis.cpp:711-724) */
static double frictionTorque(double speed, int strength) {
	double s = std::fabs(speed * 45.0);
	double ramp = 1.0;
	if (s < 4096.0) {
		ramp = (1.0 + std::sin(M_PI * (s / 4096.0 - 0.5))) / 2.0;
	}
	return strength * ramp * 0.7 * 32.0;
}

TEST_CASE_FIXTURE(TorqueFixture, "the internal friction opposes the movement with a constant force above the ramp-up speed") {
	axisCommand(Axis_commands::axisfriction, CMDtype::set, 100);
	useLargeRange();
	finishFadeIn();
	moveAt(100, 300, 0);
	CHECK_NEAR(torque(), -frictionTorque(100, 100), 3);
	CHECK_NEAR(torque(), -2240, 3);
	moveAt(-200, 300, 0);
	CHECK_NEAR(torque(), 2240, 3);
}

TEST_CASE_FIXTURE(TorqueFixture, "the internal friction ramps up with a sine curve below the ramp-up speed") {
	axisCommand(Axis_commands::axisfriction, CMDtype::set, 100);
	useLargeRange();
	finishFadeIn();
	moveAt(50, 300, 0);
	CHECK_NEAR(torque(), -frictionTorque(50, 100), 15);
	CHECK(std::abs(torque()) < 2000);
}

TEST_CASE_FIXTURE(TorqueFixture, "the internal friction is zero at rest") {
	axisCommand(Axis_commands::axisfriction, CMDtype::set, 255);
	finishFadeIn();
	cycles(100, 0);
	CHECK(torque() == 0);
}

TEST_CASE_FIXTURE(TorqueFixture, "the internal inertia opposes the acceleration") {
	axisCommand(Axis_commands::axisinertia, CMDtype::set, 100);
	useLargeRange();
	finishFadeIn();
	double speed = 0;
	for (int i = 0; i < 300; i++) {
		speed += 1.0; // 1000 deg/s2
		setDeg(posDeg + speed / 1000.0);
		cycle(0);
	}
	// accel * strength * 4 * 0.7 / 255
	CHECK_NEAR(torque(), -1000.0 * 100 * 4 * 0.7 / 255.0, 60);
}

TEST_CASE_FIXTURE(TorqueFixture, "the internal inertia is zero at a constant speed") {
	axisCommand(Axis_commands::axisinertia, CMDtype::set, 255);
	useLargeRange();
	finishFadeIn();
	moveAt(200, 300, 0);
	CHECK(std::abs(torque()) <= 30);
}

// ---------------------------------------------------------------------------------------------------------------
// Expo curve and equalizer
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TorqueFixture, "a positive expo bends the game torque curve with exponent 1 + expo / 50") {
	axisCommand(Axis_commands::expo, CMDtype::set, 50); // exponent 2
	finishFadeIn();
	cycle(16384);
	CHECK_NEAR(torque(), 32767.0 * std::pow(16384.0 / 32767.0, 2.0), 2);
	cycle(-16384);
	CHECK_NEAR(torque(), -32767.0 * std::pow(16384.0 / 32767.0, 2.0), 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "a negative expo bends the game torque curve with exponent 1 / (1 + |expo| / 50)") {
	axisCommand(Axis_commands::expo, CMDtype::set, -50); // exponent 0.5
	finishFadeIn();
	cycle(16384);
	CHECK_NEAR(torque(), 32767.0 * std::sqrt(16384.0 / 32767.0), 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "the expo keeps full scale and zero game torque") {
	axisCommand(Axis_commands::expo, CMDtype::set, 100);
	finishFadeIn();
	cycle(32767);
	CHECK_NEAR(torque(), 32767, 1);
	cycle(0);
	CHECK(torque() == 0);
}

TEST_CASE_FIXTURE(TorqueFixture, "the expo is limited to +-127") {
	axisCommand(Axis_commands::expo, CMDtype::set, 127);
	finishFadeIn();
	cycle(16384);
	int32_t at127 = torque();
	CHECK(at127 < 8192); // stronger than expo 50
	axisCommand(Axis_commands::expo, CMDtype::set, 200);
	CHECK(axisGet(Axis_commands::expo) == 127);
	cycle(16385);
	cycle(16384);
	CHECK(torque() == at127);
	axisCommand(Axis_commands::expo, CMDtype::set, -200);
	CHECK(axisGet(Axis_commands::expo) == -127);
}

// Axis.cpp:1026 calls abs() on a float. On the target the float overload comes in through a transitive
// <math.h> include (TMC4671.h -> SPI.h); ffb_target_tests compiles Axis.cpp with -include math.h to match.
// Without it ::abs(int) would truncate the exponent (host-vs-target caveat, see PROGRESS).
TEST_CASE_FIXTURE(TorqueFixture, "an expo of 25 bends the game torque curve with exponent 1.5") {
	axisCommand(Axis_commands::expo, CMDtype::set, 25);
	finishFadeIn();
	cycle(16384);
	CHECK_NEAR(torque(), 32767.0 * std::pow(16384.0 / 32767.0, 1.5), 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "the largest expo of 127 bends the game torque curve with exponent 3.54") {
	axisCommand(Axis_commands::expo, CMDtype::set, 127);
	finishFadeIn();
	cycle(16384);
	CHECK_NEAR(torque(), 32767.0 * std::pow(16384.0 / 32767.0, 1.0 + 127.0 / 50.0), 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "an expo set back to 0 makes the game torque curve linear again") {
	axisCommand(Axis_commands::expo, CMDtype::set, 50);
	axisCommand(Axis_commands::expo, CMDtype::set, 0);
	finishFadeIn();
	cycle(16384);
	CHECK_NEAR(torque(), 16384, 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "the expo is applied before the effect ratio") {
	axisCommand(Axis_commands::expo, CMDtype::set, 50);
	axisCommand(Axis_commands::fxratio, CMDtype::set, 127);
	finishFadeIn();
	cycle(16384);
	CHECK_NEAR(torque(), 32767.0 * 0.25 * 127.0 / 255.0, 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "the expo does not change the endstop torque") {
	axisCommand(Axis_commands::expo, CMDtype::set, 50);
	finishFadeIn();
	setDeg(455);
	cycle(0);
	CHECK_NEAR(torque(), -endstopTorque(5), 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "an equalizer with all gains at 0 dB passes the game torque unchanged") {
	axisCommand(Axis_commands::equalizer, CMDtype::set, 1);
	finishFadeIn();
	cycles(300, 10000);
	CHECK_NEAR(torque(), 10000, 2);
}

TEST_CASE_FIXTURE(TorqueFixture, "the low shelf band of the equalizer boosts a constant game torque by its gain") {
	axisCommand(Axis_commands::equalizer, CMDtype::set, 1);
	axisCommand(Axis_commands::eqb1, CMDtype::set, 60); // +6 dB below 10 Hz
	finishFadeIn();
	cycles(1000, 8000);
	CHECK_NEAR(torque(), 8000.0 * std::pow(10.0, 6.0 / 20.0), 40);
}

TEST_CASE_FIXTURE(TorqueFixture, "a disabled equalizer does not change the game torque") {
	axisCommand(Axis_commands::eqb1, CMDtype::set, 60);
	finishFadeIn();
	cycles(300, 8000);
	CHECK_NEAR(torque(), 8000, 1);
}

// ---------------------------------------------------------------------------------------------------------------
// Speed limiter
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TorqueFixture, "the speed limiter is disabled with a maximum speed of 0") {
	useLargeRange();
	finishFadeIn();
	moveAt(500, 300, 10000);
	CHECK_NEAR(torque(), 10000, 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "the speed limiter does not reduce a torque against the movement") {
	useLargeRange();
	finishFadeIn();
	moveAt(500, 300, -10000);
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 100);
	moveAt(500, 20, -10000);
	CHECK_NEAR(torque(), -10000, 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "the speed limiter does not reduce the torque below the maximum speed") {
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 600);
	useLargeRange();
	finishFadeIn();
	moveAt(500, 300, 10000);
	CHECK_NEAR(torque(), 10000, 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "the speed limiter reduces the torque just above the maximum speed") {
	useLargeRange();
	finishFadeIn();
	moveAt(150, 300, 10000);
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 100);
	moveAt(150, 1, 10000);
	CHECK(torque() < 9990);
}

TEST_CASE_FIXTURE(TorqueFixture, "the speed limiter reduces a torque in the direction of the movement but never inverts it") {
	useLargeRange();
	finishFadeIn();
	moveAt(500, 300, 100);
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 100);
	moveAt(500, 20, 100);
	CHECK(torque() == 0);
	moveAt(500, 1, 10000);
	CHECK(torque() < 10000);
	CHECK(torque() >= 0);
}

TEST_CASE_FIXTURE(TorqueFixture, "the speed limiter acts in the axis direction of an inverted axis") {
	axisCommand(Axis_commands::invert, CMDtype::set, 1);
	useLargeRange();
	finishFadeIn();
	moveAt(-500, 300, 10000); // axis moves positive, game torque positive
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 100);
	moveAt(-500, 1, 10000);
	CHECK(torque() > -10000);
	CHECK(torque() <= 0);
}

TEST_CASE_FIXTURE(TorqueFixture, "the speed limiter reduces a negative torque in the direction of a negative movement") {
	useLargeRange();
	finishFadeIn();
	moveAt(-500, 300, -10000);
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 100);
	moveAt(-500, 1, -10000);
	CHECK(torque() > -10000);
	CHECK(torque() <= 0);
}

#ifdef USE_DSP_FUNCTIONS
// arm_pid_f32 with Kp 0.3, Ki 0.03, Kd 0: out[n] = out[n-1] + 0.33 * e[n] - 0.3 * e[n-1], e in deg/s
// NEEDS REVIEW #32: the gain differs by orders of magnitude from the build without CMSIS-DSP; which one is intended?
TEST_CASE_FIXTURE(TorqueFixture, "characterization: with CMSIS-DSP the speed limiter reduces the torque by a PI controller on the speed error in deg/s") {
	useLargeRange();
	finishFadeIn();
	moveAt(500, 300, 10000);
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 100);
	moveAt(500, 1, 10000);
	CHECK_NEAR(torque(), 10000 - 0.33 * 400, 2);
	moveAt(500, 9, 10000);
	CHECK_NEAR(torque(), 10000 - (0.33 * 400 + 9 * 0.03 * 400), 3);
}

// NEEDS REVIEW #32
TEST_CASE_FIXTURE(TorqueFixture, "characterization: with CMSIS-DSP the speed limiter restarts its integral when it is not active") {
	useLargeRange();
	finishFadeIn();
	moveAt(500, 300, 10000);
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 100);
	moveAt(500, 10, 10000);
	moveAt(500, 1, -10000); // torque against the movement: limiter inactive
	moveAt(500, 1, 10000);
	CHECK_NEAR(torque(), 10000 - 0.33 * 400, 2);
}
#else
// Fallback: speed error scaled by 0x7fff / maxspeed, P 0.3, I 0.03 * torque scaler, I limited to 0..power
// NEEDS REVIEW #32: the gain differs by orders of magnitude from the CMSIS-DSP build (the target); which one is intended?
TEST_CASE_FIXTURE(TorqueFixture, "characterization: without CMSIS-DSP the speed limiter reduces the torque by a PI controller on the speed error scaled to the torque range") {
	useLargeRange();
	finishFadeIn();
	moveAt(110, 300, 10000);
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 100);
	moveAt(110, 1, 10000);
	const double reducer = 10.0 * 32767.0 / 100.0;
	CHECK_NEAR(torque(), 10000 - (reducer * 0.3 + reducer * 0.03), 3);
	moveAt(110, 9, 10000);
	CHECK_NEAR(torque(), 10000 - (reducer * 0.3 + 10 * reducer * 0.03), 4);
}

// NEEDS REVIEW #32
TEST_CASE_FIXTURE(TorqueFixture, "characterization: without CMSIS-DSP the speed limiter restarts its integral when it is not active") {
	useLargeRange();
	finishFadeIn();
	moveAt(110, 300, 10000);
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 100);
	moveAt(110, 10, 10000);
	moveAt(110, 1, -10000);
	moveAt(110, 1, 10000);
	const double reducer = 10.0 * 32767.0 / 100.0;
	CHECK_NEAR(torque(), 10000 - (reducer * 0.3 + reducer * 0.03), 3);
}

// NEEDS REVIEW #32 (the torque is gone from about 1.9 times the maximum speed)
TEST_CASE_FIXTURE(TorqueFixture, "characterization: without CMSIS-DSP the speed limiter removes the torque far above the maximum speed") {
	useLargeRange();
	finishFadeIn();
	moveAt(500, 300, 10000);
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 100);
	moveAt(500, 1, 10000);
	CHECK(torque() == 0);
}
#endif

// ---------------------------------------------------------------------------------------------------------------
// Hands-off detection
// The two passing cases below cannot fail today: handsOff is computed but never used (bug 35), so they only
// guard against a future implementation cutting the torque while the hands are on the wheel. The check runs on
// every 11th call through a function static check_counter (Axis.cpp:1339), shared by all axes and never reset:
// the phase of the 11-cycle check depends on the test order.
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TorqueFixture, "with hands-off detection enabled a moving wheel below the speed threshold keeps the game torque") {
	axisCommand(Axis_commands::handsoff, CMDtype::set, 1);
	useLargeRange();
	finishFadeIn();
	// Oscillating movement: acceleration varies (std dev far above the threshold), speed below 720 deg/s
	for (int i = 0; i < 1000; i++) {
		setDeg(10.0 * std::sin(2.0 * M_PI * 2.0 * i / 1000.0));
		cycle(5000);
	}
	CHECK_NEAR(torque(), 5000, 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "with hands-off detection enabled a small game torque passes unchanged") {
	axisCommand(Axis_commands::handsoff, CMDtype::set, 1);
	finishFadeIn();
	cycles(1000, 900);
	CHECK_NEAR(torque(), 900, 1);
}

TEST_CASE_FIXTURE(TorqueFixture, "hands-off detection cuts the torque when the wheel spins faster than the speed threshold" *
		KNOWN_ISSUE("(bug 35) Axis.cpp:1346-1349 sets handsOff ('Safety override: if speed is very high, cut torque') but handsOff is "
					"never read: updateTorque does not use it. The first CHECK fails")) {
	axisCommand(Axis_commands::handsoff, CMDtype::set, 1);
	useLargeRange();
	finishFadeIn();
	moveAt(1000, 300, 5000); // above the default threshold of 720 deg/s
	CHECK(torque() == 0);
}

TEST_CASE_FIXTURE(TorqueFixture, "hands-off detection reduces the game torque after 500 ms without hands on the wheel" *
		KNOWN_ISSUE("(bug 35) Axis.cpp:1377-1382 detects hands off (acceleration std dev below the threshold for 500 ms with a game "
					"torque above 1000) but handsOff is never used (Axis.h:455: 'updates the torque accordingly'). "
					"The first CHECK fails")) {
	axisCommand(Axis_commands::handsoff, CMDtype::set, 1);
	finishFadeIn();
	cycles(1000, 5000); // wheel does not move although a torque is applied
	CHECK(std::abs(torque()) < 4900); // clearly less than the game torque (4999 after float rounding)
}

} // TEST_SUITE
