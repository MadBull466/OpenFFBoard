/*
 * Smoke tests of the ffb_target_tests setup (P2 batch 0): the real Axis with the mock driver and encoder,
 * the torque path from a PID report to the driver, and a real main class with the fake timer and USB.
 * Detailed tests of Axis, AxesManager and the main classes are in their own files.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "FFBWheel.h"
#include "mainclass_chooser.h"

#include <cstring>
#include <type_traits>

TEST_SUITE("TargetSetup") {

TEST_CASE_FIXTURE(AxisFixture, "the axis uses the mock driver and mock encoder selected through its class choosers") {
	CHECK(axis().getDrvType() == MockMotorDriver::SELECTION_ID);
	CHECK(axis().getEncType() == MockEncoder::SELECTION_ID);
	CHECK(driver().setupCalls == 1);
	CHECK(driver().running); // USB is connected: setDrvType starts the motor
	CHECK(driver().assignedEncoder().get() == &encoder());
}

TEST_CASE_FIXTURE(AxisFixture, "an encoder position is reported by the axis metrics in degrees and scaled to the range") {
	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	axisCommand(Axis_commands::degrees, CMDtype::set, 900);
	encoder().setDegrees(90.0f); // 90 of the 900 degree range

	update();

	CHECK_NEAR(metrics().posDegrees, 90.0, 0.01);
	CHECK_NEAR(metrics().pos_f, 0.2, 1e-5);			// 2 / 900 * 90: -1..1 over the range
	CHECK_NEAR(metrics().pos_scaled_16b, 6553, 1); // 0xffff / 900 * 90
}

TEST_CASE_FIXTURE(AxisFixture, "with the default inversion the axis position has the opposite sign of the encoder") {
	encoder().setDegrees(45.0f);

	update();

	CHECK(axisGet(Axis_commands::invert) == 1);
	CHECK_NEAR(metrics().posDegrees, -45.0, 0.01);
}

TEST_CASE_FIXTURE(AxisFixture, "a constant force sent by the PID host reaches the motor driver inverted after the fade-in") {
	setFullScale();
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(cf, 10000);

	run(1100); // The fade-in after the driver became ready takes 1 s at 1 kHz

	// EffectsCalculator passes -10000 for a positive constant force in direction 90 deg (east), the axis
	// inverts it (default) and scales it with power 0x7fff and effect ratio 255 (both 1.0)
	CHECK_NEAR(driver().lastTorque, 10000, 2);
	CHECK(axis().getTorque() == driver().lastTorque);
}

TEST_CASE_FIXTURE(AxisFixture, "the fade-in starts the torque at zero when the driver becomes ready") {
	setFullScale();
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(cf, 10000);

	run(1);
	int32_t first = axis().getTorque();
	run(499);
	int32_t half = axis().getTorque();

	CHECK(std::abs(first) < 50);
	CHECK_NEAR(half, 5000, 50); // linear ramp over 1 s
}

TEST_CASE_FIXTURE(AxisFixture, "an emergency stop sends zero torque and puts the driver into emergency stop") {
	setFullScale();
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(cf, 10000);
	run(1100);
	REQUIRE(driver().lastTorque != 0);

	axis().emergencyStop(false);

	CHECK(driver().lastTorque == 0);
	CHECK(driver().emergencyStopCalls == 1);
	CHECK(driver().emergency);
	CHECK(control.emergency);
}

TEST_CASE_FIXTURE(AxisFixture, "an axis restored from empty flash applies its default power of 5000 and effect ratio of 204" *
		KNOWN_ISSUE("Axis::torqueScaler and effectRatioScaler stay 0 until power / fxratio are set by command or "
					"restored from flash, although power reads 5000 and fxratio 204. No torque is output. "
					"The first CHECK on the driver torque fails")) {
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(cf, 10000);

	run(1100);

	// Expected: 10000 * fxratio 204/255 * power 5000/0x7fff, inverted
	CHECK_NEAR(driver().lastTorque, 10000.0 * 204.0 / 255.0 * 5000.0 / 32767.0, 2);
	CHECK(axisGet(Axis_commands::power) == 5000);
	CHECK(axisGet(Axis_commands::fxratio) == 204);
}

TEST_CASE_FIXTURE(TargetFixture, "the main class registry offers each main class of the F407 feature set once") {
	std::vector<uint16_t> ids;
	for (const class_entry<FFBoardMain> &e : class_registry) {
		ids.push_back(e.selectionId);
	}
	// FFBoardMain (failsafe), FFBWheel, FFBJoystick, FFBHIDExt, CanBridge, CANInputMain. No TMC debug bridge, no MIDI
	CHECK(ids == std::vector<uint16_t>{0, CLSID_MAIN_FFBWHEEL, CLSID_MAIN_FFBJOY, CLSID_MAIN_FFBEXT, CLSID_MAIN_CAN, CLSID_MAIN_CANINPUT});
}

/** Selects MockMotorDriver and MockEncoder on the axis created by a main class. Returns the encoder */
static MockEncoder &selectMocksOnMainclassAxis() {
	// The axis created by the AxesManager of the main class is found through the command handler registry
	Axis *axis = dynamic_cast<Axis *>(CommandHandler::getHandlerFromClassName("axis", 0));
	REQUIRE(axis != nullptr);
	axis->driverChooser = ClassChooser<MotorDriver>(AxisFixture::driverRegistry());
	axis->encoderChooser = ClassChooser<Encoder>(AxisFixture::encoderRegistry());
	axis->setDrvType(MockMotorDriver::SELECTION_ID);
	axis->setEncType(MockEncoder::SELECTION_ID);
	MockEncoder *encoder = dynamic_cast<MockEncoder *>(axis->getEncoder());
	REQUIRE(encoder != nullptr);
	return *encoder;
}

TEST_CASE_FIXTURE(TargetFixture, "an FFBWheel sends the position of its axis in the HID gamepad report when the FFB timer fires" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	MockEncoder &encoder = selectMocksOnMainclassAxis();
	encoder.setDegrees(90.0f); // 90 of the default 900 degrees: -0.2 of the range with the default inversion

	HostRtos::runUntilBlocked(wheel); // Run() starts TIM_FFB and waits for its update interrupt
	REQUIRE(HostTarget::timerRunning(&htim13));
	REQUIRE(HostTarget::timerUpdateIrqEnabled(&htim13));
	for (int i = 0; i < 5 && HostUsb::state().hidReports.empty(); i++) {
		HostPlatform::advanceMs(1);
		REQUIRE(HostTarget::timerUpdate(&htim13));
		HostRtos::runUntilBlocked(wheel);
	}

	REQUIRE(HostUsb::state().hidReports.size() == 1);
	const std::vector<uint8_t> &report = HostUsb::state().hidReports.back().data;
	REQUIRE(report.size() == 25); // id, 64 buttons, 8 x int16 axes
	CHECK(report[0] == 1);		  // gamepad report id
	int16_t x = 0;
	std::memcpy(&x, &report[9], sizeof(x));
	// getLastScaledEnc(): -0.2 * 0x7fffffff = -429496729 (float: -429496736), >> 16 for the 16 bit report = -6554
	CHECK(x == -6554);
}

TEST_CASE_FIXTURE(TargetFixture, "pressing button A of an FFBWheel zeros the axis position in the next update" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	MockEncoder &encoder = selectMocksOnMainclassAxis();
	encoder.setDegrees(30.0f);
	HostRtos::runUntilBlocked(wheel);
	encoder.setPosHistory.clear();

	HostTarget::setPin(BUTTON_A_GPIO_Port, BUTTON_A_Pin, true); // rising edge interrupt (MX_GPIO_Init)
	HostTarget::triggerExti(BUTTON_A_Pin);
	CHECK(encoder.setPosHistory.empty()); // only flagged in the interrupt
	REQUIRE(HostTarget::timerUpdate(&htim13));
	HostRtos::runUntilBlocked(wheel);

	CHECK(encoder.setPosHistory == std::vector<int32_t>{0});
	CHECK(encoder.getPos() == 0);
}

TEST_CASE("the HID gamepad report of a main class is deleted through a base class with a virtual destructor" *
		KNOWN_ISSUE("FFBHIDMain owns its HID_GamepadReport<T> as unique_ptr<HID_GamepadReport_base>, which has no "
					"virtual destructor: deleting a main class is undefined behaviour. The first CHECK fails")) {
	CHECK(std::has_virtual_destructor<HID_GamepadReport_base>::value);
}

} // TEST_SUITE
