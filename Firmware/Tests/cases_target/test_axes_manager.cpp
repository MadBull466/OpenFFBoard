/*
 * Tests of AxesManager (FFBoard/Src/AxesManager.cpp): creation and deletion of axes by the axis count,
 * update / updateTorque over all axes, emergency stop, USB suspend and resume, zeroing, sample rate and the
 * wiring to the effects calculator.
 *
 * The axes are created by the manager itself (real Axis objects). They are reached through the command handler
 * registry ("axis", instance 0 = X, 1 = Y) like the configurator does, and MockMotorDriver / MockEncoder are
 * selected through their public class choosers.
 *
 * AxesManager has no commands of its own: the "axes" command of FFBHIDMain is commented out
 * (FFBHIDMain_commmands.cpp:51-57). The commands of the managed axes are tested here only as far as they show
 * that the manager created and registered them.
 */
#include "doctest.h"
#include "AxisFixture.h"

#include "AxesManager.h"

#include <cmath>
#include <memory>
#include <vector>

namespace {

struct AxesManagerFixture : TargetFixture {
	/** The axis with the given instance as found by the command system, nullptr if there is none */
	static Axis *registeredAxis(uint8_t instance) {
		return dynamic_cast<Axis *>(CommandHandler::getHandlerFromClassName("axis", instance));
	}

	/** Amount of axis command handlers currently registered */
	static size_t registeredAxisCount() { return CommandHandler::getHandlersFromClassName("axis").size(); }

	Axis &axis(uint8_t instance) {
		Axis *a = registeredAxis(instance);
		REQUIRE(a != nullptr);
		return *a;
	}

	/** Selects MockMotorDriver and MockEncoder on a managed axis (same as AxisFixture::selectMocks) */
	void selectMocks(uint8_t instance) {
		Axis &a = axis(instance);
		a.driverChooser = ClassChooser<MotorDriver>(AxisFixture::driverRegistry());
		a.encoderChooser = ClassChooser<Encoder>(AxisFixture::encoderRegistry());
		a.setDrvType(MockMotorDriver::SELECTION_ID);
		a.setEncType(MockEncoder::SELECTION_ID);
	}

	MockMotorDriver &driver(uint8_t instance) {
		MockMotorDriver *drv = dynamic_cast<MockMotorDriver *>(axis(instance).getDriver());
		REQUIRE(drv != nullptr);
		return *drv;
	}
	MockEncoder &encoder(uint8_t instance) {
		MockEncoder *enc = dynamic_cast<MockEncoder *>(axis(instance).getEncoder());
		REQUIRE(enc != nullptr);
		return *enc;
	}

	CommandStatus axisCommand(uint8_t instance, Axis_commands cmd, CMDtype type, int64_t val = 0, std::vector<CommandReply> *replies = nullptr) {
		std::vector<CommandReply> local;
		ParsedCommand parsed;
		parsed.cmdId = static_cast<uint32_t>(cmd);
		parsed.type = type;
		parsed.val = val;
		parsed.target = &axis(instance);
		return axis(instance).command(parsed, replies ? *replies : local);
	}

	/** Power 0x7fff and effect ratio 255: torque scaler and effect ratio 1.0; internal damper off */
	void setFullScale(uint8_t instance) {
		axisCommand(instance, Axis_commands::power, CMDtype::set, 0x7fff);
		axisCommand(instance, Axis_commands::fxratio, CMDtype::set, 255);
		axisCommand(instance, Axis_commands::axisdamper, CMDtype::set, 0);
	}

	/** Creates `count` axes with the mocks selected and full scale settings */
	void createAxesWithMocks(int8_t count) {
		REQUIRE(manager->setAxisCount(count));
		for (uint8_t i = 0; i < count; i++) {
			selectMocks(i);
			setFullScale(i);
		}
	}

	/** One FFB cycle like FFBHIDMain::updateControl without emergency stop: 1 ms later, update, updateTorque */
	void cycle(uint32_t n = 1) {
		for (uint32_t i = 0; i < n; i++) {
			HostPlatform::advanceMs(1);
			manager->update();
			manager->updateTorque();
		}
	}

	/** Starts an infinite constant force in direction 90 deg (X axis only) */
	uint8_t startConstantForce(int16_t magnitude) {
		uint8_t index = host.allocateEffect(FFB_EFFECT_CONSTANT);
		host.setEffect(index, FFB_EFFECT_CONSTANT);
		host.startEffect(index);
		host.setConstantForce(index, magnitude);
		return index;
	}

	volatile Control_t control;
	std::shared_ptr<EffectsCalculator> effects = std::make_shared<EffectsCalculator>();
	HidFFB ffb{effects, 2}; // 2 axis descriptor: direction enable bit 0x04
	PidHost host;
	std::unique_ptr<AxesManager> manager = std::make_unique<AxesManager>(&control, effects);
};

} // namespace

TEST_SUITE("AxesManager") {

// ---------------------------------------------------------------------------
// Axis count
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(AxesManagerFixture, "a new axes manager has no axes") {
	CHECK(manager->getAxisCount() == 0);
	CHECK(registeredAxisCount() == 0);
	CHECK(manager->getAxisValues()->empty());
	CHECK(control.update_disabled); // Control_t default, not touched before an axis count is set
}

TEST_CASE_FIXTURE(AxesManagerFixture, "the axis counts from 1 to MAX_AXIS are valid") {
	CHECK_FALSE(manager->validAxisRange(0));
	for (uint8_t count = 1; count <= MAX_AXIS; count++) {
		CAPTURE(count);
		CHECK(manager->validAxisRange(count));
	}
	CHECK_FALSE(manager->validAxisRange(MAX_AXIS + 1));
	CHECK_FALSE(manager->validAxisRange(255));
}

TEST_CASE_FIXTURE(AxesManagerFixture, "setting one axis creates the X axis as axis instance 0 and enables the updates") {
	CHECK(manager->setAxisCount(1));

	CHECK(manager->getAxisCount() == 1);
	CHECK(registeredAxisCount() == 1);
	REQUIRE(registeredAxis(0) != nullptr);
	CHECK(registeredAxis(1) == nullptr);
	CHECK_FALSE(control.update_disabled);
}

TEST_CASE_FIXTURE(AxesManagerFixture, "setting two axes creates the X and Y axes as axis instances 0 and 1") {
	CHECK(manager->setAxisCount(2));

	CHECK(manager->getAxisCount() == 2);
	CHECK(registeredAxisCount() == 2);
	CHECK(registeredAxis(0) != nullptr);
	CHECK(registeredAxis(1) != nullptr);
	CHECK(registeredAxis(0) != registeredAxis(1));
}

TEST_CASE_FIXTURE(AxesManagerFixture, "an axis count outside 1 to MAX_AXIS is rejected and leaves the axes unchanged") {
	REQUIRE(manager->setAxisCount(1));
	Axis *x = registeredAxis(0);
	control.update_disabled = true;

	for (int8_t count : {(int8_t)0, (int8_t)(MAX_AXIS + 1), (int8_t)-1, (int8_t)127}) {
		CAPTURE((int)count);
		CHECK_FALSE(manager->setAxisCount(count));
		CHECK(manager->getAxisCount() == 1);
		CHECK(registeredAxisCount() == 1);
		CHECK(registeredAxis(0) == x);
	}
	CHECK(control.update_disabled); // Rejected counts do not enable the updates
}

TEST_CASE_FIXTURE(AxesManagerFixture, "a rejected axis count does not create axes on a new manager") {
	CHECK_FALSE(manager->setAxisCount(0));
	CHECK(manager->getAxisCount() == 0);
	CHECK(registeredAxisCount() == 0);
	CHECK(control.update_disabled);
}

TEST_CASE_FIXTURE(AxesManagerFixture, "reducing the axis count deletes the last axis and keeps the first") {
	REQUIRE(manager->setAxisCount(2));
	Axis *x = registeredAxis(0);

	CHECK(manager->setAxisCount(1));

	CHECK(manager->getAxisCount() == 1);
	CHECK(registeredAxisCount() == 1);
	CHECK(registeredAxis(0) == x);
	CHECK(registeredAxis(1) == nullptr);
}

TEST_CASE_FIXTURE(AxesManagerFixture, "increasing the axis count keeps the existing axis with its settings") {
	REQUIRE(manager->setAxisCount(1));
	Axis *x = registeredAxis(0);
	axisCommand(0, Axis_commands::power, CMDtype::set, 1234);

	CHECK(manager->setAxisCount(2));

	CHECK(registeredAxis(0) == x);
	std::vector<CommandReply> replies;
	axisCommand(0, Axis_commands::power, CMDtype::get, 0, &replies);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 1234);
	CHECK(registeredAxis(1) != nullptr);
}

TEST_CASE_FIXTURE(AxesManagerFixture, "setting the current axis count again keeps the axes") {
	REQUIRE(manager->setAxisCount(2));
	Axis *x = registeredAxis(0);
	Axis *y = registeredAxis(1);

	CHECK(manager->setAxisCount(2));

	CHECK(registeredAxis(0) == x);
	CHECK(registeredAxis(1) == y);
	CHECK(registeredAxisCount() == 2);
}

TEST_CASE_FIXTURE(AxesManagerFixture, "deleting the axes manager deletes its axes and their drivers") {
	createAxesWithMocks(2);
	REQUIRE(MockMotorDriver::instances() == 2);

	manager.reset();

	CHECK(registeredAxisCount() == 0);
	CHECK(MockMotorDriver::instances() == 0);
}

TEST_CASE_FIXTURE(AxesManagerFixture, "the axes created by the manager answer axis commands") {
	REQUIRE(manager->setAxisCount(2));
	axisCommand(1, Axis_commands::power, CMDtype::set, 2000);

	std::vector<CommandReply> x, y;
	CHECK(axisCommand(0, Axis_commands::power, CMDtype::get, 0, &x) == CommandStatus::OK);
	CHECK(axisCommand(1, Axis_commands::power, CMDtype::get, 0, &y) == CommandStatus::OK);
	REQUIRE(x.size() == 1);
	REQUIRE(y.size() == 1);
	CHECK(x[0].val == 5000); // default
	CHECK(y[0].val == 2000);
}

// ---------------------------------------------------------------------------
// update / updateTorque
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(AxesManagerFixture, "update reads the encoder of every axis") {
	createAxesWithMocks(2);
	encoder(0).setDegrees(90.0f);
	encoder(1).setDegrees(-45.0f);

	HostPlatform::advanceMs(1);
	manager->update();

	// Axes are inverted by default
	CHECK(axis(0).getMetrics()->posDegrees == doctest::Approx(-90.0).epsilon(1e-4));
	CHECK(axis(1).getMetrics()->posDegrees == doctest::Approx(45.0).epsilon(1e-4));
}

TEST_CASE_FIXTURE(AxesManagerFixture, "update without updateTorque sends no torque to the drivers") {
	createAxesWithMocks(2);
	startConstantForce(10000);
	driver(0).torqueHistory.clear();
	driver(1).torqueHistory.clear();

	for (int i = 0; i < 1100; i++) {
		HostPlatform::advanceMs(1);
		manager->update();
	}

	CHECK(driver(0).torqueHistory.empty());
	CHECK(driver(1).torqueHistory.empty());
}

TEST_CASE_FIXTURE(AxesManagerFixture, "updateTorque sends the effect torque calculated in update to the driver of each axis") {
	createAxesWithMocks(2);
	uint8_t index = host.allocateEffect(FFB_EFFECT_CONSTANT);
	PidHost::EffectParams params;
	params.enableAxis = PidHost::AXIS_X | PidHost::AXIS_Y | PidHost::DIRECTION_ENABLE_2AXIS;
	params.directionX = 13500; // 135 deg: sin = 0.707 on X, -cos = 0.707 on Y
	host.setEffect(index, FFB_EFFECT_CONSTANT, params);
	host.startEffect(index);
	host.setConstantForce(index, 10000);

	cycle(1100); // fade-in of 1 s after the driver became ready

	// EffectsCalculator: -10000 * 0.707 per axis; inverted by the axes
	CHECK(driver(0).lastTorque == doctest::Approx(7071).epsilon(0.002));
	CHECK(driver(1).lastTorque == doctest::Approx(7071).epsilon(0.002));
}

TEST_CASE_FIXTURE(AxesManagerFixture, "update without effects calculator still reads the encoders") {
	manager = std::make_unique<AxesManager>(&control, nullptr);
	createAxesWithMocks(1);
	encoder(0).setDegrees(36.0f); // 1000 counts

	HostPlatform::advanceMs(1);
	manager->update();
	manager->updateTorque();

	CHECK(axis(0).getMetrics()->posDegrees == doctest::Approx(-36.0).epsilon(1e-4));
}

TEST_CASE_FIXTURE(AxesManagerFixture, "the effects calculator gives the idle spring to all axes while FFB is off") {
	createAxesWithMocks(2);
	encoder(0).setDegrees(90.0f);
	encoder(1).setDegrees(-90.0f);

	cycle(1100);

	REQUIRE_FALSE(effects->isActive());
	// Idle spring default strength 127: clipped to 127 * 35 = 4445. Restoring: torque opposes the encoder position
	CHECK(driver(0).lastTorque == -4445);
	CHECK(driver(1).lastTorque == 4445);
}

// ---------------------------------------------------------------------------
// Axis values for the HID report
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(AxesManagerFixture, "the axis values contain the scaled position of every axis with an encoder") {
	createAxesWithMocks(2);
	axisCommand(0, Axis_commands::invert, CMDtype::set, 0);
	axisCommand(1, Axis_commands::invert, CMDtype::set, 0);
	encoder(0).setDegrees(90.0f);  // 0.2 of the 900 deg range
	encoder(1).setDegrees(-225.0f); // -0.5
	HostPlatform::advanceMs(1);
	manager->update();

	std::vector<int32_t> *values = manager->getAxisValues();

	REQUIRE(values->size() == 2);
	CHECK((*values)[0] == axis(0).getLastScaledEnc());
	CHECK((*values)[1] == axis(1).getLastScaledEnc());
	CHECK((double)(*values)[0] == doctest::Approx(0.2 * 0x7fffffff).epsilon(1e-5));
	CHECK((double)(*values)[1] == doctest::Approx(-0.5 * 0x7fffffff).epsilon(1e-5));
}

TEST_CASE_FIXTURE(AxesManagerFixture, "the axis values are rebuilt on every call") {
	createAxesWithMocks(1);
	CHECK(manager->getAxisValues()->size() == 1);
	CHECK(manager->getAxisValues()->size() == 1);
}

TEST_CASE_FIXTURE(AxesManagerFixture, "characterization: axes without an encoder have no axis value") {
	// NEEDS REVIEW #25: an axis without encoder is left out of the axis values instead of reporting 0 in its slot
	REQUIRE(manager->setAxisCount(1)); // Empty flash: no driver, no encoder
	CHECK(manager->getAxisValues()->empty());
}

TEST_CASE_FIXTURE(AxesManagerFixture, "characterization: an axis without encoder is left out and the next axis takes its place in the axis values") {
	// NEEDS REVIEW #25: with X without encoder the Y position is reported in the first (X) slot of the HID gamepad report
	REQUIRE(manager->setAxisCount(2));
	selectMocks(1);
	axisCommand(1, Axis_commands::invert, CMDtype::set, 0);
	encoder(1).setDegrees(90.0f);
	HostPlatform::advanceMs(1);
	manager->update();

	std::vector<int32_t> *values = manager->getAxisValues();

	REQUIRE(values->size() == 1);
	CHECK((*values)[0] == axis(1).getLastScaledEnc());
	CHECK((*values)[0] != 0);
}

// ---------------------------------------------------------------------------
// Emergency stop, USB suspend / resume, zeroing, sample rate
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(AxesManagerFixture, "an emergency stop puts the drivers of all axes into emergency stop") {
	createAxesWithMocks(2);
	encoder(0).setDegrees(90.0f);
	encoder(1).setDegrees(90.0f);
	cycle(1100); // Idle spring torque on both
	REQUIRE(driver(0).lastTorque != 0);
	REQUIRE(driver(1).lastTorque != 0);

	manager->emergencyStop(false);

	for (uint8_t i = 0; i < 2; i++) {
		CAPTURE((int)i);
		// End state only: Axis::emergencyStop sends 0, but MotorDriver::emergencyStop (base of the mock) also turns 0
		CHECK(driver(i).lastTorque == 0);
		CHECK(driver(i).emergencyStopCalls == 1);
		CHECK(driver(i).emergency);
	}
	CHECK(control.emergency);
}

TEST_CASE_FIXTURE(AxesManagerFixture, "resetting the emergency stop releases all drivers and fades the torque in again") {
	createAxesWithMocks(2);
	encoder(0).setDegrees(90.0f);
	encoder(1).setDegrees(90.0f);
	cycle(1100);
	int16_t before = driver(0).lastTorque;
	manager->emergencyStop(false);

	manager->emergencyStop(true);
	CHECK_FALSE(control.emergency);
	for (uint8_t i = 0; i < 2; i++) {
		CAPTURE((int)i);
		CHECK(driver(i).emergencyStopCalls == 2);
		CHECK_FALSE(driver(i).emergency);
	}

	cycle(1);
	CHECK(std::abs(driver(0).lastTorque) < std::abs(before) / 100); // fade-in starts at 0
	// Documented fade 0.5 s; the firmware takes 2 s (bug 30); 2100 cycles cover both
	cycle(2100);
	CHECK(driver(0).lastTorque == before);
	CHECK(driver(1).lastTorque == before);
}

TEST_CASE_FIXTURE(AxesManagerFixture, "a USB suspend stops the motors of all axes") {
	createAxesWithMocks(2);
	encoder(0).setDegrees(90.0f);
	encoder(1).setDegrees(90.0f);
	cycle(1100);
	uint32_t stopsBefore[2] = {driver(0).stopCalls, driver(1).stopCalls};

	manager->usbSuspend();

	for (uint8_t i = 0; i < 2; i++) {
		CAPTURE((int)i);
		// End state only: Axis::usbSuspend sends 0, but MotorDriver::stopMotor (base of the mock) also turns 0
		CHECK(driver(i).lastTorque == 0);
		CHECK_FALSE(driver(i).running);
		CHECK(driver(i).stopCalls == stopsBefore[i] + 1);
	}
}

TEST_CASE_FIXTURE(AxesManagerFixture, "a USB resume starts the motors of all axes and fades the torque in") {
	createAxesWithMocks(2);
	encoder(0).setDegrees(90.0f);
	encoder(1).setDegrees(90.0f);
	cycle(1100);
	int16_t before = driver(1).lastTorque;
	manager->usbSuspend();
	uint32_t startsBefore = driver(1).startCalls;

	manager->usbResume();

	for (uint8_t i = 0; i < 2; i++) {
		CAPTURE((int)i);
		CHECK(driver(i).running);
		CHECK(driver(i).startCalls == startsBefore + 1);
	}
	cycle(1);
	CHECK(std::abs(driver(1).lastTorque) < std::abs(before) / 100);
	// Documented fade 0.5 s; the firmware takes 2 s (bug 30); 2100 cycles cover both
	cycle(2100);
	CHECK(driver(1).lastTorque == before);
}

TEST_CASE_FIXTURE(AxesManagerFixture, "resetting the position zeros the encoders of all axes") {
	createAxesWithMocks(2);
	encoder(0).setDegrees(30.0f);
	encoder(1).setDegrees(-60.0f);

	manager->resetPosZero();
	cycle(1);

	for (uint8_t i = 0; i < 2; i++) {
		CAPTURE((int)i);
		CHECK(encoder(i).setPosHistory == std::vector<int32_t>{0});
		CHECK(encoder(i).getPos() == 0);
		CHECK(axis(i).getMetrics()->posDegrees == doctest::Approx(0.0));
	}
}

TEST_CASE_FIXTURE(AxesManagerFixture, "resetting the position fades the torque in from a quarter") {
	createAxesWithMocks(1);
	startConstantForce(10000);
	cycle(1100);
	REQUIRE(driver(0).lastTorque == doctest::Approx(10000).epsilon(0.001));

	manager->resetPosZero();
	cycle(1);

	CHECK(driver(0).lastTorque == doctest::Approx(2500).epsilon(0.01)); // startForceFadeIn(0.25, 0.5)
}

TEST_CASE_FIXTURE(AxesManagerFixture, "a new sample rate is used by all axes for the speed") {
	createAxesWithMocks(2);
	for (uint8_t i = 0; i < 2; i++) {
		encoder(i).setCpr(3600000); // 0.0001 deg per count: no quantisation of the 0.1 deg steps
	}

	manager->updateSamplerate(500.0f);
	for (int n = 1; n <= 300; n++) {
		encoder(0).setRawPosition(n * 1000);  // +0.1 deg per update
		encoder(1).setRawPosition(-n * 1000); // -0.1 deg per update
		cycle(1);
	}

	// 0.1 deg per update at 500 updates per second = 50 deg/s, inverted by default
	CHECK(axis(0).getMetrics()->speed == doctest::Approx(-50.0).epsilon(0.01));
	CHECK(axis(1).getMetrics()->speed == doctest::Approx(50.0).epsilon(0.01));
}

} // TEST_SUITE
