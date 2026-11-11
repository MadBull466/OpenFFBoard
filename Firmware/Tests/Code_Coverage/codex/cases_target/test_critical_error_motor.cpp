/*
 * CDX-010: real FFBWheel -> timer/main task -> Axis -> recording motor driver.
 * The emergency API must stop a motor that is already applying force and keep
 * it stopped on subsequent control ticks (Axis::emergencyStop contract).
 * This checks the driver interface, not physical hardware or recovery policy.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "CommandHelpers.h"
#include "FFBWheel.h"
#include "TestHelpers.h"

namespace {
struct CriticalMotorFixture : TargetFixture {
	FFBWheel wheel;
	PidHost host;
	MockMotorDriver *motor = nullptr;

	void tick(unsigned count) {
		for (unsigned i = 0; i < count; ++i) {
			HostPlatform::advanceMs(1);
			REQUIRE(HostTarget::timerUpdate(&htim13));
			HostRtos::runUntilBlocked(wheel);
		}
	}

	void requireModelled() {
		REQUIRE_ALL_HAL_MODELLED();
		REQUIRE(HostRtos::kernelFaults().empty());
	}

	void applyForce() {
		// usbInit selects the wheel's HID direction mask; omitting it changes
		// this one-axis force's direction and invalidates the positive control.
		wheel.usbInit();
		auto *axis = dynamic_cast<Axis *>(CommandHandler::getHandlerFromClassName("axis", 0));
		REQUIRE(axis != nullptr);
		axis->driverChooser = ClassChooser<MotorDriver>(AxisFixture::driverRegistry());
		axis->encoderChooser = ClassChooser<Encoder>(AxisFixture::encoderRegistry());
		axis->setDrvType(MockMotorDriver::SELECTION_ID);
		axis->setEncType(MockEncoder::SELECTION_ID);
		motor = dynamic_cast<MockMotorDriver *>(axis->getDriver());
		REQUIRE(motor != nullptr);
		REQUIRE(dynamic_cast<MockEncoder *>(axis->getEncoder()) != nullptr);
		REQUIRE(CommandHelpers::run(*axis, "power", CMDtype::set, 0x7fff) == CommandStatus::OK);
		REQUIRE(CommandHelpers::run(*axis, "fxratio", CMDtype::set, 255) == CommandStatus::OK);
		host.deviceControl(0x01);
		auto effect = host.allocateEffect(FFB_EFFECT_CONSTANT);
		host.setEffect(effect, FFB_EFFECT_CONSTANT);
		host.setConstantForce(effect, 10000);
		host.startEffect(effect);
		HostRtos::runUntilBlocked(wheel);
		REQUIRE(HostTarget::timerRunning(&htim13));
		tick(1100); // past the 1 s startup fade; time is simulated
		requireModelled();
		// Full power and ratio imply approximately the requested 10000 units;
		// allow integer rounding. This prevents an idle/zero-force false pass.
		REQUIRE(motor->lastTorque > 9900);
		REQUIRE(motor->lastTorque <= 10000);
		REQUIRE_FALSE(motor->emergency);
	}

	void checkInhibited() {
		requireModelled();
		// FIRST defect assertion is motor-side. A flag-only repair leaves this
		// false and its previously applied torque latched at 9999.
		CHECK(motor->emergency);
		CHECK(motor->lastTorque == 0);
		CHECK(CommandHelpers::get(wheel, "estop") == 1);
		for (unsigned i = 0; i < 3; ++i) {
			tick(1);
			requireModelled();
			CHECK(motor->lastTorque == 0);
			CHECK(motor->emergency);
		}
	}
};
} // namespace

TEST_SUITE("CodexCriticalErrorMotor") {
TEST_CASE_FIXTURE(CriticalMotorFixture,
	"an explicit emergency command stops applied motor force through later ticks" * DELETES_FFBHIDMAIN) {
	applyForce();
	REQUIRE(CommandHelpers::run(wheel, "estop", CMDtype::set, 1) == CommandStatus::OK);
	checkInhibited();
}

TEST_CASE_FIXTURE(CriticalMotorFixture,
	"a new critical error stops applied motor force through later ticks"
	* KNOWN_ISSUE("bug 39 / CDX-010: FFBHIDMain::errorCallback resets emergency stop; the first failing CHECK is motor->emergency")
	* DELETES_FFBHIDMAIN) {
	applyForce();
	ErrorHandler::addError(Error(ErrorCode::overtemp, ErrorType::critical, "Codex motor inhibition"));
	checkInhibited();
}
}
