/*
 * AxisFixture.h
 *
 * Fixtures of ffb_target_tests (F407 feature set, real Axis.h). Use with TEST_CASE_FIXTURE(...).
 *
 * TargetFixture  Resets the fake platform (FirmwareFixture) and the F407 part of it (HostTarget): timers,
 *                GPIO idle levels, chip ids, the CAN port, the fake HAL ADC / SPI / I2C and the global bus ports.
 *                Checks at the end that the test did not reach a HAL function or path without a fake
 *                (HostTarget::state().unmodelledHalCalls).
 *                That check is a CHECK in the fixture destructor. In a KNOWN_ISSUE (should_fail) test it would
 *                count as the expected failure, so such tests call REQUIRE_ALL_HAL_MODELLED() right before the
 *                assertion that describes the defect whenever they could reach a HAL call (see below).
 * AxisFixture    Real Axis objects with MockMotorDriver and MockEncoder selected through the public class
 *                choosers (Axis::driverChooser / Axis::encoderChooser), a real EffectsCalculator and HidFFB,
 *                and a simulated PID host. update() runs one FFB cycle the way FFBHIDMain::updateControl
 *                does through AxesManager.
 */
#ifndef HOSTTEST_AXISFIXTURE_H_
#define HOSTTEST_AXISFIXTURE_H_

#include "FirmwareFixture.h"
#include "HostTarget.h"
#include "MockAnalogSource.h"
#include "MockButtonSource.h"
#include "MockCANPort.h"
#include "MockEncoder.h"
#include "MockMotorDriver.h"
#include "PidHost.h"

#include "Axis.h" // The real one: ffb_target_tests has no fakes/ directory
#include "EffectsCalculator.h"
#include "FFBoardMain.h"
#include "HidFFB.h"
#include "USBdevice.h"

#include "doctest.h"

#include <memory>
#include <string>
#include <vector>

/**
 * Marks test cases that delete an FFBHIDMain main class (FFBWheel, FFBJoystick, FFBHIDExt).
 * Suspected firmware bug: FFBHIDMain::reportHID is a unique_ptr<HID_GamepadReport_base>, but
 * HID_GamepadReport_base (ffb_defs.h) has no virtual destructor. Deleting the HID_GamepadReport<T> through it is
 * undefined behaviour (ASan: new-delete-type-mismatch in ~FFBHIDMain). The test cases run and must pass in normal
 * builds; they are skipped in sanitizer builds. The defect itself is described by the KNOWN_ISSUE case
 * "the HID gamepad report of a main class is deleted through a base class with a virtual destructor".
 */
#ifdef HOSTTEST_SANITIZERS
#define DELETES_FFBHIDMAIN doctest::description("Skipped with sanitizers: deleting an FFBHIDMain is undefined behaviour (HID_GamepadReport_base has no virtual destructor)") * doctest::skip()
#else
#define DELETES_FFBHIDMAIN doctest::description("Deletes an FFBHIDMain: undefined behaviour (HID_GamepadReport_base has no virtual destructor), skipped with sanitizers")
#endif

/**
 * Fails the test (REQUIRE) if a HAL function without a fake was reached so far. Use it in KNOWN_ISSUE tests
 * before the assertion that describes the defect: the first failing assertion must be the defect, not the
 * fixture's unmodelled-HAL check at the end of the test.
 */
#define REQUIRE_ALL_HAL_MODELLED()                                                                                 \
	do {                                                                                                           \
		INFO("first HAL function without a fake: "                                                                 \
			 << (HostTarget::state().unmodelledHalCalls.empty() ? std::string() : HostTarget::state().unmodelledHalCalls.front())); \
		REQUIRE(HostTarget::state().unmodelledHalCalls.empty());                                                   \
	} while (0)

/** Access to the protected PersistentStorage::startupComplete (set by restoreFlashStartupCb, no public reset) */
struct PersistentStorageStartup : PersistentStorage {
	static void reset() { startupComplete = false; }
};

extern FFBoardMain *mainclass; // cppmain.cpp
extern USBdevice *usb_device;  // global_callbacks.cpp

struct TargetFixture : FirmwareFixture {
	TargetFixture() { resetTarget(); }
	~TargetFixture() override {
		const auto &calls = HostTarget::state().unmodelledHalCalls;
		INFO("first HAL function without a fake: " << (calls.empty() ? std::string() : calls.front()));
		CHECK_MESSAGE(calls.empty(), "the test reached HAL functions or HAL paths that have no fake (HostTarget::state().unmodelledHalCalls)");
		resetTarget();
	}

	/**
	 * Resets the F407 part: the main class created by HostTarget::runCppmain (deleted), HostTarget::reset (timers,
	 * GPIO, chip ids, canport, HAL handles, fake ADC and ADC buffers, TIM7 tick, IWDG, CAN receive FIFOs, globals of
	 * cppmain.cpp / voltagesense.cpp / ledEffects.cpp), PersistentStorage::startupComplete (false like after boot),
	 * MockMotorDriver::creatable, MockButtonSource::creatable, MockAnalogSource::creatable,
	 * SystemCommands::errorPrintingEnabled, the global HID handler, mainclass and usb_device. Also (resetTargetConfig):
	 * the fake HAL SPI / I2C, the global SPI ports and i2cport, the SPI / I2C handler lists. See
	 * HostTarget::resetTargetConfig for what is NOT reset.
	 */
	static void resetTarget() {
		HostTarget::deleteCppmainMainclass();
		HostTarget::reset();
		SystemCommands::errorPrintingEnabled = true;
		PersistentStorageStartup::reset();
		MockMotorDriver::creatable() = true;
		MockButtonSource::creatable() = true;
		MockAnalogSource::creatable() = true;
		UsbHidHandler::globalHidHandler = nullptr; // Not reset by the firmware when a handler is deleted
		mainclass = nullptr;
		usb_device = nullptr;
	}
};

struct AxisFixture : TargetFixture {
	/** Creates `axisCount` axes ('X', 'Y', ...) like AxesManager::setAxisCount and selects the mocks on each */
	explicit AxisFixture(uint8_t axisCount = 1) : ffb(effects, axisCount) {
		control.update_disabled = false; // AxesManager::setAxisCount enables updates
		// FFBWheel::usbInit (1 axis) uses the 2 axis HID descriptor: direction enable is bit 2 like for 2 axes
		ffb.setDirectionEnableMask(0x04);
		for (uint8_t i = 0; i < axisCount; i++) {
			axes.push_back(std::make_unique<Axis>('X' + i, &control));
			selectMocks(i);
		}
	}

	/** Driver registry offered to the axis: the firmware's "no driver" entry and the mock */
	static const std::vector<class_entry<MotorDriver>> &driverRegistry() {
		static const std::vector<class_entry<MotorDriver>> registry = {add_class<MotorDriver, MotorDriver>(0), MockMotorDriver::classEntry()};
		return registry;
	}
	/** Encoder registry offered to the axis: the firmware's "no encoder" entry and the mock */
	static const std::vector<class_entry<Encoder>> &encoderRegistry() {
		static const std::vector<class_entry<Encoder>> registry = {add_class<Encoder, Encoder>(0), MockEncoder::classEntry()};
		return registry;
	}

	/**
	 * Replaces the class choosers of an axis and selects MockMotorDriver and MockEncoder with the firmware's
	 * own setDrvType / setEncType (the same functions the drvtype / enctype commands use).
	 */
	void selectMocks(uint8_t i) {
		Axis &a = axis(i);
		a.driverChooser = ClassChooser<MotorDriver>(driverRegistry());
		a.encoderChooser = ClassChooser<Encoder>(encoderRegistry());
		a.setDrvType(MockMotorDriver::SELECTION_ID);
		a.setEncType(MockEncoder::SELECTION_ID);
	}

	Axis &axis(uint8_t i = 0) { return *axes.at(i); }
	/** The driver owned by the axis. Only valid while the mock is selected */
	MockMotorDriver &driver(uint8_t i = 0) {
		MockMotorDriver *drv = dynamic_cast<MockMotorDriver *>(axis(i).getDriver());
		REQUIRE(drv != nullptr);
		return *drv;
	}
	/** The encoder of the axis. Only valid while the mock is selected */
	MockEncoder &encoder(uint8_t i = 0) {
		MockEncoder *enc = dynamic_cast<MockEncoder *>(axis(i).getEncoder());
		REQUIRE(enc != nullptr);
		return *enc;
	}
	metric_t &metrics(uint8_t i = 0) { return *axis(i).getMetrics(); }

	/**
	 * One FFB cycle like FFBHIDMain::updateControl -> AxesManager::update / updateTorque:
	 * prepareForUpdate of all axes, effect calculation, then the drive torque unless the emergency stop is active.
	 * Does not advance the time.
	 * Not modelled from updateControl: control.request_update_disabled (moved to update_disabled there),
	 * control.resetEncoder (resetPosZero there), the HID report and the clip LED pulse during the emergency stop.
	 * Use a real main class (FFBWheel) to test those.
	 *
	 * Note for hands-off detection tests: Axis::updateHandsOffState (Axis.cpp:1339) keeps a function-local
	 * static check_counter that persists across axes and test cases and can not be reset. Run at least 11 cycles
	 * before asserting the hands-off state.
	 */
	void update() {
		if (control.update_disabled) {
			return;
		}
		for (auto &a : axes) {
			a->prepareForUpdate();
		}
		effects->calculateEffects(axes);
		if (!control.emergency) {
			for (auto &a : axes) {
				a->updateDriveTorque();
			}
		}
	}

	/** Runs `cycles` FFB cycles, advancing the simulated time by 1 ms before each (1 kHz update rate) */
	void run(uint32_t cycles) {
		for (uint32_t i = 0; i < cycles; i++) {
			HostPlatform::advanceMs(1);
			update();
		}
	}

	/** Executes an axis command like the command thread does. Replies are appended to `replies` if given */
	CommandStatus axisCommand(Axis_commands cmd, CMDtype type, int64_t val = 0, std::vector<CommandReply> *replies = nullptr, int64_t adr = 0, uint8_t i = 0) {
		std::vector<CommandReply> localReplies;
		ParsedCommand parsed;
		parsed.cmdId = static_cast<uint32_t>(cmd);
		parsed.type = type;
		parsed.val = val;
		parsed.adr = adr;
		parsed.target = &axis(i);
		return axis(i).command(parsed, replies ? *replies : localReplies);
	}

	/** Reads a value with a get command. Fails the test if the command does not return exactly one reply */
	int64_t axisGet(Axis_commands cmd, uint8_t i = 0) {
		std::vector<CommandReply> replies;
		CHECK(axisCommand(cmd, CMDtype::get, 0, &replies, 0, i) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies.front().val;
	}

	/**
	 * Full scale torque path: power 0x7fff (torque scaler 1.0), effect ratio 255 (100 %), like a user
	 * configuration sent by the configurator. An axis restored from empty flash has both scalers at 0.
	 */
	void setFullScale(uint8_t i = 0) {
		axisCommand(Axis_commands::power, CMDtype::set, 0x7fff, nullptr, 0, i);
		axisCommand(Axis_commands::fxratio, CMDtype::set, 255, nullptr, 0, i);
	}

	/** Creates, configures and starts an effect like a game does (see PidHost). @return effect block index */
	uint8_t startEffect(uint8_t type, const PidHost::EffectParams &params = PidHost::EffectParams()) {
		uint8_t index = host.allocateEffect(type);
		host.setEffect(index, type, params);
		host.startEffect(index);
		return index;
	}

	volatile Control_t control;												   //!< Global control flags (owned by FFBHIDMain on the target)
	std::shared_ptr<EffectsCalculator> effects = std::make_shared<EffectsCalculator>(); //!< Real effects calculator
	HidFFB ffb;																   //!< Real HID PID handler (registered as global HID handler)
	PidHost host;															   //!< Simulated PID driver of the host
	std::vector<std::unique_ptr<Axis>> axes;								   //!< Owned like AxesManager does
};

#endif
