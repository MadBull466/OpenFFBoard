/*
 * Tests of the main classes: FFBoardMain (base / failsafe), CustomMain (example), the FFBHIDMain variants FFBWheel,
 * FFBJoystick, FFBHIDExt (what differs between them: axis count, USB descriptors, FFB interface) and the main class
 * registry (mainclass_chooser.cpp).
 *
 * The shared FFBHIDMain behaviour (report, rate, commands, estop) is tested in test_ffbhidmain.cpp with FFBWheel.
 * Every test that deletes an FFBHIDMain-derived class carries DELETES_FFBHIDMAIN (suspected bug 24).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "CustomMain.h"
#include "FFBHIDExt.h"
#include "FFBJoystick.h"
#include "FFBWheel.h"
#include "mainclass_chooser.h"
#include "usb_descriptors.h"
#include "usb_hid_ffb_desc.h"

#include <cstring>
#include <memory>
#include <set>
#include <string>
#include <vector>

extern ClassChooser<FFBoardMain> mainchooser; // cppmain.cpp (host_stubs_target.cpp)

namespace {

struct MainClassFixture : TargetFixture {
	/** Executes a command of `handler` by its name like the command thread does */
	static CommandStatus cmd(CommandHandler &handler, const std::string &name, CMDtype type, int64_t val = 0,
							 std::vector<CommandReply> *replies = nullptr, int64_t adr = 0) {
		CmdHandlerCommanddef *def = handler.getCommandFromName(name);
		REQUIRE_MESSAGE(def != nullptr, "command " << name << " not registered");
		ParsedCommand parsed;
		parsed.cmdId = def->cmdId;
		parsed.type = type;
		parsed.val = val;
		parsed.adr = adr;
		parsed.target = &handler;
		std::vector<CommandReply> local;
		return handler.command(parsed, replies ? *replies : local);
	}
	static int64_t get(CommandHandler &handler, const std::string &name) {
		std::vector<CommandReply> replies;
		CHECK(cmd(handler, name, CMDtype::get, 0, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies.front().val;
	}

	/** Selects MockMotorDriver and MockEncoder on an axis of the main class. Returns the encoder */
	static MockEncoder &selectMocks(uint8_t instance) {
		Axis *axis = dynamic_cast<Axis *>(CommandHandler::getHandlerFromClassName("axis", instance));
		REQUIRE(axis != nullptr);
		axis->driverChooser = ClassChooser<MotorDriver>(AxisFixture::driverRegistry());
		axis->encoderChooser = ClassChooser<Encoder>(AxisFixture::encoderRegistry());
		axis->setDrvType(MockMotorDriver::SELECTION_ID);
		axis->setEncType(MockEncoder::SELECTION_ID);
		MockEncoder *encoder = dynamic_cast<MockEncoder *>(axis->getEncoder());
		REQUIRE(encoder != nullptr);
		return *encoder;
	}

	/** Runs the USB device thread started by registerUsb() (USBdevice::Run initialises TinyUSB and waits for events) */
	static void runUsbThread() {
		TaskHandle_t usb = HostRtos::findTask("USB");
		REQUIRE(usb != nullptr);
		HostRtos::runUntilBlocked(usb);
	}

	/** Gamepad reports (report id 1) */
	static std::vector<std::vector<uint8_t>> gamepadReports() {
		std::vector<std::vector<uint8_t>> reports;
		for (const HostUsb::HidReport &r : HostUsb::state().hidReports) {
			if (!r.data.empty() && r.data[0] == 1) {
				reports.push_back(r.data);
			}
		}
		return reports;
	}
	static int16_t axis16(const std::vector<uint8_t> &report, int i) {
		int16_t v = 0;
		REQUIRE(report.size() == 25);
		std::memcpy(&v, &report[9 + 2 * i], sizeof(v));
		return v;
	}
};

} // namespace

// =====================================================================================================================
TEST_SUITE("FFBoardMain") {

TEST_CASE_FIXTURE(MainClassFixture, "the failsafe main class has id 0 and the name Basic (Failsafe)") {
	FFBoardMain main;
	CHECK(main.getInfo().id == 0);
	CHECK(std::string(main.getInfo().name) == "Basic (Failsafe)");
	CHECK(main.getClassType() == ClassType::Mainclass);
	CHECK(FFBoardMain::isCreatable());
}

TEST_CASE_FIXTURE(MainClassFixture, "the failsafe main class registers the command handler main with the internal commands") {
	FFBoardMain main;
	CHECK(CommandHandler::getHandlerFromClassName("main", 0) == &main);
	CHECK(main.getCommandFromName("id") != nullptr);
	CHECK(main.getCommandFromName("name") != nullptr);
	CHECK(main.getCommandFromName("help") != nullptr);
}

TEST_CASE_FIXTURE(MainClassFixture, "the failsafe main class has no own commands") {
	FFBoardMain main;
	std::vector<CommandReply> replies;
	ParsedCommand parsed;
	parsed.cmdId = 0;
	parsed.type = CMDtype::get;
	parsed.target = &main;

	CHECK(main.command(parsed, replies) == CommandStatus::NOT_FOUND);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(MainClassFixture, "the failsafe help string tells the user to choose another main class") {
	FFBoardMain main;
	CHECK(main.getHelpstring() == "Failsafe mainclass with no features. Choose a different mainclass. sys.lsmain to get a list");
}

TEST_CASE_FIXTURE(MainClassFixture, "usbInit of the failsafe main class registers a CDC-only USB device") {
	FFBoardMain main;

	main.usbInit();

	REQUIRE(usb_device != nullptr);
	CHECK(usb_device->getUsbConfigurationDesc(0) == usb_cdc_conf);
	CHECK(usb_device->getUsbDeviceDesc() == reinterpret_cast<const uint8_t *>(&usb_devdesc_ffboard_composite));
	runUsbThread();
	CHECK(HostTarget::state().usbInits == 1);
}

TEST_CASE_FIXTURE(MainClassFixture, "the default update of a main class waits 1 ms") {
	FFBoardMain main;
	uint64_t before = HostPlatform::nowUs();

	main.update();

	CHECK(HostPlatform::nowUs() - before == 1000);
}

TEST_CASE_FIXTURE(MainClassFixture, "USB suspend and resume of the failsafe main class send nothing") {
	FFBoardMain main;

	main.usbSuspend();
	main.usbResume();

	CHECK(HostUsb::state().hidReports.empty());
	CHECK(HostUsb::state().cdcTx.empty());
}

TEST_CASE_FIXTURE(MainClassFixture, "a command received on the CDC port is executed and answered on the CDC port") {
	FFBoardMain main;
	std::string line = "main.id?\n";
	uint32_t len = line.size();

	main.cdcRcv(line.data(), &len);
	HostRtos::runUntilIdle();

	CHECK(HostUsb::state().cdcTx == "[main.id?|0]\n");
}

TEST_CASE_FIXTURE(MainClassFixture, "data read from the CDC port is executed as a command") {
	FFBoardMain main;
	HostUsb::cdcReceive("main.id?\n");

	main.cdcRcvReady(0);
	HostRtos::runUntilIdle();

	CHECK(HostUsb::state().cdcRx.empty());
	CHECK(HostUsb::state().cdcTx == "[main.id?|0]\n");
}

TEST_CASE_FIXTURE(MainClassFixture, "one CDC read takes at most the 64 bytes of the full speed CDC buffer") {
	FFBoardMain main;
	REQUIRE(main.cdcCmdInterface->bufferCapacity() > 64);
	HostUsb::cdcReceive(std::string(100, ' '));

	main.cdcRcvReady(0);

	CHECK(HostUsb::state().cdcRx.size() == 100 - 64);
}

TEST_CASE_FIXTURE(MainClassFixture, "one CDC read takes at most the free space of the command buffer") {
	FFBoardMain main;
	std::string pending(1000, 'x'); // an unfinished command fills the parser buffer
	uint32_t len = pending.size();
	main.cdcRcv(pending.data(), &len);
	uint32_t capacity = main.cdcCmdInterface->bufferCapacity();
	REQUIRE(capacity < 64);
	HostUsb::cdcReceive(std::string(64, 'y'));

	main.cdcRcvReady(0);

	CHECK(HostUsb::state().cdcRx.size() == 64 - capacity);
}

} // TEST_SUITE FFBoardMain

// =====================================================================================================================
TEST_SUITE("CustomMain") {

TEST_CASE_FIXTURE(MainClassFixture, "the custom main class is a hidden class with the reserved custom id") {
	CustomMain main;
	CHECK(main.getInfo().id == CLSID_CUSTOM);
	CHECK(std::string(main.getInfo().name) == "Custom");
	CHECK(main.getInfo().visibility == ClassVisibility::hidden);
}

TEST_CASE_FIXTURE(MainClassFixture, "the custom main class has its own help string") {
	CustomMain main;
	CHECK(main.getHelpstring() == "This is a basic example mainclass with commands");
}

TEST_CASE_FIXTURE(MainClassFixture, "the example command stores a value and returns it") {
	CustomMain main;
	CHECK(get(main, "command") == 0);

	CHECK(cmd(main, "command", CMDtype::set, -1234) == CommandStatus::OK);

	CHECK(get(main, "command") == -1234);
}

TEST_CASE_FIXTURE(MainClassFixture, "an unknown command of the custom main class is not found") {
	CustomMain main;
	std::vector<CommandReply> replies;
	ParsedCommand parsed;
	parsed.cmdId = 7;
	parsed.type = CMDtype::get;
	parsed.target = &main;
	CHECK(main.command(parsed, replies) == CommandStatus::NOT_FOUND);
}

TEST_CASE_FIXTURE(MainClassFixture, "the custom main class keeps the internal commands registered once") {
	CustomMain main; // FFBoardMain and CustomMain::registerCommands both register the internal commands
	const std::string help = main.getCommandsHelpstring();
	INFO(help);
	size_t ids = 0;
	for (size_t pos = 0; (pos = help.find("\nid", pos)) != std::string::npos; pos++) {
		ids++;
	}
	if (help.rfind("id", 0) == 0) {
		ids++;
	}
	CHECK(ids == 1);
	CHECK(main.getCommandFromName("command") != nullptr);
}

TEST_CASE_FIXTURE(MainClassFixture, "the custom main class is not in the class registry of the F407 feature set") {
	CHECK_FALSE(mainchooser.isValidClassId(CLSID_CUSTOM));
}

} // TEST_SUITE CustomMain

// =====================================================================================================================
TEST_SUITE("FFBWheel") {

TEST_CASE_FIXTURE(MainClassFixture, "the wheel has id 1 and the name FFB Wheel (1 Axis)" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	CHECK(wheel.getInfo().id == CLSID_MAIN_FFBWHEEL);
	CHECK(std::string(wheel.getInfo().name) == "FFB Wheel (1 Axis)");
	CHECK(FFBWheel::info.id == CLSID_MAIN_FFBWHEEL);
}

TEST_CASE_FIXTURE(MainClassFixture, "the wheel creates one axis" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	CHECK(CommandHandler::getHandlersFromClassName("axis").size() == 1);
}

TEST_CASE_FIXTURE(MainClassFixture, "the wheel receives the PID reports of the host with its HID FFB handler" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	PidHost host;
	REQUIRE(UsbHidHandler::globalHidHandler != nullptr);
	CHECK(dynamic_cast<HidFFB *>(UsbHidHandler::globalHidHandler) != nullptr);

	host.deviceControl(0x01);

	CHECK(wheel.getFfbActive());
}

TEST_CASE_FIXTURE(MainClassFixture, "usbInit of the wheel uses the 2 axis FFB descriptors with 16 bit axes" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;

	wheel.usbInit();

	CHECK(UsbHidHandler::getHidDesc() == hid_2ffb_desc);
	REQUIRE(usb_device != nullptr);
	CHECK(usb_device->getUsbConfigurationDesc(0) == usb_cdc_hid_conf_2axis);
	runUsbThread();
	CHECK(HostTarget::state().usbInits == 1);
}

/** Constant force 10000 in direction 90 deg (PidHost defaults: X axis and direction enable bit 0x04) at full scale.
 *  Returns the driver torque after the 1 s fade-in */
static int16_t wheelTorqueForEastForce(FFBWheel &wheel) {
	MainClassFixture::selectMocks(0);
	Axis *axis = dynamic_cast<Axis *>(CommandHandler::getHandlerFromClassName("axis", 0));
	REQUIRE(axis != nullptr);
	REQUIRE(MainClassFixture::cmd(*axis, "power", CMDtype::set, 0x7fff) == CommandStatus::OK);
	REQUIRE(MainClassFixture::cmd(*axis, "fxratio", CMDtype::set, 255) == CommandStatus::OK);
	PidHost host;
	host.deviceControl(0x01);
	uint8_t cf = host.allocateEffect(FFB_EFFECT_CONSTANT);
	host.setEffect(cf, FFB_EFFECT_CONSTANT);
	host.startEffect(cf);
	host.setConstantForce(cf, 10000);
	HostRtos::runUntilBlocked(wheel); // starts the FFB timer
	for (int i = 0; i < 1100; i++) {
		HostPlatform::advanceMs(1);
		REQUIRE(HostTarget::timerUpdate(&htim13));
		HostRtos::runUntilBlocked(wheel);
	}
	MockMotorDriver *drv = dynamic_cast<MockMotorDriver *>(axis->getDriver());
	REQUIRE(drv != nullptr);
	return drv->lastTorque;
}

TEST_CASE_FIXTURE(MainClassFixture, "after usbInit a constant force in direction 90 deg acts on the wheel at full strength" * DELETES_FFBHIDMAIN) {
	// FFBWheel::usbInit sets the direction enable bit 0x04 of the 2 axis descriptor (HidFFB default for 1 axis: 0x02)
	FFBWheel wheel;
	wheel.usbInit();

	CHECK_NEAR(wheelTorqueForEastForce(wheel), 10000, 2);
}

TEST_CASE_FIXTURE(MainClassFixture, "characterization: without usbInit the wheel treats direction 90 deg as a linear X direction of half strength" * DELETES_FFBHIDMAIN) {
	// Shows that usbInit matters: with the HidFFB default mask (0x02) the direction enable bit 0x04 sent by the host is
	// ignored and the X magnitude becomes (9000 - 18000) / 18000 = -0.5 (HidFFB.cpp set_effect).
	// NEEDS REVIEW: only reachable if the main class is used before usbInit (cppmain calls usbInit at startup).
	FFBWheel wheel;

	CHECK_NEAR(wheelTorqueForEastForce(wheel), -5000, 2);
}

} // TEST_SUITE FFBWheel

// =====================================================================================================================
TEST_SUITE("FFBJoystick") {

TEST_CASE_FIXTURE(MainClassFixture, "the joystick has id 2 and the name FFB Joystick (2 Axis)" * DELETES_FFBHIDMAIN) {
	FFBJoystick joystick;
	CHECK(joystick.getInfo().id == CLSID_MAIN_FFBJOY);
	CHECK(std::string(joystick.getInfo().name) == "FFB Joystick (2 Axis)");
}

TEST_CASE_FIXTURE(MainClassFixture, "the joystick creates two axes" * DELETES_FFBHIDMAIN) {
	FFBJoystick joystick;
	CHECK(CommandHandler::getHandlersFromClassName("axis").size() == 2);
}

TEST_CASE_FIXTURE(MainClassFixture, "the joystick reports both axis positions in X and Y" * DELETES_FFBHIDMAIN) {
	FFBJoystick joystick;
	selectMocks(0).setDegrees(90.0f);  // -0.2 of 900 deg (inverted)
	selectMocks(1).setDegrees(-90.0f); // +0.2

	HostRtos::runUntilBlocked(joystick);
	HostPlatform::advanceMs(1);
	REQUIRE(HostTarget::timerUpdate(&htim13));
	HostRtos::runUntilBlocked(joystick);

	auto reports = gamepadReports();
	REQUIRE(reports.size() == 1);
	CHECK(axis16(reports[0], 0) == -6554);
	CHECK(axis16(reports[0], 1) == 6553);
	CHECK(axis16(reports[0], 2) == 0);
}

TEST_CASE_FIXTURE(MainClassFixture, "usbInit of the joystick uses the 2 axis FFB descriptors" * DELETES_FFBHIDMAIN) {
	FFBJoystick joystick;

	joystick.usbInit();

	CHECK(UsbHidHandler::getHidDesc() == hid_2ffb_desc);
	REQUIRE(usb_device != nullptr);
	CHECK(usb_device->getUsbConfigurationDesc(0) == usb_cdc_hid_conf_2axis);
}

TEST_CASE_FIXTURE(MainClassFixture, "the joystick receives the PID reports of the host with its HID FFB handler" * DELETES_FFBHIDMAIN) {
	FFBJoystick joystick;
	PidHost host;
	REQUIRE(dynamic_cast<HidFFB *>(UsbHidHandler::globalHidHandler) != nullptr);

	host.deviceControl(0x01);

	CHECK(joystick.getFfbActive());
}

} // TEST_SUITE FFBJoystick

// =====================================================================================================================
TEST_SUITE("FFBHIDExt") {

TEST_CASE_FIXTURE(MainClassFixture, "the external FFB gamepad has id 3 and the name HID Gamepad (Ext FFB)" * DELETES_FFBHIDMAIN) {
	FFBHIDExt ext;
	CHECK(ext.getInfo().id == CLSID_MAIN_FFBEXT);
	CHECK(std::string(ext.getInfo().name) == "HID Gamepad (Ext FFB)");
}

TEST_CASE_FIXTURE(MainClassFixture, "the external FFB gamepad creates two axes and an effects manager per axis" * DELETES_FFBHIDMAIN) {
	FFBHIDExt ext;
	CHECK(CommandHandler::getHandlersFromClassName("axis").size() == 2);
	CHECK(CommandHandler::getHandlerFromClassName("fxm", 0) != nullptr);
	CHECK(CommandHandler::getHandlerFromClassName("fxm", 1) != nullptr);
}

TEST_CASE_FIXTURE(MainClassFixture, "the external FFB gamepad does not take the PID reports of the host" * DELETES_FFBHIDMAIN) {
	FFBHIDExt ext;
	CHECK(UsbHidHandler::globalHidHandler == nullptr);
}

TEST_CASE_FIXTURE(MainClassFixture, "FFB of the external FFB gamepad is switched on by the effects manager command" * DELETES_FFBHIDMAIN) {
	FFBHIDExt ext;
	CommandHandler *fxm = CommandHandler::getHandlerFromClassName("fxm", 0);
	REQUIRE(fxm != nullptr);
	CHECK(get(ext, "ffbactive") == 0);

	REQUIRE(cmd(*fxm, "ffbstate", CMDtype::set, 1) == CommandStatus::OK);

	CHECK(get(ext, "ffbactive") == 1);
}

TEST_CASE_FIXTURE(MainClassFixture, "USB suspend switches the FFB of the external FFB gamepad off" * DELETES_FFBHIDMAIN) {
	FFBHIDExt ext;
	CommandHandler *fxm = CommandHandler::getHandlerFromClassName("fxm", 0);
	REQUIRE(fxm != nullptr);
	ext.usbResume();
	REQUIRE(cmd(*fxm, "ffbstate", CMDtype::set, 1) == CommandStatus::OK);

	ext.usbSuspend();

	CHECK(get(ext, "ffbactive") == 0);
}

TEST_CASE_FIXTURE(MainClassFixture, "usbInit of the external FFB gamepad uses the gamepad descriptors without FFB" * DELETES_FFBHIDMAIN) {
	FFBHIDExt ext;

	ext.usbInit();

	CHECK(UsbHidHandler::getHidDesc() == hid_gamepad_desc);
	REQUIRE(usb_device != nullptr);
	CHECK(usb_device->getUsbConfigurationDesc(0) == usb_cdc_hid_conf_gamepad);
}

} // TEST_SUITE FFBHIDExt

// =====================================================================================================================
TEST_SUITE("MainClassChooser") {

TEST_CASE_FIXTURE(MainClassFixture, "the main class ids of the registry are unique") {
	std::set<uint16_t> ids;
	for (const class_entry<FFBoardMain> &e : class_registry) {
		CAPTURE(e.selectionId);
		CHECK(ids.insert(e.selectionId).second);
	}
}

TEST_CASE_FIXTURE(MainClassFixture, "each registry entry is selected by the class id of its class") {
	for (const class_entry<FFBoardMain> &e : class_registry) {
		CAPTURE(e.info.name);
		CHECK(e.selectionId == e.info.id);
	}
}

TEST_CASE_FIXTURE(MainClassFixture, "the main class names of the registry are unique and not empty") {
	std::set<std::string> names;
	for (const class_entry<FFBoardMain> &e : class_registry) {
		std::string name = e.info.name;
		CAPTURE(name);
		CHECK_FALSE(name.empty());
		CHECK(names.insert(name).second);
	}
}

TEST_CASE_FIXTURE(MainClassFixture, "every main class of the registry is creatable") {
	for (const class_entry<FFBoardMain> &e : class_registry) {
		CAPTURE(e.info.name);
		CHECK(mainchooser.isCreatable(e.selectionId));
		CHECK(mainchooser.isValidClassId(e.selectionId));
	}
}

TEST_CASE_FIXTURE(MainClassFixture, "an id that is not in the registry is neither valid nor creatable") {
	CHECK_FALSE(mainchooser.isValidClassId(0x7ff));
	CHECK_FALSE(mainchooser.isCreatable(0x7ff));
	CHECK(mainchooser.Create(0x7ff) == nullptr);
}

TEST_CASE_FIXTURE(MainClassFixture, "the class list offers every main class as creatable with id and name") {
	std::vector<CommandReply> replies;
	mainchooser.replyAvailableClasses(replies);

	REQUIRE(replies.size() == class_registry.size());
	CHECK(replies[0].reply == "0:1:Basic (Failsafe)");
	CHECK(replies[1].reply == "1:1:FFB Wheel (1 Axis)");
	CHECK(replies[2].reply == "2:1:FFB Joystick (2 Axis)");
	CHECK(replies[3].reply == "3:1:HID Gamepad (Ext FFB)");
}

TEST_CASE_FIXTURE(MainClassFixture, "the chooser creates the failsafe main class for id 0") {
	std::unique_ptr<FFBoardMain> main(mainchooser.Create(0));
	REQUIRE(main != nullptr);
	CHECK(main->getInfo().id == 0);
	CHECK(main->getSelectionID() == 0);
}

TEST_CASE_FIXTURE(MainClassFixture, "the chooser creates an FFBWheel for its id" * DELETES_FFBHIDMAIN) {
	std::unique_ptr<FFBoardMain> main(mainchooser.Create(CLSID_MAIN_FFBWHEEL));
	REQUIRE(main != nullptr);
	CHECK(dynamic_cast<FFBWheel *>(main.get()) != nullptr);
	CHECK(main->getSelectionID() == CLSID_MAIN_FFBWHEEL);
}

TEST_CASE_FIXTURE(MainClassFixture, "the chooser creates an FFBJoystick for its id" * DELETES_FFBHIDMAIN) {
	std::unique_ptr<FFBoardMain> main(mainchooser.Create(CLSID_MAIN_FFBJOY));
	REQUIRE(main != nullptr);
	CHECK(dynamic_cast<FFBJoystick *>(main.get()) != nullptr);
}

TEST_CASE_FIXTURE(MainClassFixture, "the chooser creates an FFBHIDExt for its id" * DELETES_FFBHIDMAIN) {
	std::unique_ptr<FFBoardMain> main(mainchooser.Create(CLSID_MAIN_FFBEXT));
	REQUIRE(main != nullptr);
	CHECK(dynamic_cast<FFBHIDExt *>(main.get()) != nullptr);
}

TEST_CASE_FIXTURE(MainClassFixture, "the chooser creates the CAN bridge and the CAN input main class for their ids") {
	for (uint16_t id : {static_cast<uint16_t>(CLSID_MAIN_CAN), static_cast<uint16_t>(CLSID_MAIN_CANINPUT)}) {
		CAPTURE(id);
		std::unique_ptr<FFBoardMain> main(mainchooser.Create(id));
		REQUIRE(main != nullptr);
		CHECK(main->getInfo().id == id);
	}
}

} // TEST_SUITE MainClassChooser
