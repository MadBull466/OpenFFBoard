/*
 * UsbFirmwareGlue: the firmware side of the USB device end to end on the real TinyUSB stack: the mount / unmount /
 * resume callbacks of global_callbacks.cpp reaching the main class, USBdevice (descriptor callbacks, string
 * descriptors with the UID based serial, USBdevice::Run in its own thread) and the configurations of the other main
 * classes (CDC only, gamepad).
 *
 * Already covered elsewhere: the descriptor tables byte by byte (cases_target/test_usb_descriptors.cpp, bugs 46, 47),
 * the bus suspend that never reaches the main class (bug 42, cases_usb/test_usb_setup.cpp).
 * Not compiled in any test variant: the 1-axis descriptors (FFBWHEEL_USE_1AXIS_DESC is off in the F407 feature set,
 * the 32-bit 1-axis variant does not compile: bug 51).
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "UsbFixture.h"

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace {

using Handshake = HostDcd::Handshake;

/** Serial number like USBdevice::getUsbSerial: UID words 2, 1, 0 as 8 hex digits each */
std::string expectedSerial() {
	char buf[25];
	std::snprintf(buf, sizeof(buf), "%08x%08x%08x", (unsigned)HostTarget::state().uid[2], (unsigned)HostTarget::state().uid[1],
				  (unsigned)HostTarget::state().uid[0]);
	return buf;
}

} // namespace

TEST_SUITE("UsbFirmwareGlue") {

// ---------------------------------------------------------------- mount / unmount / resume reaching the main class

TEST_CASE_FIXTURE(UsbFixture, "unplugging the cable of a configured device suspends the main class") {
	startDevice();
	enumerate();
	REQUIRE(main->suspends == 0);
	HostDcd::detach();
	process();
	CHECK_FALSE(tud_mounted());
	CHECK(main->suspends == 1); // DCD_EVENT_UNPLUGGED -> tud_umount_cb -> mainclass->usbSuspend()
}

TEST_CASE_FIXTURE(UsbFixture, "a host that sets configuration 0 suspends the main class") {
	startDevice();
	enumerate();
	REQUIRE(host.setConfiguration(0).ok());
	CHECK_FALSE(tud_mounted());
	CHECK(main->suspends == 1); // usbd.c: SET_CONFIGURATION(0) of a configured device -> tud_umount_cb
}

TEST_CASE_FIXTURE(UsbFixture, "a device plugged in again and enumerated resumes the main class again") {
	startDevice();
	enumerate();
	HostDcd::detach();
	process();
	HostDcd::attach();
	enumerate();
	CHECK(main->suspends == 1);
	CHECK(main->resumes == 2); // tud_mount_cb at both enumerations
}

TEST_CASE_FIXTURE(UsbFixture, "characterization: a bus reset of a configured device neither suspends nor resumes the main class until the host configures it again (NEEDS REVIEW 157)") {
	// NEEDS REVIEW 157: TinyUSB calls no callback on a bus reset (usbd.c:589-593), so the main class keeps its state
	// (e.g. FFB effects keep playing) between a host reset and the next SET_CONFIGURATION, and then gets a second
	// usbResume without a usbSuspend. Should the firmware stop FFB on a bus reset?
	startDevice();
	enumerate();
	host.busReset();
	CHECK_FALSE(tud_mounted());
	CHECK(main->suspends == 0);
	CHECK(main->resumes == 1);
	enumerate();
	CHECK(main->suspends == 0);
	CHECK(main->resumes == 2);
}

TEST_CASE_FIXTURE(UsbFixture, "a resume before the first setup packet is not passed to the main class") {
	// usbd.c:1176-1180: suspend / resume are skipped until the device is connected (first SETUP received)
	startDevice();
	host.busReset();
	HostDcd::suspend();
	HostDcd::resume();
	process();
	CHECK(main->resumes == 0);
}

// ---------------------------------------------------------------- configurations of the other main classes

TEST_CASE_FIXTURE(UsbFixture, "the CDC only configuration of the failsafe main class enumerates without a HID interface") {
	startDevice(usb_cdc_conf, nullptr);
	enumerate();
	CHECK(host.interfaces.size() == 2);
	CHECK(host.findInterface(TUSB_CLASS_HID) == nullptr);
	REQUIRE(host.cdcSetControlLineState(true, true).ok());
	REQUIRE(host.cdcWrite("sys.main?\n") == Handshake::Ack);
	process();
	CHECK(host.cdcRead() == "[sys.main?|0]\n");
}

TEST_CASE_FIXTURE(UsbFixture, "the gamepad configuration of the external FFB main class serves the gamepad report descriptor") {
	startDevice(usb_cdc_hid_conf_gamepad, hid_gamepad_desc);
	enumerate();
	const UsbHost::Interface *hid = host.findInterface(TUSB_CLASS_HID);
	REQUIRE(hid != nullptr);
	CHECK(hid->hidReportLength == USB_HID_GAMEPAD_REPORT_DESC_SIZE);
	UsbHost::ControlResult r = host.hidGetReportDescriptor();
	REQUIRE(r.ok());
	CHECK(r.data == std::vector<uint8_t>(hid_gamepad_desc, hid_gamepad_desc + USB_HID_GAMEPAD_REPORT_DESC_SIZE));
}

// ---------------------------------------------------------------- string descriptors

TEST_CASE_FIXTURE(UsbFixture, "the serial number string read by the host is the 96 bit unique id as 24 hex digits") {
	startDevice();
	enumerate();
	REQUIRE(host.deviceDescriptor[16] == 3); // iSerialNumber
	const std::string serial = host.getString(3);
	CHECK(serial == expectedSerial());
	CHECK(serial.size() == 24);
}

TEST_CASE_FIXTURE(UsbFixture, "the serial number follows the unique id of the chip") {
	HostTarget::state().uid[0] = 0x12345678;
	HostTarget::state().uid[1] = 0x9ABCDEF0;
	HostTarget::state().uid[2] = 0x0000BEEF;
	startDevice();
	enumerate();
	CHECK(host.getString(3) == "0000beef9abcdef012345678");
}

TEST_CASE_FIXTURE(UsbFixture, "the string descriptor 0 lists US English as the only language") {
	startDevice();
	enumerate();
	UsbHost::ControlResult r = host.getDescriptor(TUSB_DESC_STRING, 0, 255);
	REQUIRE(r.ok());
	CHECK(r.data == std::vector<uint8_t>{4, TUSB_DESC_STRING, 0x09, 0x04});
}

TEST_CASE_FIXTURE(UsbFixture, "the string descriptor length covers exactly the string read by the host") {
	startDevice();
	enumerate();
	UsbHost::ControlResult r = host.getDescriptor(TUSB_DESC_STRING, 1, 255);
	REQUIRE(r.ok());
	REQUIRE(r.data.size() >= 2);
	CHECK(r.data[0] == r.data.size());
	CHECK(r.data[0] == 2 + 2 * std::string("Open FFBoard").size());
	CHECK(r.data[1] == TUSB_DESC_STRING);
}

TEST_CASE_FIXTURE(UsbFixture, "the interface strings read by the host name the CDC and HID functions with a bracketed suffix") {
	startDevice();
	enumerate();
	const std::string cdc = host.getString(host.findInterface(TUSB_CLASS_CDC)->stringIndex);
	const std::string hid = host.getString(host.findInterface(TUSB_CLASS_HID)->stringIndex);
	CHECK(cdc.rfind("FFBoard CDC(", 0) == 0);
	CHECK(hid.rfind("FFBoard HID(", 0) == 0);
	CHECK(cdc.back() == ')');
	CHECK(hid.back() == ')');
	CHECK(cdc.size() == std::string("FFBoard CDC()").size() + 4); // appendSerial 4 (suffix content: bug 47)
	CHECK(hid.size() == std::string("FFBoard HID()").size() + 4);
}

TEST_CASE_FIXTURE(UsbFixture, "a string index without a string is stalled") {
	startDevice();
	enumerate();
	CHECK(host.getDescriptor(TUSB_DESC_STRING, 7, 255).stalled());
	CHECK(host.getConfiguration().ok());
}

// ---------------------------------------------------------------- USBdevice::Run in its own thread

TEST_CASE_FIXTURE(UsbFixture, "USBdevice::Run initialises TinyUSB in the USB thread and processes the enumeration there") {
	main = std::make_unique<UsbTestMain>();
	REQUIRE(HostRtos::makeResumable("CDCCMD") != nullptr);
	REQUIRE(HostRtos::makeResumable("CMD_MAIN") != nullptr);
	mainclass = main.get();
	UsbHidHandler::setHidDesc(hid_2ffb_desc);
	auto device = std::make_unique<USBdevice>(&usb_devdesc_ffboard_composite, usb_cdc_hid_conf_2axis, &usb_ffboard_strings_default);
	device->registerUsb(); // usb_device = this, starts the thread "USB"
	CHECK(usb_device == device.get());
	TaskHandle_t usbTask = HostRtos::makeResumable("USB");
	REQUIRE(usbTask != nullptr);
	CHECK_FALSE(HostDcd::state().initialised);
	HostRtos::runUntilBlocked(usbTask); // tusb_init, then tud_task waits for the first event
	CHECK(HostDcd::state().initialised);
	CHECK(HostDcd::state().pullup);
	// A host whose pump only runs the firmware threads: every event is processed by the USB thread
	UsbHost threadHost([] { HostRtos::runUntilIdle(); });
	REQUIRE(threadHost.enumerate());
	CHECK(tud_mounted());
	CHECK(main->resumes == 1);
	REQUIRE(threadHost.cdcSetControlLineState(true, true).ok());
	REQUIRE(threadHost.cdcWrite("sys.main?\n") == Handshake::Ack);
	HostRtos::runUntilIdle();
	CHECK(threadHost.cdcRead() == "[sys.main?|0]\n");
	CHECK(threadHost.errors().empty());
	device.reset(); // Deletes the USB task parked in tud_task
	usb_device = nullptr;
}

} // TEST_SUITE
