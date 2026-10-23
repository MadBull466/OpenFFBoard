/*
 * USB HID handler base class (UsbHidHandler.cpp)
 *
 * The TinyUSB HID callbacks (tud_hid_set_report_cb, tud_hid_get_report_cb, tud_hid_descriptor_report_cb in
 * global_callbacks.cpp, not compiled for the host) forward reports to UsbHidHandler::globalHidHandler and
 * return UsbHidHandler::getHidDesc() as the report descriptor.
 */
#include "doctest.h"
#include "TestHelpers.h"

#include "UsbHidHandler.h"

#include <cstring>
#include <vector>

namespace {

/** Saves and restores the static state of UsbHidHandler (also clears a dangling globalHidHandler) */
struct HidHandlerFixture {
	UsbHidHandler *savedHandler = UsbHidHandler::globalHidHandler;
	const uint8_t *savedDesc = UsbHidHandler::getHidDesc();
	HidHandlerFixture() {
			UsbHidHandler::setHidDesc(nullptr);
	}
	~HidHandlerFixture() {
		UsbHidHandler::globalHidHandler = savedHandler;
		UsbHidHandler::setHidDesc(savedDesc);
	}
};

class RecordingHidHandler : public UsbHidHandler {
public:
	void hidOut(uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) override {
		outId = report_id;
		outType = report_type;
		outData.assign(buffer, buffer + bufsize);
	}
	uint16_t hidGet(uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) override {
		getId = report_id;
		getType = report_type;
		buffer[0] = 0x42;
		return 1;
	}
	uint8_t outId = 0;
	hid_report_type_t outType = HID_REPORT_TYPE_INVALID;
	std::vector<uint8_t> outData;
	uint8_t getId = 0;
	hid_report_type_t getType = HID_REPORT_TYPE_INVALID;
};

const uint8_t descA[] = {0x05, 0x01, 0x09, 0x04};
const uint8_t descB[] = {0x05, 0x0F};

} // namespace

TEST_SUITE("UsbHidHandler") {

TEST_CASE_FIXTURE(HidHandlerFixture, "the default report callbacks return no data and ignore out reports") {
	UsbHidHandler handler;
	uint8_t buffer[4] = {1, 2, 3, 4};
	CHECK(handler.hidGet(1, HID_REPORT_TYPE_FEATURE, buffer, sizeof(buffer)) == 0);
	CHECK(buffer[0] == 1); // Untouched
	CHECK(buffer[3] == 4);
	handler.hidOut(2, HID_REPORT_TYPE_OUTPUT, buffer, sizeof(buffer)); // Base class: no effect
	CHECK(buffer[0] == 1);
}

TEST_CASE_FIXTURE(HidHandlerFixture, "a new handler does not receive reports until it registers") {
	UsbHidHandler handler;
	CHECK(UsbHidHandler::globalHidHandler == nullptr);
	handler.registerHidCallback();
	CHECK(UsbHidHandler::globalHidHandler == &handler);
}

TEST_CASE_FIXTURE(HidHandlerFixture, "the handler registered last receives the reports") {
	RecordingHidHandler first;
	RecordingHidHandler second;
	first.registerHidCallback();
	second.registerHidCallback();
	REQUIRE(UsbHidHandler::globalHidHandler == &second);

	// Like tud_hid_set_report_cb / tud_hid_get_report_cb
	const uint8_t report[2] = {0xAA, 0xBB};
	UsbHidHandler::globalHidHandler->hidOut(5, HID_REPORT_TYPE_OUTPUT, report, sizeof(report));
	uint8_t reply[8] = {};
	CHECK(UsbHidHandler::globalHidHandler->hidGet(6, HID_REPORT_TYPE_FEATURE, reply, sizeof(reply)) == 1);

	CHECK(second.outId == 5);
	CHECK(second.outType == HID_REPORT_TYPE_OUTPUT);
	CHECK(second.outData == std::vector<uint8_t>{0xAA, 0xBB});
	CHECK(second.getId == 6);
	CHECK(second.getType == HID_REPORT_TYPE_FEATURE);
	CHECK(reply[0] == 0x42);
	CHECK(first.outData.empty());
}

TEST_CASE_FIXTURE(HidHandlerFixture, "setHidDesc stores the report descriptor returned by getHidDesc") {
	CHECK(UsbHidHandler::getHidDesc() == nullptr);
	UsbHidHandler::setHidDesc(descA);
	CHECK(UsbHidHandler::getHidDesc() == descA);
	UsbHidHandler::setHidDesc(descB);
	CHECK(UsbHidHandler::getHidDesc() == descB);
}

TEST_CASE_FIXTURE(HidHandlerFixture, "the report descriptor is shared by all handlers") {
	UsbHidHandler a;
	UsbHidHandler b;
	a.setHidDesc(descA);
	CHECK(b.getHidDesc() == descA);
	CHECK(UsbHidHandler::hid_desc == descA);
}

TEST_CASE_FIXTURE(HidHandlerFixture, "destroying a handler that is not registered keeps the registered handler") {
	RecordingHidHandler registered;
	registered.registerHidCallback();
	{
		UsbHidHandler other;
	}
	CHECK(UsbHidHandler::globalHidHandler == &registered);
}

TEST_CASE_FIXTURE(HidHandlerFixture, "a registered handler unregisters when destroyed" * KNOWN_ISSUE("UsbHidHandler::~UsbHidHandler does not clear globalHidHandler (UsbHidHandler.cpp:19-21). Reports received afterwards are passed to a destroyed object by the tinyusb callbacks (global_callbacks.cpp:317, 332, 344). Same root cause as the HidFFB case `handler unregisters when destroyed`")) {
	{
		UsbHidHandler handler;
		handler.registerHidCallback();
	}
	bool unregistered = UsbHidHandler::globalHidHandler == nullptr;
	CHECK(unregistered);
}

} // TEST_SUITE
