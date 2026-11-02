/*
 * HidDevice: TinyUSB's HID class driver (FFBoard/USB/class/hid/hid_device.c, vendor tier B: tested, never fixed) on
 * the fake device controller, with the firmware's HID glue: the callbacks tud_hid_descriptor_report_cb,
 * tud_hid_set_report_cb and tud_hid_get_report_cb of global_callbacks.cpp, UsbHidHandler, the HID command interface
 * and HidFFB (PID reports) over the real stack.
 *
 * The F407 wheel configuration: HID interface 2, interrupt IN 0x83 and OUT 0x02 with 64 bytes, report descriptor
 * hid_2ffb_desc. CFG_TUD_HID_EP_BUFSIZE is the default 64 (control and endpoint buffers).
 * Reports with a report id: the PID output reports and the HID command report carry their id in the first byte
 * (FFB_SetEffect_t::reportId, HID_CMD_Data_t::reportId); the PID feature reports do not (ffb_defs.h:262, 270, 278,
 * TinyUSB removes / adds the id of control transfers).
 *
 * RecordingHidHandler (a UsbHidHandler registered as the global HID handler) records what the firmware's callbacks
 * pass to the handler. It unregisters itself when destroyed (the UsbHidHandler destructor does not: bug 19).
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "UsbFixture.h"

#include "HidCommandInterface.h"
#include "ffb_defs.h"

#include <cstring>
#include <string>
#include <vector>

namespace {

using Handshake = HostDcd::Handshake;

constexpr uint8_t EP_IN = 0x83;	 // usb_descriptors.h USB_CONF_DESC_HID_CDC: HID interrupt IN
constexpr uint8_t EP_OUT = 0x02; // HID interrupt OUT
constexpr uint16_t BUFSIZE = CFG_TUD_HID_EP_BUFSIZE;

/** UsbHidHandler recording the calls of global_callbacks.cpp; hidGet answers with `getReply` for `getId` */
struct RecordingHidHandler : UsbHidHandler {
	struct Out {
		uint8_t id;
		hid_report_type_t type;
		std::vector<uint8_t> data;
	};
	std::vector<Out> outs;
	struct Get {
		uint8_t id;
		hid_report_type_t type;
		uint16_t reqlen;
	};
	std::vector<Get> gets;
	uint8_t getId = 0x42;
	std::vector<uint8_t> getReply = {1, 2, 3};

	RecordingHidHandler() { registerHidCallback(); }
	~RecordingHidHandler() override {
		if (globalHidHandler == this) {
			globalHidHandler = nullptr;
		}
	}
	void hidOut(uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) override {
		outs.push_back({report_id, report_type, std::vector<uint8_t>(buffer, buffer + bufsize)});
	}
	uint16_t hidGet(uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) override {
		gets.push_back({report_id, report_type, reqlen});
		if (report_id != getId) {
			return 0;
		}
		std::memcpy(buffer, getReply.data(), getReply.size());
		return (uint16_t)getReply.size();
	}
};

struct HidFixture : UsbFixture {
	/** Starts the F407 wheel device and enumerates it */
	void start() {
		startDevice();
		enumerate();
	}
	/** Starts the device with a real HidFFB (EffectsCalculator + HidFFB like an FFBWheel) and enumerates it */
	HidFFB &startWithFfb() {
		startDevice();
		HidFFB &f = createHidFfb();
		enumerate();
		return f;
	}

	UsbHost::ControlResult hidRequest(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wLength,
									  const std::vector<uint8_t> &data = {}) {
		return host.control(UsbHost::request(bmRequestType, bRequest, wValue, host.hidInterface(), wLength), data);
	}

	/** One IN transaction on the HID IN endpoint without retries */
	Handshake inPacket(std::vector<uint8_t> &data) { return HostDcd::in(host.address, EP_IN & 0x0F, data); }

	template <typename T>
	static std::vector<uint8_t> bytes(const T &report) {
		std::vector<uint8_t> v(sizeof(T));
		std::memcpy(v.data(), &report, sizeof(T));
		return v;
	}

	/** Creates an effect over USB like a PID driver: Create New Effect (feature 0x11), Block Load (feature 0x12) */
	uint8_t createEffect(uint8_t type) {
		REQUIRE(host.hidSetReport(HID_REPORT_TYPE_FEATURE, HID_ID_NEWEFREP, {HID_ID_NEWEFREP, type, 0, 0}).ok());
		UsbHost::ControlResult r = host.hidGetReport(HID_REPORT_TYPE_FEATURE, HID_ID_BLKLDREP, 5);
		REQUIRE(r.ok());
		REQUIRE(r.data.size() == 5);
		REQUIRE(r.data[2] == 1); // load status success
		return r.data[1];
	}
	/** Reads all pending input reports (e.g. the PID state reports sent by HidFFB) */
	void drainInputReports() {
		std::vector<uint8_t> data;
		while (inPacket(data) == Handshake::Ack) {
			process();
		}
	}
};

} // namespace

TEST_SUITE("HidDevice") {

// ---------------------------------------------------------------- descriptors

TEST_CASE_FIXTURE(HidFixture, "the report descriptor read by the host is the report descriptor set by the firmware") {
	start();
	REQUIRE(host.findInterface(TUSB_CLASS_HID)->hidReportLength == USB_HID_2FFB_REPORT_DESC_SIZE);
	UsbHost::ControlResult r = host.hidGetReportDescriptor();
	REQUIRE(r.ok());
	CHECK(r.data == std::vector<uint8_t>(hid_2ffb_desc, hid_2ffb_desc + USB_HID_2FFB_REPORT_DESC_SIZE));
}

TEST_CASE_FIXTURE(HidFixture, "the HID class descriptor can be read from the interface and equals the one in the configuration") {
	start();
	UsbHost::ControlResult r = hidRequest(0x81, TUSB_REQ_GET_DESCRIPTOR, HID_DESC_TYPE_HID << 8, 9);
	REQUIRE(r.ok());
	// Configuration: config 9, IAD 8, CDC control 9 + functional 5+5+4+5, notification EP 7, CDC data 9 + 2 EP 14,
	// HID interface 9, then the HID descriptor
	const auto pos = std::search(host.configDescriptor.begin(), host.configDescriptor.end(), r.data.begin(), r.data.end());
	CHECK(pos != host.configDescriptor.end());
	CHECK(r.data.size() == 9);
	CHECK(r.data[1] == HID_DESC_TYPE_HID);
}

TEST_CASE_FIXTURE(HidFixture, "a request for the physical descriptor of the HID interface is stalled") {
	start();
	CHECK(hidRequest(0x81, TUSB_REQ_GET_DESCRIPTOR, HID_DESC_TYPE_PHYSICAL << 8, 64).stalled());
}

TEST_CASE_FIXTURE(HidFixture, "an undefined HID class request is stalled") {
	start();
	CHECK(hidRequest(0xA1, 0x05, 0, 1).stalled()); // HID 1.11 7.2: 0x04..0x08 are reserved
}

TEST_CASE_FIXTURE(HidFixture, "a vendor request to the HID interface is stalled") {
	start();
	CHECK(hidRequest(0xC1, 0x01, 0, 1).stalled());
}

// ---------------------------------------------------------------- idle and protocol

TEST_CASE_FIXTURE(HidFixture, "an idle rate set by the host is read back") {
	start();
	REQUIRE(hidRequest(0x21, HID_REQ_CONTROL_SET_IDLE, 25 << 8, 0).ok()); // 25 * 4 ms, all reports (id 0)
	UsbHost::ControlResult r = hidRequest(0xA1, HID_REQ_CONTROL_GET_IDLE, 0, 1);
	REQUIRE(r.ok());
	CHECK(r.data == std::vector<uint8_t>{25});
}

TEST_CASE_FIXTURE(HidFixture, "the idle rate is 0 after the enumeration") {
	start();
	UsbHost::ControlResult r = hidRequest(0xA1, HID_REQ_CONTROL_GET_IDLE, 0, 1);
	REQUIRE(r.ok());
	CHECK(r.data == std::vector<uint8_t>{0});
}

TEST_CASE_FIXTURE(HidFixture, "the protocol is the report protocol after the enumeration") {
	start();
	UsbHost::ControlResult r = hidRequest(0xA1, HID_REQ_CONTROL_GET_PROTOCOL, 0, 1);
	REQUIRE(r.ok());
	CHECK(r.data == std::vector<uint8_t>{HID_PROTOCOL_REPORT});
	CHECK(tud_hid_n_get_protocol(0) == HID_PROTOCOL_REPORT);
	CHECK(tud_hid_n_interface_protocol(0) == HID_ITF_PROTOCOL_NONE); // Not a boot interface
}

TEST_CASE_FIXTURE(HidFixture, "a protocol set by the host is read back") {
	start();
	REQUIRE(hidRequest(0x21, HID_REQ_CONTROL_SET_PROTOCOL, HID_PROTOCOL_BOOT, 0).ok());
	CHECK(tud_hid_n_get_protocol(0) == HID_PROTOCOL_BOOT);
	UsbHost::ControlResult r = hidRequest(0xA1, HID_REQ_CONTROL_GET_PROTOCOL, 0, 1);
	REQUIRE(r.ok());
	CHECK(r.data == std::vector<uint8_t>{HID_PROTOCOL_BOOT});
}

TEST_CASE_FIXTURE(HidFixture, "a bus reset sets the report protocol again") {
	start();
	REQUIRE(hidRequest(0x21, HID_REQ_CONTROL_SET_PROTOCOL, HID_PROTOCOL_BOOT, 0).ok());
	enumerate();
	CHECK(tud_hid_n_get_protocol(0) == HID_PROTOCOL_REPORT);
}

// ---------------------------------------------------------------- GET_REPORT / SET_REPORT through the firmware callbacks

TEST_CASE_FIXTURE(HidFixture, "GET_REPORT returns the report id followed by the data of the HID handler") {
	start();
	RecordingHidHandler handler;
	UsbHost::ControlResult r = host.hidGetReport(HID_REPORT_TYPE_FEATURE, 0x42, 10);
	REQUIRE(r.ok());
	CHECK(r.data == std::vector<uint8_t>{0x42, 1, 2, 3});
	REQUIRE(handler.gets.size() == 1);
	CHECK(handler.gets[0].id == 0x42);
	CHECK(handler.gets[0].type == HID_REPORT_TYPE_FEATURE);
	CHECK(handler.gets[0].reqlen == 9); // wLength minus the id byte added by TinyUSB
}

TEST_CASE_FIXTURE(HidFixture, "GET_REPORT without a report id returns only the data of the HID handler") {
	start();
	RecordingHidHandler handler;
	handler.getId = 0;
	UsbHost::ControlResult r = host.hidGetReport(HID_REPORT_TYPE_INPUT, 0, 10);
	REQUIRE(r.ok());
	CHECK(r.data == std::vector<uint8_t>{1, 2, 3});
	REQUIRE(handler.gets.size() == 1);
	CHECK(handler.gets[0].type == HID_REPORT_TYPE_INPUT);
	CHECK(handler.gets[0].reqlen == 10);
}

TEST_CASE_FIXTURE(HidFixture, "GET_REPORT passes at most the control buffer size to the HID handler") {
	start();
	RecordingHidHandler handler;
	REQUIRE(host.hidGetReport(HID_REPORT_TYPE_FEATURE, 0x42, 200).ok());
	REQUIRE(handler.gets.size() == 1);
	CHECK(handler.gets[0].reqlen == BUFSIZE - 1);
}

TEST_CASE_FIXTURE(HidFixture, "GET_REPORT without a report id is stalled when the HID handler has no data") {
	start();
	RecordingHidHandler handler; // answers only report 0x42
	CHECK(host.hidGetReport(HID_REPORT_TYPE_FEATURE, 0, 10).stalled());
	CHECK(handler.gets.size() == 1);
}

TEST_CASE_FIXTURE(HidFixture, "GET_REPORT to a HID handler with the default report functions is stalled") {
	start();
	struct PlainHandler : UsbHidHandler {
		~PlainHandler() override { globalHidHandler = nullptr; }
	} handler; // UsbHidHandler::hidGet returns 0, hidOut ignores the report
	handler.registerHidCallback();
	CHECK(host.hidSetReport(HID_REPORT_TYPE_FEATURE, 0, {1, 2}).ok());
	CHECK(host.hidGetReport(HID_REPORT_TYPE_FEATURE, 0, 10).stalled());
}

TEST_CASE_FIXTURE(HidFixture, "GET_REPORT without a registered HID handler is stalled") {
	start();
	REQUIRE(UsbHidHandler::globalHidHandler == nullptr);
	CHECK(host.hidGetReport(HID_REPORT_TYPE_FEATURE, 0, 10).stalled()); // tud_hid_get_report_cb returns 0
}

TEST_CASE_FIXTURE(HidFixture, "GET_REPORT for a report the HID handler does not have is stalled (V27)" *
								  KNOWN_ISSUE("V27 (TinyUSB, vendor): hid_device.c:299-306 adds the report id byte before calling "
											  "tud_hid_get_report_cb, so TU_ASSERT(xferlen > 0) never fails for a report id other than 0: "
											  "a callback result of 0 is answered with the id byte alone instead of the STALL promised by "
											  "hid_device.h:126 (USB 2.0 9.2.7 request error)")) {
	start();
	RecordingHidHandler handler; // answers only report 0x42
	UsbHost::ControlResult r = host.hidGetReport(HID_REPORT_TYPE_FEATURE, 0x43, 10);
	REQUIRE(handler.gets.size() == 1);
	REQUIRE_USB_MODELLED();
	CHECK(r.stalled()); // Defect: ACK with the single byte 0x43
}

TEST_CASE_FIXTURE(HidFixture, "SET_REPORT passes the report without its id byte to the HID handler") {
	start();
	RecordingHidHandler handler;
	REQUIRE(host.hidSetReport(HID_REPORT_TYPE_FEATURE, 0x42, {0x42, 7, 8, 9}).ok());
	REQUIRE(handler.outs.size() == 1);
	CHECK(handler.outs[0].id == 0x42);
	CHECK(handler.outs[0].type == HID_REPORT_TYPE_FEATURE);
	CHECK(handler.outs[0].data == std::vector<uint8_t>{7, 8, 9});
}

TEST_CASE_FIXTURE(HidFixture, "SET_REPORT without a report id passes all bytes to the HID handler") {
	start();
	RecordingHidHandler handler;
	REQUIRE(host.hidSetReport(HID_REPORT_TYPE_FEATURE, 0, {5, 6}).ok());
	REQUIRE(handler.outs.size() == 1);
	CHECK(handler.outs[0].id == 0);
	CHECK(handler.outs[0].data == std::vector<uint8_t>{5, 6});
}

TEST_CASE_FIXTURE(HidFixture, "SET_REPORT whose first byte is not the report id passes all bytes to the HID handler") {
	// hid_device.c:324 removes the first byte only if it equals the report id of wValue
	start();
	RecordingHidHandler handler;
	REQUIRE(host.hidSetReport(HID_REPORT_TYPE_FEATURE, 0x42, {0x41, 7, 8}).ok());
	REQUIRE(handler.outs.size() == 1);
	CHECK(handler.outs[0].id == 0x42);
	CHECK(handler.outs[0].data == std::vector<uint8_t>{0x41, 7, 8});
}

TEST_CASE_FIXTURE(HidFixture, "SET_REPORT longer than the control buffer is stalled and not passed on") {
	start();
	RecordingHidHandler handler;
	std::vector<uint8_t> data(BUFSIZE + 1, 0x42);
	CHECK(host.hidSetReport(HID_REPORT_TYPE_FEATURE, 0x42, data).stalled());
	CHECK(handler.outs.empty());
}

TEST_CASE_FIXTURE(HidFixture, "SET_REPORT without a registered HID handler is accepted") {
	start();
	CHECK(host.hidSetReport(HID_REPORT_TYPE_FEATURE, HID_ID_NEWEFREP, {HID_ID_NEWEFREP, 1, 0, 0}).ok());
}

// ---------------------------------------------------------------- output reports on the interrupt OUT endpoint

TEST_CASE_FIXTURE(HidFixture, "an output report on the interrupt endpoint reaches the HID handler with the id of its first byte") {
	start();
	RecordingHidHandler handler;
	REQUIRE(host.hidWrite({0x0C, 0x01, 0x02}) == Handshake::Ack);
	process();
	REQUIRE(handler.outs.size() == 1);
	CHECK(handler.outs[0].id == 0x0C); // TinyUSB passes id 0, global_callbacks.cpp:313-315 takes the first byte
	CHECK(handler.outs[0].type == HID_REPORT_TYPE_OUTPUT);
	// The payload; today the id byte is kept in front (the layout HidFFB and the HID command interface expect)
	REQUIRE(handler.outs[0].data.size() >= 2);
	CHECK(std::vector<uint8_t>(handler.outs[0].data.end() - 2, handler.outs[0].data.end()) == std::vector<uint8_t>{0x01, 0x02});
}

TEST_CASE_FIXTURE(HidFixture, "the interrupt OUT endpoint is armed again after every output report") {
	start();
	RecordingHidHandler handler;
	for (uint8_t i = 1; i <= 3; i++) {
		REQUIRE(host.hidWrite({0x0D, i}) == Handshake::Ack);
		process();
	}
	REQUIRE(handler.outs.size() == 3);
	CHECK(handler.outs[2].id == 0x0D);
	REQUIRE_FALSE(handler.outs[2].data.empty());
	CHECK(handler.outs[2].data.back() == 3); // the payload of the third report (with or without the id byte)
	CHECK(HostDcd::endpoint(EP_OUT).armed);
}

TEST_CASE_FIXTURE(HidFixture, "a 64 byte output report is delivered as one report") {
	start();
	RecordingHidHandler handler;
	std::vector<uint8_t> report(BUFSIZE, 0x5A);
	report[0] = 0xA1;
	REQUIRE(host.hidWrite(report) == Handshake::Ack);
	process();
	REQUIRE(handler.outs.size() == 1);
	CHECK(handler.outs[0].id == 0xA1);
	// All 64 bytes in one report: the id byte (kept today, global_callbacks.cpp:313-315) and the 63 payload bytes
	REQUIRE(handler.outs[0].data.size() >= BUFSIZE - 1);
	CHECK(std::vector<uint8_t>(handler.outs[0].data.end() - (BUFSIZE - 1), handler.outs[0].data.end()) ==
		  std::vector<uint8_t>(report.begin() + 1, report.end()));
}

TEST_CASE_FIXTURE(HidFixture, "an output report sent with SET_REPORT reaches the HID handler like one sent on the interrupt endpoint (bug 138)" *
								  KNOWN_ISSUE("bug 138: TinyUSB removes the id byte of a SET_REPORT (hid_device.c:323-327) but keeps it in "
											  "interrupt OUT reports (hid_device.c:396, id 0); global_callbacks.cpp:313-315 only fixes the id "
											  "argument, so the handlers (HidFFB::hidOut, hidCmdCallback, which expect the id in byte 0 of "
											  "output reports) parse a SET_REPORT(Output) one byte off")) {
	start();
	RecordingHidHandler handler;
	REQUIRE(host.hidWrite({0x0C, 0x01}) == Handshake::Ack);
	process();
	REQUIRE(host.hidSetReport(HID_REPORT_TYPE_OUTPUT, 0x0C, {0x0C, 0x01}).ok());
	REQUIRE(handler.outs.size() == 2);
	REQUIRE(handler.outs[1].id == 0x0C);
	REQUIRE_USB_MODELLED();
	CHECK(handler.outs[1].data == handler.outs[0].data); // Defect: {0x01} instead of {0x0C, 0x01}
}

TEST_CASE_FIXTURE(HidFixture, "a PID device control sent with SET_REPORT(Output) enables the actuators (bug 138)" *
								  KNOWN_ISSUE("bug 138: the output report reaches HidFFB::hidOut without its id byte "
											  "(see the case above); ffb_control reads report[1], one byte after the control value")) {
	HidFFB &ffb = startWithFfb();
	REQUIRE_FALSE(ffb.getFfbActive());
	REQUIRE(host.hidSetReport(HID_REPORT_TYPE_OUTPUT, HID_ID_CTRLREP, {HID_ID_CTRLREP, 0x01}).ok());
	REQUIRE_USB_MODELLED();
	CHECK(ffb.getFfbActive()); // Defect: still off
}

// ---------------------------------------------------------------- input reports on the interrupt IN endpoint

TEST_CASE_FIXTURE(HidFixture, "an input report with a report id is sent with the id as its first byte") {
	start();
	const uint8_t data[] = {10, 20, 30};
	REQUIRE(tud_hid_n_ready(0));
	REQUIRE(tud_hid_n_report(0, 0x01, data, sizeof(data)));
	std::vector<uint8_t> packet;
	REQUIRE(inPacket(packet) == Handshake::Ack);
	CHECK(packet == std::vector<uint8_t>{0x01, 10, 20, 30});
}

TEST_CASE_FIXTURE(HidFixture, "an input report without a report id is sent unchanged") {
	start();
	const uint8_t data[] = {0xA1, 1, 2};
	REQUIRE(tud_hid_n_report(0, 0, data, sizeof(data)));
	std::vector<uint8_t> packet;
	REQUIRE(inPacket(packet) == Handshake::Ack);
	CHECK(packet == std::vector<uint8_t>{0xA1, 1, 2});
}

TEST_CASE_FIXTURE(HidFixture, "the HID interface is busy until the host has read the input report") {
	start();
	const uint8_t first[] = {1};
	const uint8_t second[] = {2};
	REQUIRE(tud_hid_n_report(0, 0x01, first, 1));
	CHECK_FALSE(tud_hid_n_ready(0));
	CHECK_FALSE(tud_hid_n_report(0, 0x01, second, 1)); // Rejected, the first report is not overwritten
	std::vector<uint8_t> packet;
	REQUIRE(inPacket(packet) == Handshake::Ack);
	CHECK(packet == std::vector<uint8_t>{0x01, 1});
	process(); // Transfer complete
	CHECK(tud_hid_n_ready(0));
	CHECK(tud_hid_n_report(0, 0x01, second, 1));
	REQUIRE(inPacket(packet) == Handshake::Ack);
	CHECK(packet == std::vector<uint8_t>{0x01, 2});
}

TEST_CASE_FIXTURE(HidFixture, "the HID host gets nothing while no input report is sent") {
	start();
	std::vector<uint8_t> packet;
	CHECK(inPacket(packet) == Handshake::Nak);
}

TEST_CASE_FIXTURE(HidFixture, "an input report that does not fit into the endpoint buffer is rejected") {
	start();
	std::vector<uint8_t> big(BUFSIZE, 7); // 64 bytes + id byte
	CHECK_FALSE(tud_hid_n_report(0, 0x01, big.data(), (uint16_t)big.size()));
	std::vector<uint8_t> packet;
	CHECK(inPacket(packet) == Handshake::Nak);
}

TEST_CASE_FIXTURE(HidFixture, "an input report is sent after a rejected report that did not fit into the endpoint buffer (V28)" *
								  KNOWN_ISSUE("V28 (TinyUSB, vendor): tud_hid_n_report claims the IN endpoint (hid_device.c:115) and returns "
											  "on the failed tu_memcpy_s (:120, :123) without usbd_edpt_release: the endpoint stays claimed, "
											  "tud_hid_n_ready (busy only) stays true and every later report fails until a bus reset")) {
	start();
	std::vector<uint8_t> big(BUFSIZE, 7);
	REQUIRE_FALSE(tud_hid_n_report(0, 0x01, big.data(), (uint16_t)big.size()));
	REQUIRE(tud_hid_n_ready(0));
	const uint8_t small[] = {1, 2};
	REQUIRE_USB_MODELLED();
	CHECK(tud_hid_n_report(0, 0x01, small, sizeof(small))); // Defect: false, the endpoint is still claimed
}

TEST_CASE_FIXTURE(HidFixture, "the HID interface is not ready before the host has configured the device") {
	startDevice();
	CHECK_FALSE(tud_hid_n_ready(0));
	host.busReset();
	CHECK_FALSE(tud_hid_n_ready(0));
}

TEST_CASE_FIXTURE(HidFixture, "an input report is rejected while the device is not configured (V29)" *
								  KNOWN_ISSUE("V29 (TinyUSB, vendor): tud_hid_n_report does not check tud_ready / the interface; with "
											  "ep_in 0 (not configured, hid_device.c:115, 126) it claims EP0 OUT and arms a transfer of the "
											  "report buffer on EP0 OUT and returns true, although nothing is sent (the next SETUP resets "
											  "EP0). Latent: the firmware sends only after host requests (HidFFB) or checks tud_hid_n_ready")) {
	startDevice();
	host.busReset(); // Default state: EP0 open, no configuration
	const uint8_t data[] = {1, 2, 3};
	REQUIRE_USB_MODELLED();
	CHECK_FALSE(tud_hid_n_report(0, 0x01, data, sizeof(data))); // Defect: true
	CHECK_FALSE(HostDcd::endpoint(0x00).armed);
}

TEST_CASE_FIXTURE(HidFixture, "the keyboard report helper sends the modifier, a reserved byte and six key codes") {
	// Report helpers of hid_device.c that the firmware does not use (vendor API)
	start();
	const uint8_t keys[6] = {4, 5, 6, 0, 0, 0};
	REQUIRE(tud_hid_n_keyboard_report(0, 0x07, 0x02, keys));
	std::vector<uint8_t> packet;
	REQUIRE(inPacket(packet) == Handshake::Ack);
	CHECK(packet == std::vector<uint8_t>{0x07, 0x02, 0, 4, 5, 6, 0, 0, 0});
	process();
	REQUIRE(tud_hid_n_keyboard_report(0, 0, 0, nullptr)); // no key codes: all zero
	REQUIRE(inPacket(packet) == Handshake::Ack);
	CHECK(packet == std::vector<uint8_t>(8, 0));
}

TEST_CASE_FIXTURE(HidFixture, "the mouse, absolute mouse and gamepad report helpers send their report structs") {
	start();
	std::vector<uint8_t> packet;
	REQUIRE(tud_hid_n_mouse_report(0, 0, 0x01, 2, -3, 4, -5));
	REQUIRE(inPacket(packet) == Handshake::Ack);
	CHECK(packet == std::vector<uint8_t>{0x01, 2, 0xFD, 4, 0xFB});
	process();
	REQUIRE(tud_hid_n_abs_mouse_report(0, 0, 0x01, 0x1234, -2, 1, -1));
	REQUIRE(inPacket(packet) == Handshake::Ack);
	CHECK(packet == std::vector<uint8_t>{0x01, 0x34, 0x12, 0xFE, 0xFF, 1, 0xFF});
	process();
	REQUIRE(tud_hid_n_gamepad_report(0, 0, 1, 2, 3, 4, 5, 6, 7, 0x00010203));
	REQUIRE(inPacket(packet) == Handshake::Ack);
	CHECK(packet == std::vector<uint8_t>{1, 2, 3, 4, 5, 6, 7, 0x03, 0x02, 0x01, 0x00});
}

// ---------------------------------------------------------------- HidFFB (PID) over the real stack

TEST_CASE_FIXTURE(HidFixture, "the PID pool feature report read over USB reports the effect memory, the effect count and shared parameter blocks") {
	startWithFfb();
	UsbHost::ControlResult r = host.hidGetReport(HID_REPORT_TYPE_FEATURE, HID_ID_POOLREP, 5);
	REQUIRE(r.ok());
	REQUIRE(r.data.size() == 5);
	CHECK(r.data[0] == HID_ID_POOLREP);
	// ramPoolSize = effects * sizeof(FFB_Effect) (HidFFB.cpp:35). Host-vs-target: FFB_Effect holds pointers (filters),
	// so the value differs on the 32-bit target
	CHECK((r.data[1] | r.data[2] << 8) == (uint16_t)(MAX_EFFECTS * sizeof(FFB_Effect)));
	CHECK(r.data[3] == MAX_EFFECTS);
	CHECK(r.data[4] == 1);
}

TEST_CASE_FIXTURE(HidFixture, "a new effect is reported to the host in a PID state input report") {
	startWithFfb();
	REQUIRE(host.hidSetReport(HID_REPORT_TYPE_FEATURE, HID_ID_NEWEFREP, {HID_ID_NEWEFREP, FFB_EFFECT_CONSTANT, 0, 0}).ok());
	std::vector<uint8_t> packet;
	REQUIRE(inPacket(packet) == Handshake::Ack);
	REQUIRE(packet.size() == sizeof(reportFFB_status_t));
	CHECK(packet[0] == HID_ID_STATE);
}

TEST_CASE_FIXTURE(HidFixture, "a constant force effect created, set and started over USB plays in the effects calculator") {
	HidFFB &ffb = startWithFfb();
	REQUIRE(host.hidWrite({HID_ID_CTRLREP, 0x01}) == Handshake::Ack); // enable actuators
	process();
	REQUIRE(ffb.getFfbActive());
	const uint8_t index = createEffect(FFB_EFFECT_CONSTANT);
	REQUIRE(index >= 1);
	drainInputReports();

	FFB_SetEffect_t set;
	set.effectBlockIndex = index;
	set.effectType = FFB_EFFECT_CONSTANT;
	set.duration = 1000;
	set.enableAxis = 0x01;
	REQUIRE(host.hidWrite(HidFixture::bytes(set)) == Handshake::Ack);
	process();
	drainInputReports();
	FFB_SetConstantForce_Data_t cf{};
	cf.reportId = HID_ID_CONSTREP;
	cf.effectBlockIndex = index;
	cf.magnitude = 5000;
	REQUIRE(host.hidWrite(HidFixture::bytes(cf)) == Handshake::Ack);
	process();
	FFB_EffOp_Data_t op{HID_ID_EFOPREP, index, 1, 1}; // start, loop count 1
	REQUIRE(host.hidWrite(HidFixture::bytes(op)) == Handshake::Ack);
	process();

	const FFB_Effect &e = effects->effects[index - 1];
	CHECK(e.type == FFB_EFFECT_CONSTANT);
	CHECK(e.magnitude == 5000);
	CHECK(e.duration == 1000);
	CHECK(e.state == 1);
}

TEST_CASE_FIXTURE(HidFixture, "an effect freed with the PID block free report over USB is deleted") {
	startWithFfb();
	const uint8_t index = createEffect(FFB_EFFECT_SINE);
	REQUIRE(effects->effects[index - 1].type == FFB_EFFECT_SINE);
	drainInputReports();
	REQUIRE(host.hidWrite({HID_ID_BLKFRREP, index}) == Handshake::Ack);
	process();
	CHECK(effects->effects[index - 1].type == FFB_EFFECT_NONE);
}

TEST_CASE_FIXTURE(HidFixture, "characterization: GET_REPORT for the PID state input report does not return the report (NEEDS REVIEW 156)") {
	// NEEDS REVIEW 156: HidFFB::hidGet (HidFFB.cpp:167-188) answers only the block load and pool feature reports. HID 1.11
	// 7.2.1 makes GET_REPORT mandatory; input reports (PID state 0x02, gamepad 0x01) are only sent on the interrupt
	// endpoint. Today the host gets the id byte alone (V27) instead of a stall; this case only checks that no state
	// report comes back, so it holds with and without V27.
	startWithFfb();
	UsbHost::ControlResult r = host.hidGetReport(HID_REPORT_TYPE_INPUT, HID_ID_STATE, sizeof(reportFFB_status_t));
	CHECK_FALSE((r.ok() && r.data.size() == sizeof(reportFFB_status_t)));
}

// ---------------------------------------------------------------- HID command interface over the real stack

TEST_CASE_FIXTURE(HidFixture, "a HID command request on the interrupt endpoint is answered with a HID command input report") {
	start();
	auto hidCmd = std::make_unique<HID_CommandInterface>();
	// Run waits in the middle of its loop (while(!tud_hid_n_ready) Delay(1)): resumable like the CDC thread
	REQUIRE(HostRtos::makeResumable("HIDCMD") != nullptr);
	HID_CMD_Data_t request;
	request.type = HidCmdType::request;
	request.clsid = CMDCLSID_MAIN;
	request.instance = 0;
	request.cmd = (uint32_t)CommandHandlerCommands::id;
	REQUIRE(host.hidWrite(HidFixture::bytes(request)) == Handshake::Ack);
	process();
	std::vector<uint8_t> packet;
	REQUIRE(inPacket(packet) == Handshake::Ack);
	REQUIRE(packet.size() == sizeof(HID_CMD_Data_t));
	HID_CMD_Data_t reply;
	std::memcpy(&reply, packet.data(), sizeof(reply));
	CHECK(reply.reportId == HID_ID_HIDCMD);
	CHECK(reply.type == HidCmdType::request);
	CHECK(uint16_t(reply.clsid) == CMDCLSID_MAIN);
	CHECK(uint32_t(reply.cmd) == (uint32_t)CommandHandlerCommands::id);
	CHECK(uint64_t(reply.data) == main->getInfo().id);
	hidCmd.reset();
}

TEST_CASE_FIXTURE(HidFixture, "a HID command for a class that does not exist is answered with not found") {
	start();
	auto hidCmd = std::make_unique<HID_CommandInterface>();
	REQUIRE(HostRtos::makeResumable("HIDCMD") != nullptr);
	HID_CMD_Data_t request;
	request.type = HidCmdType::request;
	request.clsid = 0x7FF;
	request.instance = 0;
	request.cmd = 1;
	REQUIRE(host.hidWrite(HidFixture::bytes(request)) == Handshake::Ack);
	process();
	std::vector<uint8_t> packet;
	REQUIRE(inPacket(packet) == Handshake::Ack);
	REQUIRE(packet.size() == sizeof(HID_CMD_Data_t));
	CHECK(packet[0] == HID_ID_HIDCMD);
	CHECK(packet[1] == (uint8_t)HidCmdType::notFound);
	hidCmd.reset();
}

} // TEST_SUITE
