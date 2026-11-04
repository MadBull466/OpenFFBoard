/*
 * TinyUSB device core (FFBoard/USB/device/usbd.c and the helpers of tusb.c, vendor tier B: tested, never fixed) on the
 * fake device controller (support/HostDcd.h) with the firmware's descriptors and callbacks, driven by the simulated
 * host (support/UsbHost.h). Reference: USB 2.0 chapter 9 (device states 9.1, standard requests 9.4) and the
 * documentation of the TinyUSB device API (usbd.h, usbd_pvt.h).
 *
 * Application class driver: TinyUSB lets the application add class drivers (usbd_app_driver_get_cb, usbd_pvt.h:66-69,
 * read by tud_rhport_init, usbd.c:478-480). This file defines that callback for a recording test driver that claims
 * vendor-specific interfaces (class 0xFF) and uses the endpoint API of usbd_pvt.h like a class driver. It is inert
 * unless a test enables it (setAppDriver): then usbd_app_driver_get_cb returns no driver and the stack is exactly as
 * without the callback (_app_driver_count 0), so the other ffb_usb_tests cases are not affected. Enabled tests use the
 * vendor configuration below (one vendor interface with bulk OUT 0x03 / IN 0x83) or the firmware's configurations
 * (the test driver ignores their interfaces). test_usbd_control.cpp uses it through setAppDriver and
 * vendorConfiguration (declared there).
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "UsbFixture.h"

#include "common/tusb_private.h"
#include "device/usbd_pvt.h"
#include "ffb_defs.h"

#include <cstring>
#include <string>
#include <vector>

// ---------------- Test class driver ----------------

namespace a_usbcore {

using ControlHook = bool (*)(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request);

struct AppDriverState {
	bool enabled = false;
	ControlHook control = nullptr; //!< control_xfer_cb of the driver; nullptr: every request is refused (false)
	uint32_t inits = 0, resets = 0, opens = 0, sofs = 0;
	uint8_t epOut = 0, epIn = 0; //!< Endpoints opened by open()
	struct Xfer {
		uint8_t ep;
		xfer_result_t result;
		uint32_t bytes;
	};
	std::vector<Xfer> xfers;							//!< xfer_cb calls
	std::vector<tusb_control_request_t> controlRequests; //!< control_xfer_cb calls (any stage)
	std::vector<uint8_t> controlStages;
	bool answerStandardEndpointRequests = false; //!< control: answer forwarded standard endpoint requests with a status
};
AppDriverState appDriver;

void setAppDriver(bool enabled, ControlHook control) {
	appDriver = AppDriverState();
	appDriver.enabled = enabled;
	appDriver.control = control;
}

namespace {
void drvInit() { appDriver.inits++; }
void drvReset(uint8_t) { appDriver.resets++; }
uint16_t drvOpen(uint8_t rhport, tusb_desc_interface_t const *itf, uint16_t maxLen) {
	appDriver.opens++;
	if (itf->bInterfaceClass != TUSB_CLASS_VENDOR_SPECIFIC) {
		return 0;
	}
	const uint16_t len = (uint16_t)(sizeof(tusb_desc_interface_t) + itf->bNumEndpoints * sizeof(tusb_desc_endpoint_t));
	if (len > maxLen) {
		return 0;
	}
	if (itf->bNumEndpoints == 2 &&
		!usbd_open_edpt_pair(rhport, tu_desc_next(itf), 2, TUSB_XFER_BULK, &appDriver.epOut, &appDriver.epIn)) {
		return 0;
	}
	return len;
}
bool drvControl(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
	appDriver.controlRequests.push_back(*request);
	appDriver.controlStages.push_back(stage);
	if (appDriver.answerStandardEndpointRequests && request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
		request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT) {
		return stage == CONTROL_STAGE_SETUP ? tud_control_status(rhport, request) : true;
	}
	return appDriver.control != nullptr && appDriver.control(rhport, stage, request);
}
bool drvXfer(uint8_t, uint8_t ep, xfer_result_t result, uint32_t bytes) {
	appDriver.xfers.push_back({ep, result, bytes});
	return true;
}
void drvSof(uint8_t, uint32_t) { appDriver.sofs++; }

const usbd_class_driver_t appDrivers[] = {{"TEST", drvInit, nullptr, drvReset, drvOpen, drvControl, drvXfer, drvSof}};

const uint8_t vendorConf[] = {
	TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP | TUSB_DESC_CONFIG_ATT_SELF_POWERED, 100),
	TUD_VENDOR_DESCRIPTOR(0, 0, 0x03, 0x83, 64)};
} // namespace

const uint8_t *vendorConfiguration() { return vendorConf; }

} // namespace a_usbcore

extern "C" usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
	if (!a_usbcore::appDriver.enabled) {
		*driver_count = 0;
		return nullptr;
	}
	*driver_count = 1;
	return a_usbcore::appDrivers;
}

namespace {

using a_usbcore::appDriver;
using Handshake = HostDcd::Handshake;

/** The firmware's CDC configuration without the remote wakeup and self-powered attributes */
const uint8_t *cdcConfNoWakeup() {
	static uint8_t conf[TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN];
	static bool made = false;
	if (!made) {
		std::memcpy(conf, usb_cdc_conf, sizeof(conf));
		conf[7] = 0x80; // bmAttributes: reserved bit only (bus powered, no remote wakeup)
		made = true;
	}
	return conf;
}

/** CDC (interfaces 0, 1) followed by a vendor interface that reuses number 1 */
const uint8_t duplicateItfConf[] = {
	TUD_CONFIG_DESCRIPTOR(1, 3, 0, TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_VENDOR_DESC_LEN, 0x80, 100),
	TUD_CDC_DESCRIPTOR(0, 4, 0x82, 8, 0x01, 0x81, 64), TUD_VENDOR_DESCRIPTOR(1, 0, 0x03, 0x83, 64)};

/** A configuration descriptor with the wrong descriptor type */
const uint8_t wrongTypeConf[] = {9, TUSB_DESC_INTERFACE, 9, 0, 0, 1, 0, 0x80, 50};

struct UsbdCoreFixture : UsbFixture {
	~UsbdCoreFixture() override { a_usbcore::setAppDriver(false, nullptr); }

	/** Starts the device with the vendor configuration and the test class driver */
	void startVendorDevice() {
		a_usbcore::setAppDriver(true, nullptr);
		startDevice(a_usbcore::vendorConfiguration());
	}
	UsbHost::ControlResult req(uint8_t type, uint8_t request, uint16_t value, uint16_t index, uint16_t length) {
		return host.control(UsbHost::request(type, request, value, index, length));
	}
	UsbHost::ControlResult getStatus(uint8_t recipient, uint16_t index) {
		return req((uint8_t)(0x80 | recipient), TUSB_REQ_GET_STATUS, 0, index, 2);
	}
	UsbHost::ControlResult feature(bool set, uint8_t recipient, uint16_t selector, uint16_t index) {
		return req(recipient, set ? TUSB_REQ_SET_FEATURE : TUSB_REQ_CLEAR_FEATURE, selector, index, 0);
	}
	/** One IN transaction (with NAK retries); the data of the packet */
	std::vector<uint8_t> inPacket(uint8_t ep, Handshake expected = Handshake::Ack) {
		std::vector<uint8_t> data;
		CHECK(host.in(ep, data) == expected);
		process();
		return data;
	}
};

/** An endpoint descriptor (USB 2.0 9.6.6) */
tusb_desc_endpoint_t epDesc(uint8_t addr, uint8_t type, uint16_t size, uint8_t interval = 0) {
	const uint8_t raw[7] = {7, TUSB_DESC_ENDPOINT, addr, type, (uint8_t)size, (uint8_t)(size >> 8), interval};
	tusb_desc_endpoint_t d;
	std::memcpy(&d, raw, sizeof(raw));
	return d;
}

constexpr uint8_t RCPT_DEVICE = 0, RCPT_INTERFACE = 1, RCPT_ENDPOINT = 2;
const std::vector<uint8_t> status0{0, 0}, status1{1, 0}, status3{3, 0};

} // namespace

TEST_SUITE("UsbdCore") {

// ---------------- Device states (USB 2.0 9.1.1) ----------------

TEST_CASE_FIXTURE(UsbdCoreFixture, "a bus reset puts the device into the default state at address 0 with full speed") {
	startDevice();
	enumerate();
	host.busReset();
	CHECK_FALSE(tud_mounted());
	CHECK_FALSE(tud_connected()); // connected again only after the first SETUP (usbd.c:610-612)
	CHECK(tud_speed_get() == TUSB_SPEED_FULL);
	CHECK(HostDcd::state().address == 0);
	CHECK(host.getDescriptor(TUSB_DESC_DEVICE, 0, 18).ok()); // answers at the default address
	CHECK(tud_connected());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "characterization: a bus reset of a configured device unmounts it without the unmount callback") {
	// NEEDS REVIEW 152: usbd.c:589-593 resets the configuration without tud_umount_cb (only SET_CONFIGURATION(0) and
	// the unplug event call it), so the firmware's main class gets no usbSuspend for the lost configuration and sees
	// two usbResume calls (mount callbacks) in a row after re-enumeration. Intended?
	startDevice();
	enumerate();
	host.busReset();
	CHECK_FALSE(tud_mounted());
	CHECK(main->suspends == 0);
	enumerate();
	CHECK(main->resumes == 2);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a suspend and a resume before the first SETUP after a bus reset are ignored") {
	startDevice();
	host.busReset();
	HostDcd::suspend(); // e.g. unstable lines while plugging in (usbd.c:1164-1172)
	process();
	CHECK_FALSE(tud_suspended());
	HostDcd::resume();
	process();
	CHECK(main->resumes == 0);
	CHECK(main->suspends == 0);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "in the address state the device is not mounted and GET_CONFIGURATION returns 0") {
	startDevice();
	REQUIRE(host.enumerate(7, false));
	CHECK(HostDcd::state().address == 7);
	CHECK_FALSE(tud_mounted());
	CHECK(host.getConfiguration().data == std::vector<uint8_t>{0});
	CHECK_FALSE(HostDcd::endpoint(0x81).enabled);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "SET_CONFIGURATION opens the endpoints of the configuration and calls the mount callback") {
	startDevice();
	enumerate();
	for (uint8_t ep : {0x01, 0x81, 0x82, 0x83, 0x02}) {
		INFO("endpoint " << (int)ep);
		CHECK(HostDcd::endpoint(ep).enabled);
	}
	CHECK(HostDcd::endpoint(0x82).type == TUSB_XFER_INTERRUPT);
	CHECK(HostDcd::endpoint(0x01).type == TUSB_XFER_BULK);
	CHECK(HostDcd::endpoint(0x01).armed); // CDC receive armed by the class driver
	CHECK(main->resumes == 1);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "SET_CONFIGURATION(0) returns to the address state, closes the endpoints and calls the unmount callback") {
	startDevice();
	enumerate();
	REQUIRE(host.setConfiguration(0).ok());
	CHECK_FALSE(tud_mounted());
	CHECK(main->suspends == 1); // tud_umount_cb -> usbSuspend
	CHECK(HostDcd::state().closeAllCalls == 1);
	CHECK(host.getConfiguration().data == std::vector<uint8_t>{0});
	CHECK(host.out(0x01, {'x'}) == Handshake::NoResponse); // endpoint disabled
	CHECK(HostDcd::state().address == 5);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "SET_CONFIGURATION with the active configuration value keeps the configuration") {
	startDevice();
	enumerate();
	REQUIRE(host.setConfiguration(1).ok());
	CHECK(tud_mounted());
	CHECK(HostDcd::state().closeAllCalls == 0);
	CHECK(main->resumes == 1); // no second mount callback
	CHECK(HostDcd::endpoint(0x01).armed);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "the device works again after its configuration was set to 0 and back to 1") {
	startDevice();
	enumerate();
	REQUIRE(host.setConfiguration(0).ok());
	REQUIRE(host.setConfiguration(1).ok());
	CHECK(tud_mounted());
	CHECK(main->resumes == 2);
	REQUIRE(host.cdcSetControlLineState(true, true).ok());
	REQUIRE(host.cdcWrite("sys.main?\n") == Handshake::Ack);
	process();
	CHECK(host.cdcRead() == "[sys.main?|0]\n");
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "SET_CONFIGURATION with a configuration value the device does not have is rejected (bug 134)" * 
								  KNOWN_ISSUE("bug 134 (firmware USB glue): USBdevice::getUsbConfigurationDesc (USBdevice.cpp:55-57, "
													   "called by tud_descriptor_configuration_cb, global_callbacks.cpp:282-284) ignores the index and "
													   "returns configuration 1 for every index; process_set_config (usbd.c:970-971) only rejects a NULL "
													   "descriptor, so SET_CONFIGURATION(2) mounts configuration 1 as configuration 2 instead of a Request "
													   "Error (USB 2.0 9.4.7; bNumConfigurations is 1)")) {
	startDevice();
	REQUIRE(host.enumerate(5, false));
	UsbHost::ControlResult r = host.setConfiguration(2);
	REQUIRE_USB_MODELLED();
	CHECK(r.stalled()); // Defect: acknowledged
	CHECK_FALSE(tud_mounted());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "GET_DESCRIPTOR of a configuration index the device does not have is rejected (bug 134)" * 
								  KNOWN_ISSUE("bug 134 (firmware USB glue): USBdevice::getUsbConfigurationDesc (USBdevice.cpp:55-57) "
													   "ignores the index: GET_DESCRIPTOR(CONFIGURATION, 1) returns configuration 0 instead of a Request "
													   "Error (USB 2.0 9.4.3, 9.6.1 bNumConfigurations 1)")) {
	startDevice();
	enumerate();
	UsbHost::ControlResult r = host.getDescriptor(TUSB_DESC_CONFIGURATION, 1, 9);
	REQUIRE_USB_MODELLED();
	CHECK(r.stalled()); // Defect: the descriptor of configuration index 0
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a configuration with an interface no class driver supports is rejected") {
	startDevice(a_usbcore::vendorConfiguration()); // vendor interface, test driver not enabled
	REQUIRE(host.enumerate(5, false));
	CHECK(host.setConfiguration(1).stalled()); // usbd.c:1052-1053
	CHECK_FALSE(tud_mounted());
	CHECK(main->resumes == 0);
	CHECK(host.getConfiguration().data == std::vector<uint8_t>{0});
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a configuration that uses an interface number twice is rejected") {
	a_usbcore::setAppDriver(true, nullptr);
	startDevice(duplicateItfConf);
	REQUIRE(host.enumerate(5, false));
	CHECK(host.setConfiguration(1).stalled()); // usbd.c:1037-1038
	CHECK_FALSE(tud_mounted());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a configuration descriptor with the wrong descriptor type is rejected") {
	startDevice(wrongTypeConf);
	REQUIRE(host.enumerate(5, false));
	CHECK(host.setConfiguration(1).stalled()); // usbd.c:971
	CHECK_FALSE(tud_mounted());
}

// ---------------- Bus reset, unplug, disconnect ----------------

TEST_CASE_FIXTURE(UsbdCoreFixture, "a bus reset in the middle of a control read lets the host enumerate again") {
	startDevice();
	host.busReset();
	REQUIRE(HostDcd::setup(0, UsbHost::request(0x80, TUSB_REQ_GET_DESCRIPTOR, TUSB_DESC_CONFIGURATION << 8, 0, 255)) == Handshake::Ack);
	process();
	std::vector<uint8_t> packet;
	REQUIRE(HostDcd::in(0, 0, packet) == Handshake::Ack);
	REQUIRE(packet.size() == 64); // first of two packets
	enumerate();
	CHECK(host.configDescriptor.size() == TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_HID_INOUT_DESC_LEN);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a bus reset drops a pending transfer and the endpoint accepts a new one after re-enumeration") {
	startVendorDevice();
	enumerate();
	uint8_t data[4] = {1, 2, 3, 4};
	REQUIRE(usbd_edpt_xfer(0, 0x83, data, 4));
	REQUIRE(usbd_edpt_busy(0, 0x83));
	host.busReset();
	CHECK_FALSE(usbd_edpt_busy(0, 0x83));
	enumerate();
	uint8_t data2[2] = {7, 8};
	REQUIRE(usbd_edpt_xfer(0, 0x83, data2, 2));
	CHECK(inPacket(3) == std::vector<uint8_t>{7, 8});
	REQUIRE(appDriver.xfers.size() == 1); // only the second transfer completed
	CHECK(appDriver.xfers[0].bytes == 2);
	CHECK(appDriver.resets >= 2);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "unplugging the cable unmounts the device and calls the unmount callback") {
	startDevice();
	enumerate();
	HostDcd::detach();
	process();
	CHECK_FALSE(tud_mounted());
	CHECK_FALSE(tud_connected());
	CHECK(main->suspends == 1);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "after unplugging and plugging in again the device enumerates again") {
	startDevice();
	enumerate();
	HostDcd::detach();
	process();
	HostDcd::attach();
	enumerate();
	CHECK(main->resumes == 2);
	REQUIRE(host.cdcSetControlLineState(true, true).ok());
	REQUIRE(host.cdcWrite("sys.main?\n") == Handshake::Ack);
	process();
	CHECK(host.cdcRead() == "[sys.main?|0]\n");
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "tud_disconnect removes the pull-up so that the host loses the device") {
	startDevice();
	enumerate();
	CHECK(tud_disconnect());
	CHECK_FALSE(HostDcd::state().pullup);
	CHECK(host.getConfiguration().status == Handshake::NoResponse);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "characterization: after tud_disconnect TinyUSB keeps the configured state until the next bus reset") {
	// NEEDS REVIEW 153: the pull-up off raises no controller event (dcd_dwc2.c:515-531), so tud_mounted() stays true and
	// no unmount callback runs while the device is off the bus. Should tud_disconnect reset the device state?
	startDevice();
	enumerate();
	REQUIRE(tud_disconnect());
	process();
	CHECK(tud_mounted());
	CHECK(main->suspends == 0);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "tud_connect after tud_disconnect makes the device visible again and it enumerates") {
	startDevice();
	enumerate();
	REQUIRE(tud_disconnect());
	CHECK(tud_connect());
	CHECK(HostDcd::state().pullup);
	CHECK_FALSE(HostDcd::reachable()); // the host resets a newly connected device first
	enumerate();
	CHECK(main->resumes == 2);
}

// ---------------- Suspend, resume, remote wakeup (USB 2.0 7.1.7.6, 7.1.7.7, 9.4.1) ----------------

TEST_CASE_FIXTURE(UsbdCoreFixture, "tud_remote_wakeup is refused while the bus is not suspended") {
	startDevice();
	enumerate();
	REQUIRE(feature(true, RCPT_DEVICE, TUSB_REQ_FEATURE_REMOTE_WAKEUP, 0).ok());
	CHECK_FALSE(tud_remote_wakeup());
	CHECK(HostDcd::state().remoteWakeups == 0);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "tud_remote_wakeup is refused when the host has not enabled remote wakeup") {
	startDevice();
	enumerate();
	HostDcd::suspend();
	process();
	REQUIRE(tud_suspended());
	CHECK_FALSE(tud_remote_wakeup());
	CHECK(HostDcd::state().remoteWakeups == 0);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "tud_remote_wakeup is refused after the host disabled remote wakeup again") {
	startDevice();
	enumerate();
	REQUIRE(feature(true, RCPT_DEVICE, TUSB_REQ_FEATURE_REMOTE_WAKEUP, 0).ok());
	REQUIRE(feature(false, RCPT_DEVICE, TUSB_REQ_FEATURE_REMOTE_WAKEUP, 0).ok());
	HostDcd::suspend();
	process();
	CHECK_FALSE(tud_remote_wakeup());
	CHECK(HostDcd::state().remoteWakeups == 0);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "tud_remote_wakeup signals resume when suspended and enabled, and the next SOF resumes the device") {
	startDevice();
	enumerate();
	REQUIRE(feature(true, RCPT_DEVICE, TUSB_REQ_FEATURE_REMOTE_WAKEUP, 0).ok());
	HostDcd::suspend();
	process();
	REQUIRE(tud_suspended());
	const uint32_t resumes = main->resumes;
	CHECK(tud_remote_wakeup());
	CHECK(HostDcd::state().remoteWakeups == 1);
	CHECK(tud_suspended()); // until the bus is active again
	HostDcd::sof();			// the host resumed the bus: the SOF interrupt enabled for the wakeup ends the suspend
	process();
	CHECK_FALSE(tud_suspended());
	CHECK(main->resumes == resumes + 1); // usbd.c:1194-1199 -> tud_resume_cb
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "tud_remote_wakeup is refused for a configuration without the remote wakeup attribute") {
	startDevice(cdcConfNoWakeup());
	enumerate();
	REQUIRE(feature(true, RCPT_DEVICE, TUSB_REQ_FEATURE_REMOTE_WAKEUP, 0).ok());
	HostDcd::suspend();
	process();
	REQUIRE(tud_suspended());
	CHECK_FALSE(tud_remote_wakeup());
	CHECK(HostDcd::state().remoteWakeups == 0);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "with the SOF callback enabled an SOF ends a suspend") {
	startDevice();
	enumerate();
	tud_sof_cb_enable(true);
	CHECK(HostDcd::state().sofInterrupt);
	HostDcd::suspend();
	process();
	REQUIRE(tud_suspended());
	HostDcd::sof();
	process();
	CHECK_FALSE(tud_suspended());
	tud_sof_cb_enable(false);
	CHECK_FALSE(HostDcd::state().sofInterrupt);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "SOF events reach TinyUSB and the class drivers only while the SOF interrupt is enabled") {
	startVendorDevice();
	enumerate();
	HostDcd::sof();
	process();
	CHECK(appDriver.sofs == 0);
	tud_sof_cb_enable(true);
	HostDcd::sof();
	HostDcd::sof();
	process();
	CHECK(appDriver.sofs == 2); // the driver's sof handler runs in the interrupt (usbd.c:1185-1190)
	tud_sof_cb_enable(false);
	HostDcd::sof();
	process();
	CHECK(appDriver.sofs == 2);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a change of the configuration turns the SOF interrupt off") {
	startDevice();
	enumerate();
	tud_sof_cb_enable(true);
	REQUIRE(HostDcd::state().sofInterrupt);
	REQUIRE(host.setConfiguration(0).ok());
	CHECK_FALSE(HostDcd::state().sofInterrupt); // usbd.c:775
}

// ---------------- Standard device requests (USB 2.0 9.4) ----------------

TEST_CASE_FIXTURE(UsbdCoreFixture, "GET_STATUS of the configured device reports self powered and the remote wakeup state") {
	startDevice();
	enumerate();
	CHECK(getStatus(RCPT_DEVICE, 0).data == status1);
	REQUIRE(feature(true, RCPT_DEVICE, TUSB_REQ_FEATURE_REMOTE_WAKEUP, 0).ok());
	CHECK(getStatus(RCPT_DEVICE, 0).data == status3);
	REQUIRE(feature(false, RCPT_DEVICE, TUSB_REQ_FEATURE_REMOTE_WAKEUP, 0).ok());
	CHECK(getStatus(RCPT_DEVICE, 0).data == status1);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "GET_STATUS of a bus-powered configuration reports bus powered") {
	startDevice(cdcConfNoWakeup());
	enumerate();
	CHECK(getStatus(RCPT_DEVICE, 0).data == status0);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "characterization: GET_STATUS of the device reports bus powered before the configuration is set") {
	// NEEDS REVIEW 154: the self-powered bit is taken from the configuration descriptor only at SET_CONFIGURATION
	// (usbd.c:974-975, cleared by every bus reset); in the address state the self-powered F407 board reports bus powered
	// (USB 2.0 9.4.5: "whether the device is currently self-powered"). Acceptable?
	startDevice();
	REQUIRE(host.enumerate(5, false));
	CHECK(getStatus(RCPT_DEVICE, 0).data == status0);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "SET_FEATURE(TEST_MODE) is rejected by the full-speed device") {
	startDevice();
	enumerate();
	// USB 2.0 9.4.9 / 7.1.20: test mode is required for high-speed devices only (CFG_TUD_TEST_MODE is off)
	CHECK(feature(true, RCPT_DEVICE, TUSB_REQ_FEATURE_TEST_MODE, 0x0100).stalled());
	CHECK(host.getConfiguration().ok());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "CLEAR_FEATURE of a device feature other than remote wakeup is rejected") {
	startDevice();
	enumerate();
	CHECK(feature(false, RCPT_DEVICE, TUSB_REQ_FEATURE_TEST_MODE, 0).stalled()); // test mode cannot be cleared (9.4.1)
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "unknown and unsupported standard device requests are rejected") {
	startDevice();
	enumerate();
	CHECK(req(0x80, TUSB_REQ_SYNCH_FRAME, 0, 0, 2).stalled()); // SYNCH_FRAME is an endpoint request
	CHECK(req(0x00, 0x0F, 0, 0, 0).stalled());
	CHECK(req(0x80, TUSB_REQ_GET_INTERFACE, 0, 0, 1).stalled()); // interface request sent to the device
	CHECK(host.getConfiguration().data == std::vector<uint8_t>{1});
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a vendor request is rejected when the firmware has no vendor handler") {
	startDevice();
	enumerate();
	CHECK(req(0xC0, 0x01, 0, 0, 4).stalled()); // tud_vendor_control_xfer_cb: weak default (usbd.c:84-89)
	CHECK(req(0x40, 0x01, 0, 0, 0).stalled());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a request with the reserved type is rejected") {
	startDevice();
	enumerate();
	CHECK(req(0xE0, TUSB_REQ_GET_STATUS, 0, 0, 2).stalled()); // bmRequestType type 3 (usbd.c:713)
	CHECK(host.getConfiguration().ok());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a request to the recipient other is rejected") {
	startDevice();
	enumerate();
	CHECK(req(0x83, TUSB_REQ_GET_STATUS, 0, 0, 2).stalled()); // recipient 3 (usbd.c:956-959)
	CHECK(host.getConfiguration().ok());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a class request to the device is routed to the class driver of the interface in wIndex") {
	startDevice();
	enumerate();
	REQUIRE(host.cdcSetLineCoding(115200, 0, 0, 8).ok());
	UsbHost::ControlResult r = req(0xA0, CDC_REQUEST_GET_LINE_CODING, 0, 0, 7); // recipient device, interface 0
	REQUIRE(r.ok());
	CHECK(r.data == std::vector<uint8_t>{0x00, 0xC2, 0x01, 0x00, 0, 0, 8});
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a class request to the device for an interface that does not exist is rejected") {
	startDevice();
	enumerate();
	CHECK(req(0xA0, CDC_REQUEST_GET_LINE_CODING, 0, 5, 7).stalled());  // no driver for interface 5
	CHECK(req(0xA0, CDC_REQUEST_GET_LINE_CODING, 0, 20, 7).stalled()); // beyond CFG_TUD_INTERFACE_MAX
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "class requests to the device, an interface and an endpoint reach the class driver that owns them") {
	startVendorDevice();
	enumerate();
	CHECK(req(0x20, 0x01, 0, 0, 0).stalled()); // device recipient, interface 0; the test driver refuses everything
	CHECK(req(0x21, 0x02, 0, 0, 0).stalled()); // interface 0
	CHECK(req(0x22, 0x03, 0, 0x83, 0).stalled()); // endpoint 0x83
	REQUIRE(appDriver.controlRequests.size() == 3);
	CHECK(appDriver.controlRequests[0].bRequest == 0x01);
	CHECK(appDriver.controlRequests[1].bRequest == 0x02);
	CHECK(appDriver.controlRequests[2].bRequest == 0x03);
	CHECK(appDriver.controlStages[2] == CONTROL_STAGE_SETUP);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a class request to an endpoint without a class driver is rejected") {
	startVendorDevice();
	enumerate();
	CHECK(req(0x22, 0x03, 0, 0x81, 0).stalled()); // endpoint 0x81 is not in the configuration (usbd.c:911)
	CHECK(appDriver.controlRequests.empty());
}

// ---------------- Standard interface requests (USB 2.0 9.4.4, 9.4.5, 9.4.10) ----------------

TEST_CASE_FIXTURE(UsbdCoreFixture, "GET_INTERFACE returns alternate setting 0 for interfaces without alternate settings") {
	startDevice();
	enumerate();
	for (uint16_t itf : {0, 1, 2}) {
		INFO("interface " << itf);
		UsbHost::ControlResult r = req(0x81, TUSB_REQ_GET_INTERFACE, 0, itf, 1);
		REQUIRE(r.ok());
		CHECK(r.data == std::vector<uint8_t>{0});
	}
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "SET_INTERFACE with alternate setting 0 is accepted") {
	startDevice();
	enumerate();
	CHECK(req(0x01, TUSB_REQ_SET_INTERFACE, 0, 0, 0).ok());
	CHECK(req(0x01, TUSB_REQ_SET_INTERFACE, 0, 2, 0).ok());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "SET_INTERFACE with an alternate setting the interface does not have is rejected (V22)" * 
								  KNOWN_ISSUE("V22 (TinyUSB, vendor): when the class driver refuses a standard interface request, "
													   "the fallback of usbd.c:876-895 acknowledges SET_INTERFACE for any alternate setting without "
													   "checking the descriptor (USB 2.0 9.4.10: Request Error if the alternate setting does not exist)")) {
	startDevice();
	enumerate();
	UsbHost::ControlResult r = req(0x01, TUSB_REQ_SET_INTERFACE, 1, 0, 0);
	REQUIRE_USB_MODELLED();
	CHECK(r.stalled()); // Defect: acknowledged
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "GET_STATUS of an interface returns two zero bytes (V22)" * 
								  KNOWN_ISSUE("V22 (TinyUSB, vendor): the fallback for standard interface requests the class driver "
													   "refuses (usbd.c:876-895) handles only GET_INTERFACE / SET_INTERFACE: GET_STATUS of an interface is "
													   "stalled although USB 2.0 9.4.5 requires two zero bytes in the configured state (the CDC and HID "
													   "drivers do not answer it)")) {
	startDevice();
	enumerate();
	UsbHost::ControlResult r = getStatus(RCPT_INTERFACE, 0);
	REQUIRE_USB_MODELLED();
	CHECK(r.ok()); // Defect: STALL
	CHECK(r.data == status0);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a standard request to an interface that does not exist is rejected") {
	startDevice();
	enumerate();
	CHECK(req(0x81, TUSB_REQ_GET_INTERFACE, 0, 3, 1).stalled());
	CHECK(req(0x81, TUSB_REQ_GET_INTERFACE, 0, 16, 1).stalled()); // beyond CFG_TUD_INTERFACE_MAX (usbd.c:868)
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "interface requests in the address state are rejected") {
	startDevice();
	REQUIRE(host.enumerate(5, false));
	CHECK(req(0x81, TUSB_REQ_GET_INTERFACE, 0, 0, 1).stalled()); // USB 2.0 9.4.4: Request Error in the address state
	CHECK(req(0x01, TUSB_REQ_SET_INTERFACE, 0, 0, 0).stalled());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "SET_FEATURE and CLEAR_FEATURE to an interface are rejected") {
	startDevice();
	enumerate();
	CHECK(feature(true, RCPT_INTERFACE, 0, 0).stalled()); // USB 2.0 defines no interface features (Table 9-6)
	CHECK(feature(false, RCPT_INTERFACE, 0, 2).stalled());
}

// ---------------- Standard endpoint requests (USB 2.0 9.4.1, 9.4.5, 9.4.9) ----------------

TEST_CASE_FIXTURE(UsbdCoreFixture, "GET_STATUS of an endpoint reports whether it is halted") {
	startDevice();
	enumerate();
	CHECK(getStatus(RCPT_ENDPOINT, 0x81).data == status0);
	REQUIRE(feature(true, RCPT_ENDPOINT, TUSB_REQ_FEATURE_EDPT_HALT, 0x81).ok());
	CHECK(getStatus(RCPT_ENDPOINT, 0x81).data == status1);
	CHECK(HostDcd::endpoint(0x81).stalled);
	REQUIRE(feature(false, RCPT_ENDPOINT, TUSB_REQ_FEATURE_EDPT_HALT, 0x81).ok());
	CHECK(getStatus(RCPT_ENDPOINT, 0x81).data == status0);
	CHECK_FALSE(HostDcd::endpoint(0x81).stalled);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a halted IN endpoint answers STALL and after CLEAR_FEATURE sends again starting with DATA0") {
	startVendorDevice();
	enumerate();
	uint8_t a[2] = {1, 2};
	REQUIRE(usbd_edpt_xfer(0, 0x83, a, 2));
	REQUIRE(inPacket(3) == std::vector<uint8_t>{1, 2}); // DATA0; the next packet would be DATA1
	REQUIRE(feature(true, RCPT_ENDPOINT, TUSB_REQ_FEATURE_EDPT_HALT, 0x83).ok());
	CHECK(usbd_edpt_stalled(0, 0x83));
	inPacket(3, Handshake::Stall);
	REQUIRE(feature(false, RCPT_ENDPOINT, TUSB_REQ_FEATURE_EDPT_HALT, 0x83).ok());
	CHECK_FALSE(usbd_edpt_stalled(0, 0x83));
	uint8_t b[1] = {3};
	REQUIRE(usbd_edpt_xfer(0, 0x83, b, 1));
	CHECK(inPacket(3) == std::vector<uint8_t>{3});
	CHECK_FALSE(HostDcd::state().packets.back().data1); // USB 2.0 9.4.5: CLEAR_FEATURE(HALT) resets the toggle to DATA0
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "the halt of an endpoint is forwarded to its class driver") {
	startVendorDevice();
	enumerate();
	REQUIRE(feature(true, RCPT_ENDPOINT, TUSB_REQ_FEATURE_EDPT_HALT, 0x03).ok());
	REQUIRE(appDriver.controlRequests.size() == 1); // usbd.c:932-939 (e.g. USBTMC resets its buffers)
	CHECK(appDriver.controlRequests[0].bRequest == TUSB_REQ_SET_FEATURE);
	CHECK(uint16_t(appDriver.controlRequests[0].wIndex) == 0x03);
	CHECK(HostDcd::endpoint(0x03).stalled);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a class driver that answers a forwarded endpoint request itself gets no second status stage") {
	startVendorDevice();
	appDriver.answerStandardEndpointRequests = true;
	enumerate();
	CHECK(feature(false, RCPT_ENDPOINT, TUSB_REQ_FEATURE_EDPT_HALT, 0x83).ok()); // usbd.c:941-942
	CHECK(HostDcd::endpoint(0x80).transfers == HostDcd::endpoint(0x80).completions); // one status ZLP, no extra
	CHECK(host.getConfiguration().ok());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "an endpoint request for an endpoint number the stack does not have is rejected") {
	startDevice();
	enumerate();
	CHECK(getStatus(RCPT_ENDPOINT, 0x87).stalled()); // usbd.c:906: CFG_TUD_ENDPPOINT_MAX 6 for the STM32F4
	CHECK(feature(true, RCPT_ENDPOINT, TUSB_REQ_FEATURE_EDPT_HALT, 0x07).stalled());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "unsupported standard endpoint requests are rejected") {
	startDevice();
	enumerate();
	CHECK(req(0x82, TUSB_REQ_SYNCH_FRAME, 0, 0x81, 2).stalled()); // only for isochronous endpoints (USB 2.0 9.4.11)
	CHECK(req(0x82, TUSB_REQ_GET_DESCRIPTOR, TUSB_DESC_DEVICE << 8, 0x81, 18).stalled());
	CHECK(host.getConfiguration().ok());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "GET_STATUS of an endpoint that is not in the configuration is rejected (V23)" * 
								  KNOWN_ISSUE("V23 (TinyUSB, vendor): standard endpoint requests (usbd.c:901-953) only check the "
													   "endpoint number against CFG_TUD_ENDPPOINT_MAX, not whether the endpoint belongs to the "
													   "configuration: GET_STATUS of endpoint 0x03 (exists in the core, not in the configuration) returns 'not halted' instead of a Request Error "
													   "(USB 2.0 9.4.5)")) {
	startDevice();
	enumerate();
	UsbHost::ControlResult r = getStatus(RCPT_ENDPOINT, 0x03); // endpoint 3 exists in the OTG_FS core
	REQUIRE_USB_MODELLED();
	CHECK(r.stalled()); // Defect: ACK with 0x0000
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "SET_FEATURE(ENDPOINT_HALT) of an endpoint that is not in the configuration is rejected (V23)" * 
								  KNOWN_ISSUE("V23 (TinyUSB, vendor): SET/CLEAR_FEATURE to an endpoint without a class driver "
													   "(usbd.c:922-944: the status stage is only queued inside 'if (driver)') is neither acknowledged nor "
													   "stalled: the host's status stage is NAKed until it times out (USB 2.0 9.4.9: Request Error for an "
													   "endpoint that does not exist, and for endpoints other than 0 in the address state)")) {
	startDevice();
	enumerate();
	UsbHost::ControlResult r = feature(true, RCPT_ENDPOINT, TUSB_REQ_FEATURE_EDPT_HALT, 0x03); // endpoint 3 exists in the OTG_FS core
	REQUIRE_USB_MODELLED();
	CHECK(r.stalled()); // Defect: NAK in the status stage
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "SET_FEATURE(ENDPOINT_HALT) of an endpoint number the controller does not have is rejected (V31)" *
								  KNOWN_ISSUE("V31 (TinyUSB, vendor): usbd.c checks endpoint numbers only against CFG_TUD_ENDPPOINT_MAX, "
											  "which is TUP_DCD_ENDPOINT_MAX 6 for every STM32F4 (tusb_mcu.h:201); the F407 OTG_FS core has 4. "
											  "SET/CLEAR_FEATURE(ENDPOINT_HALT) of 0x84 / 0x85 / 0x04 / 0x05 reaches dcd_edpt_stall / "
											  "dcd_edpt_clear_stall (dwc2 edpt_disable, dfifo_flush_tx) for registers the core does not have, "
											  "and the request is not stalled (USB 2.0 9.4.9: Request Error for an endpoint that does not exist)")) {
	startDevice();
	enumerate();
	UsbHost::ControlResult r = feature(true, RCPT_ENDPOINT, TUSB_REQ_FEATURE_EDPT_HALT, 0x84);
	HostDcd::takeFault("dcd_edpt_stall of endpoint 0x84"); // Part of the defect (not modelled: no such registers), not asserted
	REQUIRE_USB_MODELLED();
	INFO("stage " << r.stage << ", " << r.status);
	CHECK(r.stalled()); // Defect: the stall reaches the controller, then no status stage (NAK until the host times out)
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "CLEAR_FEATURE(ENDPOINT_HALT) of the default control endpoint completes (V23)" * 
								  KNOWN_ISSUE("V23 (TinyUSB, vendor): EP0 has no class driver, so CLEAR_FEATURE(ENDPOINT_HALT, 0) "
													   "gets no status stage (usbd.c:932-943) and the host times out; USB 2.0 9.4.1 / 9.4.9 allow an "
													   "acknowledgement or a Request Error, not a control transfer that never ends")) {
	startDevice();
	enumerate();
	UsbHost::ControlResult r = feature(false, RCPT_ENDPOINT, TUSB_REQ_FEATURE_EDPT_HALT, 0x00);
	REQUIRE_USB_MODELLED();
	INFO("stage " << r.stage << ", " << r.status);
	CHECK((r.ok() || r.stalled())); // Defect: NAK in the status stage
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "characterization: SET_FEATURE with a feature selector that endpoints do not have is acknowledged") {
	// NEEDS REVIEW 155: usbd.c:924-943 ignores feature selectors other than ENDPOINT_HALT and acknowledges the request
	// ("STD request must always be ACKed"); USB 2.0 9.4.9 requires a Request Error for a feature that does not exist.
	startDevice();
	enumerate();
	CHECK(feature(true, RCPT_ENDPOINT, TUSB_REQ_FEATURE_REMOTE_WAKEUP, 0x81).ok());
	CHECK_FALSE(HostDcd::endpoint(0x81).stalled);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "CLEAR_FEATURE(ENDPOINT_HALT) of an endpoint that is not halted keeps its armed transfer busy (V24)" * 
								  KNOWN_ISSUE("V24 (TinyUSB, vendor): usbd_edpt_clear_stall (usbd.c:1398-1409, comment 'only clear "
													   "if currently stalled') clears 'busy' also when the endpoint was not halted and its transfer is still "
													   "armed in the controller; the class driver can then claim the endpoint and arm a second transfer on top "
													   "of the first (e.g. tud_cdc_n_read -> _prep_out_transaction). USB 2.0 9.4.1: hosts may send "
													   "CLEAR_FEATURE(HALT) to an endpoint that is not halted (it only resets the data toggle)")) {
	startVendorDevice();
	enumerate();
	uint8_t buf[64];
	REQUIRE(usbd_edpt_xfer(0, 0x03, buf, 64));
	REQUIRE(feature(false, RCPT_ENDPOINT, TUSB_REQ_FEATURE_EDPT_HALT, 0x03).ok());
	REQUIRE(HostDcd::endpoint(0x03).armed); // the controller still has the transfer (dcd_dwc2.c:665-675)
	REQUIRE_USB_MODELLED();
	CHECK(usbd_edpt_busy(0, 0x03)); // Defect: false
	CHECK_FALSE(usbd_edpt_claim(0, 0x03));
}

// ---------------- Endpoint API for class drivers (usbd_pvt.h) ----------------

TEST_CASE_FIXTURE(UsbdCoreFixture, "an endpoint can be claimed once and released once") {
	startVendorDevice();
	enumerate();
	CHECK(usbd_edpt_claim(0, 0x83));
	CHECK_FALSE(usbd_edpt_claim(0, 0x83));
	CHECK(usbd_edpt_release(0, 0x83));
	CHECK_FALSE(usbd_edpt_release(0, 0x83));
	CHECK(usbd_edpt_claim(0, 0x83));
	CHECK(usbd_edpt_release(0, 0x83));
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a busy endpoint can neither be claimed nor released until its transfer completes") {
	startVendorDevice();
	enumerate();
	REQUIRE(usbd_edpt_claim(0, 0x83));
	uint8_t data[3] = {1, 2, 3};
	REQUIRE(usbd_edpt_xfer(0, 0x83, data, 3));
	CHECK(usbd_edpt_busy(0, 0x83));
	CHECK_FALSE(usbd_edpt_release(0, 0x83));
	CHECK_FALSE(usbd_edpt_claim(0, 0x83));
	CHECK(inPacket(3) == std::vector<uint8_t>{1, 2, 3});
	CHECK_FALSE(usbd_edpt_busy(0, 0x83)); // completion clears busy and claimed (usbd.c:637-638)
	CHECK(usbd_edpt_claim(0, 0x83));
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "an OUT transfer completes with the received bytes and is reported to the class driver") {
	startVendorDevice();
	enumerate();
	REQUIRE(appDriver.epOut == 0x03);
	REQUIRE(appDriver.epIn == 0x83);
	uint8_t buf[64] = {};
	REQUIRE(usbd_edpt_xfer(0, 0x03, buf, sizeof(buf)));
	CHECK(host.out(3, {10, 20, 30}) == Handshake::Ack); // short packet ends the transfer
	process();
	REQUIRE(appDriver.xfers.size() == 1);
	CHECK(appDriver.xfers[0].ep == 0x03);
	CHECK(appDriver.xfers[0].result == XFER_RESULT_SUCCESS);
	CHECK(appDriver.xfers[0].bytes == 3);
	CHECK(buf[2] == 30);
	CHECK_FALSE(usbd_edpt_busy(0, 0x03));
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a second transfer on a busy endpoint is refused") {
	startVendorDevice();
	enumerate();
	uint8_t a[1] = {1}, b[1] = {2};
	REQUIRE(usbd_edpt_xfer(0, 0x83, a, 1));
	CHECK_FALSE(usbd_edpt_xfer(0, 0x83, b, 1)); // usbd.c:1326
	CHECK(HostDcd::endpoint(0x83).transfers == 1);
	CHECK(inPacket(3) == std::vector<uint8_t>{1});
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "usbd_edpt_stall halts an endpoint until usbd_edpt_clear_stall") {
	startVendorDevice();
	enumerate();
	usbd_edpt_stall(0, 0x83);
	CHECK(usbd_edpt_stalled(0, 0x83));
	CHECK(usbd_edpt_busy(0, 0x83)); // a halted endpoint is not ready for transfers
	CHECK_FALSE(usbd_edpt_ready(0, 0x83));
	inPacket(3, Handshake::Stall);
	CHECK(getStatus(RCPT_ENDPOINT, 0x83).data == status1);
	usbd_edpt_clear_stall(0, 0x83);
	CHECK_FALSE(usbd_edpt_stalled(0, 0x83));
	CHECK(usbd_edpt_ready(0, 0x83));
	inPacket(3, Handshake::Nak);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "usbd_edpt_open checks the endpoint number and the full-speed packet size limits") {
	startVendorDevice();
	enumerate();
	auto open = [](uint8_t addr, uint8_t type, uint16_t size) {
		const tusb_desc_endpoint_t d = epDesc(addr, type, size, 1);
		return usbd_edpt_open(0, &d);
	};
	CHECK_FALSE(open(0x86, TUSB_XFER_INTERRUPT, 8)); // number >= CFG_TUD_ENDPPOINT_MAX (usbd.c:1280)
	CHECK_FALSE(open(0x02, TUSB_XFER_BULK, 128));	  // full-speed bulk at most 64 (tusb.c:189-196)
	CHECK_FALSE(open(0x82, TUSB_XFER_INTERRUPT, 65));
	CHECK_FALSE(open(0x82, TUSB_XFER_ISOCHRONOUS, 1024)); // full-speed isochronous at most 1023
	CHECK_FALSE(open(0x82, TUSB_XFER_CONTROL, 64));		  // not a class endpoint type
	CHECK_FALSE(HostDcd::endpoint(0x82).enabled);
	CHECK(open(0x82, TUSB_XFER_INTERRUPT, 64));
	CHECK(HostDcd::endpoint(0x82).enabled);
	CHECK(HostDcd::endpoint(0x82).maxPacket == 64);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "usbd_edpt_close leaves the endpoint open on the DWC2 controller") {
	// usbd.c:1425-1427: with TUP_DCD_EDPT_ISO_ALLOC (DWC2, tusb_mcu.h:546-548) closing is left to the ISO API
	startVendorDevice();
	enumerate();
	usbd_edpt_close(0, 0x83);
	CHECK(HostDcd::endpoint(0x83).enabled);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a completed transfer on an endpoint without class driver is dropped") {
	startVendorDevice();
	enumerate();
	const tusb_desc_endpoint_t d = epDesc(0x02, TUSB_XFER_BULK, 64, 0);
	REQUIRE(usbd_edpt_open(0, &d)); // opened directly, not bound to a driver
	uint8_t buf[64];
	REQUIRE(usbd_edpt_xfer(0, 0x02, buf, 64));
	CHECK(host.out(2, {1}) == Handshake::Ack);
	process();
	CHECK(appDriver.xfers.empty()); // usbd.c:644-645
	CHECK_FALSE(usbd_edpt_busy(0, 0x02));
	CHECK(host.getConfiguration().ok());
}

namespace {
int deferredCalls = 0;
void *deferredParam = nullptr;
void deferred(void *param) {
	deferredCalls++;
	deferredParam = param;
}
} // namespace

TEST_CASE_FIXTURE(UsbdCoreFixture, "usbd_defer_func runs the function in the USB task") {
	startDevice();
	deferredCalls = 0;
	int marker = 0;
	usbd_defer_func(deferred, &marker, false);
	CHECK(deferredCalls == 0);
	CHECK(tud_task_event_ready());
	process();
	CHECK(deferredCalls == 1);
	CHECK(deferredParam == &marker);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "usbd_int_set(false) holds the controller events until usbd_int_set(true)") {
	startDevice();
	enumerate();
	usbd_int_set(false);
	CHECK_FALSE(HostDcd::state().interruptsEnabled);
	HostDcd::suspend();
	process();
	CHECK_FALSE(tud_suspended());
	usbd_int_set(true);
	CHECK(HostDcd::state().interruptsEnabled);
	process();
	CHECK(tud_suspended());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "an event of unknown type from the controller is ignored") {
	startDevice();
	enumerate();
	dcd_event_t e;
	std::memset(&e, 0, sizeof(e));
	e.event_id = DCD_EVENT_COUNT; // usbd.c:686-688 (TU_BREAKPOINT is empty on the host)
	dcd_event_handler(&e, false);
	process();
	CHECK(tud_mounted());
	CHECK(host.getConfiguration().ok());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "of two SETUPs received before the USB task runs only the last one is processed") {
	startDevice();
	enumerate();
	// USB 2.0 8.5.3: a new SETUP aborts the control transfer in progress (usbd.c:602-608)
	REQUIRE(HostDcd::setup(5, UsbHost::request(0x00, TUSB_REQ_SET_FEATURE, TUSB_REQ_FEATURE_REMOTE_WAKEUP, 0, 0)) == Handshake::Ack);
	REQUIRE(HostDcd::setup(5, UsbHost::request(0x80, TUSB_REQ_GET_STATUS, 0, 0, 2)) == Handshake::Ack);
	process();
	std::vector<uint8_t> data;
	REQUIRE(HostDcd::in(5, 0, data) == Handshake::Ack);
	CHECK(data == status1); // remote wakeup was not enabled: the first request was skipped
	process();				// data stage complete: the status stage is armed
	CHECK(HostDcd::out(5, 0, {}) == Handshake::Ack);
	process();
}

// ---------------- tusb.c: stack init, interrupt handler, delay ----------------

TEST_CASE_FIXTURE(UsbdCoreFixture, "tusb_init with an explicit device role initialises the device stack on rhport 0") {
	mainclass = nullptr;
	const tusb_rhport_init_t init = {TUSB_ROLE_DEVICE, TUSB_SPEED_FULL};
	CHECK_FALSE(tusb_inited());
	CHECK(tusb_init(0, &init));
	CHECK(tusb_inited());
	CHECK(tud_inited());
	CHECK(HostDcd::state().initCalls == 1);
	CHECK(HostDcd::state().pullup);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "tusb_init is refused for an invalid role or controller") {
	const tusb_rhport_init_t invalidRole = {TUSB_ROLE_INVALID, TUSB_SPEED_FULL};
	const tusb_rhport_init_t device = {TUSB_ROLE_DEVICE, TUSB_SPEED_FULL};
	CHECK_FALSE(tusb_init(0, &invalidRole));
	CHECK_FALSE(tusb_init(2, &device)); // TUP_USBIP_CONTROLLER_NUM 2
	CHECK_FALSE(tusb_inited());
	CHECK(HostDcd::state().initCalls == 0);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "initialising an initialised stack again changes nothing") {
	startDevice();
	enumerate();
	CHECK(tusb_init());
	CHECK(HostDcd::state().initCalls == 1);
	CHECK(tud_mounted());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "tud_rhport_init without init parameters is refused") {
	CHECK_FALSE(tud_rhport_init(0, nullptr)); // usbd.c:455
	CHECK_FALSE(tud_inited());
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "without an initialised stack tud_task does nothing and tud_deinit succeeds") {
	CHECK_FALSE(tud_inited());
	CHECK_FALSE(tud_task_event_ready());
	tud_task_ext(0, false);
	CHECK(tud_deinit(0));
	CHECK(HostDcd::state().initCalls == 0);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "tud_deinit disconnects the device and deletes the stack's queue and mutex") {
	startDevice();
	enumerate();
	const HostRtos::ObjectCounts before = HostRtos::objectCounts();
	CHECK(tud_deinit(0));
	CHECK_FALSE(tud_inited());
	CHECK_FALSE(HostDcd::state().pullup);
	CHECK_FALSE(HostDcd::state().interruptsEnabled);
	const HostRtos::ObjectCounts after = HostRtos::objectCounts();
	CHECK(after.queues + 1 == before.queues);
	CHECK(after.semaphores < before.semaphores); // the endpoint mutex and the class drivers' FIFO mutexes
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "tusb_int_handler forwards the interrupt of the device controller to the controller driver") {
	startDevice();
	tusb_int_handler(1, true); // not initialised: no role
	tusb_int_handler(5, true); // no such controller (tusb.c:123)
	CHECK(HostDcd::state().faults.empty());
	tusb_int_handler(0, true);
	const std::vector<std::string> faults = HostDcd::takeFaults(); // the fake has no interrupt source to handle
	REQUIRE(faults.size() == 1);
	CHECK(faults[0].find("dcd_int_handler") != std::string::npos);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "tusb_time_delay_ms_api waits with the RTOS delay") {
	const uint64_t start = HostPlatform::nowUs();
	tusb_time_delay_ms_api(5); // osal_task_delay -> vTaskDelay (tusb.c:48-50)
	CHECK(HostPlatform::nowUs() - start == 5000);
}

// ---------------- tusb.c: descriptor helpers ----------------

TEST_CASE("tu_desc_find, tu_desc_find2 and tu_desc_find3 find descriptors in a configuration descriptor") {
	const uint8_t *conf = usb_cdc_hid_conf_2axis;
	const uint8_t *end = conf + (conf[2] | conf[3] << 8);
	const uint8_t *ep = tu_desc_find(conf, end, TUSB_DESC_ENDPOINT);
	REQUIRE(ep != nullptr);
	CHECK(ep[2] == 0x82); // the first endpoint: CDC notification
	const uint8_t *hidIn = tu_desc_find2(conf, end, TUSB_DESC_ENDPOINT, 0x83);
	REQUIRE(hidIn != nullptr);
	CHECK(hidIn[3] == TUSB_XFER_INTERRUPT);
	const uint8_t *hidItf = tu_desc_find3(conf, end, TUSB_DESC_INTERFACE, 2, 0);
	REQUIRE(hidItf != nullptr);
	CHECK(hidItf[5] == TUSB_CLASS_HID);
	CHECK(tu_desc_find(conf, end, TUSB_DESC_STRING) == nullptr);
	CHECK(tu_desc_find2(conf, end, TUSB_DESC_ENDPOINT, 0x84) == nullptr);
	CHECK(tu_desc_find3(conf, end, TUSB_DESC_INTERFACE, 3, 0) == nullptr);
}

TEST_CASE("tu_desc_get_interface_total_len sums an interface with its class and endpoint descriptors") {
	const uint8_t *conf = usb_cdc_hid_conf_2axis;
	const uint16_t total = (uint16_t)(conf[2] | conf[3] << 8);
	const uint8_t *end = conf + total;
	const auto *hidItf = (const tusb_desc_interface_t *)tu_desc_find3(conf, end, TUSB_DESC_INTERFACE, 2, 0);
	REQUIRE(hidItf != nullptr);
	CHECK(tu_desc_get_interface_total_len(hidItf, 1, (uint16_t)(end - (const uint8_t *)hidItf)) == 9 + 9 + 7 + 7);
	const auto *cdcItf = (const tusb_desc_interface_t *)tu_desc_find3(conf, end, TUSB_DESC_INTERFACE, 0, 0);
	REQUIRE(cdcItf != nullptr);
	// The CDC function without its IAD (8 bytes): control interface, functional descriptors, notification
	// endpoint, data interface and its 2 endpoints; it stops at the next IAD-less interface after the 2 interfaces
	CHECK(tu_desc_get_interface_total_len(cdcItf, 2, (uint16_t)(end - (const uint8_t *)cdcItf)) == TUD_CDC_DESC_LEN - 8);
}

TEST_CASE("tu_desc_get_interface_total_len stops at the next interface association") {
	const uint8_t conf[] = {TUD_VENDOR_DESCRIPTOR(0, 0, 0x03, 0x83, 64), TUD_CDC_DESCRIPTOR(1, 4, 0x82, 8, 0x01, 0x81, 64)};
	CHECK(tu_desc_get_interface_total_len((const tusb_desc_interface_t *)conf, 2, sizeof(conf)) == TUD_VENDOR_DESC_LEN);
}

TEST_CASE("tu_edpt_validate applies the packet size limits of full and high speed") {
	auto valid = [](uint8_t type, uint16_t size, tusb_speed_t speed) {
		const tusb_desc_endpoint_t d = epDesc(0x81, type, size);
		return tu_edpt_validate(&d, speed);
	};
	CHECK(valid(TUSB_XFER_ISOCHRONOUS, 1023, TUSB_SPEED_FULL)); // USB 2.0 5.6.3
	CHECK_FALSE(valid(TUSB_XFER_ISOCHRONOUS, 1024, TUSB_SPEED_FULL));
	CHECK(valid(TUSB_XFER_ISOCHRONOUS, 1024, TUSB_SPEED_HIGH));
	CHECK(valid(TUSB_XFER_BULK, 64, TUSB_SPEED_FULL)); // 5.8.3
	CHECK_FALSE(valid(TUSB_XFER_BULK, 65, TUSB_SPEED_FULL));
	CHECK(valid(TUSB_XFER_BULK, 512, TUSB_SPEED_HIGH));
	CHECK_FALSE(valid(TUSB_XFER_BULK, 64, TUSB_SPEED_HIGH));
	CHECK(valid(TUSB_XFER_INTERRUPT, 64, TUSB_SPEED_FULL)); // 5.7.3
	CHECK_FALSE(valid(TUSB_XFER_INTERRUPT, 65, TUSB_SPEED_FULL));
	CHECK(valid(TUSB_XFER_INTERRUPT, 1024, TUSB_SPEED_HIGH));
	CHECK_FALSE(valid(TUSB_XFER_CONTROL, 64, TUSB_SPEED_FULL));
}

// ---------------- tusb.c: endpoint streams (tu_edpt_stream_*) ----------------

namespace {
struct Streams {
	uint8_t txFifo[128], rxFifo[128], txEp[64], rxEp[64];
	tu_edpt_stream_t tx, rx;
	Streams(bool withFifo) {
		std::memset(&tx, 0, sizeof(tx));
		std::memset(&rx, 0, sizeof(rx));
		REQUIRE(tu_edpt_stream_init(&tx, false, true, false, withFifo ? txFifo : nullptr, withFifo ? sizeof(txFifo) : 0, txEp, sizeof(txEp)));
		REQUIRE(tu_edpt_stream_init(&rx, false, false, false, withFifo ? rxFifo : nullptr, withFifo ? sizeof(rxFifo) : 0, rxEp, sizeof(rxEp)));
		const tusb_desc_endpoint_t in = epDesc(0x83, TUSB_XFER_BULK, 64, 0);
		const tusb_desc_endpoint_t out = epDesc(0x03, TUSB_XFER_BULK, 64, 0);
		tu_edpt_stream_open(&tx, &in);
		tu_edpt_stream_open(&rx, &out);
	}
	~Streams() {
		tu_edpt_stream_deinit(&tx);
		tu_edpt_stream_deinit(&rx);
	}
};
} // namespace

TEST_CASE_FIXTURE(UsbdCoreFixture, "a buffered stream write keeps less than a packet in the FIFO until it is flushed") {
	startVendorDevice();
	enumerate();
	Streams s(true);
	const std::string text = "hello";
	CHECK(tu_edpt_stream_write(0, &s.tx, text.data(), (uint32_t)text.size()) == 5);
	CHECK_FALSE(usbd_edpt_busy(0, 0x83));
	CHECK(tu_edpt_stream_write_available(0, &s.tx) == 128 - 5);
	CHECK(tu_edpt_stream_write_xfer(0, &s.tx) == 5);
	CHECK(inPacket(3) == std::vector<uint8_t>(text.begin(), text.end()));
	CHECK(tu_edpt_stream_write_xfer(0, &s.tx) == 0); // nothing left
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a buffered stream write of a full packet is sent at once and followed by a ZLP if needed") {
	startVendorDevice();
	enumerate();
	Streams s(true);
	std::vector<uint8_t> data(64, 0xAB);
	CHECK(tu_edpt_stream_write(0, &s.tx, data.data(), 64) == 64);
	CHECK(usbd_edpt_busy(0, 0x83));
	CHECK(tu_edpt_stream_write_xfer(0, &s.tx) == 0); // FIFO empty
	CHECK(inPacket(3) == data);
	CHECK_FALSE(tu_edpt_stream_write_zlp_if_needed(0, &s.tx, 10)); // not a multiple of the packet size
	CHECK(tu_edpt_stream_write_zlp_if_needed(0, &s.tx, 64));
	CHECK(inPacket(3).empty()); // the ZLP ends the transfer (USB 2.0 5.8.3)
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a stream write waits while the endpoint is busy") {
	startVendorDevice();
	enumerate();
	Streams s(true);
	uint8_t a[1] = {1};
	REQUIRE(usbd_edpt_xfer(0, 0x83, a, 1)); // endpoint busy with another transfer
	CHECK(tu_edpt_stream_write(0, &s.tx, a, 1) == 1); // buffered
	CHECK(tu_edpt_stream_write_xfer(0, &s.tx) == 0);  // claim fails
	CHECK(inPacket(3) == std::vector<uint8_t>{1});
	CHECK(tu_edpt_stream_write_xfer(0, &s.tx) == 1);
	CHECK(inPacket(3) == std::vector<uint8_t>{1});
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "an unbuffered stream write sends at most one endpoint buffer directly") {
	startVendorDevice();
	enumerate();
	Streams s(false);
	CHECK(tu_edpt_stream_write_available(0, &s.tx) == 64);
	std::vector<uint8_t> data(100, 7);
	CHECK(tu_edpt_stream_write(0, &s.tx, data.data(), 100) == 64);
	CHECK(tu_edpt_stream_write_available(0, &s.tx) == 0); // busy
	CHECK(tu_edpt_stream_write(0, &s.tx, data.data(), 1) == 0);
	CHECK(tu_edpt_stream_write(0, &s.tx, data.data(), 0) == 0); // zero length is not supported (tusb.c:367)
	CHECK(inPacket(3).size() == 64);
	CHECK(tu_edpt_stream_write_available(0, &s.tx) == 64);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a buffered stream read arms a multiple of the packet size and returns the received bytes") {
	startVendorDevice();
	enumerate();
	Streams s(true);
	CHECK(tu_edpt_stream_read_xfer(0, &s.rx) == 64); // FIFO space 128, limited by the 64-byte endpoint buffer
	CHECK(usbd_edpt_busy(0, 0x03));
	CHECK(host.out(3, {'a', 'b', 'c'}) == Handshake::Ack);
	process();
	REQUIRE(appDriver.xfers.size() == 1);
	tu_edpt_stream_read_xfer_complete(&s.rx, appDriver.xfers[0].bytes); // what the class driver does in xfer_cb
	CHECK(tu_edpt_stream_read_available(&s.rx) == 3);
	char buf[8] = {};
	CHECK(tu_edpt_stream_read(0, &s.rx, buf, sizeof(buf)) == 3); // and arms the next transfer
	CHECK(std::string(buf) == "abc");
	CHECK(usbd_edpt_busy(0, 0x03));
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a buffered stream read arms only whole packets of the free FIFO space") {
	startVendorDevice();
	enumerate();
	uint8_t fifo[128], epBuf[128];
	tu_edpt_stream_t s;
	std::memset(&s, 0, sizeof(s));
	REQUIRE(tu_edpt_stream_init(&s, false, false, false, fifo, sizeof(fifo), epBuf, sizeof(epBuf)));
	const tusb_desc_endpoint_t out = epDesc(0x03, TUSB_XFER_BULK, 64);
	tu_edpt_stream_open(&s, &out);
	std::vector<uint8_t> fill(28, 1);
	REQUIRE(tu_fifo_write_n(&s.ff, fill.data(), 28) == 28); // 100 bytes of space
	CHECK(tu_edpt_stream_read_xfer(0, &s) == 64);			 // one packet, not 100 (tusb.c:448-450)
	CHECK(HostDcd::endpoint(0x03).length == 64);
	CHECK(tu_edpt_stream_deinit(&s));
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a buffered stream read is not armed while the FIFO has less than a packet of space") {
	startVendorDevice();
	enumerate();
	Streams s(true);
	std::vector<uint8_t> fill(80, 1);
	REQUIRE(tu_fifo_write_n(&s.rx.ff, fill.data(), 80) == 80); // 48 bytes of space left
	CHECK(tu_edpt_stream_read_xfer(0, &s.rx) == 0);
	CHECK_FALSE(usbd_edpt_busy(0, 0x03));
	CHECK(host.out(3, {1}) == Handshake::Nak);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "an unbuffered stream read arms the whole endpoint buffer once") {
	startVendorDevice();
	enumerate();
	Streams s(false);
	CHECK(tu_edpt_stream_read_xfer(0, &s.rx) == 64);
	CHECK(tu_edpt_stream_read_xfer(0, &s.rx) == 0); // already armed: claim fails
	CHECK(HostDcd::endpoint(0x03).transfers == 1);
	CHECK(host.out(3, {9, 8}) == Handshake::Ack);
	process();
	CHECK(s.rxEp[1] == 8);
}

TEST_CASE_FIXTURE(UsbdCoreFixture, "a host stream transfers nothing in the device-only stack") {
	startVendorDevice();
	enumerate();
	uint8_t ep[64];
	tu_edpt_stream_t h;
	std::memset(&h, 0, sizeof(h));
	REQUIRE(tu_edpt_stream_init(&h, true, true, false, nullptr, 0, ep, sizeof(ep))); // is_host: CFG_TUH_ENABLED is 0
	const tusb_desc_endpoint_t in = epDesc(0x83, TUSB_XFER_BULK, 64);
	tu_edpt_stream_open(&h, &in);
	CHECK(tu_edpt_stream_write(0, &h, ep, 4) == 0);
	CHECK(tu_edpt_stream_read_xfer(0, &h) == 0);
	CHECK(tu_edpt_stream_write_available(0, &h) == 0); // reported busy
	CHECK_FALSE(usbd_edpt_busy(0, 0x83));
	tu_edpt_stream_close(&h);
	CHECK(tu_edpt_stream_deinit(&h));
}

} // TEST_SUITE
