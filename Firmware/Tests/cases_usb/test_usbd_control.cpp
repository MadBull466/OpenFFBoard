/*
 * TinyUSB control transfers on EP0 (FFBoard/USB/device/usbd_control.c, vendor tier B: tested, never fixed): data
 * stages of control reads and writes in packets of 64 bytes (CFG_TUD_ENDPOINT0_SIZE), short packets and zero-length
 * packets, the status stage and the stage callbacks of the class drivers (SETUP, DATA, ACK). Reference: USB 2.0 5.5.3
 * (data stage ends with a short packet or wLength bytes), 8.5.3 (control transfers, status stage), 9.3 / 9.4.3.
 *
 * Standard requests use the firmware's descriptors; data stages of arbitrary length use the test class driver of
 * test_usbd_core.cpp (a vendor interface 0, see there) whose control callback is set per test.
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "UsbFixture.h"

#include "device/usbd_pvt.h"

#include <cstring>
#include <vector>

// test_usbd_core.cpp
namespace a_usbcore {
using ControlHook = bool (*)(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request);
void setAppDriver(bool enabled, ControlHook control);
const uint8_t *vendorConfiguration();
} // namespace a_usbcore

namespace {

using Handshake = HostDcd::Handshake;
using Packet = HostDcd::Packet;

constexpr uint8_t REQ_READ = 0x01;	//!< Class request of the test: control read of `readLength` bytes of `readData`
constexpr uint8_t REQ_WRITE = 0x02; //!< Class request of the test: control write into `writeBuffer`

/** State of the control callback of the test driver (reset by the fixture) */
struct ControlState {
	std::vector<uint8_t> readData; //!< Data offered for REQ_READ
	uint16_t readLength = 0;	   //!< Length passed to tud_control_xfer for REQ_READ
	bool readNullBuffer = false;   //!< Pass NULL as buffer
	uint8_t writeBuffer[256] = {};
	bool dataStageResult = true; //!< Return value of the DATA stage callback (REQ_READ and REQ_WRITE)
	std::vector<uint8_t> stages; //!< Stages of every callback, in order
	uint16_t lastWLength = 0;
};
ControlState ctl;

bool controlHook(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
	ctl.stages.push_back(stage);
	ctl.lastWLength = request->wLength;
	if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS) {
		return false;
	}
	switch (request->bRequest) {
	case REQ_READ:
		if (stage == CONTROL_STAGE_SETUP) {
			return tud_control_xfer(rhport, request, ctl.readNullBuffer ? nullptr : ctl.readData.data(), ctl.readLength);
		}
		if (stage == CONTROL_STAGE_DATA) {
			return ctl.dataStageResult;
		}
		return true;
	case REQ_WRITE:
		if (stage == CONTROL_STAGE_SETUP) {
			return tud_control_xfer(rhport, request, ctl.writeBuffer, request->wLength);
		}
		if (stage == CONTROL_STAGE_DATA) {
			return ctl.dataStageResult;
		}
		return true;
	default:
		return false;
	}
}

struct UsbdControlFixture : UsbFixture {
	UsbdControlFixture() { ctl = ControlState(); }
	~UsbdControlFixture() override { a_usbcore::setAppDriver(false, nullptr); }

	void startVendorDevice() {
		a_usbcore::setAppDriver(true, controlHook);
		startDevice(a_usbcore::vendorConfiguration());
		enumerate();
		HostDcd::takeFaults(); // none expected; keeps the packet log comparison below simple
		ctl.stages.clear();
		firstPacket = HostDcd::state().packets.size();
	}
	/** Sizes of the IN data packets of EP0 since `firstPacket` (each ACKed IN packet on EP0, status ZLPs included) */
	std::vector<size_t> ep0InPacketSizes() const {
		std::vector<size_t> sizes;
		const auto &p = HostDcd::state().packets;
		for (size_t i = firstPacket; i < p.size(); i++) {
			if (p[i].pid == Packet::Pid::In && p[i].ep == 0x80 && p[i].handshake == Handshake::Ack) {
				sizes.push_back(p[i].data.size());
			}
		}
		return sizes;
	}
	UsbHost::ControlResult read(uint16_t wLength) { return host.control(UsbHost::request(0xA1, REQ_READ, 0, 0, wLength)); }
	std::vector<uint8_t> pattern(size_t n) {
		std::vector<uint8_t> v(n);
		for (size_t i = 0; i < n; i++) {
			v[i] = (uint8_t)(i * 7 + 1);
		}
		return v;
	}
	size_t firstPacket = 0;
};

} // namespace

TEST_SUITE("UsbdControl") {

// ---------------- Standard control reads: wLength shorter / longer than the descriptor ----------------

TEST_CASE_FIXTURE(UsbdControlFixture, "GET_DESCRIPTOR with a wLength shorter than the descriptor returns wLength bytes") {
	startDevice();
	enumerate();
	UsbHost::ControlResult r = host.getDescriptor(TUSB_DESC_DEVICE, 0, 8);
	REQUIRE(r.ok());
	CHECK(r.data.size() == 8);
	CHECK(r.data == std::vector<uint8_t>(host.deviceDescriptor.begin(), host.deviceDescriptor.begin() + 8));
	r = host.getDescriptor(TUSB_DESC_CONFIGURATION, 0, 100); // 64 + 36 of 107 bytes
	REQUIRE(r.ok());
	CHECK(r.data == std::vector<uint8_t>(host.configDescriptor.begin(), host.configDescriptor.begin() + 100));
}

TEST_CASE_FIXTURE(UsbdControlFixture, "GET_DESCRIPTOR with a wLength longer than the descriptor ends with a short packet") {
	startDevice();
	enumerate();
	firstPacket = HostDcd::state().packets.size();
	UsbHost::ControlResult r = host.getDescriptor(TUSB_DESC_DEVICE, 0, 255);
	REQUIRE(r.ok());
	CHECK(r.data.size() == 18);
	r = host.getDescriptor(TUSB_DESC_CONFIGURATION, 0, 255);
	REQUIRE(r.ok());
	CHECK(r.data.size() == TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_HID_INOUT_DESC_LEN);
	CHECK(ep0InPacketSizes() == std::vector<size_t>{18, 64, 43});
}

TEST_CASE_FIXTURE(UsbdControlFixture, "a control read of exactly wLength bytes in full packets has no zero-length packet") {
	startDevice();
	enumerate();
	firstPacket = HostDcd::state().packets.size();
	UsbHost::ControlResult r = host.getDescriptor(TUSB_DESC_CONFIGURATION, 0, 64);
	REQUIRE(r.ok());
	CHECK(r.data.size() == 64);
	CHECK(ep0InPacketSizes() == std::vector<size_t>{64});
}

TEST_CASE_FIXTURE(UsbdControlFixture, "GET_DESCRIPTOR of an unknown descriptor type or string index is rejected") {
	startDevice();
	enumerate();
	CHECK(host.getDescriptor(TUSB_DESC_STRING, 7, 255).stalled());			// interface strings are 4..6
	CHECK(host.getDescriptor(TUSB_DESC_DEVICE_QUALIFIER, 0, 10).stalled()); // full-speed only device (USB 2.0 9.6.2)
	CHECK(host.getDescriptor(TUSB_DESC_OTHER_SPEED_CONFIG, 0, 9).stalled());
	CHECK(host.getDescriptor(TUSB_DESC_BOS, 0, 5).stalled()); // bcdUSB 2.00: no BOS
	CHECK(host.getDescriptor(TUSB_DESC_INTERFACE, 0, 9).stalled()); // not directly accessible (USB 2.0 9.4.3)
	CHECK(host.getDescriptor(0x30, 0, 9).stalled());
	CHECK(host.getDescriptor(TUSB_DESC_DEVICE, 0, 18).ok());
}

TEST_CASE_FIXTURE(UsbdControlFixture, "the string descriptors of the interfaces carry the interface names") {
	startDevice();
	enumerate();
	CHECK(host.getString(4).rfind("FFBoard CDC", 0) == 0);
	CHECK(host.getString(5).rfind("FFBoard HID", 0) == 0);
	UsbHost::ControlResult r = host.getDescriptor(TUSB_DESC_STRING, 0, 255);
	REQUIRE(r.ok());
	CHECK(r.data == std::vector<uint8_t>{4, TUSB_DESC_STRING, 0x09, 0x04}); // LANGID US English
}

// ---------------- Control reads of a class driver ----------------

TEST_CASE_FIXTURE(UsbdControlFixture, "a control read of more than one packet is sent in packets of 64 bytes") {
	startVendorDevice();
	ctl.readData = pattern(150);
	ctl.readLength = 150;
	UsbHost::ControlResult r = read(150);
	REQUIRE(r.ok());
	CHECK(r.data == ctl.readData);
	CHECK(ep0InPacketSizes() == std::vector<size_t>{64, 64, 22});
}

TEST_CASE_FIXTURE(UsbdControlFixture, "a control read shorter than wLength that ends on a packet boundary ends with a zero-length packet") {
	startVendorDevice();
	ctl.readData = pattern(128);
	ctl.readLength = 128;
	UsbHost::ControlResult r = read(255);
	REQUIRE(r.ok());
	CHECK(r.data == ctl.readData);
	CHECK(ep0InPacketSizes() == std::vector<size_t>{64, 64, 0}); // USB 2.0 5.5.3
}

TEST_CASE_FIXTURE(UsbdControlFixture, "a control read of exactly wLength bytes on a packet boundary has no zero-length packet") {
	startVendorDevice();
	ctl.readData = pattern(128);
	ctl.readLength = 128;
	UsbHost::ControlResult r = read(128);
	REQUIRE(r.ok());
	CHECK(ep0InPacketSizes() == std::vector<size_t>{64, 64});
}

TEST_CASE_FIXTURE(UsbdControlFixture, "a control read offering more than wLength sends only wLength bytes") {
	startVendorDevice();
	ctl.readData = pattern(200);
	ctl.readLength = 200;
	UsbHost::ControlResult r = read(70);
	REQUIRE(r.ok());
	CHECK(r.data == pattern(70));
	CHECK(ep0InPacketSizes() == std::vector<size_t>{64, 6});
}

TEST_CASE_FIXTURE(UsbdControlFixture, "a control read without data answers the data stage with a zero-length packet") {
	startVendorDevice();
	ctl.readLength = 0;
	UsbHost::ControlResult r = read(8);
	REQUIRE(r.ok());
	CHECK(r.data.empty());
	CHECK(ep0InPacketSizes() == std::vector<size_t>{0});
}

TEST_CASE_FIXTURE(UsbdControlFixture, "a control read with data but no buffer is rejected") {
	startVendorDevice();
	ctl.readNullBuffer = true;
	ctl.readLength = 4;
	CHECK(read(4).stalled()); // usbd_control.c:110-112
}

TEST_CASE_FIXTURE(UsbdControlFixture, "the status stage of a control read is armed only after the last data packet") {
	startVendorDevice();
	ctl.readData = pattern(100);
	ctl.readLength = 100;
	REQUIRE(HostDcd::setup(5, UsbHost::request(0xA1, REQ_READ, 0, 0, 100)) == Handshake::Ack);
	process();
	std::vector<uint8_t> packet;
	REQUIRE(HostDcd::in(5, 0, packet) == Handshake::Ack);
	process();
	CHECK(HostDcd::out(5, 0, {}) == Handshake::Nak); // data stage not complete
	REQUIRE(HostDcd::in(5, 0, packet) == Handshake::Ack);
	CHECK(packet.size() == 36);
	process();
	CHECK(ctl.stages == std::vector<uint8_t>{CONTROL_STAGE_SETUP, CONTROL_STAGE_DATA});
	CHECK(HostDcd::out(5, 0, {}) == Handshake::Ack);
	process();
	CHECK(ctl.stages.back() == CONTROL_STAGE_ACK);
}

// ---------------- Control writes ----------------

TEST_CASE_FIXTURE(UsbdControlFixture, "a control write of more than one packet delivers all bytes to the driver before its status stage") {
	startVendorDevice();
	const std::vector<uint8_t> data = pattern(150);
	UsbHost::ControlResult r = host.control(UsbHost::request(0x21, REQ_WRITE, 0, 0, 150), data);
	REQUIRE(r.ok());
	CHECK(std::vector<uint8_t>(ctl.writeBuffer, ctl.writeBuffer + 150) == data);
	CHECK(ctl.stages == std::vector<uint8_t>{CONTROL_STAGE_SETUP, CONTROL_STAGE_DATA, CONTROL_STAGE_ACK});
}

TEST_CASE_FIXTURE(UsbdControlFixture, "a short packet ends the data stage of a control write early") {
	startVendorDevice();
	REQUIRE(HostDcd::setup(5, UsbHost::request(0x21, REQ_WRITE, 0, 0, 100)) == Handshake::Ack);
	process();
	REQUIRE(HostDcd::out(5, 0, pattern(20)) == Handshake::Ack); // short packet (USB 2.0 5.5.3)
	process();
	CHECK(ctl.stages == std::vector<uint8_t>{CONTROL_STAGE_SETUP, CONTROL_STAGE_DATA});
	std::vector<uint8_t> zlp;
	CHECK(HostDcd::in(5, 0, zlp) == Handshake::Ack);
	CHECK(zlp.empty());
	process();
	CHECK(std::vector<uint8_t>(ctl.writeBuffer, ctl.writeBuffer + 20) == pattern(20));
	CHECK(ctl.stages.back() == CONTROL_STAGE_ACK);
}

TEST_CASE_FIXTURE(UsbdControlFixture, "the status stage of a control write is armed only after the data stage") {
	startVendorDevice();
	REQUIRE(HostDcd::setup(5, UsbHost::request(0x21, REQ_WRITE, 0, 0, 70)) == Handshake::Ack);
	process();
	std::vector<uint8_t> zlp;
	CHECK(HostDcd::in(5, 0, zlp) == Handshake::Nak);
	REQUIRE(HostDcd::out(5, 0, pattern(64)) == Handshake::Ack);
	process();
	CHECK(HostDcd::in(5, 0, zlp) == Handshake::Nak); // 6 bytes still missing
	REQUIRE(HostDcd::out(5, 0, pattern(6)) == Handshake::Ack);
	process();
	CHECK(ctl.stages.back() == CONTROL_STAGE_DATA); // the ACK stage follows the status stage
	CHECK(HostDcd::in(5, 0, zlp) == Handshake::Ack);
	process();
	CHECK(ctl.stages.back() == CONTROL_STAGE_ACK);
}

TEST_CASE_FIXTURE(UsbdControlFixture, "a driver that refuses the received data stalls the status stage") {
	startVendorDevice();
	ctl.dataStageResult = false;
	UsbHost::ControlResult r = host.control(UsbHost::request(0x21, REQ_WRITE, 0, 0, 10), pattern(10));
	CHECK(r.stalled());
	CHECK(r.stage == "status");
	CHECK(ctl.stages == std::vector<uint8_t>{CONTROL_STAGE_SETUP, CONTROL_STAGE_DATA}); // no ACK stage
	CHECK(host.getConfiguration().ok()); // the next SETUP clears the stall
}

TEST_CASE_FIXTURE(UsbdControlFixture, "a driver that refuses the end of the data stage of a control read stalls the status stage") {
	startVendorDevice();
	ctl.readData = pattern(10);
	ctl.readLength = 10;
	ctl.dataStageResult = false;
	UsbHost::ControlResult r = read(10);
	CHECK(r.data == pattern(10)); // the data stage itself is sent
	CHECK(r.stalled());
	CHECK(r.stage == "status"); // usbd_control.c:209-212: both EP0 directions stalled instead of arming the status OUT
	CHECK(HostDcd::endpoint(0x00).stalled);
	CHECK(ctl.stages == std::vector<uint8_t>{CONTROL_STAGE_SETUP, CONTROL_STAGE_DATA}); // no ACK stage
	CHECK(host.getConfiguration().ok()); // the next SETUP clears the stall
}

TEST_CASE_FIXTURE(UsbdControlFixture, "a data packet in the status stage of a control read is answered with a stall") {
	// Fake controller (DCFG.NZLSOHSK, dcd_dwc2.c:444): the core stalls a non-zero-length status OUT and drops the packet
	startVendorDevice();
	ctl.readData = pattern(4);
	ctl.readLength = 4;
	REQUIRE(HostDcd::setup(5, UsbHost::request(0xA1, REQ_READ, 0, 0, 4)) == Handshake::Ack);
	process();
	std::vector<uint8_t> data;
	REQUIRE(HostDcd::in(5, 0, data) == Handshake::Ack);
	REQUIRE(data == pattern(4));
	process();
	REQUIRE(HostDcd::endpoint(0x00).armed); // the status OUT (0 bytes) is armed
	CHECK(HostDcd::out(5, 0, {0x55}) == Handshake::Stall);
	CHECK(HostDcd::endpoint(0x00).armed); // the packet was dropped, the status stage is still open
	CHECK(HostDcd::out(5, 0, {}) == Handshake::Ack);
	process();
	CHECK(ctl.stages.back() == CONTROL_STAGE_ACK);
}

TEST_CASE_FIXTURE(UsbdControlFixture, "a request without data stage gets an IN status stage and the ACK callback") {
	startVendorDevice();
	UsbHost::ControlResult r = host.control(UsbHost::request(0x21, REQ_WRITE, 0, 0, 0));
	REQUIRE(r.ok());
	CHECK(ctl.stages == std::vector<uint8_t>{CONTROL_STAGE_SETUP, CONTROL_STAGE_ACK});
	CHECK(ep0InPacketSizes() == std::vector<size_t>{0});
}

TEST_CASE_FIXTURE(UsbdControlFixture, "a new SETUP in the middle of a control write starts a new control transfer") {
	startVendorDevice();
	REQUIRE(HostDcd::setup(5, UsbHost::request(0x21, REQ_WRITE, 0, 0, 100)) == Handshake::Ack);
	process();
	REQUIRE(HostDcd::out(5, 0, pattern(64)) == Handshake::Ack);
	process();
	ctl.readData = pattern(3);
	ctl.readLength = 3;
	UsbHost::ControlResult r = read(3); // USB 2.0 8.5.3: the SETUP aborts the unfinished transfer
	REQUIRE(r.ok());
	CHECK(r.data == pattern(3));
	CHECK(host.getConfiguration().ok());
}

TEST_CASE_FIXTURE(UsbdControlFixture, "the status stage of SET_ADDRESS is sent by the controller driver and completes the request") {
	startDevice();
	host.busReset();
	REQUIRE(host.setAddress(12).ok()); // usbd.c:749-757: dcd_set_address queues the status ZLP itself
	process();
	CHECK(HostDcd::state().setAddressCalls == std::vector<uint8_t>{12});
	CHECK(HostDcd::state().address == 12);
	CHECK(host.getDescriptor(TUSB_DESC_DEVICE, 0, 18).ok());
}

TEST_CASE_FIXTURE(UsbdControlFixture, "a device-to-host request with wLength 0 completes with an IN status stage (V30)" * 
									 KNOWN_ISSUE("V30 (TinyUSB, vendor): "
														  "_status_stage_xact (usbd_control.c:76-80) takes the status direction from bmRequestType only "
														  "('opposite to the data phase'); a request with the IN direction bit and wLength 0 has no data "
														  "stage, so its status stage is IN (USB 2.0 8.5.3, figure 8-37 'no-data control'), but TinyUSB "
														  "arms an OUT ZLP and the host's status IN is NAKed until it times out")) {
	startDevice();
	enumerate();
	UsbHost::ControlResult r = host.getDescriptor(TUSB_DESC_DEVICE, 0, 0); // also: GET_STATUS / GET_CONFIGURATION with wLength 0
	REQUIRE_USB_MODELLED();
	INFO("stage " << r.stage << ", " << r.status);
	CHECK(r.ok()); // Defect: NAK in the status stage (EP0 OUT armed instead of IN)
}

} // TEST_SUITE
