/*
 * CdcDevice: TinyUSB's CDC ACM class driver (FFBoard/USB/class/cdc/cdc_device.c, vendor tier B: tested, never fixed)
 * on the fake device controller, together with the firmware's CDC glue: the TinyUSB callbacks tud_cdc_rx_cb /
 * tud_cdc_tx_complete_cb of global_callbacks.cpp, FFBoardMain::cdcRcvReady and CDCcomm (send, busy retry, semaphore).
 *
 * The F407 configuration (tusb_config.h): one CDC interface, receive FIFO 512 bytes, transmit FIFO 1024 bytes,
 * endpoint buffers and bulk packets of 64 bytes (full speed). Control interface 0, data endpoints OUT 0x01 / IN 0x81.
 *
 * The firmware defines only tud_cdc_rx_cb and tud_cdc_tx_complete_cb. The other CDC callbacks of cdc_device.h
 * (line state, line coding, send break, wanted char) are weak and NULL on the target; this file defines recording
 * versions for the whole ffb_usb_tests executable (they only record) to test that TinyUSB calls them.
 * Host-vs-target: on the target these four callbacks are not linked, so cdc_device.c skips them (the `if (cb)` checks).
 *
 * Tests about TinyUSB's receive FIFO stop the firmware from reading it: CdcFixture starts the device with CdcTestMain,
 * a UsbTestMain whose cdcRcvReady (called by tud_cdc_rx_cb) can be told not to read. `mainclass` stays set, so the
 * mount / unmount / suspend callbacks (which dereference it without a check) stay safe.
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "UsbFixture.h"

#include "CDCcomm.h"

#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace {

/** What the CDC callbacks defined below received */
struct CdcCallbackLog {
	uint32_t lineStateCalls = 0;
	bool dtr = false, rts = false;
	uint32_t lineCodingCalls = 0;
	cdc_line_coding_t lineCoding{};
	uint32_t sendBreakCalls = 0;
	uint16_t breakDuration = 0;
	std::vector<char> wanted; //!< tud_cdc_rx_wanted_cb calls
	size_t availableAtWanted = 0; //!< tud_cdc_n_available at the last wanted char callback
};
CdcCallbackLog cdcLog;

} // namespace

// Recording CDC callbacks (weak and NULL in the firmware, see the file comment)
extern "C" {
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
	(void)itf;
	cdcLog.lineStateCalls++;
	cdcLog.dtr = dtr;
	cdcLog.rts = rts;
}
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *coding) {
	(void)itf;
	cdcLog.lineCodingCalls++;
	cdcLog.lineCoding = *coding;
}
void tud_cdc_send_break_cb(uint8_t itf, uint16_t duration_ms) {
	(void)itf;
	cdcLog.sendBreakCalls++;
	cdcLog.breakDuration = duration_ms;
}
void tud_cdc_rx_wanted_cb(uint8_t itf, char wanted_char) {
	cdcLog.wanted.push_back(wanted_char);
	cdcLog.availableAtWanted = tud_cdc_n_available(itf);
}
}

namespace {

using Handshake = HostDcd::Handshake;

constexpr uint8_t EP_OUT = 0x01; // usb_descriptors.h USB_CONF_DESC_HID_CDC: CDC data OUT
constexpr uint8_t EP_IN = 0x81;	 // CDC data IN
constexpr size_t PACKET = 64;
constexpr size_t RX_FIFO = CFG_TUD_CDC_RX_BUFSIZE; // 512
constexpr size_t TX_FIFO = CFG_TUD_CDC_TX_BUFSIZE; // 1024

std::string pattern(size_t n, size_t offset = 0) {
	std::string s(n, ' ');
	for (size_t i = 0; i < n; i++) {
		s[i] = (char)('A' + (i + offset) % 26);
	}
	return s;
}

/** UsbTestMain whose receive callback can stop reading the CDC port (the data then stays in TinyUSB's receive FIFO) */
class CdcTestMain : public UsbTestMain {
public:
	bool reading = true;
	void cdcRcvReady(uint8_t itf) override {
		if (reading) {
			UsbTestMain::cdcRcvReady(itf);
		} else {
			cdcReadyCalls++;
		}
	}
};

struct CdcFixture : UsbFixture {
	CdcFixture() { cdcLog = CdcCallbackLog(); }

	/**
	 * UsbFixture::startDevice with CdcTestMain as the main class (same steps: resumable CDCCMD / CMD_MAIN, mainclass,
	 * usb_device with the F407 wheel configuration, HID descriptor, tusb_init)
	 */
	void startDevice() {
		auto m = std::make_unique<CdcTestMain>();
		cdcMain = m.get();
		main = std::move(m);
		REQUIRE(HostRtos::makeResumable("CDCCMD") != nullptr);
		REQUIRE(HostRtos::makeResumable("CMD_MAIN") != nullptr);
		mainclass = main.get();
		usb_device = usbDevice(usb_cdc_hid_conf_2axis);
		UsbHidHandler::setHidDesc(hid_2ffb_desc);
		REQUIRE(tusb_init());
		process();
	}
	CdcTestMain *cdcMain = nullptr;

	/** Starts the F407 wheel device, enumerates it and (if dtr) opens the port like a terminal (DTR and RTS set) */
	void start(bool dtr = true) {
		startDevice();
		enumerate();
		if (dtr) {
			REQUIRE(host.cdcSetControlLineState(true, true).ok());
		}
	}
	/** The firmware no longer reads the CDC port: received data stays in TinyUSB's receive FIFO (see file comment) */
	void stopFirmwareReading() { cdcMain->reading = false; }

	/** One IN transaction on the CDC data IN endpoint without NAK retries */
	Handshake inPacket(std::vector<uint8_t> &data) { return HostDcd::in(host.address, EP_IN & 0x0F, data); }
	/** One OUT transaction on the CDC data OUT endpoint without NAK retries */
	Handshake outPacket(const std::vector<uint8_t> &data) { return HostDcd::out(host.address, EP_OUT, data); }

	/** Sizes of the packets the host received (ACK) from the CDC data IN endpoint */
	static std::vector<size_t> inPacketSizes() {
		std::vector<size_t> sizes;
		for (const HostDcd::Packet &p : HostDcd::state().packets) {
			if (p.pid == HostDcd::Packet::Pid::In && p.ep == EP_IN && p.handshake == Handshake::Ack) {
				sizes.push_back(p.data.size());
			}
		}
		return sizes;
	}

	/** Reads everything the application can read from the receive FIFO */
	static std::string readAll() {
		std::string s;
		char buf[600];
		uint32_t n;
		while ((n = tud_cdc_n_read(0, buf, sizeof(buf))) > 0) {
			s.append(buf, n);
		}
		return s;
	}
};

} // namespace

TEST_SUITE("CdcDevice") {

// ---------------------------------------------------------------- line coding

TEST_CASE_FIXTURE(CdcFixture, "the line coding is 115200 baud with 8 data bits, no parity and 1 stop bit before the host sets one") {
	start(false);
	UsbHost::ControlResult r = host.cdcGetLineCoding();
	REQUIRE(r.ok());
	// CDC PSTN 1.2 table 17: dwDTERate, bCharFormat (0 = 1 stop bit), bParityType (0 = none), bDataBits
	CHECK(r.data == std::vector<uint8_t>{0x00, 0xC2, 0x01, 0x00, 0, 0, 8});
}

TEST_CASE_FIXTURE(CdcFixture, "a line coding set by the host is passed to the line coding callback after the status stage") {
	start(false);
	REQUIRE(host.cdcSetLineCoding(9600, 2, 1, 7).ok());
	CHECK(cdcLog.lineCodingCalls == 1);
	CHECK(uint32_t(cdcLog.lineCoding.bit_rate) == 9600);
	CHECK(cdcLog.lineCoding.stop_bits == 2);
	CHECK(cdcLog.lineCoding.parity == 1);
	CHECK(cdcLog.lineCoding.data_bits == 7);
	cdc_line_coding_t coding;
	tud_cdc_n_get_line_coding(0, &coding);
	CHECK(uint32_t(coding.bit_rate) == 9600);
}

TEST_CASE_FIXTURE(CdcFixture, "the line coding callback is not called for a GET_LINE_CODING request") {
	start(false);
	REQUIRE(host.cdcGetLineCoding().ok());
	CHECK(cdcLog.lineCodingCalls == 0);
}

TEST_CASE_FIXTURE(CdcFixture, "the line coding set by the host is kept over a bus reset and a new enumeration") {
	// cdc_device.c:57: the members from wanted_char on (line coding, FIFOs) are not cleared by a bus reset
	start(false);
	REQUIRE(host.cdcSetLineCoding(250000, 0, 0, 8).ok());
	enumerate();
	UsbHost::ControlResult r = host.cdcGetLineCoding();
	REQUIRE(r.ok());
	CHECK(r.data == std::vector<uint8_t>{0x90, 0xD0, 0x03, 0x00, 0, 0, 8});
}

// ---------------------------------------------------------------- control line state

TEST_CASE_FIXTURE(CdcFixture, "DTR set by the host makes the port connected and reaches the line state callback with DTR and RTS") {
	start(false);
	CHECK_FALSE(tud_cdc_n_connected(0));
	CHECK_FALSE(CDCcomm::connected(0));
	REQUIRE(host.cdcSetControlLineState(true, true).ok());
	CHECK(tud_cdc_n_connected(0));
	CHECK(CDCcomm::connected(0));
	CHECK(tud_cdc_n_get_line_state(0) == 0x03);
	CHECK(cdcLog.lineStateCalls == 1);
	CHECK(cdcLog.dtr);
	CHECK(cdcLog.rts);
}

TEST_CASE_FIXTURE(CdcFixture, "RTS without DTR does not make the port connected") {
	start(false);
	REQUIRE(host.cdcSetControlLineState(false, true).ok());
	CHECK(tud_cdc_n_get_line_state(0) == 0x02);
	CHECK_FALSE(tud_cdc_n_connected(0));
	CHECK(cdcLog.lineStateCalls == 1);
	CHECK_FALSE(cdcLog.dtr);
	CHECK(cdcLog.rts);
}

TEST_CASE_FIXTURE(CdcFixture, "clearing DTR disconnects the port") {
	start();
	REQUIRE(host.cdcSetControlLineState(false, false).ok());
	CHECK_FALSE(tud_cdc_n_connected(0));
	CHECK(tud_cdc_n_get_line_state(0) == 0);
	CHECK(cdcLog.lineStateCalls == 2);
}

TEST_CASE_FIXTURE(CdcFixture, "a bus reset clears DTR and RTS") {
	start();
	REQUIRE(tud_cdc_n_connected(0));
	host.busReset();
	CHECK(tud_cdc_n_get_line_state(0) == 0);
	CHECK_FALSE(tud_cdc_n_connected(0));
}

TEST_CASE_FIXTURE(CdcFixture, "unplugging the cable clears DTR and the port is no longer ready") {
	start();
	HostDcd::detach();
	process();
	CHECK_FALSE(tud_cdc_n_connected(0));
	CHECK_FALSE(tud_cdc_n_ready(0));
	CHECK(tud_cdc_n_get_line_state(0) == 0);
}

TEST_CASE_FIXTURE(CdcFixture, "the port is ready only after the host has configured the device") {
	startDevice();
	CHECK_FALSE(tud_cdc_n_ready(0));
	host.busReset();
	CHECK_FALSE(tud_cdc_n_ready(0));
	enumerate();
	CHECK(tud_cdc_n_ready(0));
	CHECK_FALSE(tud_cdc_n_connected(0)); // Ready (configured) but no terminal yet
}

// ---------------------------------------------------------------- other class requests

TEST_CASE_FIXTURE(CdcFixture, "SEND_BREAK is acknowledged and its duration reaches the send break callback") {
	start(false);
	// CDC PSTN 1.2 6.3.13: wValue = duration of the break in ms
	REQUIRE(host.control(UsbHost::request(0x21, CDC_REQUEST_SEND_BREAK, 250, host.cdcControlInterface(), 0)).ok());
	CHECK(cdcLog.sendBreakCalls == 1);
	CHECK(cdcLog.breakDuration == 250);
}

TEST_CASE_FIXTURE(CdcFixture, "an unsupported CDC class request is stalled") {
	start(false);
	// SET_COMM_FEATURE (PSTN 6.3.1) is optional for ACM and not implemented by TinyUSB
	UsbHost::ControlResult r = host.control(UsbHost::request(0x21, CDC_REQUEST_SET_COMM_FEATURE, 1, host.cdcControlInterface(), 2), {0, 0});
	CHECK(r.stalled());
	CHECK(host.cdcGetLineCoding().ok()); // The next request works again
}

TEST_CASE_FIXTURE(CdcFixture, "a CDC class request addressed to the data interface is stalled") {
	start(false);
	// Interface 1 (CDC data) belongs to the CDC driver, but the requests of PSTN 6.3 go to the control interface
	const uint16_t dataItf = host.findInterface(TUSB_CLASS_CDC_DATA)->number;
	UsbHost::ControlResult r = host.control(UsbHost::request(0xA1, CDC_REQUEST_GET_LINE_CODING, 0, dataItf, 7));
	CHECK(r.stalled());
}

// ---------------------------------------------------------------- receive FIFO

TEST_CASE_FIXTURE(CdcFixture, "data from the host is stored in the receive FIFO until the application reads it") {
	start();
	stopFirmwareReading();
	REQUIRE(host.cdcWrite("hello") == Handshake::Ack);
	process();
	CHECK(main->cdcReadyCalls == 1);
	CHECK(tud_cdc_n_available(0) == 5);
	CHECK(readAll() == "hello");
	CHECK(tud_cdc_n_available(0) == 0);
}

TEST_CASE_FIXTURE(CdcFixture, "peek returns the first received byte without removing it") {
	start();
	stopFirmwareReading();
	REQUIRE(host.cdcWrite("hello") == Handshake::Ack);
	process();
	uint8_t first = 0;
	CHECK(tud_cdc_n_peek(0, &first));
	CHECK(first == 'h');
	CHECK(tud_cdc_n_available(0) == 5);
}

TEST_CASE_FIXTURE(CdcFixture, "peek fails while the receive FIFO is empty") {
	start();
	uint8_t first = 0;
	CHECK_FALSE(tud_cdc_n_peek(0, &first));
}

TEST_CASE_FIXTURE(CdcFixture, "a read smaller than the received data leaves the rest in the receive FIFO") {
	start();
	stopFirmwareReading();
	REQUIRE(host.cdcWrite("hello") == Handshake::Ack);
	process();
	char buf[3];
	CHECK(tud_cdc_n_read(0, buf, sizeof(buf)) == 3);
	CHECK(std::string(buf, 3) == "hel");
	CHECK(tud_cdc_n_available(0) == 2);
	CHECK(readAll() == "lo");
}

TEST_CASE_FIXTURE(CdcFixture, "the receive FIFO accepts packets while 64 bytes are free and then NAKs the host until the application reads") {
	start();
	stopFirmwareReading();
	const std::string data = pattern(RX_FIFO + PACKET);
	for (size_t i = 0; i < RX_FIFO / PACKET; i++) {
		INFO("packet " << i);
		REQUIRE(host.out(EP_OUT, std::vector<uint8_t>(data.begin() + (long)(i * PACKET), data.begin() + (long)((i + 1) * PACKET))) == Handshake::Ack);
	}
	process();
	CHECK(tud_cdc_n_available(0) == RX_FIFO);
	const std::vector<uint8_t> last(data.end() - (long)PACKET, data.end());
	CHECK(outPacket(last) == Handshake::Nak); // Flow control: no transfer armed, no data lost
	char buf[PACKET];
	REQUIRE(tud_cdc_n_read(0, buf, sizeof(buf)) == PACKET); // 64 bytes free again: the endpoint is armed
	CHECK(outPacket(last) == Handshake::Ack);
	process();
	CHECK(std::string(buf, PACKET) + readAll() == data);
}

TEST_CASE_FIXTURE(CdcFixture, "the endpoint stays NAKing while less than 64 bytes of the receive FIFO are free") {
	start();
	stopFirmwareReading();
	for (size_t i = 0; i < RX_FIFO / PACKET; i++) {
		REQUIRE(host.out(EP_OUT, std::vector<uint8_t>(PACKET, (uint8_t)i)) == Handshake::Ack);
	}
	process();
	char buf[PACKET - 1];
	REQUIRE(tud_cdc_n_read(0, buf, sizeof(buf)) == PACKET - 1); // 63 bytes free: not enough for a full packet
	CHECK(outPacket({1, 2, 3}) == Handshake::Nak);
}

TEST_CASE_FIXTURE(CdcFixture, "read flush discards the received data and arms the endpoint again") {
	start();
	stopFirmwareReading();
	for (size_t i = 0; i < RX_FIFO / PACKET; i++) {
		REQUIRE(host.out(EP_OUT, std::vector<uint8_t>(PACKET, 'x')) == Handshake::Ack);
	}
	process();
	REQUIRE(outPacket({'y'}) == Handshake::Nak);
	tud_cdc_n_read_flush(0);
	CHECK(tud_cdc_n_available(0) == 0);
	CHECK(outPacket({'y'}) == Handshake::Ack);
	process();
	CHECK(readAll() == "y");
}

TEST_CASE_FIXTURE(CdcFixture, "a bus reset discards received data that the application has not read") {
	start();
	stopFirmwareReading();
	REQUIRE(host.cdcWrite("old") == Handshake::Ack);
	process();
	REQUIRE(tud_cdc_n_available(0) == 3);
	host.busReset();
	CHECK(tud_cdc_n_available(0) == 0);
}

TEST_CASE_FIXTURE(CdcFixture, "a receive FIFO configured as persistent keeps unread data over a bus reset") {
	startDevice(); // cdcd_init (tusb_init) clears the FIFO configuration: configure afterwards
	tud_cdc_configure_fifo_t cfg{};
	cfg.rx_persistent = 1;
	REQUIRE(tud_cdc_configure_fifo(&cfg));
	enumerate();
	REQUIRE(host.cdcSetControlLineState(true, true).ok());
	stopFirmwareReading();
	REQUIRE(host.cdcWrite("kept") == Handshake::Ack);
	process();
	host.busReset();
	CHECK(readAll() == "kept");
}

TEST_CASE_FIXTURE(CdcFixture, "a FIFO configuration without a configuration struct is rejected") {
	CHECK_FALSE(tud_cdc_configure_fifo(nullptr));
}

TEST_CASE_FIXTURE(CdcFixture, "the wanted char callback is called once per wanted char in a received packet") {
	start();
	stopFirmwareReading();
	tud_cdc_n_set_wanted_char(0, '\n');
	REQUIRE(host.cdcWrite("ab\ncd\n") == Handshake::Ack);
	process();
	CHECK(cdcLog.wanted == std::vector<char>{'\n', '\n'});
	CHECK(cdcLog.availableAtWanted == 6); // The packet is in the FIFO before the callback
}

TEST_CASE_FIXTURE(CdcFixture, "the wanted char callback is not called while no wanted char is set") {
	start();
	stopFirmwareReading();
	REQUIRE(host.cdcWrite("ab\n") == Handshake::Ack);
	process();
	CHECK(cdcLog.wanted.empty());
	CHECK(tud_cdc_n_available(0) == 3);
}

// ---------------------------------------------------------------- transmit FIFO

TEST_CASE_FIXTURE(CdcFixture, "less than a packet written by the application is sent only after a flush") {
	start();
	CHECK(tud_cdc_n_write(0, "hello", 5) == 5);
	process();
	std::vector<uint8_t> data;
	CHECK(inPacket(data) == Handshake::Nak);
	CHECK(tud_cdc_n_write_flush(0) == 5);
	REQUIRE(inPacket(data) == Handshake::Ack);
	CHECK(std::string(data.begin(), data.end()) == "hello");
}

TEST_CASE_FIXTURE(CdcFixture, "a write that fills a packet starts the transfer without a flush") {
	start();
	const std::string text = pattern(PACKET + 6);
	CHECK(tud_cdc_n_write(0, text.data(), (uint32_t)text.size()) == text.size());
	CHECK(host.cdcRead() == text); // 64 bytes, then the rest is flushed when the first transfer completes
	CHECK(inPacketSizes() == std::vector<size_t>{64, 6});
}

TEST_CASE_FIXTURE(CdcFixture, "a write of exactly one packet starts the transfer without a flush") {
	start();
	const std::string text = pattern(PACKET);
	REQUIRE(tud_cdc_n_write(0, text.data(), (uint32_t)text.size()) == PACKET);
	std::vector<uint8_t> data;
	REQUIRE(inPacket(data) == Handshake::Ack);
	CHECK(std::string(data.begin(), data.end()) == text);
}

TEST_CASE_FIXTURE(CdcFixture, "a transfer that is a multiple of 64 bytes is ended by a zero length packet") {
	start();
	const std::string text = pattern(2 * PACKET);
	REQUIRE(tud_cdc_n_write(0, text.data(), (uint32_t)text.size()) == text.size());
	CHECK(host.cdcRead() == text);
	// USB 2.0 5.8.3: a bulk transfer ends with a short packet; a full last packet needs a ZLP
	CHECK(inPacketSizes() == std::vector<size_t>{64, 64, 0});
}

TEST_CASE_FIXTURE(CdcFixture, "a transfer shorter than 64 bytes is not followed by a zero length packet") {
	start();
	REQUIRE(tud_cdc_n_write(0, "abc", 3) == 3);
	REQUIRE(tud_cdc_n_write_flush(0) == 3);
	CHECK(host.cdcRead() == "abc");
	CHECK(inPacketSizes() == std::vector<size_t>{3});
}

TEST_CASE_FIXTURE(CdcFixture, "a flush while a transfer is running sends nothing and the data follows after the running transfer") {
	start();
	REQUIRE(tud_cdc_n_write(0, "first", 5) == 5);
	REQUIRE(tud_cdc_n_write_flush(0) == 5);
	REQUIRE(tud_cdc_n_write(0, "second", 6) == 6);
	CHECK(tud_cdc_n_write_flush(0) == 0); // The IN endpoint is busy
	CHECK(host.cdcRead() == "firstsecond");
	CHECK(inPacketSizes() == std::vector<size_t>{5, 6});
}

TEST_CASE_FIXTURE(CdcFixture, "a flush with an empty transmit FIFO sends nothing") {
	start();
	CHECK(tud_cdc_n_write_flush(0) == 0);
	std::vector<uint8_t> data;
	CHECK(inPacket(data) == Handshake::Nak);
}

TEST_CASE_FIXTURE(CdcFixture, "write available is the free space of the transmit FIFO") {
	start();
	CHECK(tud_cdc_n_write_available(0) == TX_FIFO);
	REQUIRE(tud_cdc_n_write(0, "0123456789", 10) == 10);
	CHECK(tud_cdc_n_write_available(0) == TX_FIFO - 10);
}

TEST_CASE_FIXTURE(CdcFixture, "write clear discards the data that was not sent yet") {
	start();
	REQUIRE(tud_cdc_n_write(0, "drop", 4) == 4);
	CHECK(tud_cdc_n_write_clear(0));
	CHECK(tud_cdc_n_write_available(0) == TX_FIFO);
	CHECK(tud_cdc_n_write_flush(0) == 0);
}

TEST_CASE_FIXTURE(CdcFixture, "with DTR set a full transmit FIFO takes no more data and nothing written before is lost") {
	start(true);
	const std::string text = pattern(2000);
	// 1024 bytes fit; the first 64 go into the endpoint buffer at once (write flushes a full packet)
	REQUIRE(tud_cdc_n_write(0, text.data(), (uint32_t)text.size()) == TX_FIFO);
	CHECK(tud_cdc_n_write_available(0) == PACKET);
	CHECK(tud_cdc_n_write(0, text.data() + TX_FIFO, (uint32_t)(text.size() - TX_FIFO)) == PACKET);
	CHECK(tud_cdc_n_write_available(0) == 0);
	CHECK(tud_cdc_n_write(0, "x", 1) == 0);
	CHECK(host.cdcRead() == text.substr(0, TX_FIFO + PACKET));
}

TEST_CASE_FIXTURE(CdcFixture, "without DTR a full transmit FIFO keeps the newest data") {
	// cdc_device.c:244-247: overwritable until a terminal sets DTR, so the most recent data is sent
	start(false);
	const std::string text = pattern(3000);
	tud_cdc_n_write(0, text.data(), (uint32_t)text.size());
	const std::string received = host.cdcRead();
	CHECK(received == text.substr(text.size() - TX_FIFO));
}

TEST_CASE_FIXTURE(CdcFixture, "data written before the device is configured is not sent and is discarded by the bus reset") {
	startDevice();
	CHECK(tud_cdc_n_write(0, "early", 5) == 5);
	CHECK(tud_cdc_n_write_flush(0) == 0); // Not ready
	enumerate();
	CHECK(tud_cdc_n_write_available(0) == TX_FIFO);
	CHECK(tud_cdc_n_write_flush(0) == 0);
}

TEST_CASE_FIXTURE(CdcFixture, "a transmit FIFO configured as persistent sends data written before a bus reset afterwards") {
	startDevice(); // cdcd_init (tusb_init) clears the FIFO configuration: configure afterwards
	tud_cdc_configure_fifo_t cfg{};
	cfg.tx_persistent = 1;
	REQUIRE(tud_cdc_configure_fifo(&cfg));
	enumerate();
	REQUIRE(host.cdcSetControlLineState(true, true).ok());
	REQUIRE(tud_cdc_n_write(0, "kept", 4) == 4);
	enumerate();
	REQUIRE(host.cdcSetControlLineState(true, true).ok());
	CHECK(tud_cdc_n_write_flush(0) == 4);
	CHECK(host.cdcRead() == "kept");
}

TEST_CASE_FIXTURE(CdcFixture, "a bus reset makes the transmit FIFO overwritable again until the next DTR") {
	start(true);
	enumerate(); // DTR cleared, FIFO overwritable
	const std::string text = pattern(1500);
	tud_cdc_n_write(0, text.data(), (uint32_t)text.size());
	CHECK(host.cdcRead() == text.substr(text.size() - TX_FIFO));
}

// ---------------------------------------------------------------- firmware receive path

TEST_CASE_FIXTURE(CdcFixture, "every received packet reaches the main class through the receive callback") {
	start();
	const uint32_t before = main->cdcReadyCalls;
	REQUIRE(host.cdcWrite("sys.main?\n") == Handshake::Ack);
	process();
	CHECK(main->cdcReadyCalls == before + 1);
	CHECK(tud_cdc_n_available(0) == 0); // FFBoardMain::cdcRcvReady read everything
	CHECK(host.cdcRead() == "[sys.main?|0]\n");
}

TEST_CASE_FIXTURE(CdcFixture, "a command line split over two packets is executed once and answered") {
	start();
	REQUIRE(host.out(EP_OUT, {'s', 'y', 's', '.', 'm'}) == Handshake::Ack);
	process();
	REQUIRE(host.out(EP_OUT, {'a', 'i', 'n', '?', '\n'}) == Handshake::Ack);
	process();
	CHECK(host.cdcRead() == "[sys.main?|0]\n");
}

TEST_CASE_FIXTURE(CdcFixture, "several commands longer than one packet are all answered") {
	start();
	std::string line;
	std::string expected;
	for (int i = 0; i < 10; i++) {
		line += "sys.main?;";
		expected += "[sys.main?|0]\n";
	}
	line += "\n";
	REQUIRE(line.size() > PACKET);
	REQUIRE(host.cdcWrite(line) == Handshake::Ack);
	process();
	CHECK(host.cdcRead() == expected);
}

// ---------------------------------------------------------------- CDCcomm (firmware send path)

TEST_CASE_FIXTURE(CdcFixture, "CDCcomm sends a reply that fits into the transmit FIFO at once") {
	start();
	std::string reply = "reply\n";
	CHECK(CDCcomm::cdcSend(&reply, 0) == reply.size());
	CHECK(CDCcomm::remainingData(0) == 0);
	CHECK(host.cdcRead() == "reply\n");
}

TEST_CASE_FIXTURE(CdcFixture, "CDCcomm sends nothing while the device is not configured") {
	startDevice();
	std::string reply = "lost";
	CHECK(CDCcomm::cdcSend(&reply, 0) == 0);
	enumerate();
	CHECK(tud_cdc_n_write_available(0) == TX_FIFO);
	// The semaphore was not taken: the next reply is sent at once (a taken one would block the test thread)
	std::string next = "next";
	CHECK(CDCcomm::cdcSend(&next, 0) == 4);
	CHECK(host.cdcRead() == "next");
}

TEST_CASE_FIXTURE(CdcFixture, "CDCcomm sends nothing for an empty reply") {
	start();
	std::string empty;
	CHECK(CDCcomm::cdcSend(&empty, 0) == 0);
	std::string next = "x";
	CHECK(CDCcomm::cdcSend(&next, 0) == 1); // The semaphore is still free
	CHECK(host.cdcRead() == "x");
}

TEST_CASE_FIXTURE(CdcFixture, "CDCcomm can send the next reply after the host has read the previous one") {
	start();
	std::string first = "first\n";
	REQUIRE(CDCcomm::cdcSend(&first, 0) == first.size());
	CHECK(host.cdcRead() == "first\n"); // Transfer complete: tud_cdc_tx_complete_cb -> CDCcomm::cdcFinished gives the semaphore
	std::string second = "second\n";
	CHECK(CDCcomm::cdcSend(&second, 0) == second.size());
	CHECK(host.cdcRead() == "second\n");
}

TEST_CASE_FIXTURE(CdcFixture, "CDCcomm sends a reply longer than the free transmit FIFO completely and in order") {
	start();
	std::string reply = pattern(3000);
	const uint16_t sent = CDCcomm::cdcSend(&reply, 0);
	CHECK(sent == TX_FIFO);
	CHECK(CDCcomm::remainingData(0) == reply.size() - TX_FIFO);
	CHECK(host.cdcRead() == reply); // The rest follows from the transfer complete callbacks (busy retry)
	CHECK(CDCcomm::remainingData(0) == 0);
	std::string next = "next";
	CHECK(CDCcomm::cdcSend(&next, 0) == 4); // The retries left the semaphore free
	CHECK(host.cdcRead() == "next");
}

TEST_CASE_FIXTURE(CdcFixture, "CDCcomm sends a long reply completely also while no terminal has set DTR") {
	// Without DTR the transmit FIFO is overwritable (cdc_device.c:247): cdcSend writes at most the free space
	// (CDCcomm.cpp:68-69), so nothing of the reply is overwritten
	start(false);
	std::string reply = pattern(2500);
	REQUIRE(CDCcomm::cdcSend(&reply, 0) == TX_FIFO);
	CHECK(host.cdcRead() == reply);
}

TEST_CASE_FIXTURE(CdcFixture, "a command reply waits while a long reply is still pending and is sent after it") {
	start();
	// The command thread polls CDC_CommandInterface::readyToSend (no remaining CDCcomm data) with Delay(1), at most
	// 100 times (FFBoardMainCommandThread.cpp:130-133). The fake advances the time in vTaskDelay; with a delay budget of
	// 1 ms the thread parks after each poll and continues at the next runUntilBlocked, so the host can read in between
	// like on the target (where the higher priority USB thread runs while the command thread waits).
	HostRtos::setDelayBudgetMs(1);
	std::string longReply = pattern(2000);
	REQUIRE(CDCcomm::cdcSend(&longReply, 0) == TX_FIFO);
	REQUIRE(host.cdcWrite("sys.main?\n") == Handshake::Ack);
	process();
	TaskHandle_t commandThread = HostRtos::findTask("CMD_MAIN");
	REQUIRE(commandThread != nullptr);
	HostRtos::runUntilBlocked(commandThread); // One more poll: still not ready
	CHECK(host.cdcRead() == longReply);		  // The reply is held back while the long reply is sent
	CHECK(HAL_GetTick() < 100);
	HostRtos::runUntilBlocked(commandThread); // Next poll: ready, the replies go to the CDC thread
	process();
	CHECK(host.cdcRead() == "[sys.main?|0]\n");
}

TEST_CASE_FIXTURE(CdcFixture, "a cleared remaining buffer is not sent by the transfer complete callback") {
	start();
	std::string reply = pattern(1500);
	REQUIRE(CDCcomm::cdcSend(&reply, 0) == TX_FIFO);
	REQUIRE(CDCcomm::remainingData(0) == 1500 - TX_FIFO);
	CDCcomm::clearRemainingBuffer(0);
	CHECK(CDCcomm::remainingData(0) == 0);
	CHECK(host.cdcRead() == reply.substr(0, TX_FIFO));
	// Leave CDCcomm idle for the next test: clearRemainingBuffer does not reset the private busy retry flag
	// (CDCcomm.cpp:51-53), which only a complete send clears; such a send is not flushed while the flag is set (:70)
	std::string idle = "z";
	REQUIRE(CDCcomm::cdcSend(&idle, 0) == 1);
	tud_cdc_n_write_flush(0);
	CHECK(host.cdcRead() == "z");
}

TEST_CASE_FIXTURE(CdcFixture, "the CDC reply path works again after the host reset the device while a reply was pending (bug 137)" *
								  KNOWN_ISSUE("bug 137: CDCcomm::cdcSend (CDCcomm.cpp:67) takes cdcSems[itf] until tud_cdc_tx_complete_cb; "
											  "a bus reset or unplug drops the running IN transfer without a completion, nothing gives the "
											  "semaphore back (global_callbacks.cpp:365-380 do not reset CDCcomm), so the next cdcSend of "
											  "the CDC thread blocks forever: no CDC reply until the board is power cycled")) {
	start();
	std::string pending = pattern(100);
	REQUIRE(CDCcomm::cdcSend(&pending, 0) == 100); // 64 bytes in the endpoint, 36 in the FIFO, semaphore taken
	// The host does not read and resets the device (e.g. the PC restarts while the externally powered board runs)
	enumerate();
	REQUIRE(host.cdcSetControlLineState(true, true).ok());
	REQUIRE(host.cdcWrite("sys.main?\n") == Handshake::Ack);
	process();
	HostPlatform::advanceMs(1000); // Lets a timeout based fix (e.g. a finite Take) expire
	process();
	REQUIRE_USB_MODELLED();
	CHECK(host.cdcRead() == "[sys.main?|0]\n"); // Defect: empty, the CDC thread waits for the semaphore forever
}

TEST_CASE_FIXTURE(CdcFixture, "the CDC reply path works again after the host reset the device while a reply longer than the FIFO was pending (bug 137)" *
								  KNOWN_ISSUE("bug 137: as above; in addition the unsent rest of the long reply stays in "
											  "CDCcomm::remainingStrs with usb_busy_retry set (CDCcomm.cpp:74-77), so even with the semaphore "
											  "given back the next cdcSend does not flush (CDCcomm.cpp:70) and its reply is not sent")) {
	start();
	std::string pending = pattern(3000);
	REQUIRE(CDCcomm::cdcSend(&pending, 0) == TX_FIFO); // 1976 bytes remain for the retry, semaphore taken
	enumerate();
	REQUIRE(host.cdcSetControlLineState(true, true).ok());
	REQUIRE(host.cdcWrite("sys.main?\n") == Handshake::Ack);
	process();
	HostPlatform::advanceMs(1000);
	process();
	REQUIRE_USB_MODELLED();
	const std::string received = host.cdcRead();
	const std::string reply = "[sys.main?|0]\n";
	// Whether the rest of the old reply is dropped or sent first is not specified: the new reply must arrive at the end
	CHECK((received.size() >= reply.size() && received.compare(received.size() - reply.size(), reply.size(), reply) == 0)); // Defect: empty
	// Cleanup for the next test (until UsbFixture resets it): the defect leaves CDCcomm's private busy retry flag set,
	// which only a complete cdcSend clears; give the semaphore back and send one byte
	CDCcomm::clearRemainingBuffer(0);
	CDCcomm::cdcFinished(0);
	std::string idle = "z";
	CDCcomm::cdcSend(&idle, 0);
	tud_cdc_n_write_flush(0);
	host.cdcRead();
}

TEST_CASE_FIXTURE(CdcFixture, "the CDC reply path works again after the cable was unplugged while a reply was pending (bug 137)" *
								  KNOWN_ISSUE("bug 137: as above, after an unplug (DCD_EVENT_UNPLUGGED, tud_umount_cb) instead of a bus "
											  "reset: the dropped IN transfer never completes and cdcSems[0] stays taken")) {
	start();
	std::string pending = pattern(100);
	REQUIRE(CDCcomm::cdcSend(&pending, 0) == 100);
	HostDcd::detach();
	process();
	HostDcd::attach();
	enumerate();
	REQUIRE(host.cdcSetControlLineState(true, true).ok());
	REQUIRE(host.cdcWrite("sys.main?\n") == Handshake::Ack);
	process();
	HostPlatform::advanceMs(1000);
	process();
	REQUIRE_USB_MODELLED();
	CHECK(host.cdcRead() == "[sys.main?|0]\n"); // Defect: empty
}

} // TEST_SUITE
