/*
 * Tests of the CAN bridge main class (UserExtensions/Src/CanBridge.cpp): the GVRET / SavvyCAN binary protocol on the
 * CDC port (0xE7 switches to binary mode, 0xF1 <cmd> <data> commands), received CAN frames forwarded to the host
 * (binary GVRET frames or "CAN:<id>:<data>" log lines in text mode), the text commands of the bridge (main.can /
 * main.rtr / main.spd), the LEDs and the use of the global `canport` (the real CANPort_2B on the fake HAL CAN).
 *
 * GVRET reference (collin80/GVRET, SavvyCAN gvretserial.cpp): frame to the host
 *   F1 00 <timestamp us, 4 bytes LE> <id 4 bytes LE, bit 31 = extended> <len | bus << 4> <data> <0>
 * Commands of the host: 00 build frame, 01 time sync, 02 digital in, 03 analog in, 04 digital out, 05 set CAN config,
 * 06 get CAN config, 07 device info, 08 single wire, 09 keepalive, 0A set system, 0B echo frame, 0C number of buses,
 * 0D get extended buses, 0E set extended buses.
 *
 * GVRET bytes are passed like tud_cdc_rx_cb -> FFBoardMain::cdcRcvReady does: in a 64 byte buffer like
 * FFBoardMain::cdcbuf (the rest zero), see gvret(). Only the bug 119 case uses a buffer of the exact size.
 *
 * Host-vs-target notes: `char` is unsigned like on ARM (-funsigned-char; CanBridge compares char with 0xE7 / 0xF1).
 * The CDC buffer of the F407 (full speed) is 64 bytes, so the uint8_t positions of cdcRcv do not overflow; with a high
 * speed port (512 byte buffer) they would.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "CommandHelpers.h"
#include "TestHelpers.h"

#include "CANPort2B.h"
#include "CanBridge.h"
#include "cpp_target_config.h"

#include <array>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

namespace {

std::string bytes(std::initializer_list<int> list) {
	std::string s;
	for (int b : list) {
		s.push_back(static_cast<char>(b));
	}
	return s;
}

/** Bytes received on CDC, passed in a 64 byte buffer like FFBoardMain::cdcRcvReady (cdcbuf), the rest zero */
void gvret(CanBridge &bridge, const std::string &data) {
	std::array<char, 64> buf{};
	REQUIRE(data.size() <= buf.size());
	std::copy(data.begin(), data.end(), buf.begin());
	uint32_t len = static_cast<uint32_t>(data.size());
	bridge.cdcRcv(buf.data(), &len);
}

std::string &cdcOut() { return HostUsb::state().cdcTx; }

/** SavvyCAN's connect: 0xE7 0xE7 switches the bridge to the binary GVRET protocol */
void enterGvret(CanBridge &bridge) {
	gvret(bridge, bytes({0xE7, 0xE7}));
	cdcOut().clear();
}

/** A GVRET frame to the host (bus 0, timestamp as given) */
std::string gvretFrame(uint32_t time, uint32_t id, const std::vector<uint8_t> &data) {
	std::string s = bytes({0xF1, 0x00});
	for (int i = 0; i < 4; i++) s.push_back(static_cast<char>((time >> (8 * i)) & 0xFF));
	for (int i = 0; i < 4; i++) s.push_back(static_cast<char>((id >> (8 * i)) & 0xFF));
	s.push_back(static_cast<char>(data.size() & 0x0F));
	for (uint8_t b : data) s.push_back(static_cast<char>(b));
	s.push_back(0);
	return s;
}

struct BridgeFixture : TargetFixture {
	CanBridge bridge; // Constructed after the reset of TargetFixture, destroyed before its final reset
};

constexpr uint32_t ESR_BOFF = 0x00000004U; // stm32f407xx.h CAN_ESR_BOFF

} // namespace

TEST_SUITE("CanBridge") {

// ---------------------------------------------------------------- class, port, filter

TEST_CASE_FIXTURE(TargetFixture, "the CAN bridge is the visible GVRET main class with the CAN bridge class id") {
	CanBridge bridge;

	CHECK(bridge.getInfo().id == CLSID_MAIN_CAN);
	CHECK(std::string(bridge.getInfo().name) == "CAN Bridge (GVRET)");
	CHECK(bridge.getInfo().visibility == ClassVisibility::visible);
	CHECK(bridge.getHelpstring().find("GVRET") != std::string::npos);
}

TEST_CASE_FIXTURE(TargetFixture, "the CAN bridge takes the CAN port in addition to the CAN command interface") {
	CanBridge bridge;

	CHECK(canport.getPortUsers() == 2); // CAN_CommandInterface of FFBoardMain (CANCOMMANDS) and the bridge
	CHECK(hcan1.State == HAL_CAN_STATE_LISTENING);
}

TEST_CASE_FIXTURE(TargetFixture, "the CAN bridge switches the transceiver out of silent mode") {
	CanBridge bridge;

	CHECK_FALSE(HostTarget::outputPin(CAN_S_GPIO_Port, CAN_S_Pin));
}

TEST_CASE_FIXTURE(TargetFixture, "the filter of the CAN bridge accepts every standard and extended id into FIFO 0") {
	CanBridge bridge;

	for (uint32_t id : {0x000u, 0x123u, 0x7FFu}) {
		CAPTURE(id);
		uint32_t fifo = 99;
		CHECK(HostTarget::canAccepts(id, false, false, &fifo));
		CHECK(fifo == CAN_RX_FIFO0);
	}
	CHECK(HostTarget::canAccepts(0x1FFFFFFF, true));
	CHECK(HostTarget::canAccepts(0x00012345, true));
}

TEST_CASE_FIXTURE(TargetFixture, "deleting the CAN bridge removes its filter and stops the CAN port") {
	auto bridge = std::make_unique<CanBridge>();
	REQUIRE(HostTarget::canAccepts(0x555));

	bridge.reset();

	CHECK_FALSE(HostTarget::canAccepts(0x555));
	CHECK(canport.getPortUsers() == 0);
	CHECK(hcan1.State == HAL_CAN_STATE_READY);
}

// ---------------------------------------------------------------- GVRET mode and commands

TEST_CASE_FIXTURE(BridgeFixture, "binary GVRET commands are not answered before the host sends 0xE7") {
	gvret(bridge, bytes({0xF1, 0x09}));
	HostRtos::runUntilIdle();

	CHECK(cdcOut().empty());
}

TEST_CASE_FIXTURE(BridgeFixture, "switching to GVRET mode with 0xE7 is not answered") {
	gvret(bridge, bytes({0xE7, 0xE7}));
	HostRtos::runUntilIdle();

	CHECK(cdcOut().empty());
}

TEST_CASE_FIXTURE(BridgeFixture, "a keepalive is answered with F1 09 DE AD") {
	enterGvret(bridge);

	gvret(bridge, bytes({0xF1, 0x09}));

	CHECK(cdcOut() == bytes({0xF1, 0x09, 0xDE, 0xAD}));
}

TEST_CASE_FIXTURE(BridgeFixture, "the time sync reply carries the microseconds since start up") {
	enterGvret(bridge);
	HostPlatform::advanceMs(1234); // 1234000 us = 0x0012D450

	gvret(bridge, bytes({0xF1, 0x01}));

	CHECK(cdcOut() == bytes({0xF1, 0x01, 0x50, 0xD4, 0x12, 0x00}));
}

TEST_CASE_FIXTURE(BridgeFixture, "the device info reply reports build 257, EEPROM version 1 and no file output, auto logging or single wire") {
	enterGvret(bridge);

	gvret(bridge, bytes({0xF1, 0x07}));

	CHECK(cdcOut() == bytes({0xF1, 0x07, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00}));
}

TEST_CASE_FIXTURE(BridgeFixture, "the number of buses reply is 1") {
	enterGvret(bridge);

	gvret(bridge, bytes({0xF1, 0x0C}));

	CHECK(cdcOut() == bytes({0xF1, 0x0C, 0x01}));
}

TEST_CASE_FIXTURE(BridgeFixture, "the CAN config reply reports bus 0 enabled at 500 kbit/s and bus 1 disabled") {
	enterGvret(bridge);

	gvret(bridge, bytes({0xF1, 0x06}));

	// per bus: enabled | listenOnly << 4, speed LE (500000 = 0x0007A120; bus 1 keeps the default speed of CAN_Config_t)
	CHECK(cdcOut() == bytes({0xF1, 0x06, 0x01, 0x20, 0xA1, 0x07, 0x00, 0x00, 0x20, 0xA1, 0x07, 0x00}));
}

TEST_CASE_FIXTURE(BridgeFixture, "setting the CAN config of bus 0 changes the speed of the CAN port") {
	enterGvret(bridge);

	// speed 250000 = 0x0003D090, config byte 0xC0: flags valid, enabled; bus 1: 0
	gvret(bridge, bytes({0xF1, 0x05, 0x90, 0xD0, 0x03, 0xC0, 0x00, 0x00, 0x00, 0x00}));

	CHECK(canport.getSpeed() == 250000);
	CHECK(CAN1->BTR == 0x001a000bU); // preset "250k" of cpp_target_config.cpp
	CHECK(hcan1.State == HAL_CAN_STATE_LISTENING);
	CHECK(cdcOut().empty());
}

TEST_CASE_FIXTURE(BridgeFixture, "a CAN config with a speed that is no preset keeps the speed of the CAN port") {
	enterGvret(bridge);

	gvret(bridge, bytes({0xF1, 0x05, 0xE0, 0x93, 0x04, 0xC0, 0x00, 0x00, 0x00, 0x00})); // 300000

	CHECK(canport.getSpeed() == 500000);
}

TEST_CASE_FIXTURE(BridgeFixture, "the CAN config reply reports the speed set by the host" *
									 KNOWN_ISSUE("bug 120: CanBridge.cpp:218-228 (command 5) passes the speed to the port but "
												 "never stores it in conf1.speed; command 6 (CanBridge.cpp:230-238) reports "
												 "conf1.speed, which stays 500000 (CAN_Config_t default, CanBridge.h). The same "
												 "after main.spd. The CHECK fails")) {
	enterGvret(bridge);
	gvret(bridge, bytes({0xF1, 0x05, 0x90, 0xD0, 0x03, 0xC0, 0x00, 0x00, 0x00, 0x00})); // 250000, enabled
	REQUIRE(canport.getSpeed() == 250000);
	cdcOut().clear();

	gvret(bridge, bytes({0xF1, 0x06}));
	REQUIRE_ALL_HAL_MODELLED();

	REQUIRE(cdcOut().size() == 12);
	CHECK(cdcOut().substr(2, 5) == bytes({0x01, 0x90, 0xD0, 0x03, 0x00}));
}

TEST_CASE_FIXTURE(BridgeFixture, "characterization: the listen-only flag of the CAN config is reported but does not silence the port (NEEDS REVIEW 124)") {
	// CanBridge.cpp:223-226 stores enabled / listenOnly in conf1; nothing applies them (no setSilentMode, no
	// stop/start, frames are still sent). Disabling bus 0 (0x80 without 0x40) is reported and ignored the same way.
	enterGvret(bridge);
	gvret(bridge, bytes({0xF1, 0x05, 0x20, 0xA1, 0x07, 0xE0, 0x00, 0x00, 0x00, 0x00})); // 500000, valid, enabled, listen only

	gvret(bridge, bytes({0xF1, 0x06}));

	REQUIRE(cdcOut().size() == 12);
	CHECK(static_cast<uint8_t>(cdcOut()[2]) == 0x11);
	CHECK_FALSE(HostTarget::outputPin(CAN_S_GPIO_Port, CAN_S_Pin));
	gvret(bridge, bytes({0xF1, 0x00, 0x23, 0x01, 0x00, 0x00, 0x00, 0x01, 0x55}));
	CHECK(HostTarget::canTxFrames().size() == 1);
}

TEST_CASE_FIXTURE(BridgeFixture, "a CAN config without the flag bit keeps the enabled and listen-only flags") {
	enterGvret(bridge);
	gvret(bridge, bytes({0xF1, 0x05, 0x20, 0xA1, 0x07, 0xA0, 0x00, 0x00, 0x00, 0x00})); // valid, disabled, listen only
	gvret(bridge, bytes({0xF1, 0x05, 0x20, 0xA1, 0x07, 0x40, 0x00, 0x00, 0x00, 0x00})); // 0x40 without 0x80: ignored
	cdcOut().clear();

	gvret(bridge, bytes({0xF1, 0x06}));

	REQUIRE(cdcOut().size() == 12);
	CHECK(static_cast<uint8_t>(cdcOut()[2]) == 0x10);
}

TEST_CASE_FIXTURE(BridgeFixture, "a build-frame command sends a standard frame on bus 0") {
	enterGvret(bridge);

	gvret(bridge, bytes({0xF1, 0x00, 0x23, 0x01, 0x00, 0x00, 0x00, 0x03, 0xAA, 0xBB, 0xCC}));

	REQUIRE(HostTarget::canTxFrames().size() == 1);
	const HostTarget::CanTxFrame &f = HostTarget::canTxFrames()[0];
	CHECK(f.id == 0x123);
	CHECK_FALSE(f.extId);
	CHECK_FALSE(f.rtr);
	CHECK(f.dlc == 3);
	CHECK(f.payload() == std::vector<uint8_t>{0xAA, 0xBB, 0xCC});
	CHECK(cdcOut().empty());
}

TEST_CASE_FIXTURE(BridgeFixture, "a build-frame command with bit 31 of the id sends an extended frame") {
	enterGvret(bridge);

	gvret(bridge, bytes({0xF1, 0x00, 0x45, 0x23, 0x01, 0x80, 0x00, 0x02, 0x01, 0x02}));

	REQUIRE(HostTarget::canTxFrames().size() == 1);
	CHECK(HostTarget::canTxFrames()[0].id == 0x00012345);
	CHECK(HostTarget::canTxFrames()[0].extId);
	CHECK(HostTarget::canTxFrames()[0].payload() == std::vector<uint8_t>{0x01, 0x02});
}

TEST_CASE_FIXTURE(BridgeFixture, "a build-frame command for another bus sends nothing") {
	enterGvret(bridge);

	gvret(bridge, bytes({0xF1, 0x00, 0x23, 0x01, 0x00, 0x00, 0x01, 0x01, 0xAA}));

	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(BridgeFixture, "a build-frame transfer shorter than 8 bytes sends nothing") {
	enterGvret(bridge);

	gvret(bridge, bytes({0xF1, 0x00, 0x23, 0x01, 0x00, 0x00, 0x00}));

	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(BridgeFixture, "characterization: a build-frame length above 8 is passed to the CAN frame unclipped (NEEDS REVIEW 125)") {
	// CanBridge.cpp:190-193: only the copy is clipped to 8 bytes; sendMessage gets msgLen 12 and HAL_CAN_AddTxMessage
	// writes DLC 12 into the mailbox (the HAL does not check IS_CAN_DLC without USE_FULL_ASSERT). DLC 9..15 means 8 data
	// bytes on a CAN 2.0 bus; a length of 16 or more would corrupt the TDTR register (DLC = len & 0xF)
	enterGvret(bridge);

	gvret(bridge, bytes({0xF1, 0x00, 0x23, 0x01, 0x00, 0x00, 0x00, 0x0C, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}));

	REQUIRE(HostTarget::canTxFrames().size() == 1);
	CHECK(HostTarget::canTxFrames()[0].dlc == 12);
	CHECK(HostTarget::canTxFrames()[0].payload() == std::vector<uint8_t>{1, 2, 3, 4, 5, 6, 7, 8});
}

TEST_CASE_FIXTURE(BridgeFixture, "several GVRET commands in one transfer are all answered in order") {
	enterGvret(bridge);

	gvret(bridge, bytes({0xF1, 0x0C, 0xF1, 0x09, 0xF1, 0x07}));

	CHECK(cdcOut() == bytes({0xF1, 0x0C, 0x01, 0xF1, 0x09, 0xDE, 0xAD, 0xF1, 0x07, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00}));
}

TEST_CASE_FIXTURE(BridgeFixture, "a GVRET command after one 0xE7 byte in the same transfer is answered") {
	gvret(bridge, bytes({0xE7, 0xF1, 0x09}));

	CHECK(cdcOut() == bytes({0xF1, 0x09, 0xDE, 0xAD}));
}

TEST_CASE_FIXTURE(BridgeFixture, "the connect sequence of SavvyCAN in one transfer is answered") {
	gvret(bridge, bytes({0xE7, 0xE7, 0xF1, 0x0C, 0xF1, 0x06, 0xF1, 0x07}));

	CHECK(cdcOut() == bytes({0xF1, 0x0C, 0x01, 0xF1, 0x06, 0x01, 0x20, 0xA1, 0x07, 0x00, 0x00, 0x20, 0xA1, 0x07, 0x00, 0xF1,
							 0x07, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00}));
}

TEST_CASE_FIXTURE(BridgeFixture, "a GVRET command after two 0xE7 bytes in the same transfer is answered" *
									 KNOWN_ISSUE("bug 117: the start marker search (CanBridge.cpp:156-162) uses `i < *Len-pos` "
												 "while it sets pos = i+1 in the loop, so it only looks at the first half of the "
												 "transfer: in E7 E7 F1 09 the F1 at index 2 is not checked, pos stays 2 and 0xF1 "
												 "is executed as an (unknown) command; the keepalive is not answered. The CHECK "
												 "fails")) {
	gvret(bridge, bytes({0xE7, 0xE7, 0xF1, 0x09}));
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(cdcOut() == bytes({0xF1, 0x09, 0xDE, 0xAD}));
}

TEST_CASE_FIXTURE(BridgeFixture, "bytes without a 0xF1 start marker are not executed as a command" *
									 KNOWN_ISSUE("bug 117: when the shortened search (CanBridge.cpp:156-162) finds no 0xF1, pos "
												 "is left at the middle of the transfer and the byte there is executed as a "
												 "command: E7 E7 07 is answered with the device info. The CHECK fails")) {
	gvret(bridge, bytes({0xE7, 0xE7, 0x07}));
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(cdcOut().empty());
}

TEST_CASE_FIXTURE(BridgeFixture, "a data byte 0xF1 of a build-frame command is not read as the start of a new command" *
									 KNOWN_ISSUE("bug 118: cdcRcv splits the transfer at every 0xF1 (CanBridge.cpp:171-178) "
												 "instead of using the length of each command; build-frame reads its fields by "
												 "offset, but the parser continues after the 0xF1 in the frame data and executes "
												 "the next byte (here 07: device info, answered on CDC). An id or data byte 0xF1 "
												 "can also send a second frame. The CHECK fails")) {
	enterGvret(bridge);

	gvret(bridge, bytes({0xF1, 0x00, 0x23, 0x01, 0x00, 0x00, 0x00, 0x02, 0xF1, 0x07}));
	REQUIRE_ALL_HAL_MODELLED();

	REQUIRE(HostTarget::canTxFrames().size() == 1);
	REQUIRE(HostTarget::canTxFrames()[0].payload() == std::vector<uint8_t>{0xF1, 0x07});
	CHECK(cdcOut().empty());
}

TEST_CASE_FIXTURE(BridgeFixture, "a GVRET command at the end of the received data is answered without reading past the data" *
									 KNOWN_ISSUE_UB_HIDDEN("bug 119: the search for the next 0xF1 (CanBridge.cpp:172-177) runs "
														   "`i < *Len-pos` over data = Buf+pos+1, so for the last command it "
														   "reads data[*Len-pos-1] = Buf[*Len], one byte past the received "
														   "data (heap-buffer-overflow with an exact-size buffer; in cdcbuf of "
														   "64 bytes only when 64 bytes were received). The byte does not "
														   "change the result")) {
	enterGvret(bridge);
	std::vector<char> exact = {static_cast<char>(0xF1), 0x09};
	uint32_t len = static_cast<uint32_t>(exact.size());

	bridge.cdcRcv(exact.data(), &len);

	CHECK(cdcOut() == bytes({0xF1, 0x09, 0xDE, 0xAD}));
}

TEST_CASE_FIXTURE(BridgeFixture, "characterization: unsupported GVRET commands are ignored without a reply (NEEDS REVIEW 123)") {
	// CanBridge.cpp:206-216, 250-271: get digital in (2), get analog in (3), set digital out (4), single wire (8), set
	// system (10), echo frame (11), get / set extended buses (13, 14) and unknown commands do nothing; GVRET answers
	// 2, 3 and 13 (SavvyCAN does not use them)
	enterGvret(bridge);

	for (int cmd : {0x02, 0x03, 0x04, 0x08, 0x0A, 0x0B, 0x0D, 0x0E, 0x20}) {
		CAPTURE(cmd);
		gvret(bridge, bytes({0xF1, cmd, 0x00, 0x00}));
		CHECK(cdcOut().empty());
	}
	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(BridgeFixture, "text command lines are still executed in GVRET mode") {
	enterGvret(bridge);
	CommandHelpers::CdcConsole console(bridge);

	CHECK(console.send("main.0.spd?\n") == "[main.0.spd?|500000]\n");
}

TEST_CASE_FIXTURE(BridgeFixture, "GVRET commands received through the CDC port are answered") {
	enterGvret(bridge);
	HostUsb::cdcReceive(bytes({0xF1, 0x09}));

	bridge.cdcRcvReady(0); // tud_cdc_rx_cb of global_callbacks.cpp

	CHECK(cdcOut() == bytes({0xF1, 0x09, 0xDE, 0xAD}));
}

// ---------------------------------------------------------------- received frames to the host

TEST_CASE_FIXTURE(BridgeFixture, "a received extended frame is sent to the host with bit 31 of the id set") {
	enterGvret(bridge);
	REQUIRE(HostTarget::canReceive(0x01ABCDEF, {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}, true) == 1);

	bridge.update();

	CHECK(cdcOut() == gvretFrame(0, 0x81ABCDEF, {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}));
}

TEST_CASE_FIXTURE(BridgeFixture, "a received frame without data is sent to the host with length 0") {
	enterGvret(bridge);
	REQUIRE(HostTarget::canReceive(0x10, {}) == 1);

	bridge.update();

	CHECK(cdcOut() == gvretFrame(0, 0x10, {}));
}

TEST_CASE_FIXTURE(BridgeFixture, "the timestamp of a received frame is the time of reception in microseconds" *
									 KNOWN_ISSUE("bug 121: CanBridge.cpp:101 sends the CAN hardware timestamp of the frame "
												 "(CAN_RDTxR TIME, 16 bit, only counting in time triggered mode, which "
												 "MX_CAN1_Init leaves off: 0) instead of microseconds since start up like the "
												 "GVRET protocol and the time sync reply (CanBridge.cpp:200). The CHECK fails")) {
	enterGvret(bridge);
	HostPlatform::advanceMs(5);
	REQUIRE(HostTarget::canReceive(0x123, {0x11}) == 1);

	bridge.update();
	REQUIRE_ALL_HAL_MODELLED();

	REQUIRE(cdcOut().size() == 13);
	CHECK(cdcOut().substr(2, 4) == bytes({0x88, 0x13, 0x00, 0x00})); // 5000 us
}

TEST_CASE_FIXTURE(BridgeFixture, "a received frame with a DLC above 8 is sent to the host with its 8 data bytes") {
	// A CAN 2.0 frame may carry DLC 9..15 (8 data bytes). HAL_CAN_GetRxMessage truncates the DLC to 8
	// (stm32f4xx_hal_can.c:1554-1557), so CanBridge.cpp:112-114 never reads beyond msg.data[8]
	enterGvret(bridge);
	CAN_RxHeaderTypeDef header = {};
	header.StdId = 0x55;
	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = 12;
	const uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	HostTarget::canQueueRx(CAN_RX_FIFO0, header, data);
	REQUIRE(HostTarget::canRxInterrupt(CAN_RX_FIFO0) >= 1);

	bridge.update();

	REQUIRE(cdcOut().size() == 20); // F1 00, time, id, length, 8 data bytes, 0
	CHECK(cdcOut().substr(11, 8) == bytes({1, 2, 3, 4, 5, 6, 7, 8}));
}

TEST_CASE_FIXTURE(BridgeFixture, "update sends nothing when no frame was received") {
	enterGvret(bridge);
	uint32_t writes = HostUsb::state().cdcWrites;

	bridge.update();

	CHECK(cdcOut().empty());
	CHECK(HostUsb::state().cdcWrites == writes);
}

TEST_CASE_FIXTURE(BridgeFixture, "a received frame is sent to the host only once") {
	enterGvret(bridge);
	REQUIRE(HostTarget::canReceive(0x123, {0x11}) == 1);
	bridge.update();
	cdcOut().clear();

	bridge.update();

	CHECK(cdcOut().empty());
}

TEST_CASE_FIXTURE(BridgeFixture, "characterization: only the last frame received before update is sent to the host (NEEDS REVIEW 126)") {
	// canRxPendCallback (CanBridge.cpp:76-85) keeps one frame (lastmsg) and a flag; the vector rxmessages
	// (CanBridge.h) is unused. Frames arriving faster than the main loop calls update() are lost
	enterGvret(bridge);
	REQUIRE(HostTarget::canReceive(0x101, {0x01}) == 1);
	REQUIRE(HostTarget::canReceive(0x102, {0x02}) == 1);

	bridge.update();

	CHECK(cdcOut() == gvretFrame(0, 0x102, {0x02}));
}

TEST_CASE_FIXTURE(BridgeFixture, "a frame from receive FIFO 1 is not sent to the host") {
	enterGvret(bridge);
	CAN_RxHeaderTypeDef header = {};
	header.StdId = 0x55;
	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = 1;
	const uint8_t data[8] = {0x42};
	HostTarget::canQueueRx(CAN_RX_FIFO1, header, data);
	REQUIRE(HostTarget::canRxInterrupt(CAN_RX_FIFO1) >= 1);

	bridge.update();

	CHECK(cdcOut().empty());
}

TEST_CASE_FIXTURE(BridgeFixture, "a received frame pulses the system LED") {
	uint32_t before = HostPlatform::leds().sysPulses;

	REQUIRE(HostTarget::canReceive(0x123, {0x11}) == 1);

	CHECK(HostPlatform::leds().sysPulses == before + 1);
}

TEST_CASE_FIXTURE(BridgeFixture, "in text mode a received frame is logged as CAN:<id>:<data>") {
	REQUIRE(HostTarget::canReceive(0x123, {0x11, 0x22}) == 1);

	bridge.update();

	CHECK(cdcOut() == "[>!CAN:291:8721\n]"); // data as little endian 64 bit number
}

// ---------------------------------------------------------------- text commands

TEST_CASE_FIXTURE(BridgeFixture, "the can get command returns the data and the id of the last received frame") {
	REQUIRE(HostTarget::canReceive(0x123, {0x11, 0x22}) == 1);
	std::vector<CommandReply> replies;

	CHECK(CommandHelpers::run(bridge, "can", CMDtype::get, 0, &replies) == CommandStatus::OK);

	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 0x2211);
	CHECK(replies[0].adr == 0x123);
}

TEST_CASE_FIXTURE(BridgeFixture, "the can command with an address sends an 8 byte frame with that id") {
	CHECK(CommandHelpers::run(bridge, "can", CMDtype::setat, 0x0807060504030201, nullptr, 0x123) == CommandStatus::OK);

	REQUIRE(HostTarget::canTxFrames().size() == 1);
	const HostTarget::CanTxFrame &f = HostTarget::canTxFrames()[0];
	CHECK(f.id == 0x123);
	CHECK_FALSE(f.extId);
	CHECK_FALSE(f.rtr);
	CHECK(f.dlc == 8);
	CHECK(f.value() == 0x0807060504030201u);
}

TEST_CASE_FIXTURE(BridgeFixture, "the can command sends an extended frame for an id with bit 31") {
	CHECK(CommandHelpers::run(bridge, "can", CMDtype::setat, 1, nullptr, 0x80000123) == CommandStatus::OK);

	REQUIRE(HostTarget::canTxFrames().size() == 1);
	CHECK(HostTarget::canTxFrames()[0].id == 0x123);
	CHECK(HostTarget::canTxFrames()[0].extId);
}

TEST_CASE_FIXTURE(BridgeFixture, "a can command line on the CDC port sends the frame") {
	CommandHelpers::CdcConsole console(bridge);

	std::string reply = console.send("main.0.can=4386?291\n"); // 0x1122 to id 0x123
	INFO(reply);

	REQUIRE(HostTarget::canTxFrames().size() == 1);
	CHECK(HostTarget::canTxFrames()[0].id == 0x123);
	CHECK(HostTarget::canTxFrames()[0].value() == 0x1122);
}

TEST_CASE_FIXTURE(BridgeFixture, "the can command rejects a set without address") {
	CHECK(CommandHelpers::run(bridge, "can", CMDtype::set, 5) == CommandStatus::ERR);

	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(BridgeFixture, "the rtr command sends a remote frame with the given id") {
	CHECK(CommandHelpers::run(bridge, "rtr", CMDtype::set, 0x321) == CommandStatus::OK);

	REQUIRE(HostTarget::canTxFrames().size() == 1);
	const HostTarget::CanTxFrame &f = HostTarget::canTxFrames()[0];
	CHECK(f.id == 0x321);
	CHECK(f.rtr);
	CHECK(f.dlc == 8);
}

TEST_CASE_FIXTURE(BridgeFixture, "the rtr get command returns the data and the id of the last received frame") {
	REQUIRE(HostTarget::canReceive(0x77, {0x05}) == 1);
	std::vector<CommandReply> replies;

	CHECK(CommandHelpers::run(bridge, "rtr", CMDtype::get, 0, &replies) == CommandStatus::OK);

	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 5);
	CHECK(replies[0].adr == 0x77);
}

TEST_CASE_FIXTURE(BridgeFixture, "the rtr command rejects a set with address") {
	CHECK(CommandHelpers::run(bridge, "rtr", CMDtype::setat, 1, nullptr, 2) == CommandStatus::ERR);

	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(BridgeFixture, "the spd command reports the speed of the CAN port") {
	CHECK(CommandHelpers::get(bridge, "spd") == 500000);
}

TEST_CASE_FIXTURE(BridgeFixture, "the spd command changes the speed of the CAN port to a preset") {
	CHECK(CommandHelpers::run(bridge, "spd", CMDtype::set, 1000000) == CommandStatus::OK);

	CHECK(canport.getSpeed() == 1000000);
	CHECK(CommandHelpers::get(bridge, "spd") == 1000000);
}

TEST_CASE_FIXTURE(BridgeFixture, "the spd command with a speed that is no preset keeps the speed") {
	CommandHelpers::run(bridge, "spd", CMDtype::set, 300000);

	CHECK(canport.getSpeed() == 500000);
}

TEST_CASE_FIXTURE(BridgeFixture, "an unknown command id of the bridge is not found") {
	ParsedCommand cmd;
	cmd.cmdId = 0x7777;
	cmd.type = CMDtype::get;
	cmd.target = &bridge;
	std::vector<CommandReply> replies;

	CHECK(bridge.command(cmd, replies) == CommandStatus::NOT_FOUND);
}

// ---------------------------------------------------------------- LEDs and errors

TEST_CASE_FIXTURE(BridgeFixture, "a frame that can not be queued pulses the error LED") {
	for (int i = 0; i < 3; i++) { // no other node acknowledges: the 3 mailboxes stay pending
		REQUIRE(CommandHelpers::run(bridge, "can", CMDtype::setat, i, nullptr, 0x100 + i) == CommandStatus::OK);
	}
	REQUIRE(HostTarget::canTxPending() == 0b111);
	uint32_t before = HostPlatform::leds().errPulses;

	CommandHelpers::run(bridge, "can", CMDtype::setat, 3, nullptr, 0x103);

	CHECK(HostPlatform::leds().errPulses == before + 1);
	CHECK(HostTarget::canTxFrames().size() == 3);
}

TEST_CASE_FIXTURE(BridgeFixture, "a sent frame does not pulse the error LED") {
	uint32_t before = HostPlatform::leds().errPulses;

	CommandHelpers::run(bridge, "can", CMDtype::setat, 1, nullptr, 0x100);

	CHECK(HostPlatform::leds().errPulses == before);
}

TEST_CASE_FIXTURE(BridgeFixture, "a CAN error of the port pulses the error LED") {
	HostTarget::canSetIrqLineEnabled(HostTarget::CanIrqLine::Sce, true); // a board whose NVIC enables CAN1_SCE_IRQn
	uint32_t before = HostPlatform::leds().errPulses;

	HostTarget::canSetErrorState(HostTarget::CanErrorState::BusOff);

	REQUIRE((CAN1->ESR & ESR_BOFF) != 0);
	CHECK(HostPlatform::leds().errPulses == before + 1);
}

TEST_CASE_FIXTURE(BridgeFixture, "a CAN error of another port does not pulse the error LED") {
	uint32_t before = HostPlatform::leds().errPulses;

	bridge.canErrorCallback(nullptr, HAL_CAN_ERROR_BOF); // global_callbacks.cpp passes handleToPort(hcan): nullptr for an unknown handle

	CHECK(HostPlatform::leds().errPulses == before);
}

} // TEST_SUITE
