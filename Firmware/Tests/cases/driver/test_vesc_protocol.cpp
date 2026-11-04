/*
 * VESC CAN driver, groups G3 (receive buffer protocol and CRC, V20-V26) and G4 (decoding, V30-V35).
 * Charter: Firmware/Tests/driver/VESC_REQUIREMENTS.md. Driver: Firmware/FFBoard/UserExtensions/Src/VescCAN.cpp.
 *
 * Method: the driver (VESC_1, VESC id 255 = default, OpenFFBoard id 0x40) is constructed and NOT run (no runFor): the
 * replies of the fake VESC are delivered straight into its filters (FakeVescPeer::sendReply / inject), and the effect
 * is read through the public commands `vescstate`, `errorflags`, `voltage` and `pos`. State numbers (VescState,
 * VescCAN.h:34-41): 0 UNKNOWN, 1 INCOMPATIBLE, 2 PONG, 3 COMPATIBLE, 4 READY, 5 ERROR.
 * Hand-built frames use the exact bytes of SRC (VESC firmware 7.01, Tests/bldc-master, git-ignored):
 *   SRC comm/comm_can.c:443-505 (reply framing), :1646-1771 (FILL / PROCESS rules), util/crc.c (CRC-16/XMODEM),
 *   commands.c:231-293 (FW_VERSION), :384-479 (GET_VALUES_SELECTIVE).
 * Reference message A = values reply (mask 0x8100, 24.0 V, fault 0) = 32 00 00 81 00 00 F0 00, CRC 0x523A
 * (independent bit-by-bit CRC-16/XMODEM, poly 0x1021, init 0; check value crc("123456789") = 0x31C3).
 *
 * Host vs target: bytes of a CAN frame beyond the DLC are 0 on the host (the fake HAL zero-fills the 8 data bytes). On the
 * target they come from the mailbox registers (RM0090 32.9.5) and are not guaranteed to be 0: the V26 characterizations
 * depend on this. buffer_rx (64 bytes) is not initialised by the constructor.
 */
#include "doctest.h"
#include "VescFixture.h"
#include "TestHelpers.h"

#include <cstdint>
#include <string>
#include <vector>

namespace {

using Bytes = std::vector<uint8_t>;
using Peer = FakeVescPeer;

constexpr int UNKNOWN = 0, INCOMPATIBLE = 1, COMPATIBLE = 3, READY = 4, ERROR_ST = 5;
constexpr uint8_t OFFB = 0x40;

struct ProtoFixture : VescFixture {
	Ptr<VESC_1> v;

	ProtoFixture() : v(makeVesc1()) {}

	int64_t state() { return CommandHelpers::get(*v, "vescstate"); }
	int64_t errorflags() { return CommandHelpers::get(*v, "errorflags"); }
	int64_t voltage() { return CommandHelpers::get(*v, "voltage"); }
	int64_t pos() { return CommandHelpers::get(*v, "pos"); }

	/** Frame to the driver: id = 0x40 | packet << 8, extended */
	uint32_t inj(uint8_t packet, const Bytes &data) { return peer0.inject(Peer::makeId(OFFB, packet), data); }

	/** Message A (values, 24.0 V, fault 0) as hand-built frames: FILL 0 (7 bytes), FILL 7 (1 byte), PROCESS [id 1, flag 1, len 8, CRC 523A] */
	void sendA() {
		inj(Peer::PKT_FILL_RX_BUFFER, {0x00, 0x32, 0x00, 0x00, 0x81, 0x00, 0x00, 0xF0});
		inj(Peer::PKT_FILL_RX_BUFFER, {0x07, 0x00});
		inj(Peer::PKT_PROCESS_RX_BUFFER, {0x01, 0x01, 0x00, 0x08, 0x52, 0x3A});
	}
	/** Short frame with COMM_GET_VALUES_SELECTIVE, mask 0x8000 (fault code only): 6 bytes payload fits one frame */
	void sendShortFault(uint8_t code) { inj(Peer::PKT_PROCESS_SHORT_BUFFER, {0x01, 0x01, 0x32, 0x00, 0x00, 0x80, 0x00, code}); }

	/** FW reply with the given version and test number (pairing 0), delivered normally */
	void sendFw(uint8_t major, uint8_t minor, uint8_t test, bool pairing = false) {
		peer0.fw().major = major;
		peer0.fw().minor = minor;
		peer0.fw().testVersion = test;
		peer0.fw().pairing = pairing;
		peer0.sendFwVersion();
	}
};

} // namespace

TEST_SUITE("VESC buffer protocol") {

// ---------------------------------------------------------------------------------------------------------------
// V20 multi-frame reply with CRC
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE_FIXTURE(ProtoFixture, "[V20] a FW version reply sent as FILL frames plus PROCESS_RX_BUFFER with a correct CRC is decoded (COMPATIBLE)") {
	CHECK(state() == UNKNOWN);
	peer0.sendFwVersion(); // 31 bytes: FILL at 0, 7, 14, 21, 28 + PROCESS
	CHECK(state() == COMPATIBLE);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V20] hand-built values reply (FILL 00 + 7 bytes, FILL 07 + 1 byte, PROCESS [01 01 00 08 52 3A]) is decoded: READY, 24000 mV") {
	// HD: message A = 32 00 00 81 00 00 F0 00; 24.0 V * 10 = 240 = 0x00F0; CRC-16/XMODEM = 0x523A, high byte first
	sendA();
	CHECK(state() == READY);
	CHECK(voltage() == 24000);
	CHECK(errorflags() == 0);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V20] a PROCESS_RX_BUFFER with the CRC low bit flipped (0x523B) is ignored") {
	inj(Peer::PKT_FILL_RX_BUFFER, {0x00, 0x32, 0x00, 0x00, 0x81, 0x00, 0x00, 0xF0});
	inj(Peer::PKT_FILL_RX_BUFFER, {0x07, 0x00});
	inj(Peer::PKT_PROCESS_RX_BUFFER, {0x01, 0x01, 0x00, 0x08, 0x52, 0x3B});
	CHECK(state() == UNKNOWN);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V20] a CRC sent low byte first (3A 52 instead of 52 3A) is ignored") {
	inj(Peer::PKT_FILL_RX_BUFFER, {0x00, 0x32, 0x00, 0x00, 0x81, 0x00, 0x00, 0xF0});
	inj(Peer::PKT_FILL_RX_BUFFER, {0x07, 0x00});
	inj(Peer::PKT_PROCESS_RX_BUFFER, {0x01, 0x01, 0x00, 0x08, 0x3A, 0x52});
	CHECK(state() == UNKNOWN);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V20] a FW version reply with a corrupt CRC from the peer is ignored") {
	peer0.setCorruptCrc(true);
	peer0.sendFwVersion();
	CHECK(state() == UNKNOWN);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V20] a reply whose data is changed after the CRC was computed is ignored") {
	// message with fault 3 but the CRC of message A (fault 0): the CRC covers all 8 bytes
	inj(Peer::PKT_FILL_RX_BUFFER, {0x00, 0x32, 0x00, 0x00, 0x81, 0x00, 0x00, 0xF0});
	inj(Peer::PKT_FILL_RX_BUFFER, {0x07, 0x03});
	inj(Peer::PKT_PROCESS_RX_BUFFER, {0x01, 0x01, 0x00, 0x08, 0x52, 0x3A});
	CHECK(state() == UNKNOWN); // (errorflags / voltage are uninitialised before the first reply, B-V13: not read)
}

// ---------------------------------------------------------------------------------------------------------------
// V21 short reply
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE_FIXTURE(ProtoFixture, "[V21] a short reply (PROCESS_SHORT_BUFFER [01 01 32 00 00 80 00 03], 6 byte payload) is decoded: ERROR, error flag 3") {
	// HD: payload 32 | mask 00 00 80 00 (bit 15 only) | fault 3 = 6 bytes -> one frame, DLC 8, no CRC
	sendShortFault(3);
	CHECK(state() == ERROR_ST);
	CHECK(errorflags() == 3);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V21] a short reply built by the fake peer (fault 0) arrives as one PROCESS_SHORT_BUFFER frame and gives READY") {
	CHECK(peer0.sendReply(OFFB, peer0.valuesPayload(0x8000)) == 1);
	REQUIRE(peer0.sent().size() == 1);
	CHECK(peer0.sent()[0].packet == Peer::PKT_PROCESS_SHORT_BUFFER);
	CHECK(peer0.sent()[0].dlc == 8);
	CHECK(state() == READY);
}

// ---------------------------------------------------------------------------------------------------------------
// V22 FILL_RX_BUFFER_LONG
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE_FIXTURE(ProtoFixture, "[V22] a FW version reply sent with FILL_RX_BUFFER_LONG frames is decoded") {
	peer0.setLongFill(true);
	peer0.sendFwVersion();
	REQUIRE_FALSE(peer0.sent().empty());
	CHECK(peer0.sent()[0].packet == Peer::PKT_FILL_RX_BUFFER_LONG);
	CHECK(state() == COMPATIBLE);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V22] hand-built values reply (LONG [00 00 + 6 bytes], LONG [00 06 + 2 bytes], PROCESS) is decoded: READY") {
	// HD: offsets are 2 bytes big endian: 0x0000, then 0x0006 (6 bytes of message A already in)
	inj(Peer::PKT_FILL_RX_BUFFER_LONG, {0x00, 0x00, 0x32, 0x00, 0x00, 0x81, 0x00, 0x00});
	inj(Peer::PKT_FILL_RX_BUFFER_LONG, {0x00, 0x06, 0xF0, 0x00});
	inj(Peer::PKT_PROCESS_RX_BUFFER, {0x01, 0x01, 0x00, 0x08, 0x52, 0x3A});
	CHECK(state() == READY);
	CHECK(voltage() == 24000);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V22] a LONG fill with a wrong CRC is ignored too") {
	peer0.setLongFill(true);
	peer0.setCorruptCrc(true);
	peer0.sendFwVersion();
	CHECK(state() == UNKNOWN);
}

// ---------------------------------------------------------------------------------------------------------------
// V23 length check on PROCESS_RX_BUFFER
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE_FIXTURE(ProtoFixture, "[V23] a FW version reply with a dropped FILL frame (buffer incomplete) is not decoded") {
	// Plain sibling: the CRC of the incomplete buffer differs, so it is rejected by the CRC even though the driver does
	// not count the buffered bytes. The reply is unique (names and UUID differ from every other test), because buffer_rx
	// is not initialised and the heap block of a driver of an earlier test can still hold an identical message.
	peer0.fw().hwName = "XQ7";
	peer0.fw().uuid = {{0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC}};
	peer0.setDropFillFrame(2); // offset 14 left out
	peer0.sendFwVersion();
	CHECK(state() == UNKNOWN);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V23] a PROCESS_RX_BUFFER alone is not decoded when its message was already processed (buffer count 0 != length 8)" * KNOWN_ISSUE("B-V4: the driver has no count of buffered bytes (VescCAN.cpp:643-667), the stale buffer content passes the CRC and is decoded again")) {
	// SRC comm_can.c:1722-1737: after a PROCESS the buffer count is reset to 0; a PROCESS announcing 8 bytes then
	// finds no buffer with count 8 and is dropped.
	sendA();
	CHECK(state() == READY);
	sendShortFault(3);
	CHECK(state() == ERROR_ST);
	inj(Peer::PKT_PROCESS_RX_BUFFER, {0x01, 0x01, 0x00, 0x08, 0x52, 0x3A}); // replay of the PROCESS frame only
	CHECK(state() == ERROR_ST); // defect: READY (message A decoded again)
}

TEST_CASE_FIXTURE(ProtoFixture, "[V23] a PROCESS_RX_BUFFER announcing 7 bytes after 8 were buffered is not decoded" * KNOWN_ISSUE("B-V4: the driver has no count of buffered bytes (VescCAN.cpp:643-667), the first 7 bytes pass a CRC computed over 7 bytes")) {
	// HD: CRC-16/XMODEM of 32 00 00 81 00 00 F0 (first 7 bytes of message A) = 0x7DFD; buffered count is 8, not 7,
	// so SRC drops the PROCESS frame
	sendA();
	sendShortFault(3);
	CHECK(state() == ERROR_ST);
	inj(Peer::PKT_PROCESS_RX_BUFFER, {0x01, 0x01, 0x00, 0x07, 0x7D, 0xFD});
	CHECK(state() == ERROR_ST); // defect: READY
}

TEST_CASE_FIXTURE(ProtoFixture, "[V23] a PROCESS_RX_BUFFER announcing more than the 64 byte buffer is ignored") {
	// characterization of the driver's own limit (BUFFER_RX_SIZE 64); announced length 0x0041 = 65
	sendA();
	sendShortFault(3);
	inj(Peer::PKT_PROCESS_RX_BUFFER, {0x01, 0x01, 0x00, 0x41, 0x52, 0x3A});
	CHECK(state() == ERROR_ST);
}

// ---------------------------------------------------------------------------------------------------------------
// V24 long replies
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE_FIXTURE(ProtoFixture, "[V24] a FW version reply of exactly 64 bytes (the size of the driver's buffer) is decoded") {
	// HD: 29 + n + m = 64 -> n + m = 35: hw name 20 chars, fw name 15 chars; last FILL frame at offset 63 with 1 byte
	peer0.fw().hwName = std::string(20, 'H');
	peer0.fw().fwName = std::string(15, 'F');
	REQUIRE(peer0.fwVersionPayload().size() == 64);
	peer0.sendFwVersion();
	CHECK(state() == COMPATIBLE);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V24] a FW version reply of 70 bytes is decoded" * KNOWN_ISSUE_UB("B-V5: PROCESS_RX_BUFFER with length > 64 is dropped (VescCAN.cpp:651), the FILL frames at offsets 63..69 write beyond buffer_rx (VescCAN.cpp:630)")) {
	// HD: 29 + n + m = 70 -> n + m = 41: hw name 21, fw name 20 chars. SRC rx buffer is 512 bytes, a real VESC
	// with long names sends exactly this. Defect assertion first: the state stays UNKNOWN.
	peer0.fw().hwName = std::string(21, 'H');
	peer0.fw().fwName = std::string(20, 'F');
	CHECK(peer0.fwVersionPayload().size() == 70);
	peer0.sendFwVersion();
	CHECK(state() == COMPATIBLE);
}

// ---------------------------------------------------------------------------------------------------------------
// V25 / V26 malformed traffic (L4: characterization)
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE_FIXTURE(ProtoFixture, "[V25] characterization: FILL frames in reverse order are all copied at their offsets and the message is decoded (a VESC would drop them)") {
	// SRC comm_can.c:1652-1660 drops a FILL whose offset does not continue the running offset (and the PROCESS then
	// finds no buffer with the announced count). The driver copies by offset, the CRC matches -> COMPATIBLE.
	// NEEDS REVIEW (L4): only reachable with a broken sender.
	peer0.setReverseFill(true);
	peer0.sendFwVersion();
	CHECK(state() == COMPATIBLE);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V26] characterization: a PROCESS_RX_BUFFER with DLC 4 (CRC bytes absent, read as 0) is not decoded") {
	// NEEDS REVIEW (L4). Host: bytes beyond the DLC are 0, CRC of message A (0x523A) != 0x0000. On the target the
	// bytes beyond the DLC are not guaranteed to be 0.
	sendA();
	sendShortFault(3);
	CHECK(state() == ERROR_ST);
	inj(Peer::PKT_PROCESS_RX_BUFFER, {0x01, 0x01, 0x00, 0x08});
	CHECK(state() == ERROR_ST);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V26] characterization: a PROCESS_SHORT_BUFFER with DLC 2 (no payload) decodes nothing") {
	// NEEDS REVIEW (L4). length - 2 = 0 -> decode_buffer returns at once
	inj(Peer::PKT_PROCESS_SHORT_BUFFER, {0x01, 0x01});
	CHECK(state() == UNKNOWN);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V26] characterization: a FILL_RX_BUFFER frame with DLC 1 (offset only, no data) copies nothing and decodes nothing") {
	// NEEDS REVIEW (L4). memcpy of 0 bytes
	inj(Peer::PKT_FILL_RX_BUFFER, {0x00});
	CHECK(state() == UNKNOWN);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V26] a PROCESS_SHORT_BUFFER with DLC 1 does not decode anything" * KNOWN_ISSUE_UB("candidate defect (no B-V id yet): header.length - 2 underflows to 0xFFFFFFFF, decode_buffer gets len 255 and reads the bytes beyond the frame as a FW_VERSION message (VescCAN.cpp:671)")) {
	// Intended: a frame without payload is ignored (L4 says characterization, but the observed result is a state
	// change from reading unrelated bytes, so the intended result is asserted under a marker).
	inj(Peer::PKT_PROCESS_SHORT_BUFFER, {0x01});
	CHECK(state() == UNKNOWN);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V26] a FILL_RX_BUFFER frame with DLC 0 does not crash" * doctest::skip() * doctest::description("KNOWN ISSUE (crash): candidate defect (no B-V id yet): header.length - 1 underflows to 0xFFFFFFFF, memcpy of 4 GB (VescCAN.cpp:630). Skipped because it terminates the test run")) {
	inj(Peer::PKT_FILL_RX_BUFFER, {});
	CHECK(state() == UNKNOWN);
}

} // TEST_SUITE

// =====================================================================================================================
TEST_SUITE("VESC decoding") {

// ---------------------------------------------------------------------------------------------------------------
// V30 / V31 FW_VERSION
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE_FIXTURE(ProtoFixture, "[V30] a paired release 5.3 (pairing 1, test version 0) is COMPATIBLE" * KNOWN_ISSUE("B-V6: the driver reads the pairing flag as the test version (VescCAN.cpp:529-530), so 5.3 'test 1' < 5.3.51 gives INCOMPATIBLE")) {
	// SRC commands.c:250-251: pairing_done is sent BEFORE FW_TEST_VERSION_NUMBER (layout of 7.01, charter L1)
	sendFw(5, 3, 0, true);
	CHECK(state() == COMPATIBLE);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V30] a release 5.3 with pairing 0 is COMPATIBLE") {
	sendFw(5, 3, 0, false);
	CHECK(state() == COMPATIBLE);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V31] characterization: release 7.01 is COMPATIBLE") {
	sendFw(7, 1, 0);
	CHECK(state() == COMPATIBLE);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V31] characterization: release 5.2 is INCOMPATIBLE") {
	// FW_MIN_RELEASE = 5.3.51 (VescCAN.h:25): releases compare (major << 8 | minor) with 0x0503
	sendFw(5, 2, 0);
	CHECK(state() == INCOMPATIBLE);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V31] characterization: test build 5.3.51 is COMPATIBLE") {
	sendFw(5, 3, 51);
	CHECK(state() == COMPATIBLE);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V31] characterization: test build 5.3.50 is INCOMPATIBLE" * KNOWN_ISSUE("B-V6: the driver never reads the test version byte (VescCAN.cpp:529-530 reads the pairing flag), so a test build is judged by the release rule and 5.3.50 is COMPATIBLE")) {
	// intended rule (VescCAN.h:25 FW_MIN_RELEASE 5.3.51): 5.3.50 < 5.3.51. Note: [V31] 5.3.51 passes only by accident
	// (the release rule accepts 5.3).
	sendFw(5, 3, 50);
	CHECK(state() == INCOMPATIBLE);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V31] characterization: release 6.0 is COMPATIBLE") {
	sendFw(6, 0, 0);
	CHECK(state() == COMPATIBLE);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V31] characterization: a FW reply moves INCOMPATIBLE to COMPATIBLE when a newer version is reported") {
	sendFw(5, 2, 0);
	CHECK(state() == INCOMPATIBLE);
	sendFw(7, 1, 0);
	CHECK(state() == COMPATIBLE);
}

// V32 (B-V7, swapped memcpy) and V35 (unrequested bits) are not observable through the public interface (charter L5):
// no tests.

// ---------------------------------------------------------------------------------------------------------------
// V33 GET_VALUES_SELECTIVE
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE_FIXTURE(ProtoFixture, "[V33] a values reply with 24.0 V gives the voltage command 24000 mV") {
	// HD: raw 0x00F0 = 240, / 10 = 24.0 V, * 1000 = 24000
	peer0.sendValues();
	CHECK(voltage() == 24000);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V33] a values reply with 12.5 V gives 12500 mV") {
	// HD: raw 0x007D = 125, / 10 = 12.5 V
	peer0.setVin(12.5f);
	peer0.sendValues();
	CHECK(voltage() == 12500);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V33] the voltage is a signed 16 bit value: raw 0xFF9C (-100) gives -10000 mV") {
	// HD: 0xFF9C = -100, / 10 = -10.0 V, * 1000 = -10000; frame built by hand: FILL 0 + 7 bytes, FILL 7 + 1 byte, PROCESS
	// CRC computed with the fake's CRC (independent check vector 0x31C3 is in the fake's self-test)
	const Bytes msg = {0x32, 0x00, 0x00, 0x81, 0x00, 0xFF, 0x9C, 0x00}; // 32 | mask 00 00 81 00 | v_in FF 9C | fault 0
	Bytes fill0 = {0x00};
	fill0.insert(fill0.end(), msg.begin(), msg.begin() + 7);
	Bytes fill1 = {0x07, msg[7]};
	const uint16_t crc = Peer::crc16(msg);
	inj(Peer::PKT_FILL_RX_BUFFER, fill0);
	inj(Peer::PKT_FILL_RX_BUFFER, fill1);
	inj(Peer::PKT_PROCESS_RX_BUFFER, {0x01, 0x01, 0x00, 0x08, uint8_t(crc >> 8), uint8_t(crc)});
	CHECK(state() == READY);
	CHECK(voltage() == -10000);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V33] fault code 0 gives READY and error flags 0") {
	peer0.sendValues();
	CHECK(state() == READY);
	CHECK(errorflags() == 0);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V33] fault code 3 gives ERROR and error flags 3") {
	peer0.setFaultCode(3);
	peer0.sendValues();
	CHECK(state() == ERROR_ST);
	CHECK(errorflags() == 3);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V33] fault code 255 gives ERROR and error flags 255 (uint8)") {
	peer0.setFaultCode(255);
	peer0.sendValues();
	CHECK(state() == ERROR_ST);
	CHECK(errorflags() == 255);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V33] a reply with fault 0 after a fault returns from ERROR to READY") {
	peer0.setFaultCode(3);
	peer0.sendValues();
	CHECK(state() == ERROR_ST);
	peer0.setFaultCode(0);
	peer0.sendValues();
	CHECK(state() == READY);
	CHECK(errorflags() == 0);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V33] the voltage is also taken from a reply that reports a fault") {
	peer0.setVin(12.5f);
	peer0.setFaultCode(3);
	peer0.sendValues();
	CHECK(voltage() == 12500);
	CHECK(errorflags() == 3);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V33] a values reply moves a COMPATIBLE driver to READY") {
	peer0.sendFwVersion();
	CHECK(state() == COMPATIBLE);
	peer0.sendValues();
	CHECK(state() == READY);
}

// ---------------------------------------------------------------------------------------------------------------
// V34 bit 16 pid_pos
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE_FIXTURE(ProtoFixture, "[V34] pid_pos 90 degrees in a values reply (mask 0x18100) gives position 0.25 turns (250000000 in the pos command)") {
	// HD: 90 deg = 90e6 = 0x055D4A80 (int32 BE); 90 / 360 = 0.25 turns; pos command = 0.25 * 1e9 = 250000000
	peer0.setPidPosDeg(90.0);
	peer0.sendValues(OFFB, 0x18100);
	CHECK(state() == READY);
	CHECK(pos() == 250000000);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V34] pid_pos is read after the fault byte: 12.5 V, fault 3 and 90 degrees all decode") {
	peer0.setVin(12.5f);
	peer0.setFaultCode(3);
	peer0.setPidPosDeg(90.0);
	peer0.sendValues(OFFB, 0x18100);
	CHECK(voltage() == 12500);
	CHECK(errorflags() == 3);
	CHECK(pos() == 250000000);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V34] pid_pos -90 degrees (int32 two's complement) gives position -0.25 turns") {
	// HD: -90e6 = 0xFAA2B580; delta -90 does not count a turn (|delta| <= 180)
	peer0.setPidPosDeg(-90.0);
	peer0.sendValues(OFFB, 0x18100);
	CHECK(pos() == -250000000);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V34] mask 0x10000 (pid_pos only) decodes the position and leaves the state alone") {
	// HD: payload 32 | 00 01 00 00 | int32 = 9 bytes (FILL + PROCESS)
	peer0.setPidPosDeg(90.0);
	peer0.sendValues(OFFB, 0x10000);
	CHECK(pos() == 250000000);
	CHECK(state() == UNKNOWN);
}

TEST_CASE_FIXTURE(ProtoFixture, "[V34] the position from pid_pos equals the position of a POLL_ROTOR_POS reply of the same angle") {
	peer0.setPidPosDeg(90.0);
	peer0.sendValues(OFFB, 0x18100);
	const int64_t fromValues = pos();
	peer0.setRotorDeg(90.0);
	peer0.sendRotorPosition();
	CHECK(pos() == fromValues);
}

} // TEST_SUITE
