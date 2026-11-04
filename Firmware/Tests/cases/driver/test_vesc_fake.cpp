/*
 * Self-tests of FakeVescPeer (fakes/FakeVescPeer.h): CRC, reply framing, COMM layouts, request decoding, timeout model
 * and fault injection. The VESC driver class is NOT used here: the test plays the master (the driver side) through the
 * real canport (CANPort_2B on the fake HAL CAN), exactly the way the driver sends (CAN_tx_msg, extended ids), and
 * receives the peer's frames through filters like the driver's (id low byte 0x40, mask 0xFF; POLL_ROTOR_POS 56 << 8,
 * mask 0xFF00).
 * Source of the expected values: VESC firmware 7.01 (SRC, cited in the fake's header) and hand derivations (HD) in the
 * comments; the CRC values were derived with an independent bit-by-bit CRC-16/XMODEM (poly 0x1021, init 0).
 */
#include "doctest.h"
#include "VescFixture.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {

using V = FakeVescPeer::ViolationKind;
using Bytes = std::vector<uint8_t>;
constexpr uint8_t OFFB = 0x40; // the driver's default id: replies are addressed to it

/** A CAN handler that records what the filters of the test let through */
struct RxCapture : CanHandler {
	struct Rx {
		uint32_t id;
		bool ext, rtr;
		uint32_t dlc;
		std::array<uint8_t, 8> data;
		uint8_t packet() const { return static_cast<uint8_t>(id >> 8); }
		uint8_t unit() const { return static_cast<uint8_t>(id); }
		Bytes bytes() const { return Bytes(data.begin(), data.begin() + dlc); }
	};
	std::vector<Rx> rx;
	void canRxPendCallback(CANPort *, CAN_rx_msg &msg) override {
		Rx r{msg.header.id, msg.header.extId, msg.header.rtr, msg.header.length, {}};
		std::memcpy(r.data.data(), msg.data, 8);
		rx.push_back(r);
	}
};

/**
 * Receiver written from SRC comm_can.c:1646-1771 (FILL at the running offset, PROCESS accepted when the buffered count
 * equals the length and the CRC matches): returns the reassembled COMM messages. Independent of the driver.
 */
std::vector<Bytes> reassemble(const std::vector<RxCapture::Rx> &rx, bool *crcFailed = nullptr) {
	std::vector<Bytes> out;
	Bytes buf(512, 0);
	size_t count = 0;
	if (crcFailed) {
		*crcFailed = false;
	}
	for (const auto &f : rx) {
		const Bytes d = f.bytes();
		switch (f.packet()) {
		case FakeVescPeer::PKT_FILL_RX_BUFFER:
			if (d.size() >= 1 && d[0] == count) {
				std::memcpy(buf.data() + d[0], d.data() + 1, d.size() - 1);
				count = d[0] + d.size() - 1;
			}
			break;
		case FakeVescPeer::PKT_FILL_RX_BUFFER_LONG:
			if (d.size() >= 2 && size_t((d[0] << 8) | d[1]) == count) {
				std::memcpy(buf.data() + count, d.data() + 2, d.size() - 2);
				count += d.size() - 2;
			}
			break;
		case FakeVescPeer::PKT_PROCESS_RX_BUFFER: {
			const size_t len = (d[2] << 8) | d[3];
			const uint16_t crc = static_cast<uint16_t>((d[4] << 8) | d[5]);
			if (len == count && FakeVescPeer::crc16(buf.data(), len) == crc) {
				out.emplace_back(buf.begin(), buf.begin() + len);
			} else if (crcFailed) {
				*crcFailed = true;
			}
			count = 0;
			break;
		}
		case FakeVescPeer::PKT_PROCESS_SHORT_BUFFER:
			out.emplace_back(d.begin() + 2, d.end());
			break;
		}
	}
	return out;
}

struct PeerFixture : VescFixture {
	RxCapture cap;

	PeerFixture() {
		canport.setSilentMode(false);
		canport.takePort();
		CAN_filter f0; // replies to the driver's id
		f0.extid = true;
		f0.filter_id = OFFB;
		f0.filter_mask = 0xFF;
		f0.buffer = 0;
		canport.addCanFilter(f0);
		CAN_filter f1; // POLL_ROTOR_POS replies of any unit
		f1.extid = true;
		f1.filter_id = uint32_t(FakeVescPeer::PKT_POLL_ROTOR_POS) << 8;
		f1.filter_mask = 0xFF00;
		f1.buffer = 0;
		canport.addCanFilter(f1);
	}
	~PeerFixture() override { canport.freePort(); }

	/** The master (driver side) puts a frame on the bus; the peers decode it. Extended data frame by default */
	void master(uint32_t id, const Bytes &data, bool ext = true, bool rtr = false) {
		CAN_tx_msg m;
		m.header.id = id;
		m.header.length = static_cast<uint32_t>(data.size());
		m.header.rtr = rtr;
		m.header.extId = ext;
		for (size_t i = 0; i < data.size(); i++) {
			m.data[i] = data[i];
		}
		REQUIRE(canport.sendMessage(m));
		process();
	}
	/** FW version request exactly as the driver sends it (VescCAN.cpp:405-413) */
	void requestFw(uint8_t unit = 0x01) { master(FakeVescPeer::makeId(unit, 8), {OFFB, 0x00, 0x00}); }
	/** Values request exactly as the driver sends it, DLC 7 or 8 */
	void requestValues(uint8_t unit = 0x01, size_t dlc = 7, uint32_t mask = 0x8100) {
		Bytes d = {OFFB, 0x00, 50, uint8_t(mask >> 24), uint8_t(mask >> 16), uint8_t(mask >> 8), uint8_t(mask), 0xA5};
		d.resize(dlc);
		master(FakeVescPeer::makeId(unit, 8), d);
	}
	static uint32_t nowMs() { return static_cast<uint32_t>(HostPlatform::nowUs() / 1000); }
};

} // namespace

TEST_SUITE("FakeVescPeer") {

// ---------------------------------------------------------------------------------------------------------------
// CRC
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("the peer's CRC-16/XMODEM gives the check value 0x31C3 for the ASCII string 123456789") {
	const char *s = "123456789";
	CHECK(FakeVescPeer::crc16(reinterpret_cast<const uint8_t *>(s), 9) == 0x31C3);
}

TEST_CASE("the CRC of nothing and of a zero byte is 0 (init 0)") {
	CHECK(FakeVescPeer::crc16(nullptr, 0) == 0x0000);
	const uint8_t z[1] = {0};
	CHECK(FakeVescPeer::crc16(z, 1) == 0x0000);
}

TEST_CASE("CRC values of the lead's vectors: 00 32 -> 0x1611, 00 05 06 -> 0x9F33, 00..10 -> 0x65E5") {
	CHECK(FakeVescPeer::crc16(Bytes{0x00, 0x32}) == 0x1611);
	CHECK(FakeVescPeer::crc16(Bytes{0x00, 0x05, 0x06}) == 0x9F33);
	Bytes seq;
	for (int i = 0; i < 17; i++) {
		seq.push_back(static_cast<uint8_t>(i));
	}
	CHECK(FakeVescPeer::crc16(seq) == 0x65E5);
}

TEST_CASE("appending the CRC high byte first to a message makes the CRC of the whole 0") {
	Bytes m = {0x32, 0x00, 0x00, 0x81, 0x00, 0x00, 0xF0, 0x00};
	const uint16_t c = FakeVescPeer::crc16(m);
	m.push_back(uint8_t(c >> 8));
	m.push_back(uint8_t(c));
	CHECK(FakeVescPeer::crc16(m) == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// reply framing
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE_FIXTURE(PeerFixture, "a reply of up to 6 bytes is one PROCESS_SHORT_BUFFER frame [own id, 1, payload...] with DLC len + 2 (SRC comm_can.c:445-455)") {
	CHECK(peer0.sendReply(OFFB, {0x32, 0xAA, 0xBB}) == 1);
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].id == 0x0840); // 0x40 | 8 << 8
	CHECK(cap.rx[0].ext);
	CHECK(cap.rx[0].dlc == 5);
	CHECK(cap.rx[0].bytes() == Bytes({0x01, 0x01, 0x32, 0xAA, 0xBB}));
}

TEST_CASE_FIXTURE(PeerFixture, "a reply of exactly 6 bytes is still one short frame with DLC 8 and 7 bytes need FILL + PROCESS") {
	peer0.sendReply(OFFB, Bytes(6, 0x11));
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].packet() == FakeVescPeer::PKT_PROCESS_SHORT_BUFFER);
	CHECK(cap.rx[0].dlc == 8);
	cap.rx.clear();
	peer0.sendReply(OFFB, Bytes(7, 0x11));
	REQUIRE(cap.rx.size() == 2);
	CHECK(cap.rx[0].packet() == FakeVescPeer::PKT_FILL_RX_BUFFER);
	CHECK(cap.rx[1].packet() == FakeVescPeer::PKT_PROCESS_RX_BUFFER);
}

TEST_CASE_FIXTURE(PeerFixture, "a 17 byte reply is FILL frames at offsets 0, 7 and 14 with 7, 7 and 3 data bytes followed by PROCESS_RX_BUFFER [id, 1, 00 11, CRC] (SRC comm_can.c:462-503)") {
	Bytes seq;
	for (int i = 0; i < 17; i++) {
		seq.push_back(static_cast<uint8_t>(i));
	}
	CHECK(peer0.sendReply(OFFB, seq) == 4);
	REQUIRE(cap.rx.size() == 4);
	for (int i = 0; i < 3; i++) {
		CHECK(cap.rx[i].id == 0x0540); // 0x40 | 5 << 8
	}
	// HD: offset byte, then 7 / 7 / 3 data bytes; DLC = data + 1
	CHECK(cap.rx[0].bytes() == Bytes({0, 0, 1, 2, 3, 4, 5, 6}));
	CHECK(cap.rx[0].dlc == 8);
	CHECK(cap.rx[1].bytes() == Bytes({7, 7, 8, 9, 10, 11, 12, 13}));
	CHECK(cap.rx[2].bytes() == Bytes({14, 14, 15, 16}));
	CHECK(cap.rx[2].dlc == 4);
	// PROCESS_RX_BUFFER: DLC 6, [sender 1, flag 1, length 0x0011, CRC 0x65E5 high byte first]
	CHECK(cap.rx[3].id == 0x0740);
	CHECK(cap.rx[3].dlc == 6);
	CHECK(cap.rx[3].bytes() == Bytes({0x01, 0x01, 0x00, 0x11, 0x65, 0xE5}));
	bool crcFailed = true;
	const auto msgs = reassemble(cap.rx, &crcFailed);
	REQUIRE(msgs.size() == 1);
	CHECK(msgs[0] == seq);
	CHECK_FALSE(crcFailed);
}

TEST_CASE_FIXTURE(PeerFixture, "the reply id is the requester id | packet << 8, the requester is payload byte 0 of the request") {
	master(FakeVescPeer::makeId(0x01, 8), {OFFB, 0x00, 0x00});
	REQUIRE_FALSE(cap.rx.empty());
	CHECK(cap.rx[0].unit() == 0x40);
	CHECK(cap.rx.back().packet() == FakeVescPeer::PKT_PROCESS_RX_BUFFER);
	CHECK(cap.rx.back().data[0] == 0x01); // the VESC's own id in payload byte 0
	CHECK(cap.rx.back().data[1] == 0x01); // flag 1: the answer goes to the driver's "send" path
	// another requester id: the reply goes there (the driver's filters would not let it through)
	peer0.clearLog();
	master(FakeVescPeer::makeId(0x01, 8), {0x55, 0x00, 0x00});
	REQUIRE_FALSE(peer0.sent().empty());
	CHECK(peer0.sent()[0].id == (0x0500u | 0x55));
}

TEST_CASE_FIXTURE(PeerFixture, "a reply with a message longer than 259 bytes continues with FILL_RX_BUFFER_LONG frames [offset hi, offset lo, up to 6 bytes] (SRC comm_can.c:478-492)") {
	Bytes big(270);
	for (size_t i = 0; i < big.size(); i++) {
		big[i] = static_cast<uint8_t>(i * 7 + 3);
	}
	peer0.sendReply(OFFB, big);
	// FILL at offsets 0, 7, ..., 252 (i <= 255): 37 frames, end_a = 259; LONG from 259: 11 bytes in 2 frames; + PROCESS
	REQUIRE(cap.rx.size() == 37 + 2 + 1);
	CHECK(cap.rx[36].bytes()[0] == 252);
	CHECK(cap.rx[37].packet() == FakeVescPeer::PKT_FILL_RX_BUFFER_LONG);
	CHECK(cap.rx[37].bytes()[0] == 0x01); // 259 = 0x0103
	CHECK(cap.rx[37].bytes()[1] == 0x03);
	CHECK(cap.rx[37].dlc == 8);
	CHECK(cap.rx[38].bytes()[0] == 0x01); // 265 = 0x0109
	CHECK(cap.rx[38].bytes()[1] == 0x09);
	CHECK(cap.rx[38].dlc == 7); // 5 bytes left + 2
	const auto msgs = reassemble(cap.rx);
	REQUIRE(msgs.size() == 1);
	CHECK(msgs[0] == big);
}

// ---------------------------------------------------------------------------------------------------------------
// COMM layouts
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE_FIXTURE(PeerFixture, "the default FW_VERSION reply is the 7.01 layout: 0, 7, 1, HW name, NUL, 12 UUID bytes, 8 flag bytes, FW name, NUL, 4 CRC bytes (SRC commands.c:231-293)") {
	const Bytes p = peer0.fwVersionPayload();
	// HD: 3 + (2 + 1) + 12 + 8 + (0 + 1) + 4 = 31 bytes for HW_NAME "60" and an empty FW_NAME
	REQUIRE(p.size() == 31);
	CHECK(p[0] == 0);
	CHECK(p[1] == 7);
	CHECK(p[2] == 1);
	CHECK(p[3] == '6');
	CHECK(p[4] == '0');
	CHECK(p[5] == 0);
	for (int i = 0; i < 12; i++) {
		CHECK(p[6 + i] == i + 1); // UUID
	}
	CHECK(p[26] == 0); // empty FW name terminator
	CHECK(Bytes(p.begin() + 27, p.end()) == Bytes({0xDE, 0xAD, 0xBE, 0xEF})); // hw crc, big endian
}

TEST_CASE_FIXTURE(PeerFixture, "the FW_VERSION reply has the pairing flag BEFORE the test version number (SRC commands.c:250-251)") {
	peer0.fw().pairing = true;
	peer0.fw().testVersion = 51;
	const Bytes p = peer0.fwVersionPayload();
	// UUID ends at index 6 + 12 = 18 (exclusive): pairing_done at 18, FW_TEST_VERSION_NUMBER at 19, HW_TYPE at 20
	REQUIRE(p.size() > 21);
	CHECK(p[18] == 1);
	CHECK(p[19] == 51);
	CHECK(p[20] == 0);
}

TEST_CASE_FIXTURE(PeerFixture, "a FW_VERSION reply with long names has the length 29 + n + m and the hardware CRC as the last 4 bytes big endian") {
	peer0.fw().hwName = std::string(40, 'H');
	peer0.fw().fwName = std::string(30, 'F');
	const Bytes p = peer0.fwVersionPayload();
	REQUIRE(p.size() == 29 + 40 + 30); // HD: 3 + 41 + 12 + 8 + 31 + 4 = 99
	CHECK(p.size() > 64);
	CHECK(Bytes(p.end() - 4, p.end()) == Bytes({0xDE, 0xAD, 0xBE, 0xEF}));
}

TEST_CASE_FIXTURE(PeerFixture, "a request for the firmware version is answered with FILL + PROCESS frames that reassemble to the FW_VERSION payload") {
	requestFw();
	CHECK(peer0.requestCount(0) == 1);
	bool crcFailed = true;
	const auto msgs = reassemble(cap.rx, &crcFailed);
	REQUIRE(msgs.size() == 1);
	CHECK_FALSE(crcFailed);
	CHECK(msgs[0] == peer0.fwVersionPayload());
	CHECK(cap.rx.back().packet() == FakeVescPeer::PKT_PROCESS_RX_BUFFER);
}

TEST_CASE_FIXTURE(PeerFixture, "the values reply for mask 0x8100 (24.0 V, fault 0) is 32 00 00 81 00 00 F0 00 sent as FILL 00 + 7 bytes, FILL 07 + 1 byte, PROCESS [01 01 00 08 52 3A]") {
	// HD: 24.0 V * 10 = 240 = 0x00F0 int16 big endian, fault 0; CRC-16/XMODEM of the 8 bytes = 0x523A
	requestValues();
	REQUIRE(cap.rx.size() == 3);
	CHECK(cap.rx[0].id == 0x0540);
	CHECK(cap.rx[0].bytes() == Bytes({0x00, 0x32, 0x00, 0x00, 0x81, 0x00, 0x00, 0xF0}));
	CHECK(cap.rx[1].bytes() == Bytes({0x07, 0x00}));
	CHECK(cap.rx[2].id == 0x0740);
	CHECK(cap.rx[2].bytes() == Bytes({0x01, 0x01, 0x00, 0x08, 0x52, 0x3A}));
}

TEST_CASE_FIXTURE(PeerFixture, "the values payload for 12.5 V and fault 3 is 32 00 00 81 00 00 7D 03 with CRC 0x0F9D, 12.59 V truncates to raw 125") {
	peer0.setVin(12.5f);
	peer0.setFaultCode(3);
	const Bytes p = peer0.valuesPayload(0x8100);
	CHECK(p == Bytes({0x32, 0x00, 0x00, 0x81, 0x00, 0x00, 0x7D, 0x03}));
	CHECK(FakeVescPeer::crc16(p) == 0x0F9D);
	peer0.setVin(12.59f); // 12.59 * 10 = 125.9 -> (int16) truncates toward zero to 125 = 0x007D (SRC buffer.c:71-72)
	CHECK(peer0.valuesPayload(0x8100) == Bytes({0x32, 0x00, 0x00, 0x81, 0x00, 0x00, 0x7D, 0x03}));
}

TEST_CASE_FIXTURE(PeerFixture, "the values payload with bit 16 appends pid_pos as int32 big endian degrees * 1e6 after the fault byte") {
	peer0.setPidPosDeg(90.0); // HD: 90e6 = 0x055D4A80
	const Bytes p = peer0.valuesPayload(0x18100);
	CHECK(p == Bytes({0x32, 0x00, 0x01, 0x81, 0x00, 0x00, 0xF0, 0x00, 0x05, 0x5D, 0x4A, 0x80}));
}

TEST_CASE_FIXTURE(PeerFixture, "a mask with bits the fake does not model gives no payload") {
	CHECK(peer0.valuesPayload(0x8101).empty());
	CHECK(peer0.valuesPayload(1u << 17).empty());
	CHECK_FALSE(peer0.valuesPayload(0).empty()); // [50, 00 00 00 00]
}

TEST_CASE_FIXTURE(PeerFixture, "POLL_ROTOR_POS is answered with the VESC's own id | 56 << 8, DLC 4, int32 big endian degrees * 1e5 (SRC comm_can.c:2023-2030)") {
	peer0.setRotorDeg(90.0); // HD: 90 * 100000 = 9000000 = 0x00895440
	master(FakeVescPeer::makeId(0x01, 56), {});
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].id == 0x3801);
	CHECK(cap.rx[0].ext);
	CHECK(cap.rx[0].dlc == 4);
	CHECK(cap.rx[0].bytes() == Bytes({0x00, 0x89, 0x54, 0x40}));
}

TEST_CASE_FIXTURE(PeerFixture, "the rotor angle truncates toward zero and a negative angle is two's complement") {
	peer0.setRotorDeg(359.999996); // * 1e5 = 35999999.6 -> 35999999
	peer0.sendRotorPosition();
	// HD: 36000000 = 0x02255100, so 35999999 = 0x022550FF
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].bytes() == Bytes({0x02, 0x25, 0x50, 0xFF}));
	cap.rx.clear();
	peer0.setRotorDeg(-0.000015); // * 1e5 = -1.5 -> -1 = FF FF FF FF
	peer0.sendRotorPosition();
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].bytes() == Bytes({0xFF, 0xFF, 0xFF, 0xFF}));
}

TEST_CASE_FIXTURE(PeerFixture, "a broadcast POLL_ROTOR_POS request (unit 255) is answered by every present peer, each with its own id") {
	peer1.setPresent(true);
	peer1.reset();
	master(FakeVescPeer::makeId(255, 56), {});
	REQUIRE(cap.rx.size() == 2);
	CHECK(cap.rx[0].id == 0x3801);
	CHECK(cap.rx[1].id == 0x3802);
}

TEST_CASE_FIXTURE(PeerFixture, "a PING [0x40] is answered with a PONG to 0x40 | 18 << 8, DLC 2 [own id, hw type 0] (SRC comm_can.c:1843-1850)") {
	master(FakeVescPeer::makeId(0x01, 17), {OFFB});
	CHECK(peer0.pingCount() == 1);
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].id == 0x1240);
	CHECK(cap.rx[0].dlc == 2);
	CHECK(cap.rx[0].bytes() == Bytes({0x01, 0x00}));
}

TEST_CASE_FIXTURE(PeerFixture, "an unsolicited PONG can be sent without a PING") {
	CHECK(peer0.sendPong() == 1);
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].id == 0x1240);
	CHECK(peer0.pingCount() == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// request decoding and violations
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE_FIXTURE(PeerFixture, "SET_CURRENT_REL (packet 10, DLC 4, int32 big endian) is decoded and the ratio is raw / 1e5") {
	master(FakeVescPeer::makeId(0x01, 10), {0x00, 0x00, 0xC3, 0x51}); // 50001
	REQUIRE(peer0.log().size() == 1);
	CHECK(peer0.currentSeen());
	CHECK(peer0.lastCurrentRaw() == 50001);
	CHECK(peer0.lastCurrentRatio() == doctest::Approx(0.50001f));
	master(FakeVescPeer::makeId(0x01, 10), {0xFF, 0xFE, 0x79, 0x60}); // -100000
	CHECK(peer0.lastCurrentRaw() == -100000);
	CHECK(peer0.lastCurrentRatio() == -1.0f);
	CHECK(cap.rx.empty()); // SET commands are not answered
	CHECK(peer0.violations().empty());
}

TEST_CASE_FIXTURE(PeerFixture, "frames to unit 255 are accepted, frames to another unit are foreign and ignored") {
	master(FakeVescPeer::makeId(255, 10), {0, 0, 0, 1});
	CHECK(peer0.lastCurrentRaw() == 1);
	master(FakeVescPeer::makeId(0x07, 10), {0, 0, 0, 2});
	CHECK(peer0.lastCurrentRaw() == 1);
	CHECK(peer0.foreignFrames() == 1);
	CHECK(peer0.log().size() == 1);
}

TEST_CASE_FIXTURE(PeerFixture, "an absent peer neither logs nor answers") {
	peer0.setPresent(false);
	requestFw();
	CHECK(peer0.log().empty());
	CHECK(cap.rx.empty());
}

TEST_CASE_FIXTURE(PeerFixture, "the values request with DLC 7 is clean, with DLC 8 the 8th byte is recorded as undocumented and the request is answered (B-V2)") {
	requestValues(0x01, 7);
	CHECK(peer0.undocumented().empty());
	requestValues(0x01, 8);
	REQUIRE(peer0.undocumented().size() == 1);
	CHECK(peer0.undocumented()[0].dlc == 8);
	CHECK(peer0.undocumented()[0].extraByte == 0xA5);
	CHECK(peer0.violations().empty());
	CHECK(peer0.requestCount(50) == 2);
	CHECK(reassemble(cap.rx).size() == 2);
}

TEST_CASE_FIXTURE(PeerFixture, "a values request with DLC 6 is a WrongDlc violation and is not answered") {
	requestValues(0x01, 6);
	REQUIRE(peer0.violations().size() == 1);
	CHECK(peer0.violations()[0].kind == V::WrongDlc);
	CHECK(cap.rx.empty());
	CHECK_FALSE(peer0.log()[0].accepted);
	CHECK(peer0.unexpectedViolations().size() == 1);
	peer0.expectViolation(V::WrongDlc, 8);
	CHECK(peer0.unexpectedViolations().empty());
}

TEST_CASE_FIXTURE(PeerFixture, "violations: standard id, RTR, id bits 16-28, flag, unknown packet, unknown COMM, unmodelled mask, wrong DLCs") {
	master(0x0108, {OFFB, 0, 0}, false);						  // standard id
	master(FakeVescPeer::makeId(1, 56), {}, true, true);			  // RTR
	master(FakeVescPeer::makeId(1, 56) | (1u << 16), {});		  // id bit 16
	master(FakeVescPeer::makeId(1, 8), {OFFB, 1, 0});			  // flag 1
	master(FakeVescPeer::makeId(1, 7), {});						  // packet 7 (driver never sends it)
	master(FakeVescPeer::makeId(1, 8), {OFFB, 0, 22});			  // COMM_ROTOR_POSITION is not a request of the driver
	requestValues(1, 7, 0x8101);								  // mask bit 0 not modelled
	master(FakeVescPeer::makeId(1, 8), {OFFB, 0});				  // DLC 2
	master(FakeVescPeer::makeId(1, 8), {OFFB, 0, 0, 0});		  // FW request with DLC 4
	master(FakeVescPeer::makeId(1, 10), {0, 0, 1});				  // SET_CURRENT_REL DLC 3
	master(FakeVescPeer::makeId(1, 17), {});					  // PING DLC 0
	master(FakeVescPeer::makeId(1, 56), {0});					  // POLL DLC 1
	const std::vector<V> want = {V::StandardId, V::RemoteFrame, V::UpperIdBits, V::BadFlag, V::UnknownPacket, V::UnknownComm,
								  V::UnsupportedMask, V::WrongDlc, V::WrongDlc, V::WrongDlc, V::WrongDlc, V::WrongDlc};
	REQUIRE(peer0.violations().size() == want.size());
	for (size_t i = 0; i < want.size(); i++) {
		CHECK(peer0.violations()[i].kind == want[i]);
	}
	CHECK(cap.rx.empty());
	CHECK_FALSE(peer0.currentSeen());
	for (auto k : {V::StandardId, V::RemoteFrame, V::UpperIdBits, V::BadFlag, V::UnknownPacket, V::UnknownComm, V::UnsupportedMask, V::WrongDlc}) {
		peer0.expectViolation(k);
	}
	CHECK(peer0.unexpectedViolations().empty());
	CHECK(std::string(peer0.violations()[0].text()).size() > 10);
}

TEST_CASE_FIXTURE(PeerFixture, "an expected violation of another packet does not hide a violation") {
	master(FakeVescPeer::makeId(1, 7), {});
	peer0.expectViolation(V::UnknownPacket, 6);
	peer0.expectViolation(V::WrongDlc);
	CHECK(peer0.unexpectedViolations().size() == 1);
	peer0.expectViolation(V::UnknownPacket, 7);
	CHECK(peer0.unexpectedViolations().empty());
}

TEST_CASE_FIXTURE(PeerFixture, "frames that the bus did not acknowledge are not seen by the peer") {
	setTxAutoComplete(false);
	CAN_tx_msg m;
	m.header.id = FakeVescPeer::makeId(1, 10);
	m.header.length = 4;
	m.header.extId = true;
	REQUIRE(canport.sendMessage(m));
	process();
	CHECK(peer0.log().empty());
	HostTarget::canTxCompleteAll();
	peer0.process();
	CHECK(peer0.log().size() == 1);
}

// ---------------------------------------------------------------------------------------------------------------
// timeout model
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE_FIXTURE(PeerFixture, "the VESC times out more than 1000 ms after the last SET_CURRENT_REL and the effective ratio is 0 then (SRC timeout.c:186-253)") {
	master(FakeVescPeer::makeId(1, 10), {0x00, 0x00, 0xC3, 0x51});
	CHECK_FALSE(peer0.timedOut());
	HostPlatform::advanceMs(1000);
	CHECK_FALSE(peer0.timedOut()); // exactly 1000 ms: not more than the timeout
	CHECK(peer0.effectiveRatio() == doctest::Approx(0.50001f));
	HostPlatform::advanceMs(1);
	CHECK(peer0.timedOut());
	CHECK(peer0.effectiveRatio() == 0.0f);
	master(FakeVescPeer::makeId(1, 10), {0x00, 0x00, 0xC3, 0x51});
	CHECK_FALSE(peer0.timedOut());
	CHECK(peer0.effectiveRatio() == doctest::Approx(0.50001f));
}

TEST_CASE_FIXTURE(PeerFixture, "requests for values, firmware version, rotor position and PING do not reset the VESC timeout, only SET commands do") {
	master(FakeVescPeer::makeId(1, 10), {0, 0, 0, 1});
	HostPlatform::advanceMs(900);
	requestFw();
	requestValues();
	master(FakeVescPeer::makeId(1, 56), {});
	master(FakeVescPeer::makeId(1, 17), {OFFB});
	HostPlatform::advanceMs(101);
	CHECK(peer0.timedOut());
}

TEST_CASE_FIXTURE(PeerFixture, "the timeout is configurable and 0 disables it") {
	peer0.setTimeoutMs(200);
	master(FakeVescPeer::makeId(1, 10), {0, 0, 0, 1});
	HostPlatform::advanceMs(201);
	CHECK(peer0.timedOut());
	peer0.setTimeoutMs(0);
	CHECK_FALSE(peer0.timedOut());
}

// ---------------------------------------------------------------------------------------------------------------
// fault injection
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE_FIXTURE(PeerFixture, "a silent peer decodes and logs requests but sends nothing, also no unsolicited PONG or reply") {
	peer0.setSilent(true);
	requestFw();
	CHECK(peer0.requestCount(0) == 1);
	CHECK(peer0.sendPong() == 0);
	CHECK(peer0.sendRotorPosition() == 0);
	CHECK(cap.rx.empty());
	CHECK(peer0.sent().empty());
	peer0.clearFaults();
	requestFw();
	CHECK_FALSE(cap.rx.empty());
}

TEST_CASE_FIXTURE(PeerFixture, "with respond off requests are logged but not answered") {
	peer0.setRespond(false);
	requestFw();
	master(FakeVescPeer::makeId(1, 56), {});
	CHECK(peer0.log().size() == 2);
	CHECK(cap.rx.empty());
}

TEST_CASE_FIXTURE(PeerFixture, "a delayed reply is delivered after the delay in simulated time, all frames in order") {
	peer0.setReplyDelayMs(100);
	requestValues();
	CHECK(cap.rx.empty());
	CHECK(peer0.nextEventUs() == HostPlatform::nowUs() + 100000);
	peer0.advance(99);
	CHECK(cap.rx.empty());
	peer0.advance(1);
	REQUIRE(cap.rx.size() == 3);
	CHECK(cap.rx[0].packet() == FakeVescPeer::PKT_FILL_RX_BUFFER);
	CHECK(cap.rx[2].packet() == FakeVescPeer::PKT_PROCESS_RX_BUFFER);
	CHECK(peer0.sent()[0].timeMs == nowMs());
}

TEST_CASE_FIXTURE(PeerFixture, "a corrupt CRC changes the low bit of the CRC and the reassembly fails") {
	peer0.setCorruptCrc(true);
	requestValues();
	REQUIRE(cap.rx.size() == 3);
	CHECK(cap.rx[2].bytes() == Bytes({0x01, 0x01, 0x00, 0x08, 0x52, 0x3B})); // 0x523A ^ 1
	bool crcFailed = false;
	CHECK(reassemble(cap.rx, &crcFailed).empty());
	CHECK(crcFailed);
}

TEST_CASE_FIXTURE(PeerFixture, "a dropped FILL frame leaves the buffer short and the VESC-side receiver rejects the PROCESS frame") {
	peer0.setDropFillFrame(1);
	requestValues();
	REQUIRE(cap.rx.size() == 2);
	CHECK(cap.rx[0].bytes()[0] == 0);
	CHECK(cap.rx[1].packet() == FakeVescPeer::PKT_PROCESS_RX_BUFFER);
	CHECK(reassemble(cap.rx).empty());
}

TEST_CASE_FIXTURE(PeerFixture, "reversed FILL frames arrive out of sequence: offset 7 before offset 0") {
	peer0.setReverseFill(true);
	requestValues();
	REQUIRE(cap.rx.size() == 3);
	CHECK(cap.rx[0].bytes()[0] == 7);
	CHECK(cap.rx[1].bytes()[0] == 0);
	CHECK(cap.rx[2].packet() == FakeVescPeer::PKT_PROCESS_RX_BUFFER);
	CHECK(reassemble(cap.rx).empty());
}

TEST_CASE_FIXTURE(PeerFixture, "long fill framing sends the whole buffer as FILL_RX_BUFFER_LONG frames of 6 bytes at 2 byte offsets") {
	peer0.setLongFill(true);
	requestValues();
	REQUIRE(cap.rx.size() == 3); // 8 bytes: 6 + 2, then PROCESS
	CHECK(cap.rx[0].packet() == FakeVescPeer::PKT_FILL_RX_BUFFER_LONG);
	CHECK(cap.rx[0].bytes() == Bytes({0x00, 0x00, 0x32, 0x00, 0x00, 0x81, 0x00, 0x00}));
	CHECK(cap.rx[1].bytes() == Bytes({0x00, 0x06, 0xF0, 0x00}));
	CHECK(reassemble(cap.rx).size() == 1);
}

TEST_CASE_FIXTURE(PeerFixture, "a short DLC fault truncates the frames of the peer; with a packet number only that packet") {
	peer0.setFaultDlc(3, FakeVescPeer::PKT_PROCESS_RX_BUFFER);
	requestValues();
	REQUIRE(cap.rx.size() == 3);
	CHECK(cap.rx[0].dlc == 8);
	CHECK(cap.rx[2].dlc == 3);
	CHECK(cap.rx[2].bytes() == Bytes({0x01, 0x01, 0x00}));
	cap.rx.clear();
	peer0.setFaultDlc(1);
	peer0.sendRotorPosition();
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].dlc == 1);
}

TEST_CASE_FIXTURE(PeerFixture, "a wrong sender id changes the id of POLL_ROTOR_POS replies and payload byte 0 of the others") {
	peer0.setFaultSenderId(0x22);
	master(FakeVescPeer::makeId(1, 56), {});
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].id == 0x3822);
	cap.rx.clear();
	requestFw();
	CHECK(cap.rx.back().data[0] == 0x22);
}

TEST_CASE_FIXTURE(PeerFixture, "a wrong destination sends the replies to another id which the driver's filters reject") {
	peer0.setFaultDest(0x41);
	CHECK(peer0.sendReply(OFFB, {0x32}) == 0);
	CHECK(cap.rx.empty());
	REQUIRE(peer0.sent().size() == 1);
	CHECK(peer0.sent()[0].id == 0x0841);
	CHECK(peer0.sent()[0].accepted == 0);
}

TEST_CASE_FIXTURE(PeerFixture, "inject sends a raw frame as it is, also while silent") {
	peer0.setSilent(true);
	CHECK(peer0.inject(0x0840, {0x01, 0x01, 0x32}) == 1);
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].bytes() == Bytes({0x01, 0x01, 0x32}));
}

TEST_CASE_FIXTURE(PeerFixture, "STATUS frames are off by default and carry the VESC's own id, so the driver's filters reject them") {
	peer0.advance(100);
	CHECK(peer0.sent().empty());
	peer0.status().rpm = 1000;
	peer0.status().totalCurrent = 1.5;
	peer0.status().duty = 0.25;
	CHECK(peer0.sendStatus(FakeVescPeer::PKT_STATUS) == 0); // id 0x0901: not 0x40, not packet 56
	REQUIRE(peer0.sent().size() == 1);
	CHECK(peer0.sent()[0].id == 0x0901);
	CHECK(peer0.sent()[0].dlc == 8);
	// HD: rpm 1000 = 0x000003E8, current 1.5 A * 10 = 15 = 0x000F, duty 0.25 * 1000 = 250 = 0x00FA
	const std::array<uint8_t, 8> want{0x00, 0x00, 0x03, 0xE8, 0x00, 0x0F, 0x00, 0xFA};
	CHECK(peer0.sent()[0].data == want);
}

TEST_CASE_FIXTURE(PeerFixture, "cyclic STATUS 1 and 5 frames run at the configured period") {
	peer0.status().vin = 24.0;
	peer0.status().tachometer = -2;
	peer0.setStatusCyclic(0x11, 20);
	peer0.advance(40);
	REQUIRE(peer0.sent().size() == 4);
	CHECK(peer0.sent()[0].id == 0x0901);
	CHECK(peer0.sent()[1].id == 0x1B01); // STATUS_5 = 27 = 0x1B
	// STATUS_5: tachometer int32 (-2 = FF FF FF FE), v_in * 10 = 240 = 0x00F0, 0
	const std::array<uint8_t, 8> want{0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0xF0, 0x00, 0x00};
	CHECK(peer0.sent()[1].data == want);
	CHECK(peer0.sent()[0].timeMs + 20 == peer0.sent()[2].timeMs);
	peer0.setStatusCyclic(0);
	peer0.advance(100);
	CHECK(peer0.sent().size() == 4);
}

TEST_CASE_FIXTURE(PeerFixture, "reset clears logs and faults but keeps the unit id") {
	peer0.setSilent(true);
	peer0.setVin(5.0f);
	requestFw();
	peer0.reset();
	CHECK(peer0.log().empty());
	CHECK_FALSE(peer0.silent());
	CHECK(peer0.vin() == 24.0f);
	CHECK(peer0.unitId() == 0x01);
}

TEST_CASE_FIXTURE(PeerFixture, "the log cursor follows clearTxFrames so old frames are not decoded twice") {
	requestFw();
	CHECK(peer0.log().size() == 1);
	clearTxFrames();
	requestFw();
	CHECK(peer0.log().size() == 2);
	CHECK(peer0.requestCount(0) == 2);
}

}
