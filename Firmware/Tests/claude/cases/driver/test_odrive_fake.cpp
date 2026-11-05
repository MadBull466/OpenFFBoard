/*
 * Self-tests of FakeOdrivePeer (fakes/FakeOdrivePeer.h): id decode, strict decoding per CSV, call and response, cyclic
 * timing, axis state model and fault injection. The ODrive driver class is NOT used here: the test plays the master
 * through the real canport (CANPort_2B on the fake HAL CAN), exactly the way the driver sends (CAN_tx_msg), and receives
 * the peer's frames through filters like the driver's (id = node << 5, mask 0x7E0).
 * Source of the expected values: "CAN Protocol - ODrive Documentation 0.5.6" (SPEC) and its message table (CSV), see the fake's header.
 */
#include "doctest.h"
#include "OdriveFixture.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {

using V = FakeOdrivePeer::ViolationKind;
using Bytes = std::vector<uint8_t>;

/** A CAN handler that records what the filters of the test let through */
struct RxCapture : CanHandler {
	struct Rx {
		uint32_t id;
		bool ext, rtr;
		uint32_t dlc;
		std::array<uint8_t, 8> data;
		uint8_t fifo;
	};
	std::vector<Rx> rx;
	void canRxPendCallback(CANPort *, CAN_rx_msg &msg) override {
		Rx r{msg.header.id, msg.header.extId, msg.header.rtr, msg.header.length, {}, msg.fifo};
		std::memcpy(r.data.data(), msg.data, 8);
		rx.push_back(r);
	}
};

struct PeerFixture : OdriveFixture {
	RxCapture cap;

	PeerFixture() {
		canport.setSilentMode(false);
		canport.takePort();
		CAN_filter f0;
		f0.filter_id = 0 << 5;
		f0.filter_mask = 0x7E0;
		f0.buffer = 0;
		canport.addCanFilter(f0);
		CAN_filter f1;
		f1.filter_id = 1 << 5;
		f1.filter_mask = 0x7E0;
		f1.buffer = 1;
		canport.addCanFilter(f1);
		peer1.setHeartbeat(false); // keeps the capture free of node 1 heartbeats caught up by process()
	}
	~PeerFixture() override { canport.freePort(); }

	/** The master (driver side) puts a frame on the bus; the peers decode it */
	void master(uint32_t id, const Bytes &data, bool rtr = false, bool ext = false) {
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
	static uint32_t nowMs() { return static_cast<uint32_t>(HostPlatform::nowUs() / 1000); }
};

} // namespace

TEST_SUITE("FakeOdrivePeer") {

TEST_CASE_FIXTURE(PeerFixture, "a torque frame of node 0 is decoded by id >> 5 and id & 0x1F and its float is little endian (CSV 0x00E)") {
	// 1.5f = 0x3FC00000 -> bytes 00 00 C0 3F
	master(0x00E, {0x00, 0x00, 0xC0, 0x3F});
	REQUIRE(peer0.log().size() == 1);
	CHECK(peer0.log()[0].cmd == 0x0E);
	CHECK(peer0.torqueSeen());
	CHECK(peer0.lastTorque() == 1.5f);
	CHECK(peer0.foreignFrames() == 0);
}

TEST_CASE_FIXTURE(PeerFixture, "a frame of node 1 (id 0x2E = 1 << 5 | 0x0E) is logged by the node 1 peer and counted as foreign by the node 0 peer") {
	master(0x2E, {0x00, 0x00, 0x80, 0x3F}); // 1.0f
	CHECK(peer0.log().empty());
	CHECK(peer0.foreignFrames() == 1);
	REQUIRE(peer1.log().size() == 1);
	CHECK(peer1.lastTorque() == 1.0f);
	CHECK(peer1.violations().empty());
}

TEST_CASE_FIXTURE(PeerFixture, "a set state frame (DLC 4, uint32 little endian) is logged and requests that state") {
	master(0x007, {0x08, 0x00, 0x00, 0x00});
	REQUIRE(peer0.requestedStates().size() == 1);
	CHECK(peer0.requestedStates()[0] == 8);
	CHECK(peer0.log()[0].u32(0) == 8);
	CHECK(peer0.violations().empty());
}

TEST_CASE_FIXTURE(PeerFixture, "a set controller modes frame (DLC 8, two int32) is decoded signed little endian (CSV 0x00B)") {
	master(0x00B, {0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00});
	CHECK(peer0.modesSeen());
	CHECK(peer0.lastControlMode() == -1);
	CHECK(peer0.lastInputMode() == 3);
	CHECK(peer0.violations().empty());
}

TEST_CASE_FIXTURE(PeerFixture, "a wrong DLC on a set message is a violation and the frame is not applied (CSV 0x007 4, 0x00B 8, 0x00E 4)") {
	master(0x007, Bytes(8, 0));
	master(0x007, Bytes(3, 0));
	master(0x00B, Bytes(4, 0));
	master(0x00E, Bytes(8, 0));
	REQUIRE(peer0.violations().size() == 4);
	for (const auto &v : peer0.violations()) {
		CHECK(v.kind == V::WrongDlc);
	}
	CHECK(peer0.requestedStates().empty());
	CHECK_FALSE(peer0.modesSeen());
	CHECK_FALSE(peer0.torqueSeen());
	CHECK_FALSE(peer0.log()[0].accepted);
	peer0.expectViolation(V::WrongDlc);
	CHECK(peer0.unexpectedViolations().empty());
}

TEST_CASE_FIXTURE(PeerFixture, "an RTR request with data (DLC != 0) is a violation and is not answered") {
	master(0x009, {0x01}, true);
	REQUIRE(peer0.violations().size() == 1);
	CHECK(peer0.violations()[0].kind == V::WrongDlc);
	CHECK(peer0.sent().empty());
	CHECK(cap.rx.empty());
	peer0.expectViolation(V::WrongDlc, 0x09);
}

TEST_CASE_FIXTURE(PeerFixture, "RTR on a set message and a data frame on a call-and-response message are violations (SPEC call and response)") {
	master(0x00E, {}, true);					   // RTR on set torque
	master(0x009, Bytes(8, 0));					   // data frame on get encoder estimates
	master(0x017, Bytes(8, 0));					   // data frame on get bus voltage
	REQUIRE(peer0.violations().size() == 3);
	for (const auto &v : peer0.violations()) {
		CHECK(v.kind == V::RtrMismatch);
	}
	CHECK_FALSE(peer0.torqueSeen());
	CHECK(peer0.sent().empty());
	peer0.expectViolation(V::RtrMismatch);
}

TEST_CASE_FIXTURE(PeerFixture, "a command id outside the driver's set is an UnknownCommand violation (estop 0x002)") {
	master(0x002, {});
	REQUIRE(peer0.violations().size() == 1);
	CHECK(peer0.violations()[0].kind == V::UnknownCommand);
	CHECK(peer0.violations()[0].cmd == 0x02);
	CHECK(peer0.unexpectedViolations().size() == 1);
	peer0.expectViolation(V::UnknownCommand, 0x02);
	CHECK(peer0.unexpectedViolations().empty());
}

TEST_CASE_FIXTURE(PeerFixture, "an expected violation of another command does not hide a violation") {
	master(0x002, {});
	peer0.expectViolation(V::UnknownCommand, 0x03);
	peer0.expectViolation(V::WrongDlc);
	CHECK(peer0.unexpectedViolations().size() == 1);
	peer0.expectViolation(V::UnknownCommand);
}

TEST_CASE_FIXTURE(PeerFixture, "an extended id frame for this node is a violation, for another node it is foreign") {
	master(0x00E, {0, 0, 0, 0}, false, true);		  // extended, node 0
	master((1u << 5) | 0x0E, {0, 0, 0, 0}, false, true); // extended, node 1
	REQUIRE(peer0.violations().size() == 1);
	CHECK(peer0.violations()[0].kind == V::ExtendedId);
	CHECK(peer0.foreignFrames() == 1);
	CHECK_FALSE(peer0.torqueSeen());
	peer0.expectViolation(V::ExtendedId);
	peer1.expectViolation(V::ExtendedId);
}

TEST_CASE_FIXTURE(PeerFixture, "start anticogging with DLC 0 is clean and with DLC 1 is undocumented, not a violation (L3, CSV 0x010 has no signals)") {
	master(0x010, {});
	CHECK(peer0.undocumented().empty());
	master(0x010, {0x00});
	CHECK(peer0.anticoggingCount() == 2);
	REQUIRE(peer0.undocumented().size() == 1);
	CHECK(peer0.undocumented()[0].dlc == 1);
	CHECK(peer0.violations().empty());
}

TEST_CASE_FIXTURE(PeerFixture, "an RTR request of the encoder estimates is answered with the same id and DLC 8, position then velocity as little endian float32 (CSV 0x009)") {
	peer0.setEncoder(1.25f, -2.5f); // 0x3FA00000, 0xC0200000
	master(0x009, {}, true);
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].id == 0x009);
	CHECK_FALSE(cap.rx[0].rtr);
	CHECK_FALSE(cap.rx[0].ext);
	CHECK(cap.rx[0].dlc == 8);
	const std::array<uint8_t, 8> want{0x00, 0x00, 0xA0, 0x3F, 0x00, 0x00, 0x20, 0xC0};
	CHECK(cap.rx[0].data == want);
	CHECK(peer0.rtrCount(0x09) == 1);
}

TEST_CASE_FIXTURE(PeerFixture, "an RTR request of the bus voltage is answered with voltage then current (CSV 0x017)") {
	peer0.setBus(24.0f, 1.0f); // 0x41C00000, 0x3F800000
	master(0x017, {}, true);
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].id == 0x017);
	const std::array<uint8_t, 8> want{0x00, 0x00, 0xC0, 0x41, 0x00, 0x00, 0x80, 0x3F};
	CHECK(cap.rx[0].data == want);
}

TEST_CASE_FIXTURE(PeerFixture, "the node 1 response uses id 0x029 and arrives on FIFO 1 when the Rx1 line is enabled (fixture default)") {
	master((1u << 5) | 0x09, {}, true);
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].id == ((1u << 5) | 0x09));
	CHECK(cap.rx[0].fifo == 1);
}

TEST_CASE_FIXTURE(PeerFixture, "with the real F407 interrupt set-up a node 1 frame waits in FIFO 1 until the next TX completion") {
	setRx1Interrupt(false);
	peer1.setHeartbeat(false);
	master((1u << 5) | 0x09, {}, true);
	REQUIRE(peer1.sent().size() == 1);
	CHECK(peer1.sent()[0].accepted == 0); // no interrupt: nothing delivered yet
	CHECK(cap.rx.empty());
	// a TX completion raises CAN1_TX, whose handler also services FIFO 1
	master(0x00E, {0, 0, 0, 0});
	CHECK(cap.rx.size() == 1);
}

TEST_CASE_FIXTURE(PeerFixture, "a frame whose TX mailbox is not completed never reaches the peer; after the completion it does") {
	setTxAutoComplete(false);
	CAN_tx_msg m;
	m.header.id = 0x00E;
	m.header.length = 4;
	REQUIRE(canport.sendMessage(m));
	process();
	CHECK(peer0.log().empty());
	REQUIRE(HostTarget::canTxComplete(0));
	process();
	CHECK(peer0.log().size() == 1);
}

TEST_CASE_FIXTURE(PeerFixture, "the heartbeat comes every 100 ms on simulated time, the first one 100 ms after the start") {
	peer0.advance(99);
	CHECK(peer0.sent().empty());
	peer0.advance(1);
	REQUIRE(peer0.sent().size() == 1);
	peer0.advance(900);
	REQUIRE(peer0.sent().size() == 10);
	for (size_t i = 0; i < 10; i++) {
		CHECK(peer0.sent()[i].timeMs == 100 * (i + 1));
		CHECK(peer0.sent()[i].id == 0x001);
		CHECK(peer0.sent()[i].dlc == 8);
	}
	CHECK(cap.rx.size() == 10);
}

TEST_CASE_FIXTURE(PeerFixture, "the heartbeat payload follows CSV 0x001: error uint32 @0, state @4, flags in bit 0 of bytes 5 to 7, trajectory done bit 7 of byte 7") {
	peer0.setAxisError(0x01020304);
	peer0.setState(8);
	peer0.setFlags(true, true, true, true);
	peer0.sendHeartbeat();
	REQUIRE(cap.rx.size() == 1);
	const std::array<uint8_t, 8> want{0x04, 0x03, 0x02, 0x01, 0x08, 0x01, 0x01, 0x81};
	CHECK(cap.rx[0].data == want);
	peer0.setFlags(false, false, false, true);
	peer0.sendHeartbeat();
	CHECK(cap.rx[1].data[5] == 0);
	CHECK(cap.rx[1].data[6] == 0);
	CHECK(cap.rx[1].data[7] == 0x80);
}

TEST_CASE_FIXTURE(PeerFixture, "the heartbeat period is configurable and the heartbeat can be switched off") {
	peer0.setHeartbeat(true, 250);
	peer0.advance(1000);
	CHECK(peer0.sent().size() == 4);
	peer0.setHeartbeat(false);
	peer0.advance(1000);
	CHECK(peer0.sent().size() == 4);
}

TEST_CASE_FIXTURE(PeerFixture, "the cyclic encoder estimates are off by default and run every 10 ms when enabled (SPEC cyclic defaults)") {
	peer0.setHeartbeat(false);
	peer0.advance(100);
	CHECK(peer0.sent().empty());
	peer0.setEncoderCyclic(true);
	peer0.setEncoder(0.5f, 0);
	peer0.advance(100);
	REQUIRE(peer0.sent().size() == 10);
	CHECK(peer0.sent()[0].id == 0x009);
	CHECK(peer0.sent()[1].timeMs - peer0.sent()[0].timeMs == 10);
}

TEST_CASE_FIXTURE(PeerFixture, "a process() call after a long time jump delivers one cyclic frame and counts the skipped ones") {
	peer0.setHeartbeat(true, 100);
	HostPlatform::advanceMs(1000);
	peer0.process();
	CHECK(peer0.sent().size() == 1);
	CHECK(peer0.skippedCyclic() == 9);
	CHECK(peer0.nextEventUs() > HostPlatform::nowUs());
}

TEST_CASE_FIXTURE(PeerFixture, "a calibration request puts the axis into that state at once and back to IDLE after the calibration time") {
	peer0.setHeartbeat(false);
	peer0.setCalibrationMs(500);
	master(0x007, {3, 0, 0, 0});
	CHECK(peer0.state() == FakeOdrivePeer::STATE_FULL_CALIBRATION);
	peer0.advance(499);
	CHECK(peer0.state() == FakeOdrivePeer::STATE_FULL_CALIBRATION);
	peer0.advance(1);
	CHECK(peer0.state() == FakeOdrivePeer::STATE_IDLE);
}

TEST_CASE_FIXTURE(PeerFixture, "a closed loop request stays in closed loop") {
	master(0x007, {8, 0, 0, 0});
	peer0.advance(10000);
	CHECK(peer0.state() == FakeOdrivePeer::STATE_CLOSED_LOOP);
}

TEST_CASE_FIXTURE(PeerFixture, "a configured apply delay postpones the state change and a new request replaces the pending one") {
	peer0.setHeartbeat(false);
	peer0.setRequestApplyDelayMs(300);
	master(0x007, {8, 0, 0, 0});
	peer0.advance(200);
	CHECK(peer0.state() == FakeOdrivePeer::STATE_IDLE);
	master(0x007, {1, 0, 0, 0}); // replaces the pending closed loop request
	peer0.advance(1000);
	CHECK(peer0.state() == FakeOdrivePeer::STATE_IDLE);
	CHECK(peer0.requestedStates() == std::vector<uint32_t>({8, 1}));
}

TEST_CASE_FIXTURE(PeerFixture, "in refusal mode requests are recorded as refused, the state stays and the configured axis error is raised") {
	peer0.setRefuseRequests(true, 0x1);
	master(0x007, {3, 0, 0, 0});
	CHECK(peer0.state() == FakeOdrivePeer::STATE_IDLE);
	CHECK(peer0.refusedStates() == std::vector<uint32_t>({3}));
	CHECK(peer0.axisError() == 0x1);
	peer0.setRefuseRequests(false);
	peer0.refuseState(8);
	master(0x007, {8, 0, 0, 0});
	master(0x007, {3, 0, 0, 0});
	CHECK(peer0.state() == FakeOdrivePeer::STATE_FULL_CALIBRATION);
	CHECK(peer0.refusedStates().size() == 2);
}

TEST_CASE_FIXTURE(PeerFixture, "a silent axis sends nothing and answers nothing but still logs the requests") {
	peer0.setSilent(true);
	peer0.advance(1000);
	master(0x009, {}, true);
	master(0x00E, {0, 0, 0, 0});
	CHECK(peer0.sent().empty());
	CHECK(cap.rx.empty());
	CHECK(peer0.log().size() == 2);
	CHECK(peer0.sendHeartbeat() == 0);
}

TEST_CASE_FIXTURE(PeerFixture, "with responses off the heartbeat continues but RTR requests are not answered") {
	peer0.setRespond(false);
	master(0x009, {}, true);
	CHECK(peer0.sent().empty());
	peer0.advance(100);
	REQUIRE(peer0.sent().size() == 1);
	CHECK(peer0.sent()[0].id == 0x001);
}

TEST_CASE_FIXTURE(PeerFixture, "a late response arrives after the configured delay and not before") {
	peer0.setHeartbeat(false);
	peer0.setResponseDelayMs(50);
	master(0x009, {}, true);
	peer0.advance(49);
	CHECK(cap.rx.empty());
	peer0.advance(1);
	CHECK(cap.rx.size() == 1);
}

TEST_CASE_FIXTURE(PeerFixture, "a wrong node id on the axis frames makes the driver side filter reject them") {
	peer0.setFaultNode(5);
	peer0.sendHeartbeat();
	REQUIRE(peer0.sent().size() == 1);
	CHECK(peer0.sent()[0].id == ((5u << 5) | 0x01));
	CHECK(peer0.sent()[0].accepted == 0);
	CHECK(cap.rx.empty());
}

TEST_CASE_FIXTURE(PeerFixture, "a short frame fault sends the configured DLC and the unused bytes are 0") {
	peer0.setAxisError(0xFFFFFFFF);
	peer0.setFaultDlc(4);
	peer0.sendHeartbeat();
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].dlc == 4);
	CHECK(cap.rx[0].data[3] == 0xFF);
	CHECK(cap.rx[0].data[4] == 0);
}

TEST_CASE_FIXTURE(PeerFixture, "an extended id fault sends the same id value as an extended frame") {
	peer0.setFaultExtended(true);
	peer0.sendHeartbeat();
	REQUIRE(peer0.sent().size() == 1);
	CHECK(peer0.sent()[0].extId);
	CHECK(peer0.sent()[0].id == 0x001);
}

TEST_CASE_FIXTURE(PeerFixture, "clearFaults removes every injected fault") {
	peer0.setSilent(true);
	peer0.setRespond(false);
	peer0.setFaultNode(3);
	peer0.setFaultDlc(2);
	peer0.setFaultExtended(true);
	peer0.setResponseDelayMs(10);
	peer0.clearFaults();
	peer0.sendHeartbeat();
	REQUIRE(cap.rx.size() == 1);
	CHECK(cap.rx[0].id == 0x001);
	CHECK(cap.rx[0].dlc == 8);
	CHECK_FALSE(cap.rx[0].ext);
}

TEST_CASE_FIXTURE(PeerFixture, "a raw injected frame bypasses silence and faults") {
	peer0.setSilent(true);
	CHECK(peer0.inject(0x001, Bytes(8, 0)) > 0);
	CHECK(cap.rx.size() == 1);
}

TEST_CASE_FIXTURE(PeerFixture, "a bus-off node does not receive the peer's frames") {
	HostTarget::canSetErrorState(HostTarget::CanErrorState::BusOff);
	peer0.sendHeartbeat();
	REQUIRE(peer0.sent().size() == 1);
	CHECK(peer0.sent()[0].accepted == 0);
	CHECK(cap.rx.empty());
}

TEST_CASE_FIXTURE(PeerFixture, "after clearTxFrames the peer does not decode old frames again and decodes new ones") {
	master(0x00E, {0, 0, 0, 0});
	clearTxFrames();
	master(0x00E, {0, 0, 0x80, 0x3F});
	CHECK(peer0.count(0x0E) == 2);
	CHECK(peer0.lastTorque() == 1.0f);
}

} // TEST_SUITE
