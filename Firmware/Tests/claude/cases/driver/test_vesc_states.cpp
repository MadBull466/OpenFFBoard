/*
 * VESC CAN driver: G5 state machine and timing (V40-V44) and G6 torque (V50-V53).
 * Requirements: Tests/claude/charters/driver/VESC_REQUIREMENTS.md (lead decisions L1-L7).
 *
 * Time base (code path of VescCAN::Run, VescCAN.cpp:353-394): the thread body runs at 500 ms, 1000 ms, ... after
 * construction (the fixture schedules it, charter D2 L9). Body order: (1) UNKNOWN -> FW version request, end of body;
 * (2) state >= COMPATIBLE and port not waiting -> values request; (3) READY / ERROR and HAL_GetTick() -
 * lastVescResponse > 1000 -> UNKNOWN; (4) lastTorque != 0, started, READY, port not waiting -> SET_CURRENT_REL.
 * lastVescResponse is refreshed by every frame that reaches canRxPendCallback (VescCAN.cpp:605), the peer answers
 * inside the same millisecond as the request.
 * Timeout: reply at R, bodies at R+500 (age 500), R+1000 (age 1000, not > 1000: still READY), R+1500 (age 1500 > 1000:
 * UNKNOWN). So the fall back is observable between R+1000 and R+1500, not earlier.
 * Frames: driver frames go to the VESC id, 255 (broadcast) by default: id = 0xFF | packet << 8. SET_CURRENT_REL is
 * packet 10 = 0x0AFF. Ratio = (int32)(power / 32767.0f * 1e5f), big endian (SPEC / SRC buffer.c:75-76).
 * Host-vs-target: float arithmetic is single precision on both (the driver computes in float); no result depends on
 * type sizes. lastTorque is not initialised by the driver (VescCAN.h:119); tests call turn(0) before startMotor.
 * The numeric state values (UNKNOWN 0, INCOMPATIBLE 1, PONG 2, COMPATIBLE 3, READY 4, ERROR 5, VescCAN.h:27) are
 * read through the command "vescstate".
 */
#include "doctest.h"
#include "VescFixture.h"

#include <cstdint>
#include <vector>

namespace {

using Bytes = std::vector<uint8_t>;
using FP = FakeVescPeer;

constexpr uint32_t ST_UNKNOWN = 0, ST_INCOMPATIBLE = 1, ST_PONG = 2, ST_COMPATIBLE = 3, ST_READY = 4, ST_ERROR = 5;

struct VescStates : VescFixture {
	static uint32_t nowMs() { return static_cast<uint32_t>(HostPlatform::nowUs() / 1000); }

	static uint32_t stateOf(VescCAN &d) { return static_cast<uint32_t>(CommandHelpers::get(d, "vescstate")); }

	/** Bounded wait (no assertion): runs until motorReady() or maxMs simulated. @return motorReady() */
	bool toReady(VescCAN &d, uint32_t maxMs = 20000) {
		for (uint32_t t = 0; t < maxMs && !d.motorReady(); t += 100) {
			runFor(100);
		}
		return d.motorReady();
	}

	/** SET_CURRENT_REL frames of the driver to the broadcast id */
	static std::vector<HostTarget::CanTxFrame> setFrames() { return txFramesPacket(0xFF, FP::PKT_SET_CURRENT_REL); }
	static std::vector<HostTarget::CanTxFrame> shortFrames() { return txFramesPacket(0xFF, FP::PKT_PROCESS_SHORT_BUFFER); }

	/** Time (ms) of the last frame the peer delivered to the driver */
	static uint32_t lastReplyMs(FP &p) {
		REQUIRE_FALSE(p.sent().empty());
		return p.sent().back().timeMs;
	}

	/** Runs the simulated clock up to the absolute time `ms` (bodies due at exactly `ms` run) */
	void runUntilMs(uint32_t ms) {
		REQUIRE(ms >= nowMs());
		runFor(ms - nowMs());
	}

	/** Torque power 16384: ratio 16384 / 32767 = 0.5000153 -> * 1e5 = 50001.53 -> truncated 50001 = 0x0000C351 */
	static Bytes torqueHalfBytes() { return Bytes{0x00, 0x00, 0xC3, 0x51}; }
};

} // namespace

TEST_SUITE("VESC states") {

// ---------------------------------------------------------------- V40 request cycle

TEST_CASE_FIXTURE(VescStates, "[V40] characterization: in UNKNOWN the FW version request repeats every 500 ms and no values request is sent") {
	auto v = makeVesc1();
	const uint32_t t0 = nowMs();
	peer0.setSilent(true);
	runFor(5000);
	const auto fw = shortFrames();
	REQUIRE(fw.size() == 10); // bodies at t0 + 500 ... t0 + 5000
	CHECK(fw.front().time == t0 + 500);
	for (size_t i = 1; i < fw.size(); i++) {
		const uint32_t gap = fw[i].time - fw[i - 1].time;
		CHECK(gap >= 490);
		CHECK(gap <= 510);
	}
	CHECK(peer0.requestCount(FP::COMM_FW_VERSION) == 10);
	CHECK(peer0.requestCount(FP::COMM_GET_VALUES_SELECTIVE) == 0);
	CHECK(stateOf(*v) == ST_UNKNOWN);
}

TEST_CASE_FIXTURE(VescStates, "[V40] characterization: in COMPATIBLE a values request is sent every 500 ms and the FW version is not requested again") {
	auto v = makeVesc1();
	runFor(500); // body 1: FW request, reply 7.01 -> COMPATIBLE
	REQUIRE(stateOf(*v) == ST_COMPATIBLE);
	peer0.setRespond(false); // stays COMPATIBLE
	clearTxFrames();
	const uint32_t t1 = nowMs();
	runFor(5000);
	const auto req = shortFrames();
	REQUIRE(req.size() == 10); // bodies at t1 + 500 ... t1 + 5000
	CHECK(req.front().time == t1 + 500);
	for (size_t i = 1; i < req.size(); i++) {
		const uint32_t gap = req[i].time - req[i - 1].time;
		CHECK(gap >= 490);
		CHECK(gap <= 510);
	}
	CHECK(peer0.requestCount(FP::COMM_GET_VALUES_SELECTIVE) == 10);
	CHECK(peer0.requestCount(FP::COMM_FW_VERSION) == 1);
	CHECK(stateOf(*v) == ST_COMPATIBLE);
}

// isWaiting() of the port is true only while a task is blocked inside sendMessage (CANPort2B.cpp:206-212, sendTimeout
// 20 ms), not merely because all mailboxes are pending. The "port waiting" branch of Run (VescCAN.cpp:366) therefore
// needs a second task blocked on the port: not reachable with the fixture (reported). What is observable:
TEST_CASE_FIXTURE(VescStates, "[V40] characterization: with a bus that acknowledges nothing the values request is still sent at every body (mailboxes pending is not 'port waiting')") {
	auto v = makeVesc1();
	runFor(500);
	REQUIRE(stateOf(*v) == ST_COMPATIBLE);
	peer0.setRespond(false);
	clearTxFrames();
	setTxAutoComplete(false);
	runFor(3000); // bodies at +500 ... +3000
	CHECK(shortFrames().size() == 6);
	CHECK(stateOf(*v) == ST_COMPATIBLE);
}

// ---------------------------------------------------------------- V41 liveness timeout

TEST_CASE_FIXTURE(VescStates, "[V41] characterization: READY is kept 1000 ms after the last frame and falls back to UNKNOWN at the body 1500 ms after it") {
	auto v = makeVesc1();
	reachReady(*v);
	const uint32_t r = lastReplyMs(peer0);
	peer0.setSilent(true);
	runUntilMs(r + 1000); // body at r + 1000: age 1000 is not > 1000
	CHECK(v->motorReady());
	CHECK(stateOf(*v) == ST_READY);
	runUntilMs(r + 1500); // body at r + 1500: age 1500 > 1000
	CHECK_FALSE(v->motorReady());
	CHECK(stateOf(*v) == ST_UNKNOWN);
}

TEST_CASE_FIXTURE(VescStates, "[V41] characterization: ERROR falls back to UNKNOWN at the body 1500 ms after the last frame") {
	auto v = makeVesc1();
	reachReady(*v);
	peer0.setFaultCode(3);
	runFor(500); // values reply with fault 3 -> ERROR
	REQUIRE(stateOf(*v) == ST_ERROR);
	const uint32_t r = lastReplyMs(peer0);
	peer0.setSilent(true);
	runUntilMs(r + 1000);
	CHECK(stateOf(*v) == ST_ERROR);
	runUntilMs(r + 1500);
	CHECK(stateOf(*v) == ST_UNKNOWN);
}

TEST_CASE_FIXTURE(VescStates, "[V41] characterization: after the fall back the FW version request restarts and the next reply brings the driver back to READY") {
	auto v = makeVesc1();
	reachReady(*v);
	const uint32_t r = lastReplyMs(peer0);
	peer0.setSilent(true);
	runUntilMs(r + 1500);
	REQUIRE(stateOf(*v) == ST_UNKNOWN);
	const size_t fwBefore = peer0.requestCount(FP::COMM_FW_VERSION);
	peer0.setSilent(false);
	runFor(500); // FW request, reply -> COMPATIBLE
	CHECK(peer0.requestCount(FP::COMM_FW_VERSION) == fwBefore + 1);
	CHECK(toReady(*v, 5000));
}

// ---------------------------------------------------------------- V42 unsolicited PONG

TEST_CASE_FIXTURE(VescStates, "[V42] an unsolicited PONG addressed to the driver is accepted by its filter") {
	auto v = makeVesc1();
	CHECK(peer0.sendPong() > 0); // id 0x40 | 18 << 8
}

TEST_CASE_FIXTURE(VescStates, "[V42] after an unsolicited PONG the driver keeps sending requests" * KNOWN_ISSUE("B-V8: state PONG has no exit (VescCAN.cpp:358-369, 676-678): Run sends nothing in PONG")) {
	auto v = makeVesc1();
	(void)toReady(*v);
	CHECK(peer0.sendPong() > 0); // the PONG reached the driver (passes the filters)
	clearTxFrames();
	runFor(2000);
	CHECK(shortFrames().size() >= 1); // the defect: none
}

TEST_CASE_FIXTURE(VescStates, "[V42] the driver is READY again within 5 s after an unsolicited PONG" * KNOWN_ISSUE("B-V8: state PONG has no exit (VescCAN.cpp:358-369, 676-678)")) {
	auto v = makeVesc1();
	(void)toReady(*v);
	CHECK(peer0.sendPong() > 0); // the PONG reached the driver (passes the filters)
	runFor(5000);
	CHECK(v->motorReady()); // the defect
}

// ---------------------------------------------------------------- V43 states without timeout

TEST_CASE_FIXTURE(VescStates, "[V43] characterization: INCOMPATIBLE stays INCOMPATIBLE for 30 s and sends no further request") {
	peer0.fw().major = 5; // 5.2 release < 5.3 (the driver's own rule, V31)
	peer0.fw().minor = 2;
	auto v = makeVesc1();
	runFor(500);
	REQUIRE(stateOf(*v) == ST_INCOMPATIBLE);
	clearTxFrames();
	runFor(30000);
	CHECK(stateOf(*v) == ST_INCOMPATIBLE);
	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(VescStates, "[V43] characterization: COMPATIBLE stays COMPATIBLE for 30 s without any reply") {
	auto v = makeVesc1();
	runFor(500);
	REQUIRE(stateOf(*v) == ST_COMPATIBLE);
	peer0.setRespond(false);
	runFor(30000);
	CHECK(stateOf(*v) == ST_COMPATIBLE);
}

TEST_CASE_FIXTURE(VescStates, "[V43] characterization: any frame accepted by the filters (a rotor position reply) keeps READY alive although the values replies stop") {
	auto v = makeVesc1();
	reachReady(*v);
	peer0.setRespond(false);
	for (int i = 0; i < 10; i++) { // 4 s, a frame every 400 ms (age never above 400 ms at a body)
		runFor(400);
		CHECK(peer0.sendRotorPosition() > 0);
	}
	CHECK(stateOf(*v) == ST_READY);
}

TEST_CASE_FIXTURE(VescStates, "[V43] characterization: a frame the filters reject does not keep READY alive") {
	auto v = makeVesc1();
	reachReady(*v);
	peer0.setFaultDest(0x41); // replies go to another id: rejected by the filters
	const size_t sentBefore = peer0.sent().size();
	runFor(3000);
	REQUIRE(peer0.sent().size() > sentBefore); // the peer did send replies ...
	for (size_t i = sentBefore; i < peer0.sent().size(); i++) {
		CHECK(peer0.sent()[i].accepted == 0); // ... and the filters rejected every one
	}
	CHECK(stateOf(*v) == ST_UNKNOWN);
}

// ---------------------------------------------------------------- V44 motorReady

TEST_CASE_FIXTURE(VescStates, "[V44] characterization: motorReady is true in READY") {
	auto v = makeVesc1();
	reachReady(*v);
	CHECK(stateOf(*v) == ST_READY);
	CHECK(v->motorReady());
}

TEST_CASE_FIXTURE(VescStates, "[V44] characterization: motorReady is false in UNKNOWN") {
	auto v = makeVesc1();
	CHECK(stateOf(*v) == ST_UNKNOWN);
	CHECK_FALSE(v->motorReady());
}

TEST_CASE_FIXTURE(VescStates, "[V44] characterization: motorReady is false in INCOMPATIBLE") {
	peer0.fw().major = 5;
	peer0.fw().minor = 2;
	auto v = makeVesc1();
	runFor(500);
	CHECK(stateOf(*v) == ST_INCOMPATIBLE);
	CHECK_FALSE(v->motorReady());
}

TEST_CASE_FIXTURE(VescStates, "[V44] characterization: motorReady is false in COMPATIBLE") {
	auto v = makeVesc1();
	runFor(500);
	CHECK(stateOf(*v) == ST_COMPATIBLE);
	CHECK_FALSE(v->motorReady());
}

TEST_CASE_FIXTURE(VescStates, "[V44] characterization: motorReady is false in ERROR") {
	peer0.setFaultCode(3);
	auto v = makeVesc1();
	runFor(1000); // FW reply, values reply with fault 3
	CHECK(stateOf(*v) == ST_ERROR);
	CHECK_FALSE(v->motorReady());
}

TEST_CASE_FIXTURE(VescStates, "[V44] characterization: motorReady is false in PONG") {
	auto v = makeVesc1();
	REQUIRE(peer0.sendPong() > 0);
	CHECK(stateOf(*v) == ST_PONG);
	CHECK_FALSE(v->motorReady());
}

// ---------------------------------------------------------------- V50 turn sends only when READY and started

TEST_CASE_FIXTURE(VescStates, "[V50] characterization: turn in READY and started sends one SET_CURRENT_REL frame, id 0x0AFF, DLC 4, 00 00 C3 51") {
	auto v = makeVesc1();
	reachReady(*v);
	v->turn(0);
	v->startMotor();
	clearTxFrames();
	v->turn(16384);
	const auto f = setFrames();
	REQUIRE(f.size() == 1);
	CHECK(f[0].extId);
	CHECK_FALSE(f[0].rtr);
	CHECK(f[0].id == 0x0AFF);
	CHECK(f[0].dlc == 4);
	CHECK(bytes(f[0]) == torqueHalfBytes());
	process();
	CHECK(peer0.lastCurrentRaw() == 50001);
}

TEST_CASE_FIXTURE(VescStates, "[V50] characterization: turn in READY without startMotor sends nothing") {
	auto v = makeVesc1();
	reachReady(*v);
	clearTxFrames();
	v->turn(16384);
	CHECK(setFrames().empty());
}

TEST_CASE_FIXTURE(VescStates, "[V50] characterization: turn after startMotor in UNKNOWN sends nothing, also not from the thread") {
	auto v = makeVesc1();
	peer0.setSilent(true);
	v->startMotor();
	v->turn(16384);
	runFor(3000);
	CHECK(setFrames().empty());
	CHECK(stateOf(*v) == ST_UNKNOWN);
}

TEST_CASE_FIXTURE(VescStates, "[V50] characterization: turn after startMotor in COMPATIBLE sends nothing, also not from the thread") {
	auto v = makeVesc1();
	runFor(500);
	REQUIRE(stateOf(*v) == ST_COMPATIBLE);
	peer0.setRespond(false);
	v->startMotor();
	v->turn(16384);
	runFor(3000);
	CHECK(setFrames().empty());
}

TEST_CASE_FIXTURE(VescStates, "[V50] characterization: turn after startMotor in INCOMPATIBLE sends nothing") {
	peer0.fw().major = 5;
	peer0.fw().minor = 2;
	auto v = makeVesc1();
	runFor(500);
	REQUIRE(stateOf(*v) == ST_INCOMPATIBLE);
	v->startMotor();
	v->turn(16384);
	runFor(3000);
	CHECK(setFrames().empty());
}

TEST_CASE_FIXTURE(VescStates, "[V50] characterization: turn after startMotor in ERROR sends nothing, also not from the thread") {
	peer0.setFaultCode(3);
	auto v = makeVesc1();
	runFor(1000);
	REQUIRE(stateOf(*v) == ST_ERROR);
	v->startMotor();
	v->turn(16384);
	runFor(3000);
	CHECK(setFrames().empty());
}

// ---------------------------------------------------------------- V51 repeat

TEST_CASE_FIXTURE(VescStates, "[V51] characterization: while running the last torque is repeated so that the gap between SET frames is at most 1000 ms and the VESC never times out") {
	auto v = makeVesc1();
	reachReady(*v);
	v->turn(0);
	v->startMotor();
	clearTxFrames();
	v->turn(16384);
	const uint32_t t0 = nowMs();
	for (int i = 0; i < 50; i++) { // 5 s in 100 ms steps
		runFor(100);
		CHECK_FALSE(peer0.timedOut()); // VESC default timeout 1000 ms (SRC appconf_default.h:27-31)
	}
	const auto f = setFrames();
	// first frame by turn at t0, then one per body: at most 500 ms apart, so 10 bodies in 5000 ms -> 11 frames (10..11)
	REQUIRE(f.size() >= 10);
	CHECK(f.size() <= 11);
	CHECK(f.front().time == t0);
	for (size_t i = 0; i < f.size(); i++) {
		CHECK(bytes(f[i]) == torqueHalfBytes());
		if (i > 0) {
			const uint32_t gap = f[i].time - f[i - 1].time;
			CHECK(gap > 0);
			CHECK(gap <= 1000);
		}
	}
	CHECK(peer0.lastCurrentRaw() == 50001);
	CHECK(peer0.effectiveRatio() > 0.4f);
}

TEST_CASE_FIXTURE(VescStates, "[V51] characterization: turn(0) sends a zero ratio once and the repeats stop") {
	auto v = makeVesc1();
	reachReady(*v);
	v->turn(0);
	v->startMotor();
	v->turn(16384);
	runFor(1000);
	clearTxFrames();
	v->turn(0);
	runFor(3000);
	const auto f = setFrames();
	REQUIRE(f.size() == 1);
	CHECK(bytes(f[0]) == Bytes({0, 0, 0, 0}));
}

TEST_CASE_FIXTURE(VescStates, "[V51] characterization: the torque is not repeated while the driver is not started") {
	auto v = makeVesc1();
	reachReady(*v);
	v->turn(16384); // not started: nothing sent, lastTorque is 0.5
	runFor(3000);
	CHECK(setFrames().empty());
}

// ---------------------------------------------------------------- V52 stop

TEST_CASE_FIXTURE(VescStates, "[V52] characterization: stopMotor sends one SET_CURRENT_REL with ratio 0 (00 00 00 00)") {
	auto v = makeVesc1();
	reachReady(*v);
	v->turn(0);
	v->startMotor();
	v->turn(16384);
	clearTxFrames();
	v->stopMotor();
	const auto f = setFrames();
	REQUIRE(f.size() == 1);
	CHECK(f[0].id == 0x0AFF);
	CHECK(f[0].dlc == 4);
	CHECK(bytes(f[0]) == Bytes({0, 0, 0, 0}));
	process();
	CHECK(peer0.lastCurrentRaw() == 0);
}

TEST_CASE_FIXTURE(VescStates, "[V52] characterization: after stopMotor no SET_CURRENT_REL is sent by the thread while stopped") {
	auto v = makeVesc1();
	reachReady(*v);
	v->turn(0);
	v->startMotor();
	v->turn(16384);
	v->stopMotor();
	clearTxFrames();
	runFor(3000);
	CHECK(setFrames().empty());
}

TEST_CASE_FIXTURE(VescStates, "[V52] characterization: stopMotor when not started sends nothing") {
	auto v = makeVesc1();
	reachReady(*v);
	clearTxFrames();
	v->stopMotor();
	CHECK(setFrames().empty());
}

TEST_CASE_FIXTURE(VescStates, "[V52] after a stop and a new start no old torque is sent until turn is called again" * KNOWN_ISSUE("B-V9: stopMotor does not reset lastTorque (VescCAN.cpp:132-139), so the first body after startMotor repeats the old torque (VescCAN.cpp:380-383)")) {
	auto v = makeVesc1();
	(void)toReady(*v);
	v->turn(0);
	v->startMotor();
	v->turn(16384);
	v->stopMotor();
	v->startMotor();
	clearTxFrames();
	runFor(2000);
	CHECK(setFrames().empty()); // the defect: 0x0AFF with 00 00 C3 51 at the next body
}

// ---------------------------------------------------------------- V53 no clamp

TEST_CASE_FIXTURE(VescStates, "[V53] characterization: power -32768 gives ratio -1.00003 (not clamped): FF FE 79 5D") {
	auto v = makeVesc1();
	reachReady(*v);
	v->turn(0);
	v->startMotor();
	clearTxFrames();
	v->turn(-32768);
	// -32768 / 32767 = -1.0000305; * 1e5 = -100003.05; truncated -100003 = 2^32 - 0x186A3 = 0xFFFE795D
	const auto f = setFrames();
	REQUIRE(f.size() == 1);
	CHECK(bytes(f[0]) == Bytes({0xFF, 0xFE, 0x79, 0x5D}));
	process();
	CHECK(peer0.lastCurrentRaw() == -100003);
}

}
