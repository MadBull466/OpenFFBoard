/*
 * ODrive CAN driver: G4 connection, state machine, timing (O30-O39) and G8 errors, robustness (O70, O71).
 * Requirements: Firmware/Tests/driver/ODRIVE_REQUIREMENTS.md (lead decisions L1-L11).
 *
 * Time base (code path of ODriveCAN::Run): the thread body runs at 500 ms, 1000 ms, ... after construction (the fixture
 * schedules it). The node timeout is "HAL_GetTick() - lastCanMessage > 2000", evaluated in the body. The peer's heartbeat
 * comes every 100 ms (SPEC "Cyclic Messages"), first one at 100 ms. The peer's calibration takes 2000 ms (L10, assumption).
 * The state numbers (1 IDLE, 3 full calibration, 8 closed loop, ...) and the 2000 ms timeout are firmware behaviour, not
 * in SPEC / CSV (L2): tests that depend on them are named "characterization:". The frame encoding around them (id, DLC,
 * little endian payload) is asserted against CSV 0x007 / 0x00B / 0x00E / 0x009 / 0x017.
 * Host-vs-target: none of these tests depends on type sizes; the driver reads the frame data through a uint64_t cast
 * (ODriveCAN.cpp:394, aliasing, little endian host and target).
 */
#include "doctest.h"
#include "OdriveFixture.h"

#include "ErrorHandler.h"

#include <cstdint>
#include <vector>

namespace {

using Bytes = std::vector<uint8_t>;
using FP = FakeOdrivePeer;

const Bytes STATE_IDLE_BYTES{1, 0, 0, 0};
const Bytes STATE_FULL_CAL_BYTES{3, 0, 0, 0};
const Bytes STATE_CLOSED_LOOP_BYTES{8, 0, 0, 0};
const Bytes MODES_TORQUE_PASSTHROUGH{1, 0, 0, 0, 1, 0, 0, 0}; // CSV 0x00B: int32 control mode @0, int32 input mode @4
const Bytes TORQUE_ZERO{0, 0, 0, 0};						   // CSV 0x00E: float32 0.0f

bool isConnected(CommandHandler &d) { return CommandHelpers::get(d, "connected") == 1; }

/** Exact frame: standard id, data frame, DLC, payload */
void checkDataFrame(const HostTarget::CanTxFrame &f, uint32_t id, const Bytes &payload) {
	CHECK(f.id == id);
	CHECK_FALSE(f.extId);
	CHECK_FALSE(f.rtr);
	CHECK(f.dlc == payload.size());
	CHECK(f.payload() == payload);
}

/** Runs until motorReady() or maxMs of simulated time. @return motorReady() */
bool runUntilReady(OdriveFixture &fx, ODriveCAN &d, uint32_t maxMs) {
	for (uint32_t t = 0; t < maxMs && !d.motorReady(); t += 100) {
		fx.runFor(100);
	}
	return d.motorReady();
}

/** (cmd, first payload byte) of the state / mode frames the driver sent to node 0, in order */
std::vector<std::pair<uint32_t, uint32_t>> stateAndModeFrames() {
	std::vector<std::pair<uint32_t, uint32_t>> r;
	for (const auto &f : HostTarget::canTxFrames()) {
		if (f.id == 0x07 || f.id == 0x0B) {
			r.emplace_back(f.id, f.data[0]);
		}
	}
	return r;
}

} // namespace

TEST_SUITE("ODrive states") {

// ---------------------------------------------------------------- O30 connection timeout

TEST_CASE_FIXTURE(OdriveFixture, "[O30] connected is false after 3 s without a frame and true again in the next loop after a frame of the node") {
	auto od = makeOdrive1();
	peer0.setSilent(true);
	runFor(3000); // bodies up to 3000 ms: 3000 - 0 > 2000
	REQUIRE_FALSE(isConnected(*od));
	peer0.setSilent(false);
	peer0.sendHeartbeat(); // frame at 3000 ms
	runFor(500);		   // body at 3500 ms: 3500 - 3000 = 500 <= 2000
	CHECK(isConnected(*od));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O30] characterization: connected stays true while the last frame is younger than 2000 ms") {
	auto od = makeOdrive1();
	reachRunning(*od);
	peer0.setSilent(true);
	// last heartbeat t_last in (t0 - 100, t0]; at t0 + 1850 every body sees an age below 1850 + 100 = 1950 ms <= 2000
	runFor(1850);
	CHECK(isConnected(*od));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O30] characterization: connected is false when no frame came for more than 2000 ms") {
	auto od = makeOdrive1();
	reachRunning(*od);
	peer0.setSilent(true);
	// the last body in (t0 + 2000, t0 + 2500] sees an age of more than 2000 ms
	runFor(2500);
	CHECK_FALSE(isConnected(*od));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O30] characterization: after the timeout motorReady is false and the reported ODrive state is undefined (0)") {
	auto od = makeOdrive1();
	reachRunning(*od);
	peer0.setSilent(true);
	runFor(2500);
	CHECK_FALSE(od->motorReady());
	CHECK(CommandHelpers::get(*od, "state") == 0); // ODriveState::AXIS_STATE_UNDEFINED
}

// ---------------------------------------------------------------- O31 no frame, no connection

TEST_CASE_FIXTURE(OdriveFixture, "[O31] connected is false while no frame of the node was ever received" * KNOWN_ISSUE("B-O2: lastCanMessage starts at 0, so the timeout test (tick - 0 > 2000) reports connected for the first 2 s")) {
	auto od = makeOdrive1();
	peer0.setSilent(true);
	runFor(1000); // bodies at 500 and 1000 ms
	CHECK_FALSE(isConnected(*od));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O31] no encoder estimates are requested while no frame of the node was ever received" * KNOWN_ISSUE("B-O2: lastCanMessage starts at 0, so connected is true for the first 2 s and getPos_f() sends RTR 0x09")) {
	auto od = makeOdrive1();
	peer0.setSilent(true);
	runFor(1000);
	(void)od->getPos_f();
	CHECK(txFramesCmd(0, 0x09).empty());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O31] characterization: after 3 s without any frame getPos_f requests no encoder estimates") {
	auto od = makeOdrive1();
	peer0.setSilent(true);
	runFor(3000);
	(void)od->getPos_f();
	CHECK(txFramesCmd(0, 0x09).empty());
}

// ---------------------------------------------------------------- O32 start-up sequence

TEST_CASE_FIXTURE(OdriveFixture, "[O32] characterization: ODrive reports IDLE: the next loop requests the full calibration sequence (0x07 = 3) and nothing else") {
	auto od = makeOdrive1();
	runFor(600); // heartbeat at 100 ms -> WAIT_READY, body at 500 ms -> request
	const auto st = txFramesCmd(0, 0x07);
	REQUIRE(st.size() == 1);
	checkDataFrame(st[0], 0x07, STATE_FULL_CAL_BYTES);
	CHECK(st[0].time >= 500);
	CHECK(st[0].time <= 600);
	CHECK(txFramesCmd(0, 0x0B).empty());
	CHECK(peer0.requestedStates() == std::vector<uint32_t>({3}));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O32] characterization: no state request is sent before the first heartbeat") {
	auto od = makeOdrive1();
	peer0.setHeartbeat(false);
	runFor(2000);
	CHECK(txFramesCmd(0, 0x07).empty());
	CHECK(txFramesCmd(0, 0x0B).empty());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O32] characterization: while the ODrive is in a calibration state (3, 4, 6, 7, 13) the driver requests nothing and is not ready") {
	uint32_t st = 0;
	SUBCASE("full calibration (3)") { st = 3; }
	SUBCASE("motor calibration (4)") { st = 4; }
	SUBCASE("encoder index search (6)") { st = 6; }
	SUBCASE("encoder offset calibration (7)") { st = 7; }
	SUBCASE("encoder hall phase calibration (13)") { st = 13; }
	CAPTURE(st);
	peer0.setState(st); // stays: setState starts no calibration timer
	auto od = makeOdrive1();
	runFor(6000);
	CHECK(txFramesCmd(0, 0x07).empty());
	CHECK(txFramesCmd(0, 0x0B).empty());
	CHECK_FALSE(od->motorReady());
	// the calibration ends: IDLE heartbeat -> exactly one request for closed loop (0x07 = 8)
	heartbeat(peer0, FP::STATE_IDLE);
	runFor(1000);
	const auto fr = txFramesCmd(0, 0x07);
	REQUIRE(fr.size() == 1);
	checkDataFrame(fr[0], 0x07, STATE_CLOSED_LOOP_BYTES);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O32] characterization: from a calibration state the driver goes to RUNNING with Set Controller Modes when the ODrive reports closed loop directly") {
	uint32_t st = 0;
	SUBCASE("full calibration (3)") { st = 3; }
	SUBCASE("motor calibration (4)") { st = 4; }
	SUBCASE("encoder index search (6)") { st = 6; }
	SUBCASE("encoder offset calibration (7)") { st = 7; }
	SUBCASE("encoder hall phase calibration (13)") { st = 13; }
	CAPTURE(st);
	peer0.setState(st);
	auto od = makeOdrive1();
	runFor(1000); // WAIT_CALIBRATION_DONE
	heartbeat(peer0, FP::STATE_CLOSED_LOOP);
	runFor(1500); // body: START_RUNNING, next body: RUNNING and modes
	CHECK(peer0.modesSeen());
	CHECK(od->motorReady());
	const auto md = txFramesCmd(0, 0x0B);
	REQUIRE(md.size() == 1);
	checkDataFrame(md[0], 0x0B, MODES_TORQUE_PASSTHROUGH);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O32] characterization: when the calibration ends in IDLE the driver requests closed loop (0x07 = 8)") {
	auto od = makeOdrive1();
	reachRunning(*od);
	const auto st = txFramesCmd(0, 0x07);
	REQUIRE(st.size() == 2);
	checkDataFrame(st[1], 0x07, STATE_CLOSED_LOOP_BYTES);
	// calibration request at >= 500 ms, 2000 ms calibration (L10), body after the IDLE heartbeat: 3000 ms
	CHECK(st[1].time >= 2500);
	CHECK(st[1].time <= 3500);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O32] characterization: when the ODrive reports closed loop the driver sends the controller modes torque / passthrough once") {
	auto od = makeOdrive1();
	reachRunning(*od);
	runFor(2000);
	const auto md = txFramesCmd(0, 0x0B);
	REQUIRE(md.size() == 1);
	checkDataFrame(md[0], 0x0B, MODES_TORQUE_PASSTHROUGH); // values 1 / 1: driver enum numbers (L2)
	CHECK(od->motorReady());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O32] characterization: the start-up sends exactly 0x07 = 3, 0x07 = 8, 0x0B modes in this order") {
	auto od = makeOdrive1();
	reachRunning(*od);
	runFor(2000);
	const std::vector<std::pair<uint32_t, uint32_t>> expected{{0x07, 3}, {0x07, 8}, {0x0B, 1}};
	CHECK(stateAndModeFrames() == expected);
	CHECK(peer0.violations().empty());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O32] characterization: entering RUNNING zeroes the position (getPos_f is 0 although the ODrive reports 0.25 turns)") {
	peer0.setEncoder(0.25f, 0.0f);
	peer0.setEncoderCyclic(true, 10);
	auto od = makeOdrive1();
	reachRunning(*od);
	// setPos(0): posOffset = lastPos - 0 -> lastPos - posOffset = 0 exactly
	CHECK(od->getPos_f() == 0.0f);
	CHECK(od->getPos() == 0);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O32] characterization: with storepos on, entering RUNNING keeps the reported position (0.25 turns)") {
	peer0.setEncoder(0.25f, 0.0f);
	peer0.setEncoderCyclic(true, 10);
	auto od = makeOdrive1();
	REQUIRE(CommandHelpers::run(*od, "storepos", CMDtype::set, 1) == CommandStatus::OK);
	reachRunning(*od);
	CHECK(od->getPos_f() == 0.25f); // posOffset stays 0 (nothing stored in flash)
}

// ---------------------------------------------------------------- O33 modes before RUNNING is used

TEST_CASE_FIXTURE(OdriveFixture, "[O33] the controller modes are sent when the ODrive is already in closed loop at connect" * KNOWN_ISSUE("B-O6: the T7 path (ODriveCAN.cpp:196-199) sets RUNNING without setMode(), motorReady() is true but the ODrive never got torque / passthrough")) {
	peer0.setState(FP::STATE_CLOSED_LOOP);
	auto od = makeOdrive1();
	runFor(2000);
	CHECK(peer0.modesSeen()); // the defect
	CHECK(od->motorReady());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O33] the controller modes are sent when the closed-loop heartbeat arrives after START_RUNNING" * KNOWN_ISSUE("B-O6: START_RUNNING sets RUNNING before it checks the ODrive state; the ODrive was still IDLE, so setMode() is skipped and the T7 path does not run in RUNNING")) {
	peer0.setRequestApplyDelayMs(1000); // the ODrive needs two loops to enter closed loop
	auto od = makeOdrive1();
	const bool ready = runUntilReady(*this, *od, 20000);
	runFor(1500);
	CHECK(peer0.modesSeen()); // the defect
	CHECK(ready);
	CHECK(peer0.state() == FP::STATE_CLOSED_LOOP);
}

// ---------------------------------------------------------------- O34 refused calibration

TEST_CASE_FIXTURE(OdriveFixture, "[O34] characterization: a refused calibration request is not repeated and the driver never becomes ready (no retry)") {
	peer0.setRefuseRequests(true);
	auto od = makeOdrive1();
	runFor(10000);
	CHECK(peer0.requestedStates() == std::vector<uint32_t>({3}));
	CHECK(peer0.refusedStates() == std::vector<uint32_t>({3}));
	CHECK(txFramesCmd(0, 0x07).size() == 1);
	CHECK_FALSE(od->motorReady());
	CHECK(isConnected(*od)); // the heartbeats continue
}

// ---------------------------------------------------------------- O35 motorReady

TEST_CASE_FIXTURE(OdriveFixture, "[O35] motorReady is false before any frame of the ODrive") {
	auto od = makeOdrive1();
	peer0.setHeartbeat(false);
	runFor(1000);
	CHECK_FALSE(od->motorReady());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O35] characterization: motorReady is false in closed loop until the loop has run once (local state WAIT_READY)") {
	peer0.setState(FP::STATE_CLOSED_LOOP);
	auto od = makeOdrive1();
	runFor(450); // heartbeats at 100 .. 400 ms, first body at 500 ms
	CHECK_FALSE(od->motorReady());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O35] motorReady is true in RUNNING with the ODrive in closed loop") {
	auto od = makeOdrive1();
	reachRunning(*od);
	CHECK(od->motorReady());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O35] characterization: motorReady is false when the ODrive leaves closed loop (IDLE) while the driver is RUNNING") {
	auto od = makeOdrive1();
	reachRunning(*od);
	heartbeat(peer0, FP::STATE_IDLE);
	runFor(100);
	CHECK_FALSE(od->motorReady());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O35] characterization: motorReady is false while the ODrive calibrates") {
	auto od = makeOdrive1();
	runFor(1500); // calibration requested at 500 ms, runs until 2500 ms
	REQUIRE(peer0.state() == FP::STATE_FULL_CALIBRATION);
	CHECK_FALSE(od->motorReady());
}

// ---------------------------------------------------------------- O36 start / stop / emergency stop

TEST_CASE_FIXTURE(OdriveFixture, "[O36] characterization: startMotor with the ODrive in IDLE sends 0x07 = 8 (closed loop)") {
	auto od = makeOdrive1();
	runFor(150); // heartbeat at 100 ms: IDLE
	clearTxFrames();
	od->startMotor();
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkDataFrame(HostTarget::canTxFrames()[0], 0x07, STATE_CLOSED_LOOP_BYTES);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O36] characterization: startMotor sends nothing when the ODrive is not in IDLE") {
	peer0.setState(FP::STATE_CLOSED_LOOP);
	auto od = makeOdrive1();
	runFor(150);
	clearTxFrames();
	od->startMotor();
	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O36] characterization: stopMotor in closed loop sends torque 0 and then requests IDLE (0x07 = 1)") {
	auto od = makeOdrive1();
	reachRunning(*od);
	clearTxFrames();
	od->stopMotor();
	REQUIRE(HostTarget::canTxFrames().size() == 2);
	checkDataFrame(HostTarget::canTxFrames()[0], 0x0E, TORQUE_ZERO);
	checkDataFrame(HostTarget::canTxFrames()[1], 0x07, STATE_IDLE_BYTES);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O36] characterization: stopMotor with the ODrive in IDLE sends torque 0 only") {
	auto od = makeOdrive1();
	runFor(150);
	clearTxFrames();
	od->stopMotor();
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkDataFrame(HostTarget::canTxFrames()[0], 0x0E, TORQUE_ZERO);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O36] characterization: emergencyStop(false) does what stopMotor does") {
	auto od = makeOdrive1();
	reachRunning(*od);
	clearTxFrames();
	od->emergencyStop(false);
	REQUIRE(HostTarget::canTxFrames().size() == 2);
	checkDataFrame(HostTarget::canTxFrames()[0], 0x0E, TORQUE_ZERO);
	checkDataFrame(HostTarget::canTxFrames()[1], 0x07, STATE_IDLE_BYTES);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O36] characterization: emergencyStop(true) does what startMotor does") {
	auto od = makeOdrive1();
	runFor(150);
	clearTxFrames();
	od->emergencyStop(true);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkDataFrame(HostTarget::canTxFrames()[0], 0x07, STATE_CLOSED_LOOP_BYTES);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O36] characterization: stopMotor then startMotor returns the driver to ready without a new calibration or modes frame") {
	auto od = makeOdrive1();
	reachRunning(*od);
	od->stopMotor();
	runFor(200); // IDLE heartbeat
	CHECK_FALSE(od->motorReady());
	od->startMotor();
	runFor(200); // closed loop heartbeat
	CHECK(od->motorReady());
	CHECK(peer0.requestedStates() == std::vector<uint32_t>({3, 8, 1, 8}));
	CHECK(txFramesCmd(0, 0x0B).size() == 1);
}

// ---------------------------------------------------------------- O37 automatic calibration

TEST_CASE_FIXTURE(OdriveFixture, "[O37] characterization: the ODrive is calibrated and put in closed loop although startMotor was never called") {
	auto od = makeOdrive1();
	reachRunning(*od);
	CHECK(peer0.requestedStates() == std::vector<uint32_t>({3, 8}));
}

// ---------------------------------------------------------------- O38 bus voltage request

TEST_CASE_FIXTURE(OdriveFixture, "[O38] characterization: without a voltage reply the driver sends RTR 0x17 (DLC 0) in every loop from 1500 ms on") {
	auto od = makeOdrive1();
	peer0.setRespond(false); // heartbeats continue, requests are not answered
	runFor(3000);
	// lastVoltageUpdate = 0: body at 1000 ms: 1000 > 1000 false; 1500, 2000, 2500, 3000 ms: request
	const auto fr = txFramesCmd(0, 0x17);
	REQUIRE(fr.size() == 4);
	for (size_t i = 0; i < fr.size(); i++) {
		CHECK(fr[i].time == 1500 + 500 * i);
		CHECK(fr[i].rtr);
		CHECK_FALSE(fr[i].extId);
		CHECK(fr[i].dlc == 0);
	}
}

TEST_CASE_FIXTURE(OdriveFixture, "[O38] characterization: with immediate replies the driver requests 0x17 again when the reply is older than 1000 ms (1500, 3000, 4500 ms)") {
	auto od = makeOdrive1();
	runFor(5000);
	// reply at 1500 ms -> lastVoltageUpdate = 1500; 2500 ms: 1000 > 1000 false; 3000 ms: 1500 > 1000 -> request; 4500 ms again
	const auto fr = txFramesCmd(0, 0x17);
	REQUIRE(fr.size() == 3);
	CHECK(fr[0].time == 1500);
	CHECK(fr[1].time == 3000);
	CHECK(fr[2].time == 4500);
}

// ---------------------------------------------------------------- O39 loss and reconnect

TEST_CASE_FIXTURE(OdriveFixture, "[O39] characterization: after a loss and a new IDLE heartbeat the driver requests the calibration again and reaches closed loop") {
	auto od = makeOdrive1();
	reachRunning(*od);
	peer0.setSilent(true);
	peer0.setState(FP::STATE_IDLE); // the ODrive rebooted
	runFor(3000);
	REQUIRE_FALSE(isConnected(*od));
	peer0.setSilent(false);
	peer0.sendHeartbeat();
	REQUIRE(runUntilReady(*this, *od, 20000));
	CHECK(peer0.requestedStates() == std::vector<uint32_t>({3, 8, 3, 8}));
	CHECK(isConnected(*od));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O39] characterization: after a loss and a heartbeat with the ODrive still in closed loop the driver is ready again without a new request") {
	auto od = makeOdrive1();
	reachRunning(*od);
	peer0.setSilent(true);
	runFor(3000);
	REQUIRE_FALSE(od->motorReady());
	peer0.setSilent(false);
	peer0.sendHeartbeat();
	runFor(1000);
	CHECK(od->motorReady());
	CHECK(peer0.requestedStates() == std::vector<uint32_t>({3, 8}));
}

// ---------------------------------------------------------------- O70 errors not raised

TEST_CASE_FIXTURE(OdriveFixture, "[O70] characterization: an axis error in the heartbeat is readable with the errors command but not added to the firmware error handler") {
	auto od = makeOdrive1();
	runFor(150);
	const size_t before = ErrorHandler::getErrors().size();
	heartbeat(peer0, FP::STATE_IDLE, 0x4001);
	CHECK(CommandHelpers::get(*od, "errors") == 0x4001);
	CHECK(ErrorHandler::getErrors().size() == before);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O70] characterization: CAN bus error states are ignored by the driver (no error raised, still connected)") {
	auto od = makeOdrive1();
	reachRunning(*od);
	const size_t before = ErrorHandler::getErrors().size();
	HostTarget::canSetErrorState(HostTarget::CanErrorState::Warning);
	HostTarget::canSetErrorState(HostTarget::CanErrorState::Passive);
	HostTarget::canSetLastErrorCode(3); // acknowledge error
	runFor(2000);
	CHECK(ErrorHandler::getErrors().size() == before);
	CHECK(isConnected(*od));
	CHECK(od->motorReady());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O70] characterization: after an axis error that drops the ODrive to IDLE the driver sends no state, mode or torque frame") {
	auto od = makeOdrive1();
	reachRunning(*od);
	clearTxFrames();
	heartbeat(peer0, FP::STATE_IDLE, 0x1);
	runFor(10000);
	CHECK(txFramesCmd(0, 0x07).empty());
	CHECK(txFramesCmd(0, 0x0B).empty());
	CHECK(txFramesCmd(0, 0x0E).empty());
	CHECK_FALSE(od->motorReady());
	CHECK(isConnected(*od));
}

// ---------------------------------------------------------------- O71 bus-off, silent node, TX failures

TEST_CASE_FIXTURE(OdriveFixture, "[O71] characterization: a silent node is detected through the timeout while the driver keeps requesting the bus voltage") {
	auto od = makeOdrive1();
	peer0.setSilent(true);
	runFor(10000);
	CHECK_FALSE(isConnected(*od));
	// no reply: a request in every body from 1500 ms to 10000 ms = (10000 - 1500) / 500 + 1
	CHECK(txFramesCmd(0, 0x17).size() == 18);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O71] characterization: after bus-off no frame reaches the driver and the loss is detected through the timeout") {
	auto od = makeOdrive1();
	reachRunning(*od);
	HostTarget::canSetErrorState(HostTarget::CanErrorState::BusOff);
	runFor(3000);
	CHECK_FALSE(isConnected(*od));
	CHECK_FALSE(od->motorReady());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O71] characterization: with TX mailboxes that never complete the driver thread keeps running and still detects a node loss") {
	setTxAutoComplete(false);
	auto od = makeOdrive1();
	runFor(5000); // 0x07 and 0x17 requests fill the three mailboxes; the heartbeats still arrive
	CHECK(isConnected(*od));
	peer0.setSilent(true);
	runFor(3000);
	CHECK_FALSE(isConnected(*od)); // the thread was not blocked
}

}
