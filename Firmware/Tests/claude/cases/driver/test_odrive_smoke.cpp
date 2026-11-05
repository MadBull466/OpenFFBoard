/*
 * Smoke tests of the ODrive driver test infrastructure (fixture + FakeOdrivePeer + driver link together).
 * Behaviour tests of the driver are written per requirement group (Tests/claude/charters/driver/ODRIVE_REQUIREMENTS.md).
 */
#include "doctest.h"
#include "OdriveFixture.h"

TEST_SUITE("ODrive smoke") {

TEST_CASE_FIXTURE(OdriveFixture, "[O1] constructing ODriveCAN1 starts the CAN port and registers one standard-id filter for node 0 with the node bits as mask") {
	auto od = makeOdrive1();
	CHECK(HostTarget::canPort().started);
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x000); // node 0 << 5
	CHECK(HostTarget::canPort().filters[0].filter_mask == 0x7E0);
	CHECK_FALSE(HostTarget::canPort().filters[0].extid);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O32] characterization: after the first heartbeat reports IDLE the first frame of the driver is a well-formed standard-id Set Axis Requested State of node 0 for state number 3") {
	auto od = makeOdrive1();
	runFor(499);
	CHECK(HostTarget::canTxFrames().empty()); // the thread has not run its first body yet (Delay(500) first)
	runFor(1);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	const auto &f = HostTarget::canTxFrames()[0];
	CHECK(f.id == 0x007); // node 0 << 5 | 0x07 (CSV 0x007)
	CHECK_FALSE(f.extId);
	CHECK_FALSE(f.rtr);
	CHECK(f.dlc == 4);
	// characterization: the number 3 is the driver's FULL_CALIBRATION_SEQUENCE (enum numbers are not in the spec, L2)
	CHECK(f.value() == 3);
	CHECK(peer0.violations().empty());
	CHECK(peer0.requestedStates() == std::vector<uint32_t>({3}));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O32] the driver thread runs its body at 500 ms, 1000 ms, ... of simulated time (fixture scheduling)") {
	auto od = makeOdrive1();
	peer0.setSilent(true); // no heartbeat, no reply: the only frames are the thread's voltage requests
	runFor(3000);
	// ODriveCAN::Run requests 0x17 when now - lastVoltageUpdate > 1000: the bodies at 1500, 2000, 2500 and 3000 ms
	// (characterization: the 1000 ms threshold and the 500 ms loop are firmware behaviour)
	const auto frames = txFramesCmd(0, 0x17);
	REQUIRE(frames.size() == 4);
	for (size_t i = 0; i < 4; i++) {
		CHECK(frames[i].time == 1500 + 500 * i);
		CHECK(frames[i].rtr);
		CHECK(frames[i].dlc == 0);
	}
	CHECK(peer0.violations().empty());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O32] reachRunning brings ODriveCAN1 against the peer to motorReady with the control modes sent") {
	auto od = makeOdrive1();
	reachRunning(*od);
	CHECK(od->motorReady());
	CHECK(peer0.state() == FakeOdrivePeer::STATE_CLOSED_LOOP);
	CHECK(peer0.requestedStates() == std::vector<uint32_t>({3, 8}));
	CHECK(peer0.modesSeen());
	CHECK(peer0.violations().empty());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O3] ODriveCAN2 receives the node 1 heartbeat on FIFO 1 and reaches motorReady against peer 1") {
	auto od = makeOdrive2();
	reachRunning(*od);
	CHECK(od->motorReady());
	REQUIRE(HostTarget::canTxFrames().size() > 0);
	CHECK(HostTarget::canTxFrames()[0].id == ((1u << 5) | 0x07));
	CHECK(peer1.violations().empty());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O3] characterization: with the real F407 interrupt set-up ODriveCAN2 sees no heartbeat before one of its own frames completes") {
	setRx1Interrupt(false);
	auto od = makeOdrive2();
	runFor(450); // heartbeats at 100 .. 400 wait in FIFO 1: no interrupt, and the driver has sent nothing to complete
	CHECK(HostTarget::canRxPending(1) > 0);
	CHECK(HostTarget::canTxFrames().empty());
}

}
