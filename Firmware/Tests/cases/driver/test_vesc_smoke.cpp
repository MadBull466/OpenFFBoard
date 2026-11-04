/*
 * Smoke tests of the VESC driver test infrastructure (fixture + FakeVescPeer + driver link together).
 * Behaviour tests of the driver are written per requirement group (Firmware/Tests/driver/VESC_REQUIREMENTS.md).
 */
#include "doctest.h"
#include "VescFixture.h"

TEST_SUITE("VESC smoke") {

TEST_CASE_FIXTURE(VescFixture, "[V1] constructing VESC_1 starts the CAN port and registers two extended-id filters") {
	auto v = makeVesc1();
	CHECK(HostTarget::canPort().started);
	REQUIRE(HostTarget::canPort().filters.size() == 2);
	CHECK(HostTarget::canPort().filters[0].extid);
	CHECK(HostTarget::canPort().filters[1].extid);
}

TEST_CASE_FIXTURE(VescFixture, "[V12] the first frame of the driver comes at 500 ms and is the FW version request: PROCESS_SHORT_BUFFER to VESC id 255, DLC 3, [0x40, 0x00, 0x00]") {
	auto v = makeVesc1();
	runFor(499);
	CHECK(HostTarget::canTxFrames().empty()); // the thread has not run its first body yet (Delay(500) first)
	runFor(1);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	const auto &f = HostTarget::canTxFrames()[0];
	CHECK(f.extId);
	CHECK_FALSE(f.rtr);
	CHECK(f.id == 0x08FF); // VESC id 255 (default, broadcast) | PROCESS_SHORT_BUFFER (8) << 8
	CHECK(f.dlc == 3);
	CHECK(bytes(f) == std::vector<uint8_t>({0x40, 0x00, 0x00}));
	CHECK(f.time == 500);
	CHECK(peer0.violations().empty());
	CHECK(peer0.requestCount(FakeVescPeer::COMM_FW_VERSION) == 1);
}

TEST_CASE_FIXTURE(VescFixture, "[V40] characterization: with a silent VESC the driver repeats the FW version request at 500, 1000, 1500 and 2000 ms (fixture scheduling)") {
	auto v = makeVesc1();
	peer0.setSilent(true);
	runFor(2000);
	const auto frames = txFramesPacket(0xFF, FakeVescPeer::PKT_PROCESS_SHORT_BUFFER);
	REQUIRE(frames.size() == 4);
	for (size_t i = 0; i < 4; i++) {
		CHECK(frames[i].time == 500 + 500 * i);
	}
	CHECK_FALSE(v->motorReady());
}

TEST_CASE_FIXTURE(VescFixture, "[V30] reachReady brings VESC_1 against the 7.01 peer to READY: FW reply, then values reply with fault 0") {
	auto v = makeVesc1();
	reachReady(*v);
	CHECK(v->motorReady());
	CHECK(peer0.requestCount(FakeVescPeer::COMM_FW_VERSION) == 1);
	CHECK(peer0.requestCount(FakeVescPeer::COMM_GET_VALUES_SELECTIVE) >= 1);
	CHECK(peer0.violations().empty());
}

TEST_CASE_FIXTURE(VescFixture, "[V13] the values request has the id of PROCESS_SHORT_BUFFER and the first 7 bytes [0x40, 0x00, 50, 00 00 81 00]") {
	auto v = makeVesc1();
	reachReady(*v);
	const auto frames = txFramesPacket(0xFF, FakeVescPeer::PKT_PROCESS_SHORT_BUFFER);
	REQUIRE(frames.size() >= 2);
	const auto &f = frames[1];
	CHECK(f.dlc >= 7);
	CHECK(std::vector<uint8_t>(f.data.begin(), f.data.begin() + 7) == std::vector<uint8_t>({0x40, 0x00, 50, 0x00, 0x00, 0x81, 0x00}));
}

TEST_CASE_FIXTURE(VescFixture, "[V70] VESC_2 against peer 1 reaches READY and sends its frames to the broadcast id like VESC_1") {
	peer0.setPresent(false);
	peer1.setPresent(true);
	auto v = makeVesc2();
	reachReady(*v);
	CHECK(v->motorReady());
	CHECK(peer1.requestCount(FakeVescPeer::COMM_FW_VERSION) == 1);
	CHECK(peer0.log().empty());
	CHECK(peer1.violations().empty());
}

}
