/*
 * P16 T1 skeleton of the MyActuator RMD driver tests: proves that the fixture, FakeRmdPeer and the driver link together.
 * Behaviour tests are written per requirement group in T2 (Tests/claude/charters/driver/RMD_REQUIREMENTS.md).
 */
#include "doctest.h"
#include "RmdFixture.h"

TEST_SUITE("RMD setup") {

TEST_CASE_FIXTURE(RmdFixture, "[R2.1] constructing RmdMotorCAN1 registers the reply filter 0x241 with mask 0x3FF and starts the CAN port") {
	auto rmd = makeRmd1();
	CHECK(HostTarget::canPort().started);
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x241); // SP 1.1.2: reply id 0x240 + ID, ID 1
	CHECK(HostTarget::canPort().filters[0].filter_mask == 0x3FF);
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.1] constructing RmdMotorCAN2 registers the reply filter 0x242 for node 2") {
	auto rmd = makeRmd2();
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x242); // ID 2
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.5] characterization: the thread sends nothing before 100 ms and the 0xB5 model request at 100 ms to id 0x141 with DLC 8") {
	auto rmd = makeRmd1();
	runFor(99);
	CHECK(HostTarget::canTxFrames().empty());
	runFor(1);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	const auto &f = HostTarget::canTxFrames()[0];
	CHECK(f.id == 0x141); // SP 1.1.2: 0x140 + ID 1
	CHECK_FALSE(f.extId);
	CHECK_FALSE(f.rtr);
	CHECK(f.dlc == 8);
	CHECK(f.data[0] == 0xB5); // SP 2.33
	CHECK(f.time == 100);	  // characterization: the 100 ms start delay is firmware behaviour
	CHECK(peer1.violations().empty());
}

TEST_CASE_FIXTURE(RmdFixture, "[R4.1] characterization: the thread runs at 100 ms, 120 ms (0xB6 enable of 0x92) and the 0xB5 again at 1270 ms against a silent motor (fixture scheduling of every Delay)") {
	auto rmd = makeRmd1();
	peer1.setSilent(true);
	runFor(1271);
	const auto b5 = txFramesCmd(0x141, 0xB5);
	REQUIRE(b5.size() == 2);
	CHECK(b5[0].time == 100);
	CHECK(b5[1].time == 1270); // 100 + 20 + 150 + 1000: the second cycle starts 1170 ms after the first (characterization of the constants)
	const auto b6 = txFramesCmd(0x141, 0xB6);
	REQUIRE(b6.size() >= 1);
	CHECK(b6[0].time == 120);
	CHECK(b6[0].data[1] == 0x92);
	CHECK(b6[0].data[2] == 1);
	CHECK(peer1.violations().empty());
}

TEST_CASE_FIXTURE(RmdFixture, "[R4.2] a motor that answers makes motorReady true at the end of the first cycle and registers the cyclic position report") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	CHECK(rmd->motorReady());
	CHECK(peer1.cyclicEnabled(0x92)); // the firmware's 0xB6 enable was accepted
	CHECK(peer1.violations().empty());
}

TEST_CASE_FIXTURE(RmdFixture, "[R10.4] RmdMotorCAN2 receives the replies of node 2 on FIFO 1 and reaches motorReady against peer 2") {
	auto rmd = makeRmd2();
	reachAvailable(*rmd);
	CHECK(rmd->motorReady());
	REQUIRE(HostTarget::canTxFrames().size() > 0);
	CHECK(HostTarget::canTxFrames()[0].id == 0x142);
	CHECK(peer2.violations().empty());
}

TEST_CASE_FIXTURE(RmdFixture, "[R5.5] the 0xB5 reply of the motor is read back through the model command") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	// SP 2.33 example: 0x58 0x38 0x53 0x32 0x56 0x31 0x30 = "X8S2V10" (the peer's default model)
	std::vector<CommandReply> replies;
	REQUIRE(CommandHelpers::run(*rmd, "model", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "X8S2V10");
}

}
