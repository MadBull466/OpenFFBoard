/*
 * Smoke tests of the TMC4671 driver test infrastructure (fixture + FakeTmc4671 + driver link together).
 * Behaviour tests of the driver are written per requirement group (Tests/claude/charters/driver/TMC4671_REQUIREMENTS.md).
 */
#include "doctest.h"
#include "TmcFixture.h"

TEST_SUITE("TMC4671 smoke") {

TEST_CASE_FIXTURE(TmcFixture, "[R1] constructing TMC_1 with a present chip answers the ping and sends only 5 byte frames with CS low") {
	auto tmc = makeTmc1();
	CHECK(tmc->pingDriver());
	const auto &transfers = HostTarget::spiTransfers(&hspi1);
	REQUIRE_FALSE(transfers.empty());
	for (const auto &t : transfers) {
		CHECK(t.size == 5);
		CHECK(t.selectedCs() == 0); // PA4 = TMC_1
	}
	CHECK(chip1.violations().empty());
}

TEST_CASE_FIXTURE(TmcFixture, "[R70] reachShutdown brings a TMC_1 with a present chip, power and no encoder to Shutdown") {
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	CHECK(tmc->getState() == TMC_ControlState::Shutdown);
	CHECK_FALSE(tmc->motorReady());
}

TEST_CASE_FIXTURE(TmcFixture, "[R72] reachRunning brings TMC_1 to Running with motorReady") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	CHECK(tmc->getState() == TMC_ControlState::Running);
	CHECK(tmc->motorReady());
}

TEST_CASE_FIXTURE(TmcFixture, "[R70] runUntilState returns false and the state stays when the supply is missing") {
	auto tmc = makeTmc1();
	chip1.setMicrosPerTransfer(10);
	chip1.setAdcRaw(0, 0x7FFF7FFF); // all-zero ADC words crash calibrateAdcOffset (division by zero, charter R78 / B-T16)
	powerOff();
	tmc->setupDriver();
	CHECK_FALSE(runUntilState(*tmc, TMC_ControlState::Shutdown, 2000));
	CHECK(tmc->getState() != TMC_ControlState::Shutdown);
}

}
