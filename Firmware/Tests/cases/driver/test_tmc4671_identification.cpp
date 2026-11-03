/*
 * TMC4671 driver, group G2: identification and presence (requirements R10-R14).
 * Charter: Firmware/Tests/driver/TMC4671_REQUIREMENTS.md. Fake peer: fakes/FakeTmc4671.h.
 *
 * Sources: DS = TMC4671-LA datasheet rev 2.07: CHIPINFO_DATA 0x00 selected by CHIPINFO_ADDR 0x01 (0 SI_TYPE = ASCII "4671"
 * = 0x34363731, 1 SI_VERSION u16.u16), DS §4.2.1 p17 example, DS 0x00/0x01 p77.
 *
 * Host vs target: no size dependent assertions; the version string comes from std::to_string on uint32_t values.
 */
#include "doctest.h"
#include "TmcFixture.h"

#include <algorithm>
#include <string>
#include <vector>

namespace {

/** Reply of the "tmctype" command: text and number */
CommandReply tmctype(TMC4671 &tmc) {
	std::vector<CommandReply> replies;
	CHECK(CommandHelpers::run(tmc, "tmctype", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	return replies.front();
}

bool hasError(ErrorCode code) {
	for (const Error &e : ErrorHandler::getErrors()) {
		if (e.isError() && e.code == code) {
			return true;
		}
	}
	return false;
}

} // namespace

TEST_SUITE("TMC4671 identification") {

// ---------------------------------------------------------------- R10

TEST_CASE_FIXTURE(TmcFixture, "[R10] pingDriver reports a chip with SI_TYPE 0x34363731 present") {
	auto tmc = makeTmc1();
	CHECK(tmc->pingDriver());
}

TEST_CASE_FIXTURE(TmcFixture, "[R10] pingDriver selects CHIPINFO_ADDR 0 with a write and then reads CHIPINFO_DATA 0x00") {
	auto tmc = makeTmc1();
	chip1.setChipInfo(1, 0x00010003);
	// Leave CHIPINFO_ADDR on 1 (SI_VERSION) so the ping must change it. DS 0x01 p77
	chip1.setReg(0x01, 1);
	chip1.clearLog();
	CHECK(tmc->pingDriver());
	const auto &log = chip1.log();
	REQUIRE(log.size() == 2);
	CHECK(log[0].write);
	CHECK(log[0].addr == 0x01);
	CHECK(log[0].data == 0u); // SI_TYPE
	CHECK_FALSE(log[1].write);
	CHECK(log[1].addr == 0x00);
	CHECK(log[1].answer == 0x34363731u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R10] pingDriver reports the chip absent for SI_TYPE 0, 0xFFFFFFFF and other values") {
	auto tmc = makeTmc1();
	// HD: all values differ from 0x34363731 ("4671"); bus without chip reads 0 (no pull) or 0xFFFFFFFF (pull-up),
	// the others are near misses: last digit changed, byte order reversed, one bit flipped
	const uint32_t values[] = {0x00000000, 0xFFFFFFFF, 0x34363730, 0x31373634, 0x34363731 ^ 0x80000000, 0x00000001};
	for (uint32_t v : values) {
		chip1.setChipInfo(0, v);
		CHECK_MESSAGE(!tmc->pingDriver(), "SI_TYPE 0x" << std::hex << v);
	}
}

TEST_CASE_FIXTURE(TmcFixture, "[R10] pingDriver reports the chip absent when the chip does not answer") {
	auto tmc = makeTmc1();
	chip1.setPresent(false);
	CHECK_FALSE(tmc->pingDriver());
}

// ---------------------------------------------------------------- R11

TEST_CASE_FIXTURE(TmcFixture, "[R11] tmctype names the chip 4671 when SI_TYPE reads 0x34363731") {
	auto tmc = makeTmc1();
	CommandReply r = tmctype(*tmc);
	// DS §4.2.1 p17: 0x34363731 is ASCII "4671"
	CHECK(r.reply.find("4671") != std::string::npos);
}

TEST_CASE_FIXTURE(TmcFixture, "[R11] tmctype reports SI_VERSION 0x00010003 as major.minor 1.3"
							  * KNOWN_ISSUE("B-T3: getTmcType uses '&&' instead of '&' (TMC4671.cpp:1855), both version numbers become 1 or 0")) {
	auto tmc = makeTmc1();
	chip1.setChipInfo(1, 0x00010003);
	CommandReply r = tmctype(*tmc);
	// DS 0x00 p77: SI_VERSION is u16.u16 -> major 1 (bits 31..16), minor 3 (bits 15..0)
	CHECK(r.reply.find("1.3") != std::string::npos);
}

TEST_CASE_FIXTURE(TmcFixture, "[R11] tmctype reports SI_VERSION 0x00020007 as major.minor 2.7"
							  * KNOWN_ISSUE("B-T3: getTmcType uses '&&' instead of '&' (TMC4671.cpp:1855), both version numbers become 1 or 0")) {
	auto tmc = makeTmc1();
	chip1.setChipInfo(1, 0x00020007);
	CommandReply r = tmctype(*tmc);
	// DS 0x00 p77: major 2, minor 7
	CHECK(r.reply.find("2.7") != std::string::npos);
}

TEST_CASE_FIXTURE(TmcFixture, "[R11] tmctype says no driver connected when SI_TYPE reads 0") {
	auto tmc = makeTmc1();
	chip1.setChipInfo(0, 0x00000000);
	CommandReply r = tmctype(*tmc);
	CHECK(r.reply.find("No driver connected") != std::string::npos);
	CHECK(r.reply.find("4671") == std::string::npos);
}

TEST_CASE_FIXTURE(TmcFixture, "[R11] tmctype says no driver connected when SI_TYPE reads 0xFFFFFFFF") {
	auto tmc = makeTmc1();
	chip1.setChipInfo(0, 0xFFFFFFFF);
	CommandReply r = tmctype(*tmc);
	CHECK(r.reply.find("No driver connected") != std::string::npos);
	CHECK(r.reply.find("4671") == std::string::npos);
}

// ---------------------------------------------------------------- R12

TEST_CASE_FIXTURE(TmcFixture, "[R12] characterization: an engineering sample (SI_VERSION 0x00010000) lowers the SPI clock to about 1 MHz") {
	auto tmc = makeTmc1();
	chip1.setChipInfo(1, 0x00010000);
	const double before = 84e6 / double(2u << (hspi1.Init.BaudRatePrescaler >> 3));
	reachShutdown(*tmc);
	const double after = 84e6 / double(2u << (hspi1.Init.BaudRatePrescaler >> 3));
	// HD: closest prescaler to 1 MHz from 84 MHz / {2..256}: 84/64 = 1.3125 MHz (distance 0.3125) beats 84/128 = 0.65625 (0.34375)
	CHECK(before > 2e6);
	CHECK(hspi1.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_64);
	CHECK(after == doctest::Approx(1.3125e6));
}

TEST_CASE_FIXTURE(TmcFixture, "[R12] characterization: an engineering sample does not write the PI precision (CONFIG_ADDR 62)") {
	auto tmc = makeTmc1();
	chip1.setChipInfo(1, 0x00010000);
	reachShutdown(*tmc);
	// DS 0x4E p94: CONFIG_ADDR 62 = ADVANCED_PI_REPRESENT
	for (const auto &w : chip1.configWrites()) {
		CHECK_MESSAGE(w.first != 62u, "CONFIG_DATA written with CONFIG_ADDR 62");
	}
}

TEST_CASE_FIXTURE(TmcFixture, "[R12] characterization: a V1.3 chip keeps the fast SPI clock and gets the PI precision write (control case of R12)") {
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	CHECK(hspi1.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_16); // HD: closest to 8 MHz below 10 MHz: 84/16 = 5.25 MHz
	const auto &w = chip1.configWrites();
	const bool wrote62 = std::any_of(w.begin(), w.end(), [](const std::pair<uint32_t, uint32_t> &p) { return p.first == 62u; });
	CHECK(wrote62);
}

// ---------------------------------------------------------------- R13

TEST_CASE_FIXTURE(TmcFixture, "[R13] characterization: a silent chip leaves the driver uninitialized") {
	auto tmc = makeTmc1();
	chip1.setPresent(false);
	powerOn();
	tmc->setupDriver();
	runThreadFor(*tmc, 2000);
	CHECK(tmc->getState() == TMC_ControlState::uninitialized);
	CHECK_FALSE(tmc->motorReady());
}

TEST_CASE_FIXTURE(TmcFixture, "[R13] characterization: a silent chip gets only the ping (write CHIPINFO_ADDR 0, read CHIPINFO_DATA)") {
	auto tmc = makeTmc1();
	chip1.setPresent(false);
	powerOn();
	tmc->setupDriver();
	chip1.clearLog();
	runThreadFor(*tmc, 2000);
	REQUIRE_FALSE(chip1.log().empty());
	for (const auto &f : chip1.log()) {
		if (f.write) {
			CHECK_MESSAGE(f.addr == 0x01, "write to register 0x" << std::hex << int(f.addr));
			CHECK(f.data == 0u);
		} else {
			CHECK_MESSAGE(f.addr == 0x00, "read of register 0x" << std::hex << int(f.addr));
		}
	}
	CHECK(chip1.configWrites().empty());
}

TEST_CASE_FIXTURE(TmcFixture, "[R13] characterization: a silent chip is pinged again every 250 ms") {
	auto tmc = makeTmc1();
	chip1.setPresent(false);
	powerOn();
	tmc->setupDriver();
	chip1.clearLog();
	runThreadFor(*tmc, 2000);
	std::vector<uint64_t> pingTimes;
	for (const auto &f : chip1.log()) {
		if (!f.write && f.addr == 0x00) {
			pingTimes.push_back(f.timeUs);
		}
	}
	// HD: Delay(250) in the uninitialized state, window 2000 ms -> 2000 / 250 = 8 pings (7 to 9 allowed for the window edges)
	REQUIRE(pingTimes.size() >= 7);
	CHECK(pingTimes.size() <= 9);
	for (size_t i = 1; i < pingTimes.size(); i++) {
		const uint64_t gap = pingTimes[i] - pingTimes[i - 1];
		// The fixture's fake kernel delivers delays in steps of at most 10 ms (setDelayBudgetMs(10)): gap in [250, 260] ms
		CHECK_MESSAGE(gap >= 250000, "gap " << gap << " us");
		CHECK_MESSAGE(gap <= 260000, "gap " << gap << " us");
	}
}

TEST_CASE_FIXTURE(TmcFixture, "[R13] characterization: a chip that appears later is initialised by the retry") {
	auto tmc = makeTmc1();
	chip1.setPresent(false);
	powerOn();
	chip1.setAdcRaw(0, 0x7FFF7FFF);
	chip1.setMicrosPerTransfer(10);
	tmc->setupDriver();
	runThreadFor(*tmc, 1000);
	REQUIRE(tmc->getState() == TMC_ControlState::uninitialized);
	chip1.setPresent(true);
	CHECK(runUntilState(*tmc, TMC_ControlState::Shutdown, 10000));
}

// ---------------------------------------------------------------- R14

TEST_CASE_FIXTURE(TmcFixture, "[R14] a chip that never answers is reported as a communication error after 5 s"
							  * KNOWN_ISSUE("B-T4: the uninitialized state only pings and retries; the communication error is only raised inside initialize(), which is not reached when the ping fails")) {
	auto tmc = makeTmc1();
	chip1.setPresent(false);
	powerOn();
	tmc->setupDriver();
	runThreadFor(*tmc, 5000);
	// The firmware defines ErrorCode::tmcCommunicationError ("TMC not responding") for exactly this case (lead decision L12)
	CHECK(hasError(ErrorCode::tmcCommunicationError));
}

TEST_CASE_FIXTURE(TmcFixture, "[R14] no communication error is raised while the chip answers") {
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	runThreadFor(*tmc, 1000);
	CHECK_FALSE(hasError(ErrorCode::tmcCommunicationError));
}

} // TEST_SUITE
