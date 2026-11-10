/*
 * PWM driver (MotorPWM), phase P14 T2, work order PWC: commands (R4.1-R4.7) and persistence (R5.1-R5.5).
 * Charter: Tests/claude/charters/driver/PWM_REQUIREMENTS.md. Unit: Firmware/FFBoard/UserExtensions/Src/MotorPWM.cpp.
 *
 * Requirements covered
 *   R4.1 freq / mode / dir get and set (replies, getters, effect at the fake TIM1 and on the output)
 *   R4.2 info of freq (kHz names, RC names in RC mode), R4.3 info of mode, exact text including the "\n"
 *   R4.4 freq set above 3, R4.5 mode set above 3, R4.6 negative / huge / aliasing values, R4.7 unknown command id
 *   R5.1 saved word bit layout (flash address ADR_PWM_MODE), R5.2 round trip of all 32 combinations,
 *   R5.3 empty flash, R5.4 stored invalid speed / mode, R5.5 bit 3 of the stored word
 *
 * Oracles: NAMES (the firmware's own labels for the info lines), HD (hand derivations in comments next to the assertions:
 * ARR = 168000000 / f, RC ARR / PSC, CCR = pval * ARR / 0xffff, flash word bits), FW where the charter says so.
 * Commands are resolved by name (CommandHelpers::run). The flash word is read at the fake EEPROM
 * (HostPlatform::flash()[ADR_PWM_MODE]), like the ODrive flash tests.
 *
 * Timer values of the four speeds (168 MHz, hand derived, charter R2.2 / R2.3):
 *   non-RC: LOW 168e6/3000 = 56000, MID 168e6/9000 = 18666 (floor of 18666.67), HIGH 168e6/17000 = 9882 (floor of 9882.35),
 *           VERYHIGH 168e6/24000 = 7000, always PSC 0 (arr_raw / 0xffff = 0 for all four)
 *   RC:     ARR 40000 / 30000 / 20000 / 10000, PSC 168e6 / 2e6 = 84
 *
 * Host versus target: command values are int64_t (host and target alike); the cast (SpeedPWM_DRV) / (ModePWM_DRV) of an
 * int64_t goes to an enum with the fixed underlying type uint8_t, so the conversion is modulo 256 on both (no UB, same on
 * ARM GCC). The centered CCR product (0x7fff + power) * 56000 is computed in int32_t by the firmware and
 * overflows from pval 38348, i.e. power 5581 (38347 * 56000 = 2147432000 still fits, 38348 * 56000 = 2147488000 > 2147483647; host and target alike, signed overflow); this file keeps
 * every output check at power <= 5000 (37767 * 56000 = 2114952000 fits) and reports the overflow to the lead.
 *
 * Characterizations (NEEDS REVIEW): R5.4 stored speed 4..7 ignored; R5.5 bit 3 of the stored word taken into the mode, and
 * saveFlash masks the mode with 3 bits.
 * Reserved bits 8-14 of the stored word are ignored (plain test).
 * KNOWN_ISSUEs: B-PWM-2 (freq set range check, R4.4 / R4.6), B-PWM-3 (mode set / stored mode range check, R4.5 / R4.6 /
 * R5.4). Every KNOWN_ISSUE initialises the driver unconditionally.
 */
#include "doctest.h"
#include "PwmFixture.h"

#include "CommandHelpers.h"
#include "eeprom_addresses.h"

#include <climits>
#include <cstdint>
#include <string>
#include <vector>

namespace {

struct SpeedRow {
	SpeedPWM_DRV speed;
	uint32_t arrPwm; // non-RC mode
	uint32_t arrRc;	 // RC mode
};
// Hand derivation in the file header
const SpeedRow kSpeeds[] = {
	{SpeedPWM_DRV::LOW, 56000, 40000},
	{SpeedPWM_DRV::MID, 18666, 30000},
	{SpeedPWM_DRV::HIGH, 9882, 20000},
	{SpeedPWM_DRV::VERYHIGH, 7000, 10000},
};
constexpr uint32_t kRcPsc = 84; // 168000000 / 2000000

uint16_t storedWord() { return HostPlatform::flash().at(ADR_PWM_MODE); }

CommandStatus exec(MotorPWM &p, const char *name, CMDtype type, int64_t val = 0, std::vector<CommandReply> *replies = nullptr) {
	return CommandHelpers::run(p, name, type, val, replies);
}

/** Constructs a driver with the given state, saves it and destroys it again; returns the word that reached the flash */
uint16_t saveState(PwmFixture &f, ModePWM_DRV mode, SpeedPWM_DRV speed, bool invert) {
	auto p = f.makePwm();
	p->setMode(mode);
	p->setPwmSpeed(speed);
	exec(*p, "dir", CMDtype::set, invert ? 1 : 0);
	p->saveFlash();
	p.reset();
	return storedWord();
}

} // namespace

TEST_SUITE("PWM config") {

// ------------------------------------------------------------------ R4.1 freq / mode / dir

TEST_CASE_FIXTURE(PwmFixture, "[R4.1 freq] get replies the speed index 0 after construction on an empty flash") {
	auto p = makePwm();
	std::vector<CommandReply> r;
	CHECK(exec(*p, "freq", CMDtype::get, 0, &r) == CommandStatus::OK);
	REQUIRE(r.size() == 1);
	CHECK(r.front().val == 0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.1 freq] set 0..3 selects the speed, freq get returns it and the timer holds the kHz period") {
	auto p = makePwm();
	for (size_t i = 0; i < 4; ++i) {
		INFO("speed " << i);
		CHECK(exec(*p, "freq", CMDtype::set, static_cast<int64_t>(i)) == CommandStatus::OK);
		CHECK(p->getPwmSpeed() == kSpeeds[i].speed);
		CHECK(CommandHelpers::get(*p, "freq") == static_cast<int64_t>(i));
		CHECK(arr() == kSpeeds[i].arrPwm);
		CHECK(psc() == 0);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.1 freq] set in RC mode selects the RC frame period: ARR 40000 / 30000 / 20000 / 10000 with PSC 84") {
	auto p = makePwm();
	REQUIRE(exec(*p, "mode", CMDtype::set, 0) == CommandStatus::OK);
	for (size_t i = 0; i < 4; ++i) {
		INFO("speed " << i);
		CHECK(exec(*p, "freq", CMDtype::set, static_cast<int64_t>(i)) == CommandStatus::OK);
		CHECK(arr() == kSpeeds[i].arrRc);
		CHECK(psc() == kRcPsc);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.1 mode] get replies the default mode 1 (centered) after construction on an empty flash") {
	auto p = makePwm();
	CHECK(CommandHelpers::get(*p, "mode") == 1);
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.1 mode] set 0..3 selects the mode and mode get returns it") {
	auto p = makePwm();
	const ModePWM_DRV modes[] = {ModePWM_DRV::RC_PWM, ModePWM_DRV::CENTERED_PWM, ModePWM_DRV::PWM_DIR, ModePWM_DRV::PWM_DUAL};
	for (int i = 0; i < 4; ++i) {
		INFO("mode " << i);
		CHECK(exec(*p, "mode", CMDtype::set, i) == CommandStatus::OK);
		CHECK(p->getMode() == modes[i]);
		CHECK(CommandHelpers::get(*p, "mode") == i);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.1 mode] set re-initialises the timer for the new mode: RC gives ARR 40000 / PSC 84, back to a PWM mode gives ARR 56000 / PSC 0") {
	auto p = makePwm();
	REQUIRE(exec(*p, "mode", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(arr() == 40000);
	CHECK(psc() == kRcPsc);
	REQUIRE(exec(*p, "mode", CMDtype::set, 2) == CommandStatus::OK);
	CHECK(arr() == 56000);
	CHECK(psc() == 0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.1 dir] get replies 0 after construction on an empty flash") {
	auto p = makePwm();
	CHECK(CommandHelpers::get(*p, "dir") == 0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.1 dir] set 1 then set 0 is read back by dir get") {
	auto p = makePwm();
	CHECK(exec(*p, "dir", CMDtype::set, 1) == CommandStatus::OK);
	CHECK(CommandHelpers::get(*p, "dir") == 1);
	CHECK(exec(*p, "dir", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(CommandHelpers::get(*p, "dir") == 0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.1 dir] set 1 inverts the output: centered power +1000 gives the CCR of -1000") {
	auto p = makePwm();
	p->startMotor();
	// centered, ARR 56000: CCR = (0x7fff + power) * 56000 / 0xffff (integer)
	//   power +1000: 33767 * 56000 / 65535 = 28854 (1890952000 / 65535 = 28854.x)
	//   power -1000: 31767 * 56000 / 65535 = 27145 (1778952000 / 65535 = 27145.x)
	p->turn(1000);
	REQUIRE(ccr(1) == 28854);
	REQUIRE(exec(*p, "dir", CMDtype::set, 1) == CommandStatus::OK);
	p->turn(1000);
	CHECK(ccr(1) == 27145);
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.1 mode] set keeps the speed and the direction") {
	auto p = makePwm();
	REQUIRE(exec(*p, "freq", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE(exec(*p, "dir", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(exec(*p, "mode", CMDtype::set, 2) == CommandStatus::OK);
	std::vector<CommandReply> r;
	CHECK(exec(*p, "freq", CMDtype::get, 0, &r) == CommandStatus::OK);
	REQUIRE(r.size() == 1);
	CHECK(r.front().val == 2);
	r.clear();
	CHECK(exec(*p, "dir", CMDtype::get, 0, &r) == CommandStatus::OK);
	REQUIRE(r.size() == 1);
	CHECK(r.front().val == 1);
	CHECK(arr() == 9882); // HIGH in a PWM mode: 168e6 / 17000
	CHECK(psc() == 0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.1 freq] set keeps the mode and the direction") {
	auto p = makePwm();
	REQUIRE(exec(*p, "mode", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE(exec(*p, "dir", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(exec(*p, "freq", CMDtype::set, 3) == CommandStatus::OK);
	std::vector<CommandReply> r;
	CHECK(exec(*p, "mode", CMDtype::get, 0, &r) == CommandStatus::OK);
	REQUIRE(r.size() == 1);
	CHECK(r.front().val == 2);
	r.clear();
	CHECK(exec(*p, "dir", CMDtype::get, 0, &r) == CommandStatus::OK);
	REQUIRE(r.size() == 1);
	CHECK(r.front().val == 1);
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.1 command] freq, mode and dir set answer no reply and write nothing to the flash") {
	auto p = makePwm();
	for (const char *name : {"freq", "mode", "dir"}) {
		INFO("command " << name);
		std::vector<CommandReply> r;
		CHECK(exec(*p, name, CMDtype::set, 1, &r) == CommandStatus::OK);
		CHECK(r.empty());
	}
	CHECK(HostPlatform::flash().empty()); // persistence only through saveFlash
}

// ------------------------------------------------------------------ R4.2 / R4.3 info

TEST_CASE_FIXTURE(PwmFixture, "[R4.2 freq] info lists the four kHz names with their index in a PWM mode") {
	auto p = makePwm();
	std::vector<CommandReply> r;
	CHECK(exec(*p, "freq", CMDtype::info, 0, &r) == CommandStatus::OK);
	REQUIRE(r.size() == 4);
	CHECK(r[0].reply == "3khz:0\n");
	CHECK(r[1].reply == "9khz:1\n");
	CHECK(r[2].reply == "17khz:2\n");
	CHECK(r[3].reply == "24khz:3\n");
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.2 freq] info lists the kHz names in every PWM mode (centered, PWM/DIR, dual)") {
	auto p = makePwm();
	for (int m = 1; m <= 3; ++m) {
		INFO("mode " << m);
		REQUIRE(exec(*p, "mode", CMDtype::set, m) == CommandStatus::OK);
		std::vector<CommandReply> r;
		CHECK(exec(*p, "freq", CMDtype::info, 0, &r) == CommandStatus::OK);
		REQUIRE(r.size() == 4);
		CHECK(r[0].reply == "3khz:0\n");
		CHECK(r[1].reply == "9khz:1\n");
		CHECK(r[2].reply == "17khz:2\n");
		CHECK(r[3].reply == "24khz:3\n");
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.2 freq] info lists the four frame times with their index in RC mode") {
	auto p = makePwm();
	REQUIRE(exec(*p, "mode", CMDtype::set, 0) == CommandStatus::OK);
	std::vector<CommandReply> r;
	CHECK(exec(*p, "freq", CMDtype::info, 0, &r) == CommandStatus::OK);
	REQUIRE(r.size() == 4);
	CHECK(r[0].reply == "20ms:0\n");
	CHECK(r[1].reply == "15ms:1\n");
	CHECK(r[2].reply == "10ms:2\n");
	CHECK(r[3].reply == "5ms:3\n");
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.3 mode] info lists the four mode names with their index") {
	auto p = makePwm();
	std::vector<CommandReply> r;
	CHECK(exec(*p, "mode", CMDtype::info, 0, &r) == CommandStatus::OK);
	REQUIRE(r.size() == 4);
	CHECK(r[0].reply == "RC PPM:0\n");
	CHECK(r[1].reply == "0%-50%-100% Centered:1\n");
	CHECK(r[2].reply == "0-100% PWM/DIR:2\n");
	CHECK(r[3].reply == "0-100% dual PWM:3\n");
}

// ------------------------------------------------------------------ R4.4 freq set above 3

TEST_CASE_FIXTURE(PwmFixture, "[R4.4 freq] set above 3 leaves the speed and the timer period unchanged") {
	auto p = makePwm();
	REQUIRE(exec(*p, "freq", CMDtype::set, 2) == CommandStatus::OK); // HIGH: ARR 9882 (168e6 / 17000)
	for (int64_t v : {4, 5, 100, 255}) {
		INFO("value " << v);
		exec(*p, "freq", CMDtype::set, v);
		CHECK(p->getPwmSpeed() == SpeedPWM_DRV::HIGH);
		CHECK(arr() == 9882);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.4 freq] set above 3 is rejected with status ERR" *
								  KNOWN_ISSUE("B-PWM-2: freq set 4..255 is ignored silently and answers OK (no range check in MotorPWM::command)")) {
	auto p = makePwm();
	for (int64_t v : {4, 5, 100, 255}) {
		INFO("value " << v);
		CHECK(exec(*p, "freq", CMDtype::set, v) == CommandStatus::ERR);
	}
}

// ------------------------------------------------------------------ R4.5 mode set above 3

TEST_CASE_FIXTURE(PwmFixture, "[R4.5 mode] set 4 is rejected with status ERR" *
								  KNOWN_ISSUE("B-PWM-3: mode set 4 is accepted: status OK, the invalid mode is stored")) {
	auto p = makePwm();
	CHECK(exec(*p, "mode", CMDtype::set, 4) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.5 mode] set 4 leaves the mode unchanged" *
								  KNOWN_ISSUE("B-PWM-3: mode get returns 4 after mode set 4")) {
	auto p = makePwm();
	REQUIRE(exec(*p, "mode", CMDtype::set, 2) == CommandStatus::OK);
	exec(*p, "mode", CMDtype::set, 4);
	CHECK(p->getMode() == ModePWM_DRV::PWM_DIR);
	CHECK(CommandHelpers::get(*p, "mode") == 2);
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.5 mode] set 4 while running leaves the output alive: centered power +5000 still gives its CCR" *
								  KNOWN_ISSUE("B-PWM-3: after mode set 4 turn writes nothing, CCR1 stays 0 (output frozen)")) {
	auto p = makePwm();
	p->startMotor();
	exec(*p, "mode", CMDtype::set, 4);
	p->turn(5000);
	// intended: mode stays centered, ARR 56000: CCR1 = (32767 + 5000) * 56000 / 65535 = 2114952000 / 65535 = 32272.x -> 32272
	CHECK(ccr(1) == 32272);
}

// ------------------------------------------------------------------ R4.6 negative / huge / aliasing values

TEST_CASE_FIXTURE(PwmFixture, "[R4.6 freq] values 256, 257, 258 and INT64_MIN do not alias to speeds 0, 1, 2" *
								  KNOWN_ISSUE("B-PWM-2: cmd.val is truncated to uint8_t, freq set 256 selects speed 0")) {
	auto p = makePwm();
	for (int64_t v : {int64_t(256), int64_t(257), int64_t(258), INT64_MIN}) {
		INFO("value " << v);
		REQUIRE(exec(*p, "freq", CMDtype::set, 3) == CommandStatus::OK); // VERYHIGH
		exec(*p, "freq", CMDtype::set, v);
		CHECK(p->getPwmSpeed() == SpeedPWM_DRV::VERYHIGH);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.6 freq] set -1 and INT64_MAX are rejected with status ERR" *
								  KNOWN_ISSUE("B-PWM-2: both truncate to 255, which is ignored, the status is OK")) {
	auto p = makePwm();
	for (int64_t v : {int64_t(-1), INT64_MAX}) {
		INFO("value " << v);
		CHECK(exec(*p, "freq", CMDtype::set, v) == CommandStatus::ERR);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.6 mode] values 256, 257, 258 and INT64_MIN do not alias to modes 0, 1, 2" *
								  KNOWN_ISSUE("B-PWM-3: cmd.val is truncated to uint8_t, mode set 256 selects mode 0 (RC)")) {
	auto p = makePwm();
	for (int64_t v : {int64_t(256), int64_t(257), int64_t(258), INT64_MIN}) {
		INFO("value " << v);
		REQUIRE(exec(*p, "mode", CMDtype::set, 3) == CommandStatus::OK); // PWM_DUAL
		exec(*p, "mode", CMDtype::set, v);
		CHECK(p->getMode() == ModePWM_DRV::PWM_DUAL);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.6 mode] set -1 and INT64_MAX are rejected with status ERR" *
								  KNOWN_ISSUE("B-PWM-3: both truncate to 255 which is stored, the status is OK")) {
	auto p = makePwm();
	for (int64_t v : {int64_t(-1), INT64_MAX}) {
		INFO("value " << v);
		CHECK(exec(*p, "mode", CMDtype::set, v) == CommandStatus::ERR);
	}
}

// ------------------------------------------------------------------ R4.7 unknown id

TEST_CASE_FIXTURE(PwmFixture, "[R4.7 command] an unknown command id returns NOT_FOUND and changes nothing") {
	auto p = makePwm();
	ParsedCommand cmd;
	cmd.cmdId = 1000; // far above the three ids of the driver
	cmd.type = CMDtype::set;
	cmd.val = 1;
	cmd.target = p.get();
	std::vector<CommandReply> r;
	CHECK(p->command(cmd, r) == CommandStatus::NOT_FOUND);
	CHECK(r.empty());
	CHECK(p->getMode() == ModePWM_DRV::CENTERED_PWM);
	CHECK(p->getPwmSpeed() == SpeedPWM_DRV::LOW);
}

TEST_CASE_FIXTURE(PwmFixture, "[R4.7 command] an unknown command id returns NOT_FOUND with get and info as well") {
	auto p = makePwm();
	for (CMDtype t : {CMDtype::get, CMDtype::info}) {
		ParsedCommand cmd;
		cmd.cmdId = 1000;
		cmd.type = t;
		cmd.target = p.get();
		std::vector<CommandReply> r;
		CHECK(p->command(cmd, r) == CommandStatus::NOT_FOUND);
		CHECK(r.empty());
	}
}

// ------------------------------------------------------------------ R5.1 saved word

TEST_CASE_FIXTURE(PwmFixture, "[R5.1 save] mode 2, speed 3, inversion on is stored as the single word 0x8032 at ADR_PWM_MODE") {
	// bits: mode 2 = 0x0002; speed 3 << 4 = 0x0030; invert 1 << 15 = 0x8000; sum 0x8032
	CHECK(saveState(*this, ModePWM_DRV::PWM_DIR, SpeedPWM_DRV::VERYHIGH, true) == 0x8032);
	CHECK(HostPlatform::flash().size() == 1);
}

TEST_CASE_FIXTURE(PwmFixture, "[R5.1 save] the mode is stored in bits 0-2") {
	// speed LOW (0), no inversion: the word is the mode itself
	CHECK(saveState(*this, ModePWM_DRV::RC_PWM, SpeedPWM_DRV::LOW, false) == 0x0000);
	CHECK(saveState(*this, ModePWM_DRV::CENTERED_PWM, SpeedPWM_DRV::LOW, false) == 0x0001);
	CHECK(saveState(*this, ModePWM_DRV::PWM_DIR, SpeedPWM_DRV::LOW, false) == 0x0002);
	CHECK(saveState(*this, ModePWM_DRV::PWM_DUAL, SpeedPWM_DRV::LOW, false) == 0x0003);
}

TEST_CASE_FIXTURE(PwmFixture, "[R5.1 save] the speed is stored in bits 4-6") {
	// mode RC (0), no inversion: the word is speed << 4
	CHECK(saveState(*this, ModePWM_DRV::RC_PWM, SpeedPWM_DRV::LOW, false) == 0x0000);
	CHECK(saveState(*this, ModePWM_DRV::RC_PWM, SpeedPWM_DRV::MID, false) == 0x0010);
	CHECK(saveState(*this, ModePWM_DRV::RC_PWM, SpeedPWM_DRV::HIGH, false) == 0x0020);
	CHECK(saveState(*this, ModePWM_DRV::RC_PWM, SpeedPWM_DRV::VERYHIGH, false) == 0x0030);
}

TEST_CASE_FIXTURE(PwmFixture, "[R5.1 save] the inversion is stored in bit 15") {
	CHECK(saveState(*this, ModePWM_DRV::RC_PWM, SpeedPWM_DRV::LOW, true) == 0x8000);
	CHECK(saveState(*this, ModePWM_DRV::CENTERED_PWM, SpeedPWM_DRV::LOW, true) == 0x8001);
}

TEST_CASE_FIXTURE(PwmFixture, "[R5.1 save] all 32 combinations are stored as (mode & 3) | speed << 4 | invert << 15") {
	for (int m = 0; m < 4; ++m) {
		for (int s = 0; s < 4; ++s) {
			for (int inv = 0; inv < 2; ++inv) {
				INFO("mode " << m << " speed " << s << " invert " << inv);
				// hand-written layout of the charter: mode bits 0-1 (values 0..3), speed bits 4-5, inversion bit 15
				const uint16_t expected = static_cast<uint16_t>((m & 3) | (s << 4) | (inv << 15));
				CHECK(saveState(*this, static_cast<ModePWM_DRV>(m), static_cast<SpeedPWM_DRV>(s), inv != 0) == expected);
			}
		}
	}
}

// ------------------------------------------------------------------ R5.2 round trip

TEST_CASE_FIXTURE(PwmFixture, "[R5.2 restore] all 4 modes x 4 speeds x 2 directions survive save, destroy, construct, with the matching timer period") {
	for (int m = 0; m < 4; ++m) {
		for (int s = 0; s < 4; ++s) {
			for (int inv = 0; inv < 2; ++inv) {
				INFO("mode " << m << " speed " << s << " invert " << inv);
				saveState(*this, static_cast<ModePWM_DRV>(m), static_cast<SpeedPWM_DRV>(s), inv != 0);
				auto p = makePwm();
				CHECK(p->getMode() == static_cast<ModePWM_DRV>(m));
				CHECK(p->getPwmSpeed() == static_cast<SpeedPWM_DRV>(s));
				CHECK(CommandHelpers::get(*p, "dir") == inv);
				if (m == 0) { // RC: ARR 40000 / 30000 / 20000 / 10000, PSC 84
					CHECK(arr() == kSpeeds[s].arrRc);
					CHECK(psc() == kRcPsc);
				} else {
					CHECK(arr() == kSpeeds[s].arrPwm);
					CHECK(psc() == 0);
				}
			}
		}
	}
}

// ------------------------------------------------------------------ R5.3 empty flash

TEST_CASE_FIXTURE(PwmFixture, "[R5.3 restore] an empty flash gives mode centered, speed LOW and direction not inverted") {
	auto p = makePwm();
	CHECK(p->getMode() == ModePWM_DRV::CENTERED_PWM);
	CHECK(p->getPwmSpeed() == SpeedPWM_DRV::LOW);
	CHECK(CommandHelpers::get(*p, "dir") == 0);
	CHECK(arr() == 56000);
}

TEST_CASE_FIXTURE(PwmFixture, "[R5.3 restore] construction on an empty flash writes nothing to the flash") {
	auto p = makePwm();
	CHECK(HostPlatform::flash().empty());
}

TEST_CASE_FIXTURE(PwmFixture, "[R5.3 restore] reserved bits 8-14 of the stored word are ignored: 0x7F00 gives mode RC, speed LOW, no inversion") {
	HostPlatform::flash()[ADR_PWM_MODE] = 0x7F00;
	auto p = makePwm();
	CHECK(p->getMode() == ModePWM_DRV::RC_PWM);
	CHECK(p->getPwmSpeed() == SpeedPWM_DRV::LOW);
	CHECK(CommandHelpers::get(*p, "dir") == 0);
}

// ------------------------------------------------------------------ R5.4 invalid stored values

TEST_CASE_FIXTURE(PwmFixture, "[R5.4 restore] characterization: a stored speed of 4..7 is ignored, the speed stays LOW while the mode is restored") {
	for (uint16_t s = 4; s <= 7; ++s) {
		INFO("stored speed " << s);
		HostPlatform::flash()[ADR_PWM_MODE] = static_cast<uint16_t>((s << 4) | 2); // mode 2 (PWM/DIR), speed s
		auto p = makePwm();
		CHECK(p->getMode() == ModePWM_DRV::PWM_DIR);
		CHECK(p->getPwmSpeed() == SpeedPWM_DRV::LOW);
		CHECK(arr() == 56000); // 3 kHz of the unchanged speed LOW
		CHECK(psc() == 0);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R5.4 restore] a stored mode above 3 is not taken over as the mode" *
								  KNOWN_ISSUE("B-PWM-3: stored modes 4..7 are accepted into mode (no range check in restoreFlash / setMode)")) {
	for (uint16_t m = 4; m <= 7; ++m) {
		INFO("stored mode " << m);
		HostPlatform::flash()[ADR_PWM_MODE] = m; // speed 0, no inversion
		auto p = makePwm();
		CHECK(static_cast<uint8_t>(p->getMode()) <= 3);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R5.4 restore] a stored mode above 3 leaves the output alive: power +5000 changes CCR1 from 0" *
								  KNOWN_ISSUE("B-PWM-3: with a stored invalid mode turn writes nothing, CCR1 stays 0")) {
	HostPlatform::flash()[ADR_PWM_MODE] = 4; // mode 4, speed LOW, no inversion
	auto p = makePwm();
	p->startMotor();
	p->turn(5000);
	// every valid mode puts a non-zero CCR on channel 1 for +5000 (RC pulse, centered, PWM/DIR duty, dual channel 1)
	CHECK(ccr(1) != 0);
}

// ------------------------------------------------------------------ R5.5 bit 3

TEST_CASE_FIXTURE(PwmFixture, "[R5.5 restore] characterization: bit 3 of the stored word changes the restored mode (0x0009 differs from 0x0001)") {
	// restoreFlash masks the mode with 0xf, saveFlash with 0x7: the value of the resulting mode is not asserted
	HostPlatform::flash()[ADR_PWM_MODE] = 0x0001;
	ModePWM_DRV withoutBit3;
	{
		auto p = makePwm();
		withoutBit3 = p->getMode();
	}
	HostPlatform::flash()[ADR_PWM_MODE] = 0x0009;
	auto p = makePwm();
	CHECK(p->getMode() != withoutBit3);
}

TEST_CASE_FIXTURE(PwmFixture, "[R5.5 restore] the stored word 0x0009 never produces a mode above 3" *
								  KNOWN_ISSUE("B-PWM-3: restoreFlash masks the mode with 0xf, 0x0009 gives mode 9 (no range check)")) {
	HostPlatform::flash()[ADR_PWM_MODE] = 0x0009;
	auto p = makePwm();
	CHECK(static_cast<uint8_t>(p->getMode()) <= 3);
}

TEST_CASE_FIXTURE(PwmFixture, "[R5.5 save] characterization: saveFlash masks the mode with 3 bits, modes 12, 13 and 14 are written as 4, 5 and 6") {
	// 12 & 7 = 4, 13 & 7 = 5, 14 & 7 = 6 (a 2-bit mask would give 0, 1, 2); speed LOW, no inversion
	const std::pair<int, uint16_t> modes[] = {{12, 0x0004}, {13, 0x0005}, {14, 0x0006}};
	for (const auto &m : modes) {
		INFO("mode " << m.first);
		CHECK(saveState(*this, static_cast<ModePWM_DRV>(m.first), SpeedPWM_DRV::LOW, false) == m.second);
	}
}

}
