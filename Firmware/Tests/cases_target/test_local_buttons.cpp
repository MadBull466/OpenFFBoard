/*
 * Tests of LocalButtons (LocalButtons.cpp): the D pins DIN0..DIN7 of the F407 board as button source
 * (ffb_target_tests). The pins are inputs with pull-up on the fake GPIO (HostTarget::setPin); a pressed button pulls
 * its pin low (polarity 0, the default).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "LocalButtons.h"
#include "eeprom_addresses.h"

#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace {

struct LocalButtonsFixture : TargetFixture {
	/**
	 * The command handler of the source as the command system finds it (LocalButtons inherits CommandHandler privately)
	 */
	static CommandHandler &handlerOf(LocalButtons &) {
		CommandHandler *handler = CommandHandler::getHandlerFromClassName("dpin");
		REQUIRE(handler != nullptr);
		return *handler;
	}
	CommandStatus cmd(LocalButtons &source, const std::string &name, CMDtype type, int64_t val = 0, std::vector<CommandReply> *replies = nullptr) {
		CommandHandler &handler = handlerOf(source);
		CmdHandlerCommanddef *def = handler.getCommandFromName(name);
		REQUIRE_MESSAGE(def != nullptr, "command " << name << " not registered");
		ParsedCommand parsed;
		parsed.cmdId = def->cmdId;
		parsed.type = type;
		parsed.val = val;
		parsed.target = &handler;
		std::vector<CommandReply> local;
		return handler.command(parsed, replies ? *replies : local);
	}
	int64_t get(LocalButtons &handler, const std::string &name) {
		std::vector<CommandReply> replies;
		CHECK(cmd(handler, name, CMDtype::get, 0, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies.front().val;
	}

	/** DIN pin n (0..7) at the given level */
	static void setDin(int n, bool high) {
		static const std::array<std::pair<GPIO_TypeDef *, uint16_t>, 8> pins{{{DIN0_GPIO_Port, DIN0_Pin}, {DIN1_GPIO_Port, DIN1_Pin},
																			   {DIN2_GPIO_Port, DIN2_Pin}, {DIN3_GPIO_Port, DIN3_Pin},
																			   {DIN4_GPIO_Port, DIN4_Pin}, {DIN5_GPIO_Port, DIN5_Pin},
																			   {DIN6_GPIO_Port, DIN6_Pin}, {DIN7_GPIO_Port, DIN7_Pin}}};
		HostTarget::setPin(pins.at(n).first, pins.at(n).second, high);
	}
	static void press(int n) { setDin(n, false); }
	static void release(int n) { setDin(n, true); }

	static uint64_t read(LocalButtons &buttons) {
		uint64_t value = 0;
		buttons.readButtons(&value);
		return value;
	}
};

} // namespace

TEST_SUITE("LocalButtons") {

TEST_CASE_FIXTURE(LocalButtonsFixture, "local buttons are identified as D-Pins with the local button class id") {
	LocalButtons buttons;
	CHECK(std::string(buttons.getInfo().name) == "D-Pins");
	CHECK(buttons.getInfo().id == CLSID_BTN_LOCAL);
	CHECK(LocalButtons::isCreatable());
	CHECK(buttons.getClassType() == ClassType::Buttonsource);
	CHECK(buttons.getHelpstring() == "Digital pin button source");
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "without a stored configuration all 8 pins are enabled with low active polarity and no pulse mode") {
	LocalButtons buttons;
	CHECK(get(buttons, "mask") == 0xff);
	CHECK(get(buttons, "polarity") == 0);
	CHECK(get(buttons, "pulse") == 0);
	CHECK(buttons.getBtnNum() == 8);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "no button is reported while all pins are high") {
	LocalButtons buttons;
	CHECK(read(buttons) == 0);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "readButtons returns the number of enabled pins") {
	LocalButtons buttons;
	REQUIRE(cmd(buttons, "mask", CMDtype::set, 0b10110) == CommandStatus::OK);
	uint64_t value = 0;
	CHECK(buttons.readButtons(&value) == 3);
	CHECK(buttons.getBtnNum() == 3);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "the buttons of the enabled pins are packed without gaps in pin order") {
	LocalButtons buttons;
	REQUIRE(cmd(buttons, "mask", CMDtype::set, 0b10100100) == CommandStatus::OK); // pins 2, 5, 7
	press(2);
	press(7);
	press(3); // not enabled

	CHECK(read(buttons) == 0b101);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "mask bits above the available pins are ignored") {
	LocalButtons buttons;
	REQUIRE(cmd(buttons, "mask", CMDtype::set, 0xff01) == CommandStatus::OK);
	CHECK(buttons.getBtnNum() == 1);
	press(0);
	CHECK(read(buttons) == 0b1);
	CHECK(get(buttons, "mask") == 0xff01);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "positive polarity reports the pins that are high") {
	LocalButtons buttons;
	REQUIRE(cmd(buttons, "polarity", CMDtype::set, 1) == CommandStatus::OK);
	for (int i = 0; i < 8; i++) {
		press(i); // low
	}
	release(1);
	release(6);

	CHECK(read(buttons) == 0b01000010);
	CHECK(get(buttons, "polarity") == 1);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "readButtons ORs the buttons into the given value") {
	LocalButtons buttons;
	press(0);
	uint64_t value = 0x100;
	buttons.readButtons(&value);
	CHECK(value == 0x101);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "values returns the current buttons of the enabled pins") {
	LocalButtons buttons;
	REQUIRE(cmd(buttons, "mask", CMDtype::set, 0b1100) == CommandStatus::OK);
	press(3);
	CHECK(get(buttons, "values") == 0b10);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "pins returns the number of available pins") {
	LocalButtons buttons;
	CHECK(get(buttons, "pins") == 8);
	CHECK(LocalButtons::maxButtons == 8);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "readButton reads the level of a pin and is false outside the available pins") {
	press(4);
	CHECK(LocalButtons::readButton(3) == true);
	CHECK(LocalButtons::readButton(4) == false);
	CHECK(LocalButtons::readButton(8) == false);
	CHECK(LocalButtons::readButton(-1) == false);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "commands reject request types they do not support") {
	LocalButtons buttons;
	CHECK(cmd(buttons, "mask", CMDtype::info) == CommandStatus::ERR);
	CHECK(cmd(buttons, "polarity", CMDtype::info) == CommandStatus::ERR);
	CHECK(cmd(buttons, "pins", CMDtype::set, 4) == CommandStatus::ERR);
	CHECK(cmd(buttons, "values", CMDtype::set, 1) == CommandStatus::ERR);
	CHECK(cmd(buttons, "pulse", CMDtype::info) == CommandStatus::ERR);
	CHECK(get(buttons, "pins") == 8);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "a command id that is not a local button command is not found") {
	LocalButtons buttons;
	ParsedCommand parsed;
	parsed.cmdId = 0x7f;
	parsed.type = CMDtype::get;
	std::vector<CommandReply> replies;
	CHECK(handlerOf(buttons).command(parsed, replies) == CommandStatus::NOT_FOUND);
}

// ---------------------------------------------------------------- pulse mode

TEST_CASE_FIXTURE(LocalButtonsFixture, "a button in pulse mode reports a press as a pulse that ends 50 ms later") {
	LocalButtons buttons;
	REQUIRE(cmd(buttons, "pulse", CMDtype::set, 0b1) == CommandStatus::OK);
	HostPlatform::setTimeMs(1000);
	REQUIRE(read(buttons) == 0);

	press(0);
	CHECK(read(buttons) == 0b1); // edge: pulse starts
	HostPlatform::advanceMs(50);
	CHECK(read(buttons) == 0b1); // held for 50 ms
	HostPlatform::advanceMs(1);
	CHECK(read(buttons) == 0); // ends although the pin stays low
	HostPlatform::advanceMs(100);
	CHECK(read(buttons) == 0);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "a button in pulse mode reports a release as another pulse") {
	LocalButtons buttons;
	REQUIRE(cmd(buttons, "pulse", CMDtype::set, 0b1) == CommandStatus::OK);
	HostPlatform::setTimeMs(1000);
	press(0);
	read(buttons);
	HostPlatform::advanceMs(100);
	REQUIRE(read(buttons) == 0);

	release(0);
	CHECK(read(buttons) == 0b1);
	HostPlatform::advanceMs(51);
	CHECK(read(buttons) == 0);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "buttons outside the pulse mask are reported directly while another button pulses") {
	LocalButtons buttons;
	REQUIRE(cmd(buttons, "pulse", CMDtype::set, 0b1) == CommandStatus::OK);
	HostPlatform::setTimeMs(1000);
	press(0);
	REQUIRE(read(buttons) == 0b1);

	press(1);
	HostPlatform::advanceMs(10);
	CHECK(read(buttons) == 0b11);
	release(1);
	HostPlatform::advanceMs(10);
	CHECK(read(buttons) == 0b01);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "characterization: a pulse ends early when another pulse mode button changes") {
	// NEEDS REVIEW 80: on an edge of any pulse button only the new edges are output (LocalButtons.cpp:82-86); the
	// held pulse of the first button (lastOutputs) is dropped after 10 ms instead of 50 ms
	LocalButtons buttons;
	REQUIRE(cmd(buttons, "pulse", CMDtype::set, 0b11) == CommandStatus::OK);
	HostPlatform::setTimeMs(1000);
	press(0);
	REQUIRE(read(buttons) == 0b01);

	HostPlatform::advanceMs(10);
	press(1);

	CHECK(read(buttons) == 0b10);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "pulse returns the pulse mask") {
	LocalButtons buttons;
	REQUIRE(cmd(buttons, "pulse", CMDtype::set, 0x81) == CommandStatus::OK);
	CHECK(get(buttons, "pulse") == 0x81);
}

// ---------------------------------------------------------------- flash

TEST_CASE_FIXTURE(LocalButtonsFixture, "saveFlash stores the mask, the polarity and the pulse mask") {
	LocalButtons buttons;
	REQUIRE(cmd(buttons, "mask", CMDtype::set, 0x1234F) == CommandStatus::OK);
	REQUIRE(cmd(buttons, "polarity", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(cmd(buttons, "pulse", CMDtype::set, 0x10003) == CommandStatus::OK);

	buttons.saveFlash();

	CHECK(HostPlatform::flash()[ADR_LOCAL_BTN_CONF] == 0x234F); // 16 bits
	CHECK(HostPlatform::flash()[ADR_LOCAL_BTN_CONF_2] == 1);
	CHECK(HostPlatform::flash()[ADR_LOCAL_BTN_CONF_3] == 0x0003);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "a new source restores the stored mask, polarity and pulse mask") {
	HostPlatform::flash()[ADR_LOCAL_BTN_CONF] = 0x0F;
	HostPlatform::flash()[ADR_LOCAL_BTN_CONF_2] = 1;
	HostPlatform::flash()[ADR_LOCAL_BTN_CONF_3] = 0x02;
	LocalButtons buttons;

	CHECK(get(buttons, "mask") == 0x0F);
	CHECK(buttons.getBtnNum() == 4);
	CHECK(get(buttons, "polarity") == 1);
	CHECK(get(buttons, "pulse") == 0x02);
}

TEST_CASE_FIXTURE(LocalButtonsFixture, "a stored polarity uses only bit 0") {
	HostPlatform::flash()[ADR_LOCAL_BTN_CONF_2] = 0xFFFE;
	LocalButtons buttons;
	CHECK(get(buttons, "polarity") == 0);
}

} // TEST_SUITE
