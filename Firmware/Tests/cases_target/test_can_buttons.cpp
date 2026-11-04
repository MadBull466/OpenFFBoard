/*
 * Tests of CanButtons (CanButtons.cpp): buttons received in 8 byte CAN frames (ffb_target_tests). Frames reach the
 * source like on the target: HostTarget::canReceive applies the filters of the global `canport` (MockCANPort with
 * the handle &hcan1), queues the frame in the receive FIFO and raises the interrupt, global_callbacks.cpp calls every
 * CanHandler.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "CanButtons.h"
#include "eeprom_addresses.h"

#include <cstdint>
#include <string>
#include <vector>

namespace {

struct CanButtonsFixture : TargetFixture {
	CommandStatus cmd(CommandHandler &handler, const std::string &name, CMDtype type, int64_t val = 0, std::vector<CommandReply> *replies = nullptr) {
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
	int64_t get(CommandHandler &handler, const std::string &name) {
		std::vector<CommandReply> replies;
		CHECK(cmd(handler, name, CMDtype::get, 0, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies.front().val;
	}
	static uint64_t read(CanButtons &buttons) {
		uint64_t value = 0;
		buttons.readButtons(&value);
		return value;
	}
	static const std::vector<uint8_t> ALL_SET;
};
const std::vector<uint8_t> CanButtonsFixture::ALL_SET{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

} // namespace

TEST_SUITE("CanButtons") {

TEST_CASE_FIXTURE(CanButtonsFixture, "CAN buttons are identified as CAN Buttons with the CAN button class id") {
	CanButtons buttons;
	CHECK(std::string(buttons.getInfo().name) == "CAN Buttons");
	CHECK(buttons.getInfo().id == CLSID_BTN_CAN);
	CHECK(static_cast<ButtonSource &>(buttons).getClassType() == ClassType::Buttonsource);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "without a stored configuration CAN buttons report 32 buttons of CAN id 100, not inverted") {
	CanButtons buttons;
	CHECK(get(buttons, "btnnum") == 32);
	CHECK(buttons.getBtnNum() == 32);
	CHECK(get(buttons, "canid") == 100);
	CHECK(get(buttons, "invert") == 0);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "CAN buttons install a filter for their CAN id and start the CAN port") {
	CanButtons buttons;
	MockCANPort &port = HostTarget::canPort();
	CHECK(port.started);
	REQUIRE(port.activeFilters() == 1);
	CHECK(port.filters.back().filter_id == 100);
	CHECK(port.filters.back().filter_mask == 0xFFFF);
	CHECK(port.filters.back().buffer == 0);
	CHECK(port.passesFilter(100));
	CHECK_FALSE(port.passesFilter(101));
}

TEST_CASE_FIXTURE(CanButtonsFixture, "deleting CAN buttons removes their filter") {
	{
		CanButtons buttons;
		REQUIRE(HostTarget::canPort().activeFilters() == 1);
	}
	CHECK(HostTarget::canPort().activeFilters() == 0);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "without a stored configuration all 32 reported buttons are received" *
										  KNOWN_ISSUE("CanButtons.h:50 initialises mask to 0xff while the constructor sets btnnum to 32 "
													  "directly (CanButtons.cpp:22) instead of calling setBtnNum: without a stored "
													  "configuration only buttons 1-8 of the 32 reported buttons work (bug 54). The CHECK fails")) {
	CanButtons buttons;
	REQUIRE(get(buttons, "btnnum") == 32);
	REQUIRE(HostTarget::canReceive(100, ALL_SET) == 1);
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(read(buttons) == 0xFFFFFFFF);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "the 8 data bytes of a frame are the buttons in little endian order") {
	CanButtons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 64) == CommandStatus::OK);

	REQUIRE(HostTarget::canReceive(100, {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF}) == 1);

	CHECK(read(buttons) == 0xEFCDAB8967452301);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "the buttons are masked to the button count") {
	CanButtons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 12) == CommandStatus::OK);
	REQUIRE(HostTarget::canReceive(100, ALL_SET) == 1);

	uint64_t value = 0;
	CHECK(buttons.readButtons(&value) == 12);
	CHECK(value == 0xFFF);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "inverted CAN buttons report the inverted bits masked to the button count") {
	CanButtons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 16) == CommandStatus::OK);
	REQUIRE(cmd(buttons, "invert", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(get(buttons, "invert") == 1);

	REQUIRE(HostTarget::canReceive(100, {0x0F, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}) == 1);

	CHECK(read(buttons) == 0x0FF0);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "readButtons reports the last received frame until the next one arrives") {
	CanButtons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 8) == CommandStatus::OK);
	REQUIRE(HostTarget::canReceive(100, {0x05, 0, 0, 0, 0, 0, 0, 0}) == 1);
	CHECK(read(buttons) == 0x05);
	CHECK(read(buttons) == 0x05);

	REQUIRE(HostTarget::canReceive(100, {0x0A, 0, 0, 0, 0, 0, 0, 0}) == 1);
	CHECK(read(buttons) == 0x0A);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "a frame with another CAN id is not accepted") {
	CanButtons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 8) == CommandStatus::OK);

	CHECK(HostTarget::canReceive(101, ALL_SET) == 0);

	CHECK(read(buttons) == 0);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "a frame with fewer than 8 data bytes is ignored") {
	CanButtons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 8) == CommandStatus::OK);

	REQUIRE(HostTarget::canReceive(100, {0xFF, 0xFF, 0xFF, 0xFF}) == 1);

	CHECK(read(buttons) == 0);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "a remote frame with the CAN id is ignored") {
	CanButtons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 8) == CommandStatus::OK);

	REQUIRE(HostTarget::canReceive(100, ALL_SET, false, true) == 1);

	CHECK(read(buttons) == 0);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "a frame for another CAN handler with another id does not change the buttons") {
	CanButtons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 8) == CommandStatus::OK);
	alignas(8) CAN_rx_msg msg; // the firmware reads the data as uint64_t (CanButtons.cpp:142)
	msg.header.id = 0x101;
	msg.header.length = 8;
	for (uint8_t &b : msg.data) {
		b = 0xFF;
	}

	buttons.canRxPendCallback(&HostTarget::canPort(), msg);

	CHECK(read(buttons) == 0);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "an extended frame whose low 11 bits equal the CAN id is ignored" *
										  KNOWN_ISSUE("(bug 72) canRxPendCallback (CanButtons.cpp:137-138) masks the id to 11 bits and does not check "
													  "header.extId: an extended (29 bit) frame for another CAN handler (every handler sees every "
													  "accepted frame) with id 0x10064 sets the buttons of CAN id 100. The CHECK fails")) {
	CanButtons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 8) == CommandStatus::OK);
	alignas(8) CAN_rx_msg msg;
	msg.header.id = 0x10064;
	msg.header.extId = true;
	msg.header.length = 8;
	for (uint8_t &b : msg.data) {
		b = 0xFF;
	}
	{
		HostPlatform::IsrContext isr;
		buttons.canRxPendCallback(&HostTarget::canPort(), msg);
	}
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(read(buttons) == 0);
}

// ---------------------------------------------------------------- commands

TEST_CASE_FIXTURE(CanButtonsFixture, "btnnum is limited to 1 to 64 buttons") {
	CanButtons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(get(buttons, "btnnum") == 1);
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 65) == CommandStatus::OK);
	CHECK(get(buttons, "btnnum") == 64);
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 40) == CommandStatus::OK);
	CHECK(get(buttons, "btnnum") == 40);
	CHECK(buttons.getBtnNum() == 40);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "a button count of 1 reports only the first bit") {
	CanButtons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(HostTarget::canReceive(100, ALL_SET) == 1);
	CHECK(read(buttons) == 1);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "canid set moves the filter to the new 11 bit id") {
	CanButtons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 8) == CommandStatus::OK);

	REQUIRE(cmd(buttons, "canid", CMDtype::set, 0x8000 | 0x123) == CommandStatus::OK);

	CHECK(get(buttons, "canid") == 0x123);
	MockCANPort &port = HostTarget::canPort();
	CHECK(port.activeFilters() == 1);
	CHECK(port.passesFilter(0x123));
	CHECK_FALSE(port.passesFilter(100));
	CHECK(HostTarget::canReceive(100, ALL_SET) == 0);
	REQUIRE(HostTarget::canReceive(0x123, {0x42, 0, 0, 0, 0, 0, 0, 0}) == 1);
	CHECK(read(buttons) == 0x42);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "characterization: btnnum and canid return OK without a reply for unsupported request types and btnnum truncates to 8 bits") {
	// NEEDS REVIEW 83: unsupported request types return OK without a reply (CanButtons.cpp:109-127), unlike the
	// ERR of other commands; btnnum takes the value as uint8_t (setBtnNum(uint8_t), CanButtons.cpp:84), so 300 becomes 44
	CanButtons buttons;
	std::vector<CommandReply> replies;
	CHECK(cmd(buttons, "btnnum", CMDtype::info, 0, &replies) == CommandStatus::OK);
	CHECK(cmd(buttons, "canid", CMDtype::getat, 0, &replies) == CommandStatus::OK);
	CHECK(replies.empty());
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 300) == CommandStatus::OK);
	CHECK(get(buttons, "btnnum") == 44);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "a command id that is not a CAN button command is not found") {
	CanButtons buttons;
	ParsedCommand parsed;
	parsed.cmdId = 0x7f;
	parsed.type = CMDtype::get;
	std::vector<CommandReply> replies;
	CHECK(buttons.command(parsed, replies) == CommandStatus::NOT_FOUND);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "invert rejects request types it does not support") {
	CanButtons buttons;
	CHECK(cmd(buttons, "invert", CMDtype::info) == CommandStatus::ERR);
}

// ---------------------------------------------------------------- flash

TEST_CASE_FIXTURE(CanButtonsFixture, "saveFlash stores the button count, invert flag and CAN id") {
	CanButtons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 24) == CommandStatus::OK);
	REQUIRE(cmd(buttons, "invert", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(cmd(buttons, "canid", CMDtype::set, 0x7AB) == CommandStatus::OK);

	buttons.saveFlash();

	CHECK(HostPlatform::flash()[ADR_CANBTN_CONF1] == ((1 << 6) | (24 - 1)));
	CHECK(HostPlatform::flash()[ADR_CANBTN_CONF2] == 0x7AB);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "a new source restores the stored configuration and filters the stored CAN id") {
	HostPlatform::flash()[ADR_CANBTN_CONF1] = (1 << 6) | (64 - 1);
	HostPlatform::flash()[ADR_CANBTN_CONF2] = 0xF234; // 11 bits used
	CanButtons buttons;

	CHECK(get(buttons, "btnnum") == 64);
	CHECK(get(buttons, "invert") == 1);
	CHECK(get(buttons, "canid") == 0x234);
	CHECK(HostTarget::canPort().filters.back().filter_id == 0x234);
	REQUIRE(HostTarget::canReceive(0x234, {0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F}) == 1);
	CHECK(read(buttons) == 0x8000000000000001);
}

TEST_CASE_FIXTURE(CanButtonsFixture, "a stored button count of 32 makes all 32 buttons work") {
	HostPlatform::flash()[ADR_CANBTN_CONF1] = 32 - 1;
	CanButtons buttons;
	REQUIRE(HostTarget::canReceive(100, ALL_SET) == 1);
	CHECK(read(buttons) == 0xFFFFFFFF);
}

} // TEST_SUITE
