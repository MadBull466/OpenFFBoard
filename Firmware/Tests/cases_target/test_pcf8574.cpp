/*
 * Tests of PCF8574 / PCF8574Buttons (PCF8574.cpp): I2C port expanders (addresses 0x20.. for each 8 buttons) as
 * button source (ffb_target_tests). The reads run through the real I2CPort (I2C.cpp) on the fake HAL I2C (hi2c1):
 * readButtons() starts an interrupt driven read of the first expander, the receive complete interrupt
 * (HostTarget::i2cComplete) notifies the source's thread, which reads the next expander or publishes the buttons.
 * Expanders are plain I2C devices: a read returns registers[0]. Pins with pull-up read 1 when released, so the
 * buttons are inverted by default.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "HostRtos.h"
#include "TestHelpers.h"

#include "PCF8574.h"
#include "eeprom_addresses.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

extern I2CPort i2cport;

namespace {

struct PcfFixture : TargetFixture {
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

	/** Expander with the 7 bit address 0x20 + index whose input port reads `inputs` */
	static HostTarget::I2cDevice &expander(uint8_t index, uint8_t inputs) {
		HostTarget::I2cDevice &dev = HostTarget::i2cDevice(&hi2c1, 0x20 + index);
		dev.pointerRegister = false;
		dev.setRegister(0, {inputs});
		return dev;
	}
	/**
	 * One complete read cycle of `expanders` expanders: readButtons starts it, each receive interrupt is followed by
	 * the thread (next expander or done). @return the buttons reported by the next readButtons
	 */
	static uint64_t readCycle(PCF8574Buttons &buttons, int expanders) {
		uint64_t value = 0;
		buttons.readButtons(&value);
		for (int i = 0; i < expanders; i++) {
			REQUIRE(HostTarget::i2cTransferPending(&hi2c1));
			REQUIRE(HostTarget::i2cComplete(&hi2c1));
			HostRtos::runUntilBlocked(buttons);
		}
		REQUIRE_FALSE(HostTarget::i2cTransferPending(&hi2c1));
		value = 0;
		buttons.readButtons(&value); // starts the next cycle
		return value;
	}
	/** 7 bit addresses of the receive transfers so far */
	static std::vector<uint8_t> readAddresses() {
		std::vector<uint8_t> addresses;
		for (const HostTarget::I2cTransfer &t : HostTarget::i2cTransfers(&hi2c1)) {
			if (t.call == HostTarget::I2cCall::MasterReceive) {
				addresses.push_back(t.address);
			}
		}
		return addresses;
	}
};

} // namespace

TEST_SUITE("PCF8574") {

TEST_CASE_FIXTURE(PcfFixture, "PCF8574 buttons are identified as I2C PCF8574 with the PCF button class id") {
	PCF8574Buttons buttons;
	CHECK(std::string(buttons.getInfo().name) == "I2C PCF8574");
	CHECK(buttons.getInfo().id == CLSID_BTN_PCF);
	CHECK(PCF8574Buttons::isCreatable());
	CHECK(buttons.getClassType() == ClassType::Buttonsource);
	CHECK(buttons.getHelpstring() == "btnnum/8 devices required. Addresses starting at 0x20.");
}

TEST_CASE_FIXTURE(PcfFixture, "without a stored configuration PCF8574 buttons read 8 inverted buttons at 100 kbit/s") {
	PCF8574Buttons buttons;
	CHECK(get(buttons, "btnnum") == 8);
	CHECK(buttons.getBtnNum() == 8);
	CHECK(get(buttons, "invert") == 1);
	CHECK(get(buttons, "speed") == 0);
	CHECK(hi2c1.Init.ClockSpeed == 100000);
}

TEST_CASE_FIXTURE(PcfFixture, "PCF8574 buttons take the I2C port and release it when deleted") {
	uint32_t deinits = HostTarget::i2cDeInitCount(&hi2c1);
	{
		PCF8574Buttons buttons;
		CHECK(HostTarget::i2cDeInitCount(&hi2c1) == deinits);
	}
	CHECK(HostTarget::i2cDeInitCount(&hi2c1) == deinits + 1); // last user: HAL_I2C_DeInit
}

TEST_CASE_FIXTURE(PcfFixture, "the inputs of one expander are reported inverted") {
	expander(0, 0x5A);
	PCF8574Buttons buttons;

	CHECK(readCycle(buttons, 1) == 0xA5);
	CHECK(readAddresses().front() == 0x20);
}

TEST_CASE_FIXTURE(PcfFixture, "with invert off the inputs are reported as read") {
	expander(0, 0x5A);
	PCF8574Buttons buttons;
	REQUIRE(cmd(buttons, "invert", CMDtype::set, 0) == CommandStatus::OK);
	REQUIRE(get(buttons, "invert") == 0);

	CHECK(readCycle(buttons, 1) == 0x5A);
}

TEST_CASE_FIXTURE(PcfFixture, "16 buttons are read from the expanders 0x20 and 0x21 in that order") {
	expander(0, 0xF0);
	expander(1, 0x3C);
	PCF8574Buttons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 16) == CommandStatus::OK);

	CHECK(readCycle(buttons, 2) == 0xC30F);
	CHECK(readAddresses() == std::vector<uint8_t>{0x20, 0x21, 0x20});
}

TEST_CASE_FIXTURE(PcfFixture, "a button count that is not a multiple of 8 reads the next expander and masks the buttons") {
	expander(0, 0x00);
	expander(1, 0x00);
	PCF8574Buttons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 12) == CommandStatus::OK);

	uint64_t value = readCycle(buttons, 2);

	CHECK(value == 0xFFF);
	CHECK(readAddresses() == std::vector<uint8_t>{0x20, 0x21, 0x20});
}

TEST_CASE_FIXTURE(PcfFixture, "64 buttons are read from 8 expanders") {
	for (uint8_t i = 0; i < 8; i++) {
		expander(i, static_cast<uint8_t>(~(1u << i))); // expander i: pin i pressed
	}
	PCF8574Buttons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 64) == CommandStatus::OK);

	CHECK(readCycle(buttons, 8) == 0x8040201008040201);
	CHECK(readAddresses() == std::vector<uint8_t>{0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x20});
}

TEST_CASE_FIXTURE(PcfFixture, "the buttons change only when the last expander of a cycle was read") {
	expander(0, 0xFE);
	expander(1, 0xFF);
	PCF8574Buttons buttons;
	REQUIRE(cmd(buttons, "invert", CMDtype::set, 0) == CommandStatus::OK);
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 16) == CommandStatus::OK);
	REQUIRE(readCycle(buttons, 2) == 0xFFFE);
	expander(0, 0x00);
	REQUIRE(HostTarget::i2cComplete(&hi2c1)); // first expander of the next cycle
	HostRtos::runUntilBlocked(buttons);

	uint64_t value = 0;
	buttons.readButtons(&value);

	CHECK(value == 0xFFFE); // the half read cycle is not visible
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	HostRtos::runUntilBlocked(buttons);
	value = 0;
	buttons.readButtons(&value);
	CHECK(value == 0xFF00);
}

TEST_CASE_FIXTURE(PcfFixture, "readButtons does not start a read while the previous read is pending") {
	expander(0, 0xFF);
	PCF8574Buttons buttons;
	uint64_t value = 0;
	buttons.readButtons(&value);
	REQUIRE(HostTarget::i2cTransfers(&hi2c1).size() == 1);

	CHECK(buttons.readButtons(&value) == 8);

	CHECK(HostTarget::i2cTransfers(&hi2c1).size() == 1);
}

TEST_CASE_FIXTURE(PcfFixture, "readButtons does not start a read while another transfer uses the I2C port") {
	expander(2, 0xFF);
	PCF8574Buttons buttons;
	uint8_t data = 0x55;
	buttons.writeByteIT(0x22, &data); // another transfer on the bus (I2C state busy)
	REQUIRE(i2cport.isTaken());
	uint64_t value = 0;

	CHECK(buttons.readButtons(&value) == 8);

	CHECK(readAddresses().empty());
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
}

TEST_CASE_FIXTURE(PcfFixture, "a NACK of an expander keeps the last buttons and the next readButtons starts a new cycle") {
	expander(0, 0x0F);
	HostTarget::I2cDevice &second = expander(1, 0xF0);
	PCF8574Buttons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 16) == CommandStatus::OK);
	REQUIRE(readCycle(buttons, 2) == 0x0FF0);
	second.nackAddress = true;
	REQUIRE(HostTarget::i2cComplete(&hi2c1)); // 0x20
	HostRtos::runUntilBlocked(buttons);		  // starts 0x21
	uint32_t inits = HostTarget::i2cInitCount(&hi2c1);

	REQUIRE(HostTarget::i2cComplete(&hi2c1)); // NACK: error callback

	CHECK(HostTarget::i2cInitCount(&hi2c1) == inits + 1); // resetPort
	HostTarget::i2cClearTransfers(&hi2c1);
	uint64_t value = 0;
	buttons.readButtons(&value);
	CHECK(value == 0x0FF0);
	CHECK(readAddresses() == std::vector<uint8_t>{0x20});
	CHECK(HostTarget::i2cTransferPending(&hi2c1));
}

TEST_CASE_FIXTURE(PcfFixture, "a bus error keeps the last buttons and releases the port") {
	expander(0, 0x0F);
	PCF8574Buttons buttons;
	REQUIRE(readCycle(buttons, 1) == 0xF0);

	REQUIRE(HostTarget::i2cFail(&hi2c1, HAL_I2C_ERROR_BERR));

	CHECK_FALSE(i2cport.isTaken());
	uint64_t value = 0;
	buttons.readButtons(&value);
	CHECK(value == 0xF0);
	CHECK(HostTarget::i2cTransferPending(&hi2c1));
}

TEST_CASE_FIXTURE(PcfFixture, "a read that does not complete within 250 ms is aborted and restarted") {
	expander(0, 0xFF);
	PCF8574Buttons buttons;
	uint64_t value = 0;
	buttons.readButtons(&value);
	REQUIRE(HostTarget::i2cTransferPending(&hi2c1));
	uint32_t inits = HostTarget::i2cInitCount(&hi2c1);

	HostPlatform::advanceMs(250);
	buttons.readButtons(&value);
	CHECK(HostTarget::i2cInitCount(&hi2c1) == inits);
	CHECK(HostTarget::i2cTransfers(&hi2c1).size() == 1);

	HostPlatform::advanceMs(1);
	buttons.readButtons(&value);
	CHECK(HostTarget::i2cInitCount(&hi2c1) == inits + 1); // resetPort: HAL_I2C_Init drops the stuck transfer
	CHECK(HostTarget::i2cTransfers(&hi2c1).size() == 2);
	CHECK(HostTarget::i2cTransferPending(&hi2c1));
}

TEST_CASE_FIXTURE(PcfFixture, "a receive completion while no read is active changes nothing") {
	expander(0, 0x12);
	expander(1, 0x34);
	PCF8574Buttons buttons;
	REQUIRE(cmd(buttons, "invert", CMDtype::set, 0) == CommandStatus::OK);
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 16) == CommandStatus::OK);
	REQUIRE(readCycle(buttons, 2) == 0x3412);
	REQUIRE(HostTarget::i2cFail(&hi2c1)); // the next cycle fails: no read active any more
	HostTarget::i2cClearTransfers(&hi2c1);

	{
		HostPlatform::IsrContext isr;
		buttons.i2cRxCompleted(&i2cport);
	}
	HostRtos::runUntilBlocked(buttons);

	CHECK(HostTarget::i2cTransfers(&hi2c1).empty()); // the thread was not notified: no next expander read
	uint64_t value = 0;
	buttons.readButtons(&value);
	CHECK(value == 0x3412);
}

TEST_CASE_FIXTURE(PcfFixture, "a receive completion of another I2C port during a read changes nothing") {
	static const std::array<I2CPortHardwareConfig::I2CPortHardwareConfig_preset, 1> presets{{{hi2c1.Init, "100kb/s"}}};
	static const I2CPortHardwareConfig config(false, presets);
	I2C_HandleTypeDef otherHandle = {};
	I2CPort other(otherHandle, config, 1);
	expander(0, 0x12);
	expander(1, 0x34);
	PCF8574Buttons buttons;
	REQUIRE(cmd(buttons, "invert", CMDtype::set, 0) == CommandStatus::OK);
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 16) == CommandStatus::OK);
	uint64_t value = 0;
	buttons.readButtons(&value); // read of 0x20 pending
	REQUIRE(HostTarget::i2cTransfers(&hi2c1).size() == 1);

	{
		HostPlatform::IsrContext isr;
		buttons.i2cRxCompleted(&other);
	}
	HostRtos::runUntilBlocked(buttons);

	CHECK(HostTarget::i2cTransfers(&hi2c1).size() == 1); // no read of the next expander started
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	HostRtos::runUntilBlocked(buttons);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	HostRtos::runUntilBlocked(buttons);
	value = 0;
	buttons.readButtons(&value);
	CHECK(value == 0x3412);
}

TEST_CASE_FIXTURE(PcfFixture, "characterization: before the first complete read every inverted button reads pressed") {
	// NEEDS REVIEW 81: lastButtons starts at 0 and is inverted (default invert = true), so the first readButtons and
	// every readButtons while the expander does not answer report all buttons pressed (PCF8574.cpp:138-143)
	HostTarget::i2cDevice(&hi2c1, 0x20).nackAddress = true; // no expander
	PCF8574Buttons buttons;
	uint64_t value = 0;
	buttons.readButtons(&value);
	CHECK(value == 0xFF);
	REQUIRE(HostTarget::i2cComplete(&hi2c1)); // NACK
	value = 0;
	buttons.readButtons(&value);
	CHECK(value == 0xFF);
}

// ---------------------------------------------------------------- commands

TEST_CASE_FIXTURE(PcfFixture, "btnnum is limited to 1 to 64 buttons") {
	PCF8574Buttons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(get(buttons, "btnnum") == 1);
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 70) == CommandStatus::OK);
	CHECK(get(buttons, "btnnum") == 64);
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 20) == CommandStatus::OK);
	CHECK(buttons.getBtnNum() == 20);
}

TEST_CASE_FIXTURE(PcfFixture, "a button count of 1 reads one expander and reports one button") {
	expander(0, 0x00);
	PCF8574Buttons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 1) == CommandStatus::OK);
	CHECK(readCycle(buttons, 1) == 0x1);
}

TEST_CASE_FIXTURE(PcfFixture, "a btnnum change reinitialises the I2C port") {
	PCF8574Buttons buttons;
	uint32_t inits = HostTarget::i2cInitCount(&hi2c1);
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 16) == CommandStatus::OK);
	CHECK(HostTarget::i2cInitCount(&hi2c1) == inits + 1);
}

TEST_CASE_FIXTURE(PcfFixture, "speed set selects the 400 kbit/s preset of the I2C port") {
	PCF8574Buttons buttons;

	REQUIRE(cmd(buttons, "speed", CMDtype::set, 1) == CommandStatus::OK);

	CHECK(get(buttons, "speed") == 1);
	CHECK(hi2c1.Init.ClockSpeed == 400000);
	REQUIRE(cmd(buttons, "speed", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(hi2c1.Init.ClockSpeed == 100000);
}

TEST_CASE_FIXTURE(PcfFixture, "speed with a preset beyond the last selects the last preset" *
								  doctest::description("KNOWN ISSUE (crash): I2CPort::setSpeedPreset (I2C.cpp:49) clips the preset to "
													   "presets.size() (2) instead of size()-1, and I2CPortHardwareConfig::getPreset (I2C.h:37) "
													   "reads presets[2] of the 2 element span (same pattern as bugs 37 and 56). Out of bounds: "
													   "aborts the run with _GLIBCXX_ASSERTIONS (GCC 16 -O0). Skipped like the other crash known issues") *
								  doctest::skip()) {
	PCF8574Buttons buttons;

	CHECK(cmd(buttons, "speed", CMDtype::set, 2) == CommandStatus::OK);

	CHECK(get(buttons, "speed") == 1);
	CHECK(hi2c1.Init.ClockSpeed == 400000);
}

TEST_CASE_FIXTURE(PcfFixture, "invert rejects request types it does not support") {
	PCF8574Buttons buttons;
	CHECK(cmd(buttons, "invert", CMDtype::info) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(PcfFixture, "characterization: btnnum and speed return OK without a reply for unsupported request types and btnnum truncates to 8 bits") {
	// NEEDS REVIEW 83: unsupported request types return OK without a reply (PCF8574.cpp:190-207), unlike the
	// ERR of other commands; btnnum takes the value as uint8_t (setBtnNum(uint8_t), PCF8574.cpp:173), so 300 becomes 44
	PCF8574Buttons buttons;
	std::vector<CommandReply> replies;
	CHECK(cmd(buttons, "btnnum", CMDtype::info, 0, &replies) == CommandStatus::OK);
	CHECK(cmd(buttons, "speed", CMDtype::getat, 0, &replies) == CommandStatus::OK);
	CHECK(replies.empty());
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 300) == CommandStatus::OK);
	CHECK(get(buttons, "btnnum") == 44);
}

TEST_CASE_FIXTURE(PcfFixture, "a command id that is not a PCF8574 command is not found") {
	PCF8574Buttons buttons;
	ParsedCommand parsed;
	parsed.cmdId = 0x7f;
	parsed.type = CMDtype::get;
	std::vector<CommandReply> replies;
	CHECK(buttons.command(parsed, replies) == CommandStatus::NOT_FOUND);
}

// ---------------------------------------------------------------- flash

TEST_CASE_FIXTURE(PcfFixture, "saveFlash stores the button count and the invert flag") {
	PCF8574Buttons buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 24) == CommandStatus::OK);
	buttons.saveFlash();
	CHECK(HostPlatform::flash()[ADR_PCFBTN_CONF1] == ((1 << 6) | (24 - 1)));

	REQUIRE(cmd(buttons, "invert", CMDtype::set, 0) == CommandStatus::OK);
	buttons.saveFlash();
	CHECK(HostPlatform::flash()[ADR_PCFBTN_CONF1] == 24 - 1);
}

TEST_CASE_FIXTURE(PcfFixture, "a new source restores the stored button count and invert flag") {
	HostPlatform::flash()[ADR_PCFBTN_CONF1] = 16 - 1; // not inverted
	expander(0, 0x12);
	expander(1, 0x34);
	PCF8574Buttons buttons;

	CHECK(get(buttons, "btnnum") == 16);
	CHECK(get(buttons, "invert") == 0);
	CHECK(readCycle(buttons, 2) == 0x3412);
}

// ---------------------------------------------------------------- PCF8574 base: single byte transfers

TEST_CASE_FIXTURE(PcfFixture, "readByte reads the input port of an expander with a blocking receive") {
	expander(3, 0x77);
	PCF8574Buttons buttons;

	CHECK(buttons.readByte(0x23) == 0x77);

	const HostTarget::I2cTransfer &t = HostTarget::i2cTransfers(&hi2c1).back();
	CHECK(t.call == HostTarget::I2cCall::MasterReceive);
	CHECK(t.mode == HostTarget::BusMode::Blocking);
	CHECK(t.devAddress == (0x23 << 1));
	CHECK(t.size == 1);
	CHECK(t.timeout == 250);
}

TEST_CASE_FIXTURE(PcfFixture, "readByte of a missing expander returns 0") {
	HostTarget::i2cDevice(&hi2c1, 0x24).nackAddress = true;
	PCF8574Buttons buttons;
	CHECK(buttons.readByte(0x24) == 0);
	CHECK(HostTarget::i2cTransfers(&hi2c1).back().result == HAL_ERROR);
}

TEST_CASE_FIXTURE(PcfFixture, "writeByte writes the output byte with a blocking transmit") {
	expander(1, 0xFF);
	PCF8574Buttons buttons;

	buttons.writeByte(0x21, 0x0F);

	const HostTarget::I2cTransfer &t = HostTarget::i2cTransfers(&hi2c1).back();
	CHECK(t.call == HostTarget::I2cCall::MasterTransmit);
	CHECK(t.mode == HostTarget::BusMode::Blocking);
	CHECK(t.devAddress == (0x21 << 1));
	CHECK(t.data == std::vector<uint8_t>{0x0F});
	CHECK(t.timeout == 250);
}

TEST_CASE_FIXTURE(PcfFixture, "writeByteIT writes the output byte with an interrupt driven transmit") {
	expander(2, 0xFF);
	PCF8574Buttons buttons;
	uint8_t data = 0xC3;

	buttons.writeByteIT(0x22, &data);
	REQUIRE(HostTarget::i2cTransferPending(&hi2c1));
	REQUIRE(HostTarget::i2cComplete(&hi2c1));

	const HostTarget::I2cTransfer &t = HostTarget::i2cTransfers(&hi2c1).back();
	CHECK(t.call == HostTarget::I2cCall::MasterTransmit);
	CHECK(t.mode == HostTarget::BusMode::Interrupt);
	CHECK(t.data == std::vector<uint8_t>{0xC3});
	CHECK(t.completed);
	CHECK_FALSE(i2cport.isTaken());
}

} // TEST_SUITE
