/*
 * Tests of ShifterAnalog (ShifterAnalog.cpp): Logitech G29/G27 analog H-pattern and sequential shifter as button
 * source (ffb_target_tests). X and Y come from the ADC1 DMA buffer (fake HAL ADC, 12 bit values made 16 bit); the
 * G29 reverse signal is a D pin (fake GPIO), the G27 reverse and user buttons are read from a shift register on the
 * external SPI port (fake HAL SPI, hspi2).
 * Default channels: X = analog pin 6 (ADC1 rank 5), Y = analog pin 5 (rank 4). Default thresholds (G29):
 * X_12 25600, X_56 40000, Y_135 51200, Y_246 13600.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "ShifterAnalog.h"
#include "eeprom_addresses.h"
#include "global_callbacks.h"

#include <cstdint>
#include <string>
#include <vector>

extern SPIPort external_spi;

namespace {

struct ShifterFixture : TargetFixture {
	ShifterFixture() {
		startADC(); // HAL_ADC_Start_DMA into ADC1_BUF like cppmain
		setReverse(false);
	}

	/**
	 * The command handler of the source as the command system finds it (ShifterAnalog inherits CommandHandler privately)
	 */
	static CommandHandler &handlerOf(ShifterAnalog &) {
		CommandHandler *handler = CommandHandler::getHandlerFromClassName("shifter");
		REQUIRE(handler != nullptr);
		return *handler;
	}
	CommandStatus cmd(ShifterAnalog &source, const std::string &name, CMDtype type, int64_t val = 0, std::vector<CommandReply> *replies = nullptr) {
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
	int64_t get(ShifterAnalog &handler, const std::string &name) {
		std::vector<CommandReply> replies;
		CHECK(cmd(handler, name, CMDtype::get, 0, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies.front().val;
	}

	/** Analog pin values (16 bit scale, multiples of 16) of the 6 analog pins; ranks 6/7 (temperature, VREFINT) fixed */
	void setAnalogPins(const std::vector<uint16_t> &pins) {
		std::vector<uint32_t> raw;
		for (uint16_t v : pins) {
			raw.push_back(v >> 4); // 12 bit ADC
		}
		raw.resize(6, 0);
		raw.push_back(1000);
		raw.push_back(1500);
		REQUIRE(HostTarget::adcConvert(&hadc1, raw));
	}
	/** Shifter position with the default channels: X on analog pin 6, Y on analog pin 5 */
	void setXY(uint16_t x, uint16_t y) { setAnalogPins({0, 0, 0, 0, y, x}); }
	/** G29 reverse signal on DIN0 (revbtn 1): high = reverse pressed */
	static void setReverse(bool high) { HostTarget::setPin(DIN0_GPIO_Port, DIN0_Pin, high); }

	static uint64_t read(ShifterAnalog &shifter) {
		uint64_t value = 0xDEAD; // readButtons overwrites the value
		shifter.readButtons(&value);
		return value;
	}
	/** Buttons in the given position */
	uint64_t buttonsAt(ShifterAnalog &shifter, uint16_t x, uint16_t y) {
		setXY(x, y);
		return read(shifter);
	}

	static constexpr int64_t G29_H = 0, G29_SEQ = 1, G27_H = 2, G27_SEQ = 3;
	// Positions (16 bit) left / middle / right and up / centre / down of the default thresholds
	static constexpr uint16_t LEFT = 10000, MIDDLE = 32000, RIGHT = 50000;
	static constexpr uint16_t UP = 60000, CENTRE = 32000, DOWN = 4992;
};

} // namespace

TEST_SUITE("ShifterAnalog") {

TEST_CASE_FIXTURE(ShifterFixture, "the shifter is identified as Shifter Analog with the shifter class id") {
	ShifterAnalog shifter;
	CHECK(std::string(shifter.getInfo().name) == "Shifter Analog");
	CHECK(shifter.getInfo().id == CLSID_BTN_SHIFTER);
	CHECK(ShifterAnalog::isCreatable());
	CHECK(shifter.getClassType() == ClassType::Buttonsource);
	CHECK(shifter.getHelpstring() == "Analog 6+1 gear shifter button source");
}

TEST_CASE_FIXTURE(ShifterFixture, "without a stored configuration the shifter is a G29 H-pattern shifter with the G29 thresholds") {
	ShifterAnalog shifter;
	CHECK(get(shifter, "mode") == G29_H);
	CHECK(get(shifter, "x12") == 25600);
	CHECK(get(shifter, "x56") == 40000);
	CHECK(get(shifter, "y135") == 51200);
	CHECK(get(shifter, "y246") == 13600);
	CHECK(get(shifter, "xchan") == 6);
	CHECK(get(shifter, "ychan") == 5);
	CHECK(get(shifter, "revbtn") == 1);
	CHECK(get(shifter, "cspin") == 1);
	CHECK(shifter.getBtnNum() == 7);
}

// ---------------------------------------------------------------- H-pattern

TEST_CASE_FIXTURE(ShifterFixture, "the H-pattern positions select the buttons of gears 1 to 6") {
	ShifterAnalog shifter;
	CHECK(buttonsAt(shifter, LEFT, UP) == 1 << 0);
	CHECK(buttonsAt(shifter, LEFT, DOWN) == 1 << 1);
	CHECK(buttonsAt(shifter, MIDDLE, UP) == 1 << 2);
	CHECK(buttonsAt(shifter, MIDDLE, DOWN) == 1 << 3);
	CHECK(buttonsAt(shifter, RIGHT, UP) == 1 << 4);
	CHECK(buttonsAt(shifter, RIGHT, DOWN) == 1 << 5);
}

TEST_CASE_FIXTURE(ShifterFixture, "the centre row of the H-pattern is neutral") {
	ShifterAnalog shifter;
	CHECK(buttonsAt(shifter, LEFT, CENTRE) == 0);
	CHECK(buttonsAt(shifter, MIDDLE, CENTRE) == 0);
	CHECK(buttonsAt(shifter, RIGHT, CENTRE) == 0);
}

TEST_CASE_FIXTURE(ShifterFixture, "positions exactly on a threshold belong to the middle column and the neutral row") {
	ShifterAnalog shifter;
	CHECK(buttonsAt(shifter, 25600, UP) == 1 << 2);	 // x == X_12: gear 3, not 1
	CHECK(buttonsAt(shifter, 40000, DOWN) == 1 << 3); // x == X_56: gear 4, not 6
	CHECK(buttonsAt(shifter, LEFT, 51200) == 0);	 // y == Y_135: neutral
	CHECK(buttonsAt(shifter, LEFT, 13600) == 0);	 // y == Y_246: neutral
}

TEST_CASE_FIXTURE(ShifterFixture, "the reverse signal in the sixth gear position selects reverse instead of gear 6") {
	ShifterAnalog shifter;
	setReverse(true);
	CHECK(buttonsAt(shifter, RIGHT, DOWN) == 1 << 6);
}

TEST_CASE_FIXTURE(ShifterFixture, "the reverse signal has no effect in the other gear positions") {
	ShifterAnalog shifter;
	setReverse(true);
	CHECK(buttonsAt(shifter, RIGHT, UP) == 1 << 4);
	CHECK(buttonsAt(shifter, LEFT, DOWN) == 1 << 1);
	CHECK(buttonsAt(shifter, MIDDLE, CENTRE) == 0);
}

TEST_CASE_FIXTURE(ShifterFixture, "revbtn selects the D pin of the reverse signal") {
	ShifterAnalog shifter;
	REQUIRE(cmd(shifter, "revbtn", CMDtype::set, 3) == CommandStatus::OK);
	setReverse(true); // DIN0: no longer the reverse signal
	HostTarget::setPin(DIN2_GPIO_Port, DIN2_Pin, false);
	CHECK(buttonsAt(shifter, RIGHT, DOWN) == 1 << 5);

	HostTarget::setPin(DIN2_GPIO_Port, DIN2_Pin, true);
	CHECK(buttonsAt(shifter, RIGHT, DOWN) == 1 << 6);
	CHECK(get(shifter, "revbtn") == 3);
}

TEST_CASE_FIXTURE(ShifterFixture, "readButtons returns the 7 buttons of the H-pattern") {
	ShifterAnalog shifter;
	setXY(LEFT, UP);
	uint64_t value = 0;
	CHECK(shifter.readButtons(&value) == 7);
}

// ---------------------------------------------------------------- sequential

TEST_CASE_FIXTURE(ShifterFixture, "in sequential mode left is button 1, right is button 2 and the middle is none") {
	ShifterAnalog shifter;
	REQUIRE(cmd(shifter, "mode", CMDtype::set, G29_SEQ) == CommandStatus::OK);
	CHECK(shifter.getBtnNum() == 2);
	CHECK(buttonsAt(shifter, LEFT, CENTRE) == 1 << 0);
	CHECK(buttonsAt(shifter, RIGHT, CENTRE) == 1 << 1);
	CHECK(buttonsAt(shifter, MIDDLE, CENTRE) == 0);
	CHECK(buttonsAt(shifter, 25600, CENTRE) == 0);
	CHECK(buttonsAt(shifter, 40000, CENTRE) == 0);
}

TEST_CASE_FIXTURE(ShifterFixture, "in sequential mode the y position and the reverse signal are ignored") {
	ShifterAnalog shifter;
	REQUIRE(cmd(shifter, "mode", CMDtype::set, G29_SEQ) == CommandStatus::OK);
	setReverse(true);
	CHECK(buttonsAt(shifter, RIGHT, DOWN) == 1 << 1);
	CHECK(buttonsAt(shifter, LEFT, UP) == 1 << 0);
}

TEST_CASE_FIXTURE(ShifterFixture, "characterization: an unknown mode reports no buttons") {
	// NEEDS REVIEW 82: the mode command (and restoreFlash) accept any value (ShifterAnalog.cpp:172, 223); an unknown
	// mode is kept, saved and reports 0 buttons. Should it be rejected like other invalid values?
	ShifterAnalog shifter;
	REQUIRE(cmd(shifter, "mode", CMDtype::set, 7) == CommandStatus::OK);
	CHECK(shifter.getBtnNum() == 0);
	CHECK(buttonsAt(shifter, LEFT, UP) == 0);
	CHECK(get(shifter, "gear") == 0);
}

// ---------------------------------------------------------------- thresholds, channels, values

TEST_CASE_FIXTURE(ShifterFixture, "the threshold commands move the gear boundaries") {
	ShifterAnalog shifter;
	REQUIRE(cmd(shifter, "x12", CMDtype::set, 5000) == CommandStatus::OK);
	REQUIRE(cmd(shifter, "x56", CMDtype::set, 60000) == CommandStatus::OK);
	REQUIRE(cmd(shifter, "y135", CMDtype::set, 61000) == CommandStatus::OK);
	REQUIRE(cmd(shifter, "y246", CMDtype::set, 1000) == CommandStatus::OK);

	CHECK(get(shifter, "x12") == 5000);
	CHECK(get(shifter, "x56") == 60000);
	CHECK(get(shifter, "y135") == 61000);
	CHECK(get(shifter, "y246") == 1000);
	CHECK(buttonsAt(shifter, LEFT, UP) == 0);		  // now middle column, neutral row
	CHECK(buttonsAt(shifter, LEFT, 62000) == 1 << 2); // gear 3
	CHECK(buttonsAt(shifter, 4000, 62000) == 1 << 0);
	CHECK(buttonsAt(shifter, 61008, 992) == 1 << 5);
}

TEST_CASE_FIXTURE(ShifterFixture, "xchan and ychan select the analog pins of the X and Y signal") {
	ShifterAnalog shifter;
	REQUIRE(cmd(shifter, "xchan", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(cmd(shifter, "ychan", CMDtype::set, 2) == CommandStatus::OK);
	CHECK(get(shifter, "xchan") == 1);
	CHECK(get(shifter, "ychan") == 2);

	setAnalogPins({RIGHT, UP, 0, 0, DOWN, LEFT}); // the default channels would give gear 2

	CHECK(read(shifter) == 1 << 4); // gear 5
}

TEST_CASE_FIXTURE(ShifterFixture, "vals returns the 16 bit X and Y values of the last read") {
	ShifterAnalog shifter;
	setXY(0x1230, 0xABC0);
	read(shifter);

	std::vector<CommandReply> replies;
	REQUIRE(cmd(shifter, "vals", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 0x1230);
	CHECK(replies[0].adr == 0xABC0);
}

TEST_CASE_FIXTURE(ShifterFixture, "gear returns the gear decoded by the last read") {
	ShifterAnalog shifter;
	setReverse(true);
	buttonsAt(shifter, RIGHT, DOWN);
	CHECK(get(shifter, "gear") == 7);
	buttonsAt(shifter, MIDDLE, UP);
	CHECK(get(shifter, "gear") == 3);
}

TEST_CASE_FIXTURE(ShifterFixture, "mode info lists the modes with their SPI and local reverse flags") {
	ShifterAnalog shifter;
	std::vector<CommandReply> replies;
	REQUIRE(cmd(shifter, "mode", CMDtype::info, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "G29-H:0,0,1\nG29 Sequential:1,0,1\nG27-H:2,1,0\nG27 Sequential:3,1,0\n");
}

TEST_CASE_FIXTURE(ShifterFixture, "characterization: mode returns OK without a reply for request types it does not support") {
	// NEEDS REVIEW 83: CanButtons btnnum/canid, PCF8574Buttons btnnum/speed and ShifterAnalog mode return OK with no
	// reply for unsupported request types (ShifterAnalog.cpp:221-229 has no else branch), while other commands return
	// ERR; CanButtons / PCF8574Buttons btnnum truncate the value to uint8_t before limiting it
	ShifterAnalog shifter;
	std::vector<CommandReply> replies;
	CHECK(cmd(shifter, "mode", CMDtype::getat, 0, &replies) == CommandStatus::OK);
	CHECK(replies.empty());
	CHECK(cmd(shifter, "mode", CMDtype::setat, G29_SEQ) == CommandStatus::OK);
	CHECK(get(shifter, "mode") == G29_H);
}

TEST_CASE_FIXTURE(ShifterFixture, "commands reject request types they do not support") {
	ShifterAnalog shifter;
	CHECK(cmd(shifter, "x12", CMDtype::info) == CommandStatus::ERR);
	CHECK(cmd(shifter, "vals", CMDtype::set, 1) == CommandStatus::ERR);
	CHECK(cmd(shifter, "gear", CMDtype::set, 1) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(ShifterFixture, "a command id that is not a shifter command is not found") {
	ShifterAnalog shifter;
	ParsedCommand parsed;
	parsed.cmdId = 0x7f;
	parsed.type = CMDtype::get;
	std::vector<CommandReply> replies;
	CHECK(handlerOf(shifter).command(parsed, replies) == CommandStatus::NOT_FOUND);
}

TEST_CASE_FIXTURE(ShifterFixture, "xchan accepts only the analog pins 1 to 6" *
									   KNOWN_ISSUE("xchan / ychan store any value (ShifterAnalog.cpp:243-246, handleGetSet), and updateAdc "
												   "indexes the ADC1 DMA buffer with x_chan-1 unchecked (ShifterAnalog.cpp:55-56): channel 0 "
												   "reads ADC1_BUF[-1], channels 7/8 read the temperature and VREFINT ranks, 9 and above read "
												   "past the 8 element buffer. The CHECK after setting 0 fails")) {
	ShifterAnalog shifter;
	cmd(shifter, "xchan", CMDtype::set, 0); // not read afterwards: the out of bounds read is not executed
	REQUIRE_ALL_HAL_MODELLED();

	int64_t channel = get(shifter, "xchan");
	CHECK((channel >= 1 && channel <= 6));
}

// ---------------------------------------------------------------- G27 (SPI buttons)

TEST_CASE_FIXTURE(ShifterFixture, "G27 modes report the 12 user buttons before the gear buttons") {
	ShifterAnalog shifter;
	REQUIRE(cmd(shifter, "mode", CMDtype::set, G27_H) == CommandStatus::OK);
	CHECK(shifter.getBtnNum() == 19);
	REQUIRE(cmd(shifter, "mode", CMDtype::set, G27_SEQ) == CommandStatus::OK);
	CHECK(shifter.getBtnNum() == 14);
}

TEST_CASE_FIXTURE(ShifterFixture, "a G27 mode reserves the first free CS pin of the external SPI port and a G29 mode frees it") {
	ShifterAnalog shifter;
	REQUIRE(cmd(shifter, "mode", CMDtype::set, G27_H) == CommandStatus::OK);
	CHECK_FALSE(external_spi.isPinFree(external_spi.getCsPins()[0]));
	REQUIRE(cmd(shifter, "mode", CMDtype::set, G27_SEQ) == CommandStatus::OK); // keeps the client
	CHECK(external_spi.getFreeCsPins().size() == 2);

	REQUIRE(cmd(shifter, "mode", CMDtype::set, G29_H) == CommandStatus::OK);

	CHECK(external_spi.getFreeCsPins().size() == 3);
}

TEST_CASE_FIXTURE(ShifterFixture, "a G27 read receives 2 bytes LSB first at prescaler 64 with clock idle low and an active low CS") {
	ShifterAnalog shifter;
	REQUIRE(cmd(shifter, "mode", CMDtype::set, G27_H) == CommandStatus::OK);
	setXY(MIDDLE, CENTRE);

	read(shifter);

	REQUIRE(HostTarget::spiTransferPending(&hspi2));
	const HostTarget::SpiTransfer &transfer = HostTarget::spiTransfers(&hspi2).front();
	CHECK(transfer.call == HostTarget::SpiCall::Receive);
	CHECK(transfer.mode == HostTarget::BusMode::Dma);
	CHECK(transfer.size == 2);
	CHECK(transfer.init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_64);
	CHECK(transfer.init.FirstBit == SPI_FIRSTBIT_LSB);
	CHECK(transfer.init.CLKPolarity == SPI_POLARITY_LOW);
	CHECK(transfer.init.CLKPhase == SPI_PHASE_1EDGE);
	CHECK(transfer.csAtStart.at(0) == false);
	REQUIRE(HostTarget::spiComplete(&hspi2));
	CHECK(HostTarget::outputPin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin) == true);
}

TEST_CASE_FIXTURE(ShifterFixture, "a G27 shifter reports the user buttons and the gear of the previous SPI read") {
	ShifterAnalog shifter;
	REQUIRE(cmd(shifter, "mode", CMDtype::set, G27_H) == CommandStatus::OK);
	setXY(RIGHT, DOWN);
	CHECK(read(shifter) == 1 << (12 + 5)); // nothing received yet: no user buttons, no reverse: gear 6
	REQUIRE(HostTarget::spiTransferPending(&hspi2));
	HostTarget::spiQueueRx(&hspi2, {0x52, 0x34}); // bits 4..15 user buttons 0x345, bit 1 reverse

	REQUIRE(HostTarget::spiComplete(&hspi2));
	uint64_t value = read(shifter);

	CHECK(value == (0x345 | (1 << (12 + 6)))); // reverse (gear 7) after the 12 user buttons
	REQUIRE(HostTarget::spiComplete(&hspi2));
}

TEST_CASE_FIXTURE(ShifterFixture, "a G27 sequential shifter reports the user buttons and then the 2 sequential buttons") {
	ShifterAnalog shifter;
	REQUIRE(cmd(shifter, "mode", CMDtype::set, G27_SEQ) == CommandStatus::OK);
	setXY(RIGHT, CENTRE);
	CHECK(read(shifter) == 1 << (12 + 1)); // nothing received yet: gear 2 after the 12 user buttons
	HostTarget::spiQueueRx(&hspi2, {0x50, 0x34}); // user buttons 0x345 in bits 4..15
	REQUIRE(HostTarget::spiComplete(&hspi2));

	CHECK(read(shifter) == (0x345 | (1 << 13)));
	REQUIRE(HostTarget::spiComplete(&hspi2));
}

TEST_CASE_FIXTURE(ShifterFixture, "a G27 shifter ignores the local reverse pin") {
	ShifterAnalog shifter;
	REQUIRE(cmd(shifter, "mode", CMDtype::set, G27_SEQ) == CommandStatus::OK);
	REQUIRE(cmd(shifter, "mode", CMDtype::set, G27_H) == CommandStatus::OK);
	setReverse(true);

	CHECK(buttonsAt(shifter, RIGHT, DOWN) == 1 << (12 + 5));
	REQUIRE(HostTarget::spiComplete(&hspi2));
}

TEST_CASE_FIXTURE(ShifterFixture, "cspin selects the CS pin of the G27 button reads" *
									   KNOWN_ISSUE("The cspin command only stores cs_pin_num (ShifterAnalog.cpp:241-242, handleGetSet); "
												   "setCSPin (ShifterAnalog.cpp:204-214) is never called and the G27 client always uses the "
												   "first free CS pin (line 197). The CHECK on the selected CS pin fails")) {
	ShifterAnalog shifter;
	REQUIRE(cmd(shifter, "mode", CMDtype::set, G27_H) == CommandStatus::OK);
	REQUIRE(cmd(shifter, "cspin", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE(get(shifter, "cspin") == 2);
	for (OutputPin &pin : external_spi.getCsPins()) {
		pin.write(true); // all CS pins inactive (high) before the read
	}
	setXY(MIDDLE, CENTRE);
	read(shifter);
	REQUIRE(HostTarget::spiTransferPending(&hspi2));
	int selected = HostTarget::spiTransfers(&hspi2).front().selectedCs(true);
	REQUIRE(HostTarget::spiComplete(&hspi2));
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(selected > 0); // -1 would mean no or several CS pins asserted
}

// ---------------------------------------------------------------- flash

TEST_CASE_FIXTURE(ShifterFixture, "saveFlash stores the mode, channels, reverse pin, CS pin and thresholds") {
	ShifterAnalog shifter;
	REQUIRE(cmd(shifter, "mode", CMDtype::set, G29_SEQ) == CommandStatus::OK);
	REQUIRE(cmd(shifter, "xchan", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE(cmd(shifter, "ychan", CMDtype::set, 3) == CommandStatus::OK);
	REQUIRE(cmd(shifter, "revbtn", CMDtype::set, 4) == CommandStatus::OK);
	REQUIRE(cmd(shifter, "cspin", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE(cmd(shifter, "x12", CMDtype::set, 1111) == CommandStatus::OK);
	REQUIRE(cmd(shifter, "x56", CMDtype::set, 2222) == CommandStatus::OK);
	REQUIRE(cmd(shifter, "y135", CMDtype::set, 3333) == CommandStatus::OK);
	REQUIRE(cmd(shifter, "y246", CMDtype::set, 4444) == CommandStatus::OK);

	shifter.saveFlash();

	auto &flash = HostPlatform::flash();
	CHECK(flash[ADR_SHIFTERANALOG_CONF] == G29_SEQ);
	CHECK(flash[ADR_SHIFTERANALOG_CONF_2] == 0x0203);
	CHECK(flash[ADR_SHIFTERANALOG_CONF_3] == 0x0402);
	CHECK(flash[ADR_SHIFTERANALOG_X_12] == 1111);
	CHECK(flash[ADR_SHIFTERANALOG_X_56] == 2222);
	CHECK(flash[ADR_SHIFTERANALOG_Y_135] == 3333);
	CHECK(flash[ADR_SHIFTERANALOG_Y_246] == 4444);
}

TEST_CASE_FIXTURE(ShifterFixture, "a new shifter restores the stored configuration") {
	auto &flash = HostPlatform::flash();
	flash[ADR_SHIFTERANALOG_CONF] = G29_SEQ;
	flash[ADR_SHIFTERANALOG_CONF_2] = 0x0102; // X pin 1, Y pin 2
	flash[ADR_SHIFTERANALOG_CONF_3] = 0x0302; // reverse D pin 3, CS 2
	flash[ADR_SHIFTERANALOG_X_12] = 1000;
	flash[ADR_SHIFTERANALOG_X_56] = 2000;
	flash[ADR_SHIFTERANALOG_Y_135] = 3000;
	flash[ADR_SHIFTERANALOG_Y_246] = 4000;
	ShifterAnalog shifter;

	CHECK(get(shifter, "mode") == G29_SEQ);
	CHECK(shifter.getBtnNum() == 2);
	CHECK(get(shifter, "xchan") == 1);
	CHECK(get(shifter, "ychan") == 2);
	CHECK(get(shifter, "revbtn") == 3);
	CHECK(get(shifter, "cspin") == 2);
	CHECK(get(shifter, "x12") == 1000);
	CHECK(get(shifter, "x56") == 2000);
	CHECK(get(shifter, "y135") == 3000);
	CHECK(get(shifter, "y246") == 4000);
	setAnalogPins({2496, 0, 0, 0, 0, 0}); // X pin 1 above X_56
	CHECK(read(shifter) == 1 << 1);
}

TEST_CASE_FIXTURE(ShifterFixture, "a stored G27 mode creates the SPI client of the G27 buttons") {
	HostPlatform::flash()[ADR_SHIFTERANALOG_CONF] = G27_H;
	ShifterAnalog shifter;

	CHECK(shifter.getBtnNum() == 19);
	CHECK(external_spi.getFreeCsPins().size() == 2);
}

} // TEST_SUITE
