/*
 * Tests of SPI_Buttons (SPIButtons.cpp): shift register button source on the external SPI port (ffb_target_tests).
 * The DMA read cycle runs through the real SPIPort (SPI.cpp) on the fake HAL SPI (hspi2): readButtons() returns the
 * bits of the previous read and starts the next DMA receive; HostTarget::spiComplete ends it like the DMA interrupt.
 * CS pins of external_spi: index 0 SPI2_NSS, 1 SPI2_SS2, 2 SPI2_SS3.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "SPIButtons.h"
#include "eeprom_addresses.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

extern SPIPort external_spi;

namespace {

struct SpiButtonsFixture : TargetFixture {
	/** Executes a command of `handler` by its name like the command thread does */
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
	/** Value of a get command (exactly one reply expected) */
	int64_t get(CommandHandler &handler, const std::string &name) {
		std::vector<CommandReply> replies;
		CHECK(cmd(handler, name, CMDtype::get, 0, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies.front().val;
	}
	/** Reply string of an info command */
	std::string info(CommandHandler &handler, const std::string &name) {
		std::vector<CommandReply> replies;
		CHECK(cmd(handler, name, CMDtype::info, 0, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies.front().reply;
	}

	/**
	 * One read cycle: readButtons starts the DMA read, the shift registers answer `miso`, the DMA completes, the next
	 * readButtons returns the processed bits (and starts the next read, which is completed with zeros so that no
	 * transfer is pending when the source is deleted).
	 */
	uint64_t readCycle(SPI_Buttons &buttons, const std::vector<uint8_t> &miso) {
		uint64_t value = 0;
		buttons.readButtons(&value);
		REQUIRE(HostTarget::spiTransferPending(&hspi2));
		HostTarget::spiQueueRx(&hspi2, miso);
		REQUIRE(HostTarget::spiComplete(&hspi2));
		value = 0;
		buttons.readButtons(&value);
		if (HostTarget::spiTransferPending(&hspi2)) {
			REQUIRE(HostTarget::spiComplete(&hspi2));
		}
		return value;
	}
	/** The first transfer on hspi2 (the read started by the first readButtons) */
	const HostTarget::SpiTransfer &firstRead() {
		REQUIRE(!HostTarget::spiTransfers(&hspi2).empty());
		return HostTarget::spiTransfers(&hspi2).front();
	}
	/** Starts one read and returns its transfer record (completed with zeros) */
	HostTarget::SpiTransfer readOnce(SPI_Buttons &buttons) {
		HostTarget::spiClearTransfers(&hspi2);
		uint64_t value = 0;
		buttons.readButtons(&value);
		REQUIRE(HostTarget::spiTransferPending(&hspi2));
		REQUIRE(HostTarget::spiComplete(&hspi2));
		return HostTarget::spiTransfers(&hspi2).front();
	}
	void configure(SPI_Buttons &buttons, int64_t mode, int64_t count) {
		REQUIRE(cmd(buttons, "mode", CMDtype::set, mode) == CommandStatus::OK);
		REQUIRE(cmd(buttons, "btnnum", CMDtype::set, count) == CommandStatus::OK);
	}
	static constexpr int64_t TM = 0;
	static constexpr int64_t PISOSR = 1; // 74xx165
};

OutputPin csPin(size_t index) { return external_spi.getCsPins().at(index); }
/** Same port and pin (GpioPin::operator== is not const, doctest can not use it in an expression) */
bool samePin(const OutputPin &a, const OutputPin &b) { return a.getPort() == b.getPort() && a.getPin() == b.getPin(); }

} // namespace

TEST_SUITE("SPIButtons") {

// ---------------------------------------------------------------- construction, defaults

TEST_CASE_FIXTURE(SpiButtonsFixture, "SPI buttons reserve the first free CS pin of the external SPI port and free it when deleted") {
	{
		SPI_Buttons_1 buttons;
		CHECK_FALSE(external_spi.isPinFree(csPin(0)));
		CHECK(external_spi.isPinFree(csPin(1)));
		CHECK(external_spi.isPinFree(csPin(2)));
	}
	CHECK(external_spi.isPinFree(csPin(0)));
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "SPI buttons do not use a CS pin that another device of the port has reserved" *
										  KNOWN_ISSUE("(bug 68) The constructor reserves the first free CS pin (SPIButtons.cpp:48), but restoreFlash -> "
													  "setConfig frees it and selects the stored CS number (default 1 = pin 0) without checking that "
													  "the pin is free (SPIButtons.cpp:94-100, 'TODO check if pin is free' at line 46): both devices "
													  "drive the same CS pin, and deleting the buttons frees the pin of the other device. The first "
													  "CHECK fails")) {
	REQUIRE(external_spi.reserveCsPin(csPin(0))); // used by another device
	SPI_Buttons_1 buttons;
	REQUIRE_ALL_HAL_MODELLED();

	CHECK_FALSE(samePin(buttons.getSpiConfig()->cs, csPin(0)));
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "characterization: without a stored configuration SPI buttons read 1 button in Thrustmaster mode on CS 1 at the fast speed preset") {
	// NEEDS REVIEW 77: restoreFlash decodes the defaults (0, 1) as speed preset 0 ("Fast", prescaler 16) although
	// ButtonSourceConfig::spi_speed defaults to 1 ("Medium", SPIButtons.h:31) and SPIBUTTONS_SPEED is prescaler 32
	SPI_Buttons_1 buttons;

	CHECK(get(buttons, "mode") == TM);
	CHECK(get(buttons, "btnnum") == 1);
	CHECK(get(buttons, "cs") == 1);
	CHECK(get(buttons, "spispeed") == 0);
	CHECK(buttons.getBtnNum() == 1);
	CHECK(readOnce(buttons).init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_16);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "SPI buttons 1 are identified as SPI Buttons 1 with the SPI button class id") {
	SPI_Buttons_1 buttons;
	CHECK(std::string(buttons.getInfo().name) == "SPI Buttons 1");
	CHECK(buttons.getInfo().id == CLSID_BTN_SPI);
	CHECK(buttons.getClassType() == ClassType::Buttonsource);
	CHECK(buttons.getHelpstring() == "SPI 2 Button");
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "SPI buttons 1 are creatable while the external SPI port is not taken exclusively") {
	CHECK(SPI_Buttons_1::isCreatable());
	external_spi.takeExclusive(true);
	CHECK_FALSE(SPI_Buttons_1::isCreatable());
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "SPI buttons are not creatable while every CS pin of the external SPI port is in use" *
										  KNOWN_ISSUE("(bug 67) SPI_Buttons_1::isCreatable (SPIButtons.cpp:29-31) uses SPIPort::hasFreePins, which checks "
													  "getCsPins().size() (all pins of the port, SPI.cpp:220-222) instead of the free pins: the "
													  "source is offered while no CS pin is free, and its constructor then reads "
													  "getFreeCsPins()[0] of an empty vector (SPIButtons.cpp:48). The CHECK fails")) {
	for (size_t i = 0; i < 3; i++) {
		REQUIRE(external_spi.reserveCsPin(csPin(i)));
	}
	REQUIRE(external_spi.getFreeCsPins().empty());
	REQUIRE_ALL_HAL_MODELLED();

	CHECK_FALSE(SPI_Buttons_1::isCreatable());
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "SPI buttons 2 are never creatable") {
	CHECK_FALSE(SPI_Buttons_2::isCreatable());
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "SPI buttons 2 restore their configuration from their own flash addresses") {
	HostPlatform::flash()[ADR_SPI_BTN_2_CONF] = (1 << 8) | (16 - 1); // 74xx165, 16 buttons
	HostPlatform::flash()[ADR_SPI_BTN_2_CONF_2] = 2;				   // CS 2
	SPI_Buttons_2 buttons;

	CHECK(std::string(buttons.getInfo().name) == "SPI Buttons 2");
	CHECK(buttons.getInfo().id == CLSID_BTN_SPI);
	CHECK(get(buttons, "mode") == PISOSR);
	CHECK(get(buttons, "btnnum") == 16);
	CHECK(get(buttons, "cs") == 2);
}

// ---------------------------------------------------------------- SPI settings of the modes

TEST_CASE_FIXTURE(SpiButtonsFixture, "Thrustmaster mode reads LSB first with clock idle low, sampling on the first edge and an active low CS") {
	SPI_Buttons_1 buttons;
	configure(buttons, TM, 8);

	HostTarget::SpiTransfer read = readOnce(buttons);

	CHECK(read.call == HostTarget::SpiCall::Receive);
	CHECK(read.mode == HostTarget::BusMode::Dma);
	CHECK(read.init.FirstBit == SPI_FIRSTBIT_LSB);
	CHECK(read.init.CLKPolarity == SPI_POLARITY_LOW);
	CHECK(read.init.CLKPhase == SPI_PHASE_1EDGE);
	CHECK(read.init.Mode == SPI_MODE_MASTER);
	CHECK(read.csAtStart.at(0) == false); // asserted (low) while the bits are shifted in
	CHECK(HostTarget::outputPin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin) == true);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "74xx165 mode reads LSB first with clock idle high, sampling on the second edge and an active high CS") {
	SPI_Buttons_1 buttons;
	configure(buttons, PISOSR, 8);

	HostTarget::SpiTransfer read = readOnce(buttons);

	CHECK(read.init.FirstBit == SPI_FIRSTBIT_LSB);
	CHECK(read.init.CLKPolarity == SPI_POLARITY_HIGH);
	CHECK(read.init.CLKPhase == SPI_PHASE_2EDGE);
	CHECK(read.csAtStart.at(0) == true);
	CHECK(HostTarget::outputPin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin) == false);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "a mode change reconfigures the SPI port with HAL_SPI_Init") {
	SPI_Buttons_1 buttons;
	uint32_t inits = HostTarget::spiInitCount(&hspi2);

	REQUIRE(cmd(buttons, "mode", CMDtype::set, PISOSR) == CommandStatus::OK);

	CHECK(HostTarget::spiInitCount(&hspi2) == inits + 1);
	CHECK(hspi2.Init.CLKPolarity == SPI_POLARITY_HIGH);
	CHECK(hspi2.Init.CLKPhase == SPI_PHASE_2EDGE);
}

TEST_CASE("SPI buttons read the shift register chain with the number of bytes of the mode and button count") {
	struct Case {
		int64_t mode;
		int64_t count;
		uint16_t bytes;
	};
	// Thrustmaster: 1 + (n + 2) / 8 (2 extra id bits); 74xx165: 1 + (n - 1) / 8
	const std::array<Case, 9> cases{{{0, 5, 1}, {0, 6, 2}, {0, 12, 2}, {0, 14, 3}, {0, 21, 3}, {1, 1, 1}, {1, 8, 1}, {1, 9, 2}, {1, 24, 3}}};
	for (const Case &c : cases) {
		CAPTURE(c.mode);
		CAPTURE(c.count);
		SpiButtonsFixture f;
		SPI_Buttons_1 buttons;
		f.configure(buttons, c.mode, c.count);
		CHECK(f.readOnce(buttons).size == c.bytes);
	}
}

// ---------------------------------------------------------------- decoding

TEST_CASE_FIXTURE(SpiButtonsFixture, "74xx165 mode reports the received bits masked to the button count") {
	SPI_Buttons_1 buttons;
	configure(buttons, PISOSR, 12);

	CHECK(readCycle(buttons, {0x34, 0xF2}) == 0x234);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "74xx165 mode with 8 buttons reports the received byte") {
	SPI_Buttons_1 buttons;
	configure(buttons, PISOSR, 8);

	CHECK(readCycle(buttons, {0xA5}) == 0xA5);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "74xx165 mode with 32 buttons reports the four received bytes in bus order") {
	SPI_Buttons_1 buttons;
	configure(buttons, PISOSR, 32);

	CHECK(readCycle(buttons, {0x01, 0x23, 0x45, 0x67}) == 0x67452301);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "inverted 74xx165 buttons report the received bits inverted and masked") {
	SPI_Buttons_1 buttons;
	configure(buttons, PISOSR, 12);
	REQUIRE(cmd(buttons, "btnpol", CMDtype::set, 1) == CommandStatus::OK);

	CHECK(readCycle(buttons, {0x34, 0xF2}) == 0xDCB);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "Thrustmaster mode drops the extra leading bits of a button count that is not byte aligned") {
	SPI_Buttons_1 buttons;
	configure(buttons, TM, 12); // 2 bytes, the first 4 bits are cut

	CHECK(readCycle(buttons, {0x30, 0x12}) == 0x123);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "inverted Thrustmaster buttons report the remaining bits inverted") {
	SPI_Buttons_1 buttons;
	configure(buttons, TM, 12);
	REQUIRE(cmd(buttons, "btnpol", CMDtype::set, 1) == CommandStatus::OK);

	CHECK(readCycle(buttons, {0x30, 0x12}) == 0xEDC);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "characterization: Thrustmaster mode with a byte aligned button count drops the whole first byte") {
	// NEEDS REVIEW 78: offset = 8 - n % 8 is 8 (not 0) for n = 8, 16, ...; with cutRight (forced in TM mode) the first
	// byte is shifted out (SPIButtons.cpp:122, 160-163), while 7 buttons drop 1 bit and 9 buttons drop 7 bits. The
	// `if(offset)` guard suggests 0 was expected for aligned counts. With 64 buttons the last 8 buttons are lost
	// (only 8 of the 9 received bytes are copied)
	SPI_Buttons_1 buttons;
	configure(buttons, TM, 8); // 2 bytes

	CHECK(readCycle(buttons, {0xAB, 0xCD}) == 0xCD);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "readButtons returns the bits of the previous read and starts the next one") {
	SPI_Buttons_1 buttons;
	configure(buttons, PISOSR, 8);
	uint64_t value = 0xFF;

	CHECK(buttons.readButtons(&value) == 8);

	CHECK(value == 0x00); // nothing read yet
	REQUIRE(HostTarget::spiTransferPending(&hspi2));
	HostTarget::spiQueueRx(&hspi2, {0x22});
	REQUIRE(HostTarget::spiComplete(&hspi2));
	value = 0;
	buttons.readButtons(&value);
	CHECK(value == 0x22);
	CHECK(HostTarget::spiTransfers(&hspi2).size() == 2);
	REQUIRE(HostTarget::spiComplete(&hspi2));
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "readButtons does not start a read while the previous read is pending") {
	SPI_Buttons_1 buttons;
	configure(buttons, PISOSR, 8);
	uint64_t value = 0;
	buttons.readButtons(&value);
	REQUIRE(HostTarget::spiTransfers(&hspi2).size() == 1);

	CHECK(buttons.readButtons(&value) == 8);

	CHECK(HostTarget::spiTransfers(&hspi2).size() == 1);
	REQUIRE(HostTarget::spiComplete(&hspi2));
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "readButtons does not start a read while another device holds the SPI port") {
	SPI_Buttons_1 buttons;
	configure(buttons, PISOSR, 8);
	external_spi.takeSemaphore();
	uint64_t value = 0;

	CHECK(buttons.readButtons(&value) == 8);

	CHECK(HostTarget::spiTransfers(&hspi2).empty());
	external_spi.giveSemaphore();
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "a failed read releases CS and the SPI port and keeps the last buttons") {
	SPI_Buttons_1 buttons;
	configure(buttons, PISOSR, 8);
	uint64_t value = 0;
	buttons.readButtons(&value);
	HostTarget::spiQueueRx(&hspi2, {0x5A});
	REQUIRE(HostTarget::spiComplete(&hspi2));
	buttons.readButtons(&value); // returns 0x5A, starts the read that fails
	REQUIRE(HostTarget::spiTransferPending(&hspi2));

	REQUIRE(HostTarget::spiFail(&hspi2));

	CHECK_FALSE(external_spi.isTaken());
	CHECK(HostTarget::outputPin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin) == false); // 74xx165: inactive level low
	HostTarget::spiClearTransfers(&hspi2);
	value = 0;
	buttons.readButtons(&value);
	CHECK(value == 0x5A);
	CHECK(HostTarget::spiTransferPending(&hspi2)); // a new read was started
	REQUIRE(HostTarget::spiComplete(&hspi2));
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "a read of 8 bytes stays inside the receive buffer of the source" *
										  KNOWN_ISSUE_UB("SPIButtons.h:81 spi_buf has 4 bytes, but SPIButtons.cpp:125-129 computes up to 9 bytes "
														 "(Thrustmaster above 29 buttons, 74xx165 above 32): the DMA writes past spi_buf and "
														 "the memcpy at SPIButtons.cpp:174 reads past it (bug 55). In SPI_Buttons_1/2 the "
														 "excess lands in the tail padding of the object (5 bytes on host and target), so it "
														 "is only visible in a class derived from SPI_Buttons whose members reuse that "
														 "padding (Itanium C++ ABI). The CHECK on the derived member fails")) {
	struct DerivedButtons : SPI_Buttons {
		DerivedButtons() : SPI_Buttons(ADR_SPI_BTN_1_CONF, ADR_SPI_BTN_1_CONF_2) { guard.fill(0xA5); }
		const ClassIdentifier getInfo() override { return SPI_Buttons_1::info; }
		std::array<uint8_t, 8> guard; // a member of a derived class placed after the SPI_Buttons members
	};
	DerivedButtons buttons;
	configure(buttons, PISOSR, 64);
	uint64_t value = 0;
	buttons.readButtons(&value);
	REQUIRE(HostTarget::spiTransfers(&hspi2).back().size == 8);
	HostTarget::spiQueueRx(&hspi2, {1, 2, 3, 4, 5, 6, 7, 8});
	REQUIRE(HostTarget::spiComplete(&hspi2));
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(buttons.guard == std::array<uint8_t, 8>{0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5});
}

// ---------------------------------------------------------------- commands

TEST_CASE_FIXTURE(SpiButtonsFixture, "mode info lists the modes with their ids") {
	SPI_Buttons_1 buttons;
	CHECK(info(buttons, "mode") == "Thrustmaster/HEF4021BT:0\n74xx165:1\n");
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "mode set selects the mode that mode get returns") {
	SPI_Buttons_1 buttons;
	REQUIRE(cmd(buttons, "mode", CMDtype::set, PISOSR) == CommandStatus::OK);
	CHECK(get(buttons, "mode") == PISOSR);
	REQUIRE(cmd(buttons, "mode", CMDtype::set, TM) == CommandStatus::OK);
	CHECK(get(buttons, "mode") == TM);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "btnnum set changes the button count up to 64") {
	SPI_Buttons_1 buttons;
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 20) == CommandStatus::OK);
	CHECK(get(buttons, "btnnum") == 20);
	CHECK(buttons.getBtnNum() == 20);

	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 100) == CommandStatus::OK);
	CHECK(get(buttons, "btnnum") == 64);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "btnpol set inverts the buttons and btnpol get returns it") {
	SPI_Buttons_1 buttons;
	CHECK(get(buttons, "btnpol") == 0);
	REQUIRE(cmd(buttons, "btnpol", CMDtype::set, 5) == CommandStatus::OK);
	CHECK(get(buttons, "btnpol") == 1);
	REQUIRE(cmd(buttons, "btnpol", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(get(buttons, "btnpol") == 0);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "a mode change sets the cut flag of the mode preset") {
	SPI_Buttons_1 buttons;
	configure(buttons, TM, 12);
	CHECK(get(buttons, "btncut") == 1);
	configure(buttons, PISOSR, 12);
	CHECK(get(buttons, "btncut") == 0);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "btncut set cuts the extra bits of 74xx165 buttons on the right" *
										  KNOWN_ISSUE("(bug 75) setConfig stores the requested cutRight and then overwrites it with the mode preset "
													  "(TM: true, 74xx165: false, SPIButtons.cpp:104, 110), so the registered btncut set "
													  "(SPIButtons.cpp:219-224) never has an effect. The CHECK on the read buttons fails")) {
	SPI_Buttons_1 buttons;
	configure(buttons, PISOSR, 12);
	REQUIRE(cmd(buttons, "btncut", CMDtype::set, 1) == CommandStatus::OK);
	uint64_t value = readCycle(buttons, {0x34, 0xF2});
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(value == 0xF23); // 0xF234 shifted right by the 4 extra bits
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "cs set moves the source to another CS pin of the port") {
	SPI_Buttons_1 buttons;
	configure(buttons, PISOSR, 8);

	REQUIRE(cmd(buttons, "cs", CMDtype::set, 2) == CommandStatus::OK);

	CHECK(get(buttons, "cs") == 2);
	CHECK(external_spi.isPinFree(csPin(0)));
	CHECK_FALSE(external_spi.isPinFree(csPin(1)));
	HostTarget::SpiTransfer read = readOnce(buttons);
	CHECK(read.csAtStart.at(1) == true); // active high in 74xx165 mode
	CHECK(HostTarget::outputPin(SPI2_SS2_GPIO_Port, SPI2_SS2_Pin) == false); // released after the read
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "cs set with a number without CS pin keeps the current pin") {
	SPI_Buttons_1 buttons;
	REQUIRE(cmd(buttons, "cs", CMDtype::set, 3) == CommandStatus::OK);
	REQUIRE(samePin(buttons.getSpiConfig()->cs, csPin(2)));

	REQUIRE(cmd(buttons, "cs", CMDtype::set, 4) == CommandStatus::OK); // no pin 4

	CHECK(samePin(buttons.getSpiConfig()->cs, csPin(2)));
	CHECK_FALSE(external_spi.isPinFree(csPin(2)));
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "spispeed info lists the speed presets with their ids") {
	SPI_Buttons_1 buttons;
	CHECK(info(buttons, "spispeed") == "Fast:0\nMedium:1\nSlow:2\n");
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "spispeed set selects the prescaler of the preset for the next read") {
	struct Case {
		int64_t preset;
		uint32_t prescaler;
	};
	for (const Case &c : {Case{0, SPI_BAUDRATEPRESCALER_16}, Case{1, SPI_BAUDRATEPRESCALER_32}, Case{2, SPI_BAUDRATEPRESCALER_64}}) {
		CAPTURE(c.preset);
		SPI_Buttons_1 buttons;
		REQUIRE(cmd(buttons, "spispeed", CMDtype::set, c.preset) == CommandStatus::OK);
		CHECK(get(buttons, "spispeed") == c.preset);
		CHECK(hspi2.Init.BaudRatePrescaler == c.prescaler); // initSPI reconfigured the port
		CHECK(readOnce(buttons).init.BaudRatePrescaler == c.prescaler);
	}
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "spispeed with a preset beyond the last selects the last preset" *
										  doctest::description("KNOWN ISSUE (crash): SPI_Buttons::setSpiSpeed (SPIButtons.cpp:139) clips the preset to "
															   "speedPresets.size() (3) instead of size()-1 and reads speedPresets[3] of a 3 element "
															   "std::array (line 141, bug 56). Out of bounds: aborts the run with _GLIBCXX_ASSERTIONS "
															   "(GCC 16 -O0). Skipped like the other crash known issues") *
										  doctest::skip()) {
	SPI_Buttons_1 buttons;

	CHECK(cmd(buttons, "spispeed", CMDtype::set, 3) == CommandStatus::OK);

	CHECK(get(buttons, "spispeed") == 2);
	CHECK(hspi2.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_64);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "a stored speed preset beyond the last selects the last preset" *
										  doctest::description("KNOWN ISSUE (crash): restoreFlash decodes the speed preset as 2 bits (0..3, "
															   "SPIButtons.cpp:274) and setConfig reads speedPresets[3] of a 3 element std::array "
															   "without a check (line 115, bug 56). Out of bounds: aborts the run with "
															   "_GLIBCXX_ASSERTIONS (GCC 16 -O0). Skipped like the other crash known issues") *
										  doctest::skip()) {
	HostPlatform::flash()[ADR_SPI_BTN_1_CONF_2] = (3 << 3) | 1;
	SPI_Buttons_1 buttons;

	CHECK(get(buttons, "spispeed") == 2);
	CHECK(hspi2.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_64);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "commands reject request types they do not support") {
	SPI_Buttons_1 buttons;
	CHECK(cmd(buttons, "btnnum", CMDtype::info) == CommandStatus::ERR);
	CHECK(cmd(buttons, "btnpol", CMDtype::info) == CommandStatus::ERR);
	CHECK(cmd(buttons, "btncut", CMDtype::info) == CommandStatus::ERR);
	CHECK(cmd(buttons, "mode", CMDtype::setat) == CommandStatus::ERR);
	CHECK(cmd(buttons, "spispeed", CMDtype::getat) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "a command id that is not an SPI button command is not found") {
	SPI_Buttons_1 buttons;
	ParsedCommand parsed;
	parsed.cmdId = 0x7f;
	parsed.type = CMDtype::get;
	std::vector<CommandReply> replies;
	CHECK(buttons.command(parsed, replies) == CommandStatus::NOT_FOUND);
}

// ---------------------------------------------------------------- flash

TEST_CASE_FIXTURE(SpiButtonsFixture, "saveFlash stores button count, invert, cut, mode, CS number and speed preset") {
	SPI_Buttons_1 buttons;
	configure(buttons, PISOSR, 20);
	REQUIRE(cmd(buttons, "btnpol", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(cmd(buttons, "cs", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE(cmd(buttons, "spispeed", CMDtype::set, 2) == CommandStatus::OK);

	buttons.saveFlash();

	CHECK(HostPlatform::flash()[ADR_SPI_BTN_1_CONF] == ((1 << 8) | (1 << 6) | (20 - 1))); // cut 0 (74xx165)
	CHECK(HostPlatform::flash()[ADR_SPI_BTN_1_CONF_2] == ((2 << 3) | 2));
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "saveFlash stores the cut flag of Thrustmaster mode") {
	SPI_Buttons_1 buttons;
	configure(buttons, TM, 64);

	buttons.saveFlash();

	CHECK(HostPlatform::flash()[ADR_SPI_BTN_1_CONF] == ((1 << 7) | 63));
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "a new source restores the stored configuration") {
	HostPlatform::flash()[ADR_SPI_BTN_1_CONF] = (1 << 8) | (1 << 6) | (20 - 1);
	HostPlatform::flash()[ADR_SPI_BTN_1_CONF_2] = (1 << 3) | 3; // medium, CS 3
	SPI_Buttons_1 buttons;

	CHECK(get(buttons, "mode") == PISOSR);
	CHECK(get(buttons, "btnnum") == 20);
	CHECK(get(buttons, "btnpol") == 1);
	CHECK(get(buttons, "cs") == 3);
	CHECK(get(buttons, "spispeed") == 1);
	CHECK(samePin(buttons.getSpiConfig()->cs, csPin(2)));
	HostTarget::SpiTransfer read = readOnce(buttons);
	CHECK(read.size == 3);
	CHECK(read.init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_32);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "a saved configuration is restored unchanged by a new source") {
	{
		SPI_Buttons_1 buttons;
		configure(buttons, TM, 30);
		REQUIRE(cmd(buttons, "btnpol", CMDtype::set, 1) == CommandStatus::OK);
		REQUIRE(cmd(buttons, "spispeed", CMDtype::set, 2) == CommandStatus::OK);
		buttons.saveFlash();
	}
	SPI_Buttons_1 restored;

	CHECK(get(restored, "mode") == TM);
	CHECK(get(restored, "btnnum") == 30);
	CHECK(get(restored, "btnpol") == 1);
	CHECK(get(restored, "btncut") == 1);
	CHECK(get(restored, "spispeed") == 2);
	CHECK(get(restored, "cs") == 1);
}

TEST_CASE_FIXTURE(SpiButtonsFixture, "a button count set to 0 is restored as the count the source reported before saving" *
										  KNOWN_ISSUE("(bug 69) btnnum accepts 0 (SPIButtons.cpp:91 only limits the maximum), but saveFlash stores "
													  "(numButtons-1) & 0x3F (SPIButtons.cpp:278): 0 is saved as 63 and restored as 64 buttons. "
													  "CanButtons and PCF8574Buttons limit the count to 1..64. The CHECK after the restore fails")) {
	int64_t before = -1;
	{
		SPI_Buttons_1 buttons;
		REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 0) == CommandStatus::OK);
		before = get(buttons, "btnnum");
		buttons.saveFlash();
	}
	SPI_Buttons_1 restored;
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(get(restored, "btnnum") == before);
}

} // TEST_SUITE
