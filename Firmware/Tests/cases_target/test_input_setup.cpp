/*
 * Smoke tests of the P5 batch 0 infrastructure (ffb_target_tests): the fake HAL SPI (F4) and I2C (F5) under the real
 * bus wrappers (SPI.cpp, I2C.cpp), CAN frames through the receive FIFO and global_callbacks.cpp, the local inputs on
 * the fake GPIO / ADC, and MockButtonSource / MockAnalogSource selected through the real class choosers.
 * One input source per case; the module tests of P5 (buttons, analog) cover the sources in detail.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "HostRtos.h"
#include "HostUsb.h"
#include "InputSources.h"
#include "TestHelpers.h"

#include "ADS111X.h"
#include "CanAnalog.h"
#include "CanButtons.h"
#include "FFBWheel.h"
#include "I2CHandler.h"
#include "LocalAnalog.h"
#include "LocalButtons.h"
#include "PCF8574.h"
#include "SPIButtons.h"
#include "global_callbacks.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace {

/** Executes a command of `handler` by its name like the command thread does */
CommandStatus cmd(CommandHandler &handler, const std::string &name, CMDtype type, int64_t val = 0) {
	CmdHandlerCommanddef *def = handler.getCommandFromName(name);
	REQUIRE_MESSAGE(def != nullptr, "command " << name << " not registered");
	ParsedCommand parsed;
	parsed.cmdId = def->cmdId;
	parsed.type = type;
	parsed.val = val;
	parsed.target = &handler;
	std::vector<CommandReply> replies;
	return handler.command(parsed, replies);
}

/** Buttons of the gamepad reports (report id 1: id, 64 bit buttons, axes) sent so far */
std::vector<uint64_t> reportedButtons() {
	std::vector<uint64_t> buttons;
	for (const HostUsb::HidReport &r : HostUsb::state().hidReports) {
		if (r.data.size() >= 9 && r.data[0] == 1) {
			uint64_t b = 0;
			std::memcpy(&b, &r.data[1], sizeof(b));
			buttons.push_back(b);
		}
	}
	return buttons;
}

} // namespace

TEST_SUITE("InputSetup") {

TEST_CASE_FIXTURE(TargetFixture, "SPI buttons read through the external SPI port return the scripted shift register bits") {
	SPI_Buttons_1 buttons; // first free CS pin of external_spi: SPI2_NSS
	REQUIRE(cmd(buttons, "mode", CMDtype::set, 1) == CommandStatus::OK); // 74xx165: CS active high
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 8) == CommandStatus::OK);
	uint64_t value = 0;
	buttons.readButtons(&value); // returns the last state and starts the DMA read of the next one
	REQUIRE(HostTarget::spiTransferPending(&hspi2));
	HostTarget::spiQueueRx(&hspi2, {0xA5});

	REQUIRE(HostTarget::spiComplete(&hspi2));

	value = 0;
	CHECK(buttons.readButtons(&value) == 8);
	CHECK(value == 0xA5);
	const HostTarget::SpiTransfer &read = HostTarget::spiTransfers(&hspi2).front();
	CHECK(read.call == HostTarget::SpiCall::Receive);
	CHECK(read.mode == HostTarget::BusMode::Dma);
	CHECK(read.size == 1);
	CHECK(read.selectedCs(false) == 0); // SPI2_NSS high (active) while the bits were shifted in
	CHECK(read.csAtEnd == std::vector<bool>{true, false, false});
}

TEST_CASE_FIXTURE(TargetFixture, "PCF8574 buttons read through the I2C port report the inverted input byte of the expander") {
	HostTarget::I2cDevice &expander = HostTarget::i2cDevice(&hi2c1, 0x20); // first PCF8574 (A2..A0 low)
	expander.pointerRegister = false;
	expander.setRegister(0, {0x5A}); // P1, P3, P4, P6 high (released with pull-up), the others pulled low (pressed)
	PCF8574Buttons buttons;
	uint64_t value = 0;
	buttons.readButtons(&value); // starts the interrupt driven read of the first expander
	REQUIRE(HostTarget::i2cTransferPending(&hi2c1));

	REQUIRE(HostTarget::i2cComplete(&hi2c1)); // receive complete interrupt notifies the thread
	HostRtos::runUntilBlocked(buttons);		  // last expander read: the thread publishes the buttons

	value = 0;
	CHECK(buttons.readButtons(&value) == 8);
	CHECK(value == 0xA5);
	const HostTarget::I2cTransfer &read = HostTarget::i2cTransfers(&hi2c1).front();
	CHECK(read.call == HostTarget::I2cCall::MasterReceive);
	CHECK(read.mode == HostTarget::BusMode::Interrupt);
	CHECK(read.devAddress == (0x20 << 1));
	CHECK(read.data == std::vector<uint8_t>{0x5A});
}

TEST_CASE_FIXTURE(TargetFixture, "an ADS111X single ended conversion read through the I2C port becomes the value of the first analog axis") {
	HostTarget::I2cDevice &adc = HostTarget::i2cDevice(&hi2c1, 0x48); // ADDR pin to GND
	adc.registerWidth = 2;	   // 16 bit registers, MSB first
	adc.autoIncrement = false; // the pointer register selects one register
	adc.setRegister16(0x00, 0x6000); // conversion register
	ADS111X_AnalogSource ads;
	HostRtos::runUntilBlocked(ads); // initialize(), then waits for a request
	ads.getAxes();					// requests a conversion cycle
	HostRtos::runUntilBlocked(ads); // writes the config register: start a single conversion of AIN0
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	HostRtos::runUntilBlocked(ads); // polls the config register (the written OS bit 15 reads back as "done")
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	HostRtos::runUntilBlocked(ads); // reads the conversion register
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	HostRtos::runUntilBlocked(ads); // cycle done

	std::vector<int32_t> *axes = ads.getAxes();

	REQUIRE(axes->size() == 1);
	CHECK(axes->at(0) == (0x6000 - 0x3fff) * 2 - 1); // single ended: 0..0x7fff mapped to -0x7fff..0x7fff
	CHECK(adc.register16(0x01) == 0xC583);			 // OS | AIN0 vs GND | PGA 2.048 V | single shot | 128 SPS
}

TEST_CASE_FIXTURE(TargetFixture, "a CAN frame with the configured id received by the CAN port sets the CAN buttons") {
	CanButtons buttons; // CAN id 100
	REQUIRE(cmd(buttons, "btnnum", CMDtype::set, 32) == CommandStatus::OK);

	REQUIRE(HostTarget::canReceive(100, {0x05, 0x80, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00}) == 1);

	uint64_t value = 0;
	CHECK(buttons.readButtons(&value) == 32);
	CHECK(value == 0x01008005);
}

TEST_CASE_FIXTURE(TargetFixture, "a CAN frame with the configured id received by the CAN port sets the CAN analog axis") {
	CanAnalog<8> analog; // CAN id 110, 1 axis

	REQUIRE(HostTarget::canReceive(110, {0x34, 0x12, 0, 0, 0, 0, 0, 0}) == 1);

	std::vector<int32_t> *axes = analog.getAxes();
	REQUIRE(axes->size() == 1);
	CHECK(axes->at(0) == 0x1234);
}

TEST_CASE_FIXTURE(TargetFixture, "local buttons report the D pins that are pulled low") {
	LocalButtons buttons; // DIN0..7, low active
	HostTarget::setPin(DIN1_GPIO_Port, DIN1_Pin, false);
	HostTarget::setPin(DIN7_GPIO_Port, DIN7_Pin, false);

	uint64_t value = 0;
	CHECK(buttons.readButtons(&value) == 8);
	CHECK(value == 0b10000010);
}

TEST_CASE_FIXTURE(TargetFixture, "local analog reads the analog pins from the ADC1 DMA buffer") {
	LocalAnalog analog;
	startADC(); // HAL_ADC_Start_DMA into ADC1_BUF like cppmain
	REQUIRE(HostTarget::adcConvert(&hadc1, {0, 0x800, 0xFFF, 0x100, 0x200, 0x300, 1000, 1500}));

	std::vector<int32_t> *axes = analog.getAxes();

	// 12 bit values shifted to 16 bit and centred; ranks 6 and 7 (temperature, VREFINT) are not analog pins
	CHECK(*axes == std::vector<int32_t>{-0x7fff, 0x8000 - 0x7fff, 0xFFF0 - 0x7fff, 0x1000 - 0x7fff, 0x2000 - 0x7fff, 0x3000 - 0x7fff});
}

TEST_CASE_FIXTURE(TargetFixture, "the analog values of a mock analog source selected through the analog chooser are returned by the selectable inputs") {
	SelectableInputs inputs{ClassChooser<ButtonSource>{MockButtonSource::registry()}, ClassChooser<AnalogSource>{MockAnalogSource::registry()}};
	inputs.setAinTypes(1 << MockAnalogSource::SELECTION_ID);
	REQUIRE(MockAnalogSource::instances().size() == 1);
	MockAnalogSource::last()->axes = {100, -200};

	CHECK(*inputs.getAnalogValues() == std::vector<int32_t>{100, -200});
}

TEST_CASE_FIXTURE(TargetFixture, "the buttons of a mock button source selected through the button chooser of a wheel are in its HID report" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	InputSources::useMockRegistries(InputSources::of(wheel));
	REQUIRE(cmd(wheel, "btntypes", CMDtype::set, 1 << MockButtonSource::SELECTION_ID) == CommandStatus::OK);
	REQUIRE(MockButtonSource::last() != nullptr);
	MockButtonSource::last()->buttons = 0b1001;
	HostRtos::runUntilBlocked(wheel); // main thread starts TIM_FFB and waits
	REQUIRE(HostTarget::timerRunning(&htim13));

	HostPlatform::advanceMs(1);
	REQUIRE(HostTarget::timerUpdate(&htim13));
	HostRtos::runUntilBlocked(wheel); // one FFB cycle sends the gamepad report

	REQUIRE(!reportedButtons().empty());
	CHECK(reportedButtons().back() == 0b1001);
	CHECK(MockButtonSource::last()->readCalls >= 1);
}

TEST_CASE_FIXTURE(TargetFixture, "a destroyed I2C handler is removed from the I2C handler list" *
										  KNOWN_ISSUE("I2CHandler.cpp: ~I2CHandler calls addCallbackHandler instead of removeCallbackHandler, so a deleted "
													  "handler stays in I2CHandler::getI2CHandlers() and the next I2C callback of global_callbacks.cpp "
													  "calls a deleted object (latent: the only I2C handlers of the firmware are the global I2C ports). "
													  "The CHECK on the list fails")) {
	struct Handler : I2CHandler {};
	const std::vector<I2CHandler *> &list = I2CHandler::getI2CHandlers();
	std::uintptr_t address = 0;
	{
		Handler handler;
		address = reinterpret_cast<std::uintptr_t>(static_cast<I2CHandler *>(&handler));
		REQUIRE(std::count_if(list.begin(), list.end(), [&](I2CHandler *h) { return reinterpret_cast<std::uintptr_t>(h) == address; }) == 1);
	}
	REQUIRE_ALL_HAL_MODELLED();

	// Only the pointer values are compared; the deleted handler is not called (the fixture restores the list)
	CHECK(std::count_if(list.begin(), list.end(), [&](I2CHandler *h) { return reinterpret_cast<std::uintptr_t>(h) == address; }) == 0);
}

} // TEST_SUITE
