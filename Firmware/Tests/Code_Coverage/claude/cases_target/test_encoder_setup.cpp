/*
 * Smoke tests of the P6 batch 0 infrastructure (ffb_target_tests): the local encoder on the fake encoder timer
 * (HostTarget::encoderTimerCount: counting, overflow / underflow update interrupt through global_callbacks.cpp) and the
 * index pin (HostTarget::encoderIndexPulse), the SPI encoders on the encoder SPI port (ext3_spi, hspi3) with scripted
 * frames (HostTarget::spiFrameBytes), the encoder threads as resumable tasks (HostRtos::makeResumable), an encoder
 * selected through the encoder chooser of a real axis (AxisFixture::selectFirmwareEncoder) and the mock bus devices
 * (MockSPIDevice, MockI2CDevice). One encoder or device per case; the module tests of P6 cover them in detail.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "HostRtos.h"
#include "MockI2CDevice.h"
#include "MockSPIDevice.h"
#include "TestHelpers.h"

#include "EncoderBissC.h"
#include "EncoderLocal.h"
#include "EncoderSSI.h"
#include "ExtiHandler.h"
#include "MtEncoderSPI.h"
#include "TimerHandler.h"
#include "cpp_target_config.h"
#include "flash_helpers.h"
#include "semaphore.hpp"
#include "thread.hpp"

#include <algorithm>
#include <cstdint>
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

/**
 * AMT23 (CUI AMT22/23 SSI protocol) response: 2 check bits and the 14 bit position, MSB first. K1 = odd parity over
 * the odd position bits 13, 11, ..., 1; K0 = odd parity over the even bits 12, 10, ..., 0 (the XOR of all bits of
 * each group including its check bit is 1).
 */
uint16_t amt23Frame(uint16_t position) {
	position &= 0x3FFF;
	uint8_t odd = 0, even = 0;
	for (int bit = 0; bit < 14; bit++) {
		uint8_t b = (position >> bit) & 1U;
		if (bit % 2) {
			odd ^= b;
		} else {
			even ^= b;
		}
	}
	const uint16_t k1 = odd ^ 1U, k0 = even ^ 1U;
	return static_cast<uint16_t>((k1 << 15) | (k0 << 14) | position);
}

/**
 * BiSS-C CRC: polynomial x^6 + x + 1 (0x43), start value 0, over the data bits MSB first, transmitted inverted
 * (BiSS C protocol description, iC-Haus). Bitwise reference, independent of the table driven firmware code.
 */
uint8_t bissCrc6(uint32_t data, int bits) {
	uint8_t crc = 0;
	for (int i = bits - 1; i >= 0; i--) {
		const uint8_t feedback = static_cast<uint8_t>(((crc >> 5) & 1U) ^ ((data >> i) & 1U));
		crc = static_cast<uint8_t>((crc << 1) & 0x3F);
		if (feedback) {
			crc ^= 0x03;
		}
	}
	return static_cast<uint8_t>(~crc & 0x3F);
}

/**
 * 64 bit MISO stream of one BiSS-C single cycle read (22 bit position): the idle level 1 and 3 acknowledge bits (0)
 * before the start bit, start bit 1, CDS 0, the position, nE and nW (1 = no error / no warning, active low) and the
 * inverted CRC6 over position, nE, nW; then the line stays low.
 */
uint64_t bissFrame(uint32_t position) {
	const uint32_t data = ((position & 0x3FFFFF) << 2) | 0x3; // position, nE = 1, nW = 1
	uint64_t frame = 1ULL << 63;							  // idle (still high at the first sample)
	frame |= 1ULL << 59;									  // start bit after 3 acknowledge bits
	frame |= static_cast<uint64_t>(data) << 34;				  // CDS = 0 at bit 58, data at bits 57..34
	frame |= static_cast<uint64_t>(bissCrc6(data, 24)) << 28;
	return frame;
}

/** Index of a CS pin in the pin list of a port (-1 if it is not one of them) */
int csIndex(SPIPort &port, const OutputPin &pin) {
	auto &pins = port.getCsPins();
	auto it = std::find(pins.begin(), pins.end(), pin);
	return it == pins.end() ? -1 : static_cast<int>(it - pins.begin());
}

/** True if the list holds a pointer with this address (only the pointer values are compared) */
template <class T>
bool listed(const std::vector<T *> &list, std::uintptr_t address) {
	return std::any_of(list.begin(), list.end(), [address](T *p) { return reinterpret_cast<std::uintptr_t>(p) == address; });
}

} // namespace

TEST_SUITE("EncoderSetup") {

TEST_CASE_FIXTURE(TargetFixture, "the local encoder position follows the counts of the encoder timer from its centre") {
	EncoderLocal enc; // starts TIM_ENC (htim3) at its centre 0x7fff
	REQUIRE(HostTarget::timerRunning(&htim3));
	REQUIRE(HostTarget::timerCounter(&htim3) == 0x7fff);

	CHECK(HostTarget::encoderTimerCount(&htim3, 100) == 0);
	CHECK(enc.getPos() == 100);
	CHECK(HostTarget::encoderTimerCount(&htim3, -250) == 0);
	CHECK(enc.getPos() == -150);
}

TEST_CASE_FIXTURE(TargetFixture, "an overflow of the encoder timer keeps the local encoder position continuous") {
	EncoderLocal enc;

	// 0x7fff + 0x8005 counts: past ARR 0xffff, the counter wraps to 0 and continues to 4
	CHECK(HostTarget::encoderTimerCount(&htim3, 0x8005) == 1);

	CHECK(HostTarget::timerCounter(&htim3) == 4);
	CHECK(enc.getPos() == 0x8005);
}

TEST_CASE_FIXTURE(TargetFixture, "an underflow of the encoder timer keeps the local encoder position continuous") {
	EncoderLocal enc;

	// 0x7fff - 0x8005 counts: below 0, the counter wraps to ARR 0xffff and continues to 0xfffa
	CHECK(HostTarget::encoderTimerCount(&htim3, -0x8005) == 1);

	CHECK(HostTarget::timerCounter(&htim3) == 0xfffa);
	CHECK(enc.getPos() == -0x8005);
}

TEST_CASE_FIXTURE(TargetFixture, "an index pulse with the index enabled centres the local encoder at the index") {
	EncoderLocal enc;
	REQUIRE(cmd(enc, "index", CMDtype::set, 1) == CommandStatus::OK);
	HostTarget::encoderTimerCount(&htim3, 123);
	REQUIRE(enc.getPos() == 123);

	HostTarget::encoderIndexPulse(); // ENCODER_Z falling edge: EXTI callback while the pin reads low

	CHECK(HostTarget::timerCounter(&htim3) == 0x7fff); // counter centred at the index
	CHECK(enc.getPos() == 0);						   // position = stored offset (0)
}

TEST_CASE_FIXTURE(AxisFixture, "the local encoder selected through the encoder chooser of an axis reads the encoder timer") {
	Encoder *enc = selectFirmwareEncoder(2); // Encoder::all_encoders: 2 = local ABN

	REQUIRE(dynamic_cast<EncoderLocal *>(enc) != nullptr);
	HostTarget::encoderTimerCount(&htim3, 400);
	CHECK(axis().getEncoder()->getPos() == 400);
	CHECK(EncoderLocal::inUse);
}

TEST_CASE_FIXTURE(TargetFixture, "deleting the local encoder stops the encoder timer and unregisters its interrupt handlers") {
	auto *enc = new EncoderLocal();
	const std::uintptr_t exti = reinterpret_cast<std::uintptr_t>(static_cast<ExtiHandler *>(enc));
	REQUIRE(listed(ExtiHandler::extiHandlers, exti));
	REQUIRE(TimerHandler::timerHandlers.size() == 1); // the encoder is the only timer handler
	REQUIRE(HostTarget::timerRunning(&htim3));

	delete enc;

	CHECK_FALSE(HostTarget::timerRunning(&htim3)); // ~EncoderLocal: CR1 = 0
	CHECK_FALSE(listed(ExtiHandler::extiHandlers, exti));
	CHECK(TimerHandler::timerHandlers.empty());
	CHECK(EncoderLocal::isCreatable());
}

TEST_CASE_FIXTURE(TargetFixture, "an SSI encoder in AMT23 mode decodes the 14 bit position read from the encoder SPI port") {
	// Stored configuration (EncoderSSI::saveFlash layout): bits - 1 = 13, speed preset 0, mode 1 (AMT23)
	REQUIRE(Flash_Write(ADR_SSI_CONF1, (1 << 8) | 13));
	EncoderSSI enc;
	enc.getPosAbs(); // starts the DMA read of the next frame
	REQUIRE(HostTarget::spiTransferPending(&hspi3));
	HostTarget::spiQueueRx(&hspi3, HostTarget::spiFrameBytes(amt23Frame(0x3000), 2));

	REQUIRE(HostTarget::spiComplete(&hspi3));
	const bool csReleased = HostTarget::spiCsLevels(&hspi3)[0]; // before getPos starts the next read

	CHECK(enc.getPos() == 0x3000 - 0x2000); // centred: position - half a turn
	const HostTarget::SpiTransfer &read = HostTarget::spiTransfers(&hspi3).front();
	CHECK(read.call == HostTarget::SpiCall::Receive);
	CHECK(read.mode == HostTarget::BusMode::Dma);
	CHECK(read.size == 2);
	// SPI3_SS1 low (active) while the frame was clocked in, released (high) by the completion callback. (SPI3_SS2/3 are
	// low too: MX_GPIO_Init resets all SPI3 CS outputs and only a device drives its own pin, so selectedCs() finds no
	// single active pin)
	REQUIRE(read.csAtStart.size() == 3);
	CHECK_FALSE(read.csAtStart[0]);
	CHECK(csReleased);
	CHECK(read.init.CLKPolarity == SPI_POLARITY_LOW);
	CHECK(read.init.CLKPhase == SPI_PHASE_2EDGE);
	CHECK(read.init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_32);
}

TEST_CASE_FIXTURE(TargetFixture, "deleting an SSI encoder frees its CS pin and the exclusive use of the encoder SPI port") {
	REQUIRE(Flash_Write(ADR_SSI_CONF1, (1 << 8) | 13)); // defined mode (AMT23)
	const OutputPin cs = ext3_spi.getCsPins()[0];
	auto *enc = new EncoderSSI();
	REQUIRE_FALSE(ext3_spi.isPinFree(cs));
	REQUIRE_FALSE(ext3_spi.hasFreePins()); // taken exclusively

	delete enc;

	CHECK(ext3_spi.isPinFree(cs));
	CHECK(ext3_spi.hasFreePins());
	CHECK(EncoderSSI::isCreatable());
}

TEST_CASE_FIXTURE(TargetFixture, "the BiSS-C encoder thread decodes the frame read with DMA from the encoder SPI port") {
	EncoderBissC enc;
	TaskHandle_t task = HostRtos::makeResumable("BISSENC"); // waits for a request and for the DMA in one loop
	REQUIRE(task != nullptr);
	enc.getPosAbs();				 // requests a position (no wait at time 0)
	HostRtos::runUntilBlocked(task); // starts the DMA read and waits for its notification
	REQUIRE(HostTarget::spiTransferPending(&hspi3));
	HostTarget::spiQueueRx(&hspi3, HostTarget::spiFrameBytes(bissFrame(0x1234), 8));
	REQUIRE(HostTarget::spiComplete(&hspi3)); // receive complete interrupt notifies the thread

	HostRtos::runUntilBlocked(task); // decodes the frame, waits for the next request

	CHECK(enc.getPos() == -0x1234); // direction inverted by default
	const HostTarget::SpiTransfer &read = HostTarget::spiTransfers(&hspi3).front();
	CHECK(read.call == HostTarget::SpiCall::Receive);
	CHECK(read.mode == HostTarget::BusMode::Dma);
	CHECK(read.size == 8);
	CHECK(read.init.CLKPolarity == SPI_POLARITY_HIGH);
	CHECK(read.init.CLKPhase == SPI_PHASE_2EDGE);
}

TEST_CASE_FIXTURE(TargetFixture, "deleting a BiSS-C encoder ends its waiting thread and releases the encoder SPI port") {
	const OutputPin cs = ext3_spi.getCsPins()[0];
	const size_t tasksBefore = HostRtos::taskCount();
	auto *enc = new EncoderBissC();
	REQUIRE(HostRtos::makeResumable("BISSENC") != nullptr);
	HostRtos::runUntilBlocked(HostRtos::findTask("BISSENC")); // parks in the wait for a request
	REQUIRE(HostRtos::taskCount() == tasksBefore + 1);

	delete enc;

	CHECK(HostRtos::findTask("BISSENC") == nullptr);
	CHECK(HostRtos::taskCount() == tasksBefore);
	CHECK(ext3_spi.isPinFree(cs));
	CHECK(ext3_spi.hasFreePins());
}

TEST_CASE_FIXTURE(TargetFixture, "the MT6825 encoder thread decodes the angle read with DMA from the encoder SPI port") {
	MtEncoderSPI enc; // mode MT6825, first free CS pin of the encoder SPI port
	TaskHandle_t task = HostRtos::makeResumable("MTENC");
	REQUIRE(task != nullptr);
	enc.getPosAbs();				 // requests an angle (no wait at time 0)
	HostRtos::runUntilBlocked(task); // starts the burst read of the angle registers, waits for the DMA
	REQUIRE(HostTarget::spiTransferPending(&hspi3));
	// Dummy byte, angle bits 17..10 = 0x40, bits 9..4 = 0x01 (no magnet warning), bits 3..0 = 0: both parity groups even
	HostTarget::spiQueueRx(&hspi3, {0x00, 0x40, 0x04, 0x00});
	REQUIRE(HostTarget::spiComplete(&hspi3));

	HostRtos::runUntilBlocked(task); // processes the angle, waits for the next request

	CHECK(enc.getPos() == ((0x40 << 10) | (0x01 << 4)));
	const HostTarget::SpiTransfer &read = HostTarget::spiTransfers(&hspi3).front();
	CHECK(read.call == HostTarget::SpiCall::TransmitReceive);
	CHECK(read.mode == HostTarget::BusMode::Dma);
	CHECK(read.tx == std::vector<uint8_t>{0x83, 0x00, 0x00, 0x00}); // read command of register 0x03
	REQUIRE(read.csAtStart.size() == 3);
	CHECK_FALSE(read.csAtStart[0]);			   // SPI3_SS1 asserted (low) during the transfer
	CHECK(HostTarget::spiCsLevels(&hspi3)[0]); // and released by the completion callback
	CHECK_FALSE(ext3_spi.isTaken()); // the semaphore taken for the transfer is given back by the completion
}

TEST_CASE_FIXTURE(TargetFixture, "a resumable thread continues after the blocking call it waited in") {
	struct TwoWaits : cpp_freertos::Thread {
		TwoWaits() : cpp_freertos::Thread("TWOWAITS", 64, 1) { Start(); }
		void Run() override {
			while (true) {
				request.Take(); // first wait
				stage = 1;
				WaitForNotification(); // second wait
				stage = 2;
			}
		}
		cpp_freertos::BinarySemaphore request{false};
		int stage = 0;
	} thread;
	HostRtos::makeResumable(thread.GetHandle());
	thread.request.Give();
	HostRtos::runUntilBlocked(thread);
	REQUIRE(thread.stage == 1);

	thread.Notify();
	HostRtos::runUntilBlocked(thread); // continues after WaitForNotification (not from the start of Run)

	CHECK(thread.stage == 2);
}

TEST_CASE_FIXTURE(TargetFixture, "a mock SPI device is called back at the begin, end and completion of its DMA receive") {
	MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]); // SPI2_NSS, active low
	uint8_t buf[2] = {0, 0};
	external_spi.receive_DMA(buf, 2, &dev);
	REQUIRE(HostTarget::spiTransferPending(&hspi2));
	HostTarget::spiQueueRx(&hspi2, {0x12, 0x34});

	REQUIRE(HostTarget::spiComplete(&hspi2));

	using Call = MockSPIDevice::Call;
	CHECK(dev.calls() == std::vector<Call>{Call::BeginTransfer, Call::EndTransfer, Call::RxCompleted});
	REQUIRE(dev.events.size() == 3);
	CHECK(dev.events[1].port == &external_spi);
	CHECK_FALSE(dev.events[1].csHigh); // CS asserted until the end of the transfer
	CHECK(dev.events[1].portTaken);	   // port semaphore held during the transfer
	CHECK(buf[0] == 0x12);
	CHECK(buf[1] == 0x34);
	CHECK(HostTarget::spiCsLevels(&hspi2)[csIndex(external_spi, dev.config().cs)]); // released (high)
	CHECK_FALSE(external_spi.isTaken());
}

TEST_CASE_FIXTURE(TargetFixture, "a mock I2C device is called back at the start, end and completion of its interrupt driven memory read") {
	HostTarget::I2cDevice &chip = HostTarget::i2cDevice(&hi2c1, 0x50);
	chip.setRegister(0x10, {0xAB});
	MockI2CDevice dev(&hi2c1);
	i2cport.takePort(&dev);
	uint8_t value = 0;
	REQUIRE(i2cport.readMemIT(&dev, 0x50, 0x10, I2C_MEMADD_SIZE_8BIT, &value, 1));

	REQUIRE(HostTarget::i2cComplete(&hi2c1));

	using Call = MockI2CDevice::Call;
	CHECK(dev.calls() == std::vector<Call>{Call::StartTransfer, Call::EndTransfer, Call::RxCompleted});
	CHECK(dev.events.front().port == &i2cport);
	CHECK(value == 0xAB);
	i2cport.freePort(&dev);
}

} // TEST_SUITE
