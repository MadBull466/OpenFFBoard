// Closeout observations use the existing host bus models and public driver interfaces.
// Register numbers follow ADS111X.h/.cpp's local map; no independent device-spec claim.
#include "doctest.h"
#include "AxisFixture.h"
#include "CommandHelpers.h"
#include "HostRtos.h"
#include "TestHelpers.h"
#include "ADS111X.h"
#include "ShifterAnalog.h"
#include "EncoderBissC.h"
#include "SPI.h"
#include "cpp_target_config.h"
#include "thread.hpp"
#include <array>
#include <functional>
#include <utility>

namespace {
struct OnceTask : cpp_freertos::Thread {
	explicit OnceTask(std::function<void()> f) : Thread("CDXCLOSE", 128, 1), body(std::move(f)) {
		Start(); HostRtos::makeResumable(GetHandle());
	}
	void Run() override { body(); done = true; while (true) WaitForNotification(); }
	std::function<void()> body;
	bool done = false;
};
struct InputsBusesFixture : TargetFixture {
	HostTarget::I2cDevice &adc = HostTarget::i2cDevice(&hi2c1, 0x48);
	InputsBusesFixture() {
		adc.registerWidth = 2;
		adc.autoIncrement = false;
		// Host reset leaves GPIO low; establish inactive CS for unselected devices.
		for (auto &pin : external_spi.getCsPins()) pin.write(true);
	}
	// Completion is a distinct actor; permit blocking and interrupt implementations.
	void finishI2c(OnceTask &caller) {
		HostRtos::runUntilBlocked(caller);
		for (int step = 0; step < 4; ++step) {
			if (HostTarget::i2cTransferPending(&hi2c1)) HostTarget::i2cComplete(&hi2c1);
			if (!caller.done) HostRtos::runUntilBlocked(caller);
		}
	}
	enum class Mode { blocking, interrupt, dma };
	static void exchange(Mode mode, SPIDevice &dev, uint8_t *tx, uint8_t *rx) {
		switch (mode) {
		case Mode::blocking: external_spi.transmitReceive(tx, rx, 2, &dev, 10); break;
		case Mode::interrupt: external_spi.transmitReceive_IT(tx, rx, 2, &dev); break;
		case Mode::dma: external_spi.transmitReceive_DMA(tx, rx, 2, &dev); break;
		}
	}
	void retry(Mode mode, SPIDevice &dev, uint8_t *tx, uint8_t *rx) {
		HostTarget::spiQueueRx(&hspi2, {0xA6, 0x39});
		OnceTask caller([&] { exchange(mode, dev, tx, rx); });
		HostRtos::runUntilBlocked(caller);
		if (HostTarget::spiTransferPending(&hspi2)) HostTarget::spiComplete(&hspi2);
		REQUIRE_ALL_HAL_MODELLED();
		// First defect observation is delivered bytes, not a synthetic semaphore fault.
		CHECK(std::array<uint8_t, 2>{rx[0], rx[1]} == std::array<uint8_t, 2>{0xA6, 0x39});
		CHECK(caller.done);
		if (caller.done) {
			CHECK(HostTarget::spiTransfers(&hspi2).back().completed);
			CHECK(HostTarget::spiTransfers(&hspi2).back().tx == std::vector<uint8_t>{0x53, 0xC7});
			CHECK(HostTarget::spiTransfers(&hspi2).back().selectedCs(true) == 0);
			const auto &pin = external_spi.getCsPins()[0];
			CHECK(HostTarget::outputPin(pin.getPort(), pin.getPin()));
			CHECK_FALSE(external_spi.isTaken());
			bool acquired = false;
			OnceTask nextOwner([&] {
				external_spi.takeSemaphore();
				acquired = true;
				external_spi.giveSemaphore();
			});
			HostRtos::runUntilBlocked(nextOwner);
			REQUIRE_ALL_HAL_MODELLED();
			CHECK(acquired);
		}
	}
};
}
TEST_SUITE("CodexCloseoutInputsBuses") {
TEST_CASE_FIXTURE(InputsBusesFixture, "serialized ADS public writes deliver both distinct threshold registers") {
	ADS111X ads(i2cport);
	OnceTask writer([&] { ads.writeReg(2, 0x1000); ads.writeReg(3, 0x2000); });
	finishI2c(writer);
	REQUIRE(writer.done);
	CHECK(adc.register16(2) == 0x1000);
	CHECK(adc.register16(3) == 0x2000);
}
TEST_CASE_FIXTURE(InputsBusesFixture, "ADS setThresh preserves both distinct threshold values after actor completion" *
	KNOWN_ISSUE("bug 66 / CDX-011: the high write overwrites the pending low buffer before taking the semaphore; the first register CHECK fails")) {
	ADS111X ads(i2cport);
	adc.setRegister16(2, 0xBEEF); adc.setRegister16(3, 0xCAFE);
	OnceTask writer([&] { ads.setThresh(0x1000, 0x2000); });
	finishI2c(writer);
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(adc.register16(2) == 0x1000);
	CHECK(adc.register16(3) == 0x2000);
	CHECK(writer.done);
}
TEST_CASE_FIXTURE(InputsBusesFixture, "ADS interrupt helper delivers bus bytes into the exact guarded caller buffer") {
	ADS111X ads(i2cport);
	adc.setRegister16(0, 0x1234);
	std::array<uint16_t, 3> guarded{0xA1B2, 0xDEAD, 0xC3D4};
	ads.readRegIT(0, &guarded[1]);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	// Inspect object bytes to express bus order without host-endian assumptions.
	const auto *bytes = reinterpret_cast<const unsigned char *>(&guarded[1]);
	CHECK(bytes[0] == 0x12); CHECK(bytes[1] == 0x34);
	CHECK(guarded.front() == 0xA1B2); CHECK(guarded.back() == 0xC3D4);
}
TEST_CASE_FIXTURE(InputsBusesFixture, "G27 CS2 selects exact pin index one and preserves another device ownership" *
	KNOWN_ISSUE("bug 71: cspin only stores the command value; actual G27 CS remains pin index zero")) {
	SPIDevice unrelated(external_spi, external_spi.getCsPins()[2]);
	unrelated.clearChipSelect();
	ShifterAnalog shifter;
	auto *handler = CommandHandler::getHandlerFromClassName("shifter");
	REQUIRE(handler != nullptr);
	REQUIRE(CommandHelpers::run(*handler, "mode", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE(CommandHelpers::run(*handler, "cspin", CMDtype::set, 2) == CommandStatus::OK);
	for (auto &pin : external_spi.getCsPins()) pin.write(true);
	uint64_t buttons = 0;
	shifter.readButtons(&buttons);
	REQUIRE(HostTarget::spiTransferPending(&hspi2));
	const int selected = HostTarget::spiTransfers(&hspi2).back().selectedCs(true);
	REQUIRE(HostTarget::spiComplete(&hspi2));
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(selected == 1);
	CHECK_FALSE(external_spi.isPinFree(external_spi.getCsPins()[2]));
	CHECK(HostTarget::outputPin(external_spi.getCsPins()[2].getPort(), external_spi.getCsPins()[2].getPin()));
	CHECK_FALSE(external_spi.isPinFree(external_spi.getCsPins()[1]));
	CHECK(external_spi.isPinFree(external_spi.getCsPins()[0]));
}
TEST_CASE_FIXTURE(InputsBusesFixture, "SPI blocking timeout permits a subsequent exchange with delivered bytes") {
	SPIDevice dev(external_spi, external_spi.getCsPins()[0]);
	uint8_t tx[2]{0x53, 0xC7}, rx[2]{0x11, 0x22};
	HostTarget::spiFailNextBlocking(&hspi2, HAL_TIMEOUT);
	exchange(Mode::blocking, dev, tx, rx);
	REQUIRE(HostTarget::spiTransfers(&hspi2).back().result == HAL_TIMEOUT);
	retry(Mode::blocking, dev, tx, rx);
}
TEST_CASE_FIXTURE(InputsBusesFixture, "SPI exchanges deliver distinct bytes in each supported mode") {
	for (auto mode : {Mode::blocking, Mode::interrupt, Mode::dma}) {
		SUBCASE(mode == Mode::blocking ? "blocking" : mode == Mode::interrupt ? "interrupt" : "dma") {
			SPIDevice dev(external_spi, external_spi.getCsPins()[0]);
			uint8_t tx[2]{0x53, 0xC7}, rx[2]{0x11, 0x22};
			retry(mode, dev, tx, rx);
		}
	}
}
TEST_CASE_FIXTURE(InputsBusesFixture, "SPI failed DMA start permits subsequent exchange and delivers bytes" *
	KNOWN_ISSUE("bug 86: ignored failed start retains semaphore and busy HAL handle; retry cannot deliver bytes")) {
	SPIDevice dev(external_spi, external_spi.getCsPins()[0]);
	uint8_t tx[2]{0x53, 0xC7}, rx[2]{0x11, 0x22};
	HostTarget::spiFailNextDmaStart(&hspi2);
	exchange(Mode::dma, dev, tx, rx);
	REQUIRE(HostTarget::spiTransfers(&hspi2).back().result == HAL_ERROR);
	REQUIRE_FALSE(HostTarget::spiTransferPending(&hspi2));
	retry(Mode::dma, dev, tx, rx);
}
TEST_CASE_FIXTURE(InputsBusesFixture, "SPI rejected interrupt start permits subsequent exchange and delivers bytes" *
	KNOWN_ISSUE("bug 86: ignored HAL_BUSY retains semaphore and busy HAL handle; retry cannot deliver bytes")) {
	SPIDevice dev(external_spi, external_spi.getCsPins()[0]);
	uint8_t tx[2]{0x53, 0xC7}, rx[2]{0x11, 0x22};
	hspi2.State = HAL_SPI_STATE_BUSY_TX;
	exchange(Mode::interrupt, dev, tx, rx);
	REQUIRE(HostTarget::spiTransfers(&hspi2).back().result == HAL_BUSY);
	REQUIRE_FALSE(HostTarget::spiTransferPending(&hspi2));
	retry(Mode::interrupt, dev, tx, rx);
}
}
