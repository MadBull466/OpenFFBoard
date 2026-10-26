/*
 * Tests of the SPI bus wrappers (ffb_target_tests): SPIPort, SPIDevice (SPI.cpp) and SpiHandler (SpiHandler.cpp).
 *
 * The ports are the global ports of the target configuration on the fake HAL SPI (F4, host_target_bus.cpp):
 * external_spi (hspi2, reconfigurable, 42 MHz base clock), motor_spi (hspi1, not reconfigurable, 84 MHz) and, for
 * pin list tests, a local port on its own handle. The device is MockSPIDevice (records the calls the port makes on its
 * device; callBase keeps the semaphore and CS handling of SPIDevice) or a plain SPIDevice for the defaults.
 * MX_GPIO_Init leaves the SPI2 CS pins low, so the tests release the CS pin of a new device first (idle level) to see
 * the assertion. IT/DMA transfers stay pending until HostTarget::spiComplete / spiFail (callbacks in interrupt
 * context through global_callbacks.cpp).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "HostPlatform.h"
#include "HostRtos.h"
#include "MockSPIDevice.h"
#include "TestHelpers.h"

#include "SPI.h"
#include "SpiHandler.h"
#include "cpp_target_config.h"
#include "thread.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <new>
#include <string>
#include <utility>
#include <vector>

namespace {

using Call = MockSPIDevice::Call;
using HostTarget::BusMode;
using HostTarget::SpiCall;

/** Output level of a pin (ODR) */
bool level(const OutputPin &pin) { return HostTarget::outputPin(pin.getPort(), pin.getPin()); }

/** Index of a CS pin in the pin list of a port (-1 if it is not one of them) */
int csIndex(SPIPort &port, const OutputPin &pin) {
	auto &pins = port.getCsPins();
	auto it = std::find(pins.begin(), pins.end(), pin);
	return it == pins.end() ? -1 : static_cast<int>(it - pins.begin());
}

/** Same port and pin (GpioPin::operator== is not const) */
bool samePin(const OutputPin &a, const OutputPin &b) { return a.getPort() == b.getPort() && a.getPin() == b.getPin(); }
bool samePins(const std::vector<OutputPin> &a, const std::vector<OutputPin> &b) {
	return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), samePin);
}
size_t countPin(const std::vector<OutputPin> &list, const OutputPin &pin) {
	return static_cast<size_t>(std::count_if(list.begin(), list.end(), [&pin](const OutputPin &p) { return samePin(p, pin); }));
}

/** A pin that is not a CS pin of external_spi (the first CS pin of motor_spi, SPI1_SS1) */
OutputPin foreignPin() { return OutputPin(*SPI1_SS1_GPIO_Port, SPI1_SS1_Pin); }

/** One of the nine transfer functions of SPIPort */
struct Op {
	const char *name;
	SpiCall call;
	BusMode mode;
};
const std::vector<Op> &allOps() {
	static const std::vector<Op> ops = {
		{"transmit", SpiCall::Transmit, BusMode::Blocking},
		{"receive", SpiCall::Receive, BusMode::Blocking},
		{"transmitReceive", SpiCall::TransmitReceive, BusMode::Blocking},
		{"transmit_IT", SpiCall::Transmit, BusMode::Interrupt},
		{"receive_IT", SpiCall::Receive, BusMode::Interrupt},
		{"transmitReceive_IT", SpiCall::TransmitReceive, BusMode::Interrupt},
		{"transmit_DMA", SpiCall::Transmit, BusMode::Dma},
		{"receive_DMA", SpiCall::Receive, BusMode::Dma},
		{"transmitReceive_DMA", SpiCall::TransmitReceive, BusMode::Dma},
	};
	return ops;
}
const std::vector<Op> &asyncOps() {
	static const std::vector<Op> ops = [] {
		std::vector<Op> list;
		for (const Op &op : allOps()) {
			if (op.mode != BusMode::Blocking) {
				list.push_back(op);
			}
		}
		return list;
	}();
	return ops;
}

/** Starts (blocking: runs) a transfer of `size` bytes with the function `op` (blocking timeout 10) */
void startTransfer(SPIPort &port, SPIDevice *dev, const Op &op, uint8_t *tx, uint8_t *rx, uint16_t size) {
	switch (op.mode) {
	case BusMode::Blocking:
		switch (op.call) {
		case SpiCall::Transmit: port.transmit(tx, size, dev, 10); break;
		case SpiCall::Receive: port.receive(rx, size, dev, 10); break;
		case SpiCall::TransmitReceive: port.transmitReceive(tx, rx, size, dev, 10); break;
		}
		break;
	case BusMode::Interrupt:
		switch (op.call) {
		case SpiCall::Transmit: port.transmit_IT(tx, size, dev); break;
		case SpiCall::Receive: port.receive_IT(rx, size, dev); break;
		case SpiCall::TransmitReceive: port.transmitReceive_IT(tx, rx, size, dev); break;
		}
		break;
	case BusMode::Dma:
		switch (op.call) {
		case SpiCall::Transmit: port.transmit_DMA(tx, size, dev); break;
		case SpiCall::Receive: port.receive_DMA(rx, size, dev); break;
		case SpiCall::TransmitReceive: port.transmitReceive_DMA(tx, rx, size, dev); break;
		}
		break;
	}
}

/** Completion callback the device gets for a transfer function */
Call completionOf(const Op &op) {
	switch (op.call) {
	case SpiCall::Transmit: return Call::TxCompleted;
	case SpiCall::Receive: return Call::RxCompleted;
	default: return Call::TxRxCompleted;
	}
}

/**
 * A firmware-like task running `body` once, then waiting for notifications forever. Not resumable by default: a
 * blocking call that would wait forever leaves Run() (HostRtos::TaskBlocked), so `done` shows whether body finished.
 */
struct BodyTask : cpp_freertos::Thread {
	explicit BodyTask(std::function<void()> f, const char *name = "SPITASK") : cpp_freertos::Thread(name, 128, 1), body(std::move(f)) { Start(); }
	void Run() override {
		body();
		done = true;
		while (true) {
			WaitForNotification();
		}
	}
	std::function<void()> body;
	bool done = false;
};

/** True if a task could take the port semaphore now (it gives it back at once) */
bool semaphoreAvailable(SPIPort &port) {
	BodyTask probe([&port] {
		port.takeSemaphore();
		port.giveSemaphore();
	}, "SPIPROBE");
	HostRtos::runUntilBlocked(probe);
	return probe.done;
}

struct SpiFixture : TargetFixture {
	/** A mock device on the first free CS pin of external_spi, CS released (idle high, active low) */
	static MockSPIDevice *idleDevice(MockSPIDevice &dev) {
		dev.clearChipSelect();
		return &dev;
	}
	/** A device configuration that differs from the configuration of hspi2 (CPOL high, CPHA 2nd edge, prescaler 256) */
	static SPIConfig otherConfig(const OutputPin &cs) {
		SPIConfig config(cs);
		config.peripheral = hspi2.Init;
		config.peripheral.CLKPolarity = SPI_POLARITY_HIGH;
		config.peripheral.CLKPhase = SPI_PHASE_2EDGE;
		config.peripheral.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
		return config;
	}
	/** Ends a pending IT/DMA transfer (nothing for blocking transfers) */
	static void finish(const Op &op, SPI_HandleTypeDef *hspi) {
		if (op.mode != BusMode::Blocking) {
			REQUIRE(HostTarget::spiComplete(hspi));
		}
	}
	/** A local port with three CS pins of SPI2 on its own handle (a copy of hspi2 with another instance) */
	struct LocalPort {
		SPI_HandleTypeDef handle = hspi2;
		std::vector<OutputPin> pins = external_spi.getCsPins();
		SPIPort port;
		LocalPort() : port((handle.Instance = &handle, handle), pins, 42000000, true) {}
	};
};

} // namespace

TEST_SUITE("SPIPort") {

// --- CS pins ---

TEST_CASE_FIXTURE(SpiFixture, "a new SPI port offers all its CS pins as free pins") {
	LocalPort local;

	CHECK(samePins(local.port.getCsPins(), local.pins));
	CHECK(samePins(local.port.getFreeCsPins(), local.pins));
}

TEST_CASE_FIXTURE(SpiFixture, "reserving a free CS pin removes it from the free pins") {
	OutputPin pin = external_spi.getCsPins()[1];

	CHECK(external_spi.reserveCsPin(pin));

	CHECK_FALSE(external_spi.isPinFree(pin));
	CHECK(external_spi.getFreeCsPins().size() == 2);
	CHECK(external_spi.getCsPins().size() == 3);
}

TEST_CASE_FIXTURE(SpiFixture, "reserving a CS pin that is already reserved fails") {
	OutputPin pin = external_spi.getCsPins()[0];
	REQUIRE(external_spi.reserveCsPin(pin));

	CHECK_FALSE(external_spi.reserveCsPin(pin));
	CHECK(external_spi.getFreeCsPins().size() == 2);
}

TEST_CASE_FIXTURE(SpiFixture, "reserving a pin that is not a CS pin of the port fails") {
	CHECK_FALSE(external_spi.reserveCsPin(foreignPin()));
	CHECK(external_spi.getFreeCsPins().size() == 3);
}

TEST_CASE_FIXTURE(SpiFixture, "freeing a reserved CS pin makes it free again") {
	OutputPin pin = external_spi.getCsPins()[2];
	REQUIRE(external_spi.reserveCsPin(pin));

	CHECK(external_spi.freeCsPin(pin));

	CHECK(external_spi.isPinFree(pin));
	CHECK(external_spi.getFreeCsPins().size() == 3);
}

TEST_CASE_FIXTURE(SpiFixture, "freeing a CS pin that is already free fails and does not list it twice") {
	OutputPin pin = external_spi.getCsPins()[0];

	CHECK_FALSE(external_spi.freeCsPin(pin));

	CHECK(countPin(external_spi.getFreeCsPins(), pin) == 1);
	CHECK(external_spi.getFreeCsPins().size() == 3);
}

TEST_CASE_FIXTURE(SpiFixture, "freeing a pin that is not a CS pin of the port fails") {
	CHECK_FALSE(external_spi.freeCsPin(foreignPin()));
	CHECK(external_spi.getFreeCsPins().size() == 3);
}

TEST_CASE_FIXTURE(SpiFixture, "a pin that is not a CS pin of the port is not free") {
	CHECK_FALSE(external_spi.isPinFree(foreignPin()));
	CHECK(external_spi.isPinFree(external_spi.getCsPins()[0]));
}

TEST_CASE_FIXTURE(SpiFixture, "getCsPin returns the CS pin at an index of the pin list") {
	OutputPin *pin = external_spi.getCsPin(1);

	REQUIRE(pin != nullptr);
	CHECK(*pin == OutputPin(*SPI2_SS2_GPIO_Port, SPI2_SS2_Pin));
}

TEST_CASE_FIXTURE(SpiFixture, "getCsPin with an index beyond the pin list returns no pin") {
	CHECK(external_spi.getCsPin(3) == nullptr);
	CHECK(external_spi.getCsPin(0xFFFF) == nullptr);
}

TEST_CASE_FIXTURE(SpiFixture, "a port with free CS pins that is not used exclusively has free pins") {
	CHECK(external_spi.hasFreePins());
}

TEST_CASE_FIXTURE(SpiFixture, "a port used exclusively has no free pins") {
	external_spi.takeExclusive(true);

	CHECK_FALSE(external_spi.hasFreePins());

	external_spi.takeExclusive(false);
	CHECK(external_spi.hasFreePins());
}

TEST_CASE_FIXTURE(SpiFixture, "a port without CS pins has no free pins") {
	SPI_HandleTypeDef handle = hspi2;
	handle.Instance = &handle;
	SPIPort port(handle, {}, 42000000);

	CHECK_FALSE(port.hasFreePins());
}

// bug 67 (SPIButtons / SPI): the unit level view of the defect described by the KNOWN_ISSUE in test_spi_buttons.cpp
TEST_CASE_FIXTURE(SpiFixture, "a port whose CS pins are all reserved has no free pins" *
								  KNOWN_ISSUE("bug 67: SPIPort::hasFreePins (SPI.cpp:220-222) checks getCsPins().size(), the number of all CS "
											  "pins, instead of the free pins: it stays true with every pin reserved. The CHECK fails")) {
	for (const OutputPin &pin : std::vector<OutputPin>(external_spi.getCsPins())) {
		REQUIRE(external_spi.reserveCsPin(pin));
	}
	REQUIRE(external_spi.getFreeCsPins().empty());
	REQUIRE_ALL_HAL_MODELLED();

	CHECK_FALSE(external_spi.hasFreePins());
}

// --- Configuration and clock ---

TEST_CASE_FIXTURE(SpiFixture, "configurePort without a configuration leaves the port unchanged") {
	const SPI_InitTypeDef before = hspi2.Init;

	external_spi.configurePort(nullptr);

	CHECK(HostTarget::spiInitCount(&hspi2) == 0);
	CHECK(hspi2.Init.BaudRatePrescaler == before.BaudRatePrescaler);
}

TEST_CASE_FIXTURE(SpiFixture, "configurePort with the current configuration does not reinitialise the port") {
	SPI_InitTypeDef config = hspi2.Init;

	external_spi.configurePort(&config);

	CHECK(HostTarget::spiInitCount(&hspi2) == 0);
}

TEST_CASE_FIXTURE(SpiFixture, "configurePort with a new configuration reinitialises the port with it") {
	SPI_InitTypeDef config = hspi2.Init;
	config.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	config.CLKPhase = SPI_PHASE_2EDGE;

	external_spi.configurePort(&config);

	CHECK(HostTarget::spiInitCount(&hspi2) == 1);
	CHECK(hspi2.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_128);
	CHECK(hspi2.Init.CLKPhase == SPI_PHASE_2EDGE);
}

TEST_CASE_FIXTURE(SpiFixture, "a port reports the base clock and the HAL handle it was constructed with") {
	CHECK(external_spi.getBaseClk() == 42000000);
	CHECK(motor_spi.getBaseClk() == 84000000);
	CHECK(external_spi.getPortHandle() == &hspi2);
	CHECK(motor_spi.getPortHandle() == &hspi1);
}

TEST_CASE_FIXTURE(SpiFixture, "the closest prescaler to a reachable clock gives exactly that clock") {
	auto [prescaler, clock] = external_spi.getClosestPrescaler(10500000); // 42 MHz / 4

	CHECK(prescaler == SPI_BAUDRATEPRESCALER_4);
	CHECK(clock == doctest::Approx(10500000.0));
}

TEST_CASE_FIXTURE(SpiFixture, "a clock between two prescaler clocks selects the prescaler with the nearer clock") {
	// 42 MHz: /8 = 5.25 MHz, /16 = 2.625 MHz. 3 MHz is nearer to 2.625 MHz, 5 MHz nearer to 5.25 MHz
	CHECK(external_spi.getClosestPrescaler(3000000).first == SPI_BAUDRATEPRESCALER_16);
	CHECK(external_spi.getClosestPrescaler(5000000).first == SPI_BAUDRATEPRESCALER_8);
	CHECK(external_spi.getClosestPrescaler(5000000).second == doctest::Approx(5250000.0));
}

TEST_CASE_FIXTURE(SpiFixture, "the closest prescaler depends on the base clock of the port") {
	// 84 MHz / 8 = 10.5 MHz on motor_spi; on external_spi (42 MHz) that is prescaler 4
	CHECK(motor_spi.getClosestPrescaler(10500000).first == SPI_BAUDRATEPRESCALER_8);
	CHECK(motor_spi.getClosestPrescaler(1e9f).first == SPI_BAUDRATEPRESCALER_2);
	CHECK(motor_spi.getClosestPrescaler(1.0f).first == SPI_BAUDRATEPRESCALER_256);
}

TEST_CASE_FIXTURE(SpiFixture, "a maximum below the closest clock selects the closest clock below the maximum") {
	// 21 MHz (/2) would be exact, but the maximum is 20 MHz: 10.5 MHz (/4)
	auto [prescaler, clock] = external_spi.getClosestPrescaler(21000000, 0, 20000000);

	CHECK(prescaler == SPI_BAUDRATEPRESCALER_4);
	CHECK(clock == doctest::Approx(10500000.0));
}

TEST_CASE_FIXTURE(SpiFixture, "a minimum above the closest clock selects the closest clock above the minimum") {
	// 42 MHz / 256 = 164 kHz is closest to 100 kHz, but the minimum is 200 kHz: 328 kHz (/128)
	CHECK(external_spi.getClosestPrescaler(100000, 200000).first == SPI_BAUDRATEPRESCALER_128);
}

TEST_CASE_FIXTURE(SpiFixture, "a clock equal to a limit is outside the limits") {
	// The limits are exclusive: max = 21 MHz excludes 21 MHz (/2)
	CHECK(external_spi.getClosestPrescaler(21000000, 0, 21000000).first == SPI_BAUDRATEPRESCALER_4);
	CHECK(external_spi.getClosestPrescaler(164062.5f, 164062.5f).first == SPI_BAUDRATEPRESCALER_128);
}

// NEEDS REVIEW 105: getClosestPrescaler (SPI.cpp:318-326) starts with the first entry (prescaler 2, the fastest clock)
// and returns it when no prescaler gives a clock inside (min, max). The caller cannot tell that the limits were not met
// and gets the fastest clock, the opposite of a maximum. Intended? (No firmware caller passes limits today.)
TEST_CASE_FIXTURE(SpiFixture, "characterization: a clock range that no prescaler reaches selects prescaler 2") {
	auto [prescaler, clock] = external_spi.getClosestPrescaler(1000000, 50000, 100000); // 42 MHz / 256 = 164 kHz > 100 kHz

	CHECK(prescaler == SPI_BAUDRATEPRESCALER_2);
	CHECK(clock == doctest::Approx(21000000.0));
}

// --- Reconfiguration and exclusive use ---

TEST_CASE_FIXTURE(SpiFixture, "every transfer of a device reconfigures a reconfigurable port with the device configuration") {
	for (const Op &op : allOps()) {
		SUBCASE(op.name) {
			MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
			dev.setConfig(otherConfig(dev.config().cs));
			uint8_t tx[2] = {1, 2}, rx[2] = {0, 0};

			startTransfer(external_spi, &dev, op, tx, rx, 2);

			REQUIRE(HostTarget::spiTransfers(&hspi2).size() == 1);
			const HostTarget::SpiTransfer &t = HostTarget::spiTransfers(&hspi2).front();
			CHECK(HostTarget::spiInitCount(&hspi2) == 1);
			CHECK(t.init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_256);
			CHECK(t.init.CLKPolarity == SPI_POLARITY_HIGH);
			CHECK(t.init.CLKPhase == SPI_PHASE_2EDGE);
			finish(op, &hspi2);
		}
	}
}

TEST_CASE_FIXTURE(SpiFixture, "every transfer on a port used exclusively keeps the port configuration") {
	for (const Op &op : allOps()) {
		SUBCASE(op.name) {
			MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
			dev.setConfig(otherConfig(dev.config().cs));
			external_spi.takeExclusive(true);
			uint8_t tx[2] = {1, 2}, rx[2] = {0, 0};

			startTransfer(external_spi, &dev, op, tx, rx, 2);

			REQUIRE(HostTarget::spiTransfers(&hspi2).size() == 1);
			CHECK(HostTarget::spiInitCount(&hspi2) == 0);
			CHECK(HostTarget::spiTransfers(&hspi2).front().init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_32);
			finish(op, &hspi2);
		}
	}
}

TEST_CASE_FIXTURE(SpiFixture, "a transfer on a port that does not allow reconfiguration keeps the port configuration") {
	MockSPIDevice dev(motor_spi, motor_spi.getFreeCsPins()[1]);
	SPIConfig config = dev.config();
	config.peripheral.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	dev.setConfig(config);
	uint8_t tx[1] = {0x42};

	motor_spi.transmit(tx, 1, &dev, 10);
	motor_spi.transmit_DMA(tx, 1, &dev);
	REQUIRE(HostTarget::spiComplete(&hspi1));

	CHECK(HostTarget::spiInitCount(&hspi1) == 0);
	REQUIRE(HostTarget::spiTransfers(&hspi1).size() == 2);
	CHECK(HostTarget::spiTransfers(&hspi1)[0].init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_8);
	CHECK(HostTarget::spiTransfers(&hspi1)[1].init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_8);
}

TEST_CASE_FIXTURE(SpiFixture, "a transfer of a device with the configuration of the port does not reinitialise the port") {
	MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]); // copies hspi2.Init
	uint8_t tx[1] = {0x42};

	external_spi.transmit(tx, 1, &dev, 10);

	CHECK(HostTarget::spiInitCount(&hspi2) == 0);
}

TEST_CASE_FIXTURE(SpiFixture, "transfers of two devices with different configurations reconfigure the port for each device") {
	MockSPIDevice first(external_spi, external_spi.getFreeCsPins()[0]);
	MockSPIDevice second(external_spi, external_spi.getFreeCsPins()[0]);
	second.setConfig(otherConfig(second.config().cs));
	uint8_t tx[1] = {0x42};

	external_spi.transmit(tx, 1, &second, 10);
	external_spi.transmit(tx, 1, &first, 10);

	REQUIRE(HostTarget::spiTransfers(&hspi2).size() == 2);
	CHECK(HostTarget::spiTransfers(&hspi2)[0].init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_256);
	CHECK(HostTarget::spiTransfers(&hspi2)[1].init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_32);
	CHECK(HostTarget::spiInitCount(&hspi2) == 2);
}

// --- Transfers ---

TEST_CASE_FIXTURE(SpiFixture, "every transfer asserts the active low CS pin of the device during the transfer and releases it afterwards") {
	for (const Op &op : allOps()) {
		SUBCASE(op.name) {
			MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[1]); // SPI2_SS2
			idleDevice(dev);
			const int cs = csIndex(external_spi, dev.config().cs);
			uint8_t tx[2] = {1, 2}, rx[2] = {0, 0};

			startTransfer(external_spi, &dev, op, tx, rx, 2);
			REQUIRE(HostTarget::spiTransfers(&hspi2).size() == 1);
			CHECK(HostTarget::spiTransfers(&hspi2).front().csAtStart[cs] == false); // selected (the other SPI2 CS pins start low too: selectedCs() does not apply)
			CHECK(HostTarget::spiTransfers(&hspi2).front().call == op.call);
			CHECK(HostTarget::spiTransfers(&hspi2).front().mode == op.mode);
			finish(op, &hspi2);

			CHECK(HostTarget::spiTransfers(&hspi2).front().csAtEnd[cs] == false); // still selected at the end of the data
			CHECK(level(dev.config().cs));										 // released afterwards
		}
	}
}

TEST_CASE_FIXTURE(SpiFixture, "an active high CS pin is driven high during the transfer and low afterwards") {
	SPIConfig config(external_spi.getFreeCsPins()[0], false); // cspol false: active high
	MockSPIDevice dev(external_spi, config);
	dev.clearChipSelect();
	REQUIRE_FALSE(level(dev.config().cs));
	const int cs = csIndex(external_spi, dev.config().cs);
	uint8_t tx[1] = {0x42}, rx[1] = {0};

	external_spi.transmitReceive_IT(tx, rx, 1, &dev);
	CHECK(HostTarget::spiTransfers(&hspi2).front().csAtStart[cs]);
	REQUIRE(HostTarget::spiComplete(&hspi2));

	CHECK_FALSE(level(dev.config().cs));
}

TEST_CASE_FIXTURE(SpiFixture, "a blocking transmit sends the buffer with the given timeout") {
	MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
	const uint8_t tx[3] = {0xA1, 0xB2, 0xC3};

	external_spi.transmit(tx, 3, &dev, 25);

	REQUIRE(HostTarget::spiTransfers(&hspi2).size() == 1);
	const HostTarget::SpiTransfer &t = HostTarget::spiTransfers(&hspi2).front();
	CHECK(t.tx == std::vector<uint8_t>{0xA1, 0xB2, 0xC3});
	CHECK(t.timeout == 25);
	CHECK(t.completed);
}

TEST_CASE_FIXTURE(SpiFixture, "a blocking receive stores the received bytes in the buffer") {
	MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
	HostTarget::spiQueueRx(&hspi2, {0x12, 0x34});
	uint8_t rx[2] = {0, 0};

	external_spi.receive(rx, 2, &dev, 7);

	CHECK(rx[0] == 0x12);
	CHECK(rx[1] == 0x34);
	CHECK(HostTarget::spiTransfers(&hspi2).front().timeout == 7);
}

TEST_CASE_FIXTURE(SpiFixture, "a blocking transmit-receive sends the transmit buffer and stores the received bytes") {
	MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
	HostTarget::spiQueueRx(&hspi2, {0x55, 0x66});
	const uint8_t tx[2] = {0x01, 0x02};
	uint8_t rx[2] = {0, 0};

	external_spi.transmitReceive(tx, rx, 2, &dev, 9);

	const HostTarget::SpiTransfer &t = HostTarget::spiTransfers(&hspi2).front();
	CHECK(t.tx == std::vector<uint8_t>{0x01, 0x02});
	CHECK(t.timeout == 9);
	CHECK(rx[0] == 0x55);
	CHECK(rx[1] == 0x66);
}

TEST_CASE_FIXTURE(SpiFixture, "an interrupt driven transmit-receive delivers the received bytes at its completion") {
	MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
	const uint8_t tx[2] = {0x01, 0x02};
	uint8_t rx[2] = {0, 0};
	external_spi.transmitReceive_IT(tx, rx, 2, &dev);
	HostTarget::spiQueueRx(&hspi2, {0x77, 0x88});
	REQUIRE(rx[0] == 0);

	REQUIRE(HostTarget::spiComplete(&hspi2));

	CHECK(rx[0] == 0x77);
	CHECK(rx[1] == 0x88);
	CHECK(HostTarget::spiTransfers(&hspi2).front().tx == std::vector<uint8_t>{0x01, 0x02});
}

TEST_CASE_FIXTURE(SpiFixture, "a blocking transfer begins and ends the transfer of its device and leaves the port free") {
	MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
	uint8_t tx[1] = {0x42};

	external_spi.transmit(tx, 1, &dev, 10);

	CHECK(dev.calls() == std::vector<Call>{Call::BeginTransfer, Call::EndTransfer});
	CHECK(dev.events[1].portTaken); // taken until the end of the transfer
	CHECK_FALSE(external_spi.isTaken());
	CHECK(semaphoreAvailable(external_spi));
}

TEST_CASE_FIXTURE(SpiFixture, "the port is taken during every interrupt driven or DMA transfer and free after its completion") {
	for (const Op &op : asyncOps()) {
		SUBCASE(op.name) {
			MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
			uint8_t tx[2] = {1, 2}, rx[2] = {0, 0};

			startTransfer(external_spi, &dev, op, tx, rx, 2);
			CHECK(external_spi.isTaken());
			CHECK_FALSE(semaphoreAvailable(external_spi));
			CHECK(dev.calls() == std::vector<Call>{Call::BeginTransfer});

			REQUIRE(HostTarget::spiComplete(&hspi2));
			CHECK_FALSE(external_spi.isTaken());
			CHECK(semaphoreAvailable(external_spi));
		}
	}
}

// NEEDS REVIEW 106: the blocking transfer functions return void (SPI.cpp:158-183): a HAL_TIMEOUT / HAL_ERROR of the HAL
// is not reported to the caller (e.g. SPIButtons, ShifterAnalog use the buffer as if it was received). Intended?
TEST_CASE_FIXTURE(SpiFixture, "a blocking transfer that fails in the HAL still releases the CS pin and the port") {
	MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
	idleDevice(dev);
	uint8_t rx[2] = {0xAA, 0xBB};
	HostTarget::spiFailNextBlocking(&hspi2, HAL_TIMEOUT);

	external_spi.receive(rx, 2, &dev, 5);

	REQUIRE(HostTarget::spiTransfers(&hspi2).front().result == HAL_TIMEOUT);
	CHECK(dev.calls() == std::vector<Call>{Call::BeginTransfer, Call::EndTransfer});
	CHECK(level(dev.config().cs));
	CHECK_FALSE(external_spi.isTaken());
	CHECK(semaphoreAvailable(external_spi));
}

// bug 86: the IT/DMA wrappers ignore the HAL return value
TEST_CASE_FIXTURE(SpiFixture, "a DMA transfer that the HAL fails to start releases the CS pin and the port" *
								  KNOWN_ISSUE("bug 86: SPIPort::transmit_DMA & co. (SPI.cpp:89-156) ignore the return value of "
											  "HAL_SPI_*_DMA / _IT. When the HAL fails to start the transfer (HAL_ERROR: DMA stream "
											  "start failed; HAL_BUSY) no completion or error callback follows, so the device keeps its "
											  "CS pin asserted, the port semaphore stays taken and current_device stays set: the next "
											  "transfer of any device waits forever. The first CHECK fails")) {
	MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
	idleDevice(dev);
	uint8_t tx[2] = {1, 2};
	HostTarget::spiFailNextDmaStart(&hspi2);

	external_spi.transmit_DMA(tx, 2, &dev);
	REQUIRE(HostTarget::spiTransfers(&hspi2).front().result == HAL_ERROR);
	REQUIRE_FALSE(HostTarget::spiTransferPending(&hspi2));
	REQUIRE_ALL_HAL_MODELLED();

	CHECK_FALSE(external_spi.isTaken());
	CHECK(level(dev.config().cs));
	CHECK(dev.count(Call::EndTransfer) == 1);
}

TEST_CASE_FIXTURE(SpiFixture, "an interrupt driven transfer rejected by a busy HAL releases the CS pin and the port" *
								  KNOWN_ISSUE("bug 86: SPIPort::receive_IT (SPI.cpp:148-156) ignores HAL_BUSY of HAL_SPI_Receive_IT "
											  "(the handle is still busy, e.g. after a failed DMA start the HAL leaves State BUSY_xx): "
											  "no callback follows, the CS pin stays asserted and the port semaphore stays taken. "
											  "The first CHECK fails")) {
	MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
	idleDevice(dev);
	uint8_t rx[2] = {0, 0};
	hspi2.State = HAL_SPI_STATE_BUSY_TX; // left busy by the HAL (failed DMA start) or another user of the handle

	external_spi.receive_IT(rx, 2, &dev);
	REQUIRE(HostTarget::spiTransfers(&hspi2).front().result == HAL_BUSY);
	REQUIRE_ALL_HAL_MODELLED();

	CHECK_FALSE(external_spi.isTaken());
	CHECK(level(dev.config().cs));
}

// --- Callback dispatch ---

TEST_CASE_FIXTURE(SpiFixture, "the completion of a transfer ends the transfer of the current device and reports the kind of transfer") {
	for (const Op &op : asyncOps()) {
		SUBCASE(op.name) {
			MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
			uint8_t tx[2] = {1, 2}, rx[2] = {0, 0};
			startTransfer(external_spi, &dev, op, tx, rx, 2);

			REQUIRE(HostTarget::spiComplete(&hspi2));

			CHECK(dev.calls() == std::vector<Call>{Call::BeginTransfer, Call::EndTransfer, completionOf(op)});
			for (const MockSPIDevice::Event &e : dev.events) {
				CHECK(e.port == &external_spi);
			}
			CHECK(dev.events[2].csHigh); // CS released before the device is told
		}
	}
}

TEST_CASE_FIXTURE(SpiFixture, "a transfer error ends the transfer of the current device and reports the error") {
	MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
	idleDevice(dev);
	uint8_t rx[2] = {0, 0};
	external_spi.receive_DMA(rx, 2, &dev);

	REQUIRE(HostTarget::spiFail(&hspi2, HAL_SPI_ERROR_DMA));

	CHECK(dev.calls() == std::vector<Call>{Call::BeginTransfer, Call::EndTransfer, Call::RequestError});
	CHECK(dev.events[2].port == &external_spi);
	CHECK(level(dev.config().cs));
	CHECK_FALSE(external_spi.isTaken());
}

TEST_CASE_FIXTURE(SpiFixture, "a completion or error is reported to the current device only once") {
	for (const Op &op : asyncOps()) {
		if (op.mode != BusMode::Interrupt) {
			continue;
		}
		for (bool fail : {false, true}) {
			SUBCASE((std::string(op.name) + (fail ? " error" : " completion")).c_str()) {
				MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
				uint8_t tx[1] = {0x42}, rx[1] = {0};
				startTransfer(external_spi, &dev, op, tx, rx, 1);
				REQUIRE((fail ? HostTarget::spiFail(&hspi2) : HostTarget::spiComplete(&hspi2)));
				REQUIRE(dev.events.size() == 3);

				{
					HostPlatform::IsrContext isr;
					HAL_SPI_TxCpltCallback(&hspi2);
					HAL_SPI_RxCpltCallback(&hspi2);
					HAL_SPI_TxRxCpltCallback(&hspi2);
					HAL_SPI_ErrorCallback(&hspi2);
				}

				CHECK(dev.events.size() == 3);
			}
		}
	}
}

TEST_CASE_FIXTURE(SpiFixture, "a completion or error without a transfer in progress calls no device") {
	MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);

	external_spi.SpiTxCplt(&hspi2);
	external_spi.SpiRxCplt(&hspi2);
	external_spi.SpiTxRxCplt(&hspi2);
	external_spi.SpiError(&hspi2);

	CHECK(dev.events.empty());
	CHECK_FALSE(external_spi.isTaken());
}

TEST_CASE_FIXTURE(SpiFixture, "a completion or error of another SPI port is not reported to the current device") {
	const std::vector<std::pair<const char *, void (SPIPort::*)(SPI_HandleTypeDef *)>> callbacks = {
		{"TxCplt", &SPIPort::SpiTxCplt}, {"RxCplt", &SPIPort::SpiRxCplt}, {"TxRxCplt", &SPIPort::SpiTxRxCplt}, {"Error", &SPIPort::SpiError}};
	for (const auto &callback : callbacks) {
		SUBCASE(callback.first) {
			MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
			uint8_t rx[2] = {0, 0};
			external_spi.receive_IT(rx, 2, &dev);

			(external_spi.*callback.second)(&hspi1); // other instance

			CHECK(dev.calls() == std::vector<Call>{Call::BeginTransfer});
			CHECK(external_spi.isTaken());
			REQUIRE(HostTarget::spiComplete(&hspi2)); // the own completion still reaches the device
			CHECK(dev.calls() == std::vector<Call>{Call::BeginTransfer, Call::EndTransfer, Call::RxCompleted});
		}
	}
}

TEST_CASE_FIXTURE(SpiFixture, "only the device that started the transfer is called back, not another device of the port") {
	MockSPIDevice first(external_spi, external_spi.getFreeCsPins()[0]);
	MockSPIDevice second(external_spi, external_spi.getFreeCsPins()[0]);
	uint8_t tx[1] = {0x42};
	external_spi.transmit(tx, 1, &first, 10);
	external_spi.transmit_DMA(tx, 1, &second);

	REQUIRE(HostTarget::spiComplete(&hspi2));

	CHECK(first.calls() == std::vector<Call>{Call::BeginTransfer, Call::EndTransfer});
	CHECK(second.calls() == std::vector<Call>{Call::BeginTransfer, Call::EndTransfer, Call::TxCompleted});
}

TEST_CASE_FIXTURE(SpiFixture, "the completion of one port reaches only the device of that port") {
	MockSPIDevice external(external_spi, external_spi.getFreeCsPins()[0]);
	MockSPIDevice motor(motor_spi, motor_spi.getFreeCsPins()[0]);
	uint8_t a[1] = {0}, b[1] = {0};
	external_spi.receive_IT(a, 1, &external);
	motor_spi.receive_IT(b, 1, &motor);

	REQUIRE(HostTarget::spiComplete(&hspi1));

	CHECK(motor.count(Call::RxCompleted) == 1);
	CHECK(external.calls() == std::vector<Call>{Call::BeginTransfer});
	CHECK(external_spi.isTaken());
	CHECK_FALSE(motor_spi.isTaken());
	REQUIRE(HostTarget::spiComplete(&hspi2));
	CHECK(external.count(Call::RxCompleted) == 1);
}

// --- Arbitration ---

TEST_CASE_FIXTURE(SpiFixture, "a transfer of a second device from a task waits until the transfer of the first device is complete") {
	MockSPIDevice first(external_spi, external_spi.getFreeCsPins()[0]);
	MockSPIDevice second(external_spi, external_spi.getFreeCsPins()[0]);
	idleDevice(first);
	idleDevice(second);
	uint8_t a[1] = {0x11}, b[1] = {0x22};
	external_spi.transmit_IT(a, 1, &first);
	BodyTask task([&] { external_spi.transmit(b, 1, &second, 10); });
	HostRtos::makeResumable(task.GetHandle());

	HostRtos::runUntilBlocked(task);
	CHECK_FALSE(task.done);
	CHECK(HostTarget::spiTransfers(&hspi2).size() == 1);
	CHECK(level(second.config().cs)); // not selected while waiting

	REQUIRE(HostTarget::spiComplete(&hspi2));
	HostRtos::runUntilBlocked(task);

	CHECK(task.done);
	REQUIRE(HostTarget::spiTransfers(&hspi2).size() == 2);
	CHECK(HostTarget::spiTransfers(&hspi2)[1].tx == std::vector<uint8_t>{0x22});
	CHECK(HostTarget::spiTransfers(&hspi2)[1].csAtStart[csIndex(external_spi, second.config().cs)] == false);
	CHECK(HostTarget::spiTransfers(&hspi2)[1].csAtStart[csIndex(external_spi, first.config().cs)]); // first released
	CHECK(first.count(Call::TxCompleted) == 1);
	CHECK_FALSE(external_spi.isTaken());
}

TEST_CASE_FIXTURE(SpiFixture, "a transfer can be started in interrupt context and completes like one started by a task") {
	MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
	uint8_t rx[1] = {0};
	HostTarget::spiQueueRx(&hspi2, {0x99});
	{
		HostPlatform::IsrContext isr;
		external_spi.receive_IT(rx, 1, &dev);
	}
	CHECK(external_spi.isTaken());
	CHECK_FALSE(semaphoreAvailable(external_spi));

	REQUIRE(HostTarget::spiComplete(&hspi2));

	CHECK(rx[0] == 0x99);
	CHECK(dev.count(Call::RxCompleted) == 1);
	CHECK(semaphoreAvailable(external_spi));
}

// bug 87: the completion callbacks clear current_device after calling the device
TEST_CASE_FIXTURE(SpiFixture, "a transfer that a device starts from its completion callback is completed to that device" *
								  KNOWN_ISSUE("bug 87: SPIPort::SpiTxCplt / SpiRxCplt / SpiTxRxCplt / SpiError (SPI.cpp:240-243, "
											  "254-257, 267-270, 281-284) set current_device = nullptr after calling the device. A "
											  "transfer the device starts in its callback sets current_device first, so it is lost: "
											  "the completion of the follow-up transfer calls nobody, its CS pin stays asserted and "
											  "the port semaphore stays taken. Latent: the firmware devices only notify a thread in "
											  "the callback. The first CHECK fails")) {
	struct ChainDevice : MockSPIDevice {
		using MockSPIDevice::MockSPIDevice;
		void spiTxCompleted(SPIPort *p) override {
			MockSPIDevice::spiTxCompleted(p);
			if (!chained) {
				chained = true;
				p->transmit_IT(next, 1, this);
			}
		}
		uint8_t next[1] = {0x5A};
		bool chained = false;
	} dev(external_spi, external_spi.getFreeCsPins()[0]);
	uint8_t first[1] = {0xA5};
	external_spi.transmit_IT(first, 1, &dev);
	REQUIRE(HostTarget::spiComplete(&hspi2));
	REQUIRE(HostTarget::spiTransferPending(&hspi2)); // the follow-up transfer started in the callback
	REQUIRE(dev.count(Call::TxCompleted) == 1);

	REQUIRE(HostTarget::spiComplete(&hspi2));
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(dev.count(Call::TxCompleted) == 2);
	CHECK_FALSE(external_spi.isTaken());
}

} // TEST_SUITE

TEST_SUITE("SPIDevice") {

TEST_CASE_FIXTURE(SpiFixture, "a device created with a CS pin reserves it, uses it active low and copies the port configuration") {
	OutputPin pin = external_spi.getCsPins()[2];

	SPIDevice dev(external_spi, pin);

	CHECK_FALSE(external_spi.isPinFree(pin));
	CHECK(dev.getSpiConfig()->cs == pin);
	CHECK(dev.getSpiConfig()->cspol); // active low
	CHECK(dev.getSpiConfig()->peripheral.BaudRatePrescaler == hspi2.Init.BaudRatePrescaler);
	CHECK(dev.getSpiConfig()->peripheral.CLKPolarity == hspi2.Init.CLKPolarity);
	CHECK(dev.getSpiConfig()->peripheral.Mode == SPI_MODE_MASTER);
}

TEST_CASE_FIXTURE(SpiFixture, "a device created with an SPI configuration reserves its CS pin and keeps its CS polarity") {
	OutputPin pin = external_spi.getCsPins()[1];
	SPIConfig config(pin, false);

	SPIDevice dev(external_spi, config);

	CHECK_FALSE(external_spi.isPinFree(pin));
	CHECK(dev.getSpiConfig()->cs == pin);
	CHECK_FALSE(dev.getSpiConfig()->cspol);
}

// NEEDS REVIEW 107: SPIDevice(port, SPIConfig&) (SPI.cpp:329-332) replaces the peripheral settings of the given
// configuration with the port's current configuration, so only cs and cspol of the argument are used. No firmware
// device uses this constructor. Intended (the comment-less code suggests "copy the port defaults")?
TEST_CASE_FIXTURE(SpiFixture, "characterization: a device created with an SPI configuration uses the port configuration instead of its peripheral settings") {
	SPIConfig config(external_spi.getCsPins()[1]);
	config.peripheral.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	config.peripheral.CLKPhase = SPI_PHASE_2EDGE;

	SPIDevice dev(external_spi, config);

	CHECK(dev.getSpiConfig()->peripheral.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_32); // hspi2
	CHECK(dev.getSpiConfig()->peripheral.CLKPhase == SPI_PHASE_1EDGE);
	CHECK(config.peripheral.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_256); // the argument is not changed
}

TEST_CASE_FIXTURE(SpiFixture, "a default SPI configuration is an 8 bit master configuration with an active low CS pin") {
	SPIConfig config(external_spi.getCsPins()[0]);

	CHECK(config.cspol);
	CHECK(config.peripheral.Mode == SPI_MODE_MASTER);
	CHECK(config.peripheral.Direction == SPI_DIRECTION_2LINES);
	CHECK(config.peripheral.DataSize == SPI_DATASIZE_8BIT);
	CHECK(config.peripheral.CLKPolarity == SPI_POLARITY_LOW);
	CHECK(config.peripheral.CLKPhase == SPI_PHASE_1EDGE);
	CHECK(config.peripheral.NSS == SPI_NSS_SOFT);
	CHECK(config.peripheral.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_4);
	CHECK(config.peripheral.FirstBit == SPI_FIRSTBIT_MSB);
}

TEST_CASE_FIXTURE(SpiFixture, "deleting a device frees its CS pin") {
	OutputPin pin = external_spi.getCsPins()[0];
	auto *dev = new SPIDevice(external_spi, pin);
	REQUIRE_FALSE(external_spi.isPinFree(pin));

	delete dev;

	CHECK(external_spi.isPinFree(pin));
	CHECK(external_spi.getFreeCsPins().size() == 3);
}

TEST_CASE_FIXTURE(SpiFixture, "updating the CS pin to the current pin succeeds without changes") {
	OutputPin pin = external_spi.getCsPins()[0];
	SPIDevice dev(external_spi, pin);

	CHECK(dev.updateCSPin(pin));

	CHECK(dev.getSpiConfig()->cs == pin);
	CHECK_FALSE(external_spi.isPinFree(pin));
	CHECK(external_spi.getFreeCsPins().size() == 2);
}

TEST_CASE_FIXTURE(SpiFixture, "updating the CS pin to a free pin moves the reservation to it") {
	OutputPin oldPin = external_spi.getCsPins()[0];
	OutputPin newPin = external_spi.getCsPins()[2];
	SPIDevice dev(external_spi, oldPin);

	CHECK(dev.updateCSPin(newPin));

	CHECK(dev.getSpiConfig()->cs == newPin);
	CHECK(external_spi.isPinFree(oldPin));
	CHECK_FALSE(external_spi.isPinFree(newPin));
}

TEST_CASE_FIXTURE(SpiFixture, "updating the CS pin to a reserved pin fails and keeps the current pin") {
	OutputPin pin = external_spi.getCsPins()[0];
	OutputPin taken = external_spi.getCsPins()[1];
	SPIDevice other(external_spi, taken);
	SPIDevice dev(external_spi, pin);

	CHECK_FALSE(dev.updateCSPin(taken));

	CHECK(dev.getSpiConfig()->cs == pin);
	CHECK_FALSE(external_spi.isPinFree(pin));
}

// NEEDS REVIEW 108: a device whose CS pin is not a CS pin of its port (the constructor ignores the failed reservation)
// can never move to a free CS pin: updateCSPin fails at freeCsPin(old pin) (SPI.cpp:350-352). Intended?
TEST_CASE_FIXTURE(SpiFixture, "characterization: updating the CS pin of a device whose pin is not a CS pin of the port fails") {
	SPIDevice dev(external_spi, foreignPin());
	OutputPin target = external_spi.getCsPins()[0];

	CHECK_FALSE(dev.updateCSPin(target));

	CHECK(dev.getSpiConfig()->cs == foreignPin());
	CHECK(external_spi.isPinFree(target));
}

TEST_CASE_FIXTURE(SpiFixture, "assertChipSelect drives the CS pin to its active level and clearChipSelect to its idle level") {
	SPIConfig highConfig(external_spi.getCsPins()[1], false);
	SPIDevice low(external_spi, external_spi.getCsPins()[0]);
	SPIDevice high(external_spi, highConfig);

	low.assertChipSelect();
	high.assertChipSelect();
	CHECK_FALSE(level(low.getSpiConfig()->cs));
	CHECK(level(high.getSpiConfig()->cs));

	low.clearChipSelect();
	high.clearChipSelect();
	CHECK(level(low.getSpiConfig()->cs));
	CHECK_FALSE(level(high.getSpiConfig()->cs));
}

TEST_CASE_FIXTURE(SpiFixture, "a device without overrides takes the port and drives its CS pin around an interrupt driven transfer") {
	SPIDevice dev(external_spi, external_spi.getCsPins()[0]);
	dev.clearChipSelect();
	const int cs = csIndex(external_spi, dev.getSpiConfig()->cs);
	uint8_t tx[1] = {0x42}, rx[1] = {0};

	external_spi.transmitReceive_DMA(tx, rx, 1, &dev);
	CHECK(external_spi.isTaken());
	CHECK(HostTarget::spiTransfers(&hspi2).front().csAtStart[cs] == false); // selected (the other SPI2 CS pins start low too: selectedCs() does not apply)
	REQUIRE(HostTarget::spiComplete(&hspi2)); // the empty completion callbacks of SPIDevice
	external_spi.receive_IT(rx, 1, &dev);
	REQUIRE(HostTarget::spiComplete(&hspi2));
	external_spi.transmit_IT(tx, 1, &dev);
	REQUIRE(HostTarget::spiFail(&hspi2));

	CHECK_FALSE(external_spi.isTaken());
	CHECK(level(dev.getSpiConfig()->cs));
	CHECK(semaphoreAvailable(external_spi));
}

// bug 57: ~SPIDevice does not clear the port's current_device
TEST_CASE_FIXTURE(SpiFixture, "a device deleted during its transfer is not called back by the port" *
								  KNOWN_ISSUE_UB("bug 57: ~SPIDevice (SPI.cpp:338-340) only frees the CS pin and leaves the port's "
												 "current_device pointing to the deleted device, so the completion calls a deleted "
												 "object. Here a new device is created at the same address (as a heap allocator "
												 "would reuse it) and receives the completion of the old transfer. Undefined "
												 "behaviour: skipped in sanitizer builds. The first CHECK fails")) {
	alignas(MockSPIDevice) unsigned char storage[sizeof(MockSPIDevice)];
	auto *dev = new (storage) MockSPIDevice(external_spi, external_spi.getCsPins()[0]);
	uint8_t rx[2] = {0, 0};
	external_spi.receive_DMA(rx, 2, dev);
	dev->~MockSPIDevice();
	auto *successor = new (storage) MockSPIDevice(external_spi, external_spi.getCsPins()[1]);

	REQUIRE(HostTarget::spiComplete(&hspi2));
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(successor->events.empty());
	successor->~MockSPIDevice();
}

} // TEST_SUITE

TEST_SUITE("SpiHandler") {

TEST_CASE_FIXTURE(SpiFixture, "a new SPI handler is added to the SPI handler list and removed when it is destroyed") {
	std::vector<SpiHandler *> &list = SpiHandler::getSPIHandlers();
	const size_t before = list.size();
	{
		struct Handler : SpiHandler {} handler;
		CHECK(list.size() == before + 1);
		CHECK(std::count(list.begin(), list.end(), static_cast<SpiHandler *>(&handler)) == 1);
	}
	CHECK(list.size() == before);
}

TEST_CASE_FIXTURE(SpiFixture, "an SPI port is an SPI handler of the SPI handler list") {
	std::vector<SpiHandler *> &list = SpiHandler::getSPIHandlers();
	for (SPIPort *port : {&external_spi, &motor_spi, &ext3_spi}) {
		CHECK(std::count(list.begin(), list.end(), static_cast<SpiHandler *>(port)) == 1);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "the SPI handler callbacks without overrides change nothing") {
	struct Handler : SpiHandler {} handler;
	const SPI_HandleTypeDef before = hspi2;

	handler.SpiTxCplt(&hspi2);
	handler.SpiRxCplt(&hspi2);
	handler.SpiTxRxCplt(&hspi2);
	handler.SpiTxHalfCplt(&hspi2);
	handler.SpiRxHalfCplt(&hspi2);
	handler.SpiTxRxHalfCplt(&hspi2);
	handler.SpiError(&hspi2);

	CHECK(hspi2.State == before.State);
	CHECK(hspi2.ErrorCode == before.ErrorCode);
	CHECK(HostTarget::spiTransfers(&hspi2).empty());
}

TEST_CASE_FIXTURE(SpiFixture, "the half transfer callbacks of a DMA transfer do not end the transfer of the device") {
	MockSPIDevice dev(external_spi, external_spi.getFreeCsPins()[0]);
	uint8_t rx[2] = {0, 0};
	external_spi.receive_DMA(rx, 2, &dev);

	{
		HostPlatform::IsrContext isr;
		HAL_SPI_RxHalfCpltCallback(&hspi2);
		HAL_SPI_TxHalfCpltCallback(&hspi2);
		HAL_SPI_TxRxHalfCpltCallback(&hspi2);
	}

	CHECK(dev.calls() == std::vector<Call>{Call::BeginTransfer});
	CHECK(external_spi.isTaken());
	REQUIRE(HostTarget::spiComplete(&hspi2));
}

} // TEST_SUITE
