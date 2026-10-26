/*
 * Tests of the I2C bus wrappers (ffb_target_tests): I2CPort, I2CDevice (I2C.cpp) and I2CHandler (I2CHandler.cpp).
 *
 * The port is the global i2cport of the target configuration (hi2c1, presets 100 kb/s and 400 kb/s, instance 0) on the
 * fake HAL I2C (F5, host_target_bus.cpp) with scripted bus devices (HostTarget::i2cDevice). The port device is
 * MockI2CDevice (records the calls of the port; callBase keeps the semaphore handling and the reset on errors of
 * I2CDevice) or a plain I2CDevice for the defaults. A device takes the port (takePort) before it uses it, like the
 * firmware devices. IT/DMA transfers stay pending until HostTarget::i2cComplete / i2cFail (callbacks in interrupt
 * context through global_callbacks.cpp). hi2c1 has no DMA stream (stm32f4xx_hal_msp.c) unless i2cSetDmaLinked.
 *
 * The port semaphore is not visible through the interface (isTaken() reads the HAL state): semaphoreAvailable() runs a
 * task that takes and gives it; the task leaves Run() at the take if the semaphore is taken (HostRtos::TaskBlocked).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "HostPlatform.h"
#include "HostRtos.h"
#include "MockI2CDevice.h"
#include "TestHelpers.h"

#include "I2C.h"
#include "I2CHandler.h"
#include "cpp_target_config.h"
#include "eeprom_addresses.h"
#include "thread.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace {

using Call = MockI2CDevice::Call;
using HostTarget::BusMode;
using HostTarget::I2cCall;

constexpr uint8_t CHIP = 0x50; // 7 bit address of the scripted bus device

/** A firmware-like task running `body` once, then waiting for notifications forever (see the file comment) */
struct BodyTask : cpp_freertos::Thread {
	explicit BodyTask(std::function<void()> f, const char *name = "I2CTASK") : cpp_freertos::Thread(name, 128, 1), body(std::move(f)) { Start(); }
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
bool semaphoreAvailable(I2CPort &port) {
	BodyTask probe([&port] {
		port.takeSemaphore();
		port.giveSemaphore();
	}, "I2CPROBE");
	HostRtos::runUntilBlocked(probe);
	return probe.done;
}

/** One of the interrupt driven / DMA transfer functions of I2CPort */
struct AsyncOp {
	const char *name;
	bool read;
	std::function<bool(I2CPort &, I2CDevice *, uint8_t *)> start; // 2 bytes at CHIP (memory address 0x10)
};
const std::vector<AsyncOp> &asyncOps() {
	static const std::vector<AsyncOp> ops = {
		{"transmitMasterIT", false, [](I2CPort &p, I2CDevice *d, uint8_t *b) { return p.transmitMasterIT(d, CHIP, b, 2); }},
		{"receiveMasterIT", true, [](I2CPort &p, I2CDevice *d, uint8_t *b) { return p.receiveMasterIT(d, CHIP, b, 2); }},
		{"writeMemIT", false, [](I2CPort &p, I2CDevice *d, uint8_t *b) { return p.writeMemIT(d, CHIP, 0x10, I2C_MEMADD_SIZE_8BIT, b, 2); }},
		{"readMemIT", true, [](I2CPort &p, I2CDevice *d, uint8_t *b) { return p.readMemIT(d, CHIP, 0x10, I2C_MEMADD_SIZE_8BIT, b, 2); }},
		{"transmitMasterDMA", false, [](I2CPort &p, I2CDevice *d, uint8_t *b) { return p.transmitMasterDMA(d, CHIP, b, 2); }},
		{"receiveMasterDMA", true, [](I2CPort &p, I2CDevice *d, uint8_t *b) { return p.receiveMasterDMA(d, CHIP, b, 2); }},
	};
	return ops;
}

/** One of the blocking transfer functions of I2CPort (timeout 10) */
struct BlockingOp {
	const char *name;
	std::function<bool(I2CPort &, I2CDevice *, uint8_t, uint8_t *)> run; // 2 bytes at the address
};
const std::vector<BlockingOp> &blockingOps() {
	static const std::vector<BlockingOp> ops = {
		{"transmitMaster", [](I2CPort &p, I2CDevice *d, uint8_t a, uint8_t *b) { return p.transmitMaster(d, a, b, 2, 10); }},
		{"receiveMaster", [](I2CPort &p, I2CDevice *d, uint8_t a, uint8_t *b) { return p.receiveMaster(d, a, b, 2, 10); }},
		{"writeMem", [](I2CPort &p, I2CDevice *d, uint8_t a, uint8_t *b) { return p.writeMem(d, a, 0x10, I2C_MEMADD_SIZE_8BIT, b, 2, 10); }},
		{"readMem", [](I2CPort &p, I2CDevice *d, uint8_t a, uint8_t *b) { return p.readMem(d, a, 0x10, I2C_MEMADD_SIZE_8BIT, b, 2, 10); }},
		{"isDeviceReady", [](I2CPort &p, I2CDevice *d, uint8_t a, uint8_t *) { return p.isDeviceReady(d, a, 3, 10); }},
	};
	return ops;
}

struct I2cFixture : TargetFixture {
	I2cFixture() { chip = &HostTarget::i2cDevice(&hi2c1, CHIP); }
	HostTarget::I2cDevice *chip = nullptr;

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
	const HostTarget::I2cTransfer &lastTransfer() {
		REQUIRE_FALSE(HostTarget::i2cTransfers(&hi2c1).empty());
		return HostTarget::i2cTransfers(&hi2c1).back();
	}
};

/** A second I2C port (instance 1) on its own handle with one preset; config initialised with preset 0 */
struct OtherPort {
	static const I2CPortHardwareConfig &hwConfig(bool canChangeSpeed) {
		static const std::array<I2CPortHardwareConfig::I2CPortHardwareConfig_preset, 1> presets{{{hi2c1.Init, "100kb/s"}}};
		static const I2CPortHardwareConfig fixed(false, presets), changeable(true, presets);
		return canChangeSpeed ? changeable : fixed;
	}
	explicit OtherPort(bool canChangeSpeed = false) : port(handle, hwConfig(canChangeSpeed), 1) {
		handle.Instance = &handle;
		port.setSpeedPreset(0);
	}
	I2C_HandleTypeDef handle = {};
	I2CPort port;
};

} // namespace

TEST_SUITE("I2CPort") {

// --- Users, speed presets and configuration ---

TEST_CASE_FIXTURE(I2cFixture, "a new I2C port has no users and speed preset 0") {
	OtherPort other;

	CHECK(other.port.getPortUsers() == 0);
	CHECK(other.port.getSpeedPreset() == 0);
	CHECK(i2cport.getSpeedPreset() == 0);
}

TEST_CASE_FIXTURE(I2cFixture, "the first user takes the port with the configuration of the selected speed preset") {
	MockI2CDevice dev;

	i2cport.takePort(&dev);

	CHECK(i2cport.getPortUsers() == 1);
	CHECK(HostTarget::i2cInitCount(&hi2c1) == 1);
	CHECK(hi2c1.Init.ClockSpeed == 100000);
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "a speed preset selected while the port has no users is applied when the first user takes the port") {
	MockI2CDevice dev;
	i2cport.setSpeedPreset(1);
	CHECK(HostTarget::i2cInitCount(&hi2c1) == 0);
	CHECK(hi2c1.Init.ClockSpeed == 100000);

	i2cport.takePort(&dev);

	CHECK(i2cport.getSpeedPreset() == 1);
	CHECK(hi2c1.Init.ClockSpeed == 400000);
	CHECK(HostTarget::i2cInitCount(&hi2c1) == 1);
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "a speed preset selected while the port is in use reinitialises the port with it") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);

	i2cport.setSpeedPreset(1);

	CHECK(hi2c1.Init.ClockSpeed == 400000);
	CHECK(HostTarget::i2cInitCount(&hi2c1) == 2);
	i2cport.setSpeedPreset(0);
	CHECK(hi2c1.Init.ClockSpeed == 100000);
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "a second user does not reinitialise the port") {
	MockI2CDevice first, second;
	i2cport.takePort(&first);

	i2cport.takePort(&second);

	CHECK(i2cport.getPortUsers() == 2);
	CHECK(HostTarget::i2cInitCount(&hi2c1) == 1);
	i2cport.freePort(&second);
	i2cport.freePort(&first);
}

TEST_CASE_FIXTURE(I2cFixture, "only the last user that frees the port deinitialises it") {
	MockI2CDevice first, second;
	i2cport.takePort(&first);
	i2cport.takePort(&second);

	i2cport.freePort(&first);
	CHECK(i2cport.getPortUsers() == 1);
	CHECK(HostTarget::i2cDeInitCount(&hi2c1) == 0);

	i2cport.freePort(&second);
	CHECK(i2cport.getPortUsers() == 0);
	CHECK(HostTarget::i2cDeInitCount(&hi2c1) == 1);
}

TEST_CASE_FIXTURE(I2cFixture, "freeing a port without users keeps the user count at zero") {
	MockI2CDevice dev;

	i2cport.freePort(&dev);

	CHECK(i2cport.getPortUsers() == 0);
	i2cport.takePort(&dev); // the next user is the first again
	CHECK(i2cport.getPortUsers() == 1);
	CHECK(HostTarget::i2cInitCount(&hi2c1) == 1);
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "freeing the port by the device of a pending transfer releases the port and drops its completion") {
	MockI2CDevice dev(&hi2c1), other;
	i2cport.takePort(&other);
	i2cport.takePort(&dev);
	uint8_t buf[2] = {0, 0};
	REQUIRE(i2cport.receiveMasterIT(&dev, CHIP, buf, 2));
	REQUIRE_FALSE(semaphoreAvailable(i2cport));

	i2cport.freePort(&dev);
	CHECK(semaphoreAvailable(i2cport));
	REQUIRE(HostTarget::i2cComplete(&hi2c1)); // the port is still in use by `other`: the transfer ends normally

	CHECK(dev.calls() == std::vector<Call>{Call::StartTransfer});
	i2cport.freePort(&other);
}

TEST_CASE_FIXTURE(I2cFixture, "configurePort without a configuration leaves the port unchanged") {
	i2cport.configurePort(nullptr);

	CHECK(HostTarget::i2cInitCount(&hi2c1) == 0);
}

TEST_CASE_FIXTURE(I2cFixture, "configurePort reinitialises the port with the given configuration") {
	I2C_InitTypeDef config = hi2c1.Init;
	config.ClockSpeed = 250000;
	config.NoStretchMode = I2C_NOSTRETCH_ENABLE;

	i2cport.configurePort(&config);

	CHECK(HostTarget::i2cInitCount(&hi2c1) == 1);
	CHECK(hi2c1.Init.ClockSpeed == 250000);
	CHECK(hi2c1.Init.NoStretchMode == I2C_NOSTRETCH_ENABLE);
}

TEST_CASE_FIXTURE(I2cFixture, "resetting the port reinitialises it with its configuration and releases it") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	i2cport.setSpeedPreset(1);
	uint8_t buf[2] = {0, 0};
	REQUIRE(i2cport.receiveMasterIT(&dev, CHIP, buf, 2));
	const uint32_t inits = HostTarget::i2cInitCount(&hi2c1);

	i2cport.resetPort();

	CHECK(HostTarget::i2cInitCount(&hi2c1) == inits + 1);
	CHECK(hi2c1.Init.ClockSpeed == 400000);
	CHECK_FALSE(HostTarget::i2cTransferPending(&hi2c1)); // the reinit stopped the transfer
	CHECK_FALSE(i2cport.isTaken());
	CHECK(semaphoreAvailable(i2cport));
	i2cport.freePort(&dev);
}

// --- Commands and flash ---

TEST_CASE_FIXTURE(I2cFixture, "the speed command returns the speed preset") {
	i2cport.setSpeedPreset(1);
	std::vector<CommandReply> replies;

	CHECK(cmd(i2cport, "speed", CMDtype::get, 0, &replies) == CommandStatus::OK);

	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 1);
}

TEST_CASE_FIXTURE(I2cFixture, "the speed command selects a speed preset") {
	CHECK(cmd(i2cport, "speed", CMDtype::set, 1) == CommandStatus::OK);

	CHECK(i2cport.getSpeedPreset() == 1);
}

TEST_CASE_FIXTURE(I2cFixture, "the speed command lists the presets by name with their number") {
	std::vector<CommandReply> replies;

	CHECK(cmd(i2cport, "speed", CMDtype::info, 0, &replies) == CommandStatus::OK);

	REQUIRE(replies.size() == 2);
	CHECK(replies[0].reply == "100kb/s:0");
	CHECK(replies[1].reply == "400kb/s:1");
}

// NEEDS REVIEW 109: on a port whose hardware cannot change the speed (canChangeSpeed false) the speed set is ignored,
// but the command returns OK (I2C.cpp:72-73), so the GUI shows no error. Intended? (The F407 port can change speed.)
TEST_CASE_FIXTURE(I2cFixture, "characterization: setting the speed of a port that cannot change its speed is ignored and returns OK") {
	OtherPort other(false);
	MockI2CDevice dev;
	other.port.takePort(&dev); // in use: an accepted preset would reinitialise the port
	const uint32_t inits = HostTarget::i2cInitCount(&other.handle);

	CHECK(cmd(other.port, "speed", CMDtype::set, 0) == CommandStatus::OK);

	CHECK(other.port.getSpeedPreset() == 0);
	CHECK(HostTarget::i2cInitCount(&other.handle) == inits);
	other.port.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "the speed command of a port that can change its speed accepts its only preset") {
	OtherPort other(true);
	MockI2CDevice dev;
	other.port.takePort(&dev);
	const uint32_t inits = HostTarget::i2cInitCount(&other.handle);

	CHECK(cmd(other.port, "speed", CMDtype::set, 0) == CommandStatus::OK);

	CHECK(HostTarget::i2cInitCount(&other.handle) == inits + 1);
	other.port.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "the speed command ignores request types it does not support") {
	std::vector<CommandReply> replies;

	CHECK(cmd(i2cport, "speed", CMDtype::setat, 1, &replies) == CommandStatus::OK);

	CHECK(replies.empty());
	CHECK(i2cport.getSpeedPreset() == 0);
}

TEST_CASE_FIXTURE(I2cFixture, "a command id the port does not know is not found") {
	ParsedCommand parsed;
	parsed.cmdId = 0x7F;
	parsed.type = CMDtype::get;
	parsed.target = &i2cport;
	std::vector<CommandReply> replies;

	CHECK(i2cport.command(parsed, replies) == CommandStatus::NOT_FOUND);
}

TEST_CASE_FIXTURE(I2cFixture, "the I2C port identifies itself as a port class") {
	CHECK(std::string(i2cport.getInfo().name) == "I2C port");
	CHECK(i2cport.getInfo().id == CLSID_I2CPORT);
	CHECK(i2cport.getClassType() == ClassType::Port);
}

TEST_CASE_FIXTURE(I2cFixture, "the first I2C port saves its speed preset to flash") {
	i2cport.setSpeedPreset(1);

	i2cport.saveFlash();

	REQUIRE(HostPlatform::flash().count(ADR_I2CCONF1) == 1);
	CHECK(HostPlatform::flash()[ADR_I2CCONF1] == 1);
}

TEST_CASE_FIXTURE(I2cFixture, "the first I2C port restores its speed preset from flash") {
	HostPlatform::flash()[ADR_I2CCONF1] = 1;

	i2cport.restoreFlash();

	CHECK(i2cport.getSpeedPreset() == 1);
}

TEST_CASE_FIXTURE(I2cFixture, "without a stored configuration the I2C port keeps its speed preset") {
	i2cport.setSpeedPreset(1);

	i2cport.restoreFlash();

	CHECK(i2cport.getSpeedPreset() == 1);
}

TEST_CASE_FIXTURE(I2cFixture, "a second I2C port neither saves nor restores its speed preset") {
	OtherPort other(true);
	HostPlatform::flash().erase(ADR_I2CCONF1);

	other.port.saveFlash();
	CHECK(HostPlatform::flash().count(ADR_I2CCONF1) == 0);

	HostPlatform::flash()[ADR_I2CCONF1] = 1;
	other.port.restoreFlash();
	CHECK(other.port.getSpeedPreset() == 0);
}

// --- Blocking transfers ---

TEST_CASE_FIXTURE(I2cFixture, "a blocking master transmit sends the data to the shifted 7 bit address") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	chip->pointerRegister = false; // plain device: every written byte is data
	uint8_t data[2] = {0x10, 0xAB};

	CHECK(i2cport.transmitMaster(&dev, CHIP, data, 2, 15));

	const HostTarget::I2cTransfer &t = lastTransfer();
	CHECK(t.call == I2cCall::MasterTransmit);
	CHECK(t.devAddress == CHIP << 1);
	CHECK(t.timeout == 15);
	CHECK(t.data == std::vector<uint8_t>{0x10, 0xAB});
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "every transfer function passes an unshifted address unchanged when asked to") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	uint8_t buf[2] = {0x10, 0x01};
	const uint16_t addr = CHIP << 1;

	CHECK(i2cport.transmitMaster(&dev, addr, buf, 2, 10, false));
	CHECK(i2cport.receiveMaster(&dev, addr, buf, 2, 10, false));
	CHECK(i2cport.writeMem(&dev, addr, 0x10, I2C_MEMADD_SIZE_8BIT, buf, 2, 10, false));
	CHECK(i2cport.readMem(&dev, addr, 0x10, I2C_MEMADD_SIZE_8BIT, buf, 2, 10, false));
	CHECK(i2cport.isDeviceReady(&dev, addr, 1, 10, false));
	CHECK(i2cport.transmitMasterIT(&dev, addr, buf, 2, false));
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(i2cport.receiveMasterIT(&dev, addr, buf, 2, false));
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(i2cport.writeMemIT(&dev, addr, 0x10, I2C_MEMADD_SIZE_8BIT, buf, 2, false));
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(i2cport.readMemIT(&dev, addr, 0x10, I2C_MEMADD_SIZE_8BIT, buf, 2, false));
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	HostTarget::i2cSetDmaLinked(&hi2c1, true);
	CHECK(i2cport.transmitMasterDMA(&dev, addr, buf, 2, false));
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(i2cport.receiveMasterDMA(&dev, addr, buf, 2, false));
	REQUIRE(HostTarget::i2cComplete(&hi2c1));

	REQUIRE(HostTarget::i2cTransfers(&hi2c1).size() == 11);
	for (const HostTarget::I2cTransfer &t : HostTarget::i2cTransfers(&hi2c1)) {
		CHECK(t.devAddress == addr);
		CHECK(t.address == CHIP);
	}
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "every interrupt driven or DMA transfer function shifts the 7 bit address by default") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	HostTarget::i2cSetDmaLinked(&hi2c1, true);
	uint8_t buf[2] = {0x10, 0x01};

	for (const AsyncOp &op : asyncOps()) {
		REQUIRE(op.start(i2cport, &dev, buf));
		REQUIRE(HostTarget::i2cComplete(&hi2c1));
		CHECK(lastTransfer().devAddress == CHIP << 1);
	}
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "a blocking master receive reads the bytes of the device") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	chip->readQueue.push_back({0x11, 0x22});
	uint8_t buf[2] = {0, 0};

	CHECK(i2cport.receiveMaster(&dev, CHIP, buf, 2, 10));

	CHECK(buf[0] == 0x11);
	CHECK(buf[1] == 0x22);
	CHECK(lastTransfer().call == I2cCall::MasterReceive);
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "a blocking memory write writes the data to the memory address") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	uint8_t data[1] = {0x5C};

	CHECK(i2cport.writeMem(&dev, CHIP, 0x21, I2C_MEMADD_SIZE_8BIT, data, 1, 10));

	CHECK(chip->registerBytes(0x21) == std::vector<uint8_t>{0x5C});
	CHECK(lastTransfer().memAddress == 0x21);
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "a blocking memory read reads the data at the memory address") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	chip->setRegister(0x0300, {0x7E});
	uint8_t data[1] = {0};

	CHECK(i2cport.readMem(&dev, CHIP, 0x0300, I2C_MEMADD_SIZE_16BIT, data, 1, 10));

	CHECK(data[0] == 0x7E);
	CHECK(lastTransfer().memAddSize == 2);
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "isDeviceReady is true for a device that acknowledges its address") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);

	CHECK(i2cport.isDeviceReady(&dev, CHIP, 4, 10));

	CHECK(lastTransfer().call == I2cCall::IsDeviceReady);
	CHECK(lastTransfer().trials == 4);
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "every blocking transfer to an address without a device returns false") {
	for (const BlockingOp &op : blockingOps()) {
		SUBCASE(op.name) {
			MockI2CDevice dev;
			i2cport.takePort(&dev);
			uint8_t buf[2] = {0x10, 0x01};

			CHECK_FALSE(op.run(i2cport, &dev, 0x51, buf));

			CHECK(lastTransfer().result != HAL_OK);
			i2cport.freePort(&dev);
		}
	}
}

TEST_CASE_FIXTURE(I2cFixture, "every blocking transfer starts and ends the transfer of its device and leaves the port free") {
	for (const BlockingOp &op : blockingOps()) {
		for (uint8_t address : {CHIP, uint8_t(0x51)}) { // acknowledged and not acknowledged
			SUBCASE((std::string(op.name) + (address == CHIP ? " ok" : " nack")).c_str()) {
				MockI2CDevice dev(&hi2c1);
				i2cport.takePort(&dev);
				uint8_t buf[2] = {0x10, 0x01};

				op.run(i2cport, &dev, address, buf);

				CHECK(dev.calls() == std::vector<Call>{Call::StartTransfer, Call::EndTransfer});
				CHECK(dev.events[0].port == &i2cport);
				CHECK_FALSE(i2cport.isTaken());
				CHECK(semaphoreAvailable(i2cport));
				i2cport.freePort(&dev);
			}
		}
	}
}

TEST_CASE_FIXTURE(I2cFixture, "a blocking transfer that times out in the HAL returns false and leaves the port free") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	HostTarget::i2cFailNextBlocking(&hi2c1, HAL_TIMEOUT);
	uint8_t data[1] = {0};

	CHECK_FALSE(i2cport.readMem(&dev, CHIP, 0x00, I2C_MEMADD_SIZE_8BIT, data, 1, 10));

	CHECK(semaphoreAvailable(i2cport));
	i2cport.freePort(&dev);
}

// --- Interrupt driven and DMA transfers ---

TEST_CASE_FIXTURE(I2cFixture, "every interrupt driven or DMA transfer keeps the port taken until its completion and then reports it") {
	for (const AsyncOp &op : asyncOps()) {
		SUBCASE(op.name) {
			MockI2CDevice dev(&hi2c1);
			i2cport.takePort(&dev);
			HostTarget::i2cSetDmaLinked(&hi2c1, true);
			uint8_t buf[2] = {0x10, 0x01};

			CHECK(op.start(i2cport, &dev, buf));
			CHECK(i2cport.isTaken());
			CHECK_FALSE(semaphoreAvailable(i2cport));
			CHECK(dev.calls() == std::vector<Call>{Call::StartTransfer});

			REQUIRE(HostTarget::i2cComplete(&hi2c1));

			CHECK(dev.calls() == std::vector<Call>{Call::StartTransfer, Call::EndTransfer, op.read ? Call::RxCompleted : Call::TxCompleted});
			CHECK(dev.events[2].port == &i2cport);
			CHECK_FALSE(i2cport.isTaken());
			CHECK(semaphoreAvailable(i2cport));
			i2cport.freePort(&dev);
		}
	}
}

TEST_CASE_FIXTURE(I2cFixture, "an interrupt driven memory read delivers the data at its completion") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	chip->setRegister(0x10, {0x3C});
	chip->setRegister(0x11, {0x4D});
	uint8_t buf[2] = {0, 0};
	REQUIRE(i2cport.readMemIT(&dev, CHIP, 0x10, I2C_MEMADD_SIZE_8BIT, buf, 2));
	CHECK(buf[0] == 0);

	REQUIRE(HostTarget::i2cComplete(&hi2c1));

	CHECK(buf[0] == 0x3C);
	CHECK(buf[1] == 0x4D);
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "an interrupt driven memory write writes the data at its completion") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	uint8_t buf[1] = {0x99};
	REQUIRE(i2cport.writeMemIT(&dev, CHIP, 0x05, I2C_MEMADD_SIZE_8BIT, buf, 1));

	REQUIRE(HostTarget::i2cComplete(&hi2c1));

	CHECK(chip->registerBytes(0x05) == std::vector<uint8_t>{0x99});
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "a DMA transfer on a port without DMA stream returns false") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	uint8_t buf[2] = {0, 0};

	CHECK_FALSE(i2cport.transmitMasterDMA(&dev, CHIP, buf, 2));
	CHECK_FALSE(i2cport.receiveMasterDMA(&dev, CHIP, buf, 2));

	CHECK(lastTransfer().errorCode == HAL_I2C_ERROR_DMA_PARAM);
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "an interrupt driven transfer to an address without a device reports an error to the device, which resets the port") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	uint8_t buf[2] = {0, 0};
	REQUIRE(i2cport.receiveMasterIT(&dev, 0x51, buf, 2)); // started, NACK of the address when it ends
	const uint32_t inits = HostTarget::i2cInitCount(&hi2c1);

	REQUIRE(HostTarget::i2cComplete(&hi2c1));

	CHECK(dev.calls() == std::vector<Call>{Call::StartTransfer, Call::Error});
	CHECK(dev.events[1].port == &i2cport);
	CHECK(HostTarget::i2cInitCount(&hi2c1) == inits + 1);
	CHECK(semaphoreAvailable(i2cport));
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "a bus error during an interrupt driven transfer is reported to the device only") {
	MockI2CDevice dev;
	dev.callBase = false;
	i2cport.takePort(&dev);
	uint8_t buf[1] = {0x01};
	REQUIRE(i2cport.transmitMasterIT(&dev, CHIP, buf, 1));
	const uint32_t inits = HostTarget::i2cInitCount(&hi2c1);

	REQUIRE(HostTarget::i2cFail(&hi2c1, HAL_I2C_ERROR_ARLO));

	CHECK(dev.calls() == std::vector<Call>{Call::StartTransfer, Call::Error});
	CHECK(HostTarget::i2cInitCount(&hi2c1) == inits); // the port does not reset by itself while a device is current
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "an I2C error without a transfer in progress resets the port") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	i2cport.setSpeedPreset(1);
	const uint32_t inits = HostTarget::i2cInitCount(&hi2c1);

	{
		HostPlatform::IsrContext isr;
		HAL_I2C_ErrorCallback(&hi2c1);
	}

	CHECK(HostTarget::i2cInitCount(&hi2c1) == inits + 1);
	CHECK(hi2c1.Init.ClockSpeed == 400000);
	CHECK(semaphoreAvailable(i2cport));
	i2cport.freePort(&dev);
}

// bug 58: the IT/DMA wrappers keep the semaphore when the HAL returns an error
TEST_CASE_FIXTURE(I2cFixture, "an interrupt driven transfer rejected by the HAL releases the port" *
								  KNOWN_ISSUE("bug 58: I2CPort::transmitMasterIT & co. (I2C.cpp:155-159, 175-179, 198-212) take the "
											  "port semaphore (startI2CTransfer) and return false when the HAL rejects the transfer "
											  "(here HAL_BUSY: bus busy), but never end the transfer: no callback follows and the "
											  "semaphore stays taken, so the next transfer of any device waits forever. The first "
											  "CHECK fails")) {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	HostTarget::i2cSetBusBusy(&hi2c1, true);
	uint8_t buf[1] = {0x01};

	REQUIRE_FALSE(i2cport.transmitMasterIT(&dev, CHIP, buf, 1));
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(semaphoreAvailable(i2cport));
	CHECK(dev.count(Call::EndTransfer) == 1);
	HostTarget::i2cSetBusBusy(&hi2c1, false);
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "a DMA transfer on a port without DMA stream releases the port" *
								  KNOWN_ISSUE("bug 58: I2CPort::receiveMasterDMA (I2C.cpp:169-173) returns false when the HAL "
											  "rejects the DMA transfer (hi2c1 has no DMA stream on the F407: HAL_ERROR, "
											  "HAL_I2C_ERROR_DMA_PARAM) but keeps the port semaphore: every later transfer waits "
											  "forever. The first CHECK fails")) {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	uint8_t buf[2] = {0, 0};

	REQUIRE_FALSE(i2cport.receiveMasterDMA(&dev, CHIP, buf, 2));
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(semaphoreAvailable(i2cport));
	i2cport.freePort(&dev);
}

// --- Callback dispatch ---

TEST_CASE_FIXTURE(I2cFixture, "a completion without a transfer in progress calls no device") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);

	i2cport.I2cTxCplt(&hi2c1);
	i2cport.I2cRxCplt(&hi2c1);

	CHECK(dev.events.empty());
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "a completion of another I2C port is not reported to the current device") {
	OtherPort other;
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	uint8_t buf[2] = {0, 0};
	REQUIRE(i2cport.receiveMasterIT(&dev, CHIP, buf, 2));

	i2cport.I2cTxCplt(&other.handle);
	i2cport.I2cRxCplt(&other.handle);

	CHECK(dev.calls() == std::vector<Call>{Call::StartTransfer});
	REQUIRE(HostTarget::i2cComplete(&hi2c1)); // the own completion still reaches the device
	CHECK(dev.count(Call::RxCompleted) == 1);
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "a completion is reported to the current device only once") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	uint8_t buf[1] = {0x01};
	REQUIRE(i2cport.transmitMasterIT(&dev, CHIP, buf, 1));
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	REQUIRE(dev.events.size() == 3);

	{
		HostPlatform::IsrContext isr;
		HAL_I2C_MasterTxCpltCallback(&hi2c1);
		HAL_I2C_MemRxCpltCallback(&hi2c1);
	}

	CHECK(dev.events.size() == 3);
	i2cport.freePort(&dev);
}

// bug 88: I2cError has no instance check
TEST_CASE_FIXTURE(I2cFixture, "an error of another I2C port is not reported to the device of this port" *
								  KNOWN_ISSUE("bug 88: I2CPort::I2cError (I2C.cpp:250-256) does not compare the handle with its own "
											  "(unlike I2cTxCplt / I2cRxCplt): an error of any I2C peripheral calls i2cError of this "
											  "port's current device (which resets this port and drops its transfer) or resets the "
											  "port. Latent on the F407 (one I2C port). The first CHECK fails")) {
	OtherPort other;
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	uint8_t buf[2] = {0, 0};
	REQUIRE(i2cport.receiveMasterIT(&dev, CHIP, buf, 2));
	REQUIRE_ALL_HAL_MODELLED();

	i2cport.I2cError(&other.handle);

	CHECK(dev.count(Call::Error) == 0);
	CHECK(HostTarget::i2cTransferPending(&hi2c1));
	i2cport.freePort(&dev);
}

// --- Arbitration ---

TEST_CASE_FIXTURE(I2cFixture, "a blocking transfer of a second device from a task waits until the transfer of the first device is complete") {
	MockI2CDevice first, second;
	i2cport.takePort(&first);
	i2cport.takePort(&second);
	uint8_t a[2] = {0, 0}, b[1] = {0x42};
	REQUIRE(i2cport.readMemIT(&first, CHIP, 0x10, I2C_MEMADD_SIZE_8BIT, a, 2));
	bool result = false;
	BodyTask task([&] { result = i2cport.transmitMaster(&second, CHIP, b, 1, 10); });
	HostRtos::makeResumable(task.GetHandle());

	HostRtos::runUntilBlocked(task);
	CHECK_FALSE(task.done);
	CHECK(HostTarget::i2cTransfers(&hi2c1).size() == 1);

	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	HostRtos::runUntilBlocked(task);

	CHECK(task.done);
	CHECK(result);
	REQUIRE(HostTarget::i2cTransfers(&hi2c1).size() == 2);
	CHECK(HostTarget::i2cTransfers(&hi2c1)[1].call == I2cCall::MasterTransmit);
	CHECK(semaphoreAvailable(i2cport));
	i2cport.freePort(&second);
	i2cport.freePort(&first);
}

// bug 89: the transfer functions set currentDevice before they take the port semaphore
TEST_CASE_FIXTURE(I2cFixture, "the completion of a transfer reaches its device while a second device waits for the port" *
								  KNOWN_ISSUE("bug 89: every I2CPort transfer function (I2C.cpp:141-220) sets currentDevice = device "
											  "before startI2CTransfer takes the port semaphore. A device of another task that waits "
											  "for the port (e.g. PCF8574 buttons and an ADS111X on i2cport) replaces the device of "
											  "the pending transfer: the completion ends the transfer of the waiting device and "
											  "reports it to that device, the device that started the transfer is never called back. "
											  "The first CHECK fails")) {
	MockI2CDevice first, second;
	i2cport.takePort(&first);
	i2cport.takePort(&second);
	uint8_t a[2] = {0, 0}, b[1] = {0x42};
	REQUIRE(i2cport.readMemIT(&first, CHIP, 0x10, I2C_MEMADD_SIZE_8BIT, a, 2));
	BodyTask task([&] { i2cport.transmitMaster(&second, CHIP, b, 1, 10); });
	HostRtos::makeResumable(task.GetHandle());
	HostRtos::runUntilBlocked(task); // waits for the port

	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(first.count(Call::RxCompleted) == 1);
	CHECK(second.count(Call::RxCompleted) == 0);
	HostRtos::runUntilBlocked(task);
	i2cport.freePort(&second);
	i2cport.freePort(&first);
}

TEST_CASE_FIXTURE(I2cFixture, "a transfer can be started in interrupt context and completes like one started by a task") {
	MockI2CDevice dev;
	i2cport.takePort(&dev);
	chip->setRegister(0x10, {0x61});
	uint8_t buf[1] = {0};
	bool started = false;
	{
		HostPlatform::IsrContext isr;
		started = i2cport.readMemIT(&dev, CHIP, 0x10, I2C_MEMADD_SIZE_8BIT, buf, 1);
	}
	REQUIRE(started);
	CHECK_FALSE(semaphoreAvailable(i2cport));

	REQUIRE(HostTarget::i2cComplete(&hi2c1));

	CHECK(buf[0] == 0x61);
	CHECK(dev.count(Call::RxCompleted) == 1);
	CHECK(semaphoreAvailable(i2cport));
	i2cport.freePort(&dev);
}

// bug 87 (I2C part): the completion callbacks clear currentDevice after calling the device
TEST_CASE_FIXTURE(I2cFixture, "a transfer that a device starts from its completion callback is completed to that device" *
								  KNOWN_ISSUE("bug 87: I2CPort::I2cTxCplt / I2cRxCplt (I2C.cpp:230-233, 244-247) set currentDevice = "
											  "nullptr after calling the device, so a transfer the device starts in its callback "
											  "loses its device: its completion calls nobody and the port semaphore stays taken. "
											  "Latent: PCF8574 and ADS111X only notify their thread in the callback. The first CHECK "
											  "fails")) {
	struct ChainDevice : MockI2CDevice {
		void i2cRxCompleted(I2CPort *p) override {
			MockI2CDevice::i2cRxCompleted(p);
			if (!chained) {
				chained = true;
				p->readMemIT(this, CHIP, 0x11, I2C_MEMADD_SIZE_8BIT, buf, 1);
			}
		}
		uint8_t buf[1] = {0};
		bool chained = false;
	} dev;
	i2cport.takePort(&dev);
	uint8_t first[1] = {0};
	REQUIRE(i2cport.readMemIT(&dev, CHIP, 0x10, I2C_MEMADD_SIZE_8BIT, first, 1));
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	REQUIRE(HostTarget::i2cTransferPending(&hi2c1)); // the follow-up transfer started in the callback

	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(dev.count(Call::RxCompleted) == 2);
	CHECK(semaphoreAvailable(i2cport));
	i2cport.freePort(&dev);
}

} // TEST_SUITE

TEST_SUITE("I2CDevice") {

TEST_CASE_FIXTURE(I2cFixture, "an I2C device without overrides takes the port for a transfer and gives it back at the end") {
	I2CDevice dev;
	i2cport.takePort(&dev);
	uint8_t buf[1] = {0x01};

	REQUIRE(i2cport.transmitMasterIT(&dev, CHIP, buf, 1));
	CHECK_FALSE(semaphoreAvailable(i2cport));
	REQUIRE(HostTarget::i2cComplete(&hi2c1)); // the empty completion callbacks of I2CDevice
	CHECK(semaphoreAvailable(i2cport));

	REQUIRE(i2cport.receiveMasterIT(&dev, CHIP, buf, 1));
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(semaphoreAvailable(i2cport));
	i2cport.freePort(&dev);
}

TEST_CASE_FIXTURE(I2cFixture, "an I2C device without overrides resets the port on a transfer error") {
	I2CDevice dev;
	i2cport.takePort(&dev);
	uint8_t buf[1] = {0x01};
	REQUIRE(i2cport.transmitMasterIT(&dev, CHIP, buf, 1));
	const uint32_t inits = HostTarget::i2cInitCount(&hi2c1);

	REQUIRE(HostTarget::i2cFail(&hi2c1, HAL_I2C_ERROR_BERR));

	CHECK(HostTarget::i2cInitCount(&hi2c1) == inits + 1);
	CHECK(semaphoreAvailable(i2cport));
	i2cport.freePort(&dev);
}

} // TEST_SUITE

TEST_SUITE("I2CHandler") {

TEST_CASE_FIXTURE(I2cFixture, "a new I2C handler is added to the I2C handler list") {
	// Removal when destroyed: KNOWN_ISSUE "a destroyed I2C handler is removed from the I2C handler list" (bug 53,
	// test_input_setup.cpp). The fixture restores the list
	struct Handler : I2CHandler {} handler;
	std::vector<I2CHandler *> &list = I2CHandler::getI2CHandlers();

	CHECK(std::count(list.begin(), list.end(), static_cast<I2CHandler *>(&handler)) == 1);
	CHECK(std::count(list.begin(), list.end(), static_cast<I2CHandler *>(&i2cport)) == 1);
}

TEST_CASE_FIXTURE(I2cFixture, "the I2C handler callbacks without overrides change nothing") {
	struct Handler : I2CHandler {} handler;
	const I2C_HandleTypeDef before = hi2c1;

	handler.I2cTxCplt(&hi2c1);
	handler.I2cRxCplt(&hi2c1);
	handler.I2cError(&hi2c1);

	CHECK(hi2c1.State == before.State);
	CHECK(hi2c1.ErrorCode == before.ErrorCode);
	CHECK(HostTarget::i2cInitCount(&hi2c1) == 0);
	CHECK(HostTarget::i2cTransfers(&hi2c1).empty());
}

} // TEST_SUITE
