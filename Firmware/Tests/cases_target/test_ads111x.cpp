/*
 * ADS111X (FFBoard/UserExtensions/Src/ADS111X.cpp): the ADS1113/4/5 I2C ADC driver (register access, config register
 * with MUX / PGA / MODE / DR / OS bits) and ADS111X_AnalogSource, the analog source that converts its inputs one after
 * the other in its own thread (start a single shot conversion, poll the OS bit of the config register, read the
 * conversion register), with error recovery, commands and flash configuration.
 *
 * The bus is the fake HAL I2C (F5) under the real I2CPort (`i2cport` on hi2c1) and the I2C callbacks of
 * global_callbacks.cpp. The ADC is a scripted device at 0x48 with 16 bit big endian registers (pointer register
 * selects one register, no auto increment). Interrupt transfers stay pending until HostTarget::i2cComplete; the source
 * thread runs with HostRtos::runUntilBlocked. Its Delay(1) before each poll advances the simulated time by 1 ms.
 *
 * ADS111x datasheet (SBAS444) config register: OS bit 15 (write 1: start a single conversion; read 0: converting,
 * 1: idle), MUX bits 14:12 (000 AIN0-AIN1, 001 AIN0-AIN3, 010 AIN1-AIN3, 011 AIN2-AIN3, 1xx AINx-GND), PGA bits 11:9,
 * MODE bit 8 (1 = single shot), DR bits 7:5, COMP_MODE/POL/LAT bits 4:2, COMP_QUE bits 1:0 (11 = comparator off).
 *
 * Host-vs-target (FMA): values that pass the float scaling of AnalogAxisProcessing are compared with a tolerance of
 * 2 counts: the target's single precision FPU may contract the multiply-adds into VFMA (GCC -ffp-contract=fast),
 * the x86-64 host without -mfma does not, which can change the truncated integer by 1.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "HostRtos.h"
#include "TestHelpers.h"

#include "ADS111X.h"
#include "eeprom_addresses.h"

#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr uint8_t ADDR = 0x48;
constexpr uint16_t MUX_AIN0_AIN1 = 0x0000, MUX_AIN0_AIN3 = 0x1000, MUX_AIN1_AIN3 = 0x2000, MUX_AIN2_AIN3 = 0x3000;
constexpr uint16_t MUX_AIN0 = 0x4000, MUX_AIN1 = 0x5000, MUX_AIN2 = 0x6000, MUX_AIN3 = 0x7000;

/** Config register value written to start a single shot conversion (OS, MODE single shot, comparator off) */
constexpr uint16_t startConfig(uint16_t mux, uint16_t gain = 2, uint16_t rate = 4) {
	return static_cast<uint16_t>(0x8000 | mux | (gain << 9) | 0x0100 | (rate << 5) | 0x0003);
}

/** Single ended conversion as ADS111X_AnalogSource reports it before processing */
int32_t singleEnded(int16_t raw) { return (std::max<int16_t>(raw, 0) - 0x3fff) * 2 - 1; }

struct Commands {
	CommandHandler &handler;
	std::vector<CommandReply> replies;
	CommandStatus run(const std::string &name, CMDtype type, int64_t val = 0, int64_t adr = 0) {
		CmdHandlerCommanddef *def = handler.getCommandFromName(name);
		REQUIRE_MESSAGE(def != nullptr, "command " << name << " not registered");
		replies.clear();
		ParsedCommand parsed;
		parsed.cmdId = def->cmdId;
		parsed.type = type;
		parsed.val = val;
		parsed.adr = adr;
		parsed.target = &handler;
		return handler.command(parsed, replies);
	}
	int64_t get(const std::string &name) {
		REQUIRE(run(name, CMDtype::get) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies[0].val;
	}
};

struct AdsFixture : TargetFixture {
	HostTarget::I2cDevice &adc = HostTarget::i2cDevice(&hi2c1, ADDR);
	/** Conversion result per MUX setting, set by the device hook when a conversion is started */
	std::map<uint16_t, uint16_t> inputs;

	AdsFixture() {
		adc.registerWidth = 2;
		adc.autoIncrement = false;
		adc.setRegister16(0x00, 0x0000);
		adc.onTransfer = [this](HostTarget::I2cDevice &dev, const HostTarget::I2cTransfer &t) {
			if (t.call == HostTarget::I2cCall::MemWrite && t.memAddress == 0x01 && t.data.size() == 2) {
				const uint16_t config = static_cast<uint16_t>((t.data[0] << 8) | t.data[1]);
				auto it = inputs.find(config & 0x7000);
				if (it != inputs.end()) {
					dev.setRegister16(0x00, it->second);
				}
			}
		};
	}

	/** Config register values written to the ADC (start of each conversion) */
	std::vector<uint16_t> configWrites() const {
		std::vector<uint16_t> values;
		for (const HostTarget::I2cTransfer &t : HostTarget::i2cTransfers(&hi2c1)) {
			if (t.call == HostTarget::I2cCall::MemWrite && t.memAddress == 0x01 && t.result == HAL_OK) {
				REQUIRE(t.data.size() == 2);
				values.push_back(static_cast<uint16_t>((t.data[0] << 8) | t.data[1]));
			}
		}
		return values;
	}
	/** Number of started transfers that read a register */
	size_t reads(uint16_t reg) const {
		size_t n = 0;
		for (const HostTarget::I2cTransfer &t : HostTarget::i2cTransfers(&hi2c1)) {
			n += (t.call == HostTarget::I2cCall::MemRead && t.memAddress == reg && t.result == HAL_OK) ? 1 : 0;
		}
		return n;
	}
	/** Completes pending transfers and runs the thread until nothing is pending (at most `steps` completions) */
	void drive(ADS111X_AnalogSource &ads, int steps = 100) {
		HostRtos::runUntilBlocked(ads);
		for (int i = 0; i < steps && HostTarget::i2cTransferPending(&hi2c1); i++) {
			HostTarget::i2cComplete(&hi2c1);
			HostRtos::runUntilBlocked(ads);
		}
	}
	/**
	 * One complete conversion cycle over all inputs, the way the main class drives the source on the target:
	 * getAxes() requests the cycle (state idle -> beginSampling, notifies the thread), then drive() alternates
	 * HostTarget::i2cComplete (the I2C interrupt of each pending transfer: config write, OS polls, conversion read) and
	 * HostRtos::runUntilBlocked (the thread starts the next transfer) until no transfer is pending, i.e. the thread went
	 * idle again. The second getAxes() returns the axes of the finished cycle (it also requests the next cycle, which is
	 * not driven). Tests use it when they need the result of a whole cycle rather than its individual bus steps.
	 */
	std::vector<int32_t> cycle(ADS111X_AnalogSource &ads) {
		ads.getAxes();
		drive(ads);
		return *ads.getAxes();
	}
	/** Starts the source thread (initialize: port reset, idle) */
	void start(ADS111X_AnalogSource &ads) { HostRtos::runUntilBlocked(ads); }
};

} // namespace

TEST_SUITE("ADS111X") {

// ---------------------------------------------------------------- ADS111X driver

TEST_CASE_FIXTURE(AdsFixture, "an ADS111X takes the I2C port and configures it with the selected speed preset") {
	i2cport.setSpeedPreset(1); // 400 kb/s (no HAL call while the port has no users)
	const uint32_t inits = HostTarget::i2cInitCount(&hi2c1);
	{
		ADS111X ads(i2cport);
		CHECK(i2cport.getPortUsers() == 1);
		CHECK(HostTarget::i2cInitCount(&hi2c1) == inits + 1);
		CHECK(hi2c1.Init.ClockSpeed == 400000);
	}
	CHECK(i2cport.getPortUsers() == 0);
	CHECK(HostTarget::i2cDeInitCount(&hi2c1) == 1);
}

TEST_CASE_FIXTURE(AdsFixture, "starting a single ended conversion writes the config register with the input against GND") {
	ADS111X ads(i2cport);
	const uint16_t mux[] = {MUX_AIN0, MUX_AIN1, MUX_AIN2, MUX_AIN3};
	for (uint8_t ch = 0; ch < 4; ch++) {
		CAPTURE(ch);
		HostTarget::i2cClearTransfers(&hi2c1);
		ads.startConversion(ch, false);
		REQUIRE(HostTarget::i2cComplete(&hi2c1));
		const HostTarget::I2cTransfer &t = HostTarget::i2cTransfers(&hi2c1).front();
		CHECK(t.call == HostTarget::I2cCall::MemWrite);
		CHECK(t.mode == HostTarget::BusMode::Interrupt);
		CHECK(t.devAddress == (ADDR << 1));
		CHECK(t.memAddress == 0x01);
		CHECK(t.memAddSize == 1);
		CHECK(adc.register16(0x01) == startConfig(mux[ch]));
	}
}

TEST_CASE_FIXTURE(AdsFixture, "starting a differential conversion selects the documented input pairs") {
	ADS111X ads(i2cport);
	const uint16_t mux[] = {MUX_AIN0_AIN1, MUX_AIN2_AIN3, MUX_AIN0_AIN3, MUX_AIN1_AIN3};
	for (uint8_t ch = 0; ch < 4; ch++) {
		CAPTURE(ch);
		ads.startConversion(ch, true);
		REQUIRE(HostTarget::i2cComplete(&hi2c1));
		CHECK(adc.register16(0x01) == startConfig(mux[ch]));
	}
}

TEST_CASE_FIXTURE(AdsFixture, "a channel above 3 starts a conversion of channel 0") {
	ADS111X ads(i2cport);
	ads.startConversion(7, false);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(adc.register16(0x01) == startConfig(MUX_AIN0));
	ads.startConversion(9, true);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(adc.register16(0x01) == startConfig(MUX_AIN0_AIN1));
}

TEST_CASE_FIXTURE(AdsFixture, "the gain is written to the PGA bits of the next conversion") {
	ADS111X ads(i2cport);
	ads.setGain(5);
	ads.startConversion(0, false);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(adc.register16(0x01) == startConfig(MUX_AIN0, 5));
	ads.setGain(0);
	ads.startConversion(0, false);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(adc.register16(0x01) == startConfig(MUX_AIN0, 0));
}

TEST_CASE_FIXTURE(AdsFixture, "a gain above 5 is limited to 5 (0.256 V)") {
	ADS111X ads(i2cport);
	ads.setGain(7);
	ads.startConversion(0, false);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(((adc.register16(0x01) >> 9) & 0x7) == 5);
}

TEST_CASE_FIXTURE(AdsFixture, "the data rate is written to the DR bits of the next conversion and limited to 7") {
	ADS111X ads(i2cport);
	ads.setDatarate(7);
	ads.startConversion(0, false);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(adc.register16(0x01) == startConfig(MUX_AIN0, 2, 7));
	ads.setDatarate(12);
	ads.startConversion(0, false);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(((adc.register16(0x01) >> 5) & 0x7) == 7);
	ads.setDatarate(0);
	ads.startConversion(0, false);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(adc.register16(0x01) == startConfig(MUX_AIN0, 2, 0));
}

TEST_CASE_FIXTURE(AdsFixture, "an ADS111X at another address is accessed at that address") {
	HostTarget::I2cDevice &second = HostTarget::i2cDevice(&hi2c1, 0x4B);
	second.registerWidth = 2;
	second.autoIncrement = false;
	ADS111X ads(i2cport, 0x4B);
	ads.startConversion(1, false);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(second.register16(0x01) == startConfig(MUX_AIN1));
	CHECK(adc.registers.count(0x01) == 0);
}

TEST_CASE_FIXTURE(AdsFixture, "an interrupt register read reads two bytes of the register into the buffer") {
	ADS111X ads(i2cport);
	adc.setRegister16(0x00, 0x1234);
	uint16_t value = 0;
	ads.readRegIT(0x00, &value);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	const HostTarget::I2cTransfer &t = HostTarget::i2cTransfers(&hi2c1).front();
	CHECK(t.call == HostTarget::I2cCall::MemRead);
	CHECK(t.mode == HostTarget::BusMode::Interrupt);
	CHECK(t.memAddress == 0x00);
	CHECK(t.size == 2);
	CHECK(t.data == std::vector<uint8_t>{0x12, 0x34}); // bus order, MSB first; the caller swaps the bytes
}

TEST_CASE_FIXTURE(AdsFixture, "a blocking register write sends the value MSB first") {
	ADS111X ads(i2cport);
	ads.writeReg(0x02, 0xA1B2);
	const HostTarget::I2cTransfer &t = HostTarget::i2cTransfers(&hi2c1).front();
	CHECK(t.call == HostTarget::I2cCall::MemWrite);
	CHECK(t.mode == HostTarget::BusMode::Blocking);
	CHECK(t.timeout == 100);
	CHECK(t.result == HAL_OK);
	CHECK(adc.register16(0x02) == 0xA1B2);
}

TEST_CASE_FIXTURE(AdsFixture, "an interrupt register write sends the value MSB first") {
	ADS111X ads(i2cport);
	ads.writeRegIT(0x03, 0x7FF0);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(adc.register16(0x03) == 0x7FF0);
}

TEST_CASE_FIXTURE(AdsFixture, "setting the thresholds starts the write of the low threshold register" *
				  KNOWN_ISSUE("bug 66: writeRegIT writes the shared writeItBuffer before taking the port semaphore "
							  "(ADS111X.cpp:31-40), so the high threshold overwrites the bytes of the pending low threshold "
							  "write (the HAL reads the buffer during the transfer): register 2 receives 0x2000. The CHECK on "
							  "register 2 fails")) {
	ADS111X ads(i2cport);
	ads.setThresh(0x1000, 0x2000); // the second write waits for the port on the target (not modelled: no task)
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	const HostTarget::I2cTransfer &t = HostTarget::i2cTransfers(&hi2c1).front();
	CHECK(t.memAddress == 0x02);
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(adc.register16(0x02) == 0x1000);
}

TEST_CASE_FIXTURE(AdsFixture, "a blocking register read returns the register value" *
				  KNOWN_ISSUE("readReg returns the two bytes in bus order without swapping them (ADS111X.cpp:24-29), "
							  "unlike writeReg / writeRegIT (__REV16) and the conversion path (i2cRxCompleted): 0x8583 "
							  "reads as 0x8385. Latent: readReg has no caller. The CHECK on the value fails")) {
	ADS111X ads(i2cport);
	adc.setRegister16(0x01, 0x8583);
	const uint16_t value = ads.readReg(0x01);
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(value == 0x8583);
}

TEST_CASE_FIXTURE(AdsFixture, "a blocking register read of a device that does not answer returns 0") {
	adc.nackAddress = true;
	ADS111X ads(i2cport);
	CHECK(ads.readReg(0x01) == 0);
	CHECK(HostTarget::i2cTransfers(&hi2c1).front().result == HAL_ERROR);
}

// ---------------------------------------------------------------- analog source: conversion cycle

TEST_CASE_FIXTURE(AdsFixture, "a new ADS111X analog source has one single ended input with gain 2 and data rate 4") {
	ADS111X_AnalogSource ads;
	Commands c{ads};
	CHECK(c.get("inputs") == 1);
	CHECK(c.get("diff") == 0);
	CHECK(c.get("gain") == 2);
	CHECK(c.get("rate") == 4);
	CHECK(*ads.getAxes() == std::vector<int32_t>{0});
}

TEST_CASE_FIXTURE(AdsFixture, "the source thread resets the I2C port when it starts and waits for a request") {
	ADS111X_AnalogSource ads;
	const uint32_t inits = HostTarget::i2cInitCount(&hi2c1);
	start(ads);
	CHECK(HostTarget::i2cInitCount(&hi2c1) == inits + 1);
	CHECK(HostTarget::i2cTransfers(&hi2c1).empty());
}

TEST_CASE_FIXTURE(AdsFixture, "reading the axes starts a conversion of the first input") {
	ADS111X_AnalogSource ads;
	start(ads);
	ads.getAxes();
	HostRtos::runUntilBlocked(ads);
	CHECK(configWrites() == std::vector<uint16_t>{startConfig(MUX_AIN0)});
	CHECK(HostTarget::i2cTransferPending(&hi2c1));
}

TEST_CASE_FIXTURE(AdsFixture, "after the conversion start the source polls the config register 1 ms later") {
	ADS111X_AnalogSource ads;
	start(ads);
	ads.getAxes();
	HostRtos::runUntilBlocked(ads);
	const uint64_t t0 = HostPlatform::nowUs();
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	HostRtos::runUntilBlocked(ads);
	CHECK(reads(0x01) == 1);
	CHECK(HostPlatform::nowUs() - t0 == 1000);
}

TEST_CASE_FIXTURE(AdsFixture, "the source keeps polling while the OS bit reports a conversion in progress") {
	adc.readQueue = {{0x45, 0x83}, {0x45, 0x83}, {0xC5, 0x83}}; // OS 0, 0, then 1
	adc.setRegister16(0x00, 0x2000);
	ADS111X_AnalogSource ads;
	start(ads);
	std::vector<int32_t> axes = cycle(ads);
	CHECK(reads(0x01) == 3);
	CHECK(reads(0x00) == 1);
	CHECK(axes[0] == singleEnded(0x2000));
}

TEST_CASE_FIXTURE(AdsFixture, "the conversion register is read only after the OS bit reports the end of the conversion") {
	adc.readQueue = {{0x45, 0x83}};
	ADS111X_AnalogSource ads;
	start(ads);
	ads.getAxes();
	HostRtos::runUntilBlocked(ads);
	REQUIRE(HostTarget::i2cComplete(&hi2c1)); // config written
	HostRtos::runUntilBlocked(ads);
	REQUIRE(HostTarget::i2cComplete(&hi2c1)); // first poll: busy
	HostRtos::runUntilBlocked(ads);
	CHECK(reads(0x00) == 0);
	CHECK(reads(0x01) == 2);
}

TEST_CASE_FIXTURE(AdsFixture, "a single ended conversion of 0 is reported as the lowest axis value") {
	adc.setRegister16(0x00, 0x0000);
	ADS111X_AnalogSource ads;
	start(ads);
	CHECK(cycle(ads)[0] == -0x7fff);
}

TEST_CASE_FIXTURE(AdsFixture, "a single ended full scale conversion is reported as the highest axis value") {
	adc.setRegister16(0x00, 0x7FFF);
	ADS111X_AnalogSource ads;
	start(ads);
	CHECK(cycle(ads)[0] == 0x7fff);
}

TEST_CASE_FIXTURE(AdsFixture, "a negative single ended conversion is reported as the lowest axis value") {
	adc.setRegister16(0x00, 0xFF00); // slightly below GND
	ADS111X_AnalogSource ads;
	start(ads);
	CHECK(cycle(ads)[0] == -0x7fff);
}

TEST_CASE_FIXTURE(AdsFixture, "a single ended conversion is mapped linearly to the axis range") {
	adc.setRegister16(0x00, 0x4000);
	ADS111X_AnalogSource ads;
	start(ads);
	CHECK(cycle(ads)[0] == 1);
}

TEST_CASE_FIXTURE(AdsFixture, "a differential conversion is reported as the signed conversion value") {
	adc.setRegister16(0x00, 0xEDCC); // -0x1234
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("diff", CMDtype::set, 1) == CommandStatus::OK);
	start(ads);
	CHECK(cycle(ads)[0] == -0x1234);
}

TEST_CASE_FIXTURE(AdsFixture, "the lowest differential conversion is clipped to minus 0x7fff") {
	adc.setRegister16(0x00, 0x8000);
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("diff", CMDtype::set, 1) == CommandStatus::OK);
	start(ads);
	CHECK(cycle(ads)[0] == -0x7fff);
}

TEST_CASE_FIXTURE(AdsFixture, "a cycle converts every input in order and reports them as axes") {
	inputs = {{MUX_AIN0, 0x1000}, {MUX_AIN1, 0x2000}, {MUX_AIN2, 0x3000}, {MUX_AIN3, 0x7000}};
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("inputs", CMDtype::set, 4) == CommandStatus::OK);
	start(ads);
	std::vector<int32_t> axes = cycle(ads);
	CHECK(configWrites() == std::vector<uint16_t>{startConfig(MUX_AIN0), startConfig(MUX_AIN1), startConfig(MUX_AIN2), startConfig(MUX_AIN3)});
	CHECK(axes == std::vector<int32_t>{singleEnded(0x1000), singleEnded(0x2000), singleEnded(0x3000), singleEnded(0x7000)});
}

TEST_CASE_FIXTURE(AdsFixture, "in differential mode a cycle converts the pairs AIN0-AIN1 and AIN2-AIN3") {
	inputs = {{MUX_AIN0_AIN1, 0x0100}, {MUX_AIN2_AIN3, 0xFF00}};
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("diff", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(c.run("inputs", CMDtype::set, 2) == CommandStatus::OK);
	start(ads);
	std::vector<int32_t> axes = cycle(ads);
	CHECK(configWrites() == std::vector<uint16_t>{startConfig(MUX_AIN0_AIN1), startConfig(MUX_AIN2_AIN3)});
	CHECK(axes == std::vector<int32_t>{0x0100, -0x0100});
}

TEST_CASE_FIXTURE(AdsFixture, "gain and data rate set by command are used by the next conversions") {
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("gain", CMDtype::set, 4) == CommandStatus::OK);
	REQUIRE(c.run("rate", CMDtype::set, 6) == CommandStatus::OK);
	start(ads);
	cycle(ads);
	CHECK(configWrites() == std::vector<uint16_t>{startConfig(MUX_AIN0, 4, 6)});
}

TEST_CASE_FIXTURE(AdsFixture, "the next reading of the axes after a finished cycle starts a new cycle") {
	ADS111X_AnalogSource ads;
	start(ads);
	adc.setRegister16(0x00, 0x1000);
	REQUIRE(cycle(ads)[0] == singleEnded(0x1000));
	adc.setRegister16(0x00, 0x5000);
	CHECK(cycle(ads)[0] == singleEnded(0x5000));
	CHECK(configWrites().size() == 2);
}

TEST_CASE_FIXTURE(AdsFixture, "reading the axes during a cycle does not start another conversion") {
	ADS111X_AnalogSource ads;
	start(ads);
	ads.getAxes();
	HostRtos::runUntilBlocked(ads);
	ads.getAxes();
	ads.getAxes();
	drive(ads);
	CHECK(configWrites().size() == 1);
}

TEST_CASE_FIXTURE(AdsFixture, "the axes keep the previous values until a cycle is finished") {
	adc.setRegister16(0x00, 0x1000);
	ADS111X_AnalogSource ads;
	start(ads);
	REQUIRE(cycle(ads)[0] == singleEnded(0x1000));
	adc.setRegister16(0x00, 0x6000);
	ads.getAxes();
	HostRtos::runUntilBlocked(ads);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	HostRtos::runUntilBlocked(ads);
	CHECK(ads.getAxes()->at(0) == singleEnded(0x1000));
}

TEST_CASE_FIXTURE(AdsFixture, "no cycle starts while the I2C port is busy with another transfer") {
	ADS111X_AnalogSource ads;
	start(ads);
	HostTarget::i2cDevice(&hi2c1, 0x20).pointerRegister = false; // another device on the bus
	uint8_t other = 0;
	REQUIRE(HAL_I2C_Master_Receive_IT(&hi2c1, 0x20 << 1, &other, 1) == HAL_OK);
	ads.getAxes();
	HostRtos::runUntilBlocked(ads);
	CHECK(configWrites().empty());
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	ads.getAxes();
	drive(ads);
	CHECK(configWrites().size() == 1);
}

TEST_CASE_FIXTURE(AdsFixture, "the axis values pass the analog processing of the source") {
	adc.setRegister16(0x00, 0x7FFF);
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("min", CMDtype::setat, -0x7fff, 0) == CommandStatus::OK);
	REQUIRE(c.run("max", CMDtype::setat, 0, 0) == CommandStatus::OK);
	start(ads);
	CHECK(cycle(ads)[0] == 0x7fff); // above the manual range: clipped
	REQUIRE(c.run("rawval", CMDtype::get) == CommandStatus::OK);
	CHECK(c.replies.at(0).val == 0x7fff);
	adc.setRegister16(0x00, 0x0000); // single ended: -0x7fff, the minimum of the manual range
	CHECK_NEAR(cycle(ads)[0], -32767, 2); // margin independent (NEEDS REVIEW 86); tolerance for truncation / FMA
	adc.setRegister16(0x00, 0x2000); // single ended: -16383, inside the manual range
	const int32_t inside = cycle(ads)[0];
	CHECK(inside > -32767);
	CHECK(inside < 0x7fff);
}

TEST_CASE_FIXTURE(AdsFixture, "a register write completed while the source is idle does not wake the source thread") {
	ADS111X_AnalogSource ads;
	start(ads);
	ads.writeRegIT(0x02, 0x1234); // e.g. a threshold write through the driver interface
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(HostRtos::pendingNotifications(ads) == 0);
	HostRtos::runUntilBlocked(ads);
	CHECK(configWrites().empty());
}

TEST_CASE_FIXTURE(AdsFixture, "a register read completed while the source is idle does not change the axes") {
	adc.setRegister16(0x00, 0x7000);
	ADS111X_AnalogSource ads;
	start(ads);
	uint16_t value = 0;
	ads.readRegIT(0x00, &value);
	REQUIRE(HostTarget::i2cComplete(&hi2c1));
	CHECK(HostRtos::pendingNotifications(ads) == 0);
	CHECK(value == 0x0070); // bus order, not swapped by the idle source
	CHECK(*ads.getAxes() == std::vector<int32_t>{0});
}

// ---------------------------------------------------------------- analog source: errors and recovery

TEST_CASE_FIXTURE(AdsFixture, "a device that does not acknowledge the conversion start ends the cycle and resets the port") {
	ADS111X_AnalogSource ads;
	start(ads);
	adc.nackAddress = true;
	const uint32_t inits = HostTarget::i2cInitCount(&hi2c1);
	ads.getAxes();
	HostRtos::runUntilBlocked(ads);
	REQUIRE(HostTarget::i2cComplete(&hi2c1)); // NACK: HAL_I2C_ErrorCallback with AF
	CHECK(HostTarget::i2cTransfers(&hi2c1).back().errorCode == HAL_I2C_ERROR_AF);
	HostRtos::runUntilBlocked(ads);
	CHECK(HostTarget::i2cInitCount(&hi2c1) == inits + 1);
	CHECK_FALSE(HostTarget::i2cTransferPending(&hi2c1));
}

TEST_CASE_FIXTURE(AdsFixture, "after a NACK the next reading of the axes starts a new cycle at the first input") {
	inputs = {{MUX_AIN0, 0x1000}, {MUX_AIN1, 0x2000}};
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("inputs", CMDtype::set, 2) == CommandStatus::OK);
	start(ads);
	ads.getAxes();
	drive(ads, 3); // input 0 converted, conversion of input 1 started
	adc.nackAddress = true;
	drive(ads); // NACK
	adc.nackAddress = false;
	HostTarget::i2cClearTransfers(&hi2c1);
	std::vector<int32_t> axes = cycle(ads);
	CHECK(configWrites() == std::vector<uint16_t>{startConfig(MUX_AIN0), startConfig(MUX_AIN1)});
	CHECK(axes == std::vector<int32_t>{singleEnded(0x1000), singleEnded(0x2000)});
}

TEST_CASE_FIXTURE(AdsFixture, "a bus error while reading the conversion keeps the previous axis value") {
	adc.setRegister16(0x00, 0x1000);
	ADS111X_AnalogSource ads;
	start(ads);
	REQUIRE(cycle(ads)[0] == singleEnded(0x1000));
	adc.setRegister16(0x00, 0x6000);
	ads.getAxes();
	drive(ads, 2); // conversion started and polled: the conversion register read is pending
	REQUIRE(reads(0x00) == 2);
	REQUIRE(HostTarget::i2cFail(&hi2c1, HAL_I2C_ERROR_BERR));
	HostRtos::runUntilBlocked(ads);
	CHECK(ads.getAxes()->at(0) == singleEnded(0x1000));
}

TEST_CASE_FIXTURE(AdsFixture, "a conversion start rejected by a busy bus is recovered by the port reset 4 s later") {
	ADS111X_AnalogSource ads;
	start(ads);
	HostTarget::i2cSetBusBusy(&hi2c1, true);
	ads.getAxes();
	HostRtos::runUntilBlocked(ads);
	REQUIRE(HostTarget::i2cTransfers(&hi2c1).back().result == HAL_BUSY);
	HostTarget::i2cSetBusBusy(&hi2c1, false);
	HostPlatform::advanceMs(4000);
	ads.getAxes(); // not yet: the cycle is still waiting for the lost transfer
	HostRtos::runUntilBlocked(ads);
	CHECK(configWrites().empty());
	const uint32_t inits = HostTarget::i2cInitCount(&hi2c1);
	HostPlatform::advanceMs(1);
	ads.getAxes(); // more than 4 s since the last finished cycle: restart
	HostRtos::runUntilBlocked(ads);
	CHECK(HostTarget::i2cInitCount(&hi2c1) == inits + 1);
	adc.setRegister16(0x00, 0x3000);
	CHECK(cycle(ads)[0] == singleEnded(0x3000));
}

TEST_CASE_FIXTURE(AdsFixture, "a transfer that never completes is abandoned by the port reset after 4 s") {
	ADS111X_AnalogSource ads;
	start(ads);
	ads.getAxes();
	HostRtos::runUntilBlocked(ads);
	REQUIRE(HostTarget::i2cTransferPending(&hi2c1)); // the interrupt never comes
	HostPlatform::advanceMs(4001);
	ads.getAxes();
	HostRtos::runUntilBlocked(ads);
	CHECK_FALSE(HostTarget::i2cTransferPending(&hi2c1)); // HAL_I2C_Init dropped it
	adc.setRegister16(0x00, 0x5000);
	CHECK(cycle(ads)[0] == singleEnded(0x5000));
}

// NEEDS REVIEW 91: the 4 s timeout (ADS111X.cpp:327) is measured from the last finished cycle, not from the last
// request: when nobody reads the axes for more than 4 s (e.g. the main class does not poll the analog sources), the next
// getAxes starts a cycle and immediately abandons it (state none: port reset), so the first reading after a pause
// starts no conversion and resets the I2C bus of all devices. Intended?
TEST_CASE_FIXTURE(AdsFixture, "characterization: the first reading after more than 4 s without requests resets the port instead of converting") {
	ADS111X_AnalogSource ads;
	start(ads);
	cycle(ads);
	HostTarget::i2cClearTransfers(&hi2c1);
	const uint32_t inits = HostTarget::i2cInitCount(&hi2c1);
	HostPlatform::advanceMs(5000);
	ads.getAxes();
	drive(ads);
	REQUIRE_FALSE(HostTarget::i2cTransferPending(&hi2c1)); // nothing left pending that a later completion could finish
	CHECK(HostTarget::i2cTransfers(&hi2c1).empty());		// no transfer was started at all
	CHECK(configWrites().empty());
	CHECK(HostTarget::i2cInitCount(&hi2c1) == inits + 1);
}

TEST_CASE_FIXTURE(AdsFixture, "deleting the source during a transfer releases the I2C port") {
	{
		ADS111X_AnalogSource ads;
		start(ads);
		ads.getAxes();
		HostRtos::runUntilBlocked(ads);
		REQUIRE(HostTarget::i2cTransferPending(&hi2c1));
	}
	CHECK(i2cport.getPortUsers() == 0);
	CHECK(HostTarget::i2cDeInitCount(&hi2c1) == 1);
	CHECK_FALSE(HostTarget::i2cTransferPending(&hi2c1));
	// A new source can use the port (its semaphore was given back)
	HostTarget::i2cClearTransfers(&hi2c1);
	adc.setRegister16(0x00, 0x2000);
	ADS111X_AnalogSource ads;
	start(ads);
	CHECK(cycle(ads)[0] == singleEnded(0x2000));
}

TEST_CASE_FIXTURE(AdsFixture, "lowering the number of inputs during a conversion read is safe" * doctest::skip() *
				  doctest::description("KNOWN ISSUE (crash): setAxes resizes rawbuf (ADS111X.cpp:336-341) while the thread "
									   "is in a cycle; i2cRxCompleted then writes rawbuf[lastAxis] with the old index "
									   "(ADS111X.cpp:296): out of bounds (aborts the run with _GLIBCXX_ASSERTIONS; on the "
									   "target a write into the vector's spare capacity). Reachable with the inputs or diff "
									   "command while the source is polled. Skipped")) {
	inputs = {{MUX_AIN0, 0x1000}, {MUX_AIN1, 0x2000}, {MUX_AIN2, 0x3000}};
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("inputs", CMDtype::set, 3) == CommandStatus::OK);
	start(ads);
	ads.getAxes();
	drive(ads, 8); // inputs 0 and 1 converted, input 2 started and polled: its conversion register read is pending
	REQUIRE(reads(0x00) == 3);
	REQUIRE(c.run("inputs", CMDtype::set, 1) == CommandStatus::OK);
	drive(ads);
	CHECK(ads.getAxes()->size() == 1);
}

// ---------------------------------------------------------------- analog source: commands

TEST_CASE_FIXTURE(AdsFixture, "the ADS111X analog source registers its commands with the documented flags") {
	ADS111X_AnalogSource ads;
	CHECK(ads.getCommandFromName("inputs")->flags == (CMDFLAG_GET | CMDFLAG_SET));
	CHECK(ads.getCommandFromName("diff")->flags == (CMDFLAG_GET | CMDFLAG_SET));
	CHECK(ads.getCommandFromName("gain")->flags == (CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING));
	CHECK(ads.getCommandFromName("rate")->flags == (CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING));
	CHECK(ads.getCommandFromName("addr") == nullptr);
	for (const char *name : {"filter", "autocal", "values", "rawval", "min", "max"}) {
		CAPTURE(name);
		CHECK(ads.getCommandFromName(name) != nullptr);
	}
}

TEST_CASE_FIXTURE(AdsFixture, "the ADS111X analog source is the class ADS111X ADC with the command name adsAnalog") {
	ADS111X_AnalogSource ads;
	CHECK(std::string(ads.getInfo().name) == "ADS111X ADC");
	CHECK(ads.getInfo().id == CLSID_ANALOG_ADS111X);
	CHECK(std::string(ads.getCommandHandlerInfo()->clsname) == "adsAnalog");
	CHECK(ads.getClassType() == ClassType::Analogsource);
	CHECK(ADS111X_AnalogSource::isCreatable());
	CHECK(ads.getHelpstring() == "ADS1113/4/5 analog source");
}

TEST_CASE_FIXTURE(AdsFixture, "the inputs command sets 1 to 4 single ended inputs") {
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("inputs", CMDtype::set, 3) == CommandStatus::OK);
	CHECK(c.get("inputs") == 3);
	CHECK(ads.getAxes()->size() == 3);
	REQUIRE(c.run("inputs", CMDtype::set, 9) == CommandStatus::OK);
	CHECK(c.get("inputs") == 4);
	REQUIRE(c.run("inputs", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(c.get("inputs") == 1);
}

TEST_CASE_FIXTURE(AdsFixture, "differential mode limits the inputs to 2") {
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("inputs", CMDtype::set, 4) == CommandStatus::OK);
	REQUIRE(c.run("diff", CMDtype::set, 1) == CommandStatus::OK);
	CHECK(c.get("diff") == 1);
	CHECK(c.get("inputs") == 2);
	CHECK(ads.getAxes()->size() == 2);
	REQUIRE(c.run("inputs", CMDtype::set, 4) == CommandStatus::OK);
	CHECK(c.get("inputs") == 2);
}

TEST_CASE_FIXTURE(AdsFixture, "leaving differential mode keeps the number of inputs") {
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("diff", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(c.run("inputs", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE(c.run("diff", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(c.get("diff") == 0);
	CHECK(c.get("inputs") == 2);
}

TEST_CASE_FIXTURE(AdsFixture, "the gain command sets the gain limited to 5") {
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("gain", CMDtype::set, 1) == CommandStatus::OK);
	CHECK(c.get("gain") == 1);
	REQUIRE(c.run("gain", CMDtype::set, 6) == CommandStatus::OK);
	CHECK(c.get("gain") == 5);
}

TEST_CASE_FIXTURE(AdsFixture, "the gain info lists the input ranges of the 6 gains") {
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("gain", CMDtype::info) == CommandStatus::OK);
	REQUIRE(c.replies.size() == 1);
	CHECK(c.replies[0].reply == "6.144V:0,4.096V:1,2.048V:2,1.024V:3,0.512V:4,0.256V:5");
}

TEST_CASE_FIXTURE(AdsFixture, "the rate command sets the data rate limited to 7") {
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("rate", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(c.get("rate") == 0);
	REQUIRE(c.run("rate", CMDtype::set, 100) == CommandStatus::OK);
	CHECK(c.get("rate") == 7);
}

TEST_CASE_FIXTURE(AdsFixture, "the rate info lists the 8 data rates") {
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("rate", CMDtype::info) == CommandStatus::OK);
	REQUIRE(c.replies.size() == 1);
	CHECK(c.replies[0].reply == "8 SPS:0,16 SPS:1,32 SPS:2,64 SPS:3,128 SPS:4,250 SPS:5,475 SPS:6,860 SPS:7");
}

TEST_CASE_FIXTURE(AdsFixture, "the address command id is passed to the processing and not found") {
	ADS111X_AnalogSource ads;
	ParsedCommand parsed;
	parsed.cmdId = 4; // ADS111X_AnalogSource_commands::address (not registered)
	parsed.type = CMDtype::get;
	std::vector<CommandReply> replies;
	CHECK(ads.command(parsed, replies) == CommandStatus::NOT_FOUND);
}

TEST_CASE_FIXTURE(AdsFixture, "the values command of the source replies with the processed axes") {
	adc.setRegister16(0x00, 0x7FFF);
	ADS111X_AnalogSource ads;
	Commands c{ads};
	start(ads);
	cycle(ads);
	REQUIRE(c.run("values", CMDtype::get) == CommandStatus::OK);
	REQUIRE(c.replies.size() == 1);
	CHECK(c.replies[0].val == 0x7fff);
}

// ---------------------------------------------------------------- analog source: flash

TEST_CASE_FIXTURE(AdsFixture, "saving stores inputs, gain, rate, differential mode and the processing config in one word") {
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("diff", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(c.run("inputs", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE(c.run("gain", CMDtype::set, 5) == CommandStatus::OK);
	REQUIRE(c.run("rate", CMDtype::set, 6) == CommandStatus::OK);
	REQUIRE(c.run("filter", CMDtype::set, 1) == CommandStatus::OK);
	ads.saveFlash();
	CHECK(HostPlatform::flash().at(ADR_ADS111X_CONF1) == (2 | (5 << 3) | (6 << 6) | (1 << 9) | (0b10 << 10)));
}

TEST_CASE_FIXTURE(AdsFixture, "saving stores the manual range of the 4 inputs") {
	ADS111X_AnalogSource ads;
	Commands c{ads};
	REQUIRE(c.run("min", CMDtype::setat, -300, 3) == CommandStatus::OK);
	REQUIRE(c.run("max", CMDtype::setat, 400, 3) == CommandStatus::OK);
	ads.saveFlash();
	CHECK(HostPlatform::flash().at(ADR_ADS111X_MIN_3) == static_cast<uint16_t>(-300));
	CHECK(HostPlatform::flash().at(ADR_ADS111X_MAX_3) == 400);
}

TEST_CASE_FIXTURE(AdsFixture, "a new source restores its configuration from flash") {
	HostPlatform::flash()[ADR_ADS111X_CONF1] = 3 | (1 << 3) | (7 << 6) | (0b01 << 10);
	HostPlatform::flash()[ADR_ADS111X_MIN_1] = static_cast<uint16_t>(-50);
	HostPlatform::flash()[ADR_ADS111X_MAX_1] = 60;
	ADS111X_AnalogSource ads;
	Commands c{ads};
	CHECK(c.get("inputs") == 3);
	CHECK(c.get("diff") == 0);
	CHECK(c.get("gain") == 1);
	CHECK(c.get("rate") == 7);
	CHECK(c.get("autocal") == 1);
	CHECK(c.get("filter") == 0);
	REQUIRE(c.run("min", CMDtype::getat, 0, 1) == CommandStatus::OK);
	CHECK(c.replies.at(0).val == -50);
	REQUIRE(c.run("max", CMDtype::getat, 0, 1) == CommandStatus::OK);
	CHECK(c.replies.at(0).val == 60);
}

TEST_CASE_FIXTURE(AdsFixture, "a stored differential mode limits the stored inputs to 2") {
	HostPlatform::flash()[ADR_ADS111X_CONF1] = 4 | (1 << 9);
	ADS111X_AnalogSource ads;
	Commands c{ads};
	CHECK(c.get("diff") == 1);
	CHECK(c.get("inputs") == 2);
}

TEST_CASE_FIXTURE(AdsFixture, "stored gain and inputs out of range are limited") {
	HostPlatform::flash()[ADR_ADS111X_CONF1] = 7 | (7 << 3); // 7 inputs, gain 7
	ADS111X_AnalogSource ads;
	Commands c{ads};
	CHECK(c.get("inputs") == 4);
	CHECK(c.get("gain") == 5);
}

TEST_CASE_FIXTURE(AdsFixture, "a saved configuration is restored by a new source") {
	{
		ADS111X_AnalogSource ads;
		Commands c{ads};
		REQUIRE(c.run("inputs", CMDtype::set, 4) == CommandStatus::OK);
		REQUIRE(c.run("gain", CMDtype::set, 3) == CommandStatus::OK);
		REQUIRE(c.run("rate", CMDtype::set, 1) == CommandStatus::OK);
		REQUIRE(c.run("autocal", CMDtype::set, 1) == CommandStatus::OK);
		REQUIRE(c.run("filter", CMDtype::set, 1) == CommandStatus::OK);
		ads.saveFlash();
	}
	ADS111X_AnalogSource ads;
	Commands c{ads};
	CHECK(c.get("inputs") == 4);
	CHECK(c.get("diff") == 0);
	CHECK(c.get("gain") == 3);
	CHECK(c.get("rate") == 1);
	CHECK(c.get("autocal") == 1);
	CHECK(c.get("filter") == 1);
}

TEST_CASE_FIXTURE(AdsFixture, "an ADS111X analog source can be created and deleted through the analog source registry") {
	const class_entry<AnalogSource> *entry = nullptr;
	for (const auto &e : AnalogSource::all_analogsources) {
		if (e.info.id == CLSID_ANALOG_ADS111X) {
			entry = &e;
		}
	}
	REQUIRE(entry != nullptr);
	std::unique_ptr<AnalogSource> source(entry->create());
	REQUIRE(source != nullptr);
	CHECK(i2cport.getPortUsers() == 1);
	source.reset();
	CHECK(i2cport.getPortUsers() == 0);
}

} // TEST_SUITE
