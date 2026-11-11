/*
 * TmcFixture.h
 *
 * Fixture of the TMC4671 driver tests (ffb_driver_tests only, driver track D1, charter
 * Tests/Code_Coverage/claude/charters/driver/TMC4671_REQUIREMENTS.md "Fixture").
 *
 * TmcFixture is a TargetFixture (target state reset before and after the test, unmodelled HAL calls fail the test) and
 *  - clears the fake flash (HostPlatform::flash()) and sets all three SPI1 CS lines to their idle level (high),
 *  - owns one FakeTmc4671 per chip select: chip1 (PA4, TMC_1) and chip2 (PB0, TMC_2), installed on hspi1 after the
 *    target reset, so they exist before a driver object is constructed,
 *  - starts the ADC fake (startADC, VREFINT calibration) so powerOn() / powerOff() work,
 *  - makes the fake kernel take small delay steps for the "TMC" thread (setDelayBudgetMs(10)),
 *  - at teardown fails the test on any FakeTmc4671 violation the test did not declare with expectViolation().
 *
 * Supply voltage: hasPower() needs getIntV() and getExtV() > 10000 mV and getIntV() < 78000 mV. The values go through the
 * fake ADC2 (pattern claude/cases_target/test_voltagesense.cpp): analog reference 4095 mV (VREFINT_CAL 1365, reading 1100),
 * so one count = 1 mV at the pin and getIntV = counts * vSenseMult. The counts are computed with the CURRENT
 * vSenseMult (the driver changes it in setHwType), so call powerOn() / setSupply() again after a test changes the
 * hardware version if the same voltage is wanted.
 *
 * Threads: setupDriver() of the driver starts the thread "TMC". runThreadFor / runUntilState make it resumable (first
 * use) and activate it until the requested simulated time has passed. Simulated time advances through the thread's own
 * Delay() calls; if an activation does not advance time the helper advances 1 ms. Never sleeps.
 *
 * Shared helpers (public API of the driver only):
 *   reachShutdown(tmc)   chip present, power on, no encoder (the default enctype NONE), ADC raw words near 0x7FFF,
 *                        simulated time per SPI frame > 0 (busy loops on HAL_GetTick end), setupDriver(), run the
 *                        thread until getState() == Shutdown (bounded, fails with a message after 10 s simulated).
 *   reachRunning(tmc)    reachShutdown, startMotor(), run until getState() == Running and motorReady().
 *   runUntilState(tmc, state, maxMs)  generic bounded helper, true when the state was reached.
 * Commands: CommandHelpers.h (run / get / getAt) work directly on a TMC4671 (it is a CommandHandler).
 */
#ifndef HOSTTEST_TMCFIXTURE_H_
#define HOSTTEST_TMCFIXTURE_H_

#include "AxisFixture.h"
#include "CommandHelpers.h"
#include "../fakes/FakeTmc4671.h" // not an include directory: fakes/Axis.h would shadow the real Axis.h
#include "TestHelpers.h"

#include "TMC4671.h"
#include "global_callbacks.h"
#include "voltagesense.h"

#include <cmath>
#include <memory>
#include <set>
#include <sstream>

extern float vSenseMult; // voltagesense.cpp (not declared in the header)

struct TmcFixture : TargetFixture {
	FakeTmc4671 chip1{0}; //!< TMC_1, CS PA4
	FakeTmc4671 chip2{1}; //!< TMC_2, CS PB0

	TmcFixture() {
		HostPlatform::flash().clear();
		// CS lines idle (high). The target reset leaves SPI1_SS2 / SS3 low like MX_GPIO_Init of the F407VG
		// (main.c:1377), which would select two chips for every frame. See "Lead decisions needed" in the D1 report.
		HAL_GPIO_WritePin(SPI1_SS1_GPIO_Port, SPI1_SS1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SPI1_SS2_GPIO_Port, SPI1_SS2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SPI1_SS3_GPIO_Port, SPI1_SS3_Pin, GPIO_PIN_SET);
		HostRtos::setDelayBudgetMs(10);
		HostTarget::setAdcCalibration(1365, 950, 1200); // analog reference 4095 mV with VREFINT reading 1100
		startADC();
		HostTarget::adcSetInput(&hadc1, 7, 1100);
		HostTarget::adcSetInput(&hadc1, 6, 950);
		REQUIRE(HostTarget::adcConversionComplete(&hadc1));
	}

	~TmcFixture() override {
		for (FakeTmc4671 *c : {&chip1, &chip2}) {
			for (const auto &v : c->unexpectedViolations()) {
				FAIL_CHECK("TMC4671 fake on CS " << c->cs() << ": " << v.text());
			}
		}
		HostRtos::setDelayBudgetMs(1000);
	}

	// --- driver objects (construct after the fake exists: the constructor already talks to the chip) ---
	std::unique_ptr<TMC_1> makeTmc1() { return std::make_unique<TMC_1>(); }
	std::unique_ptr<TMC_2> makeTmc2() { return std::make_unique<TMC_2>(); }

	// --- supply ---
	/** Sets the external and internal supply in mV through the fake ADC2 (see header comment) */
	void setSupply(int32_t extMv, int32_t intMv) {
		supplyExt = extMv;
		supplyInt = intMv;
		refreshSupply();
	}
	/** Converts the last set voltages again with the current vSenseMult */
	void refreshSupply() {
		auto counts = [](int32_t mv) { return static_cast<uint32_t>(std::lround(static_cast<double>(mv) / vSenseMult)); };
		REQUIRE(HostTarget::adcConvert(&hadc2, {counts(supplyExt), counts(supplyInt)}));
	}
	/** 24 V on both rails: hasPower() true */
	void powerOn() { setSupply(24000, 24000); }
	/** 0 V: hasPower() false */
	void powerOff() { setSupply(0, 0); }

	// --- threads ---
	/** Activates the driver thread (made resumable on first use) until `ms` of simulated time have passed */
	void runThreadFor(TMC4671 &tmc, uint32_t ms) {
		if (!HostRtos::isResumable(tmc.GetHandle())) {
			HostRtos::makeResumable(tmc.GetHandle());
		}
		const uint64_t end = HostPlatform::nowUs() + uint64_t(ms) * 1000;
		while (HostPlatform::nowUs() < end) {
			uint64_t before = HostPlatform::nowUs();
			HostRtos::runUntilBlocked(tmc.GetHandle());
			if (HostPlatform::nowUs() == before) {
				HostPlatform::advanceMs(1);
			}
		}
	}

	/** Runs the thread until getState() == state or maxMs of simulated time have passed. @return true if reached */
	bool runUntilState(TMC4671 &tmc, TMC_ControlState state, uint32_t maxMs) {
		if (tmc.getState() == state) {
			return true;
		}
		for (uint32_t t = 0; t < maxMs; t += 10) {
			runThreadFor(tmc, 10);
			if (tmc.getState() == state) {
				return true;
			}
		}
		return tmc.getState() == state;
	}

	/** See the header comment. Fails the test with the state reached if Shutdown is not reached within 10 s */
	void reachShutdown(TMC4671 &tmc, FakeTmc4671 *chip = nullptr) {
		FakeTmc4671 &c = chip ? *chip : chip1;
		c.setPresent(true);
		c.setAdcRaw(0, 0x7FFF7FFF); // I1 [31:16] and I0 [15:0] at mid scale (DS 0x02 p77): offsets near zero current
		if (c.microsPerTransfer() == 0) {
			c.setMicrosPerTransfer(10);
		}
		powerOn();
		if (!started.count(&tmc)) {
			started.insert(&tmc);
			tmc.setupDriver();
		}
		bool ok = runUntilState(tmc, TMC_ControlState::Shutdown, 10000);
		std::ostringstream s;
		s << "reachShutdown: state " << static_cast<uint32_t>(tmc.getState()) << " after 10 s simulated";
		REQUIRE_MESSAGE(ok, s.str());
	}

	/** reachShutdown, startMotor(), run until Running and motorReady() */
	void reachRunning(TMC4671 &tmc, FakeTmc4671 *chip = nullptr) {
		reachShutdown(tmc, chip);
		tmc.startMotor();
		bool ok = runUntilState(tmc, TMC_ControlState::Running, 10000);
		std::ostringstream s;
		s << "reachRunning: state " << static_cast<uint32_t>(tmc.getState()) << " after 10 s simulated";
		REQUIRE_MESSAGE(ok, s.str());
		REQUIRE_MESSAGE(tmc.motorReady(), "reachRunning: state Running but motorReady() is false");
	}

private:
	int32_t supplyExt = 0;
	int32_t supplyInt = 0;
	std::set<TMC4671 *> started;
};

#endif
