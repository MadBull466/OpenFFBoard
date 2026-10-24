/*
 * Tests of voltagesense.cpp: ADC counts to millivolts (analog reference from VREFINT and its calibration value,
 * vSenseMult), internal and external voltage, the brake resistor control (thresholds, hysteresis, over-voltage,
 * stuck-on protection) and the chip temperature.
 *
 * The inputs go through the fake HAL ADC (F6): ADC1 rank 6 = temperature sensor, rank 7 = VREFINT; ADC2 rank 0 = VEXT,
 * rank 1 = VINT. An ADC2 conversion complete interrupt runs brakeCheck() through HAL_ADC_ConvCpltCallback
 * (global_callbacks.cpp), like on the target.
 *
 * Calibration used by most tests: VREFINT_CAL 1365 and a VREFINT reading of 1100 give an analog reference of
 * 1365 * 3300 / 1100 = 4095 mV, so one ADC count is exactly 1 mV at the pin. With vSenseMult 20 one count is 20 mV.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "ErrorHandler.h"
#include "global_callbacks.h"
#include "voltagesense.h"

#include <cstdint>

// voltagesense.h:28 declares setupBrakePin(uint32_t, int32_t, int32_t), but voltagesense.cpp:40 defines
// setupBrakePin(int32_t, int32_t, int32_t): in C++ these are two different functions and a call through the header
// does not link (suspected bug, see PROGRESS.md). The tests declare the defined function themselves.
void setupBrakePin(int32_t vdiffAct, int32_t vdiffDeact, int32_t vMax);

namespace {

constexpr uint16_t CAL_VREFINT_1MV = 1365; // with VREFINT_RAW_1MV: analog reference 4095 mV
constexpr uint32_t VREFINT_RAW_1MV = 1100;
constexpr float MULT = 20.0f;			   // mV per count with the reference above

struct VoltageFixture : TargetFixture {
	explicit VoltageFixture(uint16_t vrefintCal = CAL_VREFINT_1MV, uint32_t vrefintRaw = VREFINT_RAW_1MV) {
		HostTarget::setAdcCalibration(vrefintCal, 950, 1200);
		startADC(); // global_callbacks.cpp, called by cppmain() on the target
		HostTarget::adcSetInput(&hadc1, 7, vrefintRaw);
		HostTarget::adcSetInput(&hadc1, 6, 950); // TS_CAL1: 30 C
		REQUIRE(HostTarget::adcConversionComplete(&hadc1));
		setVSenseMult(MULT);
		HostPlatform::setTimeMs(1000); // brakeActiveTime 0 means "not braking" (see the KNOWN_ISSUE case)
	}

	/** One ADC2 conversion with the given voltages (multiples of 20 mV) at VEXT and VINT: runs brakeCheck() */
	static void convert(int32_t vextMv, int32_t vintMv) {
		REQUIRE(vextMv % 20 == 0);
		REQUIRE(vintMv % 20 == 0);
		REQUIRE(HostTarget::adcConvert(&hadc2, {(uint32_t)(vextMv / 20), (uint32_t)(vintMv / 20)}));
	}

	static bool brakeOn() { return HostTarget::outputPin(DRV_BRAKE_GPIO_Port, DRV_BRAKE_Pin); }

	/** One ADC1 conversion with the given chip temperature sensor reading (VREFINT unchanged) */
	static void convertTemp(uint32_t raw) {
		HostTarget::adcSetInput(&hadc1, 6, raw);
		REQUIRE(HostTarget::adcConversionComplete(&hadc1));
	}
};

/** Calibration that makes the analog reference exactly 3300 mV (VREFINT reading = VREFINT_CAL) */
struct Vref3300Fixture : VoltageFixture {
	Vref3300Fixture() : VoltageFixture(1500, 1500) {}
};

} // namespace

TEST_SUITE("VoltageSense") {

// ---------------------------------------------------------------------------------------------------------------------
// ADC counts to millivolts
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(Vref3300Fixture, "adcValToVoltage scales 12 bit counts to the 3300 mV reference measured with VREFINT") {
	CHECK(adcValToVoltage(4095) == doctest::Approx(3300.0f));
	CHECK(adcValToVoltage(0) == doctest::Approx(0.0f));
	CHECK(adcValToVoltage(1241) == doctest::Approx(1000.0f)); // 1241 * 3300 / 4095 = 1000.07, integer division
}

TEST_CASE_FIXTURE(Vref3300Fixture, "adcValToVoltage truncates to whole millivolts") {
	CHECK(adcValToVoltage(2048) == doctest::Approx(1650.0f)); // 1650.4
	CHECK(adcValToVoltage(1) == doctest::Approx(0.0f));		  // 0.8
}

TEST_CASE_FIXTURE(VoltageFixture, "a lower VREFINT reading means a higher analog reference and higher voltages") {
	// VREFINT_CAL 1365 measured at 3300 mV; read as 1100 the reference is 1365 * 3300 / 1100 = 4095 mV
	CHECK(adcValToVoltage(1000) == doctest::Approx(1000.0f));
	CHECK(adcValToVoltage(4095) == doctest::Approx(4095.0f));
}

TEST_CASE_FIXTURE(VoltageFixture, "the factory calibration value scales the analog reference") {
	HostTarget::setAdcCalibration(1500, 950, 1200);
	// 1500 * 3300 / 1100 = 4500 mV
	CHECK(adcValToVoltage(4095) == doctest::Approx(4500.0f));
	CHECK(adcValToVoltage(910) == doctest::Approx(1000.0f));
}

TEST_CASE_FIXTURE(TargetFixture, "without a VREFINT conversion all voltages are 0 instead of a division by zero") {
	// ADC1_BUF[7] (VREFINT) is 0 until the first ADC1 conversion
	REQUIRE(ADC1_BUF[7] == 0);
	CHECK(adcValToVoltage(4095) == 0.0f);
	startADC();
	HostTarget::adcConvert(&hadc2, {1000, 2000});
	CHECK(getExtV() == 0);
	CHECK(getIntV() == 0);
}

TEST_CASE_FIXTURE(Vref3300Fixture, "getExtV and getIntV read ADC2 rank 0 and 1 times the default multiplier 30.67") {
	setVSenseMult(30.67f); // VOLTAGE_MULT_DEFAULT (the fixture changed it)
	HostTarget::adcConvert(&hadc2, {1241, 2482}); // 1000 mV and 2000 mV at the pins
	CHECK(getExtV() == 30670);
	CHECK(getIntV() == 61340);
}

TEST_CASE_FIXTURE(TargetFixture, "the default multiplier is VOLTAGE_MULT_DEFAULT 30.67") {
	HostTarget::setAdcCalibration(1500, 950, 1200);
	startADC();
	HostTarget::adcConvert(&hadc1, {0, 0, 0, 0, 0, 0, 950, 1500});
	HostTarget::adcConvert(&hadc2, {1241, 0});
	CHECK(getExtV() == 30670);
}

TEST_CASE_FIXTURE(VoltageFixture, "setVSenseMult changes the scaling of both voltages") {
	HostTarget::adcConvert(&hadc2, {1000, 500});
	CHECK(getExtV() == 20000);
	CHECK(getIntV() == 10000);
	setVSenseMult(2.5f);
	CHECK(getExtV() == 2500);
	CHECK(getIntV() == 1250);
}

TEST_CASE_FIXTURE(VoltageFixture, "the scaled voltage is truncated to whole millivolts") {
	setVSenseMult(1.99f);
	HostTarget::adcConvert(&hadc2, {1, 3}); // 1.99 mV and 5.97 mV
	CHECK(getExtV() == 1);
	CHECK(getIntV() == 5);
}

TEST_CASE_FIXTURE(VoltageFixture, "the voltages follow the latest ADC2 conversion") {
	convert(12000, 12500);
	CHECK(getExtV() == 12000);
	CHECK(getIntV() == 12500);
	convert(24000, 24020);
	CHECK(getExtV() == 24000);
	CHECK(getIntV() == 24020);
}

// ---------------------------------------------------------------------------------------------------------------------
// Brake resistor
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(VoltageFixture, "the brake stays off while both voltages are below 6.5 V") {
	convert(0, 6480); // 6.48 V above the supply, but the device is not powered (USB leakage)
	CHECK_FALSE(brakeOn());
}

TEST_CASE_FIXTURE(VoltageFixture, "the brake is checked from 6.5 V internal voltage on") {
	convert(0, 6500);
	CHECK(brakeOn());
}

TEST_CASE_FIXTURE(VoltageFixture, "a supply voltage of 6.5 V enables the brake check") {
	convert(6500, 12000); // VINT 5.5 V above VEXT
	CHECK(brakeOn());
}

TEST_CASE_FIXTURE(VoltageFixture, "the brake switches on when the internal voltage exceeds the supply by more than 5 V") {
	convert(12000, 17000); // exactly 5 V: not more
	CHECK_FALSE(brakeOn());
	convert(12000, 17020);
	CHECK(brakeOn());
}

TEST_CASE_FIXTURE(VoltageFixture, "an active brake stays on until the difference is 4 V or less") {
	convert(12000, 17020);
	REQUIRE(brakeOn());
	convert(12000, 16020); // 4.02 V
	CHECK(brakeOn());
	convert(12000, 16000); // 4 V
	CHECK_FALSE(brakeOn());
}

TEST_CASE_FIXTURE(VoltageFixture, "an inactive brake does not switch on inside the hysteresis band") {
	convert(12000, 16500); // 4.5 V: between the off and the on threshold
	CHECK_FALSE(brakeOn());
}

TEST_CASE_FIXTURE(VoltageFixture, "the brake switches on above the maximum voltage of 65 V without any difference") {
	convert(65000, 65000);
	CHECK_FALSE(brakeOn());
	convert(65020, 65020);
	CHECK(brakeOn());
}

TEST_CASE_FIXTURE(VoltageFixture, "characterization: the over-voltage brake switches off as soon as the internal voltage is 65 V again (NEEDS REVIEW 72)") {
	// NEEDS REVIEW 72: only the difference thresholds have a hysteresis; the maximum voltage has none, so a supply near
	// 65 V can toggle the brake with every ADC2 conversion. Intended?
	convert(65020, 65020);
	REQUIRE(brakeOn());
	convert(65000, 65000);
	CHECK_FALSE(brakeOn());
}

TEST_CASE_FIXTURE(VoltageFixture, "the brake does not react to a supply higher than the internal voltage") {
	convert(40000, 12000);
	CHECK_FALSE(brakeOn());
}

TEST_CASE_FIXTURE(VoltageFixture, "setupBrakePin sets the on and off difference and the maximum voltage") {
	setupBrakePin(2000, 1000, 30000);
	convert(12000, 14000);
	CHECK_FALSE(brakeOn());
	convert(12000, 14020);
	CHECK(brakeOn());
	convert(12000, 13020);
	CHECK(brakeOn());
	convert(12000, 13000);
	CHECK_FALSE(brakeOn());
	convert(30020, 30020);
	CHECK(brakeOn());
}

TEST_CASE_FIXTURE(VoltageFixture, "a maximum voltage of 0 disables the brake resistor control completely") {
	setupBrakePin(5000, 4000, 0);
	convert(12000, 40000);
	CHECK_FALSE(brakeOn());
	convert(70000, 70000);
	CHECK_FALSE(brakeOn());
}

TEST_CASE_FIXTURE(VoltageFixture, "braking for 5 s is accepted, longer switches the brake off permanently") {
	convert(12000, 20000); // on at 1000 ms
	REQUIRE(brakeOn());
	HostPlatform::advanceMs(5000);
	convert(12000, 20000);
	CHECK(brakeOn()); // 5000 ms: not more than 5 s
	HostPlatform::advanceMs(1);
	convert(12000, 20000);
	CHECK_FALSE(brakeOn()); // stuck on: shut down
}

TEST_CASE_FIXTURE(VoltageFixture, "after the brake resistor shut down the brake stays off for any voltage") {
	convert(12000, 20000);
	HostPlatform::advanceMs(5001);
	convert(12000, 20000);
	REQUIRE_FALSE(brakeOn());
	convert(12000, 12000);
	convert(12000, 20000);
	CHECK_FALSE(brakeOn());
	convert(70000, 70000);
	CHECK_FALSE(brakeOn());
}

TEST_CASE_FIXTURE(VoltageFixture, "the brake time restarts when the brake switches off in between") {
	convert(12000, 20000); // on at 1000 ms
	HostPlatform::advanceMs(3000);
	convert(12000, 12000); // off at 4000 ms
	REQUIRE_FALSE(brakeOn());
	HostPlatform::advanceMs(1000);
	convert(12000, 20000); // on again at 5000 ms
	HostPlatform::advanceMs(4000);
	convert(12000, 20000); // 9000 ms: 8 s after the first, 4 s after the second activation
	CHECK(brakeOn());
	HostPlatform::advanceMs(1001);
	convert(12000, 20000);
	CHECK_FALSE(brakeOn());
}

TEST_CASE_FIXTURE(VoltageFixture, "a brake that switched off normally is still available more than 5 s later") {
	convert(12000, 20000); // on at 1000 ms
	HostPlatform::advanceMs(1000);
	convert(12000, 12000); // off at 2000 ms
	HostPlatform::advanceMs(5000);
	convert(12000, 12000); // 6 s after the activation, brake off all the time
	convert(12000, 20000);
	CHECK(brakeOn());
}

TEST_CASE_FIXTURE(VoltageFixture, "characterization: the brake resistor shut down reports no error and triggers no emergency stop (NEEDS REVIEW 73)") {
	// NEEDS REVIEW 73: voltagesense.cpp:24 defines the critical error "Brake resistor stuck on" but line 88 does not add it
	// ("can not create error here in ISR") and nothing adds it later. The over-voltage protection is then disabled
	// until reboot without any indication. Should the error be reported from thread context (and the motor stopped)?
	convert(12000, 20000);
	HostPlatform::advanceMs(5001);
	convert(12000, 20000);
	REQUIRE_FALSE(brakeOn());
	CHECK(ErrorHandler::getErrors().empty());
	CHECK(HostPlatform::leds().errPulses == 0);
}

TEST_CASE_FIXTURE(VoltageFixture, "braking that starts at tick 0 is also shut down after 5 s" *
		  KNOWN_ISSUE("voltagesense.cpp:78/83 uses brakeActiveTime == 0 for 'not braking': a brake activated at HAL_GetTick() == 0 "
					  "(also after the 49.7 day tick wrap) is never shut down")) {
	HostPlatform::setTimeMs(0);
	convert(12000, 20000);
	REQUIRE(brakeOn());
	HostPlatform::advanceMs(6000);
	convert(12000, 20000);
	REQUIRE_ALL_HAL_MODELLED();
	CHECK_FALSE(brakeOn()); // first failing assertion: still braking after 6 s
}

TEST_CASE_FIXTURE(VoltageFixture, "an ADC1 conversion does not run the brake check") {
	HostTarget::adcSetInputs(&hadc2, {600, 1000}); // 12 V / 20 V, not converted yet
	REQUIRE(HostTarget::adcConversionComplete(&hadc1));
	CHECK_FALSE(brakeOn());
	REQUIRE(HostTarget::adcConversionComplete(&hadc2));
	CHECK(brakeOn());
}

// ---------------------------------------------------------------------------------------------------------------------
// Chip temperature
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(Vref3300Fixture, "the chip temperature is interpolated between the calibration points 30 C and 110 C") {
	convertTemp(1200); // TS_CAL2
	CHECK(getChipTemp() == 110);
}

TEST_CASE_FIXTURE(Vref3300Fixture, "the chip temperature at the first calibration point is 30 C") {
	convertTemp(950);
	CHECK(getChipTemp() == 30);
}

TEST_CASE_FIXTURE(Vref3300Fixture, "chip temperatures below the first calibration point are negative offsets from 30 C") {
	convertTemp(700); // (700 - 950) * 80 / 250 = -80
	CHECK(getChipTemp() == -50);
}

TEST_CASE_FIXTURE(VoltageFixture, "the chip temperature reading is scaled to the 3300 mV calibration reference") {
	// Reference 4095 mV: 960 counts are 960 * 4095 / 3300 = 1191 counts at 3300 mV: (1191 - 950) * 80 / 250 + 30 = 107
	convertTemp(960);
	CHECK(getChipTemp() == 107);
}

TEST_CASE_FIXTURE(Vref3300Fixture, "the chip temperature is a moving average over 3 readings") {
	convertTemp(1075); // 70 C
	CHECK(getChipTemp() == 70);
	convertTemp(1200); // 110 C
	CHECK(getChipTemp() == 90); // (70 + 110) / 2
	CHECK(getChipTemp() == 96); // 90 + (110 - 90) / 3
	CHECK(getChipTemp() == 100); // 96 + 14 / 3 = 96 + 4 (integer)
}

TEST_CASE_FIXTURE(Vref3300Fixture, "characterization: the integer moving average of the chip temperature stops up to 2 C short of a constant reading (NEEDS REVIEW 74)") {
	// NEEDS REVIEW 74: FastMovingAverage<int32_t> adds (v - avg) / 3 in integer arithmetic (FastAvg.h:93), so the average
	// never moves while it is within 2 C of the reading: 70 C followed by a constant 72 C reads 70 C forever.
	convertTemp(1075); // 70 C
	CHECK(getChipTemp() == 70);
	getChipTemp();
	getChipTemp();
	convertTemp(1082); // (1082 - 950) * 80 / 250 = 42.2: 72 C
	for (int i = 0; i < 100; i++) {
		getChipTemp();
	}
	CHECK(getChipTemp() == 70);
}

TEST_CASE_FIXTURE(TargetFixture, "the chip temperature is 0 without a temperature or VREFINT reading") {
	HostTarget::setAdcCalibration(1500, 950, 1200);
	startADC();
	CHECK(getChipTemp() == 0); // nothing converted
	HostTarget::adcConvert(&hadc1, {0, 0, 0, 0, 0, 0, 1075, 0});
	CHECK(getChipTemp() == 0); // no VREFINT
	HostTarget::adcConvert(&hadc1, {0, 0, 0, 0, 0, 0, 0, 1500});
	CHECK(getChipTemp() == 0); // no temperature reading
}

TEST_CASE_FIXTURE(TargetFixture, "a missing reading does not enter the chip temperature average") {
	HostTarget::setAdcCalibration(1500, 950, 1200);
	startADC();
	HostTarget::adcConvert(&hadc1, {0, 0, 0, 0, 0, 0, 0, 1500});
	CHECK(getChipTemp() == 0);
	HostTarget::adcConvert(&hadc1, {0, 0, 0, 0, 0, 0, 1200, 1500});
	CHECK(getChipTemp() == 110); // first value of the average, not (0 + 110) / 2
}

} // TEST_SUITE
