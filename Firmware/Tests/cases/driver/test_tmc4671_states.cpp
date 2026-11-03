/*
 * TMC4671 driver, group G6: state machine, power, errors (thread Run(), emergency stop, status flags).
 * Requirements R70-R82 of Firmware/Tests/driver/TMC4671_REQUIREMENTS.md, lead decisions L1, L9, L10, L11, L12, L18.
 *
 * Method: the driver thread "TMC" is run in simulated time (TmcFixture::runThreadFor / runUntilState / reachShutdown /
 * reachRunning). Observables: getState(), motorReady(), the register file of FakeTmc4671, the enable pin PE7
 * (DRV_ENABLE), the stored errors. The plants (supply voltage, thermistor ADC word, ENABLE_IN input, status flags) are
 * static values set in the fake; fidelity of every plant is Low (the real chip would change them with the load), only
 * the driver reaction is under test.
 *
 * Timing: the loop in TMC4671::Run() ends every iteration with Delay(10) in addition to the delay of the state case
 * (uninitialized retry 250 + 10 ms, waitPower 250/100 + 10 ms, Running 200 + 10 ms, Shutdown 100 + 10 ms). All timing
 * assertions are ranges derived from these code paths (characterization), not exact values: the activation order of the
 * fake kernel and the SPI frame time of the fake (10 us per frame, TmcFixture) add small amounts.
 *
 * Host vs target: no assertion depends on type widths. The thermistor oracle is computed in double, the firmware
 * (getTemp) computes in float with log(): the margins used (+-10 C) are far above the float error. State enum values are
 * compared as enum values.
 *
 * Datasheet sources: DS = TMC4671-LA Rev 2.07. DS 0x63 MODE_MOTION [7:0] (0 = stopped), DS 0x1A PWM_CHOP [7:0] (0 = off,
 * 7 = centered FOC), DS 0x76 bit 15 ENABLE_IN_RAW, DS 0x7C STATUS_FLAGS (bit 28 enc_n, 30 aenc_n), DS 0x7D STATUS_MASK,
 * DS 0x02/0x03 ADC_RAW (word 0: I1 [31:16], I0 [15:0]; word 2: AGPI_B in [15:0]).
 */
#include "doctest.h"
#include "TmcFixture.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace {

/** Read-only view of the protected error storage (pattern of cases/test_error_handler.cpp). Never writes. */
class ErrorProbe : public ErrorHandler {
public:
	static bool stored(ErrorCode code) {
		for (const Error &e : errors) {
			if (e.isError() && e.code == code) {
				return true;
			}
		}
		return false;
	}
	static ErrorType typeOf(ErrorCode code) {
		for (const Error &e : errors) {
			if (e.isError() && e.code == code) {
				return e.type;
			}
		}
		return ErrorType::none;
	}
};

/** Level the firmware drives on the gate driver enable pin PE7 (DRV_ENABLE): the output data register of the fake port
 * (the fake HAL_GPIO_ReadPin reads the input register, which an output write does not change) */
bool enablePinHigh() {
	return (DRV_ENABLE_GPIO_Port->ODR & DRV_ENABLE_Pin) != 0;
}

uint32_t chopOf(const FakeTmc4671 &c) { return c.reg(0x1A) & 0xFF; }	  // DS 0x1A PWM_CHOP [7:0]
uint32_t motionOf(const FakeTmc4671 &c) { return c.reg(0x63) & 0xFF; } // DS 0x63 MODE_MOTION [7:0]

/** True if no write to PWM_CHOP (0x1A) in the log set a non-zero chopper mode, i.e. the PWM was never switched on */
bool pwmNeverEnabled(const FakeTmc4671 &c) {
	for (uint32_t v : c.writesTo(0x1A)) {
		if ((v & 0xFF) != 0) {
			return false;
		}
	}
	return true;
}

/** Number of states as an integer for messages */
uint32_t st(TMC_ControlState s) { return static_cast<uint32_t>(s); }

/**
 * Raw AGPI_B word that getTemp() converts to `celsius` (HD, inverse of TMC4671::getTemp):
 *   getTemp:  adcval = raw - 0x7FFF - adcOffset;  r = R2 * 43252 / adcval;  1/T = 1/298.15 + ln(r / R) / Beta;  T [C] = T - 273.15
 *   inverse:  r = R * exp(Beta * (1/T - 1/298.15));  adcval = R2 * 43252 / r;  raw = adcval + 0x7FFF + adcOffset
 */
uint32_t rawForTemperature(const TMC4671HardwareTypeConf &hw, double celsius) {
	const auto &t = hw.thermistorSettings;
	const double kelvin = celsius + 273.15;
	const double r = t.thermistor_R * std::exp(t.thermistor_Beta * (1.0 / kelvin - 1.0 / 298.15));
	const double adcval = t.thermistor_R2 * 43252.0 / r;
	const double raw = adcval + 0x7FFF + hw.adcOffset;
	const bool inRange = raw > 0x7FFF + hw.adcOffset + 1 && raw < 65535.0;
	REQUIRE_MESSAGE(inRange, "thermistor oracle out of the 16 bit range, choose other R2");
	return static_cast<uint32_t>(std::lround(raw));
}

/** Thermistor settings of the tests (own values so the raw word fits 16 bit; board values are not the subject) */
void useTestThermistor(TMC4671 &tmc) {
	auto &t = tmc.conf.hwconf.thermistorSettings;
	t.thermistor_R = 22000;
	t.thermistor_R2 = 300; // adcval for 100 C is about 10.7k counts: fits
	t.thermistor_Beta = 4300;
	t.temperatureEnabled = true;
}

} // namespace

TEST_SUITE("TMC4671 states") {

// ---------------------------------------------------------------------------------------------------------------------
// R70 start-up path
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R70] characterization: start-up passes uninitialized, waitPower, Shutdown in this order and needs about one second of power") {
	auto tmc = makeTmc1();
	chip1.setPresent(true);
	chip1.setAdcRaw(0, 0x7FFF7FFF);
	chip1.setMicrosPerTransfer(10); // busy loops on HAL_GetTick (calibration) need time per frame
	tmc->setupDriver();
	powerOff(); // after setupDriver: setHwType changes vSenseMult. Power off keeps the static check counter of waitPower at 0

	std::vector<TMC_ControlState> seq{tmc->getState()};
	for (int i = 0; i < 100; i++) { // 1 s without power
		runThreadFor(*tmc, 10);
		if (tmc->getState() != seq.back()) {
			seq.push_back(tmc->getState());
		}
	}
	// uninitialized: ping ok -> initialize() -> waitPower (no power: stays)
	REQUIRE(seq.size() == 2);
	CHECK(seq[0] == TMC_ControlState::uninitialized);
	CHECK(seq[1] == TMC_ControlState::waitPower);

	powerOn();
	const uint64_t t0 = HostPlatform::nowUs();
	for (int i = 0; i < 400 && tmc->getState() != TMC_ControlState::Shutdown; i++) { // L11: bounded, 4 s
		runThreadFor(*tmc, 10);
		if (tmc->getState() != seq.back()) {
			seq.push_back(tmc->getState());
		}
	}
	const uint64_t elapsedMs = (HostPlatform::nowUs() - t0) / 1000;
	REQUIRE(tmc->getState() == TMC_ControlState::Shutdown);
	REQUIRE(seq.size() == 3);
	CHECK(seq[2] == TMC_ControlState::Shutdown);
	// Code path: the check counter must exceed 5, so 6 waitPower iterations of 100 + 10 ms (the first iteration can be up
	// to 260 ms late: a no-power iteration is 250 + 10 ms) = 5 * 110 ms before the 6th; initializeWithPower() then
	// calibrates (Delay(100) + 300 ms measuring) and the state changes one 110 ms iteration later.
	CHECK(elapsedMs >= 5 * 110 + 400);
	CHECK(elapsedMs <= 260 + 5 * 110 + 400 + 110 + 300);
}

TEST_CASE_FIXTURE(TmcFixture, "[R70] characterization: motorReady stays false in Shutdown and becomes true only after startMotor") {
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	CHECK_FALSE(tmc->motorReady());
	runThreadFor(*tmc, 500);
	CHECK(tmc->getState() == TMC_ControlState::Shutdown);
	CHECK_FALSE(tmc->motorReady());
	tmc->startMotor();
	REQUIRE(runUntilState(*tmc, TMC_ControlState::Running, 1000));
	CHECK(tmc->motorReady());
}

TEST_CASE_FIXTURE(TmcFixture, "[R70] characterization: an absent chip keeps the driver uninitialized and it leaves uninitialized once the chip answers") {
	auto tmc = makeTmc1();
	chip1.setPresent(false);
	chip1.setAdcRaw(0, 0x7FFF7FFF);
	chip1.setMicrosPerTransfer(10);
	powerOn();
	tmc->setupDriver();
	powerOn();
	runThreadFor(*tmc, 1000);
	CHECK(tmc->getState() == TMC_ControlState::uninitialized);
	chip1.setPresent(true);
	// retry period 250 + 10 ms (uninitialized case + loop delay), then waitPower
	CHECK(runUntilState(*tmc, TMC_ControlState::waitPower, 400));
}

// ---------------------------------------------------------------------------------------------------------------------
// R71 no power
// ---------------------------------------------------------------------------------------------------------------------

namespace {
/** Driver started with the given supply voltages: for 3 s simulated the state must stay waitPower, PE7 low, PWM never on */
void runWithoutPower(TmcFixture &f, int32_t extMv, int32_t intMv) {
	auto tmc = f.makeTmc1();
	f.chip1.setPresent(true);
	f.chip1.setAdcRaw(0, 0x7FFF7FFF);
	f.chip1.setMicrosPerTransfer(10);
	tmc->setupDriver();
	f.setSupply(extMv, intMv);
	REQUIRE_FALSE(tmc->hasPower());
	bool pinEverHigh = false;
	bool stateWrong = false;
	for (int i = 0; i < 300; i++) {
		f.runThreadFor(*tmc, 10);
		pinEverHigh = pinEverHigh || enablePinHigh();
		stateWrong = stateWrong || (i > 60 && tmc->getState() != TMC_ControlState::waitPower);
	}
	CHECK_FALSE(stateWrong);
	CHECK(tmc->getState() == TMC_ControlState::waitPower);
	CHECK_FALSE(pinEverHigh);
	CHECK(pwmNeverEnabled(f.chip1));
	CHECK_FALSE(tmc->motorReady());
}
} // namespace

TEST_CASE_FIXTURE(TmcFixture, "[R71] with 0 V on both rails the driver stays in waitPower and never enables PE7 or the PWM") {
	runWithoutPower(*this, 0, 0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R71] with the external supply below 10 V the driver stays in waitPower and never enables PE7 or the PWM") {
	runWithoutPower(*this, 9000, 24000);
}

TEST_CASE_FIXTURE(TmcFixture, "[R71] with the internal supply below 10 V the driver stays in waitPower and never enables PE7 or the PWM") {
	runWithoutPower(*this, 24000, 9000);
}

TEST_CASE_FIXTURE(TmcFixture, "[R71] with the internal supply above 78 V the driver stays in waitPower and never enables PE7 or the PWM") {
	runWithoutPower(*this, 24000, 79000);
}

TEST_CASE_FIXTURE(TmcFixture, "[R71] characterization: hasPower needs both supplies above 10 V and the internal one below 78 V") {
	auto tmc = makeTmc1();
	struct Case {
		int32_t ext, in;
		bool power;
	};
	// Thresholds of the firmware (board facts, L1). The test values keep 1 V distance to the thresholds: one ADC count
	// is a few tens of mV (vSenseMult), so the exact boundary is not asserted.
	const Case cases[] = {{24000, 24000, true}, {11000, 11000, true}, {24000, 77000, true}, {9000, 24000, false},
						  {24000, 9000, false}, {24000, 79000, false}, {0, 0, false}};
	for (const Case &c : cases) {
		setSupply(c.ext, c.in);
		INFO("ext " << c.ext << " mV, int " << c.in << " mV");
		CHECK(tmc->hasPower() == c.power);
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// R72 start / stop motor
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R72] startMotor from Shutdown leads to Running with the centered FOC PWM and the enable pin set") {
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	tmc->startMotor();
	REQUIRE(runUntilState(*tmc, TMC_ControlState::Running, 1000));
	CHECK(chopOf(chip1) == 7); // DS 0x1A: 7 = centered PWM for FOC
	CHECK(enablePinHigh());
	CHECK(tmc->motorReady());
}

TEST_CASE_FIXTURE(TmcFixture, "[R72] stopMotor from Running stops the motion mode, switches the PWM off and leads to Shutdown") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	REQUIRE(motionOf(chip1) != 0); // stale value the stop must replace
	tmc->stopMotor();
	runThreadFor(*tmc, 300);
	CHECK(motionOf(chip1) == 0); // DS 0x63 [7:0] = 0 stopped
	CHECK(chopOf(chip1) == 0);	 // DS 0x1A [7:0] = 0 off
	CHECK(tmc->getState() == TMC_ControlState::Shutdown);
	CHECK_FALSE(tmc->motorReady());
}

// ---------------------------------------------------------------------------------------------------------------------
// R73 emergency stop
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R73] emergencyStop(false) pulls the enable pin low and stops the motion at once") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	REQUIRE(enablePinHigh());
	tmc->emergencyStop(false);
	CHECK_FALSE(enablePinHigh());
	CHECK(motionOf(chip1) == 0); // DS 0x63 [7:0] = 0
	CHECK(chopOf(chip1) == 0);	 // DS 0x1A [7:0] = 0
}

TEST_CASE_FIXTURE(TmcFixture, "[R73] after emergencyStop(false) the driver ends in Shutdown and the motor is not ready") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	tmc->emergencyStop(false);
	runThreadFor(*tmc, 500);
	CHECK(tmc->getState() == TMC_ControlState::Shutdown);
	CHECK_FALSE(tmc->motorReady());
	CHECK_FALSE(enablePinHigh());
}

TEST_CASE_FIXTURE(TmcFixture, "[R73] emergencyStop(true) after an emergency stop restores the enable pin, the PWM and the Running state") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	tmc->emergencyStop(false);
	runThreadFor(*tmc, 500);
	REQUIRE(tmc->getState() == TMC_ControlState::Shutdown);
	tmc->emergencyStop(true);
	CHECK(runUntilState(*tmc, TMC_ControlState::Running, 2000));
	CHECK(enablePinHigh());
	CHECK(chopOf(chip1) == 7); // DS 0x1A centered FOC
	CHECK(tmc->motorReady());
}

// ---------------------------------------------------------------------------------------------------------------------
// R74 hardware enable input
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R74] with ENABLE_IN high the driver stays in Running over several periodic checks") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	chip1.setEnableIn(true); // DS 0x76 bit 15
	runThreadFor(*tmc, 6000);
	CHECK(tmc->getState() == TMC_ControlState::Running);
	CHECK(enablePinHigh());
	CHECK_FALSE(ErrorProbe::stored(ErrorCode::emergencyStop));
}

TEST_CASE_FIXTURE(TmcFixture, "[R74] ENABLE_IN low in Running triggers the emergency stop with the critical emergencyStop error within one check period") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	chip1.setEnableIn(false); // DS 0x76 bit 15 low
	// Check period: tick - lastStatTime > 2000 ms, evaluated once per Running iteration of 200 + 10 ms: at most about 2.2 s
	// later (characterization of the period); 2.6 s of margin.
	runThreadFor(*tmc, 2600);
	CHECK(ErrorProbe::stored(ErrorCode::emergencyStop));
	CHECK(ErrorProbe::typeOf(ErrorCode::emergencyStop) == ErrorType::critical);
	CHECK_FALSE(enablePinHigh());
	CHECK(motionOf(chip1) == 0);
	CHECK(tmc->getState() == TMC_ControlState::Shutdown);
	CHECK_FALSE(tmc->motorReady());
}

TEST_CASE_FIXTURE(TmcFixture, "[R74] after ENABLE_IN is high again the emergencyStop error is cleared and the driver re-initialises" * KNOWN_ISSUE("B-T25: emergencyStop(true) resumes the suspended thread and startMotor() requests Running, so the Shutdown case with the estopTriggered recovery (clearError, re-init) never runs; the error stays stored while the driver is Running")) {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	chip1.setEnableIn(false);
	runThreadFor(*tmc, 2600);
	REQUIRE(ErrorProbe::stored(ErrorCode::emergencyStop));
	chip1.setEnableIn(true);
	// The driver thread parks itself after an emergency stop (Suspend in Run()); the supported way to resume is
	// emergencyStop(true) as the axis/estop logic does. Intended: the Shutdown case then clears the error and re-initialises.
	tmc->emergencyStop(true);
	runThreadFor(*tmc, 3000);
	CHECK_FALSE(ErrorProbe::stored(ErrorCode::emergencyStop));
}

// ---------------------------------------------------------------------------------------------------------------------
// R75 undervoltage while running
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R75] characterization: losing the supply in Running stops the motion, raises the undervoltage warning and goes to waitPower") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	powerOff();
	runThreadFor(*tmc, 600);
	CHECK(motionOf(chip1) == 0); // DS 0x63 [7:0] = 0
	CHECK(ErrorProbe::stored(ErrorCode::undervoltage));
	CHECK(ErrorProbe::typeOf(ErrorCode::undervoltage) == ErrorType::warning);
	CHECK(tmc->getState() == TMC_ControlState::waitPower);
	CHECK_FALSE(tmc->motorReady());
}

TEST_CASE_FIXTURE(TmcFixture, "[R75] characterization: when the supply returns the driver is in Running again") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	powerOff();
	runThreadFor(*tmc, 600);
	REQUIRE(tmc->getState() == TMC_ControlState::waitPower);
	powerOn();
	// waitPower with power: 100 + 10 ms per iteration, up to 250 + 10 ms until power is noticed
	CHECK(runUntilState(*tmc, TMC_ControlState::Running, 1000));
}

TEST_CASE_FIXTURE(TmcFixture, "[R75] the undervoltage warning is cleared when the supply returns" * KNOWN_ISSUE("B-T14: ErrorHandler::clearError(undervoltage) is only called in initializeWithPower(), which returns early when power was initialised before")) {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	powerOff();
	runThreadFor(*tmc, 600);
	REQUIRE(ErrorProbe::stored(ErrorCode::undervoltage));
	powerOn();
	runThreadFor(*tmc, 1500);
	CHECK_FALSE(ErrorProbe::stored(ErrorCode::undervoltage));
}

TEST_CASE_FIXTURE(TmcFixture, "[R75] the motion mode is restored to torque when the supply returns" * KNOWN_ISSUE("B-T14: setMotionMode(lastMotionMode) is only called in initializeWithPower(), which returns early when power was initialised before; the driver is Running with motion mode stop")) {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	REQUIRE(motionOf(chip1) == 1); // DS 0x63 [7:0] = 1 torque
	powerOff();
	runThreadFor(*tmc, 600);
	REQUIRE(motionOf(chip1) == 0);
	powerOn();
	REQUIRE(runUntilState(*tmc, TMC_ControlState::Running, 1000));
	CHECK(motionOf(chip1) == 1); // DS 0x63 [7:0] = 1 torque
}

// ---------------------------------------------------------------------------------------------------------------------
// R76 over-temperature
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R76] a temperature above the limit in Running ends in HardError") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	useTestThermistor(*tmc);
	const double limit = tmc->conf.hwconf.thermistorSettings.temp_limit;
	chip1.setAdcRaw(2, rawForTemperature(tmc->conf.hwconf, limit + 10)); // DS 0x02 word 2: AGPI_B in [15:0]
	runThreadFor(*tmc, 3000); // periodic check every > 2 s
	CHECK(tmc->getState() == TMC_ControlState::HardError);
	CHECK_FALSE(tmc->motorReady());
}

TEST_CASE_FIXTURE(TmcFixture, "[R76] a temperature below the limit keeps the driver in Running") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	useTestThermistor(*tmc);
	const double limit = tmc->conf.hwconf.thermistorSettings.temp_limit;
	chip1.setAdcRaw(2, rawForTemperature(tmc->conf.hwconf, limit - 30));
	runThreadFor(*tmc, 6000);
	CHECK(tmc->getState() == TMC_ControlState::Running);
	CHECK(tmc->motorReady());
}

TEST_CASE_FIXTURE(TmcFixture, "[R76] an over-temperature in Running switches the PWM off and stops the motion" * KNOWN_ISSUE("B-T26: OverTemp case calls stopMotor(), which only acts in the states Running / EncoderFinished, so PWM chopper (0x1A) and motion mode (0x63) stay on in HardError")) {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	useTestThermistor(*tmc);
	const double limit = tmc->conf.hwconf.thermistorSettings.temp_limit;
	chip1.setAdcRaw(2, rawForTemperature(tmc->conf.hwconf, limit + 10));
	runThreadFor(*tmc, 3000);
	REQUIRE(tmc->getState() == TMC_ControlState::HardError);
	CHECK(chopOf(chip1) == 0); // DS 0x1A [7:0] = 0 off
	CHECK(motionOf(chip1) == 0);
}

// ---------------------------------------------------------------------------------------------------------------------
// R77 ADC calibration failure in initialize()
// ---------------------------------------------------------------------------------------------------------------------

namespace {
/** No supply and an ADC whose mean is 7000 counts off the centre 0x7FFF (> 5000: calibration fails cleanly, no division by zero) */
void startWithBrokenAdc(TmcFixture &f, TMC4671 &tmc) {
	f.chip1.setPresent(true);
	f.chip1.setAdcRaw(0, (0x7FFF + 7000) | ((0x7FFF + 7000) << 16));
	f.chip1.setMicrosPerTransfer(10);
	tmc.setupDriver();
	f.powerOff();
}
} // namespace

TEST_CASE_FIXTURE(TmcFixture, "[R77] a failed ADC calibration in initialize() ends in HardError" * KNOWN_ISSUE("B-T15: initialize() requests HardError with a non-forced changeState, the thread then forces waitPower and uninitialized again: endless re-init loop")) {
	auto tmc = makeTmc1();
	startWithBrokenAdc(*this, *tmc);
	runThreadFor(*tmc, 5000);
	CHECK(tmc->getState() == TMC_ControlState::HardError);
}

TEST_CASE_FIXTURE(TmcFixture, "[R77] a failed ADC calibration in initialize() keeps the enable pin low and the PWM off") {
	auto tmc = makeTmc1();
	startWithBrokenAdc(*this, *tmc);
	bool pinEverHigh = false;
	for (int i = 0; i < 300; i++) {
		runThreadFor(*tmc, 10);
		pinEverHigh = pinEverHigh || enablePinHigh();
	}
	CHECK_FALSE(pinEverHigh);
	CHECK(pwmNeverEnabled(chip1));
	CHECK(ErrorProbe::stored(ErrorCode::adcCalibrationError));
}

// ---------------------------------------------------------------------------------------------------------------------
// R78 calibrateAdcOffset
// ---------------------------------------------------------------------------------------------------------------------

namespace {
struct CalibFixture : TmcFixture {
	std::unique_ptr<TMC_1> tmc;
	CalibFixture() {
		tmc = makeTmc1();
		chip1.setPresent(true);
		chip1.setMicrosPerTransfer(10);
	}
	static uint32_t word(uint16_t i1, uint16_t i0) { return (uint32_t(i1) << 16) | i0; } // DS 0x02 word 0
};
} // namespace

TEST_CASE_FIXTURE(CalibFixture, "[R78] calibrateAdcOffset writes the mean raw values as offsets: I0 from [15:0] to 0x09, I1 from [31:16] to 0x08") {
	chip1.setAdcRaw(0, word(0x7FFF - 200, 0x7FFF + 100)); // I1 = 32567, I0 = 32867, constant
	chip1.setReg(0x09, 0xABCD0000); // stale scale / offset: scale half must be kept (R23 layout)
	chip1.setReg(0x08, 0x12340000);
	REQUIRE(tmc->calibrateAdcOffset(150));
	// mean of constant samples = the sample (HD)
	CHECK((chip1.reg(0x09) & 0xFFFF) == 0x7FFF + 100);
	CHECK((chip1.reg(0x08) & 0xFFFF) == 0x7FFF - 200);
	CHECK((chip1.reg(0x09) >> 16) == 0xABCD);
	CHECK((chip1.reg(0x08) >> 16) == 0x1234);
}

TEST_CASE_FIXTURE(CalibFixture, "[R78] calibrateAdcOffset averages alternating samples") {
	// Samples alternate between mean-50 and mean+50 on I0 and mean+30 / mean-30 on I1. Over an even number of samples the
	// mean is exactly the mean value; with an odd number it is off by at most 1 (integer division): tolerance 1 (HD).
	const uint16_t m0 = 0x7FFF + 400, m1 = 0x7FFF - 300;
	auto toggle = std::make_shared<bool>(false);
	chip1.onRead(0x02, [toggle, m0, m1](FakeTmc4671 &c) {
		*toggle = !*toggle;
		const int s = *toggle ? 1 : -1;
		c.setAdcRaw(0, CalibFixture::word(static_cast<uint16_t>(m1 - 30 * s), static_cast<uint16_t>(m0 + 50 * s)));
	});
	REQUIRE(tmc->calibrateAdcOffset(150));
	CHECK(std::abs(int(chip1.reg(0x09) & 0xFFFF) - int(m0)) <= 1);
	CHECK(std::abs(int(chip1.reg(0x08) & 0xFFFF) - int(m1)) <= 1);
}

TEST_CASE_FIXTURE(CalibFixture, "[R78] characterization: calibrateAdcOffset succeeds when both means are exactly 5000 counts from 0x7FFF") {
	chip1.setAdcRaw(0, word(0x7FFF - 5000, 0x7FFF + 5000)); // lastraw start 33415 (default offset): within the 10000 sample filter
	CHECK(tmc->calibrateAdcOffset(150));
	CHECK_FALSE(ErrorProbe::stored(ErrorCode::adcCalibrationError));
}

TEST_CASE_FIXTURE(CalibFixture, "[R78] characterization: calibrateAdcOffset fails with the critical adcCalibrationError when the I0 mean is 5001 counts from 0x7FFF") {
	chip1.setAdcRaw(0, word(0x7FFF, 0x7FFF + 5001));
	chip1.setReg(0x09, 0xABCD1234);
	CHECK_FALSE(tmc->calibrateAdcOffset(150));
	CHECK(ErrorProbe::stored(ErrorCode::adcCalibrationError));
	CHECK(ErrorProbe::typeOf(ErrorCode::adcCalibrationError) == ErrorType::critical);
	CHECK(chip1.reg(0x09) == 0xABCD1234); // offsets are not written on failure
}

TEST_CASE_FIXTURE(CalibFixture, "[R78] characterization: calibrateAdcOffset fails when the I1 mean is 5001 counts below 0x7FFF") {
	chip1.setAdcRaw(0, word(0x7FFF - 5001, 0x7FFF));
	CHECK_FALSE(tmc->calibrateAdcOffset(150));
	CHECK(ErrorProbe::stored(ErrorCode::adcCalibrationError));
}

TEST_CASE_FIXTURE(CalibFixture, "[R78] a chip returning only zero raw words fails the calibration cleanly" * doctest::skip() *
		doctest::description("KNOWN ISSUE (crash): B-T16 calibrateAdcOffset divides by measurements_idle == 0 when the signflip filter (10000 counts from the previous sample, first reference = default offset 33415) rejects every sample: SIGFPE on host (L18). Skipped because it terminates the test run. Intended: false, adcCalibrationError, no crash")) {
	chip1.setAdcRaw(0, 0); // DS 0x02 default is 0
	CHECK_FALSE(tmc->calibrateAdcOffset(150));
	CHECK(ErrorProbe::stored(ErrorCode::adcCalibrationError));
}

// ---------------------------------------------------------------------------------------------------------------------
// R79 status flags
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R79] readFlags(false) returns the unmasked content of STATUS_FLAGS 0x7C") {
	auto tmc = makeTmc1();
	chip1.setReg(0x7D, 1u << 19);
	chip1.raiseFlags((1u << 19) | (1u << 26));
	CHECK(tmc->readFlags(false).asInt == ((1u << 19) | (1u << 26))); // DS 0x7C bit 19 not_PLL_locked, 26 adc_i_clipped
}

TEST_CASE_FIXTURE(TmcFixture, "[R79] characterization: readFlags(true) masks the flags with the mask the driver last wrote to 0x7D") {
	auto tmc = makeTmc1();
	tmc->statusMask.asInt = 1u << 19; // member used by readFlags; the register write below keeps the chip in step
	tmc->setStatusMask(tmc->statusMask); // writes DS 0x7D
	chip1.raiseFlags((1u << 19) | (1u << 26));
	CHECK(tmc->readFlags(true).asInt == (1u << 19));
}

TEST_CASE_FIXTURE(TmcFixture, "[R79] the FLAG pin interrupt clears STATUS_FLAGS 0x7C") {
	auto tmc = makeTmc1();
	tmc->setStatusMask((1u << 19) | (1u << 26));
	chip1.raiseFlags(1u << 26);
	REQUIRE(chip1.reg(0x7C) != 0);
	tmc->exti(FLAG_Pin);
	CHECK(chip1.reg(0x7C) == 0); // DS 0x7C: write 0 resets the flags
}

TEST_CASE_FIXTURE(TmcFixture, "[R79] an interrupt on another pin does not touch the flags") {
	auto tmc = makeTmc1();
	tmc->setStatusMask(1u << 26);
	chip1.raiseFlags(1u << 26);
	tmc->exti(GPIO_PIN_0 == FLAG_Pin ? GPIO_PIN_1 : GPIO_PIN_0);
	CHECK(chip1.reg(0x7C) == (1u << 26));
}

TEST_CASE_FIXTURE(TmcFixture, "[R79] characterization: an enc_n flag with an ABN encoder records the index hit and clears enc_n in the mask") {
	auto tmc = makeTmc1();
	tmc->conf.motconf.enctype = EncoderType_TMC::abn;
	tmc->abnconf.useIndex = true;
	tmc->setEncoderIndexFlagEnabled(true, false); // enables the enc_n bit 28 in 0x7D (statusMask.ENC_N)
	REQUIRE((chip1.reg(0x7D) & (1u << 28)) != 0);
	REQUIRE(tmc->getEncoderType() == EncoderType::incremental);
	chip1.raiseFlags(1u << 28); // DS 0x7C bit 28 enc_n
	tmc->exti(FLAG_Pin);
	CHECK(tmc->getEncoderType() == EncoderType::incrementalIndex);
	CHECK((chip1.reg(0x7D) & (1u << 28)) == 0);
	CHECK(chip1.reg(0x7C) == 0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R79] characterization: an aenc_n flag with a sincos encoder clears aenc_n in the mask") {
	auto tmc = makeTmc1();
	tmc->conf.motconf.enctype = EncoderType_TMC::sincos;
	tmc->setEncoderIndexFlagEnabled(true, false);
	REQUIRE((chip1.reg(0x7D) & (1u << 30)) != 0); // DS 0x7C bit 30 aenc_n
	chip1.raiseFlags(1u << 30);
	tmc->exti(FLAG_Pin);
	CHECK((chip1.reg(0x7D) & (1u << 30)) == 0);
	CHECK(chip1.reg(0x7C) == 0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R79] characterization: an enc_n flag without an ABN encoder does not record an index hit") {
	auto tmc = makeTmc1();
	tmc->conf.motconf.enctype = EncoderType_TMC::hall;
	tmc->abnconf.useIndex = true;
	tmc->setStatusMask(1u << 28);
	chip1.raiseFlags(1u << 28);
	tmc->exti(FLAG_Pin);
	CHECK(tmc->getEncoderType() == EncoderType::incremental);
	CHECK((chip1.reg(0x7D) & (1u << 28)) != 0); // mask untouched
}

// ---------------------------------------------------------------------------------------------------------------------
// R80 brake resistor failure
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R80] a brakeResistorFailure error in Running emergency-stops the driver and enters HardError") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	ErrorHandler::addError(Error(ErrorCode::brakeResistorFailure, ErrorType::critical, "brake resistor"));
	CHECK_FALSE(enablePinHigh());
	CHECK(motionOf(chip1) == 0); // DS 0x63 [7:0] = 0
	CHECK(chopOf(chip1) == 0);	 // DS 0x1A [7:0] = 0
	CHECK(tmc->getState() == TMC_ControlState::HardError);
	runThreadFor(*tmc, 1000);
	CHECK(tmc->getState() == TMC_ControlState::HardError); // blocks
	CHECK_FALSE(tmc->motorReady());
}

TEST_CASE_FIXTURE(TmcFixture, "[R80] another error does not stop the driver") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	ErrorHandler::addError(Error(ErrorCode::overtemp, ErrorType::warning, "other"));
	runThreadFor(*tmc, 500);
	CHECK(tmc->getState() == TMC_ControlState::Running);
	CHECK(enablePinHigh());
}

// ---------------------------------------------------------------------------------------------------------------------
// R81 two instances
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R81] TMC_1 talks only to CS PA4 and TMC_2 only to CS PB0") {
	auto tmc1 = makeTmc1();
	auto tmc2 = makeTmc2();
	chip1.setPresent(true);
	chip2.setPresent(true);
	chip1.clearLog();
	chip2.clearLog();
	CHECK(tmc1->pingDriver());
	CHECK(chip1.log().size() == 2); // write CHIPINFO_ADDR + read CHIPINFO_DATA
	CHECK(chip2.log().empty());
	chip1.clearLog();
	CHECK(tmc2->pingDriver());
	CHECK(chip2.log().size() == 2);
	CHECK(chip1.log().empty());
}

TEST_CASE_FIXTURE(TmcFixture, "[R81] characterization: the flash addresses of TMC_2 are those of TMC_1 plus 0x40") {
	auto tmc1 = makeTmc1();
	auto tmc2 = makeTmc2();
	const TMC4671FlashAddrs &a = tmc1->flashAddrs;
	const TMC4671FlashAddrs &b = tmc2->flashAddrs;
	CHECK(b.mconf - a.mconf == 0x40);
	CHECK(b.cpr - a.cpr == 0x40);
	CHECK(b.encA - a.encA == 0x40);
	CHECK(b.offsetFlux - a.offsetFlux == 0x40);
	CHECK(b.torque_p - a.torque_p == 0x40);
	CHECK(b.torque_i - a.torque_i == 0x40);
	CHECK(b.flux_p - a.flux_p == 0x40);
	CHECK(b.flux_i - a.flux_i == 0x40);
	CHECK(b.ADC_i0_ofs - a.ADC_i0_ofs == 0x40);
	CHECK(b.ADC_i1_ofs - a.ADC_i1_ofs == 0x40);
	CHECK(b.encOffset - a.encOffset == 0x40);
	CHECK(b.phieOffset - a.phieOffset == 0x40);
	CHECK(b.torqueFilter - a.torqueFilter == 0x40);
}

// ---------------------------------------------------------------------------------------------------------------------
// R82 motorReady
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R82] motorReady is false before the thread ran and true in Running") {
	auto tmc = makeTmc1();
	CHECK_FALSE(tmc->motorReady());
	reachRunning(*tmc);
	CHECK(tmc->motorReady());
}

TEST_CASE_FIXTURE(TmcFixture, "[R82] motorReady is false in Shutdown although power, ADC calibration and encoder are in order") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	tmc->stopMotor();
	runThreadFor(*tmc, 300);
	REQUIRE(tmc->getState() == TMC_ControlState::Shutdown);
	CHECK_FALSE(tmc->motorReady());
}

TEST_CASE_FIXTURE(TmcFixture, "[R82] motorReady is false in waitPower after the supply was lost") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	powerOff();
	runThreadFor(*tmc, 600);
	REQUIRE(tmc->getState() == TMC_ControlState::waitPower);
	CHECK_FALSE(tmc->motorReady());
}

TEST_CASE_FIXTURE(TmcFixture, "[R82] motorReady is false in HardError") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	ErrorHandler::addError(Error(ErrorCode::brakeResistorFailure, ErrorType::critical, "brake resistor"));
	REQUIRE(tmc->getState() == TMC_ControlState::HardError);
	CHECK_FALSE(tmc->motorReady());
}

} // TEST_SUITE
