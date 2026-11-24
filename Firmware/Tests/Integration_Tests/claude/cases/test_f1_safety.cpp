/*
 * F1 safety stops (WO I2-3): F1-T3 E-stop input, F1-T4 TMC ENABLE_IN, F1-T5 undervoltage and over-temperature, while a constant force
 * runs through the whole chain (HID PID report -> EffectsCalculator -> Axis -> TMC4671 driver -> FakeTmc4671).
 *
 * The position is scripted through 0x6B (PositionScript, constant 0, no plant); the torque target is read from register 0x64 of chip1
 * (FakeTmc4671 log / reg), the drive state from 0x1A and 0x63 and the driver enable pin (HostTarget::outputPin of DRV_ENABLE).
 * Settings are made over CDC (the user's path), power and fxratio first (IB 7 / bug 23).
 *
 * Oracles
 *  - TMC4671-LA datasheet Rev 2.07 (Tests/Code_Coverage/shared/specs/TMC4671-LA_datasheet_Rev2.07.pdf):
 *      0x1A PWM_SV_CHOP, PWM_CHOP u8(7:0): 0 = "off, free running" (the bridge is not driven), 7 = PWM on with FOC (DS section 5.4: "After
 *        setting the register PWM_SV_CHOP to 7 the PWM is on and ready to use");
 *      0x63 MODE_RAMP_MODE_MOTION, MODE_MOTION u8(7:0): 0 = stopped_mode, 1 = torque_mode;
 *      0x64 PID_TORQUE_FLUX_TARGET: flux [15:0], torque [31:16], s16 each;
 *      0x76 INPUTS_RAW: ENABLE_IN_RAW bit 15;
 *      0x02 / 0x03 ADC_RAW_DATA / ADC_RAW_ADDR: ADC_RAW_ADDR 2 selects ADC_AGPI_A_RAW u16(31:16) and ADC_AGPI_B_RAW u16(15:0).
 *  - USB_PID1_01.pdf 5.12 (PID State report: Actuators Enabled, Safety Switch, Actuator Power) and the Set Constant Force report (5.6.1).
 *  - Safety properties that need no firmware constant (plain): a stopped drive has 0x1A = 0 or 0x63 = 0, the enable pin low, and no
 *    non-zero torque target is written while the drive is stopped.
 *  - MATH: a constant force of magnitude M (Set Constant Force, +-32767) at axis power P (axis.0.power) reaches the chip as M * P / 32767
 *    (the axis scaling, the same derivation as in test_f1_axis.cpp F1-X7): 20000 * 10000 / 32767 = 6103.7, +-2 counts.
 *
 * Firmware paths (verified in the sources): the E-stop pin -> HAL_GPIO_EXTI_Callback (global_callbacks.cpp:93-97) -> FFBHIDMain::exti
 * (FFBHIDMain.cpp:332-355; debounce HAL_GetTick() - lastEstop > 1000, lastEstop starts at 0) -> emergencyStop (:297-300) ->
 * Axis::emergencyStop (Axis.cpp:617-624: drv->turn(0), then drv->emergencyStop) -> TMC4671::emergencyStop (TMC4671.cpp:1701-1725:
 * enable pin reset, motorEnabledRequested = false, stopMotor: 0x63 = stopped and 0x1A = off, state Shutdown); the FFB loop skips the
 * torque update while control.emergency (updateControl, FFBHIDMain.cpp:153-158). ENABLE_IN: Running state, every "tick - lastStatTime >
 * 2000" of a loop that delays 200 ms (TMC4671.cpp:553-565). Supply: hasPower (:196-199) is evaluated at the end of every state machine pass
 * (:643-650). Temperature: getTemp (:365-385) in the 2 s check (:567-574).
 *
 * Bug cross-references (coverage register, cited not re-numbered): 33 (slew jump after E-stop release), 39 / 40 (E-stop paths), 152 (supply
 * loss), 163 (ENABLE_IN stop), 164 (over-temperature); integration IB 22 / IB 25.
 */
#include "doctest.h"

#include "ChipEventLog.h"
#include "FfbHost.h"
#include "HidReportDescriptor.h"
#include "IntegrationFixture.h"
#include "PidReportBuilder.h"
#include "PositionScript.h"
#include "TestHelpers.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using hid::ReportType;

namespace {

constexpr uint32_t kDirectionX = hid::usage(0x0A, 1);

// A Running pass of TMC4671::Run is Delay(200) (:577) plus the Delay(10) at the end of the loop body (:656) = 210 ms.
// TMC4671.cpp:553-565: the ENABLE_IN check runs when tick - lastStatTime > 2000 (strictly), evaluated once per pass. From the moment the input
// changes the check is therefore at most 2000 + 210 + 1 ms away.
constexpr uint32_t kRunningPassMs = 200 + 10;
constexpr uint32_t kEnableCheckBoundMs = 2000 + kRunningPassMs + 1;
// TMC4671.cpp:643-650: the supply check is at the end of every pass of the state machine: at most one Running pass (210 ms) + 1 ms away.
constexpr uint32_t kSupplyCheckBoundMs = kRunningPassMs + 1;
// CDC_CommandInterface::sendReplies drops buffered async broadcasts when more than parserTimeout = 500 ms passed since the last send
// (CommandInterface.cpp:274-279, CommandInterface.h:80) and sends them only at the next batchDone (:296-300, a command batch).
constexpr uint32_t kBroadcastKeepMs = 500;
// 20000 * 10000 / 32767 (MATH in the file comment)
constexpr double kConstantTorque = 20000.0 * 10000.0 / 32767.0;

struct SafetyFixture : IntegrationFixture {
	std::optional<FfbHost> host;
	PositionScript script;
	std::optional<ChipEventLog> log; // declared after host: destroyed before it

	FfbHost &h() { return *host; }

	/** bring-up, power 10000 and fxratio 255 (IB 7), 360 degrees, no idle spring, constant position 0, end of the start fade (1 s) */
	void prepare() {
		bringUp();
		setSpiFrameUs(0);
		host.emplace(*this);
		h().command("axis.0.power=10000");
		h().command("axis.0.fxratio=255");
		h().command("axis.0.degrees=360");
		h().command("axis.0.invert=0");
		h().command("axis.0.idlespring=0");
		script.setConstant(0);
		attachPositionScript(script);
		h().run(1500);
	}

	pid::PidReport makeSetEffect(int64_t index, int64_t direction) {
		pid::PidReport se = h().builder().make(ReportType::Output, pid::use::SetEffectReport);
		se.set(pid::use::EffectBlockIndex, index).select(pid::use::EtConstantForce).set(pid::use::Duration, 32767).set(pid::use::Gain, 255);
		se.flag(pid::use::DirectionEnable, true);
		se.setIn({pid::use::Direction}, kDirectionX, direction);
		return se;
	}
	void setMagnitude(int64_t idx, int64_t magnitude) {
		pid::PidReport sc = h().builder().make(ReportType::Output, pid::use::SetConstantForceReport);
		sc.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, magnitude);
		h().send(sc);
	}
	void startEffect(int64_t idx) {
		pid::PidReport eo = h().builder().make(ReportType::Output, pid::use::EffectOperationReport);
		eo.set(pid::use::EffectBlockIndex, idx).select(pid::use::OpEffectStart).set(pid::use::LoopCount, 1);
		h().send(eo);
	}
	/** Constant force, polar direction (0.01 degrees), Magnitude, started; returns the block index */
	int64_t startConstant(int64_t direction, int64_t magnitude) {
		const FfbHost::CreateResult c = h().createEffect(pid::use::EtConstantForce);
		REQUIRE(c.load.success());
		const int64_t idx = c.load.effectBlockIndex;
		h().send(makeSetEffect(idx, direction));
		setMagnitude(idx, magnitude);
		startEffect(idx);
		return idx;
	}
	/** prepare(), a constant force of 6104 at the chip, the drive on, and the event log attached (after every other observer) */
	int64_t runConstant() {
		prepare();
		const int64_t idx = startConstant(9000, 20000);
		h().run(2500); // more than the 1 s fade-in of the first ready cycle (Axis.cpp:447-449)
		REQUIRE(std::abs(torque()) > 6000);
		REQUIRE(driveOn());
		log.emplace(*this, chip1);
		return idx;
	}

	int16_t torque() const { return static_cast<int16_t>((chip1.reg(0x64) >> 16) & 0xffff); }
	uint32_t pwmMode() const { return chip1.reg(0x1A) & 0xff; }
	uint32_t motionMode() const { return chip1.reg(0x63) & 0xff; }
	bool enablePin() const { return HostTarget::outputPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin); }
	/** PWM on with FOC (0x1A = 7), torque mode (0x63 = 1) and the enable pin high */
	bool driveOn() const { return pwmMode() == 7 && motionMode() == 1 && enablePin(); }
	bool driveOff() const { return pwmMode() == 0 && !enablePin(); }
	bool driverReady() { return tmc()->getState() == TMC_ControlState::Running && tmc()->motorReady(); }
	bool runUntilReady(uint32_t maxMs) {
		for (uint32_t ms = 0; ms < maxMs; ms++) {
			if (driverReady()) {
				return true;
			}
			h().run(1);
		}
		return driverReady();
	}

	/** Torque values written to 0x64 after write index `from`, in order */
	std::vector<int16_t> writesFrom(size_t from) const {
		std::vector<int16_t> out;
		const std::vector<FluxTorque> all = fluxTorqueWrites(chip1);
		for (size_t i = from; i < all.size(); i++) {
			out.push_back(all[i].torque);
		}
		return out;
	}
	static int16_t maxAbs(const std::vector<int16_t> &v) {
		int m = 0;
		for (int16_t x : v) {
			m = std::max(m, std::abs(static_cast<int>(x)));
		}
		return static_cast<int16_t>(m);
	}
	/** Reply text of `sys.errors?` (not FfbHost::command: "None" is a valid answer here) */
	std::string errorList() { return cdcCommand("sys.errors?", 200); }
	/** The part of a CDC answer after the last "[sys.errors?|" (the command's own reply); "" if there is none. Broadcast frames of the error printer come before it */
	static std::string ownReply(const std::string &text) {
		const std::string key = "[sys.errors?|";
		const size_t at = text.rfind(key);
		return at == std::string::npos ? std::string() : text.substr(at);
	}
	/** Presses the E-stop after a clock advance that satisfies the first-press rule (tick > 1000, FFBHIDMain.cpp:343) */
	void press() {
		REQUIRE(HAL_GetTick() > 1000);
		pressEstop();
	}
};

// Thermistor of the default hardware configuration (TMC4671_configs.cpp:53-72, v1.3: R2 1500, R 10000, Beta 4300, limit 90 C, adcOffset 0):
// the formula of getTemp (TMC4671.cpp:365-385) for a raw ADC_AGPI_B word. The NTC Beta equation 1/T = 1/T0 + ln(R / R0) / Beta is the
// standard one (T0 = 298.15 K); the divider constant 43252 is the firmware's.
double thermistorCelsius(uint32_t raw16, double r2, double rNominal, double beta, int adcOffset) {
	const int32_t adc = static_cast<int32_t>(raw16 & 0xffff) - 0x7fff - adcOffset;
	if (adc <= 0) {
		return 0.0;
	}
	const double r = r2 * (43252.0 / adc);
	return 1.0 / (1.0 / 298.15 + std::log(r / rNominal) / beta) - 273.15;
}

} // namespace

TEST_SUITE("F1Safety") {

// ---------------------------------------------------------------------------------------------------------------------
// F1-T3  E-stop input
// ---------------------------------------------------------------------------------------------------------------------

// Plain. Safety property: pressing the E-stop while a force runs must remove the force from the chip within one FFB cycle and stop the drive
// (oracle: DS 0x1A = 0 "off, free running", the enable pin low, 0x64 target 0). The press is the active low pin (FFBHIDMain.cpp:341); the
// firmware's path is the exti, which zeroes the torque first (Axis.cpp:618) and stops the driver (TMC4671.cpp:1701-1709).
TEST_CASE_FIXTURE(SafetyFixture, "F1-T3: an E-stop press during a constant force puts 0 at 0x64, releases the enable pin and switches the PWM off" * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	const size_t mark = fluxTorqueWrites(chip1).size();
	const size_t evMark = log->events().size();
	press();
	REQUIRE(firstFault().empty());
	h().run(1); // one FFB cycle (1 kHz default, main.hidsendspd 0)
	MESSAGE("after the press and 1 ms: 0x64 torque " << torque() << ", 0x1A " << pwmMode() << ", 0x63 " << motionMode() << ", enable " << enablePin());
	CHECK(torque() == 0);
	CHECK(pwmMode() == 0);
	CHECK_FALSE(enablePin());
	// every write to 0x64 since the press is 0 (no torque ever leaves the chip's target after the press)
	const std::vector<int16_t> after = writesFrom(mark);
	CHECK(maxAbs(after) == 0);
	// order at the chip: a PWM off write happened while the enable pin was already low (TMC4671::emergencyStop resets the pin before stopMotor)
	size_t off = log->firstWrite(0x1A, [](uint32_t d) { return (d & 0xff) == 0; }, evMark);
	REQUIRE(off != ChipEventLog::npos);
	CHECK_FALSE(log->events()[off].enable);
	// and the PWM is not switched on again afterwards (the E-stop path does not go through the critical error handler)
	CHECK(log->firstWrite(0x1A, [](uint32_t d) { return (d & 0xff) == 7; }, off) == ChipEventLog::npos);
	CHECK(log->complete());
	REQUIRE_CHAIN_MODELLED();
}

// Plain. During the stop the host keeps sending: a new magnitude of the running effect, a second effect started. None of it may reach the
// target of the chip, the PWM or the enable pin (FFBHIDMain::updateControl skips the torque update while control.emergency, :153-158).
TEST_CASE_FIXTURE(SafetyFixture, "F1-T3: torque, PWM and enable pin stay off while new reports keep arriving during the E-stop" * IT_DELETES_FFBHIDMAIN) {
	const int64_t idx = runConstant();
	press();
	h().run(20);
	REQUIRE(torque() == 0);
	const size_t mark = fluxTorqueWrites(chip1).size();
	setMagnitude(idx, 32767);
	h().run(100);
	const int64_t idx2 = startConstant(27000, 30000);
	h().run(100);
	setMagnitude(idx2, -20000);
	h().run(300);
	MESSAGE("during the stop: " << fluxTorqueWrites(chip1).size() - mark << " writes to 0x64, last torque " << torque());
	const std::vector<int16_t> after = writesFrom(mark);
	CHECK(maxAbs(after) == 0);
	CHECK(torque() == 0);
	CHECK(pwmMode() == 0);
	CHECK_FALSE(enablePin());
	CHECK(motionMode() == 0);
	REQUIRE_CHAIN_MODELLED();
}

// characterization: the recovery after the release is firmware behaviour only (PID 1.01 and the TMC datasheet do not say what a device does
// when its safety switch is released). Observed path: release -> FFBHIDMain::emergencyStop(true) -> Axis::emergencyStop(true) starts a fade
// and TMC4671::emergencyStop(true) -> startMotor (Shutdown -> Running, 0x1A = 7, enable pin set, torque mode). The running effect continues.
// INR 36: the force returns at its full torque without any new report, within the 1 s fade.
TEST_CASE_FIXTURE(SafetyFixture, "characterization: [F1-T3] after the E-stop is released the drive restarts and the running constant force returns to its torque" * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	press();
	h().run(1100); // the debounce window of FFBHIDMain.cpp:343 (> 1000 ms after the accepted press)
	REQUIRE(torque() == 0);
	releaseEstop();
	REQUIRE(firstFault().empty());
	h().run(3000); // 1 s fade (Axis.cpp:620) and the driver restart
	MESSAGE("after the release: 0x64 torque " << torque() << ", 0x1A " << pwmMode() << ", 0x63 " << motionMode() << ", enable " << enablePin() << ", TMC state " << static_cast<uint32_t>(tmc()->getState()));
	CHECK(driveOn());
	CHECK_NEAR(std::abs(torque()), kConstantTorque, 2);
	REQUIRE_CHAIN_MODELLED();
}

// Bug 33 (coverage): after an E-stop reset the slew limiter starts from the torque before the stop. Oracle: the user setting axis.0.slewrate
// (help "Torque rate limit in counts/ms", Axis.h:484): with slewrate R no two consecutive torque values may differ by more than R, and the
// chip held 0 when the force was released, so the first torque after the release is at most R in magnitude. The case sets R = 300.
// Axis::emergencyStop(true) starts a fade (Axis.cpp:619-620) but Axis::applyTorqueSlewRateLimiter clips against metric.previous.torque,
// which still holds the torque from before the stop (Axis.cpp:917-921), so the first value is far above R.
TEST_CASE_FIXTURE(SafetyFixture, "F1-T3: with slewrate R the first torques after an E-stop release do not step by more than R from the 0 the chip held" * KNOWN_ISSUE("(IB 40; same as bug 33) the slew limiter starts from the torque before the E-stop (Axis.cpp:917-921, metric.previous.torque), so the first torque after the release jumps beyond slewrate") * IT_DELETES_FFBHIDMAIN) {
	prepare();
	h().command("axis.0.slewrate=300");
	REQUIRE(h().getValue("axis.0.slewrate?") == 300);
	startConstant(9000, 20000);
	h().run(2500);
	REQUIRE(std::abs(torque()) > 6000);
	press();
	h().run(1100);
	REQUIRE(torque() == 0);
	const size_t mark = fluxTorqueWrites(chip1).size();
	releaseEstop();
	h().run(300);
	const std::vector<int16_t> after = writesFrom(mark);
	REQUIRE(!after.empty());
	int prev = 0; // the chip held 0 (turn(0) of the press)
	int worst = 0;
	for (int16_t t : after) {
		worst = std::max(worst, std::abs(static_cast<int>(t) - prev));
		prev = t;
	}
	MESSAGE("after the release: " << after.size() << " writes, first " << after.front() << ", largest step " << worst);
	REQUIRE_CHAIN_MODELLED();
	CHECK(worst <= 300);
}

// characterization: the host-visible E-stop state is what the firmware defines: `main.estop?` (help "Emergency stop", FFBHIDMain_commmands.cpp:29,
// 34-36) returns control.emergency and `main.ffbactive?` returns -1 in an emergency (:46-51). The spec does not define a PID-side or CDC-side
// signal, so the values are INR 37.
TEST_CASE_FIXTURE(SafetyFixture, "characterization: [F1-T3] the host reads the E-stop through main.estop? (1) and main.ffbactive? (-1) and both return to normal after the release" * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	REQUIRE(h().getValue("main.estop?") == 0);
	REQUIRE(h().getValue("main.ffbactive?") == 1);
	press();
	h().run(20);
	CHECK(h().getValue("main.estop?") == 1);
	CHECK(h().getValue("main.ffbactive?") == -1);
	const std::string errors = errorList();
	MESSAGE("sys.errors? during the E-stop: " << errors);
	h().run(1100);
	releaseEstop();
	h().run(20);
	CHECK(h().getValue("main.estop?") == 0);
	CHECK(h().getValue("main.ffbactive?") == 1);
	REQUIRE_CHAIN_MODELLED();
}

// characterization: the E-stop input itself raises no error: FFBHIDMain::exti calls emergencyStop without ErrorHandler::addError (FFBHIDMain.cpp:
// 345-350; the estopError member of FFBHIDMain.h:99 is never added), so neither the error printer nor sys.errors? tells the host. INR 43.
TEST_CASE_FIXTURE(SafetyFixture, "characterization: [F1-T3] the E-stop input raises no error for the host" * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	press();
	h().run(500);
	const std::string traffic = errorList();
	MESSAGE("CDC traffic after the E-stop: " << traffic);
	CHECK(traffic.find("crit") == std::string::npos);
	CHECK(traffic.find("None") != std::string::npos);
	REQUIRE_CHAIN_MODELLED();
}

// characterization: the PID State input report (PID 1.01 5.12: Actuators Enabled, Safety Switch, Actuator Power) after the E-stop. HidFFB::
// sendStatusReport (HidFFB.cpp:71-86) builds it from ffb_active only: Actuator Power always set, Safety Switch never (INR 5), so the E-stop is
// not visible in it. The report is sent on an effect operation (a new effect here). INR 38.
TEST_CASE_FIXTURE(SafetyFixture, "characterization: [F1-T3] the PID State report sent during an E-stop does not signal it" * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	press();
	h().run(20);
	const size_t mark = h().stateMark();
	const FfbHost::CreateResult c = h().createEffect(pid::use::EtConstantForce);
	REQUIRE(c.load.success());
	h().run(5);
	const std::vector<FfbHost::PidState> st = h().statesSince(mark);
	REQUIRE(!st.empty());
	REQUIRE(st.back().errors.empty());
	CHECK(st.back().actuatorPower);
	CHECK_FALSE(st.back().safetySwitch);
	MESSAGE("PID state during the E-stop: actuatorsEnabled " << st.back().actuatorsEnabled << ", effectPlaying " << st.back().effectPlaying);
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-T4  ENABLE_IN of the TMC
// ---------------------------------------------------------------------------------------------------------------------

// Plain (safety property). ENABLE_IN low (DS 0x76 bit 15) while a force runs: within one poll period of the Running loop
// (kEnableCheckBoundMs = 2000 + 210 + 1) the drive is off: PWM off, motion stopped, enable pin low (TMC4671::emergencyStop(false) resets the
// pin and calls stopMotor, TMC4671.cpp:1701-1709), and no non-zero torque is written meanwhile or afterwards while the host keeps sending.
TEST_CASE_FIXTURE(SafetyFixture, "F1-T4: ENABLE_IN low during a constant force switches the drive off within one poll period" * IT_DELETES_FFBHIDMAIN) {
	const int64_t idx = runConstant();
	chip1.setEnableIn(false);
	const size_t mark = fluxTorqueWrites(chip1).size();
	h().run(kEnableCheckBoundMs);
	MESSAGE("after " << kEnableCheckBoundMs << " ms: 0x1A " << pwmMode() << ", 0x63 " << motionMode() << ", enable " << enablePin() << ", 0x64 torque " << torque() << ", TMC state " << static_cast<uint32_t>(tmc()->getState()));
	CHECK(pwmMode() == 0);
	CHECK(motionMode() == 0);
	CHECK_FALSE(enablePin());
	CHECK(maxAbs(writesFrom(mark)) == 0); // no non-zero torque target written since the input dropped
	// the host keeps sending: nothing drives the chip again
	setMagnitude(idx, 32767);
	h().run(300);
	CHECK(pwmMode() == 0);
	CHECK_FALSE(enablePin());
	CHECK(motionMode() == 0);
	REQUIRE_CHAIN_MODELLED();
}

// Plain: the torque target of the chip. The order requires 0 at 0x64 after the ENABLE_IN stop. DEPENDENCY (lead's mutation result): this passes
// today only through the defective restart path of bug 39 / IB 42 below: errorCallback's emergencyStop(true) restarts the driver, so the
// following emergencyStop(false) -> Axis::emergencyStop's turn(0) finds motorReady and writes 0x64 = 0. TMC4671::emergencyStop itself does not
// zero 0x64, and with errorCallback fixed to emergencyStop(false) only, turn(0) returns early (TMC4671.cpp:1746, motorEnabledRequested false) and
// 0x64 would keep the old torque.
TEST_CASE_FIXTURE(SafetyFixture, "F1-T4: ENABLE_IN low during a constant force puts 0 at 0x64 within one poll period" * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	chip1.setEnableIn(false);
	h().run(kEnableCheckBoundMs);
	MESSAGE("0x64 torque " << torque() << ", PWM " << pwmMode());
	REQUIRE_CHAIN_MODELLED();
	CHECK(torque() == 0);
}

// The error reaches the host. TMC4671::estopError (TMC4671.h:641, code 2 emergencyStop, critical) is added at the stop (TMC4671.cpp:563) and the
// ErrorPrinter thread broadcasts the list (ErrorHandler.cpp:159-169, "[sys.0.errors?|2:crit:TMC emergency stop triggered]"). The CDC interface
// only buffers an async broadcast until the next command batch ends and drops it when more than parserTimeout = 500 ms passed (kBroadcastKeepMs),
// so the host reads the error with a `sys.errors?` sent right after the stop: the exchange contains the frame. Plain: the safety event reaches the
// host (the text is the firmware's own string).
TEST_CASE_FIXTURE(SafetyFixture, "F1-T4: a sys.errors? read within 500 ms of the ENABLE_IN stop returns the TMC emergency stop error" * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	chip1.setEnableIn(false);
	uint32_t ms = 0;
	while (ms < kEnableCheckBoundMs && pwmMode() != 0) {
		h().run(1);
		ms++;
	}
	REQUIRE(pwmMode() == 0); // the stop happened (the error is added in the same pass)
	h().run(50);
	const std::string traffic = errorList();
	MESSAGE("exchange 50 ms after the stop: " << traffic);
	CHECK(traffic.find("2:crit:TMC emergency stop triggered") != std::string::npos);
	REQUIRE_CHAIN_MODELLED();
}

// characterization: the broadcast frame is discarded when no command arrives within parserTimeout = 500 ms of it (CommandInterface.cpp:274-279):
// a host that reads 600 ms after the stop sees neither the frame nor, through sys.errors?, the error (bug 7 hides it). INR 44.
TEST_CASE_FIXTURE(SafetyFixture, "characterization: [F1-T4] the ENABLE_IN stop error frame is discarded when no command arrives within 500 ms" * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	chip1.setEnableIn(false);
	uint32_t ms = 0;
	while (ms < kEnableCheckBoundMs && pwmMode() != 0) {
		h().run(1);
		ms++;
	}
	REQUIRE(pwmMode() == 0);
	h().run(kBroadcastKeepMs + 100);
	const std::string traffic = errorList();
	MESSAGE("exchange 600 ms after the stop: " << traffic);
	CHECK(traffic.find("TMC emergency stop triggered") == std::string::npos);
	REQUIRE_CHAIN_MODELLED();
}

// KNOWN ISSUE (coverage bug 7): the active critical error is also what `sys.errors?` is meant to list (ErrorHandler.h:53-54: critical errors
// "have to be cleared externally"). After the printer's pass (clearTemp -> sortErrors puts the empty slots first, getErrors() stops at the
// first empty slot) the list reads None while the error is still stored and the drive stays stopped.
TEST_CASE_FIXTURE(SafetyFixture, "F1-T4: sys.errors? still lists the TMC emergency stop error after the ENABLE_IN stop" * KNOWN_ISSUE("(IB 41; same as bug 7) getErrors() returns an empty list after any clearTemp / clearError (ErrorHandler.cpp:130-143), so sys.errors? says None while the critical error is stored") * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	chip1.setEnableIn(false);
	h().run(kEnableCheckBoundMs);
	REQUIRE(pwmMode() == 0); // the stop happened, the error was raised with it
	const std::string errors = ownReply(errorList()); // the command's own reply, after the printer's pass (its frame, if any, comes before it)
	MESSAGE("sys.errors? reply: " << errors);
	REQUIRE(!errors.empty());
	REQUIRE_CHAIN_MODELLED();
	CHECK(errors.find("TMC emergency stop triggered") != std::string::npos);
}

// Safety property: once the ENABLE_IN stop switched the PWM off, it must not be switched on again while ENABLE_IN is low, and certainly not
// while 0x64 still holds the old torque. The firmware raises the critical estopError after TMC4671::emergencyStop(false); FFBHIDMain::
// errorCallback (FFBHIDMain.cpp:360-368) treats every critical error by emergencyStop(true) first, which reaches startMotor() (Shutdown ->
// Running, 0x1A = 7, enable pin high, torque mode with the old 0x64) and only then stops again for the code emergencyStop. Same as bug 39
// ("a new critical error releases an active emergency stop", FFBHIDMain.cpp:361-364).
TEST_CASE_FIXTURE(SafetyFixture, "F1-T4: after the ENABLE_IN stop the PWM is not switched on again while ENABLE_IN is low" * KNOWN_ISSUE("(IB 42; same as bug 39) the critical error handler calls emergencyStop(true) (FFBHIDMain.cpp:362-364), which restarts the drive (TMC4671.cpp:1711-1724) with the old 0x64 before the second emergencyStop(false) writes 0") * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	chip1.setEnableIn(false);
	h().run(kEnableCheckBoundMs);
	size_t off = log->firstWrite(0x1A, [](uint32_t d) { return (d & 0xff) == 0; });
	REQUIRE(off != ChipEventLog::npos);
	for (size_t i = 0; i < log->events().size(); i++) {
		const ChipEventLog::Event &e = log->events()[i];
		if (e.kind == ChipEventLog::Kind::Write && (e.addr == 0x1A || e.addr == 0x64 || e.addr == 0x63) && i >= off) {
			MESSAGE("event " << i << ": W" << static_cast<int>(e.addr) << " (decimal) = " << e.data << " enable " << e.enable);
		}
	}
	REQUIRE_CHAIN_MODELLED();
	CHECK(log->firstWrite(0x1A, [](uint32_t d) { return (d & 0xff) == 7; }, off) == ChipEventLog::npos);
}

// The state the firmware reports to the host after the stop: control.emergency is set (main.estop? = 1, main.ffbactive? = -1) because
// FFBHIDMain::errorCallback ends in emergencyStop(false) for the code emergencyStop (FFBHIDMain.cpp:366-368). characterization: that this is the
// host-visible state is firmware-defined. INR 39.
TEST_CASE_FIXTURE(SafetyFixture, "characterization: [F1-T4] after ENABLE_IN low the host reads main.estop? = 1 and a new magnitude does not restart the drive" * IT_DELETES_FFBHIDMAIN) {
	const int64_t idx = runConstant();
	chip1.setEnableIn(false);
	h().run(kEnableCheckBoundMs);
	MESSAGE("main.estop? " << h().getValue("main.estop?") << ", ffbactive " << h().getValue("main.ffbactive?"));
	CHECK(h().getValue("main.estop?") == 1);
	CHECK(h().getValue("main.ffbactive?") == -1);
	setMagnitude(idx, 32767);
	h().run(300);
	CHECK(pwmMode() == 0);
	REQUIRE_CHAIN_MODELLED();
}

// characterization: recovery when ENABLE_IN returns high. TMC4671::Run in Shutdown with estopTriggered (TMC4671.cpp:581-592) is meant to clear
// the error and re-initialise, but the host-visible state stays emergency (main.estop? 1, FFBHIDMain control.emergency), so the drive stays off
// (bug 163: the recovery in Shutdown is unreachable: the thread suspends itself). Observed here: after 10 s the drive is still off; the host's own way out is `main.estop=0`
// (FFBHIDMain_commmands.cpp:37-38, emergencyStop(true)). INR 40.
TEST_CASE_FIXTURE(SafetyFixture, "characterization: [F1-T4] after ENABLE_IN returns high the drive stays off until the host clears the emergency with main.estop=0" * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	chip1.setEnableIn(false);
	h().run(kEnableCheckBoundMs);
	REQUIRE(pwmMode() == 0);
	chip1.setEnableIn(true);
	h().run(10000);
	MESSAGE("10 s after ENABLE_IN high: TMC state " << static_cast<uint32_t>(tmc()->getState()) << ", 0x1A " << pwmMode() << ", 0x63 " << motionMode() << ", enable " << enablePin() << ", 0x64 torque " << torque() << ", main.estop? " << h().getValue("main.estop?"));
	CHECK(pwmMode() == 0);
	CHECK_FALSE(enablePin());
	CHECK(h().getValue("main.estop?") == 1);
	h().command("main.estop=0");
	h().run(3000);
	MESSAGE("after main.estop=0: TMC state " << static_cast<uint32_t>(tmc()->getState()) << ", 0x1A " << pwmMode() << ", 0x63 " << motionMode() << ", enable " << enablePin() << ", 0x64 torque " << torque() << ", main.estop? " << h().getValue("main.estop?"));
	CHECK(h().getValue("main.estop?") == 0);
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-T5  undervoltage and over-temperature
// ---------------------------------------------------------------------------------------------------------------------

// Plain (safety property). The supply drops below the hasPower threshold (internal and external above 10 V, internal below 78 V,
// TMC4671.cpp:196-199): within one Running pass (kSupplyCheckBoundMs) the driver stops the motion (setMotionMode(stop), :643-650).
// 0x63 = 0 is stopped_mode (DS): no torque control is active. 5 V on both rails.
TEST_CASE_FIXTURE(SafetyFixture, "F1-T5: a supply below the hasPower threshold stops the motion mode of the chip within one state machine pass" * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	setSupply(5000, 5000);
	h().run(kSupplyCheckBoundMs);
	MESSAGE("after " << kSupplyCheckBoundMs << " ms low: 0x63 " << motionMode() << ", 0x1A " << pwmMode() << ", enable " << enablePin() << ", 0x64 torque " << torque() << ", TMC state " << static_cast<uint32_t>(tmc()->getState()));
	CHECK(motionMode() == 0);
	REQUIRE_CHAIN_MODELLED();
}

// The torque at the chip while the supply is low (the order: torque 0 at the chip while low). The driver reacts with setMotionMode(stop) only
// (TMC4671.cpp:643-650): PWM stays 7, the enable pin high and 0x64 keeps the old torque (-6103). Intended: 0x64 = 0 (the same charter rule as IB 25,
// whose mechanism is the axis writing 0x64 only on change, Axis.cpp:488-495; here nothing zeroes the target when the driver stops).
TEST_CASE_FIXTURE(SafetyFixture, "F1-T5: the torque target at the chip is 0 while the supply is low" * KNOWN_ISSUE("(IB 43; related to IB 25) the undervoltage stop changes only the motion mode (TMC4671.cpp:643-650); 0x64 keeps the old torque, PWM stays on and the enable pin high") * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	setSupply(5000, 5000);
	h().run(kSupplyCheckBoundMs);
	MESSAGE("0x64 torque " << torque() << ", 0x1A " << pwmMode() << ", enable " << enablePin() << ", 0x63 " << motionMode());
	REQUIRE_CHAIN_MODELLED();
	CHECK(torque() == 0);
}

// The error reaches the host: lowVoltageError (TMC4671.h:639, code 10 undervoltage, warning, TMC4671.cpp:646) is broadcast by the ErrorPrinter
// thread on CDC as "[sys.0.errors?|10:warn:Low motor voltage]" (ErrorHandler.cpp:159-169; format Error::toString). Delivered with the next CDC
// exchange: the CDC interface keeps an async broadcast until the next command batch and drops it after parserTimeout = 500 ms
// (CommandInterface.cpp:274-302), so a read loop during the dropout sees nothing and the exchange 50 ms after the stop gets the frame.
TEST_CASE_FIXTURE(SafetyFixture, "F1-T5: undervoltage sends the low motor voltage warning to the host on CDC" * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	setSupply(5000, 5000);
	h().run(kSupplyCheckBoundMs + 50); // < kBroadcastKeepMs
	const std::string traffic = errorList();
	MESSAGE("CDC traffic at the next exchange after the dropout: " << traffic);
	CHECK(traffic.find("10:warn:Low motor voltage") != std::string::npos);
	REQUIRE_CHAIN_MODELLED();
}

// characterization: recovery after the supply returns. Derivation from the source: in waitPower without power a pass is Delay(250) (:480) + the
// Delay(10) of :656 = 260 ms; with power (powerInitialized is already true, so no new initialisation and no counter wait) the pass sets
// allowStateChange and ends with Delay(100) + 10 = 110 ms (:488-491, :656), and the next loop head takes requestedState (:451-453). So the
// state is Running again at most 260 + 110 + 1 = 371 ms after the supply returns. What the drive state is afterwards is firmware behaviour: the
// motion mode is not restored (0x63 stays 0, bug 152 / IB 25, whose own cases are in test_f1_axis.cpp). INR 41.
TEST_CASE_FIXTURE(SafetyFixture, "characterization: [F1-T5] after the supply returns the driver is Running again within the waitPower bound" * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	setSupply(5000, 5000);
	h().run(500);
	REQUIRE_FALSE(driverReady());
	setSupply(24000, 24000);
	uint32_t ms = 0;
	while (ms < 371 && !driverReady()) {
		h().run(1);
		ms++;
	}
	h().run(5);
	MESSAGE("ready after " << ms << " ms: 0x63 " << motionMode() << ", 0x1A " << pwmMode() << ", enable " << enablePin() << ", 0x64 torque " << torque());
	CHECK(driverReady());
	REQUIRE_CHAIN_MODELLED();
}

// Over-temperature. The default hardware configuration of a fresh flash is the first entry of tmc4671_hw_configs (TMC4671.cpp:186,
// TMC4671_configs.cpp:107: v1.3) and has temperatureEnabled = true, so the check runs without any command. getTemp (TMC4671.cpp:365-385) reads
// the thermistor on ADC_AGPI_B (ADC_RAW_ADDR 2, low 16 bits), adc = word - 0x7fff - adcOffset, r = R2 * 43252 / adc, then the Beta equation.
// MATH for the largest word 0xFFFF (v1.3: R2 1500, R 10000, Beta 4300, offset 0): adc = 0xFFFF - 0x7FFF = 32768; r = 1500 * 43252 / 32768 =
// 1980 ohm; 1/T = 1/298.15 + ln(1980 / 10000) / 4300 -> T = 62.72 C, i.e. `tmc.0.temp?` (C * 100, TMC4671.cpp:3047-3049) = 6271. The 90 C limit is
// therefore never reachable with the v1.3 config (nor with v1.2: R 22000, offset 1000: 83.84 C).
// INR 42 (text for the register): (1) the firmware's r = R2 * Vref / V (the 43252 constant over the ADC count) matches neither standard divider
// equation (R2 / (R2 + r) or r / (R2 + r)); (2) with the v1.3 / v1.2 configs the 90 C limit is unreachable by any ADC word, so the OverTemp stop
// (and bug 164's stop path) cannot be reached. The intended mapping needs the board's divider circuit (owner question IQ 14).
// This case reads the firmware's own value for the largest word and shows that the driver keeps running.
TEST_CASE_FIXTURE(SafetyFixture, "characterization: [F1-T5] the largest possible thermistor ADC word never reaches the 90 C limit, so the over-temperature stop is never taken" * IT_DELETES_FFBHIDMAIN) {
	runConstant();
	chip1.setAdcRaw(2, 0xffff);
	h().run(3 * 2000 + 600); // at least three of the 2 s checks (TMC4671.cpp:553-554)
	const int64_t temp = h().getValue("tmc.0.temp?");
	MESSAGE("tmc.0.temp? " << temp << "; TMC state " << static_cast<uint32_t>(tmc()->getState()) << ", 0x64 torque " << torque());
	CHECK(temp == 6271);
	CHECK(tmc()->getState() == TMC_ControlState::Running);
	CHECK(driveOn());
	// independent copy of the formula: the v1.2 maximum stays below the limit as well (v1.2: TMC4671_configs.cpp:34-52)
	CHECK_NEAR(thermistorCelsius(0xffff, 1500, 22000, 4300, 1000), 83.84, 0.05);
	REQUIRE_CHAIN_MODELLED();
}

} // TEST_SUITE
