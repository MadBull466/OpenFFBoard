/*
 * F1 setup (WO I0-4 and I0-5): the smoke case of the integration executable, and the first requirement tests of the F1
 * charter group G-A (infrastructure self-checks) that need the IntegrationFixture: F1-A1 (the chain comes up) and F1-A4
 * (the scheduler's rates).
 */
#include "doctest.h"

#include "HostDcd.h"
#include "HostPlatform.h"
#include "HostTarget.h"
#include "IntegrationFixture.h"
#include "TestHelpers.h"
#include "usbd.h"

#include <iostream>

TEST_SUITE("F1Setup") {

// Oracle: the TinyUSB device API (usbd.h, tud_inited / tud_deinit): tud_deinit shuts the device stack down and after it
// tud_inited() reports false until tud_init is called again. The case does not depend on test order: later F1 cases
// enumerate the device, and tud_deinit does not clear the configuration number (usbd.c, so tud_mounted() would stay
// true), but it does define tud_inited(). The reset order is the one of plan section 3.3 (tud_deinit first).
TEST_CASE("the device stack is not initialised after the global reset") {
	tud_deinit(0);
	HostDcd::reset();
	HostPlatform::reset();
	HostTarget::reset();
	CHECK_FALSE(tud_inited());
}

// Oracle: the firmware's own start-up sequence (requirement table R70, R72): cppmain selects the main class from
// ADR_CURRENT_CONFIG (default DEFAULTMAIN = 1 = FFBWheel), the USB device enumerates, the user selects the driver with the
// axis command, the TMC thread runs its state machine (uninitialized, waitPower, init with power, encoder init) to Running.
// The bound below is a test budget, not a firmware property; the time the chain actually needed is printed.
//
// What the board received when the motor started (TMC4671 datasheet Rev 2.07, Tests/Code_Coverage/shared/specs/
// TMC4671-LA_datasheet_Rev2.07.pdf; the driver's TMC4671::startMotor, TMC4671.cpp:1682-1699, does it on the USB mount):
//  - DRV_ENABLE (PE7, main.h:110-111) high: enablePin.set();
//  - register 0x1A PWM_SV_CHOP, field PWM_CHOP u8(7:0), datasheet section 7.2 p. 85: "7: centered PWM for FOC". The firmware
//    chooses TMC_PwmMode::PWM_FOC (TMC4671.h:63, value 7) and writes it with updateReg (TMC4671.cpp:2423): the low byte of
//    the last write is 7 (the other bits, SVPWM bit 8, are the firmware's choice and not asserted);
//  - register 0x63 MODE_RAMP_MODE_MOTION, field MODE_MOTION u8(7:0), datasheet p. 97-98: "1: torque_mode". The firmware chooses
//    torque mode: setupDriver sets nextMotionMode = MotionMode::torque (TMC4671.cpp:~250, value 1 at TMC4671.h:71) and
//    startMotor writes it (TMC4671.cpp:1692): the low byte of the last write is 1.
// To see the start frames the case lets the TMC reach Shutdown first, sets DRV_ENABLE low (as before a start request) and looks
// only at frames after the USB re-mount.
TEST_CASE_FIXTURE(IntegrationFixture, "F1-A1: the chain comes up with FFBWheel, USB enumerated, TMC_1 on chip 1, supply present and TMC Running" * IT_DELETES_FFBHIDMAIN) {
	const uint32_t budgetMs = 20000;
	reachShutdown(budgetMs);
	HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
	mountAndRun(budgetMs);
	const uint32_t needed = bringUpMs;
	MESSAGE("F1-A1: Running and motorReady() after " << needed << " ms of simulated time (test budget " << budgetMs << " ms)");
	CHECK(needed <= budgetMs);

	CHECK(isFfbWheel());
	CHECK(tud_mounted());
	CHECK(tud_connected());
	REQUIRE(tmc() != nullptr);
	CHECK(tmc()->getState() == TMC_ControlState::Running);
	CHECK(tmc()->motorReady());
	CHECK(tmc()->hasPower());
	// TMC_1 is the driver on chip 1 (CS PA4): the chip answered the driver, chip 2 never saw a frame
	CHECK(chip1.log().size() > 0);
	CHECK(chip2.log().size() == 0);
	CHECK(tmc()->getInfo().id == CLSID_MOT_TMC0);
	// The axis reports the driver type the user selected: value 1 (TMC_1)
	const std::string reply = cdcCommand("axis.0.drvtype?");
	INFO("reply: " << reply);
	CHECK(reply.find("axis.0.drvtype") != std::string::npos);
	CHECK(reply.find("|1]") != std::string::npos);

	// What the board received at the motor start
	CHECK(HostTarget::outputPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin));
	int pwm = -1, mode = -1;
	for (size_t i = startMark; i < chip1.log().size(); i++) {
		const FakeTmc4671::Frame &f = chip1.log()[i];
		if (f.write && f.addr == 0x1A) {
			pwm = static_cast<int>(f.data & 0xFF);
		}
		if (f.write && f.addr == 0x63) {
			mode = static_cast<int>(f.data & 0xFF);
		}
	}
	CHECK(pwm == 7);  // PWM_CHOP 7: centered PWM for FOC (DS p. 85)
	CHECK(mode == 1); // MODE_MOTION 1: torque_mode (DS p. 98)
}

// INR 1: documents today's start behaviour on a fresh flash (needs review: should a board with motor type NONE and encoder NONE
// leave Shutdown at all, and should the start request of Axis::setDrvType, made while the TMC is still uninitialised, be kept?).
// After axis.0.drvtype=1 with USB connected (Axis::setDrvType calls startMotor, Axis.cpp:542) the TMC initialises and stops in
// Shutdown (TMC_ControlState 2): setEncoderType(NONE) ends in Shutdown. A USB re-mount (tud_mount_cb -> Axis::usbResume ->
// startMotor, TMC4671.cpp:1685) then makes it Running although motor type and encoder type are still NONE.
TEST_CASE_FIXTURE(IntegrationFixture, "characterization: [F1-A1] with USB connected the TMC stays in Shutdown after axis.0.drvtype=1 on a fresh flash and runs after a USB re-mount with motor and encoder type NONE" * IT_DELETES_FFBHIDMAIN) {
	boot();
	enumerateUsb();
	selectDriver(1);
	REQUIRE(tmc() != nullptr);
	REQUIRE(runUntilTmcState(TMC_ControlState::Shutdown, 20000));
	const uint64_t t0 = HostPlatform::nowUs();
	sched.advanceMs(10000); // waited 10 s of simulated time after reaching Shutdown
	MESSAGE("INR 1: still in state " << static_cast<uint32_t>(tmc()->getState()) << " (Shutdown = 2) after " << (HostPlatform::nowUs() - t0) / 1000 << " ms");
	CHECK(tmc()->getState() == TMC_ControlState::Shutdown);
	CHECK_FALSE(tmc()->motorReady());
	const std::string mtype = cdcCommand("tmc.0.mtype?");
	const std::string encsrc = cdcCommand("tmc.0.encsrc?");
	INFO("mtype reply " << mtype << ", encsrc reply " << encsrc);
	CHECK(mtype.find("|0]") != std::string::npos); // MotorType::NONE
	CHECK(encsrc.find("|0]") != std::string::npos); // EncoderType_TMC::NONE
	replugUsb();
	REQUIRE(runUntilTmcState(TMC_ControlState::Running, 5000));
	CHECK(tmc()->motorReady());
	CHECK(cdcCommand("tmc.0.mtype?").find("|0]") != std::string::npos);
	CHECK(cdcCommand("tmc.0.encsrc?").find("|0]") != std::string::npos);
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-A4: the scheduler's rates. Hand derivation of the expected values (no firmware output is copied):
//
// FFB timer TIM13. The CubeMX init sets PSC = 84 - 1 = 83 (Targets/F407VG/Core/Src/main.c:1203) and the timer clock of
// APB1 timers is 84 MHz, so one timer count is 1 us. FFBHIDMain::setReportRate writes ARR = 1000000 * basediv / basefreq
// (FFBHIDMain.cpp:256) with basediv 1 and basefreq 1000 at the default mode 0 (FFBHIDMain.h:124-125, full speed USB):
// ARR = 1000. An up-counter with auto-reload ARR overflows every ARR + 1 counts (RM0090 "General-purpose timers", update
// event), so the update period is (ARR + 1) * (PSC + 1) / 84 MHz = 1001 * 84 / 84e6 = 1.001 ms, not 1.000 ms (the firmware
// comment says "assumes 1us timer steps" and does not add the one count; same pattern as coverage bug 201). Consequences:
//   - 100 ms = 8 400 000 timer clocks / 84 084 per update = 99.90 updates: 99 or 100 depending on the phase;
//   - 1001 ms = 84 084 000 clocks = exactly 1000 updates for any phase (84 084 * 1000 = 84 084 000), not 1001.
// So at the default rate one FFB cycle per simulated millisecond is NOT what the timer delivers: 0.1 % fewer. The tests below
// assert the derived values and say so; whether the missing count matters (rate 999 Hz, filter sample rate assumed 1000 Hz
// by updateSamplerate) is a candidate integration bug for the lead, not decided here.
//
// FFB cycles: FFBMAIN waits for a task notification that the TIM13 update ISR gives (FFBHIDMain::timerElapsed,
// FFBHIDMain.cpp:379) and runs updateControl once per wake. updateControl reads the encoder once per axis
// (AxesManager::update -> Axis::prepareForUpdate -> TMC4671::getPos: one frame reading 0x6B), a second, independent
// evidence of a cycle on the chip.
//
// TMC thread in Running: one pass of TMC4671::Run is Delay(200) (TMC4671.cpp:577, the Running case) plus the Delay(10) at the
// end of the loop (TMC4671.cpp:656): two wake-ups per 210 ms (the charter wrote only Delay(10)). In a window of 2100 ms
// (10 passes) that is exactly 20 wake-ups for any phase, against 2100 steps of 1 ms.
// ---------------------------------------------------------------------------------------------------------------------

// Only ARR is written by firmware code that runs here (FFBHIDMain::setReportRate, FFBHIDMain.cpp:256). PSC = 83 is the value the
// fake's initTimer holds (platform/src/host_target.cpp:99, copied from MX_TIM13_Init, main.c:1203): the CubeMX init code is not run.
TEST_CASE_FIXTURE(IntegrationFixture, "F1-A4: the FFB timer TIM13 has ARR 1000 written by the firmware and PSC 83 from the fake's copy of MX_TIM13_Init, a period of 1001 timer counts of 1 us" * IT_DELETES_FFBHIDMAIN) {
	bringUp();
	CHECK(htim13.Instance->ARR == 1000);
	CHECK(htim13.Instance->PSC == 83);
	CHECK(ChainScheduler::periodCycles(htim13) == 84084); // 1001 * 84 timer clocks
	CHECK(HostTarget::timerRunning(&htim13));
	CHECK(HostTarget::timerUpdateIrqEnabled(&htim13));
}

TEST_CASE_FIXTURE(IntegrationFixture, "F1-A4: the FFB main thread runs exactly one cycle per TIM13 update" * IT_DELETES_FFBHIDMAIN) {
	bringUp();
	setSpiFrameUs(0); // steady state needs no time per frame; frames that take time would move the clock and the timers
	const ChainScheduler::Stats a = sched.stats();
	const size_t reads0 = chip1.readsOf(0x6B);
	sched.advanceMs(100);
	const ChainScheduler::Stats b = sched.stats();
	const uint32_t updates = b.ffbUpdates - a.ffbUpdates;
	const uint32_t cycles = b.ffbCycles - a.ffbCycles;
	CHECK(cycles == updates);
	CHECK((updates == 99 || updates == 100)); // 99.90 updates per 100 ms, see the derivation above
	CHECK(chip1.readsOf(0x6B) - reads0 == cycles); // one encoder frame per cycle on the chip: the cycles really ran updateControl
}

// IB 1. Intended: FFBHIDMain tells updateSamplerate 1000 Hz at the default rate (getCurFFBFreq = basefreq / basediv), so one FFB
// cycle per simulated millisecond: 1001 ms give 1001 cycles. The timer delivers 1000 (derivation above). A broken bring-up would
// also fail the case, so the chain is REQUIREd to work first (cycles equal updates and encoder frames, about 1000 of them); the
// first CHECK after that is the defect.
TEST_CASE_FIXTURE(IntegrationFixture, "F1-A4: 1001 simulated ms hold 1001 FFB cycles at the default rate, one per millisecond"
	* KNOWN_ISSUE("(IB 1) FFB timer period is ARR+1 = 1001 us at the default rate (FFBHIDMain.cpp:256, RM0090 up-counter period ARR+1)") * IT_DELETES_FFBHIDMAIN) {
	bringUp();
	setSpiFrameUs(0);
	const ChainScheduler::Stats a = sched.stats();
	const size_t reads0 = chip1.readsOf(0x6B);
	sched.advanceMs(1001);
	const ChainScheduler::Stats b = sched.stats();
	const uint32_t updates = b.ffbUpdates - a.ffbUpdates;
	const uint32_t cycles = b.ffbCycles - a.ffbCycles;
	REQUIRE(cycles == updates);
	REQUIRE(updates >= 1000);
	REQUIRE(chip1.readsOf(0x6B) - reads0 == cycles);
	REQUIRE_CHAIN_MODELLED();
	CHECK(cycles == 1001);
}

// TMC wake-ups in Running: the activations of the TMC task handle only (ChainScheduler step (d)), with their times. One pass of
// TMC4671::Run is Delay(200) (TMC4671.cpp:577, Running case) and Delay(10) (TMC4671.cpp:656, loop end), so the wake-ups are
// 200 ms and 10 ms apart, alternately: in 2100 ms (10 passes of 210 ms) exactly 20 wake-ups, any two consecutive gaps add up to 210 ms.
// A delay of 190 ms instead of 200 would give gaps of 190 / 10 and fail.
TEST_CASE_FIXTURE(IntegrationFixture, "F1-A4: the TMC thread wakes in Running 200 ms and 10 ms apart alternately, 20 times in 2100 ms, not once per millisecond" * IT_DELETES_FFBHIDMAIN) {
	bringUp();
	setSpiFrameUs(0);
	REQUIRE(tmc()->getState() == TMC_ControlState::Running);
	const size_t first = sched.stats().tmcWakeUs.size();
	const uint32_t a = sched.stats().tmcActivations;
	sched.advanceMs(2100);
	const ChainScheduler::Stats &b = sched.stats();
	CHECK(b.tmcActivations - a == 20);
	REQUIRE(b.tmcWakeUs.size() == first + 20);
	std::vector<uint64_t> gapsMs;
	for (size_t i = first + 1; i < b.tmcWakeUs.size(); i++) {
		gapsMs.push_back((b.tmcWakeUs[i] - b.tmcWakeUs[i - 1]) / 1000);
	}
	for (size_t i = 0; i < gapsMs.size(); i++) {
		INFO("gap " << i << " = " << gapsMs[i] << " ms");
		CHECK((gapsMs[i] == 10 || gapsMs[i] == 200));
		if (i + 1 < gapsMs.size()) {
			CHECK(gapsMs[i] + gapsMs[i + 1] == 210);
		}
	}
	CHECK(tmc()->getState() == TMC_ControlState::Running);
}

// IB 2. The external encoder timer TIM6 is set up by TMC4671::setUpExtEncTimer (TMC4671.cpp:3153-3158) when the encoder source
// is set to ext with an encoder on the axis: PSC = ((SystemCoreClock / 2) / 1000000) + 1 = 85 ("1us ticks") and ARR = TIM_TMC_ARR =
// 250 (target_constants.h:96, "4khz"). Intended: 4 kHz = 250 us = 21 000 timer clocks of 84 MHz. The registers give
// (250 + 1) * (85 + 1) = 21 586 clocks = 257 us = 3891 Hz. Reached through public commands only: "tmc.0.combineEncoder=1" makes the
// TMC use an axis encoder (hasIntegratedEncoder false), "axis.0.enctype=2" creates the local encoder (EncoderSources.cpp:20) and
// passes it to the driver, "tmc.0.encsrc=5" (EncoderType_TMC::ext) calls setEncoderType(ext) -> setUpExtEncTimer.
TEST_CASE_FIXTURE(IntegrationFixture, "F1-A4: the TMC external encoder timer TIM6 runs at 4 kHz, a period of 250 us"
	* KNOWN_ISSUE("(IB 2) TMC external encoder timer runs at 3891 Hz instead of 4 kHz: PSC +1 instead of -1 and ARR+1 counts (TMC4671.cpp:3156-3158)") * IT_DELETES_FFBHIDMAIN) {
	reachShutdown();
	cdcCommand("tmc.0.combineEncoder=1");
	cdcCommand("axis.0.enctype=2");
	cdcCommand("tmc.0.encsrc=5");
	REQUIRE(HostTarget::timerRunning(&htim6));
	REQUIRE(HostTarget::timerUpdateIrqEnabled(&htim6));
	REQUIRE(htim6.Instance->ARR == 250); // TIM_TMC_ARR, written by the firmware
	REQUIRE(htim6.Instance->PSC > 0);	// written by the firmware (setUpExtEncTimer)
	REQUIRE_CHAIN_MODELLED();
	CHECK(ChainScheduler::periodCycles(htim6) == 21000); // 250 us at 84 MHz
}

// TIM6 scheduler rule. The registers a hand start uses are the firmware's (ARR 250, PSC 85, TMC4671.cpp:3153-3158): the period is
// (250 + 1) * (85 + 1) = 21 586 timer clocks of 84 MHz = 257 us, the value DERIVED from the firmware's registers; the intended
// period is 250 us (see the IB 2 case above). The case tests the scheduler's sub-millisecond rule only: 1000 steps of 1 ms =
// 84 000 000 clocks / 21 586 = 3891.4 updates, from phase 0: 3891 (with 0 us per frame the first step is exact, see ChainScheduler.h).
// No firmware object listens to htim6 here, so only the count is checked.
TEST_CASE_FIXTURE(IntegrationFixture, "F1-A4: a timer period below one millisecond (TIM6, ARR 250, PSC 85) raises one update per elapsed period, several in a step") {
	htim6.Instance->ARR = 250;
	htim6.Instance->PSC = 85;
	REQUIRE(HAL_TIM_Base_Start_IT(&htim6) == HAL_OK);
	CHECK(ChainScheduler::periodCycles(htim6) == 21586);
	sched.reset();
	sched.advanceMs(1);
	CHECK(sched.stats().tim6Updates == 3); // 84 000 clocks / 21 586 = 3.89: three updates in the first millisecond
	sched.advanceMs(999);
	CHECK(sched.stats().tim6Updates == 3891);
	CHECK(HAL_TIM_Base_Stop_IT(&htim6) == HAL_OK);
	sched.advanceMs(10);
	CHECK(sched.stats().tim6Updates == 3891); // a stopped timer raises nothing
}

// IB 3. Several replies are queued for the HID command interface while the HID IN endpoint is not read (the host does not poll),
// then the host reads. Intended: every queued reply arrives intact and in order. Oracle: the values the same commands answered over
// CDC (the reply text "[cmd|value]"); a HID command report is 25 bytes, byte 0 = 0xA1, data at bytes 8..15 (HID_CMD_Data_t,
// HidCommandInterface.h:20-30). The chain is REQUIREd to work first (every CDC reply arrived). The firmware keeps a reference
// into outBuffer across Delay(1) (HidCommandInterface.cpp:39-46) while queueReplyValues appends and may reallocate it (:159).
TEST_CASE_FIXTURE(IntegrationFixture, "F1-A1: replies queued for the HID command interface while its IN endpoint is not read all arrive intact and in order"
	* KNOWN_ISSUE_UB("(IB 3) HID_CommandInterface::Run iterates outBuffer across Delay(1) while queueReplyValues appends and reallocates it: use after free (HidCommandInterface.cpp:39-46, :159)") * IT_DELETES_FFBHIDMAIN) {
	reachShutdown();
	drainHid = false;
	const std::vector<std::string> commands = {"axis.0.drvtype?", "tmc.0.mtype?", "tmc.0.poles?", "axis.0.drvtype?", "tmc.0.poles?", "tmc.0.mtype?"};
	std::vector<int> expected;
	for (const std::string &c : commands) {
		const std::string r = cdcCommand(c);
		const size_t bar = r.rfind('|');
		REQUIRE(bar != std::string::npos);
		expected.push_back(std::atoi(r.c_str() + bar + 1));
	}
	REQUIRE_CHAIN_MODELLED();
	std::vector<int> received;
	for (uint32_t i = 0; i < 3000; i++) {
		HostDcd::Handshake h = HostDcd::Handshake::Nak;
		const std::vector<uint8_t> report = host.hidRead(&h);
		if (h != HostDcd::Handshake::Ack) {
			sched.advanceMs(1);
			if (received.size() >= expected.size()) {
				break;
			}
			continue;
		}
		if (report.size() == 25 && report[0] == 0xA1) {
			received.push_back(report[8]);
		}
	}
	std::string got, want;
	for (int v : received) {
		got += std::to_string(v) + " ";
	}
	for (int v : expected) {
		want += std::to_string(v) + " ";
	}
	INFO("received data bytes: " << got << "/ expected: " << want);
	CHECK(received == expected);
}

} // TEST_SUITE
