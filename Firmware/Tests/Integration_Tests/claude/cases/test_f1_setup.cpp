/*
 * F1 setup (WO I0-4 and I0-5): the smoke case of the integration executable, and the first requirement tests of the F1
 * charter group G-A (infrastructure self-checks) that need the IntegrationFixture: F1-A1 (the chain comes up) and F1-A4
 * (the scheduler's rates).
 */
#include "doctest.h"

#include "HostDcd.h"
#include "HostPlatform.h"
#include "HostTarget.h"
#include "HidReportDescriptor.h"
#include "IntegrationFixture.h"
#include "PidReportBuilder.h"
#include "TestHelpers.h"
#include "usbd.h"

#include <algorithm>
#include <cstdio>
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

// ---------------------------------------------------------------------------------------------------------------------
// F1-A2: the HID report descriptor the device sends over USB, parsed by the test's own parser, and PID reports built from it.
//
// Oracles: HID 1.11 (Tests/Code_Coverage/shared/specs/hid1_11.pdf) 7.1.1 Get_Descriptor (report descriptor, descriptor type
// 0x22, interface recipient, wLength = wDescriptorLength of the HID descriptor), 6.2.2 the item syntax (the parser), 5.8 values;
// USB_PID1_01.pdf (the PID specification file): Table 2 (usage IDs), 5.1-5.13 (what each report must contain), 7.1-7.3 (report
// summary: input 2 State; output Set Effect ... Effect Operation, Device Control, Pool Move; feature Pool, Device Gain), 9
// (device-managed effects: Create New Effect and PID Block Load are Feature reports). The report IDs are the firmware's choice
// (printed, never an oracle). Nothing here includes a firmware header: the descriptor bytes come over USB.
// ---------------------------------------------------------------------------------------------------------------------

namespace {

using hid::ReportType;

struct DescriptorFixture : IntegrationFixture {
	std::vector<uint8_t> bytes;
	hid::HidReportDescriptor desc;
	void load() {
		boot();
		enumerateUsb();
		bytes = readHidReportDescriptor();
		INFO("parse error: " << desc.error());
		REQUIRE(desc.parse(bytes));
	}
};

std::string usageName(uint32_t u) {
	const std::string n = pid::nameOf(u);
	if (!n.empty()) {
		return n;
	}
	char buf[40];
	std::snprintf(buf, sizeof(buf), "0x%04X:0x%04X", static_cast<unsigned>(u >> 16), static_cast<unsigned>(u & 0xFFFF));
	return buf;
}

// The PID usages that name a report (a Logical collection around the fields of a report) with the type the PID 1.01 text or
// example gives it: Table 2 names; 7.1 / 7.2 / 7.3 and section 9. `required` = 0 optional, 1 mandatory (see the cases).
struct ReportSpec {
	uint32_t usage;
	const char *section;
};

// Every report collection of Table 2 that applies to a force-feedback device of this kind (custom force reports 5.4 included, so the
// "not declared" list is complete).
const ReportSpec kAllReports[] = {
	{pid::use::SetEffectReport, "5.1"},		  {pid::use::SetEnvelopeReport, "5.2"},		 {pid::use::SetConditionReport, "5.3"},
	{pid::use::SetPeriodicReport, "5.5"},	  {pid::use::SetConstantForceReport, "5.6"}, {pid::use::SetRampForceReport, "5.7"},
	{pid::use::EffectOperationReport, "5.9"}, {pid::use::DeviceGainReport, "5.10"},		 {pid::use::PidPoolReport, "5.11.1"},
	{pid::use::PidPoolMoveReport, "5.11.1"},  {pid::use::PidBlockLoadReport, "5.11.2"},	 {pid::use::PidBlockFreeReport, "5.11.2"},
	{pid::use::PidStateReport, "5.12"},		  {pid::use::PidDeviceControlReport, "5.13"}, {pid::use::CreateNewEffectReport, "5.11.2"},
	{pid::use::CustomForceDataReport, "5.4"}, {pid::use::DownloadForceSample, "5.4"},	 {pid::use::SetCustomForceReport, "5.4"},
};

// All reports of any type that sit in a collection with this usage
std::vector<const hid::Report *> declared(const hid::HidReportDescriptor &d, uint32_t reportUsage) {
	std::vector<const hid::Report *> all;
	for (ReportType t : {ReportType::Input, ReportType::Output, ReportType::Feature}) {
		for (const hid::Report *r : d.findReportsByUsage(t, reportUsage)) {
			all.push_back(r);
		}
	}
	return all;
}

// A PID report that is declared exactly once, of type `type`
const hid::Report *requireReport(const hid::HidReportDescriptor &d, ReportType type, uint32_t usage, const char *section) {
	INFO("PID 1.01 section " << std::string(section) << ": " << usageName(usage) << " as " << std::string(hid::reportTypeName(type)) << " report");
	const std::vector<const hid::Report *> r = d.findReportsByUsage(type, usage);
	CHECK(r.size() == 1);
	return r.size() == 1 ? r[0] : nullptr;
}

// A field with that usage exists in that report (optionally inside the collections of `subPath`)
void requireField(const hid::HidReportDescriptor &d, ReportType type, uint32_t reportUsage, uint32_t fieldUsage, const char *section,
				  const std::vector<uint32_t> &subPath = {}) {
	INFO("PID 1.01 section " << std::string(section) << ": " << usageName(reportUsage) << " must contain " << usageName(fieldUsage));
	CHECK(d.findField(type, reportUsage, fieldUsage, subPath) != nullptr);
}

// A data field exists inside the collection `inner` of the report `reportUsage` (the usages of the field are of any page: only the
// presence of the collection with a field is asserted)
void requireFieldIn(const hid::HidReportDescriptor &d, ReportType type, uint32_t reportUsage, uint32_t inner, const char *section) {
	INFO("PID 1.01 section " << std::string(section) << ": " << usageName(reportUsage) << " must contain a " << usageName(inner) << " collection with fields");
	bool found = false;
	for (const hid::Report *r : d.findReportsByUsage(type, reportUsage)) {
		for (size_t fi : r->fields) {
			const hid::Field &f = d.fields()[fi];
			for (const hid::CollectionRef &c : f.path) {
				found = found || (c.usage == inner && !f.isConstant() && f.count > 0);
			}
		}
	}
	CHECK(found);
}

// Table 3 (p. 11): the parameter block reports each effect type uses
bool usesEnvelope(uint32_t et) {
	return et == pid::use::EtConstantForce || et == pid::use::EtRamp || et == pid::use::EtSquare || et == pid::use::EtSine ||
		   et == pid::use::EtTriangle || et == pid::use::EtSawtoothUp || et == pid::use::EtSawtoothDown;
}
bool isPeriodic(uint32_t et) {
	return et == pid::use::EtSquare || et == pid::use::EtSine || et == pid::use::EtTriangle || et == pid::use::EtSawtoothUp || et == pid::use::EtSawtoothDown;
}
bool isCondition(uint32_t et) { return et == pid::use::EtSpring || et == pid::use::EtDamper || et == pid::use::EtInertia || et == pid::use::EtFriction; }

// The Effect Type selectors the Create New Effect / Set Effect Report declare (array fields inside the Effect Type collection)
std::vector<uint32_t> declaredEffectTypes(const hid::HidReportDescriptor &d, ReportType type, uint32_t reportUsage) {
	std::vector<uint32_t> out;
	for (const hid::Report *r : d.findReportsByUsage(type, reportUsage)) {
		for (size_t fi : r->fields) {
			const hid::Field &f = d.fields()[fi];
			bool inEt = false;
			for (const hid::CollectionRef &c : f.path) {
				inEt = inEt || c.usage == pid::use::EffectType;
			}
			if (inEt && f.isArray()) {
				out.insert(out.end(), f.usages.begin(), f.usages.end());
			}
		}
	}
	return out;
}

} // namespace

TEST_CASE_FIXTURE(DescriptorFixture, "F1-A2: the HID report descriptor read over USB (GET_DESCRIPTOR type 0x22, wLength = wDescriptorLength) parses without error" * IT_DELETES_FFBHIDMAIN) {
	boot();
	enumerateUsb();
	const uint16_t wLength = hidReportDescriptorLength();
	CHECK(wLength > 0);
	const std::vector<uint8_t> got = readHidReportDescriptor(); // REQUIREs ok and exactly wLength bytes
	CHECK(got.size() == wLength);
	hid::HidReportDescriptor d;
	const bool ok = d.parse(got);
	INFO("parse error: " << d.error());
	REQUIRE(ok);
	CHECK(d.longItems() == 0);
	CHECK(d.usesReportIds()); // PID reports are told apart by their report ID (PID 1.01 section 7)
	CHECK(d.reports().size() > 0);
	// every (type, id) once: the parser merges, so two reports with the same type and ID would show as one report
	for (const hid::Report &r : d.reports()) {
		CHECK(r.reportId != 0);
		CHECK(r.byteLength() >= 2);
	}
	std::string list;
	for (const hid::Report &r : d.reports()) {
		list += std::string(hid::reportTypeName(r.type)) + " id " + std::to_string(r.reportId) + " (" + std::to_string(r.byteLength()) + " bytes, " +
				std::to_string(r.fields.size()) + " fields); ";
	}
	MESSAGE("F1-A2: report descriptor of " << got.size() << " bytes: " << list);
}

namespace {

// The effect types the device offers (Effect Type named array of Create New Effect and Set Effect, PID 1.01 5.1.1) and which
// type-specific parameter blocks Table 3 (p. 11) assigns to them
struct EffectTypes {
	std::vector<uint32_t> set, create, all;
	bool envelope = false, periodic = false, condition = false, constant = false, ramp = false;
};

EffectTypes collectEffectTypes(const hid::HidReportDescriptor &d) {
	EffectTypes e;
	e.create = declaredEffectTypes(d, ReportType::Feature, pid::use::CreateNewEffectReport);
	e.set = declaredEffectTypes(d, ReportType::Output, pid::use::SetEffectReport);
	e.all = e.set;
	for (uint32_t t : e.create) {
		if (std::find(e.all.begin(), e.all.end(), t) == e.all.end()) {
			e.all.push_back(t);
		}
	}
	for (uint32_t t : e.all) {
		e.envelope = e.envelope || usesEnvelope(t);
		e.periodic = e.periodic || isPeriodic(t);
		e.condition = e.condition || isCondition(t);
		e.constant = e.constant || t == pid::use::EtConstantForce;
		e.ramp = e.ramp || t == pid::use::EtRamp;
	}
	return e;
}

} // namespace

// Oracle: only sentences of USB_PID1_01.pdf that say "must" (quoted):
//  - 5.1: "The minimal Effect parameter block must contain (Effect) Parameter Block Index, Effect Type, Duration, Sample Period, Gain,
//    Trigger Button, Trigger Repeat Interval, Axis Direction, and Type Specific Block Handle values." (the Direction collection and
//    the Type Specific Block Offset collection stand for the last two); 5.1: "Effect Block Index varies from 1 to the maximum number
//    of effects": the logical minimum is 1;
//  - 5.3 / 5.5 / 5.6 / 5.7: "This collection must contain ..." for the Condition, Periodic, Constant Force (Magnitude) and Ramp
//    Force (Ramp Start, Ramp End) reports, which Table 3 (p. 11, Type Specific Block Offset 1) requires for the effect types that
//    use them; 5.2 (Envelope) is checked only if the descriptor declares the report: 5.1.1.1 rule 4 calls the Envelope Block optional
//    and 5.2 says "not all effect types use envelopes" (reported below, never required);
//  - 5.9: "This collection must contain Effect Block Index, Effect Operation and Loop Count usages";
//  - 5.11 "A vendor can choose whether to let the system driver manage the parameter block pool or manage it themselves": one of the
//    two models; 5.11.2: the host sends a Create New Effect Report "with the requested effect type", the device answers with a PID Block
//    Load Report "containing the newly assigned Effect Block Index", Block Load Status and "RAM Pool Available"; the PID Block Free
//    Report "will contain the Block Handle to free" ("Type Specific Block Handle and Effect Block Index will be used
//    interchangeably"); 5.11.1: the PID Pool Report holds the RAM Pool Size.
// Section 7 of the PID document is an example summary and is not cited as a rule.
TEST_CASE_FIXTURE(DescriptorFixture, "F1-A2: the descriptor declares what the 'must' sentences of PID 1.01 sections 5.1, 5.3, 5.5-5.7, 5.9 and 5.11 require" * IT_DELETES_FFBHIDMAIN) {
	load();
	const hid::HidReportDescriptor &d = desc;
	const EffectTypes ets = collectEffectTypes(d);
	CHECK(!ets.set.empty()); // 5.1: Effect Type
	for (uint32_t t : ets.all) {
		INFO("effect type " << usageName(t));
		CHECK(pid::nameOf(t).rfind("ET ", 0) == 0); // an ET usage of Table 2
	}

	// 5.1
	requireReport(d, ReportType::Output, pid::use::SetEffectReport, "5.1");
	for (uint32_t f : {pid::use::EffectBlockIndex, pid::use::Duration, pid::use::SamplePeriod, pid::use::Gain, pid::use::TriggerButton, pid::use::TriggerRepeatInterval}) {
		requireField(d, ReportType::Output, pid::use::SetEffectReport, f, "5.1");
	}
	requireFieldIn(d, ReportType::Output, pid::use::SetEffectReport, pid::use::Direction, "5.1 (Axis Direction)");
	requireFieldIn(d, ReportType::Output, pid::use::SetEffectReport, pid::use::TypeSpecificBlockOffset, "5.1 (Type Specific Block Handle)");
	{
		const hid::Field *idx = d.findField(ReportType::Output, pid::use::SetEffectReport, pid::use::EffectBlockIndex);
		REQUIRE(idx != nullptr);
		CHECK(idx->logicalMin == 1);
		MESSAGE("F1-A2: Effect Block Index logical range " << idx->logicalMin << ".." << idx->logicalMax << " in " << idx->bitSize << " bits");
	}
	// 5.3, 5.5, 5.6, 5.7 with Table 3
	if (ets.condition) {
		requireReport(d, ReportType::Output, pid::use::SetConditionReport, "5.3 / Table 3");
		for (uint32_t f : {pid::use::EffectBlockIndex, pid::use::CpOffset, pid::use::PositiveCoefficient, pid::use::NegativeCoefficient,
						   pid::use::PositiveSaturation, pid::use::NegativeSaturation, pid::use::DeadBand}) {
			requireField(d, ReportType::Output, pid::use::SetConditionReport, f, "5.3");
		}
	}
	if (ets.periodic) {
		requireReport(d, ReportType::Output, pid::use::SetPeriodicReport, "5.5 / Table 3");
		for (uint32_t f : {pid::use::EffectBlockIndex, pid::use::Magnitude, pid::use::Offset, pid::use::Phase, pid::use::Period}) {
			requireField(d, ReportType::Output, pid::use::SetPeriodicReport, f, "5.5");
		}
	}
	if (ets.constant) {
		requireReport(d, ReportType::Output, pid::use::SetConstantForceReport, "5.6 / Table 3");
		requireField(d, ReportType::Output, pid::use::SetConstantForceReport, pid::use::EffectBlockIndex, "5.6");
		requireField(d, ReportType::Output, pid::use::SetConstantForceReport, pid::use::Magnitude, "5.6");
	}
	if (ets.ramp) {
		requireReport(d, ReportType::Output, pid::use::SetRampForceReport, "5.7 / Table 3");
		for (uint32_t f : {pid::use::EffectBlockIndex, pid::use::RampStart, pid::use::RampEnd}) {
			requireField(d, ReportType::Output, pid::use::SetRampForceReport, f, "5.7");
		}
	}
	// 5.2: the Envelope is optional (5.1.1.1 rule 4): reported only; if declared its collection must contain the five usages
	const bool envDeclared = !declared(d, pid::use::SetEnvelopeReport).empty();
	MESSAGE("F1-A2: Set Envelope Report declared: " << (envDeclared ? 1 : 0) << " (optional, PID 1.01 5.1.1.1 rule 4); effect types that use an envelope declared: " << (ets.envelope ? 1 : 0));
	if (envDeclared) {
		for (uint32_t f : {pid::use::EffectBlockIndex, pid::use::AttackLevel, pid::use::AttackTime, pid::use::FadeLevel, pid::use::FadeTime}) {
			requireField(d, ReportType::Output, pid::use::SetEnvelopeReport, f, "5.2");
		}
	}
	// 5.9
	requireReport(d, ReportType::Output, pid::use::EffectOperationReport, "5.9");
	for (uint32_t f : {pid::use::EffectBlockIndex, pid::use::LoopCount}) {
		requireField(d, ReportType::Output, pid::use::EffectOperationReport, f, "5.9");
	}
	requireFieldIn(d, ReportType::Output, pid::use::EffectOperationReport, pid::use::EffectOperation, "5.9 (Effect Operation)");

	// 5.11
	const bool deviceManaged = !declared(d, pid::use::CreateNewEffectReport).empty();
	const bool driverManaged = !declared(d, pid::use::PidPoolReport).empty();
	CHECK((deviceManaged || driverManaged));
	MESSAGE("F1-A2: parameter block management: device managed (Create New Effect declared) " << (deviceManaged ? 1 : 0) << ", PID Pool Report declared " << (driverManaged ? 1 : 0));
	if (deviceManaged) {
		requireReport(d, ReportType::Feature, pid::use::CreateNewEffectReport, "5.11.2");
		requireFieldIn(d, ReportType::Feature, pid::use::CreateNewEffectReport, pid::use::EffectType, "5.11.2 (requested effect type)");
		requireReport(d, ReportType::Feature, pid::use::PidBlockLoadReport, "5.11.2");
		requireField(d, ReportType::Feature, pid::use::PidBlockLoadReport, pid::use::EffectBlockIndex, "5.11.2");
		requireFieldIn(d, ReportType::Feature, pid::use::PidBlockLoadReport, pid::use::BlockLoadStatus, "5.11.2");
		requireField(d, ReportType::Feature, pid::use::PidBlockLoadReport, pid::use::RamPoolAvailable, "5.11.2");
		const std::vector<const hid::Report *> bf = declared(d, pid::use::PidBlockFreeReport);
		CHECK(bf.size() == 1);
		CHECK((d.findField(ReportType::Output, pid::use::PidBlockFreeReport, pid::use::EffectBlockIndex) != nullptr ||
			   d.findField(ReportType::Feature, pid::use::PidBlockFreeReport, pid::use::EffectBlockIndex) != nullptr ||
			   d.findField(ReportType::Output, pid::use::PidBlockFreeReport, pid::use::BlockHandle) != nullptr ||
			   d.findField(ReportType::Feature, pid::use::PidBlockFreeReport, pid::use::BlockHandle) != nullptr));
	}
	if (driverManaged) {
		requireReport(d, ReportType::Feature, pid::use::PidPoolReport, "5.11.1");
		requireField(d, ReportType::Feature, pid::use::PidPoolReport, pid::use::RamPoolSize, "5.11.1");
	}
}

// Facts about THIS device's descriptor that the later integration tests rely on (oracle DESC, the descriptor the device sends); none
// of them is a requirement of PID 1.01 (the sentences have no "must": 5.12 and 5.13 list the usages, 5.1.1 and 5.9 describe them):
// the Direction Enable flag and Generic Desktop X / Y in Axes Enable, the three Effect Operation selectors, the six Device Control
// selectors, the PID State report with Effect Playing, the three Block Load status selectors, the Type Specific Block Offset
// ordinals, and equal effect type sets in Create New Effect and Set Effect.
TEST_CASE_FIXTURE(DescriptorFixture, "F1-A2: DESC: the descriptor offers the flags, selectors and reports the later HID tests use" * IT_DELETES_FFBHIDMAIN) {
	load();
	const hid::HidReportDescriptor &d = desc;
	const EffectTypes ets = collectEffectTypes(d);
	std::string etText;
	for (uint32_t t : ets.all) {
		etText += usageName(t) + ", ";
	}
	MESSAGE("F1-A2: effect types declared: " << etText);
	if (!ets.create.empty()) {
		std::vector<uint32_t> a = ets.create, b = ets.set;
		std::sort(a.begin(), a.end());
		std::sort(b.begin(), b.end());
		CHECK(a == b);
	}
	requireField(d, ReportType::Output, pid::use::SetEffectReport, pid::use::DirectionEnable, "DESC");
	requireField(d, ReportType::Output, pid::use::SetEffectReport, hid::usage(0x01, 0x30), "DESC (Axes Enable X)", {pid::use::AxesEnable});
	requireField(d, ReportType::Output, pid::use::SetEffectReport, hid::usage(0x01, 0x31), "DESC (Axes Enable Y)", {pid::use::AxesEnable});
	requireField(d, ReportType::Output, pid::use::SetEffectReport, hid::usage(0x0A, 1), "DESC (Type Specific Block Offset)", {pid::use::TypeSpecificBlockOffset});
	for (uint32_t f : {pid::use::OpEffectStart, pid::use::OpEffectStartSolo, pid::use::OpEffectStop}) {
		requireField(d, ReportType::Output, pid::use::EffectOperationReport, f, "DESC");
	}
	requireReport(d, ReportType::Output, pid::use::PidDeviceControlReport, "DESC");
	for (uint32_t f : {pid::use::DcEnableActuators, pid::use::DcDisableActuators, pid::use::DcStopAllEffects, pid::use::DcDeviceReset, pid::use::DcDevicePause,
					   pid::use::DcDeviceContinue}) {
		requireField(d, ReportType::Output, pid::use::PidDeviceControlReport, f, "DESC");
	}
	requireReport(d, ReportType::Input, pid::use::PidStateReport, "DESC");
	requireField(d, ReportType::Input, pid::use::PidStateReport, pid::use::EffectPlaying, "DESC");
	for (uint32_t f : {pid::use::BlockLoadSuccess, pid::use::BlockLoadFull, pid::use::BlockLoadError}) {
		requireField(d, ReportType::Feature, pid::use::PidBlockLoadReport, f, "DESC");
	}
}

// The full list with the report ID, type, length and field count of what the descriptor declares, and what PID 1.01 defines that it
// does not declare (optional or not, see the previous case for the mandatory part): the evidence of this WO (printed with MESSAGE).
TEST_CASE_FIXTURE(DescriptorFixture, "F1-A2: every PID report of PID 1.01 Table 2 is either declared once with a report ID, type, length and field count, or listed as not declared" * IT_DELETES_FFBHIDMAIN) {
	load();
	std::string have, miss;
	for (const ReportSpec &s : kAllReports) {
		const std::vector<const hid::Report *> r = declared(desc, s.usage);
		if (r.empty()) {
			miss += usageName(s.usage) + " (section " + s.section + "); ";
			continue;
		}
		CHECK(r.size() == 1); // a PID report collection is declared once, in one report type
		for (const hid::Report *rep : r) {
			have += usageName(s.usage) + ": " + hid::reportTypeName(rep->type) + " id " + std::to_string(rep->reportId) + " (0x" + [&] {
				char b[8];
				std::snprintf(b, sizeof(b), "%02X", rep->reportId);
				return std::string(b);
			}() + "), " + std::to_string(rep->byteLength()) + " bytes, " + std::to_string(rep->fields.size()) + " fields; ";
		}
	}
	MESSAGE("F1-A2: PID reports declared: " << have);
	MESSAGE("F1-A2: PID 1.01 reports not declared: " << miss);
	// the report IDs of the declared PID reports differ per type (HID 1.11 6.2.2.7: a Report ID names one report; the parser refuses
	// a descriptor in which two report collections share an ID, so this compares the declared reports themselves)
	std::vector<const hid::Report *> pidReports;
	for (const ReportSpec &s : kAllReports) {
		for (const hid::Report *r : declared(desc, s.usage)) {
			pidReports.push_back(r);
		}
	}
	for (size_t i = 0; i < pidReports.size(); i++) {
		for (size_t j = i + 1; j < pidReports.size(); j++) {
			INFO("PID reports " << i << " and " << j);
			CHECK(pidReports[i] != pidReports[j]);
			if (pidReports[i]->type == pidReports[j]->type) {
				CHECK(pidReports[i]->reportId != pidReports[j]->reportId);
			}
		}
	}
}

// Consistency of the descriptor itself (HID 1.11 5.8): the logical range of a data field fits its report size, so a host builder
// can place every value of the range. The one exception, the PID Device Control field (Report Size 1, Count 8, logical 1..6),
// has the same shape as the descriptor example of PID 1.01 section 6 (Report Size 1, Report Count 8, Logical 1..6); the case does
// not take a position on it and the shape is reported (see the case after the next one).
TEST_CASE_FIXTURE(DescriptorFixture, "F1-A2: the logical range of every data field of the PID reports fits the field's report size, except the PID Device Control field of PID 1.01 section 6" * IT_DELETES_FFBHIDMAIN) {
	load();
	for (const ReportSpec &s : kAllReports) {
		for (const hid::Report *r : declared(desc, s.usage)) {
			for (size_t fi : r->fields) {
				const hid::Field &f = desc.fields()[fi];
				if (f.isConstant()) {
					continue;
				}
				bool deviceControl = false;
				for (const hid::CollectionRef &c : f.path) {
					deviceControl = deviceControl || c.usage == pid::use::PidDeviceControl;
				}
				if (deviceControl) {
					continue;
				}
				INFO(usageName(s.usage) << " field at bit " << f.bitOffset << " size " << f.bitSize << " logical " << f.logicalMin << ".." << f.logicalMax);
				const int64_t lo = f.isSigned ? -(1ll << (f.bitSize - 1)) : 0;
				const int64_t hi = f.isSigned ? (1ll << (f.bitSize - 1)) - 1 : (1ll << f.bitSize) - 1;
				CHECK(f.logicalMin >= lo);
				CHECK(f.logicalMax <= hi);
			}
		}
	}
}

TEST_CASE_FIXTURE(DescriptorFixture, "F1-A2: the PID Device Control field has the shape of the PID 1.01 section 6 example (Report Size 1, Count 8, logical 1..6, six usages)" * IT_DELETES_FFBHIDMAIN) {
	load();
	const hid::Field *dc = desc.findField(ReportType::Output, pid::use::PidDeviceControlReport, pid::use::DcEnableActuators);
	REQUIRE(dc != nullptr);
	// PID 1.01 section 6 "PID Device Control Report Definition": LOGICAL_MINIMUM (1), LOGICAL_MAXIMUM (6), REPORT_SIZE (1), REPORT_COUNT (8),
	// OUTPUT (bytes 91 02) after the six DC usages; Figure 16 shows the report as ID + one byte.
	CHECK(dc->logicalMin == 1);
	CHECK(dc->logicalMax == 6);
	CHECK(dc->bitSize == 1);
	CHECK(dc->count == 8);
	CHECK(dc->usages.size() == 6);
	CHECK(dc->flags == 0x02);
	const hid::Report *r = desc.reportOf(*dc);
	REQUIRE(r != nullptr);
	CHECK(r->byteLength() == 2);
}

// INR 2 (question to the owner): the PID State input report has no Effect Block Index. PID 1.01 section 5.12 describes the state
// of "the effect identified by the Effect Handle", and the section 6 example (PID State Report Definition) and Figure 15 (p. 39)
// start the report with Effect Block Index (7 bits) and ROM Flag. The firmware descriptor has those lines commented out
// (usb_hid_ffb_desc.h HIDDESC_FFB_STATEREP, lines 162-169): report ID + one byte with Device Paused, Actuators Enabled, Safety
// Switch, Actuator Power and Effect Playing, then 3 constant bits. Documents today's shape; whether it is intended is open.
TEST_CASE_FIXTURE(DescriptorFixture, "characterization: [F1-A2] the PID State input report declares the status flags without an Effect Block Index" * IT_DELETES_FFBHIDMAIN) {
	load();
	REQUIRE(desc.findReportsByUsage(ReportType::Input, pid::use::PidStateReport).size() == 1);
	const hid::Report *st = desc.findReportsByUsage(ReportType::Input, pid::use::PidStateReport)[0];
	CHECK(st->byteLength() == 2);
	CHECK(st->bitLength == 8);
	CHECK(desc.findField(ReportType::Input, pid::use::PidStateReport, pid::use::EffectBlockIndex) == nullptr);
	CHECK(desc.findField(ReportType::Input, pid::use::PidStateReport, pid::use::RomFlag) == nullptr);
	for (uint32_t u : {pid::use::DevicePaused, pid::use::ActuatorsEnabled, pid::use::SafetySwitch, pid::use::ActuatorPower, pid::use::EffectPlaying}) {
		CHECK(desc.findField(ReportType::Input, pid::use::PidStateReport, u) != nullptr);
	}
}

// INR 3 (low): PID 1.01 section 5.1.1 says the Direction collection holds axes or vectors of the Generic Desktop page (X, Y, Z or
// Rx, Ry, Rz); the firmware declares one field of two 16 bit elements with the Ordinal usages (page 0x0A) instance 1 and 2,
// logical 0..36000 (usb_hid_2ffb_desc.c:200-216: Usage Ordinals 1 and 2 at 202-203, Logical Maximum 36000 at 209, Report Size 16 at
// 213, Report Count 2 at 215). Documents today's shape.
TEST_CASE_FIXTURE(DescriptorFixture, "characterization: [F1-A2] the Direction collection of Set Effect holds two ordinal-usage direction values 0..36000" * IT_DELETES_FFBHIDMAIN) {
	load();
	const hid::Field *dir = desc.findField(ReportType::Output, pid::use::SetEffectReport, hid::usage(0x0A, 1), {pid::use::Direction});
	REQUIRE(dir != nullptr);
	CHECK(dir->logicalMin == 0);
	CHECK(dir->logicalMax == 36000);
	CHECK(dir->bitSize == 16);
	CHECK(dir->count == 2);
	CHECK(dir->usageOf(0) == hid::usage(0x0A, 1));
	CHECK(dir->usageOf(1) == hid::usage(0x0A, 2));
	CHECK(desc.findField(ReportType::Output, pid::use::SetEffectReport, hid::usage(0x01, 0x30), {pid::use::Direction}) == nullptr);
}

// INR 4: PID 1.01 section 9 (the example sequence of creating a device-managed effect, step 5) says that if the device cannot create
// the effect it "will return a 0 Effect Block Index and the appropriate Block Load Status". The firmware's Block Load Effect Block
// Index has logical range 1..40 (usb_hid_ffb_desc.h HIDDESC_FFB_BLOCKLOADREP, lines 674-676: Logical Maximum MAX_EFFECTS, Logical
// Minimum 1), which does not include 0. Documents today's shape.
TEST_CASE_FIXTURE(DescriptorFixture, "characterization: [F1-A2] the Block Load Effect Block Index has logical range 1..40, which does not include the 0 PID 1.01 section 9 returns on failure" * IT_DELETES_FFBHIDMAIN) {
	load();
	const hid::Field *idx = desc.findField(ReportType::Feature, pid::use::PidBlockLoadReport, pid::use::EffectBlockIndex);
	REQUIRE(idx != nullptr);
	CHECK(idx->logicalMin == 1);
	CHECK(idx->logicalMax == 40);
	CHECK(idx->logicalMin > 0);
}

// Independent cross-check of the parser (the other F1-A2 cases compare the parser with itself): length and two field offsets of three
// reports derived by hand from the descriptor SOURCE bytes the device sends (read for this derivation only, nothing of it is
// included; the build uses MAX_AXIS 2 and AXIS2_FFB_HID_DESC, constants.h:13 / :43, MAX_EFFECTS 40, ffb_defs.h:21).
//
// Set Effect (usb_hid_ffb_desc.h:206-270 followed by usb_hid_2ffb_desc.c:182-229), bits after the report ID:
//   Effect Block Index Size 8 Count 1 (h:215-217)                    bits   0..7
//   Effect Type Size 8 Count 1 (h:236-238)        bits   8..15
//   Duration, Trigger Repeat Interval, Sample Period, Start Delay: Size 16 Count 4 (h:250-252)   bits 16..79
//   Gain Size 8 Count 1 (h:260-262)                                  bits  80..87
//   Trigger Button Size 8 Count 1 (h:268-270)                        bits  88..95
//   Axes Enable: X, Y, Size 1 Count 2 (c:189-191)       bits  96..97
//   Direction Enable: Size 1 Count 1 (c:194-196)        bit   98
//   constant padding: Size 1 Count 5 (c:197-198)        bits  99..103
//   Direction: Size 16 Count 2 (c:213,215)              bits 104..135
//   Type Specific Block Offset: Size 16 Count 2 (c:227-229)  bits 136..167
//   total 168 bits = 21 bytes, + the report ID byte = 22. Offsets asserted: Direction Enable 98, Type Specific Block Offset 136.
// Set Condition (usb_hid_2ffb_desc.c:271-326):
//   Effect Block Index Size 8 Count 1 (c:279-281) bits 0..7; Parameter Block Offset Size 4 (c:287-289) bits 8..11; Type Specific Block Offset
//   Size 2 Count 2 (c:294-296) bits 12..15; CP Offset Size 16 Count 1 (c:304-306) bits 16..31; Positive / Negative Coefficient
//   Size 16 Count 2 (c:311-312) bits 32..63; Positive / Negative Saturation Size 16 Count 2 (c:319-321) bits 64..95; Dead Band Size 16
//   Count 1 (c:324-326) bits 96..111. Total 112 bits = 14 bytes + 1 = 15. Offsets asserted: CP Offset 16, Dead Band 96.
// Block Load (usb_hid_ffb_desc.h:669-703): Effect Block Index Size 8 Count 1 (h:679-681) bits 0..7; Block Load Status Size 8 Count 1
//   (h:691-693) bits 8..15; RAM Pool Available Size 16 Count 1 (h:700-702) bits 16..31. Total 32 bits = 4 bytes + 1 = 5. Offsets
//   asserted: Block Load Status 8, RAM Pool Available 16.
TEST_CASE_FIXTURE(DescriptorFixture, "F1-A2: the parser's length and field offsets of Set Effect, Set Condition and Block Load equal the hand derivation from the descriptor source" * IT_DELETES_FFBHIDMAIN) {
	load();
	struct Case {
		ReportType type;
		uint32_t report;
		uint32_t bytes;
		uint32_t fieldA, fieldB;
		uint32_t offA, offB;
		uint32_t pathB; //!< collection of field B (0 = none): the ordinals also appear in Direction
	};
	const Case cases[] = {
		{ReportType::Output, pid::use::SetEffectReport, 22, pid::use::DirectionEnable, hid::usage(0x0A, 1), 98, 136, pid::use::TypeSpecificBlockOffset},
		{ReportType::Output, pid::use::SetConditionReport, 15, pid::use::CpOffset, pid::use::DeadBand, 16, 96, 0},
		{ReportType::Feature, pid::use::PidBlockLoadReport, 5, pid::use::BlockLoadSuccess, pid::use::RamPoolAvailable, 8, 16, 0},
	};
	for (const Case &c : cases) {
		INFO(usageName(c.report));
		const std::vector<const hid::Report *> r = desc.findReportsByUsage(c.type, c.report);
		REQUIRE(r.size() == 1);
		CHECK(r[0]->byteLength() == c.bytes);
		const hid::Field *a = desc.findField(c.type, c.report, c.fieldA);
		const hid::Field *b = desc.findField(c.type, c.report, c.fieldB, c.pathB ? std::vector<uint32_t>{c.pathB} : std::vector<uint32_t>{});
		REQUIRE(a != nullptr);
		REQUIRE(b != nullptr);
		CHECK(a->bitOffset == c.offA);
		CHECK(b->bitOffset == c.offB);
	}
}

// IB 4: PID 1.01 section 5.11.2 "RAM Pool Available DV" and the section 9 example "FEATURE (Data,Var,Abs)" (B1 02). The firmware
// declares B1 00 (Data, Array, Abs): usb_hid_ffb_desc.h ~line 702.
TEST_CASE_FIXTURE(DescriptorFixture, "F1-A2: RAM Pool Available in the PID Block Load report is a Variable field"
	* KNOWN_ISSUE("(IB 4) RAM Pool Available in the PID Block Load report is declared Array, PID 1.01 5.11.2 DV / section 9 example Var") * IT_DELETES_FFBHIDMAIN) {
	load();
	const hid::Field *f = desc.findField(ReportType::Feature, pid::use::PidBlockLoadReport, pid::use::RamPoolAvailable);
	REQUIRE(f != nullptr);
	REQUIRE_CHAIN_MODELLED();
	CHECK(f->isVariable());
}

TEST_CASE_FIXTURE(DescriptorFixture, "F1-A2: the builder makes every declared PID output and feature report with the descriptor's length and report ID" * IT_DELETES_FFBHIDMAIN) {
	load();
	pid::PidReportBuilder b(desc);
	size_t built = 0;
	for (const ReportSpec &s : kAllReports) {
		for (ReportType t : {ReportType::Output, ReportType::Feature}) {
			const hid::Report *r = b.find(t, s.usage);
			if (r == nullptr) {
				continue;
			}
			INFO(usageName(s.usage) << " (" << std::string(hid::reportTypeName(t)) << ")");
			pid::PidReport rep = b.make(t, s.usage);
			REQUIRE(rep.valid());
			CHECK(rep.bytes().size() == r->byteLength());
			CHECK(rep.length() == r->byteLength());
			CHECK(rep.bytes()[0] == r->reportId);
			CHECK(rep.reportId() == r->reportId);
			built++;
		}
	}
	CHECK(built >= 5);
	MESSAGE("F1-A2: built " << built << " PID output / feature reports");
}

// Every Variable field element with a usage that is unique in its report is set to its logical minimum and then to its logical
// maximum through the builder (by usage, strict), and decoded back from the bytes; the values must come back. This checks
// the builder against the layout of the real descriptor (overlaps, off-by-one offsets, sign).
TEST_CASE_FIXTURE(DescriptorFixture, "F1-A2: the minimum and the maximum of every uniquely named field of the declared PID reports survive building and decoding" * IT_DELETES_FFBHIDMAIN) {
	load();
	pid::PidReportBuilder b(desc);
	size_t checked = 0;
	for (const ReportSpec &s : kAllReports) {
		for (ReportType t : {ReportType::Input, ReportType::Output, ReportType::Feature}) {
			const hid::Report *r = b.find(t, s.usage);
			if (r == nullptr) {
				continue;
			}
			// usage -> number of Variable elements of the report that carry it
			std::vector<std::pair<uint32_t, int>> seen;
			auto bump = [&](uint32_t u) {
				for (auto &p : seen) {
					if (p.first == u) {
						p.second++;
						return;
					}
				}
				seen.push_back({u, 1});
			};
			for (size_t fi : r->fields) {
				const hid::Field &f = desc.fields()[fi];
				if (f.isConstant() || !f.isVariable()) {
					continue;
				}
				for (uint32_t e = 0; e < f.count; e++) {
					bump(f.usageOf(e));
				}
			}
			for (int pass = 0; pass < 2; pass++) {
				// the descriptor's own byte buffer for this pass: only builder-settable (fitting) fields
				std::vector<uint8_t> bytes(r->byteLength(), 0);
				bytes[0] = r->reportId;
				pid::PidReport rep(desc, r, usageName(s.usage), true);
				std::vector<std::pair<uint32_t, int64_t>> expect;
				for (size_t fi : r->fields) {
					const hid::Field &f = desc.fields()[fi];
					if (f.isConstant() || !f.isVariable()) {
						continue;
					}
					const int64_t lo = f.isSigned ? -(1ll << (f.bitSize - 1)) : 0;
					const int64_t hi = f.isSigned ? (1ll << (f.bitSize - 1)) - 1 : (1ll << f.bitSize) - 1;
					if (f.logicalMin < lo || f.logicalMax > hi || f.bitSize > 32) {
						continue; // the shape case reports these
					}
					for (uint32_t e = 0; e < f.count; e++) {
						const uint32_t u = f.usageOf(e);
						int n = 0;
						for (auto &p : seen) {
							if (p.first == u) {
								n = p.second;
							}
						}
						if (u == 0 || n != 1) {
							continue;
						}
						const int64_t v = pass == 0 ? f.logicalMin : f.logicalMax;
						rep.setElement(u, e, v);
						expect.push_back({u, v});
					}
				}
				const pid::DecodedReport dec = b.decode(*r, rep.bytes());
				INFO(usageName(s.usage) << " " << std::string(hid::reportTypeName(t)) << " pass " << pass);
				CHECK(dec.errors.empty());
				for (auto &x : expect) {
					int64_t got = 0;
					INFO("usage " << usageName(x.first));
					CHECK(dec.get(x.first, got));
					CHECK(got == x.second);
					checked++;
				}
			}
		}
	}
	CHECK(checked > 20);
	MESSAGE("F1-A2: " << checked << " field values built and decoded back");
}

TEST_CASE_FIXTURE(DescriptorFixture, "F1-A2: the descriptor declares the PID State input report with the Effect Playing flag and the builder decodes a zero state report" * IT_DELETES_FFBHIDMAIN) {
	load();
	pid::PidReportBuilder b(desc);
	const hid::Report *st = b.find(ReportType::Input, pid::use::PidStateReport);
	REQUIRE(st != nullptr);
	MESSAGE("F1-A2: PID State input report: id " << static_cast<unsigned>(st->reportId) << ", " << st->byteLength() << " bytes, " << st->fields.size() << " fields");
	std::vector<uint8_t> zero(st->byteLength(), 0);
	zero[0] = st->reportId;
	const pid::DecodedReport dec = b.decode(*st, zero);
	CHECK(dec.errors.empty());
	int64_t v = -1;
	CHECK(dec.get(pid::use::EffectPlaying, v));
	CHECK(v == 0);
	// the usages of the flags the descriptor offers (5.12): Device Paused, Actuators Enabled, Safety Switch, Actuator Override Switch, Actuator Power
	std::string flags;
	for (uint32_t u : {pid::use::DevicePaused, pid::use::ActuatorsEnabled, pid::use::SafetySwitch, pid::use::ActuatorOverrideSwitch, pid::use::ActuatorPower}) {
		flags += usageName(u) + (dec.get(u, v) ? std::string(" yes; ") : std::string(" no; "));
	}
	MESSAGE("F1-A2: PID State flags declared: " << flags << "Effect Block Index declared: " << std::string(dec.get(pid::use::EffectBlockIndex, v) ? "yes" : "no"));
}

} // TEST_SUITE
