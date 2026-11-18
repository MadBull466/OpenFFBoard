/*
 * IntegrationFixture.h
 *
 * Fixture of ffb_integration_tests (plan section 3.3). One struct, no inheritance of UsbFixture / TargetFixture /
 * TmcFixture (each resets the same globals and owns `mainclass` differently); it combines what those three do, once.
 * Use with TEST_CASE_FIXTURE(IntegrationFixture, "F1-...: ...").
 *
 * Construction (reset once per test case, in this order; HostUsb::reset is never called, the recording TinyUSB fake is not
 * linked): tud_deinit, HostDcd::reset, deleteCppmainMainclass (a main class of an earlier case), HostPlatform::reset,
 * HostRtos::reset, HostTarget::reset (with the TMC4 reset hooks), ErrorHandler::clearAll, the globals TargetFixture /
 * UsbFixture reset (mainclass, usb_device, global HID handler, persistent-storage start-up flag, CDC port), the flash map
 * (cleared by HostPlatform::reset). Then the board: SPI1 CS lines idle high (the CubeMX init leaves SS2 / SS3 low), the two
 * FakeTmc4671 chips (CS 0 = PA4 = TMC_1, CS 1 = PB0 = TMC_2) with the responder installed again (the target reset removes it),
 * the chip ADC raw words at mid scale (offset calibration succeeds), the analog reference of the fake ADC.
 *
 * bringUp() starts the firmware the way the board does and returns the simulated ms it took (Running reached):
 *  1. HostTarget::runCppmain(0): the real cppmain(): flash init, startADC, main class from ADR_CURRENT_CONFIG (default
 *     DEFAULTMAIN = 1 = FFBWheel, target_constants.h; BUTTON_A is low), FFBWheel::usbInit creates the USBdevice and starts the
 *     "USB" thread;
 *  2. supply 24 V on both rails (the fake ADC2), the threads made resumable (ChainScheduler::adoptThreads);
 *  3. the simulated host enumerates and configures the device (UsbHost::enumerate); CDC DTR/RTS set;
 *  4. the user's path to the driver: the CDC command "axis.0.drvtype=1" (MotorDriver.cpp: TMC_1 is class id 1);
 *     Axis::setDrvType -> setupDriver (thread "TMC" starts) and, because the USB device is connected, startMotor();
 *  5. the ChainScheduler runs until the TMC driver has initialised and stopped in Shutdown (a fresh flash: motor type NONE,
 *     encoder NONE), the host plugs the device in again (tud_mount_cb -> Axis::usbResume -> startMotor, the board's start
 *     path when the PC connects after the board finished its start-up), and runs until Running and motorReady().
 * Nothing private is called: no startMotor() / startup step by the test.
 *
 * Time per SPI frame: kSetupFrameUs (see the comment at the constant) during bring-up; setSpiFrameUs() changes it.
 *
 * Teardown (the case fails): HostDcd faults, UsbHost errors, unmodelled HAL calls, kernel faults, FakeTmc4671 violations not
 * declared with expectViolation(), kernel objects left over (tasks, queues, semaphores, timers, event groups, compared with
 * the start; the main class and its USBdevice are deleted first). KNOWN_ISSUE cases call REQUIRE_CHAIN_MODELLED() right before
 * the assertion that describes the defect (pattern of REQUIRE_ALL_HAL_MODELLED).
 *
 * The case deletes an FFBHIDMain main class (FFBWheel): undefined behaviour in the firmware (bug 24, reportHID is deleted
 * through a base class without a virtual destructor). Cases that call bringUp() use DELETES_FFBHIDMAIN (skipped in
 * sanitizer builds), like the coverage suite.
 */
#ifndef HOSTTEST_INTEGRATION_INTEGRATIONFIXTURE_H_
#define HOSTTEST_INTEGRATION_INTEGRATIONFIXTURE_H_

#include "ChainScheduler.h"
#include "../fakes/WheelPlant.h"
#include "PositionScript.h"

#include "../../Code_Coverage/fakes/FakeTmc4671.h" // fakes/ is not an include directory (fakes/Axis.h would shadow the real Axis.h)
#include "HostDcd.h"
#include "HostPlatform.h"
#include "HostRtos.h"
#include "HostRtosExt.h"
#include "HostTarget.h"
#include "UsbHost.h"

#include "FFBoardMain.h"
#include "TMC4671.h"
#include "USBdevice.h"

#include "doctest.h"

#include <cstdint>
#include <string>
#include <vector>

extern FFBoardMain *mainclass; // cppmain.cpp
extern USBdevice *usb_device;  // global_callbacks.cpp

/**
 * Marks cases that delete an FFBHIDMain main class (bug 24): skipped in sanitizer builds. Same rule as DELETES_FFBHIDMAIN of
 * AxisFixture.h (not included here: it pulls in the target fixtures).
 */
struct IntegrationNoDecorator {
	void fill(doctest::detail::TestCase &) const {}
	void fill(doctest::detail::TestSuite &) const {}
};
#ifdef HOSTTEST_SANITIZERS
#define IT_DELETES_FFBHIDMAIN doctest::skip()
#else
#define IT_DELETES_FFBHIDMAIN IntegrationNoDecorator()
#endif

/** Fails the test (REQUIRE) if the chain recorded a fault so far; use right before the assertion that describes a defect in a KNOWN_ISSUE case */
#define REQUIRE_CHAIN_MODELLED()                                                                                       \
	do {                                                                                                               \
		INFO("first fault: " << firstFault());                                                                         \
		REQUIRE(firstFault().empty());                                                                                 \
	} while (0)

struct IntegrationFixture {
	/**
	 * Simulated time added per SPI frame during bring-up, in microseconds. 0 does not work: the firmware measures the ADC
	 * offset in a busy loop on HAL_GetTick (TMC4671::calibrateAdcOffset, TMC4671.cpp:1396: while(HAL_GetTick() - tick < 500)
	 * with SPI frames inside) that only ends if time passes, and the fake SPI takes no time by itself: with 0 the TMC thread
	 * never returns (the case hangs; checked in WO I0-5). 1 us is the smallest value that works (the loop then takes 500 ms of
	 * simulated time; a real 40 bit frame is about 7.6 us, FakeTmc4671 header, which only makes the loop shorter in run time).
	 * Steady-state cases that count things per millisecond set it to 0 again (setSpiFrameUs(0)): frames that take time move the
	 * simulated clock and so the timers (the scheduler's time base is the elapsed simulated time).
	 */
	static constexpr uint32_t kSetupFrameUs = 1;

	FakeTmc4671 chip1{0}; //!< TMC_1, CS PA4
	FakeTmc4671 chip2{1}; //!< TMC_2, CS PB0
	ChainScheduler sched;
	WheelPlant wheel; //!< the default plant: set its parameters (WheelPlant::setParams), then attachPlant()
	UsbHost host{[this] { sched.pumpUsb(); }};

	size_t startMark = 0;	//!< Size of chip1.log() just before the USB re-mount of mountAndRun: frames after it belong to the motor start
	bool drainHid = true; //!< cdcCommand drains the HID IN endpoint each step (host behaviour, see drainHidIn; IB 3)
	uint32_t bringUpMs = 0; //!< Simulated ms from the drvtype command to Running (bringUp result)

	IntegrationFixture();
	virtual ~IntegrationFixture();

	/** The reset sequence of the file comment (static: also used by the destructor) */
	static void resetAll();

	// --- bring-up (see the file comment) ---
	/** Steps 1 to 5. REQUIREs every step; @return simulated ms from the drvtype command until Running and motorReady() */
	uint32_t bringUp(uint32_t budgetMs = 20000);
	/** Steps 1 to 4 and the wait for Shutdown (the bring-up up to the point where the PC connects afterwards) */
	void reachShutdown(uint32_t budgetMs = 20000);
	/** Step 5: the host plugs the device in again, runs until Running and motorReady(). Sets bringUpMs and startMark */
	void mountAndRun(uint32_t budgetMs = 20000);
	/** Step 1 and 2 */
	void boot();
	/** Step 3 and CDC control lines */
	void enumerateUsb();
	/** The host unplugs and plugs the device in again and enumerates (tud_umount_cb, tud_mount_cb) */
	void replugUsb();
	/** Step 4: sends "axis.0.drvtype=<id>" over CDC. Does not run the chain further than needed for the reply */
	void selectDriver(uint32_t classId);
	/** Runs the scheduler until the TMC state is `state` (or maxMs passed). @return true if reached */
	bool runUntilTmcState(TMC_ControlState state, uint32_t maxMs);

	// --- CDC ---
	/** Sends one command line (a newline is appended) over CDC and steps the chain until the reply ends with a newline (maxMs). @return the reply text */
	std::string cdcCommand(const std::string &line, uint32_t maxMs = 100);

	/**
	 * Reads the HID report descriptor of the enumerated device over USB (HID 1.11 7.1.1 Get_Descriptor): control IN transfer
	 * bmRequestType 0x81 (device to host, standard, interface), bRequest GET_DESCRIPTOR (6), wValue = report descriptor type
	 * 0x22 << 8 | index 0, wIndex = the HID interface number, wLength = wDescriptorLength of the HID descriptor in the
	 * configuration descriptor (UsbHost::hidGetReportDescriptor). REQUIREs success and that exactly wLength bytes came back.
	 * Needs enumerateUsb() (bringUp() does it) first.
	 */
	std::vector<uint8_t> readHidReportDescriptor();
	/** wDescriptorLength of the HID descriptor of the configuration descriptor (0 if none) */
	uint16_t hidReportDescriptorLength() const;

	/**
	 * The simulated host takes the reports waiting on the HID interrupt IN endpoint (as its polling does) and KEEPS every one in
	 * hidInReports (WO I1-1: FfbHost decodes the PID State reports from there). Draining behaviour is unchanged (IB 3).
	 */
	void drainHidIn();

	/** One input report taken from the HID interrupt IN endpoint: bytes as received (the ID byte first), and the simulated time when it was taken */
	struct HidInReport {
		uint64_t timeUs = 0;
		std::vector<uint8_t> bytes;
	};
	std::vector<HidInReport> hidInReports; //!< every report drainHidIn took since the fixture was constructed, in order

	// --- board ---
	/** External and internal supply in mV through the fake ADC2 (counts computed with the current vSenseMult, as TmcFixture) */
	void setSupply(int32_t extMv, int32_t intMv);
	void powerOn() { setSupply(24000, 24000); }
	void powerOff() { setSupply(0, 0); }
	/** Simulated time per SPI frame of both chips (microseconds) */
	void setSpiFrameUs(uint32_t us);

	// --- plant (WO I0-7) ---
	/**
	 * Attaches a WheelPlant to a chip (default chip1 = TMC_1) and makes ChainScheduler step (f) call WheelPlant::stepToUs once per
	 * scheduler step. The plant is not owned. A plant that is a local of the test body may be destroyed before the fixture (also
	 * after a failed REQUIRE): ~WheelPlant runs a hook registered here that clears the fixture's pointer and the scheduler hook, so
	 * firstFault() never touches a destroyed plant. The member `wheel` has the lifetime of the fixture (use it by default).
	 * Its violations are part of firstFault().
	 */
	void attachPlant() { attachPlant(wheel, chip1); }
	void attachPlant(WheelPlant &plant) { attachPlant(plant, chip1); }
	void attachPlant(WheelPlant &plant, FakeTmc4671 &chip);
	void detachPlant();
	WheelPlant *plant() const { return plant_; }

	// --- position script (WO I1-1) ---
	/**
	 * Attaches a PositionScript to a chip (default chip1) and makes ChainScheduler step (f) call PositionScript::stepToUs once per
	 * scheduler step (value written at the END of the step: the firmware reads it in the FFB cycle of the next step, see
	 * PositionScript.h). The slot is shared with the plant: attaching a script while a plant is attached, or a plant while a
	 * script is attached, is a fixture fault (firstFault(); the second attach is refused). Not owned; a script destroyed before the
	 * fixture is forgotten (destroy hook, as for the plant).
	 */
	void attachPositionScript(PositionScript &script) { attachPositionScript(script, chip1); }
	void attachPositionScript(PositionScript &script, FakeTmc4671 &chip);
	void detachPositionScript();
	PositionScript *positionScript() const { return script_; }
	/** Forgets the recorded fixture fault (only for the self-test that provokes it on purpose) */
	void clearFixtureFault() { fixtureFault_.clear(); }

	// --- observation (public interfaces only) ---
	/** The TMC4671 driver object of axis 0 (CommandHandler "tmc" instance 0), nullptr if the axis has no TMC_1 */
	TMC4671 *tmc();
	/** The main class (FFBWheel after bringUp), nullptr if none */
	FFBoardMain *mainClass() const { return mainclass; }
	/** True if mainclass is an FFBWheel */
	bool isFfbWheel() const;
	/** One decoded write to PID torque / flux target 0x64: flux [15:0], torque [31:16] (TMC4671 DS 0x64) */
	struct FluxTorque {
		int16_t torque = 0;
		int16_t flux = 0;
	};
	/** Writes to register 0x64 of a chip in order, decoded */
	static std::vector<FluxTorque> fluxTorqueWrites(const FakeTmc4671 &chip);
	/** FFB main cycles run since reset (ChainScheduler::Stats::ffbCycles) */
	uint32_t ffbCycles() const { return sched.stats().ffbCycles; }

	// --- teardown checks ---
	/** First recorded fault: HostDcd, UsbHost, unmodelled HAL call, kernel fault, undeclared FakeTmc4671 violation. Empty if none */
	std::string firstFault() const;

private:
	WheelPlant *plant_ = nullptr;
	PositionScript *script_ = nullptr;
	std::string fixtureFault_; //!< first misuse of the fixture (plant and script attached together)
	HostRtos::ObjectCounts atStart_;
	uint64_t startUs_ = 0;
	int32_t supplyExt_ = 0;
	int32_t supplyInt_ = 0;
};

#endif
