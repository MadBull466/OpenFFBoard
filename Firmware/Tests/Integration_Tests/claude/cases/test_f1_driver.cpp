/*
 * F1 driver start-up and readiness (WO I2-2): F1-T1 order and safety of the start-up, F1-T2 an effect started before the driver is ready.
 *
 * Every case attaches a ChipEventLog to chip1 (TMC_1) before the driver is selected and reads the order of the register writes and the
 * level of the driver enable pin (DRV_ENABLE, PE7) sampled at each write. The position is scripted constant 0 through 0x6B
 * (PositionScript). Settings are made over CDC, power and fxratio first (IB 7 / bug 23).
 *
 * Bring-up used here (split bring-up of IntegrationFixture, as LateFixture of test_f1_axis.cpp): boot(), enumerateUsb(), settings,
 * selectDriver(1) (axis.0.drvtype=1 while USB is connected: Axis::setDrvType -> setupDriver + startMotor, Axis.cpp:512-545), wait for
 * Shutdown, then a replug (the PC connects afterwards, effects are sent after it) and the wait for Running.
 * The other order of F1-T1, the driver created before the USB mount, is the one of a power cycle with a stored drvtype
 * (rebootKeepingFlash with allowReplug = true: with a stored TMC configuration the chain ends in Shutdown without the replug, which is
 * the defect of I2-6 and not tested here).
 *
 * Oracles
 *  - TMC4671-LA datasheet Rev 2.07 (Tests/Code_Coverage/shared/specs/TMC4671-LA_datasheet_Rev2.07.pdf):
 *      0x1A PWM_SV_CHOP [7:0]: 0 = PWM off, 7 = centred PWM with FOC (the value the firmware calls PWM_FOC);
 *      0x64 PID_TORQUE_FLUX_TARGET: flux [15:0], torque [31:16], s16 each; reset value 0 (a chip that was not power-cycled keeps its
 *        last value);
 *      0x5E PID_TORQUE_FLUX_LIMITS u16(15:0): the limit of the target in the units of 0x64, so a power P the axis may command gives
 *        0x5E = P (DS + MATH, as F1-X1 of test_f1_axis.cpp).
 *    PLAIN SAFETY PROPERTY (no firmware constant): while the power stage is not enabled, no torque or flux target other than 0 may be
 *    written to 0x64, and a target that the chip holds from before must not meet an enabled power stage. "Enabled" = 0x1A [7:0] = 7
 *    and the enable pin high (the two conditions the firmware itself sets, TMC4671.cpp:423-424).
 *  - Firmware order of the start-up (TMC4671::setupDriver TMC4671.cpp:243-254, initialize :260-359, initializeWithPower :423-424,
 *    startMotor :1682-1691): characterization (INR 33).
 *  - F1-T2: the axis fade-in (usbResume Axis.cpp:639-643: startForceFadeIn() = 0 -> 1 with 0.5 per 1000 cycles, i.e. 2 s, Axis.h:361,
 *    Axis.cpp:872-876) is a function of the time since the USB mount, not of the effect: the reference run (driver already running) and
 *    the run with the effect pending before Running have the same fade timeline, so the torque at the same time since the mount must be
 *    the same.
 *  - Replugs: a running driver that is stopped by a USB detach and started again at once ends in Shutdown (stopMotor sets only
 *    requestedState, TMC4671.cpp:1679; startMotor :1685; the thread applies the stale Shutdown at :452-453, about 100 ms later): that
 *    defect is covered in test_f1_runtime.cpp (F1-T7, I2-4). The cases here that need a running driver after a replug use replugHold(): the
 *    detach is held for 300 ms (the thread delay of :577 lets the stop complete) and every such case REQUIREs a ready driver at the end.
 *  - Stop windows (mutation check of the safety cases): a stop is only a test of the safety property if a torque demand exists and the
 *    window contains FFB cycles. stopWindow() holds a USB detach 300 ms with the idle spring as demand (position 20000 counts, idlespring
 *    50: the unmount deletes the effects, Axis.cpp:663, the axis keeps computing during the suspend, NR 40); the E-stop was tried and
 *    gives no demand (the axis is not updated during an emergency).
 *
 * Float to int: nothing in this file converts a float to an integer; the settings are in range (power 7000, magnitude 20000).
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
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using hid::ReportType;

namespace {

constexpr uint32_t kDirectionX = hid::usage(0x0A, 1);
constexpr uint8_t kRegPwm = 0x1A;
constexpr uint8_t kRegTarget = 0x64;
constexpr uint8_t kRegLimits = 0x5E;
constexpr uint32_t kPwmFoc = 7; // DS 0x1A [7:0]
constexpr int64_t kPower = 7000;

struct DriverFixture : IntegrationFixture {
	std::optional<ChipEventLog> log;
	std::optional<FfbHost> host;
	PositionScript script;

	FfbHost &h() { return *host; }
	ChipEventLog &lg() { return *log; }

	/** Log attached first, then boot, USB, settings, drvtype=1 over CDC with USB connected, the driver stopped in Shutdown */
	void toShutdown(int64_t power, int64_t idleSpring = 0, int32_t position = 0) {
		log.emplace(*this, chip1);
		script.setConstant(position);
		attachPositionScript(script);
		boot();
		enumerateUsb();
		host.emplace(*this);
		h().command("axis.0.power=" + std::to_string(power));
		h().command("axis.0.fxratio=255");
		h().command("axis.0.degrees=360");
		h().command("axis.0.invert=0");
		h().command("axis.0.idlespring=" + std::to_string(idleSpring));
		selectDriver(1);
		REQUIRE(tmc() != nullptr);
		REQUIRE(runUntilTmcState(TMC_ControlState::Shutdown, 20000));
	}
	bool driverReady() { return tmc()->getState() == TMC_ControlState::Running && tmc()->motorReady(); }
	/** Replug (effects are deleted), `send`, then 1 ms steps until Running and motorReady; as mountSendRun of LateFixture */
	template <class F> void mountSendRun(F send) {
		replugUsb();
		REQUIRE(!driverReady());
		send();
		bool upNow = false;
		for (uint32_t ms = 0; ms < 20000 && !upNow; ms++) {
			upNow = driverReady();
			if (!upNow) {
				sched.advanceMs(1);
			}
		}
		REQUIRE(upNow);
	}

	/**
	 * USB detach held for `holdMs` of simulated time, then attach and enumerate (as replugUsb, with time between). The stop of a
	 * running driver completes during the hold, so the mount that follows finds Shutdown (no stale requestedState, see the header).
	 */
	void replugHold(uint32_t holdMs) {
		HostDcd::detach();
		sched.pumpUsb(); // tud_umount_cb -> usbSuspend (Axis.cpp:629-635)
		sched.advanceMs(holdMs);
		HostDcd::attach();
		enumerateUsb(); // tud_mount_cb -> usbResume (Axis.cpp:639-643)
	}
	/** Result of stopWindow */
	struct StopWindow {
		size_t startEvent = 0;	//!< size of the event log when the detach began
		uint32_t cycles = 0;	//!< FFB cycles that ran while the USB was detached
		int64_t demand = 0;		//!< axis.0.curtorque? after the mount: the axis demand that kept being computed (idle spring)
	};
	/**
	 * A real stop window with a torque demand: the USB detach (Axis::usbSuspend: turn(0), stopMotor: PWM off) held 300 ms. The effects are
	 * deleted by the unmount, so the idle spring (FFB inactive, Axis.cpp:663, needs axis.0.idlespring > 0 and a position away from the
	 * centre: toShutdown's arguments) is the demand, and the axis keeps computing during the suspend (NR 40). After the mount the driver
	 * is waited for (a clean restart takes about 110 ms) and REQUIREd ready (B1 of the review).
	 */
	StopWindow stopWindow() {
		StopWindow w;
		w.startEvent = lg().events().size();
		const uint32_t c0 = ffbCycles();
		replugHold(300);
		w.cycles = ffbCycles() - c0;
		for (uint32_t ms = 0; ms < 2000 && !driverReady(); ms++) {
			sched.advanceMs(1);
		}
		REQUIRE(driverReady());
		h().run(50);
		w.demand = h().getValue("axis.0.curtorque?");
		return w;
	}

	pid::PidReport makeSetEffect(int64_t index, uint32_t typeUsage, int64_t direction) {
		pid::PidReport se = h().builder().make(ReportType::Output, pid::use::SetEffectReport);
		se.set(pid::use::EffectBlockIndex, index).select(typeUsage).set(pid::use::Duration, 32767).set(pid::use::Gain, 255);
		se.flag(pid::use::DirectionEnable, true);
		se.setIn({pid::use::Direction}, kDirectionX, direction);
		return se;
	}
	/** Constant force, Direction Enable, polar direction (0.01 degrees), Magnitude, started */
	int64_t startConstant(int64_t direction, int64_t magnitude) {
		const FfbHost::CreateResult c = h().createEffect(pid::use::EtConstantForce);
		REQUIRE(c.load.success());
		const int64_t idx = c.load.effectBlockIndex;
		h().send(makeSetEffect(idx, pid::use::EtConstantForce, direction));
		pid::PidReport sc = h().builder().make(ReportType::Output, pid::use::SetConstantForceReport);
		sc.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, magnitude);
		h().send(sc);
		pid::PidReport eo = h().builder().make(ReportType::Output, pid::use::EffectOperationReport);
		eo.set(pid::use::EffectBlockIndex, idx).select(pid::use::OpEffectStart).set(pid::use::LoopCount, 1);
		h().send(eo);
		return idx;
	}
};

/** Result of walking the whole event log of 0x1A / 0x64 */
struct SafetyScan {
	size_t unsafeAt = ChipEventLog::npos; //!< first write of a non-zero target while 0x1A [7:0] != 7 or the enable pin was low
	size_t nonzero = 0;					 //!< writes of a non-zero target (non-vacuity)
	size_t nonzeroAfter = 0;			 //!< of those, the ones with index >= `from`
};

/** PLAIN property (file header): walks events from the start (0x1A = 0 at power-on) and tracks the PWM mode written so far */
SafetyScan scanSafety(const ChipEventLog &l, size_t from = 0) {
	SafetyScan s;
	uint32_t pwm = 0;
	for (size_t i = 0; i < l.events().size(); i++) {
		const ChipEventLog::Event &e = l.events()[i];
		if (e.kind != ChipEventLog::Kind::Write) {
			continue;
		}
		if (e.addr == kRegPwm) {
			pwm = e.data & 0xFFu;
		} else if (e.addr == kRegTarget && e.data != 0) {
			s.nonzero++;
			if (i >= from) {
				s.nonzeroAfter++;
			}
			if ((pwm != kPwmFoc || !e.enable) && s.unsafeAt == ChipEventLog::npos) {
				s.unsafeAt = i;
			}
		}
	}
	return s;
}

/** First write event at or after `from` at which the power stage is enabled: 0x1A [7:0] = 7 written before and the pin high at this write; npos if none */
size_t firstEnableInstant(const ChipEventLog &l, size_t from = 0) {
	uint32_t pwm = 0;
	for (size_t i = 0; i < l.events().size(); i++) {
		const ChipEventLog::Event &e = l.events()[i];
		if (e.kind != ChipEventLog::Kind::Write) {
			continue;
		}
		if (e.addr == kRegPwm) {
			pwm = e.data & 0xFFu;
		}
		if (i >= from && pwm == kPwmFoc && e.enable) {
			return i;
		}
	}
	return ChipEventLog::npos;
}

bool isFocWrite(uint32_t d) { return (d & 0xFFu) == kPwmFoc; }
bool isOffWrite(uint32_t d) { return (d & 0xFFu) == 0; }

int16_t torqueOf(uint32_t d) { return static_cast<int16_t>((d >> 16) & 0xFFFF); }

/** Torque in force at 0x64 at simulated time `tUs` (last write at or before it, `initial` if none) */
int16_t torqueAtTime(const ChipEventLog &l, size_t from, uint64_t tUs, int16_t initial) {
	int16_t v = initial;
	for (size_t i = from; i < l.events().size(); i++) {
		const ChipEventLog::Event &e = l.events()[i];
		if (e.kind == ChipEventLog::Kind::Write && e.addr == kRegTarget) {
			if (e.timeUs > tUs) {
				break;
			}
			v = torqueOf(e.data);
		}
	}
	return v;
}

/** Index of the first write to 0x64 at or after `from` with a non-zero torque; npos if none */
size_t firstNonzeroTorque(const ChipEventLog &l, size_t from) {
	for (size_t i = from; i < l.events().size(); i++) {
		const ChipEventLog::Event &e = l.events()[i];
		if (e.kind == ChipEventLog::Kind::Write && e.addr == kRegTarget && torqueOf(e.data) != 0) {
			return i;
		}
	}
	return ChipEventLog::npos;
}

} // namespace

TEST_SUITE("F1Driver") {

// ---------------------------------------------------------------------------------------------------------------------
// F1-T1  start-up order and safety
// ---------------------------------------------------------------------------------------------------------------------

// Plain safety property (file header). Driver selected by command while USB is connected (fresh flash), an idle spring demand exists from
// the start (position 20000 counts, idlespring 50), the effect sent before Running, then Running and 1.5 s of torque, then a REAL stop window:
// the USB detach is held 300 ms (the unmount deletes the effects and the idle spring keeps a demand, the window has about 300 FFB cycles,
// the PWM goes off). Non-vacuous: non-zero targets exist, the demand is non-zero after the window, the log is complete. Mutation check
// done by hand (report): without the motorReady gates (Axis.cpp:492, TMC4671.cpp:1746) the stop window carries non-zero writes with the PWM off.
TEST_CASE_FIXTURE(DriverFixture, "F1-T1: no non-zero torque or flux reaches 0x64 while 0x1A is not in FOC mode or the enable pin is low, driver selected with USB connected, effect pending, then a stop window" * IT_DELETES_FFBHIDMAIN) {
	toShutdown(kPower, 50, 20000);
	mountSendRun([&] { startConstant(9000, 20000); });
	setSpiFrameUs(0);
	h().run(1500);
	const StopWindow w = stopWindow();
	const SafetyScan s = scanSafety(lg());
	const size_t off = lg().firstWrite(kRegPwm, isOffWrite, w.startEvent);
	INFO("non-zero target writes " << s.nonzero << ", first unsafe event " << s.unsafeAt << ", axis demand after the window " << w.demand << ", FFB cycles in the stop window " << w.cycles << ", PWM off at event " << off);
	CHECK(lg().complete());
	CHECK(s.nonzero > 100); // the fade-in changes the target every cycle
	CHECK(w.demand != 0);	// a demand exists
	CHECK(w.cycles >= 250);
	REQUIRE(off != ChipEventLog::npos);
	REQUIRE_CHAIN_MODELLED();
	CHECK(s.unsafeAt == ChipEventLog::npos);
}

// Same property across a second stop window: a new effect is sent after the first mount, the driver is REQUIREd ready, then the detach is held
// again. The restart after each stop must not write a target before the PWM and the pin are on again.
TEST_CASE_FIXTURE(DriverFixture, "F1-T1: no non-zero torque reaches 0x64 while the PWM is off after a stop, across two held USB detaches with a new effect between" * IT_DELETES_FFBHIDMAIN) {
	toShutdown(kPower, 50, 20000);
	mountSendRun([&] { startConstant(9000, 20000); });
	setSpiFrameUs(0);
	h().run(1500);
	const StopWindow w1 = stopWindow();
	const size_t off1 = lg().firstWrite(kRegPwm, isOffWrite, w1.startEvent);
	const size_t foc1 = lg().firstWrite(kRegPwm, isFocWrite, off1 == ChipEventLog::npos ? w1.startEvent : off1);
	startConstant(27000, 15000);
	h().run(1500);
	REQUIRE(driverReady());
	const StopWindow w2 = stopWindow();
	const size_t off2 = lg().firstWrite(kRegPwm, isOffWrite, w2.startEvent);
	const SafetyScan s = scanSafety(lg());
	INFO("first window: PWM off / FOC again at events " << off1 << " / " << foc1 << ", cycles " << w1.cycles << ", demand " << w1.demand << "; second window: PWM off at " << off2 << ", cycles " << w2.cycles << ", demand " << w2.demand << ", first unsafe event " << s.unsafeAt << ", non-zero writes " << s.nonzero);
	CHECK(lg().complete());
	REQUIRE(off1 != ChipEventLog::npos);
	REQUIRE(foc1 != ChipEventLog::npos);
	CHECK(off1 < foc1);
	CHECK(firstNonzeroTorque(lg(), foc1) != ChipEventLog::npos); // the effect after the first detach reached the chip
	CHECK(w1.demand != 0);
	CHECK(w2.demand != 0);
	CHECK(w1.cycles >= 250);
	CHECK(w2.cycles >= 250);
	REQUIRE(off2 != ChipEventLog::npos);
	REQUIRE_CHAIN_MODELLED();
	CHECK(s.unsafeAt == ChipEventLog::npos);
}

// Order of the start-up as the firmware does it, the parts that are not a defect (INR 33), from the event log of a driver selected by
// command with USB connected:
//  1. Axis::setDrvType: setupDriver -> setLimits (0x5C..0x62, TMC4671.cpp:249) with the enable pin low, before any FOC write;
//  2. the TMC thread: initialize writes 0x64 = 0 (:300) and the PWM off (:301) with the enable pin high;
//  3. initializeWithPower: pin set, 0x1A FOC again (:423-424).
// The FOC write of startMotor that comes BEFORE step 2 (TMC4671.cpp:1688-1691) is not pinned here: it is IB 38 (case below).
TEST_CASE_FIXTURE(DriverFixture, "characterization: [F1-T1] the start-up writes the limits before any FOC write, then initialize zeroes 0x64 and switches the PWM off, then FOC is written again with the pin high" * IT_DELETES_FFBHIDMAIN) {
	// INR 33: order of setupDriver / initialize / initializeWithPower (TMC4671.cpp:243-254, 260-301, 423-424)
	toShutdown(kPower);
	CHECK(lg().complete());
	const size_t limits = lg().firstWrite(kRegLimits);
	const size_t foc1 = lg().firstWrite(kRegPwm, isFocWrite);
	REQUIRE(limits != ChipEventLog::npos);
	REQUIRE(foc1 != ChipEventLog::npos);
	const size_t zero64 = lg().firstWrite(kRegTarget, [](uint32_t d) { return d == 0; }, limits);
	REQUIRE(zero64 != ChipEventLog::npos);
	const size_t off = lg().firstWrite(kRegPwm, isOffWrite, zero64);
	REQUIRE(off != ChipEventLog::npos);
	const size_t foc2 = lg().firstWrite(kRegPwm, isFocWrite, off);
	REQUIRE(foc2 != ChipEventLog::npos);
	INFO("limits " << limits << " first FOC " << foc1 << " zero64 " << zero64 << " off " << off << " foc2 " << foc2);
	CHECK(limits < foc1);
	CHECK(zero64 < off);
	CHECK(off < foc2);
	CHECK_FALSE(lg().events()[limits].enable);
	CHECK(lg().events()[zero64].enable);
	CHECK(lg().events()[foc2].enable);
	REQUIRE_CHAIN_MODELLED();
}

// IB 38. The user's own path: axis.0.drvtype=1, an effect runs for 3 s, axis.0.drvtype=0 (the driver is deleted; ~TMC4671 resets only the
// enable pin, TMC4671.cpp:82-85, the chip keeps 0x1A = 7 and 0x64 = the last torque), axis.0.drvtype=1 again. Axis::setDrvType calls startMotor right
// after setupDriver, while the TMC thread has not run yet (state uninitialised, TMC4671.cpp:1688-1692): FOC and the enable pin are on with
// 0x63 = torque mode and the old target in force; only initialize() writes 0x64 = 0 (:300), later. Intended: the target is zero before the
// power stage is enabled. (A reset of the MCU alone, with the chip still powered, has the same effect if the warm-start detection around
// TMC4671.cpp:289-295 does not hold; not asserted here.)
TEST_CASE_FIXTURE(DriverFixture, "F1-T1: the power stage is not enabled while 0x64 still holds the torque of the previous driver, driver re-selected by axis.0.drvtype" * KNOWN_ISSUE("(IB 38) Axis::setDrvType -> startMotor enables FOC and the enable pin (TMC4671.cpp:1688-1691) before initialize() zeroes 0x64 (:300): the torque of the previous driver meets the enabled power stage") * IT_DELETES_FFBHIDMAIN) {
	toShutdown(kPower);
	mountAndRun();
	setSpiFrameUs(0);
	startConstant(9000, 20000);
	h().run(3000);
	const uint32_t held = chip1.reg(kRegTarget);
	REQUIRE(torqueOf(held) != 0);
	const std::string off = cdcCommand("axis.0.drvtype=0");
	INFO("drvtype=0 reply: " << off);
	h().run(100);
	const size_t mark = lg().events().size();
	selectDriver(1);
	REQUIRE(tmc() != nullptr);
	h().run(50);
	const size_t at = firstEnableInstant(lg(), mark);
	REQUIRE(at != ChipEventLog::npos);
	const std::optional<uint32_t> written = lg().valueAt(kRegTarget, at);
	const uint32_t inForce = written.has_value() ? *written : 0;
	INFO("power stage enabled at event " << at << " (" << (at - mark) << " events after the re-selection), 0x64 in force 0x" << std::hex << inForce << ", before the drvtype=0 0x" << held);
	CHECK(lg().complete());
	REQUIRE_CHAIN_MODELLED();
	CHECK(inForce == 0);
}

// IB 17 at the moment of the enable, runtime-selection path. axis.0.power = 7000 is set BEFORE the driver is selected by command (Axis::setPower
// finds no driver and only keeps the value, Axis.cpp:498-506); Axis::setDrvType never passes it on (Axis.cpp:512-545) and setupDriver writes the
// driver default (TMC4671Limits 30000, TMC4671.h:238) to 0x5E, so the power stage is enabled with a limit four times the user's power. The boot
// path is fine: restoreFlash calls setDrvType (Axis.cpp:231) before setPower (:262), so a stored power reaches the driver. This case is the
// runtime path and cross-references F1-X1 of test_f1_axis.cpp (same defect on the default power); here the power is a non-default value
// and the check is made at the enable instant.
TEST_CASE_FIXTURE(DriverFixture, "F1-T1: when the power stage is enabled 0x5E holds the axis.0.power that was set before the driver was selected" * KNOWN_ISSUE("(IB 17) a driver created by Axis::setDrvType never gets the axis power: 0x5E holds the driver default 30000 when FOC and the enable pin go on, axis.0.power? reports 7000") * IT_DELETES_FFBHIDMAIN) {
	toShutdown(kPower);
	REQUIRE(h().getValue("axis.0.power?") == kPower);
	const size_t at = firstEnableInstant(lg());
	REQUIRE(at != ChipEventLog::npos);
	const std::optional<uint32_t> lim = lg().valueAt(kRegLimits, at);
	REQUIRE(lim.has_value());
	INFO("0x5E in force at the enable instant " << (*lim & 0xFFFFu));
	REQUIRE_CHAIN_MODELLED();
	CHECK((*lim & 0xFFFFu) == kPower);
}

// The other order of F1-T1: the driver is created before the USB mount. A power cycle with the drvtype stored (axis.0.drvtype=1 and sys.save
// in flash) creates the driver in the start-up path (Axis restore), USB is not connected yet (tud_connected false: setDrvType takes the
// usbSuspend branch, Axis.cpp:537-540). Running is reached after the USB mount (rebootKeepingFlash allowReplug = true: the replug is the
// documented way of a stored configuration, the missing start without it is I2-6). Plain: Running is reached and the safety property holds
// over the whole log of the second start, effect after Running non-zero.
TEST_CASE_FIXTURE(DriverFixture, "F1-T1: with the driver created before the USB mount (stored drvtype) Running is reached and no non-zero torque reaches 0x64 before FOC and the enable pin" * IT_DELETES_FFBHIDMAIN) {
	toShutdown(kPower, 50, 20000);
	mountAndRun();
	setSpiFrameUs(0);
	const std::string saved = cdcCommand("sys.save?", 500);
	INFO("sys.save reply: " << saved);
	host.reset();
	rebootKeepingFlash(20000, true);
	REQUIRE(firstFault().empty());
	// the driver was created from the stored type before the enumeration: the stored configuration needed the replug (stored type proof)
	CHECK(rebootReplugged());
	REQUIRE(tmc() != nullptr);
	CHECK(driverReady());
	attachPositionScript(script);
	host.emplace(*this);
	h().command("axis.0.power=" + std::to_string(kPower));
	h().command("axis.0.fxratio=255");
	h().command("axis.0.degrees=360");
	h().command("axis.0.invert=0");
	h().command("axis.0.idlespring=50");
	setSpiFrameUs(0);
	const size_t mark = lg().events().size();
	startConstant(9000, 20000);
	h().run(1500);
	const StopWindow w = stopWindow();
	const SafetyScan s = scanSafety(lg());
	INFO("non-zero target writes " << s.nonzero << ", after the effect " << s.nonzeroAfter << ", first unsafe event " << s.unsafeAt << ", effect started at event " << mark << ", stop window cycles " << w.cycles << ", demand " << w.demand);
	CHECK(lg().complete());
	CHECK(s.nonzeroAfter > 100);
	CHECK(w.demand != 0);
	CHECK(w.cycles >= 250);
	REQUIRE_CHAIN_MODELLED();
	CHECK(s.unsafeAt == ChipEventLog::npos);
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-T2  effect started before the driver is ready
// ---------------------------------------------------------------------------------------------------------------------

// Cases already in test_f1_axis.cpp (LateFixture), not repeated: "a constant force set before the driver runs reaches the chip at its full
// torque once the driver is Running", "with slewrate R the first torque written after the driver runs is within R of the value the chip
// held", IB 24, IB 25, IB 22. Added here with the event log: the chip's target stays exactly 0 in every cycle before Running.
// Plain (DS 0x64 / 0x1A, safety): the effect is sent right after the replug and the chain steps 1 ms at a time; at the end of every step
// in which the driver is not Running and motorReady the register 0x64 is 0 (flux and torque) and no write to it happened. After
// Running the target becomes non-zero (not vacuous).
TEST_CASE_FIXTURE(DriverFixture, "F1-T2: an effect started before the driver is ready gives exactly 0 at 0x64 in every cycle until Running" * IT_DELETES_FFBHIDMAIN) {
	toShutdown(kPower);
	setSpiFrameUs(0);
	replugUsb();
	REQUIRE(!driverReady());
	const size_t mark = lg().events().size();
	startConstant(9000, 20000);
	uint32_t notReadySteps = 0;
	uint32_t nonZeroNotReady = 0;
	uint32_t writesNotReady = 0;
	size_t seen = mark;
	for (uint32_t ms = 0; ms < 20000 && !driverReady(); ms++) {
		sched.advanceMs(1);
		if (!driverReady()) {
			notReadySteps++;
			if (chip1.reg(kRegTarget) != 0) {
				nonZeroNotReady++;
			}
			for (; seen < lg().events().size(); seen++) {
				const ChipEventLog::Event &e = lg().events()[seen];
				if (e.kind == ChipEventLog::Kind::Write && e.addr == kRegTarget) {
					writesNotReady++;
				}
			}
		}
	}
	REQUIRE(driverReady());
	INFO("steps before Running " << notReadySteps << ", with a non-zero register " << nonZeroNotReady << ", writes to 0x64 in them " << writesNotReady);
	CHECK(notReadySteps > 20); // a real not-ready window, so the zero is a statement about a driver that is not ready
	h().run(300);
	CHECK(firstNonzeroTorque(lg(), mark) != ChipEventLog::npos);
	CHECK(lg().complete());
	REQUIRE_CHAIN_MODELLED();
	CHECK(nonZeroNotReady == 0);
	CHECK(writesNotReady == 0);
}

namespace {

/**
 * The reference comparison of F1-T2 (slewrate off, the default). Run A: the first mount after the start-up (the driver is in Shutdown, so
 * no stop is involved), the effect sent before Running, wait for Running, 300 ms: its writes to 0x64 from the first non-zero one on, as
 * (time since the mount, torque). Run R (reference, driver already running with the effect): after a settling time a USB detach held 300 ms
 * (replugHold: the stop completes, B1 of the review), the chain runs until the driver is ready again (a clean restart takes about 110 ms,
 * not the stale-Shutdown path of test_f1_runtime.cpp), then the same effect is sent and runs 400 ms. A ready driver is REQUIREd at the start
 * of R's window and at its end. Both runs have the fade of the mount (file header), so at the same time since the mount the torque of A and
 * of R is the same up to one FFB cycle. R's effect exists only from its own ready time, so A is compared from 10 ms after its first write
 * on (after R's effect has started), and the FIRST value of A is checked for continuity with A's own ramp.
 */
struct Compare {
	std::vector<std::pair<uint64_t, int16_t>> a; //!< (us since the mount, torque) of A from its first non-zero write, 60 ms
	size_t compared = 0;
	size_t outOfBand = 0;
	int worstDelta = 0;
	uint64_t dA = 0;
	uint64_t dRFirst = 0; //!< first non-zero write of R, us since its mount
};

Compare compareAgainstReference(DriverFixture &f) {
	f.toShutdown(kPower);
	f.setSpiFrameUs(0);
	Compare c;
	// run A
	const size_t markA = f.lg().events().size();
	const uint64_t t0A = HostPlatform::nowUs();
	f.mountSendRun([&] { f.startConstant(9000, 20000); });
	f.h().run(300);
	const size_t first = firstNonzeroTorque(f.lg(), markA);
	REQUIRE(first != ChipEventLog::npos);
	c.dA = f.lg().events()[first].timeUs - t0A;
	for (size_t i = first; i < f.lg().events().size(); i++) {
		const ChipEventLog::Event &e = f.lg().events()[i];
		if (e.kind == ChipEventLog::Kind::Write && e.addr == kRegTarget && e.timeUs - t0A <= c.dA + 60000) {
			c.a.push_back({e.timeUs - t0A, torqueOf(e.data)});
		}
	}
	f.h().run(2500);
	// run R
	REQUIRE(f.driverReady());
	const size_t markR = f.lg().events().size();
	f.replugHold(300);
	const uint64_t t0R = HostPlatform::nowUs(); // the mount (usbResume) happened in the enumeration that just ended
	for (uint32_t ms = 0; ms < 2000 && !f.driverReady(); ms++) {
		f.sched.advanceMs(1);
	}
	REQUIRE(f.driverReady());
	f.startConstant(9000, 20000);
	f.h().run(400);
	REQUIRE(f.driverReady());
	const size_t firstR = firstNonzeroTorque(f.lg(), markR);
	REQUIRE(firstR != ChipEventLog::npos);
	c.dRFirst = f.lg().events()[firstR].timeUs - t0R;
	for (const auto &w : c.a) {
		if (w.first < c.dA + 10000) {
			continue;
		}
		const uint64_t at = t0R + w.first;
		const int16_t r0 = torqueAtTime(f.lg(), markR, at - 1000, 0);
		const int16_t r1 = torqueAtTime(f.lg(), markR, at, 0);
		const int16_t r2 = torqueAtTime(f.lg(), markR, at + 1000, 0);
		const int lo = std::min({r0, r1, r2});
		const int hi = std::max({r0, r1, r2});
		c.compared++;
		if (w.second < lo || w.second > hi) {
			c.outOfBand++;
			c.worstDelta = std::max(c.worstDelta, std::max(lo - w.second, w.second - hi));
		}
	}
	return c;
}

} // namespace

// Plain (comparison with a reference run, no firmware constant): at the same time since the mount the torque of the run whose effect was pending
// before Running equals the torque of a reference run whose driver was already running (the same effect, started later), within one FFB cycle
// (the reference sampled at t - 1 ms, t, t + 1 ms; MATH of the band: the torque changes by about 2 counts per cycle here, power 7000 gives at
// most 4272 for the effect, 0.5 per 1000 cycles of fade). A stale value in the limiter or the fade would put A away from R. The first value of A
// has no step: it differs from the second by what the second differs from the third (+-2 counts) and is not 0.
TEST_CASE_FIXTURE(DriverFixture, "F1-T2: after Running the torque of an effect that was pending equals the torque of a reference run that was already running, at the same time since the mount, and starts without a step" * IT_DELETES_FFBHIDMAIN) {
	const Compare c = compareAgainstReference(*this);
	REQUIRE(c.a.size() > 10);
	INFO("A: first non-zero " << c.a[0].second << " at " << c.dA << " us, then " << c.a[1].second << ", " << c.a[2].second << "; R's first non-zero at " << c.dRFirst << " us; compared " << c.compared << ", out of band " << c.outOfBand << ", worst " << c.worstDelta);
	CHECK(c.a[0].second != 0);
	CHECK(c.compared > 30);
	CHECK(c.dRFirst < c.dA + 10000); // R's effect had started before the compared range
	REQUIRE_CHAIN_MODELLED();
	CHECK(std::abs((c.a[1].second - c.a[0].second) - (c.a[2].second - c.a[1].second)) <= 2);
	CHECK(c.outOfBand == 0);
}

// IB 39 (same as bug 33, which records the usbSuspend / usbResume variant). Help text "Torque rate limit in counts/ms" (Axis.cpp:186): with
// slewrate R the target at the chip never changes by more than R per millisecond. A USB stop writes 0 to 0x64 (Axis::usbSuspend: turn(0),
// Axis.cpp:631) but the limiter's base metric.previous.torque keeps the torque from before (Axis.cpp:908-921, 917): the first FFB write after
// that turn(0) is the stale base moved by R (-4272 wound down by R per cycle, plus R), whatever effect exists then. Path used: a SHORT detach (replugHold 3 ms:
// the axis keeps computing and winds the base down by R per cycle, NR 40, so a hold above 4272 / 300 = 15 ms would hide the defect). The stop
// of a running driver is applied late and the driver drops to Shutdown about 100 ms after the mount (B1 of the review, test_f1_runtime.cpp);
// the measurement covers only the 50 ms after the mount, in which the driver is still Running (REQUIREd), so the step does not depend on that
// defect. No effect is sent after the mount, so it does not depend on a new effect. Observed with R = 300, power 7000, magnitude 20000: a step of 3072 (base -4272 + 3 * 300 - 300) after the 3 ms hold; 3972 with no hold.
TEST_CASE_FIXTURE(DriverFixture, "F1-T2: with slewrate R no write to 0x64 after a USB detach and mount changes the target by more than R" * KNOWN_ISSUE("(IB 39; same as bug 33) the slew limiter base keeps the torque from before the USB stop (Axis.cpp:917): the first FFB write after turn(0) is the stale base moved by R, a step of 3072 counts at R = 300 after a 3 ms detach") * IT_DELETES_FFBHIDMAIN) {
	constexpr int kRate = 300;
	toShutdown(kPower);
	mountAndRun();
	setSpiFrameUs(0);
	h().command("axis.0.slewrate=" + std::to_string(kRate));
	startConstant(9000, 20000);
	h().run(4000); // the plateau: fade (2 s) and the slew ramp are over
	REQUIRE(driverReady());
	const size_t mark = lg().events().size();
	replugHold(3);
	h().run(50);
	REQUIRE(driverReady());
	int prev = 0;
	int worst = 0;
	size_t worstAt = 0;
	bool started = false;
	for (size_t i = mark; i < lg().events().size(); i++) {
		const ChipEventLog::Event &e = lg().events()[i];
		if (e.kind != ChipEventLog::Kind::Write || e.addr != kRegTarget) {
			continue;
		}
		const int t = torqueOf(e.data);
		if (!started) { // the stop writes 0 first (turn(0)); the steps are measured from there
			started = true;
			prev = t;
			continue;
		}
		if (std::abs(t - prev) > worst) {
			worst = std::abs(t - prev);
			worstAt = i;
		}
		prev = t;
	}
	REQUIRE(started);
	INFO("largest step between two writes to 0x64 after the detach " << worst << " (event " << worstAt << "), bound " << kRate);
	CHECK(lg().complete());
	REQUIRE_CHAIN_MODELLED();
	CHECK(worst <= kRate);
}

} // TEST_SUITE
