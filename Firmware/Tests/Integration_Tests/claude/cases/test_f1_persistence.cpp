/*
 * F1 persistence across a power cycle (WO I2-6): F1-P1 (settings set by command, saved with sys.save, survive a reboot and give the same
 * torque) and F1-P2 (what is restored is in force before the first torque), plus the default power cycle with a stored TMC configuration
 * (charter group G-T / G-P; decisions log 2026-11-24 item 3).
 *
 * Oracles
 *  - A power cycle keeps the flash and loses the RAM; `sys.save` writes every setting of the PersistentStorage handlers to flash
 *    (SystemCommands.cpp:122), so a setting the user saved reads back with the saved value after the reboot (reply of the get command, the
 *    user's view over CDC). The units are those of the commands (help text of the registerCommand lines: Axis.cpp:173-215,
 *    EffectsCalculator.cpp:42-62, TMC4671.cpp:2772-2813).
 *  - TMC4671 datasheet Rev 2.07: 0x54 PID_FLUX_P_FLUX_I and 0x56 PID_TORQUE_P_TORQUE_I (P = s16(31:16), I = s16(15:0)) hold the restored PI
 *    gains after the start; 0x5E PID_TORQUE_FLUX_LIMITS u16(15:0) limits the target values (the largest torque the axis may command at 0x64
 *    is its power, so 0x5E = power); 0x64 PID_TORQUE_FLUX_TARGET torque s16(31:16).
 *  - Sign cases: the absolute sign of a constant force is not asserted (F1-X2 characterizes it). The setting is shown to act on the sign
 *    BEFORE the reboot (the first non-zero torque after a start flips with the setting: precondition, REQUIREd) and after the reboot the
 *    first non-zero torque has the sign of the saved setting.
 *
 * Firmware paths (read for this order): Axis::saveFlash Axis.cpp:333-366, Axis::restoreFlash :221-330 (setDrvType :231 first, so the TMC
 * driver exists when setPower :262-263 runs: the IB 17 failure of 0x5E is NOT seen at boot, the driver default 30000 is replaced by the
 * power before the first torque), TMC4671::saveFlash TMC4671.cpp:108-131 and restoreFlash :147-192 (called from setupDriver :248),
 * EffectsCalculator::saveFlash / restoreFlash EffectsCalculator.cpp:763-860.
 *
 * Not tested: tmc.0.pwmlim is not saved at all (it is not in TMC4671::saveFlash, TMC4671.cpp:108-131, nor read in restoreFlash; it is also a
 * CMDFLAG_DEBUG command, TMC4671.cpp:2779, "Invalid command" in this build), so there is no round trip to test. The motor / encoder settings
 * that would break the open-loop chain (mtype, poles, encsrc, phiesrc) are not set.
 *
 * Every case that needs Running after a reboot uses rebootKeepingFlash(20000, true) (allowReplug): with a stored TMC configuration the
 * chain ends in Shutdown without a USB replug (see the default power cycle case), and the replug is not what these cases test.
 * Float to int: none of these cases passes a value near a conversion limit (all setting values are small integers).
 */
#include "doctest.h"

#include "ChipEventLog.h"
#include "FfbHost.h"
#include "HidReportDescriptor.h"
#include "IntegrationFixture.h"
#include "PidReportBuilder.h"
#include "PositionScript.h"
#include "TestHelpers.h"

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using hid::ReportType;

namespace {

constexpr uint32_t kDirectionX = hid::usage(0x0A, 1); // first Direction value of Set Effect (polar angle with Direction Enable)
constexpr uint8_t kRegFluxPI = 0x54;
constexpr uint8_t kRegTorquePI = 0x56;
constexpr uint8_t kRegTorqueFluxLimits = 0x5E;
constexpr uint8_t kRegTorqueFluxTarget = 0x64;

struct PersistFixture : IntegrationFixture {
	std::optional<FfbHost> host;
	std::optional<ChipEventLog> log; //!< emplaced by cases that need the event log (it excludes TorqueTrace: both want the step hook)
	PositionScript script;

	FfbHost &h() { return *host; }

	/** Chain up, steady state (0 us per SPI frame), host side created */
	void up() {
		bringUp();
		setSpiFrameUs(0);
		host.emplace(*this);
	}

	/** Power cycle with a USB replug if needed (allowReplug), then the host side is created again; REQUIREs no fault and Running */
	void reboot() {
		host.reset();
		rebootKeepingFlash(20000, true);
		REQUIRE(firstFault().empty());
		REQUIRE(tmc() != nullptr);
		REQUIRE(tmc()->getState() == TMC_ControlState::Running);
		setSpiFrameUs(0);
		host.emplace(*this);
	}

	void save() {
		const std::string reply = cdcCommand("sys.save?", 500);
		INFO("sys.save reply: " << reply);
		REQUIRE(reply.find("OK") != std::string::npos);
	}

	void set(const std::string &line) { h().command(line); }

	/** Sets every "cmd=value" of the list */
	void setAll(const std::vector<std::pair<std::string, int64_t>> &list) {
		for (const auto &p : list) {
			set(p.first + "=" + std::to_string(p.second));
		}
	}

	/** After the reboot every setting of the list reads back with its saved value (all checked, a lost one is named) */
	void checkAll(const std::vector<std::pair<std::string, int64_t>> &list) {
		for (const auto &p : list) {
			const int64_t got = h().getValue(p.first + "?");
			INFO(p.first << " saved " << p.second << ", after the reboot " << got);
			CHECK(got == p.second);
		}
	}

	// ---- effects (same host sequences as test_f1_hid.cpp, written again locally) ----

	int64_t newEffect(uint32_t typeUsage) {
		const FfbHost::CreateResult c = h().createEffect(typeUsage);
		REQUIRE(c.load.success());
		return c.load.effectBlockIndex;
	}
	pid::PidReport makeSetEffect(int64_t index, uint32_t typeUsage, int64_t direction) {
		pid::PidReport se = h().builder().make(ReportType::Output, pid::use::SetEffectReport);
		se.set(pid::use::EffectBlockIndex, index).select(typeUsage).set(pid::use::Duration, 32767).set(pid::use::Gain, 255);
		se.flag(pid::use::DirectionEnable, true);
		se.setIn({pid::use::Direction}, kDirectionX, direction);
		return se;
	}
	void operation(int64_t index, uint32_t op) {
		pid::PidReport eo = h().builder().make(ReportType::Output, pid::use::EffectOperationReport);
		eo.set(pid::use::EffectBlockIndex, index).select(op).set(pid::use::LoopCount, 1);
		h().send(eo);
	}
	/** Create, Set Effect (polar direction in 0.01 degrees), Set Constant Force, Effect Operation Start */
	int64_t startConstant(int64_t direction, int64_t magnitude) {
		const int64_t idx = newEffect(pid::use::EtConstantForce);
		h().send(makeSetEffect(idx, pid::use::EtConstantForce, direction));
		pid::PidReport sc = h().builder().make(ReportType::Output, pid::use::SetConstantForceReport);
		sc.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, magnitude);
		h().send(sc);
		operation(idx, pid::use::OpEffectStart);
		return idx;
	}
	int64_t startSpring(int64_t coefficient) {
		const int64_t idx = newEffect(pid::use::EtSpring);
		h().send(makeSetEffect(idx, pid::use::EtSpring, 0));
		pid::PidReport cond = h().builder().make(ReportType::Output, pid::use::SetConditionReport);
		cond.set(pid::use::EffectBlockIndex, idx).set(pid::use::ParameterBlockOffset, 0).set(pid::use::CpOffset, 0);
		cond.set(pid::use::PositiveCoefficient, coefficient).set(pid::use::NegativeCoefficient, coefficient);
		cond.set(pid::use::PositiveSaturation, 32767).set(pid::use::NegativeSaturation, 32767).set(pid::use::DeadBand, 0);
		h().send(cond);
		operation(idx, pid::use::OpEffectStart);
		return idx;
	}

	/** Torque at the chip after `waitMs`, steady over the last 100 cycles (REQUIREd); same settling rule as test_f1_hid.cpp */
	int16_t settledTorque(uint32_t waitMs = 2500) {
		TorqueTrace trace(chip1);
		trace.start(*this);
		h().observe(&trace);
		h().run(waitMs);
		h().observe(nullptr);
		const uint32_t last = ffbCycles();
		REQUIRE(last > 100);
		return trace.steadyValue(last - 99, last);
	}

	/** The 0x64 word (torque << 16 | flux) after `waitMs`, steady over the last 100 cycles in both fields (REQUIREd) */
	uint32_t settledWord(uint32_t waitMs = 2500) {
		TorqueTrace trace(chip1);
		trace.start(*this);
		h().observe(&trace);
		h().run(waitMs);
		h().observe(nullptr);
		const uint32_t last = ffbCycles();
		REQUIRE(last > 100);
		const int16_t torque = trace.steadyValue(last - 99, last);
		int16_t flux = 0;
		std::string why;
		const bool steady = trace.steadyFluxQuery(last - 99, last, flux, why);
		INFO("flux not steady: " << why);
		REQUIRE(steady);
		return (static_cast<uint32_t>(static_cast<uint16_t>(torque)) << 16) | static_cast<uint16_t>(flux);
	}

	/** The five torque biquad coefficient words of the chip (CONFIG_ADDR 0x4E = 17, 18, 20, 21, 22 behind CONFIG_DATA 0x4D, TMC4671::setBiquadTorque) */
	std::vector<uint32_t> torqueBiquadWords() const {
		std::vector<uint32_t> w;
		for (uint32_t n : {17u, 18u, 20u, 21u, 22u}) {
			w.push_back(chip1.configWord(n));
		}
		return w;
	}

	// ---- event-log based (log must be emplaced) ----

	static bool nonZeroTorque(uint32_t data) { return static_cast<int16_t>(data >> 16) != 0; }

	/** Sign (+1 / -1) of the torque of the first non-zero write to 0x64 after `mark` (REQUIREs one); the log is complete */
	int firstTorqueSign(size_t mark) {
		const size_t i = log->firstWrite(kRegTorqueFluxTarget, nonZeroTorque, mark);
		REQUIRE(i != ChipEventLog::npos);
		return static_cast<int16_t>(log->events()[i].data >> 16) > 0 ? 1 : -1;
	}

	/** A constant force on a fresh effect: the sign of its first non-zero torque (direction 9000 = 90 degrees, magnitude 8000) */
	int constantSign() {
		const size_t mark = log->events().size();
		startConstant(9000, 8000);
		h().run(2500);
		return firstTorqueSign(mark);
	}

	/** Device Reset (PID 1.01 5.8: frees every effect) and waits until the chip target is back to 0 */
	void clearEffects() {
		h().send(h().builder().deviceControl(pid::use::DcDeviceReset));
		h().run(2500);
		const std::optional<uint32_t> last = log->valueAt(kRegTorqueFluxTarget, log->events().size() - 1);
		REQUIRE(last.has_value());
		REQUIRE(static_cast<int16_t>(*last >> 16) == 0);
	}
};

// Values set before the save, different from the fresh-flash defaults (axis.0.invert 1, power 5000, degrees 900, fxratio 204 ...)
const std::vector<std::pair<std::string, int64_t>> kAxisSettings = {
	{"axis.0.power", 7000},		{"axis.0.invert", 0},		 {"axis.0.degrees", 540},		{"axis.0.fxratio", 200},		 {"axis.0.filterProfile_id", 2},
	{"axis.0.esgain", 90},		{"axis.0.idlespring", 20},	 {"axis.0.axisdamper", 40},		{"axis.0.axisfriction", 15},	 {"axis.0.axisinertia", 25},
};
const std::vector<std::pair<std::string, int64_t>> kTmcSettings = {
	{"tmc.0.fluxoffset", 1500}, {"tmc.0.torqueP", 1234}, {"tmc.0.torqueI", 567}, {"tmc.0.fluxP", 111}, {"tmc.0.fluxI", 222},
	{"tmc.0.trqbq_mode", 1},	{"tmc.0.trqbq_f", 300},	 {"tmc.0.combineEncoder", 1}, {"tmc.0.invertForce", 1},
};
const std::vector<std::pair<std::string, int64_t>> kFxSettings = {
	{"fx.0.spring", 100},	  {"fx.0.damper", 90},		 {"fx.0.friction", 80},		  {"fx.0.inertia", 70},
	{"fx.0.filterCfFreq", 200}, {"fx.0.filterCfQ", 50},	 {"fx.0.filterProfile_id", 1}, {"fx.0.damper_f", 120},
	{"fx.0.damper_q", 25},	  {"fx.0.frictionPctSpeedToRampup", 7}, {"fx.0.reconFilterMode", 1},
};

} // namespace

TEST_SUITE("F1Persistence") {

// F1-P1 axis group. Axis::saveFlash / restoreFlash (Axis.cpp:333-366, :221-330): power (:262), degrees and invert share one word (:268-274),
// fxratio is the low byte of the endstop word, filter profile, idle spring, damper / friction / inertia intensities.
TEST_CASE_FIXTURE(PersistFixture, "F1-P1: the axis settings saved with sys.save read back unchanged after a reboot" * IT_DELETES_FFBHIDMAIN) {
	up();
	setAll(kAxisSettings);
	save();
	reboot(); // allowReplug = true: Running is needed to read the commands of a started chain; the replug is not under test
	REQUIRE(h().getValue("axis.0.drvtype?") == 1);
	checkAll(kAxisSettings);
}

// F1-P1 TMC group. TMC4671::saveFlash (TMC4671.cpp:108-131): PI gains, flux offset, torque filter mode and frequency, and the encoder / hall word
// with combineEncoder and invertForce; restoreFlash :147-192 and restoreEncHallMisc :2698-2722 (invertForce is kept only with combineEncoder).
// The chip registers 0x54 / 0x56 hold the restored gains (DS: P s16(31:16), I s16(15:0)).
TEST_CASE_FIXTURE(PersistFixture, "F1-P1: the TMC settings saved with sys.save read back unchanged after a reboot and the PI gains are in the chip" * IT_DELETES_FFBHIDMAIN) {
	up();
	setAll(kTmcSettings);
	save();
	reboot();
	checkAll(kTmcSettings);
	CHECK(chip1.reg(kRegTorquePI) == ((1234u << 16) | 567u));
	CHECK(chip1.reg(kRegFluxPI) == ((111u << 16) | 222u));
}

// F1-P1 torque filter Q. The saved word of the torque filter holds the frequency (bits 12:0) and the mode (bits 15:13): all 16 bits are used
// (TMC4671.cpp:121-122, :128), and CHANGELOG.md:136-137 (v1.10.0) says "fixed Q factor, saved frequency". So trqbq_q (TMC4671.cpp:3103-3106,
// help "Torque filter q*100") is a runtime-only setting: after a power cycle it is back to its default 50 while mode and frequency are as saved.
TEST_CASE_FIXTURE(PersistFixture, "characterization: [F1-P1] the torque filter Q set by command is back to its default after a reboot while mode and frequency survive" * IT_DELETES_FFBHIDMAIN) {
	// INR 35: q is not stored (CHANGELOG.md:136-137 "fixed Q factor, saved frequency"; the flash word is full, TMC4671.cpp:121-128).
	// NEEDS REVIEW: the changelog says Q is fixed, yet the trqbq_q command changes it at runtime.
	up();
	const int64_t defaultQ = h().getValue("tmc.0.trqbq_q?");
	setAll({{"tmc.0.trqbq_mode", 1}, {"tmc.0.trqbq_f", 300}, {"tmc.0.trqbq_q", 77}});
	REQUIRE(h().getValue("tmc.0.trqbq_q?") == 77);
	save();
	reboot();
	CHECK(h().getValue("tmc.0.trqbq_mode?") == 1);
	CHECK(h().getValue("tmc.0.trqbq_f?") == 300);
	const int64_t q = h().getValue("tmc.0.trqbq_q?");
	INFO("default q " << defaultQ << ", saved 77, after the reboot " << q);
	CHECK(defaultQ == 50);
	CHECK(q == defaultQ);
}

// F1-P1 effects group: EffectsCalculator::saveFlash / restoreFlash (EffectsCalculator.cpp:763-860). Biquads of the custom profile are stored only
// while the custom profile is active (:826): the profile is set to 1 (custom) before the save.
TEST_CASE_FIXTURE(PersistFixture, "F1-P1: the effects settings saved with sys.save read back unchanged after a reboot" * IT_DELETES_FFBHIDMAIN) {
	up();
	// fx.0.filterProfile_id=1 first: damper_f / damper_q then address the custom profile that reads back
	setAll({{"fx.0.filterProfile_id", 1}});
	std::vector<std::pair<std::string, int64_t>> rest(kFxSettings.begin(), kFxSettings.end());
	setAll(rest);
	save();
	reboot();
	checkAll(kFxSettings);
}

// F1-P1 torque. The same host sequence (a constant force and a spring, the position scripted to 0, +3000, -3000 counts, each held until the
// settling rule of test_f1_hid.cpp is met: 2500 ms, last 100 FFB cycles equal) gives the same settled 0x64 word (torque s16(31:16) and flux
// s16(15:0), DS 0x64) before and after the reboot. Both runs start 3000 ms after the settings are in force, so the 1 s start fade of the axis
// (Axis.cpp:447-449) is over. Only settings that act on the 0x64 target are claimed: power (limit and scale), invert, fxratio, degrees,
// fx.0 spring gain and constant force filter, fluxoffset (flux part). invertForce / combineEncoder are left out on purpose: with combineEncoder the position comes from the external encoder, not 0x6B, so the scripted position (and the spring) would have no effect; they are covered by the F1-P2 invertForce case. The torque PI and the torque biquad act
// inside the chip (checked separately: its biquad coefficients, registers 0x4D / 0x4E, equal the ones recorded before the reboot); idle spring,
// damper, friction, inertia, filter profiles and esgain do nothing at zero speed inside the endstop. The values are a pure function of the
// settings and the position (filters settle to their DC gain), so they are equal (no bound needed).
namespace {
std::vector<uint32_t> runSequence(PersistFixture &f) {
	f.script.setConstant(0);
	f.attachPositionScript(f.script);
	f.h().run(3000);
	f.startConstant(9000, 8000);
	f.startSpring(10000);
	std::vector<uint32_t> out;
	for (int32_t pos : {0, 3000, -3000}) {
		f.script.setConstant(pos);
		out.push_back(f.settledWord());
	}
	return out;
}
} // namespace

TEST_CASE_FIXTURE(PersistFixture, "F1-P1: after a reboot the same constant force and spring sequence gives the same 0x64 word as before the save and the torque biquad is written again" * IT_DELETES_FFBHIDMAIN) {
	up();
	setAll(kAxisSettings);
	setAll({{"tmc.0.torqueP", 1234}, {"tmc.0.torqueI", 567}, {"tmc.0.fluxoffset", 1500}, {"tmc.0.trqbq_mode", 1}, {"tmc.0.trqbq_f", 300}});
	setAll(kFxSettings);
	save();
	const std::vector<uint32_t> biquadBefore = torqueBiquadWords();
	CHECK(biquadBefore != std::vector<uint32_t>(5, 0u)); // a lowpass at 300 Hz has non-zero coefficients: the comparison below is not vacuous
	const std::vector<uint32_t> before = runSequence(*this);
	REQUIRE(before.size() == 3);
	MESSAGE("settled 0x64 word before the reboot: " << before[0] << ", " << before[1] << ", " << before[2]);
	CHECK((before[0] >> 16) != 0);
	CHECK(before[1] != before[0]);
	CHECK(before[2] != before[0]);
	host.reset();
	detachPositionScript();
	reboot();
	CHECK(torqueBiquadWords() == biquadBefore);
	const std::vector<uint32_t> after = runSequence(*this);
	REQUIRE(after.size() == 3);
	MESSAGE("settled 0x64 word after the reboot: " << after[0] << ", " << after[1] << ", " << after[2]);
	CHECK(after[0] == before[0]);
	CHECK(after[1] == before[1]);
	CHECK(after[2] == before[2]);
}

// invertForce is restored only with combineEncoder (restoreEncHallMisc, TMC4671.cpp:2720: `((val >> 7) & 1) && combineEncoder`), although the
// command accepts it without combineEncoder and the torque path inverts with it (TMC4671.cpp:1753). The combineEncoder command itself forces
// invertForce off when it is cleared (:2823), so the dependency looks intended; the documents do not say. Not a defect claim.
TEST_CASE_FIXTURE(PersistFixture, "characterization: [F1-P1] tmc.0.invertForce set without combineEncoder is not restored after a reboot" * IT_DELETES_FFBHIDMAIN) {
	// INR 34: the restore drops invertForce when combineEncoder is 0 (TMC4671.cpp:2720); NEEDS REVIEW whether the command should refuse it
	up();
	REQUIRE(h().getValue("tmc.0.combineEncoder?") == 0);
	set("tmc.0.invertForce=1");
	REQUIRE(h().getValue("tmc.0.invertForce?") == 1);
	save();
	reboot();
	CHECK(h().getValue("tmc.0.combineEncoder?") == 0);
	CHECK(h().getValue("tmc.0.invertForce?") == 0);
}

// F1-P2 power. DS 0x5E: the torque limit field is the largest torque the axis may command. The driver writes its default 30000 when it is
// created (TMC4671::setupDriver, IB 17) and restoreFlash sets the axis power afterwards (Axis.cpp:231 then :262-263), so at boot the limit is the
// power before the first torque reaches the chip. (IB 17 itself is the runtime drvtype selection, F1-X1; the order expected it here, the boot path
// does not show it.)
TEST_CASE_FIXTURE(PersistFixture, "F1-P2: after a reboot the restored power is in register 0x5E before the first non-zero torque" * IT_DELETES_FFBHIDMAIN) {
	log.emplace(*this, chip1);
	up();
	set("axis.0.power=7000");
	set("axis.0.fxratio=255");
	save();
	reboot();
	startConstant(9000, 8000);
	h().run(2500);
	REQUIRE(log->complete());
	const size_t first = log->firstWrite(kRegTorqueFluxTarget, PersistFixture::nonZeroTorque, 0);
	REQUIRE(first != ChipEventLog::npos);
	// the log restarted with the reboot (onReboot): the first non-zero torque is the one of the new start
	REQUIRE(log->countWrites(kRegTorqueFluxLimits, first) > 0);
	const std::optional<uint32_t> limit = log->valueAt(kRegTorqueFluxLimits, first);
	REQUIRE(limit.has_value());
	INFO("0x5E in force at the first non-zero torque: " << (*limit & 0xffffu) << ", saved power 7000");
	CHECK((*limit & 0xffffu) == 7000u);
}

// F1-P2 axis invert. The sign of the first non-zero torque after a start follows axis.0.invert (Axis.cpp:1067-1076, applied to the torque
// path); the saved value is in force from the first torque after the reboot.
TEST_CASE_FIXTURE(PersistFixture, "F1-P2: after a reboot the saved axis.0.invert acts on the first non-zero torque" * IT_DELETES_FFBHIDMAIN) {
	log.emplace(*this, chip1);
	up();
	set("axis.0.power=7000");
	set("axis.0.fxratio=255");
	REQUIRE(h().getValue("axis.0.invert?") == 1); // fresh flash default
	const int signDefault = constantSign();
	clearEffects();
	set("axis.0.invert=0");
	const int signSaved = constantSign();
	INFO("first torque sign with invert 1: " << signDefault << ", with invert 0: " << signSaved);
	REQUIRE(signSaved == -signDefault); // the setting acts on the first torque (precondition of the comparison)
	save();
	reboot();
	REQUIRE(h().getValue("axis.0.invert?") == 0);
	const int signAfter = constantSign();
	INFO("first torque sign after the reboot " << signAfter << ", expected the sign of the saved setting " << signSaved);
	CHECK(signAfter == signSaved);
}

// F1-P2 TMC invertForce (with combineEncoder, which is the condition of the restore, TMC4671.cpp:2720; the open-loop chain still runs: probed).
TEST_CASE_FIXTURE(PersistFixture, "F1-P2: after a reboot the saved tmc.0.invertForce acts on the first non-zero torque" * IT_DELETES_FFBHIDMAIN) {
	log.emplace(*this, chip1);
	up();
	set("axis.0.power=7000");
	set("axis.0.fxratio=255");
	set("tmc.0.combineEncoder=1");
	REQUIRE(h().getValue("tmc.0.invertForce?") == 0);
	const int signDefault = constantSign();
	clearEffects();
	set("tmc.0.invertForce=1");
	const int signSaved = constantSign();
	INFO("first torque sign with invertForce 0: " << signDefault << ", with invertForce 1: " << signSaved);
	REQUIRE(signSaved == -signDefault);
	save();
	reboot();
	REQUIRE(h().getValue("tmc.0.invertForce?") == 1);
	const int signAfter = constantSign();
	INFO("first torque sign after the reboot " << signAfter << ", expected the sign of the saved setting " << signSaved);
	CHECK(signAfter == signSaved);
}

// Default power cycle with a stored TMC configuration (decisions log 2026-11-24 item 3). A board with a saved, working configuration that is
// switched on with the PC connected must drive the motor again without re-plugging USB. Firmware: the driver is created at start, the USB mount
// (tud_mount_cb -> Axis::usbResume -> TMC4671::startMotor) comes while the state machine is still uninitialised, so startMotor() only records
// motorEnabledRequested (TMC4671.cpp:1682-1686 needs state Shutdown); the initialisation ends in Shutdown (:1574-1588) and nothing asks for
// Running again: PWM on, MODE_MOTION stopped, no torque. Same lost start request as IB 24 (startMotor outside Shutdown, TMC4671.cpp:1682-1687,
// and no consumer once Shutdown is reached), here at boot instead of after a supply drop. Boot entry path: the driver is uninitialised when the
// mount arrives; Shutdown is reached through initializeWithPower -> setEncoderType (TMC4671.cpp:1586-1588, the hall branch :1573 the same way).
// The case relies on the helper's cold-start model (rebootKeepingFlash: USB enumerates before the supply rises, see IntegrationFixture.h).
TEST_CASE_FIXTURE(PersistFixture, "F1-P1: after a power cycle with a stored configuration and the PC connected the motor is driven without a USB replug" * KNOWN_ISSUE("(IB 24) a start request made before the driver is ready is lost: after a default power cycle the driver stays in Shutdown (TMC4671.cpp:1682-1687) and no torque reaches the motor") * IT_DELETES_FFBHIDMAIN) {
	up();
	set("axis.0.power=7000");
	set("axis.0.fxratio=255");
	save();
	host.reset();
	rebootKeepingFlash(); // default: no replug, allowReplug = false
	// the helper records "Running not reached" as the fixture fault: it is the defect asserted below, not a model fault. Only that fault is cleared:
	// any other (kernel object leak, observer, ...) must stay visible
	const std::string fault = firstFault();
	INFO("fixture fault after the default reboot: " << fault);
	REQUIRE(fault.find("Running not reached") != std::string::npos);
	clearFixtureFault();
	host.emplace(*this);
	setSpiFrameUs(0);
	startConstant(9000, 8000);
	const int16_t torque = settledTorque();
	const uint32_t mode = chip1.reg(0x63) & 0xffu;
	MESSAGE("after the default power cycle: TMC state " << static_cast<uint32_t>(tmc()->getState()) << " (2 Shutdown, 3 Running), torque at 0x64 " << torque
			<< ", MODE_MOTION " << mode << ", PWM " << (chip1.reg(0x1A) & 0xffu));
	REQUIRE_CHAIN_MODELLED();
	CHECK(torque != 0);
	CHECK(tmc()->getState() == TMC_ControlState::Running);
}

} // TEST_SUITE
