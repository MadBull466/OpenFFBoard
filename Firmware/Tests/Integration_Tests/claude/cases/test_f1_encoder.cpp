/*
 * test_f1_encoder.cpp (WO I2-5)
 *
 * Requirement F1-T9 (charter F1_TMC4671_CHAIN_REQUIREMENTS.md, group G-T): with an external encoder (the local quadrature encoder
 * EncoderLocal on TIM3, 4 counts per line) instead of the integrated one, the TMCENC thread writes PHI_E_EXT (DS 0x1C) at the TIM6
 * rate, the axis uses the external position and the sign conventions of F1-X2 still hold.
 *
 * THE USER'S PATH (all through the command interface, no private call):
 *   "tmc.0.combineEncoder=1"  TMC4671.cpp:2820, hasIntegratedEncoder() false (TMC4671.cpp:1881): the axis may own an encoder
 *   "axis.0.enctype=2"        Axis.cpp:1123, Axis::setEncType creates the local encoder (EncoderSources.cpp:20) and hands it to the driver
 *   "localenc.0.cpr=<C>"      EncoderLocal.cpp:61 (counts per revolution, 4 counts per line: C = 4 * lines)
 *   "tmc.0.mtype=1"           DC motor (TMC4671.cpp:2037): see below
 *   "tmc.0.encsrc=5"          TMC4671.cpp:2866, setEncoderType(ext): needs TIM_TMC and the encoder, starts TIM6 (setUpExtEncTimer)
 * and the PC connects afterwards (mountAndRun, the board's start path), as in the bring-up.
 * TWO PATHS, because the motor type decides what the TMC does with PHI_E_EXT:
 *  - DC motor ("tmc.0.mtype=1", one pole pair, TMC4671.cpp:2042-2044): encoderInit's alignment (bangInitEnc, checkEncoder) returns at once
 *    (TMC4671.cpp:976, 1183) and the chain reaches Running. DS: for a DC motor the transformations are bypassed, so PHI_E_EXT does not
 *    act on the torque. The DC cases prove what does not depend on the commutation: the encoder position reaches the axis, the TMCENC
 *    rate, the sign of the torque at 0x64 for spring and damper, the angle's own scaling at one pole pair.
 *  - BLDC motor ("tmc.0.mtype=3", "tmc.0.poles=4"): the state machine runs the alignment (TMC4671.cpp:600-603, 975-1045, 1182-1300):
 *    the field is turned through PHI_E_EXT and the ENCODER must follow, otherwise HardError (TMC4671.cpp:1236-1241). The BLDC cases prove
 *    the pole-pair scaling, the agreement of the PHI_E_EXT sign and the 0x64 torque sign, and the alignment offset.
 *    FOLLOWER (justification): the rotor of a motor locked to the field follows the commanded electrical angle. A chip write hook on 0x1C
 *    sets TIM3 (HostTarget::timerSetCounter, the fake's public API) to the mechanical position that angle means, counts = phi_e * C /
 *    (65535 * poles): a position script that follows the alignment angle, NOT a torque plant (no inertia, no torque integration). The
 *    same pattern as the coverage ExtPlant (test_tmc4671_encoder.cpp:154-165). It is active only while the chip is brought up (until
 *    Running); afterwards the cases move TIM3 themselves.
 *    KNOWN FAULT, bug 165 (coverage): bangInitEnc calls updateReg(phiEoffsetReg = 0, ...) for an external encoder (TMC4671.cpp:1005), a
 *    write to the read-only register 0x0, which FakeTmc4671 reports as a violation. The BLDC cases declare exactly that one violation
 *    (expectViolation(WriteToReadOnly, 0)); any other fault still fails them. One KNOWN_ISSUE case asserts the intended behaviour.
 *
 * ORACLES
 *  - TMC4671-LA datasheet Rev 2.07: 360 deg = 2^16 = 65536 positions for every angle (p. 16 and p. 80 "normalized to 2^16"); register
 *    0x1C PHI_E_EXT s16(15:0), an electrical angle written by the MCU when the encoder is connected to the MCU.
 *  - MATH of position: the encoder reports d counts of C per revolution as d / C revolutions (Encoder::getPos_f); Axis::scaleEncValue
 *    (Axis.cpp:587-596) gives curpos = (0xffff / D) * (360 * d / C) for the range D = axis.0.degrees (angle in degrees, axis.0.invert 0,
 *    gear ratio 1). So for D = 360: curpos = 65535 d / C. Tolerance 2 counts for the float truncation, as F1-X3.
 *  - MATH of the electrical angle: with one pole pair one mechanical revolution is one electrical revolution, so d counts give
 *    phi_e = d * 65536 / C modulo 65536 (a mechanical quarter turn is 16384 = 0x4000).
 *  - Signs: PID 1.01 and the convention of F1-X2 (test_f1_axis.cpp): a spring is restoring and a damper opposes the motion at 0x64.
 *
 * RATE: TIM6 gets ARR = TIM_TMC_ARR = 250 and PSC = (SystemCoreClock / 2) / 1e6 + 1 = 85 (TMC4671.cpp:3156-3158, target_constants.h:94-96,
 * "4khz"); the period is (250 + 1) * (85 + 1) = 21586 timer clocks of 84 MHz = 257 us = 3891 Hz (the intended 4 kHz is IB 2, F1-A4).
 */
#include "doctest.h"

#include "ChipEventLog.h"
#include "FfbHost.h"
#include "HidReportDescriptor.h"
#include "IntegrationFixture.h"
#include "PidReportBuilder.h"
#include "TestHelpers.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using hid::ReportType;

namespace {

constexpr uint32_t kDirectionX = hid::usage(0x0A, 1);
constexpr int32_t kCpr = 4000; // 1000 lines, 4 counts per line

struct Window {
	int16_t lo = 0;
	int16_t hi = 0;
	size_t n = 0;
	bool positive() const { return n > 0 && lo > 0; }
	bool negative() const { return n > 0 && hi < 0; }
};

struct EncFixture : IntegrationFixture {
	std::optional<FfbHost> host;
	FfbHost &h() { return *host; }

	/** The user's path of the file comment, then Running with the external encoder; SPI frames take no time afterwards */
	void up(int32_t cpr = kCpr) {
		reachShutdown();
		cdcCommand("tmc.0.combineEncoder=1");
		cdcCommand("axis.0.enctype=2");
		cdcCommand("localenc.0.cpr=" + std::to_string(cpr));
		cdcCommand("tmc.0.mtype=1");
		cdcCommand("tmc.0.encsrc=5");
		mountAndRun();
		setSpiFrameUs(0);
		host.emplace(*this);
	}

	static constexpr int kPoles = 4;
	bool follow = false; //!< the follower of the file comment is active

	/** BLDC with kPoles pole pairs: the user's path of the file comment, the alignment with the follower; tolerates bug 165 when asked */
	void upBldc(bool tolerateBug165 = true) {
		reachShutdown();
		cdcCommand("tmc.0.combineEncoder=1");
		cdcCommand("axis.0.enctype=2");
		cdcCommand("localenc.0.cpr=" + std::to_string(kCpr));
		cdcCommand("tmc.0.mtype=3");
		cdcCommand("tmc.0.poles=" + std::to_string(kPoles));
		follow = true;
		chip1.onWrite(0x1C, [this](FakeTmc4671 &, uint32_t d) {
			if (!follow) {
				return;
			}
			const double phi = static_cast<double>(static_cast<int16_t>(d & 0xFFFF));
			const double counts = phi * kCpr / (65535.0 * kPoles);
			HostTarget::timerSetCounter(&htim3, static_cast<uint32_t>(0x7fff + std::lround(counts)));
		});
		if (tolerateBug165) {
			chip1.expectViolation(FakeTmc4671::ViolationKind::WriteToReadOnly, 0); // bug 165, TMC4671.cpp:1005
		}
		cdcCommand("tmc.0.encsrc=5");
		mountAndRun();
		follow = false;
		setSpiFrameUs(0);
		host.emplace(*this);
	}

	void configure(int64_t power, int64_t degrees) {
		h().command("axis.0.power=" + std::to_string(power));
		h().command("axis.0.fxratio=255");
		h().command("axis.0.degrees=" + std::to_string(degrees));
		h().command("axis.0.invert=0");
		h().command("axis.0.idlespring=0");
	}

	/** Quadrature edges on TIM3 (the fake's public API), then 50 ms so the FFB cycle and the TMCENC thread see them */
	void move(int32_t counts) {
		HostTarget::encoderTimerCount(&htim3, counts);
		h().run(50);
	}

	int64_t curpos() { return h().getValue("axis.0.curpos?"); }

	/** Expected curpos of the file comment for d counts, range D degrees */
	static double expectedCurpos(int32_t d, int64_t degrees) { return (65535.0 / static_cast<double>(degrees)) * (360.0 * d / kCpr); }

	/** 0x1C as the TMCENC thread left it after the last `move` (the chip register; the async write completes in the scheduler step) */
	uint16_t phiEExt() const { return static_cast<uint16_t>(chip1.reg(0x1C) & 0xffffu); }

	/** Distance of two 16 bit angles on the circle */
	static int circDist(uint16_t a, uint16_t b) {
		const int d = (static_cast<int>(a) - static_cast<int>(b)) & 0xffff;
		return std::min(d, 0x10000 - d);
	}

	pid::PidReport makeSetEffect(int64_t index, uint32_t typeUsage, int64_t direction) {
		pid::PidReport se = h().builder().make(ReportType::Output, pid::use::SetEffectReport);
		se.set(pid::use::EffectBlockIndex, index).select(typeUsage).set(pid::use::Duration, 32767).set(pid::use::Gain, 255);
		se.flag(pid::use::DirectionEnable, true);
		se.setIn({pid::use::Direction}, kDirectionX, direction);
		return se;
	}
	void startCondition(uint32_t typeUsage, int64_t coefficient) {
		const FfbHost::CreateResult c = h().createEffect(typeUsage);
		REQUIRE(c.load.success());
		const int64_t idx = c.load.effectBlockIndex;
		h().send(makeSetEffect(idx, typeUsage, 0));
		pid::PidReport cond = h().builder().make(ReportType::Output, pid::use::SetConditionReport);
		cond.set(pid::use::EffectBlockIndex, idx).set(pid::use::ParameterBlockOffset, 0).set(pid::use::CpOffset, 0);
		cond.set(pid::use::PositiveCoefficient, coefficient).set(pid::use::NegativeCoefficient, coefficient);
		cond.set(pid::use::PositiveSaturation, 32767).set(pid::use::NegativeSaturation, 32767).set(pid::use::DeadBand, 0);
		h().send(cond);
		pid::PidReport eo = h().builder().make(ReportType::Output, pid::use::EffectOperationReport);
		eo.set(pid::use::EffectBlockIndex, idx).select(pid::use::OpEffectStart).set(pid::use::LoopCount, 1);
		h().send(eo);
	}

	/** Torque at 0x64 of the `windowMs` that follow `settleMs` */
	Window window(uint32_t settleMs, uint32_t windowMs = 100) {
		h().run(settleMs);
		TorqueTrace trace(chip1);
		trace.start(*this);
		h().observe(&trace);
		h().run(windowMs);
		h().observe(nullptr);
		Window w;
		w.n = trace.samples().size();
		if (w.n == 0) {
			return w;
		}
		w.lo = w.hi = trace.samples().front().torque;
		for (const TorqueTrace::Sample &s : trace.samples()) {
			w.lo = std::min(w.lo, s.torque);
			w.hi = std::max(w.hi, s.torque);
		}
		return w;
	}
	std::string describe(const Window &w) const {
		std::ostringstream o;
		o << "[" << w.lo << ".." << w.hi << "] over " << w.n << " cycles";
		return o.str();
	}
};

} // namespace

TEST_SUITE("F1Encoder") {

// ---------------------------------------------------------------------------------------------------------------------
// Selection
// ---------------------------------------------------------------------------------------------------------------------

// The user's sequence ends in Running with the external encoder: the source reads back as 5, the axis reports the encoder's cpr, TIM6 runs
// with its update interrupt (the TMCENC thread is then notified at every update).
TEST_CASE_FIXTURE(EncFixture, "F1-T9: the commands for the local encoder and the external-encoder source bring the driver to Running with TIM6 started" * IT_DELETES_FFBHIDMAIN) {
	up();
	REQUIRE(tmc()->getState() == TMC_ControlState::Running);
	CHECK(h().getValue("tmc.0.encsrc?") == 5);
	CHECK(h().getValue("axis.0.cpr?") == kCpr);
	CHECK(h().getValue("axis.0.enctype?") == 2);
	CHECK(HostTarget::timerRunning(&htim6));
	CHECK(HostTarget::timerUpdateIrqEnabled(&htim6));
	CHECK(htim6.Instance->ARR == 250);
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// Position
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(EncFixture, "F1-T9: a known TIM3 count change gives the proportional axis.0.curpos for D = 360 and D = 720, with odd symmetry" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 360);
	for (int64_t deg : {360, 720}) {
		h().command("axis.0.degrees=" + std::to_string(deg));
		int32_t at = 0; // counts moved so far (curpos is the position since the encoder was created)
		for (int32_t target : {0, 1000, -1000, 2500, -2500, 3900, -3900, 1}) {
			move(target - at);
			at = target;
			const int64_t pos = curpos();
			const double expected = expectedCurpos(target, deg);
			INFO("degrees " << deg << ", counts " << target << " -> curpos " << pos << ", proportional value " << expected);
			CHECK_NEAR(pos, expected, 2);
		}
		move(0 - at);
	}
	REQUIRE_CHAIN_MODELLED();
}

// The counter of TIM3 wraps at ARR (16 bit): 40000 counts from the centre value 0x7fff cross the overflow once, in each direction. The
// position must not jump (EncoderLocal::overflowCallback, EncoderLocal.cpp:119-126). The range is +-D/2 (endstop beyond it): D = 10000
// keeps the 3600 degrees inside +-5000, no endstop.
TEST_CASE_FIXTURE(EncFixture, "F1-T9: a movement across the overflow of the TIM3 counter keeps axis.0.curpos proportional in both directions" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 10000);
	int32_t at = 0;
	for (int32_t target : {40000, 0, -40000, 0}) {
		move(target - at);
		at = target;
		const int64_t pos = curpos();
		const double expected = expectedCurpos(target, 10000);
		INFO("counts " << target << " -> curpos " << pos << ", proportional value " << expected);
		CHECK_NEAR(pos, expected, 3);
	}
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// PHI_E_EXT: rate
// ---------------------------------------------------------------------------------------------------------------------

namespace {
struct RateRun {
	uint32_t updates = 0, activations = 0;
	size_t writes = 0;
};
} // namespace

// Every TIM6 update notifies TMCENC once (updateFromIsr -> NotifyFromISR, TMC4671.cpp:3188-3191) and every activation writes 0x1C
// (TMC4671.cpp:3173-3182, writeRegAsync): the writes in a window equal the updates, whatever the rate is. (The completion of an async
// write runs from the test thread in the scheduler: ChipEventLog header.)
TEST_CASE_FIXTURE(EncFixture, "F1-T9: the TMCENC thread writes PHI_E_EXT once for every TIM6 update" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 360);
	ChipEventLog log(*this, chip1);
	h().run(20);
	const size_t w0 = log.countWrites(0x1C);
	const uint32_t u0 = sched.stats().tim6Updates, a0 = sched.stats().tmcencActivations;
	h().run(1000);
	const uint32_t updates = sched.stats().tim6Updates - u0, acts = sched.stats().tmcencActivations - a0;
	const size_t writes = log.countWrites(0x1C) - w0;
	INFO("updates " << updates << ", TMCENC activations " << acts << ", writes to 0x1C " << writes);
	REQUIRE(updates > 0);
	CHECK(acts == updates);
	CHECK(writes == updates);
	REQUIRE(log.complete());
	REQUIRE_CHAIN_MODELLED();
}

// IB 2 (F1-A4): the target header says 4 kHz (TIM_TMC_ARR 250 "4khz"), the registers give 3891 Hz.
TEST_CASE_FIXTURE(EncFixture, "F1-T9: PHI_E_EXT is written at 4 kHz, 4000 times in 1000 ms" * KNOWN_ISSUE("(IB 2) the external encoder timer runs at 3891 Hz instead of 4 kHz (TMC4671.cpp:3156-3158)") * IT_DELETES_FFBHIDMAIN) {
	up();
	ChipEventLog log(*this, chip1);
	h().run(20);
	const size_t w0 = log.countWrites(0x1C);
	h().run(1000);
	const size_t writes = log.countWrites(0x1C) - w0;
	INFO("writes " << writes);
	REQUIRE_CHAIN_MODELLED();
	CHECK_NEAR(static_cast<double>(writes), 4000.0, 1);
}

// ---------------------------------------------------------------------------------------------------------------------
// PHI_E_EXT: value
// ---------------------------------------------------------------------------------------------------------------------

// DC, one pole pair: d counts of C = 4000 give phi_e = d * 65536 / 4000 mod 65536; the exact mirror property of the angle.
TEST_CASE_FIXTURE(EncFixture, "F1-T9: PHI_E_EXT is odd in the encoder position, the same count change in the other direction gives the negated angle" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 360);
	for (int32_t d : {500, 1250, 3999}) {
		move(d);
		const uint16_t plus = phiEExt();
		move(-2 * d);
		const uint16_t minus = phiEExt();
		INFO("+" << d << " -> " << plus << ", -" << d << " -> " << minus);
		CHECK(static_cast<uint16_t>(0x10000 - plus) == minus);
		move(d);
	}
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// BLDC: PHI_E_EXT acts on the commutation (file comment)
// ---------------------------------------------------------------------------------------------------------------------

namespace {
struct Bldc : EncFixture {
	/** int16 value of PHI_E_EXT (DS 0x1C s16) */
	int16_t phiS() const { return static_cast<int16_t>(phiEExt()); }
	/** (a - b) mod 65536 */
	static uint16_t sub(uint16_t a, uint16_t b) { return static_cast<uint16_t>(a - b); }
	static uint16_t idealDelta(int32_t d) { return static_cast<uint16_t>((static_cast<int64_t>(d) * kPoles * 65536 / kCpr) & 0xffff); }
};
} // namespace

// Bug 165 (coverage): bangInitEnc's updateReg(phiEoffsetReg = 0) writes the read-only register 0x0 (TMC4671.cpp:1005) for an external
// encoder; the external-encoder offset is kept in a member, no chip register is meant (TMC4671.cpp:1039-1041). Intended: no write to a
// read-only register during the alignment.
TEST_CASE_FIXTURE(Bldc, "F1-T9: the BLDC external-encoder alignment writes no read-only register of the chip" * KNOWN_ISSUE("(IB 50; same as bug 165) bangInitEnc calls updateReg(phiEoffsetReg = 0, ...) for the external encoder: a write to the read-only register 0x0 (TMC4671.cpp:1005)") * IT_DELETES_FFBHIDMAIN) {
	upBldc(false);
	size_t toReadOnly = 0;
	for (const FakeTmc4671::Violation &v : chip1.violations()) {
		if (v.kind == FakeTmc4671::ViolationKind::WriteToReadOnly && v.addr == 0) {
			toReadOnly++;
		}
	}
	INFO("writes to read-only register 0x0: " << toReadOnly);
	CHECK(toReadOnly == 0);
	chip1.expectViolation(FakeTmc4671::ViolationKind::WriteToReadOnly, 0); // declared after the check so teardown reports nothing else
}

TEST_CASE_FIXTURE(Bldc, "F1-T9: a BLDC motor with four pole pairs reaches Running with the external encoder after the alignment" * IT_DELETES_FFBHIDMAIN) {
	upBldc();
	REQUIRE(tmc()->getState() == TMC_ControlState::Running);
	CHECK(h().getValue("tmc.0.mtype?") == 3);
	CHECK(h().getValue("tmc.0.poles?") == kPoles);
	CHECK(h().getValue("tmc.0.encsrc?") == 5);
	REQUIRE_CHAIN_MODELLED();
}

// MATH: poles pole pairs, d counts of C: mechanical angle d / C revolutions, electrical angle poles * d / C revolutions, so
// phi_e(d) - phi_e(0) = poles * d * 65536 / C mod 65536 (DS: 65536 per electrical revolution), 65.536 per count for 4 pole pairs and
// C = 4000. The firmware truncates d * 0xffff / C and then multiplies by the poles (TMC4671.cpp:953-960): the quantisation is at most
// poles counts, plus one of rounding: tolerance poles + 1. d = 1000 is a quarter mechanical turn = one electrical revolution = 0 mod 65536.
// phi_e(0) is the alignment offset (see below), subtracted.
TEST_CASE_FIXTURE(Bldc, "F1-T9: with four pole pairs PHI_E_EXT advances four electrical revolutions per mechanical revolution" * IT_DELETES_FFBHIDMAIN) {
	upBldc();
	configure(5000, 10000);
	move(0);
	const uint16_t base = phiEExt();
	for (int32_t d : {100, 250, -300, 500, 1000, 1001, -1000}) {
		HostTarget::timerSetCounter(&htim3, static_cast<uint32_t>(0x7fff + d));
		h().run(50);
		const uint16_t got = sub(phiEExt(), base);
		const uint16_t ideal = idealDelta(d);
		INFO("counts " << d << " -> PHI_E_EXT - offset " << got << ", ideal " << ideal);
		CHECK(circDist(got, ideal) <= kPoles + 1);
	}
	REQUIRE_CHAIN_MODELLED();
}

// DS: 360 deg = 65536. With p pole pairs one mechanical revolution is p electrical revolutions, so after whole revolutions the angle is
// the starting angle again. The firmware scales by 0xffff instead of 65536 (TMC4671.cpp:953-960: phiE_t = pos * 0xffff, / cpr, then *
// pole_pairs): every mechanical revolution loses pole_pairs counts (4 per revolution here, 20 after 5), the same family as IB 16 for the
// integrated encoder. Coverage pins the same scaling with characterizations: test_tmc4671_encoder.cpp:712 (0x6664 for pos 1000, cpr
// 10000, 4 pole pairs), :733 (0xFFFC for pos 2500, NEEDS REVIEW 369) and the ExtPlant at :160 (65535 * poles in its inverse).
TEST_CASE_FIXTURE(Bldc, "F1-T9: PHI_E_EXT returns to its starting value after five full revolutions of the encoder" * KNOWN_ISSUE("(IB 49; related to IB 16) the external-encoder angle is scaled by 0xffff, not 65536: pole_pairs counts are lost per mechanical revolution (TMC4671.cpp:953-960)") * IT_DELETES_FFBHIDMAIN) {
	upBldc();
	configure(5000, 10000);
	move(0);
	const uint16_t start = phiEExt();
	move(5 * kCpr);
	const uint16_t after = phiEExt();
	INFO("start " << start << ", after 5 revolutions " << after << " (ideal: equal)");
	REQUIRE_CHAIN_MODELLED();
	CHECK(after == start);
}

// MATH of the sign agreement. A positive torque (0x64 torque field) drives the field, and so the rotor, toward increasing phi_e (FOC
// convention). The encoder counts with phi_e (follower = the alignment's own picture), so a positive torque raises the count for
// encdir 0; with encdir 1 the angle is negated (TMC4671.cpp:954-956) and the torque flips (TMC4671.cpp:1753). Either way the motion that
// a positive torque causes is "toward increasing int16(phi_e)" of the angle written to the chip. A restoring spring needs a motion back
// toward the centre, i.e. toward phi_e = 0 (the displacement d and phi(d) have the same sign for |phi| < 32768: here |phi| = 19660):
// sign(torque at 0x64) = - sign(int16(PHI_E_EXT)), for displacements of both signs and for encdir 0 and 1. A disagreement would make
// the spring push the rotor away from the centre.
TEST_CASE_FIXTURE(Bldc, "F1-T9: the 0x64 torque sign is opposite to the sign of PHI_E_EXT for a spring, both displacements, encdir 0 and 1" * IT_DELETES_FFBHIDMAIN) {
	upBldc();
	configure(5000, 900);
	startCondition(pid::use::EtSpring, 10000);
	cdcCommand("sys.0.debug=1"); // tmc.0.encdir is a debug command; IntegrationFixture::resetAll clears the flag
	for (int dir : {0, 1}) {
		for (int32_t d : {300, -300}) {
			// back to the centre first: a change of the direction alone does not rewrite 0x64 while the axis torque is unchanged
			// (Axis.cpp:488-495)
			HostTarget::timerSetCounter(&htim3, 0x7fff);
			h().run(200);
			cdcCommand("tmc.0.encdir=" + std::to_string(dir));
			REQUIRE(h().getValue("tmc.0.encdir?") == dir);
			move(d);
			const Window w = window(2500);
			const int phi = phiS();
			INFO("encdir " << dir << ", displacement " << d << " counts: int16(PHI_E_EXT) " << phi << ", torque " << describe(w));
			REQUIRE(std::abs(phi) > 1000);
			if (phi > 0) {
				CHECK(w.negative());
			} else {
				CHECK(w.positive());
			}
		}
	}
	REQUIRE_CHAIN_MODELLED();
}

// The mirror of the angle with encdir 1: int16(phi) of +d is the negation of encdir 0's (the commutation follows the reversed counting).
TEST_CASE_FIXTURE(Bldc, "F1-T9: tmc.0.encdir=1 negates the BLDC electrical angle for the same encoder position" * IT_DELETES_FFBHIDMAIN) {
	upBldc();
	configure(5000, 900);
	cdcCommand("sys.0.debug=1");
	move(0);
	const uint16_t offset = phiEExt(); // the alignment offset is part of every angle and is not negated (TMC4671.cpp:959-960)
	move(300);
	const uint16_t normal = sub(phiEExt(), offset);
	cdcCommand("tmc.0.encdir=1");
	REQUIRE(h().getValue("tmc.0.encdir?") == 1);
	h().run(50);
	const uint16_t reversed = sub(phiEExt(), offset);
	INFO("PHI_E_EXT - offset " << normal << " -> " << reversed << " after encdir=1 (negated: " << (0x10000 - normal) << ")");
	CHECK(circDist(reversed, static_cast<uint16_t>(0x10000 - normal)) <= 1);
	REQUIRE_CHAIN_MODELLED();
}

// Alignment offset (TMC4671.cpp:1026-1041, 959-960): bangInitEnc stores phiEpos - phi_enc in externalEncoderPhieOffset (0 is replaced by 1) and
// every angle written afterwards includes it. Plain: the offset is the same at every position, i.e. phi_e(d) - scaled(d) is constant.
// The follower makes the encoder follow the field without a mounting offset, so the firmware's value is only the quantisation of that
// picture (INR 52): characterization of the value.
TEST_CASE_FIXTURE(Bldc, "F1-T9: the alignment offset is added to PHI_E_EXT at every position" * IT_DELETES_FFBHIDMAIN) {
	upBldc();
	configure(5000, 10000);
	HostTarget::timerSetCounter(&htim3, 0x7fff);
	h().run(50);
	const uint16_t base = phiEExt();
	for (int32_t d : {-300, 7, 400}) {
		HostTarget::timerSetCounter(&htim3, static_cast<uint32_t>(0x7fff + d));
		h().run(50);
		const uint16_t offsetAt = sub(phiEExt(), static_cast<uint16_t>(static_cast<int64_t>(d) * 65535 / kCpr * kPoles));
		INFO("counts " << d << " -> offset " << offsetAt << ", offset at 0 " << base);
		CHECK(offsetAt == base);
	}
	REQUIRE_CHAIN_MODELLED();
}

TEST_CASE_FIXTURE(Bldc, "characterization: [F1-T9] the alignment offset against a follower rotor is the firmware's quantisation value" * IT_DELETES_FFBHIDMAIN) {
	upBldc();
	HostTarget::timerSetCounter(&htim3, 0x7fff);
	h().run(50);
	// INR 52: the follower puts the encoder exactly on the commanded angle, so phiEpos - phi_enc is 0, which the firmware replaces by 1
	// ("0 invalid", TMC4671.cpp:1029-1031): the offset of the angle at count 0 is 1. The DS defines no value for it
	INFO("PHI_E_EXT at count 0 (the offset): " << phiEExt());
	CHECK(phiEExt() == 1);
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// Signs (F1-X2 conventions) with the external position
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(EncFixture, "F1-T9: a spring is restoring at the chip with the external position, axis.0.invert 0 and 1" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 900);
	startCondition(pid::use::EtSpring, 10000);
	int32_t at = 0;
	for (int inv : {0, 1}) {
		h().command("axis.0.invert=" + std::to_string(inv));
		for (int32_t d : {1000, -1000}) { // +-90 degrees of a D = 900 range: no endstop
			move(d - at);
			at = d;
			const Window w = window(2500);
			// F1-X2 convention: the torque at the chip opposes the displacement of the chip's position, for both settings of
			// axis.invert (it flips the measured position, Axis.cpp:604-606, and the torque, so the two flips cancel)
			const bool expectNegative = d > 0;
			INFO("axis.invert=" << inv << ", displacement " << d << " counts -> torque " << describe(w));
			if (expectNegative) {
				CHECK(w.negative());
			} else {
				CHECK(w.positive());
			}
		}
	}
	REQUIRE_CHAIN_MODELLED();
}

namespace {
struct Rotating : EncFixture {
	/** The encoder turns `perMs` counts every scheduler step (the fake's public API, once per step in the plant slot) */
	void rotate(int32_t perMs) {
		if (perMs == 0) {
			sched.plantHook = nullptr;
			return;
		}
		sched.plantHook = [perMs](uint64_t) { HostTarget::encoderTimerCount(&htim3, perMs); };
	}
	~Rotating() override { sched.plantHook = nullptr; }
};
} // namespace

// Damper: the encoder turns at +-2 counts / ms = 180 degrees / s. D = 1800 keeps the 2600 ms (468 degrees) inside the range.
TEST_CASE_FIXTURE(Rotating, "F1-T9: a damper opposes the motion at the chip with the external position, both directions" * IT_DELETES_FFBHIDMAIN) {
	up();
	configure(5000, 1800);
	startCondition(pid::use::EtDamper, 10000);
	for (int32_t v : {2, -2}) {
		rotate(v);
		const Window w = window(2500);
		rotate(0);
		INFO("speed " << v << " counts/ms -> torque " << describe(w));
		if (v > 0) {
			CHECK(w.negative());
		} else {
			CHECK(w.positive());
		}
		move(-v * 2600); // back near the centre (D = 1800: the next run stays inside the range)
	}
	REQUIRE_CHAIN_MODELLED();
}

} // TEST_SUITE
