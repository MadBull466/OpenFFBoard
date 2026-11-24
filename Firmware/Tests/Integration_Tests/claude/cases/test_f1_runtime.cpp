/*
 * F1 runtime events during a running effect (WO I2-4): F1-T6 TMC commands, F1-T7 USB state, F1-T8 SPI faults.
 *
 * Chain: PID output reports over the real USB stack (FfbHost) -> HidFFB -> EffectsCalculator -> Axis -> TMC4671 driver -> frames of the
 * fake chip (FakeTmc4671, logged by ChipEventLog). The CDC commands of the user path ("tmc.0.<command>=<value>") are sent with
 * FfbHost::command / ChipEventLog::runCommandOnWrite.
 *
 * LIMIT (decided by the lead, decisions log 2026-11-24 item 1, plan section 7): ChipEventLog::runCommandOnWrite sends the command at the
 * end of the scheduler step of the trigger write, so a command always runs BETWEEN two activations of the writing thread (for the FFB
 * thread: between two FFB cycles, with the SPI semaphore free), never between two frames of one activation. The race of the 0x63
 * read-modify-write of `tmc.0.seqpi` (TMC4671::setSequentialPI, TMC4671.cpp:2011-2014: updateReg = R63 + W63) against the firmware's own
 * read-modify-write of 0x63 (setMotionMode) is therefore NOT producible on the cooperative host and is not claimed as tested by any case
 * of this file. What the "stream" cases prove is the weaker property the host can show: the command's frames sit between two FFB cycles,
 * the 0x64 stream before and after is unbroken.
 *
 * Oracles
 *  - TMC4671-LA datasheet Rev 2.07 (Tests/Code_Coverage/shared/specs/TMC4671-LA_datasheet_Rev2.07.pdf): 0x54 / 0x56 / 0x58 / 0x5A
 *    (PID_xxx_P_xxx_I: I s16 [15:0], P s16 [31:16], DS p96), 0x5D PIDOUT_UQ_UD_LIMITS (s16 [15:0], 0..32767, p96-97), 0x63 (MODE_MOTION
 *    u8 [7:0], MODE_PID_TYPE bit 31: 0 parallel / 1 sequential PI, p97-98), 0x64 (flux s16 [15:0], torque s16 [31:16], p98), 0x1A (PWM_CHOP
 *    [7:0], 0 = off), 0x4D CONFIG_DATA / 0x4E CONFIG_ADDR (torque biquad: CONFIG_ADDR 17 a1, 18 a2, 20 b0, 21 b1, 22 b2, 23 enable; each
 *    CONFIG_DATA write follows its CONFIG_ADDR write; enable range 0..1 in the register table, section 4.8.1 eq. 33: Y(n) = b0 X(n) + b1
 *    X(n-1) + b2 X(n-2) + a1 Y(n-1) + a2 Y(n-2), all s32 Q3.29, so the DC gain of a stored filter is (b0 + b1 + b2) / (2^29 - a1 - a2): 1 for
 *    a low pass). SPEC AMBIGUITY (INR 51, question IQ 15 to the owner): the DS writes "bit(31)" with range 0..0x1 for both MODE_PID_TYPE of 0x63
 *    and biquad_t_enable. The literal reading is bit 31 of CONFIG_DATA; the firmware writes `bq.enable & 0x1`, i.e. bit 0 (TMC4671.cpp:2271), and
 *    under the literal reading the filter would never be enabled on the chip. Until it is answered, every assertion on the VALUE of the enable
 *    (the case `characterization: [F1-T6] ... enable`) is a characterization; the assertions of enable 0 (disabled under either reading),
 *    the CONFIG_ADDR order, the coefficients and the DC gain stay plain.
 *  - Behaviour fixed only by the firmware (scaling of fluxoffset, what a command does to the next torque, what a GET does) is
 *    `characterization:` with `// INR <n>`.
 *  - USB: a bus reset / unplug ends the session (USB 2.0 9.1.1); a bus suspend is documented by the firmware itself as stopping FFB
 *    (global_callbacks.cpp:362-366 "Called on usb disconnect and suspend", FFBHIDMain::usbSuspend "Deactivates FFB", FFBHIDMain.cpp:303-311).
 *  - Spring: PID 1.01 5.3 (force = A (q - q0): linear in the position).
 *
 * Settings of every case (power first, IB 7 / bug 23): axis.0.power 5000 (or 10000 for the spring), axis.0.fxratio 255, idlespring 0 (no
 * torque without an effect), axis.0.invert left at the firmware default (all torque assertions are relative to a torque read at the chip
 * or a pair of torques), a settle time of 2500 ms after the settings (the fade-in of Axis::updateTorque, see test_f1_effects.cpp).
 *
 * Why the force changes in the interleaving cases: Axis::updateDriveTorque calls TMC4671::turn() only when the torque changed
 * (Axis.cpp:480-495, torqueChanged = metric.current.torque != metric.previous.torque, :921). A constant force produces one 0x64 write and
 * then none, so the cases that need a stream of 0x64 writes use a slow ramp force (RampStart -32767 .. RampEnd 32767 over 30 s: one
 * write every few cycles) and the cases of what a command does to the written value change the magnitude after the command.
 *
 * F1-T8 and HAL errors: the two HAL_ERROR cases cannot fail with the current firmware. The fake applies the data on HAL_ERROR
 * (ChipEventLog.h:101-108), so they only catch an implementation that substitutes a value when the status is not OK.
 *
 * Float to integer (F1-B2 input): TMC4671::turn (TMC4671.cpp:1752) narrows `maxOffsetFlux` (uint16) to int16 as the upper clip bound
 * (clip<int32_t, int16_t>): for tmc.0.fluxoffset above 32767 the bound is negative (not tested; no case uses such a value). Biquad
 * coefficients (TMC4671Biquad, TMC4671.h:347-352) convert a float times 2^29 to int32: for the cases' cut-offs the values are far inside the
 * int32 range, the result equal on x86 and ARM; the trqbq_f = 0 case reaches f = 1 Hz (coefficient near 2^29 * 2), still inside.
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
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using hid::ReportType;

namespace {

constexpr uint32_t kDirectionX = hid::usage(0x0A, 1);
constexpr uint8_t kRegTorque = 0x64;

int16_t torqueOf(uint32_t reg64) { return static_cast<int16_t>((reg64 >> 16) & 0xffff); }
int16_t fluxOf(uint32_t reg64) { return static_cast<int16_t>(reg64 & 0xffff); }

/** One expected SPI frame of a command: write flag and register address */
struct FrameSpec {
	bool write;
	uint8_t addr;
};

/** How the first 0x64 write after a command differs from the stream before it */
enum class Jump : uint8_t {
	None,
	TorqueNegated, //!< invertForce = 1: the torque sign flips
	FluxStep,	   //!< fluxoffset: the flux field takes -min(|torque|, F)
};

struct RunFixture : IntegrationFixture {
	std::optional<FfbHost> host;
	PositionScript script;
	std::unique_ptr<ChipEventLog> log;

	FfbHost &h() { return *host; }

	/** Bring-up, settings (power and fxratio first), the end of the fade-in; the event log attaches last */
	void up(int64_t power = 5000, bool attachLog = true) {
		bringUp();
		setSpiFrameUs(0);
		host.emplace(*this);
		h().command("axis.0.power=" + std::to_string(power));
		h().command("axis.0.fxratio=255");
		h().command("axis.0.idlespring=0");
		h().run(2500);
		if (attachLog) {
			log = std::make_unique<ChipEventLog>(*this, chip1);
		}
	}

	pid::PidReport makeSetEffect(int64_t index, uint32_t typeUsage, int64_t direction) {
		pid::PidReport se = h().builder().make(ReportType::Output, pid::use::SetEffectReport);
		se.set(pid::use::EffectBlockIndex, index).select(typeUsage).set(pid::use::Duration, 32767).set(pid::use::Gain, 255);
		se.set(pid::use::StartDelay, 0);
		se.flag(pid::use::DirectionEnable, true);
		se.setIn({pid::use::Direction}, kDirectionX, direction);
		return se;
	}
	int64_t newEffect(uint32_t typeUsage) {
		const FfbHost::CreateResult c = h().createEffect(typeUsage);
		REQUIRE(c.load.success());
		return c.load.effectBlockIndex;
	}
	void start(int64_t index) {
		pid::PidReport eo = h().builder().make(ReportType::Output, pid::use::EffectOperationReport);
		eo.set(pid::use::EffectBlockIndex, index).select(pid::use::OpEffectStart).set(pid::use::LoopCount, 1);
		h().send(eo);
	}
	void setMagnitude(int64_t idx, int64_t magnitude) {
		pid::PidReport sc = h().builder().make(ReportType::Output, pid::use::SetConstantForceReport);
		sc.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, magnitude);
		h().send(sc);
	}
	/** Constant force (Direction Enable, polar 90 degrees), started; returns the block index */
	int64_t startConstant(int64_t magnitude) {
		const int64_t idx = newEffect(pid::use::EtConstantForce);
		h().send(makeSetEffect(idx, pid::use::EtConstantForce, 9000));
		setMagnitude(idx, magnitude);
		start(idx);
		return idx;
	}
	/** Slow ramp force -32767 .. 32767 over 30 s (see the file comment); runs 2500 ms so that the torque is non-zero and changing */
	void startRamp() {
		const int64_t idx = newEffect(pid::use::EtRamp);
		pid::PidReport se = makeSetEffect(idx, pid::use::EtRamp, 9000);
		se.set(pid::use::Duration, 30000);
		h().send(se);
		pid::PidReport sr = h().builder().make(ReportType::Output, pid::use::SetRampForceReport);
		sr.set(pid::use::EffectBlockIndex, idx).set(pid::use::RampStart, -32767).set(pid::use::RampEnd, 32767);
		h().send(sr);
		start(idx);
		h().run(2500);
	}
	/** Spring with equal coefficients, no dead band, full saturation, CP Offset 0 (the F1-E5 setup) */
	void startSpring(int64_t coefficient) {
		const int64_t idx = newEffect(pid::use::EtSpring);
		pid::PidReport se = makeSetEffect(idx, pid::use::EtSpring, 0);
		h().send(se);
		pid::PidReport cond = h().builder().make(ReportType::Output, pid::use::SetConditionReport);
		cond.set(pid::use::EffectBlockIndex, idx).set(pid::use::ParameterBlockOffset, 0).set(pid::use::CpOffset, 0);
		cond.set(pid::use::PositiveCoefficient, coefficient).set(pid::use::NegativeCoefficient, coefficient);
		cond.set(pid::use::PositiveSaturation, 32767).set(pid::use::NegativeSaturation, 32767).set(pid::use::DeadBand, 0);
		h().send(cond);
		start(idx);
	}

	int16_t chipTorque() const { return torqueOf(chip1.reg(kRegTorque)); }
	int16_t chipFlux() const { return fluxOf(chip1.reg(kRegTorque)); }
	/** Torque at the chip after `ms` of run time (the constant force has settled: one change, then no more writes) */
	int16_t settled(uint32_t ms = 60) {
		h().run(ms);
		return chipTorque();
	}
	bool driverReady() { return tmc()->getState() == TMC_ControlState::Running && tmc()->motorReady(); }
	/**
	 * Unplug, wait, plug in again (enumeration), then run until the driver is Running and ready (REQUIRE). The wait is 300 ms: stopMotor only
	 * REQUESTS Shutdown (changeState without force, TMC4671.cpp:798-808) and the TMC thread applies the request at the top of its loop, after the
	 * Delay(200) at the end of the Running branch (TMC4671.cpp:577); with a shorter pause startMotor of the mount finds the state still Running
	 * and the request wins afterwards (the fast-replug case below).
	 */
	void replugWithPause(uint32_t pauseMs = 300) {
		HostDcd::detach();
		sched.pumpUsb();
		h().run(pauseMs);
		HostDcd::attach();
		enumerateUsb();
		bool upAgain = false;
		for (uint32_t ms = 0; ms < 20000 && !upAgain; ms++) {
			upAgain = driverReady();
			if (!upAgain) {
				h().run(1);
			}
		}
		REQUIRE(upAgain);
	}

	/** The torque biquad frames: (CONFIG_ADDR, CONFIG_DATA) pairs of the log from event `from`, in order; `alternates` is false if the two registers do not alternate ADDR, DATA */
	struct Biquad {
		std::vector<std::pair<uint32_t, uint32_t>> pairs;
		bool alternates = true;
		std::optional<uint32_t> at(uint32_t addr) const {
			std::optional<uint32_t> r;
			for (const auto &p : pairs) {
				if (p.first == addr) {
					r = p.second; // the last write of that address
				}
			}
			return r;
		}
		int32_t s32(uint32_t addr) const { return static_cast<int32_t>(at(addr).value_or(0)); }
		/** DC gain of the stored coefficients (DS 4.8.1 eq. 33): (b0 + b1 + b2) / (2^29 - a1 - a2) */
		double dcGain() const {
			const double num = static_cast<double>(s32(20)) + s32(21) + s32(22);
			const double den = 536870912.0 - s32(17) - s32(18);
			return num / den;
		}
	};
	Biquad biquadSince(size_t from) const {
		Biquad b;
		bool wantData = false;
		uint32_t addr = 0;
		for (size_t i = from; i < log->events().size(); i++) {
			const ChipEventLog::Event &e = log->events()[i];
			if (e.kind != ChipEventLog::Kind::Write) {
				continue;
			}
			if (e.addr == 0x4E) {
				if (wantData) {
					b.alternates = false;
				}
				addr = e.data;
				wantData = true;
			} else if (e.addr == 0x4D) {
				if (!wantData) {
					b.alternates = false;
				}
				b.pairs.emplace_back(addr, e.data);
				wantData = false;
			}
		}
		if (wantData) {
			b.alternates = false;
		}
		return b;
	}
	size_t mark() const { return log->events().size(); }

	/**
	 * The interleaving check of F1-T6 (b) for one command, with a ramp force running: the command is sent by runCommandOnWrite on a
	 * write to 0x64, the frames of the next 150 ms are analysed.
	 *  1. the command's frames are exactly `burst`, contiguous, directly after the trigger frame and before the writer's next R6B;
	 *  2. no frame sits between a cycle's R6B and the W64 that completes it (every R6B is followed by W64 or by the next R6B);
	 *  3. at most one W64 per cycle (two W64 are separated by an R6B), no duplicate value (written only on change);
	 *  4. the stream is continuous: consecutive W64 differ by at most 3 counts in torque and 0 in flux, except the first write after the
	 *     command for `jump`;
	 *  5. at the end the chip holds the torque the axis computes (axis.0.curtorque, negated for TorqueNegated), within 3 counts.
	 */
	void checkStream(const std::string &cmd, const std::vector<FrameSpec> &burst, Jump jump, int fluxOffsetF = 0) {
		const size_t mark0 = chip1.log().size();
		const size_t w64Before = log->countWrites(kRegTorque);
		h().run(20);
		REQUIRE(log->countWrites(kRegTorque) > w64Before + 3); // the premise: a stream of 0x64 writes
		log->runCommandOnWrite(kRegTorque, cmd);
		for (uint32_t i = 0; i < 100 && log->commandState() < ChipEventLog::CommandState::Sent; i++) {
			h().run(1);
		}
		REQUIRE(log->commandState() >= ChipEventLog::CommandState::Sent);
		CHECK(log->commandRuns() == 1);
		h().run(150);
		const auto &frames = chip1.log();
		const size_t tf = log->events()[log->commandTriggerEvent()].chipFrame;
		REQUIRE(tf < frames.size());
		REQUIRE(frames[tf].write);
		REQUIRE(frames[tf].addr == kRegTorque);
		// On the target the TMC thread can run between two frames of an FFB cycle, because the SPI semaphore is taken per frame, not per cycle.
		// The frames of the TMC thread's own periodic work in the Running state are all derived from the source: statusCheck (TMC4671.cpp:2618-2637:
		// readFlags R7C, setStatusFlags W7C, readFlags R7C), the delayed W7C of the flag check (:652-654), the R76 of the enable input (:550-556)
		// and getTemp's W03 / R02 (ADC_RAW_ADDR / ADC_RAW_DATA, :370-371). They are not the command's and are left out of the frame lists below.
		auto foreign = [&](size_t i) { return frames[i].addr == 0x7C || frames[i].addr == 0x76 || frames[i].addr == 0x03 || frames[i].addr == 0x02; };
		// 1. the burst
		size_t nextR6B = tf + 1;
		while (nextR6B < frames.size() && !(!frames[nextR6B].write && frames[nextR6B].addr == 0x6B)) {
			nextR6B++;
		}
		REQUIRE(nextR6B < frames.size());
		std::vector<size_t> burstIdx;
		for (size_t i = tf + 1; i < nextR6B; i++) {
			if (!foreign(i)) {
				burstIdx.push_back(i);
			}
		}
		{
			std::ostringstream o;
			for (size_t i : burstIdx) {
				o << (frames[i].write ? "W" : "R") << std::hex << static_cast<int>(frames[i].addr) << " ";
			}
			INFO("frames between the trigger W64 and the next R6B: " << o.str());
			REQUIRE(burstIdx.size() == burst.size());
			for (size_t i = 0; i < burst.size(); i++) {
				CHECK(frames[burstIdx[i]].write == burst[i].write);
				CHECK(frames[burstIdx[i]].addr == burst[i].addr);
			}
		}
		// 2. no frame between R6B and its W64
		size_t cycles = 0;
		for (size_t i = mark0; i + 1 < frames.size(); i++) {
			if (!frames[i].write && frames[i].addr == 0x6B) {
				cycles++;
				size_t j = i + 1;
				while (j + 1 < frames.size() && foreign(j)) {
					j++;
				}
				const FakeTmc4671::Frame &nx = frames[j];
				const bool ok = (nx.write && nx.addr == kRegTorque) || (!nx.write && nx.addr == 0x6B);
				if (!ok) {
					std::ostringstream ctx;
					for (size_t q = i; q <= j + 1 && q < frames.size(); q++) {
						ctx << (frames[q].write ? " W" : " R") << "0x" << std::hex << static_cast<int>(frames[q].addr) << std::dec;
					}
					FAIL_CHECK("frame " << j << " splits the cycle that starts with the R6B at frame " << i << ", frames from there:" << ctx.str());
				}
			}
		}
		CHECK(cycles > 100);
		// 3 and 4. the 0x64 stream
		std::vector<std::pair<size_t, uint32_t>> w64;
		for (size_t i = mark0; i < frames.size(); i++) {
			if (frames[i].write && frames[i].addr == kRegTorque) {
				w64.emplace_back(i, frames[i].data);
			}
		}
		REQUIRE(w64.size() > 20);
		for (size_t k = 1; k < w64.size(); k++) {
			bool rBetween = false;
			for (size_t i = w64[k - 1].first + 1; i < w64[k].first; i++) {
				rBetween = rBetween || (!frames[i].write && frames[i].addr == 0x6B);
			}
			if (!rBetween) {
				FAIL_CHECK("two W64 without an R6B between them at frames " << w64[k - 1].first << " and " << w64[k].first);
			}
			if (w64[k].second == w64[k - 1].second) {
				FAIL_CHECK("duplicate W64 value " << w64[k].second << " at frame " << w64[k].first);
			}
			const bool afterCmd = w64[k - 1].first <= tf && w64[k].first > tf; // the first write after the trigger
			const int dT = static_cast<int>(torqueOf(w64[k].second)) - torqueOf(w64[k - 1].second);
			const int dF = static_cast<int>(fluxOf(w64[k].second)) - fluxOf(w64[k - 1].second);
			if (afterCmd && jump == Jump::TorqueNegated) {
				// invertForce = 1: the new torque is minus the one the stream continues with: T_new + T_prev within the 3 counts of a step
				CHECK(std::abs(static_cast<int>(torqueOf(w64[k].second)) + torqueOf(w64[k - 1].second)) <= 3);
				CHECK(dF == 0);
			} else if (afterCmd && jump == Jump::FluxStep) {
				CHECK(std::abs(dT) <= 3);
				const int expectFlux = -std::min<int>(std::abs(torqueOf(w64[k].second)), fluxOffsetF);
				CHECK(fluxOf(w64[k].second) == expectFlux);
			} else {
				if (std::abs(dT) > 3 || dF != 0) {
					FAIL_CHECK("discontinuity at frame " << w64[k].first << ": torque " << torqueOf(w64[k - 1].second) << " -> " << torqueOf(w64[k].second) << ", flux " << fluxOf(w64[k - 1].second) << " -> " << fluxOf(w64[k].second));
				}
			}
		}
		// 5. the chip holds the axis torque
		const int64_t axisTorque = h().getValue("axis.0.curtorque?");
		const int64_t expectChip = (jump == Jump::TorqueNegated) ? -axisTorque : axisTorque;
		CHECK_NEAR(chipTorque(), expectChip, 3);
		REQUIRE_CHAIN_MODELLED();
	}
};

} // namespace

TEST_SUITE("F1Runtime") {

// ---------------------------------------------------------------------------------------------------------------------
// F1-T6 (a): the register the command defines
// ---------------------------------------------------------------------------------------------------------------------

// DS 0x56: PID_TORQUE_I [15:0], PID_TORQUE_P [31:16]. setPids (TMC4671.cpp:2116-2123) writes all four PI registers; the other three keep the
// values of their own conf and 0x63 is rewritten with the same mode.
TEST_CASE_FIXTURE(RunFixture, "F1-T6: tmc.0.torqueP=P writes P into the P field of 0x56 and keeps the I field, the other PI registers and the motion mode" * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	const int16_t t0 = settled();
	REQUIRE(t0 != 0);
	const int64_t torqueI = h().getValue("tmc.0.torqueI?");
	const uint32_t r54 = chip1.reg(0x54), r58 = chip1.reg(0x58), r5A = chip1.reg(0x5A), r63 = chip1.reg(0x63);
	for (int64_t p : {1234, 2345}) {
		INFO("P " << p);
		REQUIRE((chip1.reg(0x56) >> 16) != static_cast<uint32_t>(p));
		h().command("tmc.0.torqueP=" + std::to_string(p));
		h().run(10);
		CHECK((chip1.reg(0x56) >> 16) == static_cast<uint32_t>(p));
		CHECK((chip1.reg(0x56) & 0xffff) == static_cast<uint32_t>(torqueI & 0xffff));
		CHECK(chip1.reg(0x54) == r54);
		CHECK(chip1.reg(0x58) == r58);
		CHECK(chip1.reg(0x5A) == r5A);
		CHECK(chip1.reg(0x63) == r63);
	}
	CHECK(chipTorque() == t0); // the force itself is not touched
	REQUIRE_CHAIN_MODELLED();
}

TEST_CASE_FIXTURE(RunFixture, "F1-T6: tmc.0.torqueI=I writes I into the I field of 0x56 and keeps the P field and the motion mode" * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	const int16_t t0 = settled();
	REQUIRE(t0 != 0);
	const int64_t torqueP = h().getValue("tmc.0.torqueP?");
	const uint32_t r63 = chip1.reg(0x63);
	for (int64_t i : {321, 654}) {
		INFO("I " << i);
		REQUIRE((chip1.reg(0x56) & 0xffff) != static_cast<uint32_t>(i));
		h().command("tmc.0.torqueI=" + std::to_string(i));
		h().run(10);
		CHECK((chip1.reg(0x56) & 0xffff) == static_cast<uint32_t>(i));
		CHECK((chip1.reg(0x56) >> 16) == static_cast<uint32_t>(torqueP & 0xffff));
		CHECK(chip1.reg(0x63) == r63);
	}
	CHECK(chipTorque() == t0);
	REQUIRE_CHAIN_MODELLED();
}

// DS 0x63: MODE_PID_TYPE bit 31 (0 parallel / 1 sequential PI), MODE_MOTION [7:0]. TMC4671.cpp:2011-2014 updateReg(0x63, v, 0x1, 31).
TEST_CASE_FIXTURE(RunFixture, "F1-T6: tmc.0.seqpi sets and clears bit 31 of 0x63 and keeps every other bit, the running torque mode included" * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	const int16_t t0 = settled();
	REQUIRE(t0 != 0);
	const uint32_t before = chip1.reg(0x63);
	REQUIRE((before & 0xFF) == 1); // torque_mode (DS 0x63 MODE_MOTION 1) while the force runs
	const uint32_t other = before & 0x7fffffffu;
	const uint32_t first = (before >> 31) ? 0 : 1; // the opposite of what is set now, then back
	for (uint32_t v : {first, 1 - first}) {
		INFO("seqpi " << v);
		h().command("tmc.0.seqpi=" + std::to_string(v));
		h().run(10);
		CHECK((chip1.reg(0x63) >> 31) == v);
		CHECK((chip1.reg(0x63) & 0x7fffffffu) == other);
		CHECK(h().getValue("tmc.0.seqpi?") == v);
	}
	CHECK(chipTorque() == t0);
	REQUIRE_CHAIN_MODELLED();
}

// DS 0x5D: PIDOUT_UQ_UD_LIMITS s16 [15:0], 0..32767. TMC4671.cpp:2140 writeReg(0x5D, limit)
TEST_CASE_FIXTURE(RunFixture, "F1-T6: tmc.0.pwmlim=L writes L into the limit field of 0x5D and leaves the running torque alone" * IT_DELETES_FFBHIDMAIN) {
	up();
	h().command("sys.0.debug=1"); // pwmlim is a debug command (CMDFLAG_DEBUG, TMC4671.cpp:2779): "Invalid command" without it
	startConstant(20000);
	const int16_t t0 = settled();
	REQUIRE(t0 != 0);
	for (int64_t l : {12000, 20000}) {
		INFO("limit " << l);
		REQUIRE((chip1.reg(0x5D) & 0xffff) != static_cast<uint32_t>(l));
		h().command("tmc.0.pwmlim=" + std::to_string(l));
		h().run(10);
		CHECK((chip1.reg(0x5D) & 0xffff) == static_cast<uint32_t>(l));
		CHECK(h().getValue("tmc.0.pwmlim?") == l);
	}
	CHECK(chipTorque() == t0);
	REQUIRE_CHAIN_MODELLED();
}

// DS 4.8.1 and the 0x4D / 0x4E register table: a mode switch writes the five coefficients and the enable of the torque biquad through
// CONFIG_ADDR 17, 18, 20, 21, 22, 23. The lowpass is a unity-gain filter at DC (eq. 33: (b0 + b1 + b2) / (2^29 - a1 - a2) = 1).
// MATH of the tolerance: the coefficients are floats times 2^29 truncated to int32; a1 near 1.8 * 2^29 has a float step of 64 counts, so the
// denominator is exact to ~130 counts of about 2^29 * (1 - a1 - a2) (at 1000 Hz / 25 kHz about 5e6): 3e-5; 0.01 is far above that and far
// below the gain of a wrong filter.
TEST_CASE_FIXTURE(RunFixture, "F1-T6: tmc.0.trqbq_mode=1 writes the torque biquad through CONFIG_ADDR 17, 18, 20, 21, 22, 23 with unity DC gain" * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	const int16_t t0 = settled();
	REQUIRE(t0 != 0);
	REQUIRE(h().getValue("tmc.0.trqbq_mode?") == 0);
	const size_t m = mark();
	h().command("tmc.0.trqbq_mode=1");
	h().run(10);
	const Biquad b = biquadSince(m);
	CHECK(b.alternates);
	REQUIRE(b.pairs.size() == 6);
	const std::vector<uint32_t> order = {17, 18, 20, 21, 22, 23};
	for (size_t i = 0; i < order.size(); i++) {
		INFO("pair " << i);
		CHECK(b.pairs[i].first == order[i]);
	}
	CHECK_NEAR(b.dcGain(), 1.0, 0.01);
	CHECK(h().getValue("tmc.0.trqbq_mode?") == 1);
	CHECK(chipTorque() == t0);
	REQUIRE_CHAIN_MODELLED();
}

// INR 51: the value of the enable write. The literal reading of the DS ("bit(31)", range 0..0x1) is bit 31; the firmware writes bit 0 (the
// value 1, TMC4671.cpp:2271), with which the filter would never be enabled under the literal reading. Records what the firmware writes.
TEST_CASE_FIXTURE(RunFixture, "characterization: [F1-T6] tmc.0.trqbq_mode=1 writes the value 1 (bit 0) as the enable of the torque biquad" * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	REQUIRE(settled() != 0);
	const size_t m = mark();
	h().command("tmc.0.trqbq_mode=1");
	h().run(10);
	const Biquad b = biquadSince(m);
	REQUIRE(b.pairs.size() == 6);
	// INR 51
	CHECK(b.at(23).value_or(99) == 1);
	REQUIRE_CHAIN_MODELLED();
}

TEST_CASE_FIXTURE(RunFixture, "F1-T6: tmc.0.trqbq_mode=0 after a lowpass writes the enable of the torque biquad as 0" * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	h().command("tmc.0.trqbq_mode=1");
	const int16_t t0 = settled();
	REQUIRE(t0 != 0);
	REQUIRE(biquadSince(0).pairs.size() == 6); // the lowpass was written (its enable value is the INR 51 question)
	const size_t m = mark();
	h().command("tmc.0.trqbq_mode=0");
	h().run(10);
	const Biquad b = biquadSince(m);
	CHECK(b.alternates);
	REQUIRE(b.pairs.size() == 6);
	CHECK(b.pairs.back().first == 23);
	CHECK(b.pairs.back().second == 0);
	CHECK(chipTorque() == t0);
	REQUIRE_CHAIN_MODELLED();
}

// A lower cut-off frequency weights the input less (b0 of a low pass grows with the cut-off); both filters keep unity DC gain.
TEST_CASE_FIXTURE(RunFixture, "F1-T6: tmc.0.trqbq_f changes the cut-off of the torque lowpass: a lower frequency stores a smaller b0 and keeps unity DC gain" * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	h().command("tmc.0.trqbq_mode=1");
	h().command("tmc.0.trqbq_f=800");
	h().run(10);
	const size_t m = mark();
	h().command("tmc.0.trqbq_f=400");
	h().run(10);
	const Biquad lo = biquadSince(m);
	const size_t m2 = mark();
	h().command("tmc.0.trqbq_f=800");
	h().run(10);
	const Biquad hi = biquadSince(m2);
	REQUIRE(lo.pairs.size() == 6);
	REQUIRE(hi.pairs.size() == 6);
	CHECK(lo.alternates);
	CHECK(hi.alternates);
	CHECK(lo.s32(20) > 0);
	CHECK(lo.s32(20) < hi.s32(20));
	CHECK_NEAR(lo.dcGain(), 1.0, 0.01);
	CHECK_NEAR(hi.dcGain(), 1.0, 0.01);
	CHECK(h().getValue("tmc.0.trqbq_f?") == 800);
	REQUIRE_CHAIN_MODELLED();
}

// The help text of the command (TMC4671.cpp:2809): "Torque filter freq 1000 max. 0 to disable". The handler clips the value to 1..0x1fff
// (TMC4671.cpp:3095), so f = 0 does not disable the filter: it enables a lowpass at 1 Hz in the torque path. The help text may be stale (it
// also says "1000 max" while the code clips at 0x1fff); the case keeps the intended "0 disables" until the owner says otherwise. The == 0 check
// holds under either reading of the enable field (INR 51).
TEST_CASE_FIXTURE(RunFixture, "F1-T6: tmc.0.trqbq_f=0 disables the torque filter as its help text says" * KNOWN_ISSUE("(IB 44) trqbq_f=0 is clipped to 1 (TMC4671.cpp:3095) and the torque lowpass runs at 1 Hz instead of being disabled, although the help text says '0 to disable' (TMC4671.cpp:2809)") * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	h().command("tmc.0.trqbq_mode=1");
	h().command("tmc.0.trqbq_f=800");
	h().run(10);
	REQUIRE(biquadSince(0).pairs.size() == 6);
	const size_t m = mark();
	h().command("tmc.0.trqbq_f=0");
	h().run(10);
	const Biquad b = biquadSince(m);
	REQUIRE(b.pairs.size() == 6); // the filter was rewritten
	REQUIRE_CHAIN_MODELLED();
	CHECK(b.at(23).value_or(99) == 0);
}

// invertForce: no register of its own. turn() negates the torque (TMC4671.cpp:1753-1755), so the next 0x64 write after a change of the
// force carries the negated torque. Plain: inverting reverses the sign of the torque written for the same force (the relation is the
// definition of "invert"; no firmware constant). MATH: force 20000 gives T1 at the chip (|T1| ~ 3051); after invertForce=1 and a change of
// the magnitude to 10000 the torque is -T1/2 within the rounding of the chain (two truncations of 1 count each, so 3).
TEST_CASE_FIXTURE(RunFixture, "F1-T6: tmc.0.invertForce=1 reverses the sign of the torque written for the next change of the force and =0 restores it" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = startConstant(20000);
	const int16_t t1 = settled();
	REQUIRE(std::abs(t1) > 2000);
	REQUIRE(h().getValue("tmc.0.invertForce?") == 0);
	h().command("tmc.0.invertForce=1");
	CHECK(h().getValue("tmc.0.invertForce?") == 1);
	setMagnitude(idx, 10000);
	const int16_t t2 = settled();
	INFO("t1 " << t1 << ", t2 " << t2);
	CHECK_NEAR(t2, -t1 / 2.0, 3);
	h().command("tmc.0.invertForce=0");
	setMagnitude(idx, 20000);
	const int16_t t3 = settled();
	INFO("t3 " << t3);
	CHECK_NEAR(t3, t1, 3);
	REQUIRE_CHAIN_MODELLED();
}

// INR 46: Axis::updateDriveTorque calls turn() only when the axis torque changed (Axis.cpp:488-495, :921), so a command that changes the
// mapping of a held force (invertForce, fluxoffset) does not reach the chip until the force changes: 0x64 keeps the old sign. Related to IB 22
// (same cause: no re-send of a held torque). Records today's behaviour for review (intended: the change acts at once?).
TEST_CASE_FIXTURE(RunFixture, "characterization: [F1-T6] a held constant force is not rewritten when tmc.0.invertForce changes, 0x64 keeps the old torque until the force changes" * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	const int16_t t1 = settled();
	REQUIRE(std::abs(t1) > 2000);
	const size_t w = log->countWrites(kRegTorque);
	h().command("tmc.0.invertForce=1");
	h().run(200);
	// INR 46: no write, the torque keeps its sign
	CHECK(log->countWrites(kRegTorque) == w);
	CHECK(chipTorque() == t1);
	REQUIRE_CHAIN_MODELLED();
}

// fluxoffset (TMC4671.cpp:1752): flux = idleFlux - min(|torque|, maxOffsetFlux), idleFlux 0 (TMC4671.h:496); DS 0x64 s16 [15:0] flux. The scaling
// is the firmware's (help text "Offset flux scale for field weakening"): characterization, INR 47. Plain parts: the torque field is not
// changed by the command, the flux field is signed 16 bit.
TEST_CASE_FIXTURE(RunFixture, "characterization: [F1-T6] tmc.0.fluxoffset=F makes the flux field of the next 0x64 write -min(|torque|, F)" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = startConstant(20000);
	const int16_t t1 = settled();
	REQUIRE(std::abs(t1) > 2000);
	REQUIRE(chipFlux() == 0); // fluxoffset default 0
	h().command("tmc.0.fluxoffset=1000");
	CHECK(h().getValue("tmc.0.fluxoffset?") == 1000);
	setMagnitude(idx, 10000); // |torque| about 1525 > 1000
	const int16_t t2 = settled();
	INFO("t2 " << t2);
	CHECK_NEAR(t2, t1 / 2.0, 3); // plain: the torque field is the same as without the command
	// INR 47
	CHECK(chipFlux() == -1000);
	setMagnitude(idx, 4000); // |torque| about 610 < 1000
	const int16_t t3 = settled();
	CHECK(chipFlux() == -std::abs(t3));
	REQUIRE_CHAIN_MODELLED();
}

// combineEncoder (TMC4671.cpp:2820-2824): after every handling of the command, a GET included, `if(!conf.combineEncoder) conf.invertForce = false`
// ("Force off"). With the invariant "invertForce needs combineEncoder" (the flash restore masks it, :2720; IB 15: the setter bypasses it).
// INR 48: the firmware's own rule, no external oracle.
TEST_CASE_FIXTURE(RunFixture, "characterization: [F1-T6] tmc.0.combineEncoder=0 switches invertForce off so that the next torque is not inverted" * IT_DELETES_FFBHIDMAIN) {
	up();
	const int64_t idx = startConstant(20000);
	const int16_t t1 = settled();
	REQUIRE(std::abs(t1) > 2000);
	h().command("tmc.0.invertForce=1");
	setMagnitude(idx, 10000);
	const int16_t t2 = settled();
	CHECK_NEAR(t2, -t1 / 2.0, 3); // inverted (plain part, see the invertForce case)
	REQUIRE(h().getValue("tmc.0.invertForce?") == 1);
	h().command("tmc.0.combineEncoder=0");
	// INR 48
	CHECK(h().getValue("tmc.0.invertForce?") == 0);
	setMagnitude(idx, 5000);
	const int16_t t3 = settled();
	INFO("t3 " << t3);
	CHECK_NEAR(t3, t1 / 4.0, 3); // not inverted: same sign as the first force
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-T6 (b): the 0x64 stream stays unbroken when the command runs in the middle of it
// ---------------------------------------------------------------------------------------------------------------------

// Frame lists: setPids (TMC4671.cpp:2116-2123): W54 W56 W58 W5A, then setSequentialPI -> updateReg: R63 W63 (:2571-2576). setUqUdLimit :2140: W5D.
// setTorqueFilter -> setBiquadTorque (:2256-2272): six times (W4E, W4D). seqpi: R63 W63.
#define F1T6_STREAM(NAME, CMD, BURST, JUMP, FLUXF, PRE)                                                                \
	TEST_CASE_FIXTURE(RunFixture, "F1-T6: the 0x64 stream stays unbroken when " NAME " runs in the middle of it" * IT_DELETES_FFBHIDMAIN) { \
		up();                                                                                                          \
		PRE;                                                                                                           \
		startRamp();                                                                                                   \
		checkStream(CMD, BURST, JUMP, FLUXF);                                                                          \
	}

const std::vector<FrameSpec> kSetPidsFrames = {{true, 0x54}, {true, 0x56}, {true, 0x58}, {true, 0x5A}, {false, 0x63}, {true, 0x63}};
const std::vector<FrameSpec> kSeqPiFrames = {{false, 0x63}, {true, 0x63}};
const std::vector<FrameSpec> kPwmLimFrames = {{true, 0x5D}};
const std::vector<FrameSpec> kBiquadFrames = {{true, 0x4E}, {true, 0x4D}, {true, 0x4E}, {true, 0x4D}, {true, 0x4E}, {true, 0x4D},
											  {true, 0x4E}, {true, 0x4D}, {true, 0x4E}, {true, 0x4D}, {true, 0x4E}, {true, 0x4D}};
const std::vector<FrameSpec> kNoFrames = {};

F1T6_STREAM("tmc.0.torqueP", "tmc.0.torqueP=1234", kSetPidsFrames, Jump::None, 0, (void)0)
F1T6_STREAM("tmc.0.torqueI", "tmc.0.torqueI=321", kSetPidsFrames, Jump::None, 0, (void)0)
F1T6_STREAM("tmc.0.seqpi", "tmc.0.seqpi=1", kSeqPiFrames, Jump::None, 0, (void)0)
F1T6_STREAM("tmc.0.pwmlim", "tmc.0.pwmlim=12000", kPwmLimFrames, Jump::None, 0, h().command("sys.0.debug=1"))
F1T6_STREAM("tmc.0.trqbq_mode", "tmc.0.trqbq_mode=1", kBiquadFrames, Jump::None, 0, (void)0)
F1T6_STREAM("tmc.0.trqbq_f", "tmc.0.trqbq_f=400", kBiquadFrames, Jump::None, 0, h().command("tmc.0.trqbq_mode=1"))
F1T6_STREAM("tmc.0.invertForce", "tmc.0.invertForce=1", kNoFrames, Jump::TorqueNegated, 0, (void)0)
F1T6_STREAM("tmc.0.fluxoffset", "tmc.0.fluxoffset=1000", kNoFrames, Jump::FluxStep, 1000, (void)0)
F1T6_STREAM("tmc.0.combineEncoder (value 0, already 0: no frames, no change)", "tmc.0.combineEncoder=0", kNoFrames, Jump::None, 0, (void)0)

// ---------------------------------------------------------------------------------------------------------------------
// F1-T7: USB state with an effect running
// ---------------------------------------------------------------------------------------------------------------------

// Safety (no firmware constant): after the unplug the chip carries torque 0 and the PWM is off, and nothing non-zero follows.
// Path: tud_umount_cb -> FFBHIDMain::usbSuspend -> Axis::usbSuspend: turn(0), stopMotor (Axis.cpp:629-635; TMC4671.cpp:1671-1680: motion mode
// stop, PWM off). DS 0x1A PWM_CHOP [7:0]: 0 = off.
TEST_CASE_FIXTURE(RunFixture, "F1-T7: unplugging USB with a force running puts torque 0 at the chip, switches the PWM off and no non-zero torque follows" * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	const int16_t t1 = settled();
	REQUIRE(std::abs(t1) > 2000);
	REQUIRE((chip1.reg(0x1A) & 0xFF) == 7); // FOC
	const size_t n0 = mark();
	HostDcd::detach();
	sched.pumpUsb();
	h().run(300);
	CHECK(chipTorque() == 0);
	CHECK(chipFlux() == 0);
	CHECK((chip1.reg(0x1A) & 0xFF) == 0);
	const std::optional<uint32_t> pwm = log->valueAt(0x1A, log->events().size() - 1);
	REQUIRE(pwm.has_value());
	CHECK((*pwm & 0xFF) == 0);
	for (size_t i = n0; i < log->events().size(); i++) {
		const ChipEventLog::Event &e = log->events()[i];
		if (e.kind == ChipEventLog::Kind::Write && e.addr == kRegTorque) {
			INFO("event " << i);
			CHECK(torqueOf(e.data) == 0);
		}
	}
	CHECK(log->countWrites(kRegTorque) > log->countWrites(kRegTorque, n0)); // the zero was written
	REQUIRE_CHAIN_MODELLED();
}

// A bus suspend: the firmware declares tud_suspend_cb() without the bool parameter TinyUSB calls (same as coverage bug 42), so
// FFBHIDMain::usbSuspend is never called and the torque stays on. Intended: the firmware's own comment (global_callbacks.cpp:362-364
// "Called on usb disconnect and suspend", FFBHIDMain.cpp:303-311 "Deactivates FFB"): the torque is 0 after the suspend.
TEST_CASE_FIXTURE(RunFixture, "F1-T7: a USB bus suspend with a force running puts torque 0 at the chip" * KNOWN_ISSUE("(IB 45; same as bug 42) tud_suspend_cb() is defined without the bool parameter (global_callbacks.cpp:365) that TinyUSB calls (usbd.c:659), so a bus suspend never reaches FFBHIDMain::usbSuspend and the force keeps driving the motor") * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	const int16_t t1 = settled();
	REQUIRE(std::abs(t1) > 2000);
	HostDcd::suspend();
	sched.pumpUsb();
	h().run(300);
	REQUIRE(HostDcd::state().suspended);
	REQUIRE_CHAIN_MODELLED();
	CHECK(chipTorque() == 0);
}

// Re-enumeration: the replug runs tud_umount_cb (usbSuspend: effects deleted, torque 0, motor stopped) and tud_mount_cb (usbResume: fade-in and
// startMotor). INR 49: that the effects do not survive is the firmware's documented behaviour (FFBHIDMain.cpp:310 "Delete all effects").
TEST_CASE_FIXTURE(RunFixture, "characterization: [F1-T7] after an unplug and a replug the old force is gone: the driver runs again and the torque stays 0" * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	const int16_t t1 = settled();
	REQUIRE(std::abs(t1) > 2000);
	replugWithPause();
	const size_t n0 = mark();
	h().run(2500);
	// INR 49
	CHECK(chipTorque() == 0);
	for (size_t i = n0; i < log->events().size(); i++) {
		const ChipEventLog::Event &e = log->events()[i];
		if (e.kind == ChipEventLog::Kind::Write && e.addr == kRegTorque) {
			CHECK(torqueOf(e.data) == 0);
		}
	}
	REQUIRE_CHAIN_MODELLED();
}

// After the replug a new force reaches the chip at the same torque as the one before the unplug (plain: same setting, same effect; +-3 counts of
// rounding), once the fade-in of the resume has finished (0.5 s from usbResume, 1.0 s from the first ready cycle: 2500 ms wait).
TEST_CASE_FIXTURE(RunFixture, "F1-T7: after an unplug and a replug a new constant force reaches the chip at the same torque as before" * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	const int16_t t1 = settled();
	REQUIRE(std::abs(t1) > 2000);
	replugWithPause();
	startConstant(20000);
	h().run(2500);
	CHECK_NEAR(chipTorque(), t1, 3);
	REQUIRE_CHAIN_MODELLED();
}

// Bug 42 side effect: tud_resume_cb() has the signature TinyUSB calls (void), so a bus RESUME reaches FFBHIDMain::usbResume, although the
// suspend before it never did. Axis::usbResume (Axis.cpp:641-645) starts a force fade-in from 0 (startForceFadeIn) on a running motor: the
// held force drops to zero and ramps back. INR 50: today's behaviour, no external oracle (intended: a resume without a suspend changes nothing).
TEST_CASE_FIXTURE(RunFixture, "characterization: [F1-T7] a bus resume after a suspend that never reached the main class restarts the force from zero" * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	const int16_t t1 = settled();
	REQUIRE(std::abs(t1) > 2000);
	HostDcd::suspend();
	sched.pumpUsb();
	h().run(100);
	REQUIRE(chipTorque() == t1); // the suspend changed nothing (bug 42, previous case)
	HostDcd::resume();
	sched.pumpUsb();
	h().run(100);
	// INR 50: 100 ms after the resume the torque is below a quarter of the held value, and it grows from there
	const int16_t early = chipTorque();
	h().run(300);
	const int16_t later = chipTorque();
	INFO("held " << t1 << ", 100 ms after the resume " << early << ", 400 ms after " << later);
	CHECK(std::abs(early) < std::abs(t1) / 4);
	CHECK(std::abs(later) > std::abs(early));
	REQUIRE_CHAIN_MODELLED();
}

// Mechanism (stale requestedState): the unmount's stopMotor only REQUESTS Shutdown (changeState without force sets requestedState,
// TMC4671.cpp:798-808, :1676-1679); the mount's startMotor, run before the TMC thread applied that request, still sees the state Running and
// requests nothing (:1685); the thread then applies the old request at the top of its loop (:452-454; its Running branch sleeps 200 ms, :577).
// The driver ends in Shutdown with PWM on and motion mode torque while the axis keeps computing torque that turn() drops (motorReady false): the
// chip holds the last 0x64 value for good. It does not depend on the thread's phase when the replug is immediate (gap 0): every immediate
// replug of a running wheel ends this way (found by the I2-2 review: every second one; the phase sweep of this WO shows it at 0 to 200 ms
// in 20 ms steps). With a gap longer than the thread's 200 ms sleep the request is applied before the mount and the driver restarts (the plain
// replug cases above). Intended: after the re-enumeration the driver is Running again (charter F1-X7 recovery).
TEST_CASE_FIXTURE(RunFixture, "F1-T7: after an immediate unplug and replug of a running wheel the driver is Running again" * KNOWN_ISSUE("(IB 47; related to IB 24 and bug 152) the unmount only requests Shutdown (TMC4671.cpp:798-808, :1676-1679), the mount's startMotor sees the stale state Running and requests nothing (:1685), and the thread applies the stale Shutdown request afterwards (:452-454, :577): every immediate replug of a running wheel ends in Shutdown with the PWM on and the force frozen") * IT_DELETES_FFBHIDMAIN) {
	up();
	startConstant(20000);
	const int16_t t1 = settled();
	REQUIRE(std::abs(t1) > 2000);
	REQUIRE(driverReady()); // Running before the unplug
	replugUsb();			// detach, USB step, attach, enumeration: no simulated time in between
	h().run(1500);
	REQUIRE_CHAIN_MODELLED();
	CHECK(driverReady());
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-T8: SPI faults with a running spring
// ---------------------------------------------------------------------------------------------------------------------

namespace {

/** Spring running against a scripted position that ramps `rate` counts per FFB cycle (keyed to the cycle count, IB 1) */
struct SpringFixture : RunFixture {
	double k = 0; //!< torque per count, measured at two constant positions (PID 5.3: the spring is linear in the position)

	void prepare() {
		up(10000);
		h().command("axis.0.degrees=360");
		h().command("axis.0.invert=0");
		h().command("axis.0.axisdamper=0");
		h().command("axis.0.axisinertia=0");
		h().command("axis.0.axisfriction=0");
		script.setConstant(0);
		attachPositionScript(script);
		h().run(1500);
		startSpring(8000);
		h().run(200);
		// calibration: the torque at 1000 and 3000 counts (steady) gives the slope; both must be unclipped and non-zero
		script.setConstant(1000);
		h().run(300);
		const int16_t a = chipTorque();
		script.setConstant(3000);
		h().run(300);
		const int16_t b = chipTorque();
		REQUIRE(std::abs(b) > 100);
		REQUIRE(std::abs(b) < 9000);
		REQUIRE(std::abs(a) < std::abs(b));
		k = (static_cast<double>(b) - a) / 2000.0;
		script.setConstant(3000);
	}
	void rampFrom(int32_t rate) {
		const int32_t base = script.lastValue();
		const uint32_t c0 = ffbCycles();
		script.setFunction([=, this](uint64_t) { return static_cast<int32_t>(base + static_cast<int64_t>(rate) * static_cast<int64_t>(ffbCycles() - c0)); });
	}
	/** torque values (decoded) of the 0x64 writes after event index `from` */
	std::vector<int16_t> torquesFrom(size_t from) {
		std::vector<int16_t> out;
		for (size_t i = from; i < log->events().size(); i++) {
			const ChipEventLog::Event &e = log->events()[i];
			if (e.kind == ChipEventLog::Kind::Write && e.addr == kRegTorque) {
				out.push_back(torqueOf(e.data));
			}
		}
		return out;
	}
	/** Largest |difference| of consecutive entries */
	static int largestStep(const std::vector<int16_t> &v) {
		int worst = 0;
		for (size_t i = 1; i < v.size(); i++) {
			worst = std::max(worst, std::abs(static_cast<int>(v[i]) - v[i - 1]));
		}
		return worst;
	}
};

constexpr int32_t kRate = 10; // counts per FFB cycle

} // namespace

// MATH: the spring is linear (PID 5.3), slope k torque per count measured at two constant positions. The ramp moves kRate counts per cycle, so
// the torque changes by |k| * kRate per cycle (k = -0.3025 measured, kRate 10: 3.0 counts per cycle). A failed read frame delivers its data (the HAL status is
// ignored, SPI.cpp:176-183), so no step beyond one cycle of motion (plus the rounding of 3 counts) may appear, and one lost cycle at most
// doubles it. Bound: 2 |k| kRate + 3 = 9.05.
TEST_CASE_FIXTURE(SpringFixture, "F1-T8: a HAL_ERROR on the read of 0x6B does not make the spring torque at the chip jump" * IT_DELETES_FFBHIDMAIN) {
	prepare();
	rampFrom(kRate);
	h().run(50);
	const size_t n0 = mark();
	log->armSpiFailure(0x6B, false, HAL_ERROR);
	h().run(100);
	REQUIRE(log->failedTransferIndex() >= 0);
	const std::vector<int16_t> t = torquesFrom(n0);
	REQUIRE(t.size() > 50);
	const double bound = 2.0 * std::fabs(k) * kRate + 3.0;
	INFO("k " << k << ", bound " << bound << ", largest step " << largestStep(t));
	CHECK(largestStep(t) <= bound);
	REQUIRE_CHAIN_MODELLED();
}

TEST_CASE_FIXTURE(SpringFixture, "F1-T8: a HAL_ERROR on the write of 0x64 does not make the spring torque at the chip jump" * IT_DELETES_FFBHIDMAIN) {
	prepare();
	rampFrom(kRate);
	h().run(50);
	const size_t n0 = mark();
	log->armSpiFailure(kRegTorque, true, HAL_ERROR);
	h().run(100);
	REQUIRE(log->failedTransferIndex() >= 0);
	const std::vector<int16_t> t = torquesFrom(n0);
	REQUIRE(t.size() > 50);
	const double bound = 2.0 * std::fabs(k) * kRate + 3.0;
	INFO("k " << k << ", bound " << bound << ", largest step " << largestStep(t));
	CHECK(largestStep(t) <= bound);
	REQUIRE_CHAIN_MODELLED();
}

// A write frame lost to a timeout (the frame after the R6B of a cycle in which the position changed is the W64): nothing reaches the chip.
// With the position held afterwards the spring torque is constant, so Axis::updateDriveTorque (writes only on change, Axis.cpp:488-495, :921)
// never repeats the lost value: the chip keeps the torque of the old position for good. Intended (charter rule 3): 0x64 equals the axis
// torque (axis.0.curtorque) a few cycles later.
TEST_CASE_FIXTURE(SpringFixture, "F1-T8: a 0x64 write lost to an SPI timeout is repeated so that the chip holds the torque the axis computes" * KNOWN_ISSUE("(IB 46; related to IB 22, IB 25 and bug 140) the torque is written only when it changes (Axis.cpp:488-495, :921) and the result of the SPI transfer is ignored (SPI.cpp:163, :181; TMC4671.cpp:2543-2556), so a W64 lost to a timeout is never repeated while the force is held") * IT_DELETES_FFBHIDMAIN) {
	prepare();
	const int16_t before = chipTorque();
	script.setConstant(6000); // written at the end of this step: the next FFB cycle reads it
	h().run(1);
	REQUIRE(script.lastValue() == 6000);
	const size_t w64Before = log->countWrites(kRegTorque);
	log->armSpiTimeoutAfter(0x6B, false); // the R6B of that cycle matches, the W64 after it times out
	h().run(300);
	REQUIRE(log->failedTransferIndex() >= 1);
	{
		// setup: the frame that timed out is the cycle's W64 and not a frame of the TMC thread (a timed-out transfer records no bytes, so it is
		// identified by its neighbours): it follows the armed R6B, the cycle's next frame is the next cycle's R6B (a W64 here would mean the
		// W64 went through and the TMC thread's frame was hit), and no W64 reached the chip afterwards
		const auto &tr = HostTarget::spiTransfers(&hspi1);
		const size_t bad = static_cast<size_t>(log->failedTransferIndex());
		REQUIRE(bad + 1 < tr.size());
		REQUIRE(!tr[bad - 1].tx.empty());
		REQUIRE(!tr[bad + 1].tx.empty());
		REQUIRE(tr[bad - 1].tx[0] == 0x6B);
		REQUIRE(tr[bad + 1].tx[0] == 0x6B);
		REQUIRE(log->countWrites(kRegTorque) == w64Before);
	}
	const int64_t axisTorque = h().getValue("axis.0.curtorque?");
	INFO("chip " << chipTorque() << ", axis " << axisTorque << ", before the step " << before);
	REQUIRE(std::abs(axisTorque - before) > 100); // the premise: the step moved the torque
	REQUIRE_CHAIN_MODELLED();
	CHECK_NEAR(chipTorque(), axisTorque, 3);
}


// A read frame lost to a timeout (the frame after a W64 is the next cycle's R6B): nothing is received, readReg memcpy's its uninitialised
// 5-byte stack buffer (TMC4671.cpp:2531-2535: `uint8_t tbuf[5]` never written, the HAL status ignored at SPI.cpp:181) and the garbage is used as
// the position. Undefined behaviour: the value depends on what the stack held. Intended: the status is checked and no garbage is used, so
// the spring torque does not jump (same bound as above). The assertion is on the intended bound, never on a garbage value.
TEST_CASE_FIXTURE(SpringFixture, "F1-T8: a read of 0x6B lost to an SPI timeout does not make the spring torque at the chip jump" * KNOWN_ISSUE_UB("(IB 48; same as bug 140) readReg uses its uninitialised receive buffer when the SPI transfer times out (TMC4671.cpp:2531-2535, status ignored at SPI.cpp:181): the garbage is taken as the position and the torque jumps") * IT_DELETES_FFBHIDMAIN) {
	prepare();
	rampFrom(kRate);
	h().run(50);
	const size_t n0 = mark();
	std::vector<int16_t> t = {chipTorque()}; // the value the chip held when the fault was armed starts the series: a frozen chip has no later writes
	log->armSpiTimeoutAfter(kRegTorque, true); // the frame after the next W64 is the next cycle's R6B
	h().run(100);
	REQUIRE(log->failedTransferIndex() >= 0);
	const std::vector<int16_t> w = torquesFrom(n0);
	t.insert(t.end(), w.begin(), w.end());
	script.setConstant(script.lastValue()); // hold the position so that the chip must settle on the axis torque
	h().run(100);
	const int64_t axisTorque = h().getValue("axis.0.curtorque?");
	const double bound = 2.0 * std::fabs(k) * kRate + 3.0;
	INFO("k " << k << ", bound " << bound << ", largest step " << largestStep(t) << ", writes " << w.size() << ", chip " << chipTorque() << ", axis " << axisTorque);
	REQUIRE_CHAIN_MODELLED();
	// the defect, in either of its forms: a step beyond the spring's own motion (garbage taken as the position), or a chip that no longer
	// follows the axis (the garbage position trips the out-of-bounds stop, Axis.cpp:431-434, and the drive stays off)
	CHECK(largestStep(t) <= bound);
	CHECK_NEAR(chipTorque(), axisTorque, 3);
}

} // TEST_SUITE
