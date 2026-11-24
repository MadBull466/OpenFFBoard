/*
 * ChipEventLog.h (WO I2-1)
 *
 * Ordered record of the driver events of ONE FakeTmc4671 chip, attached to an IntegrationFixture, plus three fault / interleaving
 * helpers that need the same hook points (armSpiFailure, armSpiTimeoutAfter, runCommandOnWrite). For the phase-I2 cases (driver in
 * the chain, persistence). Oracle of the register meanings: TMC4671-LA datasheet Rev 2.07 (0x64 PID_TORQUE_FLUX_TARGET: flux
 * [15:0], torque [31:16], DS p98).
 *
 * HOW THE LOG GETS ITS DATA
 *  - Writes: FakeTmc4671::onWrite hooks on ALL 128 register addresses (the chip's log() has no pin level and its hooks are the only
 *    way to sample the enable pin at the moment of the write). The fake runs a write hook inside the SPI frame, after the write was
 *    applied and logged: the event takes its time from chip.log().back().timeUs (the START of the frame), the pin level
 *    HostTarget::outputPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin) at that moment (frame end: pin changes made by the firmware before
 *    the frame are seen, a change made after the frame is not). Writes to read-only registers are violations in the fake, run no hook
 *    and are not in the log (complete() ignores them).
 *  - Enable level: a Kind::Enable event whenever the level differs from the last recorded one (initially the level at attach), sampled
 *    at the end of every scheduler step (ChainScheduler::stepEndHook, step (g)). RESOLUTION LIMIT: a pin change that happens between
 *    two writes of the same step is seen only at the end of that step, so the Enable event then comes AFTER those writes in the log;
 *    the per-write `enable` field (sampled at the write) is exact and is the better evidence inside one step. The time of an Enable
 *    event is the time of the end of the step.
 *  - Coexistence: a FakeTmc4671 holds one hook per address and has no getter, so the log cannot chain to a hook installed before
 *    it. It owns the slots of the chip (all addresses, read and write): a user hook on the same chip goes through
 *    ChipEventLog::onWrite / onRead, which the log runs after it recorded the frame. A hook installed directly with chip.onWrite /
 *    onRead after the log attached REPLACES the log's hook for that address: complete() then returns false (frames the chip logged
 *    that the log did not see). sched.stepEndHook: the log chains to the hook that was there when it attached (called after the log's
 *    own sampling). RULE: attach the log AFTER every observer that sets sched.stepEndHook (TorqueTrace::start requires an empty hook
 *    and stop() / its destructor set it to nullptr, which also removes the log's sampling and any pending runCommandOnWrite send).
 *    The log counts its own hook calls against the scheduler steps since attach: a dropped hook is a fixture fault in
 *    firstFault() ("step sampling lost") and complete() returns false. Stop a TorqueTrace only after the log is destroyed.
 *    Presence: complete() uses the chip's presence at attach; a change of present() afterwards is the same kind of fault (frames of an
 *    absent chip run no hook). PositionScript / WheelPlant use no hook (they write with setReg in scheduler step (f)): they coexist.
 *  - Detach: the destructor clears the chip's hooks (clearHooks) and the log stops sampling (a later step hook call is a no-op).
 *    IntegrationFixture::rebootKeepingFlash resets the chips: it calls onReboot() on every attached log (events cleared, hooks
 *    installed again on the fresh chip, pending arms dropped).
 *
 * armSpiFailure (HAL_ERROR) and armSpiTimeoutAfter (HAL_TIMEOUT): see the functions.
 * runCommandOnWrite: see the function.
 */
#ifndef HOSTTEST_INTEGRATION_CHIPEVENTLOG_H_
#define HOSTTEST_INTEGRATION_CHIPEVENTLOG_H_

#include "IntegrationFixture.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class ChipEventLog {
public:
	enum class Kind : uint8_t {
		Write,	//!< a register write frame the chip received
		Enable, //!< the driver enable pin level changed (seen at a step end)
	};
	struct Event {
		Kind kind = Kind::Write;
		uint64_t timeUs = 0;	//!< Write: start of the frame (FakeTmc4671::Frame::timeUs); Enable: time at the end of the scheduler step
		uint8_t addr = 0;		//!< Write only
		uint32_t data = 0;		//!< Write only
		size_t chipFrame = 0;	//!< Write only: index of the frame in chip.log() (frames of any address, reads included)
		bool enable = false;	//!< level of DRV_ENABLE (GPIOE pin 7) when sampled (Write: at the write, Enable: the new level)
	};
	static constexpr size_t npos = static_cast<size_t>(-1);
	using DataPredicate = std::function<bool(uint32_t data)>;

	ChipEventLog(IntegrationFixture &fx, FakeTmc4671 &chip);
	~ChipEventLog();
	ChipEventLog(const ChipEventLog &) = delete;
	ChipEventLog &operator=(const ChipEventLog &) = delete;

	const std::vector<Event> &events() const { return events_; }
	/** True if every write frame to a writable register that the chip logged since the log attached (or rebooted) is in the log (see Coexistence) */
	bool complete() const;

	// --- queries ---
	/** Index of the first Write event to `addr` at or after `from` whose data satisfies `pred` (any data if empty); npos if none */
	size_t firstWrite(uint8_t addr, const DataPredicate &pred = {}, size_t from = 0) const;
	/** Index of the first Enable event to `level` at or after `from`; npos if none */
	size_t firstEnable(bool level, size_t from = 0) const;
	/** True if every write to 0x64 with index < i has torque 0 and flux 0 (DS 0x64: flux [15:0], torque [31:16]). Vacuously true without such writes (count them with countWrites) */
	bool fluxTorqueZeroBefore(size_t i) const;
	/** Number of Write events to `addr` with index < i (i beyond the end: all) */
	size_t countWrites(uint8_t addr, size_t i = npos) const;
	/** Data of the last write to `addr` with index <= i (the value in force at event i, e.g. register 0x1A); nullopt if none */
	std::optional<uint32_t> valueAt(uint8_t addr, size_t i) const;
	/** Level of the enable pin at event i (the event's own field); i beyond the end: the last recorded level (initial level if empty) */
	bool enableAt(size_t i) const;

	// --- user hooks (see Coexistence) ---
	/** Hook run by the log after it recorded a write to `addr` (replaces the log's own user hook of that address) */
	void onWrite(uint8_t addr, FakeTmc4671::WriteHook fn);
	/** Hook run by the log before the chip computes the answer of a read of `addr` */
	void onRead(uint8_t addr, FakeTmc4671::ReadHook fn);

	// --- SPI failure ---
	/**
	 * Fails the next blocking SPI transfer of hspi1 that carries the register access (chip, address, write or read) with HAL_ERROR:
	 * exactly that frame, identical to HostTarget::spiFailNextBlocking(&hspi1, HAL_ERROR) on that frame (the data is transferred and
	 * applied, ErrorCode HAL_SPI_ERROR_FLAG, result HAL_ERROR, SpiTransfer::completed false). How: spiFailNextBlocking decides at the
	 * START of the transfer (it takes the flag before the responder, hence before any chip hook, runs), so a chip hook can not arm
	 * it for its own frame. The hook sets hspi1.ErrorCode = HAL_SPI_ERROR_FLAG inside the frame instead; the fake derives the result
	 * from ErrorCode after the responder returned, so the transfer record is the same as with the flag. The next transfer resets
	 * ErrorCode (the HAL functions of the fake do) and succeeds. The firmware ignores the status of HAL_SPI_TransmitReceive
	 * (SPI.cpp:176-183): what it sees is the data. Fires once. Status other than HAL_ERROR is refused (fixture fault): a HAL_TIMEOUT
	 * transfer delivers nothing to the chip, so no chip hook (which runs after delivery) can make it: use armSpiTimeoutAfter.
	 */
	void armSpiFailure(uint8_t addr, bool onWrite, HAL_StatusTypeDef status);
	/**
	 * HAL_TIMEOUT for the frame AFTER the first frame that carries (address, write or read): when that frame is seen, the log calls
	 * spiFailNextBlocking(&hspi1, HAL_TIMEOUT), so the very next blocking transfer of hspi1 (any chip) times out (nothing transferred,
	 * the chip never sees it, the receive buffer keeps its old content). This is the nearest EXACT alternative to a content-bound
	 * timeout (see armSpiFailure): the case knows which frame follows. Fires once.
	 */
	void armSpiTimeoutAfter(uint8_t addr, bool onWrite);
	/** Index in spiTransfers(&hspi1) of the transfer the last armed failure hit; -1 if none yet */
	int64_t failedTransferIndex() const { return failedTransfer_; }

	// --- command in the middle of the FFB thread ---
	/**
	 * Once, when the chip next receives a write to `addr`, sends the CDC command `cdcLine` (a newline is appended) at the END of the
	 * scheduler step in which the write happened (step (g)); the command threads (priority 37 / 32, above FFBMAIN 30) process it in
	 * step (a) of the next step, before the timers and before the FFB thread's next cycle. WHAT IT PRODUCES: the command runs between
	 * two ACTIVATIONS of the writing thread (for the FFB thread: between two FFB cycles; a trigger on the last frame of a cycle, such as
	 * the W64 of the torque, puts the command's frames in front of the next cycle's first frame). Measured with tmc.0.torqueP=:
	 * W64 (trigger), then the command's W54 W56 W58 W5A R63 W63, then the writer's next R6B. The SPI semaphore is free, so a command that
	 * writes TMC registers runs in full.
	 * WHAT IT DOES NOT PRODUCE (limit, no shared change possible): a command between two frames INSIDE one activation of the writer.
	 * The scheduler is cooperative, a task can not be activated from inside another (HostRtos::runUntilBlocked refuses while a task
	 * runs) and the writer yields only at blocking calls: parking it inside the frame (vTaskDelay) was tried and is worse (the parked
	 * writer holds the SPI semaphore, so the command waits, and the resumed writer retakes it for its next frame before the command's
	 * threads run: W64, R6B, then the command's frames). Consequently a command's read-modify-write (R63 .. W63) can not straddle the
	 * FFB thread's own read-modify-write of the same register (setMotionMode's R63 / W63 are in one activation): that race can not be
	 * produced with this helper. A write from the test thread (E-stop exti, async SPI completion) triggers it the same way.
	 * Needs the log's step sampling (see Coexistence): if it was dropped, firstFault() reports it and a pending command is never sent.
	 */
	void runCommandOnWrite(uint8_t addr, const std::string &cdcLine);
	enum class CommandState : uint8_t { Idle, Armed, Fired, Sent, Done };
	CommandState commandState() const { return cmdState_; }
	/** Times the command was sent to the device (0 or 1) */
	uint32_t commandRuns() const { return cmdRuns_; }
	/** Index in events() of the write that triggered it; npos if not fired */
	size_t commandTriggerEvent() const { return cmdEvent_; }
	/** Reply text received so far (ends with a newline when complete) */
	const std::string &commandReply() const { return cmdReply_; }
	/** Writes of the same chip (all addresses) that the log recorded after the trigger until the reply was complete */
	size_t writesDuringCommand() const { return cmdWritesDuring_; }

	/** Non-empty text if the log can no longer be trusted: its step-end sampling was dropped (another observer replaced sched.stepEndHook: scheduler steps since attach differ from the log's own hook calls) or the chip's presence changed since attach. Part of firstFault() */
	std::string hookFault() const;

	// --- called by the fixture ---
	void onReboot();
	void detachFromFixture();

private:
	struct Armed {
		bool active = false;
		bool onWrite = false;
		uint8_t addr = 0;
		bool timeoutAfter = false;
	};
	void install();
	void recordWrite(FakeTmc4671 &chip, uint8_t addr, uint32_t data);
	void readHook(FakeTmc4671 &chip, uint8_t addr);
	void stepEnd();
	bool pinLevel() const;
	size_t countAllWrites() const;
	void faultOnce(const std::string &what);

	IntegrationFixture *fx_;
	FakeTmc4671 *chip_;
	std::vector<Event> events_;
	bool lastLevel_ = false;
	uint64_t stepsBase_ = 0;	//!< sched.stats().steps at attach / reboot
	uint64_t stepsSeen_ = 0;	//!< calls of the log's own step hook since then
	bool presentAtAttach_ = true;
	size_t chipLogBase_ = 0; //!< chip.log().size() at attach / reboot: frames before it are not expected in the log
	std::map<uint8_t, FakeTmc4671::WriteHook> userWrite_;
	std::map<uint8_t, FakeTmc4671::ReadHook> userRead_;
	Armed fail_;
	int64_t failedTransfer_ = -1;
	CommandState cmdState_ = CommandState::Idle;
	uint8_t cmdAddr_ = 0;
	std::string cmdLine_;
	uint32_t cmdRuns_ = 0;
	size_t cmdEvent_ = npos;
	std::string cmdReply_;
	size_t cmdWritesDuring_ = 0;
	std::shared_ptr<bool> alive_;
};

#endif
