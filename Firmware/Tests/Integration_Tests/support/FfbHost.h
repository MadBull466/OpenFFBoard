/*
 * FfbHost.h
 *
 * The host's PID side for an IntegrationFixture (WO I1-1), and TorqueTrace, the observer of the torque the firmware commands at
 * the chip. Every phase-I1 test of ffb_integration_tests builds on these.
 *
 * Host side rule (charter I1_WORK_ORDERS.md rule 1): no firmware struct or header is used for reports. The report descriptor is read
 * over USB once (GET_DESCRIPTOR type 0x22, IntegrationFixture::readHidReportDescriptor), parsed (support/HidReportDescriptor) and
 * every report is built and decoded by pid::PidReportBuilder from it. Feature reports go through the control endpoint (HID 1.11
 * 7.2.2 Set_Report, 7.2.1 Get_Report; wValue = report type 3 (Feature) << 8 | report ID; the data stage starts with the report ID
 * byte, which the descriptor-derived report bytes() already contain), output reports through the interrupt OUT endpoint
 * (UsbHost::hidWrite). Oracle: USB_PID1_01.pdf 5.11.1 (PID Pool), 5.11.2 (Create New Effect, Block Load, Block Free), 5.12 (PID
 * State).
 *
 * WHICH SCHEDULER STEP PROCESSES A SENT REPORT. send() / sendRaw() put the report on the OUT endpoint (HostDcd), then call
 * ChainScheduler::pumpUsb(): step (a) WITHOUT time: the "USB" thread (tud_task) runs until blocked, takes the OUT data and calls
 * the firmware's HID handler (HidFFB::hidOut), then the command threads. So the effect table of the firmware is already updated when
 * send() returns, and nothing needs simulated time. The first FFB cycle that can act on it is the one of the NEXT
 * ChainScheduler::advanceMs step, in which (a) runs once more (nothing left), then the timers fire (b) and FFBMAIN runs (c) if its
 * TIM13 update came in that step (1 step in about 1000 has none, IB 1). The torque it produces is written to 0x64 in that cycle.
 * Feature SET_REPORT (createEffect) is processed the same way (control transfer, status stage, pump).
 *
 * The host's HID polling: after every send / feature transfer and in run() the simulated host takes the IN endpoint
 * (IntegrationFixture::drainHidIn, which keeps every report in hidInReports). The PID State reports of the firmware
 * (HidFFB::sendStatusReport) are decoded from there by the descriptor's PID State report.
 *
 * Checks: every USB handshake is REQUIREd (a stalled or NAKed transfer fails the test at once), report length and report ID of
 * a decoded feature report are checked by the builder. A decoded value is reported as the descriptor declares it, never "fixed":
 * IB 4: PID Block Load's "RAM Pool Available" (usage 0xAC) is declared as an Array field (Feature 0x00), the PID document's 5.11.2
 * example has a Variable field. BlockLoad::ramPoolAvailable holds the raw value of the declared field and ramPoolAvailableIsArray
 * tells how it is declared; the host decodes it as the declared type (an Array item reports the index of the selected control, HID
 * 1.11 6.2.2.5).
 */
#ifndef HOSTTEST_INTEGRATION_FFBHOST_H_
#define HOSTTEST_INTEGRATION_FFBHOST_H_

#include "HidReportDescriptor.h"
#include "IntegrationFixture.h"
#include "PidReportBuilder.h"

#include <cstdint>
#include <string>
#include <vector>

class FfbHost {
public:
	/** Decoded PID Block Load report (USB_PID1_01.pdf 5.11.2) */
	struct BlockLoad {
		bool decoded = false;			//!< the bytes had the descriptor's length and report ID and every field was found
		std::vector<uint8_t> raw;		//!< the bytes as received (report ID first)
		std::vector<std::string> errors; //!< decode errors

		bool indexDeclared = false;
		int64_t effectBlockIndex = 0;	//!< Effect Block Index (usage 0x22), raw value
		bool statusDeclared = false;
		int64_t status = 0;				//!< raw value of the Block Load Status array field
		uint32_t statusUsage = 0;		//!< the usage the status value selects (use::BlockLoadSuccess / Full / Error), 0 if it selects none
		bool success() const { return statusUsage == pid::use::BlockLoadSuccess; }
		bool full() const { return statusUsage == pid::use::BlockLoadFull; }
		bool error() const { return statusUsage == pid::use::BlockLoadError; }
		bool ramPoolDeclared = false;
		int64_t ramPoolAvailable = 0;	//!< RAM Pool Available (usage 0xAC), raw value of the field as the descriptor declares it
		bool ramPoolAvailableIsArray = false; //!< IB 4: the descriptor declares it as an Array (not Variable)
		uint32_t ramPoolAvailableBits = 0;
	};

	/** Result of createEffect: the effect type asked for and the Block Load that followed */
	struct CreateResult {
		uint32_t effectType = 0;
		BlockLoad load;
	};

	/** Decoded PID Pool report (USB_PID1_01.pdf 5.11.1) */
	struct PoolResult {
		bool decoded = false;
		std::vector<uint8_t> raw;
		std::vector<std::string> errors;
		bool hasRamPoolSize = false, hasSimultaneousEffectsMax = false, hasDeviceManagedPool = false, hasSharedParameterBlocks = false;
		int64_t ramPoolSize = 0;
		int64_t simultaneousEffectsMax = 0;
		int64_t deviceManagedPool = 0;
		int64_t sharedParameterBlocks = 0;
	};

	/** Decoded PID State input report (USB_PID1_01.pdf 5.12): the flags the descriptor declares */
	struct PidState {
		uint64_t timeUs = 0; //!< when the simulated host took the report
		std::vector<uint8_t> raw;
		bool devicePaused = false, actuatorsEnabled = false, safetySwitch = false, actuatorPower = false, effectPlaying = false;
		bool hasDevicePaused = false, hasActuatorsEnabled = false, hasSafetySwitch = false, hasActuatorPower = false, hasEffectPlaying = false;
		std::vector<std::string> errors; //!< decode errors (wrong length / ID, no PID State report in the descriptor): the flags are then not valid
	};

	/** The fixture must have brought the chain up (bringUp()): the constructor reads and parses the descriptor over USB (REQUIREs) */
	explicit FfbHost(IntegrationFixture &fixture);

	IntegrationFixture &fixture() { return fx_; }
	const hid::HidReportDescriptor &descriptor() const { return desc_; }
	const pid::PidReportBuilder &builder() const { return builder_; }

	// --- feature reports (control endpoint) ---
	/** Control SET_REPORT, report type Feature (3), the report's ID, data = report.bytes(); REQUIREs the handshake, then the USB step and the IN drain */
	void setFeature(const pid::PidReport &report);
	/** Control GET_REPORT, report type Feature, wLength = the descriptor's report length (ID byte included); REQUIREs the handshake and that data came back. @return the bytes */
	std::vector<uint8_t> getFeature(const hid::Report &report);

	/**
	 * Create New Effect (feature SET_REPORT with the effect type selected, PID 5.11.2), then GET_REPORT of PID Block Load,
	 * decoded. `byteCount` >= 0 also sets the Byte Count field (Generic Desktop 0x3B, custom effects only; -1 leaves it 0).
	 */
	CreateResult createEffect(uint32_t effectTypeUsage, int64_t byteCount = -1);
	/** GET_REPORT of PID Block Load without a create (the last answer the device keeps) */
	BlockLoad getBlockLoad();
	/** GET_REPORT of PID Pool, decoded */
	PoolResult getPool();

	// --- output reports (interrupt OUT endpoint) ---
	/** hidWrite, REQUIRE an ACK, then the USB step (see "WHICH SCHEDULER STEP", above) and the IN drain */
	void send(const pid::PidReport &report);
	/** Same for arbitrary bytes (short / long / undeclared ID reports of F1-H5). Does not REQUIRE the handshake. @return the handshake */
	HostDcd::Handshake sendRaw(const std::vector<uint8_t> &bytes);

	// --- PID State capture ---
	/** Takes the IN endpoint (drainHidIn) and decodes every captured report whose ID is the descriptor's PID State report, oldest first */
	std::vector<PidState> states();
	/** Number of PID State reports captured so far (a mark for statesSince) */
	size_t stateMark();
	/** The reports captured after the first `mark` ones */
	std::vector<PidState> statesSince(size_t mark);
	/** The latest report. @return false if none */
	bool latestState(PidState &out);

	// --- 0x64 writes made by the host's own transfers ---
	/**
	 * The firmware runs in the USB step of every send / setFeature / sendRaw and may write 0x64 there (a write caused by a report is
	 * not an FFB cycle's write). Those frames are counted here (all transfers since construction) and, if a TorqueTrace is observed
	 * with observe(), taken out of the trace's per-cycle `writes` (TorqueTrace::skipLog), so writesPerCycleMax() reflects FFB cycles only.
	 */
	uint32_t pumpWrites() const { return pumpWrites_; }
	void observe(class TorqueTrace *trace) { trace_ = trace; }

	// --- time ---
	/** Advances the chain n ms in 1 ms steps, the host taking the IN endpoint after each step */
	void run(uint32_t ms);

	// --- configuration over CDC (the user's path) ---
	/**
	 * Sends one command line over CDC and REQUIREs a reply that is not an error. Reply format of the firmware
	 * (StringCommandInterface::formatReply, CommandInterface.cpp:118-161): "[" + command text + "|" + value + "]\n", the value being
	 * the integer (CommandReplyType::INT, generateReplyValueString CommandInterface.cpp:214), "OK" for an acknowledge (:221, and for an empty
	 * reply list :139), and "ERR" (:153), "NOT_FOUND" (:135) or "None" (:155) for a failed command. The helper REQUIREs "[...|...]\n" and a
	 * value that is none of ERR, NOT_FOUND, None. @return the whole reply
	 */
	std::string command(const std::string &line);
	/** Sends a get command ("axis.0.power?") and returns the integer of the reply value; REQUIREs that the whole value is an integer */
	int64_t getValue(const std::string &line);

	// --- decoders (static: usable on hand-built bytes) ---
	static BlockLoad decodeBlockLoad(const pid::PidReportBuilder &b, const std::vector<uint8_t> &bytes);
	static PoolResult decodePool(const pid::PidReportBuilder &b, const std::vector<uint8_t> &bytes);
	/** A decode error is recorded in PidState::errors and, if failTests, reported with FAIL_CHECK */
	static PidState decodeState(const pid::PidReportBuilder &b, const std::vector<uint8_t> &bytes, uint64_t timeUs = 0, bool failTests = true);
	/** The value of a CDC reply "[text|value]\n": text after the last '|' up to the last ']'. @return false if there is no such frame */
	static bool replyValueText(const std::string &reply, std::string &valueText);

private:
	/** The host takes the IN endpoint before every transfer, as a polling host does (an undrained report blocks the next IN report) */
	void beforeTransfer() { fx_.drainHidIn(); }
	/** After a USB step caused by the host: count the 0x64 writes of the step and keep them out of the trace */
	void afterPump(size_t logMark);
	size_t logMark() const { return fx_.chip1.log().size(); }

	IntegrationFixture &fx_;
	class TorqueTrace *trace_ = nullptr;
	uint32_t pumpWrites_ = 0;
	std::vector<uint8_t> descBytes_;
	hid::HidReportDescriptor desc_;
	pid::PidReportBuilder builder_;
};

/**
 * TorqueTrace: what the firmware commands at the chip, one sample per FFB cycle.
 *
 * A sample is keyed by the FFB cycle index: ChainScheduler::Stats::ffbCycles right after that cycle ran (the n-th cycle since the
 * scheduler reset has index n). It is taken at the end of the scheduler step (ChainScheduler step (g), stepEndHook) in which the cycle
 * ran, so it also sees what the plant or the position script wrote in step (f). Content:
 *  - cycle: the index; timeUs: HostPlatform::nowUs() at the end of that step (the cycle ran earlier in the same step);
 *  - torque and flux: register 0x64 PID_TORQUE_FLUX_TARGET of the chip, DS 0x64 (TMC4671-LA datasheet Rev 2.07): flux [15:0], torque
 *    [31:16], both signed 16 bit. The value is the register as it stands at the end of the step (FakeTmc4671::reg), so a cycle without
 *    a write keeps the last value: the driver writes 0x64 only on change (Axis.cpp:488-495);
 *  - writes: the number of write frames to 0x64 in the log during that step.
 * A step without an FFB cycle (IB 1: about one step per 1000 ms has none) produces no sample: its 0x64 writes (a write by the TMC
 * thread or at a start / stop) are counted in strayWrites(), and the value carries over to the next sample. Cycles that elapsed
 * without a sample (a step that ran more than one cycle; the scheduler runs at most one per step) are counted in missedCycles().
 * Start / stop: start() installs ChainScheduler::stepEndHook (one observer at a time: a second start is a test failure), stop()
 * removes it; the destructor stops. onStep() is the feed, public so a self-test can drive it without a scheduler.
 */
class TorqueTrace {
public:
	struct Sample {
		uint32_t cycle = 0;
		uint64_t timeUs = 0;
		int16_t torque = 0;
		int16_t flux = 0;
		uint32_t writes = 0;
	};

	static constexpr uint8_t kRegTorqueFluxTarget = 0x64;

	explicit TorqueTrace(const FakeTmc4671 &chip) : chip_(chip) {}
	~TorqueTrace();
	TorqueTrace(const TorqueTrace &) = delete;
	TorqueTrace &operator=(const TorqueTrace &) = delete;

	/** Starts sampling every step of the fixture's scheduler (baseline: now). Clears the earlier samples */
	void start(IntegrationFixture &fixture);
	/** Starts without a scheduler: the caller feeds onStep (baseline cycle count `ffbCycles`) */
	void startManual(uint32_t ffbCycles);
	void stop();
	bool running() const { return running_; }

	/** One step: the cycle counter after the step, the simulated time at its end. Samples if the counter moved */
	void onStep(uint32_t ffbCycles, uint64_t nowUs);

	const std::vector<Sample> &samples() const { return samples_; }
	uint32_t strayWrites() const { return strayWrites_; }
	uint32_t missedCycles() const { return missed_; }
	/** The sample of that cycle index, nullptr if there is none */
	const Sample *find(uint32_t cycle) const;
	/** Torque of the sample of that cycle (REQUIREs that the sample exists) */
	int16_t torqueAt(uint32_t cycle) const;
	int16_t fluxAt(uint32_t cycle) const;
	/**
	 * Query without REQUIRE: true if there is a sample for every cycle in [fromCycle, toCycle] (inclusive) and all have the same torque
	 * (flux for steadyFluxQuery); `value` is then that value, else `why` says which cycle is missing or differs.
	 */
	bool steadyQuery(uint32_t fromCycle, uint32_t toCycle, int16_t &value, std::string &why) const;
	bool steadyFluxQuery(uint32_t fromCycle, uint32_t toCycle, int16_t &value, std::string &why) const;
	/** REQUIREs a sample for every cycle in [fromCycle, toCycle] (inclusive) and that all have the same torque. @return that torque */
	int16_t steadyValue(uint32_t fromCycle, uint32_t toCycle) const;
	/** Same for the flux field */
	int16_t steadyFlux(uint32_t fromCycle, uint32_t toCycle) const;
	/**
	 * Takes the 0x64 write frames logged since the last step out of the next sample (they were made by a host transfer's USB step, not by
	 * an FFB cycle): the log mark moves to the end and the writes are added to pumpWrites().
	 */
	void skipLog();
	uint32_t pumpWrites() const { return pumpWrites_; }
	/** Largest number of 0x64 writes of any sample (0 if none) */
	uint32_t writesPerCycleMax() const;

private:
	const FakeTmc4671 &chip_;
	ChainScheduler *sched_ = nullptr;
	bool running_ = false;
	uint32_t lastCycles_ = 0;
	size_t logMark_ = 0;
	uint32_t strayWrites_ = 0;
	uint32_t missed_ = 0;
	uint32_t pumpWrites_ = 0;
	std::vector<Sample> samples_;
};

#endif
