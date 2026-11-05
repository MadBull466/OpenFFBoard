/*
 * FakeOdrivePeer.h
 *
 * Fake ODrive axis on the fake CAN bus (ffb_driver_tests only). Peer of the ODrive CAN driver
 * (Firmware/FFBoard/UserExtensions/Src/ODriveCAN.cpp). Requirements: Tests/claude/charters/driver/ODRIVE_REQUIREMENTS.md,
 * section "Fake peer: FakeOdrivePeer".
 *
 * Sources: "CAN Protocol - ODrive Documentation 0.5.6" (cited "SPEC <section>") and its message table "CAN Messages"
 * as CSV (cited "CSV <cmd id>"). Lead decision L1: the 0.5.6 spec is the oracle (the driver was written for 0.5.4).
 *
 * MODELS
 *  - One axis per node id (SPEC "CAN Frame"): standard 11 bit id = node << 5 | cmd, node 0..0x3F, cmd 0..0x1F. The peer
 *    decodes the master frames of the fake HAL (HostTarget::canTxFrames()) addressed to its node by `id >> 5` and
 *    `id & 0x1F`. Frames of other nodes are counted (foreignFrames) and ignored. Only frames the bus acknowledged
 *    (CanTxResult::Sent) are seen: a frame that is still pending (TX mailboxes not completed), failed or aborted never
 *    reached the ODrive. All multibyte values are little endian (SPEC).
 *  - Strict decoding of the master frames per CSV (violations(), expectViolation(), unexpectedViolations()):
 *      0x07 Set Axis Requested State  data frame, DLC 4, uint32 @0            (CSV 0x007)
 *      0x0B Set Controller Modes      data frame, DLC 8, int32 @0, int32 @4   (CSV 0x00B)
 *      0x0E Set Input Torque          data frame, DLC 4, float32 @0           (CSV 0x00E)
 *      0x09 Get Encoder Estimates     remote frame (RTR), DLC 0               (CSV 0x009, SPEC call and response)
 *      0x17 Get Bus Voltage / Current remote frame (RTR), DLC 0               (CSV 0x017)
 *      0x10 Start Anticogging         data frame, no signals (CSV 0x010)
 *    A violation is: extended id addressed to this node (the driver uses standard ids only), RTR on a set message, a
 *    data frame on a call-and-response message, wrong DLC, any other command id (the driver never sends them).
 *    Frames with a violation are logged but not applied and not answered. 0x10 with DLC != 0 is NOT a violation
 *    (lead decision L3): it is recorded in undocumented().
 *    ASSUMPTION: the DLC of a master RTR request is 0 (the charter requires it; neither SPEC nor CSV states a DLC
 *    for a remote frame).
 *  - Responses (SPEC "call and response"): RTR 0x09 -> data frame with the same id, DLC 8, float32 position estimate
 *    @0 and float32 velocity estimate @4 (CSV 0x009); RTR 0x17 -> same id, DLC 8, float32 bus voltage @0, float32 bus
 *    current @4 (CSV 0x017). Units are not stated by CSV (SPEC lists turns and turns/s for the estimates, V and A).
 *  - Cyclic messages (SPEC "Cyclic Messages", defaults): heartbeat 0x01 every 100 ms (on by default), encoder estimates
 *    0x09 every 10 ms (off by default: keeps logs small). Heartbeat layout (CSV 0x001): axis error uint32 @0, current
 *    state uint8 @4, motor error flag bit 0 of byte 5, encoder error flag bit 0 of byte 6, controller error flag
 *    bit 0 of byte 7, trajectory done flag bit 7 of byte 7. Driven by simulated time: process() / advance(ms).
 *    The first cyclic frame comes one period after the construction of the peer (or after the cyclic message was
 *    enabled). ASSUMPTION: the real axis sends its first heartbeat shortly after boot; the phase against the
 *    firmware's own 500 ms loop is not specified.
 *  - Axis state model (Low confidence, lead decision L2): a requested state (0x07) is applied after
 *    requestApplyDelayMs (default 0). A calibration state (3 full, 4 motor, 6 index search, 7 encoder offset, 13 hall
 *    phase: numbers = the driver's own enums, ODriveCAN.h:22) returns to IDLE (1) after calibrationMs (default 2000,
 *    an arbitrary duration, SPEC gives none). A closed loop request (8) stays. Refusal mode: requests are recorded and
 *    ignored (state unchanged), optionally with an axis error word. A new request replaces a pending one. The numbers
 *    of the states are the driver's, they are NOT in SPEC / CSV.
 *  - Fault injection: silent (no frames of the axis at all, requests are still decoded and logged), no responses only,
 *    late responses (delay), responses and cyclic frames on a wrong node id, short frames (DLC override, the missing
 *    bytes are 0), extended id frames, raw frames (inject), an error state / bus-off or uncompleted mailboxes through
 *    the HostTarget CAN API directly (not wrapped here).
 *  - Records: decoded frame log {time, id, cmd, rtr, dlc, data, decoded values}, last torque, last modes, requested
 *    states (all, and the refused ones), anticogging count, frames sent to the driver.
 *
 * DOES NOT MODEL
 *  The ODrive control loop and motor, its own error generation (axis error is set by the test), the CANopen messages,
 *  messages other than the ones above (a command id outside the table is a violation), baud rate mismatch (the HAL
 *  fake does not model bit timing), arbitration and bus timing (frames take no time; a response is delivered within
 *  process(), after the optional delay), the 0.5.4 / 0.5.6 differences (L1), time jumps: when simulated time is
 *  advanced by more than one period between two process() calls (e.g. a firmware Delay) the missed cyclic frames are
 *  not delivered, one frame with the state at the time of the call is (count in skippedCyclic()); OdriveFixture avoids
 *  this by stepping the time itself.
 *
 * CONFIDENCE
 *  id and payload encoding: High (CSV). Call and response: High (SPEC). Cyclic timing (period): Medium (SPEC default,
 *  phase assumed). Axis state behaviour and the state numbers: Low (L2).
 *
 * ASSUMPTIONS (summary): DLC 0 of RTR requests, first cyclic frame one period after start, calibration duration 2000 ms,
 * a calibration state ends in IDLE, units of the estimates, requests while calibrating are applied at once.
 */
#ifndef HOSTTEST_FAKEODRIVEPEER_H_
#define HOSTTEST_FAKEODRIVEPEER_H_

#include "HostTarget.h"

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

class FakeOdrivePeer {
public:
	// Command ids (CSV)
	static constexpr uint8_t CMD_HEARTBEAT = 0x01;
	static constexpr uint8_t CMD_SET_STATE = 0x07;
	static constexpr uint8_t CMD_ENCODER_ESTIMATES = 0x09;
	static constexpr uint8_t CMD_SET_MODES = 0x0B;
	static constexpr uint8_t CMD_SET_TORQUE = 0x0E;
	static constexpr uint8_t CMD_ANTICOGGING = 0x10;
	static constexpr uint8_t CMD_BUS_VOLTAGE = 0x17;

	// Axis state numbers: the DRIVER's enum numbers (ODriveCAN.h:22), not from SPEC / CSV (L2, Low confidence)
	static constexpr uint32_t STATE_UNDEFINED = 0;
	static constexpr uint32_t STATE_IDLE = 1;
	static constexpr uint32_t STATE_FULL_CALIBRATION = 3;
	static constexpr uint32_t STATE_MOTOR_CALIBRATION = 4;
	static constexpr uint32_t STATE_ENCODER_INDEX_SEARCH = 6;
	static constexpr uint32_t STATE_ENCODER_OFFSET_CALIBRATION = 7;
	static constexpr uint32_t STATE_CLOSED_LOOP = 8;
	static constexpr uint32_t STATE_HALL_PHASE_CALIBRATION = 13;
	/** 3, 4, 6, 7, 13: states that return to IDLE after calibrationMs */
	static bool isCalibrationState(uint32_t state);

	/** One frame of the driver (master) addressed to this node, or that violated the protocol */
	struct Frame {
		uint32_t timeMs = 0; //!< HAL_GetTick() when the driver queued the frame (CanTxFrame::time)
		uint32_t id = 0;
		uint8_t cmd = 0; //!< id & 0x1F
		bool rtr = false;
		bool extId = false;
		uint8_t dlc = 0;
		std::array<uint8_t, 8> data{};
		bool accepted = true; //!< false: a violation was recorded, the frame was not applied
		/** Little endian 32 bit word 0 or 1 of the data bytes */
		uint32_t u32(int word = 0) const;
		int32_t i32(int word = 0) const { return static_cast<int32_t>(u32(word)); }
		float f32(int word = 0) const;
	};

	enum class ViolationKind : uint8_t {
		ExtendedId,		//!< Extended id frame for this node (driver uses standard ids)
		RtrMismatch,	//!< RTR on a set message (0x07, 0x0B, 0x0E, 0x10) or a data frame on 0x09 / 0x17 (SPEC call and response)
		WrongDlc,		//!< DLC differs from CSV (0x07: 4, 0x0B: 8, 0x0E: 4, RTR 0x09 / 0x17: 0)
		UnknownCommand, //!< Command id not in the set the driver uses
	};
	struct Violation {
		ViolationKind kind = ViolationKind::UnknownCommand;
		int cmd = -1;
		uint32_t id = 0;
		uint8_t dlc = 0;
		uint32_t timeMs = 0;
		std::string text() const;
	};
	struct Undocumented {
		uint8_t cmd = 0;
		uint8_t dlc = 0;
		uint32_t timeMs = 0;
	};
	/** A frame the peer put on the bus towards the driver */
	struct Sent {
		uint32_t timeMs = 0;
		uint32_t id = 0;
		bool extId = false;
		bool rtr = false;
		uint8_t dlc = 0;
		std::array<uint8_t, 8> data{};
		uint32_t accepted = 0; //!< canReceive() result: message pending callbacks (0 = rejected by the filters / not delivered)
	};

	explicit FakeOdrivePeer(uint8_t nodeId = 0);

	/** Power-on state of the model with node `nodeId` kept: IDLE, no error, heartbeat 100 ms on, encoder cyclic off, no faults, empty logs, cursor at the end of the current TX frame log */
	void reset();
	uint8_t nodeId() const { return node_; }
	/** Changes the node id the peer answers to (the other node ids are foreign) */
	void setNodeId(uint8_t node) { node_ = node & 0x3F; }

	// --- time ---
	/** Decodes new driver frames, answers them, emits due cyclic frames and state changes (in time order up to HostPlatform::nowUs()) */
	void process();
	/** Advances the simulated time by `ms` in 1 ms steps and calls process() at each step (the driver thread is not run) */
	void advance(uint32_t ms);
	/** Earliest due time (us) of a cyclic frame, a pending state change or a delayed response; UINT64_MAX if there is none */
	uint64_t nextEventUs() const;
	/** Ignore the TX frames queued so far (after HostTarget::canClearTxFrames()). OdriveFixture::clearTxFrames does both */
	void syncCursor();

	// --- axis model (what the heartbeat reports) ---
	uint32_t state() const { return state_; }
	/** Sets the state directly (no delay, no calibration timer) */
	void setState(uint32_t s);
	void setAxisError(uint32_t e) { axisError_ = e; }
	uint32_t axisError() const { return axisError_; }
	void setFlags(bool motor, bool encoder, bool controller, bool trajectoryDone);
	/** Delay between a state request and the state change (default 0) */
	void setRequestApplyDelayMs(uint32_t ms) { applyDelayMs_ = ms; }
	/** Time in a calibration state before it ends in IDLE (default 2000) */
	void setCalibrationMs(uint32_t ms) { calibrationMs_ = ms; }
	/** Refuse every request: recorded in refusedStates(), the state stays; axis error `error` is set if non-zero */
	void setRefuseRequests(bool refuse, uint32_t error = 0);
	/** Refuse requests for one state number only */
	void refuseState(uint32_t state, bool refuse = true);
	void setEncoder(float pos, float vel) { pos_ = pos; vel_ = vel; }
	void setBus(float voltage, float current) { vbus_ = voltage; ibus_ = current; }

	// --- cyclic messages ---
	void setHeartbeat(bool enabled, uint32_t periodMs = 100);
	void setEncoderCyclic(bool enabled, uint32_t periodMs = 10);
	bool heartbeatEnabled() const { return hbEnabled_; }
	bool encoderCyclicEnabled() const { return encEnabled_; }
	/** Number of cyclic frames not delivered because process() was called late (see header) */
	uint32_t skippedCyclic() const { return skippedCyclic_; }
	/** Payload of the heartbeat for the current model state (CSV 0x001) */
	std::array<uint8_t, 8> heartbeatPayload() const;
	/** Sends a heartbeat / encoder estimates / bus voltage frame now (faults apply). @return canReceive result */
	uint32_t sendHeartbeat();
	uint32_t sendEncoderEstimates();
	uint32_t sendBusVoltage();
	/** Sends any frame as it is (no faults, no silence), e.g. a frame for another node. @return canReceive result */
	uint32_t inject(uint32_t id, const std::vector<uint8_t> &data, bool extId = false, bool rtr = false);

	// --- fault injection (persistent until cleared) ---
	/** Silent axis: no heartbeat, no responses, no manual sends. Requests are still decoded and logged */
	void setSilent(bool silent) { silent_ = silent; }
	bool silent() const { return silent_; }
	/** false: heartbeats continue, RTR requests are not answered */
	void setRespond(bool respond) { respond_ = respond; }
	/** Delay of the responses to RTR requests (late response) */
	void setResponseDelayMs(uint32_t ms) { responseDelayMs_ = ms; }
	/** Axis frames carry node id `node` instead of the peer's (wrong node id); -1: off */
	void setFaultNode(int node) { faultNode_ = node; }
	/** Axis frames are sent with this DLC (short frame; bytes beyond are 0); -1: off */
	void setFaultDlc(int dlc) { faultDlc_ = dlc; }
	/** Axis frames are sent with an extended id (same value node << 5 | cmd) */
	void setFaultExtended(bool ext) { faultExt_ = ext; }
	void clearFaults();

	// --- records ---
	const std::vector<Frame> &log() const { return log_; }
	void clearLog();
	/** Logged frames with command id `cmd`, in order */
	std::vector<Frame> framesWithCmd(uint8_t cmd) const;
	size_t count(uint8_t cmd) const;
	/** Number of RTR frames with command id `cmd` */
	size_t rtrCount(uint8_t cmd) const;
	bool torqueSeen() const { return torqueSeen_; }
	float lastTorque() const { return lastTorque_; }
	bool modesSeen() const { return modesSeen_; }
	int32_t lastControlMode() const { return lastControl_; }
	int32_t lastInputMode() const { return lastInput_; }
	/** Requested states in order (accepted frames, applied or refused) */
	const std::vector<uint32_t> &requestedStates() const { return requested_; }
	const std::vector<uint32_t> &refusedStates() const { return refused_; }
	uint32_t anticoggingCount() const { return anticogging_; }
	uint32_t foreignFrames() const { return foreign_; }
	const std::vector<Sent> &sent() const { return sent_; }
	const std::vector<Violation> &violations() const { return violations_; }
	const std::vector<Undocumented> &undocumented() const { return undocumented_; }
	/** Declares a violation as expected (cmd -1: any command). Expected violations are not reported by unexpectedViolations() */
	void expectViolation(ViolationKind kind, int cmd = -1);
	std::vector<Violation> unexpectedViolations() const;

private:
	struct Expected {
		ViolationKind kind;
		int cmd;
	};
	struct Out {
		uint64_t dueUs;
		uint32_t id;
		bool ext;
		bool rtr;
		std::vector<uint8_t> data;
	};

	void poll();
	void decode(const HostTarget::CanTxFrame &f);
	void violate(ViolationKind kind, const Frame &f);
	void applyRequest(uint32_t requested);
	void enterState(uint32_t s);
	/** Builds an axis frame with the faults applied and delivers it (or queues it with `delayMs`). @return canReceive result (0 if queued / suppressed) */
	uint32_t emit(uint8_t cmd, const std::array<uint8_t, 8> &payload, uint32_t delayMs);
	uint32_t deliver(uint32_t id, const std::vector<uint8_t> &data, bool ext, bool rtr);
	std::array<uint8_t, 8> encoderPayload() const;
	std::array<uint8_t, 8> busPayload() const;

	uint8_t node_;
	size_t cursor_ = 0;

	uint32_t state_ = STATE_IDLE;
	uint32_t axisError_ = 0;
	bool motorFlag_ = false, encoderFlag_ = false, controllerFlag_ = false, trajDone_ = false;
	uint32_t applyDelayMs_ = 0;
	uint32_t calibrationMs_ = 2000;
	bool refuseAll_ = false;
	uint32_t refusalError_ = 0;
	std::set<uint32_t> refuseStates_;
	// pending state change: either the delayed application of a request or the end of a calibration
	bool pendingValid_ = false;
	uint64_t pendingDueUs_ = 0;
	uint32_t pendingState_ = 0;
	float pos_ = 0, vel_ = 0, vbus_ = 24.0f, ibus_ = 0;

	bool hbEnabled_ = true;
	uint32_t hbPeriodMs_ = 100;
	uint64_t hbNextUs_ = 0;
	bool encEnabled_ = false;
	uint32_t encPeriodMs_ = 10;
	uint64_t encNextUs_ = 0;
	uint32_t skippedCyclic_ = 0;

	bool silent_ = false;
	bool respond_ = true;
	uint32_t responseDelayMs_ = 0;
	int faultNode_ = -1;
	int faultDlc_ = -1;
	bool faultExt_ = false;

	std::deque<Out> out_;
	std::vector<Frame> log_;
	std::vector<Sent> sent_;
	std::vector<Violation> violations_;
	std::vector<Undocumented> undocumented_;
	std::vector<Expected> expected_;
	std::vector<uint32_t> requested_, refused_;
	bool torqueSeen_ = false, modesSeen_ = false;
	float lastTorque_ = 0;
	int32_t lastControl_ = 0, lastInput_ = 0;
	uint32_t anticogging_ = 0, foreign_ = 0;
};

#endif
