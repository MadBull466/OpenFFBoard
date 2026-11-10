/*
 * FakeRmdPeer.h
 *
 * Fake MyActuator RMD motor on the fake CAN bus (ffb_driver_tests only). Peer of the RMD CAN driver
 * (Firmware/FFBoard/UserExtensions/Src/RmdMotorCAN.cpp). Requirements: Tests/claude/charters/driver/RMD_REQUIREMENTS.md,
 * section "Fake: MyActuator RMD motor on the fake CAN".
 *
 * Source: MyActuator "CAN Bus Motor Motion Protocol V4.3" (cited "SP <section>"; the PDF is supplied by the owner, git-ignored,
 * never copied beyond short citations). Pattern: FakeOdrivePeer (it reads the frames the driver transmitted from
 * HostTarget::canTxFrames() that the bus acknowledged and injects replies with HostTarget::canReceive()).
 *
 * MODELS
 *  - One motor per node id (SP 1.1.2: ID 1..32; the driver's defaults are node 1 for instance 1 and node 2 for instance 2).
 *    Command frames are standard 11 bit id 0x140 + ID, DLC 8 (SP 1.1.2). The multi motor id 0x280 is answered by every
 *    present motor (broadcast). Frames with another id are counted (foreignFrames()) and ignored. Replies carry the standard
 *    id 0x240 + ID, DLC 8. Only frames the bus acknowledged (CanTxResult::Sent) are seen.
 *  - Commands decoded (SP section in brackets); everything is little endian:
 *      0x80 motor shutdown [2.17], 0x81 motor stop [2.18], 0x77 brake release [2.27], 0x78 brake lock [2.28]:
 *           reply = the sent frame. 0x80 / 0x81 clear the running flag.
 *      0xA1 torque closed loop [2.19]: data[4..5] iq int16 (0.01 A/LSB), the other bytes 0x00; reply: data[1] temperature
 *           int8, data[2..3] iq int16, data[4..5] speed int16 (1 dps), data[6..7] shaft angle int16 (1 degree). Sets the
 *           running flag. With followTorque the iq of the next replies is the one last commanded.
 *      0x92 multi turn angle [2.12]: reply data[4..7] int32 (0.01 degree/LSB), data[1..3] 0.
 *      0x94 single turn angle [2.13]: reply data[6..7] uint16 (0..35999, 0.01 degree), data[1..5] 0.
 *      0x9A status 1 [2.14]: reply data[1] motor temperature int8, data[2] MOS temperature int8, data[3] brake state,
 *           data[4..5] voltage uint16 (0.1 V), data[6..7] error flags.
 *      0xB5 model [2.33]: reply data[1..7] seven ASCII characters.
 *      0xB6 active reply [2.34]: data[1] command to report cyclically (0x60 0x61 0x62 0x92 0x9A 0x9C 0x9D 0x9E),
 *           data[2] enable 0 / 1, data[3..4] interval (unit 10 ms: SP example value 1 = 20 ms, so interval + 1 periods of
 *           10 ms), other bytes 0x00. No reply to the 0xB6 itself. After enabling, the motor sends the selected reply
 *           every (interval + 1) * 10 ms until it is disabled; several commands alternate independently. Of the commands
 *           in the SP list replyPayload() models 0x92, 0x9A and 0xA1 (and 0x94); the others report {cmd, 0 x 7}.
 *  - Strictness (violations(), expectViolation(), unexpectedViolations(), the fixture fails the test on an undeclared
 *    one): extended id, RTR frame, DLC other than 8, a command byte the firmware is not expected to send (anything other than
 *    0x80, 0xA1, 0x92, 0x9A, 0xB5, 0xB6; allowCommand(cmd) declares others), a non-zero byte in a field the SP marks NULL, a
 *    0xB6 command byte outside the SP list, an enable byte other than 0 / 1. A frame with a violation is logged but not answered
 *    and not applied.
 *  - Fault injection (persistent until clearFaults()): silent (no replies at all, frames are still decoded and logged),
 *    delayed replies, replies with the id of another node, a wrong command byte in the reply, a short DLC, raw frames
 *    (inject), unsolicited replies (sendReply).
 *  - Records: every frame received {time, id, cmd, data}, count per command byte, last iq, frames sent to the driver.
 *
 * DOES NOT MODEL
 *  Motor dynamics (the iq, speed and angle values are set by the test or follow nothing), the PID / speed / position modes
 *  (0xA2-0xA9), brake hardware, the multi motor semantics beyond a broadcast reply, motion mode 0x400 + ID, RS485 CRC,
 *  the motor's own error generation, baud rate mismatch, arbitration and bus time (a reply is delivered within process()),
 *  time jumps: when simulated time is advanced by more than one period between two process() calls the missed cyclic
 *  replies are not delivered (one is, counted in skippedCyclic()); RmdFixture steps the time itself.
 *
 * CONFIDENCE
 *  Ids, frame layouts and units of the used commands: High (SP). Cyclic reply period: Medium (SP example "1 = 20 ms", the
 *  first reply one period after enabling is an assumption). Reply latency (zero), model string content, the default
 *  state values: Low / arbitrary.
 */
#ifndef HOSTTEST_FAKERMDPEER_H_
#define HOSTTEST_FAKERMDPEER_H_

#include "HostTarget.h"

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

class FakeRmdPeer {
public:
	/** One command frame of the driver for this motor (or a rejected one) */
	struct Frame {
		uint32_t timeMs = 0; //!< HAL_GetTick() when the driver queued the frame
		uint32_t id = 0;
		uint8_t cmd = 0; //!< data[0]
		bool rtr = false;
		bool extId = false;
		uint8_t dlc = 0;
		std::array<uint8_t, 8> data{};
		bool accepted = true; //!< false: a violation was recorded, the frame was not applied or answered
		bool broadcast = false; //!< id 0x280
	};

	enum class ViolationKind : uint8_t {
		ExtendedId,	   //!< extended id for this motor's id
		Rtr,		   //!< remote frame
		WrongDlc,	   //!< DLC != 8
		UnknownCommand, //!< command byte not in the set the driver uses
		NonZeroNull,   //!< non-zero byte in a field the SP marks NULL
		BadReportCmd,  //!< 0xB6 with a command byte outside the SP list
		BadEnable,	   //!< 0xB6 with an enable byte other than 0 / 1
	};
	struct Violation {
		ViolationKind kind = ViolationKind::UnknownCommand;
		int cmd = -1;
		uint32_t id = 0;
		uint8_t dlc = 0;
		uint32_t timeMs = 0;
		std::string text() const;
	};
	/** A frame the peer put on the bus towards the driver */
	struct Sent {
		uint32_t timeMs = 0;
		uint32_t id = 0;
		bool extId = false;
		uint8_t dlc = 0;
		std::array<uint8_t, 8> data{};
		uint32_t accepted = 0; //!< canReceive() result (0 = rejected by the filters / not delivered)
	};

	explicit FakeRmdPeer(uint8_t nodeId = 1);

	/** Power-on state: no errors, 25 degC, 480 (48.0 V), model "X8S2V10", iq / speed / angles 0, no cyclic replies, no faults, empty logs, cursor at the end of the TX frame log */
	void reset();
	uint8_t nodeId() const { return node_; }
	void setNodeId(uint8_t node) { node_ = node; }

	// --- time ---
	void process();
	void advance(uint32_t ms);
	uint64_t nextEventUs() const;
	void syncCursor();

	// --- motor state (what the replies report) ---
	void setTemperature(int8_t t) { temp_ = t; }
	void setMosTemperature(int8_t t) { mosTemp_ = t; }
	void setBrakeState(uint8_t b) { brake_ = b; }
	void setVoltage(uint16_t v01) { voltage_ = v01; } //!< 0.1 V units
	void setErrors(uint16_t e) { errors_ = e; }
	void setModel(const std::string &m); //!< up to 7 characters, padded with 0
	void setMultiTurn(int32_t angle001) { multiTurn_ = angle001; } //!< 0.01 degree units
	void setSingleTurn(uint16_t angle001) { singleTurn_ = angle001; }
	void setShaftAngle(int16_t degrees) { shaftAngle_ = degrees; }
	void setSpeed(int16_t dps) { speed_ = dps; }
	void setIq(int16_t iq) { iq_ = iq; }
	void setFollowTorque(bool on) { followTorque_ = on; }
	int8_t temperature() const { return temp_; }
	uint16_t errors() const { return errors_; }
	int16_t iq() const { return iq_; }
	bool running() const { return running_; }

	// --- replies ---
	/** Builds the reply payload of command `cmd` for the current state (0xA1, 0x92, 0x94, 0x9A, 0xB5; others: {cmd, 0...}) */
	std::array<uint8_t, 8> replyPayload(uint8_t cmd) const;
	/** Sends the reply of command `cmd` now (unsolicited; faults apply). @return canReceive result */
	uint32_t sendReply(uint8_t cmd);
	/** Sends any frame as it is (no faults, no silence). @return canReceive result */
	uint32_t inject(uint32_t id, const std::vector<uint8_t> &data, bool extId = false, bool rtr = false);
	bool cyclicEnabled(uint8_t cmd) const { return cyclic_.count(cmd) != 0; }
	uint32_t cyclicPeriodMs(uint8_t cmd) const;
	uint32_t skippedCyclic() const { return skippedCyclic_; }

	// --- fault injection ---
	void setSilent(bool silent) { silent_ = silent; }
	bool silent() const { return silent_; }
	void setResponseDelayMs(uint32_t ms) { responseDelayMs_ = ms; }
	/** Replies carry the id 0x240 + `node` instead of the peer's; -1 off */
	void setFaultNode(int node) { faultNode_ = node; }
	/** Replies carry command byte `cmd` in data[0] instead of the real one; -1 off */
	void setFaultCmdByte(int cmd) { faultCmd_ = cmd; }
	/** Replies are sent with this DLC (bytes beyond are 0); -1 off */
	void setFaultDlc(int dlc) { faultDlc_ = dlc; }
	void clearFaults();
	/** Declares a command byte the driver may send in addition to the expected set (0x80, 0xA1, 0x92, 0x9A, 0xB5, 0xB6); it is answered with the sent frame */
	void allowCommand(uint8_t cmd) { allowed_.insert(cmd); }

	// --- records ---
	const std::vector<Frame> &log() const { return log_; }
	void clearLog();
	std::vector<Frame> framesWithCmd(uint8_t cmd) const;
	size_t count(uint8_t cmd) const;
	int16_t lastCommandedIq() const { return lastIq_; }
	bool iqSeen() const { return iqSeen_; }
	uint32_t foreignFrames() const { return foreign_; }
	const std::vector<Sent> &sent() const { return sent_; }
	const std::vector<Violation> &violations() const { return violations_; }
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
		std::vector<uint8_t> data;
	};
	struct Cyclic {
		uint64_t nextUs;
		uint32_t periodMs;
	};

	void poll();
	void decode(const HostTarget::CanTxFrame &f);
	void violate(ViolationKind kind, const Frame &f);
	uint32_t emit(const std::array<uint8_t, 8> &payload, uint32_t delayMs);
	uint32_t deliver(uint32_t id, const std::vector<uint8_t> &data);

	uint8_t node_;
	size_t cursor_ = 0;

	int8_t temp_ = 25, mosTemp_ = 25;
	uint8_t brake_ = 1;
	uint16_t voltage_ = 480;
	uint16_t errors_ = 0;
	std::array<uint8_t, 7> model_{};
	int32_t multiTurn_ = 0;
	uint16_t singleTurn_ = 0;
	int16_t shaftAngle_ = 0, speed_ = 0, iq_ = 0;
	bool followTorque_ = true;
	bool running_ = false;

	std::map<uint8_t, Cyclic> cyclic_;
	uint32_t skippedCyclic_ = 0;

	bool silent_ = false;
	uint32_t responseDelayMs_ = 0;
	int faultNode_ = -1, faultCmd_ = -1, faultDlc_ = -1;
	std::set<uint8_t> allowed_;

	std::deque<Out> out_;
	std::vector<Frame> log_;
	std::vector<Sent> sent_;
	std::vector<Violation> violations_;
	std::vector<Expected> expected_;
	int16_t lastIq_ = 0;
	bool iqSeen_ = false;
	uint32_t foreign_ = 0;
};

#endif
