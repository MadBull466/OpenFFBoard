/*
 * FakeVescPeer.h
 *
 * Fake VESC on the fake CAN bus (ffb_driver_tests only). Peer of the VESC CAN driver
 * (Firmware/FFBoard/UserExtensions/Src/VescCAN.cpp). Requirements: Tests/claude/charters/driver/VESC_REQUIREMENTS.md,
 * section "Fake peer: FakeVescPeer" and "Facts".
 *
 * Sources:
 *  - SRC <path>:<line>  = the VESC firmware source supplied by the user, Tests/shared/specs/bldc-master (git-ignored, never
 *    committed): VESC firmware 7.01, test build 1 (SRC conf_general.h:24-27). Lead decision L1: SRC 7.01 is the oracle
 *    for everything SPEC does not give.
 *  - SPEC <slide>       = "VESC 6 CAN Formats v0.1" slides (extended id layout, packet numbers 0-27, STATUS 1-5).
 *    A third-party document with the author's guesses; only its definite statements are used.
 *
 * MODELS (one VESC unit per FakeVescPeer, unit id default 0x01)
 *  - Ids (SPEC id layout; SRC datatypes.h:1161-1231): extended 29 bit id = unit id (bits 0-7) | packet << 8 (bits 8-15),
 *    bits 16-28 zero. The peer decodes the frames of the driver from HostTarget::canTxFrames() that the bus acknowledged
 *    (CanTxResult::Sent; a frame still pending, failed or aborted never reached the VESC). A frame is for this unit if
 *    its low id byte is the unit id or 255 (broadcast, SRC comm_can.c:1609); other frames are counted (foreignFrames())
 *    and ignored.
 *  - Strict decoding of the driver's frames (violations(), expectViolation(), unexpectedViolations()). A violation is:
 *    a standard id frame, an RTR frame (SPEC "Remote bit 0"), id bits 16-28 set, a packet other than the ones the driver
 *    uses (5/6/7 are replies, the driver never sends them; 8, 10, 17, 56 are accepted), a wrong DLC
 *    (SET_CURRENT_REL 4 per SPEC; PROCESS_SHORT_BUFFER 3 for COMM_FW_VERSION and 7 for COMM_GET_VALUES_SELECTIVE;
 *    PING 1; POLL_ROTOR_POS 0), a PROCESS_SHORT_BUFFER flag other than 0 (the driver always asks for "process and reply
 *    to the sender id"), a COMM id other than 0 and 50, a GET_VALUES_SELECTIVE mask the fake cannot answer (bits other
 *    than 8, 15, 16). Frames with a violation are logged but not applied and not answered.
 *    The 8th byte of a PROCESS_SHORT_BUFFER GET_VALUES request (DLC 8) is NOT a violation: the driver sends it with an
 *    uninitialised value (B-V2); the VESC reads the mask only (SRC commands.c:393). It is recorded in undocumented().
 *  - Replies built exactly like SRC comm_can_send_buffer (comm_can.c:443-505): payload (a COMM message, COMM id first)
 *    <= 6 bytes: one PROCESS_SHORT_BUFFER frame [own id, flag 1, payload...], DLC len+2; longer: FILL_RX_BUFFER frames
 *    [offset, up to 7 bytes] at offsets 0, 7, 14, ... while offset <= 255, then FILL_RX_BUFFER_LONG frames
 *    [offset hi, offset lo, up to 6 bytes] for the rest, then PROCESS_RX_BUFFER [own id, flag 1, length BE, CRC BE]
 *    (DLC 6). Reply id = requester id (payload byte 0 of the request) | packet << 8. The CRC is CRC-16/XMODEM (poly
 *    0x1021, init 0, no reflection, no final xor), high byte first, over the whole COMM message including its COMM id
 *    byte (SRC util/crc.c:26-64, comm_can.c:497-500). It is the peer's OWN implementation of SRC crc.c (table copied
 *    from SRC; the firmware's CRC.cpp is not used), checked against the check value crc("123456789") = 0x31C3.
 *  - COMM_FW_VERSION reply in the 7.01 layout (SRC commands.c:231-293): [0, major, minor, HW_NAME\0, UUID (12),
 *    pairing_done, FW_TEST_VERSION_NUMBER (0 = release), HW_TYPE (0), custom cfg num, phase filters flag, QML hw flag,
 *    QML app flag, nrf flags, FW_NAME\0, hw crc (4, BE)], length 29 + n + m. Configurable: major, minor, test version,
 *    pairing, HW_NAME, FW_NAME, UUID. NOTE the order: pairing flag FIRST, test version number SECOND (B-V6).
 *  - COMM_GET_VALUES_SELECTIVE reply (SRC commands.c:384-479): [50, mask u32 BE, fields of the set bits in ascending
 *    order]. Modelled bits: 8 v_in int16 big endian = (int16)(V * 10) (truncation, buffer_append_float16 SRC
 *    buffer.c:71-72), 15 fault code uint8 (mc_fault_code, 0 = none), 16 pid_pos int32 = (int32)(degrees * 1e6). Configurable
 *    v_in, fault code, pid_pos. Example (mask 0x8100, 24.0 V, fault 0): 32 00 00 81 00 00 F0 00, CRC 0x523A.
 *  - POLL_ROTOR_POS (SRC comm_can.c:2023-2030): request id unit | 56 << 8, payload ignored (DLC 0 expected); reply id
 *    OWN id | 56 << 8 (not the requester's), DLC 4, int32 BE = (int32)(degrees * 100000.0). A broadcast request is
 *    answered by every present peer.
 *  - PING/PONG (SRC comm_can.c:660-669, 1843-1861): PING DLC 1 [sender id]; PONG to id sender | 18 << 8, DLC 2 [own id,
 *    hw type 0]. An unsolicited PONG can be sent (sendPong).
 *  - STATUS 1-5 (SPEC; SRC comm_can.c:1216-1274): id = OWN id | packet << 8, optional (off by default, as in the
 *    VESC: can_status_msgs = 0, appconf_default.h:66-80), cyclic with setStatusCyclic or manual with sendStatus.
 *  - Timeout (SRC appconf_default.h:27-31, timeout.c:186-253): only SET commands (SET_CURRENT_REL here) reset a
 *    timer; the VESC has timed out when more than timeoutMs (default 1000, 0 = disabled) passed since the last SET
 *    (or since the reset of the peer if none came); on timeout it brakes with 0 A (effectiveRatio() is 0).
 *  - Records: decoded frame log, last current ratio, request counts per COMM id and per packet, frames sent to the
 *    driver, undocumented items.
 *  - Time: process() (called by VescFixture) decodes new frames, answers them and sends due frames in time order.
 *
 * FAULT INJECTION (persistent until clearFaults()): silent (decodes but sends nothing at all), no replies, delayed
 * replies, corrupt CRC, a missing FILL frame, FILL frames in reverse order, FILL_RX_BUFFER_LONG framing for the whole
 * buffer, short DLC (optionally for one packet), wrong sender id (the unit id the peer puts into its frames), wrong
 * destination of the replies, unsolicited PONG, raw frames (inject), long names (FwInfo, reply > 64 bytes).
 *
 * DOES NOT MODEL
 *  Motor control, current limits (the SET_CURRENT_REL ratio is only recorded, SRC mc_interface.c:744-757 not
 *  modelled), the optional off-delay of SET_CURRENT_REL (DLC >= 6, SRC comm_can.c:1816-1820; the driver never sends it:
 *  DLC != 4 is a violation), encoder hardware, the other SET_* / CONF_* / BMS / IO packets, GET_VALUES bits other than
 *  8, 15, 16, receive buffer 512 bytes and the 3 parallel rx buffers of the real VESC (the peer never receives
 *  multi-frame messages from the driver), the VESC dropping out-of-sequence FILL frames (this is the driver's job to
 *  survive, see fault injection), arbitration and bus timing (frames take no time; a reply is delivered within
 *  process(), after the optional delay), VESC firmware versions other than 7.01 (the 5.x layouts are not determinable
 *  from SRC, charter L1), a VESC with controller id -1 ("from UUID", appconf_default.h:25; the unit id is set by the
 *  test), unit id 255 for the peer itself (255 is only the broadcast target), multi-VESC arbitration timing, time jumps:
 *  when the simulated time is advanced by more than one period between two process() calls the missed cyclic STATUS
 *  frames are not delivered (count in skippedCyclic()); VescFixture avoids this by stepping the time itself.
 *
 * CONFIDENCE
 *  Framing, buffer protocol, CRC, COMM layouts of bits 8/15/16, POLL_ROTOR_POS, PING/PONG: High (SRC 7.01).
 *  FW_VERSION layout of firmware older than 7.01: Low (not determinable, charter L1 and open question 16).
 *  Reply and timeout timing: Medium (SRC gives the 1000 ms default; the reply latency of a real VESC is not modelled).
 *  STATUS 1-5: Medium (SPEC and SRC agree field by field; unsolicited, off by default).
 *
 * ASSUMPTIONS: unit id 0x01 (a real VESC derives its id from the UUID by default), HW_NAME "60" and empty FW_NAME,
 * UUID bytes 01..0C, the 7 extra FW_VERSION flags 0, hw crc 0xDEADBEEF, replies without latency, a request with a flag
 * other than 0 is a violation (the driver only sends 0), the CRC fault flips the lowest CRC bit.
 */
#ifndef HOSTTEST_FAKEVESCPEER_H_
#define HOSTTEST_FAKEVESCPEER_H_

#include "HostTarget.h"

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

class FakeVescPeer {
public:
	// CAN_PACKET_ID (SRC datatypes.h:1161-1231)
	static constexpr uint8_t PKT_FILL_RX_BUFFER = 5;
	static constexpr uint8_t PKT_FILL_RX_BUFFER_LONG = 6;
	static constexpr uint8_t PKT_PROCESS_RX_BUFFER = 7;
	static constexpr uint8_t PKT_PROCESS_SHORT_BUFFER = 8;
	static constexpr uint8_t PKT_STATUS = 9;
	static constexpr uint8_t PKT_SET_CURRENT_REL = 10;
	static constexpr uint8_t PKT_STATUS_2 = 14;
	static constexpr uint8_t PKT_STATUS_3 = 15;
	static constexpr uint8_t PKT_STATUS_4 = 16;
	static constexpr uint8_t PKT_PING = 17;
	static constexpr uint8_t PKT_PONG = 18;
	static constexpr uint8_t PKT_STATUS_5 = 27;
	static constexpr uint8_t PKT_POLL_ROTOR_POS = 56;
	// COMM_PACKET_ID (SRC datatypes.h:969, 991, 1019)
	static constexpr uint8_t COMM_FW_VERSION = 0;
	static constexpr uint8_t COMM_ROTOR_POSITION = 22;
	static constexpr uint8_t COMM_GET_VALUES_SELECTIVE = 50;

	/** Extended id of a frame: unit id (bits 0-7) | packet << 8 */
	static constexpr uint32_t makeId(uint8_t unit, uint8_t packet) { return uint32_t(unit) | (uint32_t(packet) << 8); }

	/** CRC-16/XMODEM as in SRC util/crc.c (crc16 = crc16_rolling(0, buf, len)) */
	static uint16_t crc16(const uint8_t *buf, size_t len);
	static uint16_t crc16(const std::vector<uint8_t> &v) { return crc16(v.data(), v.size()); }

	/** Content of the COMM_FW_VERSION reply (configurable, defaults: release 7.01) */
	struct FwInfo {
		uint8_t major = 7;
		uint8_t minor = 1;
		uint8_t testVersion = 0; //!< FW_TEST_VERSION_NUMBER (0 = release); sent AFTER the pairing flag
		bool pairing = false;	 //!< pairing_done, sent BEFORE the test version
		std::string hwName = "60";
		std::string fwName = "";
		std::array<uint8_t, 12> uuid{{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}};
		uint8_t hwType = 0;
		uint32_t hwCrc = 0xDEADBEEF;
	};

	/** One frame as it goes on the bus (id, DLC = data.size()) */
	struct RawFrame {
		uint32_t id = 0;
		std::vector<uint8_t> data;
	};
	/** Options of buildBuffer() (the faults of the peer) */
	struct BufferOptions {
		bool longFill = false;	  //!< all FILL frames as FILL_RX_BUFFER_LONG (not what a real VESC does below 260 bytes)
		bool corruptCrc = false;  //!< CRC xor 0x0001
		int dropFill = -1;		  //!< index of the FILL frame (in transmission order) that is left out
		bool reverseFill = false; //!< FILL frames in reverse order
	};
	/**
	 * Frames of SRC comm_can_send_buffer(dest, payload, len, send = `flag`) sent by the unit `sender`: the id low byte
	 * is `dest`, payload byte 0 is `sender`. Static: usable by tests that need the exact frames of a reply.
	 */
	static std::vector<RawFrame> buildBuffer(uint8_t dest, uint8_t sender, uint8_t flag, const std::vector<uint8_t> &payload, const BufferOptions &opt);
	static std::vector<RawFrame> buildBuffer(uint8_t dest, uint8_t sender, uint8_t flag, const std::vector<uint8_t> &payload) { return buildBuffer(dest, sender, flag, payload, BufferOptions()); }

	/** One frame of the driver addressed to this unit (or that violated the protocol) */
	struct Frame {
		uint32_t timeMs = 0; //!< HAL_GetTick() when the driver queued the frame (CanTxFrame::time)
		uint32_t id = 0;
		uint8_t unit = 0;	//!< id & 0xFF
		uint8_t packet = 0; //!< (id >> 8) & 0xFF
		bool extId = false;
		bool rtr = false;
		uint8_t dlc = 0;
		std::array<uint8_t, 8> data{};
		bool accepted = true; //!< false: a violation was recorded, the frame was not applied
		int comm = -1;		  //!< COMM id of a PROCESS_SHORT_BUFFER request (data[2]); -1 otherwise
		/** Big endian 16 / 32 bit value at byte offset `off` of the data */
		uint16_t u16(int off = 0) const;
		uint32_t u32(int off = 0) const;
		int32_t i32(int off = 0) const { return static_cast<int32_t>(u32(off)); }
	};

	enum class ViolationKind : uint8_t {
		StandardId,	   //!< Standard id frame (the driver uses extended ids)
		RemoteFrame,   //!< RTR frame (SPEC: remote bit 0)
		UpperIdBits,   //!< Id bits 16-28 not zero
		WrongDlc,	   //!< DLC differs from the packet's format (see header)
		UnknownPacket, //!< Packet number the driver does not use
		BadFlag,	   //!< PROCESS_SHORT_BUFFER flag other than 0
		UnknownComm,   //!< COMM id other than 0 (FW_VERSION) and 50 (GET_VALUES_SELECTIVE)
		UnsupportedMask, //!< GET_VALUES_SELECTIVE mask with bits the fake cannot answer (other than 8, 15, 16)
	};
	struct Violation {
		ViolationKind kind = ViolationKind::UnknownPacket;
		int packet = -1;
		uint32_t id = 0;
		uint8_t dlc = 0;
		uint32_t timeMs = 0;
		std::string text() const;
	};
	/** A deviation that is documented but not a violation: the uninitialised 8th byte of the values request (B-V2) */
	struct Undocumented {
		uint8_t packet = 0;
		uint8_t dlc = 0;
		uint8_t extraByte = 0; //!< data[7]
		uint32_t timeMs = 0;
	};
	/** A frame the peer put on the bus towards the driver */
	struct Sent {
		uint32_t timeMs = 0;
		uint32_t id = 0;
		uint8_t packet = 0;
		uint8_t dlc = 0;
		std::array<uint8_t, 8> data{};
		uint32_t accepted = 0; //!< canReceive() result: message pending callbacks (0 = rejected by the filters / not delivered)
	};

	explicit FakeVescPeer(uint8_t unitId = 0x01);

	/** Power-on state of the model with the unit id kept: present, release 7.01, 24.0 V, fault 0, angle 0, timeout 1000 ms, no faults, STATUS off, empty logs, cursor at the end of the current TX frame log */
	void reset();
	uint8_t unitId() const { return unit_; }
	void setUnitId(uint8_t unit) { unit_ = unit; }
	/** false: the unit is not on the bus (frames are skipped: no log, no reply, no violation) */
	void setPresent(bool present) { present_ = present; }
	bool present() const { return present_; }

	// --- time ---
	/** Decodes new driver frames, answers them, sends due frames (in time order up to HostPlatform::nowUs()) */
	void process();
	/** Advances the simulated time by `ms` in 1 ms steps and calls process() at each step (the driver thread is not run) */
	void advance(uint32_t ms);
	/** Earliest due time (us) of a cyclic frame or a delayed reply; UINT64_MAX if there is none */
	uint64_t nextEventUs() const;
	/** Ignore the TX frames queued so far (after HostTarget::canClearTxFrames()). VescFixture::clearTxFrames does both */
	void syncCursor();

	// --- model values ---
	FwInfo &fw() { return fw_; }
	const FwInfo &fw() const { return fw_; }
	void setVin(float volts) { vin_ = volts; }
	float vin() const { return vin_; }
	void setFaultCode(uint8_t code) { faultCode_ = code; }
	uint8_t faultCode() const { return faultCode_; }
	/** pid_pos_now in degrees (GET_VALUES bit 16, x1e6) */
	void setPidPosDeg(double deg) { pidPos_ = deg; }
	/** encoder angle in degrees (POLL_ROTOR_POS reply, x1e5) */
	void setRotorDeg(double deg) { rotor_ = deg; }
	double rotorDeg() const { return rotor_; }
	/** Timeout of the VESC in ms (default 1000, 0 = disabled) */
	void setTimeoutMs(uint32_t ms) { timeoutMs_ = ms; }
	/** More than the timeout since the last SET command (or the reset of the peer) at the current simulated time */
	bool timedOut() const;
	/** Time (ms, HAL_GetTick) of the last SET command; the reset time if none came */
	uint32_t lastSetMs() const { return lastSetMs_; }
	/** The last SET_CURRENT_REL ratio (raw int32 / 1e5); 0 before the first */
	bool currentSeen() const { return currentSeen_; }
	int32_t lastCurrentRaw() const { return lastCurrentRaw_; }
	float lastCurrentRatio() const { return static_cast<float>(lastCurrentRaw_) / 1e5f; }
	/** Ratio the VESC applies now: 0 when timedOut() (brake current 0 A, SRC timeout.c:186-253), else the last ratio */
	float effectiveRatio() const { return timedOut() ? 0.0f : lastCurrentRatio(); }

	// --- payload builders (also usable by tests) ---
	/** COMM_FW_VERSION reply message for the current FwInfo (length 29 + n + m) */
	std::vector<uint8_t> fwVersionPayload() const;
	/** COMM_GET_VALUES_SELECTIVE reply message for `mask` with the current values; empty if the mask has bits other than 8, 15, 16 */
	std::vector<uint8_t> valuesPayload(uint32_t mask) const;
	/** The unit id the peer puts into its frames (the unit id, or the wrong one of setFaultSenderId) */
	uint8_t senderId() const { return faultSender_ >= 0 ? static_cast<uint8_t>(faultSender_) : unit_; }

	// --- sending to the driver (faults apply, except for inject) ---
	/** Sends `payload` (a COMM message) the way a reply is sent (short or FILL + PROCESS) to `dest`. @return number of delivered frames accepted by the driver's filters */
	uint32_t sendReply(uint8_t dest, const std::vector<uint8_t> &payload);
	uint32_t sendFwVersion(uint8_t dest = 0x40) { return sendReply(dest, fwVersionPayload()); }
	uint32_t sendValues(uint8_t dest = 0x40, uint32_t mask = 0x8100) { return sendReply(dest, valuesPayload(mask)); }
	/** POLL_ROTOR_POS reply: id OWN id | 56 << 8, DLC 4, int32 BE degrees * 1e5 */
	uint32_t sendRotorPosition();
	/** PONG to `dest`: id dest | 18 << 8, DLC 2 [own id, hw type 0] (unsolicited when no PING came) */
	uint32_t sendPong(uint8_t dest = 0x40);
	/** STATUS message `packet` (9, 14, 15, 16 or 27) with the status values */
	uint32_t sendStatus(uint8_t packet);
	/** Sends any frame as it is (no faults, no silence), extended id. @return canReceive result */
	uint32_t inject(uint32_t id, const std::vector<uint8_t> &data, bool extId = true);

	// --- STATUS 1-5 (off by default) ---
	struct StatusValues {
		double rpm = 0, totalCurrent = 0, duty = 0;
		double ampHours = 0, ampHoursCharged = 0, wattHours = 0, wattHoursCharged = 0;
		double tempFet = 0, tempMotor = 0, inputCurrent = 0, pidPos = 0;
		int32_t tachometer = 0;
		double vin = 24.0;
	};
	StatusValues &status() { return status_; }
	/** Cyclic STATUS frames: `mask` bit 0 STATUS, 1 STATUS_2, 2 STATUS_3, 3 STATUS_4, 4 STATUS_5; 0 = off */
	void setStatusCyclic(uint8_t mask, uint32_t periodMs = 20);
	uint32_t skippedCyclic() const { return skippedCyclic_; }

	// --- fault injection (persistent until cleared) ---
	/** Silent unit: sends nothing at all (replies, PONG, STATUS, manual sends). Requests are still decoded and logged */
	void setSilent(bool silent) { silent_ = silent; }
	bool silent() const { return silent_; }
	/** false: requests are decoded but not answered; STATUS continues */
	void setRespond(bool respond) { respond_ = respond; }
	/** Delay of the replies to requests (late reply), in ms */
	void setReplyDelayMs(uint32_t ms) { replyDelayMs_ = ms; }
	/** The CRC of PROCESS_RX_BUFFER frames is wrong (xor 0x0001) */
	void setCorruptCrc(bool on) { opt_.corruptCrc = on; }
	/** The FILL frame with this index (transmission order) is not sent; -1: off */
	void setDropFillFrame(int index) { opt_.dropFill = index; }
	/** FILL frames are sent in reverse order (out of sequence) */
	void setReverseFill(bool on) { opt_.reverseFill = on; }
	/** Whole buffers use FILL_RX_BUFFER_LONG framing (V22; a real VESC does that only beyond 259 bytes) */
	void setLongFill(bool on) { opt_.longFill = on; }
	/** Frames of the peer are sent with this DLC (bytes beyond are 0); -1: off. `packet` -1: all packets, else only that packet number */
	void setFaultDlc(int dlc, int packet = -1) {
		faultDlc_ = dlc;
		faultDlcPacket_ = packet;
	}
	/** The unit id in the peer's frames (POLL_ROTOR_POS id, payload byte 0, PONG, STATUS) is `id` instead of the unit id; -1: off */
	void setFaultSenderId(int id) { faultSender_ = id; }
	/** Replies go to this destination id (id low byte) instead of the requester's; -1: off */
	void setFaultDest(int id) { faultDest_ = id; }
	void clearFaults();

	// --- records ---
	const std::vector<Frame> &log() const { return log_; }
	void clearLog();
	/** Accepted driver frames of packet `packet`, in order */
	std::vector<Frame> framesWithPacket(uint8_t packet) const;
	size_t count(uint8_t packet) const;
	/** Number of accepted PROCESS_SHORT_BUFFER requests with COMM id `comm` */
	size_t requestCount(uint8_t comm) const;
	/** Number of accepted PING frames */
	size_t pingCount() const { return count(PKT_PING); }
	uint32_t foreignFrames() const { return foreign_; }
	const std::vector<Sent> &sent() const { return sent_; }
	const std::vector<Violation> &violations() const { return violations_; }
	const std::vector<Undocumented> &undocumented() const { return undocumented_; }
	/** Declares a violation as expected (packet -1: any packet). Expected violations are not reported by unexpectedViolations() */
	void expectViolation(ViolationKind kind, int packet = -1);
	std::vector<Violation> unexpectedViolations() const;

private:
	struct Expected {
		ViolationKind kind;
		int packet;
	};
	struct Out {
		uint64_t dueUs;
		uint32_t id;
		std::vector<uint8_t> data;
	};

	void poll();
	void decode(const HostTarget::CanTxFrame &f);
	void violate(ViolationKind kind, Frame &f);
	void handleRequest(const Frame &f);
	/** Applies the DLC fault, delivers or queues. @return canReceive result (0 if queued / suppressed) */
	uint32_t emit(uint32_t id, const std::vector<uint8_t> &data, uint32_t delayMs);
	uint32_t deliver(uint32_t id, const std::vector<uint8_t> &data);
	uint32_t destFor(uint8_t requester) const { return faultDest_ >= 0 ? static_cast<uint8_t>(faultDest_) : requester; }

	uint8_t unit_;
	bool present_ = true;
	size_t cursor_ = 0;

	FwInfo fw_;
	float vin_ = 24.0f;
	uint8_t faultCode_ = 0;
	double pidPos_ = 0, rotor_ = 0;
	uint32_t timeoutMs_ = 1000;
	uint32_t lastSetMs_ = 0;
	bool currentSeen_ = false;
	int32_t lastCurrentRaw_ = 0;

	StatusValues status_;
	uint8_t statusMask_ = 0;
	uint32_t statusPeriodMs_ = 20;
	uint64_t statusNextUs_ = 0;
	uint32_t skippedCyclic_ = 0;

	bool silent_ = false;
	bool respond_ = true;
	uint32_t replyDelayMs_ = 0;
	BufferOptions opt_;
	int faultDlc_ = -1;
	int faultDlcPacket_ = -1;
	int faultSender_ = -1;
	int faultDest_ = -1;

	std::deque<Out> out_;
	std::vector<Frame> log_;
	std::vector<Sent> sent_;
	std::vector<Violation> violations_;
	std::vector<Undocumented> undocumented_;
	std::vector<Expected> expected_;
	std::map<uint8_t, size_t> commCount_;
	uint32_t foreign_ = 0;
};

#endif
