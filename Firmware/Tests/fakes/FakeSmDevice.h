/*
 * FakeSmDevice.h
 *
 * Fake SimpleMotion V2 device on the motor UART (huart3), peer of MotorSimplemotion (ffb_driver_tests only, phase P15 T1).
 * Requirements: Tests/claude/charters/driver/SIMPLEMOTION_REQUIREMENTS.md, section "Fake: SimpleMotion device on the motor UART".
 *
 * Sources: the SimpleMotion V2 reference library and documentation supplied by the owner (Tests/shared/specs/SimpleMotionV2-master,
 * git-ignored, never committed): sm485.h (frame ids), simplemotion.c (smSendSMCMD, smFastUpdateCycle), sm_consts.c (CRC tables),
 * simplemotion_defs.h (subpacket types and sizes, SMP_RETURN_PARAM_ADDR / _LEN, parameter ids). The CRC functions below are the
 * peer's OWN bitwise implementations (CRC-8 poly 0x07 init 0x52; CRC-16/ARC, wire order = ARC low byte first, i.e. the
 * "byte swapped" value of the library high byte first); the firmware's tables (CRC.cpp) are not used. Checked by the cases of
 * claude/cases/driver/test_sm_setup.cpp: crc8({0x10,0x01,0,0,0,0}, 0x52) = 0x46, crc16arc("123456789") = 0xBB3D (library value
 * 0x3DBB). UART HAL contracts as in platform/src/host_platform.cpp (single shot receive slot, interrupt context for the callbacks).
 *
 * MODELS (one device per FakeSmDevice, listening on one UART handle)
 *  - Fast update (frame id 0x10, 7 bytes {0x10, addr, write1 LE, write2 LE, crc8}): checked, the setpoint (write1, as int16) is
 *    recorded, the reply {0x11, position LE, status LE, crc8} (6 bytes) is built from position16 / status16.
 *  - Instant command (frame id 0x24, {0x24, len, addr, payload[len], crc16 low, crc16 high}, len + 5 bytes): checked, the
 *    payload is decoded into subpackets (type = top 2 bits: 0 = write 32 bit value (4 bytes, 30 bit value), 1 = write 24 bit
 *    value (3 bytes, 22 bit), 2 = set write address (2 bytes, 14 bit address); type 3 is not an outbound type). Parameters
 *    are a map (default table in the constructor); a write stores the value (special addresses: 9 SMP_RETURN_PARAM_ADDR,
 *    10 SMP_RETURN_PARAM_LEN, 554 SMP_SYSTEM_CONTROL records a restart request). EVERY outbound subpacket returns one reply
 *    subpacket holding the value of the parameter at the current return address in the current return format (LEN 0 = 32 bit,
 *    1 = 24 bit, 2 = 16 bit (14 bit value), 3 = status byte): the reply of the subpacket that changes the address or the
 *    length already uses the new setting. Reply: {0x25, len, addr, subpackets, crc16}.
 *  - Reply delivery: the peer never delivers inside the firmware's transmit or receive-arm call (the firmware sets its wait
 *    flags after them). The reply is delivered by pump(), called by the fixture's process() and by onPoll() (HAL_IWDG_Refresh,
 *    i.e. refreshWatchdog() in the busy-wait loops of readParameter / writeParameter). One pump step delivers first the
 *    transmit-complete interrupt of the transmit, then the reply bytes (in chunks, see below). When nothing could be delivered
 *    onPoll advances the simulated time by 1 ms, so a silent peer ends the 10 ms wait of the firmware.
 *  - Records: all frames received (frames()), parameter writes in order (writes()), fast update setpoints, restart requests.
 *  - Strictness (violations(), expectViolation(), unexpectedViolations()): a frame header other than 0x10 / 0x24, a fast frame
 *    not 7 bytes, an instant frame whose length does not match len + 5, a wrong CRC, a subpacket that runs over the payload end,
 *    an outbound subpacket of type 3, a write address above 0x3FFF cannot occur (14 bit). A frame with a violation is recorded
 *    but not applied and not answered. A frame for another address is counted (foreignFrames()) and ignored when
 *    filterAddress is set; by default (filterAddress false) the peer answers every address and puts the address of the
 *    request into its reply (B-SM-1: the driver's own address is not known to the test).
 *
 * FAULT INJECTION (persistent until clearFaults()): silent (decodes, answers nothing), corruptCrc (flips the lowest bit of the
 * last CRC byte of every reply), replyAddress (address byte of instant replies; -1 = the request's), truncateReply (drops the
 * last n bytes of the reply), padSubpackets (appends n 32 bit subpackets to an instant reply: valid CRC, announced length
 * grows, an overlong reply), chunk / gapMs (bytes per pump step and simulated ms between steps; 0 = all bytes in one step),
 * errorOnByte (the byte index of the reply with huart.ErrorCode set to 1 while it is delivered), autoTxComplete (false: the
 * transmit-complete interrupt is not delivered, the write-enable pin stays high), injectRaw (queues raw bytes as the next
 * reply, no checks).
 *
 * DOES NOT MODEL: the RS485 electrical layer and collisions (a frame sent while a reply is pending drops that reply and counts
 * droppedReplies()), baud timing (frames take no time), buffered commands, SM bus status subpacket semantics beyond the ACK
 * byte, device-side meaning of any parameter beyond the table, write protection of read-only parameters, device addresses
 * other than the one echoed, the SM_RETURN_STATUS fault bits.
 *
 * CONFIDENCE: High for frame formats, CRCs and subpacket sizes (reference library, checked by script and by the setup cases);
 * Medium for the return-format semantics (one reply subpacket per outbound subpacket, the new format applies at once: taken
 * from the comments of simplemotion_defs.h only); the default parameter table is an assumption (healthy ABN servo in torque mode).
 */
#ifndef HOSTTEST_FAKESMDEVICE_H_
#define HOSTTEST_FAKESMDEVICE_H_

#include "HostPlatform.h"

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

class FakeSmDevice : public HostPlatform::UartPeer {
public:
	// Parameter ids (simplemotion_defs.h)
	static constexpr uint16_t SMP_RETURN_PARAM_ADDR = 9;
	static constexpr uint16_t SMP_RETURN_PARAM_LEN = 10;
	static constexpr uint16_t SMP_CUMULATIVE_STATUS = 13;
	static constexpr uint16_t SMP_FB1_DEVICE_SELECTION = 493;
	static constexpr uint16_t SMP_FAULTS = 552;
	static constexpr uint16_t SMP_STATUS = 553;
	static constexpr uint16_t SMP_SYSTEM_CONTROL = 554;
	static constexpr uint16_t SMP_CONTROL_MODE = 559;
	static constexpr uint16_t SMP_ENCODER_PPR = 565;
	static constexpr uint16_t SMP_ACTUAL_BUS_VOLTAGE = 900;
	static constexpr uint16_t SMP_ACTUAL_TORQUE = 901;
	static constexpr uint16_t SMP_CONTROL_BITS1 = 2533;
	static constexpr uint16_t SMP_DEVICE_TYPE = 6020;
	// Status word bits (simplemotion_defs.h STAT_*)
	static constexpr uint16_t STAT_FAULTSTOP = 1U << 5;
	static constexpr uint16_t STAT_INITIALIZED = 1U << 12;

	/** CRC-8, polynomial 0x07, no reflection, no final xor (sm_consts.c table_crc8), bitwise */
	static uint8_t crc8(const uint8_t *buf, size_t len, uint8_t init);
	/** CRC-16/ARC (reflected polynomial 0xA001, init 0), bitwise. On the wire the low byte comes first (= the library's byte swapped value high byte first) */
	static uint16_t crc16arc(const uint8_t *buf, size_t len);

	/** `huart` is the UART_HandleTypeDef* the peer listens on (&huart3) */
	explicit FakeSmDevice(const void *huart);
	~FakeSmDevice() override;

	// --- device state (set by the test) ---
	uint8_t address = 1;	 //!< only used when filterAddress is set
	bool filterAddress = false;
	uint16_t position16 = 0; //!< fast reply: lowest 16 bits of the position feedback
	uint16_t status16 = STAT_INITIALIZED; //!< fast reply: first 16 bits of SMP_STATUS (healthy: initialized, no faultstop)
	std::map<uint16_t, uint32_t> params; //!< parameter table (defaults: healthy ABN1 servo, 1000 pulses/rev, torque mode, 48 V)
	uint32_t retAddr = 0;	   //!< current SMP_RETURN_PARAM_ADDR
	uint32_t retLen = 0;	   //!< current SMP_RETURN_PARAM_LEN (device default 0 = 32 bit: an assumption)

	// --- fault injection ---
	bool silent = false;
	bool corruptCrc = false;
	int replyAddress = -1;
	size_t truncateReply = 0;
	size_t padSubpackets = 0;
	size_t chunk = 0;
	uint32_t gapMs = 0;
	int errorOnByte = -1;
	bool autoTxComplete = true;
	void injectRaw(const std::vector<uint8_t> &bytes);
	void clearFaults();

	// --- records ---
	enum class Kind { Fast, Instant, Invalid };
	struct Frame {
		Kind kind;
		std::vector<uint8_t> bytes;
		uint64_t timeUs; //!< simulated time of the transmit
	};
	const std::vector<Frame> &frames() const { return frameLog; }
	size_t frameCount() const { return frameLog.size(); }
	void clearFrames();
	/** Parameter writes in order: (address, value) as decoded from the subpackets */
	const std::vector<std::pair<uint16_t, uint32_t>> &writes() const { return writeLog; }
	/** Fast update setpoints (write1 as int16) in order */
	const std::vector<int16_t> &setpoints() const { return setpointLog; }
	/** Parameter ids requested through SMP_RETURN_PARAM_ADDR writes, in order */
	const std::vector<uint32_t> &returnAddressRequests() const { return retAddrLog; }
	uint32_t restartRequests() const { return restarts; }
	uint32_t foreignFrames() const { return foreign; }
	uint32_t droppedReplies() const { return dropped; }
	uint32_t bytesNotAccepted() const { return notAccepted; } //!< reply bytes the firmware did not take (receive not re-armed)
	bool receiveArmed() const { return armed; }
	bool txCompletePending() const { return txPending; }
	bool replyPending() const { return !pendingReply.empty(); }
	size_t pendingReplyBytes() const { return pendingReply.size(); }
	uint32_t unknownReads() const { return unknownReadCount; } //!< reads of an id that has no table entry (the value 0 is returned)

	struct Violation {
		std::string message;
		const std::string &text() const { return message; }
	};
	const std::vector<Violation> &violations() const { return violationLog; }
	void expectViolation() { violationsExpected = true; }
	std::vector<Violation> unexpectedViolations() const { return violationsExpected ? std::vector<Violation>{} : violationLog; }

	/** One delivery step. Returns true when something was delivered (transmit-complete interrupt or reply bytes) */
	bool pump();
	/** pump() until nothing is pending or `maxSteps` steps were made (gaps advance the simulated time) */
	void pumpAll(unsigned maxSteps = 64);

	// HostPlatform::UartPeer
	void onTransmit(const void *huart, const uint8_t *data, uint16_t size) override;
	void onReceiveArmed(const void *huart) override;
	void onPoll() override;

private:
	const void *port;
	bool armed = false;
	bool txPending = false;
	bool violationsExpected = false;
	std::vector<uint8_t> pendingReply;
	size_t replyPos = 0;
	bool chunkStarted = false;
	bool gapPaid = false;
	std::vector<Frame> frameLog;
	std::vector<std::pair<uint16_t, uint32_t>> writeLog;
	std::vector<int16_t> setpointLog;
	std::vector<uint32_t> retAddrLog;
	std::vector<Violation> violationLog;
	uint32_t restarts = 0, foreign = 0, dropped = 0, notAccepted = 0, unknownReadCount = 0;

	void violate(const std::string &msg) { violationLog.push_back({msg}); }
	void handleFast(const std::vector<uint8_t> &f);
	void handleInstant(const std::vector<uint8_t> &f);
	void queueReply(std::vector<uint8_t> bytes);
	uint32_t readParam(uint32_t id);
	void appendReplySubpacket(std::vector<uint8_t> &out);
};

#endif
