/*
 * VESC CAN driver: G7 encoder (V60-V64), G8 commands (V70-V72), G9 persistent settings (V80-V81).
 * Charter: Firmware/Tests/driver/VESC_REQUIREMENTS.md. Driver: Firmware/FFBoard/UserExtensions/Src/VescCAN.cpp (VC).
 *
 * Sources: SRC = VESC firmware 7.01 (Tests/bldc-master, never committed), HD = hand derivation in the comment,
 * FW = firmware behaviour (characterization). Flash addresses: Firmware/FFBoard/UserExtensions/Inc/eeprom_addresses.h
 * lines 206-211: ADR_VESC1_CANID 0x3E0, ADR_VESC1_DATA 0x3E1, ADR_VESC1_OFFSET 0x3E2, ADR_VESC2_CANID 0x3E3,
 * ADR_VESC2_DATA 0x3E4, ADR_VESC2_OFFSET 0x3E5. VESC_1 = instance 0, VESC_2 = instance 1 (VescCAN.cpp:84-92).
 *
 * Packing (VC:196, 201, 217): CANID word = (VESC id << 8) | OFFB id; DATA word = useencoder << 3;
 * OFFSET word = (uint16)(int16)(fractional offset * 10000) with the fractional offset in [-0.5, 0.5].
 *
 * Encoder: POLL_ROTOR_POS reply = int32 BE degrees * 1e5 (SRC comm_can.c:2023-2030); getPos_f = lastPos - posOffset,
 * lastPos = angle / 360 + counted turns; getPos = getCpr() (0xFFFF) * getPos_f, truncated toward zero.
 * The rotor reply of FakeVescPeer is delivered synchronously (canReceive -> canRxPendCallback), so the tests read the
 * result right after peer0.sendRotorPosition(). The driver is kept out of READY in most encoder tests, because
 * getPos_f() in READY sends a POLL_ROTOR_POS request.
 *
 * Host vs target: float is single precision on both; all float values below are chosen exactly representable or the
 * expected integers have a margin of at least 0.04 to the nearest integer. float -> int32 conversions in range are
 * defined; the one out of range case is B-V12 (KNOWN_ISSUE_UB). Flash words are uint16_t; (int16_t) of an in-range
 * int is exact on both. Not reached: VescCAN.cpp states and frames covered by other work orders.
 */
#include "doctest.h"
#include "VescFixture.h"

#include <map>

namespace {

// eeprom_addresses.h:206-211
constexpr uint16_t ADR_V1_CANID = 0x3E0;
constexpr uint16_t ADR_V1_DATA = 0x3E1;
constexpr uint16_t ADR_V1_OFFSET = 0x3E2;
constexpr uint16_t ADR_V2_CANID = 0x3E3;
constexpr uint16_t ADR_V2_DATA = 0x3E4;
constexpr uint16_t ADR_V2_OFFSET = 0x3E5;

bool hasWord(uint16_t adr) { return HostPlatform::flash().count(adr) != 0; }

uint16_t word(uint16_t adr) {
	const auto &f = HostPlatform::flash();
	auto it = f.find(adr);
	REQUIRE_MESSAGE(it != f.end(), "flash word 0x" << std::hex << adr << " was not written");
	return it->second;
}

void setWord(uint16_t adr, uint16_t value) { HostPlatform::flash()[adr] = value; }

CommandStatus setCmd(CommandHandler &h, const char *name, int64_t val) { return CommandHelpers::run(h, name, CMDtype::set, val); }
int64_t getCmd(CommandHandler &h, const char *name) { return CommandHelpers::get(h, name); }

constexpr int64_t STATE_UNKNOWN = 0, STATE_COMPATIBLE = 3, STATE_READY = 4, STATE_ERROR = 5; // VescState (VescCAN.h:27-35)

struct EncFixture : VescFixture {
	/** The VESC reports `deg` as its rotor angle; returns the canReceive result (0 = rejected by the filters) */
	uint32_t rotor(double deg) {
		peer0.setRotorDeg(deg);
		return peer0.sendRotorPosition();
	}
};

} // namespace

TEST_SUITE("VESC encoder") {

// ---------------------------------------------------------------- V60

TEST_CASE_FIXTURE(EncFixture, "[V60] a rotor position of 90 degrees reads as 0.25 turns and getPos 16383 (cpr 65535)") {
	auto v = makeVesc1();
	// HD: 90 * 1e5 = 9000000 = 00 89 54 40 -> 9000000 / 1e5 = 90.0 deg -> 90 / 360 = 0.25 turns.
	// getPos = 65535 * 0.25 = 16383.75 -> truncated 16383.
	CHECK(rotor(90.0) != 0);
	CHECK(v->getPos_f() == 0.25f);
	CHECK(v->getCpr() == 0xFFFF);
	CHECK(v->getPos() == 16383);
}

TEST_CASE_FIXTURE(EncFixture, "[V60] a rotor position of 36 degrees reads as 0.1 turns and getPos 6553") {
	auto v = makeVesc1();
	// HD: 36 / 360 = 0.1 turns (float 0.1f); 65535 * 0.1 = 6553.5 -> truncated 6553 (margin 0.5 to the float error).
	CHECK(rotor(36.0) != 0);
	CHECK_NEAR(v->getPos_f(), 0.1, 1e-6);
	CHECK(v->getPos() == 6553);
}

TEST_CASE_FIXTURE(EncFixture, "[V60] a rotor position of 180 degrees from the start is 0.5 turns and counts no turn (|delta| = 180 is not > 180)") {
	auto v = makeVesc1();
	// HD: delta = 180 - 0 = 180, not > 180 (VC:480). 180 / 360 = 0.5; getPos = 65535 * 0.5 = 32767.5 -> 32767.
	CHECK(rotor(180.0) != 0);
	CHECK(v->getPos_f() == 0.5f);
	CHECK(v->getPos() == 32767);
}

TEST_CASE_FIXTURE(EncFixture, "[V60] with the default VESC id 255 a rotor position from any unit id is accepted") {
	auto v = makeVesc1();
	// Filter: id packet 56 << 8, mask 0xFF00 (VC:115-116). Unit 0x22 -> id 0x3822.
	peer0.setFaultSenderId(0x22);
	CHECK(rotor(90.0) != 0);
	CHECK(v->getPos_f() == 0.25f);
}

TEST_CASE_FIXTURE(EncFixture, "[V60] with a configured VESC id only the rotor position of that unit is accepted") {
	// The id comes from flash (0x0140: VESC id 1, OFFB id 0x40), not from the command: after a vesccanid set the old
	// filter stays registered (B-V1, covered by V2), which would accept every unit.
	setWord(ADR_V1_CANID, 0x0140);
	auto v = makeVesc1();
	CHECK(rotor(90.0) != 0); // unit 0x01 = peer0
	CHECK(v->getPos_f() == 0.25f);
	peer0.setFaultSenderId(0x22); // another VESC on the bus
	CHECK(rotor(180.0) == 0);	  // rejected by the filter
	CHECK(v->getPos_f() == 0.25f); // unchanged
}

TEST_CASE_FIXTURE(EncFixture, "[V60] getPos_f in READY sends a POLL_ROTOR_POS request (packet 56, DLC 0) and the answer is the new position" * KNOWN_ISSUE_UB_HIDDEN("B-V3: sendMsg memcpy(dest, nullptr, 0) for the empty POLL_ROTOR_POS request, UBSan nonnull-attribute at VC:497")) {
	auto v = makeVesc1();
	reachReady(*v);
	clearTxFrames();
	peer0.setRotorDeg(45.0); // HD: 45 / 360 = 0.125
	(void)v->getPos_f();	 // sends the request; the position is read before the answer arrives
	const auto frames = txFramesPacket(0xFF, FakeVescPeer::PKT_POLL_ROTOR_POS);
	REQUIRE(frames.size() == 1);
	CHECK(frames[0].extId);
	CHECK(frames[0].dlc == 0);
	process(); // the peer answers
	CHECK(v->getPos_f() == 0.125f);
}

TEST_CASE_FIXTURE(EncFixture, "[V60] getPos_f outside READY sends no request") {
	auto v = makeVesc1();
	(void)v->getPos_f();
	CHECK(txFramesPacket(0xFF, FakeVescPeer::PKT_POLL_ROTOR_POS).empty());
	CHECK(HostTarget::canTxFrames().empty());
}

// ---------------------------------------------------------------- V61

TEST_CASE_FIXTURE(EncFixture, "[V61] characterization: a forward wrap 350 -> 10 degrees counts one turn: 1.0278 turns") {
	auto v = makeVesc1();
	// Steps below 180 degrees each; the first sample is 0 (delta 0) to stay clear of B-V10.
	for (double deg : {0.0, 90.0, 180.0, 270.0, 350.0}) {
		rotor(deg);
	}
	CHECK_NEAR(v->getPos_f(), 350.0 / 360.0, 1e-6);
	rotor(10.0); // HD: delta = 10 - 350 = -340, |delta| > 180, delta < 0 -> mtPos += 1 (VC:483)
	CHECK_NEAR(v->getPos_f(), 10.0 / 360.0 + 1.0, 1e-6); // = 1.027778
}

TEST_CASE_FIXTURE(EncFixture, "[V61] characterization: a backward wrap 10 -> 350 degrees counts one turn back: -0.0278 turns") {
	auto v = makeVesc1();
	rotor(0.0);
	rotor(10.0);
	CHECK_NEAR(v->getPos_f(), 10.0 / 360.0, 1e-6);
	rotor(350.0); // HD: delta = 340 > 180 and positive -> mtPos -= 1 (VC:483): 350/360 - 1 = -0.027778
	CHECK_NEAR(v->getPos_f(), 350.0 / 360.0 - 1.0, 1e-6);
}

TEST_CASE_FIXTURE(EncFixture, "[V61] characterization: a jump of 181 degrees counts a turn, a jump of 180 does not") {
	auto v = makeVesc1();
	rotor(0.0);
	rotor(90.0);
	rotor(270.0); // delta 180: no turn -> 0.75
	CHECK_NEAR(v->getPos_f(), 0.75, 1e-6);
	rotor(89.0); // delta -181 -> mtPos += 1 -> 89/360 + 1 = 1.247222
	CHECK_NEAR(v->getPos_f(), 89.0 / 360.0 + 1.0, 1e-6);
}

TEST_CASE_FIXTURE(EncFixture, "[V61] characterization: two full forward revolutions accumulate 2 turns") {
	auto v = makeVesc1();
	for (int rev = 0; rev < 2; rev++) {
		for (double deg : {0.0, 90.0, 180.0, 270.0}) {
			rotor(deg);
		}
	}
	rotor(0.0);
	// HD: wrap 1: 270 -> 0 at the start of revolution 2 (delta -270, +1), wrap 2: 270 -> 0 here (+1) -> 2.0 turns at 0 degrees
	CHECK_NEAR(v->getPos_f(), 2.0, 1e-6);
	CHECK(v->getPos() == 131070); // 65535 * 2
}

// ---------------------------------------------------------------- V62

TEST_CASE_FIXTURE(EncFixture, "[V62] the first sample after start-up of 270 degrees reads 0.75 turns" * KNOWN_ISSUE("B-V10: prevPos360 starts at 0, so a first sample above 180 degrees counts -1 turn (VC:478-483): -0.25 instead of 0.75")) {
	auto v = makeVesc1();
	rotor(270.0);
	CHECK(v->getPos_f() == 0.75f); // the driver gives 270 / 360 - 1 = -0.25
}

TEST_CASE_FIXTURE(EncFixture, "[V62] the first sample after start-up of 90 degrees reads 0.25 turns (below 180 degrees there is no issue)") {
	auto v = makeVesc1();
	rotor(90.0);
	CHECK(v->getPos_f() == 0.25f);
}

// ---------------------------------------------------------------- V63

TEST_CASE_FIXTURE(VescFixture, "[V63] with useencoder on getEncoderType is absolute and getEncoder is the driver itself") {
	auto v = makeVesc1();
	CHECK(getCmd(*v, "useencoder") == 1); // default (VescCAN.h:124)
	CHECK(v->getEncoderType() == EncoderType::absolute);
	CHECK(v->getEncoder() == static_cast<Encoder *>(v.get()));
	CHECK(v->hasIntegratedEncoder());
}

TEST_CASE_FIXTURE(VescFixture, "[V63] getEncoderType with useencoder off and no external encoder does not crash" * doctest::skip() * doctest::description("KNOWN ISSUE (crash): B-V11: getEncoderType dereferences MotorDriver::getEncoder(), a null pointer without an external encoder (VC:152-157). Skipped because it terminates the test run")) {
	auto v = makeVesc1();
	setCmd(*v, "useencoder", 0);
	(void)v->getEncoderType();
	CHECK(true);
}

TEST_CASE_FIXTURE(VescFixture, "[V63] characterization: with useencoder off and no external encoder getEncoder is not the driver and hasIntegratedEncoder is false") {
	auto v = makeVesc1();
	setCmd(*v, "useencoder", 0);
	CHECK(v->getEncoder() != static_cast<Encoder *>(v.get()));
	CHECK_FALSE(v->hasIntegratedEncoder());
}

// ---------------------------------------------------------------- V64

TEST_CASE_FIXTURE(EncFixture, "[V64] setPos(0) at 0.25 turns zeroes the position, sets offset 2500 and stores 0x09C4 in 0x3E2") {
	auto v = makeVesc1();
	rotor(90.0); // lastPos 0.25
	v->setPos(0);
	// HD: posOffset = 0.25 - 0/65535 = 0.25; getPos_f = 0.25 - 0.25 = 0
	CHECK(v->getPos_f() == 0.0f);
	CHECK(getCmd(*v, "offset") == 2500); // (int32)(0.25 * 10000)
	// HD: 0.25 * 10000 = 2500 = 0x09C4
	CHECK(word(ADR_V1_OFFSET) == 0x09C4);
}

TEST_CASE_FIXTURE(EncFixture, "[V64] setPos(32767) at 0.25 turns sets a negative offset and stores 0xF63D") {
	auto v = makeVesc1();
	rotor(90.0);
	v->setPos(32767);
	// HD: posOffset = 0.25 - 32767/65535 = 0.25 - 0.4999924 = -0.2499924; * 10000 = -2499.92 -> (int16) -2499
	// -2499 + 65536 = 63037 = 0xF63D. getPos_f = 0.25 + 0.2499924 = 0.4999924 = 32767 / 65535.
	CHECK(word(ADR_V1_OFFSET) == 0xF63D);
	CHECK_NEAR(v->getPos_f(), 32767.0 / 65535.0, 1e-6);
}

TEST_CASE_FIXTURE(EncFixture, "[V64] characterization: the offset command sets the offset in turns * 10000, shifts the position and stores 0x09C4") {
	auto v = makeVesc1();
	rotor(90.0);
	CHECK(setCmd(*v, "offset", 2500) == CommandStatus::OK);
	// HD: posOffset = 2500 / 10000 = 0.25 turns -> getPos_f = 0.25 - 0.25 = 0; word 2500 = 0x09C4
	CHECK(v->getPos_f() == 0.0f);
	CHECK(getCmd(*v, "offset") == 2500);
	CHECK(word(ADR_V1_OFFSET) == 0x09C4);
}

TEST_CASE_FIXTURE(EncFixture, "[V64] characterization: a negative offset command stores the two's complement word 0xF63C and moves the position up") {
	auto v = makeVesc1();
	rotor(90.0);
	setCmd(*v, "offset", -2500);
	// HD: posOffset = -0.25 -> getPos_f = 0.25 + 0.25 = 0.5; (int16)-2500 = 65536 - 2500 = 63036 = 0xF63C
	CHECK(v->getPos_f() == 0.5f);
	CHECK(word(ADR_V1_OFFSET) == 0xF63C);
}

TEST_CASE_FIXTURE(EncFixture, "[V64] a stored offset is applied after a restart: word 0x09C4 at 90 degrees reads position 0") {
	setWord(ADR_V1_OFFSET, 0x09C4); // HD: 2500 / 10000 = 0.25
	auto v = makeVesc1();
	rotor(90.0);
	CHECK(getCmd(*v, "offset") == 2500);
	CHECK(v->getPos_f() == 0.0f);
}

// ---------------------------------------------------------------- V70

TEST_CASE_FIXTURE(VescFixture, "[V70] offbcanid answers 64 (0x40) and a set moves the reply filter: replies to 0x40 are rejected, to 0x41 accepted") {
	auto v = makeVesc1();
	CHECK(getCmd(*v, "offbcanid") == 0x40);
	CHECK(setCmd(*v, "offbcanid", 0x41) == CommandStatus::OK);
	CHECK(getCmd(*v, "offbcanid") == 0x41);
	CHECK(getCmd(*v, "vescstate") == STATE_UNKNOWN);
	CHECK(peer0.sendFwVersion(0x40) == 0);
	CHECK(getCmd(*v, "vescstate") == STATE_UNKNOWN);
	CHECK(peer0.sendFwVersion(0x41) != 0);
	CHECK(getCmd(*v, "vescstate") == STATE_COMPATIBLE);
}

TEST_CASE_FIXTURE(VescFixture, "[V70] after offbcanid=0x41 the FW version request carries 0x41 as sender id: [0x41, 0x00, 0x00]") {
	auto v = makeVesc1();
	setCmd(*v, "offbcanid", 0x41);
	runFor(500);
	const auto frames = txFramesPacket(0xFF, FakeVescPeer::PKT_PROCESS_SHORT_BUFFER);
	REQUIRE(frames.size() == 1);
	CHECK(bytes(frames[0]) == std::vector<uint8_t>({0x41, 0x00, 0x00}));
}

TEST_CASE_FIXTURE(VescFixture, "[V70] vesccanid answers 255 by default and a set changes the destination of the requests to 0x0801") {
	auto v = makeVesc1();
	CHECK(getCmd(*v, "vesccanid") == 255);
	CHECK(setCmd(*v, "vesccanid", 0x01) == CommandStatus::OK);
	CHECK(getCmd(*v, "vesccanid") == 1);
	runFor(500);
	// HD: id = VESC id 0x01 | PROCESS_SHORT_BUFFER (8) << 8 = 0x0801
	const auto frames = txFrames(0x0801);
	REQUIRE(frames.size() == 1);
	CHECK(bytes(frames[0]) == std::vector<uint8_t>({0x40, 0x00, 0x00}));
	CHECK(peer0.requestCount(FakeVescPeer::COMM_FW_VERSION) == 1);
}

TEST_CASE_FIXTURE(VescFixture, "[V70] with vesccanid set to the unit id the driver reaches READY against that unit") {
	auto v = makeVesc1();
	setCmd(*v, "vesccanid", 0x01);
	reachReady(*v);
	CHECK(getCmd(*v, "vescstate") == STATE_READY);
}

TEST_CASE_FIXTURE(VescFixture, "[V70] canspd answers a preset of at least 3 after construction") {
	auto v = makeVesc1();
	CHECK(getCmd(*v, "canspd") >= 3); // VC:66-68
}

TEST_CASE_FIXTURE(VescFixture, "[V70] canspd set 5 and 4 are applied") {
	auto v = makeVesc1();
	setCmd(*v, "canspd", 5);
	CHECK(getCmd(*v, "canspd") == 5);
	setCmd(*v, "canspd", 4);
	CHECK(getCmd(*v, "canspd") == 4);
}

TEST_CASE_FIXTURE(VescFixture, "[V70] canspd set below 3 gives the minimum preset 3") {
	auto v = makeVesc1();
	setCmd(*v, "canspd", 1); // VC:288 max(3, val)
	CHECK(getCmd(*v, "canspd") == 3);
}

TEST_CASE_FIXTURE(VescFixture, "[V70] vescstate answers UNKNOWN (0), COMPATIBLE (3), READY (4) along the start-up") {
	auto v = makeVesc1();
	CHECK(getCmd(*v, "vescstate") == STATE_UNKNOWN);
	runFor(500); // FW request, reply 7.01
	CHECK(getCmd(*v, "vescstate") == STATE_COMPATIBLE);
	runFor(500); // values request, fault 0
	CHECK(getCmd(*v, "vescstate") == STATE_READY);
}

TEST_CASE_FIXTURE(VescFixture, "[V70] voltage answers the supply voltage in mV: 24.0 V -> 24000, 12.5 V -> 12500") {
	auto v = makeVesc1();
	reachReady(*v);
	// HD: reply int16 240 / 10 = 24.0 V * 1000 = 24000
	CHECK(getCmd(*v, "voltage") == 24000);
	peer0.setVin(12.5f); // HD: int16 125 / 10 = 12.5 V -> 12500
	runFor(500);
	CHECK(getCmd(*v, "voltage") == 12500);
}

TEST_CASE_FIXTURE(VescFixture, "[V70] errorflags answers the fault code of the VESC and vescstate becomes ERROR (5); fault 0 returns to READY") {
	auto v = makeVesc1();
	reachReady(*v);
	CHECK(getCmd(*v, "errorflags") == 0);
	peer0.setFaultCode(3);
	runFor(500);
	CHECK(getCmd(*v, "errorflags") == 3);
	CHECK(getCmd(*v, "vescstate") == STATE_ERROR);
	peer0.setFaultCode(0);
	runFor(500);
	CHECK(getCmd(*v, "errorflags") == 0);
	CHECK(getCmd(*v, "vescstate") == STATE_READY);
}

TEST_CASE_FIXTURE(EncFixture, "[V70] characterization: pos answers turns * 1e9: 90 degrees -> 250000000") {
	auto v = makeVesc1();
	rotor(90.0); // HD: 0.25 * 1e9 = 250000000 (= 5^9 * 2^7, exact in float)
	CHECK(getCmd(*v, "pos") == 250000000);
}

TEST_CASE_FIXTURE(EncFixture, "[V70] characterization: pos subtracts the offset: offset 2500 at 90 degrees -> 0, offset -2500 -> 500000000") {
	auto v = makeVesc1();
	rotor(90.0);
	setCmd(*v, "offset", 2500);
	CHECK(getCmd(*v, "pos") == 0);
	setCmd(*v, "offset", -2500); // HD: (0.25 + 0.25) * 1e9
	CHECK(getCmd(*v, "pos") == 500000000);
}

TEST_CASE_FIXTURE(VescFixture, "[V70] characterization: torque answers the last torque ratio * 10000 truncated: turn(16384) -> 5000, turn(-32767) -> -10000, turn(0) -> 0") {
	auto v = makeVesc1();
	v->turn(16384); // HD: 16384 / 32767 = 0.5000153 * 10000 = 5000.15 -> 5000
	CHECK(getCmd(*v, "torque") == 5000);
	v->turn(-32767); // HD: -1.0 * 10000
	CHECK(getCmd(*v, "torque") == -10000);
	v->turn(0);
	CHECK(getCmd(*v, "torque") == 0);
}

TEST_CASE_FIXTURE(EncFixture, "[V70] forceposread sends a POLL_ROTOR_POS request (id 0x38FF, DLC 0) and the answered position is applied" * KNOWN_ISSUE_UB_HIDDEN("B-V3: sendMsg memcpy(dest, nullptr, 0) for the empty POLL_ROTOR_POS request, UBSan nonnull-attribute at VC:497")) {
	auto v = makeVesc1();
	peer0.setRotorDeg(45.0); // HD: 0.125 turns
	CHECK(setCmd(*v, "forceposread", 0) == CommandStatus::OK);
	// HD: id = VESC id 255 | 56 << 8 = 0x38FF
	const auto frames = txFrames(0x38FF);
	REQUIRE(frames.size() == 1);
	CHECK(frames[0].extId);
	CHECK(frames[0].dlc == 0);
	process();
	CHECK(v->getPos_f() == 0.125f);
}

TEST_CASE_FIXTURE(VescFixture, "[V70] useencoder answers 1 by default; set 0 and set 1 are applied to getEncoder, hasIntegratedEncoder and the stored word") {
	auto v = makeVesc1();
	CHECK(getCmd(*v, "useencoder") == 1);
	setCmd(*v, "useencoder", 0);
	CHECK(getCmd(*v, "useencoder") == 0);
	CHECK_FALSE(v->hasIntegratedEncoder());
	setCmd(*v, "useencoder", 1);
	CHECK(getCmd(*v, "useencoder") == 1);
	CHECK(v->hasIntegratedEncoder());
	CHECK(v->getEncoder() == static_cast<Encoder *>(v.get()));
}

TEST_CASE_FIXTURE(VescFixture, "[V70] useencoder set to any non-zero value (7) enables the encoder and answers 1") {
	auto v = makeVesc1();
	setCmd(*v, "useencoder", 0);
	setCmd(*v, "useencoder", 7); // VC:318 val != 0
	CHECK(getCmd(*v, "useencoder") == 1);
}

TEST_CASE_FIXTURE(VescFixture, "[V70] characterization: encrate answers 0 before any position sample") {
	auto v = makeVesc1();
	CHECK(getCmd(*v, "encrate") == 0);
}

TEST_CASE_FIXTURE(EncFixture, "[V70] characterization: encrate answers the position samples per second of the last full period: 3 samples in 1.5 s -> 2") {
	auto v = makeVesc1();
	REQUIRE(HAL_GetTick() == 0); // encStartPeriod starts at 0 (VescCAN.h:130)
	rotor(10.0);
	rotor(20.0);
	rotor(30.0);
	// The thread computes the rate when more than 1000 ms passed since encStartPeriod (VC:386-391): bodies at 500 and
	// 1000 ms have period <= 1000, the body at 1500 ms has period 1500 ms.
	// HD: encRate = 3 / (1500 / 1000.0) = 2.0 -> (uint32) 2. The FW/values replies are not position samples.
	runFor(1000);
	CHECK(getCmd(*v, "encrate") == 0);
	runFor(500);
	CHECK(getCmd(*v, "encrate") == 2);
}

TEST_CASE_FIXTURE(VescFixture, "[V70] the commands of VESC_2 act on VESC_2 only (instance 1): offbcanid and useencoder differ from VESC_1") {
	setWord(ADR_V2_CANID, 0x0142); // VESC id 1, OFFB id 0x42
	setWord(ADR_V2_DATA, 0x0000);
	auto v1 = makeVesc1();
	auto v2 = makeVesc2();
	CHECK(getCmd(*v1, "offbcanid") == 0x40);
	CHECK(getCmd(*v2, "offbcanid") == 0x42);
	CHECK(getCmd(*v1, "vesccanid") == 255);
	CHECK(getCmd(*v2, "vesccanid") == 1);
	CHECK(getCmd(*v1, "useencoder") == 1);
	CHECK(getCmd(*v2, "useencoder") == 0);
	CHECK(setCmd(*v2, "offbcanid", 0x43) == CommandStatus::OK);
	CHECK(getCmd(*v1, "offbcanid") == 0x40);
}

// ---------------------------------------------------------------- V71

TEST_CASE_FIXTURE(EncFixture, "[V71] pos reports 2000000000 at 2.0 turns (below the int32 limit 2147483647)") {
	auto v = makeVesc1();
	setCmd(*v, "offset", -20000); // HD: posOffset = -2.0 turns, lastPos 0 -> position 2.0 turns; 2.0 * 1e9 = 2e9 (= 5^9 * 2^10, exact in float)
	CHECK(getCmd(*v, "pos") == 2000000000);
}

TEST_CASE_FIXTURE(EncFixture, "[V71] pos reports 3000000000 at 3.0 turns" * KNOWN_ISSUE_UB("B-V12: the position is cast to int32 (VC:303), 3e9 exceeds 2147483647: float to int32 overflow, the host gives -2147483648")) {
	auto v = makeVesc1();
	setCmd(*v, "offset", -30000); // HD: posOffset = -3.0 turns -> 3.0 * 1e9 = 3e9 (5859375 * 2^9, exact in float)
	CHECK(getCmd(*v, "pos") == 3000000000LL);
}

// ---------------------------------------------------------------- V72

TEST_CASE_FIXTURE(VescFixture, "[V72] characterization: canspd set 260 is narrowed to uint8 4 and applied as preset 4" * doctest::description("NEEDS REVIEW")) {
	auto v = makeVesc1();
	setCmd(*v, "canspd", 5);
	setCmd(*v, "canspd", 260); // HD: (uint8_t)260 = 4; max(3, 4) = 4 (VC:288, cmd.val is int64)
	CHECK(getCmd(*v, "canspd") == 4);
}

TEST_CASE_FIXTURE(VescFixture, "[V72] characterization: canspd set 9 is ignored by the port (presets above 5 are rejected)") {
	auto v = makeVesc1();
	setCmd(*v, "canspd", 4);
	setCmd(*v, "canspd", 9); // max(3, 9) = 9 > 5: CANPort_2B::setSpeedPreset returns (CANPort2B.cpp:137)
	CHECK(getCmd(*v, "canspd") == 4);
}

// ---------------------------------------------------------------- V80

TEST_CASE_FIXTURE(VescFixture, "[V80] saveFlash of VESC_1 with defaults writes 0x3E0 = 0xFF40, 0x3E1 = 0x0008, 0x3E2 = 0x0000") {
	auto v = makeVesc1();
	v->saveFlash();
	// HD: CANID = (VESC id 255 << 8) | OFFB id 0x40 = 0xFF00 | 0x40 = 0xFF40; DATA = useencoder(1) << 3 = 0x0008;
	// OFFSET = 0
	CHECK(word(ADR_V1_CANID) == 0xFF40);
	CHECK(word(ADR_V1_DATA) == 0x0008);
	CHECK(word(ADR_V1_OFFSET) == 0x0000);
	CHECK_FALSE(hasWord(ADR_V2_CANID));
	CHECK_FALSE(hasWord(ADR_V2_DATA));
	CHECK_FALSE(hasWord(ADR_V2_OFFSET));
}

TEST_CASE_FIXTURE(VescFixture, "[V80] saveFlash of VESC_2 with defaults writes 0x3E3 = 0xFF40, 0x3E4 = 0x0008, 0x3E5 = 0x0000 and nothing at VESC_1's addresses") {
	auto v = makeVesc2();
	v->saveFlash();
	CHECK(word(ADR_V2_CANID) == 0xFF40);
	CHECK(word(ADR_V2_DATA) == 0x0008);
	CHECK(word(ADR_V2_OFFSET) == 0x0000);
	CHECK_FALSE(hasWord(ADR_V1_CANID));
	CHECK_FALSE(hasWord(ADR_V1_DATA));
	CHECK_FALSE(hasWord(ADR_V1_OFFSET));
}

TEST_CASE_FIXTURE(VescFixture, "[V80] saveFlash packs the ids and the encoder flag: VESC id 0x12, OFFB id 0x41, useencoder off -> 0x1241 and 0x0000") {
	auto v = makeVesc1();
	setCmd(*v, "offbcanid", 0x41);
	setCmd(*v, "vesccanid", 0x12);
	setCmd(*v, "useencoder", 0);
	v->saveFlash();
	// HD: (0x12 << 8) | 0x41 = 0x1241; useencoder 0 -> DATA 0
	CHECK(word(ADR_V1_CANID) == 0x1241);
	CHECK(word(ADR_V1_DATA) == 0x0000);
}

TEST_CASE_FIXTURE(VescFixture, "[V80] saveFlash stores the offset as int16 fractional turns * 10000: offset command 2500 -> 0x09C4 and -2500 -> 0xF63C in 0x3E2") {
	auto v = makeVesc1();
	setCmd(*v, "offset", 2500);
	v->saveFlash();
	CHECK(word(ADR_V1_OFFSET) == 0x09C4); // HD: 2500 = 0x09C4
	setCmd(*v, "offset", -2500);
	v->saveFlash();
	CHECK(word(ADR_V1_OFFSET) == 0xF63C); // HD: 65536 - 2500 = 63036 = 0xF63C
}

TEST_CASE_FIXTURE(VescFixture, "[V80] restore reads the ids and the encoder flag: 0x3E0 = 0x1241, 0x3E1 = 0x0000 gives OFFB id 0x41, VESC id 0x12, useencoder off") {
	setWord(ADR_V1_CANID, 0x1241); // HD: low byte 0x41 = OFFB id, high byte 0x12 = VESC id (VC:167-168)
	setWord(ADR_V1_DATA, 0x0000);
	auto v = makeVesc1();
	CHECK(getCmd(*v, "offbcanid") == 0x41);
	CHECK(getCmd(*v, "vesccanid") == 0x12);
	CHECK(getCmd(*v, "useencoder") == 0);
}

TEST_CASE_FIXTURE(VescFixture, "[V80] restore of the DATA word uses bit 3 only: 0x0008 -> on, 0x0007 -> off, 0xFFF7 -> off, 0xFFF8 -> on") {
	struct Case {
		uint16_t word;
		int64_t expected;
	};
	for (Case c : {Case{0x0008, 1}, Case{0x0007, 0}, Case{0xFFF7, 0}, Case{0xFFF8, 1}}) {
		INFO("DATA word 0x" << std::hex << c.word);
		HostPlatform::flash().clear();
		setWord(ADR_V1_DATA, c.word);
		auto v = makeVesc1();
		CHECK(getCmd(*v, "useencoder") == c.expected);
	}
}

TEST_CASE_FIXTURE(VescFixture, "[V80] a restored driver sends its requests with the stored ids: VESC id 0x12, OFFB id 0x41 -> id 0x0812, [0x41, 0x00, 0x00]") {
	setWord(ADR_V1_CANID, 0x1241);
	auto v = makeVesc1();
	runFor(500);
	// HD: id = 0x12 | 8 << 8 = 0x0812
	const auto frames = txFrames(0x0812);
	REQUIRE(frames.size() == 1);
	CHECK(bytes(frames[0]) == std::vector<uint8_t>({0x41, 0x00, 0x00}));
}

TEST_CASE_FIXTURE(VescFixture, "[V80] restore of the OFFSET word: 0x09C4 -> offset 2500, 0xF63C -> -2500") {
	setWord(ADR_V1_OFFSET, 0x09C4);
	{
		auto v = makeVesc1();
		CHECK(getCmd(*v, "offset") == 2500); // HD: (int16)0x09C4 = 2500 / 10000 = 0.25 -> * 10000 = 2500
	}
	setWord(ADR_V1_OFFSET, 0xF63C);
	auto v = makeVesc1();
	CHECK(getCmd(*v, "offset") == -2500); // HD: (int16)0xF63C = -2500 -> -0.25
}

TEST_CASE_FIXTURE(VescFixture, "[V80] restore normalises an offset above half a turn into [-0.5, 0.5]: 0x1D4C (7500 = 0.75) -> -0.25 and 0xE2B4 (-7500) -> 0.25") {
	setWord(ADR_V1_OFFSET, 0x1D4C); // HD: 7500 = 0x1D4C; 0.75 > 0.5 -> 0.75 - 1 = -0.25 (VC:183-187)
	{
		auto v = makeVesc1();
		CHECK(getCmd(*v, "offset") == -2500);
	}
	setWord(ADR_V1_OFFSET, 0xE2B4); // HD: 65536 - 7500 = 58036 = 0xE2B4 -> -0.75 -> + 1 = 0.25
	auto v = makeVesc1();
	CHECK(getCmd(*v, "offset") == 2500);
}

TEST_CASE_FIXTURE(VescFixture, "[V80] restore keeps an offset of exactly half a turn: 0x1388 (5000) -> 5000, not wrapped") {
	setWord(ADR_V1_OFFSET, 0x1388); // HD: 5000 = 0x1388; 0.5 is not > 0.5
	auto v = makeVesc1();
	CHECK(getCmd(*v, "offset") == 5000);
}

TEST_CASE_FIXTURE(VescFixture, "[V80] both instances restore from and save to their own addresses") {
	setWord(ADR_V1_CANID, 0x0141); // VESC id 1, OFFB id 0x41
	setWord(ADR_V1_DATA, 0x0008);
	setWord(ADR_V1_OFFSET, 0x09C4); // 0.25
	setWord(ADR_V2_CANID, 0x0242); // VESC id 2, OFFB id 0x42
	setWord(ADR_V2_DATA, 0x0000);
	setWord(ADR_V2_OFFSET, 0xF63C); // -0.25
	auto v1 = makeVesc1();
	auto v2 = makeVesc2();
	CHECK(getCmd(*v1, "offbcanid") == 0x41);
	CHECK(getCmd(*v1, "vesccanid") == 1);
	CHECK(getCmd(*v1, "useencoder") == 1);
	CHECK(getCmd(*v1, "offset") == 2500);
	CHECK(getCmd(*v2, "offbcanid") == 0x42);
	CHECK(getCmd(*v2, "vesccanid") == 2);
	CHECK(getCmd(*v2, "useencoder") == 0);
	CHECK(getCmd(*v2, "offset") == -2500);
	// change VESC_2 only and save both: VESC_1's words stay
	setCmd(*v2, "offbcanid", 0x44); // HD: (2 << 8) | 0x44 = 0x0244
	v1->saveFlash();
	v2->saveFlash();
	CHECK(word(ADR_V1_CANID) == 0x0141);
	CHECK(word(ADR_V1_DATA) == 0x0008);
	CHECK(word(ADR_V1_OFFSET) == 0x09C4);
	CHECK(word(ADR_V2_CANID) == 0x0244);
	CHECK(word(ADR_V2_DATA) == 0x0000);
	CHECK(word(ADR_V2_OFFSET) == 0xF63C);
}

TEST_CASE_FIXTURE(VescFixture, "[V80] a save and a restart round trip keeps ids, encoder flag and offset") {
	{
		auto v = makeVesc1();
		setCmd(*v, "offbcanid", 0x45);
		setCmd(*v, "vesccanid", 0x07);
		setCmd(*v, "useencoder", 0);
		setCmd(*v, "offset", -1234);
		v->saveFlash();
	}
	// HD: CANID (7 << 8) | 0x45 = 0x0745; DATA 0; OFFSET (int16)-1234 = 64302 = 0xFB2E
	CHECK(word(ADR_V1_CANID) == 0x0745);
	CHECK(word(ADR_V1_OFFSET) == 0xFB2E);
	auto v = makeVesc1();
	CHECK(getCmd(*v, "offbcanid") == 0x45);
	CHECK(getCmd(*v, "vesccanid") == 0x07);
	CHECK(getCmd(*v, "useencoder") == 0);
	CHECK(getCmd(*v, "offset") == -1234);
}

// ---------------------------------------------------------------- V81

TEST_CASE_FIXTURE(VescFixture, "[V81] characterization: an offset of 2.75 turns is stored as the fractional -0.25 (0xF63C): the whole turns are lost" * doctest::description("NEEDS REVIEW")) {
	auto v = makeVesc1();
	setCmd(*v, "offset", 27500);
	// HD: posOffset = 2.75 (exact); 2.75 - (int)2.75 = 0.75 > 0.5 -> 0.75 - 1 = -0.25 -> -2500 -> 0xF63C
	CHECK(word(ADR_V1_OFFSET) == 0xF63C); // already written by the offset command (VC:326)
	v->saveFlash();
	CHECK(word(ADR_V1_OFFSET) == 0xF63C);
}

TEST_CASE_FIXTURE(VescFixture, "[V81] characterization: after a restart an offset of 1.5 turns reads as 0.5 turns (multi-turn part lost)" * doctest::description("NEEDS REVIEW")) {
	{
		auto v = makeVesc1();
		setCmd(*v, "offset", 15000);
		// HD: 1.5 - 1 = 0.5, not > 0.5 -> stored 5000 = 0x1388
		CHECK(word(ADR_V1_OFFSET) == 0x1388);
		CHECK(getCmd(*v, "offset") == 15000); // in RAM the full value stays
	}
	auto v = makeVesc1();
	CHECK(getCmd(*v, "offset") == 5000);
}

TEST_CASE_FIXTURE(VescFixture, "[V81] characterization: only the fractional offset is stored: 1.25 turns of offset are stored as 0.25 (0x09C4)" * doctest::description("NEEDS REVIEW")) {
	auto v = makeVesc1();
	setCmd(*v, "offset", 12500); // 1.25 turns
	// HD: 1.25 - 1 = 0.25 -> 2500 = 0x09C4
	CHECK(word(ADR_V1_OFFSET) == 0x09C4);
	v->saveFlash();
	CHECK(word(ADR_V1_OFFSET) == 0x09C4);
}

// ---------------------------------------------------------------- review additions

TEST_CASE_FIXTURE(EncFixture, "[V60] after vesccanid 255 -> 5 the driver ignores a rotor position of unit 1 (its own address check), pos stays unchanged") {
	// Through the command the old broadcast filter bank stays registered (B-V1), so the hardware filter still lets unit
	// 1 through; the driver's own check (VC:614-620) must reject it. Valid after B-V1 is fixed as well (the filter then rejects).
	auto v = makeVesc1();
	rotor(90.0);
	CHECK(getCmd(*v, "pos") == 250000000); // 0.25 turns * 1e9
	setCmd(*v, "vesccanid", 5);
	rotor(180.0); // unit 0x01
	CHECK(getCmd(*v, "pos") == 250000000);
	peer0.setFaultSenderId(5);
	CHECK(rotor(180.0) != 0); // unit 5 is accepted: 0.5 turns
	CHECK(getCmd(*v, "pos") == 500000000);
}

TEST_CASE_FIXTURE(VescFixture, "[V80] an OFFB id of 0x81 (above 0x7F) restored from flash 0xFF81 accepts replies to 0x81 and rejects replies to 0x40") {
	setWord(ADR_V1_CANID, 0xFF81); // HD: VESC id 0xFF << 8 | OFFB id 0x81
	auto v = makeVesc1();
	CHECK(getCmd(*v, "offbcanid") == 0x81);
	CHECK(peer0.sendFwVersion(0x40) == 0);
	CHECK(peer0.sendFwVersion(0x81) != 0);
	CHECK(getCmd(*v, "vescstate") == STATE_COMPATIBLE);
}

TEST_CASE_FIXTURE(VescFixture, "[V80] a VESC id of 0xC0 restored from flash word 0xC040 is read back and filters rotor positions: unit 0xC0 accepted, unit 1 rejected") {
	setWord(ADR_V1_CANID, 0xC040); // HD: 0xC0 << 8 | 0x40 = 0xC000 + 0x40
	auto v = makeVesc1();
	CHECK(getCmd(*v, "vesccanid") == 0xC0);
	CHECK(getCmd(*v, "offbcanid") == 0x40);
	peer0.setRotorDeg(90.0);
	CHECK(peer0.sendRotorPosition() == 0); // unit 1
	peer0.setFaultSenderId(0xC0);
	CHECK(peer0.sendRotorPosition() != 0);
	CHECK(v->getPos_f() == 0.25f);
}

TEST_CASE_FIXTURE(EncFixture, "[V70] characterization: encrate restarts its counter every period: 3 samples in the first period give 2, 6 in the second give 4") {
	auto v = makeVesc1();
	REQUIRE(HAL_GetTick() == 0);
	for (double d : {10.0, 20.0, 30.0}) rotor(d);
	runFor(1500); // period 1500 ms: 3 / 1.5 = 2
	CHECK(getCmd(*v, "encrate") == 2);
	for (double d : {40.0, 50.0, 60.0, 70.0, 80.0, 90.0}) rotor(d);
	runFor(1500); // bodies at 2000 and 2500 have period <= 1000; at 3000 the period is 1500: 6 / 1.5 = 4 (not 9 / 3)
	CHECK(getCmd(*v, "encrate") == 4);
}

TEST_CASE_FIXTURE(EncFixture, "[V60] getPos_f in COMPATIBLE sends no POLL_ROTOR_POS request") {
	auto v = makeVesc1();
	runFor(500);
	REQUIRE(getCmd(*v, "vescstate") == STATE_COMPATIBLE);
	clearTxFrames();
	(void)v->getPos_f();
	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(EncFixture, "[V60] getPos_f in ERROR sends no POLL_ROTOR_POS request") {
	auto v = makeVesc1();
	reachReady(*v);
	peer0.setFaultCode(3);
	runFor(500);
	REQUIRE(getCmd(*v, "vescstate") == STATE_ERROR);
	clearTxFrames();
	(void)v->getPos_f();
	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(VescFixture, "[V80] restore of a whole-turn offset word gives 0: 0x2710 (+10000) and 0xD8F0 (-10000)") {
	setWord(ADR_V1_OFFSET, 0x2710); // HD: 10000 = 0x2710 -> 1.0 turn -> 1.0 - (int)1.0 = 0
	{
		auto v = makeVesc1();
		CHECK(getCmd(*v, "offset") == 0);
	}
	setWord(ADR_V1_OFFSET, 0xD8F0); // HD: 65536 - 10000 = 55536 = 0xD8F0 -> -1.0 -> 0
	auto v = makeVesc1();
	CHECK(getCmd(*v, "offset") == 0);
}

TEST_CASE_FIXTURE(VescFixture, "[V80] restore of an offset word beyond one turn keeps the fraction: 0x30D4 (1.25) -> 2500, 0xCF2C (-1.25) -> -2500, 0x445C (1.75) -> -2500") {
	struct Case { uint16_t w; int64_t exp; };
	// HD: 12500 = 0x30D4 -> 1.25 -> 0.25 -> 2500; 65536-12500 = 53036 = 0xCF2C -> -1.25 -> -0.25;
	// 17500 = 0x445C -> 1.75 -> 0.75 > 0.5 -> -0.25 -> -2500
	for (Case c : {Case{0x30D4, 2500}, Case{0xCF2C, -2500}, Case{0x445C, -2500}}) {
		INFO("word 0x" << std::hex << c.w);
		setWord(ADR_V1_OFFSET, c.w);
		auto v = makeVesc1();
		CHECK(getCmd(*v, "offset") == c.exp);
	}
}

} // TEST_SUITE
