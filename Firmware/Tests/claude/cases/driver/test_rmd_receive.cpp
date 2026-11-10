/*
 * MyActuator RMD CAN driver (RmdMotorCAN), work order RMR: receive path, position and status
 * (requirements R5, R6 and R7 of Tests/claude/charters/driver/RMD_REQUIREMENTS.md).
 *
 * Oracles: SP = MyActuator "CAN Bus Motor Motion Protocol V4.3" (section numbers in the comments: 2.12 multi turn angle
 * 0x92, 2.13 single turn 0x94, 2.14 status 1 0x9A and its error table, 2.19 torque reply 0xA1, 2.33 model 0xB5);
 * HD = a hand derivation in the comment next to the assertion; FW = the firmware's own behaviour (characterization:).
 *
 * How it works: the peer (FakeRmdPeer) builds the reply of a command from its configured state (setTemperature, setIq,
 * setSpeed, setShaftAngle, setVoltage, setErrors, setModel, setMultiTurn, setSingleTurn). peer.sendReply(cmd) puts it on
 * the bus at once. The fake HAL delivers a received frame synchronously (canReceive -> RX interrupt -> canRxPendCallback),
 * so a test sends a reply and reads the result through the public interface (commands temp / curr / vbus / errors /
 * model / requestpos, getPos, setPos, motorReady, turn, startMotor, the frames of the driver). Most tests never advance
 * the driver thread (it starts 100 ms after construction); the few that need a thread cycle use runFor().
 * The fixture of this file starts at 10 s of simulated time so that the driver's counters that start at 0
 * (lastAngleUpdate, lastTorqueStatusUpdate_us) are well in the past.
 *
 * Reply payloads are also injected with peer.inject(...) where the peer is silent or the id is not the peer's own.
 *
 * Host versus target: lastPos is a float of degrees; getPos() is (int32_t)(lastPos * 100) - posOffset in single
 * precision (the host and the Cortex-M4F FPU give the same IEEE single results). The receive path multiplies with the
 * double 0.01 and stores to float (software double on the target, same values). Values above 2^24 counts lose counts
 * (R6.7). The ErrorStatus bit field is a GCC little endian bit field in a union (the same on host and target).
 *
 * SP bit 0x4000 (encoder data error) has no ErrorStatus flag: it falls into res2 (bits 10-15); the firmware only uses asInt.
 * Speed bytes of 0xA1: observable only through the request-mode interpolation (see the R6.5 / R5.2 cases with 1 us steps).
 *
 * Characterizations of this file (NEEDS REVIEW): the 0xA1 angle average in the request mode (R5.2), the 10 ms position
 * refresh constant (R6.3), the 0x94 formula (R6.6), "the driver never requests 0x94" (R6.6), float resolution above 2^24
 * counts (R6.7), the passive mode never requests 0x9A and never sees motor errors (R7.4).
 * KNOWN_ISSUEs: B-RMD-7 (R5.4 temperature is unsigned), B-RMD-4 (R5.7 bit field positions from overpower on),
 * B-RMD-6 (R6.5 interpolation dimension). Findings beyond the charter: see the comments marked "Lead decision".
 */
#include "doctest.h"
#include "RmdFixture.h"
#include "TestHelpers.h"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

using CommandHelpers::get;
using CommandHelpers::run;

constexpr uint32_t ID1 = 0x141; // driver frames for node 1: 0x140 + 1 (SP 1.1.2)

struct RmdRxFixture : RmdFixture {
	RmdRxFixture() {
		HostPlatform::advanceMs(10000);
		peer1.reset();
		peer2.reset();
	}

	/** Injects the reply of `cmd` of the peer's current state with arbitration id `id` (works for a silent peer too). @return canReceive result (0 = rejected by the filters) */
	static uint32_t rx(FakeRmdPeer &p, uint32_t id, uint8_t cmd) {
		const auto a = p.replyPayload(cmd);
		return p.inject(id, std::vector<uint8_t>(a.begin(), a.end()));
	}

	/** Injects a raw reply of node 1 with the given bytes (id 0x241) */
	static uint32_t rxRaw(FakeRmdPeer &p, const std::vector<uint8_t> &bytes) { return p.inject(0x241, bytes); }

	void setRequestMode(RmdMotorCAN &d, int v) { REQUIRE(run(d, "requestpos", CMDtype::set, v) == CommandStatus::OK); }

	static bool is(const HostTarget::CanTxFrame &f, std::initializer_list<uint8_t> bytes) {
		size_t i = 0;
		for (uint8_t b : bytes) {
			if (f.data[i++] != b) {
				return false;
			}
		}
		return i == 8;
	}

};

} // namespace

TEST_SUITE("RMD receive") {

// =============================================================================================================
// R5.1 foreign frames
// =============================================================================================================

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.1 rx] a 0xA1 and a 0x9A reply of node 2 (id 0x242) change none of the values of the node 1 driver") {
	auto rmd = makeRmd1();
	peer2.setTemperature(33);
	peer2.setIq(77);
	peer2.setVoltage(500);
	peer2.setErrors(0x0010);
	CHECK(rx(peer2, 0x242, 0xA1) == 0); // the filter 0x241 / 0x3FF rejects 0x242 (SP 1.1.2: reply id 0x240 + ID)
	CHECK(rx(peer2, 0x242, 0x9A) == 0);
	CHECK(get(*rmd, "temp") == 0);
	CHECK(get(*rmd, "curr") == 0);
	CHECK(get(*rmd, "vbus") == 0);
	CHECK(get(*rmd, "errors") == 0);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.1 rx] a 0x92 reply of node 2 does not move the position of the node 1 driver") {
	auto rmd = makeRmd1();
	peer2.setMultiTurn(36000);
	CHECK(rx(peer2, 0x242, 0x92) == 0);
	CHECK(rmd->getPos() == 0);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.1 rx] a 0xB5 reply of node 2 does not set the model name of the node 1 driver") {
	auto rmd = makeRmd1();
	CHECK(rx(peer2, 0x242, 0xB5) == 0);
	std::vector<CommandReply> replies;
	REQUIRE(run(*rmd, "model", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply.empty());
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.1 rx] a reply of node 1 (id 0x241) does not change the node 2 driver") {
	auto rmd = makeRmd2();
	peer1.setTemperature(44);
	peer1.setVoltage(499);
	CHECK(peer1.sendReply(0xA1) == 0); // the filter of the node 2 driver is 0x242
	CHECK(peer1.sendReply(0x9A) == 0);
	CHECK(get(*rmd, "temp") == 0);
	CHECK(get(*rmd, "vbus") == 0);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.1 rx] a reply that carries the id of another node (wrong node id fault of the peer) is not used") {
	auto rmd = makeRmd1();
	peer1.setTemperature(44);
	peer1.setFaultNode(3); // the reply goes out with id 0x243
	peer1.sendReply(0xA1);
	CHECK(get(*rmd, "temp") == 0);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.1 rx] a frame whose id differs from 0x241 only in bit 10 is ignored by the driver (the filter mask 0x3FF does not look at bit 10)") {
	// HD: id 0x641 = 0x241 | 0x400. The mask 0x3FF compares bits 9..0 only, so the filter lets the frame through and the
	// driver's own check "id != 0x240 + nodeId" is what rejects it.
	auto rmd = makeRmd1();
	peer1.setTemperature(44);
	const uint32_t accepted = rx(peer1, 0x641, 0xA1);
	REQUIRE(accepted != 0); // otherwise this test does not reach the driver's own id check (fake filter semantics)
	CHECK(get(*rmd, "temp") == 0);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.1 rx] frames of another node during a whole cycle do not make the motor available") {
	auto rmd = makeRmd1();
	peer1.setSilent(true);
	runFor(500); // inside the first cycle: it started at 100 ms and ends at 100 + 20 + 150 + 1000 = 1270 ms
	peer2.setTemperature(30);
	rx(peer2, 0x242, 0xA1);
	rx(peer2, 0x242, 0x9A);
	runFor(772); // 1272 ms: the end of the first cycle is passed
	CHECK_FALSE(rmd->motorReady());
}

// =============================================================================================================
// R5.2 0xA1 reply
// =============================================================================================================

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.2 rx] the temperature of a 0xA1 reply is returned by the temp command") {
	auto rmd = makeRmd1();
	peer1.setTemperature(50); // SP 2.19 example 0x32 = 50 degC
	peer1.sendReply(0xA1);
	CHECK(get(*rmd, "temp") == 50);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.2 rx] a positive torque current of a 0xA1 reply is returned by the curr command in 0.01 A") {
	auto rmd = makeRmd1();
	peer1.setIq(100); // SP 2.19 example 0x0064 = 100 (1 A)
	peer1.sendReply(0xA1);
	CHECK(get(*rmd, "curr") == 100);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.2 rx] a negative torque current of a 0xA1 reply is returned negative") {
	auto rmd = makeRmd1();
	peer1.setIq(-100); // SP 2.19 example 0xFF9C = -100
	peer1.sendReply(0xA1);
	CHECK(get(*rmd, "curr") == -100);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.2 rx] the most negative torque current -32768 is returned without wrap around") {
	auto rmd = makeRmd1();
	peer1.setIq(-32768); // bytes 0x00 0x80
	peer1.sendReply(0xA1);
	CHECK(get(*rmd, "curr") == -32768);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.2 rx] characterization: in the request mode a 0xA1 reply with angle 90 degrees and speed 0 moves the position to the average 45 degrees") {
	// FW: lastPos = (lastPos + (angle + interpolation)) / 2 with lastPos 0, angle 90, interpolation 0 (speed 0): 45.0 degrees,
	// 4500 counts. The averaging is the firmware's filter (charter R6.5), the angle field is the output shaft angle in degrees (SP 2.19).
	auto rmd = makeRmd1();
	setRequestMode(*rmd, 1);
	peer1.setShaftAngle(90);
	peer1.setSpeed(0);
	peer1.sendReply(0xA1);
	CHECK(rmd->getPos() == 4500);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.2 rx] characterization: in the request mode a 0xA1 reply with angle -90 degrees moves the position to -45 degrees") {
	auto rmd = makeRmd1();
	setRequestMode(*rmd, 1);
	peer1.setShaftAngle(-90); // int16 little endian 0xFFA6 = -90
	peer1.sendReply(0xA1);
	CHECK(rmd->getPos() == -4500);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.2 rx] a 0xA1 reply does not change the voltage, the error flags or the model") {
	auto rmd = makeRmd1();
	peer1.setVoltage(500);
	peer1.setErrors(0x0004);
	peer1.sendReply(0xA1);
	CHECK(get(*rmd, "vbus") == 0);
	CHECK(get(*rmd, "errors") == 0);
}

// =============================================================================================================
// R5.3 0x9A reply
// =============================================================================================================

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.3 rx] the motor temperature of a 0x9A reply is returned by the temp command") {
	auto rmd = makeRmd1();
	peer1.setTemperature(40); // SP 2.14: DATA[1] motor temperature, 1 degC / LSB
	peer1.sendReply(0x9A);
	CHECK(get(*rmd, "temp") == 40);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.3 rx] the voltage 0x01E5 of a 0x9A reply is returned as 485 (48.5 V, 0.1 V per LSB)") {
	auto rmd = makeRmd1();
	peer1.setVoltage(0x01E5); // bytes 0xE5 0x01, 0x01E5 = 256 + 229 = 485
	peer1.sendReply(0x9A);
	CHECK(get(*rmd, "vbus") == 485);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.3 rx] the error flags 0x0004 (low voltage) of a 0x9A reply are returned by the errors command") {
	auto rmd = makeRmd1();
	peer1.setErrors(0x0004);
	peer1.sendReply(0x9A);
	CHECK(get(*rmd, "errors") == 4);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.3 rx] the error flags use both bytes: 0x4010 (encoder data error and over current) is returned complete") {
	// SP 2.14: DATA[6] low byte, DATA[7] high byte: 0x10 | 0x40 << 8 = 0x4010
	auto rmd = makeRmd1();
	peer1.setErrors(0x4010);
	peer1.sendReply(0x9A);
	CHECK(get(*rmd, "errors") == 0x4010);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.3 rx] a voltage above 255 keeps its high byte (0x1234 = 4660)") {
	auto rmd = makeRmd1();
	peer1.setVoltage(0x1234); // bytes 0x34 0x12
	peer1.sendReply(0x9A);
	CHECK(get(*rmd, "vbus") == 4660);
}

// =============================================================================================================
// R5.4 signed temperature
// =============================================================================================================

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.4 rx] a temperature of -10 degC (0xF6) is read as -10" * KNOWN_ISSUE("B-RMD-7: curTemp is uint8_t (RmdMotorCAN.h:107), the SP says int8: 0xF6 reads as 246")) {
	auto rmd = makeRmd1();
	peer1.setTemperature(-10); // SP 2.14 / 2.19: signed byte, 0xF6
	peer1.sendReply(0xA1);
	CHECK(get(*rmd, "temp") == -10); // measured: 246
}

// =============================================================================================================
// R5.5 model
// =============================================================================================================

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.5 rx] a 7 character model name is returned complete") {
	auto rmd = makeRmd1();
	peer1.setModel("ABCDEFG");
	peer1.sendReply(0xB5);
	std::vector<CommandReply> replies;
	REQUIRE(run(*rmd, "model", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "ABCDEFG");
	CHECK(replies[0].reply.size() == 7); // the 8th byte of the buffer stays 0: the string is terminated
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.5 rx] a shorter model name zero padded by the motor is returned without the padding") {
	auto rmd = makeRmd1();
	peer1.setModel("X8");
	peer1.sendReply(0xB5);
	std::vector<CommandReply> replies;
	REQUIRE(run(*rmd, "model", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "X8");
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.5 rx] a second 0xB5 reply with a shorter name replaces the first one") {
	auto rmd = makeRmd1();
	peer1.setModel("ABCDEFG");
	peer1.sendReply(0xB5);
	peer1.setModel("AB"); // data[1..7] = 'A' 'B' 0 0 0 0 0
	peer1.sendReply(0xB5);
	std::vector<CommandReply> replies;
	REQUIRE(run(*rmd, "model", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "AB");
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.5 rx] a 0xB5 reply changes nothing but the model") {
	auto rmd = makeRmd1();
	peer1.setTemperature(50);
	peer1.setVoltage(500);
	peer1.sendReply(0xB5);
	CHECK(get(*rmd, "temp") == 0);
	CHECK(get(*rmd, "vbus") == 0);
	CHECK(get(*rmd, "errors") == 0);
	CHECK(rmd->getPos() == 0);
	std::vector<CommandReply> replies;
	REQUIRE(run(*rmd, "model", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "X8S2V10"); // the only thing that changed: the peer's default model
}

// =============================================================================================================
// R5.6 available, unknown command bytes
// =============================================================================================================

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.6 rx] a reply with the unknown command byte 0x60 changes no value") {
	auto rmd = makeRmd1();
	peer1.setTemperature(50);
	peer1.setIq(100);
	peer1.setVoltage(500);
	peer1.setErrors(0x0004);
	peer1.setMultiTurn(36000);
	peer1.setFaultCmdByte(0x60); // SP 2.34 lists 0x60 as a possible cyclic reply; the firmware has no handler (commented out)
	peer1.sendReply(0xA1);
	peer1.sendReply(0x9A);
	peer1.sendReply(0x92);
	CHECK(get(*rmd, "temp") == 0);
	CHECK(get(*rmd, "curr") == 0);
	CHECK(get(*rmd, "vbus") == 0);
	CHECK(get(*rmd, "errors") == 0);
	CHECK(rmd->getPos() == 0);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.6 rx] a reply with the unknown command byte 0x9C changes no value") {
	auto rmd = makeRmd1();
	peer1.setTemperature(50);
	peer1.setErrors(0x0004);
	peer1.setFaultCmdByte(0x9C);
	peer1.sendReply(0xA1);
	peer1.sendReply(0x9A);
	CHECK(get(*rmd, "temp") == 0);
	CHECK(get(*rmd, "errors") == 0);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.6 rx] a reply with the unknown command byte 0x9C received during a cycle makes the motor available at the end of the cycle") {
	auto rmd = makeRmd1();
	peer1.setSilent(true);
	runFor(500); // first cycle: 100 ms .. 1270 ms
	REQUIRE(rxRaw(peer1, {0x9C, 0, 0, 0, 0, 0, 0, 0}) != 0);
	runFor(769); // 1269 ms: the cycle has not ended
	CHECK_FALSE(rmd->motorReady());
	runFor(2); // 1271 ms: available = nextAvailable
	CHECK(rmd->motorReady());
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.6 rx] a 0xA1 reply received during a cycle makes the motor available at the end of the cycle") {
	auto rmd = makeRmd1();
	peer1.setSilent(true);
	runFor(500);
	rx(peer1, 0x241, 0xA1);
	runFor(769);
	CHECK_FALSE(rmd->motorReady());
	runFor(2);
	CHECK(rmd->motorReady());
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.6 rx] a reply received before the cycle start does not make the motor available (nextAvailable is cleared at the start of the cycle)") {
	auto rmd = makeRmd1();
	peer1.setSilent(true);
	rx(peer1, 0x241, 0xA1); // t = 0, the first cycle starts at 100 ms and clears nextAvailable
	runFor(1271);
	CHECK_FALSE(rmd->motorReady());
}

// =============================================================================================================
// R5.7 ErrorStatus bit field (public union of the driver)
// =============================================================================================================

TEST_CASE("[R5.7 rx] the ErrorStatus flags stall, undervoltage, overvoltage and overcurrent are the SP bits 0x0002, 0x0004, 0x0008, 0x0010") {
	RmdMotorCAN::ErrorStatus e;
	e.asInt = 0x0002;
	CHECK(bool(e.flags.stall));
	e.asInt = 0x0004;
	CHECK(bool(e.flags.undervoltage));
	e.asInt = 0x0008;
	CHECK(bool(e.flags.overvoltage));
	e.asInt = 0x0010;
	CHECK(bool(e.flags.overcurrent));
}

TEST_CASE("[R5.7 rx] the first four ErrorStatus flags are not set by the other three SP bits of the group") {
	RmdMotorCAN::ErrorStatus e;
	e.asInt = 0x0004 | 0x0008 | 0x0010;
	CHECK_FALSE(bool(e.flags.stall));
	e.asInt = 0x0002 | 0x0008 | 0x0010;
	CHECK_FALSE(bool(e.flags.undervoltage));
	e.asInt = 0x0002 | 0x0004 | 0x0010;
	CHECK_FALSE(bool(e.flags.overvoltage));
	e.asInt = 0x0002 | 0x0004 | 0x0008;
	CHECK_FALSE(bool(e.flags.overcurrent));
}

TEST_CASE("[R5.7 rx] the ErrorStatus flag overpower is the SP bit 0x0040 (power overrun)" * KNOWN_ISSUE("B-RMD-4: overpower is bit 5 (0x0020) in RmdMotorCAN.h:76, SP 2.14 has 0x0040 (bit 6)")) {
	RmdMotorCAN::ErrorStatus e;
	e.asInt = 0x0040;
	CHECK(bool(e.flags.overpower)); // measured: false, the flag is set by 0x0020
}

TEST_CASE("[R5.7 rx] the ErrorStatus flag calibrationWriteErr is the SP bit 0x0080" * KNOWN_ISSUE("B-RMD-4: calibrationWriteErr is bit 6 (0x0040), SP 2.14 has 0x0080 (bit 7)")) {
	RmdMotorCAN::ErrorStatus e;
	e.asInt = 0x0080;
	CHECK(bool(e.flags.calibrationWriteErr));
}

TEST_CASE("[R5.7 rx] the ErrorStatus flag overspeed is the SP bit 0x0100" * KNOWN_ISSUE("B-RMD-4: overspeed is bit 7 (0x0080), SP 2.14 has 0x0100 (bit 8)")) {
	RmdMotorCAN::ErrorStatus e;
	e.asInt = 0x0100;
	CHECK(bool(e.flags.overspeed));
}

TEST_CASE("[R5.7 rx] the ErrorStatus flag overtemp is the SP bit 0x0800 (component overtemperature)" * KNOWN_ISSUE("B-RMD-4: overtemp is bit 8 (0x0100), SP 2.14 has 0x0800 (bit 11)")) {
	RmdMotorCAN::ErrorStatus e;
	e.asInt = 0x0800;
	CHECK(bool(e.flags.overtemp));
}

TEST_CASE("[R5.7 rx] the ErrorStatus flag overtemp is the SP bit 0x1000 (motor over temperature)" * KNOWN_ISSUE("B-RMD-4: overtemp is bit 8 (0x0100), SP 2.14 has 0x1000 (bit 12)")) {
	// Lead decision needed: the struct has one overtemp flag for the two SP bits 0x0800 and 0x1000; this test and the one
	// above assume that the single flag should follow both.
	RmdMotorCAN::ErrorStatus e;
	e.asInt = 0x1000;
	CHECK(bool(e.flags.overtemp));
}

TEST_CASE("[R5.7 rx] the ErrorStatus flag calibrationErr is the SP bit 0x2000 (encoder calibration error)" * KNOWN_ISSUE("B-RMD-4: calibrationErr is bit 9 (0x0200), SP 2.14 has 0x2000 (bit 13)")) {
	RmdMotorCAN::ErrorStatus e;
	e.asInt = 0x2000;
	CHECK(bool(e.flags.calibrationErr));
}

// =============================================================================================================
// R6.1 multi turn angle
// =============================================================================================================

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.1 rx] a 0x92 reply of 36000 (0x00008CA0, 360 degrees) gives getPos 36000") {
	// SP 2.12 example: data[4..7] = 0xA0 0x8C 0x00 0x00 = 36000, 0.01 degree / LSB
	auto rmd = makeRmd1();
	peer1.setMultiTurn(36000);
	peer1.sendReply(0x92);
	CHECK(rmd->getPos() == 36000); // HD: 36000 * 0.01 = 360.0 degrees, * 100 = 36000 (exact in float)
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.1 rx] a negative 0x92 reply of -1 gives getPos -1") {
	// HD: 0xFFFFFFFF as int32 = -1; -1 * 0.01 = -0.01 degree; float(-0.01) * 100 rounds to exactly -1.0f in single precision -> -1 (checked exactly)
	auto rmd = makeRmd1();
	peer1.setMultiTurn(-1);
	peer1.sendReply(0x92);
	CHECK(rmd->getPos() == -1);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.1 rx] a negative 0x92 reply of -36000 (minus one turn) gives getPos -36000") {
	auto rmd = makeRmd1();
	peer1.setMultiTurn(-36000);
	peer1.sendReply(0x92);
	CHECK(rmd->getPos() == -36000);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.1 rx] a 0x92 reply of 0 gives getPos 0") {
	auto rmd = makeRmd1();
	peer1.setMultiTurn(36000);
	peer1.sendReply(0x92);
	peer1.setMultiTurn(0);
	peer1.sendReply(0x92);
	CHECK(rmd->getPos() == 0);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.1 rx] a 0x92 reply of 1 gives getPos 1") {
	auto rmd = makeRmd1();
	peer1.setMultiTurn(1);
	peer1.sendReply(0x92);
	CHECK(rmd->getPos() == 1);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.1 rx] a multi turn 0x92 reply of 1 234 567 (12345.67 degrees) gives getPos 1234567") {
	// HD (single precision): float(1234567 * 0.01) = 12345.669921875, * 100 = 1234566.9921875 -> rounded to float 1234567.0 (spacing 0.125)
	auto rmd = makeRmd1();
	peer1.setMultiTurn(1234567);
	peer1.sendReply(0x92);
	CHECK(rmd->getPos() == 1234567);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.1 rx] the four bytes of the 0x92 angle are all used (0x01020304 = 16909060 counts)") {
	// HD: 0x01020304 = 16909060 -> 169090.60 degrees; float spacing at 2^17..2^18 is 0.0156 degree = 1.6 counts; * 100 is
	// rounded in a float with spacing 2 at 2^24: 16909060 is even, and the run shows it exact
	auto rmd = makeRmd1();
	peer1.setMultiTurn(0x01020304);
	peer1.sendReply(0x92);
	CHECK(rmd->getPos() == 16909060);
}

// =============================================================================================================
// R6.2 setPos
// =============================================================================================================

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.2 rx] setPos(1000) after a report of 36000 makes getPos 1000") {
	// HD: posOffset = 36000 - 1000 = 35000, getPos = 36000 - 35000 = 1000
	auto rmd = makeRmd1();
	peer1.setMultiTurn(36000);
	peer1.sendReply(0x92);
	rmd->setPos(1000);
	CHECK(rmd->getPos() == 1000);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.2 rx] a 0x92 reply after setPos is relative to the offset") {
	// HD: offset 35000 (see above); a report of 37000 gives 37000 - 35000 = 2000
	auto rmd = makeRmd1();
	peer1.setMultiTurn(36000);
	peer1.sendReply(0x92);
	rmd->setPos(1000);
	peer1.setMultiTurn(37000);
	peer1.sendReply(0x92);
	CHECK(rmd->getPos() == 2000);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.2 rx] setPos(500) before any report gives getPos 500 and the first report is added to it") {
	// HD: lastPos 0 -> posOffset = 0 - 500 = -500, getPos = 0 + 500. A report of 36000: 36000 + 500 = 36500
	auto rmd = makeRmd1();
	rmd->setPos(500);
	CHECK(rmd->getPos() == 500);
	peer1.setMultiTurn(36000);
	peer1.sendReply(0x92);
	CHECK(rmd->getPos() == 36500);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.2 rx] setPos(0) after a negative report makes getPos 0") {
	auto rmd = makeRmd1();
	peer1.setMultiTurn(-5000);
	peer1.sendReply(0x92);
	rmd->setPos(0);
	CHECK(rmd->getPos() == 0);
}

// =============================================================================================================
// R6.3 position refresh in the request mode
// =============================================================================================================

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.3 rx] characterization: in the request mode getPos with a stale position sends one 0x92 request to id 0x141") {
	auto rmd = makeRmd1();
	setRequestMode(*rmd, 1);
	clearTxFrames();
	rmd->getPos(); // lastAngleUpdate 0, tick 10000: older than 10 ms
	const auto f = txFramesCmd(ID1, 0x92);
	REQUIRE(f.size() == 1);
	CHECK(is(f[0], {0x92, 0, 0, 0, 0, 0, 0, 0})); // SP 2.12 request frame
	CHECK(f[0].dlc == 8);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.3 rx] characterization: in the request mode getPos does not send a 0x92 request 10 ms after the last 0x92 reply") {
	auto rmd = makeRmd1();
	setRequestMode(*rmd, 1);
	peer1.setMultiTurn(100);
	peer1.sendReply(0x92); // lastAngleUpdate = tick
	clearTxFrames();
	HostPlatform::advanceMs(10); // tick - last = 10, the condition is "> 10"
	rmd->getPos();
	CHECK(txFramesCmd(ID1, 0x92).empty());
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.3 rx] characterization: in the request mode getPos sends a 0x92 request 11 ms after the last 0x92 reply") {
	auto rmd = makeRmd1();
	setRequestMode(*rmd, 1);
	peer1.setMultiTurn(100);
	peer1.sendReply(0x92);
	clearTxFrames();
	HostPlatform::advanceMs(11);
	rmd->getPos();
	CHECK(txFramesCmd(ID1, 0x92).size() == 1);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.3 rx] characterization: a 0xA1 reply does not refresh the position age: the 0x92 request follows 11 ms after the last 0x92 reply anyway") {
	auto rmd = makeRmd1();
	setRequestMode(*rmd, 1);
	peer1.sendReply(0x92);
	HostPlatform::advanceMs(6);
	peer1.sendReply(0xA1);
	HostPlatform::advanceMs(5); // 11 ms after the 0x92 reply, 5 ms after the 0xA1 reply
	clearTxFrames();
	rmd->getPos();
	CHECK(txFramesCmd(ID1, 0x92).size() == 1);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.3 rx] in the passive mode getPos never sends a request even when the position is old") {
	auto rmd = makeRmd1();
	clearTxFrames();
	HostPlatform::advanceMs(5000);
	rmd->getPos();
	rmd->getPos();
	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.3 rx] in the request mode the peer answers the getPos request and the next getPos returns the reported angle") {
	auto rmd = makeRmd1();
	setRequestMode(*rmd, 1);
	peer1.setMultiTurn(12345);
	rmd->getPos(); // sends 0x92
	process();	   // the peer answers within the same call
	CHECK(rmd->getPos() == 12345);
}

// =============================================================================================================
// R6.4 0xA1 in the passive mode
// =============================================================================================================

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.4 rx] a 0xA1 reply in the passive mode leaves the position unchanged") {
	auto rmd = makeRmd1();
	peer1.setMultiTurn(1000);
	peer1.sendReply(0x92);
	peer1.setShaftAngle(90);
	peer1.setSpeed(100);
	HostPlatform::advanceMs(10);
	peer1.sendReply(0xA1);
	CHECK(rmd->getPos() == 1000);
}

// =============================================================================================================
// R6.5 interpolation in the request mode
// =============================================================================================================

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.5 rx] a 0xA1 reply with speed 500 dps 10 ms after the previous one moves the position by 2.5 degrees" * KNOWN_ISSUE("B-RMD-6: interpolatedPos = speed / (1e6 * dt_us) (RmdMotorCAN.cpp:255) has the dimension 1/us^2 instead of degrees: 500 / (1e6 * 10000) = 5e-11")) {
	// HD: speed 500 dps * 0.010 s = 5.0 degrees of travel since the previous reply. The firmware averages the old position
	// and the new one: lastPos = (0 + (0 + 5.0)) / 2 = 2.5 degrees = 250 counts (previous position 0, angle field 0).
	auto rmd = makeRmd1();
	setRequestMode(*rmd, 1);
	peer1.setShaftAngle(0);
	peer1.setSpeed(0);
	peer1.sendReply(0xA1); // first reply: speed 0, lastPos stays (0 + 0) / 2 = 0, records the time
	HostPlatform::advanceMs(10);
	peer1.setSpeed(500);
	peer1.sendReply(0xA1);
	CHECK(rmd->getPos() == 250); // measured: 0 (lastPos about 2.5e-11)
}

// =============================================================================================================
// R6.6 0x94
// =============================================================================================================

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.6 rx] characterization: a 0x94 reply of 100.00 degrees after a position of 720 degrees gives 102 degrees (lastPos / 360 + angle)") {
	// FW: lastPos = 720 / 360 + 10000 * 0.01 = 2 + 100 = 102.0 degrees -> 10200 counts. SP 2.13 would place the position at
	// a multiple of 360 plus 100 degrees (820 degrees = 82000 counts): the firmware formula mixes revolutions and degrees.
	auto rmd = makeRmd1();
	peer1.setMultiTurn(72000);
	peer1.sendReply(0x92);
	REQUIRE(rmd->getPos() == 72000);
	peer1.setSingleTurn(10000);
	peer1.sendReply(0x94);
	CHECK(rmd->getPos() == 10200);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.6 rx] characterization: the driver never requests the single turn angle 0x94 in a session with request mode and thread cycles") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	setRequestMode(*rmd, 1);
	for (int i = 0; i < 30; i++) {
		runFor(100);
		rmd->getPos();
	}
	CHECK(peer1.count(0x94) == 0);
}

// =============================================================================================================
// R6.7 float resolution
// =============================================================================================================

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.7 rx] characterization: a position of 100 000 037 counts is read back as 100 000 040, 3 counts too high (float resolution above 2^24 counts)") {
	// HD (single precision): 100000037 * 0.01 = 1000000.37 -> float 1000000.375 (spacing 0.0625 at 2^19..2^20);
	// * 100 = 100000037.5 -> float spacing 8 at 2^26..2^27 -> 100000040. The expected value 100000040 is an
	// informational characterization of the float based position, not a requirement.
	auto rmd = makeRmd1();
	peer1.setMultiTurn(100000037);
	peer1.sendReply(0x92);
	const int32_t pos = rmd->getPos();
	CHECK(pos == 100000040); // HD above: lost counts, +3
}

// =============================================================================================================
// R7 status, errors, safety
// =============================================================================================================

namespace {
struct RmdSafetyFixture : RmdRxFixture {
	/** Driver is available and active (startMotor sent), the log is cleared */
	template <class Ptr>
	void ready(Ptr &rmd) {
		reachAvailable(*rmd);
		rmd->startMotor();
		clearTxFrames();
	}
};
} // namespace

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.1 rx] a 0x9A reply with an error flag sends the stop frames 0xA1 with zero torque and then 0x80") {
	auto rmd = makeRmd1();
	ready(rmd);
	peer1.setErrors(0x0004);
	peer1.sendReply(0x9A);
	const auto f = txFrames(ID1);
	REQUIRE(f.size() == 2);
	CHECK(is(f[0], {0xA1, 0, 0, 0, 0, 0, 0, 0})); // setTorque(0), SP 2.19
	CHECK(is(f[1], {0x80, 0, 0, 0, 0, 0, 0, 0})); // SP 2.17 shutdown
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.1 rx] a 0x9A reply with the flags 0x0016 returns them and makes motorReady false") {
	// SP 2.14 remark: 0x0016 = stall (0x2) + low voltage (0x4) + over current (0x10)
	auto rmd = makeRmd1();
	ready(rmd);
	REQUIRE(rmd->motorReady());
	peer1.setErrors(0x0016);
	peer1.sendReply(0x9A);
	CHECK(get(*rmd, "errors") == 0x16);
	CHECK_FALSE(rmd->motorReady());
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.1 rx] after an error flag turn sends nothing any more") {
	auto rmd = makeRmd1();
	ready(rmd);
	rmd->turn(1000);
	REQUIRE(txFramesCmd(ID1, 0xA1).size() == 1); // the motor is active and available: turn sends a torque frame
	peer1.setErrors(0x0004);
	peer1.sendReply(0x9A);
	clearTxFrames();
	rmd->turn(1000); // active is false
	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.1 rx] a flag in the high byte only (0x4000, encoder data error) also stops the motor") {
	auto rmd = makeRmd1();
	ready(rmd);
	peer1.setErrors(0x4000);
	peer1.sendReply(0x9A);
	CHECK(txFramesCmd(ID1, 0x80).size() == 1);
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.2 rx] startMotor is refused while error flags are set") {
	auto rmd = makeRmd1();
	peer1.setErrors(0x0004);
	peer1.sendReply(0x9A);
	clearTxFrames();
	rmd->startMotor();
	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.2 rx] startMotor sends a zero torque frame again after a 0x9A reply with 0 flags") {
	auto rmd = makeRmd1();
	peer1.setErrors(0x0004);
	peer1.sendReply(0x9A);
	peer1.setErrors(0);
	peer1.sendReply(0x9A);
	clearTxFrames();
	rmd->startMotor();
	const auto f = txFrames(ID1);
	REQUIRE(f.size() == 1);
	CHECK(is(f[0], {0xA1, 0, 0, 0, 0, 0, 0, 0}));
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.2 rx] a refused startMotor leaves turn silent") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	peer1.setErrors(0x0004);
	peer1.sendReply(0x9A);
	rmd->startMotor();
	clearTxFrames();
	rmd->turn(1000);
	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.3 rx] a 0x9A reply with 0 flags sends no frame") {
	auto rmd = makeRmd1();
	ready(rmd);
	peer1.setErrors(0);
	peer1.sendReply(0x9A);
	process();
	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.3 rx] a 0x9A reply with 0 flags keeps the motor active: turn still sends the torque") {
	auto rmd = makeRmd1();
	ready(rmd);
	peer1.sendReply(0x9A);
	rmd->turn(0x7fff); // HD: 32767 / 32767 * 1000 = 1000 = 0x03E8
	const auto f = txFramesCmd(ID1, 0xA1);
	REQUIRE(f.size() == 1);
	CHECK(is(f[0], {0xA1, 0, 0, 0, 0xE8, 0x03, 0, 0}));
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.4 rx] characterization: in the passive mode the driver never requests the status 0x9A over several cycles") {
	// Owner question 50: no status polling in the passive default
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	runFor(5000);
	CHECK(peer1.count(0x9A) == 0);
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.4 rx] characterization: in the passive mode the error flags of the motor are never seen") {
	auto rmd = makeRmd1();
	peer1.setErrors(0x0010); // over current in the motor
	reachAvailable(*rmd);
	runFor(5000);
	CHECK(get(*rmd, "errors") == 0);
	CHECK(rmd->motorReady()); // the motor is reported ready although it reports an over current (when asked)
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.4 rx] in the request mode the status 0x9A is requested and the motor error is seen within the first cycle") {
	auto rmd = makeRmd1();
	setRequestMode(*rmd, 1);
	peer1.setErrors(0x0010);
	runFor(200); // 0xB5 at 100 ms, 0x9A at 120 ms
	CHECK(peer1.count(0x9A) == 1);
	CHECK(get(*rmd, "errors") == 0x0010);
	CHECK_FALSE(rmd->motorReady());
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.5 rx] motorReady is false before the first cycle ends even when no error is set") {
	auto rmd = makeRmd1();
	peer1.sendReply(0xA1);
	CHECK_FALSE(rmd->motorReady()); // available is only updated at the end of a cycle
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.5 rx] motorReady is true when the motor is available and no error flag is set") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	CHECK(rmd->motorReady());
	CHECK(get(*rmd, "errors") == 0);
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.5 rx] motorReady is false when the motor is available but an error flag is set") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	peer1.setErrors(0x0008);
	peer1.sendReply(0x9A);
	CHECK_FALSE(rmd->motorReady());
}

TEST_CASE_FIXTURE(RmdSafetyFixture, "[R7.5 rx] motorReady is true again after a 0x9A reply with 0 flags while the motor stays available") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	peer1.setErrors(0x0008);
	peer1.sendReply(0x9A);
	REQUIRE_FALSE(rmd->motorReady());
	peer1.setErrors(0);
	peer1.sendReply(0x9A);
	CHECK(rmd->motorReady());
}

// =============================================================================================================
// Review additions
// =============================================================================================================

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.2 rx] characterization: in the request mode a 0xA1 reply with angle 0 and speed 0 halves the position reported by 0x92 (36000 -> 18000)") {
	// FW: 0x92 of 36000 -> lastPos 360.0 degrees. 0xA1: lastPos = (360 + (0 + 0)) / 2 = 180 degrees = 18000 counts
	auto rmd = makeRmd1();
	setRequestMode(*rmd, 1);
	peer1.setMultiTurn(36000);
	peer1.sendReply(0x92);
	peer1.setShaftAngle(0);
	peer1.setSpeed(0);
	peer1.sendReply(0xA1);
	CHECK(rmd->getPos() == 18000);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.2 rx] characterization: speed 32767 dps in a 0xA1 reply 1 us after the previous one adds speed / (1e6 * 1) / 2 degrees: getPos 1") {
	// HD: first reply: lastPos 0 (speed 0), time recorded. Second reply 1 us later: interpolation = 32767 / (1e6 * 1) = 0.032767,
	// lastPos = (0 + (0 + 0.032767)) / 2 = 0.0163835 degrees, * 100 = 1.63835 -> truncated 1. Both speed bytes are used.
	auto rmd = makeRmd1();
	setRequestMode(*rmd, 1);
	peer1.sendReply(0xA1);
	HostPlatform::advanceUs(1);
	peer1.setSpeed(32767);
	peer1.sendReply(0xA1);
	CHECK(rmd->getPos() == 1);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.2 rx] characterization: speed -32768 dps 1 us after the previous reply gives getPos -1") {
	// HD: interpolation = -32768 / 1e6 = -0.032768; lastPos = -0.016384 degrees, * 100 = -1.6384 -> truncated -1
	auto rmd = makeRmd1();
	setRequestMode(*rmd, 1);
	peer1.sendReply(0xA1);
	HostPlatform::advanceUs(1);
	peer1.setSpeed(-32768);
	peer1.sendReply(0xA1);
	CHECK(rmd->getPos() == -1);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.2 rx] characterization: every 0xA1 reply updates the time base (dt of the third reply is 1 us, not 1001 us)") {
	// HD: reply A (speed 0). 1000 us later reply B (speed 32767): interpolation 32767 / (1e6 * 1000) = 3.2767e-5, lastPos about 1.6e-5.
	// 1 us after B reply C: dt = 1 us -> interpolation 0.032767, lastPos = (1.6e-5 + 0.032767) / 2 = 0.016392 degrees -> 1.639 -> 1 count.
	// If B did not update the time base, dt would be 1001 us: interpolation 3.3e-5 -> 0 counts.
	auto rmd = makeRmd1();
	setRequestMode(*rmd, 1);
	peer1.sendReply(0xA1);
	HostPlatform::advanceUs(1000);
	peer1.setSpeed(32767);
	peer1.sendReply(0xA1);
	HostPlatform::advanceUs(1);
	peer1.sendReply(0xA1);
	CHECK(rmd->getPos() == 1);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.2 rx] the node 2 driver reads the temperature of a 0xA1 reply of node 2") {
	auto rmd = makeRmd2();
	peer2.setTemperature(33);
	peer2.sendReply(0xA1);
	CHECK(get(*rmd, "temp") == 33);
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R5.6 rx] a reply with the unknown command byte 0x60 received during a cycle makes the motor available at the end of the cycle") {
	auto rmd = makeRmd1();
	peer1.setSilent(true);
	runFor(500);
	REQUIRE(rxRaw(peer1, {0x60, 0, 0, 0, 0, 0, 0, 0}) != 0);
	runFor(769);
	CHECK_FALSE(rmd->motorReady());
	runFor(2);
	CHECK(rmd->motorReady());
}

TEST_CASE_FIXTURE(RmdRxFixture, "[R6.6 rx] characterization: a 0x94 reply refreshes the position age: no 0x92 request 10 ms later in the request mode") {
	auto rmd = makeRmd1();
	setRequestMode(*rmd, 1);
	peer1.sendReply(0x94); // lastAngleUpdate = tick
	clearTxFrames();
	HostPlatform::advanceMs(10);
	rmd->getPos();
	CHECK(txFramesCmd(ID1, 0x92).empty()); // without the refresh lastAngleUpdate would still be 0 and a request would go out
}

} // TEST_SUITE
