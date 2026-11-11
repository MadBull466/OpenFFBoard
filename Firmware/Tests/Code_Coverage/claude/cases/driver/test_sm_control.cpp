/*
 * SimpleMotion driver, phase P15 T2, work order SMC: motor control, telemetry and the control commands.
 * Charter: Tests/Code_Coverage/claude/charters/driver/SIMPLEMOTION_REQUIREMENTS.md
 *
 * Requirements covered: R8.1 (startMotor), R8.2 (stopMotor), R8.3 (emergencyStop, restart), R9.1 to R9.4 (voltage, torque,
 * failed reads), R10 (commands voltage, torque, state, restart and their access flags).
 *
 * Oracles: SM (SimpleMotion V2 library and docs, Tests/Code_Coverage/shared/specs/SimpleMotionV2-master, never copied: SMP_CONTROL_BITS1
 * 2533 bit 0 enable, SMP_SYSTEM_CONTROL 554 value 2 = restart, SMP_ACTUAL_BUS_VOLTAGE 900 = 10 mV per count,
 * SMP_ACTUAL_TORQUE 901 = 560 counts per ampere, subpacket types and sizes); HD (hand derivations next to the assertions: the
 * expected request frames are built here from the SM frame layout with the fake's independent bitwise CRC-16/ARC,
 * FakeSmDevice::crc16arc, low byte first on the wire, never the firmware's table); IF (MotorDriver::emergencyStop).
 * The driver address is indeterminate (B-SM-1): it is read from the request frame and echoed into the expected frame.
 *
 * Frame derivations (all instant commands: {0x24, len, adr, payload[len], crc16 lo, crc16 hi}):
 *  - write of parameter P with value V: [set address P: 0x80 | (P >> 8), P & 0xff][32 bit value: type 0, 4 bytes big endian].
 *    startMotor P = 2533 = 0x09E5 -> 0x89 0xE5, V = 1 -> 00 00 00 01. Payload 6 bytes.
 *    stopMotor: same with V = 0. restart: P = 554 = 0x022A -> 0x82 0x2A, V = 2 -> 00 00 00 02.
 *  - read of P without a length (the device keeps the format of the last settings read, 24 bit, retLen 1):
 *    [set address 9: 0x80 0x09][24 bit write of P: 0x40 | (P >> 16), (P >> 8) & 0xff, P & 0xff].
 *    900 = 0x0384 -> 40 03 84. 901 = 0x0385 -> 40 03 85. Payload 5 bytes.
 *  - fast update {0x10, adr, p lo, p hi, 0, 0, crc8}: the zero setpoint is p = 0.
 *
 * Telemetry arithmetic (hand derivation, charter R9):
 *  - voltage mV = counts * 10: 1200 -> 12000, 3276 -> 32760 (largest value that fits int16_t), 4800 -> 48000.
 *  - torque mA = counts * 1000 / 560 with C integer division (truncation towards zero): 560 -> 1000, 56 -> 100, 280 -> 500,
 *    1120 -> 2000, 1 -> 1 (1000 / 560 = 1.79), 3 -> 5 (3000 / 560 = 5.36), -1 -> -1 (-1.79), -3 -> -5 (-5.36).
 *    The device returns the 22 bit two's complement value (R5.7: the firmware does not sign extend). A negative count n is
 *    sent as 0x400000 - |n| (22 bits); the driver takes the low 16 bits as int16_t, which is n again for |n| < 32768, so
 *    the result equals the intended signed value (shown by the tests, no marker needed).
 *
 * Host versus target: MotorSimplemotion.cpp is compiled with -funsigned-char here like the target. getVoltage() narrows
 * `uint32_t * 10` to int16_t (implementation defined, modular on both: B-SM-3 is the same on host and target);
 * getTorque() computes ((int16_t) torque_u * 1000) in int on both (32 bit int on host and target, no overflow for 16 bit
 * input: 32768 * 1000 = 3.3e7).
 *
 * KNOWN_ISSUE: B-SM-3 (R9.2) getVoltage() returns int16_t millivolts. Measured: 4800 counts -> 48000 mod 65536 -> -17536.
 *   Intended: 48000 mV (SM: 10 mV per count; the parameter holds a 22 bit value; the command is documented "Voltage in mV").
 *   The first failing assertion of both cases is the voltage value.
 * Platform note: stopMotor() = turn(0) then a write while the fast reply is still pending. The host xSemaphoreTake polls the
 * UART peer while it waits (host_freertos.cpp, P15), which stands in for the receive interrupt that gives the port back on the target.
 * KNOWN_ISSUE B-SM-6 (review finding): getTorque() narrows the milliampere value to int16_t: 20000 counts = 35714 mA wraps to -29822.
 * Characterizations (NEEDS REVIEW): R8.3 a restart clears the hardfault flag even though the device still reports the
 * fault stop: the next fast reply with the fault bit sets it again; R8.2 the order of stopMotor's two messages (fast update
 * then the write) is the firmware's design, the SM docs do not prescribe it.
 */
#include "doctest.h"
#include "SmFixture.h"

#include <cstdint>
#include <vector>

namespace {

using Bytes = std::vector<uint8_t>;

/** Instant command frame per the SM layout: {0x24, len, adr, payload, crc16 lo, crc16 hi} */
Bytes instantFrame(uint8_t adr, const Bytes &payload) {
	Bytes f = {0x24, static_cast<uint8_t>(payload.size()), adr};
	f.insert(f.end(), payload.begin(), payload.end());
	const uint16_t crc = FakeSmDevice::crc16arc(f.data(), f.size());
	f.push_back(static_cast<uint8_t>(crc & 0xff));
	f.push_back(static_cast<uint8_t>(crc >> 8));
	return f;
}

/** Payload of a 32 bit parameter write: [set address P][32 bit V] */
Bytes writePayload(uint16_t param, uint32_t value) {
	return {static_cast<uint8_t>(0x80 | (param >> 8)), static_cast<uint8_t>(param & 0xff),
			static_cast<uint8_t>((value >> 24) & 0x3f), static_cast<uint8_t>(value >> 16),
			static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
}

/** Payload of a read without a length: [set address 9][24 bit write of P] */
Bytes readPayload(uint16_t param) {
	return {0x80, 0x09, static_cast<uint8_t>(0x40 | (param >> 16)), static_cast<uint8_t>(param >> 8),
			static_cast<uint8_t>(param)};
}

/** 22 bit two's complement of a signed count, as the device sends it */
uint32_t twos22(int32_t n) { return static_cast<uint32_t>(n) & 0x3FFFFF; }

} // namespace

TEST_SUITE("SM control") {

// ---------------------------------------------------------------- R8.1 startMotor

TEST_CASE_FIXTURE(SmFixture, "[R8.1 start] startMotor sends one instant command that writes SMP_CONTROL_BITS1 (2533) = 1 with the SM frame bytes") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.clearFrames();
	sm->startMotor();
	REQUIRE(dev.frameCount() == 1);
	const auto &f = dev.frames()[0];
	CHECK(f.kind == FakeSmDevice::Kind::Instant);
	// HD: {24, 06, adr, 89 E5, 00 00 00 01, crc16 lo, hi}
	CHECK(f.bytes == instantFrame(lastAddress(), writePayload(2533, 1)));
}

TEST_CASE_FIXTURE(SmFixture, "[R8.1 start] startMotor is stored by the device as control bits 1 = 1 (enable bit 0)") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_CONTROL_BITS1] = 0;
	sm->startMotor();
	REQUIRE_FALSE(dev.writes().empty());
	CHECK(dev.writes().back().first == 2533);
	CHECK(dev.writes().back().second == 1);
	CHECK(dev.params[FakeSmDevice::SMP_CONTROL_BITS1] == 1);
}

TEST_CASE_FIXTURE(SmFixture, "[R8.1 start] startMotor releases the port: the next fast update is sent and no reply is dropped") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	sm->startMotor();
	dev.clearFrames();
	sm->turn(10);
	REQUIRE(dev.frameCount() == 1);
	CHECK(dev.frames()[0].kind == FakeSmDevice::Kind::Fast);
	CHECK(dev.droppedReplies() == 0);
}

// ---------------------------------------------------------------- R8.2 stopMotor

TEST_CASE_FIXTURE(SmFixture, "[R8.2 stop] stopMotor sends a fast update with the setpoint 0 even after a non-zero torque") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	sm->turn(300);
	process();
	dev.clearFrames();
	sm->stopMotor();
	REQUIRE(dev.frameCount() == 2); // the zero setpoint, then the CB1 write
	const auto &f = dev.frames()[0];
	CHECK(f.kind == FakeSmDevice::Kind::Fast);
	// HD: {0x10, adr, setpoint lo, hi = 0, write2 = 0 (unused, 2 bytes), crc8 init 0x52}
	std::vector<uint8_t> expected = {0x10, lastAddress(), 0, 0, 0, 0};
	expected.push_back(FakeSmDevice::crc8(expected.data(), 6, 0x52));
	CHECK(f.bytes == expected);
	CHECK(dev.setpoints().back() == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R8.2 stop] characterization: stopMotor writes SMP_CONTROL_BITS1 (2533) = 0 after the zero-torque fast update, with the SM frame bytes" ) {
	// FW: the order (fast update first, then the write) is the firmware's design; the content of the write is SM (enable bit cleared)
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_CONTROL_BITS1] = 1;
	dev.clearFrames();
	sm->stopMotor();
	process();
	REQUIRE(dev.frameCount() == 2);
	CHECK(dev.frames()[0].kind == FakeSmDevice::Kind::Fast);
	CHECK(dev.frames()[1].kind == FakeSmDevice::Kind::Instant);
	CHECK(dev.frames()[1].bytes == instantFrame(lastAddress(), writePayload(2533, 0)));
	CHECK(dev.params[FakeSmDevice::SMP_CONTROL_BITS1] == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R8.2 stop] after stopMotor the zero setpoint is the last setpoint the device saw") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	sm->turn(1234);
	process();
	sm->stopMotor();
	process();
	REQUIRE_FALSE(dev.setpoints().empty());
	CHECK(dev.setpoints().back() == 0);
}

// ---------------------------------------------------------------- R8.3 emergencyStop, restart

TEST_CASE_FIXTURE(SmFixture, "[R8.3 estop] emergencyStop(false) behaves as stopMotor: a zero-torque fast update, then control bits 1 = 0" ) {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_CONTROL_BITS1] = 1;
	dev.clearFrames();
	sm->emergencyStop(false);
	process();
	REQUIRE(dev.frameCount() == 2);
	CHECK(dev.frames()[0].kind == FakeSmDevice::Kind::Fast);
	CHECK(dev.setpoints().back() == 0);
	CHECK(dev.frames()[1].bytes == instantFrame(lastAddress(), writePayload(2533, 0)));
}

TEST_CASE_FIXTURE(SmFixture, "[R8.3 estop] emergencyStop(true) behaves as startMotor: control bits 1 = 1 is written") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.clearFrames();
	sm->emergencyStop(true);
	REQUIRE(dev.frameCount() == 1);
	CHECK(dev.frames()[0].bytes == instantFrame(lastAddress(), writePayload(2533, 1)));
	CHECK(dev.params[FakeSmDevice::SMP_CONTROL_BITS1] == 1);
}

TEST_CASE_FIXTURE(SmFixture, "[R8.3 restart] restart writes SMP_SYSTEM_CONTROL (554) = 2 with the SM frame bytes and the device sees one restart request") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.clearFrames();
	sm->restart();
	REQUIRE(dev.frameCount() == 1);
	// HD: payload 82 2A 00 00 00 02
	CHECK(dev.frames()[0].bytes == instantFrame(lastAddress(), writePayload(554, 2)));
	CHECK(dev.restartRequests() == 1);
	REQUIRE_FALSE(dev.writes().empty());
	CHECK(dev.writes().back().first == 554);
	CHECK(dev.writes().back().second == 2);
}

TEST_CASE_FIXTURE(SmFixture, "[R8.3 restart] restart clears the faultstop state so the motor is ready again") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	REQUIRE(sm->motorReady());
	dev.status16 = FakeSmDevice::STAT_INITIALIZED | FakeSmDevice::STAT_FAULTSTOP;
	sm->turn(0);
	process();
	REQUIRE_FALSE(sm->motorReady()); // bit 5 reported: hardfault
	sm->restart();
	CHECK(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R8.3 restart] characterization: restart clears the fault flag even when the device still reports the fault stop, the next fast reply sets it again") {
	// FW: the firmware clears hardfault without looking at the device; the fake does not clear the fault on a restart request
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.status16 = FakeSmDevice::STAT_INITIALIZED | FakeSmDevice::STAT_FAULTSTOP;
	sm->turn(0);
	process();
	REQUIRE_FALSE(sm->motorReady());
	sm->restart();
	REQUIRE(sm->motorReady());
	sm->turn(0);
	process();
	CHECK_FALSE(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R8.3 restart] after restart the next fast update is sent without waiting") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	sm->restart();
	dev.clearFrames();
	sm->turn(5);
	REQUIRE(dev.frameCount() == 1);
	CHECK(dev.frames()[0].kind == FakeSmDevice::Kind::Fast);
	CHECK(dev.droppedReplies() == 0);
}

// ---------------------------------------------------------------- R9.1 voltage

TEST_CASE_FIXTURE(SmFixture, "[R9.1 voltage] getVoltage requests parameter 900 with the SM frame bytes (read without length)") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.clearFrames();
	sm->getVoltage();
	REQUIRE(dev.frameCount() == 1);
	// HD: payload 80 09 40 03 84 (900 = 0x0384)
	CHECK(dev.frames()[0].bytes == instantFrame(lastAddress(), readPayload(900)));
	REQUIRE_FALSE(dev.returnAddressRequests().empty());
	CHECK(dev.returnAddressRequests().back() == 900);
}

TEST_CASE_FIXTURE(SmFixture, "[R9.1 voltage] 1200 counts of parameter 900 are 12000 mV (10 mV per count)") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_BUS_VOLTAGE] = 1200;
	CHECK(sm->getVoltage() == 12000);
}

TEST_CASE_FIXTURE(SmFixture, "[R9.1 voltage] 3276 counts are 32760 mV, the largest value an int16_t result holds") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_BUS_VOLTAGE] = 3276;
	CHECK(sm->getVoltage() == 32760);
}

TEST_CASE_FIXTURE(SmFixture, "[R9.1 voltage] 0 counts are 0 mV") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_BUS_VOLTAGE] = 0;
	CHECK(sm->getVoltage() == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R9.1 voltage] the voltage command returns 12000 for 1200 counts") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_BUS_VOLTAGE] = 1200;
	CHECK(CommandHelpers::get(*sm, "voltage") == 12000);
}

TEST_CASE_FIXTURE(SmFixture, "[R9.1 voltage] the voltage follows the device value on every call"
								 ) {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_BUS_VOLTAGE] = 1200;
	CHECK(sm->getVoltage() == 12000);
	dev.params[FakeSmDevice::SMP_ACTUAL_BUS_VOLTAGE] = 2400;
	CHECK(sm->getVoltage() == 24000);
}

// ---------------------------------------------------------------- R9.2 voltage above 32767 mV

TEST_CASE_FIXTURE(SmFixture, "[R9.2 voltage] getVoltage of 4800 counts is 48000 mV and does not wrap negative"
								 * KNOWN_ISSUE("B-SM-3: getVoltage() returns int16_t, 4800 counts * 10 = 48000 wraps to -17536 (48000 - 65536)")) {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_BUS_VOLTAGE] = 4800;
	const int result = sm->getVoltage();
	CHECK(result == 48000); // measured -17536
}

TEST_CASE_FIXTURE(SmFixture, "[R9.2 voltage] getVoltage of 3277 counts (the first count above the int16 range) is 32770 mV and does not wrap negative"
								 * KNOWN_ISSUE("B-SM-3: 3277 counts * 10 = 32770 wraps to -32766 (32770 - 65536)")) {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_BUS_VOLTAGE] = 3277;
	const int result = sm->getVoltage();
	CHECK(result == 32770); // measured -32766
}

TEST_CASE_FIXTURE(SmFixture, "[R9.2 voltage] the voltage command of 4800 counts replies 48000 mV and not a negative number"
								 * KNOWN_ISSUE("B-SM-3: the command replies the int16_t result, 48000 becomes -17536")) {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_BUS_VOLTAGE] = 4800;
	CHECK(CommandHelpers::get(*sm, "voltage") == 48000); // measured -17536
}

// ---------------------------------------------------------------- R9.3 torque

TEST_CASE_FIXTURE(SmFixture, "[R9.3 torque] getTorque requests parameter 901 with the SM frame bytes (read without length)") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.clearFrames();
	sm->getTorque();
	REQUIRE(dev.frameCount() == 1);
	// HD: payload 80 09 40 03 85 (901 = 0x0385)
	CHECK(dev.frames()[0].bytes == instantFrame(lastAddress(), readPayload(901)));
	REQUIRE_FALSE(dev.returnAddressRequests().empty());
	CHECK(dev.returnAddressRequests().back() == 901);
}

TEST_CASE_FIXTURE(SmFixture, "[R9.3 torque] 560 counts of parameter 901 are 1000 mA (560 counts per ampere)") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_TORQUE] = 560;
	CHECK(sm->getTorque() == 1000);
}

TEST_CASE_FIXTURE(SmFixture, "[R9.3 torque] positive counts convert as counts * 1000 / 560 (integer division)") {
	// HD: 0 -> 0, 56 -> 100, 280 -> 500, 1120 -> 2000, 1 -> 1 (1.79 truncated), 3 -> 5 (5.36 truncated)
	struct Row { uint32_t counts; int mA; };
	const Row rows[] = {{0, 0}, {56, 100}, {280, 500}, {1120, 2000}, {1, 1}, {3, 5}};
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	for (const Row &r : rows) {
		INFO("counts " << r.counts);
		dev.params[FakeSmDevice::SMP_ACTUAL_TORQUE] = r.counts;
		CHECK(sm->getTorque() == r.mA);
	}
}

TEST_CASE_FIXTURE(SmFixture, "[R9.3 torque] -560 counts (22 bit two's complement 0x3FFDD0) are -1000 mA") {
	// HD: 0x400000 - 560 = 0x3FFDD0; the low 16 bits 0xFDD0 as int16_t = -560; -560 * 1000 / 560 = -1000
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_TORQUE] = twos22(-560);
	REQUIRE(dev.params[FakeSmDevice::SMP_ACTUAL_TORQUE] == 0x3FFDD0);
	CHECK(sm->getTorque() == -1000);
}

TEST_CASE_FIXTURE(SmFixture, "[R9.3 torque] negative counts convert with truncation towards zero") {
	// HD: -56 -> -100, -280 -> -500, -1 -> -1 (-1.79 truncated), -3 -> -5 (-5.36 truncated)
	struct Row { int32_t counts; int mA; };
	const Row rows[] = {{-56, -100}, {-280, -500}, {-1, -1}, {-3, -5}};
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	for (const Row &r : rows) {
		INFO("counts " << r.counts);
		dev.params[FakeSmDevice::SMP_ACTUAL_TORQUE] = twos22(r.counts);
		CHECK(sm->getTorque() == r.mA);
	}
}

TEST_CASE_FIXTURE(SmFixture, "[R9.3 torque] getTorque of 20000 counts is 35714 mA and does not wrap negative"
								 * KNOWN_ISSUE("B-SM-6: getTorque() returns int16_t mA: 20000 counts * 1000 / 560 = 35714 wraps to -29822 (35714 - 65536); same shape as B-SM-3")) {
	// HD: 20000 * 1000 = 20000000 (int arithmetic), / 560 = 35714 (truncated); SM: 560 counts per ampere. int16_t holds up to 32767 mA (18350 counts)
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_TORQUE] = 20000;
	const int result = sm->getTorque();
	CHECK(result == 35714); // measured -29822
}

TEST_CASE_FIXTURE(SmFixture, "[R9.3 torque] the torque command returns 1000 for 560 counts and -1000 for -560 counts") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_TORQUE] = 560;
	CHECK(CommandHelpers::get(*sm, "torque") == 1000);
	dev.params[FakeSmDevice::SMP_ACTUAL_TORQUE] = twos22(-560);
	CHECK(CommandHelpers::get(*sm, "torque") == -1000);
}

// ---------------------------------------------------------------- R9.4 failed reads

TEST_CASE_FIXTURE(SmFixture, "[R9.4 failed] getVoltage returns 0 when the device does not answer") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_BUS_VOLTAGE] = 1200;
	dev.silent = true;
	CHECK(sm->getVoltage() == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R9.4 failed] getTorque returns 0 when the device does not answer") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_TORQUE] = 560;
	dev.silent = true;
	CHECK(sm->getTorque() == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R9.4 failed] a failed telemetry read counts one uart error (uarterr command)") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.silent = true;
	sm->getVoltage();
	CHECK(CommandHelpers::get(*sm, "uarterr") == 1);
}

TEST_CASE_FIXTURE(SmFixture, "[R9.4 failed] after a failed telemetry read the driver releases the wait and the next fast update is sent") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.silent = true;
	REQUIRE(sm->getVoltage() == 0);
	dev.silent = false;
	advanceMs(11); // the port is released after the timeout once the last byte is older than 10 ms
	dev.clearFrames();
	sm->turn(5);
	REQUIRE(dev.frameCount() == 1);
	CHECK(dev.frames()[0].kind == FakeSmDevice::Kind::Fast);
	CHECK(dev.setpoints().back() == 5);
}

TEST_CASE_FIXTURE(SmFixture, "[R9.4 failed] after a failed read the next read works again when the device answers") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_BUS_VOLTAGE] = 1200;
	dev.silent = true;
	REQUIRE(sm->getVoltage() == 0);
	dev.silent = false;
	advanceMs(1); // the port is released after a timeout only when the last byte is older than 10 ms (strict >): an immediate retry is refused
	CHECK(sm->getVoltage() == 12000);
}

TEST_CASE_FIXTURE(SmFixture, "[R9.4 failed] the voltage and torque commands reply 0 with status OK when the device does not answer") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.silent = true;
	std::vector<CommandReply> replies;
	CHECK(CommandHelpers::run(*sm, "voltage", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 0);
	replies.clear();
	CHECK(CommandHelpers::run(*sm, "torque", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 0);
}

// ---------------------------------------------------------------- R10 commands

TEST_CASE_FIXTURE(SmFixture, "[R10.1 state] the state command returns the status word of the settings read") {
	auto sm = makeSm(1);
	dev.params[FakeSmDevice::SMP_STATUS] = 0x1001;
	REQUIRE(initialize(*sm));
	CHECK(CommandHelpers::get(*sm, "state") == 0x1001);
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 state] the state command returns the status word of the last fast reply") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.status16 = FakeSmDevice::STAT_INITIALIZED | 0x0003;
	sm->turn(0);
	process();
	CHECK(CommandHelpers::get(*sm, "state") == (FakeSmDevice::STAT_INITIALIZED | 0x0003));
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 state] the state command shows the faultstop bit 5 after a faulted reply") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.status16 = FakeSmDevice::STAT_INITIALIZED | FakeSmDevice::STAT_FAULTSTOP;
	sm->turn(0);
	process();
	const int64_t st = CommandHelpers::get(*sm, "state");
	CHECK((st & FakeSmDevice::STAT_FAULTSTOP) != 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 restart] the restart command writes 554 = 2 and replies nothing with status OK") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.clearFrames();
	std::vector<CommandReply> replies;
	CHECK(CommandHelpers::run(*sm, "restart", CMDtype::get, 0, &replies) == CommandStatus::OK);
	CHECK(replies.empty());
	REQUIRE(dev.frameCount() == 1);
	CHECK(dev.frames()[0].bytes == instantFrame(lastAddress(), writePayload(554, 2)));
	CHECK(dev.restartRequests() == 1);
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 restart] the restart command clears the faultstop state") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.status16 = FakeSmDevice::STAT_INITIALIZED | FakeSmDevice::STAT_FAULTSTOP;
	sm->turn(0);
	process();
	REQUIRE_FALSE(sm->motorReady());
	CommandHelpers::run(*sm, "restart", CMDtype::get);
	CHECK(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R2.3 flags] voltage, torque, state and restart are get-only commands") {
	auto sm = makeSm(1);
	for (const char *name : {"voltage", "torque", "state", "restart"}) {
		INFO(name);
		CmdHandlerCommanddef *def = sm->getCommandFromName(name);
		REQUIRE(def != nullptr);
		CHECK(def->flags == CMDFLAG_GET);
	}
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 id] the control commands of instance 2 behave like instance 1 (voltage, torque)") {
	auto sm = makeSm(2);
	REQUIRE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_ACTUAL_BUS_VOLTAGE] = 1200;
	dev.params[FakeSmDevice::SMP_ACTUAL_TORQUE] = 560;
	CHECK(CommandHelpers::get(*sm, "voltage") == 12000);
	CHECK(CommandHelpers::get(*sm, "torque") == 1000);
}

}
