/*
 * ODrive CAN driver, groups G3 (received frames, O20-O26) and G5 (torque and encoder, O40-O44) of
 * Tests/Code_Coverage/claude/charters/driver/ODRIVE_REQUIREMENTS.md (lead decisions L1-L11).
 *
 * Sources: SPEC = "CAN Protocol - ODrive Documentation 0.5.6", CSV = its "CAN Messages" table. Layouts used (all little
 * endian, SPEC): 0x001 heartbeat: axis error uint32 @0, axis state uint8 @4, flags in bytes 5-7; 0x009 encoder
 * estimates: position float32 @0, velocity float32 @4; 0x017 bus voltage float32 @0, current float32 @4; 0x00E input
 * torque float32 @0 (Nm). Standard id = node << 5 | cmd (SPEC "CAN Frame").
 *
 * Received frames are delivered synchronously by the fake HAL (canReceive -> RX interrupt -> canRxPendCallback), so the
 * RX tests inject a frame and read the result at once through the public interface (commands errors / state / vbus /
 * connected, getPos_f / getPos). The tests of the RX path avoid the enum numbers (L2) by using raw state bytes.
 *
 * Time base: the fixture starts at 10 s of simulated time, so that the driver's counters that start at 0
 * (lastCanMessage, lastVoltageUpdate, lastPosTime) are well in the past and no test depends on the start-up window of
 * B-O2 (connected true for the first 2 s).
 *
 * Host vs target: float values are exactly representable IEEE 754 single precision numbers wherever they are compared
 * exactly; the others use a tolerance. The float -> int32 conversion of getPos for positions beyond +-32768 turns
 * differs (x86: INT32_MIN, ARM vcvt: saturates), see O44. The driver reads the received payload through a uint64_t
 * cast of the byte array (ODriveCAN.cpp:394, aliasing / alignment): fine on the host, flagged for the target.
 */
#include "doctest.h"
#include "OdriveFixture.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace {

using CommandHelpers::get;
using CommandHelpers::run;

constexpr uint32_t CMD_HB = 0x01, CMD_ENC = 0x09, CMD_VBUS = 0x17, CMD_TORQUE = 0x0E;

struct OdrvRxFixture : OdriveFixture {
	OdrvRxFixture() {
		HostPlatform::advanceMs(10000);
		peer0.reset(); // cyclic timers relative to the new time
		peer1.reset();
	}

	/** Brings the driver to "connected" without the start-up sequence: one heartbeat with a state that triggers nothing (2), 500 ms (first thread body) */
	void link(ODriveCAN &od, FakeOdrivePeer &p) {
		p.setHeartbeat(false);
		heartbeat(p, 2);
		runFor(500);
		REQUIRE(get(od, "connected") == 1);
	}

	/** Injects a standard frame of `node` / `cmd` as if sent by the ODrive. @return canReceive result */
	uint32_t rx(FakeOdrivePeer &p, uint8_t node, uint8_t cmd, const std::vector<uint8_t> &data, bool rtr = false) {
		return p.inject((uint32_t(node) << 5) | cmd, data, false, rtr);
	}
};

std::vector<uint8_t> le32(uint32_t v) { return {uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)}; }
std::vector<uint8_t> f32le(float f) {
	uint32_t u;
	std::memcpy(&u, &f, 4);
	return le32(u);
}
std::vector<uint8_t> cat(std::vector<uint8_t> a, const std::vector<uint8_t> &b) {
	a.insert(a.end(), b.begin(), b.end());
	return a;
}
/** 8 byte payload of two floats */
std::vector<uint8_t> floats(float a, float b) { return cat(f32le(a), f32le(b)); }
/** Heartbeat payload (CSV 0x001): error uint32 @0, state uint8 @4, bytes 5-7 as given */
std::vector<uint8_t> hb(uint32_t err, uint8_t state, uint8_t b5 = 0, uint8_t b6 = 0, uint8_t b7 = 0) {
	return cat(le32(err), {state, b5, b6, b7});
}
float f32at(const HostTarget::CanTxFrame &f) {
	float v;
	std::memcpy(&v, f.data.data(), 4);
	return v;
}

} // namespace

TEST_SUITE("ODrive received frames") {

// ---------------------------------------------------------------------------------------------------------------
// O20 heartbeat
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(OdrvRxFixture, "[O20] the axis error of a heartbeat is the uint32 little endian value of bytes 0-3 and is read by the errors command") {
	auto od = makeOdrive1();
	link(*od, peer0);
	// HD: bytes 78 56 34 12 little endian = 0x12345678
	CHECK(rx(peer0, 0, CMD_HB, {0x78, 0x56, 0x34, 0x12, 0x02, 0, 0, 0}) > 0);
	CHECK(get(*od, "errors") == 0x12345678);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O20] the axis state of a heartbeat is the uint8 at byte 4 and is read by the state command") {
	auto od = makeOdrive1();
	link(*od, peer0);
	// raw state byte 0x5A (not an enum number, L2): the command returns the byte as received
	CHECK(rx(peer0, 0, CMD_HB, {0, 0, 0, 0, 0x5A, 0, 0, 0}) > 0);
	CHECK(get(*od, "state") == 0x5A);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O20] a later heartbeat replaces the error and the state of the earlier one") {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_HB, hb(0xDEADBEEF, 0x21));
	REQUIRE(get(*od, "errors") == 0xDEADBEEF);
	REQUIRE(get(*od, "state") == 0x21);
	rx(peer0, 0, CMD_HB, hb(0, 0x22));
	CHECK(get(*od, "errors") == 0);
	CHECK(get(*od, "state") == 0x22);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O20] the peer's heartbeat frame (fake) is decoded to the same error and state as the hand-built bytes") {
	auto od = makeOdrive1();
	link(*od, peer0);
	heartbeat(peer0, 0x33, 0xA5A5A5A5);
	CHECK(get(*od, "errors") == 0xA5A5A5A5);
	CHECK(get(*od, "state") == 0x33);
}

// ---------------------------------------------------------------------------------------------------------------
// O21 flag bytes
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(OdrvRxFixture, "[O21] set motor, encoder, controller and trajectory-done flag bits in bytes 5-7 leave the axis error and the state unchanged") {
	// CSV 0x001: flags are single bits @5.0, @6.0, @7.0 and @7.7; the driver stores whole bytes 5-7 (unused, L4: no
	// observable effect). What can be observed: errors and state read only bytes 0-3 and 4.
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_HB, hb(0x00000010, 0x21, 0xFF, 0xFF, 0xFF));
	CHECK(get(*od, "errors") == 0x10);
	CHECK(get(*od, "state") == 0x21);
	peer0.setFlags(true, true, true, true);
	peer0.setState(0x22);
	peer0.setAxisError(0x20);
	peer0.sendHeartbeat();
	CHECK(get(*od, "errors") == 0x20);
	CHECK(get(*od, "state") == 0x22);
}

// ---------------------------------------------------------------------------------------------------------------
// O22 encoder estimates
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(OdrvRxFixture, "[O22] the encoder estimate position is the float32 little endian value at byte 0 and is returned by getPos_f") {
	auto od = makeOdrive1();
	link(*od, peer0);
	// HD: 1.5f = 0x3FC00000 -> bytes 00 00 C0 3F; velocity word 0.5f = 00 00 00 3F must not be used for the position
	CHECK(rx(peer0, 0, CMD_ENC, {0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x3F}) > 0);
	CHECK(od->getPos_f() == 1.5f);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O22] a negative position estimate is returned as a negative position") {
	auto od = makeOdrive1();
	link(*od, peer0);
	// HD: -2.25f = 0xC0100000 -> bytes 00 00 10 C0
	rx(peer0, 0, CMD_ENC, {0x00, 0x00, 0x10, 0xC0, 0, 0, 0, 0});
	CHECK(od->getPos_f() == -2.25f);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O22] the velocity word does not change the position and the next estimate replaces the position") {
	auto od = makeOdrive1();
	link(*od, peer0);
	peer0.setEncoder(1.5f, -2.25f);
	peer0.sendEncoderEstimates();
	CHECK(od->getPos_f() == 1.5f);
	peer0.setEncoder(0.25f, 100.0f);
	peer0.sendEncoderEstimates();
	CHECK(od->getPos_f() == 0.25f);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O22] characterization: getPos is 65535 times the position in turns, truncated towards zero") {
	// cpr 0xFFFF is firmware (FW). HD: 1.5 * 65535 = 98302.5 -> 98302; -1.5 -> -98302.5 -> -98302 (both exact in float)
	auto od = makeOdrive1();
	link(*od, peer0);
	peer0.setEncoder(1.5f, 0);
	peer0.sendEncoderEstimates();
	CHECK(od->getPos() == 98302);
	peer0.setEncoder(-1.5f, 0);
	peer0.sendEncoderEstimates();
	CHECK(od->getPos() == -98302);
}

// ---------------------------------------------------------------------------------------------------------------
// O23 bus voltage
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(OdrvRxFixture, "[O23] the bus voltage is the float32 little endian value at byte 0 and the vbus command reports it in millivolts") {
	auto od = makeOdrive1();
	link(*od, peer0);
	// HD: 24.5f = 0x41C40000 -> bytes 00 00 C4 41; current word 3.0f ignored; 24.5 V = 24500 mV
	CHECK(rx(peer0, 0, CMD_VBUS, {0x00, 0x00, 0xC4, 0x41, 0x00, 0x00, 0x40, 0x40}) > 0);
	CHECK(get(*od, "vbus") == 24500);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O23] a later bus voltage reply replaces the earlier one") {
	auto od = makeOdrive1();
	link(*od, peer0);
	peer0.setBus(24.5f, 0);
	peer0.sendBusVoltage();
	REQUIRE(get(*od, "vbus") == 24500);
	peer0.setBus(12.0f, 1.0f); // HD: 12 V = 12000 mV
	peer0.sendBusVoltage();
	CHECK(get(*od, "vbus") == 12000);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O23] characterization: the vbus command truncates sub-millivolt fractions") {
	// HD: float(12.3456) = 12.345600128..., times 1000 in float = 12345.6 -> 12345 (truncation, FW)
	auto od = makeOdrive1();
	link(*od, peer0);
	peer0.setBus(12.3456f, 0);
	peer0.sendBusVoltage();
	CHECK(get(*od, "vbus") == 12345);
}

// ---------------------------------------------------------------------------------------------------------------
// O24 other nodes
// ---------------------------------------------------------------------------------------------------------------

// Note: the O24 tests are end-to-end only. The CAN filter of the driver (node bits, mask 0x7E0) already drops the frames
// of other nodes, so the driver's own node check (ODriveCAN.cpp:390-393) is unreachable through the public interface.

TEST_CASE_FIXTURE(OdrvRxFixture, "[O24] a heartbeat of another node does not change the error or the state of node 0") {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_HB, hb(0x11, 0x21));
	rx(peer0, 1, CMD_HB, hb(0x99, 0x77));
	CHECK(get(*od, "errors") == 0x11);
	CHECK(get(*od, "state") == 0x21);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O24] encoder estimates of another node do not change the position of node 0") {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(1.5f, 0));
	rx(peer0, 1, CMD_ENC, floats(7.0f, 0));
	rx(peer0, 5, CMD_ENC, floats(8.0f, 0));
	CHECK(od->getPos_f() == 1.5f);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O24] a bus voltage reply of another node does not change the voltage of node 0") {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_VBUS, floats(24.5f, 0));
	rx(peer0, 1, CMD_VBUS, floats(48.0f, 0));
	rx(peer0, 63, CMD_VBUS, floats(48.0f, 0));
	CHECK(get(*od, "vbus") == 24500);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O24] with both instances present each takes only the frames of its own node") {
	auto od1 = makeOdrive1();
	auto od2 = makeOdrive2();
	link(*od1, peer0);
	link(*od2, peer1);
	rx(peer0, 0, CMD_HB, hb(0x11, 0x21));
	rx(peer1, 1, CMD_HB, hb(0x22, 0x31));
	CHECK(get(*od1, "errors") == 0x11);
	CHECK(get(*od1, "state") == 0x21);
	CHECK(get(*od2, "errors") == 0x22);
	CHECK(get(*od2, "state") == 0x31);
	rx(peer1, 1, CMD_ENC, floats(2.0f, 0));
	CHECK(od2->getPos_f() == 2.0f);
	CHECK(od1->getPos_f() == 0.0f);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O24] frames of another node do not keep node 0 connected") {
	auto od = makeOdrive1();
	link(*od, peer0); // one heartbeat of node 0 at t0, no more
	peer0.setSilent(true); // also no answers to the driver's own voltage requests
	peer1.setHeartbeat(true, 100);
	runFor(2600); // bodies at 1000 .. 2500 (relative); the last frame of node 0 is older than 2000 ms from 2000 on
	CHECK(get(*od, "connected") == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// O25 RTR / short frames
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(OdrvRxFixture, "[O25] a valid 8 byte encoder estimate data frame is used as the position (reference for the O25 defect tests)") {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(1.5f, 0));
	CHECK(od->getPos_f() == 1.5f);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O25] an RTR request 0x09 seen on the bus does not overwrite the position" * KNOWN_ISSUE("B-O3: canRxPendCallback has no DLC / RTR check (ODriveCAN.cpp:390-435), the empty payload of the request is read as position 0")) {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(1.5f, 0));
	REQUIRE(od->getPos_f() == 1.5f);
	rx(peer0, 0, CMD_ENC, {}, true); // another master's request: RTR, DLC 0 (SPEC call and response)
	CHECK(od->getPos_f() == 1.5f);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O25] an RTR request 0x17 seen on the bus does not overwrite the bus voltage" * KNOWN_ISSUE("B-O3: canRxPendCallback has no DLC / RTR check (ODriveCAN.cpp:390-435), the empty payload of the request is read as voltage 0")) {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_VBUS, floats(24.5f, 0));
	REQUIRE(get(*od, "vbus") == 24500);
	rx(peer0, 0, CMD_VBUS, {}, true);
	CHECK(get(*od, "vbus") == 24500);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O25] an encoder estimate data frame with DLC 2 does not overwrite the position" * KNOWN_ISSUE("B-O3: canRxPendCallback has no DLC check (ODriveCAN.cpp:415-425), missing bytes are read as 0")) {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(1.5f, 0));
	REQUIRE(od->getPos_f() == 1.5f);
	rx(peer0, 0, CMD_ENC, {0x00, 0x00}); // CSV 0x009: 8 byte payload
	CHECK(od->getPos_f() == 1.5f);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O25] a bus voltage data frame with DLC 2 does not overwrite the voltage" * KNOWN_ISSUE("B-O3: canRxPendCallback has no DLC check (ODriveCAN.cpp:428-434), missing bytes are read as 0")) {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_VBUS, floats(24.5f, 0));
	REQUIRE(get(*od, "vbus") == 24500);
	rx(peer0, 0, CMD_VBUS, {0x00, 0x00});
	CHECK(get(*od, "vbus") == 24500);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O25] a heartbeat shorter than 8 bytes (DLC 4) does not change the reported state" * KNOWN_ISSUE("B-O3: canRxPendCallback has no DLC check (ODriveCAN.cpp:400-413), the missing state byte is read as 0")) {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_HB, hb(0x11, 0x21));
	REQUIRE(get(*od, "state") == 0x21);
	rx(peer0, 0, CMD_HB, {0x11, 0x00, 0x00, 0x00}); // CSV 0x001: the state is byte 4, outside DLC 4
	CHECK(get(*od, "state") == 0x21);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O25] an RTR heartbeat frame does not overwrite the axis error and state" * KNOWN_ISSUE("B-O3: canRxPendCallback has no DLC / RTR check (ODriveCAN.cpp:400-413), the empty payload is read as error 0 / state 0")) {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_HB, hb(0x11, 0x21));
	REQUIRE(get(*od, "errors") == 0x11);
	rx(peer0, 0, CMD_HB, {}, true);
	CHECK(get(*od, "errors") == 0x11);
	CHECK(get(*od, "state") == 0x21);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O25] a frame with a command id the driver does not use (0x03) changes no error, state, position or voltage") {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_HB, hb(0x11, 0x21));
	rx(peer0, 0, CMD_ENC, floats(1.5f, 0));
	rx(peer0, 0, CMD_VBUS, floats(24.5f, 0));
	rx(peer0, 0, 0x03, floats(9.0f, 9.0f)); // 0x03 Get Motor Error (CSV): not used by the driver
	CHECK(get(*od, "errors") == 0x11);
	CHECK(get(*od, "state") == 0x21);
	CHECK(od->getPos_f() == 1.5f);
	CHECK(get(*od, "vbus") == 24500);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O25] characterization: a frame with an unused command id of the node counts as a sign of life and keeps the node connected") {
	// NEEDS REVIEW: lastCanMessage is refreshed before the command switch (ODriveCAN.cpp:397)
	auto od = makeOdrive1();
	link(*od, peer0);
	peer0.setSilent(true); // no heartbeat, no answers
	runFor(1800);
	rx(peer0, 0, 0x03, floats(0, 0)); // inject ignores silence
	runFor(700); // body at 2500 ms: 700 ms since the frame (the heartbeat at 0 alone would be 2500 ms old)
	CHECK(get(*od, "connected") == 1);
}

// ---------------------------------------------------------------------------------------------------------------
// O26 extended ids
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(OdrvRxFixture, "[O26] an extended-id encoder estimate frame is not used as a position" * KNOWN_ISSUE("B-O5: canRxPendCallback does not check the extended id flag (ODriveCAN.cpp:390), the frame is interpreted like a standard frame")) {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(1.5f, 0));
	REQUIRE(od->getPos_f() == 1.5f);
	peer0.setFaultExtended(true); // same value node << 5 | cmd, 29 bit id
	peer0.setEncoder(7.0f, 0);
	peer0.sendEncoderEstimates();
	CHECK(od->getPos_f() == 1.5f);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O26] an extended-id bus voltage frame is not used as a voltage" * KNOWN_ISSUE("B-O5: canRxPendCallback does not check the extended id flag (ODriveCAN.cpp:390), the frame is interpreted like a standard frame")) {
	auto od = makeOdrive1();
	link(*od, peer0);
	peer0.setBus(24.5f, 0);
	peer0.sendBusVoltage();
	REQUIRE(get(*od, "vbus") == 24500);
	peer0.setFaultExtended(true);
	peer0.setBus(48.0f, 0);
	peer0.sendBusVoltage();
	CHECK(get(*od, "vbus") == 24500);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O26] an extended-id heartbeat frame does not change the axis error and state" * KNOWN_ISSUE("B-O5: canRxPendCallback does not check the extended id flag (ODriveCAN.cpp:390), the frame is interpreted like a standard frame")) {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_HB, hb(0x11, 0x21));
	REQUIRE(get(*od, "errors") == 0x11);
	peer0.setFaultExtended(true);
	heartbeat(peer0, 0x22, 0x99);
	CHECK(get(*od, "errors") == 0x11);
	CHECK(get(*od, "state") == 0x21);
}

} // TEST_SUITE

TEST_SUITE("ODrive torque and encoder") {

// ---------------------------------------------------------------------------------------------------------------
// O40 turn
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(OdrvRxFixture, "[O40] turn at full positive power sends the max torque as Set Input Torque 0x0E, DLC 4, float32 little endian") {
	auto od = makeOdrive1();
	REQUIRE(run(*od, "maxtorque", CMDtype::set, 1000) == CommandStatus::OK); // 10.00 Nm
	clearTxFrames();
	od->turn(32767);
	const auto fr = txFramesCmd(0, CMD_TORQUE);
	REQUIRE(fr.size() == 1);
	CHECK(fr[0].id == 0x00E); // node 0 << 5 | 0x0E (CSV 0x00E)
	CHECK_FALSE(fr[0].extId);
	CHECK_FALSE(fr[0].rtr);
	CHECK(fr[0].dlc == 4);
	// HD: 32767 / 32767 * 10 = 10.0f = 0x41200000 -> 00 00 20 41
	CHECK(fr[0].payload() == std::vector<uint8_t>({0x00, 0x00, 0x20, 0x41}));
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O40] turn at full negative power (-32767) sends minus the max torque") {
	auto od = makeOdrive1();
	REQUIRE(run(*od, "maxtorque", CMDtype::set, 1000) == CommandStatus::OK);
	clearTxFrames();
	od->turn(-32767);
	const auto fr = txFramesCmd(0, CMD_TORQUE);
	REQUIRE(fr.size() == 1);
	// HD: -10.0f = 0xC1200000 -> 00 00 20 C1
	CHECK(fr[0].payload() == std::vector<uint8_t>({0x00, 0x00, 0x20, 0xC1}));
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O40] turn with power 0 sends torque 0") {
	auto od = makeOdrive1();
	REQUIRE(run(*od, "maxtorque", CMDtype::set, 1000) == CommandStatus::OK);
	clearTxFrames();
	od->turn(0);
	const auto fr = txFramesCmd(0, CMD_TORQUE);
	REQUIRE(fr.size() == 1);
	CHECK(fr[0].payload() == std::vector<uint8_t>({0, 0, 0, 0}));
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O40] turn scales the power linearly with the max torque") {
	auto od = makeOdrive1();
	REQUIRE(run(*od, "maxtorque", CMDtype::set, 1000) == CommandStatus::OK);
	clearTxFrames();
	od->turn(-16384);
	const auto fr = txFramesCmd(0, CMD_TORQUE);
	REQUIRE(fr.size() == 1);
	// HD: -16384 / 32767 * 10 = -5.00015259... Nm (float rounding < 1e-6)
	CHECK(f32at(fr[0]) == doctest::Approx(-5.0001526).epsilon(1e-6));
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O40] the torque follows a changed max torque") {
	auto od = makeOdrive1();
	REQUIRE(run(*od, "maxtorque", CMDtype::set, 250) == CommandStatus::OK); // 2.50 Nm
	clearTxFrames();
	od->turn(32767);
	const auto fr = txFramesCmd(0, CMD_TORQUE);
	REQUIRE(fr.size() == 1);
	// HD: 2.5f = 0x40200000 -> 00 00 20 40
	CHECK(fr[0].payload() == std::vector<uint8_t>({0x00, 0x00, 0x20, 0x40}));
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O40] the torque frame of ODriveCAN2 uses node 1 (id 0x2E)") {
	auto od = makeOdrive2();
	REQUIRE(run(*od, "maxtorque", CMDtype::set, 1000) == CommandStatus::OK);
	clearTxFrames();
	od->turn(32767);
	const auto fr = txFramesCmd(1, CMD_TORQUE);
	REQUIRE(fr.size() == 1);
	CHECK(fr[0].id == 0x02E); // 1 << 5 | 0x0E
	CHECK(fr[0].dlc == 4);
	CHECK(fr[0].payload() == std::vector<uint8_t>({0x00, 0x00, 0x20, 0x41}));
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O40] the peer decodes the torque frame as a valid Set Input Torque with the sent value") {
	auto od = makeOdrive1();
	REQUIRE(run(*od, "maxtorque", CMDtype::set, 1000) == CommandStatus::OK);
	od->turn(32767);
	process();
	REQUIRE(peer0.torqueSeen());
	CHECK(peer0.lastTorque() == 10.0f);
	CHECK(peer0.violations().empty());
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O40] characterization: a power of -32768 is not clipped and sends slightly more than minus the max torque") {
	// HD: -32768 / 32767 * 10 = -10.000305 Nm. int16 minimum; no clipping in turn() (FW)
	auto od = makeOdrive1();
	REQUIRE(run(*od, "maxtorque", CMDtype::set, 1000) == CommandStatus::OK);
	clearTxFrames();
	od->turn(-32768);
	const auto fr = txFramesCmd(0, CMD_TORQUE);
	REQUIRE(fr.size() == 1);
	CHECK(f32at(fr[0]) == doctest::Approx(-10.000305).epsilon(1e-6));
}

// ---------------------------------------------------------------------------------------------------------------
// O41 turn while not ready
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(OdrvRxFixture, "[O41] characterization: turn sends a torque frame although the driver is not connected and not ready") {
	// NEEDS REVIEW: the motorReady() check in setTorque is commented out (ODriveCAN.cpp:272-274)
	auto od = makeOdrive1();
	REQUIRE_FALSE(od->motorReady());
	REQUIRE(get(*od, "connected") == 0); // no frame received yet (thread body has not run: connected false)
	clearTxFrames();
	od->turn(32767);
	CHECK(txFramesCmd(0, CMD_TORQUE).size() == 1);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O41] characterization: turn sends a torque frame while the ODrive is connected but idle") {
	auto od = makeOdrive1();
	link(*od, peer0);
	REQUIRE_FALSE(od->motorReady());
	clearTxFrames();
	od->turn(1000);
	CHECK(txFramesCmd(0, CMD_TORQUE).size() == 1);
}

// ---------------------------------------------------------------------------------------------------------------
// O42 getPos_f / getPos requests
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(OdrvRxFixture, "[O42] getPos_f while connected sends one RTR 0x09 frame with DLC 0") {
	auto od = makeOdrive1();
	link(*od, peer0);
	clearTxFrames();
	od->getPos_f();
	const auto fr = txFramesCmd(0, CMD_ENC);
	REQUIRE(fr.size() == 1);
	CHECK(fr[0].id == 0x009);
	CHECK_FALSE(fr[0].extId);
	CHECK(fr[0].rtr);
	CHECK(fr[0].dlc == 0);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O42] getPos_f while not connected sends no request") {
	auto od = makeOdrive1();
	REQUIRE(get(*od, "connected") == 0);
	clearTxFrames();
	od->getPos_f();
	CHECK(txFramesCmd(0, CMD_ENC).empty());
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O42] a second getPos_f call right after a request does not send another request while the reply is awaited") {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(1.0f, 0)); // reply received now: the next request starts a wait
	clearTxFrames();
	od->getPos_f();
	od->getPos_f();
	CHECK(txFramesCmd(0, CMD_ENC).size() == 1);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O42] characterization: a request is repeated when the reply is still missing more than 5 ms after the last reply") {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(1.0f, 0));
	clearTxFrames();
	od->getPos_f();
	HostPlatform::advanceMs(5);
	od->getPos_f(); // 5 ms: not more than 5
	CHECK(txFramesCmd(0, CMD_ENC).size() == 1);
	HostPlatform::advanceMs(1);
	od->getPos_f(); // 6 ms
	CHECK(txFramesCmd(0, CMD_ENC).size() == 2);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O42] without any reply repeated getPos_f calls within 5 ms send at most one RTR 0x09" * KNOWN_ISSUE("B-O9: the retry window is measured from the last reply (lastPosTime), not from the request (ODriveCAN.cpp:253), so without replies every getPos_f call sends an RTR 0x09 (request flood at the axis loop rate)")) {
	auto od = makeOdrive1();
	link(*od, peer0);
	clearTxFrames();
	od->getPos_f();
	od->getPos_f();
	od->getPos_f();
	CHECK(txFramesCmd(0, CMD_ENC).size() <= 1);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O42] the peer answers the getPos_f request with its position and getPos_f returns it") {
	auto od = makeOdrive1();
	link(*od, peer0);
	peer0.setEncoder(0.75f, 0);
	od->getPos_f(); // request
	process();		// peer answers with the same id (SPEC call and response)
	CHECK(od->getPos_f() == 0.75f);
	CHECK(peer0.rtrCount(CMD_ENC) >= 1);
	CHECK(peer0.violations().empty());
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O42] getPos_f returns the position received before the request, not the answer to the request just sent") {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(1.0f, 0));
	peer0.setEncoder(2.0f, 0);
	CHECK(od->getPos_f() == 1.0f); // request sent, not yet answered
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O42] getPos is 65535 times the position, for a position of one turn") {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(1.0f, 0));
	CHECK(od->getPos() == 65535); // HD: 1.0 * 65535, exact in float
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O42] getCpr is 65535 counts per turn") {
	auto od = makeOdrive1();
	CHECK(od->getCpr() == 0xFFFF);
}

// ---------------------------------------------------------------------------------------------------------------
// O43 setPos
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(OdrvRxFixture, "[O43] setPos(0) makes the current received position the zero position") {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(1.5f, 0));
	od->setPos(0);
	// HD: offset = 1.5 - 0 = 1.5; position = 1.5 - 1.5 = 0 (exact)
	CHECK(od->getPos_f() == 0.0f);
	CHECK(od->getPos() == 0);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O43] after setPos(0) later position changes are reported relative to the zero position") {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(1.5f, 0));
	od->setPos(0);
	rx(peer0, 0, CMD_ENC, floats(2.0f, 0));
	CHECK(od->getPos_f() == 0.5f); // HD: 2.0 - 1.5, exact
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O43] setPos(counts) makes getPos return counts for the last received position") {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(1.5f, 0));
	od->setPos(1000);
	// HD: offset = 1.5 - 1000/65535; position = 1000/65535 turns; times 65535 = 1000 up to float rounding
	// (position error < 1.2e-7 turns = 0.008 counts, then truncation: 999 or 1000)
	CHECK(od->getPos_f() == doctest::Approx(1000.0 / 65535.0).epsilon(1e-5));
	CHECK(od->getPos() >= 999);
	CHECK(od->getPos() <= 1000);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O43] setPos with a negative count sets a negative position for the last received position") {
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(0.0f, 0));
	od->setPos(-6554);
	// HD: offset = 0 + 6554/65535; position = -6554/65535 = -0.100008...
	CHECK(od->getPos_f() == doctest::Approx(-6554.0 / 65535.0).epsilon(1e-5));
}

// ---------------------------------------------------------------------------------------------------------------
// O44 range
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(OdrvRxFixture, "[O44] a position of +-16384 turns is converted to counts without overflow") {
	// HD: 16384 * 65535 = 1073725440 (16 significant bits: exact in float), within int32
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(16384.0f, 0));
	CHECK(od->getPos() == 1073725440);
	rx(peer0, 0, CMD_ENC, floats(-16384.0f, 0));
	CHECK(od->getPos() == -1073725440);
}

TEST_CASE_FIXTURE(OdrvRxFixture, "[O44] a position of +40000 turns (2.6e9 counts, beyond int32) does not become a negative count" * KNOWN_ISSUE_UB("B-O8: getPos converts float to int32 without a range check (ODriveCAN.cpp:264), out of range is undefined behaviour; x86 gives INT32_MIN, ARM saturates (differs on target)")) {
	// HD: 40000 * 65535 = 2.62e9 > 2147483647. Intended: no undefined behaviour and the sign of the position is kept
	auto od = makeOdrive1();
	link(*od, peer0);
	rx(peer0, 0, CMD_ENC, floats(40000.0f, 0));
	CHECK(od->getPos() > 0);
}

} // TEST_SUITE
