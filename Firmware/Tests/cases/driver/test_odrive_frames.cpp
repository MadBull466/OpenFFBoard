/*
 * ODrive CAN driver, groups G1 (construction, port, filter) and G2 (transmitted frames).
 * Requirements O1-O3 and O10-O16 of Firmware/Tests/driver/ODRIVE_REQUIREMENTS.md (charter, lead decisions L1-L11).
 *
 * Sources: SPEC = "CAN Protocol - ODrive Documentation 0.5.6", CSV = its "CAN Messages" table (not committed).
 *  - SPEC "CAN Frame": 11 bit standard id = node << 5 | cmd, node 0..0x3F, cmd 0..0x1F; all multibyte values little endian.
 *  - CSV 0x007 uint32 @0 (DLC 4), 0x00B two int32 @0 / @4 (DLC 8), 0x00E float32 @0 (DLC 4), 0x010 no signals,
 *    0x009 and 0x017 are requested with the RTR bit (SPEC call and response).
 * The enum numbers (axis states, control / input mode) are not in SPEC / CSV (charter L2): tests that need a particular
 * number are named "characterization:". The byte encoding is checked with numbers that do not depend on the enums
 * (static_cast of an arbitrary 32 bit pattern).
 *
 * Host-vs-target: the payload bytes are compared as bytes, so the host (little endian, 64 bit) and the target (little
 * endian ARM) agree. ODriveCAN::canRxPendCallback reads the data through a uint64_t pointer (ODC:394) - not reached here.
 */
#include "doctest.h"
#include "OdriveFixture.h"
#include "eeprom_addresses.h"
#include "flash_helpers.h"
#include "CANPort2B.h"

#include <cstdint>
#include <vector>

namespace {

using Bytes = std::vector<uint8_t>;
using HostTarget::CanTxFrame;

bool canSilentPinHigh() { return HostTarget::outputPin(CAN_S_GPIO_Port, CAN_S_Pin); }

/** Checks id, standard-id flag, rtr flag, dlc and the first `dlc` payload bytes of a transmitted frame */
void checkFrame(const CanTxFrame &f, uint32_t id, bool rtr, uint8_t dlc, const Bytes &payload) {
	CHECK(f.id == id);
	CHECK_FALSE(f.extId);
	CHECK(f.rtr == rtr);
	CHECK(f.dlc == dlc);
	CHECK(f.payload() == payload);
}

/** Node ids in flash (0x3D0): M0 bits 0-5, M1 bits 6-11 (ODC:127-134) */
void storeNodeIds(uint16_t m0, uint16_t m1) { Flash_Write(ADR_ODRIVE_CANID, uint16_t(m0 | (m1 << 6))); }

/** Sends all kinds of frames the driver can send; returns the ids in order */
std::vector<CanTxFrame> sendAllFrameKinds(ODriveCAN &od) {
	HostTarget::canClearTxFrames();
	od.setState(static_cast<ODriveState>(0x01020304));
	od.setMode(static_cast<ODriveControlMode>(1), static_cast<ODriveInputMode>(1));
	od.setTorque(1.0f);
	od.requestMsg(0x09);
	od.requestMsg(0x17);
	od.startAnticogging();
	return HostTarget::canTxFrames();
}

} // namespace

TEST_SUITE("ODrive construction and port") {

// ---------------------------------------------------------------- O1

TEST_CASE_FIXTURE(OdriveFixture, "[O1] constructing ODriveCAN1 starts the CAN port once") {
	auto od = makeOdrive1();
	CHECK(HostTarget::canPort().started);
	CHECK(HostTarget::canPort().startCalls == 1);
	CHECK(HostTarget::canPort2B().getPortUsers() == 1);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O1] constructing ODriveCAN1 takes the port out of silent mode") {
	// The transceiver silent pin is low = active (CANPort_2B::setSilentMode writes `silent`, CANPort2B.cpp:165-170)
	auto od = makeOdrive1();
	CHECK_FALSE(canSilentPinHigh());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O1] ODriveCAN1 registers one standard-id filter with id 0x000 and the node bits 0x7E0 as mask") {
	// HD: node 0 << 5 = 0x000; mask = bits 10..5 = 0x7E0 (SPEC CAN Frame, node bits of the id)
	auto od = makeOdrive1();
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x000);
	CHECK(HostTarget::canPort().filters[0].filter_mask == 0x7E0);
	CHECK_FALSE(HostTarget::canPort().filters[0].extid);
	CHECK(HostTarget::canActiveFilterBanks() == 1);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O1] ODriveCAN2 registers one standard-id filter with id 0x020 and the node bits 0x7E0 as mask") {
	// HD: node 1 << 5 = 0x020
	auto od = makeOdrive2();
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x020);
	CHECK(HostTarget::canPort().filters[0].filter_mask == 0x7E0);
	CHECK_FALSE(HostTarget::canPort().filters[0].extid);
	CHECK(HostTarget::canActiveFilterBanks() == 1);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O1] the hardware filter of ODriveCAN1 accepts every command id of node 0 and no frame of node 1") {
	auto od = makeOdrive1();
	CHECK(HostTarget::canAccepts(0x000));
	CHECK(HostTarget::canAccepts(0x001));
	CHECK(HostTarget::canAccepts(0x009));
	CHECK(HostTarget::canAccepts(0x01F));
	CHECK_FALSE(HostTarget::canAccepts(0x020));
	CHECK_FALSE(HostTarget::canAccepts(0x021));
	CHECK_FALSE(HostTarget::canAccepts(0x7E1));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O1] the hardware filter of ODriveCAN2 accepts every command id of node 1 and no frame of node 0") {
	auto od = makeOdrive2();
	CHECK(HostTarget::canAccepts(0x020));
	CHECK(HostTarget::canAccepts(0x021));
	CHECK(HostTarget::canAccepts(0x03F));
	CHECK_FALSE(HostTarget::canAccepts(0x000));
	CHECK_FALSE(HostTarget::canAccepts(0x001));
	CHECK_FALSE(HostTarget::canAccepts(0x041));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O1] the node id stored in flash selects the filter (node 5 -> id 0x0A0)") {
	storeNodeIds(5, 0);
	auto od = makeOdrive1();
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x0A0); // HD: 5 << 5
	CHECK(HostTarget::canPort().filters[0].filter_mask == 0x7E0);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O1] characterization: a port below the 250 kbit/s preset is raised to preset 3") {
	// Firmware only: the speed floor ("Minimum 250k", ODC:80) is not in the spec. Presets: 0 50k, 1 100k, 2 125k, 3 250k (cpp_target_config.cpp:51)
	HostTarget::canPort2B().setSpeedPreset(1);
	auto od = makeOdrive1();
	CHECK(HostTarget::canPort2B().getSpeedPreset() == 3);
	CHECK(HostTarget::canPort2B().getSpeed() == 250000);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O1] characterization: a port at preset 3 (250 kbit/s) stays at preset 3") {
	HostTarget::canPort2B().setSpeedPreset(3);
	auto od = makeOdrive1();
	CHECK(HostTarget::canPort2B().getSpeedPreset() == 3);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O1] characterization: a port above the 250 kbit/s preset (preset 5, 1000 kbit/s) is not lowered") {
	HostTarget::canPort2B().setSpeedPreset(5);
	auto od = makeOdrive1();
	CHECK(HostTarget::canPort2B().getSpeedPreset() == 5);
}

// ---------------------------------------------------------------- O2

TEST_CASE_FIXTURE(OdriveFixture, "[O2] ODriveCAN1 is creatable only while no instance exists") {
	CHECK(ODriveCAN1::isCreatable());
	{
		auto od = makeOdrive1();
		CHECK_FALSE(ODriveCAN1::isCreatable());
	}
	CHECK(ODriveCAN1::isCreatable());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O2] ODriveCAN2 is creatable only while no instance exists") {
	CHECK(ODriveCAN2::isCreatable());
	{
		auto od = makeOdrive2();
		CHECK_FALSE(ODriveCAN2::isCreatable());
	}
	CHECK(ODriveCAN2::isCreatable());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O2] an instance of ODriveCAN1 does not block the creation of ODriveCAN2") {
	auto od1 = makeOdrive1();
	CHECK(ODriveCAN2::isCreatable());
	auto od2 = makeOdrive2();
	CHECK_FALSE(ODriveCAN1::isCreatable());
	CHECK_FALSE(ODriveCAN2::isCreatable());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O2] characterization: destroying ODriveCAN1 sends a last Set Input Torque of 0 to node 0") {
	auto od = makeOdrive1();
	REQUIRE(HostTarget::canTxFrames().empty());
	od.reset();
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	// HD: id node 0 << 5 | 0x0E, DLC 4, float32 0.0f = 00 00 00 00 (CSV 0x00E)
	checkFrame(HostTarget::canTxFrames()[0], 0x00E, false, 4, Bytes{0, 0, 0, 0});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O2] characterization: destroying ODriveCAN2 sends a last Set Input Torque of 0 to node 1") {
	auto od = makeOdrive2();
	od.reset();
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x02E, false, 4, Bytes{0, 0, 0, 0}); // HD: 1 << 5 | 0x0E
}

TEST_CASE_FIXTURE(OdriveFixture, "[O2] characterization: destroying the only instance stops the port and puts the transceiver into silent mode") {
	auto od = makeOdrive1();
	od.reset();
	CHECK_FALSE(HostTarget::canPort().started);
	CHECK(HostTarget::canPort().stopCalls == 1);
	CHECK(HostTarget::canPort2B().getPortUsers() == 0);
	CHECK(canSilentPinHigh());
}

TEST_CASE_FIXTURE(OdriveFixture, "[O2] characterization: destroying the instance removes its hardware filter") {
	auto od = makeOdrive1();
	REQUIRE(HostTarget::canActiveFilterBanks() == 1);
	od.reset();
	CHECK(HostTarget::canActiveFilterBanks() == 0);
	CHECK_FALSE(HostTarget::canAccepts(0x001));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O2] characterization: with both instances the port stays started until the second one is destroyed") {
	auto od1 = makeOdrive1();
	auto od2 = makeOdrive2();
	CHECK(HostTarget::canPort2B().getPortUsers() == 2);
	CHECK(HostTarget::canActiveFilterBanks() == 2);
	od1.reset();
	CHECK(HostTarget::canPort().started);
	CHECK(HostTarget::canActiveFilterBanks() == 1);
	CHECK(HostTarget::canAccepts(0x021));
	CHECK_FALSE(HostTarget::canAccepts(0x001));
	od2.reset();
	CHECK_FALSE(HostTarget::canPort().started);
	CHECK(HostTarget::canActiveFilterBanks() == 0);
}

// ---------------------------------------------------------------- O3

TEST_CASE_FIXTURE(OdriveFixture, "[O3] characterization: the filter of ODriveCAN1 uses receive FIFO 0 and the filter of ODriveCAN2 receive FIFO 1") {
	{
		auto od = makeOdrive1();
		uint32_t fifo = 99;
		REQUIRE(HostTarget::canAccepts(0x001, false, false, &fifo));
		CHECK(fifo == 0);
	}
	{
		auto od = makeOdrive2();
		uint32_t fifo = 99;
		REQUIRE(HostTarget::canAccepts(0x021, false, false, &fifo));
		CHECK(fifo == 1);
	}
}

TEST_CASE_FIXTURE(OdriveFixture, "[O3] characterization: ODriveCAN2 processes a node 1 frame at once when the FIFO 1 interrupt line is enabled (fixture default, modelling deviation L8)") {
	auto od = makeOdrive2();
	peer1.setState(0x55); // arbitrary value: the heartbeat state byte is passed to the `state` command unchanged
	CHECK(peer1.sendHeartbeat() > 0);
	CHECK(CommandHelpers::get(*od, "state") == 0x55);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O3] characterization: ODriveCAN1 processes a node 0 heartbeat from FIFO 0") {
	auto od = makeOdrive1();
	peer0.setState(0x55);
	CHECK(peer0.sendHeartbeat() > 0);
	CHECK(CommandHelpers::get(*od, "state") == 0x55);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O3] characterization: with the real F407 interrupt set-up a node 1 frame waits in FIFO 1 and is processed at the next TX completion") {
	setRx1Interrupt(false); // F407: CAN1_RX1 is not enabled in the NVIC (HAL_CAN_MspInit)
	auto od = makeOdrive2();
	peer1.setState(0x55);
	peer1.sendHeartbeat();
	CHECK(HostTarget::canRxPending(1) == 1);
	CHECK(CommandHelpers::get(*od, "state") == 0); // not processed yet: AXIS_STATE_UNDEFINED
	od->turn(0); // a TX; its completion interrupt (CAN1_TX) services FIFO 1 as well
	process();
	CHECK(HostTarget::canRxPending(1) == 0);
	CHECK(CommandHelpers::get(*od, "state") == 0x55);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O3] ODriveCAN2 ignores a heartbeat of node 0") {
	auto od = makeOdrive2();
	peer1.setState(0x44);
	peer1.sendHeartbeat();
	peer0.setState(0x55);
	CHECK(peer0.sendHeartbeat() == 0); // rejected by the hardware filter of node 1
	CHECK(CommandHelpers::get(*od, "state") == 0x44);
}

}

TEST_SUITE("ODrive transmitted frames") {

// ---------------------------------------------------------------- O10

TEST_CASE_FIXTURE(OdriveFixture, "[O10] every frame of ODriveCAN1 has standard id node 0 << 5 | cmd and a clear extended-id flag") {
	auto od = makeOdrive1();
	const auto frames = sendAllFrameKinds(*od);
	REQUIRE(frames.size() == 6);
	const uint32_t cmds[6] = {0x07, 0x0B, 0x0E, 0x09, 0x17, 0x10};
	for (int i = 0; i < 6; i++) {
		CAPTURE(i);
		CHECK(frames[i].id == cmds[i]); // HD: 0 << 5 | cmd
		CHECK_FALSE(frames[i].extId);
	}
}

TEST_CASE_FIXTURE(OdriveFixture, "[O10] every frame of ODriveCAN2 has standard id node 1 << 5 | cmd and a clear extended-id flag") {
	auto od = makeOdrive2();
	const auto frames = sendAllFrameKinds(*od);
	REQUIRE(frames.size() == 6);
	const uint32_t cmds[6] = {0x07, 0x0B, 0x0E, 0x09, 0x17, 0x10};
	for (int i = 0; i < 6; i++) {
		CAPTURE(i);
		CHECK(frames[i].id == (0x20u | cmds[i])); // HD: 1 << 5 = 0x20
		CHECK_FALSE(frames[i].extId);
	}
}

TEST_CASE_FIXTURE(OdriveFixture, "[O10] with node id 5 every frame has id 0x0A0 | cmd") {
	storeNodeIds(5, 0);
	auto od = makeOdrive1();
	const auto frames = sendAllFrameKinds(*od);
	REQUIRE(frames.size() == 6);
	const uint32_t cmds[6] = {0x07, 0x0B, 0x0E, 0x09, 0x17, 0x10};
	for (int i = 0; i < 6; i++) {
		CAPTURE(i);
		CHECK(frames[i].id == (0x0A0u | cmds[i])); // HD: 5 << 5 = 0xA0
		CHECK_FALSE(frames[i].extId);
	}
}

TEST_CASE_FIXTURE(OdriveFixture, "[O10] with the highest node id 63 every frame has id 0x7E0 | cmd") {
	storeNodeIds(63, 0);
	auto od = makeOdrive1();
	const auto frames = sendAllFrameKinds(*od);
	REQUIRE(frames.size() == 6);
	const uint32_t cmds[6] = {0x07, 0x0B, 0x0E, 0x09, 0x17, 0x10};
	for (int i = 0; i < 6; i++) {
		CAPTURE(i);
		CHECK(frames[i].id == (0x7E0u | cmds[i])); // HD: 63 << 5 = 0x7E0
		CHECK_FALSE(frames[i].extId);
	}
}

TEST_CASE_FIXTURE(OdriveFixture, "[O10] ODriveCAN2 takes its node id from bits 6-11 of the flash word (node 63 -> id 0x7E0 | cmd)") {
	storeNodeIds(0, 63);
	auto od = makeOdrive2();
	const auto frames = sendAllFrameKinds(*od);
	REQUIRE(frames.size() == 6);
	CHECK(frames[2].id == 0x7EE);
	CHECK_FALSE(frames[2].extId);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O10] the node id set with the canid command is used for the transmitted ids (node 5)") {
	auto od = makeOdrive1();
	CHECK(CommandHelpers::run(*od, "canid", CMDtype::set, 5) == CommandStatus::OK);
	HostTarget::canClearTxFrames();
	od->setTorque(0.0f);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	CHECK(HostTarget::canTxFrames()[0].id == 0x0AE); // HD: 5 << 5 | 0x0E
	CHECK_FALSE(HostTarget::canTxFrames()[0].extId);
}

// ---------------------------------------------------------------- O11

TEST_CASE_FIXTURE(OdriveFixture, "[O11] Set Axis Requested State is sent with id 0x007, DLC 4 and the state as little endian uint32") {
	auto od = makeOdrive1();
	od->setState(static_cast<ODriveState>(0x01020304)); // HD: bytes 04 03 02 01 (CSV 0x007 uint32 @0)
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x007, false, 4, Bytes{0x04, 0x03, 0x02, 0x01});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O11] Set Axis Requested State of node 1 has id 0x027") {
	auto od = makeOdrive2();
	od->setState(static_cast<ODriveState>(0x01020304));
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x027, false, 4, Bytes{0x04, 0x03, 0x02, 0x01});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O11] characterization: requesting FULL_CALIBRATION_SEQUENCE sends the number 3 as 03 00 00 00") {
	auto od = makeOdrive1();
	od->setState(ODriveState::AXIS_STATE_FULL_CALIBRATION_SEQUENCE); // the number 3 is the driver's enum (L2)
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x007, false, 4, Bytes{0x03, 0x00, 0x00, 0x00});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O11] characterization: requesting CLOSED_LOOP_CONTROL sends the number 8 as 08 00 00 00") {
	auto od = makeOdrive1();
	od->setState(ODriveState::AXIS_STATE_CLOSED_LOOP_CONTROL); // the number 8 is the driver's enum (L2)
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x007, false, 4, Bytes{0x08, 0x00, 0x00, 0x00});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O11] the Set Axis Requested State frame is accepted by the peer as a well-formed request") {
	auto od = makeOdrive1();
	od->setState(static_cast<ODriveState>(0x01020304));
	process();
	CHECK(peer0.violations().empty());
	CHECK(peer0.requestedStates() == std::vector<uint32_t>({0x01020304}));
}

// ---------------------------------------------------------------- O12

TEST_CASE_FIXTURE(OdriveFixture, "[O12] Set Controller Modes is sent with id 0x00B, DLC 8, control mode int32 @0 and input mode int32 @4 little endian") {
	auto od = makeOdrive1();
	od->setMode(static_cast<ODriveControlMode>(0x11223344), static_cast<ODriveInputMode>(0x55667788));
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	// HD: control 0x11223344 -> 44 33 22 11 @0, input 0x55667788 -> 88 77 66 55 @4 (CSV 0x00B)
	checkFrame(HostTarget::canTxFrames()[0], 0x00B, false, 8, Bytes{0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O12] a negative mode value is sent as a two's complement int32 (control -2 -> FE FF FF FF)") {
	auto od = makeOdrive1();
	od->setMode(static_cast<ODriveControlMode>(0xFFFFFFFEu), static_cast<ODriveInputMode>(0));
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x00B, false, 8, Bytes{0xFE, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O12] Set Controller Modes of node 1 has id 0x02B") {
	auto od = makeOdrive2();
	od->setMode(static_cast<ODriveControlMode>(0x11223344), static_cast<ODriveInputMode>(0x55667788));
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x02B, false, 8, Bytes{0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O12] characterization: torque control with passthrough input sends 01 00 00 00 01 00 00 00") {
	auto od = makeOdrive1();
	od->setMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH); // 1 and 1: driver enums (L2)
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x00B, false, 8, Bytes{1, 0, 0, 0, 1, 0, 0, 0});
	process();
	CHECK(peer0.violations().empty());
	CHECK(peer0.modesSeen());
	CHECK(peer0.lastControlMode() == 1);
	CHECK(peer0.lastInputMode() == 1);
}

// ---------------------------------------------------------------- O13

TEST_CASE_FIXTURE(OdriveFixture, "[O13] Set Input Torque 1.0 Nm is sent with id 0x00E, DLC 4 and the bytes 00 00 80 3F") {
	auto od = makeOdrive1();
	od->setTorque(1.0f); // HD: IEEE 754 1.0f = 0x3F800000, little endian 00 00 80 3F
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x00E, false, 4, Bytes{0x00, 0x00, 0x80, 0x3F});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O13] Set Input Torque -0.5 Nm is sent as 00 00 00 BF") {
	auto od = makeOdrive1();
	od->setTorque(-0.5f); // HD: -0.5f = 0xBF000000
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x00E, false, 4, Bytes{0x00, 0x00, 0x00, 0xBF});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O13] Set Input Torque 2.5 Nm is sent as 00 00 20 40") {
	auto od = makeOdrive1();
	od->setTorque(2.5f); // HD: 2.5f = 0x40200000
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x00E, false, 4, Bytes{0x00, 0x00, 0x20, 0x40});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O13] Set Input Torque 0 Nm is sent as four zero bytes") {
	auto od = makeOdrive1();
	od->setTorque(0.0f);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x00E, false, 4, Bytes{0, 0, 0, 0});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O13] Set Input Torque of node 1 has id 0x02E") {
	auto od = makeOdrive2();
	od->setTorque(1.0f);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x02E, false, 4, Bytes{0x00, 0x00, 0x80, 0x3F});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O13] the torque frame is decoded by the peer as the same float value") {
	auto od = makeOdrive1();
	od->setTorque(-0.5f);
	process();
	CHECK(peer0.violations().empty());
	CHECK(peer0.torqueSeen());
	CHECK(peer0.lastTorque() == -0.5f);
}

// ---------------------------------------------------------------- O14

TEST_CASE_FIXTURE(OdriveFixture, "[O14] the encoder estimates request is a remote frame with id 0x009 and DLC 0") {
	auto od = makeOdrive1();
	od->requestMsg(0x09);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x009, true, 0, Bytes{});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O14] the bus voltage request is a remote frame with id 0x017 and DLC 0") {
	auto od = makeOdrive1();
	od->requestMsg(0x17);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x017, true, 0, Bytes{});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O14] the requests of node 1 are remote frames with ids 0x029 and 0x037") {
	auto od = makeOdrive2();
	od->requestMsg(0x09);
	od->requestMsg(0x17);
	REQUIRE(HostTarget::canTxFrames().size() == 2);
	checkFrame(HostTarget::canTxFrames()[0], 0x029, true, 0, Bytes{}); // HD: 1 << 5 | 0x09
	checkFrame(HostTarget::canTxFrames()[1], 0x037, true, 0, Bytes{}); // HD: 1 << 5 | 0x17
}

TEST_CASE_FIXTURE(OdriveFixture, "[O14] the peer accepts both requests and answers them with the same id") {
	auto od = makeOdrive1();
	peer0.setEncoder(1.5f, 0.0f);
	peer0.setBus(24.0f, 0.0f);
	od->requestMsg(0x09);
	od->requestMsg(0x17);
	process();
	CHECK(peer0.violations().empty());
	CHECK(peer0.rtrCount(0x09) == 1);
	CHECK(peer0.rtrCount(0x17) == 1);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O14] the periodic bus voltage request of the driver thread is a remote frame with id 0x037 and DLC 0 (node 1)") {
	auto od = makeOdrive2();
	peer1.setSilent(true); // no reply, no heartbeat: the voltage request repeats
	runFor(1500);
	const auto frames = txFramesCmd(1, 0x17);
	REQUIRE(!frames.empty());
	checkFrame(frames[0], 0x037, true, 0, Bytes{});
	CHECK(peer1.violations().empty());
}

// ---------------------------------------------------------------- O15

TEST_CASE_FIXTURE(OdriveFixture, "[O15] characterization: Start Anticogging is sent with id 0x010 and DLC 1 carrying one zero byte (spec: no signals)") {
	// CSV 0x010 has no signals; the driver sends sendMsg(0x10, (uint8_t)0) = DLC 1 (ODC:303). Harmless difference, lead decision L3.
	auto od = makeOdrive1();
	od->startAnticogging();
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	checkFrame(HostTarget::canTxFrames()[0], 0x010, false, 1, Bytes{0x00});
}

TEST_CASE_FIXTURE(OdriveFixture, "[O15] characterization: the peer records the Start Anticogging frame as undocumented, not as a violation") {
	auto od = makeOdrive1();
	od->startAnticogging();
	process();
	CHECK(peer0.violations().empty());
	CHECK(peer0.anticoggingCount() == 1);
	REQUIRE(peer0.undocumented().size() == 1);
	CHECK(peer0.undocumented()[0].cmd == 0x10);
	CHECK(peer0.undocumented()[0].dlc == 1);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O15] characterization: the anticogging command odrv.anticogging=1 sends the Start Anticogging frame once") {
	auto od = makeOdrive1();
	CHECK(CommandHelpers::run(*od, "anticogging", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	CHECK(HostTarget::canTxFrames()[0].id == 0x010);
}

// ---------------------------------------------------------------- O16

TEST_CASE_FIXTURE(OdriveFixture, "[O16] the highest valid node id 63 set by command is used by the filter and by the transmitted ids") {
	auto od = makeOdrive1();
	CHECK(CommandHelpers::run(*od, "canid", CMDtype::set, 63) == CommandStatus::OK);
	REQUIRE(HostTarget::canPort().filters.size() == 2); // the rebuilt filter is the second addCanFilter call
	CHECK(HostTarget::canPort().filters[1].filter_id == 0x7E0); // HD: 63 << 5
	CHECK(HostTarget::canPort().filters[1].filter_mask == 0x7E0);
	CHECK(CommandHelpers::get(*od, "canid") == 63);
	HostTarget::canClearTxFrames();
	od->setTorque(0.0f);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	CHECK(HostTarget::canTxFrames()[0].id == 0x7EE); // HD: 63 << 5 | 0x0E
}

TEST_CASE_FIXTURE(OdriveFixture, "[O16] after canid set the filter of the old node is removed and the hardware accepts the new node only") {
	auto od = makeOdrive1();
	CHECK(CommandHelpers::run(*od, "canid", CMDtype::set, 5) == CommandStatus::OK);
	CHECK(HostTarget::canActiveFilterBanks() == 1);
	CHECK(HostTarget::canAccepts(0x0A1));
	CHECK_FALSE(HostTarget::canAccepts(0x001));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O16] canid set on ODriveCAN2 changes only the TX id and the filter of that instance (node 1 -> 9)") {
	auto od1 = makeOdrive1();
	auto od2 = makeOdrive2();
	CHECK(CommandHelpers::run(*od2, "canid", CMDtype::set, 9) == CommandStatus::OK);
	// filters passed to addCanFilter: od1, od2, od2 rebuilt
	REQUIRE(HostTarget::canPort().filters.size() == 3);
	CHECK(HostTarget::canPort().filters[2].filter_id == 0x120); // HD: 9 << 5
	CHECK(HostTarget::canPort().filters[2].filter_mask == 0x7E0);
	CHECK(HostTarget::canActiveFilterBanks() == 2);
	CHECK(HostTarget::canAccepts(0x121));
	CHECK_FALSE(HostTarget::canAccepts(0x021));
	CHECK(HostTarget::canAccepts(0x001)); // ODriveCAN1 still on node 0
	HostTarget::canClearTxFrames();
	od2->setTorque(0.0f);
	od1->setTorque(0.0f);
	REQUIRE(HostTarget::canTxFrames().size() == 2);
	CHECK(HostTarget::canTxFrames()[0].id == 0x12E); // HD: 9 << 5 | 0x0E
	CHECK(HostTarget::canTxFrames()[1].id == 0x00E);
}

// B-O4 scope: the stored and reported node id (int8, unvalidated) differs from the node on the bus. The CAN identifier
// register keeps 11 bits, so TX id and RX filter (& 0x3F) agree only through that truncation (e.g. 100 -> node 36), and
// the saved value is & 0x3F, so a reload changes the node. The tests below describe the intended behaviour: rejection.
TEST_CASE_FIXTURE(OdriveFixture, "[O16] a node id above 63 is rejected" * KNOWN_ISSUE("B-O4: canid is an unvalidated int8; set 64 is stored and read back as 64 although the protocol allows 6 bits (SPEC CAN Frame, max 0x3F)")) {
	auto od = makeOdrive1();
	CommandHelpers::run(*od, "canid", CMDtype::set, 64);
	CHECK(CommandHelpers::get(*od, "canid") <= 63); // defect: reads back 64 (rejected = unchanged 0 or an error status)
}

TEST_CASE_FIXTURE(OdriveFixture, "[O16] a negative node id is rejected" * KNOWN_ISSUE("B-O4: canid is an unvalidated int8; set -1 is stored and read back as -1")) {
	auto od = makeOdrive1();
	CommandHelpers::run(*od, "canid", CMDtype::set, -1);
	CHECK(CommandHelpers::get(*od, "canid") >= 0); // defect: reads back -1
}

TEST_CASE_FIXTURE(OdriveFixture, "[O16] a node id of 200 is rejected" * KNOWN_ISSUE("B-O4: canid set 200 narrows to int8 -56 and is accepted")) {
	auto od = makeOdrive1();
	CommandHelpers::run(*od, "canid", CMDtype::set, 200);
	const int64_t id = CommandHelpers::get(*od, "canid");
	CHECK(id >= 0);
	CHECK(id <= 63);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O16] a rejected node id leaves the node unchanged" * KNOWN_ISSUE("B-O4: canid set 64 changes the stored node id to 64 instead of leaving node 3 unchanged")) {
	auto od = makeOdrive1();
	CHECK(CommandHelpers::run(*od, "canid", CMDtype::set, 3) == CommandStatus::OK);
	CommandHelpers::run(*od, "canid", CMDtype::set, 64);
	CHECK(CommandHelpers::get(*od, "canid") == 3); // defect: reads back 64
}

}
