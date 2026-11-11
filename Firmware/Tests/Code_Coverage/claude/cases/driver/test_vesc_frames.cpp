/*
 * VESC CAN driver, groups G1 (construction, port, filters) and G2 (transmitted frames).
 * Requirements V1-V3 and V10-V15 of Tests/Code_Coverage/claude/charters/driver/VESC_REQUIREMENTS.md (charter, lead decisions L1-L7).
 *
 * Sources: SPEC = "VESC 6 CAN Formats v0.1" slides (extended id = unit id | packet << 8, SET_CURRENT_REL),
 *          SRC  = VESC firmware 7.01 (Tests/Code_Coverage/shared/specs/bldc-master, not committed): comm_can.c:443-505 (request framing),
 *                 comm_can.c:1814-1820 (SET_CURRENT_REL), comm_can.c:2023-2030 (POLL_ROTOR_POS), util/buffer.c (big endian,
 *                 truncating float scaling), commands.c:384-395 (GET_VALUES_SELECTIVE request).
 *
 * Filters (VescCAN.cpp:94-121): OFFB filter id = OFFB_can_Id, mask 0xFF; VESC filter id = 56 << 8 = 0x3800, mask 0xFF00 when
 * the VESC id is 255, else id = VESC id, mask 0xFF. Both extended id, FIFO 0. A mask filter accepts a frame when
 * (id & mask) == (filter id & mask).
 *
 * Host-vs-target: payload bytes are compared as bytes; the ratio is a float on both host and target (single precision,
 * (int32)(ratio * 1e5f)); the values used below are far from a rounding boundary of a float multiplication (HD per test).
 */
#include "doctest.h"
#include "VescFixture.h"
#include "eeprom_addresses.h"
#include "flash_helpers.h"
#include "CANPort2B.h"

#include <cstdint>
#include <vector>

namespace {

using Bytes = std::vector<uint8_t>;
using HostTarget::CanTxFrame;

/** CANID word of a VESC instance in flash: VESC id << 8 | OFFB id (VescCAN.cpp:166-169, 196) */
void storeIds(uint16_t adr, uint8_t vescId, uint8_t offbId) { Flash_Write(adr, uint16_t((vescId << 8) | offbId)); }

/** Checks id, extended flag, rtr flag, dlc and the first `dlc` payload bytes of a transmitted frame */
void checkFrame(const CanTxFrame &f, uint32_t id, uint8_t dlc, const Bytes &payload) {
	CHECK(f.id == id);
	CHECK(f.extId);
	CHECK_FALSE(f.rtr);
	CHECK(f.dlc == dlc);
	CHECK(f.payload() == payload);
}

struct Fx : VescFixture {
	Ptr<VESC_1> drv;

	/** Brings `v` to READY against peer0 (unit id as configured), starts the motor and forgets the frames so far */
	void readyAndStart(VescCAN &v) {
		reachReady(v);
		v.startMotor();
		clearTxFrames();
	}

	/** The SET_CURRENT_REL frames (to VESC id 255) that turn(power) causes on a READY started VESC_1 */
	std::vector<CanTxFrame> torqueFrames(int16_t power) {
		drv = makeVesc1(); // stays alive: its destructor would send a last ratio 0
		readyAndStart(*drv);
		drv->turn(power);
		return txFramesPacket(0xFF, FakeVescPeer::PKT_SET_CURRENT_REL);
	}

	/** V10 for one VESC id: every kind of frame the driver sends has the id vid | packet << 8 */
	void checkIdLayout(uint8_t vid) {
		storeIds(ADR_VESC1_CANID, vid, 0x40);
		if (vid != 0xFF) {
			peer0.setUnitId(vid); // 255 stays a broadcast, answered by the default unit 1
		}
		auto v = makeVesc1();
		reachReady(*v); // FW version request, values requests
		v->startMotor();
		v->turn(16384); // no time passes before: the thread cannot repeat the (uninitialised, B-V13) lastTorque
		// POLL_ROTOR_POS (packet 56) is checked in the V14 tests: its sender runs into B-V3 (null memcpy, UBSan)

		const auto &all = HostTarget::canTxFrames();
		CHECK(all.size() >= 3); // FW request, values request(s), SET_CURRENT_REL
		for (const auto &f : all) {
			INFO("frame id " << f.id);
			CHECK(f.extId);
			CHECK_FALSE(f.rtr);
			CHECK((f.id >> 16) == 0);					// bits 16-28 zero
			CHECK((f.id & 0xFF) == vid);				// unit id in bits 0-7
			const uint32_t pkt = (f.id >> 8) & 0xFF;
			CHECK((pkt == 8 || pkt == 10)); // only the packets the driver uses here
		}
		CHECK(txFrames(uint32_t(vid) | (8u << 8)).size() >= 1);	 // PROCESS_SHORT_BUFFER (values request)
		CHECK(txFrames(uint32_t(vid) | (10u << 8)).size() == 1);  // SET_CURRENT_REL
		}
};

} // namespace

TEST_SUITE("VESC construction and filters") {

// ---------------------------------------------------------------- V1

TEST_CASE_FIXTURE(Fx, "[V1] constructing VESC_1 starts the CAN port once and takes one port user") {
	auto v = makeVesc1();
	CHECK(HostTarget::canPort().started);
	CHECK(HostTarget::canPort().startCalls == 1);
	CHECK(HostTarget::canPort2B().getPortUsers() == 1);
}

TEST_CASE_FIXTURE(Fx, "[V1] VESC_1 registers two extended-id filters, in order: OFFB id 0x40 with mask 0xFF, then POLL_ROTOR_POS 0x3800 with mask 0xFF00") {
	// HD: default OFFB id 0x40; default VESC id 255 -> packet 56 << 8 = 0x3800, mask bits 8-15 = 0xFF00
	auto v = makeVesc1();
	REQUIRE(HostTarget::canPort().filters.size() == 2);
	CHECK(HostTarget::canPort().filters[0].extid);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x40);
	CHECK(HostTarget::canPort().filters[0].filter_mask == 0xFF);
	CHECK(HostTarget::canPort().filters[1].extid);
	CHECK(HostTarget::canPort().filters[1].filter_id == 0x3800);
	CHECK(HostTarget::canPort().filters[1].filter_mask == 0xFF00);
	CHECK(HostTarget::canActiveFilterBanks() == 2);
}

TEST_CASE_FIXTURE(Fx, "[V1] with the default ids the hardware accepts every packet addressed to 0x40 and a POLL_ROTOR_POS frame of any unit") {
	auto v = makeVesc1();
	CHECK(HostTarget::canAccepts(0x0840, true)); // PROCESS_SHORT_BUFFER reply to the driver
	CHECK(HostTarget::canAccepts(0x0540, true)); // FILL_RX_BUFFER to the driver
	CHECK(HostTarget::canAccepts(0x0740, true)); // PROCESS_RX_BUFFER to the driver
	CHECK(HostTarget::canAccepts(0x1240, true)); // PONG to the driver
	CHECK(HostTarget::canAccepts(0x3801, true)); // POLL_ROTOR_POS reply of unit 1
	CHECK(HostTarget::canAccepts(0x38FE, true)); // POLL_ROTOR_POS reply of unit 254
}

TEST_CASE_FIXTURE(Fx, "[V1] with the default ids the hardware rejects other packets of other units, standard-id frames and a wrong OFFB id") {
	auto v = makeVesc1();
	CHECK_FALSE(HostTarget::canAccepts(0x0801, true)); // PROCESS_SHORT_BUFFER to another node
	CHECK_FALSE(HostTarget::canAccepts(0x0841, true)); // one above the OFFB id
	CHECK_FALSE(HostTarget::canAccepts(0x083F, true)); // one below the OFFB id
	CHECK_FALSE(HostTarget::canAccepts(0x3701, true)); // packet 55
	CHECK_FALSE(HostTarget::canAccepts(0x3901, true)); // packet 57
	CHECK_FALSE(HostTarget::canAccepts(0x040, false)); // standard id with the OFFB id (extended filter)
	CHECK_FALSE(HostTarget::canAccepts(0x3801, false)); // standard id, even if the low bits match
}

TEST_CASE_FIXTURE(Fx, "[V1] frames sent through canReceive: a POLL_ROTOR_POS frame passes the filters, a frame to another node does not") {
	auto v = makeVesc1();
	const std::vector<uint8_t> zeroAngle = {0, 0, 0, 0};
	CHECK(HostTarget::canReceive(0x3801, zeroAngle, true) > 0);
	CHECK(HostTarget::canReceive(0x3701, zeroAngle, true) == 0);
	CHECK(HostTarget::canReceive(0x0801, {0x01, 0x01, 50}, true) == 0);
}

TEST_CASE_FIXTURE(Fx, "[V1] a VESC id other than 255 in flash replaces the POLL filter by id = VESC id with mask 0xFF") {
	storeIds(ADR_VESC1_CANID, 0x05, 0x40);
	auto v = makeVesc1();
	REQUIRE(HostTarget::canPort().filters.size() == 2);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x40);
	CHECK(HostTarget::canPort().filters[0].filter_mask == 0xFF);
	CHECK(HostTarget::canPort().filters[1].extid);
	CHECK(HostTarget::canPort().filters[1].filter_id == 0x05);
	CHECK(HostTarget::canPort().filters[1].filter_mask == 0xFF);
}

TEST_CASE_FIXTURE(Fx, "[V1] with VESC id 5 the hardware accepts every packet of unit 5 and nothing of other units except frames to the OFFB id") {
	storeIds(ADR_VESC1_CANID, 0x05, 0x40);
	auto v = makeVesc1();
	CHECK(HostTarget::canAccepts(0x3805, true)); // POLL_ROTOR_POS of unit 5
	CHECK(HostTarget::canAccepts(0x0805, true)); // any packet of unit 5
	CHECK(HostTarget::canAccepts(0x0840, true)); // reply to the OFFB id
	CHECK_FALSE(HostTarget::canAccepts(0x3806, true)); // POLL_ROTOR_POS of unit 6
	CHECK_FALSE(HostTarget::canAccepts(0x3801, true));
	CHECK_FALSE(HostTarget::canAccepts(0x0804, true));
}

TEST_CASE_FIXTURE(Fx, "[V1] the OFFB id in flash selects the first filter (0x41) and the old id is not accepted") {
	storeIds(ADR_VESC1_CANID, 0xFF, 0x41);
	auto v = makeVesc1();
	REQUIRE(HostTarget::canPort().filters.size() == 2);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x41);
	CHECK(HostTarget::canPort().filters[0].filter_mask == 0xFF);
	CHECK(HostTarget::canAccepts(0x0841, true));
	CHECK_FALSE(HostTarget::canAccepts(0x0840, true));
}

TEST_CASE_FIXTURE(Fx, "[V1] VESC_2 takes its ids from its own flash words (0x3E3) and registers its two filters") {
	storeIds(ADR_VESC1_CANID, 0x09, 0x30); // must not be used by VESC_2
	storeIds(ADR_VESC2_CANID, 0x07, 0x42);
	auto v = makeVesc2();
	REQUIRE(HostTarget::canPort().filters.size() == 2);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x42);
	CHECK(HostTarget::canPort().filters[1].filter_id == 0x07);
	CHECK(HostTarget::canPort().filters[1].filter_mask == 0xFF);
	CHECK(HostTarget::canAccepts(0x0842, true));
	CHECK(HostTarget::canAccepts(0x0807, true));
	CHECK_FALSE(HostTarget::canAccepts(0x0830, true));
	CHECK_FALSE(HostTarget::canAccepts(0x0809, true));
}

// ---------------------------------------------------------------- V2

TEST_CASE_FIXTURE(Fx, "[V2] changing offbcanid to 0x41 makes the filter accept 0x41 and reject the old id 0x40") {
	auto v = makeVesc1();
	CHECK(CommandHelpers::run(*v, "offbcanid", CMDtype::set, 0x41) == CommandStatus::OK);
	CHECK(HostTarget::canAccepts(0x0841, true));
	CHECK_FALSE(HostTarget::canAccepts(0x0840, true));
}

TEST_CASE_FIXTURE(Fx, "[V2] changing offbcanid reads back the new id and keeps the POLL_ROTOR_POS filter") {
	auto v = makeVesc1();
	CommandHelpers::run(*v, "offbcanid", CMDtype::set, 0x41);
	CHECK(CommandHelpers::get(*v, "offbcanid") == 0x41);
	CHECK(HostTarget::canAccepts(0x3801, true));
}

TEST_CASE_FIXTURE(Fx, "[V2] changing vesccanid to 5 makes the filter accept the frames of unit 5 and keeps the OFFB id") {
	auto v = makeVesc1();
	CHECK(CommandHelpers::run(*v, "vesccanid", CMDtype::set, 5) == CommandStatus::OK);
	CHECK(CommandHelpers::get(*v, "vesccanid") == 5);
	CHECK(HostTarget::canAccepts(0x0805, true));
	CHECK(HostTarget::canAccepts(0x3805, true));
	CHECK(HostTarget::canAccepts(0x0840, true));
}

TEST_CASE_FIXTURE(Fx, "[V2] changing vesccanid from 5 to 6 rejects the frames of unit 5" * KNOWN_ISSUE("B-V1: the VESC filter id is set to -1 before removeCanFilter (VescCAN.cpp:107-110), so the old filter bank stays active and still accepts unit 5")) {
	storeIds(ADR_VESC1_CANID, 0x05, 0x40);
	auto v = makeVesc1();
	CommandHelpers::run(*v, "vesccanid", CMDtype::set, 6);
	CHECK_FALSE(HostTarget::canAccepts(0x0805, true)); // the defect: old id 5 is still accepted
	CHECK(HostTarget::canAccepts(0x0806, true));
}

TEST_CASE_FIXTURE(Fx, "[V2] changing vesccanid from 255 to 5 stops accepting POLL_ROTOR_POS frames of other units" * KNOWN_ISSUE("B-V1: the old POLL_ROTOR_POS filter (id 0x3800, mask 0xFF00) is not removed and still accepts unit 1")) {
	auto v = makeVesc1();
	CommandHelpers::run(*v, "vesccanid", CMDtype::set, 5);
	CHECK_FALSE(HostTarget::canAccepts(0x3801, true)); // the defect
}

TEST_CASE_FIXTURE(Fx, "[V2] changing vesccanid leaves exactly two filter banks active" * KNOWN_ISSUE("B-V1: the old VESC filter bank leaks, three banks are active")) {
	auto v = makeVesc1();
	CommandHelpers::run(*v, "vesccanid", CMDtype::set, 5);
	CHECK(HostTarget::canActiveFilterBanks() == 2); // the defect: 3
}

TEST_CASE_FIXTURE(Fx, "[V2] changing offbcanid leaves exactly two filter banks active" * KNOWN_ISSUE("B-V1: the old VESC filter bank leaks (the set-up replaces both filters), three banks are active")) {
	auto v = makeVesc1();
	CommandHelpers::run(*v, "offbcanid", CMDtype::set, 0x41);
	CHECK(HostTarget::canActiveFilterBanks() == 2); // the defect: 3
}

TEST_CASE_FIXTURE(Fx, "[V2] repeated changes of vesccanid never use more than two filter banks" * KNOWN_ISSUE("B-V1: every change leaks the old VESC filter bank")) {
	auto v = makeVesc1();
	uint32_t maxBanks = 0;
	for (int id = 1; id <= 5; id++) {
		CommandHelpers::run(*v, "vesccanid", CMDtype::set, id);
		maxBanks = std::max(maxBanks, HostTarget::canActiveFilterBanks());
	}
	CHECK(maxBanks <= 2); // the defect: 6
}

// ---------------------------------------------------------------- V3

TEST_CASE_FIXTURE(Fx, "[V3] characterization: VESC_1 is creatable only while no instance exists") {
	CHECK(VESC_1::isCreatable());
	{
		auto v = makeVesc1();
		CHECK_FALSE(VESC_1::isCreatable());
	}
	CHECK(VESC_1::isCreatable());
}

TEST_CASE_FIXTURE(Fx, "[V3] characterization: VESC_2 is creatable only while no instance exists") {
	CHECK(VESC_2::isCreatable());
	{
		auto v = makeVesc2();
		CHECK_FALSE(VESC_2::isCreatable());
	}
	CHECK(VESC_2::isCreatable());
}

TEST_CASE_FIXTURE(Fx, "[V3] characterization: an instance of VESC_1 does not block the creation of VESC_2") {
	auto v1 = makeVesc1();
	CHECK(VESC_2::isCreatable());
	auto v2 = makeVesc2();
	CHECK_FALSE(VESC_1::isCreatable());
	CHECK_FALSE(VESC_2::isCreatable());
}

TEST_CASE_FIXTURE(Fx, "[V3] characterization: destroying the only instance stops the port and frees its two filter banks") {
	auto v = makeVesc1();
	REQUIRE(HostTarget::canActiveFilterBanks() == 2);
	v.reset();
	CHECK_FALSE(HostTarget::canPort().started);
	CHECK(HostTarget::canPort().stopCalls == 1);
	CHECK(HostTarget::canPort2B().getPortUsers() == 0);
	CHECK(HostTarget::canActiveFilterBanks() == 0);
	CHECK_FALSE(HostTarget::canAccepts(0x0840, true));
	CHECK_FALSE(HostTarget::canAccepts(0x3801, true));
}

TEST_CASE_FIXTURE(Fx, "[V3] characterization: with both instances the port stays started until the second one is destroyed") {
	peer0.setPresent(false);
	auto v1 = makeVesc1();
	auto v2 = makeVesc2();
	CHECK(HostTarget::canPort2B().getPortUsers() == 2);
	CHECK(HostTarget::canActiveFilterBanks() == 4);
	v1.reset();
	CHECK(HostTarget::canPort().started);
	CHECK(HostTarget::canActiveFilterBanks() == 2);
	v2.reset();
	CHECK_FALSE(HostTarget::canPort().started);
	CHECK(HostTarget::canActiveFilterBanks() == 0);
}

TEST_CASE_FIXTURE(Fx, "[V3] characterization: destroying a driver that is not READY sends no frame") {
	auto v = makeVesc1();
	v.reset();
	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(Fx, "[V3] characterization: destroying a READY started driver sends one last SET_CURRENT_REL with ratio 0 and nothing afterwards") {
	auto v = makeVesc1();
	readyAndStart(*v);
	v->turn(16384);
	clearTxFrames();
	v.reset();
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	// HD: stopMotor -> setTorque(0.0) -> (int32)(0 * 1e5) = 0 -> 00 00 00 00; id 255 | 10 << 8 = 0x0AFF
	checkFrame(HostTarget::canTxFrames()[0], 0x0AFF, 4, Bytes{0, 0, 0, 0});
	runFor(3000);
	CHECK(HostTarget::canTxFrames().size() == 1);
}

TEST_CASE_FIXTURE(Fx, "[V3] characterization: a port at preset 1 is raised to preset 3 (250 kbit/s)") {
	// Presets: 0 50k, 1 100k, 2 125k, 3 250k (cpp_target_config.cpp:51); VescCAN.cpp:66-68 "Minimum 250k"
	HostTarget::canPort2B().setSpeedPreset(1);
	auto v = makeVesc1();
	CHECK(HostTarget::canPort2B().getSpeedPreset() == 3);
	CHECK(HostTarget::canPort2B().getSpeed() == 250000);
}

TEST_CASE_FIXTURE(Fx, "[V3] characterization: a port at preset 2 is raised to preset 3") {
	HostTarget::canPort2B().setSpeedPreset(2);
	auto v = makeVesc1();
	CHECK(HostTarget::canPort2B().getSpeedPreset() == 3);
}

TEST_CASE_FIXTURE(Fx, "[V3] characterization: a port at preset 3 stays at preset 3") {
	HostTarget::canPort2B().setSpeedPreset(3);
	auto v = makeVesc1();
	CHECK(HostTarget::canPort2B().getSpeedPreset() == 3);
}

TEST_CASE_FIXTURE(Fx, "[V3] characterization: a port above preset 3 (preset 5, 1000 kbit/s) is not lowered") {
	HostTarget::canPort2B().setSpeedPreset(5);
	auto v = makeVesc1();
	CHECK(HostTarget::canPort2B().getSpeedPreset() == 5);
}

}

TEST_SUITE("VESC transmitted frames") {

// ---------------------------------------------------------------- V10

TEST_CASE_FIXTURE(Fx, "[V10] the FW version, values and SET_CURRENT_REL frames to VESC id 0x01 are extended data frames with id 0x01 | packet << 8 and bits 16-28 zero") {
	checkIdLayout(0x01);
}

TEST_CASE_FIXTURE(Fx, "[V10] the FW version, values and SET_CURRENT_REL frames to VESC id 0x7F are extended data frames with id 0x7F | packet << 8 and bits 16-28 zero") {
	checkIdLayout(0x7F);
}

TEST_CASE_FIXTURE(Fx, "[V10] the FW version, values and SET_CURRENT_REL frames to VESC id 0xFF are extended data frames with id 0xFF | packet << 8 and bits 16-28 zero") {
	checkIdLayout(0xFF);
}

TEST_CASE_FIXTURE(Fx, "[V10] a VESC id above 0x7F is not truncated to 7 bits (0x80 | 8 << 8 = 0x0880)") {
	storeIds(ADR_VESC1_CANID, 0x80, 0x40);
	peer0.setUnitId(0x80);
	auto v = makeVesc1();
	runFor(500);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	CHECK(HostTarget::canTxFrames()[0].id == 0x0880);
	CHECK(HostTarget::canTxFrames()[0].extId);
}

// ---------------------------------------------------------------- V11

TEST_CASE_FIXTURE(Fx, "[V11] SET_CURRENT_REL for power 16384 is id 0x0AFF, DLC 4, 00 00 C3 51") {
	// HD: 16384 / 32767 = 0.5000152..; * 1e5 = 50001.52 -> truncated 50001 = 0x0000C351 big endian
	const auto frames = torqueFrames(16384);
	REQUIRE(frames.size() == 1);
	checkFrame(frames[0], 0x0AFF, 4, Bytes{0x00, 0x00, 0xC3, 0x51});
	process();
	CHECK(peer0.lastCurrentRaw() == 50001);
}

TEST_CASE_FIXTURE(Fx, "[V11] SET_CURRENT_REL for power -32767 is DLC 4, FF FE 79 60") {
	// HD: -32767 / 32767 = -1.0; * 1e5 = -100000 = 0xFFFE7960 (2^32 - 100000 = 4294867296 = 0xFFFE7960)
	const auto frames = torqueFrames(-32767);
	REQUIRE(frames.size() == 1);
	checkFrame(frames[0], 0x0AFF, 4, Bytes{0xFF, 0xFE, 0x79, 0x60});
	process();
	CHECK(peer0.lastCurrentRaw() == -100000);
}

TEST_CASE_FIXTURE(Fx, "[V11] SET_CURRENT_REL for power 32767 is 00 01 86 A0 (ratio 1.0, 100000)") {
	// HD: 32767 / 32767 = 1.0 * 1e5 = 100000 = 0x000186A0
	const auto frames = torqueFrames(32767);
	REQUIRE(frames.size() == 1);
	checkFrame(frames[0], 0x0AFF, 4, Bytes{0x00, 0x01, 0x86, 0xA0});
}

TEST_CASE_FIXTURE(Fx, "[V11] SET_CURRENT_REL for power 0 is 00 00 00 00") {
	const auto frames = torqueFrames(0);
	REQUIRE(frames.size() == 1);
	checkFrame(frames[0], 0x0AFF, 4, Bytes{0, 0, 0, 0});
}

TEST_CASE_FIXTURE(Fx, "[V11] SET_CURRENT_REL truncates a positive ratio towards zero (power 1: 3.05 -> 3)") {
	// HD: 1 / 32767 = 3.0518e-5; * 1e5 = 3.0518 -> (int32) 3 -> 00 00 00 03
	const auto frames = torqueFrames(1);
	REQUIRE(frames.size() == 1);
	checkFrame(frames[0], 0x0AFF, 4, Bytes{0x00, 0x00, 0x00, 0x03});
}

TEST_CASE_FIXTURE(Fx, "[V11] SET_CURRENT_REL truncates a negative ratio towards zero (power -1: -3.05 -> -3)") {
	// HD: -3.0518 -> (int32) -3 = 0xFFFFFFFD (truncation towards zero, not floor which would give -4)
	const auto frames = torqueFrames(-1);
	REQUIRE(frames.size() == 1);
	checkFrame(frames[0], 0x0AFF, 4, Bytes{0xFF, 0xFF, 0xFF, 0xFD});
}

TEST_CASE_FIXTURE(Fx, "[V11] SET_CURRENT_REL goes to the configured VESC id (0x05 -> 0x0A05)") {
	storeIds(ADR_VESC1_CANID, 0x05, 0x40);
	peer0.setUnitId(0x05);
	auto v = makeVesc1();
	readyAndStart(*v);
	v->turn(16384);
	const auto frames = txFramesPacket(0x05, FakeVescPeer::PKT_SET_CURRENT_REL);
	REQUIRE(frames.size() == 1);
	checkFrame(frames[0], 0x0A05, 4, Bytes{0x00, 0x00, 0xC3, 0x51});
}

// ---------------------------------------------------------------- V12

TEST_CASE_FIXTURE(Fx, "[V12] the FW version request is PROCESS_SHORT_BUFFER 0x08FF, DLC 3, [0x40, 0x00, 0x00]") {
	auto v = makeVesc1();
	runFor(500);
	const auto frames = txFramesPacket(0xFF, FakeVescPeer::PKT_PROCESS_SHORT_BUFFER);
	REQUIRE(frames.size() == 1);
	checkFrame(frames[0], 0x08FF, 3, Bytes{0x40, 0x00, 0x00});
	process();
	CHECK(peer0.requestCount(FakeVescPeer::COMM_FW_VERSION) == 1);
}

TEST_CASE_FIXTURE(Fx, "[V12] the FW version request carries the configured OFFB id (0x41) as sender and goes to the configured VESC id (0x05)") {
	storeIds(ADR_VESC1_CANID, 0x05, 0x41);
	peer0.setUnitId(0x05);
	auto v = makeVesc1();
	runFor(500);
	const auto frames = txFramesPacket(0x05, FakeVescPeer::PKT_PROCESS_SHORT_BUFFER);
	REQUIRE(frames.size() == 1);
	checkFrame(frames[0], 0x0805, 3, Bytes{0x41, 0x00, 0x00});
}

// ---------------------------------------------------------------- V13

TEST_CASE_FIXTURE(Fx, "[V13] the values request is PROCESS_SHORT_BUFFER 0x08FF and its first 7 bytes are [0x40, 0x00, 50, 00 00 81 00]") {
	// HD: COMM_GET_VALUES_SELECTIVE = 50 = 0x32; mask = bit 15 | bit 8 = 0x8100 -> big endian 00 00 81 00 (SRC commands.c:384-395)
	auto v = makeVesc1();
	runFor(1000); // 500 ms FW request -> COMPATIBLE, 1000 ms values request
	const auto frames = txFramesPacket(0xFF, FakeVescPeer::PKT_PROCESS_SHORT_BUFFER);
	REQUIRE(frames.size() >= 2);
	const auto &f = frames[1];
	CHECK(f.extId);
	CHECK_FALSE(f.rtr);
	CHECK(f.id == 0x08FF);
	CHECK(f.time == 1000);
	REQUIRE(f.dlc >= 7); // the defined part; the DLC itself is the KNOWN_ISSUE test below
	CHECK(std::vector<uint8_t>(f.data.begin(), f.data.begin() + 7) == Bytes({0x40, 0x00, 50, 0x00, 0x00, 0x81, 0x00}));
	process();
	CHECK(peer0.requestCount(FakeVescPeer::COMM_GET_VALUES_SELECTIVE) == 1);
}

TEST_CASE_FIXTURE(Fx, "[V13] the values request has DLC 7 (payload 5 bytes + 2)" * KNOWN_ISSUE("B-V2: askGetValue sends the whole 8 byte array (VescCAN.cpp:449-459), DLC 8 with an uninitialised 8th byte")) {
	// HD: SRC comm_can.c:445-455: DLC = payload length + 2 = (COMM id + 4 byte mask) + 2 = 7
	auto v = makeVesc1();
	runFor(1000);
	const auto frames = txFramesPacket(0xFF, FakeVescPeer::PKT_PROCESS_SHORT_BUFFER);
	const uint8_t dlc = frames.size() >= 2 ? frames[1].dlc : uint8_t(0xFF); // a missing frame is covered by the plain test above
	CHECK(dlc == 7); // the defect: 8
}

// ---------------------------------------------------------------- V14

TEST_CASE_FIXTURE(Fx, "[V14] the rotor position request is POLL_ROTOR_POS 0x38FF with DLC 0" * KNOWN_ISSUE_UB_HIDDEN("B-V3: sendMsg passes a null buffer to memcpy with length 0 (VescCAN.cpp:497); UBSan reports it, the frame itself is correct on the host")) {
	auto v = makeVesc1();
	CHECK(CommandHelpers::run(*v, "forceposread", CMDtype::get) == CommandStatus::OK);
	const auto frames = txFramesPacket(0xFF, FakeVescPeer::PKT_POLL_ROTOR_POS);
	REQUIRE(frames.size() == 1);
	checkFrame(frames[0], 0x38FF, 0, Bytes{});
	CHECK(HostTarget::canTxFrames().size() == 1);
}

TEST_CASE_FIXTURE(Fx, "[V14] the rotor position request goes to the configured VESC id (0x05 -> 0x3805)" * KNOWN_ISSUE_UB_HIDDEN("B-V3: sendMsg passes a null buffer to memcpy with length 0 (VescCAN.cpp:497); UBSan reports it, the frame itself is correct on the host")) {
	storeIds(ADR_VESC1_CANID, 0x05, 0x40);
	peer0.setUnitId(0x05);
	auto v = makeVesc1();
	CommandHelpers::run(*v, "forceposread", CMDtype::get);
	const auto frames = txFramesPacket(0x05, FakeVescPeer::PKT_POLL_ROTOR_POS);
	REQUIRE(frames.size() == 1);
	checkFrame(frames[0], 0x3805, 0, Bytes{});
	process();
	CHECK(peer0.count(FakeVescPeer::PKT_POLL_ROTOR_POS) == 1);
}

TEST_CASE_FIXTURE(Fx, "[V14] the rotor position request goes to VESC id 0x7F as 0x387F, DLC 0" * KNOWN_ISSUE_UB_HIDDEN("B-V3: sendMsg passes a null buffer to memcpy with length 0 (VescCAN.cpp:497); UBSan reports it, the frame itself is correct on the host")) {
	storeIds(ADR_VESC1_CANID, 0x7F, 0x40);
	auto v = makeVesc1();
	CommandHelpers::run(*v, "forceposread", CMDtype::get);
	const auto frames = txFramesPacket(0x7F, FakeVescPeer::PKT_POLL_ROTOR_POS);
	REQUIRE(frames.size() == 1);
	checkFrame(frames[0], 0x387F, 0, Bytes{}); // HD: 56 = 0x38
}

}
