/*
 * ODrive CAN driver, groups G6 "Commands (prefix odrv)" and G7 "Persistent settings" (requirements O50 - O54 and
 * O60 - O62 of Tests/Code_Coverage/claude/charters/driver/ODRIVE_REQUIREMENTS.md, lead decisions L1 - L11).
 *
 * Sources: SPEC / CSV = "CAN Protocol - ODrive Documentation 0.5.6" and its message table (frame layouts, cited
 * "CSV 0x0nn"), AD = flash addresses of Firmware/FFBoard/UserExtensions/Inc/eeprom_addresses.h lines 200-204
 * (ADR_ODRIVE_CANID 0x3D0, ADR_ODRIVE_SETTING1_M0 0x3D1, ADR_ODRIVE_SETTING1_M1 0x3D2, ADR_ODRIVE_OFS_M0 0x3D3,
 * ADR_ODRIVE_OFS_M1 0x3D4), HD = hand derivation in the comment, FW = firmware behaviour only (test name starts with
 * "characterization:"). The layout of 0x3D0 in the code differs from the comment in eeprom_addresses.h (L6): the code
 * is characterized (M0 bits 0-5, M1 bits 6-11, bits 12-14 cleared on save).
 *
 * Method: command tests call the command through CommandHelpers on the driver object (instance 0 = ODriveCAN1,
 * instance 1 = ODriveCAN2). Flash tests derive the words by hand, call saveFlash() and compare, or write hand derived
 * words into the fake flash first and construct a driver (the constructor calls restoreFlash). Round trips destroy the
 * object and construct a new one. Nothing sleeps.
 *
 * Derived constants used below (cpr = 0xFFFF = 65535, ODriveCAN::getCpr):
 *  - default 0x3D0 word when absent: 0x3040 = (1 << 6) | 0x3000 (M1 field = 1, bits 12/13 set); saveFlash clears 0x7000.
 *  - maxTorque t (hundredths of Nm, N) is stored as N in bits 0-11, the storepos flag as bit 12 of 0x3D1 / 0x3D2.
 *
 * Host vs target: flash words are uint16_t; every int32 -> uint16 conversion is modular (defined). Position offsets and
 * torques are float (single precision on the target, the same on the host); all float values used here (0.25, 0.75,
 * 2.5, 20.0, 24.5) are exactly representable, and the products used in the derivations are exact or have a fractional
 * part far from an integer so truncation is identical. `canspd` narrows int64 to uint8 (256 -> 0). The driver's
 * `*reinterpret_cast<uint64_t*>(msg.data)` (ODC:394) is not touched by these tests.
 */
#include "doctest.h"
#include "OdriveFixture.h"

#include <map>

namespace {

constexpr uint16_t ADR_CANID = 0x3D0;
constexpr uint16_t ADR_SET_M0 = 0x3D1;
constexpr uint16_t ADR_SET_M1 = 0x3D2;
constexpr uint16_t ADR_OFS_M0 = 0x3D3;
constexpr uint16_t ADR_OFS_M1 = 0x3D4;

using FlashMap = std::map<uint16_t, uint16_t>;

bool hasWord(uint16_t adr) { return HostPlatform::flash().count(adr) != 0; }

uint16_t word(uint16_t adr) {
	const FlashMap &f = HostPlatform::flash();
	auto it = f.find(adr);
	REQUIRE_MESSAGE(it != f.end(), "flash word 0x" << std::hex << adr << " was not written");
	return it->second;
}

void setWord(uint16_t adr, uint16_t value) { HostPlatform::flash()[adr] = value; }

CommandStatus setCmd(CommandHandler &h, const char *name, int64_t val) { return CommandHelpers::run(h, name, CMDtype::set, val); }
CommandStatus typeCmd(CommandHandler &h, const char *name, CMDtype type) { return CommandHelpers::run(h, name, type, 0); }
int64_t getCmd(CommandHandler &h, const char *name) { return CommandHelpers::get(h, name); }

} // namespace

TEST_SUITE("ODrive commands") {

// ---------------------------------------------------------------- O50 canid

TEST_CASE_FIXTURE(OdriveFixture, "[O50] canid get answers the node id 0 of ODriveCAN1 and 1 of ODriveCAN2 (defaults)") {
	auto od1 = makeOdrive1();
	auto od2 = makeOdrive2();
	CHECK(getCmd(*od1, "canid") == 0);
	CHECK(getCmd(*od2, "canid") == 1);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] canid set changes the node id: get returns it and the new Start Anticogging frame uses id node << 5 | 0x10") {
	auto od = makeOdrive1();
	REQUIRE(setCmd(*od, "canid", 5) == CommandStatus::OK);
	CHECK(getCmd(*od, "canid") == 5);
	clearTxFrames();
	REQUIRE(setCmd(*od, "anticogging", 1) == CommandStatus::OK);
	process();
	// HD: node 5 << 5 = 0xA0, | cmd 0x10 = 0xB0 (SPEC CAN Frame, CSV 0x010)
	const auto frames = txFrames(0xB0);
	REQUIRE(frames.size() == 1);
	CHECK_FALSE(frames[0].extId);
	CHECK_FALSE(frames[0].rtr);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] canid set moves the receive filter to the new node: a heartbeat of node 5 is processed") {
	auto od = makeOdrive1();
	REQUIRE(setCmd(*od, "canid", 5) == CommandStatus::OK);
	// HD: filter id 5 << 5 = 0xA0, mask 0x7E0 (node bits only)
	const auto &filters = HostTarget::canPort().filters;
	REQUIRE(filters.size() >= 2);
	CHECK_FALSE(filters.front().active); // the node 0 filter was removed
	CHECK(filters.back().active);
	CHECK(filters.back().filter_id == 0xA0);
	CHECK(filters.back().filter_mask == 0x7E0);
	peer0.setNodeId(5);
	heartbeat(peer0, 8, 0x1234);
	CHECK(getCmd(*od, "state") == 8);
	CHECK(getCmd(*od, "errors") == 0x1234);
}

// ---------------------------------------------------------------- O50 canspd

TEST_CASE_FIXTURE(OdriveFixture, "[O50] canspd set 5 changes the speed preset of the port and get returns it") {
	auto od = makeOdrive1();
	REQUIRE(setCmd(*od, "canspd", 5) == CommandStatus::OK);
	CHECK(getCmd(*od, "canspd") == 5);
	CHECK(HostTarget::canPort().getSpeedPreset() == 5);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] characterization: canspd get returns the speed preset of the port (the fake port starts at preset 4 and the driver does not lower it)") {
	auto od = makeOdrive1();
	CHECK(getCmd(*od, "canspd") == HostTarget::canPort().getSpeedPreset());
	CHECK(getCmd(*od, "canspd") == 4);
}

// ---------------------------------------------------------------- O50 errors, state, vbus

TEST_CASE_FIXTURE(OdriveFixture, "[O50] errors get returns the axis error word of the last heartbeat (CSV 0x001: uint32 @0)") {
	auto od = makeOdrive1();
	heartbeat(peer0, 1, 0x12345678);
	// HD: 0x12345678 = 305419896
	CHECK(getCmd(*od, "errors") == 305419896);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] errors rejects set (read only)") {
	auto od = makeOdrive1();
	CHECK(setCmd(*od, "errors", 0) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] characterization: state get returns the axis state byte of the last heartbeat (CSV 0x001: uint8 @4; the numbers 8 and 1 are the driver's enum, L2)") {
	auto od = makeOdrive1();
	heartbeat(peer0, 8);
	CHECK(getCmd(*od, "state") == 8); // characterization of the number: closed loop = 8 is the driver's enum (L2)
	heartbeat(peer0, 1);
	CHECK(getCmd(*od, "state") == 1);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] state rejects set (read only)") {
	auto od = makeOdrive1();
	CHECK(setCmd(*od, "state", 8) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] vbus get returns the bus voltage of the last reply in mV (CSV 0x017: float32 @0)") {
	auto od = makeOdrive1();
	peer0.setBus(24.5f, 0.0f);
	peer0.sendBusVoltage();
	CHECK(getCmd(*od, "vbus") == 24500); // HD: 24.5 V * 1000
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] vbus rejects set (read only)") {
	auto od = makeOdrive1();
	CHECK(setCmd(*od, "vbus", 1) == CommandStatus::ERR);
}

// ---------------------------------------------------------------- O50 anticogging

TEST_CASE_FIXTURE(OdriveFixture, "[O50] anticogging set 1 sends one Start Anticogging frame 0x10 that the ODrive accepts") {
	auto od = makeOdrive1();
	clearTxFrames();
	REQUIRE(setCmd(*od, "anticogging", 1) == CommandStatus::OK);
	process();
	const auto frames = txFramesCmd(0, 0x10);
	REQUIRE(frames.size() == 1);
	CHECK_FALSE(frames[0].extId);
	CHECK_FALSE(frames[0].rtr);
	CHECK(peer0.anticoggingCount() == 1);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] anticogging set 0 and anticogging get are rejected and send nothing") {
	auto od = makeOdrive1();
	clearTxFrames();
	CHECK(setCmd(*od, "anticogging", 0) == CommandStatus::ERR);
	CHECK(typeCmd(*od, "anticogging", CMDtype::get) == CommandStatus::ERR);
	process();
	CHECK(HostTarget::canTxFrames().empty());
	CHECK(peer0.anticoggingCount() == 0);
}

// ---------------------------------------------------------------- O50 connected

TEST_CASE_FIXTURE(OdriveFixture, "[O50] connected get is 0 before the driver thread has run its first body") {
	auto od = makeOdrive1();
	CHECK(getCmd(*od, "connected") == 0);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] connected get is 1 while the ODrive sends heartbeats for longer than the 2 s timeout") {
	auto od = makeOdrive1();
	// 3 s: past the first 2 s in which connected is true without any frame (B-O2, O31), so only the heartbeats can
	// keep it at 1. The same time base without heartbeats gives 0: see the silent-ODrive test below.
	runFor(3000);
	CHECK(getCmd(*od, "connected") == 1);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] connected get is 0 after a silent ODrive was not heard for more than 2 s") {
	auto od = makeOdrive1();
	peer0.setSilent(true);
	runFor(3000); // bodies at 500 ... 3000 ms; the last frame time is 0 (never heard), 2500 - 0 > 2000
	CHECK(getCmd(*od, "connected") == 0);
}

// ---------------------------------------------------------------- O50 storepos

TEST_CASE_FIXTURE(OdriveFixture, "[O50] storepos is off by default, set 1 turns it on, set 0 off") {
	auto od = makeOdrive1();
	CHECK(getCmd(*od, "storepos") == 0);
	REQUIRE(setCmd(*od, "storepos", 1) == CommandStatus::OK);
	CHECK(getCmd(*od, "storepos") == 1);
	REQUIRE(setCmd(*od, "storepos", 0) == CommandStatus::OK);
	CHECK(getCmd(*od, "storepos") == 0);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] characterization: storepos treats every non-zero value as on") {
	auto od = makeOdrive1();
	REQUIRE(setCmd(*od, "storepos", 5) == CommandStatus::OK);
	CHECK(getCmd(*od, "storepos") == 1);
}

// ---------------------------------------------------------------- O50 / O51 maxtorque

TEST_CASE_FIXTURE(OdriveFixture, "[O50] maxtorque get returns the default 1.00 Nm as 100 hundredths") {
	auto od = makeOdrive1();
	CHECK(getCmd(*od, "maxtorque") == 100);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] maxtorque set 200 is returned by get and scales the torque: turn(32767) sends 2.0 Nm (CSV 0x00E float32)") {
	auto od = makeOdrive1();
	REQUIRE(setCmd(*od, "maxtorque", 200) == CommandStatus::OK);
	CHECK(getCmd(*od, "maxtorque") == 200);
	clearTxFrames();
	od->turn(32767); // HD: 32767 / 32767 * 2.0 = 2.0 Nm
	process();
	REQUIRE(peer0.torqueSeen());
	CHECK(peer0.lastTorque() == doctest::Approx(2.0f));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] maxtorque set 0 is accepted: turn sends 0 Nm") {
	auto od = makeOdrive1();
	REQUIRE(setCmd(*od, "maxtorque", 0) == CommandStatus::OK);
	CHECK(getCmd(*od, "maxtorque") == 0);
	od->turn(32767);
	process();
	REQUIRE(peer0.torqueSeen());
	CHECK(peer0.lastTorque() == 0.0f);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] characterization: maxtorque set above 4095 is clipped to the 12 bit range (turn(32767) sends 40.95 Nm), a negative value to 0") {
	auto od = makeOdrive1();
	REQUIRE(setCmd(*od, "maxtorque", 5000) == CommandStatus::OK);
	od->turn(32767); // HD: clip(5000, 0, 0xfff) = 4095 -> 40.95 Nm
	process();
	REQUIRE(peer0.torqueSeen());
	CHECK(peer0.lastTorque() == doctest::Approx(40.95f));
	REQUIRE(setCmd(*od, "maxtorque", -7) == CommandStatus::OK);
	od->turn(32767);
	process();
	CHECK(peer0.lastTorque() == 0.0f);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O50] maxtorque rejects an info command") {
	auto od = makeOdrive1();
	CHECK(typeCmd(*od, "maxtorque", CMDtype::info) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O51] maxtorque get returns exactly the value set (53 hundredths)" *
								  KNOWN_ISSUE("B-O1: get truncates int32_t(maxTorque * 100): 0.53f * 100 is 52.999996, get returns 52")) {
	auto od = makeOdrive1();
	REQUIRE(setCmd(*od, "maxtorque", 53) == CommandStatus::OK);
	CHECK(getCmd(*od, "maxtorque") == 53);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O51] a maxtorque of 53 hundredths survives save and restore" *
								  KNOWN_ISSUE("B-O1: saveFlash stores int32_t(maxTorque * 100) & 0xfff = 52 instead of 53")) {
	{
		auto od = makeOdrive1();
		REQUIRE(setCmd(*od, "maxtorque", 53) == CommandStatus::OK);
		od->saveFlash();
	}
	auto od = makeOdrive1();
	CHECK(getCmd(*od, "maxtorque") == 53);
}

// ---------------------------------------------------------------- O52 canspd limits

TEST_CASE_FIXTURE(OdriveFixture, "[O52] characterization: canspd set 3 is accepted, set 0, 1 and 2 are raised to 3 (minimum 250 kbit/s)") {
	auto od = makeOdrive1();
	REQUIRE(setCmd(*od, "canspd", 3) == CommandStatus::OK);
	CHECK(getCmd(*od, "canspd") == 3);
	for (int v : {0, 1, 2}) {
		REQUIRE(setCmd(*od, "canspd", 5) == CommandStatus::OK);
		REQUIRE(setCmd(*od, "canspd", v) == CommandStatus::OK);
		CHECK_MESSAGE(getCmd(*od, "canspd") == 3, "set " << v);
	}
}

TEST_CASE_FIXTURE(OdriveFixture, "[O52] characterization: canspd set above the preset range (6) is ignored, the command still answers OK") {
	auto od = makeOdrive1();
	REQUIRE(setCmd(*od, "canspd", 5) == CommandStatus::OK);
	CHECK(setCmd(*od, "canspd", 6) == CommandStatus::OK); // CANPort_2B::setSpeedPreset returns for preset > 5
	CHECK(getCmd(*od, "canspd") == 5);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O52] characterization: canspd set 256 narrows to uint8 0 and is raised to 3 (NEEDS REVIEW: out-of-range values should be rejected)") {
	auto od = makeOdrive1();
	REQUIRE(setCmd(*od, "canspd", 5) == CommandStatus::OK);
	CHECK(setCmd(*od, "canspd", 256) == CommandStatus::OK); // HD: (uint8_t)256 = 0, max(3, 0) = 3
	CHECK(getCmd(*od, "canspd") == 3);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O52] canspd rejects an info command") {
	auto od = makeOdrive1();
	CHECK(typeCmd(*od, "canspd", CMDtype::info) == CommandStatus::ERR);
}

// ---------------------------------------------------------------- O53 canid get / invalid type

TEST_CASE_FIXTURE(OdriveFixture, "[O53] characterization: canid get rebuilds the receive filter (old one removed, new one with the same id and mask added), the node id is unchanged") {
	auto od = makeOdrive1();
	const size_t before = HostTarget::canPort().filters.size();
	REQUIRE(before == 1);
	CHECK(getCmd(*od, "canid") == 0);
	const auto &filters = HostTarget::canPort().filters;
	REQUIRE(filters.size() == 2);
	CHECK_FALSE(filters[0].active);
	CHECK(filters[1].active);
	CHECK(filters[1].filter_id == 0x000);
	CHECK(filters[1].filter_mask == 0x7E0);
	CHECK(getCmd(*od, "canid") == 0);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O53] characterization: canid with an info command is answered OK (the ERR of handleGetSet is dropped) and changes nothing") {
	auto od = makeOdrive1();
	std::vector<CommandReply> replies;
	CHECK(CommandHelpers::run(*od, "canid", CMDtype::info, 0, &replies) == CommandStatus::OK); // NEEDS REVIEW: charter says rejected
	CHECK(replies.empty());
	CHECK(getCmd(*od, "canid") == 0);
}

// ---------------------------------------------------------------- O54 instances

TEST_CASE_FIXTURE(OdriveFixture, "[O54] both instances answer under their own instance number") {
	auto od1 = makeOdrive1();
	auto od2 = makeOdrive2();
	CHECK(od1->getCommandHandlerInstance() == 0);
	CHECK(od2->getCommandHandlerInstance() == 1);
	CHECK(CommandHandler::getHandlerFromId(0x85, 0) == static_cast<CommandHandler *>(od1.get()));
	CHECK(CommandHandler::getHandlerFromId(0x85, 1) == static_cast<CommandHandler *>(od2.get()));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O54] characterization: both command handlers use class id 0x85 (CLSID_MOT_ODRV0) although ODriveCAN2 identifies as 0x86 (NEEDS REVIEW)") {
	auto od1 = makeOdrive1();
	auto od2 = makeOdrive2();
	CHECK(od1->getCommandHandlerInfo()->clsTypeid == 0x85);
	CHECK(od2->getCommandHandlerInfo()->clsTypeid == 0x85);
	CHECK(od1->getInfo().id == 0x85);
	CHECK(od2->getInfo().id == 0x86);
}

// ---------------------------------------------------------------- O60 save / restore of node ids and settings

TEST_CASE_FIXTURE(OdriveFixture, "[O60] characterization: saveFlash of ODriveCAN1 with defaults writes 0x3D0 = 0x0040 and 0x3D1 = 0x0064 and nothing else") {
	// HD 0x3D0: no word yet -> canIds = 0x3040; M0 field (bits 0-5) cleared and set to node 0: 0x3040; bits 12-14 cleared
	// (& ~0x7000): 0x0040 (the M1 field holds 1).
	// HD 0x3D1: maxTorque 1.0 -> 100 = 0x64; storepos off -> bit 12 clear: 0x0064.
	auto od = makeOdrive1();
	od->saveFlash();
	CHECK(word(ADR_CANID) == 0x0040);
	CHECK(word(ADR_SET_M0) == 0x0064);
	CHECK(HostPlatform::flash().size() == 2);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] characterization: saveFlash of ODriveCAN2 with defaults writes 0x3D0 = 0x0040 and 0x3D2 = 0x0064 and nothing else") {
	// HD 0x3D0: 0x3040 with the M1 field (bits 6-11, mask 0xFC0) cleared = 0x3000, | (1 << 6) = 0x3040, & ~0x7000 = 0x0040.
	auto od = makeOdrive2();
	od->saveFlash();
	CHECK(word(ADR_CANID) == 0x0040);
	CHECK(word(ADR_SET_M1) == 0x0064);
	CHECK(HostPlatform::flash().size() == 2);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] characterization: saving node 5 of motor 0 keeps the motor 1 field and clears the bits 12-14") {
	// HD: stored 0x2242 = 0b0010'0010'0100'0010: M0 field = 2, M1 field = 9 (9 << 6 = 0x240), bit 13 set (old speed bits).
	// ODriveCAN1 restores node 2. canid = 5: 0x2242 & ~0x3F = 0x2240, | 5 = 0x2245, & ~0x7000 = 0x0245.
	setWord(ADR_CANID, 0x2242);
	auto od = makeOdrive1();
	CHECK(getCmd(*od, "canid") == 2);
	REQUIRE(setCmd(*od, "canid", 5) == CommandStatus::OK);
	od->saveFlash();
	CHECK(word(ADR_CANID) == 0x0245);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] characterization: saving node 17 of motor 1 keeps the motor 0 field") {
	// HD: stored 0x0245: M0 = 5, M1 = 9. ODriveCAN2 restores (0x245 >> 6) & 0x3F = 9. canid = 17 (0x11):
	// 0x245 & ~0xFC0 = 0x005, | (17 << 6 = 0x440) = 0x445.
	setWord(ADR_CANID, 0x0245);
	auto od = makeOdrive2();
	CHECK(getCmd(*od, "canid") == 9);
	REQUIRE(setCmd(*od, "canid", 17) == CommandStatus::OK);
	od->saveFlash();
	CHECK(word(ADR_CANID) == 0x0445);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] characterization: saving node 0x0A of motor 0 over a word with all bits set replaces exactly bits 0-5 and clears bits 12-14") {
	// HD: stored 0x7FFF: M0 field = 0x3F (restored), M1 field = 0x3F, bits 12-14 set. canid = 0x0A (0b001010, bit 5 clear so the OR cannot hide a mask that misses bit 5):
	// 0x7FFF & ~0x3F = 0x7FC0, | 0x0A = 0x7FCA, & ~0x7000 = 0x0FCA.
	setWord(ADR_CANID, 0x7FFF);
	auto od = makeOdrive1();
	REQUIRE(getCmd(*od, "canid") == 0x3F);
	REQUIRE(setCmd(*od, "canid", 0x0A) == CommandStatus::OK);
	od->saveFlash();
	CHECK(word(ADR_CANID) == 0x0FCA);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] characterization: saving node 0x14 of motor 1 over a word with all bits set replaces exactly bits 6-11 and clears bits 12-14") {
	// HD: stored 0x7FFF: M1 field = (0x7FFF >> 6) & 0x3F = 0x3F. canid = 0x14 (0b010100, bit 0 clear so the OR cannot hide a mask that misses bit 6):
	// 0x7FFF & ~0xFC0 = 0x703F, | (0x14 << 6 = 0x500) = 0x753F, & ~0x7000 = 0x053F.
	setWord(ADR_CANID, 0x7FFF);
	auto od = makeOdrive2();
	REQUIRE(getCmd(*od, "canid") == 0x3F);
	REQUIRE(setCmd(*od, "canid", 0x14) == CommandStatus::OK);
	od->saveFlash();
	CHECK(word(ADR_CANID) == 0x053F);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] characterization: a max torque of 30.00 Nm (3000 hundredths, above 11 bits) is saved as 0x0BB8 in bits 0-11 and restored") {
	// HD: 3000 = 11 * 256 + 184 = 0xB00 + 0xB8 = 0xBB8 (needs bit 11); 30.0f * 100 = 3000 exactly; storepos off.
	{
		auto od = makeOdrive1();
		REQUIRE(setCmd(*od, "maxtorque", 3000) == CommandStatus::OK);
		od->saveFlash();
		CHECK(word(ADR_SET_M0) == 0x0BB8);
	}
	auto od = makeOdrive1();
	CHECK(getCmd(*od, "maxtorque") == 3000);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] characterization: restore reads the node id of motor 0 from bits 0-5 and of motor 1 from bits 6-11 of 0x3D0") {
	// HD: 0x0245: bits 0-5 = 0x05 (5), bits 6-11 = 0x245 >> 6 = 9; filters 5 << 5 = 0xA0 and 9 << 5 = 0x120.
	setWord(ADR_CANID, 0x0245);
	auto od1 = makeOdrive1();
	auto od2 = makeOdrive2();
	const auto &filters = HostTarget::canPort().filters; // before the canid gets: a get rebuilds the filter (O53)
	REQUIRE(filters.size() == 2);
	CHECK(filters[0].filter_id == 0xA0);
	CHECK(filters[1].filter_id == 0x120);
	CHECK(getCmd(*od1, "canid") == 5);
	CHECK(getCmd(*od2, "canid") == 9);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] a node id saved by motor 0 and one saved by motor 1 survive in the same word and are restored") {
	{
		auto od1 = makeOdrive1();
		REQUIRE(setCmd(*od1, "canid", 5) == CommandStatus::OK);
		od1->saveFlash();
		auto od2 = makeOdrive2();
		REQUIRE(setCmd(*od2, "canid", 9) == CommandStatus::OK);
		od2->saveFlash();
	}
	auto od1 = makeOdrive1();
	auto od2 = makeOdrive2();
	CHECK(getCmd(*od1, "canid") == 5);
	CHECK(getCmd(*od2, "canid") == 9);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] characterization: 0x3D1 holds max torque in bits 0-11 and the storepos flag in bit 12: 2.50 Nm with storepos is 0x10FA") {
	// HD: 250 hundredths = 0xFA (2.5f * 100 = 250 exactly); storepos -> 1 << 12 = 0x1000; 0x1000 | 0xFA = 0x10FA.
	// The offset word 0x3D3 is written too (storepos on): offset 0 -> 0x0000.
	auto od = makeOdrive1();
	REQUIRE(setCmd(*od, "maxtorque", 250) == CommandStatus::OK);
	REQUIRE(setCmd(*od, "storepos", 1) == CommandStatus::OK);
	od->saveFlash();
	CHECK(word(ADR_SET_M0) == 0x10FA);
	CHECK(word(ADR_OFS_M0) == 0x0000);
	CHECK_FALSE(hasWord(ADR_SET_M1));
	CHECK_FALSE(hasWord(ADR_OFS_M1));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] characterization: ODriveCAN2 stores its settings in 0x3D2 and its offset in 0x3D4, 2.00 Nm without storepos is 0x00C8") {
	// HD: 200 = 0xC8, no flag.
	auto od = makeOdrive2();
	REQUIRE(setCmd(*od, "maxtorque", 200) == CommandStatus::OK);
	od->saveFlash();
	CHECK(word(ADR_SET_M1) == 0x00C8);
	CHECK_FALSE(hasWord(ADR_SET_M0));
	CHECK_FALSE(hasWord(ADR_OFS_M0));
	CHECK_FALSE(hasWord(ADR_OFS_M1));
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] characterization: restore of 0x3D1 = 0x10FA gives max torque 250 and storepos on for motor 0, 0x3D2 is not read by motor 0") {
	setWord(ADR_SET_M0, 0x10FA);
	setWord(ADR_SET_M1, 0x00C8);
	auto od = makeOdrive1();
	CHECK(getCmd(*od, "maxtorque") == 250);
	CHECK(getCmd(*od, "storepos") == 1);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] characterization: restore of 0x3D2 = 0x10C8 gives max torque 200 and storepos on for motor 1, 0x3D1 is not read by motor 1") {
	// HD: 0x10C8 = 0x1000 | 200
	setWord(ADR_CANID, 0x0040);
	setWord(ADR_SET_M0, 0x00FA);
	setWord(ADR_SET_M1, 0x10C8);
	auto od = makeOdrive2();
	CHECK(getCmd(*od, "maxtorque") == 200);
	CHECK(getCmd(*od, "storepos") == 1);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] ODriveCAN2 restores its own settings word 0x3D2 also when the node id word 0x3D0 is absent" *
								  KNOWN_ISSUE("B-O10: restoreFlash selects 0x3D2 only inside the successful read of 0x3D0, otherwise motor 1 reads the settings of motor 0 (0x3D1)")) {
	// HD: 0x3D1 = 250 (motor 0), 0x3D2 = 200 (motor 1), no 0x3D0 word: motor 1 must restore 200.
	setWord(ADR_SET_M0, 0x00FA);
	setWord(ADR_SET_M1, 0x00C8);
	auto od = makeOdrive2();
	CHECK(getCmd(*od, "maxtorque") == 200);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] characterization: restore uses only bit 0 of the flag nibble (bits 12-15): 0x2064 is storepos off") {
	setWord(ADR_SET_M0, 0x2064);
	auto od = makeOdrive1();
	CHECK(getCmd(*od, "storepos") == 0);
	CHECK(getCmd(*od, "maxtorque") == 100);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O60] settings and storepos survive a save, destroy and construct cycle for both motors") {
	{
		auto od1 = makeOdrive1();
		auto od2 = makeOdrive2();
		REQUIRE(setCmd(*od1, "maxtorque", 250) == CommandStatus::OK);
		REQUIRE(setCmd(*od1, "storepos", 1) == CommandStatus::OK);
		REQUIRE(setCmd(*od2, "maxtorque", 2000) == CommandStatus::OK); // 20.00 Nm, 0x7D0
		od1->saveFlash();
		od2->saveFlash();
	}
	auto od1 = makeOdrive1();
	auto od2 = makeOdrive2();
	CHECK(getCmd(*od1, "maxtorque") == 250);
	CHECK(getCmd(*od1, "storepos") == 1);
	CHECK(getCmd(*od2, "maxtorque") == 2000);
	CHECK(getCmd(*od2, "storepos") == 0);
}

// ---------------------------------------------------------------- O61 position offset

TEST_CASE_FIXTURE(OdriveFixture, "[O61] characterization: with storepos on, saveFlash stores the position offset in 0x3D3 as counts (setPos(1000) at 0.25 turn -> 0x3C17)") {
	// HD: lastPos 0.25 turn. setPos(1000): posOffset = 0.25 - 1000 / 65535. Stored int32(posOffset * 65535) =
	// 0.25 * 65535 - 1000 = 16383.75 - 1000 = 15383.75 -> 15383 = 0x3C17 (0x3C00 = 15360, + 23 = 0x17).
	auto od = makeOdrive1();
	peer0.setEncoder(0.25f, 0.0f);
	peer0.sendEncoderEstimates();
	od->setPos(1000);
	REQUIRE(setCmd(*od, "storepos", 1) == CommandStatus::OK);
	od->saveFlash();
	CHECK(word(ADR_OFS_M0) == 0x3C17);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O61] the reported position continues where it was after save and restore (offset inside the int16 range)") {
	// HD: after restore posOffset = 15383 / 65535; at lastPos 0.25 the position is (16383.75 - 15383) = 1000.75 counts,
	// truncated by getPos to 1000 (the saved offset lost 0.25 count by truncation).
	{
		auto od = makeOdrive1();
		peer0.setEncoder(0.25f, 0.0f);
		peer0.sendEncoderEstimates();
		od->setPos(1000);
		REQUIRE(setCmd(*od, "storepos", 1) == CommandStatus::OK);
		od->saveFlash();
	}
	auto od = makeOdrive1();
	peer0.sendEncoderEstimates();
	CHECK(getCmd(*od, "storepos") == 1);
	CHECK(od->getPos() == 1000);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O61] a negative offset within the int16 range is saved as two's complement and restored (setPos(20000) at 0.25 turn)") {
	// HD: offset counts = 0.25 * 65535 - 20000 = 16383.75 - 20000 = -3616.25 -> int32 -3616 (truncation toward zero)
	// -> uint16 65536 - 3616 = 61920 = 0xF1E0. Restored int16 -3616: position = 16383.75 + 3616 = 19999.75 -> 19999.
	{
		auto od = makeOdrive1();
		peer0.setEncoder(0.25f, 0.0f);
		peer0.sendEncoderEstimates();
		od->setPos(20000);
		REQUIRE(setCmd(*od, "storepos", 1) == CommandStatus::OK);
		od->saveFlash();
		CHECK(word(ADR_OFS_M0) == 0xF1E0);
	}
	auto od = makeOdrive1();
	peer0.sendEncoderEstimates();
	CHECK(od->getPos() == 19999);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O61] ODriveCAN2 saves and restores its offset through 0x3D4") {
	{
		auto od = makeOdrive2();
		peer1.setEncoder(0.25f, 0.0f);
		peer1.sendEncoderEstimates();
		od->setPos(1000);
		REQUIRE(setCmd(*od, "storepos", 1) == CommandStatus::OK);
		od->saveFlash();
		CHECK(word(ADR_OFS_M1) == 0x3C17); // same derivation as for motor 0
		CHECK_FALSE(hasWord(ADR_OFS_M0));
	}
	auto od = makeOdrive2();
	peer1.sendEncoderEstimates();
	CHECK(od->getPos() == 1000);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O61] with storepos off saveFlash writes no offset and a stored offset word is not applied") {
	setWord(ADR_OFS_M0, 0x3C17); // would be 15383 counts
	auto od = makeOdrive1();
	peer0.setEncoder(0.25f, 0.0f);
	peer0.sendEncoderEstimates();
	od->saveFlash();
	CHECK(word(ADR_OFS_M0) == 0x3C17); // untouched
	CHECK(od->getPos_f() == 0.25f);	   // offset 0
}

TEST_CASE_FIXTURE(OdriveFixture, "[O61] with storepos on and no stored offset word the offset is 0") {
	setWord(ADR_SET_M0, 0x1064);
	auto od = makeOdrive1();
	REQUIRE(getCmd(*od, "storepos") == 1);
	peer0.setEncoder(0.25f, 0.0f);
	peer0.sendEncoderEstimates();
	CHECK(od->getPos_f() == 0.25f);
}

// ---------------------------------------------------------------- O62 offset beyond half a turn

TEST_CASE_FIXTURE(OdriveFixture, "[O62] an offset of 0.75 turn (49151 counts) is restored so that the position continues" *
								  KNOWN_ISSUE("B-O7: the offset is read back as int16, 49151 becomes -16385")) {
	// HD: lastPos 0.75, setPos(0): posOffset = 0.75, counts 0.75 * 65535 = 49151.25 -> 49151 = 0xBFFF (> 32767).
	// Intended: after restore at the same lastPos the position is 0. Actual: int16 0xBFFF = -16385, position 65535.
	{
		auto od = makeOdrive1();
		peer0.setEncoder(0.75f, 0.0f);
		peer0.sendEncoderEstimates();
		od->setPos(0);
		REQUIRE(setCmd(*od, "storepos", 1) == CommandStatus::OK);
		od->saveFlash();
	}
	auto od = makeOdrive1();
	peer0.sendEncoderEstimates();
	// HD: intended position = 49151.25 - 49151 = 0.25 count, truncated by getPos to 0
	CHECK(od->getPos() == 0);
}

TEST_CASE_FIXTURE(OdriveFixture, "[O62] characterization: an offset of 0.75 turn is saved as the 16 bit word 0xBFFF" ) {
	// HD: see above, 49151 = 0xBFFF. The saved word itself is complete; only the restore reads it as int16.
	auto od = makeOdrive1();
	peer0.setEncoder(0.75f, 0.0f);
	peer0.sendEncoderEstimates();
	od->setPos(0);
	REQUIRE(setCmd(*od, "storepos", 1) == CommandStatus::OK);
	od->saveFlash();
	CHECK(word(ADR_OFS_M0) == 0xBFFF);
}

} // TEST_SUITE
