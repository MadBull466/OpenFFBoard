/*
 * MyActuator RMD CAN driver (RmdMotorCAN), groups R8 "Commands" and R9 "Persistence" of
 * Tests/Code_Coverage/claude/charters/driver/RMD_REQUIREMENTS.md (work order RMC of Tests/Code_Coverage/claude/charters/driver/P16_WORK_ORDERS.md).
 *
 * Requirements: R8.1 - R8.8 (canid, maxtorque, curr / temp / vbus / errors / model / requestpos, error statuses),
 * R9.1 - R9.5 (saveFlash words, restore at construction and with restoreFlash, offsets, own addresses per instance).
 *
 * Oracles: SP = MyActuator "CAN Bus Motor Motion Protocol V4.3" (ids 1..32, frame id 0x140 + ID, reply id 0x240 + ID,
 * SP 2.19 torque frame, 2.14 status 1 reply, 2.33 model reply, 2.12 multi turn angle 0.01 degree); HD = hand derivation
 * in the comment next to the assertion (frame bytes, flash words, offset arithmetic); FW = firmware behaviour only
 * (test name starts with "characterization:", NEEDS REVIEW). Flash layout (no document, charter "Facts"):
 *  word 1 (ADR_RMD1_DATA1 0x3C0 / ADR_RMD2_DATA1 0x3C3) = ((nodeId - 1) & 0x1F) | activerequests << 6,
 *  word 2 (ADR_RMD1_TORQUE 0x3C1 / ADR_RMD2_TORQUE 0x3C4) = maxTorque & 0x7FFF,
 *  word 3 (ADR_RMD1_OFFSET 0x3C2 / ADR_RMD2_OFFSET 0x3C5) = (uint16_t)(posOffset % 36000) (C remainder, sign of dividend).
 *
 * Method: commands are executed through CommandHelpers on the driver object. Values of curr / temp / vbus / errors / model
 * come from replies of FakeRmdPeer (sendReply delivers them through the fake CAN filter and the real canRxPendCallback).
 * Positions: a 0x92 reply with multi turn angle 100000 (1000.00 degrees) sets lastPos = 1000.0f exactly, so
 * getPos() = 100000 - posOffset and setPos(p) gives posOffset = 100000 - p (all float values are integers below 2^24,
 * exact). A fresh driver has lastPos 0, hence getPos() = -posOffset until a report arrives.
 * Frame checks: the frames of the driver are read from HostTarget::canTxFrames; no test sleeps.
 *
 * Host vs target: float to int16 conversion in turn() with maxtorque above 0x7fff is undefined (B-RMD-3, the UB case is
 * skipped in sanitizer builds). All other float arithmetic here is exact. The flash words are uint16_t (modular
 * conversions are defined). int64 -> uint8 narrowing of canid (256 -> 0) is defined.
 *
 * B-RMD-8 (new): the status of handleGetSet is discarded for maxtorque, so an unsupported command type answers OK.
 * KNOWN_ISSUEs: B-RMD-2 (canid unchecked: 0 accepted, 256 becomes 0), B-RMD-3 (maxtorque above 0x7fff accepted, two
 * cases, plus the UB case), B-RMD-5 (offset stored uint16_t, restored int16_t: 35000 and -35999 do not round trip).
 * characterizations: canid 33 / 255 clamped to 32 (R8.3), saveFlash masks maxtorque 40000 to 7232, restoreFlash() after
 * construction changes the id but does not rebuild the filter.
 * Limits of the findings: B-RMD-2 for 256 is shown only for the exact wrap to 0 (261 wraps to 5 silently, 300 to 44, no
 * test of its own). The B-RMD-8 test calls command() directly with type none; the real parser never produces that type
 * for a registered command, so the defect is theoretical (a status that is ignored, no reachable wrong behaviour).
 */
#include "doctest.h"
#include "RmdFixture.h"

#include "eeprom_addresses.h"

#include <map>

namespace {

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

/** Torque of an 0xA1 frame: int16 little endian in data[4..5] (SP 2.19) */
int16_t torqueOf(const HostTarget::CanTxFrame &f) { return (int16_t)(uint16_t)(f.data[4] | (f.data[5] << 8)); }

/** The position report "1000.00 degrees" (multi turn 100000, 0.01 degree units) */
constexpr int32_t REPORT_POS = 100000;

} // namespace

TEST_SUITE("RMD commands") {

// ---------------------------------------------------------------- R8.1 canid

TEST_CASE_FIXTURE(RmdFixture, "[R8.1 canid] canid get answers node 1 for RmdMotorCAN1 and node 2 for RmdMotorCAN2") {
	auto r1 = makeRmd1();
	auto r2 = makeRmd2();
	CHECK(getCmd(*r1, "canid") == 1);
	CHECK(getCmd(*r2, "canid") == 2);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.1 canid] canid set 5 is accepted and read back") {
	auto rmd = makeRmd1();
	CHECK(setCmd(*rmd, "canid", 5) == CommandStatus::OK);
	CHECK(getCmd(*rmd, "canid") == 5);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.1 canid] after canid set 5 the driver frames use the id 0x145 with DLC 8") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "canid", 5) == CommandStatus::OK);
	clearTxFrames();
	rmd->setTorque(0);
	// HD (SP 1.1.2): command id 0x140 + ID 5 = 0x145
	const auto frames = txFrames(0x145);
	REQUIRE(frames.size() == 1);
	CHECK(frames[0].dlc == 8);
	CHECK_FALSE(frames[0].extId);
	CHECK(txFrames(0x141).empty());
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.1 canid] after canid set 5 the old reply filter 0x241 is removed and the filter 0x245 with mask 0x3FF exists") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "canid", 5) == CommandStatus::OK);
	const auto &filters = HostTarget::canPort().filters;
	REQUIRE(filters.size() == 2);
	CHECK_FALSE(filters[0].active); // HD (SP 1.1.2): old reply id 0x240 + 1
	CHECK(filters[1].active);
	CHECK(filters[1].filter_id == 0x245); // reply id 0x240 + 5
	CHECK(filters[1].filter_mask == 0x3FF);
	CHECK(HostTarget::canPort().activeFilters() == 1);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.1 canid] after canid set 5 a reply with id 0x245 is processed") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "canid", 5) == CommandStatus::OK);
	peer1.setNodeId(5);
	peer1.setTemperature(40);
	CHECK(peer1.sendReply(0xA1) != 0); // accepted by the filter
	CHECK(getCmd(*rmd, "temp") == 40);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.1 canid] after canid set 5 a reply with the old id 0x241 is not processed") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "canid", 5) == CommandStatus::OK);
	peer1.setTemperature(40); // peer1 still answers with node 1 = id 0x241
	CHECK(peer1.sendReply(0xA1) == 0); // rejected by the filter
	CHECK(getCmd(*rmd, "temp") == 0);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.1 canid] canid set 32 uses the frame id 0x160 and the reply filter 0x260") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "canid", 32) == CommandStatus::OK);
	CHECK(getCmd(*rmd, "canid") == 32);
	clearTxFrames();
	rmd->setTorque(0);
	CHECK(txFrames(0x160).size() == 1); // HD: 0x140 + 32 = 0x160
	REQUIRE(HostTarget::canPort().filters.size() == 2);
	CHECK(HostTarget::canPort().filters[1].filter_id == 0x260); // HD: 0x240 + 32
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.1 canid] canid set to the current id leaves the filter table unchanged") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "canid", 1) == CommandStatus::OK);
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].active);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.1 canid] canid set with a motor at the new node: the cycle reaches motorReady with the peer of node 5") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "canid", 5) == CommandStatus::OK);
	peer1.setNodeId(5);
	reachAvailable(*rmd);
	CHECK(rmd->motorReady());
	CHECK(txFramesCmd(0x145, 0xB5).size() == 1); // the probe goes to 0x145 (one probe in the first cycle)
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.1 canid] canid set downward: 32 then 5 moves the frames to 0x145 and the filter to 0x245") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "canid", 32) == CommandStatus::OK);
	REQUIRE(setCmd(*rmd, "canid", 5) == CommandStatus::OK);
	clearTxFrames();
	rmd->setTorque(0);
	CHECK(txFrames(0x145).size() == 1);
	CHECK(txFrames(0x160).empty());
	const auto &filters = HostTarget::canPort().filters;
	CHECK(HostTarget::canPort().activeFilters() == 1);
	REQUIRE(filters.size() == 3);
	CHECK(filters[2].active);
	CHECK(filters[2].filter_id == 0x245);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.8 status] a getat on requestpos returns ERR") {
	auto rmd = makeRmd1();
	CHECK(typeCmd(*rmd, "requestpos", CMDtype::getat) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.5 maxtorque] maxtorque set 65536 is not accepted" *
								  KNOWN_ISSUE("B-RMD-3: 65536 wraps to the uint16_t 0 and is stored with status OK")) {
	auto rmd = makeRmd1();
	CHECK(setCmd(*rmd, "maxtorque", 65536) != CommandStatus::OK); // first failing assertion: the status is OK
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.7 flags] the eight commands are registered with the flags of the charter") {
	auto rmd = makeRmd1();
	struct E { const char *name; uint32_t flags; };
	const E table[] = {
		{"canid", CMDFLAG_GET | CMDFLAG_SET},
		{"errors", CMDFLAG_GET},
		{"maxtorque", CMDFLAG_GET | CMDFLAG_SET},
		{"curr", CMDFLAG_GET},
		{"temp", CMDFLAG_GET},
		{"vbus", CMDFLAG_GET},
		{"requestpos", CMDFLAG_GET | CMDFLAG_SET},
		{"model", CMDFLAG_GET | CMDFLAG_STR_ONLY},
	};
	for (const E &e : table) {
		INFO("command " << e.name);
		CmdHandlerCommanddef *def = rmd->getCommandFromName(e.name);
		REQUIRE(def != nullptr);
		CHECK(def->flags == e.flags);
	}
}

// ---------------------------------------------------------------- R8.2 canid range (B-RMD-2)

TEST_CASE_FIXTURE(RmdFixture, "[R8.2 canid] canid set 0 is not accepted" *
								  KNOWN_ISSUE("B-RMD-2: canid set takes (uint8_t) cmd.val unchecked, 0 is accepted and the frame id becomes 0x140, which is no motor id (SP: ID 1..32)")) {
	auto rmd = makeRmd1();
	CHECK(setCmd(*rmd, "canid", 0) != CommandStatus::OK); // first failing assertion: the status is OK
	CHECK(getCmd(*rmd, "canid") != 0);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.2 canid] canid set 256 does not leave the id at 0 (not truncated to 0)" *
								  KNOWN_ISSUE("B-RMD-2: 256 is narrowed to uint8_t 0 and stored, SP: ID 1..32")) {
	auto rmd = makeRmd1();
	setCmd(*rmd, "canid", 256); // rejected or clamped are both fine, the stored id must be a valid id 1..32
	const int64_t id = getCmd(*rmd, "canid"); // first failing assertion: id is 0
	CHECK(id >= 1);
	CHECK(id <= 32);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.3 canid] characterization: canid set 33 is answered OK and clamped to 32 (frame id 0x160)") {
	auto rmd = makeRmd1();
	CHECK(setCmd(*rmd, "canid", 33) == CommandStatus::OK);
	CHECK(getCmd(*rmd, "canid") == 32); // setCanFilter: nodeId = min(nodeId, 32)
	clearTxFrames();
	rmd->setTorque(0);
	CHECK(txFrames(0x160).size() == 1);
	REQUIRE(HostTarget::canPort().filters.size() == 2);
	CHECK(HostTarget::canPort().filters[1].filter_id == 0x260);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.3 canid] characterization: canid set 255 is answered OK and clamped to 32") {
	auto rmd = makeRmd1();
	CHECK(setCmd(*rmd, "canid", 255) == CommandStatus::OK);
	CHECK(getCmd(*rmd, "canid") == 32);
}

// ---------------------------------------------------------------- R8.4 maxtorque

TEST_CASE_FIXTURE(RmdFixture, "[R8.4 maxtorque] maxtorque get answers the default 1000") {
	auto rmd = makeRmd1();
	CHECK(getCmd(*rmd, "maxtorque") == 1000);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.4 maxtorque] maxtorque set 1 is read back") {
	auto rmd = makeRmd1();
	CHECK(setCmd(*rmd, "maxtorque", 1) == CommandStatus::OK);
	CHECK(getCmd(*rmd, "maxtorque") == 1);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.4 maxtorque] maxtorque set 32767 (0x7fff) is read back") {
	auto rmd = makeRmd1();
	CHECK(setCmd(*rmd, "maxtorque", 32767) == CommandStatus::OK);
	CHECK(getCmd(*rmd, "maxtorque") == 32767);
}

namespace {
/** Sets maxtorque, makes the motor available and active, calls turn(32767) and returns the torque of the one 0xA1 frame */
int16_t fullScaleTorque(RmdFixture &fx, RmdMotorCAN &rmd, int64_t maxtorque) {
	REQUIRE(setCmd(rmd, "maxtorque", maxtorque) == CommandStatus::OK);
	fx.reachAvailable(rmd);
	rmd.startMotor();
	fx.clearTxFrames();
	rmd.turn(0x7fff);
	const auto frames = RmdFixture::txFramesCmd(0x141, 0xA1);
	REQUIRE(frames.size() == 1);
	return torqueOf(frames[0]);
}
} // namespace

TEST_CASE_FIXTURE(RmdFixture, "[R8.4 maxtorque] maxtorque 0 is read back and turn(0x7fff) sends torque 0") {
	auto rmd = makeRmd1();
	// HD: 32767 / 32767.0f * 0 = 0
	CHECK(fullScaleTorque(*this, *rmd, 0) == 0);
	CHECK(getCmd(*rmd, "maxtorque") == 0);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.4 maxtorque] turn(0x7fff) with maxtorque 1 sends torque 1") {
	auto rmd = makeRmd1();
	// HD: 32767 / 32767.0f * 1 = 1 -> data[4..5] = 0x01 0x00
	CHECK(fullScaleTorque(*this, *rmd, 1) == 1);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.4 maxtorque] turn(0x7fff) with the default maxtorque 1000 sends torque 1000 (0xE8 0x03)") {
	auto rmd = makeRmd1();
	// HD: 1000 = 0x03E8 little endian 0xE8 0x03
	REQUIRE(fullScaleTorque(*this, *rmd, 1000) == 1000);
	const auto f = txFramesCmd(0x141, 0xA1)[0];
	CHECK(f.data[4] == 0xE8);
	CHECK(f.data[5] == 0x03);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.4 maxtorque] turn(0x7fff) with maxtorque 32767 sends torque 32767 (0xFF 0x7F)") {
	auto rmd = makeRmd1();
	// HD: 32767 / 32767.0f = 1.0f exactly, * 32767 = 32767.0f -> 0x7FFF
	REQUIRE(fullScaleTorque(*this, *rmd, 32767) == 32767);
	const auto f = txFramesCmd(0x141, 0xA1)[0];
	CHECK(f.data[4] == 0xFF);
	CHECK(f.data[5] == 0x7F);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.4 maxtorque] turn(-32767) with maxtorque 500 sends -500 (0x0C 0xFE)") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "maxtorque", 500) == CommandStatus::OK);
	reachAvailable(*rmd);
	rmd->startMotor();
	clearTxFrames();
	rmd->turn(-32767);
	const auto frames = txFramesCmd(0x141, 0xA1);
	REQUIRE(frames.size() == 1);
	// HD: -32767 / 32767.0f * 500 = -500 exactly = 0xFE0C -> data[4] 0x0C, data[5] 0xFE
	CHECK(torqueOf(frames[0]) == -500);
	CHECK(frames[0].data[4] == 0x0C);
	CHECK(frames[0].data[5] == 0xFE);
}

// ---------------------------------------------------------------- R8.5 / R8.6 maxtorque above 0x7fff (B-RMD-3)

TEST_CASE_FIXTURE(RmdFixture, "[R8.5 maxtorque] maxtorque set 40000 is not accepted" *
								  KNOWN_ISSUE("B-RMD-3: maxtorque set stores any uint16_t, 40000 is accepted with status OK (the header says max 0x7fff, the flash word keeps 15 bits)")) {
	auto rmd = makeRmd1();
	CHECK(setCmd(*rmd, "maxtorque", 40000) != CommandStatus::OK); // first failing assertion: the status is OK
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.5 maxtorque] after maxtorque set 40000 the stored value is not above 0x7fff" *
								  KNOWN_ISSUE("B-RMD-3: the stored maxtorque is 40000")) {
	auto rmd = makeRmd1();
	setCmd(*rmd, "maxtorque", 40000); // rejected, clamped or masked are all fine
	CHECK(getCmd(*rmd, "maxtorque") <= 0x7fff); // reads 40000
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.6 maxtorque] turn(0x7fff) with maxtorque 40000 sends a positive torque, no float to int16 overflow" *
								  KNOWN_ISSUE_UB("B-RMD-3: (int16_t)(1.0f * 40000) is out of range, undefined; the x86 result is 40000 - 65536 = -25536")) {
	auto rmd = makeRmd1();
	setCmd(*rmd, "maxtorque", 40000);
	reachAvailable(*rmd);
	rmd->startMotor();
	clearTxFrames();
	rmd->turn(0x7fff); // the sign of the torque must follow the sign of the power
	const auto frames = txFramesCmd(0x141, 0xA1);
	REQUIRE(frames.size() == 1);
	CHECK(torqueOf(frames[0]) > 0); // first failing assertion: the host gives -25536
}

// ---------------------------------------------------------------- R8.7 read only values, requestpos

TEST_CASE_FIXTURE(RmdFixture, "[R8.7 curr] curr returns the iq of an 0xA1 reply: -100 (0xFF9C, SP 2.19 example)") {
	auto rmd = makeRmd1();
	peer1.setIq(-100);
	peer1.sendReply(0xA1);
	CHECK(getCmd(*rmd, "curr") == -100);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.7 curr] curr returns a positive iq: 100 (0x0064)") {
	auto rmd = makeRmd1();
	peer1.setIq(100);
	peer1.sendReply(0xA1);
	CHECK(getCmd(*rmd, "curr") == 100);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.7 temp] temp returns the temperature of an 0xA1 reply: 50 (0x32, SP 2.19 example)") {
	auto rmd = makeRmd1();
	peer1.setTemperature(50);
	peer1.sendReply(0xA1);
	CHECK(getCmd(*rmd, "temp") == 50);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.7 temp] temp returns the temperature of a 0x9A reply: 37") {
	auto rmd = makeRmd1();
	peer1.setTemperature(37);
	peer1.sendReply(0x9A);
	CHECK(getCmd(*rmd, "temp") == 37);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.7 vbus] vbus returns the voltage of a 0x9A reply in 0.1 V: 0x01E5 = 485") {
	auto rmd = makeRmd1();
	peer1.setVoltage(485);
	peer1.sendReply(0x9A);
	CHECK(getCmd(*rmd, "vbus") == 485);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.7 errors] errors returns the error flags of a 0x9A reply: 0x0004 (low voltage, SP 2.14)") {
	auto rmd = makeRmd1();
	peer1.setErrors(0x0004);
	peer1.sendReply(0x9A);
	CHECK(getCmd(*rmd, "errors") == 4);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.7 model] model returns the 7 characters of a 0xB5 reply (SP 2.33)") {
	auto rmd = makeRmd1();
	// the peer default model is "X8S2V10", the SP 2.33 example 0x58 0x38 0x53 0x32 0x56 0x31 0x30
	peer1.sendReply(0xB5);
	std::vector<CommandReply> replies;
	REQUIRE(CommandHelpers::run(*rmd, "model", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "X8S2V10");
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.7 requestpos] requestpos get answers 0 by default") {
	auto rmd = makeRmd1();
	CHECK(getCmd(*rmd, "requestpos") == 0);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.7 requestpos] requestpos set 1 is read back and set 0 returns to 0") {
	auto rmd = makeRmd1();
	CHECK(setCmd(*rmd, "requestpos", 1) == CommandStatus::OK);
	CHECK(getCmd(*rmd, "requestpos") == 1);
	CHECK(setCmd(*rmd, "requestpos", 0) == CommandStatus::OK);
	CHECK(getCmd(*rmd, "requestpos") == 0);
}

// ---------------------------------------------------------------- R8.8 error statuses

TEST_CASE_FIXTURE(RmdFixture, "[R8.8 status] set on the get only command curr returns ERR") {
	auto rmd = makeRmd1();
	CHECK(setCmd(*rmd, "curr", 5) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.8 status] set on the get only command temp returns ERR") {
	auto rmd = makeRmd1();
	CHECK(setCmd(*rmd, "temp", 5) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.8 status] set on the get only command vbus returns ERR") {
	auto rmd = makeRmd1();
	CHECK(setCmd(*rmd, "vbus", 5) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.8 status] set on the get only command errors returns ERR and leaves the flags") {
	auto rmd = makeRmd1();
	peer1.setErrors(0x0004);
	peer1.sendReply(0x9A);
	CHECK(setCmd(*rmd, "errors", 0) == CommandStatus::ERR);
	CHECK(getCmd(*rmd, "errors") == 4);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.8 status] set on the get only command model returns ERR") {
	auto rmd = makeRmd1();
	CHECK(setCmd(*rmd, "model", 5) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.8 status] a getat on canid returns ERR") {
	auto rmd = makeRmd1();
	CHECK(typeCmd(*rmd, "canid", CMDtype::getat) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.8 status] a command of type none on maxtorque returns ERR" *
								  KNOWN_ISSUE("B-RMD-8: command() ignores the status of handleGetSet for maxtorque (the ERR for an unsupported type is dropped), the status is OK; the other commands return ERR")) {
	auto rmd = makeRmd1();
	CHECK(typeCmd(*rmd, "maxtorque", CMDtype::none) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(RmdFixture, "[R8.8 status] a command id outside the table returns NOT_FOUND") {
	auto rmd = makeRmd1();
	ParsedCommand parsed;
	parsed.cmdId = 99; // the enum has the ids 0..7
	parsed.type = CMDtype::get;
	parsed.target = rmd.get();
	std::vector<CommandReply> replies;
	CHECK(rmd->command(parsed, replies) == CommandStatus::NOT_FOUND);
	CHECK(replies.empty());
}

// ---------------------------------------------------------------- R9.1 saveFlash

TEST_CASE_FIXTURE(RmdFixture, "[R9.1 save] saveFlash of RmdMotorCAN1 with the defaults writes the words 0x0000, 0x03E8, 0x0000") {
	auto rmd = makeRmd1();
	rmd->saveFlash();
	CHECK(word(ADR_RMD1_DATA1) == 0x0000); // HD: (1 - 1) & 0x1F = 0, activerequests 0
	CHECK(word(ADR_RMD1_TORQUE) == 0x03E8); // HD: 1000
	CHECK(word(ADR_RMD1_OFFSET) == 0x0000); // HD: posOffset 0
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.1 save] saveFlash of RmdMotorCAN2 with the defaults writes the words 0x0001, 0x03E8, 0x0000 to its own addresses") {
	auto rmd = makeRmd2();
	rmd->saveFlash();
	CHECK(word(ADR_RMD2_DATA1) == 0x0001); // HD: (2 - 1) & 0x1F = 1
	CHECK(word(ADR_RMD2_TORQUE) == 0x03E8);
	CHECK(word(ADR_RMD2_OFFSET) == 0x0000);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.1 save] saveFlash with canid 32 writes word 1 = 0x001F") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "canid", 32) == CommandStatus::OK);
	rmd->saveFlash();
	CHECK(word(ADR_RMD1_DATA1) == 0x001F); // HD: (32 - 1) & 0x1F = 31
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.1 save] saveFlash with canid 5 writes word 1 = 0x0004") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "canid", 5) == CommandStatus::OK);
	rmd->saveFlash();
	CHECK(word(ADR_RMD1_DATA1) == 0x0004); // HD: 5 - 1 = 4
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.1 save] saveFlash with requestpos on writes bit 6 of word 1 = 0x0040") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "requestpos", 1) == CommandStatus::OK);
	rmd->saveFlash();
	CHECK(word(ADR_RMD1_DATA1) == 0x0040); // HD: id 1 -> 0, | 1 << 6
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.1 save] saveFlash with canid 32 and requestpos on writes word 1 = 0x005F") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "canid", 32) == CommandStatus::OK);
	REQUIRE(setCmd(*rmd, "requestpos", 1) == CommandStatus::OK);
	rmd->saveFlash();
	CHECK(word(ADR_RMD1_DATA1) == 0x005F); // HD: 0x1F | 0x40
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.1 save] saveFlash with requestpos switched on and off again writes bit 6 clear") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "requestpos", 1) == CommandStatus::OK);
	REQUIRE(setCmd(*rmd, "requestpos", 0) == CommandStatus::OK);
	rmd->saveFlash();
	CHECK(word(ADR_RMD1_DATA1) == 0x0000);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.1 save] saveFlash with maxtorque 1 writes word 2 = 0x0001") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "maxtorque", 1) == CommandStatus::OK);
	rmd->saveFlash();
	CHECK(word(ADR_RMD1_TORQUE) == 0x0001);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.1 save] saveFlash with maxtorque 32767 writes word 2 = 0x7FFF") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "maxtorque", 32767) == CommandStatus::OK);
	rmd->saveFlash();
	CHECK(word(ADR_RMD1_TORQUE) == 0x7FFF); // HD: 32767 & 0x7FFF
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.1 save] saveFlash with a position offset of 1234 writes word 3 = 1234") {
	auto rmd = makeRmd1();
	peer1.setMultiTurn(REPORT_POS);
	peer1.sendReply(0x92);
	rmd->setPos(98766); // HD: posOffset = 100000 - 98766 = 1234
	rmd->saveFlash();
	CHECK(word(ADR_RMD1_OFFSET) == 1234); // 1234 % 36000 = 1234
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.1 save] saveFlash reduces a position offset of 40000 modulo 36000 to 4000") {
	auto rmd = makeRmd1();
	peer1.setMultiTurn(REPORT_POS);
	peer1.sendReply(0x92);
	rmd->setPos(60000); // HD: posOffset = 100000 - 60000 = 40000
	rmd->saveFlash();
	CHECK(word(ADR_RMD1_OFFSET) == 4000); // HD: 40000 % 36000 = 4000
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.1 save] saveFlash writes a position offset of -1 as 0xFFFF") {
	auto rmd = makeRmd1();
	peer1.setMultiTurn(REPORT_POS);
	peer1.sendReply(0x92);
	rmd->setPos(100001); // HD: posOffset = 100000 - 100001 = -1
	rmd->saveFlash();
	CHECK(word(ADR_RMD1_OFFSET) == 0xFFFF); // HD: -1 % -36000 = -1 -> uint16_t 65535
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.1 save] saveFlash reduces a position offset of -40000 to -4000, stored as 0xF060") {
	auto rmd = makeRmd1();
	peer1.setMultiTurn(REPORT_POS);
	peer1.sendReply(0x92);
	rmd->setPos(140000); // HD: posOffset = 100000 - 140000 = -40000
	rmd->saveFlash();
	CHECK(word(ADR_RMD1_OFFSET) == 0xF060); // HD: -40000 % -36000 = -4000 -> 65536 - 4000 = 61536 = 0xF060
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.1 save] characterization: saveFlash masks a maxtorque of 40000 to 15 bits, word 2 = 7232") {
	auto rmd = makeRmd1();
	REQUIRE(setCmd(*rmd, "maxtorque", 40000) == CommandStatus::OK); // accepted today (B-RMD-3)
	rmd->saveFlash();
	CHECK(word(ADR_RMD1_TORQUE) == 7232); // HD: 40000 = 0x9C40, & 0x7FFF = 0x1C40 = 7232
}

// ---------------------------------------------------------------- R9.5 own addresses per instance

TEST_CASE_FIXTURE(RmdFixture, "[R9.5 addresses] saveFlash of instance 1 writes nothing to the addresses of instance 2") {
	auto rmd = makeRmd1();
	rmd->saveFlash();
	CHECK(hasWord(ADR_RMD1_DATA1));
	CHECK_FALSE(hasWord(ADR_RMD2_DATA1));
	CHECK_FALSE(hasWord(ADR_RMD2_TORQUE));
	CHECK_FALSE(hasWord(ADR_RMD2_OFFSET));
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.5 addresses] saveFlash of instance 2 writes nothing to the addresses of instance 1") {
	auto rmd = makeRmd2();
	rmd->saveFlash();
	CHECK(hasWord(ADR_RMD2_DATA1));
	CHECK_FALSE(hasWord(ADR_RMD1_DATA1));
	CHECK_FALSE(hasWord(ADR_RMD1_TORQUE));
	CHECK_FALSE(hasWord(ADR_RMD1_OFFSET));
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.5 addresses] two instances with different settings save their own words") {
	auto r1 = makeRmd1();
	auto r2 = makeRmd2();
	REQUIRE(setCmd(*r1, "canid", 7) == CommandStatus::OK);
	REQUIRE(setCmd(*r1, "maxtorque", 111) == CommandStatus::OK);
	REQUIRE(setCmd(*r2, "canid", 9) == CommandStatus::OK);
	REQUIRE(setCmd(*r2, "maxtorque", 222) == CommandStatus::OK);
	REQUIRE(setCmd(*r2, "requestpos", 1) == CommandStatus::OK);
	r1->saveFlash();
	r2->saveFlash();
	CHECK(word(ADR_RMD1_DATA1) == 6); // HD: 7 - 1
	CHECK(word(ADR_RMD1_TORQUE) == 111);
	CHECK(word(ADR_RMD2_DATA1) == (8 | 0x40)); // HD: (9 - 1) | 1 << 6 = 0x48
	CHECK(word(ADR_RMD2_TORQUE) == 222);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.5 addresses] the constructors restore each instance from its own words") {
	setWord(ADR_RMD1_DATA1, 6);	 // id 7
	setWord(ADR_RMD1_TORQUE, 111);
	setWord(ADR_RMD2_DATA1, 8);	 // id 9
	setWord(ADR_RMD2_TORQUE, 222);
	auto r1 = makeRmd1();
	auto r2 = makeRmd2();
	CHECK(getCmd(*r1, "canid") == 7);
	CHECK(getCmd(*r1, "maxtorque") == 111);
	CHECK(getCmd(*r2, "canid") == 9);
	CHECK(getCmd(*r2, "maxtorque") == 222);
}

// ---------------------------------------------------------------- R9.2 restore

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] the constructor restores the id from word 1: 0x0005 gives canid 6, filter 0x246 and frame id 0x146") {
	setWord(ADR_RMD1_DATA1, 0x0005); // HD: (5 & 0x1F) + 1 = 6
	auto rmd = makeRmd1();
	CHECK(getCmd(*rmd, "canid") == 6);
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x246); // HD: 0x240 + 6
	clearTxFrames();
	rmd->setTorque(0);
	CHECK(txFrames(0x146).size() == 1); // HD: 0x140 + 6
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] word 1 = 0x001F gives canid 32, filter 0x260 and frame id 0x160") {
	setWord(ADR_RMD1_DATA1, 0x001F); // HD: (31 & 0x1F) + 1 = 32
	auto rmd = makeRmd1();
	CHECK(getCmd(*rmd, "canid") == 32);
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x260);
	clearTxFrames();
	rmd->setTorque(0);
	CHECK(txFrames(0x160).size() == 1);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] word 1 = 0x0010 gives canid 17 (bit 4 is part of the id) and filter 0x251") {
	setWord(ADR_RMD1_DATA1, 0x0010); // HD: (16 & 0x1F) + 1 = 17
	auto rmd = makeRmd1();
	CHECK(getCmd(*rmd, "canid") == 17);
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x251);
	CHECK(getCmd(*rmd, "requestpos") == 0);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] characterization: restoreFlash() after construction changes the id but does not rebuild the reply filter") {
	auto rmd = makeRmd1();
	setWord(ADR_RMD1_DATA1, 0x0004); // id 5
	rmd->restoreFlash();
	CHECK(getCmd(*rmd, "canid") == 5);
	clearTxFrames();
	rmd->setTorque(0);
	CHECK(txFrames(0x145).size() == 1); // the frames follow the new id
	REQUIRE(HostTarget::canPort().filters.size() == 1); // setCanFilter is not called: the filter stays 0x241
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x241);
	CHECK(HostTarget::canPort().filters[0].active);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] the constructor restores the request mode from bit 6 of word 1") {
	setWord(ADR_RMD1_DATA1, 0x0040);
	auto rmd = makeRmd1();
	CHECK(getCmd(*rmd, "requestpos") == 1);
	CHECK(getCmd(*rmd, "canid") == 1); // HD: (0x40 & 0x1F) + 1
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] the bits 5 and 7-15 of word 1 are ignored: 0xFFA0 gives canid 1 and requestpos 0") {
	// HD: 0xFFA0 = 1111 1111 1010 0000: bits 0-4 = 0 -> id 1, bit 6 = 0 -> request mode off, bits 5 and 7-15 set
	setWord(ADR_RMD1_DATA1, 0xFFA0);
	auto rmd = makeRmd1();
	CHECK(getCmd(*rmd, "canid") == 1);
	CHECK(getCmd(*rmd, "requestpos") == 0);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] the bits 5 and 7-15 of word 1 do not hide bit 6: 0xFFE0 gives requestpos 1") {
	setWord(ADR_RMD1_DATA1, 0xFFE0); // HD: bit 6 set, bits 0-4 = 0
	auto rmd = makeRmd1();
	CHECK(getCmd(*rmd, "canid") == 1);
	CHECK(getCmd(*rmd, "requestpos") == 1);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] the constructor restores maxtorque from word 2") {
	setWord(ADR_RMD1_TORQUE, 500);
	auto rmd = makeRmd1();
	CHECK(getCmd(*rmd, "maxtorque") == 500);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] bit 15 of word 2 is ignored: 0xFFFF gives maxtorque 32767") {
	setWord(ADR_RMD1_TORQUE, 0xFFFF);
	auto rmd = makeRmd1();
	CHECK(getCmd(*rmd, "maxtorque") == 0x7FFF); // HD: 0xFFFF & 0x7FFF
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] the constructor restores the position offset from word 3: 1234 gives getPos -1234 before any report") {
	setWord(ADR_RMD1_OFFSET, 1234);
	auto rmd = makeRmd1();
	CHECK(rmd->getPos() == -1234); // HD: lastPos 0 -> (int32)(0 * 100) - 1234
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] a stored offset 0xFFFF is restored as -1 (getPos 1 before any report)") {
	setWord(ADR_RMD1_OFFSET, 0xFFFF);
	auto rmd = makeRmd1();
	CHECK(rmd->getPos() == 1); // HD: (int16_t)0xFFFF = -1; 0 - (-1) = 1
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] only word 2 present: id, request mode and offset keep their defaults") {
	setWord(ADR_RMD1_TORQUE, 500);
	auto rmd = makeRmd1();
	CHECK(getCmd(*rmd, "canid") == 1);
	CHECK(getCmd(*rmd, "requestpos") == 0);
	CHECK(rmd->getPos() == 0);
	CHECK(getCmd(*rmd, "maxtorque") == 500);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] only word 1 present: maxtorque keeps the default 1000 and the offset 0") {
	setWord(ADR_RMD1_DATA1, 0x0002); // id 3
	auto rmd = makeRmd1();
	CHECK(getCmd(*rmd, "canid") == 3);
	CHECK(getCmd(*rmd, "maxtorque") == 1000);
	CHECK(rmd->getPos() == 0);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] only word 3 present: id and maxtorque keep their defaults") {
	setWord(ADR_RMD2_OFFSET, 77);
	auto rmd = makeRmd2();
	CHECK(getCmd(*rmd, "canid") == 2); // the default of instance 2
	CHECK(getCmd(*rmd, "maxtorque") == 1000);
	CHECK(rmd->getPos() == -77);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] restoreFlash() called later takes the words written since construction (torque, request mode, offset)") {
	auto rmd = makeRmd1();
	setWord(ADR_RMD1_DATA1, 0x0040);
	setWord(ADR_RMD1_TORQUE, 321);
	setWord(ADR_RMD1_OFFSET, 10);
	rmd->restoreFlash();
	CHECK(getCmd(*rmd, "maxtorque") == 321);
	CHECK(getCmd(*rmd, "requestpos") == 1);
	CHECK(rmd->getPos() == -10);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.2 restore] a driver constructed from the words of saveFlash has the same id, request mode and maxtorque") {
	{
		auto rmd = makeRmd1();
		REQUIRE(setCmd(*rmd, "canid", 12) == CommandStatus::OK);
		REQUIRE(setCmd(*rmd, "maxtorque", 4321) == CommandStatus::OK);
		REQUIRE(setCmd(*rmd, "requestpos", 1) == CommandStatus::OK);
		rmd->saveFlash();
	}
	auto again = makeRmd1();
	CHECK(getCmd(*again, "canid") == 12);
	CHECK(getCmd(*again, "maxtorque") == 4321);
	CHECK(getCmd(*again, "requestpos") == 1);
}

// ---------------------------------------------------------------- R9.3 / R9.4 offset round trips

namespace {
/**
 * Sets the position to `p` while the motor reports 1000.00 degrees, saves, destroys the driver, constructs a new one,
 * lets the motor report the same position and returns getPos() of the new driver (the round trip of the offset).
 */
int32_t offsetRoundTrip(RmdFixture &fx, int32_t p) {
	{
		auto rmd = fx.makeRmd1();
		fx.peer1.setMultiTurn(REPORT_POS);
		fx.peer1.sendReply(0x92);
		rmd->setPos(p);
		REQUIRE(rmd->getPos() == p);
		rmd->saveFlash();
	}
	auto again = fx.makeRmd1();
	fx.peer1.sendReply(0x92);
	return again->getPos();
}
} // namespace

TEST_CASE_FIXTURE(RmdFixture, "[R9.4 offset] a position offset of +1234 survives save and restore") {
	// HD: p = 98766 -> offset 1234 -> word 1234 -> restored 1234 -> getPos 100000 - 1234 = 98766
	CHECK(offsetRoundTrip(*this, 98766) == 98766);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.4 offset] a position offset of -1 survives save and restore") {
	// HD: p = 100001 -> offset -1 -> word 0xFFFF -> (int16_t) -1 -> getPos 100000 + 1 = 100001
	CHECK(offsetRoundTrip(*this, 100001) == 100001);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.4 offset] a position offset of -1234 survives save and restore") {
	// HD: p = 101234 -> offset -1234 -> word 64302 -> (int16_t) -1234 -> getPos 101234
	CHECK(offsetRoundTrip(*this, 101234) == 101234);
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.3 offset] a position offset of 35000 survives save and restore" *
								  KNOWN_ISSUE("B-RMD-5: the word 35000 is restored as (int16_t) = -30536, getPos gives 130536 instead of 65000")) {
	// HD: p = 65000 -> offset 100000 - 65000 = 35000 (below 36000 counts per turn, SP unit 0.01 degree) -> word 35000
	CHECK(offsetRoundTrip(*this, 65000) == 65000); // first failing assertion: 100000 - (-30536) = 130536
}

TEST_CASE_FIXTURE(RmdFixture, "[R9.4 offset] a position offset of -35999 survives save and restore" *
								  KNOWN_ISSUE("B-RMD-5: -35999 % -36000 = -35999 is stored as uint16_t 29537 and restored as +29537, getPos gives 70463 instead of 135999; the charter lists this case as plain (lead decision needed)")) {
	// HD: p = 135999 -> offset 100000 - 135999 = -35999 -> word (uint16_t)(-35999) = 65536 - 35999 = 29537 -> restored +29537
	CHECK(offsetRoundTrip(*this, 135999) == 135999); // first failing assertion: 100000 - 29537 = 70463
}

} // TEST_SUITE
