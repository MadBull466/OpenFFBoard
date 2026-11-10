/*
 * MyActuator RMD driver (RmdMotorCAN), work order RMS (phase P16 T2): identity, chooser, construction, life cycle.
 * Requirements R1.1-R1.4, R2.1-R2.4 and R10.1-R10.4 of Tests/claude/charters/driver/RMD_REQUIREMENTS.md (R2.5 and the
 * seven cases at the top of the suite come from the T1 skeleton and stay unchanged).
 *
 * Unit under test: RmdMotorCAN.cpp (constructor / destructor, setCanFilter, getInfo / isCreatable, command table), the
 * `add_class<RmdMotorCAN1/2, MotorDriver>(11 / 12)` entries of MotorDriver::all_drivers, Axis::axis1_drivers and
 * Axis::axis2_drivers, and the real ClassChooser, CANPort_2B and fake bxCAN filter banks.
 *
 * Oracles: SP (reply id 0x240 + ID, section 1.1.2; 0.01 degree unit for the cpr 36000), HD (derivations in comments:
 * flash word 1 = (nodeId - 1) & 0x1F, so id 5 = word 4 -> filter 0x240 + 5 = 0x245), IF (Encoder / MotorDriver
 * interface), FW (identity facts, command flags, preset floor; the preset floor is a characterization).
 *
 * Host vs target: nothing here depends on type sizes, packing or endianness. The filter id math is integer.
 *
 * Characterizations of this file (NEEDS REVIEW): the three preset cases (the floor 3 = 250 kbit/s is firmware only, the
 * SP says 1 Mbit/s), and the T1 cases R2.5 / R4.1. KNOWN_ISSUE: B-RMD-1 (R10.2, the port user is never given back).
 * Firmware oddity (not tested): the constructor passes CLSID_MOT_RMD1 to CommandHandler for both instances.
 * Not tested here: the thread timing (RMF), the commands' behaviour (RMC), the flash words written (RMC).
 */
#include "doctest.h"
#include "RmdFixture.h"

#include "Axis.h"
#include "CANPort2B.h"
#include "ClassChooser.h"
#include "ClassIDs.h"
#include "MotorDriver.h"
#include "eeprom_addresses.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr uint16_t kSelRmd1 = 11; // MotorDriver.cpp:51
constexpr uint16_t kSelRmd2 = 12; // MotorDriver.cpp:52

bool canSilentPinHigh() { return HostTarget::outputPin(CAN_S_GPIO_Port, CAN_S_Pin); }

void setWord(uint16_t adr, uint16_t value) { HostPlatform::flash()[adr] = value; }

/** The entries of a driver list with the given selection id */
std::vector<const class_entry<MotorDriver> *> entriesWithId(const std::vector<class_entry<MotorDriver>> &list, uint16_t id) {
	std::vector<const class_entry<MotorDriver> *> found;
	for (const auto &e : list) {
		if (e.selectionId == id) {
			found.push_back(&e);
		}
	}
	return found;
}

uint32_t commandFlags(CommandHandler &h, const char *name) {
	CmdHandlerCommanddef *def = h.getCommandFromName(name);
	REQUIRE_MESSAGE(def != nullptr, "command " << name << " not registered");
	return def->flags;
}

} // namespace

TEST_SUITE("RMD setup") {

TEST_CASE_FIXTURE(RmdFixture, "[R2.1] constructing RmdMotorCAN1 registers the reply filter 0x241 with mask 0x3FF and starts the CAN port") {
	auto rmd = makeRmd1();
	CHECK(HostTarget::canPort().started);
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x241); // SP 1.1.2: reply id 0x240 + ID, ID 1
	CHECK(HostTarget::canPort().filters[0].filter_mask == 0x3FF);
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.1] constructing RmdMotorCAN2 registers the reply filter 0x242 for node 2") {
	auto rmd = makeRmd2();
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x242); // ID 2
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.5] characterization: the thread sends nothing before 100 ms and the 0xB5 model request at 100 ms to id 0x141 with DLC 8") {
	auto rmd = makeRmd1();
	runFor(99);
	CHECK(HostTarget::canTxFrames().empty());
	runFor(1);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	const auto &f = HostTarget::canTxFrames()[0];
	CHECK(f.id == 0x141); // SP 1.1.2: 0x140 + ID 1
	CHECK_FALSE(f.extId);
	CHECK_FALSE(f.rtr);
	CHECK(f.dlc == 8);
	CHECK(f.data[0] == 0xB5); // SP 2.33
	CHECK(f.time == 100);	  // characterization: the 100 ms start delay is firmware behaviour
	CHECK(peer1.violations().empty());
}

TEST_CASE_FIXTURE(RmdFixture, "[R4.1] characterization: the thread runs at 100 ms, 120 ms (0xB6 enable of 0x92) and the 0xB5 again at 1270 ms against a silent motor (fixture scheduling of every Delay)") {
	auto rmd = makeRmd1();
	peer1.setSilent(true);
	runFor(1271);
	const auto b5 = txFramesCmd(0x141, 0xB5);
	REQUIRE(b5.size() == 2);
	CHECK(b5[0].time == 100);
	CHECK(b5[1].time == 1270); // 100 + 20 + 150 + 1000: the second cycle starts 1170 ms after the first (characterization of the constants)
	const auto b6 = txFramesCmd(0x141, 0xB6);
	REQUIRE(b6.size() == 1); // the next 0xB6 would be at 1290 ms, outside the 1271 ms window
	CHECK(b6[0].time == 120);
	CHECK(b6[0].data[1] == 0x92);
	CHECK(b6[0].data[2] == 1);
	CHECK(peer1.violations().empty());
}

TEST_CASE_FIXTURE(RmdFixture, "[R4.2] a motor that answers makes motorReady true at the end of the first cycle and registers the cyclic position report") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	CHECK(rmd->motorReady());
	CHECK(peer1.cyclicEnabled(0x92)); // the firmware's 0xB6 enable was accepted
	CHECK(peer1.violations().empty());
}

TEST_CASE_FIXTURE(RmdFixture, "[R10.4] RmdMotorCAN2 receives the replies of node 2 on FIFO 1 and reaches motorReady against peer 2") {
	auto rmd = makeRmd2();
	reachAvailable(*rmd);
	CHECK(rmd->motorReady());
	CHECK(txFramesCmd(0x142, 0xB5).size() == 1);
	CHECK(peer2.violations().empty());
}

TEST_CASE_FIXTURE(RmdFixture, "[R5.5] the 0xB5 reply of the motor is read back through the model command") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	// SP 2.33 example: 0x58 0x38 0x53 0x32 0x56 0x31 0x30 = "X8S2V10" (the peer's default model)
	std::vector<CommandReply> replies;
	REQUIRE(CommandHelpers::run(*rmd, "model", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "X8S2V10");
}

// ---------------------------------------------------------------- R1.1 identity

TEST_CASE_FIXTURE(RmdFixture, "[R1.1] RmdMotorCAN1 reports the name RMD MyActuator (1) and class id CLSID_MOT_RMD1") {
	auto rmd = makeRmd1();
	const ClassIdentifier id = rmd->getInfo();
	REQUIRE(id.name != nullptr);
	CHECK(std::string(id.name) == "RMD MyActuator (1)");
	CHECK(id.id == CLSID_MOT_RMD1);
	CHECK(RmdMotorCAN1::info.id == CLSID_MOT_RMD1);
}

TEST_CASE_FIXTURE(RmdFixture, "[R1.1] RmdMotorCAN2 reports the name RMD MyActuator (2) and class id CLSID_MOT_RMD2") {
	auto rmd = makeRmd2();
	const ClassIdentifier id = rmd->getInfo();
	REQUIRE(id.name != nullptr);
	CHECK(std::string(id.name) == "RMD MyActuator (2)");
	CHECK(id.id == CLSID_MOT_RMD2);
	CHECK(RmdMotorCAN2::info.id == CLSID_MOT_RMD2);
}

TEST_CASE_FIXTURE(RmdFixture, "[R1.1] the driver is its own absolute encoder with an integrated encoder") {
	auto rmd = makeRmd1();
	CHECK(rmd->hasIntegratedEncoder());
	CHECK(rmd->getEncoder() == static_cast<Encoder *>(rmd.get())); // IF: the driver returns itself
	CHECK(rmd->getEncoderType() == EncoderType::absolute);
}

TEST_CASE_FIXTURE(RmdFixture, "[R1.1] the counts per revolution are 36000 (SP: 0.01 degree unit, 360.00 degrees)") {
	auto rmd = makeRmd2();
	CHECK(rmd->getCpr() == 36000);
}

// ---------------------------------------------------------------- R1.2 chooser lists

TEST_CASE_FIXTURE(RmdFixture, "[R1.2] MotorDriver::all_drivers holds the ids 11 and 12 exactly once, with the RMD identities") {
	auto e11 = entriesWithId(MotorDriver::all_drivers, kSelRmd1);
	auto e12 = entriesWithId(MotorDriver::all_drivers, kSelRmd2);
	REQUIRE(e11.size() == 1);
	REQUIRE(e12.size() == 1);
	CHECK(&e11[0]->info == &RmdMotorCAN1::info);
	CHECK(&e12[0]->info == &RmdMotorCAN2::info);
}

TEST_CASE_FIXTURE(RmdFixture, "[R1.2] axis 1 lists the id 11 only and axis 2 the id 12 only") {
	CHECK(entriesWithId(Axis::axis1_drivers, kSelRmd1).size() == 1);
	CHECK(entriesWithId(Axis::axis1_drivers, kSelRmd2).empty());
	CHECK(entriesWithId(Axis::axis2_drivers, kSelRmd2).size() == 1);
	CHECK(entriesWithId(Axis::axis2_drivers, kSelRmd1).empty());
}

TEST_CASE_FIXTURE(RmdFixture, "[R1.2] creating the id 11 through the chooser builds a RmdMotorCAN1 with its filter and commands") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	std::unique_ptr<MotorDriver> drv(chooser.Create(kSelRmd1));
	REQUIRE(drv != nullptr);
	auto *rmd = dynamic_cast<RmdMotorCAN1 *>(drv.get());
	REQUIRE(rmd != nullptr);
	CHECK(drv->getSelectionID() == kSelRmd1);
	CHECK(HostTarget::canAccepts(0x241));
	CHECK(CommandHelpers::get(*rmd, "canid") == 1);
}

TEST_CASE_FIXTURE(RmdFixture, "[R1.2] creating the id 12 through the chooser builds a RmdMotorCAN2 with its filter and commands") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	std::unique_ptr<MotorDriver> drv(chooser.Create(kSelRmd2));
	REQUIRE(drv != nullptr);
	auto *rmd = dynamic_cast<RmdMotorCAN2 *>(drv.get());
	REQUIRE(rmd != nullptr);
	CHECK(drv->getSelectionID() == kSelRmd2);
	CHECK(HostTarget::canAccepts(0x242));
	CHECK(CommandHelpers::get(*rmd, "canid") == 2);
}

// ---------------------------------------------------------------- R1.3 creatable

TEST_CASE_FIXTURE(RmdFixture, "[R1.3] RmdMotorCAN1 is creatable without an object, not while one exists and again after its destruction") {
	CHECK(RmdMotorCAN1::isCreatable());
	auto rmd = makeRmd1();
	CHECK_FALSE(RmdMotorCAN1::isCreatable());
	rmd.reset();
	CHECK(RmdMotorCAN1::isCreatable());
}

TEST_CASE_FIXTURE(RmdFixture, "[R1.3] RmdMotorCAN2 is creatable without an object, not while one exists and again after its destruction") {
	CHECK(RmdMotorCAN2::isCreatable());
	auto rmd = makeRmd2();
	CHECK_FALSE(RmdMotorCAN2::isCreatable());
	rmd.reset();
	CHECK(RmdMotorCAN2::isCreatable());
}

TEST_CASE_FIXTURE(RmdFixture, "[R1.3] instance 2 stays creatable while instance 1 exists") {
	auto rmd1 = makeRmd1();
	CHECK(RmdMotorCAN2::isCreatable());
	CHECK_FALSE(RmdMotorCAN1::isCreatable());
}

TEST_CASE_FIXTURE(RmdFixture, "[R1.3] a second chooser creation of the same instance returns no object") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	std::unique_ptr<MotorDriver> first(chooser.Create(kSelRmd1));
	REQUIRE(first != nullptr);
	CHECK(chooser.isCreatable(kSelRmd2));
	CHECK_FALSE(chooser.isCreatable(kSelRmd1));
	MotorDriver *second = chooser.Create(kSelRmd1);
	CHECK(second == nullptr);
	delete second; // nothing to delete if the chooser behaves
	CHECK(RmdMotorCAN1::inUse);
}

// ---------------------------------------------------------------- R1.4 defaults

TEST_CASE_FIXTURE(RmdFixture, "[R1.4] without flash instance 1 has CAN id 1 and instance 2 has CAN id 2") {
	auto rmd1 = makeRmd1();
	auto rmd2 = makeRmd2();
	CHECK(CommandHelpers::get(*rmd1, "canid") == 1); // SP: ID 1..32
	CHECK(CommandHelpers::get(*rmd2, "canid") == 2);
}

TEST_CASE_FIXTURE(RmdFixture, "[R1.4] without flash the numeric defaults are maxtorque 1000, requestpos 0 and zero readings") {
	auto rmd = makeRmd1();
	CHECK(CommandHelpers::get(*rmd, "maxtorque") == 1000);
	CHECK(CommandHelpers::get(*rmd, "requestpos") == 0);
	CHECK(CommandHelpers::get(*rmd, "errors") == 0);
	CHECK(CommandHelpers::get(*rmd, "curr") == 0);
	CHECK(CommandHelpers::get(*rmd, "temp") == 0);
	CHECK(CommandHelpers::get(*rmd, "vbus") == 0);
}

TEST_CASE_FIXTURE(RmdFixture, "[R1.4] without flash the model name is empty and the motor is not ready") {
	auto rmd = makeRmd1();
	std::vector<CommandReply> replies;
	REQUIRE(CommandHelpers::run(*rmd, "model", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply.empty());
	CHECK_FALSE(rmd->motorReady());
}

// ---------------------------------------------------------------- R2 construction

TEST_CASE_FIXTURE(RmdFixture, "[R2.1] RmdMotorCAN1 registers its filter on receive buffer 0 and RmdMotorCAN2 on buffer 1") {
	auto rmd1 = makeRmd1();
	auto rmd2 = makeRmd2();
	REQUIRE(HostTarget::canPort().filters.size() == 2);
	CHECK(HostTarget::canPort().filters[0].buffer == 0);
	CHECK(HostTarget::canPort().filters[1].buffer == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x241);
	CHECK(HostTarget::canPort().filters[1].filter_id == 0x242);
	CHECK(HostTarget::canPort().filters[0].filter_mask == 0x3FF);
	CHECK(HostTarget::canPort().filters[1].filter_mask == 0x3FF);
	CHECK_FALSE(HostTarget::canPort().filters[0].extid);
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.1] the hardware filter of RmdMotorCAN1 passes the reply id 0x241 to FIFO 0 and rejects other motors and command ids") {
	auto rmd = makeRmd1();
	uint32_t fifo = 99;
	CHECK(HostTarget::canAccepts(0x241, false, false, &fifo));
	CHECK(fifo == 0);
	CHECK_FALSE(HostTarget::canAccepts(0x242));
	CHECK_FALSE(HostTarget::canAccepts(0x141)); // the id of the driver's own command frames
	CHECK_FALSE(HostTarget::canAccepts(0x241, true)); // extended frames
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.1] changing the id with the canid command replaces the filter") {
	auto rmd = makeRmd1();
	REQUIRE(CommandHelpers::run(*rmd, "canid", CMDtype::set, 5) == CommandStatus::OK);
	CHECK(HostTarget::canActiveFilterBanks() == 1); // the old one is gone
	CHECK(HostTarget::canAccepts(0x245)); // SP: reply id 0x240 + 5
	CHECK_FALSE(HostTarget::canAccepts(0x241));
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.2] characterization: a CAN port below the 250 kbit/s preset is raised to preset 3") {
	// FW only: the code comment says "Minimum 250k"; the SP names 1 Mbit/s (owner question 50 area)
	HostTarget::canPort2B().setSpeedPreset(1);
	auto rmd = makeRmd1();
	CHECK(HostTarget::canPort2B().getSpeedPreset() == 3);
	CHECK(HostTarget::canPort2B().getSpeed() == 250000);
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.2] characterization: a CAN port at preset 3 stays at preset 3") {
	HostTarget::canPort2B().setSpeedPreset(3);
	auto rmd = makeRmd1();
	CHECK(HostTarget::canPort2B().getSpeedPreset() == 3);
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.2] characterization: a CAN port at preset 4 (500 kbit/s) stays at preset 4") {
	HostTarget::canPort2B().setSpeedPreset(4);
	auto rmd = makeRmd1();
	CHECK(HostTarget::canPort2B().getSpeedPreset() == 4);
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.2] characterization: a CAN port above the 250 kbit/s preset (preset 5) is not lowered") {
	HostTarget::canPort2B().setSpeedPreset(5);
	auto rmd = makeRmd1();
	CHECK(HostTarget::canPort2B().getSpeedPreset() == 5);
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.2] construction takes the transceiver out of silent mode") {
	// Note: CANPort_2B::start() also calls setSilentMode(false), so this proves the end state (pin low after
	// construction), not the explicit setSilentMode(false) call of the driver constructor.
	HostTarget::canPort2B().setSilentMode(true);
	REQUIRE(canSilentPinHigh());
	auto rmd = makeRmd1();
	CHECK_FALSE(canSilentPinHigh());
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.2] the driver takes one user of the CAN port and the port is started") {
	const int32_t before = HostTarget::canPort2B().getPortUsers();
	auto rmd = makeRmd1();
	CHECK(HostTarget::canPort2B().getPortUsers() == before + 1);
	CHECK(HostTarget::canPort().started);
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.3] the eight commands are registered with their access flags") {
	auto rmd = makeRmd1();
	CHECK(commandFlags(*rmd, "canid") == (CMDFLAG_GET | CMDFLAG_SET));
	CHECK(commandFlags(*rmd, "errors") == CMDFLAG_GET);
	CHECK(commandFlags(*rmd, "maxtorque") == (CMDFLAG_GET | CMDFLAG_SET));
	CHECK(commandFlags(*rmd, "curr") == CMDFLAG_GET);
	CHECK(commandFlags(*rmd, "temp") == CMDFLAG_GET);
	CHECK(commandFlags(*rmd, "vbus") == CMDFLAG_GET);
	CHECK(commandFlags(*rmd, "requestpos") == (CMDFLAG_GET | CMDFLAG_SET));
	CHECK(commandFlags(*rmd, "model") == (CMDFLAG_GET | CMDFLAG_STR_ONLY));
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.4] the CAN id restored from flash is used for the filter (word 4 -> id 5 -> filter 0x245)") {
	// HD: word 1 = (nodeId - 1) & 0x1F = 4 for node 5; filter id = 0x240 + 5 = 0x245
	setWord(ADR_RMD1_DATA1, 4);
	auto rmd = makeRmd1();
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x245);
	CHECK(CommandHelpers::get(*rmd, "canid") == 5);
	CHECK_FALSE(HostTarget::canAccepts(0x241));
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.4] flash word 0xFFBF restores id 32 and ignores the unused bits 5 and 7-15 (filter 0x260)") {
	// HD: 0xFFA0 | 31 = 0xFFBF; bits 0-4 = 31 -> id 32; bit 6 (request mode) = 0; 0x240 + 32 = 0x260
	setWord(ADR_RMD1_DATA1, 0xFFA0 | 31);
	auto rmd = makeRmd1();
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x260);
	CHECK(CommandHelpers::get(*rmd, "canid") == 32);
	CHECK(CommandHelpers::get(*rmd, "requestpos") == 0);
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.4] flash word 0x0010 restores id 17 (filter 0x251)") {
	// HD: bits 0-4 = 16 -> id 17; 0x240 + 17 = 0x251
	setWord(ADR_RMD1_DATA1, 0x0010);
	auto rmd = makeRmd1();
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x251);
	CHECK(CommandHelpers::get(*rmd, "canid") == 17);
}

TEST_CASE_FIXTURE(RmdFixture, "[R2.4] RmdMotorCAN2 restores its id from its own flash word (word 9 -> id 10 -> filter 0x24A)") {
	// HD: (10 - 1) = 9; 0x240 + 10 = 0x24A
	setWord(ADR_RMD2_DATA1, 9);
	auto rmd = makeRmd2();
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].filter_id == 0x24A);
	CHECK(HostTarget::canPort().filters[0].buffer == 1);
}

// ---------------------------------------------------------------- R10 life cycle

TEST_CASE_FIXTURE(RmdFixture, "[R10.1] destruction sends the zero torque frame then the 0x80 shutdown to the node and nothing else") {
	auto rmd = makeRmd1();
	clearTxFrames();
	rmd.reset();
	process();
	const auto frames = txFrames(0x141);
	REQUIRE(frames.size() == 2);
	CHECK(HostTarget::canTxFrames().size() == 2);
	// SP 2.19: {0xA1, 0, 0, 0, iq lo, iq hi, 0, 0} with iq 0; SP 2.17: {0x80, 0 x 7}
	const std::array<uint8_t, 8> torque0{0xA1, 0, 0, 0, 0, 0, 0, 0};
	const std::array<uint8_t, 8> off{0x80, 0, 0, 0, 0, 0, 0, 0};
	CHECK(frames[0].dlc == 8);
	CHECK(std::equal(torque0.begin(), torque0.end(), frames[0].data.begin()));
	CHECK(frames[1].dlc == 8);
	CHECK(std::equal(off.begin(), off.end(), frames[1].data.begin()));
	CHECK(peer1.violations().empty());
}

TEST_CASE_FIXTURE(RmdFixture, "[R10.1] destruction removes the reply filter") {
	auto rmd = makeRmd1();
	REQUIRE(HostTarget::canActiveFilterBanks() == 1);
	rmd.reset();
	CHECK(HostTarget::canActiveFilterBanks() == 0);
	CHECK_FALSE(HostTarget::canAccepts(0x241));
}

TEST_CASE_FIXTURE(RmdFixture, "[R10.1] the in-use flag of each instance is set while it exists and cleared at destruction") {
	CHECK_FALSE(RmdMotorCAN1::inUse);
	CHECK_FALSE(RmdMotorCAN2::inUse);
	auto rmd1 = makeRmd1();
	CHECK(RmdMotorCAN1::inUse);
	CHECK_FALSE(RmdMotorCAN2::inUse);
	auto rmd2 = makeRmd2();
	CHECK(RmdMotorCAN2::inUse);
	rmd1.reset();
	CHECK_FALSE(RmdMotorCAN1::inUse);
	CHECK(RmdMotorCAN2::inUse);
	rmd2.reset();
	CHECK_FALSE(RmdMotorCAN2::inUse);
}

TEST_CASE_FIXTURE(RmdFixture, "[R10.2] destruction gives the CAN port user back" * KNOWN_ISSUE("B-RMD-1: ~RmdMotorCAN never calls port->freePort() although the constructor calls takePort(); ODriveCAN and VescCAN do")) {
	const int32_t before = HostTarget::canPort2B().getPortUsers();
	auto rmd = makeRmd1();
	rmd.reset();
	// intended: the user count is back at `before` (0); measured: before + 1 (the port stays started)
	CHECK(HostTarget::canPort2B().getPortUsers() == before);
	CHECK_FALSE(HostTarget::canPort().started);
}

TEST_CASE_FIXTURE(RmdFixture, "[R10.3] create, destroy, create again leaves one filter and a set in-use flag") {
	auto rmd = makeRmd1();
	rmd.reset();
	REQUIRE(HostTarget::canActiveFilterBanks() == 0);
	rmd = makeRmd1();
	CHECK(RmdMotorCAN1::inUse);
	CHECK_FALSE(RmdMotorCAN1::isCreatable());
	CHECK(HostTarget::canActiveFilterBanks() == 1);
	CHECK(HostTarget::canAccepts(0x241));
	rmd.reset();
	CHECK_FALSE(RmdMotorCAN1::inUse);
	CHECK(HostTarget::canActiveFilterBanks() == 0);
}

TEST_CASE_FIXTURE(RmdFixture, "[R10.4] a 0xA1 reply of node 2 reaches instance 2 on buffer 1 and sets its temperature") {
	auto rmd = makeRmd2();
	REQUIRE(HostTarget::canPort().filters.size() == 1);
	CHECK(HostTarget::canPort().filters[0].buffer == 1); // the fixture delivers FIFO 1 at once (modelling choice)
	reachAvailable(*rmd);
	peer2.setTemperature(33);
	CHECK(peer2.sendReply(0xA1) != 0);
	CHECK(CommandHelpers::get(*rmd, "temp") == 33); // SP 2.19: data[1] = temperature
}

TEST_CASE_FIXTURE(RmdFixture, "[R10.4] two instances at once use their own ids for the filters and for the command frames") {
	auto rmd1 = makeRmd1();
	auto rmd2 = makeRmd2();
	CHECK(HostTarget::canActiveFilterBanks() == 2);
	uint32_t fifo1 = 99, fifo2 = 99;
	CHECK(HostTarget::canAccepts(0x241, false, false, &fifo1));
	CHECK(HostTarget::canAccepts(0x242, false, false, &fifo2));
	CHECK(fifo1 == 0);
	CHECK(fifo2 == 1);
	runFor(100); // both threads send 0xB5 to their own node
	CHECK(txFramesCmd(0x141, 0xB5).size() == 1);
	CHECK(txFramesCmd(0x142, 0xB5).size() == 1);
	CHECK(peer1.violations().empty());
	CHECK(peer2.violations().empty());
}

TEST_CASE_FIXTURE(RmdFixture, "[R10.4] a reply of node 1 does not reach instance 2 and a reply of node 2 does not reach instance 1") {
	auto rmd1 = makeRmd1();
	auto rmd2 = makeRmd2();
	peer1.setTemperature(30);
	peer1.setVoltage(481);
	peer2.setTemperature(40);
	peer2.setVoltage(502);
	CHECK(peer1.sendReply(0x9A) != 0);
	CHECK(CommandHelpers::get(*rmd1, "temp") == 30);
	CHECK(CommandHelpers::get(*rmd1, "vbus") == 481);
	CHECK(CommandHelpers::get(*rmd2, "temp") == 0); // untouched
	CHECK(CommandHelpers::get(*rmd2, "vbus") == 0);
	CHECK(peer2.sendReply(0x9A) != 0);
	CHECK(CommandHelpers::get(*rmd2, "temp") == 40);
	CHECK(CommandHelpers::get(*rmd2, "vbus") == 502);
	CHECK(CommandHelpers::get(*rmd1, "temp") == 30); // still the value of node 1
	CHECK(CommandHelpers::get(*rmd1, "vbus") == 481);
}

}
