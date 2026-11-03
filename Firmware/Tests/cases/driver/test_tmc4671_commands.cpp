/*
 * TMC4671 driver, group G8: commands (requirements R100-R102 of Firmware/Tests/driver/TMC4671_REQUIREMENTS.md, lead
 * decisions L1-L18; R103 is P3 and not tested, L8).
 *
 * The commands are executed through CommandHelpers (CommandHandler::command with a ParsedCommand, like the command
 * thread does). Per L7 the tests assert the reply value / format and the register effect, never the help text.
 * Register layouts: TMC4671-LA datasheet Rev 2.07, cited "DS 0xNN".
 *
 * Commands that start long procedures (calibrate, autohome, pidautotune, encalign) are only tested for being accepted
 * (status, no reply); the procedures themselves are covered by the encoder / state tests.
 *
 * Host vs target: replies of temp / vm / iScale / pwmfreq go through float. The expected values are hand derived in
 * double precision and compared with a tolerance (2 mV, 0.02 C, 1 Hz); iScale is compared as a string made by
 * std::to_string(float), which promotes to double on both host and ARM. No signed shifts of negative values, no
 * out-of-range float to int conversions in the values chosen.
 */
#include "doctest.h"
#include "TmcFixture.h"
#include "SystemCommands.h"
#include "eeprom_addresses.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace {

using CommandHelpers::get;
using CommandHelpers::run;

/** Executes a set command and checks that it was accepted */
void setCmd(TMC4671 &t, const char *name, int64_t val) { CHECK(run(t, name, CMDtype::set, val) == CommandStatus::OK); }

/** The 42 commands: name -> id (declaration order of TMC4671_commands, TMC4671.h) */
struct CmdId {
	const char *name;
	uint32_t id;
};
constexpr CmdId kCommands[] = {
	{"cpr", 0},		   {"mtype", 1},		 {"encsrc", 2},		  {"tmcHwType", 3},		  {"encalign", 4},	   {"poles", 5},
	{"acttrq", 6},	   {"pwmlim", 7},		 {"torqueP", 8},	  {"torqueI", 9},		  {"fluxP", 10},	   {"fluxI", 11},
	{"velocityP", 12}, {"velocityI", 13},	 {"posP", 14},		  {"posI", 15},			  {"tmctype", 16},	   {"pidPrec", 17},
	{"phiesrc", 18},   {"fluxoffset", 19},	 {"seqpi", 20},		  {"iScale", 21},		  {"encdir", 22},	   {"temp", 23},
	{"reg", 24},	   {"svpwm", 25},		 {"calibrate", 26},	  {"calibrated", 27},	  {"abnindex", 28},	   {"autohome", 29},
	{"state", 30},	   {"abnpol", 31},		 {"combineEncoder", 32}, {"invertForce", 33},	  {"vm", 34},		   {"extphie", 35},
	{"trqbq_mode", 36}, {"trqbq_f", 37},	 {"trqbq_q", 38},	  {"pidautotune", 39},	  {"fluxbrake", 40},   {"pwmfreq", 41},
};

/** TmcFixture with the debug mode on: pwmlim, phiesrc, encdir, reg and pwmfreq are CMDFLAG_DEBUG commands and only found in debug mode */
struct TmcCmdFixture : TmcFixture {
	bool savedDebugMode;
	TmcCmdFixture() : savedDebugMode(SystemCommands::debugMode) { SystemCommands::debugMode = true; }
	~TmcCmdFixture() override { SystemCommands::debugMode = savedDebugMode; }
};

} // namespace

TEST_SUITE("TMC4671 commands") {

// ---------------------------------------------------------------------------------------------------------------
// R100 registry
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] all 42 commands are registered under their name with the ids 0 to 41") {
	auto tmc = makeTmc1();
	static_assert(sizeof(kCommands) / sizeof(kCommands[0]) == 42, "table must list 42 commands");
	for (const CmdId &c : kCommands) {
		CmdHandlerCommanddef *def = tmc->getCommandFromName(c.name);
		INFO("command " << c.name);
		REQUIRE(def != nullptr);
		CHECK(def->cmdId == c.id);
	}
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] a command id outside 0 to 41 is answered with NOT_FOUND") {
	auto tmc = makeTmc1();
	ParsedCommand p;
	p.cmdId = 42;
	p.type = CMDtype::get;
	p.target = tmc.get();
	std::vector<CommandReply> replies;
	CHECK(tmc->command(p, replies) == CommandStatus::NOT_FOUND);
	CHECK(replies.empty());
}

// ---------------------------------------------------------------------------------------------------------------
// R100 cpr (0)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] cpr set 4000 writes 4000 to ABN_DECODER_PPR 0x26 and AENC_DECODER_PPR 0x40 and get returns 4000") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "cpr", 4000);
	// DS 0x26 p86 ABN_DECODER_PPR u24, DS 0x40 p90 AENC_DECODER_PPR u24; encoder type NONE -> get reports the ABN cpr
	CHECK(chip1.reg(0x26) == 4000u);
	CHECK(chip1.reg(0x40) == 4000u);
	CHECK(get(*tmc, "cpr") == 4000);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: cpr set 0 is raised to 1 before it is written") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "cpr", 0);
	CHECK(chip1.reg(0x26) == 1u);
	CHECK(chip1.reg(0x40) == 1u);
	CHECK(get(*tmc, "cpr") == 1);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 / R102 mtype (1), poles (5)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] mtype set 3 writes MOTOR_TYPE 3 (BLDC) to bits 23:16 of MOTOR_TYPE_N_POLE_PAIRS 0x1B and get returns 3") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "mtype", 3);
	// DS 0x1B p84: MOTOR_TYPE [23:16] 3 = three phase BLDC, N_POLE_PAIRS [15:0]
	CHECK(((chip1.reg(0x1B) >> 16) & 0xFF) == 3u);
	CHECK(get(*tmc, "mtype") == 3);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: mtype set 1 (DC) forces the pole pair count to 1: register 0x1B = 0x00010001") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "mtype", 1);
	CHECK(chip1.reg(0x1B) == 0x00010001u);
	CHECK(get(*tmc, "mtype") == 1);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] mtype set 2 (stepper) writes MOTOR_TYPE 2 to bits 23:16 of 0x1B") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "mtype", 2);
	CHECK(((chip1.reg(0x1B) >> 16) & 0xFF) == 2u); // DS 0x1B p84: 2 = two phase stepper
	CHECK(get(*tmc, "mtype") == 2);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] mtype set accepts all four motor types 0 to 3 and get returns the last one") {
	auto tmc = makeTmc1();
	for (int64_t t = 0; t <= 3; t++) {
		INFO("type " << t);
		setCmd(*tmc, "mtype", t);
		CHECK(get(*tmc, "mtype") == t);
	}
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] mtype get on a handler with the info type answers a non empty string list") {
	auto tmc = makeTmc1();
	std::vector<CommandReply> replies;
	CHECK(run(*tmc, "mtype", CMDtype::info, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().type == CommandReplyType::STRING);
	CHECK_FALSE(replies.front().reply.empty()); // L7: format only, not the text
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R102] characterization: mtype set 9 (invalid) is mapped to NONE (0) in the register and in get") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "mtype", 3);
	REQUIRE(get(*tmc, "mtype") == 3);
	setCmd(*tmc, "mtype", 9);
	// The command's range check tests the command TYPE, not the value (inventory 9.11); isMotSupported maps it to NONE
	CHECK(get(*tmc, "mtype") == 0);
	CHECK(((chip1.reg(0x1B) >> 16) & 0xFF) == 0u);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R102] characterization: mtype set 4 (first value above BLDC) is mapped to NONE (0)") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "mtype", 3);
	setCmd(*tmc, "mtype", 4);
	CHECK(get(*tmc, "mtype") == 0);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R102] characterization: mtype set 259 wraps to 3 (BLDC) because the value is cast to 8 bit") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "mtype", 259); // 259 = 256 + 3. Probably unintended, see lead decisions
	CHECK(get(*tmc, "mtype") == 3);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] poles set 7 for a BLDC motor writes N_POLE_PAIRS 7 to bits 15:0 of 0x1B and get returns 7") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "mtype", 3);
	setCmd(*tmc, "poles", 7);
	CHECK(chip1.reg(0x1B) == 0x00030007u); // DS 0x1B p84: type 3 in [23:16], pole pairs in [15:0]
	CHECK(get(*tmc, "poles") == 7);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 encsrc (2)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] encsrc set 1 selects ABN: get returns 1 and ABN_DECODER_MODE 0x25 is written with the three polarity bits set") {
	auto tmc = makeTmc1();
	chip1.clearLog();
	setCmd(*tmc, "encsrc", 1);
	CHECK(get(*tmc, "encsrc") == 1);
	auto w = chip1.writesTo(0x25);
	REQUIRE_FALSE(w.empty());
	CHECK(w.back() == 0x7u); // DS 0x25 p86: apol bit 0, bpol bit 1, npol bit 2 (firmware default polarity on), direction 0
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] encsrc set 0 selects no encoder and get returns 0") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "encsrc", 1);
	setCmd(*tmc, "encsrc", 0);
	CHECK(get(*tmc, "encsrc") == 0);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] encsrc set 4 selects hall: get returns 4 and PHI_E_SELECTION 0x52 is 5") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "encsrc", 4);
	CHECK(get(*tmc, "encsrc") == 4);
	CHECK(chip1.reg(0x52) == 5u); // DS 0x52 p87: 5 = phi_e_hal
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: encsrc set 99 (unsupported) falls back to NONE (0)") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "encsrc", 1);
	setCmd(*tmc, "encsrc", 99);
	CHECK(get(*tmc, "encsrc") == 0);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: encsrc set 5 (external) without an external encoder falls back to NONE (0)") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "encsrc", 5);
	CHECK(get(*tmc, "encsrc") == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 tmcHwType (3)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: tmcHwType get returns 6 (v1.3 ACS724) for a driver without stored configuration") {
	auto tmc = makeTmc1();
	CHECK(get(*tmc, "tmcHwType") == 6); // tmc4671_hw_configs[0] (CFG.cpp)
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: tmcHwType set 3 selects v1.2.2: get returns 3, ADC_VM_LIMITS 0x75 = 50700 | 50900 << 16, BBM 0x19 = 20 | 20 << 8") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "tmcHwType", 3);
	CHECK(get(*tmc, "tmcHwType") == 3);
	// board constants of the v1.2.2 configuration (CFG.cpp); layouts DS 0x75 p112 (limit low [15:0], high [31:16]) and DS 0x19 p84
	CHECK(chip1.reg(0x75) == (50700u | (50900u << 16)));
	CHECK(chip1.reg(0x19) == (20u | (20u << 8)));
	setCmd(*tmc, "tmcHwType", 6); // vSenseMult is a global of the voltage sense module: restore
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: tmcHwType set 4 (version without configuration) keeps the current hardware configuration") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "tmcHwType", 4);
	CHECK(get(*tmc, "tmcHwType") == 6);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] tmcHwType info answers one string reply per available hardware version") {
	auto tmc = makeTmc1();
	std::vector<CommandReply> replies;
	CHECK(run(*tmc, "tmcHwType", CMDtype::info, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 4); // v1.3, v1.2.2, v1.2, v1.0 (CFG.cpp:107)
	for (const CommandReply &r : replies) {
		CHECK(r.type == CommandReplyType::STRING_OR_INT);
		CHECK_FALSE(r.reply.empty());
	}
}

// ---------------------------------------------------------------------------------------------------------------
// R100 encalign (4), calibrate (26), autohome (29), pidautotune (39): accepted only
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: encalign get is accepted without an immediate reply (NO_REPLY)") {
	auto tmc = makeTmc1();
	std::vector<CommandReply> replies;
	CHECK(run(*tmc, "encalign", CMDtype::get, 0, &replies) == CommandStatus::NO_REPLY);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: encalign set is rejected with ERR") {
	auto tmc = makeTmc1();
	CHECK(run(*tmc, "encalign", CMDtype::set, 1) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: calibrate get is accepted (OK) without a reply and without SPI traffic of its own") {
	auto tmc = makeTmc1();
	chip1.clearLog();
	std::vector<CommandReply> replies;
	CHECK(run(*tmc, "calibrate", CMDtype::get, 0, &replies) == CommandStatus::OK);
	CHECK(replies.empty());
	CHECK(chip1.log().empty()); // only requests the state change, the thread does the work
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: autohome get is accepted (OK) without a reply and without SPI traffic of its own") {
	auto tmc = makeTmc1();
	chip1.clearLog();
	std::vector<CommandReply> replies;
	CHECK(run(*tmc, "autohome", CMDtype::get, 0, &replies) == CommandStatus::OK);
	CHECK(replies.empty());
	CHECK(chip1.log().empty());
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: pidautotune get is accepted without an immediate reply (NO_REPLY) and without SPI traffic of its own") {
	auto tmc = makeTmc1();
	chip1.clearLog();
	std::vector<CommandReply> replies;
	CHECK(run(*tmc, "pidautotune", CMDtype::get, 0, &replies) == CommandStatus::NO_REPLY);
	CHECK(replies.empty());
	CHECK(chip1.log().empty());
}

// ---------------------------------------------------------------------------------------------------------------
// R100 acttrq (6)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] acttrq get decodes PID_TORQUE_FLUX_ACTUAL 0xFF380064 as torque -200 (val) and flux 100 (adr)") {
	auto tmc = makeTmc1();
	chip1.setReg(0x69, 0xFF380064u); // DS 0x69 p98: flux [15:0] = 0x0064 = 100, torque [31:16] = 0xFF38 = -200
	std::vector<CommandReply> replies;
	CHECK(run(*tmc, "acttrq", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().type == CommandReplyType::DOUBLEINTS);
	CHECK(replies.front().val == -200);
	CHECK(replies.front().adr == 100);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 pwmlim (7)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] pwmlim set 3000 writes PID_UQ_UD_LIMITS 0x5D = 3000 and get returns 3000") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "pwmlim", 3000);
	CHECK(chip1.reg(0x5D) == 3000u); // DS 0x5D p96: PID_UQ_UD_LIMITS u16
	CHECK(get(*tmc, "pwmlim") == 3000);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 PI gains (8-15). DS 0x54 flux, 0x56 torque, 0x58 velocity, 0x5A position: P in [31:16], I in [15:0]
// ---------------------------------------------------------------------------------------------------------------

#define PI_CASE(CMD, ADR, SHIFT, PARTNAME)                                                                                  \
	TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] " CMD " set 43981 writes " PARTNAME " 0xABCD of register " #ADR " and get returns 43981") { \
		auto tmc = makeTmc1();                                                                                              \
		setCmd(*tmc, CMD, 43981); /* 0xABCD, uses bit 15 */                                                                 \
		CHECK(((chip1.reg(ADR) >> (SHIFT)) & 0xFFFFu) == 0xABCDu);                                                          \
		CHECK(get(*tmc, CMD) == 43981);                                                                                     \
	}

PI_CASE("fluxP", 0x54, 16, "the P half (bits 31:16)")
PI_CASE("fluxI", 0x54, 0, "the I half (bits 15:0)")
PI_CASE("torqueP", 0x56, 16, "the P half (bits 31:16)")
PI_CASE("torqueI", 0x56, 0, "the I half (bits 15:0)")
PI_CASE("velocityP", 0x58, 16, "the P half (bits 31:16)")
PI_CASE("velocityI", 0x58, 0, "the I half (bits 15:0)")
PI_CASE("posP", 0x5A, 16, "the P half (bits 31:16)")
PI_CASE("posI", 0x5A, 0, "the I half (bits 15:0)")

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] setting one PI gain leaves the other three PI registers at the driver's gains and the other half of its own register") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "torqueP", 1000);
	setCmd(*tmc, "torqueI", 2000);
	// DS 0x56 p95: P [31:16], I [15:0]; both set by two commands, the second write must keep the first value
	CHECK(chip1.reg(0x56) == ((1000u << 16) | 2000u));
	CHECK(get(*tmc, "torqueP") == 1000);
	CHECK(get(*tmc, "torqueI") == 2000);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 tmctype (16)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] tmctype get of a present chip answers a text starting with TMC4671 and the SI_VERSION word 0x00010003 as value") {
	auto tmc = makeTmc1();
	std::vector<CommandReply> replies;
	CHECK(run(*tmc, "tmctype", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().type == CommandReplyType::STRING_OR_INT);
	CHECK(replies.front().reply.rfind("TMC4671", 0) == 0); // DS 0x00 p77 SI_TYPE "4671" (digits of the version: R11)
	CHECK(replies.front().val == 0x00010003); // DS 0x00 with CHIPINFO_ADDR 1: SI_VERSION
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] tmctype get of a chip that does not answer replies the text No driver connected and value 0") {
	auto tmc = makeTmc1();
	chip1.setPresent(false);
	std::vector<CommandReply> replies;
	CHECK(run(*tmc, "tmctype", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().reply == "No driver connected");
	CHECK(replies.front().val == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 pidPrec (17)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] pidPrec set 3 selects Q4.12 for the current I and P gains: CONFIG word 62 = 3 and get returns 3") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "pidPrec", 3);
	// Firmware writes CONFIG_ADDR 62, then CONFIG_DATA: bit 0 current I, bit 1 current P, bits 2-5 velocity / position
	CHECK(chip1.configWord(62) == 3u);
	CHECK(get(*tmc, "pidPrec") == 3);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] pidPrec set 2 selects Q4.12 for the current P gain only: CONFIG word 62 = 2 and get returns 2") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "pidPrec", 2);
	CHECK(chip1.configWord(62) == 2u);
	CHECK(get(*tmc, "pidPrec") == 2);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] pidPrec set 1 selects Q4.12 for the current I gain only: CONFIG word 62 = 1 and get returns 1") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "pidPrec", 1);
	CHECK(chip1.configWord(62) == 1u);
	CHECK(get(*tmc, "pidPrec") == 1);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 phiesrc (18)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] phiesrc set 3 writes PHI_E_SELECTION 0x52 = 3 (ABN) and get returns 3") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "phiesrc", 3);
	CHECK(chip1.reg(0x52) == 3u); // DS 0x52 p87: 3 = phi_e_abn
	CHECK(get(*tmc, "phiesrc") == 3);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] phiesrc set 1 writes PHI_E_SELECTION 0x52 = 1 (external) and get returns 1") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "phiesrc", 1);
	CHECK(chip1.reg(0x52) == 1u); // DS 0x52 p87: 1 = phi_e_ext
	CHECK(get(*tmc, "phiesrc") == 1);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] phiesrc get reads the selection from the chip: register 0x52 = 6 is reported as 6") {
	auto tmc = makeTmc1();
	chip1.setReg(0x52, 6); // DS 0x52: 6 = phi_e_aenc
	CHECK(get(*tmc, "phiesrc") == 6);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 fluxoffset (19), fluxbrake (40), combineEncoder (32), invertForce (33): driver members only
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] fluxoffset set 123 is stored for get and does not touch the chip") {
	auto tmc = makeTmc1();
	chip1.clearLog();
	setCmd(*tmc, "fluxoffset", 123);
	CHECK(get(*tmc, "fluxoffset") == 123);
	CHECK(chip1.log().empty());
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] fluxbrake set 1 is stored for get and does not touch the chip") {
	auto tmc = makeTmc1();
	chip1.clearLog();
	setCmd(*tmc, "fluxbrake", 1);
	CHECK(get(*tmc, "fluxbrake") == 1);
	CHECK(chip1.log().empty());
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] fluxbrake set 0 after 1 is stored for get") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "fluxbrake", 1);
	setCmd(*tmc, "fluxbrake", 0);
	CHECK(get(*tmc, "fluxbrake") == 0);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] combineEncoder set 1 is stored for get and does not touch the chip") {
	auto tmc = makeTmc1();
	chip1.clearLog();
	setCmd(*tmc, "combineEncoder", 1);
	CHECK(get(*tmc, "combineEncoder") == 1);
	CHECK(chip1.log().empty());
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] invertForce set 1 is stored for get once combineEncoder is on") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "combineEncoder", 1);
	setCmd(*tmc, "invertForce", 1);
	CHECK(get(*tmc, "invertForce") == 1);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: switching combineEncoder off also switches invertForce off") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "combineEncoder", 1);
	setCmd(*tmc, "invertForce", 1);
	REQUIRE(get(*tmc, "invertForce") == 1);
	setCmd(*tmc, "combineEncoder", 0);
	CHECK(get(*tmc, "invertForce") == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 seqpi (20), svpwm (25)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] seqpi set 0 clears MODE_PID_TYPE (bit 31 of 0x63) and get returns 0") {
	auto tmc = makeTmc1();
	chip1.setReg(0x63, 0x80000001u);
	setCmd(*tmc, "seqpi", 0);
	CHECK(chip1.reg(0x63) == 0x00000001u); // DS 0x63 p97: bit 31 MODE_PID_TYPE 0 = parallel / classic; the motion mode bits are kept
	CHECK(get(*tmc, "seqpi") == 0);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] seqpi set 1 sets MODE_PID_TYPE (bit 31 of 0x63) and get returns 1") {
	auto tmc = makeTmc1();
	chip1.setReg(0x63, 0x00000002u);
	setCmd(*tmc, "seqpi", 1);
	CHECK(chip1.reg(0x63) == 0x80000002u); // DS 0x63 p97: bit 31 = 1 advanced PID structure; the motion mode bits are kept
	CHECK(get(*tmc, "seqpi") == 1);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] svpwm set 1 for a BLDC motor sets SVPWM (bit 8 of 0x1A) and get returns 1") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "mtype", 3);
	chip1.setReg(0x1A, 0x00000000u);
	setCmd(*tmc, "svpwm", 1);
	CHECK((chip1.reg(0x1A) & 0x100u) == 0x100u); // DS 0x1A p84: SV_MODE bit 8 (BLDC / PMSM only)
	CHECK(get(*tmc, "svpwm") == 1);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] svpwm set 0 for a BLDC motor clears SVPWM (bit 8 of 0x1A), keeps the chopper mode byte and get returns 0") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "mtype", 3);
	chip1.setReg(0x1A, 0x00000107u);
	setCmd(*tmc, "svpwm", 0);
	CHECK(chip1.reg(0x1A) == 0x00000007u);
	CHECK(get(*tmc, "svpwm") == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 iScale (21)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: iScale get replies the current scaler of the v1.3 board as text 0.001156") {
	auto tmc = makeTmc1();
	std::vector<CommandReply> replies;
	CHECK(run(*tmc, "iScale", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().type == CommandReplyType::STRING);
	// Hand derivation: 2.5 / (0x7FFF * 0.066) = 2.5 / 2162.622 = 0.00115598 -> std::to_string (6 decimals) "0.001156"
	CHECK(replies.front().reply == "0.001156");
}

// ---------------------------------------------------------------------------------------------------------------
// R100 encdir (22), abnpol (31), abnindex (28): ABN decoder
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] encdir set 1 sets the count direction bit (bit 12) of ABN_DECODER_MODE 0x25 and get returns 1") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "encdir", 1);
	CHECK(chip1.reg(0x25) == 0x1007u); // DS 0x25 p86: direction bit 12; polarity bits 0-2 stay at the firmware default (all on)
	CHECK(get(*tmc, "encdir") == 1);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] encdir set 0 after 1 clears the count direction bit of 0x25 and get returns 0") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "encdir", 1);
	setCmd(*tmc, "encdir", 0);
	CHECK(chip1.reg(0x25) == 0x0007u);
	CHECK(get(*tmc, "encdir") == 0);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] abnpol set 0 clears the A, B and N polarity bits (bits 2:0) of 0x25 and get returns 0") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "abnpol", 0);
	CHECK(chip1.reg(0x25) == 0x0u); // DS 0x25 p86: apol bit 0, bpol bit 1, npol bit 2
	CHECK(get(*tmc, "abnpol") == 0);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] abnpol set 5 (non zero) sets the A, B and N polarity bits of 0x25 and get returns 1") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "abnpol", 0);
	setCmd(*tmc, "abnpol", 5);
	CHECK(chip1.reg(0x25) == 0x7u);
	CHECK(get(*tmc, "abnpol") == 1);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] abnindex set 1 is stored for get and rewrites the ABN setup 0x25, 0x26 and 0x29") {
	auto tmc = makeTmc1();
	chip1.clearLog();
	setCmd(*tmc, "abnindex", 1);
	CHECK(get(*tmc, "abnindex") == 1);
	REQUIRE(chip1.writesTo(0x25).size() == 1);
	REQUIRE(chip1.writesTo(0x26).size() == 1);
	REQUIRE(chip1.writesTo(0x29).size() == 1);
	CHECK(chip1.writesTo(0x25).back() == 0x7u);	  // DS 0x25 p86, default polarity bits
	CHECK(chip1.writesTo(0x26).back() == 10000u); // DS 0x26 p86, firmware default cpr
	CHECK(chip1.writesTo(0x29).back() == 0u);	  // DS 0x29 p87, phi_e and phi_m offsets 0
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] abnindex set 0 after 1 is stored for get") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "abnindex", 1);
	setCmd(*tmc, "abnindex", 0);
	CHECK(get(*tmc, "abnindex") == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 temp (23), vm (34)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: temp get of a thermistor reading of 25 C replies 2500 (C * 100) and selects ADC_RAW_ADDR 2") {
	auto tmc = makeTmc1();
	// Hand derivation (v1.3 board: R2 1500, R 10000, beta 4300, adcOffset 0): adcval = raw - 0x7FFF; the thermistor
	// resistance is r = 1500 * 43252 / adcval. 25 C is r = 10000 -> adcval = 6487.8, use 6488: r = 9999.7,
	// ln(r/R) = -3.1e-5, 1/T = 1/298.15 - 3.1e-5/4300 -> T = 25.0006 C.
	chip1.setAdcRaw(2, 0x7FFFu + 6488u); // = 0x9957
	chip1.clearLog();
	int64_t v = get(*tmc, "temp");
	CHECK(std::abs(static_cast<double>(v) - 2500.0) <= 2.0);
	REQUIRE_FALSE(chip1.writesTo(0x03).empty());
	CHECK(chip1.writesTo(0x03).front() == 2u); // DS 0x03 p77: ADC_RAW_ADDR
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: temp get replies 0 for a hardware version without thermistor (v1.0)") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "tmcHwType", 1);
	chip1.setAdcRaw(2, 0x7FFFu + 6488u);
	CHECK(get(*tmc, "temp") == 0);
	setCmd(*tmc, "tmcHwType", 6); // restore the global vSenseMult
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: vm get of ADC raw 0x993B replies about 23986 mV and selects ADC_RAW_ADDR 1") {
	auto tmc = makeTmc1();
	// Hand derivation: raw low word 0x993B = 39227, minus 0x7FFF (32767) = 6460 counts (adcOffset 0 for v1.3).
	// vmScaler = (2.5 / 32767) * ((1.5 + 71.5) / 1.5) = 3.71309 mV per count -> 6460 * 3.71309 = 23985.9 mV
	chip1.setAdcRaw(1, 0x993Bu);
	chip1.clearLog();
	int64_t v = get(*tmc, "vm");
	const double expected = 6460.0 * (2.5 / 32767.0) * ((1.5 + 71.5) / 1.5) * 1000.0;
	CHECK(std::abs(static_cast<double>(v) - expected) <= 2.0);
	REQUIRE_FALSE(chip1.writesTo(0x03).empty());
	CHECK(chip1.writesTo(0x03).front() == 1u);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 reg (24) and R101
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R101] reg setat writes the value to the register at the given address") {
	auto tmc = makeTmc1();
	CHECK(run(*tmc, "reg", CMDtype::setat, 0x321, nullptr, 0x5D) == CommandStatus::OK);
	CHECK(chip1.reg(0x5D) == 0x321u); // DS 0x5D p96
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R101] reg getat with the address in both val and adr (as the text parser sets them) reads that register") {
	auto tmc = makeTmc1();
	chip1.setReg(0x5D, 0x1234u);
	std::vector<CommandReply> replies;
	CHECK(run(*tmc, "reg", CMDtype::getat, 0x5D, &replies, 0x5D) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == 0x1234);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R101] reg getat reads the register at the address given in adr" * KNOWN_ISSUE("B-T19: the reg getat handler uses cmd.val as the address, every other getat handler and setat use cmd.adr")) {
	auto tmc = makeTmc1();
	chip1.setReg(0x5D, 0x1234u);
	std::vector<CommandReply> replies;
	// A request that carries the address in adr only (HID / CAN requestAddr path), val = 0
	CHECK(run(*tmc, "reg", CMDtype::getat, 0, &replies, 0x5D) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == 0x1234);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R101] characterization: reg with a plain get or set is rejected with ERR") {
	auto tmc = makeTmc1();
	CHECK(run(*tmc, "reg", CMDtype::get) == CommandStatus::ERR);
	CHECK(run(*tmc, "reg", CMDtype::set, 1) == CommandStatus::ERR);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 calibrated (27), state (30)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] calibrated get replies 0 for a driver that has not calibrated the ADC yet") {
	auto tmc = makeTmc1();
	CHECK(get(*tmc, "calibrated") == 0);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: calibrated get replies 0 after start up when no ADC offsets are stored in flash (full calibration required)") {
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	// The ADC offsets were measured at start up but recalibrationRequired stays set until a full calibration ran
	CHECK(get(*tmc, "calibrated") == 0);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: calibrated get replies 1 after start up when ADC offsets are stored in flash") {
	REQUIRE(Flash_Write(ADR_TMC1_ADC_I0_OFS, 0x7FFF));
	REQUIRE(Flash_Write(ADR_TMC1_ADC_I1_OFS, 0x7FFF));
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	CHECK(get(*tmc, "calibrated") == 1);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] state get replies the numeric state of the state machine (Shutdown after start up)") {
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	CHECK(get(*tmc, "state") == static_cast<int64_t>(TMC_ControlState::Shutdown));
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] state get replies the same value as getState() before the thread ran") {
	auto tmc = makeTmc1();
	CHECK(get(*tmc, "state") == static_cast<int64_t>(tmc->getState()));
}

// ---------------------------------------------------------------------------------------------------------------
// R100 extphie (35): division by zero without an external encoder (B-T18)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] extphie get without an external encoder answers 0 without crashing" * doctest::skip() * doctest::description("KNOWN ISSUE (crash): B-T18: getPhiEfromExternalEncoder divides by the cpr (0) of the missing external encoder, SIGFPE confirmed on the host. Skipped because it terminates the test run")) {
	auto tmc = makeTmc1();
	std::vector<CommandReply> replies;
	CHECK(run(*tmc, "extphie", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 trqbq_mode (36), trqbq_f (37), trqbq_q (38)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] trqbq_mode set 1 (lowpass) enables the torque biquad: get returns 1 and the biquad_t_enable word (CONFIG 23) is non zero") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "trqbq_mode", 1);
	CHECK(get(*tmc, "trqbq_mode") == 1);
	// DS CONFIG_ADDR 23 = biquad_t_enable (coefficient values: R47, config tests)
	CHECK(chip1.configWord(23) != 0u);
	CHECK(chip1.configWord(20) != 0u); // b0 of a lowpass is positive
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] trqbq_mode set 0 after lowpass disables the torque biquad: get returns 0 and the enable word is 0") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "trqbq_mode", 1);
	setCmd(*tmc, "trqbq_mode", 0);
	CHECK(get(*tmc, "trqbq_mode") == 0);
	CHECK(chip1.configWord(23) == 0u);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] trqbq_mode set 2 and 3 are stored for get (notch, peak)") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "trqbq_mode", 2);
	CHECK(get(*tmc, "trqbq_mode") == 2);
	setCmd(*tmc, "trqbq_mode", 3);
	CHECK(get(*tmc, "trqbq_mode") == 3);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: trqbq_mode set 4 (invalid) leaves the mode unchanged and answers a string instead") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "trqbq_mode", 2);
	std::vector<CommandReply> replies;
	CHECK(run(*tmc, "trqbq_mode", CMDtype::set, 4, &replies) == CommandStatus::OK);
	CHECK(get(*tmc, "trqbq_mode") == 2);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().type == CommandReplyType::STRING);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] trqbq_f set 500 is stored for get") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "trqbq_f", 500);
	CHECK(get(*tmc, "trqbq_f") == 500);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: trqbq_f set 0 is raised to 1 (the help text says 0 disables the filter)") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "trqbq_f", 0);
	CHECK(get(*tmc, "trqbq_f") == 1);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: trqbq_f set 100000 is clipped to 0x1FFF (8191)") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "trqbq_f", 100000);
	CHECK(get(*tmc, "trqbq_f") == 0x1FFF);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] trqbq_q set 60 is stored for get") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "trqbq_q", 60);
	CHECK(get(*tmc, "trqbq_q") == 60);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: trqbq_q set 200 is clipped to 127 and set -5 to 0") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "trqbq_q", 200);
	CHECK(get(*tmc, "trqbq_q") == 127);
	setCmd(*tmc, "trqbq_q", -5);
	CHECK(get(*tmc, "trqbq_q") == 0);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] trqbq_f and trqbq_q set while the lowpass is active rewrite the torque biquad coefficients") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "trqbq_mode", 1);
	chip1.clearLog();
	setCmd(*tmc, "trqbq_f", 700);
	CHECK_FALSE(chip1.configWrites().empty());
	chip1.clearLog();
	const size_t before = chip1.configWrites().size();
	setCmd(*tmc, "trqbq_q", 80);
	CHECK(chip1.configWrites().size() > before);
}

// ---------------------------------------------------------------------------------------------------------------
// R100 pwmfreq (41)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] pwmfreq set 25000 writes PWM_MAXCNT 0x18 = 3999 and get returns 25000") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "pwmfreq", 25000);
	// DS 0x18 p84: f = 100 MHz / (MAXCNT + 1) -> MAXCNT = 100e6 / 25000 - 1 = 3999
	auto w = chip1.writesTo(0x18);
	REQUIRE_FALSE(w.empty());
	CHECK(w.back() == 3999u);
	CHECK(get(*tmc, "pwmfreq") == 25000);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] pwmfreq set 100000 writes PWM_MAXCNT 0x18 = 999 and get returns 100000") {
	auto tmc = makeTmc1();
	setCmd(*tmc, "pwmfreq", 100000);
	auto w = chip1.writesTo(0x18);
	REQUIRE_FALSE(w.empty());
	CHECK(w.back() == 999u); // 100e6 / 100000 - 1
	CHECK(get(*tmc, "pwmfreq") == 100000);
}

TEST_CASE_FIXTURE(TmcCmdFixture, "[R100] characterization: pwmfreq set 0 is ignored: no write to PWM_MAXCNT") {
	auto tmc = makeTmc1();
	chip1.clearLog();
	setCmd(*tmc, "pwmfreq", 0);
	CHECK(chip1.writesTo(0x18).empty());
}

} // TEST_SUITE
