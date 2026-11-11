/*
 * SystemCommands (FFBoard/Src/SystemCommands.cpp): the "sys" command handler that every main class owns.
 *
 * The handler under test is the one of a real failsafe main class created through the firmware's main class chooser
 * (like cppmain.cpp does), so `mainclass`, the command thread, the CDC command interface and the error printer exist
 * as on the target. Commands are executed directly with internalCommand (one behaviour per case) and a few end to
 * end through the CDC command interface and the command thread, where the access flags of the command table are
 * checked.
 *
 * Host-vs-target notes (see Tests/Code_Coverage/claude/plans/PROGRESS.md):
 *  - NVIC_SystemReset / RebootDFU return on the host (recorded in HostTarget::state()); on the target they do not, so
 *    the code after them (the reply of reboot, dfu and a main class change) only runs on the host.
 *  - taskstats / tasklist (configGENERATE_RUN_TIME_STATS, configUSE_TRACE_FACILITY) and otp (OTPMEMORY, needs
 *    FLASH_OTP_BASE) are registered on the F407 target but not compiled in ffb_target_tests.
 *  - The map-backed Flash_* fake (host_platform.cpp) never fails Flash_Format, so the format error path is not tested.
 *  - The signature command runs on the OTP model of ffb_target_tests (HostTarget::otpSlot / setOtpSlot), which
 *    reproduces OTP_Write / OTP_Read of flash_helpers.cpp (the real source is not compiled in this executable).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "ClassChooser.h"
#include "CustomMain.h"
#include "ErrorHandler.h"
#include "FFBoardMain.h"
#include "PersistentStorage.h"
#include "SystemCommands.h"
#include "constants.h"
#include "eeprom_addresses.h"
#include "mainclass_chooser.h"
#include "voltagesense.h"

#include "FFBHIDExt.h"
#include "FFBJoystick.h"
#include "FFBWheel.h"

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

extern ClassChooser<FFBoardMain> mainchooser; // cppmain.cpp

namespace {

/** VREFINT raw value equal to the calibration value: the analog reference is exactly 3300 mV */
constexpr uint32_t VREFINT_RAW = 1500;

/** A failsafe main class created like cppmain.cpp does (mainclass, selection id 0) and its "sys" handler */
struct SysFixture : TargetFixture {
	SysFixture() {
		main.reset(mainchooser.Create(0));
		REQUIRE(main != nullptr);
		mainclass = main.get();
	}
	~SysFixture() override {
		mainclass = nullptr;
		main.reset();
	}

	SystemCommands &sys() { return main->systemCommands; }

	/** Executes a command of the sys handler directly. Replies are stored in `replies` (cleared first) */
	CommandStatus run(FFBoardMain_commands cmd, CMDtype type, int64_t val = 0, int64_t adr = 0, CommandInterface *itf = nullptr) {
		return runId(static_cast<uint32_t>(cmd), type, val, adr, itf);
	}
	CommandStatus runId(uint32_t cmdId, CMDtype type, int64_t val = 0, int64_t adr = 0, CommandInterface *itf = nullptr) {
		replies.clear();
		ParsedCommand parsed;
		parsed.cmdId = cmdId;
		parsed.type = type;
		parsed.val = val;
		parsed.adr = adr;
		parsed.target = &sys();
		parsed.instance = 0;
		parsed.originalInterface = itf;
		return sys().internalCommand(parsed, replies);
	}

	/** Sends a line to the CDC command interface of the main class and returns what the firmware answered on CDC */
	std::string cdc(const std::string &line) {
		HostUsb::state().cdcTx.clear();
		std::string data = line;
		uint32_t len = data.size();
		main->cdcRcv(data.data(), &len);
		HostRtos::runUntilIdle();
		return HostUsb::state().cdcTx;
	}

	/** Runs the analog conversions of both ADCs once: VREFINT at its calibration value, the given raw inputs */
	void convertAdcs(uint32_t tempSensorRaw, uint32_t vextRaw, uint32_t vintRaw) {
		HostTarget::setAdcCalibration(VREFINT_RAW, 950, 1200);
		startADC(); // global_callbacks.cpp, called by cppmain() on the target
		HostTarget::adcSetInput(&hadc1, 6, tempSensorRaw);
		HostTarget::adcSetInput(&hadc1, 7, VREFINT_RAW);
		REQUIRE(HostTarget::adcConversionComplete(&hadc1));
		REQUIRE(HostTarget::adcConvert(&hadc2, {vextRaw, vintRaw}));
	}

	std::unique_ptr<FFBoardMain> main;
	std::vector<CommandReply> replies;
};

/** A command interface with a help string, as the help command prints the help of the interface it came from */
struct HelpInterface : CommandInterface {
	const std::string getHelpstring() override { return "test interface help"; }
	bool getNewCommands(std::vector<ParsedCommand> &) override { return false; }
	void sendReplies(const std::vector<CommandResult> &, CommandInterface *) override {}
};

/** Records every reply sent to the command interfaces (broadcasts of the error printer) */
struct RecordingInterface : CommandInterface {
	bool getNewCommands(std::vector<ParsedCommand> &) override { return false; }
	void sendReplies(const std::vector<CommandResult> &r, CommandInterface *) override { results.insert(results.end(), r.begin(), r.end()); }
	std::vector<CommandResult> results;
};

/** Counts saveFlash calls */
struct CountingStorage : PersistentStorage {
	void saveFlash() override { saves++; }
	uint32_t saves = 0;
};

// Test registry for lsmain: a visible, a hidden, a debug and a (later) not creatable main class (ClassChooser.h:129-143)
ClassIdentifier visibleInfo = {.name = "Visible", .id = 0x100, .visibility = ClassVisibility::visible};
ClassIdentifier hiddenInfo = {.name = "Hidden", .id = 0x101, .visibility = ClassVisibility::hidden};
ClassIdentifier debugInfo = {.name = "Debug", .id = 0x102, .visibility = ClassVisibility::debug};
ClassIdentifier lockedInfo = {.name = "Locked", .id = 0x103, .visibility = ClassVisibility::visible};
bool lockedCreatable = true;

const std::vector<class_entry<FFBoardMain>> &testRegistry() {
	static const std::vector<class_entry<FFBoardMain>> registry = {
		{visibleInfo, 0x10, []() -> FFBoardMain * { return new FFBoardMain(); }},
		{hiddenInfo, 0x11, []() -> FFBoardMain * { return new FFBoardMain(); }},
		{debugInfo, 0x12, []() -> FFBoardMain * { return new FFBoardMain(); }},
		{lockedInfo, 0x13, []() -> FFBoardMain * { return new FFBoardMain(); }, []() -> bool { return lockedCreatable; }},
	};
	return registry;
}

/**
 * Replaces the global main class chooser of cppmain.cpp with the test registry (restored at the end) and creates the
 * running main class with the given selection id through it, like cppmain.cpp does.
 */
struct TestRegistryFixture : TargetFixture {
	void start(uint16_t selectionId) {
		lockedCreatable = true;
		mainchooser = ClassChooser<FFBoardMain>(testRegistry());
		main.reset(mainchooser.Create(selectionId));
		REQUIRE(main != nullptr);
		REQUIRE(main->getSelectionID() == selectionId);
		mainclass = main.get();
	}
	~TestRegistryFixture() override {
		mainclass = nullptr;
		main.reset();
		mainchooser = ClassChooser<FFBoardMain>(class_registry);
		lockedCreatable = true;
	}
	std::vector<CommandReply> lsmain() {
		std::vector<CommandReply> replies;
		ParsedCommand parsed;
		parsed.cmdId = static_cast<uint32_t>(FFBoardMain_commands::lsmain);
		parsed.type = CMDtype::get;
		parsed.target = &main->systemCommands;
		CHECK(main->systemCommands.internalCommand(parsed, replies) == CommandStatus::OK);
		return replies;
	}
	std::unique_ptr<FFBoardMain> main;
};

std::vector<std::string> texts(const std::vector<CommandReply> &replies) {
	std::vector<std::string> t;
	for (const CommandReply &r : replies) {
		t.push_back(r.reply);
	}
	return t;
}

uint32_t id(FFBoardMain_commands cmd) { return static_cast<uint32_t>(cmd); }

bool contains(const std::string &s, const std::string &part) { return s.find(part) != std::string::npos; }

const Error overtempError(ErrorCode::overtemp, ErrorType::critical, "hot");
const Error undervoltageError(ErrorCode::undervoltage, ErrorType::warning, "");
const Error infoError(ErrorCode::cmdExecutionError, ErrorType::temporary, "once");

} // namespace

TEST_SUITE("SystemCommands") {

// ---------------------------------------------------------------------------
// Handler identity and command table
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(TargetFixture, "the system command handler is the hidden internal class sys with class id 0x10") {
	SystemCommands sys;
	CHECK(std::string(sys.getInfo().name) == "System Commands");
	CHECK(sys.getInfo().id == CLSID_SYSTEM);
	CHECK(sys.getInfo().visibility == ClassVisibility::hidden);
	CHECK(sys.getClassType() == ClassType::Internal);
	CHECK(std::string(sys.getCommandHandlerInfo()->clsname) == "sys");
	CHECK(sys.getCommandHandlerInfo()->clsTypeid == CMDCLSID_SYS);
	CHECK(sys.getCommandHandlerInfo()->instance == 0);
	CHECK(CommandHandler::getHandlerFromClassName("sys", 0) == &sys);
}

TEST_CASE_FIXTURE(TargetFixture, "a system command handler is the global instance until it is deleted") {
	CHECK(SystemCommands::systemCommandsInstance == nullptr);
	{
		SystemCommands sys;
		CHECK(SystemCommands::systemCommandsInstance == &sys);
	}
	CHECK(SystemCommands::systemCommandsInstance == nullptr);
}

TEST_CASE_FIXTURE(TargetFixture, "every system command is registered with its id and access flags") {
	SystemCommands sys;
	struct Entry {
		const char *name;
		uint32_t id;
		uint32_t flags;
	};
	const Entry table[] = {
		{"help", id(FFBoardMain_commands::help), CMDFLAG_STR_ONLY},
		{"save", id(FFBoardMain_commands::save), CMDFLAG_GET},
		{"reboot", id(FFBoardMain_commands::reboot), CMDFLAG_GET},
		{"dfu", id(FFBoardMain_commands::dfu), CMDFLAG_GET},
		{"lsmain", id(FFBoardMain_commands::lsmain), CMDFLAG_GET},
		{"lsactive", id(FFBoardMain_commands::lsactive), CMDFLAG_GET},
		{"vint", id(FFBoardMain_commands::vint), CMDFLAG_GET},
		{"vext", id(FFBoardMain_commands::vext), CMDFLAG_GET},
		{"main", id(FFBoardMain_commands::main), CMDFLAG_GET | CMDFLAG_SET},
		{"swver", id(FFBoardMain_commands::swver), CMDFLAG_GET},
		{"hwtype", id(FFBoardMain_commands::hwtype), CMDFLAG_GET},
		{"flashraw", id(FFBoardMain_commands::flashraw), CMDFLAG_SETADR | CMDFLAG_GETADR},
		{"flashdump", id(FFBoardMain_commands::flashdump), CMDFLAG_GET},
		{"errors", id(FFBoardMain_commands::errors), CMDFLAG_GET},
		{"errorsclr", id(FFBoardMain_commands::errorsclr), CMDFLAG_GET},
		{"heapfree", id(FFBoardMain_commands::heapfree), CMDFLAG_GET},
		{"format", id(FFBoardMain_commands::format), CMDFLAG_SET},
		{"debug", id(FFBoardMain_commands::debug), CMDFLAG_SET | CMDFLAG_GET},
		{"devid", id(FFBoardMain_commands::devid), CMDFLAG_GET},
		{"name", static_cast<uint32_t>(CommandHandlerCommands::name), CMDFLAG_GET | CMDFLAG_STR_ONLY},
		{"cmdinfo", static_cast<uint32_t>(CommandHandlerCommands::cmdinfo), CMDFLAG_GETADR},
		{"uid", id(FFBoardMain_commands::uid), CMDFLAG_GET | CMDFLAG_GETADR | CMDFLAG_INFOSTRING},
		{"temp", id(FFBoardMain_commands::temp), CMDFLAG_GET},
		{"signature", id(FFBoardMain_commands::signature), CMDFLAG_GETADR | CMDFLAG_SETADR | CMDFLAG_GET | CMDFLAG_INFOSTRING},
	};
	for (const Entry &e : table) {
		CAPTURE(e.name);
		CmdHandlerCommanddef *def = sys.getCommandFromName(e.name);
		REQUIRE(def != nullptr);
		CHECK(def->cmdId == e.id);
		CHECK(def->flags == e.flags);
		REQUIRE(def->helpstring != nullptr);
		CHECK(std::string(def->helpstring).size() > 0);
	}
}

TEST_CASE_FIXTURE(TargetFixture, "the system command handler has no generic id, instance and command handler uid commands") {
	SystemCommands sys;
	CHECK(sys.getCommandFromName("id") == nullptr);
	CHECK(sys.getCommandFromName("instance") == nullptr);
	CHECK(sys.getCommandFromName("cmduid") == nullptr);
}

TEST_CASE_FIXTURE(TargetFixture, "the mallinfo command id has no entry in the command table") {
	SystemCommands sys;
	CHECK(sys.getCommandFromId(id(FFBoardMain_commands::mallinfo)) == nullptr);
	CHECK(sys.getCommandFromName("mallinfo") == nullptr);
}

TEST_CASE_FIXTURE(SysFixture, "a command id the system handler does not know is not found and gives no reply") {
	CHECK(runId(9999, CMDtype::get) == CommandStatus::NOT_FOUND);
	CHECK(replies.empty());
}

// ---------------------------------------------------------------------------
// Generic commands of the command handler
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(SysFixture, "the name command returns the name of the system command handler") {
	CHECK(runId(static_cast<uint32_t>(CommandHandlerCommands::name), CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::STRING);
	CHECK(replies[0].reply == "System Commands");
}

TEST_CASE_FIXTURE(SysFixture, "cmdinfo returns the flags of a system command") {
	CHECK(runId(static_cast<uint32_t>(CommandHandlerCommands::cmdinfo), CMDtype::getat, 0, id(FFBoardMain_commands::flashraw)) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == (CMDFLAG_SETADR | CMDFLAG_GETADR));
}

TEST_CASE_FIXTURE(SysFixture, "cmdinfo returns -1 for an unknown command id") {
	CHECK(runId(static_cast<uint32_t>(CommandHandlerCommands::cmdinfo), CMDtype::getat, 0, 9999) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == -1);
}

// ---------------------------------------------------------------------------
// help
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(SysFixture, "help lists the help of the interface, all active command handlers and the system commands") {
	HelpInterface itf;
	CHECK(run(FFBoardMain_commands::help, CMDtype::get, 0, 0, &itf) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::STRING);
	const std::string &help = replies[0].reply;
	CHECK(help.rfind("test interface help\nAvailable classes (use cls.0.help for more info):\n", 0) == 0);
	CHECK(contains(help, "\nsys.0\n"));
	CHECK(contains(help, "\nmain.0\n"));
	for (CommandHandler *handler : CommandHandler::getCommandHandlers()) {
		CmdHandlerInfo *info = handler->getCommandHandlerInfo();
		CHECK(contains(help, std::string(info->clsname) + "." + std::to_string(info->instance) + "\n"));
	}
	CHECK(contains(help, "\n" + sys().getCommandsHelpstring()));
	CHECK(contains(help, "swver\t( R )\tFirmware version\n"));
}

TEST_CASE_FIXTURE(SysFixture, "help without an interface starts with the list of classes") {
	CHECK(run(FFBoardMain_commands::help, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply.rfind("Available classes (use cls.0.help for more info):\n", 0) == 0);
}

TEST_CASE_FIXTURE(SysFixture, "help as info request returns the command list as CSV") {
	CHECK(run(FFBoardMain_commands::help, CMDtype::info) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == sys().getCsvHelpstring());
	CHECK(replies[0].reply.rfind("\nPrefix,Class ID, Class description\nsys.0,0x0,System Commands\n", 0) == 0);
	CHECK(contains(replies[0].reply, "\nswver,0x4,Firmware version, R\n"));
}

// ---------------------------------------------------------------------------
// Versions and chip identification
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(SysFixture, "swver returns the firmware version as text and as one integer of 8 bits per part") {
	CHECK(run(FFBoardMain_commands::swver, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::STRING_OR_INT);
	CHECK(replies[0].reply == std::to_string(SW_VERSION_INT[0]) + "." + std::to_string(SW_VERSION_INT[1]) + "." + std::to_string(SW_VERSION_INT[2]));
	CHECK(replies[0].val == ((SW_VERSION_INT[0] << 16) | (SW_VERSION_INT[1] << 8) | SW_VERSION_INT[2]));
	CHECK(replies[0].reply == "1.17.1"); // SW_VERSION_INT of constants.h at the time of writing
	CHECK(replies[0].val == 0x011101);
}

TEST_CASE_FIXTURE(SysFixture, "hwtype returns the hardware name and number of the target") {
	CHECK(run(FFBoardMain_commands::hwtype, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::STRING_OR_INT);
	CHECK(replies[0].reply == "F407VG");
	CHECK(replies[0].val == 2);
}

TEST_CASE_FIXTURE(SysFixture, "devid returns the device id with the revision id as second value") {
	HostTarget::state().devId = 0x419;
	HostTarget::state().revId = 0x2001;
	CHECK(run(FFBoardMain_commands::devid, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::DOUBLEINTS);
	CHECK(replies[0].val == 0x419);
	CHECK(replies[0].adr == 0x2001);
}

TEST_CASE_FIXTURE(SysFixture, "uid returns the lower 64 bits of the unique id as value and the upper 32 bits as second value") {
	HostTarget::state().uid[0] = 0x89ABCDEF;
	HostTarget::state().uid[1] = 0x01234567;
	HostTarget::state().uid[2] = 0xF00DCAFE;
	CHECK(run(FFBoardMain_commands::uid, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::DOUBLEINTS);
	CHECK(static_cast<uint64_t>(replies[0].val) == 0x0123456789ABCDEFULL);
	CHECK(replies[0].adr == 0xF00DCAFE);
}

TEST_CASE_FIXTURE(SysFixture, "uid with an address returns the selected 32 bit word of the unique id") {
	HostTarget::state().uid[0] = 0x11111111;
	HostTarget::state().uid[1] = 0x22222222;
	HostTarget::state().uid[2] = 0x33333333;
	for (int64_t word = 0; word < 3; word++) {
		CAPTURE(word);
		CHECK(run(FFBoardMain_commands::uid, CMDtype::getat, 0, word) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		CHECK(replies[0].type == CommandReplyType::INT);
		CHECK(replies[0].val == static_cast<int64_t>(HostTarget::state().uid[word]));
	}
}

// NEEDS REVIEW 54: uid?3 (no such word) succeeds without a reply instead of being rejected like signature?8
TEST_CASE_FIXTURE(SysFixture, "characterization: uid with an address above 2 succeeds without a reply (NEEDS REVIEW 54)") {
	CHECK(run(FFBoardMain_commands::uid, CMDtype::getat, 0, 3) == CommandStatus::OK);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(SysFixture, "uid as info request returns the 96 bit unique id as one hex string, most significant word first") {
	HostTarget::state().uid[0] = 0x0000ABCD;
	HostTarget::state().uid[1] = 0x12345678;
	HostTarget::state().uid[2] = 0x9ABCDEF0;
	// SystemCommands.cpp:307 prints the uint32_t words with %08lx: correct on ARM (arm-none-eabi: uint32_t is unsigned
	// long); on MinGW uint32_t is unsigned int, which works because long is 32 bit there (LLP64); on LP64 Linux it
	// only works because the 32 bit argument is passed zero-extended in a 64 bit register
	CHECK(run(FFBoardMain_commands::uid, CMDtype::info) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::STRING);
	CHECK(replies[0].reply == "0x9abcdef0123456780000abcd");
}

// ---------------------------------------------------------------------------
// Signature (OTP model of ffb_target_tests: erased slots read 0xFFFFFFFFFFFFFFFF, OTP_Write only writes erased slots)
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(SysFixture, "signature with an address beyond the signature length is rejected") {
	CHECK(run(FFBoardMain_commands::signature, CMDtype::getat, 0, SIGNATURELEN) == CommandStatus::ERR);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(SysFixture, "signature with a negative address is rejected") {
	CHECK(run(FFBoardMain_commands::signature, CMDtype::getat, 0, -1) == CommandStatus::ERR);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(SysFixture, "a signature write that the OTP memory does not accept is reported as error") {
	HostTarget::setOtpSlot(SIGBNATUREBASEADR, 0); // already programmed: OTP_Write only writes an erased slot
	CHECK(run(FFBoardMain_commands::signature, CMDtype::setat, 0x1234, 0) == CommandStatus::ERR);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(SysFixture, "signature with an address writes the value to that slot of the signature area") {
	CHECK(run(FFBoardMain_commands::signature, CMDtype::setat, 0x0123456789ABCDEFLL, 3) == CommandStatus::OK);
	CHECK(replies.empty());
	CHECK(HostTarget::otpSlot(SIGBNATUREBASEADR + 3) == 0x0123456789ABCDEFULL);
	CHECK(HostTarget::otpSlot(SIGBNATUREBASEADR + 2) == 0xFFFFFFFFFFFFFFFFULL);
	CHECK(HostTarget::otpSlot(SIGBNATUREBASEADR + 4) == 0xFFFFFFFFFFFFFFFFULL);
}

TEST_CASE_FIXTURE(SysFixture, "signature returns the 8 signature slots as value with the slot number as second value") {
	for (uint16_t i = 0; i < SIGNATURELEN; i++) {
		HostTarget::setOtpSlot(SIGBNATUREBASEADR + i, 0x1111111111111111ULL * (i + 1));
	}
	CHECK(run(FFBoardMain_commands::signature, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == SIGNATURELEN);
	for (uint16_t i = 0; i < SIGNATURELEN; i++) {
		CAPTURE(i);
		CHECK(replies[i].type == CommandReplyType::DOUBLEINTS);
		CHECK(static_cast<uint64_t>(replies[i].val) == 0x1111111111111111ULL * (i + 1));
		CHECK(replies[i].adr == i);
	}
}

TEST_CASE_FIXTURE(SysFixture, "signature with an address returns that slot with the slot number as second value") {
	HostTarget::setOtpSlot(SIGBNATUREBASEADR + 7, 0x8000000000000001ULL);
	CHECK(run(FFBoardMain_commands::signature, CMDtype::getat, 0, 7) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::DOUBLEINTS);
	CHECK(static_cast<uint64_t>(replies[0].val) == 0x8000000000000001ULL);
	CHECK(replies[0].adr == 7);
}

TEST_CASE_FIXTURE(SysFixture, "signature as info request returns the 8 slots as one hex string of 16 digits per slot") {
	HostTarget::setOtpSlot(SIGBNATUREBASEADR + 0, 0x0123456789ABCDEFULL);
	HostTarget::setOtpSlot(SIGBNATUREBASEADR + 7, 0x1ULL);
	// %016llx with uint64_t is correct on every ABI (unsigned long long is 64 bit everywhere)
	CHECK(run(FFBoardMain_commands::signature, CMDtype::info) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::STRING);
	std::string expected = "0123456789abcdef";
	for (int i = 1; i < 7; i++) {
		expected += "ffffffffffffffff";
	}
	expected += "0000000000000001";
	CHECK(replies[0].reply == expected);
}

TEST_CASE_FIXTURE(SysFixture, "signature reads only the signature slots") {
	run(FFBoardMain_commands::signature, CMDtype::get);
	const auto &calls = HostTarget::state().otpCalls;
	REQUIRE(calls.size() == SIGNATURELEN);
	for (uint16_t i = 0; i < SIGNATURELEN; i++) {
		CHECK_FALSE(calls[i].write);
		CHECK(calls[i].offset == SIGBNATUREBASEADR + i);
		CHECK(calls[i].result); // OTP_Read never fails for these offsets, so dat is always written
	}
}

// NEEDS REVIEW 55: unlike getat, setat has no range check against SIGNATURELEN: any OTP slot can be written, up to the
// lock bytes at offsets 64/65 (bug 41)
TEST_CASE_FIXTURE(SysFixture, "characterization: signature with an address beyond the signature length writes that OTP slot (NEEDS REVIEW 55)") {
	CHECK(run(FFBoardMain_commands::signature, CMDtype::setat, 0x55, SIGNATURELEN) == CommandStatus::OK);
	CHECK(HostTarget::otpSlot(SIGBNATUREBASEADR + SIGNATURELEN) == 0x55);
}

// NEEDS REVIEW 55: the help text says "set=1 to lock", but a plain set is rejected and nothing locks the OTP blocks
TEST_CASE_FIXTURE(SysFixture, "characterization: a plain signature set is rejected although the help offers set=1 to lock (NEEDS REVIEW 55)") {
	CHECK(run(FFBoardMain_commands::signature, CMDtype::set, 1) == CommandStatus::ERR);
	CHECK(replies.empty());
	CHECK(contains(sys().getCommandFromName("signature")->helpstring, "set=1 to lock"));
}

// ---------------------------------------------------------------------------
// Analog values
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(SysFixture, "vint returns the internal voltage in mV") {
	convertAdcs(950, 0, 1241); // 1241 of 4095 at 3300 mV = 1000 mV at the pin, times VOLTAGE_MULT_DEFAULT 30.67
	CHECK(run(FFBoardMain_commands::vint, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::INT);
	CHECK(replies[0].val == getIntV());
	CHECK(replies[0].val == 30670);
}

TEST_CASE_FIXTURE(SysFixture, "vext returns the external supply voltage in mV") {
	convertAdcs(950, 2482, 0);
	CHECK(run(FFBoardMain_commands::vext, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::INT);
	CHECK(replies[0].val == getExtV());
	CHECK(replies[0].val == 61340);
}

TEST_CASE_FIXTURE(SysFixture, "temp returns the chip temperature in degrees C") {
	convertAdcs(1200, 0, 0); // TS_CAL2 = 110 C
	CHECK(run(FFBoardMain_commands::temp, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::INT);
	CHECK(replies[0].val == getChipTemp());
	CHECK(replies[0].val == 110);
}

// ---------------------------------------------------------------------------
// Memory
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(SysFixture, "heapfree returns the free heap with the minimum ever free heap as second value") {
	CHECK(run(FFBoardMain_commands::heapfree, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::DOUBLEINTS);
	CHECK(replies[0].val == static_cast<int64_t>(xPortGetFreeHeapSize()));
	CHECK(replies[0].adr == static_cast<int64_t>(xPortGetMinimumEverFreeHeapSize()));
}

// NEEDS REVIEW 56: mallinfo has a handler (SystemCommands.cpp:224-237) but no command table entry, so it is only
// reachable by calling internalCommand directly (the command thread drops commands without an entry). Dead code?
TEST_CASE_FIXTURE(SysFixture, "characterization: the unregistered mallinfo id answers with the heap usage when called directly (NEEDS REVIEW 56)") {
	CHECK(run(FFBoardMain_commands::mallinfo, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::STRING_OR_DOUBLEINT);
	CHECK(replies[0].reply.rfind("Usage: " + std::to_string(replies[0].val) + " Size: ", 0) == 0);
	CHECK(replies[0].adr == replies[0].val);
}

// ---------------------------------------------------------------------------
// Debug mode
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(SysFixture, "debug=1 enables the debug mode") {
	CHECK(run(FFBoardMain_commands::debug, CMDtype::set, 1) == CommandStatus::OK);
	CHECK(SystemCommands::debugMode);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(SysFixture, "debug with a value other than 0 or 1 enables the debug mode") {
	CHECK(run(FFBoardMain_commands::debug, CMDtype::set, 2) == CommandStatus::OK);
	CHECK(SystemCommands::debugMode);
	run(FFBoardMain_commands::debug, CMDtype::get);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 1);
}

TEST_CASE_FIXTURE(SysFixture, "debug=0 disables the debug mode") {
	SystemCommands::debugMode = true;
	CHECK(run(FFBoardMain_commands::debug, CMDtype::set, 0) == CommandStatus::OK);
	CHECK_FALSE(SystemCommands::debugMode);
}

TEST_CASE_FIXTURE(SysFixture, "debug returns the debug mode") {
	SystemCommands::debugMode = true;
	CHECK(run(FFBoardMain_commands::debug, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 1);
}

TEST_CASE_FIXTURE(SysFixture, "debug as info request is rejected") {
	CHECK(run(FFBoardMain_commands::debug, CMDtype::info) == CommandStatus::ERR);
	CHECK(replies.empty());
	CHECK_FALSE(SystemCommands::debugMode);
}

// ---------------------------------------------------------------------------
// Reset, DFU, save
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(SysFixture, "reboot resets the chip") {
	CHECK(run(FFBoardMain_commands::reboot, CMDtype::get) == CommandStatus::OK); // host only: NVIC_SystemReset returns
	CHECK(HostTarget::state().systemResets == 1);
	CHECK(HostTarget::state().dfuReboots == 0);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(SysFixture, "dfu reboots into the DFU bootloader") {
	CHECK(run(FFBoardMain_commands::dfu, CMDtype::get) == CommandStatus::OK); // host only: RebootDFU returns
	CHECK(HostTarget::state().dfuReboots == 1);
	CHECK(HostTarget::state().systemResets == 1); // property of the fake: its RebootDFU ends in NVIC_SystemReset like main.c
}

TEST_CASE_FIXTURE(SysFixture, "save writes the settings of every persistent storage handler to flash") {
	CountingStorage a;
	CountingStorage b;
	CHECK(run(FFBoardMain_commands::save, CMDtype::get) == CommandStatus::OK);
	CHECK(a.saves == 1);
	CHECK(b.saves == 1);
	CHECK(replies.empty());
	CHECK(HostTarget::state().systemResets == 0);
}

// ---------------------------------------------------------------------------
// Main class list and selection
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(SysFixture, "lsmain lists every main class of the registry with selection id, creatable flag and name") {
	CHECK(run(FFBoardMain_commands::lsmain, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == class_registry.size());
	for (size_t i = 0; i < class_registry.size(); i++) {
		const class_entry<FFBoardMain> &e = class_registry[i];
		CAPTURE(e.info.name);
		CHECK(replies[i].reply == std::to_string(e.selectionId) + ":1:" + e.info.name);
		CHECK(replies[i].type == CommandReplyType::STRING_OR_DOUBLEINT);
		CHECK(replies[i].adr == e.selectionId);
		CHECK(replies[i].val == e.info.id);
	}
	CHECK(replies[0].reply == "0:1:Basic (Failsafe)");
	CHECK(replies[1].reply == "1:1:FFB Wheel (1 Axis)");
}

TEST_CASE_FIXTURE(TestRegistryFixture, "lsmain leaves out hidden and debug main classes") {
	start(0x10);
	std::vector<CommandReply> replies = lsmain();
	CHECK(texts(replies) == std::vector<std::string>{"16:1:Visible", "19:1:Locked"});
	REQUIRE(replies.size() == 2);
	CHECK(replies[0].type == CommandReplyType::STRING_OR_DOUBLEINT);
	CHECK(replies[0].adr == 0x10);
	CHECK(replies[0].val == 0x100);
}

TEST_CASE_FIXTURE(TestRegistryFixture, "lsmain lists debug main classes in debug mode") {
	start(0x10);
	SystemCommands::debugMode = true;
	CHECK(texts(lsmain()) == std::vector<std::string>{"16:1:Visible", "18:1:Debug", "19:1:Locked"});
}

TEST_CASE_FIXTURE(TestRegistryFixture, "lsmain lists a hidden main class while it is running") {
	start(0x11);
	CHECK(texts(lsmain()) == std::vector<std::string>{"16:1:Visible", "17:1:Hidden", "19:1:Locked"});
}

TEST_CASE_FIXTURE(TestRegistryFixture, "lsmain lists a main class that can not be created as text only with creatable flag 0") {
	start(0x10);
	lockedCreatable = false;
	std::vector<CommandReply> replies = lsmain();
	REQUIRE(replies.size() == 2);
	CHECK(replies[1].reply == "19:0:Locked");
	CHECK(replies[1].type == CommandReplyType::STRING);
}

TEST_CASE_FIXTURE(TestRegistryFixture, "lsmain lists the running main class as creatable even if it can not be created again") {
	start(0x13);
	lockedCreatable = false;
	std::vector<CommandReply> replies = lsmain();
	REQUIRE(replies.size() == 2);
	CHECK(replies[1].reply == "19:1:Locked");
	CHECK(replies[1].type == CommandReplyType::STRING);
}

TEST_CASE_FIXTURE(TargetFixture, "main returns the id of the running main class when none is stored in flash") {
	CustomMain custom; // a running main class with an id other than 0
	mainclass = &custom;
	std::vector<CommandReply> replies;
	ParsedCommand parsed;
	parsed.cmdId = id(FFBoardMain_commands::main);
	parsed.type = CMDtype::get;
	parsed.target = &custom.systemCommands;

	CHECK(custom.systemCommands.internalCommand(parsed, replies) == CommandStatus::OK);
	mainclass = nullptr;
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::INT);
	CHECK(replies[0].val == CLSID_CUSTOM);
}

TEST_CASE_FIXTURE(SysFixture, "main returns the main class id stored in flash") {
	HostPlatform::flash()[ADR_CURRENT_CONFIG] = FFBJoystick::info.id;
	CHECK(run(FFBoardMain_commands::main, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == FFBJoystick::info.id);
}

TEST_CASE_FIXTURE(SysFixture, "selecting another main class stores its id and resets the chip") {
	// Host only: NVIC_SystemReset returns and the command ends with OK; the target restarts with the new class
	CHECK(run(FFBoardMain_commands::main, CMDtype::set, FFBWheel::info.id) == CommandStatus::OK);
	CHECK(HostPlatform::flash().at(ADR_CURRENT_CONFIG) == FFBWheel::info.id);
	CHECK(HostTarget::state().systemResets == 1);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(SysFixture, "selecting the running main class stores its id without a reset") {
	HostPlatform::flash()[ADR_CURRENT_CONFIG] = FFBWheel::info.id;
	CHECK(run(FFBoardMain_commands::main, CMDtype::set, 0) == CommandStatus::OK);
	CHECK(HostPlatform::flash().at(ADR_CURRENT_CONFIG) == 0);
	CHECK(HostTarget::state().systemResets == 0);
}

// NEEDS REVIEW 57: an invalid main class id is ignored but the command reports OK (other commands answer ERR)
TEST_CASE_FIXTURE(SysFixture, "characterization: selecting a main class id that is not in the registry changes nothing and reports OK (NEEDS REVIEW 57)") {
	HostPlatform::flash()[ADR_CURRENT_CONFIG] = FFBWheel::info.id;
	CHECK(run(FFBoardMain_commands::main, CMDtype::set, 200) == CommandStatus::OK);
	CHECK(HostPlatform::flash().at(ADR_CURRENT_CONFIG) == FFBWheel::info.id);
	CHECK(HostTarget::state().systemResets == 0);
}

// NEEDS REVIEW 58: the stored id is the lower 16 bits (0), but SystemCommands.cpp:161 compares the 64 bit value with
// the running id 0, so selecting the running class this way still resets the chip
TEST_CASE_FIXTURE(SysFixture, "characterization: selecting the running main class with an id above 16 bits resets the chip (NEEDS REVIEW 58)") {
	CHECK(run(FFBoardMain_commands::main, CMDtype::set, 0x10000) == CommandStatus::OK);
	CHECK(HostPlatform::flash().at(ADR_CURRENT_CONFIG) == 0);
	CHECK(HostTarget::state().systemResets == 1);
}

// NEEDS REVIEW 58: isValidClassId takes a uint16_t, so 0x10001 is accepted as class 1 and stored as 1; it then differs
// from the running id 1 in the 64 bit comparison of SystemCommands.cpp:161, so even the running class would reset
TEST_CASE_FIXTURE(SysFixture, "characterization: a main class id above 16 bits selects the class of its lower 16 bits (NEEDS REVIEW 58)") {
	CHECK(run(FFBoardMain_commands::main, CMDtype::set, 0x10000 + FFBWheel::info.id) == CommandStatus::OK);
	CHECK(HostPlatform::flash().at(ADR_CURRENT_CONFIG) == FFBWheel::info.id);
	CHECK(HostTarget::state().systemResets == 1);
}

TEST_CASE_FIXTURE(SysFixture, "lsactive lists every active command handler as name, class name, instance, class id and handler id") {
	CHECK(run(FFBoardMain_commands::lsactive, CMDtype::get) == CommandStatus::OK);
	const std::vector<CommandHandler *> &handlers = CommandHandler::getCommandHandlers();
	REQUIRE(replies.size() == handlers.size());
	for (size_t i = 0; i < handlers.size(); i++) {
		CmdHandlerInfo *hi = handlers[i]->getCommandHandlerInfo();
		CAPTURE(hi->clsname);
		CHECK(replies[i].type == CommandReplyType::STRING_OR_DOUBLEINT);
		CHECK(replies[i].adr == hi->instance);
		CHECK(replies[i].val == hi->clsTypeid);
	}
	auto sysReply = std::find_if(replies.begin(), replies.end(), [](const CommandReply &r) { return r.reply.rfind("System Commands:", 0) == 0; });
	REQUIRE(sysReply != replies.end());
	CHECK(sysReply->reply == "System Commands:sys:0:16:" + std::to_string(sys().getCommandHandlerID()));
	CHECK(sysReply->val == CMDCLSID_SYS);
	auto mainReply = std::find_if(replies.begin(), replies.end(), [](const CommandReply &r) { return r.reply.rfind("Basic (Failsafe):", 0) == 0; });
	REQUIRE(mainReply != replies.end());
	CHECK(mainReply->reply == "Basic (Failsafe):main:0:0:" + std::to_string(main->getCommandHandlerID()));
	CHECK(mainReply->val == CMDCLSID_MAIN);
}

// ---------------------------------------------------------------------------
// Flash
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(SysFixture, "flashraw with an address writes the value to that flash address") {
	CHECK(run(FFBoardMain_commands::flashraw, CMDtype::setat, 0x4567, 0x123) == CommandStatus::OK);
	CHECK(HostPlatform::flash().at(0x123) == 0x4567);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(SysFixture, "flashraw with an address reads the value stored at that flash address") {
	HostPlatform::flash()[0x321] = 0xBEEF;
	CHECK(run(FFBoardMain_commands::flashraw, CMDtype::getat, 0, 0x321) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::INT);
	CHECK(replies[0].val == 0xBEEF);
}

// NEEDS REVIEW 59: reading an address that is not stored answers 0 with OK, indistinguishable from a stored 0
TEST_CASE_FIXTURE(SysFixture, "characterization: flashraw reading an address that is not stored replies 0 (NEEDS REVIEW 59)") {
	CHECK(run(FFBoardMain_commands::flashraw, CMDtype::getat, 0, 0x321) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::INT);
	CHECK(replies[0].val == 0);
}

// NEEDS REVIEW 64: flashraw passes the 64 bit address and value to Flash_Write(uint16_t, uint16_t) without a range
// check: 0x10005?0x10007 silently stores 5 at address 7 and a negative address wraps, both with OK
TEST_CASE_FIXTURE(SysFixture, "characterization: flashraw truncates address and value to 16 bits (NEEDS REVIEW 64)") {
	CHECK(run(FFBoardMain_commands::flashraw, CMDtype::setat, 0x10005, 0x10007) == CommandStatus::OK);
	CHECK(run(FFBoardMain_commands::flashraw, CMDtype::setat, 9, -1) == CommandStatus::OK);
	CHECK(HostPlatform::flash() == std::map<uint16_t, uint16_t>{{7, 5}, {0xFFFF, 9}});
}

TEST_CASE_FIXTURE(SysFixture, "flashraw without an address neither reads nor writes") {
	CHECK(run(FFBoardMain_commands::flashraw, CMDtype::set, 5) == CommandStatus::OK);
	CHECK(run(FFBoardMain_commands::flashraw, CMDtype::get) == CommandStatus::OK);
	CHECK(replies.empty());
	CHECK(HostPlatform::flash().empty());
}

TEST_CASE_FIXTURE(SysFixture, "flashdump returns every stored flash variable as value and address") {
	HostPlatform::flash()[0x10] = 7;
	HostPlatform::flash()[0x20] = 0xFFFF;
	CHECK(run(FFBoardMain_commands::flashdump, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 2);
	CHECK(replies[0].type == CommandReplyType::DOUBLEINTS);
	CHECK(replies[0].adr == 0x10);
	CHECK(replies[0].val == 7);
	CHECK(replies[1].type == CommandReplyType::DOUBLEINTS);
	CHECK(replies[1].adr == 0x20);
	CHECK(replies[1].val == 0xFFFF);
}

TEST_CASE_FIXTURE(TargetFixture, "replyFlashDump of an empty flash adds no replies") {
	std::vector<CommandReply> replies;
	SystemCommands::replyFlashDump(replies);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(SysFixture, "format=1 erases all stored values") {
	HostPlatform::flash()[0x10] = 7;
	HostPlatform::flash()[ADR_CURRENT_CONFIG] = 1;
	CHECK(run(FFBoardMain_commands::format, CMDtype::set, 1) == CommandStatus::OK);
	CHECK(HostPlatform::flash().empty());
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(SysFixture, "format with a value other than 1 does not erase the flash") {
	HostPlatform::flash()[0x10] = 7;
	CHECK(run(FFBoardMain_commands::format, CMDtype::set, 2) == CommandStatus::OK);
	CHECK(run(FFBoardMain_commands::format, CMDtype::set, 0) == CommandStatus::OK);
	CHECK(HostPlatform::flash().size() == 1);
}

TEST_CASE_FIXTURE(SysFixture, "format as read request does not erase the flash") {
	HostPlatform::flash()[0x10] = 7;
	CHECK(run(FFBoardMain_commands::format, CMDtype::get, 1) == CommandStatus::OK);
	CHECK(HostPlatform::flash().size() == 1);
}

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(SysFixture, "errors replies None while there is no error") {
	CHECK(run(FFBoardMain_commands::errors, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::STRING_OR_INT);
	CHECK(replies[0].reply == "None");
	CHECK(replies[0].val == 0);
}

TEST_CASE_FIXTURE(SysFixture, "errors lists every active error as code, type and info with the code as value") {
	ErrorHandler::addError(overtempError);
	ErrorHandler::addError(undervoltageError);
	CHECK(run(FFBoardMain_commands::errors, CMDtype::get) == CommandStatus::OK);
	REQUIRE(replies.size() == 2);
	CHECK(replies[0].type == CommandReplyType::STRING_OR_INT);
	CHECK(replies[0].reply == "15:crit:hot\n");
	CHECK(replies[0].val == 15);
	CHECK(replies[1].type == CommandReplyType::STRING_OR_INT);
	CHECK(replies[1].reply == "10:warn:no info\n");
	CHECK(replies[1].val == 10);
}

TEST_CASE_FIXTURE(SysFixture, "a temporary error is listed once by errors and then cleared") {
	ErrorHandler::addError(infoError);
	run(FFBoardMain_commands::errors, CMDtype::get);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "6:info:once\n");

	run(FFBoardMain_commands::errors, CMDtype::get);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "None");
}

// Suspected bug 7 (ErrorHandler.cpp:130-143): replyErrors ends with clearTemp, whose sortErrors moves the empty slots
// (code 0) in front of the stored errors; getErrors stops at the first empty slot.
TEST_CASE_FIXTURE(SysFixture, "a persistent error is still listed by a second errors command" * KNOWN_ISSUE("errors after clearTemp: empty slots sorted to the front hide the stored errors (bug 7, ErrorHandler.cpp:130-143)")) {
	ErrorHandler::addError(overtempError);
	run(FFBoardMain_commands::errors, CMDtype::get);
	REQUIRE(replies.size() == 1);
	REQUIRE(replies[0].reply == "15:crit:hot\n");

	run(FFBoardMain_commands::errors, CMDtype::get);
	REQUIRE_ALL_HAL_MODELLED();
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "15:crit:hot\n"); // defect: "None"
}

TEST_CASE_FIXTURE(SysFixture, "errorsclr clears all errors") {
	ErrorHandler::addError(overtempError);
	ErrorHandler::addError(undervoltageError);
	CHECK(run(FFBoardMain_commands::errorsclr, CMDtype::get) == CommandStatus::OK);
	CHECK(replies.empty());
	CHECK(ErrorHandler::getErrors().empty());
	run(FFBoardMain_commands::errors, CMDtype::get);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "None");
}

TEST_CASE_FIXTURE(SysFixture, "the error printer broadcasts a new error as errors reply of the system command handler") {
	RecordingInterface itf;
	TaskHandle_t printer = HostRtos::findTask("errprint");
	REQUIRE(printer != nullptr);
	HostRtos::runUntilBlocked(printer); // first pass: no error yet
	itf.results.clear();

	ErrorHandler::addError(overtempError);
	HostRtos::runUntilBlocked(printer);

	REQUIRE(itf.results.size() == 1);
	CHECK(itf.results[0].commandHandler == &sys());
	CHECK(itf.results[0].originalCommand.cmdId == id(FFBoardMain_commands::errors));
	CHECK(itf.results[0].originalCommand.type == CMDtype::get);
	REQUIRE(itf.results[0].reply.size() == 1);
	CHECK(itf.results[0].reply[0].reply == "15:crit:hot\n");
}

// ---------------------------------------------------------------------------
// End to end through the CDC command interface and the command thread (access flags of the command table)
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(SysFixture, "sys.hwtype? on the CDC port answers with the hardware type") {
	CHECK(cdc("sys.hwtype?\n") == "[sys.hwtype?|F407VG]\n");
}

TEST_CASE_FIXTURE(SysFixture, "sys.flashraw=value?address on the CDC port writes the flash") {
	cdc("sys.flashraw=42?7\n");
	CHECK(HostPlatform::flash().at(7) == 42);
}

TEST_CASE_FIXTURE(SysFixture, "sys.format? on the CDC port is not executed because format is write only") {
	HostPlatform::flash()[0x10] = 7;
	CHECK(cdc("sys.format?\n") == ""); // not executed: no reply at all (executed it would be "[sys.format?|OK]")
	CHECK(HostPlatform::flash().size() == 1);
}

TEST_CASE_FIXTURE(SysFixture, "sys.reboot=1 on the CDC port is not executed because reboot is read only") {
	cdc("sys.reboot=1\n");
	CHECK(HostTarget::state().systemResets == 0);
}

TEST_CASE_FIXTURE(SysFixture, "sys.reboot? on the CDC port resets the chip") {
	// Positive control of the access flag cases: the same command with an allowed type is executed
	cdc("sys.reboot?\n");
	CHECK(HostTarget::state().systemResets == 1);
}

TEST_CASE_FIXTURE(SysFixture, "sys.errorsclr=1 on the CDC port is not executed because errorsclr is read only") {
	// errorsclr ignores the command type, so only the access flag check of the command thread keeps the errors
	ErrorHandler::addError(overtempError);
	CHECK(cdc("sys.errorsclr=1\n") == "");
	CHECK(ErrorHandler::getErrors().size() == 1);
}

} // TEST_SUITE
