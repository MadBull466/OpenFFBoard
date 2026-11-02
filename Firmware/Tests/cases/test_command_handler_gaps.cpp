/*
 * Command handler base class (CommandHandler.cpp) and command interface base class / CDC / UART
 * details (CommandInterface.cpp), additional cases (P1)
 *
 *  - Default implementations of CommandHandler (command, getHelpstring), instance handling,
 *    logging via CDC, help strings of handlers without commands, flag overrides
 *  - CommandInterface default readyToSend, destruction through a base pointer
 *  - CDC: replies are sent early when the send buffer is full; UART: replies wait while the port is busy
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "FirmwareFixture.h"
#include "MockCommandHandler.h"
#include "MockCommandInterface.h"

#include "CommandHandler.h"
#include "CommandInterface.h"
#include "cpp_target_config.h"

#include <algorithm>
#include <memory>

namespace {

constexpr uint16_t BARE_CLSID = 0x7E20;

enum class BareCmd : uint32_t { plain = 1, rw = 2, other = 3 };

/**
 * Command handler that only uses the base class implementations (no command(), no getHelpstring()).
 * Exposes the protected helpers that firmware classes call from their constructors.
 */
class BareHandler : public CommandHandler {
public:
	explicit BareHandler(uint8_t instance = 0, const char *displayName = "Bare handler")
		: CommandHandler("bare", BARE_CLSID, instance), displayName(displayName) {}

	const ClassIdentifier getInfo() override { return ClassIdentifier{displayName, BARE_CLSID, ClassVisibility::visible}; }

	using CommandHandler::overrideCommandFlags_INT;
	using CommandHandler::setInstance;

private:
	const char *displayName;
};

/** Command interface that keeps the default readyToSend, hasNewCommands and batchDone */
class PlainInterface : public CommandInterface {
public:
	bool getNewCommands(std::vector<ParsedCommand> &commands) override {
		commands.clear();
		return false;
	}
	void sendReplies(const std::vector<CommandResult> &results, CommandInterface *originalInterface) override {
		received.insert(received.end(), results.begin(), results.end());
	}
	std::vector<CommandResult> received;
};

uint32_t bareId(BareCmd cmd) { return static_cast<uint32_t>(cmd); }

} // namespace

TEST_SUITE("CommandHandler") {

// ---------------------------------------------------------------------------
// Default implementations
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(FirmwareFixture, "the default command implementation answers not found without replies") {
	BareHandler bare;
	ParsedCommand cmd;
	cmd.cmdId = bareId(BareCmd::plain);
	cmd.type = CMDtype::get;
	cmd.target = &bare;
	std::vector<CommandReply> replies;
	CHECK(bare.command(cmd, replies) == CommandStatus::NOT_FOUND);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(CommandSystemFixture, "a registered command of a handler without own implementation is answered not found") {
	BareHandler bare;
	bare.registerCommand("plain", BareCmd::plain, "Plain command", CMDFLAG_GET);
	MockCommandInterface itf;
	itf.queueCommand(bare, bareId(BareCmd::plain), CMDtype::get);
	process();

	auto results = itf.allResults();
	REQUIRE(results.size() == 1);
	CHECK(results[0].type == CommandStatus::NOT_FOUND);
	CHECK(results[0].reply.empty());
	CHECK(results[0].commandHandler == &bare);
}

TEST_CASE_FIXTURE(FirmwareFixture, "the default description of a handler is empty") {
	BareHandler bare;
	CHECK(bare.getHelpstring().empty());
}

// ---------------------------------------------------------------------------
// Instance
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(FirmwareFixture, "the instance number passed to the constructor is reported") {
	BareHandler bare{2};
	CHECK(bare.getCommandHandlerInstance() == 2);
}

TEST_CASE_FIXTURE(FirmwareFixture, "setInstance changes the instance used for addressing") {
	BareHandler bare{0};
	bare.setInstance(3);
	CHECK(bare.getCommandHandlerInstance() == 3);
	CHECK(bare.getCommandHandlerInfo()->instance == 3);
	CHECK(CommandHandler::getHandlerFromId(BARE_CLSID, 3) == &bare);
	CHECK(CommandHandler::getHandlerFromId(BARE_CLSID, 0) == nullptr);
	CHECK(CommandHandler::getHandlerFromClassName("bare", 3) == &bare);
}

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(FirmwareFixture, "logsEnabled reports the log setting") {
	CHECK(CommandHandler::logsEnabled());
	CommandHandler::setLogsEnabled(false);
	CHECK_FALSE(CommandHandler::logsEnabled());
	CommandHandler::setLogsEnabled(true);
	CHECK(CommandHandler::logsEnabled());
}

TEST_CASE_FIXTURE(FirmwareFixture, "log messages are sent on the serial port in log format") {
	CommandHandler::logSerial("hello");
	CHECK(HostUsb::state().cdcTx == "[>!hello\n]");
}

TEST_CASE_FIXTURE(FirmwareFixture, "no log messages are sent while logs are disabled") {
	CommandHandler::setLogsEnabled(false);
	CommandHandler::logSerial("hello");
	CHECK(HostUsb::state().cdcTx.empty());
	CHECK(HostUsb::state().cdcWrites == 0);
}

TEST_CASE_FIXTURE(FirmwareFixture, "debug log messages are not sent outside debug mode") {
	CommandHandler::logSerialDebug("dbg");
	CHECK(HostUsb::state().cdcTx.empty());
}

TEST_CASE_FIXTURE(FirmwareFixture, "debug log messages are sent in debug mode") {
	SystemCommands::debugMode = true;
	CommandHandler::logSerialDebug("dbg");
	CHECK(HostUsb::state().cdcTx == "[>!dbg\n]");
}

TEST_CASE_FIXTURE(FirmwareFixture, "debug log messages respect the log setting") {
	SystemCommands::debugMode = true;
	CommandHandler::setLogsEnabled(false);
	CommandHandler::logSerialDebug("dbg");
	CHECK(HostUsb::state().cdcTx.empty());
}

// ---------------------------------------------------------------------------
// Help strings
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(FirmwareFixture, "help of a handler without commands and description") {
	BareHandler bare;
	CHECK(bare.getCommandsHelpstring() == "\nBare handler(bare.0):\nNo commands.");
	CHECK(bare.getCsvHelpstring() == "\nPrefix,Class ID, Class description\nbare.0,0x7E20,Bare handler\nNo commands.");
}

TEST_CASE_FIXTURE(FirmwareFixture, "help of a handler without display name is empty") {
	BareHandler bare{0, nullptr};
	bare.registerCommand("plain", BareCmd::plain, "Plain command", CMDFLAG_GET);
	CHECK(bare.getCommandsHelpstring().empty());
	CHECK(bare.getCsvHelpstring().empty());
}

TEST_CASE_FIXTURE(FirmwareFixture, "commands without help text are not listed in the help") {
	BareHandler bare;
	bare.registerCommand("plain", BareCmd::plain, "Plain command", CMDFLAG_GET);
	bare.registerCommand("nohelp", BareCmd::other, nullptr, CMDFLAG_GET);
	std::string help = bare.getCommandsHelpstring();
	CHECK(help.find("plain") != std::string::npos);
	CHECK(help.find("nohelp") == std::string::npos);
	std::string csv = bare.getCsvHelpstring();
	CHECK(csv.find("plain,0x1,Plain command, R\n") != std::string::npos);
	CHECK(csv.find("nohelp") == std::string::npos);
}

// ---------------------------------------------------------------------------
// Lookup
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(FirmwareFixture, "an unknown class name translates to class id 0") {
	BareHandler bare;
	CHECK(CommandHandler::getClassIdFromName("bare") == BARE_CLSID);
	CHECK(CommandHandler::getClassIdFromName("doesnotexist") == 0);
}

TEST_CASE_FIXTURE(FirmwareFixture, "deleting a handler through a base pointer removes it from the handler list") {
	size_t before = registeredCommandHandlers();
	CommandHandler *handler = new BareHandler();
	uint16_t handlerId = handler->getCommandHandlerID();
	CHECK(registeredCommandHandlers() == before + 1);
	CHECK(CommandHandler::isInHandlerList(handler));
	CHECK(CommandHandler::getHandlerFromHandlerId(handlerId) == handler);

	delete handler;
	CHECK(registeredCommandHandlers() == before);
	CHECK(CommandHandler::getHandlerFromHandlerId(handlerId) == nullptr);
	CHECK(CommandHandler::getHandlerFromId(BARE_CLSID) == nullptr);

	// The handler id is free again
	BareHandler next;
	CHECK(next.getCommandHandlerID() == handlerId);
}

// ---------------------------------------------------------------------------
// Command registration and flag overrides
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(FirmwareFixture, "registering a command id twice keeps the first definition") {
	BareHandler bare;
	bare.registerCommand("first", BareCmd::plain, "First", CMDFLAG_GET);
	bare.registerCommand("second", BareCmd::plain, "Second", CMDFLAG_SET);

	CmdHandlerCommanddef *def = bare.getCommandFromId(bareId(BareCmd::plain));
	REQUIRE(def != nullptr);
	REQUIRE(def->cmd != nullptr);
	CHECK(std::string(def->cmd) == "first");
	CHECK(def->flags == CMDFLAG_GET);
	CHECK(bare.getCommandFromName("second") == nullptr);
}

TEST_CASE_FIXTURE(FirmwareFixture, "overriding the flags of a registered command masks its access flags") {
	BareHandler bare;
	bare.registerCommand("rw", BareCmd::rw, "Read write", CMDFLAG_GET | CMDFLAG_SET);
	bare.overrideCommandFlags(BareCmd::rw, CMDFLAG_GET);

	CmdHandlerCommanddef *def = bare.getCommandFromId(bareId(BareCmd::rw));
	REQUIRE(def != nullptr);
	CHECK(def->flags == (CMDFLAG_GET | CMDFLAG_EXTOVERRIDE));
	CHECK(bare.isValidCommandId(bareId(BareCmd::rw), 0, CMDFLAG_GET));
	CHECK_FALSE(bare.isValidCommandId(bareId(BareCmd::rw), 0, CMDFLAG_SET));
	CHECK(bare.getCommandFromName("rw") == def); // Name and help are kept
}

TEST_CASE_FIXTURE(CommandSystemFixture, "a set command is rejected after its write flag was removed by an override") {
	MockCommandHandler mock;
	mock.overrideCommandFlags(MockCmd::value, CMDFLAG_GET);
	MockCommandInterface itf;
	itf.queueCommand(mock, MockCommandHandler::id(MockCmd::value), CMDtype::set, 5);
	itf.queueCommand(mock, MockCommandHandler::id(MockCmd::value), CMDtype::get);
	process();

	CHECK(mock.value == 0);
	REQUIRE(mock.received.size() == 1);
	CHECK(mock.received[0].type == CMDtype::get);
}

TEST_CASE_FIXTURE(FirmwareFixture, "overriding the flags with the default mask makes a command read only") {
	BareHandler bare;
	bare.registerCommand("rw", BareCmd::rw, "Read write", CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_GETADR | CMDFLAG_SETADR);
	bare.overrideCommandFlags_INT(bareId(BareCmd::rw)); // default mask CMDFLAG_GET | CMDFLAG_GETADR
	CmdHandlerCommanddef *def = bare.getCommandFromId(bareId(BareCmd::rw));
	REQUIRE(def != nullptr);
	CHECK(def->flags == (CMDFLAG_GET | CMDFLAG_GETADR | CMDFLAG_EXTOVERRIDE));
}

TEST_CASE_FIXTURE(FirmwareFixture, "overriding the flags of an unregistered command adds a placeholder without name") {
	BareHandler bare;
	bare.overrideCommandFlags(BareCmd::rw, CMDFLAG_GET);

	CmdHandlerCommanddef *def = bare.getCommandFromId(bareId(BareCmd::rw));
	REQUIRE(def != nullptr);
	CHECK(def->cmd == nullptr);
	CHECK(def->helpstring == nullptr);
	CHECK((def->flags & ~CMDFLAG_EXTOVERRIDE) == CMDFLAG_GET); // EXTOVERRIDE marker: see the KNOWN_ISSUE test below (bug 13)
	// Placeholders are not listed in the help
	CHECK(bare.getCommandsHelpstring() == "\nBare handler(bare.0):\ncmd\tflags\tdescription\n\n");
}

TEST_CASE_FIXTURE(FirmwareFixture, "a command registered after a flag override gets its name and the masked flags" *
		KNOWN_ISSUE("(bug 13) CommandHandler::overrideCommandFlags_INT creates the placeholder without CMDFLAG_EXTOVERRIDE, so registerCommand_INT treats it as already present and never registers the real command (CommandHandler.cpp:534, 560)")) {
	BareHandler bare;
	bare.overrideCommandFlags(BareCmd::rw, CMDFLAG_GET);
	bare.registerCommand("rw", BareCmd::rw, "Read write", CMDFLAG_GET | CMDFLAG_SET);

	CmdHandlerCommanddef *def = bare.getCommandFromId(bareId(BareCmd::rw));
	REQUIRE(def != nullptr);
	REQUIRE(def->cmd != nullptr); // first failing assertion: the placeholder is still there
	CHECK(std::string(def->cmd) == "rw");
	CHECK(std::string(def->helpstring) == "Read write");
	CHECK(bare.isValidCommandId(bareId(BareCmd::rw), 0, CMDFLAG_GET));
	// The mask itself is checked in "registering an overridden command again keeps the override mask" (line 536, bug 21)
}

TEST_CASE_FIXTURE(FirmwareFixture, "registering an overridden command again keeps the override mask" *
		KNOWN_ISSUE("(bug 21) registerCommand_INT should honor the flag mask of an override, but `flags &= flags | CMDFLAG_EXTOVERRIDE` (CommandHandler.cpp:536) does not change the flags. The new registration gets its own flags unmasked")) {
	// Independent of the placeholder defect: the command is registered before the override
	BareHandler bare;
	bare.registerCommand("rw", BareCmd::rw, "Read write", CMDFLAG_GET | CMDFLAG_SET);
	bare.overrideCommandFlags(BareCmd::rw, CMDFLAG_GET);
	bare.registerCommand("rw", BareCmd::rw, "Read write", CMDFLAG_GET | CMDFLAG_SET);

	CHECK_FALSE(bare.isValidCommandId(bareId(BareCmd::rw), 0, CMDFLAG_SET)); // first failing assertion: SET is allowed again
	CHECK(bare.isValidCommandId(bareId(BareCmd::rw), 0, CMDFLAG_GET));
	CmdHandlerCommanddef *def = bare.getCommandFromId(bareId(BareCmd::rw));
	REQUIRE(def != nullptr);
	CHECK((def->flags & (CMDFLAG_GET | CMDFLAG_SET)) == CMDFLAG_GET);
}

} // TEST_SUITE

// ---------------------------------------------------------------------------
TEST_SUITE("CommandInterface") {

TEST_CASE_FIXTURE(FirmwareFixture, "the default interface is always ready to send and has no commands") {
	PlainInterface itf;
	CHECK(itf.readyToSend());
	CHECK_FALSE(itf.hasNewCommands());
	itf.batchDone(); // default does nothing
	CHECK(itf.readyToSend());
}

TEST_CASE_FIXTURE(CommandSystemFixture, "results are delivered to an interface using the default ready check") {
	MockCommandHandler mock;
	mock.value = 12;
	PlainInterface plain;
	MockCommandInterface itf;
	itf.queueCommand(mock, MockCommandHandler::id(MockCmd::value), CMDtype::get);
	process();

	REQUIRE(plain.received.size() == 1);
	REQUIRE(plain.received[0].reply.size() == 1);
	CHECK(plain.received[0].reply[0].val == 12);
}

TEST_CASE_FIXTURE(FirmwareFixture, "deleting an interface through a base pointer unregisters it") {
	size_t before = CommandInterface::cmdInterfaces.size();
	CommandInterface *itf = new PlainInterface();
	CHECK(CommandInterface::cmdInterfaces.size() == before + 1);
	delete itf;
	CHECK(CommandInterface::cmdInterfaces.size() == before);
	CHECK(std::find(CommandInterface::cmdInterfaces.begin(), CommandInterface::cmdInterfaces.end(), itf) == CommandInterface::cmdInterfaces.end());
}

} // TEST_SUITE

// ---------------------------------------------------------------------------
TEST_SUITE("CDC_CommandInterface") {

TEST_CASE_FIXTURE(CommandSystemFixture, "replies larger than the send buffer are sent before the batch is done") {
	MockCommandHandler mock;
	CDC_CommandInterface cdc;
	CommandResult result;
	result.originalCommand.cmdId = MockCommandHandler::id(MockCmd::str);
	result.originalCommand.type = CMDtype::get;
	result.originalCommand.instance = 0;
	result.originalCommand.target = &mock;
	result.commandHandler = &mock;
	result.type = CommandStatus::OK;
	result.reply.emplace_back(std::string(CDC_CMD_BUFFER_SIZE, 'z'));

	cdc.sendReplies({result}, &cdc); // no batchDone()
	process();
	CHECK(HostUsb::state().cdcTx.rfind("[mock.0.str?|zzz", 0) == 0);
	CHECK(HostUsb::state().cdcTx.size() == std::string("[mock.0.str?|]\n").size() + CDC_CMD_BUFFER_SIZE);
}

TEST_CASE_FIXTURE(CommandSystemFixture, "a full reply buffer blocks further replies until the parser timeout expired") {
	MockCommandHandler mock;
	CDC_CommandInterface cdc;
	CommandResult result;
	result.originalCommand.cmdId = MockCommandHandler::id(MockCmd::str);
	result.originalCommand.type = CMDtype::get;
	result.originalCommand.target = &mock;
	result.commandHandler = &mock;
	result.type = CommandStatus::OK;
	result.reply.emplace_back(std::string(CDC_CMD_BUFFER_SIZE, 'z'));

	CHECK(cdc.readyToSend());
	cdc.sendReplies({result}, &cdc); // buffer full, the cdc thread did not run yet
	CHECK_FALSE(cdc.readyToSend());
	HostPlatform::advanceMs(500);
	CHECK_FALSE(cdc.readyToSend());
	HostPlatform::advanceMs(1); // more than 500 ms after the last reply: readyToSend's own timeout check (CommandInterface.cpp:311) reports ready
	CHECK(cdc.readyToSend());
}

TEST_CASE_FIXTURE(CommandSystemFixture, "small replies are buffered until the batch is done") {
	MockCommandHandler mock;
	CDC_CommandInterface cdc;
	CommandResult result;
	result.originalCommand.cmdId = MockCommandHandler::id(MockCmd::value);
	result.originalCommand.type = CMDtype::get;
	result.originalCommand.instance = 0;
	result.originalCommand.target = &mock;
	result.commandHandler = &mock;
	result.type = CommandStatus::OK;
	result.reply.emplace_back((int64_t)7);

	cdc.sendReplies({result}, &cdc);
	process();
	CHECK(HostUsb::state().cdcTx.empty());
	cdc.batchDone();
	process();
	CHECK(HostUsb::state().cdcTx == "[mock.0.value?|7]\n");
}

TEST_CASE_FIXTURE(CommandSystemFixture, "deleting a cdc interface through a base pointer unregisters it") {
	size_t before = CommandInterface::cmdInterfaces.size();
	CommandInterface *itf = new CDC_CommandInterface();
	CHECK(CommandInterface::cmdInterfaces.size() == before + 1);
	delete itf;
	CHECK(CommandInterface::cmdInterfaces.size() == before);
}

} // TEST_SUITE

// ---------------------------------------------------------------------------
TEST_SUITE("UART_CommandInterface") {

TEST_CASE_FIXTURE(CommandSystemFixture, "replies are not transmitted while the uart port is busy") {
	MockCommandHandler mock;
	UART_CommandInterface uart{115200};
	mock.value = 4;
	HostRtos::setDelayBudgetMs(10);

	HostPlatform::uartReceive("mock.value?;");
	HostRtos::runUntilBlocked(commandThread); // executes the command and queues the reply on the uart interface

	external_uart.takeSemaphore(true); // port taken by another device
	HostRtos::runUntilBlocked(uart);	   // waits for the port
	CHECK(HostPlatform::uart().tx.empty());
	CHECK(HostPlatform::uart().txTransfers == 0);

	external_uart.giveSemaphore(true);
	// Extra Notify() only because the fake kernel restarts Run() after the delay budget is used up;
	// on target the thread keeps polling the port and sends without a new notification
	uart.Notify();
	HostRtos::runUntilBlocked(uart);
	CHECK(HostPlatform::uart().tx == "[mock.value?|4]\n");
	HostPlatform::uartCompleteTx();
}

TEST_CASE_FIXTURE(FirmwareFixture, "deleting a uart interface through a base pointer unregisters it and releases the port") {
	size_t before = CommandInterface::cmdInterfaces.size();
	CommandInterface *itf = new UART_CommandInterface(115200);
	CHECK(CommandInterface::cmdInterfaces.size() == before + 1);
	CHECK(external_uart.isReserved());
	delete itf;
	CHECK(CommandInterface::cmdInterfaces.size() == before);
	CHECK_FALSE(external_uart.isReserved());
}

} // TEST_SUITE
