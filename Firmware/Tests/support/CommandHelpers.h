/*
 * CommandHelpers.h
 *
 * Helpers to execute firmware commands in tests (ffb_target_tests, P7):
 *  - CommandHelpers::run / get: a command of a handler by its name, like the command thread does
 *    (CommandHandler::command with a ParsedCommand), the way the module tests of P2-P6 do it with their local cmd().
 *  - CommandHelpers::CdcConsole: whole command lines through the CDC command interface of a main class and the command
 *    thread, like the configurator sends them ("fxm.0.new=1\n"); returns what the firmware answered on CDC. Parsing,
 *    the access flags of the command table, the handler lookup by class name and instance, and the reply format are
 *    real code (CmdParser, CommandInterface, FFBoardMainCommandThread).
 */
#ifndef HOSTTEST_COMMANDHELPERS_H_
#define HOSTTEST_COMMANDHELPERS_H_

#include "HostRtos.h"
#include "HostUsb.h"

#include "CDCcomm.h"
#include "CommandHandler.h"
#include "FFBoardMain.h"

#include "doctest.h"

#include <cstdint>
#include <string>
#include <vector>

namespace CommandHelpers {

/** Executes a command of `handler` by its name. Replies are appended to `replies` if given. Fails the test if the name is not registered */
inline CommandStatus run(CommandHandler &handler, const std::string &name, CMDtype type, int64_t val = 0,
						 std::vector<CommandReply> *replies = nullptr, int64_t adr = 0) {
	CmdHandlerCommanddef *def = handler.getCommandFromName(name);
	REQUIRE_MESSAGE(def != nullptr, "command " << name << " not registered");
	ParsedCommand parsed;
	parsed.cmdId = def->cmdId;
	parsed.type = type;
	parsed.val = val;
	parsed.adr = adr;
	parsed.target = &handler;
	std::vector<CommandReply> local;
	return handler.command(parsed, replies ? *replies : local);
}

/** Reads a value with a get command. Fails the test if the command does not return OK with exactly one reply */
inline int64_t get(CommandHandler &handler, const std::string &name, int64_t adr = 0) {
	std::vector<CommandReply> replies;
	CHECK(run(handler, name, adr != 0 ? CMDtype::getat : CMDtype::get, 0, &replies, adr) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	return replies.front().val;
}

/**
 * Sends command lines to the CDC command interface of a main class (FFBoardMain::cdcRcv, as tud_cdc_rx_cb ->
 * cdcRcvReady does on the target) and runs the firmware threads until they are idle, completing CDC transfers like the
 * USB stack (CDCcomm::cdcFinished). Any main class works; the failsafe FFBoardMain is the smallest.
 */
class CdcConsole {
public:
	explicit CdcConsole(FFBoardMain &main) : main(main) {}

	/** Sends `line` (include the terminating "\n" or ";") and returns everything the firmware wrote to CDC meanwhile */
	std::string send(const std::string &line) {
		HostUsb::state().cdcTx.clear();
		std::string data = line;
		uint32_t len = static_cast<uint32_t>(data.size());
		main.cdcRcv(data.data(), &len);
		uint32_t writesSeen = HostUsb::state().cdcWrites;
		for (int i = 0; i < 1000; i++) {
			uint32_t activations = HostRtos::runUntilIdle();
			bool sent = HostUsb::state().cdcWrites != writesSeen;
			writesSeen = HostUsb::state().cdcWrites;
			if (sent) {
				CDCcomm::cdcFinished(0);
			}
			if (activations == 0 && !sent) {
				break;
			}
		}
		return HostUsb::state().cdcTx;
	}

private:
	FFBoardMain &main;
};

} // namespace CommandHelpers

#endif
