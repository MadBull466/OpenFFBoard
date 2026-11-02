/*
 * HID command interface (HidCommandInterface.cpp), additional cases (P1)
 *
 *  - Reply branches of sendReplies for results without reply data (get-at ACK, ERR, info, NO_REPLY)
 *  - Reply types ERR and NONE in queueReplyValues
 *  - Limit for mirrored results of other interfaces
 *  - Broadcast (instance 0xFF) to handlers that do not know the command
 *  - Run() waiting for the HID endpoint, transferComplete, destruction through a base pointer
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "FirmwareFixture.h"
#include "MockCommandHandler.h"
#include "MockCommandInterface.h"

#include "HidCommandInterface.h"
#include <cstring>
#include <memory>

namespace {

/** Unpacked copy of a command report. Fields of the packed report struct can not be used in checks directly */
struct HidGapReply {
	uint8_t reportId;
	HidCmdType type;
	uint16_t clsid;
	uint8_t instance;
	uint32_t cmd;
	uint64_t data;
	uint64_t addr;
};

constexpr uint16_t HIDGAP_CLSID = 0x7E10;

enum class GapCmd : uint32_t {
	ackat = 0x40,	 //!< get-at/set-at. Returns OK without reply data
	errreply = 0x41, //!< get. Returns OK with a reply of type ERR
	nonereply = 0x42,//!< get. Returns OK with a reply of type NONE
	silent = 0x43,	 //!< info. Returns OK without reply data
};

/** Command handler with commands that produce reply combinations the MockCommandHandler does not create */
class HidGapHandler : public CommandHandler {
public:
	explicit HidGapHandler(uint16_t clsid = HIDGAP_CLSID, uint8_t instance = 0) : CommandHandler("hidgap", clsid, instance), clsid(clsid) {
		CommandHandler::registerCommands();
		registerCommand("ackat", GapCmd::ackat, "OK without reply data", CMDFLAG_GETADR | CMDFLAG_SETADR);
		registerCommand("errreply", GapCmd::errreply, "Reply of type ERR", CMDFLAG_GET);
		registerCommand("nonereply", GapCmd::nonereply, "Reply of type NONE", CMDFLAG_GET);
		registerCommand("silent", GapCmd::silent, "Info without reply data", CMDFLAG_INFOSTRING);
	}

	const ClassIdentifier getInfo() override { return ClassIdentifier{"HID gap handler", clsid, ClassVisibility::visible}; }

	CommandStatus command(const ParsedCommand &cmd, std::vector<CommandReply> &replies) override {
		received.push_back(cmd);
		switch (static_cast<GapCmd>(cmd.cmdId)) {
		case GapCmd::ackat:
		case GapCmd::silent:
			return CommandStatus::OK;
		case GapCmd::errreply:
			replies.emplace_back(CommandReplyType::ERR);
			return CommandStatus::OK;
		case GapCmd::nonereply:
			replies.emplace_back(CommandReplyType::NONE);
			return CommandStatus::OK;
		default:
			return CommandStatus::NOT_FOUND;
		}
	}

	static uint32_t id(GapCmd cmd) { return static_cast<uint32_t>(cmd); }
	std::vector<ParsedCommand> received;

private:
	uint16_t clsid;
};

std::vector<HidGapReply> decodeHidReports() {
	std::vector<HidGapReply> out;
	for (const HostUsb::HidReport &raw : HostUsb::state().hidReports) {
		REQUIRE(raw.data.size() == sizeof(HID_CMD_Data_t));
		HID_CMD_Data_t report;
		std::memcpy(&report, raw.data.data(), sizeof(HID_CMD_Data_t));
		out.push_back({report.reportId, report.type, report.clsid, report.instance, report.cmd, report.data, report.addr});
	}
	return out;
}

struct HidGapFixture : CommandSystemFixture {
	MockCommandHandler mock{"mock", CLSID_CUSTOM, 0};
	HidGapHandler gap;
	HID_CommandInterface hid;

	/** Sends a command report like the usb callback does and returns all reply reports */
	std::vector<HidGapReply> sendRaw(HidCmdType type, uint32_t cmd, uint16_t clsid, uint64_t data = 0, uint64_t addr = 0, uint8_t instance = 0) {
		HostUsb::state().hidReports.clear();
		HID_CMD_Data_t report;
		report.type = type;
		report.clsid = clsid;
		report.instance = instance;
		report.cmd = cmd;
		report.data = data;
		report.addr = addr;
		hid.hidCmdCallback(&report);
		process();
		return decodeHidReports();
	}

	std::vector<HidGapReply> sendGap(HidCmdType type, GapCmd cmd, uint64_t data = 0, uint64_t addr = 0) {
		return sendRaw(type, HidGapHandler::id(cmd), HIDGAP_CLSID, data, addr);
	}

	/** A result like the command thread creates it */
	CommandResult makeResult(CommandHandler &handler, uint32_t cmdId, CMDtype type, CommandStatus status, std::vector<CommandReply> replies = {}, int64_t val = 0, int64_t adr = 0) {
		CommandResult result;
		result.originalCommand.cmdId = cmdId;
		result.originalCommand.type = type;
		result.originalCommand.val = val;
		result.originalCommand.adr = adr;
		result.originalCommand.target = &handler;
		result.originalCommand.instance = handler.getCommandHandlerInfo()->instance;
		result.commandHandler = &handler;
		result.handlerId = handler.getCommandHandlerID();
		result.type = status;
		result.reply = std::move(replies);
		return result;
	}
};

} // namespace

TEST_SUITE("HID_CommandInterface") {

// ---------------------------------------------------------------------------
// Results without reply data (sendReplies)
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(HidGapFixture, "request with address without reply data is acknowledged with the address") {
	auto reply = sendGap(HidCmdType::requestAddr, GapCmd::ackat, 0, 7);

	REQUIRE(gap.received.size() == 1);
	CHECK(gap.received[0].type == CMDtype::getat);
	REQUIRE(reply.size() == 1);
	CHECK(reply[0].type == HidCmdType::ACK);
	CHECK(reply[0].clsid == HIDGAP_CLSID);
	CHECK(reply[0].cmd == HidGapHandler::id(GapCmd::ackat));
	CHECK(reply[0].addr == 7);
}

TEST_CASE_FIXTURE(HidGapFixture, "write with address without reply data is echoed with value and address") {
	auto reply = sendGap(HidCmdType::writeAddr, GapCmd::ackat, 3, 8);
	REQUIRE(reply.size() == 1);
	CHECK(reply[0].type == HidCmdType::requestAddr);
	CHECK(reply[0].data == 3);
	CHECK(reply[0].addr == 8);
}

TEST_CASE_FIXTURE(HidGapFixture, "a failed info request without reply data is reported with the err type") {
	// The untyped mock command accepts info requests but handleGetSet returns ERR for them
	auto reply = sendRaw(HidCmdType::info, MockCommandHandler::id(MockCmd::untyped), CLSID_CUSTOM);
	REQUIRE(mock.received.size() == 1);
	CHECK(mock.received[0].type == CMDtype::info);
	REQUIRE(reply.size() == 1);
	CHECK(reply[0].type == HidCmdType::err);
	CHECK(reply[0].cmd == MockCommandHandler::id(MockCmd::untyped));
	CHECK(reply[0].clsid == CLSID_CUSTOM);
}

TEST_CASE_FIXTURE(HidGapFixture, "a successful info request without reply data sends nothing") {
	auto reply = sendGap(HidCmdType::info, GapCmd::silent);
	CHECK(gap.received.size() == 1);
	CHECK(reply.empty());
	CHECK_FALSE(hid.waitingToSend());
}

TEST_CASE_FIXTURE(HidGapFixture, "results with NO_REPLY status are ignored even if they contain replies") {
	std::vector<CommandResult> results = {makeResult(mock, MockCommandHandler::id(MockCmd::value), CMDtype::get, CommandStatus::NO_REPLY, {CommandReply((int64_t)5)})};
	hid.sendReplies(results, &hid);
	CHECK_FALSE(hid.waitingToSend());
	process();
	CHECK(decodeHidReports().empty());
}

TEST_CASE_FIXTURE(HidGapFixture, "results of other interfaces without reply data are dropped while more than 10 reports are pending") {
	MockCommandInterface other;
	for (int i = 0; i < 20; i++) {
		std::vector<CommandResult> results = {makeResult(mock, MockCommandHandler::id(MockCmd::value), CMDtype::set, CommandStatus::OK, {}, i)};
		hid.sendReplies(results, &other);
	}
	process();
	auto reply = decodeHidReports();
	REQUIRE(reply.size() == 11);
	for (size_t i = 0; i < reply.size(); i++) {
		CHECK(reply[i].type == HidCmdType::request);
		CHECK(reply[i].data == i);
	}
}

TEST_CASE_FIXTURE(HidGapFixture, "real replies are still queued while more than 10 reports are pending") {
	MockCommandInterface other;
	for (int i = 0; i < 15; i++) {
		std::vector<CommandResult> results = {makeResult(mock, MockCommandHandler::id(MockCmd::value), CMDtype::get, CommandStatus::OK, {CommandReply((int64_t)i)})};
		hid.sendReplies(results, &other);
	}
	process();
	CHECK(decodeHidReports().size() == 15);
}

// ---------------------------------------------------------------------------
// Reply types (queueReplyValues)
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(HidGapFixture, "error replies of a handler are sent with the err type") {
	auto reply = sendGap(HidCmdType::request, GapCmd::errreply);
	REQUIRE(reply.size() == 1);
	CHECK(reply[0].type == HidCmdType::err);
	CHECK(reply[0].cmd == HidGapHandler::id(GapCmd::errreply));
}

TEST_CASE_FIXTURE(HidGapFixture, "replies of type NONE are not sent") {
	auto reply = sendGap(HidCmdType::request, GapCmd::nonereply);
	CHECK(gap.received.size() == 1);
	CHECK(reply.empty());
}

// ---------------------------------------------------------------------------
// Broadcast to all instances (hidCmdCallback with instance 0xFF)
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(HidGapFixture, "broadcast of a command unknown to the class is answered with not found") {
	auto reply = sendRaw(HidCmdType::request, MockCommandHandler::id(MockCmd::unknown), CLSID_CUSTOM, 1, 2, 0xFF);
	REQUIRE(reply.size() == 1);
	CHECK(reply[0].type == HidCmdType::notFound);
	CHECK(reply[0].cmd == MockCommandHandler::id(MockCmd::unknown));
	CHECK(reply[0].instance == 0xFF);
	CHECK(reply[0].data == 1); // Request is echoed
	CHECK(reply[0].addr == 2);
	CHECK(mock.received.empty());
}

TEST_CASE_FIXTURE(HidGapFixture, "characterization: a broadcast rejected by one handler leaves the command of an earlier handler pending") {
	// NEEDS REVIEW 7: hidCmdCallback returns on the first handler that does not know the command, after the commands
	// for earlier handlers of the class were already queued, without waking the command thread. They are executed
	// together with the next valid report. Should a rejected broadcast execute nothing (or all valid parts at once)?
	HidGapHandler sameClass{CLSID_CUSTOM, 1}; // registered after mock, does not know MockCmd::value
	mock.value = 9;

	auto reply = sendRaw(HidCmdType::request, MockCommandHandler::id(MockCmd::value), CLSID_CUSTOM, 0, 0, 0xFF);
	REQUIRE(reply.size() == 1);
	CHECK(reply[0].type == HidCmdType::notFound);
	CHECK(mock.received.empty()); // Not executed yet

	reply = sendRaw(HidCmdType::request, MockCommandHandler::id(MockCmd::readonly), CLSID_CUSTOM, 0, 0, 0);
	CHECK(mock.received.size() == 2); // The pending broadcast part is executed with the next command
	CHECK(reply.size() == 2);
}

// ---------------------------------------------------------------------------
// Sending (Run), transferComplete, lifetime
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(HidGapFixture, "replies wait while the HID endpoint is busy") {
	HostRtos::setDelayBudgetMs(10);
	HostUsb::state().hidReady = false;
	mock.value = 6;
	auto reply = sendRaw(HidCmdType::request, MockCommandHandler::id(MockCmd::value), CLSID_CUSTOM);
	CHECK(reply.empty());
	CHECK(hid.waitingToSend());
	CHECK(HostPlatform::nowUs() >= 10000); // polled the endpoint with 1 ms delays

	HostUsb::state().hidReady = true;
	// Extra Notify() only because the fake kernel restarts Run() after the delay budget is used up;
	// on target the thread keeps polling tud_hid_n_ready() and sends without a new notification
	hid.Notify();
	process();
	reply = decodeHidReports();
	REQUIRE(reply.size() == 1);
	CHECK(reply[0].data == 6);
	CHECK_FALSE(hid.waitingToSend());
}

TEST_CASE_FIXTURE(HidGapFixture, "characterization: the transfer complete callback does not change pending replies") {
	// NEEDS REVIEW: transferComplete is empty (HidCommandInterface.cpp:162). Should it trigger sending pending replies? (PROGRESS NEEDS REVIEW #8)
	HostUsb::state().hidReady = false;
	std::vector<CommandResult> results = {makeResult(mock, MockCommandHandler::id(MockCmd::value), CMDtype::get, CommandStatus::OK, {CommandReply((int64_t)1)})};
	hid.sendReplies(results, &hid);
	REQUIRE(hid.waitingToSend());

	uint8_t report[sizeof(HID_CMD_Data_t)] = {HID_ID_HIDCMD};
	hid.transferComplete(0, report, sizeof(report));
	CHECK(hid.waitingToSend());
	CHECK(HostUsb::state().hidReports.empty());
}

TEST_CASE_FIXTURE(CommandSystemFixture, "deleting the interface through a base pointer unregisters it") {
	size_t before = CommandInterface::cmdInterfaces.size();
	CommandInterface *itf = new HID_CommandInterface();
	CHECK(HID_CommandInterface::globalInterface == static_cast<HID_CommandInterface *>(itf));
	CHECK(CommandInterface::cmdInterfaces.size() == before + 1);

	delete itf;
	CHECK(HID_CommandInterface::globalInterface == nullptr);
	CHECK(CommandInterface::cmdInterfaces.size() == before);
}

} // TEST_SUITE
