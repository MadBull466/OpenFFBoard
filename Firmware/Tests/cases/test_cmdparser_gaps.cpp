/*
 * String command parser (CmdParser.cpp), additional cases (P1)
 *
 *  - Capacity boundary of CmdParser::add (a chunk is only stored if it is smaller than the free space)
 *  - Malformed address values after '?' in set-with-address commands (validity check of the '?' part)
 *
 * Note: malformed values after '?' or '=' in commands WITHOUT a valid second part make parse() loop
 * forever (known issue, see test_cmdparser.cpp "non numeric values are rejected without blocking the parser").
 * All inputs here contain a valid "=<int>" part so the parser takes the plain set path and returns.
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "FirmwareFixture.h"
#include "MockCommandHandler.h"

#include "CmdParser.h"

namespace {

/** Adds text to the parser. The parser modifies the buffer so a copy is passed */
bool feedGap(CmdParser &parser, std::string text) {
	uint32_t len = text.size();
	return parser.add(text.data(), &len);
}

struct ParserGapFixture : FirmwareFixture {
	MockCommandHandler mock{"mock", CLSID_CUSTOM, 0};
	CmdParser parser{512};
	std::vector<ParsedCommand> commands;

	/** Free space of a new parser. The ring buffer of 512 bytes can hold 511 bytes */
	static constexpr uint32_t freeSpace = 511;

	/** A terminated set command padded with leading terminators to exactly `length` bytes */
	static std::string paddedCommand(size_t length) {
		const std::string cmd = "mock.value=1;";
		REQUIRE(length >= cmd.size());
		return std::string(length - cmd.size(), ';') + cmd;
	}

	bool parse(const std::string &text) {
		feedGap(parser, text);
		return parser.parse(commands);
	}
};

} // namespace

TEST_SUITE("CmdParser") {

// ---------------------------------------------------------------------------
// Capacity boundary (CmdParser.cpp:53, freeSpace() > *Len)
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(ParserGapFixture, "free space of a new parser is one byte less than the buffer size") {
	// bufferCapacity() returns the ring buffer free space while the tick is 0 (see the known issue in test_cmdparser.cpp)
	CHECK(parser.bufferCapacity() == (int32_t)freeSpace);
}

TEST_CASE_FIXTURE(ParserGapFixture, "a chunk one byte smaller than the free space is accepted") {
	REQUIRE(parser.bufferCapacity() == (int32_t)freeSpace);
	CHECK(feedGap(parser, paddedCommand(freeSpace - 1)));
	CHECK(parser.parse(commands));
	REQUIRE(commands.size() == 1);
	CHECK(commands[0].type == CMDtype::set);
	CHECK(commands[0].val == 1);
}

TEST_CASE_FIXTURE(ParserGapFixture, "characterization: a terminated chunk exactly as large as the free space is discarded") {
	// NEEDS REVIEW: CmdParser::add stores data only if freeSpace() > len. freeSpace() already excludes the
	// slot the ring buffer keeps empty, so a chunk that would exactly fill the buffer is dropped although it fits.
	// Is the strict comparison intended (one byte of the buffer is never usable)?
	REQUIRE(parser.bufferCapacity() == (int32_t)freeSpace);
	CHECK_FALSE(feedGap(parser, paddedCommand(freeSpace))); // terminator flag is reset when the chunk is dropped
	CHECK_FALSE(parser.parse(commands));
	CHECK(commands.empty());
	CHECK(parser.bufferCapacity() == (int32_t)freeSpace); // Nothing stored
}

TEST_CASE_FIXTURE(ParserGapFixture, "a terminated chunk that does not fit discards the buffered data") {
	CHECK_FALSE(feedGap(parser, "mock.val"));
	CHECK_FALSE(feedGap(parser, paddedCommand(freeSpace - 8)));
	CHECK(parser.bufferCapacity() == (int32_t)freeSpace); // Buffer cleared
	CHECK_FALSE(parser.parse(commands));
	CHECK(commands.empty());
}

TEST_CASE_FIXTURE(ParserGapFixture, "an unterminated chunk that does not fit keeps the buffered data") {
	CHECK_FALSE(feedGap(parser, "mock.val"));
	CHECK_FALSE(feedGap(parser, std::string(freeSpace, 'x'))); // does not fit, no terminator: buffer is not cleared
	CHECK(parser.bufferCapacity() == (int32_t)freeSpace - 8);
	CHECK(feedGap(parser, "ue=3;"));
	CHECK(parser.parse(commands));
	REQUIRE(commands.size() == 1);
	CHECK(commands[0].val == 3);
}

// ---------------------------------------------------------------------------
// Malformed '?' values in complex commands (CmdParser.cpp:156, validPqm)
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(ParserGapFixture, "characterization: a set with a malformed address is parsed as a plain set") {
	// NEEDS REVIEW: "cmd=<int>?<garbage>" is not rejected. The '?' part is invalid, so the command becomes a plain
	// set of the value before '?' (strtoll stops at '?'). Should a malformed address reject the whole command?
	const char *inputs[] = {
		"mock.value=5?-a;", // sign without digit
		"mock.value=5?+;",  // sign at the end
		"mock.value=5?a5;", // digit only at the second position, no sign
		"mock.value=5?xg;", // hex prefix with a non hex digit
		"mock.value=5?y1;", // hex digit at the second position without x prefix
	};
	for (const char *text : inputs) {
		CAPTURE(std::string(text));
		commands.clear();
		CHECK(parse(text));
		REQUIRE(commands.size() == 1);
		CHECK(commands[0].type == CMDtype::set);
		CHECK(commands[0].val == 5);
		CHECK(commands[0].adr == 0);
		CHECK(commands[0].cmdId == MockCommandHandler::id(MockCmd::value));
	}
}

TEST_CASE_FIXTURE(ParserGapFixture, "valid address forms after a set value are parsed as set with address") {
	struct Case {
		const char *text;
		int64_t adr;
	};
	const Case cases[] = {
		{"mock.addr=5?-3;", -3},
		{"mock.addr=5?+3;", 3},
		{"mock.addr=5?xa;", 0xA},
		{"mock.addr=5?9;", 9},
	};
	for (const Case &c : cases) {
		CAPTURE(std::string(c.text));
		commands.clear();
		CHECK(parse(c.text));
		REQUIRE(commands.size() == 1);
		CHECK(commands[0].type == CMDtype::setat);
		CHECK(commands[0].val == 5);
		CHECK(commands[0].adr == c.adr);
	}
}

TEST_CASE_FIXTURE(ParserGapFixture, "a get with a signed address is parsed as get with address") {
	CHECK(parse("mock.addr?-4;"));
	REQUIRE(commands.size() == 1);
	CHECK(commands[0].type == CMDtype::getat);
	CHECK(commands[0].adr == -4);
}

} // TEST_SUITE
