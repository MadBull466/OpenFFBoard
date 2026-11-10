/*
 * TMCDebugBridge, phase P13 T1 skeleton: the bridge builds into ffb_driver_tests, is constructed on the fake TMC4671 and
 * its CDC replies are captured. Requirements R3.3 / R3.4 (identity, chooser) of
 * Tests/claude/charters/driver/TMCDEBUG_REQUIREMENTS.md; the full test groups follow in T2.
 *
 * Oracles: FW (the firmware's own identity data, plain because it is the interface), the TMCL reference v2.22
 * section 2.1.1 for the checksum helper of the fixture (hand sums in the comments).
 */
#include "doctest.h"
#include "TmcDebugFixture.h"

#include "constants.h"

TEST_SUITE("TMCDebugBridge setup") {

TEST_CASE("the fixture frame helper builds the 9 byte TMCL request with the 8 bit sum checksum") {
	// {1, 148, 0x12, 0, 0, 0, 0, 0}: 1 + 148 + 18 = 167 = 0xA7 (TMCL reference v2.22 section 2.1.1, 8-bit sum)
	auto f = TmcDebugFixture::frame(148, 0x12, 0);
	REQUIRE(f.size() == 9);
	CHECK(f[0] == 1);
	CHECK(f[1] == 148);
	CHECK(f[2] == 0x12);
	CHECK(f[8] == 0xA7);
	// value MSB first: 0x01020304 -> bytes 4..7 = 1,2,3,4; sum of the first 8 = 1+148+18+0+1+2+3+4 = 177 = 0xB1
	auto g = TmcDebugFixture::frame(148, 0x12, 0x01020304);
	CHECK(g[4] == 1);
	CHECK(g[5] == 2);
	CHECK(g[6] == 3);
	CHECK(g[7] == 4);
	CHECK(g[8] == 0xB1);
	// the sum wraps at 256: {1, 0xFF, 0xFF, 0, 0,0,0,0} = 1 + 255 + 255 = 511 = 0x1FF -> 0xFF
	CHECK(TmcDebugFixture::frame(0xFF, 0xFF, 0)[8] == 0xFF);
	// an explicit wrong checksum is kept
	CHECK(TmcDebugFixture::frame(148, 0x12, 0, 0x55)[8] == 0x55);
}

TEST_CASE_FIXTURE(TmcDebugFixture, "the bridge is constructed on the fake TMC4671 and identifies itself as the TMC Debug Bridge") {
	auto bridge = makeBridge();
	REQUIRE(bridge != nullptr);
	CHECK(std::string(TMCDebugBridge::info.name) == "TMC Debug Bridge");
	CHECK(bridge->getInfo().id == CLSID_MAIN_TMCDBG);
	CHECK(bridge->getInfo().visibility == ClassVisibility::debug);
	CHECK(TMCDebugBridge::isCreatable());
}

TEST_CASE_FIXTURE(TmcDebugFixture, "a frame that is not a TMCL request does not produce a CDC reply from the bridge") {
	auto bridge = makeBridge();
	clearSent();
	// length 3, not 9: handled by FFBoardMain::cdcRcv, never a TMCL reply (charter R2.1, FW)
	send(*bridge, {'x', 'y', '\n'});
	CHECK(cdcWrites() == 0);
	CHECK(cdcFlushes() == 0);
	CHECK(sent().empty());
}

}
