/*
 * TMCDebugBridge, work order TDF (phase P13 T2): the binary TMCL frames of TMCDebugBridge::cdcRcv and their robustness.
 * Requirements R2.1-R2.12 and R4.1-R4.3 of Tests/claude/charters/driver/TMCDEBUG_REQUIREMENTS.md.
 *
 * Oracles (cited "TMCL <section>" = TMCL Reference and Programming Manual v2.22, Tests/shared/specs/TMCL_reference.pdf):
 *  - TMCL 2.1 request format {module address, command, type, motor/bank, value MSB first (4), checksum}, 2.1.1 checksum
 *    = 8-bit sum of the other bytes, 2.2 reply format {reply address, module address, status, command, value MSB first
 *    (4), checksum}, 2.2.1 status codes (100 ok; 1 wrong checksum, 2 invalid command, 3 wrong type, 4 invalid value,
 *    5 configuration EEPROM locked, 6 command not available), 3.10 GGP (command 10, worked example request
 *    01 0A 42 00 00 00 00 00 4D), 3.15 GIO (command 15, example request 01 0F 03 01 00 00 00 00 14), 3.35 command 136
 *    (type 0: reply {host address, 8 character string} without checksum; type 1: normal reply format).
 *  - DS: the SPI traffic at the fake chip (FakeTmc4671, TMC4671-LA datasheet Rev 2.07 section 4.2.1: 40 bit datagram,
 *    bit 39 = write, address, data MSB first) through the approved D1 driver.
 *  - HD: hand derivations in the comments (checksum sums, voltage in 0.1 V units).
 *  - FW: not in any supplied document, test name starts with "characterization:" (NEEDS REVIEW): the opcodes 148, 146,
 *    143, the echo of request bytes 3..6 for 146, the GGP type values, the GIO type 5 voltage, the replies of 143, the
 *    version string and the binary version bytes, the truncated reply on a short CDC buffer, the missing reassembly.
 *  - KI (owner decision L5): KNOWN_ISSUE tests of the intended reply (status 1, 2, 3, not 0): B-DTDF-1 .. B-DTDF-4.
 *
 * The commands 148 and 146 are used as vehicles in the plain tests of R2.1 / R2.2 / R2.3 / R2.4 (register read / write
 * frames); their numbers are FW, not in the supplied documents (see the characterization cases of R2.5 / R2.6).
 *
 * Routing evidence: a frame that the bridge does not handle goes to FFBoardMain::cdcRcv, which appends the bytes to the
 * CDC text parser (cdcCmdInterface). The free capacity of that parser (bufferCapacity()) shrinks by the bytes it took,
 * so "passed on" / "not passed on" is read from it, no thread is needed.
 *
 * Status bytes of the replies of commands 10, 15 and 143 (64, 64, 40 decimal in the code): TMCL 2.2.1 (also TOSV / LEV
 * manuals) lists only 100 (successfully executed) and 1..6 (101 is for EEPROM programs). KNOWN_ISSUE B-DTDF-5 (GGP / GIO,
 * intended 100) and B-DTDF-6 (143 types 3 / 4, membership in {100, 1..6} only, opcode 143 is not in the documents). The
 * values 64 / 40 look like a hex versus decimal slip of 0x64 (TDB:173, 180, 189): an observation, not a fact. The plain
 * layout tests of those replies do not look at the status byte.
 *
 * NEEDS REVIEW (binary version): TMCL 3.35 gives the value field of command 136 type 1 as "byte index 1 version low,
 * 2 version high, 3 type low, 4 type high" (numbering ambiguous: 1-based while the field is MSB first), the firmware sends
 * module high, module low, firmware high, firmware low = 0, 15, 3, 7. Kept as characterization.
 *
 * Host vs target: the replies are built from bytes and the checksum is an 8-bit sum, so there is no dependence on
 * int width. The register value goes through __REV on both sides (the host stub is a byte swap), the request bytes are
 * copied with memcpy and swapped to a uint32_t, so the result is the big endian value on little endian hosts and on the
 * ARM target alike. cmd 146 reads Buf+4 with memcpy (no alignment problem). getIntV() / 100 is an int32 division of a
 * non-negative value (no signed shift).
 *
 * Not tested: named driver states (the driver state is not observable through the public interface; no firmware edit,
 * no friend), the text commands (TDC), the constructor / destructor / chooser (TDL), the driver internals (D1), the real
 * USB stack (ffb_usb_tests). The driver state (Shutdown / Running) is not observable through the public interface of the
 * bridge: R4.3 is tested with the driver thread not yet run, run idle and after "torque set" (see the case comments).
 */
#include "doctest.h"
#include "TmcDebugFixture.h"

#include "CommandHelpers.h"
#include "HostRtos.h"
#include "voltagesense.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace {

using Bytes = std::vector<uint8_t>;

/** Hex text of bytes, for readable failure messages */
std::string hex(const Bytes &b) {
	static const char *d = "0123456789ABCDEF";
	std::string s;
	for (uint8_t c : b) {
		s += d[c >> 4];
		s += d[c & 15];
		s += ' ';
	}
	return s;
}

/** Value field (bytes 4..7) of a 9 byte reply */
Bytes valueOf(const Bytes &r) { return r.size() == 9 ? Bytes(r.begin() + 4, r.begin() + 8) : Bytes(); }

/** TMCL 2.2.1: the status of a reply is 100 (no error) or 1..6 (101 is for EEPROM programs, not a direct reply) */
bool validTmclStatus(uint8_t st) { return st == 100 || (st >= 1 && st <= 6); }

/** Register used for the chip side checks: ABN_DECODER_PPR 0x26 (R/W, used by the D1 tests) */
constexpr uint8_t kReg = 0x26;

struct FramesFixture : TmcDebugFixture {
	std::unique_ptr<TMCDebugBridge> b;

	/** Bridge constructed, construction traffic and capture cleared */
	void begin() {
		b = makeBridge();
		REQUIRE(b != nullptr);
		// CmdParser::bufferCapacity() (CmdParser.cpp:76) answers the maximum once lastAddTime exceeds the clear-buffer timeout
		// (500 ms of the CDC command interface, CommandInterface.h:80): below it the capacity checks measure the real free space.
		REQUIRE(HAL_GetTick() < 400);
		chip1.clearLog();
		clearSent();
	}
	uint32_t parserCapacity() { return b->cdcCmdInterface->bufferCapacity(); }
	void put(const Bytes &bytes) { send(*b, bytes); }
	/** Everything written to CDC so far (the tests check size == 9 themselves, this helper enforces nothing) */
	Bytes reply() const { return sent(); }
	bool noChipTraffic() const { return chip1.log().empty() && chip2.log().empty(); }

	/** Runs the driver thread "TMC" of the bridge for ms of simulated time (made resumable on first use) */
	void runTmc(uint32_t ms) {
		TaskHandle_t h = HostRtos::findTask("TMC");
		REQUIRE(h != nullptr);
		if (!HostRtos::isResumable(h)) {
			HostRtos::makeResumable(h);
		}
		const uint64_t end = HostPlatform::nowUs() + uint64_t(ms) * 1000;
		while (HostPlatform::nowUs() < end) {
			uint64_t before = HostPlatform::nowUs();
			HostRtos::runUntilBlocked(h);
			if (HostPlatform::nowUs() == before) {
				HostPlatform::advanceMs(1);
			}
		}
	}
};

} // namespace

TEST_SUITE("TMCDebugBridge frames") {

// ---------------------------------------------------------------------------------------------------------------------
// R2.1 routing by length and first byte (FW design, plain)
// ---------------------------------------------------------------------------------------------------------------------

// Oracle of the routing checks: CmdParser::add (CmdParser.cpp:53-55) appends exactly Len bytes to the ring buffer when
// there is room, and bufferCapacity() is its free space, so the capacity shrinks by exactly Len. Calibrated here with a
// known 3 byte input before the frames are judged with it.
TEST_CASE_FIXTURE(FramesFixture, "[R2.1 routing] a 3 byte text input shrinks the free capacity of the text parser by exactly 3") {
	begin();
	uint32_t cap = parserCapacity();
	put({'x', 'y', 'z'});
	CHECK(cap - parserCapacity() == 3);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.1 routing] a 9 byte frame starting with 1 is handled by the bridge and not passed to the text parser") {
	begin();
	uint32_t cap = parserCapacity();
	put(frame(148, kReg, 0));
	CHECK(reply().size() == 9);
	CHECK(parserCapacity() == cap);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.1 routing] a frame of length 0 adds nothing to the text parser and causes no reply and no chip traffic") {
	begin();
	uint32_t cap = parserCapacity();
	std::unique_ptr<char[]> empty(new char[0]); // a zero size allocation: ASan reports any read of Buf[0] (R5.3)
	uint32_t len = 0;
	b->cdcRcv(empty.get(), &len);
	CHECK(parserCapacity() == cap); // nothing to add
	CHECK(cdcWrites() == 0);
	CHECK(noChipTraffic());
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.1 routing] a frame of length 1 starting with 1 is passed to the text parser and produces no reply") {
	begin();
	uint32_t cap = parserCapacity();
	put({1});
	CHECK(cap - parserCapacity() == 1);
	CHECK(cdcWrites() == 0);
	CHECK(noChipTraffic());
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.1 routing] a frame of length 8 starting with 1 is passed to the text parser and produces no reply") {
	begin();
	Bytes f = frame(148, kReg, 0);
	f.resize(8);
	uint32_t cap = parserCapacity();
	put(f);
	CHECK(cap - parserCapacity() == 8);
	CHECK(cdcWrites() == 0);
	CHECK(noChipTraffic());
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.1 routing] a frame of length 10 starting with 1 is passed to the text parser and produces no reply") {
	begin();
	Bytes f = frame(148, kReg, 0);
	f.push_back(0);
	uint32_t cap = parserCapacity();
	put(f);
	CHECK(cap - parserCapacity() == 10);
	CHECK(cdcWrites() == 0);
	CHECK(noChipTraffic());
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.1 routing] a 9 byte frame starting with 0 is passed to the text parser and produces no reply") {
	begin();
	Bytes f = frame(148, kReg, 0);
	f[0] = 0;
	uint32_t cap = parserCapacity();
	put(f);
	CHECK(cap - parserCapacity() == 9);
	CHECK(cdcWrites() == 0);
	CHECK(noChipTraffic());
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.1 routing] a 9 byte frame starting with 2 is passed to the text parser and produces no reply") {
	begin();
	Bytes f = frame(148, kReg, 0);
	f[0] = 2;
	uint32_t cap = parserCapacity();
	put(f);
	CHECK(cap - parserCapacity() == 9);
	CHECK(cdcWrites() == 0);
	CHECK(noChipTraffic());
}

// ---------------------------------------------------------------------------------------------------------------------
// R2.2 checksum rule (TMCL 2.1.1)
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(FramesFixture, "[R2.2 checksum] the worked GGP request of the TMCL reference (checksum 0x4D) is accepted and answered") {
	begin();
	// TMCL 3.10 example: 01 0A 42 00 00 00 00 00 4D; hand: 0x01 + 0x0A + 0x42 = 1 + 10 + 66 = 77 = 0x4D
	put({0x01, 0x0A, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4D});
	REQUIRE(reply().size() == 9);
	CHECK(reply()[3] == 10);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.2 checksum] the worked GIO request of the TMCL reference (checksum 0x14) is accepted and answered") {
	begin();
	// TMCL 3.15 example: 01 0F 03 01 00 00 00 00 14; hand: 1 + 15 + 3 + 1 = 20 = 0x14
	put({0x01, 0x0F, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x14});
	REQUIRE(reply().size() == 9);
	CHECK(reply()[3] == 0x0F);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.2 checksum] a request whose byte sum exceeds 255 is accepted with the sum modulo 256") {
	begin();
	// {1, 10, 0xFF, 0, 0xFF, 0xFF, 0xFF, 0xFF}: 1 + 10 + 255 + 4 * 255 = 1286 = 5 * 256 + 6 -> checksum 6
	Bytes f = frame(10, 0xFF, 0xFFFFFFFF);
	CHECK(f[8] == 6);
	put({1, 10, 0xFF, 0, 0xFF, 0xFF, 0xFF, 0xFF, 6});
	REQUIRE(reply().size() == 9);
	CHECK(reply()[3] == 10);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.2 checksum] a request whose byte sum is exactly 256 has the checksum 0 and is accepted") {
	begin();
	// {1, 10, 245, 0, 0, 0, 0, 0}: 1 + 10 + 245 = 256 -> 0
	put({1, 10, 245, 0, 0, 0, 0, 0, 0});
	REQUIRE(reply().size() == 9);
	CHECK(reply()[3] == 10);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.2 checksum] the checksum of a reply is the 8 bit sum of its first eight bytes") {
	begin();
	// register 0 -> reply head {2, 1, 0x64, 0x94, 0, 0, 0, 0}: 2 + 1 + 100 + 148 = 251 = 0xFB
	chip1.setReg(kReg, 0);
	put(frame(148, kReg, 0));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[8] == 0xFB);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.2 checksum] the checksum of a reply whose sum wraps is the sum modulo 256") {
	begin();
	// value 0xFFFFFFFF: head sum 251 + 4 * 255 = 1271 = 4 * 256 + 247 -> 0xF7
	chip1.setReg(kReg, 0xFFFFFFFF);
	put(frame(148, kReg, 0));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[8] == 0xF7);
}

// ---------------------------------------------------------------------------------------------------------------------
// R2.3 wrong checksum (TMCL 2.2, 2.2.1)
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(FramesFixture, "[R2.3 checksum] a write frame with a wrong checksum does not write the chip") {
	begin();
	chip1.setReg(kReg, 0x11111111);
	chip1.clearLog();
	put(frame(146, kReg, 0xAABBCCDD, 0x00)); // the right checksum is not 0
	CHECK(noChipTraffic());
	CHECK(chip1.reg(kReg) == 0x11111111);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.3 checksum] a read frame with a wrong checksum does not read the chip") {
	begin();
	put(frame(148, kReg, 0, 0x00));
	CHECK(noChipTraffic());
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.3 checksum] a frame with a wrong checksum is not passed to the text parser") {
	begin();
	uint32_t cap = parserCapacity();
	put(frame(148, kReg, 0, 0x00));
	CHECK(parserCapacity() == cap);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.3 checksum] a checksum that is off by one is rejected: the chip is not read" ) {
	begin();
	Bytes f = frame(148, kReg, 0);
	f[8] = static_cast<uint8_t>(f[8] + 1);
	put(f);
	CHECK(noChipTraffic());
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.3 checksum] a frame with a wrong checksum is answered with status 1" *
				  KNOWN_ISSUE("B-DTDF-1: a frame with a wrong checksum is dropped silently (TDB:143-146), TMCL 2.2 / 2.2.1 want a reply with status 1 (wrong checksum)")) {
	begin();
	put(frame(148, kReg, 0, 0x00));
	REQUIRE(reply().size() == 9); // intended: a reply is sent
	CHECK(reply()[2] == 1); // status: wrong checksum (the defect assertion)
	// Secondary: TMCL 2.2 gives the reply layout; that an error reply keeps the layout (addresses, command echo, checksum)
	// is derived from it, the text does not state it for errors.
	CHECK(reply()[0] == 2);
	CHECK(reply()[1] == 1);
	CHECK(reply()[3] == 148);
	CHECK(reply()[8] == TmcDebugFixture::sum8(reply(), 8));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.3 checksum] a write frame with a wrong checksum is answered with status 1" *
				  KNOWN_ISSUE("B-DTDF-1: a frame with a wrong checksum is dropped silently (TDB:143-146), TMCL 2.2 / 2.2.1 want a reply with status 1 (wrong checksum)")) {
	begin();
	put(frame(146, kReg, 1, 0x00));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[2] == 1);
	CHECK(reply()[3] == 146); // secondary, derived from the TMCL 2.2 layout (not stated for errors)
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.3 checksum] a GGP frame with a wrong checksum is answered with status 1" *
				  KNOWN_ISSUE("B-DTDF-1: a frame with a wrong checksum is dropped silently (TDB:143-146), TMCL 2.2 / 2.2.1 want a reply with status 1 (wrong checksum)")) {
	begin();
	put(frame(10, 5, 0, 0x00));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[2] == 1);
	CHECK(reply()[3] == 10); // secondary, derived from the TMCL 2.2 layout (not stated for errors)
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.3 checksum] frames of the commands 10, 15, 136 (types 0 and 1) and 143 with a wrong checksum cause no register traffic and are not passed to the text parser") {
	begin();
	uint32_t cap = parserCapacity();
	put(frame(10, 5, 0, 0x00));
	put(frame(15, 5, 0, 0x00));
	put(frame(0x88, 0, 0, 0x00));
	put(frame(0x88, 1, 0, 0x00));
	put(frame(143, 3, 0, 0x00));
	CHECK(noChipTraffic());
	CHECK(parserCapacity() == cap);
}

// ---------------------------------------------------------------------------------------------------------------------
// R2.4 reply layout (TMCL 2.2)
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(FramesFixture, "[R2.4 layout] the reply to a register read is {2, 1, status 100, command, value MSB first, checksum}") {
	begin();
	chip1.setReg(kReg, 0x01020304);
	put(frame(148, kReg, 0));
	// head {2, 1, 0x64, 0x94, 1, 2, 3, 4}: 2 + 1 + 100 + 148 + 1 + 2 + 3 + 4 = 261 = 256 + 5 -> checksum 5
	CHECK(hex(reply()) == hex({2, 1, 0x64, 0x94, 1, 2, 3, 4, 5}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.4 layout] the reply to a register write is {2, 1, status 100, command, 4 value bytes, checksum}") {
	begin();
	put(frame(146, kReg, 0x01020304));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[0] == 2);
	CHECK(reply()[1] == 1);
	CHECK(reply()[2] == 100);
	CHECK(reply()[3] == 146);
	CHECK(reply()[8] == TmcDebugFixture::sum8(reply(), 8));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.4 layout] the reply to the version request type 1 is {2, 1, status 100, 136, 4 value bytes, checksum}") {
	begin();
	put(frame(0x88, 1, 0));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[0] == 2);
	CHECK(reply()[1] == 1);
	CHECK(reply()[2] == 100);
	CHECK(reply()[3] == 136);
	CHECK(reply()[8] == TmcDebugFixture::sum8(reply(), 8));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.4 status] the reply to GGP carries status 100 for a successful command" *
				  KNOWN_ISSUE("B-DTDF-5: the GGP reply carries status 64 (TDB:180), TMCL 2.2.1 lists only 100 (no error) and 1..6; intended 100")) {
	begin();
	put(frame(10, 5, 0));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[2] == 100);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.4 status] the reply to GIO carries status 100 for a successful command" *
				  KNOWN_ISSUE("B-DTDF-5: the GIO reply carries status 64 (TDB:189), TMCL 2.2.1 lists only 100 (no error) and 1..6; intended 100")) {
	begin();
	put(frame(15, 5, 0));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[2] == 100);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.7 status] the reply to command 143 type 3 carries a status of the TMCL table" *
				  KNOWN_ISSUE("B-DTDF-6: the command 143 replies carry status 40 (TDB:173), TMCL 2.2.1 lists only 100 and 1..6 (opcode 143 itself is not in the supplied documents, so only membership is asserted)")) {
	begin();
	put(frame(143, 3, 0));
	REQUIRE(reply().size() == 9);
	CHECK(validTmclStatus(reply()[2]));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.7 status] the reply to command 143 type 4 carries a status of the TMCL table" *
				  KNOWN_ISSUE("B-DTDF-6: the command 143 replies carry status 40 (TDB:174), TMCL 2.2.1 lists only 100 and 1..6 (opcode 143 itself is not in the supplied documents, so only membership is asserted)")) {
	begin();
	put(frame(143, 4, 0));
	REQUIRE(reply().size() == 9);
	CHECK(validTmclStatus(reply()[2]));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.4 layout] the replies to GGP, GIO and the version request carry the reply address 2 and the module address 1 and a valid checksum") {
	begin();
	for (uint8_t cmd : {uint8_t(10), uint8_t(15), uint8_t(0x88)}) {
		clearSent();
		put(frame(cmd, 1, 0));
		REQUIRE(reply().size() == 9);
		CHECK(reply()[0] == 2);
		CHECK(reply()[1] == 1);
		CHECK(reply()[3] == (cmd == 0x88 ? 136 : cmd)); // command number echoed (TMCL 2.2)
		CHECK(reply()[8] == TmcDebugFixture::sum8(reply(), 8));
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// R2.5 command 148, read register
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(FramesFixture, "[R2.5 read] cmd 148 reads the register addressed by the type byte with one read frame at chip 1") {
	begin();
	put(frame(148, kReg, 0));
	REQUIRE(chip1.log().size() == 1);
	CHECK_FALSE(chip1.log()[0].write);
	CHECK(chip1.log()[0].addr == kReg);
	CHECK(chip2.log().empty());
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.5 read] cmd 148 sends the register content big endian in the value field") {
	begin();
	chip1.setReg(kReg, 0xA1B2C3D4);
	put(frame(148, kReg, 0));
	// head {2, 1, 0x64, 0x94, A1, B2, C3, D4}: 2+1+100+148 = 251; 161+178+195+212 = 746; 997 = 3 * 256 + 229 -> 0xE5
	CHECK(hex(reply()) == hex({2, 1, 0x64, 0x94, 0xA1, 0xB2, 0xC3, 0xD4, 0xE5}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.5 read] cmd 148 does not write to the chip") {
	begin();
	put(frame(148, kReg, 0x12345678)); // the value field of a read request is ignored
	CHECK(chip1.writesTo(kReg).empty());
	for (const auto &f : chip1.log()) {
		CHECK_FALSE(f.write);
	}
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.5 read] cmd 148 of a register with the high bit set keeps the byte order (0x80000001)") {
	begin();
	chip1.setReg(kReg, 0x80000001);
	put(frame(148, kReg, 0));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[4] == 0x80);
	CHECK(reply()[5] == 0x00);
	CHECK(reply()[6] == 0x00);
	CHECK(reply()[7] == 0x01);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.5 read] characterization: command number 148 (0x94) is the register read and its reply carries 0x94") {
	// not in the supplied TMCL documents (FW): NEEDS REVIEW
	begin();
	chip1.setReg(kReg, 7);
	put(frame(148, kReg, 0));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[3] == 0x94);
	CHECK(reply()[7] == 7);
}

// ---------------------------------------------------------------------------------------------------------------------
// R2.6 command 146, write register
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(FramesFixture, "[R2.6 write] cmd 146 writes the register addressed by the type byte with the big endian value of bytes 4 to 7") {
	begin();
	put(frame(146, kReg, 0x12345678));
	REQUIRE(chip1.writesTo(kReg).size() == 1);
	CHECK(chip1.writesTo(kReg)[0] == 0x12345678);
	CHECK(chip1.reg(kReg) == 0x12345678);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.6 write] cmd 146 with the value 0x000000FF writes 255, not 0xFF000000") {
	begin();
	put(frame(146, kReg, 0x000000FF));
	CHECK(chip1.reg(kReg) == 0x000000FF);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.6 write] cmd 146 with the value 0xFF000000 writes 0xFF000000, not 255") {
	begin();
	put(frame(146, kReg, 0xFF000000));
	CHECK(chip1.reg(kReg) == 0xFF000000);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.6 write] cmd 146 writes exactly one register frame at chip 1 and reads nothing") {
	begin();
	put(frame(146, kReg, 0x01020304));
	REQUIRE(chip1.log().size() == 1);
	CHECK(chip1.log()[0].write);
	CHECK(chip1.log()[0].addr == kReg);
	CHECK(chip1.log()[0].data == 0x01020304);
	CHECK(chip2.log().empty());
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.6 write] characterization: the reply to command 146 (0x92) echoes request bytes 3 to 6 in the value field") {
	// FW: the value is in request bytes 4..7, the code echoes dat[3..6] (R5 item 6). NEEDS REVIEW.
	begin();
	Bytes f = frame(146, kReg, 0x11223344); // request bytes 3..6 = {0, 0x11, 0x22, 0x33}
	put(f);
	// head {2, 1, 0x64, 0x92, 0, 0x11, 0x22, 0x33}: 2+1+100+146 = 249; 0+17+34+51 = 102; 351 = 256 + 95 -> 0x5F
	CHECK(hex(reply()) == hex({2, 1, 0x64, 0x92, 0x00, 0x11, 0x22, 0x33, 0x5F}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.6 write] characterization: the reply to command 146 echoes request byte 3 (motor/bank 0x5A) as the first value byte") {
	// FW. NEEDS REVIEW. Request {1, 146, 0x26, 0x5A, 0x11, 0x22, 0x33, 0x44, cs}: 1+146+38+90+17+34+51+68 = 445 = 256 + 189 -> 0xBD.
	// Reply head {2, 1, 0x64, 0x92, 0x5A, 0x11, 0x22, 0x33}: 2+1+100+146 = 249; 90+17+34+51 = 192; 441 = 256 + 185 -> 0xB9.
	begin();
	put({1, 146, kReg, 0x5A, 0x11, 0x22, 0x33, 0x44, 0xBD});
	CHECK(hex(reply()) == hex({2, 1, 0x64, 0x92, 0x5A, 0x11, 0x22, 0x33, 0xB9}));
	CHECK(chip1.reg(kReg) == 0x11223344);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.6 write] characterization: command number 146 (0x92) is the register write and its reply carries 0x92") {
	// not in the supplied TMCL documents (FW): NEEDS REVIEW
	begin();
	put(frame(146, kReg, 1));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[3] == 0x92);
}

// ---------------------------------------------------------------------------------------------------------------------
// R2.7 command 143
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(FramesFixture, "[R2.7 cmd143] characterization: command 143 type 3 answers {2, 1, 40, 0x8F, 0, 0, 0, 2, checksum}") {
	// FW (R5 item 4: opcode and meaning not in the supplied documents). NEEDS REVIEW. Status 40 is not 100 (TMCL 2.2.1).
	begin();
	put(frame(143, 3, 0));
	// 2+1+40+143+0+0+0+2 = 188 = 0xBC
	CHECK(hex(reply()) == hex({2, 1, 40, 0x8F, 0, 0, 0, 2, 0xBC}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.7 cmd143] characterization: command 143 type 4 answers {2, 1, 40, 0x8F, 2, 6, 2, 2, checksum}") {
	// FW. NEEDS REVIEW.
	begin();
	put(frame(143, 4, 0));
	// 2+1+40+143+2+6+2+2 = 198 = 0xC6
	CHECK(hex(reply()) == hex({2, 1, 40, 0x8F, 2, 6, 2, 2, 0xC6}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.7 cmd143] command 143 causes no register traffic for any type") {
	begin();
	for (uint8_t type : {uint8_t(0), uint8_t(3), uint8_t(4), uint8_t(5), uint8_t(0xFF)}) {
		put(frame(143, type, 0));
	}
	CHECK(noChipTraffic());
}

// Opcode 143 is not in the supplied documents, so the intended status for an unknown type is not specific: 2 (invalid command),
// 3 (wrong type), 4 (invalid value) and 6 (command not available) are all defensible. Asserted: a status of the TMCL
// table (TMCL 2.2.1) that is not 0.
TEST_CASE_FIXTURE(FramesFixture, "[R2.7 unknown-type] command 143 with the unknown type 5 is answered with a status of the TMCL table" *
				  KNOWN_ISSUE("B-DTDF-2: command 143 with an unknown type answers an all-zero frame with a valid checksum, status 0 (TDB:171-178); TMCL 2.2.1 lists only 100 and 1..6")) {
	begin();
	put(frame(143, 5, 0));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[2] != 0);
	CHECK(validTmclStatus(reply()[2]));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.7 unknown-type] command 143 with the unknown type 0 is answered with a status of the TMCL table" *
				  KNOWN_ISSUE("B-DTDF-2: command 143 with an unknown type answers an all-zero frame with a valid checksum, status 0 (TDB:171-178); TMCL 2.2.1 lists only 100 and 1..6")) {
	begin();
	put(frame(143, 0, 0));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[2] != 0);
	CHECK(validTmclStatus(reply()[2]));
}

// ---------------------------------------------------------------------------------------------------------------------
// R2.8 GGP (command 10, TMCL 3.10) and GIO (command 15, TMCL 3.15)
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(FramesFixture, "[R2.8 ggp] command 10 is answered with command number 10 and the reply layout") {
	begin();
	put(frame(10, 1, 0)); // TMCL 3.10: instruction number 10
	REQUIRE(reply().size() == 9);
	CHECK(reply()[0] == 2);
	CHECK(reply()[1] == 1);
	CHECK(reply()[3] == 10);
	CHECK(reply()[8] == TmcDebugFixture::sum8(reply(), 8));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.8 ggp] command 10 causes no register traffic") {
	begin();
	put(frame(10, 5, 0));
	put(frame(10, 2, 0));
	CHECK(noChipTraffic());
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.8 ggp] characterization: GGP type 5 returns the value 2") {
	// FW (comment in the code: hwid). NEEDS REVIEW.
	begin();
	put(frame(10, 5, 0));
	REQUIRE(reply().size() == 9);
	CHECK(valueOf(reply()) == Bytes({0, 0, 0, 2}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.8 ggp] characterization: GGP type 2 returns the value 1") {
	// FW (comment in the code: active). NEEDS REVIEW.
	begin();
	put(frame(10, 2, 0));
	REQUIRE(reply().size() == 9);
	CHECK(valueOf(reply()) == Bytes({0, 0, 0, 1}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.8 ggp] characterization: GGP types other than 2 and 5 return the value 0") {
	// FW. NEEDS REVIEW.
	begin();
	for (uint8_t type : {uint8_t(0), uint8_t(1), uint8_t(3), uint8_t(4), uint8_t(6), uint8_t(66), uint8_t(255)}) {
		clearSent();
		put(frame(10, type, 0));
		REQUIRE(reply().size() == 9);
		CHECK_MESSAGE(valueOf(reply()) == Bytes({0, 0, 0, 0}), "type " << int(type));
	}
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.8 gio] command 15 is answered with command number 15 and the reply layout") {
	begin();
	put(frame(15, 1, 0)); // TMCL 3.15: instruction number 15
	REQUIRE(reply().size() == 9);
	CHECK(reply()[0] == 2);
	CHECK(reply()[1] == 1);
	CHECK(reply()[3] == 15);
	CHECK(reply()[8] == TmcDebugFixture::sum8(reply(), 8));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.8 gio] characterization: GIO type 5 returns the internal supply in units of 0.1 V, big endian") {
	// FW (not in the supplied documents). NEEDS REVIEW.
	// Derivation: TmcFixture reference 4095 mV -> 1 ADC count = 1 mV at the pin, getIntV() = counts * vSenseMult in mV
	// (TmcFixture header). setSupply(24000, 24550) makes getIntV() = 24550 mV (+- the quantisation of one count, a few mV,
	// well inside 24500..24599). 24550 / 100 = 245 (integer division) = 0x00F5 -> value bytes {0, 0, 0x00, 0xF5}.
	begin();
	setSupply(24000, 24550);
	REQUIRE(getIntV() > 24500);
	REQUIRE(getIntV() < 24600);
	put(frame(15, 5, 0));
	REQUIRE(reply().size() == 9);
	CHECK(valueOf(reply()) == Bytes({0, 0, 0x00, 0xF5}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.8 gio] characterization: GIO type 5 puts the high byte of the voltage into value byte 2") {
	// FW. Derivation: setSupply(24000, 25650) -> getIntV() = 25650 mV (+- a few mV, inside 25600..25699);
	// 25650 / 100 = 256 = 0x0100 -> value bytes {0, 0, 0x01, 0x00}. NEEDS REVIEW.
	begin();
	setSupply(24000, 25650);
	REQUIRE(getIntV() > 25600);
	REQUIRE(getIntV() < 25700);
	put(frame(15, 5, 0));
	REQUIRE(reply().size() == 9);
	CHECK(valueOf(reply()) == Bytes({0, 0, 0x01, 0x00}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.8 gio] characterization: GIO type 5 truncates the supply to 0.1 V steps (24550 mV or more stays 245)") {
	// FW. NEEDS REVIEW. Derivation: a supply request of 24599 mV gives getIntV() in 24550..24599 mV (REQUIRE below); integer
	// division by 100 truncates to 245 = 0x00F5, a rounding implementation would give 246 for values from 24550 up.
	begin();
	setSupply(24000, 24599);
	REQUIRE(getIntV() >= 24550);
	REQUIRE(getIntV() <= 24599);
	put(frame(15, 5, 0));
	REQUIRE(reply().size() == 9);
	CHECK(valueOf(reply()) == Bytes({0, 0, 0x00, 0xF5}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.8 gio] characterization: GIO types other than 5 return the value 0 whatever the supply is") {
	// FW. NEEDS REVIEW.
	begin();
	setSupply(24000, 24550);
	for (uint8_t type : {uint8_t(0), uint8_t(1), uint8_t(3), uint8_t(4), uint8_t(6), uint8_t(255)}) {
		clearSent();
		put(frame(15, type, 0));
		REQUIRE(reply().size() == 9);
		CHECK_MESSAGE(valueOf(reply()) == Bytes({0, 0, 0, 0}), "type " << int(type));
	}
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.8 gio] command 15 causes no register traffic") {
	begin();
	put(frame(15, 5, 0));
	CHECK(noChipTraffic());
}

// ---------------------------------------------------------------------------------------------------------------------
// R2.9 command 0x88 = 136, get firmware version (TMCL 3.35)
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(FramesFixture, "[R2.9 version] command 136 type 0 answers 9 bytes with the host address 2 first") {
	begin();
	put(frame(0x88, 0, 0));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[0] == 2);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.9 version] command 136 type 0 answers a string of 8 printable characters without a checksum byte") {
	begin();
	put(frame(0x88, 0, 0));
	REQUIRE(reply().size() == 9);
	// TMCL 3.35: byte 1 host address, bytes 2..9 the 8 character version string, no checksum: all of bytes 1..8
	// (0 based) are characters.
	for (size_t i = 1; i < 9; i++) {
		CHECK_MESSAGE((reply()[i] >= 0x20 && reply()[i] <= 0x7E), "byte " << i);
	}
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.9 version] command 136 type 0 is one write of 9 bytes and one flush") {
	begin();
	put(frame(0x88, 0, 0));
	CHECK(cdcWrites() == 1);
	CHECK(cdcFlushes() == 1);
	CHECK(reply().size() == 9);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.9 version] characterization: the version string is 0015V307") {
	// FW (TDB:198). NEEDS REVIEW.
	begin();
	put(frame(0x88, 0, 0));
	REQUIRE(reply().size() == 9);
	CHECK(hex(reply()) == hex({2, '0', '0', '1', '5', 'V', '3', '0', '7'}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.9 version] characterization: the binary version of command 136 type 1 is the value bytes 0, 15, 3, 7") {
	// FW. Hand derivation from "0015V307": module high = (0 * 10) + 0 = 0, module low = (1 * 10) + 5 = 15,
	// firmware high = 3, firmware low = (0 * 10) + 7 = 7. The spec layout of the value field (TMCL 3.35) differs, see the
	// report ("Lead decisions needed"). NEEDS REVIEW.
	begin();
	put(frame(0x88, 1, 0));
	// head {2, 1, 0x64, 0x88, 0, 15, 3, 7}: 2+1+100+136 = 239; 0+15+3+7 = 25; 264 = 256 + 8 -> 8
	CHECK(hex(reply()) == hex({2, 1, 0x64, 0x88, 0, 15, 3, 7, 8}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.9 version] command 136 types 0 and 1 cause no register traffic") {
	begin();
	put(frame(0x88, 0, 0));
	put(frame(0x88, 1, 0));
	CHECK(noChipTraffic());
}

// ---------------------------------------------------------------------------------------------------------------------
// R2.10 unknown commands and bad types
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(FramesFixture, "[R2.10 unknown] an unknown command number causes no register traffic") {
	begin();
	put(frame(200, kReg, 0x01020304));
	CHECK(noChipTraffic());
	CHECK(chip1.reg(kReg) != 0x01020304);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.10 unknown] an unknown command number is not passed to the text parser") {
	begin();
	uint32_t cap = parserCapacity();
	put(frame(200, kReg, 0));
	CHECK(parserCapacity() == cap);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.10 unknown] command 136 with type 2 causes no register traffic and is not passed to the text parser") {
	begin();
	uint32_t cap = parserCapacity();
	put(frame(0x88, 2, 0));
	CHECK(noChipTraffic());
	CHECK(parserCapacity() == cap);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.10 unknown] an unknown command number is answered with status 2" *
				  KNOWN_ISSUE("B-DTDF-3: an unknown command number is dropped silently (TDB:228, no else branch), TMCL 2.2 / 2.2.1 want a reply with status 2 (invalid command)")) {
	begin();
	put(frame(200, kReg, 0));
	REQUIRE(reply().size() == 9); // intended: a reply is sent
	CHECK(reply()[2] == 2); // status: invalid command (the defect assertion)
	// Secondary: layout of an error reply derived from TMCL 2.2 (not stated for errors there).
	CHECK(reply()[0] == 2);
	CHECK(reply()[1] == 1);
	CHECK(reply()[3] == 200);
	CHECK(reply()[8] == TmcDebugFixture::sum8(reply(), 8));
}

// Neighbours of the implemented opcodes. 9 (SGP), 11 (STGP), 135, 137 are TMCL commands the bridge does not implement: 2
// (invalid command) or 6 (command not available) are both defensible; 147 / 149 are undocumented.
TEST_CASE_FIXTURE(FramesFixture, "[R2.10 unknown] the neighbours of the implemented opcodes are answered with status 2 or 6" *
				  KNOWN_ISSUE("B-DTDF-3: an unknown command number is dropped silently (TDB:228, no else branch), TMCL 2.2 / 2.2.1 want a reply with status 2 (invalid command) or 6")) {
	begin();
	for (uint8_t cmd : {uint8_t(9), uint8_t(11), uint8_t(135), uint8_t(137), uint8_t(147), uint8_t(149)}) {
		clearSent();
		put(frame(cmd, kReg, 0));
		REQUIRE_MESSAGE(reply().size() == 9, "command " << int(cmd));
		CHECK_MESSAGE((reply()[2] == 2 || reply()[2] == 6), "command " << int(cmd));
	}
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.10 unknown] the neighbours of the implemented opcodes and command 136 type 255 cause no register traffic and are not passed to the text parser") {
	begin();
	uint32_t cap = parserCapacity();
	for (uint8_t cmd : {uint8_t(9), uint8_t(11), uint8_t(135), uint8_t(137), uint8_t(147), uint8_t(149)}) {
		put(frame(cmd, kReg, 0x01020304));
	}
	put(frame(0x88, 255, 0));
	CHECK(noChipTraffic());
	CHECK(chip1.reg(kReg) != 0x01020304);
	CHECK(parserCapacity() == cap);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.10 unknown] command 136 with type 2 is answered with status 3" *
				  KNOWN_ISSUE("B-DTDF-4: command 136 with a type other than 0 / 1 is dropped silently (TDB:197-226), TMCL 2.2.1 wants a reply with status 3 (wrong type)")) {
	begin();
	put(frame(0x88, 2, 0));
	REQUIRE(reply().size() == 9); // intended: a reply is sent
	CHECK(reply()[2] == 3); // status: wrong type (the defect assertion)
	// Secondary: layout of an error reply derived from TMCL 2.2 (not stated for errors there).
	CHECK(reply()[0] == 2);
	CHECK(reply()[1] == 1);
	CHECK(reply()[3] == 136);
	CHECK(reply()[8] == TmcDebugFixture::sum8(reply(), 8));
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.10 unknown] command 136 with type 255 is answered with status 3" *
				  KNOWN_ISSUE("B-DTDF-4: command 136 with a type other than 0 / 1 is dropped silently (TDB:197-226), TMCL 2.2.1 wants a reply with status 3 (wrong type)")) {
	begin();
	put(frame(0x88, 255, 0));
	REQUIRE(reply().size() == 9);
	CHECK(reply()[2] == 3);
}

// ---------------------------------------------------------------------------------------------------------------------
// R2.11 one 9 byte write and one flush per reply
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(FramesFixture, "[R2.11 write] every answered command is sent with one write of 9 bytes and one flush") {
	begin();
	const std::vector<Bytes> requests = {frame(148, kReg, 0), frame(146, kReg, 1), frame(143, 3, 0), frame(10, 5, 0),
										 frame(15, 5, 0),	  frame(0x88, 0, 0),   frame(0x88, 1, 0)};
	for (const Bytes &r : requests) {
		clearSent();
		put(r);
		CHECK_MESSAGE(cdcWrites() == 1, "command " << int(r[1]) << " type " << int(r[2]));
		CHECK_MESSAGE(cdcFlushes() == 1, "command " << int(r[1]) << " type " << int(r[2]));
		CHECK_MESSAGE(sent().size() == 9, "command " << int(r[1]) << " type " << int(r[2]));
	}
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.11 write] two requests produce two separate replies, each with its own write and flush") {
	begin();
	chip1.setReg(kReg, 0x01020304);
	put(frame(148, kReg, 0));
	put(frame(148, kReg, 0));
	CHECK(cdcWrites() == 2);
	CHECK(cdcFlushes() == 2);
	REQUIRE(sent().size() == 18);
	const Bytes both = sent();
	CHECK(Bytes(both.begin(), both.begin() + 9) == Bytes(both.begin() + 9, both.end()));
}

// ---------------------------------------------------------------------------------------------------------------------
// R2.12 short CDC buffer
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(FramesFixture, "[R2.12 cdc] characterization: with 5 bytes of CDC buffer free the reply is truncated to its first 5 bytes") {
	// FW: tud_cdc_n_write accepts what fits, the bridge ignores the return value. NEEDS REVIEW.
	begin();
	HostUsb::state().cdcWriteAvailable = 5;
	chip1.setReg(kReg, 0x01020304);
	put(frame(148, kReg, 0));
	CHECK(hex(sent()) == hex({2, 1, 0x64, 0x94, 1}));
	CHECK(cdcFlushes() == 1);
}

TEST_CASE_FIXTURE(FramesFixture, "[R2.12 cdc] characterization: with no CDC buffer free nothing is sent and the bridge does not fail") {
	// FW. NEEDS REVIEW.
	begin();
	HostUsb::state().cdcWriteAvailable = 0;
	put(frame(148, kReg, 0));
	CHECK(sent().empty());
	CHECK(chip1.log().size() == 1); // the register was still read
}

// ---------------------------------------------------------------------------------------------------------------------
// R4.1 / R4.2 / R4.3 robustness
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(FramesFixture, "[R4.1 split] characterization: a frame split into 5 and 4 bytes is not reassembled, both parts go to the text parser") {
	// FW. NEEDS REVIEW.
	begin();
	Bytes f = frame(148, kReg, 0);
	uint32_t cap = parserCapacity();
	put(Bytes(f.begin(), f.begin() + 5));
	CHECK(cap - parserCapacity() == 5);
	uint32_t cap2 = parserCapacity();
	put(Bytes(f.begin() + 5, f.end()));
	CHECK(cap2 - parserCapacity() == 4);
	CHECK(cdcWrites() == 0);
	CHECK(noChipTraffic());
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.2 recovery] a valid frame after a frame with a wrong checksum is answered") {
	begin();
	chip1.setReg(kReg, 0x01020304);
	put(frame(148, kReg, 0, 0x00));
	clearSent();
	put(frame(148, kReg, 0));
	CHECK(hex(reply()) == hex({2, 1, 0x64, 0x94, 1, 2, 3, 4, 5}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.2 recovery] a valid frame after an unknown command is answered") {
	begin();
	chip1.setReg(kReg, 0x01020304);
	put(frame(200, kReg, 0));
	clearSent();
	put(frame(148, kReg, 0));
	CHECK(hex(reply()) == hex({2, 1, 0x64, 0x94, 1, 2, 3, 4, 5}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.2 recovery] a valid frame after a frame of the wrong length is answered") {
	begin();
	chip1.setReg(kReg, 0x01020304);
	put({'x', 'y', '\n'});
	clearSent();
	put(frame(148, kReg, 0));
	CHECK(hex(reply()) == hex({2, 1, 0x64, 0x94, 1, 2, 3, 4, 5}));
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.2 recovery] a write after an invalid version request type reaches the chip") {
	begin();
	put(frame(0x88, 7, 0));
	put(frame(146, kReg, 0xCAFE0001));
	CHECK(chip1.reg(kReg) == 0xCAFE0001);
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.2 recovery] the same request sent three times gets three identical replies") {
	begin();
	chip1.setReg(kReg, 0xA1B2C3D4);
	Bytes first;
	for (int i = 0; i < 3; i++) {
		clearSent();
		put(frame(148, kReg, 0));
		REQUIRE(reply().size() == 9);
		if (i == 0) {
			first = reply();
		}
		CHECK(hex(reply()) == hex(first));
	}
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.3 driver] register read and write reach the chip while the driver thread has not run yet") {
	begin();
	chip1.setReg(kReg, 0x01020304);
	put(frame(148, kReg, 0));
	CHECK(hex(reply()) == hex({2, 1, 0x64, 0x94, 1, 2, 3, 4, 5}));
	put(frame(146, kReg, 0x0A0B0C0D));
	CHECK(chip1.reg(kReg) == 0x0A0B0C0D);
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.3 driver] register read and write reach the chip after the driver thread has run idle for 10 s") {
	// The state of the driver (Shutdown or another waiting state) is not observable through the bridge's public interface.
	begin();
	chip1.setAdcRaw(0, 0x7FFF7FFF);
	runTmc(10000);
	REQUIRE_FALSE(chip1.log().empty()); // the driver thread ran and talked to the chip
	chip1.setReg(kReg, 0x01020304);
	clearSent();
	put(frame(148, kReg, 0));
	CHECK(hex(reply()) == hex({2, 1, 0x64, 0x94, 1, 2, 3, 4, 5}));
	put(frame(146, kReg, 0x0A0B0C0D));
	CHECK(chip1.reg(kReg) == 0x0A0B0C0D);
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.3 driver] register read and write reach the chip after the motor was started by the text command torque") {
	// "torque set" starts the motor first (R1.2); the driver thread then runs for 10 s. The state reached
	// (Running or the start sequence) is not observable through the public interface.
	begin();
	chip1.setAdcRaw(0, 0x7FFF7FFF);
	runTmc(10000);
	chip1.clearLog();
	CHECK(CommandHelpers::run(*b, "torque", CMDtype::set, 100) == CommandStatus::OK);
	runTmc(10000);
	{
		// the start request: startMotor() sets the PWM register 0x1A to 7 (FOC PWM, DS 0x1A; as measured by TDC)
		bool started = false;
		for (uint32_t v : chip1.writesTo(0x1A)) {
			started = started || (v & 0xFF) == 7;
		}
		REQUIRE(started);
	}
	chip1.setReg(kReg, 0x01020304);
	clearSent();
	put(frame(148, kReg, 0));
	CHECK(hex(reply()) == hex({2, 1, 0x64, 0x94, 1, 2, 3, 4, 5}));
	put(frame(146, kReg, 0x0A0B0C0D));
	CHECK(chip1.reg(kReg) == 0x0A0B0C0D);
}

} // TEST_SUITE
