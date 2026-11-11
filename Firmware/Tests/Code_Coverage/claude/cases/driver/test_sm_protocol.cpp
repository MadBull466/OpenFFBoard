/*
 * SimpleMotion driver, protocol: getSettings sequence, parameter transfers, receive path, `reg` / `devtype` / `crcerr` /
 * `uarterr` commands (phase P15 T2, work order SMP). Requirements R4.1-R4.6, R5.1-R5.8 and the R10 parts named above of
 * Tests/Code_Coverage/claude/charters/driver/SIMPLEMOTION_REQUIREMENTS.md (lead decisions L1-L4).
 *
 * Unit under test: MotorSimplemotion::getSettings / getCpr / read1Parameter / set1Parameter / queueCommand / uartRcv (through
 * the peer fake) / command (reg, devtype, crcerr, uarterr). Peer: fakes/FakeSmDevice.h through SmFixture.
 *
 * Oracles: SM (SimpleMotion V2 library: frame id 0x24, CRC-16/ARC with the low byte first on the wire, subpacket sizes 4 / 3 / 2 / 1
 * for type 0 / 1 / 2 / 3, parameter ids), HD (every request frame is derived by hand in a comment or built by the independent
 * encoder `sub()` / `frame()` / `arc()` in this file, never by the firmware's table), IF, FW (`characterization:`).
 * The address of the driver is indeterminate (B-SM-1): tests read it from the wire and never assert a value.
 *
 * Host versus target: MotorSimplemotion.cpp is compiled with -funsigned-char here (like ARM), so reply bytes of 0x80 and above
 * decode like on the target. Reply values are 32 bit unsigned, no float, no packed struct in this file. The 22 bit and 14 bit
 * values below are chosen inside the subpacket ranges, so no sign-extension difference exists.
 *
 * Characterizations (NEEDS REVIEW in PROGRESS.md): R4.2 Hall / Resolver / Serial cpr factor 4 ("TODO check" in the code), R4.5
 * 150 ms window and its boundary, R4.3 hardfault stays set after the device was repaired, R5.7 no sign extension, R5.8 address
 * above 0x3FFF, R5.4 uart error count after an overrun is not asserted exactly.
 *
 * KNOWN_ISSUE: B-SM-5 (new, R5.3): a reply rejected by the receive path (wrong CRC-16, wrong address) calls resetBuffer(),
 * which clears waitingReply; the busy-wait loop of readParameter then ends as if the reply had arrived and the call returns
 * true with the stale replyvalues[] of an earlier transfer. Two tests (wrong CRC, wrong address); the first failing
 * assertion is the return value, the stale value is never asserted.
 * Overrun boundary: the fake's replies have an even payload length (3 + n + 2 bytes with n even), so the exact 32 / 33 byte
 * buffer boundary is not hit; the overrun case uses 35 bytes. The sanitizer build proves that no byte lands beyond rxbuf.
 * Not tested: B-SM-4 (uninitialised `st`, stack content), so no test asserts the exact uarterr count of a silent getSettings.
 * Not reachable through the fake: reply gaps above 10 ms (the fake applies the gap and delivers in the same poll), a custom
 * status subpacket value (the fake sends ACK 0 only).
 */
#include "doctest.h"
#include "SmFixture.h"

#include "ErrorHandler.h"
#include "SystemCommands.h"

#include <cstdint>
#include <string>
#include <vector>

namespace {

// The enums of the driver are private types of MotorSimplemotion; the argument types of the public functions are taken from their
// pointers so the tests never name a private member.
template <class F> struct Sig;
template <class R, class C, class A, class B, class D> struct Sig<R (C::*)(A, B, D)> {
	using First = A;
	using Second = B;
	using Third = D;
};
using ReadSig = Sig<decltype(&MotorSimplemotion::read1Parameter)>;
using WriteSig = Sig<decltype(&MotorSimplemotion::set1Parameter)>;
using QueueSig = Sig<decltype(&MotorSimplemotion::queueCommand)>;
using ParamId = ReadSig::First;
using LenType = ReadSig::Third;
using CmdType = QueueSig::Second;

constexpr int kNone = -1; // MotorSimplemotion_cmdtypes::none
enum : unsigned { T32 = 0, T24 = 1, TADR = 2, TSTAT = 3 };

/** read1Parameter with a reply length type 0 (32b), 1 (24b), 3 (status) or kNone */
bool rd(MotorSimplemotion &sm, unsigned id, uint32_t &value, int len = kNone) {
	return sm.read1Parameter(static_cast<ParamId>(id), &value, static_cast<LenType>(len == kNone ? 0xff : len));
}

bool wr(MotorSimplemotion &sm, unsigned id, int32_t v, uint32_t *reply = nullptr) {
	return sm.set1Parameter(static_cast<ParamId>(id), v, reply);
}

// ---- independent encoder of the SM wire format (HD) ----

/** One subpacket: type in the top 2 bits, 4 - type bytes, big endian (simplemotion_defs.h) */
std::vector<uint8_t> sub(unsigned type, uint32_t v) {
	const unsigned size = 4 - type;
	std::vector<uint8_t> b(size);
	for (unsigned i = 0; i < size; i++) {
		b[i] = static_cast<uint8_t>((v >> (8 * (size - 1 - i))) & 0xff);
	}
	b[0] = static_cast<uint8_t>((b[0] & 0x3f) | (type << 6));
	return b;
}

std::vector<uint8_t> cat(std::vector<uint8_t> a, const std::vector<uint8_t> &b) {
	a.insert(a.end(), b.begin(), b.end());
	return a;
}

/** CRC-16/ARC, reflected polynomial 0xA001, init 0, bitwise */
uint16_t arc(const std::vector<uint8_t> &d) {
	uint16_t crc = 0;
	for (uint8_t byte : d) {
		crc ^= byte;
		for (int i = 0; i < 8; i++) {
			crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
		}
	}
	return crc;
}

/** Instant command frame {0x24, len, adr, payload, crc low, crc high} (the library's byte-swapped value goes high byte first = ARC low first) */
std::vector<uint8_t> frame(uint8_t adr, const std::vector<uint8_t> &payload) {
	std::vector<uint8_t> f = {0x24, static_cast<uint8_t>(payload.size()), adr};
	f = cat(f, payload);
	const uint16_t crc = arc(f);
	f.push_back(static_cast<uint8_t>(crc & 0xff));
	f.push_back(static_cast<uint8_t>(crc >> 8));
	return f;
}

/** Request payload of a read: [set 10][24b len] only with a length, then [set 9][24b id] */
std::vector<uint8_t> readPayload(uint32_t id, int len = kNone) {
	std::vector<uint8_t> p;
	if (len != kNone) {
		p = cat(sub(TADR, 10), sub(T24, static_cast<uint32_t>(len)));
	}
	return cat(cat(p, sub(TADR, 9)), sub(T24, id));
}

std::vector<uint8_t> writePayload(uint32_t id, uint32_t v) { return cat(sub(TADR, id), sub(T32, v & 0x3fffffff)); }

std::string hex(const std::vector<uint8_t> &b) {
	static const char *d = "0123456789abcdef";
	std::string s;
	for (uint8_t x : b) {
		s += d[x >> 4];
		s += d[x & 15];
		s += ' ';
	}
	return s;
}

uint8_t adrOf(const FakeSmDevice::Frame &f) { return f.bytes.size() > 2 ? f.bytes[2] : 0; }

bool hasError(ErrorCode code) {
	for (const Error &e : ErrorHandler::getErrors()) {
		if (e.isError() && e.code == code) {
			return true;
		}
	}
	return false;
}

bool hasCriticalError(ErrorCode code) {
	for (const Error &e : ErrorHandler::getErrors()) {
		if (e.code == code && e.type == ErrorType::critical) {
			return true;
		}
	}
	return false;
}

/** UART peer that forwards to the fake but leaves the request with the index k (counted from installation) unanswered */
struct DropOneRequest : HostPlatform::UartPeer {
	FakeSmDevice &dev;
	unsigned k;
	unsigned n = 0;
	DropOneRequest(FakeSmDevice &d, unsigned index) : dev(d), k(index) {}
	void onTransmit(const void *huart, const uint8_t *data, uint16_t size) override {
		const bool drop = (n++ == k);
		dev.silent = drop;
		dev.onTransmit(huart, data, size);
		dev.silent = false;
	}
	void onReceiveArmed(const void *huart) override { dev.onReceiveArmed(huart); }
	void onPoll() override { dev.onPoll(); }
};

int64_t command(MotorSimplemotion &sm, const char *name) { return CommandHelpers::get(sm, name); }

/** The `reg` command carries CMDFLAG_DEBUG: it is only reachable while the debug mode is on. Restores the mode */
struct DebugModeOn {
	bool saved = SystemCommands::debugMode;
	DebugModeOn() { SystemCommands::debugMode = true; }
	~DebugModeOn() { SystemCommands::debugMode = saved; }
};

} // namespace

#define SM_READY(sm)                         \
	auto sm = makeSm(1);                     \
	REQUIRE(initialize(*sm));                \
	dev.clearFrames()

TEST_SUITE("SM protocol") {

// ---------------------------------------------------------------- R4.1 settings sequence

TEST_CASE_FIXTURE(SmFixture, "[R4.1 sequence] getSettings sends exactly six instant commands and ends initialized on a healthy device") {
	auto sm = makeSm(1);
	CHECK(initialize(*sm));
	CHECK(dev.frameCount() == 6);
	for (const auto &f : dev.frames()) {
		CHECK(f.kind == FakeSmDevice::Kind::Instant);
	}
	CHECK(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R4.1 sequence] the return address requests are devtype 6020, status 553, FBR 565, FBD 493, ControlMode 559 in this order") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	const std::vector<uint32_t> expected = {6020, 553, 565, 493, 559};
	CHECK(dev.returnAddressRequests() == expected);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.1 sequence] the decoded subpacket writes are the length / address pairs of the five reads and the faults write in order") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	// [10 = len][9 = id] per read: 32 bit return for devtype, 24 bit for the others; the faults write 552 = 0 is third
	const std::vector<std::pair<uint16_t, uint32_t>> expected = {{10, 0}, {9, 6020}, {552, 0}, {10, 1}, {9, 553}, {10, 1}, {9, 565},
																 {10, 1}, {9, 493}, {10, 1}, {9, 559}};
	CHECK(dev.writes() == expected);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.1 request] the first request reads devtype 6020 with a 32 bit return format") {
	// HD: 6020 = 0x1784. payload: [set 10: 80 0A][24b 0: 40 00 00][set 9: 80 09][24b 6020: 40 17 84]
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	const std::vector<uint8_t> payload = {0x80, 0x0A, 0x40, 0x00, 0x00, 0x80, 0x09, 0x40, 0x17, 0x84};
	CHECK(hex(dev.frames()[0].bytes) == hex(frame(adrOf(dev.frames()[0]), payload)));
}

TEST_CASE_FIXTURE(SmFixture, "[R4.1 request] the second request writes faults 552 = 0 as a 32 bit value") {
	// HD: 552 = 0x0228: [set 552: 80|02 = 82, 28][32b 0: 00 00 00 00]
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	const std::vector<uint8_t> payload = {0x82, 0x28, 0x00, 0x00, 0x00, 0x00};
	CHECK(hex(dev.frames()[1].bytes) == hex(frame(adrOf(dev.frames()[1]), payload)));
}

TEST_CASE_FIXTURE(SmFixture, "[R4.1 request] the third request reads status 553 with a 24 bit return format") {
	// HD: 553 = 0x0229: [80 0A][40 00 01][80 09][40 02 29]
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	const std::vector<uint8_t> payload = {0x80, 0x0A, 0x40, 0x00, 0x01, 0x80, 0x09, 0x40, 0x02, 0x29};
	CHECK(hex(dev.frames()[2].bytes) == hex(frame(adrOf(dev.frames()[2]), payload)));
}

TEST_CASE_FIXTURE(SmFixture, "[R4.1 request] the FBR, FBD and ControlMode requests read 565, 493 and 559 with a 24 bit return format") {
	// HD: 565 = 0x0235, 493 = 0x01ED, 559 = 0x022F
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	const uint32_t ids[3] = {565, 493, 559};
	for (int i = 0; i < 3; i++) {
		const auto &f = dev.frames()[3 + i];
		const std::vector<uint8_t> payload = {0x80, 0x0A, 0x40, 0x00, 0x01, 0x80, 0x09, 0x40, static_cast<uint8_t>(ids[i] >> 8),
											  static_cast<uint8_t>(ids[i] & 0xff)};
		CHECK(hex(f.bytes) == hex(frame(adrOf(f), payload)));
	}
}

namespace {
void settingsFailsWhenRequestIsDropped(SmFixture &fx, unsigned k) {
	auto sm = fx.makeSm(1);
	DropOneRequest peer(fx.dev, k);
	HostPlatform::setUartPeer(&peer);
	CHECK_FALSE(fx.initialize(*sm));
	CHECK_FALSE(sm->motorReady());
	HostPlatform::setUartPeer(&fx.dev);
}
} // namespace

// The status read (request 2) is not dropped: its failure reads the uninitialised `st` (B-SM-4)
TEST_CASE_FIXTURE(SmFixture, "[R4.1 sequence] getSettings fails when only the devtype request (1st) is unanswered") {
	settingsFailsWhenRequestIsDropped(*this, 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.1 sequence] getSettings fails when only the FBR request (4th) is unanswered") {
	settingsFailsWhenRequestIsDropped(*this, 3);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.1 sequence] getSettings fails when only the FBD request (5th) is unanswered") {
	settingsFailsWhenRequestIsDropped(*this, 4);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.1 sequence] getSettings fails when only the ControlMode request (6th) is unanswered") {
	settingsFailsWhenRequestIsDropped(*this, 5);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.1 sequence] the faults register is cleared on the device by getSettings") {
	dev.params[FakeSmDevice::SMP_FAULTS] = 7;
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	CHECK(dev.params[FakeSmDevice::SMP_FAULTS] == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.1 sequence] all six requests of one settings run carry the same device address") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	for (const auto &f : dev.frames()) {
		CHECK(adrOf(f) == adrOf(dev.frames()[0]));
	}
}

// ---------------------------------------------------------------- R4.2 cpr

namespace {
uint32_t cprFor(SmFixture &fx, uint32_t fbd, uint32_t fbr) {
	fx.dev.params[FakeSmDevice::SMP_FB1_DEVICE_SELECTION] = fbd;
	fx.dev.params[FakeSmDevice::SMP_ENCODER_PPR] = fbr;
	auto sm = fx.makeSm(1);
	REQUIRE(fx.initialize(*sm));
	return sm->getCpr();
}
} // namespace

TEST_CASE_FIXTURE(SmFixture, "[R4.2 cpr] ABN1 (FBD 1) counts four quadrature edges per pulse: 1000 pulses give cpr 4000") {
	CHECK(cprFor(*this, 1, 1000) == 4000);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.2 cpr] ABN2 (FBD 2) counts four quadrature edges per pulse: 1000 pulses give cpr 4000") {
	CHECK(cprFor(*this, 2, 1000) == 4000);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.2 cpr] sincos 16 (FBD 6) multiplies by 16: 1000 give 16000") {
	CHECK(cprFor(*this, 6, 1000) == 16000);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.2 cpr] sincos 64 (FBD 7) multiplies by 64: 1000 give 64000") {
	CHECK(cprFor(*this, 7, 1000) == 64000);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.2 cpr] sincos 256 (FBD 8) multiplies by 256: 1000 give 256000") {
	CHECK(cprFor(*this, 8, 1000) == 256000);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.2 cpr] characterization: resolver (FBD 3) uses the factor 4") {
	// NEEDS REVIEW: the code comment says "TODO check if this is right for hall, resolver and sincos"; the SM documents give no rule
	CHECK(cprFor(*this, 3, 1000) == 4000);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.2 cpr] characterization: hall (FBD 4) uses the factor 4") {
	// NEEDS REVIEW: see resolver
	CHECK(cprFor(*this, 4, 1000) == 4000);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.2 cpr] characterization: serial (FBD 5) uses the factor 4") {
	// NEEDS REVIEW: see resolver
	CHECK(cprFor(*this, 5, 1000) == 4000);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.2 cpr] getCpr on a driver that never ran the settings runs them and returns the cpr") {
	auto sm = makeSm(1);
	advanceMs(150); // getSettings refuses to run earlier
	CHECK(sm->getCpr() == 4000);
	CHECK(dev.frameCount() == 6);
	CHECK(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R4.2 cpr] getCpr on a driver with a cpr but not initialized runs the six settings requests again") {
	dev.params[FakeSmDevice::SMP_CONTROL_MODE] = 2; // the settings fail at the last read, but the cpr is already 4000
	auto sm = makeSm(1);
	CHECK_FALSE(initialize(*sm));
	CHECK_FALSE(sm->motorReady());
	dev.params[FakeSmDevice::SMP_CONTROL_MODE] = 3;
	advanceMs(150);
	dev.clearFrames();
	CHECK(sm->getCpr() == 4000);
	CHECK(dev.frameCount() == 6);
	CHECK(sm->motorReady());
}

// ---------------------------------------------------------------- R4.3 unsupported feedback

TEST_CASE_FIXTURE(SmFixture, "[R4.3 fbd] FBD 0 (no feedback) adds the configuration error and the driver is not ready") {
	dev.params[FakeSmDevice::SMP_FB1_DEVICE_SELECTION] = 0;
	auto sm = makeSm(1);
	CHECK_FALSE(hasError(ErrorCode::externalConfigurationError));
	CHECK_FALSE(initialize(*sm));
	CHECK(hasCriticalError(ErrorCode::externalConfigurationError));
	CHECK_FALSE(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R4.3 fbd] FBD 9 (not in the supported list) adds the configuration error and the driver is not ready") {
	dev.params[FakeSmDevice::SMP_FB1_DEVICE_SELECTION] = 9;
	auto sm = makeSm(1);
	CHECK_FALSE(initialize(*sm));
	CHECK(hasCriticalError(ErrorCode::externalConfigurationError));
	CHECK_FALSE(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R4.3 fbd] characterization: FBD 10 (SMP_FBD_SERIALDATA_PORT2 in the SM defs) is rejected as invalid configuration") {
	// NEEDS REVIEW: FBD 10 is a defined SM feedback type; the firmware has no cpr rule for it and rejects it
	dev.params[FakeSmDevice::SMP_FB1_DEVICE_SELECTION] = 10;
	auto sm = makeSm(1);
	CHECK_FALSE(initialize(*sm));
	CHECK(hasCriticalError(ErrorCode::externalConfigurationError));
	CHECK_FALSE(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R4.3 fbd] a supported feedback adds no configuration error") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	CHECK_FALSE(hasError(ErrorCode::externalConfigurationError));
}

TEST_CASE_FIXTURE(SmFixture, "[R4.3 fbd] characterization: after a rejected feedback the hardfault stays set when the device is repaired, so the driver is not ready until restart") {
	// NEEDS REVIEW: getSettings sets hardfault for FBD 0 but never clears it; only restart() or a status word reply clears it
	dev.params[FakeSmDevice::SMP_FB1_DEVICE_SELECTION] = 0;
	auto sm = makeSm(1);
	CHECK_FALSE(initialize(*sm));
	dev.params[FakeSmDevice::SMP_FB1_DEVICE_SELECTION] = 1;
	CHECK(initialize(*sm)); // the settings now succeed ...
	CHECK_FALSE(sm->motorReady()); // ... but initialized && !hardfault stays false
	sm->restart();
	CHECK(sm->motorReady());
}

// ---------------------------------------------------------------- R4.4 other rejections

TEST_CASE_FIXTURE(SmFixture, "[R4.4 mode] a control mode other than 3 (torque) leaves the driver uninitialized") {
	dev.params[FakeSmDevice::SMP_CONTROL_MODE] = 2;
	auto sm = makeSm(1);
	CHECK_FALSE(initialize(*sm));
	CHECK_FALSE(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R4.4 status] a status word of 0 ends getSettings after the status read: three requests, not initialized") {
	dev.params[FakeSmDevice::SMP_STATUS] = 0;
	auto sm = makeSm(1);
	CHECK_FALSE(initialize(*sm));
	CHECK(dev.frameCount() == 3); // devtype, faults write, status
	CHECK_FALSE(sm->motorReady());
}

// ---------------------------------------------------------------- R4.5 interval

TEST_CASE_FIXTURE(SmFixture, "[R4.5 interval] characterization: a second getSettings straight after the first sends nothing and returns false, after 150 ms it runs again") {
	// NEEDS REVIEW: the 150 ms constant is not in the SM documents
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	const size_t sent = dev.frameCount();
	CHECK_FALSE(sm->getSettings());
	CHECK(dev.frameCount() == sent);
	advanceMs(150);
	CHECK(sm->getSettings());
	CHECK(dev.frameCount() == sent + 6);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.5 interval] characterization: 149 ms after the last request the settings are refused, 150 ms accepted") {
	// NEEDS REVIEW: boundary of the 150 ms constant; simulated time does not move while the peer answers inside the call
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	const size_t sent = dev.frameCount();
	advanceMs(149);
	CHECK_FALSE(sm->getSettings());
	CHECK(dev.frameCount() == sent);
	advanceMs(1);
	CHECK(sm->getSettings());
	CHECK(dev.frameCount() == sent + 6);
}

// ---------------------------------------------------------------- R4.6 silent peer

TEST_CASE_FIXTURE(SmFixture, "[R4.6 silent] with a silent peer getSettings fails, the driver is not ready and uarterr counts at least the three timeouts") {
	// the status read is the third request; with its failure `st` is read uninitialised (B-SM-4): STACK CONTENT decides whether
	// the remaining three requests are still sent (3 or 6 timeouts), so only the bound >= 3 is asserted
	dev.silent = true;
	auto sm = makeSm(1);
	CHECK_FALSE(initialize(*sm));
	CHECK_FALSE(sm->motorReady());
	CHECK(command(*sm, "uarterr") >= 3);
}

TEST_CASE_FIXTURE(SmFixture, "[R4.6 silent] a silent read returns false after 10 ms of simulated time and counts exactly one uart error") {
	SM_READY(sm);
	dev.silent = true;
	const uint64_t before = HostPlatform::nowUs();
	uint32_t v = 0;
	CHECK_FALSE(rd(*sm, 900, v));
	CHECK(HostPlatform::nowUs() - before == 10000); // silent peer: 1 ms of simulated time per idle poll, loop ends at 10 ms
	CHECK(command(*sm, "uarterr") == 1);
}

// ---------------------------------------------------------------- R5.1 request layouts

TEST_CASE_FIXTURE(SmFixture, "[R5.1 request] read1Parameter(553) without a length sends {0x24, 5, adr, 80 09, 40 02 29, crc16}") {
	// HD: 553 = 0x0229: [set 9: 80 09][24b 553: 40 02 29]; CRC-16/ARC over 24 05 adr + payload, low byte first on the wire
	SM_READY(sm);
	uint32_t v = 0;
	REQUIRE(rd(*sm, 553, v));
	REQUIRE(dev.frameCount() == 1);
	const auto &b = dev.frames()[0].bytes;
	REQUIRE(b.size() == 10);
	CHECK(b[0] == 0x24);
	CHECK(b[1] == 5);
	const std::vector<uint8_t> payload = {0x80, 0x09, 0x40, 0x02, 0x29};
	CHECK(hex(b) == hex(frame(adrOf(dev.frames()[0]), payload)));
}

TEST_CASE_FIXTURE(SmFixture, "[R5.1 request] read1Parameter(553) with a 24 bit length carries 10 payload bytes starting with 80 0A 40 00 01") {
	SM_READY(sm);
	uint32_t v = 0;
	REQUIRE(rd(*sm, 553, v, T24));
	REQUIRE(dev.frameCount() == 1);
	const auto &b = dev.frames()[0].bytes;
	REQUIRE(b.size() == 15);
	CHECK(b[1] == 10);
	const std::vector<uint8_t> payload = {0x80, 0x0A, 0x40, 0x00, 0x01, 0x80, 0x09, 0x40, 0x02, 0x29};
	CHECK(hex(b) == hex(frame(adrOf(dev.frames()[0]), payload)));
}

TEST_CASE_FIXTURE(SmFixture, "[R5.1 request] read1Parameter with a 32 bit length requests the length value 0") {
	SM_READY(sm);
	uint32_t v = 0;
	REQUIRE(rd(*sm, 901, v, T32));
	// HD: 901 = 0x0385: [80 0A][40 00 00][80 09][40 03 85]
	const std::vector<uint8_t> payload = {0x80, 0x0A, 0x40, 0x00, 0x00, 0x80, 0x09, 0x40, 0x03, 0x85};
	REQUIRE(dev.frameCount() == 1);
	CHECK(hex(dev.frames()[0].bytes) == hex(frame(adrOf(dev.frames()[0]), payload)));
}

TEST_CASE_FIXTURE(SmFixture, "[R5.1 request] set1Parameter(554, 2) sends [set 554][32b 2]: 6 payload bytes, 11 bytes in total") {
	// HD: 554 = 0x022A: [80|02 = 82, 2A][00 00 00 02]
	SM_READY(sm);
	REQUIRE(wr(*sm, 554, 2));
	REQUIRE(dev.frameCount() == 1);
	const auto &b = dev.frames()[0].bytes;
	REQUIRE(b.size() == 11);
	const std::vector<uint8_t> payload = {0x82, 0x2A, 0x00, 0x00, 0x00, 0x02};
	CHECK(hex(b) == hex(frame(adrOf(dev.frames()[0]), payload)));
}

TEST_CASE_FIXTURE(SmFixture, "[R5.1 request] set1Parameter masks a negative value to 30 bits: -1 is sent as 3F FF FF FF") {
	// HD: -1 & 0x3FFFFFFF = 0x3FFFFFFF; the type bits 00 of the 32 bit write stay in the top two bits of the first byte
	SM_READY(sm);
	REQUIRE(wr(*sm, 552, -1));
	const std::vector<uint8_t> payload = {0x82, 0x28, 0x3F, 0xFF, 0xFF, 0xFF};
	REQUIRE(dev.frameCount() == 1);
	CHECK(hex(dev.frames()[0].bytes) == hex(frame(adrOf(dev.frames()[0]), payload)));
}

TEST_CASE_FIXTURE(SmFixture, "[R5.1 queue] queueCommand writes 4 / 3 / 2 bytes for a 32 bit / 24 bit / address subpacket and returns the length") {
	SM_READY(sm);
	uint8_t buf[8] = {0};
	CHECK(sm->queueCommand(buf, static_cast<CmdType>(T32), 0x01020304) == 4);
	CHECK(hex({buf, buf + 4}) == hex(sub(T32, 0x01020304)));
	uint8_t buf2[8] = {0};
	CHECK(sm->queueCommand(buf2, static_cast<CmdType>(T24), 0x000A0B) == 3);
	CHECK(hex({buf2, buf2 + 3}) == hex(sub(T24, 0x000A0B)));
	uint8_t buf3[8] = {0};
	CHECK(sm->queueCommand(buf3, static_cast<CmdType>(TADR), 0x0229) == 2);
	CHECK(hex({buf3, buf3 + 2}) == hex(sub(TADR, 0x0229)));
}

TEST_CASE_FIXTURE(SmFixture, "[R5.1 queue] queueCommand of a status subpacket is one byte with the type bits 11") {
	SM_READY(sm);
	uint8_t buf[2] = {0};
	CHECK(sm->queueCommand(buf, static_cast<CmdType>(TSTAT), 0x05) == 1);
	CHECK(buf[0] == 0xC5);
	CHECK(buf[1] == 0);
}

// ---------------------------------------------------------------- R5.2 reply decoding

TEST_CASE_FIXTURE(SmFixture, "[R5.2 decode] a 32 bit reply (type 00, 4 bytes) returns the 30 bit value") {
	SM_READY(sm);
	dev.retLen = 0;
	dev.params[900] = 0x12345678; // fits 30 bits
	uint32_t v = 0;
	REQUIRE(rd(*sm, 900, v)); // no length: the device's current format
	CHECK(v == 0x12345678);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.2 decode] a 24 bit reply (type 01, 3 bytes) returns the 22 bit value") {
	SM_READY(sm);
	dev.retLen = 1;
	dev.params[900] = 0x2ABCDE; // fits 22 bits
	uint32_t v = 0;
	REQUIRE(rd(*sm, 900, v));
	CHECK(v == 0x2ABCDE);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.2 decode] a 16 bit reply (type 10, 2 bytes) returns the 14 bit value") {
	SM_READY(sm);
	dev.retLen = 2;
	dev.params[900] = 0x1234; // fits 14 bits
	uint32_t v = 0;
	REQUIRE(rd(*sm, 900, v));
	CHECK(v == 0x1234);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.2 decode] the value 0 of the ACK status byte (type 11, 1 byte each) reaches the caller; a non-ACK status byte cannot be produced by the fake") {
	// the fake answers each subpacket with the ACK byte 0xC0 (value 0): two 1 byte subpackets, the second one is the value
	SM_READY(sm);
	dev.retLen = 3;
	uint32_t v = 0xFFFFFFFF;
	REQUIRE(rd(*sm, 900, v));
	CHECK(v == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.2 decode] a read with a length takes the fourth reply subpacket and switches the device to that format") {
	SM_READY(sm);
	dev.retLen = 0; // the request must switch to 24 bit before the value is returned
	dev.params[901] = 0x2ABCDE;
	uint32_t v = 0;
	REQUIRE(rd(*sm, 901, v, T24));
	CHECK(v == 0x2ABCDE);
	CHECK(dev.retLen == 1);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.2 decode] the reply of a write goes to the optional pointer and the parameter is stored on the device") {
	// after a read of 900 in 32 bit format the device answers every subpacket with parameter 900; the write reply is the second subpacket
	SM_READY(sm);
	dev.params[900] = 4321;
	uint32_t x = 0;
	REQUIRE(rd(*sm, 900, x, T32));
	uint32_t reply = 0;
	REQUIRE(wr(*sm, 2533, 1, &reply));
	CHECK(reply == 4321);
	CHECK(dev.params[2533] == 1);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.2 decode] a reply longer than the request but within the 32 byte buffer is parsed (five extra subpackets)") {
	// HD: reply = 3 header + 2 * 3 (24 bit, retLen 1) + 5 * 4 padding + 2 crc = 31 bytes
	SM_READY(sm);
	dev.params[900] = 77;
	dev.padSubpackets = 5;
	uint32_t v = 0;
	CHECK(rd(*sm, 900, v));
	CHECK(v == 77);
}

// ---------------------------------------------------------------- R5.3 receive errors

TEST_CASE_FIXTURE(SmFixture, "[R5.3 crc] a reply with a wrong CRC-16 is counted by crcerr and the value of the reply is not taken" ) {
	SM_READY(sm);
	dev.params[900] = 1234;
	dev.corruptCrc = true;
	uint32_t v = 0xDEADBEEF;
	rd(*sm, 900, v); // the return value is the subject of the KNOWN_ISSUE test below
	CHECK(command(*sm, "crcerr") == 1);
	CHECK(v != 1234);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.3 crc] a reply with a wrong CRC-16 makes the read return false" *
								 KNOWN_ISSUE("B-SM-5: resetBuffer() in the CRC error branch clears waitingReply, the busy-wait ends, readParameter copies the stale replyvalues and returns true (measured true, intended false)")) {
	SM_READY(sm);
	dev.corruptCrc = true;
	uint32_t v = 0;
	CHECK_FALSE(rd(*sm, 900, v));
}

TEST_CASE_FIXTURE(SmFixture, "[R5.3 crc] a reply with a wrong CRC-16 makes the write return false" *
								 KNOWN_ISSUE("B-SM-5: same path in writeParameter (measured true, intended false)")) {
	SM_READY(sm);
	dev.corruptCrc = true;
	CHECK_FALSE(wr(*sm, 2533, 1));
}

TEST_CASE_FIXTURE(SmFixture, "[R5.3 address] a reply from another device address is not accepted as a valid reply (no crc error, no value taken)") {
	SM_READY(sm);
	uint32_t v0 = 0;
	REQUIRE(rd(*sm, 900, v0)); // learn the driver's address from the wire
	dev.replyAddress = adrOf(dev.frames().back()) ^ 0x01;
	dev.params[900] = 1234;
	uint32_t v = 0xDEADBEEF;
	rd(*sm, 900, v); // return value: KNOWN_ISSUE test below
	CHECK(v != 1234);
	CHECK(command(*sm, "crcerr") == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.3 address] a read answered by another device address returns false" *
								 KNOWN_ISSUE("B-SM-5: the address mismatch branch calls resetBuffer(), which clears waitingReply, so the read returns true with stale values (measured true, intended false)")) {
	SM_READY(sm);
	uint32_t v0 = 0;
	REQUIRE(rd(*sm, 900, v0));
	dev.replyAddress = adrOf(dev.frames().back()) ^ 0x01;
	uint32_t v = 0;
	CHECK_FALSE(rd(*sm, 900, v));
}

TEST_CASE_FIXTURE(SmFixture, "[R5.3 address] a write answered by another device address returns false" *
								 KNOWN_ISSUE("B-SM-5: same path in writeParameter (measured true, intended false)")) {
	SM_READY(sm);
	uint32_t v0 = 0;
	REQUIRE(rd(*sm, 900, v0));
	dev.replyAddress = adrOf(dev.frames().back()) ^ 0x01;
	CHECK_FALSE(wr(*sm, 2533, 1));
}

TEST_CASE_FIXTURE(SmFixture, "[R5.3 split] a reply delivered one byte at a time is assembled") {
	SM_READY(sm);
	dev.params[900] = 4242;
	dev.chunk = 1;
	uint32_t v = 0;
	REQUIRE(rd(*sm, 900, v));
	CHECK(v == 4242);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.3 split] a reply in chunks of 3 bytes with 2 ms gaps (6 ms in total, below the 10 ms timeout) is assembled") {
	// HD: reply = 3 + 6 + 2 = 11 bytes = 4 chunks, 3 gaps of 2 ms = 6 ms
	SM_READY(sm);
	dev.params[900] = 4242;
	dev.chunk = 3;
	dev.gapMs = 2;
	uint32_t v = 0;
	REQUIRE(rd(*sm, 900, v));
	CHECK(v == 4242);
	CHECK(command(*sm, "uarterr") == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.3 split] a reply whose gaps add up to more than the 10 ms timeout does not complete: the read returns false and uarterr counts one") {
	// HD: 11 bytes in chunks of 3 = 4 chunks; the gap of 6 ms is applied before each chunk after the first: chunk 2 at 6 ms, chunk 3 at
	// 12 ms, the loop then sees 12 ms >= 10 ms with chunk 4 undelivered. (With a 4 ms gap the fake would deliver chunk 4 at 12 ms inside
	// the same poll that the loop started at 8 ms: a fake limitation, so 6 ms is used.)
	SM_READY(sm);
	dev.chunk = 3;
	dev.gapMs = 6;
	uint32_t v = 0;
	CHECK_FALSE(rd(*sm, 900, v));
	CHECK(command(*sm, "uarterr") == 1);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.3 short] a reply one byte shorter than announced times out: the read returns false and uarterr counts it") {
	SM_READY(sm);
	dev.truncateReply = 1;
	uint32_t v = 0;
	CHECK_FALSE(rd(*sm, 900, v));
	CHECK(command(*sm, "uarterr") == 1);
}

// ---------------------------------------------------------------- R5.4 overrun

TEST_CASE_FIXTURE(SmFixture, "[R5.4 overrun] a reply announcing more than 27 payload bytes does not complete: the read returns false and uart errors are counted") {
	// HD: base reply 3 + 6 + 2 = 11 bytes; 6 padding subpackets of 4 bytes give 35 bytes > 32 (RXBUF_SIZE): the receive buffer
	// is full after 32 bytes, further bytes are an overrun (sanitizer clean: no write beyond rxbuf)
	SM_READY(sm);
	dev.padSubpackets = 6;
	uint32_t v = 0;
	CHECK_FALSE(rd(*sm, 900, v));
	// HD: byte 33 finds the buffer full: uarterrors = 1, the receive is not re-armed, the 2 remaining bytes of the 35 are not accepted;
	// the call then times out: uarterrors = 2
	CHECK(command(*sm, "uarterr") == 2);
	CHECK(dev.bytesNotAccepted() == 2);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.4 overrun] after an overrun the next normal request is answered") {
	SM_READY(sm);
	dev.padSubpackets = 6;
	uint32_t v = 0;
	CHECK_FALSE(rd(*sm, 900, v));
	dev.padSubpackets = 0;
	advanceMs(20); // let the uart error timeout pass (10 ms)
	dev.params[900] = 55;
	CHECK(rd(*sm, 900, v));
	CHECK(v == 55);
}

// ---------------------------------------------------------------- R5.5 uart error flag

TEST_CASE_FIXTURE(SmFixture, "[R5.5 uart] a uart error flag on a reply byte drops the byte: the read returns false and uarterr counts the error and the timeout") {
	// HD: the flagged byte is counted once (uarterrors++), the firmware does not re-arm the receive, the call times out and counts once more: 2
	SM_READY(sm);
	dev.errorOnByte = 3;
	uint32_t v = 0;
	CHECK_FALSE(rd(*sm, 900, v));
	CHECK(command(*sm, "uarterr") == 2);
}

// ---------------------------------------------------------------- R5.6 recovery

TEST_CASE_FIXTURE(SmFixture, "[R5.6 recover] after a timed-out read the next request is sent and answered") {
	SM_READY(sm);
	dev.silent = true;
	uint32_t v = 0;
	CHECK_FALSE(rd(*sm, 900, v));
	dev.silent = false;
	advanceMs(1); // the port is released more than 10 ms after the last received byte
	dev.params[900] = 99;
	CHECK(rd(*sm, 900, v));
	CHECK(v == 99);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.6 recover] characterization: a request sent in the same millisecond as the timeout of the previous one is refused, the port is released 10 ms after the last received byte") {
	// NEEDS REVIEW: prepareUartTransmit resets the buffer only when more than 10 ms passed since the last byte; at exactly 10 ms the
	// port is still taken. On the target the next request comes later; the SM documents have no such rule
	SM_READY(sm);
	dev.silent = true;
	uint32_t v = 0;
	CHECK_FALSE(rd(*sm, 900, v));
	dev.silent = false;
	CHECK_FALSE(rd(*sm, 900, v));
	advanceMs(1);
	CHECK(rd(*sm, 900, v));
}

TEST_CASE_FIXTURE(SmFixture, "[R5.6 recover] after a timed-out write the next request is sent and answered") {
	SM_READY(sm);
	dev.silent = true;
	CHECK_FALSE(wr(*sm, 2533, 1));
	CHECK(command(*sm, "uarterr") == 1);
	dev.silent = false;
	advanceMs(1); // see above
	CHECK(wr(*sm, 2533, 0));
	CHECK(dev.params[2533] == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.6 recover] after a reply with a wrong CRC-16 the next request is answered") {
	SM_READY(sm);
	dev.corruptCrc = true;
	uint32_t v = 0;
	rd(*sm, 900, v);
	dev.corruptCrc = false;
	dev.params[900] = 99;
	CHECK(rd(*sm, 900, v));
	CHECK(v == 99);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.6 recover] after a reply with a wrong address the next request is answered") {
	SM_READY(sm);
	uint32_t v = 0;
	REQUIRE(rd(*sm, 900, v));
	dev.replyAddress = adrOf(dev.frames().back()) ^ 0x01;
	rd(*sm, 900, v);
	dev.replyAddress = -1;
	dev.params[900] = 99;
	CHECK(rd(*sm, 900, v));
	CHECK(v == 99);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.6 recover] after a truncated reply the next request is answered") {
	SM_READY(sm);
	dev.truncateReply = 1;
	uint32_t v = 0;
	CHECK_FALSE(rd(*sm, 900, v));
	dev.truncateReply = 0;
	advanceMs(1); // see above
	dev.params[900] = 99;
	CHECK(rd(*sm, 900, v));
	CHECK(v == 99);
}

// ---------------------------------------------------------------- R5.7 / R5.8 limits

TEST_CASE_FIXTURE(SmFixture, "[R5.7 negative] characterization: a 22 bit -1 (0x3FFFFF) in a 24 bit reply is read as 0x3FFFFF, not sign extended") {
	// NEEDS REVIEW (owner question 38d): the sign extension is commented out in uartRcv; the SM documents give no sign rule
	SM_READY(sm);
	dev.retLen = 1;
	dev.params[900] = 0x3FFFFF;
	uint32_t v = 0;
	REQUIRE(rd(*sm, 900, v));
	CHECK(v == 0x3FFFFF);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.7 negative] characterization: a 14 bit -1 (0x3FFF) in a 16 bit reply is read as 0x3FFF, not sign extended") {
	// NEEDS REVIEW: see above
	SM_READY(sm);
	dev.retLen = 2;
	dev.params[900] = 0x3FFF;
	uint32_t v = 0;
	REQUIRE(rd(*sm, 900, v));
	CHECK(v == 0x3FFF);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.8 reg] characterization: reg setat with an address above 0x3FFF puts the high address bits into the subpacket type bits") {
	// HD: 0x4001: queueCommand ORs (0x4001 >> 8) = 0x40 into the first byte 0x80: 0xC0 = type 11, then 0x01. NEEDS REVIEW: spec has 14 address bits
	DebugModeOn debugModeOn;
	SM_READY(sm);
	dev.expectViolation(); // the device sees an outbound type 3 subpacket
	std::vector<CommandReply> replies;
	CHECK(CommandHelpers::run(*sm, "reg", CMDtype::setat, 1, &replies, 0x4001) == CommandStatus::OK);
	REQUIRE(dev.frameCount() == 1);
	const auto &b = dev.frames()[0].bytes;
	REQUIRE(b.size() > 4);
	CHECK(b[3] == 0xC0);
	CHECK(b[4] == 0x01);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 0); // no reply arrives
}

TEST_CASE_FIXTURE(SmFixture, "[R5.8 reg] reg setat with the largest 14 bit address 0x3FFF sends the address unchanged") {
	// HD: 0x3FFF: [80|3F = BF, FF]
	DebugModeOn debugModeOn;
	SM_READY(sm);
	std::vector<CommandReply> replies;
	CommandHelpers::run(*sm, "reg", CMDtype::setat, 1, &replies, 0x3FFF);
	REQUIRE(dev.frameCount() == 1);
	CHECK(dev.frames()[0].bytes[3] == 0xBF);
	CHECK(dev.frames()[0].bytes[4] == 0xFF);
}

// ---------------------------------------------------------------- R10 commands

TEST_CASE_FIXTURE(SmFixture, "[R10.1 reg] reg getat returns the value of the addressed parameter") {
	DebugModeOn debugModeOn;
	SM_READY(sm);
	dev.params[901] = 123;
	CHECK(CommandHelpers::getAt(*sm, "reg", 901) == 123);
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 reg] reg getat sends a read request of that parameter") {
	DebugModeOn debugModeOn;
	SM_READY(sm);
	CommandHelpers::getAt(*sm, "reg", 901);
	REQUIRE(dev.frameCount() == 1);
	CHECK(hex(dev.frames()[0].bytes) == hex(frame(adrOf(dev.frames()[0]), readPayload(901))));
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 reg] reg getat of a silent device returns 0") {
	DebugModeOn debugModeOn;
	SM_READY(sm);
	dev.params[901] = 123;
	dev.silent = true;
	CHECK(CommandHelpers::getAt(*sm, "reg", 901) == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 reg] reg setat writes the value to the device") {
	DebugModeOn debugModeOn;
	SM_READY(sm);
	std::vector<CommandReply> replies;
	CHECK(CommandHelpers::run(*sm, "reg", CMDtype::setat, 77, &replies, 2533) == CommandStatus::OK);
	CHECK(dev.params[2533] == 77);
	REQUIRE(dev.frameCount() == 1);
	CHECK(hex(dev.frames()[0].bytes) == hex(frame(adrOf(dev.frames()[0]), writePayload(2533, 77))));
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 reg] reg setat returns the reply of the write: the parameter at the return address") {
	// HD: after the settings the return address is 559 (ControlMode = 3) in the 24 bit format: the second reply subpacket holds 3
	DebugModeOn debugModeOn;
	SM_READY(sm);
	std::vector<CommandReply> replies;
	CommandHelpers::run(*sm, "reg", CMDtype::setat, 77, &replies, 2533);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 3);
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 devtype] devtype is 0 before the settings and the value of the device type parameter (0x1234 here) afterwards") {
	dev.params[FakeSmDevice::SMP_DEVICE_TYPE] = 0x1234;
	auto sm = makeSm(1);
	CHECK(command(*sm, "devtype") == 0);
	REQUIRE(initialize(*sm));
	CHECK(command(*sm, "devtype") == 0x1234);
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 crcerr] crcerr and uarterr are 0 on a driver that had no error") {
	SM_READY(sm);
	CHECK(command(*sm, "crcerr") == 0);
	CHECK(command(*sm, "uarterr") == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 crcerr] crcerr counts every reply with a wrong CRC-16: two bad replies give 2") {
	SM_READY(sm);
	dev.corruptCrc = true;
	uint32_t v = 0;
	rd(*sm, 900, v);
	rd(*sm, 900, v);
	CHECK(command(*sm, "crcerr") == 2);
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 uarterr] uarterr counts every timed-out request: two silent reads give 2") {
	SM_READY(sm);
	dev.silent = true;
	uint32_t v = 0;
	rd(*sm, 900, v);
	advanceMs(20);
	rd(*sm, 900, v);
	CHECK(command(*sm, "uarterr") == 2);
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 reg] reg is not reachable while the debug mode is off") {
	SM_READY(sm);
	const bool saved = SystemCommands::debugMode;
	SystemCommands::debugMode = false;
	CHECK(sm->getCommandFromName("reg") == nullptr);
	CHECK(sm->getCommandFromName("crcerr") != nullptr);
	SystemCommands::debugMode = saved;
}

TEST_CASE_FIXTURE(SmFixture, "[R10.1 unknown] a command id the driver does not know is NOT_FOUND") {
	SM_READY(sm);
	ParsedCommand parsed;
	parsed.cmdId = 99;
	parsed.type = CMDtype::get;
	std::vector<CommandReply> replies;
	CHECK(sm->command(parsed, replies) == CommandStatus::NOT_FOUND);
	CHECK(replies.empty());
}

}
