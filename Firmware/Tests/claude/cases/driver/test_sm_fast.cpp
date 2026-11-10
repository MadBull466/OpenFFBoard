/*
 * SimpleMotion driver, phase P15 T2, work order SMF: fast update, position tracking and status.
 * Requirements R3.1-R3.7, R6.1-R6.4 and R7.1-R7.3 of Tests/claude/charters/driver/SIMPLEMOTION_REQUIREMENTS.md.
 *
 * Unit under test: MotorSimplemotion::turn / sendFastUpdate / getPos / setPos / updatePosition / updateStatus / uartRcv
 * (fast reply branch) / motorReady and the commands "state" and "crcerr". Everything is driven through the public interface and
 * observed at the fake SimpleMotion device (frames, setpoints), the write-enable pin and the commands.
 *
 * Oracles: SM = SimpleMotion V2 library (fast update frame {0x10, addr, write1, write2, crc8}, CRC-8 poly 0x07 init 0x52,
 * reply {0x11, read1 = low 16 bits of the position, read2 = low 16 bits of SMP_STATUS, crc8}, STAT_FAULTSTOP bit 5,
 * STAT_INITIALIZED bit 12); HD = hand derivation in a comment next to the assertion, the CRC-8 oracle is the bitwise function
 * of this file (checked against the library value 0x46 for {10 01 00 00 00 00}), not the firmware's table; FW =
 * `characterization:` (NEEDS REVIEW).
 *
 * The driver's own address is indeterminate (B-SM-1): no test asserts its value, the tests read it from the frame.
 *
 * Position model (hand derivation from updatePosition, used by the expected values below): the report r is shifted by 0x7fff
 * (16 bit), so a report r < 0x8000 maps to position r and a report r >= 0x8000 maps to r - 0x10000 (signed view) as long as no
 * overflow is counted; the overflow is counted at the step between the reports 0x7fff and 0x8000 (shifted 0xfffe -> 0xffff
 * to 0x0000...), which is where one count is lost (B-SM-2). The wrap of the 16 bit counter at 0xffff -> 0x0000 itself is
 * continuous in the signed view and is exact. Tests that compare positions use differences, never the absolute value of a
 * report >= 0x8000 (see "Lead decisions needed" of the report: R6.1 for the report 0x8000).
 *
 * Host versus target: none of the arithmetic depends on the host (uint16_t / int32_t only, the fast frame words are little
 * endian on both); the fast frame is the packed struct Sm2FastUpdate (7 bytes).
 *
 * Characterizations (NEEDS REVIEW): R3.2 the pin stays high without a transmit-complete interrupt; R3.5 the 10 ms wait
 * constant and the behaviour inside it; R3.7 the 100 ms boundary; R6.4 the half-range threshold (step 0x7fff / 0x8000).
 * KNOWN_ISSUE: B-SM-2 (R6.2): one count is lost per 16 bit wrap crossing: forward, backward and accumulated over three wraps.
 */
#include "doctest.h"
#include "SmFixture.h"

#include "constants.h"

#include <cstdint>
#include <cstdlib>
#include <vector>

namespace {

/** Independent CRC-8 (poly 0x07, no reflection, init 0x52) for the fast update: bitwise */
uint8_t crc8Bitwise(const uint8_t *b, size_t n) {
	uint8_t crc = 0x52;
	for (size_t i = 0; i < n; i++) {
		crc ^= b[i];
		for (int k = 0; k < 8; k++) {
			crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
		}
	}
	return crc;
}

constexpr uint16_t INIT = FakeSmDevice::STAT_INITIALIZED;
constexpr uint16_t FAULT = FakeSmDevice::STAT_FAULTSTOP;

/** Sends one fast update through turn() and delivers the reply with the position and status the device reports */
void report(SmFixture &f, MotorSimplemotion &sm, uint16_t pos, uint16_t status = INIT) {
	f.dev.position16 = pos;
	f.dev.status16 = status;
	sm.turn(0);
	f.process();
}

/** The checks of R3.1 for one setpoint on an initialized driver */
void checkFastFrame(SmFixture &f, int16_t p) {
	auto sm = f.makeSm(1);
	REQUIRE(f.initialize(*sm));
	f.dev.clearFrames();
	const uint32_t transfers = HostPlatform::uart().txTransfers;
	sm->turn(p);
	REQUIRE(f.dev.frameCount() == 1);
	const auto &fr = f.dev.frames()[0];
	CHECK(fr.kind == FakeSmDevice::Kind::Fast);
	REQUIRE(fr.bytes.size() == 7);
	const uint16_t u = static_cast<uint16_t>(p);
	// HD: {0x10, address, p low byte, p high byte, 0, 0, crc8}; address learnt from the frame (B-SM-1)
	CHECK(fr.bytes[0] == 0x10);
	CHECK(fr.bytes[1] == f.lastAddress());
	CHECK(fr.bytes[2] == (u & 0xff));
	CHECK(fr.bytes[3] == (u >> 8));
	CHECK(fr.bytes[4] == 0);
	CHECK(fr.bytes[5] == 0);
	CHECK(fr.bytes[6] == crc8Bitwise(fr.bytes.data(), 6));
	CHECK(fr.bytes[6] == FakeSmDevice::crc8(fr.bytes.data(), 6, 0x52));
	CHECK(HostPlatform::uart().txTransfers == transfers + 1); // one transfer
	REQUIRE(f.dev.setpoints().size() == 1);
	CHECK(f.dev.setpoints()[0] == p);
	CHECK(f.dev.violations().empty());
}

/** Position difference helper: reports `first` then `second` on a fresh initialized driver and returns getPos() delta */
int32_t deltaOf(SmFixture &f, uint16_t first, uint16_t second) {
	auto sm = f.makeSm(1);
	REQUIRE(f.initialize(*sm));
	report(f, *sm, first);
	const int32_t a = sm->getPos();
	report(f, *sm, second);
	return sm->getPos() - a;
}

/** A device that runs a hook inside the busy-wait poll of the firmware (HAL_IWDG_Refresh in refreshWatchdog) */
struct PollHookDevice : FakeSmDevice {
	MotorSimplemotion *sm = nullptr;
	bool done = false;
	size_t framesBefore = 0, framesAfter = 0;
	explicit PollHookDevice(const void *port) : FakeSmDevice(port) {}
	void onPoll() override {
		if (sm && !done) {
			done = true;
			framesBefore = frameCount();
			sm->turn(123);
			framesAfter = frameCount();
		}
		FakeSmDevice::onPoll();
	}
};

} // namespace

TEST_SUITE("SM fast") {

// ---------------------------------------------------------------- R3.1

TEST_CASE("[R3.1 fast] the CRC-8 oracle of this file gives the library value 0x46 for {10 01 00 00 00 00}") {
	const uint8_t b[6] = {0x10, 0x01, 0, 0, 0, 0};
	CHECK(crc8Bitwise(b, 6) == 0x46); // SM: smFastUpdateCycle example, sm_consts.c table_crc8 with init 0x52
}

TEST_CASE_FIXTURE(SmFixture, "[R3.1 fast] turn(0) sends the 7 byte fast update with the setpoint 0 and the CRC-8 of the SM algorithm") {
	checkFastFrame(*this, 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.1 fast] turn(1) sends the setpoint 1 as the little endian word 01 00") {
	checkFastFrame(*this, 1);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.1 fast] turn(1000) sends the setpoint 1000 as the little endian word E8 03") {
	checkFastFrame(*this, 1000); // 1000 = 0x03E8
}

TEST_CASE_FIXTURE(SmFixture, "[R3.1 fast] turn(32767) sends the setpoint 32767 as the little endian word FF 7F") {
	checkFastFrame(*this, 32767);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.1 fast] turn(-1) sends the setpoint -1 as the two's complement word FF FF") {
	checkFastFrame(*this, -1);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.1 fast] turn(-32768) sends the setpoint -32768 as the two's complement word 00 80") {
	checkFastFrame(*this, -32768);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.1 fast] the fast update of the second instance carries the same frame layout") {
	auto sm = makeSm(2);
	REQUIRE(initialize(*sm));
	dev.clearFrames();
	sm->turn(-300); // -300 = 0xFED4
	REQUIRE(dev.frameCount() == 1);
	const auto &b = dev.frames()[0].bytes;
	REQUIRE(b.size() == 7);
	CHECK(b[0] == 0x10);
	CHECK(b[2] == 0xD4);
	CHECK(b[3] == 0xFE);
	CHECK(b[6] == crc8Bitwise(b.data(), 6));
}

TEST_CASE_FIXTURE(SmFixture, "[R3.1 fast] the address byte is the same in two consecutive fast frames and in the instant frames of the settings sequence") {
	// Limit: only consistency is checked; the true value (instance + 1) is checked by the B-SM-1 KNOWN_ISSUE_UB cases (R1.4)
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	const uint8_t instantAddress = dev.frames()[0].bytes[2]; // {0x24, len, address, ...}
	for (const auto &fr : dev.frames()) {
		REQUIRE(fr.bytes.size() > 2);
		CHECK(fr.bytes[2] == instantAddress);
	}
	dev.clearFrames();
	sm->turn(1);
	process();
	sm->turn(2);
	REQUIRE(dev.frameCount() == 2);
	CHECK(dev.frames()[0].bytes[1] == dev.frames()[1].bytes[1]);
	CHECK(dev.frames()[0].bytes[1] == instantAddress);
}

// ---------------------------------------------------------------- R3.2

TEST_CASE_FIXTURE(SmFixture, "[R3.2 fast] the write enable pin is high after turn and low after the transmit-complete interrupt") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	CHECK((GPIOE->ODR & DRV_GP1_Pin) == 0);
	sm->turn(0);
	CHECK((GPIOE->ODR & DRV_GP1_Pin) != 0);
	process(); // autoTxComplete on: transmit complete interrupt, then the reply
	CHECK((GPIOE->ODR & DRV_GP1_Pin) == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.2 fast] characterization: without a transmit-complete interrupt the write enable pin stays high") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.autoTxComplete = false;
	sm->turn(0);
	process();
	CHECK(dev.txCompletePending());
	CHECK((GPIOE->ODR & DRV_GP1_Pin) != 0); // set at the start of the transmit, only the interrupt resets it
}

// ---------------------------------------------------------------- R3.3

TEST_CASE_FIXTURE(SmFixture, "[R3.3 fast] a valid reply updates the position from the first word (little endian)") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0x1234);
	CHECK(dev.bytesNotAccepted() == 0);
	CHECK(sm->getPos() == 0x1234); // HD: report below 0x8000 = position
}

TEST_CASE_FIXTURE(SmFixture, "[R3.3 fast] a valid reply updates the status from the second word (little endian)") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0, 0x1003);
	CHECK(CommandHelpers::get(*sm, "state") == 0x1003);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.3 fast] after a valid reply the port is released and the next turn is sent") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 5);
	dev.clearFrames();
	sm->turn(77);
	REQUIRE(dev.frameCount() == 1);
	CHECK(dev.frames()[0].kind == FakeSmDevice::Kind::Fast);
	CHECK(dev.setpoints().back() == 77);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.3 fast] the receive is re-armed for every reply: consecutive turns each update the position") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	for (uint16_t p : {10, 20, 30}) {
		report(*this, *sm, p);
		CHECK(sm->getPos() == p);
	}
	CHECK(dev.bytesNotAccepted() == 0);
}

// ---------------------------------------------------------------- R3.4

TEST_CASE_FIXTURE(SmFixture, "[R3.4 fast] a reply with a wrong CRC-8 increments the crcerr counter") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	CHECK(CommandHelpers::get(*sm, "crcerr") == 0);
	dev.corruptCrc = true;
	report(*this, *sm, 100);
	CHECK(CommandHelpers::get(*sm, "crcerr") == 1);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.4 fast] a reply with a wrong CRC-8 leaves the position and the status unchanged") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 100, INIT | 0x0001);
	REQUIRE(sm->getPos() == 100);
	dev.corruptCrc = true;
	report(*this, *sm, 200, INIT | FAULT);
	CHECK(sm->getPos() == 100);
	CHECK(CommandHelpers::get(*sm, "state") == (INIT | 0x0001));
	CHECK(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R3.4 fast] after a reply with a wrong CRC-8 the port is released and the next turn is sent and accepted") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.corruptCrc = true;
	report(*this, *sm, 100);
	dev.corruptCrc = false;
	dev.clearFrames();
	report(*this, *sm, 300);
	REQUIRE(dev.frameCount() == 1);
	CHECK(sm->getPos() == 300);
}

// ---------------------------------------------------------------- R3.5

TEST_CASE_FIXTURE(SmFixture, "[R3.5 fast] a silent device gives no position update") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 50);
	dev.silent = true;
	sm->turn(0);
	REQUIRE(dev.frameCount() >= 1); // the frame was sent, the peer stayed silent
	dev.position16 = 999;
	process();
	CHECK(sm->getPos() == 50);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.5 fast] after a silent device answered again 11 ms later the next turn is sent and its reply is processed") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.silent = true;
	report(*this, *sm, 999);
	dev.silent = false;
	advanceMs(11);
	dev.clearFrames();
	report(*this, *sm, 40);
	REQUIRE(dev.frameCount() == 1);
	CHECK(dev.frames()[0].kind == FakeSmDevice::Kind::Fast);
	CHECK(sm->getPos() == 40);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.5 fast] characterization: a turn 9 ms after an unanswered one sends nothing, 11 ms after it a new frame is sent (10 ms after the last received byte)") {
	// The silent frame is sent in the same millisecond as the last byte received (the end of initialize()), so the 10 ms
	// of the firmware (HAL_GetTick() - lastTimeByteReceived > 10 while the port is taken) run from that byte.
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.silent = true;
	report(*this, *sm, 999);
	dev.clearFrames();
	advanceMs(9);
	sm->turn(1);
	process();
	CHECK(dev.frameCount() == 0); // 9 ms after the last received byte: the port is still taken by the unanswered exchange
	advanceMs(2); // 11 ms
	sm->turn(2);
	CHECK(dev.frameCount() == 1);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.5 fast] characterization: after 50 ms without any received byte the next turn after an unanswered one is sent at once, the unanswered frame starts no timeout of its own") {
	// NEEDS REVIEW, possible defect: the recovery time is measured from the last received byte, not from the unanswered
	// frame; with an idle gap of 50 ms before the silent turn the 10 ms are already over and the next turn follows
	// immediately (0 ms and 9 ms after the unanswered frame).
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	advanceMs(50);
	dev.silent = true;
	sm->turn(0);
	process();
	dev.clearFrames();
	sm->turn(1);
	CHECK(dev.frameCount() == 1); // 0 ms after the unanswered frame
	process();
	advanceMs(9);
	dev.clearFrames();
	sm->turn(2);
	CHECK(dev.frameCount() == 1); // 9 ms after it
}

// ---------------------------------------------------------------- R3.6

TEST_CASE_FIXTURE(SmFixture, "[R3.6 fast] while a parameter transfer waits for its reply a turn sends nothing") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	PollHookDevice probe(&huart3);
	probe.sm = sm.sm;
	HostPlatform::setUartPeer(&probe);
	// the "voltage" command is a parameter read (SMP_ACTUAL_BUS_VOLTAGE, read1Parameter, busy-wait for the reply); the param enum is private
	CommandHelpers::get(*sm, "voltage");
	HostPlatform::setUartPeer(&dev);
	REQUIRE(probe.done);
	CHECK(probe.framesBefore == 1); // the instant command only
	CHECK(probe.framesAfter == probe.framesBefore); // turn(123) inside the wait sent nothing
	CHECK(probe.setpoints().empty());
	CHECK(probe.violations().empty());
}

TEST_CASE_FIXTURE(SmFixture, "[R3.6 fast] a turn on an uninitialized driver whose device reports status 0 sends only the settings requests, no torque") {
	dev.params[FakeSmDevice::SMP_STATUS] = 0;
	auto sm = makeSm(1);
	HostPlatform::advanceMs(150);
	sm->turn(500);
	REQUIRE(dev.frameCount() == 3); // devtype read, faults write, status read (status 0: stop)
	for (const auto &fr : dev.frames()) {
		CHECK(fr.kind == FakeSmDevice::Kind::Instant);
	}
	CHECK(dev.setpoints().empty());
	CHECK_FALSE(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R3.6 fast] a turn on an uninitialized driver within 150 ms of the last settings attempt sends nothing") {
	dev.params[FakeSmDevice::SMP_STATUS] = 0;
	auto sm = makeSm(1);
	CHECK_FALSE(initialize(*sm)); // fails, records the time of the last send
	dev.clearFrames();
	sm->turn(500);
	CHECK(dev.frameCount() == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.6 fast] a turn on an uninitialized driver runs the six settings requests and then sends the torque in the same call") {
	auto sm = makeSm(1);
	HostPlatform::advanceMs(150);
	sm->turn(500);
	REQUIRE(dev.frameCount() == 7);
	for (size_t i = 0; i < 6; i++) {
		CHECK(dev.frames()[i].kind == FakeSmDevice::Kind::Instant);
	}
	CHECK(dev.frames()[6].kind == FakeSmDevice::Kind::Fast);
	REQUIRE(dev.setpoints().size() == 1);
	CHECK(dev.setpoints()[0] == 500);
	CHECK(sm->motorReady());
}

// ---------------------------------------------------------------- R3.7

TEST_CASE_FIXTURE(SmFixture, "[R3.7 fast] getPos 50 ms after the last position update sends no fast update") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 10);
	dev.clearFrames();
	advanceMs(50);
	CHECK(sm->getPos() == 10);
	CHECK(dev.frameCount() == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.7 fast] getPos 150 ms after the last position update sends a fast update with the last torque") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	sm->turn(-321);
	process();
	dev.clearFrames();
	advanceMs(150);
	CHECK(sm->getPos() == 0); // the position of the reply of the first turn: report 0
	REQUIRE(dev.frameCount() == 1);
	CHECK(dev.frames()[0].kind == FakeSmDevice::Kind::Fast);
	REQUIRE(dev.setpoints().size() == 1);
	CHECK(dev.setpoints()[0] == -321);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.7 fast] getPos returns the position known before the refresh and the reply to the refresh updates it") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 10);
	advanceMs(150);
	dev.position16 = 60;
	CHECK(sm->getPos() == 10); // the reply has not been delivered yet
	process();
	CHECK(sm->getPos() == 60);
}

TEST_CASE_FIXTURE(SmFixture, "[R3.7 fast] characterization: the refresh threshold is 100 ms, exactly 100 ms sends nothing and 101 ms sends a fast update") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 10);
	dev.clearFrames();
	advanceMs(100);
	sm->getPos();
	CHECK(dev.frameCount() == 0);
	advanceMs(1);
	sm->getPos();
	CHECK(dev.frameCount() == 1);
}

// ---------------------------------------------------------------- R6

TEST_CASE_FIXTURE(SmFixture, "[R6.1 fast] the first report 0 gives the position 0") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0);
	CHECK(sm->getPos() == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R6.1 fast] the first report 1 gives the position 1") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 1);
	CHECK(sm->getPos() == 1);
}

TEST_CASE_FIXTURE(SmFixture, "[R6.1 fast] the first report 0x7fff gives the position 0x7fff") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0x7fff);
	CHECK(sm->getPos() == 0x7fff);
}

TEST_CASE_FIXTURE(SmFixture, "[R6.2 fast] motion forward over the 16 bit counter wrap 0xfff0 to 0x0010 advances by 0x20") {
	// HD: reports 0xfff0 -> 0x0010 are 0x20 counts apart (mod 65536); the shifted values 0x7fef -> 0x800f do not cross the
	// overflow step of the firmware, so the result is exact
	CHECK(deltaOf(*this, 0xfff0, 0x0010) == 0x20);
}

TEST_CASE_FIXTURE(SmFixture, "[R6.2 fast] motion backward over the 16 bit counter wrap 0x0010 to 0xfff0 goes back by 0x20") {
	CHECK(deltaOf(*this, 0x0010, 0xfff0) == -0x20);
}

TEST_CASE_FIXTURE(SmFixture, "[R6.2 fast] motion forward by 0x4000 inside one half range is exact") {
	CHECK(deltaOf(*this, 0x1000, 0x5000) == 0x4000);
}

TEST_CASE_FIXTURE(SmFixture, "[R6.2 fast] KNOWN_ISSUE B-SM-2: motion forward over the report step 0x7ff0 to 0x8010 advances by 0x20" * KNOWN_ISSUE("B-SM-2: position = value + 0xffff * overflows - 0x7fff loses one count per wrap; measured 0x1f, intended 0x20 (HD: 0x8010 - 0x7ff0)")) {
	CHECK(deltaOf(*this, 0x7ff0, 0x8010) == 0x20);
}

TEST_CASE_FIXTURE(SmFixture, "[R6.2 fast] KNOWN_ISSUE B-SM-2: motion backward over the report step 0x8010 to 0x7ff0 goes back by 0x20" * KNOWN_ISSUE("B-SM-2: one count lost per wrap; measured -0x1f, intended -0x20 (HD: 0x7ff0 - 0x8010)")) {
	CHECK(deltaOf(*this, 0x8010, 0x7ff0) == -0x20);
}

TEST_CASE_FIXTURE(SmFixture, "[R6.2 fast] KNOWN_ISSUE B-SM-2: three full 16 bit revolutions forward accumulate 3 * 65536 counts" * KNOWN_ISSUE("B-SM-2: 0xffff instead of 0x10000 per wrap; measured 3 * 65536 - 3 = 196605, intended 3 * 65536 = 196608")) {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0);
	const int32_t start = sm->getPos();
	// 12 steps of 0x4000 = 3 * 0x10000 reported counts, every step below half range
	uint16_t r = 0;
	for (int i = 0; i < 12; i++) {
		r = static_cast<uint16_t>(r + 0x4000);
		report(*this, *sm, r);
	}
	CHECK(sm->getPos() - start == 3 * 65536);
}

TEST_CASE_FIXTURE(SmFixture, "[R6.2 fast] three full 16 bit revolutions forward move the position up by 3 * 65536 within the 3 counts of B-SM-2") {
	// HD: 12 steps of 0x4000 = 196608 reported counts forward; the one count lost per wrap bounds the error to 3
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0);
	const int32_t start = sm->getPos();
	uint16_t r = 0;
	for (int i = 0; i < 12; i++) {
		r = static_cast<uint16_t>(r + 0x4000);
		report(*this, *sm, r);
	}
	const int32_t d = sm->getPos() - start;
	CHECK(d > 0);
	CHECK(std::abs(d - 3 * 65536) <= 3);
}

TEST_CASE_FIXTURE(SmFixture, "[R6.2 fast] three full 16 bit revolutions backward move the position down by 3 * 65536 within the 3 counts of B-SM-2") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0);
	const int32_t start = sm->getPos();
	uint16_t r = 0;
	for (int i = 0; i < 12; i++) {
		r = static_cast<uint16_t>(r - 0x4000);
		report(*this, *sm, r);
	}
	const int32_t d = sm->getPos() - start;
	CHECK(d < 0);
	CHECK(std::abs(d + 3 * 65536) <= 3);
}

TEST_CASE_FIXTURE(SmFixture, "[R6.3 fast] setPos makes getPos return the given position") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 50);
	sm->setPos(1000);
	CHECK(sm->getPos() == 1000);
}

TEST_CASE_FIXTURE(SmFixture, "[R6.3 fast] motion after setPos is relative to the given position") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 50);
	sm->setPos(-200);
	report(*this, *sm, 80); // +30
	CHECK(sm->getPos() == -170);
}

TEST_CASE_FIXTURE(SmFixture, "[R6.4 fast] characterization: a step of exactly 0x7fff reports is followed as a forward motion of 0x7fff") {
	CHECK(deltaOf(*this, 0, 0x7fff) == 0x7fff);
}

TEST_CASE_FIXTURE(SmFixture, "[R6.4 fast] characterization: a step of exactly 0x8000 reports is counted as an overflow, the position moves by -0x7fff") {
	// 0x8000 apart is half a revolution: forward and backward are equally near, the SM documents give no rule.
	// NEEDS REVIEW: the -0x7fff is also one count short of -0x8000 (the B-SM-2 loss at the overflow step)
	CHECK(deltaOf(*this, 0, 0x8000) == -0x7fff);
}

// ---------------------------------------------------------------- R7

TEST_CASE_FIXTURE(SmFixture, "[R7.1 fast] a reply with the FAULTSTOP bit (bit 5) makes motorReady false") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	REQUIRE(sm->motorReady());
	report(*this, *sm, 0, INIT | FAULT);
	CHECK_FALSE(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R7.1 fast] a later reply without the FAULTSTOP bit makes the driver ready again") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0, INIT | FAULT);
	REQUIRE_FALSE(sm->motorReady());
	report(*this, *sm, 0, INIT);
	CHECK(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R7.1 fast] a FAULTSTOP reply does not clear the initialized state: the next turn sends no settings requests") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0, INIT | FAULT);
	advanceMs(150);
	dev.clearFrames();
	sm->turn(10);
	REQUIRE(dev.frameCount() == 1);
	CHECK(dev.frames()[0].kind == FakeSmDevice::Kind::Fast);
}

TEST_CASE_FIXTURE(SmFixture, "[R7.2 fast] a reply without the INITIALIZED bit (bit 12) makes motorReady false") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0, 0x0000);
	CHECK_FALSE(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R7.2 fast] after the INITIALIZED bit cleared the next turn within 150 ms sends nothing") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0, 0x0000);
	dev.clearFrames();
	sm->turn(10);
	CHECK(dev.frameCount() == 0);
}

TEST_CASE_FIXTURE(SmFixture, "[R7.2 fast] after the INITIALIZED bit cleared the next turn after 150 ms runs the six settings requests before the torque") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0, 0x0000);
	dev.clearFrames();
	advanceMs(150);
	sm->turn(10);
	REQUIRE(dev.frameCount() == 7);
	for (size_t i = 0; i < 6; i++) {
		CHECK(dev.frames()[i].kind == FakeSmDevice::Kind::Instant);
	}
	CHECK(dev.frames()[6].kind == FakeSmDevice::Kind::Fast);
	REQUIRE(dev.setpoints().size() == 1);
	CHECK(dev.setpoints()[0] == 10);
}

TEST_CASE_FIXTURE(SmFixture, "[R7.2 fast] after the settings sequence ran again and the device reports INITIALIZED the driver is ready") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0, 0x0000);
	advanceMs(150);
	dev.status16 = INIT;
	sm->turn(10);
	process();
	CHECK(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R7.2 fast] characterization: a reply with bit 12 set after bit 12 was clear does not make the driver ready without the settings sequence") {
	// NEEDS REVIEW: updateStatus() only clears `initialized`, it never sets it; only getSettings() does. The charter (R7.2) says
	// the next turn re-runs the settings, so this agrees with it, but a "restart-free recovery by status bit" does not exist.
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0, 0x0000);
	REQUIRE_FALSE(sm->motorReady());
	report(*this, *sm, 0, INIT);
	CHECK_FALSE(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R7.3 fast] the state command returns the status read by the settings sequence after initialization") {
	dev.params[FakeSmDevice::SMP_STATUS] = 0x1007;
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	CHECK(CommandHelpers::get(*sm, "state") == 0x1007);
}

TEST_CASE_FIXTURE(SmFixture, "[R7.3 fast] the state command returns the status word of the last fast reply") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	report(*this, *sm, 0, INIT | 0x0101);
	CHECK(CommandHelpers::get(*sm, "state") == (INIT | 0x0101));
}

}
