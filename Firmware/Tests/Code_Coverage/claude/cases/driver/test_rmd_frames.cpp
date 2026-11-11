/*
 * MyActuator RMD CAN driver (RmdMotorCAN), work order RMF: requirements R3 (frames sent, host to motor) and R4 (thread
 * behaviour, cyclic requests) of Tests/Code_Coverage/claude/charters/driver/RMD_REQUIREMENTS.md. Suite "RMD frames".
 *
 * Oracles: SP = MyActuator "CAN Bus Motor Motion Protocol V4.3" (cited by section: 1.1.2 ids and DLC, 2.17 0x80 shutdown,
 * 2.19 0xA1 torque closed loop, 2.34 0xB6 active reply, 2.33 0xB5 model, 2.14 0x9A status); HD = hand derivation in a
 * comment next to the assertion (frame bytes, little endian words, `turn` arithmetic); FW = characterization of the
 * firmware's own constants (test name starts with "characterization:", NEEDS REVIEW).
 *
 * Frames are read from the fake CAN transmit log (HostTarget::canTxFrames: id, ide, rtr, dlc, data, time). The peer fake
 * additionally fails the test on any frame that violates SP (extended id, RTR, DLC != 8, unknown command byte, non-zero
 * NULL field), so the explicit checks below are the first line and the peer is the second.
 *
 * Host versus target: `turn` converts in single precision float ((float) p / (float) 0x7fff * maxTorque, truncation toward
 * zero); the values below are derived by hand and are the same on the Cortex-M4F (IEEE single, same operations). All values
 * are in range of int16_t, so no out-of-range float conversion is involved. Time is simulated (RmdFixture::runFor).
 *
 * Characterizations of this file (NEEDS REVIEW): R3.8 (no 0x77 brake release in a session, owner question 50),
 * R4.1 / R4.2 / R4.3 / R4.4 (timing constants 100, 20, 150, 1000 ms: the SP does not specify them).
 * KNOWN_ISSUEs of this file: none.
 *
 * R4.6 limit: those tests cannot tell a deleted thread from a thread the host RTOS no longer schedules (the deletion
 * itself lives in the vendor cpp_freertos::Thread base); they prove that no frame is sent after the destruction.
 *
 * Charter wording note (R4.4): the code sends the disable frame {0xB6, 0x92, 0, ...} when the mode is switched TO the active
 * request mode (requestpos 1); switching back to the passive mode sends nothing at once and the thread re-enables the
 * cyclic report in the next cycle. The tests follow the code (the charter sentence is ambiguous on the direction).
 */
#include "doctest.h"
#include "RmdFixture.h"

#include <array>
#include <cstdint>
#include <vector>

namespace {

using Bytes = std::array<uint8_t, 8>;
using Frame = HostTarget::CanTxFrame;

struct FramesFixture : RmdFixture {
	/** The motor is available, started (active) and the frame log is empty */
	void startActive(RmdMotorCAN &d) {
		reachAvailable(d);
		d.startMotor();
		clearTxFrames();
	}
	/** Lets the peers decode, then returns the log */
	const std::vector<Frame> &frames() {
		process();
		return HostTarget::canTxFrames();
	}
	static uint32_t cmdCount(uint32_t id, uint8_t cmd) { return uint32_t(txFramesCmd(id, cmd).size()); }
	static void setReq(RmdMotorCAN &d, int v) { CHECK(CommandHelpers::run(d, "requestpos", CMDtype::set, v) == CommandStatus::OK); }
	/** Frames with command byte 0xB6 and enable byte (data[2]) `en` */
	static std::vector<Frame> b6(uint8_t en) {
		std::vector<Frame> r;
		for (const auto &f : txFramesCmd(0x141, 0xB6)) {
			if (f.data[2] == en) {
				r.push_back(f);
			}
		}
		return r;
	}
	/** One 0xA1 frame with iq lo / hi at data[4..5] and everything else 0 is the only frame logged */
	void expectOnlyTorque(uint8_t lo, uint8_t hi) {
		const auto &fr = frames();
		REQUIRE(fr.size() == 1);
		const Bytes want{0xA1, 0, 0, 0, lo, hi, 0, 0};
		CHECK(fr[0].id == 0x141);
		CHECK(fr[0].dlc == 8);
		CHECK(fr[0].data == want);
	}
};

const Bytes ZERO_TORQUE{0xA1, 0, 0, 0, 0, 0, 0, 0};
const Bytes SHUTDOWN{0x80, 0, 0, 0, 0, 0, 0, 0};

} // namespace

TEST_SUITE("RMD frames") {

// ---------------------------------------------------------------- R3.1 frame format

TEST_CASE_FIXTURE(FramesFixture, "[R3.1] the first frame of instance 1 is a standard 0x141 data frame with DLC 8 and 0xB5 with seven zero bytes") {
	auto rmd = makeRmd1();
	runFor(100);
	const auto &fr = frames();
	REQUIRE(fr.size() >= 1);
	CHECK(fr[0].id == 0x141); // SP 1.1.2: 0x140 + ID 1
	CHECK_FALSE(fr[0].extId);
	CHECK_FALSE(fr[0].rtr);
	CHECK(fr[0].dlc == 8);
	const Bytes want{0xB5, 0, 0, 0, 0, 0, 0, 0}; // SP 2.33: 0xB5, rest NULL
	CHECK(fr[0].data == want);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.1] every frame of instance 2 uses the standard id 0x142 (node 2) with DLC 8") {
	auto rmd = makeRmd2();
	runFor(100);
	rmd->startMotor();
	rmd->stopMotor();
	const auto &fr = frames();
	REQUIRE(fr.size() == 4); // 0xB5 at 100 ms (the 0xB6 follows at 120 ms), A1, A1, 0x80
	for (const auto &f : fr) {
		CHECK(f.id == 0x142);
		CHECK_FALSE(f.extId);
		CHECK_FALSE(f.rtr);
		CHECK(f.dlc == 8);
	}
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.1] in a whole session every frame is a standard 0x141 data frame with DLC 8 and the SP unused bytes are 0") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	rmd->startMotor();
	rmd->turn(12345);
	rmd->setTorque(-7);
	rmd->requestConstantReports(0x92, true, 300);
	CHECK(CommandHelpers::run(*rmd, "requestpos", CMDtype::set, 1) == CommandStatus::OK);
	runFor(2500);
	rmd->stopMotor();
	const auto &fr = frames();
	REQUIRE(fr.size() >= 8);
	for (const auto &f : fr) {
		CHECK(f.id == 0x141);
		CHECK_FALSE(f.extId);
		CHECK_FALSE(f.rtr);
		CHECK(f.dlc == 8);
		switch (f.data[0]) {
		case 0xA1: // SP 2.19: bytes 1..3, 6, 7 NULL
			CHECK(f.data[1] == 0);
			CHECK(f.data[2] == 0);
			CHECK(f.data[3] == 0);
			CHECK(f.data[6] == 0);
			CHECK(f.data[7] == 0);
			break;
		case 0xB6: // SP 2.34: bytes 5..7 NULL
			CHECK(f.data[5] == 0);
			CHECK(f.data[6] == 0);
			CHECK(f.data[7] == 0);
			break;
		case 0x80:
		case 0x92:
		case 0x9A:
		case 0xB5:
			for (int i = 1; i < 8; i++) {
				CHECK(f.data[i] == 0);
			}
			break;
		default:
			CHECK_MESSAGE(false, "unexpected command byte " << int(f.data[0]));
		}
	}
	CHECK(peer1.violations().empty());
}

// ---------------------------------------------------------------- R3.2 setTorque

// SP 2.19: frame {0xA1, 0, 0, 0, iq lo, iq hi, 0, 0}, iq int16 little endian two's complement, 0.01 A / LSB.
TEST_CASE_FIXTURE(FramesFixture, "[R3.2] setTorque(0) sends {0xA1, 0, 0, 0, 0x00, 0x00, 0, 0}") {
	auto rmd = makeRmd1();
	clearTxFrames();
	rmd->setTorque(0);
	expectOnlyTorque(0x00, 0x00);
	CHECK(peer1.lastCommandedIq() == 0);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.2] setTorque(1) sends iq bytes 0x01 0x00") {
	auto rmd = makeRmd1();
	clearTxFrames();
	rmd->setTorque(1);
	expectOnlyTorque(0x01, 0x00);
	CHECK(peer1.lastCommandedIq() == 1);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.2] setTorque(100) sends iq bytes 0x64 0x00 (SP 2.19 example)") {
	auto rmd = makeRmd1();
	clearTxFrames();
	rmd->setTorque(100); // 100 = 0x0064 -> lo 0x64, hi 0x00
	expectOnlyTorque(0x64, 0x00);
	CHECK(peer1.lastCommandedIq() == 100);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.2] setTorque(-100) sends iq bytes 0x9C 0xFF (SP 2.19 example)") {
	auto rmd = makeRmd1();
	clearTxFrames();
	rmd->setTorque(-100); // -100 = 0xFF9C (65536 - 100 = 65436) -> lo 0x9C, hi 0xFF
	expectOnlyTorque(0x9C, 0xFF);
	CHECK(peer1.lastCommandedIq() == -100);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.2] setTorque(32767) sends iq bytes 0xFF 0x7F") {
	auto rmd = makeRmd1();
	clearTxFrames();
	rmd->setTorque(32767); // 0x7FFF
	expectOnlyTorque(0xFF, 0x7F);
	CHECK(peer1.lastCommandedIq() == 32767);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.2] setTorque(-32768) sends iq bytes 0x00 0x80") {
	auto rmd = makeRmd1();
	clearTxFrames();
	rmd->setTorque(-32768); // 0x8000
	expectOnlyTorque(0x00, 0x80);
	CHECK(peer1.lastCommandedIq() == -32768);
}

// ---------------------------------------------------------------- R3.3 turn

// maxtorque default 1000. HD: torque = trunc((float) p / 32767.0f * 1000).
TEST_CASE_FIXTURE(FramesFixture, "[R3.3] turn(0) with maxtorque 1000 sends torque 0") {
	auto rmd = makeRmd1();
	startActive(*rmd);
	rmd->turn(0); // 0 / 32767 * 1000 = 0
	expectOnlyTorque(0x00, 0x00);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.3] turn(32767) with maxtorque 1000 sends torque 1000 (0x03E8)") {
	auto rmd = makeRmd1();
	startActive(*rmd);
	rmd->turn(32767); // 32767 / 32767 = 1.0 exactly, * 1000 = 1000 = 0x03E8 -> E8 03
	expectOnlyTorque(0xE8, 0x03);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.3] turn(-32767) with maxtorque 1000 sends torque -1000 (0xFC18)") {
	auto rmd = makeRmd1();
	startActive(*rmd);
	rmd->turn(-32767); // -1.0 * 1000 = -1000 = 0xFC18 (65536 - 1000 = 64536) -> 18 FC
	expectOnlyTorque(0x18, 0xFC);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.3] turn(16384) with maxtorque 1000 sends torque 500 (0x01F4)") {
	auto rmd = makeRmd1();
	startActive(*rmd);
	rmd->turn(16384); // 16384 / 32767 = 0.500015259, * 1000 = 500.015 -> trunc 500 = 0x01F4 -> F4 01
	expectOnlyTorque(0xF4, 0x01);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.3] turn(1) with maxtorque 1000 sends torque 0 (truncation toward zero)") {
	auto rmd = makeRmd1();
	startActive(*rmd);
	rmd->turn(1); // 1 / 32767 * 1000 = 0.0305 -> trunc 0
	expectOnlyTorque(0x00, 0x00);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.3] turn(-1) with maxtorque 1000 sends torque 0 (truncation toward zero, not -1)") {
	auto rmd = makeRmd1();
	startActive(*rmd);
	rmd->turn(-1); // -0.0305 -> trunc toward zero = 0 (a floor would give -1 = FF FF)
	expectOnlyTorque(0x00, 0x00);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.3] turn sends nothing while the motor is available but not started") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	clearTxFrames();
	rmd->turn(32767);
	CHECK(frames().empty());
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.3] turn sends nothing while the motor was started but is not available") {
	auto rmd = makeRmd1();
	rmd->startMotor(); // active; no cycle has finished: available is false
	clearTxFrames();
	rmd->turn(32767);
	CHECK(frames().empty());
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.3] turn(32767) with maxtorque 500 sends torque 500 (0x01F4)") {
	auto rmd = makeRmd1();
	REQUIRE(CommandHelpers::run(*rmd, "maxtorque", CMDtype::set, 500) == CommandStatus::OK);
	startActive(*rmd);
	rmd->turn(32767); // 1.0 * 500 = 500 = 0x01F4 -> F4 01
	expectOnlyTorque(0xF4, 0x01);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.3] turn(-16384) with maxtorque 200 sends torque -100 (0xFF9C)") {
	auto rmd = makeRmd1();
	REQUIRE(CommandHelpers::run(*rmd, "maxtorque", CMDtype::set, 200) == CommandStatus::OK);
	startActive(*rmd);
	rmd->turn(-16384); // -16384 / 32767 = -0.500015259, * 200 = -100.003 -> trunc toward zero -100 = 0xFF9C -> 9C FF
	expectOnlyTorque(0x9C, 0xFF);
}

// ---------------------------------------------------------------- R3.4 startMotor

TEST_CASE_FIXTURE(FramesFixture, "[R3.4] startMotor sends exactly one zero torque frame {0xA1, 0 ...} and the motor runs") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	clearTxFrames();
	rmd->startMotor();
	const auto &fr = frames();
	REQUIRE(fr.size() == 1);
	CHECK(fr[0].id == 0x141);
	CHECK(fr[0].data == ZERO_TORQUE);
	CHECK(peer1.running()); // SP 2.19: a torque command starts the motor
	rmd->turn(32767);		// proves the driver is active: the frame is sent now
	CHECK(frames().size() == 2);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.4] startMotor with error flags set sends nothing and the driver stays inactive") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	peer1.setErrors(0x0004); // SP 2.14: low voltage
	peer1.sendReply(0x9A);
	REQUIRE(CommandHelpers::get(*rmd, "errors") == 0x0004);
	peer1.setErrors(0);
	clearTxFrames();
	rmd->startMotor();
	CHECK(frames().empty());
	rmd->turn(32767); // still inactive (and motorReady is false)
	CHECK(frames().empty());
}

// ---------------------------------------------------------------- R3.5 stopMotor

TEST_CASE_FIXTURE(FramesFixture, "[R3.5] stopMotor sends {0xA1, 0 ...} first and then the shutdown {0x80, 0 ...}") {
	auto rmd = makeRmd1();
	startActive(*rmd);
	rmd->stopMotor();
	const auto &fr = frames();
	REQUIRE(fr.size() == 2);
	CHECK(fr[0].id == 0x141);
	CHECK(fr[0].data == ZERO_TORQUE);
	CHECK(fr[1].id == 0x141);
	CHECK(fr[1].data == SHUTDOWN); // SP 2.17
	CHECK_FALSE(peer1.running());
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.5] turn sends nothing after stopMotor") {
	auto rmd = makeRmd1();
	startActive(*rmd);
	rmd->stopMotor();
	clearTxFrames();
	rmd->turn(32767);
	CHECK(frames().empty());
}

// ---------------------------------------------------------------- R3.6 emergencyStop

TEST_CASE_FIXTURE(FramesFixture, "[R3.6] emergencyStop(false) sends the stopMotor frames {0xA1, 0 ...} then {0x80, 0 ...} and silences turn") {
	auto rmd = makeRmd1();
	startActive(*rmd);
	rmd->emergencyStop(false);
	const auto &fr = frames();
	REQUIRE(fr.size() == 2);
	CHECK(fr[0].data == ZERO_TORQUE);
	CHECK(fr[1].data == SHUTDOWN);
	clearTxFrames();
	rmd->turn(32767);
	CHECK(frames().empty());
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.6] emergencyStop(true) sends the startMotor frame {0xA1, 0 ...} and turn works afterwards") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	clearTxFrames();
	rmd->emergencyStop(true);
	const auto &fr = frames();
	REQUIRE(fr.size() == 1);
	CHECK(fr[0].data == ZERO_TORQUE);
	rmd->turn(32767);
	REQUIRE(frames().size() == 2);
	CHECK(HostTarget::canTxFrames()[1].data[4] == 0xE8); // torque 1000
	CHECK(HostTarget::canTxFrames()[1].data[5] == 0x03);
}

// ---------------------------------------------------------------- R3.7 requestConstantReports

TEST_CASE_FIXTURE(FramesFixture, "[R3.7] requestConstantReports(0x92, true, 0) sends {0xB6, 0x92, 1, 0, 0, 0, 0, 0}") {
	auto rmd = makeRmd1();
	clearTxFrames();
	rmd->requestConstantReports(0x92, true, 0);
	const auto &fr = frames();
	REQUIRE(fr.size() == 1);
	const Bytes want{0xB6, 0x92, 1, 0, 0, 0, 0, 0}; // SP 2.34
	CHECK(fr[0].id == 0x141);
	CHECK(fr[0].data == want);
	CHECK(peer1.cyclicEnabled(0x92));
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.7] requestConstantReports with interval 1 sends interval bytes 0x01 0x00") {
	auto rmd = makeRmd1();
	clearTxFrames();
	rmd->requestConstantReports(0x92, true, 1);
	const auto &fr = frames();
	REQUIRE(fr.size() == 1);
	const Bytes want{0xB6, 0x92, 1, 0x01, 0x00, 0, 0, 0};
	CHECK(fr[0].data == want);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.7] requestConstantReports with interval 300 sends interval bytes 0x2C 0x01 (little endian)") {
	auto rmd = makeRmd1();
	clearTxFrames();
	rmd->requestConstantReports(0x92, true, 300); // 300 = 0x012C -> lo 0x2C, hi 0x01
	const auto &fr = frames();
	REQUIRE(fr.size() == 1);
	const Bytes want{0xB6, 0x92, 1, 0x2C, 0x01, 0, 0, 0};
	CHECK(fr[0].data == want);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.7] requestConstantReports(0x9A, true, 5) sends {0xB6, 0x9A, 1, 0x05, 0, 0, 0, 0}") {
	auto rmd = makeRmd1();
	clearTxFrames();
	rmd->requestConstantReports(0x9A, true, 5); // SP 2.34: 0x9A is in the list of cyclic commands
	const auto &fr = frames();
	REQUIRE(fr.size() == 1);
	const Bytes want{0xB6, 0x9A, 1, 0x05, 0x00, 0, 0, 0};
	CHECK(fr[0].data == want);
}

TEST_CASE_FIXTURE(FramesFixture, "[R3.7] requestConstantReports with enable false sends enable byte 0") {
	auto rmd = makeRmd1();
	clearTxFrames();
	rmd->requestConstantReports(0x92, false, 0);
	const auto &fr = frames();
	REQUIRE(fr.size() == 1);
	const Bytes want{0xB6, 0x92, 0, 0, 0, 0, 0, 0};
	CHECK(fr[0].data == want);
	CHECK_FALSE(peer1.cyclicEnabled(0x92));
}

// ---------------------------------------------------------------- R3.8 no brake release

TEST_CASE_FIXTURE(FramesFixture, "[R3.8] characterization: no 0x77 brake release frame is sent in a full session (owner question 50)") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	rmd->startMotor();
	rmd->turn(20000);
	runFor(3000);
	setReq(*rmd, 1);
	runFor(2500);
	setReq(*rmd, 0);
	runFor(2500);
	rmd->stopMotor();
	rmd->emergencyStop(true);
	rmd->emergencyStop(false);
	process();
	CHECK(cmdCount(0x141, 0x77) == 0);
	CHECK(peer1.count(0x77) == 0); // the peer's own decode of the same bus traffic (independent of the TX log filter)
	CHECK(peer1.violations().empty());
}

// ---------------------------------------------------------------- R4.1 thread against a silent motor

TEST_CASE_FIXTURE(FramesFixture, "[R4.1] characterization: against a silent motor 0xB5 is sent at 100, 1270 and 2440 ms and the 0xB6 enable 20 ms after each, nothing else") {
	auto rmd = makeRmd1();
	peer1.setSilent(true);
	runFor(2461);
	// cycle: Delay(100); B5, Delay(20); B6, Delay(150); Delay(1000) -> 20 + 150 + 1000 = 1170 ms per cycle.
	const auto b5 = txFramesCmd(0x141, 0xB5);
	const auto b6e = b6(1);
	REQUIRE(b5.size() == 3);
	REQUIRE(b6e.size() == 3);
	CHECK(b5[0].time == 100);
	CHECK(b5[1].time == 1270);
	CHECK(b5[2].time == 2440);
	CHECK(b6e[0].time == 120);
	CHECK(b6e[1].time == 1290);
	CHECK(b6e[2].time == 2460);
	CHECK(HostTarget::canTxFrames().size() == 6);
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.1] the 0xB6 enable of the thread is {0xB6, 0x92, 1, 0, 0, 0, 0, 0} every cycle against a silent motor") {
	auto rmd = makeRmd1();
	peer1.setSilent(true);
	runFor(2461);
	const auto b6e = b6(1);
	REQUIRE(b6e.size() == 3);
	const Bytes want{0xB6, 0x92, 1, 0, 0, 0, 0, 0}; // SP 2.34: cyclic 0x92, interval value 0
	for (const auto &f : b6e) {
		CHECK(f.id == 0x141);
		CHECK(f.dlc == 8);
		CHECK(f.data == want);
	}
}

// ---------------------------------------------------------------- R4.2 replying motor

TEST_CASE_FIXTURE(FramesFixture, "[R4.2] characterization: motorReady turns true at 1270 ms (end of the first cycle) and not before") {
	auto rmd = makeRmd1();
	runFor(1269);
	CHECK_FALSE(rmd->motorReady());
	runFor(1);
	CHECK(rmd->motorReady());
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.2] the 0xB5 request and the 0xB6 enable are sent once while the motor keeps answering") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	runFor(6000);
	CHECK(rmd->motorReady());
	CHECK(cmdCount(0x141, 0xB5) == 1);
	CHECK(b6(1).size() == 1);
	CHECK(peer1.violations().empty());
}

// ---------------------------------------------------------------- R4.3 motor falls silent

namespace {
/** Reached available at about 1300 ms, then the motor goes silent. The cycle that started at 1270 still ends available (it saw replies); the next one (ends 3270) sees none. Returns at 3269 ms */
void silenceAndRunToLastAvailableMs(FramesFixture &fx, RmdMotorCAN &d, FakeRmdPeer &p) {
	fx.reachAvailable(d);
	p.setSilent(true);
	REQUIRE(HostPlatform::nowUs() / 1000 < 3269);
	fx.runFor(uint32_t(3269 - HostPlatform::nowUs() / 1000));
}
} // namespace

TEST_CASE_FIXTURE(FramesFixture, "[R4.3] characterization: a motor that falls silent clears motorReady at the end of the next full cycle (3270 ms), not before") {
	auto rmd = makeRmd1();
	silenceAndRunToLastAvailableMs(*this, *rmd, peer1);
	CHECK(rmd->motorReady());
	runFor(1);
	CHECK_FALSE(rmd->motorReady());
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.3] turn sends nothing after the motor fell silent although the driver was started") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	rmd->startMotor();
	peer1.setSilent(true);
	REQUIRE(HostPlatform::nowUs() / 1000 < 3270);
	runFor(uint32_t(3270 - HostPlatform::nowUs() / 1000));
	REQUIRE_FALSE(rmd->motorReady());
	clearTxFrames();
	rmd->turn(32767);
	CHECK(frames().empty());
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.3] characterization: the 0xB5 probe resumes in the cycle after the motor fell silent (3270 ms)") {
	auto rmd = makeRmd1();
	silenceAndRunToLastAvailableMs(*this, *rmd, peer1);
	clearTxFrames();
	runFor(1);
	const auto b5 = txFramesCmd(0x141, 0xB5);
	REQUIRE(b5.size() == 1);
	CHECK(b5[0].time == 3270);
}

// ---------------------------------------------------------------- R4.4 / R4.5 request mode

TEST_CASE_FIXTURE(FramesFixture, "[R4.4] switching to requestpos 1 sends the disable {0xB6, 0x92, 0, 0, 0, 0, 0, 0} once at once") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	clearTxFrames();
	setReq(*rmd, 1);
	const auto &fr = frames();
	REQUIRE(fr.size() == 1);
	const Bytes want{0xB6, 0x92, 0, 0, 0, 0, 0, 0}; // SP 2.34: enable 0
	CHECK(fr[0].id == 0x141);
	CHECK(fr[0].data == want);
	CHECK_FALSE(peer1.cyclicEnabled(0x92));
	runFor(4000);
	CHECK(b6(0).size() == 1); // not repeated by the thread
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.4] characterization: with requestpos 1 the 0x9A status request is sent once per 1000 ms cycle (2270, 3270, 4270 ms) and no 0xB6 enable") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	setReq(*rmd, 1);
	clearTxFrames();
	runFor(3500); // from about 1300 to 4800 ms
	const auto s = txFramesCmd(0x141, 0x9A);
	REQUIRE(s.size() == 3);
	CHECK(s[0].time == 2270);
	CHECK(s[1].time == 3270);
	CHECK(s[2].time == 4270);
	const Bytes want{0x9A, 0, 0, 0, 0, 0, 0, 0}; // SP 2.14
	CHECK(s[0].data == want);
	CHECK(b6(1).empty());
	CHECK(cmdCount(0x141, 0xB5) == 0);
	CHECK(rmd->motorReady()); // the replies to 0x9A keep the motor available
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.4] characterization: with requestpos 1 and a silent motor the thread probes 0xB5 then 0x9A each cycle, sends no 0xB6 enable and motorReady stays false") {
	auto rmd = makeRmd1();
	peer1.setSilent(true);
	setReq(*rmd, 1);
	clearTxFrames(); // drops the disable frame of the switch
	runFor(2161);
	// cycle: B5, Delay(20), 9A, Delay(1000) = 1020 ms: B5 at 100, 1120, 2140; 9A at 120, 1140, 2160
	const auto b5 = txFramesCmd(0x141, 0xB5);
	const auto s = txFramesCmd(0x141, 0x9A);
	REQUIRE(b5.size() == 3);
	REQUIRE(s.size() == 3);
	CHECK(b5[0].time == 100);
	CHECK(b5[1].time == 1120);
	CHECK(b5[2].time == 2140);
	CHECK(s[0].time == 120);
	CHECK(s[1].time == 1140);
	CHECK(s[2].time == 2160);
	CHECK(b6(1).empty());
	CHECK(HostTarget::canTxFrames().size() == 6);
	CHECK_FALSE(rmd->motorReady());
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.4] switching back to requestpos 0 sends nothing at once and the next cycle re-enables the cyclic report once") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	setReq(*rmd, 1);
	runFor(2000);
	clearTxFrames();
	setReq(*rmd, 0);
	CHECK(frames().empty());
	runFor(1500); // the next cycle starts within 1000 ms
	const auto en = b6(1);
	REQUIRE(en.size() == 1);
	const Bytes want{0xB6, 0x92, 1, 0, 0, 0, 0, 0};
	CHECK(en[0].data == want);
	CHECK(peer1.cyclicEnabled(0x92));
	runFor(4000);
	CHECK(b6(1).size() == 1); // not repeated
	CHECK(cmdCount(0x141, 0x9A) == 0); // passive mode again: no status requests
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.5] setting requestpos 1 twice sends the disable only once") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	clearTxFrames();
	setReq(*rmd, 1);
	setReq(*rmd, 1);
	process();
	CHECK(b6(0).size() == 1);
	CHECK(HostTarget::canTxFrames().size() == 1);
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.5] setting requestpos 0 while it is 0 sends nothing, now or in the next cycles") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	clearTxFrames();
	setReq(*rmd, 0);
	CHECK(frames().empty());
	runFor(3000);
	CHECK(HostTarget::canTxFrames().empty()); // no re-enable, no status request
}

// ---------------------------------------------------------------- R4.6 destruction

TEST_CASE_FIXTURE(FramesFixture, "[R4.6] no frame is sent after the destruction of a driver whose motor answers") {
	auto rmd = makeRmd1();
	reachAvailable(*rmd);
	rmd.reset();
	clearTxFrames();
	runFor(5000);
	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(FramesFixture, "[R4.6] no 0xB5 probe or 0xB6 enable is sent after the destruction of a driver whose motor is silent") {
	auto rmd = makeRmd1();
	peer1.setSilent(true);
	runFor(500); // probes of the first cycle (100 / 120 ms) are out
	rmd.reset();
	clearTxFrames();
	runFor(5000);
	CHECK(HostTarget::canTxFrames().empty());
}

}
