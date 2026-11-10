/*
 * TMCDebugBridge, text commands (phase P13, work order TDC). Requirements R1.1 to R1.7 and the R5 items 1, 2, 3 and 10 of
 * Tests/claude/charters/driver/TMCDEBUG_REQUIREMENTS.md (lead decisions L1 to L5).
 *
 * Unit under test: TMCDebugBridge::registerCommands and TMCDebugBridge::command (TMCDebugBridge.cpp:34-126). The commands are
 * executed through CommandHelpers (CommandHandler::command with a ParsedCommand, like the command thread does), the ids are
 * resolved by name from the registered list. Everything is observed at the fake TMC4671 (chip1: SPI frame log and register
 * file) and in the replies.
 *
 * Oracles (charter table):
 *  - DRV / DS: what the TMC4671 driver calls (turn, setTargetPos, setTargetVelocity, setOpenLoopSpeedAccel, runOpenLoop,
 *    startMotor, setMotionMode, readReg, writeReg) must do is approved in D1; the bridge tests check that the bridge calls them
 *    with the table's arguments, seen as register writes. Register layouts: TMC4671-LA datasheet Rev 2.07 (DS 0x1A PWM_SV_CHOP
 *    PWM_CHOP [7:0] 7 = FOC PWM, 0x20 OPENLOOP_ACCELERATION u32, 0x21 OPENLOOP_VELOCITY_TARGET s32, 0x23 OPENLOOP_PHI, 0x24
 *    UQ_UD_EXT UD [15:0] UQ [31:16], 0x52 PHI_E_SELECTION 2 = phi_e_openloop, 0x53 PHI_E, 0x63 MODE_MOTION [7:0] 0 stopped,
 *    1 torque, 2 velocity, 3 position, 4..6 prbs, 7 prbs_position, 8 uq_ud_ext, 9 reserved, 0x64 PID_TORQUE_FLUX_TARGET torque
 *    [31:16] flux [15:0], 0x66 PID_VELOCITY_TARGET s32, 0x68 PID_POSITION_TARGET s32, 0x6A PID_VELOCITY_ACTUAL, 0x6B
 *    PID_POSITION_ACTUAL).
 *  - FW (firmware's own design, flags, help texts, replies of the help case): plain where the structure is the interface,
 *    "characterization:" (NEEDS REVIEW) where nothing specifies it (help texts, openloopspeedpwm set = openloopspeed set,
 *    mode 9, help reply for other command types, torque truncation to 16 bit).
 *  - The TMCL specifications do not apply to the text commands.
 *
 * Fixture: TmcDebugFixture (bridge on chip1, power on). Tests that need the driver state machine run the driver thread "TMC"
 * of the bridge with simulated time only (helper runDriver, same loop as TmcFixture::runThreadFor but the thread handle is
 * found by name because the bridge's driver is private). The state machine has finished its start-up (state Shutdown) after
 * about 1.2 s of simulated time; the helpers run 2 s. The ADC raw words of chip1 are set near mid scale first (calibration).
 *
 * Host vs target: no byte order dependence in the tests (the register file is word based). Values are chosen so that no float
 * conversion is involved. torque set goes through "int16_t" of the firmware: the negative torque test makes the driver shift a
 * negative int16_t left by 16 (defined since C++20, same on ARM GCC). The firmware narrows the int64_t command value to int16_t
 * (torque) and int32_t (pos, velocity, openloop speed) without a range check: implementation defined, modular on both
 * compilers; one characterization test documents it for torque; velocity and the open loop speed are tested with values beyond 16 bit (full s32 registers). enum conversion
 * MotionMode(int64_t): the enum has a fixed underlying type (uint8_t), so the conversion is defined (modulo 256), not UB.
 *
 * R5.1: openloopspeed set and openloopspeedpwm set are identical code; the registers show no difference, so the pwm variant is
 * a characterization test. R5.2: mode set -1 is a KNOWN_ISSUE (B-DTDC-1); mode 7 is a valid mode of the datasheet (DS 0x63: prbs_position_mode),
 * not a defect, although the help text of the firmware does not list it; mode 9 is reserved in the datasheet (characterization).
 * The datasheet lists the modes 10..15 (AGPI_A and PWM_I modes) as valid TMC4671 modes while the firmware rejects >= 10 (firmware
 * design: its help text lists 0..9 only). B-DTDC-1b: -256 and INT64_MIN alias to mode 0.
 * Defect candidate B-DTDC-2 (NEEDS REVIEW, not a KNOWN_ISSUE, charter L3): the first "torque set" from the Shutdown state only
 * requests the start (startMotor() is asynchronous, the state change happens in the driver thread) and, by D1 R62
 * (TMC4671_REQUIREMENTS.md: turn() does nothing unless the motor is ready and enabled), writes no torque target; the value is
 * not applied later. A deferred write would also be a legal reading, so the three torque cases document the behaviour. R5.3: the command side never reads Buf (cdcRcv is
 * TDF's file). R5.10: SPI_transmit_receive (declared, never defined) and tmcpids (declared, never used) are informational only.
 *
 * Not tested and why: the get commands read through the driver (getPos etc.) are tested only through the register they read;
 * the narrowing of pos values beyond 32 bit (same implementation defined conversion as torque); the emergency condition of
 * startMotor (not reachable through the public interface; power loss is tested); the order of startMotor and turn in torque set
 * from Shutdown (not observable: turn is a no-op while the motor is not enabled, equivalent mutant);
 * emergency / power loss behaviour of startMotor (D1 states tests); "torque set" from the Shutdown state: documented by
 * characterization cases (B-DTDC-2).
 */
#include "doctest.h"
#include "TmcDebugFixture.h"

#include <climits>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace {

using CommandHelpers::get;
using CommandHelpers::run;

constexpr uint8_t kRegPwm = 0x1A;
constexpr uint8_t kRegOpenloopAccel = 0x20;
constexpr uint8_t kRegOpenloopSpeed = 0x21;
constexpr uint8_t kRegOpenloopPhi = 0x23;
constexpr uint8_t kRegUqUdExt = 0x24;
constexpr uint8_t kRegPhiESel = 0x52;
constexpr uint8_t kRegPhiE = 0x53;
constexpr uint8_t kRegMode = 0x63;
constexpr uint8_t kRegTorqueFlux = 0x64;
constexpr uint8_t kRegVelTarget = 0x66;
constexpr uint8_t kRegPosTarget = 0x68;
constexpr uint8_t kRegVelActual = 0x6A;
constexpr uint8_t kRegPosActual = 0x6B;

const char *const kModeHelp = "stop=0,torque=1,velocity=2,position=3,prbsflux=4,prbstorque=5,prbsvelocity=6,uqudext=8,encminimove=9";

struct TdcFixture : TmcDebugFixture {
	/** Bridge on chip1, driver thread not run (only the constructor traffic happened), frame log cleared */
	std::unique_ptr<TMCDebugBridge> bridge() {
		chip1.setAdcRaw(0, 0x7FFF7FFF); // I1 [31:16] and I0 [15:0] at mid scale (DS 0x02 p77), as TmcFixture::reachShutdown
		auto b = makeBridge();
		chip1.clearLog();
		return b;
	}

	/** Runs the driver thread of the bridge for `ms` of simulated time */
	void runDriver(uint32_t ms) {
		TaskHandle_t t = HostRtos::findTask("TMC");
		REQUIRE(t != nullptr);
		if (!HostRtos::isResumable(t)) {
			HostRtos::makeResumable(t);
		}
		const uint64_t end = HostPlatform::nowUs() + uint64_t(ms) * 1000;
		while (HostPlatform::nowUs() < end) {
			uint64_t before = HostPlatform::nowUs();
			HostRtos::runUntilBlocked(t);
			if (HostPlatform::nowUs() == before) {
				HostPlatform::advanceMs(1);
			}
		}
	}

	/** Bridge whose driver finished its start-up (state Shutdown), frame log cleared */
	std::unique_ptr<TMCDebugBridge> idleBridge() {
		auto b = bridge();
		runDriver(2000);
		chip1.clearLog();
		return b;
	}

	/** Bridge whose driver is Running: "torque set 0" requests the start (the only public way), the thread applies it */
	std::unique_ptr<TMCDebugBridge> runningBridge() {
		auto b = idleBridge();
		REQUIRE(run(*b, "torque", CMDtype::set, 0) == CommandStatus::OK);
		runDriver(300);
		chip1.clearLog();
		return b;
	}

	/** Data of the write frames to `addr` */
	std::vector<uint32_t> writes(uint8_t addr) const { return chip1.writesTo(addr); }

	/** The low byte of MODE_MOTION (DS 0x63) */
	uint32_t modeByte() const { return chip1.reg(kRegMode) & 0xFF; }

	/** All frames (write flag, address, data) since the last clearLog */
	std::vector<std::pair<std::pair<bool, uint8_t>, uint32_t>> frames() const {
		std::vector<std::pair<std::pair<bool, uint8_t>, uint32_t>> f;
		for (const auto &x : chip1.log()) {
			f.push_back({{x.write, x.addr}, x.data});
		}
		return f;
	}
};

/** Executes a command with an optional reply list */
CommandStatus exec(TMCDebugBridge &b, const char *name, CMDtype type, int64_t val = 0, int64_t adr = 0,
				   std::vector<CommandReply> *replies = nullptr) {
	return run(b, name, type, val, replies, adr);
}

const std::vector<CMDtype> kAllTypes = {CMDtype::none, CMDtype::set, CMDtype::setat, CMDtype::get,
										CMDtype::getat, CMDtype::info, CMDtype::err};

/** Checks that every command type outside `valid` is answered with ERR, no reply and no register traffic */
void checkWrongTypes(TdcFixture &f, TMCDebugBridge &b, const char *name, const std::vector<CMDtype> &valid) {
	for (CMDtype t : kAllTypes) {
		bool isValid = false;
		for (CMDtype v : valid) {
			isValid = isValid || v == t;
		}
		if (isValid) {
			continue;
		}
		INFO("command " << name << " type " << static_cast<uint32_t>(t));
		std::vector<CommandReply> replies;
		CHECK(exec(b, name, t, 5, 6, &replies) == CommandStatus::ERR);
		CHECK(replies.empty());
		CHECK(f.chip1.log().empty());
	}
}

} // namespace

TEST_SUITE("TMCDebugBridge commands") {

// ---------------------------------------------------------------------------------------------------------------
// R1.1 registry
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TdcFixture, "[R1.1 registry] the seven commands are registered with their access flags") {
	auto b = bridge();
	struct Row {
		const char *name;
		uint32_t flags;
	};
	// the flags of registerCommands (TDB:36-42)
	const Row rows[] = {
		{"reg", CMDFLAG_GETADR | CMDFLAG_SETADR},	   {"torque", CMDFLAG_GET | CMDFLAG_SET},
		{"pos", CMDFLAG_GET | CMDFLAG_SET},			   {"openloopspeed", CMDFLAG_SETADR | CMDFLAG_SET},
		{"velocity", CMDFLAG_GET | CMDFLAG_SET},	   {"mode", CMDFLAG_GET | CMDFLAG_SET},
		{"openloopspeedpwm", CMDFLAG_SETADR | CMDFLAG_SET},
	};
	for (const Row &r : rows) {
		INFO("command " << r.name);
		CmdHandlerCommanddef *def = b->getCommandFromName(r.name);
		REQUIRE(def != nullptr);
		CHECK(def->flags == r.flags);
	}
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.1 registry] the seven commands have seven distinct ids and no other id is registered") {
	auto b = bridge();
	const char *names[] = {"reg", "torque", "pos", "openloopspeed", "velocity", "mode", "openloopspeedpwm"};
	std::vector<uint32_t> ids;
	for (const char *n : names) {
		CmdHandlerCommanddef *def = b->getCommandFromName(n);
		REQUIRE(def != nullptr);
		for (uint32_t other : ids) {
			CHECK(other != def->cmdId);
		}
		ids.push_back(def->cmdId);
		CHECK(b->getCommandFromId(def->cmdId) == def); // lookup by id finds the same entry
	}
	size_t found = 0;
	for (uint32_t id = 0; id < 64; id++) {
		if (b->getCommandFromId(id) != nullptr) {
			found++;
		}
	}
	CHECK(found == 7); // ids 0..63: only the seven commands
}

// ---------------------------------------------------------------------------------------------------------------
// R1.2 torque
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 torque] get returns the torque field (bits 31:16, signed) of PID_TORQUE_FLUX_TARGET") {
	auto b = bridge();
	// 0xFC18 = 64536 = 65536 - 1000 -> -1000 as s16 (DS 0x64: torque s16 [31:16]); the flux half 0x0005 is ignored
	chip1.setReg(kRegTorqueFlux, 0xFC180005u);
	CHECK(get(*b, "torque") == -1000);
	chip1.setReg(kRegTorqueFlux, 0x03E80000u); // 0x03E8 = 1000
	CHECK(get(*b, "torque") == 1000);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 torque] set while the driver is Running writes the torque to bits 31:16 and zero flux") {
	auto b = runningBridge();
	CHECK(exec(*b, "torque", CMDtype::set, 1000) == CommandStatus::OK);
	// 1000 = 0x3E8; torque in [31:16] -> 0x03E80000 = 65536000, flux 0 (DS 0x64)
	REQUIRE_FALSE(writes(kRegTorqueFlux).empty());
	CHECK(writes(kRegTorqueFlux).back() == 0x03E80000u);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 torque] set -1000 while the driver is Running writes 0xFC18 to bits 31:16") {
	auto b = runningBridge();
	CHECK(exec(*b, "torque", CMDtype::set, -1000) == CommandStatus::OK);
	// 65536 - 1000 = 64536 = 0xFC18 (two's complement s16), flux 0
	REQUIRE_FALSE(writes(kRegTorqueFlux).empty());
	CHECK(writes(kRegTorqueFlux).back() == 0xFC180000u);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 torque] set while the driver is Running selects the torque mode and does not start the motor again") {
	auto b = runningBridge();
	chip1.setReg(kRegPwm, 0); // a start (setPwm(PWM_FOC)) would write 7 to the low byte again
	chip1.clearLog();
	CHECK(exec(*b, "torque", CMDtype::set, 100) == CommandStatus::OK);
	CHECK(modeByte() == 1); // DS 0x63: 1 = torque_mode
	CHECK(writes(kRegPwm).empty());
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 torque] set while the driver is not Running starts the motor first (PWM_CHOP 7)") {
	auto b = idleBridge(); // state Shutdown
	CHECK(exec(*b, "torque", CMDtype::set, 1000) == CommandStatus::OK);
	// startMotor: setPwm(PWM_FOC) = updateReg(0x1A, 7) (DS 0x1A PWM_CHOP [7:0] 7 = FOC PWM)
	REQUIRE_FALSE(writes(kRegPwm).empty());
	CHECK((writes(kRegPwm).back() & 0xFF) == 7);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 torque] characterization: the first set from Shutdown writes only the start (PWM 0x1A = 7, mode register stop) and no torque target") {
	// B-DTDC-2 candidate; D1 R62: turn() does nothing while the motor is not ready, startMotor() only requests the Running state
	auto b = idleBridge();
	CHECK(exec(*b, "torque", CMDtype::set, 1000) == CommandStatus::OK);
	REQUIRE_FALSE(writes(kRegPwm).empty());
	CHECK((writes(kRegPwm).back() & 0xFF) == 7);
	REQUIRE_FALSE(writes(kRegMode).empty());
	CHECK((writes(kRegMode).back() & 0xFF) == 0); // stop (DS 0x63)
	CHECK(writes(kRegTorqueFlux).empty());
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 torque] characterization: the same set after the driver thread reached Running writes the torque target and the torque mode") {
	auto b = idleBridge();
	REQUIRE(exec(*b, "torque", CMDtype::set, 1000) == CommandStatus::OK);
	runDriver(300);
	chip1.clearLog();
	CHECK(exec(*b, "torque", CMDtype::set, 1000) == CommandStatus::OK);
	REQUIRE_FALSE(writes(kRegTorqueFlux).empty());
	CHECK(writes(kRegTorqueFlux).back() == 0x03E80000u); // 1000 << 16 (DS 0x64)
	CHECK(modeByte() == 1);								 // torque mode (DS 0x63)
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 torque] characterization: the value of the first set from Shutdown is not applied later by the driver thread") {
	auto b = idleBridge();
	REQUIRE(exec(*b, "torque", CMDtype::set, 1000) == CommandStatus::OK);
	chip1.clearLog();
	runDriver(2000);
	CHECK(writes(kRegTorqueFlux).empty());
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 torque] characterization: set 40000 is narrowed to 16 bit and writes the torque -25536 (0x9C40)") {
	auto b = runningBridge();
	CHECK(exec(*b, "torque", CMDtype::set, 40000) == CommandStatus::OK);
	// 40000 = 0x9C40; as s16 it is 40000 - 65536 = -25536; the command value is an int64_t, turn takes an int16_t
	REQUIRE_FALSE(writes(kRegTorqueFlux).empty());
	CHECK(writes(kRegTorqueFlux).back() == 0x9C400000u);
}

// ---------------------------------------------------------------------------------------------------------------
// R1.2 pos, velocity
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 pos] get returns the signed actual position PID_POSITION_ACTUAL") {
	auto b = bridge();
	chip1.setReg(kRegPosActual, 0xFFFFFFF6u); // 2^32 - 10 = -10 as s32
	CHECK(get(*b, "pos") == -10);
	chip1.setReg(kRegPosActual, 123456u);
	CHECK(get(*b, "pos") == 123456);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 pos] set 100000 writes PID_POSITION_TARGET and selects position mode") {
	auto b = bridge();
	CHECK(exec(*b, "pos", CMDtype::set, 100000) == CommandStatus::OK);
	CHECK(chip1.reg(kRegPosTarget) == 100000u);
	CHECK(modeByte() == 3); // DS 0x63: 3 = position_mode
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 pos] set -5 writes the two's complement 0xFFFFFFFB to PID_POSITION_TARGET") {
	auto b = bridge();
	CHECK(exec(*b, "pos", CMDtype::set, -5) == CommandStatus::OK);
	CHECK(chip1.reg(kRegPosTarget) == 0xFFFFFFFBu); // 2^32 - 5
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 pos] set writes the target once and does not start the motor") {
	auto b = bridge();
	CHECK(exec(*b, "pos", CMDtype::set, 777) == CommandStatus::OK);
	CHECK(writes(kRegPosTarget).size() == 1);
	CHECK(writes(kRegPwm).empty());
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 velocity] get returns the signed actual velocity PID_VELOCITY_ACTUAL") {
	auto b = bridge();
	chip1.setReg(kRegVelActual, 0xFFFFF830u); // 2^32 - 2000 = -2000 as s32
	CHECK(get(*b, "velocity") == -2000);
	chip1.setReg(kRegVelActual, 12345u);
	CHECK(get(*b, "velocity") == 12345);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 velocity] set -2000 writes PID_VELOCITY_TARGET and selects velocity mode") {
	auto b = bridge();
	CHECK(exec(*b, "velocity", CMDtype::set, -2000) == CommandStatus::OK);
	CHECK(chip1.reg(kRegVelTarget) == 0xFFFFF830u); // 2^32 - 2000
	CHECK(modeByte() == 2);							 // DS 0x63: 2 = velocity_mode
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 velocity] set writes the full 32 bit value: 100000, -100000, INT32_MAX and INT32_MIN") {
	auto b = bridge();
	// DS 0x66 PID_VELOCITY_TARGET s32. 100000 = 0x186A0; 2^32 - 100000 = 0xFFFE7960; INT32_MAX = 0x7FFFFFFF; INT32_MIN = 0x80000000
	const std::pair<int64_t, uint32_t> rows[] = {{100000, 0x000186A0u}, {-100000, 0xFFFE7960u}, {2147483647LL, 0x7FFFFFFFu}, {-2147483648LL, 0x80000000u}};
	for (const auto &r : rows) {
		INFO("velocity " << r.first);
		CHECK(exec(*b, "velocity", CMDtype::set, r.first) == CommandStatus::OK);
		CHECK(chip1.reg(kRegVelTarget) == r.second);
	}
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.6 openloopspeed] set writes the full 32 bit speed: 100000, -100000, INT32_MAX and INT32_MIN") {
	auto b = bridge();
	// DS 0x21 OPENLOOP_VELOCITY_TARGET s32, same hand values as the velocity case
	const std::pair<int64_t, uint32_t> rows[] = {{100000, 0x000186A0u}, {-100000, 0xFFFE7960u}, {2147483647LL, 0x7FFFFFFFu}, {-2147483648LL, 0x80000000u}};
	for (const auto &r : rows) {
		INFO("speed " << r.first);
		CHECK(exec(*b, "openloopspeed", CMDtype::set, r.first) == CommandStatus::OK);
		CHECK(chip1.reg(kRegOpenloopSpeed) == r.second);
		CHECK(chip1.reg(kRegOpenloopAccel) == 100u);
	}
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.2 velocity] set 5000 writes 5000 to PID_VELOCITY_TARGET once and does not start the motor") {
	auto b = bridge();
	CHECK(exec(*b, "velocity", CMDtype::set, 5000) == CommandStatus::OK);
	CHECK(chip1.reg(kRegVelTarget) == 5000u);
	CHECK(writes(kRegVelTarget).size() == 1);
	CHECK(writes(kRegPwm).empty());
}

// ---------------------------------------------------------------------------------------------------------------
// R1.2 / R1.6 openloopspeed, openloopspeedpwm
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TdcFixture, "[R1.6 openloopspeed] set 500 writes the speed 500 and the acceleration 100") {
	auto b = bridge();
	CHECK(exec(*b, "openloopspeed", CMDtype::set, 500) == CommandStatus::OK);
	CHECK(chip1.reg(kRegOpenloopSpeed) == 500u);	// DS 0x21 s32
	CHECK(chip1.reg(kRegOpenloopAccel) == 100u);	// DS 0x20 u32 (table: accel 100)
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.6 openloopspeed] set -500 writes the two's complement 0xFFFFFE0C as speed") {
	auto b = bridge();
	CHECK(exec(*b, "openloopspeed", CMDtype::set, -500) == CommandStatus::OK);
	CHECK(chip1.reg(kRegOpenloopSpeed) == 0xFFFFFE0Cu); // 2^32 - 500 = 4294966796 = 0xFFFFFE0C
	CHECK(chip1.reg(kRegOpenloopAccel) == 100u);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.6 openloopspeed] set writes only the two openloop registers: no start, no mode, no angle source") {
	auto b = bridge();
	CHECK(exec(*b, "openloopspeed", CMDtype::set, 500) == CommandStatus::OK);
	auto f = frames();
	REQUIRE(f.size() == 2);
	CHECK(f[0].first == std::make_pair(true, kRegOpenloopSpeed));
	CHECK(f[1].first == std::make_pair(true, kRegOpenloopAccel));
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.6 openloopspeed] setat with adr 3000 val 700 runs the open loop with flux 3000, speed 700 and acceleration 10") {
	auto b = bridge();
	chip1.setReg(kRegPhiE, 0x1234u);
	CHECK(exec(*b, "openloopspeed", CMDtype::setat, 700, 3000) == CommandStatus::OK);
	// runOpenLoop(ud=adr=3000, uq=0, speed=val=700, accel=10, torqueMode=true)
	CHECK(chip1.reg(kRegOpenloopSpeed) == 700u);
	CHECK(chip1.reg(kRegOpenloopAccel) == 10u);
	CHECK(chip1.reg(kRegTorqueFlux) == 3000u); // flux = ud = 0x0BB8 in [15:0], torque 0 (DS 0x64)
	CHECK(modeByte() == 1);						// torque mode (true)
	CHECK(chip1.reg(kRegPhiESel) == 2u);		// phi_e_openloop (DS 0x52)
	CHECK(chip1.reg(kRegOpenloopPhi) == 0x1234u); // start at the last PHI_E
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.6 openloopspeed] setat starts the motor (PWM_CHOP 7) and does not write UQ_UD_EXT") {
	auto b = bridge();
	CHECK(exec(*b, "openloopspeed", CMDtype::setat, 700, 3000) == CommandStatus::OK);
	REQUIRE_FALSE(writes(kRegPwm).empty());
	CHECK((writes(kRegPwm).back() & 0xFF) == 7);
	CHECK(writes(kRegUqUdExt).empty());
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.6 openloopspeedpwm] set 500 writes the speed 500 and the acceleration 100") {
	auto b = bridge();
	CHECK(exec(*b, "openloopspeedpwm", CMDtype::set, 500) == CommandStatus::OK);
	CHECK(chip1.reg(kRegOpenloopSpeed) == 500u);
	CHECK(chip1.reg(kRegOpenloopAccel) == 100u);
}

TEST_CASE_FIXTURE(TdcFixture, "[R5.1 openloopspeedpwm] characterization: set sends exactly the frames of openloopspeed set") {
	auto b = bridge();
	REQUIRE(exec(*b, "openloopspeed", CMDtype::set, 500) == CommandStatus::OK);
	auto plain = frames();
	chip1.clearLog();
	REQUIRE(exec(*b, "openloopspeedpwm", CMDtype::set, 500) == CommandStatus::OK);
	CHECK(frames() == plain);
	CHECK(plain.size() == 2);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.6 openloopspeedpwm] setat with adr 3000 val 700 runs the open loop in UQ_UD_EXT mode with ud 3000, speed 700 and acceleration 10") {
	auto b = bridge();
	chip1.setReg(kRegPhiE, 0x1234u);
	CHECK(exec(*b, "openloopspeedpwm", CMDtype::setat, 700, 3000) == CommandStatus::OK);
	// runOpenLoop(3000, 0, 700, 10, torqueMode=false): uqudext mode, setUdUq(3000, 0) = 3000 | (0 << 16)
	CHECK(chip1.reg(kRegOpenloopSpeed) == 700u);
	CHECK(chip1.reg(kRegOpenloopAccel) == 10u);
	CHECK(chip1.reg(kRegUqUdExt) == 3000u); // DS 0x24 UD [15:0] UQ [31:16]
	CHECK(modeByte() == 8);					// DS 0x63: 8 = uq_ud_ext
	CHECK(chip1.reg(kRegPhiESel) == 2u);
	CHECK(chip1.reg(kRegOpenloopPhi) == 0x1234u);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.6 openloopspeedpwm] setat starts the motor (PWM_CHOP 7) and does not write the torque target with the strength") {
	auto b = bridge();
	CHECK(exec(*b, "openloopspeedpwm", CMDtype::setat, 700, 3000) == CommandStatus::OK);
	REQUIRE_FALSE(writes(kRegPwm).empty());
	CHECK((writes(kRegPwm).back() & 0xFF) == 7);
	CHECK(writes(kRegTorqueFlux).empty());
}

// ---------------------------------------------------------------------------------------------------------------
// R1.5 reg
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TdcFixture, "[R1.5 reg] getat reads the register addressed by val and not by adr") {
	auto b = bridge();
	chip1.setReg(kRegPosTarget, 0xAABBCC11u);
	chip1.setReg(kRegVelTarget, 0x01020304u);
	std::vector<CommandReply> replies;
	CHECK(exec(*b, "reg", CMDtype::getat, kRegPosTarget, kRegVelTarget, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == 0xAABBCC11);
	CHECK(chip1.readsOf(kRegPosTarget) == 1);
	CHECK(chip1.readsOf(kRegVelTarget) == 0);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.5 reg] getat sends a single read frame") {
	auto b = bridge();
	std::vector<CommandReply> replies;
	CHECK(exec(*b, "reg", CMDtype::getat, kRegPosTarget, 0, &replies) == CommandStatus::OK);
	REQUIRE(chip1.log().size() == 1);
	CHECK_FALSE(chip1.log()[0].write);
	CHECK(chip1.log()[0].addr == kRegPosTarget);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.5 reg] setat writes the value val to the register adr") {
	auto b = bridge();
	CHECK(exec(*b, "reg", CMDtype::setat, 0x11223344, kRegPosTarget) == CommandStatus::OK);
	REQUIRE(chip1.log().size() == 1);
	CHECK(chip1.log()[0].write);
	CHECK(chip1.log()[0].addr == kRegPosTarget);
	CHECK(chip1.log()[0].data == 0x11223344u);
	CHECK(chip1.reg(kRegPosTarget) == 0x11223344u);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.5 reg] setat of a value with bit 31 set writes it unsigned") {
	auto b = bridge();
	CHECK(exec(*b, "reg", CMDtype::setat, 0x80000001LL, kRegVelTarget) == CommandStatus::OK);
	CHECK(chip1.reg(kRegVelTarget) == 0x80000001u);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.5 reg] setat answers no reply") {
	auto b = bridge();
	std::vector<CommandReply> replies;
	CHECK(exec(*b, "reg", CMDtype::setat, 1, kRegVelTarget, &replies) == CommandStatus::OK);
	CHECK(replies.empty());
}

// ---------------------------------------------------------------------------------------------------------------
// R1.3 wrong types, unknown id
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TdcFixture, "[R1.3 torque] every type except get and set is answered with ERR and no register traffic") {
	auto b = bridge();
	checkWrongTypes(*this, *b, "torque", {CMDtype::get, CMDtype::set});
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.3 pos] every type except get and set is answered with ERR and no register traffic") {
	auto b = bridge();
	checkWrongTypes(*this, *b, "pos", {CMDtype::get, CMDtype::set});
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.3 velocity] every type except get and set is answered with ERR and no register traffic") {
	auto b = bridge();
	checkWrongTypes(*this, *b, "velocity", {CMDtype::get, CMDtype::set});
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.3 openloopspeed] every type except set and setat is answered with ERR and no register traffic") {
	auto b = bridge();
	checkWrongTypes(*this, *b, "openloopspeed", {CMDtype::set, CMDtype::setat});
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.3 openloopspeedpwm] every type except set and setat is answered with ERR and no register traffic") {
	auto b = bridge();
	checkWrongTypes(*this, *b, "openloopspeedpwm", {CMDtype::set, CMDtype::setat});
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.3 reg] every type except getat and setat is answered with ERR and no register traffic") {
	auto b = bridge();
	checkWrongTypes(*this, *b, "reg", {CMDtype::getat, CMDtype::setat});
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.3 unknown id] an id that is not registered is answered with NOT_FOUND, no reply and no register traffic") {
	auto b = bridge();
	uint32_t unused = 0;
	while (b->getCommandFromId(unused) != nullptr) {
		unused++;
	}
	for (uint32_t id : {unused, unused + 1, 1000u, 0xFFFFFFFFu}) {
		for (CMDtype t : kAllTypes) {
			INFO("id " << id << " type " << static_cast<uint32_t>(t));
			ParsedCommand p;
			p.cmdId = id;
			p.type = t;
			p.val = 5;
			p.adr = 6;
			p.target = b.get();
			std::vector<CommandReply> replies;
			CHECK(b->command(p, replies) == CommandStatus::NOT_FOUND);
			CHECK(replies.empty());
		}
	}
	CHECK(chip1.log().empty());
}

// ---------------------------------------------------------------------------------------------------------------
// R1.4 mode
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TdcFixture, "[R1.4 mode] get returns the low byte of MODE_MOTION") {
	auto b = bridge();
	chip1.setReg(kRegMode, 0x80000103u); // low byte 3 = position_mode (DS 0x63 [7:0]); other bits are other fields
	CHECK(get(*b, "mode") == 3);
	chip1.setReg(kRegMode, 0x00000008u);
	CHECK(get(*b, "mode") == 8);
	chip1.setReg(kRegMode, 0x000000FFu); // low byte >= 0x80: the reply is the unsigned byte, not a negative number
	CHECK(get(*b, "mode") == 255);
	chip1.setReg(kRegMode, 0x00000080u);
	CHECK(get(*b, "mode") == 128);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.4 mode] get answers one INT reply and writes nothing") {
	auto b = bridge();
	std::vector<CommandReply> replies;
	CHECK(exec(*b, "mode", CMDtype::get, 0, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().type == CommandReplyType::INT);
	for (const auto &f : chip1.log()) {
		CHECK_FALSE(f.write);
	}
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.4 mode] set 0 to 8 writes the mode to MODE_MOTION and get returns it") {
	auto b = bridge();
	for (int64_t v = 0; v <= 8; v++) {
		INFO("mode " << v);
		CHECK(exec(*b, "mode", CMDtype::set, v) == CommandStatus::OK);
		CHECK(modeByte() == static_cast<uint32_t>(v)); // DS 0x63: 0 stopped .. 7 prbs_position, 8 uq_ud_ext (7 is a valid mode)
		CHECK(get(*b, "mode") == v);
	}
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.4 mode] set 3 starts the motor (PWM_CHOP 7) and answers no reply") {
	auto b = bridge();
	std::vector<CommandReply> replies;
	CHECK(exec(*b, "mode", CMDtype::set, 3, 0, &replies) == CommandStatus::OK);
	CHECK(replies.empty());
	REQUIRE_FALSE(writes(kRegPwm).empty());
	CHECK((writes(kRegPwm).back() & 0xFF) == 7);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.4 mode] characterization: set 9 (reserved in the datasheet, encminimove in the firmware) is accepted") {
	auto b = bridge();
	CHECK(exec(*b, "mode", CMDtype::set, 9) == CommandStatus::OK);
	CHECK(modeByte() == 9u);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.4 mode] set 10 and above answers one string reply with OK") {
	auto b = bridge();
	for (int64_t v : {10, 11, 255, 256, 1000}) {
		INFO("mode " << v);
		std::vector<CommandReply> replies;
		CHECK(exec(*b, "mode", CMDtype::set, v, 0, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		CHECK(replies.front().type == CommandReplyType::STRING);
		CHECK_FALSE(replies.front().reply.empty());
	}
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.4 mode] set 10 and above does not change the mode and does not start the motor") {
	auto b = bridge();
	chip1.setReg(kRegMode, 0x80000003u);
	for (int64_t v : {10, 11, 255, 256, 1000}) {
		INFO("mode " << v);
		REQUIRE(exec(*b, "mode", CMDtype::set, v) == CommandStatus::OK);
	}
	CHECK(chip1.reg(kRegMode) == 0x80000003u);
	CHECK(chip1.log().empty());
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.4 mode] characterization: the help reply text for an out-of-range mode") {
	auto b = bridge();
	std::vector<CommandReply> replies;
	REQUIRE(exec(*b, "mode", CMDtype::set, 10, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().reply == kModeHelp);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.4 mode] characterization: a command type other than get and set answers the help text with OK and no register traffic") {
	auto b = bridge();
	for (CMDtype t : {CMDtype::none, CMDtype::setat, CMDtype::getat, CMDtype::info, CMDtype::err}) {
		INFO("type " << static_cast<uint32_t>(t));
		std::vector<CommandReply> replies;
		CHECK(exec(*b, "mode", t, 3, 0, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		CHECK(replies.front().reply == kModeHelp);
	}
	CHECK(chip1.log().empty());
}

TEST_CASE_FIXTURE(TdcFixture, "[R5.2 mode] set -256 and INT64_MIN are rejected like an out-of-range value: help text, mode unchanged" *
								  KNOWN_ISSUE("B-DTDC-1b: negative values pass the range check; -256 and INT64_MIN alias to mode 0 (modulo 256), so the motor is silently stopped")) {
	auto b = bridge();
	chip1.setReg(kRegMode, 0x80000003u);
	for (int64_t v : {int64_t(-256), INT64_MIN}) {
		INFO("mode " << v);
		std::vector<CommandReply> replies;
		CHECK(exec(*b, "mode", CMDtype::set, v, 0, &replies) == CommandStatus::OK);
		CHECK(replies.size() == 1); // fails: no reply
		CHECK(modeByte() == 3u);	// fails: 0 (stop)
	}
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.4 mode] set while the driver is Running answers OK, writes the mode and the PWM start again") {
	auto b = runningBridge();
	CHECK(exec(*b, "mode", CMDtype::set, 2) == CommandStatus::OK);
	CHECK(modeByte() == 2u);
	REQUIRE_FALSE(writes(kRegPwm).empty()); // startMotor does not check the state before setPwm (power is on)
	CHECK((writes(kRegPwm).back() & 0xFF) == 7);
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.4 mode] set while the supply is lost answers OK and writes no register (startMotor skips PWM and mode)") {
	auto b = bridge();
	powerOff(); // hasPower() false
	chip1.clearLog();
	chip1.setReg(kRegMode, 0x80000003u);
	CHECK(exec(*b, "mode", CMDtype::set, 2) == CommandStatus::OK);
	CHECK(chip1.reg(kRegMode) == 0x80000003u);
	for (const auto &f : chip1.log()) {
		CHECK_FALSE(f.write);
	}
}

TEST_CASE_FIXTURE(TdcFixture, "[R5.2 mode] set -1 is rejected like an out-of-range value: help text, mode unchanged" *
								  KNOWN_ISSUE("B-DTDC-1: the range check cmd.val < 10 is true for negative values, so -1 reaches MotionMode(-1) = 255 (modulo 256), which is written to MODE_MOTION and the motor is started")) {
	auto b = bridge();
	chip1.setReg(kRegMode, 0x80000003u); // position mode
	std::vector<CommandReply> replies;
	CHECK(exec(*b, "mode", CMDtype::set, -1, 0, &replies) == CommandStatus::OK);
	CHECK(replies.size() == 1);			// help text like for 10 (fails: no reply)
	CHECK(modeByte() == 3u);			// unchanged (fails: 255)
	CHECK(writes(kRegPwm).empty());		// motor not started (fails)
}

// ---------------------------------------------------------------------------------------------------------------
// R1.7 help strings
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TdcFixture, "[R1.7 help] characterization: getHelpstring is the fixed TMCL-IDE text") {
	auto b = bridge();
	CHECK(b->getHelpstring() ==
		  "Compatible with TMCL-IDE. To use the FOC modes position, velocity and torque you first must manually select the encoder and wait for alignment.");
}

TEST_CASE_FIXTURE(TdcFixture, "[R1.7 help] characterization: the help text of each command") {
	auto b = bridge();
	struct Row {
		const char *name;
		const char *help;
	};
	const Row rows[] = {
		{"reg", "Read or write a TMC register at adr"},
		{"torque", "Change torque and enter torque mode"},
		{"pos", "Change pos and enter pos mode"},
		{"openloopspeed", "Move openloop (current controlled). adr=strength;val=speed"},
		{"velocity", "Change velocity and enter velocity mode"},
		{"mode", "Change motion mode"},
		{"openloopspeedpwm", "Move openloop raw PWM. adr=strength;val=speed"},
	};
	for (const Row &r : rows) {
		INFO("command " << r.name);
		CmdHandlerCommanddef *def = b->getCommandFromName(r.name);
		REQUIRE(def != nullptr);
		REQUIRE(def->helpstring != nullptr);
		CHECK(std::string(def->helpstring) == r.help);
	}
}

} // TEST_SUITE
