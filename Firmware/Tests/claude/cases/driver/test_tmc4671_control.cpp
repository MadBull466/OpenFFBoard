/*
 * TMC4671 driver, group G5: control and readouts (requirements R60-R68 of
 * Tests/claude/charters/driver/TMC4671_REQUIREMENTS.md, lead decisions L1-L18).
 *
 * Datasheet citations "DS 0xNN" = TMC4671-LA datasheet Rev 2.07, section 7.2 register map.
 * Host vs target: the shifts `torque << 16` of a negative int16_t in the driver are signed shifts of a promoted int
 * (defined since C++20, identical on ARM GCC); the voltage / temperature readouts use float and double, the float
 * to int32 conversions are therefore compared with a tolerance of 2 mV / 0.1 C and the values chosen are in range.
 */
#include "doctest.h"
#include "TmcFixture.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace {

/** Hardware constants of the default board config (TMC4671.h vmScaler default), used to derive the expected mV by hand */
constexpr double kVmScalerV = (2.5 / 32767.0) * ((1.5 + 71.5) / 1.5); // volts per count, = 3.7134 mV per count (HD below)

/** Resets the frame log of the chip, so the next checks see only the frames of the call under test */
void clearLogs(FakeTmc4671 &chip) { chip.clearLog(); }

} // namespace

TEST_SUITE("TMC4671 control") {

// ---------------------------------------------------------------------------------------------------------------
// R60 setFluxTorque
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R60] setFluxTorque(-1, -1) writes 0xFFFFFFFF to PID_TORQUE_FLUX_TARGET") {
	auto tmc = makeTmc1();
	tmc->setFluxTorque(-1, -1);
	// DS 0x64 p97: flux [15:0], torque [31:16]; HD: both halves 0xFFFF
	CHECK(chip1.reg(0x64) == 0xFFFFFFFFu);
}

TEST_CASE_FIXTURE(TmcFixture, "[R60] setFluxTorque(100, -200) writes flux 0x0064 to bits 15:0 and torque 0xFF38 to bits 31:16") {
	auto tmc = makeTmc1();
	tmc->setFluxTorque(100, -200);
	// HD: -200 = 0x10000 - 200 = 65336 = 0xFF38; flux 100 = 0x0064 -> 0xFF380064
	CHECK(chip1.reg(0x64) == 0xFF380064u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R60] setFluxTorque(-300, 1000) writes flux 0xFED4 and torque 0x03E8") {
	auto tmc = makeTmc1();
	tmc->setFluxTorque(-300, 1000);
	// HD: -300 = 65536 - 300 = 65236 = 0xFED4; 1000 = 0x03E8
	CHECK(chip1.reg(0x64) == 0x03E8FED4u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R60] setFluxTorque leaves the motion mode of a driver in velocity mode in torque mode (MODE_MOTION = 1)") {
	auto tmc = makeTmc1();
	tmc->setTargetVelocity(0); // velocity mode, DS 0x63: 2
	REQUIRE((chip1.reg(0x63) & 0xFF) == 2u);
	tmc->setFluxTorque(0, 100);
	CHECK((chip1.reg(0x63) & 0xFF) == 1u); // DS 0x63 p97: 1 = torque mode
}

// ---------------------------------------------------------------------------------------------------------------
// R61 setTorque / setFlux / getTorque / getFlux
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R61] setTorque(-5) replaces only bits 31:16 of PID_TORQUE_FLUX_TARGET") {
	auto tmc = makeTmc1();
	chip1.setReg(0x64, 0x1234ABCD);
	tmc->setTorque(-5);
	// HD: -5 = 0xFFFB; the flux half 0xABCD is kept -> 0xFFFBABCD
	CHECK(chip1.reg(0x64) == 0xFFFBABCDu);
}

TEST_CASE_FIXTURE(TmcFixture, "[R61] setFlux(-2) replaces only bits 15:0 of PID_TORQUE_FLUX_TARGET") {
	auto tmc = makeTmc1();
	chip1.setReg(0x64, 0x1234ABCD);
	tmc->setFlux(-2);
	// HD: -2 = 0xFFFE; the torque half 0x1234 is kept -> 0x1234FFFE
	CHECK(chip1.reg(0x64) == 0x1234FFFEu);
}

TEST_CASE_FIXTURE(TmcFixture, "[R61] setTorque switches the motion mode to torque") {
	auto tmc = makeTmc1();
	tmc->setTargetVelocity(0);
	REQUIRE((chip1.reg(0x63) & 0xFF) == 2u);
	tmc->setTorque(10);
	CHECK((chip1.reg(0x63) & 0xFF) == 1u); // DS 0x63: 1 = torque
}

TEST_CASE_FIXTURE(TmcFixture, "[R61] setFlux switches the motion mode to torque") {
	auto tmc = makeTmc1();
	tmc->setTargetVelocity(0);
	REQUIRE((chip1.reg(0x63) & 0xFF) == 2u);
	tmc->setFlux(10);
	CHECK((chip1.reg(0x63) & 0xFF) == 1u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R61] getTorque decodes bits 31:16 as a signed 16 bit value") {
	auto tmc = makeTmc1();
	chip1.setReg(0x64, 0xFF380064); // HD: torque 0xFF38 = -200
	CHECK(tmc->getTorque() == -200);
	chip1.setReg(0x64, 0x7FFF0000);
	CHECK(tmc->getTorque() == 32767);
}

TEST_CASE_FIXTURE(TmcFixture, "[R61] getFlux decodes bits 15:0 as a positive signed 16 bit value" *
								  KNOWN_ISSUE("B-T11: getFlux returns 'readReg(0x64) && 0xffff', a bool (1), not the flux")) {
	auto tmc = makeTmc1();
	chip1.setReg(0x64, 0x00000064); // HD: flux 100
	CHECK(tmc->getFlux() == 100);
}

TEST_CASE_FIXTURE(TmcFixture, "[R61] getFlux decodes bits 15:0 as a negative signed 16 bit value" *
								  KNOWN_ISSUE("B-T11: getFlux returns 'readReg(0x64) && 0xffff', a bool (1), not the flux")) {
	auto tmc = makeTmc1();
	chip1.setReg(0x64, 0x0000FFF6); // HD: 0xFFF6 = 65536 - 10 -> -10
	CHECK(tmc->getFlux() == -10);
}

// ---------------------------------------------------------------------------------------------------------------
// R62 turn
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R62] turn writes nothing while the motor is not ready") {
	auto tmc = makeTmc1();
	REQUIRE_FALSE(tmc->motorReady());
	clearLogs(chip1);
	tmc->turn(1000);
	CHECK(chip1.writesTo(0x64).empty());
}

TEST_CASE_FIXTURE(TmcFixture, "[R62] turn writes nothing after the motor was disabled with an emergency stop") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	// The two conditions (ready, enabled) cannot be separated through the public API: emergencyStop(false) clears both.
	tmc->emergencyStop(false); // motorEnabledRequested = false and the state leaves Running
	clearLogs(chip1);
	tmc->turn(1000);
	CHECK(chip1.writesTo(0x64).empty());
}

TEST_CASE_FIXTURE(TmcFixture, "[R62] turn(1000) of a running motor writes torque 1000 and flux 0") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	clearLogs(chip1);
	tmc->turn(1000);
	// DS 0x64 p97: torque [31:16] = 0x03E8, flux [15:0] = 0 (no flux offset configured: maxOffsetFlux = 0, idleFlux = 0)
	REQUIRE(chip1.writesTo(0x64).size() == 1);
	CHECK(chip1.writesTo(0x64).back() == 0x03E80000u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R62] turn(-1000) of a running motor writes torque 0xFC18") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	clearLogs(chip1);
	tmc->turn(-1000);
	// HD: -1000 = 65536 - 1000 = 64536 = 0xFC18
	REQUIRE(chip1.writesTo(0x64).size() == 1);
	CHECK(chip1.writesTo(0x64).back() == 0xFC180000u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R62] turn(1000) with invertForce set writes the negated torque 0xFC18") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	tmc->conf.invertForce = true; // conf is public (TMC4671.h:412); the command restores false unless combineEncoder is set
	clearLogs(chip1);
	tmc->turn(1000);
	// HD: -1000 = 0xFC18
	REQUIRE(chip1.writesTo(0x64).size() == 1);
	CHECK(chip1.writesTo(0x64).back() == 0xFC180000u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R62] turn(1000) with a reversed external encoder writes the negated torque 0xFC18") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	// conf is public (TMC4671.h:412); direct write because setEncoderType(ext) needs hardware support and an encoder object
	tmc->conf.motconf.enctype = EncoderType_TMC::ext;
	tmc->conf.encoderReversed = true;
	clearLogs(chip1);
	tmc->turn(1000);
	REQUIRE(chip1.writesTo(0x64).size() == 1);
	CHECK(chip1.writesTo(0x64).back() == 0xFC180000u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R62] characterization: turn(300) with fluxoffset 500 writes flux -300 (field weakening)") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	REQUIRE(CommandHelpers::run(*tmc, "fluxoffset", CMDtype::set, 500) == CommandStatus::OK);
	clearLogs(chip1);
	tmc->turn(300);
	// FW policy: flux = idleFlux(0) - min(|power|, fluxoffset) = -300 = 0xFED4; torque 300 = 0x012C
	REQUIRE(chip1.writesTo(0x64).size() == 1);
	CHECK(chip1.writesTo(0x64).back() == 0x012CFED4u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R62] characterization: turn(1000) with fluxoffset 500 limits the flux to -500") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	REQUIRE(CommandHelpers::run(*tmc, "fluxoffset", CMDtype::set, 500) == CommandStatus::OK);
	clearLogs(chip1);
	tmc->turn(1000);
	// FW policy: flux = -min(1000, 500) = -500 = 65536 - 500 = 0xFE0C; torque 0x03E8
	REQUIRE(chip1.writesTo(0x64).size() == 1);
	CHECK(chip1.writesTo(0x64).back() == 0x03E8FE0Cu);
}

TEST_CASE_FIXTURE(TmcFixture, "[R62] characterization: turn(-300) with fluxoffset 500 writes flux -300 and torque -300") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	REQUIRE(CommandHelpers::run(*tmc, "fluxoffset", CMDtype::set, 500) == CommandStatus::OK);
	clearLogs(chip1);
	tmc->turn(-300);
	// FW policy uses |power|: flux -300 = 0xFED4, torque -300 = 0xFED4
	REQUIRE(chip1.writesTo(0x64).size() == 1);
	CHECK(chip1.writesTo(0x64).back() == 0xFED4FED4u);
}

// ---------------------------------------------------------------------------------------------------------------
// R63 flux dissipation. The formula is firmware only (characterization).
// vDiff = getIntV() - getExtV() is read from voltagesense (the input of the driver), then the expected flux is
// derived by hand: flux = clip(vDiff * 0.5 * torqueLimit * 0.0002, 0, torqueLimit); with torqueLimit 10000 the
// product is vDiff * 0.5 * 10000 * 0.0002 = vDiff * 1.0.
// ---------------------------------------------------------------------------------------------------------------

namespace {
struct FluxBrakeSetup {
	static int32_t vDiff() { return getIntV() - getExtV(); }
};
} // namespace

TEST_CASE_FIXTURE(TmcFixture, "[R63] characterization: with fluxbrake on and 2 V over the external supply turn writes the dissipation flux instead of the field weakening flux") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	tmc->setTorqueLimit(10000);
	REQUIRE(CommandHelpers::run(*tmc, "fluxbrake", CMDtype::set, 1) == CommandStatus::OK);
	setSupply(24000, 26000);
	const int32_t vDiff = FluxBrakeSetup::vDiff();
	REQUIRE(vDiff > 1900);
	REQUIRE(vDiff < 2100);
	clearLogs(chip1);
	tmc->turn(500);
	REQUIRE(chip1.writesTo(0x64).size() == 1);
	const uint32_t w = chip1.writesTo(0x64).back();
	// HD: flux = vDiff * 0.5 * 10000 * 0.0002 = vDiff (+-1 for truncation of the double product); torque 500 = 0x01F4
	CHECK((w >> 16) == 0x01F4u);
	CHECK(std::abs(int32_t(w & 0xFFFF) - vDiff) <= 1);
}

TEST_CASE_FIXTURE(TmcFixture, "[R63] characterization: the dissipation flux scales with the torque limit (0.15 per mV at limit 1500)") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	tmc->setTorqueLimit(1500);
	REQUIRE(CommandHelpers::run(*tmc, "fluxbrake", CMDtype::set, 1) == CommandStatus::OK);
	setSupply(24000, 26000);
	REQUIRE(FluxBrakeSetup::vDiff() > 1900);
	clearLogs(chip1);
	tmc->turn(500);
	// HD: 0.5 * 1500 * 0.0002 = 0.15, vDiff ~ 2000 -> ~300 (< 1500, not clipped; clipping is tested below)
	REQUIRE(chip1.writesTo(0x64).size() == 1);
	const int32_t flux = int32_t(chip1.writesTo(0x64).back() & 0xFFFF);
	const double expect = FluxBrakeSetup::vDiff() * 0.5 * 1500 * 0.0002;
	CHECK(std::abs(flux - int32_t(expect)) <= 1);
}

TEST_CASE_FIXTURE(TmcFixture, "[R63] characterization: a dissipation flux above the torque limit is clipped to the limit") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	tmc->setTorqueLimit(5000);
	REQUIRE(CommandHelpers::run(*tmc, "fluxbrake", CMDtype::set, 1) == CommandStatus::OK);
	setSupply(24000, 40000); // vDiff ~ 16 V
	const int32_t vDiff = FluxBrakeSetup::vDiff();
	REQUIRE(vDiff > 15000);
	clearLogs(chip1);
	tmc->turn(500);
	// HD: vDiff * 0.5 * 5000 * 0.0002 = vDiff * 0.5 ~ 8000 > 5000 -> clipped to the limit 5000 = 0x1388
	REQUIRE(chip1.writesTo(0x64).size() == 1);
	CHECK((chip1.writesTo(0x64).back() & 0xFFFF) == 5000u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R63] characterization: with fluxbrake off the dissipation flux is not used") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	tmc->setTorqueLimit(10000);
	setSupply(24000, 26000);
	REQUIRE(FluxBrakeSetup::vDiff() > 1900);
	clearLogs(chip1);
	tmc->turn(500);
	REQUIRE(chip1.writesTo(0x64).size() == 1);
	CHECK(chip1.writesTo(0x64).back() == 0x01F40000u); // flux 0 (no offset), torque 500
}

TEST_CASE_FIXTURE(TmcFixture, "[R63] characterization: with fluxbrake on and only 0.5 V over the external supply the dissipation flux is not used") {
	auto tmc = makeTmc1();
	reachRunning(*tmc);
	tmc->setTorqueLimit(10000);
	REQUIRE(CommandHelpers::run(*tmc, "fluxbrake", CMDtype::set, 1) == CommandStatus::OK);
	setSupply(24000, 24500);
	const int32_t vDiff = FluxBrakeSetup::vDiff();
	REQUIRE(vDiff > 0);
	REQUIRE(vDiff <= 1000); // threshold: vDiff must exceed 1000 mV (fluxDissipationLimit)
	clearLogs(chip1);
	tmc->turn(500);
	REQUIRE(chip1.writesTo(0x64).size() == 1);
	CHECK(chip1.writesTo(0x64).back() == 0x01F40000u);
}

// ---------------------------------------------------------------------------------------------------------------
// R64 actual torque / flux
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R64] getActualTorqueFlux returns flux from bits 15:0 first and torque from bits 31:16 second") {
	auto tmc = makeTmc1();
	chip1.setReg(0x69, 0x012CFFFB); // DS 0x69 p97; HD: torque 0x012C = 300, flux 0xFFFB = -5
	auto ft = tmc->getActualTorqueFlux();
	CHECK(ft.first == -5);
	CHECK(ft.second == 300);
}

TEST_CASE_FIXTURE(TmcFixture, "[R64] getActualTorqueFlux decodes a negative torque and a positive flux") {
	auto tmc = makeTmc1();
	chip1.setReg(0x69, 0xFED40064); // HD: torque 0xFED4 = -300, flux 0x0064 = 100
	auto ft = tmc->getActualTorqueFlux();
	CHECK(ft.first == 100);
	CHECK(ft.second == -300);
}

TEST_CASE_FIXTURE(TmcFixture, "[R64] the acttrq command replies torque as value and flux as address") {
	auto tmc = makeTmc1();
	chip1.setReg(0x69, 0xFED4FFFB); // HD: torque -300, flux -5
	std::vector<CommandReply> replies;
	REQUIRE(CommandHelpers::run(*tmc, "acttrq", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == -300);
	CHECK(replies[0].adr == -5);
}

// ---------------------------------------------------------------------------------------------------------------
// R65 position and velocity
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R65] getPos reads PID_POSITION_ACTUAL as a signed 32 bit value") {
	auto tmc = makeTmc1();
	chip1.setReg(0x6B, 0xFFFFFF38); // DS 0x6B p98; HD: -200
	CHECK(tmc->getPos() == -200);
	chip1.setReg(0x6B, 0x7FFFFFFF);
	CHECK(tmc->getPos() == 2147483647);
}

TEST_CASE_FIXTURE(TmcFixture, "[R65] setTmcPos writes the value to PID_POSITION_ACTUAL") {
	auto tmc = makeTmc1();
	tmc->setTmcPos(-1000);
	// HD: -1000 = 0xFFFFFC18
	REQUIRE(chip1.writesTo(0x6B).size() == 1);
	CHECK(chip1.writesTo(0x6B).back() == 0xFFFFFC18u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R65] setTargetPos writes PID_POSITION_TARGET and selects position mode") {
	auto tmc = makeTmc1();
	tmc->setTargetPos(-123456);
	// HD: -123456 = 0x100000000 - 123456 = 0xFFFE1DC0
	REQUIRE(chip1.writesTo(0x68).size() == 1);
	CHECK(chip1.writesTo(0x68).back() == 0xFFFE1DC0u);
	CHECK((chip1.reg(0x63) & 0xFF) == 3u); // DS 0x63: 3 = position mode
}

TEST_CASE_FIXTURE(TmcFixture, "[R65] getTargetPos reads PID_POSITION_TARGET") {
	auto tmc = makeTmc1();
	chip1.setReg(0x68, 0xFFFE1DC0);
	CHECK(tmc->getTargetPos() == -123456);
}

TEST_CASE_FIXTURE(TmcFixture, "[R65] setTargetVelocity writes PID_VELOCITY_TARGET and selects velocity mode") {
	auto tmc = makeTmc1();
	tmc->setTargetVelocity(-2000);
	// HD: -2000 = 0xFFFFF830
	REQUIRE(chip1.writesTo(0x66).size() == 1);
	CHECK(chip1.writesTo(0x66).back() == 0xFFFFF830u);
	CHECK((chip1.reg(0x63) & 0xFF) == 2u); // DS 0x63: 2 = velocity mode
}

TEST_CASE_FIXTURE(TmcFixture, "[R65] getTargetVelocity reads PID_VELOCITY_TARGET and getVelocity reads PID_VELOCITY_ACTUAL") {
	auto tmc = makeTmc1();
	chip1.setReg(0x66, 0xFFFFF830); // -2000
	chip1.setReg(0x6A, 0x00000FA0); // 4000
	CHECK(tmc->getTargetVelocity() == -2000);
	CHECK(tmc->getVelocity() == 4000);
}

TEST_CASE_FIXTURE(TmcFixture, "[R65] reachedPosition is true when the distance to the target is below the tolerance") {
	auto tmc = makeTmc1();
	chip1.setReg(0x68, 1000);
	chip1.setReg(0x6B, 995);
	CHECK(tmc->reachedPosition(10)); // |1000 - 995| = 5 < 10
}

TEST_CASE_FIXTURE(TmcFixture, "[R65] reachedPosition is false when the distance equals the tolerance") {
	auto tmc = makeTmc1();
	chip1.setReg(0x68, 1000);
	chip1.setReg(0x6B, 995);
	CHECK_FALSE(tmc->reachedPosition(5)); // spec: |target - actual| < tol, 5 < 5 is false
}

TEST_CASE_FIXTURE(TmcFixture, "[R65] reachedPosition works for negative positions and a target above the position") {
	auto tmc = makeTmc1();
	chip1.setReg(0x68, uint32_t(int32_t(-1000)));
	chip1.setReg(0x6B, uint32_t(int32_t(-1004)));
	CHECK(tmc->reachedPosition(5));	  // |-1000 - (-1004)| = 4 < 5
	CHECK_FALSE(tmc->reachedPosition(4)); // 4 < 4 is false
}

// ---------------------------------------------------------------------------------------------------------------
// R66 rampFlux
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R66] rampFlux from flux 0 to 100 in 20 ms writes 10, 20, ... 100 and ends at the target with torque 0") {
	auto tmc = makeTmc1();
	chip1.setReg(0x64, 0);
	tmc->rampFlux(100, 20);
	// HD: step = (100 - 0) / max(1, 20/2) = 10; ten writes, flux 10, 20, ..., 100, torque 0
	const auto w = chip1.writesTo(0x64);
	REQUIRE(w.size() == 10);
	for (size_t i = 0; i < w.size(); i++) {
		CHECK(w[i] == uint32_t((i + 1) * 10));
	}
}

TEST_CASE_FIXTURE(TmcFixture, "[R66] rampFlux from a positive flux down to a lower target ends at the target") {
	auto tmc = makeTmc1();
	chip1.setReg(0x64, 100);
	tmc->rampFlux(40, 20);
	// HD: step = (40 - 100) / 10 = -6; flux 94, 88, ... down; (100-40)/6 = 10 steps ending at exactly 40
	const auto w = chip1.writesTo(0x64);
	REQUIRE_FALSE(w.empty());
	CHECK(w.back() == 40u);
	CHECK(w.size() == 10);
}

TEST_CASE_FIXTURE(TmcFixture, "[R66] rampFlux stops at the target when the distance is not a multiple of the step" *
								  KNOWN_ISSUE("B-T23: rampFlux stops short of the target when the distance is not a multiple of the step")) {
	auto tmc = makeTmc1();
	chip1.setReg(0x64, 0);
	tmc->rampFlux(105, 20);
	// HD: step = 105 / 10 = 10; the loop stops when 105 - flux < 10, i.e. at 100. Intended: the last flux is 105.
	const auto w = chip1.writesTo(0x64);
	REQUIRE_FALSE(w.empty());
	CHECK((w.back() & 0xFFFF) == 105u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R66] rampFlux from a negative flux up to a positive target ends at the target" *
								  KNOWN_ISSUE("B-T12: rampFlux keeps the start flux as uint16_t, a negative flux (0xFFF6) becomes 65526")) {
	auto tmc = makeTmc1();
	chip1.setReg(0x64, 0xFFF6); // flux -10 (HD: 65536 - 10), torque 0
	tmc->rampFlux(100, 20);
	// Intended: ramp from -10 to 100, the last written flux is the target 100 (clipped at 0 from below by the driver).
	const auto w = chip1.writesTo(0x64);
	REQUIRE_FALSE(w.empty());
	CHECK((w.back() & 0xFFFF) == 100u);
}

// ---------------------------------------------------------------------------------------------------------------
// R67 supply voltage readout
// ---------------------------------------------------------------------------------------------------------------

// HD of the scaler: vmScaler = 2.5 / 32767 * (1.5 + 71.5) / 1.5 = 7.6296e-5 * 48.6667 = 3.7134e-3 V per count.
// raw = 0x7FFF + 1000 (adcOffset) + 6000 -> 6000 counts -> 6000 * 3.7134e-3 = 22.2805 V; exact: 6000*2.5/32767 =
// 0.457791 ; * 48.66667 = 22.27978 V -> 22279.8 mV; truncated float result 22279 or 22280 -> tolerance 2 mV.

TEST_CASE_FIXTURE(TmcFixture, "[R67] getTmcVM selects ADC_RAW_ADDR 1 before it reads ADC_RAW_DATA") {
	auto tmc = makeTmc1();
	tmc->conf.hwconf.adcOffset = 1000; // conf is public; no hardware version has this offset with the default scaler combination under test
	chip1.setAdcRaw(1, 0x7FFF + 1000 + 6000);
	clearLogs(chip1);
	tmc->getTmcVM();
	const auto &log = chip1.log();
	REQUIRE(log.size() >= 2);
	// DS 0x02/0x03 p77: write 0x03 = 1, then read 0x02
	CHECK(log[0].write);
	CHECK(log[0].addr == 0x03);
	CHECK(log[0].data == 1u);
	CHECK_FALSE(log[1].write);
	CHECK(log[1].addr == 0x02);
}

TEST_CASE_FIXTURE(TmcFixture, "[R67] characterization: getTmcVM of 6000 counts above the offset returns about 22280 mV") {
	auto tmc = makeTmc1();
	tmc->conf.hwconf.adcOffset = 1000; // conf is public; no hardware version has this offset with the default scaler combination under test
	tmc->conf.hwconf.vmScaler = float(kVmScalerV);
	chip1.setAdcRaw(1, 0x7FFF + 1000 + 6000);
	const double expected = 6000.0 * kVmScalerV * 1000.0; // = 22279.8 mV (HD above)
	CHECK(std::abs(double(tmc->getTmcVM()) - expected) <= 2.0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R67] characterization: getTmcVM uses only bits 15:0 of ADC_RAW_DATA") {
	auto tmc = makeTmc1();
	tmc->conf.hwconf.adcOffset = 1000; // conf is public; no hardware version has this offset with the default scaler combination under test
	tmc->conf.hwconf.vmScaler = float(kVmScalerV);
	chip1.setAdcRaw(1, 0x12340000u | (0x7FFF + 1000 + 6000)); // upper half is another channel (DS 0x02)
	const double expected = 6000.0 * kVmScalerV * 1000.0;
	CHECK(std::abs(double(tmc->getTmcVM()) - expected) <= 2.0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R67] getTmcVM below the ADC offset returns a negative voltage" *
								  KNOWN_ISSUE_UB("B-T13: (raw & 0xFFFF) - 0x7FFF - adcOffset is computed unsigned and wraps to about 4.29e9")) {
	auto tmc = makeTmc1();
	tmc->conf.hwconf.adcOffset = 1000; // conf is public; no hardware version has this offset with the default scaler combination under test
	tmc->conf.hwconf.vmScaler = float(kVmScalerV);
	chip1.setAdcRaw(1, 0x7FFF + 1000 - 1000); // 1000 counts below the offset
	// HD: -1000 * 3.7134e-3 V = -3.7134 V = -3713 mV
	// Host vs target: out-of-range float to int32 is UB; x86 gives INT_MIN, ARM saturates to INT_MAX, so the result differs.
	// The wrapped value (4294966296 counts) converted to int32 is out of range: UB, INT_MIN on x86, so test the exact value.
	CHECK(std::abs(double(tmc->getTmcVM()) - (-1000.0 * kVmScalerV * 1000.0)) <= 2.0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R67] the vm command replies the same voltage as getTmcVM") {
	auto tmc = makeTmc1();
	tmc->conf.hwconf.adcOffset = 1000; // conf is public; no hardware version has this offset with the default scaler combination under test
	tmc->conf.hwconf.vmScaler = float(kVmScalerV);
	chip1.setAdcRaw(1, 0x7FFF + 1000 + 6000);
	std::vector<CommandReply> replies;
	REQUIRE(CommandHelpers::run(*tmc, "vm", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(std::abs(double(replies[0].val) - 6000.0 * kVmScalerV * 1000.0) <= 2.0);
}

// ---------------------------------------------------------------------------------------------------------------
// R68 temperature. Beta formula of the firmware (characterization).
// HD for R2 = 1500, R = 10000, Beta = 4300, offset 0: r = R2 * 43252 / adcval.
//  adcval 6488: r = 64878000 / 6488 = 9999.69 ~ R -> 1/T = 1/298.15 + ln(0.99997)/4300 = 3.354016e-3 - 7e-9
//               -> T = 298.1506 K = 25.0006 C.
//  adcval 12975: r = 64878000 / 12975 = 5000.23 ~ R/2 -> 1/T = 3.354016e-3 + ln(0.5)/4300 = 3.354016e-3 - 1.61197e-4
//               = 3.192819e-3 -> T = 313.2028 K = 40.05 C.
// ---------------------------------------------------------------------------------------------------------------

namespace {
void setupThermistor(TMC4671 &tmc) {
	// conf is public (TMC4671.h:412); thermistor values chosen for a simple hand derivation, not those of a hardware version
	tmc.conf.hwconf.adcOffset = 0;
	tmc.conf.hwconf.thermistorSettings.thermistor_R2 = 1500;
	tmc.conf.hwconf.thermistorSettings.thermistor_R = 10000;
	tmc.conf.hwconf.thermistorSettings.thermistor_Beta = 4300;
	tmc.conf.hwconf.thermistorSettings.temperatureEnabled = true;
}
} // namespace

TEST_CASE_FIXTURE(TmcFixture, "[R68] getTemp selects ADC_RAW_ADDR 2 before it reads ADC_RAW_DATA") {
	auto tmc = makeTmc1();
	setupThermistor(*tmc);
	chip1.setAdcRaw(2, 0x7FFF + 6488);
	clearLogs(chip1);
	tmc->getTemp();
	const auto &log = chip1.log();
	REQUIRE(log.size() >= 2);
	CHECK(log[0].write);
	CHECK(log[0].addr == 0x03);
	CHECK(log[0].data == 2u);
	CHECK_FALSE(log[1].write);
	CHECK(log[1].addr == 0x02);
}

TEST_CASE_FIXTURE(TmcFixture, "[R68] characterization: getTemp of 6488 counts above the centre is 25 C (thermistor at its nominal resistance)") {
	auto tmc = makeTmc1();
	setupThermistor(*tmc);
	chip1.setAdcRaw(2, 0x7FFF + 6488);
	CHECK(tmc->getTemp() == doctest::Approx(25.0).epsilon(0.004)); // HD 25.0006 C, tolerance 0.1 C
}

TEST_CASE_FIXTURE(TmcFixture, "[R68] characterization: getTemp of 12975 counts above the centre is about 40.05 C") {
	auto tmc = makeTmc1();
	setupThermistor(*tmc);
	chip1.setAdcRaw(2, 0x7FFF + 12975);
	CHECK(std::abs(double(tmc->getTemp()) - 40.05) <= 0.1);
}

TEST_CASE_FIXTURE(TmcFixture, "[R68] characterization: getTemp uses only bits 15:0 of ADC_RAW_DATA") {
	auto tmc = makeTmc1();
	setupThermistor(*tmc);
	chip1.setAdcRaw(2, 0xABCD0000u | (0x7FFF + 6488));
	CHECK(std::abs(double(tmc->getTemp()) - 25.0) <= 0.1);
}

TEST_CASE_FIXTURE(TmcFixture, "[R68] getTemp returns 0 and does not touch the chip when the thermistor is disabled") {
	auto tmc = makeTmc1();
	setupThermistor(*tmc);
	tmc->conf.hwconf.thermistorSettings.temperatureEnabled = false; // conf is public; disabling is not reachable via a hw version here
	chip1.setAdcRaw(2, 0x7FFF + 6488);
	clearLogs(chip1);
	CHECK(tmc->getTemp() == 0.0f);
	CHECK(chip1.log().empty());
}

TEST_CASE_FIXTURE(TmcFixture, "[R68] characterization: getTemp returns 0 when the raw value is not above the centre and offset") {
	auto tmc = makeTmc1();
	setupThermistor(*tmc);
	chip1.setAdcRaw(2, 0x7FFF); // adcval = 0
	CHECK(tmc->getTemp() == 0.0f);
}

TEST_CASE_FIXTURE(TmcFixture, "[R68] characterization: the temp command replies the temperature in hundredths of a degree") {
	auto tmc = makeTmc1();
	setupThermistor(*tmc);
	chip1.setAdcRaw(2, 0x7FFF + 6488);
	std::vector<CommandReply> replies;
	REQUIRE(CommandHelpers::run(*tmc, "temp", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(std::abs(double(replies[0].val) - 2500.0) <= 10.0); // 25.0006 C * 100, int truncation and 0.1 C tolerance
}

} // TEST_SUITE
