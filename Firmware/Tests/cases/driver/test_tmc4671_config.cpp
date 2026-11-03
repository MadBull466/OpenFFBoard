/*
 * TMC4671 driver, group G4 "Configuration setters" (requirements R40 - R54 of driver/TMC4671_REQUIREMENTS.md).
 * R55 (setters without a caller) is dead code and not tested.
 *
 * Sources: DS = TMC4671-LA datasheet V2.07 (cited "DS 0xNN" = register map section 7.2, "DS §x pNN"), HD = hand
 * derivation in the comment, FW = firmware behaviour only (test name starts with "characterization:").
 *
 * Method: the driver object is constructed after the fake chip exists (the constructor already writes the default
 * hardware configuration), then the test preloads registers with FakeTmc4671::setReg, calls one public setter and
 * reads the register file of the fake back. Register addresses are the DS addresses.
 *
 * Host vs target: every register value is built from uint32_t/uint16_t, no result depends on the 64 bit host except
 * the float -> integer conversions named in the biquad and setPwmFreq tests (see there).
 */
#include "doctest.h"
#include "TmcFixture.h"

#include <cmath>
#include <complex>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

/** CONFIG_ADDR numbers of the torque biquad, DS 0x4E p94 (17..23 biquad_t: a1 a2 - b0 b1 b2 enable) */
constexpr uint32_t T_A1 = 17, T_A2 = 18, T_B0 = 20, T_B1 = 21, T_B2 = 22, T_EN = 23;

/** Biquad coefficients as written to the chip, Q3.29 (DS §4.8.1 p61) */
struct BqExpect {
	double a1, a2, b0, b1, b2; //!< real values: Y(n) = b0 X(n) + b1 X(n-1) + b2 X(n-2) + a1 Y(n-1) + a2 Y(n-2)
};

/**
 * HD: bilinear transform with K = tan(pi * fc / fs), norm = 1 / (1 + K/Q + K^2).
 * Standard z-domain coefficients (denominator 1 + a1z z^-1 + a2z z^-2):
 *   a1z = 2 (K^2 - 1) norm, a2z = (1 - K/Q + K^2) norm.
 * DS eq. (43)/(44) p61: the TMC register holds a1 = round(-a1z * 2^29), a2 = round(-a2z * 2^29).
 */
BqExpect lowpass(double fc, double fs, double q) {
	const double K = std::tan(kPi * fc / fs);
	const double norm = 1.0 / (1.0 + K / q + K * K);
	BqExpect e;
	e.b0 = K * K * norm;
	e.b1 = 2.0 * e.b0;
	e.b2 = e.b0;
	e.a1 = -2.0 * (K * K - 1.0) * norm;
	e.a2 = -(1.0 - K / q + K * K) * norm;
	return e;
}

/** HD: notch (zeros on the unit circle at fc): b0 = b2 = (1 + K^2) norm, b1 = 2 (K^2 - 1) norm, same denominator */
BqExpect notch(double fc, double fs, double q) {
	const double K = std::tan(kPi * fc / fs);
	const double norm = 1.0 / (1.0 + K / q + K * K);
	BqExpect e;
	e.b0 = (1.0 + K * K) * norm;
	e.b1 = 2.0 * (K * K - 1.0) * norm;
	e.b2 = e.b0;
	e.a1 = -2.0 * (K * K - 1.0) * norm;
	e.a2 = -(1.0 - K / q + K * K) * norm;
	return e;
}

constexpr double Q29 = 536870912.0; // 2^29

double real(uint32_t word) { return static_cast<int32_t>(word) / Q29; }

/**
 * Tolerance on a coefficient, in real units. Charter decision L2 accepts +-2 LSB of Q3.29 (3.7e-9) for the
 * truncation/rounding difference. That cannot hold here: the firmware computes the coefficients in single precision
 * float (24 bit mantissa, relative 6e-8 per operation; with a coefficient of about 1.8 that is ~30 LSB) and the
 * CMSIS-DSP build computes K with the table based arm_sin_f32 / arm_cos_f32. Measured deviation 1.5e-8 (DSP build);
 * 1e-6 (about 540 LSB) is used, far below the effect of a wrong sign or a wrong frequency (> 1e-3). Reported to the lead.
 */
constexpr double kCoefTol = 1e-6;

/** Float arithmetic bound for the DC gain identity (numerator and denominator are ~0.008, see test) */
constexpr double kGainTol = 1e-3;

/** fPWM of the default configuration: PWM_MAXCNT = 4095 -> 100 MHz / 4096 (DS 0x18 p84) */
constexpr double kFs = 100e6 / 4096.0;

} // namespace

TEST_SUITE("TMC4671 configuration") {

// ---------------------------------------------------------------------------------------------------------------
// R40 / R41 PI registers
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R40] setPids writes each PI register with I in bits 15:0 and P in bits 31:16 (P above 0x8000 included)") {
	auto tmc = makeTmc1();
	TMC4671PIDConf p;
	p.fluxI = 0x1111;
	p.fluxP = 0x8001;
	p.torqueI = 0x2222;
	p.torqueP = 0xFFFF;
	p.velocityI = 0x0003;
	p.velocityP = 0x8000;
	p.positionI = 0x4444;
	p.positionP = 0x0005;
	tmc->setPids(p);
	// DS 0x54 / 0x56 / 0x58 / 0x5A p96: I [15:0], P [31:16]
	CHECK(chip1.reg(0x54) == 0x80011111u);
	CHECK(chip1.reg(0x56) == 0xFFFF2222u);
	CHECK(chip1.reg(0x58) == 0x80000003u);
	CHECK(chip1.reg(0x5A) == 0x00054444u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R40] getPids decodes each PI register with I from bits 15:0 and P from bits 31:16") {
	auto tmc = makeTmc1();
	chip1.setReg(0x54, 0x80011111u);
	chip1.setReg(0x56, 0xFFFF2222u);
	chip1.setReg(0x58, 0x80000003u);
	chip1.setReg(0x5A, 0x00054444u);
	TMC4671PIDConf p = tmc->getPids();
	CHECK(p.fluxI == 0x1111);
	CHECK(p.fluxP == 0x8001);
	CHECK(p.torqueI == 0x2222);
	CHECK(p.torqueP == 0xFFFF);
	CHECK(p.velocityI == 0x0003);
	CHECK(p.velocityP == 0x8000);
	CHECK(p.positionI == 0x4444);
	CHECK(p.positionP == 0x0005);
}

TEST_CASE_FIXTURE(TmcFixture, "[R40] setPids sets MODE_PID_TYPE (bit 31 of 0x63) from sequentialPI and keeps the other bits") {
	auto tmc = makeTmc1();
	chip1.setReg(0x63, 0x7A0000ABu); // bit 31 clear, MODE_PID_SMPL [30:24] = 0x7A, motion mode 0xAB
	TMC4671PIDConf p;
	p.sequentialPI = true;
	tmc->setPids(p);
	CHECK(chip1.reg(0x63) == 0xFA0000ABu); // DS 0x63 p97: bit 31 = 1 sequential (advanced) PI
	p.sequentialPI = false;
	tmc->setPids(p);
	CHECK(chip1.reg(0x63) == 0x7A0000ABu);
}

TEST_CASE_FIXTURE(TmcFixture, "[R41] getPids does not change the controller structure of a classic (not sequential) PI setup" *
		KNOWN_ISSUE("B-T5: getPids aggregate-initialises curPids with eight values, so sequentialPI falls back to its default true (TMC4671.cpp:2131)")) {
	auto tmc = makeTmc1();
	TMC4671PIDConf p;
	p.sequentialPI = false;
	tmc->setPids(p);
	REQUIRE((chip1.reg(0x63) >> 31) == 0); // the chip is configured classic
	TMC4671PIDConf back = tmc->getPids();
	CHECK_FALSE(back.sequentialPI); // first failing assertion: the read-back reports sequential
	CHECK_FALSE(tmc->curPids.sequentialPI);
}

TEST_CASE_FIXTURE(TmcFixture, "[R41] getPids only reads: bit 31 of 0x63 stays as configured") {
	auto tmc = makeTmc1();
	TMC4671PIDConf p;
	p.sequentialPI = false;
	tmc->setPids(p);
	chip1.clearLog();
	tmc->getPids();
	CHECK(chip1.writesTo(0x63).empty());
	CHECK((chip1.reg(0x63) >> 31) == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// R42 limits
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R42] setLimits writes uq/ud, torque/flux, acceleration, velocity and position limits to 0x5D - 0x62") {
	auto tmc = makeTmc1();
	TMC4671Limits l;
	l.pid_torque_flux_ddt = 0x1111;
	l.pid_uq_ud = 0x2222;
	l.pid_torque_flux = 0x3333;
	l.pid_acc_lim = 0x44444444u;
	l.pid_vel_lim = 0x55555555u;
	l.pid_pos_low = -0x12345678;  // HD: 2^32 - 0x12345678 = 0xEDCBA988
	l.pid_pos_high = 0x7FFFFFF0;
	tmc->setLimits(l);
	// DS 0x5D PIDOUT_UQ_UD_LIMITS, 0x5E PID_TORQUE_FLUX_LIMITS (u16), 0x5F ACCELERATION, 0x60 VELOCITY, 0x61 / 0x62 POSITION LOW/HIGH (s32) p96-97
	CHECK(chip1.reg(0x5D) == 0x2222u);
	CHECK(chip1.reg(0x5E) == 0x3333u);
	CHECK(chip1.reg(0x5F) == 0x44444444u);
	CHECK(chip1.reg(0x60) == 0x55555555u);
	CHECK(chip1.reg(0x61) == 0xEDCBA988u);
	CHECK(chip1.reg(0x62) == 0x7FFFFFF0u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R42] getLimits decodes the registers 0x5D - 0x62") {
	auto tmc = makeTmc1();
	chip1.setReg(0x5D, 0x2222u);
	chip1.setReg(0x5E, 0x3333u);
	chip1.setReg(0x5F, 0x44444444u);
	chip1.setReg(0x60, 0x55555555u);
	chip1.setReg(0x61, 0xEDCBA988u); // -0x12345678 (HD as above)
	chip1.setReg(0x62, 0x7FFFFFF0u);
	TMC4671Limits l = tmc->getLimits();
	CHECK(l.pid_uq_ud == 0x2222);
	CHECK(l.pid_torque_flux == 0x3333);
	CHECK(l.pid_acc_lim == 0x44444444u);
	CHECK(l.pid_vel_lim == 0x55555555u);
	CHECK(l.pid_pos_low == -0x12345678);
	CHECK(l.pid_pos_high == 0x7FFFFFF0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R42] characterization: setLimits writes the torque/flux ddt limit to the undocumented register 0x5C and getLimits reads it back") {
	// NEEDS REVIEW: register 0x5C is not in DS rev 2.07 (charter Q1); only the driver's own use is documented here.
	auto tmc = makeTmc1();
	TMC4671Limits l;
	l.pid_torque_flux_ddt = 0x1234;
	tmc->setLimits(l);
	CHECK(chip1.reg(0x5C) == 0x1234u);
	chip1.setReg(0x5C, 0x4321u);
	CHECK(tmc->getLimits().pid_torque_flux_ddt == 0x4321);
}

TEST_CASE_FIXTURE(TmcFixture, "[R42] setUqUdLimit writes 0x5D and leaves the torque limit 0x5E alone") {
	auto tmc = makeTmc1();
	chip1.setReg(0x5E, 0x1357u);
	tmc->setUqUdLimit(0x7ABC); // DS 0x5D p96
	CHECK(chip1.reg(0x5D) == 0x7ABCu);
	CHECK(chip1.reg(0x5E) == 0x1357u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R42] setTorqueLimit writes 0x5E and leaves the uq/ud limit 0x5D alone") {
	auto tmc = makeTmc1();
	chip1.setReg(0x5D, 0x1357u);
	tmc->setTorqueLimit(20000); // DS 0x5E p96
	CHECK(chip1.reg(0x5E) == 20000u);
	CHECK(chip1.reg(0x5D) == 0x1357u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R42] setPowerLimit writes the torque limit register 0x5E") {
	auto tmc = makeTmc1();
	tmc->setPowerLimit(12345);
	CHECK(chip1.reg(0x5E) == 12345u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R42] characterization: setTorqueLimit sets the bang-init current to 75 percent of the limit") {
	// NEEDS REVIEW: FW policy (TMC4671.cpp:2150), HD: 20000 * 0.75 = 15000
	auto tmc = makeTmc1();
	tmc->setTorqueLimit(20000);
	CHECK(tmc->bangInitPower == 15000);
}

// ---------------------------------------------------------------------------------------------------------------
// R43 PWM frequency
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R43] setPwmFreq(25 kHz) writes PWM_MAXCNT 3999") {
	auto tmc = makeTmc1();
	tmc->setPwmFreq(25000.0f);
	// DS 0x18 p84: fPWM = 100 MHz / (MAXCNT + 1); HD: 1e8 / 25000 = 4000, MAXCNT = 3999
	CHECK(chip1.reg(0x18) == 3999u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R43] setPwmFreq(100 kHz) writes PWM_MAXCNT 999") {
	auto tmc = makeTmc1();
	tmc->setPwmFreq(100000.0f);
	// HD: 1e8 / 100000 = 1000, MAXCNT = 999
	CHECK(chip1.reg(0x18) == 999u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R43] setPwmFreq(10 kHz) is clipped to PWM_MAXCNT 4095") {
	auto tmc = makeTmc1();
	tmc->setPwmFreq(10000.0f);
	// HD: 1e8 / 10000 = 10000, MAXCNT = 9999, driver range 255..4095 (12 bit counter, DS 0x18 p84)
	CHECK(chip1.reg(0x18) == 4095u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R43] setPwmFreq(400 kHz) is clipped to the driver minimum PWM_MAXCNT 255") {
	auto tmc = makeTmc1();
	tmc->setPwmFreq(400000.0f);
	// HD: 1e8 / 400000 = 250, MAXCNT = 249, driver range minimum 255
	CHECK(chip1.reg(0x18) == 255u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R43] getPwmFreq returns 100 MHz / (MAXCNT + 1) after setPwmFreq") {
	auto tmc = makeTmc1();
	tmc->setPwmFreq(25000.0f);
	// HD: 1e8 / (3999 + 1) = 25000 exactly
	CHECK(tmc->getPwmFreq() == doctest::Approx(25000.0).epsilon(1e-6));
	tmc->setPwmFreq(100000.0f);
	CHECK(tmc->getPwmFreq() == doctest::Approx(100000.0).epsilon(1e-6));
}

TEST_CASE_FIXTURE(TmcFixture, "[R43] getPwmFreq of the default PWM_MAXCNT 4095 is 24414.0625 Hz") {
	auto tmc = makeTmc1();
	// HD: 1e8 / 4096 = 24414.0625
	CHECK(tmc->getPwmFreq() == doctest::Approx(24414.0625).epsilon(1e-6));
}

TEST_CASE_FIXTURE(TmcFixture, "[R43] characterization: setPwmFreq(0) and a negative frequency are ignored") {
	// NEEDS REVIEW: FW guard (TMC4671.cpp:2478), no datasheet statement
	auto tmc = makeTmc1();
	chip1.setReg(0x18, 1234);
	chip1.clearLog();
	tmc->setPwmFreq(0.0f);
	tmc->setPwmFreq(-5.0f);
	CHECK(chip1.writesTo(0x18).empty());
	CHECK(chip1.reg(0x18) == 1234u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R43] setPwmFreq(1500 Hz) is clipped to PWM_MAXCNT 4095 whatever the counter range" *
		KNOWN_ISSUE_UB("B-T6: setPwmFreq converts the float 1e8/f - 1 to uint16_t before clip(); above 65535 that conversion is undefined. HOST VS TARGET: x86 keeps the low 16 bits, ARM vcvt saturates to 65535 (clip then gives 4095), so the defect is visible only on the host")) {
	auto tmc = makeTmc1();
	tmc->setPwmFreq(1500.0f);
	// HD: 1e8 / 1500 = 66666.67, minus 1 = 66665.67 (above 65535). Intended: clipped to 4095.
	// x86 result: 66665 & 0xFFFF = 1129 (inside 255..4095, so written unchanged)
	CHECK(chip1.reg(0x18) == 4095u);
}

// ---------------------------------------------------------------------------------------------------------------
// R44 BBM
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R44] setBBM writes BBM_L to bits 7:0 and BBM_H to bits 15:8 of 0x19") {
	auto tmc = makeTmc1();
	tmc->setBBM(0x05, 0xC8);
	// DS 0x19 p84: BBM_L [7:0], BBM_H [15:8] in 10 ns steps
	CHECK(chip1.reg(0x19) == 0xC805u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R44] setBBM with different low and high times does not swap the bytes") {
	auto tmc = makeTmc1();
	tmc->setBBM(0x12, 0x34);
	CHECK(chip1.reg(0x19) == 0x3412u);
}

// ---------------------------------------------------------------------------------------------------------------
// R45 motion mode
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R45] a forced motion mode is written to MODE_MOTION bits 7:0 of 0x63 and keeps bit 31 and bits 30:24") {
	auto tmc = makeTmc1();
	struct Row { MotionMode mode; uint32_t code; };
	// DS 0x63 p97: 0 stopped, 1 torque, 2 velocity, 3 position, 8 uq_ud_ext
	const Row rows[] = {{MotionMode::stop, 0}, {MotionMode::torque, 1}, {MotionMode::velocity, 2},
						{MotionMode::position, 3}, {MotionMode::uqudext, 8}};
	for (const Row &r : rows) {
		chip1.setReg(0x63, 0x8A000005u); // bit 31 = 1, MODE_PID_SMPL [30:24] = 0x0A, old motion mode 5
		tmc->setMotionMode(r.mode, true);
		INFO("motion mode code " << r.code);
		CHECK(chip1.reg(0x63) == (0x8A000000u | r.code));
	}
}

TEST_CASE_FIXTURE(TmcFixture, "[R45] characterization: a not forced motion mode is written to the chip only when the motor is started") {
	// NEEDS REVIEW: FW deferral (setMotionMode(mode, false) stores nextMotionMode, startMotor() applies it)
	auto tmc = makeTmc1();
	powerOn();
	chip1.setReg(0x63, 0x00000000u);
	chip1.clearLog();
	tmc->setMotionMode(MotionMode::velocity, false);
	CHECK(chip1.writesTo(0x63).empty());
	CHECK((chip1.reg(0x63) & 0xFF) == 0);
	tmc->startMotor();
	CHECK((chip1.reg(0x63) & 0xFF) == 2); // DS 0x63: 2 = velocity mode
}

// ---------------------------------------------------------------------------------------------------------------
// R46 position / velocity / phi_e selectors
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R46] setPosSel writes the selection code to 0x51") {
	auto tmc = makeTmc1();
	// DS 0x51 POSITION_SELECTION p95: 0 phi_e, 1 phi_e_ext, 2 openloop, 3 abn, 5 hal, 6 aenc, 7 phi_a_aenc, 9 phi_m_abn, 10 phi_m_abn_2, 11 phi_m_aenc, 12 phi_m_hal
	const std::pair<PosSelection, uint32_t> rows[] = {
		{PosSelection::PhiE, 0}, {PosSelection::PhiE_ext, 1}, {PosSelection::PhiE_openloop, 2}, {PosSelection::PhiE_abn, 3},
		{PosSelection::PhiE_hal, 5}, {PosSelection::PhiE_aenc, 6}, {PosSelection::PhiA_aenc, 7}, {PosSelection::PhiM_abn, 9},
		{PosSelection::PhiM_abn2, 10}, {PosSelection::PhiM_aenc, 11}, {PosSelection::PhiM_hal, 12}};
	for (const auto &r : rows) {
		tmc->setPosSel(r.first);
		INFO("code " << r.second);
		CHECK(chip1.reg(0x51) == r.second);
	}
}

TEST_CASE_FIXTURE(TmcFixture, "[R46] setVelSel writes the selection code to bits 7:0 and the velocity meter to bits 15:8 of 0x50") {
	auto tmc = makeTmc1();
	// DS 0x50 VELOCITY_SELECTION p95: VELOCITY_SELECTION [7:0], VELOCITY_METER_SELECTION [15:8]
	tmc->setVelSel(VelSelection::PhiM_hal, 1);
	CHECK(chip1.reg(0x50) == 0x010Cu); // 12 | 1 << 8
	tmc->setVelSel(VelSelection::PhiM_abn, 0);
	CHECK(chip1.reg(0x50) == 0x0009u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R46] setPhiEtype writes the phi_e selection code to 0x52") {
	auto tmc = makeTmc1();
	// DS 0x52 PHI_E_SELECTION p95: 1 ext, 2 openloop, 3 abn, 5 hal, 6 aenc, 7 phi_a_aenc
	const std::pair<PhiE, uint32_t> rows[] = {{PhiE::ext, 1}, {PhiE::openloop, 2}, {PhiE::abn, 3}, {PhiE::hall, 5},
											  {PhiE::aenc, 6}, {PhiE::aencE, 7}};
	for (const auto &r : rows) {
		tmc->setPhiEtype(r.first);
		INFO("code " << r.second);
		CHECK(chip1.reg(0x52) == r.second);
	}
}

TEST_CASE_FIXTURE(TmcFixture, "[R46] setPhiEtype(extEncoder) writes the phi_e_ext code 1 to 0x52") {
	auto tmc = makeTmc1();
	tmc->setPhiEtype(PhiE::extEncoder); // DS 0x52: 1 = phi_e_ext
	CHECK(chip1.reg(0x52) == 1u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R46] getPhiEtype decodes the selection code of 0x52") {
	auto tmc = makeTmc1();
	chip1.setReg(0x52, 3);
	CHECK(tmc->getPhiEtype() == PhiE::abn);
	chip1.setReg(0x52, 5);
	CHECK(tmc->getPhiEtype() == PhiE::hall);
	chip1.setReg(0x52, 6);
	CHECK(tmc->getPhiEtype() == PhiE::aenc);
	chip1.setReg(0x52, 2);
	CHECK(tmc->getPhiEtype() == PhiE::openloop);
}

TEST_CASE_FIXTURE(TmcFixture, "[R46] characterization: getPhiEtype reports extEncoder when 0x52 is phi_e_ext and extEncoder was requested") {
	// NEEDS REVIEW: FW mapping of the driver-only value extEncoder onto the chip value 1
	auto tmc = makeTmc1();
	tmc->setPhiEtype(PhiE::extEncoder);
	CHECK(tmc->getPhiEtype() == PhiE::extEncoder);
	tmc->setPhiEtype(PhiE::ext);
	CHECK(tmc->getPhiEtype() == PhiE::ext);
}

// ---------------------------------------------------------------------------------------------------------------
// R47 torque biquad
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R47] torque filter mode none writes the enable word 0 to biquad_t_enable (CONFIG_ADDR 23)") {
	auto tmc = makeTmc1();
	chip1.setConfigWord(T_EN, 0x80000001u); // enable bit 31 (and bit 0) set before
	TMC4671Biquad_conf c;
	c.mode = TMCbiquadpreset::none;
	tmc->setTorqueFilter(c);
	CHECK(chip1.configWord(T_EN) == 0u); // DS 0x4D p93: biquad_t_enable is bit(31), 0 = off; 0 is correct for off whichever bit the driver uses
}

TEST_CASE_FIXTURE(TmcFixture, "[R47] the torque biquad is written through CONFIG_ADDR 17, 18, 20, 21, 22, 23 in this order") {
	auto tmc = makeTmc1();
	const size_t before = chip1.configWrites().size();
	TMC4671Biquad_conf c;
	c.mode = TMCbiquadpreset::lowpass;
	c.params = {500, 70};
	tmc->setTorqueFilter(c);
	const auto &w = chip1.configWrites();
	REQUIRE(w.size() == before + 6);
	const uint32_t order[] = {T_A1, T_A2, T_B0, T_B1, T_B2, T_EN};
	for (size_t i = 0; i < 6; ++i) {
		INFO("write " << i);
		CHECK(w[before + i].first == order[i]);
	}
}

TEST_CASE_FIXTURE(TmcFixture, "[R47] a lowpass torque filter sets biquad_t_enable (bit 31 of CONFIG_DATA 23)" *
		KNOWN_ISSUE("B-T24: biquad enable written to bit 0 instead of bit 31")) {
	auto tmc = makeTmc1();
	TMC4671Biquad_conf c;
	c.mode = TMCbiquadpreset::lowpass;
	c.params = {500, 70};
	tmc->setTorqueFilter(c);
	CHECK((chip1.configWord(T_EN) >> 31) == 1u); // DS 0x4D p93: biquad_t_enable is bit(31), 1 = on
}

TEST_CASE_FIXTURE(TmcFixture, "[R47] a lowpass torque filter with fc = 0 is not enabled") {
	auto tmc = makeTmc1();
	chip1.setConfigWord(T_EN, 0x80000001u); // enable bit 31 (and bit 0) set before
	TMC4671Biquad_conf c;
	c.mode = TMCbiquadpreset::lowpass;
	c.params = {0, 70};
	tmc->setTorqueFilter(c);
	CHECK(chip1.configWord(T_EN) == 0u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R47] a lowpass torque filter writes the numerator coefficients b0, b1, b2 of the bilinear lowpass") {
	auto tmc = makeTmc1();
	TMC4671Biquad_conf c;
	c.mode = TMCbiquadpreset::lowpass;
	c.params = {500, 70}; // 500 Hz, Q = 70 / 100 = 0.7 (driver convention, header comment of TMC4671Biquad_conf)
	tmc->setTorqueFilter(c);
	const BqExpect e = lowpass(500.0, kFs, 0.7); // HD: see lowpass()
	INFO("expected b0 " << e.b0 << " b1 " << e.b1 << " b2 " << e.b2);
	CHECK(std::fabs(real(chip1.configWord(T_B0)) - e.b0) <= kCoefTol);
	CHECK(std::fabs(real(chip1.configWord(T_B1)) - e.b1) <= kCoefTol);
	CHECK(std::fabs(real(chip1.configWord(T_B2)) - e.b2) <= kCoefTol);
}

TEST_CASE_FIXTURE(TmcFixture, "[R47] a lowpass torque filter writes the feedback coefficients a1, a2 with the datasheet sign (negated denominator)") {
	auto tmc = makeTmc1();
	TMC4671Biquad_conf c;
	c.mode = TMCbiquadpreset::lowpass;
	c.params = {500, 70};
	tmc->setTorqueFilter(c);
	const BqExpect e = lowpass(500.0, kFs, 0.7);
	// HD: K = tan(pi * 500 / 24414.0625) = 0.0644, a1 = -2 (K^2 - 1) norm is about +1.82 (positive), a2 about -0.83
	INFO("expected a1 " << e.a1 << " a2 " << e.a2 << ", written a1 " << real(chip1.configWord(T_A1)) << " a2 " << real(chip1.configWord(T_A2)));
	CHECK(std::fabs(real(chip1.configWord(T_A1)) - e.a1) <= kCoefTol);
	CHECK(std::fabs(real(chip1.configWord(T_A2)) - e.a2) <= kCoefTol);
}

TEST_CASE_FIXTURE(TmcFixture, "[R47] a lowpass torque filter has unity DC gain: (b0 + b1 + b2) / (1 - a1 - a2) = 1") {
	auto tmc = makeTmc1();
	TMC4671Biquad_conf c;
	c.mode = TMCbiquadpreset::lowpass;
	c.params = {500, 70};
	tmc->setTorqueFilter(c);
	// HD: DS eq. (33): Y = X (b0 + b1 + b2) + Y (a1 + a2) in steady state with X = Y = 1 => gain = sum(b) / (1 - a1 - a2).
	// The numerator is about 0.008, so the tolerance is relative.
	const double num = real(chip1.configWord(T_B0)) + real(chip1.configWord(T_B1)) + real(chip1.configWord(T_B2));
	const double den = 1.0 - real(chip1.configWord(T_A1)) - real(chip1.configWord(T_A2));
	INFO("numerator " << num << " denominator " << den);
	REQUIRE(den != 0.0);
	CHECK(std::fabs(num / den - 1.0) <= kGainTol);
}

TEST_CASE_FIXTURE(TmcFixture, "[R47] a notch torque filter writes the numerator coefficients b0, b1, b2 of the bilinear notch") {
	auto tmc = makeTmc1();
	TMC4671Biquad_conf c;
	c.mode = TMCbiquadpreset::notch;
	c.params = {500, 20}; // Q = 20 / 10 = 2 (driver convention for notch and peak)
	tmc->setTorqueFilter(c);
	const BqExpect e = notch(500.0, kFs, 2.0); // HD: see notch()
	INFO("expected b0 " << e.b0 << " b1 " << e.b1 << " b2 " << e.b2);
	CHECK(std::fabs(real(chip1.configWord(T_B0)) - e.b0) <= kCoefTol);
	CHECK(std::fabs(real(chip1.configWord(T_B1)) - e.b1) <= kCoefTol);
	CHECK(std::fabs(real(chip1.configWord(T_B2)) - e.b2) <= kCoefTol);
}

TEST_CASE_FIXTURE(TmcFixture, "[R47] a notch torque filter sets biquad_t_enable (bit 31 of CONFIG_DATA 23)" *
		KNOWN_ISSUE("B-T24: biquad enable written to bit 0 instead of bit 31")) {
	auto tmc = makeTmc1();
	TMC4671Biquad_conf c;
	c.mode = TMCbiquadpreset::notch;
	c.params = {500, 20};
	tmc->setTorqueFilter(c);
	CHECK((chip1.configWord(T_EN) >> 31) == 1u); // DS 0x4D p93
}

TEST_CASE_FIXTURE(TmcFixture, "[R47] a notch torque filter writes the feedback coefficients a1, a2 with the datasheet sign") {
	auto tmc = makeTmc1();
	TMC4671Biquad_conf c;
	c.mode = TMCbiquadpreset::notch;
	c.params = {500, 20};
	tmc->setTorqueFilter(c);
	const BqExpect e = notch(500.0, kFs, 2.0);
	INFO("expected a1 " << e.a1 << " a2 " << e.a2);
	CHECK(std::fabs(real(chip1.configWord(T_A1)) - e.a1) <= kCoefTol);
	CHECK(std::fabs(real(chip1.configWord(T_A2)) - e.a2) <= kCoefTol);
}

TEST_CASE_FIXTURE(TmcFixture, "[R47] a peak torque filter has unity DC gain") {
	auto tmc = makeTmc1();
	TMC4671Biquad_conf c;
	c.mode = TMCbiquadpreset::peak;
	c.params = {500, 20};
	c.gain = 10.0f; // dB, boost
	tmc->setTorqueFilter(c);
	// HD: a peaking EQ has gain 1 at DC and at Nyquist: sum(b) / (1 - a1 - a2) = 1 (denominator is again ~0.008)
	const double num = real(chip1.configWord(T_B0)) + real(chip1.configWord(T_B1)) + real(chip1.configWord(T_B2));
	const double den = 1.0 - real(chip1.configWord(T_A1)) - real(chip1.configWord(T_A2));
	INFO("numerator " << num << " denominator " << den);
	REQUIRE(den != 0.0);
	CHECK(std::fabs(num / den - 1.0) <= kGainTol);
}

TEST_CASE_FIXTURE(TmcFixture, "[R47] a peak torque filter of 10 dB has the gain 3.162 at the centre frequency") {
	auto tmc = makeTmc1();
	TMC4671Biquad_conf c;
	c.mode = TMCbiquadpreset::peak;
	c.params = {500, 20};
	c.gain = 10.0f;
	tmc->setTorqueFilter(c);
	// HD: a peaking EQ with boost G dB has the linear gain V = 10^(G/20) at its centre frequency: V = 10^0.5 = 3.1623.
	// |H(e^jw)| at w = 2 pi fc / fs from the written coefficients, DS eq. (33) p61 with the DS sign convention:
	// H = (b0 + b1 z^-1 + b2 z^-2) / (1 - a1 z^-1 - a2 z^-2), z^-1 = e^-jw.
	const double w = 2.0 * kPi * 500.0 / kFs;
	const std::complex<double> z1 = std::polar(1.0, -w), z2 = std::polar(1.0, -2.0 * w);
	const std::complex<double> num = real(chip1.configWord(T_B0)) + real(chip1.configWord(T_B1)) * z1 + real(chip1.configWord(T_B2)) * z2;
	const std::complex<double> den = 1.0 - real(chip1.configWord(T_A1)) * z1 - real(chip1.configWord(T_A2)) * z2;
	const double gain = std::abs(num / den);
	INFO("gain at fc " << gain);
	CHECK(std::fabs(gain / 3.16228 - 1.0) <= 0.01);
}

TEST_CASE_FIXTURE(TmcFixture, "[R47] a peak torque filter sets biquad_t_enable (bit 31 of CONFIG_DATA 23)" *
		KNOWN_ISSUE("B-T24: biquad enable written to bit 0 instead of bit 31")) {
	auto tmc = makeTmc1();
	TMC4671Biquad_conf c;
	c.mode = TMCbiquadpreset::peak;
	c.params = {500, 20};
	c.gain = 10.0f;
	tmc->setTorqueFilter(c);
	CHECK((chip1.configWord(T_EN) >> 31) == 1u); // DS 0x4D p93
}

// ---------------------------------------------------------------------------------------------------------------
// R48 hardware version configuration (board data)
// ---------------------------------------------------------------------------------------------------------------

namespace {
/** Brake limits as written to 0x75 (DS 0x75 p103: VM_LIMIT_LOW [15:0], HIGH [31:16]) */
uint32_t brakeWord(uint32_t low, uint32_t high) { return low | (high << 16); }
} // namespace

TEST_CASE_FIXTURE(TmcFixture, "[R48] characterization: hardware version 1 (v1.0 AD8417) sets brake limits 52400/52800 and BBM 20") {
	// NEEDS REVIEW: board data of TMC4671_configs.cpp (conf1_0), not in the datasheet. BBM 20 -> 0x14 in both bytes of 0x19.
	auto tmc = makeTmc1();
	tmc->setHwType(2); // away from the default, so the write of version 1 is visible
	tmc->setHwType(1);
	CHECK(chip1.reg(0x75) == brakeWord(52400, 52800));
	CHECK(chip1.reg(0x19) == 0x1414u);
	CHECK(tmc->conf.hwconf.hwVersion == 1);
	CHECK_FALSE(tmc->conf.hwconf.thermistorSettings.temperatureEnabled);
}

TEST_CASE_FIXTURE(TmcFixture, "[R48] characterization: hardware version 2 (v1.2 AD8417) sets brake limits 50700/50900 and BBM 20") {
	// NEEDS REVIEW: conf1_2
	auto tmc = makeTmc1();
	tmc->setHwType(1);
	tmc->setHwType(2);
	CHECK(chip1.reg(0x75) == brakeWord(50700, 50900));
	CHECK(chip1.reg(0x19) == 0x1414u);
	CHECK(tmc->conf.hwconf.hwVersion == 2);
	CHECK(tmc->conf.hwconf.thermistorSettings.temperatureEnabled);
	CHECK(tmc->conf.hwconf.thermistorSettings.thermistor_R == doctest::Approx(22000.0));
}

TEST_CASE_FIXTURE(TmcFixture, "[R48] characterization: hardware version 3 (v1.2.2 LEM 10) sets brake limits 50700/50900 and BBM 20") {
	// NEEDS REVIEW: conf1_2_2
	auto tmc = makeTmc1();
	tmc->setHwType(1);
	tmc->setHwType(3);
	CHECK(chip1.reg(0x75) == brakeWord(50700, 50900));
	CHECK(chip1.reg(0x19) == 0x1414u);
	CHECK(tmc->conf.hwconf.hwVersion == 3);
	CHECK(tmc->conf.hwconf.thermistorSettings.thermistor_R == doctest::Approx(10000.0));
}

TEST_CASE_FIXTURE(TmcFixture, "[R48] characterization: hardware version 6 (v1.3 ACS724) sets brake limits 50700/50900 and the longer BBM 40") {
	// NEEDS REVIEW: conf1_3. BBM 40 = 0x28 in both bytes of 0x19.
	auto tmc = makeTmc1();
	tmc->setHwType(1);
	tmc->setHwType(6);
	CHECK(chip1.reg(0x75) == brakeWord(50700, 50900));
	CHECK(chip1.reg(0x19) == 0x2828u);
	CHECK(tmc->conf.hwconf.hwVersion == 6);
}

TEST_CASE_FIXTURE(TmcFixture, "[R48] characterization: the supply voltage readout follows the ADC offset and scaler of the selected hardware version") {
	// NEEDS REVIEW: board data. getTmcVM = (raw - 0x7FFF - adcOffset) * vmScaler * 1000 mV, vmScaler = 2.5/0x7FFF * (1.5+71.5)/1.5
	// HD: raw = 0x8FFF = 36863 -> 36863 - 32767 = 4096 counts; vmScaler = 7.6296e-5 * 48.667 = 3.7131e-3 V/count
	//   version 6 (adcOffset 0):    4096 * 3.7131e-3 = 15.2087 V -> 15208.7 mV
	//   version 1 (adcOffset 1000): 3096 * 3.7131e-3 = 11.4956 V -> 11495.6 mV
	auto tmc = makeTmc1();
	chip1.setAdcRaw(1, 0x00008FFFu); // VM in [15:0] for ADC_RAW_ADDR 1 (DS 0x02 p77)
	const double scaler = (2.5 / 32767.0) * (73.0 / 1.5);
	tmc->setHwType(6);
	CHECK(std::fabs(tmc->getTmcVM() - 4096 * scaler * 1000.0) <= 1.0);
	tmc->setHwType(1);
	CHECK(std::fabs(tmc->getTmcVM() - 3096 * scaler * 1000.0) <= 1.0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R48] characterization: versions without a config (4, 5 and an unknown number) keep the previous hardware config") {
	// NEEDS REVIEW: FW fallback (no match: conf.hwconf unchanged); versions 4 and 5 exist in TMC_HW_Ver but have no table entry
	auto tmc = makeTmc1();
	tmc->setHwType(1);
	for (uint8_t v : {uint8_t(4), uint8_t(5), uint8_t(31)}) {
		tmc->setHwType(v);
		INFO("requested version " << int(v));
		CHECK(tmc->conf.hwconf.hwVersion == 1);
		CHECK(chip1.reg(0x75) == brakeWord(52400, 52800));
		CHECK(chip1.reg(0x19) == 0x1414u);
	}
}

TEST_CASE_FIXTURE(TmcFixture, "[R48] characterization: a construction with empty flash applies the first config of the table (version 6)") {
	// NEEDS REVIEW: restoreFlash() selects tmc4671_hw_configs[0] when nothing was stored
	auto tmc = makeTmc1();
	CHECK(tmc->conf.hwconf.hwVersion == 6);
	CHECK(chip1.reg(0x75) == brakeWord(50700, 50900));
	CHECK(chip1.reg(0x19) == 0x2828u);
}

// ---------------------------------------------------------------------------------------------------------------
// R49 / R50 ABN encoder
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R49] setup_ABN_Enc maps each flag to its bit of ABN_DECODER_MODE (0x25)") {
	auto tmc = makeTmc1();
	// DS 0x25 p86: apol b0, bpol b1, npol b2, use_abn_as_n b3, cln b8, direction b12
	struct Row { const char *name; bool TMC4671ABNConf::*flag; uint32_t bit; };
	const Row rows[] = {{"apol", &TMC4671ABNConf::apol, 0}, {"bpol", &TMC4671ABNConf::bpol, 1}, {"npol", &TMC4671ABNConf::npol, 2},
						{"ab_as_n", &TMC4671ABNConf::ab_as_n, 3}, {"latch_on_N", &TMC4671ABNConf::latch_on_N, 8},
						{"rdir", &TMC4671ABNConf::rdir, 12}};
	for (const Row &r : rows) {
		TMC4671ABNConf c;
		c.apol = c.bpol = c.npol = c.ab_as_n = c.latch_on_N = c.rdir = false;
		c.*(r.flag) = true;
		tmc->setup_ABN_Enc(c);
		INFO("flag " << r.name);
		CHECK(chip1.reg(0x25) == (1u << r.bit));
	}
}

TEST_CASE_FIXTURE(TmcFixture, "[R49] setup_ABN_Enc writes the PPR to 0x26 (above 16 bit too)") {
	auto tmc = makeTmc1();
	TMC4671ABNConf c;
	c.cpr = 40000; // above 65535 is not reached: 40000 < 65536, use a second value for 24 bit
	tmc->setup_ABN_Enc(c);
	CHECK(chip1.reg(0x26) == 40000u); // DS 0x26 ABN_DECODER_PPR u24 p86
	c.cpr = 0x00ABCDEF;
	tmc->setup_ABN_Enc(c);
	CHECK(chip1.reg(0x26) == 0x00ABCDEFu);
}

TEST_CASE_FIXTURE(TmcFixture, "[R49] setup_ABN_Enc writes phi_e_offset to bits 31:16 and phi_m_offset to bits 15:0 of 0x29") {
	auto tmc = makeTmc1();
	TMC4671ABNConf c;
	c.phiEoffset = -2;	   // HD: 0xFFFE
	c.phiMoffset = 0x1234;
	tmc->setup_ABN_Enc(c);
	CHECK(chip1.reg(0x29) == 0xFFFE1234u); // DS 0x29 p87
}

TEST_CASE_FIXTURE(TmcFixture, "[R49] setCpr writes the count to the ABN PPR 0x26 and the AENC PPR 0x40") {
	auto tmc = makeTmc1();
	tmc->setCpr(8192);
	CHECK(chip1.reg(0x26) == 8192u); // DS 0x26 p86
	CHECK(chip1.reg(0x40) == 8192u); // DS 0x40 AENC_DECODER_PPR p90
}

TEST_CASE_FIXTURE(TmcFixture, "[R49] setCpr(0) writes 1 instead of 0") {
	auto tmc = makeTmc1();
	tmc->setCpr(0);
	CHECK(chip1.reg(0x26) == 1u);
	CHECK(chip1.reg(0x40) == 1u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R50] enabling the encoder index with zeroEncoder sets the cln bit (bit 8) of 0x25" *
		KNOWN_ISSUE("B-T7: setEncoderIndexFlagEnabled writes bit 9 of ABN_DECODER_MODE (updateReg(0x25, zero, 0x1, 9)); DS 0x25 defines cln at bit 8 and no bit 9 (TMC4671.cpp:898)")) {
	auto tmc = makeTmc1();
	chip1.setReg(0x25, 0);
	tmc->setEncoderIndexFlagEnabled(true, true);
	CHECK((chip1.reg(0x25) & 0x100u) == 0x100u); // first failing assertion: cln is not set
	CHECK((chip1.reg(0x25) & 0x200u) == 0u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R50] disabling zeroEncoder clears the cln bit (bit 8) of 0x25" *
		KNOWN_ISSUE("B-T7: the driver clears bit 9, so bit 8 (cln) stays set")) {
	auto tmc = makeTmc1();
	chip1.setReg(0x25, 0x100);
	tmc->setEncoderIndexFlagEnabled(true, false);
	CHECK((chip1.reg(0x25) & 0x100u) == 0u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R50] enabling the encoder index flag clears the raised status flags (0x7C)") {
	auto tmc = makeTmc1();
	chip1.raiseFlags(0x04000000u | 0x10000000u); // adc_i_clipped and enc_n, DS 0x7C p107
	REQUIRE(chip1.reg(0x7C) != 0u);
	tmc->setEncoderIndexFlagEnabled(true, false);
	CHECK(chip1.reg(0x7C) == 0u); // DS 0x7C: the register is also used to reset the flags (write 0)
}

TEST_CASE_FIXTURE(TmcFixture, "[R50] the encoder index flag setter leaves all ABN_DECODER_MODE bits except 8 and 9 alone") {
	auto tmc = makeTmc1();
	chip1.setReg(0x25, 0x00001005); // apol, npol, direction
	tmc->setEncoderIndexFlagEnabled(true, true);
	CHECK((chip1.reg(0x25) & ~0x300u) == 0x1005u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R50] the encoder index flag setter with zeroEncoder preloads ABN_DECODER_COUNT_N (0x28) with 0") {
	auto tmc = makeTmc1();
	chip1.setReg(0x28, 0x1234);
	tmc->setEncoderIndexFlagEnabled(true, true); // DS 0x28 ABN_DECODER_COUNT_N p87
	CHECK(chip1.reg(0x28) == 0u);
}

// ---------------------------------------------------------------------------------------------------------------
// R51 AENC
// ---------------------------------------------------------------------------------------------------------------

namespace {
TMC4671AENCConf aencConf() {
	TMC4671AENCConf c;
	c.cpr = 1000;
	c.phiAoffset = -3;			// HD: 0xFFFD
	c.phiEoffset = 0x1111;
	c.phiMoffset = 0x2222;
	c.AENC0_offset = 0x7FF0;
	c.AENC0_scale = 256;
	c.AENC1_offset = 0x8010;
	c.AENC1_scale = -2;			// HD: 0xFFFE
	c.AENC2_offset = 0x7FFF;
	c.AENC2_scale = 0x0123;
	c.nThreshold = 0x0100;
	c.nMask = 0;
	c.uvwmode = false;
	c.rdir = false;
	return c;
}
} // namespace

TEST_CASE_FIXTURE(TmcFixture, "[R51] setup_AENC writes offset to bits 15:0 and scale to bits 31:16 of 0x0D, 0x0E and 0x0F") {
	auto tmc = makeTmc1();
	tmc->setup_AENC(aencConf());
	// DS 0x0D - 0x0F p79-80: AENC_x_OFFSET [15:0], AENC_x_SCALE [31:16] (s16)
	CHECK(chip1.reg(0x0D) == 0x01007FF0u);
	CHECK(chip1.reg(0x0E) == 0xFFFE8010u);
	CHECK(chip1.reg(0x0F) == 0x01237FFFu);
}

TEST_CASE_FIXTURE(TmcFixture, "[R51] setup_AENC writes the PPR to 0x40 and the phi_a offset to 0x3E") {
	auto tmc = makeTmc1();
	tmc->setup_AENC(aencConf());
	CHECK(chip1.reg(0x40) == 1000u);						  // DS 0x40 p90
	CHECK((chip1.reg(0x3E) & 0xFFFFu) == 0xFFFDu);			  // DS 0x3E AENC_DECODER_PHI_A_OFFSET s16 [15:0]
}

TEST_CASE_FIXTURE(TmcFixture, "[R51] setup_AENC writes phi_m_offset to bits 15:0 and phi_e_offset to bits 31:16 of 0x45" *
		KNOWN_ISSUE("B-T8: setup_AENC writes the halves of AENC_DECODER_PHI_E_PHI_M_OFFSET swapped (phiE in 15:0, phiM in 31:16), DS 0x45 p91 has phi_m in 15:0 and phi_e in 31:16 (TMC4671.cpp:1340)")) {
	auto tmc = makeTmc1();
	tmc->setup_AENC(aencConf());
	// phiM = 0x2222 -> [15:0], phiE = 0x1111 -> [31:16]
	CHECK(chip1.reg(0x45) == 0x11112222u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R51] setup_AENC writes the N threshold to bits 15:0 of 0x3C") {
	auto tmc = makeTmc1();
	tmc->setup_AENC(aencConf());
	// The upper half (nMask) was removed in DS rev V2.04 and is not asserted (charter Q1)
	CHECK((chip1.reg(0x3C) & 0xFFFFu) == 0x0100u); // DS 0x3C AENC_DECODER_N_THRESHOLD p90
}

TEST_CASE_FIXTURE(TmcFixture, "[R51] setup_AENC maps uvwmode to bit 0 and rdir to bit 12 of 0x3B") {
	auto tmc = makeTmc1();
	// DS 0x3B AENC_DECODER_MODE p90: bit 0 = uvw (120deg_n90deg), bit 12 = direction
	TMC4671AENCConf c = aencConf();
	tmc->setup_AENC(c);
	CHECK(chip1.reg(0x3B) == 0u);
	c.uvwmode = true;
	tmc->setup_AENC(c);
	CHECK(chip1.reg(0x3B) == 0x0001u);
	c.uvwmode = false;
	c.rdir = true;
	tmc->setup_AENC(c);
	CHECK(chip1.reg(0x3B) == 0x1000u);
	c.uvwmode = true;
	tmc->setup_AENC(c);
	CHECK(chip1.reg(0x3B) == 0x1001u);
}

// ---------------------------------------------------------------------------------------------------------------
// R52 Hall
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R52] setup_HALL maps each flag to its bit of HALL_MODE (0x33)") {
	auto tmc = makeTmc1();
	// DS 0x33 p89: polarity b0, synchronous PWM sampling b4 (firmware name: filter), interpolation b8, direction b12
	struct Row { const char *name; bool TMC4671HALLConf::*flag; uint32_t bit; };
	const Row rows[] = {{"polarity", &TMC4671HALLConf::polarity, 0}, {"filter", &TMC4671HALLConf::filter, 4},
						{"interpolation", &TMC4671HALLConf::interpolation, 8}, {"direction", &TMC4671HALLConf::direction, 12}};
	for (const Row &r : rows) {
		TMC4671HALLConf c;
		c.polarity = c.filter = c.interpolation = c.direction = false;
		c.blank = 0;
		c.*(r.flag) = true;
		tmc->setup_HALL(c);
		INFO("flag " << r.name);
		CHECK(chip1.reg(0x33) == (1u << r.bit));
	}
}

TEST_CASE_FIXTURE(TmcFixture, "[R52] setup_HALL writes HALL_BLANK to bits 27:16 of 0x33") {
	auto tmc = makeTmc1();
	TMC4671HALLConf c;
	c.polarity = c.filter = c.interpolation = c.direction = false;
	c.blank = 0xABC;
	tmc->setup_HALL(c);
	CHECK(chip1.reg(0x33) == 0x0ABC0000u); // DS 0x33 HALL_BLANK u12 [27:16]
}

TEST_CASE_FIXTURE(TmcFixture, "[R52] setup_HALL writes the six positions as low/high halves of 0x34, 0x35 and 0x36") {
	auto tmc = makeTmc1();
	TMC4671HALLConf c; // header defaults: 0, 10922, 21845, -32768, -21846, -10923
	tmc->setup_HALL(c);
	// DS 0x34 p89: 000 [15:0], 060 [31:16]; 0x35: 120 [15:0], 180 [31:16]; 0x36: 240 [15:0], 300 [31:16]
	// HD: 10922 = 0x2AAA, 21845 = 0x5555, -32768 = 0x8000, -21846 = 65536 - 21846 = 0xAAAA, -10923 = 65536 - 10923 = 0xD555
	CHECK(chip1.reg(0x34) == 0x2AAA0000u);
	CHECK(chip1.reg(0x35) == 0x80005555u);
	CHECK(chip1.reg(0x36) == 0xD555AAAAu);
}

TEST_CASE_FIXTURE(TmcFixture, "[R52] setup_HALL writes phi_m_offset to bits 15:0 and phi_e_offset to bits 31:16 of 0x37") {
	auto tmc = makeTmc1();
	TMC4671HALLConf c;
	c.phiMoffset = 0x1234;
	c.phiEoffset = -1; // HD: 0xFFFF
	tmc->setup_HALL(c);
	CHECK(chip1.reg(0x37) == 0xFFFF1234u); // DS 0x37 p89
}

TEST_CASE_FIXTURE(TmcFixture, "[R52] setup_HALL writes dPhiMax to 0x38") {
	auto tmc = makeTmc1();
	TMC4671HALLConf c;
	c.dPhiMax = 0x2BCD;
	tmc->setup_HALL(c);
	CHECK(chip1.reg(0x38) == 0x2BCDu); // DS 0x38 HALL_DPHI_MAX u16 p89
}

// ---------------------------------------------------------------------------------------------------------------
// R53 / R54 direct voltage and open loop
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R53] setUdUq writes ud to bits 15:0 and uq to bits 31:16 of UQ_UD_EXT (0x24), uq negative") {
	auto tmc = makeTmc1();
	tmc->setUdUq(100, -200);
	// DS 0x24 p86: UD_EXT [15:0], UQ_EXT [31:16]. HD: 100 = 0x0064, -200 = 0xFF38
	CHECK(chip1.reg(0x24) == 0xFF380064u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R53] setUdUq with a negative ud does not disturb uq" *
		KNOWN_ISSUE("B-T9: setUdUq computes ud | (uq << 16) with a sign-extended int16_t ud, so a negative ud sets all bits of the uq half (TMC4671.cpp:1667)")) {
	auto tmc = makeTmc1();
	tmc->setUdUq(-1, 5);
	// HD: ud = 0xFFFF, uq = 0x0005 -> 0x0005FFFF
	CHECK(chip1.reg(0x24) == 0x0005FFFFu);
}

TEST_CASE_FIXTURE(TmcFixture, "[R54] setOpenLoopSpeedAccel writes the velocity target to 0x21 and the acceleration to 0x20") {
	auto tmc = makeTmc1();
	tmc->setOpenLoopSpeedAccel(-1000, 5000);
	// DS 0x20 OPENLOOP_ACCELERATION, 0x21 OPENLOOP_VELOCITY_TARGET (s32) p85. HD: -1000 = 0xFFFFFC18
	CHECK(chip1.reg(0x21) == 0xFFFFFC18u);
	CHECK(chip1.reg(0x20) == 5000u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R54] runOpenLoop selects the uq_ud_ext motion mode and the open loop phi_e source") {
	auto tmc = makeTmc1();
	powerOn();
	tmc->runOpenLoop(100, 200, 300, 400);
	CHECK((chip1.reg(0x63) & 0xFFu) == 8u); // DS 0x63 p97: 8 = uq_ud_ext
	CHECK(chip1.reg(0x52) == 2u);			// DS 0x52: 2 = openloop
}

TEST_CASE_FIXTURE(TmcFixture, "[R54] runOpenLoop writes the voltages, speed and acceleration") {
	auto tmc = makeTmc1();
	powerOn();
	tmc->runOpenLoop(100, 200, 300, 400);
	CHECK(chip1.reg(0x24) == 0x00C80064u); // ud = 100 (0x64) [15:0], uq = 200 (0xC8) [31:16]
	CHECK(chip1.reg(0x21) == 300u);
	CHECK(chip1.reg(0x20) == 400u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R54] runOpenLoop starts the open loop angle (0x23) at the current phi_e (0x53)") {
	auto tmc = makeTmc1();
	powerOn();
	chip1.setReg(0x53, 0x1234);
	tmc->runOpenLoop(100, 200, 300, 400);
	CHECK(chip1.reg(0x23) == 0x1234u); // DS 0x23 OPENLOOP_PHI p85 (writable)
}

} // TEST_SUITE
