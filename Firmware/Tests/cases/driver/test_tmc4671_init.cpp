/*
 * TMC4671 driver, group G3: initialisation sequence (initialize(), initializeWithPower(), setupDriver()).
 * Requirements R20-R30 of Firmware/Tests/driver/TMC4671_REQUIREMENTS.md, lead decisions L1, L2, L9.
 *
 * Method: the register file of FakeTmc4671 is checked after the driver wrote it. Register layouts are cited from the
 * TMC4671-LA datasheet Rev 2.07 ("DS 0xNN" = section 7.2 register description, "DS §x pNN"). Where a value is a
 * firmware or board choice the test overrides it through the public `conf` / `curPids` / `curLimits` members with
 * distinct values, so the test checks the field layout and not a constant. Registers are preloaded with a stale value
 * different from the expected one, so a register that is never written is detected.
 *
 * Host vs target: `pole_pairs << 16`, `high << 16` etc. in the driver promote uint16_t to int; with values >= 0x8000
 * the shift reaches the sign bit (implementation defined before C++20, defined since). Same result on the 32 bit
 * target (int is 32 bit there too). Biquad coefficients: the driver computes them in float (and with
 * USE_DSP_FUNCTIONS in CMSIS arm_sin_f32/arm_cos_f32 on target); the oracle here is computed in double, the tolerance
 * of +-2 LSB (L2) can be different on target (to be checked on hardware).
 */
#include "doctest.h"
#include "TmcFixture.h"

#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <vector>

namespace {

using Frame = FakeTmc4671::Frame;

/** TMC_1 constructed, 24 V supply (hasPower() true: initialize() does not calibrate), then `prep`, then initialize() */
std::unique_ptr<TMC_1> initialised(TmcFixture &f, const std::function<void(TMC_1 &)> &prep = {}) {
	auto tmc = f.makeTmc1();
	f.powerOn(); // after the constructor: setHwType changes vSenseMult (fixture header)
	if (prep) {
		prep(*tmc);
	}
	REQUIRE(tmc->initialize());
	return tmc;
}

/** Index of the first write frame to `addr` in the log from `from`, or -1 */
int findWrite(const std::vector<Frame> &log, uint8_t addr, size_t from = 0, const std::function<bool(uint32_t)> &pred = {}) {
	for (size_t i = from; i < log.size(); i++) {
		if (log[i].write && log[i].addr == addr && (!pred || pred(log[i].data))) {
			return static_cast<int>(i);
		}
	}
	return -1;
}

/** CONFIG_ADDR sequence (the first element of the configWrites pairs) */
std::vector<uint32_t> configAddrSequence(const FakeTmc4671 &c) {
	std::vector<uint32_t> v;
	for (const auto &w : c.configWrites()) {
		v.push_back(w.first);
	}
	return v;
}

/** Lowpass biquad oracle (HD): bilinear transform, same as the standard RBJ-free "K = tan(pi fc/fs)" lowpass */
struct LowpassOracle {
	double b0, b1, b2, denA1, denA2; // denA1/2: denominator coefficients of 1 + denA1 z^-1 + denA2 z^-2
	LowpassOracle(double fc, double fs, double q) {
		const double K = std::tan(M_PI * fc / fs);
		const double norm = 1.0 / (1.0 + K / q + K * K);
		b0 = K * K * norm;
		b1 = 2.0 * b0;
		b2 = b0;
		denA1 = 2.0 * (K * K - 1.0) * norm;
		denA2 = (1.0 - K / q + K * K) * norm;
	}
};
constexpr double Q29 = 536870912.0; // 2^29, Q3.29 (DS §4.8.1 p61)

} // namespace

TEST_SUITE("TMC4671 init") {

// ---------------------------------------------------------------------------------------------------------------------
// R20 PWM
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R20] initialize writes PWM_MAXCNT 4095 which is a PWM frequency of 24414.0625 Hz") {
	// DS 0x18 p84: fPWM = 100 MHz / (PWM_MAXCNT + 1) = 100e6 / 4096 = 24414.0625 Hz (HD, exactly representable in float)
	auto tmc = initialised(*this, [&](TMC_1 &) { chip1.setReg(0x18, 0x00000F9F); }); // stale: DS default 3999
	CHECK(chip1.reg(0x18) == 4095);
	CHECK(tmc->getPwmFreq() == doctest::Approx(24414.0625));
}

TEST_CASE_FIXTURE(TmcFixture, "[R20] initialize packs BBM_L into bits 7..0 and BBM_H into bits 15..8 of register 0x19") {
	// DS 0x19 p84: BBM_L [7:0], BBM_H [15:8]. Distinct values 0x12 / 0x34 -> 0x3412 (HD)
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.bbmL = 0x12;
		t.conf.bbmH = 0x34;
	});
	CHECK(chip1.reg(0x19) == 0x3412);
}

TEST_CASE_FIXTURE(TmcFixture, "[R20] initialize sets both PWM polarity bits (register 0x17) to 0") {
	// DS 0x17 p84: bit 0 PWM_POLARITIES LS, bit 1 HS; 0 = active high (the firmware writes 0)
	auto tmc = initialised(*this, [&](TMC_1 &) { chip1.setReg(0x17, 0x3); });
	CHECK(chip1.reg(0x17) == 0);
}

// ---------------------------------------------------------------------------------------------------------------------
// R21 motor type and pole pairs, SVPWM
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R21] a BLDC motor with 7 pole pairs is written to 0x1B as pole pairs | 3 << 16") {
	// DS 0x1B p84-85: N_POLE_PAIRS [15:0], MOTOR_TYPE [23:16] (3 = BLDC). 7 | 3 << 16 = 0x00030007 (HD)
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.motconf.motor_type = MotorType::BLDC;
		t.conf.motconf.pole_pairs = 7;
	});
	CHECK(chip1.reg(0x1B) == 0x00030007);
}

TEST_CASE_FIXTURE(TmcFixture, "[R21] a stepper motor with 50 pole pairs is written to 0x1B as pole pairs | 2 << 16") {
	// DS 0x1B: MOTOR_TYPE 2 = two phase stepper. 50 = 0x32 -> 0x00020032 (HD)
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.motconf.motor_type = MotorType::STEPPER;
		t.conf.motconf.pole_pairs = 50;
	});
	CHECK(chip1.reg(0x1B) == 0x00020032);
}

TEST_CASE_FIXTURE(TmcFixture, "[R21] motor type none is written to 0x1B with type 0 and the configured pole pairs") {
	// DS 0x1B: MOTOR_TYPE 0 = no motor. 4 pole pairs -> 0x00000004 (HD)
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.motconf.motor_type = MotorType::NONE;
		t.conf.motconf.pole_pairs = 4;
		chip1.setReg(0x1B, 0x00FF00FF); // stale
	});
	CHECK(chip1.reg(0x1B) == 0x00000004);
}

TEST_CASE_FIXTURE(TmcFixture, "[R21] a DC motor is written with exactly 1 pole pair whatever was configured") {
	// DS 0x1B: MOTOR_TYPE 1 = DC. 1 | 1 << 16 = 0x00010001 (HD). The firmware forces 1 pole pair for DC.
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.motconf.motor_type = MotorType::DC;
		t.conf.motconf.pole_pairs = 5;
	});
	CHECK(chip1.reg(0x1B) == 0x00010001);
}

TEST_CASE_FIXTURE(TmcFixture, "[R21] SVPWM (bit 8 of 0x1A) is set for a BLDC motor with SVPWM enabled") {
	// DS 0x1A p84: PWM_SV bit 8
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.motconf.motor_type = MotorType::BLDC;
		t.conf.motconf.svpwm = true;
	});
	CHECK((chip1.reg(0x1A) & 0x100) == 0x100);
}

TEST_CASE_FIXTURE(TmcFixture, "[R21] SVPWM (bit 8 of 0x1A) is cleared for a BLDC motor with SVPWM disabled") {
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.motconf.motor_type = MotorType::BLDC;
		t.conf.motconf.svpwm = false;
		chip1.setReg(0x1A, 0x100); // stale: bit set
	});
	CHECK((chip1.reg(0x1A) & 0x100) == 0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R21] SVPWM (bit 8 of 0x1A) stays clear for a stepper motor even when SVPWM is enabled") {
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.motconf.motor_type = MotorType::STEPPER;
		t.conf.motconf.svpwm = true;
	});
	CHECK((chip1.reg(0x1A) & 0x100) == 0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R21] SVPWM (bit 8 of 0x1A) stays clear for a DC motor even when SVPWM is enabled") {
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.motconf.motor_type = MotorType::DC;
		t.conf.motconf.svpwm = true;
	});
	CHECK((chip1.reg(0x1A) & 0x100) == 0);
}

// ---------------------------------------------------------------------------------------------------------------------
// R22 ADC setup
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R22] MDEC_A is written to 0x07 bits 15..0 and MDEC_B to bits 31..16") {
	// DS 0x07 p78. 0x1234 | 0x0567 << 16 = 0x05671234 (HD)
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.mdecA = 0x1234;
		t.conf.mdecB = 0x0567;
	});
	CHECK(chip1.reg(0x07) == 0x05671234);
}

TEST_CASE_FIXTURE(TmcFixture, "[R22] MCLK_A and MCLK_B are written to 0x05 and 0x06") {
	// DS 0x05 / 0x06 p78 (dsADC_MCLK_A / _B, 32 bit)
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.mclkA = 0x11111111;
		t.conf.mclkB = 0x22222222;
	});
	CHECK(chip1.reg(0x05) == 0x11111111);
	CHECK(chip1.reg(0x06) == 0x22222222);
}

TEST_CASE_FIXTURE(TmcFixture, "[R22] sel_nclk_mclk_i bits 4 and 20 of 0x04 are set when both MCLK values are non-zero") {
	// DS 0x04 p77-78: bit 4 = sel_nclk_mclk_i (ADC A), bit 20 = sel_nclk_mclk_i (ADC B) as used by the charter
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.mclkA = 0x20000000;
		t.conf.mclkB = 0x20000000;
		chip1.setReg(0x04, 0);
	});
	CHECK((chip1.reg(0x04) & 0x00100010) == 0x00100010);
}

TEST_CASE_FIXTURE(TmcFixture, "[R22] sel_nclk_mclk_i of a channel with MCLK 0 is cleared and the other channel keeps its bit set") {
	// MCLK_A = 0 -> bit 4 cleared (stale value has it set); MCLK_B != 0 -> bit 20 set
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.mclkA = 0;
		t.conf.mclkB = 0x20000000;
		chip1.setReg(0x04, 0x00000010);
	});
	CHECK((chip1.reg(0x04) & 0x00000010) == 0);
	CHECK((chip1.reg(0x04) & 0x00100000) == 0x00100000);
}

TEST_CASE_FIXTURE(TmcFixture, "[R22] characterization: ADC_I_SELECT 0x0A is written with I0 <- ADC0, I1 <- ADC1, UX <- I0, V <- I2, WY <- I1") {
	// DS 0x0A p79: I0_SELECT [7:0], I1_SELECT [15:8], UX_SELECT [25:24], V_SELECT [27:26], WY_SELECT [29:28].
	// The channel mapping is board specific (charter L1): layout from the DS, mapping from the firmware (0x18000100).
	// Hand decode of the fields: 0x18000100 -> bits 7:0 = 0, 15:8 = 1, 25:24 = 0, 27:26 = 2, 29:28 = 1.
	auto tmc = initialised(*this, [&](TMC_1 &) { chip1.setReg(0x0A, 0); });
	const uint32_t v = chip1.reg(0x0A);
	CHECK(((v >> 0) & 0xFF) == 0);
	CHECK(((v >> 8) & 0xFF) == 1);
	CHECK(((v >> 24) & 0x3) == 0);
	CHECK(((v >> 26) & 0x3) == 2);
	CHECK(((v >> 28) & 0x3) == 1);
}

// ---------------------------------------------------------------------------------------------------------------------
// R23 ADC offsets and scales
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R23] initialize writes the I0 offset and scale to 0x09 and the I1 offset and scale to 0x08") {
	// DS 0x08 / 0x09 p78: offset [15:0], scale [31:16]. I0: 0x0150 << 16 | 0x1234 = 0x01501234; I1: 0x0180 << 16 | 0x4321
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.adc_I0_offset = 0x1234;
		t.conf.adc_I0_scale = 0x0150;
		t.conf.adc_I1_offset = 0x4321;
		t.conf.adc_I1_scale = 0x0180;
	});
	CHECK(chip1.reg(0x09) == 0x01501234);
	CHECK(chip1.reg(0x08) == 0x01804321);
}

TEST_CASE_FIXTURE(TmcFixture, "[R23] setAdcOffset keeps the scale half of 0x08 and 0x09") {
	auto tmc = initialised(*this);
	chip1.setReg(0x09, 0xAAAA5555);
	chip1.setReg(0x08, 0xBBBB6666);
	tmc->setAdcOffset(0x1234, 0x4321);
	CHECK(chip1.reg(0x09) == 0xAAAA1234);
	CHECK(chip1.reg(0x08) == 0xBBBB4321);
}

TEST_CASE_FIXTURE(TmcFixture, "[R23] setAdcScale keeps the offset half of 0x08 and 0x09") {
	auto tmc = initialised(*this);
	chip1.setReg(0x09, 0xAAAA5555);
	chip1.setReg(0x08, 0xBBBB6666);
	tmc->setAdcScale(0x0150, 0x0180);
	CHECK(chip1.reg(0x09) == 0x01505555);
	CHECK(chip1.reg(0x08) == 0x01806666);
}

// ---------------------------------------------------------------------------------------------------------------------
// R24 PI precision
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R24] each PI precision flag sets its own bit of CONFIG_DATA 62 (current I, current P, velocity I, velocity P, position I, position P = bit 0..5)") {
	// DS 0x4D/0x4E p93-94 and DS §7.1 p73: CONFIG_ADDR 62 ADVANCED_PI_REPRESENT, bit 0 current I ... bit 5 position P
	for (int bit = 0; bit < 6; bit++) {
		INFO("flag index " << bit);
		auto tmc = initialised(*this, [&](TMC_1 &t) {
			t.pidPrecision = {};
			bool *flags[6] = {&t.pidPrecision.current_I, &t.pidPrecision.current_P, &t.pidPrecision.velocity_I,
							  &t.pidPrecision.velocity_P, &t.pidPrecision.position_I, &t.pidPrecision.position_P};
			*flags[bit] = true;
			chip1.setConfigWord(62, 0xFFFF); // stale
		});
		CHECK(chip1.configWord(62) == (1u << bit));
	}
}

TEST_CASE_FIXTURE(TmcFixture, "[R24] the PI precision is written as an indirect pair: CONFIG_ADDR 62 first, then the data to CONFIG_DATA") {
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.pidPrecision = {};
		t.pidPrecision.position_P = true;
	});
	bool found = false;
	for (const auto &w : chip1.configWrites()) {
		if (w.first == 62) {
			found = true;
			CHECK(w.second == 0x20); // bit 5 (HD)
		}
	}
	CHECK(found);
}

TEST_CASE_FIXTURE(TmcFixture, "[R24] the default PI precision (all Q8.8) writes 0 to CONFIG_DATA 62") {
	// DS: bit = 0 -> Q8.8. TMC4671PidPrecision defaults are all false (TMC4671.h)
	auto tmc = initialised(*this, [&](TMC_1 &) { chip1.setConfigWord(62, 0xFFFF); });
	CHECK(chip1.configWord(62) == 0);
}

// ---------------------------------------------------------------------------------------------------------------------
// R25 flux biquad (lowpass 350 Hz, Q 0.7 at the PWM rate)
// ---------------------------------------------------------------------------------------------------------------------

namespace {
/** Last data written to CONFIG_DATA for each CONFIG_ADDR in the flux range 25..31 */
std::map<uint32_t, uint32_t> fluxBiquadWrites(const FakeTmc4671 &c) {
	std::map<uint32_t, uint32_t> m;
	for (const auto &w : c.configWrites()) {
		if (w.first >= 25 && w.first <= 31) {
			m[w.first] = w.second;
		}
	}
	return m;
}
// fs = PWM rate = 100 MHz / 4096 (HD: PWM_MAXCNT 4095)
const double FS = 100e6 / 4096.0;
} // namespace

TEST_CASE_FIXTURE(TmcFixture, "[R25] the flux biquad is written through CONFIG_ADDR 25, 26, 28, 29, 30, 31 in this order") {
	// DS 0x4E p94: 25 biquad_f a1, 26 a2, 28 b0, 29 b1, 30 b2, 31 enable (27 is not used)
	auto tmc = initialised(*this);
	std::vector<uint32_t> seq;
	for (uint32_t a : configAddrSequence(chip1)) {
		if (a >= 25 && a <= 31) {
			seq.push_back(a);
		}
	}
	CHECK(seq == std::vector<uint32_t>{25, 26, 28, 29, 30, 31});
}

TEST_CASE_FIXTURE(TmcFixture, "[R25] the flux biquad is enabled (enable word 1)") {
	auto tmc = initialised(*this);
	auto m = fluxBiquadWrites(chip1);
	REQUIRE(m.count(31) == 1);
	CHECK(m[31] == 1);
}

// Tolerance (lead decision L19, replaces +-2 LSB of L2): each coefficient within 1e-6 in real units (about 537 LSB of
// Q3.29) of the double precision oracle; DC gain within 1e-3. The driver computes in single precision (float ulp of a
// value in [1,2) is 64 LSB; on target also arm_sin/arm_cos), a wrong sign, frequency or Q is off by far more.
// The 350 Hz / Q 0.7 parameters are firmware choices without an outside source (L1): characterization.
constexpr double kCoefTol = 1e-6;
constexpr double kDcTol = 1e-3;

TEST_CASE_FIXTURE(TmcFixture, "[R25] characterization: the flux biquad numerator b0, b1, b2 is the 350 Hz Q 0.7 bilinear lowpass in Q3.29") {
	// DS §4.8.1 p61 eq 40-42: b = round(b_z * 2^29). Oracle (HD): K = tan(pi fc/fs), norm = 1/(1+K/Q+K^2),
	// b0 = K^2 norm, b1 = 2 b0, b2 = b0; fs = 100 MHz / 4096.
	auto tmc = initialised(*this);
	auto m = fluxBiquadWrites(chip1);
	LowpassOracle o(350.0, FS, 0.7);
	REQUIRE(m.count(28) == 1);
	REQUIRE(m.count(29) == 1);
	REQUIRE(m.count(30) == 1);
	CHECK(std::abs(double(int32_t(m[28])) / Q29 - o.b0) <= kCoefTol);
	CHECK(std::abs(double(int32_t(m[29])) / Q29 - o.b1) <= kCoefTol);
	CHECK(std::abs(double(int32_t(m[30])) / Q29 - o.b2) <= kCoefTol);
}

TEST_CASE_FIXTURE(TmcFixture, "[R25] characterization: the flux biquad feedback coefficients are the negated lowpass denominator in Q3.29") {
	// DS §4.8.1 p61 eq 43-44: a1 = round(-a1_z * 2^29), a2 = round(-a2_z * 2^29), a_z = denominator coefficients
	// of 1 + a1_z z^-1 + a2_z z^-2 (HD: a1_z = 2 (K^2-1) norm, a2_z = (1-K/Q+K^2) norm). The firmware negates in
	// Filters.cpp calcBiquad. For 350 Hz: a1 ~ +1.87, a2 ~ -0.879.
	auto tmc = initialised(*this);
	auto m = fluxBiquadWrites(chip1);
	LowpassOracle o(350.0, FS, 0.7);
	REQUIRE(m.count(25) == 1);
	REQUIRE(m.count(26) == 1);
	CHECK(std::abs(double(int32_t(m[25])) / Q29 - (-o.denA1)) <= kCoefTol);
	CHECK(std::abs(double(int32_t(m[26])) / Q29 - (-o.denA2)) <= kCoefTol);
}

TEST_CASE_FIXTURE(TmcFixture, "[R25] characterization: the flux biquad has unity DC gain (b0+b1+b2) / (2^29 - a1 - a2) = 1") {
	// DS eq 33 at DC: Y (1 - a1 - a2) = X (b0 + b1 + b2); a lowpass has gain 1 (HD). Tolerance 1e-3 (L19).
	auto tmc = initialised(*this);
	auto m = fluxBiquadWrites(chip1);
	REQUIRE(m.size() == 6);
	const double num = double(int32_t(m[28])) + double(int32_t(m[29])) + double(int32_t(m[30]));
	const double den = Q29 - double(int32_t(m[25])) - double(int32_t(m[26]));
	REQUIRE(den != 0.0);
	CHECK(std::abs(num / den - 1.0) <= kDcTol);
}

// ---------------------------------------------------------------------------------------------------------------------
// R26 brake limits, status mask, PI registers, controller structure
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R26] initialize writes the brake chopper limits to 0x75 as low | high << 16") {
	// DS 0x75 p103: ADC_VM_LIMIT_LOW u16 [15:0], HIGH u16 [31:16]. 0x1234 | 0x9ABC << 16 = 0x9ABC1234 (HD).
	// high >= 0x8000 reaches the sign bit of the int the driver shifts (see file header: same on target).
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.conf.hwconf.brakeLimLow = 0x1234;
		t.conf.hwconf.brakeLimHigh = 0x9ABC;
		chip1.setReg(0x75, 0);
	});
	CHECK(chip1.reg(0x75) == 0x9ABC1234);
}

TEST_CASE_FIXTURE(TmcFixture, "[R26] initialize sets the status mask 0x7D to adc_i_clipped (bit 26) and not_PLL_locked (bit 19)") {
	// DS 0x7C/0x7D p107, bit positions from the charter table: (1 << 26) | (1 << 19) = 0x04000000 | 0x00080000 = 0x04080000
	auto tmc = initialised(*this, [&](TMC_1 &) { chip1.setReg(0x7D, 0); });
	CHECK(chip1.reg(0x7D) == 0x04080000);
}

TEST_CASE_FIXTURE(TmcFixture, "[R26] initialize writes the four PI registers as I in bits 15..0 and P in bits 31..16") {
	// DS 0x54 flux, 0x56 torque, 0x58 velocity, 0x5A position (p96). Values with P >= 0x8000.
	// flux 0x8001 << 16 | 0x1111 = 0x80011111; torque 0x9002 / 0x2222; velocity 0xA003 / 0x3333; position 0xB004 / 0x4444
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.curPids.fluxI = 0x1111;
		t.curPids.fluxP = 0x8001;
		t.curPids.torqueI = 0x2222;
		t.curPids.torqueP = 0x9002;
		t.curPids.velocityI = 0x3333;
		t.curPids.velocityP = 0xA003;
		t.curPids.positionI = 0x4444;
		t.curPids.positionP = 0xB004;
	});
	CHECK(chip1.reg(0x54) == 0x80011111);
	CHECK(chip1.reg(0x56) == 0x90022222);
	CHECK(chip1.reg(0x58) == 0xA0033333);
	CHECK(chip1.reg(0x5A) == 0xB0044444);
}

TEST_CASE_FIXTURE(TmcFixture, "[R26] initialize sets the controller structure bit 31 of 0x63 for sequential PI") {
	// DS 0x63 p97: MODE_PID_TYPE bit 31, 1 = sequential/advanced
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.curPids.sequentialPI = true;
		chip1.setReg(0x63, 0);
	});
	CHECK((chip1.reg(0x63) & 0x80000000u) == 0x80000000u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R26] initialize clears the controller structure bit 31 of 0x63 for classic PI") {
	auto tmc = initialised(*this, [&](TMC_1 &t) {
		t.curPids.sequentialPI = false;
		chip1.setReg(0x63, 0x80000000u);
	});
	CHECK((chip1.reg(0x63) & 0x80000000u) == 0);
}

// ---------------------------------------------------------------------------------------------------------------------
// R27 no stale torque/flux target
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R27] initialize clears a stale torque/flux target in 0x64") {
	// DS 0x64 p97: FLUX_TARGET [15:0], TORQUE_TARGET [31:16]
	auto tmc = initialised(*this, [&](TMC_1 &) { chip1.setReg(0x64, 0x12345678); });
	CHECK(chip1.reg(0x64) == 0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R27] the last write of 0x64 before the PWM is switched to FOC writes 0") {
	auto tmc = makeTmc1();
	chip1.setReg(0x64, 0x12345678); // stale target of a previous run
	reachShutdown(*tmc);
	const auto &log = chip1.log();
	int foc = findWrite(log, 0x1A, 0, [](uint32_t d) { return (d & 0xFF) == 7; });
	REQUIRE(foc >= 0);
	int last64 = -1;
	for (int i = 0; i < foc; i++) {
		if (log[i].write && log[i].addr == 0x64) {
			last64 = i;
		}
	}
	REQUIRE(last64 >= 0);
	CHECK(log[last64].data == 0);
	CHECK(chip1.reg(0x64) == 0);
}

// ---------------------------------------------------------------------------------------------------------------------
// R28 setupDriver limits and thread
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R28] setupDriver writes the configured limits to 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62") {
	// DS 0x5D-0x62 p96-97. 0x5C (torque/flux ddt limit) is not in DS Rev 2.07 (charter Q1): not asserted.
	// 0x61 = -1000 as u32 = 2^32 - 1000 = 0xFFFFFC18 (HD)
	auto tmc = makeTmc1();
	tmc->curLimits.pid_uq_ud = 0x1234;
	tmc->curLimits.pid_torque_flux = 0x2345;
	tmc->curLimits.pid_acc_lim = 0x12345678;
	tmc->curLimits.pid_vel_lim = 0x23456789;
	tmc->curLimits.pid_pos_low = -1000;
	tmc->curLimits.pid_pos_high = 5000;
	tmc->setupDriver();
	CHECK(chip1.reg(0x5D) == 0x1234);
	CHECK(chip1.reg(0x5E) == 0x2345);
	CHECK(chip1.reg(0x5F) == 0x12345678);
	CHECK(chip1.reg(0x60) == 0x23456789);
	CHECK(chip1.reg(0x61) == 0xFFFFFC18);
	CHECK(chip1.reg(0x62) == 5000);
}

TEST_CASE_FIXTURE(TmcFixture, "[R28] setupDriver starts the TMC thread and the constructor does not") {
	auto tmc = makeTmc1();
	CHECK(HostRtos::findTask("TMC") == nullptr);
	tmc->setupDriver();
	CHECK(HostRtos::findTask("TMC") != nullptr);
}

// ---------------------------------------------------------------------------------------------------------------------
// R29 PWM FOC mode and enable pin only with power and calibrated ADC
// ---------------------------------------------------------------------------------------------------------------------

namespace {
bool enablePinHigh() { return HostTarget::outputPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin); }
bool focWritten(const FakeTmc4671 &c, size_t from = 0) {
	return findWrite(c.log(), 0x1A, from, [](uint32_t d) { return (d & 0xFF) == 7; }) >= 0;
}
} // namespace

TEST_CASE_FIXTURE(TmcFixture, "[R29] without supply voltage the enable pin stays low and the PWM is never switched to FOC") {
	// DS §4.9 p63: PWM_CHOP 7 = centered FOC. The ADC raw words are valid so the pre-power calibration passes.
	auto tmc = makeTmc1();
	chip1.setAdcRaw(0, 0x7FFF7FFF);
	chip1.setMicrosPerTransfer(10);
	powerOff();
	tmc->setupDriver();
	runThreadFor(*tmc, 5000);
	CHECK(tmc->getState() == TMC_ControlState::waitPower);
	CHECK_FALSE(enablePinHigh());
	CHECK_FALSE(focWritten(chip1));
	CHECK((chip1.reg(0x1A) & 0xFF) == 0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R29] after the supply appears the enable pin is high and the PWM is in FOC mode, written after the power condition") {
	auto tmc = makeTmc1();
	chip1.setAdcRaw(0, 0x7FFF7FFF);
	chip1.setMicrosPerTransfer(10);
	powerOff();
	tmc->setupDriver();
	runThreadFor(*tmc, 3000);
	REQUIRE_FALSE(enablePinHigh());
	bool pinAtFoc = false;
	chip1.onWrite(0x1A, [&](FakeTmc4671 &, uint32_t d) {
		if ((d & 0xFF) == 7) {
			pinAtFoc = enablePinHigh();
		}
	});
	const size_t sizeAtPower = chip1.log().size();
	powerOn();
	REQUIRE(runUntilState(*tmc, TMC_ControlState::Shutdown, 10000));
	int foc = findWrite(chip1.log(), 0x1A, sizeAtPower, [](uint32_t d) { return (d & 0xFF) == 7; });
	CHECK(foc >= static_cast<int>(sizeAtPower));
	CHECK(pinAtFoc); // the enable pin is set before the PWM mode write (enablePin.set() then setPwm)
	CHECK(enablePinHigh());
	CHECK((chip1.reg(0x1A) & 0xFF) == 7);
}

TEST_CASE_FIXTURE(TmcFixture, "[R29] initializeWithPower sets the enable pin only after the offsets of the successful calibration were written") {
	auto tmc = initialised(*this);
	chip1.setAdcRaw(0, 0x7FFF7FFF);
	chip1.setMicrosPerTransfer(10);
	REQUIRE_FALSE(enablePinHigh()); // initialize() alone does not enable
	bool pinAtOffsetWrite = true;
	bool offsetWritten = false;
	chip1.onWrite(0x09, [&](FakeTmc4671 &, uint32_t d) {
		if ((d & 0xFFFF) == 0x7FFF) { // calibrated mean of raw 0x7FFF
			pinAtOffsetWrite = enablePinHigh();
			offsetWritten = true;
		}
	});
	const size_t before = chip1.log().size();
	tmc->initializeWithPower();
	CHECK(offsetWritten);
	CHECK_FALSE(pinAtOffsetWrite);
	CHECK(enablePinHigh());
	int ofs = findWrite(chip1.log(), 0x09, before, [](uint32_t d) { return (d & 0xFFFF) == 0x7FFF; });
	int foc = findWrite(chip1.log(), 0x1A, before, [](uint32_t d) { return (d & 0xFF) == 7; });
	REQUIRE(ofs >= 0);
	REQUIRE(foc >= 0);
	CHECK(ofs < foc);
}

TEST_CASE_FIXTURE(TmcFixture, "[R29] initializeWithPower with a failed ADC calibration leaves the enable pin low and the PWM off") {
	// Raw 0x688F = 0x7FFF - 6000: the mean is more than 5000 away from 0x7FFF (TMC_ADCOFFSETFAIL) -> calibration fails.
	// The first sample is accepted by the sign-flip filter (|33415 - 26767| = 6648 < 10000).
	auto tmc = initialised(*this);
	chip1.setAdcRaw(0, 0x688F688F);
	chip1.setMicrosPerTransfer(10);
	tmc->initializeWithPower();
	CHECK_FALSE(enablePinHigh());
	CHECK_FALSE(focWritten(chip1));
	CHECK((chip1.reg(0x1A) & 0xFF) == 0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R29] initialize alone leaves the PWM off (0x1A bits 7..0 = 0)") {
	auto tmc = initialised(*this, [&](TMC_1 &) { chip1.setReg(0x1A, 0x07); });
	CHECK((chip1.reg(0x1A) & 0xFF) == 0);
	CHECK_FALSE(enablePinHigh());
}

} // TEST_SUITE
