/*
 * TMC4671 driver, group G9 "Persistent settings (flash)" (requirements R110 - R114 of driver/TMC4671_REQUIREMENTS.md).
 *
 * Sources: DS = TMC4671-LA datasheet V2.07 (cited "DS 0xNN"), AD = flash addresses of
 * Firmware/FFBoard/UserExtensions/Inc/eeprom_addresses.h (TMC_1: ADR_TMC1_* 0x320 - 0x32C, TMC_2: ADR_TMC2_*
 * 0x360 - 0x36C), HD = hand derivation in the comment from the bit layout documented in TMC4671.cpp
 * (encodeMotToInt / encodeEncHallMisc / saveFlash comments: "0-1 motor type, 2 !svpwm, 3-5 encoder type, 6-15 pole
 * pairs", "b0 npol ... b10 sequentialPI, b11-15 hardware version", "freq 13 bit | mode << 13"), FW = firmware
 * behaviour only (test name starts with "characterization:").
 *
 * Method: tests that check the flash words call saveFlash() and compare with words derived by hand. Tests that check
 * the restore write hand derived words into the fake flash first and construct a new driver object (the constructor
 * calls restoreFlash). Round trips save with one object, destroy it, reset the chip and construct a new object.
 * Nothing sleeps; the ADC offset tests (R114) use the fixture's bounded thread helpers.
 *
 * Host vs target: flash words are uint16_t, every conversion is an integer conversion defined on both (the int16_t
 * posOffsetFromIndex <-> uint16_t word uses modular arithmetic). The torque filter uses float math inside
 * setTorqueFilter, only the stored integers are asserted.
 */
#include "doctest.h"
#include "TmcFixture.h"

#include <map>
#include <set>

namespace {

using FlashMap = std::map<uint16_t, uint16_t>;

/** Flash word at `adr`, the test fails if it is absent */
uint16_t word(uint16_t adr) {
	const FlashMap &f = HostPlatform::flash();
	auto it = f.find(adr);
	REQUIRE_MESSAGE(it != f.end(), "flash word 0x" << std::hex << adr << " was not written");
	return it->second;
}

bool hasWord(uint16_t adr) { return HostPlatform::flash().count(adr) != 0; }

std::set<uint16_t> flashKeys() {
	std::set<uint16_t> keys;
	for (const auto &kv : HostPlatform::flash()) {
		keys.insert(kv.first);
	}
	return keys;
}

/** A complete set of persistent settings. */
struct Settings {
	uint8_t hw;
	MotorType motor;
	bool svpwm;
	EncoderType_TMC enc;
	uint16_t poles;
	uint32_t cpr;
	// misc word
	bool npol, reversed, abAsN, curI, curP, useIndex, combine, invert, fluxDiss, interp;
	// sequentialPI is passed separately (bit 10 of the misc word)
	// pids, offsets
	uint16_t torqueP, torqueI, fluxP, fluxI;
	int16_t encOffset;
	uint16_t offsetFlux;
	// torque filter
	uint16_t filterFreq;
	uint8_t filterMode;
};

/*
 * Settings of TMC_1 (flash 0x320...). Hand derived words:
 *  0x320 motor config: motor BLDC = 3 (bits 1:0 = 0x3), svpwm false -> bit 2 = 0x4, encoder abn = 1 -> bits 5:3 = 0x08,
 *        1023 pole pairs << 6 = 0xFFC0. 0x3 + 0x4 + 0x8 + 0xFFC0 = 0xFFCF.
 *  0x321 cpr 4000 = 0x0FA0 (enctype abn: the abn cpr is stored).
 *  0x322 misc: npol b0 0x1, reversed b1 0x2, curI b3 0x8, useIndex b5 0x20, combine b6 0x40, invert b7 0x80,
 *        interpolation b9 0x200, hardware version 1 << 11 = 0x800; abAsN, curP, fluxDiss, seqPI are 0.
 *        0x1 + 0x2 + 0x8 + 0x20 + 0x40 + 0x80 + 0x200 + 0x800 = 0xAEB.
 *  0x325 encoder offset -2 = 0xFFFE (two's complement 16 bit). 0x326 flux offset 0x0DEF.
 *  0x327..0x32A torque P/I, flux P/I as set. 0x32C torque filter: freq 8191 = 0x1FFF (13 bit full scale),
 *        mode peak = 3 << 13 = 0x6000 -> 0x7FFF.
 */
const Settings S1 = {1, MotorType::BLDC, false, EncoderType_TMC::abn, 1023, 4000, true, true, false, true, false,
					 true, true, true, false, true, 0x0123, 0x0456, 0x0789, 0x0ABC, -2, 0x0DEF, 8191, 3};

/*
 * Settings of TMC_2 (flash 0x360...). Hand derived words:
 *  0x360 stepper = 2, svpwm true -> bit 2 = 0, encoder sincos = 2 -> 2 << 3 = 0x10, 50 pole pairs << 6 = 3200 = 0xC80.
 *        0x2 + 0x10 + 0xC80 = 0xC92.
 *  0x361 cpr: enctype is not abn, so the AENC cpr is stored: 1000 = 0x03E8.
 *  0x362 misc: abAsN b2 0x4, curP b4 0x10, fluxDiss b8 0x100, seqPI b10 0x400, hardware version 6 << 11 = 0x3000;
 *        npol, reversed, curI, useIndex, combine, invert, interpolation are 0.
 *        0x4 + 0x10 + 0x100 + 0x400 + 0x3000 = 0x3514.
 *  0x365 encoder offset 300 = 0x012C. 0x366 flux offset 0x5555. 0x367..0x36A as set.
 *  0x36C torque filter: freq 1000 = 0x3E8, mode lowpass = 1 << 13 = 0x2000 -> 0x23E8.
 */
const Settings S2 = {6, MotorType::STEPPER, true, EncoderType_TMC::sincos, 50, 1000, false, false, true, false, true,
					 false, false, false, true, false, 0x1111, 0x2222, 0x3333, 0x4444, 300, 0x5555, 1000, 1};
// sequentialPI of S1 is false (misc b10 = 0), of S2 true (b10 = 0x400 in 0x3514)

/** Applies `s` to a driver object through its public interface (no flash access) */
void apply(TMC4671 &t, const Settings &s, bool seqPIvalue) {
	t.setHwType(s.hw); // first: it re-applies motor and encoder type
	t.conf.motconf.motor_type = s.motor;
	t.conf.motconf.svpwm = s.svpwm;
	t.conf.motconf.enctype = s.enc;
	t.conf.motconf.pole_pairs = s.poles;
	t.abnconf.cpr = s.cpr;
	t.aencconf.cpr = s.cpr;
	t.abnconf.npol = s.npol;
	t.conf.encoderReversed = s.reversed;
	t.abnconf.ab_as_n = s.abAsN;
	t.pidPrecision.current_I = s.curI;
	t.pidPrecision.current_P = s.curP;
	t.abnconf.useIndex = s.useIndex;
	t.conf.combineEncoder = s.combine;
	t.conf.invertForce = s.invert;
	t.conf.enableFluxDissipation = s.fluxDiss;
	t.hallconf.interpolation = s.interp;
	t.curPids.sequentialPI = seqPIvalue;
	t.curPids.torqueP = s.torqueP;
	t.curPids.torqueI = s.torqueI;
	t.curPids.fluxP = s.fluxP;
	t.curPids.fluxI = s.fluxI;
	t.abnconf.posOffsetFromIndex = s.encOffset;
	t.maxOffsetFlux = s.offsetFlux;
	CommandHelpers::run(t, "trqbq_f", CMDtype::set, s.filterFreq);
	CommandHelpers::run(t, "trqbq_mode", CMDtype::set, s.filterMode);
}

} // namespace

TEST_SUITE("TMC4671 flash") {

// ---------------------------------------------------------------------------------------------------------------
// R110 flash words of saveFlash (hand derived), both instances
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R110] saveFlash of TMC_1 writes the motor config, cpr, misc, pid, offset and filter words at 0x320.. (AD)") {
	auto tmc = makeTmc1();
	apply(*tmc, S1, false);
	HostPlatform::flash().clear();
	tmc->saveFlash();
	CHECK(word(0x320) == 0xFFCF); // ADR_TMC1_MOTCONF, HD above
	CHECK(word(0x321) == 4000);	  // ADR_TMC1_CPR
	CHECK(word(0x322) == 0xAEB);  // ADR_TMC1_ENCA
	CHECK(word(0x325) == 0xFFFE); // ADR_TMC1_ENC_OFFSET
	CHECK(word(0x326) == 0x0DEF); // ADR_TMC1_OFFSETFLUX
	CHECK(word(0x327) == 0x0123); // ADR_TMC1_TORQUE_P
	CHECK(word(0x328) == 0x0456); // ADR_TMC1_TORQUE_I
	CHECK(word(0x329) == 0x0789); // ADR_TMC1_FLUX_P
	CHECK(word(0x32A) == 0x0ABC); // ADR_TMC1_FLUX_I
	CHECK(word(0x32C) == 0x7FFF); // ADR_TMC1_TRQ_FILT
}

TEST_CASE_FIXTURE(TmcFixture, "[R110] saveFlash of TMC_2 writes the same fields at 0x360.. (AD)") {
	auto tmc = makeTmc2();
	apply(*tmc, S2, true);
	HostPlatform::flash().clear();
	tmc->saveFlash();
	CHECK(word(0x360) == 0x0C92); // ADR_TMC2_MOTCONF, HD above
	CHECK(word(0x361) == 1000);	  // ADR_TMC2_CPR
	CHECK(word(0x362) == 0x3514); // ADR_TMC2_ENCA
	CHECK(word(0x365) == 0x012C); // ADR_TMC2_ENC_OFFSET
	CHECK(word(0x366) == 0x5555); // ADR_TMC2_OFFSETFLUX
	CHECK(word(0x367) == 0x1111); // ADR_TMC2_TORQUE_P
	CHECK(word(0x368) == 0x2222); // ADR_TMC2_TORQUE_I
	CHECK(word(0x369) == 0x3333); // ADR_TMC2_FLUX_P
	CHECK(word(0x36A) == 0x4444); // ADR_TMC2_FLUX_I
	CHECK(word(0x36C) == 0x23E8); // ADR_TMC2_TRQ_FILT
}

TEST_CASE_FIXTURE(TmcFixture, "[R110] saveFlash of TMC_1 writes exactly the ten words 0x320 0x321 0x322 0x325 - 0x32A 0x32C and nothing else") {
	// 0x323 / 0x324 (ADC offsets) are only written by saveAdcParams, 0x32B (phiE offset) is not used (AD).
	auto tmc = makeTmc1();
	HostPlatform::flash().clear();
	tmc->saveFlash();
	const std::set<uint16_t> expected = {0x320, 0x321, 0x322, 0x325, 0x326, 0x327, 0x328, 0x329, 0x32A, 0x32C};
	CHECK(flashKeys() == expected);
}

TEST_CASE_FIXTURE(TmcFixture, "[R110] saveFlash of TMC_2 writes exactly the ten words 0x360 0x361 0x362 0x365 - 0x36A 0x36C and nothing else") {
	auto tmc = makeTmc2();
	HostPlatform::flash().clear();
	tmc->saveFlash();
	const std::set<uint16_t> expected = {0x360, 0x361, 0x362, 0x365, 0x366, 0x367, 0x368, 0x369, 0x36A, 0x36C};
	CHECK(flashKeys() == expected);
}

TEST_CASE_FIXTURE(TmcFixture, "[R110] the pole pair field of the motor config word is 10 bits wide (1023 stored, 1024 wraps to 0)") {
	// HD: motor NONE (0), svpwm true (bit 2 = 0), encoder NONE (0). Word = poles << 6 with the mask 0x3FF:
	// 1023 -> 0xFFC0; 1024 = 0x400 -> masked to 0 -> 0x0000. Only the static encoder is used: no driver side effects.
	TMC4671MotConf c;
	c.motor_type = MotorType::NONE;
	c.svpwm = true;
	c.enctype = EncoderType_TMC::NONE;
	c.pole_pairs = 1023;
	CHECK(TMC4671::encodeMotToInt(c) == 0xFFC0);
	c.pole_pairs = 1024;
	CHECK(TMC4671::encodeMotToInt(c) == 0x0000);
}

TEST_CASE_FIXTURE(TmcFixture, "[R110] the cpr word holds the AENC cpr when the encoder is not ABN and the ABN cpr when it is") {
	auto tmc = makeTmc1();
	tmc->abnconf.cpr = 1111; // HD: 1111 = 0x457
	tmc->aencconf.cpr = 2222; // HD: 2222 = 0x8AE
	tmc->conf.motconf.enctype = EncoderType_TMC::abn;
	HostPlatform::flash().clear();
	tmc->saveFlash();
	CHECK(word(0x321) == 1111);
	tmc->conf.motconf.enctype = EncoderType_TMC::hall;
	tmc->saveFlash();
	CHECK(word(0x321) == 2222);
}

TEST_CASE_FIXTURE(TmcFixture, "[R110] the torque filter word packs a 13 bit frequency and a 3 bit mode (freq 1000, mode notch = 2)") {
	// HD: 1000 = 0x3E8, mode 2 << 13 = 0x4000 -> 0x43E8
	auto tmc = makeTmc1();
	CommandHelpers::run(*tmc, "trqbq_f", CMDtype::set, 1000);
	CommandHelpers::run(*tmc, "trqbq_mode", CMDtype::set, 2);
	HostPlatform::flash().clear();
	tmc->saveFlash();
	CHECK(word(0x32C) == 0x43E8);
}

// ---------------------------------------------------------------------------------------------------------------
// R110 restore of hand derived words, both instances
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R110] a TMC_1 constructed with the words of 0x320.. restores motor config, misc bits, pids, offsets and filter") {
	auto &f = HostPlatform::flash();
	f[0x320] = 0xFFCF; // BLDC, !svpwm, abn, 1023 pole pairs (HD at S1)
	f[0x321] = 4000;
	f[0x322] = 0xAEB; // misc bits of S1, hardware version 1
	f[0x325] = 0xFFFE;
	f[0x326] = 0x0DEF;
	f[0x327] = 0x0123;
	f[0x328] = 0x0456;
	f[0x329] = 0x0789;
	f[0x32A] = 0x0ABC;
	f[0x32C] = 0x7FFF;
	auto tmc = makeTmc1();
	CHECK(tmc->conf.motconf.motor_type == MotorType::BLDC);
	CHECK_FALSE(tmc->conf.motconf.svpwm);
	CHECK(tmc->conf.motconf.enctype == EncoderType_TMC::abn);
	CHECK(tmc->conf.motconf.pole_pairs == 1023);
	CHECK(tmc->curPids.torqueP == 0x0123);
	CHECK(tmc->curPids.torqueI == 0x0456);
	CHECK(tmc->curPids.fluxP == 0x0789);
	CHECK(tmc->curPids.fluxI == 0x0ABC);
	CHECK(tmc->abnconf.posOffsetFromIndex == -2); // 0xFFFE as int16
	CHECK(tmc->maxOffsetFlux == 0x0DEF);
	CHECK(CommandHelpers::get(*tmc, "trqbq_f") == 8191);
	CHECK(CommandHelpers::get(*tmc, "trqbq_mode") == 3);
}

TEST_CASE_FIXTURE(TmcFixture, "[R110] a TMC_2 constructed with the words of 0x360.. restores them and ignores the TMC_1 words") {
	auto &f = HostPlatform::flash();
	f[0x320] = 0xFFCF; // TMC_1 words: must not be used by TMC_2
	f[0x327] = 0x0123;
	f[0x360] = 0x0C92; // stepper, svpwm, sincos, 50 pole pairs (HD at S2)
	f[0x361] = 1000;
	f[0x362] = 0x3514;
	f[0x365] = 0x012C;
	f[0x366] = 0x5555;
	f[0x367] = 0x1111;
	f[0x368] = 0x2222;
	f[0x369] = 0x3333;
	f[0x36A] = 0x4444;
	f[0x36C] = 0x23E8;
	auto tmc = makeTmc2();
	CHECK(tmc->conf.motconf.motor_type == MotorType::STEPPER);
	CHECK(tmc->conf.motconf.svpwm);
	CHECK(tmc->conf.motconf.enctype == EncoderType_TMC::sincos);
	CHECK(tmc->conf.motconf.pole_pairs == 50);
	CHECK(tmc->curPids.torqueP == 0x1111);
	CHECK(tmc->curPids.torqueI == 0x2222);
	CHECK(tmc->curPids.fluxP == 0x3333);
	CHECK(tmc->curPids.fluxI == 0x4444);
	CHECK(tmc->abnconf.posOffsetFromIndex == 300);
	CHECK(tmc->maxOffsetFlux == 0x5555);
	CHECK(CommandHelpers::get(*tmc, "trqbq_f") == 1000);
	CHECK(CommandHelpers::get(*tmc, "trqbq_mode") == 1);
}

TEST_CASE_FIXTURE(TmcFixture, "[R110] the misc word 0xAEB restores every bit b0 - b9 and the hardware version 1") {
	// HD (see S1): b0 npol, b1 reversed, b3 curI, b5 useIndex, b6 combine, b7 invert, b9 interpolation, hw 1.
	HostPlatform::flash()[0x322] = 0xAEB;
	auto tmc = makeTmc1();
	CHECK(tmc->abnconf.apol);
	CHECK(tmc->abnconf.bpol);
	CHECK(tmc->abnconf.npol);
	CHECK(tmc->conf.encoderReversed);
	CHECK(tmc->abnconf.rdir);
	CHECK(tmc->aencconf.rdir);
	CHECK(tmc->hallconf.direction);
	CHECK_FALSE(tmc->abnconf.ab_as_n);
	CHECK(tmc->pidPrecision.current_I);
	CHECK(tmc->pidPrecision.velocity_I);
	CHECK(tmc->pidPrecision.position_I);
	CHECK_FALSE(tmc->pidPrecision.current_P);
	CHECK_FALSE(tmc->pidPrecision.velocity_P);
	CHECK(tmc->abnconf.useIndex);
	CHECK(tmc->conf.combineEncoder);
	CHECK(tmc->conf.invertForce);
	CHECK_FALSE(tmc->conf.enableFluxDissipation);
	CHECK(tmc->hallconf.interpolation);
	CHECK_FALSE(tmc->curPids.sequentialPI);
	CHECK(CommandHelpers::get(*tmc, "tmcHwType") == 1);
}

TEST_CASE_FIXTURE(TmcFixture, "[R110] the misc word 0x3514 restores the complementary bits b2 b4 b8 b10 and the hardware version 6") {
	// HD (see S2): b2 abAsN, b4 curP, b8 fluxDiss, b10 seqPI, hw 6; everything else 0 (0x4 + 0x10 + 0x100 + 0x400 + 0x3000).
	HostPlatform::flash()[0x362] = 0x3514;
	auto tmc = makeTmc2();
	CHECK_FALSE(tmc->abnconf.apol);
	CHECK_FALSE(tmc->conf.encoderReversed);
	CHECK(tmc->abnconf.ab_as_n);
	CHECK_FALSE(tmc->pidPrecision.current_I);
	CHECK(tmc->pidPrecision.current_P);
	CHECK(tmc->pidPrecision.velocity_P);
	CHECK_FALSE(tmc->abnconf.useIndex);
	CHECK_FALSE(tmc->conf.combineEncoder);
	CHECK_FALSE(tmc->conf.invertForce);
	CHECK(tmc->conf.enableFluxDissipation);
	CHECK_FALSE(tmc->hallconf.interpolation);
	CHECK(tmc->curPids.sequentialPI);
	CHECK(CommandHelpers::get(*tmc, "tmcHwType") == 6);
}

TEST_CASE_FIXTURE(TmcFixture, "[R110] characterization: the invert force bit is only restored when the combine encoder bit is set") {
	// HD: 0x80 = b7 invertForce alone (combine b6 clear), hardware version 6 -> 0x3000 | 0x80 = 0x3080.
	// FW: restoreEncHallMisc ANDs invertForce with combineEncoder. No external spec.
	HostPlatform::flash()[0x322] = 0x3080;
	auto tmc = makeTmc1();
	CHECK_FALSE(tmc->conf.combineEncoder);
	CHECK_FALSE(tmc->conf.invertForce);
}

// ---------------------------------------------------------------------------------------------------------------
// R110 round trips
// ---------------------------------------------------------------------------------------------------------------

namespace {
void expectSettings(TMC4671 &t, const Settings &s, bool seqPIvalue) {
	CHECK(t.conf.motconf.motor_type == s.motor);
	CHECK(t.conf.motconf.svpwm == s.svpwm);
	CHECK(t.conf.motconf.enctype == s.enc);
	CHECK(t.conf.motconf.pole_pairs == s.poles);
	CHECK(t.getEncCpr() == s.cpr);
	CHECK(t.abnconf.npol == s.npol);
	CHECK(t.conf.encoderReversed == s.reversed);
	CHECK(t.abnconf.ab_as_n == s.abAsN);
	CHECK(t.pidPrecision.current_I == s.curI);
	CHECK(t.pidPrecision.current_P == s.curP);
	CHECK(t.abnconf.useIndex == s.useIndex);
	CHECK(t.conf.combineEncoder == s.combine);
	CHECK(t.conf.invertForce == s.invert);
	CHECK(t.conf.enableFluxDissipation == s.fluxDiss);
	CHECK(t.hallconf.interpolation == s.interp);
	CHECK(t.curPids.sequentialPI == seqPIvalue);
	CHECK(t.conf.hwconf.hwVersion == s.hw);
	CHECK(t.curPids.torqueP == s.torqueP);
	CHECK(t.curPids.torqueI == s.torqueI);
	CHECK(t.curPids.fluxP == s.fluxP);
	CHECK(t.curPids.fluxI == s.fluxI);
	CHECK(t.abnconf.posOffsetFromIndex == s.encOffset);
	CHECK(t.maxOffsetFlux == s.offsetFlux);
	CHECK(CommandHelpers::get(t, "trqbq_f") == s.filterFreq);
	CHECK(CommandHelpers::get(t, "trqbq_mode") == s.filterMode);
}
} // namespace

TEST_CASE_FIXTURE(TmcFixture, "[R110] settings saved by one TMC_1 object are restored by a new TMC_1 object (round trip at 0x320..)") {
	{
		auto a = makeTmc1();
		apply(*a, S1, false);
		a->saveFlash();
	}
	chip1.reset();
	auto b = makeTmc1();
	expectSettings(*b, S1, false);
}

TEST_CASE_FIXTURE(TmcFixture, "[R110] settings saved by one TMC_2 object are restored by a new TMC_2 object (round trip at 0x360..)") {
	{
		auto a = makeTmc2();
		apply(*a, S2, true);
		a->saveFlash();
	}
	chip2.reset();
	auto b = makeTmc2();
	expectSettings(*b, S2, true);
}

TEST_CASE_FIXTURE(TmcFixture, "[R110] saving TMC_2 does not change the words of TMC_1") {
	auto a = makeTmc1();
	apply(*a, S1, false);
	a->saveFlash();
	const FlashMap before = HostPlatform::flash();
	auto b = makeTmc2();
	apply(*b, S2, true);
	b->saveFlash();
	for (const auto &kv : before) {
		CHECK_MESSAGE(word(kv.first) == kv.second, "TMC_1 word 0x" << std::hex << kv.first << " changed");
	}
}

// ---------------------------------------------------------------------------------------------------------------
// R111 restore re-applies the settings to the chip (DS layouts)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R111] restoring a cpr of 4000 writes ABN_DECODER_PPR 0x26 and AENC_DECODER_PPR 0x40") {
	// DS 0x26: u24 decoder_ppr; DS 0x40: s16 periods per revolution. HD: 4000 = 0x0FA0 in both.
	HostPlatform::flash()[0x321] = 4000;
	chip1.reset();
	auto tmc = makeTmc1();
	CHECK(chip1.reg(0x26) == 0x00000FA0);
	CHECK(chip1.reg(0x40) == 0x00000FA0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R111] restoring a BLDC motor with 1023 pole pairs writes MOTOR_TYPE_N_POLE_PAIRS 0x1B") {
	// DS 0x1B: motor type in bits 23:16 (3 = three phase BLDC), pole pairs in bits 15:0.
	// HD: (3 << 16) | 1023 = 0x30000 | 0x3FF = 0x000303FF. Flash 0x320 = 0xFFCF (HD at S1), no encoder word needed.
	HostPlatform::flash()[0x320] = 0xFFCF;
	chip1.reset();
	auto tmc = makeTmc1();
	CHECK(chip1.reg(0x1B) == 0x000303FF);
}

TEST_CASE_FIXTURE(TmcFixture, "[R111] characterization: restoring hardware version 1 (v1.0) writes its brake limits to 0x75 and its bridge dead time to 0x19") {
	// Layout DS 0x75: ADC_VM_LIMIT_LOW 15:0, HIGH 31:16. DS 0x19: BBM_L 7:0, BBM_H 15:8.
	// FW values of the board table (CFG.cpp conf1_0): low 52400 = 0xCCB0, high 52800 = 0xCE40, bbm 20 = 0x14.
	// HD: 0x75 = (0xCE40 << 16) | 0xCCB0 = 0xCE40CCB0; 0x19 = 0x14 | (0x14 << 8) = 0x1414.
	// The default (no flash) is hardware v1.3 with 50700 / 50900 / 40, so the values differ from the default.
	HostPlatform::flash()[0x322] = 0x0800; // hardware version 1 << 11, all other misc bits 0
	chip1.reset();
	auto tmc = makeTmc1();
	CHECK(CommandHelpers::get(*tmc, "tmcHwType") == 1);
	CHECK(chip1.reg(0x75) == 0xCE40CCB0);
	CHECK(chip1.reg(0x19) == 0x1414);
}

TEST_CASE_FIXTURE(TmcFixture, "[R111] a round trip re-applies cpr, motor type and pole pairs to a chip that lost its registers") {
	{
		auto a = makeTmc1();
		apply(*a, S1, false);
		a->setCpr(4000);
		a->saveFlash();
	}
	chip1.reset();
	REQUIRE(chip1.reg(0x26) != 0x00000FA0); // the chip forgot (power cycle)
	REQUIRE(chip1.reg(0x1B) != 0x000303FF);
	auto b = makeTmc1();
	CHECK(chip1.reg(0x26) == 0x00000FA0);
	CHECK(chip1.reg(0x1B) == 0x000303FF); // DS 0x1B: (3 << 16) | 1023
}

TEST_CASE_FIXTURE(TmcFixture, "[R111] restoring TMC_2 writes the registers of chip 2 and leaves chip 1 untouched") {
	HostPlatform::flash()[0x361] = 1000; // HD: 0x3E8
	chip1.reset();
	chip2.reset();
	auto tmc = makeTmc2();
	CHECK(chip2.reg(0x26) == 0x000003E8);
	CHECK(chip1.writesTo(0x26).empty());
	CHECK(chip1.log().empty());
}

// ---------------------------------------------------------------------------------------------------------------
// R112 ABN cpr above 65535 (B-T20)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R112] an ABN cpr of 65535 (the largest 16 bit value) survives save and restore") {
	{
		auto a = makeTmc1();
		apply(*a, S1, false);
		a->abnconf.cpr = 65535;
		a->saveFlash();
	}
	chip1.reset();
	auto b = makeTmc1();
	CHECK(b->getEncCpr() == 65535);
	CHECK(chip1.reg(0x26) == 65535);
}

TEST_CASE_FIXTURE(TmcFixture, "[R112] an ABN cpr of 100000 survives save and restore" *
								  KNOWN_ISSUE("B-T20: the cpr is stored in one 16 bit flash word, 100000 is truncated to 34464")) {
	// DS 0x26: ABN_DECODER_PPR is u24, so 100000 (0x186A0) is a valid value. HD of the defect: 100000 mod 65536 = 34464.
	{
		auto a = makeTmc1();
		apply(*a, S1, false);
		a->abnconf.cpr = 100000;
		a->saveFlash();
	}
	chip1.reset();
	auto b = makeTmc1();
	CHECK(b->getEncCpr() == 100000);
	CHECK(chip1.reg(0x26) == 100000);
}

// ---------------------------------------------------------------------------------------------------------------
// R113 position P precision bit (B-T21)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R113] the velocity P precision bit follows the stored current P bit") {
	// HD: b4 (0x10) set, hardware version 6 (0x3000) -> 0x3010
	HostPlatform::flash()[0x322] = 0x3010;
	auto tmc = makeTmc1();
	CHECK(tmc->pidPrecision.current_P);
	CHECK(tmc->pidPrecision.velocity_P);
}

TEST_CASE_FIXTURE(TmcFixture, "[R113] the position P precision bit follows the stored current P bit" *
								  KNOWN_ISSUE("B-T21: restoreEncHallMisc assigns velocity_P twice and never restores position_P")) {
	// Intent (from the pattern velocity_I / position_I follow current_I): all P precisions share the stored bit.
	// HD: b4 (0x10) set, hardware version 6 (0x3000) -> 0x3010
	HostPlatform::flash()[0x322] = 0x3010;
	auto tmc = makeTmc1();
	CHECK(tmc->pidPrecision.position_P);
}

// ---------------------------------------------------------------------------------------------------------------
// R114 ADC offsets (characterization)
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "[R114] characterization: the ADC calibration at power-up does not write the offsets to flash") {
	// FW: only saveAdcParams (full calibration) writes 0x323 / 0x324; calibrateAdcOffset in initializeWithPower does not.
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	CHECK_FALSE(hasWord(0x323));
	CHECK_FALSE(hasWord(0x324));
	// the offsets came from the measurement: raw 0x7FFF (DS 0x02) -> DS 0x09 / 0x08 offset 0x7FFF
	CHECK((chip1.reg(0x09) & 0xFFFF) == 0x7FFF);
	CHECK((chip1.reg(0x08) & 0xFFFF) == 0x7FFF);
}

TEST_CASE_FIXTURE(TmcFixture, "[R114] characterization: a successful full calibration stores the measured ADC offsets in 0x323 and 0x324") {
	// HD: ADC_RAW_DATA 0x02 (DS p77) I1 in 31:16, I0 in 15:0. Constant raw I0 = 0x8123 (33059), I1 = 0x7E00 (32256):
	// the mean of a constant is the constant; both are within 5000 of 0x7FFF (|0x8123 - 0x7FFF| = 292, |0x7E00 - 0x7FFF| = 511).
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	chip1.setAdcRaw(0, 0x7E008123);
	REQUIRE_FALSE(hasWord(0x323));
	CommandHelpers::run(*tmc, "calibrate", CMDtype::get);
	bool stored = false;
	for (int i = 0; i < 600 && !stored; i++) { // bounded: 60 s simulated at most
		runThreadFor(*tmc, 100);
		stored = hasWord(0x323) && hasWord(0x324);
	}
	REQUIRE_MESSAGE(stored, "full calibration did not store the ADC offsets within 60 s simulated");
	CHECK(word(0x323) == 0x8123);
	CHECK(word(0x324) == 0x7E00);
}

TEST_CASE_FIXTURE(TmcFixture, "[R114] characterization: a failed full calibration does not store ADC offsets") {
	// HD: raw I0 = 0x9757 = 38743: |38743 - 0x7FFF (32767)| = 5976 > TMC_ADCOFFSETFAIL 5000 -> calibration fails.
	// It is within 10000 of the start offset, so the sample filter of calibrateAdcOffset accepts the samples
	// (all-zero words would divide by zero: B-T16, not reached here).
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	chip1.setAdcRaw(0, 0x7FFF9757);
	CommandHelpers::run(*tmc, "calibrate", CMDtype::get);
	runThreadFor(*tmc, 10000); // two attempts of 500 ms plus the delays
	CHECK_FALSE(hasWord(0x323));
	CHECK_FALSE(hasWord(0x324));
}

TEST_CASE_FIXTURE(TmcFixture, "[R114] characterization: a stored ADC offset with a neutral current reading skips the calibration at power-up") {
	// HD: stored I0 0x8123, I1 0x7E00. The chip's raw ADC words are 0x7FFF (a calibration would write 0x7FFF to
	// DS 0x09 / 0x08). PID_TORQUE_FLUX_ACTUAL 0x69 stays 0: checkAdc sees a bias of 0 < 100 and accepts the offsets.
	HostPlatform::flash()[0x323] = 0x8123;
	HostPlatform::flash()[0x324] = 0x7E00;
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	CHECK((chip1.reg(0x09) & 0xFFFF) == 0x8123);
	CHECK((chip1.reg(0x08) & 0xFFFF) == 0x7E00);
}

TEST_CASE_FIXTURE(TmcFixture, "[R114] characterization: a stored ADC offset with a biased current reading is replaced by a new calibration") {
	// HD: 0x69 = 0x03E80000: torque (31:16) = 1000, flux 0 -> mean bias 1000 >= 100, checkAdc rejects the stored offsets
	// and calibrateAdcOffset measures the raw 0x7FFF again.
	HostPlatform::flash()[0x323] = 0x8123;
	HostPlatform::flash()[0x324] = 0x7E00;
	chip1.setReg(0x69, 0x03E80000);
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	CHECK((chip1.reg(0x09) & 0xFFFF) == 0x7FFF);
	CHECK((chip1.reg(0x08) & 0xFFFF) == 0x7FFF);
}

}
