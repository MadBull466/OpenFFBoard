/*
 * PWM driver, timer set-up and output (phase P14, work order PWO). Requirements R2.1 to R2.7 and R3.1 to R3.11 of
 * Tests/claude/charters/driver/PWM_REQUIREMENTS.md (lead decisions L1 to L4).
 *
 * Unit under test: MotorPWM::setPwmSpeed / setMode / freqToPeriodPsc / turn / startMotor / stopMotor / emergencyStop /
 * motorReady (MotorPWM.cpp). Everything is driven through the public interface and observed at the fake TIM1 through the
 * PwmFixture view (ARR, PSC, CCR1-4, CCER, BDTR.MOE, CR1.CEN, derived pulse / duty / frequency). The direction inversion
 * is switched with the registered "dir" command (CommandHelpers, id resolved by name), because the member is private.
 *
 * Oracles (charter): HD = hand derivation in a comment next to the assertion (168 MHz timer clock, MotorPWM::timerConfig);
 * HAL / RM = register semantics of the fake (CCxE, MOE, CEN, ConfigChannel loads CCRx from Pulse = 0); NAMES = the labels
 * "3khz" "9khz" "17khz" "24khz" and "20ms" "15ms" "10ms" "5ms"; RC = pulse 1000 to 2000 us, centre 1500 us (the sign of the
 * mapping is not documented: characterization); IF = MotorDriver interface (emergencyStop(false) = stopMotor,
 * emergencyStop(true) = startMotor, motorReady true).
 *
 * Facts used (hand derivation, charter "Facts"):
 *  - non-RC ARR = 168000000 / f (integer division), PSC = ARR / 65535 = 0: LOW 3 kHz 56000, MID 9 kHz 18666 (18666.67),
 *    HIGH 17 kHz 9882 (9882.35), VERYHIGH 24 kHz 7000. RC: PSC = 168000000 / 2000000 = 84, ARR 40000 / 30000 / 20000 / 10000.
 *  - timer tick = (PSC + 1) / 168 us; RC: tFreq = 168 / 85 ticks per us (float), one tick = 0.506 us.
 *  - CENTERED: CCR = (32767 + p) * ARR / 65535 (integer, truncated). PWM_DIR / DUAL: CCR = |p| * ARR / 32767 (integer).
 *  - tolerance of the duty tests: one count, as the charter says (the integer truncation of the firmware).
 *  - channels: RC ch1; CENTERED ch1; PWM_DIR ch1 (duty), ch3 / ch4 (direction pair); DUAL ch1 / ch2 (cpp_target_config.cpp:62).
 *
 * Host versus target: tFreq and the RC CCR are float (single precision on both, the tests use a one tick tolerance);
 * |p| * ARR is int arithmetic, the largest product 32768 * 56000 = 1.835e9 fits int32_t on both; -power on int16_t is
 * computed in int and narrowed to int16_t (implementation defined, modular on GCC / clang host and ARM: R3.8); the
 * float to uint32_t conversion of the clipped (non-negative) RC value is in range.
 *
 * Characterizations (NEEDS REVIEW): R3.2 sign of the RC mapping (positive power gives the short pulse); R3.2 the RC mapping
 * saturates at half scale (|p| >= 16384 reaches 1000 / 2000 us, +-1000 us per full scale before the clip); R3.8 inversion
 * of -32768; R3.10 the signal keeps running after stopMotor.
 * KNOWN_ISSUE: B-PWM-1 (R2.4): the RC frame period is 1.2 % longer than the name (divider 85, not 84).
 *   Measured (PSC 84, (PSC + 1)(ARR + 1) / 168 MHz): LOW 85 * 40001 / 168e6 = 20.2386 ms (named 20 ms, +1.19 %),
 *   MID 85 * 30001 / 168e6 = 15.1791 ms (15 ms, +1.19 %), HIGH 85 * 20001 / 168e6 = 10.1196 ms (10 ms, +1.19 %),
 *   VERYHIGH 85 * 10001 / 168e6 = 5.0600 ms (5 ms, +1.20 %). Tolerance 0.1 %.
 *
 * R2.6 is checked with the call log of the fake (PwmFixture::log): Stop, Config, Start:OK per channel in order.
 */
#include "doctest.h"
#include "PwmFixture.h"
#include "CommandHelpers.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace {

using Snap = std::array<uint32_t, 4>;

struct SpeedRow {
	SpeedPWM_DRV speed;
	const char *name;
	uint32_t arr;     // hand derived, non-RC
	double nominalHz; // NAMES
	uint32_t rcArr;   // hand derived, RC
	double rcMs;      // NAMES
};

const SpeedRow kSpeeds[] = {
	{SpeedPWM_DRV::LOW, "LOW", 56000, 3000.0, 40000, 20.0},
	{SpeedPWM_DRV::MID, "MID", 18666, 9000.0, 30000, 15.0},
	{SpeedPWM_DRV::HIGH, "HIGH", 9882, 17000.0, 20000, 10.0},
	{SpeedPWM_DRV::VERYHIGH, "VERYHIGH", 7000, 24000.0, 10000, 5.0},
};

struct ModeRow {
	ModePWM_DRV mode;
	const char *name;
};

const ModeRow kModes[] = {
	{ModePWM_DRV::RC_PWM, "RC_PWM"},
	{ModePWM_DRV::CENTERED_PWM, "CENTERED_PWM"},
	{ModePWM_DRV::PWM_DIR, "PWM_DIR"},
	{ModePWM_DRV::PWM_DUAL, "PWM_DUAL"},
};

// B-PWM-4: CENTERED computes (0x7fff + p) * period in int32_t. At speed LOW (period 56000) that overflows INT32_MAX once
// 0x7fff + p > 38347 (p > 5580): undefined behaviour. Every test that turns CENTERED with a larger power therefore runs at
// speed MID (period 18666, largest product 65534 * 18666 = 1.22e9, fits) via kSafeSpeed; the only LOW tests with larger
// powers are the KNOWN_ISSUE_UB case. PWM_DIR / DUAL: |p| * period <= 32768 * 56000 = 1.84e9 fits int32_t, no overflow.
constexpr SpeedPWM_DRV kSafeSpeed = SpeedPWM_DRV::MID;

const ModeRow kNonRcModes[] = {kModes[1], kModes[2], kModes[3]};

Snap snap() {
	return {PwmFixture::ccr(1), PwmFixture::ccr(2), PwmFixture::ccr(3), PwmFixture::ccr(4)};
}

std::string snapStr(const Snap &s) {
	return "CCR1 " + std::to_string(s[0]) + " CCR2 " + std::to_string(s[1]) + " CCR3 " + std::to_string(s[2]) + " CCR4 " +
		   std::to_string(s[3]);
}

/** Driver in the given mode and speed (set from the default state), optionally started. Uses only the public interface. */
std::unique_ptr<MotorPWM> build(PwmFixture &f, ModePWM_DRV mode, SpeedPWM_DRV speed, bool start = true) {
	auto pwm = f.makePwm();
	REQUIRE(pwm != nullptr);
	pwm->setMode(mode);
	pwm->setPwmSpeed(speed);
	if (start) {
		pwm->startMotor();
	}
	return pwm;
}

void setInvert(MotorPWM &pwm, bool on) {
	REQUIRE(CommandHelpers::run(pwm, "dir", CMDtype::set, on ? 1 : 0) == CommandStatus::OK);
}

double tickUs() { return (PwmFixture::psc() + 1.0) / 168.0; } // one timer tick in us, hand derivation above

bool within(double actual, double expected, double tol) { return std::fabs(actual - expected) <= tol + 1e-9; }

// Exact hand-derived oracles (integer formulas of the header comment). The ARR is the hand-derived value of the table row
// (never read back from the register). int64 so that the test itself cannot overflow (B-PWM-4).
int64_t exactCentered(int p, uint32_t arrHand) { return static_cast<int64_t>(std::max(0, 32767 + p)) * arrHand / 65535; }
int64_t exactMagnitude(int p, uint32_t arrHand) { return static_cast<int64_t>(std::abs(p)) * arrHand / 32767; }
int64_t ccrOf(int ch) { return static_cast<int64_t>(PwmFixture::ccr(ch)); }

/** Neutral (power 0) output of a mode, from the hand derivation. Records every channel. */
void checkNeutral(ModePWM_DRV mode, uint32_t arrHand = 56000) {
	INFO(snapStr(snap()));
	switch (mode) {
	case ModePWM_DRV::RC_PWM:
		CHECK(within(PwmFixture::pulseUs(1), 1500.0, tickUs())); // RC centre
		CHECK(PwmFixture::ccr(2) == 0);
		CHECK(PwmFixture::ccr(3) == 0);
		CHECK(PwmFixture::ccr(4) == 0);
		break;
	case ModePWM_DRV::CENTERED_PWM:
		CHECK(ccrOf(1) == exactCentered(0, arrHand)); // 50 %: 32767 * ARR / 65535, LOW 27999, MID 9332
		CHECK(PwmFixture::ccr(2) == 0);
		CHECK(PwmFixture::ccr(3) == 0);
		CHECK(PwmFixture::ccr(4) == 0);
		break;
	case ModePWM_DRV::PWM_DIR:
		CHECK(PwmFixture::ccr(1) == 0);
		CHECK(PwmFixture::ccr(2) == 0);
		CHECK(PwmFixture::duty(3) == 1.0); // power >= 0: dir always high
		CHECK(PwmFixture::ccr(4) == 0);
		break;
	case ModePWM_DRV::PWM_DUAL:
		CHECK(snap() == Snap{0, 0, 0, 0});
		break;
	}
}

/** Channels (1-based) a mode does not use */
std::vector<int> unusedChannels(ModePWM_DRV mode) {
	switch (mode) {
	case ModePWM_DRV::RC_PWM:
	case ModePWM_DRV::CENTERED_PWM: return {2, 3, 4};
	case ModePWM_DRV::PWM_DIR: return {2};
	case ModePWM_DRV::PWM_DUAL: return {3, 4};
	}
	return {};
}

/** Invert the direction: output for power p with inversion on equals the output of -p without it (R3.7) */
void checkInversionSwapsSign(PwmFixture &f, ModePWM_DRV mode) {
	auto pwm = build(f, mode, kSafeSpeed);
	for (int p : {-32767, -20000, -1, 0, 1, 20000, 32767}) {
		INFO("mode " << static_cast<int>(mode) << " power " << p);
		setInvert(*pwm, false);
		pwm->turn(static_cast<int16_t>(-p));
		const Snap plain = snap();
		pwm->turn(0); // move the output away so that a stale value cannot make the comparison pass
		setInvert(*pwm, true);
		pwm->turn(static_cast<int16_t>(p));
		const Snap inverted = snap();
		CHECK_MESSAGE(inverted == plain, "plain(-p) " << snapStr(plain) << " inverted(p) " << snapStr(inverted));
	}
}

/** Expected HAL call log of one re-initialisation (R2.6): per channel 1..4 Stop, Config, Start:OK; no Start:*:ERR */
std::vector<std::string> expectedReinitLog() {
	std::vector<std::string> expected;
	for (int ch = 1; ch <= 4; ++ch) {
		expected.push_back("Stop:" + std::to_string(ch));
		expected.push_back("Config:" + std::to_string(ch));
		expected.push_back("Start:" + std::to_string(ch) + ":OK");
	}
	return expected;
}

} // namespace

TEST_SUITE("PWM output") {

// ---------------------------------------------------------------- R2: timer set-up

TEST_CASE_FIXTURE(PwmFixture, "[R2.1 timer] default construction gives 3 kHz: ARR 56000, PSC 0, CCR 0") {
	auto pwm = makePwm();
	// HD: 168000000 / 3000 = 56000, PSC = 56000 / 65535 = 0; the driver is not active, so turn(0) writes nothing and the
	// CCR stay at the ConfigChannel Pulse 0
	CHECK(arr() == 56000);
	CHECK(psc() == 0);
	CHECK(snap() == Snap{0, 0, 0, 0});
}

TEST_CASE_FIXTURE(PwmFixture, "[R2.1 timer] default construction enables all four channels, the main output and the counter") {
	auto pwm = makePwm();
	for (int ch = 1; ch <= 4; ++ch) {
		INFO("channel " << ch);
		CHECK(channelOn(ch));
	}
	CHECK(mainOutput());
	CHECK(counterOn());
}

TEST_CASE_FIXTURE(PwmFixture, "[R2.1 timer] default construction configures all four channels as PWM mode 1") {
	auto pwm = makePwm();
	// RM0090 TIM1 CCMR1 / CCMR2: OC1M bits 4-6, OC2M bits 12-14 (CCMR1), OC3M / OC4M the same in CCMR2; PWM mode 1 = 0b110
	CHECK((tim().CCMR1 & 0x70U) == 0x60U);
	CHECK((tim().CCMR1 & 0x7000U) == 0x6000U);
	CHECK((tim().CCMR2 & 0x70U) == 0x60U);
	CHECK((tim().CCMR2 & 0x7000U) == 0x6000U);
}

TEST_CASE_FIXTURE(PwmFixture, "[R2.2 timer] every speed of every non-RC mode gives its ARR with prescaler 0") {
	for (const ModeRow &m : kNonRcModes) {
		for (const SpeedRow &s : kSpeeds) {
			INFO("mode " << std::string(m.name) << " speed " << std::string(s.name));
			auto pwm = build(*this, m.mode, s.speed, false);
			// HD: LOW 168e6/3000 = 56000, MID 168e6/9000 = 18666 (18666.67), HIGH 168e6/17000 = 9882 (9882.35),
			// VERYHIGH 168e6/24000 = 7000; all below 65535 so PSC = 0
			CHECK(arr() == s.arr);
			CHECK(psc() == 0);
		}
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R2.2 timer] every speed of every non-RC mode gives the nominal frequency within 0.02 percent") {
	for (const ModeRow &m : kNonRcModes) {
		for (const SpeedRow &s : kSpeeds) {
			INFO("mode " << std::string(m.name) << " speed " << std::string(s.name) << " frequency " << outputFrequencyHz());
			auto pwm = build(*this, m.mode, s.speed, false);
			// NAMES: 3 / 9 / 17 / 24 kHz; f = 168e6 / ((PSC + 1)(ARR + 1)), e.g. HIGH 168e6 / 9883 = 16998.9 Hz (-0.007 %),
			// worst case VERYHIGH 168e6 / 7001 = 23996.6 Hz (-0.0143 %)
			CHECK(within(outputFrequencyHz(), s.nominalHz, s.nominalHz * 0.0002));
		}
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R2.3 timer] RC mode gives ARR 40000 / 30000 / 20000 / 10000 with prescaler 84") {
	for (const SpeedRow &s : kSpeeds) {
		INFO("speed " << std::string(s.name));
		auto pwm = build(*this, ModePWM_DRV::RC_PWM, s.speed, false);
		// HD: PSC = 168000000 / 2000000 = 84; periods 40000 / 30000 / 20000 / 10000 as coded in setPwmSpeed
		CHECK(arr() == s.rcArr);
		CHECK(psc() == 84);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R2.4 timer] the RC frame time is 20 / 15 / 10 / 5 ms within 0.1 percent" *
								  KNOWN_ISSUE("B-PWM-1: the prescaler register holds 84 = 168e6/2e6, but the timer divides by PSC + 1 = 85, so the frame is 1.2 % too long: LOW 20.2386 ms, MID 15.1791 ms, HIGH 10.1196 ms, VERYHIGH 5.0600 ms")) {
	for (const SpeedRow &s : kSpeeds) {
		auto pwm = build(*this, ModePWM_DRV::RC_PWM, s.speed, false);
		INFO("speed " << std::string(s.name) << " frame " << periodUs() / 1000.0 << " ms, named " << s.rcMs << " ms");
		// NAMES: (PSC + 1)(ARR + 1) / 168 MHz, e.g. LOW 85 * 40001 / 168e6 = 20.2386 ms
		CHECK(within(periodUs() / 1000.0, s.rcMs, s.rcMs * 0.001));
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R2.5 timer] changing the mode applies the period and prescaler of the new mode") {
	auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, SpeedPWM_DRV::LOW, false);
	REQUIRE(arr() == 56000);
	REQUIRE(psc() == 0);
	pwm->setMode(ModePWM_DRV::RC_PWM);
	CHECK(arr() == 40000); // HD: RC LOW
	CHECK(psc() == 84);
	pwm->setMode(ModePWM_DRV::PWM_DIR);
	CHECK(arr() == 56000); // HD: back to 3 kHz
	CHECK(psc() == 0);
	pwm->setPwmSpeed(SpeedPWM_DRV::VERYHIGH);
	pwm->setMode(ModePWM_DRV::PWM_DUAL);
	CHECK(arr() == 7000); // the speed is kept over a mode change: 168e6 / 24000
	CHECK(psc() == 0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R2.5 timer] changing the mode of a stopped driver reloads every channel with CCR 0") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DIR, SpeedPWM_DRV::LOW);
	pwm->turn(20000);
	pwm->stopMotor(); // PWM_DIR neutral: ch3 = 0xffff (always high)
	REQUIRE(ccr(3) == 0xffff);
	pwm->setMode(ModePWM_DRV::PWM_DUAL);
	// ConfigChannel loads CCRx from Pulse = 0 for every channel; the driver is inactive so turn(0) writes nothing
	CHECK_MESSAGE(snap() == (Snap{0, 0, 0, 0}), snapStr(snap()));
}

TEST_CASE_FIXTURE(PwmFixture, "[R2.5 timer] setPwmSpeed and setMode on a started driver output the neutral value of the new set-up") {
	auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, SpeedPWM_DRV::MID);
	pwm->turn(20000);
	pwm->setPwmSpeed(SpeedPWM_DRV::HIGH);
	// HD: ARR 9882, neutral 32767 * 9882 / 65535 = 323803494 / 65535 = 4940.9 -> 4940 (not the old 20000 value)
	CHECK(ccr(1) == 4940);
	pwm->setMode(ModePWM_DRV::PWM_DIR);
	CHECK(ccr(3) == 0xffff); // power 0: dir always high
	CHECK(ccr(1) == 0);
	pwm->setMode(ModePWM_DRV::RC_PWM);
	CHECK(within(pulseUs(1), 1500.0, tickUs()));
}

TEST_CASE_FIXTURE(PwmFixture, "[R2.6 timer] setMode re-initialises every channel as Stop, Config, Start:OK in channel order") {
	auto pwm = makePwm();
	clearLog();
	pwm->setMode(ModePWM_DRV::RC_PWM);
	// pwmInitTimer per channel 1..4: HAL_TIM_PWM_Stop, HAL_TIM_PWM_ConfigChannel, HAL_TIM_PWM_Start (accepted: READY after Stop)
	CHECK(log() == expectedReinitLog());
}

TEST_CASE_FIXTURE(PwmFixture, "[R2.6 timer] setPwmSpeed re-initialises every channel as Stop, Config, Start:OK in channel order") {
	auto pwm = makePwm();
	clearLog();
	pwm->setPwmSpeed(SpeedPWM_DRV::HIGH);
	CHECK(log() == expectedReinitLog());
}

TEST_CASE_FIXTURE(PwmFixture, "[R2.7 timer] the ARR of the four non-RC frequencies fits 16 bit (informational)") {
	for (const SpeedRow &s : kSpeeds) {
		INFO("speed " << std::string(s.name));
		auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, s.speed, false);
		// HD: 168000000 / f = 56000 / 18666 / 9882 / 7000, all below 65535: no wrap of the uint16_t of freqToPeriodPsc
			// (the exact values are R2.2)
			CHECK(arr() <= 0xffffU);
	}
}

// ---------------------------------------------------------------- R3: turn

TEST_CASE_FIXTURE(PwmFixture, "[R3.1 turn] turn before startMotor writes nothing to the timer in any mode") {
	for (const ModeRow &m : kModes) {
		INFO("mode " << std::string(m.name));
		auto pwm = build(*this, m.mode, SpeedPWM_DRV::LOW, false);
		const Snap before = snap();
		const uint32_t a = arr();
		const uint32_t p = psc();
		for (int16_t power : {20000, -20000, 32767, -32768, 0}) {
			pwm->turn(power);
		}
		CHECK(snap() == before);
		CHECK(arr() == a);
		CHECK(psc() == p);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.1 turn] turn after stopMotor writes nothing to the timer in any mode") {
	for (const ModeRow &m : kModes) {
		INFO("mode " << std::string(m.name));
		auto pwm = build(*this, m.mode, kSafeSpeed);
		pwm->turn(20000);
		pwm->stopMotor();
		const Snap after = snap();
		for (int16_t power : {20000, -20000, 32767, -32768}) {
			pwm->turn(power);
		}
		CHECK_MESSAGE(snap() == after, "after stop " << snapStr(after) << " after turns " << snapStr(snap()));
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.2 turn] RC power +32767 / 0 / -32767 give pulses of 1000 / 1500 / 2000 us within one timer tick") {
	auto pwm = build(*this, ModePWM_DRV::RC_PWM, SpeedPWM_DRV::LOW);
	// HD: tick = 85 / 168 = 0.506 us; limits 1000 * tFreq = 1976.47 -> CCR 1976 (999.76 us), 1500 * tFreq -> 2964 (2964 * 85 / 168 = 1499.64 us),
	// 2000 * tFreq -> 3952 (3952 * 85 / 168 = 1999.52 us). The truncation error is below one tick (0.506 us); the margin of
	// 0.03 us at 2000 us is by design (one-tick tolerance of the charter). The sign is the characterization test below.
	pwm->turn(0);
	CHECK(within(pulseUs(1), 1500.0, tickUs()));
	pwm->turn(32767);
	const double pPlus = pulseUs(1);
	pwm->turn(-32767);
	const double pMinus = pulseUs(1);
	CHECK(within(std::min(pPlus, pMinus), 1000.0, tickUs()));
	CHECK(within(std::max(pPlus, pMinus), 2000.0, tickUs()));
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.2 turn] characterization: RC positive power gives the short pulse, negative power the long pulse") {
	auto pwm = build(*this, ModePWM_DRV::RC_PWM, SpeedPWM_DRV::LOW);
	// NEEDS REVIEW: no document states the sign. Code: pulse = 1500 us - p * 1000 / 32767 (MotorPWM.cpp:73-74)
	pwm->turn(32767);
	CHECK(within(pulseUs(1), 1000.0, tickUs()));
	pwm->turn(-32767);
	CHECK(within(pulseUs(1), 2000.0, tickUs()));
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.2 turn] RC power -32768 is clipped to the 2000 us limit") {
	auto pwm = build(*this, ModePWM_DRV::RC_PWM, SpeedPWM_DRV::LOW);
	// HD: (-32768 * 1000) / 32767 = -1000 (truncated), the code pulse before the clip is 2500 us, clipped to 2000 * tFreq
	// = 3952.9 -> CCR 3952 = 1999.52 us; never above 2000 us
	pwm->turn(-32768);
	CHECK(pulseUs(1) <= 2000.0 + 1e-9);
	CHECK(within(pulseUs(1), 2000.0, tickUs()));
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.2 turn] RC pulse stays within 1000 to 2000 us and is monotonic over a sweep of powers") {
	auto pwm = build(*this, ModePWM_DRV::RC_PWM, SpeedPWM_DRV::LOW);
	std::vector<int> powers = {-32768, -32767};
	for (int p = -32768 + 1024; p < 32767; p += 1024) {
		powers.push_back(p);
	}
	powers.push_back(32767);
	std::vector<uint32_t> ccrs;
	for (int p : powers) {
		pwm->turn(static_cast<int16_t>(p));
		INFO("power " << p << " pulse " << pulseUs(1));
		CHECK(pulseUs(1) >= 1000.0 - tickUs() - 1e-9);
		CHECK(pulseUs(1) <= 2000.0 + 1e-9);
		ccrs.push_back(ccr(1));
	}
	// monotonic in one direction (the direction itself is the sign characterization above)
	bool nonIncreasing = true;
	bool nonDecreasing = true;
	for (size_t i = 1; i < ccrs.size(); ++i) {
		nonIncreasing = nonIncreasing && ccrs[i] <= ccrs[i - 1];
		nonDecreasing = nonDecreasing && ccrs[i] >= ccrs[i - 1];
	}
	CHECK((nonIncreasing || nonDecreasing));
	CHECK(ccrs.front() != ccrs.back()); // a stuck output must fail
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.2 turn] characterization: RC pulse saturates at half scale, +-16384 already gives 1000 / 2000 us") {
	auto pwm = build(*this, ModePWM_DRV::RC_PWM, SpeedPWM_DRV::LOW);
	// NEEDS REVIEW: the code offset is p * 1000 / 32767 us, so a full scale power asks +-1000 us and the clip to 1000..2000
	// us cuts it at +-500 us. HD: p = 16384: (16384 * 1000) / 32767 = 500 -> 1500 - 500 = 1000 us; p = 8192:
	// (8192 * 1000) / 32767 = 250 -> 1250 us (a full scale of +-500 us would give 1375 us at p = 8192).
	pwm->turn(16384);
	CHECK(within(pulseUs(1), 1000.0, tickUs()));
	pwm->turn(-16384);
	CHECK(within(pulseUs(1), 2000.0, tickUs()));
	pwm->turn(8192);
	CHECK(within(pulseUs(1), 1250.0, tickUs()));
	// p = -8192: (-8192 * 1000) / 32767 = -250 (int division truncates toward zero) -> 1750 us, CCR 3458 (1749.6 us)
	pwm->turn(-8192);
	CHECK(within(pulseUs(1), 1750.0, tickUs()));
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.3 turn] RC pulse width in microseconds is the same for the four speeds") {
	for (const SpeedRow &s : kSpeeds) {
		INFO("speed " << std::string(s.name));
		auto pwm = build(*this, ModePWM_DRV::RC_PWM, s.speed);
		// HD: PSC is 84 for every speed, so tick and tFreq do not depend on the speed
		pwm->turn(0);
		CHECK(within(pulseUs(1), 1500.0, tickUs()));
		pwm->turn(32767);
		CHECK(within(pulseUs(1), 1000.0, tickUs()));
		pwm->turn(-32767);
		CHECK(within(pulseUs(1), 2000.0, tickUs()));
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.4 turn] CENTERED power 0 gives 32767 * ARR / 65535 for every speed") {
	for (const SpeedRow &s : kSpeeds) {
		INFO("speed " << std::string(s.name));
		auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, s.speed);
		pwm->turn(0);
		// HD: LOW 32767 * 56000 / 65535 = 27999.57 -> 27999 (a duty of 50 % within one count of ARR / 2 = 28000)
		CHECK(ccrOf(1) == exactCentered(0, s.arr));
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.4 turn] CENTERED at MID gives the hand-derived CCR for p = 0, 1, 32767, -32767") {
	auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, SpeedPWM_DRV::MID);
	// HD (ARR 18666): p = 0: 32767 * 18666 = 611628822, / 65535 = 9332.9 -> 9332; p = 1: 32768 * 18666 = 611647488 ->
	// 9333.2 -> 9333; p = 32767: 65534 * 18666 = 1223257644 -> 18665.7 -> 18665; p = -32767: pval 0 -> 0
	pwm->turn(0);
	CHECK(ccr(1) == 9332);
	pwm->turn(1);
	CHECK(ccr(1) == 9333);
	pwm->turn(32767);
	CHECK(ccr(1) == 18665);
	pwm->turn(-32767);
	CHECK(ccr(1) == 0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.4 turn] CENTERED powers give (32767 + p) * ARR / 65535 at the speeds without overflow") {
	// MID / HIGH / VERYHIGH only (B-PWM-4, see kSafeSpeed); LOW has its own cases below
	for (const SpeedRow &s : kSpeeds) {
		if (s.speed == SpeedPWM_DRV::LOW) {
			continue;
		}
		auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, s.speed);
		for (int p : {32767, 16384, -16384, -32767}) {
			pwm->turn(static_cast<int16_t>(p));
			INFO("speed " << std::string(s.name) << " power " << p << " ccr " << ccr(1));
			CHECK(ccrOf(1) == exactCentered(p, s.arr));
		}
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.4 turn] CENTERED at speed LOW gives (32767 + p) * 56000 / 65535 for powers up to 5000 and below 0") {
	auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, SpeedPWM_DRV::LOW);
	for (int p : {5000, 1, -1, -16384, -32767}) {
		pwm->turn(static_cast<int16_t>(p));
		INFO("power " << p << " ccr " << ccr(1));
		// HD: p = 5000: 37767 * 56000 = 2.115e9 < 2^31 (no overflow); p = -32767: pval 0 -> 0
		CHECK(ccrOf(1) == exactCentered(p, 56000));
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.4 turn] CENTERED at speed LOW gives 32767 at power 5580, the largest power without overflow") {
	auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, SpeedPWM_DRV::LOW);
	// HD: pval = 32767 + 5580 = 38347, 38347 * 56000 = 2147432000 <= INT32_MAX 2147483647; / 65535 = 32767.7 -> 32767
	pwm->turn(5580);
	CHECK(ccr(1) == 32767);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.4 turn] CENTERED at speed LOW gives 32768 at power 5581" *
								  KNOWN_ISSUE_UB("B-PWM-4: pval 38348 * 56000 = 2147488000 > INT32_MAX: signed overflow from power 5581")) {
	auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, SpeedPWM_DRV::LOW);
	// intended HD: 38348 * 56000 / 65535 = 2147488000 / 65535 = 32768.1 -> 32768
	pwm->turn(5581);
	CHECK(ccr(1) == 32768);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.4 turn] CENTERED at speed LOW gives (32767 + p) * 56000 / 65535 for powers 10000, 20000 and 32767" *
								  KNOWN_ISSUE_UB("B-PWM-4: (pval * period) multiplies two int32_t, 65534 * 56000 = 3.67e9 > INT32_MAX once pval > 38347 (power above 5580): signed overflow")) {
	auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, SpeedPWM_DRV::LOW);
	for (int p : {10000, 20000, 32767}) {
		pwm->turn(static_cast<int16_t>(p));
		// intended HD: floor((32767 + p) * 56000 / 65535): 10000 -> 36544, 20000 -> 45089, 32767 -> 55999
		INFO("power " << p << " ccr " << ccr(1) << " intended " << exactCentered(p, 56000));
		CHECK(ccrOf(1) == exactCentered(p, 56000));
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.4 turn] CENTERED inverted at speed LOW gives the value of the opposite power for -10000 and -32767" *
								  KNOWN_ISSUE_UB("B-PWM-4: with inversion on, input -10000 / -32767 becomes +10000 / +32767 and the same int32_t overflow occurs")) {
	auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, SpeedPWM_DRV::LOW);
	setInvert(*pwm, true);
	for (int p : {-10000, -32767}) {
		pwm->turn(static_cast<int16_t>(p));
		// intended HD: the value of -p: 10000 -> 36544, 32767 -> 55999
		INFO("input " << p << " ccr " << ccr(1) << " intended " << exactCentered(-p, 56000));
		CHECK(ccrOf(1) == exactCentered(-p, 56000));
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.4 turn] CENTERED power -32768 gives duty 0") {
	auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, SpeedPWM_DRV::LOW);
	// HD: 0x7fff - 32768 = -1, -1 * 56000 / 65535 = 0 (integer division truncates toward zero)
	pwm->turn(-32768);
	CHECK(ccr(1) == 0);
	CHECK(duty(1) == 0.0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.4 turn] CENTERED duty is monotonic in the power") {
	auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, kSafeSpeed);
	uint32_t last = 0;
	bool first = true;
	for (int p = -32768; p <= 32767; p += 1024) {
		pwm->turn(static_cast<int16_t>(p));
		INFO("power " << p << " ccr " << ccr(1) << " previous " << last);
		if (!first) {
			CHECK(ccr(1) >= last);
		}
		last = ccr(1);
		first = false;
	}
	pwm->turn(32767);
	CHECK(ccr(1) >= last);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.5 turn] PWM_DIR power >= 0 sets dir always high and dir_n low") {
	for (int p : {0, 1, 16384, 32767}) {
		auto pwm = build(*this, ModePWM_DRV::PWM_DIR, SpeedPWM_DRV::LOW);
		pwm->turn(static_cast<int16_t>(p));
		INFO("power " << p << " " << snapStr(snap()));
		// ch3 = dir, ch4 = dir_n; CCR 0xffff is above ARR 56000: PWM mode 1 output stays high for the whole period
		CHECK(duty(3) == 1.0);
		CHECK(ccr(4) == 0);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.5 turn] PWM_DIR power < 0 sets dir low and dir_n always high") {
	for (int p : {-1, -16384, -32767, -32768}) {
		auto pwm = build(*this, ModePWM_DRV::PWM_DIR, SpeedPWM_DRV::LOW);
		pwm->turn(static_cast<int16_t>(p));
		INFO("power " << p << " " << snapStr(snap()));
		CHECK(ccr(3) == 0);
		CHECK(duty(4) == 1.0);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.5 turn] PWM_DIR duty is |p| * ARR / 32767 for every speed") {
	for (const SpeedRow &s : kSpeeds) {
		auto pwm = build(*this, ModePWM_DRV::PWM_DIR, s.speed);
		for (int p : {32767, 16384, 1, -1, -16384, -32767, -32768}) {
				INFO("speed " << std::string(s.name) << " power " << p << " ccr " << ccr(1));
				pwm->turn(static_cast<int16_t>(p));
				// HD LOW: p = 16384: 16384 * 56000 / 32767 = 28000.85 -> 28000; p = 32767: 56000 = ARR;
				// p = -32768: 32768 * 56000 / 32767 = 56001 (VERYHIGH 7000.2 -> 7000)
				CHECK(ccrOf(1) == exactMagnitude(p, s.arr));
			}
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.5 turn] PWM_DIR power 0 gives duty 0") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DIR, SpeedPWM_DRV::LOW);
	pwm->turn(20000);
	pwm->turn(0);
	CHECK(ccr(1) == 0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.5 turn] PWM_DIR power -32768 gives a duty of at least 100 percent without wrapping") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DIR, SpeedPWM_DRV::LOW);
	// HD: abs(-32768) = 32768, 32768 * 56000 / 32767 = 56001.7 -> 56001 > ARR 56000: always high
	pwm->turn(-32768);
	CHECK(ccr(1) == 56001);
	CHECK(ccr(1) > arr());
	CHECK(duty(1) == 1.0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.5 turn] PWM_DIR at LOW gives CCR 1 / 28000 / 56000 / 56001 for p = 1 / 16384 / 32767 / -32768") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DIR, SpeedPWM_DRV::LOW);
	// HD: |p| * 56000 / 32767: 1 -> 1.7 -> 1; 16384 -> 28000.85 -> 28000; 32767 -> 56000; 32768 -> 56001.7 -> 56001
	pwm->turn(1);
	CHECK(ccr(1) == 1);
	pwm->turn(16384);
	CHECK(ccr(1) == 28000);
	pwm->turn(32767);
	CHECK(ccr(1) == 56000);
	pwm->turn(-32768);
	CHECK(ccr(1) == 56001);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.6 turn] DUAL at LOW gives CCR 1 / 28000 / 56000 / 56001 for p = 1 / 16384 / 32767 / -32768") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DUAL, SpeedPWM_DRV::LOW);
	// HD as for PWM_DIR (same formula); positive power on channel 1, negative on channel 2
	pwm->turn(1);
	CHECK(ccr(1) == 1);
	pwm->turn(16384);
	CHECK(ccr(1) == 28000);
	pwm->turn(32767);
	CHECK(ccr(1) == 56000);
	pwm->turn(-32768);
	CHECK(ccr(2) == 56001);
	CHECK(ccr(1) == 0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.6 turn] DUAL power > 0 drives channel 1 with |p| * ARR / 32767 and channel 2 stays 0") {
	for (const SpeedRow &s : kSpeeds) {
		auto pwm = build(*this, ModePWM_DRV::PWM_DUAL, s.speed);
		for (int p : {1, 16384, 32767}) {
			INFO("speed " << std::string(s.name) << " power " << p << " " << snapStr(snap()));
			pwm->turn(static_cast<int16_t>(p));
			CHECK(ccrOf(1) == exactMagnitude(p, s.arr));
				CHECK(ccr(2) == 0);
		}
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.6 turn] DUAL power < 0 drives channel 2 with |p| * ARR / 32767 and channel 1 stays 0") {
	for (const SpeedRow &s : kSpeeds) {
		auto pwm = build(*this, ModePWM_DRV::PWM_DUAL, s.speed);
		for (int p : {-1, -16384, -32767, -32768}) {
			INFO("speed " << std::string(s.name) << " power " << p << " " << snapStr(snap()));
			pwm->turn(static_cast<int16_t>(p));
			// HD: p = -32768: abs = 32768, 32768 * 56000 / 32767 = 56001.7 -> 56001 (LOW)
				CHECK(ccrOf(2) == exactMagnitude(p, s.arr));
			CHECK(ccr(1) == 0);
		}
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.6 turn] DUAL power 0 sets both channels to 0") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DUAL, SpeedPWM_DRV::LOW);
	pwm->turn(20000);
	pwm->turn(0);
	CHECK(ccr(1) == 0);
	CHECK(ccr(2) == 0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.6 turn] DUAL turn(-20000) then turn(20000) moves the value from channel 2 to channel 1") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DUAL, SpeedPWM_DRV::MID);
	// HD: 20000 * 18666 / 32767 = 373320000 / 32767 = 11393.2 -> 11393
	pwm->turn(-20000);
	REQUIRE(ccr(2) == 11393);
	pwm->turn(20000);
	CHECK(ccr(2) == 0);
	CHECK(ccr(1) == 11393);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.6 turn] DUAL turn(20000) then turn(-20000) moves the value from channel 1 to channel 2") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DUAL, SpeedPWM_DRV::MID);
	pwm->turn(20000);
	REQUIRE(ccr(1) == 11393);
	pwm->turn(-20000);
	CHECK(ccr(1) == 0);
	CHECK(ccr(2) == 11393);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.6 turn] DUAL turn(-20000) then turn(0) clears both channels") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DUAL, SpeedPWM_DRV::MID);
	pwm->turn(-20000);
	REQUIRE(ccr(2) == 11393);
	pwm->turn(0);
	CHECK(ccr(1) == 0);
	CHECK(ccr(2) == 0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.5 turn] PWM_DIR turn(-20000) then turn(20000) swaps the direction pair and keeps the duty") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DIR, SpeedPWM_DRV::MID);
	pwm->turn(-20000);
	REQUIRE(ccr(3) == 0);
	REQUIRE(duty(4) == 1.0);
	pwm->turn(20000);
	CHECK(duty(3) == 1.0);
	CHECK(ccr(4) == 0);
	CHECK(ccr(1) == 11393); // HD: 20000 * 18666 / 32767
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.5 turn] PWM_DIR turn(20000) then turn(-20000) swaps the direction pair and keeps the duty") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DIR, SpeedPWM_DRV::MID);
	pwm->turn(20000);
	REQUIRE(duty(3) == 1.0);
	REQUIRE(ccr(4) == 0);
	pwm->turn(-20000);
	CHECK(ccr(3) == 0);
	CHECK(duty(4) == 1.0);
	CHECK(ccr(1) == 11393);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.7 turn] RC inverted gives 2000 us for +32767 and 1000 us for -32767") {
	auto pwm = build(*this, ModePWM_DRV::RC_PWM, SpeedPWM_DRV::LOW);
	setInvert(*pwm, true);
	pwm->turn(32767); // non-inverted +32767 gives 1000 us (characterization R3.2)
	CHECK(within(pulseUs(1), 2000.0, tickUs()));
	pwm->turn(-32767);
	CHECK(within(pulseUs(1), 1000.0, tickUs()));
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.7 turn] CENTERED inverted at MID gives 18665 for -32767") {
	auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, SpeedPWM_DRV::MID);
	setInvert(*pwm, true);
	pwm->turn(-32767); // becomes +32767: 65534 * 18666 / 65535 = 18665
	CHECK(ccr(1) == 18665);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.7 turn] PWM_DIR inverted gives dir low and dir_n high for +32767") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DIR, SpeedPWM_DRV::LOW);
	setInvert(*pwm, true);
	pwm->turn(32767); // becomes -32767: dir (ch3) 0, dir_n (ch4) always high, duty 56000
	CHECK(ccr(3) == 0);
	CHECK(duty(4) == 1.0);
	CHECK(ccr(1) == 56000);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.7 turn] DUAL inverted gives channel 2 for +20000") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DUAL, SpeedPWM_DRV::MID);
	setInvert(*pwm, true);
	pwm->turn(20000); // becomes -20000: channel 2 carries 11393, channel 1 is 0
	CHECK(ccr(2) == 11393);
	CHECK(ccr(1) == 0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.7 turn] RC inversion gives the output of the opposite power") {
	checkInversionSwapsSign(*this, ModePWM_DRV::RC_PWM);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.7 turn] CENTERED inversion gives the output of the opposite power") {
	checkInversionSwapsSign(*this, ModePWM_DRV::CENTERED_PWM);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.7 turn] PWM_DIR inversion gives the output of the opposite power") {
	checkInversionSwapsSign(*this, ModePWM_DRV::PWM_DIR);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.7 turn] DUAL inversion gives the output of the opposite power") {
	checkInversionSwapsSign(*this, ModePWM_DRV::PWM_DUAL);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.8 turn] characterization: with inversion on, power -32768 gives the output of -32768 again, not of 32767") {
	// NEEDS REVIEW: -power on int16_t is computed as int 32768 and narrowed back to int16_t -32768 (implementation defined,
	// modular on GCC / clang and ARM GCC). An intended inversion would give the output of +32768 (about +32767).
	for (const ModeRow &m : kModes) {
		INFO("mode " << std::string(m.name));
		auto pwm = build(*this, m.mode, kSafeSpeed);
		pwm->turn(-32768);
		const Snap plainMin = snap();
		pwm->turn(32767);
		const Snap plainMax = snap();
		setInvert(*pwm, true);
		pwm->turn(-32768);
		const Snap inverted = snap();
		CHECK_MESSAGE(inverted == plainMin, "inverted(-32768) " << snapStr(inverted) << " plain(-32768) " << snapStr(plainMin));
		CHECK_MESSAGE(inverted != plainMax, "plain(32767) " << snapStr(plainMax));
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.9 turn] channels a mode does not use stay at CCR 0 for every power after a switch from every mode") {
	const int powers[] = {-32768, -32767, -1, 0, 1, 32767};
	for (const ModeRow &from : kModes) {
		for (const ModeRow &to : kModes) {
			auto pwm = build(*this, from.mode, kSafeSpeed);
			pwm->turn(20000); // leave non-zero values on the channels of the old mode
			pwm->setMode(to.mode);
			for (int ch : unusedChannels(to.mode)) {
				INFO("switch " << std::string(from.name) << " -> " << std::string(to.name) << " after the switch " << snapStr(snap()));
				CHECK(ccr(ch) == 0);
			}
			for (int p : powers) {
				pwm->turn(static_cast<int16_t>(p));
				for (int ch : unusedChannels(to.mode)) {
					INFO("switch " << std::string(from.name) << " -> " << std::string(to.name) << " power " << p << " " << snapStr(snap()));
					CHECK(ccr(ch) == 0);
				}
			}
		}
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.10 turn] startMotor outputs the neutral value of the current mode") {
	for (const ModeRow &m : kModes) {
		INFO("mode " << std::string(m.name));
		auto pwm = build(*this, m.mode, SpeedPWM_DRV::LOW, false);
		// driver inactive: CCR 0 on all channels (R2.5), neutral is only written by startMotor
		REQUIRE(snap() == (Snap{0, 0, 0, 0}));
		pwm->startMotor();
		checkNeutral(m.mode);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.10 turn] stopMotor outputs the neutral value of the current mode") {
	for (const ModeRow &m : kModes) {
		INFO("mode " << std::string(m.name));
		auto pwm = build(*this, m.mode, kSafeSpeed);
		pwm->turn(20000);
		pwm->stopMotor();
		checkNeutral(m.mode, 18666);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.10 turn] characterization: the signal keeps running after stopMotor, channels, main output and counter stay on") {
	// NEEDS REVIEW (owner question): for an RC ESC a neutral signal is an arm state; whether an actuator driver should
	// switch the outputs off instead is undecided
	for (const ModeRow &m : kModes) {
		INFO("mode " << std::string(m.name));
		auto pwm = build(*this, m.mode, SpeedPWM_DRV::LOW);
		pwm->stopMotor();
		for (int ch = 1; ch <= 4; ++ch) {
			CHECK(channelOn(ch));
		}
		CHECK(mainOutput());
		CHECK(counterOn());
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.10 turn] startMotor after stopMotor makes turn effective again") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DUAL, SpeedPWM_DRV::LOW);
	pwm->stopMotor();
	pwm->turn(20000);
	REQUIRE(ccr(1) == 0); // ignored while stopped
	pwm->startMotor();
	pwm->turn(20000);
	CHECK(ccrOf(1) == exactMagnitude(20000, 56000)); // HD: 20000 * 56000 / 32767 = 34180.8 -> 34180
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.11 turn] emergencyStop(false) behaves as stopMotor: neutral output, then turn is ignored") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DIR, SpeedPWM_DRV::LOW);
	pwm->turn(-20000);
	REQUIRE(ccr(1) != 0);
	pwm->emergencyStop(false);
	checkNeutral(ModePWM_DRV::PWM_DIR);
	const Snap stopped = snap();
	pwm->turn(20000);
	CHECK(snap() == stopped);
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.11 turn] emergencyStop(true) behaves as startMotor: neutral output and turn is effective") {
	auto pwm = build(*this, ModePWM_DRV::PWM_DUAL, SpeedPWM_DRV::LOW, false);
	pwm->turn(20000);
	REQUIRE(ccr(1) == 0); // not started
	pwm->emergencyStop(true);
	checkNeutral(ModePWM_DRV::PWM_DUAL);
	pwm->turn(20000);
	CHECK(ccrOf(1) == exactMagnitude(20000, 56000));
}

TEST_CASE_FIXTURE(PwmFixture, "[R3.11 turn] motorReady is true before start, while running and after stop") {
	auto pwm = build(*this, ModePWM_DRV::CENTERED_PWM, SpeedPWM_DRV::LOW, false);
	CHECK(pwm->motorReady());
	pwm->startMotor();
	CHECK(pwm->motorReady());
	pwm->stopMotor();
	CHECK(pwm->motorReady());
}

}
