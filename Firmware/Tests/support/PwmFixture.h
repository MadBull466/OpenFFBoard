/*
 * PwmFixture.h
 *
 * Fixture of the PWM driver tests (ffb_driver_tests only, phase P14,
 * Tests/claude/charters/driver/PWM_REQUIREMENTS.md "Fixture and test layout").
 *
 * Models: TargetFixture (target state reset before and after the test, unmodelled HAL calls and strict-fake rejections
 * of the TIM1 fake fail the test) plus
 *  - the fake flash cleared (HostPlatform::flash()), so MotorPWM::restoreFlash finds no stored word,
 *  - the singleton flag MotorPWM::pwmDriverInUse reset before and after every test (it is a static the fixture must own),
 *  - a view of the fake TIM1 (platform/src/host_driver_timers.cpp): ARR, PSC, CCR1-4, enabled channels, main output,
 *    counter enable, and the derived period / pulse / duty with the 168 MHz timer clock of MotorPWM::timerConfig.
 * Source: MotorPWM.cpp / MotorPWM.h, RM0090 TIM1 chapter (period = (PSC + 1)(ARR + 1) / f_timer; PWM mode 1: output high
 * while CNT < CCR). Does not model: the counter and real pin levels (the derived values are computed from the registers),
 * shadow registers (see the fake header). Confidence: High for the register view, the derived values are plain arithmetic.
 *
 * Helpers (public interface of the driver and the register view only):
 *   makePwm()          constructs the MotorPWM (empty flash unless the test stored a word first)
 *   ccr(ch)            CCRch value, ch 1..4
 *   arr() / psc()      timer registers
 *   channelOn(ch)      CCxE of the channel
 *   mainOutput()       BDTR.MOE; counterOn() CR1.CEN
 *   periodUs()         (PSC + 1)(ARR + 1) / 168 MHz in microseconds
 *   pulseUs(ch)        CCRch * (PSC + 1) / 168 MHz in microseconds (RC pulse width)
 *   duty(ch)           min(CCRch, ARR + 1) / (ARR + 1): PWM mode 1 duty, CCR above ARR gives 1.0 (always high)
 *   outputFrequencyHz()  168 MHz / ((PSC + 1)(ARR + 1))
 *   log()              the PWM HAL calls in order ("Config:2", "Start:2:OK", "Start:2:ERR", "Stop:2"); clearLog()
 */
#ifndef HOSTTEST_PWMFIXTURE_H_
#define HOSTTEST_PWMFIXTURE_H_

#include "AxisFixture.h"
#include "TestHelpers.h"

#include "MotorPWM.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

extern uint32_t hosttestTim1MspPostInitCalls; // platform/src/host_driver_timers.cpp
extern std::vector<std::string> hosttestTim1Log; // platform/src/host_driver_timers.cpp: PWM HAL calls in order (R2.6)

struct PwmFixture : TargetFixture {
	static constexpr double kTimerHz = 168000000.0;

	PwmFixture() {
		HostPlatform::flash().clear();
		MotorPWM::pwmDriverInUse = false;
	}
	~PwmFixture() override { MotorPWM::pwmDriverInUse = false; }

	std::unique_ptr<MotorPWM> makePwm() { return std::make_unique<MotorPWM>(); }

	static const std::vector<std::string> &log() { return hosttestTim1Log; }
	static void clearLog() { hosttestTim1Log.clear(); }
	static TIM_TypeDef &tim() { return *htim1.Instance; }
	static uint32_t arr() { return tim().ARR; }
	static uint32_t psc() { return tim().PSC; }
	static uint32_t ccr(int ch) {
		switch (ch) {
		case 1: return tim().CCR1;
		case 2: return tim().CCR2;
		case 3: return tim().CCR3;
		default: return tim().CCR4;
		}
	}
	static bool channelOn(int ch) { return (tim().CCER & (1U << (4U * static_cast<unsigned>(ch - 1)))) != 0; }
	static bool mainOutput() { return (tim().BDTR & TIM_BDTR_MOE) != 0; }
	static bool counterOn() { return (tim().CR1 & TIM_CR1_CEN) != 0; }
	static double periodUs() { return (psc() + 1.0) * (arr() + 1.0) / kTimerHz * 1e6; }
	static double pulseUs(int ch) { return ccr(ch) * (psc() + 1.0) / kTimerHz * 1e6; }
	static double duty(int ch) { return std::min<double>(ccr(ch), arr() + 1.0) / (arr() + 1.0); }
	static double outputFrequencyHz() { return kTimerHz / ((psc() + 1.0) * (arr() + 1.0)); }
};

#endif
