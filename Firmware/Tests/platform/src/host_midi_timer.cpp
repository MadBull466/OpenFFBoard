/*
 * host_midi_timer.cpp
 *
 * Fake TIM9 of the F407 target for ffb_usb_tests only (P17, MIDI; Tests/claude/charters/driver/MIDI_REQUIREMENTS.md).
 * MidiMain uses it as the user timer (TIM_USER = htim9, platform/usb_midi/target_constants.h): it writes ARR, PSC and CR1
 * and starts the update interrupt with HAL_TIM_Base_Start_IT. The other executables have TIM_USER defined too but never
 * reference htim9.
 *
 * Models:  the timer handle and register block as left by MX_TIM9_Init (Targets/F407VG/Core/Src/main.c:1154-1170):
 *          Prescaler 0, up-counting, Period 65535, clock division 1, auto-reload preload disabled; HAL_TIM_Base_Init
 *          effects as initTimer of host_target.cpp (ARR = Period, PSC = Prescaler, State READY, CNT 0, timer stopped).
 *          The start / stop behaviour is the generic TIM base fake of host_target.cpp (HAL_TIM_Base_Start_IT: HAL_ERROR
 *          unless READY, State BUSY, UIE and CEN set; HostTarget::timerUpdate(&htim9) raises the update interrupt, which
 *          runs HAL_TIM_PeriodElapsedCallback and so every TimerHandler::timerElapsed).
 *          Source: ST HAL stm32f4xx_hal_tim.c (HAL_TIM_Base_Init, HAL_TIM_Base_Start_IT), RM0090 chapter 'General-purpose
 *          timers (TIM9 to TIM14)'.
 * Does not model: the counter (it only advances when a test drives it: one timerUpdate() call is one update event, the
 *          100 microsecond period MidiMain programs is not simulated), the clock configuration
 *          (MX_TIM9_Init's HAL_TIM_ConfigClockSource), the MSP init (clock enable, NVIC), the prescaler shadow register.
 * Confidence: High for the initial state (follows MX_TIM9_Init line by line), Medium for the rest (generic fake).
 * Reset:   HostTarget::reset() calls resetMidiTimer() through HostTarget::addResetHook.
 */
#include "HostTarget.h"
#include "main.h"

#include "target_constants.h"

TIM_HandleTypeDef htim9 = {};

namespace {
constexpr uint32_t HOSTTEST_TIM_COUNTERMODE_UP = 0x00000000U;
constexpr uint32_t HOSTTEST_TIM_AUTORELOAD_PRELOAD_DISABLE = 0x00000000U;

TIM_TypeDef tim9Regs = {};

void resetMidiTimer() {
	tim9Regs = TIM_TypeDef();
	htim9 = TIM_HandleTypeDef();
	htim9.Instance = &tim9Regs;
	htim9.Init.Prescaler = 0;
	htim9.Init.CounterMode = HOSTTEST_TIM_COUNTERMODE_UP;
	htim9.Init.Period = 65535;
	htim9.Init.AutoReloadPreload = HOSTTEST_TIM_AUTORELOAD_PRELOAD_DISABLE;
	tim9Regs.PSC = htim9.Init.Prescaler;
	tim9Regs.ARR = htim9.Init.Period;
	htim9.State = HAL_TIM_STATE_READY;
}

/** Registers the reset and applies it once, so htim9 is initialised before the first fixture reset (like main()) */
struct MidiTimerRegistration {
	MidiTimerRegistration() {
		resetMidiTimer();
		HostTarget::addResetHook(resetMidiTimer);
	}
} midiTimerRegistration;
} // namespace
