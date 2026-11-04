/*
 * host_driver_timers.cpp
 *
 * Fake TIM6 of the F407 target for ffb_driver_tests only (driver track, Tests/DRIVER_TEST_PLAN.md). The TMC4671
 * driver (TIM_TMC = htim6, platform/target_f407_drivers/target_constants.h) uses it as its external encoder timer.
 * The other executables have TIM_TMC defined too but never reference htim6.
 *
 * Models:  the timer handle and register block as left by MX_TIM6_Init (Targets/F407VG/Core/Src/main.c:1054-1058):
 *          Prescaler 167, up-counting, Period 65535, auto-reload preload disabled; HAL_TIM_Base_Init effects as in
 *          initTimer of host_target.cpp (ARR = Period, PSC = Prescaler, State READY, CNT 0, timer stopped).
 *          The start / stop / counter-overflow behaviour is the generic TIM base fake of host_target.cpp
 *          (HAL_TIM_Base_Start / _IT, HostTarget::timerUpdate), which works on any handle.
 *          Source: ST HAL stm32f4xx_hal_tim.c (HAL_TIM_Base_Init, TIM_Base_SetConfig), RM0090 chapter 'Basic timers
 *          (TIM6 and TIM7)'.
 * Does not model: the master mode / trigger output (MX_TIM6_Init sMasterConfig), the DAC trigger, the clock
 *          (the counter only advances when a test drives it), the MSP init (clock enable, NVIC).
 * Confidence: High for the initial state (follows MX_TIM6_Init line by line), Medium for the rest (generic fake).
 * Reset:   HostTarget::reset() calls resetDriverTimers() through HostTarget::addResetHook.
 */
#include "HostTarget.h"
#include "main.h"

namespace {
// Values of the ST HAL (stm32f4xx_hal_tim.h:518, :549); the host main.h does not define these constants
constexpr uint32_t HOSTTEST_TIM_COUNTERMODE_UP = 0x00000000U;
constexpr uint32_t HOSTTEST_TIM_AUTORELOAD_PRELOAD_DISABLE = 0x00000000U;

TIM_TypeDef tim6Regs = {};

void resetDriverTimers() {
	tim6Regs = TIM_TypeDef();
	htim6 = TIM_HandleTypeDef();
	htim6.Instance = &tim6Regs;
	htim6.Init.Prescaler = 167;
	htim6.Init.CounterMode = HOSTTEST_TIM_COUNTERMODE_UP;
	htim6.Init.Period = 65535;
	htim6.Init.AutoReloadPreload = HOSTTEST_TIM_AUTORELOAD_PRELOAD_DISABLE;
	tim6Regs.PSC = htim6.Init.Prescaler;
	tim6Regs.ARR = htim6.Init.Period;
	htim6.State = HAL_TIM_STATE_READY;
}
} // namespace

TIM_HandleTypeDef htim6 = {};

namespace {
/** Registers the reset and applies it once, so htim6 is initialised before the first fixture reset (like main()) */
struct DriverTimersRegistration {
	DriverTimersRegistration() {
		resetDriverTimers();
		HostTarget::addResetHook(resetDriverTimers);
	}
} driverTimersRegistration;
} // namespace
