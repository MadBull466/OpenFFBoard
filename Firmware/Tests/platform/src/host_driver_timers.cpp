/*
 * host_driver_timers.cpp
 *
 * Fake TIM6 of the F407 target for ffb_driver_tests only (driver track, Tests/claude/plans/DRIVER_TEST_PLAN.md). The TMC4671
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
 *
 * P14 (PWM driver, PWMDRIVER on in ffb_driver_tests only): fake TIM1 with PWM output compare, htim1 (TIM_PWM).
 * Models:  the handle as left by MX_TIM1_Init (Targets/F407VG/Core/Src/main.c:716-721): Prescaler 0, up-counting,
 *          Period 0xffff, auto-reload preload ENABLED, State READY, all four channel states READY (HAL_TIM_PWM_Init);
 *          (including the four HAL_TIM_PWM_ConfigChannel calls of MX_TIM1_Init: CCMR1 = CCMR2 = 0x6868);
 *          the register writes of the PWM HAL calls with the contracts of stm32f4xx_hal_tim.c (the copy in
 *          Targets/F411RE/Drivers):
 *            HAL_TIM_PWM_ConfigChannel: clears CCxE first (TIM_OCx_SetConfig: a running channel is switched off, so a
 *              following Start of a BUSY channel is rejected); channel 1-4 only and OCMode PWM1 / PWM2 only (else HAL_ERROR and a record in
 *              HostTarget::state().unmodelledHalCalls, which the TargetFixture check turns into a test failure);
 *              CCRx = Pulse, OCxM = OCMode, CCxP = OCPolarity, OCxPE set (preload), OCxFE = OCFastMode.
 *            HAL_TIM_PWM_Start: HAL_ERROR unless the channel state is READY (a second Start without Stop is rejected
 *              like the real HAL); state BUSY; CCxE set; MOE set (TIM1 is a break instance); counter enabled (CEN).
 *            HAL_TIM_PWM_Stop: CCxE cleared; MOE cleared and CEN cleared only if no CCxE / CCxNE bit remains
 *              (__HAL_TIM_MOE_DISABLE / __HAL_TIM_DISABLE); channel state READY.
 *            HAL_TIM_MspPostInit: only counted (hosttestTim1MspPostInitCalls).
 *          Every PWM call is also appended to hosttestTim1Log (order of Config / Start / Stop, rejected Starts included).
 *          The definition of MotorPWM::timerConfig (the board table of Targets/F407VG/Core/Src/cpp_target_config.cpp:62,
 *          same values).
 * Does not model: the counter (it never advances: no waveform, no real pin levels), the shadow registers (ARR, PSC and
 *          CCRx writes are visible at once instead of at the next update event, although ARPE and OCxPE are set), dead
 *          time, break input, complementary outputs and the idle states, DMA / interrupt variants, the MSP GPIO set-up,
 *          the HAL lock (single threaded fake), the timer clock (168 MHz is a constant of MotorPWM::timerConfig).
 * Source:  RM0090 chapter 'Advanced-control timers (TIM1 and TIM8)' (PWM mode, CCRx / ARR / PSC, BDTR.MOE), ST HAL
 *          stm32f4xx_hal_tim.c (HAL_TIM_PWM_Start, _Stop, _ConfigChannel, TIM_OCx_SetConfig).
 * Differences to the HAL that are NOT modelled and do not matter for MotorPWM: the assert_param checks of polarity, fast
 *          mode and idle states (only channel and OCMode are checked), TIM_OCx_SetConfig clearing CCxNP / CCxNE and writing
 *          CR2 OISx (complementary outputs), OCxCE cleared with the whole CCMR byte, the slave-mode branch of Start (always
 *          sets CEN; MX_TIM1_Init configures master mode only). HAL_TIM_Base_Stop_IT of the destructor is the generic fake
 *          of host_target.cpp (UIE cleared, CEN cleared if no CCxE, State READY).
 * Confidence: High for the HAL call contracts (compared call by call with stm32f4xx_hal_tim.c in review PWR3), Medium-High
 *          for the initial state, Low for timing semantics (not simulated).
 */
#include "HostTarget.h"
#include "main.h"

#include "target_constants.h"

#include <string>
#include <vector>
#ifdef PWMDRIVER
#include "MotorPWM.h"
#endif

extern uint32_t hosttestTim1MspPostInitCalls;
extern TIM_HandleTypeDef htim1;
extern std::vector<std::string> hosttestTim1Log;

namespace {
// Values of the ST HAL (stm32f4xx_hal_tim.h:518, :549); the host main.h does not define these constants
constexpr uint32_t HOSTTEST_TIM_COUNTERMODE_UP = 0x00000000U;
constexpr uint32_t HOSTTEST_TIM_AUTORELOAD_PRELOAD_DISABLE = 0x00000000U;
constexpr uint32_t HOSTTEST_TIM_AUTORELOAD_PRELOAD_ENABLE = 0x00000080U; // TIM_CR1_ARPE

TIM_TypeDef tim6Regs = {};
TIM_TypeDef tim1Regs = {};
constexpr uint8_t kChannelReady = 1U; // HAL_TIM_CHANNEL_STATE_READY
constexpr uint8_t kChannelBusy = 2U;  // HAL_TIM_CHANNEL_STATE_BUSY

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

	// MX_TIM1_Init (main.c:716-721) + HAL_TIM_PWM_Init: Prescaler 0, Period 0xffff, ARPE enabled, channels READY
	tim1Regs = TIM_TypeDef();
	htim1 = TIM_HandleTypeDef();
	htim1.Instance = &tim1Regs;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = HOSTTEST_TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.AutoReloadPreload = HOSTTEST_TIM_AUTORELOAD_PRELOAD_ENABLE;
	tim1Regs.PSC = htim1.Init.Prescaler;
	tim1Regs.ARR = htim1.Init.Period;
	// MX_TIM1_Init (main.c:737-755) also calls HAL_TIM_PWM_ConfigChannel for channels 1-4 (PWM1, Pulse 0, polarity high):
	// OCxM = PWM1 (0x60) and OCxPE (0x08) in both CCMR registers, CCRx 0, outputs not enabled
	tim1Regs.CCMR1 = 0x6868U;
	tim1Regs.CCMR2 = 0x6868U;
	htim1.State = HAL_TIM_STATE_READY;
	for (auto &c : htim1.ChannelState) {
		c = kChannelReady;
	}
	hosttestTim1MspPostInitCalls = 0;
	hosttestTim1Log.clear();
}
} // namespace

TIM_HandleTypeDef htim6 = {};
TIM_HandleTypeDef htim1 = {};
uint32_t hosttestTim1MspPostInitCalls = 0;
/** Every PWM HAL call in order, "Config:<ch>", "Start:<ch>:OK|ERR", "Stop:<ch>" (ch 1..4); cleared by the reset hook (P14, R2.6) */
std::vector<std::string> hosttestTim1Log;

namespace {
/** Registers the reset and applies it once, so htim6 is initialised before the first fixture reset (like main()) */
struct DriverTimersRegistration {
	DriverTimersRegistration() {
		resetDriverTimers();
		HostTarget::addResetHook(resetDriverTimers);
	}
} driverTimersRegistration;
} // namespace

// ---------------- TIM1 PWM output compare and the board table (P14) ----------------
namespace {
/** Index 0..3 of TIM_CHANNEL_1..4, or -1 for any other value (IS_TIM_CCX_INSTANCE / IS_TIM_CHANNELS reject it) */
int channelIndex(uint32_t channel) {
	switch (channel) {
	case TIM_CHANNEL_1: return 0;
	case TIM_CHANNEL_2: return 1;
	case TIM_CHANNEL_3: return 2;
	case TIM_CHANNEL_4: return 3;
	default: return -1;
	}
}
void record(const char *what) { HostTarget::state().unmodelledHalCalls.emplace_back(what); }
} // namespace

extern "C" {

/** Models HAL_TIM_PWM_ConfigChannel + TIM_OCx_SetConfig: see the header comment for the strictness */
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, const TIM_OC_InitTypeDef *sConfig, uint32_t Channel) {
	const int ch = channelIndex(Channel);
	if (ch < 0) {
		record("HAL_TIM_PWM_ConfigChannel: channel is not TIM_CHANNEL_1..4");
		return HAL_ERROR;
	}
	if (sConfig->OCMode != TIM_OCMODE_PWM1 && sConfig->OCMode != TIM_OCMODE_PWM2) {
		record("HAL_TIM_PWM_ConfigChannel: OCMode is not PWM1 / PWM2 (assert_param IS_TIM_PWM_MODE)");
		return HAL_ERROR;
	}
	hosttestTim1Log.push_back("Config:" + std::to_string(ch + 1));
	TIM_TypeDef *t = htim->Instance;
	volatile uint32_t *ccmr = (ch < 2) ? &t->CCMR1 : &t->CCMR2;
	const unsigned shift = (ch % 2) ? 8U : 0U;
	// OCxM (bits 4-6), OCxPE (bit 3), OCxFE (bit 2)
	uint32_t v = *ccmr & ~(0xFFU << shift);
	v |= ((sConfig->OCMode & 0x70U) | TIM_CCMR1_OC1PE | (sConfig->OCFastMode & TIM_OCFAST_ENABLE)) << shift;
	*ccmr = v;
	const unsigned ccerPos = 4U * static_cast<unsigned>(ch);
	t->CCER &= ~(1U << ccerPos); // TIM_OCx_SetConfig: TIMx->CCER &= ~TIM_CCER_CCxE
	t->CCER = (t->CCER & ~(0x2U << ccerPos)) | ((sConfig->OCPolarity & 0x2U) << ccerPos); // CCxP
	switch (ch) {
	case 0: t->CCR1 = sConfig->Pulse; break;
	case 1: t->CCR2 = sConfig->Pulse; break;
	case 2: t->CCR3 = sConfig->Pulse; break;
	default: t->CCR4 = sConfig->Pulse; break;
	}
	return HAL_OK;
}

/** Models HAL_TIM_PWM_Start: HAL_ERROR unless the channel is READY; CCxE, MOE and CEN set */
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel) {
	const int ch = channelIndex(Channel);
	if (ch < 0) {
		record("HAL_TIM_PWM_Start: channel is not TIM_CHANNEL_1..4");
		return HAL_ERROR;
	}
	if (htim->ChannelState[ch] != kChannelReady) {
		hosttestTim1Log.push_back("Start:" + std::to_string(ch + 1) + ":ERR");
		return HAL_ERROR; // like the real HAL: a channel that is not READY (already started) is rejected
	}
	hosttestTim1Log.push_back("Start:" + std::to_string(ch + 1) + ":OK");
	htim->ChannelState[ch] = kChannelBusy;
	htim->Instance->CCER |= 1U << (4U * static_cast<unsigned>(ch)); // CCxE
	htim->Instance->BDTR |= TIM_BDTR_MOE;
	htim->Instance->CR1 |= TIM_CR1_CEN;
	return HAL_OK;
}

/** Models HAL_TIM_PWM_Stop: CCxE cleared; MOE and CEN cleared only if no CCxE / CCxNE bit remains; channel READY */
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel) {
	const int ch = channelIndex(Channel);
	if (ch < 0) {
		record("HAL_TIM_PWM_Stop: channel is not TIM_CHANNEL_1..4");
		return HAL_ERROR;
	}
	hosttestTim1Log.push_back("Stop:" + std::to_string(ch + 1));
	TIM_TypeDef *t = htim->Instance;
	t->CCER &= ~(1U << (4U * static_cast<unsigned>(ch)));
	if ((t->CCER & TIM_CCER_CCxE_MASK) == 0U && (t->CCER & TIM_CCER_CCxNE_MASK) == 0U) {
		t->BDTR &= ~TIM_BDTR_MOE;
		t->CR1 &= ~TIM_CR1_CEN;
	}
	htim->ChannelState[ch] = kChannelReady;
	return HAL_OK;
}

/** HAL_TIM_MspPostInit: the pin set-up is not modelled, only counted */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim) {
	(void)htim;
	++hosttestTim1MspPostInitCalls;
}

} // extern "C"

#ifdef PWMDRIVER
// Board table of Targets/F407VG/Core/Src/cpp_target_config.cpp:62 (same values; timer 168 MHz from the member default)
const PWMConfig MotorPWM::timerConfig = {
	TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4, // channel_1..4
	1, 3, 4, // pwm_chan, dir_chan, dir_chan_n
	1,		 // centerpwm_chan
	1,		 // rcpwm_chan
	1, 2,	 // dualpwm1, dualpwm2
	&TIM_PWM, 168000000U};
#endif

#ifdef SIMPLEMOTION
// RS485 transceiver direction pin of MotorSimplemotion (P15): Targets/F407VG/Core/Src/cpp_target_config.cpp:43, same pin
// (DRV_GP1, PE12). OutputPin writes through the fake HAL_GPIO_WritePin, so the tests read the pin level from GPIOE->ODR.
#include "cpp_target_config.h"
extern const OutputPin gpMotor; // external linkage (a namespace scope const would be internal)
const OutputPin gpMotor{*DRV_GP1_GPIO_Port, DRV_GP1_Pin};
#endif
