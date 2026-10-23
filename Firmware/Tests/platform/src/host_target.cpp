/*
 * host_target.cpp
 *
 * Fake HAL of the F407 target for ffb_target_tests (declarations in platform/target_f407/main.h):
 * GPIO ports, TIM base (F7), EXTI (F7), chip ids, reset requests and clocks (F10), and the TinyUSB device
 * functions used only by the F407 sources. Test control: support/HostTarget.h.
 *
 * Every fake models the documented behaviour of the real function. The reference is the ST HAL source in
 * Targets/F407VG/Drivers/STM32F4xx_HAL_Driver (read only), cited per function.
 */
#include "HostTarget.h"
#include "HostPlatform.h"
#include "MockCANPort.h"

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"

GPIO_TypeDef hosttest_gpioa = {}, hosttest_gpiob = {}, hosttest_gpioc = {}, hosttest_gpiod = {}, hosttest_gpioe = {};

// system_stm32f4xx.c: updated by SystemClock_Config. The F407 runs at 168 MHz
uint32_t SystemCoreClock = 168000000;

namespace {
HostTarget::State targetState;
TIM_TypeDef tim3Regs = {};
TIM_TypeDef tim13Regs = {};
} // namespace

// Timer handles as initialised by MX_TIM3_Init / MX_TIM13_Init (Targets/F407VG/Core/Src/main.c)
TIM_HandleTypeDef htim3 = {};
TIM_HandleTypeDef htim13 = {};

namespace {
/** Models HAL_TIM_Base_Init (stm32f4xx_hal_tim.c) via TIM_Base_SetConfig: ARR = Period, PSC = Prescaler, State READY */
void initTimer(TIM_HandleTypeDef &htim, TIM_TypeDef &regs, uint32_t prescaler, uint32_t period) {
	regs = TIM_TypeDef();
	htim = TIM_HandleTypeDef();
	htim.Instance = &regs;
	htim.Init.Prescaler = prescaler;
	htim.Init.Period = period;
	regs.PSC = prescaler;
	regs.ARR = period;
	htim.State = HAL_TIM_STATE_READY;
}
} // namespace

namespace HostTarget {

State &state() { return targetState; }

void reset() {
	targetState = State();
	hosttest_gpioa = {};
	hosttest_gpiob = {};
	hosttest_gpioc = {};
	hosttest_gpiod = {};
	hosttest_gpioe = {};
	// MX_GPIO_Init (Targets/F407VG/Core/Src/main.c): inputs with pull-up read high while nothing is connected
	// (DIN0-7 buttons, E_STOP released). Initial output levels: SPI1_SS1 set, all others reset.
	setPin(DIN0_GPIO_Port, DIN0_Pin, true);
	setPin(DIN1_GPIO_Port, DIN1_Pin, true);
	setPin(DIN2_GPIO_Port, DIN2_Pin, true);
	setPin(DIN3_GPIO_Port, DIN3_Pin, true);
	setPin(DIN4_GPIO_Port, DIN4_Pin, true);
	setPin(DIN5_GPIO_Port, DIN5_Pin, true);
	setPin(DIN6_GPIO_Port, DIN6_Pin, true);
	setPin(DIN7_GPIO_Port, DIN7_Pin, true);
	setPin(E_STOP_GPIO_Port, E_STOP_Pin, true);
	HAL_GPIO_WritePin(SPI1_SS1_GPIO_Port, SPI1_SS1_Pin, GPIO_PIN_SET);
	initTimer(htim3, tim3Regs, 0, 0xffff); // MX_TIM3_Init: encoder mode, Period 0xffff
	initTimer(htim13, tim13Regs, 84 - 1, 100); // MX_TIM13_Init: 1 MHz, Period 100
	MockCANPort &can = canPort();
	can.sent.clear();
	can.filters.clear();
	can.sendResult = true;
}

bool timerRunning(const TIM_HandleTypeDef *htim) { return (htim->Instance->CR1 & TIM_CR1_CEN) != 0; }
bool timerUpdateIrqEnabled(const TIM_HandleTypeDef *htim) { return (htim->Instance->DIER & TIM_DIER_UIE) != 0; }

bool timerUpdate(TIM_HandleTypeDef *htim) {
	// HAL_TIM_IRQHandler: the update flag is set by the counter overflow, the callback only runs if the
	// update interrupt is enabled. A stopped counter does not overflow.
	if (!timerRunning(htim) || !timerUpdateIrqEnabled(htim)) {
		return false;
	}
	HostPlatform::IsrContext isr;
	HAL_TIM_PeriodElapsedCallback(htim);
	return true;
}

void setPin(GPIO_TypeDef *port, uint16_t pin, bool high) { port->IDR = high ? (port->IDR | pin) : (port->IDR & ~(uint32_t)pin); }
bool outputPin(const GPIO_TypeDef *port, uint16_t pin) { return (port->ODR & pin) != 0; }

void triggerExti(uint16_t pin) {
	// HAL_GPIO_EXTI_IRQHandler (stm32f4xx_hal_gpio.c): clears the pending bit and calls HAL_GPIO_EXTI_Callback
	HostPlatform::IsrContext isr;
	HAL_GPIO_EXTI_Callback(pin);
}

} // namespace HostTarget

extern "C" {

// ---------------- TIM (stm32f4xx_hal_tim.c) ----------------

/** Models HAL_TIM_Base_Start: HAL_ERROR unless State is READY; State BUSY; counter enabled (CEN) */
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim) {
	if (htim->State != HAL_TIM_STATE_READY) {
		return HAL_ERROR;
	}
	htim->State = HAL_TIM_STATE_BUSY;
	htim->Instance->CR1 |= TIM_CR1_CEN;
	return HAL_OK;
}

/** Models HAL_TIM_Base_Stop: counter disabled (__HAL_TIM_DISABLE: only if no CCx output is enabled); State READY */
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim) {
	if (htim->Instance->CCER == 0) {
		htim->Instance->CR1 &= ~TIM_CR1_CEN;
	}
	htim->State = HAL_TIM_STATE_READY;
	return HAL_OK;
}

/** Models HAL_TIM_Base_Start_IT: HAL_ERROR unless State is READY; State BUSY; update interrupt (UIE) and counter enabled */
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim) {
	if (htim->State != HAL_TIM_STATE_READY) {
		return HAL_ERROR;
	}
	htim->State = HAL_TIM_STATE_BUSY;
	htim->Instance->DIER |= TIM_DIER_UIE;
	htim->Instance->CR1 |= TIM_CR1_CEN;
	return HAL_OK;
}

/** Models HAL_TIM_Base_Stop_IT: update interrupt disabled, counter disabled (if no CCx output enabled); State READY */
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim) {
	htim->Instance->DIER &= ~TIM_DIER_UIE;
	if (htim->Instance->CCER == 0) {
		htim->Instance->CR1 &= ~TIM_CR1_CEN;
	}
	htim->State = HAL_TIM_STATE_READY;
	return HAL_OK;
}

// ---------------- System (stm32f4xx_hal.c, stm32f4xx_hal_rcc.c, core_cm4.h) ----------------

/** Models HAL_GetUIDw0/1/2: words 0..2 of the 96 bit unique device id at UID_BASE */
uint32_t HAL_GetUIDw0(void) { return targetState.uid[0]; }
uint32_t HAL_GetUIDw1(void) { return targetState.uid[1]; }
uint32_t HAL_GetUIDw2(void) { return targetState.uid[2]; }
/** Models HAL_GetDEVID: DBGMCU->IDCODE & 0xFFF */
uint32_t HAL_GetDEVID(void) { return targetState.devId & 0xFFFU; }
/** Models HAL_GetREVID: DBGMCU->IDCODE >> 16 */
uint32_t HAL_GetREVID(void) { return targetState.revId & 0xFFFFU; }
/** Models HAL_RCC_GetHCLKFreq: returns SystemCoreClock */
uint32_t HAL_RCC_GetHCLKFreq(void) { return SystemCoreClock; }

/** Models NVIC_SystemReset (core_cm4.h). Recorded. Returns, unlike on the target */
void NVIC_SystemReset(void) { targetState.systemResets++; }
/** RebootDFU / ExitDFU of Targets/F407VG/Core/Src/main.c (set a magic value in backup RAM and reset). Recorded */
void RebootDFU(void) { targetState.dfuReboots++; }
void ExitDFU(void) { targetState.dfuExits++; }

// ---------------- TinyUSB device (usbd.c, tusb.c) used by the F407 sources ----------------

/** Models tusb_rhport_init (tusb_init): initialises the device stack. Recorded, always succeeds */
bool tusb_rhport_init(uint8_t, const tusb_rhport_init_t *) {
	targetState.usbInits++;
	return true;
}

/**
 * Models tud_task_ext with an RTOS (usbd.c): waits on the USB event queue (osal_queue_receive, here forever)
 * and processes the events. The fake has no USB events, so the calling thread blocks.
 */
void tud_task_ext(uint32_t, bool) { ulTaskNotifyTake(pdTRUE, portMAX_DELAY); }

/** Models tud_speed_get: speed of the current connection (set by the test, default full speed) */
tusb_speed_t tud_speed_get(void) { return (tusb_speed_t)targetState.usbSpeed; }

} // extern "C"
