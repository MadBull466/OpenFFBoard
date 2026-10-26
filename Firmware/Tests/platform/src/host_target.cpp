/*
 * host_target.cpp
 *
 * Fake HAL of the F407 target for ffb_target_tests (declarations in platform/target_f407/main.h):
 * GPIO ports, TIM base (F7), EXTI (F7), chip ids, reset requests and clocks (F10), and the TinyUSB device
 * functions used only by the F407 sources. ADC, IWDG, CAN receive FIFO, HAL tick and the main.c callbacks are in
 * host_target_system.cpp. Test control: support/HostTarget.h.
 *
 * Every fake models the documented behaviour of the real function. The reference is the ST HAL source in
 * Targets/F407VG/Drivers/STM32F4xx_HAL_Driver (read only), cited per function.
 */
#include "HostTarget.h"
#include "HostPlatform.h"
#include "HostRtos.h"
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

namespace {
/** Kernel calls the fake RTOS can not complete (HostRtos::kernelFaults) count as unmodelled calls in this executable */
struct KernelFaultsAsUnmodelled {
	KernelFaultsAsUnmodelled() {
		HostRtos::setFaultReporter([](const std::string &what) { targetState.unmodelledHalCalls.push_back(what); });
	}
} kernelFaultsAsUnmodelled;
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
	// main() (USER CODE BEGIN 2) switches the three LEDs on before the scheduler starts
	HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
	initTimer(htim3, tim3Regs, 0, 0xffff); // MX_TIM3_Init: encoder mode, Period 0xffff
	initTimer(htim13, tim13Regs, 84 - 1, 100); // MX_TIM13_Init: 1 MHz, Period 100
	resetTargetConfig();
	resetSystem();
}

void queueUsbEvent(TaskHandle_t usbTask) { xTaskNotifyGive(usbTask); }

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

namespace {
#define HOSTTEST_TIM_SR_UIF 0x00000001U // stm32f407xx.h TIM_SR_UIF (= TIM_FLAG_UPDATE)

/** Update event of a counter wrap: UIF set; HAL_TIM_IRQHandler's update branch if UIE is enabled */
uint32_t timerWrapEvent(TIM_HandleTypeDef *htim) {
	htim->Instance->SR |= HOSTTEST_TIM_SR_UIF;
	if (timerUpdateIrqEnabled(htim)) {
		// HAL_TIM_IRQHandler (stm32f4xx_hal_tim.c): UIF and UIE set -> __HAL_TIM_CLEAR_FLAG(UPDATE), PeriodElapsedCallback
		HostPlatform::IsrContext isr;
		htim->Instance->SR &= ~HOSTTEST_TIM_SR_UIF;
		HAL_TIM_PeriodElapsedCallback(htim);
	}
	return 1;
}
} // namespace

uint32_t encoderTimerCount(TIM_HandleTypeDef *htim, int32_t counts) {
	TIM_TypeDef *tim = htim->Instance;
	if (!timerRunning(htim)) {
		return 0; // Counter disabled (CEN = 0): the encoder inputs are not counted
	}
	if (tim->CNT > tim->ARR) {
		targetState.unmodelledHalCalls.emplace_back("encoderTimerCount: counter above ARR");
		return 0;
	}
	uint32_t events = 0;
	int64_t remaining = counts;
	while (remaining > 0) {
		const int64_t toWrap = static_cast<int64_t>(tim->ARR) - tim->CNT; // counts until CNT == ARR
		if (remaining <= toWrap) {
			tim->CNT += static_cast<uint32_t>(remaining);
			break;
		}
		remaining -= toWrap + 1; // ARR -> 0: overflow
		tim->CNT = 0;
		events += timerWrapEvent(htim);
		if (!timerRunning(htim)) {
			break; // The callback stopped the counter
		}
	}
	while (remaining < 0) {
		const int64_t toWrap = tim->CNT; // counts until CNT == 0
		if (-remaining <= toWrap) {
			tim->CNT -= static_cast<uint32_t>(-remaining);
			break;
		}
		remaining += toWrap + 1; // 0 -> ARR: underflow
		tim->CNT = tim->ARR;
		events += timerWrapEvent(htim);
		if (!timerRunning(htim)) {
			break; // The callback stopped the counter
		}
	}
	return events;
}

uint32_t timerCounter(const TIM_HandleTypeDef *htim) { return htim->Instance->CNT; }
void timerSetCounter(TIM_HandleTypeDef *htim, uint32_t count) { htim->Instance->CNT = count; }

void encoderIndexPulse() {
	setPin(ENCODER_Z_GPIO_Port, ENCODER_Z_Pin, true);
	setPin(ENCODER_Z_GPIO_Port, ENCODER_Z_Pin, false);
	triggerExti(ENCODER_Z_Pin); // falling edge
	setPin(ENCODER_Z_GPIO_Port, ENCODER_Z_Pin, true);
}

void setPin(GPIO_TypeDef *port, uint16_t pin, bool high) { port->IDR = high ? (port->IDR | pin) : (port->IDR & ~(uint32_t)pin); }
bool outputPin(const GPIO_TypeDef *port, uint16_t pin) { return (port->ODR & pin) != 0; }

void triggerExti(uint16_t pin) {
	// MX_GPIO_Init configures only these pins as EXTI lines. Any other pin can not raise an interrupt on the target
	if (pin != BUTTON_A_Pin && pin != FLAG_Pin && pin != E_STOP_Pin && pin != ENCODER_Z_Pin) {
		targetState.unmodelledHalCalls.emplace_back("triggerExti on a pin without EXTI line");
		return;
	}
	// HAL_GPIO_EXTI_IRQHandler (stm32f4xx_hal_gpio.c): clears the pending bit and calls HAL_GPIO_EXTI_Callback
	HostPlatform::IsrContext isr;
	HAL_GPIO_EXTI_Callback(pin);
}

} // namespace HostTarget

namespace {
/** Models __HAL_TIM_DISABLE (stm32f4xx_hal_tim.h): clears CEN only if no channel output (CCxE / CCxNE) is enabled */
void halTimDisable(TIM_HandleTypeDef *htim) {
	if ((htim->Instance->CCER & TIM_CCER_CCxE_MASK) == 0U && (htim->Instance->CCER & TIM_CCER_CCxNE_MASK) == 0U) {
		htim->Instance->CR1 &= ~TIM_CR1_CEN;
	}
}
} // namespace

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
	halTimDisable(htim);
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
	halTimDisable(htim);
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
/**
 * RebootDFU / ExitDFU of Targets/F407VG/Core/Src/main.c: write a magic value to backup RAM (not modelled) and end
 * in NVIC_SystemReset(), so they are also counted in systemResets.
 */
void RebootDFU(void) {
	targetState.dfuReboots++;
	NVIC_SystemReset();
}
void ExitDFU(void) {
	targetState.dfuExits++;
	NVIC_SystemReset();
}

// ---------------- TinyUSB device (usbd.c, tusb.c) used by the F407 sources ----------------

/** Models tusb_rhport_init (tusb_init): initialises the device stack. Recorded, always succeeds */
bool tusb_rhport_init(uint8_t, const tusb_rhport_init_t *) {
	targetState.usbInits++;
	return true;
}

/**
 * Models tud_task_ext with an RTOS (usbd.c): waits on the USB event queue (osal_queue_receive, here forever)
 * and processes the events. The queue is modelled by the task notification of the calling (USB) task.
 */
void tud_task_ext(uint32_t, bool) {
	// One queued event (HostTarget::queueUsbEvent: a task notification) is consumed per call; without one the
	// calling task blocks like on the empty event queue. Event processing itself is not modelled (Tier B, P9).
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	targetState.usbEventsProcessed++;
}

/** Models tud_speed_get: speed of the current connection (set by the test, default full speed) */
tusb_speed_t tud_speed_get(void) { return (tusb_speed_t)targetState.usbSpeed; }

} // extern "C"
