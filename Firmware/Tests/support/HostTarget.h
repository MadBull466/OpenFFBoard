/*
 * HostTarget.h
 *
 * Test control interface of the F407 part of the fake platform (ffb_target_tests only):
 * timers (fake HAL TIM), external interrupts (EXTI), GPIO pins, chip ids, reset requests and the
 * peripheral objects of the target configuration (platform/src/host_target.cpp, host_target_config.cpp).
 *
 * Reset with HostTarget::reset() (done by AxisFixture / TargetFixture).
 */
#ifndef HOSTTEST_HOSTTARGET_H_
#define HOSTTEST_HOSTTARGET_H_

#include "main.h"

#include <cstdint>
#include <string>
#include <vector>

class MockCANPort;

// Peripheral handles of the target (normally defined by the CubeMX code in main.c)
extern TIM_HandleTypeDef htim3;	 // TIM_ENC: local quadrature encoder
extern TIM_HandleTypeDef htim13; // TIM_FFB: FFB update rate timer
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

namespace HostTarget {

struct State {
	// --- Recorded ---
	uint32_t systemResets = 0;		  //!< NVIC_SystemReset calls (the fake returns, the target would reset)
	uint32_t dfuReboots = 0;		  //!< RebootDFU calls
	uint32_t dfuExits = 0;			  //!< ExitDFU calls
	uint32_t usbInits = 0;			  //!< tusb_rhport_init calls (tusb_init)
	std::vector<std::string> unmodelledHalCalls; //!< HAL functions without a fake that were called (see host_stubs_target.cpp)

	// --- Controlled by the test ---
	uint32_t uid[3] = {0x00410032, 0x34385111, 0x30383632}; //!< HAL_GetUIDw0/1/2 (96 bit unique id)
	uint32_t devId = 0x413;									//!< HAL_GetDEVID: STM32F405/407
	uint32_t revId = 0x1007;								//!< HAL_GetREVID: revision 1
	uint8_t usbSpeed = 0; //!< tud_speed_get() result (tusb_speed_t). 0 = TUSB_SPEED_FULL
};
State &state();

/**
 * Restores the state after the CubeMX initialisation (MX_GPIO_Init, MX_TIMx_Init in main.c): timers initialised
 * and stopped (HAL_TIM_Base_Init: State READY, ARR = Period, PSC = Prescaler, CNT 0), inputs with pull-up high
 * (DIN0-7, E_STOP released), all other inputs low, outputs at their initial level (only SPI1_SS1 set),
 * chip ids default.
 * Does not touch the CAN port (MockCANPort) state other than clearing its records.
 */
void reset();

// --- Timers ---
/**
 * Simulates an update event (counter overflow) of a timer. Models HAL_TIM_IRQHandler for the update
 * interrupt: only if the counter is enabled (CR1.CEN) and the update interrupt is enabled (DIER.UIE)
 * HAL_TIM_PeriodElapsedCallback(htim) is called in interrupt context.
 * @return true if the callback was called
 */
bool timerUpdate(TIM_HandleTypeDef *htim);
/** True while the counter is enabled (CR1.CEN) */
bool timerRunning(const TIM_HandleTypeDef *htim);
/** True while the update interrupt is enabled (DIER.UIE) */
bool timerUpdateIrqEnabled(const TIM_HandleTypeDef *htim);

// --- GPIO and external interrupts ---
/** Sets the input level of a pin (IDR) */
void setPin(GPIO_TypeDef *port, uint16_t pin, bool high);
/** Output level of a pin (ODR) as written with HAL_GPIO_WritePin / TogglePin */
bool outputPin(const GPIO_TypeDef *port, uint16_t pin);
/**
 * Simulates an EXTI interrupt on a pin. Models HAL_GPIO_EXTI_IRQHandler: calls HAL_GPIO_EXTI_Callback(pin)
 * in interrupt context. Set the new pin level with setPin before.
 */
void triggerExti(uint16_t pin);

// --- Target configuration (host replacement of cpp_target_config.cpp) ---
/** The global `canport` of the firmware. A MockCANPort until the fake HAL CAN exists (P7) */
MockCANPort &canPort();

} // namespace HostTarget

#endif
