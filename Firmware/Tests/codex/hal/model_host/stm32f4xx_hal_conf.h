#ifndef CODEX_P12_MODEL_HAL_CONF_H
#define CODEX_P12_MODEL_HAL_CONF_H
#include "registers.h"
#define HAL_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define USE_RTOS 0U
#include "stm32f4xx_hal_gpio.h"
#ifdef USE_FULL_ASSERT
// Throwing happens only in the C++-compiled adapter target. Default C target
// keeps the production assertion-disabled configuration and abort handler.
extern "C++" void p12_parameter_failure(const char *, unsigned);
#define assert_param(expr) ((expr) ? (void)0U : p12_parameter_failure(#expr, __LINE__))
#else
#define assert_param(expr) ((void)0U)
#endif
// Lower boundary only: GPIO's request to enable SYSCFG, not real RCC code.
#define __HAL_RCC_SYSCFG_CLK_ENABLE() p12::enableSyscfgClock()
#endif
