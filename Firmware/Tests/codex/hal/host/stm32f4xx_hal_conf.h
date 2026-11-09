#ifndef CODEX_P12_HAL_CONF_H
#define CODEX_P12_HAL_CONF_H
/* Owned host configuration: only GPIO, using genuine HAL and device headers.
 * Real board configuration is inventoried separately; this slice is not a
 * reproduction of its complete enabled-module set.
 */
#define HAL_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define USE_RTOS 0U
#include "stm32f4xx_hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif
void p12_unsupported(const char *operation);
void assert_failed(uint8_t *file, uint32_t line);
#ifdef __cplusplus
}
#endif
#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
#else
#define assert_param(expr) ((void)0U)
#endif

/* This first slice only permits explicit caller-owned GPIO storage. Fail
 * before an unsupported system-peripheral access; never map host memory at
 * device physical addresses and never silently enable a peripheral clock.
 */
#undef SYSCFG
#undef EXTI
#define SYSCFG (p12_unsupported("SYSCFG"), (SYSCFG_TypeDef *)0)
#define EXTI (p12_unsupported("EXTI"), (EXTI_TypeDef *)0)
#define __HAL_RCC_SYSCFG_CLK_ENABLE() p12_unsupported("SYSCFG clock")
#endif
