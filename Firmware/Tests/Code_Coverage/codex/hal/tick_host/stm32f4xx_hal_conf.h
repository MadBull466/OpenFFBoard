#ifndef CODEX_P12_TICK_CONF_H
#define CODEX_P12_TICK_CONF_H
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define USE_RTOS 0U
// Minimal tick-only configuration: HAL_Init/DeInit and caches are not tested.
#define INSTRUCTION_CACHE_ENABLE 0U
#define DATA_CACHE_ENABLE 0U
#define PREFETCH_ENABLE 0U
#define TICK_INT_PRIORITY 4U // supplied F407VG configuration
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_dma.h"
#include "tick_boundary.h"
#define assert_param(expr) ((void)0U)
#define __HAL_RCC_APB1_FORCE_RESET() p12_tick_unsupported("APB1 reset")
#define __HAL_RCC_APB1_RELEASE_RESET() p12_tick_unsupported("APB1 reset release")
#define __HAL_RCC_APB2_FORCE_RESET() p12_tick_unsupported("APB2 reset")
#define __HAL_RCC_APB2_RELEASE_RESET() p12_tick_unsupported("APB2 reset release")
#define __HAL_RCC_AHB1_FORCE_RESET() p12_tick_unsupported("AHB1 reset")
#define __HAL_RCC_AHB1_RELEASE_RESET() p12_tick_unsupported("AHB1 reset release")
#define __HAL_RCC_AHB2_FORCE_RESET() p12_tick_unsupported("AHB2 reset")
#define __HAL_RCC_AHB2_RELEASE_RESET() p12_tick_unsupported("AHB2 reset release")
#define __HAL_RCC_AHB3_FORCE_RESET() p12_tick_unsupported("AHB3 reset")
#define __HAL_RCC_AHB3_RELEASE_RESET() p12_tick_unsupported("AHB3 reset release")
#endif
