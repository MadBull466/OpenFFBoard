/*
 * HalHostSystem.h (P12 real ST HAL tests, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * The host pieces the real HAL links against that on target come from outside the HAL:
 *  1. CMSIS system globals SystemCoreClock, AHBPrescTable, APBPrescTable. On target they are defined in the CubeMX
 *     file Core/Src/system_stm32f4xx.c (out of scope, not compiled). Values: see HalHostSystem.c.
 *  2. The tick. On target uwTick (stm32f4xx_hal.c) is advanced by a timer interrupt that calls the real
 *     HAL_IncTick() (the firmware's HAL_InitTick in Core/Src/stm32f4xx_hal_timebase_tim.c uses TIM6 / TIM7).
 *     A HAL polling loop (`while ((HAL_GetTick() - tickstart) < Timeout)`) only ends because that interrupt fires
 *     meanwhile. The host has no interrupts, so this file provides a strong HAL_GetTick() that first runs one
 *     simulation step (default: one 1 ms timer interrupt = one call of the real HAL_IncTick()) and then returns
 *     uwTick, exactly what the real one returns. A test can replace the step (hostTickSetStep) to let a peripheral
 *     fake react instead, or stop time (hostTickSetAutoAdvance(0)).
 *
 * MODELS: the tick interrupt as "one interrupt per HAL_GetTick call" (deterministic, no sleep). HAL_IncTick,
 * HAL_Delay, HAL_SuspendTick, HAL_ResumeTick and HAL_GetTickFreq stay the real HAL functions.
 *
 * DOES NOT MODEL: real time between two HAL_GetTick calls (the host loop runs as fast as it can; one call = one
 * tick period), interrupt priorities, a stopped SysTick (HAL_SuspendTick clears SysTick TICKINT, but the firmware's
 * tick is TIM6/TIM7 and auto-advance does not look at either).
 *
 * LIMIT (reported in every P12 report): the real HAL_GetTick() (stm32f4xx_hal.c, "return uwTick;", __weak) is
 * replaced and therefore never runs in these tests. On target the firmware uses the real one.
 *
 * CONFIDENCE: High for the system tables (RM0090 field encodings, see the .c), Medium for the tick model.
 */
#ifndef FFB_HAL_HOST_SYSTEM_H
#define FFB_HAL_HOST_SYSTEM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Restores SystemCoreClock to its reset value, the default step and auto-advance on, counters to 0. Does not touch
 * uwTick (a HAL variable, reset with hostTickSet). */
void hostSystemReset(void);
/* Sets uwTick (the HAL's tick counter) directly, e.g. to test wrap-around. */
void hostTickSet(uint32_t ticks);
/* Reads uwTick without running a step (HAL_GetTick would run one). */
uint32_t hostTickPeek(void);
/* 1 = each HAL_GetTick() runs the step first (default), 0 = time stands still. */
void hostTickSetAutoAdvance(int on);
/* Replaces the step run by HAL_GetTick(); NULL restores the default (one HAL_IncTick()). */
void hostTickSetStep(void (*step)(void));
/* Number of HAL_GetTick() calls since hostSystemReset(). */
uint32_t hostTickGetCalls(void);

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_HOST_SYSTEM_H */
