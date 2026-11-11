/*
 * HalHostSystem.c: see HalHostSystem.h.
 */
#include "HalHostSystem.h"

#include "stm32f4xx_hal.h"

#include <stddef.h>

/* RM0090 Rev 22 section 7.2.2 (HSI = internal 16 MHz RC oscillator) and 7.3.3 (RCC_CFGR.SW reset value 00 = HSI selected as system clock). */
#define HOST_RESET_SYSCLK_HZ 16000000u

uint32_t SystemCoreClock = HOST_RESET_SYSCLK_HZ;

/* Right-shift for each value of RCC_CFGR.HPRE (RM0090 section 7.3.3): 0xxx = SYSCLK not divided, 1000 = /2,
 * 1001 = /4, 1010 = /8, 1011 = /16, 1100 = /64, 1101 = /128, 1110 = /256, 1111 = /512. Divisor 2^n -> shift n
 * (note: /32 does not exist, so the shifts jump from 4 to 6). */
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
/* Right-shift for each value of RCC_CFGR.PPRE1 / PPRE2 (RM0090 section 7.3.3): 0xx = AHB clock not divided,
 * 100 = /2, 101 = /4, 110 = /8, 111 = /16. */
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

extern __IO uint32_t uwTick; /* stm32f4xx_hal.c */

static int autoAdvance = 1;
static void (*stepFn)(void);
static uint32_t getTickCalls;

void hostSystemReset(void) {
	SystemCoreClock = HOST_RESET_SYSCLK_HZ;
	autoAdvance = 1;
	stepFn = NULL;
	getTickCalls = 0U;
}

void hostTickSet(uint32_t ticks) { uwTick = ticks; }

uint32_t hostTickPeek(void) { return uwTick; }

void hostTickSetAutoAdvance(int on) { autoAdvance = on; }

void hostTickSetStep(void (*step)(void)) { stepFn = step; }

uint32_t hostTickGetCalls(void) { return getTickCalls; }

/* Strong definition: replaces the __weak HAL_GetTick of stm32f4xx_hal.c (see the LIMIT in the header). */
uint32_t HAL_GetTick(void) {
	getTickCalls++;
	if (autoAdvance) {
		if (stepFn != NULL) {
			stepFn();
		} else {
			HAL_IncTick(); /* the real HAL function: uwTick += uwTickFreq */
		}
	}
	return uwTick;
}
