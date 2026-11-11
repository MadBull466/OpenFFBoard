/*
 * HalFixture.h (P12, ffb_hal_tests / ffb_hal_tests_disco)
 *
 * Fixture of every real ST HAL test case. It maps the register space (once per process), then resets everything a
 * test can change, so the test cases do not depend on their order:
 *  - every mapped register range to 0 (fakes/stm32_hal/HalRegisterSpace.h),
 *  - the CMSIS host intrinsics' counters, hook and exclusive monitor (fakes/stm32_hal/host_cmsis_gcc.h),
 *  - SystemCoreClock, the tick step and auto-advance (fakes/stm32_hal/HalHostSystem.h),
 *  - the HAL's own tick globals uwTick, uwTickFreq, uwTickPrio to their values at program start (captured by the first
 *    fixture before any test ran).
 * Plan: Tests/Code_Coverage/claude/plans/P12_ST_HAL_PLAN.md.
 */
#ifndef FFB_TESTS_HAL_FIXTURE_H
#define FFB_TESTS_HAL_FIXTURE_H

#include "HalHostSystem.h"
#include "HalRegisterSpace.h"
#include "host_cmsis_gcc.h"
#include "stm32f4xx_hal.h"

#include "doctest.h"

extern "C" {
extern __IO uint32_t uwTick;
extern uint32_t uwTickPrio;
extern HAL_TickFreqTypeDef uwTickFreq;
}

class HalFixture {
public:
	HalFixture() {
		static const uint32_t startTickPrio = uwTickPrio;
		static const HAL_TickFreqTypeDef startTickFreq = uwTickFreq;
		REQUIRE_MESSAGE(hostRegsMap() == 1, hostRegsError());
		hostRegsReset();
		hostCmsisReset();
		hostSystemReset();
		hostTickSet(0U);
		uwTickPrio = startTickPrio;
		uwTickFreq = startTickFreq;
	}
};

#endif /* FFB_TESTS_HAL_FIXTURE_H */
