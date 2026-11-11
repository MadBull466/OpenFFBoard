/*
 * HalFixture.h (P12, ffb_hal_tests / ffb_hal_tests_disco)
 *
 * Fixture of every real ST HAL test case. It maps the register space (once per process) and registers the register
 * models, then resets everything a test can change, so the test cases do not depend on their order:
 *  - the model state outside the registers (pin levels, GPIO lock state, ids; fakes/stm32_hal/HalPeripherals.h),
 *  - every mapped register range to 0 and then to the RM0090 reset values of the modelled registers, the fault list
 *    and the access log (fakes/stm32_hal/HalRegisterSpace.h),
 *  - the CMSIS host intrinsics' counters, hook and exclusive monitor (fakes/stm32_hal/host_cmsis_gcc.h),
 *  - SystemCoreClock, the tick step and auto-advance (fakes/stm32_hal/HalHostSystem.h),
 *  - the HAL's own tick globals uwTick, uwTickFreq, uwTickPrio to their values at program start (captured by the first
 *    fixture before any test ran).
 * Then it arms the bus: from here on every register access goes through the models. At the end of the test case it
 * disarms the bus and fails the test case for every fault the models recorded (unmodelled register, reserved
 * offset, write to a read-only register).
 * Plan: Tests/Code_Coverage/claude/plans/P12_ST_HAL_PLAN.md.
 */
#ifndef FFB_TESTS_HAL_FIXTURE_H
#define FFB_TESTS_HAL_FIXTURE_H

#include "HalHostSystem.h"
#include "HalPeripherals.h"
#include "HalRegisterSpace.h"
#include "host_cmsis_gcc.h"
#include "stm32f4xx_hal.h"

#include "doctest.h"

#include <cstddef>
#include <cstdint>

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
		hostPeriphRegisterAll();
		hostPeriphResetState();
		hostRegsReset();
		hostCmsisReset();
		hostSystemReset();
		hostTickSet(0U);
		uwTickPrio = startTickPrio;
		uwTickFreq = startTickFreq;
		hostRegsArm();
	}

	~HalFixture() {
		hostRegsDisarm();
		for (std::size_t i = 0; i < hostRegsFaultCount(); ++i) {
			FAIL_CHECK("register model fault: " << hostRegsFaultText(i));
		}
	}

	HalFixture(const HalFixture &) = delete;
	HalFixture &operator=(const HalFixture &) = delete;

	/* Faults recorded so far; a test that provokes a fault on purpose reads and clears them. */
	static std::size_t faults() { return hostRegsFaultCount(); }
	static void clearFaults() { hostRegsClearFaults(); }

	/* Raw register value (no trapping, not logged). */
	static uint32_t peek(const volatile void *reg) { return hostRegsPeek(reinterpret_cast<std::uintptr_t>(reg)); }
	static void poke(volatile void *reg, uint32_t value) { hostRegsPoke(reinterpret_cast<std::uintptr_t>(reg), value); }

	/* Word the CPU wrote with the last logged write to reg; 0 and fails the test if there was none. */
	static uint32_t lastWritten(const volatile void *reg) {
		const std::uintptr_t a = reinterpret_cast<std::uintptr_t>(reg);
		for (std::size_t i = hostRegsLogCount(); i > 0; --i) {
			const HostRegAccess *e = hostRegsLogAt(i - 1);
			if (e->write && e->addr == a) return e->written;
		}
		FAIL_CHECK("no write to the register was logged");
		return 0;
	}
};

#endif /* FFB_TESTS_HAL_FIXTURE_H */
