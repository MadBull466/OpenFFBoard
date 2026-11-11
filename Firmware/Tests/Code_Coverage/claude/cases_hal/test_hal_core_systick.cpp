/*
 * P12 T1 HAL core cases that need the SysTick / NVIC / FLASH_ACR models (requirements C14-C17 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T1_REQUIREMENTS.md).
 * Sources under test: stm32f4xx_hal.c of the real ST HAL (HAL_Init, the weak default HAL_InitTick, HAL_SetTickFreq,
 * HAL_SuspendTick, HAL_ResumeTick), built into ffb_hal_tests and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 chapter 6 (1 ms time base, SysTick is the default source), PM0214 Rev 10 4.5.1-4.5.2
 * (CTRL bits ENABLE 0, TICKINT 1, CLKSOURCE 2; RELOAD = N - 1 for N clock cycles; 24-bit LOAD), 4.4.8 Table 52
 * (SysTick priority = SHPR3 bits 31:24), RM0090 Rev 22 3.9.1 (FLASH_ACR PRFTEN 8, ICEN 9, DCEN 10).
 * The 1 ms base at clock f is f / 1000 cycles, so LOAD = f / 1000 - 1 (16 MHz: 15999; 168 MHz: 167999).
 * HAL_TICK_FREQ_10HZ is a 100 ms period (HAL header constant, used as input).
 */
#include "HalFixture.h"

#include <cstdint>

extern "C" {
extern uint32_t SystemCoreClock;
}

namespace {

uint32_t shpr3() {
	const volatile uint32_t *w = reinterpret_cast<const volatile uint32_t *>(&SCB->SHP[0]);
	return w[2];
}

} // namespace

TEST_SUITE("P12 T1 HAL core SysTick") {

// ---- C14 ----

TEST_CASE_FIXTURE(HalFixture, "C14 HAL_InitTick at 16 MHz loads 15999 and returns HAL_OK") {
	CHECK(HAL_InitTick(4u) == HAL_OK);
	CHECK(SysTick->LOAD == 15999u); // 16 000 000 / 1000 = 16000 cycles per ms, RELOAD = N - 1
}

TEST_CASE_FIXTURE(HalFixture, "C14 HAL_InitTick at 168 MHz loads 167999") {
	SystemCoreClock = 168000000u;
	CHECK(HAL_InitTick(4u) == HAL_OK);
	CHECK(SysTick->LOAD == 167999u); // 168 000 000 / 1000 = 168000 cycles, RELOAD = N - 1
}

TEST_CASE_FIXTURE(HalFixture, "C14 HAL_InitTick starts SysTick with the interrupt on the processor clock") {
	REQUIRE(HAL_InitTick(4u) == HAL_OK);
	CHECK(SysTick->CTRL == 7u); // CLKSOURCE | TICKINT | ENABLE
}

TEST_CASE_FIXTURE(HalFixture, "C14 HAL_InitTick writes the priority into SysTick PRI_15 of SHPR3") {
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	REQUIRE(HAL_InitTick(5u) == HAL_OK);
	CHECK(shpr3() == 0x50000000u); // Table 52: SHPR3 31:24; priority 5 in bits 7:4 = 0x50
}

TEST_CASE_FIXTURE(HalFixture, "C14 HAL_InitTick with priority 0 and 15 gives the highest and lowest SysTick priority") {
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	REQUIRE(HAL_InitTick(0u) == HAL_OK);
	CHECK(shpr3() == 0x00000000u);
	REQUIRE(HAL_InitTick(15u) == HAL_OK);
	CHECK(shpr3() == 0xF0000000u);
}

TEST_CASE_FIXTURE(HalFixture, "C14 HAL_InitTick makes HAL_GetTickPrio return the priority") {
	REQUIRE(HAL_InitTick(7u) == HAL_OK);
	CHECK(HAL_GetTickPrio() == 7u);
}

TEST_CASE_FIXTURE(HalFixture, "C14 HAL_InitTick with priority 16 returns HAL_ERROR") {
	// 4 priority bits (PM0214 4.3.7): priorities 0..15 exist. UM1725 6.1.2 / HAL_InitTick: "HAL status".
	CHECK(HAL_InitTick(16u) == HAL_ERROR);
}

TEST_CASE_FIXTURE(HalFixture, "C14 characterization: HAL_InitTick with priority 16 has already started SysTick when it fails") {
	// NEEDS REVIEW: neither UM1725 nor PM0214 says whether a rejected priority may leave SysTick running. The HAL
	// configures the timer first and checks the priority afterwards.
	REQUIRE(HAL_InitTick(16u) == HAL_ERROR);
	CHECK(SysTick->LOAD == 15999u);
	CHECK(SysTick->CTRL == 7u);
}

TEST_CASE_FIXTURE(HalFixture, "C14 HAL_InitTick with a huge priority returns HAL_ERROR") {
	CHECK(HAL_InitTick(0xFFFFFFFFu) == HAL_ERROR);
}

TEST_CASE_FIXTURE(HalFixture, "C14 HAL_InitTick with 2 group bits encodes preemption = priority and subpriority 0") {
	// PM0214 Table 51: PRIGROUP 0b101 (NVIC_PRIORITYGROUP_2): group [7:6], sub [5:4]. HAL_InitTick uses sub 0.
	// Priority 3 -> 0b11 << 6 = 0xC0; priority 2 -> 0b10 << 6 = 0x80. SysTick priority is SHPR3 31:24 (Table 52).
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
	REQUIRE(HAL_InitTick(3u) == HAL_OK);
	CHECK(shpr3() == 0xC0000000u);
	REQUIRE(HAL_InitTick(2u) == HAL_OK);
	CHECK(shpr3() == 0x80000000u);
}

TEST_CASE_FIXTURE(HalFixture, "C14 HAL_InitTick with 3 group bits encodes preemption in bits 7:5") {
	// PM0214 Table 51: PRIGROUP 0b100 (NVIC_PRIORITYGROUP_3): group [7:5], sub [4]. Priority 5 = 0b101 << 5 = 0xA0.
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3);
	REQUIRE(HAL_InitTick(5u) == HAL_OK);
	CHECK(shpr3() == 0xA0000000u);
}

TEST_CASE_FIXTURE(HalFixture, "C14 HAL_InitTick with 0 group bits leaves no preemption bit, priority byte 0") {
	// PM0214 Table 51: PRIGROUP 0b111 (NVIC_PRIORITYGROUP_0): no group bits, sub [7:4]; HAL_InitTick uses sub 0, so 0x00.
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
	REQUIRE(HAL_InitTick(3u) == HAL_OK);
	CHECK(shpr3() == 0x00000000u);
}

// ---- C15 ----

TEST_CASE_FIXTURE(HalFixture, "C15 HAL_Init returns HAL_OK") {
	CHECK(HAL_Init() == HAL_OK);
}

TEST_CASE_FIXTURE(HalFixture, "C15 HAL_Init enables prefetch, instruction cache and data cache in FLASH_ACR") {
	REQUIRE(HAL_Init() == HAL_OK);
	// RM0090 3.9.1: PRFTEN bit 8, ICEN bit 9, DCEN bit 10 (the F407 hal_conf enables all three).
	CHECK((FLASH->ACR & (1u << 8)) != 0u);
	CHECK((FLASH->ACR & (1u << 9)) != 0u);
	CHECK((FLASH->ACR & (1u << 10)) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "C15 HAL_Init configures SysTick for a 1 ms time base at 16 MHz") {
	REQUIRE(HAL_Init() == HAL_OK);
	CHECK(SysTick->LOAD == 15999u); // 16 MHz / 1000 - 1
	CHECK((SysTick->CTRL & 3u) == 3u); // ENABLE | TICKINT
}

TEST_CASE_FIXTURE(HalFixture, "C15 characterization: HAL_Init selects priority grouping 4") {
	// NEEDS REVIEW: UM1725 6.1.4 does not name the grouping; the HAL sets NVIC_PRIORITYGROUP_4 (PRIGROUP 0b011).
	REQUIRE(HAL_Init() == HAL_OK);
	CHECK(HAL_NVIC_GetPriorityGrouping() == NVIC_PRIORITYGROUP_4);
}

TEST_CASE_FIXTURE(HalFixture, "C15 characterization: HAL_Init uses TICK_INT_PRIORITY for the tick") {
	// NEEDS REVIEW: UM1725 does not give the tick priority; the F407 hal_conf sets TICK_INT_PRIORITY.
	REQUIRE(HAL_Init() == HAL_OK);
	CHECK(HAL_GetTickPrio() == TICK_INT_PRIORITY);
	CHECK(shpr3() == (static_cast<uint32_t>(TICK_INT_PRIORITY) << 4) << 24);
}

// ---- C16 ----

TEST_CASE_FIXTURE(HalFixture, "C16 HAL_SetTickFreq to 10 Hz at 16 MHz returns HAL_OK") {
	uwTickPrio = 15u;
	CHECK(HAL_SetTickFreq(HAL_TICK_FREQ_10HZ) == HAL_OK);
	CHECK(HAL_GetTickFreq() == HAL_TICK_FREQ_10HZ);
}

TEST_CASE_FIXTURE(HalFixture, "C16 HAL_SetTickFreq to 10 Hz at 16 MHz loads 1 600 000 - 1") {
	uwTickPrio = 15u;
	REQUIRE(HAL_SetTickFreq(HAL_TICK_FREQ_10HZ) == HAL_OK);
	CHECK(SysTick->LOAD == 1599999u); // 100 ms = 1 600 000 cycles at 16 MHz, RELOAD = N - 1 (fits 24 bits)
}

TEST_CASE_FIXTURE(HalFixture, "C16 HAL_SetTickFreq fails when the reload exceeds 24 bits and returns HAL_ERROR") {
	uwTickPrio = 15u;
	SystemCoreClock = 168000000u;
	// 168 MHz at 10 Hz: 16 800 000 - 1 = 16 799 999 > 0xFFFFFF (16 777 215), does not fit STK_LOAD (PM0214 4.5.2).
	CHECK(HAL_SetTickFreq(HAL_TICK_FREQ_10HZ) == HAL_ERROR);
}

TEST_CASE_FIXTURE(HalFixture, "C16 HAL_SetTickFreq keeps the previous frequency when it fails") {
	uwTickPrio = 15u;
	SystemCoreClock = 168000000u;
	REQUIRE(HAL_SetTickFreq(HAL_TICK_FREQ_10HZ) == HAL_ERROR);
	CHECK(HAL_GetTickFreq() == HAL_TICK_FREQ_1KHZ);
}

TEST_CASE_FIXTURE(HalFixture, "C16 HAL_SetTickFreq to the current frequency returns HAL_OK without touching SysTick") {
	uwTickPrio = 15u;
	SysTick->LOAD = 0x77u;
	CHECK(HAL_SetTickFreq(HAL_TICK_FREQ_1KHZ) == HAL_OK);
	CHECK(SysTick->LOAD == 0x77u);
	CHECK(SysTick->CTRL == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "C16 HAL_SetTickFreq to 100 Hz at 168 MHz loads 1 680 000 - 1") {
	uwTickPrio = 15u;
	SystemCoreClock = 168000000u;
	REQUIRE(HAL_SetTickFreq(HAL_TICK_FREQ_100HZ) == HAL_OK);
	CHECK(HAL_GetTickFreq() == HAL_TICK_FREQ_100HZ);
	CHECK(SysTick->LOAD == 1679999u); // 10 ms = 1 680 000 cycles at 168 MHz, fits 24 bits
}

TEST_CASE_FIXTURE(HalFixture, "C16 HAL_SetTickFreq to 1 kHz at 168 MHz loads 167999 after 100 Hz") {
	uwTickPrio = 15u;
	SystemCoreClock = 168000000u;
	REQUIRE(HAL_SetTickFreq(HAL_TICK_FREQ_100HZ) == HAL_OK);
	REQUIRE(HAL_SetTickFreq(HAL_TICK_FREQ_1KHZ) == HAL_OK);
	CHECK(SysTick->LOAD == 167999u); // 1 ms = 168 000 cycles
}

TEST_CASE_FIXTURE(HalFixture, "C16 HAL_SetTickFreq fails for 100 Hz when the reload exceeds 24 bits") {
	uwTickPrio = 15u;
	// Hypothetical 1.7 GHz clock: 10 ms = 17 000 000 cycles, RELOAD 16 999 999 > 0xFFFFFF (16 777 215).
	SystemCoreClock = 1700000000u;
	CHECK(HAL_SetTickFreq(HAL_TICK_FREQ_100HZ) == HAL_ERROR);
	CHECK(HAL_GetTickFreq() == HAL_TICK_FREQ_1KHZ);
}

TEST_CASE_FIXTURE(HalFixture, "C16 HAL_SetTickFreq leaves SysTick LOAD and CTRL unchanged when it fails") {
	uwTickPrio = 15u;
	SystemCoreClock = 168000000u;
	SysTick->LOAD = 0x77u;
	SysTick->CTRL = 0u;
	REQUIRE(HAL_SetTickFreq(HAL_TICK_FREQ_10HZ) == HAL_ERROR);
	CHECK(SysTick->LOAD == 0x77u);
	CHECK(SysTick->CTRL == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "C16 HAL_SYSTICK_Config(0) returns non-zero and leaves SysTick unchanged") {
	// 0 - 1 wraps to 0xFFFFFFFF > 24 bits (PM0214 4.5.2: STK_LOAD is 24 bits), so the reload is rejected.
	SysTick->LOAD = 0x77u;
	SysTick->CTRL = 0u;
	CHECK(HAL_SYSTICK_Config(0u) != 0u);
	CHECK(SysTick->LOAD == 0x77u);
	CHECK(SysTick->CTRL == 0u);
}

// ---- C17 ----

TEST_CASE_FIXTURE(HalFixture, "C17 HAL_SuspendTick clears TICKINT and keeps ENABLE") {
	REQUIRE(HAL_SYSTICK_Config(16000u) == 0u); // CTRL = 7
	HAL_SuspendTick();
	CHECK(SysTick->CTRL == 5u); // TICKINT (bit 1) cleared; ENABLE and CLKSOURCE stay
}

TEST_CASE_FIXTURE(HalFixture, "C17 HAL_ResumeTick sets TICKINT and keeps ENABLE") {
	REQUIRE(HAL_SYSTICK_Config(16000u) == 0u);
	HAL_SuspendTick();
	HAL_ResumeTick();
	CHECK(SysTick->CTRL == 7u);
}

TEST_CASE_FIXTURE(HalFixture, "C17 HAL_SuspendTick and HAL_ResumeTick do not change LOAD") {
	REQUIRE(HAL_SYSTICK_Config(16000u) == 0u);
	HAL_SuspendTick();
	CHECK(SysTick->LOAD == 15999u);
	HAL_ResumeTick();
	CHECK(SysTick->LOAD == 15999u);
}

} // TEST_SUITE
