/*
 * P12 T1 CORTEX cases (requirements C1-C13 of Tests/Code_Coverage/claude/charters/hal/P12_T1_REQUIREMENTS.md).
 * Source under test: stm32f4xx_hal_cortex.c of the real ST HAL, built into ffb_hal_tests and ffb_hal_tests_disco.
 *
 * Oracles: PM0214 Rev 10 chapter 4 (4.2.6-4.2.9 MPU, 4.3.2-4.3.8 NVIC, 4.4.5 AIRCR with Table 51, 4.4.8 SHPR with
 * Table 52, 4.4.9 SHCSR, 4.5.1-4.5.3 SysTick), UM1725 Rev 8 chapter 11, RM0090 Rev 22 Table 62 (IRQ numbers).
 *
 * Table 51 (PM0214 4.4.5), with the 4 implemented priority bits 7:4 of the STM32F4:
 *   PRIGROUP 0b011: group priority bits [7:4], no subpriority      (NVIC_PRIORITYGROUP_4, 4 group bits)
 *   PRIGROUP 0b100: group [7:5], sub [4]                           (NVIC_PRIORITYGROUP_3)
 *   PRIGROUP 0b101: group [7:6], sub [5:4]                         (NVIC_PRIORITYGROUP_2)
 *   PRIGROUP 0b110: group [7],   sub [6:4]                         (NVIC_PRIORITYGROUP_1)
 *   PRIGROUP 0b111: no group,    sub [7:4]                         (NVIC_PRIORITYGROUP_0)
 * So the PRIGROUP of a group with n group bits is 7 - n, and the priority byte is pre << (8 - n) | sub << 4.
 *
 * This file also defines the strong HAL_SYSTICK_Callback recorder (C10), the only place that does.
 */
#include "HalCorePeripherals.h"
#include "HalFixture.h"

#include <csetjmp>
#include <cstdint>

extern "C" {
static volatile unsigned g_sysTickCallbacks = 0;
void HAL_SYSTICK_Callback(void) { g_sysTickCallbacks = g_sysTickCallbacks + 1; }
}

namespace {

const uint32_t kGroupValue[5] = {NVIC_PRIORITYGROUP_0, NVIC_PRIORITYGROUP_1, NVIC_PRIORITYGROUP_2,
                                 NVIC_PRIORITYGROUP_3, NVIC_PRIORITYGROUP_4};

// Priority byte of interrupt irq (0..239), read through the bus (PM0214 4.3.7: bits 7:4 implemented).
uint32_t iprByte(unsigned irq) {
	const volatile uint32_t *w = reinterpret_cast<const volatile uint32_t *>(&NVIC->IP[irq & ~3u]);
	return (*w >> (8u * (irq & 3u))) & 0xFFu;
}

// SHPR1..3 as words (PM0214 4.4.8, 0xE000ED18 / 1C / 20).
uint32_t shpr(unsigned n) {
	const volatile uint32_t *w = reinterpret_cast<const volatile uint32_t *>(&SCB->SHP[0]);
	return w[n - 1u];
}

uint32_t mpuRasr(unsigned region) {
	MPU->RNR = region;
	return MPU->RASR;
}
uint32_t mpuRbar(unsigned region) {
	MPU->RNR = region;
	return MPU->RBAR;
}

// C13 helper: a plain function so that no C++ object with a destructor lives between setjmp and longjmp.
std::jmp_buf g_resetJump;
void leaveEndlessLoop() { std::longjmp(g_resetJump, 1); }
int systemResetReturnsViaHook() {
	hostCmsisNopHook = leaveEndlessLoop;
	if (setjmp(g_resetJump) == 0) {
		HAL_NVIC_SystemReset();
		hostCmsisNopHook = nullptr;
		return 0; // the function returned
	}
	hostCmsisNopHook = nullptr;
	return 1; // left through the endless loop
}

} // namespace

TEST_SUITE("P12 T1 CORTEX") {

// ---- C1 ----

TEST_CASE_FIXTURE(HalFixture, "C1 HAL_NVIC_SetPriorityGrouping stores PRIGROUP = 7 - group bits with the key") {
	// PM0214 Table 51 only requires PRIGROUP 0b0xx for "no subpriority"; the exact 0b011 (and 7 - n for the others)
	// comes from the HAL's NVIC_PRIORITYGROUP_x constants, which UM1725 names, not from PM0214.
	// PM0214 4.4.5 / Table 51: n group bits -> PRIGROUP 7 - n; AIRCR reads VECTKEY field 0xFA05 << 16 | PRIGROUP << 8.
	const uint32_t prigroup[5] = {7u, 6u, 5u, 4u, 3u};
	for (unsigned n = 0; n < 5; ++n) {
		INFO("group bits " << n);
		HAL_NVIC_SetPriorityGrouping(kGroupValue[n]);
		CHECK(SCB->AIRCR == ((0xFA05u << 16) | (prigroup[n] << 8)));
	}
}

TEST_CASE_FIXTURE(HalFixture, "C1 HAL_NVIC_GetPriorityGrouping returns the group value that was set") {
	for (unsigned n = 0; n < 5; ++n) {
		INFO("group bits " << n);
		HAL_NVIC_SetPriorityGrouping(kGroupValue[n]);
		CHECK(HAL_NVIC_GetPriorityGrouping() == kGroupValue[n]);
	}
}

TEST_CASE_FIXTURE(HalFixture, "C1 HAL_NVIC_SetPriorityGrouping does not request a reset") {
	// PM0214 4.4.5: SYSRESETREQ (bit 2) must stay 0 when only the grouping is changed.
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	CHECK(hostScbResetRequests() == 0u);
}

// ---- C2 ----
// For each case the neighbours of the interrupt hold other priorities that must survive (PM0214 4.3.7: one byte each).

namespace {
void checkIprForGroup(unsigned n, unsigned pre, unsigned sub) {
	const unsigned irqs[3] = {6u /*EXTI0*/, 40u /*EXTI15_10*/, 81u /*FPU*/};
	for (unsigned irq : irqs) {
		INFO("group bits " << n << " irq " << irq);
		HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
		HAL_NVIC_SetPriority(static_cast<IRQn_Type>(irq - 1u), 0xAu, 0u); // byte 0xA0
		if (irq + 1u < 82u) HAL_NVIC_SetPriority(static_cast<IRQn_Type>(irq + 1u), 0x5u, 0u); // byte 0x50
		HAL_NVIC_SetPriorityGrouping(kGroupValue[n]);
		HAL_NVIC_SetPriority(static_cast<IRQn_Type>(irq), pre, sub);
		// Table 51: byte = pre << (8 - n) | sub << 4.
		CHECK(iprByte(irq) == ((pre << (8u - n)) | (sub << 4)));
		CHECK(iprByte(irq - 1u) == 0xA0u);
		if (irq + 1u < 82u) CHECK(iprByte(irq + 1u) == 0x50u);
	}
}
} // namespace

TEST_CASE_FIXTURE(HalFixture, "C2 HAL_NVIC_SetPriority with 4 group bits puts the preemption in bits 7:4") {
	checkIprForGroup(4, 0x5u, 0u); // 0x5 << 4 = 0x50
}

TEST_CASE_FIXTURE(HalFixture, "C2 HAL_NVIC_SetPriority with 3 group bits splits 7:5 and 4") {
	checkIprForGroup(3, 5u, 1u); // 101 << 5 = 0xA0, 1 << 4 = 0x10: 0xB0
}

TEST_CASE_FIXTURE(HalFixture, "C2 HAL_NVIC_SetPriority with 2 group bits splits 7:6 and 5:4") {
	checkIprForGroup(2, 2u, 1u); // 10 << 6 = 0x80, 01 << 4 = 0x10: 0x90
}

TEST_CASE_FIXTURE(HalFixture, "C2 HAL_NVIC_SetPriority with 1 group bit splits 7 and 6:4") {
	checkIprForGroup(1, 1u, 5u); // 1 << 7 = 0x80, 101 << 4 = 0x50: 0xD0
}

TEST_CASE_FIXTURE(HalFixture, "C2 HAL_NVIC_SetPriority with 0 group bits puts the subpriority in bits 7:4") {
	checkIprForGroup(0, 0u, 9u); // 9 << 4 = 0x90
}

TEST_CASE_FIXTURE(HalFixture, "C2 HAL_NVIC_SetPriority can write the highest and the lowest priority of the interrupt") {
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_NVIC_SetPriority(EXTI0_IRQn, 15u, 0u); // 15 << 4 = 0xF0 (lowest)
	CHECK(iprByte(6) == 0xF0u);
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0u, 0u); // highest
	CHECK(iprByte(6) == 0x00u);
}

// ---- C3 ----

TEST_CASE_FIXTURE(HalFixture, "C3 HAL_NVIC_SetPriority of SysTick writes PRI_15 (SHPR3 bits 31:24)") {
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0xAu, 0u); // Table 52: SysTick is exception 15, SHPR3 31:24; 0xA << 4 = 0xA0
	CHECK(shpr(3) == 0xA0000000u);
	CHECK(shpr(1) == 0u);
	CHECK(shpr(2) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "C3 HAL_NVIC_SetPriority of PendSV writes PRI_14 (SHPR3 bits 23:16)") {
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_NVIC_SetPriority(PendSV_IRQn, 0x6u, 0u); // exception 14, SHPR3 23:16; 0x60
	CHECK(shpr(3) == 0x00600000u);
}

TEST_CASE_FIXTURE(HalFixture, "C3 HAL_NVIC_SetPriority of SVCall writes PRI_11 (SHPR2 bits 31:24)") {
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_NVIC_SetPriority(SVCall_IRQn, 0x3u, 0u); // exception 11, SHPR2 31:24; 0x30
	CHECK(shpr(2) == 0x30000000u);
	CHECK(shpr(3) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "C3 HAL_NVIC_SetPriority of MemoryManagement writes PRI_4 (SHPR1 bits 7:0)") {
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0x9u, 0u); // exception 4, SHPR1 7:0; 0x90
	CHECK(shpr(1) == 0x00000090u);
}

TEST_CASE_FIXTURE(HalFixture, "C3 HAL_NVIC_SetPriority of BusFault and UsageFault write PRI_5 and PRI_6 of SHPR1") {
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0x2u, 0u);   // exception 5, SHPR1 15:8; 0x20
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0x7u, 0u); // exception 6, SHPR1 23:16; 0x70
	CHECK(shpr(1) == 0x00702000u);
}

TEST_CASE_FIXTURE(HalFixture, "C3 system exception priorities do not touch the interrupt priorities") {
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0xFu, 0u);
	for (unsigned irq = 0; irq < 82; ++irq) {
		INFO("irq " << irq);
		CHECK(iprByte(irq) == 0u);
	}
}

// ---- C4 ----

TEST_CASE_FIXTURE(HalFixture, "C4 HAL_NVIC_GetPriority returns the preemption and subpriority that were set") {
	for (unsigned n = 0; n < 5; ++n) {
		const unsigned preMax = (1u << n) - 1u;
		const unsigned subMax = (1u << (4u - n)) - 1u;
		const unsigned vals[3][2] = {{0u, 0u}, {preMax, subMax}, {preMax / 2u, subMax / 2u}};
		HAL_NVIC_SetPriorityGrouping(kGroupValue[n]);
		for (const auto &v : vals) {
			INFO("group bits " << n << " pre " << v[0] << " sub " << v[1]);
			uint32_t pre = 99u, sub = 99u;
			HAL_NVIC_SetPriority(EXTI15_10_IRQn, v[0], v[1]);
			HAL_NVIC_GetPriority(EXTI15_10_IRQn, HAL_NVIC_GetPriorityGrouping(), &pre, &sub);
			CHECK(pre == v[0]);
			CHECK(sub == v[1]);
		}
	}
}

TEST_CASE_FIXTURE(HalFixture, "C4 HAL_NVIC_GetPriority of a system exception returns what was set") {
	uint32_t pre = 99u, sub = 99u;
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
	HAL_NVIC_SetPriority(SysTick_IRQn, 3u, 2u);
	HAL_NVIC_GetPriority(SysTick_IRQn, NVIC_PRIORITYGROUP_2, &pre, &sub);
	CHECK(pre == 3u);
	CHECK(sub == 2u);
}

// ---- C5 ----

TEST_CASE_FIXTURE(HalFixture, "C5 HAL_NVIC_EnableIRQ sets only the enable bit of that interrupt (ISER words 0, 1, 2)") {
	const unsigned irqs[3] = {6u, 40u, 81u}; // words 0, 1, 2 (PM0214 4.3.2)
	for (unsigned irq : irqs) {
		INFO("irq " << irq);
		HAL_NVIC_EnableIRQ(static_cast<IRQn_Type>(irq));
		for (unsigned i = 0; i < 82; ++i) CHECK(hostNvicEnabled(i) == (i == irq ? 1 : 0));
		HAL_NVIC_DisableIRQ(static_cast<IRQn_Type>(irq));
	}
}

TEST_CASE_FIXTURE(HalFixture, "C5 HAL_NVIC_DisableIRQ clears only the enable bit of that interrupt (ICER words 0, 1, 2)") {
	const unsigned irqs[3] = {6u, 40u, 81u};
	for (unsigned irq : irqs) HAL_NVIC_EnableIRQ(static_cast<IRQn_Type>(irq));
	for (unsigned irq : irqs) {
		INFO("irq " << irq);
		HAL_NVIC_DisableIRQ(static_cast<IRQn_Type>(irq));
		CHECK(hostNvicEnabled(irq) == 0);
	}
	// disabled one by one: the two others stayed enabled until their turn
	for (unsigned irq : irqs) HAL_NVIC_EnableIRQ(static_cast<IRQn_Type>(irq));
	HAL_NVIC_DisableIRQ(static_cast<IRQn_Type>(40));
	CHECK(hostNvicEnabled(6) == 1);
	CHECK(hostNvicEnabled(40) == 0);
	CHECK(hostNvicEnabled(81) == 1);
}

// ---- C6 ----

TEST_CASE_FIXTURE(HalFixture, "C6 HAL_NVIC_SetPendingIRQ makes only that interrupt pending (ISPR)") {
	const unsigned irqs[3] = {6u, 40u, 81u};
	for (unsigned irq : irqs) {
		INFO("irq " << irq);
		HAL_NVIC_SetPendingIRQ(static_cast<IRQn_Type>(irq));
		for (unsigned i = 0; i < 82; ++i) CHECK(hostNvicPending(i) == (i == irq ? 1 : 0));
		HAL_NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(irq));
	}
}

TEST_CASE_FIXTURE(HalFixture, "C6 HAL_NVIC_GetPendingIRQ returns 1 for a pending and 0 for a not pending interrupt") {
	const unsigned irqs[3] = {6u, 40u, 81u};
	for (unsigned irq : irqs) {
		INFO("irq " << irq);
		CHECK(HAL_NVIC_GetPendingIRQ(static_cast<IRQn_Type>(irq)) == 0u);
		HAL_NVIC_SetPendingIRQ(static_cast<IRQn_Type>(irq));
		CHECK(HAL_NVIC_GetPendingIRQ(static_cast<IRQn_Type>(irq)) == 1u);
	}
}

TEST_CASE_FIXTURE(HalFixture, "C6 HAL_NVIC_ClearPendingIRQ clears only that interrupt (ICPR)") {
	const unsigned irqs[3] = {6u, 40u, 81u};
	for (unsigned irq : irqs) HAL_NVIC_SetPendingIRQ(static_cast<IRQn_Type>(irq));
	HAL_NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(40));
	CHECK(hostNvicPending(6) == 1);
	CHECK(hostNvicPending(40) == 0);
	CHECK(hostNvicPending(81) == 1);
	CHECK(HAL_NVIC_GetPendingIRQ(EXTI15_10_IRQn) == 0u);
}

// ---- C7 ----

TEST_CASE_FIXTURE(HalFixture, "C7 HAL_NVIC_GetActive returns 1 for an active and 0 for an inactive interrupt (IABR)") {
	const unsigned irqs[3] = {6u, 40u, 81u};
	for (unsigned irq : irqs) {
		INFO("irq " << irq);
		CHECK(HAL_NVIC_GetActive(static_cast<IRQn_Type>(irq)) == 0u);
		hostNvicSetActive(irq, 1);
		CHECK(HAL_NVIC_GetActive(static_cast<IRQn_Type>(irq)) == 1u);
		for (unsigned other : irqs) {
			if (other != irq) CHECK(HAL_NVIC_GetActive(static_cast<IRQn_Type>(other)) == 0u);
		}
		hostNvicSetActive(irq, 0);
		CHECK(HAL_NVIC_GetActive(static_cast<IRQn_Type>(irq)) == 0u);
	}
}

// ---- C8 ----
// PM0214 4.5.2: "a period of N processor clock cycles: use a RELOAD value of N-1"; 4.5.1 bit 0 ENABLE, bit 1 TICKINT,
// bit 2 CLKSOURCE (1 = processor clock); UM1725 11: SysTick_Config sets the lowest priority 0x0F, resets the counter,
// selects HCLK, enables the interrupt and starts the counter.

TEST_CASE_FIXTURE(HalFixture, "C8 HAL_SYSTICK_Config(n) loads n - 1 into STK_LOAD and returns 0") {
	const uint32_t counts[4] = {1u, 2u, 16000u, 0x1000000u};
	for (uint32_t n : counts) {
		INFO("n " << n);
		CHECK(HAL_SYSTICK_Config(n) == 0u);
		CHECK(SysTick->LOAD == n - 1u);
	}
}

TEST_CASE_FIXTURE(HalFixture, "C8 HAL_SYSTICK_Config clears the current value register") {
	poke(&SysTick->VAL, 0x1234u);
	REQUIRE(HAL_SYSTICK_Config(16000u) == 0u);
	CHECK(SysTick->VAL == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "C8 HAL_SYSTICK_Config sets ENABLE and TICKINT") {
	REQUIRE(HAL_SYSTICK_Config(16000u) == 0u);
	const uint32_t ctrl = SysTick->CTRL;
	CHECK((ctrl & 1u) == 1u); // ENABLE, bit 0
	CHECK((ctrl & 2u) == 2u); // TICKINT, bit 1
}

TEST_CASE_FIXTURE(HalFixture, "C8 HAL_SYSTICK_Config selects the processor clock as clock source") {
	// UM1725 11: "Configures the SysTick Counter clock source to be Core Clock Source (HCLK)"; PM0214 4.5.1: bit 2 = 1.
	REQUIRE(HAL_SYSTICK_Config(16000u) == 0u);
	CHECK((SysTick->CTRL & 4u) == 4u);
}

TEST_CASE_FIXTURE(HalFixture, "C8 HAL_SYSTICK_Config sets the SysTick priority to the lowest value 0x0F") {
	// UM1725 11: "Configures the SysTick IRQ priority to the lowest value 0x0F"; 4 implemented bits: PRI_15 = 0xF0.
	REQUIRE(HAL_SYSTICK_Config(16000u) == 0u);
	CHECK(shpr(3) == 0xF0000000u);
}

TEST_CASE_FIXTURE(HalFixture, "C8 HAL_SYSTICK_Config rejects a reload that exceeds 24 bits and leaves SysTick unchanged") {
	// n = 0x1000001: RELOAD = 0x1000000 does not fit the 24-bit STK_LOAD (PM0214 4.5.2).
	SysTick->LOAD = 0x55u;
	poke(&SysTick->VAL, 0x66u);
	CHECK(HAL_SYSTICK_Config(0x1000001u) != 0u);
	CHECK(SysTick->LOAD == 0x55u);
	CHECK(SysTick->VAL == 0x66u);
	CHECK(SysTick->CTRL == 0u);
	CHECK(shpr(3) == 0u);
}

// ---- C9 ----

TEST_CASE_FIXTURE(HalFixture, "C9 HAL_SYSTICK_CLKSourceConfig(HCLK_DIV8) clears CLKSOURCE and keeps the other bits") {
	REQUIRE(HAL_SYSTICK_Config(1000u) == 0u); // CTRL = ENABLE | TICKINT | CLKSOURCE
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);
	CHECK(SysTick->CTRL == 3u); // PM0214 4.5.1: bit 2 = 0 means AHB/8; ENABLE and TICKINT stay
}

TEST_CASE_FIXTURE(HalFixture, "C9 HAL_SYSTICK_CLKSourceConfig(HCLK) sets CLKSOURCE and keeps the other bits") {
	REQUIRE(HAL_SYSTICK_Config(1000u) == 0u);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	CHECK(SysTick->CTRL == 7u); // bit 2 = 1: processor clock
}

TEST_CASE_FIXTURE(HalFixture, "C9 HAL_SYSTICK_CLKSourceConfig does not change LOAD") {
	REQUIRE(HAL_SYSTICK_Config(1000u) == 0u);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);
	CHECK(SysTick->LOAD == 999u);
}

// ---- C10 ----

TEST_CASE_FIXTURE(HalFixture, "C10 HAL_SYSTICK_IRQHandler calls HAL_SYSTICK_Callback once") {
	g_sysTickCallbacks = 0;
	HAL_SYSTICK_IRQHandler();
	CHECK(g_sysTickCallbacks == 1u);
	HAL_SYSTICK_IRQHandler();
	CHECK(g_sysTickCallbacks == 2u);
}

// ---- C11 ----
// PM0214 4.2.6 MPU_CTRL: ENABLE 0, HFNMIENA 1, PRIVDEFENA 2; 4.4.9 SHCSR.MEMFAULTENA bit 16.

TEST_CASE_FIXTURE(HalFixture, "C11 HAL_MPU_Enable sets MPU_CTRL = control | ENABLE") {
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT); // PRIVDEFENA, bit 2 -> 0b101
	CHECK(MPU->CTRL == 5u);
	HAL_MPU_Disable();
	HAL_MPU_Enable(MPU_HFNMI_PRIVDEF); // HFNMIENA | PRIVDEFENA, bits 1 and 2 -> 0b111
	CHECK(MPU->CTRL == 7u);
}

TEST_CASE_FIXTURE(HalFixture, "C11 HAL_MPU_Enable sets MEMFAULTENA in SHCSR and keeps the other bits") {
	SCB->SHCSR = (1u << 17); // BUSFAULTENA, bit 17
	HAL_MPU_Enable(0u);
	CHECK(SCB->SHCSR == ((1u << 17) | (1u << 16)));
}

TEST_CASE_FIXTURE(HalFixture, "C11 HAL_MPU_Disable clears MPU_CTRL") {
	HAL_MPU_Enable(MPU_HFNMI_PRIVDEF);
	REQUIRE(MPU->CTRL == 7u);
	HAL_MPU_Disable();
	CHECK(MPU->CTRL == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "C11 HAL_MPU_Disable clears MEMFAULTENA in SHCSR and keeps the other bits") {
	SCB->SHCSR = (1u << 17);
	HAL_MPU_Enable(0u);
	REQUIRE(SCB->SHCSR == ((1u << 17) | (1u << 16)));
	HAL_MPU_Disable();
	CHECK(SCB->SHCSR == (1u << 17));
}

TEST_CASE_FIXTURE(HalFixture, "C11 HAL_MPU_ConfigRegion stores the RASR fields at their PM0214 4.2.9 positions") {
	MPU_Region_InitTypeDef r = {};
	r.Enable = MPU_REGION_ENABLE;
	r.Number = MPU_REGION_NUMBER3;
	r.BaseAddress = 0x20010000u;
	r.Size = MPU_REGION_SIZE_64KB; // SIZE field 0xF (2^(15+1) = 64 KB)
	r.SubRegionDisable = 0xA5u;
	r.TypeExtField = MPU_TEX_LEVEL2;
	r.AccessPermission = MPU_REGION_PRIV_RO; // AP 0b101
	r.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	r.IsShareable = MPU_ACCESS_SHAREABLE;
	r.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	r.IsBufferable = MPU_ACCESS_BUFFERABLE;
	HAL_MPU_ConfigRegion(&r);
	// XN 28 | AP 26:24 (5) | TEX 21:19 (2) | S 18 | C 17 (0) | B 16 | SRD 15:8 (0xA5) | SIZE 5:1 (0xF) | ENABLE 0
	const uint32_t expected = (1u << 28) | (5u << 24) | (2u << 19) | (1u << 18) | (0u << 17) | (1u << 16) |
	                          (0xA5u << 8) | (0xFu << 1) | 1u; // = 0x1515A51F
	CHECK(mpuRasr(3) == expected);
	CHECK(expected == 0x1515A51Fu);
}

TEST_CASE_FIXTURE(HalFixture, "C11 HAL_MPU_ConfigRegion stores the opposite RASR field values") {
	MPU_Region_InitTypeDef r = {};
	r.Enable = MPU_REGION_DISABLE;
	r.Number = MPU_REGION_NUMBER5;
	r.BaseAddress = 0x20002000u;
	r.Size = MPU_REGION_SIZE_4KB; // SIZE 0xB
	r.SubRegionDisable = 0u;
	r.TypeExtField = MPU_TEX_LEVEL0;
	r.AccessPermission = MPU_REGION_PRIV_RW; // AP 0b001
	r.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
	r.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	r.IsCacheable = MPU_ACCESS_CACHEABLE;
	r.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
	HAL_MPU_ConfigRegion(&r);
	// AP 1 << 24 | C 1 << 17 | SIZE 0xB << 1 (0x16), ENABLE 0
	CHECK(mpuRasr(5) == ((1u << 24) | (1u << 17) | (0xBu << 1))); // = 0x01020016
}

TEST_CASE_FIXTURE(HalFixture, "C11 HAL_MPU_ConfigRegion stores the base address in RBAR of the selected region") {
	MPU_Region_InitTypeDef r = {};
	r.Number = MPU_REGION_NUMBER3;
	r.BaseAddress = 0x20010000u;
	r.Size = MPU_REGION_SIZE_64KB;
	HAL_MPU_ConfigRegion(&r);
	// PM0214 4.2.8: ADDR in bits 31:5; a read returns the region number of MPU_RNR in bits 3:0.
	CHECK(mpuRbar(3) == (0x20010000u | 3u));
}

TEST_CASE_FIXTURE(HalFixture, "C11 HAL_MPU_ConfigRegion of one region keeps the other regions") {
	MPU_Region_InitTypeDef a = {};
	a.Enable = MPU_REGION_ENABLE;
	a.Number = MPU_REGION_NUMBER1;
	a.BaseAddress = 0x20000000u;
	a.Size = MPU_REGION_SIZE_32KB; // SIZE 0xE
	a.AccessPermission = MPU_REGION_FULL_ACCESS; // AP 0b011
	MPU_Region_InitTypeDef b = a;
	b.Number = MPU_REGION_NUMBER4;
	b.BaseAddress = 0x20008000u;
	b.Size = MPU_REGION_SIZE_1KB; // SIZE 0x9
	b.AccessPermission = MPU_REGION_PRIV_RO_URO; // AP 0b110
	HAL_MPU_ConfigRegion(&a);
	HAL_MPU_ConfigRegion(&b);
	CHECK(mpuRasr(1) == ((3u << 24) | (0xEu << 1) | 1u));
	CHECK(mpuRbar(1) == (0x20000000u | 1u));
	CHECK(mpuRasr(4) == ((6u << 24) | (0x9u << 1) | 1u));
	CHECK(mpuRbar(4) == (0x20008000u | 4u));
}

TEST_CASE_FIXTURE(HalFixture, "C11 HAL_MPU_EnableRegion selects the region in RNR and sets ENABLE of that region only") {
	MPU_Region_InitTypeDef a = {};
	a.Enable = MPU_REGION_DISABLE;
	a.Number = MPU_REGION_NUMBER1;
	a.BaseAddress = 0x20000000u;
	a.Size = MPU_REGION_SIZE_32KB;
	MPU_Region_InitTypeDef b = a;
	b.Number = MPU_REGION_NUMBER2;
	b.BaseAddress = 0x20008000u;
	HAL_MPU_ConfigRegion(&a);
	HAL_MPU_ConfigRegion(&b);
	MPU->RNR = 7u; // not 1 or 2: only a RNR write by EnableRegion can make RASR address region 2
	HAL_MPU_EnableRegion(2u);
	CHECK(MPU->RNR == 2u); // PM0214 4.2.7
	CHECK((mpuRasr(2) & 1u) == 1u);
	CHECK((mpuRasr(1) & 1u) == 0u);
	CHECK(mpuRasr(2) == ((0xEu << 1) | 1u)); // the other fields stay
}

TEST_CASE_FIXTURE(HalFixture, "C11 HAL_MPU_DisableRegion selects the region in RNR and clears ENABLE of that region only") {
	MPU_Region_InitTypeDef a = {};
	a.Enable = MPU_REGION_ENABLE;
	a.Number = MPU_REGION_NUMBER1;
	a.BaseAddress = 0x20000000u;
	a.Size = MPU_REGION_SIZE_32KB;
	MPU_Region_InitTypeDef b = a;
	b.Number = MPU_REGION_NUMBER2;
	b.BaseAddress = 0x20008000u;
	HAL_MPU_ConfigRegion(&a);
	HAL_MPU_ConfigRegion(&b);
	MPU->RNR = 7u; // not 1 or 2: only a RNR write by DisableRegion can make RASR address region 2
	HAL_MPU_DisableRegion(2u);
	CHECK(MPU->RNR == 2u);
	CHECK((mpuRasr(2) & 1u) == 0u);
	CHECK((mpuRasr(1) & 1u) == 1u);
	CHECK(mpuRasr(2) == (0xEu << 1)); // the other fields stay
}

// ---- C12 ----

TEST_CASE_FIXTURE(HalFixture, "C12 HAL_CORTEX_ClearEvent executes one SEV and one WFE") {
	// PM0214 2.5: SEV sets the event register, the following WFE clears it and returns at once.
	HAL_CORTEX_ClearEvent();
	CHECK(hostCmsisSevCount == 1u);
	CHECK(hostCmsisWfeCount == 1u);
}

namespace {
unsigned g_sevAtWfe = 99u;
void recordSevAtWfe() { g_sevAtWfe = hostCmsisSevCount; }
} // namespace

TEST_CASE_FIXTURE(HalFixture, "C12 HAL_CORTEX_ClearEvent executes the SEV before the WFE") {
	// The wait hook runs inside __WFE; a SEV that came first has already been counted then. (Without the SEV first
	// the event register would be clear and WFE would really sleep.)
	g_sevAtWfe = 99u;
	hostCmsisWaitHook = recordSevAtWfe;
	HAL_CORTEX_ClearEvent();
	CHECK(g_sevAtWfe == 1u);
}

// ---- C13 ----

TEST_CASE_FIXTURE(HalFixture, "C13 HAL_NVIC_SystemReset requests a system reset once and does not return") {
	CHECK(systemResetReturnsViaHook() == 1);
	CHECK(hostScbResetRequests() == 1u); // AIRCR written once with VECTKEY and SYSRESETREQ (PM0214 4.4.5)
}

TEST_CASE_FIXTURE(HalFixture, "C13 HAL_NVIC_SystemReset ends in the endless __NOP loop") {
	CHECK(systemResetReturnsViaHook() == 1);
	CHECK(hostCmsisNopCount >= 1u);
}

TEST_CASE_FIXTURE(HalFixture, "C13 characterization: HAL_NVIC_SystemReset keeps the PRIGROUP of AIRCR") {
	// NEEDS REVIEW: PM0214 4.4.5 does not say whether the reset request write must repeat PRIGROUP; the HAL does.
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	REQUIRE(systemResetReturnsViaHook() == 1);
	CHECK(((SCB->AIRCR >> 8) & 7u) == 3u);
}

} // TEST_SUITE
