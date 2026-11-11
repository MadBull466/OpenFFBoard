/*
 * P12 T2 PWR cases (requirements PW1-PW11 of Tests/Code_Coverage/claude/charters/hal/P12_T2_REQUIREMENTS.md).
 * Sources under test: stm32f4xx_hal_pwr.c and the F405/407 branch of stm32f4xx_hal_pwr_ex.c of the real ST HAL, built
 * into ffb_hal_tests and ffb_hal_tests_disco. Platform: PWR model of fakes/stm32_hal/HalClockPower.{h,c}, EXTI model
 * (line 16 = PVD), SCB / SysTick models of HalCorePeripherals.
 *
 * Oracles (expected values come from these, never from the HAL source):
 *  - RM0090 Rev 22 5.4.1 PWR_CR (reset 0x4000): LPDS 0, PDDS 1, CWUF 2 (write only), CSBF 3 (write only), PVDE 4,
 *    PLS[2:0] 7:5 (000 2.0 V, 001 2.1 V, 010 2.3 V, 011 2.5 V, 100 2.6 V, 101 2.7 V, 110 2.8 V, 111 2.9 V), DBP 8,
 *    FPDS 9, VOS 14 (0 = scale 2, 1 = scale 1).
 *  - RM0090 5.4.2 PWR_CSR: WUF 0, SBF 1, PVDO 2, BRR 3, EWUP 8, BRE 9 (needs DBP = 1), VOSRDY 14.
 *  - RM0090 5.3 low-power modes: Sleep (SLEEPDEEP = 0), Stop (SLEEPDEEP = 1, PDDS = 0, LPDS = regulator), Standby
 *    (SLEEPDEEP = 1, PDDS = 1). 12.2 / 12.3 EXTI: PVD is line 16, bit 16 of IMR / EMR / RTSR / FTSR / PR.
 *    7.3.16 RCC_APB1RSTR bit 28 PWRRST; 7.3.17 RCC_APB1ENR bit 28 PWREN.
 *  - PM0214 Rev 10 SCR: SLEEPONEXIT bit 1, SLEEPDEEP bit 2, SEVONPEND bit 4.
 *  - UM1725 Rev 8 PWR / PWREx chapters: API contract; "In Sleep mode, the systick is stopped" (SLEEP note), the
 *    regulator argument "is not used for the STM32F4 family" in HAL_PWR_EnterSLEEPMode.
 * HAL constants (PWR_PVD_MODE_*, PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_*, ...) are used as inputs only.
 *
 * Not covered: wake-up from Stop / Standby and the PWR clock gating (not modelled). Writing APB1RSTR bit 28 resets
 * PWR_CR to 0x4000 in the model (PW1 checks it).
 *
 * Proposed NEEDS REVIEW entries (numbers to be assigned by the lead) are written in the comment of each
 * characterization case.
 */
#include "HalFixture.h"

#include "HalClockPower.h"
#include "HalCorePeripherals.h"
#include "TestHelpers.h"

#include <cstdint>

namespace {

const uint32_t kPvdLine = 1u << 16; // EXTI line 16, RM0090 12.2.5
const uint32_t kCrDefault = 0x4000u;
const uint32_t kLpds = 1u << 0;
const uint32_t kPdds = 1u << 1;
const uint32_t kPvde = 1u << 4;
const uint32_t kDbp = 1u << 8;
const uint32_t kFpds = 1u << 9;
const uint32_t kVos = 1u << 14;
const uint32_t kCsrWuf = 1u << 0;
const uint32_t kCsrSbf = 1u << 1;
const uint32_t kCsrBrr = 1u << 3;
const uint32_t kCsrEwup = 1u << 8;
const uint32_t kCsrBre = 1u << 9;
const uint32_t kCsrVosrdy = 1u << 14;
const uint32_t kScrSleepOnExit = 1u << 1;
const uint32_t kScrSleepDeep = 1u << 2;
const uint32_t kScrSevOnPend = 1u << 4;

int g_pvdCallbacks = 0;

// What the CPU state is at the moment of each WFI / WFE.
struct WaitObs {
	int n = 0;
	uint32_t scr = 0;
	uint32_t cr = 0;
	uint32_t csr = 0;
	uint32_t stkCtrl = 0;
};
WaitObs g_obs;

void waitHook() {
	g_obs.n++;
	g_obs.scr = HalFixture::peek(&SCB->SCR);
	g_obs.cr = HalFixture::peek(&PWR->CR);
	g_obs.csr = PWR->CSR; // trapped read: the model evaluates its flags on a read
	g_obs.stkCtrl = HalFixture::peek(&SysTick->CTRL);
}

struct PwrFixture : HalFixture {
	PwrFixture() {
		g_pvdCallbacks = 0;
		g_obs = WaitObs();
		hostCmsisWaitHook = &waitHook;
	}
	~PwrFixture() { hostCmsisWaitHook = nullptr; }

	static uint32_t cr() { return peek(&PWR->CR); }
	static uint32_t csr() { return PWR->CSR; } // trapped read (flags are evaluated on a read)
	static uint32_t scr() { return peek(&SCB->SCR); }
	static uint32_t exti(const volatile uint32_t *reg) { return peek(reg) & kPvdLine; }
	static void configPvd(uint32_t level, uint32_t mode) {
		PWR_PVDTypeDef c = {};
		c.PVDLevel = level;
		c.Mode = mode;
		HAL_PWR_ConfigPVD(&c);
	}
};

const uint32_t kLevels[8] = {PWR_PVDLEVEL_0, PWR_PVDLEVEL_1, PWR_PVDLEVEL_2, PWR_PVDLEVEL_3,
                             PWR_PVDLEVEL_4, PWR_PVDLEVEL_5, PWR_PVDLEVEL_6, PWR_PVDLEVEL_7};
const uint32_t kPlsBits[8] = {0u << 5, 1u << 5, 2u << 5, 3u << 5, 4u << 5, 5u << 5, 6u << 5, 7u << 5};
const uint32_t kThresholdMv[8] = {2000, 2100, 2300, 2500, 2600, 2700, 2800, 2900};

} // namespace

// Strong definition of the HAL's weak callback (only in this file).
extern "C" void HAL_PWR_PVDCallback(void) { ++g_pvdCallbacks; }

TEST_SUITE("P12 T2 PWR") {

// ---- PW1 ----

TEST_CASE_FIXTURE(PwrFixture, "PW1 DeInit pulses the PWR reset bit 28 of APB1RSTR: set, then released") {
	HAL_PWR_DeInit();
	const uintptr_t a = reinterpret_cast<uintptr_t>(&RCC->APB1RSTR);
	int writes = 0;
	bool firstSet = false;
	uint32_t last = 0xFFFFFFFFu;
	for (size_t i = 0; i < hostRegsLogCount(); ++i) {
		const HostRegAccess *e = hostRegsLogAt(i);
		if (e->write && e->addr == a) {
			if (writes == 0) firstSet = (e->written & (1u << 28)) != 0;
			last = e->value;
			++writes;
		}
	}
	CHECK(writes >= 2);
	CHECK(firstSet);
	CHECK((last & (1u << 28)) == 0u);
	CHECK((peek(&RCC->APB1RSTR) & (1u << 28)) == 0u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW1 DeInit returns PWR_CR to its reset value 0x00004000") {
	poke(&PWR->CR, kDbp | kFpds | kPvde | (5u << 5) | kPdds | kLpds); // VOS = 0 as well
	HAL_PWR_DeInit();
	CHECK(cr() == 0x00004000u);
}

// ---- PW2 ----

TEST_CASE_FIXTURE(PwrFixture, "PW2 EnableBkUpAccess sets DBP and keeps the other PWR_CR bits") {
	poke(&PWR->CR, kCrDefault | (5u << 5) | kFpds);
	HAL_PWR_EnableBkUpAccess();
	CHECK(cr() == (kCrDefault | (5u << 5) | kFpds | kDbp));
}

TEST_CASE_FIXTURE(PwrFixture, "PW2 DisableBkUpAccess clears DBP and keeps the other PWR_CR bits") {
	poke(&PWR->CR, kCrDefault | (5u << 5) | kFpds | kDbp);
	HAL_PWR_DisableBkUpAccess();
	CHECK(cr() == (kCrDefault | (5u << 5) | kFpds));
}

TEST_CASE_FIXTURE(PwrFixture, "PW2 after reset the backup domain is write protected (DBP = 0, PWR_CR 0x4000)") {
	CHECK(cr() == kCrDefault);
}

// ---- PW3 ----

TEST_CASE_FIXTURE(PwrFixture, "PW3 ConfigPVD writes PLS[2:0] at bits 7:5 for all 8 levels") {
	for (int i = 0; i < 8; ++i) {
		poke(&PWR->CR, kCrDefault);
		configPvd(kLevels[i], PWR_PVD_MODE_NORMAL);
		INFO("level " << i);
		CHECK((cr() & (7u << 5)) == kPlsBits[i]);
	}
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 ConfigPVD replaces a previous level and keeps the other PWR_CR bits") {
	poke(&PWR->CR, kCrDefault | kDbp | kFpds | (7u << 5));
	configPvd(PWR_PVDLEVEL_2, PWR_PVD_MODE_NORMAL);
	CHECK(cr() == (kCrDefault | kDbp | kFpds | (2u << 5)));
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 ConfigPVD with the normal mode leaves line 16 out of all EXTI registers") {
	configPvd(PWR_PVDLEVEL_3, PWR_PVD_MODE_NORMAL);
	CHECK(exti(&EXTI->IMR) == 0u);
	CHECK(exti(&EXTI->EMR) == 0u);
	CHECK(exti(&EXTI->RTSR) == 0u);
	CHECK(exti(&EXTI->FTSR) == 0u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 ConfigPVD interrupt mode with rising edge sets IMR and RTSR of line 16 only") {
	configPvd(PWR_PVDLEVEL_3, PWR_PVD_MODE_IT_RISING);
	CHECK(exti(&EXTI->IMR) == kPvdLine);
	CHECK(exti(&EXTI->EMR) == 0u);
	CHECK(exti(&EXTI->RTSR) == kPvdLine);
	CHECK(exti(&EXTI->FTSR) == 0u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 ConfigPVD interrupt mode with falling edge sets IMR and FTSR of line 16 only") {
	configPvd(PWR_PVDLEVEL_3, PWR_PVD_MODE_IT_FALLING);
	CHECK(exti(&EXTI->IMR) == kPvdLine);
	CHECK(exti(&EXTI->EMR) == 0u);
	CHECK(exti(&EXTI->RTSR) == 0u);
	CHECK(exti(&EXTI->FTSR) == kPvdLine);
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 ConfigPVD interrupt mode with both edges sets IMR, RTSR and FTSR of line 16") {
	configPvd(PWR_PVDLEVEL_3, PWR_PVD_MODE_IT_RISING_FALLING);
	CHECK(exti(&EXTI->IMR) == kPvdLine);
	CHECK(exti(&EXTI->EMR) == 0u);
	CHECK(exti(&EXTI->RTSR) == kPvdLine);
	CHECK(exti(&EXTI->FTSR) == kPvdLine);
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 ConfigPVD event mode with rising edge sets EMR and RTSR of line 16 only") {
	configPvd(PWR_PVDLEVEL_3, PWR_PVD_MODE_EVENT_RISING);
	CHECK(exti(&EXTI->IMR) == 0u);
	CHECK(exti(&EXTI->EMR) == kPvdLine);
	CHECK(exti(&EXTI->RTSR) == kPvdLine);
	CHECK(exti(&EXTI->FTSR) == 0u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 ConfigPVD event mode with falling edge sets EMR and FTSR of line 16 only") {
	configPvd(PWR_PVDLEVEL_3, PWR_PVD_MODE_EVENT_FALLING);
	CHECK(exti(&EXTI->IMR) == 0u);
	CHECK(exti(&EXTI->EMR) == kPvdLine);
	CHECK(exti(&EXTI->RTSR) == 0u);
	CHECK(exti(&EXTI->FTSR) == kPvdLine);
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 ConfigPVD event mode with both edges sets EMR, RTSR and FTSR of line 16") {
	configPvd(PWR_PVDLEVEL_3, PWR_PVD_MODE_EVENT_RISING_FALLING);
	CHECK(exti(&EXTI->IMR) == 0u);
	CHECK(exti(&EXTI->EMR) == kPvdLine);
	CHECK(exti(&EXTI->RTSR) == kPvdLine);
	CHECK(exti(&EXTI->FTSR) == kPvdLine);
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 ConfigPVD leaves the other EXTI lines unchanged") {
	poke(&EXTI->IMR, 0x00000005u);
	poke(&EXTI->EMR, 0x00000A00u);
	poke(&EXTI->RTSR, 0x00001001u);
	poke(&EXTI->FTSR, 0x00400002u);
	configPvd(PWR_PVDLEVEL_1, PWR_PVD_MODE_IT_RISING_FALLING);
	CHECK(peek(&EXTI->IMR) == (0x00000005u | kPvdLine));
	CHECK(peek(&EXTI->EMR) == 0x00000A00u);
	CHECK(peek(&EXTI->RTSR) == (0x00001001u | kPvdLine));
	CHECK(peek(&EXTI->FTSR) == (0x00400002u | kPvdLine));
}

// characterization: ConfigPVD clears an earlier line 16 configuration before applying the new mode. UM1725 only says
// "configures the voltage threshold"; it does not say whether the old EXTI setup of the line survives.
// NEEDS REVIEW (proposed): ConfigPVD resets IMR / EMR / RTSR / FTSR bit 16 first, so IT rising followed by EVT falling
// leaves only EMR and FTSR set.
TEST_CASE_FIXTURE(PwrFixture, "PW3 characterization: a second ConfigPVD replaces the earlier EXTI configuration of line 16") {
	configPvd(PWR_PVDLEVEL_3, PWR_PVD_MODE_IT_RISING);
	configPvd(PWR_PVDLEVEL_3, PWR_PVD_MODE_EVENT_FALLING);
	CHECK(exti(&EXTI->IMR) == 0u);
	CHECK(exti(&EXTI->EMR) == kPvdLine);
	CHECK(exti(&EXTI->RTSR) == 0u);
	CHECK(exti(&EXTI->FTSR) == kPvdLine);
}

// characterization: reverse sequence of the case above (same NEEDS REVIEW entry): EVT rising+falling then IT rising
// leaves only IMR and RTSR set.
TEST_CASE_FIXTURE(PwrFixture, "PW3 characterization: ConfigPVD IT rising after EVT both edges clears EMR and FTSR of line 16") {
	configPvd(PWR_PVDLEVEL_3, PWR_PVD_MODE_EVENT_RISING_FALLING);
	configPvd(PWR_PVDLEVEL_3, PWR_PVD_MODE_IT_RISING);
	CHECK(exti(&EXTI->IMR) == kPvdLine);
	CHECK(exti(&EXTI->EMR) == 0u);
	CHECK(exti(&EXTI->RTSR) == kPvdLine);
	CHECK(exti(&EXTI->FTSR) == 0u);
}

// characterization: ConfigPVD does not enable the detector (PVDE stays 0 until HAL_PWR_EnablePVD). UM1725 describes
// ConfigPVD as "configures the threshold" and EnablePVD as the enable. NEEDS REVIEW (proposed): confirm that PVDE is not
// touched by ConfigPVD.
TEST_CASE_FIXTURE(PwrFixture, "PW3 characterization: ConfigPVD does not change PVDE") {
	configPvd(PWR_PVDLEVEL_4, PWR_PVD_MODE_IT_RISING);
	CHECK((cr() & kPvde) == 0u);
	poke(&PWR->CR, kCrDefault | kPvde);
	configPvd(PWR_PVDLEVEL_4, PWR_PVD_MODE_IT_RISING);
	CHECK((cr() & kPvde) == kPvde);
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 EnablePVD sets PVDE and keeps the other PWR_CR bits") {
	poke(&PWR->CR, kCrDefault | (3u << 5));
	HAL_PWR_EnablePVD();
	CHECK(cr() == (kCrDefault | (3u << 5) | kPvde));
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 DisablePVD clears PVDE and keeps the other PWR_CR bits") {
	poke(&PWR->CR, kCrDefault | (3u << 5) | kPvde);
	HAL_PWR_DisablePVD();
	CHECK(cr() == (kCrDefault | (3u << 5)));
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 PVDO reports VDD below the selected level for every PLS threshold") {
	// RM0090 5.4.2: PVDO = 1 when VDD is lower than the threshold of PLS, valid while PVDE = 1.
	for (int i = 0; i < 8; ++i) {
		INFO("level " << i);
		configPvd(kLevels[i], PWR_PVD_MODE_NORMAL);
		HAL_PWR_EnablePVD();
		hostPwrSetVddMv(kThresholdMv[i] - 100u);
		CHECK(__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO));
		hostPwrSetVddMv(kThresholdMv[i] + 100u);
		CHECK_FALSE(__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO));
		HAL_PWR_DisablePVD();
	}
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 PVDO is 0 while the detector is disabled even with VDD below the level") {
	configPvd(PWR_PVDLEVEL_7, PWR_PVD_MODE_NORMAL);
	hostPwrSetVddMv(1800u);
	CHECK_FALSE(__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO));
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 interrupt mode with rising edge: a rising level on EXTI line 16 sets pending bit 16") {
	// Line 16 is the internal PVD output (RM0090 12.2.5); the EXTI fake is driven directly (no VDD change).
	configPvd(PWR_PVDLEVEL_5, PWR_PVD_MODE_IT_RISING);
	HAL_PWR_EnablePVD();
	hostExtiSetLine(16, 0);
	CHECK(exti(&EXTI->PR) == 0u);
	hostExtiSetLine(16, 1);
	CHECK(exti(&EXTI->PR) == kPvdLine);
}

TEST_CASE_FIXTURE(PwrFixture, "PW3 event mode: a rising edge on line 16 counts an event and leaves PR clear") {
	configPvd(PWR_PVDLEVEL_5, PWR_PVD_MODE_EVENT_RISING);
	const uint32_t before = hostExtiEventCount();
	hostExtiSetLine(16, 0);
	hostExtiSetLine(16, 1);
	CHECK(hostExtiEventCount() == before + 1u);
	CHECK(exti(&EXTI->PR) == 0u);
}

// ---- PW4 ----

TEST_CASE_FIXTURE(PwrFixture, "PW4 PVD_IRQHandler with the pending bit set calls the callback once") {
	poke(&EXTI->PR, kPvdLine);
	HAL_PWR_PVD_IRQHandler();
	CHECK(g_pvdCallbacks == 1);
}

TEST_CASE_FIXTURE(PwrFixture, "PW4 PVD_IRQHandler with the pending bit set clears bit 16 by writing 1 to it") {
	poke(&EXTI->PR, kPvdLine);
	HAL_PWR_PVD_IRQHandler();
	CHECK(exti(&EXTI->PR) == 0u);
	CHECK(lastWritten(&EXTI->PR) == kPvdLine);
}

TEST_CASE_FIXTURE(PwrFixture, "PW4 PVD_IRQHandler leaves other pending lines untouched") {
	poke(&EXTI->PR, kPvdLine | 0x8u);
	HAL_PWR_PVD_IRQHandler();
	CHECK(peek(&EXTI->PR) == 0x8u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW4 PVD_IRQHandler without the pending bit calls no callback") {
	poke(&EXTI->PR, 0x8u);
	HAL_PWR_PVD_IRQHandler();
	CHECK(g_pvdCallbacks == 0);
}

TEST_CASE_FIXTURE(PwrFixture, "PW4 PVD_IRQHandler without the pending bit writes nothing and leaves PR unchanged") {
	poke(&EXTI->PR, 0x8u);
	HAL_PWR_PVD_IRQHandler();
	CHECK(hostRegsWritesTo(reinterpret_cast<uintptr_t>(&EXTI->PR)) == 0u);
	CHECK(peek(&EXTI->PR) == 0x8u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW4 a PVD edge in interrupt mode reaches the callback through the handler") {
	configPvd(PWR_PVDLEVEL_5, PWR_PVD_MODE_IT_RISING_FALLING);
	hostExtiSetLine(16, 1);
	HAL_PWR_PVD_IRQHandler();
	CHECK(g_pvdCallbacks == 1);
	HAL_PWR_PVD_IRQHandler();
	CHECK(g_pvdCallbacks == 1); // flag was cleared, a second call finds nothing
}

// ---- PW5 ----

TEST_CASE_FIXTURE(PwrFixture, "PW5 EnableWakeUpPin for pin 1 sets EWUP (bit 8)") {
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
	CHECK((csr() & kCsrEwup) == kCsrEwup);
}

TEST_CASE_FIXTURE(PwrFixture, "PW5 EnableWakeUpPin keeps the other PWR_CSR bits") {
	poke(&PWR->CR, kCrDefault | kDbp);
	hostPwrSetFlags(1, 1);
	poke(&PWR->CSR, peek(&PWR->CSR) | kCsrBre);
	const uint32_t before = csr();
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
	CHECK((csr() & ~kCsrEwup) == (before & ~kCsrEwup));
	CHECK((csr() & (kCsrSbf | kCsrWuf | kCsrBre)) == (kCsrSbf | kCsrWuf | kCsrBre));
}

TEST_CASE_FIXTURE(PwrFixture, "PW5 DisableWakeUpPin for pin 1 clears EWUP") {
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	CHECK((csr() & kCsrEwup) == 0u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW5 DisableWakeUpPin keeps the other PWR_CSR bits") {
	poke(&PWR->CR, kCrDefault | kDbp);
	hostPwrSetFlags(1, 1);
	poke(&PWR->CSR, peek(&PWR->CSR) | kCsrBre);
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	CHECK((csr() & (kCsrSbf | kCsrWuf | kCsrBre)) == (kCsrSbf | kCsrWuf | kCsrBre));
	CHECK((csr() & kCsrEwup) == 0u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW5 GET_FLAG and CLEAR_FLAG of WU and SB act through CWUF and CSBF, which read as 0") {
	// RM0090 5.4.1: CWUF (bit 2) and CSBF (bit 3) are write-only, read 0; 5.4.2: WUF bit 0, SBF bit 1.
	const uint32_t base = kCrDefault | kDbp | kFpds | (3u << 5);
	poke(&PWR->CR, base);
	hostPwrSetFlags(1, 1);
	CHECK(__HAL_PWR_GET_FLAG(PWR_FLAG_WU));
	CHECK(__HAL_PWR_GET_FLAG(PWR_FLAG_SB));
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	CHECK_FALSE(__HAL_PWR_GET_FLAG(PWR_FLAG_WU));
	CHECK(__HAL_PWR_GET_FLAG(PWR_FLAG_SB));
	CHECK((lastWritten(&PWR->CR) & (1u << 2)) == (1u << 2));
	CHECK(cr() == base);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	CHECK_FALSE(__HAL_PWR_GET_FLAG(PWR_FLAG_SB));
	CHECK((lastWritten(&PWR->CR) & (1u << 3)) == (1u << 3));
	CHECK(cr() == base);
}

// ---- PW6 ----

TEST_CASE_FIXTURE(PwrFixture, "PW6 EnterSLEEPMode with WFI executes one WFI and no WFE") {
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	CHECK(hostCmsisWfiCount == 1u);
	CHECK(hostCmsisWfeCount == 0u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW6 EnterSLEEPMode clears SLEEPDEEP before the WFI") {
	poke(&SCB->SCR, kScrSleepDeep);
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	REQUIRE(g_obs.n == 1);
	CHECK((g_obs.scr & kScrSleepDeep) == 0u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW6 EnterSLEEPMode with WFE executes WFE and no WFI") {
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
	CHECK(hostCmsisWfiCount == 0u);
	CHECK(hostCmsisWfeCount >= 1u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW6 EnterSLEEPMode with WFE clears SLEEPDEEP before the WFE") {
	poke(&SCB->SCR, kScrSleepDeep);
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
	REQUIRE(g_obs.n >= 1);
	CHECK((g_obs.scr & kScrSleepDeep) == 0u);
}

// characterization: the number of WFE instructions of a WFE entry. UM1725 says only "enter SLEEP mode with WFE
// instruction". RM0090 5.3.4 / PM0214: a stale event register makes the first WFE return at once. NEEDS REVIEW
// (proposed): WFE entry executes SEV then WFE twice (clearing a stale event), and the NO_EVT_CLEAR variant executes one
// WFE and no SEV; the exact counts are the HAL's behaviour (2 / 1).
TEST_CASE_FIXTURE(PwrFixture, "PW6 characterization: WFE entry clears a stale event first, NO_EVT_CLEAR does not") {
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
	CHECK(hostCmsisSevCount == 1u);
	CHECK(hostCmsisWfeCount == 2u);
	hostCmsisReset();
	hostCmsisWaitHook = &waitHook;
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE_NO_EVT_CLEAR);
	CHECK(hostCmsisSevCount == 0u);
	CHECK(hostCmsisWfeCount == 1u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW6 EnterSLEEPMode with the main regulator leaves PWR_CR unchanged") {
	poke(&PWR->CR, kCrDefault | kDbp | (2u << 5));
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	CHECK(cr() == (kCrDefault | kDbp | (2u << 5)));
	CHECK(g_obs.cr == (kCrDefault | kDbp | (2u << 5)));
}

TEST_CASE_FIXTURE(PwrFixture, "PW6 EnterSLEEPMode with the low-power regulator leaves PWR_CR unchanged (argument unused)") {
	poke(&PWR->CR, kCrDefault | kDbp | (2u << 5));
	HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	CHECK(cr() == (kCrDefault | kDbp | (2u << 5)));
	CHECK(g_obs.cr == (kCrDefault | kDbp | (2u << 5)));
}

TEST_CASE_FIXTURE(PwrFixture, "PW6 EnterSLEEPMode stops the SysTick interrupt during sleep" * KNOWN_ISSUE("candidate vendor bug V91: UM1725 says the SysTick is stopped in Sleep mode (HAL_PWR_EnterSLEEPMode note) but the HAL leaves SysTick TICKINT set while waiting")) {
	// TICKINT = SysTick CTRL bit 1 (PM0214 4.5.1).
	poke(&SysTick->CTRL, 0x7u);
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	REQUIRE(g_obs.n == 1);
	CHECK((g_obs.stkCtrl & 0x3u) != 0x3u); // SysTick stopped: ENABLE or TICKINT cleared
}

TEST_CASE_FIXTURE(PwrFixture, "PW6 EnterSLEEPMode leaves the SysTick interrupt enabled after the wake-up") {
	poke(&SysTick->CTRL, 0x7u);
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	CHECK((peek(&SysTick->CTRL) & 0x7u) == 0x7u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW6 EnterSLEEPMode leaves SLEEPONEXIT and SEVONPEND as they were") {
	poke(&SCB->SCR, kScrSleepOnExit | kScrSevOnPend | kScrSleepDeep);
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	CHECK(scr() == (kScrSleepOnExit | kScrSevOnPend));
}

// ---- PW7 STOP ----

TEST_CASE_FIXTURE(PwrFixture, "PW7 EnterSTOPMode with WFI executes one WFI and no WFE") {
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
	CHECK(hostCmsisWfiCount == 1u);
	CHECK(hostCmsisWfeCount == 0u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW7 EnterSTOPMode sets SLEEPDEEP while waiting") {
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
	REQUIRE(g_obs.n == 1);
	CHECK((g_obs.scr & kScrSleepDeep) == kScrSleepDeep);
}

TEST_CASE_FIXTURE(PwrFixture, "PW7 EnterSTOPMode clears SLEEPDEEP again after the wait") {
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
	CHECK((scr() & kScrSleepDeep) == 0u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW7 EnterSTOPMode clears PDDS (Stop, not Standby) while waiting") {
	poke(&PWR->CR, kCrDefault | kPdds);
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
	REQUIRE(g_obs.n == 1);
	CHECK((g_obs.cr & kPdds) == 0u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW7 EnterSTOPMode with the main regulator has LPDS = 0 while waiting") {
	poke(&PWR->CR, kCrDefault | kLpds);
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
	REQUIRE(g_obs.n == 1);
	CHECK((g_obs.cr & kLpds) == 0u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW7 EnterSTOPMode with the low-power regulator has LPDS = 1 while waiting") {
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	REQUIRE(g_obs.n == 1);
	CHECK((g_obs.cr & kLpds) == kLpds);
}

TEST_CASE_FIXTURE(PwrFixture, "PW7 EnterSTOPMode with WFE executes WFE and no WFI") {
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE);
	CHECK(hostCmsisWfiCount == 0u);
	CHECK(hostCmsisWfeCount >= 1u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW7 EnterSTOPMode with WFE has SLEEPDEEP set and the low-power regulator while waiting") {
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);
	REQUIRE(g_obs.n >= 1);
	CHECK((g_obs.scr & kScrSleepDeep) == kScrSleepDeep);
	CHECK((g_obs.cr & (kLpds | kPdds)) == kLpds);
	CHECK((scr() & kScrSleepDeep) == 0u);
}

// characterization: WFE counts of the Stop entry (see PW6 WFE case). NEEDS REVIEW (proposed): STOPENTRY_WFE runs SEV and
// two WFE (stale event cleared), STOPENTRY_WFE_NO_EVT_CLEAR one WFE and no SEV.
TEST_CASE_FIXTURE(PwrFixture, "PW7 characterization: Stop with WFE clears a stale event first, NO_EVT_CLEAR does not") {
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE);
	CHECK(hostCmsisSevCount == 1u);
	CHECK(hostCmsisWfeCount == 2u);
	hostCmsisReset();
	hostCmsisWaitHook = &waitHook;
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE_NO_EVT_CLEAR);
	CHECK(hostCmsisSevCount == 0u);
	CHECK(hostCmsisWfeCount == 1u);
}

// ---- PW7 STANDBY ----

TEST_CASE_FIXTURE(PwrFixture, "PW7 EnterSTANDBYMode executes one WFI and no WFE") {
	HAL_PWR_EnterSTANDBYMode();
	CHECK(hostCmsisWfiCount == 1u);
	CHECK(hostCmsisWfeCount == 0u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW7 EnterSTANDBYMode sets PDDS while waiting") {
	HAL_PWR_EnterSTANDBYMode();
	REQUIRE(g_obs.n == 1);
	CHECK((g_obs.cr & kPdds) == kPdds);
}

TEST_CASE_FIXTURE(PwrFixture, "PW7 EnterSTANDBYMode sets SLEEPDEEP while waiting") {
	HAL_PWR_EnterSTANDBYMode();
	REQUIRE(g_obs.n == 1);
	CHECK((g_obs.scr & kScrSleepDeep) == kScrSleepDeep);
}

// characterization: RM0090 Table 31 (5.3.6, Standby mode entry) lists among the entry conditions "WUF bit is cleared in
// Power Control register" (CWUF), and UM1725 HAL_PWR_EnterSTANDBYMode only has the I/O pin note. The HAL does not clear
// WUF or SBF. NEEDS REVIEW (proposed): the application must clear WUF itself (e.g. __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU))
// before calling EnterSTANDBYMode; confirm that this is the intended contract and not a HAL omission.
TEST_CASE_FIXTURE(PwrFixture, "PW7 characterization: EnterSTANDBYMode leaves WUF and SBF as they were") {
	hostPwrSetFlags(1, 1);
	HAL_PWR_EnterSTANDBYMode();
	REQUIRE(g_obs.n == 1);
	CHECK((g_obs.csr & kCsrWuf) == kCsrWuf);
	CHECK((g_obs.csr & kCsrSbf) == kCsrSbf);
}

// ---- PW8 ----

TEST_CASE_FIXTURE(PwrFixture, "PW8 EnableSleepOnExit sets SCR bit 1 only") {
	HAL_PWR_EnableSleepOnExit();
	CHECK(scr() == kScrSleepOnExit);
}

TEST_CASE_FIXTURE(PwrFixture, "PW8 DisableSleepOnExit clears SCR bit 1 and keeps the others") {
	poke(&SCB->SCR, kScrSleepOnExit | kScrSevOnPend | kScrSleepDeep);
	HAL_PWR_DisableSleepOnExit();
	CHECK(scr() == (kScrSevOnPend | kScrSleepDeep));
}

TEST_CASE_FIXTURE(PwrFixture, "PW8 EnableSEVOnPend sets SCR bit 4 only") {
	HAL_PWR_EnableSEVOnPend();
	CHECK(scr() == kScrSevOnPend);
}

TEST_CASE_FIXTURE(PwrFixture, "PW8 DisableSEVOnPend clears SCR bit 4 and keeps the others") {
	poke(&SCB->SCR, kScrSleepOnExit | kScrSevOnPend | kScrSleepDeep);
	HAL_PWR_DisableSEVOnPend();
	CHECK(scr() == (kScrSleepOnExit | kScrSleepDeep));
}

TEST_CASE_FIXTURE(PwrFixture, "PW8 EnableSleepOnExit and EnableSEVOnPend keep SLEEPDEEP") {
	poke(&SCB->SCR, kScrSleepDeep);
	HAL_PWR_EnableSleepOnExit();
	HAL_PWR_EnableSEVOnPend();
	CHECK(scr() == (kScrSleepDeep | kScrSleepOnExit | kScrSevOnPend));
}

// ---- PW9 ----

TEST_CASE_FIXTURE(PwrFixture, "PW9 EnableBkUpReg with DBP = 1 sets BRE, waits for BRR and returns HAL_OK") {
	HAL_PWR_EnableBkUpAccess();
	CHECK(HAL_PWREx_EnableBkUpReg() == HAL_OK);
	CHECK((csr() & kCsrBre) == kCsrBre);
	CHECK((csr() & kCsrBrr) == kCsrBrr);
}

TEST_CASE_FIXTURE(PwrFixture, "PW9 EnableBkUpReg waits until BRR rises after its start-up time") {
	hostPwrSetStartupTicks(1, 5);
	HAL_PWR_EnableBkUpAccess();
	CHECK(HAL_PWREx_EnableBkUpReg() == HAL_OK);
	CHECK(hostTickPeek() >= 5u);
	CHECK((csr() & kCsrBrr) == kCsrBrr);
}

TEST_CASE_FIXTURE(PwrFixture, "PW9 EnableBkUpReg with DBP = 0 leaves BRE clear (RM0090: BRE needs DBP)") {
	HAL_PWREx_EnableBkUpReg(); // return value: see the timeout case
	CHECK((csr() & kCsrBre) == 0u);
}

// characterization: return code and wait of a backup regulator that never gets ready. UM1725 does not describe the
// timeout. NEEDS REVIEW (proposed): EnableBkUpReg returns HAL_TIMEOUT after the HAL's own timeout (1000 ms tick count)
// and leaves BRE set; the same HAL_TIMEOUT is returned when DBP = 0 and BRE could therefore not be written.
TEST_CASE_FIXTURE(PwrFixture, "PW9 characterization: EnableBkUpReg returns HAL_TIMEOUT when BRR never rises") {
	hostPwrFailBackupRegulator(1);
	HAL_PWR_EnableBkUpAccess();
	CHECK(HAL_PWREx_EnableBkUpReg() == HAL_TIMEOUT);
	CHECK(hostTickPeek() >= 1000u);
	CHECK(hostTickPeek() < 1100u);
	CHECK((csr() & kCsrBre) == kCsrBre);
}

TEST_CASE_FIXTURE(PwrFixture, "PW9 characterization: EnableBkUpReg with DBP = 0 returns HAL_TIMEOUT (BRE not written)") {
	CHECK(HAL_PWREx_EnableBkUpReg() == HAL_TIMEOUT);
}

TEST_CASE_FIXTURE(PwrFixture, "PW9 DisableBkUpReg clears BRE, waits for BRR to clear and returns HAL_OK") {
	HAL_PWR_EnableBkUpAccess();
	REQUIRE(HAL_PWREx_EnableBkUpReg() == HAL_OK);
	CHECK(HAL_PWREx_DisableBkUpReg() == HAL_OK);
	CHECK((csr() & kCsrBre) == 0u);
	CHECK((csr() & kCsrBrr) == 0u);
}

// characterization: DisableBkUpReg with a BRR that stays set. UM1725 gives no return code. NEEDS REVIEW (proposed):
// DisableBkUpReg returns HAL_TIMEOUT after the HAL's own timeout (>= 1000 ticks) and BRE is cleared anyway.
TEST_CASE_FIXTURE(PwrFixture, "PW9 characterization: DisableBkUpReg returns HAL_TIMEOUT when BRR stays set") {
	HAL_PWR_EnableBkUpAccess();
	REQUIRE(HAL_PWREx_EnableBkUpReg() == HAL_OK);
	hostPwrStickyBackupReady(1);
	const uint32_t t0 = hostTickPeek();
	CHECK(HAL_PWREx_DisableBkUpReg() == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 >= 1000u);
	CHECK(hostTickPeek() - t0 < 1100u);
	CHECK((csr() & kCsrBre) == 0u);
}

// ---- PW10 ----

TEST_CASE_FIXTURE(PwrFixture, "PW10 EnableFlashPowerDown sets FPDS (bit 9) and keeps the other PWR_CR bits") {
	poke(&PWR->CR, kCrDefault | kDbp | (4u << 5));
	HAL_PWREx_EnableFlashPowerDown();
	CHECK(cr() == (kCrDefault | kDbp | (4u << 5) | kFpds));
}

TEST_CASE_FIXTURE(PwrFixture, "PW10 DisableFlashPowerDown clears FPDS and keeps the other PWR_CR bits") {
	poke(&PWR->CR, kCrDefault | kDbp | (4u << 5) | kFpds);
	HAL_PWREx_DisableFlashPowerDown();
	CHECK(cr() == (kCrDefault | kDbp | (4u << 5)));
}

TEST_CASE_FIXTURE(PwrFixture, "PW10 GetVoltageRange returns scale 1 after reset (VOS = 1)") {
	CHECK(HAL_PWREx_GetVoltageRange() == PWR_REGULATOR_VOLTAGE_SCALE1);
	CHECK(HAL_PWREx_GetVoltageRange() == kVos);
}

TEST_CASE_FIXTURE(PwrFixture, "PW10 GetVoltageRange returns scale 2 when VOS = 0") {
	poke(&PWR->CR, 0u);
	CHECK(HAL_PWREx_GetVoltageRange() == PWR_REGULATOR_VOLTAGE_SCALE2);
	CHECK(HAL_PWREx_GetVoltageRange() == 0u);
}

TEST_CASE_FIXTURE(PwrFixture, "PW10 GetVoltageRange reports VOS only, not the other PWR_CR bits") {
	poke(&PWR->CR, kDbp | kFpds | (7u << 5) | kPvde);
	CHECK(HAL_PWREx_GetVoltageRange() == 0u);
	poke(&PWR->CR, kVos | kDbp | kFpds | (7u << 5) | kPvde);
	CHECK(HAL_PWREx_GetVoltageRange() == kVos);
}

// ---- PW11 ----

TEST_CASE_FIXTURE(PwrFixture, "PW11 ControlVoltageScaling to scale 2 clears VOS, waits for VOSRDY and returns HAL_OK") {
	CHECK(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) == HAL_OK);
	CHECK((cr() & kVos) == 0u);
	CHECK((csr() & kCsrVosrdy) == kCsrVosrdy);
}

TEST_CASE_FIXTURE(PwrFixture, "PW11 ControlVoltageScaling back to scale 1 sets VOS and returns HAL_OK") {
	REQUIRE(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) == HAL_OK);
	CHECK(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) == HAL_OK);
	CHECK((cr() & kVos) == kVos);
	CHECK(HAL_PWREx_GetVoltageRange() == PWR_REGULATOR_VOLTAGE_SCALE1);
}

TEST_CASE_FIXTURE(PwrFixture, "PW11 ControlVoltageScaling keeps the other PWR_CR bits") {
	poke(&PWR->CR, kCrDefault | kDbp | (3u << 5) | kFpds);
	REQUIRE(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) == HAL_OK);
	CHECK(cr() == (kDbp | (3u << 5) | kFpds));
}

TEST_CASE_FIXTURE(PwrFixture, "PW11 ControlVoltageScaling waits until VOSRDY rises after its start-up time") {
	hostPwrSetStartupTicks(6, 1);
	CHECK(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) == HAL_OK);
	CHECK(hostTickPeek() >= 6u);
}

// characterization: timeout of a regulator that never gets ready. UM1725 gives no return code. NEEDS REVIEW (proposed):
// ControlVoltageScaling returns HAL_TIMEOUT when VOSRDY never rises, after the HAL's own timeout (>= 1000 ticks as
// assumed here is the HAL's 1 s PWR_VOSRDY_TIMEOUT_VALUE); VOS keeps the requested value.
TEST_CASE_FIXTURE(PwrFixture, "PW11 characterization: ControlVoltageScaling returns HAL_TIMEOUT when VOSRDY never rises") {
	hostPwrFailVos(1);
	CHECK(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) == HAL_TIMEOUT);
	CHECK(hostTickPeek() >= 1000u);
	CHECK(hostTickPeek() < 1100u);
	CHECK((cr() & kVos) == 0u);
}

// characterization: the HAL enables the PWR peripheral clock (RCC_APB1ENR bit 28, RM0090 7.3.17) itself. UM1725 does not
// say so for ControlVoltageScaling. NEEDS REVIEW (proposed): confirm that PWREN is set as a side effect.
TEST_CASE_FIXTURE(PwrFixture, "PW11 characterization: ControlVoltageScaling enables the PWR clock (APB1ENR bit 28)") {
	REQUIRE((peek(&RCC->APB1ENR) & (1u << 28)) == 0u);
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
	CHECK((peek(&RCC->APB1ENR) & (1u << 28)) == (1u << 28));
}

} // TEST_SUITE
