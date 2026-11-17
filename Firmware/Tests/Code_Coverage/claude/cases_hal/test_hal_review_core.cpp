// P12 review witnesses (Codex review of P12, 2026-11-17): cases that pin down model corrections and test gaps of the core, clock, power, DMA and
// MPU models. Every case states its source. Each was written to FAIL against the model or tests as they were before the review fix.
#include "HalFixture.h"
#include "HalClockPower.h"
#include "HalDma.h"

namespace {
unsigned delayCalls, delayElapsedMs;
// A tick source that is phase sensitive: the first call after the HAL read the start tick does not move time (the 100 Hz tick has not elapsed
// yet), the second moves 1 ms, every later call 10 ms (a 10 ms tick period).
void phaseStep() {
	++delayCalls;
	if (delayCalls == 1) return;
	delayElapsedMs += delayCalls == 2 ? 1u : 10u;
	HAL_IncTick();
}
}  // namespace

TEST_CASE_FIXTURE(HalFixture, "RV1 HAL_Delay of 7 ms with a 10 ms tick lets at least 7 ms of simulated time pass even when the first tick arrives early (P12C-T1-001)") {
	// UM1725 HAL_Delay: "provides a minimum delay (in milliseconds)". With a 10 ms tick the HAL adds one tick period (wait = 7 + 10 ticks of the
	// 10 ms tick = 17 ms of requested wait); the elapsed time is counted independently here, not by the test double's own step count.
	uwTickFreq = HAL_TICK_FREQ_100HZ;
	delayCalls = 0;
	delayElapsedMs = 0;
	hostTickSetStep(phaseStep);
	HAL_Delay(7u);
	INFO("elapsed simulated milliseconds " << delayElapsedMs << ", tick calls " << delayCalls);
	CHECK(delayElapsedMs >= 7u);
}

TEST_CASE_FIXTURE(HalFixture, "RV2 enabling and disabling the backup domain access each read PWR_CR back (P12C-T2-001)") {
	// UM1725 / HAL source comment: after the DBP write the HAL reads the bit back to wait for the protection to take effect (RM0090 5.1.? DBP).
	// The model applies the change at once, so only the read in the register log shows the HAL waited.
	for (int enable = 0; enable < 2; ++enable) {
		hostRegsLogClear();
		if (enable) HAL_PWR_EnableBkUpAccess(); else HAL_PWR_DisableBkUpAccess();
		unsigned reads = 0;
		for (size_t i = 0; i < hostRegsLogCount(); ++i) {
			const HostRegAccess *e = hostRegsLogAt(i);
			if (!e->write && e->addr == reinterpret_cast<uintptr_t>(&PWR->CR)) ++reads;
		}
		INFO("enable = " << enable);
		CHECK(reads >= 1u);
	}
}

TEST_CASE_FIXTURE(HalFixture, "RV3 MPU_RBAR reads back the base address with the bits below the region size forced to zero (P12C-T1-002)") {
	// PM0214 MPU_RBAR: ADDR[31:N], N = Log2(region size); bits N-1:5 reserved, forced by hardware to 0. A 64 KB region based at 0x20012340 reads
	// back 0x20010000 (bits 15:5 cleared) with the region number in bits 3:0.
	MPU_Region_InitTypeDef r = {};
	r.Enable = MPU_REGION_ENABLE;
	r.Number = MPU_REGION_NUMBER3;
	r.BaseAddress = 0x20012340u;
	r.Size = MPU_REGION_SIZE_64KB;
	HAL_MPU_ConfigRegion(&r);
	MPU->RNR = 3;
	CHECK(MPU->RBAR == (0x20010000u | 3u));
}

TEST_CASE_FIXTURE(HalFixture, "RV4 RCC_PLLCFGR: PLLN, PLLP and PLLQ stay writable while only the PLLI2S runs, PLLM and PLLSRC do not (P12C-T2-002)") {
	// RM0090 7.3.2: PLLN, PLLP, PLLQ "can be written only if PLL is disabled"; PLLM and PLLSRC "only when PLL and PLLI2S are disabled".
	const uint32_t original = 8u | (192u << 6) | (4u << 24);
	RCC->PLLCFGR = original;
	RCC->CR |= (1u << 26);  // PLLI2SON
	for (int i = 0; i < 3; ++i) HAL_GetTick();
	REQUIRE((RCC->CR & (1u << 26)) != 0u);
	REQUIRE((RCC->CR & (1u << 24)) == 0u);
	const uint32_t desired = 4u | (168u << 6) | (1u << 16) | (1u << 22) | (7u << 24);
	RCC->PLLCFGR = desired;
	const uint32_t actual = RCC->PLLCFGR;
	CHECK((actual & ((1u << 22) | 63u)) == (original & ((1u << 22) | 63u)));  // PLLSRC, PLLM unchanged
	CHECK(((actual >> 6) & 511u) == 168u);   // PLLN
	CHECK(((actual >> 16) & 3u) == 1u);      // PLLP
	CHECK(((actual >> 24) & 15u) == 7u);     // PLLQ
}

TEST_CASE_FIXTURE(HalFixture, "RV5 a clock security failure of the HSE that feeds the system clock through the PLL also stops the PLLI2S (P12C-T2-003)") {
	// RM0090 6.2.7: "If the HSE oscillator clock was the clock source of PLL used as the system clock when the failure occurred, PLL is also
	// disabled. In this case, if the PLLI2S was enabled, it is also disabled when the HSE fails."
	RCC_OscInitTypeDef osc = {};
	osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc.HSEState = RCC_HSE_ON;
	osc.PLL.PLLState = RCC_PLL_ON;
	osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	osc.PLL.PLLM = 8;
	osc.PLL.PLLN = 192;
	osc.PLL.PLLP = RCC_PLLP_DIV4;
	osc.PLL.PLLQ = 4;
	REQUIRE(HAL_RCC_OscConfig(&osc) == HAL_OK);
	uwTickPrio = 15;
	RCC_ClkInitTypeDef clk = {};
	clk.ClockType = RCC_CLOCKTYPE_SYSCLK;
	clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	REQUIRE(HAL_RCC_ClockConfig(&clk, 0) == HAL_OK);
	RCC->CR |= (1u << 26);  // PLLI2SON (the shared M / source fields are locked now: the PLLI2S runs with them)
	for (int i = 0; i < 3; ++i) HAL_GetTick();
	REQUIRE((RCC->CR & (1u << 26)) != 0u);
	HAL_RCC_EnableCSS();
	hostRccHseFailure();
	CHECK((RCC->CR & (1u << 24)) == 0u);  // PLLON
	CHECK((RCC->CR & (1u << 26)) == 0u);  // PLLI2SON
	CHECK((RCC->CR & (1u << 27)) == 0u);  // PLLI2SRDY
}

TEST_CASE_FIXTURE(HalFixture, "RV6 DMA: a peripheral burst in FIFO mode forces PINCOS low when the stream is enabled, a single beat keeps it (P12C-T3-002)") {
	// RM0090 10.5.5 DMA_SxCR PINCOS: "forced low by hardware when the stream is enabled (EN = 1) if the direct mode is selected or if PBURST are
	// different from 00".
	DMA2_Stream3->FCR = (1u << 2) | 3u;  // FIFO mode (DMDIS), full threshold
	DMA2_Stream3->NDTR = 8;
	DMA2_Stream3->CR = (1u << 15) | (1u << 9) | (1u << 21);  // PINCOS, PINC, PBURST = INCR4
	REQUIRE((peek(&DMA2_Stream3->CR) & (1u << 15)) != 0);
	DMA2_Stream3->CR |= 1u;
	CHECK((peek(&DMA2_Stream3->CR) & (1u << 15)) == 0);
}

TEST_CASE_FIXTURE(HalFixture, "RV6 DMA: PINCOS survives the enable in FIFO mode with a single peripheral beat (control, P12C-T3-002)") {
	DMA2_Stream3->FCR = (1u << 2) | 3u;
	DMA2_Stream3->NDTR = 8;
	DMA2_Stream3->CR = (1u << 15) | (1u << 9);
	DMA2_Stream3->CR |= 1u;
	CHECK((peek(&DMA2_Stream3->CR) & (1u << 15)) != 0);
}
