/*
 * P12 T2 group RC (clock configuration part): HAL_RCC_ClockConfig, the frequency getters, GetOscConfig /
 * GetClockConfig, MCOConfig, the clock security system and the NMI handler of stm32f4xx_hal_rcc.c, against the RCC / PWR
 * behavioural model (fakes/stm32_hal/HalClockPower.h). Requirements RC10-RC15 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T2_REQUIREMENTS.md; the file is compiled into ffb_hal_tests (F407VG) and
 * ffb_hal_tests_disco (F407VG_DISCO).
 *
 * Oracles: UM1725 Rev 8 (54.2.5 RCC, 3.11.1 clocks), RM0090 Rev 22: 7.2 clocks, 7.3.1 RCC_CR (HSEON 16, CSSON 19),
 * 7.3.2 RCC_PLLCFGR, 7.3.3 RCC_CFGR, 7.3.4 RCC_CIR (CSSF 7, CSSC 23), 3.9.1 FLASH_ACR (LATENCY[2:0] bits 2:0), 3.4 wait
 * states. Constants of the target (stm32f4xx_hal_conf.h): HSE_VALUE 8 MHz, HSI_VALUE 16 MHz.
 *
 * Register encodings used in the derivations (RM0090 7.3.2 / 7.3.3):
 *  - RCC_CFGR: SW 1:0 (00 HSI, 01 HSE, 10 PLL), SWS 3:2 (same codes), HPRE 7:4, PPRE1 12:10, PPRE2 15:13,
 *    MCO1 22:21, MCO1PRE 26:24, MCO2PRE 29:27, MCO2 31:30.
 *  - HPRE: 0xxx /1, 1000 /2, 1001 /4, 1010 /8, 1011 /16, 1100 /64, 1101 /128, 1110 /256, 1111 /512.
 *  - PPRE: 0xx /1, 100 /2, 101 /4, 110 /8, 111 /16.
 *  - RCC_PLLCFGR: PLLM 5:0, PLLN 14:6, PLLP 17:16 (00 /2, 01 /4, 10 /6, 11 /8), PLLSRC 22 (0 HSI, 1 HSE), PLLQ 27:24.
 *  - PLL: f_VCO = f_in x N / M, f_PLLCLK = f_VCO / P (7.3.2). Computed by the helper pllClock() below.
 *  - MCO1: 00 HSI, 01 LSE, 10 HSE, 11 PLL; MCO2: 00 SYSCLK, 01 PLLI2S, 10 HSE, 11 PLL; MCOxPRE: 0xx /1, 100 /2, 101 /3,
 *    110 /4, 111 /5.
 * The HAL API enumerations (RCC_SYSCLKSOURCE_*, RCC_CLOCKTYPE_*, RCC_HSE_ON, ...) are used as inputs only; the
 * expected register values are written out here.
 *
 * Not testable with the model (reported to the lead): a FLASH_ACR latency that does not read back for a legal value
 * (the model stores the written value); only a value outside LATENCY[2:0] (8) fails to read back.
 */
#include "HalClockPower.h"
#include "HalFixture.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstdio>

extern "C" {
extern uint32_t SystemCoreClock;
}

namespace {

int g_cssCalls = 0;

constexpr uint32_t kHse = 8000000u;  // HSE_VALUE of the target
constexpr uint32_t kHsi = 16000000u; // HSI_VALUE of the target

constexpr uint32_t kHpreCode[9] = {0x0, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE, 0xF};
constexpr uint32_t kHpreDiv[9] = {1, 2, 4, 8, 16, 64, 128, 256, 512};
constexpr uint32_t kPpreCode[5] = {0x0, 0x4, 0x5, 0x6, 0x7};
constexpr uint32_t kPpreDiv[5] = {1, 2, 4, 8, 16};

/* RM0090 7.3.2: f_PLLCLK = f_in x N / (M x P). Integer division once at the end equals the two nested divisions. */
uint32_t pllClock(uint32_t in, uint32_t m, uint32_t n, uint32_t p) {
	return static_cast<uint32_t>((static_cast<uint64_t>(in) * n) / (static_cast<uint64_t>(m) * p));
}

uint32_t cfgr() { return HalFixture::peek(&RCC->CFGR); }

/* HSE on (waits for HSERDY) and the PLL of the firmware: HSE 8 MHz, M 4, N 168, P 2, Q 7. */
bool startFirmwarePll() {
	RCC_OscInitTypeDef osc = {};
	osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc.HSEState = RCC_HSE_ON;
	osc.PLL.PLLState = RCC_PLL_ON;
	osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	osc.PLL.PLLM = 4;
	osc.PLL.PLLN = 168;
	osc.PLL.PLLP = 2; // RCC_PLLP_DIV2
	osc.PLL.PLLQ = 7;
	return HAL_RCC_OscConfig(&osc) == HAL_OK;
}

bool startHse() {
	RCC_OscInitTypeDef osc = {};
	osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc.HSEState = RCC_HSE_ON;
	osc.PLL.PLLState = RCC_PLL_NONE;
	return HAL_RCC_OscConfig(&osc) == HAL_OK;
}

/* apb2 is passed in the PPRE1 position (UM1725: RCC_HCLK_DIVx); the HAL shifts it to bits 15:13. */
HAL_StatusTypeDef clockConfig(uint32_t type, uint32_t src, uint32_t hpreCode, uint32_t ppre1Code, uint32_t ppre2Code,
							  uint32_t latency) {
	uwTickPrio = 15u; // HAL_InitTick rejects the start value 16
	RCC_ClkInitTypeDef clk = {};
	clk.ClockType = type;
	clk.SYSCLKSource = src;
	clk.AHBCLKDivider = hpreCode << 4;
	clk.APB1CLKDivider = ppre1Code << 10;
	clk.APB2CLKDivider = ppre2Code << 10;
	return HAL_RCC_ClockConfig(&clk, latency);
}

constexpr uint32_t kAllTypes = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

/* Index of the first logged write to addr (with the predicate), or -1. */
template <class Pred> long firstWrite(uintptr_t addr, Pred pred) {
	for (std::size_t i = 0; i < hostRegsLogCount(); ++i) {
		const HostRegAccess *e = hostRegsLogAt(i);
		if (e->write && e->addr == addr && pred(e->written)) return static_cast<long>(i);
	}
	return -1;
}

uintptr_t addrOf(const volatile void *reg) { return reinterpret_cast<uintptr_t>(reg); }

static bool swWritten(uint32_t target) {
	return firstWrite(addrOf(&RCC->CFGR), [target](uint32_t w) { return (w & 3u) == target; }) >= 0;
}

/* Selects the PLL with raw register writes (no HAL), for the formula sweeps: PLLCFGR fields, SW = SWS = PLL. */
void forcePll(uint32_t m, uint32_t n, uint32_t pDiv, bool hse) {
	const uint32_t pCode = pDiv / 2u - 1u; // RM0090 7.3.2: 00 /2, 01 /4, 10 /6, 11 /8
	HalFixture::poke(&RCC->PLLCFGR, m | (n << 6) | (pCode << 16) | ((hse ? 1u : 0u) << 22) | (7u << 24));
	HalFixture::poke(&RCC->CFGR, (2u << 2) | 2u); // SWS = SW = PLL
}

struct Mismatch {
	unsigned bad = 0;
	char first[96] = {0};
};

void note(Mismatch &mm, const char *what, uint32_t a, uint32_t b, uint32_t c, uint32_t got, uint32_t want) {
	if (mm.bad++ == 0u) {
		snprintf(mm.first, sizeof mm.first, "%s %u %u %u: got %u want %u", what, a, b, c, got, want);
	}
}

} // namespace

extern "C" void HAL_RCC_CSSCallback(void) { ++g_cssCalls; }

struct RccClockFixture : HalFixture {
	RccClockFixture() { g_cssCalls = 0; }
};

TEST_SUITE("P12 T2 RCC clock configuration") {

// ================================================================ RC10 ClockConfig
TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig with the firmware configuration returns HAL_OK and selects the PLL") {
	REQUIRE(startFirmwarePll());
	CHECK(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5) == HAL_OK);
	CHECK(((cfgr() >> 2) & 3u) == 2u); // SWS = 10: PLL used as system clock (7.3.3)
	CHECK((cfgr() & 3u) == 2u);        // SW = 10
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig writes HPRE PPRE1 and PPRE2 exactly") {
	REQUIRE(startFirmwarePll());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5) == HAL_OK);
	CHECK(((cfgr() >> 4) & 0xFu) == 0x0u); // HPRE 0xxx = /1
	CHECK(((cfgr() >> 10) & 7u) == 0x5u);  // PPRE1 101 = /4 (42 MHz at 168 MHz)
	CHECK(((cfgr() >> 13) & 7u) == 0x4u);  // PPRE2 100 = /2 (84 MHz)
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig writes the flash latency to LATENCY[2:0]") {
	REQUIRE(startFirmwarePll());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5) == HAL_OK);
	CHECK((peek(&FLASH->ACR) & 7u) == 5u); // 3.9.1: LATENCY = 101, five wait states at 168 MHz
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig keeps the other bits of FLASH_ACR") {
	poke(&FLASH->ACR, 0x700u); // PRFTEN 8, ICEN 9, DCEN 10
	REQUIRE(startFirmwarePll());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5) == HAL_OK);
	CHECK(peek(&FLASH->ACR) == 0x705u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 a rising clock gets its latency before the switch") {
	// RM0090 3.4: when the frequency increases the new latency must be effective first.
	REQUIRE(startFirmwarePll());
	hostRegsLogClear();
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5) == HAL_OK);
	const long acr = firstWrite(addrOf(&FLASH->ACR), [](uint32_t) { return true; });
	const long sw = firstWrite(addrOf(&RCC->CFGR), [](uint32_t w) { return (w & 3u) == 2u; });
	REQUIRE(acr >= 0);
	REQUIRE(sw >= 0);
	CHECK(acr < sw);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 a falling clock gets its latency after the switch") {
	// RM0090 3.4: when the frequency decreases the latency may only be reduced after the clock has changed.
	REQUIRE(startFirmwarePll());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5) == HAL_OK);
	hostRegsLogClear();
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSI, 0x0, 0x0, 0x0, 0) == HAL_OK);
	const long sw = firstWrite(addrOf(&RCC->CFGR), [](uint32_t w) { return (w & 3u) == 0u; });
	const long acr = firstWrite(addrOf(&FLASH->ACR), [](uint32_t) { return true; });
	REQUIRE(sw >= 0);
	REQUIRE(acr >= 0);
	CHECK(sw < acr);
	CHECK((peek(&FLASH->ACR) & 7u) == 0u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig sets SystemCoreClock to the new HCLK") {
	REQUIRE(startFirmwarePll());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5) == HAL_OK);
	CHECK(SystemCoreClock == 168000000u); // HSE 8 MHz / 4 x 168 / 2 (7.3.2)
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig divides SystemCoreClock by HPRE") {
	REQUIRE(startFirmwarePll());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x9, 0x0, 0x0, 5) == HAL_OK); // HPRE 1001 = /4
	CHECK(SystemCoreClock == 42000000u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig re-initialises the tick for the new HCLK") {
	REQUIRE(startFirmwarePll());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5) == HAL_OK);
	CHECK(SysTick->LOAD == 167999u);  // 1 ms at 168 MHz: 168000 cycles, RELOAD = N - 1 (PM0214 4.5.2)
	CHECK(HAL_GetTickPrio() == 15u);  // the priority the caller put in uwTickPrio
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig of the system clock alone leaves the prescalers") {
	REQUIRE(startFirmwarePll());
	poke(&RCC->CFGR, (0xBu << 4) | (6u << 10) | (7u << 13)); // HPRE /16, PPRE1 /8, PPRE2 /16, SW = SWS = HSI
	REQUIRE(clockConfig(RCC_CLOCKTYPE_SYSCLK, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x0, 0x0, 5) == HAL_OK);
	CHECK(((cfgr() >> 4) & 0xFu) == 0xBu);
	CHECK(((cfgr() >> 10) & 7u) == 6u);
	CHECK(((cfgr() >> 13) & 7u) == 7u);
	CHECK(((cfgr() >> 2) & 3u) == 2u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig without the SYSCLK type never writes SW") {
	REQUIRE(startFirmwarePll());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSE, 0x0, 0x0, 0x0, 0) == HAL_OK); // SW = SWS = HSE
	hostRegsLogClear();
	REQUIRE(clockConfig(RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2, RCC_SYSCLKSOURCE_PLLCLK, 0x8, 0x5, 0x4, 5) == HAL_OK);
	CHECK((cfgr() & 3u) == 1u);        // SW unchanged: HSE
	CHECK(((cfgr() >> 2) & 3u) == 1u);
	CHECK_FALSE(swWritten(2u));        // no write selected the PLL
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig of PCLK1 alone changes only PPRE1") {
	poke(&RCC->CFGR, (0xBu << 4) | (6u << 10) | (7u << 13));
	REQUIRE(clockConfig(RCC_CLOCKTYPE_PCLK1, RCC_SYSCLKSOURCE_HSI, 0x0, 0x5, 0x0, 0) == HAL_OK);
	CHECK(((cfgr() >> 4) & 0xFu) == 0xBu);
	CHECK(((cfgr() >> 10) & 7u) == 5u);
	CHECK(((cfgr() >> 13) & 7u) == 7u);
	CHECK((cfgr() & 3u) == 0u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig of PCLK2 alone changes only PPRE2") {
	poke(&RCC->CFGR, (0xBu << 4) | (6u << 10) | (7u << 13));
	REQUIRE(clockConfig(RCC_CLOCKTYPE_PCLK2, RCC_SYSCLKSOURCE_HSI, 0x0, 0x0, 0x4, 0) == HAL_OK);
	CHECK(((cfgr() >> 4) & 0xFu) == 0xBu);
	CHECK(((cfgr() >> 10) & 7u) == 6u);
	CHECK(((cfgr() >> 13) & 7u) == 4u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig of HCLK alone changes only HPRE") {
	poke(&RCC->CFGR, (0xBu << 4) | (6u << 10) | (5u << 13));
	REQUIRE(clockConfig(RCC_CLOCKTYPE_HCLK, RCC_SYSCLKSOURCE_HSI, 0x8, 0x0, 0x0, 0) == HAL_OK);
	CHECK(((cfgr() >> 4) & 0xFu) == 0x8u);
	CHECK(((cfgr() >> 10) & 7u) == 6u);
	CHECK(((cfgr() >> 13) & 7u) == 5u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig to HSE selects the HSE and gives 8 MHz") {
	REQUIRE(startHse());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSE, 0x0, 0x0, 0x0, 0) == HAL_OK);
	CHECK(((cfgr() >> 2) & 3u) == 1u); // SWS = 01 HSE
	CHECK(SystemCoreClock == kHse);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig to the HSE works after a slow HSE start-up") {
	// The HSE becomes ready 50 ticks after HSEON (HSE_STARTUP_TIMEOUT is 100 ms); OscConfig waits for HSERDY.
	hostRccSetStartupTicks(HOST_OSC_HSE, 50);
	REQUIRE(startHse());
	CHECK(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSE, 0x0, 0x0, 0x0, 0) == HAL_OK);
	CHECK(((cfgr() >> 2) & 3u) == 1u);
}

static void stepThenFailHse(void) {
	HAL_IncTick();
	hostRccFail(HOST_OSC_HSE, 1); // the HSE dies right after the HAL has checked HSERDY and written SW
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 characterization: ClockConfig returns HAL_TIMEOUT when SWS never follows SW") {
	// NEEDS REVIEW: UM1725 gives no return code for the poll of SWS ("Return values: None"); the HAL gives up after
	// 5000 ticks (CLOCKSWITCH_TIMEOUT_VALUE, 5 s) and returns HAL_TIMEOUT, leaving SW at the requested source and
	// SystemCoreClock unchanged.
	REQUIRE(startHse());
	hostTickSetStep(stepThenFailHse);
	const uint32_t t0 = hostTickPeek();
	CHECK(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSE, 0x0, 0x0, 0x0, 0) == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 > 5000u);  // CLOCKSWITCH_TIMEOUT_VALUE 5000 ticks
	CHECK(hostTickPeek() - t0 <= 5002u); // and it gives up then, not later
	CHECK(((cfgr() >> 2) & 3u) == 0u); // SWS still HSI
	CHECK(SystemCoreClock == kHsi);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig with a NULL structure returns HAL_ERROR and accesses no register") {
	hostRegsLogClear();
	CHECK(HAL_RCC_ClockConfig(nullptr, 0) == HAL_ERROR);
	CHECK(hostRegsLogCount() == 0u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 ClockConfig with a latency that cannot be read back returns an error") {
	// RM0090 3.9.1: LATENCY is bits 2:0 on the F405/407, 8 does not fit; the new value is effective only once the
	// LATENCY bits read back the written value (3.4), so the HAL must refuse to go on. The return code HAL_ERROR is
	// what the HAL status of UM1725 offers.
	REQUIRE(startFirmwarePll());
	CHECK(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 8) == HAL_ERROR);
	CHECK(((cfgr() >> 2) & 3u) == 0u); // the clock was not switched
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 characterization: ClockConfig returns HAL_ERROR when a legal latency does not read back") {
	// NEEDS REVIEW: UM1725 gives no return code; the HAL status offered is HAL_ERROR (the code returns it when the
	// LATENCY bits do not read back the written value).
	REQUIRE(startFirmwarePll());
	hostFlashLatencyStuck(1);
	CHECK(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5) == HAL_ERROR);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 the clock is not switched when a legal latency does not read back") {
	// RM0090 3.4: the new latency must be effective before the frequency rises.
	REQUIRE(startFirmwarePll());
	hostFlashLatencyStuck(1);
	clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5);
	CHECK((cfgr() & 3u) == 0u);        // SW still HSI
	CHECK(((cfgr() >> 2) & 3u) == 0u); // SWS still HSI
	CHECK(SystemCoreClock == kHsi);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC10 characterization: ClockConfig returns HAL_OK although the tick could not be re-initialised") {
	// NEEDS REVIEW: with uwTickPrio at its start value 16 (out of the 4 priority bits, PM0214 4.3.7) HAL_InitTick
	// returns HAL_ERROR; ClockConfig ignores it and returns HAL_OK. UM1725 says nothing about this case.
	REQUIRE(uwTickPrio == 16u);
	RCC_ClkInitTypeDef clk = {};
	clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK;
	clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	clk.AHBCLKDivider = 0x8u << 4;
	CHECK(HAL_RCC_ClockConfig(&clk, 0) == HAL_OK);
	CHECK(SystemCoreClock == kHsi / 2u);
}

// ================================================================ RC11 frequencies
TEST_CASE_FIXTURE(RccClockFixture, "RC11 GetSysClockFreq after reset is the HSI value") {
	CHECK(HAL_RCC_GetSysClockFreq() == kHsi); // SWS = 00 (RM0090 7.3.3 reset value), UM1725: HSI_VALUE
}

TEST_CASE_FIXTURE(RccClockFixture, "RC11 GetSysClockFreq with the HSE as system clock is the HSE value") {
	poke(&RCC->CFGR, (1u << 2) | 1u); // SWS = SW = HSE
	CHECK(HAL_RCC_GetSysClockFreq() == kHse);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC11 characterization: GetSysClockFreq with SWS = 11 returns the HSI value") {
	// NEEDS REVIEW: RM0090 7.3.3 lists SWS = 11 as "not applicable"; the HAL returns HSI_VALUE for it.
	poke(&RCC->CFGR, 0xFu); // SWS = 11, SW = 11 (raw, the model ignores such a write from the CPU)
	CHECK(HAL_RCC_GetSysClockFreq() == kHsi);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC11 GetSysClockFreq of the firmware PLL is 168 MHz") {
	REQUIRE(startFirmwarePll());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5) == HAL_OK);
	CHECK(HAL_RCC_GetSysClockFreq() == 168000000u); // VCO = 8 MHz / 4 x 168 = 336 MHz, / 2
}

TEST_CASE_FIXTURE(RccClockFixture, "RC11 GetSysClockFreq follows the PLL formula over the whole range of M for both sources") {
	Mismatch mm;
	for (uint32_t src = 0; src < 2; ++src)
		for (uint32_t m = 2; m <= 63; ++m)
			for (uint32_t n : {50u, 168u, 432u})
				for (uint32_t p : {2u, 4u, 6u, 8u}) {
					forcePll(m, n, p, src != 0u);
					const uint32_t want = pllClock(src != 0u ? kHse : kHsi, m, n, p);
					const uint32_t got = HAL_RCC_GetSysClockFreq();
					if (got != want) note(mm, "M N P", m, n, p, got, want);
				}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC11 GetSysClockFreq follows the PLL formula over the whole range of N for both sources") {
	// Pure formula check: M = 2 / N = 432 also give VCO values outside the RM0090 limits.
	Mismatch mm;
	for (uint32_t src = 0; src < 2; ++src)
		for (uint32_t n = 50; n <= 432; ++n)
			for (uint32_t m : {2u, 4u, 25u, 63u})
				for (uint32_t p : {2u, 4u, 6u, 8u}) {
					forcePll(m, n, p, src != 0u);
					const uint32_t want = pllClock(src != 0u ? kHse : kHsi, m, n, p);
					const uint32_t got = HAL_RCC_GetSysClockFreq();
					if (got != want) note(mm, "M N P", m, n, p, got, want);
				}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC11 GetSysClockFreq follows the PLL formula for all four P with the HSE and the HSI") {
	Mismatch mm;
	for (uint32_t src = 0; src < 2; ++src)
		for (uint32_t p : {2u, 4u, 6u, 8u}) {
			forcePll(4, 168, p, src != 0u);
			const uint32_t want = pllClock(src != 0u ? kHse : kHsi, 4, 168, p);
			const uint32_t got = HAL_RCC_GetSysClockFreq();
			if (got != want) note(mm, "src P", src, p, 0, got, want);
		}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC11 characterization: GetSysClockFreq of a PLL whose VCO is not a multiple of P rounds down") {
	// NEEDS REVIEW: rounding of fractional hertz is a HAL choice, not specified by UM1725 / RM0090.
	// HSE 8 MHz, M 3, N 100, P 8: 8e6 x 100 / 3 = 266 666 666.67 Hz, / 8 = 33 333 333.33 Hz: a fraction of a hertz is lost.
	forcePll(3, 100, 8, true);
	CHECK(HAL_RCC_GetSysClockFreq() == 33333333u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC11 GetHCLKFreq is SYSCLK divided by every HPRE code") {
	REQUIRE(startFirmwarePll());
	Mismatch mm;
	for (unsigned i = 0; i < 9; ++i) {
		const HAL_StatusTypeDef st = clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, kHpreCode[i], 0x0, 0x0, 5);
		const uint32_t want = 168000000u / kHpreDiv[i];
		const uint32_t got = HAL_RCC_GetHCLKFreq();
		if (st != HAL_OK) note(mm, "status", i, 0, 0, st, HAL_OK);
		if (got != want) note(mm, "HPRE", kHpreCode[i], 0, 0, got, want);
	}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC11 GetHCLKFreq of the HSI is the HSI value divided by every HPRE code") {
	Mismatch mm;
	for (unsigned i = 0; i < 9; ++i) {
		const HAL_StatusTypeDef st = clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSI, kHpreCode[i], 0x0, 0x0, 0);
		const uint32_t want = kHsi / kHpreDiv[i];
		const uint32_t got = HAL_RCC_GetHCLKFreq();
		if (st != HAL_OK) note(mm, "status", i, 0, 0, st, HAL_OK);
		if (got != want) note(mm, "HPRE", kHpreCode[i], 0, 0, got, want);
	}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC11 GetPCLK1Freq is HCLK divided by every PPRE1 code for every HPRE code") {
	Mismatch mm;
	for (unsigned h = 0; h < 9; ++h)
		for (unsigned p = 0; p < 5; ++p) {
			REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSI, kHpreCode[h], kPpreCode[p], 0x0, 0) == HAL_OK);
			const uint32_t want = kHsi / kHpreDiv[h] / kPpreDiv[p];
			const uint32_t got = HAL_RCC_GetPCLK1Freq();
			if (got != want) note(mm, "HPRE PPRE1", kHpreCode[h], kPpreCode[p], 0, got, want);
		}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC11 GetPCLK2Freq is HCLK divided by every PPRE2 code for every HPRE code") {
	Mismatch mm;
	for (unsigned h = 0; h < 9; ++h)
		for (unsigned p = 0; p < 5; ++p) {
			REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSI, kHpreCode[h], 0x0, kPpreCode[p], 0) == HAL_OK);
			const uint32_t want = kHsi / kHpreDiv[h] / kPpreDiv[p];
			const uint32_t got = HAL_RCC_GetPCLK2Freq();
			if (got != want) note(mm, "HPRE PPRE2", kHpreCode[h], kPpreCode[p], 0, got, want);
		}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC11 the firmware configuration gives PCLK1 42 MHz and PCLK2 84 MHz") {
	REQUIRE(startFirmwarePll());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5) == HAL_OK);
	CHECK(HAL_RCC_GetHCLKFreq() == 168000000u);
	CHECK(HAL_RCC_GetPCLK1Freq() == 42000000u); // 168 MHz / 4
	CHECK(HAL_RCC_GetPCLK2Freq() == 84000000u); // 168 MHz / 2
}

TEST_CASE_FIXTURE(RccClockFixture, "RC11 after reset HCLK and both PCLK are the HSI value") {
	CHECK(HAL_RCC_GetHCLKFreq() == kHsi);
	CHECK(HAL_RCC_GetPCLK1Freq() == kHsi); // PPRE reset 000 = /1
	CHECK(HAL_RCC_GetPCLK2Freq() == kHsi);
}

// ================================================================ RC12 Get*Config
TEST_CASE_FIXTURE(RccClockFixture, "RC12 GetClockConfig returns the configuration ClockConfig wrote") {
	REQUIRE(startFirmwarePll());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5) == HAL_OK);
	RCC_ClkInitTypeDef clk = {};
	uint32_t lat = 99;
	HAL_RCC_GetClockConfig(&clk, &lat);
	CHECK(clk.ClockType == kAllTypes);
	CHECK(clk.SYSCLKSource == 2u);          // SW = 10 PLL
	CHECK(clk.AHBCLKDivider == 0x0u);       // HPRE /1, in the register position (bits 7:4)
	CHECK(clk.APB1CLKDivider == 0x1400u);   // PPRE1 101 at bits 12:10
	CHECK(clk.APB2CLKDivider == 0x1000u);   // PPRE2 100 (bits 15:13) reported at the PPRE1 position: 100 << 10
	CHECK(lat == 5u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC12 GetClockConfig returns every HPRE and PPRE code for registers set directly") {
	Mismatch mm;
	for (unsigned h = 0; h < 9; ++h)
		for (unsigned p = 0; p < 5; ++p) {
			poke(&RCC->CFGR, (kHpreCode[h] << 4) | (kPpreCode[p] << 10) | (kPpreCode[4 - p] << 13));
			RCC_ClkInitTypeDef clk = {};
			uint32_t lat = 0;
			HAL_RCC_GetClockConfig(&clk, &lat);
			if (clk.AHBCLKDivider != (kHpreCode[h] << 4)) note(mm, "HPRE", h, p, 0, clk.AHBCLKDivider, kHpreCode[h] << 4);
			if (clk.APB1CLKDivider != (kPpreCode[p] << 10)) note(mm, "PPRE1", h, p, 0, clk.APB1CLKDivider, kPpreCode[p] << 10);
			if (clk.APB2CLKDivider != (kPpreCode[4 - p] << 10)) note(mm, "PPRE2", h, p, 0, clk.APB2CLKDivider, kPpreCode[4 - p] << 10);
		}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC12 GetClockConfig returns the system clock selected in SW") {
	Mismatch mm;
	for (uint32_t sw : {0u, 1u, 2u}) {
		poke(&RCC->CFGR, sw);
		RCC_ClkInitTypeDef clk = {};
		uint32_t lat = 0;
		HAL_RCC_GetClockConfig(&clk, &lat);
		if (clk.SYSCLKSource != sw) note(mm, "SW", sw, 0, 0, clk.SYSCLKSource, sw);
	}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC12 GetClockConfig returns the flash latency of FLASH_ACR for every value") {
	Mismatch mm;
	for (uint32_t l = 0; l <= 7; ++l) {
		poke(&FLASH->ACR, 0x700u | l);
		RCC_ClkInitTypeDef clk = {};
		uint32_t lat = 99;
		HAL_RCC_GetClockConfig(&clk, &lat);
		if (lat != l) note(mm, "latency", l, 0, 0, lat, l);
	}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC12 GetOscConfig after reset reports HSI on and the rest off") {
	RCC_OscInitTypeDef osc = {};
	HAL_RCC_GetOscConfig(&osc);
	CHECK(osc.OscillatorType == (RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_LSI));
	CHECK(osc.HSEState == RCC_HSE_OFF);
	CHECK(osc.HSIState == RCC_HSI_ON);
	CHECK(osc.LSEState == RCC_LSE_OFF);
	CHECK(osc.LSIState == RCC_LSI_OFF);
	CHECK(osc.PLL.PLLState == RCC_PLL_OFF);
	CHECK(osc.HSICalibrationValue == 0x10u); // HSITRIM reset value 10000 (RM0090 7.3.1, CR reset 0x83)
}

TEST_CASE_FIXTURE(RccClockFixture, "RC12 GetOscConfig after reset reports the PLLCFGR reset fields") {
	RCC_OscInitTypeDef osc = {};
	HAL_RCC_GetOscConfig(&osc);
	// RCC_PLLCFGR reset 0x24003010 (RM0090 7.3.2): PLLM 16, PLLN 192, PLLP 00 = /2, PLLSRC 0 = HSI, PLLQ 4.
	CHECK(osc.PLL.PLLM == 16u);
	CHECK(osc.PLL.PLLN == 192u);
	CHECK(osc.PLL.PLLP == 2u);
	CHECK(osc.PLL.PLLQ == 4u);
	CHECK(osc.PLL.PLLSource == RCC_PLLSOURCE_HSI);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC12 GetOscConfig returns the HSE and PLL that OscConfig started") {
	REQUIRE(startFirmwarePll());
	RCC_OscInitTypeDef osc = {};
	HAL_RCC_GetOscConfig(&osc);
	CHECK(osc.HSEState == RCC_HSE_ON);
	CHECK(osc.PLL.PLLState == RCC_PLL_ON);
	CHECK(osc.PLL.PLLSource == RCC_PLLSOURCE_HSE);
	CHECK(osc.PLL.PLLM == 4u);
	CHECK(osc.PLL.PLLN == 168u);
	CHECK(osc.PLL.PLLP == 2u);
	CHECK(osc.PLL.PLLQ == 7u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC12 GetOscConfig returns the PLL fields for every PLLP and the HSI source") {
	Mismatch mm;
	for (uint32_t p : {2u, 4u, 6u, 8u}) {
		RCC_OscInitTypeDef set = {};
		set.OscillatorType = RCC_OSCILLATORTYPE_HSI;
		set.HSIState = RCC_HSI_ON;
		set.HSICalibrationValue = 0x10;
		set.PLL.PLLState = RCC_PLL_ON;
		set.PLL.PLLSource = RCC_PLLSOURCE_HSI;
		set.PLL.PLLM = 63;
		set.PLL.PLLN = 432;
		set.PLL.PLLP = p;
		set.PLL.PLLQ = 15;
		const HAL_StatusTypeDef st = HAL_RCC_OscConfig(&set);
		RCC_OscInitTypeDef got = {};
		HAL_RCC_GetOscConfig(&got);
		if (st != HAL_OK) note(mm, "status", p, 0, 0, st, HAL_OK);
		if (got.PLL.PLLP != p) note(mm, "P", p, 0, 0, got.PLL.PLLP, p);
		if (got.PLL.PLLM != 63u) note(mm, "M", p, 0, 0, got.PLL.PLLM, 63);
		if (got.PLL.PLLN != 432u) note(mm, "N", p, 0, 0, got.PLL.PLLN, 432);
		if (got.PLL.PLLQ != 15u) note(mm, "Q", p, 0, 0, got.PLL.PLLQ, 15);
		if (got.PLL.PLLSource != RCC_PLLSOURCE_HSI) note(mm, "src", p, 0, 0, got.PLL.PLLSource, RCC_PLLSOURCE_HSI);
		// the PLL must be switched off again so that the next iteration may write PLLCFGR (7.3.2)
		RCC_OscInitTypeDef off = {};
		off.OscillatorType = RCC_OSCILLATORTYPE_NONE;
		off.PLL.PLLState = RCC_PLL_OFF;
		if (HAL_RCC_OscConfig(&off) != HAL_OK) note(mm, "pll off", p, 0, 0, 0, 0);
	}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC12 GetOscConfig reports the HSE bypass") {
	RCC_OscInitTypeDef set = {};
	set.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	set.HSEState = RCC_HSE_BYPASS;
	set.PLL.PLLState = RCC_PLL_NONE;
	REQUIRE(HAL_RCC_OscConfig(&set) == HAL_OK);
	RCC_OscInitTypeDef got = {};
	HAL_RCC_GetOscConfig(&got);
	CHECK(got.HSEState == RCC_HSE_BYPASS);
	CHECK((peek(&RCC->CR) & (1u << 18)) != 0u); // HSEBYP bit 18 (7.3.1)
}

TEST_CASE_FIXTURE(RccClockFixture, "RC12 GetOscConfig reports the HSE off after it was switched off") {
	REQUIRE(startHse());
	RCC_OscInitTypeDef set = {};
	set.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	set.HSEState = RCC_HSE_OFF;
	set.PLL.PLLState = RCC_PLL_NONE;
	REQUIRE(HAL_RCC_OscConfig(&set) == HAL_OK);
	RCC_OscInitTypeDef got = {};
	HAL_RCC_GetOscConfig(&got);
	CHECK(got.HSEState == RCC_HSE_OFF);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC12 GetOscConfig reports the LSI on after OscConfig switched it on") {
	RCC_OscInitTypeDef set = {};
	set.OscillatorType = RCC_OSCILLATORTYPE_LSI;
	set.LSIState = RCC_LSI_ON;
	set.PLL.PLLState = RCC_PLL_NONE;
	REQUIRE(HAL_RCC_OscConfig(&set) == HAL_OK);
	RCC_OscInitTypeDef got = {};
	HAL_RCC_GetOscConfig(&got);
	CHECK(got.LSIState == RCC_LSI_ON);
	CHECK((peek(&RCC->CSR) & 1u) != 0u); // LSION bit 0 (7.3.21)
}

TEST_CASE_FIXTURE(RccClockFixture, "RC12 GetOscConfig reports the LSE on after OscConfig switched it on") {
	RCC_OscInitTypeDef set = {};
	set.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	set.LSEState = RCC_LSE_ON;
	set.PLL.PLLState = RCC_PLL_NONE;
	REQUIRE(HAL_RCC_OscConfig(&set) == HAL_OK);
	RCC_OscInitTypeDef got = {};
	HAL_RCC_GetOscConfig(&got);
	CHECK(got.LSEState == RCC_LSE_ON);
	CHECK((peek(&RCC->BDCR) & 1u) != 0u); // LSEON bit 0 (7.3.20)
}

TEST_CASE_FIXTURE(RccClockFixture, "RC12 GetOscConfig reports the LSE bypass after OscConfig set it") {
	RCC_OscInitTypeDef set = {};
	set.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	set.LSEState = RCC_LSE_BYPASS;
	set.PLL.PLLState = RCC_PLL_NONE;
	REQUIRE(HAL_RCC_OscConfig(&set) == HAL_OK);
	RCC_OscInitTypeDef got = {};
	HAL_RCC_GetOscConfig(&got);
	CHECK(got.LSEState == RCC_LSE_BYPASS);
	CHECK((peek(&RCC->BDCR) & 4u) != 0u); // LSEBYP bit 2
}

TEST_CASE_FIXTURE(RccClockFixture, "RC12 GetOscConfig returns the HSI trimming of RCC_CR for every value") {
	Mismatch mm;
	for (uint32_t trim = 0; trim <= 0x1F; ++trim) {
		poke(&RCC->CR, 0x3u | (trim << 3)); // HSION, HSIRDY, HSITRIM 7:3
		RCC_OscInitTypeDef got = {};
		HAL_RCC_GetOscConfig(&got);
		if (got.HSICalibrationValue != trim) note(mm, "trim", trim, 0, 0, got.HSICalibrationValue, trim);
	}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC12 characterization: GetOscConfig reports a HSE bypass bit even when HSEON is also set") {
	// NEEDS REVIEW: which state wins when HSEON and HSEBYP are both set is a HAL choice (RM0090: bypass needs HSEON).
	poke(&RCC->CR, 0x83u | (1u << 16) | (1u << 18)); // HSEON and HSEBYP
	RCC_OscInitTypeDef got = {};
	HAL_RCC_GetOscConfig(&got);
	CHECK(got.HSEState == RCC_HSE_BYPASS);
}

// ================================================================ RC13 MCO
TEST_CASE_FIXTURE(RccClockFixture, "RC13 MCOConfig MCO1 writes the source in bits 22:21 for all four sources") {
	// RM0090 7.3.3: 00 HSI, 01 LSE, 10 HSE, 11 PLLCLK.
	const uint32_t srcApi[4] = {RCC_MCO1SOURCE_HSI, RCC_MCO1SOURCE_LSE, RCC_MCO1SOURCE_HSE, RCC_MCO1SOURCE_PLLCLK};
	Mismatch mm;
	for (uint32_t code = 0; code < 4; ++code) {
		HAL_RCC_MCOConfig(RCC_MCO1, srcApi[code], RCC_MCODIV_1);
		const uint32_t got = (cfgr() >> 21) & 3u;
		if (got != code) note(mm, "MCO1", code, 0, 0, got, code);
	}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC13 MCOConfig MCO1 writes the prescaler in bits 26:24 for the divisors 1 to 5") {
	// RM0090 7.3.3 MCO1PRE: 0xx no division, 100 /2, 101 /3, 110 /4, 111 /5.
	const uint32_t divApi[5] = {RCC_MCODIV_1, RCC_MCODIV_2, RCC_MCODIV_3, RCC_MCODIV_4, RCC_MCODIV_5};
	const uint32_t code[5] = {0x0, 0x4, 0x5, 0x6, 0x7};
	Mismatch mm;
	for (unsigned i = 0; i < 5; ++i) {
		HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, divApi[i]);
		const uint32_t got = (cfgr() >> 24) & 7u;
		if (got != code[i]) note(mm, "MCO1PRE", i + 1, 0, 0, got, code[i]);
	}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC13 MCOConfig MCO2 writes the source in bits 31:30 for all four sources") {
	// RM0090 7.3.3: 00 SYSCLK, 01 PLLI2S, 10 HSE, 11 PLLCLK.
	const uint32_t srcApi[4] = {RCC_MCO2SOURCE_SYSCLK, RCC_MCO2SOURCE_PLLI2SCLK, RCC_MCO2SOURCE_HSE, RCC_MCO2SOURCE_PLLCLK};
	Mismatch mm;
	for (uint32_t code = 0; code < 4; ++code) {
		HAL_RCC_MCOConfig(RCC_MCO2, srcApi[code], RCC_MCODIV_1);
		const uint32_t got = (cfgr() >> 30) & 3u;
		if (got != code) note(mm, "MCO2", code, 0, 0, got, code);
	}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC13 MCOConfig MCO2 writes the prescaler in bits 29:27 for the divisors 1 to 5") {
	const uint32_t divApi[5] = {RCC_MCODIV_1, RCC_MCODIV_2, RCC_MCODIV_3, RCC_MCODIV_4, RCC_MCODIV_5};
	const uint32_t code[5] = {0x0, 0x4, 0x5, 0x6, 0x7};
	Mismatch mm;
	for (unsigned i = 0; i < 5; ++i) {
		HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_HSE, divApi[i]);
		const uint32_t got = (cfgr() >> 27) & 7u;
		if (got != code[i]) note(mm, "MCO2PRE", i + 1, 0, 0, got, code[i]);
	}
	CHECK_MESSAGE(mm.bad == 0u, mm.first);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC13 MCOConfig MCO1 leaves MCO2 and the other CFGR fields unchanged") {
	// MCO2 = 10, MCO2PRE = 111, PPRE2 = 100, PPRE1 = 101, HPRE = 1001: 0xB8000000 | 0x8000 | 0x1400 | 0x90.
	poke(&RCC->CFGR, 0xB8009490u);
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_3);
	// MCO1 = 11 (bits 22:21 = 0x00600000), MCO1PRE = 101 (bits 26:24 = 0x05000000)
	CHECK(cfgr() == (0xB8009490u | 0x00600000u | 0x05000000u));
}

TEST_CASE_FIXTURE(RccClockFixture, "RC13 MCOConfig MCO2 leaves MCO1 and the other CFGR fields unchanged") {
	// MCO1 = 10 (0x00400000), MCO1PRE = 111 (0x07000000), PPRE2 = 100 (0x8000), PPRE1 = 101 (0x1400), HPRE = 1001 (0x90)
	poke(&RCC->CFGR, 0x07409490u);
	HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLLCLK, RCC_MCODIV_3);
	// MCO2 = 11 (0xC0000000), MCO2PRE = 101 (bits 29:27 = 0x28000000)
	CHECK(cfgr() == (0x07409490u | 0xC0000000u | 0x28000000u));
}

TEST_CASE_FIXTURE(RccClockFixture, "RC13 MCOConfig replaces an earlier source and prescaler") {
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5);
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_2);
	CHECK(((cfgr() >> 21) & 3u) == 1u); // LSE
	CHECK(((cfgr() >> 24) & 7u) == 4u); // /2
}

TEST_CASE_FIXTURE(RccClockFixture, "RC13 characterization: MCOConfig MCO1 configures PA8 as very high speed alternate function 0") {
	// NEEDS REVIEW: UM1725 only says "PA8/PC9 should be configured in alternate function mode" (a note for the caller);
	// the HAL does it itself: push-pull alternate function (MODER 10), no pull, very high speed (OSPEEDR 11), AF0.
	// GPIOA reset values (RM0090 8.4): MODER 0xA8000000, OSPEEDR 0x0C000000, others 0.
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
	CHECK(peek(&GPIOA->MODER) == (0xA8000000u | (2u << 16)));   // PA8: bits 17:16 = 10
	CHECK(peek(&GPIOA->OSPEEDR) == (0x0C000000u | (3u << 16))); // 11
	CHECK(peek(&GPIOA->OTYPER) == 0u);                          // push-pull
	CHECK(peek(&GPIOA->PUPDR) == 0x64000000u);                  // GPIOA reset value, PA8 no pull
	CHECK(peek(&GPIOA->AFR[1]) == 0u);                          // AFRH nibble of PA8 (bits 3:0) = AF0
}

TEST_CASE_FIXTURE(RccClockFixture, "RC13 characterization: MCOConfig MCO2 configures PC9 as very high speed alternate function 0") {
	// NEEDS REVIEW: as for MCO1, now PC9 (GPIOC reset values are 0).
	HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_HSE, RCC_MCODIV_1);
	CHECK(peek(&GPIOC->MODER) == (2u << 18));   // PC9: bits 19:18 = 10
	CHECK(peek(&GPIOC->OSPEEDR) == (3u << 18)); // 11
	CHECK(peek(&GPIOC->OTYPER) == 0u);
	CHECK(peek(&GPIOC->PUPDR) == 0u);
	CHECK(peek(&GPIOC->AFR[1]) == 0u); // AFRH nibble of PC9 (bits 7:4) = AF0
}

TEST_CASE_FIXTURE(RccClockFixture, "RC13 characterization: MCOConfig enables the clock of the port of the output pin") {
	// NEEDS REVIEW: the port clock enable is not described by UM1725 for MCOConfig. RM0090 7.3.10: GPIOAEN bit 0,
	// GPIOCEN bit 2 of RCC_AHB1ENR (reset 0x00100000: CCMDATARAMEN only).
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
	CHECK(peek(&RCC->AHB1ENR) == 0x00100001u);
	HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_1);
	CHECK(peek(&RCC->AHB1ENR) == 0x00100005u);
}

// ================================================================ RC14 CSS
TEST_CASE_FIXTURE(RccClockFixture, "RC14 EnableCSS sets CSSON") {
	HAL_RCC_EnableCSS();
	CHECK((peek(&RCC->CR) & (1u << 19)) != 0u); // CSSON bit 19 (7.3.1)
}

TEST_CASE_FIXTURE(RccClockFixture, "RC14 EnableCSS leaves the other bits of RCC_CR") {
	HAL_RCC_EnableCSS();
	CHECK(peek(&RCC->CR) == (0x83u | (1u << 19))); // reset value 0x83: HSION, HSIRDY, HSITRIM 10000
}

TEST_CASE_FIXTURE(RccClockFixture, "RC14 DisableCSS clears CSSON") {
	HAL_RCC_EnableCSS();
	HAL_RCC_DisableCSS();
	CHECK((peek(&RCC->CR) & (1u << 19)) == 0u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC14 DisableCSS leaves the other bits of RCC_CR") {
	HAL_RCC_EnableCSS();
	HAL_RCC_DisableCSS();
	CHECK(peek(&RCC->CR) == 0x83u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC14 NMI handler with CSSF set calls the callback once") {
	poke(&RCC->CIR, 1u << 7); // CSSF
	HAL_RCC_NMI_IRQHandler();
	CHECK(g_cssCalls == 1);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC14 NMI handler with CSSF set clears CSSF") {
	poke(&RCC->CIR, 1u << 7);
	HAL_RCC_NMI_IRQHandler();
	CHECK((peek(&RCC->CIR) & (1u << 7)) == 0u); // CSSC (bit 23) cleared the flag (7.3.4)
}

TEST_CASE_FIXTURE(RccClockFixture, "RC14 NMI handler clears only CSSF of the pending flags") {
	poke(&RCC->CIR, (1u << 7) | (1u << 3)); // CSSF and HSERDYF
	HAL_RCC_NMI_IRQHandler();
	CHECK(peek(&RCC->CIR) == (1u << 3));
}

TEST_CASE_FIXTURE(RccClockFixture, "RC14 NMI handler without CSSF does not call the callback") {
	poke(&RCC->CIR, 1u << 3); // another flag only
	HAL_RCC_NMI_IRQHandler();
	CHECK(g_cssCalls == 0);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC14 NMI handler without CSSF writes nothing to RCC_CIR") {
	poke(&RCC->CIR, 1u << 3);
	hostRegsLogClear();
	HAL_RCC_NMI_IRQHandler();
	CHECK(firstWrite(addrOf(&RCC->CIR), [](uint32_t) { return true; }) == -1);
	CHECK(peek(&RCC->CIR) == (1u << 3));
}

TEST_CASE_FIXTURE(RccClockFixture, "RC14 a second NMI after the flag was cleared does not call the callback again") {
	poke(&RCC->CIR, 1u << 7);
	HAL_RCC_NMI_IRQHandler();
	HAL_RCC_NMI_IRQHandler();
	CHECK(g_cssCalls == 1);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC14 a HSE failure with CSS on is reported by the NMI handler") {
	REQUIRE(startHse());
	HAL_RCC_EnableCSS();
	hostRccHseFailure();
	HAL_RCC_NMI_IRQHandler();
	CHECK(g_cssCalls == 1);
	CHECK((peek(&RCC->CIR) & (1u << 7)) == 0u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC14 a HSE failure with the CSS off sets no flag and the NMI handler does nothing") {
	REQUIRE(startHse());
	hostRccHseFailure();
	HAL_RCC_NMI_IRQHandler();
	CHECK(g_cssCalls == 0);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC14 after a HSE failure with the HSE as system clock the HSI is used and the HSE is off") {
	// UM1725 ClockConfig note and RM0090 7.3.1: on a HSE failure the hardware switches to the HSI and disables the HSE.
	REQUIRE(startHse());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSE, 0x0, 0x0, 0x0, 0) == HAL_OK);
	HAL_RCC_EnableCSS();
	hostRccHseFailure();
	CHECK(HAL_RCC_GetSysClockFreq() == kHsi); // reads CFGR, SWS = HSI
	CHECK((peek(&RCC->CR) & (1u << 16)) == 0u); // HSEON off
}

// ================================================================ RC2 (clock configuration side)
TEST_CASE_FIXTURE(RccClockFixture, "RC2 clock config: DeInit leaves RCC_CIR at 0 after a ready interrupt flag was raised") {
	// RM0090 7.3.4: HSERDYIE bit 11 enables the interrupt, HSERDYF bit 3 is set when the HSE is stable; UM1725 DeInit:
	// "All interrupts disabled" (enables cleared and the flags cleared through the xxRDYC bits).
	__HAL_RCC_ENABLE_IT(RCC_IT_HSERDY);
	REQUIRE(startHse());
	REQUIRE((peek(&RCC->CIR) & (1u << 3)) != 0u);
	uwTickPrio = 15u;
	HAL_RCC_DeInit();
	CHECK(peek(&RCC->CIR) == 0u);
}

// ================================================================ RC15 errors
// NEEDS REVIEW for the four "is rejected" cases: UM1725 gives no return code for a source that is not ready; the HAL
// returns HAL_ERROR (not HAL_TIMEOUT) before it writes SW. The plain oracle is that SW was never written with the
// target source (RM0090 7.3.3: a source that is not ready must not be selected).
TEST_CASE_FIXTURE(RccClockFixture, "RC15 characterization: ClockConfig to the HSE while the HSE is off is rejected with HAL_ERROR") {
	hostRegsLogClear();
	CHECK(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSE, 0x0, 0x0, 0x0, 0) == HAL_ERROR);
	CHECK_FALSE(swWritten(1u));
	CHECK(((cfgr() >> 2) & 3u) == 0u); // SWS still HSI
}

TEST_CASE_FIXTURE(RccClockFixture, "RC15 characterization: ClockConfig to the PLL while the PLL is off is rejected with HAL_ERROR") {
	hostRegsLogClear();
	CHECK(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x0, 0x0, 0) == HAL_ERROR);
	CHECK_FALSE(swWritten(2u));
	CHECK(((cfgr() >> 2) & 3u) == 0u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC15 characterization: ClockConfig to the HSI while the HSI is off is rejected with HAL_ERROR") {
	REQUIRE(startHse());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSE, 0x0, 0x0, 0x0, 0) == HAL_OK);
	__HAL_RCC_HSI_DISABLE(); // allowed: the HSE is the system clock
	REQUIRE((peek(&RCC->CR) & 1u) == 0u);
	hostRegsLogClear();
	CHECK(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSI, 0x0, 0x0, 0x0, 0) == HAL_ERROR);
	CHECK_FALSE(swWritten(0u));
	CHECK(((cfgr() >> 2) & 3u) == 1u); // still HSE
}

TEST_CASE_FIXTURE(RccClockFixture, "RC15 characterization: ClockConfig to a HSE that never becomes ready is rejected with HAL_ERROR") {
	__HAL_RCC_HSE_CONFIG(RCC_HSE_ON); // HSEON without waiting
	hostRccFail(HOST_OSC_HSE, 1);
	hostRegsLogClear();
	CHECK(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSE, 0x0, 0x0, 0x0, 0) == HAL_ERROR);
	CHECK_FALSE(swWritten(1u));
	CHECK(((cfgr() >> 2) & 3u) == 0u);
	CHECK(SystemCoreClock == kHsi);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC15 a rejected ClockConfig leaves SystemCoreClock and the tick unchanged") {
	hostTickSet(100u);
	CHECK(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x9, 0x0, 0x0, 0) != HAL_OK);
	CHECK(SystemCoreClock == kHsi);
	CHECK(SysTick->LOAD == 0u); // no HAL_InitTick: SysTick was never configured in this case
}

TEST_CASE_FIXTURE(RccClockFixture, "RC15 ClockConfig of a clock source that is not ready yet switches when it becomes ready" * KNOWN_ISSUE("candidate vendor bug V93 (arguable: UM1725 note says the switch occurs when the source is ready; the HAL returns HAL_ERROR before writing SW)")) {
	hostRccSetStartupTicks(HOST_OSC_HSE, 5);
	__HAL_RCC_HSE_CONFIG(RCC_HSE_ON); // HSEON, HSERDY follows after 5 ticks
	const HAL_StatusTypeDef st = clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSE, 0x0, 0x0, 0x0, 0);
	for (int i = 0; i < 10; ++i) HAL_GetTick();
	CHECK(((cfgr() >> 2) & 3u) == 1u); // UM1725: SWS = HSE once the HSE is ready
	CHECK(st == HAL_OK);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC15 characterization: a rejected ClockConfig has already written HPRE and set PPRE1 and PPRE2 to /16") {
	// NEEDS REVIEW: neither UM1725 nor RM0090 say what a rejected call leaves behind. The HAL writes the highest APB
	// dividers and the new HPRE before it checks the ready flag of the source, and does not undo them on HAL_ERROR.
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSE, 0x9, 0x0, 0x0, 0) == HAL_ERROR); // HSE is off
	CHECK(((cfgr() >> 4) & 0xFu) == 0x9u);
	CHECK(((cfgr() >> 10) & 7u) == 7u);
	CHECK(((cfgr() >> 13) & 7u) == 7u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC15 ClockConfig with a latency that is too large to read back leaves the prescalers alone") {
	// The refused latency comes before every other write (3.4: the new latency must be effective first).
	poke(&RCC->CFGR, (0xBu << 4) | (6u << 10) | (5u << 13));
	REQUIRE(startFirmwarePll());
	CHECK(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 8) == HAL_ERROR);
	CHECK(((cfgr() >> 4) & 0xFu) == 0xBu);
	CHECK(((cfgr() >> 10) & 7u) == 6u);
	CHECK(((cfgr() >> 13) & 7u) == 5u);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC15 ClockConfig selects the HSI again after another source") {
	REQUIRE(startHse());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSE, 0x0, 0x0, 0x0, 0) == HAL_OK);
	CHECK(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSI, 0x0, 0x0, 0x0, 0) == HAL_OK);
	CHECK(((cfgr() >> 2) & 3u) == 0u);
	CHECK(SystemCoreClock == kHsi);
}

TEST_CASE_FIXTURE(RccClockFixture, "RC15 after a CSS fallback the PLL is refused and the HSI is accepted") {
	REQUIRE(startFirmwarePll());
	REQUIRE(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5) == HAL_OK);
	HAL_RCC_EnableCSS();
	hostRccHseFailure(); // HSE and PLL lost, HSI is the system clock
	HAL_RCC_GetSysClockFreq();
	// the PLL is gone with the HSE: selecting it again is refused, selecting the HSI works
	CHECK(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_PLLCLK, 0x0, 0x5, 0x4, 5) != HAL_OK);
	CHECK(clockConfig(kAllTypes, RCC_SYSCLKSOURCE_HSI, 0x0, 0x5, 0x4, 5) == HAL_OK);
}

} // TEST_SUITE
