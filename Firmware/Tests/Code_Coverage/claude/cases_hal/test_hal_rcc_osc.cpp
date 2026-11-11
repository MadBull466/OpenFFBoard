/*
 * P12 T2 group RC1-RC9: RCC oscillator and PLL control of the real ST HAL (stm32f4xx_hal_rcc.c, the F407 branches of
 * stm32f4xx_hal_rcc_ex.c: HAL_RCC_DeInit, HAL_RCC_OscConfig, HAL_RCC_GetSysClockFreq) against the RCC / PWR behavioural
 * model (fakes/stm32_hal/HalClockPower.h). Requirements: Tests/Code_Coverage/claude/charters/hal/P12_T2_REQUIREMENTS.md.
 *
 * Oracles: UM1725 Rev 8 (RCC chapter: DeInit and OscConfig notes), RM0090 Rev 22 7.3.1 (RCC_CR), 7.3.2 (RCC_PLLCFGR),
 * 7.3.3 (RCC_CFGR), 7.3.20 (RCC_BDCR), 7.3.21 (RCC_CSR), 7.3.23 (RCC_PLLI2SCFGR), 5.4.1 (PWR_CR).
 * Bit positions used in the derivations (RM0090):
 *  - RCC_CR: HSION 0, HSIRDY 1, HSITRIM 7:3 (reset 0x10), HSEON 16, HSERDY 17, HSEBYP 18, CSSON 19, PLLON 24, PLLRDY 25.
 *  - RCC_PLLCFGR: PLLM 5:0, PLLN 14:6, PLLP 17:16 (00 /2, 01 /4, 10 /6, 11 /8), PLLSRC 22 (0 HSI, 1 HSE), PLLQ 27:24;
 *    VCO = source / PLLM * PLLN, PLLCLK = VCO / PLLP, USB / SDIO clock = VCO / PLLQ. Reset value 0x24003010.
 *  - RCC_CFGR: SW 1:0 (00 HSI, 01 HSE, 10 PLL), SWS 3:2, HPRE 7:4, PPRE1 12:10, PPRE2 15:13, MCO1 22:21,
 *    MCO1PRE 26:24, MCO2PRE 29:27, MCO2 31:30.
 *  - RCC_CIR: ready interrupt enables 13:8. RCC_CSR: LSION 0, LSIRDY 1. RCC_BDCR: LSEON 0, LSERDY 1, LSEBYP 2,
 *    RTCSEL 9:8, RTCEN 15. PWR_CR: DBP 8.
 * Constants from the HAL header (RCC_HSE_ON, RCC_PLL_ON, ...) are inputs only; every expected register value is written
 * out here. Reserved PLLCFGR bits are masked out (the HAL writes the whole register; RM0090 says reserved bits keep their
 * reset value).
 *
 * Time: the HAL tick. Slow oscillators use hostRccSetStartupTicks, failing ones hostRccFail; a polling loop on a never ready
 * oscillator ends through the replaced HAL_GetTick (one tick per call).
 *
 * NEEDS REVIEW (proposed text, numbers are assigned by the lead; every case named "characterization:" below):
 *  - RC2 HAL_RCC_DeInit leaves SystemCoreClock = HSI_VALUE (16 MHz): derived from "HSI is the system clock", UM1725
 *    gives no variable; the charter asks for characterization.
 *  - RC2 HAL_RCC_DeInit returns HAL_TIMEOUT when the HSI never gets ready: UM1725 gives no return codes.
 *  - RC3 HAL_RCC_OscConfig(NULL) returns HAL_ERROR; HSE that never gets ready returns HAL_TIMEOUT.
 *  - RC4 HSE OFF request while HSE is SYSCLK (directly or as PLL source) returns HAL_ERROR (bit unchanged is plain).
 *  - RC5 HSI OFF request while HSI is SYSCLK returns HAL_ERROR; HSI that never gets ready returns HAL_TIMEOUT.
 *  - RC6 LSI that never gets ready returns HAL_TIMEOUT.
 *  - RC7 LSE that never gets ready returns HAL_TIMEOUT.
 *  - RC8 PLL that never gets ready, and PLL with HSE source while the HSE is off, return HAL_TIMEOUT.
 *  - RC9 PLL OFF / reconfiguration while the PLL is SYSCLK returns HAL_ERROR; repeating the running configuration
 *    returns HAL_OK.
 */
#include "HalFixture.h"
#include "HalClockPower.h"
#include "TestHelpers.h"

#include <cstdint>

namespace {

// RCC_CR bits
constexpr uint32_t CR_HSION = 1u << 0;
constexpr uint32_t CR_HSIRDY = 1u << 1;
constexpr uint32_t CR_HSEON = 1u << 16;
constexpr uint32_t CR_HSERDY = 1u << 17;
constexpr uint32_t CR_HSEBYP = 1u << 18;
constexpr uint32_t CR_CSSON = 1u << 19;
constexpr uint32_t CR_PLLON = 1u << 24;
constexpr uint32_t CR_PLLRDY = 1u << 25;
// RCC_CSR / RCC_BDCR / PWR_CR bits
constexpr uint32_t CSR_LSION = 1u << 0;
constexpr uint32_t CSR_LSIRDY = 1u << 1;
constexpr uint32_t BDCR_LSEON = 1u << 0;
constexpr uint32_t BDCR_LSERDY = 1u << 1;
constexpr uint32_t BDCR_LSEBYP = 1u << 2;
constexpr uint32_t PWR_DBP = 1u << 8;
// RCC_PLLCFGR fields without the reserved bits
constexpr uint32_t PLLCFGR_FIELDS = 0x0F437FFFu;
// RCC_CFGR values
constexpr uint32_t SW_HSE = 1u;
constexpr uint32_t SW_PLL = 2u;

uint32_t sws() { return (RCC->CFGR >> 2) & 3u; }
uint32_t pllcfgr() { return RCC->PLLCFGR & PLLCFGR_FIELDS; }

void selectSysclk(uint32_t sw) { RCC->CFGR = (RCC->CFGR & ~3u) | sw; }

HAL_StatusTypeDef oscHse(uint32_t state) {
	RCC_OscInitTypeDef o = {};
	o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	o.HSEState = state;
	o.PLL.PLLState = RCC_PLL_NONE;
	return HAL_RCC_OscConfig(&o);
}

HAL_StatusTypeDef oscHsi(uint8_t state, uint32_t trim) {
	RCC_OscInitTypeDef o = {};
	o.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	o.HSIState = state;
	o.HSICalibrationValue = trim;
	o.PLL.PLLState = RCC_PLL_NONE;
	return HAL_RCC_OscConfig(&o);
}

HAL_StatusTypeDef oscLsi(uint8_t state) {
	RCC_OscInitTypeDef o = {};
	o.OscillatorType = RCC_OSCILLATORTYPE_LSI;
	o.LSIState = state;
	o.PLL.PLLState = RCC_PLL_NONE;
	return HAL_RCC_OscConfig(&o);
}

HAL_StatusTypeDef oscLse(uint32_t state) {
	RCC_OscInitTypeDef o = {};
	o.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	o.LSEState = state;
	o.PLL.PLLState = RCC_PLL_NONE;
	return HAL_RCC_OscConfig(&o);
}

HAL_StatusTypeDef oscPll(uint8_t state, uint32_t src, uint32_t m, uint32_t n, uint32_t p, uint32_t q) {
	RCC_OscInitTypeDef o = {};
	o.OscillatorType = RCC_OSCILLATORTYPE_NONE;
	o.PLL.PLLState = state;
	o.PLL.PLLSource = src;
	o.PLL.PLLM = m;
	o.PLL.PLLN = n;
	o.PLL.PLLP = p;
	o.PLL.PLLQ = q;
	return HAL_RCC_OscConfig(&o);
}

// The firmware configuration: HSE 8 MHz, M 4, N 168, P 2, Q 7.
HAL_StatusTypeDef oscPllFirmware() { return oscPll(RCC_PLL_ON, RCC_PLLSOURCE_HSE, 4u, 168u, RCC_PLLP_DIV2, 7u); }

// HSE on, PLL on (firmware configuration), SYSCLK = PLL.
void pllIsSysclk() {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	REQUIRE(oscPllFirmware() == HAL_OK);
	selectSysclk(SW_PLL);
	REQUIRE(sws() == 2u);
}

// HSE on, SYSCLK = HSE.
void hseIsSysclk() {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	selectSysclk(SW_HSE);
	REQUIRE(sws() == 1u);
}

} // namespace

TEST_SUITE("P12 T2 RCC oscillators") {

// ---------------------------------------------------------------- RC1 reset values

TEST_CASE_FIXTURE(HalFixture, "RC1 RCC_CR reads 0x83 after reset (HSION, HSIRDY, HSITRIM 0x10)") {
	CHECK(RCC->CR == 0x00000083u); // RM0090 7.3.1: HSION 1 + HSIRDY 1 + HSITRIM 0x10 << 3 = 0x80
}

TEST_CASE_FIXTURE(HalFixture, "RC1 RCC_PLLCFGR reads 0x24003010 after reset") {
	CHECK(RCC->PLLCFGR == 0x24003010u); // RM0090 7.3.2
}

TEST_CASE_FIXTURE(HalFixture, "RC1 RCC_CFGR reads 0 after reset") {
	CHECK(RCC->CFGR == 0u); // RM0090 7.3.3
}

TEST_CASE_FIXTURE(HalFixture, "RC1 RCC_CIR reads 0 after reset") {
	CHECK(RCC->CIR == 0u); // RM0090 7.3.4
}

TEST_CASE_FIXTURE(HalFixture, "RC1 RCC_CSR reads 0x0E000000 after reset") {
	CHECK(RCC->CSR == 0x0E000000u); // RM0090 7.3.21
}

TEST_CASE_FIXTURE(HalFixture, "RC1 RCC_PLLI2SCFGR reads 0x20003000 after reset") {
	CHECK(RCC->PLLI2SCFGR == 0x20003000u); // RM0090 7.3.23
}

TEST_CASE_FIXTURE(HalFixture, "RC1 HAL_RCC_GetSysClockFreq returns the HSI frequency after reset") {
	CHECK(HAL_RCC_GetSysClockFreq() == 16000000u); // HSI 16 MHz (RM0090 7.2.2), HSI is SYSCLK after reset
}

// ---------------------------------------------------------------- RC2 HAL_RCC_DeInit

namespace {
// A configured system: HSE, PLL as SYSCLK, prescalers, MCO, CSS and ready interrupts on.
void configuredSystem() {
	uwTickPrio = 15u; // HAL_InitTick (called by DeInit) rejects the start value 16
	pllIsSysclk();
	RCC->CFGR = (RCC->CFGR & 3u) | 0xFFFFFCF0u; // HPRE, PPRE1, PPRE2, RTCPRE, I2SSRC, MCO1, MCO1PRE, MCO2PRE, MCO2 all ones
	RCC->CR |= CR_CSSON;
	RCC->CIR = 0x00003F00u; // all six ready interrupt enables
}
} // namespace

TEST_CASE_FIXTURE(HalFixture, "RC2 HAL_RCC_DeInit returns HAL_OK") {
	configuredSystem();
	CHECK(HAL_RCC_DeInit() == HAL_OK);
}

TEST_CASE_FIXTURE(HalFixture, "RC2 HAL_RCC_DeInit leaves the HSI on and selected as system clock") {
	configuredSystem();
	HAL_RCC_DeInit();
	CHECK((RCC->CR & (CR_HSION | CR_HSIRDY)) == (CR_HSION | CR_HSIRDY));
	CHECK((RCC->CFGR & 3u) == 0u); // SW = HSI
	CHECK(sws() == 0u);            // SWS = HSI
}

TEST_CASE_FIXTURE(HalFixture, "RC2 HAL_RCC_DeInit switches the HSE off") {
	configuredSystem();
	HAL_RCC_DeInit();
	CHECK((RCC->CR & (CR_HSEON | CR_HSERDY | CR_HSEBYP)) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC2 HAL_RCC_DeInit switches the PLL off") {
	configuredSystem();
	HAL_RCC_DeInit();
	CHECK((RCC->CR & (CR_PLLON | CR_PLLRDY)) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC2 HAL_RCC_DeInit switches the clock security system off") {
	configuredSystem();
	HAL_RCC_DeInit();
	CHECK((RCC->CR & CR_CSSON) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC2 HAL_RCC_DeInit sets the AHB and both APB prescalers to 1") {
	configuredSystem();
	HAL_RCC_DeInit();
	const uint32_t cfgr = RCC->CFGR;
	CHECK(((cfgr >> 4) & 0xFu) == 0u);  // HPRE 0xxx: not divided
	CHECK(((cfgr >> 10) & 7u) == 0u);   // PPRE1 0xx: not divided
	CHECK(((cfgr >> 13) & 7u) == 0u);   // PPRE2 0xx: not divided
}

TEST_CASE_FIXTURE(HalFixture, "RC2 HAL_RCC_DeInit switches MCO1 and MCO2 to their reset configuration") {
	configuredSystem();
	HAL_RCC_DeInit();
	const uint32_t cfgr = RCC->CFGR;
	CHECK(((cfgr >> 21) & 3u) == 0u); // MCO1 source: HSI (reset value)
	CHECK(((cfgr >> 24) & 7u) == 0u); // MCO1PRE: no division
	CHECK(((cfgr >> 27) & 7u) == 0u); // MCO2PRE: no division
	CHECK(((cfgr >> 30) & 3u) == 0u); // MCO2 source: SYSCLK (reset value)
}

TEST_CASE_FIXTURE(HalFixture, "RC2 HAL_RCC_DeInit disables all ready interrupts") {
	configuredSystem();
	REQUIRE((RCC->CIR & 0x3F00u) == 0x3F00u);
	HAL_RCC_DeInit();
	CHECK((RCC->CIR & 0x3F00u) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC2 HAL_RCC_DeInit returns RCC_PLLCFGR to its reset value") {
	configuredSystem();
	REQUIRE(pllcfgr() != (0x24003010u & PLLCFGR_FIELDS));
	HAL_RCC_DeInit();
	CHECK(RCC->PLLCFGR == 0x24003010u); // RM0090 7.3.2
}

TEST_CASE_FIXTURE(HalFixture, "RC2 HAL_RCC_DeInit returns RCC_PLLI2SCFGR to its reset value") {
	configuredSystem();
	RCC->PLLI2SCFGR = 0x50001400u; // PLLI2SN = 80 (bits 14:6 = 0x50 << 6 = 0x1400) and PLLI2SR = 5 (bits 30:28 = 0x50000000)
	REQUIRE(RCC->PLLI2SCFGR != 0x20003000u);
	HAL_RCC_DeInit();
	CHECK(RCC->PLLI2SCFGR == 0x20003000u); // RM0090 7.3.23
}

TEST_CASE_FIXTURE(HalFixture, "RC2 characterization: HAL_RCC_DeInit only sets HSITRIM bit 4") {
	// NEEDS REVIEW: UM1725 does not list HSITRIM. DeInit only sets HSITRIM bit 4, so a start trim of 5 ends as 0x15;
	// the RM0090 7.3.1 reset value is 0x10.
	configuredSystem();
	REQUIRE(oscHsi(RCC_HSI_ON, 0x05u) == HAL_OK); // PLL is SYSCLK (HSI is not): the trim is then written freely
	REQUIRE(((RCC->CR >> 3) & 0x1Fu) == 0x05u);
	HAL_RCC_DeInit();
	CHECK(((RCC->CR >> 3) & 0x1Fu) == 0x15u); // RM0090 7.3.1
}

TEST_CASE_FIXTURE(HalFixture, "RC2 HAL_RCC_DeInit does not change the peripheral clock enable registers") {
	configuredSystem();
	RCC->AHB1ENR = 0x00100000u | 0x7Fu;
	RCC->APB1ENR = 0x10000000u | 0x01u;
	RCC->APB2ENR = 0x00004011u;
	HAL_RCC_DeInit();
	CHECK(RCC->AHB1ENR == (0x00100000u | 0x7Fu));
	CHECK(RCC->APB1ENR == (0x10000000u | 0x01u));
	CHECK(RCC->APB2ENR == 0x00004011u);
}

TEST_CASE_FIXTURE(HalFixture, "RC2 HAL_RCC_DeInit does not change the LSE and the RTC configuration") {
	configuredSystem();
	REQUIRE(oscLse(RCC_LSE_ON) == HAL_OK);
	RCC->BDCR |= (1u << 8) | (1u << 15); // RTCSEL = 01 (LSE), RTCEN
	const uint32_t before = RCC->BDCR & ~BDCR_LSERDY;
	REQUIRE((before & (BDCR_LSEON | (1u << 8) | (1u << 15))) == (BDCR_LSEON | (1u << 8) | (1u << 15)));
	HAL_RCC_DeInit();
	CHECK((RCC->BDCR & ~BDCR_LSERDY) == before);
}

TEST_CASE_FIXTURE(HalFixture, "RC2 HAL_RCC_DeInit does not switch the LSI off" * KNOWN_ISSUE("V92: UM1725: DeInit does not modify the LSI configuration; the HAL (F407 branch in stm32f4xx_hal_rcc_ex.c) clears LSION")) {
	configuredSystem();
	REQUIRE(oscLsi(RCC_LSI_ON) == HAL_OK);
	REQUIRE((RCC->CSR & CSR_LSION) != 0u);
	HAL_RCC_DeInit();
	CHECK((RCC->CSR & CSR_LSION) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC2 characterization: HAL_RCC_DeInit sets SystemCoreClock to the HSI frequency") {
	// NEEDS REVIEW: derived (HSI is SYSCLK, prescaler 1 = 16 MHz); UM1725 does not name the variable.
	configuredSystem();
	SystemCoreClock = 168000000u;
	HAL_RCC_DeInit();
	CHECK(SystemCoreClock == 16000000u);
}

TEST_CASE_FIXTURE(HalFixture, "RC2 HAL_RCC_DeInit clears a pending HSE ready flag and its interrupt enable so that RCC_CIR is 0") {
	uwTickPrio = 15u;
	RCC->CIR = 1u << 11; // HSERDYIE (RM0090 7.3.4)
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	REQUIRE((RCC->CIR & (1u << 3)) != 0u); // HSERDYF rose (needs HSERDYIE)
	HAL_RCC_DeInit();
	CHECK(RCC->CIR == 0u); // UM1725: all interrupts disabled; the flags are cleared through the RDYC bits
}

TEST_CASE_FIXTURE(HalFixture, "RC2 characterization: HAL_RCC_DeInit returns HAL_TIMEOUT when the HSI never gets ready") {
	// NEEDS REVIEW: UM1725 gives no return codes.
	uwTickPrio = 15u;
	hostRccFail(HOST_OSC_HSI, 1);
	const uint32_t t0 = hostTickPeek();
	CHECK(HAL_RCC_DeInit() == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 > 2u);
	CHECK(hostTickPeek() - t0 <= 4u); // characterization: the loop ends within the HAL timeout + 2 ticks
}

// ---------------------------------------------------------------- RC3 HSE

TEST_CASE_FIXTURE(HalFixture, "RC3 HSE ON sets HSEON and HSERDY and returns HAL_OK") {
	CHECK(oscHse(RCC_HSE_ON) == HAL_OK);
	CHECK((RCC->CR & (CR_HSEON | CR_HSERDY)) == (CR_HSEON | CR_HSERDY));
	CHECK((RCC->CR & CR_HSEBYP) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC3 HSE ON waits until the HSE start-up time has elapsed") {
	hostRccSetStartupTicks(HOST_OSC_HSE, 10u);
	CHECK(oscHse(RCC_HSE_ON) == HAL_OK);
	CHECK((RCC->CR & CR_HSERDY) != 0u);
	CHECK(hostTickPeek() >= 10u);
	CHECK(hostTickPeek() <= 12u); // the wait ends with the start-up time (+2 ticks)
}

TEST_CASE_FIXTURE(HalFixture, "RC3 HSE bypass sets HSEBYP and HSEON and returns HAL_OK") {
	CHECK(oscHse(RCC_HSE_BYPASS) == HAL_OK);
	CHECK((RCC->CR & (CR_HSEBYP | CR_HSEON)) == (CR_HSEBYP | CR_HSEON));
	CHECK((RCC->CR & CR_HSERDY) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC3 HSE OFF clears HSEON and HSERDY and returns HAL_OK") {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	CHECK(oscHse(RCC_HSE_OFF) == HAL_OK);
	CHECK((RCC->CR & (CR_HSEON | CR_HSERDY)) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC3 HSE OFF after bypass clears HSEBYP and HSEON") {
	REQUIRE(oscHse(RCC_HSE_BYPASS) == HAL_OK);
	CHECK(oscHse(RCC_HSE_OFF) == HAL_OK);
	CHECK((RCC->CR & (CR_HSEBYP | CR_HSEON | CR_HSERDY)) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC3 characterization: HSE ON then bypass without OFF leaves HSEBYP clear") {
	// NEEDS REVIEW: UM1725 only says the transition is "not supported"; the asserted result is the model rule of
	// RM0090 7.3.1 (HSEBYP writable only while the HSE is off), not a UM1725 statement.
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	oscHse(RCC_HSE_BYPASS);
	CHECK((RCC->CR & CR_HSEBYP) == 0u);
	CHECK((RCC->CR & CR_HSEON) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC3 characterization: HSE bypass then ON without OFF leaves HSEBYP set") {
	// NEEDS REVIEW: as above (RM0090 7.3.1 model rule, UM1725 only says "not supported").
	REQUIRE(oscHse(RCC_HSE_BYPASS) == HAL_OK);
	oscHse(RCC_HSE_ON);
	CHECK((RCC->CR & CR_HSEBYP) != 0u);
	CHECK((RCC->CR & CR_HSEON) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC3 an oscillator type without HSE leaves the HSE untouched") {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	REQUIRE(oscLsi(RCC_LSI_ON) == HAL_OK);
	CHECK((RCC->CR & (CR_HSEON | CR_HSERDY)) == (CR_HSEON | CR_HSERDY));
}

TEST_CASE_FIXTURE(HalFixture, "RC3 characterization: HSE that never gets ready returns HAL_TIMEOUT after the HSE start-up timeout") {
	// NEEDS REVIEW: UM1725 gives no return codes; HSE_STARTUP_TIMEOUT is 100 ms in stm32f4xx_hal_conf.h.
	hostRccFail(HOST_OSC_HSE, 1);
	const uint32_t t0 = hostTickPeek();
	CHECK(oscHse(RCC_HSE_ON) == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 <= 102u); // characterization: the loop ends within the HAL timeout + 2 ticks
	CHECK(hostTickPeek() - t0 > 100u);
	CHECK((RCC->CR & CR_HSERDY) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC3 characterization: HAL_RCC_OscConfig(NULL) returns HAL_ERROR") {
	// NEEDS REVIEW: UM1725 lists no error case for a null pointer.
	CHECK(HAL_RCC_OscConfig(nullptr) == HAL_ERROR);
}

TEST_CASE_FIXTURE(HalFixture, "RC3 characterization: HSE OFF whose HSERDY never clears returns HAL_TIMEOUT after the HSE start-up timeout") {
	// NEEDS REVIEW: UM1725 gives no return codes; sticky ready flag models an oscillator that does not stop.
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	hostRccStickyReady(HOST_OSC_HSE, 1);
	const uint32_t t0 = hostTickPeek();
	CHECK(oscHse(RCC_HSE_OFF) == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 <= 102u); // characterization: the loop ends within the HAL timeout + 2 ticks
	CHECK(hostTickPeek() - t0 > 100u);
}

// ---------------------------------------------------------------- RC4 HSE in use

TEST_CASE_FIXTURE(HalFixture, "RC4 HSE OFF is refused while the HSE is the system clock") {
	hseIsSysclk();
	oscHse(RCC_HSE_OFF);
	CHECK((RCC->CR & CR_HSEON) != 0u);
	CHECK(sws() == 1u);
}

TEST_CASE_FIXTURE(HalFixture, "RC4 HSE OFF is refused while the HSE is the PLL source of the system clock") {
	pllIsSysclk();
	oscHse(RCC_HSE_OFF);
	CHECK((RCC->CR & CR_HSEON) != 0u);
	CHECK(sws() == 2u);
}

TEST_CASE_FIXTURE(HalFixture, "RC4 characterization: HSE OFF while the HSE is the system clock returns HAL_ERROR") {
	// NEEDS REVIEW: UM1725 gives no return codes.
	hseIsSysclk();
	CHECK(oscHse(RCC_HSE_OFF) == HAL_ERROR);
}

TEST_CASE_FIXTURE(HalFixture, "RC4 characterization: HSE OFF while the HSE is the PLL source of the system clock returns HAL_ERROR") {
	// NEEDS REVIEW: UM1725 gives no return codes.
	pllIsSysclk();
	CHECK(oscHse(RCC_HSE_OFF) == HAL_ERROR);
}

// ---------------------------------------------------------------- RC5 HSI

TEST_CASE_FIXTURE(HalFixture, "RC5 HSI OFF clears HSION and HSIRDY while another clock is the system clock") {
	hseIsSysclk();
	CHECK(oscHsi(RCC_HSI_OFF, 0x10u) == HAL_OK);
	CHECK((RCC->CR & (CR_HSION | CR_HSIRDY)) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC5 HSI ON after OFF sets HSION and HSIRDY and returns HAL_OK") {
	hseIsSysclk();
	REQUIRE(oscHsi(RCC_HSI_OFF, 0x10u) == HAL_OK);
	CHECK(oscHsi(RCC_HSI_ON, 0x10u) == HAL_OK);
	CHECK((RCC->CR & (CR_HSION | CR_HSIRDY)) == (CR_HSION | CR_HSIRDY));
}

TEST_CASE_FIXTURE(HalFixture, "RC5 HSI ON writes the calibration trim into HSITRIM bits 7:3") {
	hseIsSysclk();
	REQUIRE(oscHsi(RCC_HSI_OFF, 0x10u) == HAL_OK);
	REQUIRE(oscHsi(RCC_HSI_ON, 0x0Au) == HAL_OK);
	CHECK(((RCC->CR >> 3) & 0x1Fu) == 0x0Au); // RM0090 7.3.1: HSITRIM[4:0] at bits 7:3
}

TEST_CASE_FIXTURE(HalFixture, "RC5 the maximum calibration trim 0x1F fills HSITRIM and leaves the other CR bits alone") {
	hseIsSysclk();
	REQUIRE(oscHsi(RCC_HSI_OFF, 0x10u) == HAL_OK);
	REQUIRE(oscHsi(RCC_HSI_ON, 0x1Fu) == HAL_OK);
	CHECK((RCC->CR & 0xF8u) == 0xF8u);
	CHECK((RCC->CR & (CR_HSEON | CR_HSERDY | CR_HSION | CR_HSIRDY)) == (CR_HSEON | CR_HSERDY | CR_HSION | CR_HSIRDY));
}

TEST_CASE_FIXTURE(HalFixture, "RC5 the HSI trim can be changed while the HSI is the system clock") {
	REQUIRE(sws() == 0u);
	oscHsi(RCC_HSI_ON, 0x07u);
	CHECK(((RCC->CR >> 3) & 0x1Fu) == 0x07u);
	CHECK((RCC->CR & CR_HSION) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC5 HSI OFF is refused while the HSI is the system clock") {
	REQUIRE(sws() == 0u);
	oscHsi(RCC_HSI_OFF, 0x10u);
	CHECK((RCC->CR & CR_HSION) != 0u);
	CHECK((RCC->CR & CR_HSIRDY) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC5 HSI OFF is refused while the HSI is the PLL source of the system clock") {
	REQUIRE(oscPll(RCC_PLL_ON, RCC_PLLSOURCE_HSI, 16u, 336u, RCC_PLLP_DIV2, 7u) == HAL_OK);
	selectSysclk(SW_PLL);
	REQUIRE(sws() == 2u);
	oscHsi(RCC_HSI_OFF, 0x10u);
	CHECK((RCC->CR & CR_HSION) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC5 characterization: HSI OFF while the HSI is the system clock returns HAL_ERROR") {
	// NEEDS REVIEW: UM1725 gives no return codes.
	REQUIRE(sws() == 0u);
	CHECK(oscHsi(RCC_HSI_OFF, 0x10u) == HAL_ERROR);
}

TEST_CASE_FIXTURE(HalFixture, "RC5 characterization: HSI that never gets ready returns HAL_TIMEOUT") {
	// NEEDS REVIEW: UM1725 gives no return codes.
	hseIsSysclk();
	REQUIRE(oscHsi(RCC_HSI_OFF, 0x10u) == HAL_OK);
	hostRccFail(HOST_OSC_HSI, 1);
	const uint32_t t0 = hostTickPeek();
	CHECK(oscHsi(RCC_HSI_ON, 0x10u) == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 > 2u);
	CHECK(hostTickPeek() - t0 <= 4u); // characterization: the loop ends within the HAL timeout + 2 ticks
}

TEST_CASE_FIXTURE(HalFixture, "RC5 characterization: HSI OFF whose HSIRDY never clears returns HAL_TIMEOUT") {
	// NEEDS REVIEW: UM1725 gives no return codes; ticks waited: more than the 2 ms HSI timeout.
	hseIsSysclk();
	hostRccStickyReady(HOST_OSC_HSI, 1);
	const uint32_t t0 = hostTickPeek();
	CHECK(oscHsi(RCC_HSI_OFF, 0x10u) == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 <= 4u); // characterization: the loop ends within the HAL timeout + 2 ticks
	CHECK(hostTickPeek() - t0 > 2u);
}

// ---------------------------------------------------------------- RC6 LSI

TEST_CASE_FIXTURE(HalFixture, "RC6 LSI ON sets LSION and LSIRDY in RCC_CSR and returns HAL_OK") {
	CHECK(oscLsi(RCC_LSI_ON) == HAL_OK);
	CHECK((RCC->CSR & (CSR_LSION | CSR_LSIRDY)) == (CSR_LSION | CSR_LSIRDY)); // RM0090 7.3.21
}

TEST_CASE_FIXTURE(HalFixture, "RC6 LSI ON waits until the LSI start-up time has elapsed") {
	hostRccSetStartupTicks(HOST_OSC_LSI, 2u);
	CHECK(oscLsi(RCC_LSI_ON) == HAL_OK);
	CHECK((RCC->CSR & CSR_LSIRDY) != 0u);
	CHECK(hostTickPeek() >= 2u);
	CHECK(hostTickPeek() <= 4u); // the wait ends with the start-up time (+2 ticks)
}

TEST_CASE_FIXTURE(HalFixture, "RC6 LSI OFF clears LSION and LSIRDY and returns HAL_OK") {
	REQUIRE(oscLsi(RCC_LSI_ON) == HAL_OK);
	CHECK(oscLsi(RCC_LSI_OFF) == HAL_OK);
	CHECK((RCC->CSR & (CSR_LSION | CSR_LSIRDY)) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC6 characterization: LSI that never gets ready returns HAL_TIMEOUT") {
	// NEEDS REVIEW: UM1725 gives no return codes.
	hostRccFail(HOST_OSC_LSI, 1);
	const uint32_t t0 = hostTickPeek();
	CHECK(oscLsi(RCC_LSI_ON) == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 > 2u);
	CHECK(hostTickPeek() - t0 <= 4u); // characterization: the loop ends within the HAL timeout + 2 ticks
	CHECK((RCC->CSR & CSR_LSIRDY) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC6 characterization: LSI OFF whose LSIRDY never clears returns HAL_TIMEOUT") {
	// NEEDS REVIEW: UM1725 gives no return codes; ticks waited: more than the 2 ms LSI timeout.
	REQUIRE(oscLsi(RCC_LSI_ON) == HAL_OK);
	hostRccStickyReady(HOST_OSC_LSI, 1);
	const uint32_t t0 = hostTickPeek();
	CHECK(oscLsi(RCC_LSI_OFF) == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 <= 4u); // characterization: the loop ends within the HAL timeout + 2 ticks
	CHECK(hostTickPeek() - t0 > 2u);
}

// ---------------------------------------------------------------- RC7 LSE

TEST_CASE_FIXTURE(HalFixture, "RC7 LSE ON sets LSEON and LSERDY in RCC_BDCR and returns HAL_OK") {
	CHECK(oscLse(RCC_LSE_ON) == HAL_OK);
	CHECK((RCC->BDCR & (BDCR_LSEON | BDCR_LSERDY)) == (BDCR_LSEON | BDCR_LSERDY)); // RM0090 7.3.20
}

TEST_CASE_FIXTURE(HalFixture, "RC7 LSE ON sets DBP in PWR_CR first (the backup domain is write protected after reset)") {
	REQUIRE((PWR->CR & PWR_DBP) == 0u);
	REQUIRE(oscLse(RCC_LSE_ON) == HAL_OK);
	CHECK((PWR->CR & PWR_DBP) != 0u); // RM0090 5.4.1
	CHECK((RCC->BDCR & BDCR_LSEON) != 0u); // a write to BDCR with DBP = 0 would have been ignored (RM0090 7.3.20)
}

TEST_CASE_FIXTURE(HalFixture, "RC7 LSE ON with DBP already set returns HAL_OK") {
	PWR->CR |= PWR_DBP;
	CHECK(oscLse(RCC_LSE_ON) == HAL_OK);
	CHECK((RCC->BDCR & (BDCR_LSEON | BDCR_LSERDY)) == (BDCR_LSEON | BDCR_LSERDY));
	CHECK((PWR->CR & PWR_DBP) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC7 LSE ON waits until the LSE start-up time has elapsed") {
	hostRccSetStartupTicks(HOST_OSC_LSE, 50u);
	CHECK(oscLse(RCC_LSE_ON) == HAL_OK);
	CHECK((RCC->BDCR & BDCR_LSERDY) != 0u);
	CHECK(hostTickPeek() >= 50u);
	CHECK(hostTickPeek() <= 52u); // the wait ends with the start-up time (+2 ticks)
}

TEST_CASE_FIXTURE(HalFixture, "RC7 LSE bypass sets LSEBYP and LSEON") {
	CHECK(oscLse(RCC_LSE_BYPASS) == HAL_OK);
	CHECK((RCC->BDCR & (BDCR_LSEBYP | BDCR_LSEON)) == (BDCR_LSEBYP | BDCR_LSEON));
}

TEST_CASE_FIXTURE(HalFixture, "RC7 LSE OFF clears LSEON and LSERDY and returns HAL_OK") {
	REQUIRE(oscLse(RCC_LSE_ON) == HAL_OK);
	CHECK(oscLse(RCC_LSE_OFF) == HAL_OK);
	CHECK((RCC->BDCR & (BDCR_LSEON | BDCR_LSERDY)) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC7 LSE OFF after bypass clears LSEBYP and LSEON") {
	REQUIRE(oscLse(RCC_LSE_BYPASS) == HAL_OK);
	CHECK(oscLse(RCC_LSE_OFF) == HAL_OK);
	CHECK((RCC->BDCR & (BDCR_LSEBYP | BDCR_LSEON | BDCR_LSERDY)) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC7 characterization: LSE ON then bypass without OFF leaves LSEBYP clear") {
	// NEEDS REVIEW: UM1725 only says the transition is "not supported"; the asserted result is the model rule of
	// RM0090 7.3.20 (LSEBYP writable only while the LSE is off), not a UM1725 statement.
	REQUIRE(oscLse(RCC_LSE_ON) == HAL_OK);
	oscLse(RCC_LSE_BYPASS);
	CHECK((RCC->BDCR & BDCR_LSEBYP) == 0u);
	CHECK((RCC->BDCR & BDCR_LSEON) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC7 characterization: LSE bypass then ON without OFF leaves LSEBYP set") {
	// NEEDS REVIEW: as above (RM0090 7.3.20 model rule, UM1725 only says "not supported").
	REQUIRE(oscLse(RCC_LSE_BYPASS) == HAL_OK);
	oscLse(RCC_LSE_ON);
	CHECK((RCC->BDCR & BDCR_LSEBYP) != 0u);
	CHECK((RCC->BDCR & BDCR_LSEON) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC7 characterization: LSE that never gets ready returns HAL_TIMEOUT") {
	// NEEDS REVIEW: UM1725 gives no return codes; the timeout is LSE_STARTUP_TIMEOUT (5000 ms).
	hostRccFail(HOST_OSC_LSE, 1);
	const uint32_t t0 = hostTickPeek();
	CHECK(oscLse(RCC_LSE_ON) == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 > 5000u);
	CHECK(hostTickPeek() - t0 <= 5005u); // characterization: the loop ends within the HAL timeout + 2 ticks (+ DBP and PWR clock steps before the wait: measured 5003)
	CHECK((RCC->BDCR & BDCR_LSERDY) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC7 characterization: LSE OFF whose LSERDY never clears returns HAL_TIMEOUT") {
	// NEEDS REVIEW: UM1725 gives no return codes; ticks waited: more than LSE_STARTUP_TIMEOUT (5000).
	REQUIRE(oscLse(RCC_LSE_ON) == HAL_OK);
	hostRccStickyReady(HOST_OSC_LSE, 1);
	const uint32_t t0 = hostTickPeek();
	CHECK(oscLse(RCC_LSE_OFF) == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 <= 5002u); // characterization: the loop ends within the HAL timeout + 2 ticks
	CHECK(hostTickPeek() - t0 > 5000u);
}

// ---------------------------------------------------------------- RC8 PLL

TEST_CASE_FIXTURE(HalFixture, "RC8 the firmware PLL configuration writes the PLLCFGR fields at their RM0090 positions") {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	REQUIRE(oscPllFirmware() == HAL_OK);
	// PLLSRC bit 22 = 1 (HSE) 0x00400000; PLLM 4 = 0x4; PLLN 168 << 6 = 0x2A00; PLLP /2 = 00; PLLQ 7 << 24 = 0x07000000.
	CHECK(pllcfgr() == 0x07402A04u);
}

TEST_CASE_FIXTURE(HalFixture, "RC8 an HSI-sourced PLL configuration clears PLLSRC and writes M 16, N 336, Q 7") {
	REQUIRE(oscPll(RCC_PLL_ON, RCC_PLLSOURCE_HSI, 16u, 336u, RCC_PLLP_DIV2, 7u) == HAL_OK);
	// PLLSRC 0 (HSI); PLLM 16 = 0x10; PLLN 336 << 6 = 0x5400; PLLP /2 = 00; PLLQ 7 << 24 = 0x07000000.
	CHECK(pllcfgr() == 0x07005410u);
}

TEST_CASE_FIXTURE(HalFixture, "RC8 PLLP 2, 4, 6 and 8 are written as the codes 00, 01, 10 and 11") {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	const uint32_t p[4] = {RCC_PLLP_DIV2, RCC_PLLP_DIV4, RCC_PLLP_DIV6, RCC_PLLP_DIV8};
	const uint32_t code[4] = {0u, 1u, 2u, 3u}; // RM0090 7.3.2
	for (int i = 0; i < 4; ++i) {
		INFO("P = " << p[i]);
		REQUIRE(oscPll(RCC_PLL_ON, RCC_PLLSOURCE_HSE, 8u, 336u, p[i], 7u) == HAL_OK);
		CHECK(((RCC->PLLCFGR >> 16) & 3u) == code[i]);
	}
}

TEST_CASE_FIXTURE(HalFixture, "RC8 PLLM is written to bits 5:0 for 2, 3 and 63") {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	const uint32_t m[3] = {2u, 3u, 63u};
	for (int i = 0; i < 3; ++i) {
		INFO("M = " << m[i]);
		REQUIRE(oscPll(RCC_PLL_ON, RCC_PLLSOURCE_HSE, m[i], 336u, RCC_PLLP_DIV2, 7u) == HAL_OK);
		CHECK((RCC->PLLCFGR & 0x3Fu) == m[i]);
	}
}

TEST_CASE_FIXTURE(HalFixture, "RC8 PLLN is written to bits 14:6 for 50, 192 and 432") {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	const uint32_t n[3] = {50u, 192u, 432u};
	for (int i = 0; i < 3; ++i) {
		INFO("N = " << n[i]);
		REQUIRE(oscPll(RCC_PLL_ON, RCC_PLLSOURCE_HSE, 8u, n[i], RCC_PLLP_DIV2, 7u) == HAL_OK);
		CHECK(((RCC->PLLCFGR >> 6) & 0x1FFu) == n[i]);
	}
}

TEST_CASE_FIXTURE(HalFixture, "RC8 PLLQ is written to bits 27:24 for 2, 7 and 15") {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	const uint32_t q[3] = {2u, 7u, 15u};
	for (int i = 0; i < 3; ++i) {
		INFO("Q = " << q[i]);
		REQUIRE(oscPll(RCC_PLL_ON, RCC_PLLSOURCE_HSE, 8u, 336u, RCC_PLLP_DIV2, q[i]) == HAL_OK);
		CHECK(((RCC->PLLCFGR >> 24) & 0xFu) == q[i]);
	}
}

TEST_CASE_FIXTURE(HalFixture, "RC8 PLL ON sets PLLON and PLLRDY and returns HAL_OK") {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	CHECK(oscPllFirmware() == HAL_OK);
	CHECK((RCC->CR & (CR_PLLON | CR_PLLRDY)) == (CR_PLLON | CR_PLLRDY));
}

TEST_CASE_FIXTURE(HalFixture, "RC8 PLL ON waits until the PLL is locked") {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	hostRccSetStartupTicks(HOST_OSC_PLL, 2u);
	const uint32_t t0 = hostTickPeek();
	CHECK(oscPllFirmware() == HAL_OK);
	CHECK((RCC->CR & CR_PLLRDY) != 0u);
	CHECK(hostTickPeek() - t0 >= 2u);
	CHECK(hostTickPeek() - t0 <= 4u); // the wait ends with the start-up time (+2 ticks)
}

TEST_CASE_FIXTURE(HalFixture, "RC8 PLL OFF clears PLLON and PLLRDY and returns HAL_OK") {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	REQUIRE(oscPllFirmware() == HAL_OK);
	CHECK(oscPll(RCC_PLL_OFF, RCC_PLLSOURCE_HSE, 4u, 168u, RCC_PLLP_DIV2, 7u) == HAL_OK);
	CHECK((RCC->CR & (CR_PLLON | CR_PLLRDY)) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC8 reconfiguring a running PLL that is not the system clock writes the new fields") {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	REQUIRE(oscPllFirmware() == HAL_OK);
	CHECK(oscPll(RCC_PLL_ON, RCC_PLLSOURCE_HSE, 8u, 336u, RCC_PLLP_DIV4, 14u) == HAL_OK);
	// PLLSRC 0x00400000; PLLM 8; PLLN 336 << 6 = 0x5400; PLLP /4 = 01 << 16 = 0x10000; PLLQ 14 << 24 = 0x0E000000.
	CHECK(pllcfgr() == 0x0E415408u);
	CHECK((RCC->CR & (CR_PLLON | CR_PLLRDY)) == (CR_PLLON | CR_PLLRDY));
}

TEST_CASE_FIXTURE(HalFixture, "RC8 PLL state NONE leaves the PLL and PLLCFGR unchanged") {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	REQUIRE(oscPllFirmware() == HAL_OK);
	const uint32_t cfg = RCC->PLLCFGR;
	CHECK(oscPll(RCC_PLL_NONE, RCC_PLLSOURCE_HSE, 8u, 100u, RCC_PLLP_DIV8, 3u) == HAL_OK);
	CHECK(RCC->PLLCFGR == cfg);
	CHECK((RCC->CR & CR_PLLON) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC8 the firmware PLL configuration gives VCO 336 MHz, PLLCLK 168 MHz and a 48 MHz PLLQ clock") {
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	REQUIRE(oscPllFirmware() == HAL_OK);
	const uint32_t cfg = RCC->PLLCFGR;
	const uint32_t m = cfg & 0x3Fu, n = (cfg >> 6) & 0x1FFu, p = (((cfg >> 16) & 3u) + 1u) * 2u, q = (cfg >> 24) & 0xFu;
	const uint32_t hse = 8000000u; // HSE_VALUE of the hal_conf (8 MHz crystal)
	const uint32_t vco = hse / m * n; // RM0090 7.3.2: VCO = HSE / PLLM * PLLN = 8 MHz / 4 * 168
	CHECK(vco == 336000000u);
	CHECK(vco / p == 168000000u); // PLLCLK = VCO / PLLP
	CHECK(vco / q == 48000000u);  // USB / SDIO / RNG clock = VCO / PLLQ = 336 MHz / 7
}

TEST_CASE_FIXTURE(HalFixture, "RC8 HAL_RCC_GetSysClockFreq returns 168 MHz with the PLL of the firmware configuration selected") {
	pllIsSysclk();
	CHECK(HAL_RCC_GetSysClockFreq() == 168000000u);
}

TEST_CASE_FIXTURE(HalFixture, "RC8 characterization: PLL that never gets ready returns HAL_TIMEOUT") {
	// NEEDS REVIEW: UM1725 gives no return codes.
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	hostRccFail(HOST_OSC_PLL, 1);
	const uint32_t t0 = hostTickPeek();
	CHECK(oscPllFirmware() == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 > 2u);
	CHECK(hostTickPeek() - t0 <= 6u); // characterization: the loop ends within the HAL timeout + 2 ticks (+ the PLL-off wait before the enable: measured 5)
	CHECK((RCC->CR & CR_PLLRDY) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC8 characterization: PLL with the HSE as source returns HAL_TIMEOUT while the HSE is off") {
	// NEEDS REVIEW: the PLL does not lock without its input clock (RM0090 7.3.2 asks to enable the source first);
	// UM1725 gives no return codes.
	const uint32_t t0 = hostTickPeek();
	CHECK(oscPllFirmware() == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 > 2u);
	CHECK(hostTickPeek() - t0 <= 6u); // characterization: the loop ends within the HAL timeout + 2 ticks (+ the PLL-off wait before the enable: measured 5)
	CHECK((RCC->CR & CR_PLLRDY) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC8 characterization: PLL OFF whose PLLRDY never clears returns HAL_TIMEOUT") {
	// NEEDS REVIEW: UM1725 gives no return codes; ticks waited: more than the 2 ms PLL timeout.
	REQUIRE(oscHse(RCC_HSE_ON) == HAL_OK);
	REQUIRE(oscPllFirmware() == HAL_OK);
	hostRccStickyReady(HOST_OSC_PLL, 1);
	const uint32_t t0 = hostTickPeek();
	CHECK(oscPll(RCC_PLL_OFF, RCC_PLLSOURCE_HSE, 4u, 168u, RCC_PLLP_DIV2, 7u) == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 <= 4u); // characterization: the loop ends within the HAL timeout + 2 ticks
	CHECK(hostTickPeek() - t0 > 2u);
}

// ---------------------------------------------------------------- RC9 PLL in use

TEST_CASE_FIXTURE(HalFixture, "RC9 PLL OFF is refused while the PLL is the system clock") {
	pllIsSysclk();
	const uint32_t cfg = RCC->PLLCFGR;
	oscPll(RCC_PLL_OFF, RCC_PLLSOURCE_HSE, 4u, 168u, RCC_PLLP_DIV2, 7u);
	CHECK((RCC->CR & CR_PLLON) != 0u);
	CHECK((RCC->CR & CR_PLLRDY) != 0u);
	CHECK(RCC->PLLCFGR == cfg);
	CHECK(sws() == 2u);
}

TEST_CASE_FIXTURE(HalFixture, "RC9 a different PLL configuration is not applied while the PLL is the system clock") {
	pllIsSysclk();
	const uint32_t cfg = RCC->PLLCFGR;
	oscPll(RCC_PLL_ON, RCC_PLLSOURCE_HSE, 8u, 100u, RCC_PLLP_DIV4, 5u);
	CHECK(RCC->PLLCFGR == cfg);
	CHECK((RCC->CR & (CR_PLLON | CR_PLLRDY)) == (CR_PLLON | CR_PLLRDY));
	CHECK(sws() == 2u);
}

TEST_CASE_FIXTURE(HalFixture, "RC9 a different PLL source is not applied while the PLL is the system clock") {
	pllIsSysclk();
	const uint32_t cfg = RCC->PLLCFGR;
	oscPll(RCC_PLL_ON, RCC_PLLSOURCE_HSI, 4u, 168u, RCC_PLLP_DIV2, 7u);
	CHECK(RCC->PLLCFGR == cfg);
	CHECK((RCC->CR & CR_PLLON) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RC9 characterization: PLL OFF while the PLL is the system clock returns HAL_ERROR") {
	// NEEDS REVIEW: UM1725 only says the PLL is not disabled; it gives no return codes.
	pllIsSysclk();
	CHECK(oscPll(RCC_PLL_OFF, RCC_PLLSOURCE_HSE, 4u, 168u, RCC_PLLP_DIV2, 7u) == HAL_ERROR);
}

TEST_CASE_FIXTURE(HalFixture, "RC9 characterization: a different PLL configuration while the PLL is the system clock returns HAL_ERROR") {
	// NEEDS REVIEW: UM1725 gives no return codes.
	pllIsSysclk();
	CHECK(oscPll(RCC_PLL_ON, RCC_PLLSOURCE_HSE, 8u, 100u, RCC_PLLP_DIV4, 5u) == HAL_ERROR);
}

TEST_CASE_FIXTURE(HalFixture, "RC9 characterization: repeating the running PLL configuration while it is the system clock returns HAL_OK") {
	// NEEDS REVIEW: UM1725 gives no return codes; the PLL stays on and PLLCFGR does not change.
	pllIsSysclk();
	const uint32_t cfg = RCC->PLLCFGR;
	CHECK(oscPllFirmware() == HAL_OK);
	CHECK(RCC->PLLCFGR == cfg);
	CHECK((RCC->CR & CR_PLLON) != 0u);
}

} // TEST_SUITE
