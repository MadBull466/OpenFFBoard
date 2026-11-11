/*
 * P12 T2 group RX: RCC extended driver (stm32f4xx_hal_rcc_ex.c, STM32F405/407 branches) against the RCC / PWR behavioural
 * model (fakes/stm32_hal/HalClockPower.h). Requirements RX1-RX4 of Tests/Code_Coverage/claude/charters/hal/P12_T2_REQUIREMENTS.md.
 * Built into ffb_hal_tests (F407VG) and ffb_hal_tests_disco (F407VG_DISCO).
 *
 * Oracles: UM1725 Rev 8 chapter RCCEx (PeriphCLKConfig note: selecting the RTC clock source resets the backup domain;
 * GetPeriphCLKFreq: "Return 0 if peripheral clock identifier not managed by this API"), RM0090 Rev 22 (RCC chapter for
 * the F405/407: RCC_CR, RCC_CFGR, RCC_BDCR, RCC_PLLI2SCFGR, PWR_CR, PWR_CSR). Encodings used in the derivations:
 *  - RCC_CR (RCC 0x00): PLLI2SON bit 26, PLLI2SRDY bit 27.
 *  - RCC_PLLCFGR (0x04): PLLM bits 5:0, PLLSRC bit 22 (0 = HSI, 1 = HSE).
 *  - RCC_CFGR (0x08): I2SSRC bit 23 (0 = PLLI2S, 1 = external I2S_CKIN); RTCPRE bits 20:16 (00010 = HSE/2 ... 11111 = HSE/31).
 *  - RCC_APB1ENR (0x40): PWREN bit 28.
 *  - RCC_BDCR (0x70): LSEON bit 0, LSERDY bit 1, RTCSEL bits 9:8 (00 none, 01 LSE, 10 LSI, 11 HSE / RTCPRE),
 *    RTCEN bit 15, BDRST bit 16. RTCSEL is writable once until a backup domain reset.
 *  - RCC_PLLI2SCFGR (0x84): PLLI2SN bits 14:6, PLLI2SR bits 30:28.
 *  - PWR_CR: DBP bit 8. PWR_CSR: BRE bit 9 (reset by a backup domain reset, RM0090 PWR_CSR).
 *  - Formulas (RM0090 PLLI2SCFGR): f(VCO) = f(input) * PLLI2SN / PLLM, f(I2S clock) = f(VCO) / PLLI2SR; the input is HSE
 *    or HSI as PLLSRC says. HSE_VALUE 8 MHz, HSI_VALUE 16 MHz, EXTERNAL_CLOCK_VALUE 12.288 MHz (stm32f4xx_hal_conf.h).
 * The HAL macro constants (RCC_RTCCLKSOURCE_*, RCC_PERIPHCLK_*) are inputs only.
 *
 * The cases that observe the backup domain reset do so through PWR_CSR.BRE (cleared by a backup domain reset, RM0090 PWR_CSR);
 * BRE can only be written with DBP = 1, which the HAL call sets itself (the tests set BRE raw with poke).
 *
 * Not covered: RTC frequency through GetPeriphCLKFreq (this device's branch manages only RCC_PERIPHCLK_I2S, UM1725), a
 * DisablePLLI2S timeout (the model clears PLLI2SRDY at once when PLLI2SON is cleared).
 */
#include "HalFixture.h"
#include "HalClockPower.h"

#include <cstdint>

namespace {

constexpr uint32_t CR_PLLI2SON = 1u << 26;
constexpr uint32_t CR_PLLI2SRDY = 1u << 27;
constexpr uint32_t CFGR_I2SSRC = 1u << 23;
constexpr uint32_t BDCR_LSEON = 1u << 0;
constexpr uint32_t BDCR_LSERDY = 1u << 1;
constexpr uint32_t BDCR_RTCEN = 1u << 15;
constexpr uint32_t BDCR_BDRST = 1u << 16;
constexpr uint32_t PWR_CR_DBP_BIT = 1u << 8;
constexpr uint32_t PWR_CSR_BRE_BIT = 1u << 9;
constexpr uint32_t APB1ENR_PWREN = 1u << 28;

uint32_t rtcsel() { return (RCC->BDCR >> 8) & 3u; }
uint32_t rtcpre() { return (RCC->CFGR >> 16) & 0x1Fu; }
uint32_t i2sN() { return (RCC->PLLI2SCFGR >> 6) & 0x1FFu; }
uint32_t i2sR() { return (RCC->PLLI2SCFGR >> 28) & 7u; }

HAL_StatusTypeDef selectRtc(uint32_t source) {
	RCC_PeriphCLKInitTypeDef c{};
	c.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	c.RTCClockSelection = source;
	return HAL_RCCEx_PeriphCLKConfig(&c);
}

HAL_StatusTypeDef configI2s(uint32_t n, uint32_t r) {
	RCC_PeriphCLKInitTypeDef c{};
	c.PeriphClockSelection = RCC_PERIPHCLK_I2S;
	c.PLLI2S.PLLI2SN = n;
	c.PLLI2S.PLLI2SR = r;
	return HAL_RCCEx_PeriphCLKConfig(&c);
}

HAL_StatusTypeDef enableI2sPll(uint32_t n, uint32_t r) {
	RCC_PLLI2SInitTypeDef p{};
	p.PLLI2SN = n;
	p.PLLI2SR = r;
	return HAL_RCCEx_EnablePLLI2S(&p);
}

/* PLLM (bits 5:0) and PLLSRC (bit 22) of RCC_PLLCFGR, other bits as at reset. */
void setPllInput(bool hse, uint32_t m) {
	HalFixture::poke(&RCC->PLLCFGR, (0x24003010u & ~((1u << 22) | 0x3Fu)) | (hse ? (1u << 22) : 0u) | m);
}

void setPlli2sCfg(uint32_t n, uint32_t r) { HalFixture::poke(&RCC->PLLI2SCFGR, (n << 6) | (r << 28)); }

void setBackupRegulatorEnable() { HalFixture::poke(&PWR->CSR, HalFixture::peek(&PWR->CSR) | PWR_CSR_BRE_BIT); }

} // namespace

TEST_SUITE("P12 T2 RCC_Ex") {

// ---- RX1: RTC clock source ----

TEST_CASE_FIXTURE(HalFixture, "RX1 selecting LSE as RTC clock gives RTCSEL 01 and returns HAL_OK") {
	CHECK(selectRtc(RCC_RTCCLKSOURCE_LSE) == HAL_OK);
	CHECK(rtcsel() == 1u); // RM0090 RTCSEL: 01 = LSE
}

TEST_CASE_FIXTURE(HalFixture, "RX1 selecting LSI as RTC clock gives RTCSEL 10 and returns HAL_OK") {
	CHECK(selectRtc(RCC_RTCCLKSOURCE_LSI) == HAL_OK);
	CHECK(rtcsel() == 2u); // RM0090 RTCSEL: 10 = LSI
}

TEST_CASE_FIXTURE(HalFixture, "RX1 selecting HSE as RTC clock gives RTCSEL 11 and the RTCPRE division") {
	uint32_t source = 0, prescaler = 0;
	SUBCASE("divided by 2") { source = RCC_RTCCLKSOURCE_HSE_DIV2; prescaler = 2u; }   // RTCPRE 00010 = HSE/2
	SUBCASE("divided by 8") { source = RCC_RTCCLKSOURCE_HSE_DIV8; prescaler = 8u; }   // RTCPRE 01000 = HSE/8
	SUBCASE("divided by 31") { source = RCC_RTCCLKSOURCE_HSE_DIV31; prescaler = 31u; } // RTCPRE 11111 = HSE/31
	CHECK(selectRtc(source) == HAL_OK);
	CHECK(rtcsel() == 3u); // RM0090 RTCSEL: 11 = HSE / RTCPRE
	CHECK(rtcpre() == prescaler);
}

TEST_CASE_FIXTURE(HalFixture, "RX1 an RTC selection enables the power interface clock and backup domain write access") {
	REQUIRE((peek(&PWR->CR) & PWR_CR_DBP_BIT) == 0u);
	CHECK(selectRtc(RCC_RTCCLKSOURCE_LSI) == HAL_OK);
	CHECK((RCC->APB1ENR & APB1ENR_PWREN) != 0u); // PWR registers need the PWR clock
	CHECK((PWR->CR & PWR_CR_DBP_BIT) != 0u);     // BDCR is write protected until DBP is 1 (RM0090 RCC_BDCR)
}

TEST_CASE_FIXTURE(HalFixture, "RX1 changing the RTC source from LSI to LSE selects LSE") {
	REQUIRE(selectRtc(RCC_RTCCLKSOURCE_LSI) == HAL_OK);
	CHECK(selectRtc(RCC_RTCCLKSOURCE_LSE) == HAL_OK);
	CHECK(rtcsel() == 1u); // RTCSEL is write once: only a backup domain reset lets it change
}

TEST_CASE_FIXTURE(HalFixture, "RX1 changing the RTC source resets the backup domain (backup regulator enable is cleared)") {
	REQUIRE(selectRtc(RCC_RTCCLKSOURCE_LSI) == HAL_OK);
	setBackupRegulatorEnable();
	REQUIRE((PWR->CSR & PWR_CSR_BRE_BIT) != 0u);
	CHECK(selectRtc(RCC_RTCCLKSOURCE_HSE_DIV8) == HAL_OK);
	CHECK((PWR->CSR & PWR_CSR_BRE_BIT) == 0u); // RM0090 PWR_CSR: BRE is reset by a backup domain reset
}

TEST_CASE_FIXTURE(HalFixture, "RX1 the backup domain reset is released again after an RTC source change") {
	REQUIRE(selectRtc(RCC_RTCCLKSOURCE_LSI) == HAL_OK);
	REQUIRE(selectRtc(RCC_RTCCLKSOURCE_LSE) == HAL_OK);
	CHECK((RCC->BDCR & BDCR_BDRST) == 0u); // BDRST held at 1 would keep RTCSEL at 00
}

TEST_CASE_FIXTURE(HalFixture, "RX1 changing only the HSE prescaler updates RTCPRE and keeps the HSE selection") {
	REQUIRE(selectRtc(RCC_RTCCLKSOURCE_HSE_DIV8) == HAL_OK);
	CHECK(selectRtc(RCC_RTCCLKSOURCE_HSE_DIV5) == HAL_OK);
	CHECK(rtcsel() == 3u);
	CHECK(rtcpre() == 5u); // 00101 = HSE/5
}

TEST_CASE_FIXTURE(HalFixture, "RX1 lowering the HSE prescaler to 2 replaces the whole RTCPRE field") {
	uint32_t from = 0;
	SUBCASE("from 31") { from = RCC_RTCCLKSOURCE_HSE_DIV31; } // RTCPRE 11111
	SUBCASE("from 17") { from = RCC_RTCCLKSOURCE_HSE_DIV17; } // RTCPRE 10001
	REQUIRE(selectRtc(from) == HAL_OK);
	setBackupRegulatorEnable(); // a backup domain reset would clear it: the source (HSE) does not change
	CHECK(selectRtc(RCC_RTCCLKSOURCE_HSE_DIV2) == HAL_OK);
	CHECK(rtcsel() == 3u);
	CHECK(rtcpre() == 2u); // 00010: every bit of the field written
	CHECK((PWR->CSR & PWR_CSR_BRE_BIT) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RX1 an RTC selection leaves the PLLI2S off") {
	REQUIRE(selectRtc(RCC_RTCCLKSOURCE_LSE) == HAL_OK);
	CHECK((RCC->CR & CR_PLLI2SON) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RX1 one call with I2S and RTC configures both") {
	RCC_PeriphCLKInitTypeDef c{};
	c.PeriphClockSelection = RCC_PERIPHCLK_I2S | RCC_PERIPHCLK_RTC;
	c.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	c.PLLI2S.PLLI2SN = 192u;
	c.PLLI2S.PLLI2SR = 2u;
	CHECK(HAL_RCCEx_PeriphCLKConfig(&c) == HAL_OK);
	CHECK(rtcsel() == 2u);
	CHECK(i2sN() == 192u);
	CHECK(i2sR() == 2u);
	CHECK((RCC->CR & CR_PLLI2SRDY) != 0u);
}

/*
 * characterization: the RTC source is not re-selected needlessly: selecting the source that is already selected does not
 * reset the backup domain (the backup regulator enable survives). UM1725 says only that "selecting the RTC clock source"
 * resets the backup domain, and RM0090 needs a reset only to change RTCSEL.
 * NEEDS REVIEW text: HAL_RCCEx_PeriphCLKConfig with the already selected RTC source does not reset the backup domain
 * (UM1725 note read literally would reset it on every call; the HAL resets only when RTCSEL changes).
 */
TEST_CASE_FIXTURE(HalFixture, "RX1 characterization: re-selecting the same RTC source does not reset the backup domain") {
	REQUIRE(selectRtc(RCC_RTCCLKSOURCE_LSI) == HAL_OK);
	setBackupRegulatorEnable();
	CHECK(selectRtc(RCC_RTCCLKSOURCE_LSI) == HAL_OK);
	CHECK((PWR->CSR & PWR_CSR_BRE_BIT) != 0u);
	CHECK(rtcsel() == 2u);
}

/*
 * characterization: the first selection, from RTCSEL = 00 (no clock), does not reset the backup domain either.
 * NEEDS REVIEW text: the first RTC source selection (RTCSEL 00 to any source) does not reset the backup domain, because
 * RM0090 lets RTCSEL be written while it is 00; UM1725 note literally says the backup domain is reset.
 */
TEST_CASE_FIXTURE(HalFixture, "RX1 characterization: the first RTC source selection does not reset the backup domain") {
	setBackupRegulatorEnable(); // raw setup, RTCSEL is 00 after reset
	REQUIRE(rtcsel() == 0u);
	CHECK(selectRtc(RCC_RTCCLKSOURCE_LSE) == HAL_OK);
	CHECK(rtcsel() == 1u);
	CHECK((PWR->CSR & PWR_CSR_BRE_BIT) != 0u);
}

/*
 * characterization: the HAL stores BDCR before the reset and restores it, so RTCEN set before a source change is still
 * set afterwards. UM1725 note: "RCC_BDCR register are set to their reset values" (RTCEN would be 0).
 * NEEDS REVIEW text: after an RTC source change HAL_RCCEx_PeriphCLKConfig restores the old BDCR content (RTCEN,
 * LSEON, LSEBYP), UM1725 says BDCR is set to its reset values; decide which is intended.
 */
TEST_CASE_FIXTURE(HalFixture, "RX1 characterization: RTCEN set before a source change is still set afterwards") {
	REQUIRE(selectRtc(RCC_RTCCLKSOURCE_LSI) == HAL_OK);
	RCC->BDCR |= BDCR_RTCEN; // DBP was set by the call above
	REQUIRE((RCC->BDCR & BDCR_RTCEN) != 0u);
	CHECK(selectRtc(RCC_RTCCLKSOURCE_LSE) == HAL_OK);
	CHECK((RCC->BDCR & BDCR_RTCEN) != 0u);
}

/*
 * characterization: LSE enabled before a source change is restored and awaited: the call returns after LSERDY.
 * NEEDS REVIEW text: HAL_RCCEx_PeriphCLKConfig restores LSEON after the backup domain reset and waits for LSERDY
 * (UM1725 does not describe the LSE handling).
 */
TEST_CASE_FIXTURE(HalFixture, "RX1 characterization: LSE enabled before a source change is ready again when the call returns") {
	REQUIRE(selectRtc(RCC_RTCCLKSOURCE_LSI) == HAL_OK);
	RCC->BDCR |= BDCR_LSEON;
	hostRccSetStartupTicks(HOST_OSC_LSE, 3u);
	CHECK(selectRtc(RCC_RTCCLKSOURCE_LSE) == HAL_OK);
	CHECK((RCC->BDCR & BDCR_LSEON) != 0u);
	CHECK((RCC->BDCR & BDCR_LSERDY) != 0u);
}

/*
 * characterization: an LSE that never gets ready after the source change gives HAL_TIMEOUT and the call returns
 * without selecting the new source (RTCSEL stays 00 after the reset).
 * NEEDS REVIEW text: LSE start-up timeout in HAL_RCCEx_PeriphCLKConfig returns HAL_TIMEOUT and leaves RTCSEL at 00
 * (the backup domain is already reset, the RTC has no clock); UM1725 gives no return code.
 */
TEST_CASE_FIXTURE(HalFixture, "RX1 characterization: an LSE that never gets ready makes the source change time out") {
	REQUIRE(selectRtc(RCC_RTCCLKSOURCE_LSI) == HAL_OK);
	RCC->BDCR |= BDCR_LSEON;
	hostRccFail(HOST_OSC_LSE, 1);
	CHECK(selectRtc(RCC_RTCCLKSOURCE_HSE_DIV8) == HAL_TIMEOUT);
	CHECK(rtcsel() == 0u);
}

// ---- RX2: I2S clock source ----

TEST_CASE_FIXTURE(HalFixture, "RX2 selecting the external clock sets I2SSRC") {
	REQUIRE((RCC->CFGR & CFGR_I2SSRC) == 0u);
	__HAL_RCC_I2S_CONFIG(RCC_I2SCLKSOURCE_EXT);
	CHECK((RCC->CFGR & CFGR_I2SSRC) != 0u); // RM0090 I2SSRC 1 = external clock on I2S_CKIN
}

TEST_CASE_FIXTURE(HalFixture, "RX2 selecting the PLLI2S clears I2SSRC") {
	poke(&RCC->CFGR, CFGR_I2SSRC);
	__HAL_RCC_I2S_CONFIG(RCC_I2SCLKSOURCE_PLLI2S);
	CHECK((RCC->CFGR & CFGR_I2SSRC) == 0u); // RM0090 I2SSRC 0 = PLLI2S
}

TEST_CASE_FIXTURE(HalFixture, "RX2 selecting the I2S clock source leaves the other CFGR bits unchanged") {
	poke(&RCC->CFGR, 0x00050000u); // RTCPRE = 5
	__HAL_RCC_I2S_CONFIG(RCC_I2SCLKSOURCE_EXT);
	CHECK(RCC->CFGR == (0x00050000u | CFGR_I2SSRC));
}

/*
 * characterization: HAL_RCCEx_PeriphCLKConfig for I2S configures the PLLI2S only; the F405/407 structure has no I2S
 * source field, so I2SSRC keeps its value.
 * NEEDS REVIEW text: HAL_RCCEx_PeriphCLKConfig(RCC_PERIPHCLK_I2S) never writes I2SSRC on F405/407; the source is chosen
 * with __HAL_RCC_I2S_CONFIG (UM1725 does not say).
 */
TEST_CASE_FIXTURE(HalFixture, "RX2 characterization: the I2S peripheral clock configuration leaves I2SSRC unchanged") {
	__HAL_RCC_I2S_CONFIG(RCC_I2SCLKSOURCE_EXT);
	CHECK(configI2s(192u, 2u) == HAL_OK);
	CHECK((RCC->CFGR & CFGR_I2SSRC) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RX2 the I2S peripheral clock configuration writes PLLI2SN and PLLI2SR and starts the PLLI2S") {
	CHECK(configI2s(258u, 3u) == HAL_OK);
	CHECK(i2sN() == 258u); // bits 14:6
	CHECK(i2sR() == 3u);   // bits 30:28
	CHECK((RCC->CR & CR_PLLI2SON) != 0u);
	CHECK((RCC->CR & CR_PLLI2SRDY) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RX2 the I2S peripheral clock configuration of a running PLLI2S programs the new factors") {
	REQUIRE(configI2s(192u, 2u) == HAL_OK);
	CHECK(configI2s(100u, 4u) == HAL_OK); // PLLI2SCFGR is writable only while the PLLI2S is off (RM0090): it is stopped first
	CHECK(i2sN() == 100u);
	CHECK(i2sR() == 4u);
	CHECK((RCC->CR & CR_PLLI2SRDY) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RX2 the PLLI2S peripheral identifier writes PLLI2SN and PLLI2SR and starts the PLLI2S") {
	RCC_PeriphCLKInitTypeDef c{};
	c.PeriphClockSelection = RCC_PERIPHCLK_PLLI2S;
	c.PLLI2S.PLLI2SN = 344u;
	c.PLLI2S.PLLI2SR = 6u;
	CHECK(HAL_RCCEx_PeriphCLKConfig(&c) == HAL_OK);
	CHECK(i2sN() == 344u);
	CHECK(i2sR() == 6u);
	CHECK((RCC->CR & CR_PLLI2SON) != 0u);
	CHECK((RCC->CR & CR_PLLI2SRDY) != 0u);
}

/*
 * characterization: the PLLI2S never gets ready during the I2S peripheral clock configuration: HAL_TIMEOUT.
 * NEEDS REVIEW text: HAL_RCCEx_PeriphCLKConfig(I2S) returns HAL_TIMEOUT when PLLI2SRDY does not rise; UM1725 gives no
 * return codes.
 */
TEST_CASE_FIXTURE(HalFixture, "RX2 characterization: the I2S peripheral clock configuration times out when the PLLI2S never gets ready") {
	hostRccFail(HOST_OSC_PLLI2S, 1);
	CHECK(configI2s(192u, 2u) == HAL_TIMEOUT);
}

// ---- RX3: EnablePLLI2S / DisablePLLI2S ----

TEST_CASE_FIXTURE(HalFixture, "RX3 EnablePLLI2S writes PLLI2SN and PLLI2SR at their positions") {
	uint32_t n = 0, r = 0;
	SUBCASE("reset values") { n = 192u; r = 2u; }
	SUBCASE("lowest N and R") { n = 50u; r = 2u; }
	SUBCASE("highest N and R") { n = 432u; r = 7u; }
	SUBCASE("mid values") { n = 258u; r = 3u; }
	REQUIRE(enableI2sPll(n, r) == HAL_OK);
	CHECK(i2sN() == n);
	CHECK(i2sR() == r);
	CHECK(peek(&RCC->PLLI2SCFGR) == ((n << 6) | (r << 28))); // RM0090: N 14:6, R 30:28, other bits 0 on F405/407
}

TEST_CASE_FIXTURE(HalFixture, "RX3 EnablePLLI2S sets PLLI2SON and waits for PLLI2SRDY") {
	hostRccSetStartupTicks(HOST_OSC_PLLI2S, 2u);
	CHECK(enableI2sPll(192u, 2u) == HAL_OK);
	CHECK((RCC->CR & CR_PLLI2SON) != 0u);
	CHECK((RCC->CR & CR_PLLI2SRDY) != 0u);
	CHECK(hostTickPeek() >= 2u); // the flag needs 2 ticks after PLLI2SON
}

TEST_CASE_FIXTURE(HalFixture, "RX3 EnablePLLI2S of a running PLLI2S programs the new factors") {
	REQUIRE(enableI2sPll(192u, 2u) == HAL_OK);
	CHECK(enableI2sPll(100u, 4u) == HAL_OK); // PLLI2SCFGR can only be written while PLLI2S is off (RM0090)
	CHECK(i2sN() == 100u);
	CHECK(i2sR() == 4u);
	CHECK((RCC->CR & CR_PLLI2SRDY) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RX3 EnablePLLI2S does not change the main PLL configuration") {
	const uint32_t before = peek(&RCC->PLLCFGR);
	REQUIRE(enableI2sPll(192u, 2u) == HAL_OK);
	CHECK(peek(&RCC->PLLCFGR) == before);
}

/*
 * characterization: EnablePLLI2S returns HAL_TIMEOUT when PLLI2SRDY never rises.
 * NEEDS REVIEW text: HAL_RCCEx_EnablePLLI2S returns HAL_TIMEOUT when the PLLI2S never gets ready; UM1725 gives no
 * return codes.
 */
TEST_CASE_FIXTURE(HalFixture, "RX3 characterization: EnablePLLI2S times out when the PLLI2S never gets ready") {
	hostRccFail(HOST_OSC_PLLI2S, 1);
	CHECK(enableI2sPll(192u, 2u) == HAL_TIMEOUT);
}

TEST_CASE_FIXTURE(HalFixture, "RX3 DisablePLLI2S clears PLLI2SON and PLLI2SRDY") {
	REQUIRE(enableI2sPll(192u, 2u) == HAL_OK);
	CHECK(HAL_RCCEx_DisablePLLI2S() == HAL_OK);
	CHECK((RCC->CR & CR_PLLI2SON) == 0u);
	CHECK((RCC->CR & CR_PLLI2SRDY) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RX3 DisablePLLI2S of a stopped PLLI2S returns HAL_OK") {
	CHECK(HAL_RCCEx_DisablePLLI2S() == HAL_OK);
	CHECK((RCC->CR & CR_PLLI2SON) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "RX3 DisablePLLI2S keeps PLLI2SN and PLLI2SR") {
	REQUIRE(enableI2sPll(258u, 3u) == HAL_OK);
	REQUIRE(HAL_RCCEx_DisablePLLI2S() == HAL_OK);
	CHECK(i2sN() == 258u);
	CHECK(i2sR() == 3u);
}

/*
 * characterization: a PLLI2S whose ready flag does not clear after PLLI2SON is cleared makes DisablePLLI2S return
 * HAL_TIMEOUT after waiting (ticks elapsed). Model: hostRccStickyReady.
 * NEEDS REVIEW text: HAL_RCCEx_DisablePLLI2S returns HAL_TIMEOUT when PLLI2SRDY does not clear; UM1725 gives no return
 * codes.
 */
TEST_CASE_FIXTURE(HalFixture, "RX3 characterization: DisablePLLI2S times out when PLLI2SRDY does not clear") {
	REQUIRE(enableI2sPll(192u, 2u) == HAL_OK);
	hostRccStickyReady(HOST_OSC_PLLI2S, 1);
	const uint32_t t0 = hostTickPeek();
	CHECK(HAL_RCCEx_DisablePLLI2S() == HAL_TIMEOUT);
	CHECK(hostTickPeek() > t0); // it waited
	CHECK((RCC->CR & CR_PLLI2SON) == 0u);
}

/*
 * characterization: the PLLI2S disable wait inside the I2S PeriphCLKConfig times out the same way, before the factors
 * are written (PLLI2SCFGR keeps the old N).
 * NEEDS REVIEW text: HAL_RCCEx_PeriphCLKConfig(I2S) returns HAL_TIMEOUT when PLLI2SRDY does not clear after the
 * PLLI2S is disabled and then does not write the new factors; UM1725 gives no return codes.
 */
TEST_CASE_FIXTURE(HalFixture, "RX3 characterization: the I2S peripheral clock configuration times out when PLLI2SRDY does not clear") {
	REQUIRE(configI2s(192u, 2u) == HAL_OK);
	hostRccStickyReady(HOST_OSC_PLLI2S, 1);
	CHECK(configI2s(100u, 4u) == HAL_TIMEOUT);
	CHECK(i2sN() == 192u);
}

// ---- RX4: GetPeriphCLKConfig and GetPeriphCLKFreq ----
// UM1725 describes the GetPeriphCLKFreq result as "KHz"; the cases expect Hz, as the RM0090 formulas give with inputs in Hz.
// NEEDS REVIEW text: UM1725 wording "Frequency in KHz" vs RM0090 formulas / HAL value in Hz.

TEST_CASE_FIXTURE(HalFixture, "RX4 GetPeriphCLKConfig returns PLLI2SN and PLLI2SR of the register") {
	setPlli2sCfg(271u, 5u);
	RCC_PeriphCLKInitTypeDef c{};
	HAL_RCCEx_GetPeriphCLKConfig(&c);
	CHECK(c.PLLI2S.PLLI2SN == 271u);
	CHECK(c.PLLI2S.PLLI2SR == 5u);
}

TEST_CASE_FIXTURE(HalFixture, "RX4 GetPeriphCLKConfig returns the HSE selection with its RTCPRE prescaler") {
	poke(&RCC->CFGR, 10u << 16);  // RTCPRE = 01010 = HSE/10
	poke(&RCC->BDCR, 3u << 8);    // RTCSEL = 11 = HSE
	RCC_PeriphCLKInitTypeDef c{};
	HAL_RCCEx_GetPeriphCLKConfig(&c);
	CHECK(c.RTCClockSelection == 0x000A0300u); // RTCPRE 10 at bit 16, RTCSEL 3 at bit 8
}

TEST_CASE_FIXTURE(HalFixture, "RX4 GetPeriphCLKConfig returns the LSE selection") {
	poke(&RCC->BDCR, 1u << 8);
	RCC_PeriphCLKInitTypeDef c{};
	HAL_RCCEx_GetPeriphCLKConfig(&c);
	CHECK(c.RTCClockSelection == 0x00000100u); // RTCSEL 01 at bit 8
}

TEST_CASE_FIXTURE(HalFixture, "RX4 GetPeriphCLKConfig returns the LSI selection") {
	poke(&RCC->BDCR, 2u << 8);
	RCC_PeriphCLKInitTypeDef c{};
	HAL_RCCEx_GetPeriphCLKConfig(&c);
	CHECK(c.RTCClockSelection == 0x00000200u); // RTCSEL 10 at bit 8
}

TEST_CASE_FIXTURE(HalFixture, "RX4 GetPeriphCLKConfig returns no RTC clock after reset") {
	RCC_PeriphCLKInitTypeDef c{};
	HAL_RCCEx_GetPeriphCLKConfig(&c);
	CHECK(c.RTCClockSelection == 0u); // RTCSEL 00 and RTCPRE 0 at reset
}

TEST_CASE_FIXTURE(HalFixture, "RX4 GetPeriphCLKConfig returns what PeriphCLKConfig set for RTC and PLLI2S") {
	RCC_PeriphCLKInitTypeDef set{};
	set.PeriphClockSelection = RCC_PERIPHCLK_I2S | RCC_PERIPHCLK_RTC;
	set.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV17;
	set.PLLI2S.PLLI2SN = 344u;
	set.PLLI2S.PLLI2SR = 6u;
	REQUIRE(HAL_RCCEx_PeriphCLKConfig(&set) == HAL_OK);
	RCC_PeriphCLKInitTypeDef got{};
	HAL_RCCEx_GetPeriphCLKConfig(&got);
	CHECK(got.RTCClockSelection == RCC_RTCCLKSOURCE_HSE_DIV17); // RTCPRE 17 = 10001, RTCSEL 11
	CHECK(got.PLLI2S.PLLI2SN == 344u);
	CHECK(got.PLLI2S.PLLI2SR == 6u);
}

/*
 * characterization: GetPeriphCLKConfig reports both extended clocks (I2S and RTC) in PeriphClockSelection.
 * NEEDS REVIEW text: HAL_RCCEx_GetPeriphCLKConfig sets PeriphClockSelection to RCC_PERIPHCLK_I2S | RCC_PERIPHCLK_RTC on
 * F405/407; UM1725 says only that the structure is configured from the registers.
 */
TEST_CASE_FIXTURE(HalFixture, "RX4 characterization: GetPeriphCLKConfig reports the I2S and RTC clocks in PeriphClockSelection") {
	RCC_PeriphCLKInitTypeDef c{};
	HAL_RCCEx_GetPeriphCLKConfig(&c);
	CHECK(c.PeriphClockSelection == (RCC_PERIPHCLK_I2S | RCC_PERIPHCLK_RTC));
}

TEST_CASE_FIXTURE(HalFixture, "RX4 the I2S clock after reset with the HSI as PLL input is 96 MHz") {
	// Reset: PLLSRC 0 (HSI 16 MHz), PLLM 16, PLLI2SN 192, PLLI2SR 2. VCO = 16 MHz / 16 * 192 = 192 MHz, I2S = 192 / 2.
	CHECK(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2S) == 96000000u);
}

TEST_CASE_FIXTURE(HalFixture, "RX4 the I2S clock follows PLLM, PLLI2SN and PLLI2SR with the HSE as PLL input") {
	uint32_t m = 0, n = 0, r = 0, expected = 0;
	SUBCASE("M 8 N 192 R 2") { m = 8; n = 192; r = 2; expected = 96000000u; }   // 8 MHz / 8 = 1 MHz, * 192 = 192 MHz, / 2
	SUBCASE("M 4 N 100 R 5") { m = 4; n = 100; r = 5; expected = 40000000u; }   // 2 MHz * 100 = 200 MHz, / 5
	SUBCASE("M 2 N 50 R 4") { m = 2; n = 50; r = 4; expected = 50000000u; }     // 4 MHz * 50 = 200 MHz, / 4 (pure arithmetic: 4 MHz is outside the RM0090 1-2 MHz VCO input range)
	SUBCASE("M 8 N 432 R 3") { m = 8; n = 432; r = 3; expected = 144000000u; }  // 1 MHz * 432 = 432 MHz, / 3
	setPllInput(true, m);
	setPlli2sCfg(n, r);
	CHECK(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2S) == expected);
}

TEST_CASE_FIXTURE(HalFixture, "RX4 the I2S clock follows PLLM, PLLI2SN and PLLI2SR with the HSI as PLL input") {
	uint32_t m = 0, n = 0, r = 0, expected = 0;
	SUBCASE("M 16 N 258 R 3") { m = 16; n = 258; r = 3; expected = 86000000u; } // 16 MHz / 16 = 1 MHz, * 258 = 258 MHz, / 3
	SUBCASE("M 8 N 100 R 4") { m = 8; n = 100; r = 4; expected = 50000000u; }   // 2 MHz * 100 = 200 MHz, / 4
	setPllInput(false, m);
	setPlli2sCfg(n, r);
	CHECK(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2S) == expected);
}

TEST_CASE_FIXTURE(HalFixture, "RX4 the I2S clock is the fixed external frequency when I2SSRC selects I2S_CKIN") {
	poke(&RCC->CFGR, CFGR_I2SSRC);
	setPlli2sCfg(100u, 4u); // must have no effect
	CHECK(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2S) == 12288000u); // EXTERNAL_CLOCK_VALUE of stm32f4xx_hal_conf.h
}

TEST_CASE_FIXTURE(HalFixture, "RX4 the I2S clock after EnablePLLI2S matches the RM0090 formula") {
	setPllInput(false, 16u); // HSI 16 MHz / 16 = 1 MHz (the HSI is ready, so the PLLI2S can start)
	REQUIRE(enableI2sPll(258u, 3u) == HAL_OK);
	CHECK(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2S) == 86000000u); // 1 MHz * 258 / 3
}

TEST_CASE_FIXTURE(HalFixture, "RX4 the I2S clock with the firmware PLL input (HSE 8 MHz, PLLM 4) and N 192 R 2 is 192 MHz") {
	setPllInput(true, 4u); // 2 MHz
	setPlli2sCfg(192u, 2u); // VCO 384 MHz, / 2
	CHECK(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2S) == 192000000u);
}

TEST_CASE_FIXTURE(HalFixture, "RX4 GetPeriphCLKFreq returns 0 for the RTC identifier (not managed by this API)") {
	poke(&RCC->BDCR, 1u << 8); // LSE selected
	CHECK(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_RTC) == 0u); // UM1725: only RCC_PERIPHCLK_I2S is listed
}

TEST_CASE_FIXTURE(HalFixture, "RX4 GetPeriphCLKFreq returns 0 for an identifier it does not manage") {
	CHECK(HAL_RCCEx_GetPeriphCLKFreq(0u) == 0u);
	CHECK(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_PLLI2S) == 0u);
	CHECK(HAL_RCCEx_GetPeriphCLKFreq(0x80000000u) == 0u);
}

} // TEST_SUITE
