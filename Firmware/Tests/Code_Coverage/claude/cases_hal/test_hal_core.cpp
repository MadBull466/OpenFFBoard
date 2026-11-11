/*
 * P12 T1 HAL core cases (requirements H1-H10 of Tests/Code_Coverage/claude/charters/hal/P12_T1_REQUIREMENTS.md).
 * Sources under test: stm32f4xx_hal.c of the real ST HAL (the parts that do not touch NVIC / SCB / SysTick), built
 * into ffb_hal_tests and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 chapter 6, RM0090 Rev 22 (7.3.5-7.3.9, 9.2.7, 38.6.1, 38.16.3, 39.1), release notes V1.8.3.
 * Not compiled for STM32F407xx: HAL_EnableMemorySwappingBank / HAL_DisableMemorySwappingBank (F42x/43x/469/479 only).
 * Out (group C, needs the system control space): HAL_Init, HAL_InitTick, HAL_SetTickFreq, HAL_SuspendTick, HAL_ResumeTick.
 */
#include "HalFixture.h"

#include "../../support/TestHelpers.h"

#include <cstdint>

namespace {

// Writes to reg in the access log, in order.
struct Writes {
	uint32_t v[8];
	unsigned n;
};
Writes writesTo(volatile uint32_t *reg) {
	Writes w = {{0}, 0};
	const std::uintptr_t a = reinterpret_cast<std::uintptr_t>(reg);
	for (std::size_t i = 0; i < hostRegsLogCount(); ++i) {
		const HostRegAccess *e = hostRegsLogAt(i);
		if (e->write && e->addr == a && w.n < 8) w.v[w.n++] = e->written;
	}
	return w;
}

} // namespace

TEST_SUITE("P12 T1 HAL core") {

// ---- H1 ----

TEST_CASE_FIXTURE(HalFixture, "H1 HAL_GetHalVersion is 0x01080300 (V1.8.3)") {
	// UM1725 6.1.4: 0xXYZR, 8 bits each; release notes V1.8.3: X=1, Y=8, Z=3, R=0.
	CHECK(HAL_GetHalVersion() == 0x01080300u);
}

// ---- H2 ----

TEST_CASE_FIXTURE(HalFixture, "H2 HAL_GetDEVID returns IDCODE bits 11:0") {
	hostDbgSetIdcode(0x10070413u); // RM0090 38.6.1: DEV_ID 0x413
	CHECK(HAL_GetDEVID() == 0x413u);
	hostDbgSetIdcode(0x100F0431u); // DEV_ID 0x431 in bits 11:0 only
	CHECK(HAL_GetDEVID() == 0x431u);
}

TEST_CASE_FIXTURE(HalFixture, "H2 HAL_GetDEVID ignores IDCODE bits 15:12 (reserved)") {
	hostDbgSetIdcode(0x1007F413u); // RM0090 38.6.1: bits 15:12 reserved, DEV_ID is bits 11:0 = 0x413
	CHECK(HAL_GetDEVID() == 0x413u);
}

TEST_CASE_FIXTURE(HalFixture, "H2 HAL_GetREVID returns IDCODE bits 31:16") {
	hostDbgSetIdcode(0x10070413u); // REV_ID 0x1007 (38.6.1)
	CHECK(HAL_GetREVID() == 0x1007u);
	hostDbgSetIdcode(0x100F0413u);
	CHECK(HAL_GetREVID() == 0x100Fu);
}

// ---- H3 ----

TEST_CASE_FIXTURE(HalFixture, "H3 HAL_DBGMCU_EnableDBGSleepMode sets DBGMCU_CR bit 0 only") {
	poke(&DBGMCU->CR, 0xA5A5A5A0u); // bits 2:0 clear (RM0090 38.16.3: DBG_SLEEP = bit 0)
	HAL_DBGMCU_EnableDBGSleepMode();
	CHECK(peek(&DBGMCU->CR) == 0xA5A5A5A1u);
}

TEST_CASE_FIXTURE(HalFixture, "H3 HAL_DBGMCU_DisableDBGSleepMode clears DBGMCU_CR bit 0 only") {
	poke(&DBGMCU->CR, 0xA5A5A5A7u);
	HAL_DBGMCU_DisableDBGSleepMode();
	CHECK(peek(&DBGMCU->CR) == 0xA5A5A5A6u);
}

TEST_CASE_FIXTURE(HalFixture, "H3 HAL_DBGMCU_EnableDBGStopMode sets DBGMCU_CR bit 1 only") {
	poke(&DBGMCU->CR, 0xA5A5A5A0u); // DBG_STOP = bit 1
	HAL_DBGMCU_EnableDBGStopMode();
	CHECK(peek(&DBGMCU->CR) == 0xA5A5A5A2u);
}

TEST_CASE_FIXTURE(HalFixture, "H3 HAL_DBGMCU_DisableDBGStopMode clears DBGMCU_CR bit 1 only") {
	poke(&DBGMCU->CR, 0xA5A5A5A7u);
	HAL_DBGMCU_DisableDBGStopMode();
	CHECK(peek(&DBGMCU->CR) == 0xA5A5A5A5u);
}

TEST_CASE_FIXTURE(HalFixture, "H3 HAL_DBGMCU_EnableDBGStandbyMode sets DBGMCU_CR bit 2 only") {
	poke(&DBGMCU->CR, 0xA5A5A5A0u); // DBG_STANDBY = bit 2
	HAL_DBGMCU_EnableDBGStandbyMode();
	CHECK(peek(&DBGMCU->CR) == 0xA5A5A5A4u);
}

TEST_CASE_FIXTURE(HalFixture, "H3 HAL_DBGMCU_DisableDBGStandbyMode clears DBGMCU_CR bit 2 only") {
	poke(&DBGMCU->CR, 0xA5A5A5A7u);
	HAL_DBGMCU_DisableDBGStandbyMode();
	CHECK(peek(&DBGMCU->CR) == 0xA5A5A5A3u);
}

// ---- H4 ----
TEST_CASE_FIXTURE(HalFixture, "H4 HAL_EnableCompensationCell sets SYSCFG_CMPCR.CMP_PD") {
	poke(&SYSCFG->CMPCR, 0u); // RM0090 9.2.7: CMP_PD = bit 0
	HAL_EnableCompensationCell();
	CHECK((peek(&SYSCFG->CMPCR) & 1u) == 1u);
}

TEST_CASE_FIXTURE(HalFixture, "H4 HAL_DisableCompensationCell clears SYSCFG_CMPCR.CMP_PD") {
	poke(&SYSCFG->CMPCR, 1u);
	HAL_DisableCompensationCell();
	CHECK((peek(&SYSCFG->CMPCR) & 1u) == 0u);
}

// ---- H5 ----

TEST_CASE_FIXTURE(HalFixture, "H5 HAL_GetUIDw0, w1 and w2 return the words at 0x1FFF7A10, 0x1FFF7A14 and 0x1FFF7A18") {
	hostUidSet(0x11223344u, 0x55667788u, 0x99AABBCCu); // RM0090 39.1: three 32-bit words at those addresses
	CHECK(HAL_GetUIDw0() == 0x11223344u);
	CHECK(HAL_GetUIDw1() == 0x55667788u);
	CHECK(HAL_GetUIDw2() == 0x99AABBCCu);
}

// ---- H6 ----

TEST_CASE_FIXTURE(HalFixture, "H6 HAL_IncTick adds 1 ms to the tick at 1 kHz") {
	uwTickFreq = static_cast<HAL_TickFreqTypeDef>(1u);
	hostTickSet(100u);
	HAL_IncTick();
	CHECK(hostTickPeek() == 101u);
}

TEST_CASE_FIXTURE(HalFixture, "H6 HAL_IncTick adds 10 ms to the tick at 100 Hz") {
	uwTickFreq = static_cast<HAL_TickFreqTypeDef>(10u);
	hostTickSet(100u);
	HAL_IncTick();
	CHECK(hostTickPeek() == 110u);
}

TEST_CASE_FIXTURE(HalFixture, "H6 HAL_IncTick adds 100 ms to the tick at 10 Hz") {
	uwTickFreq = static_cast<HAL_TickFreqTypeDef>(100u);
	hostTickSet(100u);
	HAL_IncTick();
	CHECK(hostTickPeek() == 200u);
}

TEST_CASE_FIXTURE(HalFixture, "H6 HAL_IncTick wraps the 32-bit tick around") {
	uwTickFreq = static_cast<HAL_TickFreqTypeDef>(10u);
	hostTickSet(0xFFFFFFFBu); // 0xFFFFFFFB + 10 = 0x1_0000_0005
	HAL_IncTick();
	CHECK(hostTickPeek() == 5u);
}

TEST_CASE_FIXTURE(HalFixture, "H6 HAL_GetTickFreq returns the current tick frequency value") {
	uwTickFreq = static_cast<HAL_TickFreqTypeDef>(10u);
	CHECK(static_cast<uint32_t>(HAL_GetTickFreq()) == 10u);
	uwTickFreq = static_cast<HAL_TickFreqTypeDef>(100u);
	CHECK(static_cast<uint32_t>(HAL_GetTickFreq()) == 100u);
}

TEST_CASE_FIXTURE(HalFixture, "H6 HAL_GetTickPrio returns the current tick priority") {
	uwTickPrio = 7u;
	CHECK(HAL_GetTickPrio() == 7u);
	uwTickPrio = 0u;
	CHECK(HAL_GetTickPrio() == 0u);
}

// ---- H7 ----

TEST_CASE_FIXTURE(HalFixture, "H7 HAL_Delay returns after at least n ms at 1 kHz for n = 0, 1 and 7") {
	const uint32_t delays[] = {0u, 1u, 7u};
	for (uint32_t n : delays) {
		const uint32_t before = hostTickPeek();
		HAL_Delay(n);
		INFO("n = " << n);
		CHECK(hostTickPeek() - before >= n);
	}
}

TEST_CASE_FIXTURE(HalFixture, "H7 HAL_Delay returns after at least n ms at 100 Hz for n = 0, 1 and 7") {
	uwTickFreq = static_cast<HAL_TickFreqTypeDef>(10u);
	const uint32_t delays[] = {0u, 1u, 7u};
	for (uint32_t n : delays) {
		const uint32_t before = hostTickPeek();
		HAL_Delay(n);
		INFO("n = " << n);
		CHECK(hostTickPeek() - before >= n);
	}
}

TEST_CASE_FIXTURE(HalFixture, "H7 HAL_Delay counts the delay across the wrap-around of the tick") {
	hostTickSet(0xFFFFFFFCu);
	HAL_Delay(7u);
	CHECK(hostTickPeek() - 0xFFFFFFFCu >= 7u); // modulo 2^32
}

// ---- H8 ----

TEST_CASE_FIXTURE(HalFixture, "H8 HAL_Delay of a time close to HAL_MAX_DELAY waits the full time at 10 Hz" * KNOWN_ISSUE("candidate vendor bug V90: HAL_Delay adds uwTickFreq to a value below HAL_MAX_DELAY and the 32-bit sum wraps, so the delay ends after about 100 ms instead of the requested time")) {
	uwTickFreq = static_cast<HAL_TickFreqTypeDef>(100u);
	const uint32_t n = 0xFFFFFFF0u;
	const uint32_t before = hostTickPeek();
	HAL_Delay(n);
	const uint32_t elapsed = hostTickPeek() - before;
	CHECK(elapsed >= n); // UM1725 HAL_Delay: minimum delay in ms
}

// ---- H9 ----

TEST_CASE_FIXTURE(HalFixture, "H9 HAL_DeInit returns HAL_OK") {
	CHECK(HAL_DeInit() == HAL_OK);
}

// Bits RM0090 7.3.5-7.3.9 define for STM32F407 (all other bits reserved, "must be kept at reset value"):
//  AHB1RSTR: GPIOA..I bits 0-8 (0x1FF), CRC bit 12 (0x1000), DMA1/DMA2 bits 21/22 (0x600000), ETHMAC bit 25
//            (0x2000000), OTGHS bit 29 (0x20000000) = 0x226011FF.
//  AHB2RSTR: DCMI bit 0, OTGFS bit 7, RNG bit 6 = 0xC1 (CRYP bit 4 / HASH bit 5 exist only on F415/F417, not F407).
//  AHB3RSTR: FSMC bit 0 = 0x1.
//  APB1RSTR: TIM2-7 bits 0-5, TIM12-14 bits 6-8, WWDG 11, SPI2/3 14-15, USART2/3 + UART4/5 17-20, I2C1-3 21-23,
//            CAN1/2 25-26, PWR 28, DAC 29 = 0x36FEC9FF.
//  APB2RSTR: TIM1/8 bits 0-1, USART1/6 4-5, ADC 8, SDIO 11, SPI1 12, SYSCFG 14, TIM9-11 16-18 = 0x00075933.
const uint32_t kAhb1Defined = 0x226011FFu;
const uint32_t kAhb2Defined = 0x000000C1u;
const uint32_t kAhb3Defined = 0x00000001u;
const uint32_t kApb1Defined = 0x36FEC9FFu;
const uint32_t kApb2Defined = 0x00075933u;

TEST_CASE_FIXTURE(HalFixture, "H9 HAL_DeInit writes each RCC reset register first with the reset bits and then with zero") {
	// RM0090 7.3.5-7.3.9: a 1 holds the peripheral in reset, a 0 releases it. Two writes per register, force first.
	volatile uint32_t *regs[] = {&RCC->APB1RSTR, &RCC->APB2RSTR, &RCC->AHB1RSTR, &RCC->AHB2RSTR, &RCC->AHB3RSTR};
	const uint32_t defined[] = {kApb1Defined, kApb2Defined, kAhb1Defined, kAhb2Defined, kAhb3Defined};
	REQUIRE(HAL_DeInit() == HAL_OK);
	for (unsigned i = 0; i < 5; ++i) {
		const Writes w = writesTo(regs[i]);
		INFO("register offset " << (reinterpret_cast<std::uintptr_t>(regs[i]) - reinterpret_cast<std::uintptr_t>(RCC)));
		REQUIRE(w.n == 2u);
		CHECK((w.v[0] & defined[i]) == defined[i]); // every defined peripheral is put in reset
		CHECK(w.v[1] == 0u);                         // then released
		CHECK(peek(regs[i]) == 0u);
	}
}

TEST_CASE_FIXTURE(HalFixture, "H9 HAL_DeInit forces exactly the defined reset bits of AHB1RSTR, AHB2RSTR and AHB3RSTR") {
	REQUIRE(HAL_DeInit() == HAL_OK);
	CHECK(writesTo(&RCC->AHB1RSTR).v[0] == kAhb1Defined);
	CHECK(writesTo(&RCC->AHB2RSTR).v[0] == kAhb2Defined);
	CHECK(writesTo(&RCC->AHB3RSTR).v[0] == kAhb3Defined);
}

TEST_CASE_FIXTURE(HalFixture, "characterization: HAL_DeInit forces reserved bits of APB1RSTR and APB2RSTR to 1 (NEEDS REVIEW)") {
	// RM0090 7.3.8 / 7.3.9 mark the bits outside kApb1Defined / kApb2Defined reserved ("must be kept at reset value");
	// the HAL's force-reset word has some of them set (it is shared with the other F4 devices). Documents that the
	// request contains bits RM0090 calls reserved; whether this is a vendor defect is for the lead.
	// Exact difference observed today (hand-checked against the RM0090 masks above):
	//  APB1RSTR: no defined bit missing, reserved bits 31:30 set (0xC0000000).
	//  APB2RSTR: no defined bit missing, reserved bits 13, 20, 21, 22, 26 set (0x04702000).
	//  AHB1RSTR, AHB2RSTR, AHB3RSTR: equal to the defined masks (no difference).
	REQUIRE(HAL_DeInit() == HAL_OK);
	const uint32_t w1 = writesTo(&RCC->APB1RSTR).v[0];
	const uint32_t w2 = writesTo(&RCC->APB2RSTR).v[0];
	CHECK((kApb1Defined & ~w1) == 0u);
	CHECK((w1 & ~kApb1Defined) == 0xC0000000u);
	CHECK((kApb2Defined & ~w2) == 0u);
	CHECK((w2 & ~kApb2Defined) == 0x04702000u);
}

TEST_CASE_FIXTURE(HalFixture, "H9 HAL_DeInit leaves every RCC reset register at 0 whatever it held before") {
	poke(&RCC->APB1RSTR, 0x12345678u);
	poke(&RCC->APB2RSTR, 0x0F0F0F0Fu);
	poke(&RCC->AHB1RSTR, 0x00000001u);
	poke(&RCC->AHB2RSTR, 0x000000FFu);
	poke(&RCC->AHB3RSTR, 0x00000001u);
	REQUIRE(HAL_DeInit() == HAL_OK);
	CHECK(peek(&RCC->APB1RSTR) == 0u);
	CHECK(peek(&RCC->APB2RSTR) == 0u);
	CHECK(peek(&RCC->AHB1RSTR) == 0u);
	CHECK(peek(&RCC->AHB2RSTR) == 0u);
	CHECK(peek(&RCC->AHB3RSTR) == 0u);
}

// ---- H10 ----

TEST_CASE_FIXTURE(HalFixture, "H10 the default HAL_MspInit accesses no register") {
	HAL_MspInit();
	CHECK(hostRegsLogCount() == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "H10 the default HAL_MspDeInit accesses no register") {
	HAL_MspDeInit();
	CHECK(hostRegsLogCount() == 0u);
}

} // TEST_SUITE
