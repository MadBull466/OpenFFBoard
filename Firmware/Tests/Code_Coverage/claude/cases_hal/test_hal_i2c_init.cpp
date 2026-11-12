/*
 * P12 T4c I2C init cases (requirements II1-II7 of Tests/Code_Coverage/claude/charters/hal/P12_T4C_REQUIREMENTS.md).
 * Code under test: HAL_I2C_Init / DeInit / GetState / GetMode / GetError and the macros of stm32f4xx_hal_i2c.h of the real ST HAL,
 * on the I2C model fakes/stm32_hal/HalI2c.{h,c}.
 *
 * Oracles: RM0090 Rev 22 27.6 and UM1725 Rev 8 (I2C). Register layout used for the literal expectations:
 *   CR1: PE 0, ENGC 6, NOSTRETCH 7, SWRST 15.   CR2: FREQ[5:0], ITERREN 8, ITEVTEN 9, ITBUFEN 10, DMAEN 11, LAST 12.
 *   OAR1: ADD0 0, ADD[7:1] 7:1, ADD[9:8] 9:8, bit 14 "should always be kept at 1 by software", ADDMODE 15.
 *   OAR2: ENDUAL 0, ADD2[7:1] 7:1.   CCR: CCR[11:0], DUTY 14, F/S 15.   TRISE[5:0].
 *   SR1: SB 0, ADDR 1, BTF 2, ADD10 3, STOPF 4, RXNE 6, TXE 7, BERR 8, ARLO 9, AF 10, OVR 11.   SR2: MSL 0, BUSY 1, TRA 2, GENCALL 4, DUALF 7.
 * FREQ = PCLK1 in MHz. TRISE = integer part of (maximum SCL rise time / T_PCLK1) + 1 (RM0090 27.6.9: "programmed with the integer part"),
 * 1000 ns in Sm and 300 ns in Fm. CCR (27.6.8): Sm CCR = PCLK1 / (2 f), Fm DUTY 0 CCR = PCLK1 / (3 f), Fm DUTY 1 CCR = PCLK1 / (25 f) (the
 * period is 3 * CCR * T resp. 25 * CCR * T). The manual is silent on rounding when the division is not exact: those cases are
 * characterization cases.
 * Not compiled in this configuration: HAL_I2C_RegisterCallback (USE_HAL_I2C_REGISTER_CALLBACKS 0) and the I2C_Ex file (I2C_FLTR_* undefined).
 * II-asserts are not tested (USE_FULL_ASSERT off). The strong HAL_I2C_* callbacks live in test_hal_i2c_support.cpp.
 */
#include "I2cTestSupport.h"
#include "TestHelpers.h"

#include <cstdint>

extern "C" uint32_t SystemCoreClock;

namespace {

I2C_TypeDef *const iiInst[3] = {I2C1, I2C2, I2C3};

struct IiFixture : I2cFixture {
	I2C_HandleTypeDef h = {};
	IiFixture() {
		setPclk1(42000000u);
		setup(0);
		uwTickPrio = 15;
	}
	// PPRE1 = /1, so PCLK1 = SystemCoreClock (AHB prescaler of the reset CFGR is /1).
	static void setPclk1(uint32_t hz) {
		SystemCoreClock = hz;
		RCC->CFGR &= ~(7u << 10);
	}
	void setup(unsigned i) {
		h = I2C_HandleTypeDef{};
		h.Instance = iiInst[i];
		h.Init.ClockSpeed = 100000;
		h.Init.DutyCycle = I2C_DUTYCYCLE_2;
		h.Init.OwnAddress1 = 0x30;
		h.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		h.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		h.Init.OwnAddress2 = 0;
		h.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		h.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	}
	uint32_t cr1() const { return peek(&h.Instance->CR1); }
	uint32_t cr2() const { return peek(&h.Instance->CR2); }
	uint32_t oar1() const { return peek(&h.Instance->OAR1); }
	uint32_t oar2() const { return peek(&h.Instance->OAR2); }
	uint32_t ccr() const { return peek(&h.Instance->CCR); }
	uint32_t trise() const { return peek(&h.Instance->TRISE); }
	void doInit() { REQUIRE(HAL_I2C_Init(&h) == HAL_OK); }
	void initAt(uint32_t pclk, uint32_t speed, uint32_t duty = I2C_DUTYCYCLE_2) {
		setPclk1(pclk);
		h.Init.ClockSpeed = speed;
		h.Init.DutyCycle = duty;
		doInit();
	}
};

} // namespace

// ---------------------------------------------------------------------------------------------------------------------------------
// II1 Init entry conditions
// ---------------------------------------------------------------------------------------------------------------------------------

TEST_CASE("II1 init of a NULL handle returns HAL_ERROR") {
	I2cFixture f;
	CHECK(HAL_I2C_Init(nullptr) == HAL_ERROR);
	CHECK(i2cCounts.mspInit == 0u);
}

TEST_CASE_FIXTURE(IiFixture, "II1 init of a handle in state RESET calls MspInit once") {
	doInit();
	CHECK(i2cCounts.mspInit == 1u);
	CHECK(i2cCounts.last == &h);
}

TEST_CASE_FIXTURE(IiFixture, "II1 init of a handle in state RESET unlocks the handle") {
	h.Lock = HAL_LOCKED;
	doInit();
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(IiFixture, "II1 init of a handle that is not in state RESET does not call MspInit") {
	const HAL_I2C_StateTypeDef states[] = {HAL_I2C_STATE_READY, HAL_I2C_STATE_BUSY, HAL_I2C_STATE_BUSY_TX, HAL_I2C_STATE_LISTEN, HAL_I2C_STATE_ERROR};
	for (const HAL_I2C_StateTypeDef s : states) {
		i2cSupportReset();
		h.State = s;
		CHECK(HAL_I2C_Init(&h) == HAL_OK);
		CHECK_MESSAGE(i2cCounts.mspInit == 0u, "state " << (unsigned)s);
	}
}

TEST_CASE_FIXTURE(IiFixture, "II1 init of a handle that is not in state RESET leaves the lock as it was") {
	h.State = HAL_I2C_STATE_READY;
	h.Lock = HAL_LOCKED;
	doInit();
	CHECK(h.Lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(IiFixture, "II1 init pulses SWRST once") {
	doInit();
	CHECK(hostI2cSwrstCount(0) == 1u);
	CHECK((cr1() & (1u << 15)) == 0u);
}

TEST_CASE_FIXTURE(IiFixture, "II1 init clears PE before it sets SWRST and writes SWRST 1 then 0") {
	poke(&h.Instance->CR1, 1u); // PE already set by an earlier use
	hostRegsLogClear();
	doInit();
	// The first logged CR1 write is the disable, then SWRST set, then SWRST cleared (RM0090 27.6.1 PE, SWRST).
	uint32_t w[3] = {};
	unsigned n = 0;
	const uintptr_t cr1Addr = reinterpret_cast<uintptr_t>(&h.Instance->CR1);
	for (size_t i = 0; i < hostRegsLogCount() && n < 3; ++i) {
		const HostRegAccess *e = hostRegsLogAt(i);
		if (e->write && e->addr == cr1Addr) w[n++] = e->written;
	}
	REQUIRE(n == 3u);
	CHECK((w[0] & 1u) == 0u);
	CHECK((w[0] & (1u << 15)) == 0u);
	CHECK((w[1] & (1u << 15)) != 0u);
	CHECK((w[2] & (1u << 15)) == 0u);
}

TEST_CASE_FIXTURE(IiFixture, "II1 init writes CCR and TRISE only while PE is 0") {
	poke(&h.Instance->CR1, 1u);
	doInit();
	CHECK(hostI2cConfigWhileEnabled(0) == 0u);
}

TEST_CASE_FIXTURE(IiFixture, "II1 init with PCLK1 below 2 MHz in Sm returns HAL_ERROR") {
	setPclk1(1999999u);
	CHECK(HAL_I2C_Init(&h) == HAL_ERROR);
}

TEST_CASE_FIXTURE(IiFixture, "II1 init with PCLK1 of exactly 2 MHz in Sm succeeds") {
	setPclk1(2000000u);
	CHECK(HAL_I2C_Init(&h) == HAL_OK);
}

TEST_CASE_FIXTURE(IiFixture, "II1 init with PCLK1 below 4 MHz in Fm returns HAL_ERROR") {
	setPclk1(3999999u);
	h.Init.ClockSpeed = 400000;
	CHECK(HAL_I2C_Init(&h) == HAL_ERROR);
}

TEST_CASE_FIXTURE(IiFixture, "II1 init with PCLK1 of exactly 4 MHz in Fm succeeds") {
	setPclk1(4000000u);
	h.Init.ClockSpeed = 400000;
	CHECK(HAL_I2C_Init(&h) == HAL_OK);
}

TEST_CASE_FIXTURE(IiFixture, "II1 init at 100 kHz is Sm and at 100001 Hz is Fm for the PCLK1 limit") {
	// 3 MHz: enough for Sm (>= 2 MHz) but not for Fm (>= 4 MHz); the boundary between the modes is 100 kHz (UM1725: "up to 100 kHz" is Sm).
	setPclk1(3000000u);
	h.Init.ClockSpeed = 100000;
	CHECK(HAL_I2C_Init(&h) == HAL_OK);
	h.Init.ClockSpeed = 100001;
	CHECK(HAL_I2C_Init(&h) == HAL_ERROR);
}

TEST_CASE_FIXTURE(IiFixture, "II1 characterization: init with PCLK1 too low leaves the handle in state BUSY with PE 0 and no registers programmed") {
	// NEEDS REVIEW 320: UM1725 / RM0090 do not say what state a failed Init leaves; the HAL returns with State = BUSY after the SWRST pulse.
	setPclk1(1000000u);
	REQUIRE(HAL_I2C_Init(&h) == HAL_ERROR);
	CHECK(h.State == HAL_I2C_STATE_BUSY);
	CHECK((cr1() & 1u) == 0u);
	CHECK(hostI2cSwrstCount(0) == 1u);
	CHECK((cr2() & 0x3Fu) == 0u);
	CHECK(i2cCounts.mspInit == 1u);
}

TEST_CASE_FIXTURE(IiFixture, "II1 characterization: a second init after a PCLK1 error does not call MspInit again because the state is BUSY") {
	// NEEDS REVIEW 320: follows from NEEDS REVIEW 320 (state BUSY is not RESET).
	setPclk1(1000000u);
	REQUIRE(HAL_I2C_Init(&h) == HAL_ERROR);
	setPclk1(42000000u);
	REQUIRE(HAL_I2C_Init(&h) == HAL_OK);
	CHECK(i2cCounts.mspInit == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

// ---------------------------------------------------------------------------------------------------------------------------------
// II2 FREQ, TRISE, CCR
// ---------------------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(IiFixture, "II2 FREQ is PCLK1 in MHz for 8, 16, 24 and 42 MHz") {
	const uint32_t mhz[] = {8, 16, 24, 42};
	for (const uint32_t m : mhz) {
		i2cSupportReset();
		initAt(m * 1000000u, 100000);
		CHECK_MESSAGE((cr2() & 0x3Fu) == m, "PCLK1 " << m << " MHz");
	}
}

TEST_CASE_FIXTURE(IiFixture, "II2 FREQ with the firmware clock tree (168 MHz, PPRE1 /4) is 42") {
	SystemCoreClock = 168000000u;
	RCC->CFGR = (RCC->CFGR & ~(7u << 10)) | (5u << 10);
	doInit();
	CHECK((cr2() & 0x3Fu) == 42u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 FREQ keeps the integer part of a PCLK1 that is not a whole MHz") {
	// 8.5 MHz / 1e6 = 8.5 -> FREQ 8 (the manual's FREQ field holds whole MHz; fraction dropped).
	initAt(8500000u, 100000);
	CHECK((cr2() & 0x3Fu) == 8u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 Sm TRISE is FREQ + 1 for 8, 16, 24 and 42 MHz") {
	// 1000 ns / T_PCLK1 = FREQ (whole MHz), + 1: 8 -> 9 (the worked example of 27.6.9), 16 -> 17, 24 -> 25, 42 -> 43.
	const uint32_t mhz[] = {8, 16, 24, 42};
	const uint32_t want[] = {9, 17, 25, 43};
	for (unsigned k = 0; k < 4; ++k) {
		i2cSupportReset();
		initAt(mhz[k] * 1000000u, 100000);
		CHECK_MESSAGE(trise() == want[k], "PCLK1 " << mhz[k] << " MHz");
	}
}

TEST_CASE_FIXTURE(IiFixture, "II2 Fm TRISE for whole numbers of periods is 300 ns / T + 1") {
	// 10 MHz: 300 ns = 3 periods -> 4. 20 MHz: 6 -> 7. 40 MHz: 12 -> 13.
	const uint32_t mhz[] = {10, 20, 40};
	const uint32_t want[] = {4, 7, 13};
	for (unsigned k = 0; k < 3; ++k) {
		i2cSupportReset();
		initAt(mhz[k] * 1000000u, 400000);
		CHECK_MESSAGE(trise() == want[k], "PCLK1 " << mhz[k] << " MHz");
	}
}

TEST_CASE_FIXTURE(IiFixture, "II2 Fm TRISE takes the integer part of the number of periods (24 and 42 MHz)") {
	// RM0090 27.6.9: "programmed with the integer part". 24 MHz: 300 ns * 24 MHz = 7.2 -> 7, + 1 = 8. 42 MHz: 12.6 -> 12, + 1 = 13.
	initAt(24000000u, 400000);
	CHECK(trise() == 8u);
	i2cSupportReset();
	initAt(42000000u, 400000);
	CHECK(trise() == 13u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 Fm TRISE at the smallest PCLK1 of 4 MHz is 2") {
	// 300 ns * 4 MHz = 1.2 -> 1, + 1 = 2.
	initAt(4000000u, 400000);
	CHECK(trise() == 2u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 Sm CCR is PCLK1 / (2 * 100 kHz) for 8, 16, 24 and 42 MHz with F/S and DUTY clear") {
	// 8 MHz: 40 = 0x28 (the worked example of RM0090 27.6.8), 16 MHz: 80, 24 MHz: 120, 42 MHz: 210.
	const uint32_t mhz[] = {8, 16, 24, 42};
	const uint32_t want[] = {40, 80, 120, 210};
	for (unsigned k = 0; k < 4; ++k) {
		i2cSupportReset();
		initAt(mhz[k] * 1000000u, 100000);
		CHECK_MESSAGE(ccr() == want[k], "PCLK1 " << mhz[k] << " MHz");
	}
}

TEST_CASE_FIXTURE(IiFixture, "II2 Sm CCR for the worked example (FREQ 8, 100 kHz) is 0x28") {
	initAt(8000000u, 100000);
	CHECK(ccr() == 0x28u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 Sm CCR at 50 kHz is PCLK1 / 100000") {
	// 42 MHz / (2 * 50 kHz) = 420.
	initAt(42000000u, 50000);
	CHECK(ccr() == 420u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 characterization: Sm CCR rounds a non-exact quotient up") {
	// NEEDS REVIEW 320: RM0090 is silent on rounding. 7 MHz / 200 kHz = 35 exactly; 7.1 MHz / 200 kHz = 35.5 -> HAL gives 36 (a slower, safe clock).
	initAt(7100000u, 100000);
	CHECK(ccr() == 36u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 Fm duty 2 sets F/S and CCR = PCLK1 / (3 * 400 kHz) with DUTY clear") {
	// 24 MHz / 1.2 MHz = 20; 36 MHz: 30; 12 MHz: 10; F/S = bit 15.
	const uint32_t mhz[] = {12, 24, 36};
	const uint32_t want[] = {10, 20, 30};
	for (unsigned k = 0; k < 3; ++k) {
		i2cSupportReset();
		initAt(mhz[k] * 1000000u, 400000, I2C_DUTYCYCLE_2);
		CHECK_MESSAGE(ccr() == (0x8000u | want[k]), "PCLK1 " << mhz[k] << " MHz");
	}
}

TEST_CASE_FIXTURE(IiFixture, "II2 Fm duty 2 at 42 MHz and 400 kHz gives CCR 35") {
	// 42 MHz / (3 * 400 kHz) = 35 exactly.
	initAt(42000000u, 400000, I2C_DUTYCYCLE_2);
	CHECK(ccr() == 0x8023u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 characterization: Fm duty 2 rounds a non-exact quotient up") {
	// NEEDS REVIEW 320: 16 MHz / 1.2 MHz = 13.33 -> HAL gives 14; 8 MHz: 6.67 -> 7.
	initAt(16000000u, 400000, I2C_DUTYCYCLE_2);
	CHECK(ccr() == 0x800Eu);
	i2cSupportReset();
	initAt(8000000u, 400000, I2C_DUTYCYCLE_2);
	CHECK(ccr() == 0x8007u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 Fm duty 16/9 sets F/S, DUTY and CCR = PCLK1 / (25 * 400 kHz)") {
	// 10 MHz / 10 MHz = 1, 20 MHz: 2, 40 MHz: 4. F/S bit 15, DUTY bit 14 -> 0xC000 | CCR.
	const uint32_t mhz[] = {10, 20, 40};
	const uint32_t want[] = {1, 2, 4};
	for (unsigned k = 0; k < 3; ++k) {
		i2cSupportReset();
		initAt(mhz[k] * 1000000u, 400000, I2C_DUTYCYCLE_16_9);
		CHECK_MESSAGE(ccr() == (0xC000u | want[k]), "PCLK1 " << mhz[k] << " MHz");
	}
}

TEST_CASE_FIXTURE(IiFixture, "II2 Fm duty 16/9 at 42 MHz rounds the quotient 4.2 to 5") {
	// 42 MHz / 10 MHz = 4.2: not exact. RM0090 is silent on rounding.
	// characterization: NEEDS REVIEW 320, the HAL rounds up (slower clock).
	initAt(42000000u, 400000, I2C_DUTYCYCLE_16_9);
	CHECK(ccr() == 0xC005u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 Fm duty 16/9 at the PCLK1 minimum for Fm (4 MHz) gives CCR 1") {
	// 4 MHz / 10 MHz = 0.4 -> rounded up 1 (the minimum allowed in FAST DUTY mode is 1, RM0090 27.6.8).
	initAt(4000000u, 400000, I2C_DUTYCYCLE_16_9);
	CHECK((ccr() & 0xFFFu) >= 1u);
	CHECK(ccr() == 0xC001u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 a speed above 100 kHz selects Fm and 100 kHz or below selects Sm") {
	initAt(24000000u, 100000);
	CHECK((ccr() & 0x8000u) == 0u);
	i2cSupportReset();
	initAt(24000000u, 100001);
	CHECK((ccr() & 0x8000u) != 0u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 the duty cycle setting is ignored in Sm") {
	// DUTY applies to Fm only (RM0090 27.6.8 "Bit 14 DUTY: Fm mode duty cycle").
	initAt(24000000u, 100000, I2C_DUTYCYCLE_16_9);
	CHECK((ccr() & 0x4000u) == 0u);
	CHECK(ccr() == 120u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 Sm CCR of at least 4 is respected: PCLK1 2 MHz at 100 kHz gives 10") {
	// 2 MHz / 200 kHz = 10 (>= the minimum 4 of 27.6.8).
	initAt(2000000u, 100000);
	CHECK(ccr() == 10u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 characterization: Sm speed so low that the 12-bit CCR wraps below 4 is raised to 4") {
	// NEEDS REVIEW 320: 2 MHz / (2 * 244 Hz) = 4098.4 -> rounded up 4099, which does not fit the 12-bit CCR[11:0] field (RM0090 27.6.8); the HAL masks it
	// to 3 and then applies the minimum 4. The manual gives no value for a speed that the field cannot hold.
	initAt(2000000u, 244);
	CHECK(ccr() == 4u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 characterization: Sm speed that needs more than 12 CCR bits is truncated to 12 bits") {
	// NEEDS REVIEW 320: 42 MHz / (2 * 5 kHz) = 4200 > 4095 (CCR is 12 bits): HAL gives 4200 & 0xFFF = 104 (a clock 40 times too fast), no error.
	initAt(42000000u, 5000);
	CHECK(ccr() == 104u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 init overwrites old CCR and TRISE contents") {
	poke(&h.Instance->CCR, 0xCFFFu);
	poke(&h.Instance->TRISE, 0x3Fu);
	initAt(8000000u, 100000);
	CHECK(ccr() == 0x28u);
	CHECK(trise() == 9u);
}

TEST_CASE_FIXTURE(IiFixture, "II2 init overwrites an old FREQ and keeps the other CR2 bits") {
	poke(&h.Instance->CR2, 0x3Fu | 0x1F00u);
	initAt(16000000u, 100000);
	CHECK(cr2() == (0x1F00u | 16u));
}

// ---------------------------------------------------------------------------------------------------------------------------------
// II3 OAR1, OAR2, CR1
// ---------------------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(IiFixture, "II3 OAR1 in 7-bit mode holds the own address in ADD[7:1]") {
	h.Init.OwnAddress1 = 0x30; // 7-bit address 0x18 shifted by one
	doInit();
	CHECK((oar1() & 0x83FFu) == 0x0030u);
}

TEST_CASE_FIXTURE(IiFixture, "II3 OAR1 in 7-bit mode has ADDMODE clear") {
	h.Init.OwnAddress1 = 0xFE;
	doInit();
	CHECK((oar1() & 0x8000u) == 0u);
	CHECK((oar1() & 0x83FFu) == 0x00FEu);
}

TEST_CASE_FIXTURE(IiFixture, "II3 OAR1 in 10-bit mode sets ADDMODE and holds the whole 10-bit address") {
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
	h.Init.OwnAddress1 = 0x2AB; // ADD[9:8] = 10, ADD[7:0] = 0xAB
	doInit();
	CHECK((oar1() & 0x83FFu) == 0x82ABu);
}

TEST_CASE_FIXTURE(IiFixture, "II3 OAR1 in 10-bit mode with the largest address 0x3FF") {
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
	h.Init.OwnAddress1 = 0x3FF;
	doInit();
	CHECK((oar1() & 0x83FFu) == 0x83FFu);
}

TEST_CASE_FIXTURE(IiFixture, "II3 OAR1 reconfigured from 10-bit to 7-bit clears the old address bits and ADDMODE") {
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
	h.Init.OwnAddress1 = 0x3FF;
	doInit();
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	h.Init.OwnAddress1 = 0x20;
	doInit();
	CHECK((oar1() & 0x83FFu) == 0x0020u);
}

TEST_CASE_FIXTURE(IiFixture, "II3 OAR1 bit 14 is 1 after init from the reset value") {
	// RM0090 27.6.3 "Bit 14: Should always be kept at 1 by software". The reset value is 0; the HAL sets it through I2C_ADDRESSINGMODE_*.
	REQUIRE(oar1() == 0u);
	doInit();
	CHECK((oar1() & 0x4000u) == 0x4000u);
}

TEST_CASE_FIXTURE(IiFixture, "II3 OAR1 bit 14 is 1 after init in 10-bit mode") {
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
	doInit();
	CHECK((oar1() & 0xC000u) == 0xC000u);
}

TEST_CASE_FIXTURE(IiFixture, "II3 OAR1 bit 14 set before init is kept") {
	poke(&h.Instance->OAR1, 0x4000u);
	doInit();
	CHECK((oar1() & 0x4000u) == 0x4000u);
}

TEST_CASE_FIXTURE(IiFixture, "II3 OAR2 with dual addressing enabled sets ENDUAL and ADD2") {
	h.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
	h.Init.OwnAddress2 = 0x60; // 7-bit address 0x30 shifted by one
	doInit();
	CHECK(oar2() == 0x61u);
}

TEST_CASE_FIXTURE(IiFixture, "II3 OAR2 with dual addressing disabled has ENDUAL clear and ADD2 as given") {
	h.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	h.Init.OwnAddress2 = 0x60;
	doInit();
	CHECK((oar2() & 1u) == 0u);
	CHECK((oar2() & 0xFEu) == 0x60u);
}

TEST_CASE_FIXTURE(IiFixture, "II3 OAR2 largest address 0xFE with ENDUAL gives 0xFF") {
	h.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
	h.Init.OwnAddress2 = 0xFE;
	doInit();
	CHECK(oar2() == 0xFFu);
}

TEST_CASE_FIXTURE(IiFixture, "II3 OAR2 is cleared by a following init without dual addressing") {
	h.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
	h.Init.OwnAddress2 = 0xFE;
	doInit();
	h.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	h.Init.OwnAddress2 = 0;
	doInit();
	CHECK(oar2() == 0u);
}

TEST_CASE_FIXTURE(IiFixture, "II3 a second own address is answered by the target engine when dual addressing is on") {
	h.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
	h.Init.OwnAddress2 = 0x60;
	doInit();
	h.Instance->CR1 |= (1u << 10); // ACK: a target acknowledges its address only with ACK = 1 (RM0090 27.3.2)
	CHECK(hostI2cExtStart(0, 0x60) == 1);
	CHECK((h.Instance->SR2 & (1u << 7)) != 0u); // DUALF
}

TEST_CASE_FIXTURE(IiFixture, "II3 the own address given at init is answered by the target engine") {
	h.Init.OwnAddress1 = 0x42;
	doInit();
	h.Instance->CR1 |= (1u << 10); // ACK
	CHECK(hostI2cExtStart(0, 0x42) == 1);
	CHECK((h.Instance->SR1 & (1u << 1)) != 0u); // ADDR
}

TEST_CASE_FIXTURE(IiFixture, "II3 general call enabled sets ENGC (CR1 bit 6)") {
	h.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
	doInit();
	CHECK((cr1() & 0x40u) == 0x40u);
}

TEST_CASE_FIXTURE(IiFixture, "II3 general call disabled leaves ENGC clear") {
	doInit();
	CHECK((cr1() & 0x40u) == 0u);
}

TEST_CASE_FIXTURE(IiFixture, "II3 no stretch enabled sets NOSTRETCH (CR1 bit 7)") {
	h.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
	doInit();
	CHECK((cr1() & 0x80u) == 0x80u);
}

TEST_CASE_FIXTURE(IiFixture, "II3 no stretch disabled leaves NOSTRETCH clear") {
	doInit();
	CHECK((cr1() & 0x80u) == 0u);
}

TEST_CASE_FIXTURE(IiFixture, "II3 ENGC and NOSTRETCH both set together") {
	h.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
	h.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
	doInit();
	CHECK(cr1() == 0xC1u); // ENGC | NOSTRETCH | PE
}

TEST_CASE_FIXTURE(IiFixture, "II3 a second init without ENGC and NOSTRETCH clears them again") {
	h.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
	h.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
	doInit();
	h.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	h.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	doInit();
	CHECK((cr1() & 0xC0u) == 0u);
}

TEST_CASE_FIXTURE(IiFixture, "II3 init does not touch the CR2 interrupt and DMA enables") {
	poke(&h.Instance->CR2, 0x1F00u);
	doInit();
	CHECK((cr2() & 0x1F00u) == 0x1F00u);
}

TEST_CASE_FIXTURE(IiFixture, "II3 init leaves all interrupt enables off from the reset state") {
	doInit();
	CHECK((cr2() & 0x0700u) == 0u);
}

// ---------------------------------------------------------------------------------------------------------------------------------
// II4 end state, DeInit
// ---------------------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(IiFixture, "II4 init ends with PE set") {
	doInit();
	CHECK((cr1() & 1u) == 1u);
}

TEST_CASE_FIXTURE(IiFixture, "II4 init ends in state READY with no error, previous state NONE and mode NONE") {
	h.ErrorCode = HAL_I2C_ERROR_AF;
	h.PreviousState = 0x77u;
	h.Mode = HAL_I2C_MODE_MEM;
	h.State = HAL_I2C_STATE_ERROR;
	doInit();
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.PreviousState == 0u);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
}

TEST_CASE_FIXTURE(IiFixture, "II4 init of the handle that is in use repeats cleanly") {
	doInit();
	doInit();
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK((cr1() & 1u) == 1u);
	CHECK(i2cCounts.mspInit == 1u);
	CHECK(hostI2cSwrstCount(0) == 2u);
}

TEST_CASE_FIXTURE(IiFixture, "II4 init works on each of the three instances") {
	for (unsigned i = 0; i < 3; ++i) {
		i2cSupportReset();
		setup(i);
		doInit();
		CHECK_MESSAGE(h.State == HAL_I2C_STATE_READY, "I2C" << i + 1);
		CHECK_MESSAGE((cr1() & 1u) == 1u, "I2C" << i + 1);
		CHECK_MESSAGE((cr2() & 0x3Fu) == 42u, "I2C" << i + 1);
		CHECK_MESSAGE(ccr() == 210u, "I2C" << i + 1);
		CHECK_MESSAGE(trise() == 43u, "I2C" << i + 1);
		CHECK_MESSAGE(hostI2cSwrstCount(i) == 1u, "I2C" << i + 1);
	}
}

TEST_CASE_FIXTURE(IiFixture, "II4 init of one instance does not change another instance") {
	doInit();
	CHECK(peek(&I2C2->CR1) == 0u);
	CHECK(peek(&I2C3->CR2) == 0u);
	CHECK(hostI2cSwrstCount(1) == 0u);
}

TEST_CASE_FIXTURE(IiFixture, "II4 DeInit clears PE, calls MspDeInit once and ends in state RESET") {
	doInit();
	REQUIRE(HAL_I2C_DeInit(&h) == HAL_OK);
	CHECK((cr1() & 1u) == 0u);
	CHECK(i2cCounts.mspDeInit == 1u);
	CHECK(h.State == HAL_I2C_STATE_RESET);
}

TEST_CASE_FIXTURE(IiFixture, "II4 DeInit resets error code, previous state and mode") {
	doInit();
	h.ErrorCode = HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_OVR;
	h.PreviousState = 0x55u;
	h.Mode = HAL_I2C_MODE_MASTER;
	REQUIRE(HAL_I2C_DeInit(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.PreviousState == 0u);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
}

TEST_CASE_FIXTURE(IiFixture, "II4 DeInit releases the lock") {
	doInit();
	h.Lock = HAL_LOCKED;
	REQUIRE(HAL_I2C_DeInit(&h) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(IiFixture, "II4 DeInit of a NULL handle returns HAL_ERROR") {
	CHECK(HAL_I2C_DeInit(nullptr) == HAL_ERROR);
	CHECK(i2cCounts.mspDeInit == 0u);
}

TEST_CASE_FIXTURE(IiFixture, "II4 DeInit of a never initialised handle returns HAL_OK and ends in state RESET") {
	CHECK(HAL_I2C_DeInit(&h) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_RESET);
	CHECK(i2cCounts.mspDeInit == 1u);
	CHECK((cr1() & 1u) == 0u);
}

TEST_CASE_FIXTURE(IiFixture, "II4 characterization: DeInit while a transfer is in progress still resets the handle") {
	// NEEDS REVIEW 320: UM1725 does not say that DeInit refuses a busy peripheral; the HAL does not check the state.
	doInit();
	h.State = HAL_I2C_STATE_BUSY_TX;
	h.Mode = HAL_I2C_MODE_MASTER;
	CHECK(HAL_I2C_DeInit(&h) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_RESET);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
}

TEST_CASE_FIXTURE(IiFixture, "II4 init after DeInit calls MspInit again") {
	doInit();
	REQUIRE(HAL_I2C_DeInit(&h) == HAL_OK);
	doInit();
	CHECK(i2cCounts.mspInit == 2u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK((cr1() & 1u) == 1u);
}

TEST_CASE_FIXTURE(IiFixture, "II4 DeInit works on each of the three instances") {
	for (unsigned i = 0; i < 3; ++i) {
		i2cSupportReset();
		setup(i);
		doInit();
		REQUIRE(HAL_I2C_DeInit(&h) == HAL_OK);
		CHECK_MESSAGE((cr1() & 1u) == 0u, "I2C" << i + 1);
		CHECK_MESSAGE(h.State == HAL_I2C_STATE_RESET, "I2C" << i + 1);
		CHECK_MESSAGE(i2cCounts.mspDeInit == 1u, "I2C" << i + 1);
	}
}

// ---------------------------------------------------------------------------------------------------------------------------------
// II5 getters and values
// ---------------------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(IiFixture, "II5 GetState returns the state field") {
	const HAL_I2C_StateTypeDef states[] = {HAL_I2C_STATE_RESET, HAL_I2C_STATE_READY, HAL_I2C_STATE_BUSY, HAL_I2C_STATE_BUSY_TX, HAL_I2C_STATE_BUSY_RX,
	                                       HAL_I2C_STATE_LISTEN, HAL_I2C_STATE_BUSY_TX_LISTEN, HAL_I2C_STATE_BUSY_RX_LISTEN, HAL_I2C_STATE_ABORT,
	                                       HAL_I2C_STATE_TIMEOUT, HAL_I2C_STATE_ERROR};
	for (const HAL_I2C_StateTypeDef s : states) {
		h.State = s;
		CHECK(HAL_I2C_GetState(&h) == s);
	}
}

TEST_CASE_FIXTURE(IiFixture, "II5 GetMode returns the mode field") {
	const HAL_I2C_ModeTypeDef modes[] = {HAL_I2C_MODE_NONE, HAL_I2C_MODE_MASTER, HAL_I2C_MODE_SLAVE, HAL_I2C_MODE_MEM};
	for (const HAL_I2C_ModeTypeDef m : modes) {
		h.Mode = m;
		CHECK(HAL_I2C_GetMode(&h) == m);
	}
}

TEST_CASE_FIXTURE(IiFixture, "II5 GetError returns the error code field") {
	const uint32_t codes[] = {HAL_I2C_ERROR_NONE, HAL_I2C_ERROR_BERR, HAL_I2C_ERROR_ARLO, HAL_I2C_ERROR_AF, HAL_I2C_ERROR_OVR, HAL_I2C_ERROR_DMA,
	                          HAL_I2C_ERROR_TIMEOUT, HAL_I2C_ERROR_SIZE, HAL_I2C_ERROR_DMA_PARAM, HAL_I2C_WRONG_START, 0x1FFu};
	for (const uint32_t c : codes) {
		h.ErrorCode = c;
		CHECK(HAL_I2C_GetError(&h) == c);
	}
}

TEST_CASE_FIXTURE(IiFixture, "II5 the getters work on each of the three instances after init") {
	for (unsigned i = 0; i < 3; ++i) {
		setup(i);
		doInit();
		CHECK_MESSAGE(HAL_I2C_GetState(&h) == HAL_I2C_STATE_READY, "I2C" << i + 1);
		CHECK_MESSAGE(HAL_I2C_GetMode(&h) == HAL_I2C_MODE_NONE, "I2C" << i + 1);
		CHECK_MESSAGE(HAL_I2C_GetError(&h) == HAL_I2C_ERROR_NONE, "I2C" << i + 1);
	}
}

TEST_CASE_FIXTURE(IiFixture, "II5 state values follow the UM1725 bit coding") {
	// b5 initialised, b3 listen, b2 busy, b1 Rx busy, b0 Tx busy; b7:6 error info 01 abort, 10 timeout, 11 error.
	CHECK((unsigned)HAL_I2C_STATE_RESET == 0x00u);
	CHECK((unsigned)HAL_I2C_STATE_READY == 0x20u);
	CHECK((unsigned)HAL_I2C_STATE_BUSY == 0x24u);
	CHECK((unsigned)HAL_I2C_STATE_BUSY_TX == 0x21u);
	CHECK((unsigned)HAL_I2C_STATE_BUSY_RX == 0x22u);
	CHECK((unsigned)HAL_I2C_STATE_LISTEN == 0x28u);
	CHECK((unsigned)HAL_I2C_STATE_BUSY_TX_LISTEN == 0x29u);
	CHECK((unsigned)HAL_I2C_STATE_BUSY_RX_LISTEN == 0x2Au);
	CHECK((unsigned)HAL_I2C_STATE_ABORT == 0x60u);
	CHECK((unsigned)HAL_I2C_STATE_TIMEOUT == 0xA0u);
	CHECK((unsigned)HAL_I2C_STATE_ERROR == 0xE0u);
}

TEST_CASE_FIXTURE(IiFixture, "II5 mode values follow the UM1725 bit coding") {
	CHECK((unsigned)HAL_I2C_MODE_NONE == 0x00u);
	CHECK((unsigned)HAL_I2C_MODE_MASTER == 0x10u);
	CHECK((unsigned)HAL_I2C_MODE_SLAVE == 0x20u);
	CHECK((unsigned)HAL_I2C_MODE_MEM == 0x40u);
}

TEST_CASE_FIXTURE(IiFixture, "II5 error codes are single bits that map to the SR1 error flags") {
	CHECK(HAL_I2C_ERROR_NONE == 0u);
	CHECK(HAL_I2C_ERROR_BERR == 0x01u);
	CHECK(HAL_I2C_ERROR_ARLO == 0x02u);
	CHECK(HAL_I2C_ERROR_AF == 0x04u);
	CHECK(HAL_I2C_ERROR_OVR == 0x08u);
	CHECK(HAL_I2C_ERROR_DMA == 0x10u);
	CHECK(HAL_I2C_ERROR_TIMEOUT == 0x20u);
	CHECK(HAL_I2C_ERROR_SIZE == 0x40u);
	CHECK(HAL_I2C_ERROR_DMA_PARAM == 0x80u);
	CHECK(HAL_I2C_WRONG_START == 0x200u);
}

// ---------------------------------------------------------------------------------------------------------------------------------
// II6 macros
// ---------------------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(IiFixture, "II6 ENABLE sets PE and DISABLE clears it, leaving other CR1 bits") {
	poke(&h.Instance->CR1, 0xC0u);
	__HAL_I2C_ENABLE(&h);
	CHECK(cr1() == 0xC1u);
	__HAL_I2C_DISABLE(&h);
	CHECK(cr1() == 0xC0u);
}

TEST_CASE_FIXTURE(IiFixture, "II6 flag constants encode the RM0090 bit positions of SR1 and SR2") {
	CHECK((I2C_FLAG_SB & 0xFFFFu) == (1u << 0));
	CHECK((I2C_FLAG_ADDR & 0xFFFFu) == (1u << 1));
	CHECK((I2C_FLAG_BTF & 0xFFFFu) == (1u << 2));
	CHECK((I2C_FLAG_ADD10 & 0xFFFFu) == (1u << 3));
	CHECK((I2C_FLAG_STOPF & 0xFFFFu) == (1u << 4));
	CHECK((I2C_FLAG_RXNE & 0xFFFFu) == (1u << 6));
	CHECK((I2C_FLAG_TXE & 0xFFFFu) == (1u << 7));
	CHECK((I2C_FLAG_BERR & 0xFFFFu) == (1u << 8));
	CHECK((I2C_FLAG_ARLO & 0xFFFFu) == (1u << 9));
	CHECK((I2C_FLAG_AF & 0xFFFFu) == (1u << 10));
	CHECK((I2C_FLAG_OVR & 0xFFFFu) == (1u << 11));
	CHECK((I2C_FLAG_MSL & 0xFFFFu) == (1u << 0));
	CHECK((I2C_FLAG_BUSY & 0xFFFFu) == (1u << 1));
	CHECK((I2C_FLAG_TRA & 0xFFFFu) == (1u << 2));
	CHECK((I2C_FLAG_GENCALL & 0xFFFFu) == (1u << 4));
	CHECK((I2C_FLAG_DUALF & 0xFFFFu) == (1u << 7));
}

TEST_CASE_FIXTURE(IiFixture, "II6 GET_FLAG reads SR1 flags from SR1") {
	doInit();
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_AF) == RESET);
	hostI2cRaiseSr1(0, 1u << 10);
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_AF) == SET);
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_BERR) == RESET);
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_BUSY) == RESET);
}

TEST_CASE_FIXTURE(IiFixture, "II6 GET_FLAG reads SR2 flags from SR2") {
	doInit();
	hostI2cBusBusy(0, 1);
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_BUSY) == SET);
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_MSL) == RESET);
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_AF) == RESET);
}

TEST_CASE_FIXTURE(IiFixture, "II6 CLEAR_FLAG clears the flag written with 0 and leaves the other rc_w0 flags") {
	doInit();
	hostI2cRaiseSr1(0, (1u << 10) | (1u << 8) | (1u << 11)); // AF, BERR, OVR
	__HAL_I2C_CLEAR_FLAG(&h, I2C_FLAG_AF);
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_AF) == RESET);
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_BERR) == SET);
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_OVR) == SET);
}

TEST_CASE_FIXTURE(IiFixture, "II6 CLEAR_FLAG clears each of OVR, AF, ARLO and BERR") {
	doInit();
	const uint32_t flags[] = {I2C_FLAG_OVR, I2C_FLAG_AF, I2C_FLAG_ARLO, I2C_FLAG_BERR};
	for (const uint32_t f : flags) {
		hostI2cRaiseSr1(0, f & 0xFFFFu);
		REQUIRE(__HAL_I2C_GET_FLAG(&h, f) == SET);
		__HAL_I2C_CLEAR_FLAG(&h, f);
		CHECK_MESSAGE(__HAL_I2C_GET_FLAG(&h, f) == RESET, "flag " << std::hex << f);
	}
}

TEST_CASE_FIXTURE(IiFixture, "II6 CLEAR_FLAG with several flags clears all of them") {
	doInit();
	hostI2cRaiseSr1(0, (1u << 10) | (1u << 8));
	__HAL_I2C_CLEAR_FLAG(&h, I2C_FLAG_AF | I2C_FLAG_BERR);
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_AF) == RESET);
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_BERR) == RESET);
}

TEST_CASE_FIXTURE(IiFixture, "II6 ENABLE_IT sets and DISABLE_IT clears the CR2 interrupt enables") {
	doInit();
	__HAL_I2C_ENABLE_IT(&h, I2C_IT_EVT);
	CHECK((cr2() & 0x0700u) == 0x0200u);
	__HAL_I2C_ENABLE_IT(&h, I2C_IT_BUF | I2C_IT_ERR);
	CHECK((cr2() & 0x0700u) == 0x0700u);
	__HAL_I2C_DISABLE_IT(&h, I2C_IT_BUF);
	CHECK((cr2() & 0x0700u) == 0x0300u);
	__HAL_I2C_DISABLE_IT(&h, I2C_IT_EVT | I2C_IT_ERR);
	CHECK((cr2() & 0x0700u) == 0u);
	CHECK((cr2() & 0x3Fu) == 42u);
}

TEST_CASE_FIXTURE(IiFixture, "II6 interrupt constants are the CR2 bits ITERREN 8, ITEVTEN 9, ITBUFEN 10") {
	CHECK((uint32_t)I2C_IT_ERR == (1u << 8));
	CHECK((uint32_t)I2C_IT_EVT == (1u << 9));
	CHECK((uint32_t)I2C_IT_BUF == (1u << 10));
}

TEST_CASE_FIXTURE(IiFixture, "II6 GET_IT_SOURCE is SET only when all requested bits are enabled") {
	doInit();
	CHECK(__HAL_I2C_GET_IT_SOURCE(&h, I2C_IT_EVT) == RESET);
	__HAL_I2C_ENABLE_IT(&h, I2C_IT_EVT);
	CHECK(__HAL_I2C_GET_IT_SOURCE(&h, I2C_IT_EVT) == SET);
	CHECK(__HAL_I2C_GET_IT_SOURCE(&h, I2C_IT_BUF) == RESET);
	CHECK(__HAL_I2C_GET_IT_SOURCE(&h, I2C_IT_EVT | I2C_IT_BUF) == RESET);
	__HAL_I2C_ENABLE_IT(&h, I2C_IT_BUF);
	CHECK(__HAL_I2C_GET_IT_SOURCE(&h, I2C_IT_EVT | I2C_IT_BUF) == SET);
}

TEST_CASE_FIXTURE(IiFixture, "II6 CLEAR_ADDRFLAG clears ADDR by reading SR1 then SR2") {
	doInit();
	h.Instance->CR1 |= (1u << 10); // ACK
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	REQUIRE(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_ADDR) == SET);
	__HAL_I2C_CLEAR_ADDRFLAG(&h);
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_ADDR) == RESET);
}

TEST_CASE_FIXTURE(IiFixture, "II6 reading SR1 alone does not clear ADDR") {
	doInit();
	h.Instance->CR1 |= (1u << 10); // ACK
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	const uint32_t sr1 = h.Instance->SR1;
	CHECK((sr1 & (1u << 1)) != 0u);
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_ADDR) == SET);
}

TEST_CASE_FIXTURE(IiFixture, "II6 CLEAR_STOPFLAG clears STOPF by reading SR1 then writing CR1") {
	doInit();
	h.Instance->CR1 |= (1u << 10); // ACK
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	__HAL_I2C_CLEAR_ADDRFLAG(&h);
	hostI2cExtStop(0);
	REQUIRE(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_STOPF) == SET);
	__HAL_I2C_CLEAR_STOPFLAG(&h);
	CHECK(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_STOPF) == RESET);
	CHECK((cr1() & 1u) == 1u);
}

TEST_CASE_FIXTURE(IiFixture, "II6 the macros work on each of the three instances") {
	for (unsigned i = 0; i < 3; ++i) {
		setup(i);
		doInit();
		__HAL_I2C_ENABLE_IT(&h, I2C_IT_EVT);
		CHECK_MESSAGE(__HAL_I2C_GET_IT_SOURCE(&h, I2C_IT_EVT) == SET, "I2C" << i + 1);
		__HAL_I2C_DISABLE(&h);
		CHECK_MESSAGE((cr1() & 1u) == 0u, "I2C" << i + 1);
		hostI2cRaiseSr1(i, 1u << 10);
		CHECK_MESSAGE(__HAL_I2C_GET_FLAG(&h, I2C_FLAG_AF) == SET, "I2C" << i + 1);
	}
}

TEST_CASE("II6 I2C_FREQRANGE is PCLK1 / 1000000 for the speeds table of the firmware clocks") {
	CHECK(I2C_FREQRANGE(8000000U) == 8U);
	CHECK(I2C_FREQRANGE(16000000U) == 16U);
	CHECK(I2C_FREQRANGE(24000000U) == 24U);
	CHECK(I2C_FREQRANGE(42000000U) == 42U);
	CHECK(I2C_FREQRANGE(999999U) == 0U);
}

TEST_CASE("II6 I2C_RISE_TIME in Sm is FREQ + 1 and in Fm is 300 ns / T + 1") {
	CHECK(I2C_RISE_TIME(8U, 100000U) == 9U);
	CHECK(I2C_RISE_TIME(42U, 100000U) == 43U);
	CHECK(I2C_RISE_TIME(10U, 400000U) == 4U);
	CHECK(I2C_RISE_TIME(42U, 400000U) == 13U);
	CHECK(I2C_RISE_TIME(42U, 100001U) == 13U);
}

TEST_CASE("II6 I2C_SPEED results for the speeds of the firmware table") {
	// Sm 100 kHz at 42 MHz: 210. Fm 400 kHz duty 2 at 42 MHz: 35 | F/S. Fm duty 16/9 at 40 MHz: 4 | DUTY | F/S.
	CHECK(I2C_SPEED(42000000U, 100000U, I2C_DUTYCYCLE_2) == 210U);
	CHECK(I2C_SPEED(42000000U, 400000U, I2C_DUTYCYCLE_2) == (35U | I2C_CCR_FS));
	CHECK(I2C_SPEED(40000000U, 400000U, I2C_DUTYCYCLE_16_9) == (4U | I2C_CCR_DUTY | I2C_CCR_FS));
	CHECK(I2C_SPEED(8000000U, 100000U, I2C_DUTYCYCLE_2) == 0x28U);
}

TEST_CASE("II6 I2C_MIN_PCLK_FREQ flags a PCLK1 below the minimum of the mode") {
	CHECK(I2C_MIN_PCLK_FREQ(1999999U, 100000U) == 1U);
	CHECK(I2C_MIN_PCLK_FREQ(2000000U, 100000U) == 0U);
	CHECK(I2C_MIN_PCLK_FREQ(3999999U, 400000U) == 1U);
	CHECK(I2C_MIN_PCLK_FREQ(4000000U, 400000U) == 0U);
}

TEST_CASE("II6 the 10-bit and memory address helper macros split an address as RM0090 27.3.3 and UM1725 describe") {
	// 10-bit header 11110 A9 A8 R/W; the second byte is A[7:0].
	CHECK(I2C_10BIT_HEADER_WRITE(0x2ABU) == 0xF4U);
	CHECK(I2C_10BIT_HEADER_READ(0x2ABU) == 0xF5U);
	CHECK(I2C_10BIT_HEADER_WRITE(0x0FFU) == 0xF0U);
	CHECK(I2C_10BIT_HEADER_WRITE(0x3FFU) == 0xF6U);
	CHECK(I2C_10BIT_ADDRESS(0x2ABU) == 0xABU);
	CHECK(I2C_7BIT_ADD_WRITE(0xA1U) == 0xA0U);
	CHECK(I2C_7BIT_ADD_READ(0xA0U) == 0xA1U);
	CHECK(I2C_MEM_ADD_MSB(0x1234U) == 0x12U);
	CHECK(I2C_MEM_ADD_LSB(0x1234U) == 0x34U);
}

// ---------------------------------------------------------------------------------------------------------------------------------
// II7 compiled-out scope
// ---------------------------------------------------------------------------------------------------------------------------------

TEST_CASE("II7 the I2C_Ex filter macros are not defined for the F407 (RM0090 27.6.10: I2C_FLTR is F42x/F43x only)") {
#ifdef I2C_FLTR_ANOFF
	CHECK_MESSAGE(false, "I2C_FLTR_ANOFF is defined");
#else
	CHECK(true);
#endif
#ifdef I2C_FLTR_DNF
	CHECK_MESSAGE(false, "I2C_FLTR_DNF is defined");
#else
	CHECK(true);
#endif
}

TEST_CASE("II7 the I2C register callbacks are not compiled in") {
	CHECK(USE_HAL_I2C_REGISTER_CALLBACKS == 0);
}
