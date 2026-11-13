/*
 * P12 T6a group TC: configuration, events, DMA burst and synchronisation functions of the real ST HAL timer driver
 * (stm32f4xx_hal_tim.c: HAL_TIM_ConfigClockSource, SlaveConfigSynchro(_IT), ConfigOCrefClear, ConfigTI1Input, GenerateEvent,
 * DMABurst_*; stm32f4xx_hal_tim_ex.c: ConfigCommutEvent(_IT/_DMA), MasterConfigSynchronization, ConfigBreakDeadTime, RemapConfig)
 * against the timer model (fakes/stm32_hal/HalTim.h) and the DMA model (HalDma.h). Requirements TC1-TC5 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T6A_REQUIREMENTS.md; built into ffb_hal_tests (F407VG) and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 (HAL TIM Generic / Extension Driver) and RM0090 Rev 22:
 *  - SMCR (17.4.3 / 18.4.3 / 19.4.3): SMS bits 2:0 (100 reset, 101 gated, 110 trigger, 111 external clock mode 1), TS bits 6:4 (000-011 ITR0-3,
 *    100 TI1F_ED, 101 TI1FP1, 110 TI2FP2, 111 ETRF), MSM bit 7, ETF bits 11:8, ETPS bits 13:12, ECE bit 14, ETP bit 15.
 *    Note under TS / SMS: the gated mode must not be used with TI1F_ED. 17.3.13: OCREF clear needs ETPS = 00 and ECE = 0.
 *  - CCER (17.4.11): CC1E bit 0, CC1P bit 1, CC1NP bit 3, CC2P bit 5, CC2NP bit 7 (00 rising, 01 falling, 11 both edges).
 *    CCMR1 input capture: IC1F bits 7:4, IC2F bits 15:12; output compare: OC1CE bit 7, OC2CE bit 15 (CCMR2: OC3CE, OC4CE the same).
 *  - CR2: CCPC bit 0, CCUS bit 2, MMS bits 6:4 (000 reset, 001 enable, 010 update, 011 OC1, 100-111 OC1REF-OC4REF), TI1S bit 7.
 *  - DIER (17.4.4): UIE 0, CCxIE x, COMIE 5, TIE 6, UDE 8, CCxDE 8+x, COMDE 13, TDE 14. EGR (17.4.6): UG 0, CCxG x, COMG 5, TG 6, BG 7.
 *  - DCR (17.4.19): DBA bits 4:0, DBL bits 12:8 (burst of DBL + 1 transfers); DMAR (17.4.20): access goes to (DBA + index) * 4 from CR1, the
 *    index counts 0 .. DBL.
 *  - BDTR (17.4.18): DTG 7:0, LOCK 9:8, OSSI 10, OSSR 11, BKE 12, BKP 13, AOE 14, MOE 15; the LOCK bits can be written only once after reset.
 *  - OR (18.4.19 TIM2: ITR1_RMP bits 11:10; 18.4.20 TIM5: TI4_RMP bits 7:6; TIM11 TI1_RMP bits 1:0).
 *  - Table 44 (DMA2): TIM1 UP s5, CH1 s1, CH2 s2, CH3 s6, CH4 / COM s4, TRIG s0 (all channel 6); TIM8 UP s1, CH1 s2, CH2 s3, CH3 s4,
 *    CH4 / TRIG / COM s7 (channel 7). Table 43 (DMA1): TIM2 UP s1, CH1 s5, CH2 s6, CH3 s1, CH4 s7 (channel 3).
 *  - DMA_SxCR (10.5.5): EN 0, TEIE 2, HTIE 3, TCIE 4, DIR bits 7:6 (00 peripheral to memory, 01 memory to peripheral).
 * Time never matters. Every file-local symbol is in the anonymous namespace or starts with tc.
 *
 * NEEDS REVIEW proposals (the lead numbers them):
 *  NEEDS REVIEW 334/a TC1: ConfigClockSource with TI1 / TI2 / TI1ED does not write CCMR1.CCxS (RM0090 lists CCxS = 01 in its procedure); CCER.CC1E is kept.
 *  NEEDS REVIEW 334/b TC1: an invalid clock source returns HAL_ERROR after SMS, TS and the ETR bits were already cleared.
 *  NEEDS REVIEW 334/c TC2: SlaveConfigSynchro with TI1F_ED + gated mode, and with an unknown trigger, returns HAL_ERROR with SMCR already written (and for
 *     the unknown trigger also TIE / TDE untouched).
 *  NEEDS REVIEW 334/d TC3: ConfigOCrefClear with an unknown channel returns HAL_OK and changes no OCxCE bit; ConfigTI1Input ignores lock and state.
 *  NEEDS REVIEW 334/e TC4: the DMA burst start with BurstLength 1 TRANSFER (0) and a NULL buffer returns HAL_OK; Stop of a never started source returns HAL_OK;
 *     a start with DMABurstState RESET runs and leaves the state RESET.
 *  NEEDS REVIEW 334/f TC4: completion / half completion / error callbacks wired by the burst start (callback kinds per source).
 *  NEEDS REVIEW 334/g TC5: ConfigBreakDeadTime writes BDTR from scratch, so it clears MOE; InputTrigger values other than ITR0-3 leave SMCR.TS unchanged
 *     in ConfigCommutEvent.
 *
 *  NEEDS REVIEW 334/h TC1: ConfigClockSource on TIM10 / 11 / 13 / 14 reads and writes the SMCR offset (reserved there, RM0090 ch. 19) and returns HAL_OK.
 *  NEEDS REVIEW 334/i TC5: MasterConfigSynchronization on TIM6 / TIM7 reads the SMCR offset (reserved, RM0090 20.4) before the slave instance test; CR2 MMS is written.
 *  NEEDS REVIEW 334/e also: a burst start with an unknown request source (outside the API contract) returns HAL_ERROR and leaves DMABurstState BUSY.
 *
 * Vendor bug V112 (arguable: a burst start whose DMA stream cannot start returns HAL_ERROR but leaves DMABurstState BUSY).
 */
#include "HalDma.h"
#include "HalFixture.h"
#include "HalHostSystem.h"
#include "HalPeripherals.h"
#include "HalTim.h"
#include "TestHelpers.h"
#include "TimTestSupport.h"

#include <cstdint>
#include <cstring>
#include <string>

namespace {

constexpr uint32_t tcSMCR_MSM = 1u << 7;
constexpr uint32_t tcSMCR_ECE = 1u << 14;
constexpr uint32_t tcDIER_COMIE = 1u << 5;
constexpr uint32_t tcDIER_TIE = 1u << 6;
constexpr uint32_t tcDIER_UIE = 1u;
constexpr uint32_t tcDIER_COMDE = 1u << 13;
constexpr uint32_t tcDIER_TDE = 1u << 14;
constexpr uint32_t tcBDTR_MOE = 1u << 15;

/* Instances with the complete SMCR (ETR bits) and the ones with TS / SMS / MSM only. */
const unsigned tcFull[6] = {1, 8, 2, 3, 4, 5};
const unsigned tcTrig[8] = {1, 8, 2, 3, 4, 5, 9, 12};

struct TcRig {
	TIM_HandleTypeDef h;
	TIM_TypeDef *t;
	explicit TcRig(unsigned tim) : t(timInstance(tim)) {
		hostPeriphResetState(); // a case builds several rigs: start each one from the reset state, bus armed again
		hostRegsReset();
		hostRegsArm();
		std::memset(&h, 0, sizeof h);
		h.Instance = t;
		h.Init.Period = 999;
		REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
		timSupportReset();
	}
	~TcRig() { if (HalFixture::faults() != 0u) MESSAGE("FAULT " << std::string(hostRegsFaultText(0))); CHECK(HalFixture::faults() == 0u); }
	void set(volatile uint32_t *r, uint32_t v) { HalFixture::poke(r, v); }
};

uint32_t tcSmcrMask(unsigned tim) { return (tim == 9 || tim == 12) ? 0xF7u : 0xFFF7u; }

const uint32_t tcPrescalers[4] = {TIM_ETRPRESCALER_DIV1, TIM_ETRPRESCALER_DIV2, TIM_ETRPRESCALER_DIV4, TIM_ETRPRESCALER_DIV8};
const uint32_t tcPrescalerBits[4] = {0u, 1u << 12, 2u << 12, 3u << 12};

/* Common checks: state READY, lock released. */
#define TC_IDLE(r) \
	do { \
		CHECK((r).h.State == HAL_TIM_STATE_READY); \
		CHECK((r).h.Lock == HAL_UNLOCKED); \
	} while (0)

} // namespace

/* ================================================================ TC1: ConfigClockSource */

TEST_CASE_FIXTURE(TimFixture, "TC1 the internal clock source clears SMS, TS and the ETR bits and keeps MSM") {
	for (unsigned tim : tcTrig) {
		CAPTURE(tim);
		TcRig r(tim);
		r.set(&r.t->SMCR, tcSmcrMask(tim)); // every defined bit set
		TIM_ClockConfigTypeDef c{};
		c.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_OK);
		CHECK(r.t->SMCR == tcSMCR_MSM); // RM0090: SMS = 000 selects CK_INT
		TC_IDLE(r);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC1 external clock mode 1 on ETR sets SMS = 111, TS = ETRF and prescaler, polarity and filter") {
	const uint32_t pols[2] = {TIM_CLOCKPOLARITY_NONINVERTED, TIM_CLOCKPOLARITY_INVERTED};
	const uint32_t filters[3] = {0, 5, 15};
	for (unsigned tim : tcFull)
		for (unsigned p = 0; p < 4; ++p)
			for (unsigned po = 0; po < 2; ++po)
				for (uint32_t f : filters) {
					CAPTURE(tim);
					CAPTURE(p);
					CAPTURE(po);
					CAPTURE(f);
					TcRig r(tim);
					r.set(&r.t->SMCR, tcSMCR_MSM);
					TIM_ClockConfigTypeDef c{};
					c.ClockSource = TIM_CLOCKSOURCE_ETRMODE1;
					c.ClockPrescaler = tcPrescalers[p];
					c.ClockPolarity = pols[po];
					c.ClockFilter = f;
					CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_OK);
					// SMS = 111, TS = 111, ETPS, ETP (bit 15), ETF (bits 11:8), MSM kept
					CHECK(r.t->SMCR == (0x77u | tcSMCR_MSM | tcPrescalerBits[p] | (po ? 0x8000u : 0u) | (f << 8)));
					TC_IDLE(r);
				}
}

TEST_CASE_FIXTURE(TimFixture, "TC1 external clock mode 2 sets ECE and the ETR conditioning and leaves SMS and TS at 0") {
	const uint32_t filters[3] = {0, 3, 15};
	for (unsigned tim : tcFull)
		for (unsigned p = 0; p < 4; ++p)
			for (unsigned po = 0; po < 2; ++po)
				for (uint32_t f : filters) {
					CAPTURE(tim);
					CAPTURE(p);
					CAPTURE(po);
					CAPTURE(f);
					TcRig r(tim);
					r.set(&r.t->SMCR, 0x77u); // an earlier mode 1 configuration must be cleared
					TIM_ClockConfigTypeDef c{};
					c.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
					c.ClockPrescaler = tcPrescalers[p];
					c.ClockPolarity = po ? TIM_CLOCKPOLARITY_INVERTED : TIM_CLOCKPOLARITY_NONINVERTED;
					c.ClockFilter = f;
					CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_OK);
					CHECK(r.t->SMCR == (tcSMCR_ECE | tcPrescalerBits[p] | (po ? 0x8000u : 0u) | (f << 8)));
					TC_IDLE(r);
				}
}

TEST_CASE_FIXTURE(TimFixture, "TC1 the TI1 clock source selects TI1FP1 in external clock mode 1 with polarity and filter") {
	const uint32_t pols[3] = {TIM_CLOCKPOLARITY_RISING, TIM_CLOCKPOLARITY_FALLING, TIM_CLOCKPOLARITY_BOTHEDGE};
	const uint32_t polBits[3] = {0x0u, 0x2u, 0xAu}; // CC1NP:CC1P = 00 rising, 01 falling, 11 both
	const uint32_t filters[2] = {0, 0xA};
	for (unsigned tim : tcTrig)
		for (unsigned p = 0; p < 3; ++p)
			for (uint32_t f : filters) {
				CAPTURE(tim);
				CAPTURE(p);
				CAPTURE(f);
				TcRig r(tim);
				TIM_ClockConfigTypeDef c{};
				c.ClockSource = TIM_CLOCKSOURCE_TI1;
				c.ClockPolarity = pols[p];
				c.ClockFilter = f;
				CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_OK);
				CHECK(r.t->SMCR == 0x57u); // TS = 101, SMS = 111
				CHECK((r.t->CCER & 0xAu) == polBits[p]);
				CHECK(((r.t->CCMR1 >> 4) & 0xFu) == f);
				TC_IDLE(r);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TC1 the TI2 clock source selects TI2FP2 in external clock mode 1 with polarity and filter") {
	const uint32_t pols[3] = {TIM_CLOCKPOLARITY_RISING, TIM_CLOCKPOLARITY_FALLING, TIM_CLOCKPOLARITY_BOTHEDGE};
	const uint32_t polBits[3] = {0x0u, 0x20u, 0xA0u}; // CC2NP:CC2P
	const uint32_t filters[2] = {0, 0x9};
	for (unsigned tim : tcTrig)
		for (unsigned p = 0; p < 3; ++p)
			for (uint32_t f : filters) {
				CAPTURE(tim);
				CAPTURE(p);
				CAPTURE(f);
				TcRig r(tim);
				TIM_ClockConfigTypeDef c{};
				c.ClockSource = TIM_CLOCKSOURCE_TI2;
				c.ClockPolarity = pols[p];
				c.ClockFilter = f;
				CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_OK);
				CHECK(r.t->SMCR == 0x67u); // TS = 110, SMS = 111
				CHECK((r.t->CCER & 0xA0u) == polBits[p]);
				CHECK(((r.t->CCMR1 >> 12) & 0xFu) == f);
				TC_IDLE(r);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TC1 the TI1ED clock source selects TI1F_ED in external clock mode 1 with polarity and filter") {
	const uint32_t pols[3] = {TIM_CLOCKPOLARITY_RISING, TIM_CLOCKPOLARITY_FALLING, TIM_CLOCKPOLARITY_BOTHEDGE};
	const uint32_t polBits[3] = {0x0u, 0x2u, 0xAu};
	for (unsigned tim : tcTrig)
		for (unsigned p = 0; p < 3; ++p) {
			CAPTURE(tim);
			CAPTURE(p);
			TcRig r(tim);
			TIM_ClockConfigTypeDef c{};
			c.ClockSource = TIM_CLOCKSOURCE_TI1ED;
			c.ClockPolarity = pols[p];
			c.ClockFilter = 7;
			CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_OK);
			CHECK(r.t->SMCR == 0x47u); // TS = 100, SMS = 111
			CHECK((r.t->CCER & 0xAu) == polBits[p]);
			CHECK(((r.t->CCMR1 >> 4) & 0xFu) == 7u);
			TC_IDLE(r);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TC1 the ITR0 to ITR3 clock sources select that trigger in external clock mode 1") {
	const uint32_t src[4] = {TIM_CLOCKSOURCE_ITR0, TIM_CLOCKSOURCE_ITR1, TIM_CLOCKSOURCE_ITR2, TIM_CLOCKSOURCE_ITR3};
	for (unsigned tim : tcTrig)
		for (uint32_t i = 0; i < 4; ++i) {
			CAPTURE(tim);
			CAPTURE(i);
			TcRig r(tim);
			r.set(&r.t->SMCR, 0x70u); // an earlier TS must be replaced
			TIM_ClockConfigTypeDef c{};
			c.ClockSource = src[i];
			CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_OK);
			CHECK(r.t->SMCR == ((i << 4) | 0x7u));
			TC_IDLE(r);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TC1 a TI clock source configuration keeps CC1E / CC2E as they were") {
	// characterization of the preserved enable bit is NEEDS REVIEW 334/a; the polarity bits are replaced
	TcRig r(3);
	r.set(&r.t->CCER, 0x1u | 0x2u); // CC1E, CC1P
	TIM_ClockConfigTypeDef c{};
	c.ClockSource = TIM_CLOCKSOURCE_TI1;
	c.ClockPolarity = TIM_CLOCKPOLARITY_RISING;
	CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_OK);
	CHECK(r.t->CCER == 0x1u);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC1 the TI1 clock source does not change CC1S of CCMR1") {
	// NEEDS REVIEW 334/a: RM0090 18.3.? lists CC1S = 01 in the external clock mode 1 procedure; the HAL writes only the filter
	TcRig r(2);
	r.set(&r.t->CCMR1, 0x0u);
	TIM_ClockConfigTypeDef c{};
	c.ClockSource = TIM_CLOCKSOURCE_TI1;
	c.ClockPolarity = TIM_CLOCKPOLARITY_RISING;
	CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_OK);
	CHECK((r.t->CCMR1 & 0x3u) == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC1 an unknown clock source returns HAL_ERROR after SMS, TS and the ETR bits were cleared") {
	// NEEDS REVIEW 334/b
	TcRig r(2);
	r.set(&r.t->SMCR, 0xFFF7u);
	TIM_ClockConfigTypeDef c{};
	c.ClockSource = 0x00030000u;
	CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_ERROR);
	CHECK(r.t->SMCR == tcSMCR_MSM);
	TC_IDLE(r);
}

TEST_CASE_FIXTURE(TimFixture, "TC1 a locked handle returns HAL_BUSY and leaves SMCR and the lock") {
	TcRig r(2);
	r.set(&r.t->SMCR, 0x77u);
	r.h.Lock = HAL_LOCKED;
	TIM_ClockConfigTypeDef c{};
	c.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_BUSY);
	CHECK(r.t->SMCR == 0x77u);
	CHECK(r.h.Lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(TimFixture, "TC1 a configuration ends with the state READY whatever the state was" ) {
	TcRig r(2);
	r.h.State = HAL_TIM_STATE_ERROR;
	TIM_ClockConfigTypeDef c{};
	c.ClockSource = TIM_CLOCKSOURCE_ITR1;
	CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_OK);
	TC_IDLE(r);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC1 ConfigClockSource internal on TIM10 / 11 / 13 / 14 returns HAL_OK and accesses the reserved SMCR offset") {
	// NEEDS REVIEW 334/h: the HAL reads and writes TIMx_SMCR, offset 0x08, reserved on these timers (RM0090 ch. 19 register map)
	const unsigned singles[4] = {10, 11, 13, 14};
	for (unsigned tim : singles) {
		CAPTURE(tim);
		TcRig r(tim);
		TIM_ClockConfigTypeDef c{};
		c.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_OK);
		CHECK(HalFixture::faults() > 0u);
		HalFixture::clearFaults();
		TC_IDLE(r);
	}
}

/* ================================================================ TC2: SlaveConfigSynchro */

namespace {
const uint32_t tcModes[5] = {TIM_SLAVEMODE_DISABLE, TIM_SLAVEMODE_RESET, TIM_SLAVEMODE_GATED, TIM_SLAVEMODE_TRIGGER, TIM_SLAVEMODE_EXTERNAL1};
const uint32_t tcModeBits[5] = {0u, 4u, 5u, 6u, 7u};

HAL_StatusTypeDef tcSlave(TcRig &r, bool it, const TIM_SlaveConfigTypeDef &c) { return it ? HAL_TIM_SlaveConfigSynchro_IT(&r.h, &c) : HAL_TIM_SlaveConfigSynchro(&r.h, &c); }
} // namespace

TEST_CASE_FIXTURE(TimFixture, "TC2 ETRF trigger writes TS = 111, the mode and the ETR prescaler, polarity and filter") {
	for (int it = 0; it < 2; ++it)
		for (unsigned tim : tcFull)
			for (unsigned m = 0; m < 5; ++m)
				for (unsigned p = 0; p < 4; ++p) {
					CAPTURE(it);
					CAPTURE(tim);
					CAPTURE(m);
					CAPTURE(p);
					TcRig r(tim);
					r.set(&r.t->SMCR, tcSMCR_MSM);
					TIM_SlaveConfigTypeDef c{};
					c.SlaveMode = tcModes[m];
					c.InputTrigger = TIM_TS_ETRF;
					c.TriggerPrescaler = tcPrescalers[p];
					c.TriggerPolarity = TIM_TRIGGERPOLARITY_INVERTED;
					c.TriggerFilter = 6;
					CHECK(tcSlave(r, it != 0, c) == HAL_OK);
					CHECK(r.t->SMCR == (tcSMCR_MSM | 0x70u | tcModeBits[m] | tcPrescalerBits[p] | 0x8000u | (6u << 8)));
					TC_IDLE(r);
				}
}

TEST_CASE_FIXTURE(TimFixture, "TC2 TI1FP1 trigger writes TS = 101, the mode, the CC1 polarity and the IC1 filter") {
	const uint32_t pols[3] = {TIM_TRIGGERPOLARITY_RISING, TIM_TRIGGERPOLARITY_FALLING, TIM_TRIGGERPOLARITY_BOTHEDGE};
	const uint32_t polBits[3] = {0u, 2u, 0xAu};
	for (unsigned tim : tcTrig)
		for (unsigned m = 0; m < 5; ++m)
			for (unsigned p = 0; p < 3; ++p) {
				CAPTURE(tim);
				CAPTURE(m);
				CAPTURE(p);
				TcRig r(tim);
				TIM_SlaveConfigTypeDef c{};
				c.SlaveMode = tcModes[m];
				c.InputTrigger = TIM_TS_TI1FP1;
				c.TriggerPolarity = pols[p];
				c.TriggerFilter = 0xC;
				CHECK(HAL_TIM_SlaveConfigSynchro(&r.h, &c) == HAL_OK);
				CHECK(r.t->SMCR == (0x50u | tcModeBits[m]));
				CHECK((r.t->CCER & 0xAu) == polBits[p]);
				CHECK(((r.t->CCMR1 >> 4) & 0xFu) == 0xCu);
				TC_IDLE(r);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TC2 TI2FP2 trigger writes TS = 110, the mode, the CC2 polarity and the IC2 filter") {
	const uint32_t pols[3] = {TIM_TRIGGERPOLARITY_RISING, TIM_TRIGGERPOLARITY_FALLING, TIM_TRIGGERPOLARITY_BOTHEDGE};
	const uint32_t polBits[3] = {0u, 0x20u, 0xA0u};
	for (unsigned tim : tcTrig)
		for (unsigned m = 0; m < 5; ++m)
			for (unsigned p = 0; p < 3; ++p) {
				CAPTURE(tim);
				CAPTURE(m);
				CAPTURE(p);
				TcRig r(tim);
				TIM_SlaveConfigTypeDef c{};
				c.SlaveMode = tcModes[m];
				c.InputTrigger = TIM_TS_TI2FP2;
				c.TriggerPolarity = pols[p];
				c.TriggerFilter = 0x3;
				CHECK(HAL_TIM_SlaveConfigSynchro(&r.h, &c) == HAL_OK);
				CHECK(r.t->SMCR == (0x60u | tcModeBits[m]));
				CHECK((r.t->CCER & 0xA0u) == polBits[p]);
				CHECK(((r.t->CCMR1 >> 12) & 0xFu) == 0x3u);
				TC_IDLE(r);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TC2 TI1F_ED trigger writes TS = 100, the mode and the IC1 filter for every mode but gated") {
	for (int it = 0; it < 2; ++it)
		for (unsigned tim : tcTrig)
			for (unsigned m = 0; m < 5; ++m) {
				if (tcModes[m] == TIM_SLAVEMODE_GATED) continue;
				CAPTURE(it);
				CAPTURE(tim);
				CAPTURE(m);
				TcRig r(tim);
				r.set(&r.t->CCER, 0x1u); // CC1E must survive
				TIM_SlaveConfigTypeDef c{};
				c.SlaveMode = tcModes[m];
				c.InputTrigger = TIM_TS_TI1F_ED;
				c.TriggerFilter = 0xF;
				CHECK(tcSlave(r, it != 0, c) == HAL_OK);
				CHECK(r.t->SMCR == (0x40u | tcModeBits[m]));
				CHECK(((r.t->CCMR1 >> 4) & 0xFu) == 0xFu);
				CHECK((r.t->CCER & 0x1u) == 0x1u);
				TC_IDLE(r);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TC2 TI1F_ED trigger in gated mode is rejected with HAL_ERROR (RM0090: gated mode must not be used with TI1F_ED)") {
	for (int it = 0; it < 2; ++it) {
		CAPTURE(it);
		TcRig r(2);
		r.set(&r.t->DIER, tcDIER_TIE);
		TIM_SlaveConfigTypeDef c{};
		c.SlaveMode = TIM_SLAVEMODE_GATED;
		c.InputTrigger = TIM_TS_TI1F_ED;
		CHECK(tcSlave(r, it != 0, c) == HAL_ERROR);
		CHECK(r.t->DIER == tcDIER_TIE); // no interrupt change on the error path
		TC_IDLE(r);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC2 the rejected TI1F_ED gated configuration leaves SMCR written with TS = 100 and SMS = 101") {
	// NEEDS REVIEW 334/c
	TcRig r(2);
	TIM_SlaveConfigTypeDef c{};
	c.SlaveMode = TIM_SLAVEMODE_GATED;
	c.InputTrigger = TIM_TS_TI1F_ED;
	CHECK(HAL_TIM_SlaveConfigSynchro(&r.h, &c) == HAL_ERROR);
	CHECK(r.t->SMCR == 0x45u);
}

TEST_CASE_FIXTURE(TimFixture, "TC2 ITR0 to ITR3 triggers write the trigger and the mode only") {
	const uint32_t trg[4] = {TIM_TS_ITR0, TIM_TS_ITR1, TIM_TS_ITR2, TIM_TS_ITR3};
	for (unsigned tim : tcTrig)
		for (uint32_t i = 0; i < 4; ++i)
			for (unsigned m = 0; m < 5; ++m) {
				CAPTURE(tim);
				CAPTURE(i);
				CAPTURE(m);
				TcRig r(tim);
				r.set(&r.t->CCMR1, 0xF0F0u);
				TIM_SlaveConfigTypeDef c{};
				c.SlaveMode = tcModes[m];
				c.InputTrigger = trg[i];
				CHECK(HAL_TIM_SlaveConfigSynchro(&r.h, &c) == HAL_OK);
				CHECK(r.t->SMCR == ((i << 4) | tcModeBits[m]));
				CHECK(r.t->CCMR1 == 0xF0F0u);
				TC_IDLE(r);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TC2 the polling variant clears the trigger interrupt and trigger DMA request, the IT variant enables TIE and clears TDE") {
	for (unsigned tim : tcTrig) {
		CAPTURE(tim);
		const uint32_t others = (tim == 9 || tim == 12) ? 0x3u : 0x103u; // UIE, CC1IE (+ UDE)
		TIM_SlaveConfigTypeDef c{};
		c.SlaveMode = TIM_SLAVEMODE_RESET;
		c.InputTrigger = TIM_TS_ITR0;
		{
			TcRig r(tim);
			r.set(&r.t->DIER, (tim == 9 || tim == 12) ? 0x47u : 0x5F5Fu);
			CHECK(HAL_TIM_SlaveConfigSynchro(&r.h, &c) == HAL_OK);
			CHECK((r.t->DIER & (tcDIER_TIE | tcDIER_TDE)) == 0u);
			CHECK((r.t->DIER & others) == others); // the other bits are kept
		}
		{
			TcRig r(tim);
			r.set(&r.t->DIER, (tim == 9 || tim == 12) ? 0x07u : 0x1F1Fu | tcDIER_TDE);
			CHECK(HAL_TIM_SlaveConfigSynchro_IT(&r.h, &c) == HAL_OK);
			CHECK((r.t->DIER & tcDIER_TIE) == tcDIER_TIE);
			CHECK((r.t->DIER & tcDIER_TDE) == 0u);
			CHECK((r.t->DIER & others) == others);
		}
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC2 an IT configuration enables the trigger interrupt line of the handler") {
	TcRig r(3);
	TIM_SlaveConfigTypeDef c{};
	c.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	c.InputTrigger = TIM_TS_ITR1;
	REQUIRE(HAL_TIM_SlaveConfigSynchro_IT(&r.h, &c) == HAL_OK);
	CHECK(hostTimIrqPending(3) == 0u);
	hostTimRaiseFlags(3, 1u << 6); // TIF
	CHECK(hostTimIrqPending(3) == (1u << 6));
	HAL_TIM_IRQHandler(&r.h);
	CHECK(timCounts.trigger == 1u);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC2 an unknown trigger returns HAL_ERROR after SMCR was written and leaves DIER") {
	// NEEDS REVIEW 334/c
	for (int it = 0; it < 2; ++it) {
		CAPTURE(it);
		TcRig r(2);
		r.set(&r.t->DIER, tcDIER_TDE);
		TIM_SlaveConfigTypeDef c{};
		c.SlaveMode = TIM_SLAVEMODE_TRIGGER;
		c.InputTrigger = TIM_TS_NONE;
		CHECK(tcSlave(r, it != 0, c) == HAL_ERROR);
		CHECK(r.t->DIER == tcDIER_TDE);
		CHECK((r.t->SMCR & 7u) == 6u);
		TC_IDLE(r);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC2 a locked handle returns HAL_BUSY without any register change") {
	for (int it = 0; it < 2; ++it) {
		TcRig r(2);
		r.set(&r.t->SMCR, 0x25u);
		r.h.Lock = HAL_LOCKED;
		TIM_SlaveConfigTypeDef c{};
		c.SlaveMode = TIM_SLAVEMODE_RESET;
		c.InputTrigger = TIM_TS_ITR3;
		CHECK(tcSlave(r, it != 0, c) == HAL_BUSY);
		CHECK(r.t->SMCR == 0x25u);
		CHECK(r.h.Lock == HAL_LOCKED);
	}
}

/* ================================================================ TC3: OCREF clear, TI1 input, GenerateEvent */

namespace {
volatile uint32_t *tcCcmrOf(TIM_TypeDef *t, unsigned ch) { return ch <= 2 ? &t->CCMR1 : &t->CCMR2; }
uint32_t tcOcce(unsigned ch) { return (ch & 1u) ? 0x80u : 0x8000u; }
const uint32_t tcChan[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
} // namespace

TEST_CASE_FIXTURE(TimFixture, "TC3 OCREF clear with the ETR source enables OCxCE of the channel only and configures ETR") {
	for (unsigned tim : tcFull)
		for (unsigned ch = 1; ch <= 4; ++ch) {
			CAPTURE(tim);
			CAPTURE(ch);
			TcRig r(tim);
			r.set(&r.t->SMCR, tcSMCR_ECE | tcSMCR_MSM); // ECE must be cleared (RM0090 17.3.13)
			TIM_ClearInputConfigTypeDef c{};
			c.ClearInputState = ENABLE;
			c.ClearInputSource = TIM_CLEARINPUTSOURCE_ETR;
			c.ClearInputPolarity = TIM_CLEARINPUTPOLARITY_INVERTED;
			c.ClearInputPrescaler = TIM_CLEARINPUTPRESCALER_DIV1;
			c.ClearInputFilter = 9;
			CHECK(HAL_TIM_ConfigOCrefClear(&r.h, &c, tcChan[ch - 1]) == HAL_OK);
			CHECK(r.t->SMCR == (tcSMCR_MSM | 0x8000u | (9u << 8)));
			CHECK(*tcCcmrOf(r.t, ch) == tcOcce(ch));
			CHECK(*tcCcmrOf(r.t, ch <= 2 ? 3 : 1) == 0u);
			TC_IDLE(r);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TC3 OCREF clear with state DISABLE clears OCxCE of that channel and keeps the others") {
	for (unsigned tim : tcFull)
		for (unsigned ch = 1; ch <= 4; ++ch) {
			CAPTURE(tim);
			CAPTURE(ch);
			TcRig r(tim);
			r.set(&r.t->CCMR1, 0x8080u);
			r.set(&r.t->CCMR2, 0x8080u);
			TIM_ClearInputConfigTypeDef c{};
			c.ClearInputState = DISABLE;
			c.ClearInputSource = TIM_CLEARINPUTSOURCE_NONE;
			CHECK(HAL_TIM_ConfigOCrefClear(&r.h, &c, tcChan[ch - 1]) == HAL_OK);
			CHECK(*tcCcmrOf(r.t, ch) == (0x8080u & ~tcOcce(ch)));
			CHECK(*tcCcmrOf(r.t, ch <= 2 ? 3 : 1) == 0x8080u);
			TC_IDLE(r);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TC3 OCREF clear with the NONE source clears the ETR bits of SMCR and keeps SMS, TS and MSM") {
	TcRig r(1);
	r.set(&r.t->SMCR, 0xFFF7u);
	TIM_ClearInputConfigTypeDef c{};
	c.ClearInputState = ENABLE;
	c.ClearInputSource = TIM_CLEARINPUTSOURCE_NONE;
	CHECK(HAL_TIM_ConfigOCrefClear(&r.h, &c, TIM_CHANNEL_1) == HAL_OK);
	CHECK(r.t->SMCR == 0x00F7u);
	CHECK((r.t->CCMR1 & 0x80u) == 0x80u);
}

TEST_CASE_FIXTURE(TimFixture, "TC3 OCREF clear with the ETR source and a prescaler other than off is rejected without a change (RM0090 17.3.13)") {
	for (unsigned p = 1; p < 4; ++p) {
		CAPTURE(p);
		TcRig r(2);
		r.set(&r.t->SMCR, 0x0u);
		TIM_ClearInputConfigTypeDef c{};
		c.ClearInputState = ENABLE;
		c.ClearInputSource = TIM_CLEARINPUTSOURCE_ETR;
		c.ClearInputPrescaler = tcPrescalers[p];
		CHECK(HAL_TIM_ConfigOCrefClear(&r.h, &c, TIM_CHANNEL_1) == HAL_ERROR);
		CHECK(r.t->SMCR == 0u);
		CHECK((r.t->CCMR1 & 0x80u) == 0u);
		TC_IDLE(r);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC3 OCREF clear with an unknown source returns HAL_ERROR and does not touch OCxCE") {
	TcRig r(2);
	TIM_ClearInputConfigTypeDef c{};
	c.ClearInputState = ENABLE;
	c.ClearInputSource = 7;
	CHECK(HAL_TIM_ConfigOCrefClear(&r.h, &c, TIM_CHANNEL_1) == HAL_ERROR);
	CHECK(r.t->CCMR1 == 0u);
	TC_IDLE(r);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC3 OCREF clear with an unknown channel returns HAL_OK and changes no OCxCE bit") {
	// NEEDS REVIEW 334/d
	TcRig r(2);
	TIM_ClearInputConfigTypeDef c{};
	c.ClearInputState = ENABLE;
	c.ClearInputSource = TIM_CLEARINPUTSOURCE_NONE;
	CHECK(HAL_TIM_ConfigOCrefClear(&r.h, &c, 0x14u) == HAL_OK);
	CHECK(r.t->CCMR1 == 0u);
	CHECK(r.t->CCMR2 == 0u);
	TC_IDLE(r);
}

TEST_CASE_FIXTURE(TimFixture, "TC3 OCREF clear on a locked handle returns HAL_BUSY") {
	TcRig r(2);
	r.h.Lock = HAL_LOCKED;
	TIM_ClearInputConfigTypeDef c{};
	c.ClearInputState = ENABLE;
	c.ClearInputSource = TIM_CLEARINPUTSOURCE_NONE;
	CHECK(HAL_TIM_ConfigOCrefClear(&r.h, &c, TIM_CHANNEL_1) == HAL_BUSY);
	CHECK(r.t->CCMR1 == 0u);
	CHECK(r.h.Lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(TimFixture, "TC3 ConfigTI1Input selects the CH1 pin or the XOR of CH1 to CH3 through TI1S and keeps the other CR2 bits") {
	const unsigned tims[6] = {1, 8, 2, 3, 4, 5};
	for (unsigned tim : tims) {
		CAPTURE(tim);
		TcRig r(tim);
		const uint32_t keep = (tim == 1 || tim == 8) ? 0x0105u : 0x0008u; // OIS1 + CCUS / CCDS
		r.set(&r.t->CR2, keep);
		CHECK(HAL_TIM_ConfigTI1Input(&r.h, TIM_TI1SELECTION_XORCOMBINATION) == HAL_OK);
		CHECK(r.t->CR2 == (keep | 0x80u));
		CHECK(HAL_TIM_ConfigTI1Input(&r.h, TIM_TI1SELECTION_CH1) == HAL_OK);
		CHECK(r.t->CR2 == keep);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC3 ConfigTI1Input ignores the lock and the state of the handle") {
	// NEEDS REVIEW 334/d
	TcRig r(2);
	r.h.Lock = HAL_LOCKED;
	r.h.State = HAL_TIM_STATE_BUSY;
	CHECK(HAL_TIM_ConfigTI1Input(&r.h, TIM_TI1SELECTION_XORCOMBINATION) == HAL_OK);
	CHECK((r.t->CR2 & 0x80u) == 0x80u);
	CHECK(r.h.Lock == HAL_LOCKED);
	CHECK(r.h.State == HAL_TIM_STATE_BUSY);
}

TEST_CASE_FIXTURE(TimFixture, "TC3 GenerateEvent UPDATE reloads the counter and sets UIF") {
	for (unsigned tim : {1u, 2u, 3u, 5u, 6u, 9u, 10u}) {
		CAPTURE(tim);
		TcRig r(tim);
		hostTimSetCounter(tim, 123);
		CHECK(HAL_TIM_GenerateEvent(&r.h, TIM_EVENTSOURCE_UPDATE) == HAL_OK);
		CHECK(r.t->CNT == 0u); // RM0090: UG re-initializes the counter
		CHECK((r.t->SR & 1u) == 1u);
		CHECK(r.t->EGR == 0u); // EGR bits are cleared by hardware
		TC_IDLE(r);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC3 GenerateEvent CC1 to CC4 sets only the flag of that output compare channel") {
	for (unsigned tim : tcFull)
		for (unsigned ch = 1; ch <= 4; ++ch) {
			CAPTURE(tim);
			CAPTURE(ch);
			TcRig r(tim);
			CHECK(HAL_TIM_GenerateEvent(&r.h, 1u << ch) == HAL_OK);
			CHECK((r.t->SR & 0x1Fu) == (1u << ch));
			TC_IDLE(r);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TC3 GenerateEvent CC1 / CC2 on TIM9 and CC1 on TIM10 set that flag") {
	{
		TcRig r(9);
		CHECK(HAL_TIM_GenerateEvent(&r.h, TIM_EVENTSOURCE_CC2) == HAL_OK);
		CHECK((r.t->SR & 0x7u) == 4u);
	}
	{
		TcRig r(10);
		CHECK(HAL_TIM_GenerateEvent(&r.h, TIM_EVENTSOURCE_CC1) == HAL_OK);
		CHECK((r.t->SR & 0x3u) == 2u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC3 GenerateEvent CC1 on an input channel captures the counter and a second one sets the overcapture flag") {
	TcRig r(2);
	r.set(&r.t->CCMR1, 0x1u); // CC1S = 01: input
	hostTimSetCounter(2, 0x1234u);
	CHECK(HAL_TIM_GenerateEvent(&r.h, TIM_EVENTSOURCE_CC1) == HAL_OK);
	CHECK(r.t->CCR1 == 0x1234u);
	CHECK((r.t->SR & 0x2u) == 0x2u);
	CHECK((r.t->SR & 0x200u) == 0u);
	hostTimSetCounter(2, 0x4321u);
	CHECK(HAL_TIM_GenerateEvent(&r.h, TIM_EVENTSOURCE_CC1) == HAL_OK);
	CHECK(r.t->CCR1 == 0x4321u);
	CHECK((r.t->SR & 0x200u) == 0x200u); // CC1OF
}

TEST_CASE_FIXTURE(TimFixture, "TC3 GenerateEvent COM sets COMIF, TRIGGER sets TIF and BREAK sets BIF and clears MOE") {
	TcRig r(1);
	r.set(&r.t->BDTR, tcBDTR_MOE);
	CHECK(HAL_TIM_GenerateEvent(&r.h, TIM_EVENTSOURCE_COM) == HAL_OK);
	CHECK((r.t->SR & 0xE0u) == 0x20u);
	CHECK(HAL_TIM_GenerateEvent(&r.h, TIM_EVENTSOURCE_TRIGGER) == HAL_OK);
	CHECK((r.t->SR & 0xE0u) == 0x60u);
	CHECK((r.t->BDTR & tcBDTR_MOE) == tcBDTR_MOE);
	CHECK(HAL_TIM_GenerateEvent(&r.h, TIM_EVENTSOURCE_BREAK) == HAL_OK);
	CHECK((r.t->SR & 0xE0u) == 0xE0u);
	CHECK((r.t->BDTR & tcBDTR_MOE) == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "TC3 GenerateEvent TRIGGER on a general purpose timer sets TIF and the interrupt line reaches the handler") {
	TcRig r(4);
	r.set(&r.t->DIER, tcDIER_TIE);
	CHECK(HAL_TIM_GenerateEvent(&r.h, TIM_EVENTSOURCE_TRIGGER) == HAL_OK);
	CHECK(hostTimIrqPending(4) == (1u << 6));
	HAL_TIM_IRQHandler(&r.h);
	CHECK(timCounts.trigger == 1u);
}

TEST_CASE_FIXTURE(TimFixture, "TC3 GenerateEvent with several sources sets all their flags") {
	TcRig r(1);
	CHECK(HAL_TIM_GenerateEvent(&r.h, TIM_EVENTSOURCE_UPDATE | TIM_EVENTSOURCE_CC1 | TIM_EVENTSOURCE_CC3 | TIM_EVENTSOURCE_COM) == HAL_OK);
	CHECK((r.t->SR & 0xFFu) == (0x01u | 0x02u | 0x08u | 0x20u));
}

TEST_CASE_FIXTURE(TimFixture, "TC3 GenerateEvent ignores a source bit the timer does not have (TIM6 has only UG)") {
	TcRig r(6);
	CHECK(HAL_TIM_GenerateEvent(&r.h, TIM_EVENTSOURCE_CC1 | TIM_EVENTSOURCE_COM) == HAL_OK);
	CHECK(r.t->SR == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "TC3 GenerateEvent on a locked handle returns HAL_BUSY and generates nothing") {
	TcRig r(2);
	r.h.Lock = HAL_LOCKED;
	CHECK(HAL_TIM_GenerateEvent(&r.h, TIM_EVENTSOURCE_UPDATE) == HAL_BUSY);
	CHECK(r.t->SR == 0u);
	CHECK(r.h.Lock == HAL_LOCKED);
}

/* ================================================================ TC4: DMA burst */

namespace {

struct TcSrc {
	uint32_t src;     // TIM_DMA_*
	uint16_t id;      // TIM_DMA_ID_*
	unsigned stream;
	uint32_t channel; // DMA_CHANNEL_x
	bool cc;          // capture / compare source
};

/* Index = TIM_DMA_ID_*. */
const TcSrc tcTim1[7] = {{TIM_DMA_UPDATE, TIM_DMA_ID_UPDATE, 5, DMA_CHANNEL_6, false}, {TIM_DMA_CC1, TIM_DMA_ID_CC1, 1, DMA_CHANNEL_6, true},
                         {TIM_DMA_CC2, TIM_DMA_ID_CC2, 2, DMA_CHANNEL_6, true},       {TIM_DMA_CC3, TIM_DMA_ID_CC3, 6, DMA_CHANNEL_6, true},
                         {TIM_DMA_CC4, TIM_DMA_ID_CC4, 4, DMA_CHANNEL_6, true},       {TIM_DMA_COM, TIM_DMA_ID_COMMUTATION, 4, DMA_CHANNEL_6, false},
                         {TIM_DMA_TRIGGER, TIM_DMA_ID_TRIGGER, 0, DMA_CHANNEL_6, false}};
const TcSrc tcTim8[7] = {{TIM_DMA_UPDATE, TIM_DMA_ID_UPDATE, 1, DMA_CHANNEL_7, false}, {TIM_DMA_CC1, TIM_DMA_ID_CC1, 2, DMA_CHANNEL_7, true},
                         {TIM_DMA_CC2, TIM_DMA_ID_CC2, 3, DMA_CHANNEL_7, true},       {TIM_DMA_CC3, TIM_DMA_ID_CC3, 4, DMA_CHANNEL_7, true},
                         {TIM_DMA_CC4, TIM_DMA_ID_CC4, 7, DMA_CHANNEL_7, true},       {TIM_DMA_COM, TIM_DMA_ID_COMMUTATION, 7, DMA_CHANNEL_7, false},
                         {TIM_DMA_TRIGGER, TIM_DMA_ID_TRIGGER, 7, DMA_CHANNEL_7, false}};
const TcSrc tcTim2[5] = {{TIM_DMA_UPDATE, TIM_DMA_ID_UPDATE, 1, DMA_CHANNEL_3, false}, {TIM_DMA_CC1, TIM_DMA_ID_CC1, 5, DMA_CHANNEL_3, true},
                         {TIM_DMA_CC2, TIM_DMA_ID_CC2, 6, DMA_CHANNEL_3, true},       {TIM_DMA_CC3, TIM_DMA_ID_CC3, 1, DMA_CHANNEL_3, true},
                         {TIM_DMA_CC4, TIM_DMA_ID_CC4, 7, DMA_CHANNEL_3, true}};

DMA_Stream_TypeDef *tcStream(unsigned controller, unsigned s) {
	DMA_Stream_TypeDef *const d1[8] = {DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7};
	DMA_Stream_TypeDef *const d2[8] = {DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7};
	return controller == 0 ? d1[s] : d2[s];
}

constexpr uint32_t tcDmaEN = 1u << 0, tcDmaTEIE = 1u << 2, tcDmaHTIE = 1u << 3, tcDmaTCIE = 1u << 4;

/* A timer with DMA handles linked for every source of its table (direction: memory to peripheral for write bursts, peripheral to memory for reads). */
struct TcBurst : TcRig {
	DMA_HandleTypeDef dma[7];
	const TcSrc *tab;
	unsigned count, controller;
	uint32_t buf[32];
	TcBurst(unsigned tim, bool write, uint32_t mode = DMA_NORMAL) : TcRig(tim), tab(tim == 1 ? tcTim1 : tim == 8 ? tcTim8 : tcTim2), count(tim == 2 ? 5 : 7), controller(tim == 2 ? 0 : 1) {
		std::memset(dma, 0, sizeof dma);
		std::memset(buf, 0, sizeof buf);
		for (unsigned i = 0; i < count; ++i) {
			DMA_HandleTypeDef &d = dma[tab[i].id];
			d.Instance = tcStream(controller, tab[i].stream);
			d.Init.Channel = tab[i].channel;
			d.Init.Direction = write ? DMA_MEMORY_TO_PERIPH : DMA_PERIPH_TO_MEMORY;
			d.Init.PeriphInc = DMA_PINC_DISABLE;
			d.Init.MemInc = DMA_MINC_ENABLE;
			d.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
			d.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
			d.Init.Mode = mode;
			d.Init.Priority = DMA_PRIORITY_LOW;
			d.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
			d.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
			d.Init.MemBurst = DMA_MBURST_SINGLE;
			d.Init.PeriphBurst = DMA_PBURST_SINGLE;
			REQUIRE(HAL_DMA_Init(&d) == HAL_OK);
			d.Parent = &h;
			h.hdma[tab[i].id] = &d;
		}
		timSupportReset();
	}
	DMA_Stream_TypeDef *stream(unsigned i) { return tcStream(controller, tab[i].stream); }
	uint32_t addr() const { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buf)); }
	uint32_t dmarAddr() const { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&t->DMAR)); }
	void complete(unsigned i) {
		hostDmaComplete(controller, tab[i].stream);
		HAL_DMA_IRQHandler(&dma[tab[i].id]);
	}
};

unsigned tcTims[3] = {1, 8, 2};
} // namespace

#define TC_FOR_BURSTS(tim, i, write) \
	for (unsigned tim : tcTims) \
		for (unsigned i = 0; i < (tim == 2 ? 5u : 7u); ++i)

TEST_CASE_FIXTURE(TimFixture, "TC4 WriteStart programs DCR, enables that request, starts the stream towards DMAR and sets the burst state BUSY") {
	TC_FOR_BURSTS(tim, i, true) {
		CAPTURE(tim);
		CAPTURE(i);
		TcBurst b(tim, true);
		REQUIRE(b.h.DMABurstState == HAL_DMA_BURST_STATE_READY);
		CHECK(HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, b.tab[i].src, b.buf, TIM_DMABURSTLENGTH_3TRANSFERS) == HAL_OK);
		CHECK(b.t->DCR == (0x0Du | 0x200u)); // DBA = 13 (CCR1), DBL = 2
		CHECK((b.t->DIER & 0x7F00u) == b.tab[i].src);
		DMA_Stream_TypeDef *s = b.stream(i);
		CHECK((s->CR & tcDmaEN) == tcDmaEN);
		CHECK((s->CR & 0xC0u) == 0x40u); // memory to peripheral
		CHECK((s->CR & (tcDmaTCIE | tcDmaHTIE | tcDmaTEIE)) == (tcDmaTCIE | tcDmaHTIE | tcDmaTEIE));
		CHECK(s->NDTR == 3u); // one burst of DBL + 1 = 3 transfers
		CHECK(s->PAR == b.dmarAddr());
		CHECK(s->M0AR == b.addr());
		CHECK(b.h.DMABurstState == HAL_DMA_BURST_STATE_BUSY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC4 ReadStart programs DCR, enables that request and starts the stream from DMAR to the buffer") {
	TC_FOR_BURSTS(tim, i, false) {
		CAPTURE(tim);
		CAPTURE(i);
		TcBurst b(tim, false);
		CHECK(HAL_TIM_DMABurst_ReadStart(&b.h, TIM_DMABASE_CCR1, b.tab[i].src, b.buf, TIM_DMABURSTLENGTH_4TRANSFERS) == HAL_OK);
		CHECK(b.t->DCR == (0x0Du | 0x300u));
		CHECK((b.t->DIER & 0x7F00u) == b.tab[i].src);
		DMA_Stream_TypeDef *s = b.stream(i);
		CHECK((s->CR & tcDmaEN) == tcDmaEN);
		CHECK((s->CR & 0xC0u) == 0u); // peripheral to memory
		CHECK(s->NDTR == 4u);
		CHECK(s->PAR == b.dmarAddr());
		CHECK(s->M0AR == b.addr());
		CHECK(b.h.DMABurstState == HAL_DMA_BURST_STATE_BUSY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC4 MultiWriteStart and MultiReadStart program NDTR with the data length, not the burst length") {
	for (int rd = 0; rd < 2; ++rd) {
		CAPTURE(rd);
		TcBurst b(1, rd == 0);
		HAL_StatusTypeDef st = rd == 0 ? HAL_TIM_DMABurst_MultiWriteStart(&b.h, TIM_DMABASE_ARR, TIM_DMA_CC1, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS, 8)
		                               : HAL_TIM_DMABurst_MultiReadStart(&b.h, TIM_DMABASE_ARR, TIM_DMA_CC1, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS, 8);
		CHECK(st == HAL_OK);
		CHECK(b.t->DCR == (0x0Bu | 0x100u));
		CHECK(tcStream(1, 1)->NDTR == 8u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC4 the burst length 1 to 18 transfers and every base address reach DCR and NDTR") {
	for (int rd = 0; rd < 2; ++rd)
		for (uint32_t n = 1; n <= 18; ++n) {
			CAPTURE(rd);
			CAPTURE(n);
			TcBurst b(1, rd == 0);
			const uint32_t len = (n - 1u) << 8;
			const uint32_t base = n % 18u; // 1..17, 0: covers CR1 .. BDTR
			HAL_StatusTypeDef st = rd == 0 ? HAL_TIM_DMABurst_WriteStart(&b.h, base, TIM_DMA_UPDATE, b.buf, len) : HAL_TIM_DMABurst_ReadStart(&b.h, base, TIM_DMA_UPDATE, b.buf, len);
			CHECK(st == HAL_OK);
			CHECK(b.t->DCR == (base | len));
			CHECK(tcStream(1, 5)->NDTR == n);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TC4 a start while a burst is BUSY returns HAL_BUSY and changes nothing") {
	for (int rd = 0; rd < 2; ++rd) {
		CAPTURE(rd);
		TcBurst b(1, rd == 0);
		REQUIRE((rd == 0 ? HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_CC1, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS)
		                 : HAL_TIM_DMABurst_ReadStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_CC1, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS)) == HAL_OK);
		const uint32_t dier = b.t->DIER, dcr = b.t->DCR;
		HAL_StatusTypeDef st = rd == 0 ? HAL_TIM_DMABurst_MultiWriteStart(&b.h, TIM_DMABASE_CCR2, TIM_DMA_CC2, b.buf, TIM_DMABURSTLENGTH_3TRANSFERS, 3)
		                               : HAL_TIM_DMABurst_MultiReadStart(&b.h, TIM_DMABASE_CCR2, TIM_DMA_CC2, b.buf, TIM_DMABURSTLENGTH_3TRANSFERS, 3);
		CHECK(st == HAL_BUSY);
		CHECK(b.t->DIER == dier);
		CHECK(b.t->DCR == dcr);
		CHECK((tcStream(1, 2)->CR & tcDmaEN) == 0u);
		CHECK(b.h.DMABurstState == HAL_DMA_BURST_STATE_BUSY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC4 a NULL buffer with more than one transfer is rejected and the state stays READY") {
	for (int rd = 0; rd < 2; ++rd) {
		CAPTURE(rd);
		TcBurst b(1, rd == 0);
		HAL_StatusTypeDef st = rd == 0 ? HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_CC1, nullptr, TIM_DMABURSTLENGTH_2TRANSFERS)
		                               : HAL_TIM_DMABurst_ReadStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_CC1, nullptr, TIM_DMABURSTLENGTH_2TRANSFERS);
		CHECK(st == HAL_ERROR);
		CHECK(b.h.DMABurstState == HAL_DMA_BURST_STATE_READY);
		CHECK(b.t->DCR == 0u);
		CHECK(b.t->DIER == 0u);
		CHECK((tcStream(1, 1)->CR & tcDmaEN) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC4 a NULL buffer with a burst of one transfer is accepted and the stream is started towards address 0") {
	// NEEDS REVIEW 334/e: the NULL check compares the encoded BurstLength (0 for one transfer) with 0
	TcBurst b(1, true);
	CHECK(HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_CC1, nullptr, TIM_DMABURSTLENGTH_1TRANSFER) == HAL_OK);
	CHECK(tcStream(1, 1)->M0AR == 0u);
	CHECK(b.h.DMABurstState == HAL_DMA_BURST_STATE_BUSY);
}

TEST_CASE_FIXTURE(TimFixture, "TC4 WriteStop and ReadStop abort the stream, clear only that request and return the burst state to READY") {
	TC_FOR_BURSTS(tim, i, true) {
		CAPTURE(tim);
		CAPTURE(i);
		for (int rd = 0; rd < 2; ++rd) {
			CAPTURE(rd);
			TcBurst b(tim, rd == 0);
			REQUIRE((rd == 0 ? HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, b.tab[i].src, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS)
			                 : HAL_TIM_DMABurst_ReadStart(&b.h, TIM_DMABASE_CCR1, b.tab[i].src, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS)) == HAL_OK);
			b.set(&b.t->DIER, b.t->DIER | tcDIER_UIE); // an unrelated bit that must survive
			CHECK((rd == 0 ? HAL_TIM_DMABurst_WriteStop(&b.h, b.tab[i].src) : HAL_TIM_DMABurst_ReadStop(&b.h, b.tab[i].src)) == HAL_OK);
			CHECK((b.t->DIER & b.tab[i].src) == 0u);
			CHECK((b.t->DIER & tcDIER_UIE) == tcDIER_UIE);
			CHECK((b.stream(i)->CR & tcDmaEN) == 0u);
			CHECK(b.h.DMABurstState == HAL_DMA_BURST_STATE_READY);
		}
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC4 after a stop a new start is accepted") {
	TcBurst b(1, true);
	REQUIRE(HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS) == HAL_OK);
	REQUIRE(HAL_TIM_DMABurst_WriteStop(&b.h, TIM_DMA_UPDATE) == HAL_OK);
	HAL_DMA_IRQHandler(&b.dma[TIM_DMA_ID_UPDATE]); // HAL_DMA_Abort_IT ends in the interrupt handler (UM1725: abort in interrupt mode)
	CHECK(HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS) == HAL_OK);
	CHECK(b.h.DMABurstState == HAL_DMA_BURST_STATE_BUSY);
}

TEST_CASE_FIXTURE(TimFixture, "TC4 a stop with an unknown request source returns HAL_ERROR and changes nothing") {
	for (int rd = 0; rd < 2; ++rd) {
		CAPTURE(rd);
		TcBurst b(1, true);
		REQUIRE(HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_CC1, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS) == HAL_OK);
		const uint32_t dier = b.t->DIER;
		CHECK((rd == 0 ? HAL_TIM_DMABurst_WriteStop(&b.h, 0x0u) : HAL_TIM_DMABurst_ReadStop(&b.h, 0x0u)) == HAL_ERROR);
		CHECK(b.t->DIER == dier);
		CHECK(b.h.DMABurstState == HAL_DMA_BURST_STATE_BUSY);
		CHECK((tcStream(1, 1)->CR & tcDmaEN) == tcDmaEN);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC4 a stop of a source that was never started returns HAL_OK and leaves the state READY") {
	// NEEDS REVIEW 334/e
	TcBurst b(1, true);
	CHECK(HAL_TIM_DMABurst_WriteStop(&b.h, TIM_DMA_CC1) == HAL_OK);
	CHECK(b.h.DMABurstState == HAL_DMA_BURST_STATE_READY);
}

TEST_CASE_FIXTURE(TimFixture, "TC4 a start with an unknown request source returns HAL_ERROR and writes neither DCR nor DIER") {
	for (int rd = 0; rd < 2; ++rd) {
		CAPTURE(rd);
		TcBurst b(1, rd == 0);
		HAL_StatusTypeDef st = rd == 0 ? HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, 0x0u, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS)
		                               : HAL_TIM_DMABurst_ReadStart(&b.h, TIM_DMABASE_CCR1, 0x0u, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS);
		CHECK(st == HAL_ERROR);
		CHECK(b.t->DCR == 0u);
		CHECK(b.t->DIER == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC4 a start with an unknown source returns HAL_ERROR and leaves the burst state BUSY so the next start returns HAL_BUSY") {
	// NEEDS REVIEW 334/e: the source is outside the API contract; the state is set BUSY before the source switch
	TcBurst b(1, true);
	CHECK(HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, 0x0u, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS) == HAL_ERROR);
	CHECK(b.h.DMABurstState == HAL_DMA_BURST_STATE_BUSY);
	CHECK(HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_CC1, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS) == HAL_BUSY);
}

TEST_CASE_FIXTURE(TimFixture, "TC4 a start rejected because the DMA stream cannot start leaves the burst state READY" * KNOWN_ISSUE("V112 (arguable, UM1725 gives no state rule for a failed start): when HAL_DMA_Start_IT fails (stream BUSY) the function returns HAL_ERROR with DMABurstState left BUSY (stm32f4xx_hal_tim.c, state set BUSY before the DMA start), so every later start returns HAL_BUSY")) {
	TcBurst b(1, true);
	b.dma[TIM_DMA_ID_CC1].State = HAL_DMA_STATE_BUSY;
	CHECK(HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_CC1, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS) == HAL_ERROR);
	CHECK(b.h.DMABurstState == HAL_DMA_BURST_STATE_READY);
}

TEST_CASE_FIXTURE(TimFixture, "TC4 a start whose DMA stream cannot start writes neither DCR nor DIER") {
	TcBurst b(1, true);
	b.dma[TIM_DMA_ID_CC1].State = HAL_DMA_STATE_BUSY;
	CHECK(HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_CC1, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS) == HAL_ERROR);
	CHECK(b.t->DCR == 0u);
	CHECK(b.t->DIER == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC4 a start with the burst state RESET runs and leaves the state RESET") {
	// NEEDS REVIEW 334/e: DMABurstState is neither READY nor BUSY: the start proceeds
	TcBurst b(1, true);
	b.h.DMABurstState = HAL_DMA_BURST_STATE_RESET;
	CHECK(HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS) == HAL_OK);
	CHECK(b.h.DMABurstState == HAL_DMA_BURST_STATE_RESET);
	CHECK((b.t->DIER & TIM_DMA_UPDATE) == TIM_DMA_UPDATE);
}

TEST_CASE_FIXTURE(TimFixture, "TC4 a write burst played with plain DMAR accesses writes consecutive registers from DBA and wraps after DBL + 1 accesses") {
	TcBurst b(1, true);
	REQUIRE(HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, b.buf, TIM_DMABURSTLENGTH_3TRANSFERS) == HAL_OK);
	b.t->DMAR = 0x11u;
	b.t->DMAR = 0x22u;
	b.t->DMAR = 0x33u;
	CHECK(b.t->CCR1 == 0x11u);
	CHECK(b.t->CCR2 == 0x22u);
	CHECK(b.t->CCR3 == 0x33u);
	b.t->DMAR = 0x44u; // the index counts 0 .. DBL, then starts again
	CHECK(b.t->CCR1 == 0x44u);
	CHECK(b.t->CCR2 == 0x22u);
}

TEST_CASE_FIXTURE(TimFixture, "TC4 a read burst played with plain DMAR accesses reads consecutive registers from DBA") {
	TcBurst b(1, false);
	b.set(&b.t->CCR1, 0xA1u);
	b.set(&b.t->CCR2, 0xA2u);
	b.set(&b.t->CCR3, 0xA3u);
	b.set(&b.t->CCR4, 0xA4u);
	REQUIRE(HAL_TIM_DMABurst_ReadStart(&b.h, TIM_DMABASE_CCR2, TIM_DMA_UPDATE, b.buf, TIM_DMABURSTLENGTH_3TRANSFERS) == HAL_OK);
	CHECK(b.t->DMAR == 0xA2u);
	CHECK(b.t->DMAR == 0xA3u);
	CHECK(b.t->DMAR == 0xA4u);
	CHECK(b.t->DMAR == 0xA2u);
}

TEST_CASE_FIXTURE(TimFixture, "TC4 a burst on the 32 bit TIM2 transfers 32 bit values to CCR1 and CCR2") {
	TcBurst b(2, true);
	REQUIRE(HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS) == HAL_OK);
	b.t->DMAR = 0x12345678u;
	b.t->DMAR = 0x9ABCDEF0u;
	CHECK(b.t->CCR1 == 0x12345678u);
	CHECK(b.t->CCR2 == 0x9ABCDEF0u);
}

TEST_CASE_FIXTURE(TimFixture, "TC4 the burst completion callback of the update source reports the period elapsed and sets the timer state READY") {
	for (int rd = 0; rd < 2; ++rd) {
		CAPTURE(rd);
		TcBurst b(1, rd == 0);
		b.h.State = HAL_TIM_STATE_BUSY;
		REQUIRE((rd == 0 ? HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS)
		                 : HAL_TIM_DMABurst_ReadStart(&b.h, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS)) == HAL_OK);
		b.complete(0);
		CHECK(timCounts.period == 1u);
		CHECK(b.h.State == HAL_TIM_STATE_READY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC4 burst completion calls the callback of the source: pulse finished (write CCx), capture (read CCx), commutation, trigger") {
	// NEEDS REVIEW 334/f
	for (unsigned i = 1; i < 7; ++i)
		for (int rd = 0; rd < 2; ++rd) {
			CAPTURE(i);
			CAPTURE(rd);
			TcBurst b(1, rd == 0);
			REQUIRE((rd == 0 ? HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, b.tab[i].src, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS)
			                 : HAL_TIM_DMABurst_ReadStart(&b.h, TIM_DMABASE_CCR1, b.tab[i].src, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS)) == HAL_OK);
			b.complete(i);
			REQUIRE(timCounts.orderCount == 1u);
			const unsigned kind = timCounts.order[0];
			if (b.tab[i].cc)
				CHECK(kind == (rd == 0 ? TIM_CB_PWM_PULSE : TIM_CB_IC_CAPTURE));
			else if (b.tab[i].src == TIM_DMA_COM)
				CHECK(kind == TIM_CB_COMMUT);
			else
				CHECK(kind == TIM_CB_TRIGGER);
		}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC4 the half transfer of a burst calls the half callback of the source") {
	// NEEDS REVIEW 334/f
	for (unsigned i = 0; i < 7; ++i) {
		CAPTURE(i);
		TcBurst b(1, true);
		REQUIRE(HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, b.tab[i].src, b.buf, TIM_DMABURSTLENGTH_4TRANSFERS) == HAL_OK);
		hostDmaStepItems(1, b.tab[i].stream, 2);
		HAL_DMA_IRQHandler(&b.dma[b.tab[i].id]);
		REQUIRE(timCounts.orderCount == 1u);
		const unsigned kind = timCounts.order[0];
		if (b.tab[i].cc)
			CHECK(kind == TIM_CB_PWM_PULSE_HALF);
		else if (b.tab[i].src == TIM_DMA_UPDATE)
			CHECK(kind == TIM_CB_PERIOD_HALF);
		else if (b.tab[i].src == TIM_DMA_COM)
			CHECK(kind == TIM_CB_COMMUT_HALF);
		else
			CHECK(kind == TIM_CB_TRIGGER_HALF);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC4 a DMA transfer error of a burst stream calls the error callback") {
	for (unsigned i = 0; i < 7; ++i) {
		CAPTURE(i);
		TcBurst b(1, true);
		REQUIRE(HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, b.tab[i].src, b.buf, TIM_DMABURSTLENGTH_2TRANSFERS) == HAL_OK);
		hostDmaRaise(1, b.tab[i].stream, HOST_DMA_TEIF);
		HAL_DMA_IRQHandler(&b.dma[b.tab[i].id]);
		CHECK(timCounts.error == 1u);
	}
}

/* ================================================================ TC5: TimEx */

namespace {
HAL_StatusTypeDef tcCommut(TcRig &r, int mode, uint32_t trig, uint32_t src) {
	return mode == 0 ? HAL_TIMEx_ConfigCommutEvent(&r.h, trig, src) : mode == 1 ? HAL_TIMEx_ConfigCommutEvent_IT(&r.h, trig, src) : HAL_TIMEx_ConfigCommutEvent_DMA(&r.h, trig, src);
}
const unsigned tcAdv[2] = {1, 8};
} // namespace

TEST_CASE_FIXTURE(TimFixture, "TC5 ConfigCommutEvent in every variant sets CCPC and CCUS according to the commutation source") {
	for (unsigned tim : tcAdv)
		for (int mode = 0; mode < 3; ++mode)
			for (int sw = 0; sw < 2; ++sw) {
				CAPTURE(tim);
				CAPTURE(mode);
				CAPTURE(sw);
				TcBurst b(tim, true);
				b.set(&b.t->CR2, sw ? 0x0100u | 0x4u : 0x0100u); // OIS1 kept; CCUS preset to the other value
				CHECK(tcCommut(b, mode, TIM_TS_ITR0, sw ? TIM_COMMUTATION_SOFTWARE : TIM_COMMUTATION_TRGI) == HAL_OK);
				CHECK(b.t->CR2 == (0x0100u | 0x1u | (sw ? 0u : 0x4u)));
				TC_IDLE(b);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TC5 ConfigCommutEvent with ITR0 to ITR3 selects that trigger in SMCR and keeps the other bits") {
	const uint32_t trg[4] = {TIM_TS_ITR0, TIM_TS_ITR1, TIM_TS_ITR2, TIM_TS_ITR3};
	for (unsigned tim : tcAdv)
		for (int mode = 0; mode < 3; ++mode)
			for (uint32_t i = 0; i < 4; ++i) {
				CAPTURE(tim);
				CAPTURE(mode);
				CAPTURE(i);
				TcBurst b(tim, true);
				b.set(&b.t->SMCR, 0x0087u | 0x70u); // SMS = 111, MSM, TS = 111 before
				CHECK(tcCommut(b, mode, trg[i], TIM_COMMUTATION_TRGI) == HAL_OK);
				CHECK(b.t->SMCR == (0x87u | (i << 4)));
			}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC5 ConfigCommutEvent with a trigger other than ITR0 to ITR3 leaves SMCR unchanged") {
	// NEEDS REVIEW 334/g
	const uint32_t trg[3] = {TIM_TS_NONE, TIM_TS_TI1F_ED, TIM_TS_ETRF};
	for (unsigned tim : tcAdv)
		for (uint32_t t : trg) {
			CAPTURE(tim);
			CAPTURE(t);
			TcBurst b(tim, true);
			b.set(&b.t->SMCR, 0x37u);
			CHECK(HAL_TIMEx_ConfigCommutEvent(&b.h, t, TIM_COMMUTATION_SOFTWARE) == HAL_OK);
			CHECK(b.t->SMCR == 0x37u);
			CHECK((b.t->CR2 & 0x1u) == 1u);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TC5 ConfigCommutEvent variants set COMIE and COMDE as polling: none, IT: COMIE, DMA: COMDE") {
	for (unsigned tim : tcAdv)
		for (int mode = 0; mode < 3; ++mode) {
			CAPTURE(tim);
			CAPTURE(mode);
			TcBurst b(tim, true);
			b.set(&b.t->DIER, tcDIER_UIE | tcDIER_COMIE | tcDIER_COMDE);
			CHECK(tcCommut(b, mode, TIM_TS_ITR1, TIM_COMMUTATION_SOFTWARE) == HAL_OK);
			const uint32_t want = tcDIER_UIE | (mode == 1 ? tcDIER_COMIE : 0u) | (mode == 2 ? tcDIER_COMDE : 0u);
			CHECK(b.t->DIER == want);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TC5 the IT variant leads a commutation event to the commutation callback") {
	TcBurst b(1, true);
	REQUIRE(HAL_TIMEx_ConfigCommutEvent_IT(&b.h, TIM_TS_ITR0, TIM_COMMUTATION_SOFTWARE) == HAL_OK);
	CHECK(HAL_TIM_GenerateEvent(&b.h, TIM_EVENTSOURCE_COM) == HAL_OK);
	CHECK(hostTimIrqPending(1) == (1u << 5));
	HAL_TIM_IRQHandler(&b.h);
	CHECK(timCounts.commut == 1u);
	CHECK((b.t->SR & (1u << 5)) == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "TC5 the DMA variant wires the commutation DMA completion, half completion and error to the callbacks") {
	TcBurst b(1, true);
	REQUIRE(HAL_TIMEx_ConfigCommutEvent_DMA(&b.h, TIM_TS_ITR0, TIM_COMMUTATION_SOFTWARE) == HAL_OK);
	DMA_HandleTypeDef &d = b.dma[TIM_DMA_ID_COMMUTATION];
	REQUIRE(HAL_DMA_Start_IT(&d, b.addr(), b.dmarAddr(), 4) == HAL_OK);
	b.h.State = HAL_TIM_STATE_BUSY;
	hostDmaStepItems(1, 4, 2);
	HAL_DMA_IRQHandler(&d);
	CHECK(timCounts.commutHalf == 1u);
	b.complete(5);
	CHECK(timCounts.commut == 1u);
	CHECK(b.h.State == HAL_TIM_STATE_READY);
	REQUIRE(HAL_DMA_Start_IT(&d, b.addr(), b.dmarAddr(), 4) == HAL_OK);
	hostDmaRaise(1, 4, HOST_DMA_TEIF);
	HAL_DMA_IRQHandler(&d);
	CHECK(timCounts.error == 1u);
}

TEST_CASE_FIXTURE(TimFixture, "TC5 ConfigCommutEvent on a locked handle returns HAL_BUSY in every variant and changes nothing") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TcBurst b(1, true);
		b.h.Lock = HAL_LOCKED;
		CHECK(tcCommut(b, mode, TIM_TS_ITR1, TIM_COMMUTATION_TRGI) == HAL_BUSY);
		CHECK(b.t->CR2 == 0u);
		CHECK(b.t->SMCR == 0u);
		CHECK(b.t->DIER == 0u);
		CHECK(b.h.Lock == HAL_LOCKED);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC5 MasterConfigSynchronization writes MMS for every trigger output and MSM for every timer with SMCR") {
	const uint32_t trgo[8] = {TIM_TRGO_RESET, TIM_TRGO_ENABLE, TIM_TRGO_UPDATE, TIM_TRGO_OC1, TIM_TRGO_OC1REF, TIM_TRGO_OC2REF, TIM_TRGO_OC3REF, TIM_TRGO_OC4REF};
	for (unsigned tim : {1u, 8u, 2u, 3u, 4u, 5u})
		for (uint32_t m = 0; m < 8; ++m)
			for (int msm = 0; msm < 2; ++msm) {
				CAPTURE(tim);
				CAPTURE(m);
				CAPTURE(msm);
				TcRig r(tim);
				const bool hasSmcr = true;
				const uint32_t keepCr2 = (tim == 1 || tim == 8) ? 0x0105u : (tim <= 5 ? 0x88u : 0u);
				r.set(&r.t->CR2, keepCr2 | 0x70u); // previous MMS = 111 is replaced
				if (hasSmcr) r.set(&r.t->SMCR, msm ? 0x07u : 0x87u);
				TIM_MasterConfigTypeDef c{};
				c.MasterOutputTrigger = trgo[m];
				c.MasterSlaveMode = msm ? TIM_MASTERSLAVEMODE_ENABLE : TIM_MASTERSLAVEMODE_DISABLE;
				CHECK(HAL_TIMEx_MasterConfigSynchronization(&r.h, &c) == HAL_OK);
				CHECK(r.t->CR2 == (keepCr2 | (m << 4))); // MMS bits 6:4
				if (hasSmcr) CHECK(r.t->SMCR == (msm ? 0x87u : 0x07u));
				TC_IDLE(r);
			}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC5 MasterConfigSynchronization of TIM6 and TIM7 writes the MMS bits and reads the reserved SMCR offset") {
	// NEEDS REVIEW 334/i
	const uint32_t trgo[8] = {TIM_TRGO_RESET, TIM_TRGO_ENABLE, TIM_TRGO_UPDATE, TIM_TRGO_OC1, TIM_TRGO_OC1REF, TIM_TRGO_OC2REF, TIM_TRGO_OC3REF, TIM_TRGO_OC4REF};
	for (unsigned tim : {6u, 7u})
		for (uint32_t m = 0; m < 8; ++m) {
			CAPTURE(tim);
			CAPTURE(m);
			TcRig r(tim);
			r.set(&r.t->CR2, 0x70u);
			TIM_MasterConfigTypeDef c{};
			c.MasterOutputTrigger = trgo[m];
			CHECK(HAL_TIMEx_MasterConfigSynchronization(&r.h, &c) == HAL_OK);
			CHECK(HalFixture::faults() > 0u);
			HalFixture::clearFaults();
			CHECK(r.t->CR2 == (m << 4));
		}
}

TEST_CASE_FIXTURE(TimFixture, "TC5 MasterConfigSynchronization on a locked handle returns HAL_BUSY and changes nothing") {
	TcRig r(2);
	r.h.Lock = HAL_LOCKED;
	TIM_MasterConfigTypeDef c{};
	c.MasterOutputTrigger = TIM_TRGO_UPDATE;
	c.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	CHECK(HAL_TIMEx_MasterConfigSynchronization(&r.h, &c) == HAL_BUSY);
	CHECK(r.t->CR2 == 0u);
	CHECK(r.t->SMCR == 0u);
	CHECK(r.h.Lock == HAL_LOCKED);
}

namespace {
uint32_t tcBdtr(unsigned tim, const TIM_BreakDeadTimeConfigTypeDef &c) {
	TcRig r(tim);
	CHECK(HAL_TIMEx_ConfigBreakDeadTime(&r.h, &c) == HAL_OK);
	TC_IDLE(r);
	return r.t->BDTR;
}
} // namespace

TEST_CASE_FIXTURE(TimFixture, "TC5 ConfigBreakDeadTime writes the dead time into DTG bits 7:0") {
	for (unsigned tim : tcAdv)
		for (uint32_t dt : {0u, 1u, 0x7Fu, 0x80u, 0xFFu}) {
			CAPTURE(tim);
			CAPTURE(dt);
			TIM_BreakDeadTimeConfigTypeDef c{};
			c.DeadTime = dt;
			CHECK(tcBdtr(tim, c) == dt);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TC5 ConfigBreakDeadTime maps OSSR, OSSI, break enable, break polarity and AOE to their own BDTR bits") {
	for (unsigned tim : tcAdv) {
		CAPTURE(tim);
		TIM_BreakDeadTimeConfigTypeDef c{};
		c.OffStateRunMode = TIM_OSSR_ENABLE;
		CHECK(tcBdtr(tim, c) == (1u << 11));
		c = {};
		c.OffStateIDLEMode = TIM_OSSI_ENABLE;
		CHECK(tcBdtr(tim, c) == (1u << 10));
		c = {};
		c.BreakState = TIM_BREAK_ENABLE;
		CHECK(tcBdtr(tim, c) == (1u << 12));
		c = {};
		c.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
		CHECK(tcBdtr(tim, c) == (1u << 13));
		c = {};
		c.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
		CHECK(tcBdtr(tim, c) == (1u << 14));
		c = {};
		c.OffStateRunMode = TIM_OSSR_DISABLE;
		c.OffStateIDLEMode = TIM_OSSI_DISABLE;
		c.BreakState = TIM_BREAK_DISABLE;
		c.BreakPolarity = TIM_BREAKPOLARITY_LOW;
		c.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
		CHECK(tcBdtr(tim, c) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC5 ConfigBreakDeadTime replaces the whole BDTR so a field set before and not selected is cleared") {
	TcRig r(1);
	r.set(&r.t->BDTR, 0x7CFFu); // DTG, OSSI, OSSR, BKE, BKP, AOE
	TIM_BreakDeadTimeConfigTypeDef c{};
	c.DeadTime = 0x12;
	CHECK(HAL_TIMEx_ConfigBreakDeadTime(&r.h, &c) == HAL_OK);
	CHECK(r.t->BDTR == 0x12u);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC5 ConfigBreakDeadTime on a running output clears MOE") {
	// NEEDS REVIEW 334/g: BDTR is written from 0, MOE (bit 15) included
	TcRig r(1);
	r.set(&r.t->BDTR, tcBDTR_MOE);
	TIM_BreakDeadTimeConfigTypeDef c{};
	c.BreakState = TIM_BREAK_ENABLE;
	CHECK(HAL_TIMEx_ConfigBreakDeadTime(&r.h, &c) == HAL_OK);
	CHECK(r.t->BDTR == (1u << 12));
}

TEST_CASE_FIXTURE(TimFixture, "TC5 the LOCK level of the configuration lands in BDTR bits 9:8 for every level") {
	for (unsigned tim : tcAdv) {
		const uint32_t levels[4] = {TIM_LOCKLEVEL_OFF, TIM_LOCKLEVEL_1, TIM_LOCKLEVEL_2, TIM_LOCKLEVEL_3};
		for (uint32_t l = 0; l < 4; ++l) {
			CAPTURE(tim);
			CAPTURE(l);
			TIM_BreakDeadTimeConfigTypeDef c{};
			c.LockLevel = levels[l];
			CHECK((tcBdtr(tim, c) & 0x300u) == (l << 8));
		}
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC5 a configuration with LOCK level 1 keeps the DTG, BKE, BKP and AOE values written in the same call") {
	// RM0090 17.4.18: once TIMx_BDTR has been written its content is frozen: the written value is what is frozen
	TIM_BreakDeadTimeConfigTypeDef c{};
	c.LockLevel = TIM_LOCKLEVEL_1;
	c.DeadTime = 0x55;
	c.BreakState = TIM_BREAK_ENABLE;
	c.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	c.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
	CHECK(tcBdtr(1, c) == (0x100u | 0x55u | (1u << 12) | (1u << 13) | (1u << 14)));
}

TEST_CASE_FIXTURE(TimFixture, "TC5 a configuration with LOCK level 2 keeps OSSR and OSSI written in the same call") {
	TIM_BreakDeadTimeConfigTypeDef c{};
	c.LockLevel = TIM_LOCKLEVEL_2;
	c.OffStateRunMode = TIM_OSSR_ENABLE;
	c.OffStateIDLEMode = TIM_OSSI_ENABLE;
	CHECK(tcBdtr(8, c) == (0x200u | (1u << 10) | (1u << 11)));
}

TEST_CASE_FIXTURE(TimFixture, "TC5 after LOCK level 1 a second configuration changes OSSR and OSSI but not LOCK, DTG, BKE, BKP, AOE") {
	TcRig r(1);
	TIM_BreakDeadTimeConfigTypeDef c{};
	c.LockLevel = TIM_LOCKLEVEL_1;
	c.DeadTime = 0x21;
	c.BreakState = TIM_BREAK_ENABLE;
	REQUIRE(HAL_TIMEx_ConfigBreakDeadTime(&r.h, &c) == HAL_OK);
	const uint32_t first = r.t->BDTR;
	TIM_BreakDeadTimeConfigTypeDef d{};
	d.LockLevel = TIM_LOCKLEVEL_OFF;
	d.DeadTime = 0x7E;
	d.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	d.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
	d.OffStateRunMode = TIM_OSSR_ENABLE;
	d.OffStateIDLEMode = TIM_OSSI_ENABLE;
	CHECK(HAL_TIMEx_ConfigBreakDeadTime(&r.h, &d) == HAL_OK);
	CHECK((r.t->BDTR & 0xC00u) == 0xC00u); // OSSI / OSSR follow the second call
	CHECK((r.t->BDTR & (0x300u | 0xFFu | (1u << 12) | (1u << 13) | (1u << 14))) == (first & (0x300u | 0xFFu | (1u << 12) | (1u << 13) | (1u << 14))));
	CHECK(hostTimProtectedWrites(1) > 0u);
}

TEST_CASE_FIXTURE(TimFixture, "TC5 after LOCK level 2 a second configuration also keeps OSSR and OSSI") {
	TcRig r(8);
	TIM_BreakDeadTimeConfigTypeDef c{};
	c.LockLevel = TIM_LOCKLEVEL_2;
	c.OffStateRunMode = TIM_OSSR_ENABLE;
	REQUIRE(HAL_TIMEx_ConfigBreakDeadTime(&r.h, &c) == HAL_OK);
	const uint32_t first = r.t->BDTR;
	TIM_BreakDeadTimeConfigTypeDef d{};
	d.OffStateIDLEMode = TIM_OSSI_ENABLE;
	CHECK(HAL_TIMEx_ConfigBreakDeadTime(&r.h, &d) == HAL_OK);
	CHECK((r.t->BDTR & 0x3FFFu) == (first & 0x3FFFu));
}

TEST_CASE_FIXTURE(TimFixture, "TC5 ConfigBreakDeadTime on a locked handle returns HAL_BUSY and changes nothing") {
	TcRig r(1);
	r.h.Lock = HAL_LOCKED;
	TIM_BreakDeadTimeConfigTypeDef c{};
	c.DeadTime = 5;
	CHECK(HAL_TIMEx_ConfigBreakDeadTime(&r.h, &c) == HAL_BUSY);
	CHECK(r.t->BDTR == 0u);
	CHECK(r.h.Lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(TimFixture, "TC5 RemapConfig writes the OR register of TIM2, TIM5 and TIM11 for every input") {
	struct Row { unsigned tim; uint32_t remap, expect; };
	const Row rows[] = {
	    {2, TIM_TIM2_TIM8_TRGO, 0x0000u}, {2, TIM_TIM2_ETH_PTP, 0x0400u}, {2, TIM_TIM2_USBFS_SOF, 0x0800u}, {2, TIM_TIM2_USBHS_SOF, 0x0C00u},
	    {5, TIM_TIM5_GPIO, 0x00u},        {5, TIM_TIM5_LSI, 0x40u},       {5, TIM_TIM5_LSE, 0x80u},          {5, TIM_TIM5_RTC, 0xC0u},
	    {11, TIM_TIM11_GPIO, 0x0u},       {11, TIM_TIM11_HSE, 0x2u},
	};
	for (const Row &w : rows) {
		CAPTURE(w.tim);
		CAPTURE(w.remap);
		TcRig r(w.tim);
		r.set(&r.t->OR, w.tim == 2 ? 0x0C00u : w.tim == 5 ? 0xC0u : 0x3u); // all remap bits set: the write replaces them
		CHECK(HAL_TIMEx_RemapConfig(&r.h, w.remap) == HAL_OK);
		CHECK(r.t->OR == w.expect);
		TC_IDLE(r);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC5 RemapConfig on a locked handle returns HAL_BUSY and leaves OR") {
	TcRig r(2);
	r.set(&r.t->OR, 0x0400u);
	r.h.Lock = HAL_LOCKED;
	CHECK(HAL_TIMEx_RemapConfig(&r.h, TIM_TIM2_USBFS_SOF) == HAL_BUSY);
	CHECK(r.t->OR == 0x0400u);
	CHECK(r.h.Lock == HAL_LOCKED);
}

/* ================================================================ fix pass: every burst source, both directions, plain and Multi */

namespace {
/* variant: 0 WriteStart, 1 MultiWriteStart, 2 ReadStart, 3 MultiReadStart. 4 data items (WriteStart / ReadStart: burst of 4 transfers). */
HAL_StatusTypeDef tcBurstStart(TcBurst &b, int variant, uint32_t src, uint32_t *buf, uint32_t burstLen = 0xFFFFFFFFu) {
	const bool plain = (variant & 1) == 0;
	if (burstLen == 0xFFFFFFFFu) burstLen = plain ? TIM_DMABURSTLENGTH_4TRANSFERS : TIM_DMABURSTLENGTH_2TRANSFERS;
	switch (variant) {
	case 0: return HAL_TIM_DMABurst_WriteStart(&b.h, TIM_DMABASE_CCR1, src, buf, burstLen);
	case 1: return HAL_TIM_DMABurst_MultiWriteStart(&b.h, TIM_DMABASE_CCR1, src, buf, burstLen, 4);
	case 2: return HAL_TIM_DMABurst_ReadStart(&b.h, TIM_DMABASE_CCR1, src, buf, burstLen);
	default: return HAL_TIM_DMABurst_MultiReadStart(&b.h, TIM_DMABASE_CCR1, src, buf, burstLen, 4);
	}
}
unsigned tcExpectedKind(const TcSrc &s, bool write, bool half) {
	if (s.cc) return write ? (half ? TIM_CB_PWM_PULSE_HALF : TIM_CB_PWM_PULSE) : (half ? TIM_CB_IC_CAPTURE_HALF : TIM_CB_IC_CAPTURE);
	if (s.src == TIM_DMA_UPDATE) return half ? TIM_CB_PERIOD_HALF : TIM_CB_PERIOD;
	if (s.src == TIM_DMA_COM) return half ? TIM_CB_COMMUT_HALF : TIM_CB_COMMUT;
	return half ? TIM_CB_TRIGGER_HALF : TIM_CB_TRIGGER;
}
} // namespace

#define TC_ALL_VARIANTS(tim, i, variant) \
	for (unsigned tim : tcTims) \
		for (unsigned i = 0; i < (tim == 2 ? 5u : 7u); ++i) \
			for (int variant = 0; variant < 4; ++variant)

TEST_CASE_FIXTURE(TimFixture, "TC4 the burst completion of every source and variant reaches the callback of that source and direction") {
	TC_ALL_VARIANTS(tim, i, v) {
		CAPTURE(tim);
		CAPTURE(i);
		CAPTURE(v);
		TcBurst b(tim, v < 2);
		REQUIRE(tcBurstStart(b, v, b.tab[i].src, b.buf) == HAL_OK);
		b.complete(i);
		REQUIRE(timCounts.orderCount == 1u);
		CHECK(timCounts.order[0] == tcExpectedKind(b.tab[i], v < 2, false));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC4 the burst half completion of every source and variant reaches the half callback of that source and direction") {
	TC_ALL_VARIANTS(tim, i, v) {
		CAPTURE(tim);
		CAPTURE(i);
		CAPTURE(v);
		TcBurst b(tim, v < 2);
		REQUIRE(tcBurstStart(b, v, b.tab[i].src, b.buf) == HAL_OK);
		hostDmaStepItems(b.controller, b.tab[i].stream, 2);
		HAL_DMA_IRQHandler(&b.dma[b.tab[i].id]);
		REQUIRE(timCounts.orderCount == 1u);
		CHECK(timCounts.order[0] == tcExpectedKind(b.tab[i], v < 2, true));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC4 the burst DMA error of every source and variant reaches the error callback") {
	TC_ALL_VARIANTS(tim, i, v) {
		CAPTURE(tim);
		CAPTURE(i);
		CAPTURE(v);
		TcBurst b(tim, v < 2);
		REQUIRE(tcBurstStart(b, v, b.tab[i].src, b.buf) == HAL_OK);
		hostDmaRaise(b.controller, b.tab[i].stream, HOST_DMA_TEIF);
		HAL_DMA_IRQHandler(&b.dma[b.tab[i].id]);
		REQUIRE(timCounts.orderCount == 1u);
		CHECK(timCounts.order[0] == TIM_CB_ERROR);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC4 a burst start whose DMA stream cannot start returns HAL_ERROR (not HAL_BUSY) for every source and variant and writes nothing") {
	TC_ALL_VARIANTS(tim, i, v) {
		CAPTURE(tim);
		CAPTURE(i);
		CAPTURE(v);
		TcBurst b(tim, v < 2);
		b.dma[b.tab[i].id].State = HAL_DMA_STATE_BUSY;
		CHECK(tcBurstStart(b, v, b.tab[i].src, b.buf) == HAL_ERROR);
		CHECK(b.t->DCR == 0u);
		CHECK(b.t->DIER == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC4 every start variant programs the stream towards DMAR in the direction of its name for every source") {
	TC_ALL_VARIANTS(tim, i, v) {
		CAPTURE(tim);
		CAPTURE(i);
		CAPTURE(v);
		TcBurst b(tim, v < 2);
		REQUIRE(tcBurstStart(b, v, b.tab[i].src, b.buf) == HAL_OK);
		DMA_Stream_TypeDef *s = b.stream(i);
		CHECK(s->NDTR == 4u);
		CHECK(s->PAR == b.dmarAddr());
		CHECK(s->M0AR == b.addr());
		CHECK((b.t->DIER & 0x7F00u) == b.tab[i].src);
		CHECK(b.t->DCR == ((v & 1) == 0 ? (0x0Du | 0x300u) : (0x0Du | 0x100u)));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC4 a NULL buffer with more than one transfer is rejected by all four start functions") {
	for (int v = 0; v < 4; ++v) {
		CAPTURE(v);
		TcBurst b(1, v < 2);
		CHECK(tcBurstStart(b, v, TIM_DMA_CC1, nullptr, TIM_DMABURSTLENGTH_2TRANSFERS) == HAL_ERROR);
		CHECK(b.h.DMABurstState == HAL_DMA_BURST_STATE_READY);
		CHECK(b.t->DIER == 0u);
		CHECK((tcStream(1, 1)->CR & tcDmaEN) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TC4 a NULL buffer with TIM_DMABURSTLENGTH_1TRANSFER is accepted by all four start functions") {
	// NEEDS REVIEW 334/e: the NULL check tests the encoded burst length (0 for one transfer); UM1725 does not say that NULL is invalid
	for (int v = 0; v < 4; ++v) {
		CAPTURE(v);
		TcBurst b(1, v < 2);
		CHECK(tcBurstStart(b, v, TIM_DMA_CC1, nullptr, TIM_DMABURSTLENGTH_1TRANSFER) == HAL_OK);
		CHECK(tcStream(1, 1)->M0AR == 0u);
		CHECK(b.h.DMABurstState == HAL_DMA_BURST_STATE_BUSY);
		CHECK((b.t->DIER & TIM_DMA_CC1) == TIM_DMA_CC1);
	}
}

/* Input stage helpers: the old filter of the channel is replaced, the filter and polarity of the other channel are kept. */

TEST_CASE_FIXTURE(TimFixture, "TC1 the TI1 based clock sources replace the IC1 filter and polarity and keep the channel 2 bits of CCMR1 and CCER") {
	const uint32_t srcs[2] = {TIM_CLOCKSOURCE_TI1, TIM_CLOCKSOURCE_TI1ED};
	for (unsigned tim : tcTrig)
		for (uint32_t src : srcs) {
			CAPTURE(tim);
			CAPTURE(src);
			TcRig r(tim);
			r.set(&r.t->CCMR1, (0xFu << 4) | (0xAu << 12)); // IC1F = 15 (old), IC2F = 10 (other channel)
			r.set(&r.t->CCER, 0x1u | 0xAu | 0xB0u);         // CC1E, CC1P + CC1NP, CC2E + CC2P + CC2NP
			TIM_ClockConfigTypeDef c{};
			c.ClockSource = src;
			c.ClockPolarity = TIM_CLOCKPOLARITY_FALLING; // CC1P only
			c.ClockFilter = 5;
			CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_OK);
			CHECK(r.t->CCMR1 == ((5u << 4) | (0xAu << 12)));
			CHECK(r.t->CCER == (0x1u | 0x2u | 0xB0u));
		}
}

TEST_CASE_FIXTURE(TimFixture, "TC1 the TI2 clock source replaces the IC2 filter and polarity and keeps the channel 1 bits of CCMR1 and CCER") {
	for (unsigned tim : tcTrig) {
		CAPTURE(tim);
		TcRig r(tim);
		r.set(&r.t->CCMR1, (0xAu << 4) | (0xFu << 12)); // IC1F = 10 (other channel), IC2F = 15 (old)
		r.set(&r.t->CCER, 0x10u | 0xA0u | 0xBu);         // CC2E, CC2P + CC2NP, CC1E + CC1P + CC1NP
		TIM_ClockConfigTypeDef c{};
		c.ClockSource = TIM_CLOCKSOURCE_TI2;
		c.ClockPolarity = TIM_CLOCKPOLARITY_FALLING; // CC2P only
		c.ClockFilter = 3;
		CHECK(HAL_TIM_ConfigClockSource(&r.h, &c) == HAL_OK);
		CHECK(r.t->CCMR1 == ((0xAu << 4) | (3u << 12)));
		CHECK(r.t->CCER == (0x10u | 0x20u | 0xBu));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TC2 the TI1FP1 and TI1F_ED triggers replace the IC1 filter and keep the channel 2 bits") {
	const uint32_t trg[2] = {TIM_TS_TI1FP1, TIM_TS_TI1F_ED};
	for (unsigned tim : tcTrig)
		for (uint32_t t : trg) {
			CAPTURE(tim);
			CAPTURE(t);
			TcRig r(tim);
			r.set(&r.t->CCMR1, (0xFu << 4) | (0xAu << 12));
			r.set(&r.t->CCER, 0x1u | 0xAu | 0xB0u);
			TIM_SlaveConfigTypeDef c{};
			c.SlaveMode = TIM_SLAVEMODE_RESET;
			c.InputTrigger = t;
			c.TriggerPolarity = TIM_TRIGGERPOLARITY_FALLING;
			c.TriggerFilter = 5;
			CHECK(HAL_TIM_SlaveConfigSynchro(&r.h, &c) == HAL_OK);
			CHECK(r.t->CCMR1 == ((5u << 4) | (0xAu << 12)));
			// TI1FP1 replaces the polarity (CC1P only); TI1F_ED leaves the polarity as it was
			CHECK(r.t->CCER == (t == TIM_TS_TI1FP1 ? (0x1u | 0x2u | 0xB0u) : (0x1u | 0xAu | 0xB0u)));
		}
}

TEST_CASE_FIXTURE(TimFixture, "TC2 the TI2FP2 trigger replaces the IC2 filter and polarity and keeps the channel 1 bits") {
	for (unsigned tim : tcTrig) {
		CAPTURE(tim);
		TcRig r(tim);
		r.set(&r.t->CCMR1, (0xAu << 4) | (0xFu << 12));
		r.set(&r.t->CCER, 0x10u | 0xA0u | 0xBu);
		TIM_SlaveConfigTypeDef c{};
		c.SlaveMode = TIM_SLAVEMODE_RESET;
		c.InputTrigger = TIM_TS_TI2FP2;
		c.TriggerPolarity = TIM_TRIGGERPOLARITY_FALLING;
		c.TriggerFilter = 3;
		CHECK(HAL_TIM_SlaveConfigSynchro(&r.h, &c) == HAL_OK);
		CHECK(r.t->CCMR1 == ((0xAu << 4) | (3u << 12)));
		CHECK(r.t->CCER == (0x10u | 0x20u | 0xBu));
	}
}
