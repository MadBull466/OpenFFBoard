/*
 * P12 T6b group AB: ADC init, channel configuration, blocking conversion and the analog watchdog of stm32f4xx_hal_adc.c
 * (HAL_ADC_Init / DeInit / ConfigChannel / AnalogWDGConfig / Start / Stop / PollForConversion / PollForEvent / GetValue / GetState /
 * GetError and ADC_Init through Init) against the ADC register model (fakes/stm32_hal/HalAdc.h). Requirements AB1-AB5 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T6B_REQUIREMENTS.md; compiled into ffb_hal_tests and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 chapter 7 (HAL ADC Generic Driver: function descriptions, state and error code names) and RM0090 Rev 22
 * chapter 13 (13.3 functional description, Table 66 analog watchdog channel selection, 13.8 data alignment, 13.13 registers: CR1, CR2,
 * SMPR1 / SMPR2, SQR1-3, HTR / LTR, CCR field encodings). Expected register values are written as hand derived constants.
 *
 * NEEDS REVIEW proposals:
 *  NEEDS REVIEW 338/AB-1  characterization: HAL_ADC_Init on a handle whose State holds REG_BUSY / INJ_BUSY clears both and sets READY; other
 *               state bits survive. UM1725 only says "initializes".
 *  NEEDS REVIEW 338/AB-2  characterization: HAL_ADC_Start with the state INJ_BUSY keeps ErrorCode bits other than OVR / DMA (otherwise all are cleared).
 *  NEEDS REVIEW 338/AB-3  characterization: enabling the VBAT channel keeps TSVREFE and enabling the temperature sensor keeps VBATE (the F407 has the
 *               two on different channels, 18 and 16; RM0090 is silent about one switching off the other on this device).
 *  NEEDS REVIEW 338/AB-4  characterization: HAL_ADC_PollForEvent with an event type other than AWD / OVR (EOC here) takes the OVR path (state REG_OVR,
 *               ErrorCode OVR, OVR flag cleared) and leaves the requested flag set. UM1725 allows AWD and OVR only.
 *  NEEDS REVIEW 338/AB-5  characterization: HAL_ADC_Start / HAL_ADC_Stop do not check the state and never return HAL_BUSY for a running conversion
 *               (only for a locked handle). UM1725 gives only "HAL: status".
 *  NEEDS REVIEW 338/AB-6  characterization: the state bit TIMEOUT stays set after a PollForConversion timeout, and the handle is unlocked.
 *
 * KNOWN ISSUE candidates: V113 HAL_ADC_DeInit does not reset the registers (UM1725: "Deinitializes the ADCx peripheral registers to
 * their default reset values"; HAL_ADC_DeInit stm32f4xx_hal_adc.c lines 405-454 only clears ADON and runs MspDeInit).
 *
 * Not reachable with the model (reported): the else branches of HAL_ADC_Start / HAL_ADC_Start_IT ("ADON still 0 after the enable": ERROR_INTERNAL),
 * the "ADON still set" branch of HAL_ADC_Stop and HAL_ADC_DeInit (the model always honours a write to ADON), the 3 us stabilisation loops
 * (a plain counter loop, no observable effect; they run in every Start).
 */
#include "AdcTestSupport.h"
#include "HalFixture.h"
#include "HalHostSystem.h"
#include "TestHelpers.h"

#include <cstdint>

namespace {

const uint32_t kAbSampleTimes[8] = {ADC_SAMPLETIME_3CYCLES,  ADC_SAMPLETIME_15CYCLES, ADC_SAMPLETIME_28CYCLES,  ADC_SAMPLETIME_56CYCLES,
                                    ADC_SAMPLETIME_84CYCLES, ADC_SAMPLETIME_112CYCLES, ADC_SAMPLETIME_144CYCLES, ADC_SAMPLETIME_480CYCLES};

ADC_HandleTypeDef abHandle(unsigned n) {
	ADC_HandleTypeDef h{};
	h.Instance = adcInstance(n);
	h.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	h.Init.Resolution = ADC_RESOLUTION_12B;
	h.Init.ScanConvMode = DISABLE;
	h.Init.ContinuousConvMode = DISABLE;
	h.Init.DiscontinuousConvMode = DISABLE;
	h.Init.NbrOfDiscConversion = 1;
	h.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	h.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	h.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	h.Init.NbrOfConversion = 1;
	h.Init.DMAContinuousRequests = DISABLE;
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	return h;
}

// Back to the power-on state of every register and model, the bus armed again (hostRegsReset disarms it).
void abReset() {
	hostRegsDisarm();
	hostPeriphResetState();
	hostRegsReset();
	hostRegsArm();
	adcSupportReset();
	hostTickSet(0U);
}

void abInitOk(ADC_HandleTypeDef &h) { REQUIRE(HAL_ADC_Init(&h) == HAL_OK); }

HAL_StatusTypeDef abCfg(ADC_HandleTypeDef &h, uint32_t ch, uint32_t rank, uint32_t st = ADC_SAMPLETIME_3CYCLES) {
	ADC_ChannelConfTypeDef c{};
	c.Channel = ch;
	c.Rank = rank;
	c.SamplingTime = st;
	return HAL_ADC_ConfigChannel(&h, &c);
}

HAL_StatusTypeDef abWdg(ADC_HandleTypeDef &h, uint32_t mode, uint32_t ch, uint32_t hi, uint32_t lo, FunctionalState it) {
	ADC_AnalogWDGConfTypeDef w{};
	w.WatchdogMode = mode;
	w.Channel = ch;
	w.HighThreshold = hi;
	w.LowThreshold = lo;
	w.ITMode = it;
	return HAL_ADC_AnalogWDGConfig(&h, &w);
}

// ADC n initialised (12 bit, right aligned, software start, one conversion) with channel ch as rank 1 and analog value v.
void abSingle(ADC_HandleTypeDef &h, unsigned n, uint32_t ch, uint16_t v) {
	h = abHandle(n);
	abInitOk(h);
	REQUIRE(abCfg(h, ch, 1) == HAL_OK);
	hostAdcSetAnalog(n, ch, v);
}

}  // namespace

// ------------------------------------------------------------------ AB1 Init / DeInit

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init and DeInit with a NULL handle return HAL_ERROR") {
	CHECK(HAL_ADC_Init(nullptr) == HAL_ERROR);
	CHECK(HAL_ADC_DeInit(nullptr) == HAL_ERROR);
	CHECK(adcCounts.mspInit == 0u);
	CHECK(adcCounts.mspDeInit == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init of a handle in state RESET runs MspInit once and leaves it READY, no error, unlocked") {
	for (unsigned n = 1; n <= 3; ++n) {
		abReset();
		ADC_HandleTypeDef h = abHandle(n);
		h.ErrorCode = 0xFFu;
		h.Lock = HAL_LOCKED;
		REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
		INFO("ADC" << n);
		CHECK(adcCounts.mspInit == 1u);
		CHECK(adcCounts.last == &h);
		CHECK(h.State == HAL_ADC_STATE_READY);
		CHECK(HAL_ADC_GetState(&h) == HAL_ADC_STATE_READY);
		CHECK(h.ErrorCode == HAL_ADC_ERROR_NONE);
		CHECK(HAL_ADC_GetError(&h) == HAL_ADC_ERROR_NONE);
		CHECK(h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init of a handle that is not in state RESET does not run MspInit again but configures the registers") {
	ADC_HandleTypeDef h = abHandle(1);
	abInitOk(h);
	REQUIRE(adcCounts.mspInit == 1u);
	h.Init.Resolution = ADC_RESOLUTION_8B;
	h.Init.DataAlign = ADC_DATAALIGN_LEFT;
	REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
	CHECK(adcCounts.mspInit == 1u);
	CHECK((ADC1->CR1 & ADC_CR1_RES) == (2u << 24));
	CHECK((ADC1->CR2 & ADC_CR2_ALIGN) == ADC_CR2_ALIGN);
	CHECK(h.State == HAL_ADC_STATE_READY);
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 characterization: Init clears the busy state bits and sets READY, other state bits stay (NEEDS REVIEW 338/AB-1)") {
	ADC_HandleTypeDef h = abHandle(2);
	h.State = HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY | HAL_ADC_STATE_REG_EOC;
	REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
	CHECK(h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC));
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init with the internal error state returns HAL_ERROR, writes no register and unlocks the handle") {
	ADC_HandleTypeDef h = abHandle(1);
	h.State = HAL_ADC_STATE_ERROR_INTERNAL;
	h.Lock = HAL_LOCKED;
	h.Init.Resolution = ADC_RESOLUTION_6B;
	h.Init.ScanConvMode = ENABLE;
	h.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	CHECK(HAL_ADC_Init(&h) == HAL_ERROR);
	CHECK(h.State == HAL_ADC_STATE_ERROR_INTERNAL);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(adcCounts.mspInit == 0u);
	CHECK(ADC1->CR1 == 0u);
	CHECK(ADC1->CR2 == 0u);
	CHECK(ADC1->SQR1 == 0u);
	CHECK(ADC->CCR == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init writes the clock prescaler of every value into CCR ADCPRE and keeps the other CCR bits") {
	// RM0090 13.13.18 ADCPRE[1:0]: 00 PCLK2 / 2, 01 / 4, 10 / 6, 11 / 8.
	const uint32_t pre[4] = {ADC_CLOCK_SYNC_PCLK_DIV2, ADC_CLOCK_SYNC_PCLK_DIV4, ADC_CLOCK_SYNC_PCLK_DIV6, ADC_CLOCK_SYNC_PCLK_DIV8};
	for (unsigned n = 1; n <= 3; ++n) {
		for (unsigned i = 0; i < 4; ++i) {
			abReset();
			ADC->CCR = ADC_CCR_TSVREFE | ADC_CCR_VBATE | (3u << 16) | ADC_CCR_DDS;  // ADCPRE preloaded with 11
			ADC_HandleTypeDef h = abHandle(n);
			h.Init.ClockPrescaler = pre[i];
			REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
			INFO("ADC" << n << " prescaler code " << i);
			CHECK(ADC->CCR == (ADC_CCR_TSVREFE | ADC_CCR_VBATE | ADC_CCR_DDS | (i << 16)));
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init writes the resolution into CR1 RES (12, 10, 8, 6 bit) and keeps the other CR1 bits") {
	// RM0090 13.13.3 RES[1:0] (bits 25:24): 00 12 bit, 01 10 bit, 10 8 bit, 11 6 bit.
	const uint32_t res[4] = {ADC_RESOLUTION_12B, ADC_RESOLUTION_10B, ADC_RESOLUTION_8B, ADC_RESOLUTION_6B};
	for (unsigned n = 1; n <= 3; ++n) {
		for (unsigned i = 0; i < 4; ++i) {
			abReset();
			adcInstance(n)->CR1 = ADC_CR1_RES | ADC_CR1_AWDEN | ADC_CR1_EOCIE;  // RES preloaded with 11
			ADC_HandleTypeDef h = abHandle(n);
			h.Init.Resolution = res[i];
			REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
			INFO("ADC" << n << " RES " << i);
			CHECK(adcInstance(n)->CR1 == (ADC_CR1_AWDEN | ADC_CR1_EOCIE | (i << 24)));
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init sets CR1 SCAN (bit 8) for ScanConvMode ENABLE and clears it for DISABLE") {
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ScanConvMode = ENABLE;
	abInitOk(h);
	CHECK(ADC1->CR1 == 0x100u);
	h.Init.ScanConvMode = DISABLE;
	abInitOk(h);
	CHECK(ADC1->CR1 == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init writes the data alignment into CR2 ALIGN (bit 11)") {
	ADC_HandleTypeDef h = abHandle(3);
	h.Init.DataAlign = ADC_DATAALIGN_LEFT;
	abInitOk(h);
	CHECK((ADC3->CR2 & 0x800u) == 0x800u);
	h.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	abInitOk(h);
	CHECK((ADC3->CR2 & 0x800u) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init with software start clears EXTSEL and EXTEN") {
	ADC1->CR2 = ADC_CR2_EXTSEL | ADC_CR2_EXTEN | ADC_CR2_ALIGN;
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	h.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;  // ignored with software start
	abInitOk(h);
	CHECK((ADC1->CR2 & (ADC_CR2_EXTSEL | ADC_CR2_EXTEN)) == 0u);
	CHECK((ADC1->CR2 & ADC_CR2_ALIGN) == 0u);  // right aligned requested
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init writes the external trigger source of every event into CR2 EXTSEL") {
	// RM0090 13.13.4 EXTSEL[3:0] (bits 27:24): 0000 Timer 1 CC1, 0001 T1 CC2, 0010 T1 CC3, 0011 T2 CC2, 0100 T2 CC3, 0101 T2 CC4,
	// 0110 T2 TRGO, 0111 T3 CC1, 1000 T3 TRGO, 1001 T4 CC4, 1010 T5 CC1, 1011 T5 CC2, 1100 T5 CC3, 1101 T8 CC1, 1110 T8 TRGO, 1111 EXTI line 11.
	const uint32_t src[16] = {ADC_EXTERNALTRIGCONV_T1_CC1, ADC_EXTERNALTRIGCONV_T1_CC2, ADC_EXTERNALTRIGCONV_T1_CC3, ADC_EXTERNALTRIGCONV_T2_CC2,
	                          ADC_EXTERNALTRIGCONV_T2_CC3, ADC_EXTERNALTRIGCONV_T2_CC4, ADC_EXTERNALTRIGCONV_T2_TRGO, ADC_EXTERNALTRIGCONV_T3_CC1,
	                          ADC_EXTERNALTRIGCONV_T3_TRGO, ADC_EXTERNALTRIGCONV_T4_CC4, ADC_EXTERNALTRIGCONV_T5_CC1, ADC_EXTERNALTRIGCONV_T5_CC2,
	                          ADC_EXTERNALTRIGCONV_T5_CC3, ADC_EXTERNALTRIGCONV_T8_CC1, ADC_EXTERNALTRIGCONV_T8_TRGO, ADC_EXTERNALTRIGCONV_Ext_IT11};
	for (unsigned n = 1; n <= 3; ++n) {
		for (unsigned i = 0; i < 16; ++i) {
			abReset();
			ADC_HandleTypeDef h = abHandle(n);
			h.Init.ExternalTrigConv = src[i];
			h.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
			REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
			INFO("ADC" << n << " EXTSEL " << i);
			CHECK((adcInstance(n)->CR2 & ADC_CR2_EXTSEL) == (i << 24));
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init replaces a previous EXTSEL value instead of or-ing into it") {
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	h.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;  // 1110
	abInitOk(h);
	h.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC2;  // 0001
	abInitOk(h);
	CHECK((ADC1->CR2 & ADC_CR2_EXTSEL) == (1u << 24));
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init writes the external trigger edge into CR2 EXTEN (none, rising, falling, both)") {
	// RM0090 13.13.4 EXTEN[1:0] (bits 29:28): 00 disabled, 01 rising, 10 falling, 11 both.
	const uint32_t edge[4] = {ADC_EXTERNALTRIGCONVEDGE_NONE, ADC_EXTERNALTRIGCONVEDGE_RISING, ADC_EXTERNALTRIGCONVEDGE_FALLING,
	                          ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING};
	for (unsigned i = 0; i < 4; ++i) {
		abReset();
		ADC2->CR2 = ADC_CR2_EXTEN;  // preloaded with 11
		ADC_HandleTypeDef h = abHandle(2);
		h.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
		h.Init.ExternalTrigConvEdge = edge[i];
		REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
		INFO("EXTEN " << i);
		CHECK((ADC2->CR2 & ADC_CR2_EXTEN) == (i << 28));
		CHECK((ADC2->CR2 & ADC_CR2_EXTSEL) == (8u << 24));
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init writes ContinuousConvMode into CR2 CONT (bit 1) and DMAContinuousRequests into DDS (bit 9)") {
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ContinuousConvMode = ENABLE;
	h.Init.DMAContinuousRequests = ENABLE;
	abInitOk(h);
	CHECK((ADC1->CR2 & 0x202u) == 0x202u);
	CHECK((ADC1->CR2 & ADC_CR2_DMA) == 0u);  // Init never enables the DMA request itself
	h.Init.ContinuousConvMode = DISABLE;
	abInitOk(h);
	CHECK((ADC1->CR2 & 0x202u) == 0x200u);
	h.Init.DMAContinuousRequests = DISABLE;
	h.Init.ContinuousConvMode = ENABLE;
	abInitOk(h);
	CHECK((ADC1->CR2 & 0x202u) == 0x002u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init writes EOCSelection into CR2 EOCS (bit 10): sequence 0, single conversion 1") {
	ADC_HandleTypeDef h = abHandle(3);
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	abInitOk(h);
	CHECK((ADC3->CR2 & 0x400u) == 0x400u);
	h.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	abInitOk(h);
	CHECK((ADC3->CR2 & 0x400u) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init with discontinuous mode sets DISCEN and DISCNUM = NbrOfDiscConversion - 1 for 1 to 8 conversions") {
	// RM0090 13.13.2 DISCNUM[2:0] (bits 15:13): 000 = 1 channel ... 111 = 8 channels; DISCEN bit 11.
	for (unsigned n = 1; n <= 8; ++n) {
		abReset();
		ADC_HandleTypeDef h = abHandle(1);
		h.Init.DiscontinuousConvMode = ENABLE;
		h.Init.NbrOfDiscConversion = n;
		REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
		INFO("NbrOfDiscConversion " << n);
		CHECK((ADC1->CR1 & ADC_CR1_DISCEN) == ADC_CR1_DISCEN);
		CHECK(((ADC1->CR1 >> 13) & 7u) == n - 1u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init discontinuous mode replaces a previous DISCNUM") {
	ADC1->CR1 = ADC_CR1_DISCNUM;  // 111
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.DiscontinuousConvMode = ENABLE;
	h.Init.NbrOfDiscConversion = 2;  // DISCNUM = 001
	abInitOk(h);
	CHECK(ADC1->CR1 == (ADC_CR1_DISCEN | (1u << 13)));
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init without discontinuous mode clears DISCEN") {
	ADC1->CR1 = ADC_CR1_DISCEN | (5u << 13);
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.DiscontinuousConvMode = DISABLE;
	h.Init.NbrOfDiscConversion = 8;
	abInitOk(h);
	CHECK((ADC1->CR1 & ADC_CR1_DISCEN) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init writes the sequence length into SQR1 L = NbrOfConversion - 1 for 1 to 16 conversions and keeps ranks 13-16") {
	// RM0090 13.13.9 L[3:0] (bits 23:20): 0000 = 1 conversion ... 1111 = 16 conversions.
	for (unsigned n = 1; n <= 3; ++n) {
		for (unsigned len = 1; len <= 16; ++len) {
			abReset();
			adcInstance(n)->SQR1 = 0x000ABCDEu;  // ranks 13-16, L = 0
			ADC_HandleTypeDef h = abHandle(n);
			h.Init.NbrOfConversion = len;
			REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
			INFO("ADC" << n << " length " << len);
			CHECK(adcInstance(n)->SQR1 == (0x000ABCDEu | ((len - 1u) << 20)));
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init replaces a previous sequence length") {
	ADC1->SQR1 = 0x00F00000u;  // L = 15
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.NbrOfConversion = 3;
	abInitOk(h);
	CHECK(ADC1->SQR1 == (2u << 20));
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init does not enable the converter, start a conversion or touch the sampling and watchdog registers") {
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ScanConvMode = ENABLE;
	h.Init.NbrOfConversion = 4;
	abInitOk(h);
	CHECK((ADC1->CR2 & ADC_CR2_ADON) == 0u);
	CHECK(hostAdcRegularRunning(1) == 0);
	CHECK(ADC1->SR == 0u);
	CHECK(ADC1->SMPR1 == 0u);
	CHECK(ADC1->SMPR2 == 0u);
	CHECK(ADC1->HTR == 0xFFFu);  // reset value
	CHECK(ADC1->LTR == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 DeInit runs MspDeInit once, stops the converter, sets state RESET and clears the error code and the lock") {
	for (unsigned n = 1; n <= 3; ++n) {
		abReset();
		ADC_HandleTypeDef h = abHandle(n);
		abInitOk(h);
		REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
		REQUIRE((adcInstance(n)->CR2 & ADC_CR2_ADON) != 0u);
		h.ErrorCode = HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA;
		h.Lock = HAL_LOCKED;
		INFO("ADC" << n);
		CHECK(HAL_ADC_DeInit(&h) == HAL_OK);
		CHECK(adcCounts.mspDeInit == 1u);
		CHECK(adcCounts.last == &h);
		CHECK((adcInstance(n)->CR2 & ADC_CR2_ADON) == 0u);
		CHECK(h.State == HAL_ADC_STATE_RESET);
		CHECK(h.ErrorCode == HAL_ADC_ERROR_NONE);
		CHECK(h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 Init after DeInit runs MspInit again") {
	ADC_HandleTypeDef h = abHandle(2);
	abInitOk(h);
	REQUIRE(HAL_ADC_DeInit(&h) == HAL_OK);
	abInitOk(h);
	CHECK(adcCounts.mspInit == 2u);
	CHECK(h.State == HAL_ADC_STATE_READY);
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 DeInit puts CR1 back to its reset value" * KNOWN_ISSUE("V113 UM1725: HAL_ADC_DeInit 'Deinitializes the ADCx peripheral registers to their default reset values'; stm32f4xx_hal_adc.c 405-454 only clears ADON and calls MspDeInit, CR1 keeps SCAN / RES")) {
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ScanConvMode = ENABLE;
	h.Init.Resolution = ADC_RESOLUTION_6B;
	abInitOk(h);
	REQUIRE(ADC1->CR1 == (ADC_CR1_SCAN | ADC_CR1_RES));
	REQUIRE(HAL_ADC_DeInit(&h) == HAL_OK);
	CHECK(ADC1->CR1 == 0u);  // RM0090 13.13.2 reset value 0x00000000
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 DeInit puts CR2 and SQR1 back to their reset values" * KNOWN_ISSUE("V113 UM1725: HAL_ADC_DeInit 'Deinitializes the ADCx peripheral registers to their default reset values'; stm32f4xx_hal_adc.c 405-454 leaves CR2 ALIGN / EOCS / CONT and SQR1 L as Init set them")) {
	ADC_HandleTypeDef h = abHandle(2);
	h.Init.DataAlign = ADC_DATAALIGN_LEFT;
	h.Init.ContinuousConvMode = ENABLE;
	h.Init.NbrOfConversion = 5;
	abInitOk(h);
	REQUIRE(HAL_ADC_DeInit(&h) == HAL_OK);
	CHECK(ADC2->CR2 == 0u);   // RM0090 13.13.3 reset value 0
	CHECK(ADC2->SQR1 == 0u);  // 13.13.9 reset value 0
}

// ------------------------------------------------------------------ AB2 ConfigChannel

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel places the channel of ranks 1 to 6 in SQR3 at bit 5 * (rank - 1)") {
	// RM0090 13.13.11 SQR3: SQ1[4:0] bits 4:0, SQ2 9:5, ... SQ6 29:25.
	for (unsigned n = 1; n <= 3; ++n) {
		abReset();
		ADC_HandleTypeDef h = abHandle(n);
		abInitOk(h);
		uint32_t expect = 0;
		for (uint32_t rank = 1; rank <= 6; ++rank) {
			const uint32_t ch = rank + 10u;  // 11..16, distinct nonzero values
			REQUIRE(abCfg(h, ch, rank) == HAL_OK);
			expect |= ch << (5u * (rank - 1u));
		}
		INFO("ADC" << n);
		CHECK(adcInstance(n)->SQR3 == expect);
		CHECK(adcInstance(n)->SQR2 == 0u);
		CHECK(adcInstance(n)->SQR1 == 0u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel places the channel of ranks 7 to 12 in SQR2 at bit 5 * (rank - 7)") {
	// RM0090 13.13.10 SQR2: SQ7[4:0] bits 4:0 ... SQ12 29:25.
	ADC_HandleTypeDef h = abHandle(1);
	abInitOk(h);
	uint32_t expect = 0;
	for (uint32_t rank = 7; rank <= 12; ++rank) {
		const uint32_t ch = rank + 2u;  // 9..14
		REQUIRE(abCfg(h, ch, rank) == HAL_OK);
		expect |= ch << (5u * (rank - 7u));
	}
	CHECK(ADC1->SQR2 == expect);
	CHECK(ADC1->SQR3 == 0u);
	CHECK(ADC1->SQR1 == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel places the channel of ranks 13 to 16 in SQR1 at bit 5 * (rank - 13) and leaves L alone") {
	// RM0090 13.13.9 SQR1: SQ13[4:0] bits 4:0 ... SQ16 19:15, L bits 23:20.
	ADC_HandleTypeDef h = abHandle(3);
	h.Init.NbrOfConversion = 16;  // L = 15
	abInitOk(h);
	uint32_t expect = 15u << 20;
	for (uint32_t rank = 13; rank <= 16; ++rank) {
		const uint32_t c = rank - 5u;  // 8..11
		REQUIRE(abCfg(h, c, rank) == HAL_OK);
		expect |= c << (5u * (rank - 13u));
	}
	CHECK(ADC3->SQR1 == expect);
	CHECK(ADC3->SQR2 == 0u);
	CHECK(ADC3->SQR3 == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel of every rank 1 to 16 returns HAL_OK and sets exactly the 5 bit field of that rank") {
	for (uint32_t rank = 1; rank <= 16; ++rank) {
		abReset();
		ADC_HandleTypeDef h = abHandle(1);
		abInitOk(h);
		REQUIRE(HAL_OK == abCfg(h, ADC_CHANNEL_6, rank));  // 6 = 00110
		const uint32_t shift = rank < 7 ? 5u * (rank - 1u) : (rank < 13 ? 5u * (rank - 7u) : 5u * (rank - 13u));
		INFO("rank " << rank);
		CHECK(ADC1->SQR3 == (rank < 7 ? (6u << shift) : 0u));
		CHECK(ADC1->SQR2 == ((rank >= 7 && rank < 13) ? (6u << shift) : 0u));
		CHECK(ADC1->SQR1 == (rank >= 13 ? (6u << shift) : 0u));
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel replaces the old channel of a rank and leaves the neighbouring ranks") {
	ADC1->SQR3 = 0x3FFFFFFFu;  // all six ranks = 11111
	ADC1->SQR2 = 0x3FFFFFFFu;
	ADC1->SQR1 = 0x000FFFFFu;  // ranks 13-16 = 11111, L = 0
	ADC_HandleTypeDef h = abHandle(1);
	h.State = HAL_ADC_STATE_READY;
	REQUIRE(abCfg(h, ADC_CHANNEL_2, 3) == HAL_OK);
	REQUIRE(abCfg(h, ADC_CHANNEL_0, 9) == HAL_OK);
	REQUIRE(abCfg(h, ADC_CHANNEL_1, 14) == HAL_OK);
	CHECK(ADC1->SQR3 == ((0x3FFFFFFFu & ~(0x1Fu << 10)) | (2u << 10)));
	CHECK(ADC1->SQR2 == (0x3FFFFFFFu & ~(0x1Fu << 10)));
	CHECK(ADC1->SQR1 == ((0x000FFFFFu & ~(0x1Fu << 5)) | (1u << 5)));
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel sets the 3 bit sampling time of channels 0 to 9 in SMPR2 at bit 3 * channel") {
	// RM0090 13.13.7 SMPR2: SMP0[2:0] bits 2:0 ... SMP9 bits 29:27; code 000 = 3 cycles ... 111 = 480 cycles.
	for (uint32_t ch = 0; ch <= 9; ++ch) {
		for (uint32_t st = 0; st < 8; ++st) {
			abReset();
			ADC_HandleTypeDef h = abHandle(1);
			abInitOk(h);
			REQUIRE(abCfg(h, ch, 1, kAbSampleTimes[st]) == HAL_OK);
			INFO("channel " << ch << " sampling code " << st);
			CHECK(ADC1->SMPR2 == (st << (3u * ch)));
			CHECK(ADC1->SMPR1 == 0u);
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel sets the 3 bit sampling time of channels 10 to 18 in SMPR1 at bit 3 * (channel - 10)") {
	// RM0090 13.13.6 SMPR1: SMP10[2:0] bits 2:0 ... SMP18 bits 26:24.
	for (uint32_t ch = 10; ch <= 18; ++ch) {
		for (uint32_t st = 0; st < 8; ++st) {
			abReset();
			ADC_HandleTypeDef h = abHandle(1);
			abInitOk(h);
			REQUIRE(abCfg(h, ch, 1, kAbSampleTimes[st]) == HAL_OK);
			INFO("channel " << ch << " sampling code " << st);
			CHECK(ADC1->SMPR1 == (st << (3u * (ch - 10u))));
			CHECK(ADC1->SMPR2 == 0u);
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel replaces the old sampling time of a channel and keeps the other channels") {
	ADC2->SMPR1 = 0x07FFFFFFu;
	ADC2->SMPR2 = 0x3FFFFFFFu;
	ADC_HandleTypeDef h = abHandle(2);
	h.State = HAL_ADC_STATE_READY;
	REQUIRE(abCfg(h, ADC_CHANNEL_12, 1, ADC_SAMPLETIME_15CYCLES) == HAL_OK);  // code 001
	REQUIRE(abCfg(h, ADC_CHANNEL_4, 1, ADC_SAMPLETIME_3CYCLES) == HAL_OK);    // code 000
	CHECK(ADC2->SMPR1 == ((0x07FFFFFFu & ~(7u << 6)) | (1u << 6)));
	CHECK(ADC2->SMPR2 == (0x3FFFFFFFu & ~(7u << 12)));
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel works the same way on ADC2 and ADC3 for sampling time and rank") {
	for (unsigned n = 2; n <= 3; ++n) {
		abReset();
		ADC_HandleTypeDef h = abHandle(n);
		abInitOk(h);
		REQUIRE(abCfg(h, ADC_CHANNEL_14, 8, ADC_SAMPLETIME_480CYCLES) == HAL_OK);
		INFO("ADC" << n);
		CHECK(adcInstance(n)->SMPR1 == (7u << 12));
		CHECK(adcInstance(n)->SQR2 == (14u << 5));
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel does not touch the CCR of an ordinary channel") {
	ADC_HandleTypeDef h = abHandle(1);
	abInitOk(h);
	const uint32_t ccr = ADC->CCR;
	for (uint32_t ch = 0; ch <= 15; ++ch) REQUIRE(abCfg(h, ch, 1) == HAL_OK);
	CHECK(ADC->CCR == ccr);
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel of the temperature sensor channel 16 on ADC1 sets TSVREFE") {
	// RM0090 13.10: the temperature sensor is on ADC1_IN16; TSVREFE (CCR bit 23) wakes it up.
	ADC_HandleTypeDef h = abHandle(1);
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_TEMPSENSOR, 1, ADC_SAMPLETIME_480CYCLES) == HAL_OK);
	CHECK((ADC->CCR & ADC_CCR_TSVREFE) == ADC_CCR_TSVREFE);
	CHECK((ADC->CCR & ADC_CCR_VBATE) == 0u);
	CHECK(ADC1->SMPR1 == (7u << 18));  // channel 16: bit 3 * 6
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel of VREFINT channel 17 on ADC1 sets TSVREFE") {
	ADC_HandleTypeDef h = abHandle(1);
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_VREFINT, 1) == HAL_OK);
	CHECK((ADC->CCR & ADC_CCR_TSVREFE) == ADC_CCR_TSVREFE);
	CHECK((ADC->CCR & ADC_CCR_VBATE) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel of the VBAT channel 18 on ADC1 sets VBATE") {
	ADC_HandleTypeDef h = abHandle(1);
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_VBAT, 1) == HAL_OK);
	CHECK((ADC->CCR & ADC_CCR_VBATE) == ADC_CCR_VBATE);
	CHECK((ADC->CCR & ADC_CCR_TSVREFE) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel of the internal channels 16, 17, 18 on ADC2 and ADC3 leaves TSVREFE and VBATE clear") {
	// The internal channels exist on ADC1 only (RM0090 13.10 and 13.11), so the HAL must not enable them from another ADC.
	for (unsigned n = 2; n <= 3; ++n) {
		for (uint32_t ch = 16; ch <= 18; ++ch) {
			abReset();
			ADC_HandleTypeDef h = abHandle(n);
			abInitOk(h);
			REQUIRE(abCfg(h, ch, 1) == HAL_OK);
			INFO("ADC" << n << " channel " << ch);
			CHECK((ADC->CCR & (ADC_CCR_TSVREFE | ADC_CCR_VBATE)) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 characterization: on ADC1 the VBAT channel keeps TSVREFE and the temperature channel keeps VBATE (NEEDS REVIEW 338/AB-3)") {
	ADC_HandleTypeDef h = abHandle(1);
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_TEMPSENSOR, 1) == HAL_OK);
	REQUIRE(abCfg(h, ADC_CHANNEL_VBAT, 2) == HAL_OK);
	CHECK((ADC->CCR & (ADC_CCR_TSVREFE | ADC_CCR_VBATE)) == (ADC_CCR_TSVREFE | ADC_CCR_VBATE));
	abReset();
	h = abHandle(1);
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_VBAT, 1) == HAL_OK);
	REQUIRE(abCfg(h, ADC_CHANNEL_VREFINT, 2) == HAL_OK);
	CHECK((ADC->CCR & (ADC_CCR_TSVREFE | ADC_CCR_VBATE)) == (ADC_CCR_TSVREFE | ADC_CCR_VBATE));
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel of the internal channels keeps ADCPRE and the other CCR bits") {
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_VBAT, 1) == HAL_OK);
	CHECK((ADC->CCR & ADC_CCR_ADCPRE) == (2u << 16));
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel of a locked handle returns HAL_BUSY and writes no register") {
	ADC_HandleTypeDef h = abHandle(1);
	abInitOk(h);
	h.Lock = HAL_LOCKED;
	CHECK(abCfg(h, ADC_CHANNEL_VBAT, 5, ADC_SAMPLETIME_480CYCLES) == HAL_BUSY);
	CHECK(ADC1->SQR3 == 0u);
	CHECK(ADC1->SMPR1 == 0u);
	CHECK(ADC->CCR == (ADC_CLOCK_SYNC_PCLK_DIV4 | 0u));
	CHECK(h.Lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AB2 ConfigChannel unlocks the handle and works while a conversion is running") {
	ADC_HandleTypeDef h = abHandle(1);
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_1, 1) == HAL_OK);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK(abCfg(h, ADC_CHANNEL_9, 1) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((ADC1->SQR3 & 0x1Fu) == 9u);
	CHECK(h.State == HAL_ADC_STATE_REG_BUSY);  // configuring does not change the state
}

// ------------------------------------------------------------------ AB3 Start / Stop

TEST_CASE_FIXTURE(AdcFixture, "AB3 Start sets ADON, launches the regular conversion by software and leaves the state REG_BUSY") {
	for (unsigned n = 1; n <= 3; ++n) {
		abReset();
		ADC_HandleTypeDef h;
		abSingle(h, n, ADC_CHANNEL_5, 0x123);
		REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
		INFO("ADC" << n);
		CHECK((adcInstance(n)->CR2 & ADC_CR2_ADON) == ADC_CR2_ADON);
		CHECK(hostAdcRegularRunning(n) == 1);
		CHECK((adcInstance(n)->SR & ADC_SR_STRT) == ADC_SR_STRT);  // RM0090 13.13.1: STRT is set when the regular conversion starts
		CHECK(h.State == HAL_ADC_STATE_REG_BUSY);
		CHECK(h.ErrorCode == HAL_ADC_ERROR_NONE);
		CHECK(h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 Start with an external trigger enables the converter but does not start a conversion") {
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	h.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	abInitOk(h);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK((ADC1->CR2 & ADC_CR2_ADON) == ADC_CR2_ADON);
	CHECK(hostAdcRegularRunning(1) == 0);
	CHECK((ADC1->SR & ADC_SR_STRT) == 0u);
	CHECK(h.State == HAL_ADC_STATE_REG_BUSY);  // "a conversion ... can occur" (UM1725 state description)
	CHECK(hostAdcExternalTrigger(1, 0) == 1);  // the trigger event then starts it
	CHECK(hostAdcRegularRunning(1) == 1);
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 Start clears the EOC and OVR flags and the stale state bits READY, REG_EOC, REG_OVR") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_1, 10);
	HalFixture::poke(&ADC1->SR, ADC_SR_EOC | ADC_SR_OVR | ADC_SR_AWD);
	h.State = HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_AWD1;
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK((ADC1->SR & (ADC_SR_EOC | ADC_SR_OVR)) == 0u);
	CHECK((ADC1->SR & ADC_SR_AWD) == ADC_SR_AWD);  // other flags stay
	CHECK(h.State == (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_AWD1));
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 Start clears a previous overrun and DMA error code") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_1, 10);
	h.ErrorCode = HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA | HAL_ADC_ERROR_INTERNAL;
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_ADC_ERROR_NONE);
	CHECK(HAL_ADC_GetError(&h) == HAL_ADC_ERROR_NONE);
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 Start with JAUTO set marks the injected group busy and clears INJ_EOC") {
	ADC_HandleTypeDef h;
	abSingle(h, 2, ADC_CHANNEL_1, 10);
	ADC2->CR1 |= ADC_CR1_JAUTO;
	h.State |= HAL_ADC_STATE_INJ_EOC;
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK((h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
	CHECK((h.State & HAL_ADC_STATE_INJ_EOC) == 0u);
	CHECK((h.State & HAL_ADC_STATE_REG_BUSY) == HAL_ADC_STATE_REG_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 characterization: Start with the injected group busy clears only the OVR and DMA error bits (NEEDS REVIEW 338/AB-2)") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_1, 10);
	h.State |= HAL_ADC_STATE_INJ_BUSY;
	h.ErrorCode = HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA | HAL_ADC_ERROR_INTERNAL;
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_ADC_ERROR_INTERNAL);
	CHECK((h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 Start of a converter that is already enabled keeps ADON and returns HAL_OK") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_1, 10);
	ADC1->CR2 |= ADC_CR2_ADON;
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK((ADC1->CR2 & ADC_CR2_ADON) == ADC_CR2_ADON);
	CHECK(hostAdcRegularRunning(1) == 1);
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 Start of a locked handle returns HAL_BUSY without enabling the converter") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_1, 10);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_ADC_Start(&h) == HAL_BUSY);
	CHECK((ADC1->CR2 & ADC_CR2_ADON) == 0u);
	CHECK(h.State == HAL_ADC_STATE_READY);
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 characterization: Start of a handle that is already running returns HAL_OK (NEEDS REVIEW 338/AB-5)") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_1, 10);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK(HAL_ADC_Start(&h) == HAL_OK);
	CHECK(h.State == HAL_ADC_STATE_REG_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 Start with ADC1 as the multi mode master still starts by software") {
	// RM0090 13.9: in multi ADC mode the master (ADC1) is triggered, the slaves follow it.
	ADC->CCR |= 0x6u;  // MULTI = 00110 regular simultaneous
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_1, 10);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK(hostAdcRegularRunning(1) == 1);
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 Start with ADC2 or ADC3 as multi mode slave does not start the conversion by software") {
	for (unsigned n = 2; n <= 3; ++n) {
		abReset();
		ADC->CCR |= 0x6u;  // MULTI != 0
		ADC_HandleTypeDef h;
		abSingle(h, n, ADC_CHANNEL_1, 10);
		REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
		INFO("ADC" << n);
		CHECK((adcInstance(n)->CR2 & ADC_CR2_ADON) == ADC_CR2_ADON);
		CHECK(hostAdcRegularRunning(n) == 0);
		CHECK(h.State == HAL_ADC_STATE_REG_BUSY);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 Start with ADC1 as multi mode master and an external trigger does not start by software") {
	ADC->CCR |= 0x6u;
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC2;
	h.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
	abInitOk(h);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK(hostAdcRegularRunning(1) == 0);
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 Stop clears ADON, ends the conversion, sets the state READY and returns HAL_OK") {
	for (unsigned n = 1; n <= 3; ++n) {
		abReset();
		ADC_HandleTypeDef h;
		abSingle(h, n, ADC_CHANNEL_5, 100);
		REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
		INFO("ADC" << n);
		CHECK(HAL_ADC_Stop(&h) == HAL_OK);
		CHECK((adcInstance(n)->CR2 & ADC_CR2_ADON) == 0u);
		CHECK(hostAdcRegularRunning(n) == 0);
		CHECK(h.State == HAL_ADC_STATE_READY);
		CHECK(h.Lock == HAL_UNLOCKED);
		CHECK(hostAdcConvert(n, 1) == 0u);  // a stopped converter does nothing
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 Stop also ends a running injected state (UM1725: Stop stops injected channels too)") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_1, 100);
	h.State = HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY;
	ADC1->CR2 |= ADC_CR2_ADON;
	REQUIRE(HAL_ADC_Stop(&h) == HAL_OK);
	CHECK(h.State == HAL_ADC_STATE_READY);
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 Stop of a locked handle returns HAL_BUSY and leaves the converter running") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_1, 10);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_ADC_Stop(&h) == HAL_BUSY);
	CHECK((ADC1->CR2 & ADC_CR2_ADON) == ADC_CR2_ADON);
	CHECK(h.State == HAL_ADC_STATE_REG_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 characterization: Stop of a converter that was never started returns HAL_OK (NEEDS REVIEW 338/AB-5)") {
	ADC_HandleTypeDef h = abHandle(3);
	abInitOk(h);
	CHECK(HAL_ADC_Stop(&h) == HAL_OK);
	CHECK(h.State == HAL_ADC_STATE_READY);
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 Stop reaches READY for a converter with CONT, ALIGN, DDS, DMA and EOCS set and never sets ERROR_INTERNAL") {
	for (unsigned n = 1; n <= 3; ++n) {
		abReset();
		ADC_HandleTypeDef h = abHandle(n);
		h.Init.ContinuousConvMode = ENABLE;
		h.Init.DataAlign = ADC_DATAALIGN_LEFT;
		h.Init.DMAContinuousRequests = ENABLE;
		h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
		abInitOk(h);
		adcInstance(n)->CR2 |= ADC_CR2_DMA;
		REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
		INFO("ADC" << n);
		CHECK(HAL_ADC_Stop(&h) == HAL_OK);
		CHECK(h.State == HAL_ADC_STATE_READY);
		CHECK((h.State & HAL_ADC_STATE_ERROR_INTERNAL) == 0u);
		CHECK((adcInstance(n)->CR2 & ADC_CR2_ADON) == 0u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB1 DeInit of a converter with CONT, ALIGN, DDS, DMA and EOCS set runs MspDeInit and ends in RESET without ERROR_INTERNAL") {
	for (unsigned n = 1; n <= 3; ++n) {
		abReset();
		ADC_HandleTypeDef h = abHandle(n);
		h.Init.ContinuousConvMode = ENABLE;
		h.Init.DataAlign = ADC_DATAALIGN_LEFT;
		h.Init.DMAContinuousRequests = ENABLE;
		h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
		abInitOk(h);
		adcInstance(n)->CR2 |= ADC_CR2_DMA;
		REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
		INFO("ADC" << n);
		CHECK(HAL_ADC_DeInit(&h) == HAL_OK);
		CHECK(adcCounts.mspDeInit == 1u);
		CHECK(h.State == HAL_ADC_STATE_RESET);
		CHECK(h.ErrorCode == HAL_ADC_ERROR_NONE);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 Start still starts by software when EXTEN is 00 although EXTSEL names a timer event") {
	for (unsigned multi = 0; multi < 2; ++multi) {
		abReset();
		if (multi) ADC->CCR |= 0x6u;  // MULTI != 0: ADC1 is the master
		ADC_HandleTypeDef h = abHandle(1);
		h.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
		h.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		abInitOk(h);
		REQUIRE((ADC1->CR2 & ADC_CR2_EXTSEL) == (14u << 24));
		REQUIRE((ADC1->CR2 & ADC_CR2_EXTEN) == 0u);
		REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
		INFO("multi " << multi);
		CHECK(hostAdcRegularRunning(1) == 1);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB3 a conversion can be started again after Stop") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_3, 0x55);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(HAL_ADC_Stop(&h) == HAL_OK);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK(hostAdcRegularRunning(1) == 1);
	CHECK(hostAdcConvert(1, 1) == 1u);
	CHECK(HAL_ADC_GetValue(&h) == 0x55u);
}

// ------------------------------------------------------------------ AB4 PollForConversion / PollForEvent / GetValue

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion returns HAL_OK once the conversion is done, clears STRT and EOC and sets REG_EOC") {
	for (unsigned n = 1; n <= 3; ++n) {
		abReset();
		ADC_HandleTypeDef h;
		abSingle(h, n, ADC_CHANNEL_7, 0x2A5);
		REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
		REQUIRE(hostAdcConvert(n, 1) == 1u);
		REQUIRE((adcInstance(n)->SR & ADC_SR_EOC) != 0u);
		INFO("ADC" << n);
		CHECK(HAL_ADC_PollForConversion(&h, 10) == HAL_OK);
		CHECK((adcInstance(n)->SR & (ADC_SR_EOC | ADC_SR_STRT)) == 0u);
		CHECK((h.State & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC);
		CHECK(HAL_ADC_GetValue(&h) == 0x2A5u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion leaves other flags (AWD, OVR) set") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_7, 0x2A5);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	HalFixture::poke(&ADC1->SR, HalFixture::peek(&ADC1->SR) | ADC_SR_AWD | ADC_SR_JEOC);
	REQUIRE(HAL_ADC_PollForConversion(&h, 10) == HAL_OK);
	CHECK((ADC1->SR & (ADC_SR_AWD | ADC_SR_JEOC)) == (ADC_SR_AWD | ADC_SR_JEOC));
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion after a software started single conversion leaves the state READY with REG_EOC") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_7, 1);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	REQUIRE(HAL_ADC_PollForConversion(&h, 10) == HAL_OK);
	CHECK(h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC));
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion in continuous mode keeps the state REG_BUSY") {
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ContinuousConvMode = ENABLE;
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_7, 1) == HAL_OK);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	REQUIRE(HAL_ADC_PollForConversion(&h, 10) == HAL_OK);
	CHECK(h.State == (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_REG_EOC));
	CHECK(hostAdcRegularRunning(1) == 1);  // RM0090 13.3.3: continuous mode restarts by itself
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion with an external trigger keeps the state REG_BUSY") {
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	h.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_7, 1) == HAL_OK);
	hostAdcSetAnalog(1, 7, 77);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcExternalTrigger(1, 0) == 1);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	REQUIRE(HAL_ADC_PollForConversion(&h, 10) == HAL_OK);
	CHECK(h.State == (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_REG_EOC));
	CHECK(HAL_ADC_GetValue(&h) == 77u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion after one rank of a scan sequence with EOC per conversion keeps REG_BUSY") {
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ScanConvMode = ENABLE;
	h.Init.NbrOfConversion = 2;
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_1, 1) == HAL_OK);
	REQUIRE(abCfg(h, ADC_CHANNEL_2, 2) == HAL_OK);
	hostAdcSetAnalog(1, 1, 11);
	hostAdcSetAnalog(1, 2, 22);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	REQUIRE(HAL_ADC_PollForConversion(&h, 10) == HAL_OK);
	CHECK(h.State == (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_REG_EOC));
	CHECK(HAL_ADC_GetValue(&h) == 11u);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	REQUIRE(HAL_ADC_PollForConversion(&h, 10) == HAL_OK);
	CHECK(HAL_ADC_GetValue(&h) == 22u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion of a scan sequence with EOC at the end of the sequence returns the last rank and the state READY") {
	ADC_HandleTypeDef h = abHandle(2);
	h.Init.ScanConvMode = ENABLE;
	h.Init.NbrOfConversion = 3;
	h.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_1, 1) == HAL_OK);
	REQUIRE(abCfg(h, ADC_CHANNEL_2, 2) == HAL_OK);
	REQUIRE(abCfg(h, ADC_CHANNEL_3, 3) == HAL_OK);
	hostAdcSetAnalog(2, 1, 100);
	hostAdcSetAnalog(2, 2, 200);
	hostAdcSetAnalog(2, 3, 300);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK(hostAdcConvert(2, 2) == 2u);
	CHECK(HAL_ADC_PollForConversion(&h, 0) == HAL_TIMEOUT);  // EOC only after the last rank
	h.State = HAL_ADC_STATE_REG_BUSY;
	h.Lock = HAL_UNLOCKED;
	CHECK(hostAdcConvert(2, 1) == 1u);
	REQUIRE(HAL_ADC_PollForConversion(&h, 10) == HAL_OK);
	CHECK(h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC));
	CHECK(HAL_ADC_GetValue(&h) == 300u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion does not set READY while the injected group is busy") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_7, 5);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	h.State |= HAL_ADC_STATE_INJ_BUSY;
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	REQUIRE(HAL_ADC_PollForConversion(&h, 10) == HAL_OK);
	CHECK(h.State == (HAL_ADC_STATE_INJ_BUSY | HAL_ADC_STATE_REG_EOC));
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion with Timeout 0 and no conversion returns HAL_TIMEOUT, sets the TIMEOUT state bit and unlocks") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_7, 5);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_ADC_PollForConversion(&h, 0) == HAL_TIMEOUT);
	CHECK((h.State & HAL_ADC_STATE_TIMEOUT) == HAL_ADC_STATE_TIMEOUT);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 characterization: the TIMEOUT state bit stays set next to REG_BUSY after a poll timeout (NEEDS REVIEW 338/AB-6)") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_7, 5);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK(HAL_ADC_PollForConversion(&h, 0) == HAL_TIMEOUT);
	CHECK(h.State == (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_TIMEOUT));
	CHECK(hostAdcRegularRunning(1) == 1);  // the conversion is not cancelled by the timeout
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion waits about Timeout ticks before it returns HAL_TIMEOUT") {
	const uint32_t timeouts[4] = {1, 5, 10, 50};
	for (uint32_t t : timeouts) {
		abReset();
		ADC_HandleTypeDef h;
		abSingle(h, 1, ADC_CHANNEL_7, 5);
		REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
		const uint32_t before = hostTickPeek();
		REQUIRE(HAL_ADC_PollForConversion(&h, t) == HAL_TIMEOUT);
		const uint32_t elapsed = hostTickPeek() - before;
		INFO("Timeout " << t << " elapsed " << elapsed);
		CHECK(elapsed >= t);
		CHECK(elapsed <= t + 3u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion with HAL_MAX_DELAY waits until the conversion completes") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_7, 0x321);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	hostAdcSetPollReads(1, 5);  // the conversion finishes after the loop read the status 5 times
	CHECK(HAL_ADC_PollForConversion(&h, HAL_MAX_DELAY) == HAL_OK);
	CHECK(HAL_ADC_GetValue(&h) == 0x321u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion with a finite Timeout succeeds when the conversion ends in time") {
	ADC_HandleTypeDef h;
	abSingle(h, 3, ADC_CHANNEL_9, 0x7F);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	hostAdcSetPollReads(3, 1);
	CHECK(HAL_ADC_PollForConversion(&h, 100) == HAL_OK);
	CHECK(HAL_ADC_GetValue(&h) == 0x7Fu);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion without a started conversion times out") {
	ADC_HandleTypeDef h = abHandle(2);
	abInitOk(h);
	CHECK(HAL_ADC_PollForConversion(&h, 3) == HAL_TIMEOUT);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion with EOCS and DMA both set returns HAL_ERROR with the CONFIG error state, unlocked") {
	// UM1725: not possible in DMA mode with EOCSelection ADC_EOC_SINGLE_CONV.
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	abInitOk(h);
	ADC1->CR2 |= ADC_CR2_DMA;
	h.Lock = HAL_LOCKED;
	CHECK(HAL_ADC_PollForConversion(&h, 10) == HAL_ERROR);
	CHECK((h.State & HAL_ADC_STATE_ERROR_CONFIG) == HAL_ADC_STATE_ERROR_CONFIG);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForConversion with DMA set but EOCS clear (sequence end) is allowed") {
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_1, 1) == HAL_OK);
	ADC1->CR2 |= ADC_CR2_DMA;
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	CHECK(HAL_ADC_PollForConversion(&h, 10) == HAL_OK);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 GetValue returns DR right aligned for every resolution (model check of the data path)") {
	// Analog input 0x123 (291) at 12 bit; reduced to 10 / 8 / 6 bit by dropping 2 / 4 / 6 low bits: 0x48, 0x12, 0x04 (RM0090 13.8).
	const uint32_t res[4] = {ADC_RESOLUTION_12B, ADC_RESOLUTION_10B, ADC_RESOLUTION_8B, ADC_RESOLUTION_6B};
	const uint32_t expect[4] = {0x123u, 0x48u, 0x12u, 0x04u};
	for (unsigned i = 0; i < 4; ++i) {
		abReset();
		ADC_HandleTypeDef h = abHandle(1);
		h.Init.Resolution = res[i];
		abInitOk(h);
		REQUIRE(abCfg(h, ADC_CHANNEL_4, 1) == HAL_OK);
		hostAdcSetAnalog(1, 4, 0x123);
		REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
		REQUIRE(hostAdcConvert(1, 1) == 1u);
		INFO("resolution code " << i);
		CHECK(HAL_ADC_GetValue(&h) == expect[i]);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 GetValue returns DR left aligned for every resolution") {
	// RM0090 13.4 Figures 49 / 50: regular data is left aligned on a half-word basis (MSB at bit 15): 12 bit 0x123 << 4 = 0x1230,
	// 10 bit 0x48 << 6 = 0x1200, 8 bit 0x12 << 8 = 0x1200; 6 bit data is aligned on a byte basis (D5 at bit 7): 0x04 << 2 = 0x10.
	// (Model check of the alignment of DR: the HAL only returns DR.)
	const uint32_t res[4] = {ADC_RESOLUTION_12B, ADC_RESOLUTION_10B, ADC_RESOLUTION_8B, ADC_RESOLUTION_6B};
	const uint32_t expect[4] = {0x1230u, 0x1200u, 0x1200u, 0x10u};
	for (unsigned i = 0; i < 4; ++i) {
		abReset();
		ADC_HandleTypeDef h = abHandle(2);
		h.Init.Resolution = res[i];
		h.Init.DataAlign = ADC_DATAALIGN_LEFT;
		abInitOk(h);
		REQUIRE(abCfg(h, ADC_CHANNEL_4, 1) == HAL_OK);
		hostAdcSetAnalog(2, 4, 0x123);
		REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
		REQUIRE(hostAdcConvert(2, 1) == 1u);
		INFO("resolution code " << i);
		CHECK(HAL_ADC_GetValue(&h) == expect[i]);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 GetValue of the full scale and of zero at 12 bit") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_0, 0xFFF);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	CHECK(HAL_ADC_GetValue(&h) == 0xFFFu);
	hostAdcSetAnalog(1, 0, 0);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	CHECK(HAL_ADC_GetValue(&h) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 GetValue reading DR clears EOC") {
	// RM0090 13.13.1 EOC: cleared by software or by reading ADC_DR.
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_0, 9);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	REQUIRE((ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC);
	CHECK(HAL_ADC_GetValue(&h) == 9u);
	CHECK((ADC1->SR & ADC_SR_EOC) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 GetValue reads each rank of a sequence in turn") {
	ADC_HandleTypeDef h = abHandle(3);
	h.Init.ScanConvMode = ENABLE;
	h.Init.NbrOfConversion = 3;
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	abInitOk(h);
	const uint32_t chs[3] = {ADC_CHANNEL_8, ADC_CHANNEL_9, ADC_CHANNEL_10};
	const uint16_t vals[3] = {0x111, 0x222, 0x333};
	for (uint32_t i = 0; i < 3; ++i) {
		REQUIRE(abCfg(h, chs[i], i + 1u) == HAL_OK);
		hostAdcSetAnalog(3, chs[i], vals[i]);
	}
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	for (unsigned i = 0; i < 3; ++i) {
		REQUIRE(hostAdcConvert(3, 1) == 1u);
		REQUIRE(HAL_ADC_PollForConversion(&h, 10) == HAL_OK);
		CHECK(HAL_ADC_GetValue(&h) == vals[i]);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 GetState and GetError return the state and the error code of the handle") {
	ADC_HandleTypeDef h = abHandle(1);
	h.State = 0x12345678u;
	h.ErrorCode = 0x0Fu;
	CHECK(HAL_ADC_GetState(&h) == 0x12345678u);
	CHECK(HAL_ADC_GetError(&h) == 0x0Fu);
	h.State = HAL_ADC_STATE_RESET;
	abInitOk(h);
	CHECK(HAL_ADC_GetState(&h) == HAL_ADC_STATE_READY);
	CHECK(HAL_ADC_GetError(&h) == HAL_ADC_ERROR_NONE);
}

// Sets up ADC n with an analog watchdog on all regular channels, thresholds LT = 0x100, HT = 0x800 and channel 3 as the only rank.
static void abWatchdogRig(ADC_HandleTypeDef &h, unsigned n) {
	h = abHandle(n);
	REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
	REQUIRE(abCfg(h, ADC_CHANNEL_3, 1) == HAL_OK);
	REQUIRE(abWdg(h, ADC_ANALOGWATCHDOG_ALL_REG, ADC_CHANNEL_3, 0x800, 0x100, DISABLE) == HAL_OK);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForEvent AWD returns HAL_OK when the value leaves the window, sets AWD1 and clears the flag") {
	for (unsigned n = 1; n <= 3; ++n) {
		abReset();
		ADC_HandleTypeDef h;
		abWatchdogRig(h, n);
		hostAdcSetAnalog(n, 3, 0x900);
		REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
		REQUIRE(hostAdcConvert(n, 1) == 1u);
		REQUIRE((adcInstance(n)->SR & ADC_SR_AWD) == ADC_SR_AWD);
		INFO("ADC" << n);
		CHECK(HAL_ADC_PollForEvent(&h, ADC_AWD_EVENT, 10) == HAL_OK);
		CHECK((h.State & HAL_ADC_STATE_AWD1) == HAL_ADC_STATE_AWD1);
		CHECK((adcInstance(n)->SR & ADC_SR_AWD) == 0u);
		CHECK(h.ErrorCode == HAL_ADC_ERROR_NONE);
		CHECK(adcCounts.window == 0u);  // the polling path does not call the callback
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForEvent AWD with no event times out with the TIMEOUT state bit and unlocks") {
	ADC_HandleTypeDef h;
	abWatchdogRig(h, 1);
	hostAdcSetAnalog(1, 3, 0x400);  // inside the window
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_ADC_PollForEvent(&h, ADC_AWD_EVENT, 4) == HAL_TIMEOUT);
	CHECK((h.State & HAL_ADC_STATE_TIMEOUT) == HAL_ADC_STATE_TIMEOUT);
	CHECK((h.State & HAL_ADC_STATE_AWD1) == 0u);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForEvent waits about Timeout ticks before it returns HAL_TIMEOUT") {
	const uint32_t timeouts[3] = {0, 3, 20};
	for (uint32_t t : timeouts) {
		abReset();
		ADC_HandleTypeDef h = abHandle(1);
		abInitOk(h);
		const uint32_t before = hostTickPeek();
		REQUIRE(HAL_ADC_PollForEvent(&h, ADC_OVR_EVENT, t) == HAL_TIMEOUT);
		const uint32_t elapsed = hostTickPeek() - before;
		INFO("Timeout " << t << " elapsed " << elapsed);
		CHECK(elapsed >= t);
		CHECK(elapsed <= t + 3u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForEvent AWD with HAL_MAX_DELAY waits until the watchdog fires") {
	ADC_HandleTypeDef h;
	abWatchdogRig(h, 2);
	hostAdcSetAnalog(2, 3, 0x050);  // below LT
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	hostAdcSetPollReads(2, 4);
	CHECK(HAL_ADC_PollForEvent(&h, ADC_AWD_EVENT, HAL_MAX_DELAY) == HAL_OK);
	CHECK((h.State & HAL_ADC_STATE_AWD1) == HAL_ADC_STATE_AWD1);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForEvent OVR returns HAL_OK after an overrun, sets REG_OVR and the OVR error code and clears the flag") {
	// RM0090 13.8.1: with EOCS = 1 a conversion that ends while EOC is still set (DR not read) sets OVR.
	for (unsigned n = 1; n <= 3; ++n) {
		abReset();
		ADC_HandleTypeDef h = abHandle(n);
		h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
		h.Init.ContinuousConvMode = ENABLE;
		abInitOk(h);
		REQUIRE(abCfg(h, ADC_CHANNEL_2, 1) == HAL_OK);
		REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
		REQUIRE(hostAdcConvert(n, 1) == 1u);
		ADC_HandleTypeDef &hr = h;
		hostAdcConvert(n, 1);  // second conversion with EOC still set (DR never read)
		REQUIRE((adcInstance(n)->SR & ADC_SR_OVR) == ADC_SR_OVR);
		INFO("ADC" << n);
		CHECK(HAL_ADC_PollForEvent(&hr, ADC_OVR_EVENT, 10) == HAL_OK);
		CHECK((hr.State & HAL_ADC_STATE_REG_OVR) == HAL_ADC_STATE_REG_OVR);
		CHECK((hr.ErrorCode & HAL_ADC_ERROR_OVR) == HAL_ADC_ERROR_OVR);
		CHECK(HAL_ADC_GetError(&hr) == HAL_ADC_ERROR_OVR);
		CHECK((adcInstance(n)->SR & ADC_SR_OVR) == 0u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 PollForEvent OVR with no overrun times out") {
	ADC_HandleTypeDef h = abHandle(1);
	abInitOk(h);
	CHECK(HAL_ADC_PollForEvent(&h, ADC_OVR_EVENT, 2) == HAL_TIMEOUT);
	CHECK((h.State & HAL_ADC_STATE_TIMEOUT) != 0u);
	CHECK(h.ErrorCode == HAL_ADC_ERROR_NONE);
}

TEST_CASE_FIXTURE(AdcFixture, "AB4 characterization: PollForEvent with EOC takes the overrun path and leaves EOC set (NEEDS REVIEW 338/AB-4)") {
	ADC_HandleTypeDef h;
	abSingle(h, 1, ADC_CHANNEL_0, 9);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	CHECK(HAL_ADC_PollForEvent(&h, ADC_FLAG_EOC, 10) == HAL_OK);
	CHECK((h.State & HAL_ADC_STATE_REG_OVR) != 0u);
	CHECK((h.ErrorCode & HAL_ADC_ERROR_OVR) != 0u);
	CHECK((ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC);
}

// ------------------------------------------------------------------ AB5 AnalogWDGConfig

TEST_CASE_FIXTURE(AdcFixture, "AB5 AnalogWDGConfig sets AWDSGL, JAWDEN and AWDEN for every watchdog mode as in RM0090 Table 66") {
	// Table 66: all injected: JAWDEN; all regular: AWDEN; all regular and injected: AWDEN + JAWDEN; single injected: AWDSGL + JAWDEN;
	// single regular: AWDSGL + AWDEN; single regular or injected: all three; none: AWDEN = JAWDEN = 0.
	const struct { uint32_t mode, bits; } rows[7] = {
	    {ADC_ANALOGWATCHDOG_NONE, 0u},
	    {ADC_ANALOGWATCHDOG_ALL_INJEC, ADC_CR1_JAWDEN},
	    {ADC_ANALOGWATCHDOG_ALL_REG, ADC_CR1_AWDEN},
	    {ADC_ANALOGWATCHDOG_ALL_REGINJEC, ADC_CR1_AWDEN | ADC_CR1_JAWDEN},
	    {ADC_ANALOGWATCHDOG_SINGLE_INJEC, ADC_CR1_AWDSGL | ADC_CR1_JAWDEN},
	    {ADC_ANALOGWATCHDOG_SINGLE_REG, ADC_CR1_AWDSGL | ADC_CR1_AWDEN},
	    {ADC_ANALOGWATCHDOG_SINGLE_REGINJEC, ADC_CR1_AWDSGL | ADC_CR1_AWDEN | ADC_CR1_JAWDEN}};
	for (unsigned n = 1; n <= 3; ++n) {
		for (const auto &r : rows) {
			abReset();
			adcInstance(n)->CR1 = ADC_CR1_AWDSGL | ADC_CR1_AWDEN | ADC_CR1_JAWDEN;  // all three preloaded: the HAL must clear first
			ADC_HandleTypeDef h = abHandle(n);
			h.State = HAL_ADC_STATE_READY;
			REQUIRE(abWdg(h, r.mode, ADC_CHANNEL_0, 0xFFF, 0, DISABLE) == HAL_OK);
			INFO("ADC" << n << " mode bits " << r.bits);
			CHECK(adcInstance(n)->CR1 == r.bits);
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB5 AnalogWDGConfig writes the high and low threshold into HTR and LTR") {
	const uint32_t pairs[4][2] = {{0xFFF, 0}, {0x800, 0x100}, {1, 0}, {0xABC, 0x123}};  // {high, low}
	for (unsigned n = 1; n <= 3; ++n) {
		for (const auto &p : pairs) {
			abReset();
			ADC_HandleTypeDef h = abHandle(n);
			h.State = HAL_ADC_STATE_READY;
			REQUIRE(abWdg(h, ADC_ANALOGWATCHDOG_ALL_REG, ADC_CHANNEL_0, p[0], p[1], DISABLE) == HAL_OK);
			INFO("ADC" << n << " HT " << p[0] << " LT " << p[1]);
			CHECK(adcInstance(n)->HTR == p[0]);
			CHECK(adcInstance(n)->LTR == p[1]);
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB5 AnalogWDGConfig writes the guarded channel into CR1 AWDCH for channels 0 to 18 and replaces the old one") {
	for (uint32_t ch = 0; ch <= 18; ++ch) {
		abReset();
		ADC1->CR1 = ADC_CR1_AWDCH;  // 11111 preloaded
		ADC_HandleTypeDef h = abHandle(1);
		h.State = HAL_ADC_STATE_READY;
		REQUIRE(abWdg(h, ADC_ANALOGWATCHDOG_SINGLE_REG, ch, 0xFFF, 0, DISABLE) == HAL_OK);
		INFO("channel " << ch);
		CHECK((ADC1->CR1 & ADC_CR1_AWDCH) == ch);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB5 AnalogWDGConfig with ITMode ENABLE sets AWDIE (CR1 bit 6) and with DISABLE clears it") {
	ADC_HandleTypeDef h = abHandle(2);
	h.State = HAL_ADC_STATE_READY;
	REQUIRE(abWdg(h, ADC_ANALOGWATCHDOG_ALL_REG, ADC_CHANNEL_0, 0xFFF, 0, ENABLE) == HAL_OK);
	CHECK((ADC2->CR1 & ADC_CR1_AWDIE) == ADC_CR1_AWDIE);
	CHECK((ADC2->CR1 & 0x40u) == 0x40u);
	REQUIRE(abWdg(h, ADC_ANALOGWATCHDOG_ALL_REG, ADC_CHANNEL_0, 0xFFF, 0, DISABLE) == HAL_OK);
	CHECK((ADC2->CR1 & ADC_CR1_AWDIE) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB5 AnalogWDGConfig keeps the other CR1 bits (SCAN, RES, EOCIE, JEOCIE, DISCEN, DISCNUM)") {
	const uint32_t keep = ADC_CR1_SCAN | ADC_CR1_RES | ADC_CR1_EOCIE | ADC_CR1_JEOCIE | ADC_CR1_DISCEN | ADC_CR1_DISCNUM | ADC_CR1_OVRIE;
	ADC1->CR1 = keep;
	ADC_HandleTypeDef h = abHandle(1);
	h.State = HAL_ADC_STATE_READY;
	REQUIRE(abWdg(h, ADC_ANALOGWATCHDOG_SINGLE_REGINJEC, ADC_CHANNEL_9, 0xFFF, 0, ENABLE) == HAL_OK);
	CHECK((ADC1->CR1 & keep) == keep);
	CHECK((ADC1->CR1 & ~keep) == (ADC_CR1_AWDSGL | ADC_CR1_AWDEN | ADC_CR1_JAWDEN | ADC_CR1_AWDIE | 9u));
}

TEST_CASE_FIXTURE(AdcFixture, "AB5 AnalogWDGConfig unlocks the handle and returns HAL_OK") {
	ADC_HandleTypeDef h = abHandle(1);
	h.State = HAL_ADC_STATE_READY;
	CHECK(abWdg(h, ADC_ANALOGWATCHDOG_ALL_REG, ADC_CHANNEL_0, 0xFFF, 0, DISABLE) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(h.State == HAL_ADC_STATE_READY);
}

TEST_CASE_FIXTURE(AdcFixture, "AB5 AnalogWDGConfig of a locked handle returns HAL_BUSY and writes nothing") {
	ADC_HandleTypeDef h = abHandle(1);
	h.Lock = HAL_LOCKED;
	CHECK(abWdg(h, ADC_ANALOGWATCHDOG_ALL_REG, ADC_CHANNEL_5, 0x200, 0x100, ENABLE) == HAL_BUSY);
	CHECK(ADC1->CR1 == 0u);
	CHECK(ADC1->HTR == 0xFFFu);  // reset value
	CHECK(ADC1->LTR == 0u);
	CHECK(h.Lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AB5 watchdog on all regular channels raises AWD for a value above HT or below LT and not at the thresholds") {
	// RM0090 13.3.6: the AWD flag is set when the converted value is higher than HTR or lower than LTR.
	const struct { uint16_t v; bool awd; } rows[6] = {{0x800, false}, {0x801, true}, {0x100, false}, {0x0FF, true}, {0x400, false}, {0xFFF, true}};
	for (const auto &r : rows) {
		abReset();
		ADC_HandleTypeDef h;
		abWatchdogRig(h, 1);
		hostAdcSetAnalog(1, 3, r.v);
		REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
		REQUIRE(hostAdcConvert(1, 1) == 1u);
		INFO("value " << r.v);
		CHECK(((ADC1->SR & ADC_SR_AWD) != 0u) == r.awd);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AB5 watchdog in single regular channel mode ignores the other channels of the scan") {
	// Table 66: AWDSGL = 1 and AWDEN = 1 guards only the channel in AWDCH.
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ScanConvMode = ENABLE;
	h.Init.NbrOfConversion = 2;
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_1, 1) == HAL_OK);
	REQUIRE(abCfg(h, ADC_CHANNEL_2, 2) == HAL_OK);
	REQUIRE(abWdg(h, ADC_ANALOGWATCHDOG_SINGLE_REG, ADC_CHANNEL_2, 0x800, 0x100, DISABLE) == HAL_OK);
	hostAdcSetAnalog(1, 1, 0xF00);  // out of the window but not the guarded channel
	hostAdcSetAnalog(1, 2, 0x400);  // guarded channel inside the window
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	CHECK((ADC1->SR & ADC_SR_AWD) == 0u);
	REQUIRE(HAL_ADC_GetValue(&h) == 0xF00u);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	CHECK((ADC1->SR & ADC_SR_AWD) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB5 watchdog in single regular channel mode raises AWD when the guarded channel leaves the window") {
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ScanConvMode = ENABLE;
	h.Init.NbrOfConversion = 2;
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_1, 1) == HAL_OK);
	REQUIRE(abCfg(h, ADC_CHANNEL_2, 2) == HAL_OK);
	REQUIRE(abWdg(h, ADC_ANALOGWATCHDOG_SINGLE_REG, ADC_CHANNEL_2, 0x800, 0x100, DISABLE) == HAL_OK);
	hostAdcSetAnalog(1, 1, 0x400);
	hostAdcSetAnalog(1, 2, 0x010);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	REQUIRE(HAL_ADC_GetValue(&h) == 0x400u);  // reading DR clears EOC, so the next conversion is no overrun
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	CHECK((ADC1->SR & ADC_SR_AWD) == ADC_SR_AWD);
}

TEST_CASE_FIXTURE(AdcFixture, "AB5 watchdog on all regular channels raises AWD for any channel of the scan") {
	ADC_HandleTypeDef h = abHandle(1);
	h.Init.ScanConvMode = ENABLE;
	h.Init.NbrOfConversion = 2;
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	abInitOk(h);
	REQUIRE(abCfg(h, ADC_CHANNEL_1, 1) == HAL_OK);
	REQUIRE(abCfg(h, ADC_CHANNEL_2, 2) == HAL_OK);
	REQUIRE(abWdg(h, ADC_ANALOGWATCHDOG_ALL_REG, ADC_CHANNEL_2, 0x800, 0x100, DISABLE) == HAL_OK);
	hostAdcSetAnalog(1, 1, 0xF00);
	hostAdcSetAnalog(1, 2, 0x400);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	CHECK((ADC1->SR & ADC_SR_AWD) == ADC_SR_AWD);
}

TEST_CASE_FIXTURE(AdcFixture, "AB5 watchdog mode NONE never raises AWD") {
	ADC_HandleTypeDef h;
	abWatchdogRig(h, 1);
	REQUIRE(abWdg(h, ADC_ANALOGWATCHDOG_NONE, ADC_CHANNEL_3, 0x800, 0x100, DISABLE) == HAL_OK);
	hostAdcSetAnalog(1, 3, 0xF00);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	CHECK((ADC1->SR & ADC_SR_AWD) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB5 injected only watchdog does not guard a regular conversion") {
	ADC_HandleTypeDef h;
	abWatchdogRig(h, 1);
	REQUIRE(abWdg(h, ADC_ANALOGWATCHDOG_ALL_INJEC, ADC_CHANNEL_3, 0x800, 0x100, DISABLE) == HAL_OK);
	hostAdcSetAnalog(1, 3, 0xF00);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	CHECK((ADC1->SR & ADC_SR_AWD) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB5 watchdog interrupt request follows AWDIE: pending with ITMode ENABLE, none with DISABLE") {
	ADC_HandleTypeDef h;
	abWatchdogRig(h, 3);
	hostAdcSetAnalog(3, 3, 0xF00);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(3, 1) == 1u);
	REQUIRE((ADC3->SR & ADC_SR_AWD) == ADC_SR_AWD);
	CHECK(hostAdcIrqPending(3) == 0u);  // AWDIE clear
	REQUIRE(abWdg(h, ADC_ANALOGWATCHDOG_ALL_REG, ADC_CHANNEL_3, 0x800, 0x100, ENABLE) == HAL_OK);
	CHECK(hostAdcIrqPending(3) != 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AB5 a threshold change takes effect for the next conversion") {
	ADC_HandleTypeDef h;
	abWatchdogRig(h, 1);
	hostAdcSetAnalog(1, 3, 0x900);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(abWdg(h, ADC_ANALOGWATCHDOG_ALL_REG, ADC_CHANNEL_3, 0xA00, 0x100, DISABLE) == HAL_OK);  // HT above the value
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	CHECK((ADC1->SR & ADC_SR_AWD) == 0u);
}
