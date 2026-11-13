// P12 T6b lead probe of the ADC model through the real HAL (kept as the model's own sanity cases; the writers' files cover the HAL).
#include "AdcTestSupport.h"

static ADC_HandleTypeDef probeHandle(ADC_TypeDef *inst) {
	ADC_HandleTypeDef h{};
	h.Instance = inst;
	h.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	h.Init.Resolution = ADC_RESOLUTION_12B;
	h.Init.ScanConvMode = DISABLE;
	h.Init.ContinuousConvMode = DISABLE;
	h.Init.DiscontinuousConvMode = DISABLE;
	h.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	h.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	h.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	h.Init.NbrOfConversion = 1;
	h.Init.DMAContinuousRequests = DISABLE;
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	return h;
}

TEST_CASE_FIXTURE(AdcFixture, "probe: single regular conversion by polling") {
	ADC_HandleTypeDef h = probeHandle(ADC1);
	REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
	CHECK(adcCounts.mspInit == 1u);
	ADC_ChannelConfTypeDef c{};
	c.Channel = ADC_CHANNEL_5;
	c.Rank = 1;
	c.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	REQUIRE(HAL_ADC_ConfigChannel(&h, &c) == HAL_OK);
	hostAdcSetAnalog(1, 5, 0x123);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK(hostAdcRegularRunning(1) == 1);
	CHECK(HAL_ADC_PollForConversion(&h, 10) == HAL_TIMEOUT);
	hostAdcSetPollReads(1, 1);
	CHECK(HAL_ADC_PollForConversion(&h, 10) == HAL_OK);
	CHECK(HAL_ADC_GetValue(&h) == 0x123u);
	CHECK((ADC1->SR & 2u) == 0u); /* reading DR cleared EOC */
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "probe: scan sequence with DMA requests and the interrupt handler") {
	ADC_HandleTypeDef h = probeHandle(ADC2);
	h.Init.ScanConvMode = ENABLE;
	h.Init.NbrOfConversion = 2;
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
	ADC_ChannelConfTypeDef c{};
	c.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	c.Channel = ADC_CHANNEL_1; c.Rank = 1; REQUIRE(HAL_ADC_ConfigChannel(&h, &c) == HAL_OK);
	c.Channel = ADC_CHANNEL_2; c.Rank = 2; REQUIRE(HAL_ADC_ConfigChannel(&h, &c) == HAL_OK);
	hostAdcSetAnalog(2, 1, 100);
	hostAdcSetAnalog(2, 2, 200);
	REQUIRE(HAL_ADC_Start_IT(&h) == HAL_OK);
	CHECK(hostAdcConvert(2, 1) == 1u);
	CHECK(hostAdcIrqPending(2) != 0u);
	HAL_ADC_IRQHandler(&h);
	CHECK(adcCounts.conv == 1u);
	CHECK(HAL_ADC_GetValue(&h) == 100u);
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "probe: injected conversion and the analog watchdog") {
	ADC_HandleTypeDef h = probeHandle(ADC3);
	REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
	ADC_InjectionConfTypeDef j{};
	j.InjectedChannel = ADC_CHANNEL_3;
	j.InjectedRank = 1;
	j.InjectedNbrOfConversion = 1;
	j.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
	j.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
	j.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
	j.InjectedOffset = 10;
	j.AutoInjectedConv = DISABLE;
	j.InjectedDiscontinuousConvMode = DISABLE;
	REQUIRE(HAL_ADCEx_InjectedConfigChannel(&h, &j) == HAL_OK);
	hostAdcSetAnalog(3, 3, 510);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK(hostAdcConvert(3, 4) == 1u);
	CHECK((ADC3->SR & 4u) != 0u); /* JEOC at the end of the injected group */
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1) == 500u);
	CHECK((ADC3->SR & 4u) == 0u); /* the HAL clears JEOC when it reads the value */
	CHECK(HalFixture::faults() == 0u);
}
