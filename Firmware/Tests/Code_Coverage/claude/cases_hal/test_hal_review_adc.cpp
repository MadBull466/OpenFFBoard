// P12 review witnesses, ADC (Codex review of P12, P12C-T6B-002).
#include "AdcTestSupport.h"

TEST_CASE_FIXTURE(AdcFixture, "RV15 sixteen regular ranks convert their own channels across the SQR3 / SQR2 / SQR1 boundaries (P12C-T6B-002)") {
	// RM0090 13.3.8 / 13.13.9-11: ranks 1-6 live in SQR3, 7-12 in SQR2, 13-16 in SQR1. The earlier cases looked at register bits only and
	// converted at most three ranks, so a wrong channel selection above rank 6 passed.
	ADC_HandleTypeDef h{};
	h.Instance = ADC1;
	h.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	h.Init.ScanConvMode = ENABLE;
	h.Init.NbrOfConversion = 16;
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
	for (unsigned r = 1; r <= 16; ++r) {
		ADC_ChannelConfTypeDef c{};
		c.Rank = r;
		c.Channel = r;
		REQUIRE(HAL_ADC_ConfigChannel(&h, &c) == HAL_OK);
		hostAdcSetAnalog(1, r, 100 + r);  // channel r reads 100 + r
	}
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	for (unsigned r = 1; r <= 16; ++r) {
		CAPTURE(r);
		REQUIRE(hostAdcConvert(1, 1) == 1u);
		CHECK(HAL_ADC_GetValue(&h) == 100 + r);
	}
	CHECK(hostAdcRegularRunning(1) == 0);
	CHECK(hostAdcConversions(1) == 16u);
}
