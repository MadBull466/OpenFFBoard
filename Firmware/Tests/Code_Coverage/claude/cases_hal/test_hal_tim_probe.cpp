// P12 T6a lead probe of the timer model through the real HAL (kept as the model's own sanity cases; the writers' files cover the HAL).
#include "TimTestSupport.h"

TEST_CASE_FIXTURE(TimFixture, "probe: base timer start, update event and interrupt handler") {
	TIM_HandleTypeDef h{};
	h.Instance = TIM3;
	h.Init.Prescaler = 83;
	h.Init.CounterMode = TIM_COUNTERMODE_UP;
	h.Init.Period = 999;
	h.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	h.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	CHECK(timCounts.mspInit[0] == 1u);
	CHECK(TIM3->ARR == 999u);
	CHECK(TIM3->PSC == 83u);
	REQUIRE(HAL_TIM_Base_Start_IT(&h) == HAL_OK);
	CHECK((TIM3->CR1 & 1u) == 1u);
	CHECK((TIM3->DIER & 1u) == 1u);
	hostTimAdvance(3, 1000);
	CHECK(hostTimIrqPending(3) != 0u);
	HAL_TIM_IRQHandler(&h);
	CHECK(timCounts.period == 1u);
	CHECK((TIM3->SR & 1u) == 0u);
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "probe: PWM channel on TIM1 and complementary output") {
	TIM_HandleTypeDef h{};
	h.Instance = TIM1;
	h.Init.Period = 4199;
	REQUIRE(HAL_TIM_PWM_Init(&h) == HAL_OK);
	TIM_OC_InitTypeDef oc{};
	oc.OCMode = TIM_OCMODE_PWM1;
	oc.Pulse = 2100;
	REQUIRE(HAL_TIM_PWM_ConfigChannel(&h, &oc, TIM_CHANNEL_1) == HAL_OK);
	CHECK(TIM1->CCR1 == 2100u);
	REQUIRE(HAL_TIM_PWM_Start(&h, TIM_CHANNEL_1) == HAL_OK);
	CHECK((TIM1->CCER & 1u) == 1u);
	CHECK((TIM1->BDTR & (1u << 15)) != 0u); /* MOE set by the start of an advanced timer */
	REQUIRE(HAL_TIMEx_PWMN_Start(&h, TIM_CHANNEL_1) == HAL_OK);
	CHECK((TIM1->CCER & 4u) == 4u);
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "probe: input capture on TIM2 and a basic timer register fault") {
	TIM_HandleTypeDef h{};
	h.Instance = TIM2;
	REQUIRE(HAL_TIM_IC_Init(&h) == HAL_OK);
	TIM_IC_InitTypeDef ic{};
	ic.ICPolarity = TIM_ICPOLARITY_RISING;
	ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
	ic.ICPrescaler = TIM_ICPSC_DIV1;
	REQUIRE(HAL_TIM_IC_ConfigChannel(&h, &ic, TIM_CHANNEL_1) == HAL_OK);
	REQUIRE(HAL_TIM_IC_Start_IT(&h, TIM_CHANNEL_1) == HAL_OK);
	CHECK(hostTimCapture(2, 1, 0x12345u) == 1);
	HAL_TIM_IRQHandler(&h);
	CHECK(timCounts.icCapture == 1u);
	CHECK(HAL_TIM_ReadCapturedValue(&h, TIM_CHANNEL_1) == 0x12345u);
	CHECK(HalFixture::faults() == 0u);
}

// UM1725 (HAL TIM generic driver): HAL_TIM_DMABurstState "Return actual state of a DMA burst operation".
TEST_CASE_FIXTURE(TimFixture, "TC4 HAL_TIM_DMABurstState reports READY after Init and the state the handle holds") {
	TIM_HandleTypeDef h{};
	h.Instance = TIM1;
	h.Init.Period = 99;
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	CHECK(HAL_TIM_DMABurstState(&h) == HAL_DMA_BURST_STATE_READY);
	h.DMABurstState = HAL_DMA_BURST_STATE_BUSY;
	CHECK(HAL_TIM_DMABurstState(&h) == HAL_DMA_BURST_STATE_BUSY);
	CHECK(HalFixture::faults() == 0u);
}
