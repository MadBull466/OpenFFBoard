// P12 review witnesses, timers (Codex review of P12, P12C-T6A-001 and P12C-T6A-002).
#include "TimTestSupport.h"

TEST_CASE_FIXTURE(TimFixture, "RV13 reading CCRx of an input channel clears only its CCxIF and keeps CCxOF and the other flags (P12C-T6A-001)") {
	// RM0090 TIMx_SR: CCxIF "is cleared by software or by reading the captured data stored in the TIMx_CCRx register"; CCxOF only by writing 0.
	TIM_HandleTypeDef h{};
	h.Instance = TIM3;
	REQUIRE(HAL_TIM_IC_Init(&h) == HAL_OK);
	TIM_IC_InitTypeDef c{};
	c.ICSelection = TIM_ICSELECTION_DIRECTTI;
	REQUIRE(HAL_TIM_IC_ConfigChannel(&h, &c, TIM_CHANNEL_1) == HAL_OK);
	REQUIRE(HAL_TIM_IC_Start(&h, TIM_CHANNEL_1) == HAL_OK);
	REQUIRE(hostTimCapture(3, 1, 10) == 1);
	hostTimRaiseFlags(3, 1u | 4u);  // UIF (bit 0) and CC2IF (bit 2): channel 2 is not an input here, other flags must survive the CCR1 read
	REQUIRE((TIM3->SR & 2u) == 2u);  // CC1IF set by the capture
	CHECK(HAL_TIM_ReadCapturedValue(&h, TIM_CHANNEL_1) == 10u);
	CHECK(TIM3->SR == (1u | 4u));  // CC1IF (bit 1) is gone, UIF and CC2IF stay
	REQUIRE(hostTimCapture(3, 1, 20) == 1);
	CHECK((TIM3->SR & 0x200u) == 0u);  // the read in between: no overcapture
	REQUIRE(hostTimCapture(3, 1, 30) == 1);
	CHECK((TIM3->SR & 0x200u) != 0u);  // no read between the two captures: CC1OF
	CHECK(HAL_TIM_ReadCapturedValue(&h, TIM_CHANNEL_1) == 30u);
	CHECK(TIM3->SR == (1u | 4u | 0x200u));  // CC1OF survives the read
	TIM3->CCR2 = 50u;                  // an output channel: reading it does not clear CC2IF
	CHECK(HAL_TIM_ReadCapturedValue(&h, TIM_CHANNEL_2) == 50u);
	CHECK((TIM3->SR & 4u) != 0u);
}

TEST_CASE_FIXTURE(TimFixture, "RV14 with LOCK level 3 the output compare mode bits of CCMR cannot be changed on TIM1 (P12C-T6A-002)") {
	// RM0090 TIMx_BDTR LOCK: level 3 adds OCxM and OCxPE to the bits protected by levels 1 and 2.
	TIM_HandleTypeDef h{};
	h.Instance = TIM1;
	h.Init.Period = 99;
	REQUIRE(HAL_TIM_PWM_Init(&h) == HAL_OK);
	TIM_OC_InitTypeDef c{};
	c.OCMode = TIM_OCMODE_PWM1;
	c.Pulse = 20;
	REQUIRE(HAL_TIM_PWM_ConfigChannel(&h, &c, TIM_CHANNEL_1) == HAL_OK);
	TIM_BreakDeadTimeConfigTypeDef bd{};
	bd.LockLevel = TIM_LOCKLEVEL_3;
	REQUIRE(HAL_TIMEx_ConfigBreakDeadTime(&h, &bd) == HAL_OK);
	CHECK((TIM1->CCMR1 & 0x78u) == 0x68u);  // OC1M = PWM mode 1 (110), OC1PE
	TIM1->CCMR1 = 0;
	CHECK((TIM1->CCMR1 & 0x78u) == 0x68u);  // unchanged: locked
}
