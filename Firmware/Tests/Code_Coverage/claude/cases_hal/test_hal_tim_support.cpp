// P12 T6a: the strong timer callbacks shared by the timer test files (see TimTestSupport.h).
#include "TimTestSupport.h"

#include <cstring>

TimCallbackCounts timCounts;

void timSupportReset(void) { std::memset(&timCounts, 0, sizeof timCounts); }

TIM_TypeDef *timInstance(unsigned n) {
	static TIM_TypeDef *const t[14] = {TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, TIM9, TIM10, TIM11, TIM12, TIM13, TIM14};
	return (n >= 1u && n <= 14u) ? t[n - 1u] : nullptr;
}
unsigned timNumberOf(const TIM_TypeDef *inst) {
	for (unsigned n = 1; n <= 14u; ++n)
		if (timInstance(n) == inst) return n;
	return 0;
}

extern "C" {
static void note(TIM_HandleTypeDef *h, unsigned kind) {
	timCounts.last = h;
	timCounts.channelAtLast = h->Channel;
	timCounts.stateAtLast = h->State;
	timCounts.srAtLast = h->Instance->SR;
	if (kind != 0u && timCounts.orderCount < 32u) {
		timCounts.srOrder[timCounts.orderCount] = h->Instance->SR;
		timCounts.order[timCounts.orderCount++] = kind;
	}
}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *h) { timCounts.mspInit[0]++; note(h, 0); }
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *h) { timCounts.mspDeInit[0]++; note(h, 0); }
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *h) { timCounts.mspInit[1]++; note(h, 0); }
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *h) { timCounts.mspDeInit[1]++; note(h, 0); }
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *h) { timCounts.mspInit[2]++; note(h, 0); }
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *h) { timCounts.mspDeInit[2]++; note(h, 0); }
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *h) { timCounts.mspInit[3]++; note(h, 0); }
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *h) { timCounts.mspDeInit[3]++; note(h, 0); }
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *h) { timCounts.mspInit[4]++; note(h, 0); }
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *h) { timCounts.mspDeInit[4]++; note(h, 0); }
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *h) { timCounts.mspInit[5]++; note(h, 0); }
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *h) { timCounts.mspDeInit[5]++; note(h, 0); }
void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef *h) { timCounts.mspInit[6]++; note(h, 0); }
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef *h) { timCounts.mspDeInit[6]++; note(h, 0); }
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *h) { timCounts.period++; note(h, TIM_CB_PERIOD); }
void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *h) { timCounts.periodHalf++; note(h, TIM_CB_PERIOD_HALF); }
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *h) { timCounts.ocDelay++; note(h, TIM_CB_OC_DELAY); }
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *h) { timCounts.icCapture++; note(h, TIM_CB_IC_CAPTURE); }
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *h) { timCounts.icCaptureHalf++; note(h, TIM_CB_IC_CAPTURE_HALF); }
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *h) { timCounts.pwmPulse++; note(h, TIM_CB_PWM_PULSE); }
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *h) { timCounts.pwmPulseHalf++; note(h, TIM_CB_PWM_PULSE_HALF); }
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *h) { timCounts.trigger++; note(h, TIM_CB_TRIGGER); }
void HAL_TIM_TriggerHalfCpltCallback(TIM_HandleTypeDef *h) { timCounts.triggerHalf++; note(h, TIM_CB_TRIGGER_HALF); }
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *h) { timCounts.error++; note(h, TIM_CB_ERROR); }
void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *h) { timCounts.commut++; note(h, TIM_CB_COMMUT); }
void HAL_TIMEx_CommutHalfCpltCallback(TIM_HandleTypeDef *h) { timCounts.commutHalf++; note(h, TIM_CB_COMMUT_HALF); }
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *h) { timCounts.breakCb++; note(h, TIM_CB_BREAK); }
}
