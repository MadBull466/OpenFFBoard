/*
 * TimTestSupport.h (P12 T6a): the strong HAL_TIM_* / HAL_TIMEx_* callbacks of the timer test files, defined once in test_hal_tim_support.cpp.
 * They count their calls and remember the handle, the active channel and the state seen at the last call. TimFixture resets the counters.
 */
#ifndef FFB_TESTS_TIM_TEST_SUPPORT_H
#define FFB_TESTS_TIM_TEST_SUPPORT_H

#include "HalFixture.h"
#include "HalTim.h"

enum {
	TIM_CB_PERIOD = 1, TIM_CB_PERIOD_HALF, TIM_CB_OC_DELAY, TIM_CB_IC_CAPTURE, TIM_CB_IC_CAPTURE_HALF, TIM_CB_PWM_PULSE, TIM_CB_PWM_PULSE_HALF,
	TIM_CB_TRIGGER, TIM_CB_TRIGGER_HALF, TIM_CB_ERROR, TIM_CB_COMMUT, TIM_CB_COMMUT_HALF, TIM_CB_BREAK
};

struct TimCallbackCounts {
	unsigned mspInit[8], mspDeInit[8]; /* index: 0 base, 1 oc, 2 pwm, 3 ic, 4 one pulse, 5 encoder, 6 hall sensor */
	unsigned period, periodHalf, ocDelay, icCapture, icCaptureHalf, pwmPulse, pwmPulseHalf, trigger, triggerHalf, error, commut, commutHalf, breakCb;
	TIM_HandleTypeDef *last;       /* handle of the most recent callback */
	HAL_TIM_ActiveChannel channelAtLast;
	HAL_TIM_StateTypeDef stateAtLast;
	uint32_t srAtLast;            /* TIMx->SR when the most recent callback ran (IRQ flag clear order) */
	uint32_t srOrder[32];          /* SR at each ordered callback, parallel to order[] */
	unsigned order[32];            /* callback kinds in call order (TIM_CB_*), at most 32 */
	unsigned orderCount;
};

extern TimCallbackCounts timCounts;
void timSupportReset(void);

/* Timer number 1..14 of an instance. */
unsigned timNumberOf(const TIM_TypeDef *inst);
TIM_TypeDef *timInstance(unsigned number);

class TimFixture : public HalFixture {
public:
	TimFixture() { timSupportReset(); }
};

#endif /* FFB_TESTS_TIM_TEST_SUPPORT_H */
