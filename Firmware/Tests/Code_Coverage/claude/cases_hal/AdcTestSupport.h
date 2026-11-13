/*
 * AdcTestSupport.h (P12 T6b): the strong HAL_ADC_* / HAL_ADCEx_* callbacks of the ADC test files, defined once in test_hal_adc_support.cpp.
 * They count their calls and remember the handle, the state, the error code and ADC_SR seen at each call. AdcFixture resets the counters.
 */
#ifndef FFB_TESTS_ADC_TEST_SUPPORT_H
#define FFB_TESTS_ADC_TEST_SUPPORT_H

#include "HalAdc.h"
#include "HalFixture.h"

enum { ADC_CB_CONV = 1, ADC_CB_CONV_HALF, ADC_CB_WINDOW, ADC_CB_ERROR, ADC_CB_INJECTED };

struct AdcCallbackCounts {
	unsigned mspInit, mspDeInit, conv, convHalf, window, error, injected;
	ADC_HandleTypeDef *last;  /* handle of the most recent callback */
	uint32_t stateAtLast;     /* hadc->State when the most recent callback ran */
	uint32_t errorAtLast;     /* hadc->ErrorCode when it ran */
	uint32_t srAtLast;        /* hadc->Instance->SR when it ran (flag clear order) */
	uint32_t srOrder[32];     /* SR at each ordered callback, parallel to order[] */
	unsigned order[32];       /* callback kinds in call order (ADC_CB_*), at most 32 */
	unsigned orderCount;
};

extern AdcCallbackCounts adcCounts;
void adcSupportReset(void);

/* ADC number 1..3 of an instance (0 when not an ADC). */
unsigned adcNumberOf(const ADC_TypeDef *inst);
ADC_TypeDef *adcInstance(unsigned number);

class AdcFixture : public HalFixture {
public:
	AdcFixture() { adcSupportReset(); }
};

#endif /* FFB_TESTS_ADC_TEST_SUPPORT_H */
