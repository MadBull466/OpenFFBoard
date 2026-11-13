// P12 T6b: the strong ADC callbacks shared by the ADC test files (see AdcTestSupport.h).
#include "AdcTestSupport.h"

#include <cstring>

AdcCallbackCounts adcCounts;

void adcSupportReset(void) { std::memset(&adcCounts, 0, sizeof adcCounts); }

ADC_TypeDef *adcInstance(unsigned n) {
	static ADC_TypeDef *const t[3] = {ADC1, ADC2, ADC3};
	return (n >= 1u && n <= 3u) ? t[n - 1u] : nullptr;
}
unsigned adcNumberOf(const ADC_TypeDef *inst) {
	for (unsigned n = 1; n <= 3u; ++n)
		if (adcInstance(n) == inst) return n;
	return 0;
}

extern "C" {
static void note(ADC_HandleTypeDef *h, unsigned kind) {
	adcCounts.last = h;
	adcCounts.stateAtLast = h->State;
	adcCounts.errorAtLast = h->ErrorCode;
	adcCounts.srAtLast = h->Instance->SR;
	if (kind != 0u && adcCounts.orderCount < 32u) {
		adcCounts.srOrder[adcCounts.orderCount] = h->Instance->SR;
		adcCounts.order[adcCounts.orderCount++] = kind;
	}
}
void HAL_ADC_MspInit(ADC_HandleTypeDef *h) { adcCounts.mspInit++; note(h, 0); }
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *h) { adcCounts.mspDeInit++; note(h, 0); }
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *h) { adcCounts.conv++; note(h, ADC_CB_CONV); }
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *h) { adcCounts.convHalf++; note(h, ADC_CB_CONV_HALF); }
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *h) { adcCounts.window++; note(h, ADC_CB_WINDOW); }
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *h) { adcCounts.error++; note(h, ADC_CB_ERROR); }
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *h) { adcCounts.injected++; note(h, ADC_CB_INJECTED); }
}
