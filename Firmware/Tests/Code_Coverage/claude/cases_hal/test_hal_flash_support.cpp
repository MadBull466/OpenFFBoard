// P12 T7: the strong flash callbacks shared by the flash test files (see FlashTestSupport.h).
#include "FlashTestSupport.h"

#include <cstring>

FlashCallbackCounts flashCounts;

void flashSupportReset(void) { std::memset(&flashCounts, 0, sizeof flashCounts); }

extern "C" {
static void note(unsigned kind, uint32_t value) {
	flashCounts.lastValue = value;
	if (flashCounts.orderCount < 32u) {
		flashCounts.valueOrder[flashCounts.orderCount] = value;
		flashCounts.srOrder[flashCounts.orderCount] = FLASH->SR;
		flashCounts.errorCodeOrder[flashCounts.orderCount] = HAL_FLASH_GetError();
		flashCounts.order[flashCounts.orderCount++] = kind;
	}
}
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue) {
	flashCounts.eop++;
	note(FLASH_CB_EOP, ReturnValue);
}
void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue) {
	flashCounts.error++;
	note(FLASH_CB_ERROR, ReturnValue);
}
}
