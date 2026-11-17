// P12 T7: the strong flash callbacks shared by the flash test files (see FlashTestSupport.h).
#include "FlashTestSupport.h"

#include <cstring>

FlashCallbackCounts flashCounts;

extern "C" {
extern FLASH_ProcessTypeDef pFlash; /* HAL software state (stm32f4xx_hal_flash.c), also reset here: a case that leaves pFlash.Lock set must not break the next one */
}

/* 1 if a write that sets FLASH_ACR.ICRST was already in the register log when a callback ran (Codex review P12C-T7-002: the HAL must reset the
 * caches before it calls the end-of-operation callback; checked at the callback, not after the IRQ handler returned). The case clears the log first. */
unsigned flashCacheResetSeenAtCallback = 0;
unsigned flashObserveCacheReset = 0; /* set by the one case that wants it: scanning the log in every callback would make the other cases slow */

void flashSupportReset(void) {
	std::memset(&flashCounts, 0, sizeof flashCounts);
	std::memset(const_cast<FLASH_ProcessTypeDef *>(&pFlash), 0, sizeof pFlash);
	flashCacheResetSeenAtCallback = 0;
	flashObserveCacheReset = 0;
}

extern "C" {
static void note(unsigned kind, uint32_t value) {
	if (flashObserveCacheReset) {
		for (size_t i = 0; i < hostRegsLogCount(); ++i) {
			const HostRegAccess *e = hostRegsLogAt(i);
			if (e->write && e->addr == reinterpret_cast<uintptr_t>(&FLASH->ACR) && (e->written & 0x800u) != 0u) flashCacheResetSeenAtCallback = 1;
		}
	}
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
