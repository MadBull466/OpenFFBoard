/*
 * FlashTestSupport.h (P12 T7): the strong HAL_FLASH_* callbacks of the flash and watchdog test files, defined once in test_hal_flash_support.cpp.
 * They count their calls and remember the return value, FLASH->SR at the call and the order. FlashFixture resets the counters and unlocks nothing.
 */
#ifndef FFB_TESTS_FLASH_TEST_SUPPORT_H
#define FFB_TESTS_FLASH_TEST_SUPPORT_H

#include "HalFixture.h"
#include "HalFlash.h"
#include "HalIwdg.h"

enum { FLASH_CB_EOP = 1, FLASH_CB_ERROR };

struct FlashCallbackCounts {
	unsigned eop, error;
	uint32_t lastValue;        /* ReturnValue of the most recent callback */
	uint32_t valueOrder[32];   /* ReturnValue of each callback in order */
	unsigned order[32];        /* callback kinds in call order (FLASH_CB_*) */
	uint32_t srOrder[32];      /* FLASH->SR when each callback ran */
	uint32_t errorCodeOrder[32]; /* HAL_FLASH_GetError() when each callback ran */
	unsigned orderCount;
};

extern FlashCallbackCounts flashCounts;
void flashSupportReset(void);

class FlashFixture : public HalFixture {
public:
	FlashFixture() { flashSupportReset(); }
};

#endif /* FFB_TESTS_FLASH_TEST_SUPPORT_H */
