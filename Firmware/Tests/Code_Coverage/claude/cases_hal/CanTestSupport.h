/*
 * CanTestSupport.h (P12 T5): the strong HAL_CAN_* callbacks of the CAN test files, defined once in test_hal_can_support.cpp.
 * They count their calls and remember the handle and the state / error code seen at the last call. CanFixture resets the counters.
 */
#ifndef FFB_TESTS_CAN_TEST_SUPPORT_H
#define FFB_TESTS_CAN_TEST_SUPPORT_H

#include "HalCan.h"
#include "HalFixture.h"

struct CanCallbackCounts {
	unsigned mspInit, mspDeInit;
	unsigned txComplete[3], txAbort[3];
	unsigned rx0Pending, rx0Full, rx1Pending, rx1Full, sleep, wakeUp, error;
	CAN_HandleTypeDef *last;       /* handle of the most recent callback */
	uint32_t errorCodeAtError;     /* hcan->ErrorCode seen by the most recent ErrorCallback */
	HAL_CAN_StateTypeDef stateAtLast;
	unsigned order[32];            /* callback kinds in call order (CAN_CB_*), at most 32 */
	unsigned orderCount;
};

enum { CAN_CB_TX0 = 1, CAN_CB_TX1, CAN_CB_TX2, CAN_CB_ABORT0, CAN_CB_ABORT1, CAN_CB_ABORT2, CAN_CB_RX0_PENDING, CAN_CB_RX0_FULL, CAN_CB_RX1_PENDING, CAN_CB_RX1_FULL, CAN_CB_SLEEP, CAN_CB_WAKEUP, CAN_CB_ERROR };

extern CanCallbackCounts canCounts;
void canSupportReset(void);

/* Index (0 = CAN1, 1 = CAN2) of an instance. */
unsigned canIndexOf(const CAN_TypeDef *inst);

class CanFixture : public HalFixture {
public:
	CanFixture() { canSupportReset(); }
};

#endif /* FFB_TESTS_CAN_TEST_SUPPORT_H */
