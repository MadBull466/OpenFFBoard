// P12 T5: the strong CAN callbacks shared by the CAN test files (see CanTestSupport.h).
#include "CanTestSupport.h"

#include <cstring>

CanCallbackCounts canCounts;

void canSupportReset(void) { std::memset(&canCounts, 0, sizeof canCounts); }

unsigned canIndexOf(const CAN_TypeDef *inst) { return inst == CAN1 ? 0u : 1u; }

extern "C" {
static void note(CAN_HandleTypeDef *h, unsigned kind) {
	canCounts.last = h;
	canCounts.stateAtLast = h->State;
	if (kind != 0u && canCounts.orderCount < 32u) canCounts.order[canCounts.orderCount++] = kind;
}
void HAL_CAN_MspInit(CAN_HandleTypeDef *h) { canCounts.mspInit++; note(h, 0); }
void HAL_CAN_MspDeInit(CAN_HandleTypeDef *h) { canCounts.mspDeInit++; note(h, 0); }
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *h) { canCounts.txComplete[0]++; note(h, CAN_CB_TX0); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *h) { canCounts.txComplete[1]++; note(h, CAN_CB_TX1); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *h) { canCounts.txComplete[2]++; note(h, CAN_CB_TX2); }
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *h) { canCounts.txAbort[0]++; note(h, CAN_CB_ABORT0); }
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *h) { canCounts.txAbort[1]++; note(h, CAN_CB_ABORT1); }
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *h) { canCounts.txAbort[2]++; note(h, CAN_CB_ABORT2); }
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h) { canCounts.rx0Pending++; note(h, CAN_CB_RX0_PENDING); }
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *h) { canCounts.rx0Full++; note(h, CAN_CB_RX0_FULL); }
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *h) { canCounts.rx1Pending++; note(h, CAN_CB_RX1_PENDING); }
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *h) { canCounts.rx1Full++; note(h, CAN_CB_RX1_FULL); }
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *h) { canCounts.sleep++; note(h, CAN_CB_SLEEP); }
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *h) { canCounts.wakeUp++; note(h, CAN_CB_WAKEUP); }
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *h) {
	canCounts.error++;
	canCounts.errorCodeAtError = h->ErrorCode;
	note(h, CAN_CB_ERROR);
}
}
