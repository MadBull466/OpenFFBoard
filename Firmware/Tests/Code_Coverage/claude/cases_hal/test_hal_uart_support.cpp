// P12 T4b: the strong UART callbacks shared by the UART test files (see UartTestSupport.h).
#include "UartTestSupport.h"

UartCallbackCounts uartCounts;

void uartSupportReset(void) { uartCounts = UartCallbackCounts{}; }

extern "C" {
static void note(UART_HandleTypeDef *h, unsigned kind) {
	uartCounts.last = h;
	uartCounts.gStateAtLast = h->gState;
	uartCounts.rxStateAtLast = h->RxState;
	if (kind != 0u && uartCounts.orderCount < 32u) uartCounts.order[uartCounts.orderCount++] = kind;
}
void HAL_UART_MspInit(UART_HandleTypeDef *h) { uartCounts.mspInit++; note(h, 0); }
void HAL_UART_MspDeInit(UART_HandleTypeDef *h) { uartCounts.mspDeInit++; note(h, 0); }
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h) { uartCounts.txCplt++; note(h, UART_CB_TXCPLT); }
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *h) { uartCounts.txHalf++; note(h, UART_CB_TXHALF); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) { uartCounts.rxCplt++; note(h, UART_CB_RXCPLT); }
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *h) { uartCounts.rxHalf++; note(h, UART_CB_RXHALF); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h) {
	uartCounts.error++;
	uartCounts.errorCodeAtError = h->ErrorCode;
	note(h, UART_CB_ERROR);
}
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *h) { uartCounts.abortCplt++; note(h, UART_CB_ABORT); }
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *h) { uartCounts.abortTxCplt++; note(h, UART_CB_ABORT_TX); }
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *h) { uartCounts.abortRxCplt++; note(h, UART_CB_ABORT_RX); }
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h, uint16_t size) {
	uartCounts.rxEvent++;
	uartCounts.rxEventSize = size;
	note(h, UART_CB_RXEVENT);
}
}
