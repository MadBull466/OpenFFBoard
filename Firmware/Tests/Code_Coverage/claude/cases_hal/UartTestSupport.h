/*
 * UartTestSupport.h (P12 T4b): the strong HAL_UART_* callbacks of the UART test files, defined once in test_hal_uart_support.cpp.
 * They count their calls and remember the handle of the last call. The test files include this header and use UartFixture below
 * (it resets the counters at the start of each case).
 */
#ifndef FFB_TESTS_UART_TEST_SUPPORT_H
#define FFB_TESTS_UART_TEST_SUPPORT_H

#include "HalFixture.h"
#include "HalUart.h"

struct UartCallbackCounts {
	unsigned mspInit, mspDeInit;
	unsigned txCplt, txHalf, rxCplt, rxHalf;
	unsigned error, abortCplt, abortTxCplt, abortRxCplt;
	unsigned rxEvent;
	UART_HandleTypeDef *last;          /* handle of the most recent callback */
	uint16_t rxEventSize;              /* Size given to the most recent HAL_UARTEx_RxEventCallback */
	uint32_t errorCodeAtError;         /* huart->ErrorCode seen by the most recent ErrorCallback */
	HAL_UART_StateTypeDef gStateAtLast, rxStateAtLast; /* states seen by the most recent callback */
	unsigned order[32];                /* callback kinds in call order (UART_CB_*), at most 32 */
	unsigned orderCount;
};

enum { UART_CB_TXCPLT = 1, UART_CB_TXHALF, UART_CB_RXCPLT, UART_CB_RXHALF, UART_CB_ERROR, UART_CB_ABORT, UART_CB_ABORT_TX, UART_CB_ABORT_RX, UART_CB_RXEVENT };

extern UartCallbackCounts uartCounts;
void uartSupportReset(void);

class UartFixture : public HalFixture {
public:
	UartFixture() { uartSupportReset(); }
};

#endif /* FFB_TESTS_UART_TEST_SUPPORT_H */
