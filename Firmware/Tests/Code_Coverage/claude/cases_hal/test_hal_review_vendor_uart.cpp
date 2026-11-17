// V128 (vendor, ST HAL UART): an Rx DMA error abort clears the Tx counter of an unrelated interrupt driven transmission.
// Found by the Codex review of P12 (P12C-T4B-001), re-checked in the source and reproduced here.
#include "UartTestSupport.h"
#include "HalDma.h"
#include "TestHelpers.h"

TEST_CASE_FIXTURE(UartFixture, "RV20 an Rx DMA error does not clear the byte counter of a running interrupt driven transmission" *
				  KNOWN_ISSUE("V128: UART_DMAAbortOnError (stm32f4xx_hal_uart.c:3359) sets RxXferCount AND TxXferCount to 0 although only the Rx stream was aborted; with HAL_UART_Transmit_IT running (BUSY_TX, TXEIE) the next Tx interrupt does --TxXferCount from 0 to 65535 and keeps sending. UM1725 says nothing that a receive error stops a transmission; the state of the Tx process (gState BUSY_TX, TXEIE) is left unchanged by the HAL, only its counter is cleared")) {
	UART_HandleTypeDef h{};
	h.Instance = USART2;
	h.Init.BaudRate = 115200;
	h.Init.WordLength = UART_WORDLENGTH_8B;
	h.Init.StopBits = UART_STOPBITS_1;
	h.Init.Mode = UART_MODE_TX_RX;
	h.Init.OverSampling = UART_OVERSAMPLING_16;
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	DMA_HandleTypeDef d{};
	d.Instance = DMA1_Stream5;
	d.Init.Channel = DMA_CHANNEL_4;
	d.Init.Direction = DMA_PERIPH_TO_MEMORY;
	d.Init.MemInc = DMA_MINC_ENABLE;
	d.Init.Mode = DMA_NORMAL;
	REQUIRE(HAL_DMA_Init(&d) == HAL_OK);
	h.hdmarx = &d;
	d.Parent = &h;
	uint8_t tx[3] = {0x31, 0x32, 0x33}, rx[4]{};
	REQUIRE(HAL_UART_Transmit_IT(&h, tx, 3) == HAL_OK);
	REQUIRE(HAL_UART_Receive_DMA(&h, rx, 4) == HAL_OK);
	hostUartRxNow(1, 0x41, 0);
	hostUartRxNow(1, 0x42, 0);  // the second frame overruns the first one: ORE
	HAL_UART_IRQHandler(&h);
	REQUIRE(uartCounts.error == 0);
	HAL_DMA_IRQHandler(&d);  // the Rx DMA abort completes: UART_DMAAbortOnError, then the error callback
	REQUIRE(uartCounts.error == 1);
	REQUIRE(uartCounts.errorCodeAtError == HAL_UART_ERROR_ORE);
	REQUIRE(h.gState == HAL_UART_STATE_BUSY_TX);  // the transmission goes on, as far as the HAL state says
	REQUIRE((h.Instance->CR1 & USART_CR1_TXEIE) != 0);
	CHECK(h.TxXferCount == 3);  // the defect assertion: still three bytes to send
	// One more Tx interrupt: the first byte is still read in bounds (the pointer was not touched); the count then wraps below 0.
	HAL_UART_IRQHandler(&h);
	CHECK(h.TxXferCount == 2);
	CHECK(hostUartTxAt(1, 0) == 0x31);
}
