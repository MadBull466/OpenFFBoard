// P12 review witnesses, UART (Codex review of P12, P12C-T4B-002).
#include "UartTestSupport.h"

namespace {
UART_HandleTypeDef mkHandle() {
	UART_HandleTypeDef h{};
	h.Instance = USART1;
	h.Init.BaudRate = 115200;
	h.Init.WordLength = UART_WORDLENGTH_8B;
	h.Init.StopBits = UART_STOPBITS_1;
	h.Init.Parity = UART_PARITY_NONE;
	h.Init.Mode = UART_MODE_TX_RX;
	h.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	h.Init.OverSampling = UART_OVERSAMPLING_16;
	return h;
}
}  // namespace

TEST_CASE_FIXTURE(UartFixture, "RV8 an idle line in address mark mute mode does not set IDLE, even after a frame was received (P12C-T4B-002)") {
	// RM0090 30.3.6 "In mute mode: None of the reception status bits can be set."
	UART_HandleTypeDef h = mkHandle();
	REQUIRE(HAL_MultiProcessor_Init(&h, 1, UART_WAKEUPMETHOD_ADDRESSMARK) == HAL_OK);
	hostUartRxNow(0, 0x81, 0);
	REQUIRE((h.Instance->SR & USART_SR_RXNE) != 0);
	volatile uint32_t data = h.Instance->DR;
	(void)data;
	REQUIRE(HAL_MultiProcessor_EnterMuteMode(&h) == HAL_OK);
	REQUIRE((h.Instance->CR1 & USART_CR1_RWU) != 0);
	hostUartIdleLine(0);
	CHECK((h.Instance->SR & USART_SR_IDLE) == 0);
	CHECK((h.Instance->CR1 & USART_CR1_RWU) != 0);  // address mark wake-up needs a matching address, an idle line does not wake it
}

TEST_CASE_FIXTURE(UartFixture, "RV8 an idle line after a received frame sets IDLE in the active receiver (control, P12C-T4B-002)") {
	UART_HandleTypeDef h = mkHandle();
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	hostUartRxNow(0, 0x11, 0);
	volatile uint32_t data = h.Instance->DR;
	(void)data;
	hostUartIdleLine(0);
	CHECK((h.Instance->SR & USART_SR_IDLE) != 0);
}
