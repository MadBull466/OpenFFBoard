/*
 * P12 T4b group UQ: UART interrupt mode (HAL_UART_Transmit_IT / Receive_IT / HAL_UARTEx_ReceiveToIdle_IT, HAL_UART_IRQHandler
 * with the static UART_Transmit_IT / UART_EndTransmit_IT / UART_Receive_IT behind it) of stm32f4xx_hal_uart.c against the UART
 * register model (fakes/stm32_hal/HalUart.h). Requirements UQ1-UQ6 of Tests/Code_Coverage/claude/charters/hal/P12_T4B_REQUIREMENTS.md.
 *
 * Oracles: RM0090 Rev 22 chapter 30 (30.3.2 transmitter, 30.3.3 receiver, 30.3.7 parity, 30.4 Table 147: TXE/TXEIE, TC/TCIE,
 * RXNE and ORE/RXNEIE, IDLE/IDLEIE, PE/PEIE, NF or ORE or FE/EIE; 30.6.1 flags and their clear sequences) and UM1725 Rev 8
 * chapter 70 (state names, error code names, callbacks run at the end of the process).
 *
 * Interrupts are not delivered by the model: the tests call HAL_UART_IRQHandler themselves and advance the HAL tick between
 * calls (a frame of the model ends after its tick based frame time). Hand derived line frames (RM0090 30.3.7, parity bit
 * replaces the MSB of the word, even parity: total number of ones even):
 *   8 bit word:  data 0x41 (2 ones in 7 bits): even 0x41, odd 0xC1;  data 0x07 (3 ones): even 0x87, odd 0x07.
 *   9 bit word:  data 0xFF (8 ones): even 0x0FF, odd 0x1FF;          data 0x01 (1 one): even 0x101, odd 0x001.
 *
 * NEEDS REVIEW proposals (characterization cases; the lead assigns the numbers):
 *  NEEDS REVIEW 316: 8 bit word with parity: the HAL returns the 7 data bits (DR & 0x7F), the parity bit is masked (RM0090 30.6.2 only
 *             says the MSB read is the parity bit).
 *  NEEDS REVIEW 316: a frame received with PE / FE / NE is still stored in the buffer and counts as one of Size frames (non blocking
 *             error: UM1725 only calls the error callback).
 *  NEEDS REVIEW 316: after a blocking error (ORE) the error code stays set (HAL_UART_GetError) and the buffer holds the frame read
 *             in the same call.
 *  NEEDS REVIEW 316: for an erroneous last frame the handler runs RxCpltCallback before ErrorCallback.
 *  NEEDS REVIEW 316: PE / FE / NF flag whose enable (PEIE / EIE) is 0 while RXNEIE is 1: the handler neither reports nor reads the
 *             frame (RXNE stays set). The API always sets EIE (and PEIE with parity), so only a test that clears them gets here.
 *  NEEDS REVIEW 316: IDLE branch with ReceptionType TOIDLE and IDLEIE returns without looking at TXE / TC, even when no event is
 *             reported; an error flag has priority over IDLE.
 *  NEEDS REVIEW 316: handler on a READY handle with TXEIE set and TXE = 1 does nothing and does not look at TC (state check inside
 *             UART_Transmit_IT); with RXNEIE and RXNE it does not read DR.
 *  NEEDS REVIEW 316: CR3.DMAR set with hdmarx NULL: every error is blocking, DMAR is cleared, the ErrorCallback runs directly.
 */
#include "HalFixture.h"
#include "HalUart.h"
#include "TestHelpers.h"
#include "UartTestSupport.h"

#include <cstdint>
#include <cstring>

namespace {

USART_TypeDef *uartIrqInstance(unsigned i) {
	switch (i) {
	case 0: return USART1;
	case 1: return USART2;
	case 2: return USART3;
	case 3: return UART4;
	case 4: return UART5;
	default: return USART6;
	}
}

struct UartIrqRig {
	UART_HandleTypeDef h;
	unsigned idx;
	USART_TypeDef *u() { return h.Instance; }
	uint32_t cr1() { return HalFixture::peek(&u()->CR1); }
	uint32_t cr3() { return HalFixture::peek(&u()->CR3); }
	uint32_t sr() { return HalFixture::peek(&u()->SR); }
	void setCr1(uint32_t bits) { HalFixture::poke(&u()->CR1, cr1() | bits); }
	void clrCr1(uint32_t bits) { HalFixture::poke(&u()->CR1, cr1() & ~bits); }
	void setCr3(uint32_t bits) { HalFixture::poke(&u()->CR3, cr3() | bits); }
	void clrCr3(uint32_t bits) { HalFixture::poke(&u()->CR3, cr3() & ~bits); }
	void setSr(uint32_t bits) { HalFixture::poke(&u()->SR, sr() | bits); }
};

void uartIrqInit(UartIrqRig &r, unsigned idx, bool nine = false, uint32_t parity = UART_PARITY_NONE) {
	std::memset(&r.h, 0, sizeof r.h);
	r.idx = idx;
	r.h.Instance = uartIrqInstance(idx);
	r.h.Init.BaudRate = 115200;
	r.h.Init.WordLength = nine ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
	r.h.Init.StopBits = UART_STOPBITS_1;
	r.h.Init.Parity = parity;
	r.h.Init.Mode = UART_MODE_TX_RX;
	r.h.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	r.h.Init.OverSampling = UART_OVERSAMPLING_16;
	REQUIRE(HAL_UART_Init(&r.h) == HAL_OK);
	uartSupportReset();
	hostUartTxClear(idx);
}

void uartIrqTick() { hostTickSet(hostTickPeek() + 1U); }

/* One tick, then one handler call, until the Tx state is READY (or maxIter reached). */
void uartIrqPumpTx(UartIrqRig &r, unsigned maxIter) {
	for (unsigned i = 0; i < maxIter && r.h.gState != HAL_UART_STATE_READY; ++i) {
		uartIrqTick();
		HAL_UART_IRQHandler(&r.h);
	}
}

/* The peer sends one frame, the handler runs once. */
void uartIrqRxOne(UartIrqRig &r, uint16_t bits, uint32_t err = 0) {
	hostUartRxNow(r.idx, bits, err);
	HAL_UART_IRQHandler(&r.h);
}

uint8_t uartIrqByte(unsigned k) { return (uint8_t)(0x80u + 7u * k); } /* 0x80, 0x87, 0x8E, ... all with the MSB set at first */

const unsigned uartIrqSizes[4] = {1, 2, 3, 16};

const uint32_t uartIrqRxIes = USART_CR1_RXNEIE | USART_CR1_PEIE;

} // namespace

/* ======================================================================== UQ1: Transmit_IT */

TEST_CASE_FIXTURE(UartFixture, "UQ1 Transmit_IT sets BUSY_TX and TXEIE only and returns HAL_OK on every instance") {
	for (unsigned idx = 0; idx < HOST_UART_COUNT; ++idx) {
		CAPTURE(idx);
		UartIrqRig r;
		uartIrqInit(r, idx);
		uint8_t d[2] = {1, 2};
		REQUIRE(HAL_UART_Transmit_IT(&r.h, d, 2) == HAL_OK);
		CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK((r.cr1() & USART_CR1_TXEIE) != 0);
		CHECK((r.cr1() & USART_CR1_TCIE) == 0);
		CHECK((r.cr1() & uartIrqRxIes) == 0);
		CHECK(hostUartTxCount(idx) == 0u); /* nothing is sent before the interrupt */
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ1 Transmit_IT clears a stale error code") {
	UartIrqRig r;
	uartIrqInit(r, 1);
	r.h.ErrorCode = HAL_UART_ERROR_ORE | HAL_UART_ERROR_FE;
	uint8_t d = 5;
	REQUIRE(HAL_UART_Transmit_IT(&r.h, &d, 1) == HAL_OK);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartFixture, "UQ1 Transmit_IT with a NULL buffer or Size 0 returns HAL_ERROR and starts nothing") {
	UartIrqRig r;
	uartIrqInit(r, 0);
	uint8_t d = 5;
	CHECK(HAL_UART_Transmit_IT(&r.h, nullptr, 3) == HAL_ERROR);
	CHECK(HAL_UART_Transmit_IT(&r.h, &d, 0) == HAL_ERROR);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK((r.cr1() & USART_CR1_TXEIE) == 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ1 Transmit_IT while a transmission runs returns HAL_BUSY and keeps the running one") {
	UartIrqRig r;
	uartIrqInit(r, 2);
	uint8_t a[2] = {0x11, 0x22}, b[2] = {0x33, 0x44};
	REQUIRE(HAL_UART_Transmit_IT(&r.h, a, 2) == HAL_OK);
	CHECK(HAL_UART_Transmit_IT(&r.h, b, 2) == HAL_BUSY);
	uartIrqPumpTx(r, 20);
	REQUIRE(hostUartTxCount(2) == 2u);
	CHECK(hostUartTxAt(2, 0) == 0x11);
	CHECK(hostUartTxAt(2, 1) == 0x22);
}

TEST_CASE_FIXTURE(UartFixture, "UQ1 each handler call with TXE sends exactly one frame") {
	UartIrqRig r;
	uartIrqInit(r, 0);
	uint8_t d[3] = {0xA1, 0xA2, 0xA3};
	REQUIRE(HAL_UART_Transmit_IT(&r.h, d, 3) == HAL_OK);
	HAL_UART_IRQHandler(&r.h);
	CHECK(hostUartTxCount(0) == 1u);
	HAL_UART_IRQHandler(&r.h); /* the first frame is in the shift register, TXE is 1 again: the second waits in TDR */
	CHECK(hostUartTxCount(0) == 1u);
	CHECK((r.sr() & USART_SR_TXE) == 0);
	uartIrqTick(); /* the first frame ended, the second moves to the shift register */
	HAL_UART_IRQHandler(&r.h); /* TXE = 1: the third frame is handed over */
	CHECK(hostUartTxCount(0) == 2u);
	CHECK(hostUartTxAt(0, 0) == 0xA1);
	CHECK(hostUartTxAt(0, 1) == 0xA2);
}

TEST_CASE_FIXTURE(UartFixture, "UQ1 a handler call with TXE = 0 sends nothing") {
	UartIrqRig r;
	uartIrqInit(r, 0);
	uint8_t d[3] = {1, 2, 3};
	REQUIRE(HAL_UART_Transmit_IT(&r.h, d, 3) == HAL_OK);
	HAL_UART_IRQHandler(&r.h);
	HAL_UART_IRQHandler(&r.h); /* second frame waits in TDR: TXE = 0 */
	REQUIRE((r.sr() & USART_SR_TXE) == 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(hostUartTxCount(0) == 1u); /* no further frame left the shift register */
	CHECK((r.sr() & USART_SR_TXE) == 0);
	CHECK((r.cr1() & (USART_CR1_TXEIE | USART_CR1_TCIE)) != 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ1 the bytes appear on the line in order for Size 1, 2, 3 and 16 on every instance") {
	for (unsigned idx = 0; idx < HOST_UART_COUNT; ++idx) {
		for (unsigned n : uartIrqSizes) {
			CAPTURE(idx);
			CAPTURE(n);
			UartIrqRig r;
			uartIrqInit(r, idx);
			uint8_t d[16];
			for (unsigned k = 0; k < n; ++k) d[k] = uartIrqByte(k);
			REQUIRE(HAL_UART_Transmit_IT(&r.h, d, (uint16_t)n) == HAL_OK);
			uartIrqPumpTx(r, 100);
			REQUIRE(hostUartTxCount(idx) == n);
			for (unsigned k = 0; k < n; ++k) CHECK(hostUartTxAt(idx, k) == uartIrqByte(k));
			CHECK(r.h.gState == HAL_UART_STATE_READY);
			CHECK(uartCounts.txCplt == 1u);
		}
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ1 after the last frame is handed over TXEIE is cleared, TCIE set and no callback has run") {
	UartIrqRig r;
	uartIrqInit(r, 1);
	uint8_t d[2] = {1, 2};
	REQUIRE(HAL_UART_Transmit_IT(&r.h, d, 2) == HAL_OK);
	HAL_UART_IRQHandler(&r.h);
	CHECK((r.cr1() & USART_CR1_TXEIE) != 0); /* one frame still to hand over */
	CHECK((r.cr1() & USART_CR1_TCIE) == 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK((r.cr1() & USART_CR1_TXEIE) == 0);
	CHECK((r.cr1() & USART_CR1_TCIE) != 0);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
	CHECK(uartCounts.txCplt == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ1 at TC the next handler call clears TCIE, sets READY and calls TxCpltCallback exactly once") {
	UartIrqRig r;
	uartIrqInit(r, 3);
	uint8_t d = 0x5A;
	REQUIRE(HAL_UART_Transmit_IT(&r.h, &d, 1) == HAL_OK);
	HAL_UART_IRQHandler(&r.h);
	REQUIRE((r.cr1() & USART_CR1_TCIE) != 0);
	CHECK((r.sr() & USART_SR_TC) == 0); /* the frame is still on the line */
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.txCplt == 0u);
	uartIrqTick();
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.txCplt == 1u);
	CHECK(uartCounts.last == &r.h);
	CHECK((r.cr1() & USART_CR1_TCIE) == 0);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK(uartCounts.gStateAtLast == HAL_UART_STATE_READY); /* state is READY when the callback runs */
	for (int i = 0; i < 3; ++i) {
		uartIrqTick();
		HAL_UART_IRQHandler(&r.h);
	}
	CHECK(uartCounts.txCplt == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ1 a second Transmit_IT after the first completed sends its own data") {
	UartIrqRig r;
	uartIrqInit(r, 5);
	uint8_t a = 0x31, b[2] = {0x32, 0x33};
	REQUIRE(HAL_UART_Transmit_IT(&r.h, &a, 1) == HAL_OK);
	uartIrqPumpTx(r, 20);
	REQUIRE(HAL_UART_Transmit_IT(&r.h, b, 2) == HAL_OK);
	uartIrqPumpTx(r, 20);
	REQUIRE(hostUartTxCount(5) == 3u);
	CHECK(hostUartTxAt(5, 1) == 0x32);
	CHECK(hostUartTxAt(5, 2) == 0x33);
	CHECK(uartCounts.txCplt == 2u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ1 9 bit words without parity send the low 9 bits of 16 bit values") {
	UartIrqRig r;
	uartIrqInit(r, 2, true);
	/* 0xFE5A & 0x1FF = 0x05A, 0x01A5 stays, 0x0100 stays */
	uint16_t d[3] = {0xFE5A, 0x01A5, 0x0100};
	REQUIRE(HAL_UART_Transmit_IT(&r.h, reinterpret_cast<const uint8_t *>(d), 3) == HAL_OK);
	uartIrqPumpTx(r, 30);
	REQUIRE(hostUartTxCount(2) == 3u);
	CHECK(hostUartTxAt(2, 0) == 0x05A);
	CHECK(hostUartTxAt(2, 1) == 0x1A5);
	CHECK(hostUartTxAt(2, 2) == 0x100);
	CHECK(uartCounts.txCplt == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ1 8 bit words with parity carry 7 data bits and the parity bit (even and odd)") {
	const struct { uint32_t parity; uint16_t in[2]; uint16_t line[2]; } t[2] = {
		{UART_PARITY_EVEN, {0x41, 0x07}, {0x41, 0x87}},
		{UART_PARITY_ODD, {0x41, 0x07}, {0xC1, 0x07}},
	};
	for (auto &c : t) {
		CAPTURE(c.parity);
		UartIrqRig r;
		uartIrqInit(r, 0, false, c.parity);
		uint8_t d[2] = {(uint8_t)c.in[0], (uint8_t)c.in[1]};
		REQUIRE(HAL_UART_Transmit_IT(&r.h, d, 2) == HAL_OK);
		uartIrqPumpTx(r, 30);
		REQUIRE(hostUartTxCount(0) == 2u);
		CHECK(hostUartTxAt(0, 0) == c.line[0]);
		CHECK(hostUartTxAt(0, 1) == c.line[1]);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ1 9 bit words with parity carry 8 data bits and the parity bit read from byte data") {
	const struct { uint32_t parity; uint8_t in[2]; uint16_t line[2]; } t[2] = {
		{UART_PARITY_EVEN, {0xFF, 0x01}, {0x0FF, 0x101}},
		{UART_PARITY_ODD, {0xFF, 0x01}, {0x1FF, 0x001}},
	};
	for (auto &c : t) {
		CAPTURE(c.parity);
		UartIrqRig r;
		uartIrqInit(r, 1, true, c.parity);
		REQUIRE(HAL_UART_Transmit_IT(&r.h, c.in, 2) == HAL_OK);
		uartIrqPumpTx(r, 30);
		REQUIRE(hostUartTxCount(1) == 2u);
		CHECK(hostUartTxAt(1, 0) == c.line[0]);
		CHECK(hostUartTxAt(1, 1) == c.line[1]);
	}
}

/* ======================================================================== UQ2: Receive_IT */

TEST_CASE_FIXTURE(UartFixture, "UQ2 Receive_IT without parity sets BUSY_RX, RXNEIE and EIE but not PEIE, TXEIE or IDLEIE") {
	for (unsigned idx = 0; idx < HOST_UART_COUNT; ++idx) {
		CAPTURE(idx);
		UartIrqRig r;
		uartIrqInit(r, idx);
		uint8_t buf[2];
		REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 2) == HAL_OK);
		CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK((r.cr1() & USART_CR1_RXNEIE) != 0);
		CHECK((r.cr3() & USART_CR3_EIE) != 0);
		CHECK((r.cr1() & USART_CR1_PEIE) == 0);
		CHECK((r.cr1() & (USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_IDLEIE)) == 0);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ2 Receive_IT with parity also sets PEIE") {
	for (uint32_t parity : {UART_PARITY_EVEN, UART_PARITY_ODD}) {
		UartIrqRig r;
		uartIrqInit(r, 1, false, parity);
		uint8_t buf[2];
		REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 2) == HAL_OK);
		CHECK((r.cr1() & USART_CR1_PEIE) != 0);
		CHECK((r.cr1() & USART_CR1_RXNEIE) != 0);
		CHECK((r.cr3() & USART_CR3_EIE) != 0);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ2 Receive_IT clears a stale error code") {
	UartIrqRig r;
	uartIrqInit(r, 4);
	r.h.ErrorCode = HAL_UART_ERROR_PE | HAL_UART_ERROR_NE;
	uint8_t buf[1];
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 1) == HAL_OK);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartFixture, "UQ2 Receive_IT with a NULL buffer or Size 0 returns HAL_ERROR and starts nothing") {
	UartIrqRig r;
	uartIrqInit(r, 0);
	uint8_t buf[1];
	CHECK(HAL_UART_Receive_IT(&r.h, nullptr, 2) == HAL_ERROR);
	CHECK(HAL_UART_Receive_IT(&r.h, buf, 0) == HAL_ERROR);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK((r.cr1() & USART_CR1_RXNEIE) == 0);
	CHECK((r.cr3() & USART_CR3_EIE) == 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ2 Receive_IT while a reception runs returns HAL_BUSY and keeps the running one") {
	UartIrqRig r;
	uartIrqInit(r, 0);
	uint8_t a[2] = {0, 0}, b[2] = {0, 0};
	REQUIRE(HAL_UART_Receive_IT(&r.h, a, 2) == HAL_OK);
	CHECK(HAL_UART_Receive_IT(&r.h, b, 2) == HAL_BUSY);
	uartIrqRxOne(r, 0x61);
	uartIrqRxOne(r, 0x62);
	CHECK(a[0] == 0x61);
	CHECK(a[1] == 0x62);
	CHECK(b[0] == 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ2 each handler call with RXNE reads one frame, the last one ends the reception for Size 1, 2, 3 and 16") {
	for (unsigned idx = 0; idx < HOST_UART_COUNT; ++idx) {
		for (unsigned n : uartIrqSizes) {
			CAPTURE(idx);
			CAPTURE(n);
			UartIrqRig r;
			uartIrqInit(r, idx);
			uint8_t buf[16] = {};
			REQUIRE(HAL_UART_Receive_IT(&r.h, buf, (uint16_t)n) == HAL_OK);
			for (unsigned k = 0; k < n; ++k) {
				CHECK(uartCounts.rxCplt == 0u);
				CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
				uartIrqRxOne(r, uartIrqByte(k));
				CHECK(buf[k] == uartIrqByte(k));
				CHECK((r.sr() & USART_SR_RXNE) == 0); /* the frame was read */
			}
			CHECK(uartCounts.rxCplt == 1u);
			CHECK(r.h.RxState == HAL_UART_STATE_READY);
			CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
			CHECK(uartCounts.rxStateAtLast == HAL_UART_STATE_READY);
		}
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ2 after the last frame RXNEIE, PEIE and EIE are cleared") {
	UartIrqRig r;
	uartIrqInit(r, 2, false, UART_PARITY_EVEN);
	uint8_t buf[1];
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 1) == HAL_OK);
	REQUIRE((r.cr1() & uartIrqRxIes) == uartIrqRxIes);
	uartIrqRxOne(r, 0x41);
	CHECK((r.cr1() & uartIrqRxIes) == 0);
	CHECK((r.cr3() & USART_CR3_EIE) == 0);
	CHECK(uartCounts.rxCplt == 1u);
	CHECK(uartCounts.last == &r.h);
}

TEST_CASE_FIXTURE(UartFixture, "UQ2 a frame arriving after the reception ended does not call the callback again") {
	UartIrqRig r;
	uartIrqInit(r, 0);
	uint8_t buf[1];
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 1) == HAL_OK);
	uartIrqRxOne(r, 0x10);
	uartIrqRxOne(r, 0x20); /* RXNEIE is 0 now */
	CHECK(uartCounts.rxCplt == 1u);
	CHECK(buf[0] == 0x10);
	CHECK((r.sr() & USART_SR_RXNE) != 0); /* nobody read it */
}

TEST_CASE_FIXTURE(UartFixture, "UQ2 a second Receive_IT after the first completed receives its own data") {
	UartIrqRig r;
	uartIrqInit(r, 3);
	uint8_t a[1], b[2];
	REQUIRE(HAL_UART_Receive_IT(&r.h, a, 1) == HAL_OK);
	uartIrqRxOne(r, 0x71);
	REQUIRE(HAL_UART_Receive_IT(&r.h, b, 2) == HAL_OK);
	uartIrqRxOne(r, 0x72);
	uartIrqRxOne(r, 0x73);
	CHECK(b[0] == 0x72);
	CHECK(b[1] == 0x73);
	CHECK(uartCounts.rxCplt == 2u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ2 9 bit words without parity are stored as 16 bit values with 9 valid bits") {
	UartIrqRig r;
	uartIrqInit(r, 1, true);
	uint16_t buf[3] = {};
	REQUIRE(HAL_UART_Receive_IT(&r.h, reinterpret_cast<uint8_t *>(buf), 3) == HAL_OK);
	uartIrqRxOne(r, 0x1A5);
	uartIrqRxOne(r, 0x100);
	uartIrqRxOne(r, 0x0FF);
	CHECK(buf[0] == 0x1A5);
	CHECK(buf[1] == 0x100);
	CHECK(buf[2] == 0x0FF);
	CHECK(uartCounts.rxCplt == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ2 9 bit words with parity are stored as bytes holding the 8 data bits") {
	const struct { uint32_t parity; uint16_t line[2]; } t[2] = {
		{UART_PARITY_EVEN, {0x0FF, 0x101}},
		{UART_PARITY_ODD, {0x1FF, 0x001}},
	};
	for (auto &c : t) {
		CAPTURE(c.parity);
		UartIrqRig r;
		uartIrqInit(r, 0, true, c.parity);
		uint8_t buf[2] = {};
		REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 2) == HAL_OK);
		uartIrqRxOne(r, c.line[0]);
		uartIrqRxOne(r, c.line[1]);
		CHECK(buf[0] == 0xFF);
		CHECK(buf[1] == 0x01);
		CHECK(uartCounts.error == 0u);
		CHECK(uartCounts.rxCplt == 1u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ2 characterization: 8 bit words with parity are stored without the parity bit (NEEDS REVIEW 316)") {
	const struct { uint32_t parity; uint16_t line[2]; } t[2] = {
		{UART_PARITY_EVEN, {0x41, 0x87}},
		{UART_PARITY_ODD, {0xC1, 0x07}},
	};
	for (auto &c : t) {
		CAPTURE(c.parity);
		UartIrqRig r;
		uartIrqInit(r, 1, false, c.parity);
		uint8_t buf[2] = {};
		REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 2) == HAL_OK);
		uartIrqRxOne(r, c.line[0]);
		uartIrqRxOne(r, c.line[1]);
		CHECK(buf[0] == 0x41);
		CHECK(buf[1] == 0x07);
		CHECK(uartCounts.error == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ2 8 bit words without parity keep the MSB of the data") {
	UartIrqRig r;
	uartIrqInit(r, 5);
	uint8_t buf[1] = {};
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 1) == HAL_OK);
	uartIrqRxOne(r, 0xFF);
	CHECK(buf[0] == 0xFF);
}

/* ======================================================================== UQ3: errors in IT mode */

/* Receive_IT of 4 frames (parity: with PEIE), returns with the transfer started. */
#define UART_IRQ_START_RX(r, idx, buf, nine, parity)    \
	uartIrqInit(r, idx, nine, parity);                  \
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 4) == HAL_OK)

TEST_CASE_FIXTURE(UartFixture, "UQ3 an overrun is a blocking error: transfer ended, READY, code ORE, ErrorCallback once") {
	for (unsigned idx = 0; idx < HOST_UART_COUNT; ++idx) {
		CAPTURE(idx);
		UartIrqRig r;
		uint8_t buf[4] = {};
		UART_IRQ_START_RX(r, idx, buf, false, UART_PARITY_NONE);
		hostUartRxNow(idx, 0x11, 0);
		hostUartRxNow(idx, 0x22, 0); /* RXNE still set: ORE */
		REQUIRE((r.sr() & USART_SR_ORE) != 0);
		HAL_UART_IRQHandler(&r.h);
		CHECK(uartCounts.error == 1u);
		CHECK(uartCounts.errorCodeAtError == HAL_UART_ERROR_ORE);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(uartCounts.rxCplt == 0u);
		CHECK((r.cr1() & uartIrqRxIes) == 0);
		CHECK((r.cr3() & USART_CR3_EIE) == 0);
		CHECK((r.sr() & USART_SR_ORE) == 0); /* SR read then DR read clears it (RM0090 30.6.1) */
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 characterization: after the overrun the code stays ORE and the first frame is in the buffer (NEEDS REVIEW 316)") {
	UartIrqRig r;
	uint8_t buf[4] = {};
	UART_IRQ_START_RX(r, 0, buf, false, UART_PARITY_NONE);
	hostUartRxNow(0, 0x11, 0);
	hostUartRxNow(0, 0x22, 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_ORE);
	CHECK(buf[0] == 0x11);
	CHECK(buf[1] == 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 after a blocking error a new Receive_IT can start and succeeds") {
	UartIrqRig r;
	uint8_t buf[4] = {}, b2[1] = {};
	UART_IRQ_START_RX(r, 2, buf, false, UART_PARITY_NONE);
	hostUartRxNow(2, 0x11, 0);
	hostUartRxNow(2, 0x22, 0);
	HAL_UART_IRQHandler(&r.h);
	REQUIRE(HAL_UART_Receive_IT(&r.h, b2, 1) == HAL_OK);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
	uartIrqRxOne(r, 0x44);
	CHECK(b2[0] == 0x44);
	CHECK(uartCounts.rxCplt == 1u);
	CHECK(uartCounts.error == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 an overrun is reported with EIE alone (RXNEIE 0)") {
	UartIrqRig r;
	uint8_t buf[4] = {};
	UART_IRQ_START_RX(r, 1, buf, false, UART_PARITY_NONE);
	r.clrCr1(USART_CR1_RXNEIE);
	hostUartRxNow(1, 0x11, 0);
	hostUartRxNow(1, 0x22, 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 1u);
	CHECK(uartCounts.errorCodeAtError == HAL_UART_ERROR_ORE);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK((r.cr3() & USART_CR3_EIE) == 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 an overrun is reported with RXNEIE alone (EIE 0)") {
	UartIrqRig r;
	uint8_t buf[4] = {};
	UART_IRQ_START_RX(r, 3, buf, false, UART_PARITY_NONE);
	r.clrCr3(USART_CR3_EIE);
	hostUartRxNow(3, 0x11, 0);
	hostUartRxNow(3, 0x22, 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 1u);
	CHECK(uartCounts.errorCodeAtError == HAL_UART_ERROR_ORE);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 an overrun with only PEIE set is not reported (RM0090 Table 147: ORE needs RXNEIE or EIE)") {
	UartIrqRig r;
	uint8_t buf[4] = {};
	UART_IRQ_START_RX(r, 0, buf, false, UART_PARITY_EVEN);
	r.clrCr1(USART_CR1_RXNEIE);
	r.clrCr3(USART_CR3_EIE);
	hostUartRxNow(0, 0x11, 0);
	hostUartRxNow(0, 0x22, 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 0u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 a parity error is non blocking: code PE in the callback, cleared after it, transfer goes on") {
	UartIrqRig r;
	uint8_t buf[3] = {};
	uartIrqInit(r, 2, false, UART_PARITY_EVEN);
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 3) == HAL_OK);
	uartIrqRxOne(r, 0xC1); /* data 0x41 needs parity bit 0 for even parity: PE */
	CHECK(uartCounts.error == 1u);
	CHECK(uartCounts.errorCodeAtError == HAL_UART_ERROR_PE);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK((r.cr1() & USART_CR1_RXNEIE) != 0);
	uartIrqRxOne(r, 0x41);
	uartIrqRxOne(r, 0x87);
	CHECK(uartCounts.rxCplt == 1u);
	CHECK(uartCounts.error == 1u);
	CHECK(buf[1] == 0x41);
	CHECK(buf[2] == 0x07);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 characterization: the frame with the parity error is stored and counts as one frame (NEEDS REVIEW 316)") {
	UartIrqRig r;
	uint8_t buf[2] = {};
	uartIrqInit(r, 1, false, UART_PARITY_EVEN);
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 2) == HAL_OK);
	uartIrqRxOne(r, 0xC1);
	CHECK(buf[0] == 0x41);
	uartIrqRxOne(r, 0x41);
	CHECK(uartCounts.rxCplt == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 a framing error is non blocking: code FE, callback, code cleared, transfer goes on") {
	for (unsigned idx = 0; idx < HOST_UART_COUNT; ++idx) {
		CAPTURE(idx);
		UartIrqRig r;
		uint8_t buf[2] = {};
		uartIrqInit(r, idx);
		REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 2) == HAL_OK);
		uartIrqRxOne(r, 0x00, HOST_UART_RX_FE);
		CHECK(uartCounts.error == 1u);
		CHECK(uartCounts.errorCodeAtError == HAL_UART_ERROR_FE);
		CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
		CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
		uartIrqRxOne(r, 0x55);
		CHECK(buf[1] == 0x55);
		CHECK(uartCounts.rxCplt == 1u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 a noise error is non blocking: code NE, callback, code cleared, transfer goes on") {
	for (unsigned idx = 0; idx < HOST_UART_COUNT; ++idx) {
		CAPTURE(idx);
		UartIrqRig r;
		uint8_t buf[2] = {};
		uartIrqInit(r, idx);
		REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 2) == HAL_OK);
		uartIrqRxOne(r, 0x3C, HOST_UART_RX_NOISE);
		CHECK(uartCounts.error == 1u);
		CHECK(uartCounts.errorCodeAtError == HAL_UART_ERROR_NE);
		CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
		CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
		uartIrqRxOne(r, 0x55);
		CHECK(uartCounts.rxCplt == 1u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 PE, FE and NE together give one callback with all three codes, then the code is cleared") {
	UartIrqRig r;
	uint8_t buf[2] = {};
	uartIrqInit(r, 0, false, UART_PARITY_EVEN);
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 2) == HAL_OK);
	uartIrqRxOne(r, 0xC1, HOST_UART_RX_FE | HOST_UART_RX_NOISE);
	CHECK(uartCounts.error == 1u);
	CHECK(uartCounts.errorCodeAtError == (HAL_UART_ERROR_PE | HAL_UART_ERROR_FE | HAL_UART_ERROR_NE));
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 an overrun together with a framing error is blocking and reports both codes") {
	UartIrqRig r;
	uint8_t buf[4] = {};
	UART_IRQ_START_RX(r, 4, buf, false, UART_PARITY_NONE);
	hostUartRxNow(4, 0x00, HOST_UART_RX_FE);
	hostUartRxNow(4, 0x22, 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 1u);
	CHECK(uartCounts.errorCodeAtError == (HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE));
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 characterization: for an erroneous last frame RxCpltCallback runs before ErrorCallback (NEEDS REVIEW 316)") {
	UartIrqRig r;
	uint8_t buf[1] = {};
	uartIrqInit(r, 0);
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 1) == HAL_OK);
	uartIrqRxOne(r, 0x00, HOST_UART_RX_FE);
	REQUIRE(uartCounts.orderCount == 2u);
	CHECK(uartCounts.order[0] == UART_CB_RXCPLT);
	CHECK(uartCounts.order[1] == UART_CB_ERROR);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 a framing error is reported with EIE alone and the frame is not read without RXNEIE") {
	UartIrqRig r;
	uint8_t buf[2] = {};
	uartIrqInit(r, 1);
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 2) == HAL_OK);
	r.clrCr1(USART_CR1_RXNEIE);
	hostUartRxNow(1, 0x66, HOST_UART_RX_FE);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 1u);
	CHECK(uartCounts.errorCodeAtError == HAL_UART_ERROR_FE);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK(buf[0] == 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 characterization: FE with RXNEIE but without EIE reports nothing and does not read the frame (NEEDS REVIEW 316)") {
	UartIrqRig r;
	uint8_t buf[2] = {};
	uartIrqInit(r, 1);
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 2) == HAL_OK);
	r.clrCr3(USART_CR3_EIE);
	hostUartRxNow(1, 0x66, HOST_UART_RX_FE);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 0u);
	CHECK(buf[0] == 0);
	CHECK((r.sr() & USART_SR_RXNE) != 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 characterization: NE with RXNEIE but without EIE reports nothing (NEEDS REVIEW 316)") {
	UartIrqRig r;
	uint8_t buf[2] = {};
	uartIrqInit(r, 2);
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 2) == HAL_OK);
	r.clrCr3(USART_CR3_EIE);
	hostUartRxNow(2, 0x66, HOST_UART_RX_NOISE);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 0u);
	CHECK((r.sr() & USART_SR_RXNE) != 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 characterization: PE with RXNEIE but without PEIE reports nothing and does not read the frame (NEEDS REVIEW 316)") {
	UartIrqRig r;
	uint8_t buf[2] = {};
	uartIrqInit(r, 0, false, UART_PARITY_EVEN);
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 2) == HAL_OK);
	r.clrCr1(USART_CR1_PEIE);
	hostUartRxNow(0, 0xC1, 0);
	REQUIRE((r.sr() & USART_SR_PE) != 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 0u);
	CHECK(buf[0] == 0);
	CHECK((r.sr() & USART_SR_RXNE) != 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 a parity error with PEIE alone is reported and the frame is not read") {
	UartIrqRig r;
	uint8_t buf[2] = {};
	uartIrqInit(r, 3, false, UART_PARITY_ODD);
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 2) == HAL_OK);
	r.clrCr1(USART_CR1_RXNEIE);
	r.clrCr3(USART_CR3_EIE);
	hostUartRxNow(3, 0x41, 0); /* odd parity needs bit 7 = 1 for data 0x41: PE */
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 1u);
	CHECK(uartCounts.errorCodeAtError == HAL_UART_ERROR_PE);
	CHECK(buf[0] == 0);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 characterization: with DMAR set and no DMA handle an overrun ends the reception and calls ErrorCallback (NEEDS REVIEW 316)") {
	UartIrqRig r;
	uint8_t buf[4] = {};
	UART_IRQ_START_RX(r, 1, buf, false, UART_PARITY_NONE);
	r.setCr3(USART_CR3_DMAR);
	REQUIRE(r.h.hdmarx == nullptr);
	hostUartRxNow(1, 0x11, 0);
	hostUartRxNow(1, 0x22, 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 1u);
	CHECK(uartCounts.errorCodeAtError == HAL_UART_ERROR_ORE);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK((r.cr3() & USART_CR3_DMAR) == 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 characterization: with DMAR set and no DMA handle a framing error is blocking too (NEEDS REVIEW 316)") {
	UartIrqRig r;
	uint8_t buf[4] = {};
	UART_IRQ_START_RX(r, 2, buf, false, UART_PARITY_NONE);
	r.setCr3(USART_CR3_DMAR);
	hostUartRxNow(2, 0x00, HOST_UART_RX_FE);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 1u);
	CHECK(uartCounts.errorCodeAtError == HAL_UART_ERROR_FE);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK((r.cr3() & USART_CR3_DMAR) == 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ3 a blocking error ends a ReceiveToIdle reception and clears IDLEIE") {
	UartIrqRig r;
	uint8_t buf[4] = {};
	uartIrqInit(r, 0);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 4) == HAL_OK);
	REQUIRE((r.cr1() & USART_CR1_IDLEIE) != 0);
	hostUartRxNow(0, 0x11, 0);
	hostUartRxNow(0, 0x22, 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 1u);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK((r.cr1() & USART_CR1_IDLEIE) == 0);
	CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
}

/* ======================================================================== UQ4: enables, idle peripheral, order */

TEST_CASE_FIXTURE(UartFixture, "UQ4 the handler on an idle peripheral does nothing") {
	for (unsigned idx = 0; idx < HOST_UART_COUNT; ++idx) {
		CAPTURE(idx);
		UartIrqRig r;
		uartIrqInit(r, idx);
		const uint32_t cr1 = r.cr1(), cr3 = r.cr3(), sr = r.sr();
		HAL_UART_IRQHandler(&r.h);
		CHECK(r.cr1() == cr1);
		CHECK(r.cr3() == cr3);
		CHECK(r.sr() == sr);
		CHECK(uartCounts.orderCount == 0u);
		CHECK(hostUartTxCount(idx) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ4 TXE and TC flags without their enables do nothing") {
	UartIrqRig r;
	uartIrqInit(r, 0);
	REQUIRE((r.sr() & (USART_SR_TXE | USART_SR_TC)) == (USART_SR_TXE | USART_SR_TC));
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.txCplt == 0u);
	CHECK(hostUartTxCount(0) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ4 TC without TCIE during a transmission does not complete it") {
	UartIrqRig r;
	uartIrqInit(r, 1);
	uint8_t d = 0x42;
	REQUIRE(HAL_UART_Transmit_IT(&r.h, &d, 1) == HAL_OK);
	HAL_UART_IRQHandler(&r.h);
	r.clrCr1(USART_CR1_TCIE);
	uartIrqTick();
	HAL_UART_IRQHandler(&r.h);
	REQUIRE((r.sr() & USART_SR_TC) != 0);
	CHECK(uartCounts.txCplt == 0u);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
}

TEST_CASE_FIXTURE(UartFixture, "UQ4 RXNE without RXNEIE does not read the frame") {
	UartIrqRig r;
	uint8_t buf[1] = {};
	uartIrqInit(r, 2);
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 1) == HAL_OK);
	r.clrCr1(USART_CR1_RXNEIE);
	r.clrCr3(USART_CR3_EIE);
	hostUartRxNow(2, 0x99, 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(buf[0] == 0);
	CHECK(uartCounts.rxCplt == 0u);
	CHECK((r.sr() & USART_SR_RXNE) != 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ4 error flags without any receive enable do not stop a running transmission") {
	UartIrqRig r;
	uartIrqInit(r, 0);
	uint8_t d[2] = {0x71, 0x72};
	REQUIRE(HAL_UART_Transmit_IT(&r.h, d, 2) == HAL_OK);
	hostUartRxNow(0, 0x00, HOST_UART_RX_FE);
	hostUartRxNow(0, 0x01, 0); /* ORE */
	REQUIRE((r.sr() & (USART_SR_FE | USART_SR_ORE)) == (USART_SR_FE | USART_SR_ORE));
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 0u);
	CHECK(hostUartTxCount(0) == 1u);
	uartIrqPumpTx(r, 20);
	CHECK(hostUartTxCount(0) == 2u);
	CHECK(uartCounts.txCplt == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ4 a plain Receive_IT ignores an IDLE flag with IDLEIE set") {
	UartIrqRig r;
	uint8_t buf[3] = {};
	uartIrqInit(r, 1);
	REQUIRE(HAL_UART_Receive_IT(&r.h, buf, 3) == HAL_OK);
	uartIrqRxOne(r, 0x31);
	r.setCr1(USART_CR1_IDLEIE);
	hostUartIdleLine(1);
	REQUIRE((r.sr() & USART_SR_IDLE) != 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.rxEvent == 0u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
}

TEST_CASE_FIXTURE(UartFixture, "UQ4 characterization: on a READY handle TXEIE with TXE does nothing and TC is not looked at (NEEDS REVIEW 316)") {
	UartIrqRig r;
	uartIrqInit(r, 0);
	r.setCr1(USART_CR1_TXEIE | USART_CR1_TCIE);
	REQUIRE((r.sr() & (USART_SR_TXE | USART_SR_TC)) == (USART_SR_TXE | USART_SR_TC));
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.txCplt == 0u);
	CHECK(hostUartTxCount(0) == 0u);
	r.clrCr1(USART_CR1_TXEIE);
	HAL_UART_IRQHandler(&r.h); /* now TC is served */
	CHECK(uartCounts.txCplt == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ4 characterization: on a READY handle RXNEIE with RXNE does not read DR (NEEDS REVIEW 316)") {
	UartIrqRig r;
	uartIrqInit(r, 3);
	r.setCr1(USART_CR1_RXNEIE);
	hostUartRxNow(3, 0x21, 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.rxCplt == 0u);
	CHECK((r.sr() & USART_SR_RXNE) != 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ4 characterization: with TXE and TC both set and both enables the handler serves TXE first") {
	UartIrqRig r;
	uint8_t d[2] = {0x10, 0x20};
	uartIrqInit(r, 2);
	REQUIRE(HAL_UART_Transmit_IT(&r.h, d, 2) == HAL_OK);
	r.setCr1(USART_CR1_TCIE);
	/* the idle transmitter at reset has TC = 1 and TXE = 1 */
	REQUIRE((r.sr() & (USART_SR_TXE | USART_SR_TC)) == (USART_SR_TXE | USART_SR_TC));
	HAL_UART_IRQHandler(&r.h);
	CHECK(hostUartTxCount(2) == 1u);
	CHECK(uartCounts.txCplt == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ4 characterization: RXNE is served before IDLE (NEEDS REVIEW 316)") {
	UartIrqRig r;
	uint8_t buf[4] = {};
	uartIrqInit(r, 0);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 4) == HAL_OK);
	hostUartRxNow(0, 0x11, 0);
	hostUartIdleLine(0);
	REQUIRE((r.sr() & (USART_SR_RXNE | USART_SR_IDLE)) == (USART_SR_RXNE | USART_SR_IDLE));
	HAL_UART_IRQHandler(&r.h);
	CHECK(buf[0] == 0x11);
	CHECK(uartCounts.rxEvent == 0u); /* the first call only read the byte (and cleared IDLE by SR + DR) */
}

TEST_CASE_FIXTURE(UartFixture, "UQ4 characterization: an error flag has priority over IDLE (NEEDS REVIEW 316)") {
	UartIrqRig r;
	uint8_t buf[4] = {};
	uartIrqInit(r, 0);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 4) == HAL_OK);
	uartIrqRxOne(r, 0x11);
	r.setSr(USART_SR_IDLE);
	hostUartRxNow(0, 0x00, HOST_UART_RX_FE);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 1u);
	CHECK(uartCounts.rxEvent == 0u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
}

TEST_CASE_FIXTURE(UartFixture, "UQ4 characterization: a handler call with IDLE and no data consumes the call, TXE waits for the next one (NEEDS REVIEW 316)") {
	UartIrqRig r;
	uint8_t rx[4] = {}, tx[2] = {0x01, 0x02};
	uartIrqInit(r, 1);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, rx, 4) == HAL_OK);
	REQUIRE(HAL_UART_Transmit_IT(&r.h, tx, 2) == HAL_OK);
	r.setSr(USART_SR_IDLE);
	HAL_UART_IRQHandler(&r.h);
	CHECK(hostUartTxCount(1) == 0u);
	CHECK(uartCounts.rxEvent == 0u);
	HAL_UART_IRQHandler(&r.h);
	CHECK(hostUartTxCount(1) == 1u);
}

/* ======================================================================== UQ5: ReceiveToIdle_IT */

TEST_CASE_FIXTURE(UartFixture, "UQ5 ReceiveToIdle_IT sets BUSY_RX, IDLEIE, RXNEIE and EIE (PEIE with parity) and returns HAL_OK") {
	for (unsigned idx = 0; idx < HOST_UART_COUNT; ++idx) {
		CAPTURE(idx);
		UartIrqRig r;
		uint8_t buf[4];
		uartIrqInit(r, idx);
		REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 4) == HAL_OK);
		CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
		CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_TOIDLE);
		CHECK((r.cr1() & USART_CR1_IDLEIE) != 0);
		CHECK((r.cr1() & USART_CR1_RXNEIE) != 0);
		CHECK((r.cr3() & USART_CR3_EIE) != 0);
		CHECK((r.cr1() & USART_CR1_PEIE) == 0);
	}
	UartIrqRig p;
	uint8_t buf[4];
	uartIrqInit(p, 1, false, UART_PARITY_ODD);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&p.h, buf, 4) == HAL_OK);
	CHECK((p.cr1() & USART_CR1_PEIE) != 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 ReceiveToIdle_IT clears a pending IDLE flag") {
	UartIrqRig r;
	uint8_t buf[4];
	uartIrqInit(r, 2);
	hostUartRxNow(2, 0x01, 0);
	hostUartIdleLine(2);
	REQUIRE((r.sr() & USART_SR_IDLE) != 0);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 4) == HAL_OK);
	CHECK((r.sr() & USART_SR_IDLE) == 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 ReceiveToIdle_IT with a NULL buffer or Size 0 returns HAL_ERROR, a busy handle HAL_BUSY") {
	UartIrqRig r;
	uint8_t buf[4];
	uartIrqInit(r, 0);
	CHECK(HAL_UARTEx_ReceiveToIdle_IT(&r.h, nullptr, 4) == HAL_ERROR);
	CHECK(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 0) == HAL_ERROR);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK((r.cr1() & USART_CR1_IDLEIE) == 0);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 4) == HAL_OK);
	CHECK(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 4) == HAL_BUSY);
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 the idle line after fewer than Size frames calls RxEventCallback with the received length") {
	for (unsigned idx = 0; idx < HOST_UART_COUNT; ++idx) {
		for (unsigned n = 1; n <= 3; ++n) {
			CAPTURE(idx);
			CAPTURE(n);
			UartIrqRig r;
			uint8_t buf[8] = {};
			uartIrqInit(r, idx);
			REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 8) == HAL_OK);
			for (unsigned k = 0; k < n; ++k) uartIrqRxOne(r, uartIrqByte(k));
			CHECK(uartCounts.rxEvent == 0u);
			hostUartIdleLine(idx);
			HAL_UART_IRQHandler(&r.h);
			CHECK(uartCounts.rxEvent == 1u);
			CHECK(uartCounts.rxEventSize == n);
			CHECK(uartCounts.rxCplt == 0u);
			CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_IDLE);
			for (unsigned k = 0; k < n; ++k) CHECK(buf[k] == uartIrqByte(k));
		}
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 after the idle event the reception is ended, interrupts cleared, IDLE flag cleared, READY") {
	UartIrqRig r;
	uint8_t buf[8] = {};
	uartIrqInit(r, 0, false, UART_PARITY_EVEN);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 8) == HAL_OK);
	uartIrqRxOne(r, 0x41);
	hostUartIdleLine(0);
	REQUIRE((r.sr() & USART_SR_IDLE) != 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK(uartCounts.rxStateAtLast == HAL_UART_STATE_READY);
	CHECK((r.cr1() & (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_IDLEIE)) == 0);
	CHECK((r.cr3() & USART_CR3_EIE) == 0);
	CHECK((r.sr() & USART_SR_IDLE) == 0);
	CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 a second idle flag after the event does nothing") {
	UartIrqRig r;
	uint8_t buf[8] = {};
	uartIrqInit(r, 1);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 8) == HAL_OK);
	uartIrqRxOne(r, 0x01);
	hostUartIdleLine(1);
	HAL_UART_IRQHandler(&r.h);
	REQUIRE(uartCounts.rxEvent == 1u);
	r.setCr1(USART_CR1_IDLEIE);
	r.setSr(USART_SR_IDLE);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.rxEvent == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 all Size frames give RxEventCallback with Size and event type TC, not RxCpltCallback") {
	for (unsigned idx = 0; idx < HOST_UART_COUNT; ++idx) {
		CAPTURE(idx);
		UartIrqRig r;
		uint8_t buf[3] = {};
		uartIrqInit(r, idx);
		REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 3) == HAL_OK);
		uartIrqRxOne(r, 0x91);
		uartIrqRxOne(r, 0x92);
		CHECK(uartCounts.rxEvent == 0u);
		uartIrqRxOne(r, 0x93);
		CHECK(uartCounts.rxEvent == 1u);
		CHECK(uartCounts.rxEventSize == 3u);
		CHECK(uartCounts.rxCplt == 0u);
		CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK((r.cr1() & USART_CR1_IDLEIE) == 0);
		CHECK((r.cr1() & uartIrqRxIes) == 0);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 an IDLE flag set together with the last frame is cleared and the Size event is the only one") {
	UartIrqRig r;
	uint8_t buf[2] = {};
	uartIrqInit(r, 3);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 2) == HAL_OK);
	uartIrqRxOne(r, 0x01);
	hostUartRxNow(3, 0x02, 0);
	hostUartIdleLine(3); /* IDLE together with the last frame still unread */
	REQUIRE((r.sr() & USART_SR_IDLE) != 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.rxEvent == 1u);
	CHECK(uartCounts.rxEventSize == 2u);
	CHECK((r.sr() & USART_SR_IDLE) == 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.rxEvent == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 an IDLE flag before any data was received gives no event and keeps the reception running") {
	UartIrqRig r;
	uint8_t buf[4] = {};
	uartIrqInit(r, 0);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 4) == HAL_OK);
	r.setSr(USART_SR_IDLE);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.rxEvent == 0u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK((r.cr1() & USART_CR1_IDLEIE) != 0);
	CHECK((r.sr() & USART_SR_IDLE) == 0); /* the handler cleared the flag */
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 after an idle event a new ReceiveToIdle_IT and a plain Receive_IT both start") {
	UartIrqRig r;
	uint8_t a[4] = {}, b[1] = {};
	uartIrqInit(r, 2);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, a, 4) == HAL_OK);
	uartIrqRxOne(r, 0x05);
	hostUartIdleLine(2);
	HAL_UART_IRQHandler(&r.h);
	REQUIRE(uartCounts.rxEvent == 1u);
	REQUIRE(HAL_UART_Receive_IT(&r.h, b, 1) == HAL_OK);
	CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
	uartIrqRxOne(r, 0x06);
	CHECK(uartCounts.rxCplt == 1u);
	CHECK(uartCounts.rxEvent == 1u);
	CHECK(b[0] == 0x06);
}

/* ======================================================================== UQ6: Tx and Rx together */

TEST_CASE_FIXTURE(UartFixture, "UQ6 Transmit_IT and Receive_IT together show BUSY_TX_RX") {
	UartIrqRig r;
	uint8_t tx[2] = {1, 2}, rx[2];
	uartIrqInit(r, 1);
	REQUIRE(HAL_UART_Transmit_IT(&r.h, tx, 2) == HAL_OK);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_BUSY_TX);
	REQUIRE(HAL_UART_Receive_IT(&r.h, rx, 2) == HAL_OK);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_BUSY_TX_RX);
}

TEST_CASE_FIXTURE(UartFixture, "UQ6 the state falls back to BUSY_RX when the transmission ends first and to READY after the reception") {
	UartIrqRig r;
	uint8_t tx[1] = {0x77}, rx[2] = {};
	uartIrqInit(r, 0);
	REQUIRE(HAL_UART_Receive_IT(&r.h, rx, 2) == HAL_OK);
	REQUIRE(HAL_UART_Transmit_IT(&r.h, tx, 1) == HAL_OK);
	uartIrqPumpTx(r, 20);
	CHECK(uartCounts.txCplt == 1u);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_BUSY_RX);
	uartIrqRxOne(r, 0x01);
	uartIrqRxOne(r, 0x02);
	CHECK(uartCounts.rxCplt == 1u);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
	CHECK(hostUartTxAt(0, 0) == 0x77);
}

TEST_CASE_FIXTURE(UartFixture, "UQ6 the state falls back to BUSY_TX when the reception ends first") {
	UartIrqRig r;
	uint8_t tx[2] = {0x01, 0x02}, rx[1] = {};
	uartIrqInit(r, 3);
	REQUIRE(HAL_UART_Transmit_IT(&r.h, tx, 2) == HAL_OK);
	REQUIRE(HAL_UART_Receive_IT(&r.h, rx, 1) == HAL_OK);
	uartIrqRxOne(r, 0x5A);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_BUSY_TX);
	CHECK(rx[0] == 0x5A);
	uartIrqPumpTx(r, 20);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
	CHECK(hostUartTxCount(3) == 2u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ6 with TXE and RXNE both pending the handler serves RXNE first, TXE with the next call") {
	UartIrqRig r;
	uint8_t tx[2] = {0xA1, 0xA2}, rx[2] = {};
	uartIrqInit(r, 2);
	REQUIRE(HAL_UART_Transmit_IT(&r.h, tx, 2) == HAL_OK);
	REQUIRE(HAL_UART_Receive_IT(&r.h, rx, 2) == HAL_OK);
	hostUartRxNow(2, 0x33, 0);
	REQUIRE((r.sr() & (USART_SR_TXE | USART_SR_RXNE)) == (USART_SR_TXE | USART_SR_RXNE));
	HAL_UART_IRQHandler(&r.h);
	CHECK(rx[0] == 0x33);
	CHECK(hostUartTxCount(2) == 0u);
	HAL_UART_IRQHandler(&r.h);
	CHECK(hostUartTxCount(2) == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ6 Transmit_IT runs while a polling Receive of the peer data completes and keeps its own state") {
	UartIrqRig r;
	uint8_t tx[2] = {0x41, 0x42}, rx[2] = {};
	uartIrqInit(r, 1);
	REQUIRE(HAL_UART_Transmit_IT(&r.h, tx, 2) == HAL_OK);
	hostUartRxPush(1, 0xC3, 0);
	hostUartRxPush(1, 0xC4, 0);
	REQUIRE(HAL_UART_Receive(&r.h, rx, 2, 100) == HAL_OK);
	CHECK(rx[0] == 0xC3);
	CHECK(rx[1] == 0xC4);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	uartIrqPumpTx(r, 20);
	CHECK(uartCounts.txCplt == 1u);
	CHECK(hostUartTxCount(1) == 2u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ6 Receive_IT keeps running while a polling Transmit completes") {
	UartIrqRig r;
	uint8_t tx[2] = {0x51, 0x52}, rx[2] = {};
	uartIrqInit(r, 4);
	REQUIRE(HAL_UART_Receive_IT(&r.h, rx, 2) == HAL_OK);
	REQUIRE(HAL_UART_Transmit(&r.h, tx, 2, 100) == HAL_OK);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK(hostUartTxCount(4) == 2u);
	uartIrqRxOne(r, 0x08);
	uartIrqRxOne(r, 0x09);
	CHECK(rx[0] == 0x08);
	CHECK(rx[1] == 0x09);
	CHECK(uartCounts.rxCplt == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UQ6 a reception error does not disturb a running transmission") {
	UartIrqRig r;
	uint8_t tx[3] = {1, 2, 3}, rx[4] = {};
	uartIrqInit(r, 0);
	REQUIRE(HAL_UART_Transmit_IT(&r.h, tx, 3) == HAL_OK);
	REQUIRE(HAL_UART_Receive_IT(&r.h, rx, 4) == HAL_OK);
	hostUartRxNow(0, 0x11, 0);
	hostUartRxNow(0, 0x22, 0); /* ORE */
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 1u);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
	CHECK((r.cr1() & USART_CR1_TXEIE) != 0);
	uartIrqPumpTx(r, 30);
	CHECK(hostUartTxCount(0) == 3u);
	CHECK(uartCounts.txCplt == 1u);
}

/* ======================================================================== UQ5 fix pass: ToIdle end states and IDLE boundaries */

TEST_CASE_FIXTURE(UartFixture, "UQ5 the Size event ends ReceiveToIdle_IT with ReceptionType STANDARD, event type TC and IDLEIE cleared") {
	for (unsigned idx = 0; idx < HOST_UART_COUNT; ++idx) {
		CAPTURE(idx);
		UartIrqRig r;
		uint8_t buf[2] = {};
		uartIrqInit(r, idx);
		REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 2) == HAL_OK);
		uartIrqRxOne(r, 0x01);
		uartIrqRxOne(r, 0x02);
		CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
		CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
		CHECK((r.cr1() & USART_CR1_IDLEIE) == 0);
		CHECK(uartCounts.rxEventSize == 2u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 an idle line pushed with the last frame leaves IDLE cleared, one Size event and event type TC") {
	/* On the real USART the DR read of the last frame already clears IDLE (SR read, then DR read, RM0090 30.6.1), so the
	 * HAL's own IDLE clear in UART_Receive_IT (line 3656) is not reachable on the host; the observable outcome is checked. */
	for (unsigned idx = 0; idx < HOST_UART_COUNT; ++idx) {
		CAPTURE(idx);
		UartIrqRig r;
		uint8_t buf[2] = {};
		uartIrqInit(r, idx);
		REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 2) == HAL_OK);
		uartIrqRxOne(r, 0x01);
		hostUartRxNow(idx, 0x02, 0);
		hostUartIdleLine(idx);
		HAL_UART_IRQHandler(&r.h);
		CHECK(uartCounts.rxEvent == 1u);
		CHECK(uartCounts.rxEventSize == 2u);
		CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
		CHECK((r.sr() & USART_SR_IDLE) == 0);
		CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
		HAL_UART_IRQHandler(&r.h);
		CHECK(uartCounts.rxEvent == 1u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 characterization: IDLE with nothing remaining to receive gives no event and leaves the reception state alone (NR_UART_i)") {
	UartIrqRig r;
	uint8_t buf[4] = {};
	uartIrqInit(r, 0);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 4) == HAL_OK);
	uartIrqRxOne(r, 0x01);
	r.h.RxXferCount = 0; /* the state the HAL has when every element was received but the end was not processed */
	r.setSr(USART_SR_IDLE);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.rxEvent == 0u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK((r.cr1() & USART_CR1_IDLEIE) != 0);
	CHECK((r.sr() & USART_SR_IDLE) == 0);
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 one frame before IDLE already gives an event of length 1 (boundary of received > 0)") {
	UartIrqRig r;
	uint8_t buf[2] = {};
	uartIrqInit(r, 2);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, buf, 2) == HAL_OK);
	uartIrqRxOne(r, 0x7E);
	hostUartIdleLine(2);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.rxEvent == 1u);
	CHECK(uartCounts.rxEventSize == 1u);
	CHECK(buf[0] == 0x7E);
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 characterization: after an idle event ReceptionType is STANDARD and the event type stays IDLE until the next reception ends (NR_UART_j)") {
	UartIrqRig r;
	uint8_t a[4] = {}, b[1] = {};
	uartIrqInit(r, 1);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, a, 4) == HAL_OK);
	uartIrqRxOne(r, 0x01);
	hostUartIdleLine(1);
	HAL_UART_IRQHandler(&r.h);
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_IDLE);
	CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
	REQUIRE(HAL_UART_Receive_IT(&r.h, b, 1) == HAL_OK);
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_IDLE); /* only reset when that reception ends */
	uartIrqRxOne(r, 0x02);
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 a later IDLE does not produce an event for a plain Receive_IT started after an idle event") {
	UartIrqRig r;
	uint8_t a[4] = {}, b[3] = {};
	uartIrqInit(r, 3);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, a, 4) == HAL_OK);
	uartIrqRxOne(r, 0x01);
	hostUartIdleLine(3);
	HAL_UART_IRQHandler(&r.h);
	REQUIRE(uartCounts.rxEvent == 1u);
	REQUIRE(HAL_UART_Receive_IT(&r.h, b, 3) == HAL_OK);
	uartIrqRxOne(r, 0x02);
	r.setCr1(USART_CR1_IDLEIE);
	hostUartIdleLine(3);
	REQUIRE((r.sr() & USART_SR_IDLE) != 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.rxEvent == 1u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
}

TEST_CASE_FIXTURE(UartFixture, "UQ5 a blocking error during ReceiveToIdle_IT resets ReceptionType so a later IDLE gives no event for a plain Receive_IT") {
	UartIrqRig r;
	uint8_t a[4] = {}, b[3] = {};
	uartIrqInit(r, 0);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&r.h, a, 4) == HAL_OK);
	hostUartRxNow(0, 0x11, 0);
	hostUartRxNow(0, 0x22, 0);
	HAL_UART_IRQHandler(&r.h);
	REQUIRE(uartCounts.error == 1u);
	REQUIRE(HAL_UART_Receive_IT(&r.h, b, 3) == HAL_OK);
	uartIrqRxOne(r, 0x02);
	r.setCr1(USART_CR1_IDLEIE);
	hostUartIdleLine(0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.rxEvent == 0u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
}
