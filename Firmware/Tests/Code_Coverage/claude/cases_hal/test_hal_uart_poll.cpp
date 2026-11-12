/*
 * P12 T4b group UP: UART polling transfers (HAL_UART_Transmit, HAL_UART_Receive, HAL_UARTEx_ReceiveToIdle, HAL_UARTEx_GetRxEventType and
 * the internal UART_WaitOnFlagUntilTimeout) of stm32f4xx_hal_uart.c against the UART register model (fakes/stm32_hal/HalUart.h).
 * Requirements UP1-UP8 of Tests/Code_Coverage/claude/charters/hal/P12_T4B_REQUIREMENTS.md; compiled into ffb_hal_tests and
 * ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 chapter 70 (blocking API: HAL_OK / HAL_ERROR / HAL_BUSY / HAL_TIMEOUT, "9 bits without parity: pData is a set of
 * u16", ReceiveToIdle: "HAL_OK if the expected number of data is received or if reception is stopped after IDLE event, RxLen gives the
 * number of data", GetRxEventType) and RM0090 Rev 22 chapter 30 (30.3.2 transmitter, 30.3.3 receiver, 30.3.7 parity, Table 146,
 * 30.6.1 SR, 30.6.2 DR).
 *
 * Parity frames are hand derived (RM0090 30.3.7: with PCE = 1 the MSB of the word is the parity bit; PS = 0 even: the number of 1 bits of
 * the whole frame is even; PS = 1 odd: it is odd; Table 146: M = 0 and PCE = 1: 7 data bits + parity at bit 7; M = 1 and PCE = 1: 8 data
 * bits + parity at bit 8). Parity bit = xor of the data bits for even parity, its inverse for odd parity.
 *
 * Time: HAL_GetTick of the host runs one tick per call (HalHostSystem.h). Timeout cases check the elapsed ticks as hostTickPeek() after
 * minus before: the HAL stops when HAL_GetTick() - tickstart > Timeout, so the lower bound is Timeout and the upper bound allows the two
 * ticks of the start up (tickstart read, first loop pass) plus one of slack.
 *
 * NEEDS REVIEW proposals (characterization cases; the lead numbers them, proposals use NEEDS REVIEW 315):
 *  NEEDS REVIEW 315: with parity and an 8 bit word HAL_UART_Receive / ReceiveToIdle return the data masked to 7 bits (UM1725 is silent about
 *             the masking; RM0090 30.6.2 says the parity is the MSB of the word).
 *  NEEDS REVIEW 315: a polling Transmit / Receive that times out leaves ErrorCode NONE and the state READY (UM1725 names no error code).
 *  NEEDS REVIEW 315: Timeout 0 means "no wait": Transmit of a frame that takes a tick returns HAL_TIMEOUT, Receive with a frame already
 *             received returns HAL_OK (UM1725 only says "Timeout duration").
 *  NEEDS REVIEW 315: PE / FE / NE during polling Receive are not reported: HAL_OK, ErrorCode NONE (UM1725 and RM0090 silent about the HAL).
 *  NEEDS REVIEW 315: ReceiveToIdle does not look at ORE (it waits until the Timeout), and HAL_UART_Receive with HAL_MAX_DELAY ignores an ORE
 *             that is pending (the ORE check sits inside "Timeout != HAL_MAX_DELAY"). Reported to the lead as a decision item.
 *  NEEDS REVIEW 315: ReceiveToIdle clears a stale ErrorCode at its start (UM1725 is silent).
 *  NEEDS REVIEW 315: on a ReceiveToIdle timeout *RxLen holds the number of frames received so far.
 */
#include "HalFixture.h"
#include "TestHelpers.h"
#include "UartTestSupport.h"

#include <cstdint>
#include <cstring>

namespace {

struct UartPollRig {
	UART_HandleTypeDef h;
	unsigned idx;
	USART_TypeDef *inst() { return h.Instance; }
};

USART_TypeDef *uartPollInstance(unsigned i) {
	switch (i) {
	case 0: return USART1;
	case 1: return USART2;
	case 2: return USART3;
	case 3: return UART4;
	case 4: return UART5;
	default: return USART6;
	}
}

void uartPollInit(UartPollRig &r, unsigned idx, uint32_t word = UART_WORDLENGTH_8B, uint32_t parity = UART_PARITY_NONE) {
	std::memset(&r.h, 0, sizeof r.h);
	r.idx = idx;
	r.h.Instance = uartPollInstance(idx);
	r.h.Init.BaudRate = 115200;
	r.h.Init.WordLength = word;
	r.h.Init.StopBits = UART_STOPBITS_1;
	r.h.Init.Parity = parity;
	r.h.Init.Mode = UART_MODE_TX_RX;
	r.h.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	r.h.Init.OverSampling = UART_OVERSAMPLING_16;
	REQUIRE(HAL_UART_Init(&r.h) == HAL_OK);
	hostUartTxClear(idx);
	uartSupportReset();
}

void uartPollPush(unsigned idx, const uint16_t *bits, unsigned n, uint32_t err = 0) {
	for (unsigned k = 0; k < n; ++k) hostUartRxPush(idx, bits[k], err);
}

uint32_t uartPollSr(UartPollRig &r) { return HalFixture::peek(&r.inst()->SR); }

/* Frames that were on the line must equal the expected list. */
void uartPollCheckLine(unsigned idx, const uint16_t *exp, unsigned n) {
	REQUIRE(hostUartTxCount(idx) == n);
	for (unsigned k = 0; k < n; ++k) {
		INFO("frame " << k);
		CHECK(hostUartTxAt(idx, k) == exp[k]);
	}
}

/* Tick step hooks (HalHostSystem.h): every HAL_GetTick runs one of them. */
unsigned uartPollStepCount, uartPollStepIdleAt, uartPollStepIdx;
UART_HandleTypeDef *uartPollStepHandle;
bool uartPollSawBusyTx;

void uartPollStepIdle() {
	HAL_IncTick();
	if (++uartPollStepCount == uartPollStepIdleAt) hostUartIdleLine(uartPollStepIdx);
}

void uartPollStepPush() {
	HAL_IncTick();
	if (++uartPollStepCount == 3) {
		hostUartRxPush(uartPollStepIdx, 0x61, 0);
		hostUartRxPush(uartPollStepIdx, 0x62, 0);
	}
}

bool uartPollSawBusyRx;
void uartPollStepWatchRx() {
	HAL_IncTick();
	if (HAL_UART_GetState(uartPollStepHandle) == HAL_UART_STATE_BUSY_RX) uartPollSawBusyRx = true;
}

void uartPollStepWatch() {
	HAL_IncTick();
	if (HAL_UART_GetState(uartPollStepHandle) == HAL_UART_STATE_BUSY_TX) uartPollSawBusyTx = true;
}

void uartPollArmIdle(unsigned idx, unsigned atStep) {
	uartPollStepCount = 0;
	uartPollStepIdleAt = atStep;
	uartPollStepIdx = idx;
	hostTickSetStep(uartPollStepIdle);
}

/* Elapsed ticks of a call, with the tick read before and after (HAL_GetTick would add one). */
struct UartPollClock {
	uint32_t before;
	UartPollClock() : before(hostTickPeek()) {}
	uint32_t elapsed() const { return hostTickPeek() - before; }
};

void uartPollCheckTimeoutBounds(uint32_t elapsed, uint32_t timeout) {
	INFO("elapsed " << elapsed << " timeout " << timeout);
	CHECK(elapsed >= timeout);
	CHECK(elapsed <= timeout + 3u);
}

}  // namespace

/* ------------------------------------------------------------------------------------------------------------------ UP1 */

TEST_CASE_FIXTURE(UartFixture, "UP1 8 bit no parity transmit puts the bytes on the line in order on every instance") {
	static const uint8_t data[] = {0x00, 0x55, 0xA5, 0xFF};
	static const uint16_t line[] = {0x00, 0x55, 0xA5, 0xFF};
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartPollRig r;
		uartPollInit(r, i);
		REQUIRE(HAL_UART_Transmit(&r.h, data, sizeof data, 100) == HAL_OK);
		uartPollCheckLine(i, line, 4);
		CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
		CHECK(hostUartConfigWhileEnabled(i) == 0u);
		CHECK(hostUartDisabledBusy(i) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UP1 transmit leaves the other instances without a frame") {
	static const uint8_t data[] = {0x11, 0x22};
	UartPollRig a, b;
	uartPollInit(a, 1);
	uartPollInit(b, 2);
	REQUIRE(HAL_UART_Transmit(&a.h, data, 2, 100) == HAL_OK);
	CHECK(hostUartTxCount(1) == 2u);
	CHECK(hostUartTxCount(2) == 0u);
	CHECK(hostUartTxCount(0) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UP1 transmit returns with TC set and TXE set and no frame left in flight") {
	static const uint8_t data[] = {0x31, 0x32, 0x33};
	UartPollRig r;
	uartPollInit(r, 0);
	hostUartSetFrameTicks(2);
	REQUIRE(HAL_UART_Transmit(&r.h, data, 3, 100) == HAL_OK);
	CHECK(hostUartTxCount(0) == 3u);
	CHECK((uartPollSr(r) & USART_SR_TC) != 0u);
	CHECK((uartPollSr(r) & USART_SR_TXE) != 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UP1 transmit takes at least the time of all frames: the call waits for TC of the last frame") {
	// Hand derivation: 2 frames of 3 ticks each run one after the other (one shift register): at least 6 ticks.
	static const uint8_t data[] = {0x41, 0x42};
	UartPollRig r;
	uartPollInit(r, 2);
	hostUartSetFrameTicks(3);
	UartPollClock clk;
	REQUIRE(HAL_UART_Transmit(&r.h, data, 2, 100) == HAL_OK);
	CHECK(clk.elapsed() >= 6u);
	CHECK(clk.elapsed() <= 9u);
}

TEST_CASE_FIXTURE(UartFixture, "UP1 the state is BUSY_TX while a transmit runs and READY afterwards") {
	static const uint8_t data[] = {0x01, 0x02, 0x03};
	UartPollRig r;
	uartPollInit(r, 3);
	hostUartSetFrameTicks(2);
	uartPollStepHandle = &r.h;
	uartPollSawBusyTx = false;
	hostTickSetStep(uartPollStepWatch);
	REQUIRE(HAL_UART_Transmit(&r.h, data, 3, 100) == HAL_OK);
	CHECK(uartPollSawBusyTx);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
	CHECK(r.h.ErrorCode == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartFixture, "UP1 transmit clears an old error code at the start") {
	static const uint8_t data[] = {0x77};
	UartPollRig r;
	uartPollInit(r, 0);
	r.h.ErrorCode = HAL_UART_ERROR_FE;
	REQUIRE(HAL_UART_Transmit(&r.h, data, 1, 100) == HAL_OK);
	CHECK(r.h.ErrorCode == HAL_UART_ERROR_NONE);
}

/* ------------------------------------------------------------------------------------------------------------------ UP2 */

TEST_CASE_FIXTURE(UartFixture, "UP2 8 bit word with even parity sends 7 data bits and the parity at bit 7") {
	// Even: parity = xor of the 7 data bits. 0x41 (2 ones) -> 0, 0x07 (3) -> 1, 0xFF: data 0x7F (7) -> 1, 0x80: data 0 -> 0, 0x00 -> 0.
	static const uint8_t data[] = {0x41, 0x07, 0xFF, 0x80, 0x00};
	static const uint16_t line[] = {0x41, 0x87, 0xFF, 0x00, 0x00};
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartPollRig r;
		uartPollInit(r, i, UART_WORDLENGTH_8B, UART_PARITY_EVEN);
		REQUIRE(HAL_UART_Transmit(&r.h, data, 5, 100) == HAL_OK);
		uartPollCheckLine(i, line, 5);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UP2 8 bit word with odd parity sends 7 data bits and the parity at bit 7") {
	// Odd: parity = inverse of the xor. 0x41 -> 1 (0xC1), 0x07 -> 0 (0x07), 0xFF: data 0x7F -> 0 (0x7F), 0x80: data 0 -> 1 (0x80), 0x00 -> 1 (0x80).
	static const uint8_t data[] = {0x41, 0x07, 0xFF, 0x80, 0x00};
	static const uint16_t line[] = {0xC1, 0x07, 0x7F, 0x80, 0x80};
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartPollRig r;
		uartPollInit(r, i, UART_WORDLENGTH_8B, UART_PARITY_ODD);
		REQUIRE(HAL_UART_Transmit(&r.h, data, 5, 100) == HAL_OK);
		uartPollCheckLine(i, line, 5);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UP2 9 bit word with even parity sends 8 data bits and the parity at bit 8") {
	// Even: 0x55 (4 ones) -> 0 (0x055), 0x07 (3) -> 1 (0x107), 0xFF (8) -> 0 (0x0FF), 0x00 -> 0, 0x80 (1) -> 1 (0x180).
	static const uint8_t data[] = {0x55, 0x07, 0xFF, 0x00, 0x80};
	static const uint16_t line[] = {0x055, 0x107, 0x0FF, 0x000, 0x180};
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartPollRig r;
		uartPollInit(r, i, UART_WORDLENGTH_9B, UART_PARITY_EVEN);
		REQUIRE(HAL_UART_Transmit(&r.h, data, 5, 100) == HAL_OK);
		uartPollCheckLine(i, line, 5);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UP2 9 bit word with odd parity sends 8 data bits and the parity at bit 8") {
	// Odd: 0x55 -> 1 (0x155), 0x07 -> 0 (0x007), 0xFF -> 1 (0x1FF), 0x00 -> 1 (0x100), 0x80 -> 0 (0x080).
	static const uint8_t data[] = {0x55, 0x07, 0xFF, 0x00, 0x80};
	static const uint16_t line[] = {0x155, 0x007, 0x1FF, 0x100, 0x080};
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartPollRig r;
		uartPollInit(r, i, UART_WORDLENGTH_9B, UART_PARITY_ODD);
		REQUIRE(HAL_UART_Transmit(&r.h, data, 5, 100) == HAL_OK);
		uartPollCheckLine(i, line, 5);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UP2 9 bit word without parity reads pData as 16 bit values and sends 9 bits") {
	// No parity: the frame is the low 9 bits of each value; 0xFFFF -> 0x1FF.
	static const uint16_t data[] = {0x1A5, 0x0FF, 0x100, 0x000, 0xFFFF};
	static const uint16_t line[] = {0x1A5, 0x0FF, 0x100, 0x000, 0x1FF};
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartPollRig r;
		uartPollInit(r, i, UART_WORDLENGTH_9B, UART_PARITY_NONE);
		REQUIRE(HAL_UART_Transmit(&r.h, reinterpret_cast<const uint8_t *>(data), 5, 100) == HAL_OK);
		uartPollCheckLine(i, line, 5);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UP2 8 bit word without parity sends all 8 bits of the byte") {
	static const uint8_t data[] = {0x80, 0xFF, 0x7F};
	static const uint16_t line[] = {0x80, 0xFF, 0x7F};
	UartPollRig r;
	uartPollInit(r, 0);
	REQUIRE(HAL_UART_Transmit(&r.h, data, 3, 100) == HAL_OK);
	uartPollCheckLine(0, line, 3);
}

/* ------------------------------------------------------------------------------------------------------------------ UP3 */

TEST_CASE_FIXTURE(UartFixture, "UP3 receive of an 8 bit no parity word returns the bytes the peer sent on every instance") {
	static const uint16_t line[] = {0x00, 0x5A, 0xFF, 0x80};
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartPollRig r;
		uartPollInit(r, i);
		uartPollPush(i, line, 4);
		uint8_t buf[4] = {1, 1, 1, 1};
		REQUIRE(HAL_UART_Receive(&r.h, buf, 4, 100) == HAL_OK);
		CHECK(buf[0] == 0x00);
		CHECK(buf[1] == 0x5A);
		CHECK(buf[2] == 0xFF);
		CHECK(buf[3] == 0x80);
		CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
		CHECK(r.h.ErrorCode == HAL_UART_ERROR_NONE);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UP3 characterization: receive of an 8 bit even parity word returns the 7 data bits without the parity bit") {
	// Line frames (hand derived, even): data 0x41 -> 0x41 (2 ones), data 0x07 -> 0x87 (3 ones + parity 1), data 0x7F -> 0xFF (7 ones + parity 1).
	static const uint16_t line[] = {0x41, 0x87, 0xFF};
	UartPollRig r;
	uartPollInit(r, 1, UART_WORDLENGTH_8B, UART_PARITY_EVEN);
	uartPollPush(1, line, 3);
	uint8_t buf[3] = {0, 0, 0};
	REQUIRE(HAL_UART_Receive(&r.h, buf, 3, 100) == HAL_OK);
	CHECK(buf[0] == 0x41);
	CHECK(buf[1] == 0x07);
	CHECK(buf[2] == 0x7F);
	CHECK((uartPollSr(r) & USART_SR_PE) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UP3 characterization: receive of an 8 bit odd parity word returns the 7 data bits without the parity bit") {
	// Line frames (odd): data 0x41 -> 0xC1 (2 ones + parity 1), data 0x07 -> 0x07 (3 ones), data 0x7F -> 0x7F (7 ones).
	static const uint16_t line[] = {0xC1, 0x07, 0x7F};
	UartPollRig r;
	uartPollInit(r, 2, UART_WORDLENGTH_8B, UART_PARITY_ODD);
	uartPollPush(2, line, 3);
	uint8_t buf[3] = {0, 0, 0};
	REQUIRE(HAL_UART_Receive(&r.h, buf, 3, 100) == HAL_OK);
	CHECK(buf[0] == 0x41);
	CHECK(buf[1] == 0x07);
	CHECK(buf[2] == 0x7F);
}

TEST_CASE_FIXTURE(UartFixture, "UP3 receive of a 9 bit even parity word returns the 8 data bits without the parity bit") {
	// Even, 9 bit: 0x55 -> 0x055, 0x07 -> 0x107, 0xFF -> 0x0FF, 0x80 -> 0x180 (parity at bit 8, RM0090 Table 146).
	static const uint16_t line[] = {0x055, 0x107, 0x0FF, 0x180};
	UartPollRig r;
	uartPollInit(r, 3, UART_WORDLENGTH_9B, UART_PARITY_EVEN);
	uartPollPush(3, line, 4);
	uint8_t buf[4] = {0, 0, 0, 0};
	REQUIRE(HAL_UART_Receive(&r.h, buf, 4, 100) == HAL_OK);
	CHECK(buf[0] == 0x55);
	CHECK(buf[1] == 0x07);
	CHECK(buf[2] == 0xFF);
	CHECK(buf[3] == 0x80);
}

TEST_CASE_FIXTURE(UartFixture, "UP3 receive of a 9 bit odd parity word returns the 8 data bits without the parity bit") {
	// Odd, 9 bit: 0x55 -> 0x155, 0x07 -> 0x007, 0xFF -> 0x1FF, 0x00 -> 0x100.
	static const uint16_t line[] = {0x155, 0x007, 0x1FF, 0x100};
	UartPollRig r;
	uartPollInit(r, 4, UART_WORDLENGTH_9B, UART_PARITY_ODD);
	uartPollPush(4, line, 4);
	uint8_t buf[4] = {1, 1, 1, 1};
	REQUIRE(HAL_UART_Receive(&r.h, buf, 4, 100) == HAL_OK);
	CHECK(buf[0] == 0x55);
	CHECK(buf[1] == 0x07);
	CHECK(buf[2] == 0xFF);
	CHECK(buf[3] == 0x00);
}

TEST_CASE_FIXTURE(UartFixture, "UP3 receive of a 9 bit no parity word returns 16 bit values of 9 bits") {
	static const uint16_t line[] = {0x1A5, 0x0FF, 0x100, 0x000};
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartPollRig r;
		uartPollInit(r, i, UART_WORDLENGTH_9B, UART_PARITY_NONE);
		uartPollPush(i, line, 4);
		uint16_t buf[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
		REQUIRE(HAL_UART_Receive(&r.h, reinterpret_cast<uint8_t *>(buf), 4, 100) == HAL_OK);
		CHECK(buf[0] == 0x1A5);
		CHECK(buf[1] == 0x0FF);
		CHECK(buf[2] == 0x100);
		CHECK(buf[3] == 0x000);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UP3 receive reads exactly Size frames and leaves the next one pending") {
	static const uint16_t line[] = {0x10, 0x20, 0x30};
	UartPollRig r;
	uartPollInit(r, 0);
	uartPollPush(0, line, 3);
	uint8_t buf[2] = {0, 0};
	REQUIRE(HAL_UART_Receive(&r.h, buf, 2, 100) == HAL_OK);
	CHECK(buf[0] == 0x10);
	CHECK(buf[1] == 0x20);
	uint8_t next = 0;
	REQUIRE(HAL_UART_Receive(&r.h, &next, 1, 100) == HAL_OK);
	CHECK(next == 0x30);
}

/* ------------------------------------------------------------------------------------------------------------------ UP4 */

TEST_CASE_FIXTURE(UartFixture, "UP4 transmit times out after about Timeout ticks when the last frame never completes") {
	static const uint8_t data[] = {0x5A};
	UartPollRig r;
	uartPollInit(r, 0);
	hostUartTxStuck(0, 1);
	UartPollClock clk;
	CHECK(HAL_UART_Transmit(&r.h, data, 1, 10) == HAL_TIMEOUT);
	uartPollCheckTimeoutBounds(clk.elapsed(), 10);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UP4 transmit times out while waiting for TXE of a later byte") {
	// Byte 1 is in the shift register (stuck), byte 2 in TDR, byte 3 waits for TXE: only one frame ever starts.
	static const uint8_t data[] = {0x01, 0x02, 0x03};
	UartPollRig r;
	uartPollInit(r, 1);
	hostUartTxStuck(1, 1);
	UartPollClock clk;
	CHECK(HAL_UART_Transmit(&r.h, data, 3, 25) == HAL_TIMEOUT);
	uartPollCheckTimeoutBounds(clk.elapsed(), 25);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
	CHECK(hostUartTxCount(1) == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UP4 transmit with a frame time longer than Timeout times out and with a longer Timeout completes") {
	static const uint8_t data[] = {0x66};
	UartPollRig r;
	uartPollInit(r, 2);
	hostUartSetFrameTicks(5);
	UartPollClock clk;
	CHECK(HAL_UART_Transmit(&r.h, data, 1, 3) == HAL_TIMEOUT);
	uartPollCheckTimeoutBounds(clk.elapsed(), 3);
	// The frame is still running in the model: a new transmit later finishes with enough time.
	CHECK(HAL_UART_Transmit(&r.h, data, 1, 50) == HAL_OK);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UP4 receive times out after about Timeout ticks when no frame arrives") {
	UartPollRig r;
	uartPollInit(r, 3);
	uint8_t buf[2] = {0, 0};
	UartPollClock clk;
	CHECK(HAL_UART_Receive(&r.h, buf, 2, 7) == HAL_TIMEOUT);
	uartPollCheckTimeoutBounds(clk.elapsed(), 7);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UP4 receive of more frames than the peer sent returns the frames that came and then times out") {
	static const uint16_t line[] = {0x21, 0x22};
	UartPollRig r;
	uartPollInit(r, 4);
	uartPollPush(4, line, 2);
	uint8_t buf[4] = {0, 0, 0, 0};
	UartPollClock clk;
	CHECK(HAL_UART_Receive(&r.h, buf, 4, 12) == HAL_TIMEOUT);
	uartPollCheckTimeoutBounds(clk.elapsed(), 12);
	CHECK(buf[0] == 0x21);
	CHECK(buf[1] == 0x22);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UP4 receive completes when the frames come within the Timeout") {
	// 3 frames, one per 4 ticks: the last arrives at tick 12, well inside Timeout 40.
	static const uint16_t line[] = {0x01, 0x02, 0x03};
	UartPollRig r;
	uartPollInit(r, 5);
	hostUartSetFrameTicks(4);
	uartPollPush(5, line, 3);
	uint8_t buf[3] = {0, 0, 0};
	CHECK(HAL_UART_Receive(&r.h, buf, 3, 40) == HAL_OK);
	CHECK(buf[2] == 0x03);
}

TEST_CASE_FIXTURE(UartFixture, "UP4 characterization: a timed out transmit and receive leave the error code NONE and the state READY") {
	static const uint8_t data[] = {0x01};
	UartPollRig r;
	uartPollInit(r, 0);
	hostUartTxStuck(0, 1);
	CHECK(HAL_UART_Transmit(&r.h, data, 1, 3) == HAL_TIMEOUT);
	CHECK(r.h.ErrorCode == HAL_UART_ERROR_NONE);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	uint8_t b = 0;
	CHECK(HAL_UART_Receive(&r.h, &b, 1, 3) == HAL_TIMEOUT);
	CHECK(r.h.ErrorCode == HAL_UART_ERROR_NONE);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UP4 characterization: Timeout 0 times a transmit out at once") {
	// One frame takes a tick, so TC is not yet set when the HAL looks at it once.
	static const uint8_t data[] = {0x44};
	UartPollRig r;
	uartPollInit(r, 0);
	UartPollClock clk;
	CHECK(HAL_UART_Transmit(&r.h, data, 1, 0) == HAL_TIMEOUT);
	CHECK(clk.elapsed() <= 3u);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UP4 characterization: Timeout 0 receive returns a frame that is already there and times out otherwise") {
	UartPollRig r;
	uartPollInit(r, 1);
	uint8_t b = 0;
	CHECK(HAL_UART_Receive(&r.h, &b, 1, 0) == HAL_TIMEOUT);
	hostUartRxNow(1, 0x6B, 0);
	CHECK(HAL_UART_Receive(&r.h, &b, 1, 0) == HAL_OK);
	CHECK(b == 0x6B);
}

TEST_CASE_FIXTURE(UartFixture, "UP4 HAL_MAX_DELAY transmit completes") {
	static const uint8_t data[] = {0x10, 0x20, 0x30};
	static const uint16_t line[] = {0x10, 0x20, 0x30};
	UartPollRig r;
	uartPollInit(r, 0);
	hostUartSetFrameTicks(3);
	hostUartSetPollReads(1);
	CHECK(HAL_UART_Transmit(&r.h, data, 3, HAL_MAX_DELAY) == HAL_OK);
	uartPollCheckLine(0, line, 3);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UP4 HAL_MAX_DELAY receive completes") {
	static const uint16_t line[] = {0x91, 0x92};
	UartPollRig r;
	uartPollInit(r, 2);
	hostUartSetFrameTicks(3);
	hostUartSetPollReads(1);
	uartPollPush(2, line, 2);
	uint8_t buf[2] = {0, 0};
	CHECK(HAL_UART_Receive(&r.h, buf, 2, HAL_MAX_DELAY) == HAL_OK);
	CHECK(buf[0] == 0x91);
	CHECK(buf[1] == 0x92);
}

/* ------------------------------------------------------------------------------------------------------------------ UP5 */

TEST_CASE_FIXTURE(UartFixture, "UP5 a NULL data pointer returns HAL_ERROR and leaves the state READY") {
	UartPollRig r;
	uartPollInit(r, 0);
	uint16_t len = 0;
	CHECK(HAL_UART_Transmit(&r.h, nullptr, 3, 10) == HAL_ERROR);
	CHECK(HAL_UART_Receive(&r.h, nullptr, 3, 10) == HAL_ERROR);
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, nullptr, 3, &len, 10) == HAL_ERROR);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK(hostUartTxCount(0) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UP5 Size 0 returns HAL_ERROR and leaves the state READY") {
	UartPollRig r;
	uartPollInit(r, 1);
	uint8_t buf[2] = {0, 0};
	uint16_t len = 0;
	CHECK(HAL_UART_Transmit(&r.h, buf, 0, 10) == HAL_ERROR);
	CHECK(HAL_UART_Receive(&r.h, buf, 0, 10) == HAL_ERROR);
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 0, &len, 10) == HAL_ERROR);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK(hostUartTxCount(1) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UP5 transmit while a transmit runs returns HAL_BUSY and sends nothing") {
	static const uint8_t data[] = {0x01};
	UartPollRig r;
	uartPollInit(r, 0);
	REQUIRE(HAL_UART_Transmit_IT(&r.h, data, 1) == HAL_OK);
	CHECK(HAL_UART_Transmit(&r.h, data, 1, 10) == HAL_BUSY);
	CHECK(hostUartTxCount(0) == 0u);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
}

TEST_CASE_FIXTURE(UartFixture, "UP5 receive and receive to idle while a reception runs return HAL_BUSY") {
	UartPollRig r;
	uartPollInit(r, 0);
	uint8_t run[2] = {0, 0}, buf[2] = {0, 0};
	uint16_t len = 5;
	REQUIRE(HAL_UART_Receive_IT(&r.h, run, 2) == HAL_OK);
	CHECK(HAL_UART_Receive(&r.h, buf, 1, 10) == HAL_BUSY);
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 1, &len, 10) == HAL_BUSY);
	CHECK(len == 5u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
}

TEST_CASE_FIXTURE(UartFixture, "UP5 a polling receive works while a transmit runs (Tx and Rx states are independent)") {
	static const uint8_t data[] = {0x01};
	UartPollRig r;
	uartPollInit(r, 2);
	REQUIRE(HAL_UART_Transmit_IT(&r.h, data, 1) == HAL_OK);
	hostUartRxNow(2, 0x3C, 0);
	uint8_t b = 0;
	CHECK(HAL_UART_Receive(&r.h, &b, 1, 10) == HAL_OK);
	CHECK(b == 0x3C);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UP5 a polling transmit works while a reception runs (Tx and Rx states are independent)") {
	static const uint8_t data[] = {0x5C};
	UartPollRig r;
	uartPollInit(r, 2);
	uint8_t run[2] = {0, 0};
	REQUIRE(HAL_UART_Receive_IT(&r.h, run, 2) == HAL_OK);
	CHECK(HAL_UART_Transmit(&r.h, data, 1, 10) == HAL_OK);
	CHECK(hostUartTxAt(2, 0) == 0x5C);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
}

TEST_CASE_FIXTURE(UartFixture, "UP5 a handle that was never initialised returns HAL_BUSY for all three calls") {
	UART_HandleTypeDef h;
	std::memset(&h, 0, sizeof h);
	h.Instance = USART3;
	uint8_t b[2] = {0, 0};
	uint16_t len = 0;
	CHECK(HAL_UART_Transmit(&h, b, 1, 10) == HAL_BUSY);
	CHECK(HAL_UART_Receive(&h, b, 1, 10) == HAL_BUSY);
	CHECK(HAL_UARTEx_ReceiveToIdle(&h, b, 1, &len, 10) == HAL_BUSY);
	CHECK(hostUartTxCount(2) == 0u);
}

/* ------------------------------------------------------------------------------------------------------------------ UP6 */

namespace {
/* A frame arrives while RXNE = 1 (ORE set), then the application reads DR without reading SR first: RXNE is clear, ORE stays. */
void uartPollMakePendingOre(unsigned idx, UartPollRig &r) {
	hostUartRxNow(idx, 0x11, 0);
	hostUartRxNow(idx, 0x22, 0);
	REQUIRE((uartPollSr(r) & USART_SR_ORE) != 0u);
	volatile uint32_t sink = r.inst()->DR;
	(void)sink;
	REQUIRE((uartPollSr(r) & USART_SR_RXNE) == 0u);
	REQUIRE((uartPollSr(r) & USART_SR_ORE) != 0u);
}
}  // namespace

TEST_CASE_FIXTURE(UartFixture, "UP6 an overrun ends a polling receive early with error code ORE and state READY") {
	UartPollRig r;
	uartPollInit(r, 0);
	uartPollMakePendingOre(0, r);
	uint8_t b = 0;
	UartPollClock clk;
	CHECK(HAL_UART_Receive(&r.h, &b, 1, 10) != HAL_OK);
	CHECK(clk.elapsed() < 10u);
	CHECK(r.h.ErrorCode == HAL_UART_ERROR_ORE);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UP6 an overrun ends a polling receive with HAL_ERROR" * KNOWN_ISSUE("candidate vendor bug V103 (arguable: UM1725 states no return code for an overrun; the intended behaviour is the charter UP6 HAL_ERROR with ErrorCode ORE): HAL_UART_Receive reports an overrun as HAL_TIMEOUT although the call ended well before Timeout. UART_WaitOnFlagUntilTimeout returns HAL_ERROR after setting ErrorCode ORE (stm32f4xx_hal_uart.c:3217) but HAL_UART_Receive maps every non-HAL_OK result to HAL_TIMEOUT (line 1264)")) {
	UartPollRig r;
	uartPollInit(r, 0);
	uartPollMakePendingOre(0, r);
	uint8_t b = 0;
	UartPollClock clk;
	HAL_StatusTypeDef st = HAL_UART_Receive(&r.h, &b, 1, 10);
	REQUIRE(clk.elapsed() < 10u);
	CHECK(st == HAL_ERROR);
}

TEST_CASE_FIXTURE(UartFixture, "UP6 the overrun flag is cleared by the failing receive and the next receive succeeds") {
	UartPollRig r;
	uartPollInit(r, 1);
	uartPollMakePendingOre(1, r);
	uint8_t b = 0;
	REQUIRE(HAL_UART_Receive(&r.h, &b, 1, 10) != HAL_OK);
	CHECK((uartPollSr(r) & USART_SR_ORE) == 0u);
	hostUartRxNow(1, 0x77, 0);
	CHECK(HAL_UART_Receive(&r.h, &b, 1, 10) == HAL_OK);
	CHECK(b == 0x77);
	CHECK(r.h.ErrorCode == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartFixture, "UP6 an overrun leaves no receive interrupt enable set") {
	UartPollRig r;
	uartPollInit(r, 2);
	uartPollMakePendingOre(2, r);
	r.inst()->CR1 |= USART_CR1_RXNEIE | USART_CR1_PEIE;  // as if an IT reception had left them on: UART_EndRxTransfer clears them
	r.inst()->CR3 |= USART_CR3_EIE;
	REQUIRE((HalFixture::peek(&r.inst()->CR1) & (USART_CR1_RXNEIE | USART_CR1_PEIE)) == (USART_CR1_RXNEIE | USART_CR1_PEIE));
	uint8_t b = 0;
	REQUIRE(HAL_UART_Receive(&r.h, &b, 1, 10) != HAL_OK);
	CHECK((HalFixture::peek(&r.inst()->CR1) & (USART_CR1_RXNEIE | USART_CR1_PEIE)) == 0u);
	CHECK((HalFixture::peek(&r.inst()->CR3) & USART_CR3_EIE) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UP6 characterization: a parity error in a polling receive is not reported") {
	// 8 bit even parity: 0x41 has 2 ones, so the frame 0xC1 (3 ones) has the wrong parity.
	UartPollRig r;
	uartPollInit(r, 0, UART_WORDLENGTH_8B, UART_PARITY_EVEN);
	hostUartRxNow(0, 0xC1, 0);
	REQUIRE((uartPollSr(r) & USART_SR_PE) != 0u);
	uint8_t b = 0;
	CHECK(HAL_UART_Receive(&r.h, &b, 1, 10) == HAL_OK);
	CHECK(b == 0x41);
	CHECK(r.h.ErrorCode == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartFixture, "UP6 characterization: a framing error in a polling receive is not reported") {
	UartPollRig r;
	uartPollInit(r, 1);
	hostUartRxNow(1, 0x55, HOST_UART_RX_FE);
	REQUIRE((uartPollSr(r) & USART_SR_FE) != 0u);
	uint8_t b = 0;
	CHECK(HAL_UART_Receive(&r.h, &b, 1, 10) == HAL_OK);
	CHECK(b == 0x55);
	CHECK(r.h.ErrorCode == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartFixture, "UP6 characterization: a noise error in a polling receive is not reported") {
	UartPollRig r;
	uartPollInit(r, 2);
	hostUartRxNow(2, 0xAA, HOST_UART_RX_NOISE);
	REQUIRE((uartPollSr(r) & USART_SR_NE) != 0u);
	uint8_t b = 0;
	CHECK(HAL_UART_Receive(&r.h, &b, 1, 10) == HAL_OK);
	CHECK(b == 0xAA);
	CHECK(r.h.ErrorCode == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartFixture, "UP6 characterization: receive to idle does not look at the overrun flag and times out") {
	UartPollRig r;
	uartPollInit(r, 0);
	uartPollMakePendingOre(0, r);
	uint8_t buf[2] = {0, 0};
	uint16_t len = 9;
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 2, &len, 6) == HAL_TIMEOUT);
	CHECK(r.h.ErrorCode == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartFixture, "UP6 characterization: a HAL_MAX_DELAY receive ignores a pending overrun") {
	UartPollRig r;
	uartPollInit(r, 0);
	uartPollMakePendingOre(0, r);
	hostUartSetPollReads(1);
	hostUartRxPush(0, 0x5E, 0);
	uint8_t b = 0;
	CHECK(HAL_UART_Receive(&r.h, &b, 1, HAL_MAX_DELAY) == HAL_OK);
	CHECK(b == 0x5E);
	CHECK(r.h.ErrorCode == HAL_UART_ERROR_NONE);
}

/* ------------------------------------------------------------------------------------------------------------------ UP7 */

TEST_CASE_FIXTURE(UartFixture, "UP7 receive to idle returns at the idle line with the length received and the IDLE event") {
	static const uint16_t line[] = {0x41, 0x42, 0x43};
	UartPollRig r;
	uartPollInit(r, 0);
	uartPollPush(0, line, 3);
	uartPollArmIdle(0, 8);
	uint8_t buf[8] = {0};
	uint16_t len = 99;
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 8, &len, 100) == HAL_OK);
	CHECK(len == 3u);
	CHECK(buf[0] == 0x41);
	CHECK(buf[1] == 0x42);
	CHECK(buf[2] == 0x43);
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_IDLE);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK((uartPollSr(r) & USART_SR_IDLE) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UP7 receive to idle returns when Size frames arrived with the transfer complete event") {
	static const uint16_t line[] = {0x11, 0x22, 0x33};
	UartPollRig r;
	uartPollInit(r, 1);
	uartPollPush(1, line, 3);
	uint8_t buf[3] = {0, 0, 0};
	uint16_t len = 99;
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 3, &len, 100) == HAL_OK);
	CHECK(len == 3u);
	CHECK(buf[2] == 0x33);
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UP7 receive to idle times out with HAL_TIMEOUT after about Timeout ticks when nothing arrives") {
	UartPollRig r;
	uartPollInit(r, 2);
	uint8_t buf[4] = {0, 0, 0, 0};
	uint16_t len = 99;
	UartPollClock clk;
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 4, &len, 9) == HAL_TIMEOUT);
	uartPollCheckTimeoutBounds(clk.elapsed(), 9);
	CHECK(len == 0u);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UP7 characterization: a receive to idle that times out reports the frames received so far in RxLen") {
	static const uint16_t line[] = {0x51, 0x52};
	UartPollRig r;
	uartPollInit(r, 3);
	uartPollPush(3, line, 2);
	uint8_t buf[4] = {0, 0, 0, 0};
	uint16_t len = 99;
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 4, &len, 10) == HAL_TIMEOUT);
	CHECK(len == 2u);
	CHECK(buf[0] == 0x51);
	CHECK(buf[1] == 0x52);
}

TEST_CASE_FIXTURE(UartFixture, "UP7 an idle line before any frame was received does not end the reception") {
	// IDLE is set by a frame that is then hidden (RXNE written 0); the HAL clears IDLE without ending. 2 frames arrive later (step 3) and end it by Size.
	UartPollRig r;
	uartPollInit(r, 4);
	hostUartRxNow(4, 0x99, 0);
	hostUartIdleLine(4);
	REQUIRE((uartPollSr(r) & USART_SR_IDLE) != 0u);
	r.inst()->SR = ~static_cast<uint32_t>(USART_SR_RXNE);
	REQUIRE((uartPollSr(r) & USART_SR_RXNE) == 0u);
	uartPollStepCount = 0;
	uartPollStepIdx = 4;
	hostTickSetStep(uartPollStepPush);
	uint8_t buf[2] = {0, 0};
	uint16_t len = 99;
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 2, &len, 100) == HAL_OK);
	CHECK(len == 2u);
	CHECK(buf[0] == 0x61);
	CHECK(buf[1] == 0x62);
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
	CHECK((uartPollSr(r) & USART_SR_IDLE) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UP7 the event type of a later call that completes by Size is TC again after an idle ended one") {
	static const uint16_t line[] = {0x71, 0x72};
	UartPollRig r;
	uartPollInit(r, 5);
	uartPollPush(5, line, 2);
	uartPollArmIdle(5, 8);
	uint8_t buf[4] = {0};
	uint16_t len = 0;
	REQUIRE(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 4, &len, 100) == HAL_OK);
	REQUIRE(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_IDLE);
	hostTickSetStep(nullptr);
	uartPollPush(5, line, 2);
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 2, &len, 100) == HAL_OK);
	CHECK(len == 2u);
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
}

TEST_CASE_FIXTURE(UartFixture, "UP7 receive to idle of 9 bit no parity words gives 16 bit values and counts elements") {
	static const uint16_t line[] = {0x1A5, 0x0FF};
	UartPollRig r;
	uartPollInit(r, 0, UART_WORDLENGTH_9B, UART_PARITY_NONE);
	uartPollPush(0, line, 2);
	uartPollArmIdle(0, 8);
	uint16_t buf[6] = {0};
	uint16_t len = 99;
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, reinterpret_cast<uint8_t *>(buf), 6, &len, 100) == HAL_OK);
	CHECK(len == 2u);
	CHECK(buf[0] == 0x1A5);
	CHECK(buf[1] == 0x0FF);
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_IDLE);
}

TEST_CASE_FIXTURE(UartFixture, "UP7 receive to idle of 9 bit words with parity gives the 8 data bits") {
	// Even parity, 9 bit: 0x07 -> 0x107, 0x55 -> 0x055.
	static const uint16_t line[] = {0x107, 0x055};
	UartPollRig r;
	uartPollInit(r, 1, UART_WORDLENGTH_9B, UART_PARITY_EVEN);
	uartPollPush(1, line, 2);
	uint8_t buf[2] = {0, 0};
	uint16_t len = 99;
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 2, &len, 100) == HAL_OK);
	CHECK(len == 2u);
	CHECK(buf[0] == 0x07);
	CHECK(buf[1] == 0x55);
}

TEST_CASE_FIXTURE(UartFixture, "UP7 characterization: receive to idle of 8 bit words with parity gives the 7 data bits") {
	// Odd parity: data 0x41 -> 0xC1, data 0x7F -> 0x7F.
	static const uint16_t line[] = {0xC1, 0x7F};
	UartPollRig r;
	uartPollInit(r, 2, UART_WORDLENGTH_8B, UART_PARITY_ODD);
	uartPollPush(2, line, 2);
	uint8_t buf[2] = {0, 0};
	uint16_t len = 99;
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 2, &len, 100) == HAL_OK);
	CHECK(buf[0] == 0x41);
	CHECK(buf[1] == 0x7F);
}

TEST_CASE_FIXTURE(UartFixture, "UP7 receive to idle with HAL_MAX_DELAY completes by Size") {
	static const uint16_t line[] = {0x81, 0x82};
	UartPollRig r;
	uartPollInit(r, 3);
	hostUartSetFrameTicks(3);
	hostUartSetPollReads(1);
	uartPollPush(3, line, 2);
	uint8_t buf[2] = {0, 0};
	uint16_t len = 99;
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 2, &len, HAL_MAX_DELAY) == HAL_OK);
	CHECK(len == 2u);
	CHECK(buf[1] == 0x82);
}

/* ------------------------------------------------------------------------------------------------------------------ UP8 */

TEST_CASE_FIXTURE(UartFixture, "UP8 frames sent back to back keep their order for Size 1, 2, 3 and 16 with 1 and 3 ticks per frame") {
	static const unsigned sizes[] = {1, 2, 3, 16};
	static const unsigned ticks[] = {1, 3};
	for (unsigned s = 0; s < 4; ++s) {
		for (unsigned t = 0; t < 2; ++t) {
			INFO("size " << sizes[s] << " ticks " << ticks[t]);
			UartPollRig r;
			uartPollInit(r, 2);
			hostUartSetFrameTicks(ticks[t]);
			uint8_t data[16];
			uint16_t exp[16];
			for (unsigned k = 0; k < sizes[s]; ++k) {
				data[k] = static_cast<uint8_t>(0x30 + 7 * k);  // 0x30, 0x37, 0x3E, ...: distinct bytes
				exp[k] = data[k];
			}
			REQUIRE(HAL_UART_Transmit(&r.h, data, static_cast<uint16_t>(sizes[s]), 500) == HAL_OK);
			uartPollCheckLine(2, exp, sizes[s]);
			CHECK((uartPollSr(r) & USART_SR_TC) != 0u);
			CHECK(hostUartDisabledBusy(2) == 0u);
			hostUartTxClear(2);
		}
	}
}

TEST_CASE_FIXTURE(UartFixture, "UP8 a second transmit after a first one appends its frames after the first ones") {
	static const uint8_t a[] = {0x01, 0x02};
	static const uint8_t b[] = {0x03, 0x04, 0x05};
	static const uint16_t exp[] = {0x01, 0x02, 0x03, 0x04, 0x05};
	UartPollRig r;
	uartPollInit(r, 0);
	hostUartSetFrameTicks(2);
	REQUIRE(HAL_UART_Transmit(&r.h, a, 2, 100) == HAL_OK);
	REQUIRE(HAL_UART_Transmit(&r.h, b, 3, 100) == HAL_OK);
	uartPollCheckLine(0, exp, 5);
}

/* ---- fix pass additions ---- */

TEST_CASE_FIXTURE(UartFixture, "UP6 a polling transmit is not affected by a pending overrun") {
	static const uint8_t data[] = {0x12, 0x34};
	UartPollRig r;
	uartPollInit(r, 0);
	uartPollMakePendingOre(0, r);
	REQUIRE((HalFixture::peek(&r.inst()->CR1) & USART_CR1_RE) != 0u);
	hostUartSetFrameTicks(3);  // the TXE and TC waits loop several times
	CHECK(HAL_UART_Transmit(&r.h, data, 2, 100) == HAL_OK);
	CHECK(hostUartTxCount(0) == 2u);
	CHECK(r.h.ErrorCode == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartFixture, "UP4 transmit timeout bounds hold with the tick counter at a large value") {
	static const uint8_t data[] = {0x5A};
	UartPollRig r;
	uartPollInit(r, 0);
	hostTickSet(0x80000000u);
	hostUartTxStuck(0, 1);
	UartPollClock clk;
	CHECK(HAL_UART_Transmit(&r.h, data, 1, 10) == HAL_TIMEOUT);
	uartPollCheckTimeoutBounds(clk.elapsed(), 10);
}

TEST_CASE_FIXTURE(UartFixture, "UP4 transmit timeout bounds hold across the wrap-around of the tick counter") {
	static const uint8_t data[] = {0x5A};
	UartPollRig r;
	uartPollInit(r, 1);
	hostTickSet(0xFFFFFFF8u);  // wraps after 8 ticks, inside the Timeout of 20
	hostUartTxStuck(1, 1);
	UartPollClock clk;
	CHECK(HAL_UART_Transmit(&r.h, data, 1, 20) == HAL_TIMEOUT);
	uartPollCheckTimeoutBounds(clk.elapsed(), 20);
}

TEST_CASE_FIXTURE(UartFixture, "UP4 receive timeout bounds hold across the wrap-around of the tick counter") {
	UartPollRig r;
	uartPollInit(r, 2);
	hostTickSet(0xFFFFFFF8u);
	uint8_t b = 0;
	UartPollClock clk;
	CHECK(HAL_UART_Receive(&r.h, &b, 1, 20) == HAL_TIMEOUT);
	uartPollCheckTimeoutBounds(clk.elapsed(), 20);
}

TEST_CASE_FIXTURE(UartFixture, "UP7 receive to idle timeout bounds hold across the wrap-around of the tick counter") {
	UartPollRig r;
	uartPollInit(r, 3);
	hostTickSet(0xFFFFFFF8u);
	uint8_t buf[2] = {0, 0};
	uint16_t len = 99;
	UartPollClock clk;
	CHECK(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 2, &len, 20) == HAL_TIMEOUT);
	uartPollCheckTimeoutBounds(clk.elapsed(), 20);
}

TEST_CASE_FIXTURE(UartFixture, "UP3 the Rx state is BUSY_RX while a polling receive runs and READY afterwards") {
	static const uint16_t line[] = {0x01, 0x02};
	UartPollRig r;
	uartPollInit(r, 1);
	hostUartSetFrameTicks(3);
	uartPollPush(1, line, 2);
	uartPollStepHandle = &r.h;
	uartPollSawBusyRx = false;
	hostTickSetStep(uartPollStepWatchRx);
	uint8_t buf[2] = {0, 0};
	REQUIRE(HAL_UART_Receive(&r.h, buf, 2, 100) == HAL_OK);
	CHECK(uartPollSawBusyRx);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UP7 the Rx state is BUSY_RX while a receive to idle runs and READY afterwards") {
	static const uint16_t line[] = {0x01, 0x02};
	UartPollRig r;
	uartPollInit(r, 1);
	hostUartSetFrameTicks(3);
	uartPollPush(1, line, 2);
	uartPollStepHandle = &r.h;
	uartPollSawBusyRx = false;
	hostTickSetStep(uartPollStepWatchRx);
	uint8_t buf[2] = {0, 0};
	uint16_t len = 0;
	REQUIRE(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 2, &len, 100) == HAL_OK);
	CHECK(uartPollSawBusyRx);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UP7 characterization: receive to idle clears a stale error code at its start") {
	static const uint16_t line[] = {0x01, 0x02};
	UartPollRig r;
	uartPollInit(r, 0);
	r.h.ErrorCode = HAL_UART_ERROR_FE;
	uartPollPush(0, line, 2);
	uint8_t buf[2] = {0, 0};
	uint16_t len = 0;
	REQUIRE(HAL_UARTEx_ReceiveToIdle(&r.h, buf, 2, &len, 100) == HAL_OK);
	CHECK(r.h.ErrorCode == HAL_UART_ERROR_NONE);
}
