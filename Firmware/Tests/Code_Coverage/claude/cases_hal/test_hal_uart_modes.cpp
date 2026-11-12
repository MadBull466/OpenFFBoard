/*
 * P12 T4b group UM: UART special modes (HAL_HalfDuplex_Init / EnableTransmitter / EnableReceiver, HAL_LIN_Init / HAL_LIN_SendBreak,
 * HAL_MultiProcessor_Init / EnterMuteMode / ExitMuteMode) of stm32f4xx_hal_uart.c against the UART register model
 * (fakes/stm32_hal/HalUart.h). Requirements UM1-UM4 of Tests/Code_Coverage/claude/charters/hal/P12_T4B_REQUIREMENTS.md; compiled into
 * ffb_hal_tests and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 (HAL UART Generic Driver: the three Init functions, the helper functions of "UART_Exported_Functions_Group3",
 * state and handle names) and RM0090 Rev 22 chapter 30: 30.3.6 (multiprocessor: idle line and address mark wake-up), 30.3.8 (LIN, SBK
 * sends a break and is reset by hardware at the end of it), 30.3.10 / 30.6.4 (HDSEL, LINEN, CLKEN, SCEN, IREN, OVER8 forced to 0 in LIN
 * mode), 30.6.1 (SR), 30.6.3 / 30.6.4 / 30.6.5 (CR1, CR2 ADD / LBDL / LINEN, CR3 HDSEL).
 * The tests run on all six instances (USART1, USART2, USART3, UART4, UART5, USART6) where the behaviour is per instance.
 *
 * NEEDS REVIEW proposals (characterization cases; the lead assigns the numbers):
 *  NEEDS REVIEW 314 characterization: the helper functions (Enable* of the half duplex, SendBreak, Enter / ExitMuteMode, the three Init
 *            functions) do not look at the state: called while a transmit is running (BUSY_TX) they return HAL_OK and leave gState READY,
 *            so the running transfer's state is lost. UM1725 only says what each function enables.
 *  NEEDS REVIEW 314 characterization: a handle with Lock == HAL_LOCKED makes Enable*, SendBreak, Enter / ExitMuteMode return HAL_BUSY without
 *            touching a register (the __HAL_LOCK macro). UM1725 only states "HAL status".
 *  NEEDS REVIEW 314 characterization: EnterMuteMode / ExitMuteMode set RxEventType to HAL_UART_RXEVENT_TC (UM1725 is silent).
 *  NEEDS REVIEW 314 characterization: the Init functions do not look at the previous state: with gState BUSY_TX they reinitialise and leave
 *            both states READY and the error code NONE.
 */
#include "HalFixture.h"
#include "HalUart.h"
#include "TestHelpers.h"
#include "UartTestSupport.h"

#include <cstdint>
#include <cstring>

namespace {

enum UartModesKind { UM_HD, UM_LIN10, UM_LIN11, UM_MP_IDLE, UM_MP_ADDR };
const unsigned uartModesKindCount = 5;

USART_TypeDef *uartModesInstance(unsigned i) {
	switch (i) {
	case 0: return USART1;
	case 1: return USART2;
	case 2: return USART3;
	case 3: return UART4;
	case 4: return UART5;
	default: return USART6;
	}
}

struct UartModesRig {
	UART_HandleTypeDef h;
	unsigned idx;
	USART_TypeDef *u() { return h.Instance; }
};

/* The handle is prepared, not initialised (state RESET). */
void uartModesPrepare(UartModesRig &r, unsigned idx, uint32_t word = UART_WORDLENGTH_8B, uint32_t parity = UART_PARITY_NONE,
                      uint32_t mode = UART_MODE_TX_RX, uint32_t over = UART_OVERSAMPLING_16) {
	std::memset(&r.h, 0, sizeof r.h);
	r.idx = idx;
	r.h.Instance = uartModesInstance(idx);
	r.h.Init.BaudRate = 115200;
	r.h.Init.WordLength = word;
	r.h.Init.StopBits = UART_STOPBITS_1;
	r.h.Init.Parity = parity;
	r.h.Init.Mode = mode;
	r.h.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	r.h.Init.OverSampling = over;
}

HAL_StatusTypeDef uartModesCall(UART_HandleTypeDef *h, UartModesKind k, uint8_t addr = 5) {
	switch (k) {
	case UM_HD: return HAL_HalfDuplex_Init(h);
	case UM_LIN10: return HAL_LIN_Init(h, UART_LINBREAKDETECTLENGTH_10B);
	case UM_LIN11: return HAL_LIN_Init(h, UART_LINBREAKDETECTLENGTH_11B);
	case UM_MP_IDLE: return HAL_MultiProcessor_Init(h, addr, UART_WAKEUPMETHOD_IDLELINE);
	default: return HAL_MultiProcessor_Init(h, addr, UART_WAKEUPMETHOD_ADDRESSMARK);
	}
}

void uartModesInit(UartModesRig &r, unsigned idx, UartModesKind k, uint8_t addr = 5, uint32_t word = UART_WORDLENGTH_8B,
                   uint32_t parity = UART_PARITY_NONE, uint32_t mode = UART_MODE_TX_RX) {
	uartModesPrepare(r, idx, word, parity, mode);
	REQUIRE(uartModesCall(&r.h, k, addr) == HAL_OK);
	hostUartTxClear(idx);
}

/* Every bit that a mode Init must clear or overwrite is set in advance (UE is 0 after reset, so the writes are legal). */
void uartModesDirty(USART_TypeDef *u) {
	u->CR2 |= USART_CR2_LINEN | USART_CR2_CLKEN | USART_CR2_LBDL | USART_CR2_ADD;
	u->CR3 |= USART_CR3_HDSEL | USART_CR3_IREN | USART_CR3_SCEN;
	u->CR1 |= USART_CR1_WAKE;
}

const uint32_t uartModesCr2Off = USART_CR2_LINEN | USART_CR2_CLKEN;
const uint32_t uartModesCr3Off = USART_CR3_SCEN | USART_CR3_IREN;

/* Advance the host tick and let the model evaluate (it reacts to a read of SR). */
void uartModesTickTo(USART_TypeDef *u, uint32_t tick) {
	hostTickSet(tick);
	(void)u->SR;
}

/* Receive one frame register-wise: true if RXNE is set, then reads DR. */
bool uartModesTakeFrame(USART_TypeDef *u, uint32_t &data) {
	if ((u->SR & USART_SR_RXNE) == 0u) return false;
	data = u->DR;
	return true;
}

uint8_t uartModesBuf[8];

} // namespace

/* ---------------------------------------------------------------- UM1 half duplex */

TEST_CASE_FIXTURE(UartFixture, "UM1 HAL_HalfDuplex_Init sets HDSEL on every instance") {
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesPrepare(r, i);
		CHECK((r.u()->CR3 & USART_CR3_HDSEL) == 0u);
		REQUIRE(HAL_HalfDuplex_Init(&r.h) == HAL_OK);
		CHECK((r.u()->CR3 & USART_CR3_HDSEL) == USART_CR3_HDSEL);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM1 HAL_HalfDuplex_Init clears LINEN CLKEN SCEN and IREN") {
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesPrepare(r, i);
		uartModesDirty(r.u());
		REQUIRE(HAL_HalfDuplex_Init(&r.h) == HAL_OK);
		CHECK((r.u()->CR2 & uartModesCr2Off) == 0u);
		CHECK((r.u()->CR3 & uartModesCr3Off) == 0u);
		CHECK((r.u()->CR3 & USART_CR3_HDSEL) == USART_CR3_HDSEL);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM1 HAL_HalfDuplex_Init enables the USART with the direction of the init structure") {
	const uint32_t modes[3] = {UART_MODE_TX, UART_MODE_RX, UART_MODE_TX_RX};
	for (unsigned i : {1u, 3u}) { /* USART2 and UART4 */
		for (uint32_t m : modes) {
			INFO("instance " << i << " mode " << m);
			UartModesRig r;
			uartModesPrepare(r, i, UART_WORDLENGTH_8B, UART_PARITY_NONE, m);
			REQUIRE(HAL_HalfDuplex_Init(&r.h) == HAL_OK);
			const uint32_t cr1 = r.u()->CR1;
			CHECK((cr1 & USART_CR1_UE) == USART_CR1_UE);
			CHECK((cr1 & (USART_CR1_TE | USART_CR1_RE)) == m);
		}
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM1 HAL_HalfDuplex_EnableTransmitter sets TE and clears RE") {
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesInit(r, i, UM_HD, 5, UART_WORDLENGTH_8B, UART_PARITY_NONE, UART_MODE_RX);
		CHECK((r.u()->CR1 & (USART_CR1_TE | USART_CR1_RE)) == USART_CR1_RE);
		CHECK(HAL_HalfDuplex_EnableTransmitter(&r.h) == HAL_OK);
		CHECK((r.u()->CR1 & (USART_CR1_TE | USART_CR1_RE)) == USART_CR1_TE);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM1 HAL_HalfDuplex_EnableReceiver sets RE and clears TE") {
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesInit(r, i, UM_HD, 5, UART_WORDLENGTH_8B, UART_PARITY_NONE, UART_MODE_TX);
		CHECK((r.u()->CR1 & (USART_CR1_TE | USART_CR1_RE)) == USART_CR1_TE);
		CHECK(HAL_HalfDuplex_EnableReceiver(&r.h) == HAL_OK);
		CHECK((r.u()->CR1 & (USART_CR1_TE | USART_CR1_RE)) == USART_CR1_RE);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM1 switching the half duplex direction twice returns to the first direction") {
	UartModesRig r;
	uartModesInit(r, 2, UM_HD);
	REQUIRE(HAL_HalfDuplex_EnableTransmitter(&r.h) == HAL_OK);
	REQUIRE(HAL_HalfDuplex_EnableReceiver(&r.h) == HAL_OK);
	REQUIRE(HAL_HalfDuplex_EnableTransmitter(&r.h) == HAL_OK);
	CHECK((r.u()->CR1 & (USART_CR1_TE | USART_CR1_RE)) == USART_CR1_TE);
	CHECK(HAL_HalfDuplex_EnableTransmitter(&r.h) == HAL_OK); /* idempotent */
	CHECK((r.u()->CR1 & (USART_CR1_TE | USART_CR1_RE)) == USART_CR1_TE);
}

TEST_CASE_FIXTURE(UartFixture, "UM1 HAL_HalfDuplex_Enable* keep UE and the word format bits") {
	for (unsigned i : {0u, 4u}) { /* USART1 and UART5 */
		INFO("instance " << i);
		UartModesRig r;
		uartModesInit(r, i, UM_HD, 5, UART_WORDLENGTH_9B, UART_PARITY_ODD);
		const uint32_t fmt = USART_CR1_UE | USART_CR1_M | USART_CR1_PCE | USART_CR1_PS;
		REQUIRE((r.u()->CR1 & fmt) == fmt);
		REQUIRE(HAL_HalfDuplex_EnableReceiver(&r.h) == HAL_OK);
		CHECK((r.u()->CR1 & fmt) == fmt);
		REQUIRE(HAL_HalfDuplex_EnableTransmitter(&r.h) == HAL_OK);
		CHECK((r.u()->CR1 & fmt) == fmt);
		CHECK(hostUartConfigWhileEnabled(i) == 0u);
		CHECK(hostUartDisabledBusy(i) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM1 data of a transmit in half duplex appears on the line in order") {
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesInit(r, i, UM_HD);
		REQUIRE(HAL_HalfDuplex_EnableTransmitter(&r.h) == HAL_OK);
		const uint8_t data[3] = {0x3C, 0xA5, 0x00};
		REQUIRE(HAL_UART_Transmit(&r.h, const_cast<uint8_t *>(data), 3, 100) == HAL_OK);
		REQUIRE(hostUartTxCount(i) == 3u);
		CHECK(hostUartTxAt(i, 0) == 0x3Cu);
		CHECK(hostUartTxAt(i, 1) == 0xA5u);
		CHECK(hostUartTxAt(i, 2) == 0x00u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM1 a receive in half duplex after EnableReceiver returns the frames of the peer") {
	for (unsigned i : {1u, 3u}) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesInit(r, i, UM_HD);
		REQUIRE(HAL_HalfDuplex_EnableReceiver(&r.h) == HAL_OK);
		hostUartRxPush(i, 0x31, 0);
		hostUartRxPush(i, 0xC2, 0);
		std::memset(uartModesBuf, 0, sizeof uartModesBuf);
		REQUIRE(HAL_UART_Receive(&r.h, uartModesBuf, 2, 100) == HAL_OK);
		CHECK(uartModesBuf[0] == 0x31);
		CHECK(uartModesBuf[1] == 0xC2);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM1 no frame goes on the line while the half duplex receiver is selected") {
	UartModesRig r;
	uartModesInit(r, 2, UM_HD);
	REQUIRE(HAL_HalfDuplex_EnableReceiver(&r.h) == HAL_OK);
	uartModesBuf[0] = 0x55;
	/* RM0090 30.3.2: the transmitter sends only with TE = 1; the write to DR stays in TDR and TC never comes. */
	CHECK(HAL_UART_Transmit(&r.h, uartModesBuf, 1, 5) == HAL_TIMEOUT);
	CHECK(hostUartTxCount(2) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UM1 characterization: the half duplex helpers run while a transmit is active and leave the state READY") {
	/* NEEDS REVIEW 314 */
	UartModesRig r;
	uartModesInit(r, 1, UM_HD);
	REQUIRE(HAL_HalfDuplex_EnableTransmitter(&r.h) == HAL_OK);
	uartModesBuf[0] = 1;
	uartModesBuf[1] = 2;
	REQUIRE(HAL_UART_Transmit_IT(&r.h, uartModesBuf, 2) == HAL_OK);
	REQUIRE(r.h.gState == HAL_UART_STATE_BUSY_TX);
	CHECK(HAL_HalfDuplex_EnableReceiver(&r.h) == HAL_OK);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK((r.u()->CR1 & (USART_CR1_TE | USART_CR1_RE)) == USART_CR1_RE);
}

TEST_CASE_FIXTURE(UartFixture, "UM1 characterization: the half duplex helpers return HAL_BUSY for a locked handle and change nothing") {
	/* NEEDS REVIEW 314 */
	UartModesRig r;
	uartModesInit(r, 1, UM_HD);
	const uint32_t before = r.u()->CR1;
	r.h.Lock = HAL_LOCKED;
	CHECK(HAL_HalfDuplex_EnableTransmitter(&r.h) == HAL_BUSY);
	CHECK(HAL_HalfDuplex_EnableReceiver(&r.h) == HAL_BUSY);
	CHECK(r.u()->CR1 == before);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK(r.h.Lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(UartFixture, "UM1 the half duplex helpers leave the handle unlocked after success") {
	UartModesRig r;
	uartModesInit(r, 1, UM_HD);
	REQUIRE(HAL_HalfDuplex_EnableTransmitter(&r.h) == HAL_OK);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	REQUIRE(HAL_HalfDuplex_EnableReceiver(&r.h) == HAL_OK);
	CHECK(r.h.Lock == HAL_UNLOCKED);
}

/* ---------------------------------------------------------------- UM2 LIN */

TEST_CASE_FIXTURE(UartFixture, "UM2 HAL_LIN_Init sets LINEN on every instance") {
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesPrepare(r, i);
		CHECK((r.u()->CR2 & USART_CR2_LINEN) == 0u);
		REQUIRE(HAL_LIN_Init(&r.h, UART_LINBREAKDETECTLENGTH_10B) == HAL_OK);
		CHECK((r.u()->CR2 & USART_CR2_LINEN) == USART_CR2_LINEN);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM2 HAL_LIN_Init with the 10 bit break length clears LBDL") {
	for (unsigned i : {0u, 3u}) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesPrepare(r, i);
		uartModesDirty(r.u()); /* LBDL = 1 in advance */
		REQUIRE((r.u()->CR2 & USART_CR2_LBDL) == USART_CR2_LBDL);
		REQUIRE(HAL_LIN_Init(&r.h, UART_LINBREAKDETECTLENGTH_10B) == HAL_OK);
		CHECK((r.u()->CR2 & USART_CR2_LBDL) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM2 HAL_LIN_Init with the 11 bit break length sets LBDL") {
	for (unsigned i : {0u, 3u}) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesPrepare(r, i);
		REQUIRE((r.u()->CR2 & USART_CR2_LBDL) == 0u);
		REQUIRE(HAL_LIN_Init(&r.h, UART_LINBREAKDETECTLENGTH_11B) == HAL_OK);
		CHECK((r.u()->CR2 & USART_CR2_LBDL) == USART_CR2_LBDL);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM2 HAL_LIN_Init clears HDSEL CLKEN SCEN and IREN") {
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesPrepare(r, i);
		uartModesDirty(r.u());
		REQUIRE(HAL_LIN_Init(&r.h, UART_LINBREAKDETECTLENGTH_10B) == HAL_OK);
		CHECK((r.u()->CR3 & (USART_CR3_HDSEL | uartModesCr3Off)) == 0u);
		CHECK((r.u()->CR2 & USART_CR2_CLKEN) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM2 OVER8 reads 0 after HAL_LIN_Init even when oversampling by 8 was asked") {
	/* RM0090 30.6.4 / 30.3.3: in LIN mode the OVER8 bit is forced to 0 by hardware. */
	for (unsigned i : {2u, 4u}) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesPrepare(r, i, UART_WORDLENGTH_8B, UART_PARITY_NONE, UART_MODE_TX_RX, UART_OVERSAMPLING_8);
		REQUIRE(HAL_LIN_Init(&r.h, UART_LINBREAKDETECTLENGTH_10B) == HAL_OK);
		CHECK((r.u()->CR1 & USART_CR1_OVER8) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM2 HAL_LIN_Init enables the USART and keeps the configured direction") {
	UartModesRig r;
	uartModesPrepare(r, 1, UART_WORDLENGTH_8B, UART_PARITY_NONE, UART_MODE_TX);
	REQUIRE(HAL_LIN_Init(&r.h, UART_LINBREAKDETECTLENGTH_11B) == HAL_OK);
	CHECK((r.u()->CR1 & (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE)) == (USART_CR1_UE | USART_CR1_TE));
}

TEST_CASE_FIXTURE(UartFixture, "UM2 HAL_LIN_SendBreak sets SBK and the model sends a break") {
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		hostUartSetFrameTicks(4);
		UartModesRig r;
		uartModesInit(r, i, UM_LIN10);
		CHECK(HAL_LIN_SendBreak(&r.h) == HAL_OK);
		CHECK((r.u()->CR1 & USART_CR1_SBK) == USART_CR1_SBK);
		CHECK(hostUartBreakCount(i) == 1u);
		CHECK(hostUartTxCount(i) == 0u); /* a break is not a data frame */
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(r.h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM2 SBK stays set during the break and clears at its end") {
	/* RM0090 30.3.2: SBK is reset by hardware when the break character is completed. */
	hostUartSetFrameTicks(4);
	UartModesRig r;
	uartModesInit(r, 3, UM_LIN11);
	REQUIRE(HAL_LIN_SendBreak(&r.h) == HAL_OK);
	uartModesTickTo(r.u(), 3);
	CHECK((r.u()->CR1 & USART_CR1_SBK) == USART_CR1_SBK);
	uartModesTickTo(r.u(), 4);
	CHECK((r.u()->CR1 & USART_CR1_SBK) == 0u);
	CHECK(hostUartBreakCount(3) == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UM2 a second break after the first has ended is sent too") {
	hostUartSetFrameTicks(2);
	UartModesRig r;
	uartModesInit(r, 1, UM_LIN10);
	REQUIRE(HAL_LIN_SendBreak(&r.h) == HAL_OK);
	uartModesTickTo(r.u(), 2);
	REQUIRE((r.u()->CR1 & USART_CR1_SBK) == 0u);
	CHECK(HAL_LIN_SendBreak(&r.h) == HAL_OK);
	CHECK((r.u()->CR1 & USART_CR1_SBK) == USART_CR1_SBK);
	CHECK(hostUartBreakCount(1) == 2u);
}

TEST_CASE_FIXTURE(UartFixture, "UM2 a break waits for the data frame in progress and goes after it") {
	/* RM0090 30.3.2: the break character is sent after completing the current character transmission. */
	hostUartSetFrameTicks(4);
	UartModesRig r;
	uartModesInit(r, 1, UM_LIN10);
	r.u()->DR = 0x66; /* a data frame starts (written straight to DR) */
	hostUartTxClear(1);
	REQUIRE(HAL_LIN_SendBreak(&r.h) == HAL_OK);
	CHECK(hostUartBreakCount(1) == 0u); /* the data frame still runs */
	uartModesTickTo(r.u(), 4);
	CHECK(hostUartBreakCount(1) == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UM2 characterization: HAL_LIN_SendBreak returns HAL_BUSY for a locked handle and sends nothing") {
	/* NEEDS REVIEW 314 */
	UartModesRig r;
	uartModesInit(r, 1, UM_LIN10);
	r.h.Lock = HAL_LOCKED;
	CHECK(HAL_LIN_SendBreak(&r.h) == HAL_BUSY);
	CHECK((r.u()->CR1 & USART_CR1_SBK) == 0u);
	CHECK(hostUartBreakCount(1) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UM2 characterization: HAL_LIN_SendBreak while a transmit is active leaves the state READY") {
	/* NEEDS REVIEW 314 */
	UartModesRig r;
	uartModesInit(r, 1, UM_LIN10);
	uartModesBuf[0] = 7;
	REQUIRE(HAL_UART_Transmit_IT(&r.h, uartModesBuf, 1) == HAL_OK);
	REQUIRE(r.h.gState == HAL_UART_STATE_BUSY_TX);
	CHECK(HAL_LIN_SendBreak(&r.h) == HAL_OK);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
}

/* ---------------------------------------------------------------- UM3 multiprocessor */

TEST_CASE_FIXTURE(UartFixture, "UM3 HAL_MultiProcessor_Init stores the node address in ADD") {
	const uint8_t addrs[4] = {0, 1, 5, 0xF};
	for (unsigned i : {0u, 3u}) {
		for (uint8_t a : addrs) {
			INFO("instance " << i << " address " << unsigned(a));
			UartModesRig r;
			uartModesPrepare(r, i);
			r.u()->CR2 |= USART_CR2_ADD; /* ADD = 0xF in advance */
			REQUIRE(HAL_MultiProcessor_Init(&r.h, a, UART_WAKEUPMETHOD_IDLELINE) == HAL_OK);
			CHECK((r.u()->CR2 & USART_CR2_ADD) == a);
		}
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM3 HAL_MultiProcessor_Init selects the idle line wake-up by clearing WAKE") {
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesPrepare(r, i);
		uartModesDirty(r.u()); /* WAKE = 1 in advance */
		REQUIRE(HAL_MultiProcessor_Init(&r.h, 5, UART_WAKEUPMETHOD_IDLELINE) == HAL_OK);
		CHECK((r.u()->CR1 & USART_CR1_WAKE) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM3 HAL_MultiProcessor_Init selects the address mark wake-up by setting WAKE") {
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesPrepare(r, i);
		REQUIRE((r.u()->CR1 & USART_CR1_WAKE) == 0u);
		REQUIRE(HAL_MultiProcessor_Init(&r.h, 5, UART_WAKEUPMETHOD_ADDRESSMARK) == HAL_OK);
		CHECK((r.u()->CR1 & USART_CR1_WAKE) == USART_CR1_WAKE);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM3 HAL_MultiProcessor_Init clears LINEN CLKEN HDSEL SCEN and IREN") {
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesPrepare(r, i);
		uartModesDirty(r.u());
		REQUIRE(HAL_MultiProcessor_Init(&r.h, 5, UART_WAKEUPMETHOD_ADDRESSMARK) == HAL_OK);
		CHECK((r.u()->CR2 & uartModesCr2Off) == 0u);
		CHECK((r.u()->CR3 & (USART_CR3_HDSEL | uartModesCr3Off)) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM3 HAL_MultiProcessor_Init enables the USART") {
	UartModesRig r;
	uartModesPrepare(r, 4);
	REQUIRE(HAL_MultiProcessor_Init(&r.h, 5, UART_WAKEUPMETHOD_IDLELINE) == HAL_OK);
	CHECK((r.u()->CR1 & USART_CR1_UE) == USART_CR1_UE);
}

TEST_CASE_FIXTURE(UartFixture, "UM3 EnterMuteMode sets RWU and ExitMuteMode clears it") {
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesInit(r, i, UM_MP_IDLE);
		CHECK((r.u()->CR1 & USART_CR1_RWU) == 0u);
		CHECK(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
		CHECK((r.u()->CR1 & USART_CR1_RWU) == USART_CR1_RWU);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(HAL_MultiProcessor_ExitMuteMode(&r.h) == HAL_OK);
		CHECK((r.u()->CR1 & USART_CR1_RWU) == 0u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM3 characterization: Enter and ExitMuteMode set the Rx event type to TC") {
	/* NEEDS REVIEW 314 */
	UartModesRig r;
	uartModesInit(r, 1, UM_MP_IDLE);
	r.h.RxEventType = HAL_UART_RXEVENT_IDLE;
	REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
	r.h.RxEventType = HAL_UART_RXEVENT_IDLE;
	REQUIRE(HAL_MultiProcessor_ExitMuteMode(&r.h) == HAL_OK);
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
}

TEST_CASE_FIXTURE(UartFixture, "UM3 in idle line mute mode received frames are dropped and no status bit is set") {
	for (unsigned i : {1u, 4u}) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesInit(r, i, UM_MP_IDLE);
		REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
		hostUartRxNow(i, 0x41, 0);
		hostUartRxNow(i, 0x42, HOST_UART_RX_FE);
		const uint32_t sr = r.u()->SR;
		CHECK((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE | USART_SR_IDLE)) == 0u);
		CHECK((r.u()->CR1 & USART_CR1_RWU) == USART_CR1_RWU);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM3 in idle line mute mode an idle frame clears RWU and does not set IDLE") {
	for (unsigned i : {2u, 3u}) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesInit(r, i, UM_MP_IDLE);
		REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
		hostUartIdleLine(i);
		CHECK((r.u()->CR1 & USART_CR1_RWU) == 0u);
		CHECK((r.u()->SR & USART_SR_IDLE) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM3 after the idle line wake-up frames are received normally") {
	for (unsigned i : {2u, 3u}) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesInit(r, i, UM_MP_IDLE);
		REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
		hostUartRxNow(i, 0x11, 0); /* dropped */
		hostUartIdleLine(i);
		hostUartRxNow(i, 0x22, 0);
		uint32_t d = 0;
		REQUIRE(uartModesTakeFrame(r.u(), d));
		CHECK(d == 0x22u);
		CHECK(!uartModesTakeFrame(r.u(), d));
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM3 ExitMuteMode wakes up an idle line receiver by software") {
	UartModesRig r;
	uartModesInit(r, 0, UM_MP_IDLE);
	REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
	hostUartRxNow(0, 0x11, 0);
	REQUIRE((r.u()->SR & USART_SR_RXNE) == 0u);
	REQUIRE(HAL_MultiProcessor_ExitMuteMode(&r.h) == HAL_OK);
	hostUartRxNow(0, 0x33, 0);
	uint32_t d = 0;
	REQUIRE(uartModesTakeFrame(r.u(), d));
	CHECK(d == 0x33u);
}

TEST_CASE_FIXTURE(UartFixture, "UM3 in mute mode a frame sets no ORE although the receive register is full") {
	/* RM0090 30.3.6: in mute mode none of the reception status bits can be set. */
	UartModesRig r;
	uartModesInit(r, 1, UM_MP_IDLE);
	hostUartRxNow(1, 0x11, 0);
	REQUIRE((r.u()->SR & USART_SR_RXNE) == USART_SR_RXNE);
	REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
	CHECK((r.u()->CR1 & USART_CR1_RWU) == USART_CR1_RWU); /* idle line mode: RWU can be written with RXNE = 1 */
	hostUartRxNow(1, 0x22, 0);
	CHECK((r.u()->SR & USART_SR_ORE) == 0u);
	CHECK(r.u()->DR == 0x11u); /* the frame in the register is kept */
}

TEST_CASE_FIXTURE(UartFixture, "UM3 in address mark mode a non matching address is dropped and RWU stays set") {
	for (unsigned i : {1u, 4u}) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesInit(r, i, UM_MP_ADDR, 5);
		REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
		hostUartRxNow(i, 0x83, 0); /* address mark (MSB 1), node 3 != 5 */
		CHECK((r.u()->SR & USART_SR_RXNE) == 0u);
		CHECK((r.u()->CR1 & USART_CR1_RWU) == USART_CR1_RWU);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM3 in address mark mode the matching address wakes up and is received") {
	for (unsigned i : {1u, 4u}) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesInit(r, i, UM_MP_ADDR, 5);
		REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
		hostUartRxNow(i, 0x85, 0);
		CHECK((r.u()->CR1 & USART_CR1_RWU) == 0u);
		uint32_t d = 0;
		REQUIRE(uartModesTakeFrame(r.u(), d));
		CHECK(d == 0x85u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM3 in address mark mode a data frame with the node bits of this address is dropped") {
	UartModesRig r;
	uartModesInit(r, 2, UM_MP_ADDR, 5);
	REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
	hostUartRxNow(2, 0x05, 0); /* MSB 0: data, not an address */
	CHECK((r.u()->SR & USART_SR_RXNE) == 0u);
	CHECK((r.u()->CR1 & USART_CR1_RWU) == USART_CR1_RWU);
}

TEST_CASE_FIXTURE(UartFixture, "UM3 in address mark mode the data after a matching address is received and after a foreign address dropped") {
	UartModesRig r;
	uartModesInit(r, 3, UM_MP_ADDR, 5);
	REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
	uint32_t d = 0;
	hostUartRxNow(3, 0x85, 0);
	REQUIRE(uartModesTakeFrame(r.u(), d));
	hostUartRxNow(3, 0x12, 0);
	REQUIRE(uartModesTakeFrame(r.u(), d));
	CHECK(d == 0x12u);
	/* a foreign address puts the receiver back in mute mode (RM0090 30.3.6): its data is dropped */
	REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
	hostUartRxNow(3, 0x86, 0);
	hostUartRxNow(3, 0x13, 0);
	CHECK(!uartModesTakeFrame(r.u(), d));
}

TEST_CASE_FIXTURE(UartFixture, "UM3 in address mark mode with 9 bit words bit 8 marks the address") {
	for (unsigned i : {1u, 3u}) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesInit(r, i, UM_MP_ADDR, 5, UART_WORDLENGTH_9B);
		REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
		hostUartRxNow(i, 0x103, 0); /* bit 8 = 1, node 3: foreign */
		hostUartRxNow(i, 0x085, 0); /* bit 8 = 0: data, even with node bits 5 */
		CHECK((r.u()->SR & USART_SR_RXNE) == 0u);
		hostUartRxNow(i, 0x105, 0); /* matching address */
		CHECK((r.u()->CR1 & USART_CR1_RWU) == 0u);
		uint32_t d = 0;
		REQUIRE(uartModesTakeFrame(r.u(), d));
		CHECK(d == 0x105u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM3 in address mark mode the node addresses 0 and 15 are compared on all four bits") {
	UartModesRig a;
	uartModesInit(a, 1, UM_MP_ADDR, 0);
	REQUIRE(HAL_MultiProcessor_EnterMuteMode(&a.h) == HAL_OK);
	hostUartRxNow(1, 0x90, 0); /* MSB 1 = address; bits 6:4 are 001, only the 4 LSB (0) are compared: matches node 0 */
	uint32_t d = 0;
	REQUIRE(uartModesTakeFrame(a.u(), d));
	CHECK(d == 0x90u);

	UartModesRig b;
	uartModesInit(b, 3, UM_MP_ADDR, 0xF);
	REQUIRE(HAL_MultiProcessor_EnterMuteMode(&b.h) == HAL_OK);
	hostUartRxNow(3, 0x8E, 0);
	CHECK((b.u()->SR & USART_SR_RXNE) == 0u);
	hostUartRxNow(3, 0x8F, 0);
	REQUIRE(uartModesTakeFrame(b.u(), d));
	CHECK(d == 0x8Fu);
}

TEST_CASE_FIXTURE(UartFixture, "UM3 in address mark mode RWU cannot be written while RXNE is set") {
	/* RM0090 30.3.6: the RWU bit can be written when the receiver buffer contains no data (RXNE = 0), otherwise ignored. */
	UartModesRig r;
	uartModesInit(r, 2, UM_MP_ADDR, 5);
	hostUartRxNow(2, 0x12, 0);
	REQUIRE((r.u()->SR & USART_SR_RXNE) == USART_SR_RXNE);
	(void)HAL_MultiProcessor_EnterMuteMode(&r.h);
	CHECK((r.u()->CR1 & USART_CR1_RWU) == 0u);
	REQUIRE(r.u()->DR == 0x12u); /* RXNE cleared */
	REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
	CHECK((r.u()->CR1 & USART_CR1_RWU) == USART_CR1_RWU);
}

TEST_CASE_FIXTURE(UartFixture, "UM3 ExitMuteMode wakes up an address mark receiver by software") {
	UartModesRig r;
	uartModesInit(r, 3, UM_MP_ADDR, 5);
	REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
	hostUartRxNow(3, 0x12, 0);
	REQUIRE((r.u()->SR & USART_SR_RXNE) == 0u);
	REQUIRE(HAL_MultiProcessor_ExitMuteMode(&r.h) == HAL_OK);
	hostUartRxNow(3, 0x34, 0);
	uint32_t d = 0;
	REQUIRE(uartModesTakeFrame(r.u(), d));
	CHECK(d == 0x34u);
}

TEST_CASE_FIXTURE(UartFixture, "UM3 HAL_UART_Receive in address mark mute mode returns only the addressed message") {
	for (unsigned i : {1u, 3u}) {
		INFO("instance " << i);
		UartModesRig r;
		uartModesInit(r, i, UM_MP_ADDR, 5);
		REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
		hostUartRxPush(i, 0x83, 0); /* foreign address */
		hostUartRxPush(i, 0x11, 0); /* its data: dropped */
		hostUartRxPush(i, 0x85, 0); /* our address */
		hostUartRxPush(i, 0x22, 0);
		std::memset(uartModesBuf, 0, sizeof uartModesBuf);
		REQUIRE(HAL_UART_Receive(&r.h, uartModesBuf, 2, 100) == HAL_OK);
		CHECK(uartModesBuf[0] == 0x85);
		CHECK(uartModesBuf[1] == 0x22);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM3 characterization: the mute mode helpers return HAL_BUSY for a locked handle and change nothing") {
	/* NEEDS REVIEW 314 */
	UartModesRig r;
	uartModesInit(r, 1, UM_MP_IDLE);
	r.h.Lock = HAL_LOCKED;
	CHECK(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_BUSY);
	CHECK((r.u()->CR1 & USART_CR1_RWU) == 0u);
	r.h.Lock = HAL_UNLOCKED;
	REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
	r.h.Lock = HAL_LOCKED;
	CHECK(HAL_MultiProcessor_ExitMuteMode(&r.h) == HAL_BUSY);
	CHECK((r.u()->CR1 & USART_CR1_RWU) == USART_CR1_RWU);
}

TEST_CASE_FIXTURE(UartFixture, "UM3 characterization: the mute mode helpers while a transmit is active leave the state READY") {
	/* NEEDS REVIEW 314 */
	UartModesRig r;
	uartModesInit(r, 1, UM_MP_IDLE);
	uartModesBuf[0] = 7;
	REQUIRE(HAL_UART_Transmit_IT(&r.h, uartModesBuf, 1) == HAL_OK);
	REQUIRE(r.h.gState == HAL_UART_STATE_BUSY_TX);
	CHECK(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	r.h.gState = HAL_UART_STATE_BUSY_TX;
	CHECK(HAL_MultiProcessor_ExitMuteMode(&r.h) == HAL_OK);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
}

/* ---------------------------------------------------------------- UM4 state and error handling */

TEST_CASE_FIXTURE(UartFixture, "UM4 the three special mode Init functions return HAL_ERROR for a NULL handle") {
	CHECK(HAL_HalfDuplex_Init(nullptr) == HAL_ERROR);
	CHECK(HAL_LIN_Init(nullptr, UART_LINBREAKDETECTLENGTH_10B) == HAL_ERROR);
	CHECK(HAL_MultiProcessor_Init(nullptr, 5, UART_WAKEUPMETHOD_IDLELINE) == HAL_ERROR);
	CHECK(uartCounts.mspInit == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UM4 the first Init calls MspInit once and a repeated Init does not") {
	for (unsigned k = 0; k < uartModesKindCount; ++k) {
		INFO("kind " << k);
		uartSupportReset();
		UartModesRig r;
		uartModesPrepare(r, 1);
		REQUIRE(uartModesCall(&r.h, (UartModesKind)k) == HAL_OK);
		CHECK(uartCounts.mspInit == 1u);
		CHECK(uartCounts.last == &r.h);
		REQUIRE(uartModesCall(&r.h, (UartModesKind)k) == HAL_OK);
		CHECK(uartCounts.mspInit == 1u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM4 after a successful Init both states are READY and the error code is NONE") {
	for (unsigned k = 0; k < uartModesKindCount; ++k) {
		INFO("kind " << k);
		UartModesRig r;
		uartModesPrepare(r, 3);
		r.h.ErrorCode = HAL_UART_ERROR_ORE | HAL_UART_ERROR_FE;
		REQUIRE(uartModesCall(&r.h, (UartModesKind)k) == HAL_OK);
		CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
		CHECK(r.h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM4 Init writes the configuration only while UE is 0 and never disables a busy USART") {
	for (unsigned k = 0; k < uartModesKindCount; ++k) {
		INFO("kind " << k);
		UartModesRig r;
		uartModesPrepare(r, 2, UART_WORDLENGTH_9B, UART_PARITY_EVEN);
		REQUIRE(uartModesCall(&r.h, (UartModesKind)k) == HAL_OK);
		/* a second Init with UE = 1: the HAL clears UE first, so no change happens while enabled */
		REQUIRE(uartModesCall(&r.h, (UartModesKind)k) == HAL_OK);
		CHECK(hostUartConfigWhileEnabled(2) == 0u);
		CHECK(hostUartDisabledBusy(2) == 0u);
		CHECK((r.u()->CR1 & USART_CR1_UE) == USART_CR1_UE);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM4 the Init functions keep the format bits of the init structure") {
	for (unsigned k = 0; k < uartModesKindCount; ++k) {
		INFO("kind " << k);
		UartModesRig r;
		uartModesPrepare(r, 0, UART_WORDLENGTH_9B, UART_PARITY_ODD);
		REQUIRE(uartModesCall(&r.h, (UartModesKind)k) == HAL_OK);
		const uint32_t fmt = USART_CR1_M | USART_CR1_PCE | USART_CR1_PS;
		CHECK((r.u()->CR1 & fmt) == fmt);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM4 the Init functions of the other instances leave their registers untouched") {
	UartModesRig other;
	uartModesInit(other, 4, UM_HD);
	const uint32_t cr1 = other.u()->CR1, cr2 = other.u()->CR2, cr3 = other.u()->CR3, brr = other.u()->BRR;
	UartModesRig r;
	uartModesPrepare(r, 1);
	REQUIRE(HAL_MultiProcessor_Init(&r.h, 9, UART_WAKEUPMETHOD_ADDRESSMARK) == HAL_OK);
	CHECK(other.u()->CR1 == cr1);
	CHECK(other.u()->CR2 == cr2);
	CHECK(other.u()->CR3 == cr3);
	CHECK(other.u()->BRR == brr);
}

TEST_CASE_FIXTURE(UartFixture, "UM4 characterization: Init while a transmit is active resets both states to READY") {
	/* NEEDS REVIEW 314 */
	for (unsigned k = 0; k < uartModesKindCount; ++k) {
		INFO("kind " << k);
		UartModesRig r;
		uartModesInit(r, 1, (UartModesKind)k);
		uartModesBuf[0] = 7;
		REQUIRE(HAL_UART_Transmit_IT(&r.h, uartModesBuf, 1) == HAL_OK);
		REQUIRE(r.h.gState == HAL_UART_STATE_BUSY_TX);
		CHECK(uartModesCall(&r.h, (UartModesKind)k) == HAL_OK);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM4 Init of one mode after another replaces the mode bits") {
	UartModesRig r;
	uartModesInit(r, 3, UM_LIN11);
	REQUIRE((r.u()->CR2 & USART_CR2_LINEN) == USART_CR2_LINEN);
	REQUIRE(HAL_HalfDuplex_Init(&r.h) == HAL_OK);
	CHECK((r.u()->CR2 & USART_CR2_LINEN) == 0u);
	CHECK((r.u()->CR3 & USART_CR3_HDSEL) == USART_CR3_HDSEL);
	REQUIRE(HAL_MultiProcessor_Init(&r.h, 2, UART_WAKEUPMETHOD_ADDRESSMARK) == HAL_OK);
	CHECK((r.u()->CR3 & USART_CR3_HDSEL) == 0u);
	CHECK((r.u()->CR2 & USART_CR2_ADD) == 2u);
	REQUIRE(HAL_LIN_Init(&r.h, UART_LINBREAKDETECTLENGTH_10B) == HAL_OK);
	CHECK((r.u()->CR2 & USART_CR2_LINEN) == USART_CR2_LINEN);
}

/* ---------------------------------------------------------------- fix pass */

TEST_CASE_FIXTURE(UartFixture, "UM4 a re-Init with a different format while UE is 1 changes M PCE PS STOP and BRR only with UE 0") {
	/* RM0090 30.6.3 / 30.6.4: M, PCE, PS, STOP, BRR must not change while UE = 1, so the HAL has to clear UE first. */
	for (unsigned k = 0; k < uartModesKindCount; ++k) {
		INFO("kind " << k);
		UartModesRig r;
		uartModesPrepare(r, 2, UART_WORDLENGTH_8B, UART_PARITY_NONE);
		REQUIRE(uartModesCall(&r.h, (UartModesKind)k) == HAL_OK);
		REQUIRE((r.u()->CR1 & USART_CR1_UE) == USART_CR1_UE);
		r.h.Init.WordLength = UART_WORDLENGTH_9B;
		r.h.Init.Parity = UART_PARITY_ODD;
		r.h.Init.StopBits = UART_STOPBITS_2;
		r.h.Init.BaudRate = 9600;
		REQUIRE(uartModesCall(&r.h, (UartModesKind)k) == HAL_OK);
		CHECK(hostUartConfigWhileEnabled(2) == 0u);
		const uint32_t fmt = USART_CR1_M | USART_CR1_PCE | USART_CR1_PS;
		CHECK((r.u()->CR1 & fmt) == fmt);
		CHECK((r.u()->CR2 & USART_CR2_STOP) == UART_STOPBITS_2);
		CHECK((r.u()->CR1 & USART_CR1_UE) == USART_CR1_UE);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM4 the Init functions unlock a locked handle in state RESET") {
	for (unsigned k = 0; k < uartModesKindCount; ++k) {
		INFO("kind " << k);
		UartModesRig r;
		uartModesPrepare(r, 1);
		r.h.Lock = HAL_LOCKED;
		REQUIRE(r.h.gState == HAL_UART_STATE_RESET);
		REQUIRE(uartModesCall(&r.h, (UartModesKind)k) == HAL_OK);
		CHECK(r.h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UM3 Enter and ExitMuteMode leave the handle unlocked") {
	UartModesRig r;
	uartModesInit(r, 1, UM_MP_IDLE);
	REQUIRE(HAL_MultiProcessor_EnterMuteMode(&r.h) == HAL_OK);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	REQUIRE(HAL_MultiProcessor_ExitMuteMode(&r.h) == HAL_OK);
	CHECK(r.h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(UartFixture, "UM4 characterization: the Rx event type is TC after each special mode Init") {
	/* NR_UART_5 (lead: 314) */
	for (unsigned k = 0; k < uartModesKindCount; ++k) {
		INFO("kind " << k);
		UartModesRig r;
		uartModesPrepare(r, 3);
		r.h.RxEventType = HAL_UART_RXEVENT_IDLE;
		REQUIRE(uartModesCall(&r.h, (UartModesKind)k) == HAL_OK);
		CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
	}
}
