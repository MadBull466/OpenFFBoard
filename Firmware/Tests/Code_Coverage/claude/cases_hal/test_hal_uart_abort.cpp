/*
 * P12 T4b group UA: abort of the real ST HAL UART driver (stm32f4xx_hal_uart.c: HAL_UART_Abort, AbortTransmit, AbortReceive and the _IT
 * variants, the static UART_DMA{Tx,Rx}AbortCallback, UART_DMA{Tx,Rx}OnlyAbortCallback and UART_DMAAbortOnError) together with the real HAL DMA,
 * against the UART model (fakes/stm32_hal/HalUart.h) and the DMA model (fakes/stm32_hal/HalDma.h). Requirements UA1-UA4 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T4B_REQUIREMENTS.md; compiled into ffb_hal_tests (F407VG) and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 (UART chapter: the notes of HAL_UART_Abort* : disable the UART interrupts, disable the DMA request, abort the DMA
 * transfer by HAL_DMA_Abort (blocking) or HAL_DMA_Abort_IT (_IT), set the handle state to READY, call the user abort complete callback
 * at the end of an _IT abort; "when exiting function, Abort is considered as completed" for the blocking ones) and RM0090 Rev 22:
 *  - Table 1: USART1 0x40011000, USART2 0x40004400, USART3 0x40004800, UART4 0x40004C00, UART5 0x40005000, USART6 0x40011400.
 *  - Tables 43 / 44 (channel 4; USART6 channel 5): USART1 DMA2 Tx stream 7 / Rx stream 2, USART2 DMA1 Tx 6 / Rx 5, USART3 DMA1 Tx 3 / Rx 1,
 *    UART4 DMA1 Tx 4 / Rx 2, UART5 DMA1 Tx 7 / Rx 0, USART6 DMA2 Tx 6 / Rx 1.
 *  - 30.6.1 SR: PE 0, FE 1, NE 2, ORE 3, IDLE 4, RXNE 5, TC 6, TXE 7. 30.6.4 CR1: RE 2, TE 3, IDLEIE 4, RXNEIE 5, TCIE 6, TXEIE 7, PEIE 8,
 *    PS 9, PCE 10, UE 13. 30.6.6 CR3: EIE 0, DMAR 6, DMAT 7.
 * The tests stand in for the DMA engine: they call HAL_DMA_IRQHandler (which finishes the abort of a stream started by HAL_DMA_Abort_IT,
 * the model sets TCIF at a software stop) and hostDmaComplete. Time is the HAL tick: every HAL_GetTick() call advances it by 1 ms.
 * Every file-local symbol is prefixed uartAbort or sits in the anonymous namespace.
 *
 * NEEDS REVIEW proposals (numbers assigned by the lead):
 *  NEEDS REVIEW 318 UA1: the HAL's Abort does not flush the receive data register: a received frame stays in DR (RXNE stays 1) (UM1725 silent).
 *  NEEDS REVIEW 318 UA1: HAL_UART_Abort resets ErrorCode to NONE, AbortTransmit and AbortReceive leave it alone (UM1725 silent).
 *  NEEDS REVIEW 318 UA1: a blocking abort whose DMA stream never stops returns HAL_TIMEOUT, sets HAL_UART_ERROR_DMA, and leaves the state BUSY
 *     (UM1725 silent about the timeout).
 *  NEEDS REVIEW 318 UA1: a blocking abort whose DMA stream is not busy (HAL_DMA_Abort returns HAL_ERROR, not a timeout) ignores that and returns HAL_OK.
 *  NEEDS REVIEW 318 UA2: an _IT abort whose HAL_DMA_Abort_IT fails (stream not busy) runs the abort callback at once; DMAT set without a DMA handle
 *     is cleared and the callback runs at once.
 *  NEEDS REVIEW 318 UA1/UA2: Abort_IT and the DMA abort callbacks of Abort_IT reset ErrorCode, the Only variants (AbortTransmit_IT, AbortReceive_IT) do not.
 *  NEEDS REVIEW 318 UA1/UA2: after an abort the Tx / Rx transfer counters read 0 and ReceptionType reads STANDARD (UM1725 silent).
 */
#include "HalDma.h"
#include "HalFixture.h"
#include "HalHostSystem.h"
#include "HalUart.h"
#include "TestHelpers.h"
#include "UartTestSupport.h"

#include <cstdint>
#include <cstring>

namespace {

/* RM0090 30.6.1 SR, 30.6.4 CR1, 30.6.6 CR3. */
constexpr uint32_t uartAbortSR_IDLE = 1u << 4;
constexpr uint32_t uartAbortSR_RXNE = 1u << 5;
constexpr uint32_t uartAbortCR1_IDLEIE = 1u << 4;
constexpr uint32_t uartAbortCR1_RXNEIE = 1u << 5;
constexpr uint32_t uartAbortCR1_TCIE = 1u << 6;
constexpr uint32_t uartAbortCR1_TXEIE = 1u << 7;
constexpr uint32_t uartAbortCR1_PEIE = 1u << 8;
constexpr uint32_t uartAbortCR3_EIE = 1u << 0;
constexpr uint32_t uartAbortCR3_DMAR = 1u << 6;
constexpr uint32_t uartAbortCR3_DMAT = 1u << 7;
constexpr uint32_t uartAbortDMA_EN = 1u << 0;

constexpr uint32_t uartAbortTxIEs = uartAbortCR1_TXEIE | uartAbortCR1_TCIE;
constexpr uint32_t uartAbortRxIEs1 = uartAbortCR1_RXNEIE | uartAbortCR1_PEIE | uartAbortCR1_IDLEIE;

const char *const uartAbortNames[6] = {"USART1", "USART2", "USART3", "UART4", "UART5", "USART6"};
#define UART_ABORT_EACH(n) for (unsigned n = 0; n < 6; ++n) SUBCASE(uartAbortNames[n])

struct UartAbortMap {
	unsigned c, txS, rxS;
	uint32_t channel;
};

/* RM0090 Tables 43 and 44. */
const UartAbortMap uartAbortMaps[6] = {
    {1, 7, 2, DMA_CHANNEL_4}, {0, 6, 5, DMA_CHANNEL_4}, {0, 3, 1, DMA_CHANNEL_4},
    {0, 4, 2, DMA_CHANNEL_4}, {0, 7, 0, DMA_CHANNEL_4}, {1, 6, 1, DMA_CHANNEL_5},
};

USART_TypeDef *uartAbortInstance(unsigned n) {
	USART_TypeDef *const a[6] = {USART1, USART2, USART3, UART4, UART5, USART6};
	return a[n];
}

DMA_Stream_TypeDef *uartAbortStream(unsigned c, unsigned s) {
	DMA_Stream_TypeDef *const d1[8] = {DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3,
	                                   DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7};
	DMA_Stream_TypeDef *const d2[8] = {DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3,
	                                   DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7};
	return c == 0 ? d1[s] : d2[s];
}

struct UartAbortRig {
	unsigned n;
	UartAbortMap m;
	USART_TypeDef *inst;
	UART_HandleTypeDef h;
	DMA_HandleTypeDef tx, rx;
	uint8_t txBuf[16];
	uint8_t rxBuf[16];

	/* linkDma: the DMA handles are initialised with the real HAL_DMA_Init and linked to the UART handle. */
	explicit UartAbortRig(unsigned idx, bool linkDma = false, uint32_t parity = UART_PARITY_NONE)
	    : n(idx), m(uartAbortMaps[idx]), inst(uartAbortInstance(idx)) {
		std::memset(&h, 0, sizeof h);
		for (unsigned i = 0; i < sizeof txBuf; ++i) {
			txBuf[i] = static_cast<uint8_t>(0x30u + i);
			rxBuf[i] = 0;
		}
		h.Instance = inst;
		h.Init.BaudRate = 115200;
		h.Init.WordLength = UART_WORDLENGTH_8B;
		h.Init.StopBits = UART_STOPBITS_1;
		h.Init.Parity = parity;
		h.Init.Mode = UART_MODE_TX_RX;
		h.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		h.Init.OverSampling = UART_OVERSAMPLING_16;
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		uartSupportReset();
		if (linkDma) {
			makeDma(tx, uartAbortStream(m.c, m.txS), DMA_MEMORY_TO_PERIPH);
			makeDma(rx, uartAbortStream(m.c, m.rxS), DMA_PERIPH_TO_MEMORY);
			h.hdmatx = &tx;
			tx.Parent = &h;
			h.hdmarx = &rx;
			rx.Parent = &h;
		}
	}

	void makeDma(DMA_HandleTypeDef &d, DMA_Stream_TypeDef *s, uint32_t dir) {
		std::memset(&d, 0, sizeof d);
		d.Instance = s;
		d.Init.Channel = m.channel;
		d.Init.Direction = dir;
		d.Init.PeriphInc = DMA_PINC_DISABLE;
		d.Init.MemInc = DMA_MINC_ENABLE;
		d.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		d.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		d.Init.Mode = DMA_NORMAL;
		d.Init.Priority = DMA_PRIORITY_LOW;
		d.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		d.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		d.Init.MemBurst = DMA_MBURST_SINGLE;
		d.Init.PeriphBurst = DMA_PBURST_SINGLE;
		REQUIRE(HAL_DMA_Init(&d) == HAL_OK);
	}

	void startTxIT() { REQUIRE(HAL_UART_Transmit_IT(&h, txBuf, 4) == HAL_OK); }
	void startRxIT() { REQUIRE(HAL_UART_Receive_IT(&h, rxBuf, 4) == HAL_OK); }
	void startRxIdleIT() { REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&h, rxBuf, 8) == HAL_OK); }
	void startTxDMA() { REQUIRE(HAL_UART_Transmit_DMA(&h, txBuf, 4) == HAL_OK); }
	void startRxDMA() { REQUIRE(HAL_UART_Receive_DMA(&h, rxBuf, 4) == HAL_OK); }
	void startRxIdleDMA() { REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&h, rxBuf, 8) == HAL_OK); }

	uint32_t cr1() const { return inst->CR1; }
	uint32_t cr3() const { return inst->CR3; }
	static bool streamOn(const DMA_HandleTypeDef &d) { return (d.Instance->CR & uartAbortDMA_EN) != 0u; }
	void txStreamDone() { HAL_DMA_IRQHandler(&tx); }
	void rxStreamDone() { HAL_DMA_IRQHandler(&rx); }
	unsigned callbacksTotal() const {
		return uartCounts.abortCplt + uartCounts.abortTxCplt + uartCounts.abortRxCplt + uartCounts.txCplt + uartCounts.rxCplt +
		       uartCounts.error + uartCounts.rxEvent;
	}
};

class UartAbortFx : public UartFixture {};

} // namespace

/* ------------------------------------------------------------------------------------------------------------------ UA1 */

TEST_CASE_FIXTURE(UartAbortFx, "UA1 Abort with nothing running returns HAL_OK, keeps both states READY and runs no callback") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(r.callbacksTotal() == 0u);
		CHECK(HAL_UART_AbortTransmit(&r.h) == HAL_OK);
		CHECK(HAL_UART_AbortReceive(&r.h) == HAL_OK);
		CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
		CHECK(r.callbacksTotal() == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 Abort of a Transmit_IT clears TXEIE, sets the state READY and runs no callback") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		r.startTxIT();
		REQUIRE((r.cr1() & uartAbortCR1_TXEIE) != 0u);
		REQUIRE(r.h.gState == HAL_UART_STATE_BUSY_TX);
		CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortTxIEs) == 0u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(r.callbacksTotal() == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 Abort of a Transmit_IT whose last frame is loaded (TCIE set) clears TCIE") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		REQUIRE(HAL_UART_Transmit_IT(&r.h, r.txBuf, 1) == HAL_OK);
		HAL_UART_IRQHandler(&r.h); // TXE = 1: the single frame is sent, TXEIE off, TCIE on
		REQUIRE((r.cr1() & uartAbortCR1_TCIE) != 0u);
		CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortTxIEs) == 0u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(uartCounts.txCplt == 0u); // the abort ends it before the complete callback
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 Abort of a Receive_IT clears RXNEIE and EIE (no parity: PEIE stays 0), state READY, no callback") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		r.startRxIT();
		REQUIRE((r.cr1() & uartAbortCR1_RXNEIE) != 0u);
		REQUIRE((r.cr3() & uartAbortCR3_EIE) != 0u);
		CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
		CHECK((r.cr1() & (uartAbortCR1_RXNEIE | uartAbortCR1_PEIE)) == 0u);
		CHECK((r.cr3() & uartAbortCR3_EIE) == 0u);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(r.callbacksTotal() == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 Abort of a Receive_IT with parity also clears PEIE") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, false, UART_PARITY_EVEN);
		r.startRxIT();
		REQUIRE((r.cr1() & uartAbortCR1_PEIE) != 0u);
		CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
		CHECK((r.cr1() & (uartAbortCR1_RXNEIE | uartAbortCR1_PEIE)) == 0u);
		CHECK((r.cr3() & uartAbortCR3_EIE) == 0u);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 Abort of a ReceiveToIdle_IT also clears IDLEIE and returns to standard reception") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		r.startRxIdleIT();
		REQUIRE((r.cr1() & uartAbortCR1_IDLEIE) != 0u);
		CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortRxIEs1) == 0u);
		CHECK((r.cr3() & uartAbortCR3_EIE) == 0u);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
		CHECK(r.callbacksTotal() == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 Abort of a Transmit_DMA clears DMAT, stops the Tx stream and sets the state READY without a callback") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxDMA();
		REQUIRE((r.cr3() & uartAbortCR3_DMAT) != 0u);
		REQUIRE(UartAbortRig::streamOn(r.tx));
		CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
		CHECK((r.cr3() & uartAbortCR3_DMAT) == 0u);
		CHECK(!UartAbortRig::streamOn(r.tx));
		CHECK(r.tx.State == HAL_DMA_STATE_READY);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(r.callbacksTotal() == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 Abort of a Receive_DMA clears DMAR and EIE, stops the Rx stream and sets the state READY without a callback") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startRxDMA();
		REQUIRE((r.cr3() & uartAbortCR3_DMAR) != 0u);
		REQUIRE((r.cr3() & uartAbortCR3_EIE) != 0u);
		REQUIRE(UartAbortRig::streamOn(r.rx));
		CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
		CHECK((r.cr3() & (uartAbortCR3_DMAR | uartAbortCR3_EIE)) == 0u);
		CHECK(!UartAbortRig::streamOn(r.rx));
		CHECK(r.rx.State == HAL_DMA_STATE_READY);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(r.callbacksTotal() == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 Abort of a Receive_DMA with parity also clears PEIE") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true, UART_PARITY_ODD);
		r.startRxDMA();
		REQUIRE((r.cr1() & uartAbortCR1_PEIE) != 0u);
		CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortCR1_PEIE) == 0u);
		CHECK((r.cr3() & (uartAbortCR3_DMAR | uartAbortCR3_EIE)) == 0u);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 Abort of a ReceiveToIdle_DMA clears IDLEIE and DMAR and stops the Rx stream") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startRxIdleDMA();
		REQUIRE((r.cr1() & uartAbortCR1_IDLEIE) != 0u);
		CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortCR1_IDLEIE) == 0u);
		CHECK((r.cr3() & (uartAbortCR3_DMAR | uartAbortCR3_EIE)) == 0u);
		CHECK(!UartAbortRig::streamOn(r.rx));
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
		CHECK(r.callbacksTotal() == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 Abort of a Transmit_IT and a Receive_IT at the same time ends both and runs no callback") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		r.startTxIT();
		r.startRxIT();
		REQUIRE(HAL_UART_GetState(&r.h) == HAL_UART_STATE_BUSY_TX_RX);
		CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
		CHECK((r.cr1() & (uartAbortTxIEs | uartAbortRxIEs1)) == 0u);
		CHECK((r.cr3() & uartAbortCR3_EIE) == 0u);
		CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
		CHECK(r.callbacksTotal() == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 characterization: Abort leaves a received frame in the data register (RXNE stays set)") {
	// NEEDS REVIEW 318: the charter lists a flush of the Rx data register; the HAL does not touch DR and UM1725 does not say.
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		r.startRxIT();
		hostUartRxNow(n, 0x41u, 0u);
		REQUIRE((r.inst->SR & uartAbortSR_RXNE) != 0u);
		CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
		CHECK((r.inst->SR & uartAbortSR_RXNE) != 0u);
		CHECK(r.inst->DR == 0x41u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 characterization: Abort resets ErrorCode, AbortTransmit and AbortReceive leave it") {
	// NEEDS REVIEW 318: UM1725 says nothing about ErrorCode in the abort functions. The code is set directly (a left over error).
	UartAbortRig r(0);
	r.h.ErrorCode = HAL_UART_ERROR_FE;
	CHECK(HAL_UART_AbortTransmit(&r.h) == HAL_OK);
	CHECK(HAL_UART_AbortReceive(&r.h) == HAL_OK);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_FE);
	CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 AbortTransmit of a Transmit_IT clears TXEIE and TCIE, sets the gState READY, no callback") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		r.startTxIT();
		CHECK(HAL_UART_AbortTransmit(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortTxIEs) == 0u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(r.callbacksTotal() == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 AbortTransmit of a Transmit_DMA clears DMAT, stops the Tx stream and sets the gState READY") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxDMA();
		CHECK(HAL_UART_AbortTransmit(&r.h) == HAL_OK);
		CHECK((r.cr3() & uartAbortCR3_DMAT) == 0u);
		CHECK(!UartAbortRig::streamOn(r.tx));
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(r.callbacksTotal() == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 AbortReceive of a Receive_IT clears RXNEIE, PEIE and EIE and sets the RxState READY, no callback") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, false, UART_PARITY_EVEN);
		r.startRxIT();
		CHECK(HAL_UART_AbortReceive(&r.h) == HAL_OK);
		CHECK((r.cr1() & (uartAbortCR1_RXNEIE | uartAbortCR1_PEIE)) == 0u);
		CHECK((r.cr3() & uartAbortCR3_EIE) == 0u);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(r.callbacksTotal() == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 AbortReceive of a ReceiveToIdle_IT also clears IDLEIE") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		r.startRxIdleIT();
		CHECK(HAL_UART_AbortReceive(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortRxIEs1) == 0u);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 AbortReceive of a Receive_DMA clears DMAR and EIE, stops the Rx stream and sets the RxState READY") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startRxDMA();
		CHECK(HAL_UART_AbortReceive(&r.h) == HAL_OK);
		CHECK((r.cr3() & (uartAbortCR3_DMAR | uartAbortCR3_EIE)) == 0u);
		CHECK(!UartAbortRig::streamOn(r.rx));
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(r.callbacksTotal() == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 AbortReceive of a ReceiveToIdle_DMA also clears IDLEIE") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startRxIdleDMA();
		CHECK(HAL_UART_AbortReceive(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortCR1_IDLEIE) == 0u);
		CHECK(!UartAbortRig::streamOn(r.rx));
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 a transfer can be started again after a blocking Abort of a DMA transfer") {
	UartAbortRig r(1, true);
	r.startTxDMA();
	r.startRxDMA();
	REQUIRE(HAL_UART_Abort(&r.h) == HAL_OK);
	CHECK(HAL_UART_Transmit_DMA(&r.h, r.txBuf, 3) == HAL_OK);
	CHECK(HAL_UART_Receive_DMA(&r.h, r.rxBuf, 3) == HAL_OK);
	CHECK(UartAbortRig::streamOn(r.tx));
	CHECK(UartAbortRig::streamOn(r.rx));
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 characterization: Abort of a Tx stream that never stops returns HAL_TIMEOUT, sets ErrorCode DMA and leaves gState BUSY_TX") {
	// NEEDS REVIEW 318
	UartAbortRig r(2, true);
	r.startTxDMA();
	hostDmaStickEnabled(r.m.c, r.m.txS, 1);
	CHECK(HAL_UART_Abort(&r.h) == HAL_TIMEOUT);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_DMA);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
	CHECK((r.cr3() & uartAbortCR3_DMAT) == 0u);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 characterization: Abort of an Rx stream that never stops returns HAL_TIMEOUT, sets ErrorCode DMA and leaves RxState BUSY_RX") {
	// NEEDS REVIEW 318
	UartAbortRig r(2, true);
	r.startRxDMA();
	hostDmaStickEnabled(r.m.c, r.m.rxS, 1);
	CHECK(HAL_UART_Abort(&r.h) == HAL_TIMEOUT);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_DMA);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK((r.cr3() & uartAbortCR3_DMAR) == 0u);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 characterization: AbortTransmit of a Tx stream that never stops returns HAL_TIMEOUT with ErrorCode DMA") {
	// NEEDS REVIEW 318
	UartAbortRig r(3, true);
	r.startTxDMA();
	hostDmaStickEnabled(r.m.c, r.m.txS, 1);
	CHECK(HAL_UART_AbortTransmit(&r.h) == HAL_TIMEOUT);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_DMA);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 characterization: AbortReceive of an Rx stream that never stops returns HAL_TIMEOUT with ErrorCode DMA") {
	// NEEDS REVIEW 318
	UartAbortRig r(3, true);
	r.startRxDMA();
	hostDmaStickEnabled(r.m.c, r.m.rxS, 1);
	CHECK(HAL_UART_AbortReceive(&r.h) == HAL_TIMEOUT);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_DMA);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 characterization: a blocking abort whose DMA streams are not busy any more returns HAL_OK and sets the states READY") {
	// NEEDS REVIEW 318: the streams are stopped through the DMA API first, DMAT and DMAR stay set in the UART.
	UartAbortRig r(4, true);
	r.startTxDMA();
	r.startRxDMA();
	REQUIRE(HAL_DMA_Abort(&r.tx) == HAL_OK);
	REQUIRE(HAL_DMA_Abort(&r.rx) == HAL_OK);
	r.h.ErrorCode = HAL_UART_ERROR_FE; // seeded so that the reset is observable
	CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
	CHECK((r.cr3() & (uartAbortCR3_DMAT | uartAbortCR3_DMAR)) == 0u);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 characterization: AbortTransmit and AbortReceive whose streams are not busy return HAL_OK") {
	// NEEDS REVIEW 318
	UartAbortRig r(4, true);
	r.startTxDMA();
	r.startRxDMA();
	REQUIRE(HAL_DMA_Abort(&r.tx) == HAL_OK);
	REQUIRE(HAL_DMA_Abort(&r.rx) == HAL_OK);
	CHECK(HAL_UART_AbortTransmit(&r.h) == HAL_OK);
	CHECK(HAL_UART_AbortReceive(&r.h) == HAL_OK);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 DMA requests set in the registers without a linked DMA handle are cleared by the blocking aborts") {
	UartAbortRig r(1);
	r.inst->CR3 |= uartAbortCR3_DMAT | uartAbortCR3_DMAR;
	CHECK(HAL_UART_AbortTransmit(&r.h) == HAL_OK);
	CHECK((r.cr3() & uartAbortCR3_DMAT) == 0u);
	CHECK((r.cr3() & uartAbortCR3_DMAR) != 0u);
	CHECK(HAL_UART_AbortReceive(&r.h) == HAL_OK);
	CHECK((r.cr3() & uartAbortCR3_DMAR) == 0u);
	r.inst->CR3 |= uartAbortCR3_DMAT | uartAbortCR3_DMAR;
	CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
	CHECK((r.cr3() & (uartAbortCR3_DMAT | uartAbortCR3_DMAR)) == 0u);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

/* ------------------------------------------------------------------------------------------------------------------ UA2 */

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT with nothing running returns HAL_OK and runs AbortCplt once at once") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
		CHECK(uartCounts.abortCplt == 1u);
		CHECK(uartCounts.abortTxCplt == 0u);
		CHECK(uartCounts.abortRxCplt == 0u);
		CHECK(uartCounts.last == &r.h);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT of Transmit_IT and Receive_IT clears the enables and runs AbortCplt once with both states READY") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, false, UART_PARITY_EVEN);
		r.startTxIT();
		r.startRxIT();
		CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
		CHECK(uartCounts.abortCplt == 1u);
		CHECK(uartCounts.gStateAtLast == HAL_UART_STATE_READY);
		CHECK(uartCounts.rxStateAtLast == HAL_UART_STATE_READY);
		CHECK((r.cr1() & (uartAbortTxIEs | uartAbortRxIEs1)) == 0u);
		CHECK((r.cr3() & uartAbortCR3_EIE) == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT of a ReceiveToIdle_IT clears IDLEIE and runs AbortCplt once") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		r.startRxIdleIT();
		CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortCR1_IDLEIE) == 0u);
		CHECK(uartCounts.abortCplt == 1u);
		CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
		CHECK(uartCounts.rxEvent == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT of an interrupt transfer with DMA handles linked but no DMA running runs AbortCplt at once") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxIT();
		r.startRxIT();
		CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
		CHECK(uartCounts.abortCplt == 1u);
		CHECK(r.h.hdmatx->XferAbortCallback == nullptr);
		CHECK(r.h.hdmarx->XferAbortCallback == nullptr);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT of a Transmit_DMA runs AbortCplt once after the Tx stream abort completes") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxDMA();
		CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
		CHECK((r.cr3() & uartAbortCR3_DMAT) == 0u);
		CHECK(uartCounts.abortCplt == 0u); // UM1725: completed only when the callback runs
		r.txStreamDone();
		CHECK(uartCounts.abortCplt == 1u);
		CHECK(uartCounts.gStateAtLast == HAL_UART_STATE_READY);
		CHECK(uartCounts.rxStateAtLast == HAL_UART_STATE_READY);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(!UartAbortRig::streamOn(r.tx));
		CHECK(uartCounts.txCplt == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT of a Receive_DMA runs AbortCplt once after the Rx stream abort completes") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true, UART_PARITY_ODD);
		r.startRxDMA();
		CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
		CHECK((r.cr3() & (uartAbortCR3_DMAR | uartAbortCR3_EIE)) == 0u);
		CHECK((r.cr1() & uartAbortCR1_PEIE) == 0u);
		CHECK(uartCounts.abortCplt == 0u);
		r.rxStreamDone();
		CHECK(uartCounts.abortCplt == 1u);
		CHECK(uartCounts.rxStateAtLast == HAL_UART_STATE_READY);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(!UartAbortRig::streamOn(r.rx));
		CHECK(uartCounts.rxCplt == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT of a ReceiveToIdle_DMA clears IDLEIE and runs AbortCplt once after the stream abort") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startRxIdleDMA();
		CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortCR1_IDLEIE) == 0u);
		CHECK(uartCounts.abortCplt == 0u);
		r.rxStreamDone();
		CHECK(uartCounts.abortCplt == 1u);
		CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(uartCounts.rxEvent == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 the abort callback runs once, not again when the DMA interrupt handler runs a second time") {
	UartAbortRig r(1, true);
	r.startTxDMA();
	REQUIRE(HAL_UART_Abort_IT(&r.h) == HAL_OK);
	r.txStreamDone();
	r.txStreamDone();
	CHECK(uartCounts.abortCplt == 1u);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT with DMAT set and no Tx DMA handle clears DMAT and runs AbortCplt at once") {
	UartAbortRig r(1);
	r.inst->CR3 |= uartAbortCR3_DMAT | uartAbortCR3_DMAR;
	CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
	CHECK((r.cr3() & (uartAbortCR3_DMAT | uartAbortCR3_DMAR)) == 0u);
	CHECK(uartCounts.abortCplt == 1u);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT of a Tx stream that is not busy runs AbortCplt at once") {
	// NEEDS REVIEW 318
	UartAbortRig r(2, true);
	r.startTxDMA();
	REQUIRE(HAL_DMA_Abort(&r.tx) == HAL_OK);
	CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
	CHECK(uartCounts.abortCplt == 1u);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK(r.tx.XferAbortCallback == nullptr);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT of an Rx stream that is not busy runs AbortCplt at once") {
	// NEEDS REVIEW 318
	UartAbortRig r(2, true);
	r.startRxDMA();
	REQUIRE(HAL_DMA_Abort(&r.rx) == HAL_OK);
	CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
	CHECK(uartCounts.abortCplt == 1u);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK(r.rx.XferAbortCallback == nullptr);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT of a Tx DMA and an Rx stream that is not busy runs AbortCplt once only when the Tx stream has finished aborting" *
                                   KNOWN_ISSUE("V105 (arguable, low: contrived trigger) HAL_UART_Abort_IT (stm32f4xx_hal_uart.c line 2144) sets AbortCplt = 1 when HAL_DMA_Abort_IT of the Rx stream fails even though the Tx abort is pending: AbortCpltCallback runs at once and again from UART_DMATxAbortCallback")) {
	// UM1725: "At abort completion, call user abort complete callback" (once, when the abort is complete). Scenario: Transmit_DMA running;
	// the Rx stream is stopped through the DMA API while DMAR is still set, then Abort_IT.
	UartAbortRig r(2, true);
	r.startTxDMA();
	r.startRxDMA();
	REQUIRE(HAL_DMA_Abort(&r.rx) == HAL_OK);
	REQUIRE(HAL_UART_Abort_IT(&r.h) == HAL_OK);
	CHECK(uartCounts.abortCplt == 0u); // the Tx stream is still aborting
	r.txStreamDone();
	CHECK(uartCounts.abortCplt == 1u);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 AbortTransmit_IT with no DMA runs AbortTransmitCplt once at once and not AbortCplt") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		r.startTxIT();
		CHECK(HAL_UART_AbortTransmit_IT(&r.h) == HAL_OK);
		CHECK(uartCounts.abortTxCplt == 1u);
		CHECK(uartCounts.abortCplt == 0u);
		CHECK(uartCounts.abortRxCplt == 0u);
		CHECK(uartCounts.gStateAtLast == HAL_UART_STATE_READY);
		CHECK((r.cr1() & uartAbortTxIEs) == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 AbortTransmit_IT of an interrupt transfer with a linked Tx DMA handle runs AbortTransmitCplt at once") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxIT();
		CHECK(HAL_UART_AbortTransmit_IT(&r.h) == HAL_OK);
		CHECK(uartCounts.abortTxCplt == 1u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 AbortTransmit_IT of a Transmit_DMA runs AbortTransmitCplt once after the Tx stream abort completes") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxDMA();
		CHECK(HAL_UART_AbortTransmit_IT(&r.h) == HAL_OK);
		CHECK((r.cr3() & uartAbortCR3_DMAT) == 0u);
		CHECK(uartCounts.abortTxCplt == 0u);
		r.txStreamDone();
		CHECK(uartCounts.abortTxCplt == 1u);
		CHECK(uartCounts.abortCplt == 0u);
		CHECK(uartCounts.gStateAtLast == HAL_UART_STATE_READY);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(!UartAbortRig::streamOn(r.tx));
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 AbortTransmit_IT with DMAT set and no Tx DMA handle clears DMAT and runs AbortTransmitCplt at once") {
	// NEEDS REVIEW 318
	UartAbortRig r(0);
	r.startTxIT();
	r.inst->CR3 |= uartAbortCR3_DMAT;
	CHECK(HAL_UART_AbortTransmit_IT(&r.h) == HAL_OK);
	CHECK((r.cr3() & uartAbortCR3_DMAT) == 0u);
	CHECK(uartCounts.abortTxCplt == 1u);
	CHECK(uartCounts.gStateAtLast == HAL_UART_STATE_READY);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK(r.h.TxXferCount == 0u);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 AbortTransmit_IT of a Tx stream that is not busy runs AbortTransmitCplt at once") {
	// NEEDS REVIEW 318
	UartAbortRig r(3, true);
	r.startTxDMA();
	REQUIRE(HAL_DMA_Abort(&r.tx) == HAL_OK);
	CHECK(HAL_UART_AbortTransmit_IT(&r.h) == HAL_OK);
	CHECK(uartCounts.abortTxCplt == 1u);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 AbortReceive_IT with no DMA runs AbortReceiveCplt once at once and not AbortCplt") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, false, UART_PARITY_EVEN);
		r.startRxIT();
		CHECK(HAL_UART_AbortReceive_IT(&r.h) == HAL_OK);
		CHECK(uartCounts.abortRxCplt == 1u);
		CHECK(uartCounts.abortCplt == 0u);
		CHECK(uartCounts.abortTxCplt == 0u);
		CHECK(uartCounts.rxStateAtLast == HAL_UART_STATE_READY);
		CHECK((r.cr1() & (uartAbortCR1_RXNEIE | uartAbortCR1_PEIE)) == 0u);
		CHECK((r.cr3() & uartAbortCR3_EIE) == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 AbortReceive_IT of a ReceiveToIdle_IT clears IDLEIE and runs AbortReceiveCplt once") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		r.startRxIdleIT();
		CHECK(HAL_UART_AbortReceive_IT(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortCR1_IDLEIE) == 0u);
		CHECK(uartCounts.abortRxCplt == 1u);
		CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 AbortReceive_IT of an interrupt reception with a linked Rx DMA handle runs AbortReceiveCplt at once") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startRxIT();
		CHECK(HAL_UART_AbortReceive_IT(&r.h) == HAL_OK);
		CHECK(uartCounts.abortRxCplt == 1u);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 AbortReceive_IT of a Receive_DMA runs AbortReceiveCplt once after the Rx stream abort completes") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startRxDMA();
		CHECK(HAL_UART_AbortReceive_IT(&r.h) == HAL_OK);
		CHECK((r.cr3() & (uartAbortCR3_DMAR | uartAbortCR3_EIE)) == 0u);
		CHECK(uartCounts.abortRxCplt == 0u);
		r.rxStreamDone();
		CHECK(uartCounts.abortRxCplt == 1u);
		CHECK(uartCounts.abortCplt == 0u);
		CHECK(uartCounts.rxStateAtLast == HAL_UART_STATE_READY);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(!UartAbortRig::streamOn(r.rx));
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 AbortReceive_IT of a ReceiveToIdle_DMA clears IDLEIE and runs AbortReceiveCplt once after the stream abort") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startRxIdleDMA();
		CHECK(HAL_UART_AbortReceive_IT(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortCR1_IDLEIE) == 0u);
		CHECK(uartCounts.abortRxCplt == 0u);
		r.rxStreamDone();
		CHECK(uartCounts.abortRxCplt == 1u);
		CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 AbortReceive_IT with DMAR set and no Rx DMA handle clears DMAR and runs AbortReceiveCplt at once") {
	// NEEDS REVIEW 318
	UartAbortRig r(0);
	r.startRxIdleIT();
	r.inst->CR3 |= uartAbortCR3_DMAR;
	CHECK(HAL_UART_AbortReceive_IT(&r.h) == HAL_OK);
	CHECK((r.cr3() & uartAbortCR3_DMAR) == 0u);
	CHECK(uartCounts.abortRxCplt == 1u);
	CHECK(uartCounts.rxStateAtLast == HAL_UART_STATE_READY);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK(r.h.RxXferCount == 0u);
	CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 AbortReceive_IT of an Rx stream that is not busy runs AbortReceiveCplt at once") {
	// NEEDS REVIEW 318
	UartAbortRig r(3, true);
	r.startRxDMA();
	REQUIRE(HAL_DMA_Abort(&r.rx) == HAL_OK);
	CHECK(HAL_UART_AbortReceive_IT(&r.h) == HAL_OK);
	CHECK(uartCounts.abortRxCplt == 1u);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

/* The error path: an ORE during Receive_DMA aborts the Rx stream and ends in UART_DMAAbortOnError (UD3 covers the rest of the handler). */

TEST_CASE_FIXTURE(UartAbortFx, "UA2 an overrun during Receive_DMA aborts the Rx stream and calls ErrorCallback once when the stream abort completes") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startRxDMA();
		hostUartRxNow(n, 0x11u, 0u);
		hostUartRxNow(n, 0x22u, 0u); // RXNE still set: ORE
		HAL_UART_IRQHandler(&r.h);
		CHECK((r.cr3() & uartAbortCR3_DMAR) == 0u);
		CHECK(uartCounts.error == 0u); // after the DMA abort callback
		r.rxStreamDone();
		CHECK(uartCounts.error == 1u);
		CHECK((uartCounts.errorCodeAtError & HAL_UART_ERROR_ORE) != 0u);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(r.h.RxXferCount == 0u);
		CHECK(!UartAbortRig::streamOn(r.rx));
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 an overrun during Receive_DMA with an Rx stream that is not busy calls ErrorCallback at once") {
	UartAbortRig r(1, true);
	r.startRxDMA();
	REQUIRE(HAL_DMA_Abort(&r.rx) == HAL_OK);
	hostUartRxNow(1, 0x11u, 0u);
	hostUartRxNow(1, 0x22u, 0u);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 1u);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 an overrun with DMAR set and no Rx DMA handle calls ErrorCallback at once") {
	UartAbortRig r(1);
	r.startRxIT();
	r.inst->CR3 |= uartAbortCR3_DMAR;
	hostUartRxNow(1, 0x11u, 0u);
	hostUartRxNow(1, 0x22u, 0u);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 1u);
	CHECK((r.cr3() & uartAbortCR3_DMAR) == 0u);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

/* ------------------------------------------------------------------------------------------------------------------ UA3 */

TEST_CASE_FIXTURE(UartAbortFx, "UA3 Abort_IT of Transmit_DMA and Receive_DMA runs a single AbortCplt after both streams finished (Tx first)") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxDMA();
		r.startRxDMA();
		CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
		CHECK(uartCounts.abortCplt == 0u);
		r.txStreamDone();
		CHECK(uartCounts.abortCplt == 0u); // Rx stream still aborting
		CHECK(r.h.RxState != HAL_UART_STATE_READY);
		r.rxStreamDone();
		CHECK(uartCounts.abortCplt == 1u);
		CHECK(uartCounts.gStateAtLast == HAL_UART_STATE_READY);
		CHECK(uartCounts.rxStateAtLast == HAL_UART_STATE_READY);
		CHECK((r.cr3() & (uartAbortCR3_DMAT | uartAbortCR3_DMAR | uartAbortCR3_EIE)) == 0u);
		CHECK(!UartAbortRig::streamOn(r.tx));
		CHECK(!UartAbortRig::streamOn(r.rx));
		CHECK(uartCounts.abortTxCplt + uartCounts.abortRxCplt == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA3 Abort_IT of Transmit_DMA and Receive_DMA runs a single AbortCplt after both streams finished (Rx first)") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxDMA();
		r.startRxDMA();
		CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
		r.rxStreamDone();
		CHECK(uartCounts.abortCplt == 0u);
		CHECK(r.h.gState != HAL_UART_STATE_READY);
		r.txStreamDone();
		CHECK(uartCounts.abortCplt == 1u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA3 Abort_IT of Transmit_IT and Receive_DMA runs a single AbortCplt after the Rx stream finished") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxIT();
		r.startRxDMA();
		CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortTxIEs) == 0u);
		CHECK(uartCounts.abortCplt == 0u);
		r.rxStreamDone();
		CHECK(uartCounts.abortCplt == 1u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA3 Abort_IT of Transmit_DMA and Receive_IT runs a single AbortCplt after the Tx stream finished") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxDMA();
		r.startRxIT();
		CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortRxIEs1) == 0u);
		CHECK(uartCounts.abortCplt == 0u);
		r.txStreamDone();
		CHECK(uartCounts.abortCplt == 1u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA3 blocking Abort of Transmit_DMA and Receive_DMA ends both, stops both streams and runs no callback") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxDMA();
		r.startRxDMA();
		REQUIRE(HAL_UART_GetState(&r.h) == HAL_UART_STATE_BUSY_TX_RX);
		CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
		CHECK(!UartAbortRig::streamOn(r.tx));
		CHECK(!UartAbortRig::streamOn(r.rx));
		CHECK((r.cr3() & (uartAbortCR3_DMAT | uartAbortCR3_DMAR | uartAbortCR3_EIE)) == 0u);
		CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
		CHECK(r.callbacksTotal() == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA3 blocking Abort of Transmit_IT and Receive_DMA ends both") {
	UartAbortRig r(1, true);
	r.startTxIT();
	r.startRxDMA();
	CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
	CHECK(!UartAbortRig::streamOn(r.rx));
	CHECK((r.cr1() & uartAbortTxIEs) == 0u);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
	CHECK(r.callbacksTotal() == 0u);
}

/* ------------------------------------------------------------------------------------------------------------------ UA4 */

TEST_CASE_FIXTURE(UartAbortFx, "UA4 AbortTransmit leaves a running Receive_IT untouched and it still completes") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, false, UART_PARITY_EVEN);
		r.startTxIT();
		r.startRxIT();
		const uint32_t rxBits = r.cr1() & (uartAbortCR1_RXNEIE | uartAbortCR1_PEIE);
		REQUIRE(HAL_UART_AbortTransmit(&r.h) == HAL_OK);
		CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK((r.cr1() & (uartAbortCR1_RXNEIE | uartAbortCR1_PEIE)) == rxBits);
		CHECK((r.cr3() & uartAbortCR3_EIE) != 0u);
		// even parity: frame bit 7 is the parity of the 7 data bits; 0x41 has two ones, parity bit 0
		for (unsigned i = 0; i < 4; ++i) {
			hostUartRxNow(n, 0x41u, 0u);
			HAL_UART_IRQHandler(&r.h);
		}
		CHECK(uartCounts.rxCplt == 1u);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK(r.rxBuf[0] == 0x41u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA4 AbortTransmit_IT leaves a running Receive_IT untouched and runs only AbortTransmitCplt") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxIT();
		r.startRxIT();
		REQUIRE(HAL_UART_AbortTransmit_IT(&r.h) == HAL_OK);
		CHECK(uartCounts.abortTxCplt == 1u);
		CHECK(uartCounts.abortCplt == 0u);
		CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
		CHECK((r.cr1() & uartAbortCR1_RXNEIE) != 0u);
		CHECK((r.cr3() & uartAbortCR3_EIE) != 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA4 AbortTransmit of a Transmit_DMA leaves a running Receive_DMA untouched and it still completes") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxDMA();
		r.startRxDMA();
		REQUIRE(HAL_UART_AbortTransmit(&r.h) == HAL_OK);
		CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
		CHECK((r.cr3() & uartAbortCR3_DMAR) != 0u);
		CHECK((r.cr3() & uartAbortCR3_EIE) != 0u);
		CHECK(UartAbortRig::streamOn(r.rx));
		hostDmaComplete(r.m.c, r.m.rxS);
		r.rxStreamDone();
		CHECK(uartCounts.rxCplt == 1u);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA4 AbortTransmit_IT of a Transmit_DMA leaves a running Receive_DMA untouched") {
	UartAbortRig r(1, true);
	r.startTxDMA();
	r.startRxDMA();
	REQUIRE(HAL_UART_AbortTransmit_IT(&r.h) == HAL_OK);
	r.txStreamDone();
	CHECK(uartCounts.abortTxCplt == 1u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK((r.cr3() & uartAbortCR3_DMAR) != 0u);
	CHECK(UartAbortRig::streamOn(r.rx));
	CHECK(r.rx.State == HAL_DMA_STATE_BUSY);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA4 AbortReceive leaves a running Transmit_IT untouched and it still completes") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		r.startTxIT();
		r.startRxIT();
		REQUIRE(HAL_UART_AbortReceive(&r.h) == HAL_OK);
		CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
		CHECK(r.h.RxState == HAL_UART_STATE_READY);
		CHECK((r.cr1() & uartAbortCR1_TXEIE) != 0u);
		for (unsigned guard = 0; guard < 100u && uartCounts.txCplt == 0u; ++guard) {
			HAL_GetTick(); // one frame time
			HAL_UART_IRQHandler(&r.h);
		}
		CHECK(uartCounts.txCplt == 1u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		REQUIRE(hostUartTxCount(n) == 4u);
		CHECK(hostUartTxAt(n, 0) == 0x30u);
		CHECK(hostUartTxAt(n, 3) == 0x33u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA4 AbortReceive_IT leaves a running Transmit_IT untouched and runs only AbortReceiveCplt") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxIT();
		r.startRxIT();
		REQUIRE(HAL_UART_AbortReceive_IT(&r.h) == HAL_OK);
		CHECK(uartCounts.abortRxCplt == 1u);
		CHECK(uartCounts.abortCplt == 0u);
		CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
		CHECK((r.cr1() & uartAbortCR1_TXEIE) != 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA4 AbortReceive of a Receive_DMA leaves a running Transmit_DMA untouched and it still completes") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxDMA();
		r.startRxDMA();
		REQUIRE(HAL_UART_AbortReceive(&r.h) == HAL_OK);
		CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
		CHECK((r.cr3() & uartAbortCR3_DMAT) != 0u);
		CHECK(UartAbortRig::streamOn(r.tx));
		CHECK(!UartAbortRig::streamOn(r.rx));
		hostDmaComplete(r.m.c, r.m.txS);
		r.txStreamDone();
		CHECK((r.cr3() & uartAbortCR3_DMAT) == 0u); // UM1725: Tx DMA complete clears DMAT (UD2)
		CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX); // TC interrupt still to come
		CHECK((r.cr1() & uartAbortCR1_TCIE) != 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA4 AbortReceive_IT of a Receive_DMA leaves a running Transmit_DMA untouched") {
	UartAbortRig r(1, true);
	r.startTxDMA();
	r.startRxDMA();
	REQUIRE(HAL_UART_AbortReceive_IT(&r.h) == HAL_OK);
	r.rxStreamDone();
	CHECK(uartCounts.abortRxCplt == 1u);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
	CHECK((r.cr3() & uartAbortCR3_DMAT) != 0u);
	CHECK(UartAbortRig::streamOn(r.tx));
	CHECK(r.tx.State == HAL_DMA_STATE_BUSY);
}

/* ------------------------------------------------------------------------------------------------------------------ fix pass: TCIE / TXEIE pending, ErrorCode, stale callbacks, counters */

namespace {
/* Transmit_IT of one frame, then one handler call: the frame is sent, TXEIE is off and TCIE is on (RM0090 30.3.2). */
void uartAbortTcPending(UartAbortRig &r) {
	REQUIRE(HAL_UART_Transmit_IT(&r.h, r.txBuf, 1) == HAL_OK);
	HAL_UART_IRQHandler(&r.h);
	REQUIRE((r.cr1() & uartAbortCR1_TCIE) != 0u);
	REQUIRE((r.cr1() & uartAbortCR1_TXEIE) == 0u);
}
/* Tx DMA ran to its end: UART_DMATransmitCplt cleared DMAT and set TCIE; gState stays BUSY_TX until the TC interrupt. */
void uartAbortDmaTcPending(UartAbortRig &r) {
	r.startTxDMA();
	hostDmaComplete(r.m.c, r.m.txS);
	r.txStreamDone();
	REQUIRE((r.cr1() & uartAbortCR1_TCIE) != 0u);
	REQUIRE((r.cr3() & uartAbortCR3_DMAT) == 0u);
	REQUIRE(r.h.gState == HAL_UART_STATE_BUSY_TX);
}
} // namespace

TEST_CASE_FIXTURE(UartAbortFx, "UA1 AbortTransmit with only TCIE pending (last frame loaded) clears TCIE and sets gState READY") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		uartAbortTcPending(r);
		CHECK(HAL_UART_AbortTransmit(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortTxIEs) == 0u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 AbortTransmit after a Tx DMA completion (TCIE pending, DMAT clear) clears TCIE") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		uartAbortDmaTcPending(r);
		CHECK(HAL_UART_AbortTransmit(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortTxIEs) == 0u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 Abort after a Tx DMA completion (TCIE pending) clears TCIE") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		uartAbortDmaTcPending(r);
		CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortTxIEs) == 0u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
		CHECK(r.callbacksTotal() == 0u);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT with only TCIE pending clears TCIE and runs AbortCplt once at once") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		uartAbortTcPending(r);
		CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortTxIEs) == 0u);
		CHECK(uartCounts.abortCplt == 1u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT after a Tx DMA completion (TCIE pending) clears TCIE and runs AbortCplt once at once") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		uartAbortDmaTcPending(r);
		CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortTxIEs) == 0u);
		CHECK(uartCounts.abortCplt == 1u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 AbortTransmit_IT with only TCIE pending clears TCIE and runs AbortTransmitCplt once at once") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		uartAbortTcPending(r);
		CHECK(HAL_UART_AbortTransmit_IT(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortTxIEs) == 0u);
		CHECK(uartCounts.abortTxCplt == 1u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 AbortTransmit_IT after a Tx DMA completion (TCIE pending) clears TCIE and runs AbortTransmitCplt at once") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		uartAbortDmaTcPending(r);
		CHECK(HAL_UART_AbortTransmit_IT(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortTxIEs) == 0u);
		CHECK(uartCounts.abortTxCplt == 1u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 DMAStop of a Transmit_DMA clears TXEIE and TCIE, DMAT, stops the stream and sets gState READY") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxDMA();
		r.inst->CR1 |= uartAbortCR1_TXEIE | uartAbortCR1_TCIE; // both enables seeded to observe the clearing
		CHECK(HAL_UART_DMAStop(&r.h) == HAL_OK);
		CHECK((r.cr1() & uartAbortTxIEs) == 0u);
		CHECK((r.cr3() & uartAbortCR3_DMAT) == 0u);
		CHECK(!UartAbortRig::streamOn(r.tx));
		CHECK(r.h.gState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 a Tx DMA stream error clears TXEIE and TCIE, sets gState READY and runs ErrorCallback with the DMA code") {
	UART_ABORT_EACH(n) {
		UartAbortRig r(n, true);
		r.startTxDMA();
		r.inst->CR1 |= uartAbortCR1_TXEIE | uartAbortCR1_TCIE;
		hostDmaRaise(r.m.c, r.m.txS, HOST_DMA_TEIF);
		r.txStreamDone();
		CHECK(uartCounts.error == 1u);
		CHECK((uartCounts.errorCodeAtError & HAL_UART_ERROR_DMA) != 0u);
		CHECK((r.cr1() & uartAbortTxIEs) == 0u);
		CHECK(r.h.gState == HAL_UART_STATE_READY);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 characterization: Abort_IT without DMA resets a left over ErrorCode to NONE") {
	// NEEDS REVIEW 318: the code is seeded after the transfer started.
	UART_ABORT_EACH(n) {
		UartAbortRig r(n);
		r.startRxIT();
		r.h.ErrorCode = HAL_UART_ERROR_FE;
		CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
		CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
	}
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 characterization: the Tx DMA abort callback of Abort_IT resets a left over ErrorCode to NONE") {
	// NEEDS REVIEW 318
	UartAbortRig r(1, true);
	r.startTxDMA();
	r.h.ErrorCode = HAL_UART_ERROR_PE;
	REQUIRE(HAL_UART_Abort_IT(&r.h) == HAL_OK);
	r.txStreamDone();
	REQUIRE(uartCounts.abortCplt == 1u);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 characterization: the Rx DMA abort callback of Abort_IT resets a left over ErrorCode to NONE") {
	// NEEDS REVIEW 318
	UartAbortRig r(1, true);
	r.startRxDMA();
	r.h.ErrorCode = HAL_UART_ERROR_NE;
	REQUIRE(HAL_UART_Abort_IT(&r.h) == HAL_OK);
	r.rxStreamDone();
	REQUIRE(uartCounts.abortCplt == 1u);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 characterization: AbortTransmit_IT and AbortReceive_IT (also their DMA callbacks) leave a left over ErrorCode") {
	// NEEDS REVIEW 318
	UartAbortRig r(1, true);
	r.startTxIT();
	r.h.ErrorCode = HAL_UART_ERROR_FE;
	REQUIRE(HAL_UART_AbortTransmit_IT(&r.h) == HAL_OK);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_FE);
	r.startTxDMA();
	r.h.ErrorCode = HAL_UART_ERROR_FE;
	REQUIRE(HAL_UART_AbortTransmit_IT(&r.h) == HAL_OK);
	r.txStreamDone();
	REQUIRE(uartCounts.abortTxCplt == 2u);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_FE);
	r.startRxDMA();
	r.h.ErrorCode = HAL_UART_ERROR_FE;
	REQUIRE(HAL_UART_AbortReceive_IT(&r.h) == HAL_OK);
	r.rxStreamDone();
	REQUIRE(uartCounts.abortRxCplt == 1u);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_FE);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT with DMAT clear overwrites a stale Tx abort callback with NULL") {
	// A Tx DMA abort started by AbortTransmit_IT and never completed leaves UART_DMATxOnlyAbortCallback in the handle.
	UartAbortRig r(1, true);
	r.startTxDMA();
	REQUIRE(HAL_UART_AbortTransmit_IT(&r.h) == HAL_OK);
	REQUIRE(r.tx.XferAbortCallback != nullptr);
	REQUIRE(HAL_UART_AbortTransmit(&r.h) == HAL_OK); // state READY again, DMAT already clear: callback untouched
	REQUIRE(r.tx.XferAbortCallback != nullptr);
	r.startTxIT();
	CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
	CHECK(r.tx.XferAbortCallback == nullptr);
	CHECK(uartCounts.abortCplt == 1u);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 Abort_IT with DMAR clear overwrites a stale Rx abort callback with NULL") {
	UartAbortRig r(1, true);
	r.startRxDMA();
	REQUIRE(HAL_UART_AbortReceive_IT(&r.h) == HAL_OK);
	REQUIRE(r.rx.XferAbortCallback != nullptr);
	REQUIRE(HAL_UART_AbortReceive(&r.h) == HAL_OK);
	REQUIRE(r.rx.XferAbortCallback != nullptr);
	r.startRxIT();
	CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
	CHECK(r.rx.XferAbortCallback == nullptr);
	CHECK(uartCounts.abortCplt == 1u);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 characterization: blocking Abort sets the transfer counters to 0 and ReceptionType to STANDARD") {
	// NEEDS REVIEW 318
	UartAbortRig r(1, true);
	r.startTxDMA();
	r.startRxIdleDMA();
	r.h.RxXferCount = 3; // the HAL does not count Rx in DMA mode: seeded
	REQUIRE(r.h.TxXferCount == 4u);
	CHECK(HAL_UART_Abort(&r.h) == HAL_OK);
	CHECK(r.h.TxXferCount == 0u);
	CHECK(r.h.RxXferCount == 0u);
	CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA1 characterization: AbortTransmit sets TxXferCount to 0 and AbortReceive sets RxXferCount to 0") {
	// NEEDS REVIEW 318
	UartAbortRig r(1);
	r.startTxIT();
	r.startRxIdleIT();
	REQUIRE(r.h.TxXferCount == 4u);
	REQUIRE(r.h.RxXferCount == 8u);
	CHECK(HAL_UART_AbortTransmit(&r.h) == HAL_OK);
	CHECK(r.h.TxXferCount == 0u);
	CHECK(r.h.RxXferCount == 8u);
	CHECK(HAL_UART_AbortReceive(&r.h) == HAL_OK);
	CHECK(r.h.RxXferCount == 0u);
	CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 characterization: Abort_IT and the Only variants set the transfer counters to 0 and ReceptionType to STANDARD") {
	// NEEDS REVIEW 318
	UartAbortRig r(1);
	r.startTxIT();
	r.startRxIdleIT();
	CHECK(HAL_UART_AbortTransmit_IT(&r.h) == HAL_OK);
	CHECK(r.h.TxXferCount == 0u);
	CHECK(HAL_UART_AbortReceive_IT(&r.h) == HAL_OK);
	CHECK(r.h.RxXferCount == 0u);
	CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
	r.startTxIT();
	r.startRxIdleIT();
	CHECK(HAL_UART_Abort_IT(&r.h) == HAL_OK);
	CHECK(r.h.TxXferCount == 0u);
	CHECK(r.h.RxXferCount == 0u);
	CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
}

TEST_CASE_FIXTURE(UartAbortFx, "UA2 characterization: the DMA abort callbacks set the transfer counters to 0 and ReceptionType to STANDARD") {
	// NEEDS REVIEW 318
	UartAbortRig r(1, true);
	r.startTxDMA();
	r.startRxIdleDMA();
	r.h.RxXferCount = 3;
	REQUIRE(HAL_UART_Abort_IT(&r.h) == HAL_OK);
	r.txStreamDone();
	r.rxStreamDone();
	REQUIRE(uartCounts.abortCplt == 1u);
	CHECK(r.h.TxXferCount == 0u);
	CHECK(r.h.RxXferCount == 0u);
	CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
	r.startTxDMA();
	r.startRxIdleDMA();
	r.h.RxXferCount = 3;
	REQUIRE(HAL_UART_AbortTransmit_IT(&r.h) == HAL_OK);
	r.txStreamDone();
	CHECK(r.h.TxXferCount == 0u);
	REQUIRE(HAL_UART_AbortReceive_IT(&r.h) == HAL_OK);
	r.rxStreamDone();
	CHECK(r.h.RxXferCount == 0u);
	CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
}
