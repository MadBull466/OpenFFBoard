/*
 * P12 T4a group SQ: SPI interrupt mode (HAL_SPI_Transmit_IT / Receive_IT / TransmitReceive_IT and HAL_SPI_IRQHandler with its
 * callbacks) of stm32f4xx_hal_spi.c against the SPI register model (fakes/stm32_hal/HalSpi.h). Requirements SQ1-SQ5 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T4A_REQUIREMENTS.md; compiled into ffb_hal_tests and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 chapter 66 (SPI: the Tx / Rx / TxRx complete callbacks run at the end of the process, the error
 * callback when a communication error is detected; error code names) and RM0090 Rev 22 chapter 28 (28.3.5 transmission
 * and reception, 28.3.8 BSY, 28.3.10 error flags and their clearing sequences, 28.3.11 Table 126 interrupt enables).
 *
 * Interrupts are not delivered by the model: the tests call HAL_SPI_IRQHandler in a loop (spiIrqPump) and advance the HAL tick
 * between the calls, because a frame of the model ends after its tick based frame time.
 *
 * NEEDS REVIEW proposals (characterization cases, the lead assigns the numbers, proposals use SQ-n):
 *  SQ-1 characterization: the *_IT start functions set ERRIE together with TXEIE / RXNEIE. RM0090 Table 126 allows it; UM1725
 *       is silent.
 *  SQ-2 characterization: HAL_SPI_*_IT with a NULL buffer or Size 0 returns HAL_ERROR, with a busy handle HAL_BUSY, the failed
 *       call leaves the handle unlocked. UM1725 gives only "HAL: status".
 *  SQ-3 characterization: the state after an error callback in IT mode is READY. UM1725 is silent.
 *  SQ-4 characterization: the error code stays set after the error callback (HAL_SPI_GetError). UM1725 is silent.
 *  SQ-5 characterization: HAL_SPI_Receive_IT of a 2 line master is reported as RxCplt (not TxRxCplt). UM1725 only says the
 *       callback of the "Receive" process.
 *  SQ-6 characterization: the state after an error in the middle of a transfer (data not complete): the transfer is closed.
 *  SQ-8 characterization: after an IT transfer ends (and after an abort) ERRIE is cleared together with TXEIE / RXNEIE. UM1725
 *       and RM0090 are silent.
 *  SQ-9 characterization: HAL_SPI_Abort / HAL_SPI_Abort_IT of an IT transfer wait inside the call for the SPI interrupt (the abort
 *       ISR sets State = ABORT). The test host delivers no interrupts, so the wait times out: HAL_ERROR, HAL_SPI_ERROR_ABORT in the
 *       error code, State READY, TXEIE / RXNEIE / SPE left as they were, and Abort_IT still calls AbortCpltCallback once. UM1725 says
 *       "Abort is considered as completed when exiting function" but names no timeout. The successful path (interrupt preempting the
 *       wait) cannot be reached on the host: reported to the lead as untestable.
 *  SQ-10 characterization: an abort ISR that runs after the timed out Abort_IT (handler call) clears TXEIE / RXNEIE / ERRIE and SPE.
 */
#include "HalFixture.h"
#include "HalSpi.h"
#include "SpiTestSupport.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstring>

namespace {

struct SpiIrqRig {
	SPI_HandleTypeDef h;
	unsigned idx;
	SPI_TypeDef *inst() { return h.Instance; }
};

SPI_TypeDef *spiIrqInstance(unsigned i) {
	return i == 0 ? SPI1 : (i == 1 ? SPI2 : SPI3);
}

void spiIrqInit(SpiIrqRig &r, unsigned idx, uint32_t mode, uint32_t dir, bool wide, uint32_t ti = SPI_TIMODE_DISABLE) {
	std::memset(&r.h, 0, sizeof r.h);
	r.idx = idx;
	r.h.Instance = spiIrqInstance(idx);
	r.h.Init.Mode = mode;
	r.h.Init.Direction = dir;
	r.h.Init.DataSize = wide ? SPI_DATASIZE_16BIT : SPI_DATASIZE_8BIT;
	r.h.Init.CLKPolarity = SPI_POLARITY_LOW;
	r.h.Init.CLKPhase = SPI_PHASE_1EDGE;
	r.h.Init.NSS = SPI_NSS_SOFT;
	r.h.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	r.h.Init.FirstBit = SPI_FIRSTBIT_MSB;
	r.h.Init.TIMode = ti;
	r.h.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	r.h.Init.CRCPolynomial = 7;
	REQUIRE(HAL_SPI_Init(&r.h) == HAL_OK);
	spiSupportReset();
	hostSpiTxClear(idx);
}

/* Frame data: 8 bit frames are bytes, 16 bit frames are uint16_t, both in an aligned byte buffer. */
struct SpiIrqBuf {
	alignas(4) uint8_t raw[64];
	bool wide;
	SpiIrqBuf(bool w) : wide(w) { std::memset(raw, 0, sizeof raw); }
	uint16_t get(unsigned k) const {
		if (!wide) return raw[k];
		uint16_t v;
		std::memcpy(&v, raw + 2 * k, 2);
		return v;
	}
	void set(unsigned k, uint16_t v) {
		if (!wide) { raw[k] = (uint8_t)v; return; }
		std::memcpy(raw + 2 * k, &v, 2);
	}
};

/* Frame word k of a pattern: 8 bit (0xA0 + k) and 16 bit (0xA55A + 0x0111 * k) differ in every frame. */
uint16_t spiIrqWord(bool wide, unsigned k, unsigned salt = 0) {
	return wide ? (uint16_t)(0xA55Au + 0x0111u * k + salt) : (uint16_t)(0xA0u + k + salt);
}

void spiIrqPump(SPI_HandleTypeDef *h, unsigned maxIter) {
	for (unsigned i = 0; i < maxIter && h->State != HAL_SPI_STATE_READY; ++i) {
		hostTickSet(hostTickPeek() + 1U);
		HAL_SPI_IRQHandler(h);
	}
}

uint32_t spiIrqCr2(SpiIrqRig &r) { return HalFixture::peek(&r.inst()->CR2); }
uint32_t spiIrqSr(SpiIrqRig &r) { return HalFixture::peek(&r.inst()->SR); }

unsigned spiIrqCompletions() { return spiCounts.txCplt + spiCounts.rxCplt + spiCounts.txRxCplt; }

/* Master 2 lines handle, transfer prepared in the buffers. */
void spiIrqPeer(SpiIrqRig &r, bool wide, unsigned n, unsigned salt) {
	for (unsigned k = 0; k < n; ++k) hostSpiPeerPush(r.idx, spiIrqWord(wide, k, salt));
}

/* ---- bodies, run for every instance / width / size ---- */

void spiIrqTxData(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	SpiIrqBuf b(wide);
	for (unsigned k = 0; k < n; ++k) b.set(k, spiIrqWord(wide, k));
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	REQUIRE(hostSpiTxCount(idx) == n);
	for (unsigned k = 0; k < n; ++k) {
		CAPTURE(k);
		CHECK(hostSpiTxAt(idx, k) == spiIrqWord(wide, k));
	}
}

void spiIrqTxDone(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	SpiIrqBuf b(wide);
	for (unsigned k = 0; k < n; ++k) b.set(k, spiIrqWord(wide, k));
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	CHECK(r.h.State == HAL_SPI_STATE_READY);
	CHECK(spiCounts.txCplt == 1);
	CHECK(spiCounts.rxCplt == 0);
	CHECK(spiCounts.txRxCplt == 0);
	CHECK(spiCounts.error == 0);
	CHECK(spiCounts.last == &r.h);
	for (int i = 0; i < 4; ++i) HAL_SPI_IRQHandler(&r.h); /* nothing is pending any more */
	CHECK(spiCounts.txCplt == 1);
	CHECK(hostSpiTxCount(idx) == n);
}

void spiIrqTxDisables(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	SpiIrqBuf b(wide);
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	REQUIRE(r.h.State == HAL_SPI_STATE_READY);
	CHECK((spiIrqCr2(r) & (SPI_CR2_TXEIE | SPI_CR2_RXNEIE)) == 0);
}

void spiIrqTxOneFramePerCall(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	SpiIrqBuf b(wide);
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	unsigned calls = 0;
	for (; calls < 10 * n + 20 && r.h.State != HAL_SPI_STATE_READY; ++calls) {
		hostTickSet(hostTickPeek() + 1U);
		HAL_SPI_IRQHandler(&r.h);
		CAPTURE(calls);
		REQUIRE(hostSpiTxCount(idx) <= calls + 1); /* at most one frame written per call */
	}
	CHECK(hostSpiTxCount(idx) == n);
}

void spiIrqRxData(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	spiIrqPeer(r, wide, n, 0);
	SpiIrqBuf b(wide);
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	for (unsigned k = 0; k < n; ++k) {
		CAPTURE(k);
		CHECK(b.get(k) == spiIrqWord(wide, k));
	}
}

void spiIrqRxDone(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	spiIrqPeer(r, wide, n, 0);
	SpiIrqBuf b(wide);
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	CHECK(r.h.State == HAL_SPI_STATE_READY);
	CHECK(spiIrqCompletions() == 1);
	CHECK(spiCounts.rxCplt == 1);
	CHECK(spiCounts.error == 0);
	CHECK(spiCounts.last == &r.h);
	for (int i = 0; i < 4; ++i) HAL_SPI_IRQHandler(&r.h);
	CHECK(spiIrqCompletions() == 1);
}

void spiIrqRxDisables(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	spiIrqPeer(r, wide, n, 0);
	SpiIrqBuf b(wide);
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	REQUIRE(r.h.State == HAL_SPI_STATE_READY);
	CHECK((spiIrqCr2(r) & (SPI_CR2_TXEIE | SPI_CR2_RXNEIE)) == 0);
}

void spiIrqTxRxData(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	spiIrqPeer(r, wide, n, 7);
	SpiIrqBuf tx(wide), rx(wide);
	for (unsigned k = 0; k < n; ++k) tx.set(k, spiIrqWord(wide, k));
	REQUIRE(HAL_SPI_TransmitReceive_IT(&r.h, tx.raw, rx.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	REQUIRE(hostSpiTxCount(idx) == n);
	for (unsigned k = 0; k < n; ++k) {
		CAPTURE(k);
		CHECK(hostSpiTxAt(idx, k) == spiIrqWord(wide, k));
		CHECK(rx.get(k) == spiIrqWord(wide, k, 7));
	}
}

void spiIrqTxRxLoopback(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	hostSpiPeerLoopback(idx, 1);
	SpiIrqBuf tx(wide), rx(wide);
	for (unsigned k = 0; k < n; ++k) tx.set(k, spiIrqWord(wide, k));
	REQUIRE(HAL_SPI_TransmitReceive_IT(&r.h, tx.raw, rx.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	for (unsigned k = 0; k < n; ++k) {
		CAPTURE(k);
		CHECK(rx.get(k) == tx.get(k));
	}
}

void spiIrqTxRxDone(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	spiIrqPeer(r, wide, n, 7);
	SpiIrqBuf tx(wide), rx(wide);
	REQUIRE(HAL_SPI_TransmitReceive_IT(&r.h, tx.raw, rx.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	CHECK(r.h.State == HAL_SPI_STATE_READY);
	CHECK(spiCounts.txRxCplt == 1);
	CHECK(spiIrqCompletions() == 1);
	CHECK(spiCounts.error == 0);
	CHECK(spiCounts.last == &r.h);
	for (int i = 0; i < 4; ++i) HAL_SPI_IRQHandler(&r.h);
	CHECK(spiIrqCompletions() == 1);
}

void spiIrqTxRxDisables(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	SpiIrqBuf tx(wide), rx(wide);
	REQUIRE(HAL_SPI_TransmitReceive_IT(&r.h, tx.raw, rx.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	REQUIRE(r.h.State == HAL_SPI_STATE_READY);
	CHECK((spiIrqCr2(r) & (SPI_CR2_TXEIE | SPI_CR2_RXNEIE)) == 0);
}

/* Master receive only (SQ5). In receive only mode the clock runs on after the last frame was read (RM0090 28.3.5), so the HAL has to
 * stop it at once. The handler reads the HAL tick (one tick of the model) before it disables SPE; with the default frame time of one
 * tick the extra frame would end first and leave RXNE set: a model artefact, the real frame takes many core cycles. A frame of four
 * ticks lets the extra frame still be running when SPE is cleared, as on the target. */
const unsigned kSpiIrqRxOnlyFrameTicks = 4;
void spiIrqRxOnlyData(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES_RXONLY, wide);
	hostSpiSetFrameTicks(kSpiIrqRxOnlyFrameTicks); /* see the comment at the constant */
	spiIrqPeer(r, wide, n, 3);
	SpiIrqBuf b(wide);
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	CHECK(spiCounts.rxCplt == 1);
	for (unsigned k = 0; k < n; ++k) {
		CAPTURE(k);
		CHECK(b.get(k) == spiIrqWord(wide, k, 3));
	}
}

void spiIrqRxOnlyClean(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES_RXONLY, wide);
	hostSpiSetFrameTicks(kSpiIrqRxOnlyFrameTicks);
	spiIrqPeer(r, wide, n, 3);
	SpiIrqBuf b(wide);
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	REQUIRE(r.h.State == HAL_SPI_STATE_READY);
	CHECK((spiIrqSr(r) & SPI_SR_OVR) == 0);
	CHECK(spiCounts.error == 0);
	CHECK(r.h.ErrorCode == HAL_SPI_ERROR_NONE);
}

/* Slave (SQ2). hostSpiSlaveFrame stands in for the master's clock. */
void spiIrqSlaveRx(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, wide);
	SpiIrqBuf b(wide);
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	for (unsigned k = 0; k < n; ++k) {
		hostSpiSlaveFrame(idx, spiIrqWord(wide, k, 5));
		HAL_SPI_IRQHandler(&r.h); /* RXNE: reads the frame */
		CAPTURE(k);
		CHECK(b.get(k) == spiIrqWord(wide, k, 5));
	}
	CHECK(r.h.State == HAL_SPI_STATE_READY);
	CHECK(spiCounts.rxCplt == 1);
	CHECK(spiIrqCompletions() == 1);
	CHECK(spiCounts.error == 0);
}

void spiIrqSlaveTxRx(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, wide);
	SpiIrqBuf tx(wide), rx(wide);
	for (unsigned k = 0; k < n; ++k) tx.set(k, spiIrqWord(wide, k));
	REQUIRE(HAL_SPI_TransmitReceive_IT(&r.h, tx.raw, rx.raw, (uint16_t)n) == HAL_OK);
	unsigned sent = 0;
	uint16_t out[16] = {0};
	for (unsigned g = 0; g < 10 * n + 20 && r.h.State != HAL_SPI_STATE_READY; ++g) {
		HAL_SPI_IRQHandler(&r.h);
		if (sent < n && (spiIrqSr(r) & SPI_SR_TXE) == 0 && (spiIrqSr(r) & SPI_SR_RXNE) == 0) {
			out[sent] = hostSpiSlaveFrame(idx, spiIrqWord(wide, sent, 5));
			++sent;
		}
	}
	REQUIRE(sent == n);
	for (unsigned k = 0; k < n; ++k) {
		CAPTURE(k);
		CHECK(out[k] == spiIrqWord(wide, k));
		CHECK(rx.get(k) == spiIrqWord(wide, k, 5));
	}
	CHECK(r.h.State == HAL_SPI_STATE_READY);
	CHECK(spiCounts.txRxCplt == 1);
	CHECK(spiIrqCompletions() == 1);
}

#define SPI_IRQ_SIZES(fn, i, w)                                                                                                   \
	SUBCASE("size 1") { fn(i, w, 1); }                                                                                            \
	SUBCASE("size 2") { fn(i, w, 2); }                                                                                            \
	SUBCASE("size 3") { fn(i, w, 3); }                                                                                            \
	SUBCASE("size 16") { fn(i, w, 16); }
#define SPI_IRQ_WIDTHS(fn, i)                                                                                                     \
	SUBCASE("8 bit") { SPI_IRQ_SIZES(fn, i, false) }                                                                              \
	SUBCASE("16 bit") { SPI_IRQ_SIZES(fn, i, true) }
#define SPI_IRQ_MATRIX(fn)                                                                                                        \
	SUBCASE("SPI1") { SPI_IRQ_WIDTHS(fn, 0) }                                                                                     \
	SUBCASE("SPI2") { SPI_IRQ_WIDTHS(fn, 1) }                                                                                     \
	SUBCASE("SPI3") { SPI_IRQ_WIDTHS(fn, 2) }
#define SPI_IRQ_INSTANCES(fn)                                                                                                     \
	SUBCASE("SPI1") { fn(0); }                                                                                                    \
	SUBCASE("SPI2") { fn(1); }                                                                                                    \
	SUBCASE("SPI3") { fn(2); }

} // namespace

/* ======================================================================== SQ1: start functions */

namespace {
void spiIrqStartTx(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {1, 2, 3, 4};
	CHECK(HAL_SPI_Transmit_IT(&r.h, b, 4) == HAL_OK);
	CHECK(r.h.State == HAL_SPI_STATE_BUSY_TX);
	CHECK((spiIrqCr2(r) & SPI_CR2_TXEIE) != 0);
	CHECK((spiIrqCr2(r) & SPI_CR2_RXNEIE) == 0);
	CHECK((HalFixture::peek(&r.inst()->CR1) & SPI_CR1_SPE) != 0);
}
void spiIrqStartRx(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {};
	CHECK(HAL_SPI_Receive_IT(&r.h, b, 4) == HAL_OK);
	CHECK(r.h.State == HAL_SPI_STATE_BUSY_RX);
	CHECK((spiIrqCr2(r) & SPI_CR2_RXNEIE) != 0);
	CHECK((HalFixture::peek(&r.inst()->CR1) & SPI_CR1_SPE) != 0);
}
void spiIrqStartTxRx(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t t[4] = {1, 2, 3, 4}, b[4] = {};
	CHECK(HAL_SPI_TransmitReceive_IT(&r.h, t, b, 4) == HAL_OK);
	CHECK(r.h.State == HAL_SPI_STATE_BUSY_TX_RX);
	CHECK((spiIrqCr2(r) & SPI_CR2_TXEIE) != 0);
	CHECK((spiIrqCr2(r) & SPI_CR2_RXNEIE) != 0);
	CHECK((HalFixture::peek(&r.inst()->CR1) & SPI_CR1_SPE) != 0);
}
void spiIrqStartSlaveRx(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {};
	CHECK(HAL_SPI_Receive_IT(&r.h, b, 4) == HAL_OK);
	CHECK(r.h.State == HAL_SPI_STATE_BUSY_RX);
	CHECK((spiIrqCr2(r) & SPI_CR2_RXNEIE) != 0);
	CHECK((spiIrqCr2(r) & SPI_CR2_TXEIE) == 0);
	CHECK((HalFixture::peek(&r.inst()->CR1) & SPI_CR1_SPE) != 0);
}
void spiIrqStartErrie(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {};
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b, 4) == HAL_OK);
	CHECK((spiIrqCr2(r) & SPI_CR2_ERRIE) != 0);
}
void spiIrqStartNullTx(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {};
	CHECK(HAL_SPI_Transmit_IT(&r.h, nullptr, 4) == HAL_ERROR);
	CHECK(HAL_SPI_Transmit_IT(&r.h, b, 0) == HAL_ERROR);
	CHECK(r.h.State == HAL_SPI_STATE_READY);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	CHECK((HalFixture::peek(&r.inst()->CR1) & SPI_CR1_SPE) == 0);
}
void spiIrqStartNullTxRx(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {};
	CHECK(HAL_SPI_TransmitReceive_IT(&r.h, nullptr, b, 4) == HAL_ERROR);
	CHECK(HAL_SPI_TransmitReceive_IT(&r.h, b, nullptr, 4) == HAL_ERROR);
	CHECK(HAL_SPI_TransmitReceive_IT(&r.h, b, b, 0) == HAL_ERROR);
	CHECK(r.h.State == HAL_SPI_STATE_READY);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	CHECK((HalFixture::peek(&r.inst()->CR1) & SPI_CR1_SPE) == 0);
}
void spiIrqStartNullSlaveRx(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {};
	CHECK(HAL_SPI_Receive_IT(&r.h, nullptr, 4) == HAL_ERROR);
	CHECK(HAL_SPI_Receive_IT(&r.h, b, 0) == HAL_ERROR);
	CHECK(r.h.State == HAL_SPI_STATE_READY);
	CHECK(r.h.Lock == HAL_UNLOCKED);
}
void spiIrqStartNullMasterRxState(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	(void)HAL_SPI_Receive_IT(&r.h, nullptr, 4); /* rejected: nothing may start */
	CHECK(r.h.State == HAL_SPI_STATE_READY);
}
void spiIrqStartNullMasterRxReturn(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {};
	CHECK(HAL_SPI_Receive_IT(&r.h, nullptr, 4) == HAL_ERROR);
	r.h.State = HAL_SPI_STATE_READY; /* undo the HAL's state so the second call is judged on its own */
	CHECK(HAL_SPI_Receive_IT(&r.h, b, 0) == HAL_ERROR);
}
void spiIrqStartBusy(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {1, 2, 3, 4}, c[4] = {};
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b, 4) == HAL_OK);
	CHECK(HAL_SPI_Transmit_IT(&r.h, b, 4) == HAL_BUSY);
	CHECK(HAL_SPI_Receive_IT(&r.h, c, 4) == HAL_BUSY);
	CHECK(HAL_SPI_TransmitReceive_IT(&r.h, b, c, 4) == HAL_BUSY);
	CHECK(r.h.State == HAL_SPI_STATE_BUSY_TX);
	CHECK(r.h.Lock == HAL_UNLOCKED);
}
void spiIrqStartTxRestartsAfter(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t b[2] = {0x31, 0x32};
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b, 2) == HAL_OK);
	spiIrqPump(&r.h, 40);
	REQUIRE(r.h.State == HAL_SPI_STATE_READY);
	b[0] = 0x41;
	b[1] = 0x42;
	CHECK(HAL_SPI_Transmit_IT(&r.h, b, 2) == HAL_OK);
	spiIrqPump(&r.h, 40);
	CHECK(spiCounts.txCplt == 2);
	CHECK(hostSpiTxCount(idx) == 4);
	CHECK(hostSpiTxAt(idx, 2) == 0x41);
	CHECK(hostSpiTxAt(idx, 3) == 0x42);
}
} // namespace

TEST_CASE_FIXTURE(SpiFixture, "SQ1 Transmit_IT starts a transfer: BUSY_TX, TXEIE and SPE set, RXNEIE clear") {
	SPI_IRQ_INSTANCES(spiIrqStartTx)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ1 Receive_IT of a 2 line master starts a transfer: BUSY_RX, RXNEIE and SPE set") {
	SPI_IRQ_INSTANCES(spiIrqStartRx)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ1 TransmitReceive_IT starts a transfer: BUSY_TX_RX, TXEIE, RXNEIE and SPE set") {
	SPI_IRQ_INSTANCES(spiIrqStartTxRx)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ1 Receive_IT of a slave starts a transfer: BUSY_RX, RXNEIE and SPE set, TXEIE clear") {
	SPI_IRQ_INSTANCES(spiIrqStartSlaveRx)
}
// NEEDS REVIEW SQ-1.
TEST_CASE_FIXTURE(SpiFixture, "SQ1 characterization: Receive_IT sets ERRIE") {
	SPI_IRQ_INSTANCES(spiIrqStartErrie)
}
// NEEDS REVIEW SQ-2.
TEST_CASE_FIXTURE(SpiFixture, "SQ1 characterization: Transmit_IT with a NULL buffer or Size 0 returns HAL_ERROR and leaves the handle ready and unlocked") {
	SPI_IRQ_INSTANCES(spiIrqStartNullTx)
}
// NEEDS REVIEW SQ-2.
TEST_CASE_FIXTURE(SpiFixture, "SQ1 characterization: TransmitReceive_IT with a NULL buffer or Size 0 returns HAL_ERROR and leaves the handle ready and unlocked") {
	SPI_IRQ_INSTANCES(spiIrqStartNullTxRx)
}
// NEEDS REVIEW SQ-2.
TEST_CASE_FIXTURE(SpiFixture, "SQ1 characterization: a slave Receive_IT with a NULL buffer or Size 0 returns HAL_ERROR and leaves the handle ready and unlocked") {
	SPI_IRQ_INSTANCES(spiIrqStartNullSlaveRx)
}
// NEEDS REVIEW SQ-2.
TEST_CASE_FIXTURE(SpiFixture, "SQ1 characterization: Receive_IT of a 2 line master with a NULL buffer or Size 0 returns HAL_ERROR") {
	SPI_IRQ_INSTANCES(spiIrqStartNullMasterRxReturn)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ1 a rejected Receive_IT of a 2 line master leaves the handle ready" * KNOWN_ISSUE("vendor bug V97: HAL_SPI_Receive_IT sets State = BUSY_RX before it hands a master 2 line call to TransmitReceive_IT; when that rejects the NULL buffer / Size 0 the state stays BUSY_RX and the next start returns HAL_BUSY (UM1725: a start function that returns HAL_ERROR does not start a transfer)")) {
	SPI_IRQ_INSTANCES(spiIrqStartNullMasterRxState)
}
// NEEDS REVIEW SQ-2.
TEST_CASE_FIXTURE(SpiFixture, "SQ1 characterization: a start while busy returns HAL_BUSY and leaves the running transfer") {
	SPI_IRQ_INSTANCES(spiIrqStartBusy)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ1 a new Transmit_IT after a finished one transfers the new data") {
	SPI_IRQ_INSTANCES(spiIrqStartTxRestartsAfter)
}

/* ======================================================================== SQ2: handler progress */

TEST_CASE_FIXTURE(SpiFixture, "SQ2 Transmit_IT puts the buffer on the bus in order") { SPI_IRQ_MATRIX(spiIrqTxData) }
TEST_CASE_FIXTURE(SpiFixture, "SQ2 Transmit_IT ends with the state ready and TxCplt exactly once") { SPI_IRQ_MATRIX(spiIrqTxDone) }
TEST_CASE_FIXTURE(SpiFixture, "SQ2 Transmit_IT clears TXEIE and RXNEIE after the last frame") { SPI_IRQ_MATRIX(spiIrqTxDisables) }
TEST_CASE_FIXTURE(SpiFixture, "SQ2 each handler call of Transmit_IT writes at most one frame") { SPI_IRQ_MATRIX(spiIrqTxOneFramePerCall) }
TEST_CASE_FIXTURE(SpiFixture, "SQ2 Receive_IT delivers the peer words in order") { SPI_IRQ_MATRIX(spiIrqRxData) }
// NEEDS REVIEW SQ-5.
TEST_CASE_FIXTURE(SpiFixture, "SQ2 characterization: Receive_IT of a 2 line master ends ready with RxCplt exactly once") {
	SPI_IRQ_MATRIX(spiIrqRxDone)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ2 Receive_IT clears TXEIE and RXNEIE after the last frame") { SPI_IRQ_MATRIX(spiIrqRxDisables) }
TEST_CASE_FIXTURE(SpiFixture, "SQ2 TransmitReceive_IT sends the buffer and receives the peer words") { SPI_IRQ_MATRIX(spiIrqTxRxData) }
TEST_CASE_FIXTURE(SpiFixture, "SQ2 TransmitReceive_IT in loopback receives what it sent") { SPI_IRQ_MATRIX(spiIrqTxRxLoopback) }
TEST_CASE_FIXTURE(SpiFixture, "SQ2 TransmitReceive_IT ends with the state ready and TxRxCplt exactly once") { SPI_IRQ_MATRIX(spiIrqTxRxDone) }
TEST_CASE_FIXTURE(SpiFixture, "SQ2 TransmitReceive_IT clears TXEIE and RXNEIE after the last frame") { SPI_IRQ_MATRIX(spiIrqTxRxDisables) }
TEST_CASE_FIXTURE(SpiFixture, "SQ2 a slave Receive_IT reads one frame per RXNE and calls RxCplt once") { SPI_IRQ_MATRIX(spiIrqSlaveRx) }
TEST_CASE_FIXTURE(SpiFixture, "SQ2 a slave TransmitReceive_IT exchanges the frames and calls TxRxCplt once") { SPI_IRQ_MATRIX(spiIrqSlaveTxRx) }

namespace {
unsigned spiIrqSlaveIdx = 0;
unsigned spiIrqSlaveWord = 0;
unsigned spiIrqSlaveClocked = 0;
uint16_t spiIrqSlaveOut = 0;
/* Tick step: the master clocks one frame while the HAL waits for TXE inside the handler (it polls after reading the tick). */
void spiIrqClockOneFrame() {
	if (spiIrqSlaveClocked == 0) {
		spiIrqSlaveOut = hostSpiSlaveFrame(spiIrqSlaveIdx, (uint16_t)spiIrqSlaveWord);
		spiIrqSlaveClocked = 1;
	}
}
void spiIrqSlaveTx1(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, false);
	uint8_t b[1] = {0x6C};
	spiIrqSlaveIdx = idx;
	spiIrqSlaveWord = 0x99;
	spiIrqSlaveClocked = 0;
	spiIrqSlaveOut = 0;
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b, 1) == HAL_OK);
	hostTickSetStep(spiIrqClockOneFrame);
	HAL_SPI_IRQHandler(&r.h); /* TXE: loads the frame and waits for the master to clock it out */
	hostTickSetStep(nullptr);
	CHECK(spiIrqSlaveOut == 0x6C);
	CHECK(r.h.State == HAL_SPI_STATE_READY);
	CHECK(spiCounts.txCplt == 1);
	CHECK(spiIrqCompletions() == 1);
	CHECK(spiCounts.error == 0);
}
void spiIrqIdleHandler(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	const uint32_t cr1 = HalFixture::peek(&r.inst()->CR1), cr2 = spiIrqCr2(r);
	HAL_SPI_IRQHandler(&r.h);
	CHECK(spiCounts.txCplt + spiCounts.rxCplt + spiCounts.txRxCplt + spiCounts.error == 0);
	CHECK(r.h.State == HAL_SPI_STATE_READY);
	CHECK(hostSpiTxCount(idx) == 0);
	CHECK(HalFixture::peek(&r.inst()->CR1) == cr1);
	CHECK(spiIrqCr2(r) == cr2);
}
} // namespace
TEST_CASE_FIXTURE(SpiFixture, "SQ2 a slave Transmit_IT sends its frame and calls TxCplt once") { SPI_IRQ_INSTANCES(spiIrqSlaveTx1) }

/* ======================================================================== SQ3: errors */

namespace {
/* Slave Receive_IT with two frames in a row without a read in between: the second frame sets OVR (RM0090 28.3.10). */
void spiIrqOvrSlaveSetup(SpiIrqRig &r, unsigned idx, uint8_t *buf) {
	spiIrqInit(r, idx, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, false);
	REQUIRE(HAL_SPI_Receive_IT(&r.h, buf, 4) == HAL_OK);
	hostSpiSlaveFrame(idx, 0x11);
	hostSpiSlaveFrame(idx, 0x22);
	REQUIRE((spiIrqSr(r) & SPI_SR_OVR) != 0);
}
void spiIrqOvrRxCode(unsigned idx) {
	SpiIrqRig r;
	uint8_t b[4] = {};
	spiIrqOvrSlaveSetup(r, idx, b);
	HAL_SPI_IRQHandler(&r.h);
	CHECK((HAL_SPI_GetError(&r.h) & HAL_SPI_ERROR_OVR) != 0);
	CHECK((spiCounts.errorCodeAtError & HAL_SPI_ERROR_OVR) != 0);
}
void spiIrqOvrRxCallback(unsigned idx) {
	SpiIrqRig r;
	uint8_t b[4] = {};
	spiIrqOvrSlaveSetup(r, idx, b);
	HAL_SPI_IRQHandler(&r.h);
	CHECK(spiCounts.error == 1);
	CHECK(spiCounts.last == &r.h);
	HAL_SPI_IRQHandler(&r.h);
	CHECK(spiCounts.error == 1);
}
void spiIrqOvrRxFlag(unsigned idx) {
	SpiIrqRig r;
	uint8_t b[4] = {};
	spiIrqOvrSlaveSetup(r, idx, b);
	HAL_SPI_IRQHandler(&r.h);
	CHECK((spiIrqSr(r) & SPI_SR_OVR) == 0); /* cleared by the sequence read DR, read SR */
}
void spiIrqOvrRxState(unsigned idx) {
	SpiIrqRig r;
	uint8_t b[4] = {};
	spiIrqOvrSlaveSetup(r, idx, b);
	HAL_SPI_IRQHandler(&r.h);
	CHECK(r.h.State == HAL_SPI_STATE_READY);
	CHECK(r.h.Lock == HAL_UNLOCKED);
}
void spiIrqOvrRxNoCplt(unsigned idx) {
	SpiIrqRig r;
	uint8_t b[4] = {};
	spiIrqOvrSlaveSetup(r, idx, b);
	for (int i = 0; i < 3; ++i) HAL_SPI_IRQHandler(&r.h);
	CHECK(spiIrqCompletions() == 0);
}
/* Master Transmit_IT, TXEIE cleared by the test so that the handler reaches its error treatment with OVR pending. */
void spiIrqOvrMasterTx(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t b[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b, 8) == HAL_OK);
	r.inst()->CR2 &= ~SPI_CR2_TXEIE;
	/* Two frames end without a read of DR: the second received word finds RXNE set. */
	r.inst()->DR = 0x01;
	hostTickSet(hostTickPeek() + 1U);
	r.inst()->DR = 0x02;
	hostTickSet(hostTickPeek() + 1U);
	const uint32_t sr = r.inst()->SR; /* a bus read evaluates the frames that ended */
	REQUIRE((sr & SPI_SR_OVR) != 0);
	HAL_SPI_IRQHandler(&r.h);
	CHECK(spiCounts.error == 0);
	CHECK(HAL_SPI_GetError(&r.h) == HAL_SPI_ERROR_NONE);
	CHECK((spiIrqSr(r) & SPI_SR_OVR) == 0);
	CHECK(r.h.State == HAL_SPI_STATE_BUSY_TX);
}
/* The handler serves TXE before the error flags: call it until the error callback ran (a few calls at most). */
void spiIrqUntilError(SPI_HandleTypeDef *h) {
	for (int i = 0; i < 4 && spiCounts.error == 0; ++i) HAL_SPI_IRQHandler(h);
}
void spiIrqModfCode(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {1, 2, 3, 4};
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b, 4) == HAL_OK);
	hostSpiRaiseModf(idx);
	spiIrqUntilError(&r.h);
	CHECK((HAL_SPI_GetError(&r.h) & HAL_SPI_ERROR_MODF) != 0);
	CHECK((spiCounts.errorCodeAtError & HAL_SPI_ERROR_MODF) != 0);
	CHECK(spiCounts.error == 1);
	CHECK(spiCounts.last == &r.h);
}
void spiIrqModfFlag(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {1, 2, 3, 4};
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b, 4) == HAL_OK);
	hostSpiRaiseModf(idx);
	REQUIRE((spiIrqSr(r) & SPI_SR_MODF) != 0);
	spiIrqUntilError(&r.h);
	CHECK((spiIrqSr(r) & SPI_SR_MODF) == 0); /* read SR, then write CR1 (RM0090 28.3.10) */
}
void spiIrqModfState(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {1, 2, 3, 4};
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b, 4) == HAL_OK);
	hostSpiRaiseModf(idx);
	spiIrqUntilError(&r.h);
	CHECK(r.h.State == HAL_SPI_STATE_READY);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	CHECK(spiCounts.txCplt == 0);
}
void spiIrqModfRxTx(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t t[4] = {1, 2, 3, 4}, c[4] = {};
	REQUIRE(HAL_SPI_TransmitReceive_IT(&r.h, t, c, 4) == HAL_OK);
	hostSpiRaiseModf(idx);
	spiIrqUntilError(&r.h);
	CHECK((HAL_SPI_GetError(&r.h) & HAL_SPI_ERROR_MODF) != 0);
	CHECK(spiCounts.error == 1);
	CHECK(spiCounts.txRxCplt == 0);
}
void spiIrqFre(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, false, SPI_TIMODE_ENABLE);
	uint8_t b[4] = {};
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b, 4) == HAL_OK);
	hostSpiRaiseFre(idx);
	HAL_SPI_IRQHandler(&r.h);
	CHECK((HAL_SPI_GetError(&r.h) & HAL_SPI_ERROR_FRE) != 0);
	CHECK((spiCounts.errorCodeAtError & HAL_SPI_ERROR_FRE) != 0);
	CHECK(spiCounts.error == 1);
	CHECK((spiIrqSr(r) & SPI_SR_FRE) == 0); /* cleared by reading SR (RM0090 28.3.10) */
}
void spiIrqFreState(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, false, SPI_TIMODE_ENABLE);
	uint8_t b[4] = {};
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b, 4) == HAL_OK);
	hostSpiRaiseFre(idx);
	HAL_SPI_IRQHandler(&r.h);
	CHECK(r.h.State == HAL_SPI_STATE_READY);
}
void spiIrqErrorCodeKept(unsigned idx) {
	SpiIrqRig r;
	uint8_t b[4] = {};
	spiIrqOvrSlaveSetup(r, idx, b);
	HAL_SPI_IRQHandler(&r.h);
	HAL_SPI_IRQHandler(&r.h);
	CHECK((HAL_SPI_GetError(&r.h) & HAL_SPI_ERROR_OVR) != 0);
}
} // namespace

TEST_CASE_FIXTURE(SpiFixture, "SQ3 an OVR with ERRIE in Receive_IT sets the OVR error code before the error callback") {
	SPI_IRQ_INSTANCES(spiIrqOvrRxCode)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ3 an OVR with ERRIE in Receive_IT calls the error callback exactly once") {
	SPI_IRQ_INSTANCES(spiIrqOvrRxCallback)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ3 the handler clears OVR after an OVR error") { SPI_IRQ_INSTANCES(spiIrqOvrRxFlag) }
// NEEDS REVIEW SQ-3.
TEST_CASE_FIXTURE(SpiFixture, "SQ3 characterization: after an OVR error the state is ready and the handle unlocked") {
	SPI_IRQ_INSTANCES(spiIrqOvrRxState)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ3 an OVR error does not run a completion callback") { SPI_IRQ_INSTANCES(spiIrqOvrRxNoCplt) }
// NEEDS REVIEW SQ-7 (proposed text: characterization: in BUSY_TX state the handler clears OVR without an error code or callback; UM1725 and RM0090 28.3.10 do not say how a transmit only master treats the OVR that its unread receive buffer causes).
TEST_CASE_FIXTURE(SpiFixture, "SQ3 characterization: an OVR during a master Transmit_IT is cleared without error code and callback") {
	SPI_IRQ_INSTANCES(spiIrqOvrMasterTx)
}
// NEEDS REVIEW SQ-4.
TEST_CASE_FIXTURE(SpiFixture, "SQ3 characterization: the OVR error code stays set after the error callback") {
	SPI_IRQ_INSTANCES(spiIrqErrorCodeKept)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ3 a MODF with ERRIE sets the MODF error code and calls the error callback once") {
	SPI_IRQ_INSTANCES(spiIrqModfCode)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ3 the handler clears MODF after a MODF error") { SPI_IRQ_INSTANCES(spiIrqModfFlag) }
// NEEDS REVIEW SQ-3.
TEST_CASE_FIXTURE(SpiFixture, "SQ3 characterization: after a MODF error the state is ready, the handle unlocked and TxCplt not called") {
	SPI_IRQ_INSTANCES(spiIrqModfState)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ3 a MODF in TransmitReceive_IT sets the MODF error code and calls the error callback once") {
	SPI_IRQ_INSTANCES(spiIrqModfRxTx)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ3 a TI mode FRE with ERRIE sets the FRE error code, clears FRE and calls the error callback once") {
	SPI_IRQ_INSTANCES(spiIrqFre)
}
// NEEDS REVIEW SQ-3.
TEST_CASE_FIXTURE(SpiFixture, "SQ3 characterization: after a FRE error the state is ready") { SPI_IRQ_INSTANCES(spiIrqFreState) }

/* ======================================================================== SQ4: disabled interrupts */

namespace {
void spiIrqNoTxeie(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {1, 2, 3, 4};
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b, 4) == HAL_OK);
	r.inst()->CR2 &= ~SPI_CR2_TXEIE;
	REQUIRE((spiIrqSr(r) & SPI_SR_TXE) != 0);
	for (int i = 0; i < 3; ++i) HAL_SPI_IRQHandler(&r.h);
	CHECK(hostSpiTxCount(idx) == 0);
	CHECK(spiIrqCompletions() == 0);
	CHECK(r.h.State == HAL_SPI_STATE_BUSY_TX);
}
void spiIrqNoRxneie(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {};
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b, 4) == HAL_OK);
	r.inst()->CR2 &= ~SPI_CR2_RXNEIE;
	hostSpiSlaveFrame(idx, 0x5A);
	REQUIRE((spiIrqSr(r) & SPI_SR_RXNE) != 0);
	for (int i = 0; i < 3; ++i) HAL_SPI_IRQHandler(&r.h);
	CHECK((spiIrqSr(r) & SPI_SR_RXNE) != 0); /* the frame was not read */
	CHECK(b[0] == 0);
	CHECK(spiIrqCompletions() == 0);
	CHECK(r.h.State == HAL_SPI_STATE_BUSY_RX);
}
void spiIrqNoErrie(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {};
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b, 4) == HAL_OK);
	r.inst()->CR2 &= ~(SPI_CR2_ERRIE | SPI_CR2_RXNEIE);
	hostSpiSlaveFrame(idx, 0x11);
	hostSpiSlaveFrame(idx, 0x22);
	REQUIRE((spiIrqSr(r) & SPI_SR_OVR) != 0);
	for (int i = 0; i < 3; ++i) HAL_SPI_IRQHandler(&r.h);
	CHECK(spiCounts.error == 0);
	CHECK(HAL_SPI_GetError(&r.h) == HAL_SPI_ERROR_NONE);
	CHECK((spiIrqSr(r) & SPI_SR_OVR) != 0);
}
void spiIrqNoErrieModf(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t b[4] = {1, 2, 3, 4};
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b, 4) == HAL_OK);
	r.inst()->CR2 &= ~(SPI_CR2_ERRIE | SPI_CR2_TXEIE);
	hostSpiRaiseModf(idx);
	for (int i = 0; i < 3; ++i) HAL_SPI_IRQHandler(&r.h);
	CHECK(spiCounts.error == 0);
	CHECK(HAL_SPI_GetError(&r.h) == HAL_SPI_ERROR_NONE);
	CHECK((spiIrqSr(r) & SPI_SR_MODF) != 0);
}
} // namespace

TEST_CASE_FIXTURE(SpiFixture, "SQ4 a set TXE with TXEIE clear writes no frame") { SPI_IRQ_INSTANCES(spiIrqNoTxeie) }
TEST_CASE_FIXTURE(SpiFixture, "SQ4 a set RXNE with RXNEIE clear reads no frame") { SPI_IRQ_INSTANCES(spiIrqNoRxneie) }
TEST_CASE_FIXTURE(SpiFixture, "SQ4 an OVR with ERRIE clear sets no error code and calls no callback") { SPI_IRQ_INSTANCES(spiIrqNoErrie) }
TEST_CASE_FIXTURE(SpiFixture, "SQ4 a MODF with ERRIE clear sets no error code and calls no callback") { SPI_IRQ_INSTANCES(spiIrqNoErrieModf) }
TEST_CASE_FIXTURE(SpiFixture, "SQ4 the handler of an idle initialised peripheral does nothing") { SPI_IRQ_INSTANCES(spiIrqIdleHandler) }

/* ======================================================================== SQ5: receive only master */

TEST_CASE_FIXTURE(SpiFixture, "SQ5 Receive_IT of a receive only master delivers the peer words and calls RxCplt once") {
	SPI_IRQ_MATRIX(spiIrqRxOnlyData)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ5 Receive_IT of a receive only master ends without an error and leaves no OVR") {
	SPI_IRQ_MATRIX(spiIrqRxOnlyClean)
}

namespace {
void spiIrqNoConfigWhileEnabled(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES_RXONLY, false);
	hostSpiSetFrameTicks(kSpiIrqRxOnlyFrameTicks); /* see the comment at the constant */
	spiIrqPeer(r, false, 3, 0);
	uint8_t b[3] = {};
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b, 3) == HAL_OK);
	spiIrqPump(&r.h, 40);
	CHECK(hostSpiConfigWhileEnabled(idx) == 0);
}
void spiIrqRxOnlyNeverEnableTx(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES_RXONLY, false);
	spiIrqPeer(r, false, 3, 0);
	uint8_t b[3] = {};
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b, 3) == HAL_OK);
	CHECK(r.h.State == HAL_SPI_STATE_BUSY_RX);
	CHECK((spiIrqCr2(r) & SPI_CR2_RXNEIE) != 0);
	CHECK((spiIrqCr2(r) & SPI_CR2_TXEIE) == 0);
	CHECK((HalFixture::peek(&r.inst()->CR1) & SPI_CR1_SPE) != 0);
}
} // namespace
TEST_CASE_FIXTURE(SpiFixture, "SQ5 a receive only IT transfer changes no configuration bit while SPE is set") {
	SPI_IRQ_INSTANCES(spiIrqNoConfigWhileEnabled)
}
TEST_CASE_FIXTURE(SpiFixture, "SQ5 Receive_IT of a receive only master enables RXNEIE and SPE, not TXEIE") {
	SPI_IRQ_INSTANCES(spiIrqRxOnlyNeverEnableTx)
}

/* ======================================================================== SQ2 additions: BSY, OVR and ERRIE after completion */

namespace {
uint32_t spiIrqSrLive(SpiIrqRig &r) { return r.inst()->SR; } /* a bus read evaluates the frames that ended */

/* Default frame time of one tick: the HAL's wait for TXE in Close_ISR is a counted loop without a tick read, so a longer frame would
 * time out in the model (a model limit, the target loop runs in real time). */
void spiIrqTxEnd(unsigned idx, bool wide, unsigned n) {
	if (n < 2) return; /* the end of a single frame is covered by SQ2 completion cases */
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	SpiIrqBuf b(wide);
	for (unsigned k = 0; k < n; ++k) b.set(k, spiIrqWord(wide, k));
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 40);
	REQUIRE(r.h.State == HAL_SPI_STATE_READY);
	REQUIRE(spiCounts.txCplt == 1);
	const uint32_t sr = spiIrqSrLive(r);
	CHECK((sr & SPI_SR_BSY) == 0); /* RM0090 28.3.8: the last frame is finished when the call completes */
}
void spiIrqTxEndOvr(unsigned idx, bool wide, unsigned n) {
	if (n < 2) return;
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	SpiIrqBuf b(wide);
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 40);
	REQUIRE(r.h.State == HAL_SPI_STATE_READY);
	const uint32_t sr = spiIrqSrLive(r);
	CHECK((sr & SPI_SR_OVR) == 0); /* the unread received words were cleared (read DR, read SR, 28.3.10) */
}
void spiIrqTxErrie(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	SpiIrqBuf b(wide);
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	REQUIRE(r.h.State == HAL_SPI_STATE_READY);
	CHECK((spiIrqCr2(r) & SPI_CR2_ERRIE) == 0);
}
void spiIrqRxErrie(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	spiIrqPeer(r, wide, n, 0);
	SpiIrqBuf b(wide);
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	REQUIRE(r.h.State == HAL_SPI_STATE_READY);
	CHECK((spiIrqCr2(r) & SPI_CR2_ERRIE) == 0);
}
void spiIrqTxRxErrie(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, wide);
	SpiIrqBuf tx(wide), rx(wide);
	REQUIRE(HAL_SPI_TransmitReceive_IT(&r.h, tx.raw, rx.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	REQUIRE(r.h.State == HAL_SPI_STATE_READY);
	CHECK((spiIrqCr2(r) & SPI_CR2_ERRIE) == 0);
}
} // namespace

TEST_CASE_FIXTURE(SpiFixture, "SQ2 after Transmit_IT of several frames BSY is clear") { SPI_IRQ_MATRIX(spiIrqTxEnd) }
TEST_CASE_FIXTURE(SpiFixture, "SQ2 after Transmit_IT of several frames OVR is clear") { SPI_IRQ_MATRIX(spiIrqTxEndOvr) }
// NEEDS REVIEW SQ-8.
TEST_CASE_FIXTURE(SpiFixture, "SQ2 characterization: Transmit_IT clears ERRIE after the last frame") { SPI_IRQ_MATRIX(spiIrqTxErrie) }
// NEEDS REVIEW SQ-8.
TEST_CASE_FIXTURE(SpiFixture, "SQ2 characterization: Receive_IT clears ERRIE after the last frame") { SPI_IRQ_MATRIX(spiIrqRxErrie) }
// NEEDS REVIEW SQ-8.
TEST_CASE_FIXTURE(SpiFixture, "SQ2 characterization: TransmitReceive_IT clears ERRIE after the last frame") { SPI_IRQ_MATRIX(spiIrqTxRxErrie) }

/* ======================================================================== SQ1 / SQ2: one line mode in IT */

namespace {
/* CR1 value written by the first write that sets SPE (HAL access log). */
uint32_t spiIrqFirstSpeWrite(SpiIrqRig &r) {
	const uintptr_t a = reinterpret_cast<uintptr_t>(&r.inst()->CR1);
	for (size_t i = 0; i < hostRegsLogCount(); ++i) {
		const HostRegAccess *e = hostRegsLogAt(i);
		if (e->write && e->addr == a && (e->written & SPI_CR1_SPE) != 0) return e->written;
	}
	FAIL_CHECK("no CR1 write set SPE");
	return 0;
}

void spiIrqOneLineTxBidioe(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_1LINE, false);
	uint8_t b[2] = {1, 2};
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b, 2) == HAL_OK);
	const uint32_t cr1 = spiIrqFirstSpeWrite(r);
	CHECK((cr1 & SPI_CR1_BIDIMODE) != 0);
	CHECK((cr1 & SPI_CR1_BIDIOE) != 0);
}
void spiIrqOneLineRxBidioe(unsigned idx) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_1LINE, false);
	uint8_t b[2] = {};
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b, 2) == HAL_OK);
	const uint32_t cr1 = spiIrqFirstSpeWrite(r);
	CHECK((cr1 & SPI_CR1_BIDIMODE) != 0);
	CHECK((cr1 & SPI_CR1_BIDIOE) == 0);
}
void spiIrqOneLineTxData(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_1LINE, wide);
	SpiIrqBuf b(wide);
	for (unsigned k = 0; k < n; ++k) b.set(k, spiIrqWord(wide, k));
	REQUIRE(HAL_SPI_Transmit_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 20);
	CHECK(spiCounts.txCplt == 1);
	REQUIRE(hostSpiTxCount(idx) == n);
	for (unsigned k = 0; k < n; ++k) {
		CAPTURE(k);
		CHECK(hostSpiTxAt(idx, k) == spiIrqWord(wide, k));
	}
}
void spiIrqOneLineRxData(unsigned idx, bool wide, unsigned n) {
	SpiIrqRig r;
	spiIrqInit(r, idx, SPI_MODE_MASTER, SPI_DIRECTION_1LINE, wide);
	hostSpiSetFrameTicks(kSpiIrqRxOnlyFrameTicks);
	spiIrqPeer(r, wide, n, 1);
	SpiIrqBuf b(wide);
	REQUIRE(HAL_SPI_Receive_IT(&r.h, b.raw, (uint16_t)n) == HAL_OK);
	spiIrqPump(&r.h, 10 * n + 40);
	CHECK(spiCounts.rxCplt == 1);
	CHECK(spiCounts.error == 0);
	for (unsigned k = 0; k < n; ++k) {
		CAPTURE(k);
		CHECK(b.get(k) == spiIrqWord(wide, k, 1));
	}
}
} // namespace

TEST_CASE_FIXTURE(SpiFixture, "SQ1 Transmit_IT of a 1 line master sets BIDIOE when SPE is set") { SPI_IRQ_INSTANCES(spiIrqOneLineTxBidioe) }
TEST_CASE_FIXTURE(SpiFixture, "SQ1 Receive_IT of a 1 line master leaves BIDIOE clear when SPE is set") { SPI_IRQ_INSTANCES(spiIrqOneLineRxBidioe) }
TEST_CASE_FIXTURE(SpiFixture, "SQ2 Transmit_IT of a 1 line master puts the buffer on the bus and calls TxCplt once") { SPI_IRQ_MATRIX(spiIrqOneLineTxData) }
TEST_CASE_FIXTURE(SpiFixture, "SQ2 Receive_IT of a 1 line master delivers the peer words and calls RxCplt once") { SPI_IRQ_MATRIX(spiIrqOneLineRxData) }

/* ======================================================================== SQ6: abort of an IT transfer */
/* HAL_SPI_Abort / Abort_IT wait for the SPI interrupt inside the call (see SQ-9): in this host the wait times out. */

namespace {
enum SpiIrqKind { kTx, kRx, kRxOnly, kTxRx, kSlaveRx, kSlaveTxRx };
enum SpiIrqMode { mBlocking, mIt, mItLate };
enum SpiIrqProp { pTimeout, pOvr, pNoCplt, pAbortCb, pLateCr2, pLateSpe };

struct SpiIrqSnap {
	HAL_StatusTypeDef ret;
	HAL_SPI_StateTypeDef state;
	uint32_t cr1, cr2, sr, errorCode;
	SpiCallbackCounts counts;
};

/* Starts a transfer of 8 frames of the kind, lets it run three handler calls, aborts; mItLate also lets the handler run afterwards. */
SpiIrqSnap spiIrqAbortRun(unsigned idx, unsigned kind, unsigned mode) {
	SpiIrqRig r;
	const bool slave = kind == kSlaveRx || kind == kSlaveTxRx;
	spiIrqInit(r, idx, slave ? SPI_MODE_SLAVE : SPI_MODE_MASTER, kind == kRxOnly ? SPI_DIRECTION_2LINES_RXONLY : SPI_DIRECTION_2LINES, false);
	if (kind == kRxOnly) hostSpiSetFrameTicks(kSpiIrqRxOnlyFrameTicks);
	spiIrqPeer(r, false, 16, 0);
	uint8_t tx[8] = {1, 2, 3, 4, 5, 6, 7, 8}, rx[8] = {};
	HAL_StatusTypeDef start = HAL_ERROR;
	switch (kind) {
	case kTx: start = HAL_SPI_Transmit_IT(&r.h, tx, 8); break;
	case kRx:
	case kRxOnly:
	case kSlaveRx: start = HAL_SPI_Receive_IT(&r.h, rx, 8); break;
	default: start = HAL_SPI_TransmitReceive_IT(&r.h, tx, rx, 8); break;
	}
	REQUIRE(start == HAL_OK);
	for (int i = 0; i < 3; ++i) {
		if (slave && (spiIrqSr(r) & SPI_SR_RXNE) == 0) hostSpiSlaveFrame(idx, 0x40);
		hostTickSet(hostTickPeek() + 1U);
		HAL_SPI_IRQHandler(&r.h);
	}
	REQUIRE(r.h.State != HAL_SPI_STATE_READY); /* still transferring */
	SpiIrqSnap s;
	s.ret = mode == mBlocking ? HAL_SPI_Abort(&r.h) : HAL_SPI_Abort_IT(&r.h);
	if (mode == mItLate) {
		for (int i = 0; i < 4; ++i) {
			if (slave && (spiIrqSr(r) & SPI_SR_RXNE) == 0) hostSpiSlaveFrame(idx, 0x41);
			hostTickSet(hostTickPeek() + 1U);
			HAL_SPI_IRQHandler(&r.h);
		}
	}
	s.state = r.h.State;
	s.cr1 = HalFixture::peek(&r.inst()->CR1);
	s.cr2 = spiIrqCr2(r);
	s.sr = HalFixture::peek(&r.inst()->SR);
	s.errorCode = r.h.ErrorCode;
	s.counts = spiCounts;
	return s;
}

void spiIrqAbortCheck(unsigned idx, unsigned kind, unsigned mode, unsigned prop) {
	CAPTURE(kind);
	const SpiIrqSnap s = spiIrqAbortRun(idx, kind, mode);
	switch (prop) {
	case pTimeout:
		CHECK(s.ret == HAL_ERROR);
		CHECK((s.errorCode & HAL_SPI_ERROR_ABORT) != 0);
		CHECK(s.state == HAL_SPI_STATE_READY);
		break;
	case pOvr: CHECK((s.sr & SPI_SR_OVR) == 0); break;
	case pNoCplt:
		CHECK(s.counts.txCplt + s.counts.rxCplt + s.counts.txRxCplt == 0);
		CHECK(s.counts.error == 0);
		break;
	case pAbortCb: CHECK(s.counts.abortCplt == 1); break;
	case pLateCr2: CHECK((s.cr2 & (SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE)) == 0); break;
	default: CHECK((s.cr1 & SPI_CR1_SPE) == 0); break;
	}
}

#define SPI_IRQ_ABORT_FN(name, mode, prop) \
	void name(unsigned idx, unsigned kind) { spiIrqAbortCheck(idx, kind, mode, prop); }
SPI_IRQ_ABORT_FN(spiIrqAbortTimeout, mBlocking, pTimeout)
SPI_IRQ_ABORT_FN(spiIrqAbortOvr, mBlocking, pOvr)
SPI_IRQ_ABORT_FN(spiIrqAbortNoCplt, mBlocking, pNoCplt)
SPI_IRQ_ABORT_FN(spiIrqAbortItTimeout, mIt, pTimeout)
SPI_IRQ_ABORT_FN(spiIrqAbortItOvr, mIt, pOvr)
SPI_IRQ_ABORT_FN(spiIrqAbortItNoCplt, mIt, pNoCplt)
SPI_IRQ_ABORT_FN(spiIrqAbortItCb, mIt, pAbortCb)
SPI_IRQ_ABORT_FN(spiIrqAbortLateCr2, mItLate, pLateCr2)
SPI_IRQ_ABORT_FN(spiIrqAbortLateSpe, mItLate, pLateSpe)

#define SPI_IRQ_KINDS(fn, i)                                                                                                      \
	SUBCASE("Transmit_IT") { fn(i, kTx); }                                                                                        \
	SUBCASE("Receive_IT 2 lines") { fn(i, kRx); }                                                                                 \
	SUBCASE("Receive_IT receive only") { fn(i, kRxOnly); }                                                                        \
	SUBCASE("TransmitReceive_IT") { fn(i, kTxRx); }                                                                               \
	SUBCASE("slave Receive_IT") { fn(i, kSlaveRx); }                                                                              \
	SUBCASE("slave TransmitReceive_IT") { fn(i, kSlaveTxRx); }
#define SPI_IRQ_ABORT_MATRIX(fn)                                                                                                  \
	SUBCASE("SPI1") { SPI_IRQ_KINDS(fn, 0) }                                                                                      \
	SUBCASE("SPI2") { SPI_IRQ_KINDS(fn, 1) }                                                                                      \
	SUBCASE("SPI3") { SPI_IRQ_KINDS(fn, 2) }
} // namespace

// NEEDS REVIEW SQ-9.
TEST_CASE_FIXTURE(SpiFixture, "SQ6 characterization: Abort of an IT transfer without a delivered interrupt returns HAL_ERROR with the ABORT error code and the state ready") { SPI_IRQ_ABORT_MATRIX(spiIrqAbortTimeout) }
// NEEDS REVIEW SQ-9.
TEST_CASE_FIXTURE(SpiFixture, "SQ6 characterization: Abort_IT of an IT transfer without a delivered interrupt returns HAL_ERROR with the ABORT error code and the state ready") { SPI_IRQ_ABORT_MATRIX(spiIrqAbortItTimeout) }
TEST_CASE_FIXTURE(SpiFixture, "SQ6 Abort of an IT transfer leaves OVR clear") { SPI_IRQ_ABORT_MATRIX(spiIrqAbortOvr) }
TEST_CASE_FIXTURE(SpiFixture, "SQ6 Abort_IT of an IT transfer leaves OVR clear") { SPI_IRQ_ABORT_MATRIX(spiIrqAbortItOvr) }
TEST_CASE_FIXTURE(SpiFixture, "SQ6 Abort of an IT transfer runs no completion or error callback") { SPI_IRQ_ABORT_MATRIX(spiIrqAbortNoCplt) }
TEST_CASE_FIXTURE(SpiFixture, "SQ6 Abort_IT of an IT transfer runs no completion or error callback") { SPI_IRQ_ABORT_MATRIX(spiIrqAbortItNoCplt) }
// NEEDS REVIEW SQ-9.
TEST_CASE_FIXTURE(SpiFixture, "SQ6 characterization: Abort_IT of an IT transfer calls AbortCpltCallback exactly once even when the abort ISR did not run") { SPI_IRQ_ABORT_MATRIX(spiIrqAbortItCb) }
// NEEDS REVIEW SQ-10.
TEST_CASE_FIXTURE(SpiFixture, "SQ6 characterization: the abort ISR run after a timed out Abort_IT clears TXEIE, RXNEIE and ERRIE") { SPI_IRQ_ABORT_MATRIX(spiIrqAbortLateCr2) }
// NEEDS REVIEW SQ-10.
TEST_CASE_FIXTURE(SpiFixture, "SQ6 characterization: the abort ISR run after a timed out Abort_IT clears SPE") { SPI_IRQ_ABORT_MATRIX(spiIrqAbortLateSpe) }
