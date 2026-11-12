/*
 * P12 T4b group UD: the DMA mode of the real ST HAL UART driver (stm32f4xx_hal_uart.c: HAL_UART_Transmit_DMA, Receive_DMA,
 * HAL_UARTEx_ReceiveToIdle_DMA, UART_Start_Receive_DMA, DMAPause, DMAResume, DMAStop, the static UART_DMA* callbacks reached through
 * the DMA handles, and the DMA branches of HAL_UART_IRQHandler) together with the real HAL DMA, against the UART model
 * (fakes/stm32_hal/HalUart.h) and the DMA model (fakes/stm32_hal/HalDma.h). Requirements UD1-UD6 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T4B_REQUIREMENTS.md; compiled into ffb_hal_tests (F407VG) and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 (UART chapter: DMA functions, Rx event notes, error code names) and RM0090 Rev 22:
 *  - Table 1: USART1 0x40011000, USART2 0x40004400, USART3 0x40004800, UART4 0x40004C00, UART5 0x40005000, USART6 0x40011400;
 *    30.6.2 USART_DR at offset 0x04.
 *  - Tables 43 / 44: USART1 DMA2 Tx stream 7 channel 4, Rx stream 2 or 5 channel 4; USART2 DMA1 Tx 6 / Rx 5 ch 4; USART3 DMA1 Tx 3 /
 *    Rx 1 ch 4; UART4 DMA1 Tx 4 / Rx 2 ch 4; UART5 DMA1 Tx 7 / Rx 0 ch 4; USART6 DMA2 Tx 6 or 7 / Rx 1 or 2 channel 5.
 *  - 10.5.5 DMA_SxCR: EN bit 0, TEIE 2, HTIE 3, TCIE 4, DIR bits 7:6 (00 peripheral to memory, 01 memory to peripheral),
 *    CHSEL bits 27:25. 10.5.6 NDTR counts data items.
 *  - 30.6.1 SR: PE 0, FE 1, NE 2, ORE 3, IDLE 4, RXNE 5, TC 6, TXE 7. 30.6.4 CR1: IDLEIE 4, RXNEIE 5, TCIE 6, TXEIE 7, PEIE 8.
 *    30.6.6 CR3: EIE 0, DMAR 6, DMAT 7. 30.3.13: a DMA request per TXE / RXNE when DMAT / DMAR is set; TC is cleared by software
 *    (write 0) before the DMA transfer so that the end of the DMA is the next TC.
 *  - 30.3.7: with PCE the MSB of the frame is the parity bit (even: the number of ones in the frame is even).
 * The streams are configured by the test with the real HAL_DMA_Init (the UART HAL does not configure them), and the tests stand in for
 * the DMA engine: they write / read USART_DR frame by frame and call hostDmaStepItems / hostDmaComplete / hostDmaRaise, then
 * HAL_DMA_IRQHandler. Time is the HAL tick: every HAL_GetTick() call advances it by 1 ms.
 * Every file-local symbol is prefixed uartDma or sits in the anonymous namespace.
 *
 * NEEDS REVIEW proposals (numbers assigned by the lead):
 *  NEEDS REVIEW 317 UD1: a Transmit_DMA whose DMA stream is not READY (HAL_DMA_Start_IT returns HAL_BUSY) still returns HAL_OK, sets BUSY_TX and DMAT.
 *  NEEDS REVIEW 317 UD1: a call that starts a transfer clears ErrorCode to NONE (UM1725 silent).
 *  NEEDS REVIEW 317 UD3: a stream error of the Tx stream also ends a running Rx DMA reception (shared UART_DMAError), state READY, one ErrorCallback.
 *  NEEDS REVIEW 317 UD3: an Rx error with a stream that is no longer running (HAL_DMA_Abort_IT fails) calls ErrorCallback at once; hdmarx NULL: the
 *     same, at once (UM1725 silent).
 *  NEEDS REVIEW 317 UD4: DMAStop with a NULL hdmatx / hdmarx skips the stream abort and still ends the transfer.
 *  NEEDS REVIEW 317 UD5: an IDLE event while no data item has been moved by the DMA (NDTR = Size) gives no RxEvent callback and leaves the reception running.
 */
#include "HalDma.h"
#include "HalFixture.h"
#include "HalHostSystem.h"
#include "HalUart.h"
#include "TestHelpers.h"
#include "UartTestSupport.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr uint32_t uartDmaStreamEN = 1u << 0;
constexpr uint32_t uartDmaCHSEL_MASK = 7u << 25;

struct UartDmaCfg {
	const char *name;
	unsigned idx;      // 0 USART1, 1 USART2, 2 USART3, 3 UART4, 4 UART5, 5 USART6
	unsigned c;        // 0 DMA1, 1 DMA2
	unsigned txS, rxS;
	uint32_t channel;  // 0..7
	uint32_t dr;       // RM0090 Table 1 + 0x04
};

/* RM0090 Tables 1, 43, 44. */
/* Index 1 is USART2: tests that hard code USART2 (instance 1, DMA1 Tx stream 6, Rx stream 5) use uartDmaCfgs[1]. */
const UartDmaCfg uartDmaCfgs[] = {
    {"USART1 Rx stream 2", 0, 1, 7, 2, 4, 0x40011004u},
    {"USART2", 1, 0, 6, 5, 4, 0x40004404u},
    {"USART3", 2, 0, 3, 1, 4, 0x40004804u},
    {"UART4", 3, 0, 4, 2, 4, 0x40004C04u},
    {"UART5", 4, 0, 7, 0, 4, 0x40005004u},
    {"USART6 Tx 6 Rx 1", 5, 1, 6, 1, 5, 0x40011404u},
    {"USART6 Tx 7 Rx 2", 5, 1, 7, 2, 5, 0x40011404u},
    {"USART1 Rx stream 5", 0, 1, 7, 5, 4, 0x40011004u},
};

USART_TypeDef *uartDmaInstance(unsigned i) {
	USART_TypeDef *const a[6] = {USART1, USART2, USART3, UART4, UART5, USART6};
	return a[i];
}

DMA_Stream_TypeDef *uartDmaStream(unsigned c, unsigned s) {
	DMA_Stream_TypeDef *const d1[8] = {DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7};
	DMA_Stream_TypeDef *const d2[8] = {DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7};
	return c == 0 ? d1[s] : d2[s];
}

uint32_t uartDmaChannelConst(uint32_t ch) {
	const uint32_t a[8] = {DMA_CHANNEL_0, DMA_CHANNEL_1, DMA_CHANNEL_2, DMA_CHANNEL_3, DMA_CHANNEL_4, DMA_CHANNEL_5, DMA_CHANNEL_6, DMA_CHANNEL_7};
	return a[ch];
}

struct UartDmaOpt {
	bool nine = false;                       // 9 bit words (no parity)
	uint32_t parity = UART_PARITY_NONE;      // 8 bit words
	bool circRx = false;
	bool circTx = false;
};

const UartDmaOpt uartDmaOptN8 = UartDmaOpt();

/* RM0090 30.3.7: with PCE the MSB (bit 7 of an 8 bit frame) is the parity bit; even: the frame holds an even number of ones. */
uint16_t uartDmaLine7(uint16_t data7, uint32_t parity) {
	unsigned ones = 0;
	for (unsigned b = 0; b < 7; ++b) ones += (data7 >> b) & 1u;
	const unsigned odd = ones & 1u;
	const unsigned bit = (parity == UART_PARITY_EVEN) ? odd : (odd ^ 1u);
	return static_cast<uint16_t>((data7 & 0x7Fu) | (bit << 7));
}

struct UartDmaRig {
	UartDmaCfg cfg;
	UartDmaOpt opt;
	USART_TypeDef *inst;
	DMA_Stream_TypeDef *txI, *rxI;
	UART_HandleTypeDef h;
	DMA_HandleTypeDef tx, rx;
	uint16_t txBuf[16];
	uint16_t rxBuf[16];

	explicit UartDmaRig(const UartDmaCfg &c, const UartDmaOpt &o = uartDmaOptN8)
	    : cfg(c), opt(o), inst(uartDmaInstance(c.idx)), txI(uartDmaStream(c.c, c.txS)), rxI(uartDmaStream(c.c, c.rxS)) {
		std::memset(&h, 0, sizeof h);
		std::memset(txBuf, 0, sizeof txBuf);
		std::memset(rxBuf, 0, sizeof rxBuf);
		h.Instance = inst;
		h.Init.BaudRate = 115200;
		h.Init.WordLength = opt.nine ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
		h.Init.StopBits = UART_STOPBITS_1;
		h.Init.Parity = opt.parity;
		h.Init.Mode = UART_MODE_TX_RX;
		h.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		h.Init.OverSampling = UART_OVERSAMPLING_16;
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		makeDma(tx, txI, DMA_MEMORY_TO_PERIPH, opt.circTx);
		makeDma(rx, rxI, DMA_PERIPH_TO_MEMORY, opt.circRx);
		h.hdmatx = &tx;
		tx.Parent = &h;
		h.hdmarx = &rx;
		rx.Parent = &h;
		uartSupportReset();
	}

	void makeDma(DMA_HandleTypeDef &d, DMA_Stream_TypeDef *s, uint32_t dir, bool circular) {
		std::memset(&d, 0, sizeof d);
		d.Instance = s;
		d.Init.Channel = uartDmaChannelConst(cfg.channel);
		d.Init.Direction = dir;
		d.Init.PeriphInc = DMA_PINC_DISABLE;
		d.Init.MemInc = DMA_MINC_ENABLE;
		d.Init.PeriphDataAlignment = opt.nine ? DMA_PDATAALIGN_HALFWORD : DMA_PDATAALIGN_BYTE;
		d.Init.MemDataAlignment = opt.nine ? DMA_MDATAALIGN_HALFWORD : DMA_MDATAALIGN_BYTE;
		d.Init.Mode = circular ? DMA_CIRCULAR : DMA_NORMAL;
		d.Init.Priority = DMA_PRIORITY_LOW;
		d.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		d.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		d.Init.MemBurst = DMA_MBURST_SINGLE;
		d.Init.PeriphBurst = DMA_PBURST_SINGLE;
		REQUIRE(HAL_DMA_Init(&d) == HAL_OK);
	}

	/* Data item i that the application sends: 7 bit data (parity configurations: the model adds the parity), or 9 bits. */
	uint16_t data(unsigned i) const { return opt.nine ? static_cast<uint16_t>(0x1A0u + i) : static_cast<uint16_t>(0x30u + 5u * i); }
	/* The bits on the line (and what DR reads back) for item i: with parity the MSB is the parity bit. */
	uint16_t line(unsigned i) const { return opt.parity == UART_PARITY_NONE ? data(i) : uartDmaLine7(data(i), opt.parity); }

	void fillTx(unsigned count) {
		for (unsigned i = 0; i < count; ++i) {
			if (opt.nine)
				txBuf[i] = data(i);
			else
				reinterpret_cast<uint8_t *>(txBuf)[i] = static_cast<uint8_t>(data(i));
		}
	}
	uint16_t rxWord(unsigned i) const { return opt.nine ? rxBuf[i] : reinterpret_cast<const uint8_t *>(rxBuf)[i]; }
	uint8_t *txPtr() { return reinterpret_cast<uint8_t *>(txBuf); }
	uint8_t *rxPtr() { return reinterpret_cast<uint8_t *>(rxBuf); }
	uint32_t txAddr() const { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(txBuf)); }
	uint32_t rxAddr() const { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(rxBuf)); }

	uint32_t sr() const { return HalFixture::peek(&inst->SR); }
	uint32_t cr1() const { return HalFixture::peek(&inst->CR1); }
	uint32_t cr3() const { return HalFixture::peek(&inst->CR3); }
	bool streamEnabled(const DMA_Stream_TypeDef *s) const { return (HalFixture::peek(&s->CR) & uartDmaStreamEN) != 0u; }

	void waitSr(uint32_t flag) {
		for (unsigned g = 0; g < 1000; ++g) {
			if (inst->SR & flag) return;
			HAL_GetTick();
		}
		FAIL("UART flag never set");
	}
	/* The Tx DMA engine: one DR write per TXE request. */
	void engineTx(unsigned count, unsigned first = 0) {
		for (unsigned i = first; i < first + count; ++i) {
			waitSr(USART_SR_TXE);
			inst->DR = opt.nine ? txBuf[i] : reinterpret_cast<uint8_t *>(txBuf)[i];
		}
	}
	/* The Rx DMA engine: the peer sends frame i, the DMA reads DR at once (RXNE request) and stores the item. */
	void engineRx(unsigned count, unsigned first = 0) {
		for (unsigned i = first; i < first + count; ++i) {
			hostUartRxNow(cfg.idx, line(i), 0);
			const uint32_t r = inst->DR;
			if (opt.nine)
				rxBuf[i] = static_cast<uint16_t>(r);
			else
				reinterpret_cast<uint8_t *>(rxBuf)[i] = static_cast<uint8_t>(r);
		}
	}
	void completeTx() {
		hostDmaComplete(cfg.c, cfg.txS);
		HAL_DMA_IRQHandler(&tx);
	}
	void completeRx() {
		hostDmaComplete(cfg.c, cfg.rxS);
		HAL_DMA_IRQHandler(&rx);
	}
	void rxStep(unsigned items) { hostDmaStepItems(cfg.c, cfg.rxS, items); }
	void rxIrq() { HAL_DMA_IRQHandler(&rx); }
	void txIrq() { HAL_DMA_IRQHandler(&tx); }
	/* A full Tx DMA transfer up to the DMA transfer complete interrupt; the last frame may still run. */
	void txDmaDone(unsigned count) {
		fillTx(count);
		REQUIRE(HAL_UART_Transmit_DMA(&h, txPtr(), static_cast<uint16_t>(count)) == HAL_OK);
		engineTx(count);
		completeTx();
	}
};

class UartDmaFx : public UartFixture {};

std::vector<UartDmaOpt> uartDmaFrameOpts(std::vector<std::string> &names) {
	std::vector<UartDmaOpt> v;
	UartDmaOpt o;
	v.push_back(o);
	names.push_back("8 bit no parity");
	o = UartDmaOpt();
	o.parity = UART_PARITY_EVEN;
	v.push_back(o);
	names.push_back("8 bit even parity");
	o = UartDmaOpt();
	o.parity = UART_PARITY_ODD;
	v.push_back(o);
	names.push_back("8 bit odd parity");
	o = UartDmaOpt();
	o.nine = true;
	v.push_back(o);
	names.push_back("9 bit no parity");
	return v;
}

} // namespace

/* ------------------------------------------------------------------------------------------------------------------ UD1 */

TEST_CASE_FIXTURE(UartDmaFx, "UD1 Transmit_DMA programs the Tx stream with the data register, the buffer, direction memory to peripheral, channel and Size items") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			UartDmaRig r(k);
			r.fillTx(5);
			REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 5) == HAL_OK);
			CHECK(HalFixture::peek(&r.txI->PAR) == k.dr);
			CHECK(HalFixture::peek(&r.txI->M0AR) == r.txAddr());
			CHECK(HalFixture::peek(&r.txI->NDTR) == 5u);
			const uint32_t cr = HalFixture::peek(&r.txI->CR);
			CHECK((cr & uartDmaStreamEN) == uartDmaStreamEN);
			CHECK(((cr >> 6) & 3u) == 1u); // DIR 01 memory to peripheral (RM0090 10.5.5)
			CHECK(((cr & uartDmaCHSEL_MASK) >> 25) == k.channel);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD1 Receive_DMA programs the Rx stream with the data register, the buffer, direction peripheral to memory, channel and Size items") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			UartDmaRig r(k);
			REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 7) == HAL_OK);
			CHECK(HalFixture::peek(&r.rxI->PAR) == k.dr);
			CHECK(HalFixture::peek(&r.rxI->M0AR) == r.rxAddr());
			CHECK(HalFixture::peek(&r.rxI->NDTR) == 7u);
			const uint32_t cr = HalFixture::peek(&r.rxI->CR);
			CHECK((cr & uartDmaStreamEN) == uartDmaStreamEN);
			CHECK(((cr >> 6) & 3u) == 0u); // DIR 00 peripheral to memory
			CHECK(((cr & uartDmaCHSEL_MASK) >> 25) == k.channel);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD1 Transmit_DMA and Receive_DMA of 9 bit words program NDTR with the number of 16 bit items") {
	UartDmaOpt o;
	o.nine = true;
	UartDmaRig r(uartDmaCfgs[2], o);
	r.fillTx(3);
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 3) == HAL_OK);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	CHECK(HalFixture::peek(&r.txI->NDTR) == 3u);
	CHECK(HalFixture::peek(&r.rxI->NDTR) == 4u);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD1 Transmit_DMA sets DMAT and BUSY_TX and leaves the Tx interrupt enables clear") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			UartDmaRig r(k);
			r.fillTx(2);
			REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 2) == HAL_OK);
			CHECK((r.cr3() & USART_CR3_DMAT) != 0u);
			CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
			CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_BUSY_TX);
			CHECK((r.cr1() & (USART_CR1_TXEIE | USART_CR1_TCIE)) == 0u);
			CHECK((r.cr3() & USART_CR3_DMAR) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD1 Receive_DMA sets DMAR and EIE and BUSY_RX, PEIE only with parity, and leaves RXNEIE clear") {
	const uint32_t pars[3] = {UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD};
	const char *const names[3] = {"no parity", "even parity", "odd parity"};
	for (unsigned p = 0; p < 3; ++p) {
		SUBCASE(names[p]) {
			UartDmaOpt o;
			o.parity = pars[p];
			UartDmaRig r(uartDmaCfgs[2], o);
			REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
			CHECK((r.cr3() & USART_CR3_DMAR) != 0u);
			CHECK((r.cr3() & USART_CR3_EIE) != 0u);
			CHECK(((r.cr1() & USART_CR1_PEIE) != 0u) == (pars[p] != UART_PARITY_NONE));
			CHECK((r.cr1() & USART_CR1_RXNEIE) == 0u);
			CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
			CHECK((r.cr3() & USART_CR3_DMAT) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD1 Receive_DMA clears a pending ORE flag before enabling the DMA request") {
	UartDmaRig r(uartDmaCfgs[1]);
	hostUartRxNow(1, 0x55, 0);
	hostUartRxNow(1, 0x56, 0); // RXNE still set: overrun (RM0090 30.3.3)
	REQUIRE((r.sr() & USART_SR_ORE) != 0u);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	CHECK((r.sr() & USART_SR_ORE) == 0u);
	CHECK((r.cr3() & USART_CR3_DMAR) != 0u);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD1 Transmit_DMA with a NULL buffer or Size 0 returns HAL_ERROR and starts nothing") {
	UartDmaRig r(uartDmaCfgs[1]);
	CHECK(HAL_UART_Transmit_DMA(&r.h, nullptr, 4) == HAL_ERROR);
	CHECK(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 0) == HAL_ERROR);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK((r.cr3() & USART_CR3_DMAT) == 0u);
	CHECK_FALSE(r.streamEnabled(r.txI));
}

TEST_CASE_FIXTURE(UartDmaFx, "UD1 Receive_DMA with a NULL buffer or Size 0 returns HAL_ERROR and starts nothing") {
	UartDmaRig r(uartDmaCfgs[1]);
	CHECK(HAL_UART_Receive_DMA(&r.h, nullptr, 4) == HAL_ERROR);
	CHECK(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 0) == HAL_ERROR);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK((r.cr3() & USART_CR3_DMAR) == 0u);
	CHECK_FALSE(r.streamEnabled(r.rxI));
}

TEST_CASE_FIXTURE(UartDmaFx, "UD1 Transmit_DMA while a Tx transfer runs returns HAL_BUSY and leaves the running stream alone") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(5);
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 5) == HAL_OK);
	CHECK(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 3) == HAL_BUSY);
	CHECK(HalFixture::peek(&r.txI->NDTR) == 5u);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD1 Receive_DMA while an Rx transfer runs returns HAL_BUSY and leaves the running stream alone") {
	UartDmaRig r(uartDmaCfgs[1]);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 5) == HAL_OK);
	CHECK(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 3) == HAL_BUSY);
	CHECK(HalFixture::peek(&r.rxI->NDTR) == 5u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD1 a Tx and an Rx DMA transfer run at the same time and the state is BUSY_TX_RX") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(2);
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 2) == HAL_OK);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 2) == HAL_OK);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_BUSY_TX_RX);
	CHECK((r.cr3() & (USART_CR3_DMAT | USART_CR3_DMAR)) == (USART_CR3_DMAT | USART_CR3_DMAR));
}

TEST_CASE_FIXTURE(UartDmaFx, "UD1 characterization: starting a Tx DMA transfer clears the error code") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(2);
	r.h.ErrorCode = HAL_UART_ERROR_FE;
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 2) == HAL_OK);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD1 characterization: starting an Rx DMA transfer clears the error code") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.h.ErrorCode = HAL_UART_ERROR_PE;
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 2) == HAL_OK);
	CHECK(HAL_UART_GetError(&r.h) == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD1 characterization: Transmit_DMA with a Tx stream that is already busy returns HAL_OK, sets BUSY_TX and DMAT") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(2);
	REQUIRE(HAL_DMA_Start_IT(&r.tx, r.txAddr(), uartDmaCfgs[1].dr, 2) == HAL_OK);
	CHECK(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 2) == HAL_OK);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
	CHECK((r.cr3() & USART_CR3_DMAT) != 0u);
}

/* ------------------------------------------------------------------------------------------------------------------ UD2 */

TEST_CASE_FIXTURE(UartDmaFx, "UD2 the Tx DMA puts every frame on the line in order") {
	std::vector<std::string> names;
	const auto opts = uartDmaFrameOpts(names);
	for (const auto &k : uartDmaCfgs) {
		for (std::size_t o = 0; o < opts.size(); ++o) {
			SUBCASE((std::string(k.name) + ", " + names[o]).c_str()) {
				UartDmaRig r(k, opts[o]);
				r.txDmaDone(5);
				r.waitSr(USART_SR_TC); // the last frame has left the shift register
				REQUIRE(hostUartTxCount(k.idx) == 5u);
				for (unsigned i = 0; i < 5; ++i) CHECK(hostUartTxAt(k.idx, i) == r.line(i));
			}
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD2 the Tx DMA transfer complete interrupt clears DMAT, sets TCIE and calls no callback yet") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			hostUartSetFrameTicks(3);
			UartDmaRig r(k);
			r.txDmaDone(3);
			CHECK((r.cr3() & USART_CR3_DMAT) == 0u);
			CHECK((r.cr1() & USART_CR1_TCIE) != 0u);
			CHECK(r.h.TxXferCount == 0u);
			CHECK(uartCounts.txCplt == 0u);
			CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD2 the UART interrupt at TC after the Tx DMA ends the transfer with TxCpltCallback once and state READY") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			hostUartSetFrameTicks(3);
			UartDmaRig r(k);
			r.txDmaDone(3);
			r.waitSr(USART_SR_TC);
			HAL_UART_IRQHandler(&r.h);
			CHECK(uartCounts.txCplt == 1u);
			CHECK(uartCounts.last == &r.h);
			CHECK(r.h.gState == HAL_UART_STATE_READY);
			CHECK((r.cr1() & USART_CR1_TCIE) == 0u);
			HAL_UART_IRQHandler(&r.h);
			CHECK(uartCounts.txCplt == 1u);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD2 the UART interrupt before TC after the Tx DMA does not end the transfer") {
	hostUartSetFrameTicks(5);
	UartDmaRig r(uartDmaCfgs[2]);
	r.txDmaDone(2);
	REQUIRE((r.sr() & USART_SR_TC) == 0u); // last frame still in the shift register
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.txCplt == 0u);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD2 the Tx DMA half transfer interrupt calls TxHalfCpltCallback once and the transfer goes on") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			UartDmaRig r(k);
			r.fillTx(8);
			REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 8) == HAL_OK);
			hostDmaStepItems(k.c, k.txS, 4);
			r.txIrq();
			CHECK(uartCounts.txHalf == 1u);
			CHECK(uartCounts.txCplt == 0u);
			CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
			CHECK((r.cr3() & USART_CR3_DMAT) != 0u);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD2 circular Tx DMA calls TxCpltCallback at every transfer complete and keeps DMAT set and state BUSY_TX") {
	UartDmaOpt o;
	o.circTx = true;
	UartDmaRig r(uartDmaCfgs[1], o);
	r.fillTx(4);
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 4) == HAL_OK);
	r.completeTx();
	CHECK(uartCounts.txCplt == 1u);
	r.completeTx();
	CHECK(uartCounts.txCplt == 2u);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
	CHECK((r.cr3() & USART_CR3_DMAT) != 0u);
	CHECK((r.cr1() & USART_CR1_TCIE) == 0u);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD2 a new Tx DMA transfer can start after the previous one ended") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.txDmaDone(2);
	r.waitSr(USART_SR_TC);
	HAL_UART_IRQHandler(&r.h);
	REQUIRE(r.h.gState == HAL_UART_STATE_READY);
	r.fillTx(3);
	CHECK(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 3) == HAL_OK);
	CHECK(HalFixture::peek(&r.txI->NDTR) == 3u);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD2 the Rx DMA stores the frames the peer sent, the parity bit included") {
	std::vector<std::string> names;
	const auto opts = uartDmaFrameOpts(names);
	for (const auto &k : uartDmaCfgs) {
		for (std::size_t o = 0; o < opts.size(); ++o) {
			SUBCASE((std::string(k.name) + ", " + names[o]).c_str()) {
				UartDmaRig r(k, opts[o]);
				REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 5) == HAL_OK);
				r.engineRx(5);
				r.completeRx();
				for (unsigned i = 0; i < 5; ++i) CHECK(r.rxWord(i) == r.line(i));
			}
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD2 the Rx DMA transfer complete interrupt clears DMAR EIE PEIE, sets state READY and calls RxCpltCallback once") {
	const uint32_t pars[3] = {UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD};
	for (const auto &k : uartDmaCfgs) {
		for (unsigned p = 0; p < 3; ++p) {
			SUBCASE((std::string(k.name) + ", parity " + std::to_string(p)).c_str()) {
				UartDmaOpt o;
				o.parity = pars[p];
				UartDmaRig r(k, o);
				REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 3) == HAL_OK);
				r.engineRx(3);
				r.completeRx();
				CHECK(uartCounts.rxCplt == 1u);
				CHECK(uartCounts.last == &r.h);
				CHECK(r.h.RxState == HAL_UART_STATE_READY);
				CHECK(r.h.RxXferCount == 0u);
				CHECK((r.cr3() & (USART_CR3_DMAR | USART_CR3_EIE)) == 0u);
				CHECK((r.cr1() & USART_CR1_PEIE) == 0u);
				CHECK(uartCounts.rxEvent == 0u);
			}
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD2 the Rx DMA half transfer interrupt calls RxHalfCpltCallback once and the reception goes on") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			UartDmaRig r(k);
			REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 8) == HAL_OK);
			r.engineRx(4);
			r.rxStep(4);
			r.rxIrq();
			CHECK(uartCounts.rxHalf == 1u);
			CHECK(uartCounts.rxCplt == 0u);
			CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
			CHECK((r.cr3() & USART_CR3_DMAR) != 0u);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD2 circular Rx DMA calls RxCpltCallback at every transfer complete and keeps DMAR and EIE set") {
	UartDmaOpt o;
	o.circRx = true;
	UartDmaRig r(uartDmaCfgs[1], o);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 3) == HAL_OK);
	r.engineRx(3);
	r.completeRx();
	CHECK(uartCounts.rxCplt == 1u);
	r.engineRx(3);
	r.completeRx();
	CHECK(uartCounts.rxCplt == 2u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK((r.cr3() & (USART_CR3_DMAR | USART_CR3_EIE)) == (USART_CR3_DMAR | USART_CR3_EIE));
}

TEST_CASE_FIXTURE(UartDmaFx, "UD2 a new Rx DMA transfer can start after the previous one ended and stores the new frames") {
	UartDmaRig r(uartDmaCfgs[1]);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 2) == HAL_OK);
	r.engineRx(2);
	r.completeRx();
	REQUIRE(r.h.RxState == HAL_UART_STATE_READY);
	std::memset(r.rxBuf, 0, sizeof r.rxBuf);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 2) == HAL_OK);
	r.engineRx(2, 2); // the peer sends items 2 and 3 of the pattern; the engine stores them at index 2, 3
	r.completeRx();
	CHECK(uartCounts.rxCplt == 2u);
	CHECK(r.rxWord(2) == r.line(2));
	CHECK(r.rxWord(3) == r.line(3));
}

/* ------------------------------------------------------------------------------------------------------------------ UD3 */

TEST_CASE_FIXTURE(UartDmaFx, "UD3 a stream error of the Tx stream gives HAL_UART_ERROR_DMA, one ErrorCallback and state READY") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			UartDmaRig r(k);
			r.fillTx(4);
			REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 4) == HAL_OK);
			hostDmaRaise(k.c, k.txS, HOST_DMA_TEIF);
			r.txIrq();
			CHECK(uartCounts.error == 1u);
			CHECK((uartCounts.errorCodeAtError & HAL_UART_ERROR_DMA) == HAL_UART_ERROR_DMA);
			CHECK((HAL_UART_GetError(&r.h) & HAL_UART_ERROR_DMA) == HAL_UART_ERROR_DMA);
			CHECK(r.h.gState == HAL_UART_STATE_READY);
			CHECK((r.cr1() & (USART_CR1_TXEIE | USART_CR1_TCIE)) == 0u);
			CHECK(r.h.TxXferCount == 0u);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD3 a stream error of the Rx stream gives HAL_UART_ERROR_DMA, one ErrorCallback and state READY") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			UartDmaRig r(k);
			REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
			hostDmaRaise(k.c, k.rxS, HOST_DMA_TEIF);
			r.rxIrq();
			CHECK(uartCounts.error == 1u);
			CHECK((uartCounts.errorCodeAtError & HAL_UART_ERROR_DMA) == HAL_UART_ERROR_DMA);
			CHECK(r.h.RxState == HAL_UART_STATE_READY);
			CHECK((r.cr1() & (USART_CR1_RXNEIE | USART_CR1_PEIE)) == 0u);
			CHECK((r.cr3() & USART_CR3_EIE) == 0u);
			CHECK(r.h.RxXferCount == 0u);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD3 a stream error of the Rx stream during ReceiveToIdle_DMA clears IDLEIE") {
	UartDmaRig r(uartDmaCfgs[1]);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	REQUIRE((r.cr1() & USART_CR1_IDLEIE) != 0u);
	hostDmaRaise(0, 5, HOST_DMA_TEIF);
	r.rxIrq();
	CHECK(uartCounts.error == 1u);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK((r.cr1() & USART_CR1_IDLEIE) == 0u);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD3 characterization: a stream error of the Tx stream also ends a running Rx DMA reception") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(4);
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 4) == HAL_OK);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	hostDmaRaise(0, 6, HOST_DMA_TEIF);
	r.txIrq();
	CHECK(uartCounts.error == 1u);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

namespace {
struct UartDmaErrKind {
	const char *name;
	uint32_t srFlag;
	uint32_t code;
};
const UartDmaErrKind uartDmaErrKinds[] = {
    {"overrun", USART_SR_ORE, HAL_UART_ERROR_ORE},
    {"framing error", USART_SR_FE, HAL_UART_ERROR_FE},
    {"noise error", USART_SR_NE, HAL_UART_ERROR_NE},
    {"parity error", USART_SR_PE, HAL_UART_ERROR_PE},
};

/* Starts an Rx DMA of 4 items (parity even when the error is PE) and makes the peer raise the error with the DMA having read the frame. */
void uartDmaRaiseRxError(UartDmaRig &r, const UartDmaErrKind &e) {
	const unsigned n = r.cfg.idx;
	if (e.srFlag == USART_SR_ORE) {
		hostUartRxNow(n, 0x41, 0); // not read yet: the DMA is late
		hostUartRxNow(n, 0x42, 0); // RXNE still set: overrun
	} else if (e.srFlag == USART_SR_FE) {
		hostUartRxNow(n, 0x41, HOST_UART_RX_FE);
		(void)r.inst->DR;
	} else if (e.srFlag == USART_SR_NE) {
		hostUartRxNow(n, 0x41, HOST_UART_RX_NOISE);
		(void)r.inst->DR;
	} else {
		hostUartRxNow(n, 0xC1, 0); // even parity configured: 0xC1 holds three ones
		(void)r.inst->DR;
	}
}
} // namespace

TEST_CASE_FIXTURE(UartDmaFx, "UD3 an Rx error during Rx DMA aborts the stream, ends the reception at once and calls ErrorCallback after the DMA abort callback") {
	for (const auto &e : uartDmaErrKinds) {
		SUBCASE(e.name) {
			UartDmaOpt o;
			o.parity = UART_PARITY_EVEN; // PE needs parity; the other errors are the same with parity
			UartDmaRig r(uartDmaCfgs[1], o);
			REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
			uartDmaRaiseRxError(r, e);
			REQUIRE((r.sr() & e.srFlag) != 0u);
			HAL_UART_IRQHandler(&r.h);
			CHECK(r.h.RxState == HAL_UART_STATE_READY);
			CHECK((r.cr3() & (USART_CR3_DMAR | USART_CR3_EIE)) == 0u);
			CHECK((r.cr1() & (USART_CR1_PEIE | USART_CR1_RXNEIE)) == 0u);
			CHECK_FALSE(r.streamEnabled(r.rxI));
			CHECK(uartCounts.error == 0u); // runs when the DMA abort has completed
			r.rxIrq();
			CHECK(uartCounts.error == 1u);
			CHECK((uartCounts.errorCodeAtError & e.code) == e.code);
			CHECK(uartCounts.rxCplt == 0u);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD3 an Rx error on each USART instance ends the Rx DMA reception") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			UartDmaRig r(k);
			REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
			hostUartRxNow(k.idx, 0x41, HOST_UART_RX_FE);
			(void)r.inst->DR;
			HAL_UART_IRQHandler(&r.h);
			r.rxIrq();
			CHECK(uartCounts.error == 1u);
			CHECK((uartCounts.errorCodeAtError & HAL_UART_ERROR_FE) == HAL_UART_ERROR_FE);
			CHECK(r.h.RxState == HAL_UART_STATE_READY);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD3 an Rx error during ReceiveToIdle_DMA ends the reception and clears IDLEIE") {
	UartDmaRig r(uartDmaCfgs[1]);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	hostUartRxNow(1, 0x41, 0);
	hostUartRxNow(1, 0x42, 0); // overrun
	HAL_UART_IRQHandler(&r.h);
	r.rxIrq();
	CHECK(uartCounts.error == 1u);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK((r.cr1() & USART_CR1_IDLEIE) == 0u);
	CHECK(uartCounts.rxEvent == 0u);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD3 characterization: an Rx error with the DMA stream no longer running calls ErrorCallback at once") {
	UartDmaRig r(uartDmaCfgs[1]);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	REQUIRE(HAL_DMA_Abort(&r.rx) == HAL_OK);
	hostUartRxNow(1, 0x41, 0);
	hostUartRxNow(1, 0x42, 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 1u);
	CHECK((uartCounts.errorCodeAtError & HAL_UART_ERROR_ORE) == HAL_UART_ERROR_ORE);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD3 characterization: an Rx error with a NULL hdmarx calls ErrorCallback at once") {
	UartDmaRig r(uartDmaCfgs[1]);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	r.h.hdmarx = nullptr;
	hostUartRxNow(1, 0x41, 0);
	hostUartRxNow(1, 0x42, 0);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.error == 1u);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK((r.cr3() & USART_CR3_DMAR) == 0u);
}

/* ------------------------------------------------------------------------------------------------------------------ UD4 */

TEST_CASE_FIXTURE(UartDmaFx, "UD4 DMAPause clears DMAT of a running Tx DMA transfer and keeps the state and the stream") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(4);
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 4) == HAL_OK);
	CHECK(HAL_UART_DMAPause(&r.h) == HAL_OK);
	CHECK((r.cr3() & USART_CR3_DMAT) == 0u);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
	CHECK(r.streamEnabled(r.txI));
}

TEST_CASE_FIXTURE(UartDmaFx, "UD4 DMAResume sets DMAT again after a DMAPause of a Tx DMA transfer") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(4);
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 4) == HAL_OK);
	REQUIRE(HAL_UART_DMAPause(&r.h) == HAL_OK);
	CHECK(HAL_UART_DMAResume(&r.h) == HAL_OK);
	CHECK((r.cr3() & USART_CR3_DMAT) != 0u);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD4 DMAPause clears DMAR EIE and PEIE of a running Rx DMA transfer and keeps the state and the stream") {
	UartDmaOpt o;
	o.parity = UART_PARITY_EVEN;
	UartDmaRig r(uartDmaCfgs[1], o);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	REQUIRE((r.cr1() & USART_CR1_PEIE) != 0u);
	CHECK(HAL_UART_DMAPause(&r.h) == HAL_OK);
	CHECK((r.cr3() & (USART_CR3_DMAR | USART_CR3_EIE)) == 0u);
	CHECK((r.cr1() & USART_CR1_PEIE) == 0u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK(r.streamEnabled(r.rxI));
}

TEST_CASE_FIXTURE(UartDmaFx, "UD4 DMAPause of both directions clears DMAT and DMAR") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(2);
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 2) == HAL_OK);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 2) == HAL_OK);
	CHECK(HAL_UART_DMAPause(&r.h) == HAL_OK);
	CHECK((r.cr3() & (USART_CR3_DMAT | USART_CR3_DMAR)) == 0u);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD4 DMAPause and DMAResume with nothing running return HAL_OK and change no register") {
	UartDmaRig r(uartDmaCfgs[1]);
	const uint32_t c1 = r.cr1(), c3 = r.cr3();
	CHECK(HAL_UART_DMAPause(&r.h) == HAL_OK);
	CHECK(HAL_UART_DMAResume(&r.h) == HAL_OK);
	CHECK(r.cr1() == c1);
	CHECK(r.cr3() == c3);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD4 DMAResume of an Rx DMA transfer clears ORE and sets EIE and DMAR, and PEIE only with parity") {
	const uint32_t pars[2] = {UART_PARITY_NONE, UART_PARITY_ODD};
	for (unsigned p = 0; p < 2; ++p) {
		SUBCASE(p == 0 ? "no parity" : "odd parity") {
			UartDmaOpt o;
			o.parity = pars[p];
			UartDmaRig r(uartDmaCfgs[1], o);
			REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
			REQUIRE(HAL_UART_DMAPause(&r.h) == HAL_OK);
			hostUartRxNow(1, 0x41, 0);
			hostUartRxNow(1, 0x42, 0); // overrun while paused
			REQUIRE((r.sr() & USART_SR_ORE) != 0u);
			CHECK(HAL_UART_DMAResume(&r.h) == HAL_OK);
			CHECK((r.sr() & USART_SR_ORE) == 0u);
			CHECK((r.cr3() & (USART_CR3_DMAR | USART_CR3_EIE)) == (USART_CR3_DMAR | USART_CR3_EIE));
			CHECK(((r.cr1() & USART_CR1_PEIE) != 0u) == (pars[p] != UART_PARITY_NONE));
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD4 DMAStop of a Tx DMA transfer clears DMAT, aborts the stream and sets state READY") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			UartDmaRig r(k);
			r.fillTx(4);
			REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 4) == HAL_OK);
			CHECK(HAL_UART_DMAStop(&r.h) == HAL_OK);
			CHECK((r.cr3() & USART_CR3_DMAT) == 0u);
			CHECK_FALSE(r.streamEnabled(r.txI));
			CHECK(r.h.gState == HAL_UART_STATE_READY);
			CHECK((r.cr1() & (USART_CR1_TXEIE | USART_CR1_TCIE)) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD4 DMAStop of an Rx DMA transfer clears DMAR EIE PEIE, aborts the stream and sets state READY") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			UartDmaOpt o;
			o.parity = UART_PARITY_EVEN;
			UartDmaRig r(k, o);
			REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
			CHECK(HAL_UART_DMAStop(&r.h) == HAL_OK);
			CHECK((r.cr3() & (USART_CR3_DMAR | USART_CR3_EIE)) == 0u);
			CHECK((r.cr1() & (USART_CR1_PEIE | USART_CR1_RXNEIE)) == 0u);
			CHECK_FALSE(r.streamEnabled(r.rxI));
			CHECK(r.h.RxState == HAL_UART_STATE_READY);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD4 DMAStop of both directions ends both and a new transfer can start") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(2);
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 2) == HAL_OK);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 2) == HAL_OK);
	CHECK(HAL_UART_DMAStop(&r.h) == HAL_OK);
	CHECK(HAL_UART_GetState(&r.h) == HAL_UART_STATE_READY);
	CHECK(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 2) == HAL_OK);
	CHECK(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 2) == HAL_OK);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD4 DMAStop with nothing running returns HAL_OK and changes no register") {
	UartDmaRig r(uartDmaCfgs[1]);
	const uint32_t c1 = r.cr1(), c3 = r.cr3();
	CHECK(HAL_UART_DMAStop(&r.h) == HAL_OK);
	CHECK(r.cr1() == c1);
	CHECK(r.cr3() == c3);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD4 DMAStop after a DMAPause stops the paused Tx transfer: state READY and stream aborted" *
                                 KNOWN_ISSUE("candidate vendor bug V104: DMAStop only acts when DMAT is set, so after HAL_UART_DMAPause (DMAT cleared) it does nothing: the stream keeps running and gState stays BUSY_TX (UM1725: DMAStop 'Stops the DMA Transfer'; a paused transfer is still a transfer)")) {
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(4);
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 4) == HAL_OK);
	REQUIRE(HAL_UART_DMAPause(&r.h) == HAL_OK);
	CHECK(HAL_UART_DMAStop(&r.h) == HAL_OK);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK_FALSE(r.streamEnabled(r.txI));
}

TEST_CASE_FIXTURE(UartDmaFx, "UD4 DMAStop after a DMAPause stops the paused Rx transfer: state READY and stream aborted" *
                                 KNOWN_ISSUE("candidate vendor bug V104 (Rx side): DMAStop only acts when DMAR is set, so after HAL_UART_DMAPause (DMAR cleared) it does nothing: the stream keeps running and RxState stays BUSY_RX (UM1725: DMAStop 'Stops the DMA Transfer')")) {
	UartDmaRig r(uartDmaCfgs[1]);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	REQUIRE(HAL_UART_DMAPause(&r.h) == HAL_OK);
	CHECK(HAL_UART_DMAStop(&r.h) == HAL_OK);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK_FALSE(r.streamEnabled(r.rxI));
}

TEST_CASE_FIXTURE(UartDmaFx, "UD4 characterization: DMAStop with a NULL hdmatx or hdmarx skips the stream abort and still ends the transfer") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(2);
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 2) == HAL_OK);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 2) == HAL_OK);
	r.h.hdmatx = nullptr;
	r.h.hdmarx = nullptr;
	CHECK(HAL_UART_DMAStop(&r.h) == HAL_OK);
	CHECK(r.h.gState == HAL_UART_STATE_READY);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK(r.streamEnabled(r.txI)); // the streams were not touched
	CHECK(r.streamEnabled(r.rxI));
}

/* ------------------------------------------------------------------------------------------------------------------ UD5 */

TEST_CASE_FIXTURE(UartDmaFx, "UD5 ReceiveToIdle_DMA starts the Rx stream, sets IDLEIE DMAR EIE and BUSY_RX and the event type TC") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			UartDmaRig r(k);
			REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 6) == HAL_OK);
			CHECK((r.cr1() & USART_CR1_IDLEIE) != 0u);
			CHECK((r.cr3() & (USART_CR3_DMAR | USART_CR3_EIE)) == (USART_CR3_DMAR | USART_CR3_EIE));
			CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
			CHECK(HalFixture::peek(&r.rxI->PAR) == k.dr);
			CHECK(HalFixture::peek(&r.rxI->M0AR) == r.rxAddr());
			CHECK(HalFixture::peek(&r.rxI->NDTR) == 6u);
			CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
			CHECK((r.cr1() & USART_CR1_RXNEIE) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD5 ReceiveToIdle_DMA clears a pending IDLE flag when it starts") {
	UartDmaRig r(uartDmaCfgs[1]);
	hostUartRxNow(1, 0x41, 0);
	(void)r.inst->DR;
	hostUartIdleLine(1);
	REQUIRE((r.sr() & USART_SR_IDLE) != 0u);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	CHECK((r.sr() & USART_SR_IDLE) == 0u);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD5 ReceiveToIdle_DMA with a NULL buffer or Size 0 returns HAL_ERROR and with a running reception HAL_BUSY") {
	UartDmaRig r(uartDmaCfgs[1]);
	CHECK(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, nullptr, 4) == HAL_ERROR);
	CHECK(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 0) == HAL_ERROR);
	CHECK(r.h.RxState == HAL_UART_STATE_READY);
	CHECK_FALSE(r.streamEnabled(r.rxI));
	REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	CHECK(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 4) == HAL_BUSY);
	CHECK(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_BUSY);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD5 an idle line after some items in normal mode gives the RxEvent with the received length and ends the reception") {
	std::vector<std::string> names;
	const auto opts = uartDmaFrameOpts(names);
	for (const auto &k : uartDmaCfgs) {
		for (std::size_t o = 0; o < opts.size(); ++o) {
			SUBCASE((std::string(k.name) + ", " + names[o]).c_str()) {
				UartDmaRig r(k, opts[o]);
				REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 8) == HAL_OK);
				r.engineRx(3);
				r.rxStep(3);
				hostUartIdleLine(k.idx);
				HAL_UART_IRQHandler(&r.h);
				CHECK(uartCounts.rxEvent == 1u);
				CHECK(uartCounts.rxEventSize == 3u);
				CHECK(uartCounts.rxCplt == 0u);
				CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_IDLE);
				CHECK(r.h.RxState == HAL_UART_STATE_READY);
				CHECK((r.cr3() & (USART_CR3_DMAR | USART_CR3_EIE)) == 0u);
				CHECK((r.cr1() & (USART_CR1_IDLEIE | USART_CR1_PEIE)) == 0u);
				CHECK_FALSE(r.streamEnabled(r.rxI));
				CHECK((r.sr() & USART_SR_IDLE) == 0u);
				for (unsigned i = 0; i < 3; ++i) CHECK(r.rxWord(i) == r.line(i));
			}
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD5 a second idle line after the reception ended gives no further event") {
	UartDmaRig r(uartDmaCfgs[1]);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 8) == HAL_OK);
	r.engineRx(2);
	r.rxStep(2);
	hostUartIdleLine(1);
	HAL_UART_IRQHandler(&r.h);
	REQUIRE(uartCounts.rxEvent == 1u);
	hostUartRxNow(1, 0x41, 0);
	(void)r.inst->DR;
	hostUartIdleLine(1);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.rxEvent == 1u);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD5 an idle line after some items in circular mode gives the RxEvent and the reception goes on") {
	UartDmaOpt o;
	o.circRx = true;
	UartDmaRig r(uartDmaCfgs[1], o);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 8) == HAL_OK);
	r.engineRx(3);
	r.rxStep(3);
	hostUartIdleLine(1);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.rxEvent == 1u);
	CHECK(uartCounts.rxEventSize == 3u);
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_IDLE);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK((r.cr3() & USART_CR3_DMAR) != 0u);
	CHECK((r.cr1() & USART_CR1_IDLEIE) != 0u);
	CHECK(r.streamEnabled(r.rxI));
}

TEST_CASE_FIXTURE(UartDmaFx, "UD5 an idle line with 9 bit words gives the length in items") {
	UartDmaOpt o;
	o.nine = true;
	UartDmaRig r(uartDmaCfgs[1], o);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	r.engineRx(2);
	r.rxStep(2);
	hostUartIdleLine(1);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.rxEvent == 1u);
	CHECK(uartCounts.rxEventSize == 2u);
	CHECK(r.rxBuf[0] == r.data(0));
	CHECK(r.rxBuf[1] == r.data(1));
}

TEST_CASE_FIXTURE(UartDmaFx, "UD5 characterization: an idle line before the DMA moved any item gives no RxEvent and leaves the reception running") {
	UartDmaRig r(uartDmaCfgs[1]);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 8) == HAL_OK);
	hostUartRxNow(1, 0x41, 0);
	(void)r.inst->DR; // frame read, DMA counter not stepped
	hostUartIdleLine(1);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.rxEvent == 0u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK((r.sr() & USART_SR_IDLE) == 0u);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD5 a complete ReceiveToIdle_DMA transfer gives the RxEvent with Size, type TC, and ends the reception") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			UartDmaRig r(k);
			REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 5) == HAL_OK);
			r.engineRx(5);
			r.completeRx();
			CHECK(uartCounts.rxEvent == 1u);
			CHECK(uartCounts.rxEventSize == 5u);
			CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
			CHECK(uartCounts.rxCplt == 0u);
			CHECK(r.h.RxState == HAL_UART_STATE_READY);
			CHECK((r.cr3() & (USART_CR3_DMAR | USART_CR3_EIE)) == 0u);
			CHECK((r.cr1() & USART_CR1_IDLEIE) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD5 the half transfer of ReceiveToIdle_DMA gives the RxEvent with Size / 2, type HT, and the reception goes on") {
	UartDmaRig r(uartDmaCfgs[1]);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 8) == HAL_OK);
	r.engineRx(4);
	r.rxStep(4);
	r.rxIrq();
	CHECK(uartCounts.rxEvent == 1u);
	CHECK(uartCounts.rxEventSize == 4u);
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_HT);
	CHECK(uartCounts.rxHalf == 0u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK((r.cr3() & USART_CR3_DMAR) != 0u);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD5 circular ReceiveToIdle_DMA gives the Size event at every transfer complete and keeps going") {
	UartDmaOpt o;
	o.circRx = true;
	UartDmaRig r(uartDmaCfgs[1], o);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	r.engineRx(4);
	r.completeRx();
	r.engineRx(4);
	r.completeRx();
	CHECK(uartCounts.rxEvent == 2u);
	CHECK(uartCounts.rxEventSize == 4u);
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK((r.cr1() & USART_CR1_IDLEIE) != 0u);
}

/* ------------------------------------------------------------------------------------------------------------------ UD6 */

TEST_CASE_FIXTURE(UartDmaFx, "UD6 Transmit_DMA clears the TC flag left by a polling transmit") {
	for (const auto &k : uartDmaCfgs) {
		SUBCASE(k.name) {
			UartDmaRig r(k);
			r.fillTx(2);
			REQUIRE(HAL_UART_Transmit(&r.h, r.txPtr(), 1, 100) == HAL_OK);
			REQUIRE((r.sr() & USART_SR_TC) != 0u);
			REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 2) == HAL_OK);
			CHECK((r.sr() & USART_SR_TC) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(UartDmaFx, "UD6 the end of a Tx DMA transfer after a polling transmit is the TC of the last DMA frame") {
	hostUartSetFrameTicks(4);
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(2);
	REQUIRE(HAL_UART_Transmit(&r.h, r.txPtr(), 1, 100) == HAL_OK);
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 2) == HAL_OK);
	// the DMA has not moved a frame yet; the DMA transfer complete interrupt comes first, then the UART interrupt
	hostDmaComplete(0, 6);
	HAL_DMA_IRQHandler(&r.tx);
	HAL_UART_IRQHandler(&r.h);
	CHECK(uartCounts.txCplt == 0u);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
}

/* ---------------------------------------------------------------------------------------------------- fix pass additions */

TEST_CASE_FIXTURE(UartDmaFx, "UD4 DMAPause during an Rx interrupt transfer leaves PEIE and EIE of that transfer set") {
	UartDmaOpt o;
	o.parity = UART_PARITY_EVEN;
	UartDmaRig r(uartDmaCfgs[1], o);
	REQUIRE(HAL_UART_Receive_IT(&r.h, r.rxPtr(), 4) == HAL_OK);
	REQUIRE((r.cr1() & USART_CR1_PEIE) != 0u);
	REQUIRE((r.cr3() & USART_CR3_EIE) != 0u);
	CHECK(HAL_UART_DMAPause(&r.h) == HAL_OK); // DMAR is not set: not a DMA reception
	CHECK((r.cr1() & (USART_CR1_PEIE | USART_CR1_RXNEIE)) == (USART_CR1_PEIE | USART_CR1_RXNEIE));
	CHECK((r.cr3() & USART_CR3_EIE) != 0u);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD4 DMAPause during a Tx interrupt transfer leaves TXEIE set") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(3);
	REQUIRE(HAL_UART_Transmit_IT(&r.h, r.txPtr(), 3) == HAL_OK);
	CHECK(HAL_UART_DMAPause(&r.h) == HAL_OK);
	CHECK((r.cr1() & USART_CR1_TXEIE) != 0u);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX);
	CHECK(HAL_UART_DMAResume(&r.h) == HAL_OK); // Tx busy: the HAL sets DMAT (characterization of the code is not asserted here)
}

TEST_CASE_FIXTURE(UartDmaFx, "UD3 a Tx stream error while the Tx transfer is paused (DMAT clear) keeps the Tx transfer busy") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(4);
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 4) == HAL_OK);
	REQUIRE(HAL_UART_DMAPause(&r.h) == HAL_OK);
	hostDmaRaise(0, 6, HOST_DMA_TEIF);
	r.txIrq();
	CHECK(uartCounts.error == 1u);
	CHECK((uartCounts.errorCodeAtError & HAL_UART_ERROR_DMA) == HAL_UART_ERROR_DMA);
	CHECK(r.h.gState == HAL_UART_STATE_BUSY_TX); // the Tx transfer is only ended when DMAT is set (characterization of UART_DMAError)
	CHECK(r.h.TxXferCount == 4u);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD3 an Rx stream error while the Rx transfer is paused (DMAR clear) keeps the Rx transfer busy") {
	UartDmaRig r(uartDmaCfgs[1]);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	REQUIRE(HAL_UART_DMAPause(&r.h) == HAL_OK);
	hostDmaRaise(0, 5, HOST_DMA_TEIF);
	r.rxIrq();
	CHECK(uartCounts.error == 1u);
	CHECK((uartCounts.errorCodeAtError & HAL_UART_ERROR_DMA) == HAL_UART_ERROR_DMA);
	CHECK(r.h.RxState == HAL_UART_STATE_BUSY_RX);
	CHECK(r.h.RxXferSize == 4u);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD2 a plain Rx DMA complete after an IDLE event sets the event type back to TC and calls RxCpltCallback") {
	UartDmaRig r(uartDmaCfgs[1]);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 8) == HAL_OK);
	r.engineRx(2);
	r.rxStep(2);
	hostUartIdleLine(1);
	HAL_UART_IRQHandler(&r.h);
	REQUIRE(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_IDLE);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 2) == HAL_OK);
	r.engineRx(2);
	r.completeRx();
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
	CHECK(uartCounts.rxCplt == 1u);
	CHECK(uartCounts.rxEvent == 1u); // only the idle event: a standard reception gives no RxEvent
}

TEST_CASE_FIXTURE(UartDmaFx, "UD2 the transfer complete of a ReceiveToIdle_DMA that had a half transfer event has event type TC") {
	UartDmaRig r(uartDmaCfgs[1]);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 4) == HAL_OK);
	r.engineRx(2);
	r.rxStep(2);
	r.rxIrq();
	REQUIRE(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_HT);
	r.engineRx(2, 2);
	r.completeRx();
	CHECK(HAL_UARTEx_GetRxEventType(&r.h) == HAL_UART_RXEVENT_TC);
	CHECK(uartCounts.rxEvent == 2u);
	CHECK(uartCounts.rxEventSize == 4u);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD2 characterization: after a normal-mode ReceiveToIdle_DMA completes the reception type is still TOIDLE until the next start") {
	UartDmaRig r(uartDmaCfgs[1]);
	REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&r.h, r.rxPtr(), 2) == HAL_OK);
	r.engineRx(2);
	r.completeRx();
	CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_TOIDLE);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 2) == HAL_OK);
	CHECK(r.h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
}

TEST_CASE_FIXTURE(UartDmaFx, "UD1 characterization: Transmit_DMA and Receive_DMA store Size and the buffer in the handle") {
	UartDmaRig r(uartDmaCfgs[1]);
	r.fillTx(6);
	REQUIRE(HAL_UART_Transmit_DMA(&r.h, r.txPtr(), 6) == HAL_OK);
	REQUIRE(HAL_UART_Receive_DMA(&r.h, r.rxPtr(), 7) == HAL_OK);
	CHECK(r.h.TxXferSize == 6u);
	CHECK(r.h.TxXferCount == 6u);
	CHECK(r.h.pTxBuffPtr == r.txPtr());
	CHECK(r.h.RxXferSize == 7u);
	CHECK(r.h.pRxBuffPtr == r.rxPtr());
}
