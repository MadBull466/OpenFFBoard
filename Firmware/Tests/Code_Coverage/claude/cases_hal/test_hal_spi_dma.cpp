/*
 * P12 T4a group SD: the DMA mode of the real ST HAL SPI driver (stm32f4xx_hal_spi.c: HAL_SPI_Transmit_DMA, Receive_DMA,
 * TransmitReceive_DMA, Abort, Abort_IT, DMAPause, DMAResume, DMAStop, IRQHandler) together with the real HAL DMA, against the
 * SPI model (fakes/stm32_hal/HalSpi.h) and the DMA model (fakes/stm32_hal/HalDma.h). Requirements SD1-SD6 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T4A_REQUIREMENTS.md; compiled into ffb_hal_tests (F407VG) and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 (SPI chapter 66: DMA functions, Abort notes, error code names; DMA chapter 21) and RM0090 Rev 22:
 *  - Table 43 (DMA1) and Table 44 (DMA2): SPI3_RX DMA1 stream 0 channel 0, SPI2_RX stream 3 ch 0, SPI2_TX stream 4 ch 0,
 *    SPI3_TX stream 5 ch 0; SPI1_RX DMA2 stream 2 ch 3, SPI1_TX stream 3 ch 3.
 *  - Table 1 and 28.5.4: SPI1 0x40013000, SPI2 0x40003800, SPI3 0x40003C00; SPI_DR at offset 0x0C.
 *  - 10.5.5 DMA_SxCR: EN bit 0, TEIE 2, HTIE 3, TCIE 4, DIR bits 7:6 (00 peripheral to memory, 01 memory to peripheral),
 *    CHSEL bits 27:25. 10.5.6 NDTR counts data items (not bytes).
 *  - 28.5.1 CR1: SPE bit 6, DFF bit 11, BIDIOE bit 14, BIDIMODE bit 15. 28.5.2 CR2: RXDMAEN bit 0, TXDMAEN bit 1, ERRIE bit 5.
 *    28.5.3 SR: RXNE bit 0, TXE bit 1, OVR bit 6, BSY bit 7.
 *  - 28.3.8: in transmit mode wait TXE = 1 then BSY = 0 before disabling the SPI. 28.3.9: a DMA request per TXE / RXNE; with only the
 *    Tx stream enabled OVR is set because the received data is not read; the BSY flag is monitored after TCIF.
 * The DMA streams are configured by the test (the SPI HAL does not configure them) and the tests stand in for the DMA engine: they
 * write / read SPI_DR frame by frame and call hostDmaComplete / hostDmaStepItems / hostDmaRaise, then HAL_DMA_IRQHandler.
 * Time is the HAL tick: every HAL_GetTick() call advances it by 1 ms, so waits end by themselves.
 * Every file-local symbol is prefixed spiDma or sits in the anonymous namespace.
 *
 * NEEDS REVIEW proposals (numbers assigned by the lead from 307):
 *  N1 SD1: a refused Transmit_DMA / Receive_DMA returns HAL_BUSY (not READY), HAL_ERROR (NULL buffer, Size 0); UM1725 only says "HAL status".
 *  N2 SD1: when the DMA stream cannot be started the HAL returns HAL_ERROR, sets HAL_SPI_ERROR_DMA, unlocks and keeps the running stream
 *     (the state BUSY_TX left behind is the KNOWN_ISSUE V101 case).
 *  N8 SD2: completion of a DMA transfer clears ERRIE (UM1725 silent).
 *  N3 SD2: a transfer complete with a bus that never ends the last frame ends with HAL_SPI_ERROR_FLAG after about 100 ticks
 *     (SPI_DEFAULT_TIMEOUT) and the error callback instead of the complete callback.
 *  N4 SD2: with a circular DMA stream the complete callback runs on every TCIF, the state stays BUSY_TX and the SPI DMA enable stays set.
 *  N5 SD4: Abort of a Tx only DMA transfer leaves SPE set; of a transfer with an Rx stream clears SPE without waiting for BSY (a running frame is cut).
 *  N6 SD5: DMAStop after a Tx only transfer returns HAL_ERROR and sets HAL_SPI_ERROR_DMA because the idle Rx stream cannot be aborted;
 *     DMAResume also sets RXDMAEN in a Tx only transfer.
 *  N7 SD6: 16 bit frames with a byte aligned DMA stream: NDTR is Size (no conversion), the stream moves 8 bit items.
 */
#include "HalDma.h"
#include "HalFixture.h"
#include "HalHostSystem.h"
#include "HalSpi.h"
#include "SpiTestSupport.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr uint32_t spiDmaCR1_SPE = 1u << 6;
constexpr uint32_t spiDmaCR1_BIDIOE = 1u << 14;
constexpr uint32_t spiDmaCR2_RXDMAEN = 1u << 0;
constexpr uint32_t spiDmaCR2_TXDMAEN = 1u << 1;
constexpr uint32_t spiDmaCR2_ERRIE = 1u << 5;
constexpr uint32_t spiDmaCR2_RXNEIE = 1u << 6;
constexpr uint32_t spiDmaCR2_TXEIE = 1u << 7;
constexpr uint32_t spiDmaSR_RXNE = 1u << 0;
constexpr uint32_t spiDmaSR_TXE = 1u << 1;
constexpr uint32_t spiDmaSR_OVR = 1u << 6;
constexpr uint32_t spiDmaSR_BSY = 1u << 7;
constexpr uint32_t spiDmaDMA_EN = 1u << 0;
constexpr uint32_t spiDmaDMA_TEIE = 1u << 2;
constexpr uint32_t spiDmaDMA_HTIE = 1u << 3;
constexpr uint32_t spiDmaDMA_TCIE = 1u << 4;
constexpr uint32_t spiDmaDMA_DIR_SHIFT = 6;
constexpr uint32_t spiDmaDMA_DIR_MASK = 3u << 6;
constexpr uint32_t spiDmaDMA_CHSEL_SHIFT = 25;
constexpr uint32_t spiDmaDMA_CHSEL_MASK = 7u << 25;

struct SpiDmaCfg {
	unsigned n;     // 0 = SPI1, 1 = SPI2, 2 = SPI3
	bool b16;
	std::string name;
};

std::vector<SpiDmaCfg> spiDmaConfigs() {
	std::vector<SpiDmaCfg> v;
	const char *const inst[3] = {"SPI1", "SPI2", "SPI3"};
	for (unsigned n = 0; n < 3; ++n)
		for (int b = 0; b < 2; ++b) v.push_back({n, b == 1, std::string(inst[n]) + (b ? " 16 bit" : " 8 bit")});
	return v;
}

SPI_TypeDef *spiDmaInstance(unsigned n) {
	SPI_TypeDef *const a[3] = {SPI1, SPI2, SPI3};
	return a[n];
}

/* RM0090 Tables 1, 28.5.4 (DR offset 0x0C). */
constexpr uint32_t spiDmaDrAddress[3] = {0x4001300Cu, 0x4000380Cu, 0x40003C0Cu};

struct SpiDmaMap {
	unsigned c;  // 0 = DMA1, 1 = DMA2
	unsigned txS, rxS;
	uint32_t channel;
	DMA_Stream_TypeDef *txI, *rxI;
};

/* RM0090 Tables 43 and 44. */
SpiDmaMap spiDmaMapOf(unsigned n) {
	if (n == 0) return {1, 3, 2, DMA_CHANNEL_3, DMA2_Stream3, DMA2_Stream2};
	if (n == 1) return {0, 4, 3, DMA_CHANNEL_0, DMA1_Stream4, DMA1_Stream3};
	return {0, 5, 0, DMA_CHANNEL_0, DMA1_Stream5, DMA1_Stream0};
}

struct SpiDmaRig {
	unsigned n;
	bool b16;
	SpiDmaMap m;
	SPI_TypeDef *inst;
	SPI_HandleTypeDef hspi;
	DMA_HandleTypeDef tx, rx;
	uint16_t txBuf[16];
	uint16_t rxBuf[16];

	/* oneLine: SPI_DIRECTION_1LINE. byteAlign: DMA data alignment is byte even with 16 bit frames. circular: Tx stream circular. slave: SPI_MODE_SLAVE. */
	SpiDmaRig(unsigned idx, bool is16, bool oneLine = false, bool byteAlign = false, bool circularTx = false, bool slave = false)
	    : n(idx), b16(is16), m(spiDmaMapOf(idx)), inst(spiDmaInstance(idx)) {
		std::memset(&hspi, 0, sizeof hspi);
		std::memset(txBuf, 0, sizeof txBuf);
		std::memset(rxBuf, 0, sizeof rxBuf);
		hspi.Instance = inst;
		hspi.Init.Mode = slave ? SPI_MODE_SLAVE : SPI_MODE_MASTER;
		hspi.Init.Direction = oneLine ? SPI_DIRECTION_1LINE : SPI_DIRECTION_2LINES;
		hspi.Init.DataSize = b16 ? SPI_DATASIZE_16BIT : SPI_DATASIZE_8BIT;
		hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
		hspi.Init.NSS = SPI_NSS_SOFT;
		hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
		hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
		hspi.Init.TIMode = SPI_TIMODE_DISABLE;
		hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		hspi.Init.CRCPolynomial = 7;
		REQUIRE(HAL_SPI_Init(&hspi) == HAL_OK);
		makeDma(tx, m.txI, DMA_MEMORY_TO_PERIPH, byteAlign, circularTx);
		makeDma(rx, m.rxI, DMA_PERIPH_TO_MEMORY, byteAlign, false);
		hspi.hdmatx = &tx;
		tx.Parent = &hspi;
		hspi.hdmarx = &rx;
		rx.Parent = &hspi;
		spiSupportReset();
	}

	void makeDma(DMA_HandleTypeDef &h, DMA_Stream_TypeDef *s, uint32_t dir, bool byteAlign, bool circular) {
		std::memset(&h, 0, sizeof h);
		h.Instance = s;
		h.Init.Channel = m.channel;
		h.Init.Direction = dir;
		h.Init.PeriphInc = DMA_PINC_DISABLE;
		h.Init.MemInc = DMA_MINC_ENABLE;
		h.Init.PeriphDataAlignment = (b16 && !byteAlign) ? DMA_PDATAALIGN_HALFWORD : DMA_PDATAALIGN_BYTE;
		h.Init.MemDataAlignment = (b16 && !byteAlign) ? DMA_MDATAALIGN_HALFWORD : DMA_MDATAALIGN_BYTE;
		h.Init.Mode = circular ? DMA_CIRCULAR : DMA_NORMAL;
		h.Init.Priority = DMA_PRIORITY_LOW;
		h.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		h.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		h.Init.MemBurst = DMA_MBURST_SINGLE;
		h.Init.PeriphBurst = DMA_PBURST_SINGLE;
		REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	}

	uint16_t word(unsigned i) const { return b16 ? static_cast<uint16_t>(0x1234u + 0x1111u * i) : static_cast<uint16_t>(0x21u + 3u * i); }
	void fillTx(unsigned count) {
		for (unsigned i = 0; i < count; ++i) {
			if (b16)
				txBuf[i] = word(i);
			else
				reinterpret_cast<uint8_t *>(txBuf)[i] = static_cast<uint8_t>(word(i));
		}
	}
	uint16_t rxWord(unsigned i) const { return b16 ? rxBuf[i] : reinterpret_cast<const uint8_t *>(rxBuf)[i]; }
	uint8_t *txPtr() { return reinterpret_cast<uint8_t *>(txBuf); }
	uint8_t *rxPtr() { return reinterpret_cast<uint8_t *>(rxBuf); }
	uint32_t txAddr() const { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(txBuf)); }
	uint32_t rxAddr() const { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(rxBuf)); }

	/* The tests stand in for the DMA engine: Tx requests (TXE) and Rx requests (RXNE). */
	void waitFlag(uint32_t flag) {
		for (unsigned g = 0; g < 1000; ++g) {
			if (inst->SR & flag) return;
			HAL_GetTick();
		}
		FAIL("SPI flag never set");
	}
	void engineWrite(unsigned count) { // Tx stream only: SPI_DR written while TXE = 1
		for (unsigned i = 0; i < count; ++i) {
			waitFlag(spiDmaSR_TXE);
			inst->DR = word(i);
		}
	}
	void engineTxRx(unsigned count) { // full duplex: write when TXE, read when RXNE
		for (unsigned i = 0; i < count; ++i) {
			waitFlag(spiDmaSR_TXE);
			inst->DR = word(i);
			waitFlag(spiDmaSR_RXNE);
			const uint32_t r = inst->DR;
			if (b16)
				rxBuf[i] = static_cast<uint16_t>(r);
			else
				reinterpret_cast<uint8_t *>(rxBuf)[i] = static_cast<uint8_t>(r);
		}
	}
	void completeTx() {
		hostDmaComplete(m.c, m.txS);
		HAL_DMA_IRQHandler(&tx);
	}
	void completeRx() {
		hostDmaComplete(m.c, m.rxS);
		HAL_DMA_IRQHandler(&rx);
	}
	void completeBoth() {
		hostDmaComplete(m.c, m.txS);
		hostDmaComplete(m.c, m.rxS);
		HAL_DMA_IRQHandler(&tx);
		HAL_DMA_IRQHandler(&rx);
	}
	bool streamEnabled(const DMA_HandleTypeDef &h) const { return (h.Instance->CR & spiDmaDMA_EN) != 0u; }
};

class SpiDmaFx : public SpiFixture {};

} // namespace

/* ------------------------------------------------------------------------------------------------------------------ SD1 */

TEST_CASE_FIXTURE(SpiDmaFx, "SD1 Transmit_DMA programs the Tx stream with the data register, the buffer, direction, channel and Size items") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(6);
			REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 6) == HAL_OK);
			const uint32_t cr = r.tx.Instance->CR;
			CHECK((cr & spiDmaDMA_EN) != 0u);
			CHECK(((cr & spiDmaDMA_DIR_MASK) >> spiDmaDMA_DIR_SHIFT) == 1u); // 10.5.5: 01 memory to peripheral
			CHECK(((cr & spiDmaDMA_CHSEL_MASK) >> spiDmaDMA_CHSEL_SHIFT) == (r.m.channel >> spiDmaDMA_CHSEL_SHIFT)); // the channel the test set
			CHECK(r.tx.Instance->PAR == spiDmaDrAddress[k.n]);  // Table 1 + 0x0C
			CHECK(r.tx.Instance->M0AR == r.txAddr());
			CHECK(r.tx.Instance->NDTR == 6u);  // 10.5.6: NDT counts items: 6 frames of 8 or 16 bit (PSIZE = frame size)
			CHECK((cr & spiDmaDMA_TCIE) != 0u);
			CHECK((cr & spiDmaDMA_TEIE) != 0u);
			CHECK((cr & spiDmaDMA_HTIE) != 0u); // the half transfer callback exists (UM1725: HAL_SPI_TxHalfCpltCallback)
			CHECK(hostDmaProtectedWrites(r.m.c, r.m.txS) == 0u);
			CHECK(!r.streamEnabled(r.rx));
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD1 Transmit_DMA sets TXDMAEN, ERRIE and SPE, state BUSY_TX and unlocks the handle") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(4);
			REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 4) == HAL_OK);
			CHECK((r.inst->CR2 & spiDmaCR2_TXDMAEN) != 0u); // 28.5.2
			CHECK((r.inst->CR2 & spiDmaCR2_ERRIE) != 0u);
			CHECK((r.inst->CR2 & spiDmaCR2_RXDMAEN) == 0u);
			CHECK((r.inst->CR2 & (spiDmaCR2_TXEIE | spiDmaCR2_RXNEIE)) == 0u); // DMA mode, no TXE / RXNE interrupt
			CHECK((r.inst->CR1 & spiDmaCR1_SPE) != 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_BUSY_TX);
			CHECK(HAL_SPI_GetError(&r.hspi) == HAL_SPI_ERROR_NONE);
			CHECK(r.hspi.Lock == HAL_UNLOCKED);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD1 Receive_DMA of a 2 line master starts the Rx stream and, for the clock, the Tx stream; state BUSY_RX") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			REQUIRE(HAL_SPI_Receive_DMA(&r.hspi, r.rxPtr(), 5) == HAL_OK);
			const uint32_t crR = r.rx.Instance->CR;
			CHECK((crR & spiDmaDMA_EN) != 0u);
			CHECK(((crR & spiDmaDMA_DIR_MASK) >> spiDmaDMA_DIR_SHIFT) == 0u); // 00 peripheral to memory
			CHECK(((crR & spiDmaDMA_CHSEL_MASK) >> spiDmaDMA_CHSEL_SHIFT) == (r.m.channel >> spiDmaDMA_CHSEL_SHIFT));
			CHECK(r.rx.Instance->PAR == spiDmaDrAddress[k.n]);
			CHECK(r.rx.Instance->M0AR == r.rxAddr());
			CHECK(r.rx.Instance->NDTR == 5u);
			// UM1725: "In case of MASTER mode and SPI_DIRECTION_2LINES direction, hdmatx shall be defined": the Tx stream runs too.
			CHECK(r.streamEnabled(r.tx));
			CHECK(r.tx.Instance->NDTR == 5u);
			CHECK((r.inst->CR2 & spiDmaCR2_RXDMAEN) != 0u);
			CHECK((r.inst->CR2 & spiDmaCR2_TXDMAEN) != 0u);
			CHECK((r.inst->CR2 & spiDmaCR2_ERRIE) != 0u);
			CHECK((r.inst->CR1 & spiDmaCR1_SPE) != 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_BUSY_RX);
			CHECK(r.hspi.Lock == HAL_UNLOCKED);
			CHECK(hostDmaProtectedWrites(r.m.c, r.m.rxS) == 0u);
			CHECK(hostDmaProtectedWrites(r.m.c, r.m.txS) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD1 TransmitReceive_DMA programs both streams, sets both DMA enables and the state BUSY_TX_RX") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(7);
			REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 7) == HAL_OK);
			CHECK(((r.tx.Instance->CR & spiDmaDMA_DIR_MASK) >> spiDmaDMA_DIR_SHIFT) == 1u);
			CHECK(((r.rx.Instance->CR & spiDmaDMA_DIR_MASK) >> spiDmaDMA_DIR_SHIFT) == 0u);
			CHECK(r.streamEnabled(r.tx));
			CHECK(r.streamEnabled(r.rx));
			CHECK(r.tx.Instance->PAR == spiDmaDrAddress[k.n]);
			CHECK(r.rx.Instance->PAR == spiDmaDrAddress[k.n]);
			CHECK(r.tx.Instance->M0AR == r.txAddr());
			CHECK(r.rx.Instance->M0AR == r.rxAddr());
			CHECK(r.tx.Instance->NDTR == 7u);
			CHECK(r.rx.Instance->NDTR == 7u);
			CHECK((r.inst->CR2 & spiDmaCR2_TXDMAEN) != 0u);
			CHECK((r.inst->CR2 & spiDmaCR2_RXDMAEN) != 0u);
			CHECK((r.inst->CR2 & spiDmaCR2_ERRIE) != 0u);
			CHECK((r.inst->CR1 & spiDmaCR1_SPE) != 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_BUSY_TX_RX);
			CHECK(r.hspi.Lock == HAL_UNLOCKED);
			CHECK(hostDmaProtectedWrites(r.m.c, r.m.txS) == 0u);
			CHECK(hostDmaProtectedWrites(r.m.c, r.m.rxS) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD1 Receive_DMA of a 1 line master starts only the Rx stream with BIDIOE clear and only RXDMAEN") {
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE((std::string("SPI") + std::to_string(n + 1)).c_str()) {
			SpiDmaRig r(n, false, true);
			REQUIRE(HAL_SPI_Receive_DMA(&r.hspi, r.rxPtr(), 4) == HAL_OK);
			CHECK(r.streamEnabled(r.rx));
			CHECK(!r.streamEnabled(r.tx));
			CHECK(r.rx.Instance->PAR == spiDmaDrAddress[n]);
			CHECK(r.rx.Instance->NDTR == 4u);
			CHECK((r.inst->CR2 & spiDmaCR2_RXDMAEN) != 0u);
			CHECK((r.inst->CR2 & spiDmaCR2_TXDMAEN) == 0u);
			CHECK((r.inst->CR1 & spiDmaCR1_BIDIOE) == 0u); // 28.5.1: BIDIOE 0 = output disabled (receive only)
			CHECK((r.inst->CR1 & spiDmaCR1_SPE) != 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_BUSY_RX);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD1 Transmit_DMA of a 1 line master sets BIDIOE and starts only the Tx stream") {
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE((std::string("SPI") + std::to_string(n + 1)).c_str()) {
			SpiDmaRig r(n, false, true);
			r.fillTx(3);
			REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 3) == HAL_OK);
			CHECK((r.inst->CR1 & spiDmaCR1_BIDIOE) != 0u); // 28.5.1: BIDIOE 1 = output enabled (transmit only)
			CHECK((r.inst->CR1 & spiDmaCR1_SPE) != 0u);
			CHECK(r.streamEnabled(r.tx));
			CHECK(!r.streamEnabled(r.rx));
			CHECK((r.inst->CR2 & spiDmaCR2_TXDMAEN) != 0u);
			CHECK((r.inst->CR2 & spiDmaCR2_RXDMAEN) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD1 characterization: Transmit_DMA while a transfer runs returns HAL_BUSY and does not touch the stream") {
	// NEEDS REVIEW N1: UM1725 only says "HAL status".
	SpiDmaRig r(0, false);
	r.fillTx(4);
	REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 4) == HAL_OK);
	CHECK(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 2) == HAL_BUSY);
	CHECK(r.tx.Instance->NDTR == 4u);
	CHECK(hostDmaProtectedWrites(r.m.c, r.m.txS) == 0u);
	CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_BUSY_TX);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD1 characterization: Transmit_DMA with a NULL buffer or Size 0 returns HAL_ERROR and writes nothing") {
	// NEEDS REVIEW N1.
	SpiDmaRig r(1, false);
	CHECK(HAL_SPI_Transmit_DMA(&r.hspi, nullptr, 4) == HAL_ERROR);
	CHECK(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 0) == HAL_ERROR);
	CHECK(!r.streamEnabled(r.tx));
	CHECK(r.inst->CR2 == 0u);
	CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
	CHECK(r.hspi.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD1 characterization: TransmitReceive_DMA with a NULL buffer or Size 0 returns HAL_ERROR and writes nothing") {
	// NEEDS REVIEW N1.
	SpiDmaRig r(1, false);
	CHECK(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), nullptr, 4) == HAL_ERROR);
	CHECK(HAL_SPI_TransmitReceive_DMA(&r.hspi, nullptr, r.rxPtr(), 4) == HAL_ERROR);
	CHECK(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 0) == HAL_ERROR);
	CHECK(!r.streamEnabled(r.tx));
	CHECK(!r.streamEnabled(r.rx));
	CHECK(r.inst->CR2 == 0u);
	CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
	CHECK(r.hspi.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD1 characterization: Receive_DMA with a NULL buffer returns HAL_ERROR and writes nothing") {
	// NEEDS REVIEW N1.
	SpiDmaRig r(1, false);
	CHECK(HAL_SPI_Receive_DMA(&r.hspi, nullptr, 4) == HAL_ERROR);
	CHECK(!r.streamEnabled(r.tx));
	CHECK(!r.streamEnabled(r.rx));
	CHECK(r.inst->CR2 == 0u);
	CHECK(r.hspi.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD1 Receive_DMA with a NULL buffer leaves the state READY" * KNOWN_ISSUE("candidate vendor bug V97: Receive_DMA of a 2 line master sets BUSY_RX before TransmitReceive_DMA rejects the NULL buffer, and the state is not restored: every later call returns HAL_BUSY (UM1725: a rejected call starts nothing)")) {
	SpiDmaRig r(1, false);
	CHECK(HAL_SPI_Receive_DMA(&r.hspi, nullptr, 4) == HAL_ERROR);
	CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD1 Transmit_DMA with a Tx stream that cannot start returns HAL_ERROR with the DMA error code and leaves the state READY" * KNOWN_ISSUE("candidate vendor bug V101 (arguable: UM1725 gives no state rule for a failed DMA start; the later calls return HAL_BUSY): the state stays BUSY_TX after the HAL_ERROR return")) {
	SpiDmaRig r(2, false);
	r.fillTx(4);
	// The stream is already busy (started directly): HAL_DMA_Start_IT refuses with HAL_BUSY.
	REQUIRE(HAL_DMA_Start_IT(&r.tx, r.txAddr(), spiDmaDrAddress[2], 2) == HAL_OK);
	CHECK(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 4) == HAL_ERROR);
	CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD1 characterization: Transmit_DMA with a Tx stream that cannot start sets the DMA error code, unlocks and keeps the running stream") {
	// NEEDS REVIEW N2 (the state is the KNOWN_ISSUE V101 case above).
	SpiDmaRig r(2, false);
	r.fillTx(4);
	REQUIRE(HAL_DMA_Start_IT(&r.tx, r.txAddr(), spiDmaDrAddress[2], 2) == HAL_OK);
	CHECK(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 4) == HAL_ERROR);
	CHECK((HAL_SPI_GetError(&r.hspi) & HAL_SPI_ERROR_DMA) != 0u);
	CHECK(r.hspi.Lock == HAL_UNLOCKED);
	CHECK(r.tx.Instance->NDTR == 2u);
}

/* ------------------------------------------------------------------------------------------------------------------ SD2 */

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 Tx complete runs the TxCplt callback once with the state READY and the Tx DMA enable cleared") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(3);
			REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 3) == HAL_OK);
			r.engineWrite(3);
			r.completeTx();
			CHECK(spiCounts.txCplt == 1u);
			CHECK(spiCounts.last == &r.hspi);
			CHECK(spiCounts.stateAtLast == HAL_SPI_STATE_READY); // set before the user callback
			CHECK(spiCounts.error == 0u);
			CHECK(spiCounts.rxCplt == 0u);
			CHECK(spiCounts.txRxCplt == 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK(HAL_SPI_GetError(&r.hspi) == HAL_SPI_ERROR_NONE);
			CHECK((r.inst->CR2 & spiDmaCR2_TXDMAEN) == 0u);
			CHECK(r.hspi.Lock == HAL_UNLOCKED);
			CHECK(hostDmaProtectedWrites(r.m.c, r.m.txS) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 Tx complete leaves the bus idle (TXE = 1, BSY = 0) and the frames on the wire are the buffer") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(3);
			REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 3) == HAL_OK);
			r.engineWrite(3);
			r.completeTx();
			const uint32_t sr = r.inst->SR;
			CHECK((sr & spiDmaSR_TXE) != 0u);
			CHECK((sr & spiDmaSR_BSY) == 0u); // 28.3.9: BSY monitored after TCIF
			REQUIRE(hostSpiTxCount(k.n) == 3u);
			for (unsigned i = 0; i < 3; ++i) CHECK(hostSpiTxAt(k.n, i) == r.word(i));
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 Tx complete of a 2 line master clears the OVR that the unread received frames caused") {
	// 28.3.9: with only the Tx stream OVR is set because the received data is not read; UM1725: the HAL clears it at the end.
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(3);
			REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 3) == HAL_OK);
			r.engineWrite(3);
			r.waitFlag(spiDmaSR_TXE);
			HAL_GetTick();
			REQUIRE((r.inst->SR & spiDmaSR_OVR) != 0u); // precondition from 28.3.9 / 28.3.10 (model)
			r.completeTx();
			CHECK((r.inst->SR & spiDmaSR_OVR) == 0u);
			CHECK(spiCounts.txCplt == 1u);
			CHECK(spiCounts.error == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 Tx complete waits for the last frame to end (BSY = 0) before the callback") {
	hostSpiSetFrameTicks(4);
	SpiDmaRig r(0, false);
	r.fillTx(1);
	REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 1) == HAL_OK);
	r.engineWrite(1);
	REQUIRE((r.inst->SR & spiDmaSR_BSY) != 0u);
	const uint32_t t0 = hostTickPeek();
	r.completeTx();
	const uint32_t elapsed = hostTickPeek() - t0;
	CHECK(elapsed >= 2u);   // the frame still had at least 2 of its 4 ticks to run
	CHECK(elapsed < 100u);  // far below the HAL's 100 tick timeout
	CHECK((r.inst->SR & spiDmaSR_BSY) == 0u);
	CHECK(spiCounts.txCplt == 1u);
	CHECK(spiCounts.error == 0u);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 characterization: Tx complete on a bus that never ends the frame times out after about 100 ticks with the FLAG error") {
	// NEEDS REVIEW N3.
	hostSpiSetFrameTicks(4);
	SpiDmaRig r(1, false);
	r.fillTx(1);
	REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 1) == HAL_OK);
	r.engineWrite(1);
	hostSpiPeerSilent(1, 1);
	const uint32_t t0 = hostTickPeek();
	r.completeTx();
	const uint32_t elapsed = hostTickPeek() - t0;
	CHECK(elapsed >= 90u);
	CHECK(elapsed <= 120u);
	CHECK(spiCounts.error == 1u);
	CHECK((spiCounts.errorCodeAtError & HAL_SPI_ERROR_FLAG) != 0u);
	CHECK(spiCounts.txCplt == 0u);
	CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 Rx complete of a 2 line master runs the RxCplt callback once, reads the peer words and clears both DMA enables") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			for (unsigned i = 0; i < 3; ++i) hostSpiPeerPush(k.n, r.word(i));
			REQUIRE(HAL_SPI_Receive_DMA(&r.hspi, r.rxPtr(), 3) == HAL_OK);
			r.engineTxRx(3);
			r.completeBoth();
			CHECK(spiCounts.rxCplt == 1u);
			CHECK(spiCounts.txRxCplt == 0u);
			CHECK(spiCounts.txCplt == 0u);
			CHECK(spiCounts.error == 0u);
			CHECK(spiCounts.stateAtLast == HAL_SPI_STATE_READY);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK((r.inst->CR2 & (spiDmaCR2_RXDMAEN | spiDmaCR2_TXDMAEN)) == 0u);
			CHECK((r.inst->SR & spiDmaSR_BSY) == 0u);
			for (unsigned i = 0; i < 3; ++i) CHECK(r.rxWord(i) == r.word(i));
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 TxRx complete runs the TxRxCplt callback once, with the tx frames on the wire and the peer words received") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(4);
			for (unsigned i = 0; i < 4; ++i) hostSpiPeerPush(k.n, static_cast<uint16_t>(r.word(i) ^ 0x0055u));
			REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 4) == HAL_OK);
			r.engineTxRx(4);
			r.completeBoth();
			CHECK(spiCounts.txRxCplt == 1u);
			CHECK(spiCounts.rxCplt == 0u);
			CHECK(spiCounts.txCplt == 0u);
			CHECK(spiCounts.error == 0u);
			CHECK(spiCounts.stateAtLast == HAL_SPI_STATE_READY);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK((r.inst->CR2 & (spiDmaCR2_RXDMAEN | spiDmaCR2_TXDMAEN)) == 0u);
			CHECK((r.inst->SR & spiDmaSR_BSY) == 0u);
			CHECK(r.hspi.Lock == HAL_UNLOCKED);
			for (unsigned i = 0; i < 4; ++i) {
				CHECK(hostSpiTxAt(k.n, i) == r.word(i));
				CHECK(r.rxWord(i) == (r.word(i) ^ 0x0055u));
			}
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 Rx complete of a 1 line master runs the RxCplt callback once, with the state READY and the Rx DMA enable cleared") {
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE((std::string("SPI") + std::to_string(n + 1)).c_str()) {
			SpiDmaRig r(n, false, true);
			REQUIRE(HAL_SPI_Receive_DMA(&r.hspi, r.rxPtr(), 2) == HAL_OK);
			r.completeRx();
			CHECK(spiCounts.rxCplt == 1u);
			CHECK(spiCounts.error == 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK((r.inst->CR2 & spiDmaCR2_RXDMAEN) == 0u);
			CHECK((r.inst->CR1 & spiDmaCR1_SPE) == 0u); // the receive only clock is stopped (28.3.5, 28.3.8)
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 characterization: completion of a Tx, Rx or TxRx DMA transfer clears ERRIE") {
	// NEEDS REVIEW N8: UM1725 does not say which CR2 bits the complete callback clears.
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			{
				SpiDmaRig r(k.n, k.b16);
				r.fillTx(2);
				REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 2) == HAL_OK);
				r.engineWrite(2);
				r.completeTx();
				CHECK((r.inst->CR2 & (spiDmaCR2_TXDMAEN | spiDmaCR2_RXDMAEN | spiDmaCR2_ERRIE)) == 0u);
			}
			{
				SpiDmaRig r(k.n, k.b16);
				REQUIRE(HAL_SPI_Receive_DMA(&r.hspi, r.rxPtr(), 2) == HAL_OK);
				r.engineTxRx(2);
				r.completeBoth();
				CHECK((r.inst->CR2 & (spiDmaCR2_TXDMAEN | spiDmaCR2_RXDMAEN | spiDmaCR2_ERRIE)) == 0u);
			}
			{
				SpiDmaRig r(k.n, k.b16);
				r.fillTx(2);
				REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 2) == HAL_OK);
				r.engineTxRx(2);
				r.completeBoth();
				CHECK((r.inst->CR2 & (spiDmaCR2_TXDMAEN | spiDmaCR2_RXDMAEN | spiDmaCR2_ERRIE)) == 0u);
			}
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 the Tx half transfer runs the TxHalfCplt callback once and the transfer stays busy") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(8);
			REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 8) == HAL_OK);
			hostDmaStepItems(r.m.c, r.m.txS, 4); // HTIF at half (hostDmaStepItems)
			HAL_DMA_IRQHandler(&r.tx);
			CHECK(spiCounts.txHalf == 1u);
			CHECK(spiCounts.txCplt == 0u);
			CHECK(spiCounts.rxHalf == 0u);
			CHECK(spiCounts.txRxHalf == 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_BUSY_TX);
			CHECK((r.inst->CR2 & spiDmaCR2_TXDMAEN) != 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 the Rx half transfer of a receive runs the RxHalfCplt callback once and the transfer stays busy") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			REQUIRE(HAL_SPI_Receive_DMA(&r.hspi, r.rxPtr(), 8) == HAL_OK);
			hostDmaStepItems(r.m.c, r.m.rxS, 4);
			HAL_DMA_IRQHandler(&r.rx);
			CHECK(spiCounts.rxHalf == 1u);
			CHECK(spiCounts.rxCplt == 0u);
			CHECK(spiCounts.txHalf == 0u);
			CHECK(spiCounts.txRxHalf == 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_BUSY_RX);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 the half transfer of a TransmitReceive runs the TxRxHalfCplt callback once and the transfer stays busy") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(8);
			REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 8) == HAL_OK);
			hostDmaStepItems(r.m.c, r.m.txS, 4);
			hostDmaStepItems(r.m.c, r.m.rxS, 4);
			HAL_DMA_IRQHandler(&r.tx);
			HAL_DMA_IRQHandler(&r.rx);
			CHECK(spiCounts.txRxHalf == 1u);
			CHECK(spiCounts.txHalf == 0u);
			CHECK(spiCounts.rxHalf == 0u);
			CHECK(spiCounts.txRxCplt == 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_BUSY_TX_RX);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 the DMA interrupt handler without a pending flag runs no SPI callback") {
	SpiDmaRig r(0, false);
	r.fillTx(4);
	REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 4) == HAL_OK);
	HAL_DMA_IRQHandler(&r.tx);
	CHECK(spiCounts.txCplt == 0u);
	CHECK(spiCounts.txHalf == 0u);
	CHECK(spiCounts.error == 0u);
	CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_BUSY_TX);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 characterization: a circular Tx stream calls TxCplt on every TCIF and keeps the transfer busy") {
	// NEEDS REVIEW N4: UM1725 gives a circular mode restriction list but no complete-callback rule.
	SpiDmaRig r(1, false, false, false, true);
	r.fillTx(4);
	REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 4) == HAL_OK);
	r.completeTx();
	r.completeTx();
	CHECK(spiCounts.txCplt == 2u);
	CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_BUSY_TX);
	CHECK((r.inst->CR2 & spiDmaCR2_TXDMAEN) != 0u);
	CHECK(r.streamEnabled(r.tx));
}

/* Receive_DMA of a 1 line master and of a slave (the paths that do not delegate to TransmitReceive_DMA). */

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 Receive_DMA of a 1 line master clears BIDIOE in the write that enables SPE") {
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE((std::string("SPI") + std::to_string(n + 1)).c_str()) {
			SpiDmaRig r(n, false, true);
			r.inst->CR1 = r.inst->CR1 | spiDmaCR1_BIDIOE; // BIDIOE left set by an earlier transmit, SPE still 0
			const std::size_t first = hostRegsLogCount();
			REQUIRE(HAL_SPI_Receive_DMA(&r.hspi, r.rxPtr(), 3) == HAL_OK);
			bool found = false;
			for (std::size_t i = first; i < hostRegsLogCount(); ++i) {
				const HostRegAccess *e = hostRegsLogAt(i);
				if (e->write && e->addr == reinterpret_cast<std::uintptr_t>(&r.inst->CR1) && (e->written & spiDmaCR1_SPE) != 0u) {
					CHECK((e->written & spiDmaCR1_BIDIOE) == 0u); // 28.5.1: BIDIOE 0 = receive only, set before the clock starts
					found = true;
					break;
				}
			}
			CHECK(found);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD3 an Rx stream error of a 1 line master Receive_DMA sets the DMA error code and runs ErrorCallback once") {
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE((std::string("SPI") + std::to_string(n + 1)).c_str()) {
			SpiDmaRig r(n, false, true);
			REQUIRE(HAL_SPI_Receive_DMA(&r.hspi, r.rxPtr(), 4) == HAL_OK);
			hostDmaRaise(r.m.c, r.m.rxS, HOST_DMA_TEIF);
			HAL_DMA_IRQHandler(&r.rx);
			CHECK(spiCounts.error == 1u);
			CHECK((spiCounts.errorCodeAtError & HAL_SPI_ERROR_DMA) != 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK(spiCounts.rxCplt == 0u);
			CHECK((r.inst->CR2 & spiDmaCR2_RXDMAEN) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD1 Receive_DMA of a slave starts only the Rx stream with only RXDMAEN, ERRIE and SPE set") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16, false, false, false, true);
			REQUIRE(HAL_SPI_Receive_DMA(&r.hspi, r.rxPtr(), 4) == HAL_OK);
			CHECK(r.streamEnabled(r.rx));
			CHECK(!r.streamEnabled(r.tx)); // a slave needs no clock from the Tx stream
			CHECK(r.rx.Instance->PAR == spiDmaDrAddress[k.n]);
			CHECK(r.rx.Instance->M0AR == r.rxAddr());
			CHECK(r.rx.Instance->NDTR == 4u);
			CHECK((r.inst->CR2 & spiDmaCR2_RXDMAEN) != 0u);
			CHECK((r.inst->CR2 & spiDmaCR2_TXDMAEN) == 0u);
			CHECK((r.inst->CR2 & spiDmaCR2_ERRIE) != 0u);
			CHECK((r.inst->CR1 & spiDmaCR1_SPE) != 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_BUSY_RX);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD2 Rx complete of a slave Receive_DMA runs RxCplt once and the received frames are the master's words") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16, false, false, false, true);
			REQUIRE(HAL_SPI_Receive_DMA(&r.hspi, r.rxPtr(), 3) == HAL_OK);
			for (unsigned i = 0; i < 3; ++i) {
				hostSpiSlaveFrame(k.n, r.word(i)); // the master clocks one frame
				r.waitFlag(spiDmaSR_RXNE);
				const uint32_t w = r.inst->DR; // the DMA read request
				if (k.b16)
					r.rxBuf[i] = static_cast<uint16_t>(w);
				else
					reinterpret_cast<uint8_t *>(r.rxBuf)[i] = static_cast<uint8_t>(w);
			}
			r.completeRx();
			CHECK(spiCounts.rxCplt == 1u);
			CHECK(spiCounts.error == 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK((r.inst->CR2 & spiDmaCR2_RXDMAEN) == 0u);
			CHECK((r.inst->SR & spiDmaSR_OVR) == 0u);
			for (unsigned i = 0; i < 3; ++i) CHECK(r.rxWord(i) == r.word(i));
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD3 an Rx stream error of a slave Receive_DMA sets the DMA error code and runs ErrorCallback once") {
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE((std::string("SPI") + std::to_string(n + 1)).c_str()) {
			SpiDmaRig r(n, false, false, false, false, true);
			REQUIRE(HAL_SPI_Receive_DMA(&r.hspi, r.rxPtr(), 4) == HAL_OK);
			hostDmaRaise(r.m.c, r.m.rxS, HOST_DMA_TEIF);
			HAL_DMA_IRQHandler(&r.rx);
			CHECK(spiCounts.error == 1u);
			CHECK((spiCounts.errorCodeAtError & HAL_SPI_ERROR_DMA) != 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK(spiCounts.rxCplt == 0u);
		}
	}
}

/* ------------------------------------------------------------------------------------------------------------------ SD3 */

TEST_CASE_FIXTURE(SpiDmaFx, "SD3 a Tx stream error of Transmit_DMA sets the DMA error code and runs ErrorCallback once with the state READY") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(4);
			REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 4) == HAL_OK);
			hostDmaRaise(r.m.c, r.m.txS, HOST_DMA_TEIF);
			HAL_DMA_IRQHandler(&r.tx);
			CHECK(spiCounts.error == 1u);
			CHECK((spiCounts.errorCodeAtError & HAL_SPI_ERROR_DMA) != 0u);
			CHECK((HAL_SPI_GetError(&r.hspi) & HAL_SPI_ERROR_DMA) != 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK(spiCounts.txCplt == 0u);
			CHECK((r.inst->CR2 & (spiDmaCR2_TXDMAEN | spiDmaCR2_RXDMAEN)) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD3 an Rx stream error of Receive_DMA sets the DMA error code and runs ErrorCallback once with the state READY") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			REQUIRE(HAL_SPI_Receive_DMA(&r.hspi, r.rxPtr(), 4) == HAL_OK);
			hostDmaRaise(r.m.c, r.m.rxS, HOST_DMA_TEIF);
			HAL_DMA_IRQHandler(&r.rx);
			CHECK(spiCounts.error == 1u);
			CHECK((spiCounts.errorCodeAtError & HAL_SPI_ERROR_DMA) != 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK(spiCounts.rxCplt == 0u);
			CHECK((r.inst->CR2 & (spiDmaCR2_TXDMAEN | spiDmaCR2_RXDMAEN)) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD3 an Rx stream error of TransmitReceive_DMA sets the DMA error code and runs ErrorCallback once") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(4);
			REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 4) == HAL_OK);
			hostDmaRaise(r.m.c, r.m.rxS, HOST_DMA_TEIF);
			HAL_DMA_IRQHandler(&r.rx);
			CHECK(spiCounts.error == 1u);
			CHECK((spiCounts.errorCodeAtError & HAL_SPI_ERROR_DMA) != 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK(spiCounts.txRxCplt == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD3 a Tx stream error of TransmitReceive_DMA sets the DMA error code and runs ErrorCallback" * KNOWN_ISSUE("candidate vendor bug V98: TransmitReceive_DMA sets no error callback on the Tx stream, so a Tx stream error (the stream stops, EN cleared) is not reported and the transfer never ends")) {
	// UM1725 error names: HAL_SPI_ERROR_DMA "DMA transfer error". The Tx stream stopped with TEIF: the SPI must report it.
	SpiDmaRig r(0, false);
	r.fillTx(4);
	REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 4) == HAL_OK);
	hostDmaRaise(r.m.c, r.m.txS, HOST_DMA_TEIF);
	HAL_DMA_IRQHandler(&r.tx);
	CHECK(spiCounts.error == 1u);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD3 an OVR during Transmit_DMA is cleared by the interrupt handler without an error (28.3.9)") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(4);
			REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 4) == HAL_OK);
			r.engineWrite(3);
			r.waitFlag(spiDmaSR_TXE);
			HAL_GetTick();
			REQUIRE((r.inst->SR & spiDmaSR_OVR) != 0u); // 28.3.9: OVR is set when only the Tx stream runs
			HAL_SPI_IRQHandler(&r.hspi);
			CHECK((r.inst->SR & spiDmaSR_OVR) == 0u);
			CHECK(spiCounts.error == 0u);
			CHECK(HAL_SPI_GetError(&r.hspi) == HAL_SPI_ERROR_NONE);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_BUSY_TX);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD3 an OVR during TransmitReceive_DMA sets HAL_SPI_ERROR_OVR, clears the flag and aborts both streams") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(4);
			REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 4) == HAL_OK);
			r.engineWrite(3); // the Rx data is not read: the third frame ends with RXNE set
			r.waitFlag(spiDmaSR_TXE);
			HAL_GetTick();
			REQUIRE((r.inst->SR & spiDmaSR_OVR) != 0u);
			HAL_SPI_IRQHandler(&r.hspi);
			CHECK((HAL_SPI_GetError(&r.hspi) & HAL_SPI_ERROR_OVR) != 0u);
			CHECK((r.inst->CR2 & (spiDmaCR2_TXDMAEN | spiDmaCR2_RXDMAEN | spiDmaCR2_ERRIE)) == 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK(!r.streamEnabled(r.tx));
			CHECK(!r.streamEnabled(r.rx));
			CHECK(hostDmaProtectedWrites(r.m.c, r.m.txS) == 0u);
			CHECK(hostDmaProtectedWrites(r.m.c, r.m.rxS) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD3 an OVR during TransmitReceive_DMA runs ErrorCallback once once the streams are aborted" * KNOWN_ISSUE("candidate vendor bug V99: both DMA abort callbacks (SPI_DMAAbortOnError of the Tx and of the Rx stream) call HAL_SPI_ErrorCallback, so the user sees the error twice")) {
	SpiDmaRig r(1, false);
	r.fillTx(4);
	REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 4) == HAL_OK);
	r.engineWrite(3);
	r.waitFlag(spiDmaSR_TXE);
	HAL_GetTick();
	REQUIRE((r.inst->SR & spiDmaSR_OVR) != 0u);
	HAL_SPI_IRQHandler(&r.hspi);
	HAL_DMA_IRQHandler(&r.rx); // abort completion of the Rx stream
	HAL_DMA_IRQHandler(&r.tx); // abort completion of the Tx stream
	CHECK(spiCounts.error == 1u);
}

/* ------------------------------------------------------------------------------------------------------------------ SD4 */

TEST_CASE_FIXTURE(SpiDmaFx, "SD4 Abort with no transfer returns HAL_OK, leaves the state READY and runs no callback") {
	SpiDmaRig r(0, false);
	CHECK(HAL_SPI_Abort(&r.hspi) == HAL_OK);
	CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
	CHECK(HAL_SPI_GetError(&r.hspi) == HAL_SPI_ERROR_NONE);
	CHECK(spiCounts.abortCplt == 0u); // UM1725: only the _IT variant calls the callback
	CHECK(spiCounts.error == 0u);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD4 Abort_IT with no transfer returns HAL_OK and runs AbortCplt once at once") {
	SpiDmaRig r(2, false);
	CHECK(HAL_SPI_Abort_IT(&r.hspi) == HAL_OK);
	CHECK(spiCounts.abortCplt == 1u);
	CHECK(spiCounts.stateAtLast == HAL_SPI_STATE_READY);
	CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD4 Abort of a Tx DMA transfer stops the Tx stream, clears TXDMAEN and ERRIE and sets the state READY") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(6);
			REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 6) == HAL_OK);
			CHECK(HAL_SPI_Abort(&r.hspi) == HAL_OK);
			CHECK(!r.streamEnabled(r.tx));
			CHECK(HAL_DMA_GetState(&r.tx) == HAL_DMA_STATE_READY);
			CHECK((r.inst->CR2 & (spiDmaCR2_TXDMAEN | spiDmaCR2_ERRIE)) == 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK(HAL_SPI_GetError(&r.hspi) == HAL_SPI_ERROR_NONE);
			CHECK((r.inst->SR & spiDmaSR_OVR) == 0u);
			CHECK(spiCounts.abortCplt == 0u);
			CHECK(hostDmaProtectedWrites(r.m.c, r.m.txS) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD4 Abort of a TransmitReceive DMA transfer stops both streams, clears the DMA enables and SPE") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(6);
			REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 6) == HAL_OK);
			CHECK(HAL_SPI_Abort(&r.hspi) == HAL_OK);
			CHECK(!r.streamEnabled(r.tx));
			CHECK(!r.streamEnabled(r.rx));
			CHECK((r.inst->CR2 & (spiDmaCR2_TXDMAEN | spiDmaCR2_RXDMAEN | spiDmaCR2_ERRIE)) == 0u);
			CHECK((r.inst->CR1 & spiDmaCR1_SPE) == 0u); // idle bus: BSY = 0, disabling is safe (28.3.8)
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK(spiCounts.abortCplt == 0u);
			CHECK(hostDmaProtectedWrites(r.m.c, r.m.txS) == 0u);
			CHECK(hostDmaProtectedWrites(r.m.c, r.m.rxS) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD4 Abort of a DMA transfer clears an OVR that was pending") {
	SpiDmaRig r(0, false);
	r.fillTx(6);
	REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 6) == HAL_OK);
	r.engineWrite(3);
	r.waitFlag(spiDmaSR_TXE);
	HAL_GetTick();
	REQUIRE((r.inst->SR & spiDmaSR_OVR) != 0u);
	CHECK(HAL_SPI_Abort(&r.hspi) == HAL_OK);
	CHECK((r.inst->SR & spiDmaSR_OVR) == 0u);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD4 characterization: Abort of a Tx only DMA transfer leaves SPE set") {
	// NEEDS REVIEW N5: UM1725 lists the steps of Abort without SPE; RM0090 28.3.8 makes disabling optional.
	SpiDmaRig r(0, false);
	r.fillTx(6);
	REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 6) == HAL_OK);
	REQUIRE(HAL_SPI_Abort(&r.hspi) == HAL_OK);
	CHECK((r.inst->CR1 & spiDmaCR1_SPE) != 0u);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD4 characterization: Abort of a TransmitReceive DMA transfer disables SPE while a frame is running") {
	// NEEDS REVIEW N5: the HAL clears SPE after the stream abort without waiting for TXE / BSY (RM0090 28.3.8 describes the wait).
	hostSpiSetFrameTicks(5);
	SpiDmaRig r(1, false);
	r.fillTx(4);
	REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 4) == HAL_OK);
	r.engineWrite(1);
	REQUIRE((r.inst->SR & spiDmaSR_BSY) != 0u);
	CHECK(HAL_SPI_Abort(&r.hspi) == HAL_OK);
	CHECK((r.inst->CR1 & spiDmaCR1_SPE) == 0u);
	CHECK(hostSpiDisabledBusy(1) == 1u);
	CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD4 Abort_IT of a Tx DMA transfer runs AbortCplt once after the Tx stream abort completes") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(6);
			REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 6) == HAL_OK);
			CHECK(HAL_SPI_Abort_IT(&r.hspi) == HAL_OK);
			CHECK(spiCounts.abortCplt == 0u); // UM1725: complete only when the callback runs
			HAL_DMA_IRQHandler(&r.tx);        // abort completion of the stream
			CHECK(spiCounts.abortCplt == 1u);
			CHECK(spiCounts.stateAtLast == HAL_SPI_STATE_READY);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK(HAL_SPI_GetError(&r.hspi) == HAL_SPI_ERROR_NONE);
			CHECK((r.inst->CR2 & (spiDmaCR2_TXDMAEN | spiDmaCR2_ERRIE)) == 0u);
			CHECK(!r.streamEnabled(r.tx));
			CHECK(hostDmaProtectedWrites(r.m.c, r.m.txS) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD4 Abort_IT of a TransmitReceive DMA transfer runs AbortCplt once, after both streams finished aborting") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(6);
			REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 6) == HAL_OK);
			CHECK(HAL_SPI_Abort_IT(&r.hspi) == HAL_OK);
			CHECK(spiCounts.abortCplt == 0u);
			HAL_DMA_IRQHandler(&r.tx);
			CHECK(spiCounts.abortCplt == 0u); // the Rx stream is still aborting
			HAL_DMA_IRQHandler(&r.rx);
			CHECK(spiCounts.abortCplt == 1u);
			CHECK(spiCounts.stateAtLast == HAL_SPI_STATE_READY);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK((r.inst->CR2 & (spiDmaCR2_TXDMAEN | spiDmaCR2_RXDMAEN | spiDmaCR2_ERRIE)) == 0u);
			CHECK((r.inst->CR1 & spiDmaCR1_SPE) == 0u);
			CHECK(!r.streamEnabled(r.tx));
			CHECK(!r.streamEnabled(r.rx));
			CHECK(spiCounts.error == 0u);
		}
	}
}

/* ------------------------------------------------------------------------------------------------------------------ SD5 */

TEST_CASE_FIXTURE(SpiDmaFx, "SD5 DMAPause clears TXDMAEN and RXDMAEN, keeps the state and the streams, and returns HAL_OK") {
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE((std::string("SPI") + std::to_string(n + 1)).c_str()) {
			SpiDmaRig r(n, false);
			r.fillTx(4);
			REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 4) == HAL_OK);
			CHECK(HAL_SPI_DMAPause(&r.hspi) == HAL_OK);
			CHECK((r.inst->CR2 & (spiDmaCR2_TXDMAEN | spiDmaCR2_RXDMAEN)) == 0u);
			CHECK((r.inst->CR2 & spiDmaCR2_ERRIE) != 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_BUSY_TX_RX);
			CHECK(r.streamEnabled(r.tx));
			CHECK(r.streamEnabled(r.rx));
			CHECK(r.hspi.Lock == HAL_UNLOCKED);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD5 DMAResume sets TXDMAEN and RXDMAEN again after a pause and returns HAL_OK") {
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE((std::string("SPI") + std::to_string(n + 1)).c_str()) {
			SpiDmaRig r(n, false);
			r.fillTx(4);
			REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 4) == HAL_OK);
			REQUIRE(HAL_SPI_DMAPause(&r.hspi) == HAL_OK);
			CHECK(HAL_SPI_DMAResume(&r.hspi) == HAL_OK);
			CHECK((r.inst->CR2 & spiDmaCR2_TXDMAEN) != 0u);
			CHECK((r.inst->CR2 & spiDmaCR2_RXDMAEN) != 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_BUSY_TX_RX);
			CHECK(r.hspi.Lock == HAL_UNLOCKED);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD5 DMAStop aborts both streams, clears both DMA enables and sets the state READY") {
	for (const auto &k : spiDmaConfigs()) {
		SUBCASE(k.name.c_str()) {
			SpiDmaRig r(k.n, k.b16);
			r.fillTx(4);
			REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 4) == HAL_OK);
			CHECK(HAL_SPI_DMAStop(&r.hspi) == HAL_OK);
			CHECK(!r.streamEnabled(r.tx));
			CHECK(!r.streamEnabled(r.rx));
			CHECK((r.inst->CR2 & (spiDmaCR2_TXDMAEN | spiDmaCR2_RXDMAEN)) == 0u);
			CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
			CHECK(HAL_SPI_GetError(&r.hspi) == HAL_SPI_ERROR_NONE);
			CHECK(hostDmaProtectedWrites(r.m.c, r.m.txS) == 0u);
			CHECK(hostDmaProtectedWrites(r.m.c, r.m.rxS) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD5 characterization: DMAStop after a Tx only transfer returns HAL_ERROR with the DMA error code but stops the Tx stream and sets READY") {
	// NEEDS REVIEW N6: the idle Rx stream cannot be aborted (HAL_DMA_Abort of a stream that is not busy fails).
	SpiDmaRig r(0, false);
	r.fillTx(4);
	REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 4) == HAL_OK);
	CHECK(HAL_SPI_DMAStop(&r.hspi) == HAL_ERROR);
	CHECK((HAL_SPI_GetError(&r.hspi) & HAL_SPI_ERROR_DMA) != 0u);
	CHECK(!r.streamEnabled(r.tx));
	CHECK((r.inst->CR2 & spiDmaCR2_TXDMAEN) == 0u);
	CHECK(HAL_SPI_GetState(&r.hspi) == HAL_SPI_STATE_READY);
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD5 characterization: DMAResume of a Tx only transfer also sets RXDMAEN") {
	// NEEDS REVIEW N6.
	SpiDmaRig r(2, false);
	r.fillTx(4);
	REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 4) == HAL_OK);
	REQUIRE(HAL_SPI_DMAPause(&r.hspi) == HAL_OK);
	CHECK((r.inst->CR2 & spiDmaCR2_TXDMAEN) == 0u);
	REQUIRE(HAL_SPI_DMAResume(&r.hspi) == HAL_OK);
	CHECK((r.inst->CR2 & spiDmaCR2_TXDMAEN) != 0u);
	CHECK((r.inst->CR2 & spiDmaCR2_RXDMAEN) != 0u);
}

/* ------------------------------------------------------------------------------------------------------------------ SD6 */

TEST_CASE_FIXTURE(SpiDmaFx, "SD6 characterization: Transmit_DMA of 16 bit frames on a byte aligned stream programs NDTR = Size without conversion") {
	// NEEDS REVIEW N7: UM1725 gives no formula; RM0090 10.5.6 counts data items of the stream's size (here 8 bit items).
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE((std::string("SPI") + std::to_string(n + 1)).c_str()) {
			SpiDmaRig r(n, true, false, true);
			r.fillTx(5);
			REQUIRE(HAL_SPI_Transmit_DMA(&r.hspi, r.txPtr(), 5) == HAL_OK);
			CHECK(r.tx.Instance->NDTR == 5u);
			CHECK((r.inst->CR1 & (1u << 11)) != 0u); // DFF: the SPI frame is 16 bit
			CHECK(r.tx.Instance->PAR == spiDmaDrAddress[n]);
		}
	}
}

TEST_CASE_FIXTURE(SpiDmaFx, "SD6 characterization: TransmitReceive_DMA of 16 bit frames on byte aligned streams programs NDTR = Size without conversion on both streams") {
	// NEEDS REVIEW N7.
	SpiDmaRig r(1, true, false, true);
	r.fillTx(5);
	REQUIRE(HAL_SPI_TransmitReceive_DMA(&r.hspi, r.txPtr(), r.rxPtr(), 5) == HAL_OK);
	CHECK(r.tx.Instance->NDTR == 5u);
	CHECK(r.rx.Instance->NDTR == 5u);
}
