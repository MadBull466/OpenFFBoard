/*
 * P12 T4c group I2D: the DMA mode of the real ST HAL I2C driver (stm32f4xx_hal_i2c.c: HAL_I2C_Master_Transmit_DMA, Master_Receive_DMA,
 * Slave_Transmit_DMA, Slave_Receive_DMA, Mem_Write_DMA, Mem_Read_DMA, Master_Seq_Transmit_DMA, Master_Seq_Receive_DMA,
 * Slave_Seq_Transmit_DMA, Slave_Seq_Receive_DMA, I2C_DMAXferCplt, I2C_DMAError, I2C_DMAAbort and the DMA branches of
 * HAL_I2C_EV_IRQHandler, HAL_I2C_ER_IRQHandler, I2C_ITError, I2C_Master_SB / ADDR, I2C_Slave_STOPF) together with the real HAL DMA, against the
 * I2C model (fakes/stm32_hal/HalI2c.h) and the DMA model (fakes/stm32_hal/HalDma.h). Requirements ID1-ID4 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T4C_REQUIREMENTS.md; compiled into ffb_hal_tests (F407VG) and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 (I2C chapter: function list, I2C_XferOptions descriptions, HAL_I2C_Master_Abort_IT, error code names) and RM0090 Rev 22:
 *  - Table 1: I2C1 0x40005400, I2C2 0x40005800, I2C3 0x40005C00; I2C_DR at offset 0x10.
 *  - Table 43 (DMA1): I2C1_RX stream 0 / 5 channel 1, I2C1_TX stream 6 / 7 channel 1, I2C2_RX stream 2 / 3 channel 7, I2C2_TX stream 7 channel 7,
 *    I2C3_RX stream 2 channel 3, I2C3_TX stream 4 channel 3. This file uses I2C1 tx 6 / rx 0, I2C2 tx 7 / rx 2, I2C3 tx 4 / rx 2.
 *  - 10.5.5: DMA_SxCR EN bit 0, TEIE 2, HTIE 3, TCIE 4, DIR 7:6 (00 peripheral to memory, 01 memory to peripheral), CHSEL 27:25; 10.5.6 NDTR counts items.
 *  - 27.6.1 CR1: PE 0, START 8, STOP 9, ACK 10, POS 11. 27.6.2 CR2: ITERREN 8, ITEVTEN 9, ITBUFEN 10, DMAEN 11, LAST 12. 27.6.5 SR1, 27.6.6 SR2.
 *  - 27.3.8 DMA requests: "The DMAEN bit must be set in the I2C_CR2 register before the ADDR event"; controller transmitter: after EOT "disable DMA
 *    requests then wait for a BTF event before programming the Stop condition"; controller receiver of two or more bytes: "If the LAST bit is
 *    set, I2C automatically sends a NACK after the next byte following EOT_1. The user can generate a Stop condition in the DMA Transfer Complete
 *    interrupt routine"; a single byte: "program ACK=0 when ADDR=1, before clearing ADDR flag"; transmission: "Do not enable the ITBUFEN bit in the
 *    I2C_CR2 register if DMA is used for transmission".
 *  - 27.3.3 controller receiver: the last byte is answered with a NACK.
 *
 * The DMA streams are started by the HAL (HAL_DMA_Start_IT through the real HAL DMA); the tests stand in for the DMA engine: they move the bytes
 * with plain I2C_DR accesses (a write when TXE = 1, a read when RXNE = 1) and finish the stream with hostDmaComplete + HAL_DMA_IRQHandler,
 * as in test_hal_spi_dma.cpp. The interrupts are delivered by i2cRunIrqs (the support file). A transfer that the model cannot express is
 * noted: the model has no DMA EOT_1 signal, so the LAST bit does not turn the last byte into a NACK; for 3 or more received bytes the tests check the
 * LAST bit and the data, not the NACK.
 * Every file-local symbol is prefixed i2d or sits in the anonymous namespace.
 *
 * NEEDS REVIEW proposals (numbers assigned by the lead from 320, written as NEEDS REVIEW <file number> after renumbering):
 *  NEEDS REVIEW 325 ID1: Master_Transmit_DMA / Master_Receive_DMA with Size 0 start no DMA stream, send the address (START) and enable EVT, BUF and ERR interrupts.
 *  NEEDS REVIEW 325 ID1: HAL_ERROR and HAL_I2C_ERROR_DMA_PARAM for a missing DMA handle; the master functions and Mem functions leave READY, the non sequential slave
 *     functions leave the state LISTEN (UM1725 silent; suspicious: no listen was requested).
 *  NEEDS REVIEW 325 ID1: HAL_BUSY when the handle is not READY (slave sequential: not LISTEN), HAL_ERROR for a NULL buffer or Size 0 in the slave functions.
 *  NEEDS REVIEW 325 ID1: a failing HAL_DMA_Start_IT: HAL_ERROR, HAL_I2C_ERROR_DMA, state READY, mode NONE, unlocked.
 *  NEEDS REVIEW 325 ID1: Mem_Write_DMA with Size 0: HAL_ERROR with HAL_I2C_ERROR_SIZE; Mem_Read_DMA with Size 0: address phase, STOP, HAL_OK.
 *  NEEDS REVIEW 325 ID1/ID3: an address nack in Mem_Write_DMA / Mem_Read_DMA: HAL_ERROR, AF, state READY, stream aborted, PE and ACK cleared, XferCount 0.
 *  NEEDS REVIEW 325 ID2: completion of a receive with a pending error code calls the error callback instead of the complete callback; completion after the mode
 *     was reset does nothing.
 *  NEEDS REVIEW 325 ID2: slave receive: an early stop while the DMA runs ends with HAL_I2C_ERROR_AF and the error callback; slave transmit: the controller NACK
 *     before the last byte ends with AF and the error callback only after the DMA stream stopped.
 *  NEEDS REVIEW 325 ID3: a DMA FIFO error is ignored by I2C_DMAError (state, error code, callbacks unchanged).
 *  NEEDS REVIEW 325 ID3: an I2C error during DMA stops the stream first; HAL_I2C_ErrorCallback runs only when the abort finished (HAL comment: "will lead to call
 *     HAL_I2C_ErrorCallback() at end of DMA abort procedure"); the DMA abort callback waits for a pending STOP and ends with HAL_I2C_ERROR_TIMEOUT when it stays.
 *  NEEDS REVIEW 325 ID3: when the stream cannot be aborted the HAL ends the transfer itself (PE cleared, state READY) and still calls the error callback.
 *  NEEDS REVIEW 325 ID4: slave sequential DMA calls that switch direction or repeat a direction stop the running stream first; the HAL error callback follows the abort
 *     (directly when the stream cannot be aborted); a same direction call while the stream aborts returns HAL_ERROR with HAL_I2C_ERROR_DMA.
 *  NEEDS REVIEW 325 ID3: after an ignored DMA FIFO error the complete callback of both streams is NULL, so the end of the stream never completes the transfer (state stays BUSY_TX, DMAEN set, no STOP, no callback).
 *  NEEDS REVIEW 325 ID4: after a slave Seq rx to tx switch the DMA abort callback sets state LISTEN while the new Tx stream runs.
 *  NEEDS REVIEW 325 ID1: a DMA call with the bus held busy returns HAL_BUSY with HAL_I2C_ERROR_TIMEOUT after the BUSY wait, state READY, mode NONE.
 *  NEEDS REVIEW 325 ID1: a transfer with Size 0 in the sequential DMA functions behaves like the plain ones (address phase, STOP).
 *  NEEDS REVIEW 325 ID4: a FIRST_FRAME receive of 3 or more bytes sets LAST in the ADDR handler (the HAL's own Seq_Receive_DMA does not set it for that option).
 */
#include "HalDma.h"
#include "HalFixture.h"
#include "HalHostSystem.h"
#include "HalI2c.h"
#include "I2cTestSupport.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

extern "C" uint32_t SystemCoreClock;

namespace {

constexpr uint32_t i2dCR1_PE = 1u << 0;
constexpr uint32_t i2dCR1_START = 1u << 8;
constexpr uint32_t i2dCR1_STOP = 1u << 9;
constexpr uint32_t i2dCR1_ACK = 1u << 10;
constexpr uint32_t i2dCR1_POS = 1u << 11;
constexpr uint32_t i2dCR2_ITERREN = 1u << 8;
constexpr uint32_t i2dCR2_ITEVTEN = 1u << 9;
constexpr uint32_t i2dCR2_ITBUFEN = 1u << 10;
constexpr uint32_t i2dCR2_DMAEN = 1u << 11;
constexpr uint32_t i2dCR2_LAST = 1u << 12;
constexpr uint32_t i2dSR1_ADDR = 1u << 1;
constexpr uint32_t i2dSR1_RXNE = 1u << 6;
constexpr uint32_t i2dSR1_TXE = 1u << 7;
constexpr uint32_t i2dSR1_BERR = 1u << 8;
constexpr uint32_t i2dSR1_OVR = 1u << 11;
constexpr uint32_t i2dSR2_BUSY = 1u << 1;
constexpr uint32_t i2dDMA_EN = 1u << 0;
constexpr uint32_t i2dDMA_TEIE = 1u << 2;
constexpr uint32_t i2dDMA_TCIE = 1u << 4;
constexpr uint32_t i2dDMA_DIR_SHIFT = 6;
constexpr uint32_t i2dDMA_DIR_MASK = 3u << 6;
constexpr uint32_t i2dDMA_CHSEL_SHIFT = 25;
constexpr uint32_t i2dDMA_CHSEL_MASK = 7u << 25;

/* RM0090 Table 1 + 27.6.5 (I2C_DR offset 0x10). */
constexpr uint32_t i2dDrAddress[3] = {0x40005410u, 0x40005810u, 0x40005C10u};

struct I2dMap {
	unsigned txS, rxS;
	uint32_t channel;
	DMA_Stream_TypeDef *txI, *rxI;
};
/* RM0090 Table 43 (all DMA1 = controller 0 of the model). */
I2dMap i2dMapOf(unsigned n) {
	if (n == 0) return {6, 0, DMA_CHANNEL_1, DMA1_Stream6, DMA1_Stream0};
	if (n == 1) return {7, 2, DMA_CHANNEL_7, DMA1_Stream7, DMA1_Stream2};
	return {4, 2, DMA_CHANNEL_3, DMA1_Stream4, DMA1_Stream2};
}

I2C_TypeDef *i2dInstance(unsigned n) {
	I2C_TypeDef *const a[3] = {I2C1, I2C2, I2C3};
	return a[n];
}

const char *const i2dInstanceName[3] = {"I2C1", "I2C2", "I2C3"};

/* The bytes the peer device sends (hand chosen). */
const uint8_t i2dPeerBytes[8] = {0x91, 0x82, 0x73, 0x64, 0x55, 0x46, 0x37, 0x28};

/* Bus log as text: S start, R repeated start, P stop, o:XX byte from the controller, i:XX byte from the target, "!" after a byte that was not acked. */
std::string i2dLog(unsigned n) {
	std::string s;
	for (unsigned i = 0; i < hostI2cLogCount(n); ++i) {
		const HostI2cEvent e = hostI2cLogAt(n, i);
		char b[16];
		switch (e.type) {
		case HOST_I2C_EV_START: std::snprintf(b, sizeof b, "S"); break;
		case HOST_I2C_EV_RESTART: std::snprintf(b, sizeof b, "R"); break;
		case HOST_I2C_EV_STOP: std::snprintf(b, sizeof b, "P"); break;
		case HOST_I2C_EV_BYTE_OUT: std::snprintf(b, sizeof b, "o:%02X%s", e.value, e.ack ? "" : "!"); break;
		default: std::snprintf(b, sizeof b, "i:%02X%s", e.value, e.ack ? "" : "!"); break;
		}
		if (!s.empty()) s += ' ';
		s += b;
	}
	return s;
}
bool i2dStarts(const std::string &s, const std::string &p) { return s.compare(0, p.size(), p) == 0; }
bool i2dEnds(const std::string &s, const std::string &p) { return s.size() >= p.size() && s.compare(s.size() - p.size(), p.size(), p) == 0; }

struct I2dRig {
	unsigned n;
	I2dMap m;
	I2C_TypeDef *inst;
	I2C_HandleTypeDef h;
	DMA_HandleTypeDef tx, rx;
	uint8_t buf[300];
	int dev = -1;

	/* withTx / withRx: link the DMA handles to the I2C handle. fifo: the DMA streams use the FIFO. */
	explicit I2dRig(unsigned idx, bool withTx = true, bool withRx = true, bool fifo = false, bool tenBit = false, uint32_t ownAddress = 0x30u)
	    : n(idx), m(i2dMapOf(idx)), inst(i2dInstance(idx)) {
		std::memset(&h, 0, sizeof h);
		std::memset(buf, 0, sizeof buf);
		SystemCoreClock = 168000000u;
		RCC->CFGR = (RCC->CFGR & ~(7u << 10)) | (5u << 10); /* PPRE1 /4 -> 42 MHz */
		uwTickPrio = 15;
		h.Instance = inst;
		h.Init.ClockSpeed = 100000;
		h.Init.DutyCycle = I2C_DUTYCYCLE_2;
		h.Init.OwnAddress1 = ownAddress;
		h.Init.AddressingMode = tenBit ? I2C_ADDRESSINGMODE_10BIT : I2C_ADDRESSINGMODE_7BIT;
		h.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		h.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		h.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		REQUIRE(HAL_I2C_Init(&h) == HAL_OK);
		makeDma(tx, m.txI, DMA_MEMORY_TO_PERIPH, fifo);
		makeDma(rx, m.rxI, DMA_PERIPH_TO_MEMORY, fifo);
		if (withTx) h.hdmatx = &tx;
		if (withRx) h.hdmarx = &rx;
		tx.Parent = &h;
		rx.Parent = &h;
		i2cSupportReset();
	}

	void makeDma(DMA_HandleTypeDef &d, DMA_Stream_TypeDef *s, uint32_t dir, bool fifo) {
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
		d.Init.FIFOMode = fifo ? DMA_FIFOMODE_ENABLE : DMA_FIFOMODE_DISABLE;
		d.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		d.Init.MemBurst = DMA_MBURST_SINGLE;
		d.Init.PeriphBurst = DMA_PBURST_SINGLE;
		REQUIRE(HAL_DMA_Init(&d) == HAL_OK);
	}

	void addPeer() { dev = hostI2cDeviceAdd(n, 0x50, 0); }
	void addPeer10() { dev = hostI2cDeviceAdd(n, 0x2B7, 1); }
	void queuePeer(unsigned count) { hostI2cDeviceQueue(n, dev, i2dPeerBytes, count); }
	void fillTx(unsigned count) {
		for (unsigned i = 0; i < count; ++i) buf[i] = static_cast<uint8_t>(0x11u * (i + 1u));
	}
	uint32_t bufAddr() const { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buf)); }
	unsigned irqs() { return i2cRunIrqs(&h, n); }
	bool enabled(const DMA_HandleTypeDef &d) const { return (d.Instance->CR & i2dDMA_EN) != 0u; }
	uint32_t sr1() const { return HalFixture::peek(&inst->SR1); }
	uint32_t sr2() const { return HalFixture::peek(&inst->SR2); }

	void waitSr1(uint32_t flag) {
		for (unsigned g = 0; g < 2000; ++g) {
			if (sr1() & flag) return;
			HAL_GetTick();
		}
		FAIL("I2C flag never set");
	}
	/* DMA engine, memory to peripheral: a DR write whenever TXE = 1. */
	void engineTx(const uint8_t *d, unsigned count) {
		for (unsigned i = 0; i < count; ++i) {
			waitSr1(i2dSR1_TXE);
			inst->DR = d[i];
		}
	}
	/* DMA engine, peripheral to memory: a DR read whenever RXNE = 1. */
	void engineRx(uint8_t *d, unsigned count) {
		for (unsigned i = 0; i < count; ++i) {
			waitSr1(i2dSR1_RXNE);
			d[i] = static_cast<uint8_t>(inst->DR);
		}
	}
	void doneTx() {
		hostDmaComplete(0, m.txS);
		HAL_DMA_IRQHandler(&tx);
	}
	void doneRx() {
		hostDmaComplete(0, m.rxS);
		HAL_DMA_IRQHandler(&rx);
	}
};

class I2dFx : public I2cFixture {};

constexpr uint16_t i2dDev = 0x50u << 1; /* the shifted address the HAL functions take */

} // namespace

/* ------------------------------------------------------------------------------------------------------------------ ID1 */

TEST_CASE_FIXTURE(I2dFx, "ID1 Master_Transmit_DMA programs the Tx stream with the data register, the buffer, memory-to-peripheral direction, the channel and Size items") {
	for (unsigned n = 0; n < 3; ++n) {
		for (unsigned size : {1u, 2u, 3u, 255u}) {
			SUBCASE((std::string(i2dInstanceName[n]) + " size " + std::to_string(size)).c_str()) {
				I2dRig r(n);
				r.addPeer();
				REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, static_cast<uint16_t>(size)) == HAL_OK);
				const uint32_t cr = r.tx.Instance->CR;
				CHECK((cr & i2dDMA_EN) != 0u);
				CHECK(((cr & i2dDMA_DIR_MASK) >> i2dDMA_DIR_SHIFT) == 1u); /* 10.5.5: 01 memory to peripheral */
				CHECK(((cr & i2dDMA_CHSEL_MASK) >> i2dDMA_CHSEL_SHIFT) == (r.m.channel >> i2dDMA_CHSEL_SHIFT));
				CHECK(r.tx.Instance->PAR == i2dDrAddress[n]);
				CHECK(r.tx.Instance->M0AR == r.bufAddr());
				CHECK(r.tx.Instance->NDTR == size); /* 10.5.6 */
				CHECK((cr & i2dDMA_TCIE) != 0u);
				CHECK((cr & i2dDMA_TEIE) != 0u);
				CHECK(!r.enabled(r.rx));
				CHECK(hostDmaProtectedWrites(0, r.m.txS) == 0u);
			}
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 Master_Transmit_DMA sets DMAEN, ITEVTEN, ITERREN and ACK but not ITBUFEN, requests START, state BUSY_TX in master mode and unlocks") {
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE(i2dInstanceName[n]) {
			I2dRig r(n);
			r.addPeer();
			r.fillTx(2);
			REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 2) == HAL_OK);
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) != 0u);                   /* 27.3.8: DMAEN before the ADDR event */
			CHECK((r.inst->CR2 & i2dCR2_ITEVTEN) != 0u);
			CHECK((r.inst->CR2 & i2dCR2_ITERREN) != 0u);
			CHECK((r.inst->CR2 & i2dCR2_ITBUFEN) == 0u);                 /* 27.3.8 note: no ITBUFEN with DMA transmission */
			CHECK((r.inst->CR1 & i2dCR1_ACK) != 0u);
			CHECK((r.inst->CR1 & i2dCR1_POS) == 0u);
			CHECK((r.sr2() & i2dSR2_BUSY) != 0u);                       /* START was generated: the bus is busy */
			CHECK(i2dLog(n) == "S");
			CHECK(r.h.State == HAL_I2C_STATE_BUSY_TX);
			CHECK(r.h.Mode == HAL_I2C_MODE_MASTER);
			CHECK(r.h.ErrorCode == HAL_I2C_ERROR_NONE);
			CHECK(r.h.Lock == HAL_UNLOCKED);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 Master_Receive_DMA programs the Rx stream with the data register, the buffer, peripheral-to-memory direction, the channel and Size items") {
	for (unsigned n = 0; n < 3; ++n) {
		for (unsigned size : {1u, 2u, 3u, 255u}) {
			SUBCASE((std::string(i2dInstanceName[n]) + " size " + std::to_string(size)).c_str()) {
				I2dRig r(n);
				r.addPeer();
				REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, static_cast<uint16_t>(size)) == HAL_OK);
				const uint32_t cr = r.rx.Instance->CR;
				CHECK((cr & i2dDMA_EN) != 0u);
				CHECK(((cr & i2dDMA_DIR_MASK) >> i2dDMA_DIR_SHIFT) == 0u); /* 10.5.5: 00 peripheral to memory */
				CHECK(((cr & i2dDMA_CHSEL_MASK) >> i2dDMA_CHSEL_SHIFT) == (r.m.channel >> i2dDMA_CHSEL_SHIFT));
				CHECK(r.rx.Instance->PAR == i2dDrAddress[n]);
				CHECK(r.rx.Instance->M0AR == r.bufAddr());
				CHECK(r.rx.Instance->NDTR == size);
				CHECK((cr & i2dDMA_TCIE) != 0u);
				CHECK((cr & i2dDMA_TEIE) != 0u);
				CHECK(!r.enabled(r.tx));
				CHECK(hostDmaProtectedWrites(0, r.m.rxS) == 0u);
			}
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 Master_Receive_DMA sets DMAEN, ITEVTEN, ITERREN and ACK but not ITBUFEN, requests START, state BUSY_RX in master mode and unlocks") {
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE(i2dInstanceName[n]) {
			I2dRig r(n);
			r.addPeer();
			REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, 4) == HAL_OK);
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) != 0u);
			CHECK((r.inst->CR2 & i2dCR2_ITEVTEN) != 0u);
			CHECK((r.inst->CR2 & i2dCR2_ITERREN) != 0u);
			CHECK((r.inst->CR2 & i2dCR2_ITBUFEN) == 0u);
			CHECK((r.inst->CR1 & i2dCR1_ACK) != 0u);
			CHECK((r.inst->CR1 & i2dCR1_POS) == 0u);
			CHECK(i2dLog(n) == "S");
			CHECK(r.h.State == HAL_I2C_STATE_BUSY_RX);
			CHECK(r.h.Mode == HAL_I2C_MODE_MASTER);
			CHECK(r.h.Lock == HAL_UNLOCKED);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 characterization: Master_Transmit_DMA and Master_Receive_DMA with Size 0 start no stream and enable the event, buffer and error interrupts") {
	/* NEEDS REVIEW 325: UM1725 silent on Size 0. */
	SUBCASE("transmit") {
		I2dRig r(0);
		r.addPeer();
		REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 0) == HAL_OK);
		CHECK(!r.enabled(r.tx));
		CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
		CHECK((r.inst->CR2 & (i2dCR2_ITEVTEN | i2dCR2_ITBUFEN | i2dCR2_ITERREN)) == (i2dCR2_ITEVTEN | i2dCR2_ITBUFEN | i2dCR2_ITERREN));
		CHECK(i2dLog(0) == "S");
		CHECK(r.h.State == HAL_I2C_STATE_BUSY_TX);
		CHECK(r.h.Lock == HAL_UNLOCKED);
	}
	SUBCASE("receive") {
		I2dRig r(1);
		r.addPeer();
		REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, 0) == HAL_OK);
		CHECK(!r.enabled(r.rx));
		CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
		CHECK((r.inst->CR2 & (i2dCR2_ITEVTEN | i2dCR2_ITBUFEN | i2dCR2_ITERREN)) == (i2dCR2_ITEVTEN | i2dCR2_ITBUFEN | i2dCR2_ITERREN));
		CHECK(r.h.State == HAL_I2C_STATE_BUSY_RX);
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 Slave_Transmit_DMA and Slave_Receive_DMA program their streams with the data register, the buffer, the direction and Size items") {
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE(i2dInstanceName[n]) {
			SUBCASE("transmit") {
				I2dRig r(n);
				REQUIRE(HAL_I2C_Slave_Transmit_DMA(&r.h, r.buf, 6) == HAL_OK);
				const uint32_t cr = r.tx.Instance->CR;
				CHECK((cr & i2dDMA_EN) != 0u);
				CHECK(((cr & i2dDMA_DIR_MASK) >> i2dDMA_DIR_SHIFT) == 1u);
				CHECK(r.tx.Instance->PAR == i2dDrAddress[n]);
				CHECK(r.tx.Instance->M0AR == r.bufAddr());
				CHECK(r.tx.Instance->NDTR == 6u);
				CHECK(!r.enabled(r.rx));
			}
			SUBCASE("receive") {
				I2dRig r(n);
				REQUIRE(HAL_I2C_Slave_Receive_DMA(&r.h, r.buf, 5) == HAL_OK);
				const uint32_t cr = r.rx.Instance->CR;
				CHECK((cr & i2dDMA_EN) != 0u);
				CHECK(((cr & i2dDMA_DIR_MASK) >> i2dDMA_DIR_SHIFT) == 0u);
				CHECK(r.rx.Instance->PAR == i2dDrAddress[n]);
				CHECK(r.rx.Instance->M0AR == r.bufAddr());
				CHECK(r.rx.Instance->NDTR == 5u);
				CHECK(!r.enabled(r.tx));
			}
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 Slave_Transmit_DMA and Slave_Receive_DMA enable ACK, DMAEN, ITEVTEN and ITERREN, send no START and set state BUSY_TX / BUSY_RX in slave mode") {
	for (int dir = 0; dir < 2; ++dir) {
		SUBCASE(dir == 0 ? "transmit" : "receive") {
			I2dRig r(0);
			if (dir == 0) {
				REQUIRE(HAL_I2C_Slave_Transmit_DMA(&r.h, r.buf, 3) == HAL_OK);
			} else {
				REQUIRE(HAL_I2C_Slave_Receive_DMA(&r.h, r.buf, 3) == HAL_OK);
			}
			CHECK((r.inst->CR1 & i2dCR1_ACK) != 0u); /* the own address must be acknowledged */
			CHECK((r.inst->CR1 & i2dCR1_START) == 0u);
			CHECK((r.sr2() & i2dSR2_BUSY) == 0u);
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) != 0u);
			CHECK((r.inst->CR2 & i2dCR2_ITEVTEN) != 0u);
			CHECK((r.inst->CR2 & i2dCR2_ITERREN) != 0u);
			CHECK((r.inst->CR2 & i2dCR2_ITBUFEN) == 0u);
			CHECK(r.h.State == (dir == 0 ? HAL_I2C_STATE_BUSY_TX : HAL_I2C_STATE_BUSY_RX));
			CHECK(r.h.Mode == HAL_I2C_MODE_SLAVE);
			CHECK(r.h.Lock == HAL_UNLOCKED);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 characterization: Slave_Transmit_DMA and Slave_Receive_DMA with a NULL buffer or Size 0 return HAL_ERROR and start nothing") {
	/* NEEDS REVIEW 325 */
	I2dRig r(1);
	CHECK(HAL_I2C_Slave_Transmit_DMA(&r.h, nullptr, 3) == HAL_ERROR);
	CHECK(HAL_I2C_Slave_Transmit_DMA(&r.h, r.buf, 0) == HAL_ERROR);
	CHECK(HAL_I2C_Slave_Receive_DMA(&r.h, nullptr, 3) == HAL_ERROR);
	CHECK(HAL_I2C_Slave_Receive_DMA(&r.h, r.buf, 0) == HAL_ERROR);
	CHECK(!r.enabled(r.tx));
	CHECK(!r.enabled(r.rx));
	CHECK(r.h.State == HAL_I2C_STATE_READY);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	CHECK((r.inst->CR2 & (i2dCR2_DMAEN | i2dCR2_ITEVTEN | i2dCR2_ITERREN)) == 0u);
}

TEST_CASE_FIXTURE(I2dFx, "ID1 Mem_Write_DMA sends the 8 or 16 bit memory address MSB first, programs the Tx stream and enables only ITERREN and DMAEN") {
	for (int wide = 0; wide < 2; ++wide) {
		SUBCASE(wide ? "16 bit address" : "8 bit address") {
			I2dRig r(2);
			r.addPeer();
			r.fillTx(3);
			REQUIRE(HAL_I2C_Mem_Write_DMA(&r.h, i2dDev, 0x1234, wide ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT, r.buf, 3) == HAL_OK);
			/* 8 bit: the LSB 0x34 only; 16 bit: MSB 0x12 then LSB 0x34. Start, then the write address 0x50 << 1 = 0xA0. */
			CHECK(i2dLog(2) == (wide ? "S o:A0 o:12 o:34" : "S o:A0 o:34"));
			CHECK((r.tx.Instance->CR & i2dDMA_EN) != 0u);
			CHECK(r.tx.Instance->PAR == i2dDrAddress[2]);
			CHECK(r.tx.Instance->M0AR == r.bufAddr());
			CHECK(r.tx.Instance->NDTR == 3u);
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) != 0u);
			CHECK((r.inst->CR2 & i2dCR2_ITERREN) != 0u);
			CHECK((r.inst->CR2 & (i2dCR2_ITEVTEN | i2dCR2_ITBUFEN)) == 0u);
			CHECK((r.sr1() & i2dSR1_ADDR) == 0u);
			CHECK(r.h.State == HAL_I2C_STATE_BUSY_TX);
			CHECK(r.h.Mode == HAL_I2C_MODE_MEM);
			CHECK(r.h.Lock == HAL_UNLOCKED);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 Mem_Read_DMA sends the memory address, a repeated START and the read address and programs the Rx stream") {
	for (int wide = 0; wide < 2; ++wide) {
		SUBCASE(wide ? "16 bit address" : "8 bit address") {
			I2dRig r(0);
			r.addPeer();
			r.queuePeer(4);
			REQUIRE(HAL_I2C_Mem_Read_DMA(&r.h, i2dDev, 0x1234, wide ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT, r.buf, 4) == HAL_OK);
			const std::string log = i2dLog(0);
			CHECK(i2dStarts(log, wide ? "S o:A0 o:12 o:34 R o:A1" : "S o:A0 o:34 R o:A1"));
			CHECK((r.rx.Instance->CR & i2dDMA_EN) != 0u);
			CHECK(((r.rx.Instance->CR & i2dDMA_DIR_MASK) >> i2dDMA_DIR_SHIFT) == 0u);
			CHECK(r.rx.Instance->PAR == i2dDrAddress[0]);
			CHECK(r.rx.Instance->M0AR == r.bufAddr());
			CHECK(r.rx.Instance->NDTR == 4u);
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) != 0u);
			CHECK((r.inst->CR2 & i2dCR2_ITERREN) != 0u);
			CHECK((r.inst->CR2 & (i2dCR2_ITEVTEN | i2dCR2_ITBUFEN)) == 0u);
			CHECK(r.h.State == HAL_I2C_STATE_BUSY_RX);
			CHECK(r.h.Mode == HAL_I2C_MODE_MEM);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 Mem_Read_DMA of one byte clears ACK and of two or more bytes sets LAST before the ADDR flag is cleared") {
	for (unsigned size : {1u, 2u, 5u}) {
		SUBCASE(("size " + std::to_string(size)).c_str()) {
			I2dRig r(1);
			r.addPeer();
			r.queuePeer(5);
			REQUIRE(HAL_I2C_Mem_Read_DMA(&r.h, i2dDev, 0x10, I2C_MEMADD_SIZE_8BIT, r.buf, static_cast<uint16_t>(size)) == HAL_OK);
			if (size == 1u) {
				CHECK((r.inst->CR1 & i2dCR1_ACK) == 0u); /* 27.3.8: single byte, ACK = 0 before ADDR is cleared */
				CHECK((r.inst->CR2 & i2dCR2_LAST) == 0u);
			} else {
				CHECK((r.inst->CR1 & i2dCR1_ACK) != 0u);
				CHECK((r.inst->CR2 & i2dCR2_LAST) != 0u); /* 27.3.8: two or more bytes, LAST set */
			}
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 characterization: Mem_Write_DMA with Size 0 returns HAL_ERROR with HAL_I2C_ERROR_SIZE, Mem_Read_DMA with Size 0 does the address phase, a STOP and returns HAL_OK") {
	/* NEEDS REVIEW 325 */
	SUBCASE("write") {
		I2dRig r(0);
		r.addPeer();
		CHECK(HAL_I2C_Mem_Write_DMA(&r.h, i2dDev, 0x10, I2C_MEMADD_SIZE_8BIT, r.buf, 0) == HAL_ERROR);
		CHECK(r.h.ErrorCode == HAL_I2C_ERROR_SIZE);
		CHECK(r.h.State == HAL_I2C_STATE_READY);
		CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
		CHECK(r.h.Lock == HAL_UNLOCKED);
	}
	SUBCASE("read") {
		I2dRig r(0);
		r.addPeer();
		CHECK(HAL_I2C_Mem_Read_DMA(&r.h, i2dDev, 0x10, I2C_MEMADD_SIZE_8BIT, r.buf, 0) == HAL_OK);
		CHECK(i2dStarts(i2dLog(0), "S o:A0 o:10 R o:A1"));
		CHECK(i2dEnds(i2dLog(0), "P"));
		CHECK(r.h.State == HAL_I2C_STATE_READY);
		CHECK(r.h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 characterization: Mem_Write_DMA and Mem_Read_DMA to an address that nacks return HAL_ERROR, set AF, stop the stream and disable the I2C") {
	/* NEEDS REVIEW 325 */
	for (int rd = 0; rd < 2; ++rd) {
		SUBCASE(rd ? "read" : "write") {
			I2dRig r(0); /* no peer on the bus: the address byte is not acknowledged */
			if (rd) {
				CHECK(HAL_I2C_Mem_Read_DMA(&r.h, i2dDev, 0x10, I2C_MEMADD_SIZE_8BIT, r.buf, 3) == HAL_ERROR);
			} else {
				CHECK(HAL_I2C_Mem_Write_DMA(&r.h, i2dDev, 0x10, I2C_MEMADD_SIZE_8BIT, r.buf, 3) == HAL_ERROR);
			}
			CHECK((r.h.ErrorCode & HAL_I2C_ERROR_AF) != 0u);
			CHECK(r.h.State == HAL_I2C_STATE_READY);
			CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
			CHECK(r.h.Lock == HAL_UNLOCKED);
			CHECK(r.h.XferCount == 0u);
			CHECK(r.h.XferSize == 0u);
			DMA_HandleTypeDef &d = rd ? r.rx : r.tx;
			CHECK(!r.enabled(d)); /* the stream was stopped */
			CHECK(d.XferCpltCallback == nullptr);
			CHECK((r.inst->CR1 & i2dCR1_PE) == 0u);
			CHECK((r.inst->CR1 & i2dCR1_ACK) == 0u);
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 characterization: every DMA call returns HAL_BUSY while the handle is not READY and touches no stream") {
	/* NEEDS REVIEW 325 */
	I2dRig r(0);
	r.addPeer();
	REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 3) == HAL_OK);
	const uint32_t ndtr = r.tx.Instance->NDTR;
	CHECK(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 2) == HAL_BUSY);
	CHECK(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, 2) == HAL_BUSY);
	CHECK(HAL_I2C_Slave_Transmit_DMA(&r.h, r.buf, 2) == HAL_BUSY);
	CHECK(HAL_I2C_Slave_Receive_DMA(&r.h, r.buf, 2) == HAL_BUSY);
	CHECK(HAL_I2C_Mem_Write_DMA(&r.h, i2dDev, 1, I2C_MEMADD_SIZE_8BIT, r.buf, 2) == HAL_BUSY);
	CHECK(HAL_I2C_Mem_Read_DMA(&r.h, i2dDev, 1, I2C_MEMADD_SIZE_8BIT, r.buf, 2) == HAL_BUSY);
	CHECK(HAL_I2C_Master_Seq_Transmit_DMA(&r.h, i2dDev, r.buf, 2, I2C_FIRST_FRAME) == HAL_BUSY);
	CHECK(HAL_I2C_Master_Seq_Receive_DMA(&r.h, i2dDev, r.buf, 2, I2C_FIRST_FRAME) == HAL_BUSY);
	CHECK(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 2, I2C_FIRST_FRAME) == HAL_BUSY); /* needs the LISTEN state */
	CHECK(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, r.buf, 2, I2C_FIRST_FRAME) == HAL_BUSY);
	CHECK(r.tx.Instance->NDTR == ndtr);
	CHECK(!r.enabled(r.rx));
	CHECK(r.h.State == HAL_I2C_STATE_BUSY_TX);
	CHECK(hostDmaProtectedWrites(0, r.m.txS) == 0u);
}

namespace {
struct I2dCall {
	const char *name;
	bool needsListen;
	bool rxStream;
	std::function<HAL_StatusTypeDef(I2dRig &)> call;
};
std::vector<I2dCall> i2dCalls() {
	return {
	    {"Master_Transmit_DMA", false, false, [](I2dRig &r) { return HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 3); }},
	    {"Master_Receive_DMA", false, true, [](I2dRig &r) { return HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, 3); }},
	    {"Slave_Transmit_DMA", false, false, [](I2dRig &r) { return HAL_I2C_Slave_Transmit_DMA(&r.h, r.buf, 3); }},
	    {"Slave_Receive_DMA", false, true, [](I2dRig &r) { return HAL_I2C_Slave_Receive_DMA(&r.h, r.buf, 3); }},
	    {"Mem_Write_DMA", false, false, [](I2dRig &r) { return HAL_I2C_Mem_Write_DMA(&r.h, i2dDev, 0x10, I2C_MEMADD_SIZE_8BIT, r.buf, 3); }},
	    {"Mem_Read_DMA", false, true, [](I2dRig &r) { return HAL_I2C_Mem_Read_DMA(&r.h, i2dDev, 0x10, I2C_MEMADD_SIZE_8BIT, r.buf, 3); }},
	    {"Master_Seq_Transmit_DMA", false, false, [](I2dRig &r) { return HAL_I2C_Master_Seq_Transmit_DMA(&r.h, i2dDev, r.buf, 3, I2C_FIRST_FRAME); }},
	    {"Master_Seq_Receive_DMA", false, true, [](I2dRig &r) { return HAL_I2C_Master_Seq_Receive_DMA(&r.h, i2dDev, r.buf, 3, I2C_FIRST_FRAME); }},
	    {"Slave_Seq_Transmit_DMA", true, false, [](I2dRig &r) { return HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 3, I2C_FIRST_FRAME); }},
	    {"Slave_Seq_Receive_DMA", true, true, [](I2dRig &r) { return HAL_I2C_Slave_Seq_Receive_DMA(&r.h, r.buf, 3, I2C_FIRST_FRAME); }},
	};
}
} // namespace

TEST_CASE_FIXTURE(I2dFx, "ID1 characterization: a DMA call whose HAL_DMA_Start_IT fails returns HAL_ERROR with HAL_I2C_ERROR_DMA, state READY, mode NONE, unlocked and no START or DMAEN") {
	/* NEEDS REVIEW 325: the stream is busy (started by the test) so that HAL_DMA_Start_IT returns HAL_BUSY. */
	for (const auto &c : i2dCalls()) {
		SUBCASE(c.name) {
			I2dRig r(1);
			r.addPeer();
			if (c.needsListen) REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
			DMA_HandleTypeDef &d = c.rxStream ? r.rx : r.tx;
			REQUIRE(HAL_DMA_Start_IT(&d, c.rxStream ? i2dDrAddress[1] : r.bufAddr(), c.rxStream ? r.bufAddr() : i2dDrAddress[1], 1) == HAL_OK);
			const uint32_t ndtr = d.Instance->NDTR;
			CHECK(c.call(r) == HAL_ERROR);
			CHECK((r.h.ErrorCode & HAL_I2C_ERROR_DMA) != 0u);
			CHECK(r.h.State == HAL_I2C_STATE_READY);
			CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
			CHECK(r.h.Lock == HAL_UNLOCKED);
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
			CHECK(d.Instance->NDTR == ndtr); /* the running stream was not touched */
			CHECK(hostDmaProtectedWrites(0, c.rxStream ? r.m.rxS : r.m.txS) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 characterization: a missing DMA handle returns HAL_ERROR with HAL_I2C_ERROR_DMA_PARAM, master and memory calls leave READY, slave calls leave LISTEN") {
	/* NEEDS REVIEW 325 */
	for (const auto &c : i2dCalls()) {
		SUBCASE(c.name) {
			I2dRig r(0, false, false);
			r.addPeer();
			if (c.needsListen) REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
			CHECK(c.call(r) == HAL_ERROR);
			CHECK((r.h.ErrorCode & HAL_I2C_ERROR_DMA_PARAM) != 0u);
			const bool slave = std::strncmp(c.name, "Slave", 5) == 0;
			CHECK(r.h.State == (slave ? HAL_I2C_STATE_LISTEN : HAL_I2C_STATE_READY));
			CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
			CHECK(r.h.Lock == HAL_UNLOCKED);
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 each DMA call assigns the complete and the error callback and clears the half, memory 1 and abort callbacks of its stream") {
	for (const auto &c : i2dCalls()) {
		SUBCASE(c.name) {
			I2dRig r(2);
			r.addPeer();
			if (c.needsListen) REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
			DMA_HandleTypeDef &d = c.rxStream ? r.rx : r.tx;
			auto dummy = [](DMA_HandleTypeDef *) {};
			d.XferHalfCpltCallback = dummy;
			d.XferM1CpltCallback = dummy;
			d.XferM1HalfCpltCallback = dummy;
			d.XferAbortCallback = dummy;
			REQUIRE(c.call(r) == HAL_OK);
			CHECK(d.XferCpltCallback != nullptr);
			CHECK(d.XferErrorCallback != nullptr);
			CHECK(d.XferHalfCpltCallback == nullptr);
			CHECK(d.XferAbortCallback == nullptr);
			if (std::strstr(c.name, "Seq") == nullptr) {
				CHECK(d.XferM1CpltCallback == nullptr); /* the Seq functions leave the memory 1 callbacks alone */
				CHECK(d.XferM1HalfCpltCallback == nullptr);
			}
		}
	}
}

/* ------------------------------------------------------------------------------------------------------------------ ID2 */

TEST_CASE_FIXTURE(I2dFx, "ID2 Master_Transmit_DMA moves Size bytes and ends with one MasterTxCplt, a STOP after the last byte, DMAEN cleared and the interrupts off") {
	for (unsigned n = 0; n < 3; ++n) {
		for (unsigned size : {1u, 2u, 5u}) {
			SUBCASE((std::string(i2dInstanceName[n]) + " size " + std::to_string(size)).c_str()) {
				I2dRig r(n);
				r.addPeer();
				r.fillTx(size);
				REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, static_cast<uint16_t>(size)) == HAL_OK);
				r.irqs(); /* SB: address, ADDR: cleared */
				r.engineTx(r.buf, size);
				r.doneTx();
				r.irqs(); /* BTF: STOP and the callback */
				std::string expect = "S o:A0";
				for (unsigned i = 0; i < size; ++i) {
					char b[8];
					std::snprintf(b, sizeof b, " o:%02X", 0x11u * (i + 1u));
					expect += b;
				}
				expect += " P";
				CHECK(i2dLog(n) == expect);
				CHECK(i2cCounts.masterTx == 1u);
				CHECK(i2cCounts.error == 0u);
				CHECK(i2cCounts.stateAtLast == HAL_I2C_STATE_READY);
				CHECK(r.h.State == HAL_I2C_STATE_READY);
				CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
				CHECK(r.h.ErrorCode == HAL_I2C_ERROR_NONE);
				CHECK(r.h.XferCount == 0u);
				CHECK((r.inst->CR2 & (i2dCR2_DMAEN | i2dCR2_ITEVTEN | i2dCR2_ITERREN | i2dCR2_ITBUFEN)) == 0u);
				CHECK(!r.enabled(r.tx));
				CHECK((r.sr2() & i2dSR2_BUSY) == 0u);
			}
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID2 the end of the Tx stream alone does not complete a master transmit: the callback waits for the BTF event") {
	I2dRig r(0);
	r.addPeer();
	r.fillTx(2);
	REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 2) == HAL_OK);
	r.irqs();
	r.engineTx(r.buf, 2);
	r.doneTx();
	/* 27.3.8: after EOT disable DMA requests, then wait for BTF before the STOP. */
	CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
	CHECK((r.inst->CR2 & (i2dCR2_ITEVTEN | i2dCR2_ITERREN)) == (i2dCR2_ITEVTEN | i2dCR2_ITERREN)); /* the HAL waits for BTF through the event interrupt */
	CHECK(i2cCounts.masterTx == 0u);
	CHECK(r.h.State == HAL_I2C_STATE_BUSY_TX);
	CHECK(i2dEnds(i2dLog(0), "o:22")); /* no STOP yet */
	r.irqs();
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(i2dEnds(i2dLog(0), "P"));
}

TEST_CASE_FIXTURE(I2dFx, "ID2 Master_Receive_DMA delivers the bytes in order, ends with one MasterRxCplt and a STOP, with LAST and DMAEN cleared") {
	for (unsigned n = 0; n < 3; ++n) {
		for (unsigned size : {1u, 2u, 3u, 5u}) {
			SUBCASE((std::string(i2dInstanceName[n]) + " size " + std::to_string(size)).c_str()) {
				I2dRig r(n);
				r.addPeer();
				r.queuePeer(8);
				REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, static_cast<uint16_t>(size)) == HAL_OK);
				r.irqs(); /* SB: address with R, ADDR */
				uint8_t got[8] = {};
				r.engineRx(got, size);
				r.doneRx();
				r.irqs();
				CHECK(std::memcmp(got, i2dPeerBytes, size) == 0);
				const std::string log = i2dLog(n);
				CHECK(i2dStarts(log, "S o:A1"));
				CHECK(i2dEnds(log, "P"));
				if (size == 1u) CHECK(log == "S o:A1 i:91! P");        /* 27.3.3: the last byte is not acked */
				if (size == 2u) CHECK(log == "S o:A1 i:91 i:82! P");
				CHECK(i2cCounts.masterRx == 1u);
				CHECK(i2cCounts.error == 0u);
				CHECK(r.h.State == HAL_I2C_STATE_READY);
				CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
				CHECK(r.h.ErrorCode == HAL_I2C_ERROR_NONE);
				CHECK((r.inst->CR2 & (i2dCR2_DMAEN | i2dCR2_LAST | i2dCR2_ITEVTEN | i2dCR2_ITERREN)) == 0u);
				CHECK(!r.enabled(r.rx));
				CHECK((r.sr2() & i2dSR2_BUSY) == 0u);
			}
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID2 Master_Receive_DMA in the ADDR handler clears ACK for one byte, and sets LAST for two or more bytes") {
	for (unsigned size : {1u, 2u, 3u, 6u}) {
		SUBCASE(("size " + std::to_string(size)).c_str()) {
			I2dRig r(0);
			r.addPeer();
			r.queuePeer(8);
			REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, static_cast<uint16_t>(size)) == HAL_OK);
			r.irqs();
			if (size == 1u) {
				CHECK((r.inst->CR1 & i2dCR1_ACK) == 0u); /* 27.3.8: ACK = 0 when ADDR = 1, before clearing ADDR */
				CHECK((r.inst->CR2 & i2dCR2_LAST) == 0u);
			} else {
				CHECK((r.inst->CR2 & i2dCR2_LAST) != 0u); /* 27.3.8: LAST for two or more bytes */
			}
			if (size == 2u) CHECK((r.inst->CR1 & i2dCR1_ACK) == 0u); /* two byte rule: ACK off with POS, the second byte gets the NACK */
			if (size >= 3u) CHECK((r.inst->CR1 & i2dCR1_ACK) != 0u);
			CHECK((r.sr1() & i2dSR1_ADDR) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID2 Mem_Write_DMA moves Size bytes and ends with one MemTxCplt and a STOP") {
	for (int wide = 0; wide < 2; ++wide) {
		SUBCASE(wide ? "16 bit address" : "8 bit address") {
			I2dRig r(1);
			r.addPeer();
			r.fillTx(3);
			REQUIRE(HAL_I2C_Mem_Write_DMA(&r.h, i2dDev, 0x0A55, wide ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT, r.buf, 3) == HAL_OK);
			r.engineTx(r.buf, 3);
			r.doneTx();
			r.irqs();
			CHECK(i2dLog(1) == (wide ? "S o:A0 o:0A o:55 o:11 o:22 o:33 P" : "S o:A0 o:55 o:11 o:22 o:33 P"));
			CHECK(i2cCounts.memTx == 1u);
			CHECK(i2cCounts.masterTx == 0u);
			CHECK(i2cCounts.error == 0u);
			CHECK(r.h.State == HAL_I2C_STATE_READY);
			CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
			CHECK((r.inst->CR2 & (i2dCR2_DMAEN | i2dCR2_ITEVTEN | i2dCR2_ITERREN)) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID2 Mem_Read_DMA delivers the bytes in order and ends with one MemRxCplt, a STOP and LAST cleared") {
	for (unsigned size : {1u, 2u, 4u}) {
		SUBCASE(("size " + std::to_string(size)).c_str()) {
			I2dRig r(2);
			r.addPeer();
			r.queuePeer(8);
			REQUIRE(HAL_I2C_Mem_Read_DMA(&r.h, i2dDev, 0x33, I2C_MEMADD_SIZE_8BIT, r.buf, static_cast<uint16_t>(size)) == HAL_OK);
			uint8_t got[8] = {};
			r.engineRx(got, size);
			r.doneRx();
			r.irqs();
			CHECK(std::memcmp(got, i2dPeerBytes, size) == 0);
			const std::string log = i2dLog(2);
			CHECK(i2dStarts(log, "S o:A0 o:33 R o:A1"));
			CHECK(i2dEnds(log, "P"));
			if (size == 1u) CHECK(log == "S o:A0 o:33 R o:A1 i:91! P");
			CHECK(i2cCounts.memRx == 1u);
			CHECK(i2cCounts.masterRx == 0u);
			CHECK(i2cCounts.error == 0u);
			CHECK(r.h.State == HAL_I2C_STATE_READY);
			CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
			CHECK((r.inst->CR2 & (i2dCR2_DMAEN | i2dCR2_LAST | i2dCR2_ITERREN)) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID2 characterization: completion of a receive with an error code set calls the error callback and not MasterRxCplt, completion after the mode was reset does nothing") {
	/* NEEDS REVIEW 325 */
	SUBCASE("error code set") {
		I2dRig r(0);
		r.addPeer();
		r.queuePeer(8);
		REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, 3) == HAL_OK);
		r.irqs();
		uint8_t got[3];
		r.engineRx(got, 3);
		r.h.ErrorCode = HAL_I2C_ERROR_BERR; /* an error recorded while the transfer ran */
		r.doneRx();
		CHECK(i2cCounts.error == 1u);
		CHECK(i2cCounts.masterRx == 0u);
		CHECK(i2cCounts.errorCodeAtError == HAL_I2C_ERROR_BERR);
	}
	SUBCASE("mode reset") {
		I2dRig r(1);
		r.addPeer();
		r.queuePeer(8);
		REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, 3) == HAL_OK);
		r.irqs();
		r.h.Mode = HAL_I2C_MODE_NONE;
		r.doneRx();
		CHECK(i2cCounts.masterRx == 0u);
		CHECK(i2cCounts.error == 0u);
		CHECK(r.h.State == HAL_I2C_STATE_BUSY_RX);
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID2 Slave_Receive_DMA: the end of the Rx stream alone calls nothing, the STOP then ends the call with one SlaveRxCplt and the bytes in order") {
	for (unsigned n = 0; n < 3; ++n) {
		for (unsigned size : {1u, 3u}) {
			SUBCASE((std::string(i2dInstanceName[n]) + " size " + std::to_string(size)).c_str()) {
				I2dRig r(n);
				REQUIRE(HAL_I2C_Slave_Receive_DMA(&r.h, r.buf, static_cast<uint16_t>(size)) == HAL_OK);
				REQUIRE(hostI2cExtStart(n, 0x30) == 1); /* own address 0x30, write */
				r.irqs();                                /* ADDR: cleared by the HAL */
				const uint8_t sent[3] = {0xC1, 0xC2, 0xC3};
				uint8_t got[3] = {};
				for (unsigned i = 0; i < size; ++i) {
					REQUIRE(hostI2cExtWrite(n, sent[i]) == 1);
					r.engineRx(&got[i], 1);
				}
				r.doneRx();
				CHECK(i2cCounts.slaveRx == 0u);
				CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
				hostI2cExtStop(n);
				r.irqs(); /* STOPF */
				CHECK(std::memcmp(got, sent, size) == 0);
				CHECK(i2cCounts.slaveRx == 1u);
				CHECK(i2cCounts.error == 0u);
				CHECK(r.h.State == HAL_I2C_STATE_READY);
				CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
				CHECK(r.h.ErrorCode == HAL_I2C_ERROR_NONE);
				CHECK((r.inst->CR1 & i2dCR1_ACK) == 0u);
				CHECK((r.inst->CR2 & (i2dCR2_ITEVTEN | i2dCR2_ITERREN)) == 0u);
			}
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID2 Slave_Transmit_DMA: after the last byte the controller NACK ends the call with one SlaveTxCplt and the bytes were sent in order") {
	for (unsigned n = 0; n < 3; ++n) {
		for (unsigned size : {1u, 3u}) {
			SUBCASE((std::string(i2dInstanceName[n]) + " size " + std::to_string(size)).c_str()) {
				I2dRig r(n);
				r.fillTx(size);
				REQUIRE(HAL_I2C_Slave_Transmit_DMA(&r.h, r.buf, static_cast<uint16_t>(size)) == HAL_OK);
				REQUIRE(hostI2cExtStart(n, 0x31) == 1); /* own address 0x30, read */
				r.irqs();
				uint8_t got[3] = {};
				for (unsigned i = 0; i < size; ++i) {
					r.engineTx(&r.buf[i], 1);
					if (i == size - 1u) r.doneTx(); /* EOT after the last DR write */
					const int ack = hostI2cExtReadByte(n, i + 1u < size ? 1 : 0, &got[i]);
					CHECK(ack == (i + 1u < size ? 1 : 0));
				}
				CHECK(i2cCounts.slaveTx == 0u); /* the NACK has not been handled yet */
				r.irqs();                       /* AF with XferCount 0 */
				CHECK(std::memcmp(got, r.buf, size) == 0);
				CHECK(i2cCounts.slaveTx == 1u);
				CHECK(i2cCounts.error == 0u);
				CHECK(r.h.State == HAL_I2C_STATE_READY);
				CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
				CHECK(r.h.ErrorCode == HAL_I2C_ERROR_NONE);
				CHECK((r.inst->CR2 & (i2dCR2_DMAEN | i2dCR2_ITEVTEN | i2dCR2_ITERREN)) == 0u);
			}
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID2 characterization: Slave_Receive_DMA stopped by the controller before Size bytes ends with HAL_I2C_ERROR_AF and the error callback") {
	/* NEEDS REVIEW 325 */
	I2dRig r(0);
	REQUIRE(HAL_I2C_Slave_Receive_DMA(&r.h, r.buf, 4) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	r.irqs();
	uint8_t got[2] = {};
	REQUIRE(hostI2cExtWrite(0, 0xC1) == 1);
	r.engineRx(&got[0], 1);
	REQUIRE(hostI2cExtWrite(0, 0xC2) == 1);
	r.engineRx(&got[1], 1);
	hostDmaStepItems(0, r.m.rxS, 2); /* the stream moved two items: NDTR 2 left */
	hostI2cExtStop(0);
	r.irqs();
	CHECK((r.h.ErrorCode & HAL_I2C_ERROR_AF) != 0u);
	CHECK(r.h.XferCount == 2u); /* the number of items the stream did not move */
	CHECK(i2cCounts.slaveRx == 0u);
	CHECK(i2cCounts.error >= 1u);
	CHECK(!r.enabled(r.rx));
	CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
}

TEST_CASE_FIXTURE(I2dFx, "ID2 characterization: Slave_Transmit_DMA with a controller NACK before the last byte stops the stream first and reports HAL_I2C_ERROR_AF afterwards") {
	/* NEEDS REVIEW 325 */
	I2dRig r(1);
	r.fillTx(4);
	REQUIRE(HAL_I2C_Slave_Transmit_DMA(&r.h, r.buf, 4) == HAL_OK);
	REQUIRE(hostI2cExtStart(1, 0x31) == 1);
	r.irqs();
	uint8_t b = 0;
	r.engineTx(&r.buf[0], 1);
	CHECK(hostI2cExtReadByte(1, 1, &b) == 1);
	r.engineTx(&r.buf[1], 1);
	CHECK(hostI2cExtReadByte(1, 0, &b) == 0); /* NACK after the second byte of four */
	r.irqs();
	CHECK((r.h.ErrorCode & HAL_I2C_ERROR_AF) != 0u);
	CHECK(!r.enabled(r.tx));          /* stream stop requested */
	CHECK(i2cCounts.error == 0u);     /* the error callback waits for the end of the DMA abort */
	CHECK(i2cCounts.slaveTx == 0u);
	HAL_DMA_IRQHandler(&r.tx);        /* the abort finishes */
	CHECK(i2cCounts.error == 1u);
	CHECK((i2cCounts.errorCodeAtError & HAL_I2C_ERROR_AF) != 0u);
	CHECK(i2cCounts.slaveTx == 0u);
	CHECK(r.h.State == HAL_I2C_STATE_READY);
	CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
}

/* ------------------------------------------------------------------------------------------------------------------ ID3 */

namespace {
enum I2dMode { I2D_MTX, I2D_MRX, I2D_STX, I2D_SRX };
const char *const i2dModeName[4] = {"master transmit", "master receive", "slave transmit", "slave receive"};

void i2dStart(I2dRig &r, I2dMode m) {
	switch (m) {
	case I2D_MTX:
		r.addPeer();
		r.fillTx(4);
		REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 4) == HAL_OK);
		break;
	case I2D_MRX:
		r.addPeer();
		r.queuePeer(8);
		REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, 4) == HAL_OK);
		break;
	case I2D_STX:
		r.fillTx(4);
		REQUIRE(HAL_I2C_Slave_Transmit_DMA(&r.h, r.buf, 4) == HAL_OK);
		break;
	default: REQUIRE(HAL_I2C_Slave_Receive_DMA(&r.h, r.buf, 4) == HAL_OK); break;
	}
}
} // namespace

TEST_CASE_FIXTURE(I2dFx, "ID3 a DMA transfer error ends the transfer with HAL_I2C_ERROR_DMA, state READY, mode NONE, ACK off, XferCount 0 and one error callback") {
	for (int m = 0; m < 4; ++m) {
		for (unsigned n = 0; n < 3; n += 2) {
			SUBCASE((std::string(i2dModeName[m]) + " " + i2dInstanceName[n]).c_str()) {
				I2dRig r(n);
				i2dStart(r, static_cast<I2dMode>(m));
				const bool rxStream = (m == I2D_MRX || m == I2D_SRX);
				DMA_HandleTypeDef &d = rxStream ? r.rx : r.tx;
				hostDmaRaise(0, rxStream ? r.m.rxS : r.m.txS, HOST_DMA_TEIF);
				HAL_DMA_IRQHandler(&d);
				CHECK(i2cCounts.error == 1u);
				CHECK((i2cCounts.errorCodeAtError & HAL_I2C_ERROR_DMA) != 0u);
				CHECK((r.h.ErrorCode & HAL_I2C_ERROR_DMA) != 0u);
				CHECK(r.h.State == HAL_I2C_STATE_READY);
				CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
				CHECK(r.h.XferCount == 0u);
				CHECK((r.inst->CR1 & i2dCR1_ACK) == 0u);
				CHECK(!r.enabled(d)); /* TEIF stops the stream (RM0090 10.3.18) */
				CHECK(r.tx.XferCpltCallback == nullptr);
				CHECK(r.rx.XferCpltCallback == nullptr);
				CHECK(i2cCounts.masterTx + i2cCounts.masterRx + i2cCounts.slaveTx + i2cCounts.slaveRx == 0u);
			}
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID3 characterization: a DMA FIFO error is ignored, state, error code and callbacks stay as they were") {
	/* NEEDS REVIEW 325: with the FIFO error interrupt enabled the stream reports FEIF and I2C_DMAError ignores it. */
	I2dRig r(0, true, true, true);
	r.addPeer();
	r.fillTx(4);
	REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 4) == HAL_OK);
	r.tx.Instance->FCR |= DMA_SxFCR_FEIE; /* HAL_DMA_Start_IT does not enable FEIE (T3 finding): the user does, as the FIFO error needs it */
	hostDmaRaise(0, r.m.txS, HOST_DMA_FEIF);
	HAL_DMA_IRQHandler(&r.tx);
	CHECK(r.tx.ErrorCode == HAL_DMA_ERROR_FE);
	CHECK(i2cCounts.error == 0u);
	CHECK(r.h.State == HAL_I2C_STATE_BUSY_TX);
	CHECK(r.h.Mode == HAL_I2C_MODE_MASTER);
	CHECK(r.h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(r.h.XferCount == 4u);
	CHECK((r.inst->CR1 & i2dCR1_ACK) != 0u);
}

namespace {
struct I2dErr {
	const char *name;
	uint32_t code;
	std::function<void(I2dRig &)> inject;
};
std::vector<I2dErr> i2dErrors() {
	return {
	    {"data byte nacked (AF)", HAL_I2C_ERROR_AF, [](I2dRig &r) { hostI2cDeviceNackDataAt(r.n, r.dev, 0); }},
	    {"arbitration lost (ARLO)", HAL_I2C_ERROR_ARLO, [](I2dRig &r) { hostI2cArbitrationLost(r.n); }},
	    {"bus error (BERR)", HAL_I2C_ERROR_BERR, [](I2dRig &r) { hostI2cRaiseSr1(r.n, i2dSR1_BERR); }},
	    {"overrun (OVR)", HAL_I2C_ERROR_OVR, [](I2dRig &r) { hostI2cRaiseSr1(r.n, i2dSR1_OVR); }},
	};
}
} // namespace

TEST_CASE_FIXTURE(I2dFx, "ID3 characterization: an I2C error during a DMA master transmit stops the stream, then reports the error once with state READY and the I2C disabled") {
	/* NEEDS REVIEW 325 */
	for (const auto &e : i2dErrors()) {
		SUBCASE(e.name) {
			I2dRig r(1);
			r.addPeer();
			r.fillTx(4);
			if (e.code == HAL_I2C_ERROR_AF) e.inject(r); /* the nack hook is set before the transfer */
			REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 4) == HAL_OK);
			r.irqs();
			r.engineTx(r.buf, 1);
			if (e.code != HAL_I2C_ERROR_AF) e.inject(r); /* ARLO, BERR, OVR in the middle of the transfer */
			r.irqs();
			CHECK((r.h.ErrorCode & e.code) != 0u);
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
			CHECK(!r.enabled(r.tx));
			CHECK((r.inst->CR2 & (i2dCR2_ITEVTEN | i2dCR2_ITERREN | i2dCR2_ITBUFEN)) == 0u);
			CHECK(i2cCounts.error == 0u); /* waits for the stream abort */
			HAL_DMA_IRQHandler(&r.tx);
			CHECK(i2cCounts.error == 1u);
			CHECK((i2cCounts.errorCodeAtError & e.code) != 0u);
			CHECK(r.h.State == HAL_I2C_STATE_READY);
			CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
			CHECK(r.h.XferCount == 0u);
			CHECK((r.inst->CR1 & i2dCR1_PE) == 0u);
			CHECK((r.inst->CR1 & i2dCR1_ACK) == 0u);
			CHECK(r.tx.XferAbortCallback == nullptr);
			CHECK(r.tx.XferCpltCallback == nullptr);
			CHECK(i2cCounts.masterTx == 0u);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID3 an address nack during a DMA master transmit sets AF, generates a STOP and ends in state READY through the DMA abort") {
	I2dRig r(0); /* nobody answers the address */
	r.fillTx(2);
	REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 2) == HAL_OK);
	r.irqs();
	CHECK((r.h.ErrorCode & HAL_I2C_ERROR_AF) != 0u);
	CHECK(i2dEnds(i2dLog(0), "P")); /* 27.3.3: software must stop after the address nack */
	CHECK(!r.enabled(r.tx));
	HAL_DMA_IRQHandler(&r.tx);
	CHECK(i2cCounts.error == 1u);
	CHECK(r.h.State == HAL_I2C_STATE_READY);
	CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
	CHECK((r.sr2() & i2dSR2_BUSY) == 0u);
}

TEST_CASE_FIXTURE(I2dFx, "ID3 characterization: an I2C error during a DMA master receive stops the Rx stream and reports the error once") {
	/* NEEDS REVIEW 325 */
	I2dRig r(2);
	r.addPeer();
	r.queuePeer(8);
	REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, 4) == HAL_OK);
	r.irqs();
	uint8_t got = 0;
	r.engineRx(&got, 1);
	hostI2cRaiseSr1(2, i2dSR1_BERR);
	r.irqs();
	CHECK(r.h.ErrorCode == HAL_I2C_ERROR_BERR);
	CHECK(!r.enabled(r.rx));
	CHECK(i2cCounts.error == 0u);
	HAL_DMA_IRQHandler(&r.rx);
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.errorCodeAtError == HAL_I2C_ERROR_BERR);
	CHECK(r.h.State == HAL_I2C_STATE_READY);
	CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
	CHECK(i2cCounts.masterRx == 0u);
}

TEST_CASE_FIXTURE(I2dFx, "ID3 characterization: an I2C error in a slave DMA transfer stops the stream and reports the error once") {
	/* NEEDS REVIEW 325 */
	for (int rx = 0; rx < 2; ++rx) {
		SUBCASE(rx ? "slave receive" : "slave transmit") {
			I2dRig r(0);
			i2dStart(r, rx ? I2D_SRX : I2D_STX);
			REQUIRE(hostI2cExtStart(0, rx ? 0x30 : 0x31) == 1);
			r.irqs();
			hostI2cRaiseSr1(0, i2dSR1_BERR);
			r.irqs();
			CHECK((r.h.ErrorCode & HAL_I2C_ERROR_BERR) != 0u);
			DMA_HandleTypeDef &d = rx ? r.rx : r.tx;
			CHECK(!r.enabled(d));
			HAL_DMA_IRQHandler(&d);
			CHECK(i2cCounts.error == 1u);
			CHECK(r.h.State == HAL_I2C_STATE_READY);
			CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID3 characterization: when the Tx stream cannot be aborted the HAL disables the I2C, sets state READY and still reports the error") {
	/* NEEDS REVIEW 325: the stream is already in the ABORT state, so HAL_DMA_Abort_IT returns HAL_ERROR. */
	I2dRig r(0);
	r.addPeer();
	r.fillTx(4);
	REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 4) == HAL_OK);
	r.irqs();
	REQUIRE(HAL_DMA_Abort_IT(&r.tx) == HAL_OK);
	hostI2cRaiseSr1(0, i2dSR1_BERR);
	r.irqs();
	CHECK(i2cCounts.error == 1u);
	CHECK((i2cCounts.errorCodeAtError & HAL_I2C_ERROR_BERR) != 0u);
	CHECK(r.h.State == HAL_I2C_STATE_READY);
	CHECK((r.inst->CR1 & i2dCR1_PE) == 0u);
	CHECK(r.tx.XferAbortCallback == nullptr);
}

TEST_CASE_FIXTURE(I2dFx, "ID3 characterization: when the Rx stream cannot be aborted the HAL stores the byte waiting in DR, disables the I2C and reports the error") {
	/* NEEDS REVIEW 325: the Rx stream ended (state READY) while DMAEN is still set, so HAL_DMA_Abort_IT returns HAL_ERROR. */
	I2dRig r(1);
	r.addPeer();
	r.queuePeer(8);
	REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, 4) == HAL_OK);
	r.irqs();
	r.waitSr1(i2dSR1_RXNE); /* a byte waits in DR */
	REQUIRE(HAL_DMA_Abort(&r.rx) == HAL_OK);
	hostI2cRaiseSr1(1, i2dSR1_BERR);
	r.irqs();
	CHECK(i2cCounts.error == 1u);
	CHECK(r.buf[0] == i2dPeerBytes[0]); /* the byte in DR went to the start of the buffer */
	CHECK(r.h.State == HAL_I2C_STATE_READY);
	CHECK((r.inst->CR1 & i2dCR1_PE) == 0u);
}

TEST_CASE_FIXTURE(I2dFx, "ID3 characterization: the DMA abort callback gives up waiting for a STOP that never completes and records HAL_I2C_ERROR_TIMEOUT") {
	/* NEEDS REVIEW 325 */
	I2dRig r(0);
	r.addPeer();
	r.fillTx(4);
	hostI2cDeviceNackDataAt(0, r.dev, 0);
	REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 4) == HAL_OK);
	r.irqs();
	r.engineTx(r.buf, 1); /* the first data byte is nacked */
	hostI2cStuck(0, 1);   /* the bus holds: the STOP the ER handler requests never completes */
	r.irqs();
	CHECK((r.inst->CR1 & i2dCR1_STOP) != 0u);
	HAL_DMA_IRQHandler(&r.tx);
	hostI2cStuck(0, 0);
	CHECK((r.h.ErrorCode & HAL_I2C_ERROR_TIMEOUT) != 0u);
	CHECK((r.h.ErrorCode & HAL_I2C_ERROR_AF) != 0u);
	CHECK(i2cCounts.error == 1u);
	CHECK(r.h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(I2dFx, "ID3 Master_Abort_IT during a DMA master transmit or receive stops the stream, sends a STOP and ends with one AbortCpltCallback and no error callback") {
	for (int rx = 0; rx < 2; ++rx) {
		for (unsigned n = 0; n < 3; ++n) {
			SUBCASE((std::string(rx ? "receive " : "transmit ") + i2dInstanceName[n]).c_str()) {
				I2dRig r(n);
				i2dStart(r, rx ? I2D_MRX : I2D_MTX);
				r.irqs();
				REQUIRE(HAL_I2C_Master_Abort_IT(&r.h, i2dDev) == HAL_OK);
				CHECK(r.h.State == HAL_I2C_STATE_ABORT);
				CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
				CHECK((r.inst->CR1 & i2dCR1_ACK) == 0u);
				CHECK(i2dEnds(i2dLog(n), "P"));
				CHECK(i2cCounts.abortCplt == 0u); /* the abort ends with the DMA abort callback */
				HAL_DMA_IRQHandler(rx ? &r.rx : &r.tx);
				CHECK(i2cCounts.abortCplt == 1u);
				CHECK(i2cCounts.error == 0u);
				CHECK(r.h.State == HAL_I2C_STATE_READY);
				CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
				CHECK(r.h.ErrorCode == HAL_I2C_ERROR_NONE);
				CHECK((r.inst->CR1 & i2dCR1_PE) == 0u);
				CHECK(r.h.XferCount == 0u);
			}
		}
	}
}

/* ------------------------------------------------------------------------------------------------------------------ ID4 */

namespace {
/* One master sequential transmit frame: the call, the address phase when the frame starts one, the data and the end of the stream. */
void i2dTxFrame(I2dRig &r, unsigned from, unsigned count, uint32_t option, bool addressPhase) {
	REQUIRE(HAL_I2C_Master_Seq_Transmit_DMA(&r.h, i2dDev, &r.buf[from], static_cast<uint16_t>(count), option) == HAL_OK);
	if (addressPhase) r.irqs();
	r.engineTx(&r.buf[from], count);
	r.doneTx();
	r.irqs();
}
/* One master sequential receive frame; the bytes land in out. */
void i2dRxFrame(I2dRig &r, uint8_t *out, unsigned count, uint32_t option, bool addressPhase) {
	REQUIRE(HAL_I2C_Master_Seq_Receive_DMA(&r.h, i2dDev, out, static_cast<uint16_t>(count), option) == HAL_OK);
	if (addressPhase) r.irqs();
	r.engineRx(out, count);
	r.doneRx();
	r.irqs();
}
} // namespace

TEST_CASE_FIXTURE(I2dFx, "ID4 Master_Seq_Transmit_DMA programs the Tx stream and starts with START, DMAEN only for the frames without an address phase") {
	struct Opt {
		uint32_t option;
		const char *name;
	};
	const Opt first[3] = {{I2C_FIRST_FRAME, "FIRST_FRAME"}, {I2C_FIRST_AND_NEXT_FRAME, "FIRST_AND_NEXT_FRAME"}, {I2C_FIRST_AND_LAST_FRAME, "FIRST_AND_LAST_FRAME"}};
	for (const auto &o : first) {
		SUBCASE(o.name) {
			I2dRig r(0);
			r.addPeer();
			REQUIRE(HAL_I2C_Master_Seq_Transmit_DMA(&r.h, i2dDev, r.buf, 3, o.option) == HAL_OK);
			CHECK(r.tx.Instance->PAR == i2dDrAddress[0]);
			CHECK(r.tx.Instance->M0AR == r.bufAddr());
			CHECK(r.tx.Instance->NDTR == 3u);
			CHECK(i2dLog(0) == "S"); /* a first frame starts the sequence */
			CHECK((r.inst->CR1 & i2dCR1_ACK) != 0u);
			CHECK((r.inst->CR2 & (i2dCR2_ITEVTEN | i2dCR2_ITERREN)) == (i2dCR2_ITEVTEN | i2dCR2_ITERREN));
			CHECK((r.inst->CR2 & i2dCR2_ITBUFEN) == 0u);
			CHECK(r.h.State == HAL_I2C_STATE_BUSY_TX);
			CHECK(r.h.Mode == HAL_I2C_MODE_MASTER);
			CHECK(r.h.XferOptions == o.option);
			CHECK(r.h.Lock == HAL_UNLOCKED);
			r.irqs(); /* address phase */
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) != 0u); /* RM0090 27.3.8: DMAEN is set before or during the ADDR event in controller mode */
			CHECK((r.sr1() & i2dSR1_ADDR) == 0u);
			CHECK(i2dStarts(i2dLog(0), "S o:A0"));
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID4 a FIRST_FRAME master transmit ends after BTF without a STOP and with one MasterTxCplt, the bus stays busy") {
	I2dRig r(1);
	r.addPeer();
	r.fillTx(2);
	i2dTxFrame(r, 0, 2, I2C_FIRST_FRAME, true);
	CHECK(i2dLog(1) == "S o:A0 o:11 o:22"); /* UM1725: start, address and data without a final stop */
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(r.h.State == HAL_I2C_STATE_READY);
	CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
	CHECK((r.sr2() & i2dSR2_BUSY) != 0u);
	CHECK((r.inst->CR2 & (i2dCR2_DMAEN | i2dCR2_ITEVTEN | i2dCR2_ITERREN)) == 0u);
}

TEST_CASE_FIXTURE(I2dFx, "ID4 FIRST_FRAME, NEXT_FRAME and LAST_FRAME master transmits make one START, the data in order, one STOP and three MasterTxCplt") {
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE(i2dInstanceName[n]) {
			I2dRig r(n);
			r.addPeer();
			r.fillTx(6);
			i2dTxFrame(r, 0, 2, I2C_FIRST_FRAME, true);
			CHECK(i2cCounts.masterTx == 1u);
			REQUIRE(HAL_I2C_Master_Seq_Transmit_DMA(&r.h, i2dDev, &r.buf[2], 2, I2C_NEXT_FRAME) == HAL_OK);
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) != 0u); /* HAL: a frame without START enables the DMA request at once */
			CHECK(i2dLog(n) == "S o:A0 o:11 o:22");     /* no new START */
			r.engineTx(&r.buf[2], 2);
			r.doneTx();
			r.irqs();
			CHECK(i2cCounts.masterTx == 2u);
			i2dTxFrame(r, 4, 2, I2C_LAST_FRAME, false);
			CHECK(i2dLog(n) == "S o:A0 o:11 o:22 o:33 o:44 o:55 o:66 P");
			CHECK(i2cCounts.masterTx == 3u);
			CHECK(i2cCounts.error == 0u);
			CHECK(r.h.State == HAL_I2C_STATE_READY);
			CHECK((r.sr2() & i2dSR2_BUSY) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID4 FIRST_AND_NEXT_FRAME then LAST_FRAME_NO_STOP master transmits and a LAST_FRAME receive make a repeated START with the read address") {
	I2dRig r(0);
	r.addPeer();
	r.queuePeer(8);
	r.fillTx(4);
	i2dTxFrame(r, 0, 2, I2C_FIRST_AND_NEXT_FRAME, true);
	REQUIRE(HAL_I2C_Master_Seq_Transmit_DMA(&r.h, i2dDev, &r.buf[2], 2, I2C_LAST_FRAME_NO_STOP) == HAL_OK);
	r.engineTx(&r.buf[2], 2);
	r.doneTx();
	r.irqs();
	CHECK(i2cCounts.masterTx == 2u);
	CHECK(i2dLog(0) == "S o:A0 o:11 o:22 o:33 o:44"); /* LAST_FRAME_NO_STOP: no STOP */
	uint8_t got[2] = {};
	i2dRxFrame(r, got, 2, I2C_LAST_FRAME, true); /* direction change: restart, address with R, stop at the end */
	CHECK(i2dLog(0) == "S o:A0 o:11 o:22 o:33 o:44 R o:A1 i:91 i:82! P");
	CHECK(std::memcmp(got, i2dPeerBytes, 2) == 0);
	CHECK(i2cCounts.masterRx == 1u);
	CHECK(r.h.State == HAL_I2C_STATE_READY);
	CHECK((r.sr2() & i2dSR2_BUSY) == 0u);
}

TEST_CASE_FIXTURE(I2dFx, "ID4 FIRST_FRAME then OTHER_AND_LAST_FRAME master transmits make a repeated START with the address before the second frame and a STOP at its end") {
	I2dRig r(2);
	r.addPeer();
	r.fillTx(2);
	i2dTxFrame(r, 0, 1, I2C_FIRST_FRAME, true);
	i2dTxFrame(r, 1, 1, I2C_OTHER_AND_LAST_FRAME, true);
	CHECK(i2dLog(2) == "S o:A0 o:11 R o:A0 o:22 P"); /* UM1725: OTHER: a restart with the slave address between the frames */
	CHECK(i2cCounts.masterTx == 2u);
	CHECK(r.h.State == HAL_I2C_STATE_READY);
	CHECK((r.sr2() & i2dSR2_BUSY) == 0u);
}

TEST_CASE_FIXTURE(I2dFx, "ID4 FIRST_AND_LAST_FRAME master transmit and receive behave like the plain DMA calls: START, data, STOP") {
	SUBCASE("transmit") {
		I2dRig r(0);
		r.addPeer();
		r.fillTx(3);
		i2dTxFrame(r, 0, 3, I2C_FIRST_AND_LAST_FRAME, true);
		CHECK(i2dLog(0) == "S o:A0 o:11 o:22 o:33 P");
		CHECK(i2cCounts.masterTx == 1u);
	}
	SUBCASE("receive") {
		I2dRig r(1);
		r.addPeer();
		r.queuePeer(8);
		uint8_t got[3] = {};
		i2dRxFrame(r, got, 3, I2C_FIRST_AND_LAST_FRAME, true);
		CHECK(i2dStarts(i2dLog(1), "S o:A1 i:91 i:82 i:73"));
		CHECK(i2dEnds(i2dLog(1), "P"));
		CHECK(std::memcmp(got, i2dPeerBytes, 3) == 0);
		CHECK(i2cCounts.masterRx == 1u);
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID4 FIRST_FRAME, NEXT_FRAME and LAST_FRAME master receives make one START, the bytes in order and one STOP") {
	I2dRig r(0);
	r.addPeer();
	r.queuePeer(8);
	uint8_t a[3] = {}, b[2] = {}, c[1] = {};
	i2dRxFrame(r, a, 3, I2C_FIRST_FRAME, true);
	CHECK(i2cCounts.masterRx == 1u);
	CHECK(r.h.State == HAL_I2C_STATE_READY);
	CHECK(i2dLog(0).find('P') == std::string::npos); /* UM1725: no final stop after FIRST_FRAME */
	i2dRxFrame(r, b, 2, I2C_NEXT_FRAME, false);
	CHECK(i2cCounts.masterRx == 2u);
	i2dRxFrame(r, c, 1, I2C_LAST_FRAME, false);
	CHECK(i2cCounts.masterRx == 3u);
	const std::string log = i2dLog(0);
	CHECK(i2dStarts(log, "S o:A1 i:91 i:82 i:73 i:64 i:55 i:46"));
	CHECK(log.find('R') == std::string::npos); /* no restart inside the sequence */
	CHECK(i2dEnds(log, "P"));
	CHECK(std::memcmp(a, &i2dPeerBytes[0], 3) == 0);
	CHECK(std::memcmp(b, &i2dPeerBytes[3], 2) == 0);
	CHECK(c[0] == i2dPeerBytes[5]);
	CHECK((r.inst->CR2 & (i2dCR2_DMAEN | i2dCR2_LAST)) == 0u);
}

TEST_CASE_FIXTURE(I2dFx, "ID4 Master_Seq_Receive_DMA sets LAST at the call for the frames that end the sequence and not for FIRST_FRAME or NEXT_FRAME") {
	struct Opt {
		uint32_t option;
		bool last;
		const char *name;
	};
	const Opt opts[3] = {{I2C_LAST_FRAME, true, "LAST_FRAME"}, {I2C_LAST_FRAME_NO_STOP, true, "LAST_FRAME_NO_STOP"}, {I2C_OTHER_AND_LAST_FRAME, true, "OTHER_AND_LAST_FRAME"}};
	for (const auto &o : opts) {
		SUBCASE(o.name) {
			I2dRig r(1);
			r.addPeer();
			r.queuePeer(8);
			REQUIRE(HAL_I2C_Master_Seq_Receive_DMA(&r.h, i2dDev, r.buf, 3, o.option) == HAL_OK);
			CHECK(((r.inst->CR2 & i2dCR2_LAST) != 0u) == o.last); /* RM0090 27.3.8: LAST makes the NACK on the last byte of this frame */
			CHECK(r.rx.Instance->NDTR == 3u);
			CHECK(r.rx.Instance->PAR == i2dDrAddress[1]);
			CHECK(((r.rx.Instance->CR & i2dDMA_DIR_MASK) >> i2dDMA_DIR_SHIFT) == 0u);
		}
	}
	SUBCASE("FIRST_FRAME") {
		I2dRig r(1);
		r.addPeer();
		REQUIRE(HAL_I2C_Master_Seq_Receive_DMA(&r.h, i2dDev, r.buf, 3, I2C_FIRST_FRAME) == HAL_OK);
		CHECK((r.inst->CR2 & i2dCR2_LAST) == 0u);
		CHECK((r.inst->CR1 & i2dCR1_ACK) != 0u);
		CHECK(i2dLog(1) == "S");
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID4 characterization: a FIRST_FRAME master receive of 3 or more bytes has LAST set by the ADDR handler") {
	/* NEEDS REVIEW 325: LAST makes the last byte of the FIRST_FRAME a NACK although the sequence goes on (RM0090 27.6.2 LAST). */
	I2dRig r(0);
	r.addPeer();
	r.queuePeer(8);
	REQUIRE(HAL_I2C_Master_Seq_Receive_DMA(&r.h, i2dDev, r.buf, 3, I2C_FIRST_FRAME) == HAL_OK);
	r.irqs();
	CHECK((r.inst->CR2 & i2dCR2_LAST) != 0u);
	CHECK((r.inst->CR2 & i2dCR2_DMAEN) != 0u); /* set in the SB handler: a frame with an address phase enables the request after the START */
}

TEST_CASE_FIXTURE(I2dFx, "ID4 Master_Seq_Receive_DMA of two bytes with LAST_FRAME after a receive keeps ACK off with POS and LAST set, after a transmit it keeps ACK on") {
	for (int afterRx = 0; afterRx < 2; ++afterRx) {
		SUBCASE(afterRx ? "after a receive frame" : "first frame") {
			I2dRig r(0);
			r.addPeer();
			r.queuePeer(8);
			uint8_t first[1] = {};
			if (afterRx) {
				i2dRxFrame(r, first, 1, I2C_FIRST_AND_NEXT_FRAME, true);
				REQUIRE(r.h.PreviousState != 0u);
			}
			REQUIRE(HAL_I2C_Master_Seq_Receive_DMA(&r.h, i2dDev, r.buf, 2, afterRx ? I2C_LAST_FRAME : I2C_LAST_FRAME_NO_STOP) == HAL_OK);
			if (afterRx) {
				CHECK((r.inst->CR1 & i2dCR1_ACK) == 0u); /* the second byte is the last: NACK through ACK = 0 with POS = 1 */
				CHECK((r.inst->CR1 & i2dCR1_POS) != 0u);
				CHECK((r.inst->CR2 & i2dCR2_LAST) != 0u);
			} else {
				CHECK((r.inst->CR1 & i2dCR1_ACK) != 0u);
			}
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID4 Slave_Seq_Receive_DMA after an address match: AddrCallback, bytes in order, SlaveRxCplt at the end of the stream and ListenCplt at the STOP") {
	for (unsigned n = 0; n < 3; n += 2) {
		SUBCASE(i2dInstanceName[n]) {
			I2dRig r(n);
			REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
			REQUIRE(hostI2cExtStart(n, 0x30) == 1); /* write: the controller transmits */
			HAL_I2C_EV_IRQHandler(&r.h);            /* one call: ADDR stays set until the Seq function clears it */
			CHECK(i2cCounts.addr == 1u);
			CHECK(i2cCounts.addrDirection == I2C_DIRECTION_TRANSMIT); /* UM1725: direction as seen by the controller */
			CHECK(i2cCounts.addrMatchCode == 0x30u);
			REQUIRE(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, r.buf, 3, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
			CHECK(r.h.State == HAL_I2C_STATE_BUSY_RX_LISTEN);
			CHECK(r.h.Mode == HAL_I2C_MODE_SLAVE);
			CHECK(r.rx.Instance->PAR == i2dDrAddress[n]);
			CHECK(r.rx.Instance->NDTR == 3u);
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) != 0u);
			CHECK((r.inst->CR1 & i2dCR1_ACK) != 0u);
			CHECK((r.sr1() & i2dSR1_ADDR) == 0u);
			const uint8_t sent[3] = {0xD1, 0xD2, 0xD3};
			uint8_t got[3] = {};
			for (unsigned i = 0; i < 3; ++i) {
				REQUIRE(hostI2cExtWrite(n, sent[i]) == 1);
				r.engineRx(&got[i], 1);
			}
			r.doneRx();
			CHECK(std::memcmp(got, sent, 3) == 0);
			CHECK(i2cCounts.slaveRx == 1u);
			CHECK(r.h.State == HAL_I2C_STATE_LISTEN); /* the listen state is kept after the frame */
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
			hostI2cExtStop(n);
			r.irqs();
			CHECK(i2cCounts.listen == 1u);
			CHECK(i2cCounts.slaveRx == 1u);
			CHECK(i2cCounts.error == 0u);
			CHECK(r.h.State == HAL_I2C_STATE_READY);
			CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID4 Slave_Seq_Transmit_DMA after an address match: AddrCallback, bytes in order, SlaveTxCplt at the end of the stream and ListenCplt at the NACK of a LAST_FRAME") {
	for (unsigned n = 0; n < 3; n += 2) {
		SUBCASE(i2dInstanceName[n]) {
			I2dRig r(n);
			r.fillTx(3);
			REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
			REQUIRE(hostI2cExtStart(n, 0x31) == 1); /* read: the controller receives */
			HAL_I2C_EV_IRQHandler(&r.h);
			CHECK(i2cCounts.addr == 1u);
			CHECK(i2cCounts.addrDirection == I2C_DIRECTION_RECEIVE);
			CHECK(i2cCounts.addrMatchCode == 0x30u);
			REQUIRE(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 3, I2C_LAST_FRAME) == HAL_OK);
			CHECK(r.h.State == HAL_I2C_STATE_BUSY_TX_LISTEN);
			CHECK(r.tx.Instance->PAR == i2dDrAddress[n]);
			CHECK(r.tx.Instance->NDTR == 3u);
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) != 0u);
			uint8_t got[3] = {};
			for (unsigned i = 0; i < 3; ++i) {
				r.engineTx(&r.buf[i], 1);
				if (i == 2u) r.doneTx();
				CHECK(hostI2cExtReadByte(n, i < 2u ? 1 : 0, &got[i]) == (i < 2u ? 1 : 0));
			}
			CHECK(std::memcmp(got, r.buf, 3) == 0);
			CHECK(i2cCounts.slaveTx == 1u);
			CHECK(r.h.State == HAL_I2C_STATE_LISTEN);
			CHECK(i2cCounts.listen == 0u);
			r.irqs(); /* the NACK: AF in the listen state after a LAST_FRAME */
			CHECK(i2cCounts.listen == 1u);
			CHECK(i2cCounts.slaveTx == 1u);
			CHECK(r.h.State == HAL_I2C_STATE_READY);
			CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID4 Slave_Seq_Transmit_DMA and Slave_Seq_Receive_DMA return HAL_ERROR for a NULL buffer or Size 0 and leave the listen state") {
	I2dRig r(0);
	REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
	CHECK(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, nullptr, 3, I2C_FIRST_FRAME) == HAL_ERROR);
	CHECK(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 0, I2C_FIRST_FRAME) == HAL_ERROR);
	CHECK(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, nullptr, 3, I2C_FIRST_FRAME) == HAL_ERROR);
	CHECK(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, r.buf, 0, I2C_FIRST_FRAME) == HAL_ERROR);
	CHECK(r.h.State == HAL_I2C_STATE_LISTEN);
	CHECK(!r.enabled(r.tx));
	CHECK(!r.enabled(r.rx));
	CHECK(r.h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(I2dFx, "ID4 characterization: switching a slave sequential receive to a transmit stops the Rx stream, starts the Tx stream and reports the error after the abort") {
	/* NEEDS REVIEW 325: the HAL comment says the abort "will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure". */
	I2dRig r(1);
	r.fillTx(2);
	REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
	REQUIRE(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, &r.buf[8], 3, I2C_FIRST_FRAME) == HAL_OK);
	REQUIRE(r.enabled(r.rx));
	REQUIRE(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 2, I2C_FIRST_FRAME) == HAL_OK);
	CHECK(!r.enabled(r.rx));
	CHECK(r.enabled(r.tx));
	CHECK(r.h.State == HAL_I2C_STATE_BUSY_TX_LISTEN);
	CHECK(r.tx.Instance->NDTR == 2u);
	CHECK(i2cCounts.error == 0u);
	HAL_DMA_IRQHandler(&r.rx); /* the abort of the Rx stream finishes */
	CHECK(i2cCounts.error == 1u);
}

TEST_CASE_FIXTURE(I2dFx, "ID4 characterization: switching a slave sequential transmit to a receive stops the Tx stream, starts the Rx stream and reports the error after the abort") {
	/* NEEDS REVIEW 325 */
	I2dRig r(1);
	REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
	REQUIRE(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 2, I2C_FIRST_FRAME) == HAL_OK);
	REQUIRE(r.enabled(r.tx));
	REQUIRE(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, &r.buf[8], 3, I2C_FIRST_FRAME) == HAL_OK);
	CHECK(!r.enabled(r.tx));
	CHECK(r.enabled(r.rx));
	CHECK(r.h.State == HAL_I2C_STATE_BUSY_RX_LISTEN);
	CHECK(r.rx.Instance->NDTR == 3u);
	HAL_DMA_IRQHandler(&r.tx);
	CHECK(i2cCounts.error == 1u);
}

TEST_CASE_FIXTURE(I2dFx, "ID4 characterization: a slave sequential switch with a stream that cannot be aborted calls the abort callback directly") {
	/* NEEDS REVIEW 325: the Rx stream already ended (state READY) while DMAEN is set, so HAL_DMA_Abort_IT returns HAL_ERROR. */
	I2dRig r(0);
	REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
	REQUIRE(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, &r.buf[8], 3, I2C_FIRST_FRAME) == HAL_OK);
	REQUIRE(HAL_DMA_Abort(&r.rx) == HAL_OK);
	r.fillTx(2);
	REQUIRE(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 2, I2C_FIRST_FRAME) == HAL_OK);
	CHECK(i2cCounts.error == 1u); /* I2C_DMAAbort ran at once */
	CHECK(r.enabled(r.tx));
}

/* ------------------------------------------------------------------------------------------------------------------ more ID1 */

TEST_CASE_FIXTURE(I2dFx, "ID1 characterization: a DMA call with the bus held busy returns HAL_BUSY with HAL_I2C_ERROR_TIMEOUT, state READY, mode NONE and starts nothing") {
	/* NEEDS REVIEW 325 */
	const auto calls = i2dCalls();
	for (unsigned idx : {0u, 1u, 4u, 5u, 6u, 7u}) {
		SUBCASE(calls[idx].name) {
			I2dRig r(0);
			r.addPeer();
			hostI2cBusBusy(0, 1);
			CHECK(calls[idx].call(r) == HAL_BUSY);
			hostI2cBusBusy(0, 0);
			CHECK((r.h.ErrorCode & HAL_I2C_ERROR_TIMEOUT) != 0u);
			CHECK(r.h.State == HAL_I2C_STATE_READY);
			CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
			CHECK(r.h.Lock == HAL_UNLOCKED);
			CHECK(!r.enabled(r.tx));
			CHECK(!r.enabled(r.rx));
			CHECK(i2dLog(0).empty());
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 every DMA call enables the I2C peripheral (PE) when it is disabled") {
	for (const auto &c : i2dCalls()) {
		SUBCASE(c.name) {
			I2dRig r(2);
			r.addPeer();
			r.queuePeer(4);
			if (c.needsListen) REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
			r.inst->CR1 &= ~i2dCR1_PE;
			CHECK(c.call(r) == HAL_OK);
			CHECK((r.inst->CR1 & i2dCR1_PE) != 0u); /* RM0090 27.6.1: PE enables the peripheral; a transfer needs it */
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 characterization: Mem_Read_DMA with Size 0 to an address that nacks returns HAL_ERROR with AF") {
	/* NEEDS REVIEW 325 */
	I2dRig r(1);
	CHECK(HAL_I2C_Mem_Read_DMA(&r.h, i2dDev, 0x10, I2C_MEMADD_SIZE_8BIT, r.buf, 0) == HAL_ERROR);
	CHECK((r.h.ErrorCode & HAL_I2C_ERROR_AF) != 0u);
	CHECK(r.h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(I2dFx, "ID2 characterization: Size 0 DMA calls start no stream and the address phase ends with a STOP") {
	/* NEEDS REVIEW 325 / NEEDS REVIEW 325: S, address, P; no callback for the receive. */
	SUBCASE("Master_Transmit_DMA") {
		I2dRig r(0);
		r.addPeer();
		REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 0) == HAL_OK);
		r.irqs();
		CHECK(i2dLog(0) == "S o:A0 P");
		CHECK(i2cCounts.masterTx == 1u);
		CHECK(r.h.State == HAL_I2C_STATE_READY);
	}
	SUBCASE("Master_Seq_Transmit_DMA FIRST_AND_LAST_FRAME") {
		I2dRig r(1);
		r.addPeer();
		REQUIRE(HAL_I2C_Master_Seq_Transmit_DMA(&r.h, i2dDev, r.buf, 0, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
		CHECK((r.inst->CR2 & (i2dCR2_ITEVTEN | i2dCR2_ITBUFEN | i2dCR2_ITERREN)) == (i2dCR2_ITEVTEN | i2dCR2_ITBUFEN | i2dCR2_ITERREN));
		CHECK(!r.enabled(r.tx));
		r.irqs();
		CHECK(i2dLog(1) == "S o:A0 P");
		CHECK(i2cCounts.masterTx == 1u);
	}
	SUBCASE("Master_Receive_DMA") {
		I2dRig r(2);
		r.addPeer();
		REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, 0) == HAL_OK);
		r.irqs();
		CHECK(i2dStarts(i2dLog(2), "S o:A1"));
		CHECK(i2dEnds(i2dLog(2), "P"));
		CHECK(!r.enabled(r.rx));
	}
	SUBCASE("Master_Seq_Receive_DMA FIRST_AND_LAST_FRAME") {
		I2dRig r(0);
		r.addPeer();
		REQUIRE(HAL_I2C_Master_Seq_Receive_DMA(&r.h, i2dDev, r.buf, 0, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
		CHECK((r.inst->CR2 & (i2dCR2_ITEVTEN | i2dCR2_ITBUFEN | i2dCR2_ITERREN)) == (i2dCR2_ITEVTEN | i2dCR2_ITBUFEN | i2dCR2_ITERREN));
		r.irqs();
		CHECK(i2dStarts(i2dLog(0), "S o:A1"));
		CHECK(i2dEnds(i2dLog(0), "P"));
	}
}

/* ------------------------------------------------------------------------------------------------------------------ more ID2 (10 bit) */

TEST_CASE_FIXTURE(I2dFx, "ID2 10 bit addressing: Master_Transmit_DMA sends the header 11110xx0, the second address byte, the data and a STOP") {
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE(i2dInstanceName[n]) {
			I2dRig r(n, true, true, false, true);
			r.addPeer10();
			r.fillTx(2);
			/* 10 bit address 0x2B7: bits 9:8 = 10b -> header 11110 10 0 = 0xF4 (write); the second byte is the low 8 bits 0xB7. */
			REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, 0x2B7, r.buf, 2) == HAL_OK);
			r.irqs();
			r.engineTx(r.buf, 2);
			r.doneTx();
			r.irqs();
			CHECK(i2dLog(n) == "S o:F4 o:B7 o:11 o:22 P");
			CHECK(i2cCounts.masterTx == 1u);
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID2 10 bit addressing: Master_Receive_DMA sends the write header and the address, a repeated START and the read header 11110xx1, then receives") {
	for (unsigned n = 0; n < 3; ++n) {
		SUBCASE(i2dInstanceName[n]) {
			I2dRig r(n, true, true, false, true);
			r.addPeer10();
			r.queuePeer(8);
			REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, 0x2B7, r.buf, 2) == HAL_OK);
			r.irqs();
			uint8_t got[2] = {};
			r.engineRx(got, 2);
			r.doneRx();
			r.irqs();
			CHECK(i2dLog(n) == "S o:F4 o:B7 R o:F5 i:91 i:82! P");
			CHECK(std::memcmp(got, i2dPeerBytes, 2) == 0);
			CHECK(i2cCounts.masterRx == 1u);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID4 characterization: Master_Seq_Receive_DMA in the ADDR handler: FIRST_FRAME of one byte clears ACK, FIRST_AND_LAST_FRAME of one byte clears ACK and requests the STOP, FIRST_AND_NEXT_FRAME keeps ACK on") {
	/* NEEDS REVIEW 325 for the FIRST_FRAME and FIRST_AND_NEXT_FRAME rows (the UM1725 option text does not say how the last byte of a frame is answered). */
	struct Row {
		uint32_t option;
		unsigned size;
		bool ackOn;
		bool stop;
		const char *name;
	};
	const Row rows[5] = {
	    {I2C_FIRST_FRAME, 1, false, false, "FIRST_FRAME 1 byte"},
	    {I2C_FIRST_AND_LAST_FRAME, 1, false, true, "FIRST_AND_LAST_FRAME 1 byte"},
	    {I2C_FIRST_AND_NEXT_FRAME, 1, true, false, "FIRST_AND_NEXT_FRAME 1 byte"},
	    {I2C_FIRST_AND_NEXT_FRAME, 2, true, false, "FIRST_AND_NEXT_FRAME 2 bytes"},
	    {I2C_FIRST_AND_NEXT_FRAME, 3, true, false, "FIRST_AND_NEXT_FRAME 3 bytes"},
	};
	for (const auto &row : rows) {
		SUBCASE(row.name) {
			I2dRig r(1);
			r.addPeer();
			r.queuePeer(8);
			REQUIRE(HAL_I2C_Master_Seq_Receive_DMA(&r.h, i2dDev, r.buf, static_cast<uint16_t>(row.size), row.option) == HAL_OK);
			r.irqs();
			CHECK(((r.inst->CR1 & i2dCR1_ACK) != 0u) == row.ackOn);
			CHECK(i2dEnds(i2dLog(1), "P") == row.stop);
			CHECK((r.sr1() & i2dSR1_ADDR) == 0u);
			CHECK(r.h.State == HAL_I2C_STATE_BUSY_RX);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID4 the event handler of a OTHER frame ignores every event until the START bit is seen") {
	I2dRig r(0);
	r.addPeer();
	r.fillTx(1);
	REQUIRE(HAL_I2C_Master_Seq_Transmit_DMA(&r.h, i2dDev, r.buf, 1, I2C_FIRST_FRAME) == HAL_OK);
	r.irqs();
	r.engineTx(r.buf, 1);
	r.doneTx();
	r.irqs();
	REQUIRE(i2cCounts.masterTx == 1u);
	REQUIRE(HAL_I2C_Master_Seq_Transmit_DMA(&r.h, i2dDev, r.buf, 1, I2C_OTHER_FRAME) == HAL_OK);
	(void)r.inst->SR1;     /* RM0090 27.3.3 EV5: read SR1 with SB set, then write DR: this clears SB and sends the address */
	r.inst->DR = 0xA0;
	REQUIRE((r.sr1() & i2dSR1_ADDR) != 0u);
	HAL_I2C_EV_IRQHandler(&r.h);
	CHECK((r.sr1() & i2dSR1_ADDR) != 0u); /* not handled: the handler returned before looking at ADDR */
	CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
	CHECK(i2cCounts.masterTx == 1u);
}

/* ------------------------------------------------------------------------------------------------------------------ more ID3 */

TEST_CASE_FIXTURE(I2dFx, "ID3 characterization: Master_Abort_IT after the Tx stream ended and before the BTF event ends at once with AbortCpltCallback") {
	/* NEEDS REVIEW 325: no DMA runs any more (DMAEN cleared by the end of the stream), so the abort needs no DMA abort callback. */
	I2dRig r(0);
	r.addPeer();
	r.fillTx(2);
	REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 2) == HAL_OK);
	r.irqs();
	r.engineTx(r.buf, 2);
	r.doneTx();
	REQUIRE(HAL_I2C_Master_Abort_IT(&r.h, i2dDev) == HAL_OK);
	CHECK(i2cCounts.abortCplt == 1u);
	CHECK(i2cCounts.error == 0u);
	CHECK(r.h.State == HAL_I2C_STATE_READY);
	CHECK(r.h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK((r.inst->CR1 & i2dCR1_PE) == 0u);
	CHECK(i2dEnds(i2dLog(0), "P"));
}

TEST_CASE_FIXTURE(I2dFx, "ID3 characterization: with DMAEN cleared and a byte waiting in DR, Master_Abort_IT and an I2C error both store that byte at the start of the buffer") {
	/* NEEDS REVIEW 325 */
	SUBCASE("abort") {
		I2dRig r(1);
		r.addPeer();
		r.queuePeer(8);
		REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, 4) == HAL_OK);
		r.irqs();
		r.waitSr1(i2dSR1_RXNE);
		r.inst->CR2 &= ~i2dCR2_DMAEN;
		REQUIRE(HAL_I2C_Master_Abort_IT(&r.h, i2dDev) == HAL_OK);
		CHECK(r.buf[0] == i2dPeerBytes[0]);
		CHECK(i2cCounts.abortCplt == 1u);
		CHECK(r.h.State == HAL_I2C_STATE_READY);
	}
	SUBCASE("bus error") {
		I2dRig r(1);
		r.addPeer();
		r.queuePeer(8);
		REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, 4) == HAL_OK);
		r.irqs();
		r.waitSr1(i2dSR1_RXNE);
		r.inst->CR2 &= ~i2dCR2_DMAEN;
		hostI2cRaiseSr1(1, i2dSR1_BERR);
		r.irqs();
		CHECK(r.buf[0] == i2dPeerBytes[0]);
		CHECK(i2cCounts.error == 1u); /* no DMA abort pending: the callback runs at once */
		CHECK((i2cCounts.errorCodeAtError & HAL_I2C_ERROR_BERR) != 0u);
		CHECK(r.h.State == HAL_I2C_STATE_READY);
		CHECK(r.h.Mode == HAL_I2C_MODE_NONE);
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID3 characterization: an I2C error during a slave sequential DMA transfer keeps the listen state, the error is reported when the DMA abort ends") {
	/* NEEDS REVIEW 325 */
	for (int rx = 0; rx < 2; ++rx) {
		SUBCASE(rx ? "receive" : "transmit") {
			I2dRig r(0);
			r.fillTx(3);
			REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
			REQUIRE(hostI2cExtStart(0, rx ? 0x30 : 0x31) == 1);
			HAL_I2C_EV_IRQHandler(&r.h);
			if (rx) {
				REQUIRE(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, r.buf, 3, I2C_FIRST_FRAME) == HAL_OK);
			} else {
				REQUIRE(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 3, I2C_FIRST_FRAME) == HAL_OK);
			}
			hostI2cRaiseSr1(0, i2dSR1_BERR);
			r.irqs();
			CHECK((r.h.ErrorCode & HAL_I2C_ERROR_BERR) != 0u);
			CHECK(r.h.State == HAL_I2C_STATE_LISTEN);
			CHECK(i2cCounts.error == 0u);
			DMA_HandleTypeDef &d = rx ? r.rx : r.tx;
			CHECK(!r.enabled(d));
			HAL_DMA_IRQHandler(&d);
			CHECK(i2cCounts.error == 1u);
			CHECK(r.h.State == HAL_I2C_STATE_LISTEN);
			CHECK((r.inst->CR1 & i2dCR1_PE) != 0u);  /* the abort callback enabled the peripheral again */
			CHECK((r.inst->CR1 & i2dCR1_ACK) != 0u); /* and the address acknowledge */
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID3 characterization: a controller NACK before the end of a slave sequential DMA transmit ends the listen phase and reports AF after the DMA abort") {
	/* NEEDS REVIEW 325 */
	I2dRig r(2);
	r.fillTx(4);
	REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
	REQUIRE(hostI2cExtStart(2, 0x31) == 1);
	HAL_I2C_EV_IRQHandler(&r.h);
	REQUIRE(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 4, I2C_FIRST_FRAME) == HAL_OK);
	uint8_t b = 0;
	r.engineTx(&r.buf[0], 1);
	CHECK(hostI2cExtReadByte(2, 0, &b) == 0); /* NACK after the first byte */
	r.irqs();
	CHECK((r.h.ErrorCode & HAL_I2C_ERROR_AF) != 0u);
	CHECK(i2cCounts.listen == 1u); /* the listen phase ended */
	CHECK(r.h.State == HAL_I2C_STATE_READY);
	CHECK(i2cCounts.error == 0u);
	HAL_DMA_IRQHandler(&r.tx);
	CHECK(i2cCounts.error == 1u);
	CHECK((i2cCounts.errorCodeAtError & HAL_I2C_ERROR_AF) != 0u);
}

TEST_CASE_FIXTURE(I2dFx, "ID4 a controller NACK after a FIRST_FRAME slave sequential DMA transmit only clears AF and keeps the listen state") {
	I2dRig r(0);
	r.fillTx(2);
	REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x31) == 1);
	HAL_I2C_EV_IRQHandler(&r.h);
	REQUIRE(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 2, I2C_FIRST_FRAME) == HAL_OK);
	uint8_t b = 0;
	r.engineTx(&r.buf[0], 1);
	CHECK(hostI2cExtReadByte(0, 1, &b) == 1);
	r.engineTx(&r.buf[1], 1);
	r.doneTx();
	CHECK(i2cCounts.slaveTx == 1u);
	CHECK(hostI2cExtReadByte(0, 0, &b) == 0);
	r.irqs();
	CHECK(i2cCounts.listen == 0u);
	CHECK(r.h.State == HAL_I2C_STATE_LISTEN);
	CHECK((r.sr1() & (1u << 10)) == 0u); /* AF cleared */
	CHECK(r.h.ErrorCode == HAL_I2C_ERROR_NONE);
}

/* ------------------------------------------------------------------------------------------------------------------ more slave */

TEST_CASE_FIXTURE(I2dFx, "ID2 characterization: Slave_Transmit_DMA stopped by the controller before the end reports AF through the DMA abort") {
	/* NEEDS REVIEW 325 */
	I2dRig r(0);
	r.fillTx(4);
	REQUIRE(HAL_I2C_Slave_Transmit_DMA(&r.h, r.buf, 4) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x31) == 1);
	r.irqs();
	uint8_t b = 0;
	r.engineTx(&r.buf[0], 1);
	CHECK(hostI2cExtReadByte(0, 1, &b) == 1);
	hostDmaStepItems(0, r.m.txS, 1); /* the stream moved one item */
	hostI2cExtStop(0);
	r.irqs();
	CHECK((r.h.ErrorCode & HAL_I2C_ERROR_AF) != 0u);
	CHECK(r.h.XferCount == 3u);
	CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
	CHECK(!r.enabled(r.tx));
	CHECK(i2cCounts.slaveTx == 0u);
	HAL_DMA_IRQHandler(&r.tx);
	CHECK(i2cCounts.error >= 1u);
	CHECK(r.h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(I2dFx, "ID2 characterization: a STOP while the stream is already being aborted calls the abort callback directly") {
	/* NEEDS REVIEW 325 */
	for (int rx = 0; rx < 2; ++rx) {
		SUBCASE(rx ? "receive" : "transmit") {
			I2dRig r(1);
			r.fillTx(4);
			if (rx) {
				REQUIRE(HAL_I2C_Slave_Receive_DMA(&r.h, r.buf, 4) == HAL_OK);
				REQUIRE(hostI2cExtStart(1, 0x30) == 1);
			} else {
				REQUIRE(HAL_I2C_Slave_Transmit_DMA(&r.h, r.buf, 4) == HAL_OK);
				REQUIRE(hostI2cExtStart(1, 0x31) == 1);
			}
			r.irqs();
			DMA_HandleTypeDef &d = rx ? r.rx : r.tx;
			REQUIRE(HAL_DMA_Abort_IT(&d) == HAL_OK); /* the stream is in the ABORT state: HAL_DMA_Abort_IT fails the second time */
			hostI2cExtStop(1);
			r.irqs();
			CHECK((r.h.ErrorCode & HAL_I2C_ERROR_AF) != 0u);
			CHECK(i2cCounts.error >= 1u);
			CHECK(r.h.State == HAL_I2C_STATE_READY);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID2 characterization: a STOP with a byte left in DR stores the byte and counts it") {
	/* NEEDS REVIEW 325: STOPF with the DMA still running: the HAL reads DR for a byte the stream did not take. */
	I2dRig r(2);
	REQUIRE(HAL_I2C_Slave_Receive_DMA(&r.h, r.buf, 4) == HAL_OK);
	REQUIRE(hostI2cExtStart(2, 0x30) == 1);
	r.irqs();
	REQUIRE(hostI2cExtWrite(2, 0xC1) == 1); /* RXNE = 1, nobody reads it */
	REQUIRE((r.sr1() & i2dSR1_RXNE) != 0u);
	hostI2cExtStop(2);
	r.irqs();
	CHECK(r.buf[0] == 0xC1);
	CHECK(r.h.XferCount == 3u); /* 4 items left in the stream, one more taken from DR */
	CHECK((r.h.ErrorCode & HAL_I2C_ERROR_AF) != 0u);
}

TEST_CASE_FIXTURE(I2dFx, "ID2 characterization: a STOP with BTF set stores the byte of the data register and the byte of the shift register") {
	/* NEEDS REVIEW 325: two bytes arrive and nobody reads: the second one waits in the shift register with BTF = 1 and the SCL held. */
	I2dRig r(0);
	REQUIRE(HAL_I2C_Slave_Receive_DMA(&r.h, r.buf, 4) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	r.irqs();
	REQUIRE(hostI2cExtWrite(0, 0xC1) == 1);
	const int second = hostI2cExtWrite(0, 0xC2);
	if (second != 1) {
		FAIL("the model does not accept a second byte while RXNE = 1: report to the lead");
	}
	hostI2cExtStop(0);
	r.irqs();
	CHECK(r.buf[0] == 0xC1);
	CHECK(r.buf[1] == 0xC2);
	CHECK(r.h.XferCount == 2u);
}

TEST_CASE_FIXTURE(I2dFx, "ID4 characterization: a STOP right after the last byte but before the DMA transfer complete interrupt ends the listen phase with SlaveRxCplt then ListenCplt") {
	/* NEEDS REVIEW 325 */
	I2dRig r(0);
	REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	HAL_I2C_EV_IRQHandler(&r.h);
	REQUIRE(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, r.buf, 2, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	uint8_t got[2] = {};
	for (unsigned i = 0; i < 2; ++i) {
		REQUIRE(hostI2cExtWrite(0, static_cast<uint8_t>(0xD1 + i)) == 1);
		r.engineRx(&got[i], 1);
	}
	hostDmaStepItems(0, r.m.rxS, 2); /* the stream is complete (NDTR 0) but its interrupt was not served yet */
	hostI2cExtStop(0);
	r.irqs();
	CHECK(r.h.XferCount == 0u);
	CHECK(i2cCounts.slaveRx == 1u);
	CHECK(i2cCounts.listen == 1u);
	CHECK(r.h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(r.h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(I2dFx, "ID4 characterization: a slave sequential call of the same direction while its stream aborts returns HAL_ERROR with HAL_I2C_ERROR_DMA") {
	/* NEEDS REVIEW 325: the stream is still in the ABORT state, so HAL_DMA_Start_IT cannot start it. */
	for (int rx = 0; rx < 2; ++rx) {
		SUBCASE(rx ? "receive" : "transmit") {
			I2dRig r(1);
			r.fillTx(3);
			REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
			if (rx) {
				REQUIRE(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, &r.buf[8], 3, I2C_FIRST_FRAME) == HAL_OK);
				CHECK(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, &r.buf[8], 3, I2C_NEXT_FRAME) == HAL_ERROR);
			} else {
				REQUIRE(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 3, I2C_FIRST_FRAME) == HAL_OK);
				CHECK(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 3, I2C_NEXT_FRAME) == HAL_ERROR);
			}
			CHECK((r.h.ErrorCode & HAL_I2C_ERROR_DMA) != 0u);
			CHECK(r.h.State == HAL_I2C_STATE_READY);
			CHECK(r.h.Lock == HAL_UNLOCKED);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID4 characterization: a slave sequential call with a stream that cannot be aborted calls the abort callback at once and starts the new stream") {
	/* NEEDS REVIEW 325: first call, stream stopped by the test (state READY, DMAEN still set), second call in each direction pair. */
	for (int firstRx = 0; firstRx < 2; ++firstRx) {
		for (int secondRx = 0; secondRx < 2; ++secondRx) {
			SUBCASE((std::string(firstRx ? "receive" : "transmit") + " then " + (secondRx ? "receive" : "transmit")).c_str()) {
				I2dRig r(2);
				r.fillTx(3);
				REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
				if (firstRx) {
					REQUIRE(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, &r.buf[8], 3, I2C_FIRST_FRAME) == HAL_OK);
					REQUIRE(HAL_DMA_Abort(&r.rx) == HAL_OK);
				} else {
					REQUIRE(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 3, I2C_FIRST_FRAME) == HAL_OK);
					REQUIRE(HAL_DMA_Abort(&r.tx) == HAL_OK);
				}
				if (secondRx) {
					REQUIRE(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, &r.buf[8], 3, I2C_NEXT_FRAME) == HAL_OK);
					CHECK(r.enabled(r.rx));
				} else {
					REQUIRE(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 3, I2C_NEXT_FRAME) == HAL_OK);
					CHECK(r.enabled(r.tx));
				}
				CHECK(i2cCounts.error == 1u);
			}
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID3 characterization: after an ignored DMA FIFO error the end of the stream does not complete the transfer") {
	/* NEEDS REVIEW 325 */
	I2dRig r(0, true, true, true);
	r.addPeer();
	r.fillTx(2);
	REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 2) == HAL_OK);
	r.tx.Instance->FCR |= DMA_SxFCR_FEIE;
	r.irqs();
	r.engineTx(r.buf, 2);
	hostDmaRaise(0, r.m.txS, HOST_DMA_FEIF);
	HAL_DMA_IRQHandler(&r.tx);
	r.doneTx();
	r.irqs();
	CHECK(r.tx.XferCpltCallback == nullptr);
	CHECK(i2cCounts.masterTx == 0u);
	CHECK(r.h.State == HAL_I2C_STATE_BUSY_TX);
	CHECK((r.inst->CR2 & i2dCR2_DMAEN) != 0u);
	CHECK(!i2dEnds(i2dLog(0), "P"));
}

TEST_CASE_FIXTURE(I2dFx, "ID4 characterization: after a slave sequential receive to transmit switch the DMA abort callback sets state LISTEN while the Tx stream runs") {
	/* NEEDS REVIEW 325 */
	I2dRig r(1);
	r.fillTx(2);
	REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
	REQUIRE(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, &r.buf[8], 3, I2C_FIRST_FRAME) == HAL_OK);
	REQUIRE(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 2, I2C_FIRST_FRAME) == HAL_OK);
	REQUIRE(r.h.State == HAL_I2C_STATE_BUSY_TX_LISTEN);
	HAL_DMA_IRQHandler(&r.rx);
	CHECK(r.h.State == HAL_I2C_STATE_LISTEN);
	CHECK(r.enabled(r.tx));
	CHECK((r.inst->CR2 & i2dCR2_DMAEN) != 0u);
}

/* ------------------------------------------------------------------------------------------------------------------ review additions */

namespace {
void i2dDummyCb(DMA_HandleTypeDef *) {}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 every DMA start call clears POS") {
	for (const auto &c : i2dCalls()) {
		SUBCASE(c.name) {
			I2dRig r(0);
			r.addPeer();
			r.queuePeer(4);
			if (c.needsListen) REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
			r.inst->CR1 |= i2dCR1_POS;
			REQUIRE(c.call(r) == HAL_OK);
			CHECK((r.inst->CR1 & i2dCR1_POS) == 0u); /* RM0090 27.6.1: POS is for the two byte receive only; a new transfer starts with POS = 0 */
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 Mem_Write_DMA and Mem_Read_DMA keep the device address, the memory address and its size in the handle") {
	for (int rd = 0; rd < 2; ++rd) {
		SUBCASE(rd ? "read" : "write") {
			I2dRig r(0);
			r.addPeer();
			r.queuePeer(4);
			if (rd) {
				REQUIRE(HAL_I2C_Mem_Read_DMA(&r.h, i2dDev, 0x1234, I2C_MEMADD_SIZE_16BIT, r.buf, 3) == HAL_OK);
			} else {
				REQUIRE(HAL_I2C_Mem_Write_DMA(&r.h, i2dDev, 0x1234, I2C_MEMADD_SIZE_16BIT, r.buf, 3) == HAL_OK);
			}
			CHECK(r.h.Devaddress == i2dDev);
			CHECK(r.h.Memaddress == 0x1234u);
			CHECK(r.h.MemaddSize == I2C_MEMADD_SIZE_16BIT);
			CHECK(r.h.XferSize == 3u);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID4 10 bit addressing: a FIRST_FRAME sequential DMA transfer sets DMAEN in the ADD10 handler, not at the call") {
	for (int rx = 0; rx < 2; ++rx) {
		SUBCASE(rx ? "receive" : "transmit") {
			I2dRig r(1, true, true, false, true);
			r.addPeer10();
			r.queuePeer(8);
			r.fillTx(2);
			if (rx) {
				REQUIRE(HAL_I2C_Master_Seq_Receive_DMA(&r.h, 0x2B7, r.buf, 2, I2C_FIRST_FRAME) == HAL_OK);
			} else {
				REQUIRE(HAL_I2C_Master_Seq_Transmit_DMA(&r.h, 0x2B7, r.buf, 2, I2C_FIRST_FRAME) == HAL_OK);
			}
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
			r.irqs();
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) != 0u); /* RM0090 27.3.8: DMAEN before the ADDR event; the 10 bit path sets it at ADD10 */
			if (rx) {
				uint8_t got[2] = {};
				r.engineRx(got, 2);
				r.doneRx();
				r.irqs();
				CHECK(i2dStarts(i2dLog(1), "S o:F4 o:B7 R o:F5 i:91 i:82"));
			} else {
				r.engineTx(r.buf, 2);
				r.doneTx();
				r.irqs();
				CHECK(i2dLog(1) == "S o:F4 o:B7 o:11 o:22");
			}
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID4 characterization: a STOP during a Slave_Seq_Receive_DMA before Size bytes ends with AF, XferCount from the stream and the listen phase ended") {
	/* NEEDS REVIEW 325 */
	I2dRig r(0);
	REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	HAL_I2C_EV_IRQHandler(&r.h);
	REQUIRE(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, r.buf, 4, I2C_FIRST_FRAME) == HAL_OK);
	uint8_t got[2] = {};
	for (unsigned i = 0; i < 2; ++i) {
		REQUIRE(hostI2cExtWrite(0, static_cast<uint8_t>(0xE1 + i)) == 1);
		r.engineRx(&got[i], 1);
	}
	hostDmaStepItems(0, r.m.rxS, 2);
	hostI2cExtStop(0);
	r.irqs();
	CHECK(r.h.XferCount == 2u);
	CHECK((r.h.ErrorCode & HAL_I2C_ERROR_AF) != 0u);
	CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
	CHECK(i2cCounts.slaveRx == 0u);
}

TEST_CASE_FIXTURE(I2dFx, "ID3 after a DMA abort ACK is off and the complete and abort callbacks of both streams are NULL") {
	I2dRig r(2);
	r.addPeer();
	r.fillTx(4);
	hostI2cDeviceNackDataAt(2, r.dev, 0);
	REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 4) == HAL_OK);
	r.rx.XferCpltCallback = i2dDummyCb;
	r.rx.XferAbortCallback = i2dDummyCb;
	r.irqs();
	r.engineTx(r.buf, 1);
	r.irqs();
	CHECK((r.inst->CR1 & i2dCR1_ACK) != 0u);
	CHECK(r.tx.XferAbortCallback != nullptr);
	HAL_DMA_IRQHandler(&r.tx);
	CHECK((r.inst->CR1 & i2dCR1_ACK) == 0u);
	CHECK(r.tx.XferCpltCallback == nullptr);
	CHECK(r.rx.XferCpltCallback == nullptr);
	CHECK(r.tx.XferAbortCallback == nullptr);
	CHECK(r.rx.XferAbortCallback == nullptr);
}

TEST_CASE_FIXTURE(I2dFx, "ID2 after the end of a DMA transfer the complete callbacks of both streams are NULL, with one or both DMA handles linked") {
	SUBCASE("transmit, rx stream callback preset") {
		I2dRig r(0);
		r.addPeer();
		r.fillTx(1);
		REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 1) == HAL_OK);
		r.rx.XferCpltCallback = i2dDummyCb;
		r.irqs();
		r.engineTx(r.buf, 1);
		r.doneTx();
		CHECK(r.tx.XferCpltCallback == nullptr);
		CHECK(r.rx.XferCpltCallback == nullptr);
	}
	SUBCASE("receive, tx stream callback preset") {
		I2dRig r(1);
		r.addPeer();
		r.queuePeer(4);
		REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, 1) == HAL_OK);
		r.tx.XferCpltCallback = i2dDummyCb;
		r.irqs();
		uint8_t g = 0;
		r.engineRx(&g, 1);
		r.doneRx();
		CHECK(r.tx.XferCpltCallback == nullptr);
		CHECK(r.rx.XferCpltCallback == nullptr);
		CHECK((r.inst->CR2 & (i2dCR2_ITEVTEN | i2dCR2_ITERREN)) == 0u);
	}
	SUBCASE("transmit with only the Tx handle linked") {
		I2dRig r(2, true, false);
		r.addPeer();
		r.fillTx(1);
		REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 1) == HAL_OK);
		r.irqs();
		r.engineTx(r.buf, 1);
		r.doneTx();
		r.irqs();
		CHECK(i2cCounts.masterTx == 1u);
		CHECK(r.tx.XferCpltCallback == nullptr);
	}
	SUBCASE("receive with only the Rx handle linked") {
		I2dRig r(2, false, true);
		r.addPeer();
		r.queuePeer(4);
		REQUIRE(HAL_I2C_Master_Receive_DMA(&r.h, i2dDev, r.buf, 1) == HAL_OK);
		r.irqs();
		uint8_t g = 0;
		r.engineRx(&g, 1);
		r.doneRx();
		r.irqs();
		CHECK(i2cCounts.masterRx == 1u);
		CHECK(r.rx.XferCpltCallback == nullptr);
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID1 Mem_Read_DMA to an address that nacks clears ACK even when it was set") {
	I2dRig r(0);
	r.inst->CR1 |= i2dCR1_ACK;
	CHECK(HAL_I2C_Mem_Read_DMA(&r.h, i2dDev, 0x10, I2C_MEMADD_SIZE_8BIT, r.buf, 3) == HAL_ERROR);
	CHECK((r.inst->CR1 & i2dCR1_ACK) == 0u);
}

TEST_CASE_FIXTURE(I2dFx, "ID4 a slave sequential call of the same direction while its stream aborts clears DMAEN and does not set it again") {
	for (int rx = 0; rx < 2; ++rx) {
		SUBCASE(rx ? "receive" : "transmit") {
			I2dRig r(1);
			r.fillTx(3);
			REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
			if (rx) {
				REQUIRE(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, &r.buf[8], 3, I2C_FIRST_FRAME) == HAL_OK);
				REQUIRE((r.inst->CR2 & i2dCR2_DMAEN) != 0u);
				CHECK(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, &r.buf[8], 3, I2C_NEXT_FRAME) == HAL_ERROR);
			} else {
				REQUIRE(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 3, I2C_FIRST_FRAME) == HAL_OK);
				REQUIRE((r.inst->CR2 & i2dCR2_DMAEN) != 0u);
				CHECK(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 3, I2C_NEXT_FRAME) == HAL_ERROR);
			}
			CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(I2dFx, "ID4 a slave sequential receive to transmit switch clears DMAEN of the Rx transfer before the Tx stream starts") {
	/* the Tx start fails (stream busy) so DMAEN stays as the switch left it */
	I2dRig r(1);
	r.fillTx(2);
	REQUIRE(HAL_I2C_EnableListen_IT(&r.h) == HAL_OK);
	REQUIRE(HAL_I2C_Slave_Seq_Receive_DMA(&r.h, &r.buf[8], 3, I2C_FIRST_FRAME) == HAL_OK);
	REQUIRE(HAL_DMA_Start_IT(&r.tx, r.bufAddr(), i2dDrAddress[1], 1) == HAL_OK);
	CHECK(HAL_I2C_Slave_Seq_Transmit_DMA(&r.h, r.buf, 2, I2C_FIRST_FRAME) == HAL_ERROR);
	CHECK((r.inst->CR2 & i2dCR2_DMAEN) == 0u);
}

TEST_CASE_FIXTURE(I2dFx, "ID3 characterization: a Master_Abort_IT whose stream cannot be aborted ends with the error callback, not AbortCpltCallback") {
	/* NEEDS REVIEW 325: ITError sets State READY before it calls the DMA abort callback directly, so I2C_DMAAbort no longer sees the ABORT state. */
	I2dRig r(0);
	r.addPeer();
	r.fillTx(4);
	REQUIRE(HAL_I2C_Master_Transmit_DMA(&r.h, i2dDev, r.buf, 4) == HAL_OK);
	r.irqs();
	REQUIRE(HAL_DMA_Abort_IT(&r.tx) == HAL_OK);
	REQUIRE(HAL_I2C_Master_Abort_IT(&r.h, i2dDev) == HAL_OK);
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.abortCplt == 0u);
	CHECK(i2cCounts.stateAtLast == HAL_I2C_STATE_READY);
	CHECK(r.h.State == HAL_I2C_STATE_READY);
}
