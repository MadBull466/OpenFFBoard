/*
 * P12 T3 group DQ: HAL_DMA_IRQHandler (and Abort_IT through it) of stm32f4xx_hal_dma.c, against the DMA register model
 * (fakes/stm32_hal/HalDma.h). Requirements DQ1-DQ8 of Tests/Code_Coverage/claude/charters/hal/P12_T3_REQUIREMENTS.md; the
 * file is compiled into ffb_hal_tests (F407VG) and ffb_hal_tests_disco (F407VG_DISCO).
 *
 * Oracles: RM0090 Rev 22 chapter 10 (10.3.9 double buffer mode, 10.3.13 / 10.3.14 transfer completion and suspension,
 * 10.3.18 error management, 10.4 Table 51 interrupt requests, 10.5.1-10.5.4 flag layout) and UM1725 Rev 8 chapter 21
 * (callbacks XferCplt / XferHalfCplt / XferM1Cplt / XferM1HalfCplt / XferError / XferAbort, HAL_DMA_GetError, error
 * code names). Flag layout (RM0090 10.5.1-10.5.4): stream 0/4 at bit 0, 1/5 at bit 6, 2/6 at bit 16, 3/7 at bit 22; in the
 * group FEIF 0, DMEIF 2, TEIF 3, HTIF 4, TCIF 5 (the HOST_DMA_* masks).
 *
 * Interrupts are not delivered by the model: the tests call HAL_DMA_IRQHandler after raising flags with the model hooks
 * (hostDmaRaise, hostDmaComplete, hostDmaStepItems), which stand in for the transfer engine.
 *
 * Naming of the double buffer callbacks (RM0090 10.3.9 only describes the CT toggle; the naming follows the UM1725
 * callback names, callback name = buffer just finished): XferCpltCallback / XferHalfCpltCallback belong to memory 0 (M0AR),
 * XferM1CpltCallback / XferM1HalfCpltCallback to memory 1 (M1AR). CT = 0 means memory 0 is the current target and CT
 * toggles at the end of each transaction, so after the end of the memory 0 buffer CT reads 1.
 *
 * NEEDS REVIEW proposals (characterization cases; the lead assigns the numbers, proposals use DQ-n):
 *  DQ-1 characterization: HAL_DMA_IRQHandler clears HTIE after a half transfer interrupt in non-circular mode. UM1725 and
 *       RM0090 are silent.
 *  DQ-2 characterization: state and lock after a transfer error (TEIF with TEIE): READY and unlocked once the stream is
 *       disabled by hardware, ErrorCode keeps TE. UM1725 is silent.
 *  DQ-3 characterization: Abort_IT with a stream whose EN never clears: Abort_IT has no timeout (the charter text of DQ7
 *       expects HAL_DMA_ERROR_TIMEOUT / state ERROR); the state stays ABORT. UM1725 is silent about a timeout of
 *       Abort_IT. The transfer error path of the IRQ handler does wait for EN = 0 with a bounded iteration count.
 *  DQ-4 characterization: the IRQ handler clears TCIE after a transfer complete in non-circular mode and TEIE after a
 *       transfer error; after the abort callback TCIE, TEIE, DMEIE and HTIE are cleared. UM1725 is silent.
 *  DQ-5 characterization: a transfer error with EN never clearing: the handler still returns, runs the error callback and
 *       leaves the handle READY and unlocked, the error code TE kept (no TIMEOUT code). UM1725 is silent.
 */
#include "HalDma.h"
#include "HalFixture.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstring>
#include <string>

namespace {

enum DmaIrqEvent { kCplt, kHalf, kM1Cplt, kM1Half, kError, kAbort };

struct DmaIrqRecord {
	DmaIrqEvent event;
	DMA_HandleTypeDef *handle;
	uint32_t state; // hdma->State when the callback ran
	uint32_t lock;  // hdma->Lock when the callback ran
	uint32_t error; // hdma->ErrorCode when the callback ran
};

DmaIrqRecord dmaIrqLog[64];
unsigned dmaIrqLogCount = 0;

void dmaIrqRecordEvent(DmaIrqEvent e, DMA_HandleTypeDef *h) {
	REQUIRE(dmaIrqLogCount < 64u);
	dmaIrqLog[dmaIrqLogCount++] = {e, h, static_cast<uint32_t>(h->State), static_cast<uint32_t>(h->Lock), h->ErrorCode};
}
void dmaIrqCplt(DMA_HandleTypeDef *h) { dmaIrqRecordEvent(kCplt, h); }
void dmaIrqHalf(DMA_HandleTypeDef *h) { dmaIrqRecordEvent(kHalf, h); }
void dmaIrqM1Cplt(DMA_HandleTypeDef *h) { dmaIrqRecordEvent(kM1Cplt, h); }
void dmaIrqM1Half(DMA_HandleTypeDef *h) { dmaIrqRecordEvent(kM1Half, h); }
void dmaIrqError(DMA_HandleTypeDef *h) { dmaIrqRecordEvent(kError, h); }
void dmaIrqAbort(DMA_HandleTypeDef *h) { dmaIrqRecordEvent(kAbort, h); }

unsigned dmaIrqCount(DmaIrqEvent e) {
	unsigned n = 0;
	for (unsigned i = 0; i < dmaIrqLogCount; ++i) n += dmaIrqLog[i].event == e ? 1u : 0u;
	return n;
}

DMA_Stream_TypeDef *dmaIrqStream(unsigned i) {
	DMA_Stream_TypeDef *const s[16] = {DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5,
									   DMA1_Stream6, DMA1_Stream7, DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3,
									   DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7};
	return s[i];
}

constexpr uint32_t kSrc = 0x40004000u; // peripheral address, never dereferenced
constexpr uint32_t kDst = 0x20000000u; // memory 0
constexpr uint32_t kDst1 = 0x20001000u; // memory 1
constexpr uint32_t kLen = 8u;
constexpr uint32_t kAllFlags = HOST_DMA_FEIF | HOST_DMA_DMEIF | HOST_DMA_TEIF | HOST_DMA_HTIF | HOST_DMA_TCIF;

/* Stream index i 0..7 = DMA1, 8..15 = DMA2. */
unsigned ctl(unsigned i) { return i / 8u; }
unsigned str(unsigned i) { return i % 8u; }

struct DmaIrqFx : HalFixture {
	DmaIrqFx() {
		hostDmaResetState();
		dmaIrqLogCount = 0;
		__HAL_RCC_DMA1_CLK_ENABLE();
		__HAL_RCC_DMA2_CLK_ENABLE();
	}

	enum Setup { kPlain, kFifo, kNoCallbacks };

	/* Initialises the handle (peripheral to memory, byte, direct mode or FIFO full threshold) with all callbacks set. */
	static void init(DMA_HandleTypeDef &h, unsigned i, uint32_t mode = DMA_NORMAL, Setup setup = kPlain) {
		h = DMA_HandleTypeDef{};
		h.Instance = dmaIrqStream(i);
		h.Init.Channel = DMA_CHANNEL_0;
		h.Init.Direction = DMA_PERIPH_TO_MEMORY;
		h.Init.PeriphInc = DMA_PINC_DISABLE;
		h.Init.MemInc = DMA_MINC_ENABLE;
		h.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		h.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		h.Init.Mode = mode;
		h.Init.Priority = DMA_PRIORITY_LOW;
		h.Init.FIFOMode = setup == kFifo ? DMA_FIFOMODE_ENABLE : DMA_FIFOMODE_DISABLE;
		h.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		h.Init.MemBurst = DMA_MBURST_SINGLE;
		h.Init.PeriphBurst = DMA_PBURST_SINGLE;
		REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
		if (setup != kNoCallbacks) {
			h.XferCpltCallback = dmaIrqCplt;
			h.XferHalfCpltCallback = dmaIrqHalf;
			h.XferM1CpltCallback = dmaIrqM1Cplt;
			h.XferM1HalfCpltCallback = dmaIrqM1Half;
			h.XferErrorCallback = dmaIrqError;
			h.XferAbortCallback = dmaIrqAbort;
		}
	}

	/* Init + HAL_DMA_Start_IT: stream enabled, BUSY. */
	static void start(DMA_HandleTypeDef &h, unsigned i, uint32_t mode = DMA_NORMAL, Setup setup = kPlain) {
		init(h, i, mode, setup);
		REQUIRE(HAL_DMA_Start_IT(&h, kSrc, kDst, kLen) == HAL_OK);
		REQUIRE((peek(&h.Instance->CR) & DMA_SxCR_EN) != 0u);
		// Start_IT does not enable FEIE (see report); the FIFO error interrupt is enabled here as the user would
		if (setup == kFifo) h.Instance->FCR |= DMA_SxFCR_FEIE;
	}

	/* Init + HAL_DMAEx_MultiBufferStart_IT: double buffer mode, CT = 0 (memory 0 is the current target). */
	static void startDouble(DMA_HandleTypeDef &h, unsigned i, Setup setup = kPlain) {
		init(h, i, DMA_CIRCULAR, setup);
		REQUIRE(HAL_DMAEx_MultiBufferStart_IT(&h, kSrc, kDst, kDst1, kLen) == HAL_OK);
		REQUIRE((peek(&h.Instance->CR) & DMA_SxCR_DBM) != 0u);
		REQUIRE((peek(&h.Instance->CR) & DMA_SxCR_CT) == 0u);
	}

	static uint32_t flags(unsigned i) { return hostDmaFlags(ctl(i), str(i)); }
	static void raise(unsigned i, uint32_t mask) { hostDmaRaise(ctl(i), str(i), mask); }
	static bool enabled(DMA_HandleTypeDef &h) { return (peek(&h.Instance->CR) & DMA_SxCR_EN) != 0u; }
	static unsigned total() { return dmaIrqLogCount; }
};

constexpr unsigned kDefault = 13; // DMA2 stream 5: HISR, bit offset 6

} // namespace

/* ------------------------------------------------------------------ DQ1 transfer complete */

TEST_CASE_FIXTURE(DmaIrqFx, "DQ1 the transfer complete flag is cleared by the handler") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	hostDmaComplete(ctl(kDefault), str(kDefault));
	REQUIRE(flags(kDefault) == HOST_DMA_TCIF); // RM0090 10.3.13: TCIF set at the end of the transfer
	HAL_DMA_IRQHandler(&h);
	CHECK(flags(kDefault) == 0u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ1 the transfer complete callback runs once and no other callback runs") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	hostDmaComplete(ctl(kDefault), str(kDefault));
	HAL_DMA_IRQHandler(&h);
	CHECK(dmaIrqCount(kCplt) == 1u);
	CHECK(total() == 1u);
	CHECK(dmaIrqLog[0].handle == &h); // UM1725: the callback receives the handle
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ1 the handle is READY and unlocked when the transfer complete callback runs") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	hostDmaComplete(ctl(kDefault), str(kDefault));
	HAL_DMA_IRQHandler(&h);
	REQUIRE(total() == 1u);
	CHECK(dmaIrqLog[0].state == static_cast<uint32_t>(HAL_DMA_STATE_READY));
	CHECK(dmaIrqLog[0].lock == static_cast<uint32_t>(HAL_UNLOCKED));
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ1 a normal mode transfer complete leaves the stream disabled and the error code NONE") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	hostDmaComplete(ctl(kDefault), str(kDefault));
	HAL_DMA_IRQHandler(&h);
	CHECK_FALSE(enabled(h)); // RM0090 10.3.13: EN cleared by hardware at the end of a normal mode transfer
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_NONE);
}

/* characterization (NEEDS REVIEW DQ-4) */
TEST_CASE_FIXTURE(DmaIrqFx, "DQ1 characterization: the handler clears TCIE after a transfer complete in non-circular mode") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	REQUIRE((peek(&h.Instance->CR) & DMA_SxCR_TCIE) != 0u);
	hostDmaComplete(ctl(kDefault), str(kDefault));
	HAL_DMA_IRQHandler(&h);
	CHECK((peek(&h.Instance->CR) & DMA_SxCR_TCIE) == 0u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ1 in circular mode the complete callback runs on every transfer end and the stream stays enabled") {
	DMA_HandleTypeDef h;
	start(h, kDefault, DMA_CIRCULAR);
	hostDmaComplete(ctl(kDefault), str(kDefault));
	HAL_DMA_IRQHandler(&h);
	hostDmaComplete(ctl(kDefault), str(kDefault));
	HAL_DMA_IRQHandler(&h);
	CHECK(dmaIrqCount(kCplt) == 2u); // RM0090 10.4 Table 51: TCIF with TCIE gives an interrupt at every end of transfer
	CHECK(flags(kDefault) == 0u);
	CHECK(enabled(h)); // RM0090 10.3.13: circular mode reloads NDTR and continues
}

/* ------------------------------------------------------------------ DQ2 half transfer */

TEST_CASE_FIXTURE(DmaIrqFx, "DQ2 the half transfer flag is cleared by the handler") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	hostDmaStepItems(ctl(kDefault), str(kDefault), kLen / 2u);
	REQUIRE(flags(kDefault) == HOST_DMA_HTIF);
	HAL_DMA_IRQHandler(&h);
	CHECK(flags(kDefault) == 0u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ2 the half transfer callback runs once and no other callback runs") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	hostDmaStepItems(ctl(kDefault), str(kDefault), kLen / 2u);
	HAL_DMA_IRQHandler(&h);
	CHECK(dmaIrqCount(kHalf) == 1u);
	CHECK(total() == 1u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ2 a half transfer leaves the stream enabled and the handle BUSY") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	hostDmaStepItems(ctl(kDefault), str(kDefault), kLen / 2u);
	HAL_DMA_IRQHandler(&h);
	CHECK(enabled(h)); // the transfer is only half done
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_BUSY);
}

/* characterization (NEEDS REVIEW DQ-1): UM1725 and RM0090 do not say whether the HAL clears HTIE after the first half
 * transfer interrupt in non-circular mode; the HAL clears it, so a second HTIF would not call the callback. */
TEST_CASE_FIXTURE(DmaIrqFx, "DQ2 characterization: the handler clears HTIE after a half transfer in non-circular mode") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	REQUIRE((peek(&h.Instance->CR) & DMA_SxCR_HTIE) != 0u);
	hostDmaStepItems(ctl(kDefault), str(kDefault), kLen / 2u);
	HAL_DMA_IRQHandler(&h);
	CHECK((peek(&h.Instance->CR) & DMA_SxCR_HTIE) == 0u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ2 in circular mode the half callback runs on each half transfer") {
	DMA_HandleTypeDef h;
	start(h, kDefault, DMA_CIRCULAR);
	hostDmaStepItems(ctl(kDefault), str(kDefault), kLen / 2u); // first half
	HAL_DMA_IRQHandler(&h);
	hostDmaStepItems(ctl(kDefault), str(kDefault), kLen / 2u); // end of transfer, NDTR reloaded
	HAL_DMA_IRQHandler(&h);
	hostDmaStepItems(ctl(kDefault), str(kDefault), kLen / 2u); // first half of the next cycle
	HAL_DMA_IRQHandler(&h);
	CHECK(dmaIrqCount(kHalf) == 2u); // RM0090 10.4: HTIF with HTIE is an interrupt at every half transfer
	CHECK(dmaIrqCount(kCplt) == 1u);
	CHECK(flags(kDefault) == 0u);
}

/* ------------------------------------------------------------------ DQ3 transfer error */

TEST_CASE_FIXTURE(DmaIrqFx, "DQ3 the transfer error flag is cleared by the handler") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	raise(kDefault, HOST_DMA_TEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK(flags(kDefault) == 0u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ3 a transfer error sets the error code TE") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	raise(kDefault, HOST_DMA_TEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK((HAL_DMA_GetError(&h) & HAL_DMA_ERROR_TE) != 0u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ3 the error callback runs once with the error code TE visible and no other callback runs") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	raise(kDefault, HOST_DMA_TEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK(dmaIrqCount(kError) == 1u);
	CHECK(total() == 1u);
	REQUIRE(total() >= 1u);
	CHECK((dmaIrqLog[0].error & HAL_DMA_ERROR_TE) != 0u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ3 the stream is disabled after a transfer error") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	raise(kDefault, HOST_DMA_TEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK_FALSE(enabled(h)); // RM0090 10.3.18: a stream with a transfer error is disabled
}

/* characterization (NEEDS REVIEW DQ-2): UM1725 does not say which state follows a transfer error. With the stream disabled
 * by the hardware the HAL leaves the handle READY and unlocked, with the error code TE kept. */
TEST_CASE_FIXTURE(DmaIrqFx, "DQ3 characterization: state after a transfer error is READY and unlocked") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	raise(kDefault, HOST_DMA_TEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((HAL_DMA_GetError(&h) & HAL_DMA_ERROR_TE) != 0u);
}

/* characterization (NEEDS REVIEW DQ-5): the handler's transfer error path waits for EN = 0 with a bounded iteration count;
 * with EN never clearing it must still return. */
TEST_CASE_FIXTURE(DmaIrqFx, "DQ3 characterization: a transfer error with EN never clearing returns, runs the error callback, READY and unlocked") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	hostDmaStickEnabled(ctl(kDefault), str(kDefault), 1);
	raise(kDefault, HOST_DMA_TEIF);                        // the model clears EN on TEIF ...
	poke(&h.Instance->CR, peek(&h.Instance->CR) | DMA_SxCR_EN); // ... a stuck stream is put back
	HAL_DMA_IRQHandler(&h);                                // must return (no hang)
	CHECK(enabled(h));
	CHECK(dmaIrqCount(kError) == 1u);
	CHECK(total() == 1u);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((HAL_DMA_GetError(&h) & HAL_DMA_ERROR_TE) != 0u);
}

/* characterization (NEEDS REVIEW DQ-4) */
TEST_CASE_FIXTURE(DmaIrqFx, "DQ3 characterization: the handler clears TEIE after a transfer error") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	REQUIRE((peek(&h.Instance->CR) & DMA_SxCR_TEIE) != 0u);
	raise(kDefault, HOST_DMA_TEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK((peek(&h.Instance->CR) & DMA_SxCR_TEIE) == 0u);
}

/* ------------------------------------------------------------------ DQ4 FIFO error and direct mode error */

TEST_CASE_FIXTURE(DmaIrqFx, "DQ4 the FIFO error flag is cleared by the handler") {
	DMA_HandleTypeDef h;
	start(h, kDefault, DMA_NORMAL, kFifo);
	raise(kDefault, HOST_DMA_FEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK(flags(kDefault) == 0u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ4 a FIFO error sets the error code FE and runs the error callback once") {
	DMA_HandleTypeDef h;
	start(h, kDefault, DMA_NORMAL, kFifo);
	raise(kDefault, HOST_DMA_FEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK((HAL_DMA_GetError(&h) & HAL_DMA_ERROR_FE) != 0u);
	CHECK(dmaIrqCount(kError) == 1u);
	CHECK(total() == 1u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ4 a FIFO overrun or underrun does not disable the stream") {
	DMA_HandleTypeDef h;
	start(h, kDefault, DMA_NORMAL, kFifo);
	raise(kDefault, HOST_DMA_FEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK(enabled(h)); // RM0090 10.3.18: it is up to the software to disable the stream
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ4 the direct mode error flag is cleared by the handler") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	REQUIRE((peek(&h.Instance->CR) & DMA_SxCR_DMEIE) != 0u);
	raise(kDefault, HOST_DMA_DMEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK(flags(kDefault) == 0u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ4 a direct mode error sets the error code DME and runs the error callback once") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	raise(kDefault, HOST_DMA_DMEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK((HAL_DMA_GetError(&h) & HAL_DMA_ERROR_DME) != 0u);
	CHECK(dmaIrqCount(kError) == 1u);
	CHECK(total() == 1u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ4 a direct mode error does not disable the stream") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	raise(kDefault, HOST_DMA_DMEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK(enabled(h)); // RM0090 10.3.18: no automatic disable for DMEIF
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ4 a FIFO error and a direct mode error together give both error codes") {
	DMA_HandleTypeDef h;
	start(h, kDefault, DMA_NORMAL, kFifo);
	raise(kDefault, HOST_DMA_FEIF | HOST_DMA_DMEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK((HAL_DMA_GetError(&h) & (HAL_DMA_ERROR_FE | HAL_DMA_ERROR_DME)) == (HAL_DMA_ERROR_FE | HAL_DMA_ERROR_DME));
	CHECK(flags(kDefault) == 0u);
}

/* ------------------------------------------------------------------ DQ5 disabled sources and NULL callbacks */

TEST_CASE_FIXTURE(DmaIrqFx, "DQ5 a transfer complete flag with TCIE clear does nothing and stays set") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	h.Instance->CR &= ~DMA_SxCR_TCIE;
	hostDmaComplete(ctl(kDefault), str(kDefault));
	HAL_DMA_IRQHandler(&h);
	CHECK(total() == 0u);
	CHECK(flags(kDefault) == HOST_DMA_TCIF); // RM0090 Table 51: the flag is only an interrupt source with its enable bit
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ5 a half transfer flag with HTIE clear does nothing and stays set") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	h.Instance->CR &= ~DMA_SxCR_HTIE;
	hostDmaStepItems(ctl(kDefault), str(kDefault), kLen / 2u);
	HAL_DMA_IRQHandler(&h);
	CHECK(total() == 0u);
	CHECK(flags(kDefault) == HOST_DMA_HTIF);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ5 a transfer error flag with TEIE clear does nothing and stays set") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	h.Instance->CR &= ~DMA_SxCR_TEIE;
	raise(kDefault, HOST_DMA_TEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK(total() == 0u);
	CHECK(flags(kDefault) == HOST_DMA_TEIF);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_NONE);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ5 a FIFO error flag with FEIE clear does nothing and stays set") {
	DMA_HandleTypeDef h;
	start(h, kDefault, DMA_NORMAL, kFifo);
	h.Instance->FCR &= ~DMA_SxFCR_FEIE;
	raise(kDefault, HOST_DMA_FEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK(total() == 0u);
	CHECK(flags(kDefault) == HOST_DMA_FEIF);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_NONE);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ5 a direct mode error flag with DMEIE clear does nothing and stays set") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	h.Instance->CR &= ~DMA_SxCR_DMEIE;
	raise(kDefault, HOST_DMA_DMEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK(total() == 0u);
	CHECK(flags(kDefault) == HOST_DMA_DMEIF);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_NONE);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ5 the handler with no flag set calls nothing and changes nothing") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	const uint32_t cr = peek(&h.Instance->CR);
	HAL_DMA_IRQHandler(&h);
	CHECK(total() == 0u);
	CHECK(peek(&h.Instance->CR) == cr);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_BUSY);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ5 a NULL transfer complete callback is not called and the flag is cleared") {
	DMA_HandleTypeDef h;
	start(h, kDefault, DMA_NORMAL, kNoCallbacks);
	hostDmaComplete(ctl(kDefault), str(kDefault));
	HAL_DMA_IRQHandler(&h);
	CHECK(flags(kDefault) == 0u);
	CHECK(total() == 0u);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ5 a NULL half transfer callback is not called and the flag is cleared") {
	DMA_HandleTypeDef h;
	start(h, kDefault, DMA_NORMAL, kNoCallbacks);
	h.Instance->CR |= DMA_SxCR_HTIE; // Start_IT enables HTIE only for a handle with a half callback
	hostDmaStepItems(ctl(kDefault), str(kDefault), kLen / 2u);
	HAL_DMA_IRQHandler(&h);
	CHECK(flags(kDefault) == 0u);
	CHECK(total() == 0u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ5 a NULL error callback is not called and the flag is cleared") {
	DMA_HandleTypeDef h;
	start(h, kDefault, DMA_NORMAL, kNoCallbacks);
	raise(kDefault, HOST_DMA_TEIF);
	HAL_DMA_IRQHandler(&h);
	CHECK(flags(kDefault) == 0u);
	CHECK(total() == 0u);
	CHECK((HAL_DMA_GetError(&h) & HAL_DMA_ERROR_TE) != 0u);
}

/* ------------------------------------------------------------------ DQ6 double buffer mode */

TEST_CASE_FIXTURE(DmaIrqFx, "DQ6 the end of the memory 0 buffer runs the transfer complete callback") {
	DMA_HandleTypeDef h;
	startDouble(h, kDefault);
	hostDmaComplete(ctl(kDefault), str(kDefault)); // end of the transaction: CT toggles to 1
	REQUIRE((peek(&h.Instance->CR) & DMA_SxCR_CT) != 0u);
	HAL_DMA_IRQHandler(&h);
	CHECK(dmaIrqCount(kCplt) == 1u);
	CHECK(total() == 1u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ6 the end of the memory 1 buffer runs the memory 1 transfer complete callback") {
	DMA_HandleTypeDef h;
	startDouble(h, kDefault);
	hostDmaComplete(ctl(kDefault), str(kDefault)); // memory 0 done
	HAL_DMA_IRQHandler(&h);
	hostDmaComplete(ctl(kDefault), str(kDefault)); // memory 1 done: CT back to 0
	REQUIRE((peek(&h.Instance->CR) & DMA_SxCR_CT) == 0u);
	HAL_DMA_IRQHandler(&h);
	CHECK(dmaIrqCount(kM1Cplt) == 1u);
	CHECK(dmaIrqCount(kCplt) == 1u); // only the first interrupt
	CHECK(total() == 2u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ6 the transfer complete flag is cleared and the stream stays enabled and BUSY") {
	DMA_HandleTypeDef h;
	startDouble(h, kDefault);
	hostDmaComplete(ctl(kDefault), str(kDefault));
	HAL_DMA_IRQHandler(&h);
	CHECK(flags(kDefault) == 0u);
	CHECK(enabled(h)); // RM0090 10.3.9: circular, the pointers swap at each end of transaction
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_BUSY);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ6 a half transfer in the memory 0 buffer runs the half callback") {
	DMA_HandleTypeDef h;
	startDouble(h, kDefault);
	hostDmaStepItems(ctl(kDefault), str(kDefault), kLen / 2u); // CT = 0, memory 0 is being transferred
	HAL_DMA_IRQHandler(&h);
	CHECK(dmaIrqCount(kHalf) == 1u);
	CHECK(total() == 1u);
	CHECK(flags(kDefault) == 0u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ6 a half transfer in the memory 1 buffer runs the memory 1 half callback") {
	DMA_HandleTypeDef h;
	startDouble(h, kDefault);
	hostDmaComplete(ctl(kDefault), str(kDefault)); // memory 0 done, CT = 1
	HAL_DMA_IRQHandler(&h);
	hostDmaStepItems(ctl(kDefault), str(kDefault), kLen / 2u); // half of memory 1
	REQUIRE((peek(&h.Instance->CR) & DMA_SxCR_CT) != 0u);
	HAL_DMA_IRQHandler(&h);
	CHECK(dmaIrqCount(kM1Half) == 1u);
	CHECK(dmaIrqCount(kHalf) == 0u);
	CHECK(flags(kDefault) == 0u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ6 the half transfer interrupt enable stays set in double buffer mode") {
	DMA_HandleTypeDef h;
	startDouble(h, kDefault);
	REQUIRE((peek(&h.Instance->CR) & DMA_SxCR_HTIE) != 0u);
	hostDmaStepItems(ctl(kDefault), str(kDefault), kLen / 2u);
	HAL_DMA_IRQHandler(&h);
	CHECK((peek(&h.Instance->CR) & DMA_SxCR_HTIE) != 0u); // the next half transfer must still interrupt
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ6 a NULL memory 1 transfer complete callback is not called and the flag is cleared") {
	DMA_HandleTypeDef h;
	startDouble(h, kDefault);
	h.XferM1CpltCallback = nullptr;
	hostDmaComplete(ctl(kDefault), str(kDefault));
	HAL_DMA_IRQHandler(&h);
	hostDmaComplete(ctl(kDefault), str(kDefault)); // CT = 0 again: the memory 1 callback is due
	HAL_DMA_IRQHandler(&h);
	CHECK(dmaIrqCount(kM1Cplt) == 0u);
	CHECK(flags(kDefault) == 0u);
}

/* ------------------------------------------------------------------ DQ7 Abort_IT */

TEST_CASE_FIXTURE(DmaIrqFx, "DQ7 after Abort_IT the next transfer complete interrupt runs the abort callback once") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	REQUIRE(HAL_DMA_Abort_IT(&h) == HAL_OK);
	REQUIRE_FALSE(enabled(h));
	REQUIRE(flags(kDefault) == HOST_DMA_TCIF); // RM0090 10.3.14: a software stop ends with TCIF set
	HAL_DMA_IRQHandler(&h);
	CHECK(dmaIrqCount(kAbort) == 1u);
	CHECK(total() == 1u); // not the transfer complete callback
	CHECK(dmaIrqLog[0].handle == &h);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ7 Abort_IT completes from the IRQ handler after the stop: callback once, flags cleared, READY") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	REQUIRE(HAL_DMA_Abort_IT(&h) == HAL_OK);
	HAL_DMA_IRQHandler(&h);
	CHECK(dmaIrqCount(kAbort) == 1u);
	CHECK(total() == 1u);
	CHECK(flags(kDefault) == 0u);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ7 the abort interrupt clears all flags of the stream") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	h.XferHalfCpltCallback = nullptr; // keep HTIF and FEIF unhandled (no HTIE, direct mode has no FEIE)
	h.Instance->CR &= ~DMA_SxCR_HTIE;
	REQUIRE(HAL_DMA_Abort_IT(&h) == HAL_OK);
	raise(kDefault, HOST_DMA_HTIF | HOST_DMA_FEIF); // TCIF is set by the stop itself (RM0090 10.3.14)
	HAL_DMA_IRQHandler(&h);
	CHECK(flags(kDefault) == 0u);
	CHECK(dmaIrqCount(kAbort) == 1u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ7 the abort interrupt leaves the handle READY and unlocked when the callback runs") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	REQUIRE(HAL_DMA_Abort_IT(&h) == HAL_OK);
	HAL_DMA_IRQHandler(&h);
	REQUIRE(total() == 1u);
	CHECK(dmaIrqLog[0].state == static_cast<uint32_t>(HAL_DMA_STATE_READY));
	CHECK(dmaIrqLog[0].lock == static_cast<uint32_t>(HAL_UNLOCKED));
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
}

/* characterization (NEEDS REVIEW DQ-4) */
TEST_CASE_FIXTURE(DmaIrqFx, "DQ7 characterization: after the abort callback TCIE, TEIE, DMEIE and HTIE are cleared") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	REQUIRE((peek(&h.Instance->CR) & (DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_HTIE)) ==
			(DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_HTIE));
	REQUIRE(HAL_DMA_Abort_IT(&h) == HAL_OK);
	HAL_DMA_IRQHandler(&h);
	REQUIRE(dmaIrqCount(kAbort) == 1u);
	CHECK((peek(&h.Instance->CR) & (DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_HTIE)) == 0u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ7 a NULL abort callback is not called and the handle returns to READY") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	h.XferAbortCallback = nullptr;
	REQUIRE(HAL_DMA_Abort_IT(&h) == HAL_OK);
	HAL_DMA_IRQHandler(&h);
	CHECK(total() == 0u);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
	CHECK(flags(kDefault) == 0u);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ7 a transfer complete arriving after Abort_IT does not run the transfer complete callback") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	REQUIRE(HAL_DMA_Abort_IT(&h) == HAL_OK);
	HAL_DMA_IRQHandler(&h);
	CHECK(dmaIrqCount(kCplt) == 0u);
}

/* characterization (NEEDS REVIEW DQ-3): the charter expects the abort to end after 5 ms with HAL_DMA_ERROR_TIMEOUT and
 * state ERROR when EN never clears. UM1725 says nothing about a timeout of Abort_IT; Abort_IT has none (a polling timeout
 * exists in HAL_DMA_Abort; the transfer error path of the IRQ handler has its own bounded wait, see DQ3). Documented here: Abort_IT returns HAL_OK without waiting, and later interrupts without
 * a flag change nothing, the state stays ABORT. */
TEST_CASE_FIXTURE(DmaIrqFx, "DQ7 characterization: Abort_IT with EN never clearing has no timeout and the state stays ABORT") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	hostDmaStickEnabled(ctl(kDefault), str(kDefault), 1);
	REQUIRE(HAL_DMA_Abort_IT(&h) == HAL_OK);
	REQUIRE(enabled(h));
	hostTickSet(10u); // 10 ms, past the 5 ms of HAL_DMA_Abort
	HAL_DMA_IRQHandler(&h);
	hostTickSet(100u);
	HAL_DMA_IRQHandler(&h);
	CHECK(total() == 0u);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_ABORT);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_NONE);
}

/* ------------------------------------------------------------------ DQ8 all 16 streams */

namespace {

DMA_HandleTypeDef dmaIrqHandles[16];

/* Starts all 16 streams (callbacks set), sets all flags of the neighbours x-1 and x+1 of the same controller and of the
 * same stream of the other controller, raises 'mask' on x and runs the handler of x. */
void dmaIrqNeighbourRun(unsigned x, uint32_t mask) {
	for (unsigned i = 0; i < 16u; ++i) DmaIrqFx::start(dmaIrqHandles[i], i);
	unsigned guarded[3];
	unsigned nGuard = 0;
	if (str(x) > 0u) guarded[nGuard++] = x - 1u;
	if (str(x) < 7u) guarded[nGuard++] = x + 1u;
	guarded[nGuard++] = (x + 8u) % 16u;
	for (unsigned k = 0; k < nGuard; ++k) DmaIrqFx::raise(guarded[k], kAllFlags);
	DmaIrqFx::raise(x, mask);
	dmaIrqLogCount = 0;
	HAL_DMA_IRQHandler(&dmaIrqHandles[x]);
	for (unsigned k = 0; k < nGuard; ++k) {
		INFO("neighbour stream " << guarded[k]);
		CHECK(DmaIrqFx::flags(guarded[k]) == kAllFlags);
	}
}

} // namespace

TEST_CASE_FIXTURE(DmaIrqFx, "DQ8 the handler of each of the 16 streams clears only its transfer complete flag") {
	for (unsigned x = 0; x < 16u; ++x) {
		const std::string name = "stream " + std::to_string(x);
		SUBCASE(name.c_str()) {
			dmaIrqNeighbourRun(x, HOST_DMA_TCIF);
			CHECK(flags(x) == 0u);
			CHECK(total() == 1u);
			REQUIRE(total() >= 1u);
			CHECK(dmaIrqLog[0].event == kCplt);
			CHECK(dmaIrqLog[0].handle == &dmaIrqHandles[x]);
		}
	}
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ8 the handler of each of the 16 streams clears only its half transfer flag") {
	for (unsigned x = 0; x < 16u; ++x) {
		const std::string name = "stream " + std::to_string(x);
		SUBCASE(name.c_str()) {
			dmaIrqNeighbourRun(x, HOST_DMA_HTIF);
			CHECK(flags(x) == 0u);
			CHECK(total() == 1u);
			REQUIRE(total() >= 1u);
			CHECK(dmaIrqLog[0].event == kHalf);
			CHECK(dmaIrqLog[0].handle == &dmaIrqHandles[x]);
		}
	}
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ8 the handler of each of the 16 streams clears only its transfer error flag") {
	for (unsigned x = 0; x < 16u; ++x) {
		const std::string name = "stream " + std::to_string(x);
		SUBCASE(name.c_str()) {
			dmaIrqNeighbourRun(x, HOST_DMA_TEIF);
			CHECK(flags(x) == 0u);
			CHECK(total() == 1u);
			REQUIRE(total() >= 1u);
			CHECK(dmaIrqLog[0].event == kError);
			CHECK(dmaIrqLog[0].handle == &dmaIrqHandles[x]);
		}
	}
}

TEST_CASE_FIXTURE(DmaIrqFx, "DQ8 the handler of each of the 16 streams clears only its direct mode error flag") {
	for (unsigned x = 0; x < 16u; ++x) {
		const std::string name = "stream " + std::to_string(x);
		SUBCASE(name.c_str()) {
			dmaIrqNeighbourRun(x, HOST_DMA_DMEIF);
			CHECK(flags(x) == 0u);
			CHECK(total() == 1u);
			REQUIRE(total() >= 1u);
			CHECK(dmaIrqLog[0].event == kError);
			CHECK(dmaIrqLog[0].handle == &dmaIrqHandles[x]);
		}
	}
}

/* ------------------------------------------------------------------ DC callback registration */
/* UM1725 21.2.3 gives the names HAL_DMA_RegisterCallback / HAL_DMA_UnRegisterCallback ("Register callbacks"); the pointer
 * storage and the call from the IRQ handler are plain tests, the return codes for invalid use are characterization.
 * NEEDS REVIEW proposals:
 *  DC-1 characterization: Register / UnRegister with an ID outside 0..6 return HAL_ERROR and change nothing.
 *  DC-2 characterization: RegisterCallback with HAL_DMA_XFER_ALL_CB_ID returns HAL_ERROR (only UnRegister accepts ALL).
 *  DC-3 characterization: Register / UnRegister while the state is not READY (state set to BUSY by hand) return HAL_ERROR,
 *       leave the pointer unchanged and the handle unlocked.
 *  DC-4 characterization: Register after Start_IT (handle still locked by Start_IT) does not store the pointer and does
 *       not return HAL_OK (the HAL returns HAL_BUSY from the lock).
 */
namespace {

typedef void (*DmaIrqCb)(DMA_HandleTypeDef *);
DmaIrqCb DMA_HandleTypeDef::*const kMembers[6] = {
	&DMA_HandleTypeDef::XferCpltCallback,   &DMA_HandleTypeDef::XferHalfCpltCallback,
	&DMA_HandleTypeDef::XferM1CpltCallback, &DMA_HandleTypeDef::XferM1HalfCpltCallback,
	&DMA_HandleTypeDef::XferErrorCallback,  &DMA_HandleTypeDef::XferAbortCallback};
const DmaIrqCb kCbs[6] = {dmaIrqCplt, dmaIrqHalf, dmaIrqM1Cplt, dmaIrqM1Half, dmaIrqError, dmaIrqAbort};
const DmaIrqEvent kEvents[6] = {kCplt, kHalf, kM1Cplt, kM1Half, kError, kAbort};
const HAL_DMA_CallbackIDTypeDef kIds[6] = {HAL_DMA_XFER_CPLT_CB_ID,     HAL_DMA_XFER_HALFCPLT_CB_ID,
										   HAL_DMA_XFER_M1CPLT_CB_ID,   HAL_DMA_XFER_M1HALFCPLT_CB_ID,
										   HAL_DMA_XFER_ERROR_CB_ID,    HAL_DMA_XFER_ABORT_CB_ID};

std::string dmaIrqIdName(unsigned k) { return "callback id " + std::to_string(k); }

bool allNull(const DMA_HandleTypeDef &h) {
	for (unsigned k = 0; k < 6; ++k) {
		if (h.*kMembers[k] != nullptr) return false;
	}
	return true;
}

} // namespace

TEST_CASE_FIXTURE(DmaIrqFx, "DC1 RegisterCallback stores the pointer in the matching member only and returns HAL_OK") {
	for (unsigned k = 0; k < 6u; ++k) {
		SUBCASE(dmaIrqIdName(k).c_str()) {
			DMA_HandleTypeDef h;
			init(h, kDefault, DMA_NORMAL, kNoCallbacks);
			REQUIRE(allNull(h));
			CHECK(HAL_DMA_RegisterCallback(&h, kIds[k], kCbs[k]) == HAL_OK);
			for (unsigned m = 0; m < 6u; ++m) CHECK((h.*kMembers[m] == kCbs[k]) == (m == k));
		}
	}
}

TEST_CASE_FIXTURE(DmaIrqFx, "DC1 a callback registered with RegisterCallback is called by the IRQ handler") {
	for (unsigned k = 0; k < 6u; ++k) {
		SUBCASE(dmaIrqIdName(k).c_str()) {
			DMA_HandleTypeDef h;
			const bool dbl = k == 2u || k == 3u;
			init(h, kDefault, dbl ? DMA_CIRCULAR : DMA_NORMAL, kNoCallbacks);
			REQUIRE(HAL_DMA_RegisterCallback(&h, kIds[k], kCbs[k]) == HAL_OK);
			if (dbl) { // the HAL needs the complete, M1 complete and error callbacks for MultiBufferStart_IT (see report)
				h.XferCpltCallback = dmaIrqCplt;
				h.XferM1CpltCallback = dmaIrqM1Cplt;
				h.XferErrorCallback = dmaIrqError;
				h.XferHalfCpltCallback = dmaIrqHalf; // HTIE is set when a half callback exists
				h.XferM1HalfCpltCallback = dmaIrqM1Half;
				REQUIRE(HAL_DMAEx_MultiBufferStart_IT(&h, kSrc, kDst, kDst1, kLen) == HAL_OK);
			} else {
				REQUIRE(HAL_DMA_Start_IT(&h, kSrc, kDst, kLen) == HAL_OK);
			}
			const unsigned s = str(kDefault), c = ctl(kDefault);
			switch (k) {
			case 0: hostDmaComplete(c, s); break;
			case 1: hostDmaStepItems(c, s, kLen / 2u); break;
			case 2: hostDmaComplete(c, s); hostDmaComplete(c, s); break; // memory 1 ends: CT back to 0
			case 3: // memory 0 ends (handled), then half of memory 1
				hostDmaComplete(c, s);
				HAL_DMA_IRQHandler(&h);
				dmaIrqLogCount = 0;
				hostDmaStepItems(c, s, kLen / 2u);
				break;
			case 4: hostDmaRaise(c, s, HOST_DMA_TEIF); break;
			default: REQUIRE(HAL_DMA_Abort_IT(&h) == HAL_OK); break;
			}
			HAL_DMA_IRQHandler(&h);
			CHECK(dmaIrqCount(kEvents[k]) == 1u);
			CHECK(total() == 1u);
		}
	}
}

TEST_CASE_FIXTURE(DmaIrqFx, "DC2 UnRegisterCallback clears the matching member only and returns HAL_OK") {
	for (unsigned k = 0; k < 6u; ++k) {
		SUBCASE(dmaIrqIdName(k).c_str()) {
			DMA_HandleTypeDef h;
			init(h, kDefault); // all six set
			CHECK(HAL_DMA_UnRegisterCallback(&h, kIds[k]) == HAL_OK);
			for (unsigned m = 0; m < 6u; ++m) CHECK((h.*kMembers[m] == nullptr) == (m == k));
		}
	}
}

TEST_CASE_FIXTURE(DmaIrqFx, "DC2 UnRegisterCallback with HAL_DMA_XFER_ALL_CB_ID clears all six members") {
	DMA_HandleTypeDef h;
	init(h, kDefault);
	CHECK(HAL_DMA_UnRegisterCallback(&h, HAL_DMA_XFER_ALL_CB_ID) == HAL_OK);
	CHECK(allNull(h));
}

/* characterization (NEEDS REVIEW DC-1) */
TEST_CASE_FIXTURE(DmaIrqFx, "DC3 characterization: an invalid callback id returns HAL_ERROR and changes nothing") {
	DMA_HandleTypeDef h;
	init(h, kDefault);
	const auto bad = static_cast<HAL_DMA_CallbackIDTypeDef>(7);
	CHECK(HAL_DMA_RegisterCallback(&h, bad, dmaIrqCplt) == HAL_ERROR);
	CHECK(HAL_DMA_UnRegisterCallback(&h, bad) == HAL_ERROR);
	for (unsigned m = 0; m < 6u; ++m) CHECK(h.*kMembers[m] == kCbs[m]);
}

/* characterization (NEEDS REVIEW DC-2) */
TEST_CASE_FIXTURE(DmaIrqFx, "DC3 characterization: RegisterCallback with HAL_DMA_XFER_ALL_CB_ID returns HAL_ERROR") {
	DMA_HandleTypeDef h;
	init(h, kDefault, DMA_NORMAL, kNoCallbacks);
	CHECK(HAL_DMA_RegisterCallback(&h, HAL_DMA_XFER_ALL_CB_ID, dmaIrqCplt) == HAL_ERROR);
	CHECK(allNull(h));
}

/* characterization (NEEDS REVIEW DC-3) */
TEST_CASE_FIXTURE(DmaIrqFx, "DC3 characterization: Register and UnRegister while the state is not READY return HAL_ERROR and leave the handle unlocked") {
	DMA_HandleTypeDef h;
	init(h, kDefault);
	h.State = HAL_DMA_STATE_BUSY;
	CHECK(HAL_DMA_RegisterCallback(&h, HAL_DMA_XFER_CPLT_CB_ID, dmaIrqHalf) == HAL_ERROR);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(h.XferCpltCallback == dmaIrqCplt);
	CHECK(HAL_DMA_UnRegisterCallback(&h, HAL_DMA_XFER_CPLT_CB_ID) == HAL_ERROR);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(h.XferCpltCallback == dmaIrqCplt);
}

/* characterization (NEEDS REVIEW DC-4); the pointer not being stored is plain */
TEST_CASE_FIXTURE(DmaIrqFx, "DC3 characterization: RegisterCallback after Start_IT does not store the pointer and is not HAL_OK") {
	DMA_HandleTypeDef h;
	start(h, kDefault);
	CHECK(HAL_DMA_RegisterCallback(&h, HAL_DMA_XFER_CPLT_CB_ID, dmaIrqHalf) != HAL_OK);
	CHECK(h.XferCpltCallback == dmaIrqCplt);
	CHECK(HAL_DMA_UnRegisterCallback(&h, HAL_DMA_XFER_CPLT_CB_ID) != HAL_OK);
	CHECK(h.XferCpltCallback == dmaIrqCplt);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DC4 the handle is unlocked after RegisterCallback and a following Start works") {
	DMA_HandleTypeDef h;
	init(h, kDefault, DMA_NORMAL, kNoCallbacks);
	REQUIRE(HAL_DMA_RegisterCallback(&h, HAL_DMA_XFER_CPLT_CB_ID, dmaIrqCplt) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(HAL_DMA_Start(&h, kSrc, kDst, kLen) == HAL_OK);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DC4 the handle is unlocked after UnRegisterCallback and a following Start works") {
	DMA_HandleTypeDef h;
	init(h, kDefault);
	REQUIRE(HAL_DMA_UnRegisterCallback(&h, HAL_DMA_XFER_ALL_CB_ID) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(HAL_DMA_Start(&h, kSrc, kDst, kLen) == HAL_OK);
}

TEST_CASE_FIXTURE(DmaIrqFx, "DC4 the handle is unlocked after a failed Register / UnRegister (invalid id) and a following Start works") {
	DMA_HandleTypeDef h;
	init(h, kDefault);
	const auto bad = static_cast<HAL_DMA_CallbackIDTypeDef>(7);
	REQUIRE(HAL_DMA_RegisterCallback(&h, bad, dmaIrqCplt) == HAL_ERROR);
	REQUIRE(HAL_DMA_UnRegisterCallback(&h, bad) == HAL_ERROR);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(HAL_DMA_Start(&h, kSrc, kDst, kLen) == HAL_OK);
}
