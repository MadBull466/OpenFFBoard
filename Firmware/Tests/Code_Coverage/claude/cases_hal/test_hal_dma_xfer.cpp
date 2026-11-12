/*
 * P12 T3 groups DS and DX: HAL_DMA_Start, Start_IT, Abort, Abort_IT, PollForTransfer, GetState, GetError
 * (stm32f4xx_hal_dma.c) and HAL_DMAEx_MultiBufferStart, _IT, ChangeMemory (stm32f4xx_hal_dma_ex.c) against the DMA
 * register model (fakes/stm32_hal/HalDma.h). Requirements DS1-DS9 and DX1-DX2 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T3_REQUIREMENTS.md; compiled into ffb_hal_tests (F407VG) and
 * ffb_hal_tests_disco (F407VG_DISCO).
 *
 * Oracles: UM1725 Rev 8 (21 DMA, 22 DMAEx: notes, error code list) and RM0090 Rev 22 chapter 10:
 *  - Table 45 (10.3.6): DIR 00 peripheral-to-memory: source DMA_SxPAR, destination DMA_SxM0AR; DIR 01 memory-to-peripheral:
 *    source DMA_SxM0AR, destination DMA_SxPAR; DIR 10 memory-to-memory: source DMA_SxPAR, destination DMA_SxM0AR.
 *  - Table 46 (10.3.9): double buffer mode: the memory is addressed by M0AR / M1AR, memory-to-memory is not allowed; CIRC is
 *    then automatic. M1AR may be written when CT = 0, M0AR when CT = 1.
 *  - 10.5.5 DMA_SxCR: EN bit 0, DMEIE 1, TEIE 2, HTIE 3, TCIE 4, CIRC 8, DBM 18, CT 19.
 *  - 10.5.6 DMA_SxNDTR: NDT is a 16 bit field (0 .. 65535).
 *  - 10.5.10 DMA_SxFCR: DMDIS bit 2 (1 = direct mode disabled, FIFO in use), FEIE bit 7.
 *  - 10.5.1-10.5.4: flag layout inside a stream's group: FEIF bit 0, DMEIF 2, TEIF 3, HTIF 4, TCIF 5.
 * The HAL API enumerations (DMA_CHANNEL_*, DMA_PERIPH_TO_MEMORY, HAL_DMA_FULL_TRANSFER, ...) are inputs only.
 * Time is the HAL tick: every HAL_GetTick() call advances it by 1 ms (HalHostSystem.h), so a timeout ends by itself.
 * Every file-local symbol is prefixed dmaXfer (the other T3 test files declare their own handles and callbacks).
 */
#include "HalDma.h"
#include "HalFixture.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace {

constexpr uint32_t dmaXferEN = 1u << 0;
constexpr uint32_t dmaXferDMEIE = 1u << 1;
constexpr uint32_t dmaXferTEIE = 1u << 2;
constexpr uint32_t dmaXferHTIE = 1u << 3;
constexpr uint32_t dmaXferTCIE = 1u << 4;
constexpr uint32_t dmaXferCIRC = 1u << 8;
constexpr uint32_t dmaXferDBM = 1u << 18;
constexpr uint32_t dmaXferCT = 1u << 19;
constexpr uint32_t dmaXferALLIE = dmaXferDMEIE | dmaXferTEIE | dmaXferHTIE | dmaXferTCIE;
constexpr uint32_t dmaXferDMDIS = 1u << 2;
constexpr uint32_t dmaXferFEIE = 1u << 7;
constexpr uint32_t dmaXferAllFlags = HOST_DMA_FEIF | HOST_DMA_DMEIF | HOST_DMA_TEIF | HOST_DMA_HTIF | HOST_DMA_TCIF;

constexpr uint32_t dmaXferPeriph = 0x40013804u; // an APB2 peripheral data register address
constexpr uint32_t dmaXferMem = 0x20003000u;    // SRAM
constexpr uint32_t dmaXferMem2 = 0x20004000u;

struct dmaXferStream {
	unsigned c;
	unsigned s;
	DMA_Stream_TypeDef *inst;
	std::string name;
};

dmaXferStream dmaXferMake(unsigned c, unsigned s) {
	DMA_Stream_TypeDef *const d1[8] = {DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3,
	                                   DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7};
	DMA_Stream_TypeDef *const d2[8] = {DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3,
	                                   DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7};
	return {c, s, c == 0 ? d1[s] : d2[s], "DMA" + std::to_string(c + 1) + " stream " + std::to_string(s)};
}

/* Streams 0, 3, 4, 7 of both controllers (flag group offsets 0, 22, 0, 22 in LISR and HISR). */
std::vector<dmaXferStream> dmaXferLayoutStreams() {
	std::vector<dmaXferStream> v;
	for (unsigned c = 0; c < 2; ++c)
		for (unsigned s : {0u, 3u, 4u, 7u}) v.push_back(dmaXferMake(c, s));
	return v;
}

int dmaXferCplt = 0, dmaXferHalf = 0, dmaXferErr = 0, dmaXferAbortCb = 0;
unsigned dmaXferStepCount = 0, dmaXferStepAt = 0;
unsigned dmaXferStepC = 0, dmaXferStepS = 0, dmaXferRaiseAt = 0, dmaXferRaiseMask = 0;

void dmaXferOnCplt(DMA_HandleTypeDef *) { ++dmaXferCplt; }
void dmaXferOnHalf(DMA_HandleTypeDef *) { ++dmaXferHalf; }
void dmaXferOnErr(DMA_HandleTypeDef *) { ++dmaXferErr; }
void dmaXferOnAbort(DMA_HandleTypeDef *) { ++dmaXferAbortCb; }

/* Tick step: raises the flags dmaXferRaiseMask of stream (dmaXferStepC, dmaXferStepS) at the dmaXferRaiseAt-th step and
 * completes its transfer at the dmaXferStepAt-th step (the transfer engine working while the HAL polls). */
void dmaXferStep() {
	HAL_IncTick();
	++dmaXferStepCount;
	if (dmaXferStepCount == dmaXferRaiseAt) hostDmaRaise(dmaXferStepC, dmaXferStepS, dmaXferRaiseMask);
	if (dmaXferStepCount == dmaXferStepAt) hostDmaComplete(dmaXferStepC, dmaXferStepS);
}

struct DmaXferFx : HalFixture {
	DmaXferFx() {
		dmaXferCplt = dmaXferHalf = dmaXferErr = dmaXferAbortCb = 0;
		dmaXferStepCount = 0;
		dmaXferStepAt = 0;
		dmaXferRaiseAt = 0;
		dmaXferRaiseMask = 0;
	}

	/* Initialised handle: byte transfers, memory increment, direct mode (FIFO off); direction and mode as asked. */
	static void make(DMA_HandleTypeDef &h, const dmaXferStream &st, uint32_t dir = DMA_PERIPH_TO_MEMORY,
	                 uint32_t mode = DMA_NORMAL, bool fifo = false) {
		h = DMA_HandleTypeDef{};
		h.Instance = st.inst;
		h.Init.Channel = DMA_CHANNEL_0;
		h.Init.Direction = dir;
		h.Init.PeriphInc = DMA_PINC_DISABLE;
		h.Init.MemInc = DMA_MINC_ENABLE;
		h.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		h.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		h.Init.Mode = mode;
		h.Init.Priority = DMA_PRIORITY_LOW;
		h.Init.FIFOMode = fifo ? DMA_FIFOMODE_ENABLE : DMA_FIFOMODE_DISABLE;
		h.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		h.Init.MemBurst = DMA_MBURST_SINGLE;
		h.Init.PeriphBurst = DMA_PBURST_SINGLE;
		REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	}
	static void callbacks(DMA_HandleTypeDef &h, bool half = true) {
		h.XferCpltCallback = dmaXferOnCplt;
		h.XferM1CpltCallback = dmaXferOnCplt;
		h.XferHalfCpltCallback = half ? dmaXferOnHalf : nullptr;
		h.XferErrorCallback = dmaXferOnErr;
		h.XferAbortCallback = dmaXferOnAbort;
	}
	static uint32_t cr(const dmaXferStream &st) { return peek(&st.inst->CR); }
	static uint32_t fcr(const dmaXferStream &st) { return peek(&st.inst->FCR); }
	static bool noProtectedWrites(const dmaXferStream &st) { return hostDmaProtectedWrites(st.c, st.s) == 0u; }

	/* True if the last write the CPU made after log position from is the write that sets EN of the stream, and all
	 * writes of NDTR, PAR, M0AR and M1AR happened before it. */
	static bool enableWrittenLast(size_t from, const dmaXferStream &st) {
		const size_t n = hostRegsLogCount();
		size_t lastWrite = n, enIdx = n, lastData = n;
		bool anyData = false;
		for (size_t i = from; i < n; ++i) {
			const HostRegAccess *e = hostRegsLogAt(i);
			if (!e->write) continue;
			lastWrite = i;
			const uintptr_t a = e->addr;
			if (a == reinterpret_cast<uintptr_t>(&st.inst->CR) && (e->written & dmaXferEN)) enIdx = i;
			if (a == reinterpret_cast<uintptr_t>(&st.inst->NDTR) || a == reinterpret_cast<uintptr_t>(&st.inst->PAR) ||
			    a == reinterpret_cast<uintptr_t>(&st.inst->M0AR) || a == reinterpret_cast<uintptr_t>(&st.inst->M1AR)) {
				lastData = i;
				anyData = true;
			}
		}
		return enIdx != n && lastWrite == enIdx && anyData && lastData < enIdx;
	}
};

} // namespace

TEST_SUITE("P12 T3 DMA start abort poll") {

/* ---------------------------------------------------------------- DS1 */

TEST_CASE_FIXTURE(DmaXferFx, "DS1 Start peripheral to memory puts the source in PAR and the destination in M0AR") {
	// RM0090 Table 45: DIR 00: source DMA_SxPAR, destination DMA_SxM0AR.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st, DMA_PERIPH_TO_MEMORY);
			CHECK(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 100) == HAL_OK);
			CHECK(peek(&st.inst->PAR) == dmaXferPeriph);
			CHECK(peek(&st.inst->M0AR) == dmaXferMem);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS1 Start memory to peripheral puts the source in M0AR and the destination in PAR") {
	// RM0090 Table 45: DIR 01: source DMA_SxM0AR, destination DMA_SxPAR.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st, DMA_MEMORY_TO_PERIPH);
			CHECK(HAL_DMA_Start(&h, dmaXferMem, dmaXferPeriph, 100) == HAL_OK);
			CHECK(peek(&st.inst->M0AR) == dmaXferMem);
			CHECK(peek(&st.inst->PAR) == dmaXferPeriph);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS1 Start memory to memory puts the source in PAR and the destination in M0AR") {
	// RM0090 Table 45: DIR 10: source DMA_SxPAR, destination DMA_SxM0AR.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st, DMA_MEMORY_TO_MEMORY, DMA_NORMAL, true);
			CHECK(HAL_DMA_Start(&h, dmaXferMem, dmaXferMem2, 100) == HAL_OK);
			CHECK(peek(&st.inst->PAR) == dmaXferMem);
			CHECK(peek(&st.inst->M0AR) == dmaXferMem2);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS1 Start writes the data length to NDTR") {
	// RM0090 10.5.6: NDT holds the number of data items to transfer.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 1234) == HAL_OK);
			CHECK((peek(&st.inst->NDTR) & 0xFFFFu) == 1234u);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS1 Start enables the stream and sets the state to BUSY") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
			CHECK((cr(st) & dmaXferEN) == dmaXferEN); // RM0090 10.5.5 EN = 1
			CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_BUSY); // UM1725 21.2.4: GetState returns the DMA state (BUSY while a transfer runs)
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS1 Start writes EN after NDTR, PAR and M0AR") {
	// RM0090 10.5.5-10.5.9: NDTR, PAR and M0AR are write protected while EN = 1, so EN is the last write.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st, DMA_MEMORY_TO_PERIPH);
			const size_t from = hostRegsLogCount();
			REQUIRE(HAL_DMA_Start(&h, dmaXferMem, dmaXferPeriph, 10) == HAL_OK);
			CHECK(enableWrittenLast(from, st));
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS1 Start enables none of the interrupts") {
	// UM1725 21.2.1: the interrupt enables come with HAL_DMA_Start_IT only.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st, DMA_PERIPH_TO_MEMORY, DMA_NORMAL, true);
			callbacks(h);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
			CHECK((cr(st) & dmaXferALLIE) == 0u);
			CHECK((fcr(st) & dmaXferFEIE) == 0u);
			CHECK(noProtectedWrites(st));
		}
	}
}

/* ---------------------------------------------------------------- DS2 */

TEST_CASE_FIXTURE(DmaXferFx, "DS2 Start_IT sets the same addresses, length and enable as Start") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st, DMA_MEMORY_TO_PERIPH);
			callbacks(h);
			const size_t from = hostRegsLogCount();
			REQUIRE(HAL_DMA_Start_IT(&h, dmaXferMem, dmaXferPeriph, 77) == HAL_OK);
			CHECK(peek(&st.inst->M0AR) == dmaXferMem);   // Table 45, DIR 01
			CHECK(peek(&st.inst->PAR) == dmaXferPeriph); // Table 45, DIR 01
			CHECK((peek(&st.inst->NDTR) & 0xFFFFu) == 77u);
			CHECK((cr(st) & dmaXferEN) == dmaXferEN);
			CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_BUSY);
			CHECK(enableWrittenLast(from, st));
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS2 Start_IT sets TCIE, TEIE and DMEIE") {
	// RM0090 10.5.5 and 10.4 Table 51: transfer complete, transfer error and direct mode error interrupt enables.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			callbacks(h);
			REQUIRE(HAL_DMA_Start_IT(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
			CHECK((cr(st) & dmaXferTCIE) == dmaXferTCIE);
			CHECK((cr(st) & dmaXferTEIE) == dmaXferTEIE);
			CHECK((cr(st) & dmaXferDMEIE) == dmaXferDMEIE);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS2 Start_IT sets HTIE when a half transfer callback is set") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			callbacks(h, true);
			REQUIRE(HAL_DMA_Start_IT(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
			CHECK((cr(st) & dmaXferHTIE) == dmaXferHTIE); // RM0090 10.5.5 HTIE bit 3
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS2 characterization: Start_IT leaves HTIE clear when no half transfer callback is set") {
	// NEEDS REVIEW (proposed): UM1725 and RM0090 do not say whether the half transfer interrupt is enabled without a
	// callback. The HAL leaves HTIE clear then. Open: accepted behaviour, or must HTIE always be set?
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			callbacks(h, false);
			REQUIRE(HAL_DMA_Start_IT(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
			CHECK((cr(st) & dmaXferHTIE) == 0u);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS2 characterization: Start_IT leaves FEIE clear although direct mode is disabled") {
	// NEEDS REVIEW (proposed, lead decision): the charter expects FEIE (RM0090 10.5.10 bit 7) to be set by Start_IT when
	// the FIFO is in use (DMDIS = 1), so that FIFO errors (FEIF) raise an interrupt. UM1725 does not say. The HAL never sets
	// FEIE in Start_IT, whereas HAL_DMAEx_MultiBufferStart_IT sets it (DX1), so the two are inconsistent and a FIFO
	// error of a Start_IT transfer reaches no error callback (DQ4). Candidate vendor bug (number from the lead).
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st, DMA_PERIPH_TO_MEMORY, DMA_NORMAL, true);
			callbacks(h);
			REQUIRE((fcr(st) & dmaXferDMDIS) == dmaXferDMDIS);
			REQUIRE(HAL_DMA_Start_IT(&h, dmaXferPeriph, dmaXferMem, 16) == HAL_OK);
			CHECK((fcr(st) & dmaXferFEIE) == 0u);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS2 characterization: Start_IT leaves FEIE clear in direct mode") {
	// NEEDS REVIEW (proposed): neither document says whether FEIE is set in direct mode (no FIFO errors occur there,
	// RM0090 10.3.? FIFO). The HAL leaves FEIE clear.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st, DMA_PERIPH_TO_MEMORY, DMA_NORMAL, false);
			callbacks(h);
			REQUIRE(HAL_DMA_Start_IT(&h, dmaXferPeriph, dmaXferMem, 16) == HAL_OK);
			CHECK((fcr(st) & dmaXferFEIE) == 0u);
			CHECK(noProtectedWrites(st));
		}
	}
}

/* ---------------------------------------------------------------- DS3 */

TEST_CASE_FIXTURE(DmaXferFx, "DS3 Start of a stream that is BUSY returns HAL_BUSY") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
			CHECK(HAL_DMA_Start(&h, dmaXferPeriph + 4u, dmaXferMem2, 20) == HAL_BUSY);
			CHECK(HAL_DMA_Start_IT(&h, dmaXferPeriph + 4u, dmaXferMem2, 20) == HAL_BUSY);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS3 Start of a stream that is BUSY does not touch the registers") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
			const uint32_t crBefore = cr(st);
			HAL_DMA_Start(&h, dmaXferPeriph + 4u, dmaXferMem2, 20);
			CHECK(peek(&st.inst->PAR) == dmaXferPeriph);
			CHECK(peek(&st.inst->M0AR) == dmaXferMem);
			CHECK((peek(&st.inst->NDTR) & 0xFFFFu) == 10u);
			CHECK(cr(st) == crBefore);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS3 Start_IT of a stream that is BUSY returns HAL_BUSY and leaves the registers alone") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			callbacks(h);
			REQUIRE(HAL_DMA_Start_IT(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
			const uint32_t crBefore = cr(st);
			const uint32_t protBefore = hostDmaProtectedWrites(st.c, st.s);
			CHECK(HAL_DMA_Start_IT(&h, dmaXferPeriph + 4u, dmaXferMem2, 20) == HAL_BUSY);
			CHECK(peek(&st.inst->PAR) == dmaXferPeriph);
			CHECK(peek(&st.inst->M0AR) == dmaXferMem);
			CHECK((peek(&st.inst->NDTR) & 0xFFFFu) == 10u);
			CHECK(cr(st) == crBefore);
			CHECK(hostDmaProtectedWrites(st.c, st.s) == protBefore);
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS3 characterization: Start_IT refused with HAL_BUSY leaves the handle unlocked") {
	// NEEDS REVIEW (proposed): UM1725 does not describe the lock. The HAL locks the handle, finds the state not READY,
	// unlocks it again and returns HAL_BUSY (so the running transfer can still be polled or aborted).
	const dmaXferStream st = dmaXferMake(1, 6);
	DMA_HandleTypeDef h;
	make(h, st);
	callbacks(h);
	REQUIRE(HAL_DMA_Start_IT(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	h.Lock = HAL_UNLOCKED;
	REQUIRE(HAL_DMA_Start_IT(&h, dmaXferPeriph, dmaXferMem2, 20) == HAL_BUSY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_BUSY);
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 MultiBufferStart_IT of a stream that is BUSY returns HAL_BUSY and leaves the registers alone") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			callbacks(h);
			REQUIRE(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
			const uint32_t crBefore = cr(st);
			const uint32_t protBefore = hostDmaProtectedWrites(st.c, st.s);
			CHECK(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferPeriph + 4u, dmaXferMem2, dmaXferMem, 32) == HAL_BUSY);
			CHECK(peek(&st.inst->PAR) == dmaXferPeriph);
			CHECK(peek(&st.inst->M0AR) == dmaXferMem);
			CHECK(peek(&st.inst->M1AR) == dmaXferMem2);
			CHECK((peek(&st.inst->NDTR) & 0xFFFFu) == 64u);
			CHECK(cr(st) == crBefore);
			CHECK(hostDmaProtectedWrites(st.c, st.s) == protBefore);
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 characterization: MultiBufferStart_IT refused with HAL_BUSY leaves the handle unlocked") {
	// NEEDS REVIEW (proposed): as for Start_IT (DS3), UM1725 does not describe the lock; the HAL unlocks before HAL_BUSY.
	const dmaXferStream st = dmaXferMake(0, 6);
	DMA_HandleTypeDef h;
	make(h, st);
	callbacks(h);
	REQUIRE(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
	h.Lock = HAL_UNLOCKED;
	REQUIRE(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferPeriph, dmaXferMem2, dmaXferMem, 32) == HAL_BUSY);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(DmaXferFx, "DS5 characterization: Abort of a Start_IT transfer with a half callback clears TCIE, TEIE, DMEIE and HTIE") {
	// NEEDS REVIEW (proposed): UM1725 does not say which interrupt enables Abort clears. The HAL clears the four enables
	// (including HTIE, which Start_IT set because a half callback exists) and FEIE, so no interrupt follows the abort.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			callbacks(h, true);
			REQUIRE(HAL_DMA_Start_IT(&h, dmaXferPeriph, dmaXferMem, 100) == HAL_OK);
			REQUIRE((cr(st) & dmaXferALLIE) == dmaXferALLIE);
			REQUIRE(HAL_DMA_Abort(&h) == HAL_OK);
			CHECK((cr(st) & dmaXferALLIE) == 0u);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS3 characterization: Start of a handle that was never initialised returns HAL_BUSY and writes no register") {
	// NEEDS REVIEW (proposed): UM1725 21.2.1 puts HAL_DMA_Init first but gives no status for a start before it. The HAL refuses
	// a start while the state is not READY (RESET here) with HAL_BUSY and writes no register.
	const dmaXferStream st = dmaXferMake(0, 2);
	DMA_HandleTypeDef h = {};
	h.Instance = st.inst;
	REQUIRE(HAL_DMA_GetState(&h) == HAL_DMA_STATE_RESET);
	const size_t from = hostRegsLogCount();
	CHECK(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_BUSY);
	size_t writes = 0;
	for (size_t i = from; i < hostRegsLogCount(); ++i) writes += hostRegsLogAt(i)->write ? 1u : 0u;
	CHECK(writes == 0u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS3 characterization: Start while the handle is locked returns HAL_BUSY") {
	// NEEDS REVIEW (proposed): UM1725 does not describe the process lock of the DMA handle. The HAL refuses a start
	// while Lock == HAL_LOCKED even though the state is READY, and writes no register.
	const dmaXferStream st = dmaXferMake(1, 3);
	DMA_HandleTypeDef h;
	make(h, st);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_BUSY);
	CHECK((peek(&st.inst->NDTR) & 0xFFFFu) == 0u);
	CHECK((cr(st) & dmaXferEN) == 0u);
	CHECK(noProtectedWrites(st));
}

/* ---------------------------------------------------------------- DS4 */

TEST_CASE_FIXTURE(DmaXferFx, "DS4 Start with the largest NDT value 65535 writes 65535 to NDTR") {
	// RM0090 10.5.6: NDT is 16 bits, 0 .. 65535.
	const dmaXferStream st = dmaXferMake(0, 7);
	DMA_HandleTypeDef h;
	make(h, st);
	CHECK(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 65535) == HAL_OK);
	CHECK((peek(&st.inst->NDTR) & 0xFFFFu) == 65535u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS4 Start with the smallest transfer of one item writes 1 to NDTR") {
	const dmaXferStream st = dmaXferMake(1, 0);
	DMA_HandleTypeDef h;
	make(h, st);
	CHECK(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 1) == HAL_OK);
	CHECK((peek(&st.inst->NDTR) & 0xFFFFu) == 1u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS4 characterization: Start with DataLength 0 returns HAL_OK and leaves the state BUSY") {
	// NEEDS REVIEW (proposed): RM0090 10.5.6 says a stream with NDT = 0 does not start (it stays idle with EN set); UM1725
	// gives no status for a zero length. The HAL accepts it (HAL_OK) and reports BUSY, although nothing will ever
	// complete. Open: should the HAL return HAL_ERROR?
	const dmaXferStream st = dmaXferMake(0, 3);
	DMA_HandleTypeDef h;
	make(h, st);
	CHECK(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 0) == HAL_OK);
	CHECK((peek(&st.inst->NDTR) & 0xFFFFu) == 0u);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_BUSY);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS4 characterization: Start with DataLength 65536 returns HAL_OK") {
	// NEEDS REVIEW (proposed): 65536 does not fit the 16 bit NDT field (RM0090 10.5.6: 0 .. 65535). UM1725 gives no
	// parameter check. The HAL accepts the length (HAL_OK); what the register then holds is the model's truncation and
	// is not asserted. Open: should the HAL return HAL_ERROR?
	const dmaXferStream st = dmaXferMake(1, 4);
	DMA_HandleTypeDef h;
	make(h, st);
	CHECK(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 65536) == HAL_OK);
	CHECK(noProtectedWrites(st));
}

/* ---------------------------------------------------------------- DS5 */

TEST_CASE_FIXTURE(DmaXferFx, "DS5 Abort of a running transfer returns HAL_OK and clears EN") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 100) == HAL_OK);
			CHECK(HAL_DMA_Abort(&h) == HAL_OK);
			CHECK((cr(st) & dmaXferEN) == 0u); // RM0090 10.5.5
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS5 Abort sets the state to READY and unlocks the handle") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 100) == HAL_OK);
			REQUIRE(HAL_DMA_Abort(&h) == HAL_OK);
			CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
			CHECK(h.Lock == HAL_UNLOCKED);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS5 Abort allows a new Start of the same stream") {
	const dmaXferStream st = dmaXferMake(1, 5);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 100) == HAL_OK);
	REQUIRE(HAL_DMA_Abort(&h) == HAL_OK);
	CHECK(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem2, 50) == HAL_OK);
	CHECK(peek(&st.inst->M0AR) == dmaXferMem2);
	CHECK((peek(&st.inst->NDTR) & 0xFFFFu) == 50u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS5 Abort clears all flags of the stream") {
	// RM0090 10.3.? (stream configuration procedure): the stream's status flags must be clear before it is enabled again.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 100) == HAL_OK);
			hostDmaRaise(st.c, st.s, HOST_DMA_HTIF | HOST_DMA_FEIF | HOST_DMA_DMEIF);
			REQUIRE(hostDmaFlags(st.c, st.s) == (HOST_DMA_HTIF | HOST_DMA_FEIF | HOST_DMA_DMEIF));
			CHECK(HAL_DMA_Abort(&h) == HAL_OK);
			CHECK(hostDmaFlags(st.c, st.s) == 0u);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS5 Abort leaves the flags of the neighbouring streams alone") {
	// RM0090 10.5.1-10.5.4: six flag bits per stream; stream x touches only its own.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			const unsigned nb = (st.s + 1u) % 8u;
			const unsigned nb2 = (st.s + 7u) % 8u;
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 100) == HAL_OK);
			hostDmaRaise(st.c, nb, dmaXferAllFlags & ~HOST_DMA_TEIF);
			hostDmaRaise(st.c, nb2, dmaXferAllFlags & ~HOST_DMA_TEIF);
			hostDmaRaise(1u - st.c, st.s, dmaXferAllFlags & ~HOST_DMA_TEIF);
			REQUIRE(HAL_DMA_Abort(&h) == HAL_OK);
			CHECK(hostDmaFlags(st.c, nb) == (dmaXferAllFlags & ~HOST_DMA_TEIF));
			CHECK(hostDmaFlags(st.c, nb2) == (dmaXferAllFlags & ~HOST_DMA_TEIF));
			CHECK(hostDmaFlags(1u - st.c, st.s) == (dmaXferAllFlags & ~HOST_DMA_TEIF));
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS5 Abort waits until the stream really stops") {
	// UM1725 (Abort note): the stream is effectively disabled only after the current data item is transferred, and Abort
	// waits for that. The model keeps EN = 1 for 3 ticks (RM0090 10.5.5: EN is cleared by hardware when done).
	const dmaXferStream st = dmaXferMake(0, 4);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 100) == HAL_OK);
	hostDmaSetDisableDelay(3);
	const uint32_t t0 = hostTickPeek();
	CHECK(HAL_DMA_Abort(&h) == HAL_OK);
	const uint32_t elapsed = hostTickPeek() - t0;
	CHECK(elapsed >= 3u); // not before EN reads 0
	CHECK(elapsed < 5u);  // and far before the HAL timeout
	CHECK((cr(st) & dmaXferEN) == 0u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS5 Abort with no transfer ongoing sets the error code NO_XFER") {
	// UM1725 DMA Error Code: HAL_DMA_ERROR_NO_XFER = "Abort requested with no Xfer ongoing".
	const dmaXferStream st = dmaXferMake(1, 2);
	DMA_HandleTypeDef h;
	make(h, st);
	HAL_DMA_Abort(&h);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_NO_XFER);
}

TEST_CASE_FIXTURE(DmaXferFx, "DS5 characterization: Abort with no transfer ongoing returns HAL_ERROR") {
	// NEEDS REVIEW (proposed): UM1725 lists the error code but gives no return status for an abort without a transfer.
	// The HAL returns HAL_ERROR (and leaves the stream disabled).
	const dmaXferStream st = dmaXferMake(1, 2);
	DMA_HandleTypeDef h;
	make(h, st);
	CHECK(HAL_DMA_Abort(&h) == HAL_ERROR);
	CHECK((cr(st) & dmaXferEN) == 0u);
}

TEST_CASE_FIXTURE(DmaXferFx, "DS5 characterization: Abort of a stream whose EN never clears times out after about 5 ms") {
	// NEEDS REVIEW (proposed): UM1725 notes that Abort waits for the stream to stop but names neither a timeout nor its
	// status. The HAL waits more than 5 ms (the model ticks 1 ms per HAL_GetTick call) and then returns HAL_TIMEOUT with
	// the error code TIMEOUT and the state TIMEOUT (HAL_DMA_STATE_TIMEOUT, not ERROR as the charter assumed).
	// Lower bound 5 ticks, upper bound 8 ticks.
	const dmaXferStream st = dmaXferMake(0, 0);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 100) == HAL_OK);
	hostDmaStickEnabled(st.c, st.s, 1);
	const uint32_t t0 = hostTickPeek();
	CHECK(HAL_DMA_Abort(&h) == HAL_TIMEOUT);
	const uint32_t elapsed = hostTickPeek() - t0;
	CHECK(elapsed >= 5u);
	CHECK(elapsed <= 8u);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_TIMEOUT);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_TIMEOUT);
	CHECK(noProtectedWrites(st));
}

/* ---------------------------------------------------------------- DS6 */

TEST_CASE_FIXTURE(DmaXferFx, "DS6 Abort_IT of a running transfer returns HAL_OK and clears EN") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			callbacks(h);
			REQUIRE(HAL_DMA_Start_IT(&h, dmaXferPeriph, dmaXferMem, 100) == HAL_OK);
			CHECK(HAL_DMA_Abort_IT(&h) == HAL_OK);
			CHECK((cr(st) & dmaXferEN) == 0u); // RM0090 10.5.5 (the model stops at once)
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS6 Abort_IT does not call the abort callback itself") {
	// UM1725 21.2.3: the abort completes in interrupt mode, the callback comes from the IRQ handler.
	const dmaXferStream st = dmaXferMake(0, 3);
	DMA_HandleTypeDef h;
	make(h, st);
	callbacks(h);
	REQUIRE(HAL_DMA_Start_IT(&h, dmaXferPeriph, dmaXferMem, 100) == HAL_OK);
	REQUIRE(HAL_DMA_Abort_IT(&h) == HAL_OK);
	CHECK(dmaXferAbortCb == 0);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS6 the IRQ handler calls the abort callback once after Abort_IT once EN is 0") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			callbacks(h);
			REQUIRE(HAL_DMA_Start_IT(&h, dmaXferPeriph, dmaXferMem, 100) == HAL_OK);
			REQUIRE(HAL_DMA_Abort_IT(&h) == HAL_OK);
			REQUIRE((cr(st) & dmaXferEN) == 0u);
			// RM0090 10.3.14: when the stream stops after EN was cleared by software, TCIF is set to indicate the end of
			// the transfer (the model does this); the IRQ handler completes the abort on it.
			REQUIRE((hostDmaFlags(st.c, st.s) & HOST_DMA_TCIF) == HOST_DMA_TCIF);
			HAL_DMA_IRQHandler(&h);
			CHECK(dmaXferAbortCb == 1);
			CHECK(dmaXferCplt == 0);
			CHECK(dmaXferErr == 0);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS6 after the abort callback the state is READY and the handle is unlocked") {
	const dmaXferStream st = dmaXferMake(1, 7);
	DMA_HandleTypeDef h;
	make(h, st);
	callbacks(h);
	REQUIRE(HAL_DMA_Start_IT(&h, dmaXferPeriph, dmaXferMem, 100) == HAL_OK);
	REQUIRE(HAL_DMA_Abort_IT(&h) == HAL_OK);
	REQUIRE((hostDmaFlags(st.c, st.s) & HOST_DMA_TCIF) == HOST_DMA_TCIF); // RM0090 10.3.14
	HAL_DMA_IRQHandler(&h);
	REQUIRE(dmaXferAbortCb == 1);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS6 Abort_IT with no transfer ongoing sets the error code NO_XFER") {
	const dmaXferStream st = dmaXferMake(0, 5);
	DMA_HandleTypeDef h;
	make(h, st);
	callbacks(h);
	HAL_DMA_Abort_IT(&h);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_NO_XFER); // UM1725 DMA Error Code list
	CHECK(dmaXferAbortCb == 0);
}

TEST_CASE_FIXTURE(DmaXferFx, "DS6 characterization: Abort_IT with no transfer ongoing returns HAL_ERROR") {
	// NEEDS REVIEW (proposed): UM1725 gives the error code but no return status. The HAL returns HAL_ERROR.
	const dmaXferStream st = dmaXferMake(0, 5);
	DMA_HandleTypeDef h;
	make(h, st);
	callbacks(h);
	CHECK(HAL_DMA_Abort_IT(&h) == HAL_ERROR);
}

/* ---------------------------------------------------------------- DS7 */

TEST_CASE_FIXTURE(DmaXferFx, "DS7 PollForTransfer returns HAL_OK when the transfer is complete") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
			hostDmaComplete(st.c, st.s);
			CHECK(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 10) == HAL_OK);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 PollForTransfer clears TCIF and sets the state READY and unlocks") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
			hostDmaComplete(st.c, st.s);
			REQUIRE((hostDmaFlags(st.c, st.s) & HOST_DMA_TCIF) != 0u);
			REQUIRE(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 10) == HAL_OK);
			CHECK((hostDmaFlags(st.c, st.s) & HOST_DMA_TCIF) == 0u);
			CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
			CHECK(h.Lock == HAL_UNLOCKED);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 PollForTransfer clears only the TCIF of its own stream") {
	const dmaXferStream st = dmaXferMake(1, 3);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	hostDmaRaise(1, 2, HOST_DMA_TCIF);
	hostDmaRaise(1, 4, HOST_DMA_TCIF);
	hostDmaRaise(0, 3, HOST_DMA_TCIF);
	hostDmaComplete(st.c, st.s);
	REQUIRE(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 10) == HAL_OK);
	CHECK(hostDmaFlags(1, 2) == HOST_DMA_TCIF);
	CHECK(hostDmaFlags(1, 4) == HOST_DMA_TCIF);
	CHECK(hostDmaFlags(0, 3) == HOST_DMA_TCIF);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 PollForTransfer at the half transfer level returns HAL_OK on HTIF") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
			hostDmaStepItems(st.c, st.s, 5); // RM0090 10.3.? : HTIF at half of NDT
			REQUIRE((hostDmaFlags(st.c, st.s) & HOST_DMA_HTIF) != 0u);
			CHECK(HAL_DMA_PollForTransfer(&h, HAL_DMA_HALF_TRANSFER, 10) == HAL_OK);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 PollForTransfer at the half transfer level clears HTIF and keeps the state BUSY") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
			hostDmaStepItems(st.c, st.s, 5);
			REQUIRE(HAL_DMA_PollForTransfer(&h, HAL_DMA_HALF_TRANSFER, 10) == HAL_OK);
			CHECK((hostDmaFlags(st.c, st.s) & HOST_DMA_HTIF) == 0u);
			CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_BUSY); // the transfer is not finished
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 a transfer polled at half level can then be polled to its end") {
	const dmaXferStream st = dmaXferMake(0, 7);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	hostDmaStepItems(st.c, st.s, 5);
	REQUIRE(HAL_DMA_PollForTransfer(&h, HAL_DMA_HALF_TRANSFER, 10) == HAL_OK);
	hostDmaStepItems(st.c, st.s, 5);
	CHECK(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 10) == HAL_OK);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 PollForTransfer returns HAL_TIMEOUT after the timeout when nothing happens") {
	// Timeout 10 ms: the poll must not end before 10 ticks and must end within a few ticks after.
	const dmaXferStream st = dmaXferMake(1, 0);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	const uint32_t t0 = hostTickPeek();
	CHECK(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 10) == HAL_TIMEOUT);
	const uint32_t elapsed = hostTickPeek() - t0;
	CHECK(elapsed >= 10u);
	CHECK(elapsed <= 13u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 PollForTransfer with a timeout sets the error code TIMEOUT") {
	// UM1725 DMA Error Code: HAL_DMA_ERROR_TIMEOUT = "Timeout error".
	const dmaXferStream st = dmaXferMake(1, 0);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	REQUIRE(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 5) == HAL_TIMEOUT);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_TIMEOUT);
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 characterization: a poll timeout sets the state READY and unlocks the handle") {
	// NEEDS REVIEW (proposed): UM1725 says nothing about the state after a poll timeout. The HAL sets READY and unlocks.
	const dmaXferStream st = dmaXferMake(0, 7);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	REQUIRE(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 5) == HAL_TIMEOUT);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 after a poll timeout the handle is not READY while the stream is still enabled" *
                                 KNOWN_ISSUE("candidate vendor bug V95: after a poll timeout HAL_DMA_PollForTransfer reports READY but leaves the stream enabled (RM0090 10.5.5-10.5.9: the registers stay write-protected)")) {
	// READY with EN = 1 means the handle can be neither polled again (NO_XFER) nor started (NDTR / PAR / M0AR are write
	// protected while EN = 1, RM0090 10.5.6-10.5.9). The first assertion is the contradiction itself.
	const dmaXferStream st = dmaXferMake(1, 5);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	REQUIRE(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 5) == HAL_TIMEOUT);
	CHECK(!(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY && (cr(st) & dmaXferEN) != 0u));
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem2, 20) == HAL_OK);
	CHECK(peek(&st.inst->M0AR) == dmaXferMem2);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 PollForTransfer with HAL_MAX_DELAY returns HAL_OK for a completed transfer") {
	// Here the transfer is already complete before the poll (the waiting case is the next one).
	const dmaXferStream st = dmaXferMake(0, 4);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	hostDmaComplete(st.c, st.s);
	CHECK(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY) == HAL_OK);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 PollForTransfer with HAL_MAX_DELAY waits until the transfer completes") {
	// The HAL never calls HAL_GetTick with HAL_MAX_DELAY, so the transfer completes after 20 reads of the flag registers.
	const dmaXferStream st = dmaXferMake(1, 3);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	hostDmaCompleteAfterReads(st.c, st.s, 20);
	CHECK(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY) == HAL_OK);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
	CHECK((hostDmaFlags(st.c, st.s) & HOST_DMA_TCIF) == 0u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 characterization: PollForTransfer with timeout 0 and no completion returns HAL_TIMEOUT") {
	// NEEDS REVIEW (proposed): UM1725 does not define Timeout = 0. The HAL treats it as "do not wait": HAL_TIMEOUT at once.
	const dmaXferStream st = dmaXferMake(0, 4);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	const uint32_t t0 = hostTickPeek();
	CHECK(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 0) == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 <= 2u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 PollForTransfer returns HAL_OK when the transfer completes while it waits") {
	const dmaXferStream st = dmaXferMake(0, 4);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	dmaXferStepC = st.c;
	dmaXferStepS = st.s;
	dmaXferStepAt = 6;
	hostTickSetStep(dmaXferStep);
	const uint32_t t0 = hostTickPeek();
	CHECK(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 50) == HAL_OK);
	CHECK(hostTickPeek() - t0 >= 5u); // it waited for the completion
	CHECK(hostTickPeek() - t0 < 50u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 a transfer error flag during polling returns HAL_ERROR with the error code TE") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
			hostDmaRaise(st.c, st.s, HOST_DMA_TEIF);
			CHECK(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 20) == HAL_ERROR);
			CHECK((HAL_DMA_GetError(&h) & HAL_DMA_ERROR_TE) == HAL_DMA_ERROR_TE);
			CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
			CHECK(h.Lock == HAL_UNLOCKED);
			CHECK((cr(st) & dmaXferEN) == 0u); // RM0090 10.3.18: a transfer error disables the stream
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 a transfer error during polling clears TEIF") {
	const dmaXferStream st = dmaXferMake(1, 3);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	hostDmaRaise(st.c, st.s, HOST_DMA_TEIF);
	REQUIRE(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 20) == HAL_ERROR);
	CHECK((hostDmaFlags(st.c, st.s) & HOST_DMA_TEIF) == 0u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 characterization: a FIFO error flag during polling is recorded and cleared and the poll goes on") {
	// NEEDS REVIEW (proposed, lead decision): the charter expects HAL_ERROR with the error code FE. UM1725 and RM0090 do not
	// say. RM0090 10.3.18 says only a transfer error (TEIF) disables the stream; the HAL treats FE like that: it adds
	// HAL_DMA_ERROR_FE to the error code, clears FEIF and keeps polling, so the poll returns HAL_OK at the end of the
	// transfer with the error code FE set.
	const dmaXferStream st = dmaXferMake(0, 3);
	DMA_HandleTypeDef h;
	make(h, st, DMA_PERIPH_TO_MEMORY, DMA_NORMAL, true);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 16) == HAL_OK);
	dmaXferStepC = st.c;
	dmaXferStepS = st.s;
	dmaXferRaiseAt = 2;
	dmaXferRaiseMask = HOST_DMA_FEIF;
	dmaXferStepAt = 6;
	hostTickSetStep(dmaXferStep);
	CHECK(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 50) == HAL_OK);
	CHECK((HAL_DMA_GetError(&h) & HAL_DMA_ERROR_FE) == HAL_DMA_ERROR_FE);
	CHECK((hostDmaFlags(st.c, st.s) & HOST_DMA_FEIF) == 0u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 characterization: a direct mode error flag during polling is recorded and cleared and the poll goes on") {
	// NEEDS REVIEW (proposed, lead decision): as for the FIFO error: the charter expects HAL_ERROR with the code DME;
	// the HAL records HAL_DMA_ERROR_DME, clears DMEIF and returns HAL_OK at the end of the transfer.
	const dmaXferStream st = dmaXferMake(1, 4);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	dmaXferStepC = st.c;
	dmaXferStepS = st.s;
	dmaXferRaiseAt = 2;
	dmaXferRaiseMask = HOST_DMA_DMEIF;
	dmaXferStepAt = 6;
	hostTickSetStep(dmaXferStep);
	CHECK(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 50) == HAL_OK);
	CHECK((HAL_DMA_GetError(&h) & HAL_DMA_ERROR_DME) == HAL_DMA_ERROR_DME);
	CHECK((hostDmaFlags(st.c, st.s) & HOST_DMA_DMEIF) == 0u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 GetState is READY after Init, BUSY after Start and RESET after DeInit") {
	// UM1725 21.2.4: GetState returns the DMA state.
	const dmaXferStream st = dmaXferMake(0, 1);
	DMA_HandleTypeDef h;
	make(h, st);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_BUSY);
	REQUIRE(HAL_DMA_Abort(&h) == HAL_OK);
	REQUIRE(HAL_DMA_DeInit(&h) == HAL_OK);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_RESET);
}

/* ---------------------------------------------------------------- DS8 */

TEST_CASE_FIXTURE(DmaXferFx, "DS8 PollForTransfer in circular mode returns HAL_ERROR with the error code NOT_SUPPORTED") {
	// UM1725 PollForTransfer note: it cannot be used in circular and double buffering mode.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st, DMA_PERIPH_TO_MEMORY, DMA_CIRCULAR);
			REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
			hostDmaComplete(st.c, st.s);
			CHECK(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 10) == HAL_ERROR);
			CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_NOT_SUPPORTED);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DS8 PollForTransfer in double buffer mode returns HAL_ERROR with the error code NOT_SUPPORTED") {
	const dmaXferStream st = dmaXferMake(1, 3);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 10) == HAL_OK);
	hostDmaComplete(st.c, st.s);
	CHECK(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 10) == HAL_ERROR);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_NOT_SUPPORTED);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS8 a rejected circular poll leaves the transfer flag set") {
	// The poll did nothing: TCIF stays (RM0090 10.4: only software clears a flag, here the HAL did not).
	const dmaXferStream st = dmaXferMake(0, 0);
	DMA_HandleTypeDef h;
	make(h, st, DMA_PERIPH_TO_MEMORY, DMA_CIRCULAR);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	hostDmaComplete(st.c, st.s);
	REQUIRE(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 10) == HAL_ERROR);
	CHECK((hostDmaFlags(st.c, st.s) & HOST_DMA_TCIF) == HOST_DMA_TCIF);
	CHECK(noProtectedWrites(st));
}

/* ---------------------------------------------------------------- DS9 */

TEST_CASE_FIXTURE(DmaXferFx, "DS9 characterization: PollForTransfer on a stream that is not busy returns HAL_ERROR") {
	// NEEDS REVIEW (proposed): UM1725 gives no status for polling a stream with no transfer. The HAL returns HAL_ERROR
	// at once (no waiting).
	const dmaXferStream st = dmaXferMake(0, 6);
	DMA_HandleTypeDef h;
	make(h, st);
	const uint32_t t0 = hostTickPeek();
	CHECK(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 10) == HAL_ERROR);
	CHECK(hostTickPeek() - t0 < 3u);
}

TEST_CASE_FIXTURE(DmaXferFx, "DS9 characterization: PollForTransfer on a stream that is not busy sets the error code NO_XFER") {
	// NEEDS REVIEW (proposed): UM1725 lists HAL_DMA_ERROR_NO_XFER only as "Abort requested with no Xfer ongoing"; the
	// HAL also uses it for a poll without a transfer.
	const dmaXferStream st = dmaXferMake(0, 6);
	DMA_HandleTypeDef h;
	make(h, st);
	HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 10);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_NO_XFER);
}

/* ---------------------------------------------------------------- DX1 */

TEST_CASE_FIXTURE(DmaXferFx, "DX1 MultiBufferStart peripheral to memory sets PAR, M0AR, M1AR, DBM and NDTR") {
	// RM0090 Table 46: DIR 00 source DMA_SxPAR, destination DMA_SxM0AR / DMA_SxM1AR.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st, DMA_PERIPH_TO_MEMORY);
			CHECK(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 200) == HAL_OK);
			CHECK(peek(&st.inst->PAR) == dmaXferPeriph);
			CHECK(peek(&st.inst->M0AR) == dmaXferMem);
			CHECK(peek(&st.inst->M1AR) == dmaXferMem2);
			CHECK((peek(&st.inst->NDTR) & 0xFFFFu) == 200u);
			CHECK((cr(st) & dmaXferDBM) == dmaXferDBM); // RM0090 10.5.5 DBM bit 18
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 MultiBufferStart memory to peripheral sets M0AR from the source and PAR from the destination") {
	// RM0090 Table 46: DIR 01 source DMA_SxM0AR / DMA_SxM1AR, destination DMA_SxPAR.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st, DMA_MEMORY_TO_PERIPH);
			CHECK(HAL_DMAEx_MultiBufferStart(&h, dmaXferMem, dmaXferPeriph, dmaXferMem2, 200) == HAL_OK);
			CHECK(peek(&st.inst->M0AR) == dmaXferMem);
			CHECK(peek(&st.inst->M1AR) == dmaXferMem2);
			CHECK(peek(&st.inst->PAR) == dmaXferPeriph);
			CHECK((cr(st) & dmaXferDBM) == dmaXferDBM);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 MultiBufferStart makes the transfer circular, enables the stream and sets BUSY") {
	// RM0090 10.3.9: with DBM = 1 circular mode is automatic. EN = 1, state BUSY (UM1725 GetState).
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st, DMA_PERIPH_TO_MEMORY, DMA_NORMAL);
			REQUIRE(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
			CHECK((cr(st) & dmaXferCIRC) == dmaXferCIRC);
			CHECK((cr(st) & dmaXferEN) == dmaXferEN);
			CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_BUSY);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 MultiBufferStart writes EN after NDTR, PAR, M0AR and M1AR") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			const size_t from = hostRegsLogCount();
			REQUIRE(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
			CHECK(enableWrittenLast(from, st));
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 MultiBufferStart enables none of the interrupts") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			callbacks(h);
			REQUIRE(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
			CHECK((cr(st) & dmaXferALLIE) == 0u);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 MultiBufferStart_IT sets addresses, DBM, NDTR, EN last and the interrupt enables") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st, DMA_MEMORY_TO_PERIPH);
			callbacks(h);
			const size_t from = hostRegsLogCount();
			REQUIRE(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferMem, dmaXferPeriph, dmaXferMem2, 96) == HAL_OK);
			CHECK(peek(&st.inst->M0AR) == dmaXferMem);
			CHECK(peek(&st.inst->M1AR) == dmaXferMem2);
			CHECK(peek(&st.inst->PAR) == dmaXferPeriph);
			CHECK((peek(&st.inst->NDTR) & 0xFFFFu) == 96u);
			CHECK((cr(st) & dmaXferDBM) == dmaXferDBM);
			CHECK((cr(st) & dmaXferEN) == dmaXferEN);
			CHECK((cr(st) & (dmaXferTCIE | dmaXferTEIE | dmaXferDMEIE)) == (dmaXferTCIE | dmaXferTEIE | dmaXferDMEIE));
			CHECK((cr(st) & dmaXferHTIE) == dmaXferHTIE); // half transfer callback is set
			CHECK(enableWrittenLast(from, st));
			CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_BUSY);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 MultiBufferStart_IT sets FEIE when direct mode is disabled") {
	const dmaXferStream st = dmaXferMake(0, 7);
	DMA_HandleTypeDef h;
	make(h, st, DMA_PERIPH_TO_MEMORY, DMA_NORMAL, true);
	callbacks(h);
	REQUIRE((fcr(st) & dmaXferDMDIS) == dmaXferDMDIS);
	REQUIRE(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
	CHECK((fcr(st) & dmaXferFEIE) == dmaXferFEIE); // RM0090 10.5.10, as Start_IT should (DS2)
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 characterization: MultiBufferStart_IT sets FEIE also in direct mode") {
	// NEEDS REVIEW (proposed): neither document says; the HAL sets FEIE unconditionally here (and never in Start_IT).
	const dmaXferStream st = dmaXferMake(1, 7);
	DMA_HandleTypeDef h;
	make(h, st, DMA_PERIPH_TO_MEMORY, DMA_NORMAL, false);
	callbacks(h);
	REQUIRE(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
	CHECK((fcr(st) & dmaXferFEIE) == dmaXferFEIE);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 characterization: MultiBufferStart_IT without the memory 1 complete callback returns HAL_ERROR with the error code PARAM") {
	// NEEDS REVIEW (proposed): UM1725 DMAEx does not say that the three callbacks XferCpltCallback, XferM1CpltCallback and
	// XferErrorCallback are mandatory. The HAL refuses to start without any of them: HAL_ERROR, error code PARAM (UM1725
	// "Parameter error"), nothing written, and this check comes before the state check.
	const dmaXferStream st = dmaXferMake(0, 3);
	DMA_HandleTypeDef h;
	make(h, st);
	callbacks(h);
	h.XferM1CpltCallback = nullptr;
	CHECK(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_ERROR);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_PARAM);
	CHECK((cr(st) & dmaXferEN) == 0u);
	CHECK(peek(&st.inst->M1AR) == 0u);
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 MultiBufferStart of a stream that is BUSY returns HAL_BUSY") {
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			callbacks(h);
			REQUIRE(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
			CHECK(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph + 4u, dmaXferMem2, dmaXferMem, 32) == HAL_BUSY);
			CHECK(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferPeriph + 4u, dmaXferMem2, dmaXferMem, 32) == HAL_BUSY);
			CHECK(peek(&st.inst->PAR) == dmaXferPeriph);
			CHECK(peek(&st.inst->M0AR) == dmaXferMem);
			CHECK(peek(&st.inst->M1AR) == dmaXferMem2);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 characterization: MultiBufferStart of a handle that was never initialised returns HAL_BUSY") {
	// NEEDS REVIEW (proposed): as DS3, UM1725 gives no status for a start before HAL_DMA_Init; the HAL returns HAL_BUSY.
	const dmaXferStream st = dmaXferMake(1, 1);
	DMA_HandleTypeDef h = {};
	h.Instance = st.inst;
	callbacks(h);
	CHECK(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_BUSY);
	CHECK(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_BUSY);
	CHECK(peek(&st.inst->M1AR) == 0u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 MultiBufferStart in memory to memory mode returns HAL_ERROR with the error code NOT_SUPPORTED") {
	// UM1725 DMAEx: "In Memory-to-Memory transfer mode, Multi (Double) Buffer mode is not allowed"; RM0090 Table 46.
	const dmaXferStream st = dmaXferMake(1, 0);
	DMA_HandleTypeDef h;
	make(h, st, DMA_MEMORY_TO_MEMORY, DMA_NORMAL, true);
	CHECK(HAL_DMAEx_MultiBufferStart(&h, dmaXferMem, dmaXferMem2, 0x20005000u, 32) == HAL_ERROR);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_NOT_SUPPORTED);
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 MultiBufferStart_IT in memory to memory mode returns HAL_ERROR with the error code NOT_SUPPORTED") {
	const dmaXferStream st = dmaXferMake(0, 4);
	DMA_HandleTypeDef h;
	make(h, st, DMA_MEMORY_TO_MEMORY, DMA_NORMAL, true);
	callbacks(h);
	CHECK(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferMem, dmaXferMem2, 0x20005000u, 32) == HAL_ERROR);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_NOT_SUPPORTED);
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 a rejected memory to memory MultiBufferStart leaves the stream disabled and unconfigured") {
	const dmaXferStream st = dmaXferMake(1, 0);
	DMA_HandleTypeDef h;
	make(h, st, DMA_MEMORY_TO_MEMORY, DMA_NORMAL, true);
	REQUIRE(HAL_DMAEx_MultiBufferStart(&h, dmaXferMem, dmaXferMem2, 0x20005000u, 32) == HAL_ERROR);
	CHECK((cr(st) & dmaXferEN) == 0u);
	CHECK((cr(st) & dmaXferDBM) == 0u);
	CHECK(peek(&st.inst->NDTR) == 0u);
	CHECK(peek(&st.inst->M1AR) == 0u);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(noProtectedWrites(st));
}

/* ---------------------------------------------------------------- DX2 */

TEST_CASE_FIXTURE(DmaXferFx, "DX2 ChangeMemory MEMORY1 while the transfer uses memory 0 writes M1AR") {
	// RM0090 10.3.9 note: with CT = 0 M1AR may be written.
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
			REQUIRE((cr(st) & dmaXferCT) == 0u);
			CHECK(HAL_DMAEx_ChangeMemory(&h, 0x20007000u, MEMORY1) == HAL_OK);
			CHECK(peek(&st.inst->M1AR) == 0x20007000u);
			CHECK(peek(&st.inst->M0AR) == dmaXferMem);
			CHECK((hostDmaFlags(st.c, st.s) & HOST_DMA_TEIF) == 0u);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DX2 ChangeMemory MEMORY0 while the transfer uses memory 1 writes M0AR") {
	// RM0090 10.3.9 note: with CT = 1 M0AR may be written. One end of transaction (TCIF) toggles CT (Table 46).
	for (const auto &st : dmaXferLayoutStreams()) {
		SUBCASE(st.name.c_str()) {
			DMA_HandleTypeDef h;
			make(h, st);
			REQUIRE(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
			hostDmaComplete(st.c, st.s);
			REQUIRE((cr(st) & dmaXferCT) == dmaXferCT);
			CHECK(HAL_DMAEx_ChangeMemory(&h, 0x20008000u, MEMORY0) == HAL_OK);
			CHECK(peek(&st.inst->M0AR) == 0x20008000u);
			CHECK(peek(&st.inst->M1AR) == dmaXferMem2);
			CHECK((hostDmaFlags(st.c, st.s) & HOST_DMA_TEIF) == 0u);
			CHECK(noProtectedWrites(st));
		}
	}
}

TEST_CASE_FIXTURE(DmaXferFx, "DX2 ChangeMemory keeps the stream enabled and BUSY") {
	const dmaXferStream st = dmaXferMake(0, 3);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
	REQUIRE(HAL_DMAEx_ChangeMemory(&h, 0x20007000u, MEMORY1) == HAL_OK);
	CHECK((cr(st) & dmaXferEN) == dmaXferEN);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_BUSY);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DX2 characterization: ChangeMemory of the memory in use is passed to the hardware") {
	// NEEDS REVIEW (proposed): UM1725 says MEMORY0 "can be changed only when the current transfer uses MEMORY1", but
	// does not say whether the HAL checks it. The HAL does not: it returns HAL_OK and writes M0AR while CT = 0; the
	// register model (RM0090 10.3.9: a write to the memory in use sets TEIF) then raises TEIF and keeps the old address. Open: should the HAL refuse the call (return HAL_ERROR, write nothing)?
	const dmaXferStream st = dmaXferMake(1, 4);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
	REQUIRE((cr(st) & dmaXferCT) == 0u);
	CHECK(HAL_DMAEx_ChangeMemory(&h, 0x20007000u, MEMORY0) == HAL_OK);
	CHECK((hostDmaFlags(st.c, st.s) & HOST_DMA_TEIF) == HOST_DMA_TEIF); // RM0090 10.3.9 note
	CHECK(peek(&st.inst->M0AR) == dmaXferMem);
	CHECK(noProtectedWrites(st));
}

/* ---------------------------------------------------------------- added coverage */

TEST_CASE_FIXTURE(DmaXferFx, "DX1 Start after MultiBufferStart and Abort runs in single buffer mode (DBM = 0)") {
	// RM0090 10.5.5: DBM selects double buffer mode; a plain Start must not inherit it from an earlier multi buffer transfer.
	const dmaXferStream st = dmaXferMake(0, 3);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
	REQUIRE(HAL_DMA_Abort(&h) == HAL_OK);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	CHECK((cr(st) & dmaXferDBM) == 0u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 MultiBufferStart_IT clears the FE, DME and TC flags of its stream and not of the neighbours") {
	// RM0090 10.3.? stream configuration: the stream's status flags are cleared before it is enabled.
	const dmaXferStream st = dmaXferMake(1, 3);
	const uint32_t f = HOST_DMA_FEIF | HOST_DMA_DMEIF | HOST_DMA_TCIF;
	DMA_HandleTypeDef h;
	make(h, st);
	callbacks(h);
	hostDmaRaise(st.c, st.s, f);
	hostDmaRaise(st.c, 2, f);
	hostDmaRaise(st.c, 4, f);
	hostDmaRaise(0, st.s, f);
	REQUIRE(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
	CHECK(hostDmaFlags(st.c, st.s) == 0u);
	CHECK(hostDmaFlags(st.c, 2) == f);
	CHECK(hostDmaFlags(st.c, 4) == f);
	CHECK(hostDmaFlags(0, st.s) == f);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 characterization: MultiBufferStart_IT without XferCpltCallback returns HAL_ERROR with the error code PARAM") {
	// NEEDS REVIEW (proposed): see the M1 callback case; UM1725 does not list the mandatory callbacks.
	const dmaXferStream st = dmaXferMake(1, 0);
	DMA_HandleTypeDef h;
	make(h, st);
	callbacks(h);
	h.XferCpltCallback = nullptr;
	CHECK(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_ERROR);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_PARAM);
	CHECK((cr(st) & dmaXferEN) == 0u);
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 characterization: MultiBufferStart_IT without XferErrorCallback returns HAL_ERROR with the error code PARAM") {
	// NEEDS REVIEW (proposed): as above.
	const dmaXferStream st = dmaXferMake(0, 0);
	DMA_HandleTypeDef h;
	make(h, st);
	callbacks(h);
	h.XferErrorCallback = nullptr;
	CHECK(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_ERROR);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_PARAM);
	CHECK((cr(st) & dmaXferEN) == 0u);
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 MultiBufferStart_IT sets HTIE when only the memory 1 half callback is set") {
	const dmaXferStream st = dmaXferMake(1, 7);
	DMA_HandleTypeDef h;
	make(h, st);
	callbacks(h, false);
	h.XferM1HalfCpltCallback = dmaXferOnHalf;
	REQUIRE(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
	CHECK((cr(st) & dmaXferHTIE) == dmaXferHTIE); // RM0090 10.5.5 HTIE bit 3
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 characterization: Abort of a MultiBufferStart_IT transfer clears every interrupt enable including FEIE") {
	// NEEDS REVIEW (proposed): UM1725 does not say which enables Abort clears. The HAL clears TCIE, TEIE, DMEIE, HTIE and
	// FEIE.
	const dmaXferStream st = dmaXferMake(0, 4);
	DMA_HandleTypeDef h;
	make(h, st, DMA_PERIPH_TO_MEMORY, DMA_NORMAL, true);
	callbacks(h);
	REQUIRE(HAL_DMAEx_MultiBufferStart_IT(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
	REQUIRE((cr(st) & dmaXferALLIE) == dmaXferALLIE);
	REQUIRE((fcr(st) & dmaXferFEIE) == dmaXferFEIE);
	REQUIRE(HAL_DMA_Abort(&h) == HAL_OK);
	CHECK((cr(st) & dmaXferALLIE) == 0u);
	CHECK((fcr(st) & dmaXferFEIE) == 0u);
	CHECK(noProtectedWrites(st));
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 characterization: a full transfer poll also clears HTIF") {
	// NEEDS REVIEW (proposed): UM1725 does not list which flags a poll clears. The full level poll clears HTIF and TCIF.
	const dmaXferStream st = dmaXferMake(0, 3);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	hostDmaRaise(st.c, st.s, HOST_DMA_HTIF);
	hostDmaComplete(st.c, st.s);
	REQUIRE(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 10) == HAL_OK);
	CHECK(hostDmaFlags(st.c, st.s) == 0u);
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 characterization: a half transfer poll leaves TCIF set") {
	// NEEDS REVIEW (proposed): the half level poll clears HTIF only; a TCIF that is already set stays.
	const dmaXferStream st = dmaXferMake(1, 4);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	hostDmaRaise(st.c, st.s, HOST_DMA_HTIF | HOST_DMA_TCIF);
	REQUIRE(HAL_DMA_PollForTransfer(&h, HAL_DMA_HALF_TRANSFER, 10) == HAL_OK);
	CHECK((hostDmaFlags(st.c, st.s) & HOST_DMA_HTIF) == 0u);
	CHECK((hostDmaFlags(st.c, st.s) & HOST_DMA_TCIF) == HOST_DMA_TCIF);
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 characterization: a transfer error poll clears HTIF and TCIF too") {
	// NEEDS REVIEW (proposed): after a transfer error the HAL clears all flags of the stream.
	const dmaXferStream st = dmaXferMake(0, 7);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	// the flags arrive while the HAL polls (an already set TCIF would end the poll before it looks at TEIF)
	dmaXferStepC = st.c;
	dmaXferStepS = st.s;
	dmaXferRaiseAt = 2;
	dmaXferRaiseMask = HOST_DMA_HTIF | HOST_DMA_TCIF | HOST_DMA_TEIF;
	hostTickSetStep(dmaXferStep);
	REQUIRE(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 20) == HAL_ERROR);
	CHECK(hostDmaFlags(st.c, st.s) == 0u);
}

TEST_CASE_FIXTURE(DmaXferFx, "DS7 characterization: a transfer error poll switches the interrupt enables off") {
	// NEEDS REVIEW (proposed): the HAL aborts the stream after a transfer error, which clears TCIE, TEIE, DMEIE and HTIE.
	const dmaXferStream st = dmaXferMake(1, 2);
	DMA_HandleTypeDef h;
	make(h, st);
	callbacks(h);
	REQUIRE(HAL_DMA_Start_IT(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	hostDmaRaise(st.c, st.s, HOST_DMA_TEIF);
	REQUIRE(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 20) == HAL_ERROR);
	CHECK((cr(st) & dmaXferALLIE) == 0u);
}

TEST_CASE_FIXTURE(DmaXferFx, "DS3 characterization: Start refused because the state is not READY leaves the handle unlocked") {
	// NEEDS REVIEW (proposed): see the Start_IT case; UM1725 does not describe the lock.
	const dmaXferStream st = dmaXferMake(0, 5);
	DMA_HandleTypeDef h;
	make(h, st);
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	h.Lock = HAL_UNLOCKED; // a started transfer keeps the handle locked; this reaches the state check
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem2, 20) == HAL_BUSY);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(DmaXferFx, "DS9 characterization: a refused poll of a stream that is not busy leaves the handle unlocked") {
	// NEEDS REVIEW (proposed): the HAL releases the lock before returning HAL_ERROR.
	const dmaXferStream st = dmaXferMake(1, 6);
	DMA_HandleTypeDef h;
	make(h, st);
	h.Lock = HAL_LOCKED;
	REQUIRE(HAL_DMA_PollForTransfer(&h, HAL_DMA_FULL_TRANSFER, 10) == HAL_ERROR);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(DmaXferFx, "DS1 characterization: Start clears a stale error code") {
	// NEEDS REVIEW (proposed): UM1725 does not say; the HAL resets the error code to NONE when a transfer starts.
	const dmaXferStream st = dmaXferMake(0, 0);
	DMA_HandleTypeDef h;
	make(h, st);
	h.ErrorCode = HAL_DMA_ERROR_TE;
	REQUIRE(HAL_DMA_Start(&h, dmaXferPeriph, dmaXferMem, 10) == HAL_OK);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_NONE);
}

TEST_CASE_FIXTURE(DmaXferFx, "DX1 MultiBufferStart refused because the state is not READY releases the lock" *
                                 KNOWN_ISSUE("candidate vendor bug V96: HAL_DMAEx_MultiBufferStart returns HAL_BUSY without releasing the lock, the handle stays locked (UM1725: the process is unlocked when the call returns; the sibling functions unlock)")) {
	const dmaXferStream st = dmaXferMake(1, 2);
	DMA_HandleTypeDef h;
	make(h, st);
	h.State = HAL_DMA_STATE_TIMEOUT; // any state but READY
	REQUIRE(h.Lock == HAL_UNLOCKED);
	REQUIRE(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_BUSY);
	CHECK(h.Lock == HAL_UNLOCKED);
	h.State = HAL_DMA_STATE_READY;
	CHECK(HAL_DMAEx_MultiBufferStart(&h, dmaXferPeriph, dmaXferMem, dmaXferMem2, 64) == HAL_OK);
}

} // TEST_SUITE
