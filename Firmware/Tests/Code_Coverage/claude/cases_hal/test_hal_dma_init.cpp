/*
 * P12 T3 group DI: HAL DMA init, deinit, state (stm32f4xx_hal_dma.c) against the DMA register model (HalDma.h).
 * Requirements DI1-DI9: Tests/Code_Coverage/claude/charters/hal/P12_T3_REQUIREMENTS.md.
 *
 * Oracles: RM0090 Rev 22 chapter 10 (10.3.x functional, 10.4 interrupts, 10.5.1-10.5.10 registers, Table 49 FIFO threshold
 * configurations, Table 1 addresses) and UM1725 Rev 8 (DMA chapter). Expected values are written out here, never read from
 * the HAL source.
 *
 * Encodings used in the derivations (RM0090 10.5.5 SxCR, 10.5.10 SxFCR):
 *  - SxCR: EN bit 0, CIRC bit 8, PINC bit 9, MINC bit 10, PSIZE bits 12:11, MSIZE bits 14:13, PL bits 17:16, PBURST bits 22:21,
 *    MBURST bits 24:23, CHSEL bits 27:25, DIR bits 7:6 (00 peripheral to memory, 01 memory to peripheral, 10 memory to memory).
 *  - SxFCR: FTH bits 1:0 (00 1/4, 01 1/2, 10 3/4, 11 full), DMDIS bit 2 (1 = direct mode disabled = FIFO in use), FS bits 5:3,
 *    FEIE bit 7; reset value 0x21.
 *  - LISR / HISR flag group of a stream: offsets 0, 6, 16, 22 for streams 0 / 4, 1 / 5, 2 / 6, 3 / 7 (RM0090 10.5.1-10.5.4);
 *    streams 0-3 are in LISR (base + 0), streams 4-7 in HISR (base + 4).
 *  - DMA1 0x40026000, DMA2 0x40026400 (RM0090 Table 1).
 * HAL status of failing calls, error codes and the order of writes are not given by UM1725 / RM0090: such cases are named
 * "characterization:" and have their proposed NEEDS REVIEW text in a comment.
 *
 * File-local symbols are prefixed dmaInit (the three DMA writers' files are linked into one executable).
 */
#include "HalDma.h"
#include "HalFixture.h"
#include "TestHelpers.h"

#include <cstdint>
#include <string>

namespace {

const unsigned dmaInitControllers = 2;
const unsigned dmaInitStreams = 8;

DMA_Stream_TypeDef *dmaInitStream(unsigned c, unsigned s) {
	DMA_Stream_TypeDef *const t[2][8] = {
		{DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7},
		{DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7}};
	return t[c][s];
}

std::string dmaInitName(unsigned c, unsigned s) {
	return std::string("DMA") + static_cast<char>('1' + c) + " stream " + static_cast<char>('0' + s);
}

const uint32_t dmaInitAllFlags = HOST_DMA_FEIF | HOST_DMA_DMEIF | HOST_DMA_TEIF | HOST_DMA_HTIF | HOST_DMA_TCIF; // 0x3D

const uint32_t dmaInitChannels[8] = {DMA_CHANNEL_0, DMA_CHANNEL_1, DMA_CHANNEL_2, DMA_CHANNEL_3,
									 DMA_CHANNEL_4, DMA_CHANNEL_5, DMA_CHANNEL_6, DMA_CHANNEL_7};
const uint32_t dmaInitSizes[3] = {DMA_MDATAALIGN_BYTE, DMA_MDATAALIGN_HALFWORD, DMA_MDATAALIGN_WORD};
const uint32_t dmaInitPSizes[3] = {DMA_PDATAALIGN_BYTE, DMA_PDATAALIGN_HALFWORD, DMA_PDATAALIGN_WORD};
const uint32_t dmaInitThresholds[4] = {DMA_FIFO_THRESHOLD_1QUARTERFULL, DMA_FIFO_THRESHOLD_HALFFULL,
									   DMA_FIFO_THRESHOLD_3QUARTERSFULL, DMA_FIFO_THRESHOLD_FULL};
const uint32_t dmaInitBursts[4] = {DMA_MBURST_SINGLE, DMA_MBURST_INC4, DMA_MBURST_INC8, DMA_MBURST_INC16};
const uint32_t dmaInitPBursts[4] = {DMA_PBURST_SINGLE, DMA_PBURST_INC4, DMA_PBURST_INC8, DMA_PBURST_INC16};

/* Direct mode, peripheral to memory, byte sizes, no increment, normal mode, low priority, channel 0. */
DMA_InitTypeDef dmaInitBase() {
	DMA_InitTypeDef i = {};
	i.Channel = DMA_CHANNEL_0;
	i.Direction = DMA_PERIPH_TO_MEMORY;
	i.PeriphInc = DMA_PINC_DISABLE;
	i.MemInc = DMA_MINC_DISABLE;
	i.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	i.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	i.Mode = DMA_NORMAL;
	i.Priority = DMA_PRIORITY_LOW;
	i.FIFOMode = DMA_FIFOMODE_DISABLE;
	i.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	i.MemBurst = DMA_MBURST_SINGLE;
	i.PeriphBurst = DMA_PBURST_SINGLE;
	return i;
}

DMA_HandleTypeDef dmaInitHandle(unsigned c, unsigned s, const DMA_InitTypeDef &init) {
	DMA_HandleTypeDef h = {};
	h.Instance = dmaInitStream(c, s);
	h.Init = init;
	return h;
}

/* A stream the CPU enables: NDT 4 so that it really runs (RM0090 10.5.6: NDT 0 keeps the stream idle). */
void dmaInitEnable(DMA_Stream_TypeDef *st) {
	st->NDTR = 4;
	st->CR |= DMA_SxCR_EN;
}

void dmaInitRaiseAll(unsigned c) {
	for (unsigned s = 0; s < dmaInitStreams; ++s) hostDmaRaise(c, s, dmaInitAllFlags);
}

void dmaInitNoCallback(DMA_HandleTypeDef *) {}

// Table 49 of RM0090 (FIFO threshold configurations): [MSIZE byte, half-word, word][FTH 1/4, 1/2, 3/4, full]
// [MBURST single, INCR4, INCR8, INCR16]. Single is not listed in Table 49 (one beat always fits). Copied from the table:
//   Byte:      1/4: INCR4 ok, INCR8 forbidden, INCR16 forbidden    1/2: INCR4, INCR8 ok, INCR16 forbidden
//              3/4: INCR4 ok, INCR8, INCR16 forbidden              full: INCR4, INCR8, INCR16 ok
//   Half-word: 1/4: all forbidden                                  1/2: INCR4 ok, INCR8, INCR16 forbidden
//              3/4: all forbidden                                  full: INCR4, INCR8 ok, INCR16 forbidden
//   Word:      1/4, 1/2, 3/4: all forbidden                        full: INCR4 ok, INCR8, INCR16 forbidden
const bool dmaInitAllowed[3][4][4] = {
	{{true, true, false, false}, {true, true, true, false}, {true, true, false, false}, {true, true, true, true}},
	{{true, false, false, false}, {true, true, false, false}, {true, false, false, false}, {true, true, true, false}},
	{{true, false, false, false}, {true, false, false, false}, {true, false, false, false}, {true, true, false, false}}};

const char *const dmaInitSizeName[3] = {"byte", "half-word", "word"};
const char *const dmaInitThresholdName[4] = {"1/4", "1/2", "3/4", "full"};
const char *const dmaInitBurstName[4] = {"single", "INCR4", "INCR8", "INCR16"};

std::string dmaInitComboName(int m, int f, int b) {
	return std::string("MSIZE ") + dmaInitSizeName[m] + " FTH " + dmaInitThresholdName[f] + " MBURST " + dmaInitBurstName[b];
}

} // namespace

// Subcase loop over the 16 streams (c, s are the controller and stream index).
#define DMA_INIT_EACH_STREAM_BEGIN                                                                                   \
	for (unsigned c = 0; c < dmaInitControllers; ++c)                                                                \
		for (unsigned s = 0; s < dmaInitStreams; ++s) {                                                              \
			const std::string dmaInitSubName = dmaInitName(c, s);                                                    \
			SUBCASE(dmaInitSubName.c_str()) {
#define DMA_INIT_EACH_STREAM_END                                                                                     \
	}                                                                                                                \
	}

TEST_SUITE("P12 T3 DMA init") {

// ---------------------------------------------------------------- DI1 init writes the fields

TEST_CASE_FIXTURE(HalFixture, "DI1 init writes SxCR exactly on each of the 16 streams") {
	DMA_INIT_EACH_STREAM_BEGIN
	DMA_InitTypeDef init = dmaInitBase();
	init.Channel = dmaInitChannels[s];                // CHSEL = s
	init.Direction = DMA_MEMORY_TO_PERIPH;            // DIR = 01
	init.PeriphInc = DMA_PINC_ENABLE;
	init.MemInc = DMA_MINC_ENABLE;
	init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // PSIZE = 01
	init.MemDataAlignment = DMA_MDATAALIGN_WORD;      // MSIZE = 10
	init.Mode = DMA_CIRCULAR;
	init.Priority = (s & 3u) << 16;                   // PL = s & 3
	init.FIFOMode = DMA_FIFOMODE_ENABLE;
	init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;     // word, full, INCR4: allowed by Table 49
	init.MemBurst = DMA_MBURST_INC4;                  // MBURST = 01
	init.PeriphBurst = DMA_PBURST_INC4;               // PBURST = 01
	DMA_HandleTypeDef h = dmaInitHandle(c, s, init);
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	// RM0090 10.5.5: CHSEL (27:25) = s, MBURST (24:23) = 01, PBURST (22:21) = 01, PL (17:16) = s & 3, MSIZE (14:13) = 10,
	// PSIZE (12:11) = 01, MINC (10), PINC (9), CIRC (8), DIR (7:6) = 01; EN (0) and the interrupt enables stay 0.
	const uint32_t expected = (s << 25) | (1u << 23) | (1u << 21) | ((s & 3u) << 16) | (2u << 13) | (1u << 11) | (1u << 10) |
							  (1u << 9) | (1u << 8) | (1u << 6);
	CHECK(peek(&dmaInitStream(c, s)->CR) == expected);
	// The other streams are untouched.
	for (unsigned c2 = 0; c2 < dmaInitControllers; ++c2)
		for (unsigned s2 = 0; s2 < dmaInitStreams; ++s2)
			if (c2 != c || s2 != s) CHECK(peek(&dmaInitStream(c2, s2)->CR) == 0u);
	DMA_INIT_EACH_STREAM_END
}

TEST_CASE_FIXTURE(HalFixture, "DI1 init writes DMDIS and FTH of SxFCR exactly on each of the 16 streams") {
	DMA_INIT_EACH_STREAM_BEGIN
	DMA_InitTypeDef init = dmaInitBase();
	init.FIFOMode = DMA_FIFOMODE_ENABLE;
	init.FIFOThreshold = dmaInitThresholds[s & 3u]; // FTH = s & 3
	DMA_HandleTypeDef h = dmaInitHandle(c, s, init);
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	// RM0090 10.5.10: DMDIS (bit 2) = 1, FTH (1:0) = s & 3, FEIE (bit 7) = 0; FS (5:3) is read only and left out of the mask.
	CHECK((peek(&dmaInitStream(c, s)->FCR) & 0x87u) == (0x4u | (s & 3u)));
	DMA_INIT_EACH_STREAM_END
}

TEST_CASE_FIXTURE(HalFixture, "DI1 init of a direct mode stream leaves DMDIS at 0") {
	DMA_InitTypeDef init = dmaInitBase();
	DMA_HandleTypeDef h = dmaInitHandle(1, 3, init);
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	CHECK((peek(&DMA2_Stream3->FCR) & 0x4u) == 0u); // RM0090 10.5.10: DMDIS = 0 is direct mode
}

TEST_CASE_FIXTURE(HalFixture, "DI1 init sets the state to READY and the error code to NONE on each of the 16 streams") {
	DMA_INIT_EACH_STREAM_BEGIN
	DMA_HandleTypeDef h = dmaInitHandle(c, s, dmaInitBase());
	h.State = HAL_DMA_STATE_ERROR;
	h.ErrorCode = HAL_DMA_ERROR_TE | HAL_DMA_ERROR_TIMEOUT;
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	CHECK(h.State == HAL_DMA_STATE_READY);
	CHECK(h.ErrorCode == HAL_DMA_ERROR_NONE);
	DMA_INIT_EACH_STREAM_END
}

TEST_CASE_FIXTURE(HalFixture, "DI1 each of the 8 channel values is written to CHSEL") {
	for (unsigned ch = 0; ch < 8; ++ch) {
		DMA_InitTypeDef init = dmaInitBase();
		init.Channel = dmaInitChannels[ch];
		DMA_HandleTypeDef h = dmaInitHandle(0, 4, init);
		REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
		CHECK(peek(&DMA1_Stream4->CR) == (ch << 25)); // CHSEL bits 27:25 (RM0090 10.5.5)
	}
}

TEST_CASE_FIXTURE(HalFixture, "DI1 each of the 3 directions is written to DIR") {
	const uint32_t dirs[3] = {DMA_PERIPH_TO_MEMORY, DMA_MEMORY_TO_PERIPH, DMA_MEMORY_TO_MEMORY};
	for (unsigned d = 0; d < 3; ++d) {
		DMA_InitTypeDef init = dmaInitBase();
		init.Direction = dirs[d];
		DMA_HandleTypeDef h = dmaInitHandle(1, 6, init);
		REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
		CHECK(peek(&DMA2_Stream6->CR) == (d << 6)); // DIR bits 7:6: 00, 01, 10 (RM0090 10.5.5)
	}
}

TEST_CASE_FIXTURE(HalFixture, "DI1 each of the 4 priorities is written to PL") {
	const uint32_t pls[4] = {DMA_PRIORITY_LOW, DMA_PRIORITY_MEDIUM, DMA_PRIORITY_HIGH, DMA_PRIORITY_VERY_HIGH};
	for (unsigned p = 0; p < 4; ++p) {
		DMA_InitTypeDef init = dmaInitBase();
		init.Priority = pls[p];
		DMA_HandleTypeDef h = dmaInitHandle(0, 2, init);
		REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
		CHECK(peek(&DMA1_Stream2->CR) == (p << 16)); // PL bits 17:16 (RM0090 10.5.5)
	}
}

TEST_CASE_FIXTURE(HalFixture, "DI1 each memory and peripheral data size is written to MSIZE and PSIZE") {
	for (unsigned m = 0; m < 3; ++m) {
		for (unsigned p = 0; p < 3; ++p) {
			DMA_InitTypeDef init = dmaInitBase();
			init.MemDataAlignment = dmaInitSizes[m];
			init.PeriphDataAlignment = dmaInitPSizes[p];
			DMA_HandleTypeDef h = dmaInitHandle(1, 1, init);
			REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
			// MSIZE bits 14:13, PSIZE bits 12:11: 00 byte, 01 half-word, 10 word (RM0090 10.5.5)
			CHECK(peek(&DMA2_Stream1->CR) == ((m << 13) | (p << 11)));
		}
	}
}

TEST_CASE_FIXTURE(HalFixture, "DI1 the increment and circular settings are written to MINC, PINC and CIRC one by one") {
	{
		DMA_InitTypeDef init = dmaInitBase();
		init.MemInc = DMA_MINC_ENABLE;
		DMA_HandleTypeDef h = dmaInitHandle(0, 0, init);
		REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
		CHECK(peek(&DMA1_Stream0->CR) == (1u << 10)); // MINC bit 10
	}
	{
		DMA_InitTypeDef init = dmaInitBase();
		init.PeriphInc = DMA_PINC_ENABLE;
		DMA_HandleTypeDef h = dmaInitHandle(0, 1, init);
		REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
		CHECK(peek(&DMA1_Stream1->CR) == (1u << 9)); // PINC bit 9
	}
	{
		DMA_InitTypeDef init = dmaInitBase();
		init.Mode = DMA_CIRCULAR;
		DMA_HandleTypeDef h = dmaInitHandle(0, 2, init);
		REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
		CHECK(peek(&DMA1_Stream2->CR) == (1u << 8)); // CIRC bit 8
	}
}

TEST_CASE_FIXTURE(HalFixture, "DI1 init of a default structure leaves SxCR at 0") {
	DMA_HandleTypeDef h = dmaInitHandle(0, 5, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	CHECK(peek(&DMA1_Stream5->CR) == 0u); // no bit of the structure set: every other SxCR bit is 0
	CHECK(hostDmaProtectedWrites(0, 5) == 0u);
}

// ---------------------------------------------------------------- DI2 init clears the flags of its stream only

TEST_CASE_FIXTURE(HalFixture, "DI2 init clears all flags of its stream and keeps those of the other streams") {
	DMA_INIT_EACH_STREAM_BEGIN
	dmaInitRaiseAll(c);
	dmaInitRaiseAll(1 - c);
	REQUIRE(hostDmaFlags(c, s) == dmaInitAllFlags);
	DMA_HandleTypeDef h = dmaInitHandle(c, s, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	CHECK(hostDmaFlags(c, s) == 0u);
	for (unsigned s2 = 0; s2 < dmaInitStreams; ++s2) {
		if (s2 != s) CHECK(hostDmaFlags(c, s2) == dmaInitAllFlags);
		CHECK(hostDmaFlags(1 - c, s2) == dmaInitAllFlags);
	}
	DMA_INIT_EACH_STREAM_END
}

// ---------------------------------------------------------------- DI3 init of an enabled stream

TEST_CASE_FIXTURE(HalFixture, "DI3 init of an enabled stream disables it, waits for EN = 0 and then configures it") {
	hostDmaSetDisableDelay(3);
	DMA_Stream_TypeDef *st = DMA2_Stream2;
	dmaInitEnable(st);
	REQUIRE((peek(&st->CR) & DMA_SxCR_EN) != 0u);
	DMA_InitTypeDef init = dmaInitBase();
	init.MemInc = DMA_MINC_ENABLE;
	DMA_HandleTypeDef h = dmaInitHandle(1, 2, init);
	CHECK(HAL_DMA_Init(&h) == HAL_OK);
	CHECK((peek(&st->CR) & DMA_SxCR_EN) == 0u);
	CHECK(peek(&st->CR) == (1u << 10)); // the configuration landed (MINC), no write was ignored because EN was still 1
	CHECK(hostDmaProtectedWrites(1, 2) == 0u);
	CHECK(h.State == HAL_DMA_STATE_READY);
	// The disable takes 3 HAL ticks: at least 3 ms passed, and not more than the HAL's own 5 ms timeout constant
	// (HAL_TIMEOUT_DMA_ABORT, not a UM1725 value) plus the polling granularity.
	CHECK(hostTickPeek() >= 3u);
	CHECK(hostTickPeek() <= 8u);
}

TEST_CASE_FIXTURE(HalFixture, "DI3 init of an enabled stream that stops at once returns HAL_OK with EN = 0") {
	DMA_Stream_TypeDef *st = DMA1_Stream7;
	dmaInitEnable(st);
	REQUIRE((peek(&st->CR) & DMA_SxCR_EN) != 0u);
	DMA_HandleTypeDef h = dmaInitHandle(0, 7, dmaInitBase());
	CHECK(HAL_DMA_Init(&h) == HAL_OK);
	CHECK((peek(&st->CR) & DMA_SxCR_EN) == 0u);
	CHECK(h.State == HAL_DMA_STATE_READY);
}

// characterization (proposed NEEDS REVIEW 301): UM1725 does not say what HAL_DMA_Init returns when the stream does not stop;
// the HAL returns HAL_TIMEOUT after about 5 ms. Review: is HAL_TIMEOUT (and not HAL_ERROR) the wanted status?
TEST_CASE_FIXTURE(HalFixture, "DI3 characterization: init of a stream whose EN never clears returns HAL_TIMEOUT after about 5 ms") {
	DMA_Stream_TypeDef *st = DMA1_Stream3;
	dmaInitEnable(st);
	hostDmaStickEnabled(0, 3, 1);
	DMA_HandleTypeDef h = dmaInitHandle(0, 3, dmaInitBase());
	CHECK(HAL_DMA_Init(&h) == HAL_TIMEOUT);
	CHECK(hostTickPeek() >= 5u);
	CHECK(hostTickPeek() <= 10u);
}

// characterization (proposed NEEDS REVIEW 302): after the timeout the HAL sets the error code HAL_DMA_ERROR_TIMEOUT (UM1725 lists
// the error code names only; the values come from the HAL header, and no link to Init is given). Review: error code TIMEOUT wanted?
TEST_CASE_FIXTURE(HalFixture, "DI3 characterization: init timeout sets the error code TIMEOUT") {
	dmaInitEnable(DMA1_Stream3);
	hostDmaStickEnabled(0, 3, 1);
	DMA_HandleTypeDef h = dmaInitHandle(0, 3, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_TIMEOUT);
	CHECK(h.ErrorCode == HAL_DMA_ERROR_TIMEOUT);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_TIMEOUT);
}

// characterization (proposed NEEDS REVIEW 303): the state after an init timeout is HAL_DMA_STATE_TIMEOUT (the state
// names come from the HAL header; UM1725 gives no state definitions and none for Init). Review: TIMEOUT (not READY or ERROR) wanted?
TEST_CASE_FIXTURE(HalFixture, "DI3 characterization: init timeout sets the state TIMEOUT") {
	dmaInitEnable(DMA1_Stream3);
	hostDmaStickEnabled(0, 3, 1);
	DMA_HandleTypeDef h = dmaInitHandle(0, 3, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_TIMEOUT);
	CHECK(h.State == HAL_DMA_STATE_TIMEOUT);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_TIMEOUT);
}

TEST_CASE_FIXTURE(HalFixture, "DI3 init succeeds again once the stream stops after an earlier timeout") {
	DMA_Stream_TypeDef *st = DMA1_Stream3;
	dmaInitEnable(st);
	hostDmaStickEnabled(0, 3, 1);
	DMA_HandleTypeDef h = dmaInitHandle(0, 3, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_TIMEOUT);
	hostDmaStickEnabled(0, 3, 0);
	st->CR = 0; // the stream stops now
	CHECK(HAL_DMA_Init(&h) == HAL_OK);
	CHECK(h.State == HAL_DMA_STATE_READY);
	CHECK(h.ErrorCode == HAL_DMA_ERROR_NONE);
}

// ---------------------------------------------------------------- DI4 FIFO / burst compatibility (RM0090 Table 49)

TEST_CASE_FIXTURE(HalFixture, "DI4 init of every combination Table 49 allows returns HAL_OK and the stream then enables without a FIFO error") {
	// The 48 combinations MSIZE x FTH x MBURST are the loops; those the table allows run here, the others in the next case.
	for (int m = 0; m < 3; ++m)
		for (int f = 0; f < 4; ++f)
			for (int b = 0; b < 4; ++b) {
				if (!dmaInitAllowed[m][f][b]) continue;
				const std::string name = dmaInitComboName(m, f, b);
				SUBCASE(name.c_str()) {
					DMA_InitTypeDef init = dmaInitBase();
					init.MemInc = DMA_MINC_ENABLE; // bursts need address increment (hal_dma.h note)
					init.MemDataAlignment = dmaInitSizes[m];
					init.PeriphDataAlignment = dmaInitPSizes[m];
					init.FIFOMode = DMA_FIFOMODE_ENABLE;
					init.FIFOThreshold = dmaInitThresholds[f];
					init.MemBurst = dmaInitBursts[b];
					DMA_HandleTypeDef h = dmaInitHandle(1, 0, init);
					CHECK(HAL_DMA_Init(&h) == HAL_OK);
					CHECK(h.ErrorCode == HAL_DMA_ERROR_NONE);
					CHECK(h.State == HAL_DMA_STATE_READY);
					// RM0090 10.3.18 / Table 49: an allowed configuration enables without FEIF and EN stays set.
					dmaInitEnable(DMA2_Stream0);
					CHECK((peek(&DMA2_Stream0->CR) & DMA_SxCR_EN) != 0u);
					CHECK((hostDmaFlags(1, 0) & HOST_DMA_FEIF) == 0u);
				}
			}
}

// characterization (proposed NEEDS REVIEW 304): RM0090 Table 49 calls these configurations forbidden (the hardware sets FEIF
// when the stream is enabled); neither RM0090 nor UM1725 says the HAL must reject them at init. The HAL returns HAL_ERROR with
// the error code HAL_DMA_ERROR_PARAM and the state READY. Review: reject at init (HAL_ERROR / PARAM / READY) wanted?
TEST_CASE_FIXTURE(HalFixture, "DI4 characterization: init of every combination Table 49 forbids returns HAL_ERROR with the error code PARAM and the state READY") {
	for (int m = 0; m < 3; ++m)
		for (int f = 0; f < 4; ++f)
			for (int b = 0; b < 4; ++b) {
				if (dmaInitAllowed[m][f][b]) continue;
				const std::string name = dmaInitComboName(m, f, b);
				SUBCASE(name.c_str()) {
					DMA_InitTypeDef init = dmaInitBase();
					init.MemInc = DMA_MINC_ENABLE;
					init.MemDataAlignment = dmaInitSizes[m];
					init.PeriphDataAlignment = dmaInitPSizes[m];
					init.FIFOMode = DMA_FIFOMODE_ENABLE;
					init.FIFOThreshold = dmaInitThresholds[f];
					init.MemBurst = dmaInitBursts[b];
					DMA_HandleTypeDef h = dmaInitHandle(1, 0, init);
					CHECK(HAL_DMA_Init(&h) == HAL_ERROR);
					CHECK(h.ErrorCode == HAL_DMA_ERROR_PARAM);
					CHECK(h.State == HAL_DMA_STATE_READY);
				}
			}
}

TEST_CASE_FIXTURE(HalFixture, "DI4 the table copied from RM0090 has 23 allowed and 25 forbidden combinations of the 48") {
	int allowed = 0, forbidden = 0;
	for (int m = 0; m < 3; ++m)
		for (int f = 0; f < 4; ++f)
			for (int b = 0; b < 4; ++b) (dmaInitAllowed[m][f][b] ? allowed : forbidden)++;
	// byte 2+3+2+4 = 11, half-word 1+2+1+3 = 7, word 1+1+1+2 = 5 allowed
	CHECK(allowed == 23);
	CHECK(forbidden == 25);
}

// characterization (proposed NEEDS REVIEW 305): RM0090 10.3.? note: with peripheral bursts and the FIFO in use, 3/4 threshold
// is forbidden when PBURST x PSIZE = FIFO size (16 bytes): (INCR4, word), (INCR8, half-word), (INCR16, byte). The HAL does not
// check this rule: HAL_OK, state READY. Review: should the HAL reject these (HAL_ERROR / PARAM) like the memory rule?
TEST_CASE_FIXTURE(HalFixture, "DI4 characterization: init with a peripheral burst that fills the FIFO and threshold 3/4 returns HAL_OK") {
	const unsigned burst[3] = {1, 2, 3};   // INCR4, INCR8, INCR16 as index of dmaInitPBursts
	const unsigned psize[3] = {2, 1, 0};   // word, half-word, byte: burst beats x size = 16 bytes
	for (int i = 0; i < 3; ++i) {
		DMA_InitTypeDef init = dmaInitBase();
		init.PeriphInc = DMA_PINC_ENABLE;
		init.PeriphDataAlignment = dmaInitPSizes[psize[i]];
		init.MemDataAlignment = dmaInitSizes[psize[i]];
		init.FIFOMode = DMA_FIFOMODE_ENABLE;
		init.FIFOThreshold = DMA_FIFO_THRESHOLD_3QUARTERSFULL;
		init.PeriphBurst = dmaInitPBursts[burst[i]];
		DMA_HandleTypeDef h = dmaInitHandle(0, 6, init);
		CHECK(HAL_DMA_Init(&h) == HAL_OK);
		CHECK(h.State == HAL_DMA_STATE_READY);
	}
}

// ---------------------------------------------------------------- DI5 direct mode

TEST_CASE_FIXTURE(HalFixture, "DI5 direct mode init accepts a burst size and threshold that FIFO mode would reject") {
	DMA_InitTypeDef init = dmaInitBase();
	init.MemInc = DMA_MINC_ENABLE;
	init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
	init.MemBurst = DMA_MBURST_INC16; // Table 49: forbidden for word / 1/4, but the FIFO is not used in direct mode
	init.PeriphBurst = DMA_PBURST_INC16;
	DMA_HandleTypeDef h = dmaInitHandle(0, 1, init);
	CHECK(HAL_DMA_Init(&h) == HAL_OK);
	CHECK(h.State == HAL_DMA_STATE_READY);
	CHECK(h.ErrorCode == HAL_DMA_ERROR_NONE);
}

TEST_CASE_FIXTURE(HalFixture, "DI5 direct mode stream has MBURST and PBURST at 0 once enabled") {
	DMA_InitTypeDef init = dmaInitBase();
	init.MemInc = DMA_MINC_ENABLE;
	init.PeriphInc = DMA_PINC_ENABLE;
	init.MemBurst = DMA_MBURST_INC8;
	init.PeriphBurst = DMA_PBURST_INC8;
	DMA_HandleTypeDef h = dmaInitHandle(1, 4, init);
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	// The HAL leaves the burst fields 0 in direct mode; set them by hand so that the hardware forcing is what is tested.
	poke(&DMA2_Stream4->CR, peek(&DMA2_Stream4->CR) | (0xFu << 21));
	REQUIRE((peek(&DMA2_Stream4->CR) & (0xFu << 21)) == (0xFu << 21));
	dmaInitEnable(DMA2_Stream4);
	// RM0090 10.5.5: in direct mode MBURST and PBURST are forced to 0 when EN = 1: bits 24:21.
	CHECK((peek(&DMA2_Stream4->CR) & (0xFu << 21)) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "DI5 direct mode stream has MSIZE equal to PSIZE once enabled although the structure differs") {
	DMA_InitTypeDef init = dmaInitBase();
	init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // PSIZE = 01
	init.MemDataAlignment = DMA_MDATAALIGN_WORD;        // MSIZE = 10 in the structure
	DMA_HandleTypeDef h = dmaInitHandle(0, 5, init);
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	dmaInitEnable(DMA1_Stream5);
	// RM0090 10.5.5: in direct mode MSIZE is forced to PSIZE on enable: MSIZE (14:13) = PSIZE (12:11) = 01.
	const uint32_t cr = peek(&DMA1_Stream5->CR);
	CHECK(((cr >> 13) & 3u) == 1u);
	CHECK(((cr >> 11) & 3u) == 1u);
}

TEST_CASE_FIXTURE(HalFixture, "DI5 FIFO mode stream keeps MBURST and MSIZE of the structure once enabled") {
	DMA_InitTypeDef init = dmaInitBase();
	init.MemInc = DMA_MINC_ENABLE;
	init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	init.FIFOMode = DMA_FIFOMODE_ENABLE;
	init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	init.MemBurst = DMA_MBURST_INC4; // word, full, INCR4: allowed
	DMA_HandleTypeDef h = dmaInitHandle(0, 5, init);
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	dmaInitEnable(DMA1_Stream5);
	const uint32_t cr = peek(&DMA1_Stream5->CR);
	CHECK(((cr >> 23) & 3u) == 1u); // MBURST stays INCR4 (01), forced only in direct mode
	CHECK(((cr >> 13) & 3u) == 2u); // MSIZE stays word (10) while PSIZE is byte
}

// ---------------------------------------------------------------- DI6 invalid handle

// characterization (proposed NEEDS REVIEW 309): UM1725 gives no return code for a NULL handle; the HAL returns HAL_ERROR.
TEST_CASE_FIXTURE(HalFixture, "DI6 characterization: init of a NULL handle returns HAL_ERROR and accesses no register") {
	CHECK(HAL_DMA_Init(nullptr) == HAL_ERROR);
	CHECK(hostRegsLogCount() == 0u);
}

// characterization (proposed NEEDS REVIEW 306): UM1725 does not define a stream that belongs to neither controller. The HAL
// takes the stream number from the address, writes the registers there and returns HAL_OK; the bus model reports the accesses
// as faults (no register at that address). Review: should HAL_DMA_Init reject an instance that is not a DMA stream?
TEST_CASE_FIXTURE(HalFixture, "DI6 characterization: init of a handle whose stream is in no DMA controller returns HAL_OK") {
	DMA_HandleTypeDef h = dmaInitHandle(0, 0, dmaInitBase());
	// 0x40026800 + 0x10 is where a third DMA controller's stream 0 would be (RM0090 Table 1 has only DMA1 and DMA2).
	h.Instance = reinterpret_cast<DMA_Stream_TypeDef *>(0x40026810u);
	CHECK(HAL_DMA_Init(&h) == HAL_OK);
	CHECK(faults() > 0u);
	clearFaults();
}

// ---------------------------------------------------------------- DI7 deinit

TEST_CASE_FIXTURE(HalFixture, "DI7 deinit writes the reset values to the six stream registers on each of the 16 streams") {
	DMA_INIT_EACH_STREAM_BEGIN
	DMA_InitTypeDef init = dmaInitBase();
	init.MemInc = DMA_MINC_ENABLE;
	init.Mode = DMA_CIRCULAR;
	init.Priority = DMA_PRIORITY_HIGH;
	init.FIFOMode = DMA_FIFOMODE_ENABLE;
	DMA_HandleTypeDef h = dmaInitHandle(c, s, init);
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	DMA_Stream_TypeDef *st = dmaInitStream(c, s);
	st->NDTR = 1234;
	st->PAR = 0x40012345u;
	st->M0AR = 0x20001000u;
	st->M1AR = 0x20002000u;
	st->FCR |= 0x80u; // FEIE
	REQUIRE(peek(&st->NDTR) == 1234u);
	REQUIRE(HAL_DMA_DeInit(&h) == HAL_OK);
	// RM0090 10.5.5-10.5.10 reset values: SxCR 0, SxNDTR 0, SxPAR 0, SxM0AR 0, SxM1AR 0, SxFCR 0x21 (FS = 100 empty, FTH = 01).
	CHECK(peek(&st->CR) == 0u);
	CHECK(peek(&st->NDTR) == 0u);
	CHECK(peek(&st->PAR) == 0u);
	CHECK(peek(&st->M0AR) == 0u);
	CHECK(peek(&st->M1AR) == 0u);
	CHECK(peek(&st->FCR) == 0x21u);
	DMA_INIT_EACH_STREAM_END
}

TEST_CASE_FIXTURE(HalFixture, "DI7 deinit clears all five flags of its stream and keeps those of the other streams") {
	DMA_INIT_EACH_STREAM_BEGIN
	DMA_HandleTypeDef h = dmaInitHandle(c, s, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	dmaInitRaiseAll(c);
	dmaInitRaiseAll(1 - c);
	REQUIRE(HAL_DMA_DeInit(&h) == HAL_OK);
	CHECK(hostDmaFlags(c, s) == 0u);
	for (unsigned s2 = 0; s2 < dmaInitStreams; ++s2) {
		if (s2 != s) CHECK(hostDmaFlags(c, s2) == dmaInitAllFlags);
		CHECK(hostDmaFlags(1 - c, s2) == dmaInitAllFlags);
	}
	DMA_INIT_EACH_STREAM_END
}

TEST_CASE_FIXTURE(HalFixture, "DI7 deinit resets the callbacks of the handle") {
	DMA_HandleTypeDef h = dmaInitHandle(0, 0, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	h.XferCpltCallback = dmaInitNoCallback;
	h.XferHalfCpltCallback = dmaInitNoCallback;
	h.XferM1CpltCallback = dmaInitNoCallback;
	h.XferM1HalfCpltCallback = dmaInitNoCallback;
	h.XferErrorCallback = dmaInitNoCallback;
	h.XferAbortCallback = dmaInitNoCallback;
	REQUIRE(HAL_DMA_DeInit(&h) == HAL_OK);
	CHECK(h.XferCpltCallback == nullptr);
	CHECK(h.XferHalfCpltCallback == nullptr);
	CHECK(h.XferM1CpltCallback == nullptr);
	CHECK(h.XferM1HalfCpltCallback == nullptr);
	CHECK(h.XferErrorCallback == nullptr);
	CHECK(h.XferAbortCallback == nullptr);
}

TEST_CASE_FIXTURE(HalFixture, "DI7 deinit resets the error code to NONE and the state to RESET") {
	DMA_HandleTypeDef h = dmaInitHandle(1, 2, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	h.ErrorCode = HAL_DMA_ERROR_TE | HAL_DMA_ERROR_FE;
	REQUIRE(HAL_DMA_DeInit(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_DMA_ERROR_NONE);
	CHECK(h.State == HAL_DMA_STATE_RESET);
}

TEST_CASE_FIXTURE(HalFixture, "DI7 deinit leaves the process unlocked") {
	DMA_HandleTypeDef h = dmaInitHandle(1, 2, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	h.Lock = HAL_LOCKED;
	REQUIRE(HAL_DMA_DeInit(&h) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(HalFixture, "DI7 deinit of an enabled stream that stops at once leaves EN = 0 and all registers reset") {
	DMA_HandleTypeDef h = dmaInitHandle(0, 4, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	DMA1_Stream4->PAR = 0x40000000u;
	DMA1_Stream4->M0AR = 0x20000000u;
	dmaInitEnable(DMA1_Stream4);
	REQUIRE(HAL_DMA_DeInit(&h) == HAL_OK);
	CHECK((peek(&DMA1_Stream4->CR) & DMA_SxCR_EN) == 0u);
	CHECK(peek(&DMA1_Stream4->NDTR) == 0u);
	CHECK(peek(&DMA1_Stream4->PAR) == 0u);
	CHECK(peek(&DMA1_Stream4->M0AR) == 0u);
}

// KNOWN ISSUE, candidate vendor bug V94 (charter numbering from V94): the stream stops after 3 ticks (RM0090 10.5.5: EN is
// cleared "after the current data transfer"); until EN reads 0 writes to SxNDTR, SxPAR, SxM0AR, SxM1AR, SxFCR and the protected
// SxCR bits are ignored (10.5.5-10.5.10). HAL_DMA_DeInit requests the disable and writes the reset values at once without
// waiting for EN = 0 (HAL_DMA_Init does wait), so the reset values are lost. First assertion: no write was ignored.
TEST_CASE_FIXTURE(HalFixture, "DI7 deinit of a stream that takes 3 ticks to stop resets all registers" * KNOWN_ISSUE("candidate vendor bug V94: HAL_DMA_DeInit requests the stop and writes the reset values at once; with EN still 1 the writes to NDTR / PAR / M0AR / M1AR / FCR are ignored (RM0090 10.5.5-10.5.10)")) {
	hostDmaSetDisableDelay(3);
	DMA_HandleTypeDef h = dmaInitHandle(0, 4, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	DMA1_Stream4->PAR = 0x40000000u;
	DMA1_Stream4->M0AR = 0x20000000u;
	dmaInitEnable(DMA1_Stream4);
	REQUIRE(HAL_DMA_DeInit(&h) == HAL_OK);
	CHECK(hostDmaProtectedWrites(0, 4) == 0u);
	CHECK(peek(&DMA1_Stream4->NDTR) == 0u);
	CHECK(peek(&DMA1_Stream4->PAR) == 0u);
	CHECK(peek(&DMA1_Stream4->M0AR) == 0u);
}

// No HAL status is asserted here (the intended wait / status is the V94 case above; UM1725 gives DeInit no timeout): only what
// is not in dispute, that the handle is cleaned and the call returns.
TEST_CASE_FIXTURE(HalFixture, "DI7 deinit of a stream whose EN never clears still resets the handle and returns") {
	DMA_HandleTypeDef h = dmaInitHandle(1, 5, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	h.XferCpltCallback = dmaInitNoCallback;
	h.ErrorCode = HAL_DMA_ERROR_TE;
	dmaInitEnable(DMA2_Stream5);
	hostDmaStickEnabled(1, 5, 1);
	(void)HAL_DMA_DeInit(&h);
	CHECK(h.State == HAL_DMA_STATE_RESET);
	CHECK(h.XferCpltCallback == nullptr);
	CHECK(h.ErrorCode == HAL_DMA_ERROR_NONE);
}

// characterization (proposed NEEDS REVIEW 310): as 309 for HAL_DMA_DeInit.
TEST_CASE_FIXTURE(HalFixture, "DI7 characterization: deinit of a NULL handle returns HAL_ERROR and accesses no register") {
	CHECK(HAL_DMA_DeInit(nullptr) == HAL_ERROR);
	CHECK(hostRegsLogCount() == 0u);
}

// characterization (proposed NEEDS REVIEW 308): UM1725 gives no status for DeInit during a transfer. The HAL returns HAL_BUSY
// when the state is BUSY and changes nothing (registers and state keep their values).
TEST_CASE_FIXTURE(HalFixture, "DI7 characterization: deinit of a handle in state BUSY returns HAL_BUSY and changes nothing") {
	DMA_HandleTypeDef h = dmaInitHandle(0, 1, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	DMA1_Stream1->M0AR = 0x20000000u;
	h.State = HAL_DMA_STATE_BUSY;
	CHECK(HAL_DMA_DeInit(&h) == HAL_BUSY);
	CHECK(h.State == HAL_DMA_STATE_BUSY);
	CHECK(peek(&DMA1_Stream1->M0AR) == 0x20000000u);
}

// ---------------------------------------------------------------- DI8 state and error code

TEST_CASE_FIXTURE(HalFixture, "DI8 the state of a handle that was never initialised is RESET") {
	DMA_HandleTypeDef h = {};
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_RESET);
}

TEST_CASE_FIXTURE(HalFixture, "DI8 the state is READY after init") {
	DMA_HandleTypeDef h = dmaInitHandle(0, 0, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_READY);
}

TEST_CASE_FIXTURE(HalFixture, "DI8 the state is BUSY after a transfer was started") {
	DMA_HandleTypeDef h = dmaInitHandle(0, 0, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	REQUIRE(HAL_DMA_Start(&h, 0x40004400u, 0x20000000u, 4) == HAL_OK);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_BUSY);
}

TEST_CASE_FIXTURE(HalFixture, "DI8 the state is RESET after deinit") {
	DMA_HandleTypeDef h = dmaInitHandle(0, 0, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	REQUIRE(HAL_DMA_DeInit(&h) == HAL_OK);
	CHECK(HAL_DMA_GetState(&h) == HAL_DMA_STATE_RESET);
}

TEST_CASE_FIXTURE(HalFixture, "DI8 the error code is NONE after a successful init") {
	DMA_HandleTypeDef h = dmaInitHandle(1, 1, dmaInitBase());
	h.ErrorCode = HAL_DMA_ERROR_DME;
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_NONE);
}

TEST_CASE_FIXTURE(HalFixture, "DI8 GetError returns every error code bit of the handle") {
	DMA_HandleTypeDef h = dmaInitHandle(1, 1, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	h.ErrorCode = HAL_DMA_ERROR_TE | HAL_DMA_ERROR_FE | HAL_DMA_ERROR_DME | HAL_DMA_ERROR_TIMEOUT | HAL_DMA_ERROR_PARAM |
				  HAL_DMA_ERROR_NO_XFER | HAL_DMA_ERROR_NOT_SUPPORTED;
	CHECK(HAL_DMA_GetError(&h) == (0x01u | 0x02u | 0x04u | 0x20u | 0x40u | 0x80u | 0x100u)); // UM1725 error code values
}

TEST_CASE_FIXTURE(HalFixture, "DI8 GetError returns PARAM after a rejected init") {
	DMA_InitTypeDef init = dmaInitBase();
	init.MemInc = DMA_MINC_ENABLE;
	init.FIFOMode = DMA_FIFOMODE_ENABLE;
	init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
	init.MemBurst = DMA_MBURST_INC8; // byte, 1/4, INCR8: forbidden by Table 49
	DMA_HandleTypeDef h = dmaInitHandle(0, 0, init);
	REQUIRE(HAL_DMA_Init(&h) != HAL_OK);
	CHECK(HAL_DMA_GetError(&h) == HAL_DMA_ERROR_PARAM);
}

// ---------------------------------------------------------------- DI9 stream base address, bit position

TEST_CASE_FIXTURE(HalFixture, "DI9 each stream gets the flag bit offset of its group after init") {
	// RM0090 10.5.1-10.5.4: offsets 0, 6, 16, 22 for streams 0 / 4, 1 / 5, 2 / 6, 3 / 7.
	const uint32_t offsets[8] = {0, 6, 16, 22, 0, 6, 16, 22};
	DMA_INIT_EACH_STREAM_BEGIN
	DMA_HandleTypeDef h = dmaInitHandle(c, s, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	CHECK(h.StreamIndex == offsets[s]);
	DMA_INIT_EACH_STREAM_END
}

TEST_CASE_FIXTURE(HalFixture, "DI9 each stream gets the address of LISR (streams 0-3) or HISR (streams 4-7) of its controller after init") {
	// RM0090 Table 1: DMA1 0x40026000, DMA2 0x40026400; LISR at +0, HISR at +4 (10.5.1, 10.5.2).
	const uint32_t base[2] = {0x40026000u, 0x40026400u};
	DMA_INIT_EACH_STREAM_BEGIN
	DMA_HandleTypeDef h = dmaInitHandle(c, s, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	CHECK(h.StreamBaseAddress == base[c] + (s < 4 ? 0u : 4u));
	DMA_INIT_EACH_STREAM_END
}

TEST_CASE_FIXTURE(HalFixture, "DI9 starting a transfer with interrupts clears only the flags of its own stream") {
	// RM0090 10.3: the flags of the stream must be cleared before it is enabled; the other streams' flags are not its business.
	DMA_INIT_EACH_STREAM_BEGIN
	DMA_HandleTypeDef h = dmaInitHandle(c, s, dmaInitBase());
	REQUIRE(HAL_DMA_Init(&h) == HAL_OK);
	dmaInitRaiseAll(c);
	dmaInitRaiseAll(1 - c);
	REQUIRE(HAL_DMA_Start_IT(&h, 0x40004400u, 0x20000000u, 4) == HAL_OK);
	CHECK(hostDmaFlags(c, s) == 0u);
	for (unsigned s2 = 0; s2 < dmaInitStreams; ++s2) {
		if (s2 != s) CHECK(hostDmaFlags(c, s2) == dmaInitAllFlags);
		CHECK(hostDmaFlags(1 - c, s2) == dmaInitAllFlags);
	}
	DMA_INIT_EACH_STREAM_END
}

} // TEST_SUITE
