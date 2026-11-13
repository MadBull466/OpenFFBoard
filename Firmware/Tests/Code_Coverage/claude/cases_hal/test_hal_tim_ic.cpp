/*
 * P12 T6a group TI: input capture, one pulse, encoder interface and hall sensor interface of the real ST HAL timer driver
 * (stm32f4xx_hal_tim.c, stm32f4xx_hal_tim_ex.c) against the timer model (fakes/stm32_hal/HalTim.h) and the DMA model (HalDma.h).
 * Requirements TI1-TI4 of Tests/Code_Coverage/claude/charters/hal/P12_T6A_REQUIREMENTS.md; compiled into ffb_hal_tests (F407VG)
 * and ffb_hal_tests_disco. File-local symbols sit in the anonymous namespace.
 *
 * Oracles: UM1725 Rev 8 (HAL TIM Generic / Extension Driver: Init / DeInit / Start / Stop return a HAL status, states) and RM0090 Rev 22:
 *  - 18.4.7 TIMx_CCMR1 input mode: CC1S bits 1:0 (01 = TI1, 10 = TI2, 11 = TRC), IC1PSC bits 3:2 (capture every 1, 2, 4, 8 events),
 *    IC1F bits 7:4, CC2S bits 9:8, IC2PSC 11:10, IC2F 15:12; CCMR2 the same for channels 3 / 4. Output mode: OC1M bits 6:4, OC2M bits 14:12.
 *  - 18.4.9 TIMx_CCER: CCxE bit 4(x-1), CCxP bit 4(x-1)+1, CCxNP bit 4(x-1)+3. 18.4.9 DIER: UIE bit 0, CCxIE bit x, CCxDE bit 8+x.
 *  - 18.4.3 TIMx_SMCR: SMS bits 2:0 (001 / 010 / 011 encoder modes 1 / 2 / 3, 100 reset, 110 trigger), TS bits 6:4 (100 TI1F_ED, 101 TI1FP1,
 *    110 TI2FP2). 18.4.2 CR2: MMS bits 6:4 (101 = OC2REF), TI1S bit 7. 18.4.1 CR1: CEN bit 0, OPM bit 3.
 *  - 17.3.15 one pulse mode (TI2FP2 trigger: CC2S = 01, TS = 110, SMS = 110; the counter is started by the trigger, not by software),
 *    17.3.16 encoder interface, 17.3.18 hall sensor interface (TI1S = 1, SMS = 100, TS = 100, CC1S = 11 (TRC), OC2M = 111 with CC2S = 00, MMS = 101).
 *  - Table 43 (DMA1 request mapping): TIM3_CH1 stream 4, TIM3_CH2 stream 5, TIM3_CH3 stream 7, TIM3_CH4 stream 2, all channel 5.
 *  - Table 1: TIM3 0x40000400, CCR1 offset 0x34. 10.5.5 DMA_SxCR: EN bit 0, TEIE bit 2, HTIE bit 3, TCIE bit 4.
 * Time never matters here. Interrupt dispatch (HAL_TIM_IRQHandler) belongs to group TQ.
 *
 * NEEDS REVIEW proposals (the lead numbers them):
 *  NEEDS REVIEW 333/a TI1: IC Start / Stop / Start_IT / Stop_IT / Start_DMA / Stop_DMA with TIM_CHANNEL_ALL (not a valid IC channel): poll Start returns
 *     HAL_OK and marks ChannelState[3] BUSY; Start_IT / Start_DMA return HAL_ERROR but leave ChannelState[3] and ChannelNState[3] BUSY (Start_DMA also
 *     leaves the counter enabled); Stop_IT / Stop_DMA return HAL_ERROR and change nothing. UM1725 only names channels 1-4.
 *  (NEEDS REVIEW 333/b became the KNOWN_ISSUE cases V112, arguable) TI1 / TI3 / TI4: when Start_DMA fails because HAL_DMA_Start_IT rejects the stream (stream not READY) the HAL returns HAL_ERROR but
 *     the channel states stay BUSY and CCxE stays set (nothing is rolled back).
 *  NEEDS REVIEW 333/c TI1 / TI3: a Stop of one channel while another channel is still enabled keeps CEN set (the HAL macro __HAL_TIM_DISABLE clears CEN only when
 *     no CCxE / CCxNE is set); IC_DeInit / Encoder_DeInit leave CEN set while a channel is still enabled. UM1725 does not describe this.
 *  NEEDS REVIEW 333/d TI1: HAL_TIM_ReadCapturedValue of TIM_CHANNEL_ALL returns 0; IC_ConfigChannel with channel 3 on a 2 channel timer (TIM9) touches CCMR2
 *     (a register the timer does not have, model fault) and returns HAL_OK; IC_ConfigChannel with a non channel value returns HAL_ERROR.
 *  NEEDS REVIEW 333/e TI2: OnePulse_ConfigChannel clears the IC prescaler of the input channel and keeps a half done configuration when the input channel is
 *     invalid (output channel configured, HAL_ERROR); OnePulse start / stop use TIM_CHANNEL_1 and 2 whatever OutputChannel says.
 *  NEEDS REVIEW 333/f TI3: HAL_TIM_Encoder_Init on TIM12 (TIM9 is the same) accepts and writes SMS = encoder mode although RM0090 chapter 19 (TIM9 / TIM12 SMCR) lists no encoder
 *     modes (the device header macro IS_TIM_ENCODER_INTERFACE_INSTANCE includes TIM9 and TIM12). Any channel value other than 1 or 2 is treated as both.
 *  NEEDS REVIEW 333/g TI3: Encoder Start_DMA with the other channel's data pointer NULL (channel 1 with pData2 NULL, channel 2 with pData1 NULL) is accepted.
 *  NEEDS REVIEW 333/h TI1: Start_DMA right after Stop_DMA returns HAL_ERROR until the stream's abort interrupt ran (HAL_DMA_Abort_IT leaves the DMA state ABORT).
 */
#include "HalDma.h"
#include "HalFixture.h"
#include "HalHostSystem.h"
#include "HalPeripherals.h"
#include "HalTim.h"
#include "TestHelpers.h"
#include "TimTestSupport.h"

#include <cstdint>
#include <cstring>

namespace {

constexpr uint32_t tiCR1_CEN = 1u << 0;
constexpr uint32_t tiCR1_OPM = 1u << 3;
constexpr uint32_t tiBDTR_MOE = 1u << 15;
constexpr uint32_t tiCh[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
constexpr uint32_t tiBusy = HAL_TIM_CHANNEL_STATE_BUSY;
constexpr uint32_t tiReady = HAL_TIM_CHANNEL_STATE_READY;

constexpr uint32_t tiCcxe(unsigned c) { return 1u << (4 * (c - 1)); }
constexpr uint32_t tiCcie(unsigned c) { return 1u << c; }
constexpr uint32_t tiCcde(unsigned c) { return 1u << (8 + c); }
constexpr uint32_t tiPol(unsigned c, unsigned pol) { return pol == 0 ? 0u : pol == 1 ? (2u << (4 * (c - 1))) : (0xAu << (4 * (c - 1))); }

/* RM0090 Table 43: TIM3 CCx request: stream and channel 5 of DMA1 (controller index 0). */
constexpr unsigned tiStreamOf[4] = {4, 5, 7, 2};

DMA_Stream_TypeDef *tiStream(unsigned stream) {
	DMA_Stream_TypeDef *const s[8] = {DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7};
	return s[stream];
}

unsigned tiChannelsOf(unsigned tim) {
	if (tim == 9 || tim == 12) return 2;
	if (tim == 10 || tim == 11 || tim == 13 || tim == 14) return 1;
	return 4;
}
constexpr unsigned tiIcTimers[12] = {1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13, 14};
constexpr unsigned tiEncTimers[7] = {1, 2, 3, 4, 5, 8, 12};
constexpr unsigned tiHallTimers[5] = {2, 3, 4, 5, 8};

struct TiRig {
	TIM_HandleTypeDef h;
	DMA_HandleTypeDef dma[4];
	uint32_t buf[16], buf2[16];
	unsigned tim;
	TIM_TypeDef *t;

	explicit TiRig(unsigned timNo, bool linkDma = false, uint32_t dmaMode = DMA_NORMAL) : tim(timNo), t(timInstance(timNo)) {
		hostPeriphResetState(); // a case may build several rigs: start from the reset state of all peripherals
		hostRegsReset(); // this disarms the bus: arm it again so the register models stay in force
		hostRegsArm();
		std::memset(&h, 0, sizeof h);
		std::memset(dma, 0, sizeof dma);
		std::memset(buf, 0, sizeof buf);
		std::memset(buf2, 0, sizeof buf2);
		h.Instance = t;
		h.Init.Prescaler = 7;
		h.Init.CounterMode = TIM_COUNTERMODE_UP;
		h.Init.Period = 999;
		h.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		h.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (linkDma) {
			REQUIRE(timNo == 3u);
			for (unsigned c = 0; c < 4; ++c) {
				dma[c].Instance = tiStream(tiStreamOf[c]);
				dma[c].Init.Channel = DMA_CHANNEL_5;
				dma[c].Init.Direction = DMA_PERIPH_TO_MEMORY;
				dma[c].Init.PeriphInc = DMA_PINC_DISABLE;
				dma[c].Init.MemInc = DMA_MINC_ENABLE;
				dma[c].Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
				dma[c].Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
				dma[c].Init.Mode = dmaMode;
				dma[c].Init.Priority = DMA_PRIORITY_LOW;
				dma[c].Init.FIFOMode = DMA_FIFOMODE_DISABLE;
				dma[c].Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
				dma[c].Init.MemBurst = DMA_MBURST_SINGLE;
				dma[c].Init.PeriphBurst = DMA_PBURST_SINGLE;
				REQUIRE(HAL_DMA_Init(&dma[c]) == HAL_OK);
				dma[c].Parent = &h;
				h.hdma[TIM_DMA_ID_CC1 + c] = &dma[c];
			}
		}
		timSupportReset();
	}
	uint32_t addr() const { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buf)); }
	uint32_t addr2() const { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buf2)); }
	unsigned state(unsigned c) const { return h.ChannelState[c - 1]; }
	unsigned nState(unsigned c) const { return h.ChannelNState[c - 1]; }
	bool cen() const { return (t->CR1 & tiCR1_CEN) != 0u; }
	bool moe() const { return (t->BDTR & tiBDTR_MOE) != 0u; }
	bool streamOn(unsigned c) const { return (tiStream(tiStreamOf[c - 1])->CR & 1u) != 0u; }
	uint32_t ccrAddr(unsigned c) const { return static_cast<uint32_t>(0x40000400u + 0x34u + 4u * (c - 1u)); }
	void allStates(unsigned s) {
		for (unsigned i = 0; i < 4; ++i) { h.ChannelState[i] = static_cast<HAL_TIM_ChannelStateTypeDef>(s); h.ChannelNState[i] = static_cast<HAL_TIM_ChannelStateTypeDef>(s); }
	}
};

TIM_IC_InitTypeDef tiIc(uint32_t pol, uint32_t sel, uint32_t psc, uint32_t filt) {
	TIM_IC_InitTypeDef c{};
	c.ICPolarity = pol;
	c.ICSelection = sel;
	c.ICPrescaler = psc;
	c.ICFilter = filt;
	return c;
}

/* IC start family: mode 0 poll, 1 IT, 2 DMA. */
HAL_StatusTypeDef tiIcStart(TiRig &r, int mode, uint32_t ch, uint16_t len = 4) {
	return mode == 0 ? HAL_TIM_IC_Start(&r.h, ch) : mode == 1 ? HAL_TIM_IC_Start_IT(&r.h, ch) : HAL_TIM_IC_Start_DMA(&r.h, ch, r.buf, len);
}
HAL_StatusTypeDef tiIcStop(TiRig &r, int mode, uint32_t ch) {
	return mode == 0 ? HAL_TIM_IC_Stop(&r.h, ch) : mode == 1 ? HAL_TIM_IC_Stop_IT(&r.h, ch) : HAL_TIM_IC_Stop_DMA(&r.h, ch);
}
/* the abort of a stream (HAL_DMA_Abort_IT) ends with the transfer complete interrupt: raise TCIF and run the handler */
void tiFinishAbort(TiRig &r, unsigned idx) {
	hostDmaRaise(0, tiStreamOf[idx], HOST_DMA_TCIF);
	HAL_DMA_IRQHandler(&r.dma[idx]);
}
constexpr uint32_t tiReq(int mode, unsigned c) { return mode == 1 ? tiCcie(c) : mode == 2 ? tiCcde(c) : 0u; }
constexpr uint32_t tiReqAll = 0x1E1Fu | (0x1Fu << 8); // every IE / DE bit a start can set (UIE, CCxIE, CCxDE, UDE)

uint32_t tiCcmrByte(const TIM_TypeDef *t, unsigned c) {
	const uint32_t v = (c <= 2) ? t->CCMR1 : t->CCMR2;
	return (v >> (((c - 1) & 1u) * 8u)) & 0xFFu;
}

} // namespace

/* ================================================================ TI1: input capture */

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Init of a NULL handle returns HAL_ERROR") { CHECK(HAL_TIM_IC_Init(nullptr) == HAL_ERROR); }

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Init of a RESET handle runs the IC MspInit once, unlocks it and leaves state and all channel states READY") {
	for (unsigned tim : tiIcTimers) {
		CAPTURE(tim);
		TiRig r(tim);
		r.h.Lock = HAL_LOCKED;
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		CHECK(timCounts.mspInit[3] == 1u);
		CHECK(timCounts.mspInit[0] == 0u);
		CHECK(r.h.Lock == HAL_UNLOCKED);
		CHECK(HAL_TIM_IC_GetState(&r.h) == HAL_TIM_STATE_READY);
		for (unsigned c = 1; c <= 4; ++c) {
			CHECK(r.state(c) == tiReady);
			CHECK(r.nState(c) == tiReady);
		}
		CHECK(r.h.DMABurstState == HAL_DMA_BURST_STATE_READY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Init of an already initialised handle does not run MspInit again") {
	TiRig r(3);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	CHECK(timCounts.mspInit[3] == 1u);
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Init writes Period to ARR and Prescaler to PSC on every timer") {
	for (unsigned tim : tiIcTimers) {
		CAPTURE(tim);
		TiRig r(tim);
		r.h.Init.Period = 0xABC;
		r.h.Init.Prescaler = 0x123;
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		CHECK(r.t->ARR == 0xABCu);
		CHECK(r.t->PSC == 0x123u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Init of a down counting TIM3 sets DIR (RM0090 18.4.1 CR1 bit 4)") {
	TiRig r(3);
	r.h.Init.CounterMode = TIM_COUNTERMODE_DOWN;
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	CHECK((r.t->CR1 & 0x10u) == 0x10u);
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_DeInit runs MspDeInit once, stops the counter, sets state and all channel states to RESET and unlocks") {
	for (unsigned tim : tiIcTimers) {
		CAPTURE(tim);
		TiRig r(tim);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		r.t->CR1 |= tiCR1_CEN;
		r.h.Lock = HAL_LOCKED;
		REQUIRE(HAL_TIM_IC_DeInit(&r.h) == HAL_OK);
		CHECK(timCounts.mspDeInit[3] == 1u);
		CHECK(timCounts.stateAtLast == HAL_TIM_STATE_BUSY); // the state is BUSY while MspDeInit runs
		CHECK(HAL_TIM_IC_GetState(&r.h) == HAL_TIM_STATE_RESET);
		CHECK(r.cen() == false);
		CHECK(r.h.Lock == HAL_UNLOCKED);
		for (unsigned c = 1; c <= 4; ++c) {
			CHECK(r.state(c) == HAL_TIM_CHANNEL_STATE_RESET);
			CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_RESET);
		}
		CHECK(r.h.DMABurstState == HAL_DMA_BURST_STATE_RESET);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Init after IC_DeInit runs MspInit again") {
	TiRig r(3);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_IC_DeInit(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	CHECK(timCounts.mspInit[3] == 2u);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI1 IC_DeInit leaves CEN set while a channel is still enabled") {
	TiRig r(3);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_IC_Start(&r.h, TIM_CHANNEL_1) == HAL_OK);
	REQUIRE(HAL_TIM_IC_DeInit(&r.h) == HAL_OK);
	CHECK(r.cen() == true); // NEEDS REVIEW 333/c
}

/* ---- ConfigChannel */

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_ConfigChannel sets the polarity bits CCxP / CCxNP of the channel (rising, falling, both edges) and keeps the others") {
	const uint32_t pols[3] = {TIM_ICPOLARITY_RISING, TIM_ICPOLARITY_FALLING, TIM_ICPOLARITY_BOTHEDGE};
	for (unsigned c = 1; c <= 4; ++c)
		for (unsigned p = 0; p < 3; ++p) {
			CAPTURE(c);
			CAPTURE(p);
			TiRig r(3);
			REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
			HalFixture::poke(&r.t->CCER, 0xAAAAu); // every CCxP / CCxNP set
			const TIM_IC_InitTypeDef cfg = tiIc(pols[p], TIM_ICSELECTION_DIRECTTI, TIM_ICPSC_DIV1, 0);
			REQUIRE(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, tiCh[c - 1]) == HAL_OK);
			const uint32_t own = 0xAu << (4 * (c - 1));
			CHECK(r.t->CCER == ((0xAAAAu & ~own) | tiPol(c, p)));
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_ConfigChannel writes the selection direct / indirect / TRC into CCxS of the channel and keeps the other channel of the register") {
	const uint32_t sels[3] = {TIM_ICSELECTION_DIRECTTI, TIM_ICSELECTION_INDIRECTTI, TIM_ICSELECTION_TRC};
	const uint32_t want[3] = {1u, 2u, 3u}; // RM0090 18.4.7 CCxS = 01 / 10 / 11
	for (unsigned c = 1; c <= 4; ++c)
		for (unsigned s = 0; s < 3; ++s) {
			CAPTURE(c);
			CAPTURE(s);
			TiRig r(3);
			REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
			const unsigned other = (c == 1) ? 2 : (c == 2) ? 1 : (c == 3) ? 4 : 3;
			(c <= 2 ? r.t->CCMR1 : r.t->CCMR2) = (c == 1 || c == 3) ? 0x5A00u : 0x005Au;
			const TIM_IC_InitTypeDef cfg = tiIc(TIM_ICPOLARITY_RISING, sels[s], TIM_ICPSC_DIV1, 0);
			REQUIRE(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, tiCh[c - 1]) == HAL_OK);
			CHECK(tiCcmrByte(r.t, c) == want[s]);
			CHECK(tiCcmrByte(r.t, other) == 0x5Au);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_ConfigChannel writes the prescaler 1 / 2 / 4 / 8 into ICxPSC of the channel") {
	const uint32_t pscs[4] = {TIM_ICPSC_DIV1, TIM_ICPSC_DIV2, TIM_ICPSC_DIV4, TIM_ICPSC_DIV8};
	for (unsigned c = 1; c <= 4; ++c)
		for (unsigned p = 0; p < 4; ++p) {
			CAPTURE(c);
			CAPTURE(p);
			TiRig r(3);
			REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
			(c <= 2 ? r.t->CCMR1 : r.t->CCMR2) = (c == 1 || c == 3) ? 0x000Cu : 0x0C00u; // old prescaler 8 must be replaced
			const TIM_IC_InitTypeDef cfg = tiIc(TIM_ICPOLARITY_RISING, TIM_ICSELECTION_DIRECTTI, pscs[p], 0);
			REQUIRE(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, tiCh[c - 1]) == HAL_OK);
			CHECK(tiCcmrByte(r.t, c) == (1u | (p << 2))); // RM0090: IC1PSC bits 3:2 = 00 / 01 / 10 / 11
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_ConfigChannel writes the filter 0 / 1 / 7 / 15 into ICxF of the channel") {
	const uint32_t filters[4] = {0, 1, 7, 15};
	for (unsigned c = 1; c <= 4; ++c)
		for (uint32_t f : filters) {
			CAPTURE(c);
			CAPTURE(f);
			TiRig r(3);
			REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
			(c <= 2 ? r.t->CCMR1 : r.t->CCMR2) = (c == 1 || c == 3) ? 0x00F0u : 0xF000u; // old filter 15 must be replaced
			const TIM_IC_InitTypeDef cfg = tiIc(TIM_ICPOLARITY_RISING, TIM_ICSELECTION_DIRECTTI, TIM_ICPSC_DIV1, f);
			REQUIRE(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, tiCh[c - 1]) == HAL_OK);
			CHECK(tiCcmrByte(r.t, c) == (1u | (f << 4))); // IC1F bits 7:4
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_ConfigChannel of an enabled channel changes CCxS without a write the model rejects and leaves CCxE as it was") {
	for (unsigned c = 1; c <= 4; ++c) {
		CAPTURE(c);
		TiRig r(3);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		const uint32_t direct = (c == 1 || c == 3) ? 0x01u : 0x0100u;
		(c <= 2 ? r.t->CCMR1 : r.t->CCMR2) = direct;
		r.t->CCER = tiCcxe(c);
		const TIM_IC_InitTypeDef cfg = tiIc(TIM_ICPOLARITY_FALLING, TIM_ICSELECTION_INDIRECTTI, TIM_ICPSC_DIV1, 0);
		REQUIRE(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, tiCh[c - 1]) == HAL_OK);
		CHECK(tiCcmrByte(r.t, c) == 2u);
		CHECK((r.t->CCER & tiCcxe(c)) == tiCcxe(c));
		CHECK(hostTimProtectedWrites(3) == 0u); // RM0090: CCxS writable only while the channel is off, so the HAL must switch it off for the write
		CHECK(HalFixture::faults() == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_ConfigChannel returns HAL_OK and leaves the handle unlocked and its state unchanged") {
	TiRig r(3);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	const TIM_IC_InitTypeDef cfg = tiIc(TIM_ICPOLARITY_RISING, TIM_ICSELECTION_DIRECTTI, TIM_ICPSC_DIV1, 0);
	CHECK(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_2) == HAL_OK);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	CHECK(HAL_TIM_IC_GetState(&r.h) == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_ConfigChannel of a locked handle returns HAL_BUSY and writes nothing") {
	TiRig r(3);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	r.h.Lock = HAL_LOCKED;
	const uint32_t ccmr = r.t->CCMR1, ccer = r.t->CCER;
	const TIM_IC_InitTypeDef cfg = tiIc(TIM_ICPOLARITY_BOTHEDGE, TIM_ICSELECTION_TRC, TIM_ICPSC_DIV8, 15);
	CHECK(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_1) == HAL_BUSY);
	CHECK(r.t->CCMR1 == ccmr);
	CHECK(r.t->CCER == ccer);
	CHECK(r.h.Lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI1 IC_ConfigChannel with an invalid channel returns HAL_ERROR, writes nothing and unlocks") {
	TiRig r(3);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	const uint32_t ccmr1 = r.t->CCMR1, ccmr2 = r.t->CCMR2, ccer = r.t->CCER;
	const TIM_IC_InitTypeDef cfg = tiIc(TIM_ICPOLARITY_BOTHEDGE, TIM_ICSELECTION_TRC, TIM_ICPSC_DIV8, 15);
	CHECK(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_ALL) == HAL_ERROR); // NEEDS REVIEW 333/d
	CHECK(r.t->CCMR1 == ccmr1);
	CHECK(r.t->CCMR2 == ccmr2);
	CHECK(r.t->CCER == ccer);
	CHECK(r.h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_ConfigChannel of channel 1 and 2 on the 2 channel timers TIM9 and TIM12 and of channel 1 on TIM10 / 11 / 13 / 14") {
	const unsigned tims[6] = {9, 12, 10, 11, 13, 14};
	for (unsigned tim : tims) {
		CAPTURE(tim);
		TiRig r(tim);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		const TIM_IC_InitTypeDef cfg = tiIc(TIM_ICPOLARITY_FALLING, TIM_ICSELECTION_DIRECTTI, TIM_ICPSC_DIV4, 3);
		REQUIRE(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_1) == HAL_OK);
		CHECK(tiCcmrByte(r.t, 1) == (1u | (2u << 2) | (3u << 4)));
		CHECK((r.t->CCER & 0xAu) == 2u);
		if (tiChannelsOf(tim) == 2) {
			REQUIRE(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_2) == HAL_OK);
			CHECK(tiCcmrByte(r.t, 2) == (1u | (2u << 2) | (3u << 4)));
			CHECK((r.t->CCER & 0xA0u) == 0x20u);
		}
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_ConfigChannel of a one channel timer selects TI1 (CC1S = 01) whatever selection is asked (RM0090 19.5.5: only 00 / 01 exist)") {
	const unsigned tims[4] = {10, 11, 13, 14};
	for (unsigned tim : tims) {
		CAPTURE(tim);
		TiRig r(tim);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		const TIM_IC_InitTypeDef cfg = tiIc(TIM_ICPOLARITY_RISING, TIM_ICSELECTION_TRC, TIM_ICPSC_DIV1, 0);
		REQUIRE(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_1) == HAL_OK);
		CHECK((r.t->CCMR1 & 3u) == 1u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI1 IC_ConfigChannel channel 3 on TIM9 (no CCMR2) returns HAL_OK and touches the missing register") {
	TiRig r(9);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	const TIM_IC_InitTypeDef cfg = tiIc(TIM_ICPOLARITY_RISING, TIM_ICSELECTION_DIRECTTI, TIM_ICPSC_DIV1, 0);
	CHECK(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_3) == HAL_OK); // NEEDS REVIEW 333/d
	CHECK(HalFixture::faults() > 0u); // TIM9 has no CCMR2 (RM0090 chapter 19): the model faults the access
	HalFixture::clearFaults();
}

/* ---- Start / Stop */

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Start enables CCxE of the channel and the counter and sets both channel states BUSY (every timer, every channel)") {
	for (unsigned tim : tiIcTimers)
		for (unsigned c = 1; c <= tiChannelsOf(tim); ++c) {
			CAPTURE(tim);
			CAPTURE(c);
			TiRig r(tim);
			REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
			REQUIRE(HAL_TIM_IC_Start(&r.h, tiCh[c - 1]) == HAL_OK);
			CHECK((r.t->CCER & tiCcxe(c)) == tiCcxe(c));
			CHECK(r.cen() == true);
			CHECK(r.state(c) == tiBusy);
			CHECK(r.nState(c) == tiBusy);
			CHECK((r.t->DIER & tiReqAll) == 0u);
			for (unsigned o = 1; o <= 4; ++o)
				if (o != c) CHECK(r.state(o) == tiReady);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Start_IT enables the interrupt CCxIE of that channel only, CCxE and the counter (every timer, every channel)") {
	for (unsigned tim : tiIcTimers)
		for (unsigned c = 1; c <= tiChannelsOf(tim); ++c) {
			CAPTURE(tim);
			CAPTURE(c);
			TiRig r(tim);
			REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
			REQUIRE(HAL_TIM_IC_Start_IT(&r.h, tiCh[c - 1]) == HAL_OK);
			CHECK((r.t->DIER & tiReqAll) == tiCcie(c));
			CHECK((r.t->CCER & tiCcxe(c)) == tiCcxe(c));
			CHECK(r.cen() == true);
			CHECK(r.state(c) == tiBusy);
			CHECK(r.nState(c) == tiBusy);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Start_DMA programs the stream (CCRx address, buffer, Length), enables CCxDE, CCxE and the counter (channels 1-4)") {
	for (unsigned c = 1; c <= 4; ++c) {
		CAPTURE(c);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_IC_Start_DMA(&r.h, tiCh[c - 1], r.buf, 6) == HAL_OK);
		DMA_Stream_TypeDef *s = tiStream(tiStreamOf[c - 1]);
		CHECK(s->PAR == r.ccrAddr(c));
		CHECK(s->M0AR == r.addr());
		CHECK(s->NDTR == 6u);
		CHECK((s->CR & 1u) == 1u);
		CHECK((s->CR & 0x1Cu) == 0x1Cu); // TCIE, HTIE, TEIE
		CHECK((r.t->DIER & tiReqAll) == tiCcde(c));
		CHECK((r.t->CCER & tiCcxe(c)) == tiCcxe(c));
		CHECK(r.cen() == true);
		CHECK(r.state(c) == tiBusy);
		CHECK(r.nState(c) == tiBusy);
		for (unsigned o = 1; o <= 4; ++o)
			if (o != c) CHECK(r.streamOn(o) == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Start_DMA with Length 1 and Length 65535 programs NDTR with that many items") {
	const uint16_t lens[2] = {1, 65535};
	for (uint16_t len : lens) {
		CAPTURE(len);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_IC_Start_DMA(&r.h, TIM_CHANNEL_1, r.buf, len) == HAL_OK);
		CHECK(tiStream(tiStreamOf[0])->NDTR == len);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Start_DMA with a NULL buffer or Length 0 returns HAL_ERROR and starts nothing") {
	for (int n = 0; n < 2; ++n) {
		CAPTURE(n);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		const HAL_StatusTypeDef st = n == 0 ? HAL_TIM_IC_Start_DMA(&r.h, TIM_CHANNEL_1, nullptr, 4) : HAL_TIM_IC_Start_DMA(&r.h, TIM_CHANNEL_1, r.buf, 0);
		CHECK(st == HAL_ERROR);
		CHECK(r.state(1) == tiReady);
		CHECK(r.nState(1) == tiReady);
		CHECK(r.t->CCER == 0u);
		CHECK(r.cen() == false);
		CHECK(r.streamOn(1) == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 Start of a channel whose state is BUSY returns HAL_ERROR (poll, IT) and HAL_BUSY (DMA) and changes nothing") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		REQUIRE(tiIcStart(r, mode, TIM_CHANNEL_2) == HAL_OK);
		const uint32_t ccer = r.t->CCER, dier = r.t->DIER;
		CHECK(tiIcStart(r, mode, TIM_CHANNEL_2) == (mode == 2 ? HAL_BUSY : HAL_ERROR));
		CHECK(r.t->CCER == ccer);
		CHECK(r.t->DIER == dier);
		CHECK(r.state(2) == tiBusy);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 Start with the complementary channel state BUSY returns HAL_ERROR (poll, IT) / HAL_BUSY (DMA) and leaves the channel state READY") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		r.h.ChannelNState[0] = HAL_TIM_CHANNEL_STATE_BUSY;
		CHECK(tiIcStart(r, mode, TIM_CHANNEL_1) == (mode == 2 ? HAL_BUSY : HAL_ERROR));
		CHECK(r.state(1) == tiReady);
		CHECK(r.t->CCER == 0u);
		CHECK(r.cen() == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 Start of a handle that was never initialised (channel states RESET) returns HAL_ERROR in every mode") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiRig r(3, true);
		CHECK(tiIcStart(r, mode, TIM_CHANNEL_1) == HAL_ERROR);
		CHECK(r.t->CCER == 0u);
		CHECK(r.cen() == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 two channels can be started one after the other and keep their own interrupt / request bits") {
	for (int mode = 1; mode < 3; ++mode) {
		CAPTURE(mode);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		REQUIRE(tiIcStart(r, mode, TIM_CHANNEL_1) == HAL_OK);
		REQUIRE(tiIcStart(r, mode, TIM_CHANNEL_3) == HAL_OK);
		CHECK((r.t->DIER & tiReqAll) == (tiReq(mode, 1) | tiReq(mode, 3)));
		CHECK(r.t->CCER == (tiCcxe(1) | tiCcxe(3)));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 Start with SMS = trigger mode enables the channel but not the counter (the trigger starts it, RM0090 18.3.15)") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		r.t->SMCR = 6u;
		REQUIRE(tiIcStart(r, mode, TIM_CHANNEL_1) == HAL_OK);
		CHECK((r.t->CCER & 1u) == 1u);
		CHECK(r.cen() == false);
		CHECK(r.state(1) == tiBusy);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 Start with another slave mode (reset 4, gated 5, external clock 7, disabled 0) still enables the counter") {
	const uint32_t modes[4] = {0, 4, 5, 7};
	for (uint32_t sms : modes)
		for (int mode = 0; mode < 3; ++mode) {
			CAPTURE(sms);
			CAPTURE(mode);
			TiRig r(3, true);
			REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
			r.t->SMCR = sms;
			REQUIRE(tiIcStart(r, mode, TIM_CHANNEL_1) == HAL_OK);
			CHECK(r.cen() == true);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 Start of TIM10 (not a slave timer, no SMCR) enables the counter without touching a missing register") {
	TiRig r(10);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_IC_Start(&r.h, TIM_CHANNEL_1) == HAL_OK);
	CHECK(r.cen() == true);
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "TI1 Stop clears CCxE, the request bits of the channel and the counter, and sets both channel states READY (all modes, channels 1-4)") {
	for (int mode = 0; mode < 3; ++mode)
		for (unsigned c = 1; c <= 4; ++c) {
			CAPTURE(mode);
			CAPTURE(c);
			TiRig r(3, true);
			REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
			REQUIRE(tiIcStart(r, mode, tiCh[c - 1]) == HAL_OK);
			REQUIRE(tiIcStop(r, mode, tiCh[c - 1]) == HAL_OK);
			CHECK(r.t->CCER == 0u);
			CHECK((r.t->DIER & tiReqAll) == 0u);
			CHECK(r.cen() == false);
			CHECK(r.state(c) == tiReady);
			CHECK(r.nState(c) == tiReady);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 Stop_DMA stops the stream of the channel") {
	for (unsigned c = 1; c <= 4; ++c) {
		CAPTURE(c);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_IC_Start_DMA(&r.h, tiCh[c - 1], r.buf, 4) == HAL_OK);
		REQUIRE(r.streamOn(c) == true);
		REQUIRE(HAL_TIM_IC_Stop_DMA(&r.h, tiCh[c - 1]) == HAL_OK);
		CHECK(r.streamOn(c) == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 Stop of one channel keeps CCxE, the request bit and the state of another started channel") {
	for (int mode = 1; mode < 3; ++mode) {
		CAPTURE(mode);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		REQUIRE(tiIcStart(r, mode, TIM_CHANNEL_1) == HAL_OK);
		REQUIRE(tiIcStart(r, mode, TIM_CHANNEL_2) == HAL_OK);
		REQUIRE(tiIcStop(r, mode, TIM_CHANNEL_1) == HAL_OK);
		CHECK(r.t->CCER == tiCcxe(2));
		CHECK((r.t->DIER & tiReqAll) == tiReq(mode, 2));
		CHECK(r.state(2) == tiBusy);
		CHECK(r.state(1) == tiReady);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI1 Stop of one channel keeps the counter running while another channel is enabled") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		REQUIRE(tiIcStart(r, mode, TIM_CHANNEL_1) == HAL_OK);
		REQUIRE(tiIcStart(r, mode, TIM_CHANNEL_2) == HAL_OK);
		REQUIRE(tiIcStop(r, mode, TIM_CHANNEL_1) == HAL_OK);
		CHECK(r.cen() == true); // NEEDS REVIEW 333/c
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 Stop of a channel that was never started returns HAL_OK and leaves the states READY") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		CHECK(tiIcStop(r, mode, TIM_CHANNEL_3) == HAL_OK);
		CHECK(r.state(3) == tiReady);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 a stopped channel can be started again") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		REQUIRE(tiIcStart(r, mode, TIM_CHANNEL_4) == HAL_OK);
		REQUIRE(tiIcStop(r, mode, TIM_CHANNEL_4) == HAL_OK);
		if (mode == 2) tiFinishAbort(r, 3); // the abort of the stream ends in its transfer complete interrupt
		CHECK(tiIcStart(r, mode, TIM_CHANNEL_4) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 Stop clears CCxE and the interrupt bit on every timer") {
	for (unsigned tim : tiIcTimers)
		for (int mode = 0; mode < 2; ++mode) {
			CAPTURE(tim);
			CAPTURE(mode);
			TiRig r(tim);
			REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
			REQUIRE(tiIcStart(r, mode, TIM_CHANNEL_1) == HAL_OK);
			REQUIRE(tiIcStop(r, mode, TIM_CHANNEL_1) == HAL_OK);
			CHECK(r.t->CCER == 0u);
			CHECK((r.t->DIER & tiReqAll) == 0u);
			CHECK(r.cen() == false);
		}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI1 poll Start and Stop with TIM_CHANNEL_ALL return HAL_OK and use the state of channel 4") {
	TiRig r(3);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	CHECK(HAL_TIM_IC_Start(&r.h, TIM_CHANNEL_ALL) == HAL_OK); // NEEDS REVIEW 333/a
	CHECK(r.state(4) == tiBusy);
	CHECK(HAL_TIM_IC_Stop(&r.h, TIM_CHANNEL_ALL) == HAL_OK);
	CHECK(r.state(4) == tiReady);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI1 Start_IT with TIM_CHANNEL_ALL returns HAL_ERROR and leaves the state of channel 4 BUSY, Stop_IT returns HAL_ERROR") {
	TiRig r(3);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	CHECK(HAL_TIM_IC_Start_IT(&r.h, TIM_CHANNEL_ALL) == HAL_ERROR); // NEEDS REVIEW 333/a
	CHECK(r.state(4) == tiBusy);
	CHECK(r.cen() == false);
	CHECK(HAL_TIM_IC_Stop_IT(&r.h, TIM_CHANNEL_ALL) == HAL_ERROR);
	CHECK(r.state(4) == tiBusy);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI1 Start_DMA with TIM_CHANNEL_ALL returns HAL_ERROR, leaves channel 4 BUSY and the counter enabled, Stop_DMA returns HAL_ERROR") {
	TiRig r(3, true);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	CHECK(HAL_TIM_IC_Start_DMA(&r.h, TIM_CHANNEL_ALL, r.buf, 4) == HAL_ERROR); // NEEDS REVIEW 333/a
	CHECK(r.state(4) == tiBusy);
	CHECK(r.cen() == true);
	CHECK(HAL_TIM_IC_Stop_DMA(&r.h, TIM_CHANNEL_ALL) == HAL_ERROR);
}

TEST_CASE_FIXTURE(TimFixture, "TI1 Start_DMA that fails because the DMA stream cannot start leaves the channel state READY" * KNOWN_ISSUE("candidate vendor bug V112 (arguable: UM1725 gives no state rule for a failed DMA start): the channel and complementary state stay BUSY after the HAL_ERROR return, so every later start of that channel returns HAL_BUSY (same pattern as V101 of the SPI driver)")) {
	TiRig r(3, true);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	REQUIRE(HAL_DMA_Start_IT(&r.dma[0], r.ccrAddr(1), r.addr(), 2) == HAL_OK); // the stream is now BUSY
	CHECK(HAL_TIM_IC_Start_DMA(&r.h, TIM_CHANNEL_1, r.buf, 4) == HAL_ERROR);
	CHECK(r.state(1) == tiReady);
	CHECK(r.nState(1) == tiReady);
}

TEST_CASE_FIXTURE(TimFixture, "TI1 the DMA completion of a normal mode capture stream calls IC_CaptureCallback and sets the channel READY") {
	TiRig r(3, true);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_IC_Start_DMA(&r.h, TIM_CHANNEL_2, r.buf, 4) == HAL_OK);
	hostDmaComplete(0, tiStreamOf[1]);
	HAL_DMA_IRQHandler(&r.dma[1]);
	CHECK(timCounts.icCapture == 1u);
	CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_2);
	CHECK(r.state(2) == tiReady);
}

/* ---- ReadCapturedValue, capture */

TEST_CASE_FIXTURE(TimFixture, "TI1 ReadCapturedValue returns CCRx of the four channels after a capture") {
	for (unsigned c = 1; c <= 4; ++c) {
		CAPTURE(c);
		TiRig r(3);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		const TIM_IC_InitTypeDef cfg = tiIc(TIM_ICPOLARITY_RISING, TIM_ICSELECTION_DIRECTTI, TIM_ICPSC_DIV1, 0);
		for (unsigned k = 1; k <= 4; ++k) REQUIRE(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, tiCh[k - 1]) == HAL_OK);
		REQUIRE(HAL_TIM_IC_Start(&r.h, tiCh[c - 1]) == HAL_OK);
		REQUIRE(hostTimCapture(3, c, 0x1000u + c) == 1);
		CHECK(HAL_TIM_ReadCapturedValue(&r.h, tiCh[c - 1]) == 0x1000u + c);
		CHECK((r.t->SR & (1u << c)) == (1u << c)); // CCxIF
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 ReadCapturedValue reads all 32 bits on the 32 bit timers TIM2 and TIM5") {
	const unsigned tims[2] = {2, 5};
	for (unsigned tim : tims)
		for (unsigned c = 1; c <= 4; ++c) {
			CAPTURE(tim);
			CAPTURE(c);
			TiRig r(tim);
			REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
			const TIM_IC_InitTypeDef cfg = tiIc(TIM_ICPOLARITY_RISING, TIM_ICSELECTION_DIRECTTI, TIM_ICPSC_DIV1, 0);
			REQUIRE(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, tiCh[c - 1]) == HAL_OK);
			REQUIRE(hostTimCapture(tim, c, 0x89ABCD00u + c) == 1);
			CHECK(HAL_TIM_ReadCapturedValue(&r.h, tiCh[c - 1]) == 0x89ABCD00u + c);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 ReadCapturedValue of channel 1 and 2 of TIM9 / TIM12 and channel 1 of TIM10 / 11 / 13 / 14") {
	const unsigned tims[6] = {9, 12, 10, 11, 13, 14};
	for (unsigned tim : tims)
		for (unsigned c = 1; c <= tiChannelsOf(tim); ++c) {
			CAPTURE(tim);
			CAPTURE(c);
			TiRig r(tim);
			REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
			const TIM_IC_InitTypeDef cfg = tiIc(TIM_ICPOLARITY_RISING, TIM_ICSELECTION_DIRECTTI, TIM_ICPSC_DIV1, 0);
			REQUIRE(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, tiCh[c - 1]) == HAL_OK);
			REQUIRE(hostTimCapture(tim, c, 0x0200u + c) == 1);
			CHECK(HAL_TIM_ReadCapturedValue(&r.h, tiCh[c - 1]) == 0x0200u + c);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 a channel configured by IC_ConfigChannel is an input: a capture event is accepted, an unconfigured channel refuses it") {
	TiRig r(3);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	CHECK(hostTimCapture(3, 1, 5) == 0);
	const TIM_IC_InitTypeDef cfg = tiIc(TIM_ICPOLARITY_RISING, TIM_ICSELECTION_DIRECTTI, TIM_ICPSC_DIV1, 0);
	REQUIRE(HAL_TIM_IC_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_1) == HAL_OK);
	CHECK(hostTimCapture(3, 1, 5) == 1);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI1 ReadCapturedValue of TIM_CHANNEL_ALL returns 0") {
	TiRig r(3);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	r.t->CCR1 = 0x11;
	CHECK(HAL_TIM_ReadCapturedValue(&r.h, TIM_CHANNEL_ALL) == 0u); // NEEDS REVIEW 333/d
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_GetState is RESET before IC_Init and READY after") {
	TiRig r(3);
	CHECK(HAL_TIM_IC_GetState(&r.h) == HAL_TIM_STATE_RESET);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	CHECK(HAL_TIM_IC_GetState(&r.h) == HAL_TIM_STATE_READY);
}

/* ================================================================ TI2: one pulse */

namespace {
TIM_OnePulse_InitTypeDef tiOp() {
	TIM_OnePulse_InitTypeDef c{};
	c.OCMode = TIM_OCMODE_PWM2;
	c.Pulse = 300;
	c.OCPolarity = TIM_OCPOLARITY_LOW;
	c.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	c.OCIdleState = TIM_OCIDLESTATE_RESET;
	c.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	c.ICPolarity = TIM_ICPOLARITY_FALLING;
	c.ICSelection = TIM_ICSELECTION_DIRECTTI;
	c.ICFilter = 6;
	return c;
}
constexpr unsigned tiOpTimers[6] = {1, 3, 2, 8, 9, 12};
} // namespace

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_Init of a NULL handle returns HAL_ERROR") { CHECK(HAL_TIM_OnePulse_Init(nullptr, TIM_OPMODE_SINGLE) == HAL_ERROR); }

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_Init of a RESET handle runs the OnePulse MspInit once, unlocks and sets state READY and channel 1 / 2 states READY") {
	for (unsigned tim : tiOpTimers) {
		CAPTURE(tim);
		TiRig r(tim);
		r.h.Lock = HAL_LOCKED;
		REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
		CHECK(timCounts.mspInit[4] == 1u);
		CHECK(r.h.Lock == HAL_UNLOCKED);
		CHECK(HAL_TIM_OnePulse_GetState(&r.h) == HAL_TIM_STATE_READY);
		for (unsigned c = 1; c <= 2; ++c) {
			CHECK(r.state(c) == tiReady);
			CHECK(r.nState(c) == tiReady);
		}
		CHECK(r.h.DMABurstState == HAL_DMA_BURST_STATE_READY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_Init with TIM_OPMODE_SINGLE sets OPM and with REPETITIVE clears it (RM0090 18.4.1 OPM bit 3)") {
	for (unsigned tim : tiOpTimers) {
		CAPTURE(tim);
		TiRig r(tim);
		REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
		CHECK((r.t->CR1 & tiCR1_OPM) == tiCR1_OPM);
		REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_REPETITIVE) == HAL_OK);
		CHECK((r.t->CR1 & tiCR1_OPM) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_Init writes Period to ARR, Prescaler to PSC and does not start the counter") {
	TiRig r(3);
	r.h.Init.Period = 0x4321;
	r.h.Init.Prescaler = 0x55;
	REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
	CHECK(r.t->ARR == 0x4321u);
	CHECK(r.t->PSC == 0x55u);
	CHECK(r.cen() == false);
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_DeInit runs MspDeInit once, stops the counter and sets state, channel states and burst state RESET") {
	TiRig r(3);
	REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
	r.t->CR1 |= tiCR1_CEN;
	r.h.Lock = HAL_LOCKED;
	REQUIRE(HAL_TIM_OnePulse_DeInit(&r.h) == HAL_OK);
	CHECK(timCounts.mspDeInit[4] == 1u);
	CHECK(timCounts.stateAtLast == HAL_TIM_STATE_BUSY);
	CHECK(r.cen() == false);
	CHECK(HAL_TIM_OnePulse_GetState(&r.h) == HAL_TIM_STATE_RESET);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	for (unsigned c = 1; c <= 2; ++c) {
		CHECK(r.state(c) == HAL_TIM_CHANNEL_STATE_RESET);
		CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_RESET);
	}
	CHECK(r.h.DMABurstState == HAL_DMA_BURST_STATE_RESET);
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_ConfigChannel output 1 / input 2 (TI2FP2 trigger): CCR1, OC1M, polarities, CC2S, IC2F, TS = 110 and SMS = 110 (RM0090 17.3.15)") {
	const unsigned tims[3] = {3, 9, 1};
	for (unsigned tim : tims) {
		CAPTURE(tim);
		TiRig r(tim);
		REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
		TIM_OnePulse_InitTypeDef cfg = tiOp();
		REQUIRE(HAL_TIM_OnePulse_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_1, TIM_CHANNEL_2) == HAL_OK);
		CHECK(r.t->CCMR1 == 0x6170u); // OC1M = 111 (PWM2) 0x70, CC2S = 01 0x100, IC2F = 6 0x6000
		CHECK(r.t->CCR1 == 300u);
		CHECK((r.t->CCER & 0xFFu) == 0x22u); // CC1P (OC1 low), CC2P (falling edge)
		CHECK(r.t->SMCR == 0x66u);           // TS = 110 (bits 6:4), SMS = 110
		CHECK(r.t->CR1 == (HalFixture::peek(&r.t->CR1) | 0u));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_ConfigChannel output 2 / input 1 (TI1FP1 trigger): CCR2, OC2M, polarities, CC1S, IC1F, TS = 101 and SMS = 110") {
	const unsigned tims[3] = {3, 9, 8};
	for (unsigned tim : tims) {
		CAPTURE(tim);
		TiRig r(tim);
		REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
		TIM_OnePulse_InitTypeDef cfg = tiOp();
		REQUIRE(HAL_TIM_OnePulse_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_2, TIM_CHANNEL_1) == HAL_OK);
		CHECK(r.t->CCMR1 == 0x7061u); // OC2M = 111 0x7000, CC1S = 01, IC1F = 6 0x60
		CHECK(r.t->CCR2 == 300u);
		CHECK((r.t->CCER & 0xFFu) == 0x22u); // CC2P (OC2 low) 0x20, CC1P (falling) 0x02
		CHECK(r.t->SMCR == 0x56u);           // TS = 101, SMS = 110
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_ConfigChannel keeps the other SMCR bits (MSM) and replaces an old TS / SMS") {
	TiRig r(3);
	REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
	r.t->SMCR = 0x80u | 0x70u | 0x7u; // MSM, TS = 111, SMS = 111
	TIM_OnePulse_InitTypeDef cfg = tiOp();
	REQUIRE(HAL_TIM_OnePulse_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_1, TIM_CHANNEL_2) == HAL_OK);
	CHECK(r.t->SMCR == 0xE6u);
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_ConfigChannel of the input channel honours the selection (indirect, TRC) and the polarity (rising, both)") {
	const uint32_t sels[2] = {TIM_ICSELECTION_INDIRECTTI, TIM_ICSELECTION_TRC};
	const uint32_t want[2] = {2u, 3u};
	for (int s = 0; s < 2; ++s) {
		CAPTURE(s);
		TiRig r(3);
		REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
		TIM_OnePulse_InitTypeDef cfg = tiOp();
		cfg.ICSelection = sels[s];
		cfg.ICPolarity = TIM_ICPOLARITY_BOTHEDGE;
		REQUIRE(HAL_TIM_OnePulse_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_1, TIM_CHANNEL_2) == HAL_OK);
		CHECK(((r.t->CCMR1 >> 8) & 3u) == want[s]);
		CHECK((r.t->CCER & 0xA0u) == 0xA0u); // CC2P | CC2NP
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_ConfigChannel on TIM1 writes the idle state (OIS1) and the complementary polarity and idle state") {
	TiRig r(1);
	REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
	TIM_OnePulse_InitTypeDef cfg = tiOp();
	cfg.OCNPolarity = TIM_OCNPOLARITY_LOW;
	cfg.OCIdleState = TIM_OCIDLESTATE_SET;
	cfg.OCNIdleState = TIM_OCNIDLESTATE_SET;
	REQUIRE(HAL_TIM_OnePulse_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_1, TIM_CHANNEL_2) == HAL_OK);
	CHECK((r.t->CR2 & 0x300u) == 0x300u);    // OIS1 bit 8, OIS1N bit 9
	CHECK((r.t->CCER & 0x8u) == 0x8u);       // CC1NP
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_ConfigChannel with equal output and input channel returns HAL_ERROR and writes nothing") {
	TiRig r(3);
	REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
	TIM_OnePulse_InitTypeDef cfg = tiOp();
	for (unsigned c = 0; c < 2; ++c) {
		CHECK(HAL_TIM_OnePulse_ConfigChannel(&r.h, &cfg, tiCh[c], tiCh[c]) == HAL_ERROR);
		CHECK(r.t->CCMR1 == 0u);
		CHECK(r.t->CCR1 == 0u);
		CHECK(r.t->SMCR == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_ConfigChannel returns the state READY and the handle unlocked after a successful call") {
	TiRig r(3);
	REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
	TIM_OnePulse_InitTypeDef cfg = tiOp();
	REQUIRE(HAL_TIM_OnePulse_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_1, TIM_CHANNEL_2) == HAL_OK);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	CHECK(HAL_TIM_OnePulse_GetState(&r.h) == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_ConfigChannel of a locked handle returns HAL_BUSY and writes nothing") {
	TiRig r(3);
	REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
	r.h.Lock = HAL_LOCKED;
	TIM_OnePulse_InitTypeDef cfg = tiOp();
	CHECK(HAL_TIM_OnePulse_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_1, TIM_CHANNEL_2) == HAL_BUSY);
	CHECK(r.t->CCMR1 == 0u);
	CHECK(r.t->SMCR == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI2 OnePulse_ConfigChannel with an invalid output channel returns HAL_ERROR, writes nothing and unlocks") {
	TiRig r(3);
	REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
	TIM_OnePulse_InitTypeDef cfg = tiOp();
	CHECK(HAL_TIM_OnePulse_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_3, TIM_CHANNEL_2) == HAL_ERROR); // NEEDS REVIEW 333/e
	CHECK(r.t->CCMR1 == 0u);
	CHECK(r.t->CCMR2 == 0u);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	CHECK(HAL_TIM_OnePulse_GetState(&r.h) == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI2 OnePulse_ConfigChannel with an invalid input channel returns HAL_ERROR after the output channel was configured") {
	TiRig r(3);
	REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
	TIM_OnePulse_InitTypeDef cfg = tiOp();
	CHECK(HAL_TIM_OnePulse_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_1, TIM_CHANNEL_3) == HAL_ERROR); // NEEDS REVIEW 333/e
	CHECK(r.t->CCR1 == 300u);
	CHECK(r.t->SMCR == 0u);
	CHECK(r.h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI2 OnePulse_ConfigChannel clears the prescaler bits of the input channel") {
	TiRig r(3);
	REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
	r.t->CCMR1 = 0x0C00u; // IC2PSC = 11
	TIM_OnePulse_InitTypeDef cfg = tiOp();
	REQUIRE(HAL_TIM_OnePulse_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_1, TIM_CHANNEL_2) == HAL_OK);
	CHECK((r.t->CCMR1 & 0x0C00u) == 0u); // NEEDS REVIEW 333/e
}

namespace {
HAL_StatusTypeDef tiOpStart(TiRig &r, int it, uint32_t ch) { return it ? HAL_TIM_OnePulse_Start_IT(&r.h, ch) : HAL_TIM_OnePulse_Start(&r.h, ch); }
HAL_StatusTypeDef tiOpStop(TiRig &r, int it, uint32_t ch) { return it ? HAL_TIM_OnePulse_Stop_IT(&r.h, ch) : HAL_TIM_OnePulse_Stop(&r.h, ch); }
} // namespace

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_Start enables CC1E and CC2E, sets the channel 1 / 2 states BUSY and does not set CEN (the trigger starts the counter)") {
	for (unsigned tim : tiOpTimers)
		for (int it = 0; it < 2; ++it) {
			CAPTURE(tim);
			CAPTURE(it);
			TiRig r(tim);
			REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
			REQUIRE(tiOpStart(r, it, TIM_CHANNEL_1) == HAL_OK);
			CHECK((r.t->CCER & 0x11u) == 0x11u);
			CHECK(r.cen() == false);
			for (unsigned c = 1; c <= 2; ++c) {
				CHECK(r.state(c) == tiBusy);
				CHECK(r.nState(c) == tiBusy);
			}
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_Start_IT enables the CC1 and CC2 interrupts, OnePulse_Start does not") {
	for (int it = 0; it < 2; ++it) {
		CAPTURE(it);
		TiRig r(3);
		REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
		REQUIRE(tiOpStart(r, it, TIM_CHANNEL_1) == HAL_OK);
		CHECK((r.t->DIER & 0x6u) == (it ? 0x6u : 0u));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_Start on TIM1 and TIM8 enables MOE, on the other timers it does not touch BDTR") {
	for (unsigned tim : tiOpTimers)
		for (int it = 0; it < 2; ++it) {
			CAPTURE(tim);
			CAPTURE(it);
			TiRig r(tim);
			REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
			REQUIRE(tiOpStart(r, it, TIM_CHANNEL_1) == HAL_OK);
			if (tim == 1 || tim == 8) CHECK(r.moe() == true);
			CHECK(HalFixture::faults() == 0u); // a timer without BDTR is not accessed
		}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI2 OnePulse_Start ignores OutputChannel (channels 1 and 2 are always used)") {
	const uint32_t chans[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
	for (uint32_t ch : chans) {
		CAPTURE(ch);
		TiRig r(3);
		REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
		CHECK(HAL_TIM_OnePulse_Start(&r.h, ch) == HAL_OK); // NEEDS REVIEW 333/e
		CHECK((r.t->CCER & 0x11u) == 0x11u);
		CHECK((r.t->CCER & 0x100u) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_Start returns HAL_ERROR and changes nothing when any of the four channel states is not READY") {
	for (int which = 0; which < 4; ++which)
		for (int it = 0; it < 2; ++it) {
			CAPTURE(which);
			CAPTURE(it);
			TiRig r(3);
			REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
			(which < 2 ? r.h.ChannelState[which] : r.h.ChannelNState[which - 2]) = HAL_TIM_CHANNEL_STATE_BUSY;
			CHECK(tiOpStart(r, it, TIM_CHANNEL_1) == HAL_ERROR);
			CHECK(r.t->CCER == 0u);
			CHECK(r.t->DIER == 0u);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_Start of a handle that was not initialised returns HAL_ERROR") {
	TiRig r(3);
	CHECK(HAL_TIM_OnePulse_Start(&r.h, TIM_CHANNEL_1) == HAL_ERROR);
	CHECK(HAL_TIM_OnePulse_Start_IT(&r.h, TIM_CHANNEL_1) == HAL_ERROR);
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_Stop clears CC1E, CC2E and the counter (set by the trigger) and sets the states READY (poll and IT)") {
	for (unsigned tim : tiOpTimers)
		for (int it = 0; it < 2; ++it) {
			CAPTURE(tim);
			CAPTURE(it);
			TiRig r(tim);
			REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
			REQUIRE(tiOpStart(r, it, TIM_CHANNEL_1) == HAL_OK);
			r.t->CR1 |= tiCR1_CEN; // the hardware sets CEN at the trigger
			REQUIRE(tiOpStop(r, it, TIM_CHANNEL_1) == HAL_OK);
			CHECK((r.t->CCER & 0x11u) == 0u);
			CHECK(r.cen() == false);
			CHECK((r.t->DIER & 0x6u) == 0u);
			if (tim == 1 || tim == 8) CHECK(r.moe() == false);
			for (unsigned c = 1; c <= 2; ++c) {
				CHECK(r.state(c) == tiReady);
				CHECK(r.nState(c) == tiReady);
			}
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_Stop_IT keeps the other interrupt enables") {
	TiRig r(3);
	REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
	r.t->DIER = 0x1u | 0x8u; // UIE, CC3IE
	REQUIRE(HAL_TIM_OnePulse_Start_IT(&r.h, TIM_CHANNEL_1) == HAL_OK);
	REQUIRE(HAL_TIM_OnePulse_Stop_IT(&r.h, TIM_CHANNEL_1) == HAL_OK);
	CHECK(r.t->DIER == 0x9u);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI2 OnePulse_Stop keeps MOE and CEN while channel 3 is still enabled (TIM1)") {
	TiRig r(1);
	REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
	REQUIRE(HAL_TIM_OnePulse_Start(&r.h, TIM_CHANNEL_1) == HAL_OK);
	r.t->CR1 |= tiCR1_CEN;
	r.t->CCER |= tiCcxe(3);
	REQUIRE(HAL_TIM_OnePulse_Stop(&r.h, TIM_CHANNEL_1) == HAL_OK);
	CHECK(r.moe() == true);
	CHECK(r.cen() == true);
}

TEST_CASE_FIXTURE(TimFixture, "TI2 a stopped one pulse can be started again") {
	for (int it = 0; it < 2; ++it) {
		TiRig r(3);
		REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
		REQUIRE(tiOpStart(r, it, TIM_CHANNEL_1) == HAL_OK);
		REQUIRE(tiOpStop(r, it, TIM_CHANNEL_1) == HAL_OK);
		CHECK(tiOpStart(r, it, TIM_CHANNEL_2) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI2 a one pulse timer in single mode stops at the update event after the trigger started it (RM0090 17.3.15)") {
	TiRig r(3);
	r.h.Init.Period = 20;
	REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
	REQUIRE(HAL_TIM_OnePulse_Start(&r.h, TIM_CHANNEL_1) == HAL_OK);
	CHECK(r.cen() == false);
	r.t->CR1 |= tiCR1_CEN; // trigger
	hostTimAdvance(3, 21);  // 0 .. ARR then the update event
	CHECK(r.cen() == false);
	CHECK((r.t->SR & 1u) == 1u);
}

TEST_CASE_FIXTURE(TimFixture, "TI2 OnePulse_GetState is RESET before and READY after OnePulse_Init") {
	TiRig r(3);
	CHECK(HAL_TIM_OnePulse_GetState(&r.h) == HAL_TIM_STATE_RESET);
	REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_REPETITIVE) == HAL_OK);
	CHECK(HAL_TIM_OnePulse_GetState(&r.h) == HAL_TIM_STATE_READY);
}

/* ================================================================ TI3: encoder */

namespace {
TIM_Encoder_InitTypeDef tiEnc(uint32_t mode) {
	TIM_Encoder_InitTypeDef c{};
	c.EncoderMode = mode;
	c.IC1Polarity = TIM_ENCODERINPUTPOLARITY_FALLING;
	c.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	c.IC1Prescaler = TIM_ICPSC_DIV2;
	c.IC1Filter = 5;
	c.IC2Polarity = TIM_ENCODERINPUTPOLARITY_RISING;
	c.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	c.IC2Prescaler = TIM_ICPSC_DIV4;
	c.IC2Filter = 9;
	return c;
}
/* Encoder family: mode 0 poll, 1 IT, 2 DMA (pData1 = buf, pData2 = buf2). */
HAL_StatusTypeDef tiEncStart(TiRig &r, int mode, uint32_t ch, uint16_t len = 4) {
	return mode == 0 ? HAL_TIM_Encoder_Start(&r.h, ch) : mode == 1 ? HAL_TIM_Encoder_Start_IT(&r.h, ch) : HAL_TIM_Encoder_Start_DMA(&r.h, ch, r.buf, r.buf2, len);
}
HAL_StatusTypeDef tiEncStop(TiRig &r, int mode, uint32_t ch) {
	return mode == 0 ? HAL_TIM_Encoder_Stop(&r.h, ch) : mode == 1 ? HAL_TIM_Encoder_Stop_IT(&r.h, ch) : HAL_TIM_Encoder_Stop_DMA(&r.h, ch);
}
} // namespace

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder_Init of a NULL handle returns HAL_ERROR") {
	const TIM_Encoder_InitTypeDef c = tiEnc(TIM_ENCODERMODE_TI12);
	CHECK(HAL_TIM_Encoder_Init(nullptr, &c) == HAL_ERROR);
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder_Init of a RESET handle runs the Encoder MspInit once, unlocks and sets state and channel 1 / 2 states READY") {
	for (unsigned tim : tiEncTimers) {
		CAPTURE(tim);
		TiRig r(tim);
		r.h.Lock = HAL_LOCKED;
		const TIM_Encoder_InitTypeDef c = tiEnc(TIM_ENCODERMODE_TI12);
		REQUIRE(HAL_TIM_Encoder_Init(&r.h, &c) == HAL_OK);
		CHECK(timCounts.mspInit[5] == 1u);
		CHECK(r.h.Lock == HAL_UNLOCKED);
		CHECK(HAL_TIM_Encoder_GetState(&r.h) == HAL_TIM_STATE_READY);
		for (unsigned k = 1; k <= 2; ++k) {
			CHECK(r.state(k) == tiReady);
			CHECK(r.nState(k) == tiReady);
		}
		CHECK(r.h.DMABurstState == HAL_DMA_BURST_STATE_READY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder_Init of an initialised handle does not run MspInit again") {
	TiRig r(3);
	const TIM_Encoder_InitTypeDef c = tiEnc(TIM_ENCODERMODE_TI12);
	REQUIRE(HAL_TIM_Encoder_Init(&r.h, &c) == HAL_OK);
	REQUIRE(HAL_TIM_Encoder_Init(&r.h, &c) == HAL_OK);
	CHECK(timCounts.mspInit[5] == 1u);
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder_Init writes SMS = 001 / 010 / 011 for encoder mode TI1 / TI2 / TI12 (RM0090 18.4.3)") {
	const uint32_t modes[3] = {TIM_ENCODERMODE_TI1, TIM_ENCODERMODE_TI2, TIM_ENCODERMODE_TI12};
	for (unsigned tim : tiEncTimers) {
		if (tim == 12) continue; // RM0090 chapter 19 has no encoder modes, see the TIM12 case
		for (unsigned m = 0; m < 3; ++m) {
			CAPTURE(tim);
			CAPTURE(m);
			TiRig r(tim);
			const TIM_Encoder_InitTypeDef c = tiEnc(modes[m]);
			REQUIRE(HAL_TIM_Encoder_Init(&r.h, &c) == HAL_OK);
			CHECK((r.t->SMCR & 7u) == m + 1u);
		}
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder_Init replaces an old SMS, clears ECE and keeps the other SMCR bits") {
	TiRig r(3);
	r.t->SMCR = 0x4000u | 0x80u | 0x7u; // ECE, MSM, SMS = 111
	const TIM_Encoder_InitTypeDef c = tiEnc(TIM_ENCODERMODE_TI2);
	REQUIRE(HAL_TIM_Encoder_Init(&r.h, &c) == HAL_OK);
	CHECK(r.t->SMCR == (0x80u | 2u));
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder_Init writes selection, prescaler and filter of both inputs into CCMR1") {
	TiRig r(3);
	r.t->CCMR1 = 0xFFFFu; // old values must be replaced
	const TIM_Encoder_InitTypeDef c = tiEnc(TIM_ENCODERMODE_TI12);
	REQUIRE(HAL_TIM_Encoder_Init(&r.h, &c) == HAL_OK);
	// IC1: CC1S 01, IC1PSC 01 (every 2), IC1F 5 -> 0x55; IC2: CC2S 01, IC2PSC 10 (every 4), IC2F 9 -> 0x99
	CHECK(r.t->CCMR1 == 0x9955u);
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder_Init writes the selection direct / indirect / TRC of input 1 and input 2") {
	const uint32_t sels[3] = {TIM_ICSELECTION_DIRECTTI, TIM_ICSELECTION_INDIRECTTI, TIM_ICSELECTION_TRC};
	for (unsigned s = 0; s < 3; ++s) {
		CAPTURE(s);
		TiRig r(3);
		TIM_Encoder_InitTypeDef c = tiEnc(TIM_ENCODERMODE_TI12);
		c.IC1Selection = sels[s];
		c.IC2Selection = sels[2 - s];
		c.IC1Prescaler = c.IC2Prescaler = TIM_ICPSC_DIV1;
		c.IC1Filter = c.IC2Filter = 0;
		REQUIRE(HAL_TIM_Encoder_Init(&r.h, &c) == HAL_OK);
		CHECK((r.t->CCMR1 & 3u) == s + 1u);
		CHECK(((r.t->CCMR1 >> 8) & 3u) == 3u - s);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder_Init writes the polarity of input 1 into CC1P and of input 2 into CC2P, and clears CC1NP / CC2NP") {
	const uint32_t pols[2] = {TIM_ENCODERINPUTPOLARITY_RISING, TIM_ENCODERINPUTPOLARITY_FALLING};
	for (unsigned p1 = 0; p1 < 2; ++p1)
		for (unsigned p2 = 0; p2 < 2; ++p2) {
			CAPTURE(p1);
			CAPTURE(p2);
			TiRig r(3);
			r.t->CCER = 0x00AAu | 0x100u; // CCxP, CCxNP of channels 1 / 2 set, CC3E set
			TIM_Encoder_InitTypeDef c = tiEnc(TIM_ENCODERMODE_TI12);
			c.IC1Polarity = pols[p1];
			c.IC2Polarity = pols[p2];
			REQUIRE(HAL_TIM_Encoder_Init(&r.h, &c) == HAL_OK);
			CHECK(r.t->CCER == (0x100u | (p1 ? 0x2u : 0u) | (p2 ? 0x20u : 0u)));
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder_Init writes Period to ARR and Prescaler to PSC and does not start the counter") {
	TiRig r(2);
	r.h.Init.Period = 0xFFFFFFFFu;
	r.h.Init.Prescaler = 3;
	const TIM_Encoder_InitTypeDef c = tiEnc(TIM_ENCODERMODE_TI12);
	REQUIRE(HAL_TIM_Encoder_Init(&r.h, &c) == HAL_OK);
	CHECK(r.t->ARR == 0xFFFFFFFFu);
	CHECK(r.t->PSC == 3u);
	CHECK(r.cen() == false);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI3 Encoder_Init on TIM12 writes SMS = encoder mode and returns HAL_OK although RM0090 lists none for TIM9 / TIM12") {
	TiRig r(12);
	const TIM_Encoder_InitTypeDef c = tiEnc(TIM_ENCODERMODE_TI12);
	CHECK(HAL_TIM_Encoder_Init(&r.h, &c) == HAL_OK); // NEEDS REVIEW 333/f
	CHECK((r.t->SMCR & 7u) == 3u);
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder_DeInit runs MspDeInit once, stops the counter and sets state, channel states and burst state RESET") {
	TiRig r(3);
	const TIM_Encoder_InitTypeDef c = tiEnc(TIM_ENCODERMODE_TI12);
	REQUIRE(HAL_TIM_Encoder_Init(&r.h, &c) == HAL_OK);
	r.t->CR1 |= tiCR1_CEN;
	r.h.Lock = HAL_LOCKED;
	REQUIRE(HAL_TIM_Encoder_DeInit(&r.h) == HAL_OK);
	CHECK(timCounts.mspDeInit[5] == 1u);
	CHECK(timCounts.stateAtLast == HAL_TIM_STATE_BUSY);
	CHECK(r.cen() == false);
	CHECK(HAL_TIM_Encoder_GetState(&r.h) == HAL_TIM_STATE_RESET);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	for (unsigned k = 1; k <= 2; ++k) {
		CHECK(r.state(k) == HAL_TIM_CHANNEL_STATE_RESET);
		CHECK(r.nState(k) == HAL_TIM_CHANNEL_STATE_RESET);
	}
	CHECK(r.h.DMABurstState == HAL_DMA_BURST_STATE_RESET);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI3 Encoder_DeInit leaves CEN set while a channel is still enabled") {
	TiRig r(3);
	const TIM_Encoder_InitTypeDef c = tiEnc(TIM_ENCODERMODE_TI12);
	REQUIRE(HAL_TIM_Encoder_Init(&r.h, &c) == HAL_OK);
	REQUIRE(HAL_TIM_Encoder_Start(&r.h, TIM_CHANNEL_ALL) == HAL_OK);
	REQUIRE(HAL_TIM_Encoder_DeInit(&r.h) == HAL_OK);
	CHECK(r.cen() == true); // NEEDS REVIEW 333/c
}

namespace {
struct TiEncRig : TiRig {
	explicit TiEncRig(unsigned tim, bool dma = false, uint32_t dmaMode = DMA_NORMAL) : TiRig(tim, dma, dmaMode) {
		const TIM_Encoder_InitTypeDef c = tiEnc(TIM_ENCODERMODE_TI12);
		REQUIRE(HAL_TIM_Encoder_Init(&h, &c) == HAL_OK);
	}
};
} // namespace

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start of channel 1 or 2 enables only that CCxE and the counter and sets only that channel and its complementary state BUSY (all modes)") {
	for (int mode = 0; mode < 3; ++mode)
		for (unsigned c = 1; c <= 2; ++c) {
			CAPTURE(mode);
			CAPTURE(c);
			TiEncRig r(3, true);
			const uint32_t ccerBefore = r.t->CCER & ~0x11u;
			REQUIRE(tiEncStart(r, mode, tiCh[c - 1]) == HAL_OK);
			CHECK((r.t->CCER & 0x11u) == tiCcxe(c));
			CHECK((r.t->CCER & ~0x11u) == ccerBefore);
			CHECK(r.cen() == true);
			CHECK(r.state(c) == tiBusy);
			CHECK(r.nState(c) == tiBusy);
			CHECK(r.state(3 - c) == tiReady);
			CHECK(r.nState(3 - c) == tiReady);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start of TIM_CHANNEL_ALL enables CC1E and CC2E and the counter and sets both channels BUSY (all modes)") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiEncRig r(3, true);
		REQUIRE(tiEncStart(r, mode, TIM_CHANNEL_ALL) == HAL_OK);
		CHECK((r.t->CCER & 0x11u) == 0x11u);
		CHECK(r.cen() == true);
		for (unsigned c = 1; c <= 2; ++c) {
			CHECK(r.state(c) == tiBusy);
			CHECK(r.nState(c) == tiBusy);
		}
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI3 Encoder Start with a channel value other than 1 or 2 is treated as both channels") {
	const uint32_t chans[2] = {TIM_CHANNEL_3, TIM_CHANNEL_4};
	for (uint32_t ch : chans)
		for (int mode = 0; mode < 3; ++mode) {
			CAPTURE(ch);
			CAPTURE(mode);
			TiEncRig r(3, true);
			CHECK(tiEncStart(r, mode, ch) == HAL_OK); // NEEDS REVIEW 333/f
			CHECK((r.t->CCER & 0x11u) == 0x11u);
			CHECK(r.state(1) == tiBusy);
			CHECK(r.state(2) == tiBusy);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_IT enables CC1IE / CC2IE of the started channels only, Start does not enable interrupts") {
	for (unsigned c = 1; c <= 3; ++c) {
		CAPTURE(c);
		TiEncRig r(3);
		const uint32_t ch = c == 3 ? TIM_CHANNEL_ALL : tiCh[c - 1];
		REQUIRE(HAL_TIM_Encoder_Start_IT(&r.h, ch) == HAL_OK);
		CHECK((r.t->DIER & tiReqAll) == (c == 3 ? 0x6u : tiCcie(c)));
	}
	TiEncRig p(3);
	REQUIRE(HAL_TIM_Encoder_Start(&p.h, TIM_CHANNEL_ALL) == HAL_OK);
	CHECK((p.t->DIER & tiReqAll) == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA of channel 1 programs stream 4 with CCR1, pData1 and Length, enables CC1DE only") {
	TiEncRig r(3, true);
	REQUIRE(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_1, r.buf, r.buf2, 7) == HAL_OK);
	DMA_Stream_TypeDef *s = tiStream(tiStreamOf[0]);
	CHECK(s->PAR == r.ccrAddr(1));
	CHECK(s->M0AR == r.addr());
	CHECK(s->NDTR == 7u);
	CHECK((s->CR & 1u) == 1u);
	CHECK(r.streamOn(2) == false);
	CHECK((r.t->DIER & tiReqAll) == tiCcde(1));
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA of channel 2 programs stream 5 with CCR2, pData2 and Length, enables CC2DE only") {
	TiEncRig r(3, true);
	REQUIRE(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_2, r.buf, r.buf2, 5) == HAL_OK);
	DMA_Stream_TypeDef *s = tiStream(tiStreamOf[1]);
	CHECK(s->PAR == r.ccrAddr(2));
	CHECK(s->M0AR == r.addr2());
	CHECK(s->NDTR == 5u);
	CHECK((s->CR & 1u) == 1u);
	CHECK(r.streamOn(1) == false);
	CHECK((r.t->DIER & tiReqAll) == tiCcde(2));
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA of TIM_CHANNEL_ALL programs both streams (pData1 to CCR1, pData2 to CCR2) and enables CC1DE and CC2DE") {
	TiEncRig r(3, true);
	REQUIRE(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_ALL, r.buf, r.buf2, 3) == HAL_OK);
	CHECK(tiStream(tiStreamOf[0])->PAR == r.ccrAddr(1));
	CHECK(tiStream(tiStreamOf[0])->M0AR == r.addr());
	CHECK(tiStream(tiStreamOf[1])->PAR == r.ccrAddr(2));
	CHECK(tiStream(tiStreamOf[1])->M0AR == r.addr2());
	CHECK(tiStream(tiStreamOf[0])->NDTR == 3u);
	CHECK(tiStream(tiStreamOf[1])->NDTR == 3u);
	CHECK(r.streamOn(1) == true);
	CHECK(r.streamOn(2) == true);
	CHECK((r.t->DIER & tiReqAll) == (tiCcde(1) | tiCcde(2)));
	CHECK((r.t->CCER & 0x11u) == 0x11u);
	CHECK(r.cen() == true);
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA with Length 0 returns HAL_ERROR for every channel selection and starts nothing") {
	const uint32_t chans[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_ALL};
	for (uint32_t ch : chans) {
		CAPTURE(ch);
		TiEncRig r(3, true);
		CHECK(HAL_TIM_Encoder_Start_DMA(&r.h, ch, r.buf, r.buf2, 0) == HAL_ERROR);
		CHECK(r.state(1) == tiReady);
		CHECK(r.state(2) == tiReady);
		CHECK(r.cen() == false);
		CHECK(r.streamOn(1) == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA of TIM_CHANNEL_ALL with pData1 or pData2 NULL returns HAL_ERROR and starts nothing") {
	for (int n = 0; n < 2; ++n) {
		CAPTURE(n);
		TiEncRig r(3, true);
		CHECK(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_ALL, n == 0 ? nullptr : r.buf, n == 1 ? nullptr : r.buf2, 4) == HAL_ERROR);
		CHECK(r.state(1) == tiReady);
		CHECK(r.streamOn(1) == false);
		CHECK(r.t->CCER == (r.t->CCER & ~0x11u));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA of channel 1 with pData1 NULL returns HAL_ERROR, of channel 2 with pData2 NULL returns HAL_ERROR") {
	{
		TiEncRig r(3, true);
		CHECK(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_1, nullptr, r.buf2, 4) == HAL_ERROR);
		CHECK(r.state(1) == tiReady);
	}
	{
		TiEncRig r(3, true);
		CHECK(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_2, r.buf, nullptr, 4) == HAL_ERROR);
		CHECK(r.state(2) == tiReady);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI3 Encoder Start_DMA ignores the data pointer of the channel not started") {
	{
		TiEncRig r(3, true);
		CHECK(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_1, r.buf, nullptr, 4) == HAL_OK); // NEEDS REVIEW 333/g
	}
	{
		TiEncRig r(3, true);
		CHECK(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_2, nullptr, r.buf2, 4) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start returns HAL_ERROR (poll, IT) / HAL_BUSY (DMA) when the channel is BUSY and changes nothing") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiEncRig r(3, true);
		REQUIRE(tiEncStart(r, mode, TIM_CHANNEL_1) == HAL_OK);
		const uint32_t ccer = r.t->CCER, dier = r.t->DIER;
		CHECK(tiEncStart(r, mode, TIM_CHANNEL_1) == (mode == 2 ? HAL_BUSY : HAL_ERROR));
		CHECK(r.t->CCER == ccer);
		CHECK(r.t->DIER == dier);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start of TIM_CHANNEL_ALL with one channel BUSY returns HAL_ERROR (poll, IT) / HAL_BUSY (DMA) and changes nothing") {
	for (int mode = 0; mode < 3; ++mode)
		for (unsigned busy = 1; busy <= 2; ++busy) {
			CAPTURE(mode);
			CAPTURE(busy);
			TiEncRig r(3, true);
			REQUIRE(tiEncStart(r, mode, tiCh[busy - 1]) == HAL_OK);
			const uint32_t ccer = r.t->CCER;
			CHECK(tiEncStart(r, mode, TIM_CHANNEL_ALL) == (mode == 2 ? HAL_BUSY : HAL_ERROR));
			CHECK(r.t->CCER == ccer);
			CHECK(r.state(3 - busy) == tiReady);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start with the complementary state of the channel BUSY is refused (HAL_ERROR poll / IT, HAL_BUSY DMA)") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiEncRig r(3, true);
		r.h.ChannelNState[1] = HAL_TIM_CHANNEL_STATE_BUSY;
		CHECK(tiEncStart(r, mode, TIM_CHANNEL_2) == (mode == 2 ? HAL_BUSY : HAL_ERROR));
		CHECK(r.state(2) == tiReady);
		CHECK(r.t->CCER == (r.t->CCER & ~0x11u));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA with a channel state that is neither READY nor BUSY (RESET) returns HAL_ERROR, as do Start and Start_IT") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiRig r(3, true); // not initialised: channel states RESET
		CHECK(tiEncStart(r, mode, TIM_CHANNEL_1) == HAL_ERROR);
		CHECK(r.cen() == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA that fails because the DMA stream cannot start leaves the channel state READY" * KNOWN_ISSUE("candidate vendor bug V112 (arguable: UM1725 gives no state rule for a failed DMA start): the channel and complementary state stay BUSY after the HAL_ERROR return, so every later start of that channel returns HAL_BUSY (same pattern as V101 of the SPI driver)")) {
	TiEncRig r(3, true);
	REQUIRE(HAL_DMA_Start_IT(&r.dma[0], r.ccrAddr(1), r.addr(), 2) == HAL_OK);
	CHECK(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_1, r.buf, r.buf2, 4) == HAL_ERROR);
	CHECK(r.state(1) == tiReady);
	CHECK(r.nState(1) == tiReady);
}

TEST_CASE_FIXTURE(TimFixture, "TI3 the encoder counts once started: the model counter advances only while CEN is set") {
	TiEncRig r(3);
	r.t->ARR = 99;
	hostTimAdvance(3, 5);
	CHECK(r.t->CNT == 0u);
	REQUIRE(HAL_TIM_Encoder_Start(&r.h, TIM_CHANNEL_ALL) == HAL_OK);
	hostTimAdvance(3, 5);
	CHECK(r.t->CNT == 5u);
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Stop of channel 1 / 2 / ALL clears CCxE, the request bits and the counter, states READY (all modes)") {
	for (int mode = 0; mode < 3; ++mode)
		for (unsigned sel = 0; sel < 3; ++sel) {
			CAPTURE(mode);
			CAPTURE(sel);
			TiEncRig r(3, true);
			const uint32_t ch = sel == 2 ? TIM_CHANNEL_ALL : tiCh[sel];
			REQUIRE(tiEncStart(r, mode, ch) == HAL_OK);
			REQUIRE(tiEncStop(r, mode, ch) == HAL_OK);
			CHECK((r.t->CCER & 0x11u) == 0u);
			CHECK((r.t->DIER & tiReqAll) == 0u);
			CHECK(r.cen() == false);
			for (unsigned c = 1; c <= 2; ++c) {
				CHECK(r.state(c) == tiReady);
				CHECK(r.nState(c) == tiReady);
			}
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Stop_DMA aborts the streams of the stopped channels only") {
	{
		TiEncRig r(3, true);
		REQUIRE(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_ALL, r.buf, r.buf2, 4) == HAL_OK);
		REQUIRE(HAL_TIM_Encoder_Stop_DMA(&r.h, TIM_CHANNEL_ALL) == HAL_OK);
		CHECK(r.streamOn(1) == false);
		CHECK(r.streamOn(2) == false);
	}
	{
		TiEncRig r(3, true);
		REQUIRE(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_ALL, r.buf, r.buf2, 4) == HAL_OK);
		REQUIRE(HAL_TIM_Encoder_Stop_DMA(&r.h, TIM_CHANNEL_2) == HAL_OK);
		CHECK(r.streamOn(1) == true);
		CHECK(r.streamOn(2) == false);
		CHECK((r.t->DIER & tiReqAll) == tiCcde(1));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Stop of one channel only sets that channel READY and keeps the other BUSY") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiEncRig r(3, true);
		REQUIRE(tiEncStart(r, mode, TIM_CHANNEL_ALL) == HAL_OK);
		REQUIRE(tiEncStop(r, mode, TIM_CHANNEL_1) == HAL_OK);
		CHECK(r.state(1) == tiReady);
		CHECK(r.nState(1) == tiReady);
		CHECK(r.state(2) == tiBusy);
		CHECK(r.nState(2) == tiBusy);
		CHECK((r.t->CCER & 0x11u) == tiCcxe(2));
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI3 Encoder Stop of one channel keeps the counter running while the other channel is enabled") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiEncRig r(3, true);
		REQUIRE(tiEncStart(r, mode, TIM_CHANNEL_ALL) == HAL_OK);
		REQUIRE(tiEncStop(r, mode, TIM_CHANNEL_1) == HAL_OK);
		CHECK(r.cen() == true); // NEEDS REVIEW 333/c
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 an encoder stopped by TIM_CHANNEL_ALL can be started again (all modes)") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiEncRig r(3, true);
		REQUIRE(tiEncStart(r, mode, TIM_CHANNEL_ALL) == HAL_OK);
		REQUIRE(tiEncStop(r, mode, TIM_CHANNEL_ALL) == HAL_OK);
		if (mode == 2) { tiFinishAbort(r, 0); tiFinishAbort(r, 1); }
		CHECK(tiEncStart(r, mode, TIM_CHANNEL_ALL) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 the DMA completion of an encoder channel 1 stream calls IC_CaptureCallback with active channel 1 and sets the channel READY") {
	TiEncRig r(3, true);
	REQUIRE(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_1, r.buf, r.buf2, 4) == HAL_OK);
	hostDmaComplete(0, tiStreamOf[0]);
	HAL_DMA_IRQHandler(&r.dma[0]);
	CHECK(timCounts.icCapture == 1u);
	CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_1);
	CHECK(r.state(1) == tiReady);
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder start and stop work on TIM1, TIM2, TIM4, TIM5 and TIM8 in poll and IT mode") {
	const unsigned tims[5] = {1, 2, 4, 5, 8};
	for (unsigned tim : tims)
		for (int mode = 0; mode < 2; ++mode) {
			CAPTURE(tim);
			CAPTURE(mode);
			TiEncRig r(tim);
			REQUIRE(tiEncStart(r, mode, TIM_CHANNEL_ALL) == HAL_OK);
			CHECK((r.t->CCER & 0x11u) == 0x11u);
			CHECK(r.cen() == true);
			REQUIRE(tiEncStop(r, mode, TIM_CHANNEL_ALL) == HAL_OK);
			CHECK(r.cen() == false);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder_GetState is RESET before and READY after Encoder_Init") {
	TiRig r(3);
	CHECK(HAL_TIM_Encoder_GetState(&r.h) == HAL_TIM_STATE_RESET);
	const TIM_Encoder_InitTypeDef c = tiEnc(TIM_ENCODERMODE_TI1);
	REQUIRE(HAL_TIM_Encoder_Init(&r.h, &c) == HAL_OK);
	CHECK(HAL_TIM_Encoder_GetState(&r.h) == HAL_TIM_STATE_READY);
}

/* ================================================================ TI4: hall sensor */

namespace {
TIM_HallSensor_InitTypeDef tiHall() {
	TIM_HallSensor_InitTypeDef c{};
	c.IC1Polarity = TIM_ICPOLARITY_FALLING;
	c.IC1Prescaler = TIM_ICPSC_DIV2;
	c.IC1Filter = 3;
	c.Commutation_Delay = 500;
	return c;
}
struct TiHallRig : TiRig {
	explicit TiHallRig(unsigned tim, bool dma = false) : TiRig(tim, dma) {
		const TIM_HallSensor_InitTypeDef c = tiHall();
		REQUIRE(HAL_TIMEx_HallSensor_Init(&h, &c) == HAL_OK);
	}
};
HAL_StatusTypeDef tiHallStart(TiRig &r, int mode) {
	return mode == 0 ? HAL_TIMEx_HallSensor_Start(&r.h) : mode == 1 ? HAL_TIMEx_HallSensor_Start_IT(&r.h) : HAL_TIMEx_HallSensor_Start_DMA(&r.h, r.buf, 4);
}
HAL_StatusTypeDef tiHallStop(TiRig &r, int mode) {
	return mode == 0 ? HAL_TIMEx_HallSensor_Stop(&r.h) : mode == 1 ? HAL_TIMEx_HallSensor_Stop_IT(&r.h) : HAL_TIMEx_HallSensor_Stop_DMA(&r.h);
}
} // namespace

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Init of a NULL handle returns HAL_ERROR") {
	const TIM_HallSensor_InitTypeDef c = tiHall();
	CHECK(HAL_TIMEx_HallSensor_Init(nullptr, &c) == HAL_ERROR);
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Init of a RESET handle runs the HallSensor MspInit once, unlocks and sets state and channel 1 / 2 states READY") {
	for (unsigned tim : tiHallTimers) {
		CAPTURE(tim);
		TiRig r(tim);
		r.h.Lock = HAL_LOCKED;
		const TIM_HallSensor_InitTypeDef c = tiHall();
		REQUIRE(HAL_TIMEx_HallSensor_Init(&r.h, &c) == HAL_OK);
		CHECK(timCounts.mspInit[6] == 1u);
		CHECK(r.h.Lock == HAL_UNLOCKED);
		CHECK(HAL_TIMEx_HallSensor_GetState(&r.h) == HAL_TIM_STATE_READY);
		for (unsigned k = 1; k <= 2; ++k) {
			CHECK(r.state(k) == tiReady);
			CHECK(r.nState(k) == tiReady);
		}
		CHECK(r.h.DMABurstState == HAL_DMA_BURST_STATE_READY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Init of an initialised handle does not run MspInit again") {
	TiHallRig r(3);
	const TIM_HallSensor_InitTypeDef c = tiHall();
	REQUIRE(HAL_TIMEx_HallSensor_Init(&r.h, &c) == HAL_OK);
	CHECK(timCounts.mspInit[6] == 1u);
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Init sets TI1S (the TI1 XOR of the three inputs) in CR2 and keeps the other CR2 bits") {
	for (unsigned tim : tiHallTimers) {
		CAPTURE(tim);
		TiRig r(tim);
		r.t->CR2 = 0x8u; // CCDS (TIM1 / 8: also a valid bit)
		const TIM_HallSensor_InitTypeDef c = tiHall();
		REQUIRE(HAL_TIMEx_HallSensor_Init(&r.h, &c) == HAL_OK);
		CHECK((r.t->CR2 & 0x80u) == 0x80u);
		CHECK((r.t->CR2 & 0x8u) == 0x8u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Init selects OC2REF as trigger output: MMS = 101 (RM0090 18.3.18)") {
	for (unsigned tim : tiHallTimers) {
		CAPTURE(tim);
		TiRig r(tim);
		r.t->CR2 = 0x70u; // old MMS = 111
		const TIM_HallSensor_InitTypeDef c = tiHall();
		REQUIRE(HAL_TIMEx_HallSensor_Init(&r.h, &c) == HAL_OK);
		CHECK((r.t->CR2 & 0x70u) == 0x50u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Init configures the slave controller in reset mode on TI1F_ED (SMS = 100, TS = 100) and keeps other SMCR bits") {
	for (unsigned tim : tiHallTimers) {
		CAPTURE(tim);
		TiRig r(tim);
		r.t->SMCR = 0x80u | 0x30u | 0x7u; // MSM, TS = 011, SMS = 111
		const TIM_HallSensor_InitTypeDef c = tiHall();
		REQUIRE(HAL_TIMEx_HallSensor_Init(&r.h, &c) == HAL_OK);
		CHECK(r.t->SMCR == (0x80u | 0x40u | 0x4u));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Init configures channel 1 as capture on TRC (CC1S = 11) with the given prescaler, filter and polarity") {
	TiRig r(3);
	r.t->CCMR1 = 0xFFu; // old channel 1 byte
	const TIM_HallSensor_InitTypeDef c = tiHall();
	REQUIRE(HAL_TIMEx_HallSensor_Init(&r.h, &c) == HAL_OK);
	CHECK(tiCcmrByte(r.t, 1) == (3u | (1u << 2) | (3u << 4))); // CC1S 11, IC1PSC 01 (every 2), IC1F 3
	CHECK((r.t->CCER & 0xAu) == 0x2u);                         // CC1P (falling), CC1NP clear
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Init writes the polarity rising / falling / both edges of channel 1") {
	const uint32_t pols[3] = {TIM_ICPOLARITY_RISING, TIM_ICPOLARITY_FALLING, TIM_ICPOLARITY_BOTHEDGE};
	for (unsigned p = 0; p < 3; ++p) {
		CAPTURE(p);
		TiRig r(3);
		r.t->CCER = 0xAu;
		TIM_HallSensor_InitTypeDef c = tiHall();
		c.IC1Polarity = pols[p];
		REQUIRE(HAL_TIMEx_HallSensor_Init(&r.h, &c) == HAL_OK);
		CHECK((r.t->CCER & 0xAu) == tiPol(1, p));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Init writes the prescaler and filter values of channel 1") {
	const uint32_t pscs[4] = {TIM_ICPSC_DIV1, TIM_ICPSC_DIV2, TIM_ICPSC_DIV4, TIM_ICPSC_DIV8};
	for (unsigned p = 0; p < 4; ++p)
		for (uint32_t f : {0u, 7u, 15u}) {
			CAPTURE(p);
			CAPTURE(f);
			TiRig r(3);
			TIM_HallSensor_InitTypeDef c = tiHall();
			c.IC1Prescaler = pscs[p];
			c.IC1Filter = f;
			REQUIRE(HAL_TIMEx_HallSensor_Init(&r.h, &c) == HAL_OK);
			CHECK(tiCcmrByte(r.t, 1) == (3u | (p << 2) | (f << 4)));
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Init programs channel 2 as PWM mode 2 output (OC2M = 111, CC2S = 00) with CCR2 = Commutation_Delay") {
	for (unsigned tim : tiHallTimers) {
		CAPTURE(tim);
		TiRig r(tim);
		r.t->CCMR1 = 0x0300u; // old CC2S = 11
		const TIM_HallSensor_InitTypeDef c = tiHall();
		REQUIRE(HAL_TIMEx_HallSensor_Init(&r.h, &c) == HAL_OK);
		CHECK(tiCcmrByte(r.t, 2) == 0x70u);
		CHECK(r.t->CCR2 == 500u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Init writes the commutation delay 0 and 0xFFFF into CCR2") {
	for (uint32_t d : {0u, 0xFFFFu}) {
		CAPTURE(d);
		TiRig r(3);
		TIM_HallSensor_InitTypeDef c = tiHall();
		c.Commutation_Delay = d;
		REQUIRE(HAL_TIMEx_HallSensor_Init(&r.h, &c) == HAL_OK);
		CHECK(r.t->CCR2 == d);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Init leaves the output polarity of channel 2 high (CC2P = 0) and does not enable channel 2 or the counter") {
	TiRig r(3);
	r.t->CCER = 0x20u; // CC2P preset
	const TIM_HallSensor_InitTypeDef c = tiHall();
	REQUIRE(HAL_TIMEx_HallSensor_Init(&r.h, &c) == HAL_OK);
	CHECK((r.t->CCER & 0x20u) == 0u);
	CHECK((r.t->CCER & 0x11u) == 0u);
	CHECK(r.cen() == false);
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Init writes Period to ARR and Prescaler to PSC") {
	TiRig r(2);
	r.h.Init.Period = 0xFFFFFFFFu; // RM0090: ARR at the maximum so the TI1 change clears the counter
	r.h.Init.Prescaler = 83;
	const TIM_HallSensor_InitTypeDef c = tiHall();
	REQUIRE(HAL_TIMEx_HallSensor_Init(&r.h, &c) == HAL_OK);
	CHECK(r.t->ARR == 0xFFFFFFFFu);
	CHECK(r.t->PSC == 83u);
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_DeInit runs MspDeInit once, stops the counter and sets state, channel states and burst state RESET") {
	TiHallRig r(3);
	r.t->CR1 |= tiCR1_CEN;
	r.h.Lock = HAL_LOCKED;
	REQUIRE(HAL_TIMEx_HallSensor_DeInit(&r.h) == HAL_OK);
	CHECK(timCounts.mspDeInit[6] == 1u);
	CHECK(timCounts.stateAtLast == HAL_TIM_STATE_BUSY);
	CHECK(r.cen() == false);
	CHECK(HAL_TIMEx_HallSensor_GetState(&r.h) == HAL_TIM_STATE_RESET);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	for (unsigned k = 1; k <= 2; ++k) {
		CHECK(r.state(k) == HAL_TIM_CHANNEL_STATE_RESET);
		CHECK(r.nState(k) == HAL_TIM_CHANNEL_STATE_RESET);
	}
	CHECK(r.h.DMABurstState == HAL_DMA_BURST_STATE_RESET);
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Start enables CC1E and the counter and sets channel 1 / 2 and complementary states BUSY") {
	for (unsigned tim : tiHallTimers) {
		CAPTURE(tim);
		TiHallRig r(tim);
		REQUIRE(HAL_TIMEx_HallSensor_Start(&r.h) == HAL_OK);
		CHECK((r.t->CCER & 0x11u) == 0x1u);
		CHECK(r.cen() == true);
		CHECK((r.t->DIER & tiReqAll) == 0u);
		for (unsigned k = 1; k <= 2; ++k) {
			CHECK(r.state(k) == tiBusy);
			CHECK(r.nState(k) == tiBusy);
		}
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Start_IT enables only the CC1 interrupt besides CC1E and the counter") {
	TiHallRig r(3);
	REQUIRE(HAL_TIMEx_HallSensor_Start_IT(&r.h) == HAL_OK);
	CHECK((r.t->DIER & tiReqAll) == tiCcie(1));
	CHECK((r.t->CCER & 0x11u) == 0x1u);
	CHECK(r.cen() == true);
	CHECK(r.state(1) == tiBusy);
	CHECK(r.state(2) == tiBusy);
	CHECK(r.nState(1) == tiBusy);
	CHECK(r.nState(2) == tiBusy);
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Start_DMA programs stream 4 with CCR1, the buffer and Length, enables CC1DE, CC1E and the counter, and sets only channel 1 BUSY") {
	TiHallRig r(3, true);
	REQUIRE(HAL_TIMEx_HallSensor_Start_DMA(&r.h, r.buf, 9) == HAL_OK);
	DMA_Stream_TypeDef *s = tiStream(tiStreamOf[0]);
	CHECK(s->PAR == r.ccrAddr(1));
	CHECK(s->M0AR == r.addr());
	CHECK(s->NDTR == 9u);
	CHECK((s->CR & 1u) == 1u);
	CHECK((r.t->DIER & tiReqAll) == tiCcde(1));
	CHECK((r.t->CCER & 0x11u) == 0x1u);
	CHECK(r.cen() == true);
	CHECK(r.state(1) == tiBusy);
	CHECK(r.nState(1) == tiBusy);
	CHECK(r.state(2) == tiReady);
	CHECK(r.nState(2) == tiReady);
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor Start with SMS = trigger mode enables CC1E but not the counter (all modes)") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiHallRig r(3, true);
		r.t->SMCR = (r.t->SMCR & ~7u) | 6u;
		REQUIRE(tiHallStart(r, mode) == HAL_OK);
		CHECK((r.t->CCER & 1u) == 1u);
		CHECK(r.cen() == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor Start with the reset slave mode set by HallSensor_Init enables the counter (all modes)") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiHallRig r(3, true);
		CHECK((r.t->SMCR & 7u) == 4u);
		REQUIRE(tiHallStart(r, mode) == HAL_OK);
		CHECK(r.cen() == true);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Start and Start_IT return HAL_ERROR and change nothing when any of the four channel states is not READY") {
	for (int which = 0; which < 4; ++which)
		for (int mode = 0; mode < 2; ++mode) {
			CAPTURE(which);
			CAPTURE(mode);
			TiHallRig r(3);
			(which < 2 ? r.h.ChannelState[which] : r.h.ChannelNState[which - 2]) = HAL_TIM_CHANNEL_STATE_BUSY;
			CHECK(tiHallStart(r, mode) == HAL_ERROR);
			CHECK((r.t->CCER & 0x11u) == 0u);
			CHECK(r.cen() == false);
			CHECK(r.t->DIER == 0u);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Start_DMA returns HAL_BUSY when channel 1 or its complementary state is BUSY and starts nothing") {
	for (int which = 0; which < 2; ++which) {
		CAPTURE(which);
		TiHallRig r(3, true);
		(which == 0 ? r.h.ChannelState[0] : r.h.ChannelNState[0]) = HAL_TIM_CHANNEL_STATE_BUSY;
		CHECK(HAL_TIMEx_HallSensor_Start_DMA(&r.h, r.buf, 4) == HAL_BUSY);
		CHECK(r.streamOn(1) == false);
		CHECK(r.cen() == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Start_DMA with a NULL buffer or Length 0 returns HAL_ERROR and starts nothing") {
	for (int n = 0; n < 2; ++n) {
		CAPTURE(n);
		TiHallRig r(3, true);
		const HAL_StatusTypeDef st = n == 0 ? HAL_TIMEx_HallSensor_Start_DMA(&r.h, nullptr, 4) : HAL_TIMEx_HallSensor_Start_DMA(&r.h, r.buf, 0);
		CHECK(st == HAL_ERROR);
		CHECK(r.state(1) == tiReady);
		CHECK(r.streamOn(1) == false);
		CHECK(r.cen() == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor Start of a handle that was not initialised returns HAL_ERROR (all modes)") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiRig r(3, true);
		CHECK(tiHallStart(r, mode) == HAL_ERROR);
		CHECK(r.cen() == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Start_DMA that fails because the DMA stream cannot start leaves channel 1 state READY" * KNOWN_ISSUE("candidate vendor bug V112 (arguable: UM1725 gives no state rule for a failed DMA start): the channel 1 and complementary state stay BUSY after the HAL_ERROR return, so every later start returns HAL_BUSY (same pattern as V101 of the SPI driver)")) {
	TiHallRig r(3, true);
	REQUIRE(HAL_DMA_Start_IT(&r.dma[0], r.ccrAddr(1), r.addr(), 2) == HAL_OK);
	CHECK(HAL_TIMEx_HallSensor_Start_DMA(&r.h, r.buf, 4) == HAL_ERROR);
	CHECK(r.state(1) == tiReady);
	CHECK(r.nState(1) == tiReady);
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Stop clears CC1E, the CC1 request bit and the counter and sets all four states READY (poll and IT)") {
	for (int mode = 0; mode < 2; ++mode) {
		CAPTURE(mode);
		TiHallRig r(3);
		REQUIRE(tiHallStart(r, mode) == HAL_OK);
		REQUIRE(tiHallStop(r, mode) == HAL_OK);
		CHECK((r.t->CCER & 0x11u) == 0u);
		CHECK((r.t->DIER & tiReqAll) == 0u);
		CHECK(r.cen() == false);
		for (unsigned k = 1; k <= 2; ++k) {
			CHECK(r.state(k) == tiReady);
			CHECK(r.nState(k) == tiReady);
		}
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Stop_IT keeps other interrupt enables") {
	TiHallRig r(3);
	r.t->DIER = 0x1u | 0x4u; // UIE, CC2IE
	REQUIRE(HAL_TIMEx_HallSensor_Start_IT(&r.h) == HAL_OK);
	REQUIRE(HAL_TIMEx_HallSensor_Stop_IT(&r.h) == HAL_OK);
	CHECK(r.t->DIER == 0x5u);
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Stop_DMA aborts the stream, clears CC1DE, CC1E and the counter and sets channel 1 and its complementary state READY") {
	TiHallRig r(3, true);
	REQUIRE(HAL_TIMEx_HallSensor_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	REQUIRE(HAL_TIMEx_HallSensor_Stop_DMA(&r.h) == HAL_OK);
	CHECK(r.streamOn(1) == false);
	CHECK((r.t->DIER & tiReqAll) == 0u);
	CHECK((r.t->CCER & 0x11u) == 0u);
	CHECK(r.cen() == false);
	CHECK(r.state(1) == tiReady);
	CHECK(r.nState(1) == tiReady);
}

TEST_CASE_FIXTURE(TimFixture, "TI4 a stopped hall sensor interface can be started again (all modes)") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiHallRig r(3, true);
		REQUIRE(tiHallStart(r, mode) == HAL_OK);
		REQUIRE(tiHallStop(r, mode) == HAL_OK);
		if (mode == 2) tiFinishAbort(r, 0);
		CHECK(tiHallStart(r, mode) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 a capture on channel 1 after HallSensor_Init is accepted (channel 1 is an input) and ReadCapturedValue returns it") {
	TiHallRig r(3);
	REQUIRE(HAL_TIMEx_HallSensor_Start(&r.h) == HAL_OK);
	REQUIRE(hostTimCapture(3, 1, 0x321u) == 1);
	CHECK(HAL_TIM_ReadCapturedValue(&r.h, TIM_CHANNEL_1) == 0x321u);
}

TEST_CASE_FIXTURE(TimFixture, "TI4 the DMA completion of the hall sensor stream calls IC_CaptureCallback with active channel 1 and sets channel 1 READY") {
	TiHallRig r(3, true);
	REQUIRE(HAL_TIMEx_HallSensor_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	hostDmaComplete(0, tiStreamOf[0]);
	HAL_DMA_IRQHandler(&r.dma[0]);
	CHECK(timCounts.icCapture == 1u);
	CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_1);
	CHECK(r.state(1) == tiReady);
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_GetState is RESET before and READY after HallSensor_Init") {
	TiRig r(3);
	CHECK(HAL_TIMEx_HallSensor_GetState(&r.h) == HAL_TIM_STATE_RESET);
	const TIM_HallSensor_InitTypeDef c = tiHall();
	REQUIRE(HAL_TIMEx_HallSensor_Init(&r.h, &c) == HAL_OK);
	CHECK(HAL_TIMEx_HallSensor_GetState(&r.h) == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TI1 Start_DMA right after Stop_DMA returns HAL_ERROR until the DMA interrupt of the abort has run") {
	TiRig r(3, true);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_IC_Start_DMA(&r.h, TIM_CHANNEL_1, r.buf, 4) == HAL_OK);
	REQUIRE(HAL_TIM_IC_Stop_DMA(&r.h, TIM_CHANNEL_1) == HAL_OK);
	CHECK(HAL_TIM_IC_Start_DMA(&r.h, TIM_CHANNEL_1, r.buf, 4) == HAL_ERROR); // NEEDS REVIEW 333/h
	CHECK(r.state(1) == tiBusy);
}

/* ---- remaining error branches */

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Start_DMA of channels 2, 3 and 4 whose stream cannot start returns HAL_ERROR") {
	for (unsigned c = 2; c <= 4; ++c) {
		CAPTURE(c);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_DMA_Start_IT(&r.dma[c - 1], r.ccrAddr(c), r.addr(), 2) == HAL_OK);
		CHECK(HAL_TIM_IC_Start_DMA(&r.h, tiCh[c - 1], r.buf, 4) == HAL_ERROR);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA of channel 2 and of TIM_CHANNEL_ALL (stream 1 or stream 2 not startable) returns HAL_ERROR") {
	for (int which = 0; which < 3; ++which) {
		CAPTURE(which);
		TiEncRig r(3, true);
		const unsigned blocked = which == 2 ? 1u : 0u; // dma index
		if (which == 0) { // channel 2 alone, its stream busy
			REQUIRE(HAL_DMA_Start_IT(&r.dma[1], r.ccrAddr(2), r.addr2(), 2) == HAL_OK);
			CHECK(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_2, r.buf, r.buf2, 4) == HAL_ERROR);
		} else {
			REQUIRE(HAL_DMA_Start_IT(&r.dma[which == 1 ? 0 : blocked], r.ccrAddr(which == 1 ? 1 : 2), r.addr(), 2) == HAL_OK);
			CHECK(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_ALL, r.buf, r.buf2, 4) == HAL_ERROR);
		}
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA of channel 2 and of TIM_CHANNEL_ALL on a handle that was not initialised returns HAL_ERROR") {
	const uint32_t chans[2] = {TIM_CHANNEL_2, TIM_CHANNEL_ALL};
	for (uint32_t ch : chans) {
		CAPTURE(ch);
		TiRig r(3, true);
		CHECK(HAL_TIM_Encoder_Start_DMA(&r.h, ch, r.buf, r.buf2, 4) == HAL_ERROR);
		CHECK(r.cen() == false);
	}
}

/* ================================================================ fix pass */

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Stop_DMA of channel 1 aborts only stream 4 and clears only CC1DE and CC1E") {
	TiEncRig r(3, true);
	REQUIRE(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_ALL, r.buf, r.buf2, 4) == HAL_OK);
	REQUIRE(HAL_TIM_Encoder_Stop_DMA(&r.h, TIM_CHANNEL_1) == HAL_OK);
	CHECK(r.streamOn(1) == false);
	CHECK(r.streamOn(2) == true);
	CHECK((r.t->DIER & tiReqAll) == tiCcde(2));
	CHECK((r.t->CCER & 0x11u) == tiCcxe(2));
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Stop of channel 2 alone keeps channel 1 enabled with its request bit, in every mode") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiEncRig r(3, true);
		REQUIRE(tiEncStart(r, mode, TIM_CHANNEL_ALL) == HAL_OK);
		REQUIRE(tiEncStop(r, mode, TIM_CHANNEL_2) == HAL_OK);
		CHECK((r.t->CCER & 0x11u) == tiCcxe(1));
		CHECK((r.t->DIER & tiReqAll) == tiReq(mode, 1));
		CHECK(r.state(1) == tiBusy);
		CHECK(r.state(2) == tiReady);
		CHECK(r.nState(2) == tiReady);
		if (mode == 2) {
			CHECK(r.streamOn(1) == true);
			CHECK(r.streamOn(2) == false);
		}
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Stop of channel 1 alone keeps channel 2 enabled with its request bit, in every mode") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiEncRig r(3, true);
		REQUIRE(tiEncStart(r, mode, TIM_CHANNEL_ALL) == HAL_OK);
		REQUIRE(tiEncStop(r, mode, TIM_CHANNEL_1) == HAL_OK);
		CHECK((r.t->CCER & 0x11u) == tiCcxe(2));
		CHECK((r.t->DIER & tiReqAll) == tiReq(mode, 2));
		CHECK(r.state(2) == tiBusy);
		CHECK(r.state(1) == tiReady);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA of both channels: a DMA error on stream 4 / stream 5 calls ErrorCallback with the matching active channel and sets only that channel READY") {
	for (unsigned c = 1; c <= 2; ++c) {
		CAPTURE(c);
		TiEncRig r(3, true);
		REQUIRE(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_ALL, r.buf, r.buf2, 4) == HAL_OK);
		hostDmaRaise(0, tiStreamOf[c - 1], HOST_DMA_TEIF);
		HAL_DMA_IRQHandler(&r.dma[c - 1]);
		CHECK(timCounts.error == 1u);
		CHECK(timCounts.channelAtLast == (c == 1 ? HAL_TIM_ACTIVE_CHANNEL_1 : HAL_TIM_ACTIVE_CHANNEL_2));
		CHECK(r.state(c) == tiReady);
		CHECK(r.state(3 - c) == tiBusy);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA of a single channel: a DMA error calls ErrorCallback with that channel active") {
	for (unsigned c = 1; c <= 2; ++c) {
		CAPTURE(c);
		TiEncRig r(3, true);
		REQUIRE(HAL_TIM_Encoder_Start_DMA(&r.h, tiCh[c - 1], r.buf, r.buf2, 4) == HAL_OK);
		hostDmaRaise(0, tiStreamOf[c - 1], HOST_DMA_TEIF);
		HAL_DMA_IRQHandler(&r.dma[c - 1]);
		CHECK(timCounts.error == 1u);
		CHECK(timCounts.channelAtLast == (c == 1 ? HAL_TIM_ACTIVE_CHANNEL_1 : HAL_TIM_ACTIVE_CHANNEL_2));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA of both channels: completion and half completion of each stream call the capture callbacks with the matching active channel") {
	for (unsigned c = 1; c <= 2; ++c) {
		CAPTURE(c);
		TiEncRig r(3, true);
		REQUIRE(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_ALL, r.buf, r.buf2, 4) == HAL_OK);
		const HAL_TIM_ActiveChannel want = c == 1 ? HAL_TIM_ACTIVE_CHANNEL_1 : HAL_TIM_ACTIVE_CHANNEL_2;
		hostDmaStepItems(0, tiStreamOf[c - 1], 2); // half of 4 items
		HAL_DMA_IRQHandler(&r.dma[c - 1]);
		CHECK(timCounts.icCaptureHalf == 1u);
		CHECK(timCounts.icCapture == 0u);
		CHECK(timCounts.channelAtLast == want);
		hostDmaComplete(0, tiStreamOf[c - 1]);
		HAL_DMA_IRQHandler(&r.dma[c - 1]);
		CHECK(timCounts.icCapture == 1u);
		CHECK(timCounts.channelAtLast == want);
		CHECK(r.state(c) == tiReady);
		CHECK(r.state(3 - c) == tiBusy);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Start_DMA: a DMA error, half completion and completion of each channel stream reach the matching callback and active channel") {
	for (unsigned c = 1; c <= 4; ++c) {
		CAPTURE(c);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_IC_Start_DMA(&r.h, tiCh[c - 1], r.buf, 4) == HAL_OK);
		const HAL_TIM_ActiveChannel want = static_cast<HAL_TIM_ActiveChannel>(1u << (c - 1));
		hostDmaRaise(0, tiStreamOf[c - 1], HOST_DMA_TEIF);
		HAL_DMA_IRQHandler(&r.dma[c - 1]);
		CHECK(timCounts.error == 1u);
		CHECK(timCounts.channelAtLast == want);
	}
	for (unsigned c = 1; c <= 4; ++c) {
		CAPTURE(c);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_IC_Start_DMA(&r.h, tiCh[c - 1], r.buf, 4) == HAL_OK);
		hostDmaStepItems(0, tiStreamOf[c - 1], 2);
		HAL_DMA_IRQHandler(&r.dma[c - 1]);
		CHECK(timCounts.icCaptureHalf == 1u);
		CHECK(timCounts.channelAtLast == static_cast<HAL_TIM_ActiveChannel>(1u << (c - 1)));
		hostDmaComplete(0, tiStreamOf[c - 1]);
		HAL_DMA_IRQHandler(&r.dma[c - 1]);
		CHECK(timCounts.icCapture == 1u);
		CHECK(timCounts.channelAtLast == static_cast<HAL_TIM_ActiveChannel>(1u << (c - 1)));
		CHECK(r.state(c) == tiReady);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start, Start_IT and Start_DMA of channel 1 / 2 / ALL: each of the four channel states singly BUSY decides the result") {
	// channel selection 0 = ch1, 1 = ch2, 2 = ALL; which = ChannelState[0], [1], ChannelNState[0], [1]
	for (int mode = 0; mode < 3; ++mode)
		for (int sel = 0; sel < 3; ++sel)
			for (int which = 0; which < 4; ++which) {
				CAPTURE(mode);
				CAPTURE(sel);
				CAPTURE(which);
				TiEncRig r(3, true);
				(which < 2 ? r.h.ChannelState[which] : r.h.ChannelNState[which - 2]) = HAL_TIM_CHANNEL_STATE_BUSY;
				const uint32_t ch = sel == 2 ? TIM_CHANNEL_ALL : tiCh[sel];
				const bool blocked = sel == 2 || (which % 2) == sel;
				const HAL_StatusTypeDef st = tiEncStart(r, mode, ch);
				if (blocked) {
					CHECK(st == (mode == 2 ? HAL_BUSY : HAL_ERROR));
					CHECK((r.t->CCER & 0x11u) == 0u);
					CHECK(r.cen() == false);
				} else {
					CHECK(st == HAL_OK);
				}
			}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor Start_DMA: only channel 1 and its complementary state decide, each singly BUSY returns HAL_BUSY") {
	for (int which = 0; which < 2; ++which) {
		CAPTURE(which);
		TiHallRig r(3, true);
		(which == 0 ? r.h.ChannelState[0] : r.h.ChannelNState[0]) = HAL_TIM_CHANNEL_STATE_BUSY;
		CHECK(HAL_TIMEx_HallSensor_Start_DMA(&r.h, r.buf, 4) == HAL_BUSY);
		CHECK((r.t->CCER & 1u) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor Start_IT and Start set all four states BUSY, Start_DMA sets channel 1 and its complementary state BUSY") {
	for (int mode = 0; mode < 3; ++mode) {
		CAPTURE(mode);
		TiHallRig r(3, true);
		REQUIRE(tiHallStart(r, mode) == HAL_OK);
		CHECK(r.state(1) == tiBusy);
		CHECK(r.nState(1) == tiBusy);
		CHECK(r.state(2) == (mode == 2 ? tiReady : tiBusy));
		CHECK(r.nState(2) == (mode == 2 ? tiReady : tiBusy));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Start_DMA with the channel state READY and the complementary state RESET (or the reverse) returns HAL_ERROR and starts nothing") {
	for (int which = 0; which < 2; ++which) {
		CAPTURE(which);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		(which == 0 ? r.h.ChannelNState[0] : r.h.ChannelState[0]) = HAL_TIM_CHANNEL_STATE_RESET;
		CHECK(HAL_TIM_IC_Start_DMA(&r.h, TIM_CHANNEL_1, r.buf, 4) == HAL_ERROR);
		CHECK(r.streamOn(1) == false);
		CHECK(r.cen() == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI1 IC_Start_DMA with only the complementary state BUSY returns HAL_BUSY on every channel") {
	for (unsigned c = 1; c <= 4; ++c) {
		CAPTURE(c);
		TiRig r(3, true);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		r.h.ChannelNState[c - 1] = HAL_TIM_CHANNEL_STATE_BUSY;
		CHECK(HAL_TIM_IC_Start_DMA(&r.h, tiCh[c - 1], r.buf, 4) == HAL_BUSY);
		CHECK(r.state(c) == tiReady);
		CHECK(r.streamOn(c) == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI2 ConfigClockSource TI2 replaces an old IC2F, keeps the channel 1 half of CCMR1 and sets the polarity of channel 2") {
	TiRig r(3);
	REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
	r.t->CCMR1 = 0xF05Au; // IC2F = 15, channel 1 half 0x5A
	r.t->CCER = 0x00A2u;  // CC2P, CC2NP, CC1P
	TIM_ClockConfigTypeDef cc{};
	cc.ClockSource = TIM_CLOCKSOURCE_TI2;
	cc.ClockPolarity = TIM_CLOCKPOLARITY_FALLING;
	cc.ClockFilter = 3;
	REQUIRE(HAL_TIM_ConfigClockSource(&r.h, &cc) == HAL_OK);
	CHECK(r.t->CCMR1 == 0x305Au); // IC2F bits 15:12 = 3
	CHECK((r.t->CCER & 0xFFu) == 0x22u); // CC2P (falling) and the untouched CC1P; CC2NP cleared
}

TEST_CASE_FIXTURE(TimFixture, "TI2 SlaveConfigSynchro with trigger TI1FP1 and with TI1F_ED replaces an old IC1F and keeps the channel 2 half of CCMR1") {
	const uint32_t triggers[2] = {TIM_TS_TI1FP1, TIM_TS_TI1F_ED};
	for (uint32_t trig : triggers) {
		CAPTURE(trig);
		TiRig r(3);
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		r.t->CCMR1 = 0x5AF0u; // IC1F = 15, channel 2 half 0x5A
		TIM_SlaveConfigTypeDef sc{};
		sc.SlaveMode = TIM_SLAVEMODE_RESET;
		sc.InputTrigger = trig;
		sc.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sc.TriggerFilter = 3;
		REQUIRE(HAL_TIM_SlaveConfigSynchro(&r.h, &sc) == HAL_OK);
		CHECK(r.t->CCMR1 == 0x5A30u); // IC1F bits 7:4 = 3
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Init on TIM8 programs the OC2 idle state OIS2 = 0 and OIS2N = 0 (idle state reset)") {
	for (unsigned tim : {1u, 8u}) {
		CAPTURE(tim);
		TiRig r(tim);
		r.t->CR2 = 0x0C00u; // OIS2 (bit 10), OIS2N (bit 11) preset
		const TIM_HallSensor_InitTypeDef c = tiHall();
		REQUIRE(HAL_TIMEx_HallSensor_Init(&r.h, &c) == HAL_OK);
		CHECK((r.t->CR2 & 0x0C00u) == 0u);
		CHECK((r.t->CR2 & 0x80u) == 0x80u);
	}
}

/* ================================================================ second fix pass */

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA with one relevant state RESET (neither READY nor BUSY) and the others READY returns HAL_ERROR and starts no stream") {
	for (int sel = 0; sel < 3; ++sel)
		for (int which = 0; which < 4; ++which) {
			CAPTURE(sel);
			CAPTURE(which);
			TiEncRig r(3, true);
			(which < 2 ? r.h.ChannelState[which] : r.h.ChannelNState[which - 2]) = HAL_TIM_CHANNEL_STATE_RESET;
			const uint32_t ch = sel == 2 ? TIM_CHANNEL_ALL : tiCh[sel];
			const bool relevant = sel == 2 || (which % 2) == sel;
			const HAL_StatusTypeDef st = HAL_TIM_Encoder_Start_DMA(&r.h, ch, r.buf, r.buf2, 4);
			if (relevant) {
				CHECK(st == HAL_ERROR);
				CHECK(r.streamOn(1) == false);
				CHECK(r.streamOn(2) == false);
				CHECK((r.t->CCER & 0x11u) == 0u);
				CHECK(r.cen() == false);
				CHECK(r.state(1) != tiBusy);
				CHECK(r.state(2) != tiBusy);
			} else {
				CHECK(st == HAL_OK);
			}
		}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA of ALL with one state BUSY and another RESET returns HAL_BUSY") {
	TiEncRig r(3, true);
	r.h.ChannelState[0] = HAL_TIM_CHANNEL_STATE_BUSY;
	r.h.ChannelNState[1] = HAL_TIM_CHANNEL_STATE_RESET;
	CHECK(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_ALL, r.buf, r.buf2, 4) == HAL_BUSY);
	CHECK(r.streamOn(1) == false);
}

TEST_CASE_FIXTURE(TimFixture, "TI4 HallSensor_Start_DMA with channel 1 or its complementary state RESET (neither READY nor BUSY) returns HAL_ERROR and starts no stream") {
	for (int which = 0; which < 2; ++which) {
		CAPTURE(which);
		TiHallRig r(3, true);
		(which == 0 ? r.h.ChannelState[0] : r.h.ChannelNState[0]) = HAL_TIM_CHANNEL_STATE_RESET;
		CHECK(HAL_TIMEx_HallSensor_Start_DMA(&r.h, r.buf, 4) == HAL_ERROR);
		CHECK(r.streamOn(1) == false);
		CHECK((r.t->CCER & 1u) == 0u);
		CHECK(r.cen() == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TI3 Encoder Start_DMA of channel 2 only: the completion of the CC2 stream calls the capture callback with active channel 2 and sets channel 2 READY") {
	TiEncRig r(3, true);
	REQUIRE(HAL_TIM_Encoder_Start_DMA(&r.h, TIM_CHANNEL_2, r.buf, r.buf2, 4) == HAL_OK);
	hostDmaStepItems(0, tiStreamOf[1], 2);
	HAL_DMA_IRQHandler(&r.dma[1]);
	CHECK(timCounts.icCaptureHalf == 1u);
	CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_2);
	hostDmaComplete(0, tiStreamOf[1]);
	HAL_DMA_IRQHandler(&r.dma[1]);
	CHECK(timCounts.icCapture == 1u);
	CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_2);
	CHECK(r.state(2) == tiReady);
	hostDmaRaise(0, tiStreamOf[1], HOST_DMA_TEIF);
	HAL_DMA_IRQHandler(&r.dma[1]);
	CHECK(timCounts.error == 1u);
	CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_2);
}
