/*
 * P12 T6a group TN: the complementary output functions of the real ST HAL timer extension (stm32f4xx_hal_tim_ex.c:
 * HAL_TIMEx_OCN_*, HAL_TIMEx_PWMN_*, HAL_TIMEx_OnePulseN_* and the private TIM_CCxNChannelCmd) against the timer model
 * (fakes/stm32_hal/HalTim.h) and the DMA model (HalDma.h). Requirements TN1-TN3 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T6A_REQUIREMENTS.md; compiled into ffb_hal_tests (F407VG) and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 (chapter "HAL TIM Extension Driver": the N functions take channels 1-3 and return a HAL status) and RM0090 Rev 22:
 *  - 17.4.9 TIMx_DIER: UIE bit 0, CC1IE..CC3IE bits 1..3, BIE bit 7, CC1DE..CC3DE bits 9..11.
 *  - 17.4.11 TIMx_CCER: CC1E bit 0, CC1NE bit 2, CC2E 4, CC2NE 6, CC3E 8, CC3NE 10 (4 bits per channel).
 *  - 17.4.18 TIMx_BDTR: MOE bit 15. 17.4.1 TIMx_CR1: CEN bit 0. 17.4.3 TIMx_SMCR: SMS bits 2:0, 110 = trigger mode (the counter
 *    starts at the trigger, so the software must not set CEN); one pulse mode is started by a trigger on the input channel.
 *  - Table 1: TIM1 0x40010000, TIM8 0x40010400. 17.4.21 register map: CCR1 0x34, CCR2 0x38, CCR3 0x3C.
 *  - Table 44 (DMA2 request mapping): TIM1_CH1 stream 1 channel 6, TIM1_CH2 stream 2 channel 6, TIM1_CH3 stream 6 channel 6,
 *    TIM8_CH1 stream 2 channel 7, TIM8_CH2 stream 3 channel 7, TIM8_CH3 stream 4 channel 7.
 *  - 10.5.5 DMA_SxCR: EN bit 0, TEIE bit 2, HTIE bit 3, TCIE bit 4. NDTR counts data items.
 * Time never matters here (no HAL tick waits). Every file-local symbol sits in the anonymous namespace or is prefixed tn.
 *
 * NEEDS REVIEW proposals (the lead numbers them):
 *  NEEDS REVIEW 335/a TN1 / TN3: the N functions with channel 4 (invalid for them): the poll Start returns HAL_OK and sets BUSY on ChannelNState[3]
 *     without enabling CC1NE-CC3NE; Start_IT / Start_DMA return HAL_ERROR after setting that state to BUSY (it is not restored); the poll
 *     Stop returns HAL_OK, the IT / DMA Stop return HAL_ERROR (UM1725 only names channels 1-3).
 *  NEEDS REVIEW 335/b TN1: the N functions on TIM3 (no complementary outputs, no BDTR): they run the same code; the model faults on the BDTR access.
 *  NEEDS REVIEW 335/c TN3: Start_DMA of a channel whose N state is BUSY returns HAL_BUSY (UM1725 silent), of a state that is neither READY nor
 *     BUSY returns HAL_ERROR.
 *  NEEDS REVIEW 335/d TN3: DMA completion of an OCN stream calls HAL_TIM_PWM_PulseFinishedCallback (not OC_DelayElapsed).
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

constexpr uint32_t tnCR1_CEN = 1u << 0;
constexpr uint32_t tnBDTR_MOE = 1u << 15;
constexpr uint32_t tnDIER_BIE = 1u << 7;
constexpr uint32_t tnDIER_CCIE_ALL = 0xEu;          // CC1IE..CC3IE
constexpr uint32_t tnDIER_CCDE_ALL = 0xEu << 8;     // CC1DE..CC3DE
constexpr uint32_t tnDMA_EN = 1u << 0;
constexpr uint32_t tnDMA_TEIE = 1u << 2;
constexpr uint32_t tnDMA_HTIE = 1u << 3;
constexpr uint32_t tnDMA_TCIE = 1u << 4;

constexpr uint32_t tnCcxe(unsigned c) { return 1u << (4 * (c - 1)); }
constexpr uint32_t tnCcxne(unsigned c) { return 4u << (4 * (c - 1)); }
constexpr uint32_t tnCcie(unsigned c) { return 1u << c; }
constexpr uint32_t tnCcde(unsigned c) { return 1u << (8 + c); }

uint32_t tnChannel(unsigned c) {
	const uint32_t v[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
	return v[c - 1];
}

/* RM0090 Table 44: stream and channel of the capture / compare request of TIM1 (index 0) and TIM8 (index 1). */
constexpr unsigned tnStreamOf[2][3] = {{1, 2, 6}, {2, 3, 4}};
constexpr uint32_t tnPeriphAddr[2][3] = {{0x40010034u, 0x40010038u, 0x4001003Cu}, {0x40010434u, 0x40010438u, 0x4001043Cu}};

DMA_Stream_TypeDef *tnStream(unsigned stream) {
	DMA_Stream_TypeDef *const s[8] = {DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7};
	return s[stream];
}

const unsigned tnTimers[2] = {1, 8};

HAL_TIM_ActiveChannel tnActive(unsigned c) {
	const HAL_TIM_ActiveChannel v[3] = {HAL_TIM_ACTIVE_CHANNEL_1, HAL_TIM_ACTIVE_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_3};
	return v[c - 1];
}

struct TnRig {
	TIM_HandleTypeDef h;
	DMA_HandleTypeDef dma[3];
	uint32_t buf[8];
	unsigned idx; // 0 = TIM1, 1 = TIM8
	TIM_TypeDef *t;

	explicit TnRig(unsigned tim, uint32_t dmaMode = DMA_NORMAL) : idx(tim == 1 ? 0 : 1), t(timInstance(tim)) {
		// a case builds several rigs: every rig starts from the reset state of all peripherals (same sequence as test_hal_spi_poll.cpp);
		// hostRegsReset clears the fault list, so the destructor checks the faults of each rig
		hostRegsDisarm();
		hostPeriphResetState();
		hostRegsReset();
		hostRegsArm();
		std::memset(&h, 0, sizeof h);
		std::memset(dma, 0, sizeof dma);
		std::memset(buf, 0, sizeof buf);
		h.Instance = t;
		h.Init.Prescaler = 0;
		h.Init.CounterMode = TIM_COUNTERMODE_UP;
		h.Init.Period = 999;
		h.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		REQUIRE(HAL_TIM_PWM_Init(&h) == HAL_OK);
		for (unsigned c = 0; c < 3; ++c) {
			dma[c].Instance = tnStream(tnStreamOf[idx][c]);
			dma[c].Init.Channel = idx == 0 ? DMA_CHANNEL_6 : DMA_CHANNEL_7;
			dma[c].Init.Direction = DMA_MEMORY_TO_PERIPH;
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
		timSupportReset();
	}
	~TnRig() { CHECK_MESSAGE(HalFixture::faults() == 0u, "register model fault in the rig of timer ", timNumberOf(t)); }
	unsigned stream(unsigned c) const { return tnStreamOf[idx][c - 1]; }
	uint32_t addr() const { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buf)); }
	HAL_TIM_ChannelStateTypeDef nState(unsigned c) const { return h.ChannelNState[c - 1]; }
	HAL_TIM_ChannelStateTypeDef state(unsigned c) const { return h.ChannelState[c - 1]; }
	bool moe() const { return (t->BDTR & tnBDTR_MOE) != 0u; }
	bool cen() const { return (t->CR1 & tnCR1_CEN) != 0u; }
};

/* The six start / stop families: OCN or PWMN, poll (0), IT (1), DMA (2). */
HAL_StatusTypeDef tnStart(TnRig &r, bool pwm, int mode, uint32_t ch, uint16_t len = 4) {
	if (pwm) return mode == 0 ? HAL_TIMEx_PWMN_Start(&r.h, ch) : mode == 1 ? HAL_TIMEx_PWMN_Start_IT(&r.h, ch) : HAL_TIMEx_PWMN_Start_DMA(&r.h, ch, r.buf, len);
	return mode == 0 ? HAL_TIMEx_OCN_Start(&r.h, ch) : mode == 1 ? HAL_TIMEx_OCN_Start_IT(&r.h, ch) : HAL_TIMEx_OCN_Start_DMA(&r.h, ch, r.buf, len);
}
HAL_StatusTypeDef tnStop(TnRig &r, bool pwm, int mode, uint32_t ch) {
	if (pwm) return mode == 0 ? HAL_TIMEx_PWMN_Stop(&r.h, ch) : mode == 1 ? HAL_TIMEx_PWMN_Stop_IT(&r.h, ch) : HAL_TIMEx_PWMN_Stop_DMA(&r.h, ch);
	return mode == 0 ? HAL_TIMEx_OCN_Stop(&r.h, ch) : mode == 1 ? HAL_TIMEx_OCN_Stop_IT(&r.h, ch) : HAL_TIMEx_OCN_Stop_DMA(&r.h, ch);
}

#define TN_FOR_ALL_VARIANTS(pwmVar, modeVar) \
	for (int pwmVar##i = 0; pwmVar##i < 2; ++pwmVar##i) \
		for (int modeVar = 0; modeVar < 3; ++modeVar) \
			for (bool pwmVar = (pwmVar##i == 1), once = true; once; once = false)

} // namespace

/* ---------------------------------------------------------------- TN1: Start */

TEST_CASE_FIXTURE(TimFixture, "TN1 the complementary start of channels 1-3 sets only that CCxNE, the main output and the counter enable (OCN and PWMN, poll)") {
	for (unsigned tim : tnTimers)
		for (unsigned pwmI = 0; pwmI < 2; ++pwmI)
			for (unsigned c = 1; c <= 3; ++c) {
				CAPTURE(tim);
				CAPTURE(pwmI);
				CAPTURE(c);
				TnRig r(tim);
				REQUIRE(r.cen() == false);
				REQUIRE(tnStart(r, pwmI == 1, 0, tnChannel(c)) == HAL_OK);
				CHECK(r.t->CCER == tnCcxne(c));
				CHECK(r.moe());
				CHECK(r.cen());
				CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_BUSY);
			}
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "TN1 the complementary start leaves the non complementary channel states and the other N states READY") {
	for (unsigned tim : tnTimers)
		for (unsigned c = 1; c <= 3; ++c) {
			CAPTURE(tim);
			CAPTURE(c);
			TnRig r(tim);
			REQUIRE(HAL_TIMEx_PWMN_Start(&r.h, tnChannel(c)) == HAL_OK);
			for (unsigned o = 1; o <= 3; ++o) {
				CAPTURE(o);
				CHECK(r.state(o) == HAL_TIM_CHANNEL_STATE_READY);
				if (o != c) CHECK(r.nState(o) == HAL_TIM_CHANNEL_STATE_READY);
			}
		}
}

TEST_CASE_FIXTURE(TimFixture, "TN1 the interrupt start enables the capture / compare interrupt of the channel and the break interrupt, not the others (OCN and PWMN)") {
	for (unsigned tim : tnTimers)
		for (unsigned pwmI = 0; pwmI < 2; ++pwmI)
			for (unsigned c = 1; c <= 3; ++c) {
				CAPTURE(tim);
				CAPTURE(pwmI);
				CAPTURE(c);
				TnRig r(tim);
				REQUIRE(tnStart(r, pwmI == 1, 1, tnChannel(c)) == HAL_OK);
				CHECK((r.t->DIER & (tnDIER_CCIE_ALL | tnDIER_BIE)) == (tnCcie(c) | tnDIER_BIE));
				CHECK(r.t->CCER == tnCcxne(c));
				CHECK(r.moe());
				CHECK(r.cen());
				CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_BUSY);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TN1 the polling start touches no interrupt or DMA request bit of DIER") {
	for (unsigned pwmI = 0; pwmI < 2; ++pwmI)
		for (unsigned c = 1; c <= 3; ++c) {
			CAPTURE(pwmI);
			CAPTURE(c);
			TnRig r(1);
			REQUIRE(tnStart(r, pwmI == 1, 0, tnChannel(c)) == HAL_OK);
			CHECK((r.t->DIER & (tnDIER_CCIE_ALL | tnDIER_BIE | tnDIER_CCDE_ALL)) == 0u);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TN1 starting two complementary channels keeps both CCxNE bits set") {
	for (unsigned tim : tnTimers) {
		TnRig r(tim);
		CAPTURE(tim);
		REQUIRE(HAL_TIMEx_OCN_Start(&r.h, TIM_CHANNEL_1) == HAL_OK);
		REQUIRE(HAL_TIMEx_PWMN_Start(&r.h, TIM_CHANNEL_3) == HAL_OK);
		CHECK(r.t->CCER == (tnCcxne(1) | tnCcxne(3)));
		CHECK(r.nState(1) == HAL_TIM_CHANNEL_STATE_BUSY);
		CHECK(r.nState(2) == HAL_TIM_CHANNEL_STATE_READY);
		CHECK(r.nState(3) == HAL_TIM_CHANNEL_STATE_BUSY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TN1 starting a complementary channel that is already BUSY returns HAL_ERROR and changes no register (poll and IT, OCN and PWMN)") {
	for (unsigned pwmI = 0; pwmI < 2; ++pwmI)
		for (int mode = 0; mode < 2; ++mode)
			for (unsigned c = 1; c <= 3; ++c) {
				CAPTURE(pwmI);
				CAPTURE(mode);
				CAPTURE(c);
				TnRig r(1);
				REQUIRE(tnStart(r, pwmI == 1, mode, tnChannel(c)) == HAL_OK);
				const uint32_t ccer = r.t->CCER, dier = r.t->DIER, cr1 = r.t->CR1, bdtr = r.t->BDTR;
				CHECK(tnStart(r, pwmI == 1, mode, tnChannel(c)) == HAL_ERROR);
				CHECK(r.t->CCER == ccer);
				CHECK(r.t->DIER == dier);
				CHECK(r.t->CR1 == cr1);
				CHECK(r.t->BDTR == bdtr);
				CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_BUSY);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TN1 starting a complementary channel whose state is RESET returns HAL_ERROR and enables nothing (poll and IT)") {
	for (unsigned pwmI = 0; pwmI < 2; ++pwmI)
		for (int mode = 0; mode < 2; ++mode) {
			CAPTURE(pwmI);
			CAPTURE(mode);
			TnRig r(8);
			r.h.ChannelNState[1] = HAL_TIM_CHANNEL_STATE_RESET;
			CHECK(tnStart(r, pwmI == 1, mode, TIM_CHANNEL_2) == HAL_ERROR);
			CHECK(r.t->CCER == 0u);
			CHECK(r.cen() == false);
			CHECK(r.moe() == false);
			CHECK(r.nState(2) == HAL_TIM_CHANNEL_STATE_RESET);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TN1 a complementary start does not set CEN when the slave mode is trigger mode (SMS = 110), for every start function") {
	TN_FOR_ALL_VARIANTS(pwm, mode) {
		for (unsigned c = 1; c <= 3; ++c) {
			CAPTURE(pwm);
			CAPTURE(mode);
			CAPTURE(c);
			TnRig r(1);
			r.t->SMCR = 6u; // RM0090 17.4.3: SMS = 110 trigger mode
			REQUIRE(tnStart(r, pwm, mode, tnChannel(c)) == HAL_OK);
			CHECK(r.t->CCER == tnCcxne(c));
			CHECK(r.moe());
			CHECK(r.cen() == false);
		}
	}
}

TEST_CASE_FIXTURE(TimFixture, "TN1 a complementary start sets CEN in the other slave modes (reset 100, gated 101, external clock 111, disabled 000)") {
	const uint32_t sms[4] = {4u, 5u, 7u, 0u};
	for (uint32_t s : sms) {
		CAPTURE(s);
		TnRig r(8);
		r.t->SMCR = s;
		REQUIRE(HAL_TIMEx_PWMN_Start(&r.h, TIM_CHANNEL_1) == HAL_OK);
		CHECK(r.cen());
	}
}

TEST_CASE_FIXTURE(TimFixture, "TN1 the start of OnePulseN enables CCxNE of the output channel and CCxE of the other channel, sets MOE, states BUSY and not CEN") {
	for (unsigned tim : tnTimers)
		for (unsigned out = 1; out <= 2; ++out) {
			const unsigned in = out == 1 ? 2 : 1;
			CAPTURE(tim);
			CAPTURE(out);
			TnRig r(tim);
			REQUIRE(HAL_TIMEx_OnePulseN_Start(&r.h, tnChannel(out)) == HAL_OK);
			CHECK(r.t->CCER == (tnCcxne(out) | tnCcxe(in)));
			CHECK(r.moe());
			CHECK(r.cen() == false); // the one pulse is started by the trigger (RM0090 17.3.13)
			for (unsigned c = 1; c <= 2; ++c) {
				CHECK(r.state(c) == HAL_TIM_CHANNEL_STATE_BUSY);
				CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_BUSY);
			}
			CHECK(r.state(3) == HAL_TIM_CHANNEL_STATE_READY);
			CHECK(r.nState(3) == HAL_TIM_CHANNEL_STATE_READY);
			CHECK((r.t->DIER & (tnDIER_CCIE_ALL | tnDIER_BIE)) == 0u);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TN1 the interrupt start of OnePulseN enables the CC1 and CC2 interrupts and the same outputs as the polling start") {
	for (unsigned tim : tnTimers)
		for (unsigned out = 1; out <= 2; ++out) {
			const unsigned in = out == 1 ? 2 : 1;
			CAPTURE(tim);
			CAPTURE(out);
			TnRig r(tim);
			REQUIRE(HAL_TIMEx_OnePulseN_Start_IT(&r.h, tnChannel(out)) == HAL_OK);
			CHECK((r.t->DIER & (tnDIER_CCIE_ALL | tnDIER_BIE)) == (tnCcie(1) | tnCcie(2)));
			CHECK(r.t->CCER == (tnCcxne(out) | tnCcxe(in)));
			CHECK(r.moe());
			CHECK(r.cen() == false);
			for (unsigned c = 1; c <= 2; ++c) {
				CHECK(r.state(c) == HAL_TIM_CHANNEL_STATE_BUSY);
				CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_BUSY);
			}
		}
}

TEST_CASE_FIXTURE(TimFixture, "TN1 OnePulseN start returns HAL_ERROR and changes nothing when any of the four channel states of channels 1 and 2 is not READY") {
	for (int it = 0; it < 2; ++it)
		for (unsigned which = 0; which < 4; ++which) {
			CAPTURE(it);
			CAPTURE(which);
			TnRig r(1);
			volatile HAL_TIM_ChannelStateTypeDef *const states[4] = {&r.h.ChannelState[0], &r.h.ChannelState[1], &r.h.ChannelNState[0], &r.h.ChannelNState[1]};
			*states[which] = HAL_TIM_CHANNEL_STATE_BUSY;
			const HAL_StatusTypeDef rc = it == 0 ? HAL_TIMEx_OnePulseN_Start(&r.h, TIM_CHANNEL_1) : HAL_TIMEx_OnePulseN_Start_IT(&r.h, TIM_CHANNEL_1);
			CHECK(rc == HAL_ERROR);
			CHECK(r.t->CCER == 0u);
			CHECK(r.t->DIER == 0u);
			CHECK(r.moe() == false);
			for (unsigned k = 0; k < 4; ++k)
				CHECK(*states[k] == (k == which ? HAL_TIM_CHANNEL_STATE_BUSY : HAL_TIM_CHANNEL_STATE_READY));
		}
}

TEST_CASE_FIXTURE(TimFixture, "TN1 OnePulseN start with a second start while BUSY returns HAL_ERROR") {
	TnRig r(8);
	REQUIRE(HAL_TIMEx_OnePulseN_Start(&r.h, TIM_CHANNEL_2) == HAL_OK);
	CHECK(HAL_TIMEx_OnePulseN_Start(&r.h, TIM_CHANNEL_2) == HAL_ERROR);
	CHECK(HAL_TIMEx_OnePulseN_Start_IT(&r.h, TIM_CHANNEL_2) == HAL_ERROR);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TN1 OCN_Start with channel 4 returns HAL_OK, enables none of CC1NE-CC3NE and leaves ChannelNState[3] BUSY") {
	// NEEDS REVIEW 335/a
	TnRig r(1);
	CHECK(HAL_TIMEx_OCN_Start(&r.h, TIM_CHANNEL_4) == HAL_OK);
	CHECK((r.t->CCER & (tnCcxne(1) | tnCcxne(2) | tnCcxne(3))) == 0u);
	CHECK(r.h.ChannelNState[3] == HAL_TIM_CHANNEL_STATE_BUSY);
	HalFixture::clearFaults();
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TN1 the interrupt and DMA starts with channel 4 return HAL_ERROR, enable nothing and leave ChannelNState[3] BUSY") {
	// NEEDS REVIEW 335/a
	for (int pwmI = 0; pwmI < 2; ++pwmI)
		for (int mode = 1; mode < 3; ++mode) {
			CAPTURE(pwmI);
			CAPTURE(mode);
			TnRig r(8);
			CHECK(tnStart(r, pwmI == 1, mode, TIM_CHANNEL_4) == HAL_ERROR);
			CHECK(r.t->CCER == 0u);
			CHECK(r.t->DIER == 0u);
			CHECK(r.cen() == false);
			CHECK(r.h.ChannelNState[3] == HAL_TIM_CHANNEL_STATE_BUSY);
		}
	HalFixture::clearFaults();
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TN1 the polling stop with channel 4 returns HAL_OK and sets ChannelNState[3] READY, the IT and DMA stops return HAL_ERROR") {
	// NEEDS REVIEW 335/a
	TnRig r(1);
	r.h.ChannelNState[3] = HAL_TIM_CHANNEL_STATE_BUSY;
	CHECK(HAL_TIMEx_PWMN_Stop(&r.h, TIM_CHANNEL_4) == HAL_OK);
	CHECK(r.h.ChannelNState[3] == HAL_TIM_CHANNEL_STATE_READY);
	r.h.ChannelNState[3] = HAL_TIM_CHANNEL_STATE_BUSY;
	CHECK(HAL_TIMEx_PWMN_Stop_IT(&r.h, TIM_CHANNEL_4) == HAL_ERROR);
	CHECK(HAL_TIMEx_OCN_Stop_IT(&r.h, TIM_CHANNEL_4) == HAL_ERROR);
	CHECK(HAL_TIMEx_OCN_Stop_DMA(&r.h, TIM_CHANNEL_4) == HAL_ERROR);
	CHECK(HAL_TIMEx_PWMN_Stop_DMA(&r.h, TIM_CHANNEL_4) == HAL_ERROR);
	CHECK(r.h.ChannelNState[3] == HAL_TIM_CHANNEL_STATE_BUSY);
	HalFixture::clearFaults();
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TN1 OCN_Start on TIM3 (no complementary outputs) runs the same code and the model faults on the missing BDTR") {
	// NEEDS REVIEW 335/b
	TIM_HandleTypeDef h{};
	h.Instance = TIM3;
	h.Init.Period = 99;
	REQUIRE(HAL_TIM_PWM_Init(&h) == HAL_OK);
	CHECK(HAL_TIMEx_OCN_Start(&h, TIM_CHANNEL_1) == HAL_OK);
	CHECK(h.ChannelNState[0] == HAL_TIM_CHANNEL_STATE_BUSY);
	CHECK((TIM3->CR1 & tnCR1_CEN) == tnCR1_CEN);
	CHECK(HalFixture::faults() > 0u);
	HalFixture::clearFaults();
}

/* ---------------------------------------------------------------- TN2: Stop */

TEST_CASE_FIXTURE(TimFixture, "TN2 the stop of the only active channel clears CCxNE, MOE and CEN and sets the N state READY (every start / stop family)") {
	TN_FOR_ALL_VARIANTS(pwm, mode) {
		for (unsigned tim : tnTimers)
			for (unsigned c = 1; c <= 3; ++c) {
				CAPTURE(pwm);
				CAPTURE(mode);
				CAPTURE(tim);
				CAPTURE(c);
				TnRig r(tim);
				REQUIRE(tnStart(r, pwm, mode, tnChannel(c)) == HAL_OK);
				REQUIRE(r.cen());
				REQUIRE(tnStop(r, pwm, mode, tnChannel(c)) == HAL_OK);
				CHECK(r.t->CCER == 0u);
				CHECK(r.moe() == false);
				CHECK(r.cen() == false);
				CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_READY);
			}
	}
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "TN2 the stop of one of two complementary channels keeps the other CCxNE, MOE and CEN") {
	TN_FOR_ALL_VARIANTS(pwm, mode) {
		CAPTURE(pwm);
		CAPTURE(mode);
		TnRig r(1);
		REQUIRE(tnStart(r, pwm, mode, TIM_CHANNEL_1) == HAL_OK);
		REQUIRE(tnStart(r, pwm, mode, TIM_CHANNEL_2) == HAL_OK);
		REQUIRE(tnStop(r, pwm, mode, TIM_CHANNEL_1) == HAL_OK);
		CHECK(r.t->CCER == tnCcxne(2));
		CHECK(r.moe());
		CHECK(r.cen());
		CHECK(r.nState(1) == HAL_TIM_CHANNEL_STATE_READY);
		CHECK(r.nState(2) == HAL_TIM_CHANNEL_STATE_BUSY);
		REQUIRE(tnStop(r, pwm, mode, TIM_CHANNEL_2) == HAL_OK);
		CHECK(r.t->CCER == 0u);
		CHECK(r.moe() == false);
		CHECK(r.cen() == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TN2 the stop keeps MOE and CEN while a non complementary channel is still enabled") {
	TN_FOR_ALL_VARIANTS(pwm, mode) {
		for (unsigned c = 1; c <= 3; ++c) {
			CAPTURE(pwm);
			CAPTURE(mode);
			CAPTURE(c);
			TnRig r(8);
			REQUIRE(tnStart(r, pwm, mode, tnChannel(c)) == HAL_OK);
			const unsigned other = (c % 3) + 1;
			r.t->CCER |= tnCcxe(other); // a regular channel enabled by the application
			REQUIRE(tnStop(r, pwm, mode, tnChannel(c)) == HAL_OK);
			CHECK(r.t->CCER == tnCcxe(other));
			CHECK(r.moe());
			CHECK(r.cen());
			CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_READY);
		}
	}
}

TEST_CASE_FIXTURE(TimFixture, "TN2 the interrupt stop clears the capture / compare interrupt of that channel only") {
	for (int pwmI = 0; pwmI < 2; ++pwmI)
		for (unsigned c = 1; c <= 3; ++c) {
			CAPTURE(pwmI);
			CAPTURE(c);
			TnRig r(1);
			const unsigned other = (c % 3) + 1;
			REQUIRE(tnStart(r, pwmI == 1, 1, tnChannel(c)) == HAL_OK);
			REQUIRE(tnStart(r, pwmI == 1, 1, tnChannel(other)) == HAL_OK);
			REQUIRE(tnStop(r, pwmI == 1, 1, tnChannel(c)) == HAL_OK);
			CHECK((r.t->DIER & tnDIER_CCIE_ALL) == tnCcie(other));
		}
}

TEST_CASE_FIXTURE(TimFixture, "TN2 the interrupt stop clears the break interrupt only when no complementary channel is left enabled") {
	for (int pwmI = 0; pwmI < 2; ++pwmI) {
		CAPTURE(pwmI);
		TnRig r(8);
		REQUIRE(tnStart(r, pwmI == 1, 1, TIM_CHANNEL_1) == HAL_OK);
		REQUIRE(tnStart(r, pwmI == 1, 1, TIM_CHANNEL_3) == HAL_OK);
		REQUIRE(tnStop(r, pwmI == 1, 1, TIM_CHANNEL_1) == HAL_OK);
		CHECK((r.t->DIER & tnDIER_BIE) == tnDIER_BIE);
		REQUIRE(tnStop(r, pwmI == 1, 1, TIM_CHANNEL_3) == HAL_OK);
		CHECK((r.t->DIER & tnDIER_BIE) == 0u);
		CHECK((r.t->DIER & tnDIER_CCIE_ALL) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TN2 the polling stop leaves the DIER bits as they were") {
	TnRig r(1);
	r.t->DIER = tnDIER_BIE | tnCcie(1);
	REQUIRE(HAL_TIMEx_OCN_Start(&r.h, TIM_CHANNEL_1) == HAL_OK);
	REQUIRE(HAL_TIMEx_OCN_Stop(&r.h, TIM_CHANNEL_1) == HAL_OK);
	CHECK((r.t->DIER & (tnDIER_BIE | tnCcie(1))) == (tnDIER_BIE | tnCcie(1)));
}

TEST_CASE_FIXTURE(TimFixture, "TN2 the stop of a channel that was never started returns HAL_OK and sets its N state READY (poll)") {
	for (int pwmI = 0; pwmI < 2; ++pwmI) {
		TnRig r(1);
		r.h.ChannelNState[0] = HAL_TIM_CHANNEL_STATE_BUSY;
		CHECK(tnStop(r, pwmI == 1, 0, TIM_CHANNEL_1) == HAL_OK);
		CHECK(r.nState(1) == HAL_TIM_CHANNEL_STATE_READY);
		CHECK(r.t->CCER == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TN2 a channel can be restarted after its stop (every family)") {
	TN_FOR_ALL_VARIANTS(pwm, mode) {
		CAPTURE(pwm);
		CAPTURE(mode);
		TnRig r(1);
		REQUIRE(tnStart(r, pwm, mode, TIM_CHANNEL_2) == HAL_OK);
		REQUIRE(tnStop(r, pwm, mode, TIM_CHANNEL_2) == HAL_OK);
		if (mode == 2) {
			// the stream was aborted: HAL_DMA_Abort_IT leaves it in the ABORT state until its interrupt, which the test delivers
			hostDmaComplete(1, r.stream(2));
			HAL_DMA_IRQHandler(&r.dma[1]);
			r.dma[1].State = HAL_DMA_STATE_READY;
		}
		CHECK(tnStart(r, pwm, mode, TIM_CHANNEL_2) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TN2 the DMA stop clears the DMA request of that channel and stops its stream") {
	for (int pwmI = 0; pwmI < 2; ++pwmI)
		for (unsigned tim : tnTimers)
			for (unsigned c = 1; c <= 3; ++c) {
				CAPTURE(pwmI);
				CAPTURE(tim);
				CAPTURE(c);
				TnRig r(tim);
				REQUIRE(tnStart(r, pwmI == 1, 2, tnChannel(c)) == HAL_OK);
				REQUIRE((r.t->DIER & tnCcde(c)) == tnCcde(c));
				REQUIRE((tnStream(r.stream(c))->CR & tnDMA_EN) == tnDMA_EN);
				REQUIRE(tnStop(r, pwmI == 1, 2, tnChannel(c)) == HAL_OK);
				CHECK((r.t->DIER & tnDIER_CCDE_ALL) == 0u);
				CHECK((tnStream(r.stream(c))->CR & tnDMA_EN) == 0u);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TN2 the DMA stop of one channel leaves the DMA request and stream of another channel running") {
	TnRig r(1);
	REQUIRE(HAL_TIMEx_PWMN_Start_DMA(&r.h, TIM_CHANNEL_1, r.buf, 4) == HAL_OK);
	REQUIRE(HAL_TIMEx_PWMN_Start_DMA(&r.h, TIM_CHANNEL_2, r.buf, 4) == HAL_OK);
	REQUIRE(HAL_TIMEx_PWMN_Stop_DMA(&r.h, TIM_CHANNEL_1) == HAL_OK);
	CHECK((r.t->DIER & tnDIER_CCDE_ALL) == tnCcde(2));
	CHECK((tnStream(r.stream(2))->CR & tnDMA_EN) == tnDMA_EN);
	CHECK(r.moe());
	CHECK(r.cen());
}

TEST_CASE_FIXTURE(TimFixture, "TN2 OnePulseN stop clears CCxNE and CCxE of the pair, MOE and CEN and sets the four states READY (poll and IT)") {
	for (unsigned tim : tnTimers)
		for (int it = 0; it < 2; ++it)
			for (unsigned out = 1; out <= 2; ++out) {
				CAPTURE(tim);
				CAPTURE(it);
				CAPTURE(out);
				TnRig r(tim);
				const uint32_t chOut = tnChannel(out);
				REQUIRE((it == 0 ? HAL_TIMEx_OnePulseN_Start(&r.h, chOut) : HAL_TIMEx_OnePulseN_Start_IT(&r.h, chOut)) == HAL_OK);
				r.t->CR1 |= tnCR1_CEN; // the trigger started the counter
				REQUIRE((it == 0 ? HAL_TIMEx_OnePulseN_Stop(&r.h, chOut) : HAL_TIMEx_OnePulseN_Stop_IT(&r.h, chOut)) == HAL_OK);
				CHECK(r.t->CCER == 0u);
				CHECK(r.moe() == false);
				CHECK(r.cen() == false);
				CHECK((r.t->DIER & tnDIER_CCIE_ALL) == 0u);
				for (unsigned c = 1; c <= 2; ++c) {
					CHECK(r.state(c) == HAL_TIM_CHANNEL_STATE_READY);
					CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_READY);
				}
			}
}

TEST_CASE_FIXTURE(TimFixture, "TN2 OnePulseN stop keeps MOE and CEN while channel 3 is still enabled") {
	for (int it = 0; it < 2; ++it) {
		CAPTURE(it);
		TnRig r(1);
		REQUIRE(HAL_TIMEx_OnePulseN_Start(&r.h, TIM_CHANNEL_1) == HAL_OK);
		r.t->CR1 |= tnCR1_CEN;
		r.t->CCER |= tnCcxe(3);
		REQUIRE((it == 0 ? HAL_TIMEx_OnePulseN_Stop(&r.h, TIM_CHANNEL_1) : HAL_TIMEx_OnePulseN_Stop_IT(&r.h, TIM_CHANNEL_1)) == HAL_OK);
		CHECK(r.t->CCER == tnCcxe(3));
		CHECK(r.moe());
		CHECK(r.cen());
	}
}

TEST_CASE_FIXTURE(TimFixture, "TN2 OnePulseN stop clears the CC1 and CC2 interrupts and keeps the CC3 interrupt") {
	TnRig r(8);
	r.t->DIER |= tnCcie(3);
	REQUIRE(HAL_TIMEx_OnePulseN_Start_IT(&r.h, TIM_CHANNEL_2) == HAL_OK);
	REQUIRE(HAL_TIMEx_OnePulseN_Stop_IT(&r.h, TIM_CHANNEL_2) == HAL_OK);
	CHECK((r.t->DIER & tnDIER_CCIE_ALL) == tnCcie(3));
}

TEST_CASE_FIXTURE(TimFixture, "TN2 OnePulseN can be started again after its stop") {
	TnRig r(1);
	REQUIRE(HAL_TIMEx_OnePulseN_Start(&r.h, TIM_CHANNEL_1) == HAL_OK);
	REQUIRE(HAL_TIMEx_OnePulseN_Stop(&r.h, TIM_CHANNEL_1) == HAL_OK);
	CHECK(HAL_TIMEx_OnePulseN_Start_IT(&r.h, TIM_CHANNEL_2) == HAL_OK);
}

/* ---------------------------------------------------------------- TN3: DMA */

TEST_CASE_FIXTURE(TimFixture, "TN3 Start_DMA programs the stream with the buffer, the CCRx address and Length items, and enables the stream (OCN and PWMN)") {
	for (int pwmI = 0; pwmI < 2; ++pwmI)
		for (unsigned tim : tnTimers)
			for (unsigned c = 1; c <= 3; ++c) {
				CAPTURE(pwmI);
				CAPTURE(tim);
				CAPTURE(c);
				TnRig r(tim);
				REQUIRE(tnStart(r, pwmI == 1, 2, tnChannel(c), 5) == HAL_OK);
				DMA_Stream_TypeDef *s = tnStream(r.stream(c));
				CHECK(s->NDTR == 5u);
				CHECK(s->PAR == tnPeriphAddr[r.idx][c - 1]);
				CHECK(s->M0AR == r.addr());
				CHECK((s->CR & tnDMA_EN) == tnDMA_EN);
				CHECK((s->CR & (tnDMA_TCIE | tnDMA_HTIE | tnDMA_TEIE)) == (tnDMA_TCIE | tnDMA_HTIE | tnDMA_TEIE));
			}
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "TN3 Start_DMA with Length 1 and Length 65535 programs NDTR with that many items") {
	const uint16_t lens[2] = {1, 65535};
	for (uint16_t len : lens) {
		CAPTURE(len);
		TnRig r(1);
		REQUIRE(HAL_TIMEx_OCN_Start_DMA(&r.h, TIM_CHANNEL_3, r.buf, len) == HAL_OK);
		CHECK(tnStream(r.stream(3))->NDTR == len);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TN3 Start_DMA enables the DMA request of the channel, CCxNE, MOE and CEN, state BUSY (OCN and PWMN)") {
	for (int pwmI = 0; pwmI < 2; ++pwmI)
		for (unsigned tim : tnTimers)
			for (unsigned c = 1; c <= 3; ++c) {
				CAPTURE(pwmI);
				CAPTURE(tim);
				CAPTURE(c);
				TnRig r(tim);
				REQUIRE(tnStart(r, pwmI == 1, 2, tnChannel(c)) == HAL_OK);
				CHECK((r.t->DIER & (tnDIER_CCDE_ALL | tnDIER_CCIE_ALL | tnDIER_BIE)) == tnCcde(c));
				CHECK(r.t->CCER == tnCcxne(c));
				CHECK(r.moe());
				CHECK(r.cen());
				CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_BUSY);
				CHECK(r.state(c) == HAL_TIM_CHANNEL_STATE_READY);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TN3 Start_DMA returns HAL_ERROR and starts nothing for a NULL buffer or Length 0") {
	for (int pwmI = 0; pwmI < 2; ++pwmI)
		for (unsigned c = 1; c <= 3; ++c)
			for (int nullBuf = 0; nullBuf < 2; ++nullBuf) {
				CAPTURE(pwmI);
				CAPTURE(c);
				CAPTURE(nullBuf);
				TnRig r(1);
				HAL_StatusTypeDef rc;
				if (pwmI == 1)
					rc = HAL_TIMEx_PWMN_Start_DMA(&r.h, tnChannel(c), nullBuf ? nullptr : r.buf, nullBuf ? 4 : 0);
				else
					rc = HAL_TIMEx_OCN_Start_DMA(&r.h, tnChannel(c), nullBuf ? nullptr : r.buf, nullBuf ? 4 : 0);
				CHECK(rc == HAL_ERROR);
				CHECK((tnStream(r.stream(c))->CR & tnDMA_EN) == 0u);
				CHECK(r.t->CCER == 0u);
				CHECK(r.t->DIER == 0u);
				CHECK(r.cen() == false);
				CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_READY);
			}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TN3 Start_DMA of a BUSY channel returns HAL_BUSY and of a RESET channel HAL_ERROR, without touching the stream") {
	// NEEDS REVIEW 335/c
	for (int pwmI = 0; pwmI < 2; ++pwmI) {
		CAPTURE(pwmI);
		TnRig r(8);
		r.h.ChannelNState[0] = HAL_TIM_CHANNEL_STATE_BUSY;
		CHECK(tnStart(r, pwmI == 1, 2, TIM_CHANNEL_1) == HAL_BUSY);
		r.h.ChannelNState[0] = HAL_TIM_CHANNEL_STATE_RESET;
		CHECK(tnStart(r, pwmI == 1, 2, TIM_CHANNEL_1) == HAL_ERROR);
		CHECK((tnStream(r.stream(1))->CR & tnDMA_EN) == 0u);
		CHECK(r.t->CCER == 0u);
		CHECK(r.t->DIER == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TN3 Start_DMA returns HAL_ERROR and enables no request, output or counter when the DMA stream cannot start") {
	for (int pwmI = 0; pwmI < 2; ++pwmI)
		for (unsigned tim : tnTimers)
			for (unsigned c = 1; c <= 3; ++c) {
				CAPTURE(pwmI);
				CAPTURE(tim);
				CAPTURE(c);
				TnRig r(tim);
				r.dma[c - 1].State = HAL_DMA_STATE_BUSY; // HAL_DMA_Start_IT then refuses
				CHECK(tnStart(r, pwmI == 1, 2, tnChannel(c)) == HAL_ERROR);
				CHECK((r.t->DIER & tnDIER_CCDE_ALL) == 0u);
				CHECK(r.t->CCER == 0u);
				CHECK(r.moe() == false);
				CHECK(r.cen() == false);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TN3 Start_DMA that fails because the DMA stream cannot start leaves the channel N state READY" * KNOWN_ISSUE("candidate vendor bug V112 (arguable: UM1725 gives no state rule for a failed DMA start): the N state stays BUSY after the HAL_ERROR return, so every later start of that channel returns HAL_BUSY (same pattern as V101 of the SPI driver)")) {
	for (int pwmI = 0; pwmI < 2; ++pwmI)
		for (unsigned c = 1; c <= 3; ++c) {
			CAPTURE(pwmI);
			CAPTURE(c);
			TnRig r(1);
			r.dma[c - 1].State = HAL_DMA_STATE_BUSY;
			REQUIRE(tnStart(r, pwmI == 1, 2, tnChannel(c)) == HAL_ERROR);
			CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_READY);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TN3 DMA transfer complete of a normal mode stream runs the pulse finished callback with the channel active and sets the N state READY") {
	for (int pwmI = 0; pwmI < 2; ++pwmI)
		for (unsigned tim : tnTimers)
			for (unsigned c = 1; c <= 3; ++c) {
				CAPTURE(pwmI);
				CAPTURE(tim);
				CAPTURE(c);
				TnRig r(tim);
				REQUIRE(tnStart(r, pwmI == 1, 2, tnChannel(c), 4) == HAL_OK);
				hostDmaComplete(1, r.stream(c));
				HAL_DMA_IRQHandler(&r.dma[c - 1]);
				CHECK(timCounts.pwmPulse == 1u);
				CHECK(timCounts.ocDelay == 0u);
				CHECK(timCounts.last == &r.h);
				CHECK(timCounts.channelAtLast == tnActive(c));
				CHECK(r.h.Channel == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
				CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_READY);
				CHECK(r.state(c) == HAL_TIM_CHANNEL_STATE_READY);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TN3 DMA transfer complete of a circular stream runs the callback and keeps the N state BUSY") {
	for (unsigned c = 1; c <= 3; ++c) {
		CAPTURE(c);
		TnRig r(1, DMA_CIRCULAR);
		REQUIRE(HAL_TIMEx_PWMN_Start_DMA(&r.h, tnChannel(c), r.buf, 4) == HAL_OK);
		hostDmaComplete(1, r.stream(c));
		HAL_DMA_IRQHandler(&r.dma[c - 1]);
		CHECK(timCounts.pwmPulse == 1u);
		CHECK(timCounts.channelAtLast == tnActive(c));
		CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_BUSY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TN3 DMA transfer complete of one channel does not change the N state of the other channels") {
	TnRig r(8);
	REQUIRE(HAL_TIMEx_OCN_Start_DMA(&r.h, TIM_CHANNEL_1, r.buf, 4) == HAL_OK);
	REQUIRE(HAL_TIMEx_OCN_Start_DMA(&r.h, TIM_CHANNEL_2, r.buf, 4) == HAL_OK);
	hostDmaComplete(1, r.stream(1));
	HAL_DMA_IRQHandler(&r.dma[0]);
	CHECK(r.nState(1) == HAL_TIM_CHANNEL_STATE_READY);
	CHECK(r.nState(2) == HAL_TIM_CHANNEL_STATE_BUSY);
	CHECK(r.nState(3) == HAL_TIM_CHANNEL_STATE_READY);
}

TEST_CASE_FIXTURE(TimFixture, "TN3 DMA half transfer runs the half pulse finished callback and keeps the N state BUSY") {
	for (int pwmI = 0; pwmI < 2; ++pwmI)
		for (unsigned c = 1; c <= 3; ++c) {
			CAPTURE(pwmI);
			CAPTURE(c);
			TnRig r(1);
			REQUIRE(tnStart(r, pwmI == 1, 2, tnChannel(c), 4) == HAL_OK);
			hostDmaStepItems(1, r.stream(c), 2); // half of 4 items: HTIF
			HAL_DMA_IRQHandler(&r.dma[c - 1]);
			CHECK(timCounts.pwmPulseHalf == 1u);
			CHECK(timCounts.pwmPulse == 0u);
			CHECK(timCounts.last == &r.h);
			CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_BUSY);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TN3 a DMA transfer error runs the error callback with the channel active and sets the N state READY") {
	for (int pwmI = 0; pwmI < 2; ++pwmI)
		for (unsigned tim : tnTimers)
			for (unsigned c = 1; c <= 3; ++c) {
				CAPTURE(pwmI);
				CAPTURE(tim);
				CAPTURE(c);
				TnRig r(tim);
				REQUIRE(tnStart(r, pwmI == 1, 2, tnChannel(c), 4) == HAL_OK);
				hostDmaRaise(1, r.stream(c), HOST_DMA_TEIF);
				HAL_DMA_IRQHandler(&r.dma[c - 1]);
				CHECK(timCounts.error == 1u);
				CHECK(timCounts.pwmPulse == 0u);
				CHECK(timCounts.last == &r.h);
				CHECK(timCounts.channelAtLast == tnActive(c));
				CHECK(r.h.Channel == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
				CHECK(r.nState(c) == HAL_TIM_CHANNEL_STATE_READY);
			}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TN3 the DMA completion of an OCN stream calls the pulse finished callback and not the OC delay elapsed callback") {
	// NEEDS REVIEW 335/d
	TnRig r(1);
	REQUIRE(HAL_TIMEx_OCN_Start_DMA(&r.h, TIM_CHANNEL_1, r.buf, 4) == HAL_OK);
	hostDmaComplete(1, r.stream(1));
	HAL_DMA_IRQHandler(&r.dma[0]);
	CHECK(timCounts.pwmPulse == 1u);
	CHECK(timCounts.ocDelay == 0u);
}

/* ---------------------------------------------------------------- non slave instance branch (TIM10: no SMCR, no BDTR) */

TEST_CASE_FIXTURE(TimFixture, "characterization: TN1 the start functions on TIM10 (not a slave instance) set CEN without reading SMCR and return HAL_OK") {
	// NEEDS REVIEW 335/b: UM1725 only defines the N functions for the advanced timers; TIM10 runs the HAL's else branch (CEN set unconditionally).
	// The model faults on the BDTR (MOE) access of the missing register; the faults are cleared and not asserted further.
	for (int mode = 0; mode < 3; ++mode) {
		for (int pwmI = 0; pwmI < 2; ++pwmI) {
			CAPTURE(mode);
			CAPTURE(pwmI);
			hostRegsDisarm();
			hostPeriphResetState();
			hostRegsReset();
			hostRegsArm();
			TIM_HandleTypeDef h{};
			DMA_HandleTypeDef d{};
			uint32_t buf[4] = {1, 2, 3, 4};
			h.Instance = TIM10;
			h.Init.Period = 99;
			REQUIRE(HAL_TIM_PWM_Init(&h) == HAL_OK);
			d.Instance = DMA2_Stream1;
			d.Init.Channel = DMA_CHANNEL_6;
			d.Init.Direction = DMA_MEMORY_TO_PERIPH;
			d.Init.MemInc = DMA_MINC_ENABLE;
			d.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
			d.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
			d.Init.Mode = DMA_NORMAL;
			d.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
			REQUIRE(HAL_DMA_Init(&d) == HAL_OK);
			d.Parent = &h;
			h.hdma[TIM_DMA_ID_CC1] = &d;
			HAL_StatusTypeDef rc;
			if (pwmI)
				rc = mode == 0 ? HAL_TIMEx_PWMN_Start(&h, TIM_CHANNEL_1) : mode == 1 ? HAL_TIMEx_PWMN_Start_IT(&h, TIM_CHANNEL_1) : HAL_TIMEx_PWMN_Start_DMA(&h, TIM_CHANNEL_1, buf, 4);
			else
				rc = mode == 0 ? HAL_TIMEx_OCN_Start(&h, TIM_CHANNEL_1) : mode == 1 ? HAL_TIMEx_OCN_Start_IT(&h, TIM_CHANNEL_1) : HAL_TIMEx_OCN_Start_DMA(&h, TIM_CHANNEL_1, buf, 4);
			CHECK(rc == HAL_OK);
			CHECK((TIM10->CR1 & tnCR1_CEN) == tnCR1_CEN);
			CHECK(h.ChannelNState[0] == HAL_TIM_CHANNEL_STATE_BUSY);
			HalFixture::clearFaults();
		}
	}
}
