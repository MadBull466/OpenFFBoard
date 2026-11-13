/*
 * P12 T6a group TB: time base, output compare and PWM of the real ST HAL timer driver (stm32f4xx_hal_tim.c: HAL_TIM_Base_*, HAL_TIM_OC_*,
 * HAL_TIM_PWM_* Init / DeInit / Start / Stop / Start_IT / Stop_IT / Start_DMA / Stop_DMA, HAL_TIM_OC_ConfigChannel, HAL_TIM_PWM_ConfigChannel, the
 * GetState functions, HAL_TIM_GetChannelState, HAL_TIM_GetActiveChannel, TIM_Base_SetConfig, TIM_OCx_SetConfig, TIM_CCxChannelCmd) against the timer
 * model (fakes/stm32_hal/HalTim.h) and the DMA model (HalDma.h). Requirements TB1-TB4 of Tests/Code_Coverage/claude/charters/hal/P12_T6A_REQUIREMENTS.md;
 * compiled into ffb_hal_tests (F407VG) and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 (chapter "HAL TIM Generic Driver": structure fields, API names, "How to use this driver") and RM0090 Rev 22:
 *  - Table 1 (base addresses), register maps 17.4.21 / 18.4.21 / 19.4.13 / 19.5.9 / 20.4.9 (ARR 0x2C, CCR1 0x34, CCR2 0x38, CCR3 0x3C, CCR4 0x40).
 *  - TIMx_CR1: CEN bit 0, UDIS 1, URS 2, OPM 3, DIR 4, CMS 6:5, ARPE 7, CKD 9:8. EGR.UG: "the counter is cleared if the center-aligned mode is selected
 *    or if DIR = 0 (upcounting), else it takes the auto-reload value if DIR = 1". SMCR.SMS 110 = trigger mode ("only the start of the counter is
 *    controlled": the software does not set CEN). RCR REP[7:0] (TIM1 / TIM8 only). DIER: UIE bit 0, CCxIE bit x, UDE bit 8, CCxDE bit 8 + x.
 *  - TIMx_CCMR1/2 (output): CCxS 1:0 (00 = output), OCxFE bit 2, OCxPE bit 3, OCxM bits 6:4 (000 frozen, 001 active on match, 010 inactive on match, 011 toggle,
 *    100 force inactive, 101 force active, 110 PWM1, 111 PWM2), channel 2 / 4 in bits 15:8. CCER: CCxE, CCxP, CCxNE, CCxNP at bit 4 (x-1) + 0 / 1 / 2 / 3
 *    (channel 4: CC4E bit 12, CC4P bit 13). CR2 (TIM1 / TIM8): OIS1 bit 8, OIS1N 9, OIS2 10, OIS2N 11, OIS3 12, OIS3N 13, OIS4 14. BDTR.MOE bit 15.
 *  - Tables 43 / 44: request mapping (TIM1: UP stream 5 / CC1 1 / CC2 2 / CC3 6 / CC4 3 of DMA2 channel 6; TIM8 channel 7: UP 1, CC1 2, CC2 3, CC3 4, CC4 7;
 *    TIM2 DMA1 channel 3: UP 7, CC1 5, CC2 6, CC3 1, CC4 7; TIM3 channel 5: UP 2, CC1 4, CC2 5, CC3 7, CC4 2; TIM4 channel 2: UP 6, CC1 0, CC2 3, CC3 7;
 *    TIM5 channel 6: UP 6, CC1 2, CC2 4, CC3 0, CC4 1; TIM6 UP stream 1 channel 7; TIM7 UP stream 2 channel 1).
 *  - 10.5.5 DMA_SxCR: EN bit 0, TEIE 2, HTIE 3, TCIE 4. NDTR counts data items.
 * Time never matters (no HAL tick waits). Every file-local symbol sits in the anonymous namespace or is prefixed tb.
 *
 * NEEDS REVIEW proposals:
 *  NEEDS REVIEW 332/a TB1: after Init the update flag UIF is clear (the HAL clears the flag its own UG set; UM1725 silent, RM0090 says UG sets UIF).
 *  NEEDS REVIEW 332/b TB1: DeInit with a capture / compare channel still enabled (CCxE or CCxNE set) leaves CEN set (the HAL macro __HAL_TIM_DISABLE only clears CEN when all
 *     channels are off); same for the Stop functions of Base (UM1725 silent).
 *  NEEDS REVIEW 332/c TB2: Start_DMA of a Base handle / channel in state BUSY returns HAL_BUSY, of one in state RESET HAL_ERROR (UM1725 silent).
 *  NEEDS REVIEW 332/d TB2: an invalid channel (TIM_CHANNEL_ALL): the poll Start returns HAL_OK and leaves ChannelState[3] BUSY with no CCxE set; Start_IT / Start_DMA return
 *     HAL_ERROR with ChannelState[3] left BUSY; the poll Stop returns HAL_OK and sets ChannelState[3] READY; Stop_IT / Stop_DMA return HAL_ERROR.
 *  NEEDS REVIEW 332/e TB2: the DMA completion of an OC stream calls HAL_TIM_PWM_PulseFinishedCallback (not OC_DelayElapsed).
 *  NEEDS REVIEW 332/f TB3: ConfigChannel keeps CCxE of a channel that is running (the HAL disables it only while it writes the registers).
 *  NEEDS REVIEW 332/g TB3: ConfigChannel clears CCxNE of channels 1-3 of TIM1 / TIM8 (a running complementary output is switched off, N state stays BUSY).
 *  NEEDS REVIEW 332/h TB3: ConfigChannel with an invalid channel returns HAL_ERROR, touches no register and releases the lock.
 *  NEEDS REVIEW 332/i TB4: GetChannelState with an invalid channel returns the state of channel 4.
 *
 * Lead decisions needed (model): (1) the HAL reads CCER (reserved on TIM6 / TIM7) in __HAL_TIM_DISABLE (Base Stop / Stop_IT / Stop_DMA / DeInit) and reads and
 *  writes CR2 (reserved on TIM10 / 11 / 13 / 14) in every OC / PWM ConfigChannel: harmless on the silicon (reserved reads 0, writes ignored) but the model faults.
 *  The tests accept exactly those faults (by address) and clear them.
 */
#include "HalDma.h"
#include "HalFixture.h"
#include "HalHostSystem.h"
#include "HalPeripherals.h"
#include "HalTim.h"
#include "TestHelpers.h"
#include "TimTestSupport.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <string>

namespace {

constexpr uint32_t tbCR1_CEN = 1u << 0;
constexpr uint32_t tbBDTR_MOE = 1u << 15;
constexpr uint32_t tbDMA_EN = 1u << 0;
constexpr uint32_t tbDMA_TEIE = 1u << 2;
constexpr uint32_t tbDMA_HTIE = 1u << 3;
constexpr uint32_t tbDMA_TCIE = 1u << 4;
constexpr uint32_t tbSMS_TRIGGER = 6u; // RM0090 SMCR.SMS = 110

struct TbInfo {
	unsigned tim, nch;
	bool adv, slave, csel, ckd, is32;
};
// RM0090 chapters 17-20 (per timer: channels, advanced, slave mode controller, counter mode selection, clock division, 32 bit)
const TbInfo tbAll[14] = {
    {1, 4, true, true, true, true, false},   {2, 4, false, true, true, true, true},   {3, 4, false, true, true, true, false},
    {4, 4, false, true, true, true, false},  {5, 4, false, true, true, true, true},   {6, 0, false, false, false, false, false},
    {7, 0, false, false, false, false, false}, {8, 4, true, true, true, true, false}, {9, 2, false, true, false, true, false},
    {10, 1, false, false, false, true, false}, {11, 1, false, false, false, true, false}, {12, 2, false, true, false, true, false},
    {13, 1, false, false, false, true, false}, {14, 1, false, false, false, true, false}};
const TbInfo &tbI(unsigned tim) { return tbAll[tim - 1]; }

const unsigned tbBase[15] = {0, 0x40010000u, 0x40000000u, 0x40000400u, 0x40000800u, 0x40000C00u, 0x40001000u, 0x40001400u,
                             0x40010400u, 0x40014000u, 0x40014400u, 0x40014800u, 0x40001800u, 0x40001C00u, 0x40002000u};
const unsigned tbAllTims[14] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
const unsigned tbChanTims[12] = {1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13, 14};
const unsigned tbDmaChanTims[6] = {1, 2, 3, 4, 5, 8};
const unsigned tbDmaBaseTims[8] = {1, 2, 3, 4, 5, 6, 7, 8};
const unsigned tbSlaveTims[8] = {1, 2, 3, 4, 5, 8, 9, 12};
const unsigned tbCselTims[6] = {1, 2, 3, 4, 5, 8};

constexpr uint32_t tbCcxe(unsigned c) { return 1u << (4 * (c - 1)); }
constexpr uint32_t tbCcxne(unsigned c) { return 4u << (4 * (c - 1)); }
constexpr uint32_t tbCcie(unsigned c) { return 1u << c; }
constexpr uint32_t tbCcde(unsigned c) { return 1u << (8 + c); }

uint32_t tbCh(unsigned c) {
	const uint32_t v[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
	return v[c - 1];
}
HAL_TIM_ActiveChannel tbActive(unsigned c) {
	const HAL_TIM_ActiveChannel v[4] = {HAL_TIM_ACTIVE_CHANNEL_1, HAL_TIM_ACTIVE_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_3, HAL_TIM_ACTIVE_CHANNEL_4};
	return v[c - 1];
}

struct TbDmaMap {
	uint8_t ctrl, chan, stream[5]; // index 0 = UP, 1..4 = CC1..CC4; 255 = no request
};
const TbDmaMap *tbMapOf(unsigned tim) {
	static const TbDmaMap m[9] = {{0, 0, {255, 255, 255, 255, 255}},
	                              {1, 6, {5, 1, 2, 6, 3}},
	                              {0, 3, {7, 5, 6, 1, 7}},
	                              {0, 5, {2, 4, 5, 7, 2}},
	                              {0, 2, {6, 0, 3, 7, 255}},
	                              {0, 6, {6, 2, 4, 0, 1}},
	                              {0, 7, {1, 255, 255, 255, 255}},
	                              {0, 1, {2, 255, 255, 255, 255}},
	                              {1, 7, {1, 2, 3, 4, 7}}};
	return tim <= 8u ? &m[tim] : nullptr;
}
DMA_Stream_TypeDef *tbStream(unsigned ctrl, unsigned s) {
	DMA_Stream_TypeDef *const a[2][8] = {
	    {DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7},
	    {DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7}};
	return a[ctrl][s];
}

uint32_t tbRd(const volatile void *reg) { return HalFixture::peek(reg); }

/* Everything back to reset between the rigs of one case. */
void tbFresh() {
	CHECK_MESSAGE(HalFixture::faults() == 0u, "unexpected register fault before the next rig");
	hostPeriphResetState();
	hostRegsReset();
	hostRegsArm();
	timSupportReset();
}

/* The faults of the reserved registers the HAL touches (see "Lead decisions needed"): every fault must be at one of the addresses. */
void tbAcceptFaultsAt(std::initializer_list<unsigned> addrs) {
	for (std::size_t i = 0; i < HalFixture::faults(); ++i) {
		const std::string t = hostRegsFaultText(i);
		bool ok = false;
		for (unsigned a : addrs) {
			char b[16];
			std::snprintf(b, sizeof b, "0x%08x", a);
			if (t.find(b) != std::string::npos) ok = true;
		}
		CHECK_MESSAGE(ok, "unexpected register fault: " << t);
	}
	HalFixture::clearFaults();
}
/* CCER of TIM6 / TIM7 is read by the stop macros; the other timers have it. */
void tbAcceptBasicStop(unsigned tim) {
	if (tim == 6u || tim == 7u) tbAcceptFaultsAt({tbBase[tim] + 0x20u});
	else CHECK(HalFixture::faults() == 0u);
}
/* CR2 of TIM10 / 11 / 13 / 14 is read and written by the OC / PWM ConfigChannel. */
void tbAcceptConfigFaults(unsigned tim) {
	if (tim == 10u || tim == 11u || tim == 13u || tim == 14u) tbAcceptFaultsAt({tbBase[tim] + 0x04u});
	else CHECK(HalFixture::faults() == 0u);
}

struct TbRig {
	TIM_HandleTypeDef h;
	DMA_HandleTypeDef dma[5];
	uint32_t buf[16];
	unsigned tim;
	TIM_TypeDef *t;
	const TbDmaMap *m;
	uint32_t period;

	explicit TbRig(unsigned tm, uint32_t dmaMode = DMA_NORMAL) : tim(tm), t(timInstance(tm)), m(tbMapOf(tm)), period(tbI(tm).is32 ? 0x12345678u : 0x1234u) {
		tbFresh();
		std::memset(&h, 0, sizeof h);
		std::memset(dma, 0, sizeof dma);
		std::memset(buf, 0, sizeof buf);
		h.Instance = t;
		h.Init.Prescaler = 0xABu;
		h.Init.CounterMode = TIM_COUNTERMODE_UP;
		h.Init.Period = period;
		h.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		h.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (m != nullptr) {
			for (unsigned id = 0; id < 5; ++id) {
				if (m->stream[id] == 255u) continue;
				dma[id].Instance = tbStream(m->ctrl, m->stream[id]);
				dma[id].Init.Channel = static_cast<uint32_t>(m->chan) << 25;
				dma[id].Init.Direction = DMA_MEMORY_TO_PERIPH;
				dma[id].Init.PeriphInc = DMA_PINC_DISABLE;
				dma[id].Init.MemInc = DMA_MINC_ENABLE;
				dma[id].Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
				dma[id].Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
				dma[id].Init.Mode = dmaMode;
				dma[id].Init.Priority = DMA_PRIORITY_LOW;
				dma[id].Init.FIFOMode = DMA_FIFOMODE_DISABLE;
				dma[id].Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
				dma[id].Init.MemBurst = DMA_MBURST_SINGLE;
				dma[id].Init.PeriphBurst = DMA_PBURST_SINGLE;
				REQUIRE(HAL_DMA_Init(&dma[id]) == HAL_OK);
				dma[id].Parent = &h;
				h.hdma[id] = &dma[id];
			}
		}
		timSupportReset();
	}
	DMA_Stream_TypeDef *stream(unsigned id) const { return tbStream(m->ctrl, m->stream[id]); }
	unsigned ctrl() const { return m->ctrl; }
	uint32_t addr() const { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buf)); }
	HAL_TIM_ChannelStateTypeDef state(unsigned c) const { return h.ChannelState[c - 1]; }
	uint32_t ccer() const { return tbRd(&t->CCER); }
	uint32_t dier() const { return tbRd(&t->DIER); }
	uint32_t cr1() const { return tbRd(&t->CR1); }
	bool cen() const { return (cr1() & tbCR1_CEN) != 0u; }
	bool moe() const { return (tbRd(&t->BDTR) & tbBDTR_MOE) != 0u; }
	uint32_t base() const { return tbBase[tim]; }
};

struct TbSnap {
	uint32_t cr1, ccer, dier, bdtr, ccr1;
	explicit TbSnap(const TbRig &r) : cr1(r.cr1()), ccer(r.ccer()), dier(r.dier()), bdtr(tbRd(&r.t->BDTR)), ccr1(tbRd(&r.t->CCR1)) {}
	bool same(const TbRig &r) const { return cr1 == r.cr1() && ccer == r.ccer() && dier == r.dier() && bdtr == tbRd(&r.t->BDTR); }
};

struct TbInitFam {
	const char *name;
	HAL_StatusTypeDef (*init)(TIM_HandleTypeDef *);
	HAL_StatusTypeDef (*deinit)(TIM_HandleTypeDef *);
	unsigned msp;
};
const TbInitFam tbInitFams[3] = {{"base", HAL_TIM_Base_Init, HAL_TIM_Base_DeInit, 0},
                                 {"OC", HAL_TIM_OC_Init, HAL_TIM_OC_DeInit, 1},
                                 {"PWM", HAL_TIM_PWM_Init, HAL_TIM_PWM_DeInit, 2}};

struct TbFam {
	const char *name;
	HAL_StatusTypeDef (*init)(TIM_HandleTypeDef *);
	HAL_StatusTypeDef (*start)(TIM_HandleTypeDef *, uint32_t);
	HAL_StatusTypeDef (*stop)(TIM_HandleTypeDef *, uint32_t);
	HAL_StatusTypeDef (*startIt)(TIM_HandleTypeDef *, uint32_t);
	HAL_StatusTypeDef (*stopIt)(TIM_HandleTypeDef *, uint32_t);
	HAL_StatusTypeDef (*startDma)(TIM_HandleTypeDef *, uint32_t, const uint32_t *, uint16_t);
	HAL_StatusTypeDef (*stopDma)(TIM_HandleTypeDef *, uint32_t);
	HAL_StatusTypeDef (*config)(TIM_HandleTypeDef *, const TIM_OC_InitTypeDef *, uint32_t);
	HAL_TIM_StateTypeDef (*getState)(const TIM_HandleTypeDef *);
	bool pwm;
};
const TbFam tbFams[2] = {
    {"OC", HAL_TIM_OC_Init, HAL_TIM_OC_Start, HAL_TIM_OC_Stop, HAL_TIM_OC_Start_IT, HAL_TIM_OC_Stop_IT, HAL_TIM_OC_Start_DMA, HAL_TIM_OC_Stop_DMA,
     HAL_TIM_OC_ConfigChannel, HAL_TIM_OC_GetState, false},
    {"PWM", HAL_TIM_PWM_Init, HAL_TIM_PWM_Start, HAL_TIM_PWM_Stop, HAL_TIM_PWM_Start_IT, HAL_TIM_PWM_Stop_IT, HAL_TIM_PWM_Start_DMA, HAL_TIM_PWM_Stop_DMA,
     HAL_TIM_PWM_ConfigChannel, HAL_TIM_PWM_GetState, true}};

HAL_StatusTypeDef tbStart(const TbFam &f, TbRig &r, int mode, unsigned c, uint16_t len = 4) {
	return mode == 0 ? f.start(&r.h, tbCh(c)) : mode == 1 ? f.startIt(&r.h, tbCh(c)) : f.startDma(&r.h, tbCh(c), r.buf, len);
}
HAL_StatusTypeDef tbStop(const TbFam &f, TbRig &r, int mode, unsigned c) {
	return mode == 0 ? f.stop(&r.h, tbCh(c)) : mode == 1 ? f.stopIt(&r.h, tbCh(c)) : f.stopDma(&r.h, tbCh(c));
}

bool tbDmaOk(unsigned tim, unsigned c) { return tbMapOf(tim) != nullptr && tbMapOf(tim)->stream[c] != 255u; }

} // namespace

/* ================================================================ TB1: Init / DeInit */

TEST_CASE_FIXTURE(TimFixture, "TB1 Init of the base, OC and PWM time base with a NULL handle returns HAL_ERROR and calls no MspInit") {
	for (const TbInitFam &f : tbInitFams) {
		CAPTURE(f.name);
		CHECK(f.init(nullptr) == HAL_ERROR);
	}
	for (unsigned i = 0; i < 8; ++i) CHECK(timCounts.mspInit[i] == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "TB1 Init in state RESET calls the MspInit of its kind once and unlocks the handle (all timers, base OC PWM)") {
	for (const TbInitFam &f : tbInitFams)
		for (unsigned tim : tbAllTims) {
			CAPTURE(f.name);
			CAPTURE(tim);
			TbRig r(tim);
			r.h.Lock = HAL_LOCKED;
			REQUIRE(f.init(&r.h) == HAL_OK);
			for (unsigned k = 0; k < 8; ++k) CHECK(timCounts.mspInit[k] == (k == f.msp ? 1u : 0u));
			CHECK(timCounts.last == &r.h);
			CHECK(timCounts.stateAtLast == HAL_TIM_STATE_RESET);
			CHECK(r.h.Lock == HAL_UNLOCKED);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 a second Init in state READY does not call MspInit again but reprograms ARR and PSC") {
	for (const TbInitFam &f : tbInitFams) {
		CAPTURE(f.name);
		TbRig r(3);
		REQUIRE(f.init(&r.h) == HAL_OK);
		r.h.Init.Period = 0x4321u;
		r.h.Init.Prescaler = 0x77u;
		REQUIRE(f.init(&r.h) == HAL_OK);
		CHECK(timCounts.mspInit[f.msp] == 1u);
		CHECK(tbRd(&r.t->ARR) == 0x4321u);
		CHECK(tbRd(&r.t->PSC) == 0x77u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 Init writes Period to ARR and Prescaler to PSC (all timers, TIM2 and TIM5 with 32 bit ARR)") {
	for (const TbInitFam &f : tbInitFams)
		for (unsigned tim : tbAllTims) {
			CAPTURE(f.name);
			CAPTURE(tim);
			TbRig r(tim);
			REQUIRE(f.init(&r.h) == HAL_OK);
			CHECK(tbRd(&r.t->ARR) == r.period);
			CHECK(tbRd(&r.t->PSC) == 0xABu);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 Init writes DIR and CMS of CR1 for up, down and centre aligned 1-3 (timers with counter mode selection)") {
	struct M { uint32_t mode, bits; };
	// RM0090: DIR bit 4 (1 = down), CMS bits 6:5 (01, 10, 11 = centre aligned 1, 2, 3)
	const M modes[5] = {{TIM_COUNTERMODE_UP, 0x00u}, {TIM_COUNTERMODE_DOWN, 0x10u}, {TIM_COUNTERMODE_CENTERALIGNED1, 0x20u},
	                    {TIM_COUNTERMODE_CENTERALIGNED2, 0x40u}, {TIM_COUNTERMODE_CENTERALIGNED3, 0x60u}};
	for (const TbInitFam &f : tbInitFams)
		for (unsigned tim : tbCselTims)
			for (const M &m : modes) {
				CAPTURE(f.name);
				CAPTURE(tim);
				CAPTURE(m.mode);
				TbRig r(tim);
				r.t->CR1 = 0x70u; // old DIR / CMS must be replaced, not merged
				r.h.Init.CounterMode = m.mode;
				REQUIRE(f.init(&r.h) == HAL_OK);
				CHECK((r.cr1() & 0x70u) == m.bits);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 Init does not write DIR or CMS on timers without counter mode selection (TIM6, TIM7, TIM9-TIM14)") {
	const unsigned tims[8] = {6, 7, 9, 10, 11, 12, 13, 14};
	for (const TbInitFam &f : tbInitFams)
		for (unsigned tim : tims) {
			CAPTURE(f.name);
			CAPTURE(tim);
			TbRig r(tim);
			r.h.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
			REQUIRE(f.init(&r.h) == HAL_OK);
			CHECK((HalFixture::lastWritten(&r.t->CR1) & 0x70u) == 0u);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 Init writes the clock division CKD of CR1 on the timers that have it") {
	struct D { uint32_t div, bits; };
	const D divs[3] = {{TIM_CLOCKDIVISION_DIV1, 0x000u}, {TIM_CLOCKDIVISION_DIV2, 0x100u}, {TIM_CLOCKDIVISION_DIV4, 0x200u}}; // RM0090 CKD 00 / 01 / 10
	for (const TbInitFam &f : tbInitFams)
		for (unsigned tim : tbAllTims) {
			if (!tbI(tim).ckd) continue;
			for (const D &d : divs) {
				CAPTURE(f.name);
				CAPTURE(tim);
				CAPTURE(d.div);
				TbRig r(tim);
				r.t->CR1 = 0x300u;
				r.h.Init.ClockDivision = d.div;
				REQUIRE(f.init(&r.h) == HAL_OK);
				CHECK((r.cr1() & 0x300u) == d.bits);
			}
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 Init does not write the clock division on TIM6 and TIM7") {
	for (unsigned tim : {6u, 7u}) {
		CAPTURE(tim);
		TbRig r(tim);
		r.h.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
		REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
		CHECK((HalFixture::lastWritten(&r.t->CR1) & 0x300u) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 Init sets and clears ARPE (bit 7 of CR1) as requested on every timer") {
	for (const TbInitFam &f : tbInitFams)
		for (unsigned tim : tbAllTims) {
			CAPTURE(f.name);
			CAPTURE(tim);
			TbRig r(tim);
			r.h.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
			REQUIRE(f.init(&r.h) == HAL_OK);
			CHECK((r.cr1() & 0x80u) == 0x80u);
			r.h.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
			REQUIRE(f.init(&r.h) == HAL_OK);
			CHECK((r.cr1() & 0x80u) == 0u);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 Init keeps UDIS, URS and OPM of CR1 and does not set CEN") {
	for (const TbInitFam &f : tbInitFams)
		for (unsigned tim : tbAllTims) {
			CAPTURE(f.name);
			CAPTURE(tim);
			TbRig r(tim);
			r.t->CR1 = 0x0Eu;
			const uint32_t keep = tbI(tim).nch == 1u ? 0x06u : 0x0Eu; // RM0090 19.5.1: TIM10 / 11 / 13 / 14 have no OPM bit
			REQUIRE(f.init(&r.h) == HAL_OK);
			CHECK((r.cr1() & 0x0Fu) == keep);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 Init writes RepetitionCounter to RCR on TIM1 and TIM8") {
	for (const TbInitFam &f : tbInitFams)
		for (unsigned tim : {1u, 8u}) {
			CAPTURE(f.name);
			CAPTURE(tim);
			TbRig r(tim);
			r.h.Init.RepetitionCounter = 0xA5u;
			REQUIRE(f.init(&r.h) == HAL_OK);
			CHECK(tbRd(&r.t->RCR) == 0xA5u);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 Init with a RepetitionCounter touches no RCR on the timers without one (no register fault)") {
	for (const TbInitFam &f : tbInitFams)
		for (unsigned tim : tbAllTims) {
			if (tbI(tim).adv) continue;
			CAPTURE(f.name);
			CAPTURE(tim);
			TbRig r(tim);
			r.h.Init.RepetitionCounter = 7u;
			REQUIRE(f.init(&r.h) == HAL_OK);
			CHECK(HalFixture::faults() == 0u);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 Init reloads the counter by an update event: 0 counting up and centre aligned, ARR counting down") {
	for (const TbInitFam &f : tbInitFams)
		for (unsigned tim : tbCselTims) {
			CAPTURE(f.name);
			CAPTURE(tim);
			TbRig r(tim);
			hostTimSetCounter(tim, 0x55u);
			REQUIRE(f.init(&r.h) == HAL_OK);
			CHECK(tbRd(&r.t->CNT) == 0u);
			hostTimSetCounter(tim, 0x55u);
			r.h.Init.CounterMode = TIM_COUNTERMODE_DOWN;
			REQUIRE(f.init(&r.h) == HAL_OK);
			CHECK(tbRd(&r.t->CNT) == r.period);
			hostTimSetCounter(tim, 0x55u);
			r.h.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
			REQUIRE(f.init(&r.h) == HAL_OK);
			CHECK(tbRd(&r.t->CNT) == 0u);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 Init on a basic timer and a one channel timer reloads the counter to 0") {
	for (unsigned tim : {6u, 7u, 9u, 10u, 14u}) {
		CAPTURE(tim);
		TbRig r(tim);
		hostTimSetCounter(tim, 0x55u);
		REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
		CHECK(tbRd(&r.t->CNT) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TB1 Init leaves the update flag UIF clear (NEEDS REVIEW 332/a)") {
	for (const TbInitFam &f : tbInitFams)
		for (unsigned tim : tbAllTims) {
			CAPTURE(f.name);
			CAPTURE(tim);
			TbRig r(tim);
			REQUIRE(f.init(&r.h) == HAL_OK);
			CHECK((tbRd(&r.t->SR) & 1u) == 0u);
			CHECK(tbRd(&r.t->DIER) == 0u);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 Init sets the state, all channel states, all complementary channel states and the DMA burst state to READY") {
	for (const TbInitFam &f : tbInitFams) {
		CAPTURE(f.name);
		TbRig r(1);
		for (unsigned k = 0; k < 4; ++k) {
			r.h.ChannelState[k] = HAL_TIM_CHANNEL_STATE_BUSY;
			r.h.ChannelNState[k] = HAL_TIM_CHANNEL_STATE_BUSY;
		}
		r.h.DMABurstState = HAL_DMA_BURST_STATE_BUSY;
		REQUIRE(f.init(&r.h) == HAL_OK);
		CHECK(r.h.State == HAL_TIM_STATE_READY);
		for (unsigned k = 0; k < 4; ++k) {
			CHECK(r.h.ChannelState[k] == HAL_TIM_CHANNEL_STATE_READY);
			CHECK(r.h.ChannelNState[k] == HAL_TIM_CHANNEL_STATE_READY);
		}
		CHECK(r.h.DMABurstState == HAL_DMA_BURST_STATE_READY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 DeInit clears CEN, calls MspDeInit of its kind once, sets all states to RESET and unlocks (all timers)") {
	for (const TbInitFam &f : tbInitFams)
		for (unsigned tim : tbAllTims) {
			CAPTURE(f.name);
			CAPTURE(tim);
			TbRig r(tim);
			REQUIRE(f.init(&r.h) == HAL_OK);
			r.t->CR1 |= tbCR1_CEN;
			r.h.Lock = HAL_LOCKED;
			CHECK(f.deinit(&r.h) == HAL_OK);
			tbAcceptBasicStop(tim);
			CHECK(r.cen() == false);
			for (unsigned k = 0; k < 8; ++k) CHECK(timCounts.mspDeInit[k] == (k == f.msp ? 1u : 0u));
			CHECK(r.h.State == HAL_TIM_STATE_RESET);
			for (unsigned k = 0; k < 4; ++k) {
				CHECK(r.h.ChannelState[k] == HAL_TIM_CHANNEL_STATE_RESET);
				CHECK(r.h.ChannelNState[k] == HAL_TIM_CHANNEL_STATE_RESET);
			}
			CHECK(r.h.DMABurstState == HAL_DMA_BURST_STATE_RESET);
			CHECK(r.h.Lock == HAL_UNLOCKED);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 Init after DeInit calls MspInit again") {
	for (const TbInitFam &f : tbInitFams) {
		CAPTURE(f.name);
		TbRig r(3);
		REQUIRE(f.init(&r.h) == HAL_OK);
		REQUIRE(f.deinit(&r.h) == HAL_OK);
		REQUIRE(f.init(&r.h) == HAL_OK);
		CHECK(timCounts.mspInit[f.msp] == 2u);
		CHECK(r.h.State == HAL_TIM_STATE_READY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TB1 DeInit leaves CEN set while a capture / compare channel is still enabled (NEEDS REVIEW 332/b)") {
	for (const TbInitFam &f : tbInitFams) {
		CAPTURE(f.name);
		TbRig r(3);
		REQUIRE(f.init(&r.h) == HAL_OK);
		r.t->CR1 |= tbCR1_CEN;
		r.t->CCER |= tbCcxe(1);
		REQUIRE(f.deinit(&r.h) == HAL_OK);
		CHECK(r.cen() == true);
		CHECK(r.h.State == HAL_TIM_STATE_RESET);
	}
}

/* ================================================================ TB2: Base Start / Stop */

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Start sets CEN and the state BUSY and leaves DIER and the channel states alone (all timers)") {
	for (unsigned tim : tbAllTims) {
		CAPTURE(tim);
		TbRig r(tim);
		REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
		CHECK(HAL_TIM_Base_Start(&r.h) == HAL_OK);
		CHECK(r.cen());
		CHECK(r.h.State == HAL_TIM_STATE_BUSY);
		CHECK(r.dier() == 0u);
		for (unsigned c = 1; c <= 4; ++c) CHECK(r.state(c) == HAL_TIM_CHANNEL_STATE_READY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Start and Start_IT return HAL_ERROR and set nothing when the state is BUSY or RESET") {
	for (int it = 0; it < 2; ++it)
		for (HAL_TIM_StateTypeDef st : {HAL_TIM_STATE_BUSY, HAL_TIM_STATE_RESET}) {
			CAPTURE(it);
			CAPTURE(static_cast<int>(st));
			TbRig r(3);
			REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
			r.h.State = st;
			CHECK((it ? HAL_TIM_Base_Start_IT(&r.h) : HAL_TIM_Base_Start(&r.h)) == HAL_ERROR);
			CHECK(r.cen() == false);
			CHECK(r.dier() == 0u);
			CHECK(r.h.State == st);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Start_IT sets UIE only, CEN and the state BUSY (all timers)") {
	for (unsigned tim : tbAllTims) {
		CAPTURE(tim);
		TbRig r(tim);
		REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
		CHECK(HAL_TIM_Base_Start_IT(&r.h) == HAL_OK);
		CHECK(r.dier() == 1u);
		CHECK(r.cen());
		CHECK(r.h.State == HAL_TIM_STATE_BUSY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Stop clears CEN, sets the state READY and keeps UIE (all timers)") {
	for (unsigned tim : tbAllTims) {
		CAPTURE(tim);
		TbRig r(tim);
		REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_Base_Start_IT(&r.h) == HAL_OK);
		CHECK(HAL_TIM_Base_Stop(&r.h) == HAL_OK);
		tbAcceptBasicStop(tim);
		CHECK(r.cen() == false);
		CHECK(r.h.State == HAL_TIM_STATE_READY);
		CHECK(r.dier() == 1u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Stop_IT clears UIE and CEN, sets the state READY and keeps the other DIER bits") {
	for (unsigned tim : tbCselTims) {
		CAPTURE(tim);
		TbRig r(tim);
		REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
		r.t->DIER = tbCcie(1);
		REQUIRE(HAL_TIM_Base_Start_IT(&r.h) == HAL_OK);
		CHECK(HAL_TIM_Base_Stop_IT(&r.h) == HAL_OK);
		CHECK(r.dier() == tbCcie(1));
		CHECK(r.cen() == false);
		CHECK(r.h.State == HAL_TIM_STATE_READY);
	}
	for (unsigned tim : {6u, 7u}) {
		TbRig r(tim);
		REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_Base_Start_IT(&r.h) == HAL_OK);
		CHECK(HAL_TIM_Base_Stop_IT(&r.h) == HAL_OK);
		tbAcceptBasicStop(tim);
		CHECK(r.dier() == 0u);
		CHECK(r.cen() == false);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base can be started again after Stop and after Stop_IT") {
	TbRig r(3);
	REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_Base_Start(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_Base_Stop(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_Base_Start_IT(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_Base_Stop_IT(&r.h) == HAL_OK);
	CHECK(HAL_TIM_Base_Start(&r.h) == HAL_OK);
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TB2 Base Stop leaves CEN set while a capture / compare channel is enabled (NEEDS REVIEW 332/b)") {
	TbRig r(3);
	REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_Base_Start(&r.h) == HAL_OK);
	r.t->CCER |= tbCcxe(2);
	CHECK(HAL_TIM_Base_Stop(&r.h) == HAL_OK);
	CHECK(r.cen() == true);
	CHECK(r.h.State == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Start, Start_IT do not set CEN in slave trigger mode (SMS = 110) and set it in every other slave mode") {
	for (unsigned tim : tbSlaveTims)
		for (int it = 0; it < 2; ++it)
			for (uint32_t sms = 0; sms < 8; ++sms) {
				CAPTURE(tim);
				CAPTURE(it);
				CAPTURE(sms);
				TbRig r(tim);
				REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
				r.t->SMCR = sms;
				CHECK((it ? HAL_TIM_Base_Start_IT(&r.h) : HAL_TIM_Base_Start(&r.h)) == HAL_OK);
				CHECK(r.cen() == (sms != tbSMS_TRIGGER));
				CHECK(r.h.State == HAL_TIM_STATE_BUSY);
				CHECK(r.dier() == (it ? 1u : 0u));
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Start sets CEN on the timers without a slave mode controller without reading SMCR (TIM6, TIM7, TIM10, TIM11, TIM13, TIM14)") {
	for (unsigned tim : {6u, 7u, 10u, 11u, 13u, 14u}) {
		CAPTURE(tim);
		TbRig r(tim);
		REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
		CHECK(HAL_TIM_Base_Start_IT(&r.h) == HAL_OK);
		CHECK(r.cen());
		CHECK(HalFixture::faults() == 0u);
	}
}

/* ================================================================ TB2: Base DMA */

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Start_DMA programs the update stream with the buffer, the ARR address and Length items and enables it") {
	for (unsigned tim : tbDmaBaseTims)
		for (uint16_t len : {uint16_t(1), uint16_t(5), uint16_t(65535)}) {
			CAPTURE(tim);
			CAPTURE(len);
			TbRig r(tim);
			REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
			REQUIRE(HAL_TIM_Base_Start_DMA(&r.h, r.buf, len) == HAL_OK);
			DMA_Stream_TypeDef *s = r.stream(0);
			CHECK(s->NDTR == len);
			CHECK(s->PAR == r.base() + 0x2Cu);
			CHECK(s->M0AR == r.addr());
			CHECK((s->CR & tbDMA_EN) == tbDMA_EN);
			CHECK((s->CR & (tbDMA_TCIE | tbDMA_HTIE | tbDMA_TEIE)) == (tbDMA_TCIE | tbDMA_HTIE | tbDMA_TEIE));
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Start_DMA enables the update DMA request UDE, CEN and the state BUSY") {
	for (unsigned tim : tbDmaBaseTims) {
		CAPTURE(tim);
		TbRig r(tim);
		REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_Base_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
		CHECK(r.dier() == (1u << 8));
		CHECK(r.cen());
		CHECK(r.h.State == HAL_TIM_STATE_BUSY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Start_DMA returns HAL_ERROR and starts nothing for a NULL buffer or Length 0") {
	for (int nullBuf = 0; nullBuf < 2; ++nullBuf) {
		CAPTURE(nullBuf);
		TbRig r(2);
		REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
		CHECK(HAL_TIM_Base_Start_DMA(&r.h, nullBuf ? nullptr : r.buf, nullBuf ? 4 : 0) == HAL_ERROR);
		CHECK((r.stream(0)->CR & tbDMA_EN) == 0u);
		CHECK(r.dier() == 0u);
		CHECK(r.cen() == false);
		CHECK(r.h.State == HAL_TIM_STATE_READY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TB2 Base Start_DMA of a BUSY handle returns HAL_BUSY and of a RESET handle HAL_ERROR, without touching the stream (NEEDS REVIEW 332/c)") {
	TbRig r(2);
	REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
	r.h.State = HAL_TIM_STATE_BUSY;
	CHECK(HAL_TIM_Base_Start_DMA(&r.h, r.buf, 4) == HAL_BUSY);
	r.h.State = HAL_TIM_STATE_RESET;
	CHECK(HAL_TIM_Base_Start_DMA(&r.h, r.buf, 4) == HAL_ERROR);
	CHECK((r.stream(0)->CR & tbDMA_EN) == 0u);
	CHECK(r.dier() == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Start_DMA returns HAL_ERROR and enables no request or counter when the DMA stream cannot start") {
	TbRig r(2);
	REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
	r.dma[0].State = HAL_DMA_STATE_BUSY; // HAL_DMA_Start_IT then refuses
	CHECK(HAL_TIM_Base_Start_DMA(&r.h, r.buf, 4) == HAL_ERROR);
	CHECK(r.dier() == 0u);
	CHECK(r.cen() == false);
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Start_DMA that fails because the DMA stream cannot start leaves the handle READY" * KNOWN_ISSUE("candidate vendor bug V112 (arguable: UM1725 gives no state rule for a failed DMA start): the state stays BUSY after the HAL_ERROR return, so every later start returns HAL_ERROR (same pattern as V101 of the SPI driver)")) {
	TbRig r(2);
	REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
	r.dma[0].State = HAL_DMA_STATE_BUSY;
	REQUIRE(HAL_TIM_Base_Start_DMA(&r.h, r.buf, 4) == HAL_ERROR);
	CHECK(r.h.State == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Stop_DMA clears UDE and CEN, stops the stream and sets the state READY") {
	for (unsigned tim : tbDmaBaseTims) {
		CAPTURE(tim);
		TbRig r(tim);
		REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_Base_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
		CHECK(HAL_TIM_Base_Stop_DMA(&r.h) == HAL_OK);
		tbAcceptBasicStop(tim);
		CHECK(r.dier() == 0u);
		CHECK(r.cen() == false);
		CHECK((r.stream(0)->CR & tbDMA_EN) == 0u);
		CHECK(r.h.State == HAL_TIM_STATE_READY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Stop_DMA of a handle that never started a transfer returns HAL_OK and sets the state READY") {
	TbRig r(5);
	REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
	CHECK(HAL_TIM_Base_Stop_DMA(&r.h) == HAL_OK);
	CHECK(r.h.State == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base Start_DMA does not set CEN in slave trigger mode and sets it in the other modes") {
	for (unsigned tim : tbDmaChanTims)
		for (uint32_t sms = 0; sms < 8; ++sms) {
			CAPTURE(tim);
			CAPTURE(sms);
			TbRig r(tim);
			REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
			r.t->SMCR = sms;
			CHECK(HAL_TIM_Base_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
			CHECK(r.cen() == (sms != tbSMS_TRIGGER));
			CHECK(r.dier() == (1u << 8));
			CHECK(r.h.State == HAL_TIM_STATE_BUSY);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base DMA transfer complete runs the period elapsed callback and sets READY (normal mode), keeps BUSY (circular mode)") {
	for (int circ = 0; circ < 2; ++circ) {
		CAPTURE(circ);
		TbRig r(1, circ ? DMA_CIRCULAR : DMA_NORMAL);
		REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_Base_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
		hostDmaComplete(r.ctrl(), r.m->stream[0]);
		HAL_DMA_IRQHandler(&r.dma[0]);
		CHECK(timCounts.period == 1u);
		CHECK(timCounts.last == &r.h);
		CHECK(r.h.State == (circ ? HAL_TIM_STATE_BUSY : HAL_TIM_STATE_READY));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base DMA half transfer runs the period elapsed half callback and keeps the state BUSY") {
	TbRig r(1);
	REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_Base_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	hostDmaStepItems(r.ctrl(), r.m->stream[0], 2);
	HAL_DMA_IRQHandler(&r.dma[0]);
	CHECK(timCounts.periodHalf == 1u);
	CHECK(timCounts.period == 0u);
	CHECK(r.h.State == HAL_TIM_STATE_BUSY);
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Base DMA transfer error runs the error callback and sets the state READY") {
	TbRig r(1);
	REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_Base_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	hostDmaRaise(r.ctrl(), r.m->stream[0], HOST_DMA_TEIF);
	HAL_DMA_IRQHandler(&r.dma[0]);
	CHECK(timCounts.error == 1u);
	CHECK(r.h.State == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TimFixture, "TB2 a base timer with the update interrupt calls the period elapsed callback once per counter overflow") {
	for (unsigned tim : tbAllTims) {
		CAPTURE(tim);
		TbRig r(tim);
		r.h.Init.Period = 9u;
		r.h.Init.Prescaler = 0u;
		REQUIRE(HAL_TIM_Base_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_Base_Start_IT(&r.h) == HAL_OK);
		hostTimAdvance(tim, 9);
		CHECK(hostTimIrqPending(tim) == 0u); // counter is at ARR, no overflow yet
		hostTimAdvance(tim, 1);
		CHECK(hostTimIrqPending(tim) != 0u);
		HAL_TIM_IRQHandler(&r.h);
		CHECK(timCounts.period == 1u);
		CHECK(hostTimIrqPending(tim) == 0u);
	}
}

/* ================================================================ TB2: OC / PWM Start / Stop */

TEST_CASE_FIXTURE(TimFixture, "TB2 the polling Start of a channel sets only its CCxE, CEN and its channel state BUSY, and returns HAL_OK (OC and PWM)") {
	for (const TbFam &f : tbFams)
		for (unsigned tim : tbChanTims)
			for (unsigned c = 1; c <= tbI(tim).nch; ++c) {
				CAPTURE(f.name);
				CAPTURE(tim);
				CAPTURE(c);
				TbRig r(tim);
				REQUIRE(f.init(&r.h) == HAL_OK);
				CHECK(f.start(&r.h, tbCh(c)) == HAL_OK);
				CHECK(r.ccer() == tbCcxe(c));
				CHECK(r.cen());
				CHECK(r.dier() == 0u);
				CHECK(r.h.State == HAL_TIM_STATE_READY);
				for (unsigned k = 1; k <= 4; ++k) CHECK(r.state(k) == (k == c ? HAL_TIM_CHANNEL_STATE_BUSY : HAL_TIM_CHANNEL_STATE_READY));
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 the main output enable MOE is set by every start of TIM1 and TIM8 and the other timers have no BDTR access") {
	for (const TbFam &f : tbFams)
		for (int mode = 0; mode < 3; ++mode)
			for (unsigned tim : tbChanTims) {
				if (mode == 2 && !tbDmaOk(tim, 1)) continue;
				CAPTURE(f.name);
				CAPTURE(mode);
				CAPTURE(tim);
				TbRig r(tim);
				REQUIRE(f.init(&r.h) == HAL_OK);
				REQUIRE(tbStart(f, r, mode, 1) == HAL_OK);
				CHECK(r.moe() == tbI(tim).adv);
				CHECK(HalFixture::faults() == 0u);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Start, Start_IT and Start_DMA return HAL_ERROR and change no register when the channel is BUSY or RESET") {
	for (const TbFam &f : tbFams)
		for (int mode = 0; mode < 3; ++mode)
			for (HAL_TIM_ChannelStateTypeDef st : {HAL_TIM_CHANNEL_STATE_BUSY, HAL_TIM_CHANNEL_STATE_RESET}) {
				if (mode == 2 && st == HAL_TIM_CHANNEL_STATE_BUSY) continue; // BUSY gives HAL_BUSY there (NEEDS REVIEW 332/c)
				CAPTURE(f.name);
				CAPTURE(mode);
				CAPTURE(static_cast<int>(st));
				TbRig r(1);
				REQUIRE(f.init(&r.h) == HAL_OK);
				r.h.ChannelState[1] = st;
				const TbSnap s(r);
				CHECK(tbStart(f, r, mode, 2) == HAL_ERROR);
				CHECK(s.same(r));
				CHECK(r.state(2) == st);
				if (mode == 2) CHECK((r.stream(2)->CR & tbDMA_EN) == 0u);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Start_IT enables only the capture / compare interrupt of the channel, with its CCxE, CEN and state BUSY (OC and PWM)") {
	for (const TbFam &f : tbFams)
		for (unsigned tim : tbChanTims)
			for (unsigned c = 1; c <= tbI(tim).nch; ++c) {
				CAPTURE(f.name);
				CAPTURE(tim);
				CAPTURE(c);
				TbRig r(tim);
				REQUIRE(f.init(&r.h) == HAL_OK);
				CHECK(f.startIt(&r.h, tbCh(c)) == HAL_OK);
				CHECK(r.dier() == tbCcie(c));
				CHECK(r.ccer() == tbCcxe(c));
				CHECK(r.cen());
				CHECK(r.state(c) == HAL_TIM_CHANNEL_STATE_BUSY);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 starting two channels keeps both CCxE and both interrupt bits") {
	for (const TbFam &f : tbFams)
		for (unsigned tim : {1u, 3u, 8u, 9u, 12u}) {
			CAPTURE(f.name);
			CAPTURE(tim);
			TbRig r(tim);
			REQUIRE(f.init(&r.h) == HAL_OK);
			REQUIRE(f.startIt(&r.h, tbCh(1)) == HAL_OK);
			REQUIRE(f.startIt(&r.h, tbCh(2)) == HAL_OK);
			CHECK(r.ccer() == (tbCcxe(1) | tbCcxe(2)));
			CHECK(r.dier() == (tbCcie(1) | tbCcie(2)));
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Start does not set CEN in slave trigger mode but still enables the channel and the main output (poll, IT, DMA; OC and PWM)") {
	for (const TbFam &f : tbFams)
		for (int mode = 0; mode < 3; ++mode)
			for (unsigned tim : tbSlaveTims) {
				if (mode == 2 && !tbDmaOk(tim, 1)) continue;
				for (uint32_t sms = 0; sms < 8; ++sms) {
					CAPTURE(f.name);
					CAPTURE(mode);
					CAPTURE(tim);
					CAPTURE(sms);
					TbRig r(tim);
					REQUIRE(f.init(&r.h) == HAL_OK);
					r.t->SMCR = sms;
					REQUIRE(tbStart(f, r, mode, 1) == HAL_OK);
					CHECK(r.cen() == (sms != tbSMS_TRIGGER));
					CHECK(r.ccer() == tbCcxe(1));
					CHECK(r.moe() == tbI(tim).adv);
					CHECK(r.state(1) == HAL_TIM_CHANNEL_STATE_BUSY);
				}
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Stop clears CCxE, CEN and MOE of the only active channel and sets its state READY (poll, IT, DMA; OC and PWM)") {
	for (const TbFam &f : tbFams)
		for (int mode = 0; mode < 3; ++mode)
			for (unsigned tim : tbChanTims)
				for (unsigned c = 1; c <= tbI(tim).nch; ++c) {
					if (mode == 2 && !tbDmaOk(tim, c)) continue;
					CAPTURE(f.name);
					CAPTURE(mode);
					CAPTURE(tim);
					CAPTURE(c);
					TbRig r(tim);
					REQUIRE(f.init(&r.h) == HAL_OK);
					REQUIRE(tbStart(f, r, mode, c) == HAL_OK);
					CHECK(tbStop(f, r, mode, c) == HAL_OK);
					CHECK(r.ccer() == 0u);
					CHECK(r.cen() == false);
					CHECK(r.moe() == false);
					CHECK(r.state(c) == HAL_TIM_CHANNEL_STATE_READY);
					CHECK(r.dier() == 0u);
				}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 the polling Stop leaves the DIER bits as they were") {
	for (const TbFam &f : tbFams) {
		CAPTURE(f.name);
		TbRig r(3);
		REQUIRE(f.init(&r.h) == HAL_OK);
		r.t->DIER = tbCcie(3) | 1u;
		REQUIRE(f.start(&r.h, tbCh(1)) == HAL_OK);
		REQUIRE(f.stop(&r.h, tbCh(1)) == HAL_OK);
		CHECK(r.dier() == (tbCcie(3) | 1u));
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Stop of one of two channels keeps the other CCxE, CEN and MOE (poll and IT)") {
	for (const TbFam &f : tbFams)
		for (int mode = 0; mode < 2; ++mode)
			for (unsigned tim : {1u, 2u, 8u, 9u, 12u}) {
				CAPTURE(f.name);
				CAPTURE(mode);
				CAPTURE(tim);
				TbRig r(tim);
				REQUIRE(f.init(&r.h) == HAL_OK);
				REQUIRE(tbStart(f, r, mode, 1) == HAL_OK);
				REQUIRE(tbStart(f, r, mode, 2) == HAL_OK);
				REQUIRE(tbStop(f, r, mode, 1) == HAL_OK);
				CHECK(r.ccer() == tbCcxe(2));
				CHECK(r.cen());
				CHECK(r.moe() == tbI(tim).adv);
				CHECK(r.dier() == (mode == 1 ? tbCcie(2) : 0u));
				REQUIRE(tbStop(f, r, mode, 2) == HAL_OK);
				CHECK(r.ccer() == 0u);
				CHECK(r.cen() == false);
				CHECK(r.moe() == false);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Stop keeps CEN and MOE of TIM1 and TIM8 while a complementary output CCxNE is still enabled") {
	for (const TbFam &f : tbFams)
		for (unsigned tim : {1u, 8u}) {
			CAPTURE(f.name);
			CAPTURE(tim);
			TbRig r(tim);
			REQUIRE(f.init(&r.h) == HAL_OK);
			REQUIRE(f.start(&r.h, tbCh(1)) == HAL_OK);
			r.t->CCER |= tbCcxne(2);
			REQUIRE(f.stop(&r.h, tbCh(1)) == HAL_OK);
			CHECK(r.ccer() == tbCcxne(2));
			CHECK(r.cen());
			CHECK(r.moe());
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Stop_IT clears the interrupt bit of that channel only") {
	for (const TbFam &f : tbFams)
		for (unsigned c = 1; c <= 4; ++c) {
			CAPTURE(f.name);
			CAPTURE(c);
			TbRig r(1);
			REQUIRE(f.init(&r.h) == HAL_OK);
			r.t->DIER = 0x1Eu | 1u; // CC1IE..CC4IE and UIE
			REQUIRE(f.startIt(&r.h, tbCh(c)) == HAL_OK);
			REQUIRE(f.stopIt(&r.h, tbCh(c)) == HAL_OK);
			CHECK(r.dier() == ((0x1Eu | 1u) & ~tbCcie(c)));
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Stop of a channel that was never started returns HAL_OK and sets its state READY (poll)") {
	for (const TbFam &f : tbFams) {
		CAPTURE(f.name);
		TbRig r(1);
		r.h.ChannelState[2] = HAL_TIM_CHANNEL_STATE_RESET;
		CHECK(f.stop(&r.h, TIM_CHANNEL_3) == HAL_OK);
		CHECK(r.state(3) == HAL_TIM_CHANNEL_STATE_READY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 a channel can be started again after its Stop (poll, IT, DMA)") {
	for (const TbFam &f : tbFams)
		for (int mode = 0; mode < 3; ++mode) {
			CAPTURE(f.name);
			CAPTURE(mode);
			TbRig r(1);
			REQUIRE(f.init(&r.h) == HAL_OK);
			REQUIRE(tbStart(f, r, mode, 3) == HAL_OK);
			REQUIRE(tbStop(f, r, mode, 3) == HAL_OK);
			if (mode == 2) HAL_DMA_IRQHandler(&r.dma[3]); // the abort of the DMA stream completes in its interrupt (TCIF)
			CHECK(tbStart(f, r, mode, 3) == HAL_OK);
		}
}

/* ---------------------------------------------------------------- OC / PWM DMA */

TEST_CASE_FIXTURE(TimFixture, "TB2 Start_DMA programs the stream of the channel with the buffer, the CCRx address and Length items, and enables it (OC and PWM)") {
	for (const TbFam &f : tbFams)
		for (unsigned tim : tbDmaChanTims)
			for (unsigned c = 1; c <= 4; ++c) {
				if (!tbDmaOk(tim, c)) continue;
				CAPTURE(f.name);
				CAPTURE(tim);
				CAPTURE(c);
				TbRig r(tim);
				REQUIRE(f.init(&r.h) == HAL_OK);
				REQUIRE(f.startDma(&r.h, tbCh(c), r.buf, 5) == HAL_OK);
				DMA_Stream_TypeDef *s = r.stream(c);
				CHECK(s->NDTR == 5u);
				CHECK(s->PAR == r.base() + 0x34u + 4u * (c - 1u)); // RM0090 register map: CCR1 0x34, CCR2 0x38, CCR3 0x3C, CCR4 0x40
				CHECK(s->M0AR == r.addr());
				CHECK((s->CR & tbDMA_EN) == tbDMA_EN);
				CHECK((s->CR & (tbDMA_TCIE | tbDMA_HTIE | tbDMA_TEIE)) == (tbDMA_TCIE | tbDMA_HTIE | tbDMA_TEIE));
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Start_DMA with Length 1 and Length 65535 programs NDTR with that many items") {
	for (uint16_t len : {uint16_t(1), uint16_t(65535)}) {
		CAPTURE(len);
		TbRig r(1);
		REQUIRE(HAL_TIM_PWM_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_PWM_Start_DMA(&r.h, TIM_CHANNEL_3, r.buf, len) == HAL_OK);
		CHECK(r.stream(3)->NDTR == len);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Start_DMA enables only the DMA request of the channel, CCxE, CEN and the state BUSY (OC and PWM)") {
	for (const TbFam &f : tbFams)
		for (unsigned tim : tbDmaChanTims)
			for (unsigned c = 1; c <= 4; ++c) {
				if (!tbDmaOk(tim, c)) continue;
				CAPTURE(f.name);
				CAPTURE(tim);
				CAPTURE(c);
				TbRig r(tim);
				REQUIRE(f.init(&r.h) == HAL_OK);
				REQUIRE(f.startDma(&r.h, tbCh(c), r.buf, 4) == HAL_OK);
				CHECK(r.dier() == tbCcde(c));
				CHECK(r.ccer() == tbCcxe(c));
				CHECK(r.cen());
				CHECK(r.state(c) == HAL_TIM_CHANNEL_STATE_BUSY);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Start_DMA returns HAL_ERROR and starts nothing for a NULL buffer or Length 0") {
	for (const TbFam &f : tbFams)
		for (unsigned c = 1; c <= 4; ++c)
			for (int nullBuf = 0; nullBuf < 2; ++nullBuf) {
				CAPTURE(f.name);
				CAPTURE(c);
				CAPTURE(nullBuf);
				TbRig r(1);
				REQUIRE(f.init(&r.h) == HAL_OK);
				CHECK(f.startDma(&r.h, tbCh(c), nullBuf ? nullptr : r.buf, nullBuf ? 4 : 0) == HAL_ERROR);
				CHECK((r.stream(c)->CR & tbDMA_EN) == 0u);
				CHECK(r.ccer() == 0u);
				CHECK(r.dier() == 0u);
				CHECK(r.cen() == false);
				CHECK(r.state(c) == HAL_TIM_CHANNEL_STATE_READY);
			}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TB2 Start_DMA of a BUSY channel returns HAL_BUSY and of a RESET channel HAL_ERROR, without touching the stream (NEEDS REVIEW 332/c)") {
	for (const TbFam &f : tbFams) {
		CAPTURE(f.name);
		TbRig r(8);
		REQUIRE(f.init(&r.h) == HAL_OK);
		r.h.ChannelState[0] = HAL_TIM_CHANNEL_STATE_BUSY;
		CHECK(f.startDma(&r.h, TIM_CHANNEL_1, r.buf, 4) == HAL_BUSY);
		r.h.ChannelState[0] = HAL_TIM_CHANNEL_STATE_RESET;
		CHECK(f.startDma(&r.h, TIM_CHANNEL_1, r.buf, 4) == HAL_ERROR);
		CHECK((r.stream(1)->CR & tbDMA_EN) == 0u);
		CHECK(r.dier() == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Start_DMA returns HAL_ERROR and enables no request, output or counter when the DMA stream cannot start") {
	for (const TbFam &f : tbFams)
		for (unsigned c = 1; c <= 4; ++c) {
			CAPTURE(f.name);
			CAPTURE(c);
			TbRig r(1);
			REQUIRE(f.init(&r.h) == HAL_OK);
			r.dma[c].State = HAL_DMA_STATE_BUSY; // HAL_DMA_Start_IT then refuses
			CHECK(f.startDma(&r.h, tbCh(c), r.buf, 4) == HAL_ERROR);
			CHECK(r.dier() == 0u);
			CHECK(r.ccer() == 0u);
			CHECK(r.moe() == false);
			CHECK(r.cen() == false);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Start_DMA that fails because the DMA stream cannot start leaves the channel state READY" * KNOWN_ISSUE("candidate vendor bug V112 (arguable: UM1725 gives no state rule for a failed DMA start): the channel state stays BUSY after the HAL_ERROR return, so every later start of that channel returns HAL_BUSY (same pattern as V101 of the SPI driver)")) {
	for (const TbFam &f : tbFams) {
		CAPTURE(f.name);
		TbRig r(1);
		REQUIRE(f.init(&r.h) == HAL_OK);
		r.dma[1].State = HAL_DMA_STATE_BUSY;
		REQUIRE(f.startDma(&r.h, TIM_CHANNEL_1, r.buf, 4) == HAL_ERROR);
		CHECK(r.state(1) == HAL_TIM_CHANNEL_STATE_READY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Stop_DMA clears the DMA request of that channel only and stops its stream, leaving another channel's stream running") {
	for (const TbFam &f : tbFams) {
		CAPTURE(f.name);
		TbRig r(1);
		REQUIRE(f.init(&r.h) == HAL_OK);
		REQUIRE(f.startDma(&r.h, TIM_CHANNEL_1, r.buf, 4) == HAL_OK);
		REQUIRE(f.startDma(&r.h, TIM_CHANNEL_2, r.buf, 4) == HAL_OK);
		CHECK(f.stopDma(&r.h, TIM_CHANNEL_1) == HAL_OK);
		CHECK(r.dier() == tbCcde(2));
		CHECK((r.stream(1)->CR & tbDMA_EN) == 0u);
		CHECK((r.stream(2)->CR & tbDMA_EN) == tbDMA_EN);
		CHECK(r.ccer() == tbCcxe(2));
		CHECK(r.cen());
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 Stop_DMA of a channel that never started a transfer returns HAL_OK and sets its state READY") {
	for (const TbFam &f : tbFams) {
		CAPTURE(f.name);
		TbRig r(1);
		REQUIRE(f.init(&r.h) == HAL_OK);
		CHECK(f.stopDma(&r.h, TIM_CHANNEL_4) == HAL_OK);
		CHECK(r.state(4) == HAL_TIM_CHANNEL_STATE_READY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 DMA transfer complete runs the pulse finished callback with the channel active and sets READY (normal mode), keeps BUSY (circular mode) (PWM)") {
	for (int circ = 0; circ < 2; ++circ)
		for (unsigned tim : {1u, 8u})
			for (unsigned c = 1; c <= 4; ++c) {
				CAPTURE(circ);
				CAPTURE(tim);
				CAPTURE(c);
				TbRig r(tim, circ ? DMA_CIRCULAR : DMA_NORMAL);
				REQUIRE(HAL_TIM_PWM_Init(&r.h) == HAL_OK);
				REQUIRE(HAL_TIM_PWM_Start_DMA(&r.h, tbCh(c), r.buf, 4) == HAL_OK);
				hostDmaComplete(r.ctrl(), r.m->stream[c]);
				HAL_DMA_IRQHandler(&r.dma[c]);
				CHECK(timCounts.pwmPulse == 1u);
				CHECK(timCounts.last == &r.h);
				CHECK(timCounts.channelAtLast == tbActive(c));
				CHECK(r.h.Channel == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
				CHECK(r.state(c) == (circ ? HAL_TIM_CHANNEL_STATE_BUSY : HAL_TIM_CHANNEL_STATE_READY));
			}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TB2 the DMA completion of an OC stream calls the pulse finished callback and not the OC delay elapsed callback (NEEDS REVIEW 332/e)") {
	TbRig r(1);
	REQUIRE(HAL_TIM_OC_Init(&r.h) == HAL_OK);
	REQUIRE(HAL_TIM_OC_Start_DMA(&r.h, TIM_CHANNEL_1, r.buf, 4) == HAL_OK);
	hostDmaComplete(r.ctrl(), r.m->stream[1]);
	HAL_DMA_IRQHandler(&r.dma[1]);
	CHECK(timCounts.pwmPulse == 1u);
	CHECK(timCounts.ocDelay == 0u);
	CHECK(r.state(1) == HAL_TIM_CHANNEL_STATE_READY);
}

TEST_CASE_FIXTURE(TimFixture, "TB2 DMA half transfer runs the half pulse finished callback with the channel active and keeps the channel BUSY (OC and PWM)") {
	for (const TbFam &f : tbFams)
		for (unsigned c = 1; c <= 4; ++c) {
			CAPTURE(f.name);
			CAPTURE(c);
			TbRig r(1);
			REQUIRE(f.init(&r.h) == HAL_OK);
			REQUIRE(f.startDma(&r.h, tbCh(c), r.buf, 4) == HAL_OK);
			hostDmaStepItems(r.ctrl(), r.m->stream[c], 2);
			HAL_DMA_IRQHandler(&r.dma[c]);
			CHECK(timCounts.pwmPulseHalf == 1u);
			CHECK(timCounts.pwmPulse == 0u);
			CHECK(timCounts.channelAtLast == tbActive(c));
			CHECK(r.state(c) == HAL_TIM_CHANNEL_STATE_BUSY);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB2 a DMA transfer error runs the error callback with the channel active and sets the channel READY (OC and PWM)") {
	for (const TbFam &f : tbFams)
		for (unsigned c = 1; c <= 4; ++c) {
			CAPTURE(f.name);
			CAPTURE(c);
			TbRig r(8);
			REQUIRE(f.init(&r.h) == HAL_OK);
			REQUIRE(f.startDma(&r.h, tbCh(c), r.buf, 4) == HAL_OK);
			hostDmaRaise(r.ctrl(), r.m->stream[c], HOST_DMA_TEIF);
			HAL_DMA_IRQHandler(&r.dma[c]);
			CHECK(timCounts.error == 1u);
			CHECK(timCounts.channelAtLast == tbActive(c));
			CHECK(r.h.Channel == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
			CHECK(r.state(c) == HAL_TIM_CHANNEL_STATE_READY);
		}
}

/* ---------------------------------------------------------------- invalid channel */

TEST_CASE_FIXTURE(TimFixture, "characterization: TB2 the polling Start with an invalid channel returns HAL_OK, enables no output and leaves the state of channel 4 BUSY (NEEDS REVIEW 332/d)") {
	for (const TbFam &f : tbFams) {
		CAPTURE(f.name);
		TbRig r(3);
		REQUIRE(f.init(&r.h) == HAL_OK);
		CHECK(f.start(&r.h, TIM_CHANNEL_ALL) == HAL_OK);
		CHECK(r.ccer() == 0u);
		CHECK(r.state(4) == HAL_TIM_CHANNEL_STATE_BUSY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TB2 Start_IT and Start_DMA with an invalid channel return HAL_ERROR, enable nothing and leave the state of channel 4 BUSY (NEEDS REVIEW 332/d)") {
	for (const TbFam &f : tbFams)
		for (int mode = 1; mode < 3; ++mode) {
			CAPTURE(f.name);
			CAPTURE(mode);
			TbRig r(3);
			REQUIRE(f.init(&r.h) == HAL_OK);
			const TbSnap s(r);
			HAL_StatusTypeDef rc = mode == 1 ? f.startIt(&r.h, TIM_CHANNEL_ALL) : f.startDma(&r.h, TIM_CHANNEL_ALL, r.buf, 4);
			CHECK(rc == HAL_ERROR);
			CHECK(s.same(r));
			CHECK(r.state(4) == HAL_TIM_CHANNEL_STATE_BUSY);
		}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TB2 the polling Stop with an invalid channel returns HAL_OK, the IT and DMA stops return HAL_ERROR (NEEDS REVIEW 332/d)") {
	for (const TbFam &f : tbFams) {
		CAPTURE(f.name);
		TbRig r(3);
		REQUIRE(f.init(&r.h) == HAL_OK);
		r.h.ChannelState[3] = HAL_TIM_CHANNEL_STATE_BUSY;
		CHECK(f.stopIt(&r.h, TIM_CHANNEL_ALL) == HAL_ERROR);
		CHECK(f.stopDma(&r.h, TIM_CHANNEL_ALL) == HAL_ERROR);
		CHECK(r.state(4) == HAL_TIM_CHANNEL_STATE_BUSY);
		CHECK(f.stop(&r.h, TIM_CHANNEL_ALL) == HAL_OK);
		CHECK(r.state(4) == HAL_TIM_CHANNEL_STATE_READY);
	}
}

/* ---------------------------------------------------------------- end to end with the model */

TEST_CASE_FIXTURE(TimFixture, "TB2 an OC channel with its interrupt sets CCxIF at the compare match and the handler calls the callbacks (all four channels)") {
	for (unsigned c = 1; c <= 4; ++c) {
		CAPTURE(c);
		TbRig r(3);
		REQUIRE(HAL_TIM_OC_Init(&r.h) == HAL_OK);
		TIM_OC_InitTypeDef oc{};
		oc.OCMode = TIM_OCMODE_TOGGLE;
		oc.Pulse = 5u;
		REQUIRE(HAL_TIM_OC_ConfigChannel(&r.h, &oc, tbCh(c)) == HAL_OK);
		REQUIRE(HAL_TIM_OC_Start_IT(&r.h, tbCh(c)) == HAL_OK);
		hostTimAdvance(3, 4);
		CHECK(hostTimIrqPending(3) == 0u);
		hostTimAdvance(3, 1);
		CHECK(hostTimIrqPending(3) == tbCcie(c));
		HAL_TIM_IRQHandler(&r.h);
		CHECK(timCounts.ocDelay == 1u);
		CHECK(timCounts.pwmPulse == 1u);
		CHECK(timCounts.channelAtLast == tbActive(c));
	}
}

/* ================================================================ TB3: ConfigChannel */

namespace {

const uint32_t tbOcModeConst[8] = {TIM_OCMODE_TIMING, TIM_OCMODE_ACTIVE, TIM_OCMODE_INACTIVE, TIM_OCMODE_TOGGLE,
                                   TIM_OCMODE_FORCED_INACTIVE, TIM_OCMODE_FORCED_ACTIVE, TIM_OCMODE_PWM1, TIM_OCMODE_PWM2};
// RM0090 OCxM field value of each constant above (000 frozen, 001 active, 010 inactive, 011 toggle, 100 forced inactive, 101 forced active, 110 PWM1, 111 PWM2)
unsigned tbShift(unsigned c) { return ((c - 1u) & 1u) * 8u; }
volatile uint32_t *tbCcmr(TbRig &r, unsigned c) { return c <= 2 ? &r.t->CCMR1 : &r.t->CCMR2; }
volatile uint32_t *tbCcr(TbRig &r, unsigned c) { return c == 1 ? &r.t->CCR1 : c == 2 ? &r.t->CCR2 : c == 3 ? &r.t->CCR3 : &r.t->CCR4; }

} // namespace

TEST_CASE_FIXTURE(TimFixture, "TB3 ConfigChannel writes OCMode to OCxM and clears CCxS, keeping the other channel's half of CCMR (all OC modes, OC)") {
	for (unsigned tim : tbChanTims)
		for (unsigned c = 1; c <= tbI(tim).nch; ++c)
			for (unsigned m = 0; m < 8; ++m) {
				CAPTURE(tim);
				CAPTURE(c);
				CAPTURE(m);
				TbRig r(tim);
				REQUIRE(HAL_TIM_OC_Init(&r.h) == HAL_OK);
				*tbCcmr(r, c) = 0xFFFFu; // CCxS = 11, OCxM = 111 and the FE / PE / CE bits all set, channel is off
				const uint32_t before = tbRd(tbCcmr(r, c));
				TIM_OC_InitTypeDef oc{};
				oc.OCMode = tbOcModeConst[m];
				oc.Pulse = 1u;
				CHECK(HAL_TIM_OC_ConfigChannel(&r.h, &oc, tbCh(c)) == HAL_OK);
				tbAcceptConfigFaults(tim);
				const uint32_t now = tbRd(tbCcmr(r, c));
				const uint32_t sh = tbShift(c);
				CHECK(((now >> sh) & 3u) == 0u);          // CCxS = output
				CHECK(((now >> (sh + 4u)) & 7u) == m);    // OCxM
				CHECK(((now >> sh) & 0x8Cu) == 0x8Cu);    // OCxCE (7), OCxPE (3), OCxFE (2) untouched
				if (c <= 2 || tim == 1 || tim == 2 || tim == 3 || tim == 4 || tim == 5 || tim == 8) {
					const uint32_t other = 0xFFu << (8u - sh);
					CHECK((now & other) == (before & other & 0xFF00u ? (before & other) : (before & other)));
				}
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB3 PWM ConfigChannel writes PWM1 and PWM2 to OCxM, clears CCxS and sets the preload enable OCxPE") {
	for (unsigned tim : tbChanTims)
		for (unsigned c = 1; c <= tbI(tim).nch; ++c)
			for (unsigned m : {6u, 7u}) {
				CAPTURE(tim);
				CAPTURE(c);
				CAPTURE(m);
				TbRig r(tim);
				REQUIRE(HAL_TIM_PWM_Init(&r.h) == HAL_OK);
				TIM_OC_InitTypeDef oc{};
				oc.OCMode = tbOcModeConst[m];
				oc.OCFastMode = TIM_OCFAST_DISABLE;
				CHECK(HAL_TIM_PWM_ConfigChannel(&r.h, &oc, tbCh(c)) == HAL_OK);
				tbAcceptConfigFaults(tim);
				const uint32_t now = tbRd(tbCcmr(r, c)) >> tbShift(c);
				CHECK((now & 3u) == 0u);
				CHECK(((now >> 4) & 7u) == m);
				CHECK((now & 8u) == 8u);  // OCxPE
				CHECK((now & 4u) == 0u);  // OCxFE
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB3 PWM ConfigChannel sets and clears the fast mode bit OCxFE as requested") {
	for (unsigned tim : tbChanTims)
		for (unsigned c = 1; c <= tbI(tim).nch; ++c) {
			CAPTURE(tim);
			CAPTURE(c);
			TbRig r(tim);
			REQUIRE(HAL_TIM_PWM_Init(&r.h) == HAL_OK);
			TIM_OC_InitTypeDef oc{};
			oc.OCMode = TIM_OCMODE_PWM1;
			oc.OCFastMode = TIM_OCFAST_ENABLE;
			REQUIRE(HAL_TIM_PWM_ConfigChannel(&r.h, &oc, tbCh(c)) == HAL_OK);
			CHECK(((tbRd(tbCcmr(r, c)) >> tbShift(c)) & 4u) == 4u);
			oc.OCFastMode = TIM_OCFAST_DISABLE;
			REQUIRE(HAL_TIM_PWM_ConfigChannel(&r.h, &oc, tbCh(c)) == HAL_OK);
			CHECK(((tbRd(tbCcmr(r, c)) >> tbShift(c)) & 4u) == 0u);
			tbAcceptConfigFaults(tim);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB3 OC ConfigChannel does not change the preload enable or the fast mode bit") {
	for (unsigned c = 1; c <= 4; ++c) {
		CAPTURE(c);
		TbRig r(1);
		REQUIRE(HAL_TIM_OC_Init(&r.h) == HAL_OK);
		TIM_OC_InitTypeDef oc{};
		oc.OCMode = TIM_OCMODE_TOGGLE;
		oc.OCFastMode = TIM_OCFAST_ENABLE;
		REQUIRE(HAL_TIM_OC_ConfigChannel(&r.h, &oc, tbCh(c)) == HAL_OK);
		CHECK(((tbRd(tbCcmr(r, c)) >> tbShift(c)) & 0xCu) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB3 ConfigChannel writes Pulse to CCRx of that channel only (OC and PWM)") {
	for (const TbFam &f : tbFams)
		for (unsigned tim : tbChanTims)
			for (unsigned c = 1; c <= tbI(tim).nch; ++c) {
				CAPTURE(f.name);
				CAPTURE(tim);
				CAPTURE(c);
				TbRig r(tim);
				REQUIRE(f.init(&r.h) == HAL_OK);
				for (unsigned k = 1; k <= tbI(tim).nch; ++k) *tbCcr(r, k) = 0x100u + k;
				TIM_OC_InitTypeDef oc{};
				oc.OCMode = f.pwm ? TIM_OCMODE_PWM1 : TIM_OCMODE_TIMING;
				oc.Pulse = tbI(tim).is32 ? 0x89ABCDEFu : 0x7654u; // CCRx of TIM2 / TIM5 is 32 bit (RM0090 18.4.15)
				REQUIRE(f.config(&r.h, &oc, tbCh(c)) == HAL_OK);
				tbAcceptConfigFaults(tim);
				for (unsigned k = 1; k <= tbI(tim).nch; ++k) CHECK(tbRd(tbCcr(r, k)) == (k == c ? oc.Pulse : 0x100u + k));
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB3 ConfigChannel writes the output polarity CCxP (low = 1) and clears it for high (OC and PWM)") {
	for (const TbFam &f : tbFams)
		for (unsigned tim : tbChanTims)
			for (unsigned c = 1; c <= tbI(tim).nch; ++c) {
				CAPTURE(f.name);
				CAPTURE(tim);
				CAPTURE(c);
				TbRig r(tim);
				REQUIRE(f.init(&r.h) == HAL_OK);
				const uint32_t pbit = 2u << (4 * (c - 1));
				TIM_OC_InitTypeDef oc{};
				oc.OCMode = f.pwm ? TIM_OCMODE_PWM1 : TIM_OCMODE_TOGGLE;
				oc.OCPolarity = TIM_OCPOLARITY_LOW;
				REQUIRE(f.config(&r.h, &oc, tbCh(c)) == HAL_OK);
				CHECK((r.ccer() & pbit) == pbit);
				oc.OCPolarity = TIM_OCPOLARITY_HIGH;
				REQUIRE(f.config(&r.h, &oc, tbCh(c)) == HAL_OK);
				CHECK((r.ccer() & pbit) == 0u);
				tbAcceptConfigFaults(tim);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB3 ConfigChannel writes the complementary polarity CCxNP and the idle states OISx / OISxN on TIM1 and TIM8") {
	for (const TbFam &f : tbFams)
		for (unsigned tim : {1u, 8u})
			for (unsigned c = 1; c <= 4; ++c) {
				CAPTURE(f.name);
				CAPTURE(tim);
				CAPTURE(c);
				TbRig r(tim);
				REQUIRE(f.init(&r.h) == HAL_OK);
				const uint32_t npbit = 8u << (4 * (c - 1));
				const uint32_t oisbit = 0x100u << (2 * (c - 1)), oisnbit = 0x200u << (2 * (c - 1));
				TIM_OC_InitTypeDef oc{};
				oc.OCMode = f.pwm ? TIM_OCMODE_PWM1 : TIM_OCMODE_TOGGLE;
				oc.OCNPolarity = TIM_OCNPOLARITY_LOW;
				oc.OCIdleState = TIM_OCIDLESTATE_SET;
				oc.OCNIdleState = TIM_OCNIDLESTATE_SET;
				REQUIRE(f.config(&r.h, &oc, tbCh(c)) == HAL_OK);
				if (c <= 3) {
					CHECK((r.ccer() & npbit) == npbit);
					CHECK((tbRd(&r.t->CR2) & oisnbit) == oisnbit);
				} else {
					CHECK((r.ccer() & 0x8000u) == 0u); // channel 4 has no complementary output: CC4NP not written
				}
				CHECK((tbRd(&r.t->CR2) & oisbit) == oisbit);
				oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
				oc.OCIdleState = TIM_OCIDLESTATE_RESET;
				oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
				REQUIRE(f.config(&r.h, &oc, tbCh(c)) == HAL_OK);
				CHECK((r.ccer() & npbit) == 0u);
				CHECK((tbRd(&r.t->CR2) & (oisbit | (c <= 3 ? oisnbit : 0u))) == 0u);
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB3 ConfigChannel does not touch CR2 or the complementary polarity bits of the timers without break feature") {
	for (const TbFam &f : tbFams)
		for (unsigned tim : {2u, 3u, 4u, 5u, 9u, 12u})
			for (unsigned c = 1; c <= tbI(tim).nch; ++c) {
				CAPTURE(f.name);
				CAPTURE(tim);
				CAPTURE(c);
				TbRig r(tim);
				REQUIRE(f.init(&r.h) == HAL_OK);
				r.t->CR2 = 0x70u; // bits that exist on TIM2-5 (0xF8) and TIM9 / TIM12 (0x70)
				r.t->CCER = 0xFFFFu;
				const uint32_t cr2 = r.t->CR2 ;
				const uint32_t npbit = 8u << (4 * (c - 1));
				const uint32_t ccerBefore = r.ccer();
				TIM_OC_InitTypeDef oc{};
				oc.OCMode = f.pwm ? TIM_OCMODE_PWM1 : TIM_OCMODE_TOGGLE;
				oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
				oc.OCNIdleState = TIM_OCNIDLESTATE_SET;
				oc.OCIdleState = TIM_OCIDLESTATE_SET;
				REQUIRE(f.config(&r.h, &oc, tbCh(c)) == HAL_OK);
				CHECK(tbRd(&r.t->CR2) == cr2);
				CHECK((r.ccer() & npbit) == (ccerBefore & npbit));
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB3 ConfigChannel turns an input channel (CCxS != 0, channel off) into an output with a writable CCRx") {
	for (unsigned tim : {1u, 3u, 5u, 9u}) {
		CAPTURE(tim);
		TbRig r(tim);
		REQUIRE(HAL_TIM_OC_Init(&r.h) == HAL_OK);
		r.t->CCMR1 = 0x0101u; // CC1S = CC2S = 01 (input, TI1 / TI2)
		TIM_OC_InitTypeDef oc{};
		oc.OCMode = TIM_OCMODE_TOGGLE;
		oc.Pulse = 0x321u;
		REQUIRE(HAL_TIM_OC_ConfigChannel(&r.h, &oc, TIM_CHANNEL_1) == HAL_OK);
		CHECK((tbRd(&r.t->CCMR1) & 3u) == 0u);
		CHECK((tbRd(&r.t->CCMR1) & 0x0300u) == 0x0100u); // channel 2 stays an input
		CHECK(tbRd(&r.t->CCR1) == 0x321u);
		CHECK(hostTimProtectedWrites(tim) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TB3 ConfigChannel keeps CCxE of a running channel and does not touch the other channels' CCxE (NEEDS REVIEW 332/f)") {
	for (const TbFam &f : tbFams)
		for (unsigned c = 1; c <= 4; ++c) {
			CAPTURE(f.name);
			CAPTURE(c);
			TbRig r(3);
			REQUIRE(f.init(&r.h) == HAL_OK);
			r.t->CCER = 0x1111u;
			TIM_OC_InitTypeDef oc{};
			oc.OCMode = f.pwm ? TIM_OCMODE_PWM1 : TIM_OCMODE_TOGGLE;
			REQUIRE(f.config(&r.h, &oc, tbCh(c)) == HAL_OK);
			CHECK((r.ccer() & 0x1111u) == 0x1111u);
		}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TB3 ConfigChannel of channels 1-3 of TIM1 and TIM8 clears CCxNE of that channel only (NEEDS REVIEW 332/g)") {
	for (const TbFam &f : tbFams)
		for (unsigned tim : {1u, 8u})
			for (unsigned c = 1; c <= 3; ++c) {
				CAPTURE(f.name);
				CAPTURE(tim);
				CAPTURE(c);
				TbRig r(tim);
				REQUIRE(f.init(&r.h) == HAL_OK);
				r.t->CCER = 0x0444u; // CC1NE, CC2NE, CC3NE
				TIM_OC_InitTypeDef oc{};
				oc.OCMode = f.pwm ? TIM_OCMODE_PWM1 : TIM_OCMODE_TOGGLE;
				REQUIRE(f.config(&r.h, &oc, tbCh(c)) == HAL_OK);
				CHECK(r.ccer() == (0x0444u & ~tbCcxne(c)));
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB3 ConfigChannel returns HAL_BUSY, writes no register and keeps the lock when the handle is locked (OC and PWM)") {
	for (const TbFam &f : tbFams)
		for (unsigned c = 1; c <= 4; ++c) {
			CAPTURE(f.name);
			CAPTURE(c);
			TbRig r(1);
			REQUIRE(f.init(&r.h) == HAL_OK);
			r.t->CCR1 = 0x11u;
			r.t->CCR2 = 0x22u;
			r.t->CCR3 = 0x33u;
			r.t->CCR4 = 0x44u;
			const uint32_t ccmr1 = tbRd(&r.t->CCMR1), ccmr2 = tbRd(&r.t->CCMR2), cr2 = tbRd(&r.t->CR2);
			r.h.Lock = HAL_LOCKED;
			TIM_OC_InitTypeDef oc{};
			oc.OCMode = f.pwm ? TIM_OCMODE_PWM2 : TIM_OCMODE_TOGGLE;
			oc.Pulse = 0x999u;
			oc.OCPolarity = TIM_OCPOLARITY_LOW;
			CHECK(f.config(&r.h, &oc, tbCh(c)) == HAL_BUSY);
			CHECK(r.h.Lock == HAL_LOCKED);
			CHECK(tbRd(&r.t->CCR1) == 0x11u);
			CHECK(tbRd(&r.t->CCR2) == 0x22u);
			CHECK(tbRd(&r.t->CCR3) == 0x33u);
			CHECK(tbRd(&r.t->CCR4) == 0x44u);
			CHECK(tbRd(&r.t->CCMR1) == ccmr1);
			CHECK(tbRd(&r.t->CCMR2) == ccmr2);
			CHECK(tbRd(&r.t->CR2) == cr2);
			CHECK(r.ccer() == 0u);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB3 ConfigChannel releases the lock after success and works on a handle that was not initialised") {
	for (const TbFam &f : tbFams) {
		CAPTURE(f.name);
		TbRig r(3);
		TIM_OC_InitTypeDef oc{};
		oc.OCMode = f.pwm ? TIM_OCMODE_PWM1 : TIM_OCMODE_TOGGLE;
		oc.Pulse = 7u;
		CHECK(f.config(&r.h, &oc, TIM_CHANNEL_2) == HAL_OK);
		CHECK(r.h.Lock == HAL_UNLOCKED);
		CHECK(tbRd(&r.t->CCR2) == 7u);
		CHECK(r.h.State == HAL_TIM_STATE_RESET);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TB3 ConfigChannel with an invalid channel returns HAL_ERROR, writes no register and releases the lock (NEEDS REVIEW 332/h)") {
	for (const TbFam &f : tbFams) {
		CAPTURE(f.name);
		TbRig r(1);
		REQUIRE(f.init(&r.h) == HAL_OK);
		const TbSnap s(r);
		const uint32_t ccmr1 = tbRd(&r.t->CCMR1), ccmr2 = tbRd(&r.t->CCMR2), cr2 = tbRd(&r.t->CR2);
		TIM_OC_InitTypeDef oc{};
		oc.OCMode = f.pwm ? TIM_OCMODE_PWM1 : TIM_OCMODE_TOGGLE;
		oc.Pulse = 0x55u;
		CHECK(f.config(&r.h, &oc, TIM_CHANNEL_ALL) == HAL_ERROR);
		CHECK(r.h.Lock == HAL_UNLOCKED);
		CHECK(s.same(r));
		CHECK(tbRd(&r.t->CCMR1) == ccmr1);
		CHECK(tbRd(&r.t->CCMR2) == ccmr2);
		CHECK(tbRd(&r.t->CR2) == cr2);
		CHECK(tbRd(&r.t->CCR1) == s.ccr1);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB3 a configured PWM channel runs end to end: compare match sets CCxIF, the handler calls the pulse finished callback") {
	for (unsigned tim : {1u, 3u, 9u, 10u}) {
		CAPTURE(tim);
		TbRig r(tim);
		r.h.Init.Period = 99u;
		r.h.Init.Prescaler = 0u;
		REQUIRE(HAL_TIM_PWM_Init(&r.h) == HAL_OK);
		TIM_OC_InitTypeDef oc{};
		oc.OCMode = TIM_OCMODE_PWM1;
		oc.Pulse = 30u;
		REQUIRE(HAL_TIM_PWM_ConfigChannel(&r.h, &oc, TIM_CHANNEL_1) == HAL_OK);
		tbAcceptConfigFaults(tim);
		REQUIRE(HAL_TIM_PWM_Start_IT(&r.h, TIM_CHANNEL_1) == HAL_OK);
		hostTimAdvance(tim, 30);
		CHECK(hostTimIrqPending(tim) == tbCcie(1));
		HAL_TIM_IRQHandler(&r.h);
		CHECK(timCounts.pwmPulse == 1u);
		CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_1);
		CHECK(r.h.Channel == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
	}
}

/* ================================================================ TB4: GetState / GetChannelState / GetActiveChannel */

TEST_CASE_FIXTURE(TimFixture, "TB4 the Base, OC and PWM GetState functions return the handle state for every state value") {
	const HAL_TIM_StateTypeDef sts[5] = {HAL_TIM_STATE_RESET, HAL_TIM_STATE_READY, HAL_TIM_STATE_BUSY, HAL_TIM_STATE_TIMEOUT, HAL_TIM_STATE_ERROR};
	TbRig r(3);
	for (HAL_TIM_StateTypeDef s : sts) {
		CAPTURE(static_cast<int>(s));
		r.h.State = s;
		CHECK(HAL_TIM_Base_GetState(&r.h) == s);
		CHECK(HAL_TIM_OC_GetState(&r.h) == s);
		CHECK(HAL_TIM_PWM_GetState(&r.h) == s);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB4 GetState follows the state: RESET before Init, READY after Init, BUSY after Start, READY after Stop (base, OC, PWM)") {
	for (const TbInitFam &f : tbInitFams) {
		CAPTURE(f.name);
		TbRig r(3);
		CHECK(HAL_TIM_Base_GetState(&r.h) == HAL_TIM_STATE_RESET);
		REQUIRE(f.init(&r.h) == HAL_OK);
		CHECK(HAL_TIM_Base_GetState(&r.h) == HAL_TIM_STATE_READY);
		if (f.msp == 0) {
			REQUIRE(HAL_TIM_Base_Start(&r.h) == HAL_OK);
			CHECK(HAL_TIM_Base_GetState(&r.h) == HAL_TIM_STATE_BUSY);
			REQUIRE(HAL_TIM_Base_Stop(&r.h) == HAL_OK);
			CHECK(HAL_TIM_Base_GetState(&r.h) == HAL_TIM_STATE_READY);
		}
		REQUIRE(f.deinit(&r.h) == HAL_OK);
		CHECK(HAL_TIM_Base_GetState(&r.h) == HAL_TIM_STATE_RESET);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB4 GetChannelState returns the state of channels 1-4 independently") {
	const HAL_TIM_ChannelStateTypeDef sts[3] = {HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_READY, HAL_TIM_CHANNEL_STATE_BUSY};
	TbRig r(1);
	for (unsigned c = 1; c <= 4; ++c)
		for (HAL_TIM_ChannelStateTypeDef s : sts) {
			CAPTURE(c);
			CAPTURE(static_cast<int>(s));
			for (unsigned k = 0; k < 4; ++k) r.h.ChannelState[k] = HAL_TIM_CHANNEL_STATE_RESET;
			r.h.ChannelState[c - 1] = s;
			for (unsigned k = 1; k <= 4; ++k) CHECK(HAL_TIM_GetChannelState(&r.h, tbCh(k)) == (k == c ? s : HAL_TIM_CHANNEL_STATE_RESET));
		}
}

TEST_CASE_FIXTURE(TimFixture, "TB4 GetChannelState follows Init, Start, Stop and DeInit (OC and PWM)") {
	for (const TbFam &f : tbFams) {
		CAPTURE(f.name);
		TbRig r(1);
		CHECK(HAL_TIM_GetChannelState(&r.h, TIM_CHANNEL_2) == HAL_TIM_CHANNEL_STATE_RESET);
		REQUIRE(f.init(&r.h) == HAL_OK);
		CHECK(HAL_TIM_GetChannelState(&r.h, TIM_CHANNEL_2) == HAL_TIM_CHANNEL_STATE_READY);
		REQUIRE(f.start(&r.h, TIM_CHANNEL_2) == HAL_OK);
		CHECK(HAL_TIM_GetChannelState(&r.h, TIM_CHANNEL_2) == HAL_TIM_CHANNEL_STATE_BUSY);
		CHECK(HAL_TIM_GetChannelState(&r.h, TIM_CHANNEL_1) == HAL_TIM_CHANNEL_STATE_READY);
		REQUIRE(f.stop(&r.h, TIM_CHANNEL_2) == HAL_OK);
		CHECK(HAL_TIM_GetChannelState(&r.h, TIM_CHANNEL_2) == HAL_TIM_CHANNEL_STATE_READY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "characterization: TB4 GetChannelState with an invalid channel returns the state of channel 4 (NEEDS REVIEW 332/i)") {
	TbRig r(1);
	r.h.ChannelState[3] = HAL_TIM_CHANNEL_STATE_BUSY;
	CHECK(HAL_TIM_GetChannelState(&r.h, TIM_CHANNEL_ALL) == HAL_TIM_CHANNEL_STATE_BUSY);
}

TEST_CASE_FIXTURE(TimFixture, "TB4 GetActiveChannel returns the active channel of the handle for every value") {
	const HAL_TIM_ActiveChannel v[5] = {HAL_TIM_ACTIVE_CHANNEL_1, HAL_TIM_ACTIVE_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_3, HAL_TIM_ACTIVE_CHANNEL_4, HAL_TIM_ACTIVE_CHANNEL_CLEARED};
	TbRig r(1);
	for (HAL_TIM_ActiveChannel a : v) {
		CAPTURE(static_cast<int>(a));
		r.h.Channel = a;
		CHECK(HAL_TIM_GetActiveChannel(&r.h) == a);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TB4 the active channel is CLEARED after Init and after a capture / compare interrupt was served") {
	TbRig r(3);
	r.h.Channel = HAL_TIM_ACTIVE_CHANNEL_4;
	REQUIRE(HAL_TIM_OC_Init(&r.h) == HAL_OK);
	CHECK(HAL_TIM_GetActiveChannel(&r.h) == HAL_TIM_ACTIVE_CHANNEL_4); // Init does not reset the field
	hostTimRaiseFlags(3, 1u << 2);
	r.t->DIER = tbCcie(2);
	HAL_TIM_IRQHandler(&r.h);
	CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_2);
	CHECK(HAL_TIM_GetActiveChannel(&r.h) == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
}

/* ================================================================ fix pass: Stop_DMA of every channel, one pulse idle states */

TEST_CASE_FIXTURE(TimFixture, "TB2 Stop_DMA of channel c aborts the stream of that channel only, not the other channels or the update stream (OC and PWM)") {
	for (const TbFam &f : tbFams)
		for (unsigned tim : {1u, 8u})
			for (unsigned c = 1; c <= 4; ++c) {
				CAPTURE(f.name);
				CAPTURE(tim);
				CAPTURE(c);
				TbRig r(tim);
				REQUIRE(f.init(&r.h) == HAL_OK);
				REQUIRE(HAL_TIM_Base_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
				for (unsigned k = 1; k <= 4; ++k) REQUIRE(f.startDma(&r.h, tbCh(k), r.buf, 4) == HAL_OK);
				CHECK(f.stopDma(&r.h, tbCh(c)) == HAL_OK);
				for (unsigned k = 1; k <= 4; ++k) {
					CAPTURE(k);
					CHECK(((r.stream(k)->CR & tbDMA_EN) != 0u) == (k != c));
					CHECK(((r.dier() & tbCcde(k)) != 0u) == (k != c));
					CHECK(r.state(k) == (k == c ? HAL_TIM_CHANNEL_STATE_READY : HAL_TIM_CHANNEL_STATE_BUSY));
				}
				CHECK((r.stream(0)->CR & tbDMA_EN) == tbDMA_EN);
				CHECK((r.dier() & (1u << 8)) == (1u << 8));
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB3 OnePulse ConfigChannel writes the output idle states OIS1 and OIS1N of TIM1 as requested") {
	for (int ois = 0; ois < 2; ++ois)
		for (int oisn = 0; oisn < 2; ++oisn) {
			CAPTURE(ois);
			CAPTURE(oisn);
			TbRig r(1);
			REQUIRE(HAL_TIM_OnePulse_Init(&r.h, TIM_OPMODE_SINGLE) == HAL_OK);
			TIM_OnePulse_InitTypeDef cfg{};
			cfg.OCMode = TIM_OCMODE_PWM2;
			cfg.Pulse = 10u;
			cfg.OCIdleState = ois ? TIM_OCIDLESTATE_SET : TIM_OCIDLESTATE_RESET;
			cfg.OCNIdleState = oisn ? TIM_OCNIDLESTATE_SET : TIM_OCNIDLESTATE_RESET;
			cfg.ICPolarity = TIM_ICPOLARITY_RISING;
			cfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
			REQUIRE(HAL_TIM_OnePulse_ConfigChannel(&r.h, &cfg, TIM_CHANNEL_1, TIM_CHANNEL_2) == HAL_OK);
			CHECK(((tbRd(&r.t->CR2) >> 8) & 1u) == static_cast<uint32_t>(ois));   // OIS1
			CHECK(((tbRd(&r.t->CR2) >> 9) & 1u) == static_cast<uint32_t>(oisn));  // OIS1N
		}
}

/* ================================================================ fix pass 2 */

TEST_CASE_FIXTURE(TimFixture, "TB3 ConfigChannel of a running input channel turns it into an output (CCxS cleared, CCRx written), keeps its CCxE and every other CCxE") {
	// RM0090: CCxS is writable only while the channel is off, and CCRx of an input channel is read only: the configuration must switch the channel off while it writes.
	for (const TbFam &f : tbFams)
		for (unsigned tim : {1u, 3u})
			for (unsigned c = 1; c <= 4; ++c) {
				CAPTURE(f.name);
				CAPTURE(tim);
				CAPTURE(c);
				TbRig r(tim);
				REQUIRE(f.init(&r.h) == HAL_OK);
				r.t->CCMR1 = 0x0101u; // CC1S = CC2S = 01 (input), channel off
				r.t->CCMR2 = 0x0101u;
				r.t->CCER = 0x1111u;  // all four channels on
				TIM_OC_InitTypeDef oc{};
				oc.OCMode = f.pwm ? TIM_OCMODE_PWM1 : TIM_OCMODE_TOGGLE;
				oc.Pulse = 0x321u;
				REQUIRE(f.config(&r.h, &oc, tbCh(c)) == HAL_OK);
				const uint32_t sh = tbShift(c);
				CHECK(((tbRd(tbCcmr(r, c)) >> sh) & 3u) == 0u);
				CHECK(tbRd(tbCcr(r, c)) == 0x321u);
				CHECK(hostTimProtectedWrites(tim) == 0u);
				CHECK((r.ccer() & 0x1111u) == 0x1111u);
				for (unsigned k = 1; k <= 4; ++k)
					if (k != c) CHECK(((tbRd(tbCcmr(r, k)) >> tbShift(k)) & 3u) == 1u); // the other channels stay inputs
			}
}

TEST_CASE_FIXTURE(TimFixture, "TB1 DeInit calls MspDeInit with the handle state BUSY (base, OC, PWM)") {
	for (const TbInitFam &f : tbInitFams) {
		CAPTURE(f.name);
		TbRig r(3);
		REQUIRE(f.init(&r.h) == HAL_OK);
		REQUIRE(f.deinit(&r.h) == HAL_OK);
		CHECK(timCounts.mspDeInit[f.msp] == 1u);
		CHECK(timCounts.stateAtLast == HAL_TIM_STATE_BUSY);
		CHECK(r.h.State == HAL_TIM_STATE_RESET);
	}
}
