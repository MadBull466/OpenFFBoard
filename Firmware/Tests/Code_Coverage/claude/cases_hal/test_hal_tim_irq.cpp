/*
 * P12 T6a group TQ: HAL_TIM_IRQHandler and the DMA completion / half completion / error callbacks of the real ST HAL timer driver
 * (stm32f4xx_hal_tim.c, stm32f4xx_hal_tim_ex.c), against the timer model (fakes/stm32_hal/HalTim.h) and the DMA model (HalDma.h).
 * Requirements TQ1-TQ3 of Tests/Code_Coverage/claude/charters/hal/P12_T6A_REQUIREMENTS.md; compiled into ffb_hal_tests and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 (TIM chapter: HAL_TIM_IRQHandler "TIM IRQ handler management", the callbacks, the state names) and RM0090 Rev 22:
 *  - 17.4.4 TIMx_DIER and 17.4.5 TIMx_SR (UIF bit 0, CC1IF..CC4IF bits 1..4, COMIF bit 5, TIF bit 6, BIF bit 7, CC1OF..CC4OF bits 9..12; an interrupt flag
 *    raises the interrupt only if its enable bit in DIER is set; the flags are cleared by software), 17.4.7 CCMR1 / CCMR2 (CCxS = 00 output, != 00 input).
 *  - Table 44 (DMA2): TIM1_TRIG S0 / S4, TIM1_CH1 S1 / S3, TIM1_CH2 S2, TIM1_CH3 S6, TIM1_CH4 S4, TIM1_UP S5, TIM1_COM S4, all channel 6.
 *  - 10.5.5 DMA_SxCR (TCIE bit 4, HTIE bit 3, TEIE bit 2), 10.3.14 (normal and circular mode: circular keeps running after the transfer complete).
 * The DMA streams are started by the real HAL functions (Start_DMA, DMABurst_MultiWriteStart); the tests play the DMA engine with hostDmaComplete,
 * hostDmaStepItems and hostDmaRaise and then call HAL_DMA_IRQHandler. The "flag cleared before the callback" order cannot be seen by the shared callbacks
 * (they do not record SR); the case checks the flag after the handler.
 * Every file-local symbol sits in the anonymous namespace or is prefixed tq.
 *
 * NEEDS REVIEW proposals (numbers assigned by the lead):
 *  NEEDS REVIEW 336/a TQ1: several flags at once call back in the order CC1, CC2, CC3, CC4, update, break, trigger, commutation (UM1725 and RM0090 give no order).
 *  NEEDS REVIEW 336/b TQ1: a set CCxOF flag is neither cleared nor reported by the handler (RM0090: CCxOF is cleared by software; UM1725 silent).
 *  NEEDS REVIEW 336/c TQ2: a completion callback of a stream not linked to a channel still reports the event, with no active channel.
 *  NEEDS REVIEW 336/d TQ2: in circular mode the completion callbacks leave the channel state BUSY / timer state BUSY (UM1725 silent; derived from "the stream keeps running").
 *  NEEDS REVIEW 336/e TQ2: the commutation DMA callbacks set the timer state READY even in circular mode and on a half transfer.
 */
#include "HalDma.h"
#include "HalFixture.h"
#include "HalHostSystem.h"
#include "HalTim.h"
#include "TimTestSupport.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstring>
#include <string>

namespace {

constexpr uint32_t tqUIF = 1u << 0;
constexpr uint32_t tqCC1IF = 1u << 1;
constexpr uint32_t tqCOMIF = 1u << 5;
constexpr uint32_t tqTIF = 1u << 6;
constexpr uint32_t tqBIF = 1u << 7;
constexpr uint32_t tqCC1OF = 1u << 9;
constexpr unsigned tqChannels[14] = {4, 4, 4, 4, 4, 0, 0, 4, 2, 1, 1, 2, 1, 1}; // CC channels of TIM1..TIM14 (RM0090 17 / 18 / 19 / 20)

bool tqHasTrigger(unsigned n) { return n == 1 || n == 2 || n == 3 || n == 4 || n == 5 || n == 8 || n == 9 || n == 12; }
bool tqHasBreak(unsigned n) { return n == 1 || n == 8; }

uint32_t tqCcFlag(unsigned ch) { return 1u << ch; }          // CCxIF and CCxIE are bit x
HAL_TIM_ActiveChannel tqActive(unsigned ch) { return static_cast<HAL_TIM_ActiveChannel>(1u << (ch - 1u)); } // HAL_TIM_ACTIVE_CHANNEL_1..4 = 1, 2, 4, 8
uint32_t tqChannel(unsigned ch) { return (ch - 1u) * 4u; }    // TIM_CHANNEL_1..4 = 0, 4, 8, 12

void tqSetInput(TIM_TypeDef *t, unsigned ch) { // CCxS = 01 (direct TI)
	if (ch == 1) t->CCMR1 |= 1u;
	if (ch == 2) t->CCMR1 |= 1u << 8;
	if (ch == 3) t->CCMR2 |= 1u;
	if (ch == 4) t->CCMR2 |= 1u << 8;
}

void tqClean(TIM_TypeDef *t) {
	t->DIER = 0;
	t->SR = 0;
	if (t != TIM6 && t != TIM7) t->CCMR1 = 0; // the basic timers have no CCMR1 / CCMR2 (RM0090 20.4)
	if (t == TIM1 || t == TIM8 || t == TIM2 || t == TIM3 || t == TIM4 || t == TIM5) t->CCMR2 = 0;
	timSupportReset();
}

// Raises every status flag the timer has, including CC1OF..CC4OF (SR bits 9..12), and returns SR.
uint32_t tqRaiseAll(unsigned n) {
	hostTimRaiseFlags(n, 0x1FFFu);
	return timInstance(n)->SR;
}

// The handled flag is already clear when each callback runs (the handler clears it first), every other flag is untouched.
void tqCheckCleared(uint32_t before, uint32_t handled, unsigned first, unsigned count, TIM_TypeDef *t) {
	for (unsigned i = first; i < first + count; ++i) {
		CAPTURE(i);
		CHECK(timCounts.srOrder[i] == (before & ~handled));
	}
	CHECK(timCounts.srAtLast == (before & ~handled));
	CHECK(t->SR == (before & ~handled));
}

TIM_HandleTypeDef tqHandle(unsigned n) {
	TIM_HandleTypeDef h;
	std::memset(&h, 0, sizeof h);
	h.Instance = timInstance(n);
	return h;
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------------------------- TQ1

TEST_CASE_FIXTURE(TimFixture, "TQ1 an output compare match of every channel calls the delay elapsed and pulse finished callbacks with the active channel") {
	for (unsigned n = 1; n <= 14; ++n) {
		for (unsigned ch = 1; ch <= tqChannels[n - 1]; ++ch) {
			CAPTURE(n);
			CAPTURE(ch);
			TIM_TypeDef *t = timInstance(n);
			tqClean(t);
			TIM_HandleTypeDef h = tqHandle(n);
			t->DIER = tqCcFlag(ch);
			const uint32_t before = tqRaiseAll(n);
			REQUIRE((before & tqCcFlag(ch)) != 0u);
			HAL_TIM_IRQHandler(&h);
			tqCheckCleared(before, tqCcFlag(ch), 0, 2, t);
			CHECK(timCounts.ocDelay == 1u);
			CHECK(timCounts.pwmPulse == 1u);
			CHECK(timCounts.icCapture == 0u);
			CHECK(timCounts.orderCount == 2u);
			CHECK(timCounts.order[0] == TIM_CB_OC_DELAY);
			CHECK(timCounts.order[1] == TIM_CB_PWM_PULSE);
			CHECK(timCounts.channelAtLast == tqActive(ch));
			CHECK(timCounts.last == &h);
			CHECK((t->SR & tqCcFlag(ch)) == 0u);
			CHECK(HAL_TIM_GetActiveChannel(&h) == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
		}
	}
}

TEST_CASE_FIXTURE(TimFixture, "TQ1 an input capture event of every channel calls the capture callback with the active channel and the captured value readable") {
	for (unsigned n = 1; n <= 14; ++n) {
		for (unsigned ch = 1; ch <= tqChannels[n - 1]; ++ch) {
			CAPTURE(n);
			CAPTURE(ch);
			TIM_TypeDef *t = timInstance(n);
			tqClean(t);
			TIM_HandleTypeDef h = tqHandle(n);
			tqSetInput(t, ch);
			t->DIER = tqCcFlag(ch);
			REQUIRE(hostTimCapture(n, ch, 0x1234u + ch) == 1);
			const uint32_t before = tqRaiseAll(n);
			HAL_TIM_IRQHandler(&h);
			tqCheckCleared(before, tqCcFlag(ch), 0, 1, t);
			CHECK(timCounts.icCapture == 1u);
			CHECK(timCounts.ocDelay == 0u);
			CHECK(timCounts.pwmPulse == 0u);
			CHECK(timCounts.channelAtLast == tqActive(ch));
			CHECK((t->SR & tqCcFlag(ch)) == 0u);
			CHECK(HAL_TIM_GetActiveChannel(&h) == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
			CHECK(HAL_TIM_ReadCapturedValue(&h, tqChannel(ch)) == 0x1234u + ch);
		}
	}
}

TEST_CASE_FIXTURE(TimFixture, "TQ1 the update flag of every timer calls the period elapsed callback and clears the flag") {
	for (unsigned n = 1; n <= 14; ++n) {
		CAPTURE(n);
		TIM_TypeDef *t = timInstance(n);
		tqClean(t);
		TIM_HandleTypeDef h = tqHandle(n);
		t->DIER = tqUIF;
		const uint32_t before = tqRaiseAll(n);
		HAL_TIM_IRQHandler(&h);
		tqCheckCleared(before, tqUIF, 0, 1, t);
		CHECK(timCounts.period == 1u);
		CHECK(timCounts.orderCount == 1u);
		CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
		CHECK((t->SR & tqUIF) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TQ1 the break flag of TIM1 and TIM8 calls the break callback and clears the flag") {
	const unsigned timers[2] = {1, 8};
	for (unsigned n : timers) {
		CAPTURE(n);
		TIM_TypeDef *t = timInstance(n);
		tqClean(t);
		TIM_HandleTypeDef h = tqHandle(n);
		t->DIER = tqBIF;
		hostTimBreak(n);
		CHECK(hostTimIrqPending(n) != 0u);
		const uint32_t before = tqRaiseAll(n);
		HAL_TIM_IRQHandler(&h);
		tqCheckCleared(before, tqBIF, 0, 1, t);
		CHECK(timCounts.breakCb == 1u);
		CHECK(timCounts.orderCount == 1u);
		CHECK((t->SR & tqBIF) == 0u);
		CHECK(hostTimIrqPending(n) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TQ1 the trigger flag of every timer with a slave mode controller calls the trigger callback and clears the flag") {
	for (unsigned n = 1; n <= 14; ++n) {
		if (!tqHasTrigger(n)) continue;
		CAPTURE(n);
		TIM_TypeDef *t = timInstance(n);
		tqClean(t);
		TIM_HandleTypeDef h = tqHandle(n);
		t->DIER = tqTIF;
		const uint32_t before = tqRaiseAll(n);
		HAL_TIM_IRQHandler(&h);
		tqCheckCleared(before, tqTIF, 0, 1, t);
		CHECK(timCounts.trigger == 1u);
		CHECK(timCounts.orderCount == 1u);
		CHECK((t->SR & tqTIF) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TQ1 the commutation flag of TIM1 and TIM8 calls the commutation callback and clears the flag") {
	const unsigned timers[2] = {1, 8};
	for (unsigned n : timers) {
		CAPTURE(n);
		TIM_TypeDef *t = timInstance(n);
		tqClean(t);
		TIM_HandleTypeDef h = tqHandle(n);
		t->DIER = tqCOMIF;
		const uint32_t before = tqRaiseAll(n);
		HAL_TIM_IRQHandler(&h);
		tqCheckCleared(before, tqCOMIF, 0, 1, t);
		CHECK(timCounts.commut == 1u);
		CHECK(timCounts.orderCount == 1u);
		CHECK((t->SR & tqCOMIF) == 0u);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TQ1 a flag whose interrupt is not enabled in DIER calls nothing and stays set") {
	for (unsigned n = 1; n <= 14; ++n) {
		TIM_TypeDef *t = timInstance(n);
		uint32_t flags = tqUIF;
		for (unsigned ch = 1; ch <= tqChannels[n - 1]; ++ch) flags |= tqCcFlag(ch);
		if (tqHasTrigger(n)) flags |= tqTIF;
		if (tqHasBreak(n)) flags |= tqBIF | tqCOMIF;
		CAPTURE(n);
		tqClean(t);
		TIM_HandleTypeDef h = tqHandle(n);
		hostTimRaiseFlags(n, flags);
		REQUIRE((t->SR & flags) == flags);
		HAL_TIM_IRQHandler(&h);
		CHECK(timCounts.orderCount == 0u);
		CHECK(timCounts.last == nullptr);
		CHECK((t->SR & flags) == flags);
		CHECK(HAL_TIM_GetActiveChannel(&h) == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TQ1 an enabled interrupt without its flag calls nothing") {
	for (unsigned n = 1; n <= 14; ++n) {
		TIM_TypeDef *t = timInstance(n);
		uint32_t en = tqUIF;
		for (unsigned ch = 1; ch <= tqChannels[n - 1]; ++ch) en |= tqCcFlag(ch);
		if (tqHasTrigger(n)) en |= tqTIF;
		if (tqHasBreak(n)) en |= tqBIF | tqCOMIF;
		CAPTURE(n);
		tqClean(t);
		TIM_HandleTypeDef h = tqHandle(n);
		t->DIER = en;
		HAL_TIM_IRQHandler(&h);
		CHECK(timCounts.orderCount == 0u);
		CHECK(timCounts.last == nullptr);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TQ1 an enabled flag of one source does not run the callback of another source") {
	TIM_TypeDef *t = TIM1;
	tqClean(t);
	TIM_HandleTypeDef h = tqHandle(1);
	t->DIER = tqCC1IF; // only CC1 enabled
	hostTimRaiseFlags(1, tqCcFlag(2) | tqUIF | tqBIF | tqTIF | tqCOMIF);
	HAL_TIM_IRQHandler(&h);
	CHECK(timCounts.orderCount == 0u);
	CHECK((t->SR & (tqCcFlag(2) | tqUIF | tqBIF | tqTIF | tqCOMIF)) == (tqCcFlag(2) | tqUIF | tqBIF | tqTIF | tqCOMIF));
}

TEST_CASE_FIXTURE(TimFixture, "TQ1 characterization: several flags at once call back in the order channels 1 to 4, update, break, trigger, commutation") {
	TIM_TypeDef *t = TIM1;
	tqClean(t);
	TIM_HandleTypeDef h = tqHandle(1);
	const uint32_t all = tqUIF | tqCcFlag(1) | tqCcFlag(2) | tqCcFlag(3) | tqCcFlag(4) | tqCOMIF | tqTIF | tqBIF;
	tqSetInput(t, 2); // CC2 and CC4 as inputs, CC1 and CC3 as outputs
	tqSetInput(t, 4);
	t->DIER = all;
	hostTimRaiseFlags(1, all);
	HAL_TIM_IRQHandler(&h);
	const unsigned expect[] = {TIM_CB_OC_DELAY, TIM_CB_PWM_PULSE, TIM_CB_IC_CAPTURE, TIM_CB_OC_DELAY, TIM_CB_PWM_PULSE,
	                           TIM_CB_IC_CAPTURE, TIM_CB_PERIOD, TIM_CB_BREAK, TIM_CB_TRIGGER, TIM_CB_COMMUT};
	REQUIRE(timCounts.orderCount == sizeof expect / sizeof expect[0]);
	for (unsigned i = 0; i < timCounts.orderCount; ++i) {
		CAPTURE(i);
		CHECK(timCounts.order[i] == expect[i]);
	}
	CHECK((t->SR & all) == 0u);
	CHECK(HAL_TIM_GetActiveChannel(&h) == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
}

TEST_CASE_FIXTURE(TimFixture, "TQ1 characterization: the overcapture flag stays set after the handler and raises no callback by itself") {
	TIM_TypeDef *t = TIM2;
	tqClean(t);
	TIM_HandleTypeDef h = tqHandle(2);
	tqSetInput(t, 1);
	t->DIER = tqCC1IF;
	REQUIRE(hostTimCapture(2, 1, 10) == 1);
	REQUIRE(hostTimCapture(2, 1, 20) == 1); // second capture before the flag was cleared: CC1OF
	REQUIRE((t->SR & tqCC1OF) != 0u);
	HAL_TIM_IRQHandler(&h);
	CHECK(timCounts.icCapture == 1u);
	CHECK((t->SR & tqCC1IF) == 0u);
	CHECK((t->SR & tqCC1OF) != 0u);
	timSupportReset();
	HAL_TIM_IRQHandler(&h); // CC1OF alone: nothing
	CHECK(timCounts.orderCount == 0u);
}

TEST_CASE_FIXTURE(TimFixture, "TQ3 a second call of the handler after the flag was cleared calls no callback again") {
	TIM_TypeDef *t = TIM3;
	tqClean(t);
	TIM_HandleTypeDef h = tqHandle(3);
	t->DIER = tqUIF | tqCcFlag(1);
	hostTimRaiseFlags(3, tqUIF | tqCcFlag(1));
	HAL_TIM_IRQHandler(&h);
	REQUIRE(timCounts.orderCount == 3u);
	HAL_TIM_IRQHandler(&h);
	CHECK(timCounts.orderCount == 3u);
	CHECK(timCounts.period == 1u);
	CHECK(timCounts.pwmPulse == 1u);
}

TEST_CASE_FIXTURE(TimFixture, "TQ1 the handler touches no register of a basic timer beyond DIER and SR") {
	TIM_TypeDef *t = TIM6;
	tqClean(t);
	TIM_HandleTypeDef h = tqHandle(6);
	t->DIER = 1u;
	hostTimRaiseFlags(6, tqUIF);
	HAL_TIM_IRQHandler(&h);
	CHECK(timCounts.period == 1u);
	CHECK(HalFixture::faults() == 0u);
}

// ---------------------------------------------------------------------------------------------------------------------------------- TQ2

namespace {

// DMA2 streams of TIM1 requests (RM0090 Table 44, channel 6): UP S5, CC1 S1, CC2 S2, CC3 S6, CC4 S4, COM S4, TRIG S0.
struct TqRoute {
	uint16_t id;
	unsigned stream;
};
const TqRoute tqRoutes[7] = {{TIM_DMA_ID_UPDATE, 5}, {TIM_DMA_ID_CC1, 1}, {TIM_DMA_ID_CC2, 2}, {TIM_DMA_ID_CC3, 6},
                             {TIM_DMA_ID_CC4, 4}, {TIM_DMA_ID_COMMUTATION, 4}, {TIM_DMA_ID_TRIGGER, 0}};

DMA_Stream_TypeDef *tqStream(unsigned s) {
	DMA_Stream_TypeDef *const t[8] = {DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7};
	return t[s];
}

struct TqRig : TimFixture {
	TIM_HandleTypeDef h;
	DMA_HandleTypeDef d[7];
	uint32_t buf[8];
	TqRig() {
		std::memset(&h, 0, sizeof h);
		std::memset(d, 0, sizeof d);
		std::memset(buf, 0, sizeof buf);
		h.Instance = TIM1;
		h.Init.Period = 999;
	}
	DMA_HandleTypeDef &link(uint16_t id, bool circular, bool toMemory) {
		unsigned stream = 0;
		for (const TqRoute &r : tqRoutes)
			if (r.id == id) stream = r.stream;
		DMA_HandleTypeDef &dm = d[id];
		dm.Instance = tqStream(stream);
		dm.Init.Channel = DMA_CHANNEL_6;
		dm.Init.Direction = toMemory ? DMA_PERIPH_TO_MEMORY : DMA_MEMORY_TO_PERIPH;
		dm.Init.PeriphInc = DMA_PINC_DISABLE;
		dm.Init.MemInc = DMA_MINC_ENABLE;
		dm.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		dm.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		dm.Init.Mode = circular ? DMA_CIRCULAR : DMA_NORMAL;
		dm.Init.Priority = DMA_PRIORITY_LOW;
		dm.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		REQUIRE(HAL_DMA_Init(&dm) == HAL_OK);
		h.hdma[id] = &dm;
		dm.Parent = &h;
		return dm;
	}
	unsigned streamOf(uint16_t id) const {
		for (const TqRoute &r : tqRoutes)
			if (r.id == id) return r.stream;
		return 0;
	}
	void irq(uint16_t id) { HAL_DMA_IRQHandler(&d[id]); }
	void complete(uint16_t id) {
		hostDmaComplete(1, streamOf(id));
		irq(id);
	}
	void half(uint16_t id) {
		hostDmaStepItems(1, streamOf(id), 2); // 4 items: the half flag at 2
		irq(id);
	}
	void error(uint16_t id) {
		hostDmaRaise(1, streamOf(id), HOST_DMA_TEIF);
		irq(id);
	}
};

enum TqKind { tqOC, tqPWM, tqIC };

} // namespace

TEST_CASE_FIXTURE(TqRig, "TQ2 the base DMA transfer complete calls the period elapsed callback and sets the timer READY in normal mode") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_UPDATE, false, false);
	REQUIRE(HAL_TIM_Base_Start_DMA(&h, buf, 4) == HAL_OK);
	REQUIRE(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_BUSY);
	complete(TIM_DMA_ID_UPDATE);
	CHECK(timCounts.period == 1u);
	CHECK(timCounts.orderCount == 1u);
	CHECK(timCounts.last == &h);
	CHECK(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_READY);
	CHECK(timCounts.stateAtLast == HAL_TIM_STATE_READY); // set before the callback
}

TEST_CASE_FIXTURE(TqRig, "TQ2 characterization: the base DMA transfer complete of a circular stream keeps the timer BUSY") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_UPDATE, true, false);
	REQUIRE(HAL_TIM_Base_Start_DMA(&h, buf, 4) == HAL_OK);
	complete(TIM_DMA_ID_UPDATE);
	CHECK(timCounts.period == 1u);
	CHECK(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_BUSY);
	complete(TIM_DMA_ID_UPDATE); // circular: every transfer complete calls back
	CHECK(timCounts.period == 2u);
	CHECK(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_BUSY);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 the base DMA half transfer calls the period elapsed half callback and leaves the timer BUSY") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_UPDATE, false, false);
	REQUIRE(HAL_TIM_Base_Start_DMA(&h, buf, 4) == HAL_OK);
	half(TIM_DMA_ID_UPDATE);
	CHECK(timCounts.periodHalf == 1u);
	CHECK(timCounts.period == 0u);
	CHECK(timCounts.order[0] == TIM_CB_PERIOD_HALF);
	CHECK(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_BUSY);
	complete(TIM_DMA_ID_UPDATE);
	CHECK(timCounts.period == 1u);
	CHECK(timCounts.orderCount == 2u);
	CHECK(timCounts.order[1] == TIM_CB_PERIOD);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 the base DMA error calls the error callback and sets the timer READY") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_UPDATE, false, false);
	REQUIRE(HAL_TIM_Base_Start_DMA(&h, buf, 4) == HAL_OK);
	error(TIM_DMA_ID_UPDATE);
	CHECK(timCounts.error == 1u);
	CHECK(timCounts.orderCount == 1u);
	CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
	CHECK(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_READY);
	CHECK(HAL_DMA_GetError(&d[TIM_DMA_ID_UPDATE]) == HAL_DMA_ERROR_TE);
}

namespace {

// Starts a channel DMA of the given kind with the real HAL. TIM1 channel ch (1..4).
void tqStartChannel(TqRig &r, TqKind kind, unsigned ch, bool circular) {
	const uint16_t id = static_cast<uint16_t>(ch); // TIM_DMA_ID_CC1..CC4 = 1..4
	r.link(id, circular, kind == tqIC);
	if (kind == tqOC) {
		REQUIRE(HAL_TIM_OC_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_OC_Start_DMA(&r.h, tqChannel(ch), r.buf, 4) == HAL_OK);
	} else if (kind == tqPWM) {
		REQUIRE(HAL_TIM_PWM_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_PWM_Start_DMA(&r.h, tqChannel(ch), r.buf, 4) == HAL_OK);
	} else {
		REQUIRE(HAL_TIM_IC_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIM_IC_Start_DMA(&r.h, tqChannel(ch), r.buf, 4) == HAL_OK);
	}
}

} // namespace

TEST_CASE_FIXTURE(TimFixture, "TQ2 the OC and PWM DMA transfer complete calls the pulse finished callback with the active channel and sets the channel READY") {
	for (int k = 0; k < 2; ++k)
		for (unsigned ch = 1; ch <= 4; ++ch) {
			CAPTURE(k);
			CAPTURE(ch);
			timSupportReset();
			hostDmaResetState();
			TqRig r;
			tqStartChannel(r, k == 0 ? tqOC : tqPWM, ch, false);
			REQUIRE(HAL_TIM_GetChannelState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_BUSY);
			r.complete(static_cast<uint16_t>(ch));
			CHECK(timCounts.pwmPulse == 1u);
			CHECK(timCounts.ocDelay == 0u);
			CHECK(timCounts.orderCount == 1u);
			CHECK(timCounts.channelAtLast == tqActive(ch));
			CHECK(HAL_TIM_GetChannelState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_READY);
			CHECK(HAL_TIM_GetActiveChannel(&r.h) == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TQ2 characterization: the OC and PWM DMA transfer complete of a circular stream keeps the channel BUSY") {
	for (int k = 0; k < 2; ++k)
		for (unsigned ch = 1; ch <= 4; ++ch) {
			CAPTURE(k);
			CAPTURE(ch);
			timSupportReset();
			TqRig r;
			tqStartChannel(r, k == 0 ? tqOC : tqPWM, ch, true);
			r.complete(static_cast<uint16_t>(ch));
			CHECK(timCounts.pwmPulse == 1u);
			CHECK(timCounts.channelAtLast == tqActive(ch));
			CHECK(HAL_TIM_GetChannelState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_BUSY);
			CHECK(HAL_TIM_GetActiveChannel(&r.h) == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TQ2 the OC and PWM DMA half transfer calls the pulse finished half callback with the active channel and leaves the channel BUSY") {
	for (int k = 0; k < 2; ++k)
		for (unsigned ch = 1; ch <= 4; ++ch) {
			CAPTURE(k);
			CAPTURE(ch);
			timSupportReset();
			TqRig r;
			tqStartChannel(r, k == 0 ? tqOC : tqPWM, ch, false);
			r.half(static_cast<uint16_t>(ch));
			CHECK(timCounts.pwmPulseHalf == 1u);
			CHECK(timCounts.pwmPulse == 0u);
			CHECK(timCounts.orderCount == 1u);
			CHECK(timCounts.channelAtLast == tqActive(ch));
			CHECK(HAL_TIM_GetChannelState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_BUSY);
			CHECK(HAL_TIM_GetActiveChannel(&r.h) == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TQ2 the OC and PWM DMA error calls the error callback with the active channel and sets the channel READY") {
	for (int k = 0; k < 2; ++k)
		for (unsigned ch = 1; ch <= 4; ++ch) {
			CAPTURE(k);
			CAPTURE(ch);
			timSupportReset();
			TqRig r;
			tqStartChannel(r, k == 0 ? tqOC : tqPWM, ch, false);
			r.error(static_cast<uint16_t>(ch));
			CHECK(timCounts.error == 1u);
			CHECK(timCounts.orderCount == 1u);
			CHECK(timCounts.channelAtLast == tqActive(ch));
			CHECK(HAL_TIM_GetChannelState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_READY);
			CHECK(HAL_TIM_GetActiveChannel(&r.h) == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TQ2 the IC DMA transfer complete calls the capture callback with the active channel and sets both channel states READY") {
	for (unsigned ch = 1; ch <= 4; ++ch) {
		CAPTURE(ch);
		timSupportReset();
		TqRig r;
		tqStartChannel(r, tqIC, ch, false);
		REQUIRE(HAL_TIM_GetChannelState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_BUSY);
		r.complete(static_cast<uint16_t>(ch));
		CHECK(timCounts.icCapture == 1u);
		CHECK(timCounts.orderCount == 1u);
		CHECK(timCounts.channelAtLast == tqActive(ch));
		CHECK(HAL_TIM_GetChannelState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_READY);
		CHECK(HAL_TIMEx_GetChannelNState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_READY);
		CHECK(HAL_TIM_GetActiveChannel(&r.h) == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TQ2 characterization: the IC DMA transfer complete of a circular stream keeps the channel BUSY") {
	for (unsigned ch = 1; ch <= 4; ++ch) {
		CAPTURE(ch);
		timSupportReset();
		TqRig r;
		tqStartChannel(r, tqIC, ch, true);
		r.complete(static_cast<uint16_t>(ch));
		CHECK(timCounts.icCapture == 1u);
		CHECK(HAL_TIM_GetChannelState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_BUSY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TQ2 the IC DMA half transfer calls the capture half callback with the active channel") {
	for (unsigned ch = 1; ch <= 4; ++ch) {
		CAPTURE(ch);
		timSupportReset();
		TqRig r;
		tqStartChannel(r, tqIC, ch, false);
		r.half(static_cast<uint16_t>(ch));
		CHECK(timCounts.icCaptureHalf == 1u);
		CHECK(timCounts.icCapture == 0u);
		CHECK(timCounts.channelAtLast == tqActive(ch));
		CHECK(HAL_TIM_GetChannelState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_BUSY);
		CHECK(HAL_TIM_GetActiveChannel(&r.h) == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TQ2 the IC DMA error calls the error callback with the active channel and sets the channel READY") {
	for (unsigned ch = 1; ch <= 4; ++ch) {
		CAPTURE(ch);
		timSupportReset();
		TqRig r;
		tqStartChannel(r, tqIC, ch, false);
		r.error(static_cast<uint16_t>(ch));
		CHECK(timCounts.error == 1u);
		CHECK(timCounts.channelAtLast == tqActive(ch));
		CHECK(HAL_TIM_GetChannelState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_READY);
	}
}

namespace {

void tqStartN(TqRig &r, bool pwm, unsigned ch, bool circular) {
	r.link(static_cast<uint16_t>(ch), circular, false);
	if (pwm) {
		REQUIRE(HAL_TIM_PWM_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIMEx_PWMN_Start_DMA(&r.h, tqChannel(ch), r.buf, 4) == HAL_OK);
	} else {
		REQUIRE(HAL_TIM_OC_Init(&r.h) == HAL_OK);
		REQUIRE(HAL_TIMEx_OCN_Start_DMA(&r.h, tqChannel(ch), r.buf, 4) == HAL_OK);
	}
}

} // namespace

TEST_CASE_FIXTURE(TimFixture, "TQ2 the complementary DMA transfer complete calls the pulse finished callback and sets only the complementary state READY") {
	for (int k = 0; k < 2; ++k)
		for (unsigned ch = 1; ch <= 3; ++ch) {
			CAPTURE(k);
			CAPTURE(ch);
			timSupportReset();
			TqRig r;
			tqStartN(r, k == 1, ch, false);
			REQUIRE(HAL_TIMEx_GetChannelNState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_BUSY);
			r.complete(static_cast<uint16_t>(ch));
			CHECK(timCounts.pwmPulse == 1u);
			CHECK(timCounts.orderCount == 1u);
			CHECK(timCounts.channelAtLast == tqActive(ch));
			CHECK(HAL_TIMEx_GetChannelNState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_READY);
			CHECK(HAL_TIM_GetChannelState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_READY); // the main channel was never started
			CHECK(HAL_TIM_GetActiveChannel(&r.h) == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TQ2 characterization: the complementary DMA transfer complete of a circular stream keeps the complementary state BUSY") {
	for (unsigned ch = 1; ch <= 3; ++ch) {
		CAPTURE(ch);
		timSupportReset();
		TqRig r;
		tqStartN(r, true, ch, true);
		r.complete(static_cast<uint16_t>(ch));
		CHECK(timCounts.pwmPulse == 1u);
		CHECK(HAL_TIMEx_GetChannelNState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_BUSY);
	}
}

TEST_CASE_FIXTURE(TimFixture, "TQ2 the complementary DMA half transfer calls the pulse finished half callback with the active channel and leaves the state BUSY") {
	for (int k = 0; k < 2; ++k)
		for (unsigned ch = 1; ch <= 3; ++ch) {
			CAPTURE(k);
			CAPTURE(ch);
			timSupportReset();
			TqRig r;
			tqStartN(r, k == 1, ch, false);
			r.half(static_cast<uint16_t>(ch));
			CHECK(timCounts.pwmPulseHalf == 1u);
			CHECK(timCounts.pwmPulse == 0u);
			CHECK(timCounts.orderCount == 1u);
			CHECK(timCounts.channelAtLast == tqActive(ch));
			CHECK(HAL_TIMEx_GetChannelNState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_BUSY);
			CHECK(HAL_TIM_GetActiveChannel(&r.h) == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
		}
}

TEST_CASE_FIXTURE(TimFixture, "TQ2 the complementary DMA error calls the error callback with the active channel and sets the complementary state READY") {
	for (int k = 0; k < 2; ++k)
		for (unsigned ch = 1; ch <= 3; ++ch) {
			CAPTURE(k);
			CAPTURE(ch);
			timSupportReset();
			TqRig r;
			tqStartN(r, k == 1, ch, false);
			r.error(static_cast<uint16_t>(ch));
			CHECK(timCounts.error == 1u);
			CHECK(timCounts.orderCount == 1u);
			CHECK(timCounts.channelAtLast == tqActive(ch));
			CHECK(HAL_TIMEx_GetChannelNState(&r.h, tqChannel(ch)) == HAL_TIM_CHANNEL_STATE_READY);
			CHECK(HAL_TIM_GetActiveChannel(&r.h) == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
		}
}

TEST_CASE_FIXTURE(TqRig, "TQ2 the trigger DMA transfer complete calls the trigger callback and sets the timer READY in normal mode") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_TRIGGER, false, false);
	REQUIRE(HAL_TIM_DMABurst_MultiWriteStart(&h, TIM_DMABASE_CR1, TIM_DMA_TRIGGER, buf, TIM_DMABURSTLENGTH_1TRANSFER, 4) == HAL_OK);
	h.State = HAL_TIM_STATE_BUSY; // simulates a timer that was started by the application
	complete(TIM_DMA_ID_TRIGGER);
	CHECK(timCounts.trigger == 1u);
	CHECK(timCounts.orderCount == 1u);
	CHECK(timCounts.stateAtLast == HAL_TIM_STATE_READY);
	CHECK(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 characterization: the trigger DMA transfer complete of a circular stream keeps the timer BUSY") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_TRIGGER, true, false);
	REQUIRE(HAL_TIM_DMABurst_MultiWriteStart(&h, TIM_DMABASE_CR1, TIM_DMA_TRIGGER, buf, TIM_DMABURSTLENGTH_1TRANSFER, 4) == HAL_OK);
	h.State = HAL_TIM_STATE_BUSY;
	complete(TIM_DMA_ID_TRIGGER);
	CHECK(timCounts.trigger == 1u);
	CHECK(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_BUSY);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 the trigger DMA half transfer calls the trigger half callback and leaves the timer state") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_TRIGGER, false, false);
	REQUIRE(HAL_TIM_DMABurst_MultiWriteStart(&h, TIM_DMABASE_CR1, TIM_DMA_TRIGGER, buf, TIM_DMABURSTLENGTH_1TRANSFER, 4) == HAL_OK);
	h.State = HAL_TIM_STATE_BUSY;
	half(TIM_DMA_ID_TRIGGER);
	CHECK(timCounts.triggerHalf == 1u);
	CHECK(timCounts.trigger == 0u);
	CHECK(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_BUSY);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 the trigger DMA error calls the error callback and sets the timer READY") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_TRIGGER, false, false);
	REQUIRE(HAL_TIM_DMABurst_MultiWriteStart(&h, TIM_DMABASE_CR1, TIM_DMA_TRIGGER, buf, TIM_DMABURSTLENGTH_1TRANSFER, 4) == HAL_OK);
	h.State = HAL_TIM_STATE_BUSY;
	error(TIM_DMA_ID_TRIGGER);
	CHECK(timCounts.error == 1u);
	CHECK(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 the burst read of the trigger source calls the trigger callbacks too") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_TRIGGER, false, true);
	REQUIRE(HAL_TIM_DMABurst_MultiReadStart(&h, TIM_DMABASE_CR1, TIM_DMA_TRIGGER, buf, TIM_DMABURSTLENGTH_1TRANSFER, 4) == HAL_OK);
	half(TIM_DMA_ID_TRIGGER);
	complete(TIM_DMA_ID_TRIGGER);
	CHECK(timCounts.triggerHalf == 1u);
	CHECK(timCounts.trigger == 1u);
	CHECK(timCounts.order[0] == TIM_CB_TRIGGER_HALF);
	CHECK(timCounts.order[1] == TIM_CB_TRIGGER);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 the burst write of the update source calls the period elapsed callbacks") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_UPDATE, false, false);
	REQUIRE(HAL_TIM_DMABurst_MultiWriteStart(&h, TIM_DMABASE_CR1, TIM_DMA_UPDATE, buf, TIM_DMABURSTLENGTH_1TRANSFER, 4) == HAL_OK);
	half(TIM_DMA_ID_UPDATE);
	complete(TIM_DMA_ID_UPDATE);
	CHECK(timCounts.periodHalf == 1u);
	CHECK(timCounts.period == 1u);
	CHECK(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 the burst write of a capture compare source calls the pulse finished callbacks with the active channel") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_CC3, false, false);
	REQUIRE(HAL_TIM_DMABurst_MultiWriteStart(&h, TIM_DMABASE_CR1, TIM_DMA_CC3, buf, TIM_DMABURSTLENGTH_1TRANSFER, 4) == HAL_OK);
	half(TIM_DMA_ID_CC3);
	CHECK(timCounts.pwmPulseHalf == 1u);
	CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_3);
	complete(TIM_DMA_ID_CC3);
	CHECK(timCounts.pwmPulse == 1u);
	CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_3);
	CHECK(HAL_TIM_GetChannelState(&h, TIM_CHANNEL_3) == HAL_TIM_CHANNEL_STATE_READY);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 the burst read of a capture compare source calls the capture callbacks with the active channel") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_CC2, false, true);
	REQUIRE(HAL_TIM_DMABurst_MultiReadStart(&h, TIM_DMABASE_CR1, TIM_DMA_CC2, buf, TIM_DMABURSTLENGTH_1TRANSFER, 4) == HAL_OK);
	half(TIM_DMA_ID_CC2);
	CHECK(timCounts.icCaptureHalf == 1u);
	CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_2);
	complete(TIM_DMA_ID_CC2);
	CHECK(timCounts.icCapture == 1u);
	CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_2);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 the commutation DMA transfer complete calls the commutation callback and sets the timer READY") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_COMMUTATION, false, false);
	REQUIRE(HAL_TIM_DMABurst_MultiWriteStart(&h, TIM_DMABASE_CR1, TIM_DMA_COM, buf, TIM_DMABURSTLENGTH_1TRANSFER, 4) == HAL_OK);
	h.State = HAL_TIM_STATE_BUSY;
	complete(TIM_DMA_ID_COMMUTATION);
	CHECK(timCounts.commut == 1u);
	CHECK(timCounts.orderCount == 1u);
	CHECK(timCounts.stateAtLast == HAL_TIM_STATE_READY);
	CHECK(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 characterization: the commutation DMA half transfer calls the half callback and already sets the timer READY") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_COMMUTATION, false, false);
	REQUIRE(HAL_TIM_DMABurst_MultiWriteStart(&h, TIM_DMABASE_CR1, TIM_DMA_COM, buf, TIM_DMABURSTLENGTH_1TRANSFER, 4) == HAL_OK);
	h.State = HAL_TIM_STATE_BUSY;
	half(TIM_DMA_ID_COMMUTATION);
	CHECK(timCounts.commutHalf == 1u);
	CHECK(timCounts.commut == 0u);
	CHECK(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 characterization: the commutation DMA transfer complete of a circular stream also sets the timer READY") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_COMMUTATION, true, false);
	REQUIRE(HAL_TIM_DMABurst_MultiWriteStart(&h, TIM_DMABASE_CR1, TIM_DMA_COM, buf, TIM_DMABURSTLENGTH_1TRANSFER, 4) == HAL_OK);
	h.State = HAL_TIM_STATE_BUSY;
	complete(TIM_DMA_ID_COMMUTATION);
	CHECK(timCounts.commut == 1u);
	CHECK(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 the commutation DMA error calls the error callback and sets the timer READY") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_COMMUTATION, false, false);
	REQUIRE(HAL_TIM_DMABurst_MultiWriteStart(&h, TIM_DMABASE_CR1, TIM_DMA_COM, buf, TIM_DMABURSTLENGTH_1TRANSFER, 4) == HAL_OK);
	h.State = HAL_TIM_STATE_BUSY;
	error(TIM_DMA_ID_COMMUTATION);
	CHECK(timCounts.error == 1u);
	CHECK(HAL_TIM_Base_GetState(&h) == HAL_TIM_STATE_READY);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 the commutation callbacks installed by ConfigCommutEvent_DMA run when the stream is started by the application") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	DMA_HandleTypeDef &dm = link(TIM_DMA_ID_COMMUTATION, false, false);
	REQUIRE(HAL_TIMEx_ConfigCommutEvent_DMA(&h, TIM_TS_ITR0, TIM_COMMUTATION_SOFTWARE) == HAL_OK);
	REQUIRE(HAL_DMA_Start_IT(&dm, reinterpret_cast<uint32_t>(buf), reinterpret_cast<uint32_t>(&TIM1->DMAR), 4) == HAL_OK);
	h.State = HAL_TIM_STATE_BUSY;
	half(TIM_DMA_ID_COMMUTATION);
	complete(TIM_DMA_ID_COMMUTATION);
	CHECK(timCounts.commutHalf == 1u);
	CHECK(timCounts.commut == 1u);
	CHECK(timCounts.order[0] == TIM_CB_COMMUT_HALF);
	CHECK(timCounts.order[1] == TIM_CB_COMMUT);
}

TEST_CASE_FIXTURE(TqRig, "TQ2 characterization: callbacks of a stream that is not linked to a channel report the event with no active channel") {
	// The stream is started for CC1, then the handle slot is cleared so that hdma matches no channel (the else branch of every callback).
	for (int ev = 0; ev < 5; ++ev) {
		CAPTURE(ev);
		timSupportReset();
		hostDmaResetState();
		TqRig r;
		if (ev == 0 || ev == 1) { // delay pulse complete / half
			tqStartChannel(r, tqPWM, 1, false);
		} else if (ev == 2 || ev == 3) { // capture complete / half
			tqStartChannel(r, tqIC, 1, false);
		} else { // complementary error
			tqStartN(r, true, 1, false);
		}
		r.h.hdma[TIM_DMA_ID_CC1] = nullptr;
		if (ev == 0 || ev == 2) r.complete(TIM_DMA_ID_CC1);
		if (ev == 1 || ev == 3) r.half(TIM_DMA_ID_CC1);
		if (ev == 4) r.error(TIM_DMA_ID_CC1);
		const unsigned kinds[5] = {TIM_CB_PWM_PULSE, TIM_CB_PWM_PULSE_HALF, TIM_CB_IC_CAPTURE, TIM_CB_IC_CAPTURE_HALF, TIM_CB_ERROR};
		REQUIRE(timCounts.orderCount == 1u);
		CHECK(timCounts.order[0] == kinds[ev]);
		CHECK(timCounts.channelAtLast == HAL_TIM_ACTIVE_CHANNEL_CLEARED);
		// the started channel keeps its BUSY state: the main channel for events 0-3, the complementary channel for event 4
		if (ev == 4) CHECK(HAL_TIMEx_GetChannelNState(&r.h, TIM_CHANNEL_1) == HAL_TIM_CHANNEL_STATE_BUSY);
		else CHECK(HAL_TIM_GetChannelState(&r.h, TIM_CHANNEL_1) == HAL_TIM_CHANNEL_STATE_BUSY);
		r.h.hdma[TIM_DMA_ID_CC1] = &r.d[TIM_DMA_ID_CC1]; // restore for the destructor-free end of the case
	}
}

// ---------------------------------------------------------------------------------------------------------------------------------- TQ3

TEST_CASE_FIXTURE(TqRig, "TQ3 one DMA half transfer and one complete are each seen once and in that order") {
	REQUIRE(HAL_TIM_Base_Init(&h) == HAL_OK);
	link(TIM_DMA_ID_UPDATE, false, false);
	REQUIRE(HAL_TIM_Base_Start_DMA(&h, buf, 4) == HAL_OK);
	half(TIM_DMA_ID_UPDATE);
	irq(TIM_DMA_ID_UPDATE); // the flag was cleared by the first call: nothing again
	complete(TIM_DMA_ID_UPDATE);
	irq(TIM_DMA_ID_UPDATE);
	REQUIRE(timCounts.orderCount == 2u);
	CHECK(timCounts.order[0] == TIM_CB_PERIOD_HALF);
	CHECK(timCounts.order[1] == TIM_CB_PERIOD);
	CHECK(timCounts.periodHalf == 1u);
	CHECK(timCounts.period == 1u);
}

TEST_CASE_FIXTURE(TimFixture, "TQ3 the callbacks of a PWM channel DMA are seen once per event in the order half, complete") {
	TqRig r;
	tqStartChannel(r, tqPWM, 2, false);
	r.half(TIM_DMA_ID_CC2);
	r.complete(TIM_DMA_ID_CC2);
	REQUIRE(timCounts.orderCount == 2u);
	CHECK(timCounts.order[0] == TIM_CB_PWM_PULSE_HALF);
	CHECK(timCounts.order[1] == TIM_CB_PWM_PULSE);
	CHECK(timCounts.pwmPulseHalf == 1u);
	CHECK(timCounts.pwmPulse == 1u);
}
