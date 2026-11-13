/*
 * P12 T6b group AQ: interrupt and DMA modes of the regular group of the real ST HAL ADC driver (stm32f4xx_hal_adc.c: HAL_ADC_Start_IT, Stop_IT,
 * IRQHandler, Start_DMA, Stop_DMA and the static ADC_DMAConvCplt / ADC_DMAHalfConvCplt / ADC_DMAError reached through the DMA handle) together with the
 * real HAL DMA, against the ADC model (fakes/stm32_hal/HalAdc.h) and the DMA model (fakes/stm32_hal/HalDma.h). Requirements AQ1-AQ4 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T6B_REQUIREMENTS.md; compiled into ffb_hal_tests (F407VG) and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 (HAL ADC Generic Driver: the function descriptions, the state names HAL_ADC_STATE_REG_BUSY / REG_EOC / REG_OVR / INJ_BUSY /
 * ERROR_DMA, the error codes, the HAL_ADC_ErrorCallback note on overrun with DMA) and RM0090 Rev 22:
 *  - Table 1 and 13.13.18: ADC1 0x40012000, ADC2 0x40012100, ADC3 0x40012200; ADC_DR at offset 0x4C. 13.13.1 SR (AWD bit 0, EOC 1, JEOC 2, JSTRT 3,
 *    STRT 4, OVR 5; flags cleared by software, EOC also by reading DR), 13.13.2 CR1 (EOCIE bit 5, AWDIE 6, JEOCIE 7, JAUTO 10, OVRIE 26),
 *    13.13.3 CR2 (ADON 0, CONT 1, DMA 8, DDS 9, EOCS 10, SWSTART 30), CCR MULTI bits 4:0.
 *  - Tables 43 / 44: DMA2 ADC1 stream 0 or 4 channel 0, ADC2 stream 2 or 3 channel 1, ADC3 stream 0 or 1 channel 2.
 *  - 13.8.1 / 13.8.2: with DMA = 1 or EOCS = 1 data lost sets OVR, the conversion in progress is aborted; recovery = reinitialise the DMA, clear OVR, trigger;
 *    DMA bit must be written 0 then 1 to start a new transfer.
 *  - 13.3.x: in multi ADC mode the master starts the slaves.
 *  - 10.5.5 DMA_SxCR (EN bit 0, TEIE 2, HTIE 3, TCIE 4, CHSEL bits 27:25), 10.5.6 NDTR counts items, PAR / M0AR addresses.
 * The tests play the DMA engine with hostDmaComplete / hostDmaStepItems / hostDmaRaise and then call HAL_DMA_IRQHandler; the ADC conversions come from
 * hostAdcConvert. Time is the HAL tick (one ms per HAL_GetTick call). Every file-local symbol is prefixed aq or sits in the anonymous namespace.
 *
 * NEEDS REVIEW proposals (numbers assigned by the lead):
 *  NEEDS REVIEW 339/a AQ2: the handler calls the conversion callbacks before it clears the flags (the callback sees EOC / STRT, JEOC / JSTRT, AWD, OVR still set);
 *     RM0090 and UM1725 do not give an order.
 *  NEEDS REVIEW 339/b AQ2: after the last conversion of a software started, non continuous scan sequence with EOCS = 1 the handler leaves the state REG_BUSY and
 *     the EOC interrupt enabled (the HAL has no end of sequence flag); UM1725 silent.
 *  NEEDS REVIEW 339/c AQ3: Start_DMA returns HAL_OK although the DMA stream refuses to start (busy) and leaves the stream and its buffer unchanged; UM1725 only says "HAL status".
 *  NEEDS REVIEW 339/d AQ4: a DMA error leaves exactly the state ERROR_DMA (the other state bits, REG_BUSY, are lost); UM1725 says only "DMA error occurrence".
 *  NEEDS REVIEW 339/e AQ4: a DMA completion with the state ERROR_INTERNAL reports the error callback without changing ErrorCode or State.
 *  NEEDS REVIEW 339/f AQ2: the handler leaves flags it does not handle (AWD, JEOC without their interrupt enables) set.
 *
 * KNOWN ISSUE proposals:
 *  V114 AQ2: after an overrun interrupt the state does not contain HAL_ADC_STATE_REG_OVR (UM1725: "HAL_ADC_STATE_REG_OVR: Overrun occurrence").
 */
#include "AdcTestSupport.h"
#include "HalDma.h"
#include "HalHostSystem.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstring>

namespace {

constexpr uint32_t aqSR_AWD = 1u << 0;
constexpr uint32_t aqSR_EOC = 1u << 1;
constexpr uint32_t aqSR_JEOC = 1u << 2;
constexpr uint32_t aqSR_JSTRT = 1u << 3;
constexpr uint32_t aqSR_STRT = 1u << 4;
constexpr uint32_t aqSR_OVR = 1u << 5;
constexpr uint32_t aqCR1_EOCIE = 1u << 5;
constexpr uint32_t aqCR1_AWDIE = 1u << 6;
constexpr uint32_t aqCR1_JEOCIE = 1u << 7;
constexpr uint32_t aqCR1_JAUTO = 1u << 10;
constexpr uint32_t aqCR1_OVRIE = 1u << 26;
constexpr uint32_t aqCR2_ADON = 1u << 0;
constexpr uint32_t aqCR2_DMA = 1u << 8;
constexpr uint32_t aqCR2_DDS = 1u << 9;
constexpr uint32_t aqCR2_EOCS = 1u << 10;
constexpr uint32_t aqJSQR_JL = 3u << 20;
constexpr uint32_t aqDMA_EN = 1u << 0;
constexpr uint32_t aqDMA_TEIE = 1u << 2;
constexpr uint32_t aqDMA_HTIE = 1u << 3;
constexpr uint32_t aqDMA_TCIE = 1u << 4;

struct AqCfg {
	bool scan = false;
	uint32_t nbr = 1;
	bool cont = false;
	uint32_t eocs = ADC_EOC_SEQ_CONV;
	bool dmaCont = false;
	bool extTrig = false;
	uint32_t extsel = ADC_EXTERNALTRIGCONV_T1_CC1; /* with edge NONE only the selection differs from the default */
	uint32_t align = ADC_DATAALIGN_RIGHT;
};

struct AqMap {
	unsigned stream;
	uint32_t channel;
};
/* RM0090 Table 44 (DMA2). */
AqMap aqMapOf(unsigned adc, bool alt) {
	if (adc == 1) return {alt ? 4u : 0u, DMA_CHANNEL_0};
	if (adc == 2) return {alt ? 3u : 2u, DMA_CHANNEL_1};
	return {alt ? 1u : 0u, DMA_CHANNEL_2};
}
DMA_Stream_TypeDef *aqStreamOf(unsigned s) {
	DMA_Stream_TypeDef *const t[8] = {DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7};
	return t[s];
}
/* RM0090 Table 1 and 13.13.18: ADC_DR address. */
constexpr uint32_t aqDrAddress[4] = {0, 0x4001204Cu, 0x4001214Cu, 0x4001224Cu};

struct AqRig {
	unsigned n;
	ADC_HandleTypeDef h;
	DMA_HandleTypeDef d;
	uint32_t buf[8];
	AqMap map;

	AqRig(unsigned adc, const AqCfg &c = AqCfg(), bool dma = false, bool circular = false, bool alt = false) : n(adc), map(aqMapOf(adc, alt)) {
		std::memset(&h, 0, sizeof h);
		std::memset(&d, 0, sizeof d);
		std::memset(buf, 0, sizeof buf);
		h.Instance = adcInstance(adc);
		h.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
		h.Init.Resolution = ADC_RESOLUTION_12B;
		h.Init.ScanConvMode = c.scan ? ENABLE : DISABLE;
		h.Init.ContinuousConvMode = c.cont ? ENABLE : DISABLE;
		h.Init.DiscontinuousConvMode = DISABLE;
		h.Init.ExternalTrigConvEdge = c.extTrig ? ADC_EXTERNALTRIGCONVEDGE_RISING : ADC_EXTERNALTRIGCONVEDGE_NONE;
		h.Init.ExternalTrigConv = c.extTrig ? ADC_EXTERNALTRIGCONV_T1_CC1 : (c.extsel != ADC_EXTERNALTRIGCONV_T1_CC1 ? c.extsel : ADC_SOFTWARE_START);
		h.Init.DataAlign = c.align;
		h.Init.NbrOfConversion = c.nbr;
		h.Init.DMAContinuousRequests = c.dmaCont ? ENABLE : DISABLE;
		h.Init.EOCSelection = c.eocs;
		REQUIRE(HAL_ADC_Init(&h) == HAL_OK);
		for (uint32_t r = 1; r <= c.nbr; ++r) {
			ADC_ChannelConfTypeDef cc{};
			cc.Channel = r; /* rank r converts channel r, analog value 100 * r */
			cc.Rank = r;
			cc.SamplingTime = ADC_SAMPLETIME_3CYCLES;
			REQUIRE(HAL_ADC_ConfigChannel(&h, &cc) == HAL_OK);
			hostAdcSetAnalog(adc, r, static_cast<uint16_t>(100u * r));
		}
		if (dma) {
			d.Instance = aqStreamOf(map.stream);
			d.Init.Channel = map.channel;
			d.Init.Direction = DMA_PERIPH_TO_MEMORY;
			d.Init.PeriphInc = DMA_PINC_DISABLE;
			d.Init.MemInc = DMA_MINC_ENABLE;
			d.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
			d.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
			d.Init.Mode = circular ? DMA_CIRCULAR : DMA_NORMAL;
			d.Init.Priority = DMA_PRIORITY_HIGH;
			d.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
			d.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
			d.Init.MemBurst = DMA_MBURST_SINGLE;
			d.Init.PeriphBurst = DMA_PBURST_SINGLE;
			REQUIRE(HAL_DMA_Init(&d) == HAL_OK);
			h.DMA_Handle = &d;
			d.Parent = &h;
		}
		adcSupportReset();
	}
	ADC_TypeDef *I() { return h.Instance; }
	uint32_t memAddr() const { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buf)); }
	void pokeSR(uint32_t v) { HalFixture::poke(&h.Instance->SR, v); }
	/* the DMA engine of the tests: stream events then the HAL DMA interrupt handler */
	void dmaComplete() {
		hostDmaComplete(1, map.stream);
		HAL_DMA_IRQHandler(&d);
	}
	void dmaHalf() {
		hostDmaStepItems(1, map.stream, 2);
		HAL_DMA_IRQHandler(&d);
	}
	void dmaError() {
		hostDmaRaise(1, map.stream, HOST_DMA_TEIF);
		HAL_DMA_IRQHandler(&d);
	}
};

AqCfg aqScan(uint32_t nbr, uint32_t eocs) {
	AqCfg c;
	c.scan = true;
	c.nbr = nbr;
	c.eocs = eocs;
	return c;
}
AqCfg aqCont() {
	AqCfg c;
	c.cont = true;
	return c;
}
AqCfg aqSel() {
	AqCfg c;
	c.extsel = ADC_EXTERNALTRIGCONV_T1_CC2; /* EXTSEL != 0, EXTEN = 00: still a software start */
	return c;
}
AqCfg aqExt() {
	AqCfg c;
	c.extTrig = true;
	return c;
}

} // namespace

// ============================================================================ AQ1 Start_IT / Stop_IT

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT enables the end of conversion and the overrun interrupt and no other") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n);
		CHECK(HAL_ADC_Start_IT(&r.h) == HAL_OK);
		CHECK((r.I()->CR1 & aqCR1_EOCIE) == aqCR1_EOCIE);
		CHECK((r.I()->CR1 & aqCR1_OVRIE) == aqCR1_OVRIE);
		CHECK((r.I()->CR1 & (aqCR1_AWDIE | aqCR1_JEOCIE)) == 0u);
		HAL_ADC_Stop_IT(&r.h);
	}
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT enables the ADC and starts a software started conversion sequence") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n);
		REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
		CHECK((r.I()->CR2 & aqCR2_ADON) == aqCR2_ADON);
		CHECK(hostAdcRegularRunning(n) == 1);
		CHECK((r.I()->SR & aqSR_STRT) == aqSR_STRT);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT with an external trigger enables the ADC and does not start a conversion") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, aqExt());
		REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
		CHECK((r.I()->CR2 & aqCR2_ADON) == aqCR2_ADON);
		CHECK(hostAdcRegularRunning(n) == 0);
		CHECK((r.I()->SR & aqSR_STRT) == 0u);
		CHECK(hostAdcExternalTrigger(n, 0) == 1); /* the trigger event starts it */
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT sets the state REG_BUSY and clears READY, REG_EOC and REG_OVR") {
	AqRig r(1);
	r.h.State |= HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR;
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	CHECK(r.h.State == HAL_ADC_STATE_REG_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT clears a stale EOC and OVR flag and leaves the AWD flag") {
	AqRig r(2);
	r.pokeSR(aqSR_EOC | aqSR_OVR | aqSR_AWD);
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	CHECK((r.I()->SR & (aqSR_EOC | aqSR_OVR)) == 0u);
	CHECK((r.I()->SR & aqSR_AWD) == aqSR_AWD);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT clears the whole error code when no injected conversion is running") {
	AqRig r(3);
	r.h.ErrorCode = HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA | HAL_ADC_ERROR_INTERNAL;
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	CHECK(r.h.ErrorCode == HAL_ADC_ERROR_NONE);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT with the injected group busy clears only the overrun and DMA error codes") {
	AqRig r(1);
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	r.h.ErrorCode = HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA | HAL_ADC_ERROR_INTERNAL;
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	CHECK(r.h.ErrorCode == HAL_ADC_ERROR_INTERNAL);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT with automatic injection sets INJ_BUSY and clears INJ_EOC") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_JAUTO;
	r.h.State |= HAL_ADC_STATE_INJ_EOC;
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_EOC) == 0u);
	CHECK((r.h.State & HAL_ADC_STATE_REG_BUSY) == HAL_ADC_STATE_REG_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT with a locked handle returns HAL_BUSY and starts nothing") {
	AqRig r(1);
	r.h.Lock = HAL_LOCKED;
	CHECK(HAL_ADC_Start_IT(&r.h) == HAL_BUSY);
	CHECK((r.I()->CR2 & aqCR2_ADON) == 0u);
	CHECK((r.I()->CR1 & (aqCR1_EOCIE | aqCR1_OVRIE)) == 0u);
	CHECK(r.h.State == HAL_ADC_STATE_READY);
	CHECK(r.h.Lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT leaves the handle unlocked") {
	AqRig r(1);
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	CHECK(r.h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT on an ADC that is already enabled still starts the conversion") {
	AqRig r(2);
	r.I()->CR2 |= aqCR2_ADON;
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	CHECK((r.I()->CR2 & aqCR2_ADON) == aqCR2_ADON);
	CHECK(hostAdcRegularRunning(2) == 1);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT in multi ADC mode software starts only the master ADC1") {
	/* RM0090 13.3.x: in multi ADC mode the slaves are started by the master. MULTI = 00110 regular simultaneous. */
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n);
		ADC->CCR = 0x6u;
		REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
		CHECK(hostAdcRegularRunning(n) == (n == 1 ? 1 : 0));
		CHECK((r.I()->CR2 & aqCR2_ADON) == aqCR2_ADON);
		ADC->CCR = 0;
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT outside multi ADC mode software starts every ADC") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n);
		ADC->CCR = 0;
		REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
		CHECK(hostAdcRegularRunning(n) == 1);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Stop_IT disables the ADC and both interrupts and ends the conversion") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n);
		REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
		REQUIRE(hostAdcRegularRunning(n) == 1);
		CHECK(HAL_ADC_Stop_IT(&r.h) == HAL_OK);
		CHECK((r.I()->CR2 & aqCR2_ADON) == 0u);
		CHECK((r.I()->CR1 & (aqCR1_EOCIE | aqCR1_OVRIE)) == 0u);
		CHECK(hostAdcRegularRunning(n) == 0);
		CHECK(hostAdcConvert(n, 1) == 0u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Stop_IT leaves the injected and watchdog interrupt enables") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_JEOCIE | aqCR1_AWDIE;
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	CHECK(HAL_ADC_Stop_IT(&r.h) == HAL_OK);
	CHECK((r.I()->CR1 & (aqCR1_JEOCIE | aqCR1_AWDIE)) == (aqCR1_JEOCIE | aqCR1_AWDIE));
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Stop_IT sets the state READY and clears REG_BUSY and INJ_BUSY") {
	AqRig r(1);
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	CHECK(HAL_ADC_Stop_IT(&r.h) == HAL_OK);
	CHECK((r.h.State & HAL_ADC_STATE_READY) == HAL_ADC_STATE_READY);
	CHECK((r.h.State & (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY)) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Stop_IT with a locked handle returns HAL_BUSY and changes nothing") {
	AqRig r(1);
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	r.h.Lock = HAL_LOCKED;
	CHECK(HAL_ADC_Stop_IT(&r.h) == HAL_BUSY);
	CHECK((r.I()->CR2 & aqCR2_ADON) == aqCR2_ADON);
	CHECK(r.h.State == HAL_ADC_STATE_REG_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Stop_IT leaves the handle unlocked and is allowed on a stopped ADC") {
	AqRig r(3);
	CHECK(HAL_ADC_Stop_IT(&r.h) == HAL_OK);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	CHECK((r.I()->CR2 & aqCR2_ADON) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 a conversion started by Start_IT raises the interrupt and Stop_IT silences it") {
	AqRig r(1);
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	CHECK(hostAdcIrqPending(1) == 0u);
	CHECK(hostAdcConvert(1, 1) == 1u);
	CHECK(hostAdcIrqPending(1) != 0u);
	HAL_ADC_Stop_IT(&r.h);
	CHECK(hostAdcIrqPending(1) == 0u);
}

// ============================================================================ AQ2 IRQHandler

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of a finished software started single conversion calls ConvCplt once and sets READY and REG_EOC") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n);
		hostAdcSetAnalog(n, 1, 0x123);
		REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
		REQUIRE(hostAdcConvert(n, 1) == 1u);
		HAL_ADC_IRQHandler(&r.h);
		CHECK(adcCounts.conv == 1u);
		CHECK(adcCounts.last == &r.h);
		CHECK(r.h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC));
		CHECK(HAL_ADC_GetValue(&r.h) == 0x123u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler after a software started single conversion disables the EOC interrupt and keeps the overrun interrupt") {
	AqRig r(1);
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	hostAdcConvert(1, 1);
	HAL_ADC_IRQHandler(&r.h);
	CHECK((r.I()->CR1 & aqCR1_EOCIE) == 0u);
	CHECK((r.I()->CR1 & aqCR1_OVRIE) == aqCR1_OVRIE);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler clears the EOC and STRT flags after the regular conversion") {
	AqRig r(1);
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	hostAdcConvert(1, 1);
	REQUIRE((r.I()->SR & (aqSR_EOC | aqSR_STRT)) == (aqSR_EOC | aqSR_STRT));
	HAL_ADC_IRQHandler(&r.h);
	CHECK((r.I()->SR & (aqSR_EOC | aqSR_STRT)) == 0u);
	CHECK(hostAdcIrqPending(1) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 characterization: the conversion callback runs while the EOC and STRT flags are still set") {
	AqRig r(1);
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	hostAdcConvert(1, 1);
	HAL_ADC_IRQHandler(&r.h);
	REQUIRE(adcCounts.conv == 1u);
	CHECK((adcCounts.srAtLast & (aqSR_EOC | aqSR_STRT)) == (aqSR_EOC | aqSR_STRT)); /* NEEDS REVIEW 339/a */
	CHECK(adcCounts.stateAtLast == (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC)); /* the state is updated before the callback */
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler with the EOC flag set but the EOC interrupt disabled calls nothing and keeps the flag") {
	AqRig r(1);
	r.pokeSR(aqSR_EOC);
	r.h.State = HAL_ADC_STATE_READY;
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.conv == 0u);
	CHECK(adcCounts.orderCount == 0u);
	CHECK((r.I()->SR & aqSR_EOC) == aqSR_EOC);
	CHECK(r.h.State == HAL_ADC_STATE_READY);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler with the EOC interrupt enabled but no EOC flag calls nothing") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_EOCIE;
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.orderCount == 0u);
	CHECK(r.h.State == HAL_ADC_STATE_READY);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler without any flag or interrupt changes nothing") {
	AqRig r(1);
	const uint32_t cr1 = r.I()->CR1;
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.orderCount == 0u);
	CHECK(r.I()->SR == 0u);
	CHECK(r.I()->CR1 == cr1);
	CHECK(r.h.State == HAL_ADC_STATE_READY);
	CHECK(r.h.ErrorCode == HAL_ADC_ERROR_NONE);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of a scan sequence with EOC at the end of the sequence calls ConvCplt once for the sequence") {
	AqRig r(1, aqScan(3, ADC_EOC_SEQ_CONV));
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	/* EOCS = 0: EOC only after the third conversion (RM0090 13.13.3 EOCS). */
	REQUIRE(hostAdcConvert(1, 2) == 2u);
	CHECK(hostAdcIrqPending(1) == 0u);
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.conv == 0u);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.conv == 1u);
	CHECK(r.h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC));
	CHECK(HAL_ADC_GetValue(&r.h) == 300u); /* last rank, channel 3 */
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of a scan sequence with EOC after every conversion calls ConvCplt once per conversion with the converted values") {
	AqRig r(2, aqScan(3, ADC_EOC_SINGLE_CONV));
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	for (unsigned i = 1; i <= 3; ++i) {
		INFO("conversion " << i);
		REQUIRE(hostAdcConvert(2, 1) == 1u);
		HAL_ADC_IRQHandler(&r.h);
		CHECK(adcCounts.conv == i);
		CHECK(HAL_ADC_GetValue(&r.h) == 100u * i);
	}
	CHECK((r.h.ErrorCode & HAL_ADC_ERROR_OVR) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of a scan sequence with EOC after every conversion keeps REG_BUSY and the EOC interrupt in the middle of the sequence") {
	AqRig r(1, aqScan(3, ADC_EOC_SINGLE_CONV));
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	hostAdcConvert(1, 1);
	HAL_ADC_IRQHandler(&r.h);
	CHECK((r.h.State & HAL_ADC_STATE_REG_BUSY) == HAL_ADC_STATE_REG_BUSY);
	CHECK((r.h.State & HAL_ADC_STATE_READY) == 0u);
	CHECK((r.h.State & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC);
	CHECK((r.I()->CR1 & aqCR1_EOCIE) == aqCR1_EOCIE);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 characterization: after the last conversion of a software started scan with EOC after every conversion the state stays REG_BUSY") {
	AqRig r(1, aqScan(2, ADC_EOC_SINGLE_CONV));
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	for (unsigned i = 0; i < 2; ++i) {
		REQUIRE(hostAdcConvert(1, 1) == 1u);
		HAL_ADC_IRQHandler(&r.h);
		HAL_ADC_GetValue(&r.h);
	}
	REQUIRE(hostAdcRegularRunning(1) == 0);
	CHECK((r.h.State & HAL_ADC_STATE_REG_BUSY) == HAL_ADC_STATE_REG_BUSY); /* NEEDS REVIEW 339/b */
	CHECK((r.I()->CR1 & aqCR1_EOCIE) == aqCR1_EOCIE);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler in continuous mode keeps REG_BUSY and the EOC interrupt and calls ConvCplt for every conversion") {
	AqRig r(3, aqCont());
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	for (unsigned i = 1; i <= 3; ++i) {
		REQUIRE(hostAdcConvert(3, 1) == 1u);
		HAL_ADC_IRQHandler(&r.h);
		CHECK(adcCounts.conv == i);
		CHECK((r.h.State & HAL_ADC_STATE_REG_BUSY) == HAL_ADC_STATE_REG_BUSY);
		CHECK((r.h.State & HAL_ADC_STATE_READY) == 0u);
		CHECK((r.I()->CR1 & aqCR1_EOCIE) == aqCR1_EOCIE);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler with an external trigger keeps REG_BUSY and the EOC interrupt after a conversion") {
	AqRig r(1, aqExt());
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	REQUIRE(hostAdcExternalTrigger(1, 0) == 1);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.conv == 1u);
	CHECK(r.h.State == (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_REG_EOC));
	CHECK((r.I()->CR1 & aqCR1_EOCIE) == aqCR1_EOCIE);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler after a software started conversion does not set READY while the injected group is busy") {
	AqRig r(1);
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	hostAdcConvert(1, 1);
	HAL_ADC_IRQHandler(&r.h);
	CHECK((r.h.State & HAL_ADC_STATE_READY) == 0u);
	CHECK((r.h.State & HAL_ADC_STATE_REG_BUSY) == 0u);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler does not set REG_EOC in the state ERROR_INTERNAL but still calls ConvCplt") {
	AqRig r(1, aqCont());
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	r.h.State |= HAL_ADC_STATE_ERROR_INTERNAL;
	hostAdcConvert(1, 1);
	HAL_ADC_IRQHandler(&r.h);
	CHECK((r.h.State & HAL_ADC_STATE_REG_EOC) == 0u);
	CHECK(adcCounts.conv == 1u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 characterization: the EOC branch clears only EOC and STRT and leaves the other set flags") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_EOCIE;
	r.pokeSR(0x3Fu);
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.conv == 1u);
	CHECK(adcCounts.window + adcCounts.injected + adcCounts.error == 0u);
	CHECK(r.I()->SR == (0x3Fu & ~(aqSR_EOC | aqSR_STRT))); /* NEEDS REVIEW 339/f */
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of the JEOC flag with its interrupt calls the injected callback and sets INJ_EOC") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n);
		r.I()->CR1 |= aqCR1_JEOCIE;
		r.pokeSR(aqSR_JEOC | aqSR_JSTRT);
		HAL_ADC_IRQHandler(&r.h);
		CHECK(adcCounts.injected == 1u);
		CHECK(adcCounts.conv == 0u);
		CHECK((r.h.State & HAL_ADC_STATE_INJ_EOC) == HAL_ADC_STATE_INJ_EOC);
		CHECK(adcCounts.last == &r.h);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of a software started single injected conversion disables JEOCIE and sets READY") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_JEOCIE;
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	r.pokeSR(aqSR_JEOC | aqSR_JSTRT);
	HAL_ADC_IRQHandler(&r.h);
	CHECK((r.I()->CR1 & aqCR1_JEOCIE) == 0u);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_BUSY) == 0u);
	CHECK(r.h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_INJ_EOC));
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of the JEOC flag does not set READY while the regular group is busy") {
	AqRig r(1);
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	r.I()->CR1 |= aqCR1_JEOCIE;
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	r.pokeSR(aqSR_JEOC);
	HAL_ADC_IRQHandler(&r.h);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_BUSY) == 0u);
	CHECK((r.h.State & HAL_ADC_STATE_READY) == 0u);
	CHECK((r.h.State & HAL_ADC_STATE_REG_BUSY) == HAL_ADC_STATE_REG_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of the JEOC flag with a multi channel injected group and EOC after every conversion keeps JEOCIE") {
	AqRig r(1, aqScan(1, ADC_EOC_SINGLE_CONV));
	r.I()->CR1 |= aqCR1_JEOCIE;
	r.I()->JSQR |= (1u << 20); /* JL = 1: two injected conversions */
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	r.pokeSR(aqSR_JEOC);
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.injected == 1u);
	CHECK((r.I()->CR1 & aqCR1_JEOCIE) == aqCR1_JEOCIE);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of the JEOC flag with a multi channel injected group and EOC at the end of the sequence disables JEOCIE") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_JEOCIE;
	r.I()->JSQR |= (1u << 20);
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	r.pokeSR(aqSR_JEOC);
	HAL_ADC_IRQHandler(&r.h);
	CHECK((r.I()->CR1 & aqCR1_JEOCIE) == 0u);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_BUSY) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of the JEOC flag with automatic injection keeps JEOCIE and INJ_BUSY") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_JEOCIE | aqCR1_JAUTO;
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	r.pokeSR(aqSR_JEOC);
	HAL_ADC_IRQHandler(&r.h);
	CHECK((r.I()->CR1 & aqCR1_JEOCIE) == aqCR1_JEOCIE);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of the JEOC flag with continuous regular conversion keeps JEOCIE") {
	AqRig r(1, aqCont());
	r.I()->CR1 |= aqCR1_JEOCIE;
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	r.pokeSR(aqSR_JEOC);
	HAL_ADC_IRQHandler(&r.h);
	CHECK((r.I()->CR1 & aqCR1_JEOCIE) == aqCR1_JEOCIE);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of the JEOC flag with an external regular trigger keeps JEOCIE") {
	AqRig r(1, aqExt());
	r.I()->CR1 |= aqCR1_JEOCIE;
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	r.pokeSR(aqSR_JEOC);
	HAL_ADC_IRQHandler(&r.h);
	CHECK((r.I()->CR1 & aqCR1_JEOCIE) == aqCR1_JEOCIE);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of the JEOC flag with an external injected trigger keeps JEOCIE") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_JEOCIE;
	r.I()->CR2 |= (1u << 20); /* JEXTEN rising */
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	r.pokeSR(aqSR_JEOC);
	HAL_ADC_IRQHandler(&r.h);
	CHECK((r.I()->CR1 & aqCR1_JEOCIE) == aqCR1_JEOCIE);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler clears the JEOC and JSTRT flags after the injected callback") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_JEOCIE;
	r.pokeSR(aqSR_JEOC | aqSR_JSTRT);
	HAL_ADC_IRQHandler(&r.h);
	CHECK((r.I()->SR & (aqSR_JEOC | aqSR_JSTRT)) == 0u);
	CHECK((adcCounts.srAtLast & (aqSR_JEOC | aqSR_JSTRT)) == (aqSR_JEOC | aqSR_JSTRT)); /* NEEDS REVIEW 339/a */
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler with the JEOC flag but no JEOCIE or with JEOCIE but no flag calls nothing") {
	AqRig r(1);
	r.pokeSR(aqSR_JEOC);
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.injected == 0u);
	CHECK((r.I()->SR & aqSR_JEOC) == aqSR_JEOC);
	r.pokeSR(0);
	r.I()->CR1 |= aqCR1_JEOCIE;
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.injected == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of the JEOC flag does not set INJ_EOC in the state ERROR_INTERNAL but still calls back") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_JEOCIE;
	r.h.State |= HAL_ADC_STATE_ERROR_INTERNAL;
	r.pokeSR(aqSR_JEOC);
	HAL_ADC_IRQHandler(&r.h);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_EOC) == 0u);
	CHECK(adcCounts.injected == 1u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 characterization: the JEOC branch clears only JEOC and JSTRT") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_JEOCIE;
	r.pokeSR(0x3Fu);
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.injected == 1u);
	CHECK(adcCounts.conv + adcCounts.window + adcCounts.error == 0u);
	CHECK(r.I()->SR == (0x3Fu & ~(aqSR_JEOC | aqSR_JSTRT))); /* NEEDS REVIEW 339/f */
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of the AWD flag with AWDIE calls the window callback and sets the state AWD1") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n);
		r.I()->CR1 |= aqCR1_AWDIE;
		r.pokeSR(aqSR_AWD);
		HAL_ADC_IRQHandler(&r.h);
		CHECK(adcCounts.window == 1u);
		CHECK((r.h.State & HAL_ADC_STATE_AWD1) == HAL_ADC_STATE_AWD1);
		CHECK(adcCounts.last == &r.h);
		CHECK((r.I()->SR & aqSR_AWD) == 0u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of the AWD flag calls the window callback before it clears the flag") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_AWDIE;
	r.pokeSR(aqSR_AWD);
	HAL_ADC_IRQHandler(&r.h);
	REQUIRE(adcCounts.window == 1u);
	CHECK((adcCounts.srAtLast & aqSR_AWD) == aqSR_AWD); /* NEEDS REVIEW 339/a */
	CHECK((adcCounts.stateAtLast & HAL_ADC_STATE_AWD1) == HAL_ADC_STATE_AWD1);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler with the AWD flag but no AWDIE or AWDIE but no flag calls nothing") {
	AqRig r(1);
	r.pokeSR(aqSR_AWD);
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.window == 0u);
	CHECK((r.I()->SR & aqSR_AWD) == aqSR_AWD);
	CHECK((r.h.State & HAL_ADC_STATE_AWD1) == 0u);
	r.pokeSR(0);
	r.I()->CR1 |= aqCR1_AWDIE;
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.window == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 characterization: the AWD branch clears only the AWD flag") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_AWDIE;
	r.pokeSR(0x3Fu);
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.window == 1u);
	CHECK(adcCounts.conv + adcCounts.injected + adcCounts.error == 0u);
	CHECK(r.I()->SR == (0x3Fu & ~aqSR_AWD)); /* NEEDS REVIEW 339/f */
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of the OVR flag with OVRIE sets the error code OVR and calls the error callback") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n);
		r.I()->CR1 |= aqCR1_OVRIE;
		r.pokeSR(aqSR_OVR);
		HAL_ADC_IRQHandler(&r.h);
		CHECK(adcCounts.error == 1u);
		CHECK((r.h.ErrorCode & HAL_ADC_ERROR_OVR) == HAL_ADC_ERROR_OVR);
		CHECK((adcCounts.errorAtLast & HAL_ADC_ERROR_OVR) == HAL_ADC_ERROR_OVR); /* the code is set before the callback */
		CHECK((r.I()->SR & aqSR_OVR) == 0u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of the OVR flag keeps the other error codes") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_OVRIE;
	r.h.ErrorCode = HAL_ADC_ERROR_DMA;
	r.pokeSR(aqSR_OVR);
	HAL_ADC_IRQHandler(&r.h);
	CHECK(r.h.ErrorCode == (HAL_ADC_ERROR_DMA | HAL_ADC_ERROR_OVR));
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler with the OVR flag but no OVRIE or OVRIE but no flag calls nothing") {
	AqRig r(1);
	r.pokeSR(aqSR_OVR);
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.error == 0u);
	CHECK(r.h.ErrorCode == HAL_ADC_ERROR_NONE);
	CHECK((r.I()->SR & aqSR_OVR) == aqSR_OVR);
	r.pokeSR(0);
	r.I()->CR1 |= aqCR1_OVRIE;
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.error == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of the OVR flag clears the flag before the error callback runs") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_OVRIE;
	r.pokeSR(aqSR_OVR);
	HAL_ADC_IRQHandler(&r.h);
	REQUIRE(adcCounts.error == 1u);
	CHECK((adcCounts.srAtLast & aqSR_OVR) == 0u); /* NEEDS REVIEW 339/a: unlike the other branches the OVR flag is cleared before the callback (and again after it) */
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 characterization: the OVR branch clears only the OVR flag") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_OVRIE;
	r.pokeSR(0x3Fu);
	HAL_ADC_IRQHandler(&r.h);
	CHECK(adcCounts.error == 1u);
	CHECK(adcCounts.conv + adcCounts.injected + adcCounts.window == 0u);
	CHECK(r.I()->SR == (0x3Fu & ~aqSR_OVR)); /* NEEDS REVIEW 339/f */
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler with all four flags and interrupts calls the callbacks in the order regular, injected, watchdog, overrun") {
	AqRig r(1);
	r.I()->CR1 |= aqCR1_EOCIE | aqCR1_JEOCIE | aqCR1_AWDIE | aqCR1_OVRIE;
	r.pokeSR(0x3Fu);
	HAL_ADC_IRQHandler(&r.h);
	REQUIRE(adcCounts.orderCount == 4u);
	CHECK(adcCounts.order[0] == static_cast<unsigned>(ADC_CB_CONV));
	CHECK(adcCounts.order[1] == static_cast<unsigned>(ADC_CB_INJECTED));
	CHECK(adcCounts.order[2] == static_cast<unsigned>(ADC_CB_WINDOW));
	CHECK(adcCounts.order[3] == static_cast<unsigned>(ADC_CB_ERROR));
	CHECK(r.I()->SR == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 an overrun of a scan with EOC after every conversion reports ConvCplt for the pending data then the error") {
	AqRig r(1, aqScan(2, ADC_EOC_SINGLE_CONV));
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u); /* EOC set, DR not read */
	REQUIRE(hostAdcConvert(1, 1) == 1u); /* data lost: OVR (RM0090 13.8.2) and the sequence is aborted */
	CHECK((r.I()->SR & aqSR_OVR) == aqSR_OVR);
	CHECK(hostAdcRegularRunning(1) == 0);
	CHECK(hostAdcIrqPending(1) != 0u);
	HAL_ADC_IRQHandler(&r.h);
	REQUIRE(adcCounts.orderCount == 2u);
	CHECK(adcCounts.order[0] == static_cast<unsigned>(ADC_CB_CONV));
	CHECK(adcCounts.order[1] == static_cast<unsigned>(ADC_CB_ERROR));
	CHECK((r.h.ErrorCode & HAL_ADC_ERROR_OVR) == HAL_ADC_ERROR_OVR);
	CHECK((r.I()->SR & aqSR_OVR) == 0u);
	CHECK(hostAdcIrqPending(1) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 an overrun interrupt sets the state REG_OVR" * KNOWN_ISSUE("V114 (arguable: UM1725 only says 'Overrun occurrence', but HAL_ADC_PollForEvent sets REG_OVR for the same event, so the interrupt path is inconsistent): UM1725 defines HAL_ADC_STATE_REG_OVR as 'Overrun occurrence' but HAL_ADC_IRQHandler (stm32f4xx_hal_adc.c OVR branch, lines 1335-1350) only sets ErrorCode, never the state bit (the bit is only cleared in Start_IT / Start_DMA)")) {
	AqRig r(1, aqScan(2, ADC_EOC_SINGLE_CONV));
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	hostAdcConvert(1, 2);
	REQUIRE((r.I()->SR & aqSR_OVR) == aqSR_OVR);
	HAL_ADC_IRQHandler(&r.h);
	REQUIRE((r.h.ErrorCode & HAL_ADC_ERROR_OVR) == HAL_ADC_ERROR_OVR);
	CHECK((r.h.State & HAL_ADC_STATE_REG_OVR) == HAL_ADC_STATE_REG_OVR);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ2 Start_IT after an overrun clears the OVR flag and the error code and the conversion resumes") {
	/* RM0090 13.8.2: recovery = clear OVR, trigger the ADC. */
	AqRig r(1, aqScan(2, ADC_EOC_SINGLE_CONV));
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	hostAdcConvert(1, 2);
	HAL_ADC_IRQHandler(&r.h);
	REQUIRE((r.h.ErrorCode & HAL_ADC_ERROR_OVR) == HAL_ADC_ERROR_OVR);
	REQUIRE(HAL_ADC_Stop_IT(&r.h) == HAL_OK);
	REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
	CHECK(r.h.ErrorCode == HAL_ADC_ERROR_NONE);
	CHECK((r.I()->SR & aqSR_OVR) == 0u);
	CHECK(hostAdcRegularRunning(1) == 1);
	CHECK(hostAdcConvert(1, 1) == 1u);
}

// ============================================================================ AQ3 Start_DMA / Stop_DMA

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA programs the DMA stream of the ADC with the data register, the buffer and the length") {
	for (unsigned n = 1; n <= 3; ++n) {
		for (int alt = 0; alt < 2; ++alt) {
			INFO("ADC" << n << (alt ? " alternate stream" : " first stream"));
			AqRig r(n, AqCfg(), true, false, alt != 0);
			REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 5) == HAL_OK);
			DMA_Stream_TypeDef *s = aqStreamOf(r.map.stream);
			CHECK(s->NDTR == 5u);
			CHECK(s->PAR == aqDrAddress[n]);
			CHECK(s->M0AR == r.memAddr());
			CHECK((s->CR & aqDMA_EN) == aqDMA_EN);
			CHECK(((s->CR >> 25) & 7u) == (n - 1u)); /* CHSEL: channel 0 ADC1, 1 ADC2, 2 ADC3 (Table 44) */
			HAL_ADC_Stop_DMA(&r.h);
		}
	}
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA enables the DMA stream interrupts for completion, half completion and error") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	const uint32_t cr = aqStreamOf(r.map.stream)->CR;
	CHECK((cr & aqDMA_TCIE) == aqDMA_TCIE);
	CHECK((cr & aqDMA_HTIE) == aqDMA_HTIE);
	CHECK((cr & aqDMA_TEIE) == aqDMA_TEIE);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA sets the DMA bit and enables the ADC and the overrun interrupt but not the EOC interrupt") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, AqCfg(), true);
		REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
		CHECK((r.I()->CR2 & aqCR2_DMA) == aqCR2_DMA);
		CHECK((r.I()->CR2 & aqCR2_ADON) == aqCR2_ADON);
		CHECK((r.I()->CR1 & aqCR1_OVRIE) == aqCR1_OVRIE);
		CHECK((r.I()->CR1 & aqCR1_EOCIE) == 0u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA keeps the DDS bit of the Init setting") {
	{
		AqRig r(1, AqCfg(), true);
		REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
		CHECK((r.I()->CR2 & aqCR2_DDS) == 0u);
		HAL_ADC_Stop_DMA(&r.h);
	}
	AqCfg c;
	c.dmaCont = true;
	AqRig r(1, c, true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	CHECK((r.I()->CR2 & aqCR2_DDS) == aqCR2_DDS);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA writes the DMA bit 0 and then 1 when it was set before (RM0090 13.8.1)") {
	AqRig r(1, AqCfg(), true);
	r.I()->CR2 |= aqCR2_ADON | aqCR2_DMA;
	hostRegsLogClear();
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	bool sawZero = false, oneAfterZero = false;
	const uintptr_t cr2 = reinterpret_cast<uintptr_t>(&r.I()->CR2);
	for (size_t i = 0; i < hostRegsLogCount(); ++i) {
		const HostRegAccess *e = hostRegsLogAt(i);
		if (!e->write || e->addr != cr2) continue;
		if (!(e->written & aqCR2_DMA)) sawZero = true;
		else if (sawZero) oneAfterZero = true;
	}
	CHECK(sawZero);
	CHECK(oneAfterZero);
	CHECK((r.I()->CR2 & aqCR2_DMA) == aqCR2_DMA);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA starts a software conversion and counts a DMA request for each regular conversion") {
	AqRig r(2, aqScan(3, ADC_EOC_SEQ_CONV), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 3) == HAL_OK);
	CHECK(hostAdcRegularRunning(2) == 1);
	CHECK(hostAdcDmaRequests(2) == 0u);
	/* the engine of the test takes the data after every conversion (an unread DR would overrun) */
	for (unsigned i = 0; i < 3; ++i) {
		REQUIRE(hostAdcConvert(2, 1) == 1u);
		r.buf[i] = r.I()->DR;
	}
	CHECK(hostAdcDmaRequests(2) == 3u);
	CHECK(r.buf[0] == 100u);
	CHECK(r.buf[1] == 200u);
	CHECK(r.buf[2] == 300u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA with an external trigger does not software start the conversion") {
	AqRig r(1, aqExt(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	CHECK((r.I()->CR2 & aqCR2_ADON) == aqCR2_ADON);
	CHECK(hostAdcRegularRunning(1) == 0);
	CHECK(hostAdcExternalTrigger(1, 0) == 1);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA in multi ADC mode software starts only the master ADC1") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, AqCfg(), true);
		ADC->CCR = 0x6u;
		REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
		CHECK(hostAdcRegularRunning(n) == (n == 1 ? 1 : 0));
		ADC->CCR = 0;
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA outside multi ADC mode software starts every ADC") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, AqCfg(), true);
		ADC->CCR = 0;
		REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
		CHECK(hostAdcRegularRunning(n) == 1);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA sets the state REG_BUSY and clears the error code and the stale EOC and OVR flags") {
	AqRig r(1, AqCfg(), true);
	r.h.State |= HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR;
	r.h.ErrorCode = HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA;
	r.pokeSR(aqSR_EOC | aqSR_OVR | aqSR_AWD);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	CHECK(r.h.State == HAL_ADC_STATE_REG_BUSY);
	CHECK(r.h.ErrorCode == HAL_ADC_ERROR_NONE);
	CHECK((r.I()->SR & (aqSR_EOC | aqSR_OVR)) == 0u);
	CHECK((r.I()->SR & aqSR_AWD) == aqSR_AWD);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA with the injected group busy clears only the overrun and DMA error codes") {
	AqRig r(3, AqCfg(), true);
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	r.h.ErrorCode = HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA | HAL_ADC_ERROR_INTERNAL;
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	CHECK(r.h.ErrorCode == HAL_ADC_ERROR_INTERNAL);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA with automatic injection sets INJ_BUSY and clears INJ_EOC") {
	AqRig r(1, AqCfg(), true);
	r.I()->CR1 |= aqCR1_JAUTO;
	r.h.State |= HAL_ADC_STATE_INJ_EOC;
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_EOC) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA with a locked handle returns HAL_BUSY and starts nothing") {
	AqRig r(1, AqCfg(), true);
	r.h.Lock = HAL_LOCKED;
	CHECK(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_BUSY);
	CHECK((r.I()->CR2 & (aqCR2_ADON | aqCR2_DMA)) == 0u);
	CHECK((aqStreamOf(r.map.stream)->CR & aqDMA_EN) == 0u);
	CHECK(r.h.Lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA leaves the handle unlocked") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	CHECK(r.h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 characterization: Start_DMA returns HAL_OK although the DMA stream is busy and the stream keeps the first buffer") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	uint32_t other[4] = {0, 0, 0, 0};
	const HAL_StatusTypeDef st = HAL_ADC_Start_DMA(&r.h, other, 2);
	CHECK(st == HAL_OK); /* NEEDS REVIEW 339/c */
	DMA_Stream_TypeDef *s = aqStreamOf(r.map.stream);
	CHECK(s->M0AR == r.memAddr());
	CHECK(s->NDTR == 4u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Stop_DMA disables the ADC, the DMA bit, the overrun interrupt and the DMA stream") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, AqCfg(), true);
		REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
		CHECK(HAL_ADC_Stop_DMA(&r.h) == HAL_OK);
		CHECK((r.I()->CR2 & (aqCR2_ADON | aqCR2_DMA)) == 0u);
		CHECK((r.I()->CR1 & aqCR1_OVRIE) == 0u);
		CHECK((aqStreamOf(r.map.stream)->CR & aqDMA_EN) == 0u);
		CHECK(r.d.State == HAL_DMA_STATE_READY);
		CHECK(hostAdcRegularRunning(n) == 0);
	}
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Stop_DMA sets the state READY and clears REG_BUSY and INJ_BUSY") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	CHECK(HAL_ADC_Stop_DMA(&r.h) == HAL_OK);
	CHECK((r.h.State & HAL_ADC_STATE_READY) == HAL_ADC_STATE_READY);
	CHECK((r.h.State & (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY)) == 0u);
	CHECK((r.h.State & HAL_ADC_STATE_ERROR_DMA) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Stop_DMA after a finished normal mode transfer returns HAL_OK and leaves the DMA ready") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 1) == HAL_OK);
	hostAdcConvert(1, 1);
	r.dmaComplete();
	REQUIRE(r.d.State == HAL_DMA_STATE_READY);
	CHECK(HAL_ADC_Stop_DMA(&r.h) == HAL_OK);
	CHECK((r.I()->CR2 & aqCR2_ADON) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Stop_DMA of a circular DMA stream aborts the stream") {
	AqRig r(1, aqCont(), true, true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	r.dmaComplete(); /* circular: the stream keeps running and stays BUSY */
	REQUIRE(r.d.State == HAL_DMA_STATE_BUSY);
	CHECK(HAL_ADC_Stop_DMA(&r.h) == HAL_OK);
	CHECK((aqStreamOf(r.map.stream)->CR & aqDMA_EN) == 0u);
	CHECK(r.d.State == HAL_DMA_STATE_READY);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Stop_DMA returns a non OK status and sets ERROR_DMA when the DMA stream cannot be stopped") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	hostDmaStickEnabled(1, r.map.stream, 1);
	CHECK(HAL_ADC_Stop_DMA(&r.h) != HAL_OK); /* the abort timeout is passed on (HAL_TIMEOUT) */
	CHECK((r.h.State & HAL_ADC_STATE_ERROR_DMA) == HAL_ADC_STATE_ERROR_DMA);
	CHECK((r.I()->CR2 & aqCR2_ADON) == 0u); /* the ADC is stopped anyway */
	CHECK((r.I()->CR1 & aqCR1_OVRIE) == 0u);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	hostDmaStickEnabled(1, r.map.stream, 0);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Stop_DMA with a locked handle returns HAL_BUSY and changes nothing") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	r.h.Lock = HAL_LOCKED;
	CHECK(HAL_ADC_Stop_DMA(&r.h) == HAL_BUSY);
	CHECK((r.I()->CR2 & (aqCR2_ADON | aqCR2_DMA)) == (aqCR2_ADON | aqCR2_DMA));
	CHECK((aqStreamOf(r.map.stream)->CR & aqDMA_EN) == aqDMA_EN);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Stop_DMA leaves the handle unlocked and is allowed on an ADC that was never started") {
	AqRig r(2, AqCfg(), true);
	CHECK(HAL_ADC_Stop_DMA(&r.h) == HAL_OK);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	CHECK(r.h.State == HAL_ADC_STATE_READY);
}

// ============================================================================ AQ4 DMA callbacks

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA completion of a software started single conversion calls ConvCplt and sets READY and REG_EOC") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, AqCfg(), true);
		REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 1) == HAL_OK);
		REQUIRE(hostAdcConvert(n, 1) == 1u);
		r.buf[0] = r.I()->DR;
		r.dmaComplete();
		CHECK(adcCounts.conv == 1u);
		CHECK(adcCounts.last == &r.h);
		CHECK(r.h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC));
		CHECK(r.buf[0] == 100u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA completion of a software started scan with EOC at the end of the sequence sets READY") {
	AqRig r(1, aqScan(3, ADC_EOC_SEQ_CONV), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 3) == HAL_OK);
	r.dmaComplete();
	CHECK(adcCounts.conv == 1u);
	CHECK((r.h.State & HAL_ADC_STATE_READY) == HAL_ADC_STATE_READY);
	CHECK((r.h.State & HAL_ADC_STATE_REG_BUSY) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA completion of a scan with EOC after every conversion keeps REG_BUSY") {
	AqRig r(1, aqScan(3, ADC_EOC_SINGLE_CONV), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 3) == HAL_OK);
	r.dmaComplete();
	CHECK(adcCounts.conv == 1u);
	CHECK((r.h.State & HAL_ADC_STATE_REG_BUSY) == HAL_ADC_STATE_REG_BUSY);
	CHECK((r.h.State & HAL_ADC_STATE_READY) == 0u);
	CHECK((r.h.State & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA completion in continuous mode keeps REG_BUSY") {
	AqRig r(1, aqCont(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	r.dmaComplete();
	CHECK(adcCounts.conv == 1u);
	CHECK(r.h.State == (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_REG_EOC));
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA completion with an external trigger keeps REG_BUSY") {
	AqRig r(1, aqExt(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	r.dmaComplete();
	CHECK(adcCounts.conv == 1u);
	CHECK(r.h.State == (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_REG_EOC));
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA completion does not set READY while the injected group is busy") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 1) == HAL_OK);
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	r.dmaComplete();
	CHECK(adcCounts.conv == 1u);
	CHECK((r.h.State & HAL_ADC_STATE_READY) == 0u);
	CHECK((r.h.State & HAL_ADC_STATE_REG_BUSY) == 0u);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA completion in circular mode calls ConvCplt for every wrap of the buffer") {
	AqRig r(1, aqCont(), true, true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	for (unsigned i = 1; i <= 3; ++i) {
		r.dmaComplete();
		CHECK(adcCounts.conv == i);
	}
	CHECK((aqStreamOf(r.map.stream)->CR & aqDMA_EN) == aqDMA_EN);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA completion does not disable the EOC interrupt of a DMA transfer (it was never enabled)") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 1) == HAL_OK);
	r.dmaComplete();
	CHECK((r.I()->CR1 & aqCR1_EOCIE) == 0u);
	CHECK((r.I()->CR1 & aqCR1_OVRIE) == aqCR1_OVRIE); /* only Stop_DMA disables it */
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA half completion calls ConvHalfCplt once and changes no state") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, aqCont(), true);
		REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
		const uint32_t st = r.h.State;
		r.dmaHalf();
		CHECK(adcCounts.convHalf == 1u);
		CHECK(adcCounts.conv == 0u);
		CHECK(adcCounts.last == &r.h);
		CHECK(r.h.State == st);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA half completion then completion call the callbacks in that order") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	r.dmaHalf();
	hostDmaStepItems(1, r.map.stream, 2);
	HAL_DMA_IRQHandler(&r.d);
	REQUIRE(adcCounts.orderCount == 2u);
	CHECK(adcCounts.order[0] == static_cast<unsigned>(ADC_CB_CONV_HALF));
	CHECK(adcCounts.order[1] == static_cast<unsigned>(ADC_CB_CONV));
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA error sets the state ERROR_DMA, the error code DMA and calls the error callback") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, AqCfg(), true);
		REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
		r.dmaError();
		CHECK(adcCounts.error == 1u);
		CHECK(adcCounts.conv == 0u);
		CHECK((r.h.State & HAL_ADC_STATE_ERROR_DMA) == HAL_ADC_STATE_ERROR_DMA);
		CHECK((r.h.ErrorCode & HAL_ADC_ERROR_DMA) == HAL_ADC_ERROR_DMA);
		CHECK((adcCounts.errorAtLast & HAL_ADC_ERROR_DMA) == HAL_ADC_ERROR_DMA); /* set before the callback */
		CHECK((adcCounts.stateAtLast & HAL_ADC_STATE_ERROR_DMA) == HAL_ADC_STATE_ERROR_DMA);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA error keeps the other error codes") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	r.h.ErrorCode |= HAL_ADC_ERROR_OVR;
	r.dmaError();
	CHECK(r.h.ErrorCode == (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 characterization: a DMA error leaves exactly the state ERROR_DMA") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	REQUIRE(r.h.State == HAL_ADC_STATE_REG_BUSY);
	r.dmaError();
	CHECK(r.h.State == HAL_ADC_STATE_ERROR_DMA); /* NEEDS REVIEW 339/d */
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA completion in the state ERROR_DMA calls the error callback instead of ConvCplt") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 1) == HAL_OK);
	r.h.State |= HAL_ADC_STATE_ERROR_DMA;
	r.dmaComplete();
	CHECK(adcCounts.conv == 0u);
	CHECK(adcCounts.error == 1u);
	CHECK((r.h.ErrorCode & HAL_ADC_ERROR_DMA) == HAL_ADC_ERROR_DMA);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 characterization: DMA completion in the state ERROR_INTERNAL reports the error callback and changes neither state nor error code") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 1) == HAL_OK);
	r.h.State |= HAL_ADC_STATE_ERROR_INTERNAL;
	r.h.ErrorCode = HAL_ADC_ERROR_INTERNAL;
	const uint32_t st = r.h.State;
	r.dmaComplete();
	CHECK(adcCounts.conv == 0u);
	CHECK(adcCounts.error == 1u); /* NEEDS REVIEW 339/e */
	CHECK(r.h.State == st);
	CHECK(r.h.ErrorCode == HAL_ADC_ERROR_INTERNAL);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 an overrun with DMA sets the error code OVR, calls the error callback and clears OVR") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, aqScan(2, ADC_EOC_SINGLE_CONV), true);
		REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
		REQUIRE(hostAdcConvert(n, 1) == 1u);
		REQUIRE(hostAdcConvert(n, 1) == 1u); /* the first data was not taken: RM0090 13.8.1 data lost */
		CHECK((r.I()->SR & aqSR_OVR) == aqSR_OVR);
		CHECK(hostAdcRegularRunning(n) == 0);
		CHECK(hostAdcIrqPending(n) != 0u);
		HAL_ADC_IRQHandler(&r.h);
		CHECK(adcCounts.error == 1u);
		CHECK(adcCounts.conv == 0u); /* the EOC interrupt is not enabled in DMA mode */
		CHECK((r.h.ErrorCode & HAL_ADC_ERROR_OVR) == HAL_ADC_ERROR_OVR);
		CHECK((r.I()->SR & aqSR_OVR) == 0u);
		CHECK(hostAdcIrqPending(n) == 0u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 after an overrun with DMA, Stop_DMA and Start_DMA clear the error and restart the conversions (UM1725 ErrorCallback note)") {
	AqRig r(1, aqScan(2, ADC_EOC_SINGLE_CONV), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	hostAdcConvert(1, 2);
	HAL_ADC_IRQHandler(&r.h);
	REQUIRE((r.h.ErrorCode & HAL_ADC_ERROR_OVR) == HAL_ADC_ERROR_OVR);
	REQUIRE(HAL_ADC_Stop_DMA(&r.h) == HAL_OK);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	CHECK(r.h.ErrorCode == HAL_ADC_ERROR_NONE);
	CHECK((r.I()->SR & aqSR_OVR) == 0u);
	CHECK(hostAdcRegularRunning(1) == 1);
	CHECK(aqStreamOf(r.map.stream)->M0AR == r.memAddr());
	CHECK(hostAdcConvert(1, 1) == 1u);
	CHECK((r.I()->SR & aqSR_OVR) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 a scan sequence by DMA delivers every converted value to the buffer and completes once") {
	AqRig r(3, aqScan(3, ADC_EOC_SEQ_CONV), true);
	hostAdcSetAnalog(3, 1, 11);
	hostAdcSetAnalog(3, 2, 2222);
	hostAdcSetAnalog(3, 3, 4095);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 3) == HAL_OK);
	for (unsigned i = 0; i < 3; ++i) {
		REQUIRE(hostAdcConvert(3, 1) == 1u);
		r.buf[i] = r.I()->DR; /* the DMA engine reads DR on the request and so clears EOC */
		hostDmaStepItems(1, r.map.stream, 1);
		HAL_DMA_IRQHandler(&r.d);
	}
	CHECK(r.buf[0] == 11u);
	CHECK(r.buf[1] == 2222u);
	CHECK(r.buf[2] == 4095u);
	CHECK(adcCounts.conv == 1u);
	CHECK(r.h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC));
	CHECK((r.I()->SR & aqSR_OVR) == 0u);
}

// ============================================================================ fix pass

TEST_CASE_FIXTURE(AdcFixture, "AQ2 IRQHandler of a software started single conversion with EOC after every conversion reaches READY and disables EOCIE") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, [] { AqCfg c; c.eocs = ADC_EOC_SINGLE_CONV; return c; }());
		REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
		REQUIRE(hostAdcConvert(n, 1) == 1u);
		HAL_ADC_IRQHandler(&r.h);
		CHECK(r.h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC));
		CHECK((r.I()->CR1 & aqCR1_EOCIE) == 0u);
		CHECK((r.I()->CR1 & aqCR1_OVRIE) == aqCR1_OVRIE);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA completion of a software started single conversion with EOC after every conversion reaches READY") {
	AqCfg c;
	c.eocs = ADC_EOC_SINGLE_CONV;
	AqRig r(1, c, true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 1) == HAL_OK);
	r.dmaComplete();
	CHECK(r.h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC));
	CHECK((r.h.State & HAL_ADC_STATE_REG_BUSY) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA completion of a software started conversion disables a set EOC interrupt") {
	AqRig r(1, AqCfg(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 1) == HAL_OK);
	r.I()->CR1 |= aqCR1_EOCIE;
	r.dmaComplete();
	CHECK((r.I()->CR1 & aqCR1_EOCIE) == 0u);
	CHECK((r.I()->CR1 & aqCR1_OVRIE) == aqCR1_OVRIE);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ4 DMA completion in continuous mode keeps a set EOC interrupt") {
	AqRig r(1, aqCont(), true);
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	r.I()->CR1 |= aqCR1_EOCIE;
	r.dmaComplete();
	CHECK((r.I()->CR1 & aqCR1_EOCIE) == aqCR1_EOCIE);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA clears the whole error code when no injected conversion is running") {
	AqRig r(3, AqCfg(), true);
	r.h.ErrorCode = HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA | HAL_ADC_ERROR_INTERNAL;
	REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
	CHECK(r.h.ErrorCode == HAL_ADC_ERROR_NONE);
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT with an external trigger selection but edge NONE still software starts") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, aqSel());
		REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
		CHECK(hostAdcRegularRunning(n) == 1);
		CHECK((r.I()->SR & aqSR_STRT) == aqSR_STRT);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA with an external trigger selection but edge NONE still software starts") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, aqSel(), true);
		REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
		CHECK(hostAdcRegularRunning(n) == 1);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Start_IT in multi ADC mode with an external trigger selection but edge NONE starts only ADC1") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, aqSel());
		ADC->CCR = 0x6u;
		REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
		CHECK(hostAdcRegularRunning(n) == (n == 1 ? 1 : 0));
		ADC->CCR = 0;
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Start_DMA in multi ADC mode with an external trigger selection but edge NONE starts only ADC1") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, aqSel(), true);
		ADC->CCR = 0x6u;
		REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
		CHECK(hostAdcRegularRunning(n) == (n == 1 ? 1 : 0));
		ADC->CCR = 0;
	}
}

namespace {
AqCfg aqFull() {
	AqCfg c;
	c.cont = true;
	c.eocs = ADC_EOC_SINGLE_CONV;
	c.dmaCont = true;
	c.align = ADC_DATAALIGN_LEFT;
	return c;
}
} // namespace

TEST_CASE_FIXTURE(AdcFixture, "AQ1 Stop_IT of a continuous, left aligned ADC with DDS and EOCS set reaches READY without ERROR_INTERNAL") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, aqFull());
		REQUIRE(HAL_ADC_Start_IT(&r.h) == HAL_OK);
		CHECK(HAL_ADC_Stop_IT(&r.h) == HAL_OK);
		CHECK((r.I()->CR2 & aqCR2_ADON) == 0u);
		CHECK((r.h.State & HAL_ADC_STATE_READY) == HAL_ADC_STATE_READY);
		CHECK((r.h.State & HAL_ADC_STATE_ERROR_INTERNAL) == 0u);
		CHECK((r.I()->CR1 & (aqCR1_EOCIE | aqCR1_OVRIE)) == 0u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AQ3 Stop_DMA of a continuous, left aligned ADC with DDS, DMA and EOCS set reaches READY without ERROR_INTERNAL") {
	for (unsigned n = 1; n <= 3; ++n) {
		INFO("ADC" << n);
		AqRig r(n, aqFull(), true, true);
		REQUIRE(HAL_ADC_Start_DMA(&r.h, r.buf, 4) == HAL_OK);
		CHECK(HAL_ADC_Stop_DMA(&r.h) == HAL_OK);
		CHECK((r.I()->CR2 & (aqCR2_ADON | aqCR2_DMA)) == 0u);
		CHECK((r.h.State & HAL_ADC_STATE_READY) == HAL_ADC_STATE_READY);
		CHECK((r.h.State & HAL_ADC_STATE_ERROR_INTERNAL) == 0u);
		CHECK((r.I()->CR2 & aqCR2_DDS) == aqCR2_DDS);
	}
}
