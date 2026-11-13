/*
 * P12 T6b group AX: the injected group and the multi ADC mode of the real ST HAL (stm32f4xx_hal_adc_ex.c:
 * HAL_ADCEx_InjectedConfigChannel, InjectedStart, InjectedStart_IT, InjectedStop, InjectedStop_IT, InjectedPollForConversion,
 * InjectedGetValue, MultiModeConfigChannel, MultiModeStart_DMA, MultiModeStop_DMA, MultiModeGetValue and the multi mode DMA callbacks)
 * against the ADC model (fakes/stm32_hal/HalAdc.h) and the DMA model (HalDma.h). Requirements AX1-AX4 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T6B_REQUIREMENTS.md; compiled into ffb_hal_tests (F407VG) and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 chapter 8 (HAL ADC Extension Driver: ADC_InjectionConfTypeDef field notes, HAL_ADCEx_* notes) and the HAL_ADC_STATE_*
 * definitions of chapter 7; RM0090 Rev 22:
 *  - Table 1 / 13.13: ADC1 0x40012000, ADC2 0x40012100, ADC3 0x40012200, common registers 0x40012300 (CSR 0x00, CCR 0x04, CDR 0x08).
 *  - 13.13.1 SR: JEOC bit 2 is set at the end of the conversion of ALL injected channels of the group (EOCS only concerns the regular group). The tests use EOCS = 0 unless they say otherwise.
 *  - 13.13.2 CR1: JEOCIE bit 7, JAUTO bit 10, JDISCEN bit 12, DISCNUM bits 15:13, JAWDEN bit 22, AWDEN bit 23, RES bits 25:24, OVRIE bit 26.
 *  - 13.13.3 CR2: ADON bit 0, JEXTSEL bits 19:16 (Table: 0000 T1_CC4, 0001 T1_TRGO, 0010 T2_CC1, 0011 T2_TRGO, 0100 T3_CC2, 0101 T3_CC4,
 *    0110 T4_CC1, 0111 T4_CC2, 1000 T4_CC3, 1001 T4_TRGO, 1010 T5_CC4, 1011 T5_TRGO, 1100 T8_CC2, 1101 T8_CC3, 1110 T8_CC4, 1111 EXTI15),
 *    JEXTEN bits 21:20 (00 disabled, 01 rising, 10 falling, 11 both), JSWSTART bit 22 (reads 0).
 *  - 13.13.4 / 13.13.5 SMPR1 (channels 10-18, 3 bits each from bit 0) and SMPR2 (channels 0-9, 3 bits each from bit 0); codes 000 3, 001 15, 010 28,
 *    011 56, 100 84, 101 112, 110 144, 111 480 cycles.
 *  - 13.13.6 JOFRx (bits 11:0), 13.13.12 JSQR: JL bits 21:20 (number of conversions - 1), JSQ4 bits 19:15, JSQ3 14:10, JSQ2 9:5, JSQ1 4:0; with
 *    JL = n - 1 the conversions run from JSQ(5 - n) to JSQ4, so conversion r (1..n) is the field at bit 5 * (3 - n + r). 13.13.13 JDRx.
 *  - 13.4: the injected result is the conversion minus the offset (a negative result is sign extended to 16 bits); data alignment right / left.
 *  - 13.9 / 13.13.16 CCR: MULTI bits 4:0 (00001 .. 01001 dual: ADC1 and ADC2 together, ADC3 independent; 10001 .. 11001 triple), DELAY bits 11:8
 *    (code n = n + 5 ADC clocks), DDS bit 13, DMA bits 15:14, ADCPRE bits 17:16, VBATE bit 22, TSVREFE bit 23. The start of conversion is triggered
 *    by the ADC1 master to the slaves. ADC_CDR is the multi mode data register; the DMA streams of the ADC are DMA2 stream 0 / 4 channel 0 (ADC1),
 *    stream 2 / 3 channel 1 (ADC2), stream 0 / 1 channel 2 (ADC3) (Table 44).
 *  - The temperature sensor (channel 16 on the F405/F407), VREFINT (17) and VBAT (18) are inputs of ADC1 only.
 *
 * NEEDS REVIEW proposals:
 *  NEEDS REVIEW 340/1  AX1: a refused injected configuration (lock) writes no register; the HAL unlocks after a successful call.
 *  NEEDS REVIEW 340/2  AX1: ConfigChannel does not touch JAWDEN / AWDEN / AWDSGL / AWDCH (the injected analog watchdog is enabled by HAL_ADC_AnalogWDGConfig).
 *  NEEDS REVIEW 340/3  AX2: InjectedStart / Start_IT keep the error code while the regular group is busy and clear it otherwise (HAL comment, UM1725 silent).
 *  NEEDS REVIEW 340/4  AX2: Stop / Stop_IT refused (regular busy or JAUTO) return HAL_ERROR and set HAL_ADC_STATE_ERROR_CONFIG, the ADC stays enabled.
 *  NEEDS REVIEW 340/5  AX3: InjectedPollForConversion with auto-injection or a continuous regular group keeps INJ_BUSY (UM1725 INJ_BUSY text: "can occur").
 *  NEEDS REVIEW 340/6  AX3: InjectedGetValue clears JEOC only (JSTRT stays); an invalid rank returns 0 and still clears JEOC.
 *  NEEDS REVIEW 340/7  AX3: InjectedPollForConversion after a timeout unlocks the handle and sets State TIMEOUT (replacing every other bit).
 *  NEEDS REVIEW 340/8  AX3: a left-aligned injected result at 10 / 8 / 6 bit resolution is shifted up to bit 15 (RM0090 shows only 12 bit alignment figures).
 *  NEEDS REVIEW 340/9  AX4: MultiModeStart_DMA returns HAL_OK although the DMA stream cannot start (busy); it does not enable the slave ADCs; it sets OVRIE and
 *            clears EOC.
 *  NEEDS REVIEW 340/10 AX4: MultiModeStop_DMA returns the DMA abort status (HAL_ERROR when the stream was idle, HAL_TIMEOUT when it never stops) but disables the ADC and sets READY.
 *  NEEDS REVIEW 340/11 AX4: the multi mode DMA complete callback ends the regular group state (REG_BUSY cleared, READY set unless INJ_BUSY) for software start,
 *            single conversion mode, and keeps it for the EOCS + scan case; an internal / DMA error state turns the completion into the error callback.
 *
 * Every file-local symbol is prefixed ax or sits in the anonymous namespace.
 */
#include "AdcTestSupport.h"
#include "HalDma.h"
#include "HalHostSystem.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstring>

namespace {

using u32 = uint32_t;

constexpr u32 axSR_JEOC = 1u << 2;
constexpr u32 axSR_JSTRT = 1u << 3;
constexpr u32 axSR_EOC = 1u << 1;

constexpr u32 axCR1_EOCIE = 1u << 5;
constexpr u32 axCR1_AWDIE = 1u << 6;
constexpr u32 axCR1_JEOCIE = 1u << 7;
constexpr u32 axCR1_SCAN = 1u << 8;
constexpr u32 axCR1_AWDSGL = 1u << 9;
constexpr u32 axCR1_JAUTO = 1u << 10;
constexpr u32 axCR1_JDISCEN = 1u << 12;
constexpr u32 axCR1_DISCNUM = 7u << 13;
constexpr u32 axCR1_JAWDEN = 1u << 22;
constexpr u32 axCR1_AWDEN = 1u << 23;
constexpr u32 axCR1_RES10 = 1u << 24;
constexpr u32 axCR1_OVRIE = 1u << 26;

constexpr u32 axCR2_ADON = 1u << 0;
constexpr u32 axCR2_CONT = 1u << 1;
constexpr u32 axCR2_DMA = 1u << 8;
constexpr u32 axCR2_EOCS = 1u << 10;
constexpr u32 axCR2_ALIGN = 1u << 11;
constexpr u32 axCR2_JEXTSEL = 0xFu << 16;
constexpr u32 axCR2_JEXTEN = 3u << 20;
constexpr u32 axCR2_JSWSTART = 1u << 22;

constexpr u32 axCCR_MULTI = 0x1Fu;
constexpr u32 axCCR_DELAY = 0xFu << 8;
constexpr u32 axCCR_DDS = 1u << 13;
constexpr u32 axCCR_DMA = 3u << 14;
constexpr u32 axCCR_ADCPRE = 3u << 16;
constexpr u32 axCCR_VBATE = 1u << 22;
constexpr u32 axCCR_TSVREFE = 1u << 23;

constexpr u32 axDMA_EN = 1u << 0;
constexpr u32 axDMA_TEIE = 1u << 2;
constexpr u32 axDMA_HTIE = 1u << 3;
constexpr u32 axDMA_TCIE = 1u << 4;

constexpr u32 axCdrAddress = 0x40012308u; /* RM0090 Table 71: common registers 0x40012300 + CDR offset 0x08 */

#define AX_ADC_N(var)  \
	unsigned var = 0;  \
	SUBCASE("ADC1") var = 1; \
	SUBCASE("ADC2") var = 2; \
	SUBCASE("ADC3") var = 3; \
	REQUIRE(var != 0u)

ADC_HandleTypeDef axHandle(unsigned n) {
	ADC_HandleTypeDef h{};
	h.Instance = adcInstance(n);
	h.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	h.Init.Resolution = ADC_RESOLUTION_12B;
	h.Init.ScanConvMode = DISABLE;
	h.Init.ContinuousConvMode = DISABLE;
	h.Init.DiscontinuousConvMode = DISABLE;
	h.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	h.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	h.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	h.Init.NbrOfConversion = 1;
	h.Init.DMAContinuousRequests = DISABLE;
	h.Init.EOCSelection = ADC_EOC_SEQ_CONV; /* EOCS = 0 */
	return h;
}

void axInit(ADC_HandleTypeDef &h) { REQUIRE(HAL_ADC_Init(&h) == HAL_OK); }

ADC_InjectionConfTypeDef axInj(u32 ch, u32 rank, u32 count, u32 offset = 0) {
	ADC_InjectionConfTypeDef j{};
	j.InjectedChannel = ch;
	j.InjectedRank = rank;
	j.InjectedNbrOfConversion = count;
	j.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
	j.InjectedOffset = offset;
	j.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
	j.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
	j.AutoInjectedConv = DISABLE;
	j.InjectedDiscontinuousConvMode = DISABLE;
	return j;
}

void axInjCfg(ADC_HandleTypeDef &h, const ADC_InjectionConfTypeDef &j) {
	ADC_InjectionConfTypeDef c = j;
	REQUIRE(HAL_ADCEx_InjectedConfigChannel(&h, &c) == HAL_OK);
}

/* The injected conversion of rank r (1..4) uses channel 2 + r. */
constexpr u32 axChannelOf(u32 rank) { return 2u + rank; }

void axSequence(ADC_HandleTypeDef &h, unsigned n, u32 count, const u32 *analog, const u32 *offset) {
	for (u32 r = 1; r <= count; ++r) {
		axInjCfg(h, axInj(axChannelOf(r), r, count, offset ? offset[r - 1] : 0u));
		hostAdcSetAnalog(n, axChannelOf(r), static_cast<uint16_t>(analog[r - 1]));
	}
}

void axRegChannel(ADC_HandleTypeDef &h, u32 ch, u32 rank) {
	ADC_ChannelConfTypeDef c{};
	c.Channel = ch;
	c.Rank = rank;
	c.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	REQUIRE(HAL_ADC_ConfigChannel(&h, &c) == HAL_OK);
}

/* One injected channel (rank 1, channel 3) with the analog value 0x123. */
void axOneInjected(ADC_HandleTypeDef &h, unsigned n, bool autoInj = false) {
	ADC_InjectionConfTypeDef j = axInj(3, 1, 1);
	j.AutoInjectedConv = autoInj ? ENABLE : DISABLE;
	axInjCfg(h, j);
	hostAdcSetAnalog(n, 3, 0x123);
}

/* External injected trigger, rising edge, JEXTSEL = TIM1 TRGO. */
void axOneInjectedExternal(ADC_HandleTypeDef &h, unsigned n) {
	ADC_InjectionConfTypeDef j = axInj(3, 1, 1);
	j.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
	j.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
	axInjCfg(h, j);
	hostAdcSetAnalog(n, 3, 0x123);
}

void axMultiMode(ADC_HandleTypeDef &h, u32 mode, u32 dma = ADC_DMAACCESSMODE_DISABLED, u32 delay = ADC_TWOSAMPLINGDELAY_5CYCLES) {
	ADC_MultiModeTypeDef m{};
	m.Mode = mode;
	m.DMAAccessMode = dma;
	m.TwoSamplingDelay = delay;
	REQUIRE(HAL_ADCEx_MultiModeConfigChannel(&h, &m) == HAL_OK);
}

u32 axCCR() { return ADC123_COMMON->CCR; }

/* ---- DMA of the multi mode ---- */
struct AxDmaMap {
	DMA_Stream_TypeDef *inst;
	unsigned stream;
	u32 channel;
};
AxDmaMap axDmaMap(unsigned n) { /* RM0090 Table 44 (DMA2 = controller 1) */
	if (n == 1) return {DMA2_Stream0, 0, DMA_CHANNEL_0};
	if (n == 2) return {DMA2_Stream2, 2, DMA_CHANNEL_1};
	return {DMA2_Stream1, 1, DMA_CHANNEL_2};
}

struct AxMulti {
	unsigned n;
	AxDmaMap m;
	ADC_HandleTypeDef h;
	DMA_HandleTypeDef d;
	u32 buf[8];

	explicit AxMulti(unsigned adc, bool circular = false, void (*tweak)(ADC_HandleTypeDef &) = nullptr) : n(adc), m(axDmaMap(adc)) {
		std::memset(buf, 0, sizeof buf);
		h = axHandle(n);
		if (tweak) tweak(h);
		axInit(h);
		std::memset(&d, 0, sizeof d);
		d.Instance = m.inst;
		d.Init.Channel = m.channel;
		d.Init.Direction = DMA_PERIPH_TO_MEMORY;
		d.Init.PeriphInc = DMA_PINC_DISABLE;
		d.Init.MemInc = DMA_MINC_ENABLE;
		d.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		d.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		d.Init.Mode = circular ? DMA_CIRCULAR : DMA_NORMAL;
		d.Init.Priority = DMA_PRIORITY_LOW;
		d.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		d.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		d.Init.MemBurst = DMA_MBURST_SINGLE;
		d.Init.PeriphBurst = DMA_PBURST_SINGLE;
		REQUIRE(HAL_DMA_Init(&d) == HAL_OK);
		h.DMA_Handle = &d;
		d.Parent = &h;
		adcSupportReset();
	}
	u32 bufAddr() const { return static_cast<u32>(reinterpret_cast<uintptr_t>(buf)); }
	void start(u32 length = 4) { REQUIRE(HAL_ADCEx_MultiModeStart_DMA(&h, buf, length) == HAL_OK); }
	void complete() {
		hostDmaComplete(1, m.stream);
		HAL_DMA_IRQHandler(&d);
	}
	void raise(u32 flag) {
		hostDmaRaise(1, m.stream, flag);
		HAL_DMA_IRQHandler(&d);
	}
};

void axTweakContinuous(ADC_HandleTypeDef &h) { h.Init.ContinuousConvMode = ENABLE; }
void axTweakExternal(ADC_HandleTypeDef &h) {
	h.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	h.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
}
void axTweakScanEocs(ADC_HandleTypeDef &h) {
	h.Init.ScanConvMode = ENABLE;
	h.Init.NbrOfConversion = 3;
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV; /* EOCS = 1 */
}
void axTweakDmaContinuous(ADC_HandleTypeDef &h) { h.Init.DMAContinuousRequests = ENABLE; }

} // namespace

/* ======================================================================================= AX1 InjectedConfigChannel */

TEST_CASE_FIXTURE(AdcFixture, "AX1 the injected sequence length JL is the number of conversions minus one") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	for (u32 count = 1; count <= 4; ++count) {
		CAPTURE(count);
		axInjCfg(h, axInj(3, 1, count));
		CHECK(((h.Instance->JSQR >> 20) & 3u) == count - 1u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 conversion r of an n conversion sequence is the JSQ field at bit 5 * (3 - n + r)") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	for (u32 count = 1; count <= 4; ++count) {
		for (u32 rank = 1; rank <= count; ++rank) {
			const u32 ch = 2u * count + rank + 3u; /* 6 .. 15, distinct for every pair */
			CAPTURE(count);
			CAPTURE(rank);
			HalFixture::poke(&h.Instance->JSQR, 0u);
			axInjCfg(h, axInj(ch, rank, count));
			CHECK(h.Instance->JSQR == (((count - 1u) << 20) | (ch << (5u * (3u - count + rank)))));
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 four ranks of a four conversion sequence fill JSQ1 to JSQ4 in order") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	const u32 ch[4] = {3, 5, 7, 9};
	for (u32 r = 1; r <= 4; ++r) axInjCfg(h, axInj(ch[r - 1], r, 4));
	/* hand derivation: JSQ1 = 3 at bit 0, JSQ2 = 5 at bit 5, JSQ3 = 7 at bit 10, JSQ4 = 9 at bit 15, JL = 3 at bit 20 */
	CHECK(h.Instance->JSQR == (3u | (5u << 5) | (7u << 10) | (9u << 15) | (3u << 20)));
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 configuring a rank again replaces its channel and leaves the other ranks") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	const u32 ch[4] = {3, 5, 7, 9};
	for (u32 r = 1; r <= 4; ++r) axInjCfg(h, axInj(ch[r - 1], r, 4));
	axInjCfg(h, axInj(11, 2, 4));
	CHECK(h.Instance->JSQR == (3u | (11u << 5) | (7u << 10) | (9u << 15) | (3u << 20)));
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 a later call with another conversion count rewrites JL") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	for (u32 r = 1; r <= 4; ++r) axInjCfg(h, axInj(2u + r, r, 4));
	REQUIRE(((h.Instance->JSQR >> 20) & 3u) == 3u);
	axInjCfg(h, axInj(8, 1, 2));
	CHECK(((h.Instance->JSQR >> 20) & 3u) == 1u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 the offset of rank r goes to JOFRr only") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	volatile u32 *const jofr[4] = {&h.Instance->JOFR1, &h.Instance->JOFR2, &h.Instance->JOFR3, &h.Instance->JOFR4};
	const u32 off[4] = {0x123, 0x456, 0x789, 0xABC};
	for (u32 r = 1; r <= 4; ++r) {
		CAPTURE(r);
		axInjCfg(h, axInj(2u + r, r, 4, off[r - 1]));
		for (u32 k = 1; k <= 4; ++k) CHECK(*jofr[k - 1] == (k <= r ? off[k - 1] : 0u));
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 a new offset replaces the old one instead of being ORed") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axInjCfg(h, axInj(3, 1, 1, 0xFFF));
	REQUIRE(h.Instance->JOFR1 == 0xFFFu);
	axInjCfg(h, axInj(3, 1, 1, 0x001));
	CHECK(h.Instance->JOFR1 == 0x001u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 the sampling time of every channel 0 to 18 is written to its SMPR field for all eight codes") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	const u32 codes[8] = {ADC_SAMPLETIME_3CYCLES,  ADC_SAMPLETIME_15CYCLES, ADC_SAMPLETIME_28CYCLES,  ADC_SAMPLETIME_56CYCLES,
	                      ADC_SAMPLETIME_84CYCLES, ADC_SAMPLETIME_112CYCLES, ADC_SAMPLETIME_144CYCLES, ADC_SAMPLETIME_480CYCLES};
	for (u32 ch = 0; ch <= 18; ++ch) {
		for (u32 i = 0; i < 8; ++i) {
			CAPTURE(ch);
			CAPTURE(i);
			HalFixture::poke(&h.Instance->SMPR1, 0u);
			HalFixture::poke(&h.Instance->SMPR2, 0u);
			ADC_InjectionConfTypeDef j = axInj(ch, 1, 1);
			j.InjectedSamplingTime = codes[i];
			axInjCfg(h, j);
			/* RM0090 13.13.4 / 13.13.5: SMPRx code i is the value i (000 .. 111) */
			if (ch <= 9u) {
				CHECK(h.Instance->SMPR2 == (i << (3u * ch)));
				CHECK(h.Instance->SMPR1 == 0u);
			} else {
				CHECK(h.Instance->SMPR1 == (i << (3u * (ch - 10u))));
				CHECK(h.Instance->SMPR2 == 0u);
			}
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 a new sampling time clears the old code of the channel and keeps the neighbours") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	for (u32 ch = 0; ch <= 18; ++ch) {
		CAPTURE(ch);
		HalFixture::poke(&h.Instance->SMPR1, 0x07FFFFFFu);
		HalFixture::poke(&h.Instance->SMPR2, 0x3FFFFFFFu);
		ADC_InjectionConfTypeDef j = axInj(ch, 1, 1);
		j.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES; /* code 000 */
		axInjCfg(h, j);
		if (ch <= 9u) {
			CHECK(h.Instance->SMPR2 == (0x3FFFFFFFu & ~(7u << (3u * ch))));
			CHECK(h.Instance->SMPR1 == 0x07FFFFFFu);
		} else {
			CHECK(h.Instance->SMPR1 == (0x07FFFFFFu & ~(7u << (3u * (ch - 10u)))));
			CHECK(h.Instance->SMPR2 == 0x3FFFFFFFu);
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 an injected sampling time overrides the one a regular call gave the same channel") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	ADC_ChannelConfTypeDef c{};
	c.Channel = ADC_CHANNEL_5;
	c.Rank = 1;
	c.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	REQUIRE(HAL_ADC_ConfigChannel(&h, &c) == HAL_OK);
	REQUIRE(((h.Instance->SMPR2 >> 15) & 7u) == 7u);
	ADC_InjectionConfTypeDef j = axInj(5, 1, 1);
	j.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES; /* UM1725: the channel property is updated to the last setting */
	axInjCfg(h, j);
	CHECK(((h.Instance->SMPR2 >> 15) & 7u) == 1u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 the temperature sensor and VREFINT of ADC1 enable TSVREFE and not VBATE") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	const u32 channels[2] = {ADC_CHANNEL_TEMPSENSOR, ADC_CHANNEL_VREFINT};
	for (u32 c : channels) {
		CAPTURE(c);
		HalFixture::poke(&ADC123_COMMON->CCR, 0u);
		axInjCfg(h, axInj(c, 1, 1));
		CHECK((axCCR() & axCCR_TSVREFE) == axCCR_TSVREFE);
		CHECK((axCCR() & axCCR_VBATE) == 0u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 the VBAT channel of ADC1 enables VBATE and not TSVREFE") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	HalFixture::poke(&ADC123_COMMON->CCR, 0u);
	axInjCfg(h, axInj(ADC_CHANNEL_VBAT, 1, 1));
	CHECK((axCCR() & axCCR_VBATE) == axCCR_VBATE);
	CHECK((axCCR() & axCCR_TSVREFE) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 the channels 16 to 18 of ADC2 and ADC3 do not change CCR") {
	for (unsigned n = 2; n <= 3; ++n) {
		ADC_HandleTypeDef h = axHandle(n);
		axInit(h);
		HalFixture::poke(&ADC123_COMMON->CCR, 0u);
		for (u32 ch = 16; ch <= 18; ++ch) {
			CAPTURE(n);
			CAPTURE(ch);
			axInjCfg(h, axInj(ch, 1, 1));
			CHECK((axCCR() & (axCCR_VBATE | axCCR_TSVREFE)) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 the channels 0 to 15 of ADC1 do not change VBATE and TSVREFE") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	HalFixture::poke(&ADC123_COMMON->CCR, 0u);
	for (u32 ch = 0; ch <= 15; ++ch) {
		CAPTURE(ch);
		axInjCfg(h, axInj(ch, 1, 1));
		CHECK((axCCR() & (axCCR_VBATE | axCCR_TSVREFE)) == 0u);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 every external trigger source and edge reaches JEXTSEL and JEXTEN") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	const u32 src[16] = {ADC_EXTERNALTRIGINJECCONV_T1_CC4, ADC_EXTERNALTRIGINJECCONV_T1_TRGO, ADC_EXTERNALTRIGINJECCONV_T2_CC1,
	                     ADC_EXTERNALTRIGINJECCONV_T2_TRGO, ADC_EXTERNALTRIGINJECCONV_T3_CC2, ADC_EXTERNALTRIGINJECCONV_T3_CC4,
	                     ADC_EXTERNALTRIGINJECCONV_T4_CC1,  ADC_EXTERNALTRIGINJECCONV_T4_CC2, ADC_EXTERNALTRIGINJECCONV_T4_CC3,
	                     ADC_EXTERNALTRIGINJECCONV_T4_TRGO, ADC_EXTERNALTRIGINJECCONV_T5_CC4, ADC_EXTERNALTRIGINJECCONV_T5_TRGO,
	                     ADC_EXTERNALTRIGINJECCONV_T8_CC2,  ADC_EXTERNALTRIGINJECCONV_T8_CC3, ADC_EXTERNALTRIGINJECCONV_T8_CC4,
	                     ADC_EXTERNALTRIGINJECCONV_EXT_IT15};
	const u32 edge[3] = {ADC_EXTERNALTRIGINJECCONVEDGE_RISING, ADC_EXTERNALTRIGINJECCONVEDGE_FALLING, ADC_EXTERNALTRIGINJECCONVEDGE_RISINGFALLING};
	for (u32 s = 0; s < 16; ++s) {
		for (u32 e = 0; e < 3; ++e) {
			CAPTURE(s);
			CAPTURE(e);
			HalFixture::poke(&h.Instance->CR2, axCR2_JEXTSEL | axCR2_JEXTEN); /* old content must be replaced */
			ADC_InjectionConfTypeDef j = axInj(3, 1, 1);
			j.ExternalTrigInjecConv = src[s];
			j.ExternalTrigInjecConvEdge = edge[e];
			axInjCfg(h, j);
			/* RM0090 13.13.3: JEXTSEL = s (Table), JEXTEN 01 rising, 10 falling, 11 both */
			CHECK((h.Instance->CR2 & axCR2_JEXTSEL) == (s << 16));
			CHECK((h.Instance->CR2 & axCR2_JEXTEN) == ((e + 1u) << 20));
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 an external source with the edge NONE leaves JEXTEN at 00 (trigger detection disabled)") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	ADC_InjectionConfTypeDef j = axInj(3, 1, 1);
	j.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T2_CC1;
	j.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
	axInjCfg(h, j);
	CHECK((h.Instance->CR2 & axCR2_JEXTEN) == 0u);
	CHECK((h.Instance->CR2 & axCR2_JEXTSEL) == (2u << 16));
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 the software start selection clears JEXTSEL and JEXTEN") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	HalFixture::poke(&h.Instance->CR2, axCR2_JEXTSEL | axCR2_JEXTEN);
	axInjCfg(h, axInj(3, 1, 1)); /* ADC_INJECTED_SOFTWARE_START */
	CHECK((h.Instance->CR2 & (axCR2_JEXTSEL | axCR2_JEXTEN)) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 the software start selection discards the edge parameter") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	ADC_InjectionConfTypeDef j = axInj(3, 1, 1);
	j.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING; /* UM1725: discarded with ADC_INJECTED_SOFTWARE_START */
	axInjCfg(h, j);
	CHECK((h.Instance->CR2 & axCR2_JEXTEN) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 AutoInjectedConv sets and clears JAUTO") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	ADC_InjectionConfTypeDef j = axInj(3, 1, 1);
	j.AutoInjectedConv = ENABLE;
	axInjCfg(h, j);
	CHECK((h.Instance->CR1 & axCR1_JAUTO) == axCR1_JAUTO);
	j.AutoInjectedConv = DISABLE;
	axInjCfg(h, j);
	CHECK((h.Instance->CR1 & axCR1_JAUTO) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 InjectedDiscontinuousConvMode sets and clears JDISCEN with scan mode on and continuous mode off") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	h.Init.ScanConvMode = ENABLE;
	axInit(h);
	ADC_InjectionConfTypeDef j = axInj(3, 1, 2);
	j.InjectedDiscontinuousConvMode = ENABLE;
	axInjCfg(h, j);
	CHECK((h.Instance->CR1 & axCR1_JDISCEN) == axCR1_JDISCEN);
	j.InjectedDiscontinuousConvMode = DISABLE;
	axInjCfg(h, j);
	CHECK((h.Instance->CR1 & axCR1_JDISCEN) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 the injected group leaves DISCNUM alone (one channel per trigger is fixed)") {
	ADC_HandleTypeDef h = axHandle(1);
	h.Init.ScanConvMode = ENABLE;
	axInit(h);
	HalFixture::poke(&h.Instance->CR1, h.Instance->CR1 | (5u << 13));
	ADC_InjectionConfTypeDef j = axInj(3, 1, 2);
	j.InjectedDiscontinuousConvMode = ENABLE;
	axInjCfg(h, j);
	CHECK((h.Instance->CR1 & axCR1_DISCNUM) == (5u << 13));
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 the injected configuration leaves the unrelated CR1 and CR2 bits unchanged") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	const u32 cr1 = axCR1_SCAN | axCR1_EOCIE | axCR1_RES10;
	const u32 cr2 = axCR2_CONT | axCR2_ALIGN | axCR2_EOCS | axCR2_DMA;
	HalFixture::poke(&h.Instance->CR1, cr1);
	HalFixture::poke(&h.Instance->CR2, cr2);
	axInjCfg(h, axInj(3, 1, 1));
	CHECK(h.Instance->CR1 == cr1);
	CHECK(h.Instance->CR2 == cr2);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 characterization: the injected configuration does not touch the analog watchdog bits of CR1") {
	// NEEDS REVIEW 340/2
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	const u32 wd = axCR1_JAWDEN | axCR1_AWDEN | axCR1_AWDSGL | 7u;
	HalFixture::poke(&h.Instance->CR1, wd);
	axInjCfg(h, axInj(3, 1, 1));
	CHECK((h.Instance->CR1 & (axCR1_JAWDEN | axCR1_AWDEN | axCR1_AWDSGL | 0x1Fu)) == wd);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 a locked handle returns HAL_BUSY and writes no register") {
	// NEEDS REVIEW 340/1
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	h.Lock = HAL_LOCKED;
	ADC_InjectionConfTypeDef j = axInj(3, 1, 4, 0x55);
	j.AutoInjectedConv = ENABLE;
	CHECK(HAL_ADCEx_InjectedConfigChannel(&h, &j) == HAL_BUSY);
	CHECK(h.Instance->JSQR == 0u);
	CHECK(h.Instance->JOFR1 == 0u);
	CHECK((h.Instance->CR1 & axCR1_JAUTO) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 a successful configuration returns HAL_OK and leaves the handle unlocked") {
	// NEEDS REVIEW 340/1
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	ADC_InjectionConfTypeDef j = axInj(3, 1, 1);
	CHECK(HAL_ADCEx_InjectedConfigChannel(&h, &j) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 the injected configuration does not change the state of the handle") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	const u32 before = h.State;
	axInjCfg(h, axInj(3, 1, 1));
	CHECK(h.State == before);
	CHECK(h.State == HAL_ADC_STATE_READY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 the external trigger source is not changed while the ADC is enabled" * KNOWN_ISSUE("V115 UM1725 8.1.1 ExternalTrigInjecConv: 'This parameter must be modified when ADC is disabled ... If ADC is enabled, this parameter setting is bypassed without error reporting'; HAL_ADCEx_InjectedConfigChannel (stm32f4xx_hal_adc_ex.c 884-893) writes JEXTSEL / JEXTEN whatever ADON is")) {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	HalFixture::poke(&h.Instance->CR2, axCR2_ADON); /* ADC enabled, JEXTSEL = 0000 (TIM1 CC4), JEXTEN = 00 */
	ADC_InjectionConfTypeDef j = axInj(3, 1, 1);
	j.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
	j.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
	REQUIRE(HAL_ADCEx_InjectedConfigChannel(&h, &j) == HAL_OK);
	CHECK((h.Instance->CR2 & axCR2_JEXTSEL) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 InjectedDiscontinuousConvMode is discarded while continuous mode is enabled" * KNOWN_ISSUE("V116 UM1725 8.1.1 InjectedDiscontinuousConvMode: 'Discontinuous mode can be enabled only if continuous mode is disabled. If continuous mode is enabled, this parameter setting is discarded'; the HAL (stm32f4xx_hal_adc_ex.c 912-921) sets JDISCEN without looking at Init.ContinuousConvMode")) {
	ADC_HandleTypeDef h = axHandle(1);
	h.Init.ScanConvMode = ENABLE;
	h.Init.ContinuousConvMode = ENABLE;
	axInit(h);
	ADC_InjectionConfTypeDef j = axInj(3, 1, 2);
	j.InjectedDiscontinuousConvMode = ENABLE;
	REQUIRE(HAL_ADCEx_InjectedConfigChannel(&h, &j) == HAL_OK);
	CHECK((h.Instance->CR1 & axCR1_JDISCEN) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX1 InjectedDiscontinuousConvMode is discarded while the scan mode is disabled" * KNOWN_ISSUE("V116 UM1725 8.1.1 InjectedDiscontinuousConvMode: 'Discontinuous mode is used only if sequencer is enabled (parameter ScanConvMode). If sequencer is disabled, this parameter is discarded'; the HAL (stm32f4xx_hal_adc_ex.c 912-921) sets JDISCEN without looking at Init.ScanConvMode")) {
	ADC_HandleTypeDef h = axHandle(1); /* ScanConvMode DISABLE, ContinuousConvMode DISABLE */
	axInit(h);
	ADC_InjectionConfTypeDef j = axInj(3, 1, 2);
	j.InjectedDiscontinuousConvMode = ENABLE;
	REQUIRE(HAL_ADCEx_InjectedConfigChannel(&h, &j) == HAL_OK);
	CHECK((h.Instance->CR1 & axCR1_JDISCEN) == 0u);
}

/* ======================================================================================= AX2 start / stop */

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart enables the ADC") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjected(h, n);
	REQUIRE((h.Instance->CR2 & axCR2_ADON) == 0u);
	CHECK(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK((h.Instance->CR2 & axCR2_ADON) == axCR2_ADON);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart with the software trigger launches the injected group and JSWSTART reads 0") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjected(h, n);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK(hostAdcInjectedRunning(n) == 1);
	CHECK((h.Instance->SR & axSR_JSTRT) == axSR_JSTRT);
	CHECK((h.Instance->CR2 & axCR2_JSWSTART) == 0u);
	CHECK(hostAdcRegularRunning(n) == 0);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart from READY leaves exactly INJ_BUSY in the state") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjected(h, n);
	REQUIRE(h.State == HAL_ADC_STATE_READY);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK(h.State == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart clears a previous INJ_EOC state bit") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	h.State |= HAL_ADC_STATE_INJ_EOC;
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK((h.State & HAL_ADC_STATE_INJ_EOC) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart clears a stale JEOC flag") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjectedExternal(h, n); /* no software start: the flag cannot come back */
	HalFixture::poke(&h.Instance->SR, axSR_JEOC);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK((h.Instance->SR & axSR_JEOC) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart does not enable the JEOC interrupt") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK((h.Instance->CR1 & axCR1_JEOCIE) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart with an external injected trigger enables the ADC and starts no conversion") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjectedExternal(h, n);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK((h.Instance->CR2 & axCR2_ADON) == axCR2_ADON);
	CHECK(hostAdcInjectedRunning(n) == 0);
	CHECK((h.Instance->SR & axSR_JSTRT) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 an external trigger after InjectedStart runs the injected group and sets JEOC") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjectedExternal(h, n);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcExternalTrigger(n, 1) == 1);
	CHECK(hostAdcConvert(n, 4) == 1u);
	CHECK((h.Instance->SR & axSR_JEOC) == axSR_JEOC);
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1) == 0x123u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart with auto-injection enables the ADC but does not software-start the injected group") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjected(h, n, true);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK((h.Instance->CR2 & axCR2_ADON) == axCR2_ADON);
	CHECK(hostAdcInjectedRunning(n) == 0);
	CHECK((h.Instance->SR & axSR_JSTRT) == 0u);
	CHECK(h.State == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 auto-injection runs the injected group after the regular sequence and gives JEOC") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axRegChannel(h, 1, 1);
	hostAdcSetAnalog(n, 1, 0x100);
	axOneInjected(h, n, true);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(n, 1) == 1u); /* the regular conversion */
	CHECK(hostAdcInjectedRunning(n) == 1);
	CHECK(hostAdcConvert(n, 1) == 1u); /* the injected conversion */
	CHECK((h.Instance->SR & axSR_JEOC) == axSR_JEOC);
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1) == 0x123u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart while the regular group runs keeps REG_BUSY and starts the injected group first") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axRegChannel(h, 1, 1);
	hostAdcSetAnalog(n, 1, 0x100);
	axOneInjected(h, n);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK(h.State == (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY));
	CHECK(hostAdcInjectedRunning(n) == 1);
	CHECK(hostAdcRegularRunning(n) == 1);
	CHECK(hostAdcConvert(n, 1) == 1u); /* RM0090 13.3.9: the injected conversion has priority */
	CHECK((h.Instance->SR & axSR_JEOC) == axSR_JEOC);
	CHECK((h.Instance->SR & axSR_EOC) == 0u);
	CHECK(hostAdcRegularRunning(n) == 1);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 characterization: InjectedStart clears the error code only when the regular group is not busy") {
	// NEEDS REVIEW 340/3
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	h.ErrorCode = HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA;
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_ADC_ERROR_NONE);
	HAL_ADCEx_InjectedStop(&h);
	h.State |= HAL_ADC_STATE_REG_BUSY;
	h.ErrorCode = HAL_ADC_ERROR_OVR;
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_ADC_ERROR_OVR);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 in triple mode InjectedStart software-starts the master ADC1 only") {
	ADC_HandleTypeDef h1 = axHandle(1), h2 = axHandle(2), h3 = axHandle(3);
	axInit(h1);
	axInit(h2);
	axInit(h3);
	axOneInjected(h1, 1);
	axOneInjected(h2, 2);
	axOneInjected(h3, 3);
	axMultiMode(h1, ADC_TRIPLEMODE_INJECSIMULT);
	REQUIRE(HAL_ADCEx_InjectedStart(&h2) == HAL_OK);
	REQUIRE(HAL_ADCEx_InjectedStart(&h3) == HAL_OK);
	CHECK(hostAdcInjectedRunning(2) == 0);
	CHECK(hostAdcInjectedRunning(3) == 0);
	REQUIRE(HAL_ADCEx_InjectedStart(&h1) == HAL_OK);
	CHECK(hostAdcInjectedRunning(1) == 1);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 in dual mode InjectedStart software-starts the master ADC1 and not the slave ADC2") {
	ADC_HandleTypeDef h1 = axHandle(1), h2 = axHandle(2);
	axInit(h1);
	axInit(h2);
	axOneInjected(h1, 1);
	axOneInjected(h2, 2);
	axMultiMode(h1, ADC_DUALMODE_INJECSIMULT);
	REQUIRE(HAL_ADCEx_InjectedStart(&h2) == HAL_OK);
	CHECK(hostAdcInjectedRunning(2) == 0);
	REQUIRE(HAL_ADCEx_InjectedStart(&h1) == HAL_OK);
	CHECK(hostAdcInjectedRunning(1) == 1);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 in dual mode InjectedStart software-starts the independent ADC3" * KNOWN_ISSUE("V117 RM0090 13.13.16 CCR MULTI: '00001 to 01001: Dual mode, ADC1 and ADC2 working together, ADC3 is independent'; HAL_ADCEx_InjectedStart (stm32f4xx_hal_adc_ex.c 218-227) starts only ADC1 whenever MULTI != 0, so an independent ADC3 is never software-started (HAL_ADC_Start handles this case at stm32f4xx_hal_adc.c 789-805)")) {
	ADC_HandleTypeDef h1 = axHandle(1), h3 = axHandle(3);
	axInit(h1);
	axInit(h3);
	axOneInjected(h3, 3);
	axMultiMode(h1, ADC_DUALMODE_INJECSIMULT);
	REQUIRE(HAL_ADCEx_InjectedStart(&h3) == HAL_OK);
	CHECK(hostAdcInjectedRunning(3) == 1);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart_IT enables the ADC and the JEOC interrupt") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjected(h, n);
	CHECK(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	CHECK((h.Instance->CR2 & axCR2_ADON) == axCR2_ADON);
	CHECK((h.Instance->CR1 & axCR1_JEOCIE) == axCR1_JEOCIE);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart_IT enables no other interrupt") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	CHECK((h.Instance->CR1 & (axCR1_EOCIE | axCR1_AWDIE | axCR1_OVRIE)) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart_IT with the software trigger launches the injected group") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjected(h, n);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	CHECK(hostAdcInjectedRunning(n) == 1);
	CHECK((h.Instance->SR & axSR_JSTRT) == axSR_JSTRT);
	CHECK(h.State == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart_IT with an external trigger or auto-injection starts no conversion") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjectedExternal(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	CHECK(hostAdcInjectedRunning(1) == 0);
	REQUIRE(HAL_ADCEx_InjectedStop_IT(&h) == HAL_OK);
	axOneInjected(h, 1, true);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	CHECK(hostAdcInjectedRunning(1) == 0);
	CHECK((h.Instance->CR1 & axCR1_JEOCIE) == axCR1_JEOCIE);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart_IT clears a stale JEOC and INJ_EOC") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjectedExternal(h, 1);
	HalFixture::poke(&h.Instance->SR, axSR_JEOC);
	h.State |= HAL_ADC_STATE_INJ_EOC;
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	CHECK((h.Instance->SR & axSR_JEOC) == 0u);
	CHECK((h.State & HAL_ADC_STATE_INJ_EOC) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 in triple mode InjectedStart_IT software-starts the master ADC1 only") {
	ADC_HandleTypeDef h1 = axHandle(1), h2 = axHandle(2), h3 = axHandle(3);
	axInit(h1);
	axInit(h2);
	axInit(h3);
	axOneInjected(h1, 1);
	axOneInjected(h2, 2);
	axOneInjected(h3, 3);
	axMultiMode(h1, ADC_TRIPLEMODE_INJECSIMULT);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h2) == HAL_OK);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h3) == HAL_OK);
	CHECK(hostAdcInjectedRunning(2) == 0);
	CHECK(hostAdcInjectedRunning(3) == 0);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h1) == HAL_OK);
	CHECK(hostAdcInjectedRunning(1) == 1);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 in dual mode InjectedStart_IT software-starts the independent ADC3" * KNOWN_ISSUE("V117 RM0090 13.13.16 CCR MULTI: '00001 to 01001: Dual mode, ADC1 and ADC2 working together, ADC3 is independent'; HAL_ADCEx_InjectedStart_IT (stm32f4xx_hal_adc_ex.c 324-333) starts only ADC1 whenever MULTI != 0, so an independent ADC3 is never software-started")) {
	ADC_HandleTypeDef h1 = axHandle(1), h3 = axHandle(3);
	axInit(h1);
	axInit(h3);
	axOneInjected(h3, 3);
	axMultiMode(h1, ADC_DUALMODE_INJECSIMULT);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h3) == HAL_OK);
	CHECK(hostAdcInjectedRunning(3) == 1);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart and InjectedStart_IT with a locked handle return HAL_BUSY and do not enable the ADC") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_ADCEx_InjectedStart(&h) == HAL_BUSY);
	CHECK(HAL_ADCEx_InjectedStart_IT(&h) == HAL_BUSY);
	CHECK((h.Instance->CR2 & axCR2_ADON) == 0u);
	CHECK((h.Instance->CR1 & axCR1_JEOCIE) == 0u);
	CHECK(h.State == HAL_ADC_STATE_READY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart unlocks the handle") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
	REQUIRE(HAL_ADCEx_InjectedStop(&h) == HAL_OK);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStop disables the ADC, stops the conversion and sets READY") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjected(h, n);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcInjectedRunning(n) == 1);
	CHECK(HAL_ADCEx_InjectedStop(&h) == HAL_OK);
	CHECK((h.Instance->CR2 & axCR2_ADON) == 0u);
	CHECK(hostAdcInjectedRunning(n) == 0);
	CHECK(h.State == HAL_ADC_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStop after a finished conversion leaves the ADC disabled and the group restartable") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	REQUIRE(HAL_ADCEx_InjectedStop(&h) == HAL_OK);
	CHECK(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK(hostAdcInjectedRunning(1) == 1);
	CHECK(h.State == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStop does not stop the ADC while the regular group runs") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axRegChannel(h, 1, 1);
	axOneInjected(h, n);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	HAL_ADCEx_InjectedStop(&h);
	/* UM1725: with a conversion on the regular group HAL_ADC_Stop must be used to stop both groups and disable the ADC */
	CHECK((h.Instance->CR2 & axCR2_ADON) == axCR2_ADON);
	CHECK(hostAdcRegularRunning(n) == 1);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 characterization: InjectedStop refused by the regular group returns HAL_ERROR with ERROR_CONFIG and unlocks") {
	// NEEDS REVIEW 340/4
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axRegChannel(h, 1, 1);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	CHECK(HAL_ADCEx_InjectedStop(&h) == HAL_ERROR);
	CHECK((h.State & HAL_ADC_STATE_ERROR_CONFIG) == HAL_ADC_STATE_ERROR_CONFIG);
	CHECK((h.State & HAL_ADC_STATE_REG_BUSY) == HAL_ADC_STATE_REG_BUSY);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStop does not stop the ADC in auto-injection mode") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjected(h, n, true);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	HAL_ADCEx_InjectedStop(&h); /* UM1725: if auto-injection is enabled HAL_ADC_Stop must be used */
	CHECK((h.Instance->CR2 & axCR2_ADON) == axCR2_ADON);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 characterization: InjectedStop in auto-injection mode returns HAL_ERROR with ERROR_CONFIG") {
	// NEEDS REVIEW 340/4
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1, true);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	CHECK(HAL_ADCEx_InjectedStop(&h) == HAL_ERROR);
	CHECK((h.State & HAL_ADC_STATE_ERROR_CONFIG) == HAL_ADC_STATE_ERROR_CONFIG);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStop with a locked handle returns HAL_BUSY and leaves the ADC enabled") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_ADCEx_InjectedStop(&h) == HAL_BUSY);
	CHECK(HAL_ADCEx_InjectedStop_IT(&h) == HAL_BUSY);
	CHECK((h.Instance->CR2 & axCR2_ADON) == axCR2_ADON);
	CHECK(h.State == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStop_IT disables the JEOC interrupt and the ADC and sets READY") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjected(h, n);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	REQUIRE((h.Instance->CR1 & axCR1_JEOCIE) == axCR1_JEOCIE);
	CHECK(HAL_ADCEx_InjectedStop_IT(&h) == HAL_OK);
	CHECK((h.Instance->CR1 & axCR1_JEOCIE) == 0u);
	CHECK((h.Instance->CR2 & axCR2_ADON) == 0u);
	CHECK(hostAdcInjectedRunning(n) == 0);
	CHECK(h.State == HAL_ADC_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStop_IT refused by the regular group keeps the interrupt and the ADC enabled") {
	// NEEDS REVIEW 340/4 for the return code and the state bit
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axRegChannel(h, 1, 1);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	CHECK(HAL_ADCEx_InjectedStop_IT(&h) == HAL_ERROR);
	CHECK((h.Instance->CR1 & axCR1_JEOCIE) == axCR1_JEOCIE);
	CHECK((h.Instance->CR2 & axCR2_ADON) == axCR2_ADON);
	CHECK((h.State & HAL_ADC_STATE_ERROR_CONFIG) == HAL_ADC_STATE_ERROR_CONFIG);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStop_IT refused in auto-injection mode keeps the interrupt and the ADC enabled") {
	// NEEDS REVIEW 340/4 for the return code and the state bit
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1, true);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	CHECK(HAL_ADCEx_InjectedStop_IT(&h) == HAL_ERROR);
	CHECK((h.Instance->CR1 & axCR1_JEOCIE) == axCR1_JEOCIE);
	CHECK((h.Instance->CR2 & axCR2_ADON) == axCR2_ADON);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 the interrupt handler after InjectedStart_IT runs the injected callback and ends in READY") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjected(h, n);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(n, 4) == 1u);
	REQUIRE(hostAdcIrqPending(n) != 0u);
	HAL_ADC_IRQHandler(&h);
	CHECK(adcCounts.injected == 1u);
	CHECK(adcCounts.last == &h);
	CHECK(adcCounts.conv == 0u);
	CHECK(h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_INJ_EOC));
	CHECK((h.Instance->SR & (axSR_JEOC | axSR_JSTRT)) == 0u);
	CHECK((h.Instance->CR1 & axCR1_JEOCIE) == 0u); /* no further injected conversion can occur */
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1) == 0x123u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 the interrupt handler with an external injected trigger keeps INJ_BUSY and the JEOC interrupt") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjectedExternal(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	REQUIRE(hostAdcExternalTrigger(1, 1) == 1);
	REQUIRE(hostAdcConvert(1, 4) == 1u);
	HAL_ADC_IRQHandler(&h);
	CHECK(adcCounts.injected == 1u);
	/* UM1725 INJ_BUSY: a conversion 'can occur' by external trigger */
	CHECK((h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
	CHECK((h.State & HAL_ADC_STATE_INJ_EOC) == HAL_ADC_STATE_INJ_EOC);
	CHECK((h.Instance->CR1 & axCR1_JEOCIE) == axCR1_JEOCIE);
	CHECK((h.Instance->SR & axSR_JEOC) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 the interrupt handler does not run the injected callback when the JEOC interrupt is not enabled") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK); /* polling start: JEOCIE = 0 */
	REQUIRE(hostAdcConvert(1, 4) == 1u);
	HAL_ADC_IRQHandler(&h);
	CHECK(adcCounts.injected == 0u);
	CHECK((h.Instance->SR & axSR_JEOC) == axSR_JEOC);
}

/* ======================================================================================= AX3 poll / values */

TEST_CASE_FIXTURE(AdcFixture, "AX3 InjectedPollForConversion returns HAL_OK once the injected conversion is complete") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjected(h, n);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	hostAdcSetPollReads(n, 1);
	CHECK(HAL_ADCEx_InjectedPollForConversion(&h, 10) == HAL_OK);
	CHECK(hostAdcConversions(n) == 1u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 InjectedPollForConversion clears JEOC and JSTRT and sets INJ_EOC") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	hostAdcSetPollReads(1, 1);
	REQUIRE(HAL_ADCEx_InjectedPollForConversion(&h, 10) == HAL_OK);
	CHECK((h.Instance->SR & (axSR_JEOC | axSR_JSTRT)) == 0u);
	CHECK((h.State & HAL_ADC_STATE_INJ_EOC) == HAL_ADC_STATE_INJ_EOC);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 after a software started single injected conversion the state is READY with INJ_EOC") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjected(h, n);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	hostAdcSetPollReads(n, 1);
	REQUIRE(HAL_ADCEx_InjectedPollForConversion(&h, 10) == HAL_OK);
	CHECK(h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_INJ_EOC));
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 a three conversion injected sequence is complete only after the third conversion") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	h.Init.ScanConvMode = ENABLE;
	axInit(h);
	const u32 analog[3] = {0x111, 0x222, 0x333};
	axSequence(h, n, 3, analog, nullptr);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(n, 2) == 2u);
	CHECK((h.Instance->SR & axSR_JEOC) == 0u); /* RM0090 13.13.1: JEOC only at the end of all injected channels */
	REQUIRE(hostAdcConvert(n, 1) == 1u);
	CHECK(HAL_ADCEx_InjectedPollForConversion(&h, 0) == HAL_OK);
	CHECK(h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_INJ_EOC)); /* software start, EOCS = 0 */
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 characterization: a software started injected sequence of three conversions with EOCS set keeps INJ_BUSY") {
	// NEEDS REVIEW 340/12 (UM1725 EOCSelection: the injected group should not be used with polling / interrupt when EOC is selected after each conversion)
	ADC_HandleTypeDef h = axHandle(1);
	h.Init.ScanConvMode = ENABLE;
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV; /* EOCS = 1 */
	axInit(h);
	const u32 analog[3] = {0x111, 0x222, 0x333};
	axSequence(h, 1, 3, analog, nullptr);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 3) == 3u);
	REQUIRE(HAL_ADCEx_InjectedPollForConversion(&h, 0) == HAL_OK);
	CHECK((h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
	CHECK((h.State & HAL_ADC_STATE_READY) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 one injected conversion with EOCS set ends in READY after the poll") {
	ADC_HandleTypeDef h = axHandle(1);
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	REQUIRE(HAL_ADCEx_InjectedPollForConversion(&h, 0) == HAL_OK);
	CHECK(h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_INJ_EOC));
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 one injected conversion with EOCS set ends in READY with the JEOC interrupt disabled after the IRQ handler") {
	ADC_HandleTypeDef h = axHandle(1);
	h.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	HAL_ADC_IRQHandler(&h);
	CHECK(adcCounts.injected == 1u);
	CHECK(h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_INJ_EOC));
	CHECK((h.Instance->CR1 & axCR1_JEOCIE) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 with an external injected trigger the state keeps INJ_BUSY after a conversion") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjectedExternal(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcExternalTrigger(1, 1) == 1);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	REQUIRE(HAL_ADCEx_InjectedPollForConversion(&h, 0) == HAL_OK);
	/* UM1725 INJ_BUSY: a conversion 'can occur' by external trigger */
	CHECK(h.State == (HAL_ADC_STATE_INJ_BUSY | HAL_ADC_STATE_INJ_EOC));
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 characterization: with auto-injection the state keeps INJ_BUSY after the injected conversion") {
	// NEEDS REVIEW 340/5
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axRegChannel(h, 1, 1);
	axOneInjected(h, 1, true);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 2) == 2u);
	REQUIRE(HAL_ADCEx_InjectedPollForConversion(&h, 0) == HAL_OK);
	CHECK((h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
	CHECK((h.State & HAL_ADC_STATE_INJ_EOC) == HAL_ADC_STATE_INJ_EOC);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 characterization: with a continuous regular group the state keeps INJ_BUSY after the injected conversion") {
	// NEEDS REVIEW 340/5
	ADC_HandleTypeDef h = axHandle(1);
	h.Init.ContinuousConvMode = ENABLE;
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	REQUIRE(HAL_ADCEx_InjectedPollForConversion(&h, 0) == HAL_OK);
	CHECK((h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 with the regular group busy the injected completion does not set READY") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axRegChannel(h, 1, 1);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADC_Start(&h) == HAL_OK);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	REQUIRE(HAL_ADCEx_InjectedPollForConversion(&h, 0) == HAL_OK);
	CHECK((h.State & HAL_ADC_STATE_READY) == 0u);
	CHECK((h.State & HAL_ADC_STATE_REG_BUSY) == HAL_ADC_STATE_REG_BUSY);
	CHECK((h.State & HAL_ADC_STATE_INJ_BUSY) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 InjectedPollForConversion waits for HAL_MAX_DELAY until the conversion appears") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	hostAdcSetPollReads(1, 5); /* the 5th status read sees the conversion */
	CHECK(HAL_ADCEx_InjectedPollForConversion(&h, HAL_MAX_DELAY) == HAL_OK);
	CHECK(hostAdcConversions(1) == 1u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 InjectedPollForConversion times out after more than the timeout and sets the state TIMEOUT") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjected(h, n);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK); /* the conversion never completes: nobody calls hostAdcConvert */
	const u32 t0 = HAL_GetTick();
	CHECK(HAL_ADCEx_InjectedPollForConversion(&h, 5) == HAL_TIMEOUT);
	CHECK((HAL_GetTick() - t0) > 5u);
	CHECK(h.State == HAL_ADC_STATE_TIMEOUT);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 InjectedPollForConversion with the timeout 0 returns HAL_TIMEOUT at once when JEOC is not set") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	const u32 t0 = HAL_GetTick();
	CHECK(HAL_ADCEx_InjectedPollForConversion(&h, 0) == HAL_TIMEOUT);
	CHECK((HAL_GetTick() - t0) <= 3u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 InjectedPollForConversion with the timeout 0 returns HAL_OK when JEOC is already set") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	CHECK(HAL_ADCEx_InjectedPollForConversion(&h, 0) == HAL_OK);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 characterization: a timed out InjectedPollForConversion unlocks the handle") {
	// NEEDS REVIEW 340/7
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	h.Lock = HAL_LOCKED;
	REQUIRE(HAL_ADCEx_InjectedPollForConversion(&h, 1) == HAL_TIMEOUT);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 the injected result of each rank 1 to 4 is its channel value, right aligned, 12 bit") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	h.Init.ScanConvMode = ENABLE;
	axInit(h);
	const u32 analog[4] = {0x111, 0x222, 0x333, 0x444};
	for (u32 count = 1; count <= 4; ++count) {
		CAPTURE(count);
		axSequence(h, n, count, analog, nullptr);
		REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
		REQUIRE(hostAdcConvert(n, count) == count);
		for (u32 r = 1; r <= count; ++r) {
			CAPTURE(r);
			CHECK(HAL_ADCEx_InjectedGetValue(&h, r) == analog[r - 1]);
		}
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 the offset of each rank is subtracted from its result") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	h.Init.ScanConvMode = ENABLE;
	axInit(h);
	const u32 analog[4] = {0x111, 0x222, 0x333, 0x444};
	const u32 off[4] = {0x011, 0x022, 0x000, 0x100};
	axSequence(h, n, 4, analog, off);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(n, 4) == 4u);
	/* hand derivation: 0x111 - 0x011 = 0x100, 0x222 - 0x022 = 0x200, 0x333 - 0, 0x444 - 0x100 = 0x344 */
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1) == 0x100u);
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_2) == 0x200u);
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_3) == 0x333u);
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_4) == 0x344u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 a result below the offset is negative and sign extended to 16 bits") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	const u32 analog[1] = {10};
	const u32 off[1] = {25};
	axSequence(h, 1, 1, analog, off);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	/* 10 - 25 = -15 = 0xFFF1 in 16 bit two's complement (RM0090 13.4: SEXT) */
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1) == 0xFFF1u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 a left aligned 12 bit injected result is shifted up by 3 bits (SEXT at bit 15, MSB at bit 14)") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	h.Init.DataAlign = ADC_DATAALIGN_LEFT;
	axInit(h);
	const u32 analog[1] = {0x123};
	axSequence(h, n, 1, analog, nullptr);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(n, 1) == 1u);
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1) == 0x0918u); /* RM0090 13.4 Figure 49: SEXT at bit 15, D11 at bit 14: 0x123 << 3 */
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 the right aligned injected result has the selected resolution") {
	const u32 res[4] = {ADC_RESOLUTION_12B, ADC_RESOLUTION_10B, ADC_RESOLUTION_8B, ADC_RESOLUTION_6B};
	/* hand derivation of 0x2A8 (680) at 12 / 10 / 8 / 6 bit: 0x2A8, 0x2A8 >> 2 = 0xAA, >> 4 = 0x2A, >> 6 = 0xA */
	const u32 expected[4] = {0x2A8, 0xAA, 0x2A, 0xA};
	for (u32 i = 0; i < 4; ++i) {
		CAPTURE(i);
		ADC_HandleTypeDef h = axHandle(1);
		h.Init.Resolution = res[i];
		axInit(h);
		const u32 analog[1] = {0x2A8};
		axSequence(h, 1, 1, analog, nullptr);
		REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
		REQUIRE(hostAdcConvert(1, 1) == 1u);
		CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1) == expected[i]);
		REQUIRE(HAL_ADCEx_InjectedStop(&h) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 the offset is subtracted at the resolution of the conversion (10 bit)") {
	ADC_HandleTypeDef h = axHandle(1);
	h.Init.Resolution = ADC_RESOLUTION_10B;
	axInit(h);
	const u32 analog[1] = {0x800}; /* 10 bit: 0x200 */
	const u32 off[1] = {0x20};
	axSequence(h, 1, 1, analog, off);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1) == 0x1E0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 a left aligned injected result at 10, 8 and 6 bit follows the shifts of RM0090 Figures 49 and 50") {
	const u32 res[3] = {ADC_RESOLUTION_10B, ADC_RESOLUTION_8B, ADC_RESOLUTION_6B};
	/* 0x2A8 reduced to 10 / 8 / 6 bit is 0xAA / 0x2A / 0xA; shifts 5 / 7 (MSB at bit 14) and 1 (6 bit data on a byte basis: D5 at bit 6) */
	const u32 expected[3] = {0xAAu << 5, 0x2Au << 7, 0xAu << 1};
	for (u32 i = 0; i < 3; ++i) {
		CAPTURE(i);
		ADC_HandleTypeDef h = axHandle(1);
		h.Init.Resolution = res[i];
		h.Init.DataAlign = ADC_DATAALIGN_LEFT;
		axInit(h);
		const u32 analog[1] = {0x2A8};
		axSequence(h, 1, 1, analog, nullptr);
		REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
		REQUIRE(hostAdcConvert(1, 1) == 1u);
		CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1) == expected[i]);
		REQUIRE(HAL_ADCEx_InjectedStop(&h) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 InjectedGetValue clears the JEOC flag") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axOneInjected(h, n);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(n, 1) == 1u);
	REQUIRE((h.Instance->SR & axSR_JEOC) == axSR_JEOC);
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1) == 0x123u);
	CHECK((h.Instance->SR & axSR_JEOC) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 characterization: InjectedGetValue clears JEOC only and an invalid rank returns 0") {
	// NEEDS REVIEW 340/6
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	HalFixture::poke(&h.Instance->SR, axSR_JEOC | axSR_JSTRT);
	CHECK(HAL_ADCEx_InjectedGetValue(&h, 0) == 0u);
	CHECK((h.Instance->SR & axSR_JEOC) == 0u);
	CHECK((h.Instance->SR & axSR_JSTRT) == axSR_JSTRT);
	CHECK(HAL_ADCEx_InjectedGetValue(&h, 5) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 InjectedGetValue does not change the state") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 1) == 1u);
	const u32 before = h.State;
	HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1);
	CHECK(h.State == before);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 the injected data registers keep the values of the last sequence") {
	ADC_HandleTypeDef h = axHandle(1);
	h.Init.ScanConvMode = ENABLE;
	axInit(h);
	const u32 a1[2] = {0x100, 0x200};
	axSequence(h, 1, 2, a1, nullptr);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 2) == 2u);
	const u32 first1 = HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1);
	const u32 first2 = HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_2);
	CHECK(first1 == 0x100u);
	CHECK(first2 == 0x200u);
	hostAdcSetAnalog(1, axChannelOf(1), 0x300);
	hostAdcSetAnalog(1, axChannelOf(2), 0x400);
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1) == 0x100u); /* no new conversion yet */
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	REQUIRE(hostAdcConvert(1, 2) == 2u);
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_1) == 0x300u);
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_2) == 0x400u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX3 the injected discontinuous mode converts one rank per trigger and JEOC follows the last") {
	ADC_HandleTypeDef h = axHandle(1);
	h.Init.ScanConvMode = ENABLE;
	axInit(h);
	const u32 analog[3] = {0x111, 0x222, 0x333};
	axSequence(h, 1, 3, analog, nullptr);
	ADC_InjectionConfTypeDef j = axInj(axChannelOf(3), 3, 3);
	j.InjectedDiscontinuousConvMode = ENABLE;
	j.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
	j.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
	axInjCfg(h, j);
	REQUIRE(HAL_ADCEx_InjectedStart(&h) == HAL_OK);
	for (u32 t = 1; t <= 3; ++t) { /* RM0090 injected discontinuous mode example: 1st trigger channel 1, 2nd channel 2, 3rd channel 3 and JEOC */
		CAPTURE(t);
		REQUIRE(hostAdcExternalTrigger(1, 1) == 1);
		REQUIRE(hostAdcConvert(1, 4) == 1u);
		CHECK(((h.Instance->SR & axSR_JEOC) != 0u) == (t == 3u));
	}
	CHECK(HAL_ADCEx_InjectedGetValue(&h, ADC_INJECTED_RANK_3) == 0x333u);
}

/* ======================================================================================= AX4 multi mode */

TEST_CASE_FIXTURE(AdcFixture, "AX4 every multi ADC mode code reaches CCR MULTI") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	const u32 mode[13] = {ADC_MODE_INDEPENDENT,           ADC_DUALMODE_REGSIMULT_INJECSIMULT, ADC_DUALMODE_REGSIMULT_ALTERTRIG,
	                      ADC_DUALMODE_INJECSIMULT,       ADC_DUALMODE_REGSIMULT,             ADC_DUALMODE_INTERL,
	                      ADC_DUALMODE_ALTERTRIG,         ADC_TRIPLEMODE_REGSIMULT_INJECSIMULT, ADC_TRIPLEMODE_REGSIMULT_AlterTrig,
	                      ADC_TRIPLEMODE_INJECSIMULT,     ADC_TRIPLEMODE_REGSIMULT,           ADC_TRIPLEMODE_INTERL,
	                      ADC_TRIPLEMODE_ALTERTRIG};
	/* RM0090 13.13.16 MULTI[4:0] */
	const u32 code[13] = {0x00, 0x01, 0x02, 0x05, 0x06, 0x07, 0x09, 0x11, 0x12, 0x15, 0x16, 0x17, 0x19};
	for (u32 i = 0; i < 13; ++i) {
		CAPTURE(i);
		axMultiMode(h, mode[i]);
		CHECK((axCCR() & axCCR_MULTI) == code[i]);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 every DMA access mode reaches CCR DMA") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	const u32 mode[4] = {ADC_DMAACCESSMODE_DISABLED, ADC_DMAACCESSMODE_1, ADC_DMAACCESSMODE_2, ADC_DMAACCESSMODE_3};
	for (u32 i = 0; i < 4; ++i) {
		CAPTURE(i);
		axMultiMode(h, ADC_MODE_INDEPENDENT, mode[i]);
		CHECK((axCCR() & axCCR_DMA) == (i << 14));
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 every sampling delay from 5 to 20 cycles reaches CCR DELAY as the cycles minus 5") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	const u32 delay[16] = {ADC_TWOSAMPLINGDELAY_5CYCLES,  ADC_TWOSAMPLINGDELAY_6CYCLES,  ADC_TWOSAMPLINGDELAY_7CYCLES,  ADC_TWOSAMPLINGDELAY_8CYCLES,
	                       ADC_TWOSAMPLINGDELAY_9CYCLES,  ADC_TWOSAMPLINGDELAY_10CYCLES, ADC_TWOSAMPLINGDELAY_11CYCLES, ADC_TWOSAMPLINGDELAY_12CYCLES,
	                       ADC_TWOSAMPLINGDELAY_13CYCLES, ADC_TWOSAMPLINGDELAY_14CYCLES, ADC_TWOSAMPLINGDELAY_15CYCLES, ADC_TWOSAMPLINGDELAY_16CYCLES,
	                       ADC_TWOSAMPLINGDELAY_17CYCLES, ADC_TWOSAMPLINGDELAY_18CYCLES, ADC_TWOSAMPLINGDELAY_19CYCLES, ADC_TWOSAMPLINGDELAY_20CYCLES};
	for (u32 i = 0; i < 16; ++i) {
		CAPTURE(i);
		axMultiMode(h, ADC_MODE_INDEPENDENT, ADC_DMAACCESSMODE_DISABLED, delay[i]);
		CHECK((axCCR() & axCCR_DELAY) == (i << 8)); /* RM0090: 0000 = 5 TADCCLK ... 1111 = 20 TADCCLK */
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 a new multi mode configuration replaces the old fields completely") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	axMultiMode(h, ADC_TRIPLEMODE_ALTERTRIG, ADC_DMAACCESSMODE_3, ADC_TWOSAMPLINGDELAY_20CYCLES);
	REQUIRE((axCCR() & (axCCR_MULTI | axCCR_DMA | axCCR_DELAY)) == (0x19u | (3u << 14) | (0xFu << 8)));
	axMultiMode(h, ADC_MODE_INDEPENDENT);
	CHECK((axCCR() & (axCCR_MULTI | axCCR_DMA | axCCR_DELAY)) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 the multi mode configuration leaves ADCPRE, DDS, VBATE and TSVREFE alone") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	const u32 other = axCCR_ADCPRE | axCCR_DDS | axCCR_VBATE | axCCR_TSVREFE;
	HalFixture::poke(&ADC123_COMMON->CCR, other);
	axMultiMode(h, ADC_DUALMODE_INTERL, ADC_DMAACCESSMODE_2, ADC_TWOSAMPLINGDELAY_9CYCLES);
	CHECK((axCCR() & other) == other);
	CHECK((axCCR() & (axCCR_MULTI | axCCR_DMA | axCCR_DELAY)) == (0x07u | (2u << 14) | (4u << 8)));
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 the multi mode configuration is one common register whichever ADC handle is used") {
	ADC_HandleTypeDef h1 = axHandle(1), h2 = axHandle(2), h3 = axHandle(3);
	axInit(h1);
	axInit(h2);
	axInit(h3);
	axMultiMode(h3, ADC_TRIPLEMODE_REGSIMULT);
	CHECK((axCCR() & axCCR_MULTI) == 0x16u);
	axMultiMode(h2, ADC_DUALMODE_INTERL);
	CHECK((axCCR() & axCCR_MULTI) == 0x07u);
	axMultiMode(h1, ADC_MODE_INDEPENDENT);
	CHECK((axCCR() & axCCR_MULTI) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 the multi mode configuration with a locked handle returns HAL_BUSY and writes nothing") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	const u32 before = axCCR();
	h.Lock = HAL_LOCKED;
	ADC_MultiModeTypeDef m{};
	m.Mode = ADC_TRIPLEMODE_INTERL;
	m.DMAAccessMode = ADC_DMAACCESSMODE_1;
	m.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_12CYCLES;
	CHECK(HAL_ADCEx_MultiModeConfigChannel(&h, &m) == HAL_BUSY);
	CHECK(axCCR() == before);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 the multi mode configuration unlocks the handle") {
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axMultiMode(h, ADC_DUALMODE_ALTERTRIG);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeGetValue returns the common data register CDR") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	const u32 values[4] = {0x00000000u, 0x12345678u, 0x80000001u, 0xFFFFFFFFu};
	for (u32 v : values) {
		CAPTURE(v);
		hostAdcSetCommonData(v);
		CHECK(HAL_ADCEx_MultiModeGetValue(&h) == v);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA enables the ADC") {
	AX_ADC_N(n);
	AxMulti r(n);
	REQUIRE((r.h.Instance->CR2 & axCR2_ADON) == 0u);
	CHECK(HAL_ADCEx_MultiModeStart_DMA(&r.h, r.buf, 4) == HAL_OK);
	CHECK((r.h.Instance->CR2 & axCR2_ADON) == axCR2_ADON);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA programs the DMA stream with ADC_CDR, the buffer and the length") {
	AX_ADC_N(n);
	const u32 lengths[3] = {1, 2, 7};
	for (u32 length : lengths) {
		CAPTURE(length);
		AxMulti r(n);
		r.start(length);
		CHECK(r.m.inst->NDTR == length);
		CHECK(r.m.inst->PAR == axCdrAddress);
		CHECK(r.m.inst->M0AR == r.bufAddr());
		CHECK((r.m.inst->CR & axDMA_EN) == axDMA_EN);
		CHECK(r.d.State == HAL_DMA_STATE_BUSY);
		REQUIRE(HAL_ADCEx_MultiModeStop_DMA(&r.h) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA enables the transfer complete, half transfer and transfer error interrupts of the stream") {
	AX_ADC_N(n);
	AxMulti r(n);
	r.start();
	const u32 want = axDMA_TCIE | axDMA_HTIE | axDMA_TEIE;
	CHECK((r.m.inst->CR & want) == want);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA installs the three DMA callbacks") {
	AxMulti r(1);
	REQUIRE(r.d.XferCpltCallback == nullptr);
	r.start();
	CHECK(r.d.XferCpltCallback != nullptr);
	CHECK(r.d.XferHalfCpltCallback != nullptr);
	CHECK(r.d.XferErrorCallback != nullptr);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA sets DDS with DMAContinuousRequests enabled") {
	AX_ADC_N(n);
	AxMulti r(n, false, axTweakDmaContinuous);
	r.start();
	CHECK((axCCR() & axCCR_DDS) == axCCR_DDS);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA clears DDS with DMAContinuousRequests disabled") {
	AX_ADC_N(n);
	AxMulti r(n);
	HalFixture::poke(&ADC123_COMMON->CCR, axCCR_DDS);
	r.start();
	CHECK((axCCR() & axCCR_DDS) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA keeps the multi mode, DMA access mode, delay and ADCPRE fields") {
	AxMulti r(1);
	axMultiMode(r.h, ADC_TRIPLEMODE_REGSIMULT, ADC_DMAACCESSMODE_1, ADC_TWOSAMPLINGDELAY_11CYCLES);
	const u32 keep = axCCR_MULTI | axCCR_DMA | axCCR_DELAY | axCCR_ADCPRE;
	const u32 before = axCCR() & keep;
	r.start();
	CHECK((axCCR() & keep) == before);
	CHECK(before == (0x16u | (1u << 14) | (6u << 8) | (1u << 16)));
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA with the software trigger starts the regular group") {
	AX_ADC_N(n);
	AxMulti r(n);
	r.start();
	CHECK(hostAdcRegularRunning(n) == 1);
	CHECK((r.h.Instance->CR2 & (1u << 30)) == 0u); /* SWSTART reads 0 */
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA with an external regular trigger does not software-start") {
	AxMulti r(1, false, axTweakExternal);
	r.start();
	CHECK((r.h.Instance->CR2 & axCR2_ADON) == axCR2_ADON);
	CHECK(hostAdcRegularRunning(1) == 0);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA from READY leaves exactly REG_BUSY") {
	AX_ADC_N(n);
	AxMulti r(n);
	REQUIRE(r.h.State == HAL_ADC_STATE_READY);
	r.start();
	CHECK(r.h.State == HAL_ADC_STATE_REG_BUSY);
	CHECK(r.h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA clears the regular EOC and OVR state bits") {
	AxMulti r(1);
	r.h.State |= HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR;
	r.start();
	CHECK((r.h.State & (HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR)) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA with auto-injection also sets INJ_BUSY") {
	AxMulti r(1);
	axOneInjected(r.h, 1, true);
	r.start();
	CHECK(r.h.State == (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY));
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 characterization: MultiModeStart_DMA clears the error code, only the regular codes while the injected group is busy") {
	// NEEDS REVIEW 340/3 / NEEDS REVIEW 340/9
	AxMulti r(1);
	r.h.ErrorCode = HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA | HAL_ADC_ERROR_INTERNAL;
	r.start();
	CHECK(r.h.ErrorCode == HAL_ADC_ERROR_NONE);
	HAL_ADCEx_MultiModeStop_DMA(&r.h);
	r.h.State |= HAL_ADC_STATE_INJ_BUSY;
	r.h.ErrorCode = HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA | HAL_ADC_ERROR_INTERNAL;
	r.start();
	CHECK(r.h.ErrorCode == HAL_ADC_ERROR_INTERNAL);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 characterization: MultiModeStart_DMA enables the overrun interrupt and clears a stale EOC") {
	// NEEDS REVIEW 340/9
	AxMulti r(1);
	HalFixture::poke(&r.h.Instance->SR, axSR_EOC);
	r.start();
	CHECK((r.h.Instance->CR1 & axCR1_OVRIE) == axCR1_OVRIE);
	CHECK((r.h.Instance->SR & axSR_EOC) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 characterization: MultiModeStart_DMA enables only the given ADC, not the slaves") {
	// NEEDS REVIEW 340/9
	AxMulti r(1);
	r.start();
	CHECK((ADC1->CR2 & axCR2_ADON) == axCR2_ADON);
	CHECK((ADC2->CR2 & axCR2_ADON) == 0u);
	CHECK((ADC3->CR2 & axCR2_ADON) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 characterization: MultiModeStart_DMA returns HAL_OK although the DMA stream is busy and cannot be started") {
	// NEEDS REVIEW 340/9
	AxMulti r(1);
	REQUIRE(HAL_DMA_Start_IT(&r.d, axCdrAddress, r.bufAddr(), 1) == HAL_OK);
	CHECK(HAL_ADCEx_MultiModeStart_DMA(&r.h, r.buf, 4) == HAL_OK);
	CHECK(r.m.inst->NDTR == 1u); /* the busy stream kept its old length */
	CHECK(r.h.State == HAL_ADC_STATE_REG_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA with a locked handle returns HAL_BUSY and starts nothing") {
	AxMulti r(1);
	r.h.Lock = HAL_LOCKED;
	CHECK(HAL_ADCEx_MultiModeStart_DMA(&r.h, r.buf, 4) == HAL_BUSY);
	CHECK((r.h.Instance->CR2 & axCR2_ADON) == 0u);
	CHECK((r.m.inst->CR & axDMA_EN) == 0u);
	CHECK(r.h.State == HAL_ADC_STATE_READY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStop_DMA disables the ADC, DDS, the DMA stream and the overrun interrupt and sets READY") {
	AX_ADC_N(n);
	AxMulti r(n, false, axTweakDmaContinuous);
	r.start();
	REQUIRE((axCCR() & axCCR_DDS) == axCCR_DDS);
	CHECK(HAL_ADCEx_MultiModeStop_DMA(&r.h) == HAL_OK);
	CHECK((r.h.Instance->CR2 & axCR2_ADON) == 0u);
	CHECK((axCCR() & axCCR_DDS) == 0u);
	CHECK((r.m.inst->CR & axDMA_EN) == 0u);
	CHECK((r.h.Instance->CR1 & axCR1_OVRIE) == 0u);
	CHECK(r.h.State == HAL_ADC_STATE_READY);
	CHECK(r.h.Lock == HAL_UNLOCKED);
	CHECK(r.d.State == HAL_DMA_STATE_READY);
	CHECK(hostAdcRegularRunning(n) == 0);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStop_DMA clears REG_BUSY and INJ_BUSY") {
	AxMulti r(1);
	axOneInjected(r.h, 1, true);
	r.start();
	REQUIRE(r.h.State == (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY));
	REQUIRE(HAL_ADCEx_MultiModeStop_DMA(&r.h) == HAL_OK);
	CHECK(r.h.State == HAL_ADC_STATE_READY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 the DMA stream can be started again after MultiModeStop_DMA") {
	AxMulti r(1);
	r.start(3);
	REQUIRE(HAL_ADCEx_MultiModeStop_DMA(&r.h) == HAL_OK);
	r.start(5);
	CHECK(r.m.inst->NDTR == 5u);
	CHECK(r.h.State == HAL_ADC_STATE_REG_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 characterization: MultiModeStop_DMA with an idle DMA stream returns HAL_ERROR but still disables the ADC") {
	// NEEDS REVIEW 340/10
	AxMulti r(1);
	HAL_ADCEx_MultiModeStart_DMA(&r.h, r.buf, 4);
	HAL_DMA_Abort(&r.d); /* the stream is idle now */
	CHECK(HAL_ADCEx_MultiModeStop_DMA(&r.h) == HAL_ERROR);
	CHECK((r.h.Instance->CR2 & axCR2_ADON) == 0u);
	CHECK(r.h.State == HAL_ADC_STATE_READY);
	CHECK(r.h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 characterization: MultiModeStop_DMA with a stream that never stops returns the abort timeout and sets READY") {
	// NEEDS REVIEW 340/10
	AxMulti r(1);
	r.start();
	hostDmaStickEnabled(1, r.m.stream, 1);
	CHECK(HAL_ADCEx_MultiModeStop_DMA(&r.h) == HAL_TIMEOUT);
	CHECK((r.h.Instance->CR2 & axCR2_ADON) == 0u);
	CHECK(r.h.State == HAL_ADC_STATE_READY);
	hostDmaStickEnabled(1, r.m.stream, 0);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStop_DMA with a locked handle returns HAL_BUSY and leaves everything running") {
	AxMulti r(1);
	r.start();
	r.h.Lock = HAL_LOCKED;
	CHECK(HAL_ADCEx_MultiModeStop_DMA(&r.h) == HAL_BUSY);
	CHECK((r.h.Instance->CR2 & axCR2_ADON) == axCR2_ADON);
	CHECK((r.m.inst->CR & axDMA_EN) == axDMA_EN);
	CHECK(r.h.State == HAL_ADC_STATE_REG_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 the DMA transfer complete runs HAL_ADC_ConvCpltCallback once with the handle") {
	AX_ADC_N(n);
	AxMulti r(n);
	r.start();
	r.complete();
	CHECK(adcCounts.conv == 1u);
	CHECK(adcCounts.last == &r.h);
	CHECK(adcCounts.convHalf == 0u);
	CHECK(adcCounts.error == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 the DMA transfer complete sets REG_EOC before the callback") {
	AxMulti r(1);
	r.start();
	r.complete();
	CHECK((adcCounts.stateAtLast & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC);
	CHECK((r.h.State & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 characterization: the DMA transfer complete of a software started single conversion ends the regular state with READY") {
	// NEEDS REVIEW 340/11
	AxMulti r(1);
	r.start();
	r.complete();
	CHECK(r.h.State == (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC));
	CHECK((r.h.Instance->CR1 & axCR1_EOCIE) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 the DMA transfer complete with a continuous regular group keeps REG_BUSY") {
	AxMulti r(1, false, axTweakContinuous);
	r.start();
	r.complete();
	CHECK(r.h.State == (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_REG_EOC)); /* UM1725 REG_BUSY: 'can occur by continuous mode' */
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 the DMA transfer complete with an external regular trigger keeps REG_BUSY") {
	AxMulti r(1, false, axTweakExternal);
	r.start();
	r.complete();
	CHECK(r.h.State == (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_REG_EOC)); /* UM1725 REG_BUSY: 'can occur by external trigger' */
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 characterization: the DMA transfer complete of an EOCS scan sequence keeps REG_BUSY") {
	// NEEDS REVIEW 340/11
	AxMulti r(1, false, axTweakScanEocs);
	r.start();
	r.complete();
	CHECK(r.h.State == (HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_REG_EOC));
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 characterization: the DMA transfer complete while the injected group is busy clears REG_BUSY without READY") {
	// NEEDS REVIEW 340/11
	AxMulti r(1);
	axOneInjected(r.h, 1, true);
	r.start();
	r.complete();
	CHECK(r.h.State == (HAL_ADC_STATE_INJ_BUSY | HAL_ADC_STATE_REG_EOC));
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 a circular DMA stream runs the complete callback on every transfer complete") {
	AxMulti r(1, true, axTweakContinuous);
	r.start();
	r.complete();
	r.complete();
	r.complete();
	CHECK(adcCounts.conv == 3u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 characterization: a DMA completion in an error state runs the error callback instead of the complete callback") {
	// NEEDS REVIEW 340/11
	AxMulti r(1);
	r.start();
	r.h.State |= HAL_ADC_STATE_ERROR_INTERNAL;
	r.complete();
	CHECK(adcCounts.conv == 0u);
	CHECK(adcCounts.error == 1u);
	CHECK(r.h.State == HAL_ADC_STATE_ERROR_DMA);
	CHECK((r.h.ErrorCode & HAL_ADC_ERROR_DMA) == HAL_ADC_ERROR_DMA);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 the DMA half transfer runs HAL_ADC_ConvHalfCpltCallback and not the complete callback") {
	AX_ADC_N(n);
	AxMulti r(n);
	r.start();
	r.raise(HOST_DMA_HTIF);
	CHECK(adcCounts.convHalf == 1u);
	CHECK(adcCounts.last == &r.h);
	CHECK(adcCounts.conv == 0u);
	CHECK(adcCounts.error == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 the DMA half transfer does not change the ADC state") {
	AxMulti r(1);
	r.start();
	const u32 before = r.h.State;
	r.raise(HOST_DMA_HTIF);
	CHECK(r.h.State == before);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 a DMA transfer error runs the error callback, sets ERROR_DMA and the DMA error code") {
	AX_ADC_N(n);
	AxMulti r(n);
	r.start();
	r.raise(HOST_DMA_TEIF);
	CHECK(adcCounts.error == 1u);
	CHECK(adcCounts.last == &r.h);
	CHECK(adcCounts.conv == 0u);
	CHECK(r.h.State == HAL_ADC_STATE_ERROR_DMA);
	CHECK((r.h.ErrorCode & HAL_ADC_ERROR_DMA) == HAL_ADC_ERROR_DMA);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 a DMA transfer error keeps the other error code bits") {
	AxMulti r(1);
	r.start();
	r.h.ErrorCode = HAL_ADC_ERROR_OVR;
	r.raise(HOST_DMA_TEIF);
	CHECK(r.h.ErrorCode == (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 the error callback sees the state ERROR_DMA and the DMA error code") {
	AxMulti r(1);
	r.start();
	r.raise(HOST_DMA_TEIF);
	CHECK(adcCounts.stateAtLast == HAL_ADC_STATE_ERROR_DMA);
	CHECK((adcCounts.errorAtLast & HAL_ADC_ERROR_DMA) == HAL_ADC_ERROR_DMA);
}

/* ======================================================================================= fix pass additions */

TEST_CASE_FIXTURE(AdcFixture, "AX1 a smaller offset replaces 0xFFF for each rank 1 to 4") {
	AX_ADC_N(n);
	ADC_HandleTypeDef h = axHandle(n);
	axInit(h);
	volatile u32 *const jofr[4] = {&h.Instance->JOFR1, &h.Instance->JOFR2, &h.Instance->JOFR3, &h.Instance->JOFR4};
	for (u32 r = 1; r <= 4; ++r) {
		CAPTURE(r);
		axInjCfg(h, axInj(2u + r, r, 4, 0xFFF));
		REQUIRE(*jofr[r - 1] == 0xFFFu);
		axInjCfg(h, axInj(2u + r, r, 4, 0x010u * r));
		CHECK(*jofr[r - 1] == 0x010u * r);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart and InjectedStart_IT software-start the injected group when only the regular group has an external trigger") {
	for (int it = 0; it < 2; ++it) {
		CAPTURE(it);
		ADC_HandleTypeDef h = axHandle(1);
		h.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
		h.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
		axInit(h);
		axOneInjected(h, 1);
		REQUIRE((h.Instance->CR2 & (3u << 28)) != 0u);
		REQUIRE((it ? HAL_ADCEx_InjectedStart_IT(&h) : HAL_ADCEx_InjectedStart(&h)) == HAL_OK);
		CHECK(hostAdcInjectedRunning(1) == 1);
		REQUIRE((it ? HAL_ADCEx_InjectedStop_IT(&h) : HAL_ADCEx_InjectedStop(&h)) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart and InjectedStart_IT software-start when JEXTSEL is not 0 but JEXTEN is 00") {
	for (int it = 0; it < 2; ++it) {
		CAPTURE(it);
		ADC_HandleTypeDef h = axHandle(1);
		axInit(h);
		ADC_InjectionConfTypeDef j = axInj(3, 1, 1);
		j.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T2_CC1;
		j.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
		axInjCfg(h, j);
		hostAdcSetAnalog(1, 3, 0x123);
		REQUIRE((h.Instance->CR2 & axCR2_JEXTSEL) != 0u);
		REQUIRE((it ? HAL_ADCEx_InjectedStart_IT(&h) : HAL_ADCEx_InjectedStart(&h)) == HAL_OK);
		CHECK(hostAdcInjectedRunning(1) == 1);
		REQUIRE((it ? HAL_ADCEx_InjectedStop_IT(&h) : HAL_ADCEx_InjectedStop(&h)) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStart and InjectedStart_IT of the ADC1 master with auto-injection in multi mode start no injected conversion") {
	for (int it = 0; it < 2; ++it) {
		CAPTURE(it);
		ADC_HandleTypeDef h = axHandle(1);
		axInit(h);
		axOneInjected(h, 1, true);
		axMultiMode(h, ADC_DUALMODE_INJECSIMULT);
		REQUIRE((it ? HAL_ADCEx_InjectedStart_IT(&h) : HAL_ADCEx_InjectedStart(&h)) == HAL_OK);
		CHECK((h.Instance->CR2 & axCR2_ADON) == axCR2_ADON);
		CHECK(hostAdcInjectedRunning(1) == 0);
		CHECK((h.Instance->SR & axSR_JSTRT) == 0u);
		axMultiMode(h, ADC_MODE_INDEPENDENT);
		HAL_ADC_Stop(&h);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 characterization: InjectedStart_IT keeps the error code while REG_BUSY is set and clears it otherwise") {
	// NEEDS REVIEW 340/3
	ADC_HandleTypeDef h = axHandle(1);
	axInit(h);
	axOneInjected(h, 1);
	h.State |= HAL_ADC_STATE_REG_BUSY;
	h.ErrorCode = HAL_ADC_ERROR_OVR;
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_ADC_ERROR_OVR);
	h.State = HAL_ADC_STATE_READY;
	h.ErrorCode = HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA;
	REQUIRE(HAL_ADCEx_InjectedStart_IT(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_ADC_ERROR_NONE);
}

TEST_CASE_FIXTURE(AdcFixture, "AX2 InjectedStop and InjectedStop_IT of an ADC with CONT, ALIGN, DDS, DMA and EOCS set reach READY without ERROR_INTERNAL") {
	for (int it = 0; it < 2; ++it) {
		CAPTURE(it);
		ADC_HandleTypeDef h = axHandle(1);
		axInit(h);
		axOneInjected(h, 1);
		REQUIRE((it ? HAL_ADCEx_InjectedStart_IT(&h) : HAL_ADCEx_InjectedStart(&h)) == HAL_OK);
		HalFixture::poke(&h.Instance->CR2, h.Instance->CR2 | axCR2_CONT | axCR2_ALIGN | (1u << 9) | axCR2_DMA | axCR2_EOCS);
		h.State = HAL_ADC_STATE_INJ_BUSY;
		REQUIRE((it ? HAL_ADCEx_InjectedStop_IT(&h) : HAL_ADCEx_InjectedStop(&h)) == HAL_OK);
		CHECK((h.Instance->CR2 & axCR2_ADON) == 0u);
		CHECK(h.State == HAL_ADC_STATE_READY);
	}
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStop_DMA of an ADC with CONT, ALIGN, DDS, DMA and EOCS set reaches READY without ERROR_INTERNAL") {
	AxMulti r(1);
	r.start();
	HalFixture::poke(&r.h.Instance->CR2, r.h.Instance->CR2 | axCR2_CONT | axCR2_ALIGN | (1u << 9) | axCR2_DMA | axCR2_EOCS);
	REQUIRE(HAL_ADCEx_MultiModeStop_DMA(&r.h) == HAL_OK);
	CHECK((r.h.Instance->CR2 & axCR2_ADON) == 0u);
	CHECK(r.h.State == HAL_ADC_STATE_READY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 a DMA completion with the state ERROR_DMA runs the error callback instead of the complete callback") {
	AxMulti r(1);
	r.start();
	r.h.State |= HAL_ADC_STATE_ERROR_DMA;
	r.complete();
	CHECK(adcCounts.conv == 0u);
	CHECK(adcCounts.error == 1u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 the DMA complete of a software started single conversion clears a set EOCIE") {
	// NEEDS REVIEW 340/11
	AxMulti r(1);
	r.start();
	HalFixture::poke(&r.h.Instance->CR1, r.h.Instance->CR1 | axCR1_EOCIE);
	r.complete();
	CHECK((r.h.Instance->CR1 & axCR1_EOCIE) == 0u);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA with auto-injection clears a previous INJ_EOC") {
	AxMulti r(1);
	axOneInjected(r.h, 1, true);
	r.h.State |= HAL_ADC_STATE_INJ_EOC;
	r.start();
	CHECK((r.h.State & HAL_ADC_STATE_INJ_EOC) == 0u);
	CHECK((r.h.State & HAL_ADC_STATE_INJ_BUSY) == HAL_ADC_STATE_INJ_BUSY);
}

TEST_CASE_FIXTURE(AdcFixture, "AX4 MultiModeStart_DMA software-starts when EXTEN is 00 although EXTSEL is not 0") {
	AxMulti r(1);
	HalFixture::poke(&r.h.Instance->CR2, 5u << 24); /* EXTSEL = 5, EXTEN = 00 */
	r.start();
	CHECK(hostAdcRegularRunning(1) == 1);
}
