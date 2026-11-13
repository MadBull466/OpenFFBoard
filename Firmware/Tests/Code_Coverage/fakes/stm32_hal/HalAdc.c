/*
 * HalAdc.c: ADC1..ADC3 and common register model (P12 T6b). See HalAdc.h for what is modelled and the RM0090 sources.
 * Register access goes through hostRegsPeek / hostRegsPoke (raw, no trapping).
 */
#include "HalAdc.h"

#include <stddef.h>
#include <string.h>

#define R_SR 0x00u
#define R_CR1 0x04u
#define R_CR2 0x08u
#define R_SMPR1 0x0Cu
#define R_SMPR2 0x10u
#define R_JOFR1 0x14u
#define R_JOFR4 0x20u
#define R_HTR 0x24u
#define R_LTR 0x28u
#define R_SQR1 0x2Cu
#define R_SQR2 0x30u
#define R_SQR3 0x34u
#define R_JSQR 0x38u
#define R_JDR1 0x3Cu
#define R_JDR4 0x48u
#define R_DR 0x4Cu

#define C_CSR 0x00u
#define C_CCR 0x04u
#define C_CDR 0x08u
#define COMMON_BASE 0x40012300u

#define SR_AWD 1u
#define SR_EOC 2u
#define SR_JEOC 4u
#define SR_JSTRT 8u
#define SR_STRT 0x10u
#define SR_OVR 0x20u
#define CR1_SCAN (1u << 8)
#define CR1_AWDSGL (1u << 9)
#define CR1_JAUTO (1u << 10)
#define CR1_DISCEN (1u << 11)
#define CR1_JDISCEN (1u << 12)
#define CR1_JAWDEN (1u << 22)
#define CR1_AWDEN (1u << 23)
#define CR1_EOCIE (1u << 5)
#define CR1_AWDIE (1u << 6)
#define CR1_JEOCIE (1u << 7)
#define CR1_OVRIE (1u << 26)
#define CR2_ADON 1u
#define CR2_CONT 2u
#define CR2_DMA (1u << 8)
#define CR2_EOCS (1u << 10)
#define CR2_ALIGN (1u << 11)
#define CR2_JEXTEN (3u << 20)
#define CR2_JSWSTART (1u << 22)
#define CR2_EXTEN (3u << 28)
#define CR2_SWSTART (1u << 30)

static const uint32_t baseOf[HOST_ADC_COUNT] = {0x40012000u, 0x40012100u, 0x40012200u};

typedef struct {
	uint16_t analog[32];
	uint8_t regRunning, injRunning;
	uint8_t regPos, injPos, discCount;
	unsigned pollReads, pollCount;
	uint32_t dmaRequests, conversions;
} AdcState;
static AdcState as[HOST_ADC_COUNT];
static uint32_t commonData;

static uint32_t peek(uintptr_t a) { return hostRegsPeek(a); }
static void poke(uintptr_t a, uint32_t v) { hostRegsPoke(a, v); }
static uint32_t get(unsigned i, uint32_t r) { return peek(baseOf[i] + r); }
static void put(unsigned i, uint32_t r, uint32_t v) { poke(baseOf[i] + r, v); }
static void setBits(unsigned i, uint32_t r, uint32_t m) { put(i, r, get(i, r) | m); }
static void clrBits(unsigned i, uint32_t r, uint32_t m) { put(i, r, get(i, r) & ~m); }

void hostAdcResetState(void) {
	memset(as, 0, sizeof as);
	commonData = 0;
}
void hostAdcSetAnalog(unsigned adc, unsigned ch, uint16_t value12) {
	if (adc >= 1u && adc <= HOST_ADC_COUNT && ch < 32u) as[adc - 1u].analog[ch] = (uint16_t)(value12 & 0xFFFu);
}
void hostAdcSetPollReads(unsigned adc, unsigned n) {
	if (adc >= 1u && adc <= HOST_ADC_COUNT) {
		as[adc - 1u].pollReads = n;
		as[adc - 1u].pollCount = 0;
	}
}
uint32_t hostAdcDmaRequests(unsigned adc) { return (adc >= 1u && adc <= HOST_ADC_COUNT) ? as[adc - 1u].dmaRequests : 0u; }
void hostAdcClearDmaRequests(unsigned adc) {
	if (adc >= 1u && adc <= HOST_ADC_COUNT) as[adc - 1u].dmaRequests = 0;
}
uint32_t hostAdcConversions(unsigned adc) { return (adc >= 1u && adc <= HOST_ADC_COUNT) ? as[adc - 1u].conversions : 0u; }
void hostAdcSetCommonData(uint32_t cdr) { commonData = cdr; }
int hostAdcRegularRunning(unsigned adc) { return (adc >= 1u && adc <= HOST_ADC_COUNT) ? as[adc - 1u].regRunning : 0; }
int hostAdcInjectedRunning(unsigned adc) { return (adc >= 1u && adc <= HOST_ADC_COUNT) ? as[adc - 1u].injRunning : 0; }

uint32_t hostAdcIrqPending(unsigned adc) {
	if (adc < 1u || adc > HOST_ADC_COUNT) return 0;
	const unsigned i = adc - 1u;
	const uint32_t sr = get(i, R_SR), cr1 = get(i, R_CR1);
	return ((sr & SR_EOC) && (cr1 & CR1_EOCIE)) || ((sr & SR_JEOC) && (cr1 & CR1_JEOCIE)) || ((sr & SR_AWD) && (cr1 & CR1_AWDIE)) ||
		   ((sr & SR_OVR) && (cr1 & CR1_OVRIE));
}

/* ------------------------------------------------------------------ conversion engine */
static unsigned resBits(unsigned i) { return 12u - 2u * ((get(i, R_CR1) >> 24) & 3u); }

static unsigned regChannel(unsigned i, unsigned pos) { /* pos 0..15 = rank 1..16 */
	if (pos < 6u) return (get(i, R_SQR3) >> (5u * pos)) & 0x1Fu;
	if (pos < 12u) return (get(i, R_SQR2) >> (5u * (pos - 6u))) & 0x1Fu;
	return (get(i, R_SQR1) >> (5u * (pos - 12u))) & 0x1Fu;
}

static uint32_t alignValue(unsigned i, uint32_t raw) {
	const unsigned bits = resBits(i);
	/* RM0090 13.4 Figures 49 / 50: left alignment on a half-word basis (MSB at bit 15), on a byte basis (D5 at bit 7) for 6 bit data */
	return (get(i, R_CR2) & CR2_ALIGN) ? ((raw << (bits == 6u ? 2u : 16u - bits)) & 0xFFFFu) : (raw & 0xFFFFu);
}

static void awdCheck(unsigned i, unsigned ch, uint32_t raw, uint32_t enableBit) {
	const uint32_t cr1 = get(i, R_CR1);
	if (!(cr1 & enableBit)) return;
	if ((cr1 & CR1_AWDSGL) && ch != (cr1 & 0x1Fu)) return;
	const uint32_t v12 = raw << (12u - resBits(i));
	if (v12 > (get(i, R_HTR) & 0xFFFu) || v12 < (get(i, R_LTR) & 0xFFFu)) setBits(i, R_SR, SR_AWD);
}

static void startInjected(unsigned i) {
	as[i].injRunning = 1;
	as[i].injPos = 0;
	setBits(i, R_SR, SR_JSTRT);
}

static void injectedStep(unsigned i) {
	const uint32_t jsqr = get(i, R_JSQR);
	const unsigned jl = (jsqr >> 20) & 3u;
	const unsigned rank = 4u - jl + as[i].injPos; /* 1..4: JSQrank */
	const unsigned ch = (jsqr >> (5u * (rank - 1u))) & 0x1Fu;
	const unsigned bits = resBits(i);
	const uint32_t raw = (uint32_t)as[i].analog[ch & 31u] >> (12u - bits);
	const unsigned slot = as[i].injPos; /* JOFR / JDR number slot + 1: the conversion order (model decision, RM0090 13.13.13 is silent for JL < 3) */
	const uint32_t off = get(i, R_JOFR1 + 4u * slot) & 0xFFFu;
	const uint32_t res = (raw - off) & 0xFFFFu;
	/* left aligned injected data: SEXT at bit 15 and D(bits-1) at bit 14 (Figure 49), D5 at bit 6 for 6 bit data (Figure 50) */
	put(i, R_JDR1 + 4u * slot, (get(i, R_CR2) & CR2_ALIGN) ? ((res << (bits == 6u ? 1u : 15u - bits)) & 0xFFFFu) : res);
	awdCheck(i, ch, raw, CR1_JAWDEN);
	as[i].conversions++;
	as[i].injPos++;
	if (as[i].injPos > jl) {
		setBits(i, R_SR, SR_JEOC);
		as[i].injRunning = 0;
		as[i].injPos = 0;
	} else if (get(i, R_CR1) & CR1_JDISCEN) {
		as[i].injRunning = 0; /* one channel per trigger; the position is kept */
	}
}

static void regularStep(unsigned i) {
	const uint32_t cr1 = get(i, R_CR1), cr2 = get(i, R_CR2);
	const unsigned len = (cr1 & CR1_SCAN) ? ((get(i, R_SQR1) >> 20) & 0xFu) + 1u : 1u;
	const unsigned pos = as[i].regPos;
	const unsigned ch = regChannel(i, pos);
	const unsigned bits = resBits(i);
	const uint32_t raw = (uint32_t)as[i].analog[ch & 31u] >> (12u - bits);
	const int last = (pos + 1u >= len);
	const int detect = (cr2 & (CR2_DMA | CR2_EOCS)) != 0u;
	as[i].conversions++;
	if (detect && (get(i, R_SR) & SR_EOC)) { /* data lost: RM0090 13.8.1 */
		setBits(i, R_SR, SR_OVR);
		as[i].regRunning = 0;
		return;
	}
	put(i, R_DR, alignValue(i, raw));
	awdCheck(i, ch, raw, CR1_AWDEN);
	if (cr2 & CR2_DMA) as[i].dmaRequests++;
	as[i].discCount++;
	const int groupDone = (cr1 & CR1_DISCEN) && as[i].discCount > ((cr1 >> 13) & 7u);
	if ((cr2 & CR2_EOCS) || last || groupDone) setBits(i, R_SR, SR_EOC);
	if (last) {
		as[i].regPos = 0;
		as[i].discCount = 0;
		if (!(cr2 & CR2_CONT) || (cr1 & CR1_DISCEN)) as[i].regRunning = 0;
		if (cr1 & CR1_JAUTO) startInjected(i);
	} else {
		as[i].regPos = (uint8_t)(pos + 1u);
		if (groupDone) {
			as[i].regRunning = 0;
			as[i].discCount = 0;
		}
	}
}

unsigned hostAdcConvert(unsigned adc, unsigned n) {
	if (adc < 1u || adc > HOST_ADC_COUNT) return 0;
	const unsigned i = adc - 1u;
	unsigned done = 0;
	while (done < n && (get(i, R_CR2) & CR2_ADON) && (as[i].injRunning || as[i].regRunning)) {
		if (as[i].injRunning) injectedStep(i);
		else regularStep(i);
		++done;
	}
	return done;
}

static int launchRegular(unsigned i) {
	const uint32_t cr2 = get(i, R_CR2);
	if (!(cr2 & CR2_ADON) || as[i].regRunning) return 0;
	if ((cr2 & (CR2_DMA | CR2_EOCS)) && (get(i, R_SR) & SR_OVR)) return 0; /* triggers ignored until software clears OVR */
	as[i].regRunning = 1;
	as[i].discCount = 0;
	setBits(i, R_SR, SR_STRT);
	return 1;
}

static int launchInjected(unsigned i) {
	const uint32_t cr2 = get(i, R_CR2);
	if (!(cr2 & CR2_ADON)) return 0;
	if (as[i].injRunning) return 0;
	if (as[i].injPos == 0u || !(get(i, R_CR1) & CR1_JDISCEN)) as[i].injPos = 0;
	as[i].injRunning = 1;
	setBits(i, R_SR, SR_JSTRT);
	return 1;
}

int hostAdcExternalTrigger(unsigned adc, int injected) {
	if (adc < 1u || adc > HOST_ADC_COUNT) return 0;
	const unsigned i = adc - 1u;
	const uint32_t cr2 = get(i, R_CR2);
	if (injected) return (cr2 & CR2_JEXTEN) ? launchInjected(i) : 0;
	return (cr2 & CR2_EXTEN) ? launchRegular(i) : 0;
}

/* ------------------------------------------------------------------ registers */
static int known(unsigned i, uintptr_t a) {
	(void)i;
	const uint32_t off = (uint32_t)(a & 0xFFu);
	return (off & 3u) == 0u && off <= R_DR;
}

static int beforeRead(unsigned i, uintptr_t a) {
	if (!known(i, a)) return 0;
	if ((uint32_t)(a & 0xFFu) == R_SR && as[i].pollReads != 0u && (as[i].regRunning || as[i].injRunning)) {
		if (++as[i].pollCount >= as[i].pollReads) {
			as[i].pollCount = 0;
			hostAdcConvert(i + 1u, 1);
		}
	}
	return 1;
}

static void afterRead(unsigned i, uintptr_t a, uint32_t value) {
	(void)value;
	if ((uint32_t)(a & 0xFFu) == R_DR) clrBits(i, R_SR, SR_EOC); /* RM0090 13.13.1: EOC cleared by reading ADC_DR */
}

static void afterWrite(unsigned i, uintptr_t a, uint32_t oldValue, uint32_t written) {
	const uint32_t off = (uint32_t)(a & 0xFFu);
	switch (off) {
	case R_SR: poke(a, oldValue & (written | ~0x3Fu)); break;
	case R_CR1: poke(a, written & 0x07C0FFFFu); break;
	case R_CR2: {
		const uint32_t nw = written & 0x7F7F0F03u;
		poke(a, nw & ~(CR2_SWSTART | CR2_JSWSTART));
		if (!(nw & CR2_ADON)) {
			as[i].regRunning = 0;
			as[i].injRunning = 0;
			break;
		}
		if ((nw & CR2_SWSTART) && !(nw & CR2_EXTEN)) launchRegular(i);
		if ((nw & CR2_JSWSTART) && !(nw & CR2_JEXTEN)) launchInjected(i);
		break;
	}
	case R_SMPR1: poke(a, written & 0x07FFFFFFu); break;
	case R_SMPR2: poke(a, written & 0x3FFFFFFFu); break;
	case R_HTR: case R_LTR: poke(a, written & 0xFFFu); break;
	case R_SQR1: poke(a, written & 0x00FFFFFFu); break;
	case R_SQR2: case R_SQR3: poke(a, written & 0x3FFFFFFFu); break;
	case R_JSQR: poke(a, written & 0x003FFFFFu); break;
	default:
		if (off >= R_JOFR1 && off <= R_JOFR4) poke(a, written & 0xFFFu);
		else poke(a, oldValue); /* JDRx and DR are read only */
		break;
	}
}

static void resetRegs(unsigned i) { put(i, R_HTR, 0xFFFu); }

#define ADC_THUNKS(N)                                                                          \
	static int known##N(uintptr_t a) { return known(N, a); }                                   \
	static int beforeRead##N(uintptr_t a) { return beforeRead(N, a); }                         \
	static void afterRead##N(uintptr_t a, uint32_t v) { afterRead(N, a, v); }                  \
	static void afterWrite##N(uintptr_t a, uint32_t o, uint32_t w) { afterWrite(N, a, o, w); } \
	static void reset##N(void) { resetRegs(N); }
ADC_THUNKS(0) ADC_THUNKS(1) ADC_THUNKS(2)

#define ADC_MODEL(N, NAME) {NAME, 0, 0x100u, reset##N, beforeRead##N, afterRead##N, known##N, afterWrite##N}
static HostRegModel adcModels[HOST_ADC_COUNT] = {ADC_MODEL(0, "ADC1"), ADC_MODEL(1, "ADC2"), ADC_MODEL(2, "ADC3")};

/* ------------------------------------------------------------------ common registers */
static int commonKnown(uintptr_t a) {
	const uint32_t off = (uint32_t)(a & 0xFFu);
	return off == C_CSR || off == C_CCR || off == C_CDR;
}
static int commonBeforeRead(uintptr_t a) {
	if (!commonKnown(a)) return 0;
	const uint32_t off = (uint32_t)(a & 0xFFu);
	if (off == C_CSR) {
		uint32_t v = 0;
		for (unsigned i = 0; i < HOST_ADC_COUNT; ++i) v |= (get(i, R_SR) & 0x3Fu) << (8u * i);
		poke(a, v);
	} else if (off == C_CDR) {
		poke(a, commonData);
	}
	return 1;
}
static void commonAfterWrite(uintptr_t a, uint32_t oldValue, uint32_t written) {
	const uint32_t off = (uint32_t)(a & 0xFFu);
	poke(a, off == C_CCR ? (written & 0x00C3EF1Fu) : oldValue); /* CSR and CDR are read only */
}
static void commonReset(void) {}
static HostRegModel commonModel = {"ADC_COMMON", COMMON_BASE, 0x100u, commonReset, commonBeforeRead, NULL, commonKnown, commonAfterWrite};

void hostAdcRegister(void) {
	for (unsigned i = 0; i < HOST_ADC_COUNT; ++i) {
		adcModels[i].base = baseOf[i];
		hostRegsAddModel(&adcModels[i]);
	}
	hostRegsAddModel(&commonModel);
}
