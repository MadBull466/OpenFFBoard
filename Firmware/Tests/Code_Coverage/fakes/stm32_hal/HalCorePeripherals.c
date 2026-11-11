/*
 * HalCorePeripherals.c: the system control space model; see HalCorePeripherals.h (PM0214 Rev 10 chapter 4).
 */
#include "HalCorePeripherals.h"

#include "HalRegisterSpace.h"

#define SCS_BASE_ADDR 0xE000E000u

/* Addresses (PM0214 Tables 44, 45, 53, 55) */
#define A_ACTLR 0xE000E008u
#define A_STK_CTRL 0xE000E010u
#define A_STK_LOAD 0xE000E014u
#define A_STK_VAL 0xE000E018u
#define A_STK_CALIB 0xE000E01Cu
#define A_ISER 0xE000E100u
#define A_ICER 0xE000E180u
#define A_ISPR 0xE000E200u
#define A_ICPR 0xE000E280u
#define A_IABR 0xE000E300u
#define A_IPR 0xE000E400u
#define A_IPR_END 0xE000E4F0u
#define A_CPUID 0xE000ED00u
#define A_ICSR 0xE000ED04u
#define A_VTOR 0xE000ED08u
#define A_AIRCR 0xE000ED0Cu
#define A_SCR 0xE000ED10u
#define A_CCR 0xE000ED14u
#define A_SHPR1 0xE000ED18u
#define A_SHPR2 0xE000ED1Cu
#define A_SHPR3 0xE000ED20u
#define A_SHCSR 0xE000ED24u
#define A_CFSR 0xE000ED28u
#define A_HFSR 0xE000ED2Cu
#define A_MMFAR 0xE000ED34u
#define A_BFAR 0xE000ED38u
#define A_AFSR 0xE000ED3Cu
#define A_MPU_TYPER 0xE000ED90u
#define A_MPU_CTRL 0xE000ED94u
#define A_MPU_RNR 0xE000ED98u
#define A_MPU_RBAR 0xE000ED9Cu
#define A_MPU_RASR 0xE000EDA0u
#define A_MPU_ALIAS_END 0xE000EDBCu /* RBAR_A1 0xEDA4 ... RASR_A3 0xEDB8 */
#define A_STIR 0xE000EF00u

#define STK_COUNTFLAG (1u << 16)
#define MPU_REGIONS 8u
#define RASR_MASK 0x173FFF3Fu

/* Model state outside the bus words */
static uint32_t nvicEnable[3], nvicPending[3], nvicActive[3]; /* 82 interrupts: 3 words */
static uint32_t resetRequests;
static uint32_t icsrPend; /* NMIPENDSET 31, PENDSVSET 28, PENDSTSET 26 */
static uint32_t mpuRbar[MPU_REGIONS], mpuRasr[MPU_REGIONS];

static uint32_t peek(uintptr_t a) { return hostRegsPeek(a); }
static void poke(uintptr_t a, uint32_t v) { hostRegsPoke(a, v); }

/* Valid interrupt bits of NVIC word n (82 interrupts: words 0, 1 full, word 2 bits 0..17). */
static uint32_t irqMask(unsigned word) { return word < 2u ? 0xFFFFFFFFu : word == 2u ? 0x0003FFFFu : 0u; }

void hostCoreResetState(void) {
	for (unsigned i = 0; i < 3u; ++i) nvicEnable[i] = nvicPending[i] = nvicActive[i] = 0;
	resetRequests = 0;
	icsrPend = 0;
	for (unsigned r = 0; r < MPU_REGIONS; ++r) mpuRbar[r] = mpuRasr[r] = 0;
}

static void coreReset(void) { /* reset values of PM0214 Tables 44, 53, 55 that are not 0 */
	poke(A_CCR, 0x00000200u);
}

static int isNvicBank(uintptr_t a, uintptr_t base) { return a >= base && a < base + 0x20u; }

/* Is the address one of the listed registers? */
static int coreKnown(uintptr_t a) {
	if (a == A_ACTLR || (a >= A_STK_CTRL && a <= A_STK_CALIB)) return 1;
	if (isNvicBank(a, A_ISER) || isNvicBank(a, A_ICER) || isNvicBank(a, A_ISPR) || isNvicBank(a, A_ICPR) ||
		isNvicBank(a, A_IABR))
		return 1;
	if (a >= A_IPR && a < A_IPR_END) return 1;
	if ((a >= A_CPUID && a <= A_HFSR) || (a >= A_MMFAR && a <= A_AFSR)) return 1;
	if (a >= A_MPU_TYPER && a < A_MPU_ALIAS_END) return 1;
	return a == A_STIR;
}

static int coreBeforeRead(uintptr_t a) {
	if (!coreKnown(a) || a == A_STIR) return 0; /* STIR is write-only */
	if (isNvicBank(a, A_ISER) || isNvicBank(a, A_ICER)) poke(a, nvicEnable[(a & 0x1Fu) / 4u < 3u ? (a & 0x1Fu) / 4u : 0] & irqMask((unsigned)((a & 0x1Fu) / 4u)));
	else if (isNvicBank(a, A_ISPR) || isNvicBank(a, A_ICPR)) poke(a, nvicPending[(a & 0x1Fu) / 4u < 3u ? (a & 0x1Fu) / 4u : 0] & irqMask((unsigned)((a & 0x1Fu) / 4u)));
	else if (isNvicBank(a, A_IABR)) poke(a, nvicActive[(a & 0x1Fu) / 4u < 3u ? (a & 0x1Fu) / 4u : 0] & irqMask((unsigned)((a & 0x1Fu) / 4u)));
	else if (a == A_CPUID) poke(a, 0x410FC241u);
	else if (a == A_ICSR) poke(a, icsrPend);
	else if (a == A_AIRCR) poke(a, (0xFA05u << 16) | (peek(a) & 0x700u));
	else if (a == A_STK_CALIB) poke(a, 0xC0000000u);
	else if (a == A_MPU_TYPER) poke(a, 0x00000800u);
	else if (a == A_MPU_RBAR || a == 0xE000EDA4u || a == 0xE000EDACu || a == 0xE000EDB4u) {
		const uint32_t rnr = peek(A_MPU_RNR) & 0xFFu;
		poke(a, (rnr < MPU_REGIONS ? mpuRbar[rnr] : 0u) | (rnr & 0xFu));
	} else if (a == A_MPU_RASR || a == 0xE000EDA8u || a == 0xE000EDB0u || a == 0xE000EDB8u) {
		const uint32_t rnr = peek(A_MPU_RNR) & 0xFFu;
		poke(a, rnr < MPU_REGIONS ? mpuRasr[rnr] : 0u);
	}
	return 1;
}

static void coreAfterRead(uintptr_t a, uint32_t value) {
	if (a == A_STK_CTRL && (value & STK_COUNTFLAG)) poke(a, value & ~STK_COUNTFLAG); /* cleared by the read */
}

static int coreBeforeWrite(uintptr_t a) {
	if (!coreKnown(a)) return 0;
	if (a == A_CPUID || isNvicBank(a, A_IABR) || a == A_STK_CALIB || a == A_MPU_TYPER) return 0; /* read-only */
	return 1;
}

static void coreAfterWrite(uintptr_t a, uint32_t oldValue, uint32_t w) {
	const unsigned word = (unsigned)((a & 0x1Fu) / 4u);
	if (isNvicBank(a, A_ISER)) { if (word < 3u) nvicEnable[word] |= w & irqMask(word); poke(a, word < 3u ? nvicEnable[word] : 0u); return; }
	if (isNvicBank(a, A_ICER)) { if (word < 3u) nvicEnable[word] &= ~(w & irqMask(word)); poke(a, word < 3u ? nvicEnable[word] : 0u); return; }
	if (isNvicBank(a, A_ISPR)) { if (word < 3u) nvicPending[word] |= w & irqMask(word); poke(a, word < 3u ? nvicPending[word] : 0u); return; }
	if (isNvicBank(a, A_ICPR)) { if (word < 3u) nvicPending[word] &= ~(w & irqMask(word)); poke(a, word < 3u ? nvicPending[word] : 0u); return; }
	if (a >= A_IPR && a < A_IPR_END) {
		uint32_t v = w & 0xF0F0F0F0u; /* bits 3:0 of every field read 0 and ignore writes */
		const unsigned firstIrq = (unsigned)((a - A_IPR) * 1u); /* 4 fields per word: interrupts firstIrq..+3 */
		for (unsigned b = 0; b < 4u; ++b)
			if (firstIrq + b >= HOST_NVIC_IRQS) v &= ~(0xFFu << (8u * b)); /* interrupt does not exist */
		poke(a, v);
		return;
	}
	switch (a) {
	case A_ACTLR: case A_CCR: case A_SHCSR: case A_MMFAR: case A_BFAR: case A_AFSR: poke(a, w); return;
	case A_CFSR: case A_HFSR: poke(a, oldValue & ~w); return; /* write 1 to clear */
	case A_STK_CTRL: poke(a, (oldValue & STK_COUNTFLAG) | (w & 7u)); return;
	case A_STK_LOAD: poke(a, w & 0x00FFFFFFu); return;
	case A_STK_VAL: poke(a, 0); poke(A_STK_CTRL, peek(A_STK_CTRL) & ~STK_COUNTFLAG); return;
	case A_ICSR:
		if (w & (1u << 31)) icsrPend |= 1u << 31;
		if (w & (1u << 28)) icsrPend |= 1u << 28;
		if (w & (1u << 27)) icsrPend &= ~(1u << 28);
		if (w & (1u << 26)) icsrPend |= 1u << 26;
		if (w & (1u << 25)) icsrPend &= ~(1u << 26);
		poke(a, icsrPend);
		return;
	case A_VTOR: poke(a, w & 0x3FFFFE00u); return;
	case A_AIRCR:
		if ((w >> 16) != 0x05FAu) { poke(a, oldValue); return; } /* no key: write ignored */
		if (w & 3u) hostRegsAddFault("AIRCR write with VECTCLRACTIVE / VECTRESET set (unpredictable)", a, w);
		if (w & (1u << 2)) resetRequests++;
		poke(a, (0xFA05u << 16) | (w & 0x700u));
		return;
	case A_SCR: poke(a, w & 0x16u); return;
	case A_SHPR1: poke(a, w & 0x00F0F0F0u); return;
	case A_SHPR2: poke(a, w & 0xF0000000u); return;
	case A_SHPR3: poke(a, w & 0xF0F00000u); return;
	case A_MPU_CTRL: poke(a, w & 7u); return;
	case A_MPU_RNR: poke(a, w & 0xFFu); return;
	case A_STIR: {
		const unsigned irq = w & 0x1FFu;
		if (irq < HOST_NVIC_IRQS) nvicPending[irq / 32u] |= 1u << (irq % 32u);
		poke(a, 0);
		return;
	}
	default: break;
	}
	if (a == A_MPU_RBAR || a == 0xE000EDA4u || a == 0xE000EDACu || a == 0xE000EDB4u) {
		if (w & (1u << 4)) poke(A_MPU_RNR, w & 0xFu); /* VALID: REGION updates RNR */
		const uint32_t rnr = peek(A_MPU_RNR) & 0xFFu;
		if (rnr < MPU_REGIONS) mpuRbar[rnr] = w & 0xFFFFFFE0u;
		poke(a, (rnr < MPU_REGIONS ? mpuRbar[rnr] : 0u) | (rnr & 0xFu));
	} else if (a == A_MPU_RASR || a == 0xE000EDA8u || a == 0xE000EDB0u || a == 0xE000EDB8u) {
		const uint32_t rnr = peek(A_MPU_RNR) & 0xFFu;
		if (rnr < MPU_REGIONS) mpuRasr[rnr] = w & RASR_MASK;
		poke(a, rnr < MPU_REGIONS ? mpuRasr[rnr] : 0u);
	}
}

static const HostRegModel coreModel = {"system control space (PM0214)", SCS_BASE_ADDR, 0x1000u, coreReset,
									   coreBeforeRead, coreAfterRead, coreBeforeWrite, coreAfterWrite};

void hostCoreRegister(void) { hostRegsAddModel(&coreModel); }

int hostNvicEnabled(unsigned irq) { return irq < HOST_NVIC_IRQS && ((nvicEnable[irq / 32u] >> (irq % 32u)) & 1u); }
int hostNvicPending(unsigned irq) { return irq < HOST_NVIC_IRQS && ((nvicPending[irq / 32u] >> (irq % 32u)) & 1u); }
void hostNvicSetActive(unsigned irq, int active) {
	if (irq >= HOST_NVIC_IRQS) return;
	if (active) nvicActive[irq / 32u] |= 1u << (irq % 32u);
	else nvicActive[irq / 32u] &= ~(1u << (irq % 32u));
}
uint32_t hostScbResetRequests(void) { return resetRequests; }
void hostSysTickCountedToZero(void) { poke(A_STK_CTRL, peek(A_STK_CTRL) | STK_COUNTFLAG); }
