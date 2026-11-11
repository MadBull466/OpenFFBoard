/*
 * HalClockPower.c: RCC and PWR behavioural models (P12 T2). See HalClockPower.h for what is modelled and the RM0090 sources.
 * Register access goes through hostRegsPeek / hostRegsPoke (raw, no trapping).
 */
#include "HalClockPower.h"

#include <stddef.h>

#include "HalHostSystem.h"

#define RCC_BASE 0x40023800u
#define PWR_BASE 0x40007000u

#define R_CR 0x00u
#define R_PLLCFGR 0x04u
#define R_CFGR 0x08u
#define R_CIR 0x0Cu
#define R_BDCR 0x70u
#define R_CSR 0x74u
#define R_PLLI2SCFGR 0x84u

#define P_CR 0x00u
#define P_CSR 0x04u

static uint32_t peek(uintptr_t a) { return hostRegsPeek(a); }
static void poke(uintptr_t a, uint32_t v) { hostRegsPoke(a, v); }

/* ---------------------------------------------------------------- oscillator state */
typedef struct {
	uint8_t active;   /* the ON bit is set */
	uint8_t settled;  /* ready regardless of time (the state after reset) */
	uint8_t fail;     /* never gets ready */
	uint8_t stuck;    /* the ready flag stays set, also after the ON bit is cleared */
	uint32_t rdyAt;   /* tick at which the flag rises */
	uint32_t startup; /* ticks between ON and ready */
	uint8_t prevReady;
} OscState;

static OscState osc[HOST_OSC_COUNT];
static uint32_t pwrVosStartup, pwrBkpStartup, pwrVosRdyAt, pwrBkpRdyAt, vddMv;
static uint8_t pwrVosSettled, pwrBkpSettled, pwrVosFail, pwrBkpFail, pwrVosActive;
static uint8_t pwrBkpActive, pwrSbf, pwrWuf, pwrBkpStuck;

static uint32_t now(void) { return hostTickPeek(); }

static int oscReadyNow(const OscState *o) {
	if (o->stuck) return 1;
	if (!o->active || o->fail) return 0;
	return o->settled || (int32_t)(now() - o->rdyAt) >= 0;
}

static void oscSwitchOn(OscState *o) {
	o->active = 1;
	o->settled = 0;
	o->rdyAt = now() + o->startup;
}
static void oscSwitchOff(OscState *o) {
	o->active = 0;
	o->settled = 0;
}

void hostClockPowerResetState(void) {
	for (int i = 0; i < HOST_OSC_COUNT; ++i) {
		osc[i].active = 0;
		osc[i].settled = 0;
		osc[i].fail = 0;
		osc[i].stuck = 0;
		osc[i].rdyAt = 0;
		osc[i].startup = 1U;
		osc[i].prevReady = 0;
	}
	osc[HOST_OSC_HSI].active = 1; /* reset state: HSI on and ready */
	osc[HOST_OSC_HSI].settled = 1;
	osc[HOST_OSC_HSI].prevReady = 1;
	pwrVosStartup = pwrBkpStartup = 1U;
	pwrVosRdyAt = pwrBkpRdyAt = 0;
	pwrVosSettled = 1; /* VOS = 1 after reset, VOSRDY set */
	pwrVosActive = 1;
	pwrBkpSettled = 0;
	pwrBkpActive = 0;
	pwrVosFail = pwrBkpFail = 0;
	pwrSbf = pwrWuf = 0;
	pwrBkpStuck = 0;
	vddMv = 3300U;
}

void hostRccSetStartupTicks(HostOsc o, uint32_t ticks) {
	if ((unsigned)o < HOST_OSC_COUNT) osc[o].startup = ticks;
}
void hostRccFail(HostOsc o, int fail) {
	if ((unsigned)o < HOST_OSC_COUNT) osc[o].fail = fail != 0;
}
void hostRccStickyReady(HostOsc o, int sticky) {
	if ((unsigned)o < HOST_OSC_COUNT) osc[o].stuck = sticky != 0;
}
void hostRccSetResetFlags(uint32_t mask) { poke(RCC_BASE + R_CSR, (peek(RCC_BASE + R_CSR) & ~0xFE000000u) | (mask & 0xFE000000u)); }

void hostPwrPeripheralReset(void);

/* ---------------------------------------------------------------- RCC */
static const uint8_t pllSrcBit = 22;

/* Which oscillator feeds the system clock now (SWS), or the PLL source behind it. Returns a bit mask of HostOsc. */
static uint32_t sysclkUses(void) {
	const uint32_t sws = (peek(RCC_BASE + R_CFGR) >> 2) & 3u;
	if (sws == 0u) return 1u << HOST_OSC_HSI;
	if (sws == 1u) return 1u << HOST_OSC_HSE;
	if (sws == 2u) {
		const uint32_t src = ((peek(RCC_BASE + R_PLLCFGR) >> pllSrcBit) & 1u) != 0u ? HOST_OSC_HSE : HOST_OSC_HSI;
		return (1u << HOST_OSC_PLL) | (1u << src);
	}
	return 0u;
}

static int swSourceReady(uint32_t sw) {
	switch (sw) {
	case 0: return oscReadyNow(&osc[HOST_OSC_HSI]);
	case 1: return oscReadyNow(&osc[HOST_OSC_HSE]);
	case 2: return oscReadyNow(&osc[HOST_OSC_PLL]) && (((peek(RCC_BASE + R_PLLCFGR) >> pllSrcBit) & 1u) != 0u
																? oscReadyNow(&osc[HOST_OSC_HSE])
																: oscReadyNow(&osc[HOST_OSC_HSI]));
	default: return 0;
	}
}

static int pllInputReady(void) {
	return ((peek(RCC_BASE + R_PLLCFGR) >> pllSrcBit) & 1u) != 0u ? oscReadyNow(&osc[HOST_OSC_HSE])
																   : oscReadyNow(&osc[HOST_OSC_HSI]);
}

static int readyOf(HostOsc which) {
	if (which == HOST_OSC_PLL || which == HOST_OSC_PLLI2S) return oscReadyNow(&osc[which]) && pllInputReady();
	return oscReadyNow(&osc[which]);
}

/* Brings the stored ready flags, SWS and the CIR flags up to date with the tick. */
static void rccEval(void) {
	uint32_t cr = peek(RCC_BASE + R_CR);
	uint32_t cir = peek(RCC_BASE + R_CIR);
	uint32_t cfgr = peek(RCC_BASE + R_CFGR);
	uint32_t csr = peek(RCC_BASE + R_CSR);
	uint32_t bdcr = peek(RCC_BASE + R_BDCR);
	const int hsi = readyOf(HOST_OSC_HSI), hse = readyOf(HOST_OSC_HSE), pll = readyOf(HOST_OSC_PLL), i2s = readyOf(HOST_OSC_PLLI2S);
	const int lse = readyOf(HOST_OSC_LSE), lsi = readyOf(HOST_OSC_LSI);
	cr = (cr & ~((1u << 1) | (1u << 17) | (1u << 25) | (1u << 27))) | ((uint32_t)hsi << 1) | ((uint32_t)hse << 17) |
		 ((uint32_t)pll << 25) | ((uint32_t)i2s << 27);
	csr = (csr & ~2u) | ((uint32_t)lsi << 1);
	bdcr = (bdcr & ~2u) | ((uint32_t)lse << 1);
	const int cur[HOST_OSC_COUNT] = {hsi, hse, pll, i2s, lse, lsi};
	/* CIR flag bits: LSI 0, LSE 1, HSI 2, HSE 3, PLL 4, PLLI2S 5 */
	static const uint8_t cirBit[HOST_OSC_COUNT] = {2, 3, 4, 5, 1, 0};
	for (int i = 0; i < HOST_OSC_COUNT; ++i) {
		/* RM0090 7.3.4: xxRDYF is set when the clock becomes stable and xxRDYIE (bit + 8) is set */
		if (cur[i] != 0 && osc[i].prevReady == 0 && (cir & (1u << (cirBit[i] + 8u))) != 0u) cir |= 1u << cirBit[i];
		osc[i].prevReady = (uint8_t)cur[i];
	}
	const uint32_t sw = cfgr & 3u;
	if (swSourceReady(sw)) cfgr = (cfgr & ~(3u << 2)) | (sw << 2);
	poke(RCC_BASE + R_CR, cr);
	poke(RCC_BASE + R_CIR, cir);
	poke(RCC_BASE + R_CFGR, cfgr);
	poke(RCC_BASE + R_CSR, csr);
	poke(RCC_BASE + R_BDCR, bdcr);
}

void hostRccHseFailure(void) {
	rccEval();
	if ((peek(RCC_BASE + R_CR) & (1u << 19)) == 0u) { /* CSS off: the oscillator just stops, nothing reacts (RM0090 7.3.1 CSSON) */
		osc[HOST_OSC_HSE].fail = 1;
		return;
	}
	const uint32_t uses = sysclkUses();
	uint32_t cr = peek(RCC_BASE + R_CR);
	uint32_t cfgr = peek(RCC_BASE + R_CFGR);
	oscSwitchOff(&osc[HOST_OSC_HSE]);
	osc[HOST_OSC_HSE].prevReady = 0;
	cr &= ~((1u << 16) | (1u << 17));
	if ((cr & (1u << 19)) != 0u) poke(RCC_BASE + R_CIR, peek(RCC_BASE + R_CIR) | (1u << 7)); /* CSSF */
	if ((uses & (1u << HOST_OSC_HSE)) != 0u) {
		cr |= 1u; /* HSION forced */
		if (osc[HOST_OSC_HSI].active == 0) {
			oscSwitchOn(&osc[HOST_OSC_HSI]);
		}
		cfgr &= ~3u; /* SW forced to HSI; SWS follows on the next read */
		oscSwitchOff(&osc[HOST_OSC_PLL]);
		cr &= ~(1u << 24);
		osc[HOST_OSC_PLL].prevReady = 0;
	}
	poke(RCC_BASE + R_CR, cr);
	poke(RCC_BASE + R_CFGR, cfgr);
}

void hostRccReset(void) {
	poke(RCC_BASE + 0x00u, 0x00000083u);
	poke(RCC_BASE + 0x04u, 0x24003010u);
	poke(RCC_BASE + 0x30u, 0x00100000u);
	poke(RCC_BASE + 0x50u, 0x7E6791FFu);
	poke(RCC_BASE + 0x54u, 0x000000F1u);
	poke(RCC_BASE + 0x58u, 0x00000001u);
	poke(RCC_BASE + 0x60u, 0x36FEC9FFu);
	poke(RCC_BASE + 0x64u, 0x00075F33u);
	poke(RCC_BASE + 0x74u, 0x0E000000u);
	poke(RCC_BASE + 0x84u, 0x20003000u);
}

int hostRccKnown(uintptr_t a) {
	const uint32_t off = (uint32_t)(a - RCC_BASE);
	switch (off) {
	case 0x00: case 0x04: case 0x08: case 0x0C: case 0x10: case 0x14: case 0x18: case 0x20: case 0x24:
	case 0x30: case 0x34: case 0x38: case 0x40: case 0x44: case 0x50: case 0x54: case 0x58: case 0x60: case 0x64:
	case 0x70: case 0x74: case 0x80: case 0x84: return 1;
	default: return 0;
	}
}

int hostRccBeforeRead(uintptr_t a) {
	if (!hostRccKnown(a)) return 0;
	const uint32_t off = (uint32_t)(a - RCC_BASE);
	if (off == R_CR || off == R_CFGR || off == R_CIR || off == R_BDCR || off == R_CSR) rccEval();
	return 1;
}

static uint32_t keepBits(uint32_t old, uint32_t mask, uint32_t w) { return (old & ~mask) | (w & mask); }

static void rccCrWrite(uint32_t oldValue, uint32_t w) {
	poke(RCC_BASE + R_CR, oldValue);
	rccEval();
	const uint32_t old = peek(RCC_BASE + R_CR);
	const uint32_t uses = sysclkUses();
	uint32_t nw = old;
	/* HSION (0), HSITRIM (7:3) */
	nw = keepBits(nw, 0xF8u, w);
	if ((w & 1u) != 0u) nw |= 1u;
	else if ((uses & (1u << HOST_OSC_HSI)) == 0u) nw &= ~1u;
	/* HSEON (16), HSEBYP (18, only while HSE is off), CSSON (19) */
	if ((w & (1u << 16)) != 0u) nw |= 1u << 16;
	else if ((uses & (1u << HOST_OSC_HSE)) == 0u) nw &= ~(1u << 16);
	if ((old & (1u << 16)) == 0u) nw = keepBits(nw, 1u << 18, w);
	nw = keepBits(nw, 1u << 19, w);
	/* PLLON (24) cannot be cleared while the PLL is the system clock; PLLI2SON (26) */
	if ((w & (1u << 24)) != 0u) nw |= 1u << 24;
	else if ((uses & (1u << HOST_OSC_PLL)) == 0u) nw &= ~(1u << 24);
	nw = keepBits(nw, 1u << 26, w);

	static const struct { uint8_t bit; HostOsc o; } on[] = {{0, HOST_OSC_HSI}, {16, HOST_OSC_HSE}, {24, HOST_OSC_PLL},
															{26, HOST_OSC_PLLI2S}};
	for (unsigned i = 0; i < 4u; ++i) {
		const int was = (old >> on[i].bit) & 1u, is = (nw >> on[i].bit) & 1u;
		if (!was && is) oscSwitchOn(&osc[on[i].o]);
		if (was && !is) {
			oscSwitchOff(&osc[on[i].o]);
			osc[on[i].o].prevReady = 0;
			nw &= ~(1u << (on[i].bit + 1u)); /* the ready flag clears at once */
		}
	}
	poke(RCC_BASE + R_CR, nw);
}

void hostRccAfterWrite(uintptr_t a, uint32_t oldValue, uint32_t written) {
	const uint32_t off = (uint32_t)(a - RCC_BASE);
	switch (off) {
	case R_CR: rccCrWrite(oldValue, written); break;
	case R_PLLCFGR: {
		const uint32_t cr = peek(RCC_BASE + R_CR);
		if ((cr & ((1u << 24) | (1u << 26))) != 0u) poke(a, oldValue); /* writable only with PLL and PLLI2S off */
		else poke(a, keepBits(oldValue, 0x0F437FFFu, written));
		break;
	}
	case R_CFGR: {
		rccEval();
		uint32_t nw = keepBits(oldValue, 0xFFFFFCF0u, written); /* everything but SWS (3:2), SW (1:0), reserved 9:8 */
		const uint32_t sw = written & 3u;
		nw = (nw & ~3u) | (sw == 3u ? (oldValue & 3u) : sw);
		nw = (nw & ~(3u << 2)) | (oldValue & (3u << 2)); /* SWS read-only */
		poke(a, nw);
		break;
	}
	case R_CIR: {
		uint32_t cir = oldValue;
		cir = keepBits(cir, 0x3F00u, written); /* RDYIE */
		cir &= ~((written >> 16) & 0x3Fu);       /* RDYC clears RDYF */
		if ((written & (1u << 23)) != 0u) cir &= ~(1u << 7); /* CSSC clears CSSF */
		poke(a, cir);
		break;
	}
	case R_BDCR: {
		const int dbp = (peek(PWR_BASE + P_CR) >> 8) & 1u;
		if (!dbp) {
			poke(a, oldValue);
			break;
		}
		uint32_t nw = oldValue;
		nw = keepBits(nw, 1u << 15, written); /* RTCEN */
		if ((oldValue & (3u << 8)) == 0u) nw = keepBits(nw, 3u << 8, written); /* RTCSEL write once */
		if ((oldValue & 1u) == 0u) nw = keepBits(nw, 1u << 2, written); /* LSEBYP only while LSE is off */
		if ((written & 1u) != 0u && (oldValue & 1u) == 0u) {
			nw |= 1u;
			oscSwitchOn(&osc[HOST_OSC_LSE]);
		} else if ((written & 1u) == 0u && (oldValue & 1u) != 0u) {
			nw &= ~3u; /* LSEON and LSERDY */
			oscSwitchOff(&osc[HOST_OSC_LSE]);
			osc[HOST_OSC_LSE].prevReady = 0;
		}
		nw = keepBits(nw, 1u << 16, written); /* BDRST */
		if ((nw & (1u << 16)) != 0u) { /* backup domain reset */
			nw &= ~((1u << 15) | (3u << 8) | (1u << 2) | 3u);
			oscSwitchOff(&osc[HOST_OSC_LSE]);
			osc[HOST_OSC_LSE].prevReady = 0;
			pwrBkpActive = 0;
			pwrBkpSettled = 0;
			poke(PWR_BASE + P_CSR, peek(PWR_BASE + P_CSR) & ~((1u << 9) | (1u << 3)));
		}
		poke(a, nw);
		break;
	}
	case R_CSR: {
		uint32_t nw = oldValue;
		if ((written & (1u << 24)) != 0u) nw &= ~0xFE000000u; /* RMVF clears the reset flags; reads 0 */
		if ((written & 1u) != 0u && (oldValue & 1u) == 0u) {
			nw |= 1u;
			oscSwitchOn(&osc[HOST_OSC_LSI]);
		} else if ((written & 1u) == 0u && (oldValue & 1u) != 0u) {
			nw &= ~3u;
			oscSwitchOff(&osc[HOST_OSC_LSI]);
			osc[HOST_OSC_LSI].prevReady = 0;
		}
		poke(a, nw);
		break;
	}
	case R_PLLI2SCFGR: {
		const uint32_t cr = peek(RCC_BASE + R_CR);
		if ((cr & (1u << 26)) != 0u) poke(a, oldValue);
		else poke(a, keepBits(oldValue, 0x70007FC0u, written));
		break;
	}
	case 0x20u: /* APB1RSTR: PWRRST (bit 28) resets the PWR registers (RM0090 7.3.16); BRE is only reset by a backup domain reset */
		poke(a, written);
		if ((written & (1u << 28)) != 0u) hostPwrPeripheralReset();
		break;
	default: poke(a, written); break;
	}
}

/* ---------------------------------------------------------------- PWR */
static int pwrKnown(uintptr_t a) { return a == PWR_BASE + P_CR || a == PWR_BASE + P_CSR; }

void hostPwrPeripheralReset(void) {
	const uint32_t csr = peek(PWR_BASE + P_CSR);
	poke(PWR_BASE + P_CR, 0x00004000u);
	poke(PWR_BASE + P_CSR, csr & (1u << 9)); /* BRE kept, EWUP cleared */
	pwrVosActive = 1;
	pwrVosSettled = 1;
}

static void pwrReset(void) { poke(PWR_BASE + P_CR, 0x00004000u); poke(PWR_BASE + P_CSR, 0u); }

static void pwrEval(void) {
	static const uint16_t thr[8] = {2000, 2100, 2300, 2500, 2600, 2700, 2800, 2900};
	const uint32_t cr = peek(PWR_BASE + P_CR);
	uint32_t csr = peek(PWR_BASE + P_CSR);
	const int vosRdy = pwrVosActive && !pwrVosFail && (pwrVosSettled || (int32_t)(now() - pwrVosRdyAt) >= 0);
	const int brr = pwrBkpStuck || (pwrBkpActive && !pwrBkpFail && (pwrBkpSettled || (int32_t)(now() - pwrBkpRdyAt) >= 0));
	const int pvdo = ((cr >> 4) & 1u) != 0u && vddMv < thr[(cr >> 5) & 7u];
	csr &= ~((1u << 14) | (1u << 3) | (1u << 2) | (1u << 1) | 1u);
	csr |= ((uint32_t)vosRdy << 14) | ((uint32_t)brr << 3) | ((uint32_t)pvdo << 2) | ((uint32_t)pwrSbf << 1) | pwrWuf;
	poke(PWR_BASE + P_CSR, csr);
}

static int pwrBeforeRead(uintptr_t a) {
	if (!pwrKnown(a)) return 0;
	if (a == PWR_BASE + P_CSR) pwrEval();
	return 1;
}

static void pwrAfterWrite(uintptr_t a, uint32_t oldValue, uint32_t written) {
	if (a == PWR_BASE + P_CR) {
		uint32_t nw = written & 0x000043F3u; /* VOS, FPDS, DBP, PLS, PVDE, PDDS, LPDS; CSBF / CWUF read 0 */
		if ((written & (1u << 3)) != 0u) pwrSbf = 0;
		if ((written & (1u << 2)) != 0u) pwrWuf = 0;
		if (((oldValue ^ nw) & (1u << 14)) != 0u) { /* VOS changed: VOSRDY follows after the start-up time */
			pwrVosActive = 1;
			pwrVosSettled = 0;
			pwrVosRdyAt = now() + pwrVosStartup;
		}
		poke(a, nw);
	} else {
		const int dbp = (peek(PWR_BASE + P_CR) >> 8) & 1u;
		uint32_t nw = oldValue;
		nw = keepBits(nw, 1u << 8, written); /* EWUP */
		if (dbp) {
			nw = keepBits(nw, 1u << 9, written); /* BRE */
			if (((oldValue ^ nw) & (1u << 9)) != 0u) {
				pwrBkpActive = (nw >> 9) & 1u;
				pwrBkpSettled = 0;
				pwrBkpRdyAt = now() + pwrBkpStartup;
			}
		}
		poke(a, nw);
	}
}

static const HostRegModel pwrModel = {"PWR", PWR_BASE, 0x400u, pwrReset, pwrBeforeRead, NULL, pwrKnown, pwrAfterWrite};

void hostClockPowerRegister(void) { hostRegsAddModel(&pwrModel); }

void hostPwrSetStartupTicks(uint32_t vos, uint32_t bkp) { pwrVosStartup = vos; pwrBkpStartup = bkp; }
void hostPwrFailVos(int fail) { pwrVosFail = fail != 0; }
void hostPwrFailBackupRegulator(int fail) { pwrBkpFail = fail != 0; }
void hostPwrStickyBackupReady(int sticky) { pwrBkpStuck = sticky != 0; }
void hostPwrSetVddMv(uint32_t mv) { vddMv = mv; }
void hostPwrSetFlags(int sbf, int wuf) { pwrSbf = sbf != 0; pwrWuf = wuf != 0; }
