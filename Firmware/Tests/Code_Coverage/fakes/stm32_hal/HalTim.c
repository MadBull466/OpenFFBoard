/*
 * HalTim.c: TIM1..TIM14 register model (P12 T6a). See HalTim.h for what is modelled and the RM0090 sources.
 * Register access goes through hostRegsPeek / hostRegsPoke (raw, no trapping).
 */
#include "HalTim.h"

#include <stddef.h>
#include <string.h>

#define R_CR1 0x00u
#define R_CR2 0x04u
#define R_SMCR 0x08u
#define R_DIER 0x0Cu
#define R_SR 0x10u
#define R_EGR 0x14u
#define R_CCMR1 0x18u
#define R_CCMR2 0x1Cu
#define R_CCER 0x20u
#define R_CNT 0x24u
#define R_PSC 0x28u
#define R_ARR 0x2Cu
#define R_RCR 0x30u
#define R_CCR1 0x34u
#define R_CCR2 0x38u
#define R_CCR3 0x3Cu
#define R_CCR4 0x40u
#define R_BDTR 0x44u
#define R_DCR 0x48u
#define R_DMAR 0x4Cu
#define R_OR 0x50u

#define CR1_CEN 1u
#define CR1_UDIS 2u
#define CR1_URS 4u
#define CR1_OPM 8u
#define CR1_DIR 0x10u
#define CR1_CMS 0x60u
#define EGR_UG 1u
#define EGR_COMG (1u << 5)
#define EGR_TG (1u << 6)
#define EGR_BG (1u << 7)
#define SR_UIF 1u
#define SR_COMIF (1u << 5)
#define SR_TIF (1u << 6)
#define SR_BIF (1u << 7)
#define SR_IRQ_FLAGS 0x00FFu
#define BDTR_MOE (1u << 15)
#define BDTR_AOE (1u << 14)
#define BDTR_LOCK 0x300u

enum { T_ADV, T_GP, T_BASIC, T_CH2, T_CH1 };

typedef struct {
	uint32_t base;
	uint8_t type, is32, nch;
	uint32_t cr1, cr2, smcr, dier, sr, egr, ccmr1, ccmr2, ccer, rcr, bdtr, dcr, orMask;
} TimDesc;

static const TimDesc descs[HOST_TIM_COUNT] = {
	/* TIM1  */ {0x40010000u, T_ADV, 0, 4, 0x3FFu, 0x7FFDu, 0xFFF7u, 0x7FFFu, 0x1EFFu, 0xFFu, 0xFFFFu, 0xFFFFu, 0xBFFFu, 0xFFu, 0xFFFFu, 0x1F1Fu, 0},
	/* TIM2  */ {0x40000000u, T_GP, 1, 4, 0x3FFu, 0xF8u, 0xFFF7u, 0x5F5Fu, 0x1E5Fu, 0x5Fu, 0xFFFFu, 0xFFFFu, 0xBBBBu, 0, 0, 0x1F1Fu, 0x0C00u},
	/* TIM3  */ {0x40000400u, T_GP, 0, 4, 0x3FFu, 0xF8u, 0xFFF7u, 0x5F5Fu, 0x1E5Fu, 0x5Fu, 0xFFFFu, 0xFFFFu, 0xBBBBu, 0, 0, 0x1F1Fu, 0},
	/* TIM4  */ {0x40000800u, T_GP, 0, 4, 0x3FFu, 0xF8u, 0xFFF7u, 0x5F5Fu, 0x1E5Fu, 0x5Fu, 0xFFFFu, 0xFFFFu, 0xBBBBu, 0, 0, 0x1F1Fu, 0},
	/* TIM5  */ {0x40000C00u, T_GP, 1, 4, 0x3FFu, 0xF8u, 0xFFF7u, 0x5F5Fu, 0x1E5Fu, 0x5Fu, 0xFFFFu, 0xFFFFu, 0xBBBBu, 0, 0, 0x1F1Fu, 0x00C0u},
	/* TIM6  */ {0x40001000u, T_BASIC, 0, 0, 0x8Fu, 0x70u, 0, 0x101u, 0x1u, 0x1u, 0, 0, 0, 0, 0, 0, 0},
	/* TIM7  */ {0x40001400u, T_BASIC, 0, 0, 0x8Fu, 0x70u, 0, 0x101u, 0x1u, 0x1u, 0, 0, 0, 0, 0, 0, 0},
	/* TIM8  */ {0x40010400u, T_ADV, 0, 4, 0x3FFu, 0x7FFDu, 0xFFF7u, 0x7FFFu, 0x1EFFu, 0xFFu, 0xFFFFu, 0xFFFFu, 0xBFFFu, 0xFFu, 0xFFFFu, 0x1F1Fu, 0},
	/* TIM9  */ {0x40014000u, T_CH2, 0, 2, 0x38Fu, 0x70u, 0xF7u, 0x47u, 0x647u, 0x47u, 0xFFFFu, 0, 0xBBu, 0, 0, 0, 0},
	/* TIM10 */ {0x40014400u, T_CH1, 0, 1, 0x387u, 0, 0, 0x3u, 0x203u, 0x3u, 0xFFu, 0, 0xBu, 0, 0, 0, 0},
	/* TIM11 */ {0x40014800u, T_CH1, 0, 1, 0x387u, 0, 0, 0x3u, 0x203u, 0x3u, 0xFFu, 0, 0xBu, 0, 0, 0, 0x0003u},
	/* TIM12 */ {0x40001800u, T_CH2, 0, 2, 0x38Fu, 0x70u, 0xF7u, 0x47u, 0x647u, 0x47u, 0xFFFFu, 0, 0xBBu, 0, 0, 0, 0},
	/* TIM13 */ {0x40001C00u, T_CH1, 0, 1, 0x387u, 0, 0, 0x3u, 0x203u, 0x3u, 0xFFu, 0, 0xBu, 0, 0, 0, 0},
	/* TIM14 */ {0x40002000u, T_CH1, 0, 1, 0x387u, 0, 0, 0x3u, 0x203u, 0x3u, 0xFFu, 0, 0xBu, 0, 0, 0, 0},
};

typedef struct {
	uint8_t dmaIdx, down, lockSet;
	uint32_t repLeft, protectedWrites;
} TimState;
static TimState ts[HOST_TIM_COUNT];

static uint32_t peek(uintptr_t a) { return hostRegsPeek(a); }
static void poke(uintptr_t a, uint32_t v) { hostRegsPoke(a, v); }
static uintptr_t reg(unsigned i, uint32_t r) { return descs[i].base + r; }
static uint32_t get(unsigned i, uint32_t r) { return peek(reg(i, r)); }
static void put(unsigned i, uint32_t r, uint32_t v) { poke(reg(i, r), v); }
static void setBits(unsigned i, uint32_t r, uint32_t m) { put(i, r, get(i, r) | m); }
static void clrBits(unsigned i, uint32_t r, uint32_t m) { put(i, r, get(i, r) & ~m); }
static uint32_t cntMask(unsigned i) { return descs[i].is32 ? 0xFFFFFFFFu : 0xFFFFu; }

void hostTimResetState(void) { memset(ts, 0, sizeof ts); }
uint32_t hostTimProtectedWrites(unsigned tim) { return (tim >= 1u && tim <= HOST_TIM_COUNT) ? ts[tim - 1u].protectedWrites : 0u; }

static void resetRegs(unsigned i) { put(i, R_ARR, cntMask(i)); }

/* ------------------------------------------------------------------ channels */
static int chExists(unsigned i, unsigned ch) { return ch >= 1u && ch <= descs[i].nch; }
static uint32_t ccmrOf(unsigned i, unsigned ch) { return get(i, ch <= 2u ? R_CCMR1 : R_CCMR2); }
static unsigned ccs(unsigned i, unsigned ch) { return (ccmrOf(i, ch) >> (((ch - 1u) & 1u) * 8u)) & 3u; }
static int ccEnabled(unsigned i, unsigned ch) {
	const unsigned shift = (ch <= 3u) ? (ch - 1u) * 4u : 12u;
	return ((get(i, R_CCER) >> shift) & 1u) != 0u;
}
static uint32_t ccrOffset(unsigned ch) { return R_CCR1 + 4u * (ch - 1u); }

static void updateEvent(unsigned i, int fromUg) {
	const uint32_t cr1 = get(i, R_CR1);
	if ((cr1 & CR1_UDIS) == 0u && !(fromUg && (cr1 & CR1_URS) != 0u)) setBits(i, R_SR, SR_UIF);
	if (descs[i].type == T_ADV && (get(i, R_BDTR) & BDTR_AOE) != 0u) setBits(i, R_BDTR, BDTR_MOE);
	if ((cr1 & CR1_OPM) != 0u) clrBits(i, R_CR1, CR1_CEN);
}

static int doCapture(unsigned i, unsigned ch, uint32_t value) {
	if (!chExists(i, ch) || ccs(i, ch) == 0u) return 0;
	if (get(i, R_SR) & (1u << ch)) setBits(i, R_SR, 1u << (8u + ch)); /* CCxOF */
	put(i, ccrOffset(ch), value & cntMask(i));
	setBits(i, R_SR, (1u << ch) & descs[i].sr);
	return 1;
}

int hostTimCapture(unsigned tim, unsigned ch, uint32_t value) {
	if (tim < 1u || tim > HOST_TIM_COUNT) return 0;
	return doCapture(tim - 1u, ch, value);
}
void hostTimSetCounter(unsigned tim, uint32_t value) { if (tim >= 1u && tim <= HOST_TIM_COUNT) put(tim - 1u, R_CNT, value & cntMask(tim - 1u)); }
void hostTimRaiseFlags(unsigned tim, uint32_t m) { if (tim >= 1u && tim <= HOST_TIM_COUNT) setBits(tim - 1u, R_SR, m & descs[tim - 1u].sr); }
void hostTimBreak(unsigned tim) {
	if (tim < 1u || tim > HOST_TIM_COUNT) return;
	setBits(tim - 1u, R_SR, SR_BIF & descs[tim - 1u].sr);
	if (descs[tim - 1u].type == T_ADV) clrBits(tim - 1u, R_BDTR, BDTR_MOE);
}
uint32_t hostTimIrqPending(unsigned tim) {
	if (tim < 1u || tim > HOST_TIM_COUNT) return 0;
	return get(tim - 1u, R_SR) & get(tim - 1u, R_DIER) & SR_IRQ_FLAGS;
}

static void compareMatches(unsigned i, uint32_t cnt) {
	for (unsigned ch = 1; ch <= descs[i].nch; ++ch)
		if (ccs(i, ch) == 0u && get(i, ccrOffset(ch)) == cnt) setBits(i, R_SR, (1u << ch) & descs[i].sr);
}

void hostTimAdvance(unsigned tim, uint32_t n) {
	unsigned i;
	if (tim < 1u || tim > HOST_TIM_COUNT) return;
	i = tim - 1u;
	for (uint32_t k = 0; k < n; ++k) {
		const uint32_t cr1 = get(i, R_CR1);
		uint64_t cnt = get(i, R_CNT);
		const uint64_t arr = get(i, R_ARR);
		int overflow = 0;
		if ((cr1 & CR1_CEN) == 0u) return;
		if ((cr1 & CR1_CMS) != 0u) { /* centre aligned: up to ARR, down to 0 */
			if (!ts[i].down) {
				if (cnt >= arr) { ts[i].down = 1; if (cnt > 0u) cnt--; } else cnt++;
			} else if (cnt == 0u) {
				ts[i].down = 0;
				overflow = 1;
				cnt = arr > 0u ? 1u : 0u;
			} else cnt--;
		} else if (descs[i].type != T_BASIC && (cr1 & CR1_DIR) != 0u) {
			if (cnt == 0u) { cnt = arr; overflow = 1; } else cnt--;
		} else {
			if (cnt >= arr) { cnt = 0; overflow = 1; } else cnt++;
		}
		put(i, R_CNT, (uint32_t)cnt);
		if (descs[i].nch != 0u) compareMatches(i, (uint32_t)cnt);
		if (overflow) {
			if (descs[i].type == T_ADV && get(i, R_RCR) != 0u && ts[i].repLeft != 0u) ts[i].repLeft--;
			else {
				ts[i].repLeft = descs[i].type == T_ADV ? get(i, R_RCR) : 0u;
				updateEvent(i, 0);
			}
		}
	}
}

/* ------------------------------------------------------------------ EGR */
static void egr(unsigned i, uint32_t v) {
	const TimDesc *d = &descs[i];
	v &= d->egr;
	if (v & EGR_UG) {
		put(i, R_CNT, ((get(i, R_CR1) & CR1_DIR) != 0u && d->type != T_BASIC) ? get(i, R_ARR) : 0u);
		ts[i].down = 0;
		ts[i].repLeft = d->type == T_ADV ? get(i, R_RCR) : 0u;
		updateEvent(i, 1);
	}
	for (unsigned ch = 1; ch <= d->nch; ++ch) {
		if (v & (1u << ch)) {
			if (ccs(i, ch) != 0u) doCapture(i, ch, get(i, R_CNT));
			else setBits(i, R_SR, 1u << ch);
		}
	}
	if (v & EGR_COMG) setBits(i, R_SR, SR_COMIF);
	if (v & EGR_TG) setBits(i, R_SR, SR_TIF);
	if (v & EGR_BG) {
		setBits(i, R_SR, SR_BIF);
		clrBits(i, R_BDTR, BDTR_MOE);
	}
}

/* ------------------------------------------------------------------ bus callbacks */
static int known(unsigned i, uintptr_t a) {
	const TimDesc *d = &descs[i];
	const uint32_t off = (uint32_t)(a - d->base);
	if ((off & 3u) != 0u) return 0;
	switch (off) {
	case R_CR1: case R_SR: case R_EGR: case R_CNT: case R_PSC: case R_ARR: case R_DIER: return 1;
	case R_CR2: return d->cr2 != 0u;
	case R_SMCR: return d->smcr != 0u;
	case R_CCMR1: return d->ccmr1 != 0u;
	case R_CCMR2: return d->ccmr2 != 0u;
	case R_CCER: return d->ccer != 0u;
	case R_RCR: return d->rcr != 0u;
	case R_CCR1: return d->nch >= 1u;
	case R_CCR2: return d->nch >= 2u;
	case R_CCR3: case R_CCR4: return d->nch >= 4u;
	case R_BDTR: return d->bdtr != 0u;
	case R_DCR: case R_DMAR: return d->dcr != 0u;
	case R_OR: return d->orMask != 0u;
	default: return 0;
	}
}

static int beforeRead(unsigned i, uintptr_t a) {
	if (!known(i, a)) return 0;
	if ((uint32_t)(a - descs[i].base) == R_DMAR) {
		const uint32_t target = ((get(i, R_DCR) & 0x1Fu) + ts[i].dmaIdx) * 4u;
		poke(a, target <= R_OR && known(i, descs[i].base + target) && target != R_DMAR ? get(i, target) : 0u);
	}
	return 1;
}

static void afterRead(unsigned i, uintptr_t a, uint32_t value) {
	(void)value;
	if ((uint32_t)(a - descs[i].base) == R_DMAR) {
		const unsigned dbl = (get(i, R_DCR) >> 8) & 0x1Fu;
		ts[i].dmaIdx = (uint8_t)(ts[i].dmaIdx >= dbl ? 0u : ts[i].dmaIdx + 1u);
	}
}

static void protectedWrite(unsigned i, uintptr_t a, uint32_t oldValue) {
	ts[i].protectedWrites++;
	poke(a, oldValue);
}

static uint32_t lockLevel(unsigned i) { return descs[i].type == T_ADV ? (get(i, R_BDTR) >> 8) & 3u : 0u; }

static void afterWrite(unsigned i, uintptr_t a, uint32_t oldValue, uint32_t written) {
	const TimDesc *d = &descs[i];
	const uint32_t off = (uint32_t)(a - d->base);
	const uint32_t lock = lockLevel(i);
	switch (off) {
	case R_CR1: poke(a, written & d->cr1); break;
	case R_CR2: {
		uint32_t nw = written & d->cr2;
		if (lock >= 1u && d->type == T_ADV) nw = (nw & ~0x7F00u) | (oldValue & 0x7F00u); /* OISx / OISxN locked */
		poke(a, nw);
		break;
	}
	case R_SMCR: poke(a, written & d->smcr); break;
	case R_DIER: poke(a, written & d->dier); break;
	case R_SR: poke(a, oldValue & (written | ~d->sr)); break;
	case R_EGR: poke(a, 0u); egr(i, written); break;
	case R_CCMR1:
	case R_CCMR2: {
		const uint32_t mask = off == R_CCMR1 ? d->ccmr1 : d->ccmr2;
		uint32_t nw = (written & mask) & ~0u;
		for (unsigned half = 0; half < 2u; ++half) {
			const unsigned ch = (off == R_CCMR1 ? 1u : 3u) + half;
			const uint32_t sh = half * 8u, csel = 3u << sh;
			if (ch > d->nch) continue;
			if (ccEnabled(i, ch) && ((oldValue ^ nw) & csel) != 0u) { /* CCxS only while the channel is off */
				nw = (nw & ~csel) | (oldValue & csel);
				ts[i].protectedWrites++;
			}
			if (lock >= 3u && d->type == T_ADV) { /* OCxM / OCxPE locked (bits 6:3 of each half) */
				const uint32_t lk = 0x78u << sh;
				nw = (nw & ~lk) | (oldValue & lk);
			}
		}
		poke(a, nw);
		break;
	}
	case R_CCER: {
		uint32_t nw = written & d->ccer;
		if (lock >= 2u && d->type == T_ADV) nw = (nw & ~0xAAAAu) | (oldValue & 0xAAAAu); /* CCxP and CCxNP locked */
		poke(a, nw);
		break;
	}
	case R_CNT: poke(a, written & cntMask(i)); break;
	case R_PSC: poke(a, written & 0xFFFFu); break;
	case R_ARR: poke(a, written & cntMask(i)); break;
	case R_RCR: poke(a, written & d->rcr); break;
	case R_CCR1: case R_CCR2: case R_CCR3: case R_CCR4: {
		const unsigned ch = (off - R_CCR1) / 4u + 1u;
		if (ccs(i, ch) != 0u) protectedWrite(i, a, oldValue); /* read-only for an input channel */
		else poke(a, written & cntMask(i));
		break;
	}
	case R_BDTR: {
		uint32_t nw = written & d->bdtr, locked = 0u;
		if ((oldValue & BDTR_LOCK) != 0u) nw = (nw & ~BDTR_LOCK) | (oldValue & BDTR_LOCK); /* LOCK is write once */
		/* the level in force is the one set BEFORE this write: the write that sets LOCK is itself accepted */
		const uint32_t bl = (oldValue >> 8) & 3u;
		if (bl >= 1u) locked |= 0x70FFu; /* DTG, BKE, BKP, AOE */
		if (bl >= 2u) locked |= 0x0C00u; /* OSSR, OSSI */
		if (((nw ^ oldValue) & locked) != 0u) ts[i].protectedWrites++;
		nw = (nw & ~locked) | (oldValue & locked);
		poke(a, nw);
		break;
	}
	case R_DCR: poke(a, written & d->dcr); ts[i].dmaIdx = 0; break;
	case R_DMAR: {
		const uint32_t target = ((get(i, R_DCR) & 0x1Fu) + ts[i].dmaIdx) * 4u;
		const unsigned dbl = (get(i, R_DCR) >> 8) & 0x1Fu;
		poke(a, 0u);
		if (target <= R_OR && target != R_DMAR && known(i, d->base + target)) {
			/* the redirected access goes through the register's own write rules */
			afterWrite(i, d->base + target, get(i, target), written);
		}
		ts[i].dmaIdx = (uint8_t)(ts[i].dmaIdx >= dbl ? 0u : ts[i].dmaIdx + 1u);
		break;
	}
	case R_OR: poke(a, written & d->orMask); break;
	default: poke(a, oldValue); break;
	}
}

#define TIM_THUNKS(N)                                                                          \
	static int known##N(uintptr_t a) { return known(N, a); }                                   \
	static int beforeRead##N(uintptr_t a) { return beforeRead(N, a); }                         \
	static void afterRead##N(uintptr_t a, uint32_t v) { afterRead(N, a, v); }                  \
	static void afterWrite##N(uintptr_t a, uint32_t o, uint32_t w) { afterWrite(N, a, o, w); } \
	static void reset##N(void) { resetRegs(N); }
TIM_THUNKS(0) TIM_THUNKS(1) TIM_THUNKS(2) TIM_THUNKS(3) TIM_THUNKS(4) TIM_THUNKS(5) TIM_THUNKS(6)
TIM_THUNKS(7) TIM_THUNKS(8) TIM_THUNKS(9) TIM_THUNKS(10) TIM_THUNKS(11) TIM_THUNKS(12) TIM_THUNKS(13)

#define TIM_MODEL(N, NAME) {NAME, 0, 0x400u, reset##N, beforeRead##N, afterRead##N, known##N, afterWrite##N}
static HostRegModel timModels[HOST_TIM_COUNT] = {
	TIM_MODEL(0, "TIM1"), TIM_MODEL(1, "TIM2"), TIM_MODEL(2, "TIM3"), TIM_MODEL(3, "TIM4"), TIM_MODEL(4, "TIM5"), TIM_MODEL(5, "TIM6"), TIM_MODEL(6, "TIM7"),
	TIM_MODEL(7, "TIM8"), TIM_MODEL(8, "TIM9"), TIM_MODEL(9, "TIM10"), TIM_MODEL(10, "TIM11"), TIM_MODEL(11, "TIM12"), TIM_MODEL(12, "TIM13"), TIM_MODEL(13, "TIM14"),
};

void hostTimRegister(void) {
	for (unsigned i = 0; i < HOST_TIM_COUNT; ++i) {
		timModels[i].base = descs[i].base;
		hostRegsAddModel(&timModels[i]);
	}
}
