/*
 * HalDma.c: DMA1 / DMA2 register model (P12 T3). See HalDma.h for what is modelled and the RM0090 sources.
 * Register access goes through hostRegsPeek / hostRegsPoke (raw, no trapping).
 */
#include "HalDma.h"

#include <stddef.h>

#include "HalHostSystem.h"

#define DMA1_BASE 0x40026000u
#define DMA2_BASE 0x40026400u

#define LISR 0x00u
#define HISR 0x04u
#define LIFCR 0x08u
#define HIFCR 0x0Cu
#define STREAM_BASE 0x10u
#define STREAM_STRIDE 0x18u
#define R_CR 0x00u
#define R_NDTR 0x04u
#define R_PAR 0x08u
#define R_M0AR 0x0Cu
#define R_M1AR 0x10u
#define R_FCR 0x14u

#define CR_EN (1u << 0)
#define CR_DBM (1u << 18)
#define CR_CT (1u << 19)
#define CR_CIRC (1u << 8)
#define CR_PFCTRL (1u << 5)
#define CR_WRITABLE 0x0FEFFFFFu
#define CR_ALWAYS 0x0000001Fu /* EN and the four interrupt enables are writable with EN = 1 */

static uintptr_t base(unsigned c) { return c == 0u ? DMA1_BASE : DMA2_BASE; }
uintptr_t hostDmaBase(unsigned c) { return base(c); }

static uint32_t peek(uintptr_t a) { return hostRegsPeek(a); }
static void poke(uintptr_t a, uint32_t v) { hostRegsPoke(a, v); }

typedef struct {
	uint8_t stick;
	uint8_t stopping;
	uint32_t stopAt;
	uint32_t ndtInit;
	uint32_t protectedWrites;
	uint8_t halfSeen;
	uint32_t readsLeft; /* complete the stream after this many reads of its status register (0: off) */
} StreamState;

static StreamState st[HOST_DMA_CONTROLLERS][HOST_DMA_STREAMS];
static unsigned disableDelay;

static uintptr_t sreg(unsigned c, unsigned s, uint32_t r) { return base(c) + STREAM_BASE + STREAM_STRIDE * s + r; }

void hostDmaCompleteAfterReads(unsigned c, unsigned s, uint32_t reads) {
	if (c < HOST_DMA_CONTROLLERS && s < HOST_DMA_STREAMS) st[c][s].readsLeft = reads;
}
void hostDmaSetDisableDelay(unsigned ticks) { disableDelay = ticks; }
void hostDmaStickEnabled(unsigned c, unsigned s, int stick) {
	if (c < HOST_DMA_CONTROLLERS && s < HOST_DMA_STREAMS) st[c][s].stick = stick != 0;
}
uint32_t hostDmaProtectedWrites(unsigned c, unsigned s) {
	return (c < HOST_DMA_CONTROLLERS && s < HOST_DMA_STREAMS) ? st[c][s].protectedWrites : 0u;
}

void hostDmaResetState(void) {
	for (unsigned c = 0; c < HOST_DMA_CONTROLLERS; ++c)
		for (unsigned s = 0; s < HOST_DMA_STREAMS; ++s) st[c][s] = (StreamState){0};
	disableDelay = 0;
}

/* ------------------------------------------------------------------ flags */
static unsigned flagShift(unsigned s) {
	static const uint8_t sh[4] = {0, 6, 16, 22};
	return sh[s & 3u];
}
static uintptr_t isr(unsigned c, unsigned s) { return base(c) + (s < 4u ? LISR : HISR); }

static void setFlags(unsigned c, unsigned s, uint32_t mask) {
	poke(isr(c, s), peek(isr(c, s)) | ((mask & 0x3Du) << flagShift(s)));
}
uint32_t hostDmaFlags(unsigned c, unsigned s) { return (peek(isr(c, s)) >> flagShift(s)) & 0x3Du; }

/* ------------------------------------------------------------------ stream control */
static void setFlags(unsigned c, unsigned s, uint32_t mask);
static void setEn(unsigned c, unsigned s, int en);
/* A software stop (EN cleared by the CPU) ends with TCIF set once the stream is stopped (RM0090 10.3.13, 10.3.14). */
static void softStop(unsigned c, unsigned s) {
	setEn(c, s, 0);
	setFlags(c, s, HOST_DMA_TCIF);
}

static void setEn(unsigned c, unsigned s, int en) {
	const uintptr_t a = sreg(c, s, R_CR);
	poke(a, en ? (peek(a) | CR_EN) : (peek(a) & ~CR_EN));
	if (!en) st[c][s].stopping = 0;
}

/* Brings a requested stop up to date with the tick. */
static void evalStop(unsigned c, unsigned s) {
	StreamState *x = &st[c][s];
	if (x->stopping && !x->stick && (int32_t)(hostTickPeek() - x->stopAt) >= 0) softStop(c, s);
}

static int fifoCompatible(uint32_t cr, uint32_t fcr) {
	const uint32_t mburst = (cr >> 23) & 3u;
	if (mburst == 0u) return 1;
	const uint32_t beats = 2u << mburst; /* INCR4 = 4, INCR8 = 8, INCR16 = 16 */
	const uint32_t size = 1u << ((cr >> 13) & 3u);
	const uint32_t thr = (((fcr & 3u) + 1u) * 4u);
	const uint32_t burstBytes = beats * size;
	return burstBytes <= 16u && (thr % burstBytes) == 0u;
}

static void startStream(unsigned c, unsigned s) {
	const uintptr_t acr = sreg(c, s, R_CR), afcr = sreg(c, s, R_FCR);
	uint32_t cr = peek(acr), fcr = peek(afcr);
	const uint32_t dir = (cr >> 6) & 3u;
	if (dir == 2u) { /* memory to memory: direct mode not allowed, PFCTRL = 0 */
		fcr |= 1u << 2;
		cr &= ~CR_PFCTRL;
	}
	if (cr & CR_DBM) cr |= CR_CIRC;
	if (cr & CR_PFCTRL) cr &= ~CR_CIRC;
	if ((fcr & (1u << 2)) == 0u) { /* direct mode */
		cr &= ~((3u << 23) | (3u << 21) | (1u << 15));
		cr = (cr & ~(3u << 13)) | (((cr >> 11) & 3u) << 13);
	}
	poke(afcr, fcr);
	poke(acr, cr);
	st[c][s].halfSeen = 0;
	st[c][s].ndtInit = peek(sreg(c, s, R_NDTR)) & 0xFFFFu;
	if ((fcr & (1u << 2)) != 0u && !fifoCompatible(cr, fcr)) { /* Table 49: FEIF, stream disabled */
		setFlags(c, s, HOST_DMA_FEIF);
		setEn(c, s, 0);
	}
}

void hostDmaRaise(unsigned c, unsigned s, uint32_t mask) {
	if (c >= HOST_DMA_CONTROLLERS || s >= HOST_DMA_STREAMS) return;
	setFlags(c, s, mask);
	if (mask & HOST_DMA_TEIF) setEn(c, s, 0); /* RM0090 10.3.18: a transfer error disables the stream */
}

void hostDmaComplete(unsigned c, unsigned s) {
	if (c >= HOST_DMA_CONTROLLERS || s >= HOST_DMA_STREAMS) return;
	const uintptr_t acr = sreg(c, s, R_CR), andt = sreg(c, s, R_NDTR);
	const uint32_t cr = peek(acr);
	setFlags(c, s, HOST_DMA_TCIF);
	if (cr & CR_CIRC) {
		poke(andt, st[c][s].ndtInit);
		if (cr & CR_DBM) poke(acr, cr ^ CR_CT);
	} else {
		poke(andt, 0u);
		setEn(c, s, 0);
	}
}

void hostDmaStepItems(unsigned c, unsigned s, uint32_t items) {
	if (c >= HOST_DMA_CONTROLLERS || s >= HOST_DMA_STREAMS) return;
	const uintptr_t andt = sreg(c, s, R_NDTR);
	uint32_t ndt = peek(andt) & 0xFFFFu;
	if ((peek(sreg(c, s, R_CR)) & CR_EN) == 0u || ndt == 0u) return;
	const uint32_t init = st[c][s].ndtInit;
	ndt = items >= ndt ? 0u : ndt - items;
	poke(andt, ndt);
	if (!st[c][s].halfSeen && ndt <= init / 2u) {
		st[c][s].halfSeen = 1;
		setFlags(c, s, HOST_DMA_HTIF);
	}
	if (ndt == 0u) {
		st[c][s].halfSeen = 0;
		hostDmaComplete(c, s);
	}
}

/* ------------------------------------------------------------------ bus callbacks */
static int known(uintptr_t a, unsigned c) {
	const uint32_t off = (uint32_t)(a - base(c));
	if (off <= HIFCR) return (off & 3u) == 0u;
	if (off < STREAM_BASE) return 0;
	const uint32_t r = (off - STREAM_BASE) % STREAM_STRIDE;
	return off < STREAM_BASE + STREAM_STRIDE * HOST_DMA_STREAMS && r <= R_FCR && (r & 3u) == 0u;
}
static int known1(uintptr_t a) { return known(a, 0); }
static int known2(uintptr_t a) { return known(a, 1); }

static int beforeRead(uintptr_t a, unsigned c) {
	if (!known(a, c)) return 0;
	const uint32_t off = (uint32_t)(a - base(c));
	if (off >= STREAM_BASE && ((off - STREAM_BASE) % STREAM_STRIDE) == R_CR) evalStop(c, (off - STREAM_BASE) / STREAM_STRIDE);
	if (off == LISR || off == HISR || off == LIFCR || off == HIFCR) {
		for (unsigned s = 0; s < HOST_DMA_STREAMS; ++s) {
			evalStop(c, s);
			if (st[c][s].readsLeft != 0u && --st[c][s].readsLeft == 0u) hostDmaComplete(c, s);
		}
	}
	return 1;
}
static int beforeRead1(uintptr_t a) { return beforeRead(a, 0); }
static int beforeRead2(uintptr_t a) { return beforeRead(a, 1); }

static void reset(unsigned c) {
	for (unsigned s = 0; s < HOST_DMA_STREAMS; ++s) poke(sreg(c, s, R_FCR), 0x00000021u);
}
static void reset1(void) { reset(0); }
static void reset2(void) { reset(1); }

static void afterWrite(uintptr_t a, uint32_t oldValue, uint32_t written, unsigned c) {
	const uint32_t off = (uint32_t)(a - base(c));
	if (off == LISR || off == HISR) { /* read-only: restore */
		poke(a, oldValue);
		return;
	}
	if (off == LIFCR || off == HIFCR) {
		const uintptr_t r = base(c) + (off == LIFCR ? LISR : HISR);
		poke(r, peek(r) & ~(written & 0x0F7D0F7Du));
		poke(a, 0u);
		return;
	}
	const unsigned s = (off - STREAM_BASE) / STREAM_STRIDE;
	const uint32_t r = (off - STREAM_BASE) % STREAM_STRIDE;
	StreamState *x = &st[c][s];
	evalStop(c, s);
	/* the register holds the raw CPU write now; the old value tells whether the stream was enabled */
	uint32_t crOld = r == R_CR ? oldValue : peek(sreg(c, s, R_CR));
	const int en = (crOld & CR_EN) != 0u;
	switch (r) {
	case R_CR: {
		uint32_t nw;
		if (en) {
			nw = (oldValue & ~CR_ALWAYS) | (written & CR_ALWAYS);
			if (((oldValue ^ written) & CR_WRITABLE & ~CR_ALWAYS & ~CR_CT) != 0u) x->protectedWrites++;
			nw = (nw & ~CR_EN) | (oldValue & CR_EN);
			poke(a, nw);
			if ((written & CR_EN) == 0u) {
				x->stopping = 1;
				x->stopAt = hostTickPeek() + disableDelay;
				if (!x->stick && disableDelay == 0u) softStop(c, s);
			}
		} else {
			nw = written & CR_WRITABLE;
			poke(a, nw & ~CR_EN);
			if (written & CR_EN) {
				poke(a, nw);
				startStream(c, s);
			}
		}
		break;
	}
	case R_NDTR:
		if (en) {
			poke(a, oldValue);
			x->protectedWrites++;
		} else {
			poke(a, written & 0xFFFFu);
		}
		break;
	case R_PAR:
		if (en) {
			poke(a, oldValue);
			x->protectedWrites++;
		} else {
			poke(a, written);
		}
		break;
	case R_M0AR:
	case R_M1AR: {
		const uint32_t cr = peek(sreg(c, s, R_CR));
		const int ct = (cr & CR_CT) != 0u;
		const int dbm = (cr & CR_DBM) != 0u;
		if (!en) {
			poke(a, written);
		} else if (dbm && ((r == R_M0AR && ct) || (r == R_M1AR && !ct))) {
			poke(a, written);
		} else if (dbm) { /* the current target: transfer error (RM0090 10.3.18), write ignored */
			poke(a, oldValue);
			hostDmaRaise(c, s, HOST_DMA_TEIF);
		} else {
			poke(a, oldValue);
			x->protectedWrites++;
		}
		break;
	}
	case R_FCR: {
		uint32_t nw = (oldValue & ~0x87u) | (written & 0x80u);
		if (en) {
			if (((oldValue ^ written) & 0x07u) != 0u) x->protectedWrites++;
			nw |= oldValue & 0x07u;
		} else {
			nw |= written & 0x07u;
		}
		poke(a, (nw & ~0x38u) | 0x20u); /* FS = 100: FIFO empty */
		break;
	}
	default: poke(a, oldValue); break;
	}
}
static void afterWrite1(uintptr_t a, uint32_t o, uint32_t w) { afterWrite(a, o, w, 0); }
static void afterWrite2(uintptr_t a, uint32_t o, uint32_t w) { afterWrite(a, o, w, 1); }

static const HostRegModel dma1Model = {"DMA1", DMA1_BASE, 0x400u, reset1, beforeRead1, NULL, known1, afterWrite1};
static const HostRegModel dma2Model = {"DMA2", DMA2_BASE, 0x400u, reset2, beforeRead2, NULL, known2, afterWrite2};

void hostDmaRegister(void) {
	hostRegsAddModel(&dma1Model);
	hostRegsAddModel(&dma2Model);
}
