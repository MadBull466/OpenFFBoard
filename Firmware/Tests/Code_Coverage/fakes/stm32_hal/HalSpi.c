/*
 * HalSpi.c: SPI1..SPI3 register model (P12 T4a). See HalSpi.h for what is modelled and the RM0090 sources.
 * Register access goes through hostRegsPeek / hostRegsPoke (raw, no trapping).
 */
#include "HalSpi.h"

#include <stddef.h>

#include "HalHostSystem.h"

#define R_CR1 0x00u
#define R_CR2 0x04u
#define R_SR 0x08u
#define R_DR 0x0Cu
#define R_CRCPR 0x10u
#define R_RXCRCR 0x14u
#define R_TXCRCR 0x18u
#define R_I2SCFGR 0x1Cu
#define R_I2SPR 0x20u

#define CR1_CPHA (1u << 0)
#define CR1_MSTR (1u << 2)
#define CR1_SPE (1u << 6)
#define CR1_DFF (1u << 11)
#define CR1_RXONLY (1u << 10)
#define CR1_BIDIMODE (1u << 15)
#define CR1_BIDIOE (1u << 14)
#define CR1_RESTRICTED 0x2CBFu /* CRCEN, DFF, RXONLY, LSBFIRST, BR, MSTR, CPOL, CPHA: not to be changed with SPE = 1 (RM0090 28.5.1) */

#define SR_RXNE (1u << 0)
#define SR_TXE (1u << 1)
#define SR_MODF (1u << 5)
#define SR_OVR (1u << 6)
#define SR_BSY (1u << 7)
#define SR_FRE (1u << 8)

#define PEER_MAX 64u
#define TXLOG_MAX 512u

static const uintptr_t bases[HOST_SPI_COUNT] = {0x40013000u, 0x40003800u, 0x40003C00u};

typedef struct {
	uint8_t busy;
	uint8_t txFull;
	uint16_t txWord;
	uint16_t curTx;
	uint32_t doneAt;
	uint16_t rxWord;
	uint8_t drReadSinceOvr;
	uint8_t srReadSinceModf;
	uint16_t peer[PEER_MAX];
	unsigned peerHead, peerCount;
	uint16_t idle;
	uint8_t loopback, silent;
	uint16_t txLog[TXLOG_MAX];
	unsigned txCount;
	uint32_t configWhileEnabled, disabledBusy;
} SpiState;

static SpiState sp[HOST_SPI_COUNT];
static unsigned frameTicks = 1U;

static uint32_t peek(uintptr_t a) { return hostRegsPeek(a); }
static void poke(uintptr_t a, uint32_t v) { hostRegsPoke(a, v); }
static uintptr_t reg(unsigned i, uint32_t r) { return bases[i] + r; }

void hostSpiResetState(void) {
	for (unsigned i = 0; i < HOST_SPI_COUNT; ++i) sp[i] = (SpiState){0};
	frameTicks = 1U;
}
void hostSpiSetFrameTicks(unsigned t) { frameTicks = t == 0u ? 1u : t; }
void hostSpiPeerPush(unsigned i, uint16_t w) {
	if (i >= HOST_SPI_COUNT || sp[i].peerCount >= PEER_MAX) return;
	sp[i].peer[(sp[i].peerHead + sp[i].peerCount++) % PEER_MAX] = w;
}
void hostSpiPeerSetIdle(unsigned i, uint16_t w) { if (i < HOST_SPI_COUNT) sp[i].idle = w; }
void hostSpiPeerLoopback(unsigned i, int on) { if (i < HOST_SPI_COUNT) sp[i].loopback = on != 0; }
void hostSpiPeerSilent(unsigned i, int s) { if (i < HOST_SPI_COUNT) sp[i].silent = s != 0; }
unsigned hostSpiTxCount(unsigned i) { return i < HOST_SPI_COUNT ? sp[i].txCount : 0u; }
uint16_t hostSpiTxAt(unsigned i, unsigned k) { return (i < HOST_SPI_COUNT && k < sp[i].txCount) ? sp[i].txLog[k] : 0u; }
void hostSpiTxClear(unsigned i) { if (i < HOST_SPI_COUNT) sp[i].txCount = 0; }
uint32_t hostSpiConfigWhileEnabled(unsigned i) { return i < HOST_SPI_COUNT ? sp[i].configWhileEnabled : 0u; }
uint32_t hostSpiDisabledBusy(unsigned i) { return i < HOST_SPI_COUNT ? sp[i].disabledBusy : 0u; }

/* ------------------------------------------------------------------ frame engine */
static uint16_t frameMask(unsigned i) { return (peek(reg(i, R_CR1)) & CR1_DFF) ? 0xFFFFu : 0x00FFu; }
static int master(unsigned i) { return (peek(reg(i, R_CR1)) & CR1_MSTR) != 0u; }
static int enabled(unsigned i) { return (peek(reg(i, R_CR1)) & CR1_SPE) != 0u; }
static int rxOnly(unsigned i) {
	const uint32_t cr1 = peek(reg(i, R_CR1));
	return (cr1 & CR1_BIDIMODE) ? ((cr1 & CR1_BIDIOE) == 0u) : ((cr1 & CR1_RXONLY) != 0u);
}

static void txLogAdd(unsigned i, uint16_t w) {
	if (sp[i].txCount < TXLOG_MAX) sp[i].txLog[sp[i].txCount++] = w;
}

static uint16_t peerNext(unsigned i) {
	if (sp[i].peerCount == 0u) return sp[i].idle;
	const uint16_t w = sp[i].peer[sp[i].peerHead];
	sp[i].peerHead = (sp[i].peerHead + 1u) % PEER_MAX;
	sp[i].peerCount--;
	return w;
}

static void deliver(unsigned i, uint16_t word) {
	uint32_t sr = peek(reg(i, R_SR));
	word &= frameMask(i);
	if (sr & SR_RXNE) {
		sr |= SR_OVR; /* the new word is lost, RM0090 28.3.10 */
		sp[i].drReadSinceOvr = 0;
	} else {
		sp[i].rxWord = word;
		sr |= SR_RXNE;
	}
	poke(reg(i, R_SR), sr);
}

static void startFrame(unsigned i, int hasTx) {
	SpiState *x = &sp[i];
	x->busy = 1;
	x->doneAt = hostTickPeek() + frameTicks;
	if (hasTx) {
		x->curTx = x->txWord & frameMask(i);
		txLogAdd(i, x->curTx);
	} else {
		x->curTx = 0;
	}
	{
		const uint32_t cr1 = peek(reg(i, R_CR1));
		const int bidiRx = (cr1 & CR1_BIDIMODE) != 0u && (cr1 & CR1_BIDIOE) == 0u;
		if (!bidiRx) poke(reg(i, R_SR), (peek(reg(i, R_SR)) | SR_BSY)); /* RM0090 28.3.7: BSY stays low in 1-line receive */
	}
}

/* Brings the frames that ended by now up to date. */
static void spiEval(unsigned i) {
	SpiState *x = &sp[i];
	for (unsigned guard = 0; guard < 1000u && x->busy && !x->silent && (int32_t)(hostTickPeek() - x->doneAt) >= 0; ++guard) {
		x->busy = 0;
		deliver(i, x->loopback ? x->curTx : peerNext(i));
		uint32_t sr = peek(reg(i, R_SR)) & ~SR_BSY;
		poke(reg(i, R_SR), sr);
		if (!enabled(i) || !master(i)) break;
		if (x->txFull) { /* the next frame starts from the Tx buffer */
			x->txFull = 0;
			poke(reg(i, R_SR), peek(reg(i, R_SR)) | SR_TXE);
			startFrame(i, 1);
		} else if (rxOnly(i)) { /* free running clock */
			startFrame(i, 0);
		}
	}
}

/* ------------------------------------------------------------------ bus callbacks */
static int known(unsigned i, uintptr_t a) {
	const uint32_t off = (uint32_t)(a - bases[i]);
	return off <= R_I2SPR && (off & 3u) == 0u;
}
static int beforeRead(unsigned i, uintptr_t a) {
	if (!known(i, a)) return 0;
	const uint32_t off = (uint32_t)(a - bases[i]);
	if (off == R_SR || off == R_DR) spiEval(i);
	if (off == R_DR) poke(a, sp[i].rxWord & frameMask(i)); /* a read returns the Rx buffer */
	return 1;
}
static void afterRead(unsigned i, uintptr_t a, uint32_t value) {
	(void)value;
	const uint32_t off = (uint32_t)(a - bases[i]);
	if (off == R_DR) {
		poke(a, 0u);
		sp[i].drReadSinceOvr = 1;
		poke(reg(i, R_SR), peek(reg(i, R_SR)) & ~SR_RXNE);
		if (enabled(i) && master(i) && rxOnly(i) && !sp[i].busy) startFrame(i, 0); /* free running: next frame */
	} else if (off == R_SR) {
		if (sp[i].drReadSinceOvr && (peek(a) & SR_OVR)) poke(a, peek(a) & ~SR_OVR);
		sp[i].drReadSinceOvr = 0;
		if (peek(a) & SR_MODF) sp[i].srReadSinceModf = 1;
		poke(a, peek(a) & ~SR_FRE); /* FRE clears when SR is read (28.5.3) */
	}
}

static void afterWrite(unsigned i, uintptr_t a, uint32_t oldValue, uint32_t written) {
	const uint32_t off = (uint32_t)(a - bases[i]);
	SpiState *x = &sp[i];
	switch (off) {
	case R_CR1: {
		const uint32_t nw = written & 0xFFFFu;
		spiEval(i);
		const int wasOn = (oldValue & CR1_SPE) != 0u, isOn = (nw & CR1_SPE) != 0u;
		if (wasOn && isOn && ((oldValue ^ nw) & CR1_RESTRICTED) != 0u) x->configWhileEnabled++;
		poke(a, nw);
		if (wasOn && !isOn) { /* disable */
			if (x->busy) x->disabledBusy++;
			x->busy = 0;
			x->txFull = 0;
			poke(reg(i, R_SR), (peek(reg(i, R_SR)) & ~SR_BSY) | SR_TXE);
		}
		if (!wasOn && isOn && (nw & CR1_MSTR)) {
			if (x->txFull) {
				x->txFull = 0;
				poke(reg(i, R_SR), peek(reg(i, R_SR)) | SR_TXE);
				startFrame(i, 1);
			} else if (rxOnly(i)) {
				startFrame(i, 0);
			}
		}
		if (x->srReadSinceModf) {
			poke(reg(i, R_SR), peek(reg(i, R_SR)) & ~SR_MODF);
			x->srReadSinceModf = 0;
		}
		break;
	}
	case R_CR2: poke(a, written & 0x00F7u); break;
	case R_SR: poke(a, oldValue); break; /* flags are hardware set; CRCERR (write 0) is not modelled */
	case R_DR: {
		const uint16_t w = (uint16_t)(written & frameMask(i));
		poke(a, 0u);
		spiEval(i);
		if (enabled(i) && master(i) && !x->busy && !x->txFull) {
			x->txWord = w;
			startFrame(i, 1);
		} else {
			x->txWord = w;
			x->txFull = 1;
			poke(reg(i, R_SR), peek(reg(i, R_SR)) & ~SR_TXE);
		}
		break;
	}
	case R_CRCPR: poke(a, written & 0xFFFFu); break;
	case R_RXCRCR:
	case R_TXCRCR: poke(a, 0u); break;
	case R_I2SCFGR: poke(a, written & 0x0FBFu); break;
	case R_I2SPR: poke(a, written & 0x03FFu); break;
	default: poke(a, oldValue); break;
	}
}

uint16_t hostSpiSlaveFrame(unsigned i, uint16_t word) {
	if (i >= HOST_SPI_COUNT || !enabled(i) || master(i)) return 0u;
	uint16_t out = 0u;
	if (sp[i].txFull) {
		out = sp[i].txWord & frameMask(i);
		txLogAdd(i, out);
		sp[i].txFull = 0;
		poke(reg(i, R_SR), peek(reg(i, R_SR)) | SR_TXE);
	}
	deliver(i, word);
	return out;
}
void hostSpiRaiseModf(unsigned i) {
	if (i >= HOST_SPI_COUNT) return;
	poke(reg(i, R_SR), (peek(reg(i, R_SR)) | SR_MODF) & ~SR_BSY);
	poke(reg(i, R_CR1), peek(reg(i, R_CR1)) & ~(CR1_MSTR | CR1_SPE));
	sp[i].busy = 0;
}
void hostSpiRaiseFre(unsigned i) {
	if (i < HOST_SPI_COUNT) poke(reg(i, R_SR), peek(reg(i, R_SR)) | SR_FRE);
}

static void resetAll(unsigned i) {
	poke(reg(i, R_SR), SR_TXE);
	poke(reg(i, R_CRCPR), 0x0007u);
	poke(reg(i, R_I2SPR), 0x0002u);
}

#define SPI_THUNKS(N)                                                                                \
	static int known##N(uintptr_t a) { return known(N, a); }                                         \
	static int beforeRead##N(uintptr_t a) { return beforeRead(N, a); }                               \
	static void afterRead##N(uintptr_t a, uint32_t v) { afterRead(N, a, v); }                        \
	static void afterWrite##N(uintptr_t a, uint32_t o, uint32_t w) { afterWrite(N, a, o, w); }       \
	static void reset##N(void) { resetAll(N); }
SPI_THUNKS(0)
SPI_THUNKS(1)
SPI_THUNKS(2)

static const HostRegModel spiModels[HOST_SPI_COUNT] = {
	{"SPI1", 0x40013000u, 0x400u, reset0, beforeRead0, afterRead0, known0, afterWrite0},
	{"SPI2", 0x40003800u, 0x400u, reset1, beforeRead1, afterRead1, known1, afterWrite1},
	{"SPI3", 0x40003C00u, 0x400u, reset2, beforeRead2, afterRead2, known2, afterWrite2},
};

void hostSpiRegister(void) {
	for (unsigned i = 0; i < HOST_SPI_COUNT; ++i) hostRegsAddModel(&spiModels[i]);
}
