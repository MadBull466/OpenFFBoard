/*
 * HalOtg.c: USB OTG full-speed core model, device mode (P12 T8). See HalOtg.h for what is modelled and the RM0090 sources.
 */
#include "HalOtg.h"

#include <stddef.h>
#include <string.h>

#define BASE HOST_OTG_BASE
#define SIZE 0x40000u

/* global */
#define R_GOTGCTL 0x000u
#define R_GOTGINT 0x004u
#define R_GAHBCFG 0x008u
#define R_GUSBCFG 0x00Cu
#define R_GRSTCTL 0x010u
#define R_GINTSTS 0x014u
#define R_GINTMSK 0x018u
#define R_GRXSTSR 0x01Cu
#define R_GRXSTSP 0x020u
#define R_GRXFSIZ 0x024u
#define R_DIEPTXF0 0x028u
#define R_HNPTXSTS 0x02Cu
#define R_GCCFG 0x038u
#define R_CID 0x03Cu
#define R_HPTXFSIZ 0x100u
#define R_DIEPTXF1 0x104u /* 0x104, 0x108, 0x10C */
/* device */
#define R_DCFG 0x800u
#define R_DCTL 0x804u
#define R_DSTS 0x808u
#define R_DIEPMSK 0x810u
#define R_DOEPMSK 0x814u
#define R_DAINT 0x818u
#define R_DAINTMSK 0x81Cu
#define R_DVBUSDIS 0x828u
#define R_DVBUSPULSE 0x82Cu
#define R_DIEPEMPMSK 0x834u
#define R_PCGCCTL 0xE00u
#define R_INEP 0x900u
#define R_OUTEP 0xB00u
#define EP_COUNT 4u

/* GINTSTS bits */
#define GI_MMIS (1u << 1)
#define GI_SOF (1u << 3)
#define GI_RXFLVL (1u << 4)
#define GI_GINAKEFF (1u << 6)
#define GI_GONAKEFF (1u << 7)
#define GI_ESUSP (1u << 10)
#define GI_USBSUSP (1u << 11)
#define GI_USBRST (1u << 12)
#define GI_ENUMDNE (1u << 13)
#define GI_IEPINT (1u << 18)
#define GI_OEPINT (1u << 19)
#define GI_DISCINT (1u << 29)
#define GI_SRQINT (1u << 30)
#define GI_WKUPINT (1u << 31)
#define GINTSTS_W1C 0xF030FC0Au /* MMIS, SOF, ESUSP..EOPF (10-15), IISOIXFR, IPXFR (20, 21), CIDSCHG (28), DISCINT, SRQINT, WKUPINT */
#define GINTMSK_MASK 0xF73CFCFEu
#define GOTGINT_W1C 0x000E0304u

/* endpoint control bits */
#define C_EPENA (1u << 31)
#define C_EPDIS (1u << 30)
#define C_SODDFRM_SD1PID (1u << 29)
#define C_SD0PID (1u << 28)
#define C_SNAK (1u << 27)
#define C_CNAK (1u << 26)
#define C_STALL (1u << 21)
#define C_NAKSTS (1u << 17)
#define C_USBAEP (1u << 15)

/* DIEPINT / DOEPINT bits */
#define I_XFRC (1u << 0)
#define I_EPDISD (1u << 1)
#define I_STUP (1u << 3)
#define I_ITTXFE (1u << 4)
#define I_OTEPDIS (1u << 4)
#define I_INEPNE (1u << 6)
#define I_TXFE (1u << 7)
#define I_NAK (1u << 13)

#define RAM_WORDS 320u /* RM0090 34.13: 1.25 KiB FIFO RAM */
#define RX_WORDS 1024u
#define TX_WORDS 1024u

enum { K_DATA = 0, K_STATUS, K_OUT_COMPLETE, K_SETUP_COMPLETE };

typedef struct {
	uint32_t off, reset, wmask, w1c;
} Def;

static const Def defs[] = {
	{R_GOTGCTL, 0x00010000u, 0x00000E02u, 0},
	{R_GOTGINT, 0, 0, GOTGINT_W1C},
	{R_GAHBCFG, 0, 0x00000181u, 0},
	{R_GUSBCFG, 0x00000A40u, 0xE0003F07u, 0},
	{R_GINTMSK, 0, GINTMSK_MASK, 0},
	{R_GRXFSIZ, 0x00000200u, 0x0000FFFFu, 0},
	{R_DIEPTXF0, 0x00000200u, 0xFFFFFFFFu, 0},
	{R_GCCFG, 0, 0x003D0000u, 0},
	{R_CID, 0x00001200u, 0, 0},
	{R_HPTXFSIZ, 0x02000400u, 0xFFFFFFFFu, 0},
	{R_DIEPTXF1, 0x02000200u, 0xFFFFFFFFu, 0},
	{R_DIEPTXF1 + 4u, 0x02000200u, 0xFFFFFFFFu, 0},
	{R_DIEPTXF1 + 8u, 0x02000200u, 0xFFFFFFFFu, 0},
	{R_DCFG, 0x02200000u, 0x00001FF7u, 0},
	{R_DSTS, 0x00000010u, 0, 0},
	{R_DIEPMSK, 0, 0x0000207Bu, 0},
	{R_DOEPMSK, 0, 0x0000313Bu, 0},
	{R_DAINTMSK, 0, 0x000F000Fu, 0},
	{R_DVBUSDIS, 0x000017D7u, 0x0000FFFFu, 0},
	{R_DVBUSPULSE, 0x000005B8u, 0x00000FFFu, 0},
	{R_DIEPEMPMSK, 0, 0x0000000Fu, 0},
	{R_PCGCCTL, 0, 0x00000003u, 0},
};
enum { DEF_COUNT = sizeof(defs) / sizeof(defs[0]) };

static struct {
	uint32_t rx[RX_WORDS];
	uint8_t rxKind[RX_WORDS];
	unsigned rxHead, rxCount, rxStatusCount;
	uint32_t tx[EP_COUNT][TX_WORDS];
	unsigned txCount[EP_COUNT];
	unsigned pollReads, pollCount;
	uint32_t reserved, overflows, underflows, rejects, duplicates;
	uint8_t outPid[EP_COUNT]; /* last accepted DATA PID of an OUT endpoint within the armed transfer: 0 none, 1 DATA0, 2 DATA1 (RM0090 35.17: a repeated PID is ACKed and the data discarded) */
	uint32_t reservedOff[16], hsOnly, hsOff[32];
	uint8_t hsWrite[32];
	uint8_t reservedWrite[16];
	uint32_t layoutFaults;
	int strict, holdDisable;
} os = {{0}, {0}, 0, 0, 0, {{0}}, {0}, 1, 0, 0, 0, 0, 0, {0}, 0, {0}, {0}, {0}, 0, 1, 0};

static uint32_t reg(uint32_t o) { return hostRegsPeek(BASE + o); }
static void put(uint32_t o, uint32_t v) { hostRegsPoke(BASE + o, v); }

/* ---------------------------------------------------------------- state helpers */
static unsigned txDepth(unsigned fifo) {
	if (fifo >= EP_COUNT) return 0;
	const uint32_t v = reg(fifo == 0u ? R_DIEPTXF0 : R_DIEPTXF1 + 4u * (fifo - 1u));
	unsigned d = v >> 16;
	return d > TX_WORDS ? TX_WORDS : d;
}
static unsigned rxDepth(void) {
	unsigned d = reg(R_GRXFSIZ) & 0xFFFFu;
	return d > RX_WORDS ? RX_WORDS : d;
}
static uint32_t inCtlOff(unsigned e) { return R_INEP + 0x20u * e; }
static uint32_t outCtlOff(unsigned e) { return R_OUTEP + 0x20u * e; }

static unsigned maxPacket(uint32_t ctl, unsigned e) {
	if (e == 0u) {
		static const unsigned mp[4] = {64, 32, 16, 8};
		return mp[ctl & 3u];
	}
	return ctl & 0x7FFu;
}

static int layoutOk(void) {
	if ((reg(R_GRXFSIZ) & 0xFFFFu) > RAM_WORDS) return 0;
	for (unsigned f = 0; f < EP_COUNT; ++f) {
		const uint32_t v = reg(f == 0u ? R_DIEPTXF0 : R_DIEPTXF1 + 4u * (f - 1u));
		if ((v & 0xFFFFu) + (v >> 16) > RAM_WORDS) return 0;
	}
	return 1;
}
static void checkLayout(void) {
	if (!layoutOk() && os.layoutFaults++ == 0u) hostRegsAddFault("OTG FIFO layout exceeds the 320 word FIFO RAM (RM0090 34.13.1)", BASE + R_GRXFSIZ, reg(R_GRXFSIZ));
}
static void rxPush(uint32_t word, uint8_t kind) {
	const unsigned at = (os.rxHead + os.rxCount) % RX_WORDS;
	os.rx[at] = word;
	os.rxKind[at] = kind;
	os.rxCount++;
	if (kind != K_DATA) os.rxStatusCount++;
}
static int rxPop(uint32_t *word, uint8_t *kind) {
	if (os.rxCount == 0u) return 0;
	*word = os.rx[os.rxHead];
	*kind = os.rxKind[os.rxHead];
	os.rxHead = (os.rxHead + 1u) % RX_WORDS;
	os.rxCount--;
	if (*kind != K_DATA) os.rxStatusCount--;
	return 1;
}

static void flushTx(unsigned num) {
	if (num == 16u) {
		memset(os.txCount, 0, sizeof os.txCount);
	} else if (num < EP_COUNT) {
		os.txCount[num] = 0;
	}
}
static void flushRx(void) {
	os.rxHead = os.rxCount = os.rxStatusCount = 0;
}

static int txfe(unsigned e) {
	const unsigned fifo = (reg(inCtlOff(e)) >> 22) & 0xFu;
	if (fifo >= EP_COUNT) return 1;
	const unsigned depth = txDepth(fifo);
	const unsigned used = os.txCount[fifo];
	if (reg(R_GAHBCFG) & (1u << 7)) return used == 0u;
	return used * 2u <= depth;
}

/* Recomputes every register that is derived from the state of the core. */
static void refresh(void) {
	uint32_t daint = 0;
	const uint32_t diepmsk = reg(R_DIEPMSK), doepmsk = reg(R_DOEPMSK), empmsk = reg(R_DIEPEMPMSK);
	for (unsigned e = 0; e < EP_COUNT; ++e) {
		uint32_t in = reg(inCtlOff(e) + 8u);
		in = txfe(e) ? (in | I_TXFE) : (in & ~I_TXFE);
		put(inCtlOff(e) + 8u, in);
		uint32_t m = diepmsk;
		if (empmsk & (1u << e)) m |= I_TXFE;
		if (in & m) daint |= 1u << e;
		if (reg(outCtlOff(e) + 8u) & doepmsk) daint |= 1u << (16u + e);
		const unsigned fifo = (reg(inCtlOff(e)) >> 22) & 0xFu;
		const unsigned depth = fifo < EP_COUNT ? txDepth(fifo) : 0u;
		const unsigned used = fifo < EP_COUNT ? os.txCount[fifo] : 0u;
		put(inCtlOff(e) + 0x18u, depth > used ? depth - used : 0u);
	}
	put(R_DAINT, daint);
	put(R_OUTEP, (reg(R_OUTEP) & ~3u) | (reg(R_INEP) & 3u)); /* RM0090: OUT endpoint 0 packet size is the one of IN endpoint 0 */
	const uint32_t daintmsk = reg(R_DAINTMSK);
	uint32_t gi = reg(R_GINTSTS);
	gi &= ~(GI_RXFLVL | GI_IEPINT | GI_OEPINT);
	if (os.rxCount != 0u) gi |= GI_RXFLVL;
	if (daint & daintmsk & 0xFFFFu) gi |= GI_IEPINT;
	if (daint & daintmsk & 0xFFFF0000u) gi |= GI_OEPINT;
	put(R_GINTSTS, gi);
	{
		const unsigned depth = txDepth(0), used = os.txCount[0];
		put(R_HNPTXSTS, 0x00080000u | (depth > used ? depth - used : 0u));
	}
	{
		uint32_t top = 0;
		if (os.rxCount != 0u) top = os.rx[os.rxHead];
		put(R_GRXSTSR, top);
	}
}

static void setGint(uint32_t bits) { put(R_GINTSTS, reg(R_GINTSTS) | bits); }
static void setIn(unsigned e, uint32_t bits) { put(inCtlOff(e) + 8u, reg(inCtlOff(e) + 8u) | bits); }
static void setOut(unsigned e, uint32_t bits) { put(outCtlOff(e) + 8u, reg(outCtlOff(e) + 8u) | bits); }

/* Power-on / core soft reset values of every register. */
static void resetRegisters(int keepSpeed) {
	const uint32_t dspd = reg(R_DCFG) & 3u;
	const uint32_t pcg = reg(R_PCGCCTL);
	for (unsigned i = 0; i < DEF_COUNT; ++i) put(defs[i].off, defs[i].reset);
	put(R_GRSTCTL, 0x80000000u);
	put(R_GINTSTS, 0x04000020u);
	put(R_GRXSTSP, 0u);
	put(R_HNPTXSTS, 0x00080200u);
	for (unsigned e = 0; e < EP_COUNT; ++e) {
		put(inCtlOff(e), 0u);
		put(inCtlOff(e) + 8u, I_TXFE);
		put(inCtlOff(e) + 0x10u, 0u);
		put(outCtlOff(e), e == 0u ? 0x00008000u : 0u);
		put(outCtlOff(e) + 8u, 0x80u);
		put(outCtlOff(e) + 0x10u, 0u);
	}
	if (keepSpeed) {
		put(R_DCFG, (reg(R_DCFG) & ~3u) | dspd);
		put(R_PCGCCTL, pcg);
	}
	flushRx();
	memset(os.txCount, 0, sizeof os.txCount);
	refresh();
}

void hostOtgResetState(void) {
	memset(&os, 0, sizeof os);
	os.pollReads = 1;
	os.strict = 1;
}
void hostOtgSetPollReads(unsigned n) {
	os.pollReads = n;
	os.pollCount = 0;
}
uint32_t hostOtgReservedAccesses(void) { return os.reserved; }
uint32_t hostOtgReservedOffset(unsigned i) { return i < 16u && i < os.reserved ? os.reservedOff[i] : 0xFFFFFFFFu; }
int hostOtgReservedWasWrite(unsigned i) { return i < 16u && i < os.reserved ? os.reservedWrite[i] : 0; }
static int hsOnlyOffset(uint32_t off) {
	if (off == 0x040u || off == 0x054u || (off >= 0x110u && off <= 0x13Cu) || off == 0x838u || off == 0x83Cu || off == 0x844u || off == 0x884u) return 1;
	if (off >= 0x900u && off < 0xD00u && (off & 0x1Fu) == 0x14u) return 1; /* DIEPDMA / DOEPDMA */
	return 0;
}
uint32_t hostOtgHsOnlyAccesses(void) { return os.hsOnly; }
uint32_t hostOtgHsOnlyOffset(unsigned i) { return i < 32u && i < os.hsOnly ? os.hsOff[i] : 0xFFFFFFFFu; }
int hostOtgHsOnlyWasWrite(unsigned i) { return i < 32u && i < os.hsOnly ? os.hsWrite[i] : 0; }
static void noteReserved(uint32_t off, int write) {
	if (hsOnlyOffset(off)) {
		if (os.hsOnly < 32u) {
			os.hsOff[os.hsOnly] = off;
			os.hsWrite[os.hsOnly] = (uint8_t)write;
		}
		os.hsOnly++;
		return;
	}
	if (os.reserved < 16u) {
		os.reservedOff[os.reserved] = off;
		os.reservedWrite[os.reserved] = (uint8_t)write;
	}
	os.reserved++;
}
uint32_t hostOtgFifoOverflows(void) { return os.overflows; }
uint32_t hostOtgFifoUnderflows(void) { return os.underflows; }
uint32_t hostOtgRxRejects(void) { return os.rejects; }
uint32_t hostOtgDuplicatePackets(void) { return os.duplicates; }
void hostOtgStrictFifo(int strict) { os.strict = strict != 0; }
unsigned hostOtgRxWordsUsed(void) { return os.rxCount; }
unsigned hostOtgRxEntries(void) { return os.rxStatusCount; }
unsigned hostOtgTxWordsUsed(unsigned fifo) { return fifo < EP_COUNT ? os.txCount[fifo] : 0u; }
const uint32_t *hostOtgTxWords(unsigned fifo) { return fifo < EP_COUNT ? os.tx[fifo] : NULL; }
void hostOtgRxFill(uint32_t value, unsigned words) {
	for (unsigned i = 0; i < words && os.rxCount < RX_WORDS; ++i) rxPush(value, K_DATA);
	refresh();
}
int hostOtgIrqPending(void) {
	return (reg(R_GAHBCFG) & 1u) != 0u && (reg(R_GINTSTS) & reg(R_GINTMSK)) != 0u;
}

/* ---------------------------------------------------------------- address classes */
typedef enum { A_RESERVED, A_DEF, A_GRSTCTL, A_GINTSTS, A_GRXSTSR, A_GRXSTSP, A_HNPTXSTS, A_DCTL, A_DAINT, A_INCTL, A_ININT, A_INTSIZ, A_DTXFSTS, A_OUTCTL, A_OUTINT, A_OUTTSIZ, A_FIFO } Class;

static const Def *findDef(uint32_t off) {
	for (unsigned i = 0; i < DEF_COUNT; ++i)
		if (defs[i].off == off) return &defs[i];
	return NULL;
}

static Class classify(uint32_t off, unsigned *ep) {
	*ep = 0;
	if (off == R_GRSTCTL) return A_GRSTCTL;
	if (off == R_GINTSTS) return A_GINTSTS;
	if (off == R_GRXSTSR) return A_GRXSTSR;
	if (off == R_GRXSTSP) return A_GRXSTSP;
	if (off == R_HNPTXSTS) return A_HNPTXSTS;
	if (off == R_DCTL) return A_DCTL;
	if (off == R_DAINT) return A_DAINT;
	if (findDef(off) != NULL) return A_DEF;
	if (off >= R_INEP && off < R_INEP + 0x20u * EP_COUNT) {
		*ep = (off - R_INEP) / 0x20u;
		switch ((off - R_INEP) % 0x20u) {
		case 0x00: return A_INCTL;
		case 0x08: return A_ININT;
		case 0x10: return A_INTSIZ;
		case 0x18: return A_DTXFSTS;
		default: return A_RESERVED;
		}
	}
	if (off >= R_OUTEP && off < R_OUTEP + 0x20u * EP_COUNT) {
		*ep = (off - R_OUTEP) / 0x20u;
		switch ((off - R_OUTEP) % 0x20u) {
		case 0x00: return A_OUTCTL;
		case 0x08: return A_OUTINT;
		case 0x10: return A_OUTTSIZ;
		default: return A_RESERVED;
		}
	}
	if (off >= 0x1000u && off < 0x1000u * (EP_COUNT + 1u)) {
		*ep = off / 0x1000u - 1u;
		return A_FIFO;
	}
	return A_RESERVED;
}

/* ---------------------------------------------------------------- reads */
static void complete(void) {
	const uint32_t r = reg(R_GRSTCTL);
	if (r & 1u) resetRegisters(1);
	const uint32_t after = reg(R_GRSTCTL);
	if (r & (1u << 5)) flushTx((r >> 6) & 0x1Fu);
	if (r & (1u << 4)) flushRx();
	put(R_GRSTCTL, (after & ~((1u << 0) | (1u << 1) | (1u << 4) | (1u << 5))) | 0x80000000u);
	refresh();
}

static int beforeRead(uintptr_t a) {
	const uint32_t off = (uint32_t)(a - BASE);
	unsigned ep;
	switch (classify(off, &ep)) {
	case A_GRSTCTL:
		if ((reg(R_GRSTCTL) & 0x33u) != 0u && os.pollReads != 0u && ++os.pollCount >= os.pollReads) {
			os.pollCount = 0;
			complete();
		}
		return 1;
	case A_GRXSTSP: {
		uint32_t w = 0;
		uint8_t k = K_DATA;
		if (os.rxCount != 0u) rxPop(&w, &k);
		hostRegsPoke(a, w);
		if (k == K_OUT_COMPLETE) setOut(w & 0xFu, I_XFRC);
		if (k == K_SETUP_COMPLETE) setOut(w & 0xFu, I_STUP);
		refresh();
		return 1;
	}
	case A_FIFO: {
		uint32_t w = 0;
		uint8_t k = K_DATA;
		if (os.rxCount == 0u || os.rxKind[os.rxHead] != K_DATA) {
			os.underflows++;
			if (os.strict) hostRegsAddFault("OTG FIFO read with no data word pending", a, 0);
		} else {
			rxPop(&w, &k);
		}
		hostRegsPoke(a, w);
		refresh();
		return 1;
	}
	case A_RESERVED:
		noteReserved(off, 0);
		hostRegsPoke(a, 0u);
		return 1;
	default:
		return 1;
	}
}

/* ---------------------------------------------------------------- writes */
static void inCtlWrite(unsigned e, uint32_t old, uint32_t w) {
	const uint32_t o = inCtlOff(e);
	uint32_t v = old;
	const uint32_t rw = (e == 0u ? 0x03C00003u : (0x000C0000u | C_USBAEP | 0x7FFu | 0x03C00000u)) | C_STALL;
	v = (old & ~rw) | (w & rw);
	if (e == 0u) v = (v & ~(0x000C0000u | C_USBAEP)) | C_USBAEP; /* EPTYP = control and USBAEP are read only */
	v = (v & ~C_NAKSTS) | (old & C_NAKSTS);
	v &= ~(C_EPENA | C_EPDIS | C_SODDFRM_SD1PID | C_SD0PID | C_SNAK | C_CNAK);
	v |= old & (C_EPENA | C_EPDIS);
	if (w & C_EPENA) v |= C_EPENA;
	if (w & C_CNAK) v &= ~C_NAKSTS;
	if (w & C_SNAK) {
		v |= C_NAKSTS;
		put(o + 8u, reg(o + 8u) | I_INEPNE);
	}
	if ((w & C_EPDIS) && (v & C_EPENA)) {
		if (os.holdDisable) {
			v |= C_EPDIS; /* the core has not finished: EPENA and EPDIS stay set until hostOtgReleaseDisable */
		} else {
			v &= ~C_EPENA;
			put(o + 8u, reg(o + 8u) | I_EPDISD);
		}
	}
	put(o, v);
}
static void outCtlWrite(unsigned e, uint32_t old, uint32_t w) {
	const uint32_t o = outCtlOff(e);
	const uint32_t rw = (e == 0u ? 0x00100000u : (0x000C0000u | 0x00100000u | C_USBAEP | 0x7FFu)) | C_STALL;
	uint32_t v = (old & ~rw) | (w & rw);
	if (e == 0u) v = (v & ~(0x000C0000u | C_USBAEP)) | C_USBAEP;
	v = (v & ~C_NAKSTS) | (old & C_NAKSTS);
	v &= ~(C_EPENA | C_EPDIS | C_SODDFRM_SD1PID | C_SD0PID | C_SNAK | C_CNAK);
	v |= old & (C_EPENA | C_EPDIS);
	if (w & C_EPENA) v |= C_EPENA;
	if ((w & C_EPENA) && !(old & C_EPENA)) os.outPid[e] = 0; /* a newly armed transfer: no PID seen yet */
	if (w & C_SD0PID) os.outPid[e] = 2; /* the next packet is expected to be DATA0 */
	if (w & C_SODDFRM_SD1PID) os.outPid[e] = 1; /* the next packet is expected to be DATA1 */
	if (w & C_CNAK) v &= ~C_NAKSTS;
	if (w & C_SNAK) v |= C_NAKSTS;
	if ((w & C_EPDIS) && (v & C_EPENA)) {
		if (os.holdDisable) {
			v |= C_EPDIS;
		} else {
			v &= ~C_EPENA;
			put(o + 8u, reg(o + 8u) | I_EPDISD);
		}
	}
	put(o, v);
}

static void afterWrite(uintptr_t a, uint32_t old, uint32_t w) {
	const uint32_t off = (uint32_t)(a - BASE);
	unsigned ep;
	const Class c = classify(off, &ep);
	switch (c) {
	case A_DEF: {
		const Def *d = findDef(off);
		uint32_t v = (old & ~(d->wmask | d->w1c)) | (w & d->wmask);
		v = (v & ~d->w1c) | (old & d->w1c & ~w);
		if (off == R_GUSBCFG) v |= 0x40u; /* PHYSEL reads 1 */
		if (off == R_PCGCCTL) v |= old & 0x10u;
		hostRegsPoke(a, v);
		break;
	}
	case A_GRSTCTL: {
		hostRegsPoke(a, 0x80000000u | (w & 0x7C0u) | (old & 0x33u) | (w & 0x33u));
		break;
	}
	case A_GINTSTS: {
		uint32_t v = old & ~(w & GINTSTS_W1C);
		hostRegsPoke(a, v);
		break;
	}
	case A_GRXSTSR:
	case A_GRXSTSP:
	case A_HNPTXSTS:
	case A_DAINT:
	case A_DTXFSTS:
		hostRegsPoke(a, old); /* read only */
		break;
	case A_DCTL: {
		uint32_t v = (old & 0x0000000Cu) | (w & 0x00000873u);
		v = (v & ~0x0Cu) | (old & 0x0Cu);
		uint32_t gi = reg(R_GINTSTS);
		if (w & (1u << 7)) { /* SGINAK */
			v |= 1u << 2;
			gi |= GI_GINAKEFF;
		}
		if (w & (1u << 8)) { /* CGINAK */
			v &= ~(1u << 2);
			gi &= ~GI_GINAKEFF;
		}
		if (w & (1u << 9)) { /* SGONAK */
			v |= 1u << 3;
			gi |= GI_GONAKEFF;
		}
		if (w & (1u << 10)) { /* CGONAK */
			v &= ~(1u << 3);
			gi &= ~GI_GONAKEFF;
		}
		put(R_GINTSTS, gi);
		hostRegsPoke(a, v);
		break;
	}
	case A_INCTL:
		inCtlWrite(ep, old, w);
		break;
	case A_ININT: {
		uint32_t cleared = w & (I_XFRC | I_EPDISD | (1u << 3) | I_ITTXFE | (1u << 5) | I_INEPNE | (1u << 11) | I_NAK);
		hostRegsPoke(a, old & ~cleared);
		break;
	}
	case A_INTSIZ:
		hostRegsPoke(a, w & (ep == 0u ? 0x0018007Fu : 0x1FFFFFFFu)); /* bits 30:29 are reserved (MCNT is not in the FS core text) */
		break;
	case A_OUTCTL:
		outCtlWrite(ep, old, w);
		break;
	case A_OUTINT: {
		uint32_t cleared = w & (I_XFRC | I_EPDISD | I_STUP | I_OTEPDIS | (1u << 5) | (1u << 8) | (1u << 12) | I_NAK);
		hostRegsPoke(a, old & ~cleared);
		break;
	}
	case A_OUTTSIZ:
		hostRegsPoke(a, w & (ep == 0u ? 0x6008007Fu : 0x1FFFFFFFu));
		break;
	case A_FIFO: {
		hostRegsPoke(a, 0u);
		checkLayout();
		if (os.txCount[ep] >= txDepth(ep)) {
			os.overflows++;
			if (os.strict) hostRegsAddFault("OTG TxFIFO write with the FIFO full", a, w);
		} else {
			os.tx[ep][os.txCount[ep]++] = w;
		}
		break;
	}
	case A_RESERVED:
		noteReserved(off, 1);
		hostRegsPoke(a, 0u);
		break;
	}
	refresh();
}

static int beforeWrite(uintptr_t a) {
	(void)a;
	return 1;
}

static void reset(void) {
	resetRegisters(0);
	put(R_GOTGCTL, 0x00010000u);
}

static HostRegModel otgModel = {"OTG_FS", BASE, SIZE, reset, beforeRead, NULL, beforeWrite, afterWrite};
void hostOtgRegister(void) { hostRegsAddModel(&otgModel); }

/* ---------------------------------------------------------------- the host */
static int disconnected(void) { return (reg(R_DCTL) & 2u) != 0u; }
static unsigned words(unsigned bytes) { return (bytes + 3u) / 4u; }

static uint32_t rxStatus(unsigned pktsts, unsigned dpid1, unsigned bcnt, unsigned ep) {
	return (pktsts << 17) | (dpid1 ? (2u << 15) : 0u) | ((bcnt & 0x7FFu) << 4) | (ep & 0xFu);
}
static unsigned rxFree(void) {
	const unsigned d = rxDepth();
	return d > os.rxCount ? d - os.rxCount : 0u;
}

HostOtgHandshake hostOtgHostSetup(const uint8_t *data8) {
	if (disconnected()) return HOSTOTG_IGNORED;
	checkLayout();
	if (rxFree() < 4u) {
		os.rejects++;
		return HOSTOTG_NAK;
	}
	rxPush(rxStatus(6u, 0u, 8u, 0u), K_STATUS);
	for (unsigned i = 0; i < 2u; ++i) {
		uint32_t w = 0;
		for (unsigned b = 0; b < 4u; ++b) w |= (uint32_t)data8[4u * i + b] << (8u * b);
		rxPush(w, K_DATA);
	}
	rxPush(rxStatus(4u, 0u, 0u, 0u), K_SETUP_COMPLETE);
	{
		const uint32_t ts = reg(outCtlOff(0) + 0x10u);
		const uint32_t cnt = (ts >> 29) & 3u;
		put(outCtlOff(0) + 0x10u, (ts & ~(3u << 29)) | ((cnt != 0u ? cnt - 1u : 0u) << 29));
		put(outCtlOff(0), reg(outCtlOff(0)) | C_NAKSTS);
	}
	refresh();
	return HOSTOTG_ACK;
}

HostOtgHandshake hostOtgHostOut(unsigned ep, const uint8_t *data, unsigned len, int pid1) {
	if (ep >= EP_COUNT || disconnected()) return HOSTOTG_IGNORED;
	const uint32_t o = outCtlOff(ep);
	const uint32_t ctl = reg(o);
	if (!(ctl & C_USBAEP)) return HOSTOTG_IGNORED;
	if (ctl & C_STALL) return HOSTOTG_STALL;
	if ((reg(R_DCTL) & (1u << 3)) != 0u || (ctl & C_NAKSTS)) {
		setOut(ep, I_NAK);
		refresh();
		return HOSTOTG_NAK;
	}
	checkLayout();
	if (!(ctl & C_EPENA)) {
		if (ep == 0u) setOut(ep, I_OTEPDIS); /* RM0090 DOEPINT bit 4: applies only to control OUT endpoints */
		else setOut(ep, I_NAK);
		refresh();
		return HOSTOTG_NAK;
	}
	if (ep != 0u && os.outPid[ep] == (pid1 ? 2u : 1u)) { /* the same DATA PID again: the device ACKs and discards the packet, no FIFO entry, no packet count change */
		os.duplicates++;
		return HOSTOTG_ACK;
	}
	const uint32_t ts = reg(o + 0x10u);
	const uint32_t pktMask = ep == 0u ? (1u << 19) : (0x3FFu << 19);
	unsigned pktcnt = (ts & pktMask) >> 19;
	const unsigned mps = maxPacket(ep == 0u ? reg(R_INEP) : ctl, ep);
	const int last = len < mps || pktcnt <= 1u;
	if (rxFree() < 1u + words(len) + (last ? 1u : 0u)) {
		os.rejects++;
		return HOSTOTG_NAK;
	}
	os.outPid[ep] = pid1 ? 2u : 1u;
	rxPush(rxStatus(2u, pid1, len, ep), K_STATUS);
	for (unsigned i = 0; i < words(len); ++i) {
		uint32_t w = 0;
		for (unsigned b = 0; b < 4u && 4u * i + b < len; ++b) w |= (uint32_t)data[4u * i + b] << (8u * b);
		rxPush(w, K_DATA);
	}
	const uint32_t xmask = ep == 0u ? 0x7Fu : 0x7FFFFu;
	uint32_t xfr = ts & xmask;
	xfr = xfr > len ? xfr - len : 0u;
	if (pktcnt != 0u) pktcnt--;
	put(o + 0x10u, (ts & ~(pktMask | xmask)) | (pktcnt << 19) | xfr);
	if (last) {
		rxPush(rxStatus(3u, 0u, 0u, ep), K_OUT_COMPLETE);
		put(o, (ctl & ~C_EPENA) | C_NAKSTS);
	}
	refresh();
	return HOSTOTG_ACK;
}

HostOtgHandshake hostOtgHostIn(unsigned ep, uint8_t *buf, unsigned cap, unsigned *len) {
	if (len) *len = 0;
	if (ep >= EP_COUNT || disconnected()) return HOSTOTG_IGNORED;
	const uint32_t o = inCtlOff(ep);
	const uint32_t ctl = reg(o);
	if (!(ctl & C_USBAEP)) return HOSTOTG_IGNORED;
	if (ctl & C_STALL) return HOSTOTG_STALL;
	if ((reg(R_DCTL) & (1u << 2)) != 0u || (ctl & C_NAKSTS) || !(ctl & C_EPENA)) {
		setIn(ep, I_NAK);
		refresh();
		return HOSTOTG_NAK;
	}
	const uint32_t ts = reg(o + 0x10u);
	const uint32_t pktMask = ep == 0u ? (3u << 19) : (0x3FFu << 19);
	const uint32_t xmask = ep == 0u ? 0x7Fu : 0x7FFFFu;
	unsigned pktcnt = (ts & pktMask) >> 19;
	unsigned xfr = ts & xmask;
	if (pktcnt == 0u) {
		setIn(ep, I_NAK);
		refresh();
		return HOSTOTG_NAK;
	}
	const unsigned mps = maxPacket(ctl, ep);
	const unsigned size = xfr < mps ? xfr : mps;
	const unsigned fifo = (ctl >> 22) & 0xFu;
	if (fifo >= EP_COUNT || os.txCount[fifo] < words(size)) {
		setIn(ep, I_ITTXFE | I_NAK);
		refresh();
		return HOSTOTG_NAK;
	}
	for (unsigned i = 0; i < size; ++i) {
		const uint32_t w = os.tx[fifo][i / 4u];
		if (buf != NULL && i < cap) buf[i] = (uint8_t)(w >> (8u * (i % 4u)));
	}
	const unsigned used = words(size);
	memmove(os.tx[fifo], os.tx[fifo] + used, (os.txCount[fifo] - used) * sizeof(uint32_t));
	os.txCount[fifo] -= used;
	if (len) *len = size;
	xfr -= size;
	pktcnt--;
	put(o + 0x10u, (ts & ~(pktMask | xmask)) | (pktcnt << 19) | xfr);
	if (pktcnt == 0u) {
		put(o, ctl & ~C_EPENA);
		setIn(ep, I_XFRC);
	}
	refresh();
	return HOSTOTG_ACK;
}

void hostOtgUsbReset(void) {
	/* RM0090 DIEPCTLx / DOEPCTLx USBAEP: the core clears it for all endpoints other than EP0 after detecting a USB reset */
	for (unsigned e = 1; e < EP_COUNT; ++e) {
		put(inCtlOff(e), reg(inCtlOff(e)) & ~C_USBAEP);
		put(outCtlOff(e), reg(outCtlOff(e)) & ~C_USBAEP);
		os.outPid[e] = 0;
	}
	setGint(GI_USBRST);
	refresh();
}
void hostOtgEnumDone(void) {
	put(R_DSTS, (reg(R_DSTS) & ~0x6u) | (3u << 1));
	setGint(GI_ENUMDNE);
	refresh();
}
void hostOtgSuspend(void) {
	put(R_DSTS, reg(R_DSTS) | 1u);
	setGint(GI_ESUSP | GI_USBSUSP);
	refresh();
}
void hostOtgResume(void) {
	put(R_DSTS, reg(R_DSTS) & ~1u);
	setGint(GI_WKUPINT);
	refresh();
}
void hostOtgSof(unsigned frame) {
	put(R_DSTS, (reg(R_DSTS) & ~0x003FFF00u) | ((frame & 0x3FFFu) << 8));
	setGint(GI_SOF);
	refresh();
}
void hostOtgVbus(int session) {
	if (session) {
		put(R_GOTGCTL, reg(R_GOTGCTL) | (1u << 19));
		setGint(GI_SRQINT);
	} else {
		put(R_GOTGCTL, reg(R_GOTGCTL) & ~(1u << 19));
		put(R_GOTGINT, reg(R_GOTGINT) | (1u << 2));
	}
	refresh();
}
void hostOtgRefresh(void) { refresh(); }
void hostOtgHoldEndpointDisable(int hold) { os.holdDisable = hold != 0; }
void hostOtgReleaseDisable(void) {
	for (unsigned e = 0; e < EP_COUNT; ++e) {
		uint32_t c = reg(inCtlOff(e));
		if ((c & C_EPDIS) != 0u) {
			put(inCtlOff(e), c & ~(C_EPENA | C_EPDIS));
			setIn(e, I_EPDISD);
		}
		c = reg(outCtlOff(e));
		if ((c & C_EPDIS) != 0u) {
			put(outCtlOff(e), c & ~(C_EPENA | C_EPDIS));
			setOut(e, I_EPDISD);
		}
	}
	refresh();
}
void hostOtgDisconnectEvent(void) {
	setGint(GI_DISCINT);
	refresh();
}
