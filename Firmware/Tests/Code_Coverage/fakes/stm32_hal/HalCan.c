/*
 * HalCan.c: bxCAN CAN1 / CAN2 register and bus model (P12 T5). See HalCan.h for what is modelled and the RM0090 sources.
 * Register access goes through hostRegsPeek / hostRegsPoke (raw, no trapping).
 */
#include "HalCan.h"

#include <stddef.h>
#include <string.h>

#include "HalHostSystem.h"

#define R_MCR 0x00u
#define R_MSR 0x04u
#define R_TSR 0x08u
#define R_RF0R 0x0Cu
#define R_RF1R 0x10u
#define R_IER 0x14u
#define R_ESR 0x18u
#define R_BTR 0x1Cu
#define R_TI(x) (0x180u + 0x10u * (x))
#define R_TDT(x) (0x184u + 0x10u * (x))
#define R_TDL(x) (0x188u + 0x10u * (x))
#define R_TDH(x) (0x18Cu + 0x10u * (x))
#define R_RI(f) (0x1B0u + 0x10u * (f))
#define R_RDT(f) (0x1B4u + 0x10u * (f))
#define R_RDL(f) (0x1B8u + 0x10u * (f))
#define R_RDH(f) (0x1BCu + 0x10u * (f))
#define R_FMR 0x200u
#define R_FM1R 0x204u
#define R_FS1R 0x20Cu
#define R_FFA1R 0x214u
#define R_FA1R 0x21Cu
#define R_FBANK 0x240u

#define MCR_INRQ (1u << 0)
#define MCR_SLEEP (1u << 1)
#define MCR_TXFP (1u << 2)
#define MCR_RFLM (1u << 3)
#define MCR_NART (1u << 4)
#define MCR_AWUM (1u << 5)
#define MCR_TTCM (1u << 7)
#define MCR_RESET (1u << 15)
#define MCR_MASK 0x000100FFu
#define MCR_RESET_VALUE 0x00010002u

#define MSR_INAK (1u << 0)
#define MSR_SLAK (1u << 1)
#define MSR_ERRI (1u << 2)
#define MSR_WKUI (1u << 3)
#define MSR_SLAKI (1u << 4)
#define MSR_RC_W1 (MSR_ERRI | MSR_WKUI | MSR_SLAKI)
#define MSR_RESET_VALUE 0x0C02u

#define IER_TMEIE (1u << 0)
#define IER_FMPIE0 (1u << 1)
#define IER_FFIE0 (1u << 2)
#define IER_FOVIE0 (1u << 3)
#define IER_FMPIE1 (1u << 4)
#define IER_FFIE1 (1u << 5)
#define IER_FOVIE1 (1u << 6)
#define IER_EWGIE (1u << 8)
#define IER_EPVIE (1u << 9)
#define IER_BOFIE (1u << 10)
#define IER_LECIE (1u << 11)
#define IER_ERRIE (1u << 15)
#define IER_WKUIE (1u << 16)
#define IER_SLKIE (1u << 17)
#define IER_MASK 0x00038F7Fu

#define RF_FMP 3u
#define RF_FULL (1u << 3)
#define RF_FOVR (1u << 4)
#define RF_RFOM (1u << 5)

#define ESR_EWGF 1u
#define ESR_EPVF 2u
#define ESR_BOFF 4u
#define ESR_LEC_MASK (7u << 4)

#define FMR_FINIT 1u
#define FMR_MASK 0x3F01u
#define FMR_RESET_VALUE 0x2A1C0E01u
#define BTR_MASK 0xC37F03FFu
#define BTR_RESET_VALUE 0x01230000u
#define FILTER_MASK 0x0FFFFFFFu
#define BANKS 28u
#define QUEUE_MAX 3u
#define TXLOG_MAX 256u
#define OUTCOME_MAX 8u

#define SR(x, b) (1u << (8u * (x) + (b))) /* TSR bit b of mailbox x: 0 RQCP, 1 TXOK, 2 ALST, 3 TERR (x = 2: bits 16..19), ABRQ at 7 */
enum { T_RQCP = 0, T_TXOK = 1, T_ALST = 2, T_TERR = 3, T_ABRQ = 7 };
enum { MB_EMPTY = 0, MB_PENDING, MB_SENDING };
enum { MODE_SLEEP = 0, MODE_INIT, MODE_NORMAL }; /* curMode: INAK has priority over SLAK */

static const uintptr_t bases[HOST_CAN_COUNT] = {0x40006400u, 0x40006800u};

typedef struct {
	/* modes */
	uint8_t modeWait, stuckSync, holdBus, busAck;
	uint32_t modeDueAt, modeDueAt2;
	/* transmit */
	uint8_t mb[3], abortReq[3];
	uint32_t seq[3], seqCounter, sendDoneAt;
	int sending;
	uint16_t timer;
	uint8_t outcome[OUTCOME_MAX];
	unsigned outcomeHead, outcomeCount;
	HostCanFrame txLog[TXLOG_MAX];
	unsigned txCount;
	/* receive */
	HostCanFrame rxq[2][QUEUE_MAX];
	uint8_t rxFmi[2][QUEUE_MAX];
	uint16_t rxTime[2][QUEUE_MAX];
	unsigned rxCount[2];
	uint32_t protectedWrites;
	uint8_t srRead;
} CanState;

static CanState cs[HOST_CAN_COUNT];
static unsigned syncTicks, frameTicks, pollReads, pollCount;
static uint32_t virtualBonus;

static uint32_t peek(uintptr_t a) { return hostRegsPeek(a); }
static void poke(uintptr_t a, uint32_t v) { hostRegsPoke(a, v); }
static uintptr_t reg(unsigned i, uint32_t r) { return bases[i] + r; }
static uintptr_t fbase(void) { return bases[0]; } /* the filter registers live in CAN1's block */
static uint32_t now(void) { return hostTickPeek() + virtualBonus; }
static uint32_t get(unsigned i, uint32_t r) { return peek(reg(i, r)); }
static void put(unsigned i, uint32_t r, uint32_t v) { poke(reg(i, r), v); }
static void setBits(unsigned i, uint32_t r, uint32_t m) { put(i, r, get(i, r) | m); }
static void clrBits(unsigned i, uint32_t r, uint32_t m) { put(i, r, get(i, r) & ~m); }

static void resetRegs(unsigned i) {
	put(i, R_MCR, MCR_RESET_VALUE);
	put(i, R_MSR, MSR_RESET_VALUE);
	put(i, R_TSR, 0x1C000000u);
	put(i, R_BTR, BTR_RESET_VALUE);
	if (i == 0u) put(0, R_FMR, FMR_RESET_VALUE);
}

void hostCanResetState(void) {
	memset(cs, 0, sizeof cs);
	for (unsigned i = 0; i < HOST_CAN_COUNT; ++i) {
		cs[i].busAck = 1;
		cs[i].sending = -1;
	}
	syncTicks = frameTicks = 0;
	pollReads = pollCount = 0;
	virtualBonus = 0;
}
void hostCanSetPollReads(unsigned n) { pollReads = n; pollCount = 0; }
void hostCanSetSyncTicks(unsigned t) { syncTicks = t; }
void hostCanSetFrameTicks(unsigned t) { frameTicks = t; }
void hostCanStuckSync(unsigned i, int s) { if (i < HOST_CAN_COUNT) cs[i].stuckSync = s != 0; }
void hostCanHoldBus(unsigned i, int h) { if (i < HOST_CAN_COUNT) cs[i].holdBus = h != 0; }
void hostCanSetBusAck(unsigned i, int a) { if (i < HOST_CAN_COUNT) cs[i].busAck = a != 0; }
void hostCanSetTimer(unsigned i, uint16_t v) { if (i < HOST_CAN_COUNT) cs[i].timer = v; }
void hostCanQueueTxOutcome(unsigned i, int o) {
	if (i < HOST_CAN_COUNT && cs[i].outcomeCount < OUTCOME_MAX) cs[i].outcome[(cs[i].outcomeHead + cs[i].outcomeCount++) % OUTCOME_MAX] = (uint8_t)o;
}
unsigned hostCanTxCount(unsigned i) { return i < HOST_CAN_COUNT ? cs[i].txCount : 0u; }
HostCanFrame hostCanTxAt(unsigned i, unsigned k) {
	HostCanFrame none;
	memset(&none, 0, sizeof none);
	return (i < HOST_CAN_COUNT && k < cs[i].txCount) ? cs[i].txLog[k] : none;
}
void hostCanTxClear(unsigned i) { if (i < HOST_CAN_COUNT) cs[i].txCount = 0; }
uint32_t hostCanProtectedWrites(unsigned i) { return i < HOST_CAN_COUNT ? cs[i].protectedWrites : 0u; }

static int curMode(unsigned i) {
	const uint32_t m = get(i, R_MSR);
	return (m & MSR_INAK) ? MODE_INIT : ((m & MSR_SLAK) ? MODE_SLEEP : MODE_NORMAL);
}

/* ------------------------------------------------------------------ transmit mailboxes */
static int mailboxCount(unsigned i, int state) {
	int n = 0;
	for (unsigned x = 0; x < 3u; ++x) n += cs[i].mb[x] == state;
	return n;
}

/* Arbitration key (lower wins): base identifier, IDE, extended identifier. Documented simplification. */
static uint64_t arbKey(unsigned i, unsigned x) {
	const uint32_t ti = get(i, R_TI(x));
	const uint32_t ide = (ti >> 2) & 1u;
	const uint32_t id = ide ? (ti >> 3) : (ti >> 21);
	const uint32_t base = ide ? (id >> 18) : id;
	return ((uint64_t)base << 20) | ((uint64_t)ide << 19) | (ide ? (id & 0x3FFFFu) : 0u);
}

/* true if mailbox a has a higher priority than b (both pending) */
static int higher(unsigned i, unsigned a, unsigned b) {
	if (get(i, R_MCR) & MCR_TXFP) return cs[i].seq[a] < cs[i].seq[b];
	{
		const uint64_t ka = arbKey(i, a), kb = arbKey(i, b);
		if (ka != kb) return ka < kb;
		return a < b;
	}
}

static void tsrRefresh(unsigned i) {
	uint32_t t = get(i, R_TSR) & ~((7u << 29) | (7u << 26) | (3u << 24));
	int lowest = -1, pending = 0;
	for (unsigned x = 0; x < 3u; ++x) {
		if (cs[i].mb[x] == MB_EMPTY) t |= 1u << (26u + x);
		else {
			++pending;
			if (lowest < 0 || higher(i, (unsigned)lowest, x)) lowest = (int)x;
		}
	}
	if (pending > 1 && lowest >= 0) t |= 1u << (29u + (unsigned)lowest);
	{
		unsigned code = 0;
		if (pending == 3) code = (unsigned)lowest;
		else {
			for (unsigned x = 0; x < 3u; ++x)
				if (cs[i].mb[x] == MB_EMPTY) { code = x; break; }
		}
		t |= code << 24;
	}
	put(i, R_TSR, t);
}

static void frameFromMailbox(unsigned i, unsigned x, HostCanFrame *f) {
	const uint32_t ti = get(i, R_TI(x)), tdt = get(i, R_TDT(x)), tdl = get(i, R_TDL(x)), tdh = get(i, R_TDH(x));
	memset(f, 0, sizeof *f);
	f->ide = (uint8_t)((ti >> 2) & 1u);
	f->rtr = (uint8_t)((ti >> 1) & 1u);
	f->id = f->ide ? (ti >> 3) : (ti >> 21);
	f->dlc = (uint8_t)(tdt & 0xFu);
	for (unsigned b = 0; b < 4u; ++b) {
		f->data[b] = (uint8_t)(tdl >> (8u * b));
		f->data[4u + b] = (uint8_t)(tdh >> (8u * b));
	}
}

static void emptyMailbox(unsigned i, unsigned x) {
	cs[i].mb[x] = MB_EMPTY;
	cs[i].abortReq[x] = 0;
	clrBits(i, R_TSR, SR(x, T_ABRQ));
	clrBits(i, R_TI(x), 1u); /* TXRQ cleared when the mailbox becomes empty */
}

static int rxStoreFrame(unsigned i, const HostCanFrame *f);

static void txComplete(unsigned i, unsigned x) {
	CanState *s = &cs[i];
	int outcome = HOST_CAN_TX_OK;
	const uint32_t mcr = get(i, R_MCR), btr = get(i, R_BTR);
	const int loop = (btr >> 30) & 1;
	s->sending = -1;
	if (s->outcomeCount != 0u) {
		outcome = s->outcome[s->outcomeHead];
		s->outcomeHead = (s->outcomeHead + 1u) % OUTCOME_MAX;
		s->outcomeCount--;
	}
	if (outcome == HOST_CAN_TX_OK && !s->busAck && !loop) { /* nobody acknowledges: acknowledgement error (LEC = 3) */
		outcome = HOST_CAN_TX_ERROR;
		put(i, R_ESR, (get(i, R_ESR) & ~ESR_LEC_MASK) | (3u << 4));
	}
	if (outcome == HOST_CAN_TX_OK) {
		HostCanFrame f;
		frameFromMailbox(i, x, &f);
		if (s->txCount < TXLOG_MAX) s->txLog[s->txCount++] = f;
		setBits(i, R_TSR, SR(x, T_RQCP) | SR(x, T_TXOK));
		if (mcr & MCR_TTCM) put(i, R_TDT(x), (get(i, R_TDT(x)) & 0xFFFFu) | ((uint32_t)s->timer << 16));
		emptyMailbox(i, x);
		if (loop) rxStoreFrame(i, &f);
		return;
	}
	setBits(i, R_TSR, outcome == HOST_CAN_TX_ARB_LOST ? SR(x, T_ALST) : SR(x, T_TERR));
	if (s->abortReq[x] || (mcr & MCR_NART)) {
		setBits(i, R_TSR, SR(x, T_RQCP));
		clrBits(i, R_TSR, SR(x, T_TXOK));
		emptyMailbox(i, x);
	} else {
		s->mb[x] = MB_PENDING; /* sent again */
	}
}

static void txEval(unsigned i) {
	CanState *s = &cs[i];
	for (unsigned guard = 0; guard < 100u; ++guard) {
		int progressed = 0;
		if (s->sending >= 0 && !s->holdBus && (int32_t)(now() - s->sendDoneAt) >= 0) {
			txComplete(i, (unsigned)s->sending);
			progressed = 1;
		}
		if (s->sending < 0 && !s->holdBus && curMode(i) == MODE_NORMAL && mailboxCount(i, MB_PENDING) > 0) {
			int best = -1;
			for (unsigned x = 0; x < 3u; ++x)
				if (s->mb[x] == MB_PENDING && (best < 0 || higher(i, x, (unsigned)best))) best = (int)x;
			s->mb[best] = MB_SENDING;
			s->sending = best;
			s->sendDoneAt = now() + frameTicks;
			progressed = 1;
		}
		if (!progressed) break;
	}
	tsrRefresh(i);
}

/* ------------------------------------------------------------------ modes */
/* INAK follows INRQ and SLAK follows SLEEP, each after `syncTicks` and independent of the other: after the reset (SLEEP = 1, SLAK = 1) INRQ = 1 gives INAK = 1 while
 * SLAK stays 1 until SLEEP is cleared (the HAL's own initialization sequence relies on this: INRQ, wait INAK, then clear SLEEP, wait SLAK). RM0090 32.4.3 only says the
 * software "must also clear the SLEEP bit" when it sets INRQ in Sleep mode, and Figure 336 shows the combined states (Medium confidence, NEEDS REVIEW). */
static void modeEval(unsigned i) {
	CanState *s = &cs[i];
	const uint32_t mcr = get(i, R_MCR), msr = get(i, R_MSR);
	const int wantInit = (mcr & MCR_INRQ) != 0u, wantSleep = (mcr & MCR_SLEEP) != 0u;
	const int haveInit = (msr & MSR_INAK) != 0u, haveSleep = (msr & MSR_SLAK) != 0u;
	if (wantInit == haveInit) s->modeWait &= (uint8_t)~1u;
	else if (!(s->modeWait & 1u)) { s->modeWait |= 1u; s->modeDueAt = now() + syncTicks; }
	if (wantSleep == haveSleep) s->modeWait &= (uint8_t)~2u;
	else if (!(s->modeWait & 2u)) { s->modeWait |= 2u; s->modeDueAt2 = now() + syncTicks; }
	if (s->stuckSync) return;
	if ((s->modeWait & 1u) && (int32_t)(now() - s->modeDueAt) >= 0) {
		s->modeWait &= (uint8_t)~1u;
		if (wantInit) setBits(i, R_MSR, MSR_INAK); else clrBits(i, R_MSR, MSR_INAK);
	}
	if ((s->modeWait & 2u) && (int32_t)(now() - s->modeDueAt2) >= 0) {
		s->modeWait &= (uint8_t)~2u;
		if (wantSleep) {
			setBits(i, R_MSR, MSR_SLAK);
			if (get(i, R_IER) & IER_SLKIE) setBits(i, R_MSR, MSR_SLAKI);
		} else {
			clrBits(i, R_MSR, MSR_SLAK | MSR_SLAKI);
		}
	}
}

static void evalAll(unsigned i) {
	modeEval(i);
	txEval(i);
}

void hostCanPoll(unsigned i) { if (i < HOST_CAN_COUNT) evalAll(i); }

void hostCanBusActivity(unsigned i) {
	if (i >= HOST_CAN_COUNT || curMode(i) != MODE_SLEEP) return;
	setBits(i, R_MSR, MSR_WKUI);
	if (get(i, R_MCR) & MCR_AWUM) {
		clrBits(i, R_MCR, MCR_SLEEP);
		evalAll(i);
	}
}

/* ------------------------------------------------------------------ filters and reception */
static uint32_t frame32(const HostCanFrame *f) {
	return (f->ide ? (f->id << 3) : (f->id << 21)) | ((uint32_t)f->ide << 2) | ((uint32_t)f->rtr << 1);
}
static uint32_t frame16(const HostCanFrame *f) {
	const uint32_t stid = f->ide ? (f->id >> 18) : f->id;
	return (stid << 5) | ((uint32_t)f->rtr << 4) | ((uint32_t)f->ide << 3) | (f->ide ? ((f->id >> 15) & 7u) : 0u);
}

static int filterMatch(unsigned i, const HostCanFrame *f, unsigned *fifoOut, unsigned *fmiOut) {
	const uint32_t fmr = peek(fbase() + R_FMR);
	unsigned c2sb = (fmr >> 8) & 0x3Fu, lo, hi, num[2] = {0, 0};
	int bestClass = 99, found = 0;
	const uint32_t f32 = frame32(f), f16 = frame16(f);
	if (c2sb > BANKS) c2sb = BANKS;
	lo = i == 0u ? 0u : c2sb;
	hi = i == 0u ? c2sb : BANKS;
	for (unsigned n = lo; n < hi; ++n) {
		const unsigned fifo = (peek(fbase() + R_FFA1R) >> n) & 1u, scale32 = (peek(fbase() + R_FS1R) >> n) & 1u, list = (peek(fbase() + R_FM1R) >> n) & 1u;
		const int active = (int)((peek(fbase() + R_FA1R) >> n) & 1u);
		const uint32_t r1 = peek(fbase() + R_FBANK + 8u * n), r2 = peek(fbase() + R_FBANK + 8u * n + 4u);
		unsigned size, cls;
		unsigned hits[4] = {0, 0, 0, 0}, nh = 0;
		if (scale32 && list) { size = 2; cls = 0; }
		else if (scale32) { size = 1; cls = 1; }
		else if (list) { size = 4; cls = 2; }
		else { size = 2; cls = 3; }
		if (active) {
			if (cls == 0u) {
				if (((f32 ^ r1) & ~1u) == 0u) hits[nh++] = 0;
				if (((f32 ^ r2) & ~1u) == 0u) hits[nh++] = 1;
			} else if (cls == 1u) {
				if (((f32 ^ r1) & r2 & ~1u) == 0u) hits[nh++] = 0;
			} else if (cls == 2u) {
				const uint32_t ids[4] = {r1 & 0xFFFFu, r1 >> 16, r2 & 0xFFFFu, r2 >> 16};
				for (unsigned k = 0; k < 4u; ++k)
					if (f16 == ids[k]) { hits[nh++] = k; break; }
			} else {
				if (((f16 ^ (r1 & 0xFFFFu)) & (r1 >> 16)) == 0u) hits[nh++] = 0;
				else if (((f16 ^ (r2 & 0xFFFFu)) & (r2 >> 16)) == 0u) hits[nh++] = 1;
			}
			if (nh != 0u && ((int)cls < bestClass)) {
				bestClass = (int)cls;
				*fifoOut = fifo;
				*fmiOut = num[fifo] + hits[0];
				found = 1;
			}
		}
		num[fifo] += size;
	}
	return found;
}

static void rxRefreshOutput(unsigned i, unsigned f) {
	uint32_t ri = 0, rdt = 0, rdl = 0, rdh = 0;
	if (cs[i].rxCount[f] != 0u) {
		const HostCanFrame *h = &cs[i].rxq[f][0];
		ri = (h->ide ? (h->id << 3) | 4u : (h->id << 21)) | ((uint32_t)h->rtr << 1);
		rdt = (uint32_t)(h->dlc & 0xFu) | ((uint32_t)cs[i].rxFmi[f][0] << 8) | ((uint32_t)cs[i].rxTime[f][0] << 16);
		for (unsigned b = 0; b < 4u; ++b) {
			rdl |= (uint32_t)h->data[b] << (8u * b);
			rdh |= (uint32_t)h->data[4u + b] << (8u * b);
		}
	}
	put(i, R_RI(f), ri);
	put(i, R_RDT(f), rdt);
	put(i, R_RDL(f), rdl);
	put(i, R_RDH(f), rdh);
	put(i, f ? R_RF1R : R_RF0R, (get(i, f ? R_RF1R : R_RF0R) & ~RF_FMP) | (uint32_t)cs[i].rxCount[f]);
}

static int rxStoreFrame(unsigned i, const HostCanFrame *frame) {
	CanState *s = &cs[i];
	unsigned fifo = 0, fmi = 0;
	int result = 1;
	if (curMode(i) != MODE_NORMAL || (peek(fbase() + R_FMR) & FMR_FINIT) != 0u) return 0;
	if (!filterMatch(i, frame, &fifo, &fmi)) return 0;
	{
		const uint32_t rfr = fifo ? R_RF1R : R_RF0R;
		if (s->rxCount[fifo] == QUEUE_MAX) {
			setBits(i, rfr, RF_FOVR);
			if (get(i, R_MCR) & 8u) { /* RFLM: the new message is discarded */
				return -1;
			}
			s->rxq[fifo][QUEUE_MAX - 1u] = *frame; /* the newest stored message is overwritten */
			s->rxFmi[fifo][QUEUE_MAX - 1u] = (uint8_t)fmi;
			s->rxTime[fifo][QUEUE_MAX - 1u] = s->timer;
			result = -1;
		} else {
			s->rxq[fifo][s->rxCount[fifo]] = *frame;
			s->rxFmi[fifo][s->rxCount[fifo]] = (uint8_t)fmi;
			s->rxTime[fifo][s->rxCount[fifo]] = s->timer;
			s->rxCount[fifo]++;
			if (s->rxCount[fifo] == QUEUE_MAX) setBits(i, rfr, RF_FULL);
		}
		rxRefreshOutput(i, fifo);
	}
	return result;
}

int hostCanRx(unsigned i, const HostCanFrame *frame) {
	if (i >= HOST_CAN_COUNT || frame == NULL) return 0;
	evalAll(i);
	return rxStoreFrame(i, frame);
}

/* ------------------------------------------------------------------ errors and interrupts */
static void errorSources(unsigned i) {
	const uint32_t ier = get(i, R_IER), esr = get(i, R_ESR);
	const unsigned lec = (esr >> 4) & 7u;
	if (((esr & ESR_EWGF) && (ier & IER_EWGIE)) || ((esr & ESR_EPVF) && (ier & IER_EPVIE)) || ((esr & ESR_BOFF) && (ier & IER_BOFIE)) ||
	    (lec >= 1u && lec <= 6u && (ier & IER_LECIE)))
		setBits(i, R_MSR, MSR_ERRI);
}

void hostCanSetCounters(unsigned i, unsigned rec, unsigned tec) {
	uint32_t esr;
	if (i >= HOST_CAN_COUNT) return;
	esr = get(i, R_ESR) & ~(0xFFFF0000u | ESR_EWGF | ESR_EPVF | ESR_BOFF);
	esr |= ((uint32_t)(rec & 0xFFu) << 24) | ((uint32_t)(tec & 0xFFu) << 16);
	if (rec >= 96u || tec >= 96u) esr |= ESR_EWGF;
	if (rec > 127u || tec > 127u) esr |= ESR_EPVF;
	if (tec > 255u) esr |= ESR_BOFF;
	put(i, R_ESR, esr);
	errorSources(i);
}
void hostCanRaiseError(unsigned i, unsigned lec) {
	if (i >= HOST_CAN_COUNT) return;
	put(i, R_ESR, (get(i, R_ESR) & ~ESR_LEC_MASK) | ((lec & 7u) << 4));
	errorSources(i);
}

unsigned hostCanIrqPending(unsigned i) {
	unsigned r = 0;
	uint32_t ier, tsr, rf0, rf1, msr;
	if (i >= HOST_CAN_COUNT) return 0;
	ier = get(i, R_IER);
	tsr = get(i, R_TSR);
	rf0 = get(i, R_RF0R);
	rf1 = get(i, R_RF1R);
	msr = get(i, R_MSR);
	if ((ier & IER_TMEIE) && (tsr & (SR(0, T_RQCP) | SR(1, T_RQCP) | SR(2, T_RQCP)))) r |= HOST_CAN_IRQ_TX;
	if (((ier & IER_FMPIE0) && (rf0 & RF_FMP)) || ((ier & IER_FFIE0) && (rf0 & RF_FULL)) || ((ier & IER_FOVIE0) && (rf0 & RF_FOVR))) r |= HOST_CAN_IRQ_FIFO0;
	if (((ier & IER_FMPIE1) && (rf1 & RF_FMP)) || ((ier & IER_FFIE1) && (rf1 & RF_FULL)) || ((ier & IER_FOVIE1) && (rf1 & RF_FOVR))) r |= HOST_CAN_IRQ_FIFO1;
	if (((msr & MSR_ERRI) && (ier & IER_ERRIE)) || ((msr & MSR_WKUI) && (ier & IER_WKUIE)) || ((msr & MSR_SLAKI) && (ier & IER_SLKIE))) r |= HOST_CAN_IRQ_SCE;
	return r;
}

/* ------------------------------------------------------------------ bus callbacks */
static int inRange(uint32_t off, uint32_t lo, uint32_t hi) { return off >= lo && off <= hi; }
static int known(unsigned i, uintptr_t a) {
	const uint32_t off = (uint32_t)(a - bases[i]);
	if ((off & 3u) != 0u) return 0;
	if (off <= R_BTR || inRange(off, 0x180u, 0x1CCu)) return 1;
	if (i != 0u) return 0;
	return off == R_FMR || off == R_FM1R || off == R_FS1R || off == R_FFA1R || off == R_FA1R || inRange(off, R_FBANK, R_FBANK + 8u * BANKS - 4u);
}
static int canWrite(unsigned i, uintptr_t a) {
	const uint32_t off = (uint32_t)(a - bases[i]);
	return known(i, a) && !inRange(off, 0x1B0u, 0x1CCu);
}

static int beforeRead(unsigned i, uintptr_t a) {
	if (!known(i, a)) return 0;
	if ((uint32_t)(a - bases[i]) == R_MSR && pollReads != 0u && ++pollCount >= pollReads) {
		pollCount = 0;
		virtualBonus++;
	}
	evalAll(i);
	return 1;
}

static void protectedWrite(unsigned i, uintptr_t a, uint32_t oldValue) {
	cs[i].protectedWrites++;
	poke(a, oldValue);
}

static void afterWrite(unsigned i, uintptr_t a, uint32_t oldValue, uint32_t written) {
	CanState *s = &cs[i];
	const uint32_t off = (uint32_t)(a - bases[i]);
	if (inRange(off, 0x180u, 0x1ACu)) { /* transmit mailbox */
		const unsigned x = (off - 0x180u) / 0x10u, w = ((off - 0x180u) % 0x10u) / 4u;
		if (s->mb[x] != MB_EMPTY) {
			protectedWrite(i, a, oldValue);
			return;
		}
		if (w == 0u) {
			poke(a, written);
			if (written & 1u) {
				s->mb[x] = MB_PENDING;
				s->seq[x] = ++s->seqCounter;
				clrBits(i, R_TSR, SR(x, T_RQCP) | SR(x, T_TXOK) | SR(x, T_ALST) | SR(x, T_TERR));
				evalAll(i);
			} else {
				tsrRefresh(i);
			}
		} else {
			poke(a, written);
		}
		return;
	}
	switch (off) {
	case R_MCR:
		if (written & MCR_RESET) {
			const uint32_t keepBusAck = s->busAck;
			memset(s->mb, 0, sizeof s->mb);
			memset(s->abortReq, 0, sizeof s->abortReq);
			s->sending = -1;
			s->rxCount[0] = s->rxCount[1] = 0;
			s->modeWait = 0;
			s->busAck = (uint8_t)keepBusAck;
			put(i, R_MCR, MCR_RESET_VALUE);
			put(i, R_MSR, (get(i, R_MSR) & ~(MSR_INAK | MSR_SLAK | MSR_SLAKI)) | MSR_SLAK);
			put(i, R_RF0R, 0);
			put(i, R_RF1R, 0);
			rxRefreshOutput(i, 0);
			rxRefreshOutput(i, 1);
			tsrRefresh(i);
			break;
		}
		poke(a, written & MCR_MASK);
		evalAll(i);
		break;
	case R_MSR: poke(a, oldValue & ~(written & MSR_RC_W1)); break;
	case R_TSR: {
		uint32_t t = oldValue;
		for (unsigned x = 0; x < 3u; ++x) {
			if (written & SR(x, T_RQCP)) t &= ~(SR(x, T_RQCP) | SR(x, T_TXOK) | SR(x, T_ALST) | SR(x, T_TERR));
			if (written & SR(x, T_TXOK)) t &= ~SR(x, T_TXOK);
			if (written & SR(x, T_ALST)) t &= ~SR(x, T_ALST);
			if (written & SR(x, T_TERR)) t &= ~SR(x, T_TERR);
		}
		poke(a, t);
		for (unsigned x = 0; x < 3u; ++x) {
			if (!(written & SR(x, T_ABRQ))) continue;
			if (s->mb[x] == MB_PENDING) {
				setBits(i, R_TSR, SR(x, T_RQCP));
				clrBits(i, R_TSR, SR(x, T_TXOK));
				emptyMailbox(i, x);
			} else if (s->mb[x] == MB_SENDING) {
				s->abortReq[x] = 1;
				setBits(i, R_TSR, SR(x, T_ABRQ));
			}
		}
		evalAll(i);
		break;
	}
	case R_RF0R:
	case R_RF1R: {
		const unsigned f = off == R_RF1R;
		uint32_t v = oldValue & ~(written & (RF_FULL | RF_FOVR));
		poke(a, v);
		if ((written & RF_RFOM) && s->rxCount[f] != 0u) {
			for (unsigned k = 1; k < QUEUE_MAX; ++k) {
				s->rxq[f][k - 1u] = s->rxq[f][k];
				s->rxFmi[f][k - 1u] = s->rxFmi[f][k];
				s->rxTime[f][k - 1u] = s->rxTime[f][k];
			}
			s->rxCount[f]--;
			rxRefreshOutput(i, f);
		}
		break;
	}
	case R_IER: poke(a, written & IER_MASK); break;
	case R_ESR: poke(a, (oldValue & ~ESR_LEC_MASK) | (written & ESR_LEC_MASK)); break;
	case R_BTR:
		if ((get(i, R_MSR) & MSR_INAK) == 0u) protectedWrite(i, a, oldValue);
		else poke(a, written & BTR_MASK);
		break;
	case R_FMR: poke(a, (oldValue & ~FMR_MASK) | (written & FMR_MASK)); break;
	case R_FM1R:
	case R_FS1R:
	case R_FFA1R:
		if ((peek(fbase() + R_FMR) & FMR_FINIT) == 0u) protectedWrite(i, a, oldValue);
		else poke(a, written & FILTER_MASK);
		break;
	case R_FA1R: poke(a, written & FILTER_MASK); break;
	default:
		if (inRange(off, R_FBANK, R_FBANK + 8u * BANKS - 4u)) {
			const unsigned n = (off - R_FBANK) / 8u;
			if (((peek(fbase() + R_FA1R) >> n) & 1u) != 0u && (peek(fbase() + R_FMR) & FMR_FINIT) == 0u) protectedWrite(i, a, oldValue);
			else poke(a, written);
		} else {
			poke(a, oldValue);
		}
		break;
	}
}

static void resetAll(unsigned i) { resetRegs(i); }

#define CAN_THUNKS(N)                                                                          \
	static int canWrite##N(uintptr_t a) { return canWrite(N, a); }                             \
	static int beforeRead##N(uintptr_t a) { return beforeRead(N, a); }                         \
	static void afterWrite##N(uintptr_t a, uint32_t o, uint32_t w) { afterWrite(N, a, o, w); } \
	static void reset##N(void) { resetAll(N); }
CAN_THUNKS(0)
CAN_THUNKS(1)

static const HostRegModel canModels[HOST_CAN_COUNT] = {
	{"CAN1", 0x40006400u, 0x400u, reset0, beforeRead0, NULL, canWrite0, afterWrite0},
	{"CAN2", 0x40006800u, 0x400u, reset1, beforeRead1, NULL, canWrite1, afterWrite1},
};

void hostCanRegister(void) {
	for (unsigned i = 0; i < HOST_CAN_COUNT; ++i) hostRegsAddModel(&canModels[i]);
}
