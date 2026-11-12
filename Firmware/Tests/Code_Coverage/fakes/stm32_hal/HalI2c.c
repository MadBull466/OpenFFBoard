/*
 * HalI2c.c: I2C1..I2C3 register and bus model (P12 T4c). See HalI2c.h for what is modelled and the RM0090 sources.
 * Register access goes through hostRegsPeek / hostRegsPoke (raw, no trapping).
 */
#include "HalI2c.h"

#include <stddef.h>

#include "HalHostSystem.h"

#define R_CR1 0x00u
#define R_CR2 0x04u
#define R_OAR1 0x08u
#define R_OAR2 0x0Cu
#define R_DR 0x10u
#define R_SR1 0x14u
#define R_SR2 0x18u
#define R_CCR 0x1Cu
#define R_TRISE 0x20u

#define CR1_PE (1u << 0)
#define CR1_ENGC (1u << 6)
#define CR1_NOSTRETCH (1u << 7)
#define CR1_START (1u << 8)
#define CR1_STOP (1u << 9)
#define CR1_ACK (1u << 10)
#define CR1_POS (1u << 11)
#define CR1_PEC (1u << 12)
#define CR1_ALERT (1u << 13)
#define CR1_SWRST (1u << 15)
#define CR1_MASK 0xBFFBu

#define CR2_ITERREN (1u << 8)
#define CR2_ITEVTEN (1u << 9)
#define CR2_ITBUFEN (1u << 10)
#define CR2_MASK 0x1F3Fu

#define OAR1_ADDMODE (1u << 15)
#define OAR1_MASK 0xC3FFu
#define OAR2_ENDUAL (1u << 0)
#define OAR2_MASK 0x00FFu
#define CCR_MASK 0xCFFFu
#define TRISE_MASK 0x003Fu

#define SR1_SB (1u << 0)
#define SR1_ADDR (1u << 1)
#define SR1_BTF (1u << 2)
#define SR1_ADD10 (1u << 3)
#define SR1_STOPF (1u << 4)
#define SR1_RXNE (1u << 6)
#define SR1_TXE (1u << 7)
#define SR1_BERR (1u << 8)
#define SR1_ARLO (1u << 9)
#define SR1_AF (1u << 10)
#define SR1_OVR (1u << 11)
#define SR1_PECERR (1u << 12)
#define SR1_TIMEOUT (1u << 14)
#define SR1_SMBALERT (1u << 15)
#define SR1_RC_W0 (SR1_SMBALERT | SR1_TIMEOUT | SR1_PECERR | SR1_OVR | SR1_AF | SR1_ARLO | SR1_BERR)
#define SR1_ERRORS SR1_RC_W0

#define SR2_MSL (1u << 0)
#define SR2_BUSY (1u << 1)
#define SR2_TRA (1u << 2)
#define SR2_GENCALL (1u << 4)
#define SR2_DUALF (1u << 7)

#define LOG_MAX 512u
#define DEV_MAX 4u
#define QUEUE_MAX 256u

enum { M_IDLE = 0, M_ADDRWAIT, M_ADDRFLIGHT, M_TX, M_RX };

static const uintptr_t bases[HOST_I2C_COUNT] = {0x40005400u, 0x40005800u, 0x40005C00u};

typedef struct {
	uint8_t used, ten, nackAddr;
	uint16_t addr;
	int32_t nackDataAt;
	uint8_t queue[QUEUE_MAX];
	unsigned queueCount, queuePos;
} Device;

typedef struct {
	uint8_t mode, addrByte, addr10, srRead;
	int dev;
	uint32_t addrDoneAt;
	/* transmit pipeline: shift register and DR */
	uint8_t shiftBusy, shiftLoaded, shiftVal, drFull, drVal, txHold;
	uint32_t txDoneAt;
	/* receive pipeline */
	uint8_t rxDr, rxShiftVal, rxShiftFull, rxInFlight, rxActive, posLatch;
	uint32_t rxDoneAt;
	unsigned dataIdx;
	/* target */
	uint8_t slActive, slAddr10;
	uint8_t stuck, busLocked;
	Device devs[DEV_MAX];
	HostI2cEvent log[LOG_MAX];
	unsigned logCount;
	uint32_t configWhileEnabled, strayDr, swrst, peOffBusy, seqBreaks;
} I2cState;

static I2cState st[HOST_I2C_COUNT];
static unsigned byteTicks;
static unsigned pollReads, pollCount;
static uint32_t virtualBonus;

static uint32_t peek(uintptr_t a) { return hostRegsPeek(a); }
static void poke(uintptr_t a, uint32_t v) { hostRegsPoke(a, v); }
static uintptr_t reg(unsigned i, uint32_t r) { return bases[i] + r; }
static uint32_t now(void) { return hostTickPeek() + virtualBonus; }
static uint32_t cr1(unsigned i) { return peek(reg(i, R_CR1)); }
static uint32_t cr2(unsigned i) { return peek(reg(i, R_CR2)); }
static uint32_t sr1(unsigned i) { return peek(reg(i, R_SR1)); }
static uint32_t sr2(unsigned i) { return peek(reg(i, R_SR2)); }
static void sr1Set(unsigned i, uint32_t m) { poke(reg(i, R_SR1), sr1(i) | m); }
static void sr1Clr(unsigned i, uint32_t m) { poke(reg(i, R_SR1), sr1(i) & ~m); }
static void sr2Set(unsigned i, uint32_t m) { poke(reg(i, R_SR2), sr2(i) | m); }
static void sr2Clr(unsigned i, uint32_t m) { poke(reg(i, R_SR2), sr2(i) & ~m); }
static void cr1Clr(unsigned i, uint32_t m) { poke(reg(i, R_CR1), cr1(i) & ~m); }
static int peOn(unsigned i) { return (cr1(i) & CR1_PE) != 0u; }
static int msl(unsigned i) { return (sr2(i) & SR2_MSL) != 0u; }
static int tra(unsigned i) { return (sr2(i) & SR2_TRA) != 0u; }

void hostI2cResetState(void) {
	for (unsigned i = 0; i < HOST_I2C_COUNT; ++i) {
		st[i] = (I2cState){0};
		st[i].dev = -1;
	}
	byteTicks = 0;
	pollReads = pollCount = 0;
	virtualBonus = 0;
}
void hostI2cSetByteTicks(unsigned t) { byteTicks = t; }
void hostI2cSetPollReads(unsigned n) { pollReads = n; pollCount = 0; }
void hostI2cStuck(unsigned i, int s) { if (i < HOST_I2C_COUNT) st[i].stuck = s != 0; }
uint32_t hostI2cConfigWhileEnabled(unsigned i) { return i < HOST_I2C_COUNT ? st[i].configWhileEnabled : 0u; }
uint32_t hostI2cStrayDrWrites(unsigned i) { return i < HOST_I2C_COUNT ? st[i].strayDr : 0u; }
uint32_t hostI2cSwrstCount(unsigned i) { return i < HOST_I2C_COUNT ? st[i].swrst : 0u; }
uint32_t hostI2cPeOffBusy(unsigned i) { return i < HOST_I2C_COUNT ? st[i].peOffBusy : 0u; }
uint32_t hostI2cSequenceBreaks(unsigned i) { return i < HOST_I2C_COUNT ? st[i].seqBreaks : 0u; }

/* ------------------------------------------------------------------ peer devices and the log */
int hostI2cDeviceAdd(unsigned i, uint16_t address, int ten) {
	if (i >= HOST_I2C_COUNT) return -1;
	for (unsigned d = 0; d < DEV_MAX; ++d) {
		if (!st[i].devs[d].used) {
			st[i].devs[d] = (Device){0};
			st[i].devs[d].used = 1;
			st[i].devs[d].ten = ten != 0;
			st[i].devs[d].addr = address;
			st[i].devs[d].nackDataAt = -1;
			return (int)d;
		}
	}
	return -1;
}
static Device *devAt(unsigned i, int d) { return (i < HOST_I2C_COUNT && d >= 0 && d < (int)DEV_MAX && st[i].devs[d].used) ? &st[i].devs[d] : NULL; }
void hostI2cDeviceNackAddress(unsigned i, int d, int nack) { Device *x = devAt(i, d); if (x) x->nackAddr = nack != 0; }
void hostI2cDeviceNackDataAt(unsigned i, int d, int index) { Device *x = devAt(i, d); if (x) x->nackDataAt = index; }
void hostI2cDeviceQueue(unsigned i, int d, const uint8_t *data, unsigned count) {
	Device *x = devAt(i, d);
	if (!x) return;
	for (unsigned k = 0; k < count && x->queueCount < QUEUE_MAX; ++k) x->queue[x->queueCount++] = data[k];
}
unsigned hostI2cLogCount(unsigned i) { return i < HOST_I2C_COUNT ? st[i].logCount : 0u; }
HostI2cEvent hostI2cLogAt(unsigned i, unsigned k) {
	HostI2cEvent none = {0, 0, 0};
	return (i < HOST_I2C_COUNT && k < st[i].logCount) ? st[i].log[k] : none;
}
void hostI2cLogClear(unsigned i) { if (i < HOST_I2C_COUNT) st[i].logCount = 0; }
static void logEv(unsigned i, uint8_t type, uint8_t value, uint8_t ack) {
	if (st[i].logCount < LOG_MAX) st[i].log[st[i].logCount++] = (HostI2cEvent){type, value, ack};
}

static void busyUpdate(unsigned i, int on) {
	if (on) sr2Set(i, SR2_BUSY);
	else if (!st[i].busLocked) sr2Clr(i, SR2_BUSY);
}
void hostI2cBusBusy(unsigned i, int on) {
	if (i >= HOST_I2C_COUNT) return;
	st[i].busLocked = on != 0;
	if (on) sr2Set(i, SR2_BUSY);
	else if (!msl(i) && !st[i].slActive) sr2Clr(i, SR2_BUSY);
}

static void resetEngine(unsigned i) {
	I2cState *x = &st[i];
	x->mode = M_IDLE;
	x->addr10 = 0;
	x->dev = -1;
	x->shiftBusy = x->shiftLoaded = x->drFull = x->txHold = 0;
	x->rxShiftFull = x->rxInFlight = x->rxActive = 0;
	x->slActive = x->slAddr10 = 0;
	x->dataIdx = 0;
	x->srRead = 0;
}

/* ------------------------------------------------------------------ controller */
static void doStart(unsigned i, int restart) {
	I2cState *x = &st[i];
	logEv(i, restart ? HOST_I2C_EV_RESTART : HOST_I2C_EV_START, 0, 0);
	x->rxActive = x->rxInFlight = 0;
	x->shiftBusy = x->shiftLoaded = x->drFull = x->txHold = 0;
	x->dataIdx = 0;
	if (!restart) { /* a repeated start keeps the 10-bit state and the selected device for the read header (27.3.3) */
		x->addr10 = 0;
		x->dev = -1;
	}
	x->mode = M_ADDRWAIT;
	sr1Set(i, SR1_SB);
	sr1Clr(i, SR1_TXE | SR1_BTF); /* cleared by hardware after a start condition (27.6.6) */
	sr2Set(i, SR2_MSL | SR2_BUSY);
	sr2Clr(i, SR2_TRA);
	cr1Clr(i, CR1_START);
}

static void doStop(unsigned i) {
	I2cState *x = &st[i];
	logEv(i, HOST_I2C_EV_STOP, 0, 0);
	x->mode = M_IDLE;
	x->addr10 = 0;
	x->dev = -1;
	x->shiftBusy = x->shiftLoaded = x->drFull = x->txHold = 0;
	x->rxActive = x->rxInFlight = 0;
	sr1Clr(i, SR1_TXE | SR1_BTF);
	sr2Clr(i, SR2_MSL | SR2_TRA);
	busyUpdate(i, 0);
	cr1Clr(i, CR1_STOP | CR1_START);
}

static int findDevice(unsigned i, uint8_t byte, int ten, uint8_t headerXX) {
	for (unsigned d = 0; d < DEV_MAX; ++d) {
		const Device *v = &st[i].devs[d];
		if (!v->used || v->ten != (ten != 0)) continue;
		if (ten ? ((v->addr >> 8) & 3u) == headerXX : v->addr == (uint16_t)(byte >> 1)) return (int)d;
	}
	return -1;
}

static void addressDone(unsigned i) {
	I2cState *x = &st[i];
	const uint8_t b = x->addrByte;
	const int rw = b & 1;
	int ack = 0;
	x->mode = M_IDLE;
	if (x->addr10 == 1) { /* second byte of a 10-bit address */
		const Device *v = devAt(i, x->dev);
		if (v && !v->nackAddr && (uint8_t)(v->addr & 0xFFu) == b) {
			ack = 1;
			x->addr10 = 2;
			sr1Set(i, SR1_ADDR);
			sr2Set(i, SR2_TRA); /* controller transmitter after the second address byte (Figure 243, EV9, EV6, EV8_1) */
			x->mode = M_TX;
			sr1Set(i, SR1_TXE);
		}
	} else if ((b & 0xF8u) == 0xF0u) { /* 10-bit header 11110xxR */
		if (rw == 0) {
			const int d = findDevice(i, b, 1, (uint8_t)((b >> 1) & 3u));
			if (d >= 0 && !st[i].devs[d].nackAddr) {
				ack = 1;
				x->dev = d;
				x->addr10 = 1;
				x->mode = M_ADDRWAIT;
				sr1Set(i, SR1_ADD10);
			}
		} else if (x->addr10 == 2 && devAt(i, x->dev) && ((devAt(i, x->dev)->addr >> 8) & 3u) == (uint16_t)((b >> 1) & 3u)) {
			ack = 1;
			sr1Set(i, SR1_ADDR);
			sr2Clr(i, SR2_TRA);
			x->mode = M_RX;
		}
	} else {
		const int d = findDevice(i, b, 0, 0);
		if (d >= 0 && !st[i].devs[d].nackAddr) {
			ack = 1;
			x->dev = d;
			sr1Set(i, SR1_ADDR);
			if (rw) {
				sr2Clr(i, SR2_TRA);
				x->mode = M_RX;
			} else {
				sr2Set(i, SR2_TRA);
				x->mode = M_TX;
				sr1Set(i, SR1_TXE);
			}
		}
	}
	logEv(i, HOST_I2C_EV_BYTE_OUT, b, (uint8_t)ack);
	if (!ack) sr1Set(i, SR1_AF);
}

static void startShift(unsigned i, uint8_t v) {
	I2cState *x = &st[i];
	x->shiftVal = v;
	x->shiftBusy = 1;
	x->txDoneAt = now() + byteTicks;
	x->txHold = (sr1(i) & SR1_ADDR) != 0u; /* EV6: SCL is stretched until ADDR is cleared, no data byte reaches the bus before (27.3.3) */
}

static void txByteDone(unsigned i) {
	I2cState *x = &st[i];
	const Device *v = devAt(i, x->dev);
	int ack = v != NULL && !(v->nackDataAt >= 0 && (unsigned)v->nackDataAt == x->dataIdx);
	x->shiftBusy = 0;
	logEv(i, HOST_I2C_EV_BYTE_OUT, x->shiftVal, (uint8_t)ack);
	x->dataIdx++;
	if (!ack) { /* nack: AF, no TXE / BTF, nothing more is sent (27.3.4) */
		x->drFull = 0;
		x->mode = M_IDLE;
		sr1Set(i, SR1_AF);
		return;
	}
	if (x->drFull && (cr1(i) & (CR1_START | CR1_STOP)) != 0u) {
		x->drFull = 0; /* model decision: a byte still in DR when START / STOP is pending after the current byte is not sent (27.6.1 leaves it open) */
		sr1Set(i, SR1_TXE | SR1_BTF);
	} else if (x->drFull) {
		x->drFull = 0;
		startShift(i, x->drVal);
		sr1Set(i, SR1_TXE);
	} else {
		sr1Set(i, SR1_TXE | SR1_BTF); /* both empty: stretched until DR is written */
	}
}

static void rxStart(unsigned i) {
	I2cState *x = &st[i];
	x->rxInFlight = 1;
	x->rxDoneAt = now() + byteTicks;
}

static void rxByteDone(unsigned i) {
	I2cState *x = &st[i];
	Device *v = devAt(i, x->dev);
	const uint8_t byte = (v && v->queuePos < v->queueCount) ? v->queue[v->queuePos++] : 0xFFu;
	const int pos = (cr1(i) & CR1_POS) != 0u;
	const int ack = pos ? x->posLatch : ((cr1(i) & CR1_ACK) != 0u);
	x->rxInFlight = 0;
	logEv(i, HOST_I2C_EV_BYTE_IN, byte, (uint8_t)ack);
	if (pos) x->posLatch = (cr1(i) & CR1_ACK) != 0u;
	if ((sr1(i) & SR1_RXNE) == 0u && !x->rxShiftFull) {
		x->rxDr = byte;
		sr1Set(i, SR1_RXNE);
	} else {
		x->rxShiftVal = byte;
		x->rxShiftFull = 1;
		sr1Set(i, SR1_BTF);
	}
	if (!ack) {
		x->rxActive = 0;
		return;
	}
	if (!x->rxShiftFull) rxStart(i);
}

static void evalController(unsigned i) {
	I2cState *x = &st[i];
	for (unsigned guard = 0; guard < 1000u; ++guard) {
		int progressed = 0;
		if (x->txHold && (sr1(i) & SR1_ADDR) == 0u) {
			x->txHold = 0;
			x->txDoneAt = now() + byteTicks;
		}
		if (x->mode == M_ADDRFLIGHT && !x->stuck && (int32_t)(now() - x->addrDoneAt) >= 0) {
			addressDone(i);
			progressed = 1;
		}
		if (x->shiftBusy && !x->txHold && !x->stuck && (int32_t)(now() - x->txDoneAt) >= 0) {
			txByteDone(i);
			progressed = 1;
		}
		if (x->rxInFlight && !x->stuck && (int32_t)(now() - x->rxDoneAt) >= 0) {
			rxByteDone(i);
			progressed = 1;
		}
		if (peOn(i) && !x->stuck) {
			const uint32_t c = cr1(i);
			const int quiet = x->mode != M_ADDRFLIGHT && !x->shiftBusy && !x->rxInFlight;
			if ((c & CR1_STOP) != 0u && msl(i) && quiet) {
				doStop(i);
				progressed = 1;
			} else if ((c & CR1_START) != 0u && quiet && (msl(i) || (sr2(i) & SR2_BUSY) == 0u)) {
				doStart(i, msl(i));
				progressed = 1;
			}
		}
		if (!progressed) break;
	}
}

/* ------------------------------------------------------------------ target (external controller) */
static int matchOwn(unsigned i, uint8_t byte, int *dual, int *gencall) {
	const uint32_t oar1 = peek(reg(i, R_OAR1)), oar2 = peek(reg(i, R_OAR2));
	*dual = *gencall = 0;
	if ((oar1 & OAR1_ADDMODE) != 0u) return 0;
	if ((byte >> 1) == ((oar1 >> 1) & 0x7Fu) && byte != 0u) return 1;
	if ((oar2 & OAR2_ENDUAL) != 0u && (byte >> 1) == ((oar2 >> 1) & 0x7Fu) && byte != 0u) {
		*dual = 1;
		return 1;
	}
	if (byte == 0u && (cr1(i) & CR1_ENGC) != 0u) {
		*gencall = 1;
		return 1;
	}
	return 0;
}

static void slaveAddressed(unsigned i, int rw, int dual, int gencall) {
	I2cState *x = &st[i];
	x->slActive = 1;
	sr1Set(i, SR1_ADDR);
	sr2Set(i, SR2_BUSY);
	if (rw) {
		sr2Set(i, SR2_TRA);
		sr1Set(i, SR1_TXE);
	} else {
		sr2Clr(i, SR2_TRA);
	}
	if (dual) sr2Set(i, SR2_DUALF); else sr2Clr(i, SR2_DUALF);
	if (gencall) sr2Set(i, SR2_GENCALL); else sr2Clr(i, SR2_GENCALL);
	x->rxShiftFull = 0;
	x->shiftLoaded = 0;
}

static int extAddress(unsigned i, uint8_t b, int restart) {
	I2cState *x = &st[i];
	const uint32_t oar1 = peek(reg(i, R_OAR1));
	int dual, gencall;
	const int ack = (cr1(i) & CR1_ACK) != 0u;
	if (!peOn(i) || msl(i)) return 0;
	if (restart) {
		x->slActive = 0;
		sr2Clr(i, SR2_TRA | SR2_DUALF | SR2_GENCALL);
	}
	if ((oar1 & OAR1_ADDMODE) != 0u) { /* 10-bit own address */
		if ((b & 0xF8u) != 0xF0u || ((b >> 1) & 3u) != ((oar1 >> 8) & 3u)) return 0;
		sr2Set(i, SR2_BUSY);
		if ((b & 1u) == 0u) {
			x->slAddr10 = 1; /* header matched: wait for the address byte, always a receiver */
			return ack;
		}
		if (x->slAddr10 == 2) { /* repeated start with the read header: transmitter */
			slaveAddressed(i, 1, 0, 0);
			return ack;
		}
		return 0;
	}
	if (!matchOwn(i, b, &dual, &gencall)) return 0;
	slaveAddressed(i, b & 1u, dual, gencall);
	return ack;
}

int hostI2cExtStart(unsigned i, uint8_t b) {
	if (i >= HOST_I2C_COUNT) return 0;
	st[i].slAddr10 = 0;
	return extAddress(i, b, 0);
}
int hostI2cExtRestart(unsigned i, uint8_t b) {
	if (i >= HOST_I2C_COUNT) return 0;
	return extAddress(i, b, 1);
}

static int stretched(unsigned i) { return (sr1(i) & SR1_ADDR) != 0u && (cr1(i) & CR1_NOSTRETCH) == 0u; }

int hostI2cExtWrite(unsigned i, uint8_t byte) {
	I2cState *x = &st[i < HOST_I2C_COUNT ? i : 0];
	if (i >= HOST_I2C_COUNT) return -2;
	if (!x->slActive && x->slAddr10 != 1) return -2;
	if (x->slAddr10 == 1) { /* second byte of the own 10-bit address */
		if (byte != (uint8_t)(peek(reg(i, R_OAR1)) & 0xFFu)) {
			x->slAddr10 = 0;
			return 0;
		}
		x->slAddr10 = 2;
		slaveAddressed(i, 0, 0, 0);
		return (cr1(i) & CR1_ACK) != 0u;
	}
	if (tra(i)) return -2;
	if (stretched(i)) return -1;
	{
		const int ack = (cr1(i) & CR1_ACK) != 0u;
		if ((sr1(i) & SR1_RXNE) == 0u && !x->rxShiftFull) {
			x->rxDr = byte;
			sr1Set(i, SR1_RXNE);
		} else if (!x->rxShiftFull) {
			if ((cr1(i) & CR1_NOSTRETCH) != 0u) {
				sr1Set(i, SR1_OVR); /* the new byte is lost */
				return ack;
			}
			x->rxShiftVal = byte;
			x->rxShiftFull = 1;
			sr1Set(i, SR1_BTF);
		} else {
			return -1; /* DR and the shift register are full: SCL is held low */
		}
		return ack;
	}
}

int hostI2cExtReadByte(unsigned i, int masterAck, uint8_t *out) {
	I2cState *x;
	uint8_t byte;
	if (i >= HOST_I2C_COUNT) return -2;
	x = &st[i];
	if (!x->slActive || !tra(i)) return -2;
	if (stretched(i)) return -1;
	if (x->shiftLoaded) {
		byte = x->shiftVal;
		x->shiftLoaded = 0;
		if (x->drFull) { /* DR moves into the shift register: TXE = 1 */
			x->shiftVal = x->drVal;
			x->shiftLoaded = 1;
			x->drFull = 0;
			sr1Set(i, SR1_TXE);
		}
	} else if ((cr1(i) & CR1_NOSTRETCH) != 0u) {
		sr1Set(i, SR1_OVR); /* underrun: the same byte again */
		byte = x->shiftVal;
	} else {
		sr1Set(i, SR1_BTF);
		return -1;
	}
	*out = byte;
	logEv(i, HOST_I2C_EV_BYTE_IN, byte, (uint8_t)(masterAck != 0));
	if (!masterAck) {
		sr1Set(i, SR1_AF); /* nack: AF, no TXE, lines released (27.3.2, 27.3.4) */
		x->shiftLoaded = x->drFull = 0;
		x->slActive = 0;
		sr1Clr(i, SR1_TXE);
	}
	return masterAck != 0;
}

void hostI2cExtStop(unsigned i) {
	I2cState *x;
	if (i >= HOST_I2C_COUNT) return;
	x = &st[i];
	logEv(i, HOST_I2C_EV_STOP, 0, 0);
	if (x->slActive && peOn(i)) sr1Set(i, SR1_STOPF);
	x->slActive = 0;
	x->slAddr10 = 0;
	sr2Clr(i, SR2_TRA | SR2_DUALF | SR2_GENCALL);
	cr1Clr(i, CR1_STOP);
	busyUpdate(i, 0);
}

void hostI2cRaiseSr1(unsigned i, uint32_t mask) { if (i < HOST_I2C_COUNT) sr1Set(i, mask & SR1_ERRORS); }
void hostI2cArbitrationLost(unsigned i) {
	if (i >= HOST_I2C_COUNT) return;
	sr1Set(i, SR1_ARLO);
	sr2Clr(i, SR2_MSL | SR2_TRA);
	st[i].mode = M_IDLE;
	st[i].shiftBusy = st[i].rxInFlight = st[i].rxActive = st[i].drFull = st[i].shiftLoaded = st[i].txHold = 0;
	cr1Clr(i, CR1_START | CR1_STOP);
}

void hostI2cPoll(unsigned i) { if (i < HOST_I2C_COUNT) evalController(i); }

int hostI2cEventPending(unsigned i) {
	uint32_t s, c;
	if (i >= HOST_I2C_COUNT) return 0;
	c = cr2(i);
	s = sr1(i);
	if ((c & CR2_ITEVTEN) == 0u) return 0;
	if ((s & (SR1_SB | SR1_ADDR | SR1_ADD10 | SR1_STOPF | SR1_BTF)) != 0u) return 1;
	return (c & CR2_ITBUFEN) != 0u && (s & (SR1_TXE | SR1_RXNE)) != 0u;
}
int hostI2cErrorPending(unsigned i) {
	if (i >= HOST_I2C_COUNT) return 0;
	return (cr2(i) & CR2_ITERREN) != 0u && (sr1(i) & SR1_ERRORS) != 0u;
}

/* ------------------------------------------------------------------ bus callbacks */
static int known(unsigned i, uintptr_t a) {
	const uint32_t off = (uint32_t)(a - bases[i]);
	return off <= R_TRISE && (off & 3u) == 0u;
}
static int canWrite(unsigned i, uintptr_t a) { return known(i, a) && (uint32_t)(a - bases[i]) != R_SR2; }

static int beforeRead(unsigned i, uintptr_t a) {
	if (!known(i, a)) return 0;
	{
		const uint32_t off = (uint32_t)(a - bases[i]);
		if (off == R_SR1 && pollReads != 0u && ++pollCount >= pollReads) {
			pollCount = 0;
			virtualBonus++;
		}
		if (off == R_SR1 || off == R_SR2 || off == R_DR) evalController(i);
		if (off == R_DR) poke(a, st[i].rxDr);
	}
	return 1;
}

static void afterRead(unsigned i, uintptr_t a, uint32_t value) {
	I2cState *x = &st[i];
	const uint32_t off = (uint32_t)(a - bases[i]);
	(void)value;
	if (off == R_SR1) {
		x->srRead = 1;
	} else if (off == R_SR2) {
		if (x->srRead && (sr1(i) & SR1_ADDR) != 0u) { /* read SR1, then SR2 clears ADDR (27.6.6) */
			sr1Clr(i, SR1_ADDR);
			if (msl(i) && x->mode == M_RX) {
				x->rxActive = 1;
				x->posLatch = 1;
				rxStart(i);
			}
		}
		x->srRead = 0;
		evalController(i);
	} else if (off == R_DR) {
		poke(a, 0u);
		x->srRead = 0;
		if (x->rxShiftFull) { /* DR gets the byte of the shift register, RxNE stays 1 */
			x->rxDr = x->rxShiftVal;
			x->rxShiftFull = 0;
			sr1Clr(i, SR1_BTF);
		} else {
			sr1Clr(i, SR1_RXNE | SR1_BTF);
		}
		if (msl(i) && x->rxActive && !x->rxInFlight && !x->rxShiftFull) rxStart(i);
		evalController(i);
	}
}

static void peDisabled(unsigned i) {
	resetEngine(i);
	poke(reg(i, R_SR1), 0u);
	poke(reg(i, R_SR2), sr2(i) & SR2_BUSY);
	cr1Clr(i, CR1_ACK | CR1_POS | CR1_PEC | CR1_ALERT | CR1_START);
}

static void afterWrite(unsigned i, uintptr_t a, uint32_t oldValue, uint32_t written) {
	I2cState *x = &st[i];
	const uint32_t off = (uint32_t)(a - bases[i]);
	switch (off) {
	case R_CR1: {
		uint32_t nw = written & CR1_MASK;
		if ((nw & CR1_SWRST) != 0u) {
			if ((oldValue & CR1_SWRST) == 0u) x->swrst++;
			resetEngine(i);
			poke(reg(i, R_SR1), 0u);
			poke(reg(i, R_SR2), x->busLocked ? SR2_BUSY : 0u);
			poke(a, CR1_SWRST);
			break;
		}
		if ((oldValue & CR1_SWRST) != 0u) { /* leaving the reset state */
			poke(a, nw);
			break;
		}
		if ((oldValue & CR1_PE) != 0u && (nw & CR1_PE) == 0u && ((sr2(i) & SR2_MSL) != 0u && (sr2(i) & SR2_BUSY) != 0u)) x->peOffBusy++;
		if ((nw & CR1_PE) == 0u && (nw & CR1_START) != 0u) nw &= ~CR1_START; /* cleared by hardware when PE = 0 */
		poke(a, nw);
		if ((oldValue & CR1_PE) != 0u && (nw & CR1_PE) == 0u) {
			peDisabled(i);
		} else {
			if (x->srRead && (sr1(i) & SR1_STOPF) != 0u) sr1Clr(i, SR1_STOPF); /* read SR1, then write CR1 */
		}
		x->srRead = 0;
		evalController(i);
		break;
	}
	case R_CR2: poke(a, written & CR2_MASK); break;
	case R_OAR1: poke(a, written & OAR1_MASK); break;
	case R_OAR2: poke(a, written & OAR2_MASK); break;
	case R_CCR:
	case R_TRISE: {
		const uint32_t mask = off == R_CCR ? CCR_MASK : TRISE_MASK;
		if (peOn(i) && ((oldValue ^ written) & mask) != 0u) x->configWhileEnabled++;
		poke(a, written & mask);
		break;
	}
	case R_SR1: poke(a, oldValue & (written | ~SR1_RC_W0)); break;
	case R_DR: {
		const uint8_t v = (uint8_t)(written & 0xFFu);
		poke(a, v);
		evalController(i);
		if (x->mode == M_ADDRWAIT && msl(i) && (sr1(i) & (SR1_SB | SR1_ADD10)) != 0u) { /* address byte (EV5 / EV9) */
			if (x->srRead) {
				sr1Clr(i, SR1_SB | SR1_ADD10);
			} else {
				x->seqBreaks++;
			}
			x->addrByte = v;
			x->mode = M_ADDRFLIGHT;
			x->addrDoneAt = now() + byteTicks;
		} else if (msl(i) && x->mode == M_TX) { /* controller transmitter */
			sr1Clr(i, SR1_BTF);
			if (!x->shiftBusy) {
				startShift(i, v);
			} else {
				x->drVal = v;
				x->drFull = 1;
				sr1Clr(i, SR1_TXE);
			}
		} else if (!msl(i) && x->slActive && tra(i)) { /* target transmitter */
			sr1Clr(i, SR1_BTF);
			if (!x->shiftLoaded) {
				x->shiftVal = v;
				x->shiftLoaded = 1;
			} else {
				x->drVal = v;
				x->drFull = 1;
				sr1Clr(i, SR1_TXE);
			}
		} else {
			x->strayDr++;
			sr1Clr(i, SR1_RXNE); /* "cleared by software reading or writing the DR register" (27.6.6) */
		}
		x->srRead = 0;
		evalController(i);
		break;
	}
	default: poke(a, oldValue); break;
	}
}

static void resetAll(unsigned i) {
	poke(reg(i, R_TRISE), 0x0002u);
}

#define I2C_THUNKS(N)                                                                          \
	static int canWrite##N(uintptr_t a) { return canWrite(N, a); }                             \
	static int beforeRead##N(uintptr_t a) { return beforeRead(N, a); }                         \
	static void afterRead##N(uintptr_t a, uint32_t v) { afterRead(N, a, v); }                  \
	static void afterWrite##N(uintptr_t a, uint32_t o, uint32_t w) { afterWrite(N, a, o, w); } \
	static void reset##N(void) { resetAll(N); }
I2C_THUNKS(0)
I2C_THUNKS(1)
I2C_THUNKS(2)

static const HostRegModel i2cModels[HOST_I2C_COUNT] = {
	{"I2C1", 0x40005400u, 0x400u, reset0, beforeRead0, afterRead0, canWrite0, afterWrite0},
	{"I2C2", 0x40005800u, 0x400u, reset1, beforeRead1, afterRead1, canWrite1, afterWrite1},
	{"I2C3", 0x40005C00u, 0x400u, reset2, beforeRead2, afterRead2, canWrite2, afterWrite2},
};

void hostI2cRegister(void) {
	for (unsigned i = 0; i < HOST_I2C_COUNT; ++i) hostRegsAddModel(&i2cModels[i]);
}
