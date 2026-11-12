/*
 * HalUart.c: USART1..USART3, UART4, UART5, USART6 register model (P12 T4b). See HalUart.h for what is modelled and the RM0090 sources.
 * Register access goes through hostRegsPeek / hostRegsPoke (raw, no trapping).
 */
#include "HalUart.h"

#include <stddef.h>

#include "HalHostSystem.h"

#define R_SR 0x00u
#define R_DR 0x04u
#define R_BRR 0x08u
#define R_CR1 0x0Cu
#define R_CR2 0x10u
#define R_CR3 0x14u
#define R_GTPR 0x18u

#define SR_PE (1u << 0)
#define SR_FE (1u << 1)
#define SR_NF (1u << 2)
#define SR_ORE (1u << 3)
#define SR_IDLE (1u << 4)
#define SR_RXNE (1u << 5)
#define SR_TC (1u << 6)
#define SR_TXE (1u << 7)
#define SR_LBD (1u << 8)
#define SR_CTS (1u << 9)
#define SR_RC_W0 (SR_RXNE | SR_TC | SR_LBD | SR_CTS)
#define SR_ERRORS (SR_PE | SR_FE | SR_NF | SR_ORE | SR_IDLE)

#define CR1_SBK (1u << 0)
#define CR1_RWU (1u << 1)
#define CR1_RE (1u << 2)
#define CR1_TE (1u << 3)
#define CR1_PS (1u << 9)
#define CR1_PCE (1u << 10)
#define CR1_WAKE (1u << 11)
#define CR1_M (1u << 12)
#define CR1_UE (1u << 13)
#define CR1_OVER8 (1u << 15)
#define CR1_MASK 0xBFFFu
#define CR1_RESTRICTED (CR1_M | CR1_PCE | CR1_PS | CR1_OVER8)

#define CR2_LINEN (1u << 14)
#define CR2_STOP (3u << 12)
#define CR2_MASK 0x7F6Fu
#define CR2_MASK_UART45 0x706Fu /* no CLKEN, CPOL, CPHA, LBCL */

#define CR3_SCEN (1u << 5)
#define CR3_IREN (1u << 1)
#define CR3_ONEBIT (1u << 11)
#define CR3_MASK 0x0FFFu
#define CR3_MASK_UART45 0x08CFu /* no CTSIE, CTSE, RTSE, SCEN, NACK */

#define PEER_MAX 64u
#define TXLOG_MAX 512u

static const uintptr_t bases[HOST_UART_COUNT] = {0x40011000u, 0x40004400u, 0x40004800u, 0x40004C00u, 0x40005000u, 0x40011400u};

typedef struct {
	uint8_t busy, isBreak, txFull;
	uint16_t txWord;
	uint32_t doneAt;
	uint16_t rxData;
	uint8_t srRead, rxSinceIdle, stuck;
	uint16_t peer[PEER_MAX];
	uint8_t peerErr[PEER_MAX];
	unsigned peerHead, peerCount;
	uint32_t rxReadyAt;
	uint16_t txLog[TXLOG_MAX];
	unsigned txCount, breaks;
	uint32_t configWhileEnabled, disabledBusy;
} UartState;

static UartState us[HOST_UART_COUNT];
static unsigned frameTicks = 1U;
static unsigned pollReads, pollCount;
static uint32_t virtualBonus;

static uint32_t peek(uintptr_t a) { return hostRegsPeek(a); }
static void poke(uintptr_t a, uint32_t v) { hostRegsPoke(a, v); }
static uintptr_t reg(unsigned i, uint32_t r) { return bases[i] + r; }
static uint32_t now(void) { return hostTickPeek() + virtualBonus; }
static int isUart45(unsigned i) { return i == 3u || i == 4u; }

void hostUartResetState(void) {
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) us[i] = (UartState){0};
	frameTicks = 1U;
	pollReads = pollCount = 0;
	virtualBonus = 0;
}
void hostUartSetFrameTicks(unsigned t) { frameTicks = t == 0u ? 1u : t; }
void hostUartSetPollReads(unsigned n) { pollReads = n; pollCount = 0; }
void hostUartTxStuck(unsigned i, int s) { if (i < HOST_UART_COUNT) us[i].stuck = s != 0; }
unsigned hostUartTxCount(unsigned i) { return i < HOST_UART_COUNT ? us[i].txCount : 0u; }
uint16_t hostUartTxAt(unsigned i, unsigned k) { return (i < HOST_UART_COUNT && k < us[i].txCount) ? us[i].txLog[k] : 0u; }
void hostUartTxClear(unsigned i) { if (i < HOST_UART_COUNT) us[i].txCount = 0; }
unsigned hostUartBreakCount(unsigned i) { return i < HOST_UART_COUNT ? us[i].breaks : 0u; }
uint32_t hostUartConfigWhileEnabled(unsigned i) { return i < HOST_UART_COUNT ? us[i].configWhileEnabled : 0u; }
uint32_t hostUartDisabledBusy(unsigned i) { return i < HOST_UART_COUNT ? us[i].disabledBusy : 0u; }

static uint32_t cr1(unsigned i) { return peek(reg(i, R_CR1)); }
static int ueOn(unsigned i) { return (cr1(i) & CR1_UE) != 0u; }
static int txOn(unsigned i) { return ueOn(i) && (cr1(i) & CR1_TE) != 0u; }
static int rxOn(unsigned i) { return ueOn(i) && (cr1(i) & CR1_RE) != 0u; }
static unsigned popcount(uint32_t v) {
	unsigned n = 0;
	for (; v != 0u; v >>= 1) n += v & 1u;
	return n;
}
static void srSet(unsigned i, uint32_t m) { poke(reg(i, R_SR), peek(reg(i, R_SR)) | m); }
static void srClear(unsigned i, uint32_t m) { poke(reg(i, R_SR), peek(reg(i, R_SR)) & ~m); }

/* ------------------------------------------------------------------ transmitter */
static uint16_t lineBits(unsigned i, uint32_t dr) {
	const uint32_t c = cr1(i);
	if ((c & CR1_PCE) == 0u) return (uint16_t)(dr & ((c & CR1_M) ? 0x1FFu : 0xFFu));
	{
		const unsigned nd = (c & CR1_M) ? 8u : 7u;
		const uint32_t data = dr & ((1u << nd) - 1u);
		const uint32_t parity = (popcount(data) & 1u) ^ ((c & CR1_PS) ? 1u : 0u); /* even: parity = xor of the data; odd: inverted */
		return (uint16_t)(data | (parity << nd));
	}
}

static void beginFrame(unsigned i, uint16_t dr) {
	UartState *x = &us[i];
	const uint16_t bits = lineBits(i, dr);
	if (x->txCount < TXLOG_MAX) x->txLog[x->txCount++] = bits;
	x->busy = 1;
	x->isBreak = 0;
	x->doneAt = now() + frameTicks;
}

static void startIfPossible(unsigned i) {
	UartState *x = &us[i];
	if (x->busy || !txOn(i)) return;
	if ((cr1(i) & CR1_SBK) != 0u) { /* a break goes after the current frame (RM0090 30.3.2) */
		x->busy = 1;
		x->isBreak = 1;
		x->breaks++;
		x->doneAt = now() + frameTicks;
	} else if (x->txFull) {
		x->txFull = 0;
		srSet(i, SR_TXE);
		beginFrame(i, x->txWord);
	}
}

/* ------------------------------------------------------------------ receiver */
static void deliver(unsigned i, uint16_t bits, uint32_t err) {
	UartState *x = &us[i];
	const uint32_t c = cr1(i);
	const unsigned m = (c & CR1_M) != 0u, pce = (c & CR1_PCE) != 0u, ps = (c & CR1_PS) != 0u;
	int addressFrame = 0;
	if ((c & CR1_RWU) != 0u && (c & CR1_WAKE) == 0u) return; /* mute mode, idle line wake-up: frames are dropped */
	if ((c & CR1_WAKE) != 0u) { /* address mark detection (RM0090 30.3.6) */
		const unsigned msb = pce ? (m ? 7u : 6u) : (m ? 8u : 7u);
		const int isAddress = ((bits >> msb) & 1u) != 0u;
		const int match = (bits & 0xFu) == (peek(reg(i, R_CR2)) & 0xFu);
		if (isAddress && !match) { /* a non-matching address: mute mode entered by hardware, no RXNE for this frame */
			poke(reg(i, R_CR1), c | CR1_RWU);
			return;
		}
		if ((c & CR1_RWU) != 0u) {
			if (!isAddress) return; /* data in mute mode */
			poke(reg(i, R_CR1), c & ~CR1_RWU); /* matching address: wake up, the frame is received */
		}
		addressFrame = isAddress;
	}
	{
		uint32_t sr = peek(reg(i, R_SR));
		if (sr & SR_RXNE) { /* RM0090 30.3.3: the new frame is lost, RDR keeps its content */
			poke(reg(i, R_SR), sr | SR_ORE);
			return;
		}
		x->rxData = (uint16_t)(bits & (m ? 0x1FFu : 0xFFu));
		sr |= SR_RXNE;
		x->rxSinceIdle = 1;
		if (pce && !addressFrame && (popcount(bits & (m ? 0x1FFu : 0xFFu)) & 1u) != (ps ? 1u : 0u)) sr |= SR_PE;
		if (err & HOST_UART_RX_FE) sr |= SR_FE;
		if ((err & HOST_UART_RX_NOISE) && (peek(reg(i, R_CR3)) & CR3_ONEBIT) == 0u) sr |= SR_NF;
		poke(reg(i, R_SR), sr);
	}
}

void hostUartRxPush(unsigned i, uint16_t bits, uint32_t err) {
	UartState *x = &us[i < HOST_UART_COUNT ? i : 0];
	if (i >= HOST_UART_COUNT || x->peerCount >= PEER_MAX) return;
	if (x->peerCount == 0u) x->rxReadyAt = now() + frameTicks;
	x->peer[(x->peerHead + x->peerCount) % PEER_MAX] = bits;
	x->peerErr[(x->peerHead + x->peerCount) % PEER_MAX] = (uint8_t)err;
	x->peerCount++;
}
void hostUartRxNow(unsigned i, uint16_t bits, uint32_t err) {
	if (i < HOST_UART_COUNT && rxOn(i)) deliver(i, bits, err);
}
void hostUartRxBreak(unsigned i) { hostUartRxNow(i, 0u, HOST_UART_RX_FE); }

void hostUartIdleLine(unsigned i) {
	if (i >= HOST_UART_COUNT || !rxOn(i)) return;
	{
		const uint32_t c = cr1(i);
		if ((c & CR1_RWU) != 0u && (c & CR1_WAKE) == 0u) {
			poke(reg(i, R_CR1), c & ~CR1_RWU); /* wakes up, IDLE is not set (RM0090 30.3.6) */
			return;
		}
	}
	if (us[i].rxSinceIdle) {
		srSet(i, SR_IDLE);
		us[i].rxSinceIdle = 0;
	}
}

/* ------------------------------------------------------------------ time */
static void uartEval(unsigned i) {
	UartState *x = &us[i];
	for (unsigned guard = 0; guard < 1000u && x->busy && !x->stuck && (int32_t)(now() - x->doneAt) >= 0; ++guard) {
		x->busy = 0;
		if (x->isBreak) {
			x->isBreak = 0;
			poke(reg(i, R_CR1), cr1(i) & ~CR1_SBK); /* cleared by hardware during the stop bit of the break */
		} else if (!x->txFull) {
			srSet(i, SR_TC | SR_TXE); /* frame complete and TXE set (RM0090 30.6.1) */
		}
		startIfPossible(i);
	}
	for (unsigned guard = 0; guard < 1000u && x->peerCount != 0u && rxOn(i) && (int32_t)(now() - x->rxReadyAt) >= 0; ++guard) {
		const uint16_t bits = x->peer[x->peerHead];
		const uint32_t err = x->peerErr[x->peerHead];
		x->peerHead = (x->peerHead + 1u) % PEER_MAX;
		x->peerCount--;
		x->rxReadyAt += frameTicks;
		deliver(i, bits, err);
	}
}

/* ------------------------------------------------------------------ bus callbacks */
static int known(unsigned i, uintptr_t a) {
	(void)i;
	const uint32_t off = (uint32_t)(a - bases[i]);
	return off <= R_GTPR && (off & 3u) == 0u;
}
static int beforeRead(unsigned i, uintptr_t a) {
	if (!known(i, a)) return 0;
	{
		const uint32_t off = (uint32_t)(a - bases[i]);
		if (off == R_SR && pollReads != 0u && ++pollCount >= pollReads) {
			pollCount = 0;
			virtualBonus++;
		}
		if (off == R_SR || off == R_DR) uartEval(i);
		if (off == R_DR) poke(a, us[i].rxData);
	}
	return 1;
}
static void afterRead(unsigned i, uintptr_t a, uint32_t value) {
	(void)value;
	const uint32_t off = (uint32_t)(a - bases[i]);
	if (off == R_SR) {
		us[i].srRead = 1;
	} else if (off == R_DR) {
		poke(a, 0u);
		srClear(i, SR_RXNE);
		if (us[i].srRead) srClear(i, SR_ERRORS); /* read SR, then read DR (RM0090 30.6.1) */
		us[i].srRead = 0;
	}
}

static void forceOver8(unsigned i) {
	if ((peek(reg(i, R_CR2)) & CR2_LINEN) != 0u || (peek(reg(i, R_CR3)) & (CR3_IREN | CR3_SCEN)) != 0u)
		poke(reg(i, R_CR1), cr1(i) & ~CR1_OVER8);
}

static void afterWrite(unsigned i, uintptr_t a, uint32_t oldValue, uint32_t written) {
	const uint32_t off = (uint32_t)(a - bases[i]);
	UartState *x = &us[i];
	switch (off) {
	case R_SR: {
		/* TC, RXNE, LBD, CTS clear by writing 0; the other bits are hardware (RM0090 30.6.1) */
		poke(a, oldValue & (written | ~SR_RC_W0));
		break;
	}
	case R_DR: {
		uartEval(i);
		x->txWord = (uint16_t)(written & 0x1FFu);
		poke(a, 0u);
		if (x->srRead) srClear(i, SR_TC | SR_PE); /* read SR, then write DR clears TC (and PE) */
		x->srRead = 0;
		if (txOn(i) && !x->busy && !x->txFull) {
			beginFrame(i, x->txWord);
		} else {
			x->txFull = 1;
			srClear(i, SR_TXE);
		}
		break;
	}
	case R_BRR:
		if (ueOn(i) && ((oldValue ^ written) & 0xFFFFu) != 0u) x->configWhileEnabled++;
		poke(a, written & 0xFFFFu);
		break;
	case R_CR1: {
		uint32_t nw = written & CR1_MASK;
		uartEval(i);
		{
			const int wasOn = (oldValue & CR1_UE) != 0u, isOn = (nw & CR1_UE) != 0u;
			if (wasOn && isOn && ((oldValue ^ nw) & CR1_RESTRICTED) != 0u) x->configWhileEnabled++;
			if (wasOn && !isOn && (x->busy || x->txFull)) x->disabledBusy++;
		}
		if ((nw & CR1_WAKE) != 0u && (peek(reg(i, R_SR)) & SR_RXNE) != 0u) nw = (nw & ~CR1_RWU) | (oldValue & CR1_RWU); /* RWU cannot change with RXNE = 1 and WAKE = 1 */
		poke(a, nw);
		forceOver8(i);
		startIfPossible(i);
		break;
	}
	case R_CR2:
		if (ueOn(i) && ((oldValue ^ written) & CR2_STOP) != 0u) x->configWhileEnabled++;
		poke(a, written & (isUart45(i) ? CR2_MASK_UART45 : CR2_MASK));
		forceOver8(i);
		break;
	case R_CR3:
		poke(a, written & (isUart45(i) ? CR3_MASK_UART45 : CR3_MASK));
		forceOver8(i);
		break;
	case R_GTPR: poke(a, written & 0xFFFFu); break;
	default: poke(a, oldValue); break;
	}
}

static void resetAll(unsigned i) { poke(reg(i, R_SR), SR_TC | SR_TXE); }

#define UART_THUNKS(N)                                                                               \
	static int known##N(uintptr_t a) { return known(N, a); }                                         \
	static int beforeRead##N(uintptr_t a) { return beforeRead(N, a); }                               \
	static void afterRead##N(uintptr_t a, uint32_t v) { afterRead(N, a, v); }                        \
	static void afterWrite##N(uintptr_t a, uint32_t o, uint32_t w) { afterWrite(N, a, o, w); }       \
	static void reset##N(void) { resetAll(N); }
UART_THUNKS(0)
UART_THUNKS(1)
UART_THUNKS(2)
UART_THUNKS(3)
UART_THUNKS(4)
UART_THUNKS(5)

static const HostRegModel uartModels[HOST_UART_COUNT] = {
	{"USART1", 0x40011000u, 0x400u, reset0, beforeRead0, afterRead0, known0, afterWrite0},
	{"USART2", 0x40004400u, 0x400u, reset1, beforeRead1, afterRead1, known1, afterWrite1},
	{"USART3", 0x40004800u, 0x400u, reset2, beforeRead2, afterRead2, known2, afterWrite2},
	{"UART4", 0x40004C00u, 0x400u, reset3, beforeRead3, afterRead3, known3, afterWrite3},
	{"UART5", 0x40005000u, 0x400u, reset4, beforeRead4, afterRead4, known4, afterWrite4},
	{"USART6", 0x40011400u, 0x400u, reset5, beforeRead5, afterRead5, known5, afterWrite5},
};

void hostUartRegister(void) {
	for (unsigned i = 0; i < HOST_UART_COUNT; ++i) hostRegsAddModel(&uartModels[i]);
}
