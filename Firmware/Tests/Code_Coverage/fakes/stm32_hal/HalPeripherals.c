/*
 * HalPeripherals.c: the T1 register models; see HalPeripherals.h for what is modelled and the RM0090 sources.
 * All register access here goes through hostRegsPeek / hostRegsPoke (raw, no trapping).
 */
#include "HalPeripherals.h"

#include "HalClockPower.h"
#include "HalCorePeripherals.h"
#include "HalDma.h"
#include "HalSpi.h"
#include "HalUart.h"
#include "HalRegisterSpace.h"

/* ---------------------------------------------------------------- addresses (RM0090 Table 1) */
#define GPIO_BASE_ADDR 0x40020000u
#define GPIO_STRIDE 0x400u
#define EXTI_BASE_ADDR 0x40013C00u
#define SYSCFG_BASE_ADDR 0x40013800u
#define RCC_BASE_ADDR 0x40023800u
#define FLASHIF_BASE_ADDR 0x40023C00u
#define DBGMCU_BASE_ADDR 0xE0042000u
#define UID_BASE_ADDR 0x1FFF7A10u

#define RCC_AHB1ENR_OFF 0x30u

/* GPIO register offsets (RM0090 8.4.11) */
enum { G_MODER = 0x00, G_OTYPER = 0x04, G_OSPEEDR = 0x08, G_PUPDR = 0x0C, G_IDR = 0x10, G_ODR = 0x14, G_BSRR = 0x18,
	   G_LCKR = 0x1C, G_AFRL = 0x20, G_AFRH = 0x24 };
/* EXTI register offsets (RM0090 12.3.7) */
enum { E_IMR = 0x00, E_EMR = 0x04, E_RTSR = 0x08, E_FTSR = 0x0C, E_SWIER = 0x10, E_PR = 0x14 };
#define EXTI_LINES_MASK 0x007FFFFFu /* lines 0..22 */
#define LCKR_LCKK (1u << 16)

/* ---------------------------------------------------------------- state outside the registers */
static uint8_t extLevel[HOST_GPIO_PORTS][16]; /* HostPinLevel */
static uint8_t lockStep[HOST_GPIO_PORTS];      /* 0 idle, 1..3 writes of the key sequence done */
static uint16_t lockSeqValue[HOST_GPIO_PORTS];
static uint8_t portLocked[HOST_GPIO_PORTS];
static uint16_t lockedPins[HOST_GPIO_PORTS];
static uint32_t clockOffAccesses;
static uint32_t floatingReads;
static uint8_t internalLine[23];  /* levels of EXTI lines 16..22 */
static uint8_t lastLineLevel[23]; /* last sampled level of every EXTI line */
static uint32_t extiEvents;
static uint32_t dbgIdcode;
static uint32_t uidWords[3];
static uint16_t flashSizeKib;

static int latencyStuck;
static uint32_t peek(uintptr_t a) { return hostRegsPeek(a); }
static void poke(uintptr_t a, uint32_t v) { hostRegsPoke(a, v); }

void hostPeriphResetState(void) {
	latencyStuck = 0;
	hostClockPowerResetState();
	hostDmaResetState();
	hostSpiResetState();
	hostUartResetState();
	for (unsigned p = 0; p < HOST_GPIO_PORTS; ++p) {
		for (unsigned i = 0; i < 16; ++i) extLevel[p][i] = HOST_PIN_FLOAT;
		lockStep[p] = 0;
		lockSeqValue[p] = 0;
		portLocked[p] = 0;
		lockedPins[p] = 0;
	}
	clockOffAccesses = 0;
	floatingReads = 0;
	for (unsigned l = 0; l < 23; ++l) {
		internalLine[l] = 0;
		lastLineLevel[l] = 0;
	}
	extiEvents = 0;
	dbgIdcode = (0x1007u << 16) | 0x413u;
	uidWords[0] = 0x12345678u;
	uidWords[1] = 0x9ABCDEF0u;
	uidWords[2] = 0x0F1E2D3Cu;
	flashSizeKib = 1024u;
	hostCoreResetState();
}

/* ---------------------------------------------------------------- GPIO pin levels */
static int pinLevelRaw(unsigned port, unsigned pin, int countFloating) {
	const uintptr_t b = GPIO_BASE_ADDR + port * GPIO_STRIDE;
	const uint32_t mode = (peek(b + G_MODER) >> (2u * pin)) & 3u;
	const uint32_t pull = (peek(b + G_PUPDR) >> (2u * pin)) & 3u;
	const int ext = extLevel[port][pin];
	if (mode == 3u) return 0; /* analog: IDR reads 0 (8.3.12) */
	if (mode == 1u) {          /* general purpose output */
		const int odr = (int)((peek(b + G_ODR) >> pin) & 1u);
		const int openDrain = (int)((peek(b + G_OTYPER) >> pin) & 1u);
		if (!openDrain || odr == 0) return odr;
	}
	if (ext == HOST_PIN_HIGH) return 1;
	if (ext == HOST_PIN_LOW) return 0;
	if (pull == 1u) return 1; /* pull-up */
	if (pull == 2u) return 0; /* pull-down */
	if (countFloating) floatingReads++;
	return 0;
}

static int lineLevel(unsigned line) {
	if (line < 16u) {
		const uint32_t cr = peek(SYSCFG_BASE_ADDR + 0x08u + 4u * (line >> 2));
		const unsigned port = (cr >> (4u * (line & 3u))) & 0xFu;
		if (port >= HOST_GPIO_PORTS) return 0;
		return pinLevelRaw(port, line, 0);
	}
	return internalLine[line] != 0;
}

/* Samples every EXTI line; an edge selected in RTSR / FTSR sets PR (if IMR) and counts an event (if EMR). */
static void extiSample(int edgesAllowed) {
	const uint32_t rtsr = peek(EXTI_BASE_ADDR + E_RTSR), ftsr = peek(EXTI_BASE_ADDR + E_FTSR);
	const uint32_t imr = peek(EXTI_BASE_ADDR + E_IMR), emr = peek(EXTI_BASE_ADDR + E_EMR);
	uint32_t pr = peek(EXTI_BASE_ADDR + E_PR);
	for (unsigned line = 0; line < 23u; ++line) {
		const int now = lineLevel(line);
		const uint32_t bit = 1u << line;
		if (edgesAllowed && now != lastLineLevel[line]) {
			const int selected = now ? (rtsr & bit) != 0 : (ftsr & bit) != 0;
			if (selected) {
				if (imr & bit) pr |= bit;
				if (emr & bit) extiEvents++;
			}
		}
		lastLineLevel[line] = (uint8_t)now;
	}
	poke(EXTI_BASE_ADDR + E_PR, pr);
}

void hostGpioSetInput(unsigned port, unsigned pin, HostPinLevel level) {
	if (port >= HOST_GPIO_PORTS || pin >= 16u) return;
	extLevel[port][pin] = (uint8_t)level;
	extiSample(1);
}

int hostGpioPinLevel(unsigned port, unsigned pin) {
	if (port >= HOST_GPIO_PORTS || pin >= 16u) return 0;
	return pinLevelRaw(port, pin, 0);
}

int hostGpioLocked(unsigned port, unsigned pin) {
	if (port >= HOST_GPIO_PORTS || pin >= 16u) return 0;
	return portLocked[port] && ((lockedPins[port] >> pin) & 1u);
}

uint32_t hostGpioClockOffAccesses(void) { return clockOffAccesses; }
uint32_t hostGpioFloatingReads(void) { return floatingReads; }

/* ---------------------------------------------------------------- GPIO model */
static unsigned gpioPort(uintptr_t a) { return (unsigned)((a - GPIO_BASE_ADDR) / GPIO_STRIDE); }
static uint32_t gpioOff(uintptr_t a) { return (uint32_t)((a - GPIO_BASE_ADDR) % GPIO_STRIDE); }

static int gpioKnownOffset(uint32_t off) { return off <= G_AFRH; }

static void gpioCountClock(unsigned port) {
	if (((peek(RCC_BASE_ADDR + RCC_AHB1ENR_OFF) >> port) & 1u) == 0u) clockOffAccesses++;
}

static void gpioReset(void) {
	for (unsigned p = 0; p < HOST_GPIO_PORTS; ++p) {
		const uintptr_t b = GPIO_BASE_ADDR + p * GPIO_STRIDE;
		poke(b + G_MODER, p == 0 ? 0xA8000000u : p == 1 ? 0x00000280u : 0u);
		poke(b + G_OSPEEDR, p == 0 ? 0x0C000000u : p == 1 ? 0x000000C0u : 0u);
		poke(b + G_PUPDR, p == 0 ? 0x64000000u : p == 1 ? 0x00000100u : 0u);
	}
}

static int gpioBeforeRead(uintptr_t a) {
	const unsigned port = gpioPort(a);
	const uint32_t off = gpioOff(a);
	if (!gpioKnownOffset(off)) return 0;
	gpioCountClock(port);
	if (off == G_IDR) {
		uint32_t v = 0;
		for (unsigned pin = 0; pin < 16u; ++pin) v |= (uint32_t)pinLevelRaw(port, pin, 1) << pin;
		poke(a, v);
	} else if (off == G_BSRR) {
		poke(a, 0); /* write-only: reads 0 */
	} else if (off == G_LCKR) {
		if (!portLocked[port]) {
			if (lockStep[port] == 3u) { /* the mandatory read of the key sequence: the lock becomes active */
				portLocked[port] = 1;
				lockedPins[port] = lockSeqValue[port];
			}
			lockStep[port] = 0; /* a read before the third write is an error in the sequence: abort */
		}
		poke(a, portLocked[port] ? (LCKR_LCKK | lockedPins[port]) : (peek(a) & 0xFFFFu));
	}
	return 1;
}

static int gpioBeforeWrite(uintptr_t a) {
	const uint32_t off = gpioOff(a);
	if (!gpioKnownOffset(off) || off == G_IDR) return 0; /* reserved offset or read-only IDR */
	gpioCountClock(gpioPort(a));
	return 1;
}

/* Bits of a configuration register that belong to locked pins. */
static uint32_t frozenMask(uint32_t off, uint16_t pins) {
	uint32_t m = 0;
	for (unsigned pin = 0; pin < 16u; ++pin) {
		if (!((pins >> pin) & 1u)) continue;
		switch (off) {
		case G_MODER: case G_OSPEEDR: case G_PUPDR: m |= 3u << (2u * pin); break;
		case G_OTYPER: m |= 1u << pin; break;
		case G_AFRL: if (pin < 8u) m |= 0xFu << (4u * pin); break;
		case G_AFRH: if (pin >= 8u) m |= 0xFu << (4u * (pin - 8u)); break;
		default: break;
		}
	}
	return m;
}

static void gpioAfterWrite(uintptr_t a, uint32_t oldValue, uint32_t written) {
	const unsigned port = gpioPort(a);
	const uint32_t off = gpioOff(a);
	const uintptr_t b = GPIO_BASE_ADDR + port * GPIO_STRIDE;
	switch (off) {
	case G_MODER: case G_OSPEEDR: case G_PUPDR: case G_OTYPER: case G_AFRL: case G_AFRH: {
		uint32_t v = written;
		if (off == G_OTYPER) v &= 0xFFFFu;
		if (portLocked[port]) {
			const uint32_t fz = frozenMask(off, lockedPins[port]);
			v = (v & ~fz) | (oldValue & fz);
		}
		poke(a, v);
		break;
	}
	case G_ODR: poke(a, written & 0xFFFFu); break;
	case G_BSRR: {
		const uint32_t set = written & 0xFFFFu;
		const uint32_t reset = (written >> 16) & 0xFFFFu & ~set; /* BSy has priority */
		poke(b + G_ODR, (peek(b + G_ODR) & ~reset) | set);
		poke(a, 0);
		break;
	}
	case G_LCKR: {
		if (portLocked[port]) { /* locked until reset: writes ignored */
			poke(a, oldValue);
			break;
		}
		const uint16_t v = (uint16_t)(written & 0xFFFFu);
		const int key = (written & LCKR_LCKK) != 0;
		switch (lockStep[port]) {
		case 0: lockStep[port] = key ? 1u : 0u; lockSeqValue[port] = v; break;
		case 1: lockStep[port] = (!key && v == lockSeqValue[port]) ? 2u : (key ? 1u : 0u); if (key) lockSeqValue[port] = v; break;
		case 2: lockStep[port] = (key && v == lockSeqValue[port]) ? 3u : 0u; break;
		default: lockStep[port] = key ? 1u : 0u; lockSeqValue[port] = v; break; /* write instead of the read: restart */
		}
		poke(a, v); /* LCK[15:0] writable while LCKK is 0; LCKK reads 0 until locked */
		break;
	}
	default: break;
	}
	extiSample(1);
}

static const HostRegModel gpioModel = {"GPIOA..GPIOI", GPIO_BASE_ADDR, HOST_GPIO_PORTS * GPIO_STRIDE, gpioReset,
									   gpioBeforeRead, NULL, gpioBeforeWrite, gpioAfterWrite};

/* ---------------------------------------------------------------- EXTI model */
static int extiBeforeAccess(uintptr_t a) { return (a - EXTI_BASE_ADDR) <= E_PR; }

static void extiAfterWrite(uintptr_t a, uint32_t oldValue, uint32_t written) {
	const uint32_t off = (uint32_t)(a - EXTI_BASE_ADDR);
	const uint32_t w = written & EXTI_LINES_MASK;
	switch (off) {
	case E_IMR: case E_EMR: case E_RTSR: case E_FTSR: poke(a, w); break;
	case E_SWIER: {
		const uint32_t rising = w & ~oldValue; /* 1 written to a bit that is 0 */
		const uint32_t imr = peek(EXTI_BASE_ADDR + E_IMR), emr = peek(EXTI_BASE_ADDR + E_EMR);
		poke(EXTI_BASE_ADDR + E_PR, peek(EXTI_BASE_ADDR + E_PR) | (rising & imr));
		for (unsigned l = 0; l < 23u; ++l)
			if ((rising & emr) & (1u << l)) extiEvents++;
		poke(a, (oldValue | w) & EXTI_LINES_MASK);
		break;
	}
	case E_PR:
		poke(a, oldValue & ~w);
		poke(EXTI_BASE_ADDR + E_SWIER, peek(EXTI_BASE_ADDR + E_SWIER) & ~w);
		break;
	default: break;
	}
}

static const HostRegModel extiModel = {"EXTI", EXTI_BASE_ADDR, 0x400u, NULL, extiBeforeAccess, NULL, extiBeforeAccess,
									   extiAfterWrite};

void hostExtiSetLine(unsigned line, int level) {
	if (line < 16u || line > 22u) return;
	internalLine[line] = level != 0;
	extiSample(1);
}

uint32_t hostExtiEventCount(void) { return extiEvents; }

/* ---------------------------------------------------------------- SYSCFG model */
static int syscfgKnown(uintptr_t a) {
	const uint32_t off = (uint32_t)(a - SYSCFG_BASE_ADDR);
	return off <= 0x14u || off == 0x20u;
}
static int syscfgBeforeWrite(uintptr_t a) { return syscfgKnown(a); }

static void syscfgAfterWrite(uintptr_t a, uint32_t oldValue, uint32_t written) {
	const uint32_t off = (uint32_t)(a - SYSCFG_BASE_ADDR);
	(void)oldValue;
	if (off == 0x00u) poke(a, written & 3u);
	else if (off == 0x04u) poke(a, written & (1u << 23));
	else if (off >= 0x08u && off <= 0x14u) {
		poke(a, written & 0xFFFFu);
		extiSample(0); /* new selection: resample without an edge */
	} else if (off == 0x20u) poke(a, (written & 1u) | ((written & 1u) << 8)); /* READY follows CMP_PD */
}

static const HostRegModel syscfgModel = {"SYSCFG", SYSCFG_BASE_ADDR, 0x400u, NULL, syscfgKnown, NULL, syscfgBeforeWrite,
										 syscfgAfterWrite};

/* ---------------------------------------------------------------- RCC model (T2: behaviour in HalClockPower.c) */
static void rccAfterWrite(uintptr_t a, uint32_t oldValue, uint32_t written) {
	hostRccAfterWrite(a, oldValue, written);
	if ((uint32_t)(a - RCC_BASE_ADDR) == RCC_AHB1ENR_OFF) extiSample(0);
}

static const HostRegModel rccModel = {"RCC", RCC_BASE_ADDR, 0x400u, hostRccReset, hostRccBeforeRead, NULL, hostRccKnown,
									  rccAfterWrite};

/* ---------------------------------------------------------------- FLASH interface (ACR only, T1) */
static int flashKnown(uintptr_t a) { return a == FLASHIF_BASE_ADDR; }
void hostFlashLatencyStuck(int stuck) { latencyStuck = stuck != 0; }
static void storeAsWritten(uintptr_t a, uint32_t oldValue, uint32_t written) {
	(void)oldValue;
	poke(a, written);
}
static void flashAcrWrite(uintptr_t a, uint32_t oldValue, uint32_t written) {
	/* RM0090 3.4 / 3.9.1: LATENCY[2:0] reads back the new value once effective; the knob keeps the old value. */
	poke(a, latencyStuck ? ((written & ~7u) | (oldValue & 7u)) : written);
}
static const HostRegModel flashModel = {"FLASH interface (ACR)", FLASHIF_BASE_ADDR, 0x400u, NULL, flashKnown, NULL,
										flashKnown, flashAcrWrite};

/* ---------------------------------------------------------------- DBGMCU */
static int dbgBeforeRead(uintptr_t a) {
	const uint32_t off = (uint32_t)(a - DBGMCU_BASE_ADDR);
	if (off > 0x0Cu) return 0;
	if (off == 0x00u) poke(a, dbgIdcode);
	return 1;
}
static int dbgBeforeWrite(uintptr_t a) {
	const uint32_t off = (uint32_t)(a - DBGMCU_BASE_ADDR);
	return off >= 0x04u && off <= 0x0Cu; /* IDCODE is read-only */
}
static const HostRegModel dbgModel = {"DBGMCU", DBGMCU_BASE_ADDR, 0x1000u, NULL, dbgBeforeRead, NULL, dbgBeforeWrite,
									  storeAsWritten};

void hostDbgSetIdcode(uint32_t idcode) { dbgIdcode = idcode; }

/* ---------------------------------------------------------------- unique id / flash size */
static int uidBeforeRead(uintptr_t a) {
	const uint32_t off = (uint32_t)(a - UID_BASE_ADDR);
	if (off <= 0x08u) {
		poke(a, uidWords[off / 4u]);
		return 1;
	}
	if (off == 0x10u) { /* word 0x1FFF7A20: flash size in the upper half-word (0x1FFF7A22) */
		poke(a, (uint32_t)flashSizeKib << 16);
		return 1;
	}
	return 0;
}
static int rejectWrite(uintptr_t a) {
	(void)a;
	return 0;
}
static const HostRegModel uidModel = {"unique id / flash size", UID_BASE_ADDR, 0x14u, NULL, uidBeforeRead, NULL,
									  rejectWrite, NULL};

void hostUidSet(uint32_t w0, uint32_t w1, uint32_t w2) {
	uidWords[0] = w0;
	uidWords[1] = w1;
	uidWords[2] = w2;
}
void hostFlashSizeSet(uint16_t kib) { flashSizeKib = kib; }

/* ---------------------------------------------------------------- registration */
void hostPeriphRegisterAll(void) {
	hostRegsAddModel(&gpioModel);
	hostRegsAddModel(&extiModel);
	hostRegsAddModel(&syscfgModel);
	hostRegsAddModel(&rccModel);
	hostRegsAddModel(&flashModel);
	hostRegsAddModel(&dbgModel);
	hostRegsAddModel(&uidModel);
	hostClockPowerRegister(); /* PWR, HalClockPower.h */
	hostDmaRegister(); /* DMA1, DMA2, HalDma.h */
	hostSpiRegister(); /* SPI1-SPI3, HalSpi.h */
	hostUartRegister(); /* USART1-3, UART4-5, USART6, HalUart.h */
	hostCoreRegister(); /* system control space, HalCorePeripherals.h */
}
