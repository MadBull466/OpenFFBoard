/*
 * HalFlash.c: embedded flash model (P12 T7). See HalFlash.h for what is modelled and the RM0090 sources.
 * Register access goes through hostRegsPeek / hostRegsPoke (raw, no trapping).
 */
#include "HalFlash.h"

#include <stddef.h>
#include <string.h>

#define FIF_BASE 0x40023C00u
#define FLASH_MEM_BASE 0x08000000u
#define FLASH_MEM_SIZE 0x00100000u
#define OTP_BASE 0x1FFF7800u
#define OTP_SIZE 0x210u
#define OTP_LOCK_BASE 0x1FFF7A00u
#define OPT_WORD0 0x1FFFC000u
#define OPT_WORD1 0x1FFFC008u

#define R_ACR 0x00u
#define R_KEYR 0x04u
#define R_OPTKEYR 0x08u
#define R_SR 0x0Cu
#define R_CR 0x10u
#define R_OPTCR 0x14u

#define SR_EOP 1u
#define SR_OPERR 2u
#define SR_WRPERR 0x10u
#define SR_PGAERR 0x20u
#define SR_PGPERR 0x40u
#define SR_PGSERR 0x80u
#define SR_BSY 0x10000u
#define SR_ERRORS (SR_OPERR | SR_WRPERR | SR_PGAERR | SR_PGPERR | SR_PGSERR)
#define CR_PG 1u
#define CR_SER 2u
#define CR_MER 4u
#define CR_SNB_SHIFT 3
#define CR_PSIZE_SHIFT 8
#define CR_STRT 0x10000u
#define CR_EOPIE 0x1000000u
#define CR_ERRIE 0x2000000u
#define CR_LOCK 0x80000000u
#define OPTCR_OPTLOCK 1u
#define OPTCR_OPTSTRT 2u

#define KEY1 0x45670123u
#define KEY2 0xCDEF89ABu
#define OPTKEY1 0x08192A3Bu
#define OPTKEY2 0x4C5D6E7Fu

extern int hostFlashLatencyIsStuck(void);

static struct {
	uint8_t keyState, optKeyState, keyLockout, optKeyLockout;
	uint8_t pending; /* 1 = an operation is running */
	uint8_t optPending;
	uint8_t x64Half; /* 1 = first word of an x64 program done, 2 = it failed: the second word is swallowed without a flag */
	uintptr_t crossAddr; /* word after an access that crosses a word boundary, and its content before the store */
	uint32_t crossOld;
	uint8_t crossing;
	unsigned pollReads, pollCount;
	uint32_t stalled, busFaults, lockedWrites;
} fs;

static uint32_t peek(uintptr_t a) { return hostRegsPeek(a); }
static void poke(uintptr_t a, uint32_t v) { hostRegsPoke(a, v); }
static uint32_t reg(uint32_t r) { return peek(FIF_BASE + r); }
static void put(uint32_t r, uint32_t v) { poke(FIF_BASE + r, v); }

void hostFlashSetPollReads(unsigned n) {
	fs.pollReads = n;
	fs.pollCount = 0;
}
uint32_t hostFlashStalledWrites(void) { return fs.stalled; }
uint32_t hostFlashBusFaults(void) { return fs.busFaults; }
uint32_t hostFlashLockedWrites(void) { return fs.lockedWrites; }

static const uint32_t sectorBase[13] = {0x08000000u, 0x08004000u, 0x08008000u, 0x0800C000u, 0x08010000u, 0x08020000u, 0x08040000u,
										0x08060000u, 0x08080000u, 0x080A0000u, 0x080C0000u, 0x080E0000u, 0x08100000u};
uint32_t hostFlashSectorBase(unsigned s) { return s <= 12u ? sectorBase[s] : 0u; }
uint32_t hostFlashSectorSize(unsigned s) { return s <= 11u ? sectorBase[s + 1u] - sectorBase[s] : 0u; }
int hostFlashSectorOf(uint32_t addr) {
	for (unsigned s = 0; s < 12u; ++s)
		if (addr >= sectorBase[s] && addr < sectorBase[s + 1u]) return (int)s;
	return -1;
}

void hostFlashFill(uint32_t addr, uint8_t value, uint32_t size) {
	if (addr >= FLASH_MEM_BASE && addr + size <= FLASH_MEM_BASE + FLASH_MEM_SIZE) { /* main flash: stays readable, only the stores are trapped */
		hostRegsFlashWritable(1);
		memset((void *)(uintptr_t)addr, value, size);
		hostRegsFlashWritable(0);
		return;
	}
	const int reprotect = hostRegsArmed(); /* OTP and the like: raw access with the bus disarmed */
	if (reprotect) hostRegsDisarm();
	memset((void *)(uintptr_t)addr, value, size);
	if (reprotect) hostRegsArm();
}

static void fillRaw(void) {
	memset((void *)(uintptr_t)FLASH_MEM_BASE, 0xFF, FLASH_MEM_SIZE);
	memset((void *)(uintptr_t)OTP_BASE, 0xFF, OTP_SIZE);
	poke(OPT_WORD0, 0x0000AAFFu); /* RDP 0xAA, USER 0xFF, as the reset value of OPTCR 0x0FFFAAED shows (RM0090 3.9.9) */
	poke(OPT_WORD1, 0x0000FFFFu);
}

void hostFlashResetState(void) {
	memset(&fs, 0, sizeof fs);
	fs.pollReads = 1;
	/* the memory is filled when the models reset (the bus is disarmed then); here only the state */
}

static int sectorProtected(unsigned sector) { return ((reg(R_OPTCR) >> 16) & (1u << sector)) == 0u; }

static void setFlags(uint32_t flags) {
	uint32_t sr = reg(R_SR) | flags;
	if ((flags & SR_ERRORS) != 0u && (reg(R_CR) & CR_ERRIE) != 0u) sr |= SR_OPERR; /* RM0090 3.6.6 */
	put(R_SR, sr);
}

static void begin(void) {
	put(R_SR, reg(R_SR) | SR_BSY);
	fs.pending = 1;
	fs.pollCount = 0;
}

int hostFlashComplete(void) {
	if (!fs.pending) return 0;
	fs.pending = 0;
	uint32_t cr = reg(R_CR), optcr = reg(R_OPTCR);
	if (fs.optPending) {
		fs.optPending = 0;
		const uint32_t rdp = (optcr >> 8) & 0xFFu, user = optcr & 0xE0u, bor = optcr & 0xCu, nwrp = (optcr >> 16) & 0xFFFu;
		poke(OPT_WORD0, (rdp << 8) | user | 0x10u | bor | 0x3u);
		poke(OPT_WORD1, 0xF000u | nwrp);
		put(R_OPTCR, optcr & ~OPTCR_OPTSTRT);
	}
	put(R_CR, cr & ~CR_STRT);
	put(R_SR, (reg(R_SR) & ~SR_BSY) | SR_EOP);
	return 1;
}

uint32_t hostFlashIrqPending(void) {
	const uint32_t sr = reg(R_SR), cr = reg(R_CR);
	return ((cr & CR_EOPIE) && (sr & SR_EOP)) || ((cr & CR_ERRIE) && (sr & SR_ERRORS));
}

/* ------------------------------------------------------------------ interface registers */
static int fifKnown(uintptr_t a) {
	const uint32_t off = (uint32_t)(a - FIF_BASE);
	return off == R_ACR || off == R_KEYR || off == R_OPTKEYR || off == R_SR || off == R_CR || off == R_OPTCR;
}

static int fifBeforeRead(uintptr_t a) {
	if (!fifKnown(a)) return 0;
	const uint32_t off = (uint32_t)(a - FIF_BASE);
	if (off == R_SR && fs.pending && fs.pollReads != 0u && ++fs.pollCount >= fs.pollReads) hostFlashComplete();
	if (off == R_KEYR || off == R_OPTKEYR) poke(a, 0u); /* write only */
	return 1;
}

static void eraseSectors(uint32_t cr) {
	if (cr & CR_MER) {
		for (unsigned s = 0; s < 12u; ++s) {
			if (sectorProtected(s)) {
				setFlags(SR_WRPERR);
				return;
			}
		}
		hostRegsFlashWritable(1);
		memset((void *)(uintptr_t)FLASH_MEM_BASE, 0xFF, FLASH_MEM_SIZE);
		hostRegsFlashWritable(0);
		begin();
	} else if (cr & CR_SER) {
		const unsigned s = (cr >> CR_SNB_SHIFT) & 0xFu;
		if (s > 11u) return; /* RM0090: not defined, no flag */
		if (sectorProtected(s)) {
			setFlags(SR_WRPERR);
			return;
		}
		hostRegsFlashWritable(1);
		memset((void *)(uintptr_t)sectorBase[s], 0xFF, hostFlashSectorSize(s));
		hostRegsFlashWritable(0);
		begin();
	}
}

static void fifAfterWrite(uintptr_t a, uint32_t oldValue, uint32_t written) {
	const uint32_t off = (uint32_t)(a - FIF_BASE);
	switch (off) {
	case R_ACR: poke(a, hostFlashLatencyIsStuck() ? ((written & ~7u) | (oldValue & 7u)) : written); break;
	case R_KEYR:
		poke(a, 0u);
		if (fs.keyLockout) break;
		if (fs.keyState == 0u && written == KEY1) fs.keyState = 1;
		else if (fs.keyState == 1u && written == KEY2) {
			fs.keyState = 0;
			put(R_CR, reg(R_CR) & ~CR_LOCK);
		} else {
			fs.keyState = 0;
			fs.keyLockout = 1; /* RM0090 3.6.1: a bus error and CR locked until the next reset */
			fs.busFaults++;
		}
		break;
	case R_OPTKEYR:
		poke(a, 0u);
		if (fs.optKeyLockout) break;
		if (fs.optKeyState == 0u && written == OPTKEY1) fs.optKeyState = 1;
		else if (fs.optKeyState == 1u && written == OPTKEY2) {
			fs.optKeyState = 0;
			put(R_OPTCR, reg(R_OPTCR) & ~OPTCR_OPTLOCK);
		} else {
			fs.optKeyState = 0;
			fs.optKeyLockout = 1;
			fs.busFaults++;
		}
		break;
	case R_SR: poke(a, (oldValue & ~(written & 0xF3u)) & 0x100F3u); break; /* flags rc_w1, BSY read only */
	case R_CR: {
		const uint32_t nw = written & 0x8301037Fu;
		if (fs.pending) {
			poke(a, oldValue);
			fs.stalled++;
			break;
		}
		if (oldValue & CR_LOCK) { /* locked: only a write that keeps LOCK = 1 is possible and it changes nothing */
			poke(a, oldValue);
			fs.lockedWrites++;
			break;
		}
		poke(a, nw);
		if (nw & CR_LOCK) {
			fs.keyState = 0;
			break;
		}
		if ((nw & CR_STRT) && !(oldValue & CR_STRT)) {
			const uint32_t before = reg(R_SR);
			eraseSectors(nw);
			if (!fs.pending) { /* nothing started: STRT does not stay */
				put(R_CR, reg(R_CR) & ~CR_STRT);
				(void)before;
			}
		}
		break;
	}
	case R_OPTCR: {
		if (fs.pending || (oldValue & OPTCR_OPTLOCK)) {
			if (fs.pending) fs.stalled++;
			else fs.lockedWrites++;
			poke(a, (oldValue & OPTCR_OPTLOCK) ? (oldValue | (written & OPTCR_OPTLOCK)) : oldValue);
			break;
		}
		const uint32_t nw = written & 0x0FFFFFEFu;
		poke(a, nw);
		if ((nw & OPTCR_OPTSTRT) && !(oldValue & OPTCR_OPTSTRT)) {
			fs.optPending = 1;
			begin();
		}
		break;
	}
	default: poke(a, oldValue); break;
	}
}

static void fifReset(void) {
	put(R_CR, CR_LOCK);
	put(R_OPTCR, 0x0FFFAAEDu);
	fillRaw();
}

static HostRegModel fifModel = {"FLASH interface", FIF_BASE, 0x400u, fifReset, fifBeforeRead, NULL, fifKnown, fifAfterWrite};

/* ------------------------------------------------------------------ flash memory and OTP */
static int memBeforeRead(uintptr_t a) {
	(void)a;
	return 1;
}
static int memBeforeWrite(uintptr_t wordAddr) {
	/* an access that crosses a word boundary also changes the next word: remember it, a failed store must leave it alone */
	const uintptr_t raw = hostRegsLastAccessAddr();
	fs.crossing = ((raw & 3u) + hostRegsLastAccessBytes()) > 4u;
	if (fs.crossing) {
		fs.crossAddr = wordAddr + 4u;
		fs.crossOld = hostRegsPeek(fs.crossAddr);
	}
	return 1;
}

static void memAfterWrite(uintptr_t wordAddr, uint32_t oldValue, uint32_t written) {
	const uintptr_t raw = hostRegsLastAccessAddr();
	const unsigned width = hostRegsLastAccessBytes();
	const uint32_t cr = reg(R_CR);
	const int otp = wordAddr >= OTP_BASE && wordAddr < OTP_BASE + OTP_SIZE;
	uint32_t flags = 0;
	if (fs.x64Half == 2u) { /* the first word of this double word failed: the second one is dropped without another flag */
		fs.x64Half = 0;
		poke(wordAddr, oldValue);
		if (fs.crossing) poke(fs.crossAddr, fs.crossOld);
		if ((raw & 7u) == 4u) return;
	}
	if (fs.pending) { /* the bus would stall until the operation ends: nothing happens */
		poke(wordAddr, oldValue);
		if (fs.crossing) poke(fs.crossAddr, fs.crossOld);
		fs.stalled++;
		return;
	}
	if (cr & CR_LOCK) { /* programming needs PG, which cannot be set while locked: sequence error */
		flags = SR_PGSERR;
	} else if (!(cr & CR_PG)) {
		flags = SR_PGSERR;
	} else {
		const unsigned psize = (cr >> CR_PSIZE_SHIFT) & 3u;
		const unsigned want = psize == 3u ? 4u : 1u << psize;
		if (width != want) flags = SR_PGPERR;
		else if ((raw & (width - 1u)) != 0u || ((raw & 15u) + width) > 16u) flags = SR_PGAERR;
		else if (psize == 3u && ((fs.x64Half == 0u && (raw & 7u) != 0u) || (fs.x64Half == 1u && (raw & 7u) != 4u))) flags = SR_PGAERR;
		else if (!otp) {
			const int sec = hostFlashSectorOf((uint32_t)raw);
			if (sec >= 0 && sectorProtected((unsigned)sec)) flags = SR_WRPERR;
		} else {
			const unsigned block = (unsigned)((raw - OTP_BASE) / 32u);
			if (block < 16u && *(volatile uint8_t *)(uintptr_t)(OTP_LOCK_BASE + block) == 0u) flags = SR_WRPERR;
		}
	}
	if (flags != 0u) {
		poke(wordAddr, oldValue);
		if (fs.crossing) poke(fs.crossAddr, fs.crossOld);
		fs.x64Half = (((cr >> CR_PSIZE_SHIFT) & 3u) == 3u && (fs.x64Half == 0u) && (raw & 7u) == 0u) ? 2u : 0u;
		setFlags(flags);
		return;
	}
	poke(wordAddr, oldValue & written); /* NOR: bits go 1 -> 0 only (RM0090 3.6.4) */
	if (((cr >> CR_PSIZE_SHIFT) & 3u) == 3u && fs.x64Half == 0u) {
		fs.x64Half = 1;
		return;
	}
	fs.x64Half = 0;
	begin();
}

static HostRegModel memModel = {"flash memory", FLASH_MEM_BASE, FLASH_MEM_SIZE, NULL, memBeforeRead, NULL, memBeforeWrite, memAfterWrite};
static HostRegModel otpModel = {"OTP", OTP_BASE, OTP_SIZE, NULL, memBeforeRead, NULL, memBeforeWrite, memAfterWrite};

void hostFlashRegister(void) {
	hostRegsAddModel(&fifModel);
	hostRegsAddModel(&memModel);
	hostRegsAddModel(&otpModel);
}
