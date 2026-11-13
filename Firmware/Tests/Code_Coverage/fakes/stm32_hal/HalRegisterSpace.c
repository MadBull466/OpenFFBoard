/*
 * HalRegisterSpace.c: see HalRegisterSpace.h. Linux i386 only (built only with FFB_TESTS_REAL_HAL, which requires
 * FFB_TESTS_32BIT on Linux): it uses MAP_FIXED_NOREPLACE, the x86 trap flag and the page-fault error code of the
 * i386 signal context.
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "HalRegisterSpace.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <ucontext.h>

#ifndef MAP_FIXED_NOREPLACE
#define MAP_FIXED_NOREPLACE 0x100000 /* Linux >= 4.17 */
#endif
#if !defined(__i386__)
#error "HalRegisterSpace.c needs a 32-bit x86 build (FFB_TESTS_32BIT)"
#endif

#define X86_TRAP_FLAG 0x100u
#define PF_WRITE 0x2u /* page-fault error code bit 1: the access was a write */

typedef struct {
	uintptr_t base;
	uint32_t size;
} RegRange;

/* RM0090 Rev 22 Table 1 and core_cm4.h; see the header. Sizes are whole 4 KiB pages. */
static const RegRange ranges[] = {
	{0x40000000u, 0x00080000u}, /* APB1, APB2, AHB1 */
	{0x50000000u, 0x00061000u}, /* AHB2: USB OTG FS ... RNG */
	{0x08000000u, 0x00100000u}, /* main flash memory 1 MB (P12 T7, HalFlash.c) */
	{0xE000E000u, 0x00001000u}, /* system control space */
	{0xE0042000u, 0x00001000u}, /* DBGMCU */
	{0x1FFF7000u, 0x00001000u}, /* unique id, flash size */
	{0x1FFFC000u, 0x00001000u}, /* option bytes */
	{0x42000000u, 0x01000000u}, /* bit-band alias of 0x40000000-0x4007FFFF (RM0090 2.3.3) */
};
enum { RANGE_COUNT = sizeof(ranges) / sizeof(ranges[0]) };

static int mapState; /* 0 = not tried, 1 = mapped, -1 = failed */
static char mapError[160];

enum { MAX_MODELS = 64, MAX_FAULTS = 64, MAX_LOG = 4096 };
static const HostRegModel *models[MAX_MODELS];
static int modelCount;

static char faultText[MAX_FAULTS][120];
static size_t faultCount;

static HostRegAccess logBuf[MAX_LOG];
static size_t logCount;
static size_t logDropped;

static volatile int armed;
static volatile int inStep;   /* between SIGSEGV and SIGTRAP */
static volatile int inHandler;
static struct {
	uintptr_t addr;
	uintptr_t rawAddr; /* exact (unaligned) address of the access */
	int flash;         /* the access is a store into main flash: its page(s) are unprotected for the step */
	uintptr_t flashPage;
	unsigned bytes;    /* access width decoded from the faulting x86 instruction (1, 2 or 4; 4 when unknown) */
	uint32_t oldValue;
	int write;
	int reject;
	const HostRegModel *model;
} step;

uintptr_t hostRegsLastAccessAddr(void) { return step.rawAddr; }
unsigned hostRegsLastAccessBytes(void) { return step.bytes; }

/* Width of the store at EIP: operand size prefix 0x66 = 16 bit; MOV / ALU / group 1 byte forms = 8 bit; else 32 bit. */
static unsigned decodeStoreWidth(const unsigned char *p) {
	unsigned w = 4;
	for (int n = 0; n < 4; ++n, ++p) {
		if (*p == 0x66) w = 2;
		else if (*p == 0xF0 || *p == 0x2E || *p == 0x36 || *p == 0x3E || *p == 0x26 || *p == 0x64 || *p == 0x65 || *p == 0xF2 || *p == 0xF3) continue;
		else break;
	}
	const unsigned op = *p;
	if (op == 0x88 || op == 0xC6 || op == 0x80 || op == 0xA2 || op == 0xFE || op == 0xF6 || (op < 0x40 && (op & 7u) == 0u)) return 1;
	if (op == 0x0F && (p[1] >= 0x90 && p[1] <= 0x9F)) return 1;
	return w;
}

static struct sigaction prevSegv, prevTrap;

/* ---------------------------------------------------------------- bit-band alias (RM0090 2.3.3)
 * bit_word_addr = 0x42000000 + byte_offset * 32 + bit_number * 4 for the peripheral region 0x40000000. A write to
 * an alias word "has the same effect as a read-modify-write operation on the targeted bit": the target word is
 * taken without read side effects, the bit is set from bit 0 of the written value, and the new word goes through
 * the target register's model as a write (beforeWrite / afterWrite, logged at the target address). A read returns
 * the target bit (0x01 / 0x00) after the target model's beforeRead / afterRead. */
#define BITBAND_ALIAS_BASE 0x42000000u
#define BITBAND_TARGET_BASE 0x40000000u
static const HostRegModel *findModel(uintptr_t addr);
static void logAccess(uintptr_t addr, uint32_t value, uint32_t written, int write);

static void bitbandTarget(uintptr_t alias, uintptr_t *word, unsigned *bit) {
	const uint32_t off = (uint32_t)(alias - BITBAND_ALIAS_BASE);
	const uintptr_t byteAddr = BITBAND_TARGET_BASE + off / 32u;
	*word = byteAddr & ~(uintptr_t)3u;
	*bit = (unsigned)((byteAddr & 3u) * 8u + (off % 32u) / 4u);
}

static int bitbandBeforeRead(uintptr_t alias) {
	uintptr_t target;
	unsigned bit;
	bitbandTarget(alias, &target, &bit);
	const HostRegModel *m = findModel(target);
	if (m == NULL || (m->beforeRead != NULL && !m->beforeRead(target))) {
		hostRegsAddFault("bit-band read of an unmodelled or rejecting register", target, 0);
		return 0;
	}
	const uint32_t v = *(volatile uint32_t *)target;
	if (m->afterRead != NULL) m->afterRead(target, v);
	logAccess(target, v, 0, 0);
	*(volatile uint32_t *)alias = (v >> bit) & 1u;
	return 1;
}

static void bitbandAfterWrite(uintptr_t alias, uint32_t oldValue, uint32_t written) {
	uintptr_t target;
	unsigned bit;
	(void)oldValue;
	bitbandTarget(alias, &target, &bit);
	*(volatile uint32_t *)alias = 0;
	const HostRegModel *m = findModel(target);
	if (m == NULL || (m->beforeWrite != NULL && !m->beforeWrite(target))) {
		hostRegsAddFault("bit-band write to an unmodelled or rejecting register", target, written);
		return;
	}
	const uint32_t old = *(volatile uint32_t *)target;
	const uint32_t nw = (written & 1u) ? (old | (1u << bit)) : (old & ~(1u << bit));
	*(volatile uint32_t *)target = nw;
	if (m->afterWrite != NULL) m->afterWrite(target, old, nw);
	logAccess(target, *(volatile uint32_t *)target, nw, 1);
}

static const HostRegModel bitbandModel = {"bit-band alias (RM0090 2.3.3)", BITBAND_ALIAS_BASE, 0x01000000u, NULL,
										  bitbandBeforeRead, NULL, NULL, bitbandAfterWrite};

#define FLASH_RANGE_BASE 0x08000000u
#define FLASH_RANGE_SIZE 0x00100000u
static int inFlashRange(uintptr_t a) { return a >= FLASH_RANGE_BASE && a < FLASH_RANGE_BASE + FLASH_RANGE_SIZE; }

/* The main flash range is not toggled with the register ranges (that was 256 pages per trap): while armed it stays PROT_READ, so reads of flash
 * memory by a test or the HAL are plain host reads (no model, no log) and only stores trap. A store fault unprotects the touched page(s) for the
 * single step and the trap handler protects them again. */
static void flashProtect(int prot) { mprotect((void *)FLASH_RANGE_BASE, FLASH_RANGE_SIZE, prot); }
/* For models that change flash memory in bulk (erase): writable while on, read only again when off (a no-op while disarmed). */
void hostRegsFlashWritable(int on) {
	if (!armed) return;
	flashProtect(on ? (PROT_READ | PROT_WRITE) : PROT_READ);
}
static void protectAll(int prot) {
	for (int i = 0; i < RANGE_COUNT; ++i) {
		if (ranges[i].base == FLASH_RANGE_BASE) continue;
		mprotect((void *)ranges[i].base, ranges[i].size, prot);
	}
}

int hostRegsMap(void) {
	if (mapState != 0) return mapState == 1;
	for (int i = 0; i < RANGE_COUNT; ++i) {
		void *want = (void *)ranges[i].base;
		void *got = mmap(want, ranges[i].size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
		if (got != want) {
			snprintf(mapError, sizeof mapError, "mmap of 0x%08lx (+0x%lx) failed: %s", (unsigned long)ranges[i].base,
					 (unsigned long)ranges[i].size, got == MAP_FAILED ? strerror(errno) : "mapped at another address");
			if (got != MAP_FAILED) munmap(got, ranges[i].size);
			mapState = -1;
			return 0;
		}
	}
	mapState = 1;
	hostRegsAddModel(&bitbandModel);
	return 1;
}

int hostRegsMapped(void) { return mapState == 1; }

const char *hostRegsError(void) { return mapError; }

int hostRegsContains(uintptr_t addr, uint32_t size) {
	for (int i = 0; i < RANGE_COUNT; ++i) {
		if (addr >= ranges[i].base && size <= ranges[i].size && addr - ranges[i].base <= ranges[i].size - size) return 1;
	}
	return 0;
}

void hostRegsAddModel(const HostRegModel *model) {
	for (int i = 0; i < modelCount; ++i)
		if (models[i] == model) return;
	if (modelCount >= MAX_MODELS) {
		fprintf(stderr, "HalRegisterSpace: model table full (MAX_MODELS)\n");
		abort();
	}
	models[modelCount++] = model;
}

static const HostRegModel *findModel(uintptr_t addr) {
	for (int i = 0; i < modelCount; ++i) {
		if (addr >= models[i]->base && addr - models[i]->base < models[i]->size) return models[i];
	}
	return NULL;
}

void hostRegsAddFault(const char *what, uintptr_t addr, uint32_t value) {
	if (faultCount < MAX_FAULTS) {
		snprintf(faultText[faultCount], sizeof faultText[0], "%s at 0x%08lx (value 0x%08lx)", what, (unsigned long)addr,
				 (unsigned long)value);
	}
	faultCount++;
}

size_t hostRegsFaultCount(void) { return faultCount; }

const char *hostRegsFaultText(size_t index) {
	return index < faultCount && index < MAX_FAULTS ? faultText[index] : "";
}

void hostRegsClearFaults(void) { faultCount = 0; }

static void logAccess(uintptr_t addr, uint32_t value, uint32_t written, int write) {
	if (logCount < MAX_LOG) {
		logBuf[logCount].addr = addr;
		logBuf[logCount].value = value;
		logBuf[logCount].written = written;
		logBuf[logCount].write = (uint8_t)(write != 0);
		logCount++;
	} else {
		logDropped++;
	}
}

size_t hostRegsLogCount(void) { return logCount; }
const HostRegAccess *hostRegsLogAt(size_t index) { return index < logCount ? &logBuf[index] : NULL; }
size_t hostRegsLogDropped(void) { return logDropped; }
void hostRegsLogClear(void) {
	logCount = 0;
	logDropped = 0;
}

size_t hostRegsWritesTo(uintptr_t addr) {
	size_t n = 0;
	for (size_t i = 0; i < logCount; ++i) n += (logBuf[i].write && logBuf[i].addr == (addr & ~(uintptr_t)3u));
	return n;
}

size_t hostRegsReadsFrom(uintptr_t addr) {
	size_t n = 0;
	for (size_t i = 0; i < logCount; ++i) n += (!logBuf[i].write && logBuf[i].addr == (addr & ~(uintptr_t)3u));
	return n;
}

uint32_t hostRegsPeek(uintptr_t addr) {
	if (inFlashRange(addr)) return *(volatile uint32_t *)(addr & ~(uintptr_t)3u); /* readable armed or not */
	const int reprotect = armed && !inHandler;
	if (reprotect) protectAll(PROT_READ | PROT_WRITE);
	const uint32_t v = *(volatile uint32_t *)(addr & ~(uintptr_t)3u);
	if (reprotect) protectAll(PROT_NONE);
	return v;
}

void hostRegsPoke(uintptr_t addr, uint32_t value) {
	if (inFlashRange(addr) && armed && !inHandler) { /* one page writable for the store */
		void *page = (void *)(addr & ~(uintptr_t)0xFFFu);
		mprotect(page, 0x1000, PROT_READ | PROT_WRITE);
		*(volatile uint32_t *)(addr & ~(uintptr_t)3u) = value;
		mprotect(page, 0x1000, PROT_READ);
		return;
	}
	const int reprotect = armed && !inHandler;
	if (reprotect) protectAll(PROT_READ | PROT_WRITE);
	*(volatile uint32_t *)(addr & ~(uintptr_t)3u) = value;
	if (reprotect) protectAll(PROT_NONE);
}

static void onSegv(int sig, siginfo_t *si, void *ucv) {
	ucontext_t *uc = (ucontext_t *)ucv;
	const uintptr_t a = (uintptr_t)si->si_addr;
	if (!armed || inStep || !hostRegsContains(a, 1)) {
		/* Not ours: give the fault to the previous handler (doctest's crash report or the default action). */
		sigaction(SIGSEGV, &prevSegv, NULL);
		(void)sig;
		return;
	}
	inHandler = 1;
	protectAll(PROT_READ | PROT_WRITE);
	step.flash = inFlashRange(a);
	if (step.flash) {
		step.flashPage = a & ~(uintptr_t)0xFFFu;
		mprotect((void *)step.flashPage, 0x2000, PROT_READ | PROT_WRITE); /* the store may cross into the next page */
	}
	step.addr = a & ~(uintptr_t)3u;
	step.rawAddr = a;
	step.bytes = decodeStoreWidth((const unsigned char *)uc->uc_mcontext.gregs[REG_EIP]);
	step.write = (uc->uc_mcontext.gregs[REG_ERR] & PF_WRITE) != 0;
	step.model = findModel(step.addr);
	step.reject = 0;
	if (step.model == NULL) {
		hostRegsAddFault(step.write ? "write to an unmodelled register" : "read of an unmodelled register", step.addr,
						 *(volatile uint32_t *)step.addr);
		step.reject = step.write;
	} else if (step.write) {
		if (step.model->beforeWrite != NULL && !step.model->beforeWrite(step.addr)) step.reject = 1;
	} else {
		if (step.model->beforeRead != NULL && !step.model->beforeRead(step.addr)) step.reject = 1;
	}
	step.oldValue = *(volatile uint32_t *)step.addr;
	inStep = 1;
	uc->uc_mcontext.gregs[REG_EFL] |= X86_TRAP_FLAG;
	inHandler = 0;
}

static void onTrap(int sig, siginfo_t *si, void *ucv) {
	ucontext_t *uc = (ucontext_t *)ucv;
	if (!inStep) {
		/* Not our single step: previous handler. */
		if (prevTrap.sa_flags & SA_SIGINFO) {
			if (prevTrap.sa_sigaction != NULL) prevTrap.sa_sigaction(sig, si, ucv);
		} else if (prevTrap.sa_handler != SIG_DFL && prevTrap.sa_handler != SIG_IGN) {
			prevTrap.sa_handler(sig);
		}
		return;
	}
	inHandler = 1;
	uc->uc_mcontext.gregs[REG_EFL] &= ~X86_TRAP_FLAG;
	volatile uint32_t *word = (volatile uint32_t *)step.addr;
	if (step.write) {
		const uint32_t written = *word;
		if (step.reject) {
			*word = step.oldValue;
			if (step.model != NULL) hostRegsAddFault("write rejected by the register model", step.addr, written);
		} else if (step.model->afterWrite != NULL) {
			step.model->afterWrite(step.addr, step.oldValue, written);
		}
		logAccess(step.addr, *word, written, 1);
	} else {
		const uint32_t value = *word;
		if (step.reject && step.model != NULL) hostRegsAddFault("read rejected by the register model", step.addr, value);
		if (!step.reject && step.model != NULL && step.model->afterRead != NULL) step.model->afterRead(step.addr, value);
		logAccess(step.addr, value, 0, 0);
	}
	inStep = 0;
	protectAll(PROT_NONE);
	if (step.flash) {
		step.flash = 0;
		mprotect((void *)step.flashPage, 0x2000, PROT_READ);
	}
	inHandler = 0;
}

void hostRegsArm(void) {
	if (mapState != 1) return;
	struct sigaction sa;
	memset(&sa, 0, sizeof sa);
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = SA_SIGINFO | SA_NODEFER;
	struct sigaction cur;
	/* Keep the previous handlers only when ours is not installed already (else a non-register fault would loop). */
	sigaction(SIGSEGV, NULL, &cur);
	if (!((cur.sa_flags & SA_SIGINFO) && cur.sa_sigaction == onSegv)) prevSegv = cur;
	sigaction(SIGTRAP, NULL, &cur);
	if (!((cur.sa_flags & SA_SIGINFO) && cur.sa_sigaction == onTrap)) prevTrap = cur;
	sa.sa_sigaction = onSegv;
	sigaction(SIGSEGV, &sa, NULL);
	sa.sa_sigaction = onTrap;
	sigaction(SIGTRAP, &sa, NULL);
	inStep = 0;
	armed = 1;
	protectAll(PROT_NONE);
	flashProtect(PROT_READ);
}

void hostRegsDisarm(void) {
	if (mapState != 1) return;
	if (armed) {
		protectAll(PROT_READ | PROT_WRITE);
		flashProtect(PROT_READ | PROT_WRITE);
	}
	armed = 0;
}

int hostRegsArmed(void) { return armed; }

void hostRegsReset(void) {
	if (mapState != 1) return;
	hostRegsDisarm();
	for (int i = 0; i < RANGE_COUNT; ++i) memset((void *)ranges[i].base, 0, ranges[i].size);
	for (int i = 0; i < modelCount; ++i)
		if (models[i]->reset != NULL) models[i]->reset();
	faultCount = 0;
	hostRegsLogClear();
}
