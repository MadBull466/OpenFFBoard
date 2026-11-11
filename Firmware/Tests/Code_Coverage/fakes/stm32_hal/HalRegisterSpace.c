/*
 * HalRegisterSpace.c: see HalRegisterSpace.h for what is modelled. Linux only (built only with FFB_TESTS_REAL_HAL,
 * which requires FFB_TESTS_32BIT on Linux).
 */
#define _GNU_SOURCE
#include "HalRegisterSpace.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

#ifndef MAP_FIXED_NOREPLACE
#define MAP_FIXED_NOREPLACE 0x100000 /* Linux >= 4.17 */
#endif

typedef struct {
	uintptr_t base;
	uint32_t size;
} RegRange;

/* RM0090 Rev 22 Table 1 and core_cm4.h; see the header. Sizes are whole 4 KiB pages. */
static const RegRange ranges[] = {
	{0x40000000u, 0x00080000u}, /* APB1, APB2, AHB1 */
	{0x50000000u, 0x00061000u}, /* AHB2: USB OTG FS ... RNG */
	{0xE000E000u, 0x00001000u}, /* system control space */
	{0xE0042000u, 0x00001000u}, /* DBGMCU */
	{0x1FFF7000u, 0x00001000u}, /* unique id, flash size */
	{0x1FFFC000u, 0x00001000u}, /* option bytes */
};
enum { RANGE_COUNT = sizeof(ranges) / sizeof(ranges[0]) };

static int mapState; /* 0 = not tried, 1 = mapped, -1 = failed */
static char mapError[160];

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
	return 1;
}

int hostRegsMapped(void) { return mapState == 1; }

void hostRegsReset(void) {
	if (mapState != 1) return;
	for (int i = 0; i < RANGE_COUNT; ++i) memset((void *)ranges[i].base, 0, ranges[i].size);
}

const char *hostRegsError(void) { return mapError; }

int hostRegsContains(uintptr_t addr, uint32_t size) {
	for (int i = 0; i < RANGE_COUNT; ++i) {
		if (addr >= ranges[i].base && size <= ranges[i].size && addr - ranges[i].base <= ranges[i].size - size) return 1;
	}
	return 0;
}
