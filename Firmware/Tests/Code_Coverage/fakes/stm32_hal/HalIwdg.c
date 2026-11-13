/*
 * HalIwdg.c: independent watchdog model (P12 T7). See HalIwdg.h for what is modelled and the RM0090 sources.
 */
#include "HalIwdg.h"

#include <stddef.h>
#include <string.h>

#define IWDG_BASE_ADDR 0x40003000u
#define R_KR 0x00u
#define R_PR 0x04u
#define R_RLR 0x08u
#define R_SR 0x0Cu

static struct {
	uint8_t writeEnabled, running, resetRequested;
	uint32_t counter, ignored, reloads;
	unsigned pollReads, pollCount;
} is = {0, 0, 0, 0, 0, 0, 1, 0};

static uint32_t reg(uint32_t r) { return hostRegsPeek(IWDG_BASE_ADDR + r); }
static void put(uint32_t r, uint32_t v) { hostRegsPoke(IWDG_BASE_ADDR + r, v); }

void hostIwdgResetState(void) {
	memset(&is, 0, sizeof is);
	is.pollReads = 1;
}
void hostIwdgSetPollReads(unsigned n) {
	is.pollReads = n;
	is.pollCount = 0;
}
void hostIwdgComplete(void) { put(R_SR, 0u); }
int hostIwdgRunning(void) { return is.running; }
uint32_t hostIwdgCounter(void) { return is.counter; }
int hostIwdgResetRequested(void) { return is.resetRequested; }
uint32_t hostIwdgIgnoredWrites(void) { return is.ignored; }
uint32_t hostIwdgReloads(void) { return is.reloads; }

uint32_t hostIwdgAdvance(uint32_t n) {
	if (!is.running) return is.counter;
	if (n >= is.counter) {
		is.counter = 0;
		is.resetRequested = 1;
	} else {
		is.counter -= n;
	}
	return is.counter;
}

static int known(uintptr_t a) {
	const uint32_t off = (uint32_t)(a - IWDG_BASE_ADDR);
	return off == R_KR || off == R_PR || off == R_RLR || off == R_SR;
}

static int beforeRead(uintptr_t a) {
	if (!known(a)) return 0;
	const uint32_t off = (uint32_t)(a - IWDG_BASE_ADDR);
	if (off == R_SR && (reg(R_SR) & 3u) != 0u && is.pollReads != 0u && ++is.pollCount >= is.pollReads) {
		is.pollCount = 0;
		hostIwdgComplete();
	}
	if (off == R_KR) hostRegsPoke(a, 0u); /* write only */
	return 1;
}

static void afterWrite(uintptr_t a, uint32_t oldValue, uint32_t written) {
	const uint32_t off = (uint32_t)(a - IWDG_BASE_ADDR);
	const uint32_t sr = reg(R_SR);
	switch (off) {
	case R_KR:
		hostRegsPoke(a, 0u);
		written &= 0xFFFFu;
		if (written == 0x5555u) {
			is.writeEnabled = 1;
		} else {
			is.writeEnabled = 0;
			if (written == 0xAAAAu) {
				is.counter = reg(R_RLR) & 0xFFFu;
				is.reloads++;
			} else if (written == 0xCCCCu) {
				is.running = 1;
				is.counter = reg(R_RLR) & 0xFFFu;
			}
		}
		break;
	case R_PR:
		if (!is.writeEnabled || (sr & 1u)) {
			hostRegsPoke(a, oldValue);
			is.ignored++;
		} else {
			hostRegsPoke(a, written & 7u);
			put(R_SR, sr | 1u);
		}
		break;
	case R_RLR:
		if (!is.writeEnabled || (sr & 2u)) {
			hostRegsPoke(a, oldValue);
			is.ignored++;
		} else {
			hostRegsPoke(a, written & 0xFFFu);
			put(R_SR, sr | 2u);
		}
		break;
	default:
		hostRegsPoke(a, oldValue); /* SR is read only */
		break;
	}
}

static void reset(void) { put(R_RLR, 0xFFFu); }

static HostRegModel iwdgModel = {"IWDG", IWDG_BASE_ADDR, 0x400u, reset, beforeRead, NULL, known, afterWrite};

void hostIwdgRegister(void) { hostRegsAddModel(&iwdgModel); }
