/*
 * host_cmsis.c: counters, hooks and the exclusive monitor of host_cmsis_gcc.h (see the header for the fidelity notes).
 */
#include "host_cmsis_gcc.h"

#include <stddef.h>

volatile uint32_t hostCmsisBarrierCount;
volatile uint32_t hostCmsisNopCount;
volatile uint32_t hostCmsisWfiCount;
volatile uint32_t hostCmsisWfeCount;
volatile uint32_t hostCmsisSevCount;
volatile uint32_t hostCmsisFailNextStrex;
volatile uint32_t hostCmsisStrexFailures;
void (*volatile hostCmsisWaitHook)(void);

static volatile void *monitorAddress; /* NULL = monitor closed */

void hostCmsisReset(void) {
	hostCmsisBarrierCount = 0U;
	hostCmsisNopCount = 0U;
	hostCmsisWfiCount = 0U;
	hostCmsisWfeCount = 0U;
	hostCmsisSevCount = 0U;
	hostCmsisFailNextStrex = 0U;
	hostCmsisStrexFailures = 0U;
	hostCmsisWaitHook = NULL;
	monitorAddress = NULL;
}

void hostCmsisWaited(void) {
	void (*hook)(void) = hostCmsisWaitHook;
	if (hook != NULL) hook();
}

void hostCmsisMonitorOpen(volatile void *addr) { monitorAddress = addr; }

uint32_t hostCmsisMonitorClaim(volatile void *addr) {
	const int open = (monitorAddress != NULL) && (monitorAddress == addr);
	monitorAddress = NULL; /* every store-exclusive closes the local monitor */
	if (!open || hostCmsisFailNextStrex != 0U) {
		if (open) hostCmsisFailNextStrex--;
		hostCmsisStrexFailures++;
		return 1U;
	}
	return 0U;
}
