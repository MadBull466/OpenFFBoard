/*
 * HalIwdg.h (P12 T7, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Model of the independent watchdog IWDG at 0x40003000 (RM0090 Rev 22 chapter 21, registers 21.4: KR 0x00, PR 0x04, RLR 0x08, SR 0x0C).
 * Confidence: High for the registers and the key protocol; Medium for the update flag timing and the counter (events, not LSI time).
 * KR (write only, reads 0): 0x5555 enables write access to PR and RLR (any other value disables it again), 0xAAAA reloads the counter from RLR (and disables write access),
 * 0xCCCC starts the watchdog (and loads the counter from RLR). PR (0x04, 3 bits, reset 0) and RLR (0x08, 12 bits, reset 0xFFF) accept writes only while write access is enabled
 * and while the update flag is clear (PVU / RVU, RM0090 21.4.3: "can be updated only when PVU = 0"). SR (read only): PVU bit 0, RVU bit 1, set by a write to PR / RLR and cleared when the
 * update completes: hostIwdgComplete(), or the n-th read of SR while a flag is set (hostIwdgSetPollReads, default 1; 0 = never, to test the timeout of HAL_IWDG_Init). A write
 * to a protected register or while its update flag is set is ignored and counted (hostIwdgIgnoredWrites). Once started the counter counts down in hostIwdgAdvance(n) events
 * (watchdog clock ticks: the prescaler value is stored, it does not divide) and hostIwdgResetRequested() reports a reset request when it reaches 0.
 * Not modelled: the LSI clock, the hardware watchdog option (WDG_SW = 0 starts it at reset), freezing in debug, the window (the F407 IWDG has no window register).
 */
#ifndef FFB_HAL_IWDG_H
#define FFB_HAL_IWDG_H

#include <stdint.h>

#include "HalRegisterSpace.h"

#ifdef __cplusplus
extern "C" {
#endif

void hostIwdgRegister(void);
void hostIwdgResetState(void);

void hostIwdgComplete(void);
void hostIwdgSetPollReads(unsigned n);
/* Counts n watchdog clock events while started; sets the reset request at 0. Returns the counter. */
uint32_t hostIwdgAdvance(uint32_t n);
int hostIwdgRunning(void);
uint32_t hostIwdgCounter(void);
int hostIwdgResetRequested(void);
uint32_t hostIwdgIgnoredWrites(void);
uint32_t hostIwdgReloads(void); /* number of 0xAAAA keys seen (including the one of the start) */

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_IWDG_H */
