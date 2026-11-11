/*
 * HalClockPower.h (P12 T2, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Behavioural models of the RCC (0x40023800) and the PWR (0x40007000) of the STM32F405/407, on the trapping bus of
 * HalRegisterSpace.h. Sources: RM0090 Rev 22, 7.3.1-7.3.23 (RCC), 5.4.1-5.4.2 (PWR). Replaces the storage-only RCC
 * model of T1. Confidence: High for register bits, reset values, write-protection rules and the SWS / ready flag
 * coupling; Medium for the timing (RM0090 gives no start-up times: the ready flag rises after a configurable number
 * of HAL ticks, default 1; the real time depends on the oscillator and the supply).
 *
 * Time base: the HAL tick (uwTick), which every HAL polling loop advances through HAL_GetTick. Ready flags are
 * evaluated lazily on every read of CR, CFGR, CIR, BDCR, CSR (RCC) and PWR_CSR, so a loop that polls a flag sees it rise
 * after the start-up time without a thread or an interrupt.
 *
 * RCC_CR (7.3.1): HSION/HSIRDY, HSEON/HSERDY, HSEBYP (writable only while HSEON was 0), CSSON, PLLON/PLLRDY,
 * PLLI2SON/PLLI2SRDY, HSITRIM[7:3]; HSICAL[15:8] and the ready flags are read-only (HSICAL reads 0x00, the undefined
 * calibration of the reset value is modelled as 0); reserved bits read 0 and ignore writes. HSION cannot be cleared
 * while the HSI is the system clock directly or through the PLL, HSEON cannot be cleared while the HSE is, PLLON not
 * while the PLL is (SWS). Clearing an ON bit clears its ready flag at once (RM0090: after 6 cycles, not modelled). A PLL
 * is ready only while its input oscillator (PLLSRC) is ready.
 * RCC_PLLCFGR (7.3.2) and RCC_PLLI2SCFGR (7.3.23): writes are ignored while the PLL (resp. PLL and PLLI2S for PLLCFGR) is
 * on. Reserved bits keep their reset value.
 * RCC_CFGR (7.3.3): SW is written; SWS follows SW once the selected source is ready, until then SWS keeps its value
 * (UM1725 ClockConfig note). SW = 3 is "not allowed" and ignored. Reserved bits 9:8 read 0.
 * RCC_CIR (7.3.4): xxRDYF set by hardware when a ready flag rises and xxRDYIE is set, cleared by writing xxRDYC; CSSF by CSSC; the
 * interrupt enable bits are stored. The NVIC side is not modelled.
 * RCC_BDCR (7.3.20): writes are ignored while PWR_CR.DBP is 0. LSEON/LSERDY, LSEBYP (writable only while LSEON is 0),
 * RTCSEL (write once: ignored while RTCSEL is not 0, until a backup domain reset), RTCEN, BDRST (setting it resets
 * LSEON, LSEBYP, RTCSEL, RTCEN; BDRST stays as written).
 * RCC_CSR (7.3.21): LSION/LSIRDY, reset flags 31:25 read-only, RMVF clears them (reads 0).
 * All other RCC registers (AHBxRSTR, APBxRSTR, AHBxENR, APBxENR, xxLPENR, SSCGR) are stored as written, as in T1.
 * PWR_CR (5.4.1): reset 0x00004000; VOS, FPDS, DBP, PLS[2:0], PVDE, PDDS, LPDS stored; CSBF and CWUF write-only (read
 * 0) and clear SBF / WUF. PWR_CSR (5.4.2): BRE (writable only while DBP is 1), EWUP stored; VOSRDY follows VOS and BRR
 * follows BRE after the start-up time on the rising edge (VOSRDY reads 1 after reset although RM0090 lists PWR_CSR as 0: VOS
 * is 1 then; BRR drops at once when BRE is cleared, RM0090 gives no fall time); PVDO is 1 while PVDE is 1 and VDD (hostPwrSetVddMv, default 3300) is below the PLS
 * threshold (2000 2100 2300 2500 2600 2700 2800 2900 mV), else 0; SBF and WUF are set by hostPwrSetFlags.
 *
 * PWR reset: writing APB1RSTR.PWRRST (bit 28) resets PWR_CR to 0x00004000 and EWUP; BRE is kept (RM0090 5.4.2: only a backup
 * domain reset clears it).
 *
 * Not modelled: clock gating of the peripherals, the real frequency of any clock, glitches and jitter, interrupts of
 * the RCC / PWR, wake-up from Stop / Standby, the effect of VOS on the maximum frequency, flash latency checks.
 */
#ifndef FFB_HAL_CLOCK_POWER_H
#define FFB_HAL_CLOCK_POWER_H

#include <stdint.h>

#include "HalRegisterSpace.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { HOST_OSC_HSI = 0, HOST_OSC_HSE, HOST_OSC_PLL, HOST_OSC_PLLI2S, HOST_OSC_LSE, HOST_OSC_LSI, HOST_OSC_COUNT } HostOsc;

/* Models (registered by hostPeriphRegisterAll). */
void hostClockPowerRegister(void);
/* State that is not stored in registers: start-up times (1 tick), failure flags, VDD, power flags. */
void hostClockPowerResetState(void);

/* RCC callbacks used by the RCC model of HalPeripherals.c. */
void hostRccReset(void);
int hostRccKnown(uintptr_t addr);
int hostRccBeforeRead(uintptr_t addr);
void hostRccAfterWrite(uintptr_t addr, uint32_t oldValue, uint32_t written);

/* Ticks between the ON bit and the ready flag (0: the flag is set at once). */
void hostRccSetStartupTicks(HostOsc osc, uint32_t ticks);
/* 1: the oscillator or PLL never gets ready (a polling loop times out). */
void hostRccFail(HostOsc osc, int fail);
/* 1: the ready flag stays set also after the ON bit is cleared (a polling loop for "flag cleared" times out). */
void hostRccStickyReady(HostOsc osc, int sticky);
/* The HSE fails. With CSSON set (CSS event): HSEON and HSERDY clear, CSSF is set in RCC_CIR; if the HSE was the system
 * clock directly or through the PLL, HSION is set and SW is forced to HSI (RM0090 7.3.1, 7.3.3). With CSSON clear the
 * oscillator just stops (HSERDY drops, HSEON stays) and nothing else reacts. */
void hostRccHseFailure(void);
/* Reset flags of RCC_CSR (bits 31:25), as set by a reset. */
void hostRccSetResetFlags(uint32_t mask);

void hostPwrSetStartupTicks(uint32_t vosTicks, uint32_t bkpTicks);
void hostPwrFailVos(int fail);
void hostPwrFailBackupRegulator(int fail);
void hostPwrStickyBackupReady(int sticky); /* BRR stays set also after BRE is cleared */
void hostPwrSetVddMv(uint32_t mv);
void hostPwrSetFlags(int standbyFlag, int wakeupFlag);

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_CLOCK_POWER_H */
