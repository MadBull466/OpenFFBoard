/*
 * HalTim.h (P12 T6a, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Register model of the 14 timers of the STM32F405/407 on the trapping bus of HalRegisterSpace.h: TIM1 and TIM8 (advanced control), TIM2 and TIM5 (32-bit
 * general purpose), TIM3 and TIM4 (16-bit general purpose), TIM6 and TIM7 (basic), TIM9 and TIM12 (two channels), TIM10, TIM11, TIM13 and TIM14 (one channel).
 * Source: RM0090 Rev 22 chapters 17 (TIM1 / TIM8, registers 17.4), 18 (TIM2 to TIM5, 18.4), 19 (TIM9 to TIM14, 19.4 and 19.5) and 20 (TIM6 / TIM7, 20.4).
 * Confidence: High for the register layouts, write masks, reset values, flag clear and event generation rules; Medium for the counting engine (it counts timer clock
 * events given by the test, not time: the prescaler value is stored but does not divide) and for the lock levels; Low for everything on the pins (outputs and inputs are not modelled).
 *
 * `tim` is the timer number 1..14. Base addresses: TIM2 0x40000000, TIM3 0x40000400, TIM4 0x40000800, TIM5 0x40000C00, TIM6 0x40001000, TIM7 0x40001400, TIM12 0x40001800,
 * TIM13 0x40001C00, TIM14 0x40002000, TIM1 0x40010000, TIM8 0x40010400, TIM9 0x40014000, TIM10 0x40014400, TIM11 0x40014800 (RM0090 Table 1).
 * Registers (word access) by type; a register a type does not have is a fault (strict), the write masks are the defined bits:
 *   CR1 0x00: TIM1 / 8 / 2-5 0x03FF; TIM9 / 12 0x038F; TIM10 / 11 / 13 / 14 0x0387; TIM6 / 7 0x008F.   CR2 0x04: TIM1 / 8 0x7FFD; TIM2-5 0x00F8; TIM9 / 12 and TIM6 / 7 0x0070; none on TIM10 / 11 / 13 / 14.
 *   SMCR 0x08: TIM1 / 8 / 2-5 0xFFF7; TIM9 / 12 0x00F7; none on the others.   DIER 0x0C: TIM1 / 8 0x7FFF; TIM2-5 0x5F5F; TIM9 / 12 0x0047; TIM10 / 11 / 13 / 14 0x0003; TIM6 / 7 0x0101.
 *   SR 0x10 (rc_w0): TIM1 / 8 0x1EFF; TIM2-5 0x1E5F; TIM9 / 12 0x0647; TIM10 / 11 / 13 / 14 0x0203; TIM6 / 7 0x0001.   EGR 0x14 (write only, reads 0): TIM1 / 8 0x00FF; TIM2-5 0x005F; TIM9 / 12 0x0047; TIM10 / 11 / 13 / 14 0x0003; TIM6 / 7 0x0001.
 *   CCMR1 0x18: all but TIM6 / 7, 0xFFFF (TIM10 / 11 / 13 / 14: 0x00FF); CCMR2 0x1C: TIM1 / 8 / 2-5 0xFFFF.   CCER 0x20: TIM1 / 8 0xBFFF; TIM2-5 0xBBBB; TIM9 / 12 0x00BB; TIM10 / 11 / 13 / 14 0x000B.
 *   CNT 0x24 (32 bit on TIM2 / 5, else 16), PSC 0x28 (16), ARR 0x2C (reset all ones, 32 bit on TIM2 / 5), RCR 0x30 (TIM1 / 8, 0x00FF), CCR1 0x34 (all but TIM6 / 7), CCR2 0x38 (TIM1 / 8 / 2-5 / 9 / 12),
 *   CCR3 0x3C and CCR4 0x40 (TIM1 / 8 / 2-5; 32 bit on TIM2 / 5), BDTR 0x44 (TIM1 / 8, 0xFFFF), DCR 0x48 and DMAR 0x4C (TIM1 / 8 / 2-5; DCR 0x1F1F), OR 0x50 (TIM2 0x0C00, TIM5 0x00C0, TIM11 0x0003).
 *
 * Behaviour: EGR bits self-clear; UG (and a counter overflow / underflow = update event) reloads the counter (CNT = 0, or ARR counting down), sets UIF unless UDIS = 1 (UG with URS = 1 does not set UIF), and with AOE
 * sets MOE; CCxG sets CCxIF and, for a channel configured as input (CCxS != 0), captures CNT into CCRx (CCxOF when CCxIF was already set); COMG sets COMIF; TG sets TIF; BG sets BIF and clears MOE. SR bits clear by
 * writing 0. CCRx of an input channel is read-only (a write is ignored and counted), CCxS bits change only while CCxE = 0 (RM0090 "writable only if the channel is OFF"; otherwise ignored and counted).
 * BDTR.LOCK is write-once until reset; LOCK >= 1 protects DTG, BKE, BKP, AOE and the OISx bits of CR2, >= 2 also CCxP / CCxNP and OSSR / OSSI, 3 also OCxM / OCxPE (RM0090 17.4.18). DMAR redirects an access to the register at
 * (DBA + burst index) * 4, the burst index counting 0 .. DBL (17.4.20; a test plays the DMA with plain DMAR accesses). hostTimAdvance counts events of the counter clock while CEN = 1: up (DIR = 0), down (DIR = 1) or
 * centre-aligned (CMS != 0: up to ARR, down to 0, update event at the underflow); RCR counts update events; OPM stops at the update event; compare matches (CNT == CCRx of an output channel) set CCxIF.
 * Interrupt request: hostTimIrqPending(tim) = SR & DIER over the interrupt flags (UIF, CC1-4IF, COMIF, TIF, BIF); DMA requests are not generated (the DMA modes use the T3 DMA model, tests call the hooks).
 *
 * Not modelled: the prescaler division, slave modes and external clocks (SMCR is stored), the output stage (OCxREF, dead time, complementary outputs, MOE gating), input filters and prescalers, the one-pulse
 * trigger, the encoder interface counting, hall sensor XOR, the DMA request generation, debug freeze.
 */
#ifndef FFB_HAL_TIM_H
#define FFB_HAL_TIM_H

#include <stdint.h>

#include "HalRegisterSpace.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { HOST_TIM_COUNT = 14 };

void hostTimRegister(void);
void hostTimResetState(void);

/* The counter clock: advances TIMtim by n events while CEN = 1 (see above). */
void hostTimAdvance(unsigned tim, uint32_t n);
void hostTimSetCounter(unsigned tim, uint32_t value);
/* A capture event on channel ch (1..4): CCRch = value, CCchIF = 1, CCchOF = 1 when CCchIF was already set. Needs the channel configured as input. Returns 1 when captured. */
int hostTimCapture(unsigned tim, unsigned ch, uint32_t value);
/* Sets status flags (SR bits, masked to the timer's SR mask). */
void hostTimRaiseFlags(unsigned tim, uint32_t srMask);
/* A break input event: BIF = 1 and MOE = 0 (TIM1 / TIM8). */
void hostTimBreak(unsigned tim);
/* SR & DIER over the interrupt flags (non-zero: the interrupt request is pending). */
uint32_t hostTimIrqPending(unsigned tim);
/* Writes of RM0090 "write protected / ignored" kinds (input-channel CCR, CCxS with the channel on, locked bits, LOCK after it was set). */
uint32_t hostTimProtectedWrites(unsigned tim);

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_TIM_H */
