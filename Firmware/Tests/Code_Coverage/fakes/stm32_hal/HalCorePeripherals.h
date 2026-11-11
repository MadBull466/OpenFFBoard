/*
 * HalCorePeripherals.h (P12 real ST HAL tests, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Register model of the Cortex-M4 system control space 0xE000E000-0xE000EFFF on the trapping bus
 * (HalRegisterSpace.h). Source: PM0214 Rev 10 "STM32 Cortex-M4 MCUs and MPUs programming manual" (git-ignored, in
 * Tests/Code_Coverage/shared/specs/), chapter 4; the number of interrupts from RM0090 Rev 22 Table 62 (STM32F405xx/07xx:
 * positions 0..81, so 82 interrupts). Strict: an address not listed below, a write to a read-only register or a
 * read of a write-only register is a fault.
 *
 * MODELS
 *  - ACTLR 0xE000E008 (4.4.1): stored, reset 0.
 *  - SysTick (4.5, 0xE000E010-0xE000E01C): STK_CTRL ENABLE / TICKINT / CLKSOURCE (bits 2:0) rw, COUNTFLAG (bit 16)
 *    read-only and cleared by a read of STK_CTRL (4.5.1 "Returns 1 if timer counted to 0 since last time this was
 *    read"); set only by hostSysTickCountedToZero (no counting is simulated). STK_LOAD bits 23:0. STK_VAL: a write of
 *    any value clears it to 0 and clears COUNTFLAG (4.5.3); reads return the stored value. STK_CALIB read-only,
 *    0xC0000000 (Table 55 reset value; the text of 4.5.4 says 0x0000000, the two disagree; the HAL does not read it).
 *  - NVIC (4.3): ISER0-7 / ICER0-7 write 1 sets / clears the enable bit, read the enable state; ISPR / ICPR the same
 *    for pending; IABR read-only (active state from hostNvicSetActive). IPR0-59 byte fields, only bits 7:4
 *    implemented (bits 3:0 read 0, writes ignored; 4.3.7). Interrupts 82..239 do not exist on the STM32F407: their
 *    bits read 0 and writes are ignored. STIR 0xE000EF00 (4.3.8): write-only, writing INTID sets that interrupt
 *    pending.
 *  - SCB (4.4): CPUID read-only 0x410FC241 (Table 53). ICSR: PENDSVSET / PENDSTSET (bits 28 / 26) set by writing 1,
 *    cleared by PENDSVCLR / PENDSTCLR (27 / 25); NMIPENDSET (31) set by writing 1; the clear bits read 0; other fields
 *    read 0. VTOR: TBLOFF bits 29:9. AIRCR (4.4.5): a write without VECTKEY 0x5FA in bits 31:16 is ignored; with
 *    the key PRIGROUP (10:8) is stored and SYSRESETREQ (bit 2) records a system reset request (reads 0); reads return
 *    0xFA05 << 16 | PRIGROUP << 8 (ENDIANESS 0: little-endian). Writing 1 to VECTCLRACTIVE / VECTRESET (bits 1 / 0;
 *    "behavior is unpredictable") is a fault. SCR bits 1, 2, 4. CCR stored (reset 0x00000200). SHPR1-3: byte
 *    fields PRI_4..PRI_15 with bits 7:4 implemented (4.4.8; reserved bytes read 0). SHCSR, CFSR (write 1 clears),
 *    HFSR (write 1 clears), MMFAR, BFAR, AFSR stored.
 *  - MPU (4.2): TYPER read-only 0x00000800 (DREGION 8). CTRL bits 2:0. RNR region number. RBAR: a write with VALID
 *    (bit 4) set first sets RNR to REGION (bits 3:0); the ADDR bits 31:5 are stored for the region in RNR; reads
 *    return the region's address | RNR. RASR stored per region (fields XN, AP, TEX, S, C, B, SRD, SIZE, ENABLE:
 *    mask 0x173FFF3F). The alias registers RBAR_A1-A3 / RASR_A1-A3 (0xE000EDA4-0xE000EDB8) act like RBAR / RASR.
 *
 * DOES NOT MODEL: exception entry / preemption (no handler is ever run by the model; tests call the IRQ handlers),
 * SysTick counting, the actual system reset (only the request is recorded), MPU address checking, the FPU registers
 * (CPACR, FPCCR, ...: faults), the access width (byte writes to IPR / SHPR are handled through the aligned word).
 *
 * CONFIDENCE: High for the register semantics listed (PM0214 text), Medium for STK_CALIB (the document disagrees with
 * itself) and for the RAZ/WI handling of interrupts 82..239 (PM0214 only says the number is product dependent).
 */
#ifndef FFB_HAL_CORE_PERIPHERALS_H
#define FFB_HAL_CORE_PERIPHERALS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { HOST_NVIC_IRQS = 82 };

/* Registers the system control space model with the bus (idempotent). */
void hostCoreRegister(void);
/* Resets the model state outside the registers (reset request, active bits). */
void hostCoreResetState(void);

int hostNvicEnabled(unsigned irq);
int hostNvicPending(unsigned irq);
void hostNvicSetActive(unsigned irq, int active);
/* Number of accepted AIRCR writes with SYSRESETREQ = 1 since the last reset. */
uint32_t hostScbResetRequests(void);
/* Sets STK_CTRL.COUNTFLAG as if the counter had reached 0. */
void hostSysTickCountedToZero(void);

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_CORE_PERIPHERALS_H */
