/*
 * HalRegisterSpace.h (P12 real ST HAL tests, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * The host "bus" of the real HAL. It maps host RAM at the real STM32F407 register addresses, so the unmodified HAL
 * can dereference its fixed peripheral pointers (GPIOA, RCC, SCB, ...) in the 32-bit (-m32) host process, and it
 * traps every access while armed so peripheral models can apply the RM0090 register behaviour.
 *
 * MODELS (address ranges; RM0090 Rev 22 section 2.3 Table 1 "STM32F4xx register boundary addresses"; the system
 * control space from CMSIS Drivers/CMSIS/Include/core_cm4.h:1550-1557, SCS_BASE 0xE000E000; the Cortex-M4
 * programming manual PM0214 is not yet in the specs folder)
 *  - 0x40000000-0x4007FFFF: APB1, APB2, AHB1 (GPIOA 0x40020000, RCC 0x40023800, FLASH interface 0x40023C00, DMA).
 *  - 0x50000000-0x50060FFF: AHB2 (USB OTG FS 0x50000000-0x5003FFFF ... RNG 0x50060800-0x50060BFF).
 *  - 0xE000E000-0xE000EFFF: Cortex-M4 system control space (SysTick, NVIC, SCB).
 *  - 0xE0042000-0xE0042FFF: DBGMCU (RM0090 section 38.16).
 *  - 0x1FFF7000-0x1FFF7FFF: unique device id 0x1FFF7A10 and flash size 0x1FFF7A22 (RM0090 sections 39.1, 39.2).
 *  - 0x1FFFC000-0x1FFFCFFF: option bytes (RM0090 section 3.7, Table 5: 0x1FFFC000-0x1FFFC00F).
 *  - 0x42000000-0x42FFFFFF: bit-band alias of 0x40000000-0x4007FFFF (RM0090 2.3.3; the HAL writes some single bits
 *    this way, e.g. SYSCFG_CMPCR.CMP_PD). Built in: a write is a read-modify-write of the target bit through the
 *    target register's model, a read returns the target bit (see HalRegisterSpace.c).
 *
 * ACCESS TRAPPING (Linux i386 only). hostRegsArm() makes every mapped page inaccessible (PROT_NONE). An access then
 * raises SIGSEGV; the handler finds the model of the address, unprotects the pages, runs the model's beforeRead /
 * beforeWrite, remembers the old word, and sets the x86 trap flag so the CPU executes exactly that one instruction
 * and raises SIGTRAP. The SIGTRAP handler runs the model's afterWrite(old, new) or afterRead, logs the access and
 * protects the pages again. A read-modify-write instruction (x86 `or [mem], x`) is reported as one write.
 * So a model sees every single access in program order, also several accesses inside one HAL call (for example the
 * three-write GPIOx_LCKR lock sequence), and can give write-only registers their read value (BSRR reads 0).
 *
 * STRICT: an access to a mapped address that no model covers, or that a model rejects (reserved offset, write to a
 * read-only register), is recorded as a fault (a rejected write is undone). HalFixture fails the test case if any
 * fault was recorded. A fault outside the mapped ranges is passed on to the previous SIGSEGV handler (doctest).
 *
 * DOES NOT MODEL
 *  - The access width (byte / half-word / word): every access is handled as an access to its aligned word.
 *  - Bus faults, access timing, the SRAM bit-band region, flash memory (0x08000000), SRAM, FSMC.
 *  - What a peripheral does with its clock off: RM0090 does not say; models only count such accesses.
 *
 * CONFIDENCE: High for the addresses and the trapping (every access is seen); per model see HalPeripherals.h.
 *
 * Model callbacks run inside the signal handlers: they must not call doctest or allocate memory.
 */
#ifndef FFB_HAL_REGISTER_SPACE_H
#define FFB_HAL_REGISTER_SPACE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct HostRegModel {
	const char *name;
	uintptr_t base;
	uint32_t size;
	/* Writes the RM0090 reset values with hostRegsPoke (called by hostRegsReset with the bus disarmed). */
	void (*reset)(void);
	/* Before a read executes; may update the word about to be read. Returns 0 to reject the read (fault). */
	int (*beforeRead)(uintptr_t addr);
	/* After a read executed (read side effects). May be NULL. */
	void (*afterRead)(uintptr_t addr, uint32_t value);
	/* Before a write executes. Returns 0 to reject the write (fault; the old word is restored). May be NULL. */
	int (*beforeWrite)(uintptr_t addr);
	/* After a write executed, with the old word and the word the CPU wrote; stores the resulting register value. */
	void (*afterWrite)(uintptr_t addr, uint32_t oldValue, uint32_t written);
} HostRegModel;

typedef struct HostRegAccess {
	uintptr_t addr;   /* aligned word address */
	uint32_t value;   /* read: value read; write: register value after the model ran */
	uint32_t written; /* write: raw word the CPU wrote */
	uint8_t write;
} HostRegAccess;

/* Maps every range once per process. Returns 1 when all ranges are mapped at their exact addresses, else 0. */
int hostRegsMap(void);
int hostRegsMapped(void);
const char *hostRegsError(void);
/* 1 if [addr, addr + size) lies inside one mapped range. */
int hostRegsContains(uintptr_t addr, uint32_t size);

/* Registers a model (static storage, at most 64; the same model twice is a no-op). */
void hostRegsAddModel(const HostRegModel *model);
/* Disarms, zeroes every range, runs every model's reset(), clears faults and the access log. */
void hostRegsReset(void);
/* Installs the SIGSEGV / SIGTRAP handlers (again each time: doctest installs its own per test case) and protects
 * the pages. */
void hostRegsArm(void);
/* Unprotects the pages: raw access, no models. */
void hostRegsDisarm(void);
int hostRegsArmed(void);

/* Raw word access that bypasses trapping and models (set-up, assertions, models). Works armed or not. */
uint32_t hostRegsPeek(uintptr_t addr);
void hostRegsPoke(uintptr_t addr, uint32_t value);

/* Faults. hostRegsAddFault may be called from models (fixed buffers). */
void hostRegsAddFault(const char *what, uintptr_t addr, uint32_t value);
size_t hostRegsFaultCount(void);
const char *hostRegsFaultText(size_t index); /* "" if out of range */
void hostRegsClearFaults(void);

/* Access log of the armed bus (the first 4096 accesses after the last clear; hostRegsLogDropped counts the rest). */
size_t hostRegsLogCount(void);
const HostRegAccess *hostRegsLogAt(size_t index); /* NULL if out of range */
size_t hostRegsLogDropped(void);
void hostRegsLogClear(void);
size_t hostRegsWritesTo(uintptr_t addr);
size_t hostRegsReadsFrom(uintptr_t addr);

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_REGISTER_SPACE_H */
