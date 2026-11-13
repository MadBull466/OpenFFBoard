/*
 * HalFlash.h (P12 T7, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Model of the embedded flash of the STM32F405/407 (1 MB): the flash interface registers at 0x40023C00, the main memory 0x08000000-0x080FFFFF with NOR semantics,
 * the OTP area 0x1FFF7800-0x1FFF7A0F (with its lock bytes) and the option byte words. The independent watchdog is in HalIwdg.h.
 * Source: RM0090 Rev 22 chapter 3 (3.3 organisation Table 5, 3.6 erase and program operations, 3.7 option bytes Tables 15 / 16, 3.8 OTP, 3.9 registers 3.9.1-3.9.9,
 * the STM32F405/407 variants). Confidence: High for the registers, masks, reset values, key sequences, sector layout and NOR programming; Medium for the error flag rules
 * (which flag for which fault, EOP after an error, OPERR only with ERRIE) and the completion timing; Low for OTP lock byte handling and anything not in RM0090.
 *
 * Registers (word access at 0x40023C00): ACR 0x00 (stored as written, latency knob of HalPeripherals), KEYR 0x04 (write only, key sequence 0x45670123 then 0xCDEF89AB
 * clears CR.LOCK; any other sequence locks CR until reset: hostFlashBusFaults counts it), OPTKEYR 0x08 (0x08192A3B then 0x4C5D6E7F clears OPTCR.OPTLOCK; wrong sequence locks
 * until reset), SR 0x0C (BSY bit 16 read only; EOP 0, OPERR 1, WRPERR 4, PGAERR 5, PGPERR 6, PGSERR 7 rc_w1), CR 0x10 (reset 0x80000000, mask 0x8301037F: PG 0, SER 1, MER 2,
 * SNB 6:3, PSIZE 9:8, STRT 16, EOPIE 24, ERRIE 25, LOCK 31), OPTCR 0x14 (reset 0x0FFFAAED, mask 0x0FFFFFEF; OPTLOCK 0, OPTSTRT 1, BOR_LEV 3:2, USER 7:5, RDP 15:8, nWRP 27:16).
 * OPTCR1 (0x18) does not exist on the F405/407 (a fault). Access by byte / half-word to OPTCR (the HAL does it) is handled as an access to the word.
 *
 * Behaviour: while LOCK = 1, writes to CR are ignored; while BSY = 1 writes to CR and OPTCR are ignored (the bus would stall: hostFlashStalledWrites counts them).
 * PROGRAM: a store into main flash or OTP with CR.PG = 1 programs the bytes written: allowed when the store width equals the parallelism (PSIZE 0 = 1 byte, 1 = 2, 2 = 4;
 * PSIZE 3 = x64: two word stores at an 8-byte aligned address), otherwise PGPERR; an unaligned store or one that crosses a 16-byte row gives PGAERR; a store with PG = 0 gives
 * PGSERR; a store into a write protected sector (OPTCR nWRP bit 0) gives WRPERR; an OTP store into a locked block (lock byte 0x00) gives WRPERR. A failing store changes nothing. A
 * good store ANDs the data into the memory (1 -> 0 only) and sets BSY. ERASE: STRT = 1 with MER (priority) or SER and SNB (sectors 0-11: 4 x 16 KB, 64 KB, 7 x 128 KB) sets BSY and
 * erases at once to 0xFF (a protected sector, or any protected sector for MER, gives WRPERR and nothing is erased; MER leaves OTP and option bytes alone); STRT with neither SER nor MER, or SNB > 11, does
 * nothing (RM0090: unpredictable, no flag). STRT stays set while BSY. OPTSTRT = 1 (OPTCR unlocked) sets BSY and, at completion, commits the OPTCR fields to the option byte words
 * 0x1FFFC000 (RDP and USER) and 0x1FFFC008 (nWRP, bits 15:12 = 0xF). COMPLETION: BSY clears, EOP = 1 (also after a failed operation that started), STRT / OPTSTRT clear, when the test calls
 * hostFlashComplete(), or on the n-th read of SR while BSY (hostFlashSetPollReads, default 1: a polling loop sees the operation complete at once; 0 = never). With ERRIE = 1 an error flag also sets OPERR.
 * hostFlashIrqPending = (EOPIE & EOP) | (ERRIE & (PGSERR | PGPERR | PGAERR | WRPERR | OPERR)).
 *
 * Not modelled: the erase / program time, the bus stall, the caches (ACR.ICRST / DCRST are plain bits), read protection levels (RDP is a stored field), dual bank devices, PCROP, the
 * sector write protect register lock modes (SPRMOD), the OTP lock byte side effects other than blocking stores, flash contents at reset (the model fills 0xFF).
 */
#ifndef FFB_HAL_FLASH_H
#define FFB_HAL_FLASH_H

#include <stdint.h>

#include "HalRegisterSpace.h"

#ifdef __cplusplus
extern "C" {
#endif

void hostFlashRegister(void);
void hostFlashResetState(void);

/* Completes the operation in progress (BSY -> 0, EOP = 1). Returns 1 when one was running. */
int hostFlashComplete(void);
/* The n-th read of SR while BSY completes the operation (1 = at the first read, default; 0 = never). */
void hostFlashSetPollReads(unsigned n);
/* (EOPIE & EOP) | (ERRIE & error flags). */
uint32_t hostFlashIrqPending(void);
/* Counters: writes ignored because of BSY, wrong key sequences (RM0090 3.6.1: a bus error), writes to CR while locked. */
uint32_t hostFlashStalledWrites(void);
uint32_t hostFlashBusFaults(void);
uint32_t hostFlashLockedWrites(void);
/* Sets bytes of main flash / OTP raw (set-up of a content; works armed or not). */
void hostFlashFill(uint32_t addr, uint8_t value, uint32_t size);
/* Sector number 0-11 of a main flash address, or -1. Start address and size of a sector. */
int hostFlashSectorOf(uint32_t addr);
uint32_t hostFlashSectorBase(unsigned sector);
uint32_t hostFlashSectorSize(unsigned sector);

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_FLASH_H */
