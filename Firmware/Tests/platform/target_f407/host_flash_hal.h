/*
 * Host test replacement for the FLASH part of the STM32F4 HAL (stm32f4xx_hal_flash.h, stm32f4xx_hal_flash_ex.h,
 * the FLASH register block and memory map of stm32f407xx.h, the legacy names of stm32_hal_legacy.h).
 *
 * Only included by main.h of ffb_flash_tests (compile definition HOSTTEST_FLASH_HAL). ffb_target_tests does not
 * define it, so its preprocessed sources do not change. With these definitions constants.h defines OTPMEMORY
 * (FLASH_OTP_BASE and FLASH_OTP_END exist) like on the real F407 target.
 *
 * Constant values are copied from the real headers in Targets/F407VG/Drivers (read only).
 * Implementation (fake flash with NOR semantics at the target addresses): platform/src/host_flash.cpp.
 * Test control: support/HostFlash.h.
 */
#ifndef HOSTTEST_FLASH_HAL_H_
#define HOSTTEST_FLASH_HAL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// stm32f4xx_hal_def.h (GNU compiler branch)
#ifndef __weak
#define __weak __attribute__((weak))
#endif

// ---------------- Memory map (stm32f407xx.h) ----------------
#define FLASH_BASE 0x08000000UL
#define FLASH_END 0x080FFFFFUL
#define FLASH_OTP_BASE 0x1FFF7800UL //!< Base address of the 528 byte OTP area (512 data bytes + 16 lock bytes)
#define FLASH_OTP_END 0x1FFF7A0FUL	//!< End address of the OTP area (last lock byte)

// ---------------- FLASH registers (stm32f407xx.h FLASH_TypeDef) ----------------
typedef struct {
	volatile uint32_t ACR;	   //!< Access control register (caches, prefetch, latency)
	volatile uint32_t KEYR;	   //!< Key register
	volatile uint32_t OPTKEYR; //!< Option byte key register
	volatile uint32_t SR;	   //!< Status register (error flags are write 1 to clear on the chip, see __HAL_FLASH_CLEAR_FLAG)
	volatile uint32_t CR;	   //!< Control register (LOCK, PG, SER, SNB, PSIZE, STRT)
	volatile uint32_t OPTCR;   //!< Option control register
} FLASH_TypeDef;
extern FLASH_TypeDef hosttest_flash_regs; // platform/src/host_flash.cpp
#define FLASH (&hosttest_flash_regs)

#define FLASH_ACR_PRFTEN 0x00000100UL
#define FLASH_ACR_ICEN 0x00000200UL
#define FLASH_ACR_DCEN 0x00000400UL
#define FLASH_ACR_ICRST 0x00000800UL
#define FLASH_ACR_DCRST 0x00001000UL

#define FLASH_SR_EOP 0x00000001UL
#define FLASH_SR_SOP 0x00000002UL
#define FLASH_SR_WRPERR 0x00000010UL
#define FLASH_SR_PGAERR 0x00000020UL
#define FLASH_SR_PGPERR 0x00000040UL
#define FLASH_SR_PGSERR 0x00000080UL
#define FLASH_SR_BSY 0x00010000UL

#define FLASH_CR_PG 0x00000001UL
#define FLASH_CR_SER 0x00000002UL
#define FLASH_CR_MER 0x00000004UL
#define FLASH_CR_SNB 0x000000F8UL
#define FLASH_CR_PSIZE 0x00000300UL
#define FLASH_CR_STRT 0x00010000UL
#define FLASH_CR_LOCK 0x80000000UL

// ---------------- stm32f4xx_hal_flash.h ----------------
#define HAL_FLASH_ERROR_NONE 0x00000000U
#define HAL_FLASH_ERROR_RD 0x00000001U
#define HAL_FLASH_ERROR_PGS 0x00000002U
#define HAL_FLASH_ERROR_PGP 0x00000004U
#define HAL_FLASH_ERROR_PGA 0x00000008U
#define HAL_FLASH_ERROR_WRP 0x00000010U
#define HAL_FLASH_ERROR_OPERATION 0x00000020U

#define FLASH_TYPEPROGRAM_BYTE 0x00000000U
#define FLASH_TYPEPROGRAM_HALFWORD 0x00000001U
#define FLASH_TYPEPROGRAM_WORD 0x00000002U
#define FLASH_TYPEPROGRAM_DOUBLEWORD 0x00000003U

#define FLASH_FLAG_EOP FLASH_SR_EOP
#define FLASH_FLAG_OPERR FLASH_SR_SOP
#define FLASH_FLAG_WRPERR FLASH_SR_WRPERR
#define FLASH_FLAG_PGAERR FLASH_SR_PGAERR
#define FLASH_FLAG_PGPERR FLASH_SR_PGPERR
#define FLASH_FLAG_PGSERR FLASH_SR_PGSERR
#define FLASH_FLAG_BSY FLASH_SR_BSY

#define __HAL_FLASH_GET_FLAG(__FLAG__) ((FLASH->SR & (__FLAG__)))
/**
 * HAL: (FLASH->SR = (__FLAG__)). The status register flags are "write 1 to clear" on the chip (RM0090 3.9.5),
 * which a plain struct member can not reproduce, so the fake clears the written bits in a function.
 */
#define __HAL_FLASH_CLEAR_FLAG(__FLAG__) hosttest_flash_clear_flag(__FLAG__)
void hosttest_flash_clear_flag(uint32_t flags);

// Cache macros: same text as the HAL (they only change ACR) except the reset, which is recorded by the fake
#define __HAL_FLASH_DATA_CACHE_ENABLE() (FLASH->ACR |= FLASH_ACR_DCEN)
#define __HAL_FLASH_DATA_CACHE_DISABLE() (FLASH->ACR &= (~FLASH_ACR_DCEN))
#define __HAL_FLASH_INSTRUCTION_CACHE_ENABLE() (FLASH->ACR |= FLASH_ACR_ICEN)
#define __HAL_FLASH_INSTRUCTION_CACHE_DISABLE() (FLASH->ACR &= (~FLASH_ACR_ICEN))
/**
 * HAL: do {FLASH->ACR |= FLASH_ACR_DCRST; FLASH->ACR &= ~FLASH_ACR_DCRST;} while(0U).
 * The fake records the reset (HostFlash::state().dataCacheResets). RM0090 3.9.1: DCRST can only be written
 * while the data cache is disabled, a reset with DCEN set is recorded as ignored.
 */
#define __HAL_FLASH_DATA_CACHE_RESET() hosttest_flash_data_cache_reset()
void hosttest_flash_data_cache_reset(void);

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
uint32_t HAL_FLASH_GetError(void);

// ---------------- stm32f4xx_hal_flash_ex.h ----------------
typedef struct {
	uint32_t TypeErase;	   //!< FLASH_TYPEERASE_SECTORS or FLASH_TYPEERASE_MASSERASE
	uint32_t Banks;		   //!< Mass erase only
	uint32_t Sector;	   //!< First sector to erase
	uint32_t NbSectors;	   //!< Number of sectors
	uint32_t VoltageRange; //!< FLASH_VOLTAGE_RANGE_x (parallelism)
} FLASH_EraseInitTypeDef;

#define FLASH_TYPEERASE_SECTORS 0x00000000U
#define FLASH_TYPEERASE_MASSERASE 0x00000001U

#define FLASH_VOLTAGE_RANGE_1 0x00000000U
#define FLASH_VOLTAGE_RANGE_2 0x00000001U
#define FLASH_VOLTAGE_RANGE_3 0x00000002U
#define FLASH_VOLTAGE_RANGE_4 0x00000003U

#define FLASH_BANK_1 1U

#define FLASH_SECTOR_0 0U
#define FLASH_SECTOR_1 1U
#define FLASH_SECTOR_2 2U
#define FLASH_SECTOR_3 3U
#define FLASH_SECTOR_4 4U
#define FLASH_SECTOR_5 5U
#define FLASH_SECTOR_6 6U
#define FLASH_SECTOR_7 7U
#define FLASH_SECTOR_8 8U
#define FLASH_SECTOR_9 9U
#define FLASH_SECTOR_10 10U
#define FLASH_SECTOR_11 11U

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);

// ---------------- Legacy names used by eeprom.c (Legacy/stm32_hal_legacy.h) ----------------
#define TYPEPROGRAM_BYTE FLASH_TYPEPROGRAM_BYTE
#define TYPEPROGRAM_HALFWORD FLASH_TYPEPROGRAM_HALFWORD
#define TYPEPROGRAM_WORD FLASH_TYPEPROGRAM_WORD
#define TYPEPROGRAM_DOUBLEWORD FLASH_TYPEPROGRAM_DOUBLEWORD
#define TYPEERASE_SECTORS FLASH_TYPEERASE_SECTORS
#define TYPEERASE_MASSERASE FLASH_TYPEERASE_MASSERASE
#define VOLTAGE_RANGE_1 FLASH_VOLTAGE_RANGE_1
#define VOLTAGE_RANGE_2 FLASH_VOLTAGE_RANGE_2
#define VOLTAGE_RANGE_3 FLASH_VOLTAGE_RANGE_3
#define VOLTAGE_RANGE_4 FLASH_VOLTAGE_RANGE_4

#ifdef __cplusplus
}
#endif

#endif
