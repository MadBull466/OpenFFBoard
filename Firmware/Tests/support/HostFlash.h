/*
 * HostFlash.h
 *
 * Test control interface of the fake HAL FLASH of ffb_flash_tests (platform/src/host_flash.cpp,
 * declarations in platform/target_f407/host_flash_hal.h).
 *
 * The fake models the STM32F407 embedded flash with NOR semantics at the TARGET addresses, because eeprom.c and
 * flash_helpers.cpp dereference absolute addresses (`*(__IO uint16_t*)0x08004000`):
 *   - main flash sectors 0..3 (16 KiB each) at 0x08000000..0x0800FFFF. The EEPROM emulation uses sector 1
 *     (page 0, 0x08004000) and sector 2 (page 1, 0x08008000).
 *   - the OTP area at 0x1FFF7800..0x1FFF7A0F: 16 blocks of 32 data bytes, then 16 lock bytes (one per block).
 * The memory is mapped at exactly these addresses when the test executable starts (see ensureMapped()).
 *
 * Semantics (reference: the ST HAL in Targets/F407VG/Drivers/STM32F4xx_HAL_Driver and RM0090 section 3):
 *   - Erase sets a whole sector to 0xFF. Programming can only clear bits: new = old & data. Requests to set a bit
 *     from 0 to 1 are recorded (State::zeroToOneRequests), the bit stays 0 like on the chip.
 *   - HAL_FLASH_Program while the flash is locked fails with PGSERR (HAL_ERROR, HAL_FLASH_ERROR_PGS), nothing is
 *     written. HAL_FLASHEx_Erase while locked returns HAL_OK and erases nothing (FLASH_CR writes are ignored).
 *   - Error flags still set in FLASH_SR when an operation starts make it fail (HAL_ERROR) before anything is
 *     written; the HAL clears them in FLASH_SetErrorCode, so the next operation works again.
 *   - OTP bytes can never be erased. A block whose lock byte is 0x00 can not be programmed (WRPERR).
 *   - Accesses outside the modelled areas (sectors 4..11, mass erase, other addresses) are not modelled: they fail
 *     with WRPERR and are recorded in State::unexpected. FlashFixture fails the test if that list is not empty.
 *
 * Reset with HostFlash::reset() (done by FlashFixture).
 */
#ifndef HOSTTEST_HOSTFLASH_H_
#define HOSTTEST_HOSTFLASH_H_

#include "main.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace HostFlash {

// ---------------- Layout of the fake (STM32F407, RM0090 table 5) ----------------
constexpr uint32_t MAIN_BASE = 0x08000000;	  //!< Sector 0
constexpr uint32_t SECTOR_SIZE = 0x4000;	  //!< Sectors 0..3 are 16 KiB
constexpr int SECTOR_COUNT = 4;				  //!< Modelled sectors 0..3
constexpr uint32_t MAIN_SIZE = SECTOR_SIZE * SECTOR_COUNT;
constexpr uint32_t EEPROM_PAGE0 = 0x08004000; //!< Sector 1 (PAGE0_BASE_ADDRESS of eeprom.h with the F407 header)
constexpr uint32_t EEPROM_PAGE1 = 0x08008000; //!< Sector 2 (PAGE1_BASE_ADDRESS)
constexpr uint32_t EEPROM_PAGE_SIZE = 0x4000;

constexpr uint32_t OTP_BASE = 0x1FFF7800;	   //!< FLASH_OTP_BASE: 16 data blocks
constexpr uint32_t OTP_BLOCK_SIZE = 32;
constexpr uint32_t OTP_BLOCKS = 16;
constexpr uint32_t OTP_DATA_SIZE = OTP_BLOCK_SIZE * OTP_BLOCKS; //!< 512 bytes
constexpr uint32_t OTP_LOCK_BASE = 0x1FFF7A00; //!< One lock byte per block. 0x00 = block locked
constexpr uint32_t OTP_END = 0x1FFF7A0F;	   //!< FLASH_OTP_END (last lock byte)

/** Maps the fake memory at the fixed target addresses (once). Prints a message and aborts the run if impossible */
void ensureMapped();

/**
 * Erases all modelled memory (sectors 0..3 and OTP to 0xFF), sets the FLASH registers to their state after HAL_Init
 * (locked, no flags, prefetch / instruction cache / data cache enabled), HAL_FLASH_GetError() to none, clears all
 * injections, the power-loss state and the recording.
 */
void reset();

/**
 * Hardware reset after a power cycle or reset: power is back, the FLASH registers return to their state after
 * HAL_Init (locked, no flags) and the error code is cleared. The memory content and the recording are kept,
 * pending injections are cleared. FlashFixture::reboot() calls this and then the firmware start-up (Flash_Init).
 */
void powerOn();

// ---------------- Raw memory access for the test (bypasses the NOR semantics, not recorded) ----------------
uint8_t read8(uint32_t address);
uint16_t read16(uint32_t address);
uint32_t read32(uint32_t address);
uint64_t read64(uint32_t address);
void write8(uint32_t address, uint8_t value);
void write16(uint32_t address, uint16_t value);
void write32(uint32_t address, uint32_t value);
void fill(uint32_t address, uint32_t length, uint8_t value);
bool isErased(uint32_t address, uint32_t length); //!< All bytes 0xFF
std::vector<uint8_t> snapshot(uint32_t address, uint32_t length);
void restore(uint32_t address, const std::vector<uint8_t> &data);

/** Sector number (0..3) of a main flash address, -1 outside the modelled sectors */
int sectorOf(uint32_t address);
inline uint32_t sectorBase(int sector) { return MAIN_BASE + (uint32_t)sector * SECTOR_SIZE; }

// ---------------- Recording ----------------
enum class OpKind { Program, Erase };

struct Operation {
	OpKind kind;
	uint32_t address;		  //!< Program: target address. Erase: base address of the sector (0 if not modelled)
	uint32_t sectorOrType;	  //!< Program: FLASH_TYPEPROGRAM_x. Erase: sector number
	uint64_t data;			  //!< Program: data argument
	HAL_StatusTypeDef result; //!< What the HAL function returned (for erases the status after this sector)
	bool applied;			  //!< The memory was changed as requested (false: error, locked, power lost, injected)
};

struct State {
	std::array<uint32_t, SECTOR_COUNT> programsPerSector{}; //!< Applied program operations per sector
	std::array<uint32_t, SECTOR_COUNT> erasesPerSector{};	//!< Applied sector erases per sector
	uint32_t otpPrograms = 0;		 //!< Applied program operations in the OTP area (data or lock bytes)
	uint32_t programsApplied = 0;	 //!< All applied program operations (main flash + OTP)
	uint32_t erasesApplied = 0;		 //!< All applied sector erases
	uint32_t programCalls = 0;		 //!< HAL_FLASH_Program calls (any result)
	uint32_t eraseCalls = 0;		 //!< HAL_FLASHEx_Erase calls (any result)
	uint32_t zeroToOneRequests = 0;	 //!< Program operations that asked to set a 0 bit to 1 (the bit stayed 0)
	uint32_t lastZeroToOneAddress = 0;
	uint32_t unlocks = 0;			 //!< HAL_FLASH_Unlock calls
	uint32_t locks = 0;				 //!< HAL_FLASH_Lock calls
	uint32_t dataCacheResets = 0;	 //!< __HAL_FLASH_DATA_CACHE_RESET while the data cache was disabled
	uint32_t ignoredDataCacheResets = 0; //!< __HAL_FLASH_DATA_CACHE_RESET while the data cache was enabled (no effect)
	uint32_t eraseCacheFlushes = 0;	 //!< Data cache flushes by HAL_FLASHEx_Erase (FLASH_FlushCaches), not counted above
	uint32_t opsWhilePowerLost = 0;	 //!< Program / erase calls after the power was cut (no effect)
	std::vector<Operation> log;		 //!< Every program call and every sector of every erase call, in order
	std::vector<std::string> unexpected; //!< Accesses the fake does not model (see the file comment)
};
State &state();

/** True while the flash is locked (FLASH_CR LOCK bit) */
bool locked();

// ---------------- Error injection ----------------
enum class Target { Any, Program, Erase };

/**
 * The next `count` program / erase operations (each sector of an erase counts as one) of the given kind fail in
 * the hardware: `srFlags` (FLASH_FLAG_PGSERR, _PGPERR, _PGAERR, _WRPERR, _OPERR) is set in FLASH_SR during the
 * operation, nothing is written, the HAL returns HAL_ERROR and HAL_FLASH_GetError() the matching
 * HAL_FLASH_ERROR_x (FLASH_SetErrorCode). Calls rejected before reaching the hardware do not count.
 */
void failNext(uint32_t count, uint32_t srFlags = FLASH_FLAG_PGSERR, Target target = Target::Any);
/**
 * Like failNext, but the operation returns `status` (HAL_TIMEOUT: BSY stays set; HAL_BUSY: the HAL process lock
 * is taken) and sets no flag. Nothing is written.
 */
void failNextWithStatus(uint32_t count, HAL_StatusTypeDef status, Target target = Target::Any);
/** Error flags already set in FLASH_SR (e.g. left over from an earlier failed operation) */
void setPendingFlags(uint32_t srFlags);
/** HAL_FLASH_Unlock fails (wrong key sequence: the flash stays locked until reset) while set */
void setUnlockFails(bool fails);

// ---------------- Power-loss injection ----------------
/**
 * After `completedPrograms` more applied program operations the power is cut during the next one: of the bits
 * that operation would clear, only those in `tornMask` (bit 0 = bit 0 of the data, as passed to
 * HAL_FLASH_Program) are cleared, the rest keep their old value. From then on every program / erase call has no
 * effect (HAL_ERROR) until powerOn() / FlashFixture::reboot(). The firmware keeps running on the host, so its
 * return values after the cut are meaningless; only the memory content counts.
 */
void cutPowerAfterPrograms(uint32_t completedPrograms, uint64_t tornMask = 0);
/**
 * After `completedErases` more applied sector erases the power is cut during the next sector erase: only the
 * bytes [erasedOffset, erasedOffset + erasedLength) of that sector (offset from the sector base) are erased,
 * the rest keeps its old content. Afterwards like cutPowerAfterPrograms.
 */
void cutPowerDuringErase(uint32_t completedErases, uint32_t erasedOffset = 0, uint32_t erasedLength = 0);
/** True after an armed power cut happened and before powerOn() */
bool powerLost();

} // namespace HostFlash

#endif
