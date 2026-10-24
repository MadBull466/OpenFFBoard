/*
 * host_flash.cpp
 *
 * Fake HAL FLASH of ffb_flash_tests: the STM32F407 embedded flash with NOR semantics, mapped at the target
 * addresses (sectors 0..3 at 0x08000000, OTP at 0x1FFF7800). Declarations: platform/target_f407/host_flash_hal.h.
 * Test control: support/HostFlash.h (semantics, injections, recording).
 *
 * Each HAL function cites the function of the real ST HAL it reproduces
 * (Targets/F407VG/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c and stm32f4xx_hal_flash_ex.c, read only).
 * Hardware behaviour below the HAL follows RM0090 (STM32F405/407 reference manual) section 3.
 *
 * Fixed-address memory: eeprom.c and flash_helpers.cpp read the flash through absolute addresses held in
 * uint32_t, so the fake memory must exist at exactly these addresses in the test process. ensureMapped() maps two
 * 64 KiB regions (0x08000000 and 0x1FFF0000; 64 KiB keeps them aligned to every page size and to the Windows
 * allocation granularity) when the executable starts:
 *   - Linux / macOS / other POSIX: mmap with the address as hint, plus MAP_FIXED_NOREPLACE where defined (Linux >=
 *     4.17). An existing mapping is never replaced (no MAP_FIXED): if the kernel returns another address the range
 *     is in use, the probe mapping is released and the run is aborted.
 *   - Windows (MinGW): VirtualAlloc with the fixed base address (fails if the range is in use).
 * If the range can not be mapped the run is aborted with a message. There is no fallback: a relocated memory
 * would require changing the firmware sources.
 */
#include "HostFlash.h"

#include "main.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#if defined(_WIN32)
#include <windows.h>
#else
#include <sys/mman.h>
#ifndef MAP_ANONYMOUS
#define MAP_ANONYMOUS MAP_ANON
#endif
#endif

FLASH_TypeDef hosttest_flash_regs = {};

namespace {

constexpr uint32_t MAP_MAIN_BASE = HostFlash::MAIN_BASE; // 64 KiB: sectors 0..3
constexpr uint32_t MAP_OTP_BASE = 0x1FFF0000;			 // 64 KiB around the OTP area
constexpr uint32_t MAP_SIZE = 0x10000;

constexpr uint32_t SR_ERRORS = FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR;
constexpr uint32_t SR_W1C = FLASH_FLAG_EOP | SR_ERRORS;
constexpr uint32_t ACR_AFTER_HAL_INIT = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN; // HAL_Init with stm32f4xx_hal_conf.h of the F407
constexpr uint32_t OPTCR_RESET = 0x0FFFAAEDUL; // RM0090 3.9.10 reset value

bool mapped = false;
uint32_t errorCode = HAL_FLASH_ERROR_NONE; // pFlash.ErrorCode

struct Injection {
	uint32_t count = 0;
	uint32_t flags = 0;
	HAL_StatusTypeDef status = HAL_OK; // != HAL_OK: returned instead of setting flags
	HostFlash::Target target = HostFlash::Target::Any;
	uint32_t skip = 0; // matching operations that still succeed before the first injected failure (failNextAfter)
};
Injection injection;
HAL_StatusTypeDef forcedWaitStatus = HAL_OK; // Returned by the next FLASH_WaitForLastOperation (injected / power lost)
bool unlockFails = false;

struct PowerCut {
	bool programArmed = false;
	uint32_t programsLeft = 0;
	uint64_t tornMask = 0;
	bool eraseArmed = false;
	uint32_t erasesLeft = 0;
	uint32_t erasedOffset = 0;
	uint32_t erasedLength = 0;
	bool lost = false;
};
PowerCut power;

inline uint8_t *mem(uint32_t address) { return reinterpret_cast<uint8_t *>(static_cast<uintptr_t>(address)); }

bool inRange(uint32_t address, uint32_t length, uint32_t base, uint32_t size) {
	return address >= base && (uint64_t)address + length <= (uint64_t)base + size;
}
bool inMain(uint32_t address, uint32_t length) { return inRange(address, length, HostFlash::MAIN_BASE, HostFlash::MAIN_SIZE); }
bool inOtp(uint32_t address, uint32_t length) { return inRange(address, length, HostFlash::OTP_BASE, HostFlash::OTP_END + 1 - HostFlash::OTP_BASE); }
bool inMapping(uint32_t address, uint32_t length) {
	return inRange(address, length, MAP_MAIN_BASE, MAP_SIZE) || inRange(address, length, MAP_OTP_BASE, MAP_SIZE);
}

void checkRaw(uint32_t address, uint32_t length) {
	HostFlash::ensureMapped();
	if (!inMapping(address, length)) {
		char msg[96];
		std::snprintf(msg, sizeof(msg), "HostFlash: raw access to 0x%08lX (%lu bytes) outside the fake flash", (unsigned long)address, (unsigned long)length);
		throw std::out_of_range(msg);
	}
}

std::string hex(uint32_t value) {
	char buf[16];
	std::snprintf(buf, sizeof(buf), "0x%08lX", (unsigned long)value);
	return buf;
}

[[noreturn]] void mappingFailed(uint32_t base, const char *reason) {
	std::fprintf(stderr,
				 "\nffb_flash_tests: FATAL: cannot map the fake flash memory at 0x%08lX (%lu KiB): %s\n"
				 "  eeprom.c and flash_helpers.cpp read the flash through absolute target addresses, so the fake memory\n"
				 "  must exist at exactly these addresses. See Firmware/Tests/README.md (ffb_flash_tests).\n",
				 (unsigned long)base, (unsigned long)(MAP_SIZE / 1024), reason);
	std::fflush(stderr);
	std::abort();
}

void mapFixed(uint32_t base) {
#if defined(_WIN32)
	void *p = VirtualAlloc(reinterpret_cast<void *>(static_cast<uintptr_t>(base)), MAP_SIZE, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
	if (p == nullptr) {
		char reason[64];
		std::snprintf(reason, sizeof(reason), "VirtualAlloc failed, GetLastError() = %lu", (unsigned long)GetLastError());
		mappingFailed(base, reason);
	}
	if (p != reinterpret_cast<void *>(static_cast<uintptr_t>(base))) {
		mappingFailed(base, "VirtualAlloc returned another address");
	}
#else
	int flags = MAP_PRIVATE | MAP_ANONYMOUS;
#ifdef MAP_FIXED_NOREPLACE
	flags |= MAP_FIXED_NOREPLACE; // Linux >= 4.17: fails with EEXIST instead of replacing a mapping
#endif
	void *want = reinterpret_cast<void *>(static_cast<uintptr_t>(base));
	void *p = mmap(want, MAP_SIZE, PROT_READ | PROT_WRITE, flags, -1, 0);
	if (p == MAP_FAILED) {
		mappingFailed(base, std::strerror(errno));
	}
	if (p != want) {
		// The address was only a hint (no MAP_FIXED_NOREPLACE, or a kernel that ignores it) and the range is in use
		munmap(p, MAP_SIZE);
		mappingFailed(base, "the address range is already in use (the kernel returned another address)");
	}
#endif
}

void setRegistersAfterReset() {
	hosttest_flash_regs.ACR = ACR_AFTER_HAL_INIT;
	hosttest_flash_regs.KEYR = 0;
	hosttest_flash_regs.OPTKEYR = 0;
	hosttest_flash_regs.SR = 0;
	hosttest_flash_regs.CR = FLASH_CR_LOCK; // RM0090 3.9.7: reset value 0x80000000
	hosttest_flash_regs.OPTCR = OPTCR_RESET;
	errorCode = HAL_FLASH_ERROR_NONE;
}

void clearInjections() {
	injection = Injection();
	forcedWaitStatus = HAL_OK;
	unlockFails = false;
	power = PowerCut();
}

/** Models FLASH_SetErrorCode (stm32f4xx_hal_flash.c): error flags -> pFlash.ErrorCode, flags cleared */
void setErrorCode() {
	uint32_t sr = hosttest_flash_regs.SR;
	if (sr & FLASH_FLAG_WRPERR) errorCode |= HAL_FLASH_ERROR_WRP;
	if (sr & FLASH_FLAG_PGAERR) errorCode |= HAL_FLASH_ERROR_PGA;
	if (sr & FLASH_FLAG_PGPERR) errorCode |= HAL_FLASH_ERROR_PGP;
	if (sr & FLASH_FLAG_PGSERR) errorCode |= HAL_FLASH_ERROR_PGS;
	if (sr & FLASH_FLAG_OPERR) errorCode |= HAL_FLASH_ERROR_OPERATION;
	hosttest_flash_regs.SR &= ~SR_ERRORS;
}

/**
 * Models FLASH_WaitForLastOperation (stm32f4xx_hal_flash.c): clears the error code, (BSY is never set by the fake,
 * an injected HAL_TIMEOUT stands for a BSY flag that does not clear), clears EOP, returns HAL_ERROR with the error
 * code set if an error flag is set.
 */
HAL_StatusTypeDef waitForLastOperation() {
	errorCode = HAL_FLASH_ERROR_NONE;
	if (forcedWaitStatus != HAL_OK) {
		HAL_StatusTypeDef s = forcedWaitStatus;
		forcedWaitStatus = HAL_OK;
		return s;
	}
	if (hosttest_flash_regs.SR & FLASH_FLAG_EOP) {
		hosttest_flash_regs.SR &= ~FLASH_FLAG_EOP;
	}
	if (hosttest_flash_regs.SR & SR_ERRORS) {
		setErrorCode();
		return HAL_ERROR;
	}
	return HAL_OK;
}

/** Consumes an armed injection for this kind of operation. True if the operation fails */
bool takeInjection(HostFlash::Target kind) {
	if (injection.count == 0 || (injection.target != HostFlash::Target::Any && injection.target != kind)) {
		return false;
	}
	if (injection.skip > 0) {
		injection.skip--;
		return false;
	}
	injection.count--;
	if (injection.status != HAL_OK) {
		forcedWaitStatus = injection.status;
	} else {
		hosttest_flash_regs.SR |= injection.flags;
	}
	return true;
}

void unexpected(const std::string &what) {
	HostFlash::state().unexpected.push_back(what);
	hosttest_flash_regs.SR |= FLASH_FLAG_WRPERR;
}

/** The program operation in the flash interface after FLASH_Program_Byte/HalfWord/Word/DoubleWord set PG and PSIZE */
bool programHardware(uint32_t typeProgram, uint32_t address, uint64_t data) {
	uint32_t size = typeProgram == FLASH_TYPEPROGRAM_BYTE ? 1 : typeProgram == FLASH_TYPEPROGRAM_HALFWORD ? 2 : typeProgram == FLASH_TYPEPROGRAM_WORD ? 4 : 8;
	if (takeInjection(HostFlash::Target::Program)) {
		return false;
	}
	if (hosttest_flash_regs.CR & FLASH_CR_LOCK) {
		// RM0090 3.9.7: FLASH_CR can not be written while LOCK is set, so PG stays 0. A write to the flash memory
		// without PG set raises PGSERR (RM0090 3.9.5). Nothing is programmed.
		hosttest_flash_regs.SR |= FLASH_FLAG_PGSERR;
		return false;
	}
	bool otp = inOtp(address, size);
	if (!inMain(address, size) && !otp) {
		unexpected("HAL_FLASH_Program at " + hex(address) + " (not modelled: only sectors 0..3 and the OTP area)");
		return false;
	}
	if ((address & 0xFU) + size > 16U) {
		// RM0090 3.9.5 PGAERR: the data to program can not be contained in the same 128-bit flash row
		hosttest_flash_regs.SR |= FLASH_FLAG_PGAERR;
		return false;
	}
	if (otp && address < HostFlash::OTP_LOCK_BASE) {
		uint32_t block = (address - HostFlash::OTP_BASE) / HostFlash::OTP_BLOCK_SIZE;
		if (*mem(HostFlash::OTP_LOCK_BASE + block) == 0x00) {
			// RM0090 3.7: a block whose lock byte is 0x00 can not be programmed any more
			hosttest_flash_regs.SR |= FLASH_FLAG_WRPERR;
			return false;
		}
	}

	bool cut = false;
	if (power.programArmed) {
		if (power.programsLeft == 0) {
			cut = true;
		} else {
			power.programsLeft--;
		}
	}

	bool zeroToOne = false;
	for (uint32_t i = 0; i < size; i++) {
		uint8_t old = *mem(address + i);
		uint8_t d = (uint8_t)(data >> (8 * i));
		if ((uint8_t)(d & ~old) != 0) {
			zeroToOne = true; // NOR flash: a programmed 0 can only be set to 1 by an erase
		}
		uint8_t clear = (uint8_t)(old & ~d);
		if (cut) {
			clear &= (uint8_t)(power.tornMask >> (8 * i));
		}
		*mem(address + i) = (uint8_t)(old & ~clear);
	}
	if (cut) {
		power.programArmed = false;
		power.lost = true;
		forcedWaitStatus = HAL_ERROR;
		return false;
	}
	HostFlash::State &st = HostFlash::state();
	if (zeroToOne) {
		st.zeroToOneRequests++;
		st.lastZeroToOneAddress = address;
	}
	st.programsApplied++;
	if (otp) {
		st.otpPrograms++;
	} else {
		st.programsPerSector[HostFlash::sectorOf(address)]++;
	}
	return true;
}

/** The sector erase in the flash interface after FLASH_Erase_Sector set SER, SNB and STRT */
bool eraseHardware(uint32_t sector) {
	if (takeInjection(HostFlash::Target::Erase)) {
		return false;
	}
	if (hosttest_flash_regs.CR & FLASH_CR_LOCK) {
		// RM0090 3.9.7: FLASH_CR writes are ignored while LOCK is set: SER and STRT are never set, nothing is erased
		// and no error flag is raised, so the HAL returns HAL_OK.
		return false;
	}
	if (sector >= (uint32_t)HostFlash::SECTOR_COUNT) {
		unexpected("HAL_FLASHEx_Erase of sector " + std::to_string(sector) + " (not modelled: only sectors 0..3)");
		return false;
	}
	uint32_t base = HostFlash::sectorBase((int)sector);
	if (power.eraseArmed) {
		if (power.erasesLeft == 0) {
			uint32_t from = power.erasedOffset < HostFlash::SECTOR_SIZE ? power.erasedOffset : HostFlash::SECTOR_SIZE;
			uint32_t len = power.erasedLength < HostFlash::SECTOR_SIZE - from ? power.erasedLength : HostFlash::SECTOR_SIZE - from;
			std::memset(mem(base + from), 0xFF, len);
			power.eraseArmed = false;
			power.lost = true;
			forcedWaitStatus = HAL_ERROR;
			return false;
		}
		power.erasesLeft--;
	}
	std::memset(mem(base), 0xFF, HostFlash::SECTOR_SIZE);
	HostFlash::State &st = HostFlash::state();
	st.erasesApplied++;
	st.erasesPerSector[sector]++;
	return true;
}

/** Models FLASH_FlushCaches (stm32f4xx_hal_flash_ex.c), called at the end of HAL_FLASHEx_Erase */
void flushCaches() {
	// Instruction cache: disable, reset, enable (not recorded). Data cache: same, recorded separately from
	// __HAL_FLASH_DATA_CACHE_RESET calls of the firmware
	if (hosttest_flash_regs.ACR & FLASH_ACR_DCEN) {
		__HAL_FLASH_DATA_CACHE_DISABLE();
		HostFlash::state().eraseCacheFlushes++;
		__HAL_FLASH_DATA_CACHE_ENABLE();
	}
}

struct MapAtStartup {
	MapAtStartup() {
		HostFlash::ensureMapped();
		HostFlash::reset();
	}
};
MapAtStartup mapAtStartup;

} // namespace

namespace HostFlash {

void ensureMapped() {
	if (mapped) {
		return;
	}
	mapFixed(MAP_MAIN_BASE);
	mapFixed(MAP_OTP_BASE);
	mapped = true;
}

State &state() {
	static State s;
	return s;
}

void reset() {
	ensureMapped();
	std::memset(mem(MAP_MAIN_BASE), 0xFF, MAP_SIZE);
	std::memset(mem(MAP_OTP_BASE), 0xFF, MAP_SIZE);
	setRegistersAfterReset();
	clearInjections();
	state() = State();
}

void powerOn() {
	setRegistersAfterReset();
	clearInjections();
}

bool locked() { return (hosttest_flash_regs.CR & FLASH_CR_LOCK) != 0; }

uint8_t read8(uint32_t address) {
	checkRaw(address, 1);
	return *mem(address);
}
uint16_t read16(uint32_t address) {
	checkRaw(address, 2);
	uint16_t v;
	std::memcpy(&v, mem(address), sizeof(v));
	return v;
}
uint32_t read32(uint32_t address) {
	checkRaw(address, 4);
	uint32_t v;
	std::memcpy(&v, mem(address), sizeof(v));
	return v;
}
uint64_t read64(uint32_t address) {
	checkRaw(address, 8);
	uint64_t v;
	std::memcpy(&v, mem(address), sizeof(v));
	return v;
}
void write8(uint32_t address, uint8_t value) {
	checkRaw(address, 1);
	*mem(address) = value;
}
void write16(uint32_t address, uint16_t value) {
	checkRaw(address, 2);
	std::memcpy(mem(address), &value, sizeof(value));
}
void write32(uint32_t address, uint32_t value) {
	checkRaw(address, 4);
	std::memcpy(mem(address), &value, sizeof(value));
}
void fill(uint32_t address, uint32_t length, uint8_t value) {
	checkRaw(address, length);
	std::memset(mem(address), value, length);
}
bool isErased(uint32_t address, uint32_t length) {
	checkRaw(address, length);
	for (uint32_t i = 0; i < length; i++) {
		if (*mem(address + i) != 0xFF) {
			return false;
		}
	}
	return true;
}
std::vector<uint8_t> snapshot(uint32_t address, uint32_t length) {
	checkRaw(address, length);
	return std::vector<uint8_t>(mem(address), mem(address) + length);
}
void restore(uint32_t address, const std::vector<uint8_t> &data) {
	checkRaw(address, (uint32_t)data.size());
	if (!data.empty()) {
		std::memcpy(mem(address), data.data(), data.size());
	}
}

int sectorOf(uint32_t address) {
	if (!inMain(address, 1)) {
		return -1;
	}
	return (int)((address - MAIN_BASE) / SECTOR_SIZE);
}

void failNext(uint32_t count, uint32_t srFlags, Target target) {
	injection = Injection{count, srFlags, HAL_OK, target};
}
void failNextAfter(uint32_t succeeding, uint32_t count, uint32_t srFlags, Target target) {
	injection = Injection{count, srFlags, HAL_OK, target, succeeding};
}
void failNextWithStatus(uint32_t count, HAL_StatusTypeDef status, Target target) {
	injection = Injection{count, 0, status, target};
}
void setPendingFlags(uint32_t srFlags) { hosttest_flash_regs.SR |= srFlags; }
void setUnlockFails(bool fails) { unlockFails = fails; }

void cutPowerAfterPrograms(uint32_t completedPrograms, uint64_t tornMask) {
	power.programArmed = true;
	power.programsLeft = completedPrograms;
	power.tornMask = tornMask;
}
void cutPowerDuringErase(uint32_t completedErases, uint32_t erasedOffset, uint32_t erasedLength) {
	power.eraseArmed = true;
	power.erasesLeft = completedErases;
	power.erasedOffset = erasedOffset;
	power.erasedLength = erasedLength;
}
bool powerLost() { return power.lost; }

} // namespace HostFlash

extern "C" {

void hosttest_flash_clear_flag(uint32_t flags) {
	// RM0090 3.9.5: EOP and the error flags are cleared by writing 1, BSY is read only
	hosttest_flash_regs.SR &= ~(flags & SR_W1C);
}

void hosttest_flash_data_cache_reset(void) {
	if (hosttest_flash_regs.ACR & FLASH_ACR_DCEN) {
		HostFlash::state().ignoredDataCacheResets++; // RM0090 3.9.1: DCRST only works while the data cache is disabled
	} else {
		HostFlash::state().dataCacheResets++;
	}
}

/** Models HAL_FLASH_Unlock (stm32f4xx_hal_flash.c): key sequence only if locked; HAL_ERROR if still locked */
HAL_StatusTypeDef HAL_FLASH_Unlock(void) {
	HostFlash::state().unlocks++;
	if (hosttest_flash_regs.CR & FLASH_CR_LOCK) {
		if (!unlockFails) {
			hosttest_flash_regs.CR &= ~FLASH_CR_LOCK; // KEY1, KEY2 written to FLASH_KEYR
		}
		if (hosttest_flash_regs.CR & FLASH_CR_LOCK) {
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

/** Models HAL_FLASH_Lock (stm32f4xx_hal_flash.c) */
HAL_StatusTypeDef HAL_FLASH_Lock(void) {
	HostFlash::state().locks++;
	hosttest_flash_regs.CR |= FLASH_CR_LOCK;
	return HAL_OK;
}

/** Models HAL_FLASH_GetError (stm32f4xx_hal_flash.c): pFlash.ErrorCode of the last operation */
uint32_t HAL_FLASH_GetError(void) { return errorCode; }

/**
 * Models HAL_FLASH_Program (stm32f4xx_hal_flash.c): wait for the last operation (pending error flags -> HAL_ERROR,
 * nothing written), program byte / half-word / word / double word (NOR: old & data), wait again (error flags
 * raised by the program operation -> HAL_ERROR with HAL_FLASH_GetError() set).
 */
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data) {
	HostFlash::State &st = HostFlash::state();
	st.programCalls++;
	HostFlash::Operation op{HostFlash::OpKind::Program, Address, TypeProgram, Data, HAL_ERROR, false};
	if (power.lost) {
		st.opsWhilePowerLost++;
		st.log.push_back(op);
		return HAL_ERROR;
	}
	HAL_StatusTypeDef status = waitForLastOperation();
	if (status == HAL_OK) {
		op.applied = programHardware(TypeProgram, Address, Data);
		status = waitForLastOperation();
		hosttest_flash_regs.CR &= ~FLASH_CR_PG;
	}
	op.result = status;
	st.log.push_back(op);
	return status;
}

/**
 * Models HAL_FLASHEx_Erase (stm32f4xx_hal_flash_ex.c): wait for the last operation; *SectorError = 0xFFFFFFFF;
 * erase sector by sector, stop at the first failing sector and report it in *SectorError; flush the caches.
 * Only sectors 0..3 are modelled; mass erase is recorded as unexpected.
 */
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError) {
	HostFlash::State &st = HostFlash::state();
	st.eraseCalls++;
	if (power.lost) {
		st.opsWhilePowerLost++;
		st.log.push_back(HostFlash::Operation{HostFlash::OpKind::Erase, 0, pEraseInit->Sector, 0, HAL_ERROR, false});
		return HAL_ERROR;
	}
	HAL_StatusTypeDef status = waitForLastOperation();
	if (status == HAL_OK) {
		*SectorError = 0xFFFFFFFFU;
		if (pEraseInit->TypeErase == FLASH_TYPEERASE_MASSERASE) {
			unexpected("HAL_FLASHEx_Erase mass erase (not modelled)");
			status = waitForLastOperation();
			st.log.push_back(HostFlash::Operation{HostFlash::OpKind::Erase, 0, 0xFFFFFFFFU, 0, status, false});
		} else {
			for (uint32_t index = pEraseInit->Sector; index < (pEraseInit->NbSectors + pEraseInit->Sector); index++) {
				HostFlash::Operation op{HostFlash::OpKind::Erase, 0, index, 0, HAL_ERROR, false};
				if (index < (uint32_t)HostFlash::SECTOR_COUNT) {
					op.address = HostFlash::sectorBase((int)index);
				}
				op.applied = eraseHardware(index);
				status = waitForLastOperation();
				hosttest_flash_regs.CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);
				op.result = status;
				st.log.push_back(op);
				if (status != HAL_OK) {
					*SectorError = index;
					break;
				}
			}
		}
		flushCaches();
	}
	return status;
}

} // extern "C"
