// P12 T7 group FE: HAL_FLASHEx_Erase / Erase_IT / OBProgram / OBGetConfig, FLASH_Erase_Sector, FLASH_FlushCaches and the option byte helpers
// of stm32f4xx_hal_flash_ex.c (F405/407 variants). Oracles: UM1725 Rev 8 chapters "HAL FLASH Extension Driver" and "HAL FLASH Generic Driver",
// RM0090 Rev 22 chapter 3 (CR / SR / OPTCR / ACR registers, sector layout, option bytes). Requirement ids: FE1 erase (poll), FE2 erase (IT),
// FE3 OBProgram, FE4 OBGetConfig (charter P12_T7_REQUIREMENTS.md).
// The advanced option byte functions (HAL_FLASHEx_AdvOBProgram / AdvOBGetConfig / OB_SelectPCROP ...) are compiled out on the F407 (preprocessor of
// stm32f4xx_hal_flash_ex.c: F42x/F43x/F401/F41x/F446/F469 only) and are therefore not tested.
#include "FlashTestSupport.h"

#include <cstring>
#include <vector>

extern "C" {
extern FLASH_ProcessTypeDef pFlash;
void FLASH_FlushCaches(void);
void FLASH_Erase_Sector(uint32_t Sector, uint8_t VoltageRange);
}

namespace {

const uint32_t feFlashBase = 0x08000000u;
const uint32_t feFlashSize = 0x100000u;
const uint32_t feOtpBase = 0x1FFF7800u;
const uint32_t feNone = 0xFFFFFFFFu;
const uint32_t feCrSer = 0x2u, feCrMer = 0x4u, feCrPg = 0x1u, feCrStrt = 0x10000u, feCrEopie = 0x1000000u, feCrErrie = 0x2000000u, feCrLock = 0x80000000u;
const uint32_t feCrSnbMask = 0x78u;
const uint32_t feSrEop = 0x1u, feSrWrperr = 0x10u, feSrBsy = 0x10000u, feSrPgserr = 0x80u;
const uint32_t feAcrIcen = 0x200u, feAcrDcen = 0x400u, feAcrIcrst = 0x800u, feAcrDcrst = 0x1000u;
const uint32_t feOptcrAfterUnlock = 0x0FFFAAECu; // reset value 0x0FFFAAED (RM0090 3.9.9) with OPTLOCK cleared by the key sequence

class FeFix : public FlashFixture {
public:
	FeFix() {
		std::memset(const_cast<FLASH_ProcessTypeDef *>(&pFlash), 0, sizeof pFlash);
		hostFlashSetPollReads(1);
	}
};

void feJump(void) { uwTick = uwTick + 60000u; } // each HAL_GetTick call moves 60 s: any wait with a finite timeout ends at once

void feUnlock() { REQUIRE(HAL_FLASH_Unlock() == HAL_OK); }
void feObUnlock() { REQUIRE(HAL_FLASH_OB_Unlock() == HAL_OK); }

uint32_t feReg(const volatile void *r) { return HalFixture::peek(r); }

void feFillAll() {
	hostFlashFill(feFlashBase, 0xA5u, feFlashSize);
	hostFlashFill(feOtpBase, 0x5Au, 0x20u);
}

// 0 = every byte of the sector is 0xA5 (untouched), 1 = every byte 0xFF (erased), 2 = anything else
unsigned feSectorClass(unsigned s) {
	const uint32_t base = hostFlashSectorBase(s), size = hostFlashSectorSize(s);
	bool allA5 = true, allFf = true;
	for (uint32_t o = 0; o < size; o += (o + 0x1000u < size || o + 4u == size) ? 0x1000u : (size - 4u - o)) { // first word, one word per 4 KB, last word
		const uint32_t w = hostRegsPeek(base + o);
		if (w != 0xA5A5A5A5u) allA5 = false;
		if (w != 0xFFFFFFFFu) allFf = false;
		if (!allA5 && !allFf) return 2u;
	}
	return allFf ? 1u : 0u;
}

// bit s set = sector s erased; bit 16 + s set = sector s neither untouched nor erased
uint32_t feSectorMask() {
	uint32_t m = 0;
	for (unsigned s = 0; s < 12u; ++s) {
		const unsigned c = feSectorClass(s);
		if (c == 1u) m |= 1u << s;
		if (c == 2u) m |= 1u << (16u + s);
	}
	return m;
}

bool feOtpUntouched() { return hostRegsPeek(feOtpBase) == 0x5A5A5A5Au && hostRegsPeek(feOtpBase + 0x1Cu) == 0x5A5A5A5Au; }

void feProtect(uint32_t sectorMask) { hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), feReg(&FLASH->OPTCR) & ~(sectorMask << 16)); }

FLASH_EraseInitTypeDef feSectors(uint32_t first, uint32_t nb, uint32_t range) {
	FLASH_EraseInitTypeDef e;
	std::memset(&e, 0, sizeof e);
	e.TypeErase = FLASH_TYPEERASE_SECTORS;
	e.Sector = first;
	e.NbSectors = nb;
	e.VoltageRange = range;
	return e;
}

FLASH_EraseInitTypeDef feMass(uint32_t range) {
	FLASH_EraseInitTypeDef e;
	std::memset(&e, 0, sizeof e);
	e.TypeErase = FLASH_TYPEERASE_MASSERASE;
	e.Banks = FLASH_BANK_1;
	e.VoltageRange = range;
	return e;
}

// raw words the CPU wrote to FLASH->CR with STRT set, in order
std::vector<uint32_t> feStrtWrites() {
	std::vector<uint32_t> v;
	for (std::size_t i = 0; i < hostRegsLogCount(); ++i) {
		const HostRegAccess *a = hostRegsLogAt(i);
		if (a->write && a->addr == reinterpret_cast<uintptr_t>(&FLASH->CR) && (a->written & feCrStrt) != 0u) v.push_back(a->written);
	}
	return v;
}

std::vector<uint32_t> feAcrWrites() {
	std::vector<uint32_t> v;
	for (std::size_t i = 0; i < hostRegsLogCount(); ++i) {
		const HostRegAccess *a = hostRegsLogAt(i);
		if (a->write && a->addr == reinterpret_cast<uintptr_t>(&FLASH->ACR)) v.push_back(a->written);
	}
	return v;
}

std::size_t feWriteCount() {
	std::size_t n = 0;
	for (std::size_t i = 0; i < hostRegsLogCount(); ++i)
		if (hostRegsLogAt(i)->write) ++n;
	return n;
}

// one hardware completion followed by the interrupt service routine
void feIrq() {
	hostFlashComplete();
	HAL_FLASH_IRQHandler();
}

FLASH_OBProgramInitTypeDef feOb() {
	FLASH_OBProgramInitTypeDef o;
	std::memset(&o, 0, sizeof o);
	return o;
}

const uint32_t feRanges[4] = {FLASH_VOLTAGE_RANGE_1, FLASH_VOLTAGE_RANGE_2, FLASH_VOLTAGE_RANGE_3, FLASH_VOLTAGE_RANGE_4};

} // namespace

// ====================================================================================== FE1: HAL_FLASHEx_Erase (polling)

TEST_CASE_FIXTURE(FeFix, "FE1 a single sector erase turns exactly that sector to 0xFF for every sector 0 to 11") {
	for (unsigned s = 0; s < 12u; ++s) {
		CAPTURE(s);
		feFillAll();
		feUnlock();
		FLASH_EraseInitTypeDef e = feSectors(s, 1, FLASH_VOLTAGE_RANGE_3);
		uint32_t err = 0;
		CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
		CHECK(feSectorMask() == (1u << s)); // RM0090 Table 5: the sector alone is erased
		CHECK(feOtpUntouched());
	}
}

TEST_CASE_FIXTURE(FeFix, "FE1 the sector layout erased is 4 x 16 KB, 64 KB, 7 x 128 KB with the first and last word of each sector erased") {
	// RM0090 Table 5 (F405/407): sector 0 0x08000000, 1 0x08004000, 2 0x08008000, 3 0x0800C000, 4 0x08010000, 5 0x08020000 ... 11 0x080E0000.
	const uint32_t base[12] = {0x08000000u, 0x08004000u, 0x08008000u, 0x0800C000u, 0x08010000u, 0x08020000u,
	                           0x08040000u, 0x08060000u, 0x08080000u, 0x080A0000u, 0x080C0000u, 0x080E0000u};
	const uint32_t size[12] = {0x4000u, 0x4000u, 0x4000u, 0x4000u, 0x10000u, 0x20000u, 0x20000u, 0x20000u, 0x20000u, 0x20000u, 0x20000u, 0x20000u};
	for (unsigned s = 0; s < 12u; ++s) {
		CAPTURE(s);
		feFillAll();
		feUnlock();
		FLASH_EraseInitTypeDef e = feSectors(s, 1, FLASH_VOLTAGE_RANGE_3);
		uint32_t err = 0;
		REQUIRE(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
		CHECK(hostRegsPeek(base[s]) == 0xFFFFFFFFu);
		CHECK(hostRegsPeek(base[s] + size[s] - 4u) == 0xFFFFFFFFu);
		if (s > 0) CHECK(hostRegsPeek(base[s] - 4u) == 0xA5A5A5A5u);
		if (s < 11u) CHECK(hostRegsPeek(base[s] + size[s]) == 0xA5A5A5A5u);
	}
}

TEST_CASE_FIXTURE(FeFix, "FE1 a range of sectors erases exactly NbSectors sectors from the first one and SectorError is 0xFFFFFFFF") {
	feFillAll();
	feUnlock();
	FLASH_EraseInitTypeDef e = feSectors(3, 4, FLASH_VOLTAGE_RANGE_3); // sectors 3, 4, 5, 6
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	CHECK(err == feNone);
	CHECK(feSectorMask() == 0x78u);
}

TEST_CASE_FIXTURE(FeFix, "FE1 a range that ends at sector 11 is erased completely") {
	feFillAll();
	feUnlock();
	FLASH_EraseInitTypeDef e = feSectors(10, 2, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	CHECK(err == feNone);
	CHECK(feSectorMask() == 0xC00u);
}

TEST_CASE_FIXTURE(FeFix, "FE1 erasing all twelve sectors one by one in a range leaves OTP untouched and no sector half erased") {
	feFillAll();
	feUnlock();
	FLASH_EraseInitTypeDef e = feSectors(0, 12, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	CHECK(feSectorMask() == 0xFFFu);
	CHECK(feOtpUntouched());
}

TEST_CASE_FIXTURE(FeFix, "FE1 programmed bits of an erased sector are 1 again so it can be programmed after the erase") {
	feUnlock();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08008000u, 0x12345678u) == HAL_OK);
	FLASH_EraseInitTypeDef e = feSectors(2, 1, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	REQUIRE(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	CHECK(hostRegsPeek(0x08008000u) == 0xFFFFFFFFu);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08008000u, 0xA5A5A5A5u) == HAL_OK);
	CHECK(hostRegsPeek(0x08008000u) == 0xA5A5A5A5u);
}

TEST_CASE_FIXTURE(FeFix, "FE1 the erase of sector 5 after a real unlock writes SER, SNB 5, PSIZE x32 and STRT") {
	// RM0090 3.9.5 CR: SER bit 1, SNB bits 6:3, PSIZE bits 9:8 (00 x8, 01 x16, 10 x32, 11 x64), STRT bit 16. UM1725: RANGE_3 is x32.
	feUnlock();
	hostRegsLogClear();
	FLASH_EraseInitTypeDef e = feSectors(5, 1, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	REQUIRE(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	const std::vector<uint32_t> w = feStrtWrites();
	REQUIRE(w.size() == 1u);
	CHECK((w[0] & feCrSer) == feCrSer);
	CHECK((w[0] & feCrMer) == 0u);
	CHECK((w[0] & feCrPg) == 0u);
	CHECK(((w[0] >> 3) & 0xFu) == 5u);
	CHECK(((w[0] >> 8) & 3u) == 2u);
}

TEST_CASE_FIXTURE(FeFix, "FE1 the erase of sector 7 with each voltage range writes PSIZE 0, 1, 2, 3 and SNB 7") {
	for (unsigned r = 0; r < 4u; ++r) {
		CAPTURE(r);
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->CR), 0u); // unlocked, the key sequence itself is FB1's business
		hostRegsLogClear();
		FLASH_EraseInitTypeDef e = feSectors(7, 1, feRanges[r]);
		uint32_t err = 0;
		REQUIRE(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
		const std::vector<uint32_t> w = feStrtWrites();
		REQUIRE(w.size() == 1u);
		CHECK(((w[0] >> 3) & 0xFu) == 7u);
		CHECK(((w[0] >> 8) & 3u) == r);
		CHECK((w[0] & feCrSer) == feCrSer);
	}
}

TEST_CASE_FIXTURE(FeFix, "FE1 a range of three sectors starts three erases with SNB 4, 5, 6 in order") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->CR), 0u);
	hostRegsLogClear();
	FLASH_EraseInitTypeDef e = feSectors(4, 3, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	REQUIRE(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	const std::vector<uint32_t> w = feStrtWrites();
	REQUIRE(w.size() == 3u);
	for (unsigned i = 0; i < 3u; ++i) CHECK(((w[i] >> 3) & 0xFu) == 4u + i);
}

TEST_CASE_FIXTURE(FeFix, "FE1 after a sector erase SER, SNB, STRT and PG are cleared and the flags are clean") {
	feUnlock();
	FLASH_EraseInitTypeDef e = feSectors(6, 2, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	REQUIRE(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	const uint32_t cr = feReg(&FLASH->CR);
	CHECK((cr & feCrSer) == 0u);
	CHECK((cr & feCrSnbMask) == 0u);
	CHECK((cr & feCrStrt) == 0u);
	CHECK((cr & feCrPg) == 0u);
	CHECK((cr & feCrLock) == 0u); // the HAL does not lock again
	CHECK(feReg(&FLASH->SR) == 0u); // BSY ended, EOP cleared by the wait, no error flag
	CHECK(hostFlashStalledWrites() == 0u);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_NONE);
}

TEST_CASE_FIXTURE(FeFix, "FE1 the erase process lock is released after a successful erase and the state machine stays NONE") {
	feUnlock();
	FLASH_EraseInitTypeDef e = feSectors(1, 1, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	REQUIRE(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	CHECK(pFlash.Lock == HAL_UNLOCKED);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
}

TEST_CASE_FIXTURE(FeFix, "FE1 an erase while the process is locked returns HAL_BUSY and touches nothing") {
	feFillAll();
	feUnlock();
	pFlash.Lock = HAL_LOCKED;
	hostRegsLogClear();
	FLASH_EraseInitTypeDef e = feSectors(2, 1, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0x1234u;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_BUSY); // UM1725: HAL_BUSY is the HAL status of a locked process
	CHECK(err == 0x1234u);
	CHECK(feSectorMask() == 0u);
	CHECK(feWriteCount() == 0u);
	CHECK(pFlash.Lock == HAL_LOCKED);
	pFlash.Lock = HAL_UNLOCKED;
}

TEST_CASE_FIXTURE(FeFix, "FE1 a sector erase of a write protected sector returns HAL_ERROR, reports that sector and erases nothing") {
	feFillAll();
	feUnlock();
	feProtect(1u << 3);
	FLASH_EraseInitTypeDef e = feSectors(3, 1, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_ERROR);
	CHECK(err == 3u);
	CHECK(feSectorMask() == 0u);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP); // RM0090 3.9.4 WRPERR
	CHECK(feReg(&FLASH->SR) == 0u); // the HAL cleared WRPERR
	CHECK(pFlash.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(FeFix, "FE1 an erase range stops at the first write protected sector, reports it, keeps the earlier ones erased and the later ones untouched") {
	feFillAll();
	feUnlock();
	feProtect(1u << 6);
	FLASH_EraseInitTypeDef e = feSectors(4, 5, FLASH_VOLTAGE_RANGE_3); // 4, 5, 6 (protected), 7, 8
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_ERROR);
	CHECK(err == 6u);
	CHECK(feSectorMask() == 0x30u); // sectors 4 and 5 only
	CHECK((feReg(&FLASH->CR) & (feCrSer | feCrSnbMask)) == 0u);
}

TEST_CASE_FIXTURE(FeFix, "FE1 each write protected sector of the range is reported as the failing sector") {
	for (unsigned p = 0; p < 12u; ++p) {
		CAPTURE(p);
		feFillAll();
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->CR), 0u);
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), 0x0FFFAAECu);
		feProtect(1u << p);
		FLASH_EraseInitTypeDef e = feSectors(0, 12, FLASH_VOLTAGE_RANGE_3);
		uint32_t err = 0;
		CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_ERROR);
		CHECK(err == p);
		CHECK(feSectorMask() == ((1u << p) - 1u));
	}
}

TEST_CASE_FIXTURE(FeFix, "FE1 an erase sets no SectorError for the next call: a good call after a failed one reports 0xFFFFFFFF again") {
	feFillAll();
	feUnlock();
	feProtect(1u << 1);
	FLASH_EraseInitTypeDef e1 = feSectors(1, 1, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	REQUIRE(HAL_FLASHEx_Erase(&e1, &err) == HAL_ERROR);
	REQUIRE(err == 1u);
	FLASH_EraseInitTypeDef e2 = feSectors(2, 1, FLASH_VOLTAGE_RANGE_3);
	CHECK(HAL_FLASHEx_Erase(&e2, &err) == HAL_OK);
	CHECK(err == feNone);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_NONE); // the wait clears the error code of the previous call
	CHECK(feSectorMask() == 0x4u);
}

TEST_CASE_FIXTURE(FeFix, "FE1 a sector erase that never completes returns HAL_TIMEOUT and reports the sector it started") {
	feFillAll();
	feUnlock();
	hostFlashSetPollReads(0);
	hostTickSetStep(feJump);
	FLASH_EraseInitTypeDef e = feSectors(4, 2, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_TIMEOUT);
	CHECK(err == 4u);
	CHECK(pFlash.Lock == HAL_UNLOCKED);
	CHECK((feReg(&FLASH->SR) & feSrBsy) != 0u); // the operation still runs: the HAL gave up
}

TEST_CASE_FIXTURE(FeFix, "FE1 a mass erase that never completes returns HAL_TIMEOUT and leaves SectorError at 0xFFFFFFFF") {
	feUnlock();
	hostFlashSetPollReads(0);
	hostTickSetStep(feJump);
	FLASH_EraseInitTypeDef e = feMass(FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_TIMEOUT);
	CHECK(err == feNone);
	CHECK(pFlash.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(FeFix, "FE1 an erase while the flash is busy from an earlier operation returns HAL_TIMEOUT and starts nothing") {
	feFillAll();
	feUnlock();
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->SR), feSrBsy); // BSY with no operation behind it: never clears
	hostTickSetStep(feJump);
	hostRegsLogClear();
	FLASH_EraseInitTypeDef e = feSectors(2, 1, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_TIMEOUT);
	CHECK(feStrtWrites().empty());
	CHECK(feSectorMask() == 0u);
	CHECK(pFlash.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(FeFix, "FE1 an erase of a flash with a pending error flag returns HAL_ERROR with that error code and erases nothing") {
	feFillAll();
	feUnlock();
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->SR), feSrPgserr);
	FLASH_EraseInitTypeDef e = feSectors(2, 1, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGS);
	CHECK(feSectorMask() == 0u);
	CHECK(pFlash.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(FeFix, "FE1 characterization: an erase of a locked flash returns HAL_OK and changes nothing") {
	// NEEDS REVIEW 343/1: UM1725 says the flash must be unlocked first but defines no status for a locked one; RM0090 3.6.1: writes to CR are ignored while LOCK = 1.
	feFillAll();
	FLASH_EraseInitTypeDef e = feSectors(2, 1, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	CHECK(err == feNone);
	CHECK(feSectorMask() == 0u);
	CHECK(hostFlashLockedWrites() > 0u);
}

TEST_CASE_FIXTURE(FeFix, "FE1 characterization: NbSectors 0 erases nothing and returns HAL_OK with SectorError 0xFFFFFFFF") {
	// NEEDS REVIEW 343/2: UM1725 requires NbSectors between 1 and max; the value 0 is outside that range and defines no status (the parameter assert is off in the firmware).
	feFillAll();
	feUnlock();
	FLASH_EraseInitTypeDef e = feSectors(2, 0, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	CHECK(err == feNone);
	CHECK(feSectorMask() == 0u);
}

TEST_CASE_FIXTURE(FeFix, "FE1 characterization: a range running past sector 11 erases the valid sectors and returns HAL_OK") {
	// NEEDS REVIEW 343/3: UM1725 requires Sector + NbSectors <= 12; beyond that no status is defined (the parameter assert is off). RM0090 CR: SNB 12-15 are not defined sectors.
	feFillAll();
	feUnlock();
	FLASH_EraseInitTypeDef e = feSectors(11, 3, FLASH_VOLTAGE_RANGE_3); // 11, 12, 13
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	CHECK(err == feNone);
	CHECK(feSectorMask() == 0x800u);
}

// ---- mass erase

TEST_CASE_FIXTURE(FeFix, "FE1 a mass erase turns all main flash to 0xFF, leaves OTP alone and reports 0xFFFFFFFF") {
	feFillAll();
	feUnlock();
	FLASH_EraseInitTypeDef e = feMass(FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	CHECK(err == feNone);
	CHECK(feSectorMask() == 0xFFFu);
	CHECK(feOtpUntouched()); // RM0090 3.8: OTP can not be erased
}

TEST_CASE_FIXTURE(FeFix, "FE1 a mass erase writes MER and STRT with PSIZE of the voltage range and no SNB") {
	for (unsigned r = 0; r < 4u; ++r) {
		CAPTURE(r);
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->CR), 0u);
		hostRegsLogClear();
		FLASH_EraseInitTypeDef e = feMass(feRanges[r]);
		uint32_t err = 0;
		REQUIRE(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
		const std::vector<uint32_t> w = feStrtWrites();
		REQUIRE(w.size() == 1u);
		CHECK((w[0] & feCrMer) == feCrMer); // RM0090 CR MER bit 2
		CHECK((w[0] & feCrSer) == 0u);
		CHECK(((w[0] >> 8) & 3u) == r);
	}
}

TEST_CASE_FIXTURE(FeFix, "FE1 after a mass erase MER, STRT, SER and SNB are cleared") {
	feUnlock();
	FLASH_EraseInitTypeDef e = feMass(FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	REQUIRE(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	const uint32_t cr = feReg(&FLASH->CR);
	CHECK((cr & (feCrMer | feCrStrt | feCrSer | feCrSnbMask)) == 0u);
	CHECK(feReg(&FLASH->SR) == 0u);
	CHECK(pFlash.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(FeFix, "FE1 the Banks field of a mass erase does not change what the F407 erases") {
	for (uint32_t banks = 0; banks < 4u; ++banks) {
		CAPTURE(banks);
		feFillAll();
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->CR), 0u);
		FLASH_EraseInitTypeDef e = feMass(FLASH_VOLTAGE_RANGE_3);
		e.Banks = banks;
		uint32_t err = 0;
		CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
		CHECK(feSectorMask() == 0xFFFu); // a single bank device: UM1725 FLASHEx_Banks lists FLASH_BANK_1 only
	}
}

TEST_CASE_FIXTURE(FeFix, "FE1 a mass erase with one write protected sector returns HAL_ERROR with the WRP error code and erases nothing") {
	feFillAll();
	feUnlock();
	feProtect(1u << 9);
	FLASH_EraseInitTypeDef e = feMass(FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP);
	CHECK(feSectorMask() == 0u);
	CHECK((feReg(&FLASH->CR) & feCrMer) == 0u); // the HAL clears MER also after the failure
	CHECK(pFlash.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(FeFix, "FE1 a mass erase while the process is locked returns HAL_BUSY") {
	feFillAll();
	feUnlock();
	pFlash.Lock = HAL_LOCKED;
	FLASH_EraseInitTypeDef e = feMass(FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_BUSY);
	CHECK(feSectorMask() == 0u);
	pFlash.Lock = HAL_UNLOCKED;
}

TEST_CASE_FIXTURE(FeFix, "FE1 characterization: a mass erase of a locked flash returns HAL_OK and erases nothing") {
	// NEEDS REVIEW 343/4: same as the sector erase of a locked flash (NEEDS REVIEW 343/1).
	feFillAll();
	FLASH_EraseInitTypeDef e = feMass(FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	CHECK(feSectorMask() == 0u);
}

// ---- FLASH_Erase_Sector (called directly)

TEST_CASE_FIXTURE(FeFix, "FE1 FLASH_Erase_Sector starts the erase: SER, SNB, PSIZE and STRT are set and the flash is busy") {
	for (unsigned r = 0; r < 4u; ++r) {
		CAPTURE(r);
		feFillAll();
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->CR), 0u);
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->SR), 0u);
		hostFlashSetPollReads(0);
		FLASH_Erase_Sector(9, static_cast<uint8_t>(feRanges[r]));
		const uint32_t cr = feReg(&FLASH->CR);
		CHECK((cr & feCrSer) == feCrSer);
		CHECK(((cr >> 3) & 0xFu) == 9u);
		CHECK(((cr >> 8) & 3u) == r);
		CHECK((cr & feCrStrt) == feCrStrt);
		CHECK((feReg(&FLASH->SR) & feSrBsy) != 0u);
		CHECK(feSectorMask() == 0x200u);
		CHECK(hostFlashComplete() == 1);
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->SR), 0u);
	}
}

TEST_CASE_FIXTURE(FeFix, "FE1 FLASH_Erase_Sector replaces an older SNB and PSIZE instead of or-ing them") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->CR), 0xFu << 3 | 0x300u); // SNB 15, PSIZE 3 left over
	FLASH_Erase_Sector(2, FLASH_VOLTAGE_RANGE_1);
	const uint32_t cr = feReg(&FLASH->CR);
	CHECK(((cr >> 3) & 0xFu) == 2u);
	CHECK(((cr >> 8) & 3u) == 0u);
}

// ---- cache flush

TEST_CASE_FIXTURE(FeFix, "FE1 FLASH_FlushCaches with both caches off does not write ACR") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->ACR), 5u); // latency only
	hostRegsLogClear();
	FLASH_FlushCaches();
	CHECK(feAcrWrites().empty());
	CHECK(feReg(&FLASH->ACR) == 5u);
}

TEST_CASE_FIXTURE(FeFix, "FE1 FLASH_FlushCaches resets the instruction cache with ICEN off during the reset and ICEN on afterwards") {
	// RM0090 3.9.1 ACR: ICRST can be written only when the instruction cache is disabled (ICEN = 0).
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->ACR), feAcrIcen | 5u);
	hostRegsLogClear();
	FLASH_FlushCaches();
	const std::vector<uint32_t> w = feAcrWrites();
	bool resetSeen = false;
	for (std::size_t i = 0; i < w.size(); ++i) {
		CHECK((w[i] & (feAcrDcen | feAcrDcrst)) == 0u); // the data cache is never touched
		if ((w[i] & feAcrIcrst) != 0u && !resetSeen) {
			resetSeen = true;
			CHECK((w[i] & feAcrIcen) == 0u);
		}
	}
	CHECK(resetSeen);
	CHECK((feReg(&FLASH->ACR) & feAcrIcen) == feAcrIcen);
	CHECK((feReg(&FLASH->ACR) & 7u) == 5u); // latency kept
}

TEST_CASE_FIXTURE(FeFix, "FE1 FLASH_FlushCaches resets the data cache with DCEN off during the reset and DCEN on afterwards") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->ACR), feAcrDcen | 3u);
	hostRegsLogClear();
	FLASH_FlushCaches();
	const std::vector<uint32_t> w = feAcrWrites();
	bool resetSeen = false;
	for (std::size_t i = 0; i < w.size(); ++i) {
		CHECK((w[i] & (feAcrIcen | feAcrIcrst)) == 0u);
		if ((w[i] & feAcrDcrst) != 0u && !resetSeen) {
			resetSeen = true;
			CHECK((w[i] & feAcrDcen) == 0u);
		}
	}
	CHECK(resetSeen);
	CHECK((feReg(&FLASH->ACR) & feAcrDcen) == feAcrDcen);
	CHECK((feReg(&FLASH->ACR) & 7u) == 3u);
}

TEST_CASE_FIXTURE(FeFix, "FE1 FLASH_FlushCaches resets both caches and leaves both enabled") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->ACR), feAcrIcen | feAcrDcen | 7u);
	hostRegsLogClear();
	FLASH_FlushCaches();
	const std::vector<uint32_t> w = feAcrWrites();
	bool ic = false, dc = false;
	for (std::size_t i = 0; i < w.size(); ++i) {
		if ((w[i] & feAcrIcrst) != 0u && !ic) {
			ic = true;
			CHECK((w[i] & feAcrIcen) == 0u);
		}
		if ((w[i] & feAcrDcrst) != 0u && !dc) {
			dc = true;
			CHECK((w[i] & feAcrDcen) == 0u);
		}
	}
	CHECK(ic);
	CHECK(dc);
	CHECK((feReg(&FLASH->ACR) & (feAcrIcen | feAcrDcen)) == (feAcrIcen | feAcrDcen));
	CHECK((feReg(&FLASH->ACR) & 7u) == 7u);
}

TEST_CASE_FIXTURE(FeFix, "FE1 FLASH_FlushCaches pulses ICRST and DCRST: they are set and cleared again and read 0 afterwards") {
	// RM0090 3.9.1 ACR: ICRST / DCRST are reset requests; the HAL sets and clears them again (a pulse) while the cache is disabled.
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->ACR), feAcrIcen | feAcrDcen);
	hostRegsLogClear();
	FLASH_FlushCaches();
	CHECK((feReg(&FLASH->ACR) & (feAcrIcrst | feAcrDcrst)) == 0u);
	bool icSet = false, icCleared = false, dcSet = false, dcCleared = false;
	for (uint32_t w : feAcrWrites()) {
		if ((w & feAcrIcrst) != 0u) icSet = true;
		else if (icSet) icCleared = true;
		if ((w & feAcrDcrst) != 0u) dcSet = true;
		else if (dcSet) dcCleared = true;
	}
	CHECK(icSet);
	CHECK(icCleared);
	CHECK(dcSet);
	CHECK(dcCleared);
}

TEST_CASE_FIXTURE(FeFix, "FE1 a successful erase flushes an enabled instruction cache") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->ACR), feAcrIcen);
	feUnlock();
	hostRegsLogClear();
	FLASH_EraseInitTypeDef e = feSectors(2, 1, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	REQUIRE(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	bool reset = false;
	for (uint32_t w : feAcrWrites()) reset = reset || (w & feAcrIcrst) != 0u;
	CHECK(reset);
	CHECK((feReg(&FLASH->ACR) & feAcrIcen) == feAcrIcen);
}

TEST_CASE_FIXTURE(FeFix, "FE1 a successful mass erase flushes an enabled data cache") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->ACR), feAcrDcen);
	feUnlock();
	hostRegsLogClear();
	FLASH_EraseInitTypeDef e = feMass(FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	REQUIRE(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	bool reset = false;
	for (uint32_t w : feAcrWrites()) reset = reset || (w & feAcrDcrst) != 0u;
	CHECK(reset);
	CHECK((feReg(&FLASH->ACR) & feAcrDcen) == feAcrDcen);
}

TEST_CASE_FIXTURE(FeFix, "FE1 an erase that fails to start (flash busy) does not flush the caches") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->ACR), feAcrIcen | feAcrDcen);
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->SR), feSrBsy);
	feUnlock();
	hostTickSetStep(feJump);
	hostRegsLogClear();
	FLASH_EraseInitTypeDef e = feSectors(2, 1, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_TIMEOUT);
	CHECK(feAcrWrites().empty());
}

// ====================================================================================== FE2: HAL_FLASHEx_Erase_IT

TEST_CASE_FIXTURE(FeFix, "FE2 Erase_IT of a sector range enables EOPIE and ERRIE, starts the first sector and returns HAL_OK") {
	feFillAll();
	feUnlock();
	hostFlashSetPollReads(0);
	FLASH_EraseInitTypeDef e = feSectors(3, 2, FLASH_VOLTAGE_RANGE_3);
	CHECK(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	const uint32_t cr = feReg(&FLASH->CR);
	CHECK((cr & feCrEopie) == feCrEopie);
	CHECK((cr & feCrErrie) == feCrErrie);
	CHECK((cr & feCrSer) == feCrSer);
	CHECK(((cr >> 3) & 0xFu) == 3u);
	CHECK((cr & feCrStrt) == feCrStrt);
	CHECK((feReg(&FLASH->SR) & feSrBsy) != 0u);
	CHECK(feSectorMask() == 0x8u); // the first sector only (the HAL starts the next one from the interrupt)
	CHECK(hostFlashIrqPending() == 0u);
}

TEST_CASE_FIXTURE(FeFix, "FE2 Erase_IT saves the procedure, the number of sectors, the first sector and the voltage range for the interrupt") {
	feUnlock();
	hostFlashSetPollReads(0);
	FLASH_EraseInitTypeDef e = feSectors(5, 4, FLASH_VOLTAGE_RANGE_2);
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_SECTERASE);
	CHECK(pFlash.NbSectorsToErase == 4u);
	CHECK(pFlash.Sector == 5u);
	CHECK(pFlash.VoltageForErase == FLASH_VOLTAGE_RANGE_2);
}

TEST_CASE_FIXTURE(FeFix, "FE2 Erase_IT clears pending flags before it starts") {
	feUnlock();
	hostFlashSetPollReads(0);
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->SR), 0xF3u); // EOP, OPERR, WRPERR, PGAERR, PGPERR, PGSERR
	FLASH_EraseInitTypeDef e = feSectors(1, 1, FLASH_VOLTAGE_RANGE_3);
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	CHECK((feReg(&FLASH->SR) & 0xF3u) == 0u);
}

TEST_CASE_FIXTURE(FeFix, "FE2 Erase_IT for one sector calls the end callback once with 0xFFFFFFFF and returns the state machine to NONE") {
	feFillAll();
	feUnlock();
	hostFlashSetPollReads(0);
	FLASH_EraseInitTypeDef e = feSectors(2, 1, FLASH_VOLTAGE_RANGE_3);
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	CHECK(hostFlashComplete() == 1);
	CHECK(hostFlashIrqPending() == 1u);
	HAL_FLASH_IRQHandler();
	CHECK(flashCounts.eop == 1u);
	CHECK(flashCounts.error == 0u);
	CHECK(flashCounts.lastValue == feNone); // UM1725: 0xFFFFFFFF means all the selected sectors have been erased
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
	CHECK(feSectorMask() == 0x4u);
}

TEST_CASE_FIXTURE(FeFix, "FE2 after the last sector the handler clears EOP, the interrupt enables, SER, SNB and the interrupt request") {
	feUnlock();
	hostFlashSetPollReads(0);
	FLASH_EraseInitTypeDef e = feSectors(2, 1, FLASH_VOLTAGE_RANGE_3);
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	feIrq();
	const uint32_t cr = feReg(&FLASH->CR);
	CHECK((cr & (feCrEopie | feCrErrie | feCrSer | feCrSnbMask | feCrPg | feCrMer)) == 0u);
	CHECK((feReg(&FLASH->SR) & feSrEop) == 0u);
	CHECK(hostFlashIrqPending() == 0u);
}

TEST_CASE_FIXTURE(FeFix, "FE2 Erase_IT of three sectors reports sector 2, sector 3 and then 0xFFFFFFFF, one per interrupt") {
	feFillAll();
	feUnlock();
	hostFlashSetPollReads(0);
	FLASH_EraseInitTypeDef e = feSectors(2, 3, FLASH_VOLTAGE_RANGE_3);
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	feIrq();
	REQUIRE(flashCounts.eop == 1u);
	CHECK(flashCounts.valueOrder[0] == 2u); // UM1725: Sectors Erase: the sector which has been erased
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_SECTERASE);
	CHECK(pFlash.Sector == 3u);
	CHECK(pFlash.NbSectorsToErase == 2u);
	CHECK(((feReg(&FLASH->CR) >> 3) & 0xFu) == 3u); // the handler started sector 3
	CHECK((feReg(&FLASH->CR) & (feCrEopie | feCrErrie)) == (feCrEopie | feCrErrie)); // the interrupts stay on in between
	feIrq();
	REQUIRE(flashCounts.eop == 2u);
	CHECK(flashCounts.valueOrder[1] == 3u);
	CHECK(pFlash.Sector == 4u);
	CHECK(pFlash.NbSectorsToErase == 1u);
	feIrq();
	REQUIRE(flashCounts.eop == 3u);
	CHECK(flashCounts.valueOrder[2] == feNone);
	CHECK(flashCounts.error == 0u);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
	CHECK(feSectorMask() == 0x1Cu);
}

TEST_CASE_FIXTURE(FeFix, "FE2 each follow-up sector of an Erase_IT range starts with the saved voltage range") {
	for (unsigned r = 0; r < 4u; ++r) {
		CAPTURE(r);
		flashSupportReset();
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->CR), 0u);
		hostFlashSetPollReads(0);
		hostRegsLogClear();
		FLASH_EraseInitTypeDef e = feSectors(0, 2, feRanges[r]);
		REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
		feIrq();
		feIrq();
		const std::vector<uint32_t> w = feStrtWrites();
		REQUIRE(w.size() == 2u);
		CHECK(((w[0] >> 8) & 3u) == r);
		CHECK(((w[1] >> 8) & 3u) == r);
		CHECK(((w[0] >> 3) & 0xFu) == 0u);
		CHECK(((w[1] >> 3) & 0xFu) == 1u);
	}
}

TEST_CASE_FIXTURE(FeFix, "FE2 the end callback of the last sector runs after the caches were flushed") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->ACR), feAcrIcen);
	feUnlock();
	hostFlashSetPollReads(0);
	FLASH_EraseInitTypeDef e = feSectors(2, 1, FLASH_VOLTAGE_RANGE_3);
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	hostRegsLogClear();
	feIrq();
	bool reset = false;
	for (uint32_t w : feAcrWrites()) reset = reset || (w & feAcrIcrst) != 0u;
	CHECK(reset);
	CHECK(flashCounts.eop == 1u);
}

TEST_CASE_FIXTURE(FeFix, "FE2 an interrupt without a finished operation calls no callback") {
	feUnlock();
	hostFlashSetPollReads(0);
	FLASH_EraseInitTypeDef e = feSectors(2, 1, FLASH_VOLTAGE_RANGE_3);
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	HAL_FLASH_IRQHandler(); // BSY still set, no flag
	CHECK(flashCounts.eop == 0u);
	CHECK(flashCounts.error == 0u);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_SECTERASE);
}

TEST_CASE_FIXTURE(FeFix, "FE2 Erase_IT mass erase sets MER with STRT, saves the bank and erases all main flash") {
	feFillAll();
	feUnlock();
	hostFlashSetPollReads(0);
	FLASH_EraseInitTypeDef e = feMass(FLASH_VOLTAGE_RANGE_3);
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	const uint32_t cr = feReg(&FLASH->CR);
	CHECK((cr & feCrMer) == feCrMer);
	CHECK((cr & feCrStrt) == feCrStrt);
	CHECK((cr & (feCrEopie | feCrErrie)) == (feCrEopie | feCrErrie));
	CHECK(((cr >> 8) & 3u) == 2u);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_MASSERASE);
	CHECK(pFlash.Bank == FLASH_BANK_1);
	CHECK(feSectorMask() == 0xFFFu);
	CHECK(feOtpUntouched());
}

TEST_CASE_FIXTURE(FeFix, "FE2 mass erase end callback returns the requested bank and the state returns to NONE") {
	feUnlock();
	hostFlashSetPollReads(0);
	FLASH_EraseInitTypeDef e = feMass(FLASH_VOLTAGE_RANGE_3);
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	feIrq();
	REQUIRE(flashCounts.eop == 1u);
	CHECK(flashCounts.lastValue == FLASH_BANK_1); // UM1725: Mass Erase: the bank number which has been requested to erase
	CHECK(flashCounts.error == 0u);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
	CHECK((feReg(&FLASH->CR) & (feCrMer | feCrEopie | feCrErrie | feCrStrt)) == 0u);
}

TEST_CASE_FIXTURE(FeFix, "FE2 mass erase in IT mode writes PSIZE of the voltage range") {
	for (unsigned r = 0; r < 4u; ++r) {
		CAPTURE(r);
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->CR), 0u);
		hostFlashSetPollReads(0);
		hostRegsLogClear();
		FLASH_EraseInitTypeDef e = feMass(feRanges[r]);
		REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
		const std::vector<uint32_t> w = feStrtWrites();
		REQUIRE(w.size() == 1u);
		CHECK(((w[0] >> 8) & 3u) == r);
		CHECK((w[0] & feCrMer) == feCrMer);
		feIrq();
	}
}

TEST_CASE_FIXTURE(FeFix, "FE2 Erase_IT of a write protected sector raises the error interrupt and the error callback gets the sector") {
	feFillAll();
	feUnlock();
	feProtect(1u << 3);
	FLASH_EraseInitTypeDef e = feSectors(3, 2, FLASH_VOLTAGE_RANGE_3);
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	CHECK(hostFlashIrqPending() == 1u); // ERRIE and WRPERR
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.error == 1u);
	CHECK(flashCounts.eop == 0u);
	CHECK(flashCounts.lastValue == 3u); // UM1725: Sectors Erase: the sector number which returned an error
	CHECK((flashCounts.errorCodeOrder[0] & HAL_FLASH_ERROR_WRP) == HAL_FLASH_ERROR_WRP);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
	CHECK(feSectorMask() == 0u);
	CHECK((feReg(&FLASH->SR) & 0xF3u) == 0u);
	CHECK((feReg(&FLASH->CR) & (feCrEopie | feCrErrie | feCrSer | feCrSnbMask)) == 0u);
}

TEST_CASE_FIXTURE(FeFix, "FE2 an error in the middle of an Erase_IT range reports that sector after the earlier sectors were reported") {
	feFillAll();
	feUnlock();
	feProtect(1u << 2);
	hostFlashSetPollReads(0);
	FLASH_EraseInitTypeDef e = feSectors(1, 3, FLASH_VOLTAGE_RANGE_3); // 1, 2 (protected), 3
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	feIrq(); // sector 1 done, the handler starts sector 2 which fails
	REQUIRE(flashCounts.eop == 1u);
	CHECK(flashCounts.valueOrder[0] == 1u);
	CHECK(hostFlashIrqPending() == 1u);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.error == 1u);
	CHECK(flashCounts.lastValue == 2u);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
	CHECK(feSectorMask() == 0x2u); // sector 1 erased, 2 and 3 untouched
	CHECK(flashCounts.order[0] == FLASH_CB_EOP);
	CHECK(flashCounts.order[1] == FLASH_CB_ERROR);
}

TEST_CASE_FIXTURE(FeFix, "FE2 a failed mass erase in IT mode reports the bank to the error callback") {
	feFillAll();
	feUnlock();
	feProtect(1u << 0);
	FLASH_EraseInitTypeDef e = feMass(FLASH_VOLTAGE_RANGE_3);
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	CHECK(hostFlashIrqPending() == 1u);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.error == 1u);
	CHECK(flashCounts.lastValue == FLASH_BANK_1);
	CHECK((flashCounts.errorCodeOrder[0] & HAL_FLASH_ERROR_WRP) == HAL_FLASH_ERROR_WRP);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
	CHECK(feSectorMask() == 0u);
	CHECK((feReg(&FLASH->CR) & (feCrMer | feCrEopie | feCrErrie)) == 0u);
}

TEST_CASE_FIXTURE(FeFix, "FE2 a second Erase_IT after a finished one starts again from its own parameters") {
	feFillAll();
	feUnlock();
	hostFlashSetPollReads(0);
	FLASH_EraseInitTypeDef a = feSectors(0, 1, FLASH_VOLTAGE_RANGE_3);
	REQUIRE(HAL_FLASHEx_Erase_IT(&a) == HAL_OK);
	feIrq();
	REQUIRE(flashCounts.eop == 1u);
	FLASH_EraseInitTypeDef b = feSectors(7, 1, FLASH_VOLTAGE_RANGE_3);
	REQUIRE(HAL_FLASHEx_Erase_IT(&b) == HAL_OK);
	CHECK(pFlash.Sector == 7u);
	feIrq();
	REQUIRE(flashCounts.eop == 2u);
	CHECK(flashCounts.lastValue == feNone);
	CHECK(feSectorMask() == 0x81u);
}

// ====================================================================================== FE3: HAL_FLASHEx_OBProgram

TEST_CASE_FIXTURE(FeFix, "FE3 enabling the write protection of one sector clears its nWRP bit and nothing else, for every sector") {
	// OPTCR after unlock 0x0FFFAAEC: nWRP (27:16) = 0xFFF. Protecting sector s: nWRP = 0xFFF & ~(1 << s); OPTCR = (nWRP << 16) | 0xAAEC.
	for (unsigned s = 0; s < 12u; ++s) {
		CAPTURE(s);
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), feOptcrAfterUnlock);
		FLASH_OBProgramInitTypeDef o = feOb();
		o.OptionType = OPTIONBYTE_WRP;
		o.WRPState = OB_WRPSTATE_ENABLE;
		o.WRPSector = 1u << s;
		o.Banks = FLASH_BANK_1;
		CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
		CHECK(feReg(&FLASH->OPTCR) == ((0xFFFu & ~(1u << s)) << 16 | 0xAAECu));
	}
}

TEST_CASE_FIXTURE(FeFix, "FE3 the OB_WRP_SECTOR_n constants select the sector n") {
	const uint32_t c[12] = {OB_WRP_SECTOR_0, OB_WRP_SECTOR_1, OB_WRP_SECTOR_2, OB_WRP_SECTOR_3, OB_WRP_SECTOR_4,  OB_WRP_SECTOR_5,
	                        OB_WRP_SECTOR_6, OB_WRP_SECTOR_7, OB_WRP_SECTOR_8, OB_WRP_SECTOR_9, OB_WRP_SECTOR_10, OB_WRP_SECTOR_11};
	for (unsigned s = 0; s < 12u; ++s) {
		CAPTURE(s);
		CHECK(c[s] == (1u << s)); // UM1725 FLASHEx_WRP_Sectors: sector n is bit n, as OPTCR nWRPn is bit 16 + n
	}
}

TEST_CASE_FIXTURE(FeFix, "FE3 enabling the write protection of several sectors in one call clears all their nWRP bits") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), feOptcrAfterUnlock);
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_WRP;
	o.WRPState = OB_WRPSTATE_ENABLE;
	o.WRPSector = OB_WRP_SECTOR_0 | OB_WRP_SECTOR_5 | OB_WRP_SECTOR_11;
	o.Banks = FLASH_BANK_1;
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	CHECK(feReg(&FLASH->OPTCR) == ((0xFFFu & ~0x821u) << 16 | 0xAAECu)); // 0xFFF & ~0x821 = 0x7DE
}

TEST_CASE_FIXTURE(FeFix, "FE3 enabling the write protection of all sectors clears every nWRP bit") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), feOptcrAfterUnlock);
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_WRP;
	o.WRPState = OB_WRPSTATE_ENABLE;
	o.WRPSector = OB_WRP_SECTOR_All;
	o.Banks = FLASH_BANK_1;
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	CHECK(feReg(&FLASH->OPTCR) == 0x0000AAECu);
}

TEST_CASE_FIXTURE(FeFix, "FE3 disabling the write protection sets the nWRP bits of the selected sectors only") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), 0x0000AAECu); // all sectors protected
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_WRP;
	o.WRPState = OB_WRPSTATE_DISABLE;
	o.WRPSector = OB_WRP_SECTOR_2 | OB_WRP_SECTOR_9;
	o.Banks = FLASH_BANK_1;
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	CHECK(feReg(&FLASH->OPTCR) == ((0x204u) << 16 | 0xAAECu));
}

TEST_CASE_FIXTURE(FeFix, "FE3 disabling the write protection of all sectors sets every nWRP bit") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), 0x0000AAECu);
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_WRP;
	o.WRPState = OB_WRPSTATE_DISABLE;
	o.WRPSector = OB_WRP_SECTOR_All;
	o.Banks = FLASH_BANK_1;
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	CHECK(feReg(&FLASH->OPTCR) == feOptcrAfterUnlock);
}

TEST_CASE_FIXTURE(FeFix, "FE3 a protected sector programmed after OBProgram and OB_Launch is rejected with WRPERR") {
	feUnlock();
	feObUnlock();
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_WRP;
	o.WRPState = OB_WRPSTATE_ENABLE;
	o.WRPSector = OB_WRP_SECTOR_1;
	o.Banks = FLASH_BANK_1;
	REQUIRE(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08004000u, 0u) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08008000u, 0u) == HAL_OK);
}

TEST_CASE_FIXTURE(FeFix, "FE3 the RDP level is written to option byte 1 for level 0, 1 and 2") {
	// RM0090 3.7.2: RDP 0xAA level 0, 0xCC level 2, any other value level 1 (the HAL constant is 0x55).
	const uint8_t levels[3] = {OB_RDP_LEVEL_0, OB_RDP_LEVEL_1, OB_RDP_LEVEL_2};
	const uint32_t bytes[3] = {0xAAu, 0x55u, 0xCCu};
	for (unsigned i = 0; i < 3u; ++i) {
		CAPTURE(i);
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), 0x0FF5A5ECu);
		FLASH_OBProgramInitTypeDef o = feOb();
		o.OptionType = OPTIONBYTE_RDP;
		o.RDPLevel = levels[i];
		CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
		CHECK(feReg(&FLASH->OPTCR) == (0x0FF50000u | (bytes[i] << 8) | 0xECu)); // only byte 1 changed
	}
}

TEST_CASE_FIXTURE(FeFix, "FE3 the user byte keeps OPTLOCK, OPTSTRT and BOR and replaces the three user bits, for all eight combinations") {
	// USERConfig bits: IWDG_SW 0x20 (OPTCR bit 5), STOP_NO_RST 0x40 (bit 6), STDBY_NO_RST 0x80 (bit 7). Low byte kept: 0xEC & 0x1F = 0x0C.
	for (unsigned v = 0; v < 8u; ++v) {
		CAPTURE(v);
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), 0x0FFFAA0Cu | 0xE0u); // user bits all 1 first
		FLASH_OBProgramInitTypeDef o = feOb();
		o.OptionType = OPTIONBYTE_USER;
		o.USERConfig = static_cast<uint8_t>(v << 5);
		CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
		CHECK(feReg(&FLASH->OPTCR) == (0x0FFFAA00u | (v << 5) | 0x0Cu));
	}
}

TEST_CASE_FIXTURE(FeFix, "FE3 the user config uses the named option constants for the three bits") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), feOptcrAfterUnlock);
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_USER;
	o.USERConfig = OB_IWDG_HW | OB_STOP_NO_RST | OB_STDBY_RST; // 0x00 | 0x40 | 0x00
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	CHECK((feReg(&FLASH->OPTCR) & 0xE0u) == 0x40u);
	o.USERConfig = OB_IWDG_SW | OB_STOP_RST | OB_STDBY_NO_RST; // 0x20 | 0x00 | 0x80
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	CHECK((feReg(&FLASH->OPTCR) & 0xE0u) == 0xA0u);
}

TEST_CASE_FIXTURE(FeFix, "FE3 the BOR level is written to OPTCR bits 3:2 for every level and the user bits stay") {
	// RM0090 3.9.9: BOR_LEV 00 level 3, 01 level 2, 10 level 1, 11 off. HAL: BOR_LEVEL3 0x00, LEVEL2 0x04, LEVEL1 0x08, OFF 0x0C.
	const uint8_t lev[4] = {OB_BOR_LEVEL3, OB_BOR_LEVEL2, OB_BOR_LEVEL1, OB_BOR_OFF};
	for (unsigned i = 0; i < 4u; ++i) {
		CAPTURE(i);
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), 0x0FFFAAECu);
		FLASH_OBProgramInitTypeDef o = feOb();
		o.OptionType = OPTIONBYTE_BOR;
		o.BORLevel = lev[i];
		CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
		CHECK(feReg(&FLASH->OPTCR) == (0x0FFFAAE0u | (i << 2)));
	}
}

TEST_CASE_FIXTURE(FeFix, "FE3 all four option types in one call program WRP, RDP, user and BOR together") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), feOptcrAfterUnlock);
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_WRP | OPTIONBYTE_RDP | OPTIONBYTE_USER | OPTIONBYTE_BOR;
	o.WRPState = OB_WRPSTATE_ENABLE;
	o.WRPSector = OB_WRP_SECTOR_4;
	o.Banks = FLASH_BANK_1;
	o.RDPLevel = OB_RDP_LEVEL_1;
	o.USERConfig = OB_IWDG_SW | OB_STOP_NO_RST;
	o.BORLevel = OB_BOR_LEVEL2;
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	// nWRP = 0xFFF & ~0x10 = 0xFEF; RDP 0x55; user 0x20 | 0x40 = 0x60 (the order USER then BOR keeps both); BOR level 2 = 0x04.
	CHECK(feReg(&FLASH->OPTCR) == (0x0FEF0000u | 0x5500u | 0x60u | 0x04u));
}

TEST_CASE_FIXTURE(FeFix, "FE3 OBProgram does not set OPTSTRT and does not change the option byte words before OB_Launch") {
	feObUnlock();
	const uint32_t word0 = hostRegsPeek(0x1FFFC000u), word1 = hostRegsPeek(0x1FFFC008u);
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_RDP | OPTIONBYTE_BOR;
	o.RDPLevel = OB_RDP_LEVEL_1;
	o.BORLevel = OB_BOR_LEVEL1;
	REQUIRE(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	CHECK((feReg(&FLASH->OPTCR) & 2u) == 0u); // RM0090 OPTCR OPTSTRT bit 1 is set by HAL_FLASH_OB_Launch, not here
	CHECK(hostFlashComplete() == 0);
	CHECK(hostRegsPeek(0x1FFFC000u) == word0);
	CHECK(hostRegsPeek(0x1FFFC008u) == word1);
}

TEST_CASE_FIXTURE(FeFix, "FE3 after OBProgram and OB_Launch the option byte words hold the new RDP and BOR") {
	feObUnlock();
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_RDP | OPTIONBYTE_BOR;
	o.RDPLevel = OB_RDP_LEVEL_1;
	o.BORLevel = OB_BOR_LEVEL1;
	REQUIRE(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	REQUIRE(HAL_FLASH_OB_Launch() == HAL_OK);
	CHECK(((hostRegsPeek(0x1FFFC000u) >> 8) & 0xFFu) == 0x55u); // RM0090 Table 16: RDP in byte 1 of the option bytes
}

TEST_CASE_FIXTURE(FeFix, "FE3 OBProgram of the write protection does not need the main flash CR to be unlocked") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), feOptcrAfterUnlock);
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_WRP;
	o.WRPState = OB_WRPSTATE_ENABLE;
	o.WRPSector = OB_WRP_SECTOR_8;
	o.Banks = FLASH_BANK_1;
	CHECK((feReg(&FLASH->CR) & feCrLock) == feCrLock);
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	CHECK(((feReg(&FLASH->OPTCR) >> 16) & 0x100u) == 0u);
}

TEST_CASE_FIXTURE(FeFix, "FE3 the process lock is released after OBProgram") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), feOptcrAfterUnlock);
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_BOR;
	o.BORLevel = OB_BOR_LEVEL1;
	REQUIRE(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	CHECK(pFlash.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(FeFix, "FE3 OBProgram while the process is locked returns HAL_BUSY and writes nothing") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), feOptcrAfterUnlock);
	pFlash.Lock = HAL_LOCKED;
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_WRP | OPTIONBYTE_RDP | OPTIONBYTE_USER | OPTIONBYTE_BOR;
	o.WRPState = OB_WRPSTATE_ENABLE;
	o.WRPSector = OB_WRP_SECTOR_All;
	o.RDPLevel = OB_RDP_LEVEL_2;
	o.BORLevel = OB_BOR_OFF;
	hostRegsLogClear();
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_BUSY);
	CHECK(feWriteCount() == 0u);
	CHECK(feReg(&FLASH->OPTCR) == feOptcrAfterUnlock);
	pFlash.Lock = HAL_UNLOCKED;
}

TEST_CASE_FIXTURE(FeFix, "FE3 option byte programming while the flash is busy returns HAL_TIMEOUT for WRP, RDP and user and writes nothing") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), feOptcrAfterUnlock);
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->SR), feSrBsy); // never clears
	hostTickSetStep(feJump);
	const uint32_t types[3] = {OPTIONBYTE_WRP, OPTIONBYTE_RDP, OPTIONBYTE_USER};
	for (unsigned i = 0; i < 3u; ++i) {
		CAPTURE(i);
		FLASH_OBProgramInitTypeDef o = feOb();
		o.OptionType = types[i];
		o.WRPState = OB_WRPSTATE_ENABLE;
		o.WRPSector = OB_WRP_SECTOR_3;
		o.Banks = FLASH_BANK_1;
		o.RDPLevel = OB_RDP_LEVEL_1;
		o.USERConfig = 0;
		hostRegsLogClear();
		CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_TIMEOUT);
		CHECK(feReg(&FLASH->OPTCR) == feOptcrAfterUnlock);
		CHECK(pFlash.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(FeFix, "FE3 the WRP disable also returns HAL_TIMEOUT while the flash is busy") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), 0x0000AAECu);
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->SR), feSrBsy);
	hostTickSetStep(feJump);
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_WRP;
	o.WRPState = OB_WRPSTATE_DISABLE;
	o.WRPSector = OB_WRP_SECTOR_3;
	o.Banks = FLASH_BANK_1;
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_TIMEOUT);
	CHECK(feReg(&FLASH->OPTCR) == 0x0000AAECu);
}

TEST_CASE_FIXTURE(FeFix, "FE3 option byte programming with a pending error flag returns HAL_ERROR with that error code and writes nothing") {
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), feOptcrAfterUnlock);
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->SR), feSrPgserr);
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_RDP;
	o.RDPLevel = OB_RDP_LEVEL_1;
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGS);
	CHECK(feReg(&FLASH->OPTCR) == feOptcrAfterUnlock);
}

TEST_CASE_FIXTURE(FeFix, "FE3 characterization: OBProgram on locked option bytes returns HAL_OK and changes nothing") {
	// NEEDS REVIEW 343/6: UM1725 requires HAL_FLASH_OB_Unlock first but defines no status for a locked option control register; RM0090: writes to OPTCR are ignored while OPTLOCK = 1.
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_WRP | OPTIONBYTE_RDP | OPTIONBYTE_USER | OPTIONBYTE_BOR;
	o.WRPState = OB_WRPSTATE_ENABLE;
	o.WRPSector = OB_WRP_SECTOR_All;
	o.Banks = FLASH_BANK_1;
	o.RDPLevel = OB_RDP_LEVEL_1;
	o.BORLevel = OB_BOR_LEVEL1;
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	CHECK(feReg(&FLASH->OPTCR) == 0x0FFFAAEDu);
}

TEST_CASE_FIXTURE(FeFix, "FE3 characterization: OptionType 0 does nothing and returns HAL_ERROR") {
	// NEEDS REVIEW 343/7: UM1725 does not define the status for an empty option type; the HAL returns its initial HAL_ERROR.
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), feOptcrAfterUnlock);
	FLASH_OBProgramInitTypeDef o = feOb();
	hostRegsLogClear();
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_ERROR);
	CHECK(feWriteCount() == 0u);
	CHECK(pFlash.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(FeFix, "FE3 characterization: the status of OBProgram is the one of the last option type processed") {
	// NEEDS REVIEW 343/8: UM1725 gives only "HAL Status". With the flash busy the WRP step times out, then the BOR step (which never waits) runs and returns HAL_OK; the HAL_OK hides the timeout.
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), feOptcrAfterUnlock);
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->SR), feSrBsy);
	hostTickSetStep(feJump);
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_WRP | OPTIONBYTE_BOR;
	o.WRPState = OB_WRPSTATE_ENABLE;
	o.WRPSector = OB_WRP_SECTOR_3;
	o.Banks = FLASH_BANK_1;
	o.BORLevel = OB_BOR_LEVEL1;
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	CHECK(((feReg(&FLASH->OPTCR) >> 16) & 8u) == 8u); // the WRP step did not happen
}

TEST_CASE_FIXTURE(FeFix, "FE3 characterization: the HAL does not wait for a running operation before writing the BOR level") {
	// NEEDS REVIEW 343/9: UM1725 / RM0090 3.6.1 do not say the BOR step must wait; the HAL writes OPTCR at once and returns HAL_OK although an erase is running (the other steps wait).
	feUnlock();
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), feOptcrAfterUnlock);
	hostFlashSetPollReads(0);
	FLASH_EraseInitTypeDef e = feSectors(2, 1, FLASH_VOLTAGE_RANGE_3);
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	hostRegsLogClear();
	FLASH_OBProgramInitTypeDef o = feOb();
	o.OptionType = OPTIONBYTE_BOR;
	o.BORLevel = OB_BOR_LEVEL1;
	CHECK(HAL_FLASHEx_OBProgram(&o) == HAL_OK);
	CHECK(hostTickGetCalls() == 0u); // no tick read: no wait loop
	CHECK(hostRegsWritesTo(reinterpret_cast<uintptr_t>(&FLASH->OPTCR)) > 0u);
}

// ====================================================================================== FE4: HAL_FLASHEx_OBGetConfig

TEST_CASE_FIXTURE(FeFix, "FE4 OBGetConfig reports all four option types whatever the structure held before") {
	FLASH_OBProgramInitTypeDef o;
	std::memset(&o, 0xAB, sizeof o);
	HAL_FLASHEx_OBGetConfig(&o);
	CHECK(o.OptionType == (OPTIONBYTE_WRP | OPTIONBYTE_RDP | OPTIONBYTE_USER | OPTIONBYTE_BOR));
}

TEST_CASE_FIXTURE(FeFix, "FE4 OBGetConfig writes no register") {
	FLASH_OBProgramInitTypeDef o = feOb();
	hostRegsLogClear();
	HAL_FLASHEx_OBGetConfig(&o);
	CHECK(feWriteCount() == 0u);
}

TEST_CASE_FIXTURE(FeFix, "FE4 the reset value of OPTCR reads as RDP level 0, no BOR and user bits all 1") {
	// RM0090 3.9.9 reset value 0x0FFFAAED: RDP 0xAA (level 0), BOR_LEV 11 (off), USER 111, nWRP 0xFFF.
	FLASH_OBProgramInitTypeDef o = feOb();
	HAL_FLASHEx_OBGetConfig(&o);
	CHECK(o.RDPLevel == OB_RDP_LEVEL_0);
	CHECK(o.BORLevel == OB_BOR_OFF);
	CHECK(o.USERConfig == (OB_IWDG_SW | OB_STOP_NO_RST | OB_STDBY_NO_RST));
	CHECK(o.WRPSector == 0xFFFu);
}

TEST_CASE_FIXTURE(FeFix, "FE4 RDP 0xAA is level 0 and 0xCC is level 2") {
	FLASH_OBProgramInitTypeDef o = feOb();
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), 0x0FFFAAEDu);
	HAL_FLASHEx_OBGetConfig(&o);
	CHECK(o.RDPLevel == OB_RDP_LEVEL_0);
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), 0x0FFFCCEDu);
	HAL_FLASHEx_OBGetConfig(&o);
	CHECK(o.RDPLevel == OB_RDP_LEVEL_2);
}

TEST_CASE_FIXTURE(FeFix, "FE4 every RDP byte other than 0xAA and 0xCC reads as level 1") {
	// RM0090 3.7.2: any value other than 0xAA and 0xCC is level 1.
	const uint32_t v[8] = {0x55u, 0x00u, 0xFFu, 0xA9u, 0xABu, 0xCBu, 0xCDu, 0x01u};
	for (unsigned i = 0; i < 8u; ++i) {
		CAPTURE(v[i]);
		FLASH_OBProgramInitTypeDef o = feOb();
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), 0x0FFF0000u | (v[i] << 8) | 0xEDu);
		HAL_FLASHEx_OBGetConfig(&o);
		CHECK(o.RDPLevel == OB_RDP_LEVEL_1);
	}
}

TEST_CASE_FIXTURE(FeFix, "FE4 the three user bits are returned in their option byte positions for all eight combinations and no other bit") {
	for (unsigned v = 0; v < 8u; ++v) {
		CAPTURE(v);
		FLASH_OBProgramInitTypeDef o = feOb();
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), 0x0FFFAA00u | (v << 5) | 0x0Fu); // BOR and lock bits set to prove they are masked
		HAL_FLASHEx_OBGetConfig(&o);
		CHECK(o.USERConfig == static_cast<uint8_t>(v << 5));
	}
}

TEST_CASE_FIXTURE(FeFix, "FE4 the BOR level is bits 3:2 of OPTCR for all four values and no other bit") {
	for (unsigned i = 0; i < 4u; ++i) {
		CAPTURE(i);
		FLASH_OBProgramInitTypeDef o = feOb();
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), 0x0FFFAAE1u | (i << 2)); // user bits and OPTLOCK set to prove they are masked
		HAL_FLASHEx_OBGetConfig(&o);
		CHECK(o.BORLevel == (i << 2));
	}
	CHECK(OB_BOR_LEVEL3 == 0x00u);
	CHECK(OB_BOR_LEVEL2 == 0x04u);
	CHECK(OB_BOR_LEVEL1 == 0x08u);
	CHECK(OB_BOR_OFF == 0x0Cu);
}

TEST_CASE_FIXTURE(FeFix, "FE4 characterization: WRPSector is the raw nWRP field, a 0 bit meaning protected") {
	// NEEDS REVIEW 343/10: the struct field is documented as "the sector(s) to be write protected", which would suggest 1 = protected; the HAL returns the nWRP bits (RM0090: nWRPi = 0 protects).
	const uint32_t v[5] = {0xFFFu, 0x000u, 0x5A5u, 0x001u, 0x800u};
	for (unsigned i = 0; i < 5u; ++i) {
		CAPTURE(v[i]);
		FLASH_OBProgramInitTypeDef o = feOb();
		hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->OPTCR), (v[i] << 16) | 0xAAEDu);
		HAL_FLASHEx_OBGetConfig(&o);
		CHECK(o.WRPSector == v[i]);
	}
}

TEST_CASE_FIXTURE(FeFix, "FE4 characterization: OBGetConfig leaves WRPState and Banks as the caller set them") {
	// NEEDS REVIEW 343/11: UM1725 describes only OptionType, WRPSector, RDPLevel, USERConfig and BORLevel as results of the call.
	FLASH_OBProgramInitTypeDef o = feOb();
	o.WRPState = 0x1234u;
	o.Banks = 0x5678u;
	HAL_FLASHEx_OBGetConfig(&o);
	CHECK(o.WRPState == 0x1234u);
	CHECK(o.Banks == 0x5678u);
}

TEST_CASE_FIXTURE(FeFix, "FE4 what OBProgram wrote is what OBGetConfig reads back for RDP, user and BOR") {
	feObUnlock();
	FLASH_OBProgramInitTypeDef w = feOb();
	w.OptionType = OPTIONBYTE_RDP | OPTIONBYTE_USER | OPTIONBYTE_BOR;
	w.RDPLevel = OB_RDP_LEVEL_2;
	w.USERConfig = OB_IWDG_SW | OB_STDBY_NO_RST;
	w.BORLevel = OB_BOR_LEVEL2;
	REQUIRE(HAL_FLASHEx_OBProgram(&w) == HAL_OK);
	FLASH_OBProgramInitTypeDef r = feOb();
	HAL_FLASHEx_OBGetConfig(&r);
	CHECK(r.RDPLevel == OB_RDP_LEVEL_2);
	CHECK(r.USERConfig == (OB_IWDG_SW | OB_STDBY_NO_RST));
	CHECK(r.BORLevel == OB_BOR_LEVEL2);
}

TEST_CASE_FIXTURE(FeFix, "FE1 a mass erase after a sector erase with another voltage range replaces PSIZE") {
	for (unsigned first = 2; first < 4u; ++first) {
		for (unsigned second = 0; second < 2u; ++second) {
			CAPTURE(first);
			CAPTURE(second);
			hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->CR), 0u);
			FLASH_EraseInitTypeDef a = feSectors(1, 1, feRanges[first]);
			uint32_t err = 0;
			REQUIRE(HAL_FLASHEx_Erase(&a, &err) == HAL_OK);
			hostRegsLogClear();
			FLASH_EraseInitTypeDef m = feMass(feRanges[second]);
			REQUIRE(HAL_FLASHEx_Erase(&m, &err) == HAL_OK);
			const std::vector<uint32_t> w = feStrtWrites();
			REQUIRE(w.size() == 1u);
			CHECK(((w[0] >> 8) & 3u) == second); // RM0090 CR PSIZE holds only the new parallelism
		}
	}
}

TEST_CASE_FIXTURE(FeFix, "FE2 Erase_IT clears a pending OPERR flag before it starts") {
	feUnlock();
	hostFlashSetPollReads(0);
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->SR), 0x2u);
	FLASH_EraseInitTypeDef e = feSectors(1, 1, FLASH_VOLTAGE_RANGE_3);
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	CHECK((feReg(&FLASH->SR) & 0x2u) == 0u);
}

TEST_CASE_FIXTURE(FeFix, "FE4 characterization: a sector protected with OBProgram reads back as a 0 bit in OBGetConfig WRPSector") {
	// NEEDS REVIEW 343/10 evidence: OBProgram(ENABLE, sector 4) then OBGetConfig; the struct field says "sectors to be write protected" but the bit read back is 0 for the protected sector.
	feObUnlock();
	FLASH_OBProgramInitTypeDef w = feOb();
	w.OptionType = OPTIONBYTE_WRP;
	w.WRPState = OB_WRPSTATE_ENABLE;
	w.WRPSector = OB_WRP_SECTOR_4;
	w.Banks = FLASH_BANK_1;
	REQUIRE(HAL_FLASHEx_OBProgram(&w) == HAL_OK);
	FLASH_OBProgramInitTypeDef r = feOb();
	HAL_FLASHEx_OBGetConfig(&r);
	CHECK(r.WRPSector == (0xFFFu & ~OB_WRP_SECTOR_4));
	CHECK(r.WRPSector != OB_WRP_SECTOR_4);
}

TEST_CASE_FIXTURE(FeFix, "FE1 characterization: SectorError is left untouched when the first wait fails (stale error flag)") {
	// NEEDS REVIEW 343/12: UM1725 defines SectorError only for sectors that failed or 0xFFFFFFFF; when the call fails before any sector starts the HAL writes nothing to it.
	feUnlock();
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->SR), feSrPgserr);
	FLASH_EraseInitTypeDef e = feSectors(2, 1, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0x1234u;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_ERROR);
	CHECK(err == 0x1234u);
}

TEST_CASE_FIXTURE(FeFix, "FE1 characterization: SectorError is left untouched when the first wait times out (flash busy)") {
	// NEEDS REVIEW 343/13: as NEEDS REVIEW 343/12 for a busy flash.
	feUnlock();
	hostRegsPoke(reinterpret_cast<uintptr_t>(&FLASH->SR), feSrBsy);
	hostTickSetStep(feJump);
	FLASH_EraseInitTypeDef e = feMass(FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0x1234u;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_TIMEOUT);
	CHECK(err == 0x1234u);
}

TEST_CASE_FIXTURE(FeFix, "FE1 a timeout is detected when the tick starts at a non-zero value") {
	feUnlock();
	hostTickSet(0x10000u);
	hostFlashSetPollReads(0);
	hostTickSetStep(feJump);
	FLASH_EraseInitTypeDef e = feSectors(4, 1, FLASH_VOLTAGE_RANGE_3);
	uint32_t err = 0;
	CHECK(HAL_FLASHEx_Erase(&e, &err) == HAL_TIMEOUT);
	CHECK(err == 4u);
}
