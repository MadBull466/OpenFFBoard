// P12 T7 group FB: HAL_FLASH_Program / Program_IT / IRQHandler / Unlock / Lock / OB_Unlock / OB_Lock / OB_Launch / GetError,
// FLASH_WaitForLastOperation and FLASH_SetErrorCode (through the public functions and the SR flags), real ST HAL, F407VG and F407VG_DISCO.
// Oracles: UM1725 Rev 8 chapter "HAL FLASH Generic Driver", RM0090 Rev 22 chapter 3 (3.6.1 unlock, 3.6.2 Table 13, 3.6.4 programming and errors,
// 3.6.6 interrupts, 3.7 option bytes Tables 15 / 16, 3.8 OTP, 3.9 registers). Charter: P12_T7_REQUIREMENTS.md (group FB).
#include "FlashTestSupport.h"

#include <cstdint>
#include <cstring>

extern "C" {
extern FLASH_ProcessTypeDef pFlash;
}

// misaligned store: skipped under sanitizers, legal on Cortex-M4 (the HAL performs a misaligned C store; UBSan aborts the run)
#ifdef HOSTTEST_SANITIZERS
#define FB_MISALIGNED_STORE * doctest::skip()
#else
#define FB_MISALIGNED_STORE
#endif

namespace {

// The HAL keeps its state in the global pFlash; every case starts from the zero state of a fresh program.
class FbFixture : public FlashFixture {
public:
	FbFixture() { std::memset(const_cast<FLASH_ProcessTypeDef *>(&pFlash), 0, sizeof pFlash); }
};

constexpr uint32_t kSr_EOP = 0x1u, kSr_OPERR = 0x2u, kSr_WRPERR = 0x10u, kSr_PGAERR = 0x20u, kSr_PGPERR = 0x40u, kSr_PGSERR = 0x80u;
constexpr uint32_t kSr_BSY = 0x10000u;
constexpr uint32_t kCr_PG = 0x1u, kCr_PSIZE = 0x300u, kCr_EOPIE = 0x1000000u, kCr_ERRIE = 0x2000000u, kCr_LOCK = 0x80000000u;
constexpr uint32_t kOptcr_OPTLOCK = 0x1u, kOptcr_OPTSTRT = 0x2u;
constexpr uint32_t kOtpBase = 0x1FFF7800u;

// Raw (not trapped, not logged, no model side effect) register values.
uint32_t rawSr() { return HalFixture::peek(&FLASH->SR); }
uint32_t rawCr() { return HalFixture::peek(&FLASH->CR); }
uint32_t rawOptcr() { return HalFixture::peek(&FLASH->OPTCR); }

uint32_t wordAt(uint32_t addr) { return hostRegsPeek(addr); }
uint32_t byteAt(uint32_t addr) { return (hostRegsPeek(addr & ~3u) >> (8u * (addr & 3u))) & 0xFFu; }

void unlockAll() {
	REQUIRE(HAL_FLASH_Unlock() == HAL_OK);
}

// RM0090 3.9.9: nWRPi = 0 write protects sector i (OPTCR bits 27:16).
void protectSector(unsigned sector) { HalFixture::poke(&FLASH->OPTCR, rawOptcr() & ~(1u << (16u + sector))); }

// Starts a raw program of a word by hand (RM0090 3.6.4) so that BSY is set when a HAL call begins.
void startRawProgram(uint32_t addr) {
	FLASH->CR |= (0x2u << 8) | kCr_PG;
	*reinterpret_cast<volatile uint32_t *>(addr) = 0u;
	REQUIRE((rawSr() & kSr_BSY) != 0u);
}

void setProcedure(FLASH_ProcedureTypeDef proc) { pFlash.ProcedureOnGoing = proc; }

// The bits the handler clears when the procedure is over (RM0090 3.9.8: PG 0, SER 1, MER 2, SNB 6:3) and the two interrupt enables.
constexpr uint32_t kCleanedByIrq = 0x1u | 0x2u | 0x4u | 0x78u;

} // namespace

// ------------------------------------------------------------------------------------------------ FB1 locks

TEST_CASE_FIXTURE(FbFixture, "model: FB1 the flash control register is locked after reset") {
	CHECK((rawCr() & kCr_LOCK) != 0u);
	CHECK((rawOptcr() & kOptcr_OPTLOCK) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB1 unlock clears CR.LOCK and returns HAL_OK") {
	CHECK(HAL_FLASH_Unlock() == HAL_OK);
	CHECK((rawCr() & kCr_LOCK) == 0u);
	CHECK(hostFlashBusFaults() == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB1 unlock writes KEY1 0x45670123 and then KEY2 0xCDEF89AB to KEYR") {
	REQUIRE(HAL_FLASH_Unlock() == HAL_OK);
	uint32_t keys[4] = {};
	unsigned n = 0;
	const uintptr_t keyr = reinterpret_cast<uintptr_t>(&FLASH->KEYR);
	for (size_t i = 0; i < hostRegsLogCount(); ++i) {
		const HostRegAccess *e = hostRegsLogAt(i);
		if (e->write && e->addr == keyr && n < 4u) keys[n++] = e->written;
	}
	REQUIRE(n == 2u);
	CHECK(keys[0] == 0x45670123u);
	CHECK(keys[1] == 0xCDEF89ABu);
}

TEST_CASE_FIXTURE(FbFixture, "characterization: FB1 unlock of an already unlocked flash returns HAL_OK and writes no key") {
	// NEEDS REVIEW 342/5: UM1725 only says "HAL: Status"; the HAL skips the key sequence when LOCK is already clear.
	REQUIRE(HAL_FLASH_Unlock() == HAL_OK);
	hostRegsLogClear();
	CHECK(HAL_FLASH_Unlock() == HAL_OK);
	CHECK(hostRegsWritesTo(reinterpret_cast<uintptr_t>(&FLASH->KEYR)) == 0u);
	CHECK((rawCr() & kCr_LOCK) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB1 unlock returns HAL_ERROR after a wrong key sequence locked the register until reset") {
	// RM0090 3.6.1: any wrong sequence returns a bus error and locks FLASH_CR until the next reset.
	FLASH->KEYR = 0x11111111u;
	FLASH->KEYR = 0x22222222u;
	CHECK(hostFlashBusFaults() == 1u);
	CHECK(HAL_FLASH_Unlock() == HAL_ERROR);
	CHECK((rawCr() & kCr_LOCK) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB1 unlock fails after KEY1 was followed by a wrong second key") {
	FLASH->KEYR = 0x45670123u;
	FLASH->KEYR = 0x12345678u;
	CHECK(HAL_FLASH_Unlock() == HAL_ERROR);
	CHECK((rawCr() & kCr_LOCK) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB1 lock sets CR.LOCK and returns HAL_OK") {
	unlockAll();
	CHECK(HAL_FLASH_Lock() == HAL_OK);
	CHECK((rawCr() & kCr_LOCK) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB1 an unlock after a lock succeeds again") {
	unlockAll();
	REQUIRE(HAL_FLASH_Lock() == HAL_OK);
	CHECK(HAL_FLASH_Unlock() == HAL_OK);
	CHECK((rawCr() & kCr_LOCK) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB1 lock of an already locked flash returns HAL_OK and keeps it locked") {
	CHECK(HAL_FLASH_Lock() == HAL_OK);
	CHECK((rawCr() & kCr_LOCK) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "model: FB1 a locked flash ignores writes to CR") {
	REQUIRE(HAL_FLASH_Lock() == HAL_OK);
	FLASH->CR |= kCr_PG;
	CHECK((rawCr() & kCr_PG) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB1 option byte unlock clears OPTCR.OPTLOCK and returns HAL_OK") {
	CHECK(HAL_FLASH_OB_Unlock() == HAL_OK);
	CHECK((rawOptcr() & kOptcr_OPTLOCK) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB1 option byte unlock writes OPTKEY1 0x08192A3B and then OPTKEY2 0x4C5D6E7F to OPTKEYR") {
	REQUIRE(HAL_FLASH_OB_Unlock() == HAL_OK);
	uint32_t keys[4] = {};
	unsigned n = 0;
	const uintptr_t keyr = reinterpret_cast<uintptr_t>(&FLASH->OPTKEYR);
	for (size_t i = 0; i < hostRegsLogCount(); ++i) {
		const HostRegAccess *e = hostRegsLogAt(i);
		if (e->write && e->addr == keyr && n < 4u) keys[n++] = e->written;
	}
	REQUIRE(n == 2u);
	CHECK(keys[0] == 0x08192A3Bu);
	CHECK(keys[1] == 0x4C5D6E7Fu);
}

TEST_CASE_FIXTURE(FbFixture, "model: FB1 option byte unlock does not change CR.LOCK") {
	REQUIRE(HAL_FLASH_OB_Unlock() == HAL_OK);
	CHECK((rawCr() & kCr_LOCK) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "model: FB1 unlock does not change OPTCR.OPTLOCK") {
	REQUIRE(HAL_FLASH_Unlock() == HAL_OK);
	CHECK((rawOptcr() & kOptcr_OPTLOCK) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "characterization: FB1 option byte unlock of an already unlocked register returns HAL_ERROR and writes no key") {
	// NEEDS REVIEW 342/6: UM1725 only says "HAL: Status"; the HAL returns HAL_ERROR when OPTLOCK is already clear.
	REQUIRE(HAL_FLASH_OB_Unlock() == HAL_OK);
	hostRegsLogClear();
	CHECK(HAL_FLASH_OB_Unlock() == HAL_ERROR);
	CHECK(hostRegsWritesTo(reinterpret_cast<uintptr_t>(&FLASH->OPTKEYR)) == 0u);
	CHECK((rawOptcr() & kOptcr_OPTLOCK) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "characterization: FB1 option byte unlock returns HAL_OK although a wrong sequence kept OPTLOCK set") {
	// NEEDS REVIEW 342/1: UM1725 does not say whether HAL_FLASH_OB_Unlock verifies the result (HAL_FLASH_Unlock does: HAL_ERROR). The HAL returns HAL_OK
	// without reading OPTLOCK back. RM0090 3.7.2: after a wrong key sequence OPTCR stays locked until reset.
	FLASH->OPTKEYR = 0x11111111u;
	FLASH->OPTKEYR = 0x22222222u;
	CHECK(hostFlashBusFaults() == 1u);
	CHECK(HAL_FLASH_OB_Unlock() == HAL_OK);
	CHECK((rawOptcr() & kOptcr_OPTLOCK) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB1 option byte lock sets OPTCR.OPTLOCK and returns HAL_OK") {
	REQUIRE(HAL_FLASH_OB_Unlock() == HAL_OK);
	CHECK(HAL_FLASH_OB_Lock() == HAL_OK);
	CHECK((rawOptcr() & kOptcr_OPTLOCK) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB1 option byte lock of an already locked register returns HAL_OK") {
	CHECK(HAL_FLASH_OB_Lock() == HAL_OK);
	CHECK((rawOptcr() & kOptcr_OPTLOCK) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB1 an option byte unlock after an option byte lock succeeds again") {
	REQUIRE(HAL_FLASH_OB_Unlock() == HAL_OK);
	REQUIRE(HAL_FLASH_OB_Lock() == HAL_OK);
	CHECK(HAL_FLASH_OB_Unlock() == HAL_OK);
	CHECK((rawOptcr() & kOptcr_OPTLOCK) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "model: FB1 option byte lock does not change CR.LOCK") {
	unlockAll();
	REQUIRE(HAL_FLASH_OB_Lock() == HAL_OK);
	CHECK((rawCr() & kCr_LOCK) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "model: FB1 a locked option control register ignores a write of OPTSTRT") {
	REQUIRE(HAL_FLASH_OB_Unlock() == HAL_OK);
	REQUIRE(HAL_FLASH_OB_Lock() == HAL_OK);
	FLASH->OPTCR |= kOptcr_OPTSTRT;
	CHECK((rawOptcr() & kOptcr_OPTSTRT) == 0u);
	CHECK((rawSr() & kSr_BSY) == 0u);
}

// ------------------------------------------------------------------------------------------------ FB2 program (poll)

TEST_CASE_FIXTURE(FbFixture, "FB2 programming a byte stores it and leaves its neighbours erased") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x08000001u, 0xA5u) == HAL_OK);
	CHECK(byteAt(0x08000000u) == 0xFFu);
	CHECK(byteAt(0x08000001u) == 0xA5u);
	CHECK(byteAt(0x08000002u) == 0xFFu);
	CHECK(byteAt(0x08000003u) == 0xFFu);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming a byte ignores the upper bits of the data") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x08000004u, 0x123456789ABCDEF0ull) == HAL_OK);
	CHECK(wordAt(0x08000004u) == 0xFFFFFFF0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming a half word stores it little endian and leaves its neighbours erased") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x08000002u, 0x1234u) == HAL_OK);
	CHECK(wordAt(0x08000000u) == 0x1234FFFFu);
	CHECK(wordAt(0x08000004u) == 0xFFFFFFFFu);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming a half word ignores the upper bits of the data") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x08000000u, 0xDEADBEEF1234ull) == HAL_OK);
	CHECK(wordAt(0x08000000u) == 0xFFFF1234u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming a word stores it and leaves its neighbours erased") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000010u, 0x12345678u) == HAL_OK);
	CHECK(wordAt(0x0800000Cu) == 0xFFFFFFFFu);
	CHECK(wordAt(0x08000010u) == 0x12345678u);
	CHECK(wordAt(0x08000014u) == 0xFFFFFFFFu);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming a word ignores the upper 32 bits of the data") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000010u, 0x1122334455667788ull) == HAL_OK);
	CHECK(wordAt(0x08000010u) == 0x55667788u);
	CHECK(wordAt(0x08000014u) == 0xFFFFFFFFu);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming a double word stores the low word first and the high word at the next address") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x08000008u, 0x1122334455667788ull) == HAL_OK);
	CHECK(wordAt(0x08000008u) == 0x55667788u);
	CHECK(wordAt(0x0800000Cu) == 0x11223344u);
	CHECK(wordAt(0x08000004u) == 0xFFFFFFFFu);
	CHECK(wordAt(0x08000010u) == 0xFFFFFFFFu);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a word can be programmed at the base of each of the twelve sectors") {
	unlockAll();
	for (unsigned s = 0; s < 12u; ++s) {
		const uint32_t a = hostFlashSectorBase(s);
		INFO("sector " << s);
		REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, a, 0xC0DE0000u + s) == HAL_OK);
		CHECK(wordAt(a) == 0xC0DE0000u + s);
		CHECK(wordAt(a + 4u) == 0xFFFFFFFFu);
	}
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a byte can be programmed in the last byte of each sector") {
	unlockAll();
	for (unsigned s = 0; s < 12u; ++s) {
		const uint32_t a = hostFlashSectorBase(s) + hostFlashSectorSize(s) - 1u;
		INFO("sector " << s);
		REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, a, 0x3Cu) == HAL_OK);
		CHECK(byteAt(a) == 0x3Cu);
	}
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a half word can be programmed in the last half word of the flash") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x080FFFFEu, 0x55AAu) == HAL_OK);
	CHECK(wordAt(0x080FFFFCu) == 0x55AAFFFFu);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a double word can be programmed in the last double word of the flash") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x080FFFF8u, 0x0123456789ABCDEFull) == HAL_OK);
	CHECK(wordAt(0x080FFFF8u) == 0x89ABCDEFu);
	CHECK(wordAt(0x080FFFFCu) == 0x01234567u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming over programmed data keeps only the bits that are 0 in both (NOR flash)") {
	// RM0090 3.6.4: changing bits from 1 to 0 needs no erase, writing 1 needs an erase.
	// 0x12345678 AND 0xFFFF00FF = 0x12340078 (byte 1: 0x56 AND 0x00 = 0x00; other bytes AND 0xFF unchanged).
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08008000u, 0x12345678u) == HAL_OK);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08008000u, 0xFFFF00FFu) == HAL_OK);
	CHECK(wordAt(0x08008000u) == 0x12340078u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming ones over zeros cannot restore the bits") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x08008000u, 0x00u) == HAL_OK);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x08008000u, 0xFFu) == HAL_OK);
	CHECK(byteAt(0x08008000u) == 0x00u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming over data of another width merges by AND as well") {
	// byte 0x0F at 0x08008001 then half word 0xF0F0 at 0x08008000: 0x0F AND 0xF0 = 0x00 (high byte of the half word at 0x...01), 0xF0 stays at 0x...00.
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x08008001u, 0x0Fu) == HAL_OK);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x08008000u, 0xF0F0u) == HAL_OK);
	CHECK(byteAt(0x08008000u) == 0xF0u);
	CHECK(byteAt(0x08008001u) == 0x00u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming a word over a preset memory content ANDs with it") {
	// memory preset to 0x0F by hand, programmed 0xFF000000 + ... : each byte ANDed.
	unlockAll();
	hostFlashFill(0x08010000u, 0x0Fu, 4);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08010000u, 0x3CC3A55Au) == HAL_OK);
	// 0x3C & 0x0F = 0x0C, 0xC3 & 0x0F = 0x03, 0xA5 & 0x0F = 0x05, 0x5A & 0x0F = 0x0A
	CHECK(wordAt(0x08010000u) == 0x0C03050Au);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming a byte leaves the store width a byte on the bus (x8 parallelism)") {
	unlockAll();
	hostRegsLogClear();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x08000003u, 0x11u) == HAL_OK);
	CHECK(byteAt(0x08000003u) == 0x11u);
	CHECK(HalFixture::faults() == 0u);
	CHECK(rawSr() == 0u); // a PGPERR would be left in SR if the store width and PSIZE did not match
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming a byte selects PSIZE x8 (00, Table 13) and clears PG afterwards") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x08000000u, 0x00u) == HAL_OK);
	CHECK((rawCr() & kCr_PG) == 0u);
	CHECK(((rawCr() & kCr_PSIZE) >> 8) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming a half word selects PSIZE x16 (01, Table 13) and clears PG afterwards") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x08000000u, 0x00u) == HAL_OK);
	CHECK((rawCr() & kCr_PG) == 0u);
	CHECK(((rawCr() & kCr_PSIZE) >> 8) == 1u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming a word selects PSIZE x32 (10, Table 13) and clears PG afterwards") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x00u) == HAL_OK);
	CHECK((rawCr() & kCr_PG) == 0u);
	CHECK(((rawCr() & kCr_PSIZE) >> 8) == 2u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming a double word selects PSIZE x64 (11, Table 13) and clears PG afterwards") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x08000000u, 0x00u) == HAL_OK);
	CHECK((rawCr() & kCr_PG) == 0u);
	CHECK(((rawCr() & kCr_PSIZE) >> 8) == 3u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming after a wider program selects the new parallelism (PSIZE is replaced, not ORed)") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x08000000u, 0x00u) == HAL_OK);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x08000010u, 0x7Eu) == HAL_OK);
	CHECK(((rawCr() & kCr_PSIZE) >> 8) == 0u);
	CHECK(byteAt(0x08000010u) == 0x7Eu);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a half word program right after a word program selects PSIZE x16 (01)") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x00u) == HAL_OK);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x08000010u, 0x1234u) == HAL_OK);
	CHECK(((rawCr() & kCr_PSIZE) >> 8) == 1u);
	CHECK(wordAt(0x08000010u) == 0xFFFF1234u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a half word program right after a double word program selects PSIZE x16 (01)") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x08000000u, 0x00u) == HAL_OK);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x08000010u, 0x1234u) == HAL_OK);
	CHECK(((rawCr() & kCr_PSIZE) >> 8) == 1u);
	CHECK(wordAt(0x08000010u) == 0xFFFF1234u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a wait that starts at a large tick value times out after the timeout, counted from its start") {
	unlockAll();
	startRawProgram(0x08000000u);
	hostFlashSetPollReads(0);
	hostTickSet(0x10000u);
	CHECK(FLASH_WaitForLastOperation(10u) == HAL_TIMEOUT);
	CHECK(hostTickPeek() >= 0x10000u + 10u);
	CHECK(hostTickPeek() <= 0x10000u + 10u + 3u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a successful program leaves BSY, EOP and the error flags clear") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x01u) == HAL_OK);
	CHECK(rawSr() == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a successful program leaves the error code at none and releases the process lock") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x01u) == HAL_OK);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_NONE);
	CHECK(pFlash.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a poll program leaves the procedure state at none") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x01u) == HAL_OK);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a poll program does not enable the flash interrupts") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x01u) == HAL_OK);
	CHECK((rawCr() & (kCr_EOPIE | kCr_ERRIE)) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a poll program waits for a previous operation in progress and then succeeds") {
	// UM1725: the program waits for the last operation; the poll model completes the first operation at the first SR read.
	unlockAll();
	startRawProgram(0x08000000u);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000010u, 0xCAFEBABEu) == HAL_OK);
	CHECK(wordAt(0x08000010u) == 0xCAFEBABEu);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a poll program waits several SR reads for the operation to end") {
	unlockAll();
	startRawProgram(0x08000000u);
	hostFlashSetPollReads(5);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000010u, 0x0BADF00Du) == HAL_OK);
	CHECK(wordAt(0x08000010u) == 0x0BADF00Du);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a poll program returns HAL_TIMEOUT when the previous operation never ends and programs nothing") {
	unlockAll();
	startRawProgram(0x08000000u);
	hostFlashSetPollReads(0);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000010u, 0x01234567u) == HAL_TIMEOUT);
	CHECK(wordAt(0x08000010u) == 0xFFFFFFFFu);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 the timeout of a poll program is counted in HAL_GetTick ticks and releases the process lock") {
	unlockAll();
	startRawProgram(0x08000000u);
	hostFlashSetPollReads(0);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000010u, 0x0u) == HAL_TIMEOUT);
	CHECK(hostTickPeek() > 0u);
	CHECK(pFlash.Lock == HAL_UNLOCKED);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_NONE);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a poll program returns HAL_TIMEOUT when its own operation never ends") {
	unlockAll();
	hostFlashSetPollReads(0);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000020u, 0x00000000u) == HAL_TIMEOUT);
	CHECK(wordAt(0x08000020u) == 0x00000000u); // the data went into the flash, only the wait failed
	CHECK(pFlash.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a poll program works again after a timeout once the operation has ended") {
	unlockAll();
	hostFlashSetPollReads(0);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000020u, 0x0u) == HAL_TIMEOUT);
	REQUIRE(hostFlashComplete() == 1);
	hostFlashSetPollReads(1);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000030u, 0xAAAA5555u) == HAL_OK);
	CHECK(wordAt(0x08000030u) == 0xAAAA5555u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a poll program returns HAL_BUSY while the process lock is held and programs nothing") {
	// __HAL_LOCK returns HAL_BUSY (UM1725 HAL_StatusTypeDef: HAL_BUSY) when the object is already locked.
	unlockAll();
	pFlash.Lock = HAL_LOCKED;
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x0u) == HAL_BUSY);
	CHECK(wordAt(0x08000000u) == 0xFFFFFFFFu);
	CHECK(pFlash.Lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a poll program on a locked flash returns HAL_ERROR with a programming sequence error") {
	// RM0090 3.6.4: a write to flash without PG set (PG cannot be set while CR is locked) sets PGSERR.
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x0u) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGS);
	CHECK(wordAt(0x08000000u) == 0xFFFFFFFFu);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 the error flag is cleared in SR once the HAL has stored the error code") {
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x0u) == HAL_ERROR);
	CHECK((rawSr() & kSr_PGSERR) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a poll program in a write protected sector returns HAL_ERROR with the write protection error") {
	unlockAll();
	protectSector(3);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, hostFlashSectorBase(3), 0x0u) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP);
	CHECK(wordAt(hostFlashSectorBase(3)) == 0xFFFFFFFFu);
	CHECK((rawSr() & kSr_WRPERR) == 0u);
	CHECK((rawCr() & kCr_PG) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a write protected sector does not block the other sectors") {
	unlockAll();
	protectSector(3);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, hostFlashSectorBase(2), 0x1u) == HAL_OK);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, hostFlashSectorBase(4), 0x2u) == HAL_OK);
	CHECK(wordAt(hostFlashSectorBase(2)) == 0x1u);
	CHECK(wordAt(hostFlashSectorBase(4)) == 0x2u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 every sector can be write protected: the last word of each protected sector is rejected") {
	unlockAll();
	for (unsigned s = 0; s < 12u; ++s) {
		const uint32_t a = hostFlashSectorBase(s) + hostFlashSectorSize(s) - 4u;
		INFO("sector " << s);
		HalFixture::poke(&FLASH->OPTCR, 0x0FFFAAECu);
		protectSector(s);
		CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, a, 0x0u) == HAL_ERROR);
		CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP);
		CHECK(wordAt(a) == 0xFFFFFFFFu);
	}
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a double word in a write protected sector returns HAL_ERROR and stores nothing") {
	unlockAll();
	protectSector(0);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x08000000u, 0x0ull) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP);
	CHECK(wordAt(0x08000000u) == 0xFFFFFFFFu);
	CHECK(wordAt(0x08000004u) == 0xFFFFFFFFu);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 the error code of a failed program is cleared by the next successful program") {
	unlockAll();
	protectSector(3);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, hostFlashSectorBase(3), 0x0u) == HAL_ERROR);
	REQUIRE(HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, hostFlashSectorBase(4), 0x0u) == HAL_OK);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_NONE);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a failed program releases the process lock so the next call is not busy") {
	unlockAll();
	protectSector(3);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, hostFlashSectorBase(3), 0x0u) == HAL_ERROR);
	CHECK(pFlash.Lock == HAL_UNLOCKED);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, hostFlashSectorBase(4), 0x0u) != HAL_BUSY);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a word crossing the 128 bit row boundary returns HAL_ERROR with the alignment error and stores nothing" FB_MISALIGNED_STORE) {
	// RM0090 3.6.4: data that would cross the 128-bit row boundary: not performed, PGAERR. 0x0800000E + 4 crosses 0x08000010.
	unlockAll();
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800000Eu, 0x0u) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGA);
	CHECK(wordAt(0x0800000Cu) == 0xFFFFFFFFu);
	CHECK(wordAt(0x08000010u) == 0xFFFFFFFFu);
	CHECK((rawSr() & kSr_PGAERR) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a half word crossing the 128 bit row boundary returns HAL_ERROR with the alignment error" FB_MISALIGNED_STORE) {
	unlockAll();
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x0800001Fu, 0x0u) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGA);
	CHECK(wordAt(0x0800001Cu) == 0xFFFFFFFFu);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a double word crossing the 128 bit row boundary returns HAL_ERROR with the alignment error") {
	// 0x0800000C + 8 crosses 0x08000010.
	unlockAll();
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0800000Cu, 0x0ull) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGA);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a failed alignment leaves PG cleared and the process lock released" FB_MISALIGNED_STORE) {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800000Eu, 0x0u) == HAL_ERROR);
	CHECK((rawCr() & kCr_PG) == 0u);
	CHECK(pFlash.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(FbFixture, "model: FB2 the HAL cannot produce a parallelism error because it sets PSIZE to the width of the store") {
	// every width at an aligned address: no PGPERR in the error code.
	unlockAll();
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x08000000u, 0u) == HAL_OK);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x08000002u, 0u) == HAL_OK);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000004u, 0u) == HAL_OK);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x08000008u, 0u) == HAL_OK);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_NONE);
}

TEST_CASE_FIXTURE(FbFixture, "characterization: FB2 a program type above 3 is programmed as a double word") {
	// NEEDS REVIEW 342/2: UM1725 lists only the four FLASH_TYPEPROGRAM values; the final else branch programs 64 bits for any other value
	// (assert_param is compiled out, USE_FULL_ASSERT is off).
	unlockAll();
	CHECK(HAL_FLASH_Program(7u, 0x08000000u, 0x1122334455667788ull) == HAL_OK);
	CHECK(wordAt(0x08000000u) == 0x55667788u);
	CHECK(wordAt(0x08000004u) == 0x11223344u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 programming the OTP area works for byte, half word, word and double word") {
	// RM0090 3.8: the OTP area (0x1FFF7800-0x1FFF7A0F) is programmed like the main flash. Erased OTP is 0xFF.
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, kOtpBase + 0u, 0x11u) == HAL_OK);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, kOtpBase + 2u, 0x2233u) == HAL_OK);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, kOtpBase + 4u, 0x44556677u) == HAL_OK);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, kOtpBase + 8u, 0x8899AABBCCDDEEFFull) == HAL_OK);
	CHECK(wordAt(kOtpBase + 0u) == 0x2233FF11u);
	CHECK(wordAt(kOtpBase + 4u) == 0x44556677u);
	CHECK(wordAt(kOtpBase + 8u) == 0xCCDDEEFFu);
	CHECK(wordAt(kOtpBase + 12u) == 0x8899AABBu);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 an OTP word programmed twice keeps only the bits that are 0 in both") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, kOtpBase + 0x40u, 0xFF00FF00u) == HAL_OK);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, kOtpBase + 0x40u, 0x0FF00FF0u) == HAL_OK);
	CHECK(wordAt(kOtpBase + 0x40u) == 0x0F000F00u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 an OTP block whose lock byte is 0x00 is write protected") {
	// RM0090 3.8: a lock byte 0x00 locks its 32-byte block (the model rejects the store with WRPERR; confidence Low in HalFlash.h).
	unlockAll();
	hostFlashFill(0x1FFF7A00u + 1u, 0x00u, 1); // lock byte of block 1: 0x1FFF7820-0x1FFF783F
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, kOtpBase + 0x20u, 0x0u) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP);
	CHECK(wordAt(kOtpBase + 0x20u) == 0xFFFFFFFFu);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a locked OTP block does not block the neighbouring block") {
	unlockAll();
	hostFlashFill(0x1FFF7A00u + 1u, 0x00u, 1);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, kOtpBase + 0x00u, 0x0u) == HAL_OK);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, kOtpBase + 0x40u, 0x0u) == HAL_OK);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a program with a stale EOP flag set returns HAL_OK and clears EOP") {
	unlockAll();
	HalFixture::poke(&FLASH->SR, kSr_EOP);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x0u) == HAL_OK);
	CHECK((rawSr() & kSr_EOP) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a program with a stale error flag set reports that error") {
	// the HAL clears the error code, waits, finds the stale flag in SR and reports it.
	unlockAll();
	HalFixture::poke(&FLASH->SR, kSr_PGSERR);
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x0u) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGS);
}

// ------------------------------------------------------------------------------------------------ FB2 FLASH_WaitForLastOperation and FLASH_SetErrorCode

TEST_CASE_FIXTURE(FbFixture, "FB2 waiting with nothing in progress returns HAL_OK") {
	CHECK(FLASH_WaitForLastOperation(50u) == HAL_OK);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 waiting with timeout zero while busy returns HAL_TIMEOUT at once") {
	unlockAll();
	startRawProgram(0x08000000u);
	hostFlashSetPollReads(0);
	CHECK(FLASH_WaitForLastOperation(0u) == HAL_TIMEOUT);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 waiting with timeout zero when idle returns HAL_OK") {
	CHECK(FLASH_WaitForLastOperation(0u) == HAL_OK);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 waiting with an unlimited timeout returns HAL_OK once the operation ends") {
	unlockAll();
	startRawProgram(0x08000000u);
	hostFlashSetPollReads(3);
	CHECK(FLASH_WaitForLastOperation(HAL_MAX_DELAY) == HAL_OK);
	CHECK((rawSr() & kSr_BSY) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 waiting longer than the timeout while busy returns HAL_TIMEOUT after more than that many ticks") {
	unlockAll();
	startRawProgram(0x08000000u);
	hostFlashSetPollReads(0);
	CHECK(FLASH_WaitForLastOperation(10u) == HAL_TIMEOUT);
	CHECK(hostTickPeek() > 10u);
	CHECK((rawSr() & kSr_BSY) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 waiting returns HAL_OK when the operation ends before the timeout") {
	unlockAll();
	startRawProgram(0x08000000u);
	hostFlashSetPollReads(4);
	CHECK(FLASH_WaitForLastOperation(100u) == HAL_OK);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 waiting clears the end of operation flag") {
	unlockAll();
	startRawProgram(0x08000000u);
	CHECK(FLASH_WaitForLastOperation(100u) == HAL_OK);
	CHECK((rawSr() & kSr_EOP) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 a new wait starts from a clean error code") {
	// UM1725: FLASH_WaitForLastOperation then GetError. A new wait starts from a clean error code.
	HalFixture::poke(&FLASH->SR, kSr_WRPERR);
	REQUIRE(FLASH_WaitForLastOperation(10u) == HAL_ERROR);
	REQUIRE(HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP);
	REQUIRE(FLASH_WaitForLastOperation(10u) == HAL_OK);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_NONE);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 the write protection flag gives HAL_ERROR and the error code WRP") {
	HalFixture::poke(&FLASH->SR, kSr_WRPERR);
	CHECK(FLASH_WaitForLastOperation(10u) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP);
	CHECK((rawSr() & kSr_WRPERR) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 the alignment flag gives HAL_ERROR and the error code PGA") {
	HalFixture::poke(&FLASH->SR, kSr_PGAERR);
	CHECK(FLASH_WaitForLastOperation(10u) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGA);
	CHECK((rawSr() & kSr_PGAERR) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 the parallelism flag gives HAL_ERROR and the error code PGP") {
	HalFixture::poke(&FLASH->SR, kSr_PGPERR);
	CHECK(FLASH_WaitForLastOperation(10u) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGP);
	CHECK((rawSr() & kSr_PGPERR) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 the sequence flag gives HAL_ERROR and the error code PGS") {
	HalFixture::poke(&FLASH->SR, kSr_PGSERR);
	CHECK(FLASH_WaitForLastOperation(10u) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGS);
	CHECK((rawSr() & kSr_PGSERR) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 the operation error flag gives HAL_ERROR and the error code OPERATION") {
	HalFixture::poke(&FLASH->SR, kSr_OPERR);
	CHECK(FLASH_WaitForLastOperation(10u) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_OPERATION);
	CHECK((rawSr() & kSr_OPERR) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 several error flags give the combination of their error codes") {
	// 0x10 | 0x08 | 0x04 | 0x02 | 0x20 = 0x3E
	HalFixture::poke(&FLASH->SR, kSr_WRPERR | kSr_PGAERR | kSr_PGPERR | kSr_PGSERR | kSr_OPERR);
	CHECK(FLASH_WaitForLastOperation(10u) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == 0x3Eu);
	CHECK((rawSr() & 0xF3u) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 an error flag together with EOP gives HAL_ERROR and clears both") {
	HalFixture::poke(&FLASH->SR, kSr_EOP | kSr_PGSERR);
	CHECK(FLASH_WaitForLastOperation(10u) == HAL_ERROR);
	CHECK((rawSr() & 0xF3u) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 the error code is the same as HAL_FLASH_GetError reports until the next wait") {
	HalFixture::poke(&FLASH->SR, kSr_PGAERR);
	REQUIRE(FLASH_WaitForLastOperation(10u) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGA);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGA);
}

TEST_CASE_FIXTURE(FbFixture, "model: FB2 the HAL error code constants are the documented bit values") {
	// UM1725 / header: RD 0x01, PGS 0x02, PGP 0x04, PGA 0x08, WRP 0x10, OPERATION 0x20.
	CHECK(HAL_FLASH_ERROR_NONE == 0u);
	CHECK(HAL_FLASH_ERROR_RD == 0x01u);
	CHECK(HAL_FLASH_ERROR_PGS == 0x02u);
	CHECK(HAL_FLASH_ERROR_PGP == 0x04u);
	CHECK(HAL_FLASH_ERROR_PGA == 0x08u);
	CHECK(HAL_FLASH_ERROR_WRP == 0x10u);
	CHECK(HAL_FLASH_ERROR_OPERATION == 0x20u);
}

TEST_CASE_FIXTURE(FbFixture, "FB2 the error code starts at none") {
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_NONE);
}

// ------------------------------------------------------------------------------------------------ FB3 program IT

TEST_CASE_FIXTURE(FbFixture, "FB3 program IT returns HAL_OK and enables EOPIE and ERRIE") {
	unlockAll();
	CHECK(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x01u) == HAL_OK);
	CHECK((rawCr() & kCr_EOPIE) != 0u);
	CHECK((rawCr() & kCr_ERRIE) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 program IT sets the procedure state to program and remembers the address") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x08004010u, 0x01u) == HAL_OK);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_PROGRAM);
	CHECK(pFlash.Address == 0x08004010u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 program IT stores the word at once and leaves the operation busy") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0xDEADBEEFu) == HAL_OK);
	CHECK(wordAt(0x08000000u) == 0xDEADBEEFu);
	CHECK((rawSr() & kSr_BSY) != 0u);
	CHECK(hostFlashIrqPending() == 0u);
	CHECK(flashCounts.eop == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 program IT of a byte stores one byte") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_BYTE, 0x08000002u, 0x5Au) == HAL_OK);
	CHECK(wordAt(0x08000000u) == 0xFF5AFFFFu);
	CHECK(((rawCr() & kCr_PSIZE) >> 8) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 program IT of a half word stores two bytes") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_HALFWORD, 0x08000002u, 0x1234u) == HAL_OK);
	CHECK(wordAt(0x08000000u) == 0x1234FFFFu);
	CHECK(((rawCr() & kCr_PSIZE) >> 8) == 1u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 program IT of a double word stores both words") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x08000008u, 0x1122334455667788ull) == HAL_OK);
	CHECK(wordAt(0x08000008u) == 0x55667788u);
	CHECK(wordAt(0x0800000Cu) == 0x11223344u);
	CHECK(((rawCr() & kCr_PSIZE) >> 8) == 3u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 program IT of a word selects PSIZE x32") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x0u) == HAL_OK);
	CHECK(((rawCr() & kCr_PSIZE) >> 8) == 2u);
	CHECK((rawCr() & kCr_PG) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the end of a program raises the interrupt request and the handler reports the address") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x08010020u, 0xA5A5A5A5u) == HAL_OK);
	REQUIRE(hostFlashComplete() == 1);
	CHECK(hostFlashIrqPending() != 0u);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.eop == 1u);
	CHECK(flashCounts.error == 0u);
	CHECK(flashCounts.lastValue == 0x08010020u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the handler reports the address for each program width") {
	unlockAll();
	const uint32_t types[4] = {FLASH_TYPEPROGRAM_BYTE, FLASH_TYPEPROGRAM_HALFWORD, FLASH_TYPEPROGRAM_WORD, FLASH_TYPEPROGRAM_DOUBLEWORD};
	for (unsigned i = 0; i < 4; ++i) {
		const uint32_t a = 0x08020000u + 0x20u * i;
		INFO("type " << types[i]);
		REQUIRE(HAL_FLASH_Program_IT(types[i], a, 0x0u) == HAL_OK);
		REQUIRE(hostFlashComplete() == 1);
		HAL_FLASH_IRQHandler();
		REQUIRE(flashCounts.eop == i + 1u);
		CHECK(flashCounts.lastValue == a);
	}
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the handler clears the end of operation flag") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x0u) == HAL_OK);
	REQUIRE(hostFlashComplete() == 1);
	HAL_FLASH_IRQHandler();
	CHECK((rawSr() & kSr_EOP) == 0u);
	CHECK(hostFlashIrqPending() == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the handler returns the procedure state to none and disables the program interrupt bits") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x0u) == HAL_OK);
	REQUIRE(hostFlashComplete() == 1);
	HAL_FLASH_IRQHandler();
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
	CHECK((rawCr() & (kCr_EOPIE | kCr_ERRIE | kCr_PG)) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the end of operation callback sees the end of operation flag already cleared") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x0u) == HAL_OK);
	REQUIRE(hostFlashComplete() == 1);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.orderCount == 1u);
	CHECK((flashCounts.srOrder[0] & kSr_EOP) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 a second handler call after the end does nothing") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x0u) == HAL_OK);
	REQUIRE(hostFlashComplete() == 1);
	HAL_FLASH_IRQHandler();
	HAL_FLASH_IRQHandler();
	CHECK(flashCounts.eop == 1u);
	CHECK(flashCounts.error == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the handler does not call back before the operation has ended") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x0u) == HAL_OK);
	hostFlashSetPollReads(0); // the handler reads SR: the default model would complete the operation at that read
	HAL_FLASH_IRQHandler();
	CHECK(flashCounts.orderCount == 0u);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_PROGRAM);
	CHECK((rawCr() & (kCr_EOPIE | kCr_ERRIE)) == (kCr_EOPIE | kCr_ERRIE));
}

TEST_CASE_FIXTURE(FbFixture, "FB3 a program IT can follow a finished program IT") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x1u) == HAL_OK);
	REQUIRE(hostFlashComplete() == 1);
	HAL_FLASH_IRQHandler();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x08000010u, 0x2u) == HAL_OK);
	REQUIRE(hostFlashComplete() == 1);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.eop == 2u);
	CHECK(flashCounts.valueOrder[0] == 0x08000000u);
	CHECK(flashCounts.valueOrder[1] == 0x08000010u);
	CHECK(wordAt(0x08000010u) == 0x2u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 a write protection error in program IT raises the interrupt request with ERRIE and OPERR") {
	// RM0090 3.6.6: with ERRIE an error flag also sets OPERR.
	unlockAll();
	protectSector(2);
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, hostFlashSectorBase(2), 0x0u) == HAL_OK);
	CHECK((rawSr() & kSr_WRPERR) != 0u);
	CHECK((rawSr() & kSr_OPERR) != 0u);
	CHECK(hostFlashIrqPending() != 0u);
	CHECK(wordAt(hostFlashSectorBase(2)) == 0xFFFFFFFFu);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the handler reports a write protection error with the address and the error code") {
	unlockAll();
	protectSector(2);
	const uint32_t a = hostFlashSectorBase(2) + 8u;
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, a, 0x0u) == HAL_OK);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.error == 1u);
	CHECK(flashCounts.eop == 0u);
	CHECK(flashCounts.lastValue == a);
	CHECK(HAL_FLASH_GetError() == (HAL_FLASH_ERROR_WRP | HAL_FLASH_ERROR_OPERATION)); // WRPERR 0x10 and the OPERR the ERRIE set
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the error callback sees the error code already stored and the flags cleared") {
	unlockAll();
	protectSector(2);
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, hostFlashSectorBase(2), 0x0u) == HAL_OK);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.orderCount == 1u);
	CHECK((flashCounts.errorCodeOrder[0] & HAL_FLASH_ERROR_WRP) != 0u);
	CHECK((flashCounts.srOrder[0] & (kSr_WRPERR | kSr_OPERR)) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 after an error the handler stops the procedure and disables the interrupts") {
	unlockAll();
	protectSector(2);
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, hostFlashSectorBase(2), 0x0u) == HAL_OK);
	HAL_FLASH_IRQHandler();
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
	CHECK((rawCr() & (kCr_EOPIE | kCr_ERRIE | kCr_PG)) == 0u);
	CHECK(hostFlashIrqPending() == 0u);
	CHECK((rawSr() & 0xF3u) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 an alignment error in program IT is reported by the handler with the address" FB_MISALIGNED_STORE) {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x0800000Eu, 0x0u) == HAL_OK);
	CHECK(hostFlashIrqPending() != 0u);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.error == 1u);
	CHECK(flashCounts.lastValue == 0x0800000Eu);
	CHECK((HAL_FLASH_GetError() & HAL_FLASH_ERROR_PGA) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 program IT on a locked flash ends with a sequence error in the handler") {
	// CR is locked: EOPIE / ERRIE / PG cannot be set, the store sets PGSERR (RM0090 3.6.4); without ERRIE no OPERR and no interrupt request.
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x0u) == HAL_OK);
	CHECK(hostFlashIrqPending() == 0u);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.error == 1u);
	CHECK(flashCounts.lastValue == 0x08000000u);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGS);
	CHECK(wordAt(0x08000000u) == 0xFFFFFFFFu);
}

TEST_CASE_FIXTURE(FbFixture, "characterization: FB3 an error together with the end of operation calls the error callback first and then the end callback") {
	// NEEDS REVIEW 342/3: EOP and an error flag together cannot happen on hardware (RM0090: a failed store raises no EOP in the model, a started one ends without error); UM1725 does not say what happens when EOP and an error flag are both pending in one interrupt; the HAL
	// handles the error first, stops the procedure and then, as EOP is set, reports the end with the same address.
	setProcedure(FLASH_PROC_PROGRAM);
	pFlash.Address = 0x08000040u;
	HalFixture::poke(&FLASH->SR, kSr_EOP | kSr_PGSERR);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.orderCount == 2u);
	CHECK(flashCounts.order[0] == FLASH_CB_ERROR);
	CHECK(flashCounts.order[1] == FLASH_CB_EOP);
	CHECK(flashCounts.valueOrder[0] == 0x08000040u);
	CHECK(flashCounts.valueOrder[1] == 0x08000040u);
}

namespace {
void fbCheckIrqErrorFlag(uint32_t flag, uint32_t code) {
	pFlash.ProcedureOnGoing = FLASH_PROC_PROGRAM;
	pFlash.Address = 0x08001234u;
	HalFixture::poke(&FLASH->SR, flag);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.error == 1u);
	CHECK(flashCounts.eop == 0u);
	CHECK(flashCounts.lastValue == 0x08001234u);
	CHECK(HAL_FLASH_GetError() == code);
	CHECK((rawSr() & 0xF3u) == 0u);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
}
} // namespace

TEST_CASE_FIXTURE(FbFixture, "FB3 the handler maps the operation error flag to error code OPERATION") {
	fbCheckIrqErrorFlag(kSr_OPERR, HAL_FLASH_ERROR_OPERATION);
}
TEST_CASE_FIXTURE(FbFixture, "FB3 the handler maps the write protection flag to error code WRP") {
	fbCheckIrqErrorFlag(kSr_WRPERR, HAL_FLASH_ERROR_WRP);
}
TEST_CASE_FIXTURE(FbFixture, "FB3 the handler maps the alignment flag to error code PGA") {
	fbCheckIrqErrorFlag(kSr_PGAERR, HAL_FLASH_ERROR_PGA);
}
TEST_CASE_FIXTURE(FbFixture, "FB3 the handler maps the parallelism flag to error code PGP") {
	fbCheckIrqErrorFlag(kSr_PGPERR, HAL_FLASH_ERROR_PGP);
}
TEST_CASE_FIXTURE(FbFixture, "FB3 the handler maps the sequence flag to error code PGS") {
	fbCheckIrqErrorFlag(kSr_PGSERR, HAL_FLASH_ERROR_PGS);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 several error flags in one interrupt give one callback and the combined error code") {
	setProcedure(FLASH_PROC_PROGRAM);
	pFlash.Address = 0x08000100u;
	HalFixture::poke(&FLASH->SR, kSr_WRPERR | kSr_PGAERR);
	HAL_FLASH_IRQHandler();
	CHECK(flashCounts.error == 1u);
	CHECK(HAL_FLASH_GetError() == (HAL_FLASH_ERROR_WRP | HAL_FLASH_ERROR_PGA));
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the error code accumulates over two handler errors until a wait clears it") {
	setProcedure(FLASH_PROC_PROGRAM);
	HalFixture::poke(&FLASH->SR, kSr_WRPERR);
	HAL_FLASH_IRQHandler();
	setProcedure(FLASH_PROC_PROGRAM);
	HalFixture::poke(&FLASH->SR, kSr_PGSERR);
	HAL_FLASH_IRQHandler();
	CHECK(HAL_FLASH_GetError() == (HAL_FLASH_ERROR_WRP | HAL_FLASH_ERROR_PGS));
	CHECK(FLASH_WaitForLastOperation(10u) == HAL_OK);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_NONE);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the handler without a procedure and without flags only clears the program bits and the interrupt enables") {
	unlockAll();
	// PG, SER, MER, SNB = 0xF, EOPIE, ERRIE and PSIZE = 3 set by hand.
	HalFixture::poke(&FLASH->CR, kCleanedByIrq | kCr_EOPIE | kCr_ERRIE | kCr_PSIZE);
	pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
	HAL_FLASH_IRQHandler();
	CHECK(flashCounts.orderCount == 0u);
	CHECK((rawCr() & (kCleanedByIrq | kCr_EOPIE | kCr_ERRIE)) == 0u);
	CHECK((rawCr() & kCr_PSIZE) == kCr_PSIZE);
	CHECK((rawCr() & kCr_LOCK) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the handler with a program in progress and no flags changes nothing") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD, 0x08000000u, 0x0u) == HAL_OK);
	hostFlashSetPollReads(0);
	const uint32_t cr = rawCr();
	HAL_FLASH_IRQHandler();
	CHECK(rawCr() == cr);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_PROGRAM);
	CHECK(flashCounts.orderCount == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the handler reports the faulty sector of a sector erase error and invalidates it") {
	// UM1725 HAL_FLASH_OperationErrorCallback: Sectors Erase: Sector number which returned an error.
	setProcedure(FLASH_PROC_SECTERASE);
	pFlash.Sector = 6u;
	HalFixture::poke(&FLASH->SR, kSr_WRPERR);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.error == 1u);
	CHECK(flashCounts.lastValue == 6u);
	CHECK(pFlash.Sector == 0xFFFFFFFFu);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the handler reports the bank of a mass erase error") {
	// UM1725: Mass Erase: Bank number which has been requested to erase.
	setProcedure(FLASH_PROC_MASSERASE);
	pFlash.Bank = FLASH_BANK_1;
	HalFixture::poke(&FLASH->SR, kSr_WRPERR);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.error == 1u);
	CHECK(flashCounts.lastValue == FLASH_BANK_1);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the handler ends a sector erase with the callback value 0xFFFFFFFF after the last sector") {
	// UM1725 HAL_FLASH_EndOfOperationCallback: 0xFFFFFFFF means that all the selected sectors have been erased.
	setProcedure(FLASH_PROC_SECTERASE);
	pFlash.NbSectorsToErase = 1u;
	pFlash.Sector = 5u;
	HalFixture::poke(&FLASH->SR, kSr_EOP);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.eop == 1u);
	CHECK(flashCounts.lastValue == 0xFFFFFFFFu);
	CHECK(pFlash.Sector == 0xFFFFFFFFu);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
	CHECK(pFlash.NbSectorsToErase == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the handler reports each erased sector and starts the next one while sectors remain") {
	// UM1725: Sectors Erase: Sector which has been erased. Two sectors from 3: callback(3), then sector 4 is erased and callback(0xFFFFFFFF).
	unlockAll();
	hostFlashFill(hostFlashSectorBase(4), 0x00u, 16);
	setProcedure(FLASH_PROC_SECTERASE);
	pFlash.NbSectorsToErase = 2u;
	pFlash.Sector = 3u;
	pFlash.VoltageForErase = FLASH_PSIZE_WORD >> 8;
	HalFixture::poke(&FLASH->SR, kSr_EOP);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.eop == 1u);
	CHECK(flashCounts.valueOrder[0] == 3u);
	CHECK(pFlash.Sector == 4u);
	CHECK(pFlash.NbSectorsToErase == 1u);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_SECTERASE);
	CHECK(wordAt(hostFlashSectorBase(4)) == 0xFFFFFFFFu);
	CHECK((rawCr() & 0x2u) != 0u); // SER still set, the procedure goes on
	REQUIRE(hostFlashComplete() == 1);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.eop == 2u);
	CHECK(flashCounts.valueOrder[1] == 0xFFFFFFFFu);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the handler ends a mass erase with the requested bank as callback value") {
	setProcedure(FLASH_PROC_MASSERASE);
	pFlash.Bank = FLASH_BANK_1;
	HalFixture::poke(&FLASH->SR, kSr_EOP);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.eop == 1u);
	CHECK(flashCounts.lastValue == FLASH_BANK_1);
	CHECK(pFlash.ProcedureOnGoing == FLASH_PROC_NONE);
	CHECK((rawSr() & kSr_EOP) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB3 the end of a sector erase or mass erase disables the interrupt enables") {
	unlockAll();
	HalFixture::poke(&FLASH->CR, kCr_EOPIE | kCr_ERRIE);
	setProcedure(FLASH_PROC_MASSERASE);
	pFlash.Bank = FLASH_BANK_1;
	HalFixture::poke(&FLASH->SR, kSr_EOP);
	HAL_FLASH_IRQHandler();
	CHECK((rawCr() & (kCr_EOPIE | kCr_ERRIE)) == 0u);
}

// ------------------------------------------------------------------------------------------------ FB4 option byte launch

namespace {
// Unlocks OPTCR and sets USER = 0b001 (bits 7:5 = 0x20) and nWRP of sector 5 = 0 (protected), RDP stays 0xAA.
void fbPrepareOptionBytes() {
	REQUIRE(HAL_FLASH_OB_Unlock() == HAL_OK);
	FLASH->OPTCR = (rawOptcr() & ~0xE0u & ~(1u << 21)) | 0x20u;
}
} // namespace

TEST_CASE_FIXTURE(FbFixture, "FB4 option byte launch returns HAL_OK and ends with OPTSTRT and BSY clear") {
	fbPrepareOptionBytes();
	CHECK(HAL_FLASH_OB_Launch() == HAL_OK);
	CHECK((rawOptcr() & kOptcr_OPTSTRT) == 0u);
	CHECK((rawSr() & kSr_BSY) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB4 option byte launch commits RDP and USER to the option byte word at 0x1FFFC000") {
	// RM0090 Tables 15 / 16: word 0x1FFFC000 bits 15:8 RDP, bits 7:5 USER. RDP 0xAA, USER 0x20 -> bits 15:5 = 0xAA20.
	fbPrepareOptionBytes();
	REQUIRE(HAL_FLASH_OB_Launch() == HAL_OK);
	CHECK((wordAt(0x1FFFC000u) & 0xFFE0u) == 0xAA20u);
}

TEST_CASE_FIXTURE(FbFixture, "FB4 option byte launch commits nWRP to the option byte word at 0x1FFFC008") {
	// nWRP bits 11:0 of 0x1FFFC008; sector 5 protected: 0xFFF & ~(1 << 5) = 0xFDF.
	fbPrepareOptionBytes();
	REQUIRE(HAL_FLASH_OB_Launch() == HAL_OK);
	CHECK((wordAt(0x1FFFC008u) & 0xFFFu) == 0xFDFu);
}

TEST_CASE_FIXTURE(FbFixture, "model: FB4 option byte launch keeps the option bytes unchanged until it is called") {
	fbPrepareOptionBytes();
	CHECK((wordAt(0x1FFFC000u) & 0xFFE0u) == 0xAAE0u); // reset: USER bits 7:5 = 111
	CHECK((wordAt(0x1FFFC008u) & 0xFFFu) == 0xFFFu);
}

TEST_CASE_FIXTURE(FbFixture, "FB4 option byte launch keeps the OPTCR fields it did not touch") {
	fbPrepareOptionBytes();
	const uint32_t before = rawOptcr() & ~kOptcr_OPTSTRT;
	REQUIRE(HAL_FLASH_OB_Launch() == HAL_OK);
	CHECK((rawOptcr() & ~kOptcr_OPTSTRT) == before);
}

TEST_CASE_FIXTURE(FbFixture, "FB4 option byte launch sets OPTSTRT and leaves it set while BSY (timeout)") {
	fbPrepareOptionBytes();
	hostFlashSetPollReads(0);
	CHECK(HAL_FLASH_OB_Launch() == HAL_TIMEOUT);
	CHECK((rawOptcr() & kOptcr_OPTSTRT) != 0u);
	CHECK((rawSr() & kSr_BSY) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB4 option byte launch that timed out does not commit before the operation ends") {
	fbPrepareOptionBytes();
	hostFlashSetPollReads(0);
	REQUIRE(HAL_FLASH_OB_Launch() == HAL_TIMEOUT);
	CHECK((wordAt(0x1FFFC000u) & 0xFFE0u) == 0xAAE0u);
	REQUIRE(hostFlashComplete() == 1);
	CHECK((wordAt(0x1FFFC000u) & 0xFFE0u) == 0xAA20u);
	CHECK((rawOptcr() & kOptcr_OPTSTRT) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB4 option byte launch waits several SR reads for the end of the operation") {
	fbPrepareOptionBytes();
	hostFlashSetPollReads(6);
	CHECK(HAL_FLASH_OB_Launch() == HAL_OK);
	CHECK((wordAt(0x1FFFC008u) & 0xFFFu) == 0xFDFu);
}

TEST_CASE_FIXTURE(FbFixture, "FB4 option byte launch clears the end of operation flag") {
	fbPrepareOptionBytes();
	REQUIRE(HAL_FLASH_OB_Launch() == HAL_OK);
	CHECK((rawSr() & kSr_EOP) == 0u);
}

TEST_CASE_FIXTURE(FbFixture, "model: FB4 option byte launch does not touch CR.LOCK") {
	fbPrepareOptionBytes();
	REQUIRE(HAL_FLASH_OB_Launch() == HAL_OK);
	CHECK((rawCr() & kCr_LOCK) != 0u);
}

TEST_CASE_FIXTURE(FbFixture, "FB4 option byte launch with a pending error flag returns HAL_ERROR with that error code") {
	fbPrepareOptionBytes();
	HalFixture::poke(&FLASH->SR, kSr_WRPERR);
	CHECK(HAL_FLASH_OB_Launch() == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP);
}

TEST_CASE_FIXTURE(FbFixture, "characterization: FB4 option byte launch on a locked OPTCR returns HAL_OK and changes nothing") {
	// NEEDS REVIEW 342/4: UM1725 does not say what OB_Launch returns when OPTCR is locked. The HAL does not check OPTLOCK: the OPTSTRT write is
	// ignored by the hardware (RM0090 3.7.2 requires OPTLOCK = 0) and the following wait finds no operation.
	CHECK(HAL_FLASH_OB_Launch() == HAL_OK);
	CHECK((rawOptcr() & kOptcr_OPTSTRT) == 0u);
	CHECK((wordAt(0x1FFFC000u) & 0xFFE0u) == 0xAAE0u);
	CHECK(hostFlashLockedWrites() == 1u);
}

TEST_CASE_FIXTURE(FbFixture, "FB4 option byte launch is accepted for a second launch after the first")
{
	fbPrepareOptionBytes();
	REQUIRE(HAL_FLASH_OB_Launch() == HAL_OK);
	FLASH->OPTCR = (rawOptcr() & ~0xE0u) | 0x60u;
	CHECK(HAL_FLASH_OB_Launch() == HAL_OK);
	CHECK((wordAt(0x1FFFC000u) & 0xFFE0u) == 0xAA60u);
}

TEST_CASE_FIXTURE(FbFixture, "FB4 option byte launch commits a changed read protection byte") {
	// RDP 0xCC (level 2) written to OPTCR bits 15:8 and committed to bits 15:8 of the option word.
	REQUIRE(HAL_FLASH_OB_Unlock() == HAL_OK);
	FLASH->OPTCR = (rawOptcr() & ~0xFF00u) | 0xCC00u;
	REQUIRE(HAL_FLASH_OB_Launch() == HAL_OK);
	CHECK(((wordAt(0x1FFFC000u) >> 8) & 0xFFu) == 0xCCu);
}

// ------------------------------------------------------------------------------------------------ misc FB: model sanity visible through the HAL

TEST_CASE_FIXTURE(FbFixture, "model: FB2 programming through the HAL raises no register model fault") {
	unlockAll();
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x08000000u, 0x0ull) == HAL_OK);
	CHECK(HalFixture::faults() == 0u);
}
