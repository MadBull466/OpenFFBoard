/*
 * Tests of flash_helpers.cpp (USE_EEPROM_EMULATION and OTPMEMORY branches, F407 feature set) on the real
 * eeprom.c and the fake HAL FLASH: Flash_Init, Flash_Write, Flash_Read, Flash_ReadWriteDefault, Flash_ReadDefault,
 * Flash_Dump, Flash_Format, Flash_Write_Defaults, the data cache handling and locking around every operation, the
 * handling of HAL errors, and OTP_Write / OTP_Read.
 *
 * eeprom.c itself (page states, transfer details, power loss inside eeprom.c) is tested in test_eeprom.cpp.
 *
 * The flash mutex (flash_helpers.cpp:13 flashMutex) is not used: its Lock/Unlock calls are commented out
 * (flash_helpers.cpp:66, 86, 96, 98), so there is nothing to observe.
 *
 * Host vs target: OTP_Write / OTP_Read compute 32-bit addresses (uint32_t) and dereference them as uint64_t; the
 * fake maps the OTP area at its target address, so the arithmetic is the same as on the chip.
 */
#include "doctest.h"
#include "FlashFixture.h"
#include "TestHelpers.h"

#include <optional>
#include <tuple>
#include <vector>

namespace doctest {
/** Prints the result of read() in failed assertions */
template <> struct StringMaker<std::optional<uint16_t>> {
	static String convert(const std::optional<uint16_t> &v) { return v ? toString((unsigned)*v) : String("missing"); }
};
} // namespace doctest

namespace {

/** The error flags of FLASH_SR that make the next flash operation fail when left pending (EOP is no error) */
const std::vector<uint32_t> errorFlags = {FLASH_FLAG_OPERR, FLASH_FLAG_WRPERR, FLASH_FLAG_PGAERR, FLASH_FLAG_PGPERR,
										  FLASH_FLAG_PGSERR};

bool dataCacheEnabled() { return (FLASH->ACR & FLASH_ACR_DCEN) != 0; }

/** The stored value of a variable, empty if Flash_Read does not find it (use inside CHECK) */
std::optional<uint16_t> read(uint16_t adr) {
	uint16_t v = 0;
	if (!Flash_Read(adr, &v)) {
		return std::nullopt;
	}
	return v;
}

/** Address of the 8 byte OTP slot `offset` as OTP_Write / OTP_Read compute it */
uint32_t otpSlot(uint32_t offset) { return HostFlash::OTP_BASE + offset * 8; }

using Dump = std::vector<std::tuple<uint16_t, uint16_t>>;

} // namespace

TEST_SUITE("FlashHelpers") {

// ---------------------------------------------------------------- Flash_Init

TEST_CASE_FIXTURE(FlashFixture, "Flash_Init leaves the flash locked and the data cache reset and enabled") {
	CHECK(init());

	CHECK(HostFlash::locked());
	CHECK(HostFlash::state().unlocks == 1);
	CHECK(HostFlash::state().locks == 1);
	CHECK(HostFlash::state().dataCacheResets == 1);
	CHECK(HostFlash::state().ignoredDataCacheResets == 0); // reset while the cache was disabled (RM0090 3.9.1)
	CHECK(dataCacheEnabled());
}

TEST_CASE_FIXTURE(FlashFixture, "Flash_Init keeps the stored variables of a valid page") {
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 0x1234}});

	CHECK(init());

	CHECK(read(ADR_CURRENT_CONFIG) == 0x1234);
	CHECK(HostFlash::state().programsApplied == 0);
	CHECK(HostFlash::state().erasesApplied == 0);
}

TEST_CASE_FIXTURE(FlashFixture, "Flash_Init clears an error flag left by an earlier operation before repairing the pages") {
	for (uint32_t flag : errorFlags) {
		CAPTURE(flag);
		resetIteration(); // blank flash: EE_Init formats (programs the page 0 header)
		HostFlash::setPendingFlags(flag);

		CHECK(init());

		CHECK(pageStatus(0) == VALID_PAGE);
	}
}

TEST_CASE_FIXTURE(FlashFixture, "Flash_Init reports failure and locks the flash when the pages can not be repaired") {
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});
	writePage(1, VALID_PAGE); // two valid pages: EE_Init formats
	HostFlash::failNext(1, FLASH_FLAG_WRPERR, HostFlash::Target::Erase);

	CHECK_FALSE(init());

	CHECK(HostFlash::locked());
	CHECK(dataCacheEnabled());
}

TEST_CASE_FIXTURE(FlashFixture, "Flash_Init reports failure when the flash can not be unlocked and the pages need programming") {
	// Defensive path: on the chip a failing unlock is a bus error (model assumption, see HostFlash::setUnlockFails),
	// so this is probably unreachable without a fault.
	HostFlash::setUnlockFails(true); // blank flash: the format has to program the page 0 header

	CHECK_FALSE(init());

	CHECK(pageStatus(0) == ERASED);
	CHECK(HostFlash::locked());
}

TEST_CASE_FIXTURE(FlashFixture, "characterization: Flash_Init reports success when the flash can not be unlocked and only an erase was needed") {
	// NEEDS REVIEW 43: Flash_Init ignores the result of HAL_FLASH_Unlock. Page 0 ERASED but dirty, page 1 VALID:
	// EE_Init only erases page 0. An erase while locked is ignored without an error flag (model assumption inferred
	// from RM0090 3.9.7, HAL_FLASHEx_Erase returns HAL_OK), so the repair silently does not happen.
	// Defensive path: on the chip a failing unlock is a bus error (model assumption, see HostFlash::setUnlockFails),
	// so this state is probably unreachable without a fault.
	writePage(0, ERASED, {{ADR_HW_VERSION, 0x0BAD}});
	writePage(1, VALID_PAGE, {{ADR_CURRENT_CONFIG, 0x1234}});
	HostFlash::setUnlockFails(true);

	CHECK(init());

	CHECK(HostFlash::state().erasesApplied == 0);
	CHECK_FALSE(HostFlash::isErased(HostFlash::EEPROM_PAGE0 + 4, 4)); // the stale record is still there
	CHECK(read(ADR_CURRENT_CONFIG) == 0x1234);
}

// ---------------------------------------------------------------- Flash_Write

TEST_CASE_FIXTURE(FlashFixture, "writing a variable that is not stored yet stores it and returns true") {
	REQUIRE(init());

	CHECK(Flash_Write(ADR_FFB_EFFECTS1, 0x0102));

	CHECK(read(ADR_FFB_EFFECTS1) == 0x0102);
	CHECK(usedRecords(0) == 1);
}

TEST_CASE_FIXTURE(FlashFixture, "writing a different value appends a record with the new value and returns true") {
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_FFB_EFFECTS1, 1));

	CHECK(Flash_Write(ADR_FFB_EFFECTS1, 2));

	CHECK(read(ADR_FFB_EFFECTS1) == 2);
	auto records = pageRecords(0);
	REQUIRE(records.size() == 2);
	CHECK(records[1].address == ADR_FFB_EFFECTS1);
	CHECK(records[1].value == 2);
}

TEST_CASE_FIXTURE(FlashFixture, "writing the stored value again programs nothing, does not unlock the flash and returns false") {
	// flash_helpers.cpp:63 "Returns true on success or false if variable is the same or error"
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_FFB_EFFECTS1, 7));
	const auto before = HostFlash::state();

	CHECK_FALSE(Flash_Write(ADR_FFB_EFFECTS1, 7));

	CHECK(HostFlash::state().programCalls == before.programCalls);
	CHECK(HostFlash::state().unlocks == before.unlocks);
	CHECK(HostFlash::state().dataCacheResets == before.dataCacheResets);
	CHECK(usedRecords(0) == 1);
	CHECK(HostFlash::locked());
}

TEST_CASE_FIXTURE(FlashFixture, "writing a value of 0xFFFF to a missing variable stores it") {
	REQUIRE(init());

	CHECK(Flash_Write(ADR_FFB_EFFECTS1, 0xFFFF));

	CHECK(usedRecords(0) == 1);
	CHECK(read(ADR_FFB_EFFECTS1) == 0xFFFF);
}

TEST_CASE_FIXTURE(FlashFixture, "Flash_Write without a valid page writes nothing and returns false") {
	// Blank flash without Flash_Init: EE_ReadVariable reports NO_VALID_PAGE
	CHECK_FALSE(Flash_Write(ADR_CURRENT_CONFIG, 1));

	CHECK(HostFlash::state().programCalls == 0);
	CHECK(HostFlash::state().unlocks == 0);
	CHECK(HostFlash::isErased(HostFlash::EEPROM_PAGE0, 2 * HostFlash::EEPROM_PAGE_SIZE));
}

TEST_CASE_FIXTURE(FlashFixture, "Flash_Write leaves the flash locked and resets the data cache before and after writing") {
	REQUIRE(init());
	const auto before = HostFlash::state();

	REQUIRE(Flash_Write(ADR_CURRENT_CONFIG, 1));

	CHECK(HostFlash::locked());
	CHECK(HostFlash::state().unlocks == before.unlocks + 1);
	CHECK(HostFlash::state().locks == before.locks + 1);
	CHECK(HostFlash::state().dataCacheResets == before.dataCacheResets + 2);
	CHECK(HostFlash::state().ignoredDataCacheResets == 0);
	CHECK(dataCacheEnabled());
}

TEST_CASE_FIXTURE(FlashFixture, "Flash_Write clears an error flag left by an earlier operation before writing") {
	for (uint32_t flag : errorFlags) {
		CAPTURE(flag);
		resetIteration();
		REQUIRE(init());
		HostFlash::setPendingFlags(flag);

		CHECK(Flash_Write(ADR_CURRENT_CONFIG, 0x55));

		CHECK(read(ADR_CURRENT_CONFIG) == 0x55);
	}
}

TEST_CASE_FIXTURE(FlashFixture, "a failed program operation makes Flash_Write return false, keeps the old value and locks the flash") {
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_CURRENT_CONFIG, 1));
	HostFlash::failNext(1, FLASH_FLAG_PGPERR, HostFlash::Target::Program);

	CHECK_FALSE(Flash_Write(ADR_CURRENT_CONFIG, 2));

	CHECK(read(ADR_CURRENT_CONFIG) == 1);
	CHECK(HostFlash::locked());
	CHECK(dataCacheEnabled());
}

TEST_CASE_FIXTURE(FlashFixture, "Flash_Write returns false and keeps the old value when the flash can not be unlocked") {
	// Defensive path: on the chip a failing unlock is a bus error (model assumption, see HostFlash::setUnlockFails)
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_CURRENT_CONFIG, 1));
	HostFlash::setUnlockFails(true);
	const uint32_t programsBefore = HostFlash::state().programsApplied;

	CHECK_FALSE(Flash_Write(ADR_CURRENT_CONFIG, 2));

	CHECK(read(ADR_CURRENT_CONFIG) == 1);
	CHECK(HostFlash::state().programsApplied == programsBefore);
}

TEST_CASE_FIXTURE(FlashFixture, "a write into a full page moves the variables to the other page and returns true") {
	fillPage(0, VALID_PAGE, RECORDS_PER_PAGE, {ADR_HW_VERSION, ADR_SW_VERSION}); // value = record index

	CHECK(Flash_Write(ADR_CURRENT_CONFIG, 0x77));

	CHECK(pageStatus(1) == VALID_PAGE);
	CHECK(pageStatus(0) == ERASED);
	CHECK(read(ADR_CURRENT_CONFIG) == 0x77);
	CHECK(read(ADR_HW_VERSION) == RECORDS_PER_PAGE - 1); // last record index 4094 (even)
	CHECK(read(ADR_SW_VERSION) == RECORDS_PER_PAGE - 2);
	CHECK(HostFlash::locked());
}

TEST_CASE_FIXTURE(FlashFixture, "Flash_Write returns false when the page transfer fails") {
	fillPage(0, VALID_PAGE, RECORDS_PER_PAGE, {ADR_HW_VERSION, ADR_SW_VERSION});
	HostFlash::failNext(1, FLASH_FLAG_WRPERR, HostFlash::Target::Erase); // erase of the old page

	CHECK_FALSE(Flash_Write(ADR_CURRENT_CONFIG, 0x77));

	CHECK(HostFlash::locked());
}

// ---------------------------------------------------------------- Flash_Read

TEST_CASE_FIXTURE(FlashFixture, "reading a variable that was never written returns false and leaves the buffer unchanged") {
	REQUIRE(init());
	uint16_t v = 0x4242;

	CHECK_FALSE(Flash_Read(ADR_CURRENT_CONFIG, &v));

	CHECK(v == 0x4242);
}

TEST_CASE_FIXTURE(FlashFixture, "reading returns the newest value of a variable") {
	REQUIRE(init());
	for (uint16_t value : {10, 20, 30}) {
		REQUIRE(Flash_Write(ADR_CURRENT_CONFIG, value));
		REQUIRE(Flash_Write(ADR_HW_VERSION, (uint16_t)(value + 1)));
	}
	uint16_t v = 0;

	CHECK(Flash_Read(ADR_CURRENT_CONFIG, &v));

	CHECK(v == 30);
}

TEST_CASE_FIXTURE(FlashFixture, "reading without a valid page returns false") {
	uint16_t v = 0x4242;

	CHECK_FALSE(Flash_Read(ADR_CURRENT_CONFIG, &v));

	CHECK(v == 0x4242);
}

TEST_CASE_FIXTURE(FlashFixture, "reading does not unlock, program or lock the flash") {
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 3}});
	uint16_t v = 0;

	REQUIRE(Flash_Read(ADR_CURRENT_CONFIG, &v));

	CHECK(HostFlash::state().unlocks == 0);
	CHECK(HostFlash::state().locks == 0);
	CHECK(HostFlash::state().programCalls == 0);
}

TEST_CASE_FIXTURE(FlashFixture, "characterization: checkempty has no effect with the EEPROM emulation, a stored 0xFFFF is found") {
	// The I2C EEPROM variant (flash_helpers.cpp:186) reports a 0xFFFF value as missing when checkempty is set.
	// With the EEPROM emulation a stored 0xFFFF is a real value. NEEDS REVIEW 46: should both backends agree?
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 0xFFFF}});
	uint16_t v = 0;

	CHECK(Flash_Read(ADR_CURRENT_CONFIG, &v, true));
	CHECK(v == 0xFFFF);
}

// ---------------------------------------------------------------- Flash_ReadWriteDefault / Flash_ReadDefault

TEST_CASE_FIXTURE(FlashFixture, "a missing variable gets the default in the buffer and in the flash and the call returns false") {
	REQUIRE(init());
	uint16_t v = 0;

	CHECK_FALSE(Flash_ReadWriteDefault(ADR_FFB_EFFECTS2, &v, 0x0A0B));

	CHECK(v == 0x0A0B);
	CHECK(read(ADR_FFB_EFFECTS2) == 0x0A0B);
	CHECK(HostFlash::locked());
}

TEST_CASE_FIXTURE(FlashFixture, "an existing variable is returned unchanged, nothing is written and the call returns true") {
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_FFB_EFFECTS2, 0x1111));
	const auto before = HostFlash::state();
	uint16_t v = 0;

	CHECK(Flash_ReadWriteDefault(ADR_FFB_EFFECTS2, &v, 0x2222));

	CHECK(v == 0x1111);
	CHECK(HostFlash::state().programCalls == before.programCalls);
	CHECK(HostFlash::state().unlocks == before.unlocks);
	CHECK(read(ADR_FFB_EFFECTS2) == 0x1111);
}

TEST_CASE_FIXTURE(FlashFixture, "writing the default resets the data cache before and after writing and locks the flash") {
	REQUIRE(init());
	const auto before = HostFlash::state();
	uint16_t v = 0;

	Flash_ReadWriteDefault(ADR_FFB_EFFECTS2, &v, 1);

	CHECK(HostFlash::locked());
	CHECK(HostFlash::state().locks == before.locks + 1);
	CHECK(HostFlash::state().dataCacheResets == before.dataCacheResets + 2);
	CHECK(HostFlash::state().ignoredDataCacheResets == 0);
	CHECK(dataCacheEnabled());
}

TEST_CASE_FIXTURE(FlashFixture, "Flash_ReadWriteDefault clears an error flag left by an earlier operation before writing the default") {
	for (uint32_t flag : errorFlags) {
		CAPTURE(flag);
		resetIteration();
		REQUIRE(init());
		HostFlash::setPendingFlags(flag);
		uint16_t v = 0;

		Flash_ReadWriteDefault(ADR_FFB_EFFECTS2, &v, 0x0C0D);

		CHECK(read(ADR_FFB_EFFECTS2) == 0x0C0D);
	}
}

TEST_CASE_FIXTURE(FlashFixture, "characterization: Flash_ReadWriteDefault returns the default and false also when storing the default fails") {
	// NEEDS REVIEW 42: the result of EE_WriteVariable is ignored (flash_helpers.cpp:117); the caller can not tell
	// "missing, default stored" from "missing, default lost".
	REQUIRE(init());
	HostFlash::failNext(1, FLASH_FLAG_PGPERR, HostFlash::Target::Program);
	uint16_t v = 0;

	CHECK_FALSE(Flash_ReadWriteDefault(ADR_FFB_EFFECTS2, &v, 0x0A0B));

	CHECK(v == 0x0A0B);
	uint16_t stored = 0;
	CHECK_FALSE(Flash_Read(ADR_FFB_EFFECTS2, &stored)); // the default was not stored
	CHECK(HostFlash::locked());
}

TEST_CASE_FIXTURE(FlashFixture, "Flash_ReadDefault returns the stored value or the default without writing it") {
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_FFB_EFFECTS3, 5));
	const auto before = HostFlash::state();

	CHECK(Flash_ReadDefault<uint16_t>(ADR_FFB_EFFECTS3, 9) == 5);
	CHECK(Flash_ReadDefault<int>(ADR_FFB_CF_FILTER, -3) == -3);

	CHECK(HostFlash::state().programCalls == before.programCalls);
	uint16_t v = 0;
	CHECK_FALSE(Flash_Read(ADR_FFB_CF_FILTER, &v));
}

TEST_CASE_FIXTURE(FlashFixture, "pack and unpack combine and split the high and the low byte of a half-word") {
	CHECK(pack(0x12, 0x34) == 0x1234);
	CHECK(unpack(0xABCD) == std::tuple<uint8_t, uint8_t>{0xAB, 0xCD});
	auto [hb, lb] = unpack(pack(0xFE, 0x01));
	CHECK(hb == 0xFE);
	CHECK(lb == 0x01);
}

// ---------------------------------------------------------------- Flash_Dump

TEST_CASE_FIXTURE(FlashFixture, "dumping an empty flash adds nothing") {
	REQUIRE(init());
	Dump dump;

	Flash_Dump(&dump);
	Flash_Dump(&dump, true);

	CHECK(dump.empty());
}

TEST_CASE_FIXTURE(FlashFixture, "the dump lists the stored exportable variables as (address, value) in the order of the export table") {
	REQUIRE(init());
	// Written in reverse table order
	REQUIRE(Flash_Write(ADR_AXIS1_POWER, 0x00C8));
	REQUIRE(Flash_Write(ADR_FFB_EFFECTS1, 0x0102));
	REQUIRE(Flash_Write(ADR_CURRENT_CONFIG, 0x0001));
	Dump expected;
	for (uint32_t i = 0; i < NB_EXPORTABLE_ADR; i++) {
		uint16_t adr = exportableFlashAddresses[i];
		if (adr == ADR_CURRENT_CONFIG) expected.emplace_back(adr, 0x0001);
		if (adr == ADR_FFB_EFFECTS1) expected.emplace_back(adr, 0x0102);
		if (adr == ADR_AXIS1_POWER) expected.emplace_back(adr, 0x00C8);
	}
	REQUIRE(expected.size() == 3);
	Dump dump;

	Flash_Dump(&dump);

	CHECK(dump == expected);
}

TEST_CASE_FIXTURE(FlashFixture, "the dump leaves out the board-specific variables unless includeAll is set") {
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_HW_VERSION, 0x0003));
	REQUIRE(Flash_Write(ADR_TMC1_ADC_I0_OFS, 0x8000));
	Dump exported;
	Dump all;

	Flash_Dump(&exported);
	Flash_Dump(&all, true);

	CHECK(exported.empty());
	CHECK(all == Dump{{ADR_HW_VERSION, 0x0003}, {ADR_TMC1_ADC_I0_OFS, 0x8000}});
}

TEST_CASE_FIXTURE(FlashFixture, "the dump with includeAll lists every stored variable of VirtAddVarTab in table order") {
	REQUIRE(init());
	Dump expected;
	for (uint32_t i = 0; i < NB_OF_VAR; i++) {
		REQUIRE(Flash_Write(VirtAddVarTab[i], (uint16_t)(i * 3)));
		expected.emplace_back(VirtAddVarTab[i], (uint16_t)(i * 3));
	}
	Dump dump;

	Flash_Dump(&dump, true);

	CHECK(dump == expected);
}

TEST_CASE_FIXTURE(FlashFixture, "the dump leaves out stored addresses that are not in the address table") {
	REQUIRE(init());
	REQUIRE(Flash_Write(0x7FFE, 1)); // eeprom.c stores any address
	Dump dump;

	Flash_Dump(&dump, true);

	CHECK(dump.empty());
}

TEST_CASE_FIXTURE(FlashFixture, "characterization: Flash_Dump appends to the entries already in the result") {
	// NEEDS REVIEW 47: Flash_Dump push_backs into the caller's vector without clearing it. Intended (callers pass
	// an empty vector) or should it clear?
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_CURRENT_CONFIG, 2));
	Dump dump = {{0x1234, 0x5678}};

	Flash_Dump(&dump);

	CHECK(dump == Dump{{0x1234, 0x5678}, {ADR_CURRENT_CONFIG, 2}});
}

// ---------------------------------------------------------------- Flash_Format

TEST_CASE_FIXTURE(FlashFixture, "Flash_Format deletes all variables, leaves page 0 valid and returns true") {
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_CURRENT_CONFIG, 1));
	REQUIRE(Flash_Write(ADR_HW_VERSION, 2));

	CHECK(Flash_Format());

	uint16_t v = 0;
	CHECK_FALSE(Flash_Read(ADR_CURRENT_CONFIG, &v));
	CHECK_FALSE(Flash_Read(ADR_HW_VERSION, &v));
	CHECK(pageStatus(0) == VALID_PAGE);
	CHECK(usedRecords(0) == 0);
	CHECK(pageStatus(1) == ERASED);
	CHECK(Flash_Write(ADR_CURRENT_CONFIG, 3)); // usable again
	CHECK(read(ADR_CURRENT_CONFIG) == 3);
}

TEST_CASE_FIXTURE(FlashFixture, "Flash_Format leaves the flash locked and the data cache reset and enabled") {
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_CURRENT_CONFIG, 1));
	const auto before = HostFlash::state();

	REQUIRE(Flash_Format());

	CHECK(HostFlash::locked());
	CHECK(HostFlash::state().unlocks == before.unlocks + 1);
	CHECK(HostFlash::state().locks == before.locks + 1);
	CHECK(HostFlash::state().dataCacheResets == before.dataCacheResets + 1);
	CHECK(HostFlash::state().ignoredDataCacheResets == 0);
	CHECK(dataCacheEnabled());
}

TEST_CASE_FIXTURE(FlashFixture, "Flash_Format returns false and locks the flash when an erase fails") {
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_CURRENT_CONFIG, 1));
	HostFlash::failNext(1, FLASH_FLAG_WRPERR, HostFlash::Target::Erase);

	CHECK_FALSE(Flash_Format());

	CHECK(HostFlash::locked());
	CHECK(dataCacheEnabled());
}

TEST_CASE_FIXTURE(FlashFixture, "Flash_Format returns false and deletes nothing when the flash can not be unlocked") {
	// Defensive path: on the chip a failing unlock is a bus error (model assumption, see HostFlash::setUnlockFails)
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_CURRENT_CONFIG, 1));
	HostFlash::setUnlockFails(true);

	CHECK_FALSE(Flash_Format());

	CHECK(read(ADR_CURRENT_CONFIG) == 1);
}

TEST_CASE_FIXTURE(FlashFixture, "characterization: Flash_Format does not clear an error flag left by an earlier operation and fails once") {
	// NEEDS REVIEW 44: Flash_Init, Flash_Write and Flash_ReadWriteDefault clear the FLASH_SR error flags before
	// using the flash (workaround for the TODO at flash_helpers.cpp:16), Flash_Format does not. The HAL clears the
	// flags when it reports them, so a second call works.
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_CURRENT_CONFIG, 1));
	HostFlash::setPendingFlags(FLASH_FLAG_PGSERR);

	CHECK_FALSE(Flash_Format());
	CHECK(read(ADR_CURRENT_CONFIG) == 1);

	CHECK(Flash_Format());
	uint16_t v = 0;
	CHECK_FALSE(Flash_Read(ADR_CURRENT_CONFIG, &v));
}

TEST_CASE_FIXTURE(FlashFixture, "a variable written after Flash_Format following a page transfer is read back"
		* KNOWN_ISSUE("bug V5: EE_VerifyPageFullyErased (eeprom.c:333-356) reports page 1 as erased, so EE_Format "
					  "(called by Flash_Format, flash_helpers.cpp:38) leaves page 1 VALID next to the new valid page 0; "
					  "writes then go to page 1 (EE_FindValidPage WRITE) while reads use page 0")) {
	// State after a page transfer: page 0 erased, page 1 valid
	writePage(0, ERASED);
	writePage(1, VALID_PAGE, {{ADR_CURRENT_CONFIG, 5}});
	REQUIRE(Flash_Format());
	REQUIRE(Flash_Write(ADR_FLASH_VERSION, 0x42));
	REQUIRE_FLASH_ACCESSES_MODELLED();

	uint16_t v = 0;
	CHECK(Flash_Read(ADR_FLASH_VERSION, &v)); // fails: the record is in page 1, reads use page 0
	CHECK(v == 0x42);
	CHECK(pageStatus(1) == ERASED);
}

// ---------------------------------------------------------------- Flash_Write_Defaults

TEST_CASE_FIXTURE(FlashFixture, "writing the factory defaults changes nothing when no defaults are defined") {
	REQUIRE(init());
	REQUIRE(flash_factory_defaults.empty());
	const auto before = HostFlash::state();

	Flash_Write_Defaults();

	CHECK(HostFlash::state().programCalls == before.programCalls);
	CHECK(HostFlash::state().unlocks == before.unlocks);
}

// ---------------------------------------------------------------- OTP

TEST_CASE_FIXTURE(FlashFixture, "an OTP slot that was never written reads as all ones") {
	uint64_t v = 0;

	CHECK(OTP_Read(0, &v));

	CHECK(v == 0xFFFFFFFFFFFFFFFFULL);
}

TEST_CASE_FIXTURE(FlashFixture, "a value written to an OTP slot is read back and the flash is locked again") {
	CHECK(OTP_Write(3, 0x1122334455667788ULL));

	uint64_t v = 0;
	CHECK(OTP_Read(3, &v));
	CHECK(v == 0x1122334455667788ULL);
	CHECK(HostFlash::read64(otpSlot(3)) == 0x1122334455667788ULL); // slot 3 = bytes 24..31 of block 0
	CHECK(HostFlash::state().otpPrograms == 2);					   // two words
	CHECK(HostFlash::locked());
}

TEST_CASE_FIXTURE(FlashFixture, "a second write to a programmed OTP slot is rejected and the first value stays") {
	REQUIRE(OTP_Write(5, 0x00000000FFFF0000ULL));
	const auto before = HostFlash::state();

	CHECK_FALSE(OTP_Write(5, 0x0000000000000000ULL)); // would only clear bits, still rejected

	uint64_t v = 0;
	CHECK(OTP_Read(5, &v));
	CHECK(v == 0x00000000FFFF0000ULL);
	CHECK(HostFlash::state().programCalls == before.programCalls);
}

TEST_CASE_FIXTURE(FlashFixture, "writing an OTP slot leaves the neighbouring slots erased") {
	REQUIRE(OTP_Write(10, 0));

	uint64_t v = 0;
	CHECK(OTP_Read(9, &v));
	CHECK(v == 0xFFFFFFFFFFFFFFFFULL);
	CHECK(OTP_Read(11, &v));
	CHECK(v == 0xFFFFFFFFFFFFFFFFULL);
	CHECK(HostFlash::read64(otpSlot(10)) == 0);
}

TEST_CASE_FIXTURE(FlashFixture, "the last slot of the OTP data area (offset 63) can be written and read") {
	CHECK(OTP_Write(63, 0x0123456789ABCDEFULL));

	uint64_t v = 0;
	CHECK(OTP_Read(63, &v));
	CHECK(v == 0x0123456789ABCDEFULL);
	CHECK(HostFlash::read64(HostFlash::OTP_LOCK_BASE - 8) == 0x0123456789ABCDEFULL);
	CHECK(HostFlash::isErased(HostFlash::OTP_LOCK_BASE, 16)); // no block locked
}

TEST_CASE_FIXTURE(FlashFixture, "OTP offsets after the end of the OTP area are rejected by write and read") {
	for (uint16_t offset : {(uint16_t)66, (uint16_t)100, (uint16_t)0xFFFF}) {
		CAPTURE(offset);
		uint64_t v = 0x5A5A;

		CHECK_FALSE(OTP_Write(offset, 0));
		CHECK_FALSE(OTP_Read(offset, &v));

		CHECK(v == 0x5A5A);
	}
	CHECK(HostFlash::state().programCalls == 0);
	CHECK(HostFlash::state().unlocks == 0);
}

TEST_CASE_FIXTURE(FlashFixture, "an OTP write at offset 64 is rejected because it would reach the lock bytes of blocks 0 to 7"
		* KNOWN_ISSUE("bug 41: OTP_Write checks adr > FLASH_OTP_END (flash_helpers.cpp:302), the end of the lock "
					  "bytes, not of the 512 byte data area: offset 64 programs the lock bytes 0x1FFF7A00..07")) {
	const bool written = OTP_Write(64, 0);
	REQUIRE_FLASH_ACCESSES_MODELLED();

	CHECK_FALSE(written); // fails: the write is accepted
	CHECK(HostFlash::isErased(HostFlash::OTP_LOCK_BASE, 16));
}

TEST_CASE_FIXTURE(FlashFixture, "an OTP write at offset 65 is rejected because it would reach the lock bytes of blocks 8 to 15"
		* KNOWN_ISSUE("bug 41: OTP_Write checks adr > FLASH_OTP_END (flash_helpers.cpp:302): offset 65 programs the "
					  "lock bytes 0x1FFF7A08..0F")) {
	const bool written = OTP_Write(65, 0);
	REQUIRE_FLASH_ACCESSES_MODELLED();

	CHECK_FALSE(written); // fails: the write is accepted
	CHECK(HostFlash::isErased(HostFlash::OTP_LOCK_BASE, 16));
}

TEST_CASE_FIXTURE(FlashFixture, "an OTP read at offset 64 is rejected because it is past the OTP data area"
		* KNOWN_ISSUE("bug 41: OTP_Read checks adr > FLASH_OTP_END (flash_helpers.cpp:320): offsets 64 and 65 return "
					  "the lock bytes as data")) {
	uint64_t v = 0x5A5A;
	const bool read64 = OTP_Read(64, &v);
	REQUIRE_FLASH_ACCESSES_MODELLED();

	CHECK_FALSE(read64); // fails: the lock bytes are returned
	CHECK(v == 0x5A5A);
}

TEST_CASE_FIXTURE(FlashFixture, "an OTP write fails and programs nothing when the flash can not be unlocked") {
	// Defensive path (flash_helpers.cpp:309): on the chip a failing unlock is a bus error (model assumption, see
	// HostFlash::setUnlockFails), so this is probably unreachable without a fault.
	HostFlash::setUnlockFails(true);

	CHECK_FALSE(OTP_Write(0, 0x1234));

	CHECK(HostFlash::state().programCalls == 0);
	CHECK(HostFlash::isErased(otpSlot(0), 8));
	CHECK(HostFlash::locked());
}

TEST_CASE_FIXTURE(FlashFixture, "an OTP write reports failure when programming the first word fails") {
	HostFlash::failNext(1, FLASH_FLAG_PGPERR, HostFlash::Target::Program);

	CHECK_FALSE(OTP_Write(2, 0x1111111122222222ULL));

	CHECK(HostFlash::locked());
}

TEST_CASE_FIXTURE(FlashFixture, "an OTP write reports failure when programming the second word fails") {
	HostFlash::failNextAfter(1, 1, FLASH_FLAG_PGPERR, HostFlash::Target::Program); // low word works, high word fails

	CHECK_FALSE(OTP_Write(2, 0x1111111122222222ULL));

	CHECK(HostFlash::read32(otpSlot(2)) == 0x22222222U); // the low word was programmed
	CHECK(HostFlash::read32(otpSlot(2) + 4) == 0xFFFFFFFFU);
	CHECK(HostFlash::locked());
}

TEST_CASE_FIXTURE(FlashFixture, "characterization: after a failed first word the second word of the OTP slot is still programmed") {
	// NEEDS REVIEW 45: OTP_Write always programs the high word (flash_helpers.cpp:312), also after the low word
	// failed, so the slot is half written and can never be written again (curval != all ones). Stop after the
	// first failure instead?
	HostFlash::failNext(1, FLASH_FLAG_PGPERR, HostFlash::Target::Program);
	REQUIRE_FALSE(OTP_Write(2, 0x1111111122222222ULL));

	CHECK(HostFlash::read32(otpSlot(2)) == 0xFFFFFFFFU);
	CHECK(HostFlash::read32(otpSlot(2) + 4) == 0x11111111U);
	CHECK_FALSE(OTP_Write(2, 0x1111111122222222ULL)); // no retry possible
}

TEST_CASE_FIXTURE(FlashFixture, "an OTP write to a locked OTP block fails and leaves the slot erased") {
	HostFlash::write8(HostFlash::OTP_LOCK_BASE + 1, 0x00); // block 1 = slots 4..7 locked

	CHECK_FALSE(OTP_Write(4, 0));

	CHECK(HostFlash::isErased(otpSlot(4), 8));
	CHECK(HostFlash::locked());
}

TEST_CASE_FIXTURE(FlashFixture, "characterization: an OTP slot written with all ones stays writable") {
	// NEEDS REVIEW 45 (part): programming all ones changes no bit, so the "already written" check
	// (curval != all ones, flash_helpers.cpp:306) still passes and the slot can be written again
	REQUIRE(OTP_Write(7, 0xFFFFFFFFFFFFFFFFULL));

	CHECK(OTP_Write(7, 0x42));

	uint64_t v = 0;
	CHECK(OTP_Read(7, &v));
	CHECK(v == 0x42);
}

} // TEST_SUITE
