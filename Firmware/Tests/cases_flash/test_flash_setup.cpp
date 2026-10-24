/*
 * Smoke tests of ffb_flash_tests: the fake HAL FLASH (NOR semantics, lock, errors) and the real flash layer on top
 * of it (flash_helpers.cpp + eeprom.c), including one power-loss scenario.
 * Detailed tests of eeprom.c, flash_helpers.cpp, eeprom_addresses.c and PersistentStorage are in the module files.
 */
#include "doctest.h"
#include "FlashFixture.h"
#include "TestHelpers.h"

#include <map>

TEST_SUITE("FlashSetup") {

TEST_CASE_FIXTURE(FlashFixture, "the fake flash memory is at the target addresses and erased") {
	// eeprom.c reads PAGE0_BASE_ADDRESS (EEPROM_START_ADDRESS = 0x08004000) through an absolute address
	CHECK(PAGE0_BASE_ADDRESS == HostFlash::EEPROM_PAGE0);
	CHECK(PAGE1_BASE_ADDRESS == HostFlash::EEPROM_PAGE1);
	CHECK(*(volatile uint16_t *)(uintptr_t)PAGE0_BASE_ADDRESS == 0xFFFF);
	CHECK(HostFlash::isErased(HostFlash::MAIN_BASE, HostFlash::MAIN_SIZE));
	CHECK(HostFlash::isErased(HostFlash::OTP_BASE, HostFlash::OTP_END + 1 - HostFlash::OTP_BASE));
	CHECK(HostFlash::locked());
}

TEST_CASE_FIXTURE(FlashFixture, "a sector erase sets the whole sector to 0xFF and reports no failed sector") {
	HostFlash::fill(HostFlash::EEPROM_PAGE0, HostFlash::EEPROM_PAGE_SIZE, 0x00);
	HostFlash::fill(HostFlash::EEPROM_PAGE1, HostFlash::EEPROM_PAGE_SIZE, 0x00);
	REQUIRE(HAL_FLASH_Unlock() == HAL_OK);
	FLASH_EraseInitTypeDef erase = {FLASH_TYPEERASE_SECTORS, 0, FLASH_SECTOR_1, 1, FLASH_VOLTAGE_RANGE_3};
	uint32_t sectorError = 0;

	CHECK(HAL_FLASHEx_Erase(&erase, &sectorError) == HAL_OK);

	CHECK(sectorError == 0xFFFFFFFFU);
	CHECK(HostFlash::isErased(HostFlash::EEPROM_PAGE0, HostFlash::EEPROM_PAGE_SIZE));
	CHECK(HostFlash::read8(HostFlash::EEPROM_PAGE1) == 0x00); // sector 2 untouched
	CHECK(HostFlash::state().erasesPerSector[1] == 1);
	CHECK(HostFlash::state().erasesPerSector[2] == 0);
}

TEST_CASE_FIXTURE(FlashFixture, "programming can only clear bits") {
	const uint32_t adr = HostFlash::EEPROM_PAGE0 + 8;
	REQUIRE(HAL_FLASH_Unlock() == HAL_OK);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, adr, 0x00FF) == HAL_OK);

	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, adr, 0xFF0F) == HAL_OK); // the chip reports no error

	CHECK(HostFlash::read16(adr) == 0x000F); // old & new: the 0 bits of the first value stay 0
	CHECK(HostFlash::state().zeroToOneRequests == 1);
	CHECK(HostFlash::state().lastZeroToOneAddress == adr);
	CHECK(HostFlash::state().programsPerSector[1] == 2);
}

TEST_CASE_FIXTURE(FlashFixture, "programming while the flash is locked fails with a sequence error and writes nothing") {
	const uint32_t adr = HostFlash::EEPROM_PAGE0;

	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, adr, 0x1234) == HAL_ERROR);

	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGS);
	CHECK(HostFlash::read16(adr) == 0xFFFF);
	CHECK(__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGSERR) == 0); // cleared by FLASH_SetErrorCode like the HAL
	CHECK(HostFlash::state().programsApplied == 0);
}

TEST_CASE_FIXTURE(FlashFixture, "an injected programming error is reported by the HAL and leaves the flash unchanged") {
	const uint32_t adr = HostFlash::EEPROM_PAGE0 + 4;
	REQUIRE(HAL_FLASH_Unlock() == HAL_OK);
	HostFlash::failNext(1, FLASH_FLAG_PGPERR, HostFlash::Target::Program);

	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, adr, 0x1234) == HAL_ERROR);
	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGP);
	CHECK(HostFlash::read16(adr) == 0xFFFF);
	// Only the next operation fails
	CHECK(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, adr, 0x1234) == HAL_OK);
	CHECK(HostFlash::read16(adr) == 0x1234);
}

TEST_CASE_FIXTURE(FlashFixture, "the first start formats the blank flash with page 0 valid and locks the flash again") {
	CHECK(init());

	CHECK(pageStatus(0) == VALID_PAGE);
	CHECK(pageStatus(1) == ERASED);
	CHECK(usedRecords(0) == 0);
	CHECK(HostFlash::locked());
	// Only the EEPROM sectors are touched (sector 0 holds the firmware on the target)
	CHECK(HostFlash::state().programsPerSector[0] == 0);
	CHECK(HostFlash::state().erasesPerSector[0] == 0);
	CHECK(HostFlash::state().programsPerSector[3] == 0);
}

TEST_CASE_FIXTURE(FlashFixture, "a written variable is read back and stored as one record in the valid page") {
	REQUIRE(init());

	CHECK(Flash_Write(ADR_CURRENT_CONFIG, 0x1234));

	uint16_t value = 0;
	CHECK(Flash_Read(ADR_CURRENT_CONFIG, &value));
	CHECK(value == 0x1234);
	auto records = pageRecords(0);
	REQUIRE(records.size() == 1);
	CHECK(records[0].address == ADR_CURRENT_CONFIG);
	CHECK(records[0].value == 0x1234);
	CHECK(HostFlash::locked());
}

TEST_CASE_FIXTURE(FlashFixture, "a written variable survives a reboot") {
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_CURRENT_CONFIG, 0xBEEF));

	CHECK(reboot());

	uint16_t value = 0;
	CHECK(Flash_Read(ADR_CURRENT_CONFIG, &value));
	CHECK(value == 0xBEEF);
	CHECK(pageStatus(0) == VALID_PAGE);
	CHECK(usedRecords(0) == 1);
}

TEST_CASE_FIXTURE(FlashFixture, "a power loss at any program step of a page transfer keeps every variable at its old or new value") {
	// Page 0 is full: the next write moves the newest value of every variable to page 1 (EE_PageTransfer).
	const std::vector<uint16_t> addresses = {ADR_HW_VERSION, ADR_SW_VERSION, ADR_CURRENT_CONFIG};
	std::map<uint16_t, uint16_t> old;
	for (uint32_t i = 0; i < RECORDS_PER_PAGE; i++) {
		old[addresses[i % addresses.size()]] = (uint16_t)i; // fillPage: value = record index, the last one wins
	}
	const uint16_t newAddress = ADR_FLASH_VERSION;
	const uint16_t newValue = 0x1234;

	// Reference run without power loss: number of program operations of the write
	fillPage(0, VALID_PAGE, RECORDS_PER_PAGE, addresses);
	REQUIRE(Flash_Write(newAddress, newValue));
	const uint32_t programs = HostFlash::state().programsApplied;
	REQUIRE(programs == 1 + 2 + 2 * addresses.size() + 1); // RECEIVE_DATA, new record, transferred records, VALID_PAGE
	REQUIRE(pageStatus(1) == VALID_PAGE);

	for (uint32_t completed = 0; completed < programs; completed++) {
		CAPTURE(completed);
		resetIteration();
		fillPage(0, VALID_PAGE, RECORDS_PER_PAGE, addresses);
		HostFlash::cutPowerAfterPrograms(completed);

		Flash_Write(newAddress, newValue); // result is meaningless after the power loss
		REQUIRE(HostFlash::powerLost());

		CHECK(reboot());
		for (uint16_t adr : addresses) {
			uint16_t value = 0;
			CAPTURE(adr);
			CHECK(Flash_Read(adr, &value));
			CHECK(value == old[adr]);
		}
		uint16_t value = 0;
		if (Flash_Read(newAddress, &value)) {
			CHECK(value == newValue);
		}
		CHECK(((pageStatus(0) == VALID_PAGE) != (pageStatus(1) == VALID_PAGE))); // exactly one valid page
	}
}

} // TEST_SUITE
