// P12 T7 lead probe of the flash and watchdog models through the real HAL (kept as the models' own sanity cases; the writers' files cover the HAL).
#include "FlashTestSupport.h"

TEST_CASE_FIXTURE(FlashFixture, "probe: unlock, program a word, erase its sector") {
	CHECK((FLASH->CR & 0x80000000u) != 0u);
	REQUIRE(HAL_FLASH_Unlock() == HAL_OK);
	CHECK((FLASH->CR & 0x80000000u) == 0u);
	const uint32_t addr = 0x08008000u; /* sector 2 */
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, 0x12345678u) == HAL_OK);
	CHECK(hostRegsPeek(addr) == 0x12345678u);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, 0xFFFF00FFu) == HAL_OK); /* NOR: 1 -> 0 only */
	CHECK(hostRegsPeek(addr) == 0x12340078u);
	FLASH_EraseInitTypeDef e{};
	e.TypeErase = FLASH_TYPEERASE_SECTORS;
	e.Sector = FLASH_SECTOR_2;
	e.NbSectors = 1;
	e.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	uint32_t err = 0;
	REQUIRE(HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
	CHECK(err == 0xFFFFFFFFu);
	CHECK(hostRegsPeek(addr) == 0xFFFFFFFFu);
	CHECK(HAL_FLASH_Lock() == HAL_OK);
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(FlashFixture, "probe: byte and halfword programming and the write protected sector") {
	REQUIRE(HAL_FLASH_Unlock() == HAL_OK);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x08000001u, 0xA5u) == HAL_OK);
	REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x08000002u, 0x1234u) == HAL_OK);
	CHECK(hostRegsPeek(0x08000000u) == 0x1234A5FFu);
	FLASH->OPTCR = FLASH->OPTCR; /* OPTCR is locked: ignored */
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(FlashFixture, "probe: IWDG init, refresh and expiry") {
	IWDG_HandleTypeDef h{};
	h.Instance = IWDG;
	h.Init.Prescaler = IWDG_PRESCALER_8;
	h.Init.Reload = 100;
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK(hostIwdgRunning() == 1);
	CHECK(hostIwdgCounter() == 100u);
	hostIwdgAdvance(40);
	CHECK(HAL_IWDG_Refresh(&h) == HAL_OK);
	CHECK(hostIwdgCounter() == 100u);
	hostIwdgAdvance(100);
	CHECK(hostIwdgResetRequested() == 1);
	CHECK(HalFixture::faults() == 0u);
}
