// P12 review witnesses, flash and watchdog (Codex review of P12, P12C-T7-002 and P12C-T7-003).
#include "FlashTestSupport.h"

#include <cstring>

extern "C" {
extern FLASH_ProcessTypeDef pFlash;
}
extern unsigned flashCacheResetSeenAtCallback;
extern unsigned flashObserveCacheReset;

TEST_CASE_FIXTURE(FlashFixture, "RV18 the end-of-operation callback of the last sector runs after the instruction cache reset (P12C-T7-002)") {
	// HAL_FLASH_IRQHandler resets the caches (FLASH_FlushCaches) before it calls the callback; checked inside the callback, because the state
	// after the handler returned is the same for a flush that comes after it.
	std::memset(const_cast<FLASH_ProcessTypeDef *>(&pFlash), 0, sizeof pFlash);
	flashCacheResetSeenAtCallback = 0;
	flashObserveCacheReset = 1;
	HalFixture::poke(&FLASH->ACR, 0x200u);  // ICEN
	REQUIRE(HAL_FLASH_Unlock() == HAL_OK);
	hostFlashSetPollReads(0);
	FLASH_EraseInitTypeDef e{};
	e.TypeErase = FLASH_TYPEERASE_SECTORS;
	e.Sector = 2;
	e.NbSectors = 1;
	e.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	REQUIRE(HAL_FLASHEx_Erase_IT(&e) == HAL_OK);
	hostRegsLogClear();
	REQUIRE(hostFlashComplete() == 1);
	HAL_FLASH_IRQHandler();
	REQUIRE(flashCounts.eop == 1u);
	CHECK(flashCacheResetSeenAtCallback == 1u);
}

TEST_CASE_FIXTURE(FlashFixture, "RV19 advancing the watchdog by zero events changes nothing, also for a reload value of 0 (P12C-T7-003)") {
	IWDG_HandleTypeDef h{};
	h.Instance = IWDG;
	h.Init.Prescaler = IWDG_PRESCALER_4;
	SUBCASE("zero reload") { h.Init.Reload = 0; }
	SUBCASE("positive reload (control)") { h.Init.Reload = 17; }
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	REQUIRE(hostIwdgResetRequested() == 0);
	CHECK(hostIwdgAdvance(0) == h.Init.Reload);
	CHECK(hostIwdgResetRequested() == 0);
	CHECK(hostIwdgCounter() == h.Init.Reload);
	CHECK(hostIwdgRunning() == 1);
	REQUIRE(HAL_IWDG_Refresh(&h) == HAL_OK);
	CHECK(hostIwdgResetRequested() == 0);
}
