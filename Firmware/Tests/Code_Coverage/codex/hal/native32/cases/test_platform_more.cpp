#include "native.h"
namespace {
std::vector<uint32_t> flashDone,flashErrors;
struct Flash : native::Fixture {
    Flash() { flashDone.clear();flashErrors.clear(); }
};
void pllEvents() {
    native::event([](uint32_t){
        RCC->CR=(RCC->CR&~RCC_CR_PLLI2SRDY)|((RCC->CR&RCC_CR_PLLI2SON)?RCC_CR_PLLI2SRDY:0);
    });
}
}
extern "C" void HAL_FLASH_EndOfOperationCallback(uint32_t address) { flashDone.push_back(address); }
extern "C" void HAL_FLASH_OperationErrorCallback(uint32_t address) { flashErrors.push_back(address); }
TEST_SUITE("P12NativePlatformRecovery") {
TEST_CASE_FIXTURE(Flash,"flash doubleword program retains both distinct words") {
    REQUIRE(HAL_FLASH_Unlock()==HAL_OK);
    REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,FLASH_BASE+0x180,0x123456789abcdef0ULL)==HAL_OK);
    CHECK(*reinterpret_cast<uint64_t*>(FLASH_BASE+0x180)==0x123456789abcdef0ULL);
    CHECK((FLASH->CR&FLASH_CR_PG)==0);
}
TEST_CASE_FIXTURE(Flash,"flash interrupt program reports the selected address exactly once") {
    REQUIRE(HAL_FLASH_Unlock()==HAL_OK);
    REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD,FLASH_BASE+0x180,0x13579bdf)==HAL_OK);
    native::flush(); native::advanceBus(); HAL_FLASH_IRQHandler(); native::flush();
    CHECK(flashDone==std::vector<uint32_t>{FLASH_BASE+0x180}); CHECK(flashErrors.empty());
    CHECK((FLASH->SR&FLASH_SR_EOP)==0); CHECK((FLASH->CR&(FLASH_CR_PG|FLASH_CR_EOPIE|FLASH_CR_ERRIE))==0);
}
TEST_CASE_FIXTURE(Flash,"flash interrupt error reports the failed program address and clears its error flag") {
    REQUIRE(HAL_FLASH_Unlock()==HAL_OK);
    REQUIRE(HAL_FLASH_Program_IT(FLASH_TYPEPROGRAM_WORD,FLASH_BASE+0x184,0x13579bdf)==HAL_OK);
    native::flush();native::advanceBus(); FLASH->SR=FLASH_SR_WRPERR; HAL_FLASH_IRQHandler(); native::flush();
    CHECK(flashErrors==std::vector<uint32_t>{FLASH_BASE+0x184}); CHECK(flashDone.empty());
    CHECK((HAL_FLASH_GetError()&HAL_FLASH_ERROR_WRP)!=0); CHECK((FLASH->SR&FLASH_SR_WRPERR)==0);
}
TEST_CASE_FIXTURE(Flash,"flash sector erase interrupt advances sectors then reports the terminal sentinel") {
    REQUIRE(HAL_FLASH_Unlock()==HAL_OK); FLASH_EraseInitTypeDef c{};
    c.TypeErase=FLASH_TYPEERASE_SECTORS;c.Sector=FLASH_SECTOR_2;c.NbSectors=2;c.VoltageRange=FLASH_VOLTAGE_RANGE_3;
    REQUIRE(HAL_FLASHEx_Erase_IT(&c)==HAL_OK); native::flush();
    FLASH->SR=FLASH_SR_EOP;HAL_FLASH_IRQHandler(); native::flush(); CHECK(flashDone==std::vector<uint32_t>{2});
    CHECK((FLASH->CR&FLASH_CR_SNB)==(3U<<3));
    FLASH->SR=FLASH_SR_EOP;HAL_FLASH_IRQHandler(); native::flush();
    CHECK(flashDone==std::vector<uint32_t>{2,0xffffffffU}); CHECK((FLASH->CR&(FLASH_CR_SER|FLASH_CR_EOPIE))==0);
}
TEST_CASE_FIXTURE(Flash,"flash option user configuration preserves a nondefault brownout selection") {
    REQUIRE(HAL_FLASH_OB_Unlock()==HAL_OK); native::flush(); FLASH->OPTCR=FLASH->OPTCR|OB_BOR_LEVEL2;
    FLASH_OBProgramInitTypeDef c{};c.OptionType=OPTIONBYTE_USER;c.USERConfig=OB_IWDG_SW|OB_STOP_NO_RST|OB_STDBY_NO_RST;
    REQUIRE(HAL_FLASHEx_OBProgram(&c)==HAL_OK); native::flush();
    CHECK((FLASH->OPTCR&FLASH_OPTCR_BOR_LEV)==OB_BOR_LEVEL2);
    FLASH_OBProgramInitTypeDef read{};HAL_FLASHEx_OBGetConfig(&read);CHECK(read.USERConfig==c.USERConfig);
}
TEST_CASE_FIXTURE(Flash,"flash option brownout configuration can be read back independently") {
    REQUIRE(HAL_FLASH_OB_Unlock()==HAL_OK); FLASH_OBProgramInitTypeDef c{};
    c.OptionType=OPTIONBYTE_BOR;c.BORLevel=OB_BOR_LEVEL3;
    REQUIRE(HAL_FLASHEx_OBProgram(&c)==HAL_OK); FLASH_OBProgramInitTypeDef read{};
    HAL_FLASHEx_OBGetConfig(&read);CHECK(read.BORLevel==OB_BOR_LEVEL3);
}
TEST_CASE_FIXTURE(Flash,"flash option write protection enable and disable invert only selected sector bits") {
    REQUIRE(HAL_FLASH_OB_Unlock()==HAL_OK); native::flush();FLASH->OPTCR=FLASH->OPTCR|FLASH_OPTCR_nWRP;
    FLASH_OBProgramInitTypeDef c{};c.OptionType=OPTIONBYTE_WRP;c.WRPSector=OB_WRP_SECTOR_2;c.Banks=FLASH_BANK_1;
    c.WRPState=OB_WRPSTATE_ENABLE; REQUIRE(HAL_FLASHEx_OBProgram(&c)==HAL_OK); native::flush();
    CHECK((FLASH->OPTCR&FLASH_OPTCR_nWRP)==(FLASH_OPTCR_nWRP&~(1U<<18)));
    c.WRPState=OB_WRPSTATE_DISABLE;REQUIRE(HAL_FLASHEx_OBProgram(&c)==HAL_OK);native::flush();
    CHECK((FLASH->OPTCR&FLASH_OPTCR_nWRP)==FLASH_OPTCR_nWRP);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC PLLI2S enable observes readiness and preserves its selected factors") {
    pllEvents(); RCC_PLLI2SInitTypeDef c{};c.PLLI2SN=192;c.PLLI2SR=3;
#if defined(STM32F411xE)
    c.PLLI2SM=16;
#endif
    REQUIRE(HAL_RCCEx_EnablePLLI2S(&c)==HAL_OK);CHECK((RCC->CR&RCC_CR_PLLI2SRDY)!=0);
    CHECK(((RCC->PLLI2SCFGR>>6)&511)==192);CHECK(((RCC->PLLI2SCFGR>>28)&7)==3);
    REQUIRE(HAL_RCCEx_DisablePLLI2S()==HAL_OK);CHECK((RCC->CR&(RCC_CR_PLLI2SON|RCC_CR_PLLI2SRDY))==0);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC PLLI2S readiness timeout rejects an unready clock") {
    RCC_PLLI2SInitTypeDef c{};c.PLLI2SN=192;c.PLLI2SR=3;
#if defined(STM32F411xE)
    c.PLLI2SM=16;
#endif
    CHECK(HAL_RCCEx_EnablePLLI2S(&c)==HAL_TIMEOUT);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC peripheral I2S configuration requests its independent clock factors") {
    pllEvents();RCC_PeriphCLKInitTypeDef c{};c.PeriphClockSelection=RCC_PERIPHCLK_I2S;
    c.PLLI2S.PLLI2SN=240;c.PLLI2S.PLLI2SR=5;
#if defined(STM32F411xE)
    c.PLLI2S.PLLI2SM=16;
#endif
    REQUIRE(HAL_RCCEx_PeriphCLKConfig(&c)==HAL_OK);
    CHECK(((RCC->PLLI2SCFGR>>6)&511)==240);CHECK(((RCC->PLLI2SCFGR>>28)&7)==5);
}
#if defined(PWR_CSR_BRE)
TEST_CASE_FIXTURE(native::Fixture,"backup regulator enable waits for an explicitly supplied ready event") {
    native::event([](uint32_t){ if(PWR->CSR&PWR_CSR_BRE) PWR->CSR=PWR->CSR|PWR_CSR_BRR; });
    CHECK(HAL_PWREx_EnableBkUpReg()==HAL_OK);CHECK((PWR->CSR&PWR_CSR_BRE)!=0);
}
TEST_CASE_FIXTURE(native::Fixture,"backup regulator enable times out without readiness") {
    CHECK(HAL_PWREx_EnableBkUpReg()==HAL_TIMEOUT);CHECK((PWR->CSR&PWR_CSR_BRE)!=0);
}
TEST_CASE_FIXTURE(native::Fixture,"backup regulator disable waits until the ready indication clears") {
    PWR->CSR=PWR_CSR_BRE|PWR_CSR_BRR;
    native::event([](uint32_t){if(!(PWR->CSR&PWR_CSR_BRE))PWR->CSR=PWR->CSR&~PWR_CSR_BRR;});
    CHECK(HAL_PWREx_DisableBkUpReg()==HAL_OK);CHECK((PWR->CSR&(PWR_CSR_BRE|PWR_CSR_BRR))==0);
}
TEST_CASE_FIXTURE(native::Fixture,"backup regulator disable times out while readiness remains asserted") {
    PWR->CSR=PWR_CSR_BRE|PWR_CSR_BRR;CHECK(HAL_PWREx_DisableBkUpReg()==HAL_TIMEOUT);
}
#endif
}
