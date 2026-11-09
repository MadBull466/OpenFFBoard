#include "native.h"
#include <algorithm>
namespace {
void clockEvents() {
    native::event([](uint32_t){
        uint32_t cr=RCC->CR;
        cr=(cr&~(RCC_CR_HSIRDY|RCC_CR_HSERDY|RCC_CR_PLLRDY))|
            ((cr&RCC_CR_HSION)?RCC_CR_HSIRDY:0)|((cr&RCC_CR_HSEON)?RCC_CR_HSERDY:0)|((cr&RCC_CR_PLLON)?RCC_CR_PLLRDY:0);
        RCC->CR=cr;
        uint32_t sw=RCC->CFGR&RCC_CFGR_SW;
        RCC->CFGR=(RCC->CFGR&~RCC_CFGR_SWS)|(sw<<2);
    });
}
}
TEST_SUITE("P12NativeCore") {
TEST_CASE_FIXTURE(native::Fixture,"HAL initialization requests caches priority grouping and the millisecond reload") {
    REQUIRE(HAL_Init()==HAL_OK);
    CHECK((FLASH->ACR&(FLASH_ACR_ICEN|FLASH_ACR_DCEN))==(FLASH_ACR_ICEN|FLASH_ACR_DCEN));
    CHECK(SysTick->LOAD==15999); CHECK(HAL_NVIC_GetPriorityGrouping()==NVIC_PRIORITYGROUP_4);
}
TEST_CASE_FIXTURE(native::Fixture,"HAL revision and device accessors extract independent identity fields") {
    DBGMCU->IDCODE=0x10000413;
    CHECK(HAL_GetREVID()==0x1000); CHECK(HAL_GetDEVID()==0x413); CHECK(HAL_GetHalVersion()==0x01080300);
}
TEST_CASE_FIXTURE(native::Fixture,"HAL UID accessors preserve the three distinct supplied words") {
    auto *uid=reinterpret_cast<uint32_t*>(UID_BASE); uid[0]=0x1234abcd; uid[1]=0xbcde3456; uid[2]=0x87654321;
    CHECK(HAL_GetUIDw0()==uid[0]); CHECK(HAL_GetUIDw1()==uid[1]); CHECK(HAL_GetUIDw2()==uid[2]);
}
TEST_CASE_FIXTURE(native::Fixture,"debug sleep stop and standby requests preserve unrelated control bits") {
    DBGMCU->CR=0x100;
    HAL_DBGMCU_EnableDBGSleepMode(); HAL_DBGMCU_EnableDBGStopMode(); HAL_DBGMCU_EnableDBGStandbyMode();
    CHECK((DBGMCU->CR&7)==7);
    HAL_DBGMCU_DisableDBGSleepMode(); HAL_DBGMCU_DisableDBGStopMode(); HAL_DBGMCU_DisableDBGStandbyMode();
    CHECK(DBGMCU->CR==0x100);
}
TEST_CASE_FIXTURE(native::Fixture,"Cortex priority wrapper round trips the selected interrupt priority") {
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4); HAL_NVIC_SetPriority(DMA1_Stream0_IRQn,6,0);
    uint32_t preempt=99, sub=99;
    HAL_NVIC_GetPriority(DMA1_Stream0_IRQn,NVIC_PRIORITYGROUP_4,&preempt,&sub);
    CHECK(preempt==6); CHECK(sub==0);
}
TEST_CASE_FIXTURE(native::Fixture,"Cortex interrupt enable and disable wrappers issue the selected bit requests") {
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    CHECK((NVIC->ISER[0]&(1U<<DMA1_Stream0_IRQn))!=0);
    HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    CHECK((NVIC->ICER[0]&(1U<<DMA1_Stream0_IRQn))!=0);
}
TEST_CASE_FIXTURE(native::Fixture,"Cortex SysTick configuration rejects an oversized reload") {
    CHECK(HAL_SYSTICK_Config(0x1000001)==1);
    CHECK(HAL_SYSTICK_Config(16000)==0); CHECK(SysTick->LOAD==15999);
}
}
TEST_SUITE("P12NativeRCC") {
TEST_CASE_FIXTURE(native::Fixture,"RCC clock frequency getters apply independently selected bus divisors") {
    SystemCoreClock=16000000; RCC->CFGR=RCC_CFGR_PPRE1_DIV4|RCC_CFGR_PPRE2_DIV2;
    CHECK(HAL_RCC_GetSysClockFreq()==16000000); CHECK(HAL_RCC_GetHCLKFreq()==16000000);
    CHECK(HAL_RCC_GetPCLK1Freq()==4000000); CHECK(HAL_RCC_GetPCLK2Freq()==8000000);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC oscillator configuration requests HSE and observes its explicit ready event") {
    clockEvents(); RCC_OscInitTypeDef c{}; c.OscillatorType=RCC_OSCILLATORTYPE_HSE;
    c.HSEState=RCC_HSE_ON; c.PLL.PLLState=RCC_PLL_NONE;
    CHECK(HAL_RCC_OscConfig(&c)==HAL_OK); CHECK((RCC->CR&RCC_CR_HSEON)!=0);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC oscillator configuration times out without HSE readiness") {
    RCC_OscInitTypeDef c{}; c.OscillatorType=RCC_OSCILLATORTYPE_HSE; c.HSEState=RCC_HSE_ON;
    c.PLL.PLLState=RCC_PLL_NONE;
    CHECK(HAL_RCC_OscConfig(&c)==HAL_TIMEOUT);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC clock configuration requests independent AHB and APB prescalers") {
    clockEvents(); RCC_ClkInitTypeDef c{};
    c.ClockType=RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource=RCC_SYSCLKSOURCE_HSI; c.AHBCLKDivider=RCC_SYSCLK_DIV2;
    c.APB1CLKDivider=RCC_HCLK_DIV4; c.APB2CLKDivider=RCC_HCLK_DIV2;
    REQUIRE(HAL_RCC_ClockConfig(&c,FLASH_LATENCY_0)==HAL_OK);
    CHECK(SystemCoreClock==8000000); CHECK(HAL_RCC_GetPCLK1Freq()==2000000); CHECK(HAL_RCC_GetPCLK2Freq()==4000000);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC MCO configuration selects HSI with the requested divided output") {
    HAL_RCC_MCOConfig(RCC_MCO1,RCC_MCO1SOURCE_HSI,RCC_MCODIV_4);
    CHECK((RCC->CFGR&RCC_CFGR_MCO1PRE)==RCC_MCODIV_4);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC clock security requests can be enabled and disabled independently") {
    HAL_RCC_EnableCSS(); native::flush(); CHECK((RCC->CR&RCC_CR_CSSON)!=0);
    HAL_RCC_DisableCSS(); native::flush(); CHECK((RCC->CR&RCC_CR_CSSON)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC oscillator readback preserves the selected HSI trim") {
    RCC->CR=RCC_CR_HSION|RCC_CR_HSIRDY|(13U<<3);
    RCC_OscInitTypeDef c{}; HAL_RCC_GetOscConfig(&c); CHECK(c.HSICalibrationValue==13);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC extension peripheral clock readback exposes independent PLLI2S fields") {
    RCC->PLLI2SCFGR=(192U<<6)|(3U<<28);
    RCC_PeriphCLKInitTypeDef c{}; HAL_RCCEx_GetPeriphCLKConfig(&c);
    CHECK(c.PLLI2S.PLLI2SN==192); CHECK(c.PLLI2S.PLLI2SR==3);
}
}
TEST_SUITE("P12NativePower") {
TEST_CASE_FIXTURE(native::Fixture,"power backup-domain access changes only its permission bit") {
    HAL_PWR_EnableBkUpAccess(); CHECK((PWR->CR&PWR_CR_DBP)!=0);
    HAL_PWR_DisableBkUpAccess(); CHECK((PWR->CR&PWR_CR_DBP)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"power voltage detector selects the requested threshold and interrupt edges") {
    PWR_PVDTypeDef c{}; c.PVDLevel=PWR_PVDLEVEL_5; c.Mode=PWR_PVD_MODE_IT_RISING_FALLING;
    HAL_PWR_ConfigPVD(&c); HAL_PWR_EnablePVD(); native::flush();
    CHECK((PWR->CR&PWR_CR_PLS)==PWR_PVDLEVEL_5); CHECK((PWR->CR&PWR_CR_PVDE)!=0);
    CHECK((EXTI->RTSR&EXTI_RTSR_TR16)!=0); CHECK((EXTI->FTSR&EXTI_FTSR_TR16)!=0);
    HAL_PWR_DisablePVD(); native::flush(); CHECK((PWR->CR&PWR_CR_PVDE)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"power wakeup pin request can be enabled and disabled") {
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); native::flush(); CHECK((PWR->CSR&PWR_CSR_EWUP)!=0);
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); native::flush(); CHECK((PWR->CSR&PWR_CSR_EWUP)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"power sleep request records WFI without sleeping the host") {
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);
    CHECK(std::count(native::cpuRequests().begin(),native::cpuRequests().end(),4)==1);
    CHECK((SCB->SCR&SCB_SCR_SLEEPDEEP_Msk)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"power stop request restores the sleepdeep control after its recorded WFI") {
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
    CHECK(std::count(native::cpuRequests().begin(),native::cpuRequests().end(),4)==1);
    CHECK((SCB->SCR&SCB_SCR_SLEEPDEEP_Msk)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"power sleep-on-exit and event-on-pending controls preserve each other") {
    HAL_PWR_EnableSleepOnExit(); HAL_PWR_EnableSEVOnPend();
    CHECK((SCB->SCR&(SCB_SCR_SLEEPONEXIT_Msk|SCB_SCR_SEVONPEND_Msk))==(SCB_SCR_SLEEPONEXIT_Msk|SCB_SCR_SEVONPEND_Msk));
    HAL_PWR_DisableSleepOnExit(); CHECK((SCB->SCR&SCB_SCR_SEVONPEND_Msk)!=0);
    HAL_PWR_DisableSEVOnPend(); CHECK((SCB->SCR&SCB_SCR_SEVONPEND_Msk)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"power extension flash powerdown toggles its own request") {
    HAL_PWREx_EnableFlashPowerDown(); native::flush(); CHECK((PWR->CR&PWR_CR_FPDS)!=0);
    HAL_PWREx_DisableFlashPowerDown(); native::flush(); CHECK((PWR->CR&PWR_CR_FPDS)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"watchdog initialization programs its selected prescaler and reload") {
    IWDG_HandleTypeDef h{}; h.Instance=IWDG; h.Init.Prescaler=IWDG_PRESCALER_64; h.Init.Reload=1000;
    REQUIRE(HAL_IWDG_Init(&h)==HAL_OK); CHECK(IWDG->PR==IWDG_PRESCALER_64); CHECK(IWDG->RLR==1000);
    CHECK(HAL_IWDG_Refresh(&h)==HAL_OK); CHECK(IWDG->KR==0xaaaa);
}
TEST_CASE_FIXTURE(native::Fixture,"watchdog initialization times out while update flags remain busy") {
    IWDG_HandleTypeDef h{}; h.Instance=IWDG; h.Init.Prescaler=IWDG_PRESCALER_64; h.Init.Reload=1000;
    IWDG->SR=IWDG_SR_PVU; CHECK(HAL_IWDG_Init(&h)==HAL_TIMEOUT);
}
}
TEST_SUITE("P12NativeFLASH") {
TEST_CASE_FIXTURE(native::Fixture,"flash unlock uses the key protocol and lock restores protection") {
    REQUIRE(HAL_FLASH_Unlock()==HAL_OK); CHECK((FLASH->CR&FLASH_CR_LOCK)==0);
    CHECK(HAL_FLASH_Lock()==HAL_OK); CHECK((FLASH->CR&FLASH_CR_LOCK)!=0);
}
TEST_CASE_FIXTURE(native::Fixture,"flash program stores a selected word in initially erased anonymous flash") {
    REQUIRE(HAL_FLASH_Unlock()==HAL_OK);
    REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_BASE+0x100,0x1234abcd)==HAL_OK);
    CHECK(*reinterpret_cast<uint32_t*>(FLASH_BASE+0x100)==0x1234abcd);
    CHECK(HAL_FLASH_GetError()==HAL_FLASH_ERROR_NONE);
}
TEST_CASE_FIXTURE(native::Fixture,"flash byte and halfword program requests preserve neighboring erased bytes") {
    REQUIRE(HAL_FLASH_Unlock()==HAL_OK);
    REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,FLASH_BASE+0x100,0x35)==HAL_OK);
    REQUIRE(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,FLASH_BASE+0x102,0x5678)==HAL_OK);
    CHECK(*reinterpret_cast<uint32_t*>(FLASH_BASE+0x100)==0x5678ff35);
}
TEST_CASE_FIXTURE(native::Fixture,"flash option unlock and lock use their independent key pair") {
    CHECK(HAL_FLASH_OB_Unlock()==HAL_OK); native::flush(); CHECK((FLASH->OPTCR&FLASH_OPTCR_OPTLOCK)==0);
    CHECK(HAL_FLASH_OB_Lock()==HAL_OK); CHECK((FLASH->OPTCR&FLASH_OPTCR_OPTLOCK)!=0);
}
TEST_CASE_FIXTURE(native::Fixture,"flash erase issues a sector-specific request without claiming RAM erased itself") {
    REQUIRE(HAL_FLASH_Unlock()==HAL_OK);
    FLASH_EraseInitTypeDef c{}; c.TypeErase=FLASH_TYPEERASE_SECTORS; c.Sector=FLASH_SECTOR_5;
    c.NbSectors=1; c.VoltageRange=FLASH_VOLTAGE_RANGE_3; uint32_t failed=0;
    REQUIRE(HAL_FLASHEx_Erase(&c,&failed)==HAL_OK);
    CHECK(failed==0xffffffffU);
    bool found=false;
    for(const auto &w:native::writes()) if(w.address==reinterpret_cast<uintptr_t>(&FLASH->CR)&&
       (w.value&FLASH_CR_SER)&&((w.value&FLASH_CR_SNB)==(5U<<3))&&(w.value&FLASH_CR_STRT)) found=true;
    CHECK(found);
}
}

#if defined(STM32F411xE)
TEST_SUITE("P12NativeF411RequestContracts") {
TEST_CASE_FIXTURE(native::Fixture,"F411 flash interface clock stop enables power clock and requests interface stop") {
    REQUIRE(HAL_FLASHEx_StopFlashInterfaceClk()==HAL_OK); native::flush();
    CHECK((RCC->APB1ENR&RCC_APB1ENR_PWREN)!=0); CHECK((PWR->CR&PWR_CR_FISSR)!=0);
}
TEST_CASE_FIXTURE(native::Fixture,"F411 flash interface clock start removes only its stop request") {
    PWR->CR=PWR_CR_FISSR|PWR_CR_DBP;
    REQUIRE(HAL_FLASHEx_StartFlashInterfaceClk()==HAL_OK); native::flush(); CHECK(PWR->CR==PWR_CR_DBP);
}
TEST_CASE_FIXTURE(native::Fixture,"F411 flash sleep enable preserves backup access") {
    PWR->CR=PWR_CR_DBP; REQUIRE(HAL_FLASHEx_EnableFlashSleepMode()==HAL_OK); native::flush();
    CHECK(PWR->CR==(PWR_CR_DBP|PWR_CR_FMSSR));
}
TEST_CASE_FIXTURE(native::Fixture,"F411 flash sleep disable preserves backup access") {
    PWR->CR=PWR_CR_DBP|PWR_CR_FMSSR; REQUIRE(HAL_FLASHEx_DisableFlashSleepMode()==HAL_OK); native::flush();
    CHECK(PWR->CR==PWR_CR_DBP);
}
}
#endif
TEST_SUITE("P12NativeClockBranches") {
TEST_CASE_FIXTURE(native::Fixture,"RCC LSI configuration observes the supplied ready and stop events") {
    native::event([](uint32_t){RCC->CSR=(RCC->CSR&~RCC_CSR_LSIRDY)|((RCC->CSR&RCC_CSR_LSION)?RCC_CSR_LSIRDY:0);});
    RCC_OscInitTypeDef c{};c.OscillatorType=RCC_OSCILLATORTYPE_LSI;c.LSIState=RCC_LSI_ON;c.PLL.PLLState=RCC_PLL_NONE;
    REQUIRE(HAL_RCC_OscConfig(&c)==HAL_OK);CHECK((RCC->CSR&RCC_CSR_LSIRDY)!=0);
    c.LSIState=RCC_LSI_OFF;REQUIRE(HAL_RCC_OscConfig(&c)==HAL_OK);CHECK((RCC->CSR&RCC_CSR_LSION)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC LSE configuration requests backup permission and observes readiness") {
    native::event([](uint32_t){RCC->BDCR=(RCC->BDCR&~RCC_BDCR_LSERDY)|((RCC->BDCR&RCC_BDCR_LSEON)?RCC_BDCR_LSERDY:0);});
    RCC_OscInitTypeDef c{};c.OscillatorType=RCC_OSCILLATORTYPE_LSE;c.LSEState=RCC_LSE_ON;c.PLL.PLLState=RCC_PLL_NONE;
    REQUIRE(HAL_RCC_OscConfig(&c)==HAL_OK);CHECK((RCC->BDCR&RCC_BDCR_LSERDY)!=0);CHECK((PWR->CR&PWR_CR_DBP)!=0);
    c.LSEState=RCC_LSE_OFF;REQUIRE(HAL_RCC_OscConfig(&c)==HAL_OK);CHECK((RCC->BDCR&RCC_BDCR_LSEON)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC HSI calibration changes the requested trim without disabling the active source") {
    RCC_OscInitTypeDef c{};c.OscillatorType=RCC_OSCILLATORTYPE_HSI;c.HSIState=RCC_HSI_ON;
    c.HSICalibrationValue=9;c.PLL.PLLState=RCC_PLL_NONE;
    REQUIRE(HAL_RCC_OscConfig(&c)==HAL_OK);CHECK(((RCC->CR&RCC_CR_HSITRIM)>>3)==9);CHECK((RCC->CR&RCC_CR_HSION)!=0);
    c.HSIState=RCC_HSI_OFF;CHECK(HAL_RCC_OscConfig(&c)==HAL_ERROR);CHECK((RCC->CR&RCC_CR_HSION)!=0);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC PLL request programs its exact source multiplication and divisors") {
    clockEvents();RCC_OscInitTypeDef c{};c.OscillatorType=RCC_OSCILLATORTYPE_NONE;c.PLL.PLLState=RCC_PLL_ON;
    c.PLL.PLLSource=RCC_PLLSOURCE_HSI;c.PLL.PLLM=16;c.PLL.PLLN=192;c.PLL.PLLP=RCC_PLLP_DIV4;c.PLL.PLLQ=4;
    REQUIRE(HAL_RCC_OscConfig(&c)==HAL_OK);
    CHECK((RCC->PLLCFGR&63)==16);CHECK(((RCC->PLLCFGR>>6)&511)==192);CHECK(((RCC->PLLCFGR>>16)&3)==1);
    CHECK(((RCC->PLLCFGR>>24)&15)==4);CHECK((RCC->CR&RCC_CR_PLLON)!=0);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC PLL frequency getter applies source input M N and P independently") {
    RCC->CFGR=RCC_CFGR_SWS_PLL;RCC->PLLCFGR=16|(192U<<6)|(1U<<16)|(4U<<24);
    CHECK(HAL_RCC_GetSysClockFreq()==48000000);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC clock switch rejects an unready PLL without selecting it") {
    RCC_ClkInitTypeDef c{};c.ClockType=RCC_CLOCKTYPE_SYSCLK;c.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;
    CHECK(HAL_RCC_ClockConfig(&c,FLASH_LATENCY_3)==HAL_ERROR);CHECK((RCC->CFGR&RCC_CFGR_SW)==RCC_CFGR_SW_HSI);
}
TEST_CASE_FIXTURE(native::Fixture,"RCC clock readback preserves independent prescaler and flash latency fields") {
    RCC->CFGR=RCC_CFGR_HPRE_DIV4|RCC_CFGR_PPRE1_DIV8|RCC_CFGR_PPRE2_DIV2;FLASH->ACR=FLASH_LATENCY_3;
    RCC_ClkInitTypeDef c{};uint32_t latency=0;HAL_RCC_GetClockConfig(&c,&latency);
    CHECK(c.AHBCLKDivider==RCC_SYSCLK_DIV4);CHECK(c.APB1CLKDivider==RCC_HCLK_DIV8);
    CHECK(c.APB2CLKDivider==RCC_HCLK_DIV2);CHECK(latency==3);
}
TEST_CASE_FIXTURE(native::Fixture,"Cortex pending interrupt wrappers expose pending input and selected clear request") {
    NVIC->ISPR[0]=1U<<DMA1_Stream0_IRQn;CHECK(HAL_NVIC_GetPendingIRQ(DMA1_Stream0_IRQn)==1);
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);CHECK(NVIC->ICPR[0]==(1U<<DMA1_Stream0_IRQn));
}
TEST_CASE_FIXTURE(native::Fixture,"Cortex active interrupt wrapper reads the selected supplied active bit") {
    NVIC->IABR[0]=1U<<DMA1_Stream1_IRQn;CHECK(HAL_NVIC_GetActive(DMA1_Stream1_IRQn)==1);
    CHECK(HAL_NVIC_GetActive(DMA1_Stream0_IRQn)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"Cortex SysTick source selection preserves its other control requests") {
    SysTick->CTRL=SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_TICKINT_Msk;
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);CHECK((SysTick->CTRL&SysTick_CTRL_CLKSOURCE_Msk)!=0);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);CHECK((SysTick->CTRL&SysTick_CTRL_CLKSOURCE_Msk)==0);
    CHECK((SysTick->CTRL&(SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_TICKINT_Msk))==(SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_TICKINT_Msk));
}
}
TEST_SUITE("P12NativePlatformRequests") {
TEST_CASE_FIXTURE(native::Fixture,"HAL deinitialization issues reset and release requests for selected AHB1 peripherals") {
    REQUIRE(HAL_DeInit()==HAL_OK);CHECK(RCC->AHB1RSTR==0);CHECK(RCC->AHB2RSTR==0);
    CHECK(RCC->APB1RSTR==0);CHECK(RCC->APB2RSTR==0);
    bool asserted=false;for(const auto &w:native::writes())
        if(w.address==reinterpret_cast<uintptr_t>(&RCC->AHB1RSTR)&&(w.value&(RCC_AHB1RSTR_GPIOCRST|RCC_AHB1RSTR_DMA1RST))==(RCC_AHB1RSTR_GPIOCRST|RCC_AHB1RSTR_DMA1RST))asserted=true;
    CHECK(asserted);
}
TEST_CASE_FIXTURE(native::Fixture,"HAL compensation-cell enable and disable preserve unrelated control bits") {
    SYSCFG->CMPCR=1U<<8;HAL_EnableCompensationCell();native::flush();CHECK(SYSCFG->CMPCR==0x101);
    HAL_DisableCompensationCell();native::flush();CHECK(SYSCFG->CMPCR==0x100);
}
TEST_CASE_FIXTURE(native::Fixture,"MPU configuration requests the selected base size access and execution permissions") {
    MPU_Region_InitTypeDef c{};c.Enable=MPU_REGION_ENABLE;c.Number=3;c.BaseAddress=0x20000000;
    c.Size=MPU_REGION_SIZE_256B;c.AccessPermission=MPU_REGION_FULL_ACCESS;c.DisableExec=MPU_INSTRUCTION_ACCESS_DISABLE;
    HAL_MPU_ConfigRegion(&c);CHECK(MPU->RNR==3);CHECK(MPU->RBAR==0x20000000);CHECK(MPU->RASR==0x1300000f);
    HAL_MPU_DisableRegion(3);CHECK((MPU->RASR&1)==0);HAL_MPU_EnableRegion(3);CHECK((MPU->RASR&1)!=0);
}
TEST_CASE_FIXTURE(native::Fixture,"MPU enable and disable request fault routing without claiming memory protection") {
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);CHECK((MPU->CTRL&MPU_CTRL_ENABLE_Msk)!=0);
    CHECK((SCB->SHCSR&SCB_SHCSR_MEMFAULTENA_Msk)!=0);
    HAL_MPU_Disable();CHECK(MPU->CTRL==0);CHECK((SCB->SHCSR&SCB_SHCSR_MEMFAULTENA_Msk)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"Cortex clear-event wrapper records its event and wait requests") {
    HAL_CORTEX_ClearEvent();CHECK(std::count(native::cpuRequests().begin(),native::cpuRequests().end(),6)==1);
    CHECK(std::count(native::cpuRequests().begin(),native::cpuRequests().end(),5)==1);
}
TEST_CASE_FIXTURE(native::Fixture,"power standby requests deep sleep and records WFI without host sleep") {
    HAL_PWR_EnterSTANDBYMode();CHECK((PWR->CR&PWR_CR_PDDS)!=0);CHECK((SCB->SCR&SCB_SCR_SLEEPDEEP_Msk)!=0);
    CHECK(std::count(native::cpuRequests().begin(),native::cpuRequests().end(),4)==1);
}
TEST_CASE_FIXTURE(native::Fixture,"power sleep via WFE records event clearing and both wait requests") {
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFE);
    CHECK(std::count(native::cpuRequests().begin(),native::cpuRequests().end(),6)==1);
    CHECK(std::count(native::cpuRequests().begin(),native::cpuRequests().end(),5)==2);
}
TEST_CASE_FIXTURE(native::Fixture,"power voltage scaling succeeds only after supplied regulator readiness") {
    native::event([](uint32_t){PWR->CSR=PWR->CSR|PWR_CSR_VOSRDY;
        RCC->CR=(RCC->CR&~RCC_CR_PLLRDY)|((RCC->CR&RCC_CR_PLLON)?RCC_CR_PLLRDY:0);});
    REQUIRE(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2)==HAL_OK);
    CHECK(HAL_PWREx_GetVoltageRange()==PWR_REGULATOR_VOLTAGE_SCALE2);
}
TEST_CASE_FIXTURE(native::Fixture,"power voltage scaling times out without supplied regulator readiness") {
    CHECK(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2)==HAL_TIMEOUT);
}
}
