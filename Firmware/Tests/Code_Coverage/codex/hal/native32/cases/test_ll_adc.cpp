#include "native.h"
#include "stm32f4xx_ll_adc.h"
#ifdef USE_FULL_LL_DRIVER
TEST_SUITE("P12NativeLLADC") {
TEST_CASE_FIXTURE(native::Fixture,"LL ADC initialization requests a nondefault resolution and alignment") {
    LL_ADC_InitTypeDef c{}; LL_ADC_StructInit(&c);
    c.Resolution=LL_ADC_RESOLUTION_10B; c.DataAlignment=LL_ADC_DATA_ALIGN_LEFT;
    c.SequencersScanMode=LL_ADC_SEQ_SCAN_ENABLE;
    REQUIRE(LL_ADC_Init(ADC1,&c)==SUCCESS);
    CHECK((ADC1->CR1&ADC_CR1_RES)==LL_ADC_RESOLUTION_10B);
    CHECK((ADC1->CR2&ADC_CR2_ALIGN)==LL_ADC_DATA_ALIGN_LEFT); CHECK((ADC1->CR1&ADC_CR1_SCAN)!=0);
}
TEST_CASE_FIXTURE(native::Fixture,"LL ADC initialization rejects an already enabled converter") {
    LL_ADC_InitTypeDef c{}; LL_ADC_StructInit(&c); ADC1->CR2=ADC_CR2_ADON;
    CHECK(LL_ADC_Init(ADC1,&c)==ERROR);
}
TEST_CASE_FIXTURE(native::Fixture,"LL regular ADC initialization programs scan length and DMA selection") {
    ADC1->CR1=ADC_CR1_SCAN; // documented prerequisite for a sequencer length >1
    LL_ADC_REG_InitTypeDef c{}; LL_ADC_REG_StructInit(&c);
    c.SequencerLength=LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS;
    c.DMATransfer=LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
    REQUIRE(LL_ADC_REG_Init(ADC1,&c)==SUCCESS);
    CHECK((ADC1->SQR1&ADC_SQR1_L)==(2U<<20)); CHECK((ADC1->CR2&ADC_CR2_DMA)!=0);
}
TEST_CASE_FIXTURE(native::Fixture,"LL injected ADC initialization requests four ranks without enabling conversion") {
    ADC1->CR1=ADC_CR1_SCAN;
    LL_ADC_INJ_InitTypeDef c{}; LL_ADC_INJ_StructInit(&c); c.SequencerLength=LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS;
    REQUIRE(LL_ADC_INJ_Init(ADC1,&c)==SUCCESS);
    CHECK((ADC1->JSQR&ADC_JSQR_JL)==(3U<<20)); CHECK((ADC1->CR2&ADC_CR2_ADON)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"LL ADC common initialization applies its selected divided clock") {
    LL_ADC_CommonInitTypeDef c{}; LL_ADC_CommonStructInit(&c); c.CommonClock=LL_ADC_CLOCK_SYNC_PCLK_DIV8;
    REQUIRE(LL_ADC_CommonInit(ADC,&c)==SUCCESS); CHECK((ADC->CCR&ADC_CCR_ADCPRE)==LL_ADC_CLOCK_SYNC_PCLK_DIV8);
}
TEST_CASE_FIXTURE(native::Fixture,"LL ADC deinitialization clears enabled conversion before resetting configuration") {
    ADC1->CR1=ADC_CR1_SCAN; ADC1->CR2=ADC_CR2_ADON;
    CHECK(LL_ADC_DeInit(ADC1)==SUCCESS); CHECK(ADC1->CR1==0); CHECK((ADC1->CR2&ADC_CR2_ADON)==0);
}
}
#endif
