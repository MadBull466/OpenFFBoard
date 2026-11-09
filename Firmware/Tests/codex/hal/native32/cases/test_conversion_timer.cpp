#include "native.h"
namespace {
unsigned adcDone, timerElapsed;
struct Adc : native::Fixture {
    ADC_HandleTypeDef h{};
    Adc() {
        adcDone=0; h.Instance=ADC1; h.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4;
        h.Init.Resolution=ADC_RESOLUTION_12B; h.Init.DataAlign=ADC_DATAALIGN_RIGHT;
        h.Init.ScanConvMode=DISABLE; h.Init.EOCSelection=ADC_EOC_SINGLE_CONV;
        h.Init.NbrOfConversion=1; h.Init.ExternalTrigConv=ADC_SOFTWARE_START;
    }
};
struct Timer : native::Fixture {
    TIM_HandleTypeDef h{};
    Timer() {
        timerElapsed=0; h.Instance=TIM2; h.Init.Prescaler=15; h.Init.Period=999;
        h.Init.CounterMode=TIM_COUNTERMODE_UP; h.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
        h.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
    }
};
}
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*) { ++adcDone; }
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*) { ++timerElapsed; }
TEST_SUITE("P12NativeADC") {
TEST_CASE_FIXTURE(Adc,"ADC initialization selects resolution conversion count and clock division") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);
    CHECK((ADC1->CR1&ADC_CR1_RES)==ADC_RESOLUTION_12B);
    CHECK((ADC->CCR&ADC_CCR_ADCPRE)==ADC_CLOCK_SYNC_PCLK_DIV4);
    CHECK((ADC1->SQR1&ADC_SQR1_L)==0);
}
TEST_CASE_FIXTURE(Adc,"ADC channel configuration sets independent sequence and sampling fields") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);
    ADC_ChannelConfTypeDef c{}; c.Channel=ADC_CHANNEL_9; c.Rank=1; c.SamplingTime=ADC_SAMPLETIME_84CYCLES;
    REQUIRE(HAL_ADC_ConfigChannel(&h,&c)==HAL_OK);
    CHECK((ADC1->SQR3&31)==9); CHECK(((ADC1->SMPR2>>27)&7)==ADC_SAMPLETIME_84CYCLES);
}
TEST_CASE_FIXTURE(Adc,"ADC software start enables conversion and requests its software trigger") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK); REQUIRE(HAL_ADC_Start(&h)==HAL_OK);
    CHECK((ADC1->CR2&ADC_CR2_ADON)!=0); CHECK((ADC1->CR2&ADC_CR2_SWSTART)!=0);
}
TEST_CASE_FIXTURE(Adc,"ADC polling completes after an explicitly injected conversion event") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK); REQUIRE(HAL_ADC_Start(&h)==HAL_OK);
    native::event([](uint32_t){ADC1->SR=ADC_SR_EOC; ADC1->DR=0xabc;});
    CHECK(HAL_ADC_PollForConversion(&h,5)==HAL_OK); CHECK(HAL_ADC_GetValue(&h)==0xabc);
}
TEST_CASE_FIXTURE(Adc,"ADC polling returns timeout without an end of conversion event") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK); REQUIRE(HAL_ADC_Start(&h)==HAL_OK);
    CHECK(HAL_ADC_PollForConversion(&h,1)==HAL_TIMEOUT); CHECK((h.State&HAL_ADC_STATE_TIMEOUT)!=0);
}
TEST_CASE_FIXTURE(Adc,"ADC interrupt completion calls the public callback") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK); REQUIRE(HAL_ADC_Start_IT(&h)==HAL_OK);
    ADC1->SR=ADC_SR_EOC; HAL_ADC_IRQHandler(&h);
    CHECK(adcDone==1); CHECK((h.State&HAL_ADC_STATE_REG_EOC)!=0);
}
TEST_CASE_FIXTURE(Adc,"ADC analog watchdog preserves distinct low and high threshold requests") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);
    ADC_AnalogWDGConfTypeDef c{}; c.WatchdogMode=ADC_ANALOGWATCHDOG_SINGLE_REG;
    c.LowThreshold=0x123; c.HighThreshold=0xbcd; c.Channel=ADC_CHANNEL_5; c.ITMode=ENABLE;
    REQUIRE(HAL_ADC_AnalogWDGConfig(&h,&c)==HAL_OK);
    CHECK(ADC1->LTR==0x123); CHECK(ADC1->HTR==0xbcd); CHECK((ADC1->CR1&ADC_CR1_AWDCH)==5);
}
TEST_CASE_FIXTURE(Adc,"ADC injected conversion selects the requested rank and returns that data register") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);
    ADC_InjectionConfTypeDef c{}; c.InjectedChannel=ADC_CHANNEL_9; c.InjectedRank=2;
    c.InjectedNbrOfConversion=4; c.InjectedSamplingTime=ADC_SAMPLETIME_84CYCLES;
    c.ExternalTrigInjecConv=ADC_INJECTED_SOFTWARE_START;
    REQUIRE(HAL_ADCEx_InjectedConfigChannel(&h,&c)==HAL_OK);
    CHECK(((ADC1->JSQR>>5)&31)==9);
    ADC1->JDR2=0x456; CHECK(HAL_ADCEx_InjectedGetValue(&h,ADC_INJECTED_RANK_2)==0x456);
}
#if defined(ADC2) && defined(ADC3)
TEST_CASE_FIXTURE(Adc,"ADC multimode configuration changes the common mode without enabling a transfer") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);
    ADC_MultiModeTypeDef c{}; c.Mode=ADC_DUALMODE_REGSIMULT; c.DMAAccessMode=ADC_DMAACCESSMODE_1;
    c.TwoSamplingDelay=ADC_TWOSAMPLINGDELAY_8CYCLES;
    REQUIRE(HAL_ADCEx_MultiModeConfigChannel(&h,&c)==HAL_OK);
    CHECK((ADC->CCR&ADC_CCR_MULTI)==ADC_DUALMODE_REGSIMULT);
    CHECK((ADC->CCR&ADC_CCR_DMA)==ADC_DMAACCESSMODE_1);
}
#endif
TEST_CASE_FIXTURE(Adc,"ADC stop disables conversion and releases ready state") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK); REQUIRE(HAL_ADC_Start(&h)==HAL_OK);
    CHECK(HAL_ADC_Stop(&h)==HAL_OK); CHECK((ADC1->CR2&ADC_CR2_ADON)==0);
    CHECK(HAL_ADC_DeInit(&h)==HAL_OK);
}
}
TEST_SUITE("P12NativeTIM") {
TEST_CASE_FIXTURE(Timer,"timer base initialization programs period prescaler and preload") {
    REQUIRE(HAL_TIM_Base_Init(&h)==HAL_OK);
    CHECK(TIM2->PSC==15); CHECK(TIM2->ARR==999); CHECK((TIM2->CR1&TIM_CR1_ARPE)!=0);
}
TEST_CASE_FIXTURE(Timer,"timer base start and stop control counter enable and public state") {
    REQUIRE(HAL_TIM_Base_Init(&h)==HAL_OK);
    CHECK(HAL_TIM_Base_Start(&h)==HAL_OK); CHECK((TIM2->CR1&TIM_CR1_CEN)!=0);
    CHECK(HAL_TIM_Base_Stop(&h)==HAL_OK); CHECK((TIM2->CR1&TIM_CR1_CEN)==0);
}
TEST_CASE_FIXTURE(Timer,"timer update interrupt invokes only the enabled callback and requests exact flag clearing") {
    REQUIRE(HAL_TIM_Base_Init(&h)==HAL_OK); REQUIRE(HAL_TIM_Base_Start_IT(&h)==HAL_OK);
    TIM2->SR=TIM_SR_UIF|TIM_SR_CC1IF; HAL_TIM_IRQHandler(&h); native::flush();
    CHECK(timerElapsed==1); CHECK((TIM2->SR&TIM_SR_UIF)==0); CHECK((TIM2->SR&TIM_SR_CC1IF)!=0);
}
TEST_CASE_FIXTURE(Timer,"timer PWM configuration preserves the exact compare value") {
    REQUIRE(HAL_TIM_PWM_Init(&h)==HAL_OK);
    TIM_OC_InitTypeDef c{}; c.OCMode=TIM_OCMODE_PWM1; c.Pulse=123; c.OCPolarity=TIM_OCPOLARITY_HIGH;
    REQUIRE(HAL_TIM_PWM_ConfigChannel(&h,&c,TIM_CHANNEL_1)==HAL_OK);
    CHECK(TIM2->CCR1==123); CHECK((TIM2->CCMR1&TIM_CCMR1_OC1M)==TIM_OCMODE_PWM1);
    REQUIRE(HAL_TIM_PWM_Start(&h,TIM_CHANNEL_1)==HAL_OK); CHECK((TIM2->CCER&TIM_CCER_CC1E)!=0);
    CHECK(HAL_TIM_PWM_Stop(&h,TIM_CHANNEL_1)==HAL_OK);
}
TEST_CASE_FIXTURE(Timer,"timer input capture requests falling polarity and the selected filter") {
    REQUIRE(HAL_TIM_IC_Init(&h)==HAL_OK);
    TIM_IC_InitTypeDef c{}; c.ICPolarity=TIM_ICPOLARITY_FALLING; c.ICSelection=TIM_ICSELECTION_DIRECTTI;
    c.ICPrescaler=TIM_ICPSC_DIV4; c.ICFilter=6;
    REQUIRE(HAL_TIM_IC_ConfigChannel(&h,&c,TIM_CHANNEL_3)==HAL_OK);
    CHECK((TIM2->CCER&TIM_CCER_CC3P)!=0); CHECK(((TIM2->CCMR2>>4)&15)==6);
    CHECK(HAL_TIM_IC_Start(&h,TIM_CHANNEL_3)==HAL_OK); CHECK(HAL_TIM_IC_Stop(&h,TIM_CHANNEL_3)==HAL_OK);
}
TEST_CASE_FIXTURE(Timer,"timer master synchronization requests update trigger output") {
    REQUIRE(HAL_TIM_Base_Init(&h)==HAL_OK);
    TIM_MasterConfigTypeDef c{}; c.MasterOutputTrigger=TIM_TRGO_UPDATE; c.MasterSlaveMode=TIM_MASTERSLAVEMODE_ENABLE;
    REQUIRE(HAL_TIMEx_MasterConfigSynchronization(&h,&c)==HAL_OK);
    CHECK((TIM2->CR2&TIM_CR2_MMS)==TIM_TRGO_UPDATE); CHECK((TIM2->SMCR&TIM_SMCR_MSM)!=0);
}
TEST_CASE_FIXTURE(Timer,"timer encoder initialization enables two independently configured channels") {
    TIM_Encoder_InitTypeDef c{}; c.EncoderMode=TIM_ENCODERMODE_TI12;
    c.IC1Selection=c.IC2Selection=TIM_ICSELECTION_DIRECTTI;
    c.IC1Polarity=c.IC2Polarity=TIM_ICPOLARITY_RISING;
    c.IC1Prescaler=c.IC2Prescaler=TIM_ICPSC_DIV1; c.IC1Filter=3; c.IC2Filter=5;
    REQUIRE(HAL_TIM_Encoder_Init(&h,&c)==HAL_OK);
    CHECK((TIM2->SMCR&TIM_SMCR_SMS)==TIM_ENCODERMODE_TI12);
    REQUIRE(HAL_TIM_Encoder_Start(&h,TIM_CHANNEL_ALL)==HAL_OK);
    CHECK((TIM2->CCER&(TIM_CCER_CC1E|TIM_CCER_CC2E))==(TIM_CCER_CC1E|TIM_CCER_CC2E));
    CHECK(HAL_TIM_Encoder_Stop(&h,TIM_CHANNEL_ALL)==HAL_OK);
}
TEST_CASE_FIXTURE(Timer,"advanced timer break configuration preserves deadtime and break polarity") {
    h.Instance=TIM1; REQUIRE(HAL_TIM_PWM_Init(&h)==HAL_OK);
    TIM_BreakDeadTimeConfigTypeDef c{}; c.DeadTime=37; c.BreakState=TIM_BREAK_ENABLE;
    c.BreakPolarity=TIM_BREAKPOLARITY_HIGH; c.AutomaticOutput=TIM_AUTOMATICOUTPUT_ENABLE;
    REQUIRE(HAL_TIMEx_ConfigBreakDeadTime(&h,&c)==HAL_OK);
    CHECK((TIM1->BDTR&TIM_BDTR_DTG)==37); CHECK((TIM1->BDTR&TIM_BDTR_BKE)!=0);
}
}
TEST_SUITE("P12NativeADCRecovery") {
TEST_CASE_FIXTURE(Adc,"ADC injected software start and stop control their dedicated trigger and state") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);ADC_InjectionConfTypeDef c{};
    c.InjectedChannel=ADC_CHANNEL_4;c.InjectedRank=1;c.InjectedNbrOfConversion=1;
    c.InjectedSamplingTime=ADC_SAMPLETIME_15CYCLES;c.ExternalTrigInjecConv=ADC_INJECTED_SOFTWARE_START;
    REQUIRE(HAL_ADCEx_InjectedConfigChannel(&h,&c)==HAL_OK);
    REQUIRE(HAL_ADCEx_InjectedStart(&h)==HAL_OK);CHECK((ADC1->CR2&ADC_CR2_JSWSTART)!=0);
    CHECK((h.State&HAL_ADC_STATE_INJ_BUSY)!=0);REQUIRE(HAL_ADCEx_InjectedStop(&h)==HAL_OK);
    CHECK((ADC1->CR2&ADC_CR2_ADON)==0);CHECK((h.State&HAL_ADC_STATE_INJ_BUSY)==0);
}
TEST_CASE_FIXTURE(Adc,"ADC injected polling observes its separate completion flag and rank value") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);REQUIRE(HAL_ADCEx_InjectedStart(&h)==HAL_OK);
    native::event([](uint32_t){ADC1->SR=ADC_SR_JEOC;ADC1->JDR1=0x71b;});
    REQUIRE(HAL_ADCEx_InjectedPollForConversion(&h,5)==HAL_OK);CHECK(HAL_ADCEx_InjectedGetValue(&h,1)==0x71b);
    CHECK((h.State&HAL_ADC_STATE_INJ_EOC)!=0);
}
TEST_CASE_FIXTURE(Adc,"ADC injected polling times out without a conversion event") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);REQUIRE(HAL_ADCEx_InjectedStart(&h)==HAL_OK);
    CHECK(HAL_ADCEx_InjectedPollForConversion(&h,2)==HAL_TIMEOUT);CHECK((h.State&HAL_ADC_STATE_TIMEOUT)!=0);
}
TEST_CASE_FIXTURE(Adc,"ADC injected interrupt stop disables only injected completion requests") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);REQUIRE(HAL_ADCEx_InjectedStart_IT(&h)==HAL_OK);
    CHECK((ADC1->CR1&ADC_CR1_JEOCIE)!=0);REQUIRE(HAL_ADCEx_InjectedStop_IT(&h)==HAL_OK);
    CHECK((ADC1->CR1&ADC_CR1_JEOCIE)==0);CHECK((h.State&HAL_ADC_STATE_INJ_BUSY)==0);
}
TEST_CASE_FIXTURE(Adc,"ADC watchdog event polling records its watchdog indication") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);ADC1->SR=ADC_SR_AWD;
    CHECK(HAL_ADC_PollForEvent(&h,ADC_AWD_EVENT,2)==HAL_OK);CHECK((h.State&HAL_ADC_STATE_AWD1)!=0);
}
TEST_CASE_FIXTURE(Adc,"ADC overrun event polling records its data loss error") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);native::flush();
    ADC1->SR=ADC_SR_OVR;CHECK(HAL_ADC_PollForEvent(&h,ADC_OVR_EVENT,2)==HAL_OK);
    CHECK((HAL_ADC_GetError(&h)&HAL_ADC_ERROR_OVR)!=0);
}
TEST_CASE_FIXTURE(Adc,"ADC DMA start programs the real stream and completion delivers the sampled value") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);DMA_HandleTypeDef d{};
    d.Instance=DMA2_Stream0;d.Init.Direction=DMA_PERIPH_TO_MEMORY;
    d.Init.MemDataAlignment=DMA_MDATAALIGN_WORD;d.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK);d.Parent=&h;h.DMA_Handle=&d;uint32_t value=0;
    REQUIRE(HAL_ADC_Start_DMA(&h,&value,1)==HAL_OK);ADC1->DR=0x987;ADC1->SR=ADC_SR_EOC;
    native::completeDMA(d);HAL_DMA_IRQHandler(&d);CHECK(value==0x987);CHECK(adcDone==1);
    REQUIRE(HAL_ADC_Stop_DMA(&h)==HAL_OK);CHECK((ADC1->CR2&ADC_CR2_DMA)==0);
}
TEST_CASE_FIXTURE(Adc,"ADC interrupt stop clears conversion and overrun interrupt requests") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);REQUIRE(HAL_ADC_Start_IT(&h)==HAL_OK);
    REQUIRE(HAL_ADC_Stop_IT(&h)==HAL_OK);CHECK((ADC1->CR1&(ADC_CR1_EOCIE|ADC_CR1_OVRIE))==0);
    CHECK((ADC1->CR2&ADC_CR2_ADON)==0);
}
}
TEST_SUITE("P12NativeADCSequences") {
TEST_CASE_FIXTURE(Adc,"ADC continuous scan DMA delivers a guarded sequence and remains active until stopped") {
    h.Init.ScanConvMode=ENABLE;h.Init.NbrOfConversion=3;h.Init.ContinuousConvMode=ENABLE;h.Init.DMAContinuousRequests=ENABLE;
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);DMA_HandleTypeDef d{};
    d.Instance=DMA2_Stream0;d.Init.Direction=DMA_PERIPH_TO_MEMORY;d.Init.MemInc=DMA_MINC_ENABLE;d.Init.Mode=DMA_CIRCULAR;
    d.Init.MemDataAlignment=DMA_MDATAALIGN_WORD;d.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK);d.Parent=&h;h.DMA_Handle=&d;uint32_t values[]={0xdeadbeef,0,0,0,0xdeadbeef};
    REQUIRE(HAL_ADC_Start_DMA(&h,values+1,3)==HAL_OK);native::adcInput(0,{0x123,0x456,0x789});
    native::completeDMA(d);HAL_DMA_IRQHandler(&d);
    CHECK(values[0]==0xdeadbeef);CHECK(values[1]==0x123);CHECK(values[2]==0x456);CHECK(values[3]==0x789);CHECK(values[4]==0xdeadbeef);
    CHECK(adcDone==1);CHECK((HAL_ADC_GetState(&h)&HAL_ADC_STATE_REG_BUSY)!=0);
    REQUIRE(HAL_ADC_Stop_DMA(&h)==HAL_OK);CHECK((h.State&HAL_ADC_STATE_REG_BUSY)==0);
}
#if defined(ADC2) && defined(ADC3)
TEST_CASE_FIXTURE(Adc,"ADC multimode DMA delivers the supplied packed common data word") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);ADC_MultiModeTypeDef c{};
    c.Mode=ADC_DUALMODE_REGSIMULT;c.DMAAccessMode=ADC_DMAACCESSMODE_2;c.TwoSamplingDelay=ADC_TWOSAMPLINGDELAY_5CYCLES;
    REQUIRE(HAL_ADCEx_MultiModeConfigChannel(&h,&c)==HAL_OK);DMA_HandleTypeDef d{};
    d.Instance=DMA2_Stream0;d.Init.Direction=DMA_PERIPH_TO_MEMORY;
    d.Init.MemDataAlignment=DMA_MDATAALIGN_WORD;d.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK);d.Parent=&h;h.DMA_Handle=&d;uint32_t packed=0;
    REQUIRE(HAL_ADCEx_MultiModeStart_DMA(&h,&packed,1)==HAL_OK);ADC->CDR=0x04560123;
    native::completeDMA(d);HAL_DMA_IRQHandler(&d);
    CHECK(packed==0x04560123);CHECK(HAL_ADCEx_MultiModeGetValue(&h)==0x04560123);CHECK(adcDone==1);
}
TEST_CASE_FIXTURE(Adc,"ADC multimode DMA stop aborts an active stream and disables the master") {
    REQUIRE(HAL_ADC_Init(&h)==HAL_OK);ADC_MultiModeTypeDef c{};
    c.Mode=ADC_DUALMODE_REGSIMULT;c.DMAAccessMode=ADC_DMAACCESSMODE_2;c.TwoSamplingDelay=ADC_TWOSAMPLINGDELAY_5CYCLES;
    REQUIRE(HAL_ADCEx_MultiModeConfigChannel(&h,&c)==HAL_OK);DMA_HandleTypeDef d{};
    d.Instance=DMA2_Stream0;d.Init.Direction=DMA_PERIPH_TO_MEMORY;
    d.Init.MemDataAlignment=DMA_MDATAALIGN_WORD;d.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK);d.Parent=&h;h.DMA_Handle=&d;uint32_t packed=0;
    REQUIRE(HAL_ADCEx_MultiModeStart_DMA(&h,&packed,1)==HAL_OK);
    REQUIRE(HAL_ADCEx_MultiModeStop_DMA(&h)==HAL_OK);CHECK((ADC1->CR2&ADC_CR2_ADON)==0);
    CHECK((d.Instance->CR&DMA_SxCR_EN)==0);CHECK((h.State&HAL_ADC_STATE_REG_BUSY)==0);
}

#endif
}
