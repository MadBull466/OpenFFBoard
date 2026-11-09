#include "native.h"
namespace {
struct Tim : native::Fixture {
    TIM_HandleTypeDef h{}; DMA_HandleTypeDef dma[7]{};
    Tim() {
        h.Instance=TIM2; h.Init.Prescaler=7; h.Init.Period=4095;
        h.Init.CounterMode=TIM_COUNTERMODE_UP; h.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
        for(unsigned i=0;i<7;++i) {
            dma[i].Instance=reinterpret_cast<DMA_Stream_TypeDef*>(DMA1_Stream0_BASE+24*i);
            dma[i].Init.Direction=DMA_MEMORY_TO_PERIPH; dma[i].Init.MemInc=DMA_MINC_ENABLE;
            dma[i].Init.MemDataAlignment=DMA_MDATAALIGN_WORD; dma[i].Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;
            if(HAL_DMA_Init(&dma[i])!=HAL_OK) throw std::runtime_error("DMA fixture init");
            dma[i].Parent=&h; h.hdma[i]=&dma[i];
        }
    }
    TIM_OC_InitTypeDef output() {
        TIM_OC_InitTypeDef c{}; c.OCMode=TIM_OCMODE_PWM1; c.Pulse=731;
        c.OCPolarity=TIM_OCPOLARITY_HIGH; c.OCNPolarity=TIM_OCNPOLARITY_HIGH; return c;
    }
};
}
TEST_SUITE("P12NativeTimerModes") {
TEST_CASE_FIXTURE(Tim,"timer output compare configures and enables each independent channel") {
    REQUIRE(HAL_TIM_OC_Init(&h)==HAL_OK); auto c=output();c.OCMode=TIM_OCMODE_TOGGLE;
    for(auto channel:{TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4}) {
        REQUIRE(HAL_TIM_OC_ConfigChannel(&h,&c,channel)==HAL_OK);
        REQUIRE(HAL_TIM_OC_Start(&h,channel)==HAL_OK);
        CHECK((TIM2->CCER&(TIM_CCER_CC1E<<channel))!=0);
        CHECK(HAL_TIM_OC_Stop(&h,channel)==HAL_OK); CHECK((TIM2->CCER&(TIM_CCER_CC1E<<channel))==0);
    }
}
TEST_CASE_FIXTURE(Tim,"timer output compare interrupt start enables only its selected request") {
    REQUIRE(HAL_TIM_OC_Init(&h)==HAL_OK); auto c=output();c.OCMode=TIM_OCMODE_TOGGLE;
    REQUIRE(HAL_TIM_OC_ConfigChannel(&h,&c,TIM_CHANNEL_2)==HAL_OK);
    REQUIRE(HAL_TIM_OC_Start_IT(&h,TIM_CHANNEL_2)==HAL_OK); CHECK((TIM2->DIER&TIM_DIER_CC2IE)!=0);
    CHECK(HAL_TIM_OC_Stop_IT(&h,TIM_CHANNEL_2)==HAL_OK); CHECK((TIM2->DIER&TIM_DIER_CC2IE)==0);
}
TEST_CASE_FIXTURE(Tim,"timer PWM interrupt start and stop preserve another enabled channel") {
    REQUIRE(HAL_TIM_PWM_Init(&h)==HAL_OK); auto c=output();
    REQUIRE(HAL_TIM_PWM_ConfigChannel(&h,&c,TIM_CHANNEL_1)==HAL_OK);
    REQUIRE(HAL_TIM_PWM_ConfigChannel(&h,&c,TIM_CHANNEL_3)==HAL_OK);
    REQUIRE(HAL_TIM_PWM_Start(&h,TIM_CHANNEL_3)==HAL_OK);
    REQUIRE(HAL_TIM_PWM_Start_IT(&h,TIM_CHANNEL_1)==HAL_OK);
    CHECK(HAL_TIM_PWM_Stop_IT(&h,TIM_CHANNEL_1)==HAL_OK);
    CHECK((TIM2->CCER&TIM_CCER_CC3E)!=0); CHECK((TIM2->CR1&TIM_CR1_CEN)!=0);
}
TEST_CASE_FIXTURE(Tim,"timer input capture interrupt mode changes the selected capture request") {
    REQUIRE(HAL_TIM_IC_Init(&h)==HAL_OK);
    TIM_IC_InitTypeDef c{}; c.ICSelection=TIM_ICSELECTION_DIRECTTI; c.ICPolarity=TIM_ICPOLARITY_RISING;
    REQUIRE(HAL_TIM_IC_ConfigChannel(&h,&c,TIM_CHANNEL_4)==HAL_OK);
    REQUIRE(HAL_TIM_IC_Start_IT(&h,TIM_CHANNEL_4)==HAL_OK); CHECK((TIM2->DIER&TIM_DIER_CC4IE)!=0);
    CHECK(HAL_TIM_IC_Stop_IT(&h,TIM_CHANNEL_4)==HAL_OK); CHECK((TIM2->DIER&TIM_DIER_CC4IE)==0);
}
TEST_CASE_FIXTURE(Tim,"timer base DMA requests the autoreload endpoint and exact element count") {
    REQUIRE(HAL_TIM_Base_Init(&h)==HAL_OK); uint32_t periods[]={111,222,333};
    REQUIRE(HAL_TIM_Base_Start_DMA(&h,periods,3)==HAL_OK);
    CHECK(dma[TIM_DMA_ID_UPDATE].Instance->PAR==reinterpret_cast<uint32_t>(&TIM2->ARR));
    CHECK(dma[TIM_DMA_ID_UPDATE].Instance->NDTR==3); CHECK((TIM2->DIER&TIM_DIER_UDE)!=0);
    CHECK(HAL_TIM_Base_Stop_DMA(&h)==HAL_OK); CHECK((TIM2->DIER&TIM_DIER_UDE)==0);
}
TEST_CASE_FIXTURE(Tim,"timer PWM DMA programs its selected compare register and buffer") {
    REQUIRE(HAL_TIM_PWM_Init(&h)==HAL_OK); auto c=output();
    REQUIRE(HAL_TIM_PWM_ConfigChannel(&h,&c,TIM_CHANNEL_2)==HAL_OK); uint32_t pulses[]={111,222,333};
    REQUIRE(HAL_TIM_PWM_Start_DMA(&h,TIM_CHANNEL_2,pulses,3)==HAL_OK);
    CHECK(dma[TIM_DMA_ID_CC2].Instance->PAR==reinterpret_cast<uint32_t>(&TIM2->CCR2));
    CHECK(dma[TIM_DMA_ID_CC2].Instance->M0AR==reinterpret_cast<uint32_t>(pulses));
    CHECK(HAL_TIM_PWM_Stop_DMA(&h,TIM_CHANNEL_2)==HAL_OK);
}
TEST_CASE_FIXTURE(Tim,"timer output compare DMA requests the independently selected compare endpoint") {
    REQUIRE(HAL_TIM_OC_Init(&h)==HAL_OK); auto c=output();c.OCMode=TIM_OCMODE_TOGGLE;
    REQUIRE(HAL_TIM_OC_ConfigChannel(&h,&c,TIM_CHANNEL_4)==HAL_OK); uint32_t values[]={0x123,0x456};
    REQUIRE(HAL_TIM_OC_Start_DMA(&h,TIM_CHANNEL_4,values,2)==HAL_OK);
    CHECK(dma[TIM_DMA_ID_CC4].Instance->PAR==reinterpret_cast<uint32_t>(&TIM2->CCR4));
    CHECK(HAL_TIM_OC_Stop_DMA(&h,TIM_CHANNEL_4)==HAL_OK);
}
TEST_CASE_FIXTURE(Tim,"timer input capture DMA requests a peripheral-to-memory capture stream") {
    REQUIRE(HAL_TIM_IC_Init(&h)==HAL_OK);
    auto &d=dma[TIM_DMA_ID_CC3]; d.Init.Direction=DMA_PERIPH_TO_MEMORY; REQUIRE(HAL_DMA_Init(&d)==HAL_OK);
    uint32_t captures[2]{};
    REQUIRE(HAL_TIM_IC_Start_DMA(&h,TIM_CHANNEL_3,captures,2)==HAL_OK);
    CHECK(d.Instance->PAR==reinterpret_cast<uint32_t>(&TIM2->CCR3)); CHECK(d.Instance->M0AR==reinterpret_cast<uint32_t>(captures));
    CHECK(HAL_TIM_IC_Stop_DMA(&h,TIM_CHANNEL_3)==HAL_OK);
}
TEST_CASE_FIXTURE(Tim,"timer slave reset mode selects the requested trigger input") {
    REQUIRE(HAL_TIM_Base_Init(&h)==HAL_OK); TIM_SlaveConfigTypeDef c{};
    c.SlaveMode=TIM_SLAVEMODE_RESET; c.InputTrigger=TIM_TS_TI1FP1; c.TriggerPolarity=TIM_TRIGGERPOLARITY_RISING;
    c.TriggerPrescaler=TIM_TRIGGERPRESCALER_DIV1; c.TriggerFilter=5;
    REQUIRE(HAL_TIM_SlaveConfigSynchro(&h,&c)==HAL_OK);
    CHECK((TIM2->SMCR&TIM_SMCR_SMS)==TIM_SLAVEMODE_RESET); CHECK((TIM2->SMCR&TIM_SMCR_TS)==TIM_TS_TI1FP1);
}
TEST_CASE_FIXTURE(Tim,"timer external clock selection requests the selected ETR filter") {
    REQUIRE(HAL_TIM_Base_Init(&h)==HAL_OK); TIM_ClockConfigTypeDef c{};
    c.ClockSource=TIM_CLOCKSOURCE_ETRMODE2; c.ClockPolarity=TIM_CLOCKPOLARITY_NONINVERTED;
    c.ClockPrescaler=TIM_CLOCKPRESCALER_DIV1; c.ClockFilter=6;
    REQUIRE(HAL_TIM_ConfigClockSource(&h,&c)==HAL_OK);
    CHECK((TIM2->SMCR&TIM_SMCR_ECE)!=0); CHECK(((TIM2->SMCR&TIM_SMCR_ETF)>>8)==6);
}
TEST_CASE_FIXTURE(Tim,"timer DMA burst write preserves its requested register base and burst length") {
    REQUIRE(HAL_TIM_Base_Init(&h)==HAL_OK); uint32_t values[]={111,222,333};
    REQUIRE(HAL_TIM_DMABurst_WriteStart(&h,TIM_DMABASE_ARR,TIM_DMA_UPDATE,values,TIM_DMABURSTLENGTH_3TRANSFERS)==HAL_OK);
    CHECK((TIM2->DCR&TIM_DCR_DBA)==TIM_DMABASE_ARR);
    CHECK((TIM2->DCR&TIM_DCR_DBL)==TIM_DMABURSTLENGTH_3TRANSFERS);
    CHECK(dma[TIM_DMA_ID_UPDATE].Instance->PAR==reinterpret_cast<uint32_t>(&TIM2->DMAR));
    CHECK(HAL_TIM_DMABurst_WriteStop(&h,TIM_DMA_UPDATE)==HAL_OK);
}
TEST_CASE_FIXTURE(Tim,"advanced timer complementary PWM changes its N output independently") {
    h.Instance=TIM1; REQUIRE(HAL_TIM_PWM_Init(&h)==HAL_OK); auto c=output();
    REQUIRE(HAL_TIM_PWM_ConfigChannel(&h,&c,TIM_CHANNEL_1)==HAL_OK);
    REQUIRE(HAL_TIMEx_PWMN_Start(&h,TIM_CHANNEL_1)==HAL_OK);
    CHECK((TIM1->CCER&TIM_CCER_CC1NE)!=0); CHECK((TIM1->BDTR&TIM_BDTR_MOE)!=0);
    CHECK(HAL_TIMEx_PWMN_Stop(&h,TIM_CHANNEL_1)==HAL_OK); CHECK((TIM1->CCER&TIM_CCER_CC1NE)==0);
}
TEST_CASE_FIXTURE(Tim,"advanced timer complementary output compare interrupt controls the selected N output") {
    h.Instance=TIM1; REQUIRE(HAL_TIM_OC_Init(&h)==HAL_OK); auto c=output();c.OCMode=TIM_OCMODE_TOGGLE;
    REQUIRE(HAL_TIM_OC_ConfigChannel(&h,&c,TIM_CHANNEL_2)==HAL_OK);
    REQUIRE(HAL_TIMEx_OCN_Start_IT(&h,TIM_CHANNEL_2)==HAL_OK); CHECK((TIM1->CCER&TIM_CCER_CC2NE)!=0);
    CHECK(HAL_TIMEx_OCN_Stop_IT(&h,TIM_CHANNEL_2)==HAL_OK); CHECK((TIM1->CCER&TIM_CCER_CC2NE)==0);
}
TEST_CASE_FIXTURE(Tim,"timer hall configuration requests the XOR input and independent commutation delay") {
    TIM_HallSensor_InitTypeDef c{}; c.IC1Polarity=TIM_ICPOLARITY_RISING; c.IC1Prescaler=TIM_ICPSC_DIV1;
    c.IC1Filter=4; c.Commutation_Delay=37;
    REQUIRE(HAL_TIMEx_HallSensor_Init(&h,&c)==HAL_OK);
    CHECK((TIM2->CR2&TIM_CR2_TI1S)!=0); CHECK(TIM2->CCR2==37);
    REQUIRE(HAL_TIMEx_HallSensor_Start_IT(&h)==HAL_OK); CHECK((TIM2->DIER&TIM_DIER_CC1IE)!=0);
    CHECK(HAL_TIMEx_HallSensor_Stop_IT(&h)==HAL_OK);
}
TEST_CASE_FIXTURE(Tim,"timer one-pulse setup enables paired trigger and output channels") {
    REQUIRE(HAL_TIM_OnePulse_Init(&h,TIM_OPMODE_SINGLE)==HAL_OK);
    TIM_OnePulse_InitTypeDef c{};c.OCMode=TIM_OCMODE_PWM2;c.Pulse=73;c.OCPolarity=TIM_OCPOLARITY_HIGH;
    c.ICPolarity=TIM_ICPOLARITY_RISING;c.ICSelection=TIM_ICSELECTION_DIRECTTI;c.ICFilter=3;
    REQUIRE(HAL_TIM_OnePulse_ConfigChannel(&h,&c,TIM_CHANNEL_1,TIM_CHANNEL_2)==HAL_OK);
    REQUIRE(HAL_TIM_OnePulse_Start(&h,TIM_CHANNEL_1)==HAL_OK);
    CHECK(TIM2->CCR1==73);CHECK((TIM2->CCER&(TIM_CCER_CC1E|TIM_CCER_CC2E))==(TIM_CCER_CC1E|TIM_CCER_CC2E));
    CHECK(HAL_TIM_OnePulse_Stop(&h,TIM_CHANNEL_1)==HAL_OK);
}
}
TEST_SUITE("P12NativeTimerExtendedLifecycle") {
TEST_CASE_FIXTURE(Tim,"advanced timer complementary output compare starts and stops each supported N channel") {
    h.Instance=TIM1;REQUIRE(HAL_TIM_OC_Init(&h)==HAL_OK);auto c=output();c.OCMode=TIM_OCMODE_TOGGLE;
    for(auto channel:{TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3}) {
        REQUIRE(HAL_TIM_OC_ConfigChannel(&h,&c,channel)==HAL_OK);
        REQUIRE(HAL_TIMEx_OCN_Start(&h,channel)==HAL_OK); CHECK((TIM1->CCER&(TIM_CCER_CC1NE<<channel))!=0);
        REQUIRE(HAL_TIMEx_OCN_Stop(&h,channel)==HAL_OK); CHECK((TIM1->CCER&(TIM_CCER_CC1NE<<channel))==0);
    }
}
TEST_CASE_FIXTURE(Tim,"advanced timer complementary PWM interrupt selects each N channel independently") {
    h.Instance=TIM1;REQUIRE(HAL_TIM_PWM_Init(&h)==HAL_OK);auto c=output();
    for(auto channel:{TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3}) {
        REQUIRE(HAL_TIM_PWM_ConfigChannel(&h,&c,channel)==HAL_OK);
        REQUIRE(HAL_TIMEx_PWMN_Start_IT(&h,channel)==HAL_OK);CHECK((TIM1->CCER&(TIM_CCER_CC1NE<<channel))!=0);
        CHECK((TIM1->DIER&(TIM_DIER_CC1IE<<(channel/4)))!=0);
        REQUIRE(HAL_TIMEx_PWMN_Stop_IT(&h,channel)==HAL_OK);CHECK((TIM1->CCER&(TIM_CCER_CC1NE<<channel))==0);
    }
}
TEST_CASE_FIXTURE(Tim,"advanced timer complementary PWM DMA selects the corresponding compare register") {
    h.Instance=TIM1;REQUIRE(HAL_TIM_PWM_Init(&h)==HAL_OK);auto c=output();uint32_t pulses[]={11,22};
    for(auto channel:{TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3}) {
        REQUIRE(HAL_TIM_PWM_ConfigChannel(&h,&c,channel)==HAL_OK);
        REQUIRE(HAL_TIMEx_PWMN_Start_DMA(&h,channel,pulses,2)==HAL_OK);
        auto &d=dma[TIM_DMA_ID_CC1+channel/4];CHECK(d.Instance->PAR==reinterpret_cast<uint32_t>(&TIM1->CCR1)+channel);
        REQUIRE(HAL_TIMEx_PWMN_Stop_DMA(&h,channel)==HAL_OK);CHECK((d.Instance->CR&DMA_SxCR_EN)==0);
    }
}
TEST_CASE_FIXTURE(Tim,"advanced timer complementary output compare DMA selects each compare register") {
    h.Instance=TIM1;REQUIRE(HAL_TIM_OC_Init(&h)==HAL_OK);auto c=output();c.OCMode=TIM_OCMODE_TOGGLE;uint32_t pulses[]={11,22};
    for(auto channel:{TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3}) {
        REQUIRE(HAL_TIM_OC_ConfigChannel(&h,&c,channel)==HAL_OK);
        REQUIRE(HAL_TIMEx_OCN_Start_DMA(&h,channel,pulses,2)==HAL_OK);
        auto &d=dma[TIM_DMA_ID_CC1+channel/4];CHECK(d.Instance->PAR==reinterpret_cast<uint32_t>(&TIM1->CCR1)+channel);
        REQUIRE(HAL_TIMEx_OCN_Stop_DMA(&h,channel)==HAL_OK);CHECK((d.Instance->CR&DMA_SxCR_EN)==0);
    }
}
TEST_CASE_FIXTURE(Tim,"timer Hall polling mode and deinitialization release their capture output") {
    TIM_HallSensor_InitTypeDef c{};
    c.IC1Polarity=TIM_ICPOLARITY_RISING;c.IC1Prescaler=TIM_ICPSC_DIV1;
    REQUIRE(HAL_TIMEx_HallSensor_Init(&h,&c)==HAL_OK);REQUIRE(HAL_TIMEx_HallSensor_Start(&h)==HAL_OK);
    CHECK((TIM2->CCER&TIM_CCER_CC1E)!=0);REQUIRE(HAL_TIMEx_HallSensor_Stop(&h)==HAL_OK);
    CHECK((TIM2->CCER&TIM_CCER_CC1E)==0);CHECK(HAL_TIMEx_HallSensor_DeInit(&h)==HAL_OK);
}
TEST_CASE_FIXTURE(Tim,"timer Hall DMA requests capture channel one and disables it on stop") {
    TIM_HallSensor_InitTypeDef c{};c.IC1Polarity=TIM_ICPOLARITY_RISING;c.IC1Prescaler=TIM_ICPSC_DIV1;
    REQUIRE(HAL_TIMEx_HallSensor_Init(&h,&c)==HAL_OK);
    auto &d=dma[TIM_DMA_ID_CC1];d.Init.Direction=DMA_PERIPH_TO_MEMORY;REQUIRE(HAL_DMA_Init(&d)==HAL_OK);
    uint32_t captures[2]{};REQUIRE(HAL_TIMEx_HallSensor_Start_DMA(&h,captures,2)==HAL_OK);
    CHECK(d.Instance->PAR==reinterpret_cast<uint32_t>(&TIM2->CCR1));CHECK((TIM2->DIER&TIM_DIER_CC1DE)!=0);
    REQUIRE(HAL_TIMEx_HallSensor_Stop_DMA(&h)==HAL_OK);CHECK((TIM2->DIER&TIM_DIER_CC1DE)==0);
}
TEST_CASE_FIXTURE(Tim,"timer DMA burst read selects the capture endpoint and peripheral direction") {
    REQUIRE(HAL_TIM_Base_Init(&h)==HAL_OK);auto &d=dma[TIM_DMA_ID_CC2];
    d.Init.Direction=DMA_PERIPH_TO_MEMORY;REQUIRE(HAL_DMA_Init(&d)==HAL_OK);uint32_t data[3]{};
    REQUIRE(HAL_TIM_DMABurst_ReadStart(&h,TIM_DMABASE_CCR1,TIM_DMA_CC2,data,TIM_DMABURSTLENGTH_3TRANSFERS)==HAL_OK);
    CHECK(d.Instance->PAR==reinterpret_cast<uint32_t>(&TIM2->DMAR));CHECK(d.Instance->NDTR==3);
    CHECK((TIM2->DCR&TIM_DCR_DBA)==TIM_DMABASE_CCR1);
    REQUIRE(HAL_TIM_DMABurst_ReadStop(&h,TIM_DMA_CC2)==HAL_OK);CHECK((TIM2->DIER&TIM_DIER_CC2DE)==0);
}
TEST_CASE_FIXTURE(Tim,"timer captured-value access distinguishes all four capture channels") {
    TIM2->CCR1=101;TIM2->CCR2=202;TIM2->CCR3=303;TIM2->CCR4=404;
    CHECK(HAL_TIM_ReadCapturedValue(&h,TIM_CHANNEL_1)==101);CHECK(HAL_TIM_ReadCapturedValue(&h,TIM_CHANNEL_2)==202);
    CHECK(HAL_TIM_ReadCapturedValue(&h,TIM_CHANNEL_3)==303);CHECK(HAL_TIM_ReadCapturedValue(&h,TIM_CHANNEL_4)==404);
}
TEST_CASE_FIXTURE(Tim,"advanced timer commutation interrupt selects its trigger and software source") {
    h.Instance=TIM1;REQUIRE(HAL_TIM_Base_Init(&h)==HAL_OK);
    REQUIRE(HAL_TIMEx_ConfigCommutEvent_IT(&h,TIM_TS_ITR2,TIM_COMMUTATION_SOFTWARE)==HAL_OK);
    CHECK((TIM1->SMCR&TIM_SMCR_TS)==TIM_TS_ITR2);CHECK((TIM1->DIER&TIM_DIER_COMIE)!=0);
    CHECK((TIM1->CR2&TIM_CR2_CCPC)!=0);CHECK((TIM1->CR2&TIM_CR2_CCUS)==0);
}
TEST_CASE_FIXTURE(Tim,"advanced timer commutation DMA selects trigger-driven requests") {
    h.Instance=TIM1;REQUIRE(HAL_TIM_Base_Init(&h)==HAL_OK);
    REQUIRE(HAL_TIMEx_ConfigCommutEvent_DMA(&h,TIM_TS_ITR1,TIM_COMMUTATION_TRGI)==HAL_OK);
    CHECK((TIM1->SMCR&TIM_SMCR_TS)==TIM_TS_ITR1);CHECK((TIM1->DIER&TIM_DIER_COMDE)!=0);
    CHECK((TIM1->CR2&TIM_CR2_CCUS)!=0);CHECK(dma[TIM_DMA_ID_COMMUTATION].XferCpltCallback!=nullptr);
}
}
namespace {unsigned pwmDone,captureDone;}
extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef*) {++pwmDone;}
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef*) {++captureDone;}
TEST_SUITE("P12NativeTimerDMAEvents") {
TEST_CASE_FIXTURE(Tim,"timer base DMA completion applies all supplied reload values and releases its state") {
    REQUIRE(HAL_TIM_Base_Init(&h)==HAL_OK);uint32_t values[]={123,456,789};
    REQUIRE(HAL_TIM_Base_Start_DMA(&h,values,3)==HAL_OK);
    native::completeDMA(dma[TIM_DMA_ID_UPDATE]);HAL_DMA_IRQHandler(&dma[TIM_DMA_ID_UPDATE]);
    CHECK(TIM2->ARR==789);CHECK(HAL_TIM_Base_GetState(&h)==HAL_TIM_STATE_READY);
}
TEST_CASE_FIXTURE(Tim,"timer PWM DMA completion reports a finished pulse for the selected channel") {
    pwmDone=0;REQUIRE(HAL_TIM_PWM_Init(&h)==HAL_OK);auto c=output();
    REQUIRE(HAL_TIM_PWM_ConfigChannel(&h,&c,TIM_CHANNEL_3)==HAL_OK);uint32_t values[]={123,456};
    REQUIRE(HAL_TIM_PWM_Start_DMA(&h,TIM_CHANNEL_3,values,2)==HAL_OK);
    native::completeDMA(dma[TIM_DMA_ID_CC3]);HAL_DMA_IRQHandler(&dma[TIM_DMA_ID_CC3]);
    CHECK(TIM2->CCR3==456);CHECK(pwmDone==1);CHECK(HAL_TIM_GetChannelState(&h,TIM_CHANNEL_3)==HAL_TIM_CHANNEL_STATE_READY);
    CHECK(HAL_TIM_GetActiveChannel(&h)==HAL_TIM_ACTIVE_CHANNEL_CLEARED);
}
TEST_CASE_FIXTURE(Tim,"timer capture DMA completion delivers the supplied captures before notifying completion") {
    captureDone=0;REQUIRE(HAL_TIM_IC_Init(&h)==HAL_OK);auto &d=dma[TIM_DMA_ID_CC4];
    d.Init.Direction=DMA_PERIPH_TO_MEMORY;REQUIRE(HAL_DMA_Init(&d)==HAL_OK);uint32_t data[2]{};
    REQUIRE(HAL_TIM_IC_Start_DMA(&h,TIM_CHANNEL_4,data,2)==HAL_OK);TIM2->CCR4=0x123456;
    native::completeDMA(d);HAL_DMA_IRQHandler(&d);CHECK(data[0]==0x123456);CHECK(data[1]==0x123456);
    CHECK(captureDone==1);CHECK(HAL_TIM_GetChannelState(&h,TIM_CHANNEL_4)==HAL_TIM_CHANNEL_STATE_READY);
}
TEST_CASE_FIXTURE(Tim,"advanced timer complementary DMA completion releases the selected N channel") {
    pwmDone=0;h.Instance=TIM1;REQUIRE(HAL_TIM_PWM_Init(&h)==HAL_OK);auto c=output();uint32_t pulses[]={13,57};
    REQUIRE(HAL_TIM_PWM_ConfigChannel(&h,&c,TIM_CHANNEL_2)==HAL_OK);
    REQUIRE(HAL_TIMEx_PWMN_Start_DMA(&h,TIM_CHANNEL_2,pulses,2)==HAL_OK);
    native::completeDMA(dma[TIM_DMA_ID_CC2]);HAL_DMA_IRQHandler(&dma[TIM_DMA_ID_CC2]);
    CHECK(TIM1->CCR2==57);CHECK(pwmDone==1);CHECK(HAL_TIMEx_GetChannelNState(&h,TIM_CHANNEL_2)==HAL_TIM_CHANNEL_STATE_READY);
}
TEST_CASE_FIXTURE(Tim,"timer encoder interrupt lifecycle requests both capture channels and releases them on stop") {
    TIM_Encoder_InitTypeDef c{};c.EncoderMode=TIM_ENCODERMODE_TI12;
    c.IC1Selection=c.IC2Selection=TIM_ICSELECTION_DIRECTTI;c.IC1Polarity=c.IC2Polarity=TIM_ICPOLARITY_RISING;
    REQUIRE(HAL_TIM_Encoder_Init(&h,&c)==HAL_OK);REQUIRE(HAL_TIM_Encoder_Start_IT(&h,TIM_CHANNEL_ALL)==HAL_OK);
    CHECK((TIM2->DIER&(TIM_DIER_CC1IE|TIM_DIER_CC2IE))==(TIM_DIER_CC1IE|TIM_DIER_CC2IE));
    REQUIRE(HAL_TIM_Encoder_Stop_IT(&h,TIM_CHANNEL_ALL)==HAL_OK);CHECK((TIM2->DIER&(TIM_DIER_CC1IE|TIM_DIER_CC2IE))==0);
    CHECK(HAL_TIM_Encoder_DeInit(&h)==HAL_OK);
}
TEST_CASE_FIXTURE(Tim,"timer encoder DMA assigns independent capture endpoints for both encoder channels") {
    TIM_Encoder_InitTypeDef c{};c.EncoderMode=TIM_ENCODERMODE_TI12;
    c.IC1Selection=c.IC2Selection=TIM_ICSELECTION_DIRECTTI;c.IC1Polarity=c.IC2Polarity=TIM_ICPOLARITY_RISING;
    REQUIRE(HAL_TIM_Encoder_Init(&h,&c)==HAL_OK);
    for(unsigned i:{TIM_DMA_ID_CC1,TIM_DMA_ID_CC2}) {dma[i].Init.Direction=DMA_PERIPH_TO_MEMORY;REQUIRE(HAL_DMA_Init(&dma[i])==HAL_OK);}
    uint32_t a[2]{},b[2]{};REQUIRE(HAL_TIM_Encoder_Start_DMA(&h,TIM_CHANNEL_ALL,a,b,2)==HAL_OK);
    CHECK(dma[TIM_DMA_ID_CC1].Instance->PAR==reinterpret_cast<uint32_t>(&TIM2->CCR1));
    CHECK(dma[TIM_DMA_ID_CC2].Instance->PAR==reinterpret_cast<uint32_t>(&TIM2->CCR2));
    REQUIRE(HAL_TIM_Encoder_Stop_DMA(&h,TIM_CHANNEL_ALL)==HAL_OK);
    CHECK((TIM2->DIER&(TIM_DIER_CC1DE|TIM_DIER_CC2DE))==0);
}
TEST_CASE_FIXTURE(Tim,"timer one-pulse interrupt lifecycle enables and disables paired channels") {
    REQUIRE(HAL_TIM_OnePulse_Init(&h,TIM_OPMODE_SINGLE)==HAL_OK);
    REQUIRE(HAL_TIM_OnePulse_Start_IT(&h,TIM_CHANNEL_2)==HAL_OK);
    CHECK((TIM2->DIER&(TIM_DIER_CC1IE|TIM_DIER_CC2IE))==(TIM_DIER_CC1IE|TIM_DIER_CC2IE));
    REQUIRE(HAL_TIM_OnePulse_Stop_IT(&h,TIM_CHANNEL_2)==HAL_OK);CHECK((TIM2->DIER&(TIM_DIER_CC1IE|TIM_DIER_CC2IE))==0);
    CHECK(HAL_TIM_OnePulse_DeInit(&h)==HAL_OK);
}
}
