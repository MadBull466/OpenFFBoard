#include "native.h"
extern "C" {
void p12_probe_write32(volatile uint32_t*,uint32_t);
void p12_probe_write_twice(volatile uint32_t*,uint32_t,uint32_t);
uint32_t p12_probe_read32(volatile uint32_t*);
}
namespace { unsigned gpioCalls; uint16_t gpioPin; }
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin) { ++gpioCalls; gpioPin=pin; }
TEST_SUITE("P12NativeGPIO") {
TEST_CASE_FIXTURE(native::Fixture,"real C GPIO set reset and toggle obey modeled output-bit effects") {
    GPIOC->ODR=0xaaaa; HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET); native::flush(); CHECK(GPIOC->ODR==0xaaab);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET); native::flush(); CHECK(GPIOC->ODR==0x2aab);
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1); native::flush(); CHECK(GPIOC->ODR==0x2aa9);
}
TEST_CASE_FIXTURE(native::Fixture,"real C GPIO reads only the selected input bit") {
    GPIOC->IDR=GPIO_PIN_5; CHECK(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5)==GPIO_PIN_SET);
    CHECK(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4)==GPIO_PIN_RESET);
}
TEST_CASE_FIXTURE(native::Fixture,"real C GPIO alternate configuration spans both mapping registers") {
    GPIO_InitTypeDef c{}; c.Pin=GPIO_PIN_7|GPIO_PIN_8; c.Mode=GPIO_MODE_AF_OD;
    c.Pull=GPIO_PULLDOWN; c.Speed=GPIO_SPEED_FREQ_HIGH; c.Alternate=7;
    HAL_GPIO_Init(GPIOC,&c); native::flush();
    CHECK(GPIOC->AFR[0]==0x70000000); CHECK(GPIOC->AFR[1]==7); CHECK(GPIOC->MODER==0x28000);
    HAL_GPIO_DeInit(GPIOC,GPIO_PIN_7); native::flush(); CHECK(GPIOC->AFR[0]==0); CHECK(GPIOC->AFR[1]==7);
}
TEST_CASE_FIXTURE(native::Fixture,"real C GPIO analog configuration selects only its requested mode field") {
    GPIO_InitTypeDef c{}; c.Pin=GPIO_PIN_6; c.Mode=GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOC,&c); native::flush(); CHECK(GPIOC->MODER==0x3000);
}
TEST_CASE_FIXTURE(native::Fixture,"real C GPIO lock accepts the key sequence and freezes selected fields only") {
    REQUIRE(HAL_GPIO_LockPin(GPIOC,GPIO_PIN_3)==HAL_OK);
    GPIO_InitTypeDef c{}; c.Pin=GPIO_PIN_3|GPIO_PIN_4; c.Mode=GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOC,&c); native::flush(); CHECK(GPIOC->MODER==0x100);
}
TEST_CASE_FIXTURE(native::Fixture,"native lock model rejects a changed key mask before activation") {
    p12_probe_write_twice(&GPIOC->LCKR,GPIO_LCKR_LCKK|GPIO_PIN_3,GPIO_PIN_4);
    p12_probe_write32(&GPIOC->LCKR,GPIO_LCKR_LCKK|GPIO_PIN_3);
    CHECK((p12_probe_read32(&GPIOC->LCKR)&GPIO_LCKR_LCKK)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"real C GPIO pending handler clears its line before delivering the exact callback argument") {
    gpioCalls=0; gpioPin=0; EXTI->PR=GPIO_PIN_5|GPIO_PIN_9;
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5); native::flush();
    CHECK(gpioCalls==1); CHECK(gpioPin==GPIO_PIN_5); CHECK(EXTI->PR==GPIO_PIN_9);
}
}
TEST_SUITE("P12NativeEXTIMore") {
TEST_CASE_FIXTURE(native::Fixture,"EXTI event-only mode preserves the absence of an interrupt mask") {
    EXTI_HandleTypeDef h{}; EXTI_ConfigTypeDef c{};
    c.Line=EXTI_LINE_7;c.Mode=EXTI_MODE_EVENT;c.Trigger=EXTI_TRIGGER_FALLING;c.GPIOSel=EXTI_GPIOD;
    REQUIRE(HAL_EXTI_SetConfigLine(&h,&c)==HAL_OK);
    CHECK((EXTI->EMR&(1U<<7))!=0); CHECK((EXTI->IMR&(1U<<7))==0);
    CHECK((EXTI->RTSR&(1U<<7))==0); CHECK((EXTI->FTSR&(1U<<7))!=0);
}
TEST_CASE_FIXTURE(native::Fixture,"EXTI software trigger sets pending only for the enabled interrupt line") {
    EXTI_HandleTypeDef h{}; REQUIRE(HAL_EXTI_GetHandle(&h,EXTI_LINE_7)==HAL_OK);
    EXTI->IMR=1U<<7; HAL_EXTI_GenerateSWI(&h); native::flush();
    CHECK(HAL_EXTI_GetPending(&h,EXTI_TRIGGER_RISING_FALLING)==1);
    HAL_EXTI_ClearPending(&h,EXTI_TRIGGER_RISING_FALLING); native::flush();
    CHECK((EXTI->SWIER&(1U<<7))==0); CHECK((EXTI->PR&(1U<<7))==0);
}
TEST_CASE_FIXTURE(native::Fixture,"EXTI masked software trigger does not fabricate a pending interrupt") {
    EXTI_HandleTypeDef h{}; REQUIRE(HAL_EXTI_GetHandle(&h,EXTI_LINE_7)==HAL_OK);
    HAL_EXTI_GenerateSWI(&h); native::flush(); CHECK((EXTI->PR&(1U<<7))==0);
}
}
