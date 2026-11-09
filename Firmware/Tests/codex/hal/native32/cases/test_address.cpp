#include "native.h"
TEST_SUITE("P12NativeAddressControls") {
TEST_CASE_FIXTURE(native::Fixture, "DMA stream addresses retain their complete target width") {
    CHECK(sizeof(void*) == 4);
    CHECK(reinterpret_cast<uint32_t>(DMA1_Stream0) == DMA1_Stream0_BASE);
    CHECK(reinterpret_cast<uint32_t>(DMA2_Stream7) == DMA2_Stream7_BASE);
}
TEST_CASE_FIXTURE(native::Fixture, "anonymous register storage is reset independently for each fixture") {
    CHECK(DMA1_Stream0->CR == 0);
    CHECK(GPIOC->MODER == 0);
    CHECK(RCC->CR == (RCC_CR_HSION | RCC_CR_HSIRDY));
}
}
