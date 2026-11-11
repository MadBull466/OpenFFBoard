#include "native.h"
#ifdef USE_FULL_ASSERT
namespace {
struct AssertFixture : native::Fixture {
    AssertFixture(){native::expectAssertion(true);}
    ~AssertFixture(){native::expectAssertion(false);}
};
}
TEST_SUITE("P12NativeParameterAssertions") {
TEST_CASE_FIXTURE(AssertFixture,"full-assert GPIO rejects an empty pin selection before changing configuration") {
    GPIO_InitTypeDef c{};c.Pin=0;c.Mode=GPIO_MODE_OUTPUT_PP;
    CHECK_THROWS_AS(HAL_GPIO_Init(GPIOC,&c),native::AssertionObserved);
    CHECK(GPIOC->MODER==0);
}
TEST_CASE_FIXTURE(AssertFixture,"full-assert DMA rejects a reserved channel selection") {
    DMA_HandleTypeDef d{};d.Instance=DMA1_Stream0;d.Init.Channel=8U<<25;
    CHECK_THROWS_AS(HAL_DMA_Init(&d),native::AssertionObserved);
    CHECK(d.Instance->CR==0);
}
}
#endif
