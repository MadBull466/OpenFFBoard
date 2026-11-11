#include "doctest.h"
#include "stm32f4xx_hal.h"
#include <stdexcept>

namespace {
struct ModelFixture {
    ModelFixture() { p12::reset(); }
    ~ModelFixture() { p12::reset(); }
};
void lockSequence(GPIO_TypeDef *g, std::uint16_t pins) {
    // RM0090 8.4.8: 1,0,1 key writes with constant pins, then read.
    g->LCKR = GPIO_LCKR_LCKK | pins;
    g->LCKR = pins;
    g->LCKR = GPIO_LCKR_LCKK | pins;
    (void)static_cast<std::uint32_t>(g->LCKR);
}
}

TEST_SUITE("P12GpioModelControls") {
TEST_CASE_FIXTURE(ModelFixture, "model BSRR set and reset preserve unrelated output bits") {
    p12::seed(GPIOC->ODR, 0xaaaa);
    GPIOC->BSRR = GPIO_BSRR_BR1 | GPIO_BSRR_BS0;
    CHECK(p12::peek(GPIOC->ODR) == 0xaaa9);
}
TEST_CASE_FIXTURE(ModelFixture, "model BSRR gives set priority over reset for the same pin") {
    GPIOC->BSRR = GPIO_BSRR_BR15 | GPIO_BSRR_BS15;
    CHECK(p12::peek(GPIOC->ODR) == GPIO_PIN_15);
}
TEST_CASE_FIXTURE(ModelFixture, "model write-only BSRR reads zero after a write") {
    GPIOC->BSRR = GPIO_BSRR_BS7;
    CHECK(static_cast<std::uint32_t>(GPIOC->BSRR) == 0);
}
TEST_CASE_FIXTURE(ModelFixture, "model pending clear removes only the written one bits") {
    p12::injectPending(GPIO_PIN_2 | GPIO_PIN_9);
    EXTI->PR = GPIO_PIN_2;
    CHECK(p12::peek(EXTI->PR) == GPIO_PIN_9);
}
TEST_CASE_FIXTURE(ModelFixture, "model pending zero write preserves pending requests") {
    p12::injectPending(GPIO_PIN_2 | GPIO_PIN_9);
    EXTI->PR = 0;
    CHECK(p12::peek(EXTI->PR) == (GPIO_PIN_2 | GPIO_PIN_9));
}
TEST_CASE_FIXTURE(ModelFixture, "model valid lock freezes selected configuration fields only") {
    lockSequence(GPIOC, GPIO_PIN_3);
    GPIOC->MODER = 0xffffffffU;
    GPIOC->OTYPER = 0xffff;
    GPIOC->OSPEEDR = 0xffffffffU;
    GPIOC->PUPDR = 0x55555555U;
    GPIOC->AFR[0] = 0xffffffffU;
    CHECK(p12::peek(GPIOC->MODER) == 0xffffff3fU);
    CHECK(p12::peek(GPIOC->OTYPER) == 0xfff7);
    CHECK(p12::peek(GPIOC->OSPEEDR) == 0xffffff3fU);
    CHECK(p12::peek(GPIOC->PUPDR) == 0x55555515U);
    CHECK(p12::peek(GPIOC->AFR[0]) == 0xffff0fffU);
    GPIOC->BSRR = GPIO_BSRR_BS3;
    CHECK(p12::peek(GPIOC->ODR) == GPIO_PIN_3); // lock does not freeze output data
}
TEST_CASE_FIXTURE(ModelFixture, "model rejects every three-write key pattern other than one zero one") {
    for (unsigned pattern = 0; pattern < 8; ++pattern) {
        if (pattern == 5) continue;
        CAPTURE(pattern);
        p12::reset();
        for (int bit = 2; bit >= 0; --bit)
            GPIOC->LCKR = GPIO_PIN_3 | (((pattern >> bit) & 1U) ? GPIO_LCKR_LCKK : 0U);
        CHECK((static_cast<std::uint32_t>(GPIOC->LCKR) & GPIO_LCKR_LCKK) == 0);
        GPIOC->MODER = 0x40;
        CHECK(p12::peek(GPIOC->MODER) == 0x40);
    }
}
TEST_CASE_FIXTURE(ModelFixture, "model lock aborts when the pin mask changes during the key sequence") {
    GPIOC->LCKR = GPIO_LCKR_LCKK | GPIO_PIN_3;
    GPIOC->LCKR = GPIO_PIN_4;
    GPIOC->LCKR = GPIO_LCKR_LCKK | GPIO_PIN_3;
    CHECK((static_cast<std::uint32_t>(GPIOC->LCKR) & GPIO_LCKR_LCKK) == 0);
}
TEST_CASE_FIXTURE(ModelFixture, "model reset clears an established lock") {
    lockSequence(GPIOC, GPIO_PIN_3);
    p12::reset();
    GPIOC->MODER = 0x40;
    CHECK(p12::peek(GPIOC->MODER) == 0x40);
    CHECK(static_cast<std::uint32_t>(GPIOC->LCKR) == 0);
}
TEST_CASE_FIXTURE(ModelFixture, "model locks high alternate fields without freezing low alternate fields") {
    lockSequence(GPIOC, GPIO_PIN_15);
    GPIOC->AFR[1] = 0xffffffffU;
    GPIOC->AFR[0] = 0xffffffffU;
    CHECK(p12::peek(GPIOC->AFR[1]) == 0x0fffffffU);
    CHECK(p12::peek(GPIOC->AFR[0]) == 0xffffffffU);
}
TEST_CASE_FIXTURE(ModelFixture, "model lock preserves nonzero fields and cannot be retargeted before reset") {
    p12::seed(GPIOC->MODER, 0x80);
    p12::seed(GPIOC->OTYPER, GPIO_PIN_3);
    p12::seed(GPIOC->PUPDR, 0x80);
    p12::seed(GPIOC->AFR[0], 0x9000);
    lockSequence(GPIOC, GPIO_PIN_3);
    GPIOC->LCKR = GPIO_LCKR_LCKK | GPIO_PIN_4;
    CHECK(static_cast<std::uint32_t>(GPIOC->LCKR) == (GPIO_LCKR_LCKK | GPIO_PIN_3));
    GPIOC->MODER = 0;
    GPIOC->OTYPER = 0;
    GPIOC->PUPDR = 0;
    GPIOC->AFR[0] = 0;
    CHECK(p12::peek(GPIOC->MODER) == 0x80);
    CHECK(p12::peek(GPIOC->OTYPER) == GPIO_PIN_3);
    CHECK(p12::peek(GPIOC->PUPDR) == 0x80);
    CHECK(p12::peek(GPIOC->AFR[0]) == 0x9000);
}
TEST_CASE_FIXTURE(ModelFixture, "model reset initializes the documented port A and B debug fields") {
    CHECK(p12::peek(GPIOA->MODER) == 0xa8000000U);
    CHECK(p12::peek(GPIOA->OSPEEDR) == 0x0c000000U);
    CHECK(p12::peek(GPIOA->PUPDR) == 0x64000000U);
    CHECK(p12::peek(GPIOB->MODER) == 0x00000280U);
    CHECK(p12::peek(GPIOB->OSPEEDR) == 0x000000c0U);
    CHECK(p12::peek(GPIOB->PUPDR) == 0x00000100U);
}
TEST_CASE_FIXTURE(ModelFixture, "model rejects the reserved pull configuration encoding") {
    CHECK_THROWS_AS(GPIOC->PUPDR = 3, std::logic_error);
}
TEST_CASE_FIXTURE(ModelFixture, "model rejects an unregistered register access") {
    p12::Register unknown;
    CHECK_THROWS_AS(unknown = 1, std::logic_error);
}
TEST_CASE_FIXTURE(ModelFixture, "model rejects writes to input data") {
    CHECK_THROWS_AS(GPIOC->IDR = 1, std::logic_error);
}
TEST_CASE_FIXTURE(ModelFixture, "model rejects routing access before the SYSCFG clock request") {
    CHECK_THROWS_AS((void)static_cast<std::uint32_t>(SYSCFG->EXTICR[0]), std::logic_error);
}
TEST_CASE_FIXTURE(ModelFixture, "model rejects unimplemented software interrupt generation") {
    CHECK_THROWS_AS(EXTI->SWIER = GPIO_PIN_0, std::logic_error);
}
TEST_CASE_FIXTURE(ModelFixture, "model rejects erratum-dependent PH10 routing instead of promising silicon fidelity") {
    p12::enableSyscfgClock();
    CHECK_THROWS_AS(SYSCFG->EXTICR[2] = 0x700, std::logic_error);
}
}

TEST_SUITE("P12RealGpioRegisters") {
TEST_CASE_FIXTURE(ModelFixture, "real HAL set requests drive modeled output bits without touching neighbors") {
    p12::seed(GPIOC->ODR, 0xaaaa);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_15, GPIO_PIN_SET);
    CHECK(p12::peek(GPIOC->ODR) == 0xaaab);
}
TEST_CASE_FIXTURE(ModelFixture, "real HAL reset requests clear modeled output bits without touching neighbors") {
    p12::seed(GPIOC->ODR, 0xaaaa);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 | GPIO_PIN_15, GPIO_PIN_RESET);
    CHECK(p12::peek(GPIOC->ODR) == 0x2aa8);
}
TEST_CASE_FIXTURE(ModelFixture, "real HAL mixed toggle changes only the selected modeled outputs") {
    p12::seed(GPIOC->ODR, 0xaaaa);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_15);
    CHECK(p12::peek(GPIOC->ODR) == 0x2aab);
}
TEST_CASE_FIXTURE(ModelFixture, "real HAL output initialization changes only the selected mode type speed and pull fields") {
    p12::seed(GPIOC->MODER, 0xaaaaaaaaU);
    p12::seed(GPIOC->OSPEEDR, 0x55555555U);
    GPIO_InitTypeDef init{GPIO_PIN_0 | GPIO_PIN_15, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH, 0};
    HAL_GPIO_Init(GPIOC, &init);
    // RM0090: two-bit fields at 2*n; output=01, very-high=11, pullup=01.
    CHECK(p12::peek(GPIOC->MODER) == 0x6aaaaaa9U);
    CHECK(p12::peek(GPIOC->OTYPER) == 0x8001);
    CHECK(p12::peek(GPIOC->OSPEEDR) == 0xd5555557U);
    CHECK(p12::peek(GPIOC->PUPDR) == 0x40000001U);
    CHECK_FALSE(p12::syscfgClockEnabled());
}
TEST_CASE_FIXTURE(ModelFixture, "real HAL input initialization selects input and pull down without changing other mode fields") {
    p12::seed(GPIOC->MODER, 0xffffffffU);
    GPIO_InitTypeDef init{GPIO_PIN_1 | GPIO_PIN_14, GPIO_MODE_INPUT, GPIO_PULLDOWN, 0, 0};
    HAL_GPIO_Init(GPIOC, &init);
    CHECK(p12::peek(GPIOC->MODER) == 0xcffffff3U);
    CHECK(p12::peek(GPIOC->PUPDR) == 0x20000008U);
}
TEST_CASE_FIXTURE(ModelFixture, "real HAL alternate initialization maps pins across both alternate registers") {
    p12::seed(GPIOC->AFR[0], 0x12345678);
    p12::seed(GPIOC->AFR[1], 0x89abcdef);
    GPIO_InitTypeDef init{GPIO_PIN_7 | GPIO_PIN_8, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM, 7};
    HAL_GPIO_Init(GPIOC, &init);
    CHECK(p12::peek(GPIOC->AFR[0]) == 0x72345678);
    CHECK(p12::peek(GPIOC->AFR[1]) == 0x89abcde7);
    CHECK(p12::peek(GPIOC->MODER) == 0x00028000);
    CHECK(p12::peek(GPIOC->OTYPER) == 0x180);
}
TEST_CASE_FIXTURE(ModelFixture, "real HAL rising interrupt setup enables its clock before routing and preserves other lines") {
    p12::seed(SYSCFG->EXTICR[1], 0x3210);
    p12::seed(EXTI->FTSR, GPIO_PIN_5 | GPIO_PIN_3);
    p12::seed(EXTI->EMR, GPIO_PIN_5 | GPIO_PIN_3);
    GPIO_InitTypeDef init{GPIO_PIN_5, GPIO_MODE_IT_RISING, GPIO_NOPULL, 0, 0};
    HAL_GPIO_Init(GPIOC, &init);
    CHECK(p12::syscfgClockEnabled());
    CHECK(p12::peek(SYSCFG->EXTICR[1]) == 0x3220);
    CHECK(p12::peek(EXTI->RTSR) == GPIO_PIN_5);
    CHECK(p12::peek(EXTI->FTSR) == GPIO_PIN_3);
    CHECK(p12::peek(EXTI->IMR) == GPIO_PIN_5);
    CHECK(p12::peek(EXTI->EMR) == GPIO_PIN_3);
}
TEST_CASE_FIXTURE(ModelFixture, "real HAL falling event setup clears the selected interrupt and rising trigger") {
    p12::seed(EXTI->RTSR, GPIO_PIN_8 | GPIO_PIN_2);
    p12::seed(EXTI->IMR, GPIO_PIN_8 | GPIO_PIN_2);
    GPIO_InitTypeDef init{GPIO_PIN_8, GPIO_MODE_EVT_FALLING, GPIO_NOPULL, 0, 0};
    HAL_GPIO_Init(GPIOD, &init);
    CHECK(p12::peek(SYSCFG->EXTICR[2]) == 3);
    CHECK(p12::peek(EXTI->FTSR) == GPIO_PIN_8);
    CHECK(p12::peek(EXTI->RTSR) == GPIO_PIN_2);
    CHECK(p12::peek(EXTI->IMR) == GPIO_PIN_2);
    CHECK(p12::peek(EXTI->EMR) == GPIO_PIN_8);
}
TEST_CASE_FIXTURE(ModelFixture, "real HAL both-edge interrupt setup enables both selected triggers") {
    GPIO_InitTypeDef init{GPIO_PIN_12, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLDOWN, 0, 0};
    HAL_GPIO_Init(GPIOC, &init);
    CHECK(p12::peek(EXTI->RTSR) == GPIO_PIN_12);
    CHECK(p12::peek(EXTI->FTSR) == GPIO_PIN_12);
    CHECK(p12::peek(EXTI->IMR) == GPIO_PIN_12);
    CHECK(p12::peek(EXTI->EMR) == 0);
    CHECK(p12::peek(SYSCFG->EXTICR[3]) == 2);
}
TEST_CASE_FIXTURE(ModelFixture, "real HAL analog initialization selects the requested analog mode only") {
    GPIO_InitTypeDef init{GPIO_PIN_6, GPIO_MODE_ANALOG, GPIO_NOPULL, 0, 0};
    HAL_GPIO_Init(GPIOC, &init);
    CHECK(p12::peek(GPIOC->MODER) == 0x3000);
    CHECK_FALSE(p12::syscfgClockEnabled());
}
TEST_CASE_FIXTURE(ModelFixture, "real HAL deinitialization releases its own route and selected configuration only") {
    p12::enableSyscfgClock();
    p12::seed(SYSCFG->EXTICR[1], 0x100);
    p12::seed(GPIOB->MODER, 0xffffffffU);
    p12::seed(GPIOB->AFR[0], 0xffffffffU);
    p12::seed(GPIOB->OTYPER, 0xffff);
    p12::seed(GPIOB->OSPEEDR, 0xffffffffU);
    p12::seed(GPIOB->PUPDR, 0xaaaaaaaaU);
    p12::seed(EXTI->IMR, GPIO_PIN_6 | GPIO_PIN_4);
    p12::seed(EXTI->EMR, GPIO_PIN_6 | GPIO_PIN_4);
    p12::seed(EXTI->RTSR, GPIO_PIN_6 | GPIO_PIN_4);
    p12::seed(EXTI->FTSR, GPIO_PIN_6 | GPIO_PIN_4);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
    CHECK(p12::peek(SYSCFG->EXTICR[1]) == 0);
    CHECK(p12::peek(GPIOB->MODER) == 0xffffcfffU);
    CHECK(p12::peek(GPIOB->AFR[0]) == 0xf0ffffffU);
    CHECK(p12::peek(GPIOB->OTYPER) == 0xffbf);
    CHECK(p12::peek(GPIOB->OSPEEDR) == 0xffffcfffU);
    CHECK(p12::peek(GPIOB->PUPDR) == 0xaaaa8aaaU);
    CHECK(p12::peek(EXTI->IMR) == GPIO_PIN_4);
    CHECK(p12::peek(EXTI->EMR) == GPIO_PIN_4);
    CHECK(p12::peek(EXTI->RTSR) == GPIO_PIN_4);
    CHECK(p12::peek(EXTI->FTSR) == GPIO_PIN_4);
}
TEST_CASE_FIXTURE(ModelFixture, "real HAL deinitialization preserves a line routed to another port") {
    p12::enableSyscfgClock();
    p12::seed(SYSCFG->EXTICR[1], 0x100);
    p12::seed(EXTI->IMR, GPIO_PIN_6);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6);
    CHECK(p12::peek(SYSCFG->EXTICR[1]) == 0x100);
    CHECK(p12::peek(EXTI->IMR) == GPIO_PIN_6);
}
TEST_CASE_FIXTURE(ModelFixture, "real HAL lock completes the key protocol and protects subsequent pin configuration") {
    REQUIRE(HAL_GPIO_LockPin(GPIOC, GPIO_PIN_3) == HAL_OK);
    const auto &ops = p12::trace();
    REQUIRE(ops.size() == 5);
    CHECK(ops[0].isWrite); CHECK(ops[0].value == (GPIO_LCKR_LCKK | GPIO_PIN_3));
    CHECK(ops[1].isWrite); CHECK(ops[1].value == GPIO_PIN_3);
    CHECK(ops[2].isWrite); CHECK(ops[2].value == (GPIO_LCKR_LCKK | GPIO_PIN_3));
    CHECK_FALSE(ops[3].isWrite); CHECK_FALSE(ops[4].isWrite);
    GPIO_InitTypeDef init{GPIO_PIN_3 | GPIO_PIN_4, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0};
    HAL_GPIO_Init(GPIOC, &init);
    CHECK(p12::peek(GPIOC->MODER) == 0x100); // pin 3 locked as input; pin 4 becomes output
}
TEST_CASE_FIXTURE(ModelFixture, "real HAL interrupt handler clears the selected pending bit before its callback") {
    p12::injectPending(GPIO_PIN_2 | GPIO_PIN_9);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
    REQUIRE(p12::callbacks().size() == 1);
    CHECK(p12::callbacks()[0].pin == GPIO_PIN_2);
    CHECK(p12::callbacks()[0].pendingAtCallback == GPIO_PIN_9);
    CHECK(p12::peek(EXTI->PR) == GPIO_PIN_9);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
    CHECK(p12::callbacks().size() == 1);
}
TEST_CASE_FIXTURE(ModelFixture, "real HAL interrupt handler does not deliver an unrelated pending line") {
    p12::injectPending(GPIO_PIN_9);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
    CHECK(p12::callbacks().empty());
    CHECK(p12::peek(EXTI->PR) == GPIO_PIN_9);
}
TEST_CASE_FIXTURE(ModelFixture, "full assertions reject an empty GPIO pin mask before register accesses") {
    CHECK_THROWS_AS(HAL_GPIO_WritePin(GPIOC, 0, GPIO_PIN_SET), std::invalid_argument);
    CHECK(p12::trace().empty());
}
TEST_CASE_FIXTURE(ModelFixture, "full assertions reject an invalid configuration mode before register accesses") {
    GPIO_InitTypeDef init{GPIO_PIN_0, 0xffffffffU, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0};
    CHECK_THROWS_AS(HAL_GPIO_Init(GPIOC, &init), std::invalid_argument);
    CHECK(p12::trace().empty());
}
TEST_CASE_FIXTURE(ModelFixture, "full assertions reject an unknown GPIO instance before register accesses") {
    GPIO_TypeDef unknown;
    GPIO_InitTypeDef init{GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0};
    CHECK_THROWS_AS(HAL_GPIO_Init(&unknown, &init), std::invalid_argument);
    CHECK(p12::trace().empty());
}
}
