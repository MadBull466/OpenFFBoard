#include "doctest.h"
#include "stm32f4xx_hal.h"
#include <cstdint>

namespace {
// Storage observation, NOT an electrical/peripheral simulator. The ST device
// header supplies the real GPIO layout. BSRR writes are observed as requests;
// they do not update ODR/IDR here. No lock, EXTI, clock, atomicity or W1C model.
// Source: HAL GPIO API comments and ST device-header BSRR BS/BR definitions.
// Confidence: high for exact stored words; peripheral effects unverified.
struct GpioFixture {
    GPIO_TypeDef port{};
    GpioFixture() {
        port.IDR = 0x5aa5U;
        port.ODR = 0xa55aU;
        port.BSRR = 0xdeadbeefU;
        port.MODER = 0x12345678U;
    }
    void checkUnrelatedStorage() {
        CHECK(port.IDR == 0x5aa5U);
        CHECK(port.ODR == 0xa55aU);
        CHECK(port.MODER == 0x12345678U);
    }
};
}

TEST_SUITE("P12RealGpioIO") {
TEST_CASE_FIXTURE(GpioFixture, "each selected high input is read as set") {
    for (unsigned bit = 0; bit < 16; ++bit) {
        CAPTURE(bit);
        const auto pin = static_cast<std::uint16_t>(1U << bit);
        port.IDR = pin;
        CHECK(HAL_GPIO_ReadPin(&port, pin) == GPIO_PIN_SET);
    }
}

TEST_CASE_FIXTURE(GpioFixture, "unrelated high inputs do not set the selected low input") {
    for (unsigned bit = 0; bit < 16; ++bit) {
        CAPTURE(bit);
        const auto pin = static_cast<std::uint16_t>(1U << bit);
        port.IDR = static_cast<std::uint16_t>(~pin);
        CHECK(HAL_GPIO_ReadPin(&port, pin) == GPIO_PIN_RESET);
    }
}

TEST_CASE_FIXTURE(GpioFixture, "reading an input leaves register storage unchanged") {
    CHECK(HAL_GPIO_ReadPin(&port, GPIO_PIN_0) == GPIO_PIN_SET);
    checkUnrelatedStorage();
    CHECK(port.BSRR == 0xdeadbeefU);
}

TEST_CASE_FIXTURE(GpioFixture, "setting each output requests only its named BSRR set bit") {
    // ST stm32f407xx.h GPIO_BSRR_BS0..BS15 define contiguous bits 0..15.
    for (unsigned bit = 0; bit < 16; ++bit) {
        CAPTURE(bit);
        const auto pin = static_cast<std::uint16_t>(1U << bit);
        HAL_GPIO_WritePin(&port, pin, GPIO_PIN_SET);
        CHECK(port.BSRR == static_cast<std::uint32_t>(pin));
        checkUnrelatedStorage();
    }
}

TEST_CASE_FIXTURE(GpioFixture, "resetting each output requests only its named BSRR reset bit") {
    // Device-header BR0..BR15 are bits 16..31; uint32_t keeps bit 31 defined.
    for (unsigned bit = 0; bit < 16; ++bit) {
        CAPTURE(bit);
        const auto pin = static_cast<std::uint16_t>(1U << bit);
        HAL_GPIO_WritePin(&port, pin, GPIO_PIN_RESET);
        CHECK(port.BSRR == (std::uint32_t{1} << (16U + bit)));
        checkUnrelatedStorage();
    }
}

TEST_CASE_FIXTURE(GpioFixture, "toggling a selected low output requests its set bit") {
    for (unsigned bit = 0; bit < 16; ++bit) {
        CAPTURE(bit);
        const auto pin = static_cast<std::uint16_t>(1U << bit);
        port.ODR = static_cast<std::uint16_t>(~pin);
        HAL_GPIO_TogglePin(&port, pin);
        CHECK(port.BSRR == static_cast<std::uint32_t>(pin));
        CHECK(port.ODR == static_cast<std::uint16_t>(~pin));
    }
}

TEST_CASE_FIXTURE(GpioFixture, "toggling a selected high output requests its reset bit") {
    for (unsigned bit = 0; bit < 16; ++bit) {
        CAPTURE(bit);
        const auto pin = static_cast<std::uint16_t>(1U << bit);
        port.ODR = pin;
        HAL_GPIO_TogglePin(&port, pin);
        CHECK(port.BSRR == (std::uint32_t{1} << (16U + bit)));
        CHECK(port.ODR == pin);
    }
}

TEST_CASE_FIXTURE(GpioFixture, "a mixed toggle requests distinct set and reset bits only for selected outputs") {
    port.ODR = GPIO_PIN_15 | GPIO_PIN_7 | GPIO_PIN_3;
    HAL_GPIO_TogglePin(&port, GPIO_PIN_15 | GPIO_PIN_8 | GPIO_PIN_0);
    CHECK(port.BSRR == (GPIO_BSRR_BR15 | GPIO_BSRR_BS8 | GPIO_BSRR_BS0));
    CHECK(port.ODR == (GPIO_PIN_15 | GPIO_PIN_7 | GPIO_PIN_3));
}
}
