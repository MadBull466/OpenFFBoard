/*
 * P12 T0 smoke cases: prove that the unmodified ST HAL (compiled as 32-bit host code) runs against the host register
 * space and the host tick. Built into ffb_hal_tests (F407VG) and ffb_hal_tests_disco (F407VG_DISCO).
 *
 * Oracles: RM0090 Rev 22 (Table 1 addresses; 8.4.5 GPIOx_IDR; 8.4.6 GPIOx_ODR; 8.4.7 GPIOx_BSRR) and UM1725 Rev 8
 * (HAL_GPIO_ReadPin "Reads the specified input port pin", HAL_GPIO_WritePin "Sets or clears the selected data port
 * bit", HAL_Delay "provides minimum delay (in milliseconds)").
 *
 * Updated in T1: the bus traps every access now (HalRegisterSpace.h), so BSRR reads 0 like the hardware and IDR is
 * read-only; the cases check the written BSRR word from the access log and the resulting ODR, and set the input level
 * with hostGpioSetInput instead of writing IDR.
 */
#include "HalFixture.h"

#include <cstdint>

TEST_SUITE("P12 T0 smoke") {

TEST_CASE_FIXTURE(HalFixture, "the GPIOA, RCC and system control blocks are mapped at their STM32F407 addresses") {
	// RM0090 Table 1: GPIOA 0x4002 0000, RCC 0x4002 3800. core_cm4.h: SCB at SCS_BASE + 0xD00 = 0xE000 ED00.
	CHECK(reinterpret_cast<std::uintptr_t>(GPIOA) == 0x40020000u);
	CHECK(reinterpret_cast<std::uintptr_t>(RCC) == 0x40023800u);
	CHECK(reinterpret_cast<std::uintptr_t>(SCB) == 0xE000ED00u);
	CHECK(hostRegsContains(0x40020000u, 4) == 1);
	CHECK(hostRegsContains(0x40023800u, 4) == 1);
	CHECK(hostRegsContains(0xE000ED00u, 4) == 1);
	GPIOA->ODR = 0x1234u;
	CHECK(GPIOA->ODR == 0x1234u);
}

TEST_CASE_FIXTURE(HalFixture, "HAL_GPIO_WritePin with GPIO_PIN_SET requests the set through the BS bit of the pin in BSRR") {
	// RM0090 8.4.7: BSy (bits 15:0) = 1 sets ODRy; BRy (bits 31:16) = 1 resets it. Pin 5 set: only bit 5 is written.
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	CHECK(lastWritten(&GPIOA->BSRR) == (1u << 5));
	CHECK(peek(&GPIOA->ODR) == (1u << 5));
}

TEST_CASE_FIXTURE(HalFixture, "HAL_GPIO_WritePin with GPIO_PIN_RESET requests the reset through the BR bit of the pin in BSRR") {
	// RM0090 8.4.7: BR5 is bit 16 + 5.
	poke(&GPIOA->ODR, 0xFFFFu);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	CHECK(lastWritten(&GPIOA->BSRR) == (1u << (16 + 5)));
	CHECK(peek(&GPIOA->ODR) == (0xFFFFu & ~(1u << 5)));
}

TEST_CASE_FIXTURE(HalFixture, "HAL_GPIO_ReadPin returns GPIO_PIN_SET when the IDR bit of the pin is 1") {
	hostGpioSetInput(1, 7, HOST_PIN_HIGH); // PB7 input (reset mode) driven high: IDR7 = 1 (RM0090 8.4.5)
	CHECK(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET);
}

TEST_CASE_FIXTURE(HalFixture, "HAL_GPIO_ReadPin returns GPIO_PIN_RESET when only the IDR bits of other pins are 1") {
	for (unsigned pin = 0; pin < 16; ++pin) hostGpioSetInput(1, pin, pin == 7 ? HOST_PIN_LOW : HOST_PIN_HIGH);
	CHECK(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET);
}

TEST_CASE_FIXTURE(HalFixture, "HAL_Delay returns only after at least the requested number of milliseconds of simulated time") {
	// One simulated tick interrupt (1 ms, uwTickFreq default) per HAL_GetTick call, see HalHostSystem.h.
	const uint32_t before = hostTickPeek();
	HAL_Delay(5u);
	CHECK(hostTickPeek() - before >= 5u);
	CHECK(hostTickGetCalls() > 0u);
}

}
