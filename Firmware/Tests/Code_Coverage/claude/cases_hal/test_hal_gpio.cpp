/*
 * P12 T1 group G: HAL GPIO driver (stm32f4xx_hal_gpio.c) against the register models (HalPeripherals.h).
 * Requirements G1-G14: Tests/Code_Coverage/claude/charters/hal/P12_T1_REQUIREMENTS.md.
 *
 * Oracles: RM0090 Rev 22 (8.3.5 bitwise handling, 8.3.6 locking, 8.3.9 / 8.3.10 input / output, 8.3.12 analog,
 * 8.4.1-8.4.10 registers, 9.2.3-9.2.6 EXTICR, 12.2.4 / 12.3.1-12.3.6 EXTI, 7.3.14 APB2ENR) and UM1725 Rev 8 (31.2).
 * Encodings used in the derivations (RM0090):
 *  - MODERy (2 bits): 00 input, 01 output, 10 alternate function, 11 analog (8.4.1).
 *  - OTYPERy: 0 push-pull, 1 open-drain (8.4.2).
 *  - OSPEEDRy: 00 low, 01 medium, 10 fast, 11 high (8.4.3); the HAL's speed argument 0..3 is the register value.
 *  - PUPDRy: 00 none, 01 pull-up, 10 pull-down (8.4.4).
 *  - AFRL holds pins 0-7, AFRH pins 8-15, 4 bits per pin (8.4.9, 8.4.10).
 * Port index: A = 0 ... I = 8 (EXTICR value 0000 = A ... 1000 = I, RM0090 9.2.3).
 * Inputs from the HAL header (GPIO_MODE_*, GPIO_PULLUP, ...) are inputs only; expected values are written out here.
 */
#include "HalFixture.h"
#include "TestHelpers.h"

#include <cstdint>

namespace {

int g_cbCount = 0;
uint16_t g_cbPin = 0;

void resetCallbackRecorder() {
	g_cbCount = 0;
	g_cbPin = 0;
}

GPIO_TypeDef *portOf(unsigned i) {
	GPIO_TypeDef *const ports[HOST_GPIO_PORTS] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI};
	return ports[i];
}

void enableAllPortClocks() {
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
}

void initPin(GPIO_TypeDef *port, uint32_t pins, uint32_t mode, uint32_t pull, uint32_t speed, uint32_t alt = 0) {
	GPIO_InitTypeDef init = {};
	init.Pin = pins;
	init.Mode = mode;
	init.Pull = pull;
	init.Speed = speed;
	init.Alternate = alt;
	HAL_GPIO_Init(port, &init);
}

/* Fills the five configuration registers of a port with all ones so that "others unchanged" is visible. */
struct Ones : HalFixture {
	void fill(GPIO_TypeDef *p) {
		poke(&p->MODER, 0xFFFFFFFFu);
		poke(&p->OTYPER, 0xFFFFu);
		poke(&p->OSPEEDR, 0xFFFFFFFFu);
		poke(&p->PUPDR, 0xFFFFFFFFu);
		poke(&p->AFR[0], 0xFFFFFFFFu);
		poke(&p->AFR[1], 0xFFFFFFFFu);
	}
};

const uint32_t kAllLines = 0x7FFFFFu; /* EXTI IMR/EMR/RTSR/FTSR keep bits 22:0 (RM0090 12.3.1-12.3.4) */

} // namespace

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	++g_cbCount;
	g_cbPin = GPIO_Pin;
}

TEST_SUITE("P12 T1 GPIO") {

// ---------------------------------------------------------------- G1 output
TEST_CASE_FIXTURE(Ones, "G1 push-pull output on a low pin sets MODER 01 and OTYPER 0 and keeps the other bits") {
	enableAllPortClocks();
	fill(GPIOF);
	initPin(GPIOF, GPIO_PIN_3, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	// pin 3: MODER bits 7:6 = 01 -> 0xFFFFFFFF & ~(3<<6) | (1<<6) = 0xFFFFFF7F
	CHECK(peek(&GPIOF->MODER) == 0xFFFFFF7Fu);
	// OTYPER bit 3 = 0 -> 0xFFFF & ~8 = 0xFFF7
	CHECK(peek(&GPIOF->OTYPER) == 0xFFF7u);
	// OSPEEDR bits 7:6 = 00 (low), PUPDR bits 7:6 = 00 (none): 0xFFFFFFFF & ~(3<<6) = 0xFFFFFF3F
	CHECK(peek(&GPIOF->OSPEEDR) == 0xFFFFFF3Fu);
	CHECK(peek(&GPIOF->PUPDR) == 0xFFFFFF3Fu);
}

TEST_CASE_FIXTURE(Ones, "G1 push-pull output on a high pin sets MODER 01 and OTYPER 0 and keeps the other bits") {
	enableAllPortClocks();
	fill(GPIOF);
	initPin(GPIOF, GPIO_PIN_12, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	// pin 12: bits 25:24; MODER = 0xFFFFFFFF & ~(3<<24) | (1<<24) = 0xFDFFFFFF
	CHECK(peek(&GPIOF->MODER) == 0xFDFFFFFFu);
	CHECK(peek(&GPIOF->OTYPER) == 0xEFFFu);       // 0xFFFF & ~(1<<12)
	CHECK(peek(&GPIOF->OSPEEDR) == 0xFCFFFFFFu);  // bits 25:24 = 00
	CHECK(peek(&GPIOF->PUPDR) == 0xFCFFFFFFu);    // bits 25:24 = 00
}

TEST_CASE_FIXTURE(HalFixture, "G1 open-drain output on a low pin sets MODER 01 and OTYPER 1") {
	enableAllPortClocks();
	initPin(GPIOG, GPIO_PIN_3, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	CHECK(peek(&GPIOG->MODER) == (1u << 6));   // pin 3: bits 7:6 = 01
	CHECK(peek(&GPIOG->OTYPER) == (1u << 3));  // open-drain = 1
}

TEST_CASE_FIXTURE(HalFixture, "G1 open-drain output on a high pin sets MODER 01 and OTYPER 1") {
	enableAllPortClocks();
	initPin(GPIOG, GPIO_PIN_12, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	CHECK(peek(&GPIOG->MODER) == (1u << 24));  // pin 12: bits 25:24 = 01
	CHECK(peek(&GPIOG->OTYPER) == (1u << 12));
}

TEST_CASE_FIXTURE(HalFixture, "G1 each of the four output speeds is written to OSPEEDR of a low and a high pin") {
	enableAllPortClocks();
	for (uint32_t speed = 0; speed < 4; ++speed) {
		poke(&GPIOH->OSPEEDR, 0);
		initPin(GPIOH, GPIO_PIN_5, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, speed);
		// pin 5: bits 11:10
		CHECK(peek(&GPIOH->OSPEEDR) == (speed << 10));
		poke(&GPIOH->OSPEEDR, 0);
		initPin(GPIOH, GPIO_PIN_10, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, speed);
		// pin 10: bits 21:20
		CHECK(peek(&GPIOH->OSPEEDR) == (speed << 20));
	}
}

TEST_CASE_FIXTURE(HalFixture, "G1 each of the three pulls is written to PUPDR of an output on a low and a high pin") {
	enableAllPortClocks();
	// PUPDR values: none 00, pull-up 01, pull-down 10 (RM0090 8.4.4)
	const uint32_t pulls[3] = {GPIO_NOPULL, GPIO_PULLUP, GPIO_PULLDOWN};
	const uint32_t codes[3] = {0u, 1u, 2u};
	for (int i = 0; i < 3; ++i) {
		poke(&GPIOH->PUPDR, 0);
		initPin(GPIOH, GPIO_PIN_6, GPIO_MODE_OUTPUT_PP, pulls[i], GPIO_SPEED_FREQ_LOW);
		CHECK(peek(&GPIOH->PUPDR) == (codes[i] << 12)); // pin 6: bits 13:12
		poke(&GPIOH->PUPDR, 0);
		initPin(GPIOH, GPIO_PIN_14, GPIO_MODE_OUTPUT_PP, pulls[i], GPIO_SPEED_FREQ_LOW);
		CHECK(peek(&GPIOH->PUPDR) == (codes[i] << 28)); // pin 14: bits 29:28
	}
}

// ---------------------------------------------------------------- G2 alternate function
TEST_CASE_FIXTURE(Ones, "G2 push-pull alternate function on a low pin sets MODER 10 and the AFRL nibble only") {
	enableAllPortClocks();
	fill(GPIOD);
	initPin(GPIOD, GPIO_PIN_3, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIO_AF5_SPI1);
	// pin 3: MODER bits 7:6 = 10 -> 0xFFFFFFFF & ~(3<<6) | (2<<6) = 0xFFFFFFBF
	CHECK(peek(&GPIOD->MODER) == 0xFFFFFFBFu);
	CHECK(peek(&GPIOD->OTYPER) == 0xFFF7u);     // push-pull: bit 3 = 0
	// AFRL nibble 3 = bits 15:12 = 5 -> 0xFFFF0FFF | 0x5000 = 0xFFFF5FFF
	CHECK(peek(&GPIOD->AFR[0]) == 0xFFFF5FFFu);
	CHECK(peek(&GPIOD->AFR[1]) == 0xFFFFFFFFu); // AFRH untouched
}

TEST_CASE_FIXTURE(Ones, "G2 open-drain alternate function on a high pin sets MODER 10 and the AFRH nibble only") {
	enableAllPortClocks();
	fill(GPIOD);
	poke(&GPIOD->OTYPER, 0);
	initPin(GPIOD, GPIO_PIN_12, GPIO_MODE_AF_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIO_AF9_CAN1);
	// pin 12: MODER bits 25:24 = 10 -> 0xFFFFFFFF & ~(3<<24) | (2<<24) = 0xFEFFFFFF
	CHECK(peek(&GPIOD->MODER) == 0xFEFFFFFFu);
	CHECK(peek(&GPIOD->OTYPER) == (1u << 12));  // open-drain
	// AFRH nibble (12-8)=4 = bits 19:16 = 9 -> 0xFFF0FFFF | 0x90000 = 0xFFF9FFFF
	CHECK(peek(&GPIOD->AFR[1]) == 0xFFF9FFFFu);
	CHECK(peek(&GPIOD->AFR[0]) == 0xFFFFFFFFu); // AFRL untouched
}

TEST_CASE_FIXTURE(HalFixture, "G2 alternate function 15 on pin 8 fills AFRH nibble 0 and on pin 7 fills AFRL nibble 7") {
	enableAllPortClocks();
	initPin(GPIOE, GPIO_PIN_8, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 15);
	CHECK(peek(&GPIOE->AFR[1]) == 0xFu);        // pin 8 = first AFRH nibble
	CHECK(peek(&GPIOE->AFR[0]) == 0u);
	initPin(GPIOE, GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 15);
	CHECK(peek(&GPIOE->AFR[0]) == 0xF0000000u); // pin 7 = last AFRL nibble, bits 31:28
	CHECK(peek(&GPIOE->AFR[1]) == 0xFu);
}

TEST_CASE_FIXTURE(HalFixture, "G2 alternate function writes the speed and the pull of the pin") {
	enableAllPortClocks();
	initPin(GPIOE, GPIO_PIN_2, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH, 1);
	CHECK(peek(&GPIOE->OSPEEDR) == (3u << 4)); // pin 2: bits 5:4 = 11
	CHECK(peek(&GPIOE->PUPDR) == (1u << 4));   // pull-up 01
}

// ---------------------------------------------------------------- G3 input
TEST_CASE_FIXTURE(Ones, "G3 input with pull-up on a low pin sets MODER 00 and PUPDR 01 and keeps speed and type") {
	enableAllPortClocks();
	fill(GPIOC);
	initPin(GPIOC, GPIO_PIN_7, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);
	// pin 7: bits 15:14. MODER 00: 0xFFFFFFFF & ~(3<<14) = 0xFFFF3FFF
	CHECK(peek(&GPIOC->MODER) == 0xFFFF3FFFu);
	// PUPDR 01: 0xFFFF3FFF | (1<<14) = 0xFFFF7FFF
	CHECK(peek(&GPIOC->PUPDR) == 0xFFFF7FFFu);
	CHECK(peek(&GPIOC->OSPEEDR) == 0xFFFFFFFFu);
	CHECK(peek(&GPIOC->OTYPER) == 0xFFFFu);
}

TEST_CASE_FIXTURE(Ones, "G3 input with pull-down on a high pin sets MODER 00 and PUPDR 10 and keeps speed and type") {
	enableAllPortClocks();
	fill(GPIOC);
	initPin(GPIOC, GPIO_PIN_11, GPIO_MODE_INPUT, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW);
	// pin 11: bits 23:22. MODER 00: 0xFFFFFFFF & ~(3<<22) = 0xFF3FFFFF
	CHECK(peek(&GPIOC->MODER) == 0xFF3FFFFFu);
	// PUPDR 10: 0xFF3FFFFF | (2<<22) = 0xFFBFFFFF
	CHECK(peek(&GPIOC->PUPDR) == 0xFFBFFFFFu);
	CHECK(peek(&GPIOC->OSPEEDR) == 0xFFFFFFFFu);
	CHECK(peek(&GPIOC->OTYPER) == 0xFFFFu);
}

TEST_CASE_FIXTURE(Ones, "G3 input without pull clears PUPDR of the pin") {
	enableAllPortClocks();
	fill(GPIOC);
	initPin(GPIOC, GPIO_PIN_11, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	CHECK(peek(&GPIOC->PUPDR) == 0xFF3FFFFFu); // bits 23:22 = 00
}

// ---------------------------------------------------------------- G4 analog
TEST_CASE_FIXTURE(HalFixture, "G4 analog mode sets MODER 11 of the pin only") {
	enableAllPortClocks();
	initPin(GPIOC, GPIO_PIN_6, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	CHECK(peek(&GPIOC->MODER) == (3u << 12)); // pin 6: bits 13:12
}

TEST_CASE_FIXTURE(HalFixture, "G4 ReadPin of an analog pin returns RESET although the pin is driven high") {
	enableAllPortClocks();
	hostGpioSetInput(2, 6, HOST_PIN_HIGH);
	initPin(GPIOC, GPIO_PIN_6, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	// RM0090 8.3.12: in analog mode the Schmitt trigger output is forced to 0, so IDR reads 0.
	CHECK(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_RESET);
}

TEST_CASE_FIXTURE(HalFixture, "characterization: G4 analog mode with GPIO_PULLUP leaves PUPDR of the pin unchanged") {
	// RM0090 8.3.12 only says the pull resistors are disabled in analog mode; whether the HAL skips PUPDR is not specified.
	enableAllPortClocks();
	poke(&GPIOC->PUPDR, 0xFFFFFFFFu);
	initPin(GPIOC, GPIO_PIN_6, GPIO_MODE_ANALOG, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);
	CHECK(peek(&GPIOC->PUPDR) == 0xFFFFFFFFu);
}

// ---------------------------------------------------------------- G5 EXTI modes
namespace {
struct ModeCase {
	uint32_t mode;
	bool rising, falling, interrupt, event;
};
const ModeCase kModes[] = {
	{GPIO_MODE_IT_RISING, true, false, true, false},
	{GPIO_MODE_IT_FALLING, false, true, true, false},
	{GPIO_MODE_IT_RISING_FALLING, true, true, true, false},
	{GPIO_MODE_EVT_RISING, true, false, false, true},
	{GPIO_MODE_EVT_FALLING, false, true, false, true},
	{GPIO_MODE_EVT_RISING_FALLING, true, true, false, true},
};
// Line 6 starts with all four registers full of ones so the "others unchanged" and the clearing are visible.
void checkExtiMode(HalFixture &f, const ModeCase &m) {
	enableAllPortClocks();
	f.poke(&EXTI->RTSR, kAllLines);
	f.poke(&EXTI->FTSR, kAllLines);
	f.poke(&EXTI->IMR, kAllLines);
	f.poke(&EXTI->EMR, kAllLines);
	initPin(GPIOC, GPIO_PIN_6, m.mode, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	const uint32_t bit = 1u << 6;
	CHECK(f.peek(&EXTI->RTSR) == (m.rising ? kAllLines : (kAllLines & ~bit)));
	CHECK(f.peek(&EXTI->FTSR) == (m.falling ? kAllLines : (kAllLines & ~bit)));
	CHECK(f.peek(&EXTI->IMR) == (m.interrupt ? kAllLines : (kAllLines & ~bit)));
	CHECK(f.peek(&EXTI->EMR) == (m.event ? kAllLines : (kAllLines & ~bit)));
}
} // namespace

TEST_CASE_FIXTURE(HalFixture, "G5 interrupt on the rising edge sets RTSR and IMR and clears FTSR and EMR of the line") {
	checkExtiMode(*this, kModes[0]);
}
TEST_CASE_FIXTURE(HalFixture, "G5 interrupt on the falling edge sets FTSR and IMR and clears RTSR and EMR of the line") {
	checkExtiMode(*this, kModes[1]);
}
TEST_CASE_FIXTURE(HalFixture, "G5 interrupt on both edges sets RTSR FTSR and IMR and clears EMR of the line") {
	checkExtiMode(*this, kModes[2]);
}
TEST_CASE_FIXTURE(HalFixture, "G5 event on the rising edge sets RTSR and EMR and clears FTSR and IMR of the line") {
	checkExtiMode(*this, kModes[3]);
}
TEST_CASE_FIXTURE(HalFixture, "G5 event on the falling edge sets FTSR and EMR and clears RTSR and IMR of the line") {
	checkExtiMode(*this, kModes[4]);
}
TEST_CASE_FIXTURE(HalFixture, "G5 event on both edges sets RTSR FTSR and EMR and clears IMR of the line") {
	checkExtiMode(*this, kModes[5]);
}

TEST_CASE_FIXTURE(HalFixture, "G5 an EXTI mode enables the SYSCFG clock in RCC_APB2ENR bit 14") {
	enableAllPortClocks();
	CHECK((peek(&RCC->APB2ENR) & (1u << 14)) == 0u); // reset value 0 (RM0090 7.3.14)
	initPin(GPIOC, GPIO_PIN_6, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	CHECK((peek(&RCC->APB2ENR) & (1u << 14)) != 0u);
}

TEST_CASE_FIXTURE(HalFixture, "G5 a plain output mode leaves the SYSCFG clock and the EXTI registers untouched") {
	enableAllPortClocks();
	initPin(GPIOC, GPIO_PIN_6, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	CHECK((peek(&RCC->APB2ENR) & (1u << 14)) == 0u);
	CHECK(peek(&EXTI->IMR) == 0u);
	CHECK(peek(&EXTI->RTSR) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "G5 EXTICR selects each of the ports A to I for the line of the pin") {
	enableAllPortClocks();
	for (unsigned p = 0; p < HOST_GPIO_PORTS; ++p) {
		poke(&SYSCFG->EXTICR[1], 0);
		initPin(portOf(p), GPIO_PIN_6, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
		// RM0090 9.2.4: EXTICR2 holds EXTI4..7, line 6 = nibble 2 = bits 11:8; value = port index (0000 A ... 1000 I)
		CHECK(peek(&SYSCFG->EXTICR[1]) == (p << 8));
	}
}

TEST_CASE_FIXTURE(HalFixture, "G5 EXTICR nibbles of other lines keep their value for pins 2 and 13") {
	enableAllPortClocks();
	poke(&SYSCFG->EXTICR[0], 0xFFFFu);
	poke(&SYSCFG->EXTICR[3], 0xFFFFu);
	initPin(GPIOC, GPIO_PIN_2, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	initPin(GPIOD, GPIO_PIN_13, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	// EXTICR1 (9.2.3): line 2 = bits 11:8 = C = 2: 0xFFFF & ~0x0F00 | 0x0200 = 0xF2FF
	CHECK(peek(&SYSCFG->EXTICR[0]) == 0xF2FFu);
	// EXTICR4 (9.2.6): line 13 = nibble 1 = bits 7:4 = D = 3: 0xFFFF & ~0xF0 | 0x30 = 0xFF3F
	CHECK(peek(&SYSCFG->EXTICR[3]) == 0xFF3Fu);
	CHECK(peek(&SYSCFG->EXTICR[1]) == 0u);
	CHECK(peek(&SYSCFG->EXTICR[2]) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "G5 a rising edge on a pin in rising interrupt mode sets its EXTI_PR bit and a falling edge does not") {
	enableAllPortClocks();
	hostGpioSetInput(2, 6, HOST_PIN_LOW);
	initPin(GPIOC, GPIO_PIN_6, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	CHECK(peek(&EXTI->PR) == 0u);
	hostGpioSetInput(2, 6, HOST_PIN_HIGH);
	CHECK(peek(&EXTI->PR) == (1u << 6));
	EXTI->PR = (1u << 6); // rc_w1 (RM0090 12.3.6)
	hostGpioSetInput(2, 6, HOST_PIN_LOW);
	CHECK(peek(&EXTI->PR) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "G5 a falling edge on a pin in falling interrupt mode sets its EXTI_PR bit and a rising edge does not") {
	enableAllPortClocks();
	hostGpioSetInput(3, 9, HOST_PIN_LOW);
	initPin(GPIOD, GPIO_PIN_9, GPIO_MODE_IT_FALLING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	hostGpioSetInput(3, 9, HOST_PIN_HIGH);
	CHECK(peek(&EXTI->PR) == 0u);
	hostGpioSetInput(3, 9, HOST_PIN_LOW);
	CHECK(peek(&EXTI->PR) == (1u << 9));
}

TEST_CASE_FIXTURE(HalFixture, "G5 an edge on a pin of another port than the selected one does not set EXTI_PR") {
	enableAllPortClocks();
	hostGpioSetInput(2, 6, HOST_PIN_LOW);
	hostGpioSetInput(3, 6, HOST_PIN_LOW);
	initPin(GPIOC, GPIO_PIN_6, GPIO_MODE_IT_RISING_FALLING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	hostGpioSetInput(3, 6, HOST_PIN_HIGH); // PD6: line 6 follows PC6 (EXTICR = C)
	CHECK(peek(&EXTI->PR) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "G5 a selected edge in event mode counts an event and does not set EXTI_PR") {
	enableAllPortClocks();
	hostGpioSetInput(4, 1, HOST_PIN_LOW);
	initPin(GPIOE, GPIO_PIN_1, GPIO_MODE_EVT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	const uint32_t before = hostExtiEventCount();
	hostGpioSetInput(4, 1, HOST_PIN_HIGH);
	CHECK(hostExtiEventCount() == before + 1u);
	CHECK(peek(&EXTI->PR) == 0u); // IMR bit clear: no pending request (RM0090 12.2.4)
}

// ---------------------------------------------------------------- G6 several pins
TEST_CASE_FIXTURE(HalFixture, "G6 Init of several pins configures exactly those pins") {
	enableAllPortClocks();
	initPin(GPIOE, GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
	// MODER 01 at bits 3:2, 9:8, 19:18 = 0x4 + 0x100 + 0x40000 = 0x40104; PUPDR 01 at the same bits = 0x40104
	CHECK(peek(&GPIOE->MODER) == 0x40104u);
	CHECK(peek(&GPIOE->PUPDR) == 0x40104u);
	// OSPEEDR 10 (speed 2) at the same bits: 0x8 + 0x200 + 0x80000 = 0x80208
	CHECK(peek(&GPIOE->OSPEEDR) == 0x80208u);
	CHECK(peek(&GPIOE->OTYPER) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "G6 Init with GPIO_PIN_All configures all sixteen pins") {
	enableAllPortClocks();
	initPin(GPIOE, GPIO_PIN_All, GPIO_MODE_OUTPUT_OD, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW);
	CHECK(peek(&GPIOE->MODER) == 0x55555555u);   // 01 in every pair
	CHECK(peek(&GPIOE->OTYPER) == 0xFFFFu);      // open-drain
	CHECK(peek(&GPIOE->PUPDR) == 0xAAAAAAAAu);   // 10 in every pair
}

TEST_CASE_FIXTURE(HalFixture, "G6 Init of several alternate function pins sets each AFR nibble") {
	enableAllPortClocks();
	initPin(GPIOE, GPIO_PIN_0 | GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 7);
	CHECK(peek(&GPIOE->AFR[0]) == 0x7u);  // pin 0: AFRL bits 3:0
	CHECK(peek(&GPIOE->AFR[1]) == 0x70u); // pin 9: AFRH nibble 1, bits 7:4
	CHECK(peek(&GPIOE->MODER) == 0x80002u); // pin 0: 10 at bits 1:0; pin 9: 10 at bits 19:18 -> 0x2 + 0x80000
}

// ---------------------------------------------------------------- G7 DeInit
TEST_CASE_FIXTURE(HalFixture, "G7 DeInit of the pin restores reset values of a pin whose reset values are 0 on ports C to I") {
	enableAllPortClocks();
	for (unsigned p = 2; p < HOST_GPIO_PORTS; ++p) {
		GPIO_TypeDef *port = portOf(p);
		initPin(port, GPIO_PIN_9, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH, 7);
		HAL_GPIO_DeInit(port, GPIO_PIN_9);
		// RM0090 8.4.1-8.4.4, 8.4.9-8.4.10: reset value 0 for all of these registers on ports C..I
		CHECK(peek(&port->MODER) == 0u);
		CHECK(peek(&port->OTYPER) == 0u);
		CHECK(peek(&port->OSPEEDR) == 0u);
		CHECK(peek(&port->PUPDR) == 0u);
		CHECK(peek(&port->AFR[0]) == 0u);
		CHECK(peek(&port->AFR[1]) == 0u);
	}
}

TEST_CASE_FIXTURE(HalFixture, "G7 DeInit of a low pin restores its reset values and keeps the other pins") {
	enableAllPortClocks();
	initPin(GPIOF, GPIO_PIN_2 | GPIO_PIN_4, GPIO_MODE_AF_OD, GPIO_PULLDOWN, GPIO_SPEED_FREQ_HIGH, 3);
	HAL_GPIO_DeInit(GPIOF, GPIO_PIN_2);
	// pin 4 keeps: MODER 10 at bits 9:8 = 0x200, OTYPER bit 4 = 0x10, OSPEEDR 10 = 0x200, PUPDR 10 = 0x200, AFRL nibble 4 = 3 -> 0x30000
	CHECK(peek(&GPIOF->MODER) == 0x200u);
	CHECK(peek(&GPIOF->OTYPER) == 0x10u);
	CHECK(peek(&GPIOF->OSPEEDR) == 0x200u);
	CHECK(peek(&GPIOF->PUPDR) == 0x200u);
	CHECK(peek(&GPIOF->AFR[0]) == 0x30000u);
}

TEST_CASE_FIXTURE(HalFixture, "G7 DeInit of a non-debug pin of port A and B keeps the debug pins at their reset values") {
	enableAllPortClocks();
	initPin(GPIOA, GPIO_PIN_5, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);
	initPin(GPIOB, GPIO_PIN_7, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
	// RM0090 8.4.1-8.4.4: back to the reset values, which are non-zero only for the debug pins
	CHECK(peek(&GPIOA->MODER) == 0xA8000000u);
	CHECK(peek(&GPIOA->OSPEEDR) == 0x0C000000u);
	CHECK(peek(&GPIOA->PUPDR) == 0x64000000u);
	CHECK(peek(&GPIOA->OTYPER) == 0u);
	CHECK(peek(&GPIOB->MODER) == 0x00000280u);
	CHECK(peek(&GPIOB->OSPEEDR) == 0x000000C0u);
	CHECK(peek(&GPIOB->PUPDR) == 0x00000100u);
	CHECK(peek(&GPIOB->OTYPER) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "G7 DeInit of all pins of port A restores the RM0090 reset values" * KNOWN_ISSUE("V89: HAL_GPIO_DeInit clears the debug pins PA13/PA14/PA15 to 0 instead of their reset values (SWD/JTAG alternate function)")) {
	enableAllPortClocks();
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_All);
	// RM0090 8.4.1: MODER reset 0xA8000000 (PA13..15 alternate function); 8.4.3 OSPEEDR 0x0C000000; 8.4.4 PUPDR 0x64000000
	CHECK(peek(&GPIOA->MODER) == 0xA8000000u);
	CHECK(peek(&GPIOA->OSPEEDR) == 0x0C000000u);
	CHECK(peek(&GPIOA->PUPDR) == 0x64000000u);
}

TEST_CASE_FIXTURE(HalFixture, "G7 DeInit of all pins of port B restores the RM0090 reset values" * KNOWN_ISSUE("V89: HAL_GPIO_DeInit clears the debug pins PB3/PB4 to 0 instead of their reset values (JTDO / NJTRST)")) {
	enableAllPortClocks();
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_All);
	// RM0090 8.4.1: MODER reset 0x00000280 (PB3, PB4 alternate function); OSPEEDR 0x000000C0; PUPDR 0x00000100
	CHECK(peek(&GPIOB->MODER) == 0x00000280u);
	CHECK(peek(&GPIOB->OSPEEDR) == 0x000000C0u);
	CHECK(peek(&GPIOB->PUPDR) == 0x00000100u);
}

TEST_CASE_FIXTURE(HalFixture, "G7 DeInit of PA13 restores its MODER reset value of alternate function" * KNOWN_ISSUE("V89: HAL_GPIO_DeInit clears PA13 to input instead of the reset alternate function (SWDIO)")) {
	enableAllPortClocks();
	initPin(GPIOA, GPIO_PIN_13, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_13);
	// PA13 = bits 27:26 of MODER: reset 10 (0xA8000000 bits 27:26 = 10), PUPDR 01, OSPEEDR 11
	CHECK((peek(&GPIOA->MODER) >> 26 & 3u) == 2u);
	CHECK((peek(&GPIOA->PUPDR) >> 26 & 3u) == 1u);
	CHECK((peek(&GPIOA->OSPEEDR) >> 26 & 3u) == 3u);
}

// ---------------------------------------------------------------- G8 DeInit of EXTI pins
TEST_CASE_FIXTURE(HalFixture, "G8 DeInit of an interrupt pin clears IMR RTSR FTSR and the EXTICR nibble of its line") {
	enableAllPortClocks();
	initPin(GPIOC, GPIO_PIN_4, GPIO_MODE_IT_RISING_FALLING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);
	CHECK(peek(&EXTI->IMR) == 0u);
	CHECK(peek(&EXTI->RTSR) == 0u);
	CHECK(peek(&EXTI->FTSR) == 0u);
	CHECK(peek(&SYSCFG->EXTICR[1]) == 0u); // line 4: EXTICR2 bits 3:0 (was C = 2)
}

TEST_CASE_FIXTURE(HalFixture, "G8 DeInit of an event pin clears EMR of its line") {
	enableAllPortClocks();
	initPin(GPIOC, GPIO_PIN_4, GPIO_MODE_EVT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	CHECK(peek(&EXTI->EMR) == (1u << 4));
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);
	CHECK(peek(&EXTI->EMR) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "G8 DeInit of one EXTI pin leaves the EXTI configuration of other lines") {
	enableAllPortClocks();
	initPin(GPIOC, GPIO_PIN_4, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	initPin(GPIOC, GPIO_PIN_5, GPIO_MODE_IT_FALLING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);
	CHECK(peek(&EXTI->IMR) == (1u << 5));
	CHECK(peek(&EXTI->FTSR) == (1u << 5));
	CHECK(peek(&EXTI->RTSR) == 0u);
	CHECK(peek(&SYSCFG->EXTICR[1]) == (2u << 4)); // line 5 nibble 1 still C = 2
}

TEST_CASE_FIXTURE(HalFixture, "G8 DeInit of pin 11 clears only nibble 3 of EXTICR3 and keeps the neighbouring nibbles") {
	enableAllPortClocks();
	initPin(GPIOB, GPIO_PIN_8, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	initPin(GPIOE, GPIO_PIN_9, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	initPin(GPIOD, GPIO_PIN_10, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	initPin(GPIOC, GPIO_PIN_11, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	// RM0090 9.2.5: EXTICR3 nibbles 0..3 = lines 8..11 = B(1) E(4) D(3) C(2) -> 0x2341
	REQUIRE(peek(&SYSCFG->EXTICR[2]) == 0x2341u);
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11);
	CHECK(peek(&SYSCFG->EXTICR[2]) == 0x0341u);
	CHECK(peek(&EXTI->IMR) == ((1u << 8) | (1u << 9) | (1u << 10)));
}

TEST_CASE_FIXTURE(HalFixture, "G8 DeInit of pin 14 clears only nibble 2 of EXTICR4 and keeps the neighbouring nibbles") {
	enableAllPortClocks();
	initPin(GPIOF, GPIO_PIN_12, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	initPin(GPIOG, GPIO_PIN_13, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	initPin(GPIOH, GPIO_PIN_14, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	initPin(GPIOI, GPIO_PIN_15, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	// RM0090 9.2.6: EXTICR4 nibbles 0..3 = lines 12..15 = F(5) G(6) H(7) I(8) -> 0x8765
	REQUIRE(peek(&SYSCFG->EXTICR[3]) == 0x8765u);
	HAL_GPIO_DeInit(GPIOH, GPIO_PIN_14);
	CHECK(peek(&SYSCFG->EXTICR[3]) == 0x8065u);
	CHECK(peek(&EXTI->IMR) == ((1u << 12) | (1u << 13) | (1u << 15)));
}

TEST_CASE_FIXTURE(HalFixture, "G8 DeInit of a pin whose line is owned by another port keeps the EXTI configuration") {
	enableAllPortClocks();
	initPin(GPIOC, GPIO_PIN_4, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	initPin(GPIOD, GPIO_PIN_4, GPIO_MODE_IT_FALLING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW); // line 4 now selects D (RM0090 9.2.4: one port per line)
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);
	CHECK(peek(&EXTI->IMR) == (1u << 4));
	CHECK(peek(&EXTI->FTSR) == (1u << 4));
	CHECK(peek(&SYSCFG->EXTICR[1]) == 3u); // D = 3
}

// ---------------------------------------------------------------- G9 ReadPin
TEST_CASE_FIXTURE(HalFixture, "G9 ReadPin of an input driven high returns SET") {
	enableAllPortClocks();
	initPin(GPIOC, GPIO_PIN_8, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	hostGpioSetInput(2, 8, HOST_PIN_HIGH);
	CHECK(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET);
}

TEST_CASE_FIXTURE(HalFixture, "G9 ReadPin of an input driven low returns RESET") {
	enableAllPortClocks();
	initPin(GPIOC, GPIO_PIN_8, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	hostGpioSetInput(2, 8, HOST_PIN_LOW);
	CHECK(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET);
}

TEST_CASE_FIXTURE(HalFixture, "G9 ReadPin of a floating input with pull-up returns SET") {
	enableAllPortClocks();
	initPin(GPIOC, GPIO_PIN_8, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);
	CHECK(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET); // RM0090 8.3.2 figure: pull-up on an undriven pin reads 1
}

TEST_CASE_FIXTURE(HalFixture, "G9 ReadPin of a floating input with pull-down returns RESET") {
	enableAllPortClocks();
	initPin(GPIOC, GPIO_PIN_8, GPIO_MODE_INPUT, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW);
	CHECK(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET);
}

TEST_CASE_FIXTURE(HalFixture, "G9 ReadPin of a push-pull output returns its ODR level") {
	enableAllPortClocks();
	initPin(GPIOC, GPIO_PIN_8, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	const GPIO_PinState high = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8); // RM0090 8.3.10: IDR gives the I/O state
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	const GPIO_PinState low = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
	CHECK(high == GPIO_PIN_SET);
	CHECK(low == GPIO_PIN_RESET);
}

TEST_CASE_FIXTURE(HalFixture, "G9 ReadPin of a pin ignores the level of other pins of the port") {
	enableAllPortClocks();
	for (unsigned pin = 0; pin < 16; ++pin) hostGpioSetInput(2, pin, pin == 8 ? HOST_PIN_LOW : HOST_PIN_HIGH);
	CHECK(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET);
}

// ---------------------------------------------------------------- G10 WritePin
TEST_CASE_FIXTURE(HalFixture, "G10 WritePin SET of several pins sets exactly those ODR bits") {
	enableAllPortClocks();
	poke(&GPIOD->ODR, 0x00F0u);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8, GPIO_PIN_SET);
	// RM0090 8.3.5 / 8.4.7: 0x00F0 | 0x0001 | 0x0002 | 0x0100 = 0x01F3
	CHECK(peek(&GPIOD->ODR) == 0x01F3u);
}

TEST_CASE_FIXTURE(HalFixture, "G10 WritePin RESET of several pins resets exactly those ODR bits") {
	enableAllPortClocks();
	poke(&GPIOD->ODR, 0xFFFFu);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_15, GPIO_PIN_RESET);
	// 0xFFFF & ~(0x0010 | 0x0020 | 0x8000) = 0x7FCF
	CHECK(peek(&GPIOD->ODR) == 0x7FCFu);
}

TEST_CASE_FIXTURE(HalFixture, "G10 WritePin does not change the ODR of another port") {
	enableAllPortClocks();
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_All, GPIO_PIN_SET);
	CHECK(peek(&GPIOD->ODR) == 0xFFFFu);
	CHECK(peek(&GPIOE->ODR) == 0u);
}

// ---------------------------------------------------------------- G11 TogglePin
TEST_CASE_FIXTURE(HalFixture, "G11 TogglePin of several pins with mixed levels inverts exactly those pins") {
	enableAllPortClocks();
	poke(&GPIOD->ODR, 0x00F0u);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_15);
	// toggled mask 0x80C3; 0x00F0 ^ 0x80C3 = 0x8033 (pins 0,1,15 low -> high; pins 6,7 high -> low; pins 4,5 keep)
	CHECK(peek(&GPIOD->ODR) == 0x8033u);
}

TEST_CASE_FIXTURE(HalFixture, "G11 TogglePin twice restores the level of the pin") {
	enableAllPortClocks();
	poke(&GPIOD->ODR, 0x0100u);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);
	CHECK(peek(&GPIOD->ODR) == 0x0108u);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);
	CHECK(peek(&GPIOD->ODR) == 0x0100u);
}

// ---------------------------------------------------------------- G12 / G13 LockPin
TEST_CASE_FIXTURE(HalFixture, "G12 LockPin returns HAL_OK and locks exactly the selected pins") {
	enableAllPortClocks();
	CHECK(HAL_GPIO_LockPin(GPIOD, GPIO_PIN_3 | GPIO_PIN_9) == HAL_OK);
	CHECK(hostGpioLocked(3, 3) == 1);
	CHECK(hostGpioLocked(3, 9) == 1);
	CHECK(hostGpioLocked(3, 4) == 0);
	CHECK(hostGpioLocked(4, 3) == 0); // another port
}

TEST_CASE_FIXTURE(HalFixture, "G12 Init of a locked pin leaves MODER OTYPER OSPEEDR PUPDR and AFR of the pin unchanged") {
	enableAllPortClocks();
	initPin(GPIOD, GPIO_PIN_3, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);
	CHECK(HAL_GPIO_LockPin(GPIOD, GPIO_PIN_3) == HAL_OK);
	initPin(GPIOD, GPIO_PIN_3, GPIO_MODE_AF_OD, GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, 5);
	// RM0090 8.4.8: the locked pin keeps its configuration: output (01), push-pull, speed 00, pull-up 01, AF 0
	CHECK(peek(&GPIOD->MODER) == (1u << 6));
	CHECK(peek(&GPIOD->OTYPER) == 0u);
	CHECK(peek(&GPIOD->OSPEEDR) == 0u);
	CHECK(peek(&GPIOD->PUPDR) == (1u << 6));
	CHECK(peek(&GPIOD->AFR[0]) == 0u);
}

TEST_CASE_FIXTURE(HalFixture, "G12 Init of an unlocked pin of a port with a locked pin changes its configuration") {
	enableAllPortClocks();
	CHECK(HAL_GPIO_LockPin(GPIOD, GPIO_PIN_3) == HAL_OK);
	initPin(GPIOD, GPIO_PIN_4, GPIO_MODE_AF_OD, GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, 5);
	CHECK(peek(&GPIOD->MODER) == (2u << 8));       // pin 4: bits 9:8 = 10
	CHECK(peek(&GPIOD->OTYPER) == (1u << 4));
	CHECK(peek(&GPIOD->OSPEEDR) == (3u << 8));
	CHECK(peek(&GPIOD->PUPDR) == (2u << 8));
	CHECK(peek(&GPIOD->AFR[0]) == (5u << 16));     // AFRL nibble 4 = bits 19:16
}

TEST_CASE_FIXTURE(HalFixture, "characterization: G13 LockPin of an already locked port returns HAL_OK although the new pins stay unlocked") {
	// RM0090 8.4.8: LCKR is locked until the next reset, so the second sequence cannot lock pin 5; UM1725 gives no error case.
	enableAllPortClocks();
	CHECK(HAL_GPIO_LockPin(GPIOD, GPIO_PIN_3) == HAL_OK);
	CHECK(HAL_GPIO_LockPin(GPIOD, GPIO_PIN_5) == HAL_OK);
	CHECK(hostGpioLocked(3, 5) == 0);
	CHECK(hostGpioLocked(3, 3) == 1);
}

// ---------------------------------------------------------------- G14 IRQ handler
TEST_CASE_FIXTURE(HalFixture, "G14 EXTI_IRQHandler of a pending line clears PR and calls the callback once with the pin") {
	resetCallbackRecorder();
	EXTI->IMR = (1u << 5);
	EXTI->SWIER = (1u << 5); // RM0090 12.3.5: sets PR because IMR is set
	REQUIRE(peek(&EXTI->PR) == (1u << 5));
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
	CHECK(peek(&EXTI->PR) == 0u);
	CHECK(g_cbCount == 1);
	CHECK(g_cbPin == GPIO_PIN_5);
}

TEST_CASE_FIXTURE(HalFixture, "G14 EXTI_IRQHandler of a line that is not pending calls nothing and leaves PR") {
	resetCallbackRecorder();
	EXTI->IMR = (1u << 6);
	EXTI->SWIER = (1u << 6); // only line 6 pending
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
	CHECK(g_cbCount == 0);
	CHECK(peek(&EXTI->PR) == (1u << 6));
}

TEST_CASE_FIXTURE(HalFixture, "G14 EXTI_IRQHandler clears only the PR bit of the handled pin") {
	resetCallbackRecorder();
	EXTI->IMR = (1u << 5) | (1u << 6);
	EXTI->SWIER = (1u << 5) | (1u << 6);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
	CHECK(peek(&EXTI->PR) == (1u << 6)); // rc_w1: writing 1 to bit 5 only (RM0090 12.3.6)
	CHECK(g_cbCount == 1);
}

} // TEST_SUITE
