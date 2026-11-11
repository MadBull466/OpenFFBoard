/*
 * P12 T1 EXTI cases (requirements E1-E10 of Tests/Code_Coverage/claude/charters/hal/P12_T1_REQUIREMENTS.md).
 * Sources under test: stm32f4xx_hal_exti.c of the real ST HAL, built into ffb_hal_tests and ffb_hal_tests_disco.
 *
 * Oracles: RM0090 Rev 22 (9.2.3-9.2.6 SYSCFG_EXTICRx, 12.2.4 / 12.2.5 EXTI functions and lines, 12.3.1-12.3.6 EXTI
 * registers) and UM1725 Rev 8 chapter 24 (EXTI HAL). Register values below are derived from RM0090 encodings:
 *  - IMR / EMR / RTSR / FTSR / PR bit n = line n (12.3.1-12.3.4, 12.3.6).
 *  - EXTICR[n / 4] holds the port of line n in bits [4 * (n % 4) + 3 : 4 * (n % 4)], 0000 = A, 0001 = B, 0010 = C ... 1000 = I.
 * No HAL weak function is defined here (the EXTI handle callback is registered with HAL_EXTI_RegisterCallback).
 */
#include "HalFixture.h"

#include <cstdint>

namespace {

const uint32_t kAllLines = 0x007FFFFFu; // lines 0..22, the implemented bits (RM0090 12.3.1)

int g_cbCalls = 0;
void countingCallback() { ++g_cbCalls; }

struct ExtiFixture : HalFixture {
	ExtiFixture() { g_cbCalls = 0; }
	EXTI_HandleTypeDef h = {};
	EXTI_ConfigTypeDef cfg = {};

	static uint32_t cr(unsigned i) { return peek(&SYSCFG->EXTICR[i]); }
	static uint32_t nibble(unsigned line) { return (cr(line / 4u) >> (4u * (line % 4u))) & 0xFu; }
	// Makes line pending through the software interrupt (RM0090 12.3.5: needs the IMR bit).
	static void makePending(unsigned line) {
		poke(&EXTI->IMR, peek(&EXTI->IMR) | (1u << line));
		EXTI->SWIER = 1u << line;
	}
	void setLine(uint32_t line, uint32_t trig, uint32_t mode, uint32_t port = EXTI_GPIOA) {
		cfg.Line = line;
		cfg.Trigger = trig;
		cfg.Mode = mode;
		cfg.GPIOSel = port;
		REQUIRE(HAL_EXTI_SetConfigLine(&h, &cfg) == HAL_OK);
	}
};

} // namespace

TEST_SUITE("P12 T1 EXTI") {

// ---- E1 ----

TEST_CASE_FIXTURE(ExtiFixture, "E1 SetConfigLine with a NULL handle returns HAL_ERROR and accesses no register") {
	cfg.Line = EXTI_LINE_3;
	cfg.Mode = EXTI_MODE_INTERRUPT;
	CHECK(HAL_EXTI_SetConfigLine(nullptr, &cfg) == HAL_ERROR);
	CHECK(hostRegsLogCount() == 0u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E1 SetConfigLine with a NULL config returns HAL_ERROR and accesses no register") {
	CHECK(HAL_EXTI_SetConfigLine(&h, nullptr) == HAL_ERROR);
	CHECK(hostRegsLogCount() == 0u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E1 GetConfigLine with a NULL handle or a NULL config returns HAL_ERROR and accesses no register") {
	CHECK(HAL_EXTI_GetConfigLine(nullptr, &cfg) == HAL_ERROR);
	CHECK(HAL_EXTI_GetConfigLine(&h, nullptr) == HAL_ERROR);
	CHECK(hostRegsLogCount() == 0u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E1 ClearConfigLine with a NULL handle returns HAL_ERROR and accesses no register") {
	CHECK(HAL_EXTI_ClearConfigLine(nullptr) == HAL_ERROR);
	CHECK(hostRegsLogCount() == 0u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E1 GetHandle with a NULL handle returns HAL_ERROR and accesses no register") {
	CHECK(HAL_EXTI_GetHandle(nullptr, EXTI_LINE_3) == HAL_ERROR);
	CHECK(hostRegsLogCount() == 0u);
}

// ---- E2 ----

TEST_CASE_FIXTURE(ExtiFixture, "E2 a rising trigger on a GPIO line sets only its RTSR bit") {
	setLine(EXTI_LINE_5, EXTI_TRIGGER_RISING, EXTI_MODE_INTERRUPT);
	CHECK(peek(&EXTI->RTSR) == (1u << 5)); // RM0090 12.3.3 bit 5
	CHECK(peek(&EXTI->FTSR) == 0u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E2 a falling trigger on a GPIO line sets only its FTSR bit") {
	setLine(EXTI_LINE_9, EXTI_TRIGGER_FALLING, EXTI_MODE_INTERRUPT);
	CHECK(peek(&EXTI->FTSR) == (1u << 9)); // RM0090 12.3.4
	CHECK(peek(&EXTI->RTSR) == 0u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E2 a rising and falling trigger on a GPIO line sets both its RTSR and FTSR bits") {
	setLine(EXTI_LINE_12, EXTI_TRIGGER_RISING_FALLING, EXTI_MODE_INTERRUPT);
	CHECK(peek(&EXTI->RTSR) == (1u << 12));
	CHECK(peek(&EXTI->FTSR) == (1u << 12));
}

TEST_CASE_FIXTURE(ExtiFixture, "E2 trigger none on a GPIO line clears its RTSR and FTSR bits and keeps the other lines") {
	poke(&EXTI->RTSR, kAllLines);
	poke(&EXTI->FTSR, kAllLines);
	setLine(EXTI_LINE_5, EXTI_TRIGGER_NONE, EXTI_MODE_INTERRUPT);
	CHECK(peek(&EXTI->RTSR) == (kAllLines & ~(1u << 5)));
	CHECK(peek(&EXTI->FTSR) == (kAllLines & ~(1u << 5)));
}

TEST_CASE_FIXTURE(ExtiFixture, "E2 interrupt mode on a GPIO line sets only its IMR bit") {
	setLine(EXTI_LINE_7, EXTI_TRIGGER_RISING, EXTI_MODE_INTERRUPT);
	CHECK(peek(&EXTI->IMR) == (1u << 7)); // RM0090 12.3.1
	CHECK(peek(&EXTI->EMR) == 0u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E2 event mode on a GPIO line sets only its EMR bit") {
	setLine(EXTI_LINE_7, EXTI_TRIGGER_RISING, EXTI_MODE_EVENT);
	CHECK(peek(&EXTI->EMR) == (1u << 7)); // RM0090 12.3.2
	CHECK(peek(&EXTI->IMR) == 0u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E2 interrupt and event mode on a GPIO line sets both its IMR and EMR bits") {
	setLine(EXTI_LINE_14, EXTI_TRIGGER_RISING, EXTI_MODE_INTERRUPT | EXTI_MODE_EVENT);
	CHECK(peek(&EXTI->IMR) == (1u << 14));
	CHECK(peek(&EXTI->EMR) == (1u << 14));
}

TEST_CASE_FIXTURE(ExtiFixture, "E2 mode none on a GPIO line clears its IMR and EMR bits and keeps the other lines") {
	poke(&EXTI->IMR, kAllLines);
	poke(&EXTI->EMR, kAllLines);
	setLine(EXTI_LINE_2, EXTI_TRIGGER_RISING, EXTI_MODE_NONE);
	CHECK(peek(&EXTI->IMR) == (kAllLines & ~(1u << 2)));
	CHECK(peek(&EXTI->EMR) == (kAllLines & ~(1u << 2)));
}

TEST_CASE_FIXTURE(ExtiFixture, "E2 the GPIO selection of every GPIO line is written to its EXTICR nibble and the other nibbles stay") {
	for (unsigned line = 0; line < 16; ++line) {
		for (unsigned i = 0; i < 4; ++i) poke(&SYSCFG->EXTICR[i], 0x7777u); // other nibbles: port H
		const uint32_t port = (line % 9u); // 0 = A ... 8 = I (RM0090 9.2.3)
		setLine(EXTI_LINE_0 + line, EXTI_TRIGGER_RISING, EXTI_MODE_INTERRUPT, port);
		INFO("line " << line);
		CHECK(nibble(line) == port);
		for (unsigned other = 0; other < 16; ++other) {
			if (other != line) CHECK(nibble(other) == 7u);
		}
	}
}

TEST_CASE_FIXTURE(ExtiFixture, "E2 SetConfigLine of a GPIO line stores the line in the handle and returns HAL_OK") {
	cfg.Line = EXTI_LINE_11;
	cfg.Trigger = EXTI_TRIGGER_RISING;
	cfg.Mode = EXTI_MODE_INTERRUPT;
	cfg.GPIOSel = EXTI_GPIOC;
	CHECK(HAL_EXTI_SetConfigLine(&h, &cfg) == HAL_OK);
	CHECK(h.Line == static_cast<uint32_t>(EXTI_LINE_11));
}

TEST_CASE_FIXTURE(ExtiFixture, "E2 configuring one GPIO line leaves every other line of IMR, EMR, RTSR and FTSR as it was") {
	const uint32_t others = kAllLines & ~(1u << 6);
	poke(&EXTI->IMR, others);
	poke(&EXTI->EMR, 0x00155555u & others);
	poke(&EXTI->RTSR, 0x002AAAAAu & others);
	poke(&EXTI->FTSR, 0x00123456u & others);
	setLine(EXTI_LINE_6, EXTI_TRIGGER_RISING_FALLING, EXTI_MODE_INTERRUPT | EXTI_MODE_EVENT);
	CHECK(peek(&EXTI->IMR) == kAllLines);
	CHECK(peek(&EXTI->EMR) == ((0x00155555u & others) | (1u << 6)));
	CHECK(peek(&EXTI->RTSR) == ((0x002AAAAAu & others) | (1u << 6)));
	CHECK(peek(&EXTI->FTSR) == ((0x00123456u & others) | (1u << 6)));
}

// ---- E3 ----

TEST_CASE_FIXTURE(ExtiFixture, "E3 a non-GPIO line 16 to 22 gets its trigger bits as asked") {
	const uint32_t lines[] = {EXTI_LINE_16, EXTI_LINE_17, EXTI_LINE_18, EXTI_LINE_19, EXTI_LINE_20, EXTI_LINE_21, EXTI_LINE_22};
	for (unsigned n = 16; n <= 22; ++n) {
		poke(&EXTI->RTSR, 0);
		poke(&EXTI->FTSR, 0);
		setLine(lines[n - 16], EXTI_TRIGGER_RISING, EXTI_MODE_INTERRUPT);
		INFO("line " << n);
		CHECK(peek(&EXTI->RTSR) == (1u << n));
		CHECK(peek(&EXTI->FTSR) == 0u);
		setLine(lines[n - 16], EXTI_TRIGGER_FALLING, EXTI_MODE_INTERRUPT);
		CHECK(peek(&EXTI->RTSR) == 0u);
		CHECK(peek(&EXTI->FTSR) == (1u << n));
		setLine(lines[n - 16], EXTI_TRIGGER_RISING_FALLING, EXTI_MODE_INTERRUPT);
		CHECK(peek(&EXTI->RTSR) == (1u << n));
		CHECK(peek(&EXTI->FTSR) == (1u << n));
		setLine(lines[n - 16], EXTI_TRIGGER_NONE, EXTI_MODE_INTERRUPT);
		CHECK(peek(&EXTI->RTSR) == 0u);
		CHECK(peek(&EXTI->FTSR) == 0u);
	}
}

TEST_CASE_FIXTURE(ExtiFixture, "E3 a non-GPIO line 16 to 22 gets its IMR and EMR bits as asked") {
	const uint32_t lines[] = {EXTI_LINE_16, EXTI_LINE_17, EXTI_LINE_18, EXTI_LINE_19, EXTI_LINE_20, EXTI_LINE_21, EXTI_LINE_22};
	for (unsigned n = 16; n <= 22; ++n) {
		poke(&EXTI->IMR, 0);
		poke(&EXTI->EMR, 0);
		INFO("line " << n);
		setLine(lines[n - 16], EXTI_TRIGGER_RISING, EXTI_MODE_INTERRUPT);
		CHECK(peek(&EXTI->IMR) == (1u << n));
		CHECK(peek(&EXTI->EMR) == 0u);
		setLine(lines[n - 16], EXTI_TRIGGER_RISING, EXTI_MODE_EVENT);
		CHECK(peek(&EXTI->IMR) == 0u);
		CHECK(peek(&EXTI->EMR) == (1u << n));
		setLine(lines[n - 16], EXTI_TRIGGER_RISING, EXTI_MODE_INTERRUPT | EXTI_MODE_EVENT);
		CHECK(peek(&EXTI->IMR) == (1u << n));
		CHECK(peek(&EXTI->EMR) == (1u << n));
		setLine(lines[n - 16], EXTI_TRIGGER_RISING, EXTI_MODE_NONE);
		CHECK(peek(&EXTI->IMR) == 0u);
		CHECK(peek(&EXTI->EMR) == 0u);
	}
}

TEST_CASE_FIXTURE(ExtiFixture, "E3 configuring a non-GPIO line writes no SYSCFG_EXTICR register") {
	// RM0090 12.2.5: lines 16-22 are internal events, not selected by SYSCFG_EXTICR (9.2.3 covers lines 0-15).
	setLine(EXTI_LINE_17, EXTI_TRIGGER_RISING_FALLING, EXTI_MODE_INTERRUPT | EXTI_MODE_EVENT, EXTI_GPIOI);
	for (unsigned i = 0; i < 4; ++i) {
		CHECK(hostRegsWritesTo(reinterpret_cast<std::uintptr_t>(&SYSCFG->EXTICR[i])) == 0u);
		CHECK(cr(i) == 0u);
	}
}

// ---- E4 ----

TEST_CASE_FIXTURE(ExtiFixture, "E4 GetConfigLine returns what SetConfigLine set for a GPIO line") {
	setLine(EXTI_LINE_10, EXTI_TRIGGER_RISING_FALLING, EXTI_MODE_INTERRUPT | EXTI_MODE_EVENT, EXTI_GPIOE);
	EXTI_ConfigTypeDef got = {};
	REQUIRE(HAL_EXTI_GetConfigLine(&h, &got) == HAL_OK);
	CHECK(got.Line == static_cast<uint32_t>(EXTI_LINE_10));
	CHECK(got.Mode == static_cast<uint32_t>(EXTI_MODE_INTERRUPT | EXTI_MODE_EVENT));
	CHECK(got.Trigger == static_cast<uint32_t>(EXTI_TRIGGER_RISING_FALLING));
	CHECK(got.GPIOSel == static_cast<uint32_t>(EXTI_GPIOE));
}

TEST_CASE_FIXTURE(ExtiFixture, "E4 GetConfigLine returns port I (selection 1000) for a GPIO line") {
	// RM0090 9.2.3: EXTIx = 1000 selects PI[x]. Line 2: EXTICR1 bits 11:8.
	h.Line = EXTI_LINE_2;
	poke(&SYSCFG->EXTICR[0], 0x8u << 8);
	EXTI_ConfigTypeDef got = {};
	REQUIRE(HAL_EXTI_GetConfigLine(&h, &got) == HAL_OK);
	CHECK(got.GPIOSel == static_cast<uint32_t>(EXTI_GPIOI));
	CHECK(got.GPIOSel == 8u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E4 GetConfigLine returns what SetConfigLine set for a non-GPIO line") {
	setLine(EXTI_LINE_21, EXTI_TRIGGER_FALLING, EXTI_MODE_EVENT);
	EXTI_ConfigTypeDef got = {};
	REQUIRE(HAL_EXTI_GetConfigLine(&h, &got) == HAL_OK);
	CHECK(got.Line == static_cast<uint32_t>(EXTI_LINE_21));
	CHECK(got.Mode == static_cast<uint32_t>(EXTI_MODE_EVENT));
	CHECK(got.Trigger == static_cast<uint32_t>(EXTI_TRIGGER_FALLING));
}

TEST_CASE_FIXTURE(ExtiFixture, "E4 GetConfigLine reads a configuration written to the registers") {
	// Line 13 on port G: IMR13, RTSR13; EXTICR4 nibble 1 (bits 7:4) = 0110 (G, RM0090 9.2.6).
	h.Line = EXTI_LINE_13;
	poke(&EXTI->IMR, 1u << 13);
	poke(&EXTI->RTSR, 1u << 13);
	poke(&SYSCFG->EXTICR[3], 0x6u << 4);
	EXTI_ConfigTypeDef got = {};
	REQUIRE(HAL_EXTI_GetConfigLine(&h, &got) == HAL_OK);
	CHECK(got.Mode == static_cast<uint32_t>(EXTI_MODE_INTERRUPT));
	CHECK(got.Trigger == static_cast<uint32_t>(EXTI_TRIGGER_RISING));
	CHECK(got.GPIOSel == static_cast<uint32_t>(EXTI_GPIOG));
}

TEST_CASE_FIXTURE(ExtiFixture, "E4 GetConfigLine of a line with nothing configured returns mode none and trigger none") {
	h.Line = EXTI_LINE_4;
	EXTI_ConfigTypeDef got = {};
	got.Mode = 0xFFu;
	got.Trigger = 0xFFu;
	REQUIRE(HAL_EXTI_GetConfigLine(&h, &got) == HAL_OK);
	CHECK(got.Mode == static_cast<uint32_t>(EXTI_MODE_NONE));
	CHECK(got.Trigger == static_cast<uint32_t>(EXTI_TRIGGER_NONE));
}

// ---- E5 ----

TEST_CASE_FIXTURE(ExtiFixture, "E5 ClearConfigLine of a GPIO line clears its IMR, EMR, RTSR, FTSR bits and keeps the other lines") {
	poke(&EXTI->IMR, kAllLines);
	poke(&EXTI->EMR, kAllLines);
	poke(&EXTI->RTSR, kAllLines);
	poke(&EXTI->FTSR, kAllLines);
	h.Line = EXTI_LINE_8;
	CHECK(HAL_EXTI_ClearConfigLine(&h) == HAL_OK);
	const uint32_t expected = kAllLines & ~(1u << 8);
	CHECK(peek(&EXTI->IMR) == expected);
	CHECK(peek(&EXTI->EMR) == expected);
	CHECK(peek(&EXTI->RTSR) == expected);
	CHECK(peek(&EXTI->FTSR) == expected);
}

TEST_CASE_FIXTURE(ExtiFixture, "E5 ClearConfigLine of a GPIO line clears its EXTICR nibble and keeps the other nibbles") {
	for (unsigned i = 0; i < 4; ++i) poke(&SYSCFG->EXTICR[i], 0x8888u);
	h.Line = EXTI_LINE_9; // EXTICR3 nibble 1
	CHECK(HAL_EXTI_ClearConfigLine(&h) == HAL_OK);
	CHECK(cr(2) == 0x8808u); // RM0090 9.2.5: bits 7:4 cleared
	CHECK(cr(0) == 0x8888u);
	CHECK(cr(1) == 0x8888u);
	CHECK(cr(3) == 0x8888u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E5 ClearConfigLine of line 14 clears EXTICR4 bits 11:8 and keeps the neighbours") {
	for (unsigned i = 0; i < 4; ++i) poke(&SYSCFG->EXTICR[i], 0x8888u);
	h.Line = EXTI_LINE_14; // EXTICR4 nibble 2 (RM0090 9.2.6)
	CHECK(HAL_EXTI_ClearConfigLine(&h) == HAL_OK);
	CHECK(cr(3) == 0x8088u);
	CHECK(cr(0) == 0x8888u);
	CHECK(cr(1) == 0x8888u);
	CHECK(cr(2) == 0x8888u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E5 ClearConfigLine of a non-GPIO line clears its four bits, keeps the other lines and writes no EXTICR") {
	poke(&EXTI->IMR, kAllLines);
	poke(&EXTI->EMR, kAllLines);
	poke(&EXTI->RTSR, kAllLines);
	poke(&EXTI->FTSR, kAllLines);
	h.Line = EXTI_LINE_19;
	CHECK(HAL_EXTI_ClearConfigLine(&h) == HAL_OK);
	const uint32_t expected = kAllLines & ~(1u << 19);
	CHECK(peek(&EXTI->IMR) == expected);
	CHECK(peek(&EXTI->EMR) == expected);
	CHECK(peek(&EXTI->RTSR) == expected);
	CHECK(peek(&EXTI->FTSR) == expected);
	for (unsigned i = 0; i < 4; ++i) CHECK(hostRegsWritesTo(reinterpret_cast<std::uintptr_t>(&SYSCFG->EXTICR[i])) == 0u);
}

// ---- E6 ----

TEST_CASE_FIXTURE(ExtiFixture, "E6 RegisterCallback with the common callback id returns HAL_OK and the callback is called on a pending line") {
	h.Line = EXTI_LINE_1;
	CHECK(HAL_EXTI_RegisterCallback(&h, HAL_EXTI_COMMON_CB_ID, countingCallback) == HAL_OK);
	makePending(1);
	HAL_EXTI_IRQHandler(&h);
	CHECK(g_cbCalls == 1);
}

TEST_CASE_FIXTURE(ExtiFixture, "E6 RegisterCallback with any other id returns HAL_ERROR and stores nothing") {
	h.Line = EXTI_LINE_1;
	CHECK(HAL_EXTI_RegisterCallback(&h, static_cast<EXTI_CallbackIDTypeDef>(1), countingCallback) == HAL_ERROR);
	CHECK(HAL_EXTI_RegisterCallback(&h, static_cast<EXTI_CallbackIDTypeDef>(0x7F), countingCallback) == HAL_ERROR);
	makePending(1);
	HAL_EXTI_IRQHandler(&h);
	CHECK(g_cbCalls == 0);
}

TEST_CASE_FIXTURE(ExtiFixture, "E6 GetHandle stores the line in the handle and returns HAL_OK") {
	CHECK(HAL_EXTI_GetHandle(&h, EXTI_LINE_20) == HAL_OK);
	CHECK(h.Line == static_cast<uint32_t>(EXTI_LINE_20));
}

// ---- E7 ----

TEST_CASE_FIXTURE(ExtiFixture, "E7 IRQHandler of a pending line clears its PR bit and calls the callback once") {
	h.Line = EXTI_LINE_6;
	REQUIRE(HAL_EXTI_RegisterCallback(&h, HAL_EXTI_COMMON_CB_ID, countingCallback) == HAL_OK);
	makePending(6);
	REQUIRE(peek(&EXTI->PR) == (1u << 6));
	HAL_EXTI_IRQHandler(&h);
	CHECK(peek(&EXTI->PR) == 0u); // RM0090 12.3.6: cleared by writing 1
	CHECK(g_cbCalls == 1);
}

TEST_CASE_FIXTURE(ExtiFixture, "E7 IRQHandler of a pending line without a registered callback clears PR and returns") {
	h.Line = EXTI_LINE_6; // handle zero-initialised: no callback
	makePending(6);
	HAL_EXTI_IRQHandler(&h);
	CHECK(peek(&EXTI->PR) == 0u);
	CHECK(g_cbCalls == 0);
}

TEST_CASE_FIXTURE(ExtiFixture, "E7 IRQHandler of a line that is not pending calls nothing and leaves PR") {
	h.Line = EXTI_LINE_6;
	REQUIRE(HAL_EXTI_RegisterCallback(&h, HAL_EXTI_COMMON_CB_ID, countingCallback) == HAL_OK);
	makePending(3); // another line
	HAL_EXTI_IRQHandler(&h);
	CHECK(g_cbCalls == 0);
	CHECK(peek(&EXTI->PR) == (1u << 3));
}

TEST_CASE_FIXTURE(ExtiFixture, "E7 IRQHandler clears only the PR bit of the handle's line") {
	h.Line = EXTI_LINE_3;
	REQUIRE(HAL_EXTI_RegisterCallback(&h, HAL_EXTI_COMMON_CB_ID, countingCallback) == HAL_OK);
	makePending(3);
	makePending(5);
	makePending(20);
	HAL_EXTI_IRQHandler(&h);
	CHECK(peek(&EXTI->PR) == ((1u << 5) | (1u << 20)));
	CHECK(g_cbCalls == 1);
}

// ---- E8 ----

TEST_CASE_FIXTURE(ExtiFixture, "E8 GetPending returns 1 for a pending line and 0 for a line that is not") {
	makePending(4);
	h.Line = EXTI_LINE_4;
	CHECK(HAL_EXTI_GetPending(&h, EXTI_TRIGGER_RISING_FALLING) == 1u);
	h.Line = EXTI_LINE_5;
	CHECK(HAL_EXTI_GetPending(&h, EXTI_TRIGGER_RISING_FALLING) == 0u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E8 GetPending reads the PR bit of a non-GPIO line") {
	poke(&EXTI->PR, 1u << 22);
	h.Line = EXTI_LINE_22;
	CHECK(HAL_EXTI_GetPending(&h, EXTI_TRIGGER_RISING_FALLING) == 1u);
	h.Line = EXTI_LINE_21;
	CHECK(HAL_EXTI_GetPending(&h, EXTI_TRIGGER_RISING_FALLING) == 0u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E8 ClearPending clears exactly the PR bit of the line") {
	makePending(2);
	makePending(4);
	h.Line = EXTI_LINE_2;
	HAL_EXTI_ClearPending(&h, EXTI_TRIGGER_RISING_FALLING);
	CHECK(peek(&EXTI->PR) == (1u << 4)); // rc_w1: a 0 bit in the write has no effect (12.3.6)
}

TEST_CASE_FIXTURE(ExtiFixture, "E8 ClearPending of a line that is not pending changes nothing") {
	makePending(4);
	h.Line = EXTI_LINE_9;
	HAL_EXTI_ClearPending(&h, EXTI_TRIGGER_RISING_FALLING);
	CHECK(peek(&EXTI->PR) == (1u << 4));
}

// ---- E9 ----

TEST_CASE_FIXTURE(ExtiFixture, "E9 GenerateSWI with the IMR bit set makes the line pending") {
	poke(&EXTI->IMR, 1u << 7);
	h.Line = EXTI_LINE_7;
	HAL_EXTI_GenerateSWI(&h);
	CHECK(peek(&EXTI->PR) == (1u << 7)); // RM0090 12.3.5
}

TEST_CASE_FIXTURE(ExtiFixture, "E9 GenerateSWI with the IMR bit clear leaves PR at 0") {
	poke(&EXTI->IMR, kAllLines & ~(1u << 7));
	h.Line = EXTI_LINE_7;
	HAL_EXTI_GenerateSWI(&h);
	CHECK(peek(&EXTI->PR) == 0u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E9 GenerateSWI on a non-GPIO line with the IMR bit set makes that line pending") {
	poke(&EXTI->IMR, 1u << 20);
	h.Line = EXTI_LINE_20;
	HAL_EXTI_GenerateSWI(&h);
	CHECK(peek(&EXTI->PR) == (1u << 20));
}

// ---- E10 ----

TEST_CASE_FIXTURE(ExtiFixture, "E10 a GPIO line configured rising becomes pending on a rising edge of the selected pin") {
	__HAL_RCC_GPIOC_CLK_ENABLE();
	hostGpioSetInput(2, 3, HOST_PIN_LOW);
	setLine(EXTI_LINE_3, EXTI_TRIGGER_RISING, EXTI_MODE_INTERRUPT, EXTI_GPIOC);
	REQUIRE(peek(&EXTI->PR) == 0u);
	hostGpioSetInput(2, 3, HOST_PIN_HIGH);
	CHECK(peek(&EXTI->PR) == (1u << 3)); // RM0090 12.2.4
}

TEST_CASE_FIXTURE(ExtiFixture, "E10 a GPIO line configured rising does not become pending on a falling edge") {
	__HAL_RCC_GPIOC_CLK_ENABLE();
	hostGpioSetInput(2, 3, HOST_PIN_HIGH);
	setLine(EXTI_LINE_3, EXTI_TRIGGER_RISING, EXTI_MODE_INTERRUPT, EXTI_GPIOC);
	hostGpioSetInput(2, 3, HOST_PIN_LOW);
	CHECK(peek(&EXTI->PR) == 0u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E10 a GPIO line does not become pending on an edge of the same pin number of another port") {
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	hostGpioSetInput(2, 3, HOST_PIN_LOW);
	hostGpioSetInput(3, 3, HOST_PIN_LOW);
	setLine(EXTI_LINE_3, EXTI_TRIGGER_RISING, EXTI_MODE_INTERRUPT, EXTI_GPIOC);
	hostGpioSetInput(3, 3, HOST_PIN_HIGH); // PD3: the line selects port C (RM0090 9.2.3)
	CHECK(peek(&EXTI->PR) == 0u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E10 a GPIO line in event mode counts an event and does not become pending on an edge") {
	__HAL_RCC_GPIOC_CLK_ENABLE();
	hostGpioSetInput(2, 3, HOST_PIN_LOW);
	setLine(EXTI_LINE_3, EXTI_TRIGGER_RISING, EXTI_MODE_EVENT, EXTI_GPIOC);
	const uint32_t before = hostExtiEventCount();
	hostGpioSetInput(2, 3, HOST_PIN_HIGH);
	CHECK(peek(&EXTI->PR) == 0u); // IMR bit clear (12.2.4)
	CHECK(hostExtiEventCount() == before + 1u);
}

TEST_CASE_FIXTURE(ExtiFixture, "E10 a line 16 to 22 becomes pending when its input rises") {
	setLine(EXTI_LINE_17, EXTI_TRIGGER_RISING, EXTI_MODE_INTERRUPT);
	hostExtiSetLine(17, 0);
	REQUIRE(peek(&EXTI->PR) == 0u);
	hostExtiSetLine(17, 1);
	CHECK(peek(&EXTI->PR) == (1u << 17));
}

TEST_CASE_FIXTURE(ExtiFixture, "E10 a line 16 to 22 configured falling becomes pending when its input falls and not when it rises") {
	setLine(EXTI_LINE_17, EXTI_TRIGGER_FALLING, EXTI_MODE_INTERRUPT);
	hostExtiSetLine(17, 0);
	hostExtiSetLine(17, 1);
	CHECK(peek(&EXTI->PR) == 0u);
	hostExtiSetLine(17, 0);
	CHECK(peek(&EXTI->PR) == (1u << 17));
}

} // TEST_SUITE
