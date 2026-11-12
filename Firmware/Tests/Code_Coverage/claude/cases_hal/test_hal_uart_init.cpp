/*
 * P12 T4b group UI: HAL_UART_Init, HAL_UART_DeInit, HAL_UART_GetState, HAL_UART_GetError and the baud rate generation of
 * UART_SetConfig in stm32f4xx_hal_uart.c against the UART register model (fakes/stm32_hal/HalUart.h). Requirements UI1-UI7 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T4B_REQUIREMENTS.md; compiled into ffb_hal_tests and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 chapter 70 (HAL UART Generic Driver: init / deinit, state and error names) and RM0090 Rev 22 chapter 30:
 *  - registers 30.6.1-30.6.8. CR1: OVER8 15, UE 13, M 12, WAKE 11, PCE 10, PS 9, PEIE 8, TXEIE 7, TCIE 6, RXNEIE 5, IDLEIE 4, TE 3,
 *    RE 2, RWU 1, SBK 0. CR2: LINEN 14, STOP 13:12 (00 = 1, 01 = 0.5, 10 = 2, 11 = 1.5 stop bits), CLKEN 11. CR3: CTSE 9, RTSE 8,
 *    SCEN 5, HDSEL 3, IREN 1. BRR: DIV_Mantissa 15:4, DIV_Fraction 3:0 (OVER8 = 1: bits 2:0, bit 3 kept cleared).
 *  - 30.3.4: baud = fPCLK / (8 * (2 - OVER8) * USARTDIV); USARTDIV = DIV_Mantissa + DIV_Fraction / (8 * (2 - OVER8)), so the
 *    fraction has 1/16 resolution (OVER8 = 0) or 1/8 (OVER8 = 1); DIV_Fraction is "the nearest real number" of
 *    fraction * 16 (or * 8); when it rounds up to 16 (or 8) the carry goes into the mantissa (RM0090 examples 2 and 3).
 *  - Tables 134-137 of RM0090 (error calculation): "Value programmed in the baud rate register" is USARTDIV. BRR = (mantissa << 4) |
 *    fraction, so for OVER8 = 0 the BRR equals USARTDIV * 16 and for OVER8 = 1 USARTDIV is stored with the fraction in 3 bits.
 *    The tables below are taken from those rows. A worked row: fPCLK = 8 MHz, 9600 baud, OVER8 = 0: USARTDIV = 8e6 / (16 * 9600) =
 *    52.0833; fraction 0.0833 * 16 = 1.33 -> 1; mantissa 52 = 0x34; BRR = 0x341 (table: 52.0625 = 52 + 1/16). Same row, OVER8 = 1:
 *    USARTDIV = 8e6 / (8 * 9600) = 104.1667; 0.1667 * 8 = 1.33 -> 1; mantissa 104 = 0x68; BRR = 0x681 (table: 104.125).
 *
 * PCLK: UART_SetConfig takes HAL_RCC_GetPCLK1Freq (USART2, USART3, UART4, UART5) or HAL_RCC_GetPCLK2Freq (USART1, USART6). RM0090 7.3.3:
 * HCLK = SystemCoreClock, PCLKx = HCLK / (1 .. 16) with RCC_CFGR PPRE1 (bits 12:10) and PPRE2 (bits 15:13): 0xx = /1, 100 = /2,
 * 101 = /4, 110 = /8, 111 = /16. The tests set SystemCoreClock and RCC_CFGR directly; settings used:
 *   S1: HCLK 16 MHz, PPRE1 = /2, PPRE2 = /1  -> PCLK1 = 8 MHz,  PCLK2 = 16 MHz
 *   S2: HCLK 48 MHz, PPRE1 = /2, PPRE2 = /4  -> PCLK1 = 24 MHz, PCLK2 = 12 MHz
 *   T : HCLK 168 MHz, PPRE1 = /4, PPRE2 = /2 -> PCLK1 = 42 MHz, PCLK2 = 84 MHz (the firmware's clock tree)
 *   U : HCLK 100 MHz, PPRE1 = /4, PPRE2 = /2 -> PCLK1 = 25 MHz, PCLK2 = 50 MHz
 *
 * UI7: USE_HAL_UART_REGISTER_CALLBACKS is 0U in both targets' stm32f4xx_hal_conf.h, the Register / UnRegister callback functions
 * are not compiled, there is nothing to test (reported to the lead).
 *
 * NEEDS REVIEW proposals (characterization cases, the lead assigns the numbers, proposals use NEEDS REVIEW 313):
 *  NEEDS REVIEW 313: HAL_UART_Init does not touch the CR1 bits outside M, PCE, PS, TE, RE, OVER8, UE (e.g. interrupt enables stay as they were).
 *  NEEDS REVIEW 313: stop bit codes 0.5 and 1.5 (raw STOP = 01 / 11) are written to CR2 as given; the HAL defines only UART_STOPBITS_1 / _2 (UM1725).
 *  (the rounding tie 12 MHz / 38400 baud is the KNOWN_ISSUE pair V102 (arguable add-on), RM0090 Tables 134 / 135 print 19.5625 / 39.125.)
 *  NEEDS REVIEW 313: HAL_UART_DeInit leaves BRR, CR1 (TE, RE, M, ...), CR2, CR3 as they were; only UE is cleared.
 *  NEEDS REVIEW 313: HAL_UART_DeInit unlocks the handle and works on a handle that was never initialised.
 *  NEEDS REVIEW 313: HAL_UART_Init on a handle in state RESET unlocks it (Lock = HAL_UNLOCKED).
 */
#include "HalFixture.h"
#include "HalUart.h"
#include "TestHelpers.h"
#include "UartTestSupport.h"

#include <cstdint>
#include <cstring>

extern "C" uint32_t SystemCoreClock;

namespace {

constexpr uint32_t CR1_OVER8 = 1u << 15, CR1_UE = 1u << 13, CR1_M = 1u << 12, CR1_PCE = 1u << 10, CR1_PS = 1u << 9;
constexpr uint32_t CR1_TXEIE = 1u << 7, CR1_RXNEIE = 1u << 5, CR1_IDLEIE = 1u << 4, CR1_TE = 1u << 3, CR1_RE = 1u << 2;
constexpr uint32_t CR2_LINEN = 1u << 14, CR2_STOP = 3u << 12, CR2_CLKEN = 1u << 11;
constexpr uint32_t CR3_CTSE = 1u << 9, CR3_RTSE = 1u << 8, CR3_SCEN = 1u << 5, CR3_HDSEL = 1u << 3, CR3_IREN = 1u << 1;

USART_TypeDef *uartInitInst(unsigned i) {
	static USART_TypeDef *const t[6] = {USART1, USART2, USART3, UART4, UART5, USART6};
	return t[i];
}
bool uartInitOnApb2(unsigned i) { return i == 0 || i == 5; }
const char *uartInitName(unsigned i) {
	static const char *const n[6] = {"USART1", "USART2", "USART3", "UART4", "UART5", "USART6"};
	return n[i];
}

/* Handle in state RESET with the given configuration. */
void uartInitMk(UART_HandleTypeDef &h, unsigned i, uint32_t baud = 115200u, uint32_t wl = UART_WORDLENGTH_8B,
				uint32_t stop = UART_STOPBITS_1, uint32_t par = UART_PARITY_NONE, uint32_t mode = UART_MODE_TX_RX,
				uint32_t hw = UART_HWCONTROL_NONE, uint32_t os = UART_OVERSAMPLING_16) {
	std::memset(&h, 0, sizeof h);
	h.Instance = uartInitInst(i);
	h.Init.BaudRate = baud;
	h.Init.WordLength = wl;
	h.Init.StopBits = stop;
	h.Init.Parity = par;
	h.Init.Mode = mode;
	h.Init.HwFlowCtl = hw;
	h.Init.OverSampling = os;
}

/* RM0090 7.3.3: HCLK and the APB prescaler codes (PPRE1 bits 12:10, PPRE2 bits 15:13). */
void uartInitClocks(uint32_t hclk, uint32_t ppre1, uint32_t ppre2) {
	SystemCoreClock = hclk;
	HalFixture::poke(&RCC->CFGR, (ppre1 << 10) | (ppre2 << 13));
}
void uartInitS1() { uartInitClocks(16000000u, 4u, 0u); }
void uartInitS2() { uartInitClocks(48000000u, 4u, 5u); }
void uartInitT() { uartInitClocks(168000000u, 5u, 4u); }
void uartInitU() { uartInitClocks(100000000u, 5u, 4u); }

uint32_t uartInitCr1(unsigned i) { return HalFixture::peek(&uartInitInst(i)->CR1); }
uint32_t uartInitCr2(unsigned i) { return HalFixture::peek(&uartInitInst(i)->CR2); }
uint32_t uartInitCr3(unsigned i) { return HalFixture::peek(&uartInitInst(i)->CR3); }
uint32_t uartInitBrr(unsigned i) { return HalFixture::peek(&uartInitInst(i)->BRR); }

struct UartInitRow {
	uint32_t baud;
	uint32_t brr;
};

/* Baud rate rows of RM0090 Tables 134-137 (USARTDIV converted to BRR as described in the header). The 12 MHz, 38400 baud row
 * (a rounding tie) is left out here: it is the KNOWN_ISSUE pair V102 (arguable add-on). */
const UartInitRow uartInitRows8_16[] = {{1200, 0x1A0B}, {2400, 0xD05}, {9600, 0x341}, {19200, 0x1A1}, {38400, 0xD0}, {57600, 0x8B}, {115200, 0x45}, {230400, 0x23}, {460800, 0x11}};
const UartInitRow uartInitRows12_16[] = {{1200, 0x2710}, {2400, 0x1388}, {9600, 0x4E2}, {19200, 0x271}, {57600, 0xD0}, {115200, 0x68}, {230400, 0x34}, {460800, 0x1A}};
const UartInitRow uartInitRows16_16[] = {{1200, 0x3415}, {2400, 0x1A0B}, {9600, 0x683}, {19200, 0x341}, {38400, 0x1A1}, {57600, 0x116}, {115200, 0x8B}, {230400, 0x45}, {460800, 0x23}, {921600, 0x11}};
const UartInitRow uartInitRows24_16[] = {{1200, 0x4E20}, {2400, 0x2710}, {9600, 0x9C4}, {19200, 0x4E2}, {38400, 0x271}, {57600, 0x1A1}, {115200, 0xD0}, {230400, 0x68}, {460800, 0x34}, {921600, 0x1A}};
const UartInitRow uartInitRows8_8[] = {{1200, 0x3413}, {2400, 0x1A05}, {9600, 0x681}, {19200, 0x341}, {38400, 0x1A0}, {57600, 0x113}, {115200, 0x85}, {230400, 0x43}, {460800, 0x21}, {921600, 0x11}};
const UartInitRow uartInitRows12_8[] = {{1200, 0x4E20}, {2400, 0x2710}, {9600, 0x9C2}, {19200, 0x4E1}, {57600, 0x1A0}, {115200, 0xD0}, {230400, 0x64}, {460800, 0x32}, {921600, 0x15}};
const UartInitRow uartInitRows16_8[] = {{1200, 0x6825}, {2400, 0x3413}, {9600, 0xD03}, {19200, 0x681}, {38400, 0x341}, {57600, 0x226}, {115200, 0x113}, {230400, 0x85}, {460800, 0x43}, {921600, 0x21}, {2000000, 0x10}};
const UartInitRow uartInitRows24_8[] = {{1200, 0x9C40}, {2400, 0x4E20}, {9600, 0x1384}, {19200, 0x9C2}, {38400, 0x4E1}, {57600, 0x341}, {115200, 0x1A0}, {230400, 0xD0}, {460800, 0x64}, {921600, 0x32}, {2000000, 0x14}, {3000000, 0x10}};

template <size_t N>
void uartInitCheckRows(unsigned inst, const UartInitRow (&rows)[N], uint32_t os) {
	for (size_t k = 0; k < N; ++k) {
		UART_HandleTypeDef h;
		uartInitMk(h, inst, rows[k].baud, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_NONE, os);
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		CAPTURE(uartInitName(inst));
		CAPTURE(rows[k].baud);
		CHECK(uartInitBrr(inst) == rows[k].brr);
	}
}

/* All six instances: the APB1 ones use rows1, the APB2 ones (USART1, USART6) rows2. */
template <size_t N1, size_t N2>
void uartInitCheckAll(const UartInitRow (&rows1)[N1], const UartInitRow (&rows2)[N2], uint32_t os) {
	for (unsigned i = 0; i < 6; ++i) {
		if (uartInitOnApb2(i)) uartInitCheckRows(i, rows2, os);
		else uartInitCheckRows(i, rows1, os);
	}
}

/* BRR of one Init on one instance under the current clocks. */
uint32_t uartInitBrrOf(unsigned inst, uint32_t baud, uint32_t os) {
	UART_HandleTypeDef h;
	uartInitMk(h, inst, baud, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_NONE, os);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	return uartInitBrr(inst);
}

} // namespace

/* ---------------------------------------------------------------- UI1 */

TEST_CASE_FIXTURE(UartFixture, "UI1 Init of an 8N1 transmit and receive configuration sets TE, RE and UE and nothing else in CR1, on all six instances") {
	for (unsigned i = 0; i < 6; ++i) {
		UART_HandleTypeDef h;
		uartInitMk(h, i);
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		CAPTURE(uartInitName(i));
		CHECK(uartInitCr1(i) == (CR1_UE | CR1_TE | CR1_RE));
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI1 mode transmit only sets TE and clears RE") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1, 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(uartInitCr1(1) == (CR1_UE | CR1_TE));
}

TEST_CASE_FIXTURE(UartFixture, "UI1 mode receive only sets RE and clears TE") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1, 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_RX);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(uartInitCr1(1) == (CR1_UE | CR1_RE));
}

TEST_CASE_FIXTURE(UartFixture, "UI1 a 9 bit word sets M") {
	UART_HandleTypeDef h;
	uartInitMk(h, 2, 9600, UART_WORDLENGTH_9B);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(uartInitCr1(2) == (CR1_UE | CR1_M | CR1_TE | CR1_RE));
}

TEST_CASE_FIXTURE(UartFixture, "UI1 even parity sets PCE and leaves PS clear") {
	UART_HandleTypeDef h;
	uartInitMk(h, 2, 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_EVEN);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(uartInitCr1(2) == (CR1_UE | CR1_PCE | CR1_TE | CR1_RE)); // RM0090 30.6.4: PS 0 = even
}

TEST_CASE_FIXTURE(UartFixture, "UI1 odd parity sets PCE and PS") {
	UART_HandleTypeDef h;
	uartInitMk(h, 2, 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_ODD);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(uartInitCr1(2) == (CR1_UE | CR1_PCE | CR1_PS | CR1_TE | CR1_RE));
}

TEST_CASE_FIXTURE(UartFixture, "UI1 oversampling by 8 sets OVER8") {
	UART_HandleTypeDef h;
	uartInitMk(h, 0, 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_NONE, UART_OVERSAMPLING_8);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(uartInitCr1(0) == (CR1_UE | CR1_OVER8 | CR1_TE | CR1_RE));
}

TEST_CASE_FIXTURE(UartFixture, "UI1 oversampling by 16 leaves OVER8 clear") {
	UART_HandleTypeDef h;
	uartInitMk(h, 0, 9600);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK((uartInitCr1(0) & CR1_OVER8) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UI1 one stop bit writes STOP = 00") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1, 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK((uartInitCr2(1) & CR2_STOP) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UI1 two stop bits write STOP = 10") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1, 9600, UART_WORDLENGTH_8B, UART_STOPBITS_2);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK((uartInitCr2(1) & CR2_STOP) == (2u << 12));
}

TEST_CASE_FIXTURE(UartFixture, "characterization: UI1 the raw stop bit codes 0.5 and 1.5 are written to STOP as 01 and 11 (NEEDS REVIEW 313)") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1, 9600, UART_WORDLENGTH_8B, 1u << 12);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK((uartInitCr2(1) & CR2_STOP) == (1u << 12));
	uartInitMk(h, 1, 9600, UART_WORDLENGTH_8B, 3u << 12);
	REQUIRE(HAL_UART_DeInit(&h) == HAL_OK);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK((uartInitCr2(1) & CR2_STOP) == (3u << 12));
}

TEST_CASE_FIXTURE(UartFixture, "UI1 a changed stop bit setting replaces the earlier one") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1, 9600, UART_WORDLENGTH_8B, UART_STOPBITS_2);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	h.Init.StopBits = UART_STOPBITS_1;
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK((uartInitCr2(1) & CR2_STOP) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UI1 Init clears LINEN and CLKEN of CR2") {
	for (unsigned i = 0; i < 6; ++i) {
		if (i == 3 || i == 4) continue; // UART4 / UART5 have no CLKEN (see UI3)
		USART_TypeDef *u = uartInitInst(i);
		u->CR2 = CR2_LINEN | CR2_CLKEN; // UE = 0 after reset
		REQUIRE((uartInitCr2(i) & (CR2_LINEN | CR2_CLKEN)) == (CR2_LINEN | CR2_CLKEN));
		UART_HandleTypeDef h;
		uartInitMk(h, i);
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		CAPTURE(uartInitName(i));
		CHECK((uartInitCr2(i) & (CR2_LINEN | CR2_CLKEN)) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI1 Init clears LINEN of CR2 on UART4 and UART5") {
	for (unsigned i = 3; i < 5; ++i) {
		uartInitInst(i)->CR2 = CR2_LINEN;
		REQUIRE((uartInitCr2(i) & CR2_LINEN) != 0u);
		UART_HandleTypeDef h;
		uartInitMk(h, i);
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		CAPTURE(uartInitName(i));
		CHECK((uartInitCr2(i) & CR2_LINEN) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI1 Init clears HDSEL and IREN of CR3 on every instance") {
	for (unsigned i = 0; i < 6; ++i) {
		uartInitInst(i)->CR3 = CR3_HDSEL | CR3_IREN;
		REQUIRE((uartInitCr3(i) & (CR3_HDSEL | CR3_IREN)) == (CR3_HDSEL | CR3_IREN));
		UART_HandleTypeDef h;
		uartInitMk(h, i);
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		CAPTURE(uartInitName(i));
		CHECK((uartInitCr3(i) & (CR3_HDSEL | CR3_IREN)) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI1 Init clears SCEN of CR3 on USART1, USART2, USART3 and USART6") {
	const unsigned list[4] = {0, 1, 2, 5};
	for (unsigned k = 0; k < 4; ++k) {
		const unsigned i = list[k];
		uartInitInst(i)->CR3 = CR3_SCEN;
		REQUIRE((uartInitCr3(i) & CR3_SCEN) != 0u);
		UART_HandleTypeDef h;
		uartInitMk(h, i);
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		CAPTURE(uartInitName(i));
		CHECK((uartInitCr3(i) & CR3_SCEN) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI1 hardware flow control none leaves CTSE and RTSE clear") {
	UART_HandleTypeDef h;
	uartInitMk(h, 0);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK((uartInitCr3(0) & (CR3_CTSE | CR3_RTSE)) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UI1 hardware flow control RTS sets RTSE only") {
	UART_HandleTypeDef h;
	uartInitMk(h, 0, 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_RTS);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK((uartInitCr3(0) & (CR3_CTSE | CR3_RTSE)) == CR3_RTSE);
}

TEST_CASE_FIXTURE(UartFixture, "UI1 hardware flow control CTS sets CTSE only") {
	UART_HandleTypeDef h;
	uartInitMk(h, 0, 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_CTS);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK((uartInitCr3(0) & (CR3_CTSE | CR3_RTSE)) == CR3_CTSE);
}

TEST_CASE_FIXTURE(UartFixture, "UI1 hardware flow control RTS and CTS sets both on USART1, USART2, USART3 and USART6") {
	const unsigned list[4] = {0, 1, 2, 5};
	for (unsigned k = 0; k < 4; ++k) {
		UART_HandleTypeDef h;
		uartInitMk(h, list[k], 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_RTS_CTS);
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		CAPTURE(uartInitName(list[k]));
		CHECK((uartInitCr3(list[k]) & (CR3_CTSE | CR3_RTSE)) == (CR3_CTSE | CR3_RTSE));
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI1 a later Init without flow control clears CTSE and RTSE") {
	UART_HandleTypeDef h;
	uartInitMk(h, 0, 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_RTS_CTS);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	h.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK((uartInitCr3(0) & (CR3_CTSE | CR3_RTSE)) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UI1 a later Init replaces the word length, parity and mode bits of the first") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1, 9600, UART_WORDLENGTH_9B, UART_STOPBITS_1, UART_PARITY_ODD, UART_MODE_TX, UART_HWCONTROL_NONE, UART_OVERSAMPLING_8);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	REQUIRE(uartInitCr1(1) == (CR1_UE | CR1_OVER8 | CR1_M | CR1_PCE | CR1_PS | CR1_TE));
	h.Init.WordLength = UART_WORDLENGTH_8B;
	h.Init.Parity = UART_PARITY_NONE;
	h.Init.Mode = UART_MODE_RX;
	h.Init.OverSampling = UART_OVERSAMPLING_16;
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(uartInitCr1(1) == (CR1_UE | CR1_RE));
}

TEST_CASE_FIXTURE(UartFixture, "UI1 Init leaves the state READY for both the transmitter and the receiver") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(h.gState == HAL_UART_STATE_READY);
	CHECK(h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UI1 Init sets the error code to NONE") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	h.ErrorCode = HAL_UART_ERROR_ORE | HAL_UART_ERROR_DMA; // stale value in a not yet initialised handle
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartFixture, "UI1 Init calls MspInit exactly once") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(uartCounts.mspInit == 1u);
	CHECK(uartCounts.last == &h);
}

TEST_CASE_FIXTURE(UartFixture, "UI1 Init never changes CR1, CR2 or BRR while UE is 1, on the first Init of every instance") {
	for (unsigned i = 0; i < 6; ++i) {
		UART_HandleTypeDef h;
		uartInitMk(h, i, 9600, UART_WORDLENGTH_9B, UART_STOPBITS_2, UART_PARITY_ODD, UART_MODE_TX_RX, UART_HWCONTROL_NONE, UART_OVERSAMPLING_8);
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		CAPTURE(uartInitName(i));
		CHECK(hostUartConfigWhileEnabled(i) == 0u);
		CHECK(hostUartDisabledBusy(i) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI1 a second Init with a different configuration changes CR1, CR2 and BRR only with UE = 0") {
	UART_HandleTypeDef h;
	uartInitMk(h, 2, 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	REQUIRE((uartInitCr1(2) & CR1_UE) != 0u); // the first Init left the USART enabled
	h.Init.BaudRate = 115200u;
	h.Init.WordLength = UART_WORDLENGTH_9B;
	h.Init.StopBits = UART_STOPBITS_2;
	h.Init.Parity = UART_PARITY_EVEN;
	h.Init.OverSampling = UART_OVERSAMPLING_8;
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(hostUartConfigWhileEnabled(2) == 0u);
	CHECK((uartInitCr1(2) & CR1_UE) != 0u);
}

TEST_CASE_FIXTURE(UartFixture, "characterization: UI1 Init does not touch the CR1 bits outside M, PCE, PS, TE, RE, OVER8 and UE (NEEDS REVIEW 313)") {
	uartInitInst(1)->CR1 = CR1_IDLEIE | CR1_RXNEIE | CR1_TXEIE;
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(uartInitCr1(1) == (CR1_UE | CR1_TE | CR1_RE | CR1_IDLEIE | CR1_RXNEIE | CR1_TXEIE));
}

/* ---------------------------------------------------------------- UI2: baud rate */

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR with OVER8 = 0 at PCLK1 = 8 MHz and PCLK2 = 16 MHz equals RM0090 Tables 134 and 136 on all six instances") {
	uartInitS1();
	uartInitCheckAll(uartInitRows8_16, uartInitRows16_16, UART_OVERSAMPLING_16);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR with OVER8 = 1 at PCLK1 = 8 MHz and PCLK2 = 16 MHz equals RM0090 Tables 135 and 137 on all six instances") {
	uartInitS1();
	uartInitCheckAll(uartInitRows8_8, uartInitRows16_8, UART_OVERSAMPLING_8);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR with OVER8 = 0 at PCLK1 = 24 MHz and PCLK2 = 12 MHz equals RM0090 Tables 136 and 133 on all six instances") {
	uartInitS2();
	uartInitCheckAll(uartInitRows24_16, uartInitRows12_16, UART_OVERSAMPLING_16);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR with OVER8 = 1 at PCLK1 = 24 MHz and PCLK2 = 12 MHz equals RM0090 Tables 137 and 135 on all six instances") {
	uartInitS2();
	uartInitCheckAll(uartInitRows24_8, uartInitRows12_8, UART_OVERSAMPLING_8);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 the baud rate of an APB2 instance follows PCLK2 and of an APB1 instance PCLK1, never the other bus") {
	uartInitS1(); // PCLK1 = 8 MHz, PCLK2 = 16 MHz; 9600 baud, OVER8 = 0: 8 MHz -> 0x341 (52.0625), 16 MHz -> 0x683 (104.1875)
	CHECK(uartInitBrrOf(0, 9600, UART_OVERSAMPLING_16) == 0x683u); // USART1
	CHECK(uartInitBrrOf(5, 9600, UART_OVERSAMPLING_16) == 0x683u); // USART6
	CHECK(uartInitBrrOf(1, 9600, UART_OVERSAMPLING_16) == 0x341u); // USART2
	CHECK(uartInitBrrOf(2, 9600, UART_OVERSAMPLING_16) == 0x341u); // USART3
	CHECK(uartInitBrrOf(3, 9600, UART_OVERSAMPLING_16) == 0x341u); // UART4
	CHECK(uartInitBrrOf(4, 9600, UART_OVERSAMPLING_16) == 0x341u); // UART5
}

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR at the firmware clock tree (PCLK1 = 42 MHz, PCLK2 = 84 MHz) for 9600 baud, OVER8 = 0") {
	uartInitT();
	// 42e6 / (16 * 9600) = 273.4375: fraction 0.4375 * 16 = 7 exactly; mantissa 273 = 0x111 -> 0x1117
	// 84e6 / (16 * 9600) = 546.875: fraction 0.875 * 16 = 14; mantissa 546 = 0x222 -> 0x222E
	CHECK(uartInitBrrOf(1, 9600, UART_OVERSAMPLING_16) == 0x1117u);
	CHECK(uartInitBrrOf(4, 9600, UART_OVERSAMPLING_16) == 0x1117u);
	CHECK(uartInitBrrOf(0, 9600, UART_OVERSAMPLING_16) == 0x222Eu);
	CHECK(uartInitBrrOf(5, 9600, UART_OVERSAMPLING_16) == 0x222Eu);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR at the firmware clock tree for 9600 baud, OVER8 = 1") {
	uartInitT();
	// 42e6 / (8 * 9600) = 546.875: fraction 0.875 * 8 = 7 exactly; mantissa 546 = 0x222 -> 0x2227
	// 84e6 / (8 * 9600) = 1093.75: fraction 0.75 * 8 = 6; mantissa 1093 = 0x445 -> 0x4456
	CHECK(uartInitBrrOf(2, 9600, UART_OVERSAMPLING_8) == 0x2227u);
	CHECK(uartInitBrrOf(3, 9600, UART_OVERSAMPLING_8) == 0x2227u);
	CHECK(uartInitBrrOf(0, 9600, UART_OVERSAMPLING_8) == 0x4456u);
	CHECK(uartInitBrrOf(5, 9600, UART_OVERSAMPLING_8) == 0x4456u);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR for 921600 baud on PCLK2 = 84 MHz, OVER8 = 0 and 1") {
	uartInitT();
	// OVER8 = 0: 84e6 / (16 * 921600) = 5.69661; 0.69661 * 16 = 11.15 -> 11; mantissa 5 -> 0x5B
	// OVER8 = 1: 84e6 / (8 * 921600) = 11.39323; 0.39323 * 8 = 3.15 -> 3; mantissa 11 = 0xB -> 0xB3
	CHECK(uartInitBrrOf(0, 921600, UART_OVERSAMPLING_16) == 0x5Bu);
	CHECK(uartInitBrrOf(5, 921600, UART_OVERSAMPLING_8) == 0xB3u);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR for 115200 baud on PCLK2 = 84 MHz, OVER8 = 1") {
	uartInitT();
	// 84e6 / (8 * 115200) = 91.14583; 0.14583 * 8 = 1.17 -> 1; mantissa 91 = 0x5B -> 0x5B1
	CHECK(uartInitBrrOf(0, 115200, UART_OVERSAMPLING_8) == 0x5B1u);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 a fraction that rounds up to 16 carries into the mantissa on PCLK2 = 84 MHz, OVER8 = 0") {
	uartInitT();
	// 40700 baud: USARTDIV = 84e6 / (16 * 40700) = 128.99263; 0.99263 * 16 = 15.88 -> 16 = overflow, carry: mantissa 129 = 0x81,
	// fraction 0 -> BRR = 0x810 (RM0090 30.3.4 example 3: 50.99 -> 0x330)
	CHECK(uartInitBrrOf(0, 40700, UART_OVERSAMPLING_16) == 0x810u);
	CHECK(uartInitBrrOf(5, 40700, UART_OVERSAMPLING_16) == 0x810u);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 a fraction that rounds up to 8 carries into the mantissa on PCLK2 = 84 MHz, OVER8 = 1") {
	uartInitT();
	// 19700 baud: USARTDIV = 84e6 / (8 * 19700) = 532.99492; 0.99492 * 8 = 7.96 -> 8 = overflow, carry: mantissa 533 = 0x215,
	// fraction 0 -> BRR = 0x2150 (RM0090 30.3.4 OVER8 = 1 example 3: 50.99 -> 0x0330)
	CHECK(uartInitBrrOf(0, 19700, UART_OVERSAMPLING_8) == 0x2150u);
	CHECK(uartInitBrrOf(5, 19700, UART_OVERSAMPLING_8) == 0x2150u);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 a fraction that rounds up to 16 carries into the mantissa on PCLK1 = 42 MHz, OVER8 = 0") {
	uartInitT();
	// 44500 baud: USARTDIV = 42e6 / (16 * 44500) = 58.98876; 0.98876 * 16 = 15.82 -> 16, carry: mantissa 59 = 0x3B -> 0x3B0
	for (unsigned i = 1; i < 5; ++i) {
		CAPTURE(uartInitName(i));
		CHECK(uartInitBrrOf(i, 44500, UART_OVERSAMPLING_16) == 0x3B0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI2 a fraction that rounds up to 8 carries into the mantissa on PCLK1 = 42 MHz, OVER8 = 1") {
	uartInitT();
	// 44500 baud: USARTDIV = 42e6 / (8 * 44500) = 117.97753; 0.97753 * 8 = 7.82 -> 8, carry: mantissa 118 = 0x76 -> 0x760
	for (unsigned i = 1; i < 5; ++i) {
		CAPTURE(uartInitName(i));
		CHECK(uartInitBrrOf(i, 44500, UART_OVERSAMPLING_8) == 0x760u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI2 with OVER8 = 1 bit 3 of BRR stays clear for every baud rate and instance") {
	uartInitT();
	const uint32_t bauds[6] = {9600u, 19700u, 44500u, 115200u, 460800u, 921600u};
	for (unsigned i = 0; i < 6; ++i) {
		for (unsigned k = 0; k < 6; ++k) {
			CAPTURE(uartInitName(i));
			CAPTURE(bauds[k]);
			CHECK((uartInitBrrOf(i, bauds[k], UART_OVERSAMPLING_8) & 8u) == 0u); // RM0090 30.6.3: DIV_Fraction3 must be kept cleared
		}
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR of the same baud rate follows a change of the APB prescaler between two Inits") {
	uartInitClocks(48000000u, 0u, 0u); // PCLK1 = 48 MHz: 9600 baud, OVER8 = 0: 48e6 / (16 * 9600) = 312.5 -> 0x1388
	CHECK(uartInitBrrOf(1, 9600, UART_OVERSAMPLING_16) == 0x1388u);
	uartInitClocks(48000000u, 4u, 0u); // PCLK1 = 24 MHz: 156.25 -> 156 = 0x9C, fraction 4 -> 0x9C4
	CHECK(uartInitBrrOf(1, 9600, UART_OVERSAMPLING_16) == 0x9C4u);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR of the highest APB prescaler /16 divides HCLK by 16") {
	uartInitClocks(128000000u, 7u, 7u); // PCLK1 = PCLK2 = 8 MHz; 9600 baud, OVER8 = 0: 0x341 (table row above)
	CHECK(uartInitBrrOf(1, 9600, UART_OVERSAMPLING_16) == 0x341u);
	CHECK(uartInitBrrOf(0, 9600, UART_OVERSAMPLING_16) == 0x341u);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 Init writes BRR with UE = 0") {
	uartInitS1();
	UART_HandleTypeDef h;
	uartInitMk(h, 1, 9600);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	h.Init.BaudRate = 19200u;
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(hostUartConfigWhileEnabled(1) == 0u);
	CHECK(uartInitBrr(1) == 0x1A1u); // 8 MHz, 19200: 26.0625 (Table 134 row 4)
}

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR for 38400 baud on PCLK2 = 12 MHz, OVER8 = 0 is the value RM0090 Table 134 prints" *
						   KNOWN_ISSUE("V102 (arguable add-on) rounding tie: 12 MHz / 38400 baud, OVER8 = 0 is USARTDIV 19.53125, fraction exactly 8.5; RM0090 Table 134 prints 19.5625 (BRR 0x139), the HAL writes 0x138 (UART_DIVFRAQ_SAMPLING16 truncates to hundredths)")) {
	uartInitS2(); // PCLK2 = 12 MHz
	// 12e6 / (16 * 38400) = 19.53125; 0.53125 * 16 = 8.5; the table prints 19.5625 = 19 + 9/16 -> mantissa 0x13, fraction 9 -> 0x139
	CHECK(uartInitBrrOf(0, 38400, UART_OVERSAMPLING_16) == 0x139u);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR for 38400 baud on PCLK2 = 12 MHz, OVER8 = 1 is the value RM0090 Table 135 prints" *
						   KNOWN_ISSUE("V102 (arguable add-on) rounding tie: 12 MHz / 38400 baud, OVER8 = 1 is USARTDIV 39.0625, fraction exactly 0.5; RM0090 Table 135 prints 39.125 (BRR 0x271), the HAL writes 0x270 (UART_DIVFRAQ_SAMPLING8 truncates to hundredths)")) {
	uartInitS2(); // PCLK2 = 12 MHz
	// 12e6 / (8 * 38400) = 39.0625; 0.0625 * 8 = 0.5; the table prints 39.125 = 39 + 1/8 -> mantissa 0x27, fraction 1 -> 0x271
	CHECK(uartInitBrrOf(5, 38400, UART_OVERSAMPLING_8) == 0x271u);
}

TEST_CASE_FIXTURE(UartFixture, "UI4 GetRxEventType after Init is the transfer complete type") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(HAL_UARTEx_GetRxEventType(&h) == HAL_UART_RXEVENT_TC);
}

TEST_CASE_FIXTURE(UartFixture, "characterization: UI5 after DeInit the Rx event type is the transfer complete type and the reception type standard (NEEDS REVIEW 313)") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	REQUIRE(HAL_UART_DeInit(&h) == HAL_OK);
	CHECK(HAL_UARTEx_GetRxEventType(&h) == HAL_UART_RXEVENT_TC);
	CHECK(h.ReceptionType == HAL_UART_RECEPTION_STANDARD);
}

/* The next four cases: RM0090 30.3.4 ("DIV_Fraction = the nearest real number") demands the fraction rounded from the exact
 * USARTDIV. The HAL truncates USARTDIV to hundredths before it rounds the fraction (UART_DIVFRAQ_SAMPLING16 / 8 in
 * stm32f4xx_hal_uart.h), so USARTDIV 22.78646 becomes 22.78 and its fraction 0.78 * 16 = 12.48 is rounded to 12 although the exact
 * 12.58 is nearest to 13. */
TEST_CASE_FIXTURE(UartFixture, "UI2 BRR for 115200 baud on PCLK1 = 42 MHz, OVER8 = 0 is the nearest fraction" *
						   KNOWN_ISSUE("V102 UART_BRR_SAMPLING16 truncates USARTDIV to 1/100 before rounding the fraction: 42 MHz / 115200 baud gives 22.78 -> fraction 12 (0x16C) while RM0090 30.3.4 asks for the nearest fraction, 13 (0x16D)")) {
	uartInitT();
	// 42e6 / (16 * 115200) = 22.78646; 0.78646 * 16 = 12.58 -> nearest is 13; mantissa 22 = 0x16 -> 0x16D
	CHECK(uartInitBrrOf(1, 115200, UART_OVERSAMPLING_16) == 0x16Du);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR for 230400 baud on PCLK2 = 84 MHz, OVER8 = 0 is the nearest fraction" *
						   KNOWN_ISSUE("V102 same truncation: 84 MHz / 230400 baud gives USARTDIV 22.78646, the HAL writes fraction 12 (0x16C) instead of the nearest 13 (0x16D)")) {
	uartInitT();
	// 84e6 / (16 * 230400) = 22.78646; fraction 12.58 -> 13 -> 0x16D
	CHECK(uartInitBrrOf(0, 230400, UART_OVERSAMPLING_16) == 0x16Du);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR for 921600 baud on PCLK1 = 42 MHz, OVER8 = 0 is the nearest fraction" *
						   KNOWN_ISSUE("V102 same truncation: 42 MHz / 921600 baud gives USARTDIV 2.84830, the HAL writes fraction 13 (0x2D) instead of the nearest 14 (0x2E)")) {
	uartInitT();
	// 42e6 / (16 * 921600) = 2.84830; 0.84830 * 16 = 13.57 -> 14 -> 0x2E
	CHECK(uartInitBrrOf(4, 921600, UART_OVERSAMPLING_16) == 0x2Eu);
}

TEST_CASE_FIXTURE(UartFixture, "UI2 BRR for 460800 baud on PCLK2 = 50 MHz, OVER8 = 1 is the nearest fraction" *
						   KNOWN_ISSUE("V102 same truncation with OVER8 = 1 (UART_BRR_SAMPLING8): 50 MHz / 460800 baud gives USARTDIV 13.56 -> fraction 4 (0xD4) while the nearest 1/8 fraction is 5 (0xD5)")) {
	uartInitU();
	// 50e6 / (8 * 460800) = 13.56337; 0.56337 * 8 = 4.507 -> 5; mantissa 13 = 0xD -> 0xD5
	CHECK(uartInitBrrOf(5, 460800, UART_OVERSAMPLING_8) == 0xD5u);
}

/* ---------------------------------------------------------------- UI3 */

TEST_CASE_FIXTURE(UartFixture, "UI3 Init of one instance leaves every register of the other five unchanged") {
	for (unsigned i = 0; i < 6; ++i) {
		uint32_t before[6][7];
		for (unsigned o = 0; o < 6; ++o) {
			USART_TypeDef *u = uartInitInst(o);
			const volatile uint32_t *r[7] = {&u->SR, &u->DR, &u->BRR, &u->CR1, &u->CR2, &u->CR3, &u->GTPR};
			for (unsigned k = 0; k < 7; ++k) before[o][k] = HalFixture::peek(r[k]);
		}
		UART_HandleTypeDef h;
		uartInitMk(h, i, 9600, UART_WORDLENGTH_9B, UART_STOPBITS_2, UART_PARITY_EVEN, UART_MODE_TX_RX, UART_HWCONTROL_NONE, UART_OVERSAMPLING_8);
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		for (unsigned o = 0; o < 6; ++o) {
			if (o == i) continue;
			USART_TypeDef *u = uartInitInst(o);
			const volatile uint32_t *r[7] = {&u->SR, &u->DR, &u->BRR, &u->CR1, &u->CR2, &u->CR3, &u->GTPR};
			for (unsigned k = 0; k < 7; ++k) {
				CAPTURE(uartInitName(i));
				CAPTURE(uartInitName(o));
				CAPTURE(k);
				CHECK(HalFixture::peek(r[k]) == before[o][k]);
			}
		}
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI3 Init of an instance changes its own CR1 and BRR") {
	// A check that the test above does not pass because Init does nothing: every instance gets its own CR1 / BRR written.
	for (unsigned i = 0; i < 6; ++i) {
		UART_HandleTypeDef h;
		uartInitMk(h, i, 9600);
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		CAPTURE(uartInitName(i));
		CHECK(uartInitCr1(i) == (CR1_UE | CR1_TE | CR1_RE));
		CHECK(uartInitBrr(i) != 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI3 UART4 and UART5 keep CTSE and RTSE at 0 when Init is asked for hardware flow control") {
	for (unsigned i = 3; i < 5; ++i) {
		UART_HandleTypeDef h;
		uartInitMk(h, i, 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_RTS_CTS);
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		CAPTURE(uartInitName(i));
		CHECK((uartInitCr3(i) & (CR3_CTSE | CR3_RTSE)) == 0u); // RM0090 30.6.6: CTSE / RTSE not available on UART4 / UART5
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI3 UART4 and UART5 have no CLKEN bit") {
	for (unsigned i = 3; i < 5; ++i) {
		uartInitInst(i)->CR2 = CR2_CLKEN;
		CAPTURE(uartInitName(i));
		CHECK((uartInitCr2(i) & CR2_CLKEN) == 0u); // RM0090 30.6.5: not available on UART4 / UART5
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI3 USART1, USART2, USART3 and USART6 have the CLKEN bit") {
	const unsigned list[4] = {0, 1, 2, 5};
	for (unsigned k = 0; k < 4; ++k) {
		uartInitInst(list[k])->CR2 = CR2_CLKEN;
		CAPTURE(uartInitName(list[k]));
		CHECK((uartInitCr2(list[k]) & CR2_CLKEN) != 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI3 Init of an instance writes the registers at the instance's own RM0090 Table 1 base address") {
	const uintptr_t bases[6] = {0x40011000u, 0x40004400u, 0x40004800u, 0x40004C00u, 0x40005000u, 0x40011400u};
	for (unsigned i = 0; i < 6; ++i) {
		UART_HandleTypeDef h;
		uartInitMk(h, i);
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		CAPTURE(uartInitName(i));
		CHECK(reinterpret_cast<uintptr_t>(h.Instance) == bases[i]);
		CHECK(HalFixture::peek(reinterpret_cast<volatile void *>(bases[i] + 0x0Cu)) == (CR1_UE | CR1_TE | CR1_RE));
	}
}

/* ---------------------------------------------------------------- UI4 */

TEST_CASE_FIXTURE(UartFixture, "UI4 Init with a NULL handle returns HAL_ERROR") {
	CHECK(HAL_UART_Init(nullptr) == HAL_ERROR);
}

TEST_CASE_FIXTURE(UartFixture, "UI4 Init with a NULL handle does not call MspInit") {
	REQUIRE(HAL_UART_Init(nullptr) == HAL_ERROR);
	CHECK(uartCounts.mspInit == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UI4 the first Init of a handle in state RESET calls MspInit") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(h.gState == HAL_UART_STATE_RESET);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(uartCounts.mspInit == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UI4 a repeated Init of a handle in state READY does not call MspInit again") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(uartCounts.mspInit == 1u);
}

TEST_CASE_FIXTURE(UartFixture, "UI4 Init after DeInit calls MspInit again") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	REQUIRE(HAL_UART_DeInit(&h) == HAL_OK);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(uartCounts.mspInit == 2u);
}

TEST_CASE_FIXTURE(UartFixture, "UI4 a repeated Init returns HAL_OK, applies the new baud rate and keeps the state READY") {
	uartInitS1();
	UART_HandleTypeDef h;
	uartInitMk(h, 1, 9600);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	REQUIRE(uartInitBrr(1) == 0x341u); // 8 MHz, 9600 baud (Table 134 row 3)
	h.Init.BaudRate = 19200u;
	CHECK(HAL_UART_Init(&h) == HAL_OK);
	CHECK(uartInitBrr(1) == 0x1A1u); // 26.0625
	CHECK(h.gState == HAL_UART_STATE_READY);
	CHECK(h.RxState == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UI4 a 9 bit word without parity sets M and leaves PCE clear") {
	UART_HandleTypeDef h;
	uartInitMk(h, 2, 9600, UART_WORDLENGTH_9B, UART_STOPBITS_1, UART_PARITY_NONE);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK((uartInitCr1(2) & (CR1_M | CR1_PCE | CR1_PS)) == CR1_M);
}

TEST_CASE_FIXTURE(UartFixture, "UI4 a 9 bit word with even parity sets M and PCE") {
	UART_HandleTypeDef h;
	uartInitMk(h, 2, 9600, UART_WORDLENGTH_9B, UART_STOPBITS_1, UART_PARITY_EVEN);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK((uartInitCr1(2) & (CR1_M | CR1_PCE | CR1_PS)) == (CR1_M | CR1_PCE));
}

TEST_CASE_FIXTURE(UartFixture, "UI4 a 9 bit word with odd parity sets M, PCE and PS") {
	UART_HandleTypeDef h;
	uartInitMk(h, 2, 9600, UART_WORDLENGTH_9B, UART_STOPBITS_1, UART_PARITY_ODD);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK((uartInitCr1(2) & (CR1_M | CR1_PCE | CR1_PS)) == (CR1_M | CR1_PCE | CR1_PS));
}

TEST_CASE_FIXTURE(UartFixture, "UI4 an 8 bit word with parity leaves M clear") {
	UART_HandleTypeDef h;
	uartInitMk(h, 2, 9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_ODD);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK((uartInitCr1(2) & CR1_M) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "characterization: UI4 Init of a handle in state RESET unlocks it (NEEDS REVIEW 313)") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	h.Lock = HAL_LOCKED;
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
}

/* ---------------------------------------------------------------- UI5 */

TEST_CASE_FIXTURE(UartFixture, "UI5 DeInit clears UE on every instance") {
	for (unsigned i = 0; i < 6; ++i) {
		UART_HandleTypeDef h;
		uartInitMk(h, i);
		REQUIRE(HAL_UART_Init(&h) == HAL_OK);
		REQUIRE((uartInitCr1(i) & CR1_UE) != 0u);
		CAPTURE(uartInitName(i));
		CHECK(HAL_UART_DeInit(&h) == HAL_OK);
		CHECK((uartInitCr1(i) & CR1_UE) == 0u);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI5 DeInit sets the transmitter and the receiver state to RESET") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	REQUIRE(HAL_UART_DeInit(&h) == HAL_OK);
	CHECK(h.gState == HAL_UART_STATE_RESET);
	CHECK(h.RxState == HAL_UART_STATE_RESET);
}

TEST_CASE_FIXTURE(UartFixture, "UI5 DeInit sets the error code to NONE") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	h.ErrorCode = HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE;
	REQUIRE(HAL_UART_DeInit(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartFixture, "UI5 DeInit calls MspDeInit exactly once and not MspInit") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	REQUIRE(HAL_UART_DeInit(&h) == HAL_OK);
	CHECK(uartCounts.mspDeInit == 1u);
	CHECK(uartCounts.mspInit == 1u);
	CHECK(uartCounts.last == &h);
}

TEST_CASE_FIXTURE(UartFixture, "UI5 DeInit with a NULL handle returns HAL_ERROR") {
	CHECK(HAL_UART_DeInit(nullptr) == HAL_ERROR);
}

TEST_CASE_FIXTURE(UartFixture, "UI5 DeInit with a NULL handle does not call MspDeInit") {
	REQUIRE(HAL_UART_DeInit(nullptr) == HAL_ERROR);
	CHECK(uartCounts.mspDeInit == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UI5 DeInit of an idle USART does not disable it while a frame is in flight") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	REQUIRE(HAL_UART_DeInit(&h) == HAL_OK);
	CHECK(hostUartDisabledBusy(1) == 0u);
}

TEST_CASE_FIXTURE(UartFixture, "UI5 a handle can be initialised again after DeInit and is READY") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	REQUIRE(HAL_UART_DeInit(&h) == HAL_OK);
	CHECK(HAL_UART_Init(&h) == HAL_OK);
	CHECK(h.gState == HAL_UART_STATE_READY);
	CHECK(h.RxState == HAL_UART_STATE_READY);
	CHECK((uartInitCr1(1) & CR1_UE) != 0u);
}

TEST_CASE_FIXTURE(UartFixture, "characterization: UI5 DeInit clears UE only and leaves BRR, CR1, CR2 and CR3 as they were (NEEDS REVIEW 313)") {
	uartInitS1();
	UART_HandleTypeDef h;
	uartInitMk(h, 1, 9600, UART_WORDLENGTH_9B, UART_STOPBITS_2, UART_PARITY_ODD);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	const uint32_t cr1 = uartInitCr1(1), cr2 = uartInitCr2(1), cr3 = uartInitCr3(1), brr = uartInitBrr(1);
	REQUIRE(HAL_UART_DeInit(&h) == HAL_OK);
	CHECK(uartInitCr1(1) == (cr1 & ~CR1_UE));
	CHECK(uartInitCr2(1) == cr2);
	CHECK(uartInitCr3(1) == cr3);
	CHECK(uartInitBrr(1) == brr);
}

TEST_CASE_FIXTURE(UartFixture, "characterization: UI5 DeInit unlocks the handle and works on a handle that was never initialised (NEEDS REVIEW 313)") {
	UART_HandleTypeDef h;
	uartInitMk(h, 2);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_UART_DeInit(&h) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(uartCounts.mspDeInit == 1u);
	CHECK(h.gState == HAL_UART_STATE_RESET);
}

/* ---------------------------------------------------------------- UI6 */

TEST_CASE_FIXTURE(UartFixture, "UI6 GetState is READY after Init") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(HAL_UART_GetState(&h) == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UI6 GetState is RESET before Init and after DeInit") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	CHECK(HAL_UART_GetState(&h) == HAL_UART_STATE_RESET);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	REQUIRE(HAL_UART_DeInit(&h) == HAL_OK);
	CHECK(HAL_UART_GetState(&h) == HAL_UART_STATE_RESET);
}

TEST_CASE_FIXTURE(UartFixture, "UI6 GetState is BUSY_TX while an interrupt mode transmission runs") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	uint8_t d[2] = {0x11, 0x22};
	REQUIRE(HAL_UART_Transmit_IT(&h, d, 2) == HAL_OK);
	CHECK(HAL_UART_GetState(&h) == HAL_UART_STATE_BUSY_TX);
}

TEST_CASE_FIXTURE(UartFixture, "UI6 GetState is BUSY_RX while an interrupt mode reception runs") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	uint8_t d[2] = {};
	REQUIRE(HAL_UART_Receive_IT(&h, d, 2) == HAL_OK);
	CHECK(HAL_UART_GetState(&h) == HAL_UART_STATE_BUSY_RX);
}

TEST_CASE_FIXTURE(UartFixture, "UI6 GetState is BUSY_TX_RX while a transmission and a reception run at the same time") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	uint8_t tx[2] = {0x11, 0x22}, rx[2] = {};
	REQUIRE(HAL_UART_Transmit_IT(&h, tx, 2) == HAL_OK);
	REQUIRE(HAL_UART_Receive_IT(&h, rx, 2) == HAL_OK); // UM1725: Tx and Rx state are separate
	CHECK(HAL_UART_GetState(&h) == HAL_UART_STATE_BUSY_TX_RX);
}

TEST_CASE_FIXTURE(UartFixture, "UI6 a reception started first and a transmission second also give BUSY_TX_RX") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	uint8_t tx[2] = {0x11, 0x22}, rx[2] = {};
	REQUIRE(HAL_UART_Receive_IT(&h, rx, 2) == HAL_OK);
	REQUIRE(HAL_UART_Transmit_IT(&h, tx, 2) == HAL_OK);
	CHECK(HAL_UART_GetState(&h) == HAL_UART_STATE_BUSY_TX_RX);
}

TEST_CASE_FIXTURE(UartFixture, "UI6 GetState is READY again after a polling transmit of one byte") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	uint8_t d = 0x5A;
	REQUIRE(HAL_UART_Transmit(&h, &d, 1, 100) == HAL_OK);
	CHECK(HAL_UART_GetState(&h) == HAL_UART_STATE_READY);
}

TEST_CASE_FIXTURE(UartFixture, "UI6 GetError is NONE after Init") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	CHECK(HAL_UART_GetError(&h) == HAL_UART_ERROR_NONE);
}

TEST_CASE_FIXTURE(UartFixture, "UI6 GetError returns each error code bit of the handle") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	const uint32_t codes[5] = {HAL_UART_ERROR_PE, HAL_UART_ERROR_NE, HAL_UART_ERROR_FE, HAL_UART_ERROR_ORE, HAL_UART_ERROR_DMA};
	for (unsigned k = 0; k < 5; ++k) {
		h.ErrorCode = codes[k];
		CAPTURE(k);
		CHECK(HAL_UART_GetError(&h) == codes[k]);
	}
}

TEST_CASE_FIXTURE(UartFixture, "UI6 GetError returns a combination of error code bits unchanged") {
	UART_HandleTypeDef h;
	uartInitMk(h, 1);
	REQUIRE(HAL_UART_Init(&h) == HAL_OK);
	h.ErrorCode = HAL_UART_ERROR_PE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE;
	CHECK(HAL_UART_GetError(&h) == (HAL_UART_ERROR_PE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE));
}

TEST_CASE_FIXTURE(UartFixture, "UI6 the error code names have distinct single bits") {
	// UM1725 lists the codes as separate flags: PE, NE, FE, ORE, DMA must not overlap and NONE is 0.
	const uint32_t codes[5] = {HAL_UART_ERROR_PE, HAL_UART_ERROR_NE, HAL_UART_ERROR_FE, HAL_UART_ERROR_ORE, HAL_UART_ERROR_DMA};
	CHECK(HAL_UART_ERROR_NONE == 0u);
	uint32_t all = 0;
	for (unsigned k = 0; k < 5; ++k) {
		CHECK(codes[k] != 0u);
		CHECK((codes[k] & (codes[k] - 1u)) == 0u);
		CHECK((all & codes[k]) == 0u);
		all |= codes[k];
	}
}
