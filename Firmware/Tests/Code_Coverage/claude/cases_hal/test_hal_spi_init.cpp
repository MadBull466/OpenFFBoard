/*
 * P12 T4a SPI init cases (requirements SI1-SI7 of Tests/Code_Coverage/claude/charters/hal/P12_T4A_REQUIREMENTS.md).
 * Sources under test: stm32f4xx_hal_spi.c of the real ST HAL (HAL_SPI_Init / DeInit / GetState / GetError), built into
 * ffb_hal_tests and ffb_hal_tests_disco, on the SPI register model fakes/stm32_hal/HalSpi.{h,c}.
 *
 * Oracles: RM0090 Rev 22 28.5.1 / 28.5.2 and UM1725 Rev 8 (SPI). Expected register words are written with literal bit values:
 *   CR1: CPHA bit 0, CPOL bit 1, MSTR bit 2, BR[2:0] bits 5:3, SPE bit 6, LSBFIRST bit 7, SSI bit 8, SSM bit 9, RXONLY bit 10,
 *        DFF bit 11, CRCEN bit 13, BIDIOE bit 14, BIDIMODE bit 15.
 *   CR2: SSOE bit 2, FRF bit 4 (TXEIE 7, RXNEIE 6, ERRIE 5, TXDMAEN 1, RXDMAEN 0).
 *   I2SCFGR: I2SMOD bit 11. CRCPR reset value 0x0007.
 * Master mode with software NSS needs SSI = 1 (RM0090 28.3.1: the NSS level comes from SSI), so the base word of a master with
 * software NSS is MSTR | SSI | SSM = 0x0304.
 * Not compiled in this configuration: HAL_SPI_RegisterCallback / UnRegisterCallback (USE_HAL_SPI_REGISTER_CALLBACKS 0): SI7 has
 * nothing to test. SI6 (parameter asserts) is not tested (USE_FULL_ASSERT off, owner's decision 68).
 * The strong HAL_SPI_* callbacks live in test_hal_spi_support.cpp.
 */
#include "SpiTestSupport.h"

#include <cstdint>
#include <string>

static_assert(USE_HAL_SPI_REGISTER_CALLBACKS == 0, "SI7: the register / unregister functions are not compiled in this build");

namespace {

SPI_TypeDef *const spiInitInst[3] = {SPI1, SPI2, SPI3};

struct SpiInitFixture : SpiFixture {
	SPI_HandleTypeDef h = {};

	SpiInitFixture() { setup(0); }

	// Default configuration: master, 2 lines, 8 bit, CPOL 0, CPHA 0, soft NSS, prescaler 2, MSB first, Motorola, no CRC.
	void setup(unsigned spi) {
		h = SPI_HandleTypeDef{};
		h.Instance = spiInitInst[spi];
		h.Init.Mode = SPI_MODE_MASTER;
		h.Init.Direction = SPI_DIRECTION_2LINES;
		h.Init.DataSize = SPI_DATASIZE_8BIT;
		h.Init.CLKPolarity = SPI_POLARITY_LOW;
		h.Init.CLKPhase = SPI_PHASE_1EDGE;
		h.Init.NSS = SPI_NSS_SOFT;
		h.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
		h.Init.FirstBit = SPI_FIRSTBIT_MSB;
		h.Init.TIMode = SPI_TIMODE_DISABLE;
		h.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		h.Init.CRCPolynomial = 7;
	}
	uint32_t cr1() const { return peek(&h.Instance->CR1); }
	uint32_t cr2() const { return peek(&h.Instance->CR2); }
	void doInit() { REQUIRE(HAL_SPI_Init(&h) == HAL_OK); }
};

const uint32_t spiInitBase = 0x0304u; // MSTR | SSI | SSM

} // namespace

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 master with software NSS writes CR1 = MSTR | SSI | SSM") {
	doInit();
	CHECK(cr1() == spiInitBase);
	CHECK(cr2() == 0u);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 slave with software NSS writes CR1 = SSM only") {
	h.Init.Mode = SPI_MODE_SLAVE;
	doInit();
	CHECK(cr1() == 0x0200u); // MSTR 0; SSI 0 keeps the slave selected through SSM
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 slave with hardware NSS input writes CR1 = 0") {
	h.Init.Mode = SPI_MODE_SLAVE;
	h.Init.NSS = SPI_NSS_HARD_INPUT;
	doInit();
	CHECK(cr1() == 0u);
	CHECK(cr2() == 0u);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 the direction selects BIDIMODE and RXONLY") {
	uint32_t dir = 0, expect = 0;
	SUBCASE("2 lines") { dir = SPI_DIRECTION_2LINES; expect = 0x0000u; }
	SUBCASE("2 lines receive only") { dir = SPI_DIRECTION_2LINES_RXONLY; expect = 0x0400u; } // RXONLY bit 10
	SUBCASE("1 line") { dir = SPI_DIRECTION_1LINE; expect = 0x8000u; }                       // BIDIMODE bit 15
	h.Init.Direction = dir;
	doInit();
	CHECK(cr1() == (spiInitBase | expect));
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 one line direction leaves BIDIOE cleared") {
	h.Init.Direction = SPI_DIRECTION_1LINE;
	doInit();
	CHECK((cr1() & 0x4000u) == 0u); // BIDIOE bit 14: set later by Transmit
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 the data size selects DFF") {
	uint32_t size = 0, expect = 0;
	SUBCASE("8 bit") { size = SPI_DATASIZE_8BIT; expect = 0x0000u; }
	SUBCASE("16 bit") { size = SPI_DATASIZE_16BIT; expect = 0x0800u; } // DFF bit 11
	h.Init.DataSize = size;
	doInit();
	CHECK(cr1() == (spiInitBase | expect));
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 clock polarity and phase select CPOL and CPHA") {
	uint32_t pol = 0, ph = 0, expect = 0;
	SUBCASE("CPOL 0 CPHA 0") { pol = SPI_POLARITY_LOW; ph = SPI_PHASE_1EDGE; expect = 0x0u; }
	SUBCASE("CPOL 0 CPHA 1") { pol = SPI_POLARITY_LOW; ph = SPI_PHASE_2EDGE; expect = 0x1u; }
	SUBCASE("CPOL 1 CPHA 0") { pol = SPI_POLARITY_HIGH; ph = SPI_PHASE_1EDGE; expect = 0x2u; }
	SUBCASE("CPOL 1 CPHA 1") { pol = SPI_POLARITY_HIGH; ph = SPI_PHASE_2EDGE; expect = 0x3u; }
	h.Init.CLKPolarity = pol;
	h.Init.CLKPhase = ph;
	doInit();
	CHECK(cr1() == (spiInitBase | expect));
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 software NSS sets SSM and leaves SSOE cleared") {
	h.Init.NSS = SPI_NSS_SOFT;
	doInit();
	CHECK((cr1() & 0x0200u) == 0x0200u); // SSM bit 9
	CHECK((cr2() & 0x0004u) == 0u);      // SSOE bit 2
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 hardware NSS input clears SSM and SSOE") {
	h.Init.NSS = SPI_NSS_HARD_INPUT;
	doInit();
	CHECK((cr1() & 0x0200u) == 0u);
	CHECK((cr2() & 0x0004u) == 0u);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 hardware NSS output clears SSM and sets SSOE") {
	h.Init.NSS = SPI_NSS_HARD_OUTPUT;
	doInit();
	CHECK((cr1() & 0x0200u) == 0u);
	CHECK((cr2() & 0x0004u) == 0x0004u); // SSOE bit 2
}

// characterization: SSI is 1 for a master also with hardware NSS (RM0090: SSI has an effect only when SSM = 1).
// Proposed NEEDS REVIEW (number from 307): "SI1 master with hardware NSS: Init sets SSI; RM0090 28.5.1 says the bit has no effect
// without SSM, UM1725 does not say. Confirm acceptable."
TEST_CASE_FIXTURE(SpiInitFixture, "SI1 characterization: master with hardware NSS input has SSI set") {
	h.Init.NSS = SPI_NSS_HARD_INPUT;
	doInit();
	CHECK(cr1() == 0x0104u); // MSTR | SSI
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 every baud rate prescaler writes its BR value") {
	// RM0090 28.5.1: BR[2:0] 000 = fPCLK/2 ... 111 = fPCLK/256
	struct Case { uint32_t presc; uint32_t br; };
	const Case cases[8] = {
		{SPI_BAUDRATEPRESCALER_2, 0}, {SPI_BAUDRATEPRESCALER_4, 1}, {SPI_BAUDRATEPRESCALER_8, 2},
		{SPI_BAUDRATEPRESCALER_16, 3}, {SPI_BAUDRATEPRESCALER_32, 4}, {SPI_BAUDRATEPRESCALER_64, 5},
		{SPI_BAUDRATEPRESCALER_128, 6}, {SPI_BAUDRATEPRESCALER_256, 7}};
	for (unsigned i = 0; i < 8; ++i) {
		SUBCASE(("prescaler index " + std::to_string(i)).c_str()) {
			setup(0);
			h.Init.BaudRatePrescaler = cases[i].presc;
			doInit();
			CHECK(cr1() == (spiInitBase | (cases[i].br << 3)));
		}
	}
}

// characterization: in slave mode (Motorola) the prescaler is not used by the HAL and BR is written as 000. RM0090 does not say
// whether BR matters for a slave. Proposed NEEDS REVIEW: "SI1 slave: requested prescaler 256 is replaced by BR = 000 in CR1."
TEST_CASE_FIXTURE(SpiInitFixture, "SI1 characterization: slave mode writes BR = 000 whatever the prescaler") {
	h.Init.Mode = SPI_MODE_SLAVE;
	h.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	doInit();
	CHECK(cr1() == 0x0200u);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 the first bit selects LSBFIRST") {
	uint32_t first = 0, expect = 0;
	SUBCASE("MSB first") { first = SPI_FIRSTBIT_MSB; expect = 0x0000u; }
	SUBCASE("LSB first") { first = SPI_FIRSTBIT_LSB; expect = 0x0080u; } // LSBFIRST bit 7
	h.Init.FirstBit = first;
	doInit();
	CHECK(cr1() == (spiInitBase | expect));
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 TI mode sets FRF in CR2 and Motorola mode clears it") {
	uint32_t ti = 0, expect = 0;
	SUBCASE("Motorola") { ti = SPI_TIMODE_DISABLE; expect = 0x00u; }
	SUBCASE("TI") { ti = SPI_TIMODE_ENABLE; expect = 0x10u; } // FRF bit 4
	h.Init.TIMode = ti;
	doInit();
	CHECK(cr2() == expect);
}

// characterization: in TI mode the HAL forces CPOL = 0 and CPHA = 0 even when the structure asks for 1 (RM0090: both bits are
// "not used in SPI TI mode"). Proposed NEEDS REVIEW: "SI1 TI mode: requested CPOL / CPHA high is written as 0."
TEST_CASE_FIXTURE(SpiInitFixture, "SI1 characterization: TI mode writes CPOL and CPHA as 0") {
	h.Init.TIMode = SPI_TIMODE_ENABLE;
	h.Init.CLKPolarity = SPI_POLARITY_HIGH;
	h.Init.CLKPhase = SPI_PHASE_2EDGE;
	doInit();
	CHECK((cr1() & 0x3u) == 0u);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 Init clears SPE of a peripheral that was enabled") {
	poke(&h.Instance->CR1, 0x0040u); // SPE bit 6
	doInit();
	CHECK((cr1() & 0x0040u) == 0u);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 Init of a running peripheral changes no register while SPE is 1") {
	// RM0090 28.5.1: DFF, BR, MSTR, ... must only be written with SPE = 0.
	poke(&h.Instance->CR1, 0x0040u);
	h.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	h.Init.DataSize = SPI_DATASIZE_16BIT;
	doInit();
	CHECK(hostSpiConfigWhileEnabled(0) == 0u);
	CHECK(hostSpiDisabledBusy(0) == 0u);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 Init clears I2SMOD so that the SPI mode is active") {
	poke(&h.Instance->I2SCFGR, 0x0800u); // I2SMOD bit 11
	doInit();
	CHECK((peek(&h.Instance->I2SCFGR) & 0x0800u) == 0u);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 Init leaves the handle READY with error code NONE") {
	h.ErrorCode = HAL_SPI_ERROR_OVR | HAL_SPI_ERROR_MODF;
	doInit();
	CHECK(h.State == HAL_SPI_STATE_READY);
	CHECK(h.ErrorCode == HAL_SPI_ERROR_NONE);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 Init calls the MSP init callback once with the handle") {
	doInit();
	CHECK(spiCounts.mspInit == 1u);
	CHECK(spiCounts.last == &h);
	CHECK(spiCounts.mspDeInit == 0u);
}

// characterization: CR2 is written as a whole, so enable bits left in it are lost. Proposed NEEDS REVIEW: "SI1 Init overwrites CR2
// (TXEIE / RXNEIE / ERRIE / DMA enables left from an earlier use are cleared)."
TEST_CASE_FIXTURE(SpiInitFixture, "SI1 characterization: Init overwrites CR2 and clears interrupt and DMA enables") {
	poke(&h.Instance->CR2, 0x00E3u); // TXEIE, RXNEIE, ERRIE, TXDMAEN, RXDMAEN
	doInit();
	CHECK(cr2() == 0u);
}

// characterization: USE_SPI_CRC is 0, the CRC code is compiled out: CRCEN stays 0 and CRCPR keeps its reset value 0x0007 even
// with CRCCalculation = ENABLE and a polynomial in the structure. Proposed NEEDS REVIEW: "SI1 CRCPolynomial is not written to
// CRCPR (CRC compiled out); CRCEN is cleared."
TEST_CASE_FIXTURE(SpiInitFixture, "SI1 characterization: a requested CRC calculation writes neither CRCEN nor CRCPR") {
	h.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
	h.Init.CRCPolynomial = 0x1234;
	doInit();
	CHECK((cr1() & 0x2000u) == 0u);          // CRCEN bit 13
	CHECK(peek(&h.Instance->CRCPR) == 0x0007u); // reset value, RM0090 28.5.5
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI1 a combined configuration writes the OR of all its fields") {
	// slave, RX only, 16 bit, CPOL 1, CPHA 1, hardware output NSS (so SSM 0), LSB first, TI off
	h.Init.Mode = SPI_MODE_SLAVE;
	h.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
	h.Init.DataSize = SPI_DATASIZE_16BIT;
	h.Init.CLKPolarity = SPI_POLARITY_HIGH;
	h.Init.CLKPhase = SPI_PHASE_2EDGE;
	h.Init.NSS = SPI_NSS_HARD_OUTPUT;
	h.Init.FirstBit = SPI_FIRSTBIT_LSB;
	doInit();
	// RXONLY 0x400 | DFF 0x800 | CPOL 2 | CPHA 1 | LSBFIRST 0x80
	CHECK(cr1() == 0x0C83u);
	CHECK(cr2() == 0x0004u);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI2 the instances sit at the RM0090 Table 1 addresses") {
	CHECK(reinterpret_cast<std::uintptr_t>(SPI1) == 0x40013000u);
	CHECK(reinterpret_cast<std::uintptr_t>(SPI2) == 0x40003800u);
	CHECK(reinterpret_cast<std::uintptr_t>(SPI3) == 0x40003C00u);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI2 Init of each instance writes that instance and none of the others") {
	for (unsigned s = 0; s < 3; ++s) {
		SUBCASE(("instance " + std::to_string(s + 1)).c_str()) {
			setup(s);
			h.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
			h.Init.NSS = SPI_NSS_HARD_OUTPUT;
			const uint32_t other1 = 0x0045u, other2 = 0x0020u; // distinctive CR1 / CR2 words of the other instances (SPE kept 0 where used)
			for (unsigned o = 0; o < 3; ++o) {
				if (o == s) continue;
				poke(&spiInitInst[o]->CR1, 0x0001u | (other1 & 0x0004u) | (o << 3));
				poke(&spiInitInst[o]->CR2, other2);
			}
			doInit();
			// MSTR | SSI | BR 100 (32) = 0x104 | 0x20 ; SSM 0 (hard output), SSOE 1
			CHECK(cr1() == 0x0124u);
			CHECK(cr2() == 0x0004u);
			for (unsigned o = 0; o < 3; ++o) {
				if (o == s) continue;
				CHECK(peek(&spiInitInst[o]->CR1) == (0x0005u | (o << 3)));
				CHECK(peek(&spiInitInst[o]->CR2) == other2);
			}
		}
	}
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI2 Init of each instance calls MspInit with that instance's handle") {
	for (unsigned s = 0; s < 3; ++s) {
		SUBCASE(("instance " + std::to_string(s + 1)).c_str()) {
			setup(s);
			doInit();
			CHECK(spiCounts.mspInit == 1u);
			CHECK(spiCounts.last == &h);
			CHECK(spiCounts.last->Instance == spiInitInst[s]);
			CHECK(cr1() == spiInitBase);
		}
	}
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI3 Init with a NULL handle returns HAL_ERROR and calls nothing") {
	CHECK(HAL_SPI_Init(nullptr) == HAL_ERROR);
	CHECK(spiCounts.mspInit == 0u);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI3 Init of a handle in state RESET unlocks it and calls MspInit") {
	h.Lock = HAL_LOCKED;
	REQUIRE(h.State == HAL_SPI_STATE_RESET);
	doInit();
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(spiCounts.mspInit == 1u);
}

// characterization: re-init of a READY handle returns HAL_OK, does not call MspInit again and writes the registers again.
// Proposed NEEDS REVIEW: "SI3 second HAL_SPI_Init on a READY handle: HAL_OK, MspInit not called again, new CR1 written."
TEST_CASE_FIXTURE(SpiInitFixture, "SI3 characterization: Init of a READY handle does not call MspInit again") {
	doInit();
	REQUIRE(h.State == HAL_SPI_STATE_READY);
	h.Init.FirstBit = SPI_FIRSTBIT_LSB;
	CHECK(HAL_SPI_Init(&h) == HAL_OK);
	CHECK(spiCounts.mspInit == 1u);
	CHECK(cr1() == (spiInitBase | 0x0080u));
	CHECK(h.State == HAL_SPI_STATE_READY);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI3 Init after DeInit calls MspInit again") {
	doInit();
	REQUIRE(HAL_SPI_DeInit(&h) == HAL_OK);
	doInit();
	CHECK(spiCounts.mspInit == 2u);
	CHECK(h.State == HAL_SPI_STATE_READY);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI4 DeInit clears SPE") {
	doInit();
	poke(&h.Instance->CR1, cr1() | 0x0040u);
	REQUIRE(HAL_SPI_DeInit(&h) == HAL_OK);
	CHECK((cr1() & 0x0040u) == 0u);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI4 DeInit sets state RESET, error code NONE and unlocks the handle") {
	doInit();
	h.ErrorCode = HAL_SPI_ERROR_OVR;
	h.Lock = HAL_LOCKED;
	CHECK(HAL_SPI_DeInit(&h) == HAL_OK);
	CHECK(h.State == HAL_SPI_STATE_RESET);
	CHECK(h.ErrorCode == HAL_SPI_ERROR_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI4 DeInit calls the MSP deinit callback once with the handle") {
	doInit();
	CHECK(HAL_SPI_DeInit(&h) == HAL_OK);
	CHECK(spiCounts.mspDeInit == 1u);
	CHECK(spiCounts.last == &h);
}

// characterization: the HAL itself writes only SPE of CR1 in DeInit; the reset of the other registers is left to MspDeInit /
// the RCC reset. Proposed NEEDS REVIEW: "SI4 DeInit changes only CR1.SPE; CR1 configuration bits and CR2 stay as they were."
TEST_CASE_FIXTURE(SpiInitFixture, "SI4 characterization: DeInit changes only the SPE bit of the registers") {
	h.Init.NSS = SPI_NSS_HARD_OUTPUT;
	h.Init.DataSize = SPI_DATASIZE_16BIT;
	doInit();
	const uint32_t before1 = cr1(), before2 = cr2();
	poke(&h.Instance->CR1, before1 | 0x0040u);
	REQUIRE(HAL_SPI_DeInit(&h) == HAL_OK);
	CHECK(cr1() == before1);
	CHECK(cr2() == before2);
}

// characterization: DeInit with a NULL handle returns HAL_ERROR (UM1725 gives no return codes). Proposed NEEDS REVIEW: "SI4
// HAL_SPI_DeInit(NULL) returns HAL_ERROR without calling MspDeInit."
TEST_CASE_FIXTURE(SpiInitFixture, "SI4 characterization: DeInit with a NULL handle returns HAL_ERROR") {
	CHECK(HAL_SPI_DeInit(nullptr) == HAL_ERROR);
	CHECK(spiCounts.mspDeInit == 0u);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI5 GetState is RESET before Init and READY after it") {
	CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_RESET);
	doInit();
	CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI5 GetState returns the handle's state for every state name") {
	const HAL_SPI_StateTypeDef states[] = {HAL_SPI_STATE_RESET, HAL_SPI_STATE_READY, HAL_SPI_STATE_BUSY, HAL_SPI_STATE_BUSY_TX,
	                                       HAL_SPI_STATE_BUSY_RX, HAL_SPI_STATE_BUSY_TX_RX, HAL_SPI_STATE_ERROR,
	                                       HAL_SPI_STATE_ABORT};
	for (HAL_SPI_StateTypeDef s : states) {
		h.State = s;
		CHECK(HAL_SPI_GetState(&h) == s);
	}
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI5 GetError returns NONE after Init and the code bits that are set") {
	doInit();
	CHECK(HAL_SPI_GetError(&h) == HAL_SPI_ERROR_NONE);
	h.ErrorCode = HAL_SPI_ERROR_OVR | HAL_SPI_ERROR_DMA;
	CHECK(HAL_SPI_GetError(&h) == (HAL_SPI_ERROR_OVR | HAL_SPI_ERROR_DMA));
	h.ErrorCode = HAL_SPI_ERROR_MODF;
	CHECK(HAL_SPI_GetError(&h) == HAL_SPI_ERROR_MODF);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI5 the error code bits have the UM1725 values") {
	CHECK(HAL_SPI_ERROR_NONE == 0x00u);
	CHECK(HAL_SPI_ERROR_MODF == 0x01u);
	CHECK(HAL_SPI_ERROR_CRC == 0x02u);
	CHECK(HAL_SPI_ERROR_OVR == 0x04u);
	CHECK(HAL_SPI_ERROR_FRE == 0x08u);
	CHECK(HAL_SPI_ERROR_DMA == 0x10u);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI5 an interrupt transmit puts the state at BUSY_TX") {
	static uint8_t buf[2] = {1, 2};
	doInit();
	REQUIRE(HAL_SPI_Transmit_IT(&h, buf, 2) == HAL_OK);
	CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_BUSY_TX);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI5 an interrupt receive puts the state at BUSY_RX") {
	static uint8_t buf[2];
	doInit();
	REQUIRE(HAL_SPI_Receive_IT(&h, buf, 2) == HAL_OK);
	CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_BUSY_RX);
}

TEST_CASE_FIXTURE(SpiInitFixture, "SI5 an interrupt transmit receive puts the state at BUSY_TX_RX") {
	static uint8_t tx[2] = {1, 2};
	static uint8_t rx[2];
	doInit();
	REQUIRE(HAL_SPI_TransmitReceive_IT(&h, tx, rx, 2) == HAL_OK);
	CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_BUSY_TX_RX);
}
