/*
 * Tests of the CRC helpers (CRC.cpp, CRC.h: reverseBits, makeCrcTable, calculateCrc8, calculateCrc16_8,
 * calculateCrc16_8_rev) with the check values of the catalogue "CRC RevEng" (Greg Cook): the CRC of the 9 ASCII bytes
 * "123456789" for standard parameter sets. The parameter sets match the firmware users: CRC-8 x^8+x^2+x+1 (MtEncoderSPI,
 * MotorSimplemotion), CRC-16 0x1021 (VescCAN), reflected CRC-16 0x8005 with calculateCrc16_8_rev (MotorSimplemotion),
 * CRC-6 x^6+x+1 table (EncoderBissC).
 */
#include "doctest.h"
#include "TestHelpers.h"

#include "CRC.h"

#include <array>
#include <cstdint>
#include <cstring>

namespace {

std::array<uint8_t, 9> checkInput() {
	std::array<uint8_t, 9> buf{};
	std::memcpy(buf.data(), "123456789", 9);
	return buf;
}

/** Remainder of value * x^bits divided by the polynomial poly (with its x^bits term), bitwise reference */
uint32_t polyRemainder(uint32_t value, uint32_t poly, int bits) {
	uint32_t crc = value;
	for (int i = 0; i < bits; i++) {
		crc <<= 1;
		if (crc & (1U << bits)) {
			crc ^= poly | (1U << bits);
		}
	}
	return crc & ((1U << bits) - 1);
}

} // namespace

TEST_SUITE("CRC") {

TEST_CASE("reverseBits reverses the bit order of 8, 16 and 32 bit values") {
	CHECK(reverseBits<uint8_t>(0x01) == 0x80);
	CHECK(reverseBits<uint8_t>(0xB4) == 0x2D);
	CHECK(reverseBits<uint16_t>(0x0001) == 0x8000);
	CHECK(reverseBits<uint16_t>(0x1234) == 0x2C48);
	CHECK(reverseBits<uint32_t>(0x00000001) == 0x80000000U);
	CHECK(reverseBits<uint32_t>(0x12345678) == 0x1E6A2C48U);
}

TEST_CASE("reverseBits applied twice gives the original value") {
	for (uint32_t v = 0; v < 256; v++) {
		CHECK(reverseBits<uint8_t>(reverseBits<uint8_t>(static_cast<uint8_t>(v))) == v);
	}
	CHECK(reverseBits<uint16_t>(reverseBits<uint16_t>(0xA5C3)) == 0xA5C3);
	CHECK(reverseBits<uint32_t>(reverseBits<uint32_t>(0xDEADBEEF)) == 0xDEADBEEFU);
}

TEST_CASE("an 8 bit CRC table holds the remainder of every byte shifted by 8 bits") {
	std::array<uint8_t, 256> table{};
	makeCrcTable(table, static_cast<uint8_t>(0x07), 8);
	for (uint32_t i = 0; i < 256; i++) {
		CAPTURE(i);
		CHECK(table[i] == polyRemainder(i, 0x07, 8));
	}
}

TEST_CASE("a 6 bit CRC table of x^6 + x + 1 holds the remainder of every 6 bit value") {
	std::array<uint8_t, 64> table{};
	makeCrcTable(table, static_cast<uint8_t>(0x43), 6); // like EncoderBissC (POLY 0x43 includes the x^6 term)
	for (uint32_t i = 0; i < 64; i++) {
		CAPTURE(i);
		CHECK(table[i] == polyRemainder(i, 0x03, 6));
	}
}

TEST_CASE("a 16 bit CRC table holds the remainder of every byte shifted by 16 bits") {
	std::array<uint16_t, 256> table{};
	makeCrcTable(table, static_cast<uint16_t>(0x1021), 16);
	for (uint32_t i = 0; i < 256; i++) {
		CAPTURE(i);
		CHECK(table[i] == polyRemainder(i, 0x1021, 16));
	}
}

TEST_CASE("calculateCrc8 with the x^8 + x^2 + x + 1 table gives the CRC-8/SMBUS check value 0xF4") {
	std::array<uint8_t, 256> table{};
	makeCrcTable(table, static_cast<uint8_t>(0x07), 8);
	auto buf = checkInput();
	CHECK(calculateCrc8(table, buf.data(), 9) == 0xF4);
}

TEST_CASE("calculateCrc8 starts with the given initial value (CRC-8/AUTOSAR check value 0xDF)") {
	std::array<uint8_t, 256> table{};
	makeCrcTable(table, static_cast<uint8_t>(0x2F), 8);
	auto buf = checkInput();
	CHECK((calculateCrc8(table, buf.data(), 9, 0xFF) ^ 0xFF) == 0xDF); // init 0xFF, xorout 0xFF
}

TEST_CASE("a reflected 8 bit table gives the CRC-8/MAXIM-DOW check value 0xA1") {
	std::array<uint8_t, 256> table{};
	makeCrcTable(table, static_cast<uint8_t>(0x31), 8, true, true);
	auto buf = checkInput();
	CHECK(calculateCrc8(table, buf.data(), 9) == 0xA1);
}

TEST_CASE("calculateCrc8 of no bytes returns the initial value") {
	std::array<uint8_t, 256> table{};
	makeCrcTable(table, static_cast<uint8_t>(0x07), 8);
	uint8_t byte = 0x12;
	CHECK(calculateCrc8(table, &byte, 0, 0x5A) == 0x5A);
}

TEST_CASE("calculateCrc16_8 with the 0x1021 table gives the CRC-16/XMODEM check value 0x31C3") {
	std::array<uint16_t, 256> table{};
	makeCrcTable(table, static_cast<uint16_t>(0x1021), 16);
	auto buf = checkInput();
	CHECK(calculateCrc16_8(table, buf.data(), 9) == 0x31C3);
}

TEST_CASE("calculateCrc16_8 starts with the given initial value (CRC-16/IBM-3740 check value 0x29B1)") {
	std::array<uint16_t, 256> table{};
	makeCrcTable(table, static_cast<uint16_t>(0x1021), 16);
	auto buf = checkInput();
	CHECK(calculateCrc16_8(table, buf.data(), 9, 0xFFFF) == 0x29B1);
}

TEST_CASE("calculateCrc16_8_rev with the reflected 0x8005 table gives the byte swapped CRC-16/ARC check value") {
	// MotorSimplemotion: makeCrcTable(tableCRC16, 0x8005, 16, true, true) and calculateCrc16_8_rev(..., 0). The
	// reflected CRC-16/ARC of "123456789" is 0xBB3D; the MSB-first update with byte swapped table entries gives it
	// with its bytes swapped
	std::array<uint16_t, 256> table{};
	makeCrcTable(table, static_cast<uint16_t>(0x8005), 16, true, true);
	auto buf = checkInput();
	CHECK(calculateCrc16_8_rev(table, buf.data(), 9) == 0x3DBB);
}

TEST_CASE("a frame followed by its CRC-16/XMODEM gives a remainder of 0") {
	std::array<uint16_t, 256> table{};
	makeCrcTable(table, static_cast<uint16_t>(0x1021), 16);
	uint8_t buf[11];
	std::memcpy(buf, "123456789", 9);
	const uint16_t crc = calculateCrc16_8(table, buf, 9);
	buf[9] = static_cast<uint8_t>(crc >> 8);
	buf[10] = static_cast<uint8_t>(crc);
	CHECK(calculateCrc16_8(table, buf, 11) == 0);
}

} // TEST_SUITE
