/*
 * Self-tests of FakeTmc4671 (fakes/FakeTmc4671.h): frame decode, register file, stacked registers, strictness, bus
 * routing by chip select, time and hooks. The TMC4671 driver class is NOT used here: frames are sent with the fake HAL
 * SPI directly, the way the driver's readReg / writeReg do it (5 byte transfer, CS asserted by the caller).
 * Source of the expected values: TMC4671-LA datasheet Rev 2.07 ("DS §x pNN"), see the fake's header.
 */
#include "doctest.h"
#include "TmcFixture.h"

#include <cstdint>
#include <vector>

namespace {

using V = FakeTmc4671::ViolationKind;

struct FakeFixture : TmcFixture {
	static GPIO_TypeDef *csPort(int cs) { return cs == 0 ? SPI1_SS1_GPIO_Port : SPI1_SS2_GPIO_Port; }
	static uint16_t csPin(int cs) { return cs == 0 ? SPI1_SS1_Pin : SPI1_SS2_Pin; }
	static void csLow(int cs) { HAL_GPIO_WritePin(csPort(cs), csPin(cs), GPIO_PIN_RESET); }
	static void csHigh(int cs) { HAL_GPIO_WritePin(csPort(cs), csPin(cs), GPIO_PIN_SET); }

	/** Raw frame on CS `cs`; returns the MISO bytes */
	static std::vector<uint8_t> frame(int cs, std::vector<uint8_t> tx) {
		std::vector<uint8_t> rx(tx.size(), 0xEE);
		csLow(cs);
		REQUIRE(HAL_SPI_TransmitReceive(&hspi1, tx.data(), rx.data(), tx.size(), 100) == HAL_OK);
		csHigh(cs);
		return rx;
	}
	static uint32_t be(const std::vector<uint8_t> &rx) { return (uint32_t(rx[1]) << 24) | (uint32_t(rx[2]) << 16) | (uint32_t(rx[3]) << 8) | rx[4]; }
	static void write(int cs, uint8_t addr, uint32_t v) { frame(cs, {uint8_t(0x80 | addr), uint8_t(v >> 24), uint8_t(v >> 16), uint8_t(v >> 8), uint8_t(v)}); }
	static uint32_t read(int cs, uint8_t addr) { return be(frame(cs, {addr, 0, 0, 0, 0})); }
};

} // namespace

TEST_SUITE("FakeTmc4671") {

TEST_CASE_FIXTURE(FakeFixture, "the datasheet example reads SI_TYPE 0x34363731 after selecting CHIPINFO_ADDR 0 (DS §4.2.1 p17)") {
	write(0, 0x01, 0);
	std::vector<uint8_t> rx = frame(0, {0x00, 0, 0, 0, 0});
	CHECK(be(rx) == 0x34363731);
	CHECK(rx[0] == 0); // first MISO byte carries no data
}

TEST_CASE_FIXTURE(FakeFixture, "a write is stored MSB first and a read returns it in bytes 1 to 4 of its own frame") {
	write(0, 0x18, 0x00000123);
	write(0, 0x64, 0xFF380064);
	CHECK(read(0, 0x18) == 0x123);
	CHECK(read(0, 0x64) == 0xFF380064);
	CHECK(chip1.reg(0x64) == 0xFF380064);
}

TEST_CASE_FIXTURE(FakeFixture, "the answer to a write frame is the register content before the write") {
	write(0, 0x64, 0x11111111);
	std::vector<uint8_t> rx = frame(0, {0xE4, 0x22, 0x22, 0x22, 0x22});
	CHECK(be(rx) == 0x11111111); // assumption documented in the fake header
	CHECK(chip1.reg(0x64) == 0x22222222);
}

TEST_CASE_FIXTURE(FakeFixture, "the frame log records direction, address, data, answer and chip select in order") {
	write(0, 0x64, 0xAABBCCDD);
	read(0, 0x64);
	REQUIRE(chip1.log().size() == 2);
	CHECK(chip1.log()[0].write);
	CHECK(chip1.log()[0].addr == 0x64);
	CHECK(chip1.log()[0].data == 0xAABBCCDD);
	CHECK_FALSE(chip1.log()[1].write);
	CHECK(chip1.log()[1].answer == 0xAABBCCDD);
	CHECK(chip1.log()[1].cs == 0);
	CHECK(chip1.writesTo(0x64) == std::vector<uint32_t>{0xAABBCCDD});
	CHECK(chip1.readsOf(0x64) == 1);
}

TEST_CASE_FIXTURE(FakeFixture, "CHIPINFO_DATA follows CHIPINFO_ADDR: SI_VERSION defaults to 1.3 and other words are settable") {
	write(0, 0x01, 1);
	CHECK(read(0, 0x00) == 0x00010003);
	chip1.setChipInfo(2, 0x20211231);
	write(0, 0x01, 2);
	CHECK(read(0, 0x00) == 0x20211231);
	write(0, 0x01, 0);
	CHECK(read(0, 0x00) == 0x34363731);
}

TEST_CASE_FIXTURE(FakeFixture, "ADC_RAW_DATA follows ADC_RAW_ADDR (DS §7.1 p73) and returns the words set by the test") {
	chip1.setAdcRaw(0, 0x80017FFF);
	chip1.setAdcRaw(1, 0x12348000);
	write(0, 0x03, 0);
	CHECK(read(0, 0x02) == 0x80017FFF);
	write(0, 0x03, 1);
	CHECK(read(0, 0x02) == 0x12348000);
	write(0, 0x03, 2);
	CHECK(read(0, 0x02) == 0); // DS §7.3 p111: default 0
}

TEST_CASE_FIXTURE(FakeFixture, "CONFIG_DATA writes and reads hit the word selected by CONFIG_ADDR and are logged in order") {
	write(0, 0x4E, 25);
	write(0, 0x4D, 0x11111111);
	write(0, 0x4E, 62);
	write(0, 0x4D, 0x3F);
	write(0, 0x4E, 25);
	CHECK(read(0, 0x4D) == 0x11111111);
	CHECK(chip1.configWord(62) == 0x3F);
	CHECK(chip1.configWrites() == std::vector<std::pair<uint32_t, uint32_t>>{{25, 0x11111111}, {62, 0x3F}});
}

TEST_CASE_FIXTURE(FakeFixture, "PID_ERROR_DATA and INTERIM_DATA follow their address registers") {
	chip1.setPidError(3, 0xDEADBEEF);
	chip1.setInterim(4, 0x01020304);
	write(0, 0x6D, 3);
	write(0, 0x6F, 4);
	CHECK(read(0, 0x6C) == 0xDEADBEEF);
	CHECK(read(0, 0x6E) == 0x01020304);
}

TEST_CASE_FIXTURE(FakeFixture, "power-on defaults follow DS §7.3 for the registers the driver configures") {
	CHECK(chip1.reg(0x18) == 0xF9F);		// PWM_MAXCNT, DS §7.3 p112
	CHECK(chip1.reg(0x19) == 0x1414);		// BBM_L = BBM_H = 0x14
	CHECK(chip1.reg(0x1B) == 1);			// N_POLE_PAIRS = 1, MOTOR_TYPE 0
	CHECK(chip1.reg(0x08) == 0x01000000);	// ADC_I1_SCALE = 0x100 in [31:16]
	CHECK(chip1.reg(0x0A) == 0x24000100);	// I1 = 1 [15:8], V = 1 [27:26], WY = 2 [29:28]
	CHECK(chip1.reg(0x05) == 0x0CCCCCCD);	// dsADC_MCLK_A
	CHECK(chip1.reg(0x26) == 0x10000);		// ABN_DECODER_PPR
	CHECK(chip1.reg(0x40) == 1);			// AENC_DECODER_PPR
	CHECK(chip1.reg(0x5D) == 0x5A81);
	CHECK(chip1.reg(0x5E) == 0x7FFF);
	CHECK(chip1.reg(0x75) == 0xFFFFFFFF);	// ADC_VM_LIMIT_LOW and HIGH
	CHECK(chip1.reg(0x63) == 0);			// MODE_RAMP_MODE_MOTION: no default row, 0
	CHECK(chip1.reg(0x64) == 0);
	CHECK(chip1.reg(0x7C) == 0);
	CHECK(chip1.reg(0x7D) == 0);
}

TEST_CASE_FIXTURE(FakeFixture, "hall position defaults compose low and high halves (DS §7.3 p116)") {
	CHECK(chip1.reg(0x34) == 0x2AAA0000);	// _060 high, _000 low
	CHECK(chip1.reg(0x35) == 0x80005555);	// _180 = -0x8000, _120 = 0x5555
	CHECK(chip1.reg(0x36) == 0xD555AAAA);	// _300 = -0x2AAB, _240 = -0x5556
	CHECK(chip1.reg(0x38) == 0x2AAA);
}

TEST_CASE_FIXTURE(FakeFixture, "a write to a read-only register is ignored and recorded as a violation") {
	chip1.expectViolation(V::WriteToReadOnly, 0x6A);
	chip1.setReg(0x6A, 0x1234);
	write(0, 0x6A, 0xFFFFFFFF);
	CHECK(read(0, 0x6A) == 0x1234);
	REQUIRE(chip1.violations().size() == 1);
	CHECK(chip1.violations()[0].kind == V::WriteToReadOnly);
	CHECK(chip1.violations()[0].addr == 0x6A);
	CHECK(chip1.unexpectedViolations().empty());
}

TEST_CASE_FIXTURE(FakeFixture, "a violation the test did not expect is reported by unexpectedViolations") {
	write(0, 0x02, 1); // read-only ADC_RAW_DATA
	REQUIRE(chip1.unexpectedViolations().size() == 1);
	chip1.expectViolation(V::WriteToReadOnly); // declare it, otherwise the fixture fails this test at teardown
	CHECK(chip1.unexpectedViolations().empty());
}

TEST_CASE_FIXTURE(FakeFixture, "a transfer that is not a 5 byte datagram is a violation and is answered with zeros") {
	chip1.expectViolation(V::WrongFrameSize);
	std::vector<uint8_t> rx = frame(0, {0x00, 0, 0, 0});
	CHECK(rx == std::vector<uint8_t>{0, 0, 0, 0});
	frame(0, {0x00, 0, 0, 0, 0, 0});
	CHECK(chip1.violations().size() == 2);
	CHECK(chip1.log().empty());
}

TEST_CASE_FIXTURE(FakeFixture, "two chips on the bus answer only on their own chip select") {
	write(0, 0x64, 0x1);
	write(1, 0x64, 0x2);
	CHECK(read(0, 0x64) == 1);
	CHECK(read(1, 0x64) == 2);
	CHECK(chip1.log().size() == 2); // one write, one read
	CHECK(chip2.log().size() == 2);
	CHECK(chip2.log()[0].cs == 1);
}

TEST_CASE_FIXTURE(FakeFixture, "a frame with no chip select low is a violation recorded on the first chip") {
	chip1.expectViolation(V::NoSingleCsLow);
	std::vector<uint8_t> tx{0x00, 0, 0, 0, 0}, rx(5);
	REQUIRE(HAL_SPI_TransmitReceive(&hspi1, tx.data(), rx.data(), 5, 100) == HAL_OK);
	REQUIRE(chip1.violations().size() == 1);
	CHECK(chip1.violations()[0].kind == V::NoSingleCsLow);
}

TEST_CASE_FIXTURE(FakeFixture, "a chip that is not present answers zeros") {
	chip1.setPresent(false);
	write(0, 0x01, 0);
	CHECK(read(0, 0x00) == 0);
}

TEST_CASE_FIXTURE(FakeFixture, "raised status flags stay until a write of 0 to STATUS_FLAGS clears them (DS 0x7C p107)") {
	chip1.raiseFlags(1u << 26);
	chip1.raiseFlags(1u << 19);
	CHECK(read(0, 0x7C) == ((1u << 26) | (1u << 19)));
	CHECK(read(0, 0x7C) == ((1u << 26) | (1u << 19)));
	write(0, 0x7C, 0);
	CHECK(read(0, 0x7C) == 0);
}

TEST_CASE_FIXTURE(FakeFixture, "a write of PID_POSITION_ACTUAL also writes PID_POSITION_TARGET (DS 0x6B p98)") {
	write(0, 0x68, 0x1000);
	write(0, 0x6B, 0xFFFF0005);
	CHECK(read(0, 0x6B) == 0xFFFF0005);
	CHECK(read(0, 0x68) == 0xFFFF0005);
}

TEST_CASE_FIXTURE(FakeFixture, "ENABLE_IN_RAW is high by default and follows setEnableIn") {
	CHECK((read(0, 0x76) & 0x8000) != 0);
	chip1.setEnableIn(false);
	CHECK((read(0, 0x76) & 0x8000) == 0);
	chip1.setNPulse(true);
	CHECK((read(0, 0x76) & 0x4) != 0);
}

TEST_CASE_FIXTURE(FakeFixture, "accesses to addresses Rev 2.07 does not document are stored and recorded, not violations") {
	write(0, 0x5C, 0xCAFE);
	CHECK(read(0, 0x5C) == 0xCAFE);
	REQUIRE(chip1.undocumentedAccesses().size() == 2);
	CHECK(chip1.undocumentedAccesses()[0].addr == 0x5C);
	CHECK(chip1.undocumentedAccesses()[0].write);
	CHECK(chip1.violations().empty());
	read(0, 0x5F); // listed in DS §7.3 only: documented
	CHECK(chip1.undocumentedAccesses().size() == 2);
	CHECK(read(0, 0x5F) == 0x7FFFFFFF);
}

TEST_CASE_FIXTURE(FakeFixture, "simulated time advances by the configured microseconds per frame") {
	uint64_t t0 = HostPlatform::nowUs();
	read(0, 0x00);
	CHECK(HostPlatform::nowUs() == t0);
	chip1.setMicrosPerTransfer(8);
	read(0, 0x00);
	read(0, 0x00);
	CHECK(HostPlatform::nowUs() == t0 + 16);
	CHECK(chip1.log()[1].timeUs == t0); // time at the start of the frame
}

TEST_CASE_FIXTURE(FakeFixture, "read and write hooks run and can script register values") {
	int writes = 0;
	chip1.onRead(0x6A, [](FakeTmc4671 &c) { c.setReg(0x6A, 0x777); });
	chip1.onWrite(0x64, [&](FakeTmc4671 &c, uint32_t d) {
		writes++;
		c.setReg(0x69, d); // actual follows target
	});
	CHECK(read(0, 0x6A) == 0x777);
	write(0, 0x64, 0x00050003);
	CHECK(writes == 1);
	CHECK(read(0, 0x69) == 0x00050003);
}

TEST_CASE_FIXTURE(FakeFixture, "reset restores defaults and clears logs, hooks and expectations") {
	write(0, 0x18, 5);
	chip1.expectViolation(V::WrongFrameSize);
	chip1.onRead(0x00, [](FakeTmc4671 &) {});
	chip1.reset();
	CHECK(chip1.reg(0x18) == 0xF9F);
	CHECK(chip1.log().empty());
	frame(0, {0x00, 0, 0, 0, 0});
	CHECK(chip1.log().size() == 1);
	chip1.reset();
}

}
