/*
 * TMC4671 driver, group G1: SPI transport (requirements R1-R7; R8 is untestable on the host and has no test).
 * Charter: Tests/Code_Coverage/claude/charters/driver/TMC4671_REQUIREMENTS.md. Fake peer: fakes/FakeTmc4671.h (frame decode High confidence).
 *
 * Sources: DS = TMC4671-LA datasheet rev 2.07 (DS §4.2.1 p17 datagram: 40 bit, MSB first, bit 39 = WRnRD, bits 38..32 =
 * address, bits 31..0 = data; the chip answers inside the same datagram). RM = RM0090 (SPI_CR1.BR: fSCK = fPCLK / 2^(BR+1)).
 *
 * Host vs target: frames are byte arrays built with memcpy + __REV (host stub); the 32-bit values are uint32_t on both.
 * The SCK values are computed from the prescaler constant in double on the host (exact: powers of two of 84e6).
 */
#include "doctest.h"
#include "TmcFixture.h"

namespace {

using HostTarget::SpiTransfer;

/** SPI1 is clocked from APB2 = 84 MHz on the F407 board (motor_spi baseclk in cpp_target_config.cpp, RM0090 clock tree) */
constexpr double kSpiBaseClkHz = 84e6;

/** SCK in Hz from SPI_InitTypeDef::BaudRatePrescaler. HD: the constant is BR << 3 (BR = CR1[5:3]), divisor = 2^(BR+1) = 2 << BR */
double sckHz(uint32_t baudRatePrescaler) {
	return kSpiBaseClkHz / double(2u << (baudRatePrescaler >> 3));
}

/** Expect that only CS index `idx` is at its active (low) level in `levels` */
void checkOnlyCsLow(const std::vector<bool> &levels, size_t idx) {
	REQUIRE(levels.size() > idx);
	for (size_t i = 0; i < levels.size(); i++) {
		CHECK_MESSAGE(levels[i] == (i != idx), "CS index " << i << " level (true = high)");
	}
}

} // namespace

TEST_SUITE("TMC4671 SPI transport") {

// ---------------------------------------------------------------- R1

TEST_CASE_FIXTURE(TmcFixture, "[R1] a register read of TMC_1 is one 5 byte transfer with address byte, zeros and only PA4 low during the frame") {
	auto tmc = makeTmc1();
	HostTarget::spiClearTransfers(&hspi1);
	tmc->readReg(0x6A);
	const auto &transfers = HostTarget::spiTransfers(&hspi1);
	REQUIRE(transfers.size() == 1);
	const SpiTransfer &t = transfers.front();
	CHECK(t.size == 5);
	// DS §4.2.1 p17: WRnRD = 0 for a read, address 0x6A in bits 38..32, data bits 0 -> bytes {0x6A, 0, 0, 0, 0}
	CHECK(t.tx == std::vector<uint8_t>{0x6A, 0, 0, 0, 0});
	CHECK(t.selectedCs() == 0); // index 0 = PA4 = TMC_1
	checkOnlyCsLow(t.csAtStart, 0);
	checkOnlyCsLow(t.csAtEnd, 0);
	for (bool level : HostTarget::spiCsLevels(&hspi1)) {
		CHECK(level); // all CS lines high again after the frame
	}
}

TEST_CASE_FIXTURE(TmcFixture, "[R1] a register read of TMC_2 selects only PB0 (CS index 1)") {
	auto tmc = makeTmc2();
	HostTarget::spiClearTransfers(&hspi1);
	tmc->readReg(0x6A);
	const auto &transfers = HostTarget::spiTransfers(&hspi1);
	REQUIRE(transfers.size() == 1);
	const SpiTransfer &t = transfers.front();
	CHECK(t.size == 5);
	CHECK(t.tx == std::vector<uint8_t>{0x6A, 0, 0, 0, 0});
	CHECK(t.selectedCs() == 1); // index 1 = PB0 = TMC_2
	checkOnlyCsLow(t.csAtStart, 1);
	checkOnlyCsLow(t.csAtEnd, 1);
	for (bool level : HostTarget::spiCsLevels(&hspi1)) {
		CHECK(level);
	}
	CHECK(chip2.readsOf(0x6A) == 1);
	CHECK(chip1.readsOf(0x6A) == 0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R1] a read of the highest address 0x7F keeps bit 7 of the address byte clear") {
	auto tmc = makeTmc1();
	HostTarget::spiClearTransfers(&hspi1);
	tmc->readReg(0x7F);
	REQUIRE(HostTarget::spiTransfers(&hspi1).size() == 1);
	// DS §4.2.1 p17: the address is 7 bit (0..127); bit 7 of byte 0 is WRnRD and must be 0 for a read
	CHECK(HostTarget::spiTransfers(&hspi1).front().tx[0] == 0x7F);
}

// ---------------------------------------------------------------- R2

TEST_CASE_FIXTURE(TmcFixture, "[R2] the value of a read is MISO bytes 1 to 4 of the same frame, most significant byte first") {
	auto tmc = makeTmc1();
	chip1.setReg(0x6A, 0x12345678); // PID_VELOCITY_ACTUAL
	// DS §4.2.1 p17: answer inside the same datagram, MSB first -> 0x12345678 arrives as 12 34 56 78 in bytes 1..4
	CHECK(tmc->getVelocity() == 0x12345678);
	const SpiTransfer &t = HostTarget::spiTransfers(&hspi1).back();
	REQUIRE(t.rx.size() == 5);
	CHECK(t.rx[1] == 0x12);
	CHECK(t.rx[2] == 0x34);
	CHECK(t.rx[3] == 0x56);
	CHECK(t.rx[4] == 0x78);
}

TEST_CASE_FIXTURE(TmcFixture, "[R2] a read with the top bit set returns the full 32 bit value") {
	auto tmc = makeTmc1();
	chip1.setReg(0x6A, 0x80000001);
	// HD: 0x80000001 as int32 = -2147483647 (two's complement). int32_t is 32 bit on host and target.
	CHECK(tmc->getVelocity() == -2147483647);
	CHECK(tmc->readReg(0x6A) == 0x80000001u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R2] a read is not answered from the neighbouring address of the previous frame") {
	auto tmc = makeTmc1();
	chip1.setReg(0x66, 0x11111111); // target velocity
	chip1.setReg(0x6A, 0x22222222); // actual velocity
	// DS §4.2.1 p17: no pipelining, each datagram answers its own address
	CHECK(tmc->readReg(0x66) == 0x11111111u);
	CHECK(tmc->readReg(0x6A) == 0x22222222u);
	CHECK(tmc->readReg(0x66) == 0x11111111u);
}

// ---------------------------------------------------------------- R3

TEST_CASE_FIXTURE(TmcFixture, "[R3] a register write is one 5 byte transfer: 0x80 or address, then the data MSB first") {
	auto tmc = makeTmc1();
	HostTarget::spiClearTransfers(&hspi1);
	tmc->writeReg(0x66, 0xA1B2C3D4);
	const auto &transfers = HostTarget::spiTransfers(&hspi1);
	REQUIRE(transfers.size() == 1);
	const SpiTransfer &t = transfers.front();
	CHECK(t.size == 5);
	// DS §4.2.1 p17: WRnRD = 1 (bit 39) | address 0x66 -> 0xE6; data 0xA1B2C3D4 MSB first
	CHECK(t.tx == std::vector<uint8_t>{0xE6, 0xA1, 0xB2, 0xC3, 0xD4});
	CHECK(chip1.reg(0x66) == 0xA1B2C3D4u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R3] a write to address 0x7F sets only the WRnRD bit above the 7 bit address") {
	auto tmc = makeTmc1();
	HostTarget::spiClearTransfers(&hspi1);
	tmc->writeReg(0x7D, 0x00000001); // STATUS_MASK: writable, address with all low bits set
	REQUIRE(HostTarget::spiTransfers(&hspi1).size() == 1);
	// HD: 0x80 | 0x7D = 0xFD
	CHECK(HostTarget::spiTransfers(&hspi1).front().tx == std::vector<uint8_t>{0xFD, 0x00, 0x00, 0x00, 0x01});
}

TEST_CASE_FIXTURE(TmcFixture, "[R3] a write of TMC_1 holds only PA4 low for the whole frame and releases it afterwards") {
	auto tmc = makeTmc1();
	HostTarget::spiClearTransfers(&hspi1);
	tmc->writeReg(0x64, 0x00000000);
	REQUIRE(HostTarget::spiTransfers(&hspi1).size() == 1);
	const SpiTransfer &t = HostTarget::spiTransfers(&hspi1).front();
	checkOnlyCsLow(t.csAtStart, 0);
	checkOnlyCsLow(t.csAtEnd, 0);
	for (bool level : HostTarget::spiCsLevels(&hspi1)) {
		CHECK(level);
	}
}

TEST_CASE_FIXTURE(TmcFixture, "[R3] a write of TMC_2 holds only PB0 low for the whole frame and writes only its own chip") {
	auto tmc = makeTmc2();
	HostTarget::spiClearTransfers(&hspi1);
	tmc->writeReg(0x64, 0x0000ABCD);
	REQUIRE(HostTarget::spiTransfers(&hspi1).size() == 1);
	const SpiTransfer &t = HostTarget::spiTransfers(&hspi1).front();
	checkOnlyCsLow(t.csAtStart, 1);
	checkOnlyCsLow(t.csAtEnd, 1);
	CHECK(chip2.reg(0x64) == 0x0000ABCDu);
	CHECK(chip1.reg(0x64) == 0u);
}

// ---------------------------------------------------------------- R4

TEST_CASE_FIXTURE(TmcFixture, "[R4] the SPI port is configured for mode 3, 8 bit, MSB first, software NSS, master, two lines") {
	auto tmc = makeTmc1();
	const SPI_InitTypeDef &init = hspi1.Init;
	// DS §4.2.1 p17: SPI mode 3 = CPOL 1 (clock idles high), CPHA 1 (second edge samples)
	CHECK(init.CLKPolarity == SPI_POLARITY_HIGH);
	CHECK(init.CLKPhase == SPI_PHASE_2EDGE);
	CHECK(init.DataSize == SPI_DATASIZE_8BIT);
	CHECK(init.FirstBit == SPI_FIRSTBIT_MSB);
	CHECK(init.NSS == SPI_NSS_SOFT);
	CHECK(init.Mode == SPI_MODE_MASTER);
	CHECK(init.Direction == SPI_DIRECTION_2LINES);
	CHECK(init.TIMode == SPI_TIMODE_DISABLE);
	CHECK(init.CRCCalculation == SPI_CRCCALCULATION_DISABLE);
}

TEST_CASE_FIXTURE(TmcFixture, "[R4] the SCK of a register frame does not exceed 8 MHz and is not zero") {
	auto tmc = makeTmc1();
	HostTarget::spiClearTransfers(&hspi1);
	tmc->writeReg(0x64, 0);
	REQUIRE(HostTarget::spiTransfers(&hspi1).size() == 1);
	const double sck = sckHz(HostTarget::spiTransfers(&hspi1).front().init.BaudRatePrescaler);
	// DS §4.2.1 Table 2 p18: write clock up to 8 MHz. RM0090: fSCK = 84 MHz / 2^(BR+1)
	CHECK(sck <= 8e6);
	CHECK(sck > 0.0);
}

TEST_CASE_FIXTURE(TmcFixture, "[R4] the frames are clocked with the mode 3 configuration, not only the port handle after construction") {
	auto tmc = makeTmc1();
	HostTarget::spiClearTransfers(&hspi1);
	tmc->readReg(0x6A);
	REQUIRE(HostTarget::spiTransfers(&hspi1).size() == 1);
	const SPI_InitTypeDef &init = HostTarget::spiTransfers(&hspi1).front().init;
	CHECK(init.CLKPolarity == SPI_POLARITY_HIGH);
	CHECK(init.CLKPhase == SPI_PHASE_2EDGE);
	CHECK(init.DataSize == SPI_DATASIZE_8BIT);
	CHECK(init.FirstBit == SPI_FIRSTBIT_MSB);
}

// ---------------------------------------------------------------- R5

TEST_CASE_FIXTURE(TmcFixture, "[R5] a register read respects the read timing: SCK at most 2 MHz or the address byte sent as its own transfer"
							  * KNOWN_ISSUE("B-T1: SCK is 5.25 MHz (84 MHz / 16) and the read is one 5 byte transfer; TMC4671.cpp:2533 only has the comment '500ns delay after sending first byte'")) {
	auto tmc = makeTmc1();
	HostTarget::spiClearTransfers(&hspi1);
	tmc->readReg(0x6A);
	const auto &transfers = HostTarget::spiTransfers(&hspi1);
	REQUIRE(transfers.size() >= 1);
	// DS §4.2.1 Table 2 + info box p18-19: reads above 2 MHz need a pause of >= 500 ns after the address byte.
	// Lead decision L5: the pause is not observable in simulated time; accepted proxy = SCK <= 2 MHz, or the address
	// byte is a separate transfer before the 4 data bytes.
	const double sck = sckHz(transfers.front().init.BaudRatePrescaler);
	const bool addressByteSeparate = transfers.size() >= 2 && transfers.front().size == 1;
	CHECK((sck <= 2e6 || addressByteSeparate));
}

// ---------------------------------------------------------------- R6

TEST_CASE_FIXTURE(TmcFixture, "[R6] updateReg changes only the addressed one bit field and keeps all other bits as read from the chip") {
	auto tmc = makeTmc1();
	chip1.setReg(0x1A, 0xA5C3F00F); // bit 8 is 0 (nibble 0xF0 0F: bits 11..8 = 0)
	tmc->updateReg(0x1A, 1, 0x01, 8);
	// HD: 0xA5C3F00F | (1 << 8) = 0xA5C3F10F
	CHECK(chip1.reg(0x1A) == 0xA5C3F10Fu);
}

TEST_CASE_FIXTURE(TmcFixture, "[R6] updateReg clears the addressed bit without touching its neighbours") {
	auto tmc = makeTmc1();
	chip1.setReg(0x1A, 0xFFFFFFFF);
	tmc->updateReg(0x1A, 0, 0x01, 8);
	// HD: 0xFFFFFFFF & ~(1 << 8) = 0xFFFFFEFF
	CHECK(chip1.reg(0x1A) == 0xFFFFFEFFu);
}

TEST_CASE_FIXTURE(TmcFixture, "[R6] updateReg replaces a multi bit field at its shift and masks the new value") {
	auto tmc = makeTmc1();
	chip1.setReg(0x25, 0x12345678); // ABN_DECODER_MODE (read/write register)
	tmc->updateReg(0x25, 0xFD, 0x7, 4); // mask 0x7 << 4 = 0x70; 0xFD & 0x7 = 5
	// HD: 0x12345678 & ~0x70 = 0x12345608; | (5 << 4 = 0x50) = 0x12345658
	CHECK(chip1.reg(0x25) == 0x12345658u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R6] updateReg is one read frame followed by one write frame of the same address") {
	auto tmc = makeTmc1();
	chip1.setReg(0x1A, 0x00000007);
	chip1.clearLog();
	tmc->updateReg(0x1A, 1, 0x01, 8);
	const auto &log = chip1.log();
	REQUIRE(log.size() == 2);
	CHECK_FALSE(log[0].write);
	CHECK(log[0].addr == 0x1A);
	CHECK(log[1].write);
	CHECK(log[1].addr == 0x1A);
	CHECK(log[1].data == 0x00000107u); // HD: 0x07 | 0x100
}

TEST_CASE_FIXTURE(TmcFixture, "[R6] the svpwm command of a BLDC motor changes only bit 8 of PWM_CHOP_SV 0x1A") {
	auto tmc = makeTmc1();
	tmc->setMotorType(MotorType::BLDC, 7); // BLDC needed: setSvPwm forces the bit off for other motor types
	REQUIRE(CommandHelpers::get(*tmc, "mtype") == 3); // BLDC accepted by the hardware config
	chip1.setReg(0x1A, 0x000000F7); // bit 8 = 0, low byte 0xF7 must survive
	CHECK(CommandHelpers::run(*tmc, "svpwm", CMDtype::set, 1) == CommandStatus::OK);
	// DS 0x1A p84: PWM_SV is bit 8, PWM_CHOP [7:0]. HD: 0xF7 | 0x100 = 0x1F7
	CHECK(chip1.reg(0x1A) == 0x000001F7u);
}

// ---------------------------------------------------------------- R7

TEST_CASE_FIXTURE(TmcFixture, "[R7] writeRegAsync starts one 5 byte interrupt transfer with the write frame and PA4 low") {
	auto tmc = makeTmc1();
	HostTarget::spiClearTransfers(&hspi1);
	tmc->writeRegAsync(0x1C, 0x00001234); // PHI_E_EXT
	REQUIRE(HostTarget::spiTransferPending(&hspi1));
	const auto &transfers = HostTarget::spiTransfers(&hspi1);
	REQUIRE(transfers.size() == 1);
	CHECK(transfers.front().size == 5);
	checkOnlyCsLow(transfers.front().csAtStart, 0);
	// CS stays low while the transfer is pending (DS §4.2.1 p17: CS low for the whole datagram)
	std::vector<bool> levels = HostTarget::spiCsLevels(&hspi1);
	checkOnlyCsLow(levels, 0);
	CHECK(HostTarget::spiComplete(&hspi1));
	REQUIRE(HostTarget::spiTransfers(&hspi1).size() == 1);
	// DS §4.2.1 p17: 0x80 | 0x1C = 0x9C, data 0x00001234 MSB first
	CHECK(HostTarget::spiTransfers(&hspi1).front().tx == std::vector<uint8_t>{0x9C, 0x00, 0x00, 0x12, 0x34});
	CHECK(chip1.reg(0x1C) == 0x00001234u);
}

TEST_CASE_FIXTURE(TmcFixture, "[R7] the transfer complete callback releases CS and the port so the next register access proceeds") {
	auto tmc = makeTmc1();
	tmc->writeRegAsync(0x1C, 0x00000042);
	REQUIRE(HostTarget::spiTransferPending(&hspi1));
	CHECK(HostTarget::spiComplete(&hspi1));
	CHECK_FALSE(HostTarget::spiTransferPending(&hspi1));
	for (bool level : HostTarget::spiCsLevels(&hspi1)) {
		CHECK(level); // CS released by the callback
	}
	// The port semaphore was given back: a blocking access works (it would block forever otherwise)
	chip1.setReg(0x6A, 0x0BADF00D);
	CHECK(tmc->readReg(0x6A) == 0x0BADF00Du);
}

} // TEST_SUITE
