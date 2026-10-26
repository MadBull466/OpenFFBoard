/*
 * Tests of EncoderSSI (UserExtensions/Src/EncoderSSI.cpp): absolute SSI encoder read with DMA from the encoder SPI port
 * ENCODER_SPI_PORT (ext3_spi, hspi3, first CS pin SPI3_SS1, active low) (ffb_target_tests).
 *
 * Modes: AMT23 (CUI AMT22/23 protocol: 2 check bits + 14 bit position, 12 bit devices send 2 trailing zeros) and raw
 * MSB first. Configuration in flash: ADR_SSI_CONF1 = (bits - 1) | speed << 5 | mode << 8, ADR_SSI_OFS = position offset
 * >> max(0, bits - 16).
 *
 * The mode member is not initialised without a stored configuration (suspected bug 77): every test stores
 * ADR_SSI_CONF1 or sets the mode before the mode is read.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "MockSPIDevice.h"
#include "TestHelpers.h"

#include "EncoderSSI.h"
#include "cpp_target_config.h"
#include "eeprom_addresses.h"
#include "flash_helpers.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include <string>
#include <vector>

namespace {

constexpr uint8_t MODE_RAW = 0;
constexpr uint8_t MODE_AMT23 = 1;

/**
 * AMT23 (CUI AMT22/23 SSI protocol) response: 2 check bits and the 14 bit position, MSB first. K1 = odd parity over
 * the odd position bits 13, 11, ..., 1; K0 = odd parity over the even bits 12, 10, ..., 0 (the XOR of all bits of
 * each group including its check bit is 1). Independent reference (same as test_encoder_setup.cpp).
 */
uint16_t amt23Frame(uint16_t position) {
	position &= 0x3FFF;
	uint8_t odd = 0, even = 0;
	for (int bit = 0; bit < 14; bit++) {
		uint8_t b = (position >> bit) & 1U;
		if (bit % 2) {
			odd ^= b;
		} else {
			even ^= b;
		}
	}
	const uint16_t k1 = odd ^ 1U, k0 = even ^ 1U;
	return static_cast<uint16_t>((k1 << 15) | (k0 << 14) | position);
}

/**
 * Same port and pin. (GpioPin::operator== is a non-const member: inside doctest's expression decomposition
 * `CHECK(a == b)` does not reach it, so pins are compared explicitly)
 */
bool samePin(const OutputPin &a, const OutputPin &b) { return a.getPort() == b.getPort() && a.getPin() == b.getPin(); }

struct SsiFixture : TargetFixture {
	/** Stores the configuration word like EncoderSSI::saveFlash (bits - 1 in bits 0..4, speed 5..6, mode 8..11) */
	static void storeConfig(int bits, int speed, uint8_t mode) {
		REQUIRE(Flash_Write(ADR_SSI_CONF1, static_cast<uint16_t>(((bits - 1) & 0x1F) | ((speed & 0x3) << 5) | ((mode & 0xF) << 8))));
	}

	static CommandStatus cmd(EncoderSSI &enc, const std::string &name, CMDtype type, int64_t val = 0,
							 std::vector<CommandReply> *replies = nullptr) {
		CmdHandlerCommanddef *def = enc.getCommandFromName(name);
		REQUIRE_MESSAGE(def != nullptr, "command " << name << " not registered");
		ParsedCommand parsed;
		parsed.cmdId = def->cmdId;
		parsed.type = type;
		parsed.val = val;
		parsed.target = &enc;
		std::vector<CommandReply> local;
		return enc.command(parsed, replies ? *replies : local);
	}
	static int64_t get(EncoderSSI &enc, const std::string &name) {
		std::vector<CommandReply> replies;
		CHECK(cmd(enc, name, CMDtype::get, 0, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies.front().val;
	}
	static std::string info(EncoderSSI &enc, const std::string &name) {
		std::vector<CommandReply> replies;
		CHECK(cmd(enc, name, CMDtype::info, 0, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies.front().reply;
	}

	/** Delivers one frame: starts a read if none is pending (getPosAbs), then completes it with these MISO bytes */
	static void feed(EncoderSSI &enc, const std::vector<uint8_t> &bytes) {
		if (!HostTarget::spiTransferPending(&hspi3)) {
			enc.getPosAbs();
		}
		REQUIRE(HostTarget::spiTransferPending(&hspi3));
		HostTarget::spiQueueRx(&hspi3, bytes);
		REQUIRE(HostTarget::spiComplete(&hspi3));
	}
	/**
	 * Completes a read that is still pending (MISO idle 0x00). Call before an encoder is deleted inside a test: deleting it
	 * with a pending DMA read leaves the port pointing at it and the DMA writing into its buffer (bug 57 pattern)
	 */
	static void drain() {
		if (HostTarget::spiTransferPending(&hspi3)) {
			REQUIRE(HostTarget::spiComplete(&hspi3));
		}
	}
	static void feedAmt23(EncoderSSI &enc, uint16_t position) { feed(enc, HostTarget::spiFrameBytes(amt23Frame(position), 2)); }

	/** Size of the last read started on the encoder port */
	static uint16_t lastReadSize() {
		const auto &transfers = HostTarget::spiTransfers(&hspi3);
		REQUIRE_FALSE(transfers.empty());
		return transfers.back().size;
	}
};

} // namespace

TEST_SUITE("EncoderSSI") {

// ---------------- Identity and creation ----------------

TEST_CASE_FIXTURE(SsiFixture, "the SSI encoder is an absolute encoder named SSI with the SSI class id and the command handler ssienc") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	CHECK(std::string(enc.getInfo().name) == "SSI");
	CHECK(enc.getInfo().id == CLSID_ENCODER_SSI);
	CHECK(enc.getEncoderType() == EncoderType::absolute);
	CHECK(enc.getHelpstring() == "SPI SSI Encoder\n");
	CHECK(CommandHandler::getHandlerFromClassName("ssienc") == &enc);
}

TEST_CASE_FIXTURE(SsiFixture, "only one SSI encoder can be created at a time") {
	storeConfig(14, 0, MODE_AMT23);
	REQUIRE(EncoderSSI::isCreatable());
	auto enc = std::make_unique<EncoderSSI>();
	CHECK_FALSE(EncoderSSI::isCreatable());
	enc.reset();
	CHECK(EncoderSSI::isCreatable());
}

TEST_CASE_FIXTURE(SsiFixture, "an SSI encoder is not creatable while another device uses the encoder SPI port exclusively") {
	ext3_spi.takeExclusive(true);
	CHECK_FALSE(EncoderSSI::isCreatable());
	ext3_spi.takeExclusive(false);
	CHECK(EncoderSSI::isCreatable());
}

TEST_CASE_FIXTURE(SsiFixture, "the SSI encoder reserves the first CS pin and takes the encoder SPI port exclusively") {
	storeConfig(14, 0, MODE_AMT23);
	const OutputPin cs = ext3_spi.getCsPins()[0];
	EncoderSSI enc;
	CHECK_FALSE(ext3_spi.isPinFree(cs));
	CHECK(samePin(enc.getSpiConfig()->cs, cs));
	CHECK(ext3_spi.isPinFree(ext3_spi.getCsPins()[1]));
	CHECK_FALSE(ext3_spi.hasFreePins()); // exclusive
}

TEST_CASE_FIXTURE(SsiFixture, "an SSI encoder is not created on a CS pin another device holds" *
				  KNOWN_ISSUE("EncoderSSI::isCreatable (EncoderSSI.cpp:93) uses hasFreePins, which counts all CS pins of "
							  "the port (SPI.cpp:220-222), and the constructor (EncoderSSI.cpp:23) always takes getCsPins()[0] "
							  "even if it is reserved (reserveCsPin fails silently): the encoder and the other device drive "
							  "one CS pin (bug 67 pattern)")) {
	MockSPIDevice other(ext3_spi, ext3_spi.getCsPins()[0]);
	storeConfig(14, 0, MODE_AMT23);
	if (EncoderSSI::isCreatable()) {
		EncoderSSI enc;
		const bool shared = samePin(enc.getSpiConfig()->cs, other.config().cs);
		REQUIRE_ALL_HAL_MODELLED();
		CHECK_FALSE(shared);
	}
}

TEST_CASE_FIXTURE(SsiFixture, "deleting an SSI encoder keeps the CS pin of another device reserved" *
				  KNOWN_ISSUE("EncoderSSI.cpp:23 takes CS pin 0 although another device holds it; ~SPIDevice "
							  "(SPI.cpp:338-340) then frees the other device's pin")) {
	MockSPIDevice other(ext3_spi, ext3_spi.getCsPins()[0]);
	storeConfig(14, 0, MODE_AMT23);
	if (EncoderSSI::isCreatable()) {
		auto enc = std::make_unique<EncoderSSI>();
		enc.reset();
	}
	REQUIRE_ALL_HAL_MODELLED();
	CHECK_FALSE(ext3_spi.isPinFree(other.config().cs));
}

TEST_CASE_FIXTURE(SsiFixture, "deleting the SSI encoder frees its CS pin and the encoder SPI port") {
	storeConfig(14, 0, MODE_AMT23);
	const OutputPin cs = ext3_spi.getCsPins()[0];
	auto enc = std::make_unique<EncoderSSI>();
	feedAmt23(*enc, 0x1000);
	enc.reset();
	CHECK(ext3_spi.isPinFree(cs));
	CHECK(ext3_spi.hasFreePins());
	CHECK(CommandHandler::getHandlerFromClassName("ssienc") == nullptr);
}

TEST_CASE_FIXTURE(SsiFixture, "the mode of an SSI encoder created without a stored configuration is a defined mode" *
				  KNOWN_ISSUE_UB("EncoderSSI.h:73 `EncoderSSI_modes mode;` has no initialiser and the constructor sets it only "
								 "from a stored ADR_SSI_CONF1 (EncoderSSI.cpp:43-47): without flash the mode is indeterminate "
								 "(spiRxCompleted / bits / mode read it). The object is built on storage filled with 0xAA to "
								 "make the indeterminate value visible")) {
	alignas(EncoderSSI) unsigned char storage[sizeof(EncoderSSI)];
	std::memset(storage, 0xAA, sizeof(storage));
	EncoderSSI *enc = new (storage) EncoderSSI();
	std::vector<CommandReply> replies;
	cmd(*enc, "mode", CMDtype::get, 0, &replies);
	REQUIRE_ALL_HAL_MODELLED();
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val <= MODE_AMT23);
	enc->~EncoderSSI();
}

// ---------------- SPI configuration and reads ----------------

TEST_CASE_FIXTURE(SsiFixture, "the SSI encoder reads with SPI mode 1, MSB first, 8 bit and CS active low") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	enc.getPosAbs();
	REQUIRE(HostTarget::spiTransferPending(&hspi3));
	const HostTarget::SpiTransfer &read = HostTarget::spiTransfers(&hspi3).front();
	CHECK(read.call == HostTarget::SpiCall::Receive);
	CHECK(read.mode == HostTarget::BusMode::Dma);
	CHECK(read.init.CLKPolarity == SPI_POLARITY_LOW);
	CHECK(read.init.CLKPhase == SPI_PHASE_2EDGE);
	CHECK(read.init.FirstBit == SPI_FIRSTBIT_MSB);
	CHECK(read.init.DataSize == SPI_DATASIZE_8BIT);
	REQUIRE(read.csAtStart.size() == 3);
	CHECK_FALSE(read.csAtStart[0]); // SPI3_SS1 asserted (low)
	HostTarget::spiQueueRx(&hspi3, HostTarget::spiFrameBytes(amt23Frame(0), 2));
	REQUIRE(HostTarget::spiComplete(&hspi3));
	CHECK(HostTarget::spiCsLevels(&hspi3)[0]); // released (high) by the completion
}

TEST_CASE_FIXTURE(SsiFixture, "the speed presets 0, 1 and 2 select the SPI prescalers 32, 16 and 8") {
	const uint32_t expected[3] = {SPI_BAUDRATEPRESCALER_32, SPI_BAUDRATEPRESCALER_16, SPI_BAUDRATEPRESCALER_8};
	for (int speed = 0; speed < 3; speed++) {
		CAPTURE(speed);
		storeConfig(14, speed, MODE_AMT23);
		EncoderSSI enc;
		CHECK(get(enc, "speed") == speed);
		CHECK(hspi3.Init.BaudRatePrescaler == expected[speed]);
		enc.getPosAbs();
		CHECK(HostTarget::spiTransfers(&hspi3).back().init.BaudRatePrescaler == expected[speed]);
		HostTarget::spiQueueRx(&hspi3, HostTarget::spiFrameBytes(amt23Frame(0), 2));
		REQUIRE(HostTarget::spiComplete(&hspi3));
	}
}

TEST_CASE_FIXTURE(SsiFixture, "a stored speed preset 3 uses the slowest SPI clock") {
	storeConfig(14, 3, MODE_AMT23);
	EncoderSSI enc;
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_32);
}

TEST_CASE_FIXTURE(SsiFixture, "the AMT23 mode reads 2 bytes per frame") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	enc.getPosAbs();
	CHECK(lastReadSize() == 2);
}

TEST_CASE_FIXTURE(SsiFixture, "the raw mode reads as many bytes as the resolution needs") {
	// Up to 30 bits: with 31 and 32 bits the decoding mask `(1<<lenghtDataBit) - 1` and getCpr overflow (bug 77)
	const int bits[] = {1, 8, 9, 16, 17, 22, 24, 25, 30};
	const uint16_t sizes[] = {1, 1, 2, 2, 3, 3, 3, 4, 4};
	for (size_t i = 0; i < sizeof(bits) / sizeof(bits[0]); i++) {
		CAPTURE(bits[i]);
		storeConfig(bits[i], 0, MODE_RAW);
		EncoderSSI enc;
		enc.getPosAbs();
		CHECK(lastReadSize() == sizes[i]);
		REQUIRE(HostTarget::spiComplete(&hspi3));
	}
}

TEST_CASE_FIXTURE(SsiFixture, "a position request while a frame is being read does not start another read") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	feedAmt23(enc, 0x2100);
	enc.getPosAbs(); // starts the next read
	const size_t reads = HostTarget::spiTransfers(&hspi3).size();
	CHECK(enc.getPosAbs() == 0x100); // last result
	CHECK(HostTarget::spiTransfers(&hspi3).size() == reads);
}

TEST_CASE_FIXTURE(SsiFixture, "every position request after a completed frame starts the next read") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	feedAmt23(enc, 0x2000);
	const size_t reads = HostTarget::spiTransfers(&hspi3).size();
	enc.getPosAbs();
	CHECK(HostTarget::spiTransfers(&hspi3).size() == reads + 1);
	CHECK(HostTarget::spiTransferPending(&hspi3));
}

// ---------------- AMT23 decoding ----------------

TEST_CASE_FIXTURE(SsiFixture, "an AMT23 frame gives the 14 bit position centred around half a turn") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	CHECK(enc.getCpr() == 16384);
	feedAmt23(enc, 0x2000);
	CHECK(enc.getPosAbs() == 0);
	feedAmt23(enc, 0x2ABC);
	CHECK(enc.getPosAbs() == 0x0ABC);
	feedAmt23(enc, 0x1555);
	CHECK(enc.getPosAbs() == 0x1555 - 0x2000);
	CHECK(get(enc, "errors") == 0);
}

TEST_CASE_FIXTURE(SsiFixture, "a 12 bit AMT23 frame gives the 12 bit position without the 2 trailing bits") {
	storeConfig(12, 0, MODE_AMT23);
	EncoderSSI enc;
	CHECK(enc.getCpr() == 4096);
	feedAmt23(enc, 0xABC << 2); // 12 bit position, 2 zero bits
	CHECK(enc.getPosAbs() == 0xABC - 0x800);
	CHECK(get(enc, "errors") == 0);
}

TEST_CASE_FIXTURE(SsiFixture, "an AMT23 frame with both check bits wrong is counted as an error and keeps the position") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	feedAmt23(enc, 0x2100);
	REQUIRE(enc.getPosAbs() == 0x100);
	feed(enc, HostTarget::spiFrameBytes(amt23Frame(0x3000) ^ 0xC000, 2));
	CHECK(enc.getPosAbs() == 0x100);
	CHECK(get(enc, "errors") == 1);
}

TEST_CASE_FIXTURE(SsiFixture, "an AMT23 frame with one wrong check bit is counted as an error and keeps the position" *
				  KNOWN_ISSUE("EncoderSSI.cpp:110 accepts a frame if `parity` is not 0: both check bits must be right "
							  "(parity == 3), a frame with only K1 or only K0 right is decoded as a position")) {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	feedAmt23(enc, 0x2100);
	REQUIRE(enc.getPosAbs() == 0x100);
	feed(enc, HostTarget::spiFrameBytes(amt23Frame(0x3000) ^ 0x4000, 2)); // K0 wrong
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(enc.getPosAbs() == 0x100);
	CHECK(get(enc, "errors") == 1);
}

TEST_CASE_FIXTURE(SsiFixture, "crossing the zero point of the AMT23 encoder forwards continues the position in the next turn") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	feedAmt23(enc, 0x3FF0); // 8176
	REQUIRE(enc.getPosAbs() == 8176);
	feedAmt23(enc, 0x0010); // 32 counts later, after the wrap
	CHECK(enc.getPosAbs() == 8176 + 32);
}

TEST_CASE_FIXTURE(SsiFixture, "crossing the zero point of the AMT23 encoder backwards continues the position in the previous turn") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	feedAmt23(enc, 0x0010); // -8176
	REQUIRE(enc.getPosAbs() == -8176);
	feedAmt23(enc, 0x3FF0);
	CHECK(enc.getPosAbs() == -8176 - 32);
}

TEST_CASE_FIXTURE(SsiFixture, "a move of less than half a turn is not counted as a turn") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	feedAmt23(enc, 0x0100);
	feedAmt23(enc, 0x2000); // + 7936
	CHECK(enc.getPosAbs() == 0);
	feedAmt23(enc, 0x0100); // back
	CHECK(enc.getPosAbs() == 0x0100 - 0x2000);
}

// ---------------- Raw decoding ----------------

TEST_CASE_FIXTURE(SsiFixture, "a raw frame one count higher gives a position one count higher" *
				  KNOWN_ISSUE("EncoderSSI.cpp:123-124 ORs every byte shifted by `bytes` (8) and loops over all 8 buffer "
							  "bytes instead of shifting the result MSB first over transferlen bytes: 0x1234 and 0x1235 "
							  "decode as 0x3600 and 0x3700 (16 bit)")) {
	storeConfig(16, 0, MODE_RAW);
	EncoderSSI enc;
	feed(enc, {0x12, 0x34});
	const int32_t first = enc.getPosAbs();
	feed(enc, {0x12, 0x35});
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(enc.getPosAbs() - first == 1);
}

TEST_CASE_FIXTURE(SsiFixture, "changing the resolution in raw mode changes the number of bytes read" *
				  KNOWN_ISSUE("the bits command (EncoderSSI.cpp:211-216) only stores lenghtDataBit; transferlen is computed "
							  "by setMode only (EncoderSSI.cpp:190-197), so the read length stays that of the old resolution")) {
	storeConfig(8, 0, MODE_RAW);
	EncoderSSI enc;
	REQUIRE(cmd(enc, "bits", CMDtype::set, 16) == CommandStatus::OK);
	enc.getPosAbs();
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(lastReadSize() == 2);
}

TEST_CASE_FIXTURE(SsiFixture, "the counts per turn of a raw SSI encoder are 2 to the power of its resolution") {
	storeConfig(22, 0, MODE_RAW);
	EncoderSSI enc;
	CHECK(enc.getCpr() == (1U << 22));
}

// ---------------- Offset ----------------

TEST_CASE_FIXTURE(SsiFixture, "setPos sets the current position and the position follows the encoder from it") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	feedAmt23(enc, 0x2100); // 256
	enc.setPos(1000);
	CHECK(enc.getPos() == 1000);
	feedAmt23(enc, 0x2180); // + 128
	CHECK(enc.getPos() == 1128);
	CHECK(enc.getPosAbs() == 0x180); // the absolute position has no offset
}

TEST_CASE_FIXTURE(SsiFixture, "characterization: setPos after a completed turn ignores the turn") {
	// NEEDS REVIEW 97: setPos uses the single-turn position only (EncoderSSI.cpp:173 `posOffset = pos - newpos`) while
	// getPos includes the turns (line 164): after one turn forwards setPos(0) gives getPos() == cpr, not 0.
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	feedAmt23(enc, 0x3FF0);
	feedAmt23(enc, 0x0010); // one wrap forwards
	enc.setPos(0);
	CHECK(enc.getPos() == 16384);
}

// ---------------- Flash ----------------

TEST_CASE_FIXTURE(SsiFixture, "saveFlash stores the resolution, speed, mode and offset") {
	storeConfig(12, 0, MODE_RAW);
	EncoderSSI enc;
	REQUIRE(cmd(enc, "mode", CMDtype::set, MODE_AMT23) == CommandStatus::OK);
	REQUIRE(cmd(enc, "bits", CMDtype::set, 14) == CommandStatus::OK);
	REQUIRE(cmd(enc, "speed", CMDtype::set, 2) == CommandStatus::OK);
	feedAmt23(enc, 0x2100);
	enc.setPos(-44); // offset 256 + 44
	enc.saveFlash();
	uint16_t conf = 0, ofs = 0;
	REQUIRE(Flash_Read(ADR_SSI_CONF1, &conf));
	REQUIRE(Flash_Read(ADR_SSI_OFS, &ofs));
	CHECK(conf == (13 | (2 << 5) | (1 << 8)));
	CHECK(ofs == 300);
}

TEST_CASE_FIXTURE(SsiFixture, "a new SSI encoder restores the saved configuration and position offset") {
	{
		storeConfig(14, 0, MODE_AMT23);
		EncoderSSI first;
		REQUIRE(cmd(first, "speed", CMDtype::set, 1) == CommandStatus::OK);
		feedAmt23(first, 0x2100);
		first.setPos(0);
		first.saveFlash();
		drain();
	}
	EncoderSSI second;
	CHECK(get(second, "mode") == MODE_AMT23);
	CHECK(get(second, "bits") == 14);
	CHECK(get(second, "speed") == 1);
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_16);
	feedAmt23(second, 0x2100);
	CHECK(second.getPos() == 0);
}

TEST_CASE_FIXTURE(SsiFixture, "characterization: an offset of a resolution above 16 bits is restored in steps of 2^(bits-16) counts") {
	// NEEDS REVIEW 96 (part 1): the offset is stored as offset >> (bits - 16) (EncoderSSI.cpp:58, 48): with 22 bits it is
	// restored rounded down to a multiple of 64 counts (1000 -> 960). Intended to keep 16 bit flash words?
	{
		storeConfig(22, 0, MODE_AMT23); // AMT23 decoding with 22 bit bookkeeping: frame 0x2000 + 1000 -> position 1000 - 2^21 + 0x2000
		EncoderSSI first;
		feedAmt23(first, 0x2000);
		const int32_t abs = first.getPosAbs();
		first.setPos(abs - 1000); // offset 1000
		first.saveFlash();
		drain();
	}
	uint16_t ofs = 0;
	REQUIRE(Flash_Read(ADR_SSI_OFS, &ofs));
	CHECK(ofs == 1000 / 64);
	EncoderSSI second;
	feedAmt23(second, 0x2000);
	CHECK(second.getPos() == second.getPosAbs() - 960);
}

TEST_CASE_FIXTURE(SsiFixture, "a negative position offset is restored as the same offset" *
				  KNOWN_ISSUE("bug 77 (flash part): saveFlash writes the int32 offset as uint16 (EncoderSSI.cpp:58) and "
							  "restoreFlash reads it back unsigned (line 48): an offset of -5 (14 bit) is restored as "
							  "65531, the position after a restart is 4 turns (65536 counts) off")) {
	{
		storeConfig(14, 0, MODE_AMT23);
		EncoderSSI first;
		feedAmt23(first, 0x2100); // 256
		first.setPos(261);		   // offset -5
		REQUIRE(first.getPos() == 261);
		first.saveFlash();
		drain();
	}
	EncoderSSI second;
	feedAmt23(second, 0x2100);
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(second.getPos() == 261);
}

// ---------------- Commands ----------------

TEST_CASE_FIXTURE(SsiFixture, "the bits command sets and reads the resolution in raw mode") {
	storeConfig(22, 0, MODE_RAW);
	EncoderSSI enc;
	CHECK(get(enc, "bits") == 22);
	CHECK(cmd(enc, "bits", CMDtype::set, 18) == CommandStatus::OK);
	CHECK(get(enc, "bits") == 18);
	CHECK(enc.getCpr() == (1U << 18));
}

TEST_CASE_FIXTURE(SsiFixture, "without a stored configuration the resolution is 22 bits at speed preset 0") {
	EncoderSSI enc;
	REQUIRE(cmd(enc, "mode", CMDtype::set, MODE_RAW) == CommandStatus::OK); // defined mode first (bug 77)
	CHECK(get(enc, "bits") == 22);
	CHECK(get(enc, "speed") == 0);
	CHECK(enc.getCpr() == (1U << 22));
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_32);
}

TEST_CASE_FIXTURE(SsiFixture, "in AMT23 mode the bits command limits the resolution to 12 to 14 bits") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	CHECK(cmd(enc, "bits", CMDtype::set, 16) == CommandStatus::OK);
	CHECK(get(enc, "bits") == 14);
	CHECK(cmd(enc, "bits", CMDtype::set, 8) == CommandStatus::OK);
	CHECK(get(enc, "bits") == 12);
	CHECK(cmd(enc, "bits", CMDtype::set, 13) == CommandStatus::OK);
	CHECK(get(enc, "bits") == 13);
}

TEST_CASE_FIXTURE(SsiFixture, "the bits command keeps the resolution within 1 to 32 bits" *
				  KNOWN_ISSUE("bug 77 (range part): the bits command stores any value in raw mode (EncoderSSI.cpp:212); "
							  "40 bits make getCpr / spiRxCompleted shift 1 by 40 (undefined, `1<<lenghtDataBit`, lines 111, 126, "
							  "133, 179). The flash layout holds 1 to 32 bits")) {
	storeConfig(22, 0, MODE_RAW);
	EncoderSSI enc;
	cmd(enc, "bits", CMDtype::set, 40);
	const int64_t bits = get(enc, "bits");
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(bits <= 32);
	CHECK(bits >= 1);
}

TEST_CASE_FIXTURE(SsiFixture, "the bits, speed and mode commands are registered with their request types") {
	// The command thread refuses request types missing from the flags before calling command()
	// (FFBoardMainCommandThread.cpp:97-103): bits info never reaches the ignored status at EncoderSSI.cpp:212
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	CHECK(enc.getCommandFromName("bits")->flags == (CMDFLAG_GET | CMDFLAG_SET));
	CHECK(enc.getCommandFromName("speed")->flags == (CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING));
	CHECK(enc.getCommandFromName("mode")->flags == (CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING));
}

TEST_CASE_FIXTURE(SsiFixture, "the speed command sets the preset, limits it to 0 to 2 and reconfigures the port") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	CHECK(cmd(enc, "speed", CMDtype::set, 1) == CommandStatus::OK);
	CHECK(get(enc, "speed") == 1);
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_16);
	CHECK(cmd(enc, "speed", CMDtype::set, 7) == CommandStatus::OK);
	CHECK(get(enc, "speed") == 2);
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_8);
	CHECK(cmd(enc, "speed", CMDtype::set, -3) == CommandStatus::OK);
	CHECK(get(enc, "speed") == 0);
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_32);
}

TEST_CASE_FIXTURE(SsiFixture, "the speed info lists the SPI clock of each preset") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	// 42 MHz base clock of SPI3 / 32, 16, 8
	CHECK(info(enc, "speed") == "1312kHz:0\n2625kHz:1\n5250kHz:2\n");
	CHECK(enc.printSpeeds() == "1312kHz:0\n2625kHz:1\n5250kHz:2\n");
}

TEST_CASE_FIXTURE(SsiFixture, "the mode command sets and reads the mode") {
	storeConfig(14, 0, MODE_RAW);
	EncoderSSI enc;
	CHECK(get(enc, "mode") == MODE_RAW);
	CHECK(cmd(enc, "mode", CMDtype::set, MODE_AMT23) == CommandStatus::OK);
	CHECK(get(enc, "mode") == MODE_AMT23);
	enc.getPosAbs();
	CHECK(lastReadSize() == 2);
}

TEST_CASE_FIXTURE(SsiFixture, "the mode command uses the low 4 bits of the value") {
	storeConfig(14, 0, MODE_RAW);
	EncoderSSI enc;
	CHECK(cmd(enc, "mode", CMDtype::set, 0x11) == CommandStatus::OK);
	CHECK(get(enc, "mode") == MODE_AMT23);
}

TEST_CASE_FIXTURE(SsiFixture, "the mode info lists the modes") {
	storeConfig(14, 0, MODE_RAW);
	EncoderSSI enc;
	CHECK(info(enc, "mode") == "Raw:0\nAMT23:1\n");
}

TEST_CASE_FIXTURE(SsiFixture, "the mode command rejects a request other than get, set and info") {
	storeConfig(14, 0, MODE_RAW);
	EncoderSSI enc;
	std::vector<CommandReply> replies;
	CHECK(cmd(enc, "mode", CMDtype::getat, 0, &replies) == CommandStatus::ERR);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(SsiFixture, "the mode command rejects a mode that does not exist" *
				  KNOWN_ISSUE("bug 77 (range part): the mode command stores any value 0..15 (EncoderSSI.cpp:233); with mode 5 "
							  "spiRxCompleted decodes nothing and the position never changes")) {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	cmd(enc, "mode", CMDtype::set, 5);
	const int64_t mode = get(enc, "mode");
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(mode == MODE_AMT23);
}

TEST_CASE_FIXTURE(SsiFixture, "the errors command reports the number of rejected frames") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	CHECK(get(enc, "errors") == 0);
	feed(enc, {0x00, 0x00}); // both check bits 0: parity result 0
	feed(enc, {0x00, 0x00});
	CHECK(get(enc, "errors") == 2);
	CHECK(enc.getCommandFromName("errors")->flags == CMDFLAG_GET);
}

TEST_CASE_FIXTURE(SsiFixture, "a command id the SSI encoder does not handle is not found") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	ParsedCommand parsed;
	parsed.cmdId = static_cast<uint32_t>(EncoderSSI::EncoderSSI_commands::cs); // declared, not registered
	parsed.type = CMDtype::get;
	parsed.target = &enc;
	std::vector<CommandReply> replies;
	CHECK(enc.command(parsed, replies) == CommandStatus::NOT_FOUND);
	CHECK(enc.getCommandFromName("cs") == nullptr);
}

TEST_CASE_FIXTURE(SsiFixture, "the help of the SSI encoder lists its commands") {
	storeConfig(14, 0, MODE_AMT23);
	EncoderSSI enc;
	CmdHandlerCommanddef *def = enc.getCommandFromName("help");
	REQUIRE(def != nullptr);
	ParsedCommand parsed;
	parsed.cmdId = def->cmdId;
	parsed.type = CMDtype::get;
	parsed.target = &enc;
	std::vector<CommandReply> replies;
	REQUIRE(enc.internalCommand(parsed, replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	const std::string &help = replies.front().reply;
	CHECK(help.find("SPI SSI Encoder") != std::string::npos);
	for (const char *name : {"bits", "speed", "errors", "mode"}) {
		CAPTURE(name);
		CHECK(help.find(name) != std::string::npos);
	}
}

// ---------------- Axis ----------------

TEST_CASE_FIXTURE(AxisFixture, "the encoder chooser of an axis creates the SSI encoder and deleting it frees the encoder SPI port") {
	SsiFixture::storeConfig(14, 0, MODE_AMT23);
	Encoder *enc = selectFirmwareEncoder(6);
	REQUIRE(dynamic_cast<EncoderSSI *>(enc) != nullptr);
	CHECK_FALSE(EncoderSSI::isCreatable());
	CHECK_FALSE(ext3_spi.hasFreePins());
	if (HostTarget::spiTransferPending(&hspi3)) {
		HostTarget::spiQueueRx(&hspi3, HostTarget::spiFrameBytes(amt23Frame(0x2000), 2));
		REQUIRE(HostTarget::spiComplete(&hspi3));
	}
	selectFirmwareEncoder(0);
	CHECK(EncoderSSI::isCreatable());
	CHECK(ext3_spi.hasFreePins());
	CHECK(ext3_spi.isPinFree(ext3_spi.getCsPins()[0]));
}

TEST_CASE_FIXTURE(AxisFixture, "the axis reads the AMT23 position through the SSI encoder") {
	SsiFixture::storeConfig(14, 0, MODE_AMT23);
	Encoder *enc = selectFirmwareEncoder(6);
	REQUIRE(enc != nullptr);
	if (!HostTarget::spiTransferPending(&hspi3)) {
		enc->getPosAbs();
	}
	HostTarget::spiQueueRx(&hspi3, HostTarget::spiFrameBytes(amt23Frame(0x2400), 2));
	REQUIRE(HostTarget::spiComplete(&hspi3));
	CHECK(axis().getEncoder()->getPosAbs() == 0x400);
	if (HostTarget::spiTransferPending(&hspi3)) {
		REQUIRE(HostTarget::spiComplete(&hspi3));
	}
	selectFirmwareEncoder(0);
}

} // TEST_SUITE
