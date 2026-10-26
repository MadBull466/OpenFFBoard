/*
 * Tests of EncoderBissC (EncoderBissC.cpp): BiSS-C absolute encoder on the encoder SPI port (ext3_spi, hspi3) of the
 * F407 target (ffb_target_tests). The encoder thread ("BISSENC") is a resumable task (HostRtos::makeResumable): a
 * position request (getPosAbs) gives the request semaphore, the thread starts an 8 byte DMA receive and waits for its
 * notification, the test scripts the MISO bytes (HostTarget::spiQueueRx), ends the DMA (HostTarget::spiComplete) and
 * lets the thread decode the frame.
 *
 * Frame layout read by the firmware (updateFrame, EncoderBissC.cpp:163-204): 64 bits MSB first; the line is high (idle)
 * at the first sample(s), then the acknowledge (low), the start bit (1), CDS (0), the position (bits), nE and nW (active
 * low: 1 = no error / no warning) and the CRC6 (x^6 + x + 1, start value 0, transmitted inverted) over position, nE
 * and nW. updateFrame drops everything up to and including the first 1 and the bit after it, then aligns at the next 1
 * (the start bit).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "HostRtos.h"
#include "MockSPIDevice.h"
#include "TestHelpers.h"

#include "EncoderBissC.h"
#include "cpp_target_config.h"
#include "eeprom_addresses.h"
#include "semaphore.hpp"
#include "thread.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#ifdef HOSTTEST_SANITIZERS
/** Undefined behaviour that has no visible effect on the host (the intended result is reached): the sanitizer aborts the run, so skipped there */
#define UB_SKIPPED_WITH_SANITIZERS(text) doctest::description("Undefined behaviour (skipped with sanitizers): " text) * doctest::skip()
#else
#define UB_SKIPPED_WITH_SANITIZERS(text) doctest::description("Undefined behaviour (skipped with sanitizers): " text)
#endif

namespace {

constexpr int DEFAULT_BITS = 22;
constexpr int32_t CPR22 = 1 << 22;

/**
 * BiSS-C CRC: polynomial x^6 + x + 1 (0x43), start value 0, over the data bits MSB first, transmitted inverted
 * (BiSS C protocol description, iC-Haus). Bitwise reference, independent of the table driven firmware code.
 */
uint8_t bissCrc6(uint32_t data, int bits) {
	uint8_t crc = 0;
	for (int i = bits - 1; i >= 0; i--) {
		const uint8_t feedback = static_cast<uint8_t>(((crc >> 5) & 1U) ^ ((data >> i) & 1U));
		crc = static_cast<uint8_t>((crc << 1) & 0x3F);
		if (feedback) {
			crc ^= 0x03;
		}
	}
	return static_cast<uint8_t>(~crc & 0x3F);
}

/** One BiSS-C single cycle frame as it appears on MISO */
struct Frame {
	uint32_t position = 0;
	int bits = DEFAULT_BITS; //!< Position bits
	bool nE = true;			 //!< Error bit (active low: true = no error)
	bool nW = true;			 //!< Warning bit (active low: true = no warning)
	int idle = 1;			 //!< High bits before the acknowledge
	int ack = 3;			 //!< Acknowledge bits (low) before the start bit
	uint8_t crcXor = 0;		 //!< Bits flipped in the transmitted CRC (transmission error)
};

/** 64 bit MISO stream of a frame (bits after the CRC: low). Bits beyond the 64 sampled bits are lost */
uint64_t stream(const Frame &f) {
	uint64_t s = 0;
	int bit = 63;
	auto put = [&](uint64_t value, int count) {
		for (int i = count - 1; i >= 0; i--, bit--) {
			if (bit >= 0 && ((value >> i) & 1U)) {
				s |= 1ULL << bit;
			}
		}
	};
	const uint32_t mask = static_cast<uint32_t>((1ULL << f.bits) - 1);
	const uint32_t data = ((f.position & mask) << 2) | (static_cast<uint32_t>(f.nE) << 1) | static_cast<uint32_t>(f.nW);
	put(~0ULL, f.idle);
	put(0, f.ack);
	put(1, 1); // start
	put(0, 1); // CDS
	put(data, f.bits + 2);
	put(bissCrc6(data, f.bits + 2) ^ f.crcXor, 6);
	return s;
}

uint64_t stream(uint32_t position) {
	Frame f;
	f.position = position;
	return stream(f);
}

/** Calls getPosAbs of an encoder from its own (resumable) task: a call that waits parks the task in the wait */
struct PositionReader : cpp_freertos::Thread {
	explicit PositionReader(Encoder &e) : cpp_freertos::Thread("POSREADER", 64, 1), enc(e) {
		Start();
		HostRtos::makeResumable(GetHandle());
	}
	void Run() override {
		result = enc.getPosAbs();
		done = true;
		while (true) {
			idle.Take();
		}
	}
	Encoder &enc;
	int32_t result = 0;
	bool done = false;
	cpp_freertos::BinarySemaphore idle{false};
};

struct BissFixture : TargetFixture {
	std::unique_ptr<EncoderBissC> enc;
	TaskHandle_t task = nullptr;

	~BissFixture() override { enc.reset(); } // the encoder is deleted before the fixture resets the target

	EncoderBissC &create() {
		enc = std::make_unique<EncoderBissC>();
		task = HostRtos::makeResumable("BISSENC");
		REQUIRE(task != nullptr);
		return *enc;
	}

	/** Executes a command by its name like the command thread does */
	CommandStatus cmd(const std::string &name, CMDtype type, int64_t val = 0, std::vector<CommandReply> *replies = nullptr) {
		CmdHandlerCommanddef *def = enc->getCommandFromName(name);
		REQUIRE_MESSAGE(def != nullptr, "command " << name << " not registered");
		ParsedCommand parsed;
		parsed.cmdId = def->cmdId;
		parsed.type = type;
		parsed.val = val;
		parsed.target = enc.get();
		std::vector<CommandReply> local;
		return enc->command(parsed, replies ? *replies : local);
	}
	int64_t get(const std::string &name) {
		std::vector<CommandReply> replies;
		CHECK(cmd(name, CMDtype::get, 0, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies.front().val;
	}

	/** Starts the DMA read of the next frame: request (getPosAbs) and the thread up to the DMA wait */
	void startRead() {
		enc->getPosAbs();
		HostRtos::runUntilBlocked(task);
		REQUIRE(HostTarget::spiTransferPending(&hspi3));
	}
	/** Ends the pending DMA read with these 64 bits and lets the thread process the frame */
	void finishRead(uint64_t bits) {
		HostTarget::spiQueueRx(&hspi3, HostTarget::spiFrameBytes(bits, 8));
		REQUIRE(HostTarget::spiComplete(&hspi3));
		HostRtos::runUntilBlocked(task);
	}
	/** One complete read cycle */
	void read(uint64_t bits) {
		startRead();
		finishRead(bits);
	}
	void read(const Frame &f) { read(stream(f)); }
	void readPosition(uint32_t position) { read(stream(position)); }
};

} // namespace

TEST_SUITE("EncoderBissC") {

// ---------------------------------------------------------------- identity and creation

TEST_CASE_FIXTURE(BissFixture, "the BiSS-C encoder identifies itself as an absolute BISS-C encoder") {
	create();
	CHECK(enc->getEncoderType() == EncoderType::absolute);
	CHECK(std::string(enc->getInfo().name) == "BISS-C");
	CHECK(enc->getInfo().id == CLSID_ENCODER_BISS);
}

TEST_CASE_FIXTURE(BissFixture, "the BiSS-C encoder is not creatable while one exists and creatable again after deleting it") {
	REQUIRE(EncoderBissC::isCreatable());
	create();
	CHECK_FALSE(EncoderBissC::isCreatable());
	enc.reset();
	CHECK(EncoderBissC::isCreatable());
}

TEST_CASE_FIXTURE(BissFixture, "the BiSS-C encoder is not creatable while the encoder SPI port is taken exclusively") {
	ext3_spi.takeExclusive(true);
	CHECK_FALSE(EncoderBissC::isCreatable());
	ext3_spi.takeExclusive(false);
}

TEST_CASE_FIXTURE(BissFixture, "the BiSS-C encoder takes the encoder SPI port exclusively and releases it when deleted") {
	create();
	CHECK_FALSE(ext3_spi.hasFreePins()); // exclusive
	CHECK_FALSE(ext3_spi.isPinFree(ext3_spi.getCsPins()[0]));
	enc.reset();
	CHECK(ext3_spi.hasFreePins());
	CHECK(ext3_spi.isPinFree(ext3_spi.getCsPins()[0]));
}

TEST_CASE_FIXTURE(BissFixture, "the BiSS-C encoder is not creatable while another device uses a CS pin of the encoder SPI port" *
								   KNOWN_ISSUE("EncoderBissC::isCreatable (EncoderBissC.cpp:131) uses SPIPort::hasFreePins, which only "
											   "checks the exclusive flag and that the port has CS pins (SPI.cpp hasFreePins, bug 67 "
											   "pattern), and the constructor takes getCsPins()[0] (line 27) even if another device "
											   "reserved it, then takes the port exclusively although the bus 'can't be shared' "
											   "(comment at configSPI). Bug 85")) {
	MockSPIDevice other(ext3_spi, ext3_spi.getCsPins()[0]); // SPI3_SS1 reserved by another device
	REQUIRE_FALSE(ext3_spi.isPinFree(ext3_spi.getCsPins()[0]));
	REQUIRE_ALL_HAL_MODELLED();

	CHECK_FALSE(EncoderBissC::isCreatable());
}

// ---------------------------------------------------------------- SPI configuration

TEST_CASE_FIXTURE(BissFixture, "the BiSS-C encoder reads 8 bytes with DMA in SPI mode 3 with the most significant bit first") {
	create();
	startRead();
	finishRead(stream(0));

	const HostTarget::SpiTransfer &t = HostTarget::spiTransfers(&hspi3).front();
	CHECK(t.call == HostTarget::SpiCall::Receive);
	CHECK(t.mode == HostTarget::BusMode::Dma);
	CHECK(t.size == 8);
	CHECK(t.init.CLKPolarity == SPI_POLARITY_HIGH);
	CHECK(t.init.CLKPhase == SPI_PHASE_2EDGE);
	CHECK(t.init.FirstBit == SPI_FIRSTBIT_MSB);
	CHECK(t.init.DataSize == SPI_DATASIZE_8BIT);
}

TEST_CASE_FIXTURE(BissFixture, "the BiSS-C encoder drives its CS pin low during the read and high after it") {
	create();
	startRead();
	CHECK_FALSE(HostTarget::spiCsLevels(&hspi3)[0]); // SPI3_SS1 active (low) while the frame is clocked in
	finishRead(stream(0));
	CHECK(HostTarget::spiCsLevels(&hspi3)[0]);
}

TEST_CASE_FIXTURE(BissFixture, "the default speed preset 3 clocks the encoder at about 2.6 MHz") {
	create();
	CHECK(get("speed") == 3);
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_16); // 42 MHz / 16 = 2.625 MHz
}

TEST_CASE("speed presets select the SPI clock closest to 0.6, 1.3 and 2.6 MHz") {
	struct Preset {
		int64_t preset;
		uint32_t prescaler;
	};
	// 42 MHz base clock: /64 = 656 kHz, /32 = 1.31 MHz, /16 = 2.63 MHz
	const Preset presets[] = {{1, SPI_BAUDRATEPRESCALER_64}, {2, SPI_BAUDRATEPRESCALER_32}, {3, SPI_BAUDRATEPRESCALER_16}};
	for (const Preset &p : presets) {
		CAPTURE(p.preset);
		BissFixture f;
		f.create();
		if (p.preset == 3) { // the default: change it first so that set 3 has to reconfigure the port
			REQUIRE(f.cmd("speed", CMDtype::set, 1) == CommandStatus::OK);
		}
		REQUIRE(f.cmd("speed", CMDtype::set, p.preset) == CommandStatus::OK);
		CHECK(f.get("speed") == p.preset);
		CHECK(hspi3.Init.BaudRatePrescaler == p.prescaler);
		f.startRead();
		CHECK(HostTarget::spiTransfers(&hspi3).back().init.BaudRatePrescaler == p.prescaler);
		f.finishRead(stream(0));
	}
}

TEST_CASE_FIXTURE(BissFixture, "speed set below 1 selects preset 1") {
	create();
	REQUIRE(cmd("speed", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(get("speed") == 1);
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_64);
}

TEST_CASE_FIXTURE(BissFixture, "speed set above 3 selects preset 3") {
	create();
	REQUIRE(cmd("speed", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(cmd("speed", CMDtype::set, 7) == CommandStatus::OK);
	CHECK(get("speed") == 3);
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_16);
}

TEST_CASE_FIXTURE(BissFixture, "bits set above 24 lowers speed preset 3 to preset 2") {
	create();
	REQUIRE(cmd("bits", CMDtype::set, 25) == CommandStatus::OK);
	CHECK(get("speed") == 2);
}

TEST_CASE_FIXTURE(BissFixture, "bits set to 24 keeps speed preset 3") {
	create();
	REQUIRE(cmd("bits", CMDtype::set, 24) == CommandStatus::OK);
	CHECK(get("speed") == 3);
}

TEST_CASE_FIXTURE(BissFixture, "bits set above 24 with speed preset 1 keeps preset 1") {
	create();
	REQUIRE(cmd("speed", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(cmd("bits", CMDtype::set, 26) == CommandStatus::OK);
	CHECK(get("speed") == 1);
}

TEST_CASE_FIXTURE(BissFixture, "bits set above 24 lowers the SPI clock of the encoder SPI port to preset 2" *
								   KNOWN_ISSUE("The bits command lowers spiSpeed to 2 'to prevent encoder from losing data' "
											   "(EncoderBissC.cpp:253-255) but does not call configSPI: the port keeps the 2.6 MHz "
											   "clock of preset 3 until the next speed set or restart. Bug 90")) {
	create();
	REQUIRE(cmd("bits", CMDtype::set, 25) == CommandStatus::OK);
	REQUIRE(get("speed") == 2);
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_32); // 1.3 MHz
}

// ---------------------------------------------------------------- frame decoding

TEST_CASE_FIXTURE(BissFixture, "a frame with a valid CRC sets the 22 bit position") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x1ABCDE);
	CHECK(enc->getPosAbs() == 0x1ABCDE);
	CHECK(get("errors") == 0);
}

TEST_CASE_FIXTURE(BissFixture, "the position counts down by default because the direction is inverted") {
	create();
	CHECK(get("dir") == 1);
	readPosition(0x12345);
	CHECK(enc->getPosAbs() == -0x12345);
	CHECK(enc->getPos() == -0x12345);
}

TEST_CASE_FIXTURE(BissFixture, "dir set 0 makes the position count up") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(get("dir") == 0);
	readPosition(0x12345);
	CHECK(enc->getPos() == 0x12345);
}

TEST_CASE_FIXTURE(BissFixture, "a frame with two idle bits before the acknowledge is decoded") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	Frame f;
	f.position = 0x2A5A5;
	f.idle = 2;
	read(f);
	CHECK(enc->getPosAbs() == 0x2A5A5);
	CHECK(get("errors") == 0);
}

TEST_CASE_FIXTURE(BissFixture, "a frame with a long acknowledge is decoded") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	Frame f;
	f.position = 0x15555;
	f.ack = 20; // start bit at bit 42: the frame still fits into the 64 sampled bits
	read(f);
	CHECK(enc->getPosAbs() == 0x15555);
	CHECK(get("errors") == 0);
}

TEST_CASE_FIXTURE(BissFixture, "a frame whose CRC does not fit into the 64 sampled bits is counted as an error") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x100);
	Frame f;
	f.position = 0x3F0F0F;
	f.ack = 35; // start bit at bit 27: position, nE, nW and CRC need 32 bits after it
	read(f);
	CHECK(get("errors") == 1);
	CHECK(enc->getPosAbs() == 0x100);
}

TEST_CASE_FIXTURE(BissFixture, "a frame with a wrong CRC is counted as an error and keeps the last position") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x1000);
	Frame f;
	f.position = 0x2000;
	f.crcXor = 0x01;
	read(f);
	CHECK(get("errors") == 1);
	CHECK(enc->getPosAbs() == 0x1000);
}

TEST_CASE_FIXTURE(BissFixture, "a frame with a flipped position bit fails the CRC check") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x1000);
	uint64_t bits = stream(0x2000);
	bits ^= 1ULL << 40; // a data bit (position bits are 57..36 for 1 idle and 3 acknowledge bits)
	read(bits);
	CHECK(get("errors") == 1);
	CHECK(enc->getPosAbs() == 0x1000);
}

TEST_CASE_FIXTURE(BissFixture, "errors counts every frame with a wrong CRC") {
	create();
	Frame f;
	f.crcXor = 0x20;
	read(f);
	read(f);
	read(f);
	CHECK(get("errors") == 3);
}

TEST_CASE_FIXTURE(BissFixture, "a frame of all ones (line stuck high) is counted as an error") {
	create();
	read(~0ULL);
	CHECK(get("errors") == 1);
	CHECK(enc->getPosAbs() == 0);
}

TEST_CASE_FIXTURE(BissFixture, "a frame of all zeros (no encoder answer) is counted as an error" *
								   UB_SKIPPED_WITH_SANITIZERS("updateFrame calls __builtin_clzll(0) (EncoderBissC.cpp:175 and 182), "
															  "which is undefined (UBSan: 'passing zero to __builtin_clz()'); the "
															  "same happens at line 182 for a frame with only the first bit set. On "
															  "the host and on ARM the shifted value stays 0, so the CRC check fails as "
															  "intended. Bug 83")) {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x777);
	read(0ULL);
	CHECK(get("errors") == 1);
	CHECK(enc->getPosAbs() == 0x777);
}

TEST_CASE_FIXTURE(BissFixture, "characterization: a frame with the error bit set and a valid CRC updates the position") {
	// NEEDS REVIEW 99: updateFrame extracts nE / nW (EncoderBissC.cpp:189) but never uses them: a frame that reports an
	// encoder error (nE = 0, position not valid per BiSS) or a warning is accepted like any other frame and not counted
	// in errors. Should an error frame keep the last position and count as an error?
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	Frame f;
	f.position = 0x4321;
	f.nE = false;
	read(f);
	CHECK(enc->getPosAbs() == 0x4321);
	CHECK(get("errors") == 0);
}

TEST_CASE_FIXTURE(BissFixture, "characterization: a frame with the warning bit set and a valid CRC updates the position") {
	// NEEDS REVIEW 99 (see above)
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	Frame f;
	f.position = 0x4321;
	f.nW = false;
	read(f);
	CHECK(enc->getPosAbs() == 0x4321);
	CHECK(get("errors") == 0);
}

TEST_CASE_FIXTURE(BissFixture, "bits set to 16 decodes 16 bit frames and gives 65536 counts per turn") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	REQUIRE(cmd("bits", CMDtype::set, 16) == CommandStatus::OK);
	CHECK(get("bits") == 16);
	CHECK(enc->getCpr() == 65536);
	Frame f;
	f.bits = 16;
	f.position = 0x7ABC;
	read(f);
	CHECK(enc->getPosAbs() == 0x7ABC);
	CHECK(get("errors") == 0);
}

TEST_CASE_FIXTURE(BissFixture, "bits set to 26 decodes 26 bit frames") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	REQUIRE(cmd("bits", CMDtype::set, 26) == CommandStatus::OK);
	CHECK(enc->getCpr() == (1U << 26));
	Frame f;
	f.bits = 26;
	f.position = 0x1234567;
	read(f);
	CHECK(enc->getPosAbs() == 0x1234567);
	CHECK(get("errors") == 0);
}

TEST_CASE_FIXTURE(BissFixture, "a 22 bit frame read with bits set to 16 fails the CRC check") {
	create();
	REQUIRE(cmd("bits", CMDtype::set, 16) == CommandStatus::OK);
	readPosition(0x1ABCDE);
	CHECK(get("errors") == 1);
}

TEST_CASE_FIXTURE(BissFixture, "the default resolution is 22 bits") {
	create();
	CHECK(get("bits") == DEFAULT_BITS);
	CHECK(enc->getCpr() == static_cast<uint32_t>(CPR22));
}

TEST_CASE_FIXTURE(BissFixture, "bits set above 32 is not accepted" *
								   KNOWN_ISSUE_UB("The bits command stores any value (handleGetSet, EncoderBissC.cpp:252): getCpr then "
												  "shifts 1 by 32 or more (line 238; reached in this test by the destructor's "
												  "setPos(0)) and updateFrame computes (1 << (bits + 2)) - 1 (line 188, undefined "
												  "from 29 bits on); saveFlash keeps only 5 bits (bits 40 is restored as 8). Bug 84")) {
	create();
	REQUIRE(cmd("bits", CMDtype::set, 40) == CommandStatus::OK);
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(get("bits") <= 32);
}

TEST_CASE_FIXTURE(BissFixture, "bits set to 0 is not accepted" *
								   KNOWN_ISSUE("The bits command stores 0 (EncoderBissC.cpp:252): the next frame computes "
											   "halfres = 1 << -1 (line 52, undefined) and saveFlash stores max(-1, 0) = 0, restored "
											   "as 1 bit. Bug 84")) {
	create();
	REQUIRE(cmd("bits", CMDtype::set, 0) == CommandStatus::OK);
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(get("bits") >= 1);
}

// ---------------------------------------------------------------- multiturn

TEST_CASE_FIXTURE(BissFixture, "crossing from the end of a turn to its start counts one turn up") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x100000);
	readPosition(0x300000);
	readPosition(0x3FFFF0);
	readPosition(0x000010);
	CHECK(enc->getPosAbs() == CPR22 + 0x10);
}

TEST_CASE_FIXTURE(BissFixture, "crossing from the start of a turn to its end counts one turn down") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x000010);
	readPosition(0x3FFFF0);
	CHECK(enc->getPosAbs() == 0x3FFFF0 - CPR22);
}

TEST_CASE_FIXTURE(BissFixture, "a move of less than half a turn between two frames does not count a turn") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x000001);
	readPosition(0x200000); // + half a turn - 1
	readPosition(0x000001); // - half a turn + 1
	CHECK(enc->getPosAbs() == 1);
	readPosition(0x200000);
	CHECK(enc->getPosAbs() == 0x200000);
}

TEST_CASE_FIXTURE(BissFixture, "a move of more than half a turn between two frames counts a turn") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x000000);
	readPosition(0x200001); // + half a turn + 1: taken as - half a turn + 1 (one turn down)
	CHECK(enc->getPosAbs() == 0x200001 - CPR22);
}

TEST_CASE_FIXTURE(BissFixture, "several turns up accumulate in the multiturn count") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0);
	for (int turn = 0; turn < 3; turn++) {
		readPosition(0x180000);
		readPosition(0x300000);
		readPosition(0x000100);
	}
	CHECK(enc->getPosAbs() == 3 * CPR22 + 0x100);
}

TEST_CASE_FIXTURE(BissFixture, "the multiturn count is inverted with the direction") {
	create();
	readPosition(0x100000); // first frame within half a turn of the offset 0
	readPosition(0x300000);
	readPosition(0x3FFFF0);
	readPosition(0x000010);
	CHECK(enc->getPosAbs() == -(CPR22 + 0x10));
}

TEST_CASE_FIXTURE(BissFixture, "a first frame more than half a turn from the stored offset counts one turn down") {
	// EncoderBissC.cpp:54-58: the first frame compares with the restored offset (0 without a stored one)
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x300000);
	CHECK(enc->getPosAbs() == 0x300000 - CPR22);
}

TEST_CASE_FIXTURE(BissFixture, "a first frame less than half a turn from the stored offset counts no turn") {
	HostPlatform::flash()[ADR_BISSENC_OFS] = 0xF000; // offset 0xF000 << 6 = 0x3C0000
	create();
	readPosition(0x300000); // 0xC0000 below the offset
	CHECK(enc->getPosAbs() == -0x300000);
}

TEST_CASE_FIXTURE(BissFixture, "a frame with a wrong CRC does not change the multiturn count") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x100000); // first frame within half a turn of the offset 0
	readPosition(0x300000);
	readPosition(0x3FFFF0);
	Frame f;
	f.position = 0x000010;
	f.crcXor = 0x3F;
	read(f);
	CHECK(enc->getPosAbs() == 0x3FFFF0);
}

// ---------------------------------------------------------------- setPos and offset

TEST_CASE_FIXTURE(BissFixture, "setPos 0 makes the current position 0") {
	create();
	readPosition(0x12345);
	enc->setPos(0);
	CHECK(enc->getPos() == 0);
	readPosition(0x12345 + 100);
	CHECK(enc->getPos() == -100); // inverted direction
}

TEST_CASE_FIXTURE(BissFixture, "setPos 0 with dir 0 makes the current position 0") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x12345);
	enc->setPos(0);
	CHECK(enc->getPos() == 0);
	readPosition(0x12345 + 100);
	CHECK(enc->getPos() == 100);
}

TEST_CASE_FIXTURE(BissFixture, "setPos 0 clears the multiturn count") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x100000); // first frame within half a turn of the offset 0
	readPosition(0x300000);
	readPosition(0x3FFFF0);
	readPosition(0x000010); // one turn up
	REQUIRE(enc->getPosAbs() == CPR22 + 0x10);
	enc->setPos(0);
	CHECK(enc->getPos() == 0);
}

TEST_CASE_FIXTURE(BissFixture, "setPos with a value within the turn makes getPos return that value" *
								   KNOWN_ISSUE("setPos computes mtpos = newpos / getCpr() (EncoderBissC.cpp:230) with the uint32_t "
											   "result of getCpr: a negative newpos (the inverted default direction turns setPos(100) "
											   "into -100) is converted to unsigned, giving mtpos 1023 instead of 0, so getPos is one "
											   "turn (2^22) off. Reachable with the axis pos command (Axis.cpp:1151). Bug "
											   "91")) {
	create();
	readPosition(1000);
	enc->setPos(100);
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(enc->getPos() == 100);
}

TEST_CASE_FIXTURE(BissFixture, "setPos with a value within the turn and dir 0 makes getPos return that value" *
								   KNOWN_ISSUE("Bug 91 with dir 0: setPos(-100) divides -100 by the uint32_t getCpr() "
											   "(EncoderBissC.cpp:230). Bug 91")) {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(1000);
	enc->setPos(-100);
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(enc->getPos() == -100);
}

TEST_CASE_FIXTURE(BissFixture, "setPos with a positive value and dir 0 makes getPos return that value") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(1000);
	enc->setPos(100);
	CHECK(enc->getPos() == 100);
}

TEST_CASE_FIXTURE(BissFixture, "a stored offset is scaled from 16 bits to the 22 bit resolution") {
	HostPlatform::flash()[ADR_BISSENC_OFS] = 0x0100; // 0x100 << (22 - 16) = 0x4000
	create();
	readPosition(0x5000);
	CHECK(enc->getPos() == -0x5000 + 0x4000);
}

TEST_CASE_FIXTURE(BissFixture, "a stored offset is used unscaled with a resolution of 16 bits or less") {
	HostPlatform::flash()[ADR_BISSENC_CONF1] = (14 - 1) | (2 << 5) | (0 << 7); // 14 bits, preset 3, dir 0
	HostPlatform::flash()[ADR_BISSENC_OFS] = 0x0100;
	create();
	Frame f;
	f.bits = 14;
	f.position = 0x0300;
	read(f);
	CHECK(enc->getPos() == 0x0300 - 0x0100);
}

TEST_CASE_FIXTURE(BissFixture, "saveFlash stores the offset scaled to 16 bits") {
	create();
	readPosition(0x12345);
	enc->setPos(0); // offset = 0x12345
	enc->saveFlash();
	CHECK(HostPlatform::flash()[ADR_BISSENC_OFS] == (0x12345 >> 6));
}

// ---------------------------------------------------------------- flash configuration

TEST_CASE_FIXTURE(BissFixture, "without a stored configuration the encoder uses 22 bits, speed preset 3 and inverted direction") {
	create();
	CHECK(get("bits") == 22);
	CHECK(get("speed") == 3);
	CHECK(get("dir") == 1);
}

TEST_CASE_FIXTURE(BissFixture, "a stored configuration selects bits, speed preset and direction") {
	HostPlatform::flash()[ADR_BISSENC_CONF1] = (18 - 1) | ((2 - 1) << 5) | (0 << 7);
	create();
	CHECK(get("bits") == 18);
	CHECK(get("speed") == 2);
	CHECK(get("dir") == 0);
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_32);
}

TEST_CASE_FIXTURE(BissFixture, "a stored configuration with 31 bits and the inverted direction is restored") {
	// 31 bits: the largest value for which getCpr (1 << bits, EncoderBissC.cpp:238) is defined; the 5 bit field also
	// holds 32, where getCpr shifts by 32 (bug 84)
	HostPlatform::flash()[ADR_BISSENC_CONF1] = (31 - 1) | ((1 - 1) << 5) | (1 << 7);
	create();
	CHECK(get("bits") == 31);
	CHECK(get("speed") == 1);
	CHECK(get("dir") == 1);
}

TEST_CASE_FIXTURE(BissFixture, "a stored speed field of 3 clocks the encoder at about 2.6 MHz like preset 3") {
	// restoreFlash decodes 2 bits + 1 (1..4, EncoderBissC.cpp:83); configSPI treats 4 like preset 3 (default branch)
	HostPlatform::flash()[ADR_BISSENC_CONF1] = (22 - 1) | (3 << 5) | (1 << 7);
	create();
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_16);
}

TEST_CASE_FIXTURE(BissFixture, "saveFlash stores bits - 1, speed preset - 1 and the direction in the configuration word") {
	create();
	REQUIRE(cmd("bits", CMDtype::set, 18) == CommandStatus::OK);
	REQUIRE(cmd("speed", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	enc->saveFlash();
	CHECK(HostPlatform::flash()[ADR_BISSENC_CONF1] == ((18 - 1) | ((2 - 1) << 5)));

	REQUIRE(cmd("dir", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(cmd("speed", CMDtype::set, 3) == CommandStatus::OK);
	enc->saveFlash();
	CHECK(HostPlatform::flash()[ADR_BISSENC_CONF1] == ((18 - 1) | ((3 - 1) << 5) | (1 << 7)));
}

TEST_CASE_FIXTURE(BissFixture, "a saved configuration is restored by the next BiSS-C encoder") {
	create();
	REQUIRE(cmd("bits", CMDtype::set, 20) == CommandStatus::OK);
	REQUIRE(cmd("speed", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	enc->saveFlash();
	enc.reset();

	create();
	CHECK(get("bits") == 20);
	CHECK(get("speed") == 1);
	CHECK(get("dir") == 0);
}

// ---------------------------------------------------------------- request timing

TEST_CASE_FIXTURE(BissFixture, "a position request starts a read only when no read is pending") {
	create();
	startRead();
	enc->getPosAbs(); // while the DMA read is pending: no new request
	finishRead(stream(0x10));
	CHECK_FALSE(HostTarget::spiTransferPending(&hspi3)); // the thread waits for the next request
	CHECK(HostTarget::spiTransfers(&hspi3).size() == 1);
}

TEST_CASE_FIXTURE(BissFixture, "a position request after a completed read starts the next read") {
	create();
	read(stream(0x10));
	enc->getPosAbs();
	HostRtos::runUntilBlocked(task);
	CHECK(HostTarget::spiTransferPending(&hspi3));
	finishRead(stream(0x10));
}

TEST_CASE_FIXTURE(BissFixture, "a position request while a read is pending returns the last position") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x10);
	startRead();
	CHECK(enc->getPosAbs() == 0x10);
	finishRead(stream(0x20));
	CHECK(enc->getPosAbs() == 0x20);
}

TEST_CASE_FIXTURE(BissFixture, "a position request more than 2 ms after the last update waits for the requested frame") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	HostPlatform::advanceMs(3); // last update at 0 (none yet)
	PositionReader reader(*enc);
	HostRtos::runUntilBlocked(reader.GetHandle());
	CHECK_FALSE(reader.done); // waits for the update
	HostRtos::runUntilBlocked(task);
	finishRead(stream(0x55));
	HostRtos::runUntilBlocked(reader.GetHandle());
	CHECK(reader.done);
	CHECK(reader.result == 0x55);
}

TEST_CASE_FIXTURE(BissFixture, "a position request 2 ms after the last update returns the last position at once") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	HostPlatform::advanceMs(2);
	PositionReader reader(*enc);
	HostRtos::runUntilBlocked(reader.GetHandle());
	CHECK(reader.done);
	CHECK(reader.result == 0);
	read(stream(0x55)); // the request was given: complete it
}

TEST_CASE_FIXTURE(BissFixture, "a waiting position request returns the last position when no frame arrives in time") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	HostPlatform::advanceMs(3);
	PositionReader reader(*enc);
	HostRtos::runUntilBlocked(reader.GetHandle()); // waits (finite timeout)
	REQUIRE_FALSE(reader.done);
	HostRtos::runUntilBlocked(reader.GetHandle()); // activated again without an update: the wait times out
	CHECK(reader.done);
	CHECK(reader.result == 0);
	read(stream(0x66)); // complete the request
}

TEST_CASE_FIXTURE(BissFixture, "characterization: a request more than 2 ms after an update nobody waited for returns at once") {
	// NEEDS REVIEW 102: every processed frame gives waitForUpdateSem (EncoderBissC.cpp:72-73), also when no request
	// waited for it (requests within 2 ms of the last update do not take it, line 209). The binary semaphore keeps this
	// stale signal, so the next request older than 2 ms takes it at once and returns the old position instead of waiting
	// for the frame it requested. The same pattern exists in MtEncoderSPI (MtEncoderSPI.cpp:106, 268-269). Intended?
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	readPosition(0x10); // at time 0: nobody waits, the update signal stays given
	HostPlatform::advanceMs(3);
	PositionReader reader(*enc);
	HostRtos::runUntilBlocked(reader.GetHandle());
	CHECK(reader.done);			  // did not wait
	CHECK(reader.result == 0x10); // old position
	read(stream(0x20));			  // complete the request
}

// ---------------------------------------------------------------- commands

TEST_CASE_FIXTURE(BissFixture, "errors is 0 for a new encoder") {
	create();
	CHECK(get("errors") == 0);
}

TEST_CASE_FIXTURE(BissFixture, "dir set 1 inverts the direction again") {
	create();
	REQUIRE(cmd("dir", CMDtype::set, 0) == CommandStatus::OK);
	REQUIRE(cmd("dir", CMDtype::set, 1) == CommandStatus::OK);
	CHECK(get("dir") == 1);
	readPosition(0x10);
	CHECK(enc->getPosAbs() == -0x10);
}

TEST_CASE_FIXTURE(BissFixture, "a command id without a handler is rejected") {
	create();
	ParsedCommand parsed;
	parsed.cmdId = static_cast<uint32_t>(EncoderBissC::EncoderBissC_commands::cs); // declared but not registered
	parsed.type = CMDtype::get;
	parsed.target = enc.get();
	std::vector<CommandReply> replies;
	CHECK(enc->command(parsed, replies) == CommandStatus::NOT_FOUND);
	CHECK(replies.empty());
}

// ---------------------------------------------------------------- deletion

TEST_CASE_FIXTURE(BissFixture, "deleting the BiSS-C encoder with a decoded position ends its thread") {
	create();
	readPosition(0x1234);
	const size_t tasks = HostRtos::taskCount();
	enc.reset();
	CHECK(HostRtos::findTask("BISSENC") == nullptr);
	CHECK(HostRtos::taskCount() == tasks - 1);
}

} // TEST_SUITE
