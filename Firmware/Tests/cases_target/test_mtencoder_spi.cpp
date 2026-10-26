/*
 * Tests of MtEncoderSPI (MtEncoderSPI.cpp): MagnTek MT6825 / MT6835 absolute magnetic encoders on the encoder SPI port
 * (ext3_spi, hspi3, CS pins SPI3_SS1..3) of the F407 target (ffb_target_tests). The encoder thread ("MTENC") is a
 * resumable task (HostRtos::makeResumable): a position request (getPosAbs) gives the request semaphore, the thread
 * starts a DMA burst read of the angle registers and waits for its notification, the test scripts the MISO bytes
 * (HostTarget::spiQueueRx), ends the DMA (HostTarget::spiComplete) and lets the thread process the angle.
 *
 * MT6825 burst read (4 bytes, command 0x83 = read from register 0x03): byte 1 = ANGLE[17:10]; byte 2 = ANGLE[9:4]
 * (bits 7..2), no-magnet warning (bit 1), PC1 (bit 0: bytes 1 and 2 together have even parity); byte 3 = ANGLE[3:0]
 * (bits 7..4), bits 3..2 with PC2 (bit 2: bits 7..2 of byte 3 have even parity), as checked by updateAngleStatusCb.
 * MT6835 burst read (6 bytes, command 0xA0 0x03): bytes 2..4 = ANGLE[20:13], ANGLE[12:5], ANGLE[4:0] (bits 7..3) and
 * 3 status bits, byte 5 = CRC8 (x^8 + x^2 + x + 1, start 0) over bytes 2..4.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "HostRtos.h"
#include "MockSPIDevice.h"
#include "TestHelpers.h"

#include "EncoderBissC.h"
#include "MtEncoderSPI.h"
#include "cpp_target_config.h"
#include "eeprom_addresses.h"
#include "semaphore.hpp"
#include "thread.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr int32_t CPR6825 = 262144;	 // 18 bit
constexpr int32_t CPR6835 = 2097152; // 21 bit

uint8_t parity(uint32_t v) {
	uint8_t p = 0;
	for (; v; v >>= 1) {
		p ^= static_cast<uint8_t>(v & 1U);
	}
	return p;
}

/** MISO bytes of an MT6825 burst read of the angle registers (with correct parity unless flipped) */
std::vector<uint8_t> mt6825(uint32_t angle, bool nomag = false, bool breakPc1 = false, bool breakPc2 = false) {
	const uint8_t b1 = static_cast<uint8_t>(angle >> 10);
	uint8_t b2 = static_cast<uint8_t>((((angle >> 4) & 0x3F) << 2) | (nomag ? 0x02 : 0));
	b2 |= static_cast<uint8_t>(parity(b1) ^ parity(b2) ^ (breakPc1 ? 1 : 0)); // PC1: bytes 1 and 2 even
	uint8_t b3 = static_cast<uint8_t>((angle & 0xF) << 4);
	b3 |= static_cast<uint8_t>((parity(b3 >> 3) ^ (breakPc2 ? 1 : 0)) << 2); // PC2: bits 7..2 even
	return {0x00, b1, b2, b3};
}

/** CRC8 x^8 + x^2 + x + 1, start 0, MSB first (bitwise reference) */
uint8_t crc8(const std::vector<uint8_t> &bytes) {
	uint8_t crc = 0;
	for (uint8_t b : bytes) {
		crc ^= b;
		for (int i = 0; i < 8; i++) {
			crc = static_cast<uint8_t>((crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1));
		}
	}
	return crc;
}

/** MISO bytes of an MT6835 burst read of the angle registers */
std::vector<uint8_t> mt6835(uint32_t angle, uint8_t status = 0, uint8_t crcXor = 0) {
	const std::vector<uint8_t> data = {static_cast<uint8_t>(angle >> 13), static_cast<uint8_t>(angle >> 5),
									   static_cast<uint8_t>(((angle & 0x1F) << 3) | (status & 0x7))};
	return {0x00, 0x00, data[0], data[1], data[2], static_cast<uint8_t>(crc8(data) ^ crcXor)};
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

struct MtFixture : TargetFixture {
	std::unique_ptr<MtEncoderSPI> enc;
	TaskHandle_t task = nullptr;

	~MtFixture() override { enc.reset(); } // the encoder is deleted before the fixture resets the target

	MtEncoderSPI &create() {
		enc = std::make_unique<MtEncoderSPI>();
		task = HostRtos::makeResumable("MTENC");
		REQUIRE(task != nullptr);
		return *enc;
	}

	CommandStatus cmd(const std::string &name, CMDtype type, int64_t val = 0, std::vector<CommandReply> *replies = nullptr, uint32_t adr = 0) {
		CmdHandlerCommanddef *def = enc->getCommandFromName(name);
		REQUIRE_MESSAGE(def != nullptr, "command " << name << " not registered");
		ParsedCommand parsed;
		parsed.cmdId = def->cmdId;
		parsed.type = type;
		parsed.val = val;
		parsed.adr = adr;
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

	/** Starts the DMA read of the angle: request (getPosAbs) and the thread up to the DMA wait */
	void startRead() {
		enc->getPosAbs();
		HostRtos::runUntilBlocked(task);
		REQUIRE(HostTarget::spiTransferPending(&hspi3));
	}
	void finishRead(const std::vector<uint8_t> &miso) {
		HostTarget::spiQueueRx(&hspi3, miso);
		REQUIRE(HostTarget::spiComplete(&hspi3));
		HostRtos::runUntilBlocked(task);
	}
	void read(const std::vector<uint8_t> &miso) {
		startRead();
		finishRead(miso);
	}
	/** Position of the last processed angle (getPos without starting a new read: a request is already given) */
	int32_t pos() { return enc->getPos(); }
};

} // namespace

TEST_SUITE("MtEncoderSPI") {

// ---------------------------------------------------------------- identity and creation

TEST_CASE_FIXTURE(MtFixture, "the MT encoder identifies itself as an absolute MagnTek SPI encoder") {
	create();
	CHECK(enc->getEncoderType() == EncoderType::absolute);
	CHECK(std::string(enc->getInfo().name) == "MagnTek SPI");
	CHECK(enc->getInfo().id == CLSID_ENCODER_MTSPI);
}

TEST_CASE_FIXTURE(MtFixture, "the MT encoder is not creatable while one exists and creatable again after deleting it") {
	REQUIRE(MtEncoderSPI::isCreatable());
	create();
	CHECK_FALSE(MtEncoderSPI::isCreatable());
	enc.reset();
	CHECK(MtEncoderSPI::isCreatable());
}

TEST_CASE_FIXTURE(MtFixture, "the MT encoder is not creatable without a free CS pin on the encoder SPI port") {
	for (const OutputPin &pin : std::vector<OutputPin>(ext3_spi.getCsPins())) {
		ext3_spi.reserveCsPin(pin);
	}
	CHECK_FALSE(MtEncoderSPI::isCreatable());
}

TEST_CASE_FIXTURE(MtFixture, "a new MT encoder does not take a CS pin used by another device" *
								 KNOWN_ISSUE("The constructor reserves the first free CS pin (MtEncoderSPI.cpp:24), but restoreFlash "
											 "(line 67) calls setCsPin with the stored index (0 without a stored configuration), "
											 "which frees that pin and takes the stored one without checking that it is free "
											 "(lines 111-117). Here SPI3_SS1 of another device is shared and freed again when the "
											 "encoder is deleted. Bug 92")) {
	MockSPIDevice other(ext3_spi, ext3_spi.getCsPins()[0]); // SPI3_SS1 taken
	create();
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(get("cs") != 1);
}

TEST_CASE_FIXTURE(MtFixture, "deleting the MT encoder frees its CS pin") {
	create();
	REQUIRE(cmd("cs", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE_FALSE(ext3_spi.isPinFree(ext3_spi.getCsPins()[1]));
	enc.reset();
	CHECK(ext3_spi.isPinFree(ext3_spi.getCsPins()[1]));
	CHECK(ext3_spi.getFreeCsPins().size() == 3);
}

TEST_CASE_FIXTURE(MtFixture, "deleting the MT encoder ends its thread") {
	create();
	read(mt6825(0x100));
	const size_t tasks = HostRtos::taskCount();
	enc.reset();
	CHECK(HostRtos::findTask("MTENC") == nullptr);
	CHECK(HostRtos::taskCount() == tasks - 1);
}

TEST_CASE_FIXTURE(MtFixture, "characterization: the MT encoder is creatable while a BiSS-C encoder uses the encoder SPI port exclusively") {
	// NEEDS REVIEW 103: MtEncoderSPI::isCreatable (MtEncoderSPI.h) only checks for a free CS pin, not the exclusive flag
	// that EncoderBissC sets ("bus must be always active and can't be shared", EncoderBissC.cpp configSPI). Two axes
	// could then share the port with the BiSS-C encoder, which does not take the port semaphore. Intended?
	auto biss = std::make_unique<EncoderBissC>();
	REQUIRE_FALSE(ext3_spi.hasFreePins());
	CHECK(MtEncoderSPI::isCreatable());
	biss.reset();
}

TEST_CASE_FIXTURE(MtFixture, "deleting the MT encoder during its DMA read leaves the encoder SPI port free" *
								 KNOWN_ISSUE("The angle read takes the port semaphore (SPIDevice::beginSpiTransfer, via "
											 "transmitReceive_DMA at MtEncoderSPI.cpp:193); only the completion gives it back, "
											 "through the port's current_device, i.e. the deleted encoder (bug 57: ~SPIDevice and "
											 "~MtEncoderSPI, MtEncoderSPI.cpp:49-52, do not end or detach the pending transfer). "
											 "Recorded under bug 57")) {
	create();
	startRead();
	REQUIRE(ext3_spi.isTaken());
	enc.reset();
	REQUIRE_ALL_HAL_MODELLED();

	CHECK_FALSE(ext3_spi.isTaken());
}

// ---------------------------------------------------------------- MT6825 angle decoding

TEST_CASE_FIXTURE(MtFixture, "the MT6825 angle read is a 4 byte DMA burst read from register 0x03") {
	create();
	read(mt6825(0));
	const HostTarget::SpiTransfer &t = HostTarget::spiTransfers(&hspi3).front();
	CHECK(t.call == HostTarget::SpiCall::TransmitReceive);
	CHECK(t.mode == HostTarget::BusMode::Dma);
	CHECK(t.tx == std::vector<uint8_t>{0x83, 0x00, 0x00, 0x00});
	CHECK(t.init.CLKPolarity == SPI_POLARITY_HIGH);
	CHECK(t.init.CLKPhase == SPI_PHASE_2EDGE);
	CHECK(t.init.FirstBit == SPI_FIRSTBIT_MSB);
}

TEST_CASE_FIXTURE(MtFixture, "the MT encoder drives its CS pin low during the read and high after it") {
	create();
	startRead();
	CHECK_FALSE(HostTarget::spiCsLevels(&hspi3)[0]);
	finishRead(mt6825(0));
	CHECK(HostTarget::spiCsLevels(&hspi3)[0]);
	CHECK_FALSE(ext3_spi.isTaken());
}

TEST_CASE_FIXTURE(MtFixture, "an MT6825 angle with valid parity sets the 18 bit position") {
	create();
	read(mt6825(0x1ABCD));
	CHECK(pos() == 0x1ABCD);
	CHECK(get("errors") == 0);
}

TEST_CASE_FIXTURE(MtFixture, "the MT6825 has 262144 counts per turn") {
	create();
	CHECK(enc->getCpr() == static_cast<uint32_t>(CPR6825));
}

TEST_CASE_FIXTURE(MtFixture, "an MT6825 angle with the no magnet warning and valid parity is decoded") {
	create();
	read(mt6825(0x0F0F0, true));
	CHECK(pos() == 0x0F0F0);
	CHECK(get("errors") == 0);
}

TEST_CASE_FIXTURE(MtFixture, "an MT6825 angle with a parity error in the upper bytes is counted as an error and keeps the position") {
	create();
	read(mt6825(0x100));
	read(mt6825(0x200, false, true));
	CHECK(get("errors") == 1);
	CHECK(pos() == 0x100);
}

TEST_CASE_FIXTURE(MtFixture, "an MT6825 angle with a parity error in the low byte is counted as an error and keeps the position") {
	create();
	read(mt6825(0x100));
	read(mt6825(0x200, false, false, true));
	CHECK(get("errors") == 1);
	CHECK(pos() == 0x100);
}

TEST_CASE_FIXTURE(MtFixture, "errors counts every angle read with a parity error") {
	create();
	read(mt6825(0x10, false, true));
	read(mt6825(0x10, false, true));
	CHECK(get("errors") == 2);
}

// ---------------------------------------------------------------- rotations

TEST_CASE_FIXTURE(MtFixture, "crossing from the end of a turn to its start counts one turn up") {
	create();
	read(mt6825(0x10000));
	read(mt6825(0x30000));
	read(mt6825(0x3FFF0));
	read(mt6825(0x00010));
	CHECK(pos() == CPR6825 + 0x10);
}

TEST_CASE_FIXTURE(MtFixture, "crossing from the start of a turn to its end counts one turn down") {
	create();
	read(mt6825(0x00010));
	read(mt6825(0x3FFF0));
	CHECK(pos() == 0x3FFF0 - CPR6825);
}

TEST_CASE_FIXTURE(MtFixture, "a move of less than half a turn does not count a turn") {
	create();
	read(mt6825(0x00001));
	read(mt6825(0x20000)); // + half a turn - 1
	read(mt6825(0x00001));
	CHECK(pos() == 1);
}

TEST_CASE_FIXTURE(MtFixture, "a parity error does not change the turn count") {
	create();
	read(mt6825(0x00010));
	read(mt6825(0x3FFF0, false, true));
	read(mt6825(0x00020));
	CHECK(pos() == 0x20);
}

TEST_CASE_FIXTURE(MtFixture, "characterization: a first angle above half a turn counts one turn down") {
	// NEEDS REVIEW 100: lastAngleInt starts at 0 (MtEncoderSPI.h), so the first angle above half a turn is taken as an
	// underflow (MtEncoderSPI.cpp:93-94): the position starts at angle - cpr. EncoderBissC does the same on purpose
	// relative to the stored offset (comment at EncoderBissC.cpp:57); MtEncoderSPI has no such comment and does not
	// use its stored offset here. Intended?
	create();
	read(mt6825(0x30000));
	CHECK(pos() == 0x30000 - CPR6825);
}

// ---------------------------------------------------------------- MT6835

TEST_CASE_FIXTURE(MtFixture, "mode set 1 selects the MT6835 with 2097152 counts per turn") {
	create();
	REQUIRE(cmd("mode", CMDtype::set, 1) == CommandStatus::OK);
	CHECK(get("mode") == 1);
	CHECK(enc->getCpr() == static_cast<uint32_t>(CPR6835));
}

TEST_CASE_FIXTURE(MtFixture, "the MT6835 angle read is a 6 byte DMA burst read from register 0x003") {
	create();
	REQUIRE(cmd("mode", CMDtype::set, 1) == CommandStatus::OK);
	read(mt6835(0));
	const HostTarget::SpiTransfer &t = HostTarget::spiTransfers(&hspi3).front();
	CHECK(t.mode == HostTarget::BusMode::Dma);
	CHECK(t.tx == std::vector<uint8_t>{0xA0, 0x03, 0x00, 0x00, 0x00, 0x00});
}

TEST_CASE_FIXTURE(MtFixture, "an MT6835 angle with a valid CRC sets the 21 bit position") {
	create();
	REQUIRE(cmd("mode", CMDtype::set, 1) == CommandStatus::OK);
	read(mt6835(0x0ABCDE, 0x5));
	CHECK(pos() == 0x0ABCDE);
	CHECK(get("errors") == 0);
}

TEST_CASE_FIXTURE(MtFixture, "an MT6835 angle with a wrong CRC is counted as an error and keeps the position") {
	create();
	REQUIRE(cmd("mode", CMDtype::set, 1) == CommandStatus::OK);
	read(mt6835(0x1000));
	read(mt6835(0x2000, 0, 0x01));
	CHECK(get("errors") == 1);
	CHECK(pos() == 0x1000);
}

TEST_CASE_FIXTURE(MtFixture, "an MT6835 angle with a flipped status bit fails the CRC check") {
	create();
	REQUIRE(cmd("mode", CMDtype::set, 1) == CommandStatus::OK);
	read(mt6835(0x1000));
	std::vector<uint8_t> miso = mt6835(0x2000);
	miso[4] ^= 0x01;
	read(miso);
	CHECK(get("errors") == 1);
}

TEST_CASE_FIXTURE(MtFixture, "MT6835 rotations count full 21 bit turns") {
	create();
	REQUIRE(cmd("mode", CMDtype::set, 1) == CommandStatus::OK);
	read(mt6835(0x080000));
	read(mt6835(0x180000));
	read(mt6835(0x1FFFF0));
	read(mt6835(0x000010));
	CHECK(pos() == CPR6835 + 0x10);
}

TEST_CASE_FIXTURE(MtFixture, "mode set resets the position, the turn count and the error count") {
	create();
	read(mt6825(0x10, false, true)); // 1 error
	read(mt6825(0x3FFF0));
	read(mt6825(0x00010)); // turn up
	REQUIRE(get("errors") == 1);
	REQUIRE(cmd("mode", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(get("errors") == 0);
	CHECK(pos() == 0);
	read(mt6825(0x00020));
	CHECK(pos() == 0x20);
}

TEST_CASE_FIXTURE(MtFixture, "mode set with an unknown mode keeps the current mode" *
								 KNOWN_ISSUE("The mode command casts any value to MtEncoderSPI_mode (MtEncoderSPI.cpp:335; "
											 "restoreFlash likewise at line 59): mode 2 gives getCpr() 0 (line 281), "
											 "updateAngleStatus starts no transfer (the thread then waits for a DMA that never "
											 "comes) and readSpi falls off its end without a return value (lines 129-141, "
											 "undefined). Bug 82")) {
	create();
	REQUIRE(cmd("mode", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(get("mode") == 0);
	CHECK(enc->getCpr() != 0);
}

TEST_CASE_FIXTURE(MtFixture, "mode info lists the supported encoders with their ids") {
	create();
	std::vector<CommandReply> replies;
	REQUIRE(cmd("mode", CMDtype::info, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().reply == "MT6825:0\nMT6835:1");
}

// ---------------------------------------------------------------- position and offset

TEST_CASE_FIXTURE(MtFixture, "pos set makes pos get return that position") {
	create();
	read(mt6825(0x1234));
	REQUIRE(cmd("pos", CMDtype::set, 1000) == CommandStatus::OK);
	CHECK(get("pos") == 1000);
}

TEST_CASE_FIXTURE(MtFixture, "positions after pos set move by the angle change") {
	create();
	read(mt6825(0x1234));
	enc->setPos(-50);
	read(mt6825(0x1234 + 100));
	CHECK(pos() == 50);
}

TEST_CASE_FIXTURE(MtFixture, "getPosAbs ignores the offset of pos set") {
	create();
	read(mt6825(0x1234));
	enc->setPos(0);
	CHECK(enc->getPosAbs() == 0x1234);
}

// ---------------------------------------------------------------- request timing

TEST_CASE_FIXTURE(MtFixture, "a position request starts a read only when no read is in progress") {
	create();
	startRead();
	enc->getPosAbs(); // in progress: no new request
	finishRead(mt6825(0x10));
	CHECK_FALSE(HostTarget::spiTransferPending(&hspi3));
	CHECK(HostTarget::spiTransfers(&hspi3).size() == 1);
}

TEST_CASE_FIXTURE(MtFixture, "a position request after a completed read starts the next read") {
	create();
	read(mt6825(0x10));
	enc->getPosAbs();
	HostRtos::runUntilBlocked(task);
	CHECK(HostTarget::spiTransferPending(&hspi3));
	finishRead(mt6825(0x10));
}

TEST_CASE_FIXTURE(MtFixture, "a position request while a read is in progress returns the last position") {
	create();
	read(mt6825(0x10));
	startRead();
	CHECK(enc->getPosAbs() == 0x10);
	finishRead(mt6825(0x20));
	CHECK(enc->getPosAbs() == 0x20);
}

TEST_CASE_FIXTURE(MtFixture, "the angle read waits until the encoder SPI port is free") {
	create();
	ext3_spi.takeSemaphore(); // another device is using the port
	enc->getPosAbs();
	HostRtos::runUntilBlocked(task);
	CHECK_FALSE(HostTarget::spiTransferPending(&hspi3));
	ext3_spi.giveSemaphore();
	HostRtos::runUntilBlocked(task);
	CHECK(HostTarget::spiTransferPending(&hspi3));
	finishRead(mt6825(0x10));
	CHECK(pos() == 0x10);
}

TEST_CASE_FIXTURE(MtFixture, "a position request more than 2 ms after the last update waits for the requested angle") {
	create();
	HostPlatform::advanceMs(3);
	PositionReader reader(*enc);
	HostRtos::runUntilBlocked(reader.GetHandle());
	CHECK_FALSE(reader.done);
	HostRtos::runUntilBlocked(task);
	finishRead(mt6825(0x55));
	HostRtos::runUntilBlocked(reader.GetHandle());
	CHECK(reader.done);
	CHECK(reader.result == 0x55);
}

TEST_CASE_FIXTURE(MtFixture, "a position request 2 ms after the last update returns the last position at once") {
	create();
	HostPlatform::advanceMs(2);
	PositionReader reader(*enc);
	HostRtos::runUntilBlocked(reader.GetHandle());
	CHECK(reader.done);
	CHECK(reader.result == 0);
	read(mt6825(0x55)); // complete the request
}

TEST_CASE_FIXTURE(MtFixture, "characterization: a request more than 2 ms after an update nobody waited for returns at once") {
	// NEEDS REVIEW 102 (see test_encoder_bissc.cpp): every processed angle gives waitForUpdateSem (MtEncoderSPI.cpp:106),
	// a later request older than 2 ms takes this stale signal at once (lines 268-269) and returns the old position
	create();
	read(mt6825(0x10));
	HostPlatform::advanceMs(3);
	PositionReader reader(*enc);
	HostRtos::runUntilBlocked(reader.GetHandle());
	CHECK(reader.done);
	CHECK(reader.result == 0x10);
	read(mt6825(0x20));
}

// ---------------------------------------------------------------- CS pin and speed

TEST_CASE_FIXTURE(MtFixture, "cs is 1 for the first CS pin by default") {
	create();
	CHECK(get("cs") == 1);
}

TEST_CASE_FIXTURE(MtFixture, "cs set 2 moves the encoder to the second CS pin of the encoder SPI port") {
	create();
	REQUIRE(cmd("cs", CMDtype::set, 2) == CommandStatus::OK);
	CHECK(get("cs") == 2);
	CHECK(ext3_spi.isPinFree(ext3_spi.getCsPins()[0]));
	CHECK_FALSE(ext3_spi.isPinFree(ext3_spi.getCsPins()[1]));
	startRead();
	const HostTarget::SpiTransfer &t = HostTarget::spiTransfers(&hspi3).back();
	CHECK_FALSE(t.csAtStart[1]); // SPI3_SS2 active
	finishRead(mt6825(0));
	CHECK(HostTarget::spiCsLevels(&hspi3)[1]);
}

TEST_CASE_FIXTURE(MtFixture, "cs set to a pin used by another device keeps the current pin" *
								 KNOWN_ISSUE("setCsPin (MtEncoderSPI.cpp:111-117) frees its pin and takes the new one without "
											 "checking that it is free (reserveCsPin's result is ignored): two devices then share "
											 "the pin, and deleting the encoder frees the other device's pin. SPIDevice::updateCSPin "
											 "has the check. Bug 92")) {
	MockSPIDevice other(ext3_spi, ext3_spi.getCsPins()[1]); // SPI3_SS2
	create();												// SPI3_SS1
	REQUIRE(get("cs") == 1);
	REQUIRE(cmd("cs", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(get("cs") == 1);
}

TEST_CASE_FIXTURE(MtFixture, "cs set beyond the last pin selects the last pin" *
								 doctest::description("KNOWN ISSUE (crash): setCsPin clips the index to getCsPins().size() (3) instead "
													  "of size()-1 (MtEncoderSPI.cpp:113) and dereferences getCsPin(3), a null pointer "
													  "(line 114). cs set 0 (index 255) does the same. Bug 81. Skipped like the other "
													  "crash known issues") *
								 doctest::skip()) {
	create();
	CHECK(cmd("cs", CMDtype::set, 4) == CommandStatus::OK);
	CHECK(get("cs") == 3);
}

TEST_CASE_FIXTURE(MtFixture, "a stored CS index beyond the last pin selects the last pin" *
								 doctest::description("KNOWN ISSUE (crash): restoreFlash decodes the CS index as 2 bits (0..3, "
													  "MtEncoderSPI.cpp:58) and setCsPin dereferences getCsPin(3), a null pointer "
													  "(lines 113-114). Bug 81. Skipped like the other crash known issues") *
								 doctest::skip()) {
	HostPlatform::flash()[ADR_MTENC_CONF1] = 3;
	create();
	CHECK(get("cs") == 3);
}

TEST_CASE("speed presets 1 and 2 select the SPI clock closest to 5 and 2.5 MHz") {
	struct Preset {
		int64_t preset;
		uint32_t prescaler;
	};
	// 42 MHz base clock: /4 = 10.5 MHz, /8 = 5.25 MHz, /16 = 2.625 MHz
	const Preset presets[] = {{1, SPI_BAUDRATEPRESCALER_8}, {2, SPI_BAUDRATEPRESCALER_16}};
	MtFixture f;
	f.create();
	CHECK(f.get("speed") == 0);
	for (const Preset &p : presets) {
		CAPTURE(p.preset);
		REQUIRE(f.cmd("speed", CMDtype::set, p.preset) == CommandStatus::OK);
		CHECK(f.get("speed") == p.preset);
		CHECK(hspi3.Init.BaudRatePrescaler == p.prescaler);
		f.startRead();
		CHECK(HostTarget::spiTransfers(&hspi3).back().init.BaudRatePrescaler == p.prescaler);
		f.finishRead(mt6825(0));
	}
}

TEST_CASE_FIXTURE(MtFixture, "characterization: speed preset 0 and the default clock the encoder at 10.5 MHz") {
	// NEEDS REVIEW 104: the constructor (MtEncoderSPI.cpp:26, comment "10MHz max") and preset 0 (spispeeds[0] = 10e6,
	// line 302) call getClosestPrescaler(10e6) without a maximum: on the 42 MHz port this selects /4 = 10.5 MHz, above
	// the stated 10 MHz maximum (/8 = 5.25 MHz would respect it). The MT6825/MT6835 themselves may allow more (to be
	// checked against the datasheets). Intended?
	create();
	CHECK(get("speed") == 0);
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_4);
	REQUIRE(cmd("speed", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(cmd("speed", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_4);
}

TEST_CASE_FIXTURE(MtFixture, "characterization: speed info lists the actual clock of every preset") {
	// NEEDS REVIEW 104 (preset 0 is 10.5 MHz)
	create();
	std::vector<CommandReply> replies;
	REQUIRE(cmd("speed", CMDtype::info, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 3);
	CHECK(replies[0].reply == "10500000.000000:0\n");
	CHECK(replies[1].reply == "5250000.000000:1\n");
	CHECK(replies[2].reply == "2625000.000000:2\n");
}

TEST_CASE_FIXTURE(MtFixture, "speed set beyond the last preset selects the last preset" *
								 doctest::description("KNOWN ISSUE (crash): setSpiSpeed clips the preset to spispeeds.size() (3) "
													  "instead of size()-1 (MtEncoderSPI.cpp:301) and reads spispeeds[3] of a 3 element "
													  "std::array (line 302). Out of bounds: aborts the run with _GLIBCXX_ASSERTIONS "
													  "(GCC 16 -O0). Bug 81 (same pattern as bugs 37/56/74). Skipped like the other crash "
													  "known issues") *
								 doctest::skip()) {
	create();
	CHECK(cmd("speed", CMDtype::set, 3) == CommandStatus::OK);
	CHECK(get("speed") == 2);
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_16);
}

TEST_CASE_FIXTURE(MtFixture, "a stored speed preset beyond the last selects the last preset" *
								 doctest::description("KNOWN ISSUE (crash): restoreFlash decodes the speed preset as 2 bits (0..3, "
													  "MtEncoderSPI.cpp:68) and setSpiSpeed reads spispeeds[3] (lines 301-302). Bug 81. "
													  "Skipped like the other crash known issues") *
								 doctest::skip()) {
	HostPlatform::flash()[ADR_MTENC_CONF1] = 3 << 2;
	create();
	CHECK(get("speed") == 2);
}

// ---------------------------------------------------------------- flash configuration

TEST_CASE_FIXTURE(MtFixture, "a stored configuration selects mode, CS pin, speed preset and the offset") {
	HostPlatform::flash()[ADR_MTENC_CONF1] = (1 << 8) | (1 << 2) | 2; // MT6835, preset 1, SPI3_SS3
	HostPlatform::flash()[ADR_MTENC_OFS] = 0x10;					  // << 5 for the MT6835
	create();
	CHECK(get("mode") == 1);
	CHECK(get("cs") == 3);
	CHECK(get("speed") == 1);
	CHECK(hspi3.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_8);
	CHECK(ext3_spi.isPinFree(ext3_spi.getCsPins()[0]));
	CHECK_FALSE(ext3_spi.isPinFree(ext3_spi.getCsPins()[2]));
	CHECK(enc->getPos() == -(0x10 << 5));
	read(mt6835(0x1000)); // complete the request of getPos
}

TEST_CASE_FIXTURE(MtFixture, "a stored MT6825 offset is scaled by 4") {
	HostPlatform::flash()[ADR_MTENC_OFS] = 0x10;
	create();
	read(mt6825(0x1000));
	CHECK(pos() == 0x1000 - (0x10 << 2));
}

TEST_CASE_FIXTURE(MtFixture, "saveFlash stores CS pin, speed preset, mode and the scaled offset") {
	create();
	REQUIRE(cmd("cs", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE(cmd("speed", CMDtype::set, 2) == CommandStatus::OK);
	REQUIRE(cmd("mode", CMDtype::set, 1) == CommandStatus::OK);
	read(mt6835(0x2000));
	enc->setPos(0); // offset 0x2000
	enc->saveFlash();
	CHECK(HostPlatform::flash()[ADR_MTENC_CONF1] == ((1 << 8) | (2 << 2) | 1));
	CHECK(HostPlatform::flash()[ADR_MTENC_OFS] == (0x2000 >> 5));
}

TEST_CASE_FIXTURE(MtFixture, "saveFlash stores the MT6825 offset divided by 4") {
	create();
	read(mt6825(0x2000));
	enc->setPos(0);
	enc->saveFlash();
	CHECK(HostPlatform::flash()[ADR_MTENC_CONF1] == 0);
	CHECK(HostPlatform::flash()[ADR_MTENC_OFS] == (0x2000 >> 2));
}

TEST_CASE_FIXTURE(MtFixture, "a saved configuration is restored by the next MT encoder") {
	create();
	REQUIRE(cmd("cs", CMDtype::set, 3) == CommandStatus::OK);
	REQUIRE(cmd("speed", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(cmd("mode", CMDtype::set, 1) == CommandStatus::OK);
	enc->saveFlash();
	enc.reset();
	create();
	CHECK(get("cs") == 3);
	CHECK(get("speed") == 1);
	CHECK(get("mode") == 1);
}

// ---------------------------------------------------------------- register access and save

TEST_CASE_FIXTURE(MtFixture, "reg getat reads an MT6825 register with a 2 byte read command") {
	SystemCommands::debugMode = true; // reg and save are debug commands
	create();
	HostTarget::spiQueueRx(&hspi3, {0x00, 0xA5});
	std::vector<CommandReply> replies;
	REQUIRE(cmd("reg", CMDtype::getat, 0, &replies, 0x09) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == 0xA5);
	const HostTarget::SpiTransfer &t = HostTarget::spiTransfers(&hspi3).back();
	CHECK(t.mode == HostTarget::BusMode::Blocking);
	CHECK(t.tx == std::vector<uint8_t>{0x89, 0x00});
	CHECK_FALSE(ext3_spi.isTaken());
}

TEST_CASE_FIXTURE(MtFixture, "reg setat writes an MT6825 register with a 2 byte write command") {
	SystemCommands::debugMode = true; // reg and save are debug commands
	create();
	REQUIRE(cmd("reg", CMDtype::setat, 0x5A, nullptr, 0x09) == CommandStatus::OK);
	const HostTarget::SpiTransfer &t = HostTarget::spiTransfers(&hspi3).back();
	CHECK(t.call == HostTarget::SpiCall::Transmit);
	CHECK(t.tx == std::vector<uint8_t>{0x09, 0x5A});
}

TEST_CASE_FIXTURE(MtFixture, "reg getat reads an MT6835 register with a 3 byte read command") {
	SystemCommands::debugMode = true; // reg and save are debug commands
	create();
	REQUIRE(cmd("mode", CMDtype::set, 1) == CommandStatus::OK);
	HostTarget::spiQueueRx(&hspi3, {0x00, 0x00, 0x3C});
	std::vector<CommandReply> replies;
	REQUIRE(cmd("reg", CMDtype::getat, 0, &replies, 0x0A) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == 0x3C);
	CHECK(HostTarget::spiTransfers(&hspi3).back().tx == std::vector<uint8_t>{0x30, 0x0A, 0x00});
}

TEST_CASE_FIXTURE(MtFixture, "reg setat writes an MT6835 register with a 3 byte write command") {
	SystemCommands::debugMode = true; // reg and save are debug commands
	create();
	REQUIRE(cmd("mode", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(cmd("reg", CMDtype::setat, 0x77, nullptr, 0x0A) == CommandStatus::OK);
	const HostTarget::SpiTransfer &t = HostTarget::spiTransfers(&hspi3).back();
	CHECK(t.call == HostTarget::SpiCall::Transmit);
	CHECK(t.tx == std::vector<uint8_t>{0x60, 0x0A, 0x77});
}

TEST_CASE_FIXTURE(MtFixture, "characterization: an MT6835 register address above 0xFF loses its upper bits in the command") {
	// NEEDS REVIEW 101: readSpi / writeSpi build the first byte as (uint8_t)((addr & 0xf00) | 0x30) (MtEncoderSPI.cpp:136,
	// 148): addr & 0xf00 is cut by the uint8_t cast, so address bits 11..8 never reach the encoder (expected
	// 0x30 | (addr >> 8) per the MT6835 12 bit register address). Harmless if only registers below 0x100 are used?
	SystemCommands::debugMode = true; // reg and save are debug commands
	create();
	REQUIRE(cmd("mode", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(cmd("reg", CMDtype::getat, 0, nullptr, 0x10A) == CommandStatus::OK);
	CHECK(HostTarget::spiTransfers(&hspi3).back().tx == std::vector<uint8_t>{0x30, 0x0A, 0x00});
}

TEST_CASE_FIXTURE(MtFixture, "reg and save are only available in debug mode") {
	create();
	CHECK(enc->getCommandFromName("reg") == nullptr);
	CHECK(enc->getCommandFromName("save") == nullptr);
	SystemCommands::debugMode = true;
	CHECK(enc->getCommandFromName("reg") != nullptr);
	CHECK(enc->getCommandFromName("save") != nullptr);
}

TEST_CASE_FIXTURE(MtFixture, "save is not supported by the MT6825 and sends nothing") {
	SystemCommands::debugMode = true; // reg and save are debug commands
	create();
	std::vector<CommandReply> replies;
	REQUIRE(cmd("save", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == 0);
	CHECK(HostTarget::spiTransfers(&hspi3).empty());
}

TEST_CASE_FIXTURE(MtFixture, "save programs the MT6835 EEPROM and reports the acknowledge 0x55") {
	SystemCommands::debugMode = true; // reg and save are debug commands
	create();
	REQUIRE(cmd("mode", CMDtype::set, 1) == CommandStatus::OK);
	HostTarget::spiQueueRx(&hspi3, {0x00, 0x00, 0x55});
	std::vector<CommandReply> replies;
	REQUIRE(cmd("save", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == 1);
	CHECK(HostTarget::spiTransfers(&hspi3).back().tx == std::vector<uint8_t>{0xC0, 0x00, 0x00});
}

TEST_CASE_FIXTURE(MtFixture, "save reports 0 when the MT6835 does not acknowledge") {
	SystemCommands::debugMode = true; // reg and save are debug commands
	create();
	REQUIRE(cmd("mode", CMDtype::set, 1) == CommandStatus::OK);
	HostTarget::spiQueueRx(&hspi3, {0x00, 0x00, 0x00});
	std::vector<CommandReply> replies;
	REQUIRE(cmd("save", CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == 0);
}

// ---------------------------------------------------------------- commands

TEST_CASE_FIXTURE(MtFixture, "errors is 0 for a new encoder") {
	create();
	CHECK(get("errors") == 0);
}

TEST_CASE_FIXTURE(MtFixture, "commands reject request types they do not support") {
	SystemCommands::debugMode = true;
	create();
	CHECK(cmd("cs", CMDtype::info) == CommandStatus::ERR);
	CHECK(cmd("pos", CMDtype::info) == CommandStatus::ERR);
	CHECK(cmd("mode", CMDtype::getat) == CommandStatus::ERR);
	CHECK(cmd("speed", CMDtype::setat) == CommandStatus::ERR);
	CHECK(cmd("reg", CMDtype::get) == CommandStatus::ERR);
	CHECK(HostTarget::spiTransfers(&hspi3).empty());
}

TEST_CASE_FIXTURE(MtFixture, "a command id without a handler is rejected") {
	create();
	ParsedCommand parsed;
	parsed.cmdId = 99;
	parsed.type = CMDtype::get;
	parsed.target = enc.get();
	std::vector<CommandReply> replies;
	CHECK(enc->command(parsed, replies) == CommandStatus::NOT_FOUND);
	CHECK(replies.empty());
}

} // TEST_SUITE
