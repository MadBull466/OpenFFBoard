/*
 * Tests of EncoderLocal (UserExtensions/Src/EncoderLocal.cpp): the local quadrature (ABN) encoder of the F407 board on
 * TIM_ENC (htim3, encoder interface mode, ARR 0xffff) with the index pin ENCODER_Z (EXTI, falling edge)
 * (ffb_target_tests).
 *
 * The quadrature edges are simulated with HostTarget::encoderTimerCount (counter wraps raise the update interrupt,
 * HAL_TIM_PeriodElapsedCallback -> TimerHandler::timerElapsed), the index pulse with HostTarget::encoderIndexPulse.
 * Flash: ADR_ENCLOCAL_CPR (counts per turn), ADR_ENCLOCAL_OFS (index offset, 0 = index off).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "HostPlatform.h"
#include "TestHelpers.h"

#include "EncoderLocal.h"
#include "ExtiHandler.h"
#include "TimerHandler.h"
#include "eeprom_addresses.h"
#include "flash_helpers.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr uint32_t CENTRE = 0x7fff; // Counter value of position 0 (EncoderLocal.cpp:25, 67)

struct LocalEncoderFixture : TargetFixture {
	/** Executes a command of the encoder like the command thread does (class commands only) */
	static CommandStatus cmd(EncoderLocal &enc, const std::string &name, CMDtype type, int64_t val = 0,
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
	static int64_t get(EncoderLocal &enc, const std::string &name) {
		std::vector<CommandReply> replies;
		CHECK(cmd(enc, name, CMDtype::get, 0, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies.front().val;
	}
	/** Executes a command implemented by CommandHandler (help, id, name, ...) like the command thread does */
	static CommandStatus internal(EncoderLocal &enc, const std::string &name, CMDtype type, std::vector<CommandReply> &replies) {
		CmdHandlerCommanddef *def = enc.getCommandFromName(name);
		REQUIRE_MESSAGE(def != nullptr, "command " << name << " not registered");
		ParsedCommand parsed;
		parsed.cmdId = def->cmdId;
		parsed.type = type;
		parsed.target = &enc;
		return enc.internalCommand(parsed, replies);
	}
	/** Moves the encoder by `counts` quadrature counts */
	static void turn(int32_t counts) { HostTarget::encoderTimerCount(&htim3, counts); }
	/** Mathematical modulo (result in 0..m-1) */
	static int64_t mod(int64_t value, int64_t m) { return ((value % m) + m) % m; }

	/**
	 * First session: index enabled with `cpr` counts per turn, index pulse, `moved` counts away from the index,
	 * setPos(newPos), saveFlash, encoder deleted. Then a new encoder (like after a restart) restores the configuration,
	 * moves `bootMoved` counts and sees the index again.
	 * @return the position the restored encoder reports at the index
	 */
	static int32_t indexPositionAfterRestart(uint32_t cpr, int32_t moved, int32_t newPos, int32_t bootMoved = 777) {
		{
			EncoderLocal first;
			REQUIRE(cmd(first, "cpr", CMDtype::set, cpr) == CommandStatus::OK);
			REQUIRE(cmd(first, "index", CMDtype::set, 1) == CommandStatus::OK);
			HostTarget::encoderIndexPulse();
			turn(moved);
			first.setPos(newPos);
			REQUIRE(first.getPos() == newPos);
			first.saveFlash();
		}
		EncoderLocal second;
		turn(bootMoved);
		HostTarget::encoderIndexPulse();
		REQUIRE_ALL_HAL_MODELLED();
		return second.getPos();
	}
};

struct TwoAxisFixture : AxisFixture {
	TwoAxisFixture() : AxisFixture(2) {}
};

/** True if the list holds a pointer with this address (only the pointer values are compared) */
template <class T>
bool listed(const std::vector<T *> &list, std::uintptr_t address) {
	return std::any_of(list.begin(), list.end(), [address](T *p) { return reinterpret_cast<std::uintptr_t>(p) == address; });
}

} // namespace

TEST_SUITE("EncoderLocal") {

// ---------------- Identity and creation ----------------

TEST_CASE_FIXTURE(LocalEncoderFixture, "the local encoder is an incremental encoder named Local ABN with the local encoder class id") {
	EncoderLocal enc;
	CHECK(std::string(enc.getInfo().name) == "Local ABN");
	CHECK(enc.getInfo().id == CLSID_ENCODER_LOCAL);
	CHECK(enc.getEncoderType() == EncoderType::incremental);
	CHECK(enc.getHelpstring() == "Local ABN encoder\n");
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "only one local encoder can be created at a time") {
	REQUIRE(EncoderLocal::isCreatable());
	auto enc = std::make_unique<EncoderLocal>();
	CHECK_FALSE(EncoderLocal::isCreatable());
	enc.reset();
	CHECK(EncoderLocal::isCreatable());
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "creating the local encoder starts the encoder timer with the update interrupt at the centre count") {
	HostTarget::timerSetCounter(&htim3, 1234);
	EncoderLocal enc;
	CHECK(HostTarget::timerRunning(&htim3));
	CHECK(HostTarget::timerUpdateIrqEnabled(&htim3));
	CHECK(HostTarget::timerCounter(&htim3) == CENTRE);
	CHECK(enc.getTimerCount() == 0);
	CHECK(enc.getPos() == 0);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "without a stored configuration the local encoder has 0 counts per turn and the index disabled") {
	EncoderLocal enc;
	CHECK(enc.getCpr() == 0);
	CHECK(get(enc, "cpr") == 0);
	CHECK(get(enc, "index") == 0);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "a local encoder created after a deleted one counts from the centre again") {
	{
		EncoderLocal first;
		turn(500);
	}
	REQUIRE_FALSE(HostTarget::timerRunning(&htim3));
	EncoderLocal second; // HAL_TIM_Base_Start_IT returns HAL_ERROR (State still BUSY), the constructor enables CEN itself
	CHECK(HostTarget::timerRunning(&htim3));
	CHECK(second.getPos() == 0);
	turn(-20);
	CHECK(second.getPos() == -20);
}

// ---------------- Counting ----------------

TEST_CASE_FIXTURE(LocalEncoderFixture, "the position counts quadrature counts in both directions from the centre") {
	EncoderLocal enc;
	turn(1000);
	CHECK(enc.getPos() == 1000);
	CHECK(enc.getTimerCount() == 1000);
	turn(-3000);
	CHECK(enc.getPos() == -2000);
	CHECK(enc.getTimerCount() == -2000);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "the absolute position of the local encoder equals its position") {
	EncoderLocal enc;
	turn(-4321);
	CHECK(enc.getPosAbs() == -4321);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "the position stays continuous over several timer overflows") {
	EncoderLocal enc;
	CHECK(HostTarget::encoderTimerCount(&htim3, 3 * 0x10000 + 5) == 3);
	CHECK(enc.getPos() == 3 * 0x10000 + 5);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "the position stays continuous over several timer underflows") {
	EncoderLocal enc;
	CHECK(HostTarget::encoderTimerCount(&htim3, -(2 * 0x10000 + 7)) == 2);
	CHECK(enc.getPos() == -(2 * 0x10000 + 7));
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "turning back over an overflow returns to the earlier position") {
	EncoderLocal enc;
	turn(0x9000); // overflow
	turn(-0x9000); // underflow back
	CHECK(enc.getPos() == 0);
	CHECK(HostTarget::timerCounter(&htim3) == CENTRE);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "with a shorter timer period the position stays continuous over an overflow") {
	EncoderLocal enc;
	enc.setPeriod(40000); // ARR = 39999 (counter at 0x7fff)
	CHECK(htim3.Instance->ARR == 39999);
	CHECK(HostTarget::encoderTimerCount(&htim3, 10000) == 1);
	CHECK(enc.getPos() == 10000);
	CHECK(HostTarget::encoderTimerCount(&htim3, -10000) == 1);
	CHECK(enc.getPos() == 0);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "an update interrupt of another timer does not change the position") {
	EncoderLocal enc;
	turn(10);
	REQUIRE(HAL_TIM_Base_Start_IT(&htim13) == HAL_OK);
	REQUIRE(HostTarget::timerUpdate(&htim13));
	CHECK(enc.getPos() == 10);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "setPos sets the current position and keeps counting from it") {
	EncoderLocal enc;
	turn(250);
	enc.setPos(-1000);
	CHECK(enc.getPos() == -1000);
	turn(300);
	CHECK(enc.getPos() == -700);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "setPos keeps the position continuous over a later overflow") {
	EncoderLocal enc;
	enc.setPos(5);
	turn(0x8001);
	CHECK(enc.getPos() == 5 + 0x8001);
}

// ---------------- Index ----------------

TEST_CASE_FIXTURE(LocalEncoderFixture, "the first index pulse with the index enabled centres the counter and pulses the system LED") {
	EncoderLocal enc;
	REQUIRE(cmd(enc, "index", CMDtype::set, 1) == CommandStatus::OK);
	turn(-555);
	const uint32_t pulses = HostPlatform::leds().sysPulses;
	HostTarget::encoderIndexPulse();
	CHECK(HostTarget::timerCounter(&htim3) == CENTRE);
	CHECK(enc.getPos() == 0);
	CHECK(HostPlatform::leds().sysPulses == pulses + 1);
	turn(42);
	CHECK(enc.getPos() == 42);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "only the first index pulse centres the local encoder") {
	EncoderLocal enc;
	REQUIRE(cmd(enc, "index", CMDtype::set, 1) == CommandStatus::OK);
	HostTarget::encoderIndexPulse();
	turn(4096); // one turn later the index passes again
	const uint32_t pulses = HostPlatform::leds().sysPulses;
	HostTarget::encoderIndexPulse();
	CHECK(enc.getPos() == 4096);
	CHECK(HostTarget::timerCounter(&htim3) == CENTRE + 4096);
	CHECK(HostPlatform::leds().sysPulses == pulses);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "an index pulse with the index disabled does not change the position") {
	EncoderLocal enc;
	turn(321);
	HostTarget::encoderIndexPulse();
	CHECK(enc.getPos() == 321);
	CHECK(HostTarget::timerCounter(&htim3) == CENTRE + 321);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "a rising edge on the index pin does not centre the encoder") {
	EncoderLocal enc;
	REQUIRE(cmd(enc, "index", CMDtype::set, 1) == CommandStatus::OK);
	turn(77);
	HostTarget::setPin(ENCODER_Z_GPIO_Port, ENCODER_Z_Pin, true);
	HostTarget::triggerExti(ENCODER_Z_Pin); // pin reads high: not the falling edge
	CHECK(enc.getPos() == 77);
	HostTarget::encoderIndexPulse(); // the following falling edge still centres
	CHECK(enc.getPos() == 0);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "an external interrupt of another pin does not centre the encoder") {
	EncoderLocal enc;
	REQUIRE(cmd(enc, "index", CMDtype::set, 1) == CommandStatus::OK);
	turn(88);
	HostTarget::setPin(E_STOP_GPIO_Port, E_STOP_Pin, false);
	HostTarget::setPin(ENCODER_Z_GPIO_Port, ENCODER_Z_Pin, false);
	HostTarget::triggerExti(E_STOP_Pin);
	CHECK(enc.getPos() == 88);
	CHECK(HostTarget::timerCounter(&htim3) == CENTRE + 88);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "characterization: enabling the index after an index pulse was seen never centres the encoder") {
	// NEEDS REVIEW 94: indexHit is set by every falling edge, also while the index is disabled (EncoderLocal.cpp:108-109),
	// and never cleared. Enabling the index later (command or a restored configuration only applies at creation) never
	// centres the encoder until the next restart. Intended?
	EncoderLocal enc;
	HostTarget::encoderIndexPulse(); // seen while disabled
	REQUIRE(cmd(enc, "index", CMDtype::set, 1) == CommandStatus::OK);
	turn(100);
	HostTarget::encoderIndexPulse();
	CHECK(enc.getPos() == 100);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "the index offset restored from flash is the position reported at the index") {
	REQUIRE(Flash_Write(ADR_ENCLOCAL_CPR, 4096));
	REQUIRE(Flash_Write(ADR_ENCLOCAL_OFS, static_cast<uint16_t>(-300)));
	EncoderLocal enc;
	CHECK(get(enc, "index") == 1);
	turn(1234);
	HostTarget::encoderIndexPulse();
	CHECK(enc.getPos() == -300);
	turn(300);
	CHECK(enc.getPos() == 0);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "with the index restored from flash creating the encoder does not reset the position") {
	// EncoderLocal.cpp:29-30: setPos(0) only without index. Both paths start at position 0 (pos = 0 at line 26)
	REQUIRE(Flash_Write(ADR_ENCLOCAL_OFS, 10));
	EncoderLocal enc;
	CHECK(enc.getPos() == 0);
	turn(5);
	CHECK(enc.getPos() == 5);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "setPos before the index pulse only changes the current position") {
	REQUIRE(Flash_Write(ADR_ENCLOCAL_CPR, 4096));
	REQUIRE(Flash_Write(ADR_ENCLOCAL_OFS, 50));
	EncoderLocal enc;
	turn(10);
	enc.setPos(1000);
	CHECK(enc.getPos() == 1000);
	HostTarget::encoderIndexPulse(); // still centres with the stored offset
	CHECK(enc.getPos() == 50);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "zeroing after the index makes the restored encoder report the same index position modulo a turn") {
	// Index 100 counts before the zero point: the index is at -100 (mod 4096) in the user's frame
	const int32_t atIndex = indexPositionAfterRestart(4096, 100, 0);
	CHECK(mod(atIndex - (-100), 4096) == 0);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "zeroing before the index makes the restored encoder report the same index position modulo a turn") {
	// Zero point 250 counts before the index: the index is at +250 in the user's frame
	const int32_t atIndex = indexPositionAfterRestart(4096, -250, 0);
	CHECK(mod(atIndex - 250, 4096) == 0);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "zeroing after the index with a counts per turn that is not a power of two restores the index position modulo a turn" *
				  KNOWN_ISSUE("EncoderLocal.cpp:85 `(idxpos-cnt-pos) % this->cpr` converts the negative int32 to uint32 "
							  "(cpr is uint32_t): with cpr 1000 the offset of an index 100 counts before the zero point "
							  "becomes (2^32 - 100) % 1000 = 196 instead of 900 (or -100)")) {
	const int32_t atIndex = indexPositionAfterRestart(1000, 100, 0);
	CHECK(mod(atIndex - (-100), 1000) == 0);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "setting a nonzero position after the index restores the index position modulo a turn" *
				  KNOWN_ISSUE("EncoderLocal.cpp:85 subtracts the new position (`idxpos-cnt-pos`, `pos` is the parameter): "
							  "the index is at newPos - moved in the user's frame, the stored offset is -moved - newPos. "
							  "With setPos(500) 100 counts after the index the restored index reads 3496 instead of 400 (mod 4096)")) {
	const int32_t atIndex = indexPositionAfterRestart(4096, 100, 500);
	CHECK(mod(atIndex - 400, 4096) == 0);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "setting the position after the index with 0 counts per turn is safe" * doctest::skip() *
				  doctest::description("KNOWN ISSUE (crash): EncoderLocal.cpp:85 `% this->cpr` with cpr 0 (the default "
									   "without a stored cpr) is a division by zero (SIGFPE on x86; on the Cortex-M4 UDIV "
									   "returns 0 unless DIV_0_TRP is set). Reachable: index on, index pulse, zeroenc / pos "
									   "command. Skipped")) {
	EncoderLocal enc;
	REQUIRE(cmd(enc, "index", CMDtype::set, 1) == CommandStatus::OK);
	HostTarget::encoderIndexPulse();
	turn(10);
	enc.setPos(0);
	CHECK(enc.getPos() == 0);
}

// ---------------- Flash ----------------

TEST_CASE_FIXTURE(LocalEncoderFixture, "saveFlash stores the counts per turn and the index offset") {
	EncoderLocal enc;
	REQUIRE(cmd(enc, "cpr", CMDtype::set, 2048) == CommandStatus::OK);
	REQUIRE(cmd(enc, "index", CMDtype::set, 1) == CommandStatus::OK);
	HostTarget::encoderIndexPulse();
	turn(-20);
	enc.setPos(0); // index 20 counts after the zero point: offset 20
	enc.saveFlash();
	uint16_t cpr = 0, ofs = 0;
	REQUIRE(Flash_Read(ADR_ENCLOCAL_CPR, &cpr));
	REQUIRE(Flash_Read(ADR_ENCLOCAL_OFS, &ofs));
	CHECK(cpr == 2048);
	CHECK(ofs == 20);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "saveFlash with the index disabled stores the offset 0") {
	EncoderLocal enc;
	REQUIRE(cmd(enc, "cpr", CMDtype::set, 600) == CommandStatus::OK);
	enc.saveFlash();
	uint16_t cpr = 0, ofs = 1;
	REQUIRE(Flash_Read(ADR_ENCLOCAL_CPR, &cpr));
	REQUIRE(Flash_Read(ADR_ENCLOCAL_OFS, &ofs));
	CHECK(cpr == 600);
	CHECK(ofs == 0);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "saveFlash after disabling the index stores the offset 0") {
	EncoderLocal enc;
	REQUIRE(cmd(enc, "cpr", CMDtype::set, 4096) == CommandStatus::OK);
	REQUIRE(cmd(enc, "index", CMDtype::set, 1) == CommandStatus::OK);
	HostTarget::encoderIndexPulse();
	turn(40);
	enc.setPos(0); // offset 4056 (-40 mod 4096)
	REQUIRE(cmd(enc, "index", CMDtype::set, 0) == CommandStatus::OK);
	enc.saveFlash();
	uint16_t ofs = 1;
	REQUIRE(Flash_Read(ADR_ENCLOCAL_OFS, &ofs));
	CHECK(ofs == 0);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "a new local encoder restores the counts per turn and the index setting") {
	{
		EncoderLocal first;
		REQUIRE(cmd(first, "cpr", CMDtype::set, 8192) == CommandStatus::OK);
		REQUIRE(cmd(first, "index", CMDtype::set, 1) == CommandStatus::OK);
		HostTarget::encoderIndexPulse();
		turn(30);
		first.setPos(0);
		first.saveFlash();
	}
	EncoderLocal second;
	CHECK(second.getCpr() == 8192);
	CHECK(get(second, "index") == 1);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "characterization: an enabled index whose offset is 0 is restored as disabled") {
	// NEEDS REVIEW 93: EncoderLocal.cpp:49 / 61 use the stored offset 0 as "index off". An index exactly at the zero
	// point (or an enabled index that was never used to zero the encoder) is lost after saving and restoring.
	{
		EncoderLocal first;
		REQUIRE(cmd(first, "cpr", CMDtype::set, 4096) == CommandStatus::OK);
		REQUIRE(cmd(first, "index", CMDtype::set, 1) == CommandStatus::OK);
		first.saveFlash();
	}
	EncoderLocal second;
	CHECK(get(second, "index") == 0);
}

// ---------------- Commands ----------------

TEST_CASE_FIXTURE(LocalEncoderFixture, "the cpr command sets and reads the counts per turn") {
	EncoderLocal enc;
	CHECK(cmd(enc, "cpr", CMDtype::set, 4000) == CommandStatus::OK);
	CHECK(enc.getCpr() == 4000);
	CHECK(get(enc, "cpr") == 4000);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "characterization: the cpr command accepts more counts per turn than flash can store") {
	// NEEDS REVIEW 95: setCpr takes uint32_t (EncoderLocal.cpp:129), saveFlash writes Flash_Write(uint16_t) (line 48):
	// 70000 works until the next restart and then comes back as its low 16 bits. Should the command limit cpr to 65535?
	EncoderLocal enc;
	CHECK(cmd(enc, "cpr", CMDtype::set, 70000) == CommandStatus::OK);
	CHECK(enc.getCpr() == 70000);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "the index command enables and disables the index and reads the setting") {
	EncoderLocal enc;
	CHECK(cmd(enc, "index", CMDtype::set, 1) == CommandStatus::OK);
	CHECK(get(enc, "index") == 1);
	CHECK(cmd(enc, "index", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(get(enc, "index") == 0);
	turn(9);
	HostTarget::encoderIndexPulse();
	CHECK(enc.getPos() == 9);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "the index command rejects an info request") {
	EncoderLocal enc;
	std::vector<CommandReply> replies;
	CHECK(cmd(enc, "index", CMDtype::info, 0, &replies) == CommandStatus::ERR);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "the cpr and index commands are registered for get and set only") {
	EncoderLocal enc;
	CmdHandlerCommanddef *cpr = enc.getCommandFromName("cpr");
	CmdHandlerCommanddef *index = enc.getCommandFromName("index");
	REQUIRE(cpr != nullptr);
	REQUIRE(index != nullptr);
	CHECK(cpr->flags == (CMDFLAG_GET | CMDFLAG_SET));
	CHECK(index->flags == (CMDFLAG_GET | CMDFLAG_SET));
	CHECK(enc.getCommandFromName("cprx") == nullptr);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "a command id the local encoder does not know is not found") {
	EncoderLocal enc;
	ParsedCommand parsed;
	parsed.cmdId = 2; // after cpr (0) and useindex (1)
	parsed.type = CMDtype::get;
	parsed.target = &enc;
	std::vector<CommandReply> replies;
	CHECK(enc.command(parsed, replies) == CommandStatus::NOT_FOUND);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "the command handler of the local encoder is named localenc and its help lists its commands") {
	EncoderLocal enc;
	CHECK(CommandHandler::getHandlerFromClassName("localenc") == &enc);
	std::vector<CommandReply> replies;
	REQUIRE(internal(enc, "help", CMDtype::get, replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	const std::string &help = replies.front().reply;
	CHECK(help.find("Local ABN encoder") != std::string::npos);
	CHECK(help.find("cpr") != std::string::npos);
	CHECK(help.find("Enable index pin") != std::string::npos);
	replies.clear();
	REQUIRE(internal(enc, "id", CMDtype::get, replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == CLSID_ENCODER_LOCAL);
}

// ---------------- Deletion and the axis ----------------

TEST_CASE_FIXTURE(LocalEncoderFixture, "a local encoder registers itself for timer and external interrupts") {
	EncoderLocal enc;
	CHECK(listed(ExtiHandler::extiHandlers, reinterpret_cast<std::uintptr_t>(static_cast<ExtiHandler *>(&enc))));
	CHECK(TimerHandler::timerHandlers.size() == 1);
}

TEST_CASE_FIXTURE(LocalEncoderFixture, "a deleted local encoder no longer reacts to timer and index interrupts") {
	const size_t extiBefore = ExtiHandler::extiHandlers.size();
	auto enc = std::make_unique<EncoderLocal>();
	REQUIRE(ExtiHandler::extiHandlers.size() == extiBefore + 1);
	enc.reset();
	CHECK(ExtiHandler::extiHandlers.size() == extiBefore);
	CHECK(TimerHandler::timerHandlers.empty());
	CHECK_FALSE(HostTarget::timerRunning(&htim3));
	HostTarget::encoderIndexPulse(); // nothing left to call
	CHECK(HostTarget::encoderTimerCount(&htim3, 0x20000) == 0); // stopped counter: no counts, no interrupt
}

TEST_CASE_FIXTURE(TwoAxisFixture, "the encoder chooser of a second axis does not create a second local encoder") {
	Encoder *first = selectFirmwareEncoder(2, 0);
	REQUIRE(dynamic_cast<EncoderLocal *>(first) != nullptr);
	CHECK_FALSE(EncoderLocal::isCreatable());
	CHECK(selectFirmwareEncoder(2, 1) == nullptr);
	CHECK(axis(0).getEncoder() == first);
	selectFirmwareEncoder(0, 0);
	CHECK(EncoderLocal::isCreatable());
}

TEST_CASE_FIXTURE(AxisFixture, "selecting no encoder on the axis deletes the local encoder and stops the encoder timer") {
	Encoder *enc = selectFirmwareEncoder(2);
	REQUIRE(dynamic_cast<EncoderLocal *>(enc) != nullptr);
	REQUIRE(HostTarget::timerRunning(&htim3));
	selectFirmwareEncoder(0);
	CHECK_FALSE(HostTarget::timerRunning(&htim3));
	CHECK(TimerHandler::timerHandlers.empty());
	CHECK(EncoderLocal::isCreatable());
	CHECK(CommandHandler::getHandlerFromClassName("localenc") == nullptr);
}

TEST_CASE_FIXTURE(AxisFixture, "the axis position follows the local encoder over a timer overflow") {
	selectFirmwareEncoder(2);
	HostTarget::encoderTimerCount(&htim3, 0x12345);
	CHECK(axis().getEncoder()->getPos() == 0x12345);
	selectFirmwareEncoder(0);
}

} // TEST_SUITE
