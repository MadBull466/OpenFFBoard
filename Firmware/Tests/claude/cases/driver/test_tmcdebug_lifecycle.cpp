/*
 * TMCDebugBridge, group TDL (phase P13, T2): life cycle, identity, class chooser, PID dead code.
 * Requirements R3.1 - R3.5 and R4.4 of Tests/claude/charters/driver/TMCDEBUG_REQUIREMENTS.md.
 *
 * Unit under test: TMCDebugBridge::TMCDebugBridge() (TMCDebugBridge.cpp:236-252), ~TMCDebugBridge() (:254-256),
 * the identity data (:15-23) and the entry in mainclass_chooser.cpp (behind TMCDEBUG).
 *
 * Oracles (see the charter):
 *  - DRV / DS: what the constructor must do to the chip. The seven limit registers 0x5C - 0x62 and their layout are the
 *    ones the approved D1 test "[R42] setLimits ..." uses (DS 0x5D - 0x62 p96-97; 0x5C is undocumented in Rev 2.07,
 *    charter Q1). The values are those of `tmclimits` in TMCDebugBridge.h (32767, 30000, 32767, 50, 2147483647,
 *    -2147483647, 2147483647), written as 32 bit register words (hand derivation next to each assertion).
 *    Flash words are the ones of the D1 flash tests (0x320 motor config, 0x321 cpr; layout in test_tmc4671_flash.cpp).
 *  - HD: hand derivations in comments. FW: the firmware's own behaviour, tests named "characterization:" (NEEDS REVIEW).
 *  - TMCL: not used here (no frames in this group).
 *
 * What the group can observe: the bridge keeps its driver in a private member, so everything is checked at the fake
 * chip (frame log, register file), at the fake GPIO port (DRV_ENABLE, PE7), in the fake kernel (task "TMC", task
 * count) and through the class registry. The TMC_1 constructor itself also calls restoreFlash(), so the frame log
 * holds two restores before the limits: one of the driver constructor and one of the bridge (hand count per test).
 *
 * Host vs target: int64 / int32 conversions of the limits are plain 32 bit register words on both. `-2147483647` as a
 * register word is 0x80000001 (two's complement, HD). The destructor order (bridge body first, then the member `drv`)
 * is the same on target. No float, no packing, no byte-order dependence in this file.
 *
 * R3.2 is a combined bridge + driver property: the bridge destructor writes DRV_ENABLE low (TDB:255), and the member
 * `drv` is destroyed right after the body, whose TMC4671 destructor resets the same pin (TMC4671.cpp:82-85). The fake GPIO
 * port keeps only the output level (ODR, no write history), so the bridge's own line is not independently observable:
 * the cases check the pin is low after the bridge is destroyed (NEEDS REVIEW: a bridge-only mutant of that line survives).
 * R3.1 "address 1": `drv->setAddress(1)` (TDB:243) is an equivalent mutant, the TMC_1 constructor already uses address 1,
 * so it is not independently testable (a wrong address such as 2 is detected: flash words of another address are read).
 * R3.3 stays `characterization:`: the class id is persisted in the flash as the main class selection, but the value 11 is
 * only given by a code comment (ClassIDs.h:42), not by a specification. Name and visibility are covered by the registry
 * cases of R3.4.
 *
 * Not tested, and why:
 *  - "no alignment started" beyond the register evidence (no ABN set-up write after the limits): the driver state is in
 *    the private `drv`; running the driver thread through findTask() would need the power, ADC and state machine of the
 *    D1 fixtures on an object the test cannot reach (see "Lead decisions needed" of the report).
 *  - that the bridge's own destructor write to DRV_ENABLE is needed (see R3.2 above).
 *  - the EncoderInit state transition that setEncoderType(NONE) skips: the driver state is private; the encoder case
 *    uses the status mask write (DS 0x7D STATUS_MASK, TMC4671.cpp:2599 via setEncoderType) as register evidence.
 *  - R4.4 is a property of the whole TMCDebugBridge test set: the case here are run under ASan + UBSan, the result is in
 *    the report.
 */
#include "doctest.h"
#include "TmcDebugFixture.h"

#include "HostRtos.h"
#include "HostTarget.h"
#include "SystemCommands.h"
#include "mainclass_chooser.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

extern ClassChooser<FFBoardMain> mainchooser; // cppmain.cpp

namespace {

using Frame = FakeTmc4671::Frame;

bool enablePinHigh() { return HostTarget::outputPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin); }

/** Index of the first write frame to `addr` in `log`, or -1 */
int firstWrite(const std::vector<Frame> &log, uint8_t addr) {
	for (size_t i = 0; i < log.size(); i++) {
		if (log[i].write && log[i].addr == addr) {
			return static_cast<int>(i);
		}
	}
	return -1;
}

/** Index of the last write frame to `addr` in `log`, or -1 */
int lastWrite(const std::vector<Frame> &log, uint8_t addr) {
	for (size_t i = log.size(); i > 0; i--) {
		if (log[i - 1].write && log[i - 1].addr == addr) {
			return static_cast<int>(i - 1);
		}
	}
	return -1;
}

/** Activates the driver thread "TMC" (found by name, the bridge keeps its driver private) for `ms` of simulated time */
void runDriverThread(uint32_t ms) {
	TaskHandle_t task = HostRtos::findTask("TMC");
	REQUIRE(task != nullptr);
	if (!HostRtos::isResumable(task)) {
		HostRtos::makeResumable(task);
	}
	const uint64_t end = HostPlatform::nowUs() + uint64_t(ms) * 1000;
	while (HostPlatform::nowUs() < end) {
		const uint64_t before = HostPlatform::nowUs();
		HostRtos::runUntilBlocked(task);
		if (HostPlatform::nowUs() == before) {
			HostPlatform::advanceMs(1);
		}
	}
}

/** Flash words (D1 flash tests, test_tmc4671_flash.cpp): TMC_1 motor config 0x320, cpr 0x321 */
constexpr uint16_t FLASH_MOTCONF = 0x320;
constexpr uint16_t FLASH_CPR = 0x321;

/**
 * Motor config word of D1 (test_tmc4671_flash.cpp header): bits 1:0 motor type, bit 2 = !svpwm, bits 5:3 encoder
 * (1 = abn), bits 15:6 pole pairs. BLDC 3 + svpwm (bit 2 = 0) + abn 1 << 3 = 0x08 + 7 pole pairs << 6 = 0x1C0
 * = 0x3 + 0x8 + 0x1C0 = 0x1CB (HD). A driver restored from this word has encoder type abn.
 */
constexpr uint16_t MOTCONF_BLDC_ABN_7PP = 0x1CB;

} // namespace

TEST_SUITE("TMCDebugBridge lifecycle") {

// ---------------------------------------------------------------------------------------------------------------------
// R3.1 constructor
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.1 address] the constructor talks to the chip on chip select PA4 and to no other chip") {
	// TMC_1 is the driver at chip select PA4 (index 0 of the fake), TMC_2 is PB0 (index 1)
	REQUIRE(chip1.log().empty());
	auto bridge = makeBridge();
	CHECK_FALSE(chip1.log().empty());
	CHECK(chip2.log().empty());
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.1 limits] the constructor writes the seven limit registers 0x5C - 0x62 from tmclimits") {
	auto bridge = makeBridge();
	// HD, values of tmclimits (TMCDebugBridge.h) as 32 bit register words:
	// 0x5C ddt 32767 = 0x7FFF; 0x5D uq/ud 30000 = 0x7530; 0x5E torque/flux 32767 = 0x7FFF; 0x5F acceleration 50 = 0x32;
	// 0x60 velocity 2147483647 = 0x7FFFFFFF; 0x61 position low -2147483647 = 0x80000001 (two's complement);
	// 0x62 position high 2147483647 = 0x7FFFFFFF.
	CHECK(chip1.reg(0x5C) == 0x7FFFu);
	CHECK(chip1.reg(0x5D) == 0x7530u);
	CHECK(chip1.reg(0x5E) == 0x7FFFu);
	CHECK(chip1.reg(0x5F) == 0x32u);
	CHECK(chip1.reg(0x60) == 0x7FFFFFFFu);
	CHECK(chip1.reg(0x61) == 0x80000001u);
	CHECK(chip1.reg(0x62) == 0x7FFFFFFFu);
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.1 limits] each limit register is written exactly once by the bridge") {
	auto bridge = makeBridge();
	// setLimits() runs once (the driver constructor and restoreFlash() write none of the limit registers)
	for (uint8_t addr : {0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62}) {
		CAPTURE(int(addr));
		CHECK(chip1.writesTo(addr).size() == 1);
	}
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.1 restore] the stored cpr is restored from flash and written to the chip before the limits are written") {
	HostPlatform::flash()[FLASH_CPR] = 4000;
	auto bridge = makeBridge();
	const auto &log = chip1.log();
	const int firstLimit = firstWrite(log, 0x5C);
	const int lastCpr = lastWrite(log, 0x26);
	REQUIRE(firstLimit >= 0);
	REQUIRE(lastCpr >= 0);
	// restoreFlash() -> setCpr(4000) writes ABN PPR 0x26 (DS 0x26 p81, D1 test of setCpr) and AENC PPR 0x40.
	// The last restore write must be earlier than the first limit write (TDB:245 before :246).
	CHECK(lastCpr < firstLimit);
	CHECK(log[lastCpr].data == 4000u);
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.1 restore] the bridge restores the flash a second time after the driver constructor did") {
	HostPlatform::flash()[FLASH_CPR] = 4000;
	auto bridge = makeBridge();
	// HD: only the cpr word is in the flash, so one restore writes 0x26 once (setCpr). The TMC_1 constructor restores
	// once, TMCDebugBridge::TMCDebugBridge() restores again (TDB:245): two writes.
	const auto writes = chip1.writesTo(0x26);
	REQUIRE(writes.size() == 2);
	CHECK(writes[0] == 4000u);
	CHECK(writes[1] == 4000u);
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.1 restore] the motor configuration restored from flash is in effect after construction, not the default MainConfig") {
	// Why the second restore matters: `drv->conf = tmcconf` (TDB:242) replaces the configuration the TMC_1 constructor
	// restored, so only the bridge's own restoreFlash() (TDB:245) brings the stored motor type and pole pairs back.
	// Flash 0x320 (D1 layout): BLDC 3 | svpwm (bit 2 = 0) | encoder NONE (0) | 7 pole pairs << 6 = 0x1C0 -> 0x1C3 (HD).
	// The driver thread initialises the chip from `conf`: DS 0x1B N_POLE_PAIRS [15:0], MOTOR_TYPE [23:16]: 7 | 3 << 16
	// = 0x00030007 (HD, same as the D1 test "[R21] a BLDC motor with 7 pole pairs ..."). The chip must see that
	// word as its last write to 0x1B, not the default configuration's.
	HostPlatform::flash()[FLASH_MOTCONF] = 0x1C3;
	chip1.setAdcRaw(0, 0x7FFF7FFF);
	auto bridge = makeBridge();
	runDriverThread(3000);
	const int idx = lastWrite(chip1.log(), 0x1B);
	REQUIRE(idx >= 0);
	CHECK(chip1.log()[static_cast<size_t>(idx)].data == 0x00030007u);
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.1 encoder] the encoder type is set to NONE after the restore: a stored abn encoder is not set up again after the limits") {
	HostPlatform::flash()[FLASH_MOTCONF] = MOTCONF_BLDC_ABN_7PP;
	HostPlatform::flash()[FLASH_CPR] = 4000;
	auto bridge = makeBridge();
	const auto &log = chip1.log();
	const int firstLimit = firstWrite(log, 0x5C);
	REQUIRE(firstLimit >= 0);
	// Precondition: the restore applied the stored abn encoder (setup_ABN_Enc writes ABN_DECODER_MODE 0x25, DS 0x25 p81)
	// before the limits. Without it the test would prove nothing.
	REQUIRE(firstWrite(log, 0x25) >= 0);
	REQUIRE(firstWrite(log, 0x25) < firstLimit);
	// setEncoderType() ends with setStatusMask() (TMC4671.cpp:1533-1545 -> STATUS_MASK 0x7D, DS 0x7D): the bridge's call
	// (TDB:247) is the only one after the limits, and with type NONE it does not follow with an encoder set-up.
	// A missing call or one made before restoreFlash() leaves no 0x7D write after the first limit write.
	REQUIRE(lastWrite(log, 0x7D) > firstLimit);
	// setEncoderType(NONE) after the limits must not set up the abn decoder again (no 0x25 / 0x26 / 0x29 frame)
	for (size_t i = static_cast<size_t>(firstLimit); i < log.size(); i++) {
		INFO("frame " << i << " after the first limit write: addr 0x" << std::hex << int(log[i].addr));
		CHECK_FALSE((log[i].write && (log[i].addr == 0x25 || log[i].addr == 0x26 || log[i].addr == 0x29)));
	}
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.1 thread] the constructor starts the driver thread named TMC") {
	REQUIRE(HostRtos::findTask("TMC") == nullptr);
	const size_t before = HostRtos::taskCount();
	auto bridge = makeBridge();
	CHECK(HostRtos::findTask("TMC") != nullptr);
	CHECK(HostRtos::taskCount() > before); // the other tasks belong to the fixture / the command and USB infrastructure
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.1 enable] the constructor drives the gate driver enable pin DRV_ENABLE high") {
	HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
	REQUIRE_FALSE(enablePinHigh());
	auto bridge = makeBridge();
	CHECK(enablePinHigh());
}

// ---------------------------------------------------------------------------------------------------------------------
// R3.2 destructor
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.2 destructor] DRV_ENABLE is low after the bridge is destroyed (bridge + driver destructor together)") {
	auto bridge = makeBridge();
	REQUIRE(enablePinHigh());
	bridge.reset();
	CHECK_FALSE(enablePinHigh());
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.2 destructor] destroying the bridge removes the driver thread") {
	const size_t before = HostRtos::taskCount();
	auto bridge = makeBridge();
	REQUIRE(HostRtos::findTask("TMC") != nullptr);
	bridge.reset();
	CHECK(HostRtos::findTask("TMC") == nullptr);
	CHECK(HostRtos::taskCount() == before);
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.2 destructor] a bridge can be created again after it was destroyed") {
	auto first = makeBridge();
	first.reset();
	chip1.clearLog();
	auto second = makeBridge();
	// the second life cycle writes the limits again and leaves the pin high
	CHECK(chip1.writesTo(0x5C).size() == 1);
	CHECK(enablePinHigh());
	CHECK(HostRtos::findTask("TMC") != nullptr);
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.2 destructor] destroying the bridge sends no register frame to the chip") {
	auto bridge = makeBridge();
	const size_t frames = chip1.log().size();
	bridge.reset();
	CHECK(chip1.log().size() == frames);
}

// ---------------------------------------------------------------------------------------------------------------------
// R3.3 identity
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.3 identity] characterization: the class id of the bridge is 11") {
	// NEEDS REVIEW: the value 11 (0xB, ClassIDs.h:42) is a firmware choice, saved in the board's flash as the main class
	// selection; no specification. T1 checks the id against the macro, this checks the number.
	auto bridge = makeBridge();
	CHECK(bridge->getInfo().id == 11);
	CHECK(CLSID_MAIN_TMCDBG == 11);
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.3 identity] characterization: getHelpstring returns the fixed TMCL-IDE hint") {
	// NEEDS REVIEW: the text is the firmware's own (TMCDebugBridge.h, getHelpstring); no specification.
	auto bridge = makeBridge();
	CHECK(bridge->getHelpstring() ==
		  "Compatible with TMCL-IDE. To use the FOC modes position, velocity and torque you first must manually select the "
		  "encoder and wait for alignment.");
}

// ---------------------------------------------------------------------------------------------------------------------
// R3.4 class chooser
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.4 chooser] the class registry contains exactly one entry with the id of the bridge") {
	const auto entries = std::count_if(class_registry.begin(), class_registry.end(),
									   [](const class_entry<FFBoardMain> &e) { return e.selectionId == CLSID_MAIN_TMCDBG; });
	CHECK(entries == 1);
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.4 chooser] the registry entry of the bridge carries its class identifier and is creatable") {
	const class_entry<FFBoardMain> *found = nullptr;
	for (const class_entry<FFBoardMain> &e : class_registry) {
		if (e.selectionId == CLSID_MAIN_TMCDBG) {
			found = &e;
		}
	}
	REQUIRE(found != nullptr);
	CHECK(&found->info == &TMCDebugBridge::info);
	CHECK(std::string(found->info.name) == "TMC Debug Bridge");
	CHECK(found->isCreatable());
	CHECK(mainchooser.isValidClassId(CLSID_MAIN_TMCDBG));
	CHECK(mainchooser.isCreatable(CLSID_MAIN_TMCDBG));
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.4 chooser] the chooser creates a TMCDebugBridge with the selection id 11 and destroys it again") {
	chip1.setPresent(true);
	powerOn();
	const size_t before = HostRtos::taskCount();
	{
		std::unique_ptr<FFBoardMain> main(mainchooser.Create(CLSID_MAIN_TMCDBG));
		REQUIRE(main != nullptr);
		CHECK(dynamic_cast<TMCDebugBridge *>(main.get()) != nullptr);
		CHECK(main->getSelectionID() == CLSID_MAIN_TMCDBG);
		CHECK(HostRtos::findTask("TMC") != nullptr);
	}
	// destroyed through the base class pointer (virtual destructor): pin low (bridge + driver together), thread gone
	CHECK_FALSE(enablePinHigh());
	CHECK(HostRtos::taskCount() == before);
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.4 chooser] the class list shows the bridge in debug mode as creatable with its id and name") {
	const bool saved = SystemCommands::debugMode;
	SystemCommands::debugMode = true;
	std::vector<CommandReply> replies;
	mainchooser.replyAvailableClasses(replies);
	SystemCommands::debugMode = saved;
	// reply format "<id>:<creatable 1/0>:<name>" (ClassChooser.h replyAvailableClasses)
	const auto it = std::find_if(replies.begin(), replies.end(), [](const CommandReply &r) { return r.reply == "11:1:TMC Debug Bridge"; });
	CHECK(it != replies.end());
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.4 chooser] the class list hides the bridge when the debug mode is off (class visibility debug)") {
	const bool saved = SystemCommands::debugMode;
	SystemCommands::debugMode = false;
	std::vector<CommandReply> replies;
	mainchooser.replyAvailableClasses(replies);
	SystemCommands::debugMode = saved;
	const auto it = std::find_if(replies.begin(), replies.end(), [](const CommandReply &r) { return r.reply == "11:1:TMC Debug Bridge"; });
	CHECK(it == replies.end());
}

// ---------------------------------------------------------------------------------------------------------------------
// R3.5 PID dead code
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.5 pids] characterization: the constructor does not write the PID registers, tmcpids is dead code") {
	// NEEDS REVIEW: setPids(tmcpids) is commented out (TDB:244). The four PID registers (DS 0x54 FLUX_P_FLUX_I,
	// 0x56 TORQUE_P_TORQUE_I, 0x58 VELOCITY, 0x5A POSITION) get no write frame at all.
	auto bridge = makeBridge();
	for (uint8_t addr : {0x54, 0x56, 0x58, 0x5A}) {
		CAPTURE(int(addr));
		CHECK(chip1.writesTo(addr).empty());
	}
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.5 pids] characterization: after construction the PID registers hold the chip defaults, not the tmcpids values") {
	// NEEDS REVIEW. HD: tmcpids (TMCDebugBridge.h) would give 0x54 = fluxI | fluxP << 16 = 400 | 400 << 16 = 0x01900190,
	// 0x56 = 400 | 300 << 16 = 0x012C0190, 0x58 = 10 | 500 << 16 = 0x01F4000A, 0x5A = 0 | 40 << 16 = 0x00280000.
	auto bridge = makeBridge();
	// the != checks below are only meaningful if the chip default is not the tmcpids word
	REQUIRE(FakeTmc4671::defaultValue(0x54) != 0x01900190u);
	REQUIRE(FakeTmc4671::defaultValue(0x56) != 0x012C0190u);
	REQUIRE(FakeTmc4671::defaultValue(0x58) != 0x01F4000Au);
	REQUIRE(FakeTmc4671::defaultValue(0x5A) != 0x00280000u);
	CHECK(chip1.reg(0x54) == FakeTmc4671::defaultValue(0x54));
	CHECK(chip1.reg(0x56) == FakeTmc4671::defaultValue(0x56));
	CHECK(chip1.reg(0x58) == FakeTmc4671::defaultValue(0x58));
	CHECK(chip1.reg(0x5A) == FakeTmc4671::defaultValue(0x5A));
	CHECK(chip1.reg(0x54) != 0x01900190u);
	CHECK(chip1.reg(0x56) != 0x012C0190u);
	CHECK(chip1.reg(0x58) != 0x01F4000Au);
	CHECK(chip1.reg(0x5A) != 0x00280000u);
	CHECK(chip1.reg(0x63) == FakeTmc4671::defaultValue(0x63)); // sequential PI bit untouched as well
}

TEST_CASE_FIXTURE(TmcDebugFixture, "[R3.5 pids] characterization: with PID words in the flash the constructor still writes no PID register") {
	// NEEDS REVIEW. TMC_1 flash words (test_tmc4671_flash.cpp): 0x327 torque P, 0x328 torque I, 0x329 flux P, 0x32A flux I.
	// The registers are written by setPids / initialize, which the bridge never calls (TDB:244, :248 commented out).
	// Only the "not written" half is observed here (the driver's curPids is private).
	HostPlatform::flash()[0x327] = 0x0123;
	HostPlatform::flash()[0x328] = 0x0456;
	HostPlatform::flash()[0x329] = 0x0789;
	HostPlatform::flash()[0x32A] = 0x0ABC;
	auto bridge = makeBridge();
	for (uint8_t addr : {0x54, 0x56}) {
		CAPTURE(int(addr));
		CHECK(chip1.writesTo(addr).empty());
		CHECK(chip1.reg(addr) == FakeTmc4671::defaultValue(addr));
	}
	// 0x63 bit 31 is SEQUENTIAL_PI (setSequentialPI, updateReg(0x63, .., 0x1, 31), TMC4671.cpp:2011-2013): setPids also
	// writes it, the constructor does not
	CHECK(chip1.writesTo(0x63).empty());
}

} // TEST_SUITE
