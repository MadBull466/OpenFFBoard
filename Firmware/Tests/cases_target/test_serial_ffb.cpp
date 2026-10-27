/*
 * Tests of SerialFFB (FFBoard/Src/SerialFFB.cpp): the "fxm" command handler that manages the effects of an
 * EffectsCalculator directly with commands. FFBHIDExt (main class "HID Gamepad (Ext FFB)") creates two of them
 * (instance 0 and 1) on one shared EffectsCalculator, so a host program drives FFB over the CDC string interface
 * instead of HID PID reports.
 *
 * Command syntax (string interface): get "fxm.0.cmd?", get with address "fxm.0.cmd?adr", set "fxm.0.cmd=val",
 * set with address "fxm.0.cmd=val?adr"; the address is the effect index returned by "new".
 *
 * - Unit cases (SerialFfbFixture): a SerialFFB on its own EffectsCalculator; commands through CommandHandler::command
 *   (CommandHelpers::run) like the command thread, the resulting state read from the calculator's public effect array
 *   (what EffectsCalculator::calculateEffects uses).
 * - Force cases (SerialForceFixture): the SerialFFB on the EffectsCalculator of AxisFixture (real Axis, MockMotorDriver,
 *   MockEncoder). Where SerialFFB claims the same meaning as the HID PID path (HidFFB), the same effect is created
 *   first over HID PID reports (PidHost) and then over fxm commands in the same test and the axis torques are compared.
 * - End to end: command lines over the CDC command interface (CommandHelpers::CdcConsole) with a failsafe FFBoardMain
 *   or with FFBHIDExt (those cases delete an FFBHIDMain: DELETES_FFBHIDMAIN, suspected bug 24).
 *
 * Suspected bugs 122-127, NEEDS REVIEW 129-134 (Tests/PROGRESS.md).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "CommandHelpers.h"
#include "HostPlatform.h"
#include "TestHelpers.h"

#include "EffectsCalculator.h"
#include "FFBHIDExt.h"
#include "SerialFFB.h"

#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

namespace {

/** Executes an fxm command like the command thread does (CommandHandler::command with a ParsedCommand) */
CommandStatus fxmCmd(SerialFFB &fxm, const std::string &name, CMDtype type, int64_t val = 0, int64_t adr = 0,
					 std::vector<CommandReply> *replies = nullptr) {
	return CommandHelpers::run(fxm, name, type, val, replies, adr);
}

/** Reads a value of effect `adr` with a getat command. Fails the test unless the command answers OK with one reply */
int64_t fxmGetAt(SerialFFB &fxm, const std::string &name, int64_t adr) {
	std::vector<CommandReply> replies;
	CHECK(fxmCmd(fxm, name, CMDtype::getat, 0, adr, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	return replies.front().val;
}

/** Creates an effect with "new" and returns the reply (the index or -1) */
int64_t fxmNew(SerialFFB &fxm, int64_t type) {
	std::vector<CommandReply> replies;
	REQUIRE(fxmCmd(fxm, "new", CMDtype::set, type, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	return replies.front().val;
}

/** A SerialFFB (instance 0) on its own EffectsCalculator */
struct SerialFfbFixture : TargetFixture {
	std::shared_ptr<EffectsCalculator> calc = std::make_shared<EffectsCalculator>();
	std::unique_ptr<SerialFFB> fxm = std::make_unique<SerialFFB>(calc);

	CommandStatus run(const std::string &name, CMDtype type, int64_t val = 0, int64_t adr = 0, std::vector<CommandReply> *replies = nullptr) {
		return fxmCmd(*fxm, name, type, val, adr, replies);
	}
	CommandStatus setAt(const std::string &name, int64_t adr, int64_t val) { return run(name, CMDtype::setat, val, adr); }
	int64_t getAt(const std::string &name, int64_t adr) { return fxmGetAt(*fxm, name, adr); }
	int64_t create(int64_t type) { return fxmNew(*fxm, type); }
	FFB_Effect &effect(size_t i) { return calc->effects.at(i); }
};

/**
 * The SerialFFB on the EffectsCalculator of AxisFixture (real axes with mocks). Like test_ffb_integration.cpp:
 * full scale torque path, internal axis damper 0, 1100 cycles at the center so the fade-in is over.
 */
struct SerialForceFixture : AxisFixture {
	explicit SerialForceFixture(uint8_t axisCount = 1) : AxisFixture(axisCount) {
		for (uint8_t i = 0; i < axisCount; i++) {
			setFullScale(i);
			axisCommand(Axis_commands::axisdamper, CMDtype::set, 0, nullptr, 0, i);
		}
		run(1100);
	}
	SerialFFB fxm{effects, 0};

	CommandStatus setAt(const std::string &name, int64_t adr, int64_t val) { return fxmCmd(fxm, name, CMDtype::setat, val, adr); }
	int64_t create(int64_t type) { return fxmNew(fxm, type); }
	/** Starts effect `idx` and switches FFB on like a host program does */
	void start(int64_t idx) {
		REQUIRE(setAt("state", idx, 1) == CommandStatus::OK);
		REQUIRE(fxmCmd(fxm, "ffbstate", CMDtype::set, 1) == CommandStatus::OK);
	}
	/** Removes all effects of the HID path (FFB stays on: with FFB off the axis adds its idle spring) before the same effect is made over fxm */
	void clearHidEffects() {
		for (uint16_t i = 0; i < EffectsCalculator::max_effects; i++) {
			effects->free_effect(i);
		}
	}
};

struct SerialForceFixture2Axes : SerialForceFixture {
	SerialForceFixture2Axes() : SerialForceFixture(2) {}
};

} // namespace

TEST_SUITE("SerialFFB") {

// ============================================================================================ identity, command table

TEST_CASE_FIXTURE(SerialFfbFixture, "the effects manager is the command handler fxm with the name Effects manager") {
	CHECK(std::string(fxm->getCommandHandlerInfo()->clsname) == "fxm");
	CHECK(fxm->getInfo().id == CLSID_EFFECTSMGR);
	CHECK(std::string(fxm->getInfo().name) == "Effects manager");
	CHECK(CommandHandler::getHandlerFromClassName("fxm", 0) == fxm.get());
}

TEST_CASE_FIXTURE(SerialFfbFixture, "the effects manager registers its effect commands with their access types") {
	struct Entry {
		const char *name;
		uint32_t flags;
	};
	const Entry expected[] = {
		{"ffbstate", CMDFLAG_GET | CMDFLAG_SET},		{"type", CMDFLAG_GETADR},
		{"reset", CMDFLAG_GET | CMDFLAG_GETADR},		{"new", CMDFLAG_SET | CMDFLAG_INFOSTRING},
		{"mag", CMDFLAG_SETADR | CMDFLAG_GETADR},		{"state", CMDFLAG_SETADR | CMDFLAG_GETADR},
		{"period", CMDFLAG_SETADR | CMDFLAG_GETADR},	{"duration", CMDFLAG_SETADR | CMDFLAG_GETADR},
		{"offset", CMDFLAG_SETADR | CMDFLAG_GETADR},	{"deadzone", CMDFLAG_SETADR | CMDFLAG_GETADR},
		{"sat", CMDFLAG_SETADR | CMDFLAG_GETADR},		{"coeff", CMDFLAG_SETADR | CMDFLAG_GETADR},
		{"axisgain", CMDFLAG_SETADR | CMDFLAG_GETADR},
	};
	for (const Entry &e : expected) {
		CAPTURE(e.name);
		CmdHandlerCommanddef *def = fxm->getCommandFromName(e.name);
		REQUIRE(def != nullptr);
		CHECK(def->flags == e.flags);
	}
}

TEST_CASE_FIXTURE(SerialFfbFixture, "characterization: the effects manager has no command for phase, envelope, start delay, effect gain, direction or ramp levels") {
	// NEEDS REVIEW 134: HID PID sets these per effect (HidFFB set_effect / set_envelope / set_ramp / set_periodic phase).
	// Over fxm they keep the defaults of a new effect: a ramp created with new=2 has start and end level 0 (no force),
	// envelopes and phases can not be used, and the global gain is only reset to 255 by "reset" (no gain command).
	for (const char *name : {"phase", "attack", "fade", "delay", "startdelay", "gain", "direction", "startlevel", "endlevel"}) {
		CAPTURE(name);
		CHECK(fxm->getCommandFromName(name) == nullptr);
	}
}

TEST_CASE_FIXTURE(SerialFfbFixture, "an unknown command id is not found by the effects manager") {
	ParsedCommand parsed;
	parsed.cmdId = 0x7777;
	parsed.type = CMDtype::get;
	parsed.target = fxm.get();
	std::vector<CommandReply> replies;
	CHECK(fxm->command(parsed, replies) == CommandStatus::NOT_FOUND);
	CHECK(replies.empty());
}

// ============================================================================================ ffbstate, gain, reset

TEST_CASE_FIXTURE(SerialFfbFixture, "ffbstate switches the effects calculator on and off") {
	REQUIRE_FALSE(calc->isActive());

	CHECK(run("ffbstate", CMDtype::set, 1) == CommandStatus::OK);
	CHECK(calc->isActive());

	CHECK(run("ffbstate", CMDtype::set, 0) == CommandStatus::OK);
	CHECK_FALSE(calc->isActive());
}

TEST_CASE_FIXTURE(SerialFfbFixture, "ffbstate reads the state of the effects calculator") {
	calc->setActive(true);
	CHECK(CommandHelpers::get(*fxm, "ffbstate") == 1);
	CHECK(fxm->getFfbActive());
	calc->setActive(false);
	CHECK(CommandHelpers::get(*fxm, "ffbstate") == 0);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "set_gain sets the global gain of the effects calculator") {
	fxm->set_gain(100);
	CHECK(calc->getGain() == 100);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "start_FFB and stop_FFB of the control interface switch the calculator") {
	EffectsControlItf &itf = *fxm;
	itf.start_FFB();
	CHECK(calc->isActive());
	itf.stop_FFB();
	CHECK_FALSE(calc->isActive());
}

TEST_CASE_FIXTURE(SerialFfbFixture, "reset frees all effects") {
	REQUIRE(create(FFB_EFFECT_CONSTANT) == 0);
	REQUIRE(create(FFB_EFFECT_SPRING) == 1);
	REQUIRE(create(FFB_EFFECT_SINE) == 2);

	CHECK(run("reset", CMDtype::get) == CommandStatus::OK);

	for (size_t i = 0; i < 3; i++) {
		CAPTURE(i);
		CHECK(effect(i).type == FFB_EFFECT_NONE);
		CHECK(effect(i).state == 0);
	}
}

TEST_CASE_FIXTURE(SerialFfbFixture, "reset switches FFB off and restores the full global gain") {
	calc->setActive(true);
	calc->setGain(17);

	CHECK(run("reset", CMDtype::get) == CommandStatus::OK);

	CHECK_FALSE(calc->isActive());
	CHECK(calc->getGain() == 255);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "reset with an effect address frees only that effect") {
	REQUIRE(create(FFB_EFFECT_CONSTANT) == 0);
	REQUIRE(create(FFB_EFFECT_SINE) == 1);
	REQUIRE(setAt("mag", 0, 1234) == CommandStatus::OK);

	CHECK(run("reset", CMDtype::getat, 0, 1) == CommandStatus::OK);

	CHECK(effect(1).type == FFB_EFFECT_NONE);
	CHECK(effect(0).type == FFB_EFFECT_CONSTANT);
	CHECK(effect(0).magnitude == 1234);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "a freed effect slot is reused by the next new effect") {
	for (int i = 0; i < 4; i++) {
		REQUIRE(create(FFB_EFFECT_CONSTANT) == i);
	}
	REQUIRE(run("reset", CMDtype::getat, 0, 2) == CommandStatus::OK);

	CHECK(create(FFB_EFFECT_SPRING) == 2);
	CHECK(create(FFB_EFFECT_SPRING) == 4);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "reset with an effect address beyond 16 bit does not free effect 0" *
				  KNOWN_ISSUE("bug 123: SerialFFB.cpp:140 passes the 64 bit address to free_effect(uint16_t): 65536 "
							  "becomes 0 and frees effect 0; the address is not checked against the effect count "
							  "(answers OK for any address) unlike all other fxm commands")) {
	REQUIRE(create(FFB_EFFECT_CONSTANT) == 0);

	CommandStatus status = run("reset", CMDtype::getat, 0, 65536);

	CHECK(effect(0).type == FFB_EFFECT_CONSTANT); // defect: freed
	CHECK(status == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "reset rejects a set request") {
	REQUIRE(create(FFB_EFFECT_CONSTANT) == 0);
	CHECK(run("reset", CMDtype::set, 1) == CommandStatus::ERR);
	CHECK(effect(0).type == FFB_EFFECT_CONSTANT);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "deleting the effects manager frees the effects of the calculator") {
	REQUIRE(create(FFB_EFFECT_CONSTANT) == 0);
	calc->setActive(true);

	fxm.reset();

	CHECK(effect(0).type == FFB_EFFECT_NONE);
	CHECK_FALSE(calc->isActive());
}

// ============================================================================================ new

TEST_CASE_FIXTURE(SerialFfbFixture, "new creates each effect type from constant to friction in the next free slot") {
	for (int type = FFB_EFFECT_CONSTANT; type <= FFB_EFFECT_FRICTION; type++) {
		CAPTURE(type);
		CHECK(create(type) == type - 1);
		CHECK(effect(type - 1).type == type);
	}
}

TEST_CASE_FIXTURE(SerialFfbFixture, "a new effect is infinite, enabled on the axis of the instance and stopped") {
	REQUIRE(create(FFB_EFFECT_SINE) == 0);
	const FFB_Effect &e = effect(0);
	CHECK(e.duration == FFB_EFFECT_DURATION_INFINITE);
	CHECK(e.axisMagnitudes[0] == 1.0f);
	CHECK(e.axisMagnitudes[1] == 0.0f);
	CHECK(e.state == 0);
	CHECK(e.magnitude == 0);
	CHECK(e.gain == 255);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "characterization: a new effect has full coefficients and saturations and one condition block per axis") {
	// NEEDS REVIEW 133: HidFFB::new_effect starts with zero conditions and a single condition block
	// (useSingleCondition true) until Set Condition reports arrive; SerialFFB.cpp:83-86 sets 0x7fff coefficients and
	// saturations on every axis and a condition block per axis, so a spring created with new=8 is at full strength.
	REQUIRE(create(FFB_EFFECT_SPRING) == 0);
	const FFB_Effect &e = effect(0);
	CHECK_FALSE(e.useSingleCondition);
	for (int axis = 0; axis < MAX_AXIS; axis++) {
		CAPTURE(axis);
		CHECK(e.conditions[axis].cpOffset == 0);
		CHECK(e.conditions[axis].positiveCoefficient == 0x7fff);
		CHECK(e.conditions[axis].negativeCoefficient == 0x7fff);
		CHECK(e.conditions[axis].positiveSaturation == 0x7fff);
		CHECK(e.conditions[axis].negativeSaturation == 0x7fff);
		CHECK(e.conditions[axis].deadBand == 0);
	}
}

TEST_CASE_FIXTURE(SerialFfbFixture, "new records the effect type as used in the effects calculator") {
	REQUIRE(CommandHelpers::get(*calc, "effects") == 0);
	REQUIRE(create(FFB_EFFECT_SINE) == 0);
	CHECK(CommandHelpers::get(*calc, "effects") == (1 << (FFB_EFFECT_SINE - 1)));
}

TEST_CASE_FIXTURE(SerialFfbFixture, "new answers -1 for the effect types none and above custom") {
	CHECK(create(FFB_EFFECT_NONE) == -1);
	CHECK(create(FFB_EFFECT_CUSTOM + 1) == -1);
	CHECK(create(255) == -1);
	CHECK(effect(0).type == FFB_EFFECT_NONE);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "characterization: new accepts the custom effect type 12 and truncates the type to 8 bits") {
	// NEEDS REVIEW 131: the info string lists 1..11, find_free_effect accepts 12 (custom, no force calculation);
	// the 64 bit value is cast to uint8_t (SerialFFB.cpp:147 -> newEffect(uint8_t)), so 257 creates a constant force.
	CHECK(create(FFB_EFFECT_CUSTOM) == 0);
	CHECK(effect(0).type == FFB_EFFECT_CUSTOM);
	CHECK(create(256 + FFB_EFFECT_CONSTANT) == 1);
	CHECK(effect(1).type == FFB_EFFECT_CONSTANT);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "new answers -1 when all effect slots are used") {
	for (uint32_t i = 0; i < EffectsCalculator::max_effects; i++) {
		REQUIRE(create(FFB_EFFECT_CONSTANT) == i);
	}
	CHECK(create(FFB_EFFECT_SINE) == -1);
	CHECK(effect(EffectsCalculator::max_effects - 1).type == FFB_EFFECT_CONSTANT);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "new lists the effect types in its info string") {
	std::vector<CommandReply> replies;
	CHECK(run("new", CMDtype::info, 0, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "Constant=1,Ramp=2,Square=3,Sine=4,Triangle=5,Sawtooth Up=6,Sawtooth Down=7,Spring=8,Damper=9,Inertia=10,Friction=11");
}

TEST_CASE_FIXTURE(SerialFfbFixture, "new rejects a get request") {
	CHECK(run("new", CMDtype::get) == CommandStatus::ERR);
	CHECK(effect(0).type == FFB_EFFECT_NONE);
}

TEST_CASE_FIXTURE(TargetFixture, "a new effect of the effects manager of instance 1 is enabled on the second axis only") {
	auto calc = std::make_shared<EffectsCalculator>();
	SerialFFB fxm1(calc, 1);
	REQUIRE(fxmNew(fxm1, FFB_EFFECT_CONSTANT) == 0);
	CHECK(calc->effects[0].axisMagnitudes[0] == 0.0f);
	CHECK(calc->effects[0].axisMagnitudes[1] == 1.0f);
}

TEST_CASE_FIXTURE(TargetFixture, "a new effect of an effects manager with an instance beyond the axes enables the last axis" *
				  KNOWN_ISSUE_UB("bug 126: SerialFFB.cpp:82 clamps the instance with std::min(instance, MAX_AXIS) instead of "
								 "MAX_AXIS - 1: instance 2 writes axisMagnitudes[2] one past the array (into conditions[0], "
								 "overwritten right after on the host). The other commands index conditions / "
								 "axisMagnitudes with the instance without any clamp")) {
	auto calc = std::make_shared<EffectsCalculator>();
	SerialFFB fxm2(calc, MAX_AXIS);
	REQUIRE(fxmNew(fxm2, FFB_EFFECT_CONSTANT) == 0);
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(calc->effects[0].axisMagnitudes[MAX_AXIS - 1] == 1.0f); // defect: written past the array instead
}

// ============================================================================================ type

TEST_CASE_FIXTURE(SerialFfbFixture, "type reads the effect type of an effect") {
	REQUIRE(create(FFB_EFFECT_DAMPER) == 0);
	CHECK(getAt("type", 0) == FFB_EFFECT_DAMPER);
	CHECK(getAt("type", 1) == FFB_EFFECT_NONE);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "type rejects an address beyond the effects and a request without address") {
	CHECK(run("type", CMDtype::getat, 0, EffectsCalculator::max_effects) == CommandStatus::ERR);
	CHECK(run("type", CMDtype::get) == CommandStatus::ERR);
}

// ============================================================================================ mag

TEST_CASE_FIXTURE(SerialFfbFixture, "mag sets and reads the magnitude of an effect") {
	REQUIRE(create(FFB_EFFECT_SINE) == 0);
	CHECK(setAt("mag", 0, -12345) == CommandStatus::OK);
	CHECK(effect(0).magnitude == -12345);
	CHECK(getAt("mag", 0) == -12345);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "mag of the last effect slot is set") {
	CHECK(setAt("mag", EffectsCalculator::max_effects - 1, 77) == CommandStatus::OK);
	CHECK(effect(EffectsCalculator::max_effects - 1).magnitude == 77);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "mag rejects reading an address beyond the effects") {
	CHECK(run("mag", CMDtype::getat, 0, EffectsCalculator::max_effects) == CommandStatus::ERR);
	CHECK(run("mag", CMDtype::getat, 0, -1) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "mag with an address from 41 to 255 changes no effect") {
	// The status (OK today) is part of bug 122 and asserted by its KNOWN_ISSUE case
	setAt("mag", 100, 5000);
	for (size_t i = 0; i < EffectsCalculator::max_effects; i++) {
		CHECK(effect(i).magnitude == 0);
	}
}

TEST_CASE_FIXTURE(SerialFfbFixture, "mag rejects a request without address") {
	CHECK(run("mag", CMDtype::get) == CommandStatus::ERR);
	CHECK(run("mag", CMDtype::set, 5) == CommandStatus::ERR);
	CHECK(effect(0).magnitude == 0);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "mag with an address beyond 8 bit does not change effect 0" *
				  KNOWN_ISSUE("bug 122: SerialFFB.cpp:156 passes the 64 bit address to setMagnitude(uint8_t) "
							  "(SerialFFB.cpp:95-97, check `idx > effects.size()`): 256 becomes 0 and sets effect 0, "
							  "40 passes the check and writes effects[40] past the array, and an invalid address is "
							  "answered OK although the get of mag answers ERR")) {
	REQUIRE(create(FFB_EFFECT_CONSTANT) == 0);
	REQUIRE(setAt("mag", 0, 1000) == CommandStatus::OK);

	CommandStatus status = setAt("mag", 256, 5000);

	CHECK(effect(0).magnitude == 1000); // defect: 5000
	CHECK(status == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "mag rejects the address equal to the number of effects" * doctest::skip() *
				  doctest::description("KNOWN ISSUE (crash): bug 122: setMagnitude checks `idx > effects.size()` "
									   "(SerialFFB.cpp:96), so address 40 writes effects[40] one element past the "
									   "std::array (aborts the run with _GLIBCXX_ASSERTIONS; memory corruption on the "
									   "target). Skipped")) {
	CHECK(setAt("mag", EffectsCalculator::max_effects, 5000) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "setting the magnitude of a spring sets its coefficients for compatibility" *
				  KNOWN_ISSUE("bug 127: SerialFFB.cpp:106-108 (\"Compatibility for conditional effects\") only evaluates "
							  "the coefficients of the instance's condition block without assigning them: mag has no "
							  "effect on conditional effects")) {
	REQUIRE(create(FFB_EFFECT_SPRING) == 0);

	REQUIRE(setAt("mag", 0, 1000) == CommandStatus::OK);

	CHECK(effect(0).conditions[0].positiveCoefficient == 1000); // defect: still 0x7fff
	CHECK(effect(0).conditions[0].negativeCoefficient == 1000);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "setting the magnitude of a constant force counts as a constant force update") {
	REQUIRE(create(FFB_EFFECT_CONSTANT) == 0);
	for (int i = 0; i < 8; i++) {
		HostPlatform::advanceMs(10);
		REQUIRE(setAt("mag", 0, i * 100) == CommandStatus::OK);
	}
	CHECK(fxm->getConstantForceRate() == 100);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "setting the magnitude of a periodic effect is not a constant force update") {
	REQUIRE(create(FFB_EFFECT_SINE) == 0);
	for (int i = 0; i < 8; i++) {
		HostPlatform::advanceMs(10);
		REQUIRE(setAt("mag", 0, i * 100) == CommandStatus::OK);
	}
	CHECK(fxm->getConstantForceRate() == 0);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "every effects manager command counts as an effect update for the update rate") {
	REQUIRE(create(FFB_EFFECT_SINE) == 0);
	for (int i = 0; i < 30; i++) {
		HostPlatform::advanceMs(5);
		REQUIRE(getAt("type", 0) == FFB_EFFECT_SINE);
	}
	CHECK(fxm->getRate() == 200);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "characterization: mag keeps the low 16 bit of a value beyond int16") {
	// NEEDS REVIEW 131: values are not range checked; 40000 is stored as int16 -25536 (reverses the force)
	REQUIRE(create(FFB_EFFECT_CONSTANT) == 0);
	REQUIRE(setAt("mag", 0, 40000) == CommandStatus::OK);
	CHECK(effect(0).magnitude == static_cast<int16_t>(40000 - 65536));
}

// ============================================================================================ state

TEST_CASE_FIXTURE(SerialFfbFixture, "state 1 starts an effect at the current time") {
	REQUIRE(create(FFB_EFFECT_SINE) == 0);
	HostPlatform::advanceMs(1234);

	CHECK(setAt("state", 0, 1) == CommandStatus::OK);

	CHECK(effect(0).state == 1);
	CHECK(effect(0).startTime == HAL_GetTick());
	CHECK(getAt("state", 0) == 1);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "state 0 stops an effect") {
	REQUIRE(create(FFB_EFFECT_SINE) == 0);
	REQUIRE(setAt("state", 0, 1) == CommandStatus::OK);

	CHECK(setAt("state", 0, 0) == CommandStatus::OK);

	CHECK(effect(0).state == 0);
	CHECK(getAt("state", 0) == 0);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "stopping an effect keeps its start time") {
	REQUIRE(create(FFB_EFFECT_SINE) == 0);
	HostPlatform::advanceMs(50);
	REQUIRE(setAt("state", 0, 1) == CommandStatus::OK);
	HostPlatform::advanceMs(50);

	REQUIRE(setAt("state", 0, 0) == CommandStatus::OK);

	CHECK(effect(0).startTime == 50);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "starting an effect adds its start delay to the start time") {
	REQUIRE(create(FFB_EFFECT_SINE) == 0);
	effect(0).startDelay = 300; // no fxm command sets it (NEEDS REVIEW 134); an effect slot reused after HID PID could have one
	HostPlatform::advanceMs(1000);
	REQUIRE(setAt("state", 0, 1) == CommandStatus::OK);
	CHECK(effect(0).startTime == 1300);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "characterization: state starts an unallocated effect slot") {
	// NEEDS REVIEW 132: the effect commands do not check that the slot holds an effect (type none); state=1 on a free
	// slot marks it running and mag/period/... write into it. calculateEffects skips type none (no force).
	CHECK(setAt("state", 5, 1) == CommandStatus::OK);
	CHECK(effect(5).type == FFB_EFFECT_NONE);
	CHECK(effect(5).state == 1);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "state rejects an address beyond the effects") {
	CHECK(setAt("state", EffectsCalculator::max_effects, 1) == CommandStatus::ERR);
	CHECK(run("state", CMDtype::getat, 0, EffectsCalculator::max_effects) == CommandStatus::ERR);
	CHECK(run("state", CMDtype::getat, 0, -1) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "state rejects a request without address") {
	REQUIRE(create(FFB_EFFECT_SINE) == 0);
	CHECK(run("state", CMDtype::set, 1) == CommandStatus::ERR);
	CHECK(run("state", CMDtype::get) == CommandStatus::ERR);
	CHECK(effect(0).state == 0);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "setEffectState ignores an effect index beyond the effects") {
	fxm->setEffectState(EffectsCalculator::max_effects, true);
	for (size_t i = 0; i < EffectsCalculator::max_effects; i++) {
		CHECK(effect(i).state == 0);
	}
}

// ============================================================================================ period, duration

TEST_CASE_FIXTURE(SerialFfbFixture, "period sets and reads the period of an effect") {
	REQUIRE(create(FFB_EFFECT_SINE) == 0);
	CHECK(setAt("period", 0, 250) == CommandStatus::OK);
	CHECK(effect(0).period == 250);
	CHECK(getAt("period", 0) == 250);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "period rejects an address beyond the effects") {
	CHECK(setAt("period", EffectsCalculator::max_effects, 100) == CommandStatus::ERR);
	CHECK(run("period", CMDtype::getat, 0, EffectsCalculator::max_effects) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "setting the period 0 stores the shortest period 1 like the HID PID path" *
				  KNOWN_ISSUE("bug 125: SerialFFB.cpp:180 stores period 0 unchecked; HidFFB::set_periodic clips it to "
							  "1..0x7fff (HidFFB.cpp:468, \"Period is never 0\"). Triangle and sawtooth effects then "
							  "compute `% (period*1000)` = `% 0` (EffectsCalculator.cpp:276/298/321): SIGFPE on the host, "
							  "a meaningless force on the target")) {
	REQUIRE(create(FFB_EFFECT_TRIANGLE) == 0);
	REQUIRE(setAt("period", 0, 100) == CommandStatus::OK);

	setAt("period", 0, 0);

	CHECK(effect(0).period == 1); // defect: 0
}

TEST_CASE_FIXTURE(SerialFfbFixture, "duration sets and reads the duration of an effect in ms") {
	REQUIRE(create(FFB_EFFECT_CONSTANT) == 0);
	CHECK(setAt("duration", 0, 1500) == CommandStatus::OK);
	CHECK(effect(0).duration == 1500);
	CHECK(getAt("duration", 0) == 1500);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "duration rejects an address beyond the effects") {
	CHECK(setAt("duration", EffectsCalculator::max_effects, 100) == CommandStatus::ERR);
	CHECK(run("duration", CMDtype::getat, 0, EffectsCalculator::max_effects) == CommandStatus::ERR);
}

// ============================================================================================ condition parameters

TEST_CASE_FIXTURE(SerialFfbFixture, "offset sets the periodic offset and the condition center of the instance's axis") {
	// Both on purpose: "Set both condition offset and periodic offset" (SerialFFB.cpp:193)
	REQUIRE(create(FFB_EFFECT_SPRING) == 0);
	const int16_t otherAxis = effect(0).conditions[1].cpOffset;
	CHECK(setAt("offset", 0, -2000) == CommandStatus::OK);
	CHECK(effect(0).offset == -2000);
	CHECK(effect(0).conditions[0].cpOffset == -2000);
	CHECK(effect(0).conditions[1].cpOffset == otherAxis);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "deadzone sets and reads the dead band of the instance's axis") {
	REQUIRE(create(FFB_EFFECT_SPRING) == 0);
	CHECK(setAt("deadzone", 0, 500) == CommandStatus::OK);
	CHECK(effect(0).conditions[0].deadBand == 500);
	CHECK(effect(0).conditions[1].deadBand == 0);
	CHECK(getAt("deadzone", 0) == 500);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "sat sets the positive saturation of the instance's axis only") {
	REQUIRE(create(FFB_EFFECT_SPRING) == 0);
	const FFB_Effect_Condition otherAxis = effect(0).conditions[1];
	CHECK(setAt("sat", 0, 10000) == CommandStatus::OK);
	CHECK(effect(0).conditions[0].positiveSaturation == 10000);
	CHECK(effect(0).conditions[1].positiveSaturation == otherAxis.positiveSaturation);
	CHECK(effect(0).conditions[1].negativeSaturation == otherAxis.negativeSaturation);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "coeff sets the positive coefficient of the instance's axis only") {
	REQUIRE(create(FFB_EFFECT_SPRING) == 0);
	const FFB_Effect_Condition otherAxis = effect(0).conditions[1];
	CHECK(setAt("coeff", 0, -3000) == CommandStatus::OK);
	CHECK(effect(0).conditions[0].positiveCoefficient == -3000);
	CHECK(effect(0).conditions[1].positiveCoefficient == otherAxis.positiveCoefficient);
	CHECK(effect(0).conditions[1].negativeCoefficient == otherAxis.negativeCoefficient);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "characterization: reading offset, sat and coeff answers two values") {
	// NEEDS REVIEW 129: one get calls handleGetSet twice (SerialFFB.cpp:194-195, 221-222, 235-236): offset answers the
	// condition center and the periodic offset, sat the negative and positive saturation, coeff the negative and
	// positive coefficient: two reply lines for one read (the string interface separates them with "\n").
	REQUIRE(create(FFB_EFFECT_SPRING) == 0);
	effect(0).conditions[0].cpOffset = 11;
	effect(0).offset = 12;
	effect(0).conditions[0].negativeSaturation = 21;
	effect(0).conditions[0].positiveSaturation = 22;
	effect(0).conditions[0].negativeCoefficient = 31;
	effect(0).conditions[0].positiveCoefficient = 32;

	const struct {
		const char *name;
		int64_t first, second;
	} expected[] = {{"offset", 11, 12}, {"sat", 21, 22}, {"coeff", 31, 32}};
	for (const auto &e : expected) {
		CAPTURE(e.name);
		std::vector<CommandReply> replies;
		CHECK(run(e.name, CMDtype::getat, 0, 0, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 2);
		CHECK(replies[0].val == e.first);
		CHECK(replies[1].val == e.second);
	}
}

TEST_CASE_FIXTURE(SerialFfbFixture, "characterization: coeff and sat also set the negative side, so a condition can not be asymmetric") {
	// NEEDS REVIEW 130: HID PID Set Condition has separate positive / negative coefficients and saturations; fxm sets
	// both sides with one value (SerialFFB.cpp:221-222, 235-236).
	REQUIRE(create(FFB_EFFECT_SPRING) == 0);
	REQUIRE(setAt("coeff", 0, 1000) == CommandStatus::OK);
	REQUIRE(setAt("sat", 0, 9000) == CommandStatus::OK);
	CHECK(effect(0).conditions[0].negativeCoefficient == 1000);
	CHECK(effect(0).conditions[0].negativeSaturation == 9000);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "characterization: coeff and sat keep the low 16 bit of a value out of range") {
	// NEEDS REVIEW 131: no range check, static_cast in handleGetSet: coeff 40000 -> int16 -25536 (reverses the
	// spring), sat -1 -> uint16 65535
	REQUIRE(create(FFB_EFFECT_SPRING) == 0);
	REQUIRE(setAt("coeff", 0, 40000) == CommandStatus::OK);
	REQUIRE(setAt("sat", 0, -1) == CommandStatus::OK);
	CHECK(effect(0).conditions[0].positiveCoefficient == static_cast<int16_t>(40000 - 65536));
	CHECK(effect(0).conditions[0].positiveSaturation == 65535);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "the condition commands reject an address beyond the effects") {
	for (const char *name : {"offset", "deadzone", "sat", "coeff"}) {
		CAPTURE(name);
		CHECK(setAt(name, EffectsCalculator::max_effects, 1) == CommandStatus::ERR);
		CHECK(run(name, CMDtype::getat, 0, EffectsCalculator::max_effects) == CommandStatus::ERR);
	}
}

TEST_CASE_FIXTURE(SerialFfbFixture, "the parameter commands reject an info request") {
	REQUIRE(create(FFB_EFFECT_SPRING) == 0);
	for (const char *name : {"period", "duration", "offset", "deadzone", "sat", "coeff"}) {
		CAPTURE(name);
		std::vector<CommandReply> replies;
		CHECK(run(name, CMDtype::info, 0, 0, &replies) == CommandStatus::ERR);
	}
}

TEST_CASE_FIXTURE(TargetFixture, "the effects manager of instance 1 sets the condition block of the second axis") {
	auto calc = std::make_shared<EffectsCalculator>();
	SerialFFB fxm1(calc, 1);
	REQUIRE(fxmNew(fxm1, FFB_EFFECT_SPRING) == 0);
	const FFB_Effect_Condition firstAxis = calc->effects[0].conditions[0];

	REQUIRE(fxmCmd(fxm1, "coeff", CMDtype::setat, 1000, 0) == CommandStatus::OK);
	REQUIRE(fxmCmd(fxm1, "deadzone", CMDtype::setat, 20, 0) == CommandStatus::OK);

	CHECK(calc->effects[0].conditions[1].positiveCoefficient == 1000);
	CHECK(calc->effects[0].conditions[1].deadBand == 20);
	CHECK(calc->effects[0].conditions[0].positiveCoefficient == firstAxis.positiveCoefficient);
	CHECK(calc->effects[0].conditions[0].deadBand == firstAxis.deadBand);
}

// ============================================================================================ axisgain

TEST_CASE_FIXTURE(SerialFfbFixture, "axisgain reads the axis magnitude of the instance as 16 bit") {
	REQUIRE(create(FFB_EFFECT_CONSTANT) == 0);
	CHECK(getAt("axisgain", 0) == 0xffff);
	effect(0).axisMagnitudes[0] = 0.0f;
	CHECK(getAt("axisgain", 0) == 0);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "axisgain sets the axis magnitude of the instance from 16 bit") {
	REQUIRE(create(FFB_EFFECT_CONSTANT) == 0);
	CHECK(setAt("axisgain", 0, 0x8000) == CommandStatus::OK);
	CHECK(effect(0).axisMagnitudes[0] == doctest::Approx(0x8000 / 65535.0).epsilon(1e-6));
	CHECK(effect(0).axisMagnitudes[1] == 0.0f);
	CHECK(setAt("axisgain", 0, 0) == CommandStatus::OK);
	CHECK(effect(0).axisMagnitudes[0] == 0.0f);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "characterization: axisgain accepts values beyond 16 bit and negative values") {
	// NEEDS REVIEW 131: "Gain for this axis (instance) 16b" is not range checked: 0x1fffe gives magnitude 2.0 (double
	// force), -0xffff gives -1.0 (reversed force)
	REQUIRE(create(FFB_EFFECT_CONSTANT) == 0);
	REQUIRE(setAt("axisgain", 0, 0x1fffe) == CommandStatus::OK);
	CHECK(effect(0).axisMagnitudes[0] == doctest::Approx(2.0));
	REQUIRE(setAt("axisgain", 0, -0xffff) == CommandStatus::OK);
	CHECK(effect(0).axisMagnitudes[0] == doctest::Approx(-1.0));
}

TEST_CASE_FIXTURE(SerialFfbFixture, "axisgain rejects an address beyond the effects") {
	CHECK(setAt("axisgain", EffectsCalculator::max_effects, 1) == CommandStatus::ERR);
	CHECK(run("axisgain", CMDtype::getat, 0, EffectsCalculator::max_effects) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(SerialFfbFixture, "axisgain without address neither answers nor changes the effect") {
	// Not reachable over a command interface (the command table only allows getat / setat)
	REQUIRE(create(FFB_EFFECT_CONSTANT) == 0);
	std::vector<CommandReply> replies;
	run("axisgain", CMDtype::set, 0, 0, &replies);
	run("axisgain", CMDtype::get, 0, 0, &replies);
	CHECK(replies.empty());
	CHECK(effect(0).axisMagnitudes[0] == 1.0f);
}

// ============================================================================================ sample rate, filters

TEST_CASE_FIXTURE(SerialFfbFixture, "updateSamplerate recalculates the effect filters of the calculator") {
	REQUIRE(create(FFB_EFFECT_DAMPER) == 0);
	calc->setFilters(&effect(0)); // like HidFFB does; SerialFFB itself does not (bug 124)
	REQUIRE(effect(0).filter[0] != nullptr);
	const float fcAt1k = effect(0).filter[0]->getFc();

	fxm->updateSamplerate(500);

	CHECK(effect(0).filter[0]->getFc() == doctest::Approx(fcAt1k * 2.0f));
}

TEST_CASE_FIXTURE(SerialFfbFixture, "a damper created and started over fxm has its effect filters like the HID PID path" *
				  KNOWN_ISSUE("bug 124: SerialFFB never calls EffectsCalculator::setFilters (HidFFB does on new effect "
							  "and start, HidFFB.cpp:276/419): damper and inertia dereference the null filter in every "
							  "cycle (EffectsCalculator.cpp:401/409), friction as soon as the axis moves (:389): crash; "
							  "constant forces are not filtered")) {
	REQUIRE(create(FFB_EFFECT_DAMPER) == 0);
	REQUIRE(setAt("state", 0, 1) == CommandStatus::OK);

	CHECK(effect(0).filter[0] != nullptr); // defect: nullptr
	CHECK(effect(0).filter[1] != nullptr);
}

// ============================================================================================ forces

TEST_CASE_FIXTURE(SerialForceFixture, "a constant force over fxm gives the same torque as the same constant force over HID PID") {
	uint8_t hidIdx = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(hidIdx, 10000);
	run(300);
	const int32_t hidTorque = axis().getTorque();
	REQUIRE(std::abs(hidTorque) > 1000);
	clearHidEffects();
	run(300);
	REQUIRE(axis().getTorque() == 0);

	int64_t idx = create(FFB_EFFECT_CONSTANT);
	REQUIRE(setAt("mag", idx, 10000) == CommandStatus::OK);
	start(idx);
	run(300);

	CHECK(axis().getTorque() == doctest::Approx(hidTorque).epsilon(0.002));
}

TEST_CASE_FIXTURE(SerialForceFixture, "an fxm effect gives no torque while FFB is off") {
	int64_t idx = create(FFB_EFFECT_CONSTANT);
	REQUIRE(setAt("mag", idx, 10000) == CommandStatus::OK);
	REQUIRE(setAt("state", idx, 1) == CommandStatus::OK);
	run(100);
	CHECK(axis().getTorque() == 0);
}

TEST_CASE_FIXTURE(SerialForceFixture, "a stopped fxm effect gives no torque") {
	int64_t idx = create(FFB_EFFECT_CONSTANT);
	REQUIRE(setAt("mag", idx, 10000) == CommandStatus::OK);
	start(idx);
	run(100);
	REQUIRE(axis().getTorque() != 0);

	REQUIRE(setAt("state", idx, 0) == CommandStatus::OK);
	run(100);

	CHECK(axis().getTorque() == 0);
}

TEST_CASE_FIXTURE(SerialForceFixture, "axisgain of an fxm effect scales its torque") {
	int64_t idx = create(FFB_EFFECT_CONSTANT);
	REQUIRE(setAt("mag", idx, 10000) == CommandStatus::OK);
	start(idx);
	run(100);
	const int32_t full = axis().getTorque();
	REQUIRE(std::abs(full) > 1000);

	REQUIRE(setAt("axisgain", idx, 0x8000) == CommandStatus::OK);
	run(100);

	CHECK(axis().getTorque() == doctest::Approx(full / 2.0).epsilon(0.002));
}

TEST_CASE_FIXTURE(SerialForceFixture, "a spring over fxm gives the same torque as the same spring over HID PID") {
	encoder().setDegrees(-20.0f);
	run(10);
	uint8_t hidIdx = host.allocateEffect(FFB_EFFECT_SPRING);
	host.setCondition(hidIdx, 0, 0, 4000, 4000, 0x7fff, 0x7fff, 0);
	host.setEffect(hidIdx, FFB_EFFECT_SPRING);
	host.startEffect(hidIdx);
	run(50);
	const int32_t hidTorque = axis().getTorque();
	REQUIRE(std::abs(hidTorque) > 100);
	clearHidEffects();
	run(50);
	REQUIRE(axis().getTorque() == 0);

	int64_t idx = create(FFB_EFFECT_SPRING);
	REQUIRE(setAt("coeff", idx, 4000) == CommandStatus::OK);
	REQUIRE(setAt("sat", idx, 0x7fff) == CommandStatus::OK);
	start(idx);
	run(50);

	CHECK(axis().getTorque() == doctest::Approx(hidTorque).epsilon(0.002));
}

TEST_CASE_FIXTURE(SerialForceFixture, "a sine over fxm follows the same waveform as the same sine over HID PID") {
	const int cycles = 120;
	uint8_t hidIdx = host.allocateEffect(FFB_EFFECT_SINE);
	host.setEffect(hidIdx, FFB_EFFECT_SINE);
	host.setPeriodic(hidIdx, 8000, 0, 0, 100);
	host.startEffect(hidIdx);
	std::vector<int32_t> hid;
	for (int i = 0; i < cycles; i++) {
		run(1);
		hid.push_back(axis().getTorque());
	}
	clearHidEffects();
	run(50);

	int64_t idx = create(FFB_EFFECT_SINE);
	REQUIRE(setAt("mag", idx, 8000) == CommandStatus::OK);
	REQUIRE(setAt("period", idx, 100) == CommandStatus::OK);
	start(idx);
	int32_t peak = 0;
	for (int i = 0; i < cycles; i++) {
		run(1);
		CAPTURE(i);
		CHECK(axis().getTorque() == doctest::Approx(hid[i]).epsilon(0.002).scale(50));
		peak = std::max(peak, std::abs(axis().getTorque()));
	}
	CHECK(peak > 4000);
}

TEST_CASE_FIXTURE(SerialForceFixture, "an fxm effect with a duration stops after the duration") {
	int64_t idx = create(FFB_EFFECT_CONSTANT);
	REQUIRE(setAt("mag", idx, 10000) == CommandStatus::OK);
	REQUIRE(setAt("duration", idx, 100) == CommandStatus::OK);
	start(idx);
	run(100);
	REQUIRE(fxmGetAt(fxm, "state", idx) == 1);
	REQUIRE(axis().getTorque() != 0);

	run(5);

	CHECK(fxmGetAt(fxm, "state", idx) == 0);
	CHECK(axis().getTorque() == 0);
}

TEST_CASE_FIXTURE(SerialForceFixture, "a damper created over fxm damps the motion of the axis" * doctest::skip() *
				  doctest::description("KNOWN ISSUE (crash): bug 124: the damper has no filter (SerialFFB never calls "
									   "setFilters), EffectsCalculator.cpp:401 calls process() on the null filter in the "
									   "first cycle with FFB on (SIGSEGV on the host, HardFault on the target). Skipped")) {
	int64_t idx = create(FFB_EFFECT_DAMPER);
	REQUIRE(setAt("coeff", idx, 10000) == CommandStatus::OK);
	start(idx);
	encoder().setCpr(3600000);
	for (int n = 1; n <= 50; n++) {
		encoder().setRawPosition(n * 1000); // 0.1 deg per ms = 100 deg/s
		run(1);
	}
	CHECK(axis().getTorque() != 0);
}

TEST_CASE_FIXTURE(SerialForceFixture2Axes, "a constant force of the effects manager of instance 1 acts on the second axis only") {
	SerialFFB fxm1(effects, 1);
	int64_t idx = fxmNew(fxm1, FFB_EFFECT_CONSTANT);
	REQUIRE(fxmCmd(fxm1, "mag", CMDtype::setat, 10000, idx) == CommandStatus::OK);
	REQUIRE(fxmCmd(fxm1, "state", CMDtype::setat, 1, idx) == CommandStatus::OK);
	REQUIRE(fxmCmd(fxm1, "ffbstate", CMDtype::set, 1) == CommandStatus::OK);

	run(100);

	CHECK(axis(0).getTorque() == 0);
	CHECK(std::abs(axis(1).getTorque()) > 1000);
}

// ============================================================================================ CDC end to end

TEST_CASE_FIXTURE(TargetFixture, "fxm command lines on the CDC port create, configure and read back an effect") {
	FFBoardMain main;
	auto calc = std::make_shared<EffectsCalculator>();
	SerialFFB fxm(calc);
	CommandHelpers::CdcConsole console(main);

	CHECK(console.send("fxm.0.new=4\n") == "[fxm.0.new=4|0]\n");
	CHECK(console.send("fxm.0.mag=3000?0\n") == "[fxm.0.mag=3000?0|OK]\n");
	CHECK(console.send("fxm.0.period=200?0\n") == "[fxm.0.period=200?0|OK]\n");
	CHECK(console.send("fxm.0.state=1?0\n") == "[fxm.0.state=1?0|OK]\n");
	CHECK(console.send("fxm.0.mag?0\n") == "[fxm.0.mag?0|3000]\n");
	CHECK(console.send("fxm.0.type?0\n") == "[fxm.0.type?0|4]\n");
	CHECK(calc->effects[0].period == 200);
	CHECK(calc->effects[0].state == 1);
}

TEST_CASE_FIXTURE(TargetFixture, "the command table of fxm drops a magnitude without effect address on the CDC port without answer") {
	FFBoardMain main;
	auto calc = std::make_shared<EffectsCalculator>();
	SerialFFB fxm(calc);
	CommandHelpers::CdcConsole console(main);
	REQUIRE(console.send("fxm.0.new=1\n") == "[fxm.0.new=1|0]\n");

	std::string reply = console.send("fxm.0.mag=5\n");

	// FFBoardMainCommandThread::executeCommands skips commands whose type is not in the command flags (no reply)
	CHECK(reply.empty());
	CHECK(calc->effects[0].magnitude == 0);
}

TEST_CASE_FIXTURE(TargetFixture, "characterization: reading the coefficients of an effect on the CDC port answers two lines") {
	// NEEDS REVIEW 129
	FFBoardMain main;
	auto calc = std::make_shared<EffectsCalculator>();
	SerialFFB fxm(calc);
	CommandHelpers::CdcConsole console(main);
	REQUIRE(console.send("fxm.0.new=8\n") == "[fxm.0.new=8|0]\n");
	calc->effects[0].conditions[0].negativeCoefficient = 31;
	calc->effects[0].conditions[0].positiveCoefficient = 32;

	CHECK(console.send("fxm.0.coeff?0\n") == "[fxm.0.coeff?0|31\n32]\n");
}

TEST_CASE_FIXTURE(TargetFixture, "the effect types of fxm are listed by the info request on the CDC port") {
	FFBoardMain main;
	auto calc = std::make_shared<EffectsCalculator>();
	SerialFFB fxm(calc);
	CommandHelpers::CdcConsole console(main);

	std::string reply = console.send("fxm.0.new!\n");

	CHECK(reply.find("Constant=1,Ramp=2") != std::string::npos);
	CHECK(reply.find("Friction=11") != std::string::npos);
}

TEST_CASE_FIXTURE(TargetFixture, "the external FFB gamepad starts an fxm effect sent on the CDC port" * DELETES_FFBHIDMAIN) {
	FFBHIDExt ext;
	CommandHelpers::CdcConsole console(ext);

	CHECK(console.send("fxm.0.new=1\n") == "[fxm.0.new=1|0]\n");
	CHECK(console.send("fxm.0.mag=5000?0\n") == "[fxm.0.mag=5000?0|OK]\n");
	CHECK(console.send("fxm.0.state=1?0\n") == "[fxm.0.state=1?0|OK]\n");
	CHECK(console.send("fxm.0.ffbstate=1\n") == "[fxm.0.ffbstate=1|OK]\n");

	CHECK(console.send("main.0.ffbactive?\n") == "[main.0.ffbactive?|1]\n");
	CHECK(console.send("fxm.0.state?0\n") == "[fxm.0.state?0|1]\n");
}

TEST_CASE_FIXTURE(TargetFixture, "both effects managers of the external FFB gamepad share one effect list" * DELETES_FFBHIDMAIN) {
	FFBHIDExt ext;
	CommandHelpers::CdcConsole console(ext);

	REQUIRE(console.send("fxm.0.new=8\n") == "[fxm.0.new=8|0]\n");

	CHECK(console.send("fxm.1.type?0\n") == "[fxm.1.type?0|8]\n");
	CHECK(console.send("fxm.1.new=1\n") == "[fxm.1.new=1|1]\n");
	CHECK(console.send("fxm.0.axisgain?1\n") == "[fxm.0.axisgain?1|0]\n");
	CHECK(console.send("fxm.1.axisgain?1\n") == "[fxm.1.axisgain?1|65535]\n");
}

} // TEST_SUITE
