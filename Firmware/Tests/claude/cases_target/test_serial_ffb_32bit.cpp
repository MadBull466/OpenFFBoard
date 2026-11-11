/*
 * P18 T2: tests of SerialFFB (FFBoard/Src/SerialFFB.cpp) for the effect address guard `cmd.adr < effects.size()`,
 * candidate firmware bug 225 (Tests/claude/plans/P18_32BIT_BUILD_PLAN.md, section 2). Suite SerialFFB32.
 *
 * What is tested: the ten "fxm" commands that index the effect array with the address of the command (mag, state,
 * period, duration, offset, deadzone, sat, coeff, axisgain, type) reject addresses that are not an effect index
 * (negative, 2^32, 2^32 + 1, max_effects) with ERR, produce no reply and change no field of any effect slot; the last
 * slot (max_effects - 1) is accepted.
 *
 * Oracle: the intended behaviour is the documented one. SerialFFB.cpp registers these commands as "... effect adr", so
 * an address that is not an effect is an error (the guard exists for that), and the rejection that the 64-bit host
 * shows is the behaviour the existing cases of test_serial_ffb.cpp assert. Hand derivations of the conversions (HD):
 *   - ParsedCommand::adr is int64_t. effects.size() is size_t.
 *   - LP64 host: size_t is unsigned long (64 bit), the same rank as int64_t (long). The usual arithmetic
 *     conversions make the comparison unsigned, so -1 becomes 0xFFFFFFFFFFFFFFFF and is not < 40.
 *   - ILP32 (target, FFB_TESTS_32BIT): size_t is unsigned int (32 bit). It is converted to int64_t, the comparison is
 *     signed: -1 < 40 is true and the guard passes. Addresses >= 2^32 stay rejected on both widths: 4294967296 < 40
 *     is false in int64_t (ILP32) and in uint64_t (LP64).
 *
 * Width notes: negative addresses are the only cases whose result depends on the width. The cases for 2^32, 2^32 + 1,
 * max_effects and max_effects - 1 are plain tests on both widths. Reading or writing effects[-1] and so on is
 * undefined behaviour: libstdc++ assertions (_GLIBCXX_ASSERTIONS) abort the whole run on ILP32, so those cases are
 * skipped there. Not modelled: other differences between i386 and ARM (see the plan).
 *
 * Markers (support/KnownIssue32.h):
 *   KNOWN_ISSUE_32BIT_UB  every case that indexes effects[adr] with a negative address (reads and writes, and the
 *                         CDC end to end case): a plain test on LP64, skipped on ILP32.
 *   KNOWN_ISSUE_32BIT     state setat with a negative address: the guard passes but setEffectState(uint8_t) truncates
 *                         the address and returns without access, so the command answers OK instead of ERR (no out of
 *                         bounds access): a plain test on LP64, should_fail on ILP32.
 *   The other cases are plain tests on both widths.
 * Not tested for bug 225: mag setat (setMagnitude takes uint8_t) and reset getat (free_effect takes uint16_t), both
 * truncate before comparing (bug 122 territory).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "CommandHelpers.h"
#include "HostPlatform.h"
#include "KnownIssue32.h"
#include "TestHelpers.h"

#include "EffectsCalculator.h"
#include "SerialFFB.h"

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

/** Bug 225 (several cases) */
#define BUG225_TEXT                                                                                                    \
	"bug 225: ParsedCommand::adr is an int64_t (CommandHandler.h:70) and SerialFFB::command guards the effect index "   \
	"with `cmd.adr < effects.size()` (SerialFFB.cpp:157, 164, 179, 186, 193, 201, 208, 220, 227, 234). On LP64 "       \
	"size_t is unsigned long, the comparison is unsigned and every negative address is rejected. On ILP32 (the "        \
	"target) size_t is unsigned int, converted to int64_t, the comparison is signed and any negative address passes: "  \
	"effects[adr] is accessed out of bounds. Reads: mag getat (:158), state getat (:168), type getat (:228), and "     \
	"the get of period, duration, offset, deadzone, sat, coeff, axisgain. Writes: the set of period, duration, offset, " \
	"deadzone, sat, coeff (handleGetSet, :180-236) and axisgain setat (:212), at an address before the array. state "  \
	"setat is not out of bounds but answers OK (uint8_t truncation, :114-122). Reachable: serial fxm.0.<cmd>?-1 "     \
	"(parser: adr -1), HID command interface with addr all ones (HidCommandInterface.cpp:173 copies uint64_t to "       \
	"int64_t), CAN command interface (CanCommandInterface.cpp:169). Shipped firmware with FFBHIDEXT (F407VG, "          \
	"F407VG_DISCO, F411RE: target_constants.h) creates SerialFFB"

namespace {

const int64_t kMaxEffects = EffectsCalculator::max_effects; // 40, a copy so that no reference to the constant is taken

/** All fields of one effect slot that the ten commands can touch, plus the other plain data fields (not the filters) */
using SlotData = std::vector<int64_t>;

SlotData snapshotSlot(const FFB_Effect &e) {
	SlotData d;
	d.push_back(e.state);
	d.push_back(e.type);
	d.push_back(e.offset);
	d.push_back(e.gain);
	d.push_back(e.magnitude);
	d.push_back(e.startLevel);
	d.push_back(e.endLevel);
	for (float f : e.axisMagnitudes) {
		d.push_back(static_cast<int64_t>(f * 65535.0f)); // 0..1 scaled like the axisgain command, exact for the values used here
	}
	for (const FFB_Effect_Condition &c : e.conditions) {
		d.push_back(c.cpOffset);
		d.push_back(c.positiveCoefficient);
		d.push_back(c.negativeCoefficient);
		d.push_back(c.positiveSaturation);
		d.push_back(c.negativeSaturation);
		d.push_back(c.deadBand);
	}
	d.push_back(e.phase);
	d.push_back(e.period);
	d.push_back(e.duration);
	d.push_back(e.attackLevel);
	d.push_back(e.fadeLevel);
	d.push_back(e.attackTime);
	d.push_back(e.fadeTime);
	d.push_back(e.startDelay);
	d.push_back(e.startTime);
	d.push_back(e.samplePeriod);
	return d;
}

/** A SerialFFB (instance 0) on its own EffectsCalculator; slot 0 and slot 1 hold non-default values */
struct SerialFfb32Fixture : TargetFixture {
	std::shared_ptr<EffectsCalculator> calc = std::make_shared<EffectsCalculator>();
	std::unique_ptr<SerialFFB> fxm = std::make_unique<SerialFFB>(calc);

	SerialFfb32Fixture() {
		// Slot 0 is created with the "new" command (a sine, type 4, so that type differs from state 1), slot 1 by writing the fields directly, so that
		// every field a command touches has a non-default value that a stray write would change visibly.
		std::vector<CommandReply> replies;
		REQUIRE(CommandHelpers::run(*fxm, "new", CMDtype::set, FFB_EFFECT_SINE, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		REQUIRE(replies.front().val == 0);
		fill(calc->effects[0], 11);
		fill(calc->effects[1], 23);
	}

	static void fill(FFB_Effect &e, int16_t base) {
		e.state = 1;
		e.offset = base + 1;
		e.magnitude = base + 2;
		e.period = base + 3;
		e.duration = 1000u + base;
		// Every instance (condition and axis index) gets different values so that a command reading the wrong
		// instance is visible (the fixture's SerialFFB is instance 0)
		int16_t k = 0;
		for (auto &c : e.conditions) {
			c.cpOffset = base + 4 + k;
			c.positiveCoefficient = base + 5 + k;
			c.negativeCoefficient = base + 6 + k;
			c.positiveSaturation = base + 7 + k;
			c.negativeSaturation = base + 8 + k;
			c.deadBand = base + 9 + k;
			k += 50;
		}
		e.axisMagnitudes[0] = 0.5f;
#if MAX_AXIS >= 2
		e.axisMagnitudes[1] = 0.25f;
#endif
	}

	std::vector<SlotData> snapshot() const {
		std::vector<SlotData> all;
		for (const FFB_Effect &e : calc->effects) {
			all.push_back(snapshotSlot(e));
		}
		return all;
	}

	CommandStatus run(const std::string &name, CMDtype type, int64_t adr, int64_t val, std::vector<CommandReply> &replies) {
		return CommandHelpers::run(*fxm, name, type, val, &replies, adr);
	}

	/** Runs `name` with `type` at `adr` and checks: ERR, no reply, no field of any slot changed */
	void expectRejected(const std::string &name, CMDtype type, int64_t adr, int64_t val = 1234) {
		CAPTURE(name);
		CAPTURE(adr);
		std::vector<SlotData> before = snapshot();
		std::vector<CommandReply> replies;
		CommandStatus status = run(name, type, adr, val, replies);
		CHECK(status == CommandStatus::ERR);
		CHECK(replies.empty());
		CHECK(snapshot() == before);
	}
};

/** The negative addresses: -1; -40 (slot 0 minus 40 slots); -41; the smallest int64_t */
const int64_t kNegativeAddresses[] = {-1, -40, -41, std::numeric_limits<int64_t>::min()};

/** The ten commands that read with getat and the reply of the unused last slot (defaults of FFB_Effect, ffb_defs.h) */
struct GetAtCommand {
	const char *name;
	int64_t defaultValue; // first reply of a slot that holds no effect
};
const GetAtCommand kGetAtCommands[] = {
	{"mag", 0}, {"state", 0}, {"period", 0}, {"duration", FFB_EFFECT_DURATION_INFINITE}, {"offset", 0},
	{"deadzone", 0}, {"sat", 0}, {"coeff", 0}, {"axisgain", 0}, {"type", FFB_EFFECT_NONE},
};

} // namespace

// Defined as cases per command (one behaviour each): the name of the command is part of the case name.
#define NEG_GETAT_CASE(cmd)                                                                                            \
	TEST_CASE_FIXTURE(SerialFfb32Fixture,                                                                              \
					  "bug 225: fxm " cmd " getat with a negative address is rejected and changes no effect" *         \
						  KNOWN_ISSUE_32BIT_UB(BUG225_TEXT)) {                                                         \
		for (int64_t adr : kNegativeAddresses) {                                                                       \
			expectRejected(cmd, CMDtype::getat, adr);                                                                  \
		}                                                                                                              \
	}

#define NEG_SETAT_CASE(cmd)                                                                                            \
	TEST_CASE_FIXTURE(SerialFfb32Fixture,                                                                              \
					  "bug 225: fxm " cmd " setat with a negative address is rejected and changes no effect" *         \
						  KNOWN_ISSUE_32BIT_UB(BUG225_TEXT)) {                                                         \
		for (int64_t adr : kNegativeAddresses) {                                                                       \
			expectRejected(cmd, CMDtype::setat, adr, 1234);                                                            \
		}                                                                                                              \
	}

TEST_SUITE("SerialFFB32") {

// ============================================================================================ negative addresses, reads

NEG_GETAT_CASE("mag")
NEG_GETAT_CASE("state")
NEG_GETAT_CASE("period")
NEG_GETAT_CASE("duration")
NEG_GETAT_CASE("offset")
NEG_GETAT_CASE("deadzone")
NEG_GETAT_CASE("sat")
NEG_GETAT_CASE("coeff")
NEG_GETAT_CASE("axisgain")
NEG_GETAT_CASE("type")

// ============================================================================================ negative addresses, writes

NEG_SETAT_CASE("period")
NEG_SETAT_CASE("duration")
NEG_SETAT_CASE("deadzone")
NEG_SETAT_CASE("offset")
NEG_SETAT_CASE("sat")
NEG_SETAT_CASE("coeff")
NEG_SETAT_CASE("axisgain")

TEST_CASE_FIXTURE(SerialFfb32Fixture,
				  "bug 225: fxm state setat with a negative address is rejected and changes no effect" *
					  KNOWN_ISSUE_32BIT(BUG225_TEXT)) {
	// ILP32: the guard passes and setEffectState(uint8_t id) truncates the address (modulo 256):
	//   -1 -> 255, -40 -> 216, -41 -> 215 (all >= 40, return without access);
	//   -216 -> 40 (-216 + 256), exactly the boundary of the guard `id >= effects.size()` (40 >= 40 returns);
	//   INT64_MIN -> 0 (its low 8 bits are 0): slot 0 is a valid id, so on ILP32 its state is set to val 0 and the
	//   snapshot sees slot 0 change from 1 to 0 (a real change, not only a wrong status).
	// The first assertion of each address is the status: ERR intended, OK on ILP32.
	for (int64_t adr : {int64_t(-1), int64_t(-40), int64_t(-41), int64_t(-216), std::numeric_limits<int64_t>::min()}) {
		CAPTURE(adr);
		std::vector<SlotData> before = snapshot();
		std::vector<CommandReply> replies;
		CHECK(run("state", CMDtype::setat, adr, 0, replies) == CommandStatus::ERR); // the first assertion: the defect
		CHECK(replies.empty());
		CHECK(snapshot() == before); // state 1 of slots 0 and 1 stays 1
	}
}

// ============================================================================================ addresses beyond 32 bit (both widths)

TEST_CASE_FIXTURE(SerialFfb32Fixture, "bug 225: a getat of 2^32 and 2^32 + 1 is rejected by all ten commands and changes no effect") {
	// HD: ILP32: 40 is converted to int64_t and 4294967296 < 40 is false; LP64: compared as uint64_t, also false.
	for (const GetAtCommand &c : kGetAtCommands) {
		expectRejected(c.name, CMDtype::getat, INT64_C(4294967296));
		expectRejected(c.name, CMDtype::getat, INT64_C(4294967297));
	}
}

TEST_CASE_FIXTURE(SerialFfb32Fixture, "bug 225: a setat of 2^32 and 2^32 + 1 is rejected by the eight setting commands and changes no effect") {
	for (const char *name : {"period", "duration", "deadzone", "offset", "sat", "coeff", "axisgain", "state"}) {
		expectRejected(name, CMDtype::setat, INT64_C(4294967296));
		expectRejected(name, CMDtype::setat, INT64_C(4294967297));
	}
}

// ============================================================================================ the end of the effect array (both widths)

TEST_CASE_FIXTURE(SerialFfb32Fixture, "bug 225: a getat of the address max_effects is rejected by all ten commands and changes no effect") {
	// HD: effects.size() is 40 (MAX_EFFECTS, ffb_defs.h:21), the valid indices are 0..39 and 40 < 40 is false
	for (const GetAtCommand &c : kGetAtCommands) {
		expectRejected(c.name, CMDtype::getat, kMaxEffects);
	}
}

TEST_CASE_FIXTURE(SerialFfb32Fixture, "bug 225: a getat of the last effect max_effects - 1 is answered OK with the defaults of an unused slot") {
	// HD: slot 39 was never created, so it holds the defaults of FFB_Effect (ffb_defs.h:337-358): all fields 0,
	// duration FFB_EFFECT_DURATION_INFINITE, type FFB_EFFECT_NONE (0). Commands that read two fields (offset, sat,
	// coeff) answer two replies, the first one is checked.
	for (const GetAtCommand &c : kGetAtCommands) {
		CAPTURE(std::string(c.name));
		std::vector<SlotData> before = snapshot();
		std::vector<CommandReply> replies;
		CHECK(run(c.name, CMDtype::getat, kMaxEffects - 1, 0, replies) == CommandStatus::OK);
		REQUIRE(!replies.empty());
		CHECK(replies.front().val == c.defaultValue);
		CHECK(snapshot() == before);
	}
}

TEST_CASE_FIXTURE(SerialFfb32Fixture, "bug 225: a getat of the last effect max_effects - 1 answers one reply for mag, state, period, duration, axisgain and type") {
	for (const char *name : {"mag", "state", "period", "duration", "axisgain", "type"}) {
		CAPTURE(std::string(name));
		std::vector<CommandReply> replies;
		CHECK(run(name, CMDtype::getat, kMaxEffects - 1, 0, replies) == CommandStatus::OK);
		CHECK(replies.size() == 1);
	}
}

TEST_CASE_FIXTURE(SerialFfb32Fixture, "bug 225: a type getat of the last effect answers 0 (FFB_EFFECT_NONE) when no effect was created there") {
	std::vector<CommandReply> replies;
	CHECK(run("type", CMDtype::getat, kMaxEffects - 1, 0, replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == 0);
}

TEST_CASE_FIXTURE(SerialFfb32Fixture, "bug 225: a getat of effect 0 returns the value of that effect") {
	// Control: valid addresses still work (slot 0 was filled with base 11: period 14, magnitude 13)
	std::vector<CommandReply> replies;
	CHECK(run("period", CMDtype::getat, 0, 0, replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == 14);
	replies.clear();
	CHECK(run("mag", CMDtype::getat, 0, 0, replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == 13);
}

// ============================================================================================ positive controls (both widths)

TEST_CASE_FIXTURE(SerialFfb32Fixture, "bug 225: a type getat of effect 0 returns the type that new created") {
	std::vector<CommandReply> replies;
	CHECK(run("type", CMDtype::getat, 0, 0, replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == FFB_EFFECT_SINE); // new=4 in the fixture, FFB_EFFECT_SINE 0x04 (ffb_defs.h:82)
}

TEST_CASE_FIXTURE(SerialFfb32Fixture, "bug 225: a state getat of effect 0 returns the state 1 of that effect") {
	// The fixture sets state 1 on slot 0 and its type is 4, so a getat that returned the type would answer 4
	std::vector<CommandReply> replies;
	CHECK(run("state", CMDtype::getat, 0, 0, replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies.front().val == 1);
}

namespace {
using Fields = std::vector<int64_t>;
struct SetterControl {
	const char *name;
	int64_t val;
	// every field of slot `e` that the command writes (instance 0); all must equal val after the setat
	Fields (*written)(const FFB_Effect &e);
	// the fields of instance 1 that the command must not touch (empty if the command has no instance data)
	Fields (*otherInstance)(const FFB_Effect &e);
};
// Values differ from the fixture's fill values (base 23 for slot 1) and from the defaults.
// axisgain: val 65535: (float)65535 / 0xffff = 1.0f exactly, 1.0f * 0xffff = 65535.0f exactly, converted to int64 65535.
const SetterControl kSetterControls[] = {
	{"period", 500, [](const FFB_Effect &e) -> Fields { return {e.period}; },
	 [](const FFB_Effect &) -> Fields { return {}; }},
	{"duration", 7000, [](const FFB_Effect &e) -> Fields { return {static_cast<int64_t>(e.duration)}; },
	 [](const FFB_Effect &) -> Fields { return {}; }},
	{"deadzone", 300, [](const FFB_Effect &e) -> Fields { return {e.conditions[0].deadBand}; },
	 [](const FFB_Effect &e) -> Fields { return {e.conditions[1].deadBand}; }},
	// offset sets the center point of the condition of the instance and the periodic offset of the effect
	{"offset", -77, [](const FFB_Effect &e) -> Fields { return {e.conditions[0].cpOffset, e.offset}; },
	 [](const FFB_Effect &e) -> Fields { return {e.conditions[1].cpOffset}; }},
	{"sat", 4000,
	 [](const FFB_Effect &e) -> Fields { return {e.conditions[0].negativeSaturation, e.conditions[0].positiveSaturation}; },
	 [](const FFB_Effect &e) -> Fields { return {e.conditions[1].negativeSaturation, e.conditions[1].positiveSaturation}; }},
	{"coeff", -55,
	 [](const FFB_Effect &e) -> Fields { return {e.conditions[0].negativeCoefficient, e.conditions[0].positiveCoefficient}; },
	 [](const FFB_Effect &e) -> Fields { return {e.conditions[1].negativeCoefficient, e.conditions[1].positiveCoefficient}; }},
	{"axisgain", 65535, [](const FFB_Effect &e) -> Fields { return {static_cast<int64_t>(e.axisMagnitudes[0] * 65535.0f)}; },
	 [](const FFB_Effect &) -> Fields { return {}; }},
	{"state", 0, [](const FFB_Effect &e) -> Fields { return {e.state}; },
	 [](const FFB_Effect &) -> Fields { return {}; }},
};
} // namespace

TEST_CASE_FIXTURE(SerialFfb32Fixture, "bug 225: a setat at a valid address is read back by getat and stored in the fields of instance 0 only") {
	// state: the fixture's state of slot 1 is 1, so setting 0 shows an inverted or ignored write
	for (const SetterControl &c : kSetterControls) {
		CAPTURE(std::string(c.name));
		Fields other = c.otherInstance(calc->effects[1]); // the fill() values of instance 1
		std::vector<CommandReply> replies;
		CHECK(run(c.name, CMDtype::setat, 1, c.val, replies) == CommandStatus::OK);
		for (int64_t f : c.written(calc->effects[1])) {
			CHECK(f == c.val); // every written field, e.g. both saturations
		}
		replies.clear();
		CHECK(run(c.name, CMDtype::getat, 1, 0, replies) == CommandStatus::OK);
		REQUIRE(!replies.empty());
		for (const CommandReply &r : replies) {
			CHECK(r.val == c.val); // the commands with two fields answer both, both read back the set value
		}
		CHECK(c.otherInstance(calc->effects[1]) == other); // instance 1 fields untouched
	}
}

// ============================================================================================ setat boundary (both widths)

TEST_CASE_FIXTURE(SerialFfb32Fixture, "bug 225: a setat at the last effect max_effects - 1 is answered OK and changes that slot only") {
	// Values differ from the defaults of the unused slot 39 (val 1000: state becomes 1 vs 0, duration 1000 vs infinite, the others vs 0)
	for (const char *name : {"period", "duration", "deadzone", "offset", "sat", "coeff", "axisgain", "state"}) {
		CAPTURE(std::string(name));
		std::vector<SlotData> before = snapshot();
		std::vector<CommandReply> replies;
		CHECK(run(name, CMDtype::setat, kMaxEffects - 1, 1000, replies) == CommandStatus::OK);
		std::vector<SlotData> after = snapshot();
		for (size_t i = 0; i < after.size(); i++) {
			if (i == static_cast<size_t>(kMaxEffects - 1)) {
				CHECK(after[i] != before[i]);
			} else {
				CHECK(after[i] == before[i]);
			}
		}
	}
}

TEST_CASE_FIXTURE(SerialFfb32Fixture, "bug 225: a setat at the address max_effects is rejected and changes no effect") {
	for (const char *name : {"period", "duration", "deadzone", "offset", "sat", "coeff", "axisgain", "state"}) {
		expectRejected(name, CMDtype::setat, kMaxEffects, 1000);
	}
}

// ============================================================================================ CDC end to end

TEST_CASE_FIXTURE(TargetFixture,
				  "bug 225: fxm.0.type?-1 on the CDC port is answered with an error and changes no effect" *
					  KNOWN_ISSUE_32BIT_UB(BUG225_TEXT)) {
	FFBoardMain main;
	auto calc = std::make_shared<EffectsCalculator>();
	SerialFFB fxm(calc);
	CommandHelpers::CdcConsole console(main);
	REQUIRE(console.send("fxm.0.new=4\n") == "[fxm.0.new=4|0]\n");

	// CommandInterface appends "ERR" for CommandStatus::ERR (CommandInterface.cpp:152)
	CHECK(console.send("fxm.0.type?-1\n") == "[fxm.0.type?-1|ERR]\n");
	CHECK(calc->effects[0].type == 4);
}

} // TEST_SUITE
