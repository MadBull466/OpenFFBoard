/*
 * EffectsCalculator.cpp: behaviour not covered by test_effects_calculator.cpp (P1 gaps)
 *
 * - square wave phase (review survivor EffectsCalculator.cpp:235)
 * - update rate measurement at any uptime (review survivor EffectsCalculator.cpp:1275)
 * - the statistics thread Run(), getInfo(), deletion through the CommandHandler base
 * - reconstruction filter guards (equal timestamps, natural spline not initialised)
 * - remaining fx commands (filterCfQ info, effectsDetails set, friction_q/inertia_q set, monitorEffect)
 *
 * Same conventions as test_effects_calculator.cpp: a positive effect force results in a negative axis torque.
 * The reconstruction tests here never evaluate an initialised CMSIS-DSP spline, so they do not reach
 * the arm_spline_f32 over-read (suspected bug 2) and need no sanitizer exclusion.
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "FfbFixture.h"
#include "MockCommandInterface.h"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace {

/** Starts a periodic effect and reads the X axis torque at a time after the start (effect starts at t = 0) */
struct SquareFixture : FfbFixture {
	uint8_t startSquare(uint16_t magnitude, uint16_t phase, uint32_t period) {
		uint8_t index = startEffect(FFB_EFFECT_SQUARE);
		host.setPeriodic(index, magnitude, 0, phase, period);
		return index;
	}
	int32_t torqueAt(uint32_t ms) {
		HostPlatform::setTimeMs(ms);
		return torque(0);
	}
};

/**
 * Runs EffectsCalculator::Run() in a fake RTOS task.
 * The calculator derives privately from cpp_freertos::Thread and never calls Start() itself
 * (EffectsCalculator.cpp:64), so the test creates the task with the kernel API and passes the calculator.
 */
struct EffectsThreadFixture : FfbFixture {
	TaskHandle_t task = nullptr;

	EffectsThreadFixture() {
		xTaskCreate([](void *calc) { static_cast<EffectsCalculator *>(calc)->Run(); }, "FXCalcTest", EFFECT_THREAD_MEM, ec.get(),
					EFFECT_THREAD_PRIO, &task);
	}
	~EffectsThreadFixture() override { vTaskDelete(task); }
};

/** Reads the single value reply of a get command */
int64_t getValue(FfbFixture &f, EffectsCalculator_commands cmd) {
	std::vector<CommandReply> replies;
	f.fxCommand(cmd, CMDtype::get, 0, &replies);
	REQUIRE(replies.size() == 1);
	return replies[0].val;
}

/** Reads the effectsDetails string of an axis */
std::string details(FfbFixture &f, uint8_t axis = 0) {
	std::vector<CommandReply> replies;
	f.fxCommand(EffectsCalculator_commands::effectsDetails, CMDtype::getat, 0, &replies, axis);
	REQUIRE(replies.size() == 1);
	return replies[0].reply;
}

const std::string NO_STATS_ENTRY = "{\"max\":0, \"curr\":0, \"nb\":0}";

} // namespace

TEST_SUITE("EffectsCalculator") {

// ---------------------------------------------------------------------------
// Square wave phase (review survivor: EffectsCalculator.cpp:235 '+' -> '-')
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(SquareFixture, "a square phase of 18000 shifts the wave at 25 and 75 ms") {
	// Mutation guard for EffectsCalculator.cpp:235 ('+' -> '-'). It passes only because 18000 % 102 = 48 is close to
	// half of period+2 (51); the code does not really invert the wave (e.g. not at t = 1 ms). The phase handling is a
	// suspected bug, see the KNOWN_ISSUE test "square phase is a fraction of the period like for the other periodic effects".
	// Phase 0: +5000 torque in the first half, -5000 in the second (see "characterization: square starts with the negative half")
	startSquare(5000, 18000, 100);
	CHECK(torqueAt(25) == -5000);
	CHECK(torqueAt(75) == 5000);
}

TEST_CASE_FIXTURE(SquareFixture, "square phase is a fraction of the period like for the other periodic effects" *
		KNOWN_ISSUE("EffectsCalculator.cpp:235 adds the raw phase (0.01 degrees) as milliseconds instead of scaling it with the period. "
					"90 degrees on a 1000ms square shifts the wave by 984ms instead of 250ms")) {
	// 90 degrees = 250ms on a 1000ms period: torque(t) = torque without phase at t + 250ms
	startSquare(5000, 9000, 1000);
	CHECK(torqueAt(300) == -5000); // 550ms into the unshifted wave: second half. First failing check shows the defect
	CHECK(torqueAt(800) == 5000);  // 1050ms: first half of the next period
}

// ---------------------------------------------------------------------------
// Update rate measurement (review survivor: EffectsCalculator.cpp:1275 '-' -> '+')
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(FfbFixture, "constant force rate is measured at any uptime and drops to 0 one second after the last update") {
	uint8_t index = startEffect(FFB_EFFECT_CONSTANT);
	HostPlatform::setTimeMs(5000);
	for (int i = 0; i < 100; i++) {
		HostPlatform::advanceMs(10);
		host.setConstantForce(index, 1000);
	}
	CHECK(ffb.getConstantForceRate() == doctest::Approx(100).epsilon(0.02));

	HostPlatform::advanceMs(999); // Still within one second of the last update
	CHECK(ffb.getConstantForceRate() == doctest::Approx(100).epsilon(0.02));

	HostPlatform::advanceMs(2);
	CHECK(ffb.getConstantForceRate() == 0);
}

TEST_CASE_FIXTURE(FfbFixture, "report rate is measured at any uptime and drops to 0 one second after the last report") {
	uint8_t index = startEffect(FFB_EFFECT_SINE);
	HostPlatform::setTimeMs(5000);
	for (int i = 0; i < 100; i++) {
		HostPlatform::advanceMs(4);
		host.setPeriodic(index, 1000, 0, 0, 100);
	}
	CHECK(ffb.getRate() == doctest::Approx(250).epsilon(0.02));

	HostPlatform::advanceMs(999);
	CHECK(ffb.getRate() == doctest::Approx(250).epsilon(0.02));

	HostPlatform::advanceMs(2);
	CHECK(ffb.getRate() == 0);
}

// ---------------------------------------------------------------------------
// Class info and lifetime
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(FfbFixture, "class info identifies the hidden effects class") {
	CommandHandler *handler = ec.get();
	ClassIdentifier info = handler->getInfo(); // Virtual call through the command handler base
	CHECK(std::string(info.name) == "Effects");
	CHECK(info.id == CLSID_EFFECTSCALC);
	CHECK(info.visibility == ClassVisibility::hidden);
	CHECK(ec->getInfo().id == EffectsCalculator::info.id);
}

TEST_CASE_FIXTURE(FirmwareFixture, "an effects calculator deleted through its command handler base is unregistered") {
	std::unique_ptr<CommandHandler> handler = std::make_unique<EffectsCalculator>();
	CommandHandler *raw = handler.get();
	auto &handlers = CommandHandler::getCommandHandlers();
	CHECK(std::find(handlers.begin(), handlers.end(), raw) != handlers.end());

	handler.reset();
	CHECK(std::find(handlers.begin(), handlers.end(), raw) == handlers.end());
}

// ---------------------------------------------------------------------------
// Statistics thread Run()
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(EffectsThreadFixture, "effects thread waits 500ms after start and then wakes every 3s") {
	// Run() never starts on the target: Start() is commented out in the constructor (EffectsCalculator.cpp:64).
	// The test starts it through EffectsThreadFixture. See PROGRESS.md NEEDS REVIEW #9.
	SUBCASE("first wake up after 500ms, next wait is 3s") {
		HostRtos::setDelayBudgetMs(1000); // Allows the 500ms delay; the following 3000ms delay blocks
		CHECK(HostRtos::runUntilBlocked(task));
		CHECK(HostPlatform::nowUs() == (500 + 3000) * 1000ULL);
	}
	SUBCASE("the loop keeps waiting 3s per cycle") {
		HostRtos::setDelayBudgetMs(10000); // 500 + 3 x 3000 fit, the 4th 3000ms delay blocks
		CHECK(HostRtos::runUntilBlocked(task));
		CHECK(HostPlatform::nowUs() == (500 + 4 * 3000) * 1000ULL);
	}
}

TEST_CASE_FIXTURE(EffectsThreadFixture, "characterization: effects thread broadcasts nothing even with effect monitoring enabled") {
	// NEEDS REVIEW (PROGRESS.md #9): Run() skips the effectsDetails broadcast with `continue` ("TODO uncomment when stream is ok",
	// EffectsCalculator.cpp:1211) and the thread is never started (constructor, line 64). The monitorEffect command
	// is not registered (line 51 commented out). Is the monitoring stream meant to be enabled later?
	MockCommandInterface observer;
	fxCommand(EffectsCalculator_commands::monitorEffect, CMDtype::set, 1);
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(cf, 1000);
	calculate();

	HostRtos::setDelayBudgetMs(10000);
	CHECK(HostRtos::runUntilBlocked(task));
	CHECK(HostPlatform::nowUs() == (500 + 4 * 3000) * 1000ULL); // The loop ran several cycles
	CHECK(observer.replies.empty());
}

TEST_CASE_FIXTURE(FfbFixture, "effect monitoring flag can be written and read and is limited to 0..1") {
	// The monitorEffect command is not registered (EffectsCalculator.cpp:51 commented out), so it is reachable only by
	// its command id, not by name. See PROGRESS.md NEEDS REVIEW #9.
	CHECK(getValue(*this, EffectsCalculator_commands::monitorEffect) == 0);
	fxCommand(EffectsCalculator_commands::monitorEffect, CMDtype::set, 1);
	CHECK(getValue(*this, EffectsCalculator_commands::monitorEffect) == 1);
	fxCommand(EffectsCalculator_commands::monitorEffect, CMDtype::set, 7);
	CHECK(getValue(*this, EffectsCalculator_commands::monitorEffect) == 1);
	fxCommand(EffectsCalculator_commands::monitorEffect, CMDtype::set, 0);
	CHECK(getValue(*this, EffectsCalculator_commands::monitorEffect) == 0);
}

// ---------------------------------------------------------------------------
// Unsupported effect type
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(FfbFixture, "custom effects are not calculated and add no force") {
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(cf, 3000);
	uint8_t custom = startEffect(FFB_EFFECT_CUSTOM);
	REQUIRE(custom != 0);
	effect(custom).magnitude = 10000; // Not settable by any report; would create force if the type was calculated
	CHECK(effect(custom).state == 1);
	CHECK(torque(0) == -3000);
	CHECK(torque(1) == 0);
}

// ---------------------------------------------------------------------------
// Reconstruction filter guards
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(FfbFixture, "linear reconstruction uses the latest value if two updates arrive at the same time") {
	fxCommand(EffectsCalculator_commands::reconFilterMode, CMDtype::set, (int64_t)ReconFilterMode::LINEAR_INTERPOLATION);
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	HostPlatform::setTimeMs(1000);
	host.setConstantForce(cf, 2000);
	host.setConstantForce(cf, 6000); // No time between the updates: interval 0
	CHECK(torque() == -6000);
	HostPlatform::advanceMs(5);
	CHECK(torque() == -6000);
}

TEST_CASE_FIXTURE(FfbFixture, "hermite reconstruction holds the older point if the interpolation points share a timestamp") {
	// EffectsCalculator.cpp:717: interval <= 0 returns P1 instead of dividing by zero
	fxCommand(EffectsCalculator_commands::reconFilterMode, CMDtype::set, (int64_t)ReconFilterMode::SPLINE_CUBIC_HERMITE);
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	HostPlatform::setTimeMs(1000);
	host.setConstantForce(cf, 1000);
	host.setConstantForce(cf, 2000);
	host.setConstantForce(cf, 3000); // P1 and P2 now both have the time of these updates
	int32_t t = torque();
	CHECK(t == -1000);
	HostPlatform::advanceMs(5);
	CHECK(torque() == -1000);
}

#ifdef USE_DSP_FUNCTIONS
TEST_CASE_FIXTURE(FfbFixture, "natural spline reconstruction holds the latest value until a sample is received in that mode") {
	// Samples received in another mode do not initialise the CMSIS-DSP spline (EffectsCalculator.cpp:683)
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	HostPlatform::setTimeMs(1000);
	host.setConstantForce(cf, 2000);
	HostPlatform::setTimeMs(1010);
	host.setConstantForce(cf, 6000);
	fxCommand(EffectsCalculator_commands::reconFilterMode, CMDtype::set, (int64_t)ReconFilterMode::SPLINE_CUBIC_NATURAL);
	HostPlatform::setTimeMs(1015);
	CHECK(torque() == -6000);
}
#else
TEST_CASE_FIXTURE(FfbFixture, "without CMSIS-DSP the natural spline mode uses the hermite reconstruction") {
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	HostPlatform::setTimeMs(1000);
	host.setConstantForce(cf, 2000);
	HostPlatform::setTimeMs(1010);
	host.setConstantForce(cf, 6000);
	fxCommand(EffectsCalculator_commands::reconFilterMode, CMDtype::set, (int64_t)ReconFilterMode::SPLINE_CUBIC_NATURAL);
	HostPlatform::setTimeMs(1015);
	int32_t natural = torque();
	fxCommand(EffectsCalculator_commands::reconFilterMode, CMDtype::set, (int64_t)ReconFilterMode::SPLINE_CUBIC_HERMITE);
	CHECK(torque() == natural);
	CHECK(natural == -2000); // Hermite interpolates between the two older points (both 2000 here)
}
#endif

// ---------------------------------------------------------------------------
// Statistics commands
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(FfbFixture, "statistics for a not existing axis are ignored") {
	ec->calcStatsEffectType(FFB_EFFECT_CONSTANT, 1000, MAX_AXIS);
	for (uint8_t axis = 0; axis < MAX_AXIS; axis++) {
		CAPTURE(axis);
		CHECK(details(*this, axis).rfind(NO_STATS_ENTRY, 0) == 0);
	}
}

TEST_CASE_FIXTURE(FfbFixture, "effect details of a not existing axis are empty") {
	CHECK(details(*this, MAX_AXIS).empty());
	CHECK_FALSE(details(*this, 0).empty());
}

TEST_CASE_FIXTURE(FfbFixture, "resetting the used effects with 0 keeps the types of allocated effects") {
	host.allocateEffect(FFB_EFFECT_CONSTANT);
	uint8_t sine = host.allocateEffect(FFB_EFFECT_SINE);
	host.blockFree(sine);
	REQUIRE(getValue(*this, EffectsCalculator_commands::effects) == ((1 << (FFB_EFFECT_CONSTANT - 1)) | (1 << (FFB_EFFECT_SINE - 1))));

	fxCommand(EffectsCalculator_commands::effects, CMDtype::set, 0);
	CHECK(getValue(*this, EffectsCalculator_commands::effects) == (1 << (FFB_EFFECT_CONSTANT - 1)));

	fxCommand(EffectsCalculator_commands::effects, CMDtype::set, 1); // Any other value clears all
	CHECK(getValue(*this, EffectsCalculator_commands::effects) == 0);
}

TEST_CASE_FIXTURE(FfbFixture, "setting effect details to 0 resets the maximum forces only") {
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(cf, 10000);
	calculate();
	host.setConstantForce(cf, 4000);
	calculate();
	REQUIRE(details(*this).rfind("{\"max\":10000, \"curr\":-4000, \"nb\":1}", 0) == 0);

	fxCommand(EffectsCalculator_commands::effectsDetails, CMDtype::set, 0);
	CHECK(details(*this).rfind("{\"max\":0, \"curr\":-4000, \"nb\":1}", 0) == 0);
	CHECK(getValue(*this, EffectsCalculator_commands::effects) == (1 << (FFB_EFFECT_CONSTANT - 1)));
}

TEST_CASE_FIXTURE(FfbFixture, "setting effect details to a positive value resets all statistics and the used effects") {
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(cf, 10000);
	calculate();

	fxCommand(EffectsCalculator_commands::effectsDetails, CMDtype::set, 1);
	CHECK(details(*this).rfind(NO_STATS_ENTRY + ", " + NO_STATS_ENTRY, 0) == 0);
	CHECK(getValue(*this, EffectsCalculator_commands::effects) == 0);
}

TEST_CASE_FIXTURE(FfbFixture, "setting effect details to a negative value changes nothing") {
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(cf, 10000);
	calculate();
	std::string before = details(*this);

	fxCommand(EffectsCalculator_commands::effectsDetails, CMDtype::set, -1);
	CHECK(details(*this) == before);
	CHECK(getValue(*this, EffectsCalculator_commands::effects) == (1 << (FFB_EFFECT_CONSTANT - 1)));
}

// ---------------------------------------------------------------------------
// Filter settings
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(FfbFixture, "constant force filter Q info reports the Q scale") {
	std::vector<CommandReply> replies;
	fxCommand(EffectsCalculator_commands::ffbfiltercf_q, CMDtype::info, 0, &replies);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "scale:0.010000");
}

TEST_CASE_FIXTURE(FfbFixture, "friction and inertia filter Q are written to the custom profile and applied to existing effects") {
	uint8_t friction = host.allocateEffect(FFB_EFFECT_FRICTION);
	uint8_t inertia = host.allocateEffect(FFB_EFFECT_INERTIA);
	fxCommand(EffectsCalculator_commands::filterProfileId, CMDtype::set, CUSTOM_PROFILE_ID);

	fxCommand(EffectsCalculator_commands::friction_q, CMDtype::set, 35);
	fxCommand(EffectsCalculator_commands::inertia_q, CMDtype::set, 55);
	CHECK(getValue(*this, EffectsCalculator_commands::friction_q) == 35);
	CHECK(getValue(*this, EffectsCalculator_commands::inertia_q) == 55);
	CHECK(effect(friction).filter[0]->getQ() == doctest::Approx(0.35));
	CHECK(effect(inertia).filter[0]->getQ() == doctest::Approx(0.55));
	CHECK(effect(inertia).filter[1]->getQ() == doctest::Approx(0.55));
}

TEST_CASE_FIXTURE(FfbFixture, "friction and inertia filter Q changes do not affect effects while the default profile is active") {
	uint8_t friction = host.allocateEffect(FFB_EFFECT_FRICTION);
	uint8_t inertia = host.allocateEffect(FFB_EFFECT_INERTIA);

	fxCommand(EffectsCalculator_commands::friction_q, CMDtype::set, 35);
	fxCommand(EffectsCalculator_commands::inertia_q, CMDtype::set, 55);
	CHECK(getValue(*this, EffectsCalculator_commands::friction_q) == 20); // Default profile values
	CHECK(getValue(*this, EffectsCalculator_commands::inertia_q) == 20);
	CHECK(effect(friction).filter[0]->getQ() == doctest::Approx(0.20));
	CHECK(effect(inertia).filter[0]->getQ() == doctest::Approx(0.20));

	fxCommand(EffectsCalculator_commands::filterProfileId, CMDtype::set, CUSTOM_PROFILE_ID);
	CHECK(getValue(*this, EffectsCalculator_commands::friction_q) == 35);
	CHECK(effect(friction).filter[0]->getQ() == doctest::Approx(0.35));
	CHECK(effect(inertia).filter[0]->getQ() == doctest::Approx(0.55));
}

TEST_CASE_FIXTURE(FfbFixture, "friction and inertia filter Q of 0 is raised to 1") {
	fxCommand(EffectsCalculator_commands::filterProfileId, CMDtype::set, CUSTOM_PROFILE_ID);
	fxCommand(EffectsCalculator_commands::friction_q, CMDtype::set, 0);
	fxCommand(EffectsCalculator_commands::inertia_q, CMDtype::set, 0);
	CHECK(getValue(*this, EffectsCalculator_commands::friction_q) == 1);
	CHECK(getValue(*this, EffectsCalculator_commands::inertia_q) == 1);
}

} // TEST_SUITE
