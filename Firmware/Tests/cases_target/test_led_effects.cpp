/*
 * Tests of the LED effects (ledEffects.cpp): pulses, blinking and static LED states of the sys, err and clip LEDs
 * against the simulated time.
 *
 * In ffb_target_tests the public pulse/set functions are counting wrappers in host_platform.cpp that forward to the
 * real functions (HOSTTEST_REAL_LED_EFFECTS). These tests never look at the counters: they observe the real output
 * pins (HostTarget::outputPin, the ODR written by HAL_GPIO_WritePin / HAL_GPIO_TogglePin).
 *
 * updateLeds() is called by the cppmain loop about once per millisecond; the helper LedFixture::run does the same.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "ledEffects.h"

#include <cstdint>
#include <vector>

namespace {

struct Led {
	GPIO_TypeDef *port;
	uint16_t pin;
};
const Led SYS{LED_SYS_GPIO_Port, LED_SYS_Pin};
const Led ERR{LED_ERR_GPIO_Port, LED_ERR_Pin};
const Led CLIP{LED_CLIP_GPIO_Port, LED_CLIP_Pin};

struct LedFixture : TargetFixture {
	LedFixture() {
		// Start with all LEDs off and at a time > 0 (main.c switches them on before the scheduler: see the reset test)
		setSysLed(0);
		setErrLed(0);
		setClipLed(0);
		HostPlatform::setTimeMs(1000);
	}

	static bool on(const Led &led) { return HostTarget::outputPin(led.port, led.pin); }

	/** Advances the time by `ms` milliseconds, calling updateLeds() after every millisecond like the cppmain loop */
	static void run(uint32_t ms) {
		for (uint32_t i = 0; i < ms; i++) {
			HostPlatform::advanceMs(1);
			updateLeds();
		}
	}

	/** Like run(), returns the level of `led` after every millisecond */
	static std::vector<bool> record(const Led &led, uint32_t ms) {
		std::vector<bool> levels;
		for (uint32_t i = 0; i < ms; i++) {
			run(1);
			levels.push_back(on(led));
		}
		return levels;
	}

	/** Number of off->on transitions in `levels`, starting from level `initial` */
	static int risingEdges(bool initial, const std::vector<bool> &levels) {
		int edges = 0;
		bool last = initial;
		for (bool l : levels) {
			edges += (l && !last) ? 1 : 0;
			last = l;
		}
		return edges;
	}

	/** Lengths of the runs of equal levels in `levels` */
	static std::vector<size_t> phases(const std::vector<bool> &levels) {
		std::vector<size_t> lengths;
		for (size_t i = 0; i < levels.size(); i++) {
			if (i == 0 || levels[i] != levels[i - 1]) {
				lengths.push_back(0);
			}
			lengths.back()++;
		}
		return lengths;
	}
};

} // namespace

TEST_SUITE("LedEffects") {

// ---------------------------------------------------------------------------------------------------------------------
// Static LED states
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TargetFixture, "without an effect updateLeds leaves the LEDs as they are") {
	// Precondition (fake platform): main.c switched the three LEDs on before the scheduler started
	REQUIRE(HostTarget::outputPin(LED_SYS_GPIO_Port, LED_SYS_Pin));
	REQUIRE(HostTarget::outputPin(LED_ERR_GPIO_Port, LED_ERR_Pin));
	REQUIRE(HostTarget::outputPin(LED_CLIP_GPIO_Port, LED_CLIP_Pin));
	for (int i = 0; i < 1000; i++) {
		HostPlatform::advanceMs(1);
		updateLeds();
	}
	CHECK(HostTarget::outputPin(LED_SYS_GPIO_Port, LED_SYS_Pin));
	CHECK(HostTarget::outputPin(LED_ERR_GPIO_Port, LED_ERR_Pin));
	CHECK(HostTarget::outputPin(LED_CLIP_GPIO_Port, LED_CLIP_Pin));
}

TEST_CASE_FIXTURE(LedFixture, "setSysLed, setErrLed and setClipLed switch only their own LED") {
	setSysLed(1);
	CHECK(on(SYS));
	CHECK_FALSE(on(ERR));
	CHECK_FALSE(on(CLIP));

	setErrLed(1);
	CHECK(on(ERR));
	CHECK_FALSE(on(CLIP));

	setClipLed(1);
	CHECK(on(CLIP));

	setSysLed(0);
	setErrLed(0);
	CHECK_FALSE(on(SYS));
	CHECK_FALSE(on(ERR));
	CHECK(on(CLIP));
}

TEST_CASE_FIXTURE(LedFixture, "setLed treats any non-zero value as on") {
	setErrLed(2);
	CHECK(on(ERR));
}

// ---------------------------------------------------------------------------------------------------------------------
// Pulses
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(LedFixture, "an err LED pulse switches the LED on at once and off after 250 ms") {
	pulseErrLed();
	CHECK(on(ERR));
	run(249);
	CHECK(on(ERR));
	run(2); // off at 250 or 251 ms: the exact millisecond is NEEDS REVIEW 70
	CHECK_FALSE(on(ERR));
}

TEST_CASE_FIXTURE(LedFixture, "a clip LED pulse switches the LED on at once and off after 100 ms") {
	pulseClipLed();
	CHECK(on(CLIP));
	run(99);
	CHECK(on(CLIP));
	run(2); // NEEDS REVIEW 70: 100 or 101 ms
	CHECK_FALSE(on(CLIP));
}

TEST_CASE_FIXTURE(LedFixture, "a sys LED pulse switches the LED on at once and off after 25 ms") {
	pulseSysLed();
	CHECK(on(SYS));
	run(24);
	CHECK(on(SYS));
	run(2); // NEEDS REVIEW 70: 25 or 26 ms
	CHECK_FALSE(on(SYS));
}

TEST_CASE_FIXTURE(LedFixture, "a pulse only affects its own LED") {
	pulseErrLed();
	run(10);
	CHECK(on(ERR));
	CHECK_FALSE(on(SYS));
	CHECK_FALSE(on(CLIP));
}

TEST_CASE_FIXTURE(LedFixture, "a pulse during a running pulse does not extend it") {
	pulseErrLed();
	run(200);
	pulseErrLed();
	run(51);
	CHECK_FALSE(on(ERR)); // ended 251 ms after the first pulse
}

TEST_CASE_FIXTURE(LedFixture, "a pulse ends completely one period after its LED went off, then a new pulse starts again") {
	pulseClipLed();
	run(101); // off (last blink step)
	REQUIRE_FALSE(on(CLIP));
	pulseClipLed(); // still finishing: ignored
	CHECK_FALSE(on(CLIP));
	run(101); // effect finished
	pulseClipLed();
	CHECK(on(CLIP));
}

TEST_CASE_FIXTURE(LedFixture, "a pulse of an LED that is switched on blinks it off and restores it") {
	setErrLed(1);
	pulseErrLed();
	CHECK_FALSE(on(ERR));
	run(251);
	CHECK(on(ERR)); // toggled back on
	run(251);
	CHECK(on(ERR)); // end of the effect writes the static state (on)
}

TEST_CASE_FIXTURE(LedFixture, "a static state set during a pulse is the level after the pulse") {
	pulseClipLed();
	setClipLed(1); // e.g. EffectsCalculator reports clipping
	run(400);
	CHECK(on(CLIP));
}

TEST_CASE_FIXTURE(LedFixture, "updateLeds advances the effects of all three LEDs") {
	pulseSysLed();
	pulseErrLed();
	pulseClipLed();
	HostPlatform::advanceMs(251);
	updateLeds(); // a single late update
	CHECK_FALSE(on(SYS));
	CHECK_FALSE(on(ERR));
	CHECK_FALSE(on(CLIP));
}

TEST_CASE_FIXTURE(LedFixture, "without updateLeds a pulse never ends") {
	pulseErrLed();
	HostPlatform::advanceMs(10000);
	CHECK(on(ERR));
}

TEST_CASE_FIXTURE(LedFixture, "a sys LED pulse started 16 ms before the 32 bit tick wraps around lasts 25 ms" *
		  KNOWN_ISSUE("(bug 48) ledEffects.cpp:85 compares HAL_GetTick() > tick + period: tick + period wraps to a small value, so the "
					  "pulse ends at the next update (wrap-safe: HAL_GetTick() - tick > period)")) {
	HostPlatform::setTimeMs(0xFFFFFFF0ull);
	REQUIRE(HAL_GetTick() == 0xFFFFFFF0u);
	pulseSysLed();
	run(1);
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(on(SYS)); // first failing assertion: switched off after 1 ms
	run(23);
	CHECK(on(SYS)); // 24 ms
	run(3);
	CHECK_FALSE(on(SYS)); // 27 ms (25 or 26 ms, NEEDS REVIEW 70)
}

// ---------------------------------------------------------------------------------------------------------------------
// Blinking
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(LedFixture, "blinking an LED 3 times switches it on 3 times and leaves it at its static state") {
	blinkErrLed(20, 3);
	CHECK(on(ERR));
	const std::vector<bool> levels = record(ERR, 1000);
	CHECK(risingEdges(true, levels) + 1 == 3); // the first on phase started with blinkErrLed
	CHECK_FALSE(levels.back());
	pulseErrLed(); // the effect is over: a pulse starts
	CHECK(on(ERR));
}

TEST_CASE_FIXTURE(LedFixture, "characterization: with updateLeds every millisecond each blink phase lasts its period plus 1 ms (NEEDS REVIEW 70)") {
	// NEEDS REVIEW 70: ledEffects.cpp:85 switches when HAL_GetTick() > tick + period (strictly greater), so a period of
	// 20 ms gives 21 ms phases. Intended or off by one?
	blinkSysLed(20, 2);
	const std::vector<bool> levels = record(SYS, 200);
	const std::vector<size_t> p = phases(levels);
	REQUIRE(p.size() == 4); // on, off, on, off (until the end of the recording)
	CHECK(p[0] == 20);		// already on for the 1 ms before the recording started
	CHECK(p[1] == 21);
	CHECK(p[2] == 21);
}

TEST_CASE_FIXTURE(LedFixture, "blinking with 0 blinks continues until it is stopped") {
	blinkClipLed(10, 0);
	const std::vector<bool> levels = record(CLIP, 10000);
	CHECK(risingEdges(true, levels) > 400); // 22 ms per cycle: about 454
	CHECK(risingEdges(true, std::vector<bool>(levels.end() - 100, levels.end())) >= 4);
}

TEST_CASE_FIXTURE(LedFixture, "blinking with period 0 and 0 blinks stops a blink and restores the static state") {
	setSysLed(1);
	blinkSysLed(10, 0);
	run(11);
	REQUIRE(on(SYS)); // second phase (on again after starting off)
	run(11);
	REQUIRE_FALSE(on(SYS));

	blinkSysLed(0, 0);
	CHECK(on(SYS));
	run(100);
	CHECK(on(SYS));
}

TEST_CASE_FIXTURE(LedFixture, "a pulse starts at once after a blink was stopped") {
	blinkErrLed(100, 0);
	run(50);
	blinkErrLed(0, 0);
	REQUIRE_FALSE(on(ERR));
	pulseErrLed();
	CHECK(on(ERR));
}

TEST_CASE_FIXTURE(LedFixture, "a pulse does not interrupt a running blink") {
	blinkErrLed(10, 0);
	const std::vector<bool> before = record(ERR, 55);
	pulseErrLed(); // ignored: the LED is blinking
	const std::vector<bool> after = record(ERR, 55);
	std::vector<bool> all = before;
	all.insert(all.end(), after.begin(), after.end());
	const std::vector<size_t> p = phases(all);
	REQUIRE(p.size() >= 4);
	CHECK(p[1] <= 11); // a blink phase (10 or 11 ms, NEEDS REVIEW 70), not a 250 ms pulse
	for (size_t i = 1; i + 1 < p.size(); i++) {
		CHECK(p[i] == p[1]); // unchanged rhythm
	}
}

TEST_CASE_FIXTURE(LedFixture, "a blink of an LED that is on starts with the LED off") {
	setClipLed(1);
	blinkClipLed(50, 1);
	CHECK_FALSE(on(CLIP));
	run(51);
	CHECK(on(CLIP));
	run(200);
	CHECK(on(CLIP));
}

TEST_CASE_FIXTURE(LedFixture, "a new blink replaces a running blink") {
	blinkErrLed(10, 0);
	run(5);
	blinkErrLed(100, 1); // restarts with the LED on (static state off)
	CHECK(on(ERR));
	run(99);
	CHECK(on(ERR));
	run(2); // NEEDS REVIEW 70: 100 or 101 ms
	CHECK_FALSE(on(ERR));
	run(1000);
	CHECK_FALSE(on(ERR));
}

TEST_CASE_FIXTURE(LedFixture, "characterization: blinking with period 0 and blinks > 0 switches the LED on and never ends (NEEDS REVIEW 71)") {
	// NEEDS REVIEW 71: blinkLed (ledEffects.cpp:25-37) only treats period 0 AND blinks 0 as stop. With period 0 updateLed
	// never runs the effect (line 85), so the LED stays inverted; a later pulse is accepted (period == 0). No caller today.
	blinkSysLed(0, 3);
	CHECK(on(SYS));
	run(1000);
	CHECK(on(SYS));
}

} // TEST_SUITE
