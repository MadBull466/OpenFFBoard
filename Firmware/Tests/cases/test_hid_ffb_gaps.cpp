/*
 * HidFFB.cpp: behaviour not covered by test_hid_ffb.cpp (P1 gaps)
 *
 * - filter set-up when an effect is started (review survivor HidFFB.cpp:423)
 * - set effect with a different effect type (HidFFB.cpp:301-304, two suspected defects)
 * - deleting the handler through its UsbHidHandler base
 *
 * Filter state is observed through the force of a damper: with a constant speed the
 * lowpass filtered damper force rises slowly from 0 to its settled value.
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "FfbFixture.h"

#include <cmath>
#include <cstdlib>
#include <memory>

namespace {

/** A running damper on the X axis with a constant speed and its settled (filtered) torque */
struct DamperFixture : FfbFixture {
	uint8_t damper = 0;
	int32_t settled = 0;

	DamperFixture() {
		damper = startEffect(FFB_EFFECT_DAMPER);
		host.setCondition(damper, 0, 0, 10000, 10000, 10000, 10000, 0);
		metrics().speed = 200;
		run(1500);
		settled = axes[0]->ffbEffectTorque;
	}
};

} // namespace

TEST_SUITE("HidFFB") {

// ---------------------------------------------------------------------------
// Filters on effect start (review survivor: HidFFB.cpp:423 '!=' -> '==')
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(DamperFixture, "starting a stopped effect starts its filter from zero") {
	REQUIRE(std::abs(settled) > 1000);

	host.stopEffect(damper);
	run(10);
	CHECK(axes[0]->ffbEffectTorque == 0);

	host.startEffect(damper);
	int32_t first = torque();
	CHECK(first * settled >= 0);			   // Same direction
	CHECK(std::abs(first) < std::abs(settled) / 10); // Filter state was reset: no jump to the old output
}

TEST_CASE_FIXTURE(DamperFixture, "restarting a running effect keeps its filter state") {
	REQUIRE(std::abs(settled) > 1000);

	host.startEffect(damper); // Already running
	CHECK_NEAR(torque(), settled, std::abs(settled) / 100 + 1);
}

// ---------------------------------------------------------------------------
// Set Effect with a different type
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(FfbFixture, "set effect with another effect type resets the start time") {
	uint8_t index = host.allocateEffect(FFB_EFFECT_CONSTANT);
	host.setEffect(index, FFB_EFFECT_CONSTANT);
	HostPlatform::setTimeMs(500);
	host.startEffect(index);
	REQUIRE(effect(index).startTime == 500);

	host.setEffect(index, FFB_EFFECT_DAMPER);
	CHECK(effect(index).type == FFB_EFFECT_DAMPER);
	CHECK(effect(index).startTime == 0);
}

TEST_CASE_FIXTURE(FfbFixture, "set effect with another effect type sets up the filters of the new type" *
		KNOWN_ISSUE("HidFFB.cpp:301-307 calls set_filters before the new type is stored, so the filters are set up for the previous type. "
					"A constant force changed to a damper keeps the 500Hz constant force filter")) {
	uint8_t index = host.allocateEffect(FFB_EFFECT_CONSTANT);
	host.setEffect(index, FFB_EFFECT_CONSTANT);
	REQUIRE(effect(index).filter[0]->getFc() == doctest::Approx(0.5)); // Constant force filter 500Hz at 1kHz

	host.setEffect(index, FFB_EFFECT_DAMPER);
	REQUIRE(effect(index).type == FFB_EFFECT_DAMPER);
	CHECK(effect(index).filter[0]->getFc() == doctest::Approx(0.03)); // Damper filter 30Hz at 1kHz. First failing check shows the defect
	CHECK(effect(index).filter[1]->getFc() == doctest::Approx(0.03));
	CHECK(effect(index).filter[0]->getQ() == doctest::Approx(0.4));
}

TEST_CASE_FIXTURE(FfbFixture, "a running effect changed to a damper by set effect has damper filters" *
		KNOWN_ISSUE("HidFFB.cpp:301-307: set_filters runs with the previous type (sine: no filters) and the start at line 423 skips "
					"set_filters for a running effect. The damper then calls process() on a null filter (EffectsCalculator.cpp:425): "
					"segmentation fault on the host, HardFault on the target")) {
	uint8_t index = startEffect(FFB_EFFECT_SINE); // Running, no filters
	host.setEffect(index, FFB_EFFECT_DAMPER);
	host.setCondition(index, 0, 0, 10000, 10000, 10000, 10000, 0);
	REQUIRE(effect(index).state == 1);
	// First failing check shows the defect. REQUIRE stops the test before the calculation dereferences the null filter
	REQUIRE(effect(index).filter[0] != nullptr);
	REQUIRE(effect(index).filter[1] != nullptr);
	metrics().speed = 200;
	CHECK(torque() != 0);
}

TEST_CASE_FIXTURE(FfbFixture, "set effect with the same effect type keeps the start time") {
	uint8_t index = host.allocateEffect(FFB_EFFECT_CONSTANT);
	host.setEffect(index, FFB_EFFECT_CONSTANT);
	HostPlatform::setTimeMs(500);
	host.startEffect(index);

	host.setEffect(index, FFB_EFFECT_CONSTANT);
	CHECK(effect(index).startTime == 500);
}

// ---------------------------------------------------------------------------
// Lifetime
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(FirmwareFixture, "a HID FFB handler deleted through its HID handler base releases the effects calculator") {
	auto calc = std::make_shared<EffectsCalculator>();
	std::unique_ptr<UsbHidHandler> handler = std::make_unique<HidFFB>(calc, 2);
	CHECK(calc.use_count() == 2);

	handler.reset();
	CHECK(calc.use_count() == 1);
	UsbHidHandler::globalHidHandler = nullptr; // Not cleared by the firmware (KNOWN_ISSUE in test_hid_ffb.cpp)
}

} // TEST_SUITE
