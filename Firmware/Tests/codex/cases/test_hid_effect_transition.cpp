/*
 * P1-002: Set Effect updates parameters without replacing Effect Operation.
 * Functional lifecycle oracle inherited from the existing running-effect case,
 * independent of the unavailable PID specification. Sine and triangle avoid the
 * known conditional-filter setup defect. No waveform shape or timestamp policy
 * is asserted here.
 */
#include "doctest.h"
#include "FfbFixture.h"

TEST_SUITE("HidFFB") {

TEST_CASE_FIXTURE(FfbFixture, "changing a running periodic effect type preserves force until an explicit stop") {
	const uint8_t index = host.allocateEffect(FFB_EFFECT_SINE);
	REQUIRE(index != 0);
	host.setEffect(index, FFB_EFFECT_SINE);
	// Offset exceeds magnitude: either periodic waveform stays strictly positive
	// (4000 +/- 1000), independent of phase and any timestamp reset. Full gain
	// and the fixture's X direction therefore produce nonzero axis torque.
	host.setPeriodic(index, 1000, 4000, 0, 100);
	host.startEffect(index);
	HostPlatform::advanceMs(25);
	REQUIRE(effect(index).state == 1);
	REQUIRE(torque() != 0);

	host.setEffect(index, FFB_EFFECT_TRIANGLE);
	CHECK(effect(index).type == FFB_EFFECT_TRIANGLE);
	CHECK(effect(index).state == 1);
	CHECK(torque() != 0);

	host.stopEffect(index);
	CHECK(torque() == 0);
	host.startEffect(index);
	CHECK(torque() != 0);
}

TEST_CASE_FIXTURE(FfbFixture, "changing a stopped periodic effect type stays silent until an explicit start") {
	const uint8_t index = host.allocateEffect(FFB_EFFECT_SINE);
	REQUIRE(index != 0);
	host.setEffect(index, FFB_EFFECT_SINE);
	host.setPeriodic(index, 1000, 4000, 0, 100);
	host.startEffect(index);
	REQUIRE(torque() != 0);
	host.stopEffect(index);
	REQUIRE(effect(index).state == 0);
	REQUIRE(torque() == 0);

	host.setEffect(index, FFB_EFFECT_TRIANGLE);
	CHECK(effect(index).type == FFB_EFFECT_TRIANGLE);
	CHECK(effect(index).state == 0);
	CHECK(torque() == 0);
	// Positive control: silence above is due to the stopped lifecycle, rather
	// than a missing magnitude or an unusable new type.
	host.startEffect(index);
	CHECK(torque() != 0);
}

} // TEST_SUITE
