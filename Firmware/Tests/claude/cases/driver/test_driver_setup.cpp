/*
 * Driver track skeleton (Tests/claude/plans/DRIVER_TEST_PLAN.md, phase D0): the feature set of ffb_driver_tests.
 * platform/target_f407_drivers/target_constants.h enables the TMC4671, ODrive and VESC drivers on top of the
 * ffb_target_tests feature set; since P13 it also enables TMCDEBUG (TMCDebugBridge). The out-of-scope drivers and MIDI
 * stay off.
 */
#include "doctest.h"

#include "constants.h"

TEST_SUITE("DriverSetup") {

TEST_CASE("the executable enables the TMC4671, ODrive and VESC drivers, TMCDEBUG and none of the out-of-scope features") {
#ifdef TMC4671DRIVER
	CHECK(true);
#else
	FAIL("TMC4671DRIVER is not defined");
#endif
#ifdef ODRIVE
	CHECK(true);
#else
	FAIL("ODRIVE is not defined");
#endif
#ifdef VESC
	CHECK(true);
#else
	FAIL("VESC is not defined");
#endif
#ifdef TMCDEBUG
	CHECK(true);
#else
	FAIL("TMCDEBUG is not defined (P13: TMCDebugBridge is in scope in this executable)");
#endif
#if defined(MIDI) || defined(RMDCAN) || defined(PWMDRIVER) || defined(SIMPLEMOTION)
	FAIL("an out-of-scope feature macro (MIDI, RMDCAN, PWMDRIVER, SIMPLEMOTION) is defined");
#endif
}

}
