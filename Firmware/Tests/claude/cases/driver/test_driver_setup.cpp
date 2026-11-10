/*
 * Driver track skeleton (Tests/claude/plans/DRIVER_TEST_PLAN.md, phase D0): the feature set of ffb_driver_tests.
 * platform/target_f407_drivers/target_constants.h enables the TMC4671, ODrive and VESC drivers on top of the
 * ffb_target_tests feature set; since P13 it also enables TMCDEBUG (TMCDebugBridge) and since P14 PWMDRIVER (MotorPWM) and since P15 SIMPLEMOTION (MotorSimplemotion). MIDI and
 * RMDCAN stay off until their phases (P16-P17).
 */
#include "doctest.h"

#include "constants.h"

TEST_SUITE("DriverSetup") {

TEST_CASE("the executable enables the TMC4671, ODrive, VESC, PWM and SimpleMotion drivers, TMCDEBUG and none of the not yet in-scope features") {
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
#ifdef PWMDRIVER
	CHECK(true);
#else
	FAIL("PWMDRIVER is not defined (P14: MotorPWM is in scope in this executable)");
#endif
#if defined(SIMPLEMOTION)
	CHECK(true);
#elif defined(HOSTTEST_NO_SIMPLEMOTION)
	CHECK(true); // clang build: MotorSimplemotion.h does not compile (GCC-only variable length array initialisers), CMake turns it off
#else
	FAIL("SIMPLEMOTION is not defined (P15: MotorSimplemotion is in scope in this executable)");
#endif
#if defined(MIDI) || defined(RMDCAN)
	FAIL("a feature macro of a later phase (MIDI P17, RMDCAN P16) is defined");
#endif
}

}
