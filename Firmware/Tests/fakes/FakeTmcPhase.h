/*
 * FakeTmcPhase.h
 *
 * Recording double of the TMC4671 driver interface that MidiMain uses (ffb_usb_tests only, P17). The real driver
 * (FFBoard/UserExtensions/Src/TMC4671.cpp) is built into ffb_driver_tests only and is tested there; CLAUDE.md: a driver
 * that is not built into the executable under test is faked at its interface. MidiMain is the in-scope caller: the
 * tests check which calls it makes, with which arguments and in which order, never what the chip would do.
 *
 * The double is the class pair TMC4671 / TMC_1 of the shadow header platform/usb_midi/TMC4671.h (same include guard
 * TMC4671_H_ as the real header, which is therefore never read in this executable). This file holds the recording state.
 *
 * MODELS
 *  - Nothing of the chip. Every call MidiMain.cpp makes (setLimits, setEncoderType, setPhiEtype, setUdUq, setMotionMode,
 *    setPhiE_ext, initialize, hasPower, isSetUp, initializeWithPower) is appended to state().calls with up to two
 *    integer arguments, plus "construct" / "destruct" for TMC_1. Configurable answers: initialize() result, the motor type
 *    reported in conf.motconf.motor_type after construction (the real driver restores it from flash), hasPower() and
 *    isSetUp().
 *  - The field TMC4671::allowSlowSPI is a plain member (MidiMain assigns it; tests read it through MidiMain::drv).
 * SOURCE
 *  - The call sites in FFBoard/UserExtensions/Src/MidiMain.cpp:36-69 and 78-86 and the declarations in
 *    FFBoard/UserExtensions/Inc/TMC4671.h (enum values, struct TMC4671Limits, signatures).
 * DOES NOT MODEL
 *  - SPI traffic, registers, motion modes, limits, the thread, flash, anything about motor or encoder. A call outside
 *    the set above does not compile (strict by construction).
 * CONFIDENCE
 *  - High for the interface (copied from the header), Low for the semantics (there are none: it is a recorder).
 * Reset: reset() (MidiFixture calls it at the start and end of every test).
 */
#ifndef FAKES_FAKETMCPHASE_H_
#define FAKES_FAKETMCPHASE_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace FakeTmcPhase {

struct Call {
	std::string name;
	int64_t a = 0;
	int64_t b = 0;
};

struct State {
	std::vector<Call> calls;
	bool initializeResult = true; //!< what initialize() returns
	uint8_t motorType = 3;        //!< MotorType reported after construction (3 = BLDC; 0 = NONE makes MidiMain pulse the error LED)
	bool hasPower = false;        //!< what hasPower() returns
	bool isSetUp = false;         //!< what isSetUp() returns
	int constructed = 0;          //!< TMC_1 objects created since reset()
	int destroyed = 0;            //!< TMC_1 objects destroyed since reset()
};

State &state();
void reset();
void record(const char *name, int64_t a = 0, int64_t b = 0);
/** Number of recorded calls with this name */
size_t count(const std::string &name);
/** The names of all recorded calls in order */
std::vector<std::string> names();

} // namespace FakeTmcPhase

#endif /* FAKES_FAKETMCPHASE_H_ */
