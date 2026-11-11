/*
 * TMC4671.h (shadow header of ffb_usb_tests, P17)
 *
 * Replaces Firmware/FFBoard/UserExtensions/Inc/TMC4671.h in ffb_usb_tests only. It is force-included (-include, CMake) in every
 * C++ source of that executable and has the include guard of the real header, so the real header is skipped when a source
 * includes it (a quoted include of MidiMain.h would find the real file in its own directory before any include directory). MidiMain.h includes "TMC4671.h" and owns a std::unique_ptr<TMC4671> of a TMC_1; the real
 * header declares both only with TMC4671DRIVER, which stays off in this executable. The classes below are the recording
 * double of that interface (fakes/FakeTmcPhase.h: what it models, source, what it does not model, confidence).
 * The include guard is the one of the real header. Enum values and struct fields are copied from the real header
 * (UserExtensions/Inc/TMC4671.h:69-75, 80-90, 235-243); only what MidiMain uses exists.
 * Other firmware files that include "TMC4671.h" (Axis.cpp, MotorDriver.cpp, FFBHIDMain.h) use nothing of it without
 * TMC4671DRIVER.
 */
#ifndef TMC4671_H_
#define TMC4671_H_
#include "constants.h"
#include "../../fakes/FakeTmcPhase.h"

#include <cstdint>

enum class MotorType : uint8_t {NONE=0,DC=1,STEPPER=2,BLDC=3};
enum class PhiE : uint8_t {ext=1,openloop=2,abn=3,hall=5,aenc=6,aencE=7,NONE,extEncoder};
enum class MotionMode : uint8_t {stop=0,torque=1,velocity=2,position=3,prbsflux=4,prbstorque=5,prbsvelocity=6,uqudext=8,encminimove=9,NONE};
enum class EncoderType_TMC : uint8_t {NONE=0,abn=1,sincos=2,uvw=3,hall=4,ext=5}; // max 7

struct TMC4671MotConf{
	MotorType motor_type = MotorType::NONE;
};

struct TMC4671MainConfig{
	TMC4671MotConf motconf;
};

struct TMC4671Limits{
	uint16_t pid_torque_flux_ddt	= 32767;
	uint16_t pid_uq_ud				= 30000;
	uint16_t pid_torque_flux		= 30000;
	uint32_t pid_acc_lim			= 2147483647;
	uint32_t pid_vel_lim			= 2147483647;
	int32_t pid_pos_low				= -2147483647;
	int32_t pid_pos_high			= 2147483647;
};

class TMC4671 {
public:
	TMC4671() {
		conf.motconf.motor_type = static_cast<MotorType>(FakeTmcPhase::state().motorType);
		FakeTmcPhase::state().constructed++;
		FakeTmcPhase::record("construct");
	}
	virtual ~TMC4671() {
		FakeTmcPhase::state().destroyed++;
		FakeTmcPhase::record("destruct");
	}

	TMC4671MainConfig conf;
	bool allowSlowSPI = true; // as TMC4671.h:610

	void setLimits(TMC4671Limits limits) { FakeTmcPhase::record("setLimits", limits.pid_uq_ud, limits.pid_torque_flux); }
	void setEncoderType(EncoderType_TMC type) { FakeTmcPhase::record("setEncoderType", static_cast<int64_t>(type)); }
	void setPhiEtype(PhiE type) { FakeTmcPhase::record("setPhiEtype", static_cast<int64_t>(type)); }
	void setPhiE_ext(int16_t phiE) { FakeTmcPhase::record("setPhiE_ext", phiE); }
	void setUdUq(int16_t ud, int16_t uq) { FakeTmcPhase::record("setUdUq", ud, uq); }
	void setMotionMode(MotionMode mode, bool force = false) { FakeTmcPhase::record("setMotionMode", static_cast<int64_t>(mode), force); }
	bool initialize() { FakeTmcPhase::record("initialize"); return FakeTmcPhase::state().initializeResult; }
	void initializeWithPower() { FakeTmcPhase::record("initializeWithPower"); }
	bool hasPower() { FakeTmcPhase::record("hasPower"); return FakeTmcPhase::state().hasPower; }
	bool isSetUp() { FakeTmcPhase::record("isSetUp"); return FakeTmcPhase::state().isSetUp; }
};

class TMC_1 : public TMC4671 {
public:
	TMC_1() : TMC4671{} {}
};

#endif /* TMC4671_H_ */
