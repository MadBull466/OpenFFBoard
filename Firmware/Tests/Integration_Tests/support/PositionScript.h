/*
 * PositionScript.h
 *
 * A scripted source for the position register PID_POSITION_ACTUAL (0x6B) of the fake TMC4671, for the open-loop cases of
 * ffb_integration_tests (WO I1-1; plan section 6). It stands in for the motor and the encoder: the test decides what position the
 * chip reports, no torque is integrated (the closed loop is WheelPlant, phase I4). It never reads the torque register.
 *
 * Register (oracle): TMC4671-LA datasheet Rev 2.07 (Tests/Code_Coverage/shared/specs/TMC4671-LA_datasheet_Rev2.07.pdf), register
 * 0x6B PID_POSITION_ACTUAL, DS p98-99 (the same page the WheelPlant header cites): signed 32 bit, "Input position differences are
 * accumulated. Lower 16 bits display one revolution", so 65536 counts are one mechanical revolution (kCountsPerRev). The value
 * written here is that signed 32 bit count; it is written with FakeTmc4671::setReg (no mirror into 0x68, no log entry, like the
 * plant), so the firmware's own reads (TMC4671::getPos, one frame per FFB cycle) are the only frames the log shows.
 *
 * ORDER (the contract the cases rely on, proven by a self-test in test_f1_support.cpp): the script runs in scheduler step (f), the
 * LAST action of a step before the observer hook (ChainScheduler.h). It writes 0x6B at the end of step k; the FFB cycle of step k
 * ran earlier in that step (step (c)) and read the old value, so a change written in step k is first seen by the firmware in the
 * FFB cycle of step k + 1.
 *
 * Time argument: the script's time is the simulated time since attach, in microseconds (the plant's clock is the same
 * HostPlatform::nowUs). The value written in a step is value(t) with t = nowUs of that step minus the time of attach. attach()
 * itself writes value(0) at once, so the register is defined before the first step.
 *
 * Sources: constant, ramp (start counts + rate in counts per ms), sine (offset + amplitude * sin(2 pi t / period + phase)), any
 * std::function<int32_t(uint64_t us)>. Changing the source while attached takes effect at the next step.
 *
 * Float to integer (flagged for the F1-B2 audit): a double value is rounded with std::llround and then clamped to the int32 range
 * (a value outside that range saturates, it does not wrap) and a violation is recorded (strict: a test that needs the range edge uses
 * exactly INT32_MIN / INT32_MAX); NaN is a violation and writes 0; an empty std::function is a violation and writes 0. The chip's register
 * has no saturation (it wraps at 32 bit, DS p98). Violations are part of IntegrationFixture::firstFault().
 *
 * Lifetime: IntegrationFixture::attachPositionScript registers a destroy hook, so a script that is a local of the test body may be
 * destroyed before the fixture without leaving a dangling pointer (same rule as WheelPlant).
 *
 * Exclusivity: the scheduler has one step-(f) slot; attaching a script while a plant is attached, or the reverse, is a fixture fault
 * reported by IntegrationFixture::firstFault() (the second attach is refused).
 */
#ifndef HOSTTEST_INTEGRATION_POSITIONSCRIPT_H_
#define HOSTTEST_INTEGRATION_POSITIONSCRIPT_H_

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

class FakeTmc4671;

class PositionScript {
public:
	static constexpr uint8_t kRegPositionActual = 0x6B;	//!< PID_POSITION_ACTUAL, DS p98-99
	static constexpr int32_t kCountsPerRev = 65536;		//!< DS p98-99: lower 16 bits = one revolution
	using Function = std::function<int32_t(uint64_t usSinceAttach)>;

	PositionScript() = default;
	~PositionScript();
	PositionScript(const PositionScript &) = delete;
	PositionScript &operator=(const PositionScript &) = delete;

	// --- sources (the last one set is in use) ---
	void setConstant(int32_t counts);
	/** value(t) = startCounts + countsPerMs * t[ms] (rounded) */
	void setRamp(int32_t startCounts, double countsPerMs);
	/** value(t) = offsetCounts + amplitudeCounts * sin(2 pi t / periodMs + phaseRad), t in ms; periodMs must be > 0 */
	void setSine(double amplitudeCounts, double periodMs, double phaseRad = 0.0, double offsetCounts = 0.0);
	void setFunction(Function fn);

	/** The scripted value at `usSinceAttach` (pure: no chip access) */
	int32_t valueAt(uint64_t usSinceAttach) const;

	// --- binding (done by IntegrationFixture::attachPositionScript) ---
	void attach(FakeTmc4671 &chip, uint64_t nowUs);
	void detach();
	bool attached() const { return chip_ != nullptr; }
	/** Called once per scheduler step with HostPlatform::nowUs(): writes value(nowUs - attach time) to 0x6B */
	void stepToUs(uint64_t nowUs);

	/** Called by the destructor; the fixture uses it to forget the script */
	void setDestroyHook(std::function<void()> hook) { destroyHook_ = std::move(hook); }

	// --- records ---
	uint32_t steps() const { return steps_; }			   //!< stepToUs calls since attach
	int32_t lastValue() const { return lastValue_; }	   //!< last value written (the attach write included)
	const std::vector<std::string> &violations() const { return violations_; }

private:
	enum class Kind { Constant, Ramp, Sine, Function };
	int32_t toCounts(double v) const;
	void violate(const std::string &what) const;
	void write(uint64_t usSinceAttach);

	Kind kind_ = Kind::Constant;
	int32_t constant_ = 0;
	int32_t rampStart_ = 0;
	double rampPerMs_ = 0.0;
	double sineAmp_ = 0.0, sinePeriodMs_ = 1.0, sinePhase_ = 0.0, sineOffset_ = 0.0;
	Function fn_;

	FakeTmc4671 *chip_ = nullptr;
	uint64_t attachUs_ = 0;
	uint32_t steps_ = 0;
	int32_t lastValue_ = 0;
	mutable std::vector<std::string> violations_;
	std::function<void()> destroyHook_;
};

#endif
