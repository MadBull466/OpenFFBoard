/*
 * WheelPlant.h
 *
 * Wheel plant attached to the fake TMC4671 (ffb_integration_tests only; plan section 6.2). It reads, once per simulated step,
 * the torque the chip would produce from the chip's registers, integrates a wheel (rotor, shaft and rim) and writes the
 * resulting position to PID_POSITION_ACTUAL 0x6B, so the real firmware closes the loop exactly as on the board.
 *
 * Source: TMC4671-LA datasheet, IC version V1.3, document revision 2.07 (2022-01-12), cited "DS pNN" (page of the PDF,
 * printed in its header "NN / 153"); plan Tests/Integration_Tests/claude/plans/INTEGRATION_TEST_PLAN.md sections 6.2 and 6.3
 * (the plant model and the nominal parameter ranges, which are stated, not measured); audit group A
 * (charters/audit/group-a_faketmc4671.md "For the integration", G1 and G2).
 *
 * MODELS
 *  - Rigid wheel, one degree of freedom, SI units (theta in rad, omega in rad/s, torque in N m):
 *        J theta'' = tau - b theta' - F_c sign(theta')  [ - k (theta - h) - c theta' when the hand is enabled ]
 *    tau is the torque of the chip (below). b and J are constant. F_c is Coulomb friction. The optional hand is a spring k
 *    (N m/rad) and a damper c (N m s/rad) to a stationary hand position h (rad); set with setHand(), off by default.
 *  - Nominal parameter ranges (plan 6.3 step 1, "stated, not measured"): J 0.005 to 0.08 kg m^2 (total inertia), b 0.0005 to
 *    0.05 N m s/rad, F_c 0 to 0.2 N m, full-scale torque 5 to 25 N m for 32767 torque counts, current loop ideal or a lag of
 *    0.2 ms, encoder 65536 counts per revolution. Parameters outside the ranges are allowed (a test may need an extreme) and
 *    are reported by outsideNominal(); b = 0, F_c = 0 and k = 0 are valid (exact solutions below).
 *  - No default hides a choice: Params has no usable default (all NaN, setParams() records a violation for NaN, J <= 0 or full
 *    scale <= 0). A test sets every field, or takes one of the NAMED nominal sets of Params::nominalLight() / nominalMid() /
 *    nominalHeavy(), each the low end / geometric-style middle / high end of the plan 6.3 ranges (source: plan 6.3 step 1).
 *  - Integration per step dt (step()): the torque of the chip is held constant over the step (zero-order hold: the firmware
 *    writes 0x64 once per FFB cycle and the plant steps once per ms). Without Coulomb friction the step uses the EXACT solution
 *    of the linear ODE (closed form, b = 0 included; with a hand spring the under-, critically and over-damped closed forms), so
 *    energy and decay are exact up to floating point. Closed forms (x = theta - theta_eq, B = b + c, alpha = B / (2 J)):
 *       k = 0, B = 0:  omega = omega0 + a t,  theta = theta0 + omega0 t + a t^2 / 2,  a = T / J
 *       k = 0, B > 0:  omega = w_ss + (omega0 - w_ss) e^{-B t / J},  theta = theta0 + w_ss t + (omega0 - w_ss) (J / B) (1 - e^{-B t / J}),
 *                      w_ss = T / B   (T = held external torque)
 *       k > 0:         theta_eq = (T + k h) / k, w0^2 = k / J; underdamped x = e^{-alpha t}(x0 cos wd t + (v0 + alpha x0) / wd sin wd t),
 *                      critical x = e^{-alpha t}(x0 + (v0 + alpha x0) t), overdamped x = A e^{r1 t} + B e^{r2 t}
 *                      (critical when |alpha^2 - w0^2| <= 1e-9 w0^2).
 *  - Coulomb friction (documented stick / slip rule): the step is split into segments. At omega = 0 the wheel sticks while
 *    |T_static| <= F_c, T_static = tau + k (h - theta) (the force it would feel at rest; equality sticks) and then does not move
 *    for the rest of the step; otherwise it slips in the direction of T_static. While moving, the friction torque
 *    -F_c sign(omega) is constant over the segment (added to the held torque, then the linear closed form is used). If omega
 *    would reach or pass zero inside the segment the segment is cut at the crossing time (closed form for k = 0, bisection
 *    for k > 0; omega is set to exactly 0 there), so friction alone can never reverse the motion. After the cut the rule for
 *    omega = 0 applies to the rest of the step (stick, or slip towards the other side if the spring / torque is stronger).
 *  - Effective torque (audit A, G1; effectiveTorque()): tau is NOT PID_TORQUE_FLUX_TARGET at face value. Each step it is read from
 *    the chip: torque target = 0x64 bits [31:16], s16 (DS p98); it is applied only when ALL of these hold, else tau = 0:
 *      motion mode 0x63 bits [7:0] (MODE_MOTION) == 1 "torque_mode" (DS p97-98),
 *      PWM mode 0x1A bits [7:0] (PWM_CHOP of PWM_SV_CHOP) == 7 "centered PWM for FOC" (DS p84-85),
 *      ENABLE_IN: 0x76 bit 15 (ENABLE_IN_RAW, DS p104) high: "when ENI is low, all controllers are deactivated" (DS p52, 4.7.1).
 *    The target is clamped symmetrically to +-limit, limit = 0x5E bits [15:0] (PID_TORQUE_FLUX_LIMITS, u16, "limits the target
 *    values coming from the target registers", DS p97; the DS gives no sign convention for the lower bound: symmetric is an
 *    ASSUMPTION). Counts -> N m: tau = counts * fullScale / 32767 (plan 6.3; a target of -32768 with limit 0xFFFF gives a
 *    magnitude of 32768 / 32767 of full scale, not clamped to it).
 *  - Current-loop lag (Params::currentLagS, default 0 = ideal): the applied torque follows the commanded one as a first-order
 *    lag with time constant Tl. Per step the exact filter state is advanced and the MEAN of the lag output over the step is
 *    used as the held torque: tau_mean = u + (y0 - u) (Tl / dt) (1 - e^{-dt/Tl}) (u = commanded torque, y0 = state at the step
 *    start), so the impulse of the torque over the step is exact; the torque shape within the step is replaced by its mean.
 *  - Position out (audit A, G2; DS p98-99: PID_POSITION_ACTUAL s32, "Input position differences are accumulated. Lower 16 bits
 *    display one revolution", i.e. 65536 counts per mechanical revolution; the bit layout is the DS text of 0x6B): the plant
 *    keeps its own continuous angle. Each step it reads the CURRENT value of 0x6B from the chip and adds the change of the
 *    angle in whole counts since the last step (the fractional remainder is carried to the next step; wrap-around of the 32 bit
 *    register), so a firmware write to 0x6B (zeroing, Axis / TMC4671::setTmcPos) rebases the count and is not overwritten.
 *    The fake's mirror of a write into 0x68 stays the fake's business; the plant writes with setReg (no mirror, no log).
 *    No onWrite(0x6B) hook is installed: reading the register each step needs none and does not take the single hook slot of
 *    the address from other users (lead decision 2026-11-18). COST: a firmware write to 0x6B is rebased only at the next plant
 *    step, so the motion of up to one step (about 1 ms, the plant runs last in the scheduler's millisecond) that happened before
 *    the write is added after it; with the sequential scheduler this is at most one step of motion.
 *  - 65536 counts per revolution holds only when the position source is a MECHANICAL angle: POSITION_SELECTION 0x51 [7:0] =
 *    9 phi_m_abn, 10 phi_m_abn_2 or 11 phi_m_aenc (DS p95; 0, 1, 2, 3, 5, 6, 7 select electrical angles, 4 and 8 are reserved).
 *    Each step the plant reads 0x51; any other value records ONE plant violation per distinct value (firstFault() of the fixture
 *    then fails the case), unless setRequireMechanicalPositionSource(false) was called.
 *  - Registers other than 0x6B are only READ. The firmware's F1 path reads no velocity (TMC4671::getVelocity 0x6A has no caller
 *    in Axis.cpp) and torque actual 0x69 is not modelled.
 *  - step() takes dt in seconds; stepToUs(nowUs) takes the difference of simulated time since the last call (ChainScheduler step
 *    (f) calls it once per scheduler step, about 1 ms; frames that take simulated time make a step slightly longer).
 *
 * DOES NOT MODEL
 *  Electrical dynamics other than the optional first-order lag, FOC torque ripple / cogging, backlash, rim compliance, encoder
 *  noise (only the count quantisation), SPI and preemption latency, velocity / torque feedback registers, a moving hand, the
 *  chip's torque limit in flux, real motor constant (the full-scale torque is a stated parameter), torque produced by the chip's own
 *  velocity / position cascades outside torque mode (the plant gives tau = 0 there), 0x65 PID_TORQUE_OFFSET (feed-forward, no caller
 *  on the F1 path). Several zero crossings of
 *  omega inside ONE step with a hand spring (a spring period shorter than about 4 steps) are not resolved: dt must be well
 *  below a quarter of the shortest period.
 *
 * CONFIDENCE
 *  Integration of the stated model: High (closed forms; checked by the self-tests of F1-A3). Register gating and 0x6B
 *  accumulation: Medium-High (datasheet text; symmetric clamp is an assumption). Match with a real wheel: Low (nominal parameters).
 */
#ifndef HOSTTEST_INTEGRATION_WHEELPLANT_H_
#define HOSTTEST_INTEGRATION_WHEELPLANT_H_

#include "../../Code_Coverage/fakes/FakeTmc4671.h" // fakes/ is not an include directory (fakes/Axis.h would shadow the real Axis.h)

#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

class WheelPlant {
public:
	static constexpr double kPi = 3.14159265358979323846;
	static constexpr double kCountsPerRev = 65536.0;	 //!< DS p99
	static constexpr double kCountsPerRad = kCountsPerRev / (2.0 * kPi);
	static constexpr double kFullScaleCounts = 32767.0; //!< torque counts that give Params::fullScaleTorque
	static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

	struct Params {
		double inertia = kNaN;		  //!< J, kg m^2 (nominal 0.005 .. 0.08)
		double viscous = kNaN;		  //!< b, N m s/rad (nominal 0.0005 .. 0.05)
		double coulomb = kNaN;		  //!< F_c, N m (nominal 0 .. 0.2)
		double fullScaleTorque = kNaN; //!< N m for 32767 torque counts (nominal 5 .. 25)
		double currentLagS = 0.0;	  //!< first-order current-loop lag, s (0 = ideal; nominal 0 or 0.0002)

		/** Low end of every nominal range of plan 6.3 (light wheel, little friction, small motor) */
		static Params nominalLight();
		/** Middle of the nominal ranges (plan 6.3): J 0.02, b 0.005, F_c 0.05, full scale 15 N m, lag 0 */
		static Params nominalMid();
		/** High end of every nominal range (heavy rim, much friction, strong motor) */
		static Params nominalHeavy();
	};

	WheelPlant() = default;
	explicit WheelPlant(const Params &p) { setParams(p); }
	WheelPlant(const WheelPlant &) = delete;
	WheelPlant &operator=(const WheelPlant &) = delete;
	/** Calls the destroy hook (the fixture's registration, so it never keeps a pointer to a destroyed plant) */
	~WheelPlant() {
		if (onDestroy_) {
			onDestroy_();
		}
	}
	/** Registered by IntegrationFixture::attachPlant: runs in the destructor. nullptr clears it */
	void setDestroyHook(std::function<void()> fn) { onDestroy_ = std::move(fn); }
	/** Default true: record a violation when 0x51 does not select a mechanical angle (see the file comment) */
	void setRequireMechanicalPositionSource(bool on) { requireMechanical_ = on; }

	/** Sets the physical parameters (records a violation for NaN, J <= 0, negative friction / damping, full scale <= 0, lag < 0) */
	void setParams(const Params &p);
	const Params &params() const { return params_; }
	/** True if a parameter is outside the nominal range of plan 6.3 (informational) */
	bool outsideNominal() const;
	/** Enables the hands-on spring k and damper c towards the fixed hand position h (rad) */
	void setHand(double k, double c, double handPosRad);
	void clearHand() { handEnabled_ = false; }
	bool handEnabled() const { return handEnabled_; }

	/** Attaches to a chip (not owned; the chip must outlive the attachment). Does not change the state */
	void attach(FakeTmc4671 &chip, uint64_t nowUs = 0);
	void detach() { chip_ = nullptr; }
	bool attached() const { return chip_ != nullptr; }
	FakeTmc4671 *chip() const { return chip_; }

	/** Sets the mechanical state (no register write); clears the lag state and the work / step counters, keeps the parameters */
	void setState(double thetaRad, double omegaRadS);
	/** Same, and also forgets the fractional count remainder */
	void reset();

	/**
	 * The torque of the chip that reaches the wheel now, from the registers of the attached chip (see the file comment), N m,
	 * before the current-loop lag. 0 when not attached or a gating condition is off
	 */
	double effectiveTorque() const;
	/** Pure function of the registers: the counts that reach the wheel (0 if gated off). Static for the self-tests */
	static int32_t effectiveTorqueCounts(uint32_t reg64, uint32_t reg63, uint32_t reg1A, uint32_t reg76, uint32_t reg5E);

	/** One step of dt seconds: read the chip, integrate, write 0x6B. Not attached: integrates with zero chip torque */
	void step(double dt);
	/** Step to the simulated time nowUs (microseconds); a call with the same time does nothing. Used by ChainScheduler step (f) */
	void stepToUs(uint64_t nowUs);

	// --- observers ---
	double theta() const { return theta_; }		   //!< rad, continuous (not wrapped)
	double omega() const { return omega_; }		   //!< rad/s
	double tauApplied() const { return tauApplied_; } //!< N m held over the last step (chip torque after the lag, no friction / spring)
	double tauCommanded() const { return tauCommanded_; } //!< N m of effectiveTorque() at the last step (before the lag)
	double kineticEnergy() const;					  //!< 0.5 J omega^2, J
	double springEnergy() const;					  //!< 0.5 k (theta - h)^2 with the hand, else 0
	double energy() const { return kineticEnergy() + springEnergy(); }
	double chipWork() const { return chipWork_; }	  //!< integral of the chip torque over the angle (sum of tau_held * delta theta), J
	uint64_t steps() const { return steps_; }		  //!< steps taken since attach() / setState()
	double elapsedS() const { return elapsedS_; }	  //!< simulated seconds integrated since attach() / setState()
	/** The 32 bit count the plant last wrote to 0x6B (as the register value) */
	uint32_t lastPositionRegister() const { return lastReg6B_; }
	/** theta expressed in counts (not wrapped, not quantised) */
	double thetaCounts() const { return theta_ * kCountsPerRad; }
	/** Strictness: bad parameters, step with dt <= 0 or NaN. Empty if none */
	const std::vector<std::string> &violations() const { return violations_; }

private:
	struct State {
		double theta;
		double omega;
	};
	/** Exact linear step of dt: J x'' + B x' + k x = T (+ k h), held torque T, no Coulomb term */
	State linearStep(State s, double dt, double T) const;
	double springForce(double theta) const { return handEnabled_ ? handK_ * (handPos_ - theta) : 0.0; }
	double dampCoeff() const { return params_.viscous + (handEnabled_ ? handC_ : 0.0); }
	/** First time in (0, dt] at which omega of a linear segment (sign s friction included in T) reaches 0; dt + 1 if none */
	double zeroCrossing(State s, double dt, double T) const;
	void integrate(double dt, double tauHeld);
	void violate(const std::string &text) { violations_.push_back(text); }

	Params params_;
	bool paramsOk_ = false;
	bool handEnabled_ = false;
	double handK_ = 0.0;
	double handC_ = 0.0;
	double handPos_ = 0.0;
	FakeTmc4671 *chip_ = nullptr;
	std::function<void()> onDestroy_;
	bool requireMechanical_ = true;
	std::vector<uint32_t> badPosSel_;

	double theta_ = 0.0;
	double omega_ = 0.0;
	double lagState_ = 0.0;
	bool lagPrimed_ = false;
	double tauApplied_ = 0.0;
	double tauCommanded_ = 0.0;
	double chipWork_ = 0.0;
	uint64_t steps_ = 0;
	double elapsedS_ = 0.0;
	uint64_t lastUs_ = 0;

	double countAccumulator_ = 0.0; //!< whole-count carry: counts of theta not yet written
	double lastThetaCounts_ = 0.0;
	uint32_t lastReg6B_ = 0;

	std::vector<std::string> violations_;
};

#endif
