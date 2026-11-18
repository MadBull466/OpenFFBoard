/*
 * WheelPlant.cpp: see WheelPlant.h for the model, the sources and the limits.
 */
#include "WheelPlant.h"

#include <algorithm>
#include <cmath>

constexpr double WheelPlant::kPi;
constexpr double WheelPlant::kCountsPerRev;
constexpr double WheelPlant::kCountsPerRad;
constexpr double WheelPlant::kFullScaleCounts;
constexpr double WheelPlant::kNaN;

namespace {
double sgn(double v) { return v > 0.0 ? 1.0 : (v < 0.0 ? -1.0 : 0.0); }
} // namespace

WheelPlant::Params WheelPlant::Params::nominalLight() {
	Params p;
	p.inertia = 0.005;
	p.viscous = 0.0005;
	p.coulomb = 0.0;
	p.fullScaleTorque = 5.0;
	p.currentLagS = 0.0;
	return p;
}

WheelPlant::Params WheelPlant::Params::nominalMid() {
	Params p;
	p.inertia = 0.02;
	p.viscous = 0.005;
	p.coulomb = 0.05;
	p.fullScaleTorque = 15.0;
	p.currentLagS = 0.0;
	return p;
}

WheelPlant::Params WheelPlant::Params::nominalHeavy() {
	Params p;
	p.inertia = 0.08;
	p.viscous = 0.05;
	p.coulomb = 0.2;
	p.fullScaleTorque = 25.0;
	p.currentLagS = 0.0;
	return p;
}

void WheelPlant::setParams(const Params &p) {
	params_ = p;
	paramsOk_ = true;
	if (!std::isfinite(p.inertia) || p.inertia <= 0.0) {
		violate("WheelPlant: inertia must be finite and > 0");
		paramsOk_ = false;
	}
	if (!std::isfinite(p.viscous) || p.viscous < 0.0) {
		violate("WheelPlant: viscous damping must be finite and >= 0");
		paramsOk_ = false;
	}
	if (!std::isfinite(p.coulomb) || p.coulomb < 0.0) {
		violate("WheelPlant: Coulomb friction must be finite and >= 0");
		paramsOk_ = false;
	}
	if (!std::isfinite(p.fullScaleTorque) || p.fullScaleTorque <= 0.0) {
		violate("WheelPlant: full-scale torque must be finite and > 0");
		paramsOk_ = false;
	}
	if (!std::isfinite(p.currentLagS) || p.currentLagS < 0.0) {
		violate("WheelPlant: current-loop lag must be finite and >= 0");
		paramsOk_ = false;
	}
}

bool WheelPlant::outsideNominal() const {
	auto out = [](double v, double lo, double hi) { return v < lo || v > hi; };
	return out(params_.inertia, 0.005, 0.08) || out(params_.viscous, 0.0005, 0.05) || out(params_.coulomb, 0.0, 0.2) ||
		   out(params_.fullScaleTorque, 5.0, 25.0) || out(params_.currentLagS, 0.0, 0.0002);
}

void WheelPlant::setHand(double k, double c, double handPosRad) {
	if (!std::isfinite(k) || k < 0.0 || !std::isfinite(c) || c < 0.0 || !std::isfinite(handPosRad)) {
		violate("WheelPlant: hand spring and damper must be finite and >= 0, hand position finite");
		return;
	}
	handEnabled_ = true;
	handK_ = k;
	handC_ = c;
	handPos_ = handPosRad;
}

void WheelPlant::attach(FakeTmc4671 &chip, uint64_t nowUs) {
	chip_ = &chip;
	lastUs_ = nowUs;
	steps_ = 0;
	elapsedS_ = 0.0;
	chipWork_ = 0.0;
	lastThetaCounts_ = thetaCounts();
	countAccumulator_ = 0.0;
	lastReg6B_ = chip.reg(0x6B);
}

void WheelPlant::setState(double thetaRad, double omegaRadS) {
	theta_ = thetaRad;
	omega_ = omegaRadS;
	lagState_ = 0.0;
	lagPrimed_ = false;
	tauApplied_ = 0.0;
	tauCommanded_ = 0.0;
	chipWork_ = 0.0;
	steps_ = 0;
	elapsedS_ = 0.0;
	lastThetaCounts_ = thetaCounts(); // no jump is written to 0x6B
}

void WheelPlant::reset() {
	setState(0.0, 0.0);
	countAccumulator_ = 0.0;
}

double WheelPlant::kineticEnergy() const { return 0.5 * params_.inertia * omega_ * omega_; }

double WheelPlant::springEnergy() const {
	if (!handEnabled_) {
		return 0.0;
	}
	const double d = theta_ - handPos_;
	return 0.5 * handK_ * d * d;
}

int32_t WheelPlant::effectiveTorqueCounts(uint32_t reg64, uint32_t reg63, uint32_t reg1A, uint32_t reg76, uint32_t reg5E) {
	const bool torqueMode = (reg63 & 0xFFu) == 1u;	 // MODE_MOTION 1 = torque_mode, DS p97-98
	const bool focPwm = (reg1A & 0xFFu) == 7u;		 // PWM_CHOP 7 = centered PWM for FOC, DS p84-85
	const bool eniHigh = (reg76 & 0x8000u) != 0;	 // ENABLE_IN_RAW bit 15, DS p104; ENI low deactivates the controllers, DS p52
	if (!torqueMode || !focPwm || !eniHigh) {
		return 0;
	}
	const int32_t target = static_cast<int16_t>((reg64 >> 16) & 0xFFFFu); // PID_TORQUE_TARGET s16(31:16), DS p98
	const int32_t limit = static_cast<int32_t>(reg5E & 0xFFFFu);			// PID_TORQUE_FLUX_LIMITS u16(15:0), DS p97
	return std::max(-limit, std::min(limit, target));
}

double WheelPlant::effectiveTorque() const {
	if (chip_ == nullptr || !paramsOk_) {
		return 0.0;
	}
	const int32_t counts = effectiveTorqueCounts(chip_->reg(0x64), chip_->reg(0x63), chip_->reg(0x1A), chip_->reg(0x76), chip_->reg(0x5E));
	return static_cast<double>(counts) * params_.fullScaleTorque / kFullScaleCounts;
}

WheelPlant::State WheelPlant::linearStep(State s, double dt, double T) const {
	const double J = params_.inertia;
	const double B = dampCoeff();
	const double k = handEnabled_ ? handK_ : 0.0;
	State r{};
	if (k <= 0.0) {
		if (B <= 0.0) {
			const double a = T / J;
			r.omega = s.omega + a * dt;
			r.theta = s.theta + s.omega * dt + 0.5 * a * dt * dt;
		} else {
			const double x = B * dt / J;
			const double decay = std::exp(-x);
			const double oneMinus = -std::expm1(-x);
			const double wss = T / B;
			r.omega = wss + (s.omega - wss) * decay;
			r.theta = s.theta + wss * dt + (s.omega - wss) * (J / B) * oneMinus;
		}
		return r;
	}
	const double thetaEq = (T + k * handPos_) / k;
	const double x0 = s.theta - thetaEq;
	const double v0 = s.omega;
	const double alpha = B / (2.0 * J);
	const double w02 = k / J;
	const double disc = alpha * alpha - w02;
	double x = 0.0;
	double v = 0.0;
	if (std::fabs(disc) <= 1e-9 * w02) { // critically damped
		const double C = v0 + alpha * x0;
		const double e = std::exp(-alpha * dt);
		x = e * (x0 + C * dt);
		v = e * (v0 - alpha * C * dt);
	} else if (disc < 0.0) { // underdamped
		const double wd = std::sqrt(-disc);
		const double e = std::exp(-alpha * dt);
		const double c = std::cos(wd * dt);
		const double sn = std::sin(wd * dt);
		x = e * (x0 * c + ((v0 + alpha * x0) / wd) * sn);
		v = e * (v0 * c - ((alpha * v0 + w02 * x0) / wd) * sn);
	} else { // overdamped: roots -alpha +- sq
		const double sq = std::sqrt(disc);
		const double r1 = -alpha + sq;
		const double r2 = -alpha - sq;
		const double Bc = (v0 - r1 * x0) / (r2 - r1);
		const double Ac = x0 - Bc;
		const double e1 = std::exp(r1 * dt);
		const double e2 = std::exp(r2 * dt);
		x = Ac * e1 + Bc * e2;
		v = Ac * r1 * e1 + Bc * r2 * e2;
	}
	r.theta = thetaEq + x;
	r.omega = v;
	return r;
}

double WheelPlant::zeroCrossing(State s, double dt, double T) const {
	const double dir = sgn(s.omega);
	const double J = params_.inertia;
	const double B = dampCoeff();
	const double k = handEnabled_ ? handK_ : 0.0;
	if (k <= 0.0) {
		if (B <= 0.0) {
			const double a = T / J;
			if (a * s.omega >= 0.0) {
				return dt + 1.0;
			}
			const double tc = -s.omega / a;
			return tc <= dt ? tc : dt + 1.0;
		}
		const double wss = T / B;
		if (wss * s.omega >= 0.0) { // the steady-state velocity is on the same side (or zero): never reaches 0 in finite time
			return dt + 1.0;
		}
		const double tc = -(J / B) * std::log(wss / (wss - s.omega)); // omega(tc) = 0
		return tc <= dt ? tc : dt + 1.0;
	}
	if (linearStep(s, dt, T).omega * dir > 0.0) {
		return dt + 1.0; // one crossing at most per step (dt well below the spring period, see the header)
	}
	double lo = 0.0;
	double hi = dt;
	for (int i = 0; i < 200; i++) {
		const double mid = 0.5 * (lo + hi);
		if (linearStep(s, mid, T).omega * dir > 0.0) {
			lo = mid;
		} else {
			hi = mid;
		}
	}
	return hi;
}

void WheelPlant::integrate(double dt, double tauHeld) {
	const double Fc = params_.coulomb;
	const double thetaStart = theta_;
	State s{theta_, omega_};
	if (Fc <= 0.0) {
		s = linearStep(s, dt, tauHeld);
	} else {
		double rem = dt;
		for (int guard = 0; guard < 16 && rem > 0.0; guard++) {
			if (s.omega == 0.0) {
				const double Ts = tauHeld + springForce(s.theta);
				if (std::fabs(Ts) <= Fc) {
					break; // sticks for the rest of the step
				}
				const double dir = sgn(Ts);
				s = linearStep(s, rem, tauHeld - dir * Fc);
				rem = 0.0;
			} else {
				const double dir = sgn(s.omega);
				const double T = tauHeld - dir * Fc;
				const double tc = zeroCrossing(s, rem, T);
				if (tc > rem) {
					s = linearStep(s, rem, T);
					rem = 0.0;
				} else {
					s = linearStep(s, tc, T);
					s.omega = 0.0; // exactly at rest at the crossing
					rem -= tc;
				}
			}
		}
	}
	theta_ = s.theta;
	omega_ = s.omega;
	chipWork_ += tauHeld * (theta_ - thetaStart);
}

void WheelPlant::step(double dt) {
	if (!(dt > 0.0) || !std::isfinite(dt)) {
		violate("WheelPlant: step with dt <= 0 or not finite");
		return;
	}
	if (!paramsOk_) {
		violate("WheelPlant: step before valid parameters were set");
		return;
	}
	const double u = effectiveTorque();
	tauCommanded_ = u;
	double held = u;
	if (params_.currentLagS > 0.0) {
		const double Tl = params_.currentLagS;
		const double y0 = lagState_;
		const double e = std::exp(-dt / Tl);
		held = u + (y0 - u) * (Tl / dt) * (-std::expm1(-dt / Tl));
		lagState_ = u + (y0 - u) * e;
	}
	tauApplied_ = held;
	integrate(dt, held);
	steps_++;
	elapsedS_ += dt;
	if (chip_ != nullptr) {
		const uint32_t psel = chip_->reg(0x51) & 0xFFu; // POSITION_SELECTION, DS p95: 9, 10, 11 are mechanical angles
		if (requireMechanical_ && psel != 9u && psel != 10u && psel != 11u &&
			std::find(badPosSel_.begin(), badPosSel_.end(), psel) == badPosSel_.end()) {
			badPosSel_.push_back(psel);
			violate("WheelPlant: POSITION_SELECTION 0x51 = " + std::to_string(psel) + " is not a mechanical angle (9, 10, 11): 0x6B is not 65536 counts per revolution");
		}
		// PID_POSITION_ACTUAL accumulates the change of the angle in whole counts, the remainder is carried (DS p99)
		const double now = thetaCounts();
		countAccumulator_ += now - lastThetaCounts_;
		lastThetaCounts_ = now;
		const double whole = std::trunc(countAccumulator_);
		countAccumulator_ -= whole;
		const int64_t w = static_cast<int64_t>(whole);
		const uint32_t reg = chip_->reg(0x6B) + static_cast<uint32_t>(static_cast<uint64_t>(w)); // 32 bit wrap-around
		chip_->setReg(0x6B, reg);
		lastReg6B_ = reg;
	}
}

void WheelPlant::stepToUs(uint64_t nowUs) {
	if (nowUs <= lastUs_) {
		return;
	}
	const double dt = static_cast<double>(nowUs - lastUs_) * 1e-6;
	lastUs_ = nowUs;
	step(dt);
}
