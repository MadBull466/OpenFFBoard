/*
 * PositionScript.cpp - see PositionScript.h
 */
#include "PositionScript.h"

#include "../../Code_Coverage/fakes/FakeTmc4671.h"

#include <cmath>
#include <limits>

namespace {
constexpr double kTwoPi = 6.283185307179586476925286766559;
}

PositionScript::~PositionScript() {
	if (destroyHook_) {
		destroyHook_();
	}
}

void PositionScript::setConstant(int32_t counts) {
	kind_ = Kind::Constant;
	constant_ = counts;
}

void PositionScript::setRamp(int32_t startCounts, double countsPerMs) {
	kind_ = Kind::Ramp;
	rampStart_ = startCounts;
	rampPerMs_ = countsPerMs;
}

void PositionScript::setSine(double amplitudeCounts, double periodMs, double phaseRad, double offsetCounts) {
	if (!(periodMs > 0.0)) {
		violate("PositionScript::setSine: period must be > 0 ms");
		periodMs = 1.0;
	}
	kind_ = Kind::Sine;
	sineAmp_ = amplitudeCounts;
	sinePeriodMs_ = periodMs;
	sinePhase_ = phaseRad;
	sineOffset_ = offsetCounts;
}

void PositionScript::setFunction(Function fn) {
	kind_ = Kind::Function;
	fn_ = std::move(fn);
}

void PositionScript::violate(const std::string &what) const {
	if (violations_.size() < 100) {
		violations_.push_back(what);
	}
}

int32_t PositionScript::toCounts(double v) const {
	if (std::isnan(v)) {
		violate("PositionScript: the source produced NaN");
		return 0;
	}
	// llround of a value beyond the long long range is unspecified: clamp in double first
	constexpr double lo = static_cast<double>(std::numeric_limits<int32_t>::min());
	constexpr double hi = static_cast<double>(std::numeric_limits<int32_t>::max());
	if (v < lo) {
		violate("PositionScript: the source is below the int32 range (value " + std::to_string(v) + "), saturated");
		return std::numeric_limits<int32_t>::min();
	}
	if (v > hi) {
		violate("PositionScript: the source is above the int32 range (value " + std::to_string(v) + "), saturated");
		return std::numeric_limits<int32_t>::max();
	}
	return static_cast<int32_t>(std::llround(v));
}

int32_t PositionScript::valueAt(uint64_t usSinceAttach) const {
	const double tMs = static_cast<double>(usSinceAttach) / 1000.0;
	switch (kind_) {
	case Kind::Constant:
		return constant_;
	case Kind::Ramp:
		return toCounts(static_cast<double>(rampStart_) + rampPerMs_ * tMs);
	case Kind::Sine:
		return toCounts(sineOffset_ + sineAmp_ * std::sin(kTwoPi * tMs / sinePeriodMs_ + sinePhase_));
	case Kind::Function:
		if (!fn_) {
			violate("PositionScript: setFunction with an empty function (nothing to evaluate), 0 written");
			return 0;
		}
		return fn_(usSinceAttach);
	}
	return 0;
}

void PositionScript::write(uint64_t usSinceAttach) {
	lastValue_ = valueAt(usSinceAttach);
	if (chip_ != nullptr) {
		chip_->setReg(kRegPositionActual, static_cast<uint32_t>(lastValue_));
	}
}

void PositionScript::attach(FakeTmc4671 &chip, uint64_t nowUs) {
	chip_ = &chip;
	attachUs_ = nowUs;
	steps_ = 0;
	write(0);
}

void PositionScript::detach() { chip_ = nullptr; }

void PositionScript::stepToUs(uint64_t nowUs) {
	if (chip_ == nullptr) {
		return;
	}
	steps_++;
	write(nowUs >= attachUs_ ? nowUs - attachUs_ : 0);
}
