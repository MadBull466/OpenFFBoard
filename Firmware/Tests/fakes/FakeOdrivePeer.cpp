/*
 * FakeOdrivePeer.cpp - see FakeOdrivePeer.h for what is modelled, the sources and the confidence.
 */
#include "FakeOdrivePeer.h"

#include "HostPlatform.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <sstream>

namespace {
constexpr uint64_t NEVER = std::numeric_limits<uint64_t>::max();

uint32_t tickMs() { return static_cast<uint32_t>(HostPlatform::nowUs() / 1000); }

void putU32(std::array<uint8_t, 8> &d, int word, uint32_t v) {
	for (int i = 0; i < 4; i++) {
		d[word * 4 + i] = static_cast<uint8_t>(v >> (8 * i));
	}
}
void putF32(std::array<uint8_t, 8> &d, int word, float f) {
	uint32_t v;
	std::memcpy(&v, &f, sizeof(v));
	putU32(d, word, v);
}
} // namespace

bool FakeOdrivePeer::isCalibrationState(uint32_t s) {
	return s == STATE_FULL_CALIBRATION || s == STATE_MOTOR_CALIBRATION || s == STATE_ENCODER_INDEX_SEARCH ||
		   s == STATE_ENCODER_OFFSET_CALIBRATION || s == STATE_HALL_PHASE_CALIBRATION;
}

uint32_t FakeOdrivePeer::Frame::u32(int word) const {
	uint32_t v = 0;
	for (int i = 3; i >= 0; i--) {
		v = (v << 8) | data[word * 4 + i];
	}
	return v;
}
float FakeOdrivePeer::Frame::f32(int word) const {
	uint32_t v = u32(word);
	float f;
	std::memcpy(&f, &v, sizeof(f));
	return f;
}

std::string FakeOdrivePeer::Violation::text() const {
	std::ostringstream s;
	switch (kind) {
	case ViolationKind::ExtendedId: s << "extended id frame for this node"; break;
	case ViolationKind::RtrMismatch: s << "RTR bit does not match the message type (SPEC call and response)"; break;
	case ViolationKind::WrongDlc: s << "DLC differs from the CSV"; break;
	case ViolationKind::UnknownCommand: s << "command id the driver does not use"; break;
	}
	s << " (cmd 0x" << std::hex << cmd << ", id 0x" << id << std::dec << ", dlc " << int(dlc) << ", t=" << timeMs << " ms)";
	return s.str();
}

FakeOdrivePeer::FakeOdrivePeer(uint8_t nodeId) : node_(nodeId & 0x3F) {
	reset();
}

void FakeOdrivePeer::reset() {
	state_ = STATE_IDLE;
	axisError_ = 0;
	motorFlag_ = encoderFlag_ = controllerFlag_ = trajDone_ = false;
	applyDelayMs_ = 0;
	calibrationMs_ = 2000;
	refuseAll_ = false;
	refusalError_ = 0;
	refuseStates_.clear();
	pendingValid_ = false;
	pos_ = vel_ = ibus_ = 0;
	vbus_ = 24.0f;
	hbEnabled_ = true;
	hbPeriodMs_ = 100;
	hbNextUs_ = HostPlatform::nowUs() + 100000;
	encEnabled_ = false;
	encPeriodMs_ = 10;
	encNextUs_ = 0;
	skippedCyclic_ = 0;
	clearFaults();
	out_.clear();
	log_.clear();
	sent_.clear();
	violations_.clear();
	undocumented_.clear();
	expected_.clear();
	requested_.clear();
	refused_.clear();
	torqueSeen_ = modesSeen_ = false;
	lastTorque_ = 0;
	lastControl_ = lastInput_ = 0;
	anticogging_ = foreign_ = 0;
	syncCursor();
}

void FakeOdrivePeer::syncCursor() {
	cursor_ = HostTarget::canTxFrames().size();
}

void FakeOdrivePeer::clearFaults() {
	silent_ = false;
	respond_ = true;
	responseDelayMs_ = 0;
	faultNode_ = -1;
	faultDlc_ = -1;
	faultExt_ = false;
}

void FakeOdrivePeer::clearLog() {
	log_.clear();
	sent_.clear();
	requested_.clear();
	refused_.clear();
	undocumented_.clear();
	anticogging_ = foreign_ = 0;
	torqueSeen_ = modesSeen_ = false;
}

// ---------------------------------------------------------------------------------------------------------------
// axis model
// ---------------------------------------------------------------------------------------------------------------
void FakeOdrivePeer::setState(uint32_t s) {
	pendingValid_ = false;
	state_ = s;
}

void FakeOdrivePeer::setFlags(bool motor, bool encoder, bool controller, bool trajectoryDone) {
	motorFlag_ = motor;
	encoderFlag_ = encoder;
	controllerFlag_ = controller;
	trajDone_ = trajectoryDone;
}

void FakeOdrivePeer::setRefuseRequests(bool refuse, uint32_t error) {
	refuseAll_ = refuse;
	refusalError_ = error;
}

void FakeOdrivePeer::refuseState(uint32_t s, bool refuse) {
	if (refuse) {
		refuseStates_.insert(s);
	} else {
		refuseStates_.erase(s);
	}
}

void FakeOdrivePeer::enterState(uint32_t s) {
	state_ = s;
	pendingValid_ = false;
	if (isCalibrationState(s)) {
		pendingValid_ = true;
		pendingDueUs_ = HostPlatform::nowUs() + uint64_t(calibrationMs_) * 1000;
		pendingState_ = STATE_IDLE;
	}
}

void FakeOdrivePeer::applyRequest(uint32_t requested) {
	requested_.push_back(requested);
	if (refuseAll_ || refuseStates_.count(requested)) {
		refused_.push_back(requested);
		if (refusalError_) {
			axisError_ |= refusalError_;
		}
		return;
	}
	if (applyDelayMs_ == 0) {
		enterState(requested);
	} else {
		pendingValid_ = true; // replaces a pending change
		pendingDueUs_ = HostPlatform::nowUs() + uint64_t(applyDelayMs_) * 1000;
		pendingState_ = requested;
	}
}

std::array<uint8_t, 8> FakeOdrivePeer::heartbeatPayload() const {
	std::array<uint8_t, 8> d{};
	putU32(d, 0, axisError_);							  // CSV 0x001: Axis Error @0 uint32
	d[4] = static_cast<uint8_t>(state_);				  // Axis Current State @4 uint8
	d[5] = motorFlag_ ? 1 : 0;							  // Motor Error Flag @5.0
	d[6] = encoderFlag_ ? 1 : 0;						  // Encoder Error Flag @6.0
	d[7] = (controllerFlag_ ? 0x01 : 0) | (trajDone_ ? 0x80 : 0); // Controller Error Flag @7.0, Trajectory Done @7.7
	return d;
}
std::array<uint8_t, 8> FakeOdrivePeer::encoderPayload() const {
	std::array<uint8_t, 8> d{};
	putF32(d, 0, pos_); // CSV 0x009: Pos Estimate @0 float32
	putF32(d, 1, vel_); // Vel Estimate @4 float32
	return d;
}
std::array<uint8_t, 8> FakeOdrivePeer::busPayload() const {
	std::array<uint8_t, 8> d{};
	putF32(d, 0, vbus_); // CSV 0x017: Bus Voltage @0 float32
	putF32(d, 1, ibus_); // Bus Current @4 float32
	return d;
}

// ---------------------------------------------------------------------------------------------------------------
// cyclic messages and time
// ---------------------------------------------------------------------------------------------------------------
void FakeOdrivePeer::setHeartbeat(bool enabled, uint32_t periodMs) {
	hbEnabled_ = enabled;
	hbPeriodMs_ = periodMs ? periodMs : 1;
	hbNextUs_ = HostPlatform::nowUs() + uint64_t(hbPeriodMs_) * 1000;
}
void FakeOdrivePeer::setEncoderCyclic(bool enabled, uint32_t periodMs) {
	encEnabled_ = enabled;
	encPeriodMs_ = periodMs ? periodMs : 1;
	encNextUs_ = HostPlatform::nowUs() + uint64_t(encPeriodMs_) * 1000;
}

uint64_t FakeOdrivePeer::nextEventUs() const {
	uint64_t best = NEVER;
	if (pendingValid_) {
		best = std::min(best, pendingDueUs_);
	}
	if (hbEnabled_) {
		best = std::min(best, hbNextUs_);
	}
	if (encEnabled_) {
		best = std::min(best, encNextUs_);
	}
	if (!out_.empty()) {
		best = std::min(best, out_.front().dueUs);
	}
	return best;
}

void FakeOdrivePeer::process() {
	poll();
	const uint64_t now = HostPlatform::nowUs();
	for (;;) {
		uint64_t best = NEVER;
		int which = -1;
		if (pendingValid_ && pendingDueUs_ <= now && pendingDueUs_ < best) {
			best = pendingDueUs_;
			which = 0;
		}
		if (hbEnabled_ && hbNextUs_ <= now && hbNextUs_ < best) {
			best = hbNextUs_;
			which = 1;
		}
		if (encEnabled_ && encNextUs_ <= now && encNextUs_ < best) {
			best = encNextUs_;
			which = 2;
		}
		if (!out_.empty() && out_.front().dueUs <= now && out_.front().dueUs < best) {
			best = out_.front().dueUs;
			which = 3;
		}
		if (which < 0) {
			break;
		}
		switch (which) {
		case 0:
			pendingValid_ = false;
			enterState(pendingState_);
			break;
		case 1: {
			const uint64_t period = uint64_t(hbPeriodMs_) * 1000;
			hbNextUs_ += period;
			if (hbNextUs_ <= now) { // late call: deliver one frame, skip the rest
				uint64_t missed = (now - hbNextUs_) / period + 1;
				skippedCyclic_ += static_cast<uint32_t>(missed);
				hbNextUs_ += missed * period;
			}
			sendHeartbeat();
			break;
		}
		case 2: {
			const uint64_t period = uint64_t(encPeriodMs_) * 1000;
			encNextUs_ += period;
			if (encNextUs_ <= now) {
				uint64_t missed = (now - encNextUs_) / period + 1;
				skippedCyclic_ += static_cast<uint32_t>(missed);
				encNextUs_ += missed * period;
			}
			sendEncoderEstimates();
			break;
		}
		case 3: {
			Out o = std::move(out_.front());
			out_.pop_front();
			deliver(o.id, o.data, o.ext, o.rtr);
			break;
		}
		}
	}
}

void FakeOdrivePeer::advance(uint32_t ms) {
	for (uint32_t i = 0; i < ms; i++) {
		HostPlatform::advanceMs(1);
		process();
	}
}

// ---------------------------------------------------------------------------------------------------------------
// sending to the driver
// ---------------------------------------------------------------------------------------------------------------
uint32_t FakeOdrivePeer::deliver(uint32_t id, const std::vector<uint8_t> &data, bool ext, bool rtr) {
	uint32_t accepted = HostTarget::canReceive(id, data, ext, rtr);
	Sent s;
	s.timeMs = tickMs();
	s.id = id;
	s.extId = ext;
	s.rtr = rtr;
	s.dlc = static_cast<uint8_t>(data.size());
	for (size_t i = 0; i < data.size() && i < 8; i++) {
		s.data[i] = data[i];
	}
	s.accepted = accepted;
	sent_.push_back(s);
	return accepted;
}

uint32_t FakeOdrivePeer::emit(uint8_t cmd, const std::array<uint8_t, 8> &payload, uint32_t delayMs) {
	if (silent_) {
		return 0;
	}
	const uint32_t node = faultNode_ >= 0 ? static_cast<uint32_t>(faultNode_) : node_;
	const bool ext = faultExt_;
	const uint32_t id = ext ? (((node << 5) | cmd) & 0x1FFFFFFF) : ((((node & 0x3F) << 5) | cmd) & 0x7FF);
	size_t dlc = faultDlc_ >= 0 ? static_cast<size_t>(std::min(faultDlc_, 8)) : 8;
	std::vector<uint8_t> data(payload.begin(), payload.begin() + dlc);
	if (delayMs == 0) {
		return deliver(id, data, ext, false);
	}
	Out o{HostPlatform::nowUs() + uint64_t(delayMs) * 1000, id, ext, false, std::move(data)};
	auto it = std::upper_bound(out_.begin(), out_.end(), o, [](const Out &a, const Out &b) { return a.dueUs < b.dueUs; });
	out_.insert(it, std::move(o));
	return 0;
}

uint32_t FakeOdrivePeer::sendHeartbeat() { return emit(CMD_HEARTBEAT, heartbeatPayload(), 0); }
uint32_t FakeOdrivePeer::sendEncoderEstimates() { return emit(CMD_ENCODER_ESTIMATES, encoderPayload(), 0); }
uint32_t FakeOdrivePeer::sendBusVoltage() { return emit(CMD_BUS_VOLTAGE, busPayload(), 0); }

uint32_t FakeOdrivePeer::inject(uint32_t id, const std::vector<uint8_t> &data, bool extId, bool rtr) {
	return deliver(id, data, extId, rtr);
}

// ---------------------------------------------------------------------------------------------------------------
// decoding of the driver's frames
// ---------------------------------------------------------------------------------------------------------------
void FakeOdrivePeer::poll() {
	const auto &frames = HostTarget::canTxFrames();
	if (cursor_ > frames.size()) {
		cursor_ = 0; // the log was cleared behind our back: start again
	}
	while (cursor_ < frames.size()) {
		const HostTarget::CanTxFrame &f = frames[cursor_];
		if (f.result == HostTarget::CanTxResult::Pending) {
			break; // not acknowledged yet: the frame has not reached the ODrive, later frames wait (log order)
		}
		cursor_++;
		if (f.result == HostTarget::CanTxResult::Sent) {
			decode(f);
		}
	}
}

void FakeOdrivePeer::violate(ViolationKind kind, const Frame &f) {
	Violation v;
	v.kind = kind;
	v.cmd = f.cmd;
	v.id = f.id;
	v.dlc = f.dlc;
	v.timeMs = f.timeMs;
	violations_.push_back(v);
}

void FakeOdrivePeer::decode(const HostTarget::CanTxFrame &t) {
	Frame f;
	f.timeMs = t.time;
	f.id = t.id;
	f.extId = t.extId;
	f.rtr = t.rtr;
	f.dlc = t.dlc;
	f.data = t.data;
	f.cmd = static_cast<uint8_t>(t.id & 0x1F);
	const uint32_t node = t.extId ? (t.id >> 5) : ((t.id >> 5) & 0x3F);
	if (node != node_) {
		foreign_++;
		return;
	}
	if (t.extId) {
		f.accepted = false;
		log_.push_back(f);
		violate(ViolationKind::ExtendedId, f);
		return;
	}
	const bool isRequest = f.cmd == CMD_ENCODER_ESTIMATES || f.cmd == CMD_BUS_VOLTAGE;
	const bool isSet = f.cmd == CMD_SET_STATE || f.cmd == CMD_SET_MODES || f.cmd == CMD_SET_TORQUE || f.cmd == CMD_ANTICOGGING;
	if (!isRequest && !isSet) {
		f.accepted = false;
		log_.push_back(f);
		violate(ViolationKind::UnknownCommand, f);
		return;
	}
	if (isRequest != f.rtr) {
		f.accepted = false;
		log_.push_back(f);
		violate(ViolationKind::RtrMismatch, f);
		return;
	}
	uint8_t wantDlc = 0;
	switch (f.cmd) {
	case CMD_SET_STATE: wantDlc = 4; break;
	case CMD_SET_MODES: wantDlc = 8; break;
	case CMD_SET_TORQUE: wantDlc = 4; break;
	default: wantDlc = 0; break;
	}
	if (f.cmd == CMD_ANTICOGGING) {
		if (f.dlc != 0) { // L3: harmless difference, documented only
			undocumented_.push_back({f.cmd, f.dlc, f.timeMs});
		}
	} else if (f.dlc != wantDlc) {
		f.accepted = false;
		log_.push_back(f);
		violate(ViolationKind::WrongDlc, f);
		return;
	}
	log_.push_back(f);
	switch (f.cmd) {
	case CMD_SET_STATE:
		applyRequest(f.u32(0));
		break;
	case CMD_SET_MODES:
		modesSeen_ = true;
		lastControl_ = f.i32(0);
		lastInput_ = f.i32(1);
		break;
	case CMD_SET_TORQUE:
		torqueSeen_ = true;
		lastTorque_ = f.f32(0);
		break;
	case CMD_ANTICOGGING:
		anticogging_++;
		break;
	case CMD_ENCODER_ESTIMATES:
		if (respond_) {
			emit(CMD_ENCODER_ESTIMATES, encoderPayload(), responseDelayMs_);
		}
		break;
	case CMD_BUS_VOLTAGE:
		if (respond_) {
			emit(CMD_BUS_VOLTAGE, busPayload(), responseDelayMs_);
		}
		break;
	}
}

// ---------------------------------------------------------------------------------------------------------------
// records
// ---------------------------------------------------------------------------------------------------------------
std::vector<FakeOdrivePeer::Frame> FakeOdrivePeer::framesWithCmd(uint8_t cmd) const {
	std::vector<Frame> r;
	for (const Frame &f : log_) {
		if (f.cmd == cmd) {
			r.push_back(f);
		}
	}
	return r;
}
size_t FakeOdrivePeer::count(uint8_t cmd) const {
	return framesWithCmd(cmd).size();
}
size_t FakeOdrivePeer::rtrCount(uint8_t cmd) const {
	size_t n = 0;
	for (const Frame &f : log_) {
		if (f.cmd == cmd && f.rtr) {
			n++;
		}
	}
	return n;
}

void FakeOdrivePeer::expectViolation(ViolationKind kind, int cmd) {
	expected_.push_back({kind, cmd});
}

std::vector<FakeOdrivePeer::Violation> FakeOdrivePeer::unexpectedViolations() const {
	std::vector<Violation> r;
	for (const Violation &v : violations_) {
		bool ok = false;
		for (const Expected &e : expected_) {
			if (e.kind == v.kind && (e.cmd < 0 || e.cmd == v.cmd)) {
				ok = true;
				break;
			}
		}
		if (!ok) {
			r.push_back(v);
		}
	}
	return r;
}
