/*
 * FakeRmdPeer.cpp - see FakeRmdPeer.h for what is modelled, the sources and the confidence.
 */
#include "FakeRmdPeer.h"

#include "HostPlatform.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <sstream>

namespace {
constexpr uint64_t NEVER = std::numeric_limits<uint64_t>::max();

uint32_t tickMs() { return static_cast<uint32_t>(HostPlatform::nowUs() / 1000); }

// SP 2.34: the commands the motor can report cyclically
bool isReportCmd(uint8_t c) {
	return c == 0x60 || c == 0x61 || c == 0x62 || c == 0x92 || c == 0x9A || c == 0x9C || c == 0x9D || c == 0x9E;
}
} // namespace

std::string FakeRmdPeer::Violation::text() const {
	std::ostringstream s;
	switch (kind) {
	case ViolationKind::ExtendedId: s << "extended id frame (SP 1.1.2: standard ids)"; break;
	case ViolationKind::Rtr: s << "remote frame (SP 1.1.2: data frames)"; break;
	case ViolationKind::WrongDlc: s << "DLC other than 8 (SP 1.1.2)"; break;
	case ViolationKind::UnknownCommand: s << "command byte the driver is not expected to send"; break;
	case ViolationKind::NonZeroNull: s << "non-zero byte in a field the SP marks NULL"; break;
	case ViolationKind::BadReportCmd: s << "0xB6 command byte outside the SP 2.34 list"; break;
	case ViolationKind::BadEnable: s << "0xB6 enable byte other than 0 / 1"; break;
	}
	s << " (cmd 0x" << std::hex << cmd << ", id 0x" << id << std::dec << ", dlc " << int(dlc) << ", t=" << timeMs << " ms)";
	return s.str();
}

FakeRmdPeer::FakeRmdPeer(uint8_t nodeId) : node_(nodeId) {
	reset();
}

void FakeRmdPeer::reset() {
	temp_ = mosTemp_ = 25;
	brake_ = 1;
	voltage_ = 480;
	errors_ = 0;
	setModel("X8S2V10");
	multiTurn_ = 0;
	singleTurn_ = 0;
	shaftAngle_ = speed_ = iq_ = 0;
	followTorque_ = true;
	running_ = false;
	cyclic_.clear();
	skippedCyclic_ = 0;
	clearFaults();
	allowed_.clear();
	out_.clear();
	log_.clear();
	sent_.clear();
	violations_.clear();
	expected_.clear();
	lastIq_ = 0;
	iqSeen_ = false;
	foreign_ = 0;
	syncCursor();
}

void FakeRmdPeer::syncCursor() {
	cursor_ = HostTarget::canTxFrames().size();
}

void FakeRmdPeer::clearFaults() {
	silent_ = false;
	responseDelayMs_ = 0;
	faultNode_ = faultCmd_ = faultDlc_ = -1;
}

void FakeRmdPeer::clearLog() {
	log_.clear();
	sent_.clear();
	foreign_ = 0;
	iqSeen_ = false;
}

void FakeRmdPeer::setModel(const std::string &m) {
	model_.fill(0);
	for (size_t i = 0; i < m.size() && i < model_.size(); i++) {
		model_[i] = static_cast<uint8_t>(m[i]);
	}
}

std::array<uint8_t, 8> FakeRmdPeer::replyPayload(uint8_t cmd) const {
	std::array<uint8_t, 8> d{};
	d[0] = cmd;
	switch (cmd) {
	case 0xA1: // SP 2.19
		d[1] = static_cast<uint8_t>(temp_);
		d[2] = static_cast<uint8_t>(iq_ & 0xFF);
		d[3] = static_cast<uint8_t>((iq_ >> 8) & 0xFF);
		d[4] = static_cast<uint8_t>(speed_ & 0xFF);
		d[5] = static_cast<uint8_t>((speed_ >> 8) & 0xFF);
		d[6] = static_cast<uint8_t>(shaftAngle_ & 0xFF);
		d[7] = static_cast<uint8_t>((shaftAngle_ >> 8) & 0xFF);
		break;
	case 0x92: // SP 2.12
		for (int i = 0; i < 4; i++) {
			d[4 + i] = static_cast<uint8_t>((static_cast<uint32_t>(multiTurn_) >> (8 * i)) & 0xFF);
		}
		break;
	case 0x94: // SP 2.13
		d[6] = static_cast<uint8_t>(singleTurn_ & 0xFF);
		d[7] = static_cast<uint8_t>((singleTurn_ >> 8) & 0xFF);
		break;
	case 0x9A: // SP 2.14
		d[1] = static_cast<uint8_t>(temp_);
		d[2] = static_cast<uint8_t>(mosTemp_);
		d[3] = brake_;
		d[4] = static_cast<uint8_t>(voltage_ & 0xFF);
		d[5] = static_cast<uint8_t>((voltage_ >> 8) & 0xFF);
		d[6] = static_cast<uint8_t>(errors_ & 0xFF);
		d[7] = static_cast<uint8_t>((errors_ >> 8) & 0xFF);
		break;
	case 0xB5: // SP 2.33
		for (size_t i = 0; i < model_.size(); i++) {
			d[1 + i] = model_[i];
		}
		break;
	default: break;
	}
	return d;
}

uint32_t FakeRmdPeer::cyclicPeriodMs(uint8_t cmd) const {
	auto it = cyclic_.find(cmd);
	return it == cyclic_.end() ? 0 : it->second.periodMs;
}

// ---------------------------------------------------------------------------------------------------------------
// time
// ---------------------------------------------------------------------------------------------------------------
uint64_t FakeRmdPeer::nextEventUs() const {
	uint64_t best = NEVER;
	for (const auto &c : cyclic_) {
		best = std::min(best, c.second.nextUs);
	}
	if (!out_.empty()) {
		best = std::min(best, out_.front().dueUs);
	}
	return best;
}

void FakeRmdPeer::process() {
	poll();
	const uint64_t now = HostPlatform::nowUs();
	for (;;) {
		uint64_t best = NEVER;
		int which = -1; // 0 = delayed reply, 1 = cyclic
		uint8_t cyc = 0;
		if (!out_.empty() && out_.front().dueUs <= now) {
			best = out_.front().dueUs;
			which = 0;
		}
		for (const auto &c : cyclic_) {
			if (c.second.nextUs <= now && c.second.nextUs < best) {
				best = c.second.nextUs;
				which = 1;
				cyc = c.first;
			}
		}
		if (which < 0) {
			break;
		}
		if (which == 0) {
			Out o = std::move(out_.front());
			out_.pop_front();
			deliver(o.id, o.data);
		} else {
			Cyclic &c = cyclic_[cyc];
			const uint64_t period = uint64_t(c.periodMs) * 1000;
			c.nextUs += period;
			if (c.nextUs <= now) { // late call: deliver one reply, skip the rest
				uint64_t missed = (now - c.nextUs) / period + 1;
				skippedCyclic_ += static_cast<uint32_t>(missed);
				c.nextUs += missed * period;
			}
			emit(replyPayload(cyc), 0);
		}
	}
}

void FakeRmdPeer::advance(uint32_t ms) {
	for (uint32_t i = 0; i < ms; i++) {
		HostPlatform::advanceMs(1);
		process();
	}
}

// ---------------------------------------------------------------------------------------------------------------
// sending to the driver
// ---------------------------------------------------------------------------------------------------------------
uint32_t FakeRmdPeer::deliver(uint32_t id, const std::vector<uint8_t> &data) {
	uint32_t accepted = HostTarget::canReceive(id, data, false, false);
	Sent s;
	s.timeMs = tickMs();
	s.id = id;
	s.extId = false;
	s.dlc = static_cast<uint8_t>(data.size());
	for (size_t i = 0; i < data.size() && i < 8; i++) {
		s.data[i] = data[i];
	}
	s.accepted = accepted;
	sent_.push_back(s);
	return accepted;
}

uint32_t FakeRmdPeer::emit(const std::array<uint8_t, 8> &payload, uint32_t delayMs) {
	if (silent_) {
		return 0;
	}
	const uint32_t node = faultNode_ >= 0 ? static_cast<uint32_t>(faultNode_) : node_;
	const uint32_t id = (0x240 + node) & 0x7FF; // SP 1.1.2: reply id 0x240 + ID
	std::array<uint8_t, 8> p = payload;
	if (faultCmd_ >= 0) {
		p[0] = static_cast<uint8_t>(faultCmd_);
	}
	size_t dlc = faultDlc_ >= 0 ? static_cast<size_t>(std::min(faultDlc_, 8)) : 8;
	std::vector<uint8_t> data(p.begin(), p.begin() + dlc);
	if (delayMs == 0) {
		return deliver(id, data);
	}
	Out o{HostPlatform::nowUs() + uint64_t(delayMs) * 1000, id, std::move(data)};
	auto it = std::upper_bound(out_.begin(), out_.end(), o, [](const Out &a, const Out &b) { return a.dueUs < b.dueUs; });
	out_.insert(it, std::move(o));
	return 0;
}

uint32_t FakeRmdPeer::sendReply(uint8_t cmd) {
	return emit(replyPayload(cmd), 0);
}

uint32_t FakeRmdPeer::inject(uint32_t id, const std::vector<uint8_t> &data, bool extId, bool rtr) {
	uint32_t accepted = HostTarget::canReceive(id, data, extId, rtr);
	Sent s;
	s.timeMs = tickMs();
	s.id = id;
	s.extId = extId;
	s.dlc = static_cast<uint8_t>(data.size());
	for (size_t i = 0; i < data.size() && i < 8; i++) {
		s.data[i] = data[i];
	}
	s.accepted = accepted;
	sent_.push_back(s);
	return accepted;
}

// ---------------------------------------------------------------------------------------------------------------
// decoding of the driver's frames
// ---------------------------------------------------------------------------------------------------------------
void FakeRmdPeer::poll() {
	const auto &frames = HostTarget::canTxFrames();
	if (cursor_ > frames.size()) {
		cursor_ = 0; // the log was cleared behind our back: start again
	}
	while (cursor_ < frames.size()) {
		const HostTarget::CanTxFrame &f = frames[cursor_];
		if (f.result == HostTarget::CanTxResult::Pending) {
			break; // not acknowledged yet: has not reached the motor, later frames wait (log order)
		}
		cursor_++;
		if (f.result == HostTarget::CanTxResult::Sent) {
			decode(f);
		}
	}
}

void FakeRmdPeer::violate(ViolationKind kind, const Frame &f) {
	Violation v;
	v.kind = kind;
	v.cmd = f.cmd;
	v.id = f.id;
	v.dlc = f.dlc;
	v.timeMs = f.timeMs;
	violations_.push_back(v);
}

void FakeRmdPeer::decode(const HostTarget::CanTxFrame &t) {
	Frame f;
	f.timeMs = t.time;
	f.id = t.id;
	f.extId = t.extId;
	f.rtr = t.rtr;
	f.dlc = t.dlc;
	f.data = t.data;
	f.cmd = t.data[0];
	f.broadcast = !t.extId && t.id == 0x280;
	const bool mine = t.id == uint32_t(0x140 + node_);
	if (!mine && !f.broadcast) {
		foreign_++;
		return;
	}
	auto reject = [&](ViolationKind k) {
		f.accepted = false;
		log_.push_back(f);
		violate(k, f);
	};
	if (t.extId) {
		reject(ViolationKind::ExtendedId);
		return;
	}
	if (t.rtr) {
		reject(ViolationKind::Rtr);
		return;
	}
	if (t.dlc != 8) {
		reject(ViolationKind::WrongDlc);
		return;
	}
	static const std::set<uint8_t> expectedCmds = {0x80, 0xA1, 0x92, 0x9A, 0xB5, 0xB6};
	if (!expectedCmds.count(f.cmd) && !allowed_.count(f.cmd)) {
		reject(ViolationKind::UnknownCommand);
		return;
	}
	// fields the SP marks NULL must be 0x00
	auto nonZero = [&](std::initializer_list<int> idx) {
		for (int i : idx) {
			if (f.data[i] != 0) {
				return true;
			}
		}
		return false;
	};
	if (f.cmd == 0xA1) {
		if (nonZero({1, 2, 3, 6, 7})) {
			reject(ViolationKind::NonZeroNull);
			return;
		}
	} else if (f.cmd == 0xB6) {
		if (!isReportCmd(f.data[1])) {
			reject(ViolationKind::BadReportCmd);
			return;
		}
		if (f.data[2] > 1) {
			reject(ViolationKind::BadEnable);
			return;
		}
		if (nonZero({5, 6, 7})) {
			reject(ViolationKind::NonZeroNull);
			return;
		}
	} else if (nonZero({1, 2, 3, 4, 5, 6, 7})) {
		reject(ViolationKind::NonZeroNull);
		return;
	}
	log_.push_back(f);

	switch (f.cmd) {
	case 0x80: // SP 2.17: reply = the sent frame
	case 0x81:
		running_ = false;
		emit(f.data, responseDelayMs_);
		break;
	case 0x77:
	case 0x78:
		emit(f.data, responseDelayMs_);
		break;
	case 0xA1: {
		const int16_t iq = static_cast<int16_t>(f.data[4] | (f.data[5] << 8));
		lastIq_ = iq;
		iqSeen_ = true;
		running_ = true;
		if (followTorque_) {
			iq_ = iq;
		}
		emit(replyPayload(0xA1), responseDelayMs_);
		break;
	}
	case 0x92:
	case 0x94:
	case 0x9A:
	case 0xB5:
		emit(replyPayload(f.cmd), responseDelayMs_);
		break;
	case 0xB6: { // SP 2.34: no reply to the 0xB6 itself
		const uint8_t c = f.data[1];
		if (f.data[2] == 1) {
			const uint32_t interval = f.data[3] | (f.data[4] << 8);
			const uint32_t period = (interval + 1) * 10; // SP: unit 10 ms, example value 1 = 20 ms
			cyclic_[c] = Cyclic{HostPlatform::nowUs() + uint64_t(period) * 1000, period};
		} else {
			cyclic_.erase(c);
		}
		break;
	}
	default:
		emit(f.data, responseDelayMs_); // declared with allowCommand(): reply = the sent frame
		break;
	}
}

std::vector<FakeRmdPeer::Frame> FakeRmdPeer::framesWithCmd(uint8_t cmd) const {
	std::vector<Frame> r;
	for (const auto &f : log_) {
		if (f.cmd == cmd) {
			r.push_back(f);
		}
	}
	return r;
}

size_t FakeRmdPeer::count(uint8_t cmd) const {
	return framesWithCmd(cmd).size();
}

void FakeRmdPeer::expectViolation(ViolationKind kind, int cmd) {
	expected_.push_back({kind, cmd});
}

std::vector<FakeRmdPeer::Violation> FakeRmdPeer::unexpectedViolations() const {
	std::vector<Violation> r;
	for (const auto &v : violations_) {
		bool ok = false;
		for (const auto &e : expected_) {
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
