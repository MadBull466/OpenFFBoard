/*
 * FakeVescPeer.cpp - see FakeVescPeer.h for what is modelled, the sources and the confidence.
 */
#include "FakeVescPeer.h"

#include "HostPlatform.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>

namespace {
constexpr uint64_t NEVER = std::numeric_limits<uint64_t>::max();

uint32_t tickMs() { return static_cast<uint32_t>(HostPlatform::nowUs() / 1000); }

// Copy of SRC util/crc.c:26-52 (crc16_tab), machine-extracted from the supplied source; standard MSB-first table of poly 0x1021
const uint16_t CRC16_TAB[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};

void putU16(std::vector<uint8_t> &v, uint16_t x) {
	v.push_back(static_cast<uint8_t>(x >> 8));
	v.push_back(static_cast<uint8_t>(x));
}
void putU32(std::vector<uint8_t> &v, uint32_t x) {
	v.push_back(static_cast<uint8_t>(x >> 24));
	v.push_back(static_cast<uint8_t>(x >> 16));
	v.push_back(static_cast<uint8_t>(x >> 8));
	v.push_back(static_cast<uint8_t>(x));
}
// buffer_append_float16 / buffer_append_float32 (SRC util/buffer.c:71-76): (intN)(x * scale), truncation toward zero
void putF16(std::vector<uint8_t> &v, float x, float scale) { putU16(v, static_cast<uint16_t>(static_cast<int16_t>(x * scale))); }
void putF32(std::vector<uint8_t> &v, float x, float scale) { putU32(v, static_cast<uint32_t>(static_cast<int32_t>(x * scale))); }
} // namespace

uint16_t FakeVescPeer::crc16(const uint8_t *buf, size_t len) {
	// SRC util/crc.c:56-64: crc16_rolling(0, buf, len)
	uint16_t cksum = 0;
	for (size_t i = 0; i < len; i++) {
		cksum = static_cast<uint16_t>(CRC16_TAB[((cksum >> 8) ^ *buf++) & 0xFF] ^ (cksum << 8));
	}
	return cksum;
}

std::vector<FakeVescPeer::RawFrame> FakeVescPeer::buildBuffer(uint8_t dest, uint8_t sender, uint8_t flag, const std::vector<uint8_t> &payload, const BufferOptions &opt) {
	std::vector<RawFrame> out;
	const size_t len = payload.size();
	if (len <= 6) { // SRC comm_can.c:445-455
		RawFrame f;
		f.id = makeId(dest, PKT_PROCESS_SHORT_BUFFER);
		f.data.push_back(sender);
		f.data.push_back(flag);
		f.data.insert(f.data.end(), payload.begin(), payload.end());
		out.push_back(f);
		return out;
	}
	std::vector<RawFrame> fills;
	if (opt.longFill) {
		for (size_t i = 0; i < len; i += 6) {
			RawFrame f;
			f.id = makeId(dest, PKT_FILL_RX_BUFFER_LONG);
			f.data.push_back(static_cast<uint8_t>(i >> 8));
			f.data.push_back(static_cast<uint8_t>(i & 0xFF));
			const size_t n = std::min<size_t>(6, len - i);
			f.data.insert(f.data.end(), payload.begin() + i, payload.begin() + i + n);
			fills.push_back(f);
		}
	} else {
		size_t endA = 0;
		for (size_t i = 0; i < len; i += 7) { // SRC comm_can.c:462-475
			if (i > 255) {
				break;
			}
			endA = i + 7;
			RawFrame f;
			f.id = makeId(dest, PKT_FILL_RX_BUFFER);
			f.data.push_back(static_cast<uint8_t>(i));
			const size_t n = std::min<size_t>(7, len - i);
			f.data.insert(f.data.end(), payload.begin() + i, payload.begin() + i + n);
			fills.push_back(f);
		}
		for (size_t i = endA; i < len; i += 6) { // SRC comm_can.c:478-492
			RawFrame f;
			f.id = makeId(dest, PKT_FILL_RX_BUFFER_LONG);
			f.data.push_back(static_cast<uint8_t>(i >> 8));
			f.data.push_back(static_cast<uint8_t>(i & 0xFF));
			const size_t n = std::min<size_t>(6, len - i);
			f.data.insert(f.data.end(), payload.begin() + i, payload.begin() + i + n);
			fills.push_back(f);
		}
	}
	if (opt.dropFill >= 0 && static_cast<size_t>(opt.dropFill) < fills.size()) {
		fills.erase(fills.begin() + opt.dropFill);
	}
	if (opt.reverseFill) {
		std::reverse(fills.begin(), fills.end());
	}
	out = fills;
	RawFrame p; // SRC comm_can.c:494-503
	p.id = makeId(dest, PKT_PROCESS_RX_BUFFER);
	uint16_t crc = crc16(payload);
	if (opt.corruptCrc) {
		crc ^= 0x0001;
	}
	p.data = {sender, flag, static_cast<uint8_t>(len >> 8), static_cast<uint8_t>(len & 0xFF), static_cast<uint8_t>(crc >> 8), static_cast<uint8_t>(crc & 0xFF)};
	out.push_back(p);
	return out;
}

uint16_t FakeVescPeer::Frame::u16(int off) const {
	return static_cast<uint16_t>((data[off] << 8) | data[off + 1]);
}
uint32_t FakeVescPeer::Frame::u32(int off) const {
	return (uint32_t(data[off]) << 24) | (uint32_t(data[off + 1]) << 16) | (uint32_t(data[off + 2]) << 8) | data[off + 3];
}

std::string FakeVescPeer::Violation::text() const {
	std::ostringstream s;
	switch (kind) {
	case ViolationKind::StandardId: s << "standard id frame (the driver must use extended ids)"; break;
	case ViolationKind::RemoteFrame: s << "RTR frame (SPEC: remote bit 0)"; break;
	case ViolationKind::UpperIdBits: s << "id bits 16-28 not zero"; break;
	case ViolationKind::WrongDlc: s << "DLC differs from the packet's format"; break;
	case ViolationKind::UnknownPacket: s << "packet number the driver does not use"; break;
	case ViolationKind::BadFlag: s << "PROCESS_SHORT_BUFFER flag other than 0"; break;
	case ViolationKind::UnknownComm: s << "COMM id other than FW_VERSION / GET_VALUES_SELECTIVE"; break;
	case ViolationKind::UnsupportedMask: s << "GET_VALUES_SELECTIVE mask with bits other than 8, 15, 16"; break;
	}
	s << " (packet " << packet << ", id 0x" << std::hex << id << std::dec << ", dlc " << int(dlc) << ", t=" << timeMs << " ms)";
	return s.str();
}

FakeVescPeer::FakeVescPeer(uint8_t unitId) : unit_(unitId) {
	reset();
}

void FakeVescPeer::reset() {
	present_ = true;
	fw_ = FwInfo{};
	vin_ = 24.0f;
	faultCode_ = 0;
	pidPos_ = rotor_ = 0;
	timeoutMs_ = 1000;
	lastSetMs_ = tickMs();
	currentSeen_ = false;
	lastCurrentRaw_ = 0;
	status_ = StatusValues{};
	statusMask_ = 0;
	statusPeriodMs_ = 20;
	statusNextUs_ = 0;
	skippedCyclic_ = 0;
	clearFaults();
	out_.clear();
	log_.clear();
	sent_.clear();
	violations_.clear();
	undocumented_.clear();
	expected_.clear();
	commCount_.clear();
	foreign_ = 0;
	syncCursor();
}

void FakeVescPeer::syncCursor() {
	cursor_ = HostTarget::canTxFrames().size();
}

void FakeVescPeer::clearFaults() {
	silent_ = false;
	respond_ = true;
	replyDelayMs_ = 0;
	opt_ = BufferOptions{};
	faultDlc_ = -1;
	faultDlcPacket_ = -1;
	faultSender_ = -1;
	faultDest_ = -1;
}

void FakeVescPeer::clearLog() {
	log_.clear();
	sent_.clear();
	undocumented_.clear();
	commCount_.clear();
	foreign_ = 0;
}

bool FakeVescPeer::timedOut() const {
	if (timeoutMs_ == 0) {
		return false;
	}
	return tickMs() - lastSetMs_ > timeoutMs_;
}

// ---------------------------------------------------------------------------------------------------------------
// payloads
// ---------------------------------------------------------------------------------------------------------------
std::vector<uint8_t> FakeVescPeer::fwVersionPayload() const {
	// SRC commands.c:231-293
	std::vector<uint8_t> v;
	v.push_back(COMM_FW_VERSION);
	v.push_back(fw_.major);
	v.push_back(fw_.minor);
	v.insert(v.end(), fw_.hwName.begin(), fw_.hwName.end());
	v.push_back(0);
	v.insert(v.end(), fw_.uuid.begin(), fw_.uuid.end());
	v.push_back(fw_.pairing ? 1 : 0); // pairing_done (commands.c:250) ...
	v.push_back(fw_.testVersion);	  // ... then FW_TEST_VERSION_NUMBER (commands.c:251)
	v.push_back(fw_.hwType);		  // HW_TYPE_VESC
	v.push_back(0);					  // conf_custom_cfg_num()
	v.push_back(0);					  // HAS_PHASE_FILTERS
	v.push_back(0);					  // QML hw flag
	v.push_back(0);					  // QML app flag
	v.push_back(0);					  // nrf flags
	v.insert(v.end(), fw_.fwName.begin(), fw_.fwName.end());
	v.push_back(0);
	putU32(v, fw_.hwCrc);
	return v;
}

std::vector<uint8_t> FakeVescPeer::valuesPayload(uint32_t mask) const {
	if (mask & ~((1u << 8) | (1u << 15) | (1u << 16))) {
		return {};
	}
	std::vector<uint8_t> v;
	v.push_back(COMM_GET_VALUES_SELECTIVE);
	putU32(v, mask);
	if (mask & (1u << 8)) {
		putF16(v, vin_, 1e1f); // input voltage filtered, SRC commands.c:421-423
	}
	if (mask & (1u << 15)) {
		v.push_back(faultCode_); // mc_interface_get_fault(), commands.c:442-444
	}
	if (mask & (1u << 16)) {
		putF32(v, static_cast<float>(pidPos_), 1e6f); // pid_pos_now, commands.c:445-447
	}
	return v;
}

// ---------------------------------------------------------------------------------------------------------------
// time
// ---------------------------------------------------------------------------------------------------------------
void FakeVescPeer::setStatusCyclic(uint8_t mask, uint32_t periodMs) {
	statusMask_ = mask & 0x1F;
	statusPeriodMs_ = periodMs ? periodMs : 1;
	statusNextUs_ = HostPlatform::nowUs() + uint64_t(statusPeriodMs_) * 1000;
}

uint64_t FakeVescPeer::nextEventUs() const {
	uint64_t best = NEVER;
	if (statusMask_) {
		best = std::min(best, statusNextUs_);
	}
	if (!out_.empty()) {
		best = std::min(best, out_.front().dueUs);
	}
	return best;
}

void FakeVescPeer::process() {
	poll();
	const uint64_t now = HostPlatform::nowUs();
	for (;;) {
		uint64_t best = NEVER;
		int which = -1;
		if (statusMask_ && statusNextUs_ <= now && statusNextUs_ < best) {
			best = statusNextUs_;
			which = 0;
		}
		if (!out_.empty() && out_.front().dueUs <= now && out_.front().dueUs < best) {
			best = out_.front().dueUs;
			which = 1;
		}
		if (which < 0) {
			break;
		}
		if (which == 0) {
			const uint64_t period = uint64_t(statusPeriodMs_) * 1000;
			statusNextUs_ += period;
			if (statusNextUs_ <= now) { // late call: deliver one set, skip the rest
				uint64_t missed = (now - statusNextUs_) / period + 1;
				skippedCyclic_ += static_cast<uint32_t>(missed);
				statusNextUs_ += missed * period;
			}
			static const uint8_t pk[5] = {PKT_STATUS, PKT_STATUS_2, PKT_STATUS_3, PKT_STATUS_4, PKT_STATUS_5};
			for (int i = 0; i < 5; i++) {
				if (statusMask_ & (1 << i)) {
					sendStatus(pk[i]);
				}
			}
		} else {
			Out o = std::move(out_.front());
			out_.pop_front();
			deliver(o.id, o.data);
		}
	}
}

void FakeVescPeer::advance(uint32_t ms) {
	for (uint32_t i = 0; i < ms; i++) {
		HostPlatform::advanceMs(1);
		process();
	}
}

// ---------------------------------------------------------------------------------------------------------------
// sending to the driver
// ---------------------------------------------------------------------------------------------------------------
uint32_t FakeVescPeer::deliver(uint32_t id, const std::vector<uint8_t> &data) {
	uint32_t accepted = HostTarget::canReceive(id, data, true, false);
	Sent s;
	s.timeMs = tickMs();
	s.id = id;
	s.packet = static_cast<uint8_t>((id >> 8) & 0xFF);
	s.dlc = static_cast<uint8_t>(data.size());
	for (size_t i = 0; i < data.size() && i < 8; i++) {
		s.data[i] = data[i];
	}
	s.accepted = accepted;
	sent_.push_back(s);
	return accepted;
}

uint32_t FakeVescPeer::emit(uint32_t id, const std::vector<uint8_t> &dataIn, uint32_t delayMs) {
	if (silent_) {
		return 0;
	}
	std::vector<uint8_t> data = dataIn;
	const int packet = static_cast<int>((id >> 8) & 0xFF);
	if (faultDlc_ >= 0 && (faultDlcPacket_ < 0 || faultDlcPacket_ == packet)) {
		data.resize(static_cast<size_t>(std::min(faultDlc_, 8)), 0);
	}
	if (delayMs == 0) {
		return deliver(id, data);
	}
	Out o{HostPlatform::nowUs() + uint64_t(delayMs) * 1000, id, std::move(data)};
	auto it = std::upper_bound(out_.begin(), out_.end(), o, [](const Out &a, const Out &b) { return a.dueUs < b.dueUs; });
	out_.insert(it, std::move(o));
	return 0;
}

uint32_t FakeVescPeer::sendReply(uint8_t dest, const std::vector<uint8_t> &payload) {
	uint32_t n = 0;
	for (const RawFrame &f : buildBuffer(destFor(dest), senderId(), 1, payload, opt_)) {
		n += emit(f.id, f.data, replyDelayMs_);
	}
	return n;
}

uint32_t FakeVescPeer::sendRotorPosition() {
	std::vector<uint8_t> d;
	// SRC comm_can.c:2023-2030: (int32_t)(encoder_read_deg() * 100000.0), id = own controller id | 56 << 8
	putU32(d, static_cast<uint32_t>(static_cast<int32_t>(rotor_ * 100000.0)));
	return emit(makeId(senderId(), PKT_POLL_ROTOR_POS), d, replyDelayMs_);
}

uint32_t FakeVescPeer::sendPong(uint8_t dest) {
	// SRC comm_can.c:1843-1850: [own id, HW_TYPE_VESC = 0] to data8[0] | PONG << 8
	return emit(makeId(destFor(dest), PKT_PONG), {senderId(), 0}, replyDelayMs_);
}

uint32_t FakeVescPeer::sendStatus(uint8_t packet) {
	std::vector<uint8_t> d;
	const StatusValues &s = status_;
	switch (packet) {
	case PKT_STATUS: // SRC comm_can.c:1216-1224
		putU32(d, static_cast<uint32_t>(static_cast<int32_t>(s.rpm)));
		putF16(d, static_cast<float>(s.totalCurrent), 1e1f);
		putF16(d, static_cast<float>(s.duty), 1e3f);
		break;
	case PKT_STATUS_2: // :1226-1233
		putF32(d, static_cast<float>(s.ampHours), 1e4f);
		putF32(d, static_cast<float>(s.ampHoursCharged), 1e4f);
		break;
	case PKT_STATUS_3: // :1235-1242
		putF32(d, static_cast<float>(s.wattHours), 1e4f);
		putF32(d, static_cast<float>(s.wattHoursCharged), 1e4f);
		break;
	case PKT_STATUS_4: // :1244-1253
		putF16(d, static_cast<float>(s.tempFet), 1e1f);
		putF16(d, static_cast<float>(s.tempMotor), 1e1f);
		putF16(d, static_cast<float>(s.inputCurrent), 1e1f);
		putF16(d, static_cast<float>(s.pidPos), 50.0f);
		break;
	case PKT_STATUS_5: // :1255-1263
		putU32(d, static_cast<uint32_t>(s.tachometer));
		putF16(d, static_cast<float>(s.vin), 1e1f);
		putU16(d, 0);
		break;
	default:
		return 0;
	}
	return emit(makeId(senderId(), packet), d, 0);
}

uint32_t FakeVescPeer::inject(uint32_t id, const std::vector<uint8_t> &data, bool extId) {
	uint32_t accepted = HostTarget::canReceive(id, data, extId, false);
	Sent s;
	s.timeMs = tickMs();
	s.id = id;
	s.packet = static_cast<uint8_t>((id >> 8) & 0xFF);
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
void FakeVescPeer::poll() {
	const auto &frames = HostTarget::canTxFrames();
	if (cursor_ > frames.size()) {
		cursor_ = 0; // the log was cleared behind our back: start again
	}
	while (cursor_ < frames.size()) {
		const HostTarget::CanTxFrame &f = frames[cursor_];
		if (f.result == HostTarget::CanTxResult::Pending) {
			break; // not acknowledged yet: the frame has not reached the VESC, later frames wait (log order)
		}
		cursor_++;
		if (f.result == HostTarget::CanTxResult::Sent && present_) {
			decode(f);
		}
	}
}

void FakeVescPeer::violate(ViolationKind kind, Frame &f) {
	f.accepted = false;
	log_.push_back(f);
	Violation v;
	v.kind = kind;
	v.packet = f.packet;
	v.id = f.id;
	v.dlc = f.dlc;
	v.timeMs = f.timeMs;
	violations_.push_back(v);
}

void FakeVescPeer::decode(const HostTarget::CanTxFrame &t) {
	Frame f;
	f.timeMs = t.time;
	f.id = t.id;
	f.unit = static_cast<uint8_t>(t.id & 0xFF);
	f.packet = static_cast<uint8_t>((t.id >> 8) & 0xFF);
	f.extId = t.extId;
	f.rtr = t.rtr;
	f.dlc = t.dlc;
	f.data = t.data;
	if (!t.extId) {
		violate(ViolationKind::StandardId, f);
		return;
	}
	if (f.unit != unit_ && f.unit != 255) {
		foreign_++;
		return;
	}
	if (t.rtr) {
		violate(ViolationKind::RemoteFrame, f);
		return;
	}
	if ((t.id >> 16) != 0) {
		violate(ViolationKind::UpperIdBits, f);
		return;
	}
	switch (f.packet) {
	case PKT_PROCESS_SHORT_BUFFER: {
		if (f.dlc < 3) {
			violate(ViolationKind::WrongDlc, f);
			return;
		}
		if (f.data[1] != 0) {
			violate(ViolationKind::BadFlag, f);
			return;
		}
		f.comm = f.data[2];
		if (f.comm == COMM_FW_VERSION) {
			if (f.dlc != 3) {
				violate(ViolationKind::WrongDlc, f);
				return;
			}
		} else if (f.comm == COMM_GET_VALUES_SELECTIVE) {
			if (f.dlc != 7 && f.dlc != 8) { // mask in data[3..6] (SRC commands.c:393)
				violate(ViolationKind::WrongDlc, f);
				return;
			}
			if (valuesPayload(f.u32(3)).empty()) {
				violate(ViolationKind::UnsupportedMask, f);
				return;
			}
			if (f.dlc == 8) { // B-V2: uninitialised 8th byte, ignored by the VESC
				undocumented_.push_back({f.packet, f.dlc, f.data[7], f.timeMs});
			}
		} else {
			violate(ViolationKind::UnknownComm, f);
			return;
		}
		break;
	}
	case PKT_SET_CURRENT_REL:
		if (f.dlc != 4) {
			violate(ViolationKind::WrongDlc, f);
			return;
		}
		break;
	case PKT_PING:
		if (f.dlc != 1) {
			violate(ViolationKind::WrongDlc, f);
			return;
		}
		break;
	case PKT_POLL_ROTOR_POS:
		if (f.dlc != 0) {
			violate(ViolationKind::WrongDlc, f);
			return;
		}
		break;
	default:
		violate(ViolationKind::UnknownPacket, f);
		return;
	}
	log_.push_back(f);
	handleRequest(f);
}

void FakeVescPeer::handleRequest(const Frame &f) {
	switch (f.packet) {
	case PKT_PROCESS_SHORT_BUFFER:
		commCount_[static_cast<uint8_t>(f.comm)]++;
		if (!respond_) {
			break;
		}
		if (f.comm == COMM_FW_VERSION) {
			sendReply(f.data[0], fwVersionPayload());
		} else {
			sendReply(f.data[0], valuesPayload(f.u32(3)));
		}
		break;
	case PKT_SET_CURRENT_REL:
		currentSeen_ = true;
		lastCurrentRaw_ = f.i32(0);
		lastSetMs_ = f.timeMs; // only SET commands reset the timeout (SRC comm_can.c:1814-1822)
		break;
	case PKT_PING:
		if (respond_) {
			sendPong(f.data[0]);
		}
		break;
	case PKT_POLL_ROTOR_POS:
		if (respond_) {
			sendRotorPosition();
		}
		break;
	}
}

// ---------------------------------------------------------------------------------------------------------------
// records
// ---------------------------------------------------------------------------------------------------------------
std::vector<FakeVescPeer::Frame> FakeVescPeer::framesWithPacket(uint8_t packet) const {
	std::vector<Frame> r;
	for (const Frame &f : log_) {
		if (f.packet == packet && f.accepted) {
			r.push_back(f);
		}
	}
	return r;
}
size_t FakeVescPeer::count(uint8_t packet) const {
	return framesWithPacket(packet).size();
}
size_t FakeVescPeer::requestCount(uint8_t comm) const {
	auto it = commCount_.find(comm);
	return it == commCount_.end() ? 0 : it->second;
}

void FakeVescPeer::expectViolation(ViolationKind kind, int packet) {
	expected_.push_back({kind, packet});
}

std::vector<FakeVescPeer::Violation> FakeVescPeer::unexpectedViolations() const {
	std::vector<Violation> r;
	for (const Violation &v : violations_) {
		bool ok = false;
		for (const Expected &e : expected_) {
			if (e.kind == v.kind && (e.packet < 0 || e.packet == v.packet)) {
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
