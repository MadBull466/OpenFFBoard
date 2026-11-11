/*
 * FakeTmc4671.cpp
 *
 * See FakeTmc4671.h for what is modelled, the sources (TMC4671-LA datasheet Rev 2.07, "DS §x pNN"), what is not
 * modelled and the confidence ratings.
 */
#include "FakeTmc4671.h"

#include "HostPlatform.h"

#include <algorithm>
#include <sstream>

namespace {

std::vector<FakeTmc4671 *> &registry() {
	static std::vector<FakeTmc4671 *> r;
	return r;
}

/** Addresses listed in DS §7.1 p73-75 (register overview) with access "R" */
constexpr uint8_t READ_ONLY[] = {0x00, 0x02, 0x12, 0x13, 0x15, 0x16, 0x2A, 0x31, 0x39, 0x3A,
								 0x3D, 0x3F, 0x41, 0x46, 0x53, 0x69, 0x6A, 0x6C, 0x76, 0x77};

/** All addresses of DS §7.1 p73-75, plus 0x5F (PID_ACCELERATION_LIMIT, listed in DS §7.3 p122 only) */
constexpr uint8_t DOCUMENTED[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
	0x11, 0x12, 0x13, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1F, 0x20, 0x21, 0x22, 0x23,
	0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x33, 0x34, 0x35,
	0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x45, 0x46, 0x4D,
	0x4E, 0x50, 0x51, 0x52, 0x53, 0x54, 0x56, 0x58, 0x5A, 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63,
	0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x75, 0x76, 0x77, 0x78,
	0x79, 0x7B, 0x7C, 0x7D};

template <size_t N>
bool contains(const uint8_t (&arr)[N], uint8_t v) {
	return std::find(std::begin(arr), std::end(arr), v) != std::end(arr);
}

} // namespace

bool FakeTmc4671::isReadOnly(uint8_t addr) { return contains(READ_ONLY, addr & 0x7F); }
bool FakeTmc4671::isDocumented(uint8_t addr) { return contains(DOCUMENTED, addr & 0x7F); }

/**
 * Power-on defaults of DS §7.3 (p111-127). A register default is the composition of its field defaults at the bit
 * positions of DS §7.2. Everything not listed here has the default 0 (all fields 0, or no default row).
 */
uint32_t FakeTmc4671::defaultValue(uint8_t addr) {
	switch (addr & 0x7F) {
	case 0x05: return 0x0CCCCCCD;	// dsADC_MCLK_A, DS §7.3 p111
	case 0x06: return 0x0CCCCCCD;	// dsADC_MCLK_B
	case 0x07: return 0x01000100;	// MDEC_A [15:0] = 0x100, MDEC_B [31:16] = 0x100
	case 0x08: return 0x01000000;	// ADC_I1_SCALE s16 [31:16] = 0x100, offset 0
	case 0x09: return 0x01000000;	// ADC_I0_SCALE
	case 0x0A: return 0x24000100;	// I1_SELECT [15:8] = 1, V [27:26] = 1, WY [29:28] = 2: 1<<8 | 1<<26 | 2<<28
	case 0x0D: return 0x01000000;	// AENC_0_SCALE [31:16] = 0x100
	case 0x0E: return 0x01000000;
	case 0x0F: return 0x01000000;
	case 0x11: return 0x00020100;	// AENC_0_SELECT 0, _1 [15:8] = 1, _2 [23:16] = 2
	case 0x18: return 0x00000F9F;	// PWM_MAXCNT = 3999 (25 kHz)
	case 0x19: return 0x00001414;	// BBM_L [7:0] = 0x14, BBM_H [15:8] = 0x14
	case 0x1B: return 0x00000001;	// N_POLE_PAIRS = 1, MOTOR_TYPE 0
	case 0x26: return 0x00010000;	// ABN_DECODER_PPR
	case 0x2D: return 0x00010000;	// ABN_2_DECODER_PPR
	case 0x34: return 0x2AAA0000;	// HALL_POSITION_060 [31:16] = 0x2AAA, _000 = 0
	case 0x35: return 0x80005555;	// _180 [31:16] = -0x8000 (0x8000), _120 [15:0] = 0x5555
	case 0x36: return 0xD555AAAA;	// _300 [31:16] = -0x2AAB (0xD555), _240 [15:0] = -0x5556 (0xAAAA)
	case 0x38: return 0x00002AAA;	// HALL_DPHI_MAX
	case 0x40: return 0x00000001;	// AENC_DECODER_PPR
	case 0x5D: return 0x00005A81;	// PIDOUT_UQ_UD_LIMITS
	case 0x5E: return 0x00007FFF;	// PID_TORQUE_FLUX_LIMITS (u16 [15:0])
	case 0x5F: return 0x7FFFFFFF;	// PID_ACCELERATION_LIMIT (DS §7.3 only)
	case 0x60: return 0x7FFFFFFF;	// PID_VELOCITY_LIMIT
	case 0x61: return 0x80000001;	// PID_POSITION_LIMIT_LOW = -0x7FFFFFFF
	case 0x62: return 0x7FFFFFFF;	// PID_POSITION_LIMIT_HIGH
	case 0x75: return 0xFFFFFFFF;	// ADC_VM_LIMIT_LOW [15:0] and _HIGH [31:16] = 0xFFFF
	case 0x76: return 0x00008000;	// ENABLE_IN_RAW (bit 15) high: ASSUMPTION, see header (DS default 0)
	case 0x79: return 0x00009600;	// UART_BPS
	default: return 0;
	}
}

std::string FakeTmc4671::Violation::text() const {
	std::ostringstream s;
	switch (kind) {
	case ViolationKind::WrongFrameSize: s << "SPI transfer is not a 40 bit datagram"; break;
	case ViolationKind::WriteToReadOnly: s << "write to read-only register 0x" << std::hex << addr; break;
	case ViolationKind::NoSingleCsLow: s << "none or several chip selects low during a TMC4671 datagram"; break;
	}
	s << " (t=" << std::dec << timeUs << " us)";
	return s.str();
}

FakeTmc4671::FakeTmc4671(int csIndex) : csIndex(csIndex) {
	reset();
	auto &r = registry();
	r.push_back(this);
	std::sort(r.begin(), r.end(), [](const FakeTmc4671 *a, const FakeTmc4671 *b) { return a->csIndex < b->csIndex; });
	installResponder();
}

FakeTmc4671::~FakeTmc4671() {
	auto &r = registry();
	r.erase(std::remove(r.begin(), r.end(), this), r.end());
	if (r.empty()) {
		HostTarget::spiSetResponder(&hspi1, nullptr);
	}
}

void FakeTmc4671::installResponder() {
	HostTarget::spiSetResponder(&hspi1, [](const HostTarget::SpiTransfer &t) { return dispatch(t); });
}

void FakeTmc4671::reset() {
	for (uint32_t a = 0; a < regs_.size(); a++) {
		regs_[a] = defaultValue(static_cast<uint8_t>(a));
	}
	chipInfo_ = {};
	chipInfo_[0] = SI_TYPE_4671;
	chipInfo_[1] = SI_VERSION_V13;
	adcRaw_ = {};
	pidError_ = {};
	interim_ = {};
	config_.clear();
	configWrites_.clear();
	readHooks_.clear();
	writeHooks_.clear();
	log_.clear();
	violations_.clear();
	undocumented_.clear();
	expected_.clear();
	microsPerTransfer_ = 0;
	present_ = true;
}

// --- bus ---

std::vector<uint8_t> FakeTmc4671::dispatch(const HostTarget::SpiTransfer &t) {
	auto &r = registry();
	std::vector<uint8_t> zeros(t.size, 0);
	if (r.empty()) {
		return zeros;
	}
	int sel = t.selectedCs();
	if (sel < 0) {
		r.front()->violate(ViolationKind::NoSingleCsLow, -1, HostPlatform::nowUs());
		return zeros;
	}
	for (FakeTmc4671 *f : r) {
		if (f->csIndex == sel) {
			return f->handle(t, true);
		}
	}
	return zeros; // CS of another device
}

std::vector<uint8_t> FakeTmc4671::handle(const HostTarget::SpiTransfer &t, bool) {
	uint64_t now = HostPlatform::nowUs();
	std::vector<uint8_t> out(t.size, 0);
	if (microsPerTransfer_ != 0) {
		HostPlatform::advanceUs(microsPerTransfer_);
	}
	if (t.tx.size() != 5 || t.size != 5) {
		violate(ViolationKind::WrongFrameSize, -1, now);
		return out;
	}
	Frame f;
	f.write = (t.tx[0] & 0x80) != 0;
	f.addr = t.tx[0] & 0x7F;
	f.data = (uint32_t(t.tx[1]) << 24) | (uint32_t(t.tx[2]) << 16) | (uint32_t(t.tx[3]) << 8) | t.tx[4];
	f.timeUs = now;
	f.cs = csIndex;
	if (!f.write) {
		f.data = 0;
	}
	if (!present_) {
		log_.push_back(f);
		return out;
	}
	if (!isDocumented(f.addr)) {
		undocumented_.push_back({f.addr, f.write, f.data, now});
	}
	if (!f.write) {
		auto h = readHooks_.find(f.addr);
		if (h != readHooks_.end()) {
			ReadHook fn = h->second; // the hook may replace hooks
			fn(*this);
		}
	}
	f.answer = readValue(f.addr);
	if (f.write) {
		if (isReadOnly(f.addr)) {
			violate(ViolationKind::WriteToReadOnly, f.addr, now);
		} else {
			writeValue(f.addr, f.data);
		}
	}
	log_.push_back(f);
	if (f.write) {
		auto h = writeHooks_.find(f.addr);
		if (h != writeHooks_.end() && !isReadOnly(f.addr)) {
			WriteHook fn = h->second;
			fn(*this, f.data);
		}
	}
	out[1] = f.answer >> 24;
	out[2] = f.answer >> 16;
	out[3] = f.answer >> 8;
	out[4] = f.answer;
	return out;
}

uint32_t FakeTmc4671::readValue(uint8_t addr) const {
	switch (addr) {
	case 0x00: return regs_[0x01] < chipInfo_.size() ? chipInfo_[regs_[0x01]] : 0; // DS §7.1 p73
	case 0x02: return adcRaw_[regs_[0x03] & 3];
	case 0x4D: {
		auto it = config_.find(regs_[0x4E]);
		return it == config_.end() ? 0 : it->second;
	}
	case 0x6C: return pidError_[regs_[0x6D] & 0xFF];
	case 0x6E: return interim_[regs_[0x6F] & 0xFF];
	default: return regs_[addr & 0x7F];
	}
}

void FakeTmc4671::writeValue(uint8_t addr, uint32_t data) {
	switch (addr) {
	case 0x4D:
		config_[regs_[0x4E]] = data;
		configWrites_.emplace_back(regs_[0x4E], data);
		return;
	case 0x6E:
		interim_[regs_[0x6F] & 0xFF] = data;
		return;
	case 0x6B: // DS 0x6B p98: writes the same value into PID_POSITION_TARGET
		regs_[0x6B] = data;
		regs_[0x68] = data;
		return;
	case 0x7C: // assumption: written 0 bits clear the flags (a write of 0 clears all)
		regs_[0x7C] &= data;
		return;
	default:
		regs_[addr & 0x7F] = data;
	}
}

void FakeTmc4671::violate(ViolationKind kind, int addr, uint64_t timeUs) {
	violations_.push_back({kind, addr, timeUs});
}

// --- test side ---

uint32_t FakeTmc4671::reg(uint8_t addr) const { return readValue(addr & 0x7F); }

void FakeTmc4671::setReg(uint8_t addr, uint32_t value) {
	addr &= 0x7F;
	switch (addr) {
	case 0x00: chipInfo_[std::min<uint32_t>(regs_[0x01], 5)] = value; return;
	case 0x02: adcRaw_[regs_[0x03] & 3] = value; return;
	case 0x4D: config_[regs_[0x4E]] = value; return;
	case 0x6C: pidError_[regs_[0x6D] & 0xFF] = value; return;
	case 0x6E: interim_[regs_[0x6F] & 0xFF] = value; return;
	default: regs_[addr] = value;
	}
}

uint32_t FakeTmc4671::configWord(uint32_t n) const {
	auto it = config_.find(n);
	return it == config_.end() ? 0 : it->second;
}
void FakeTmc4671::setConfigWord(uint32_t n, uint32_t value) { config_[n] = value; }
void FakeTmc4671::setAdcRaw(uint8_t sel, uint32_t word) { adcRaw_[sel & 3] = word; }
void FakeTmc4671::setChipInfo(uint8_t sel, uint32_t word) { chipInfo_[std::min<uint8_t>(sel, 5)] = word; }
void FakeTmc4671::setPidError(uint8_t sel, uint32_t word) { pidError_[sel] = word; }
void FakeTmc4671::setInterim(uint8_t sel, uint32_t word) { interim_[sel] = word; }
void FakeTmc4671::raiseFlags(uint32_t mask) { regs_[0x7C] |= mask; }
void FakeTmc4671::setEnableIn(bool high) { regs_[0x76] = high ? (regs_[0x76] | 0x8000u) : (regs_[0x76] & ~0x8000u); }
void FakeTmc4671::setNPulse(bool high) { regs_[0x76] = high ? (regs_[0x76] | 0x4u) : (regs_[0x76] & ~0x4u); }

void FakeTmc4671::onRead(uint8_t addr, ReadHook fn) { readHooks_[addr & 0x7F] = std::move(fn); }
void FakeTmc4671::onWrite(uint8_t addr, WriteHook fn) { writeHooks_[addr & 0x7F] = std::move(fn); }
void FakeTmc4671::clearHooks() {
	readHooks_.clear();
	writeHooks_.clear();
}

std::vector<uint32_t> FakeTmc4671::writesTo(uint8_t addr) const {
	std::vector<uint32_t> v;
	for (const Frame &f : log_) {
		if (f.write && f.addr == addr) {
			v.push_back(f.data);
		}
	}
	return v;
}

size_t FakeTmc4671::readsOf(uint8_t addr) const {
	size_t n = 0;
	for (const Frame &f : log_) {
		if (!f.write && f.addr == addr) {
			n++;
		}
	}
	return n;
}

void FakeTmc4671::expectViolation(ViolationKind kind, int addr) { expected_.push_back({kind, addr}); }

std::vector<FakeTmc4671::Violation> FakeTmc4671::unexpectedViolations() const {
	std::vector<Violation> out;
	for (const Violation &v : violations_) {
		bool ok = std::any_of(expected_.begin(), expected_.end(), [&](const Expected &e) { return e.kind == v.kind && (e.addr < 0 || e.addr == v.addr); });
		if (!ok) {
			out.push_back(v);
		}
	}
	return out;
}
