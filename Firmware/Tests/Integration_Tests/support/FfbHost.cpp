/*
 * FfbHost.cpp - see FfbHost.h
 */
#include "FfbHost.h"

#include "doctest.h"

#include <algorithm>
#include <cerrno>
#include <cstdlib>

using hid::ReportType;

namespace {
constexpr uint8_t kFeatureType = 3; // HID 1.11 7.2.1 / 7.2.2: report type 3 = Feature
}

FfbHost::FfbHost(IntegrationFixture &fixture) : fx_(fixture), builder_(desc_) {
	descBytes_ = fx_.readHidReportDescriptor();
	INFO("report descriptor parse error: " << desc_.error());
	REQUIRE(desc_.parse(descBytes_));
}

// --- feature reports ---

void FfbHost::setFeature(const pid::PidReport &report) {
	REQUIRE(report.valid());
	INFO("report " << report.name());
	REQUIRE(report.errors().empty());
	beforeTransfer();
	const size_t mark = logMark();
	const UsbHost::ControlResult r = fx_.host.hidSetReport(kFeatureType, report.reportId(), report.bytes());
	INFO("SET_REPORT(feature) of '" << report.name() << "' ended at stage '" << r.stage << "'");
	REQUIRE(r.ok());
	fx_.sched.pumpUsb();
	afterPump(mark);
	fx_.drainHidIn();
}

std::vector<uint8_t> FfbHost::getFeature(const hid::Report &report) {
	beforeTransfer();
	const UsbHost::ControlResult r = fx_.host.hidGetReport(kFeatureType, report.reportId, static_cast<uint16_t>(report.byteLength()));
	INFO("GET_REPORT(feature id " << static_cast<unsigned>(report.reportId) << ") ended at stage '" << r.stage << "'");
	REQUIRE(r.ok());
	REQUIRE(!r.data.empty());
	return r.data;
}

FfbHost::CreateResult FfbHost::createEffect(uint32_t effectTypeUsage, int64_t byteCount) {
	pid::PidReport r = builder_.make(ReportType::Feature, pid::use::CreateNewEffectReport);
	r.select(effectTypeUsage);
	if (byteCount >= 0) {
		r.set(hid::usage(0x01, 0x3B), byteCount); // Generic Desktop 0x3B "Byte Count" as the descriptor declares it
	}
	setFeature(r);
	CreateResult res;
	res.effectType = effectTypeUsage;
	res.load = getBlockLoad();
	return res;
}

FfbHost::BlockLoad FfbHost::getBlockLoad() {
	const hid::Report *rep = builder_.find(ReportType::Feature, pid::use::PidBlockLoadReport);
	REQUIRE(rep != nullptr);
	BlockLoad b = decodeBlockLoad(builder_, getFeature(*rep));
	INFO("Block Load decode errors: " << (b.errors.empty() ? std::string("none") : b.errors.front()));
	REQUIRE(b.decoded);
	return b;
}

FfbHost::PoolResult FfbHost::getPool() {
	const hid::Report *rep = builder_.find(ReportType::Feature, pid::use::PidPoolReport);
	REQUIRE(rep != nullptr);
	PoolResult p = decodePool(builder_, getFeature(*rep));
	INFO("Pool decode errors: " << (p.errors.empty() ? std::string("none") : p.errors.front()));
	REQUIRE(p.decoded);
	return p;
}

// --- output reports ---

HostDcd::Handshake FfbHost::sendRaw(const std::vector<uint8_t> &bytes) {
	beforeTransfer();
	const size_t mark = logMark();
	const HostDcd::Handshake h = fx_.host.hidWrite(bytes);
	fx_.sched.pumpUsb();
	afterPump(mark);
	fx_.drainHidIn();
	return h;
}

void FfbHost::send(const pid::PidReport &report) {
	REQUIRE(report.valid());
	INFO("report " << report.name());
	REQUIRE(report.errors().empty());
	const HostDcd::Handshake h = sendRaw(report.bytes());
	REQUIRE(h == HostDcd::Handshake::Ack);
}

void FfbHost::afterPump(size_t mark) {
	const std::vector<FakeTmc4671::Frame> &log = fx_.chip1.log();
	for (size_t i = mark; i < log.size(); i++) {
		if (log[i].write && log[i].addr == TorqueTrace::kRegTorqueFluxTarget) {
			pumpWrites_++;
		}
	}
	if (trace_ != nullptr) {
		trace_->skipLog();
	}
}

// --- state capture ---

std::vector<FfbHost::PidState> FfbHost::states() {
	fx_.drainHidIn();
	std::vector<PidState> out;
	const hid::Report *rep = builder_.find(ReportType::Input, pid::use::PidStateReport);
	if (rep == nullptr) {
		return out;
	}
	for (const IntegrationFixture::HidInReport &r : fx_.hidInReports) {
		if (!r.bytes.empty() && r.bytes[0] == rep->reportId) {
			out.push_back(decodeState(builder_, r.bytes, r.timeUs));
		}
	}
	return out;
}

size_t FfbHost::stateMark() { return states().size(); }

std::vector<FfbHost::PidState> FfbHost::statesSince(size_t mark) {
	std::vector<PidState> all = states();
	if (mark >= all.size()) {
		return {};
	}
	return std::vector<PidState>(all.begin() + static_cast<std::ptrdiff_t>(mark), all.end());
}

bool FfbHost::latestState(PidState &out) {
	const std::vector<PidState> all = states();
	if (all.empty()) {
		return false;
	}
	out = all.back();
	return true;
}

void FfbHost::run(uint32_t ms) {
	for (uint32_t i = 0; i < ms; i++) {
		fx_.sched.advanceMs(1);
		fx_.drainHidIn();
	}
}

// --- CDC ---

bool FfbHost::replyValueText(const std::string &reply, std::string &valueText) {
	if (reply.size() < 4 || reply.front() != '[') {
		return false;
	}
	const size_t close = reply.rfind(']');
	const size_t bar = reply.rfind('|', close == std::string::npos ? std::string::npos : close);
	if (close == std::string::npos || bar == std::string::npos || bar > close) {
		return false;
	}
	valueText = reply.substr(bar + 1, close - bar - 1);
	return true;
}

std::string FfbHost::command(const std::string &line) {
	beforeTransfer();
	const std::string reply = fx_.cdcCommand(line);
	INFO("command '" << line << "' reply '" << reply << "'");
	REQUIRE(!reply.empty());
	REQUIRE(reply.back() == '\n');
	std::string value;
	REQUIRE(replyValueText(reply, value));
	REQUIRE(value != "ERR");
	REQUIRE(value != "NOT_FOUND");
	REQUIRE(value != "None");
	return reply;
}

int64_t FfbHost::getValue(const std::string &line) {
	const std::string reply = command(line);
	std::string value;
	REQUIRE(replyValueText(reply, value));
	INFO("value text '" << value << "' of reply '" << reply << "'");
	REQUIRE(!value.empty());
	char *end = nullptr;
	errno = 0;
	const long long v = std::strtoll(value.c_str(), &end, 10);
	REQUIRE(errno == 0);
	REQUIRE(*end == '\0');
	return static_cast<int64_t>(v);
}

// --- decoders ---

FfbHost::BlockLoad FfbHost::decodeBlockLoad(const pid::PidReportBuilder &b, const std::vector<uint8_t> &bytes) {
	BlockLoad out;
	out.raw = bytes;
	const hid::Report *rep = b.find(ReportType::Feature, pid::use::PidBlockLoadReport);
	if (rep == nullptr) {
		out.errors.push_back("the descriptor declares no PID Block Load feature report");
		return out;
	}
	const pid::DecodedReport d = b.decode(*rep, bytes);
	out.errors = d.errors;
	if (!d.errors.empty()) {
		return out;
	}
	for (const pid::DecodedValue &v : d.values) {
		const hid::Field &f = *v.field;
		if (f.hasUsage(pid::use::EffectBlockIndex) && f.isVariable() && !out.indexDeclared) {
			out.indexDeclared = true;
			out.effectBlockIndex = v.value;
		} else if (f.hasUsage(pid::use::BlockLoadSuccess) && !out.statusDeclared) {
			out.statusDeclared = true;
			out.status = v.value;
			for (uint32_t u : {pid::use::BlockLoadSuccess, pid::use::BlockLoadFull, pid::use::BlockLoadError}) {
				int64_t sel = 0;
				if (f.selectorValue(u, sel) && sel == v.value) {
					out.statusUsage = u;
				}
			}
		} else if (f.hasUsage(pid::use::RamPoolAvailable) && !out.ramPoolDeclared) {
			out.ramPoolDeclared = true;
			out.ramPoolAvailable = v.value;
			out.ramPoolAvailableIsArray = !f.isVariable();
			out.ramPoolAvailableBits = f.bitSize;
		}
	}
	if (!out.indexDeclared) {
		out.errors.push_back("no Effect Block Index (Variable) field in PID Block Load");
	}
	if (!out.statusDeclared) {
		out.errors.push_back("no Block Load Status field in PID Block Load");
	}
	if (!out.ramPoolDeclared) {
		out.errors.push_back("no RAM Pool Available field in PID Block Load");
	}
	out.decoded = out.errors.empty();
	return out;
}

FfbHost::PoolResult FfbHost::decodePool(const pid::PidReportBuilder &b, const std::vector<uint8_t> &bytes) {
	PoolResult out;
	out.raw = bytes;
	const hid::Report *rep = b.find(ReportType::Feature, pid::use::PidPoolReport);
	if (rep == nullptr) {
		out.errors.push_back("the descriptor declares no PID Pool feature report");
		return out;
	}
	const pid::DecodedReport d = b.decode(*rep, bytes);
	out.errors = d.errors;
	if (!d.errors.empty()) {
		return out;
	}
	out.hasRamPoolSize = d.get(pid::use::RamPoolSize, out.ramPoolSize);
	out.hasSimultaneousEffectsMax = d.get(pid::use::SimultaneousEffectsMax, out.simultaneousEffectsMax);
	out.hasDeviceManagedPool = d.get(pid::use::DeviceManagedPool, out.deviceManagedPool);
	out.hasSharedParameterBlocks = d.get(pid::use::SharedParameterBlocks, out.sharedParameterBlocks);
	out.decoded = true;
	return out;
}

FfbHost::PidState FfbHost::decodeState(const pid::PidReportBuilder &b, const std::vector<uint8_t> &bytes, uint64_t timeUs, bool failTests) {
	PidState out;
	out.raw = bytes;
	out.timeUs = timeUs;
	const hid::Report *rep = b.find(ReportType::Input, pid::use::PidStateReport);
	if (rep == nullptr) {
		out.errors.push_back("the descriptor declares no PID State input report");
	} else {
		const pid::DecodedReport d0 = b.decode(*rep, bytes);
		out.errors = d0.errors;
	}
	if (!out.errors.empty()) {
		if (failTests) {
			FAIL_CHECK("PID State decode: " << out.errors.front());
		}
		return out;
	}
	const pid::DecodedReport d = b.decode(*rep, bytes);
	int64_t v = 0;
	if (d.get(pid::use::DevicePaused, v)) {
		out.hasDevicePaused = true;
		out.devicePaused = v != 0;
	}
	if (d.get(pid::use::ActuatorsEnabled, v)) {
		out.hasActuatorsEnabled = true;
		out.actuatorsEnabled = v != 0;
	}
	if (d.get(pid::use::SafetySwitch, v)) {
		out.hasSafetySwitch = true;
		out.safetySwitch = v != 0;
	}
	if (d.get(pid::use::ActuatorPower, v)) {
		out.hasActuatorPower = true;
		out.actuatorPower = v != 0;
	}
	if (d.get(pid::use::EffectPlaying, v)) {
		out.hasEffectPlaying = true;
		out.effectPlaying = v != 0;
	}
	return out;
}

// --- TorqueTrace ---

TorqueTrace::~TorqueTrace() { stop(); }

void TorqueTrace::startManual(uint32_t ffbCycles) {
	REQUIRE_MESSAGE(!running_, "TorqueTrace::start: already running");
	samples_.clear();
	strayWrites_ = 0;
	missed_ = 0;
	pumpWrites_ = 0;
	lastCycles_ = ffbCycles;
	logMark_ = chip_.log().size();
	running_ = true;
}

void TorqueTrace::start(IntegrationFixture &fixture) {
	REQUIRE_MESSAGE(!fixture.sched.stepEndHook, "TorqueTrace::start: the scheduler already has an observer (one trace at a time)");
	startManual(fixture.ffbCycles());
	sched_ = &fixture.sched;
	IntegrationFixture *fx = &fixture;
	sched_->stepEndHook = [this, fx] { onStep(fx->ffbCycles(), HostPlatform::nowUs()); };
}

void TorqueTrace::stop() {
	if (sched_ != nullptr) {
		sched_->stepEndHook = nullptr;
		sched_ = nullptr;
	}
	running_ = false;
}

void TorqueTrace::onStep(uint32_t ffbCycles, uint64_t nowUs) {
	if (!running_) {
		return;
	}
	const std::vector<FakeTmc4671::Frame> &log = chip_.log();
	if (logMark_ > log.size()) {
		logMark_ = 0; // the log was cleared
	}
	uint32_t writes = 0;
	for (size_t i = logMark_; i < log.size(); i++) {
		if (log[i].write && log[i].addr == kRegTorqueFluxTarget) {
			writes++;
		}
	}
	logMark_ = log.size();
	if (ffbCycles == lastCycles_) {
		strayWrites_ += writes;
		return;
	}
	if (ffbCycles > lastCycles_ + 1) {
		missed_ += ffbCycles - lastCycles_ - 1;
	}
	lastCycles_ = ffbCycles;
	const uint32_t r = chip_.reg(kRegTorqueFluxTarget);
	Sample s;
	s.cycle = ffbCycles;
	s.timeUs = nowUs;
	s.flux = static_cast<int16_t>(r & 0xFFFF);
	s.torque = static_cast<int16_t>((r >> 16) & 0xFFFF);
	s.writes = writes;
	samples_.push_back(s);
}

const TorqueTrace::Sample *TorqueTrace::find(uint32_t cycle) const {
	for (const Sample &s : samples_) {
		if (s.cycle == cycle) {
			return &s;
		}
	}
	return nullptr;
}

int16_t TorqueTrace::torqueAt(uint32_t cycle) const {
	const Sample *s = find(cycle);
	INFO("TorqueTrace: no sample for FFB cycle " << cycle);
	REQUIRE(s != nullptr);
	return s->torque;
}

int16_t TorqueTrace::fluxAt(uint32_t cycle) const {
	const Sample *s = find(cycle);
	INFO("TorqueTrace: no sample for FFB cycle " << cycle);
	REQUIRE(s != nullptr);
	return s->flux;
}

namespace {
// No REQUIRE inside: used by the INFO-free queries (a REQUIRE reachable from an INFO expression deadlocks doctest's reporter)
template <typename Get>
bool steadyOf(const TorqueTrace &t, uint32_t from, uint32_t to, Get get, int16_t &value, std::string &why) {
	if (from > to) {
		why = "fromCycle > toCycle";
		return false;
	}
	bool first = true;
	for (uint32_t c = from; c <= to; c++) {
		const TorqueTrace::Sample *s = t.find(c);
		if (s == nullptr) {
			why = "no sample for FFB cycle " + std::to_string(c);
			return false;
		}
		if (first) {
			value = get(*s);
			first = false;
		} else if (get(*s) != value) {
			why = "cycle " + std::to_string(c) + " has " + std::to_string(get(*s)) + " instead of " + std::to_string(value) + " of cycle " + std::to_string(from);
			return false;
		}
	}
	return true;
}
} // namespace

bool TorqueTrace::steadyQuery(uint32_t fromCycle, uint32_t toCycle, int16_t &value, std::string &why) const {
	return steadyOf(*this, fromCycle, toCycle, [](const Sample &s) { return s.torque; }, value, why);
}

bool TorqueTrace::steadyFluxQuery(uint32_t fromCycle, uint32_t toCycle, int16_t &value, std::string &why) const {
	return steadyOf(*this, fromCycle, toCycle, [](const Sample &s) { return s.flux; }, value, why);
}

int16_t TorqueTrace::steadyValue(uint32_t fromCycle, uint32_t toCycle) const {
	int16_t v = 0;
	std::string why;
	const bool ok = steadyQuery(fromCycle, toCycle, v, why);
	INFO("TorqueTrace::steadyValue: " << why);
	REQUIRE(ok);
	return v;
}

int16_t TorqueTrace::steadyFlux(uint32_t fromCycle, uint32_t toCycle) const {
	int16_t v = 0;
	std::string why;
	const bool ok = steadyFluxQuery(fromCycle, toCycle, v, why);
	INFO("TorqueTrace::steadyFlux: " << why);
	REQUIRE(ok);
	return v;
}

void TorqueTrace::skipLog() {
	const std::vector<FakeTmc4671::Frame> &log = chip_.log();
	if (logMark_ > log.size()) {
		logMark_ = 0;
	}
	for (size_t i = logMark_; i < log.size(); i++) {
		if (log[i].write && log[i].addr == kRegTorqueFluxTarget) {
			pumpWrites_++;
		}
	}
	logMark_ = log.size();
}

uint32_t TorqueTrace::writesPerCycleMax() const {
	uint32_t m = 0;
	for (const Sample &s : samples_) {
		m = std::max(m, s.writes);
	}
	return m;
}
