/*
 * ChipEventLog.cpp: see ChipEventLog.h.
 */
#include "ChipEventLog.h"

#include "main.h"
#include "task.h"

ChipEventLog::ChipEventLog(IntegrationFixture &fx, FakeTmc4671 &chip) : fx_(&fx), chip_(&chip), alive_(std::make_shared<bool>(true)) {
	fx_->registerEventLog(this);
	lastLevel_ = pinLevel();
	chipLogBase_ = chip_->log().size();
	stepsBase_ = fx_->sched.stats().steps;
	presentAtAttach_ = chip_->present();
	install();
	// Chain to the observer that was there (see the header, Coexistence)
	std::function<void()> prev = fx_->sched.stepEndHook;
	std::weak_ptr<bool> w = alive_;
	fx_->sched.stepEndHook = [this, w, prev] {
		if (!w.expired()) {
			stepEnd();
		}
		if (prev) {
			prev();
		}
	};
}

ChipEventLog::~ChipEventLog() {
	detachFromFixture();
	alive_.reset();
}

void ChipEventLog::detachFromFixture() {
	if (fx_ != nullptr) {
		fx_->unregisterEventLog(this);
		fx_ = nullptr;
		chip_->clearHooks();
	}
}

bool ChipEventLog::pinLevel() const { return HostTarget::outputPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin); }

void ChipEventLog::install() {
	for (unsigned a = 0; a < 128; a++) {
		const uint8_t addr = static_cast<uint8_t>(a);
		chip_->onWrite(addr, [this, addr](FakeTmc4671 &c, uint32_t data) { recordWrite(c, addr, data); });
		chip_->onRead(addr, [this, addr](FakeTmc4671 &c) { readHook(c, addr); });
	}
}

void ChipEventLog::onReboot() {
	events_.clear();
	userWrite_.clear();
	userRead_.clear();
	fail_ = Armed();
	failedTransfer_ = -1;
	cmdState_ = CommandState::Idle;
	cmdRuns_ = 0;
	cmdEvent_ = npos;
	cmdReply_.clear();
	cmdWritesDuring_ = 0;
	lastLevel_ = pinLevel();
	chipLogBase_ = chip_->log().size();
	stepsSeen_ = 0;
	stepsBase_ = fx_->sched.stats().steps;
	presentAtAttach_ = chip_->present();
	install();
}

void ChipEventLog::faultOnce(const std::string &what) { fx_->reportFixtureFault("ChipEventLog: " + what); }

void ChipEventLog::recordWrite(FakeTmc4671 &chip, uint8_t addr, uint32_t data) {
	Event e;
	e.kind = Kind::Write;
	e.timeUs = chip.log().empty() ? HostPlatform::nowUs() : chip.log().back().timeUs;
	e.addr = addr;
	e.data = data;
	e.chipFrame = chip.log().empty() ? 0 : chip.log().size() - 1;
	e.enable = pinLevel();
	events_.push_back(e);
	if (cmdState_ == CommandState::Sent) {
		cmdWritesDuring_++;
	}
	auto u = userWrite_.find(addr);
	if (u != userWrite_.end()) {
		FakeTmc4671::WriteHook fn = u->second;
		fn(chip, data);
	}
	if (fail_.active && fail_.onWrite && fail_.addr == addr) {
		if (fail_.timeoutAfter) {
			HostTarget::spiFailNextBlocking(&hspi1, HAL_TIMEOUT);
			failedTransfer_ = static_cast<int64_t>(HostTarget::spiTransfers(&hspi1).size());
		} else {
			hspi1.ErrorCode = HAL_SPI_ERROR_FLAG;
			failedTransfer_ = static_cast<int64_t>(HostTarget::spiTransfers(&hspi1).size()) - 1;
		}
		fail_.active = false;
	}
	if (cmdState_ == CommandState::Armed && cmdAddr_ == addr) {
		cmdState_ = CommandState::Fired;
		cmdEvent_ = events_.size() - 1;
	}
}

void ChipEventLog::readHook(FakeTmc4671 &chip, uint8_t addr) {
	if (fail_.active && !fail_.onWrite && fail_.addr == addr) {
		if (fail_.timeoutAfter) {
			HostTarget::spiFailNextBlocking(&hspi1, HAL_TIMEOUT);
			failedTransfer_ = static_cast<int64_t>(HostTarget::spiTransfers(&hspi1).size());
		} else {
			hspi1.ErrorCode = HAL_SPI_ERROR_FLAG;
			failedTransfer_ = static_cast<int64_t>(HostTarget::spiTransfers(&hspi1).size()) - 1;
		}
		fail_.active = false;
	}
	auto u = userRead_.find(addr);
	if (u != userRead_.end()) {
		FakeTmc4671::ReadHook fn = u->second;
		fn(chip);
	}
}

void ChipEventLog::stepEnd() {
	stepsSeen_++;
	const bool level = pinLevel();
	if (level != lastLevel_) {
		Event e;
		e.kind = Kind::Enable;
		e.timeUs = HostPlatform::nowUs();
		e.enable = level;
		events_.push_back(e);
		lastLevel_ = level;
	}
	if (cmdState_ == CommandState::Fired) {
		REQUIRE(fx_->host.cdcWrite(cmdLine_ + "\n") == HostDcd::Handshake::Ack);
		cmdState_ = CommandState::Sent;
		cmdRuns_++;
	} else if (cmdState_ == CommandState::Sent) {
		if (fx_->drainHid) {
			fx_->drainHidIn();
		}
		cmdReply_ += fx_->host.cdcRead();
		if (!cmdReply_.empty() && cmdReply_.back() == '\n') {
			cmdState_ = CommandState::Done;
		}
	}
}

bool ChipEventLog::complete() const {
	if (!hookFault().empty()) {
		return false;
	}
	size_t expected = 0;
	const auto &log = chip_->log();
	for (size_t i = chipLogBase_; i < log.size(); i++) {
		if (log[i].write && !FakeTmc4671::isReadOnly(log[i].addr) && presentAtAttach_) {
			expected++;
		}
	}
	return countAllWrites() == expected;
}

size_t ChipEventLog::countAllWrites() const {
	size_t n = 0;
	for (const Event &e : events_) {
		n += e.kind == Kind::Write ? 1 : 0;
	}
	return n;
}

size_t ChipEventLog::firstWrite(uint8_t addr, const DataPredicate &pred, size_t from) const {
	for (size_t i = from; i < events_.size(); i++) {
		const Event &e = events_[i];
		if (e.kind == Kind::Write && e.addr == addr && (!pred || pred(e.data))) {
			return i;
		}
	}
	return npos;
}

size_t ChipEventLog::firstEnable(bool level, size_t from) const {
	for (size_t i = from; i < events_.size(); i++) {
		if (events_[i].kind == Kind::Enable && events_[i].enable == level) {
			return i;
		}
	}
	return npos;
}

bool ChipEventLog::fluxTorqueZeroBefore(size_t i) const {
	for (size_t k = 0; k < events_.size() && k < i; k++) {
		if (events_[k].kind == Kind::Write && events_[k].addr == 0x64 && events_[k].data != 0) {
			return false; // flux [15:0] and torque [31:16] both zero <=> the 32 bit word is zero
		}
	}
	return true;
}

size_t ChipEventLog::countWrites(uint8_t addr, size_t i) const {
	size_t n = 0;
	for (size_t k = 0; k < events_.size() && k < i; k++) {
		n += (events_[k].kind == Kind::Write && events_[k].addr == addr) ? 1 : 0;
	}
	return n;
}

std::optional<uint32_t> ChipEventLog::valueAt(uint8_t addr, size_t i) const {
	std::optional<uint32_t> v;
	for (size_t k = 0; k < events_.size() && k <= i; k++) {
		if (events_[k].kind == Kind::Write && events_[k].addr == addr) {
			v = events_[k].data;
		}
	}
	return v;
}

bool ChipEventLog::enableAt(size_t i) const {
	if (events_.empty()) {
		return lastLevel_;
	}
	return events_[i < events_.size() ? i : events_.size() - 1].enable;
}

void ChipEventLog::onWrite(uint8_t addr, FakeTmc4671::WriteHook fn) { userWrite_[addr & 0x7F] = std::move(fn); }
void ChipEventLog::onRead(uint8_t addr, FakeTmc4671::ReadHook fn) { userRead_[addr & 0x7F] = std::move(fn); }

void ChipEventLog::armSpiFailure(uint8_t addr, bool onWrite, HAL_StatusTypeDef status) {
	if (status != HAL_ERROR) {
		faultOnce("armSpiFailure: only HAL_ERROR can be bound to the frame content (see the header); use armSpiTimeoutAfter for HAL_TIMEOUT");
		return;
	}
	fail_ = {true, onWrite, static_cast<uint8_t>(addr & 0x7F), false};
	failedTransfer_ = -1;
}

void ChipEventLog::armSpiTimeoutAfter(uint8_t addr, bool onWrite) {
	fail_ = {true, onWrite, static_cast<uint8_t>(addr & 0x7F), true};
	failedTransfer_ = -1;
}

void ChipEventLog::runCommandOnWrite(uint8_t addr, const std::string &cdcLine) {
	cmdState_ = CommandState::Armed;
	cmdAddr_ = addr & 0x7F;
	cmdLine_ = cdcLine;
	cmdRuns_ = 0;
	cmdEvent_ = npos;
	cmdReply_.clear();
	cmdWritesDuring_ = 0;
}

std::string ChipEventLog::hookFault() const {
	if (fx_ == nullptr) {
		return {};
	}
	if (stepsSeen_ != fx_->sched.stats().steps - stepsBase_) {
		return "ChipEventLog: step sampling lost: " + std::to_string(fx_->sched.stats().steps - stepsBase_) + " scheduler steps since attach, " + std::to_string(stepsSeen_) +
			   " calls of the log's step hook (sched.stepEndHook was replaced, e.g. by a TorqueTrace that started or stopped after the log attached); enable events and a pending runCommandOnWrite are lost";
	}
	if (chip_->present() != presentAtAttach_) {
		return "ChipEventLog: the chip's presence changed since attach (frames of an absent chip run no hook)";
	}
	return {};
}
