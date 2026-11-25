/*
 * CaseWatchdog: see CaseWatchdog.h (per-case time limit of every doctest executable of the suite, tool WOs I2-T1 / I2-T2).
 */
#include "CaseWatchdog.h"

#include "doctest.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>

namespace ffb_it {

int caseTimeoutSecondsFrom(const char* envValue, bool* ok) {
	if (ok) {
		*ok = true;
	}
	if (envValue == nullptr || *envValue == '\0') {
		return kDefaultCaseTimeoutS;
	}
	char* end = nullptr;
	errno = 0;
	const long v = std::strtol(envValue, &end, 10);
	// digits only (strtol also accepts a sign and leading blanks), a plain int
	bool digits = true;
	for (const char* p = envValue; *p; ++p) {
		digits = digits && *p >= '0' && *p <= '9';
	}
	if (!digits || end == envValue || *end != '\0' || errno == ERANGE || v < 0 || v > 86400L * 30) {
		if (ok) {
			*ok = false;
		}
		return kDefaultCaseTimeoutS;
	}
	return static_cast<int>(v);
}

void exitOnCaseTimeout(const CaseWatchdogEvent& e) {
	std::fprintf(stderr, "CASE TIMEOUT %lld s: %s (%s:%u)\n", static_cast<long long>(e.limit.count() / 1000), e.name.c_str(),
	             e.file.c_str(), e.line);
	std::fflush(stderr);
	std::fflush(stdout);
	std::_Exit(kCaseTimeoutExitCode);
}

CaseWatchdog::CaseWatchdog(std::chrono::milliseconds limit, std::function<void(const CaseWatchdogEvent&)> action)
	: limit_(limit), action_(std::move(action)), event_{std::string(), std::string(), 0, limit} {
	if (limit_.count() > 0) {
		thread_ = std::thread([this] { loop(); });
	}
}

CaseWatchdog::~CaseWatchdog() {
	{
		std::lock_guard<std::mutex> lk(m_);
		stop_ = true;
	}
	cv_.notify_all();
	if (thread_.joinable()) {
		thread_.join();
	}
}

void CaseWatchdog::arm(const std::string& name, const std::string& file, unsigned line) {
	if (limit_.count() <= 0) {
		return;
	}
	{
		std::lock_guard<std::mutex> lk(m_);
		event_ = CaseWatchdogEvent{name, file, line, limit_};
		deadline_ = std::chrono::steady_clock::now() + limit_;
		armed_ = true;
		++generation_;
	}
	cv_.notify_all();
}

void CaseWatchdog::disarm() {
	{
		std::lock_guard<std::mutex> lk(m_);
		armed_ = false;
		++generation_;
	}
	cv_.notify_all();
}

bool CaseWatchdog::armed() const {
	std::lock_guard<std::mutex> lk(m_);
	return armed_;
}

void CaseWatchdog::loop() {
	std::unique_lock<std::mutex> lk(m_);
	while (!stop_) {
		if (!armed_) {
			cv_.wait(lk, [this] { return stop_ || armed_; });
			continue;
		}
		const unsigned long long gen = generation_;
		const auto deadline = deadline_;
		if (cv_.wait_until(lk, deadline, [this, gen] { return stop_ || generation_ != gen; })) {
			continue;  // disarmed, re-armed or stopped while waiting
		}
		if (armed_ && generation_ == gen) {
			armed_ = false;
			const CaseWatchdogEvent e = event_;
			lk.unlock();
			action_(e);  // the default action does not return
			lk.lock();
		}
	}
}

namespace {

/// The doctest listener of every executable: one watchdog for the run, armed per test case.
struct CaseWatchdogListener : doctest::IReporter {
	CaseWatchdog dog;

	explicit CaseWatchdogListener(const doctest::ContextOptions&) : dog(std::chrono::seconds(limitFromEnvironment())) {}

	static int limitFromEnvironment() {
		bool ok = true;
		const int s = caseTimeoutSecondsFrom(std::getenv(kCaseTimeoutEnv), &ok);
		if (!ok) {
			std::fprintf(stderr, "%s is not a non-negative integer number of seconds: using the default of %d s\n", kCaseTimeoutEnv,
			             kDefaultCaseTimeoutS);
		}
		return s;
	}

	void report_query(const doctest::QueryData&) override {}
	void test_run_start() override {}
	void test_run_end(const doctest::TestRunStats&) override { dog.disarm(); }
	void test_case_start(const doctest::TestCaseData& tc) override {
		dog.arm(tc.m_name ? tc.m_name : "", tc.m_file.c_str(), tc.m_line);
	}
	void test_case_reenter(const doctest::TestCaseData&) override {}  // the deadline of the case stays: the limit is per case
	void test_case_end(const doctest::CurrentTestCaseStats&) override { dog.disarm(); }
	void test_case_exception(const doctest::TestCaseException&) override {}
	void subcase_start(const doctest::SubcaseSignature&) override {}
	void subcase_end() override {}
	void log_assert(const doctest::AssertData&) override {}
	void log_message(const doctest::MessageData&) override {}
	void test_case_skipped(const doctest::TestCaseData&) override {}
};

}  // namespace

}  // namespace ffb_it

REGISTER_LISTENER("case_watchdog", 1, ffb_it::CaseWatchdogListener);
