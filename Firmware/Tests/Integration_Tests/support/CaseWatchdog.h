/*
 * CaseWatchdog (tool WO I2-T1): a time limit for every single test case of ffb_integration_tests instead of a limit for the
 * whole run (owner's decision 2026-11-24: "Do not set a time limit on the overall test runs ... Have time limits for each test
 * case").
 *
 * What it is: a doctest listener (REGISTER_LISTENER in CaseWatchdog.cpp, linked into ffb_integration_tests only). On
 * test_case_start it arms a host-clock deadline for the case, on test_case_end it disarms it. A case that is still running when
 * the deadline passes (an endless loop, a hang inside the fake kernel, a deadlock) makes the watchdog thread print one line to
 * stderr
 *
 *     CASE TIMEOUT <seconds> s: <case name> (<file>:<line>)
 *
 * flush it and end the process at once with std::_Exit(kCaseTimeoutExitCode): no destructors and no atexit handlers run,
 * because a hung case cannot be unwound. The gate (tools/it_gate.py) recognises the exit code and the line.
 *
 * The thread is a plain std::thread with a condition variable and std::chrono::steady_clock. It does not use the simulated time
 * or HostRtos (it must fire also when the case hangs inside the fake kernel or a busy loop), and it touches no test state: it
 * reads only its own mutex-protected deadline and the copy of the case name, so it is no data race for the sanitizers (the
 * integration executable is built with ASan and UBSan only; no TSan variant exists). Portable C++11 (Linux and MinGW).
 *
 * The limit: environment variable FFB_IT_CASE_TIMEOUT_S, a non-negative integer number of seconds; 0 = off; unset = the default
 * kDefaultCaseTimeoutS below. A value that is not such an integer is reported on stderr and the default is used.
 * Subcases: doctest re-enters a case once per subcase path (test_case_reenter); the limit is for the whole case, the deadline
 * is armed once at test_case_start and not reset by a reentry.
 *
 * Exit code: kCaseTimeoutExitCode = 86. It differs from the codes the gate already knows (0 passed, 1 a case failed, 124 the
 * gate's own process timeout, 127 not startable, 134 abort, 139 segmentation fault, 2 doctest option error).
 *
 * Default limit, measured (doctest --duration=true, the slowest cases of each variant, seconds; one process on the whole suite
 * per variant). What was done (2026-11-24): the four variants ran at the same time, one process each, on a 16-core machine with
 * nothing else heavy running (so a mild load, not an idle machine), FFB_IT_CASE_TIMEOUT_S=0, 434 cases run per full variant:
 *   variant  slowest five
 *   default  10.83 (F1-X1 demand beyond power), 10.71 (F1-E14 type change after Duration), 7.92 (F1-P1 reboot sequence),
 *            7.80 (F1-X7 supply returns after PC connected), 6.47 (characterization F1-T4 ENABLE_IN)
 *   m32      12.74 (F1-X1), 12.47 (F1-E14), 9.32 (F1-P1), 9.03 (F1-X7), 7.23 (F1-T4)
 *   cov      12.13 (F1-X1), 12.00 (F1-E14), 8.90 (F1-P1), 8.60 (F1-X7), 7.00 (F1-T4)
 *   san      0.30 (support: watchdog self-test), 0.06, 0.05, 0.006, 0.005. This row covers only the 97 cases that the sanitizer skip
 *            (IT_DELETES_FFBHIDMAIN) leaves; the skipped cases were not measured there, so the san limit rests on the other variants.
 *
 * Measure again (and revise the default) when a phase adds longer cases (the closed-loop phases I3 / I4), or when the slowest
 * case of any variant grows past a fifth of the limit (24 s at 120 s).
 *
 *
 * Derivation of the default: the slowest case of the slowest variant is 12.74 s (m32); 5 x 12.74 = 63.7 s, rounded up to 120 s, which is
 * 9.4 x the slowest case and leaves room for the 2-3 x slowdown that the gate's shards show when ten processes share the machine
 * (I1-T3). No per-variant factor is needed (CASE_TIMEOUT_FACTOR in it_gate.py is empty). The gate passes the same value explicitly.
 */
#pragma once

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace ffb_it {

constexpr int kCaseTimeoutExitCode = 86;
constexpr int kDefaultCaseTimeoutS = 120;
constexpr const char* kCaseTimeoutEnv = "FFB_IT_CASE_TIMEOUT_S";

/// Limit in seconds from the environment value (nullptr = unset): the value, or the default when unset or invalid.
/// `ok` (optional) is set to false for an invalid value.
int caseTimeoutSecondsFrom(const char* envValue, bool* ok = nullptr);

/// Everything the action of a firing watchdog is told.
struct CaseWatchdogEvent {
	std::string name;
	std::string file;
	unsigned line;
	std::chrono::milliseconds limit;
};

/// Prints the `CASE TIMEOUT` line to stderr, flushes and calls std::_Exit(kCaseTimeoutExitCode). The default action.
void exitOnCaseTimeout(const CaseWatchdogEvent& e);

/// One deadline timer. arm() starts the clock for a case, disarm() stops it; when the deadline passes while armed, `action`
/// runs once on the watchdog thread (and the watchdog stays disarmed). limit <= 0 never fires. Not copyable.
class CaseWatchdog {
public:
	explicit CaseWatchdog(std::chrono::milliseconds limit, std::function<void(const CaseWatchdogEvent&)> action = exitOnCaseTimeout);
	~CaseWatchdog();
	CaseWatchdog(const CaseWatchdog&) = delete;
	CaseWatchdog& operator=(const CaseWatchdog&) = delete;

	void arm(const std::string& name, const std::string& file, unsigned line);
	void disarm();
	bool armed() const;
	std::chrono::milliseconds limit() const { return limit_; }

private:
	void loop();

	std::chrono::milliseconds limit_;
	std::function<void(const CaseWatchdogEvent&)> action_;
	mutable std::mutex m_;
	std::condition_variable cv_;
	bool stop_ = false;
	bool armed_ = false;
	unsigned long long generation_ = 0;
	std::chrono::steady_clock::time_point deadline_;
	CaseWatchdogEvent event_;
	std::thread thread_;  // last member: started by the constructor after everything else exists
};

}  // namespace ffb_it
