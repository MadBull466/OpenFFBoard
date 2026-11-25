/*
 * CaseWatchdog (tool WOs I2-T1 and I2-T2): a time limit for every single test case of every doctest executable of the Claude / shared suite
 * instead of a limit for the whole run (owner's decisions 2026-11-24: "Do not set a time limit on the overall test runs as this is a
 * reoccurring issue. Have time limits for each test case." and "Per-case watchdog in all executables"). Codex's own executables (codex_*)
 * do not have it.
 *
 * What it is: a doctest listener (REGISTER_LISTENER in CaseWatchdog.cpp). On test_case_start it arms a host-clock deadline for the case, on
 * test_case_end it disarms it. A case that is still running when the deadline passes (an endless loop, a hang inside the fake kernel, a
 * deadlock, a HAL polling loop) makes the watchdog thread print one line to stderr
 *
 *     CASE TIMEOUT <seconds> s: <case name> (<file>:<line>)
 *
 * flush it and end the process at once with std::_Exit(kCaseTimeoutExitCode): no destructors and no atexit handlers run,
 * because a hung case cannot be unwound. The gate (tools/it_gate.py) recognises the exit code and the line.
 *
 * Linked into every executable through CMake (Firmware/Tests/CMakeLists.txt): ffb_add_case_watchdog(<target> <default seconds>) adds this
 * source (variable FFB_CASE_WATCHDOG_SOURCES), the compile definition FFB_CASE_TIMEOUT_DEFAULT_S=<default> and Threads::Threads to the
 * target. It must be an object of the executable itself (REGISTER_LISTENER is a static initialiser, a static library would drop it).
 * The gate fails when a doctest executable of the ctest list has no such default in compile_commands.json.
 *
 * The thread is a plain std::thread with a condition variable and std::chrono::steady_clock. It does not use the simulated time
 * or HostRtos (it must fire also when the case hangs inside the fake kernel or a busy loop), and it touches no test state: it
 * reads only its own mutex-protected deadline and the copy of the case name, so it is no data race for the sanitizers (no TSan variant
 * exists). Portable C++11 (Linux and MinGW).
 *
 * The limit: environment variable FFB_IT_CASE_TIMEOUT_S (the name is generic: it applies to every executable, not only the integration
 * one; it kept its name from tool WO I2-T1), a non-negative integer number of seconds; 0 = off; unset = the executable's compiled default
 * kDefaultCaseTimeoutS (= FFB_CASE_TIMEOUT_DEFAULT_S, 120 when a build does not name one). A value that is not such an integer is reported
 * on stderr and the default is used. Subcases: doctest re-enters a case once per subcase path (test_case_reenter); the limit is for the
 * whole case, the deadline is armed once at test_case_start and not reset by a reentry.
 *
 * Exit code: kCaseTimeoutExitCode = 86. It differs from the codes the gate already knows (0 passed, 1 a case failed, 124 the
 * gate's own process timeout (no longer used), 127 not startable, 134 abort, 139 segmentation fault, 2 doctest option error).
 *
 * Default limits, measured (2026-11-24, doctest --duration=true, FFB_IT_CASE_TIMEOUT_S=0, one process per executable, all executables of a
 * variant at the same time on a 16-core machine with other builds running, so a mild load, not an idle machine). The slowest five cases of
 * each executable and variant, in seconds (cases per full run in brackets). Where a row says "same cases" the five slowest differ by less
 * than 0.1 s from the row above.
 *
 *   ffb_unit_tests (651-654):  default 0.301 (non numeric values are rejected without blocking the parser, a 300 ms thread guard), then
 *                              0.008, 0.004, 0.004, 0.003; m32 0.300, 0.024, 0.008, 0.008, 0.008; san 0.300, 0.013, 0.009, 0.006, 0.004.
 *   ffb_target_tests (1751-1867): default 0.006 at most (a constant force reaches the motor driver ...), 0.005, 0.005, 0.004, 0.003;
 *                              m32 0.026 (the source thread resets the I2C port ...), 0.012, 0.011, 0.010, 0.009; san 0.012, 0.012, 0.008, 0.007, 0.007.
 *   ffb_flash_tests (159):     default 0.138 (power loss at any step of the EE_Init recovery ...), 0.107, 0.083, 0.030, 0.028;
 *                              m32 0.076, 0.061, 0.054, 0.028, 0.019; san 0.145, 0.142, 0.127, 0.083, 0.050.
 *   ffb_rtos_tests (278-283):  default 0.001 at most; m32 0.009 (disabling a tick hook ...), then 0.000; san 0.030 (a condition variable owns one
 *                              internal mutex ...), 0.008, 0.008, 0.007, 0.007.
 *   ffb_usb_tests (604-606):   default 0.008 (the line coding is 115200 baud ...), 0.005, 0.004, 0.004, 0.004; m32 0.048 (same case), 0.009 x4;
 *                              san 0.028, 0.003 x4.
 *   ffb_driver_tests (1706-1722): default 4.690 ([R77] a failed ADC calibration in initialize() keeps the enable pin low and the PWM off),
 *                              0.450, 0.256, 0.175, 0.101; m32 4.305, 0.556, 0.254, 0.155, 0.094; san 18.227 (same [R77] case), 2.304, 1.069, 0.651, 0.389;
 *                              clang-san 16.612, 2.109, 0.933, 0.582, 0.352; nodsp-san 17.242, 2.181, 0.970, 0.601, 0.359 (measured 2026-11-25, tool WO I2-T2 fix round).
 *   ffb_dsp_tests (1872-2486): default 0.405 (arm_fir_q7 two tap filter ...), 0.233, 0.222, 0.150, 0.144; m32 0.295 (arm_correlate_opt_q7 two values
 *                              x one coefficient ...), 0.273, 0.169, 0.168, 0.160; san 2.248 (arm_fir_q7 ...), 1.270, 1.192, 0.667, 0.650; clang-san 2.363, 1.061, 1.043, 0.608, 0.597.
 *   ffb_integration_tests (434; 97 in san, see below): default 16.430 (F1-X1 demand beyond power), 10.049 (F1-E14 type change after Duration),
 *                              7.486 (F1-P1 reboot sequence), 7.212 (F1-X7 supply returns after PC connected), 6.939 (F1-X1 full-scale constant force);
 *                              m32 12.088 (F1-X1), 11.867 (F1-E14), 8.828 (F1-P1), 8.512 (F1-X7), 6.944 (characterization F1-T4 ENABLE_IN);
 *                              cov: see tool WO I2-T1 (12.13 (F1-X1), 12.00, 8.90, 8.60, 7.00); san 0.300 (support: watchdog self-test), 0.061, 0.050,
 *                              0.011, 0.007 (clang-san and nodsp-san the same, at most 0.300 s): only the 97 cases that the sanitizer skip (IT_DELETES_FFBHIDMAIN) leaves ran, so the san limit rests on
 *                              the other variants.
 *   ffb_hal_tests / ffb_hal_tests_disco (4067 / 4063 in san; 32-bit build with FFB_TESTS_REAL_HAL=ON; both executables the same within 0.4 s):
 *                              default 17.068 (characterization: CI4 WakeUp returns ERROR with TIMEOUT when SLAK never clears), 16.867 (ID1 a DMA call with
 *                              the bus held busy ...), 11.214 (IT1 a busy bus makes a master IT start function return HAL_BUSY ...), 10.991 (IQ4 a first frame
 *                              on a bus held by another controller ...), 5.505; san 16.260 (CI4), 16.150 (ID1), 10.961 (IQ4), 10.722 (IT1), 5.419. These are
 *                              the HAL polling loops (HAL_USB_TIMEOUT = 2000U, I2C and CAN timeouts) on the trapping register bus.
 *
 * Derivation: default = at least 5 x the slowest case of the slowest measured variant, rounded to a round number, and for the fast executables
 * far more, because a limit of a few seconds would fire on a loaded gate (ten processes share the machine, I1-T3: 2-3 x slower) or in the cov
 * variant, while a hang is found just as well after 30 s:
 *   ffb_unit_tests, ffb_target_tests, ffb_flash_tests, ffb_rtos_tests, ffb_usb_tests: slowest 0.30 s (unit, 5 x = 1.5 s) -> 30 s each (100 x).
 *   ffb_dsp_tests:     slowest 2.363 s (clang-san); 5 x = 11.8 s -> 60 s (27 x).
 *   ffb_driver_tests:  slowest 18.227 s (san; of default, m32, san, clang-san, nodsp-san); 5 x = 91.1 s -> 120 s (6.6 x), unchanged by the fix round.
 *   ffb_integration_tests: slowest 16.430 s (default, loaded: three integration runs and the HAL tests at the same time); 5 x = 82.2 s -> 120 s
 *                      (7.3 x). It is also the value of tool WO I2-T1 (its measurement was 12.74 s). The gate passes the value explicitly
 *                      (read from compile_commands.json).
 *   ffb_hal_tests, ffb_hal_tests_disco: slowest 17.068 s; 5 x = 85.3 s -> 120 s (7.0 x) for the default and the san build alike.
 * No per-variant factor is needed (CASE_TIMEOUT_FACTOR in it_gate.py is empty).
 *
 * Measure again (and revise the default of that executable in CMake) when a phase adds longer cases (the closed-loop phases I3 / I4), or when
 * the slowest case of an executable grows past a fifth of its limit (24 s at 120 s, 6 s at 30 s).
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
#ifndef FFB_CASE_TIMEOUT_DEFAULT_S
#define FFB_CASE_TIMEOUT_DEFAULT_S 120  // only when a build does not name the executable's default (CMake: ffb_add_case_watchdog)
#endif
constexpr int kDefaultCaseTimeoutS = FFB_CASE_TIMEOUT_DEFAULT_S;
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
