/*
 * Hidden helper case of the per-case watchdog (tool WO I2-T2), compiled into ffb_flash_tests only (CMake globs test_*.cpp of cases_flash/).
 *
 * The firing path of the shared listener (Code_Coverage/support/CaseWatchdog.cpp: line on stderr, exit code 86 through std::_Exit) cannot be a
 * doctest case that passes: the watchdog ends the process. tools/test_it_gate.py runs this case as
 *     FFB_IT_CASE_TIMEOUT_S=1 ffb_flash_tests --no-skip -tc="support: case watchdog helper hangs on the host clock"
 * and must see exit code 86 and the line `CASE TIMEOUT 1 s: <this name> (<this file>:<line>)`. doctest::skip() keeps it out of every normal run
 * (one skipped case of this executable, never run without --no-skip, not listed by -ltc).
 */
#include "doctest.h"

#include <chrono>
#include <thread>

TEST_CASE("support: case watchdog helper hangs on the host clock" * doctest::skip()) {
	std::this_thread::sleep_for(std::chrono::seconds(30));
	FAIL("the case watchdog did not end this case");
}
