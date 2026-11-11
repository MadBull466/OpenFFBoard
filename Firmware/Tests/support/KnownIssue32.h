#pragma once
/**
 * Markers for suspected defects that depend on the data model (P18, Tests/claude/plans/P18_32BIT_BUILD_PLAN.md).
 *
 * The target (STM32F407, 32-bit ARM) is ILP32: size_t is 4 bytes. The default host build is LP64 (size_t 8 bytes);
 * the FFB_TESTS_32BIT build (-m32) is ILP32 like the target. Some defects only exist with a 32-bit size_t, for
 * example a comparison of an int64_t with size_t, which is unsigned on LP64 and signed on ILP32.
 *
 * Both markers describe the INTENDED behaviour, which the LP64 host happens to show: on LP64 the case is a plain
 * passing test. Only on ILP32 builds (and so on the target) is the defect present:
 *
 *   KNOWN_ISSUE_32BIT(text)     ILP32: should_fail (the defect makes an assertion fail, the run continues).
 *                               The first failing assertion must be the one describing the defect.
 *   KNOWN_ISSUE_32BIT_UB(text)  ILP32: skipped. The defect is undefined behaviour (e.g. an out of bounds array
 *                               access) that libstdc++ assertions or the sanitizers turn into an abort of the whole
 *                               run. tools/run_all_tests.py runs skipped cases in their own process, so the full report
 *                               still shows it.
 *
 * Like KNOWN_ISSUE_UB_HIDDEN (TestHelpers.h) the descriptions contain "KNOWN ISSUE" so the inventory
 * (grep "KNOWN_ISSUE\|KNOWN ISSUE") lists them. Not modelled: other differences between i386 and ARM (see the plan).
 */
#include "doctest.h"

#include <cstddef>

namespace KnownIssue32 {
/** True on ILP32 builds (FFB_TESTS_32BIT, like the target) */
constexpr bool ilp32 = sizeof(std::size_t) == 4;
} // namespace KnownIssue32

#define KNOWN_ISSUE_32BIT(text)                                                                                        \
	doctest::description("KNOWN ISSUE (32-bit size_t only, intended result on LP64 hosts): " text) *                 \
		doctest::should_fail(KnownIssue32::ilp32)

#define KNOWN_ISSUE_32BIT_UB(text)                                                                                     \
	doctest::description("KNOWN ISSUE (32-bit size_t only, undefined behaviour, intended result on LP64 hosts): " text) * \
		doctest::skip(KnownIssue32::ilp32)
