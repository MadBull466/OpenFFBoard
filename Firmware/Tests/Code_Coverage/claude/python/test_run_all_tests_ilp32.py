"""Tests of the 32-bit rules of tools/run_all_tests.py (P18): KNOWN_ISSUE_32BIT and KNOWN_ISSUE_32BIT_UB
(Firmware/Tests/Code_Coverage/support/KnownIssue32.h) and the 32-bit ctest exclusion list.

A KNOWN_ISSUE_32BIT(_UB) case describes the intended behaviour, which a 64-bit build shows: there it is a plain test
(doctest should_fail / skip are off). In a 32-bit build (FFB_TESTS_32BIT) it is a known bug like KNOWN_ISSUE and
KNOWN_ISSUE_UB. No test executable is run.
"""
import unittest

import helper_paths

RUN_ALL_TESTS = helper_paths.FIRMWARE_DIR / "Tests" / "Code_Coverage" / "tools" / "run_all_tests.py"
rat = helper_paths.load_script(RUN_ALL_TESTS, "run_all_tests_ilp32_under_test")

# The description prefixes written by the two macros of support/KnownIssue32.h
ILP32_DESC = "KNOWN ISSUE (32-bit size_t only, intended result on LP64 hosts): bug 225 x"
ILP32_UB_DESC = "KNOWN ISSUE (32-bit size_t only, undefined behaviour, intended result on LP64 hosts): bug 225 x"


def make_result(outcome, description, should_fail=False):
    r = rat.Result("ffb_target_tests", "Suite", "a test case", "Firmware/Tests/Code_Coverage/claude/cases_target/test_x.cpp:1",
        description, should_fail)
    r.outcome = outcome
    return r


class Ilp32PrefixTest(unittest.TestCase):
    def test_the_prefix_matches_both_macro_descriptions(self):
        self.assertTrue(ILP32_DESC.startswith(rat.ILP32_PREFIX))
        self.assertTrue(ILP32_UB_DESC.startswith(rat.ILP32_PREFIX))

    def test_the_prefix_is_not_one_of_the_other_known_issue_markers(self):
        for other in ("KNOWN ISSUE: x", "KNOWN ISSUE (undefined behaviour): x", rat.UB_HIDDEN_PREFIX + ": x"):
            self.assertFalse(other.startswith(rat.ILP32_PREFIX))


class Ilp32In64BitBuildTest(unittest.TestCase):
    def test_a_32_bit_only_known_issue_that_passes_in_a_64_bit_build_is_passed(self):
        for desc in (ILP32_DESC, ILP32_UB_DESC):
            r = make_result("passed", desc)
            self.assertEqual(rat.classify(r, sanitize=False, ilp32=False), rat.PASSED)
            self.assertIn("64-bit", r.note)

    def test_a_32_bit_only_known_issue_that_passes_in_a_64_bit_sanitizer_build_is_passed(self):
        r = make_result("passed", ILP32_UB_DESC)
        self.assertEqual(rat.classify(r, sanitize=True, ilp32=False), rat.PASSED)

    def test_a_32_bit_only_known_issue_that_fails_or_crashes_in_a_64_bit_build_is_failed(self):
        for outcome in ("failed", "crash", "timeout"):
            r = make_result(outcome, ILP32_DESC)
            self.assertEqual(rat.classify(r, sanitize=False, ilp32=False), rat.FAILED)


class Ilp32In32BitBuildTest(unittest.TestCase):
    def test_a_32_bit_known_issue_that_fails_as_expected_is_bugged(self):
        r = make_result("failed", ILP32_DESC, should_fail=True)
        self.assertEqual(rat.classify(r, ilp32=True), rat.BUGGED)

    def test_a_32_bit_known_issue_that_passes_is_failed_as_no_longer_failing(self):
        r = make_result("passed", ILP32_DESC, should_fail=True)
        self.assertEqual(rat.classify(r, ilp32=True), rat.FAILED)
        self.assertIn("no longer fails", r.note)

    def test_a_skipped_32_bit_undefined_behaviour_case_that_crashes_with_no_skip_is_bugged(self):
        r = make_result("crash", ILP32_UB_DESC)
        r.skip_decorated = True
        self.assertEqual(rat.classify(r, ilp32=True), rat.BUGGED)

    def test_a_skipped_32_bit_undefined_behaviour_case_that_passes_with_no_skip_stays_bugged(self):
        r = make_result("passed", ILP32_UB_DESC)
        r.skip_decorated = True
        self.assertEqual(rat.classify(r, ilp32=True), rat.BUGGED)


class Ilp32ConfigTest(unittest.TestCase):
    def test_classify_defaults_to_a_64_bit_build(self):
        r = make_result("passed", ILP32_DESC)
        self.assertEqual(rat.classify(r), rat.PASSED)


if __name__ == "__main__":
    unittest.main()
