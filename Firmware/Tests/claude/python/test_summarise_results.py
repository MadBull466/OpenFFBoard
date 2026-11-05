"""Tests of tools/summarise_results.py with small synthetic doctest console, XML, ctest and build logs. Fast, no executables."""
import pathlib
import unittest

import helper_paths

SUMMARISE = helper_paths.FIRMWARE_DIR / "Tests" / "tools" / "summarise_results.py"
sr = helper_paths.load_script(SUMMARISE, "summarise_results_under_test")

RULE = "=" * 79
CONSOLE_FAIL = f"""[doctest] doctest version is "2.4.11"
{RULE}
/r/Firmware/Tests/cases/test_a.cpp:40:
TEST SUITE: A
TEST CASE:  value is doubled

/r/Firmware/Tests/cases/test_a.cpp:44: ERROR: CHECK( twice(2) == 4 ) is NOT correct!
  values: CHECK( 5 == 4 )

/r/Firmware/Tests/cases/test_a.cpp:45: ERROR: CHECK( twice(3) == 6 ) is NOT correct!
  values: CHECK( 7 == 6 )

{RULE}
/r/Firmware/Tests/cases/test_a.cpp:60:
DESCRIPTION: KNOWN ISSUE: a defect
TEST SUITE: A
TEST CASE:  known bug case

/r/Firmware/Tests/cases/test_a.cpp:62: ERROR: CHECK( f() ) is NOT correct!
  values: CHECK( false )

Failed as expected so marking it as not failed
{RULE}
[doctest] test cases: 10 |  8 passed |  1 failed | 1 skipped
[doctest] assertions: 30 | 27 passed |  3 failed |
[doctest] Status: FAILURE!
"""

CONSOLE_OK = f"""{RULE}
[doctest] test cases: 3 | 3 passed | 0 failed | 0 skipped
[doctest] assertions: 5 | 5 passed | 0 failed |
[doctest] Status: SUCCESS!
"""

XML = """<?xml version="1.0" encoding="UTF-8"?>
<doctest binary="ffb_x">
  <TestSuite name="S">
    <TestCase name="ok" filename="/r/Firmware/Tests/cases/t.cpp" line="1">
      <OverallResultsAsserts successes="2" failures="0" test_case_success="true"/>
    </TestCase>
    <TestCase name="broken" filename="/r/Firmware/Tests/cases/t.cpp" line="10">
      <Expression success="false" type="CHECK" filename="/r/Firmware/Tests/cases/t.cpp" line="12">
        <Original>a == b</Original>
        <Expanded>1 == 2</Expanded>
      </Expression>
      <OverallResultsAsserts successes="0" failures="1" test_case_success="false"/>
    </TestCase>
    <TestCase name="known" filename="/r/Firmware/Tests/cases/t.cpp" line="20" description="KNOWN ISSUE: x">
      <Expression success="false" type="CHECK" filename="/r/Firmware/Tests/cases/t.cpp" line="21">
        <Original>c</Original>
        <Expanded>false</Expanded>
      </Expression>
      <OverallResultsAsserts successes="0" failures="1" test_case_success="true"/>
    </TestCase>
    <TestCase name="skipme" filename="/r/Firmware/Tests/cases/t.cpp" line="30" skipped="true">
      <OverallResultsAsserts successes="0" failures="0" test_case_success="true"/>
    </TestCase>
  </TestSuite>
  <OverallResultsAsserts successes="2" failures="2"/>
  <OverallResultsTestCases successes="2" failures="1" skipped="1"/>
</doctest>
"""

CTEST = f"""Test project /r/build
    Start 1: ffb_x

1: Test command: /r/build/ffb_x "--test-case-exclude=case one,case two"
1: {RULE}
1: [doctest] test cases: 3 | 3 passed | 0 failed | 0 skipped
1: [doctest] assertions: 5 | 5 passed | 0 failed |
1/2 Test #1: ffb_x ..........................   Passed    0.10 sec
2/2 Test #2: ffb_y ..........................***Failed    0.20 sec

50% tests passed, 1 tests failed out of 2
"""

BUILD = """[ 1%] Building CXX object a.o
/r/Firmware/FFBoard/Src/A.cpp:12:5: warning: unused variable 'x' [-Wunused-variable]
/r/Firmware/FFBoard/Src/B.cpp:34:10: error: 'foo' was not declared in this scope
/usr/bin/ld: a.o: in function `main':
/r/Firmware/Tests/x.cpp:7: undefined reference to `bar()'
collect2: error: ld returned 1 exit status
"""


class SummariseResultsTest(unittest.TestCase):
    def test_format_detection(self):
        self.assertEqual(sr.detect(XML), "xml")
        self.assertEqual(sr.detect(CTEST), "ctest")
        self.assertEqual(sr.detect(CONSOLE_FAIL), "doctest")
        self.assertEqual(sr.detect(BUILD), "build")

    def test_console_counts_failure_and_known_issue(self):
        text, status = sr.summarise(CONSOLE_FAIL)
        self.assertEqual(status, 1)
        self.assertIn("failed: 1", text)
        self.assertIn("skipped: 1", text)
        self.assertIn("known issue: 1", text)
        self.assertIn("value is doubled", text)
        self.assertIn("at Firmware/Tests/cases/test_a.cpp:40", text)
        # the first failed assertion only, with the values
        self.assertIn("test_a.cpp:44: CHECK( twice(2) == 4 ) is NOT correct! values: CHECK( 5 == 4 )", text)
        self.assertNotIn("twice(3)", text)
        self.assertIn("known bug case", text)

    def test_console_success_exits_zero(self):
        text, status = sr.summarise(CONSOLE_OK)
        self.assertEqual(status, 0)
        self.assertIn("Test cases: 3  passed: 3  failed: 0  skipped: 0  known issue: 0", text)

    def test_known_issue_that_passes_is_a_failure(self):
        log = CONSOLE_FAIL.replace("Failed as expected so marking it as not failed",
                                   "Should have failed but didn't! Marking it as failed!")
        text, _ = sr.summarise(log)
        self.assertIn("known bug case", text)
        self.assertIn("known issue fixed", text)

    def test_xml(self):
        text, status = sr.summarise(XML)
        self.assertEqual(status, 1)
        self.assertIn("Test cases: 4  passed: 1  failed: 1  skipped: 1  known issue: 1", text)
        self.assertIn("broken", text)
        self.assertIn("t.cpp:12: CHECK(a == b) values: 1 == 2", text)
        self.assertIn("Skipped test cases (1):", text)
        self.assertIn("skipme", text)

    def test_ctest_excluded_and_failed_test(self):
        text, status = sr.summarise(CTEST)
        self.assertEqual(status, 1)  # ffb_y did not pass and printed no doctest result
        self.assertIn("case one", text)
        self.assertIn("case two", text)
        self.assertIn("Excluded test cases (2):", text)
        self.assertIn("ffb_y: ***Failed", text)

    def test_exclude_option(self):
        text, _ = sr.summarise(CONSOLE_OK, exclude=["a", "b,c"])
        self.assertIn("Excluded test cases (3):", text)

    def test_build_errors(self):
        text, status = sr.summarise(BUILD)
        self.assertEqual(status, 1)
        self.assertIn("Firmware/FFBoard/Src/B.cpp:34: 'foo' was not declared in this scope", text)
        self.assertIn("undefined reference to `bar()'", text)
        self.assertIn("Compiler warnings: 1", text)
        self.assertNotIn("Test cases", text)

    def test_unknown_format_is_rejected(self):
        with self.assertRaises(ValueError):
            sr.summarise(CONSOLE_OK, fmt="nonsense")

    def test_main_reads_a_file_and_reports_missing_files(self):
        import contextlib, io, tempfile
        with tempfile.TemporaryDirectory() as tmp:
            path = pathlib.Path(tmp) / "log.txt"
            path.write_text(CONSOLE_OK)
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(sr.main([str(path)]), 0)
            self.assertIn("passed: 3", out.getvalue())
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(sr.main([str(path) + ".missing"]), 2)


if __name__ == "__main__":
    unittest.main()
