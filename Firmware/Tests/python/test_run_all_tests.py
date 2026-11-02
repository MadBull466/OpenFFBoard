"""Tests of tools/run_all_tests.py (the full test report) with small synthetic doctest XML reports and tables.

No test executable is run: XML parsing, classification, name filter escaping, number extraction and the
file-under-test lookup are tested directly.
"""
import os
import pathlib
import signal
import sys
import tempfile
import unittest

import helper_paths

RUN_ALL_TESTS = helper_paths.FIRMWARE_DIR / "Tests" / "tools" / "run_all_tests.py"
rat = helper_paths.load_script(RUN_ALL_TESTS, "run_all_tests_under_test")

HEADER = '<?xml version="1.0" encoding="UTF-8"?>\n<doctest binary="ffb_unit_tests">\n  <Options order_by="file"/>\n'

PASS_CASE = '''    <TestCase name="get command" filename="/r/cases/test_cmdparser.cpp" line="45">
      <OverallResultsAsserts successes="6" failures="0" test_case_success="true"/>
    </TestCase>
'''

FAIL_CASE = '''    <TestCase name="set command" filename="/r/cases/test_cmdparser.cpp" line="61">
      <SubCase name="negative" filename="/r/cases/test_cmdparser.cpp" line="68">
        <Expression success="false" type="CHECK" filename="/r/cases/test_cmdparser.cpp" line="70">
          <Original>
            value == -5
          </Original>
          <Expanded>
            5 == -5
          </Expanded>
        </Expression>
      </SubCase>
      <OverallResultsAsserts successes="3" failures="1" test_case_success="false"/>
    </TestCase>
'''

KNOWN_CASE = '''    <TestCase name="instance numbers above 9 can be addressed" filename="/r/cases/test_cmdparser.cpp" line="216" description="KNOWN ISSUE: (bug 16) only the first character" should_fail="true">
      <Expression success="false" type="CHECK" filename="/r/cases/test_cmdparser.cpp" line="222">
        <Original>
          target == &amp;twelve
        </Original>
      </Expression>
      <OverallResultsAsserts successes="2" failures="1" test_case_success="true"/>
    </TestCase>
'''

SKIPPED_CASE = '''    <TestCase name="command with an unknown id is rejected" filename="/r/cases/test_command_execution.cpp" line="186" description="KNOWN ISSUE (crash): x" skipped="true"/>
'''

CRASH_CASE = '''    <TestCase name="forces of a not existing axis" filename="/r/cases/test_effects_calculator.cpp" line="1062" description="KNOWN ISSUE (undefined behaviour): (bug 1) x" should_fail="true">
      <Exception crash="true">
        SIGABRT - Abort (abnormal termination) signal
      </Exception>
      <OverallResultsAsserts successes="0" failures="0" test_case_success="true"/>
    </TestCase>
'''

FOOTER = '''  <OverallResultsAsserts successes="9" failures="2"/>
  <OverallResultsTestCases successes="3" failures="1" skipped="1"/>
</doctest>
'''


def suite(*cases):
    return '  <TestSuite name="CmdParser">\n' + "".join(cases) + "  </TestSuite>\n"


def make_result(outcome, description="", should_fail=False, name="a test case", exe="ffb_unit_tests"):
    r = rat.Result(exe, "Suite", name, "Firmware/Tests/cases/test_x.cpp:1", description, should_fail)
    r.outcome = outcome
    return r


class DoctestXmlTest(unittest.TestCase):
    def test_a_complete_report_gives_every_case_with_its_result(self):
        cases, complete, totals = rat.parse_doctest_xml(HEADER + suite(PASS_CASE, FAIL_CASE, KNOWN_CASE, SKIPPED_CASE) + FOOTER)
        self.assertTrue(complete)
        self.assertEqual(totals.get("failures"), "1")
        self.assertEqual([c.name for c in cases], ["get command", "set command", "instance numbers above 9 can be addressed",
            "command with an unknown id is rejected"])
        self.assertTrue(all(c.finished for c in cases))
        self.assertEqual([c.suite for c in cases], ["CmdParser"] * 4)
        passed, failed, known, skipped = cases
        self.assertEqual((passed.asserts_failed, passed.success), (0, True))
        self.assertEqual((failed.asserts_failed, failed.success), (1, False))
        self.assertTrue(known.should_fail)
        self.assertTrue(skipped.skipped)
        self.assertEqual(skipped.description, "KNOWN ISSUE (crash): x")

    def test_the_first_failing_assertion_gives_file_line_and_expression(self):
        cases, _, _ = rat.parse_doctest_xml(HEADER + suite(FAIL_CASE) + FOOTER)
        self.assertIn("test_cmdparser.cpp:70 CHECK( value == -5 ) is 5 == -5", cases[0].first_failure)

    def test_a_truncated_report_ends_in_the_unfinished_case(self):
        text = HEADER + '  <TestSuite name="CmdParser">\n' + PASS_CASE + FAIL_CASE[:FAIL_CASE.index("<SubCase")]
        cases, complete, _ = rat.parse_doctest_xml(text)
        self.assertFalse(complete)
        self.assertEqual([c.finished for c in cases], [True, False])
        self.assertEqual(cases[-1].name, "set command")

    def test_a_crash_caught_by_doctest_is_recorded_for_its_case(self):
        cases, complete, _ = rat.parse_doctest_xml(HEADER + suite(CRASH_CASE) + FOOTER)
        self.assertTrue(complete)
        self.assertIn("SIGABRT", cases[0].crash)
        r = rat.result_from_run("ffb_unit_tests", cases[0])
        self.assertEqual(r.outcome, "crash")

    def test_an_empty_output_gives_no_cases(self):
        self.assertEqual(rat.parse_doctest_xml(""), ([], False, {}))

    def test_the_case_list_keeps_the_skip_decorator_and_the_order(self):
        text = ('<?xml version="1.0"?><doctest binary="x">'
            '<TestCase name="b" testsuite="S" filename="/r/t.cpp" line="9" skipped="false"/>'
            '<TestCase name="a" testsuite="S" filename="/r/t.cpp" line="20" skipped="true"/>'
            '<OverallResultsTestCases unskipped="1"/></doctest>')
        self.assertEqual(rat.parse_inventory(text), [("S", "b", "/r/t.cpp", 9, False), ("S", "a", "/r/t.cpp", 20, True)])


class ClassificationTest(unittest.TestCase):
    def test_a_passing_case_is_passed(self):
        self.assertEqual(rat.classify(make_result("passed")), rat.PASSED)

    def test_a_failing_case_is_failed(self):
        self.assertEqual(rat.classify(make_result("failed")), rat.FAILED)

    def test_a_crash_or_timeout_of_a_normal_case_is_failed(self):
        self.assertEqual(rat.classify(make_result("crash")), rat.FAILED)
        self.assertEqual(rat.classify(make_result("timeout")), rat.FAILED)

    def test_a_known_issue_that_fails_is_bugged(self):
        self.assertEqual(rat.classify(make_result("failed", "KNOWN ISSUE: (bug 16) x", True)), rat.BUGGED)

    def test_a_known_issue_that_crashes_or_hangs_is_bugged(self):
        for outcome in ("crash", "timeout"):
            r = make_result(outcome, "KNOWN ISSUE (crash): x")
            self.assertEqual(rat.classify(r), rat.BUGGED)

    def test_a_known_issue_that_passes_is_failed_as_no_longer_failing(self):
        r = make_result("passed", "KNOWN ISSUE: x", True)
        self.assertEqual(rat.classify(r), rat.FAILED)
        self.assertIn("no longer fails", r.note)

    def test_an_undefined_behaviour_case_with_the_intended_result_on_the_host_is_bugged(self):
        r = make_result("passed", rat.UB_HIDDEN_PREFIX + ": (bug 7) x")
        self.assertEqual(rat.classify(r), rat.BUGGED)
        self.assertIn("UB, passes on the host", r.note)

    def test_a_skipped_undefined_behaviour_case_that_passes_with_no_skip_stays_bugged(self):
        r = make_result("passed", "KNOWN ISSUE (undefined behaviour): x")
        r.skip_decorated = True
        self.assertEqual(rat.classify(r, sanitize=True), rat.BUGGED)

    def test_a_case_of_the_ctest_exclusion_list_that_fails_is_bugged_and_one_that_passes_is_passed_with_a_note(self):
        failing, passing = make_result("crash"), make_result("passed")
        failing.excluded = passing.excluded = True
        self.assertEqual(rat.classify(failing), rat.BUGGED)
        self.assertEqual(rat.classify(passing), rat.PASSED)
        self.assertTrue(passing.attention)

    def test_an_excluded_case_that_passes_is_highlighted_in_an_attention_banner(self):
        passing = make_result("passed")
        passing.excluded = True
        rat.classify(passing)
        banner = rat.attention_banner([passing])
        self.assertTrue(banner[0].startswith("!!!"))
        self.assertIn("ATTENTION", banner[1])
        self.assertTrue(any(passing.name in line and passing.location in line for line in banner))

    def test_no_attention_banner_is_written_when_nothing_needs_attention(self):
        self.assertEqual(rat.attention_banner([]), [])

    def test_a_case_deleting_a_main_class_that_fails_in_a_sanitizer_build_is_bug_24(self):
        r = make_result("crash")
        r.deletes_main = True
        self.assertEqual(rat.classify(r, sanitize=True), rat.BUGGED)
        self.assertEqual(r.numbers[0], "bug 24")
        other = make_result("crash")
        other.deletes_main = True
        self.assertEqual(rat.classify(other, sanitize=False), rat.FAILED)

    def test_python_expected_failures_are_bugged_and_unexpected_successes_failed(self):
        results = rat.records_to_results([
            {"suite": "m.T", "name": "test_a", "file": "", "line": 3, "outcome": "expected_failure", "detail": "x"},
            {"suite": "m.T", "name": "test_b", "file": "", "line": 9, "outcome": "unexpected_success", "detail": ""},
            {"suite": "m.T", "name": "test_c", "file": "", "line": 12, "outcome": "skipped", "detail": "no tool"},
            {"suite": "m.T", "name": "test_d", "file": "", "line": 15, "outcome": "passed", "detail": ""},
        ])
        self.assertEqual([rat.classify(r) for r in results], [rat.BUGGED, rat.FAILED, rat.FAILED, rat.PASSED])
        self.assertIn("no longer fails", results[1].note)

    def test_characterization_cases_are_passed(self):
        r = make_result("passed", name="characterization: the order matters")
        self.assertEqual(rat.classify(r), rat.PASSED)
        self.assertTrue(r.characterization)


class NameFilterTest(unittest.TestCase):
    def test_commas_wildcards_and_brackets_become_single_character_wildcards(self):
        self.assertEqual(rat.escape_filter("a, b * c? [d]"), "a? b ? c? ?d?")

    def test_the_escaped_filter_matches_the_original_name(self):
        name = "constant force, spring and damper are summed [x*2]?"
        pattern = rat.check_baseline.wildcard_to_regex(rat.escape_filter(name))
        self.assertTrue(pattern.match(name))
        self.assertFalse(pattern.match(name + " too"))


class NumberTest(unittest.TestCase):
    def test_bug_vendor_and_needs_review_numbers_are_found(self):
        self.assertEqual(rat.extract_numbers("KNOWN ISSUE: (bug 16) x"), ["bug 16"])
        self.assertEqual(rat.extract_numbers("bug 116: clips the index"), ["bug 116"])
        self.assertEqual(rat.extract_numbers("dereferences getCsPin(3). Bug 81."), ["bug 81"])
        self.assertEqual(rat.extract_numbers("is rejected (V29)"), ["V29"])
        self.assertEqual(rat.extract_numbers("// NEEDS REVIEW #39: the debounce"), ["NR 39"])
        self.assertEqual(rat.extract_numbers("see NR 120"), ["NR 120"])

    def test_numbers_are_in_order_of_appearance(self):
        self.assertEqual(rat.extract_numbers("V5 first, then bug 3 and NEEDS REVIEW 2"), ["V5", "bug 3", "NR 2"])

    def test_references_to_other_bugs_and_voltages_are_not_numbers(self):
        self.assertEqual(rat.extract_numbers("same pattern as bugs 37 and 56; supply 3V3"), [])

    def test_the_number_of_a_case_comes_from_its_comment_if_the_marker_has_none(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = pathlib.Path(tmp) / "test_x.cpp"
            src.write_text('// NEEDS REVIEW 25: intended?\nTEST_CASE("characterization: a" * DELETES_FFBHIDMAIN) {\n'
                '\t// bug 9 is something else\n\tCHECK(true);\n}\n', encoding="utf-8")
            r = rat.Result("ffb_target_tests", "S", "characterization: a", rat.rel(str(src)) + ":2")
            rat.annotate_numbers(r, rat.Progress(""))
        self.assertEqual(r.numbers, ["NR 25"])
        self.assertTrue(r.deletes_main)


PROGRESS = """# Progress

## NEEDS REVIEW items

| # | Module | Test / location | Question | Resolved? |
|---|---|---|---|---|
| 3 | Existing suite | 17 cases | Carried over | n/a |
| 39 | FFBHIDMain | `FFBHIDMain.cpp:343` | Debounce drops the edge \\| latching? | no |

## Suspected firmware bugs

| # | Module | Location | Description | Evidence |
|---|---|---|---|---|
| 5 | Command execution | `executeCommands` | **Dereferences** the `cmdDef` before the check | `FFBoard/Src/FFBoardMainCommandThread.cpp:99`; test `command with an unknown id is rejected` |
| 16 | CmdParser | `FFBoard/Src/CmdParser.cpp:120` | Only the first character | KNOWN_ISSUE |
| V29 | TinyUSB | `hid_device.c:110-126` | Claims EP0 OUT | KNOWN_ISSUE |

## Untestable code
"""


class ProgressTableTest(unittest.TestCase):
    def setUp(self):
        self.progress = rat.Progress(PROGRESS)

    def test_rows_of_both_tables_are_found_by_number(self):
        self.assertEqual(self.progress.description("bug 16"), "Only the first character")
        self.assertEqual(self.progress.description("V29"), "Claims EP0 OUT")
        self.assertEqual(self.progress.description("NR 39"), "Debounce drops the edge | latching?")
        self.assertEqual(self.progress.description("bug 99"), "")

    def test_markdown_emphasis_and_code_marks_are_removed_from_the_description(self):
        self.assertEqual(self.progress.description("bug 5"), "Dereferences the cmdDef before the check")

    def test_a_row_naming_the_test_gives_its_number(self):
        self.assertEqual(self.progress.find_by_test_name("command with an unknown id is rejected"), ["bug 5"])
        self.assertEqual(self.progress.find_by_test_name("no such test case anywhere"), [])

    def test_without_a_file_in_the_location_the_evidence_is_used(self):
        self.assertIn("FFBoardMainCommandThread.cpp", self.progress.location("bug 5"))


FILES = [
    "Firmware/FFBoard/Src/CmdParser.cpp", "Firmware/FFBoard/Inc/CmdParser.h", "Firmware/FFBoard/Src/Axis.cpp",
    "Firmware/FFBoard/Src/CDCcomm.cpp", "Firmware/FFBoard/Inc/CDCcomm.h", "Firmware/FFBoard/USB/class/hid/hid_device.c",
    "Firmware/FFBoard/USB/device/usbd.c", "Firmware/FFBoard/Src/FFBoardMainCommandThread.cpp",
    "Firmware/Targets/F407VG/Core/Src/main.c", "Firmware/Targets/F411RE/Core/Src/main.c",
    "Firmware/Libraries/CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.c", "Firmware/scripts/generate_memory.py",
]


class FileUnderTestTest(unittest.TestCase):
    def setUp(self):
        self.index = rat.SourceIndex(files=FILES)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def make_file(self, name, text=""):
        path = pathlib.Path(self.tmp.name) / name
        path.write_text(text, encoding="utf-8")
        return str(path)

    def test_table_paths_are_resolved_when_unambiguous(self):
        self.assertEqual(self.index.resolve("FFBoard/Src/CmdParser.cpp"), "Firmware/FFBoard/Src/CmdParser.cpp")
        self.assertEqual(self.index.resolve("hid_device.c"), "Firmware/FFBoard/USB/class/hid/hid_device.c")
        self.assertEqual(self.index.resolve("Firmware/scripts/generate_memory.py"), "Firmware/scripts/generate_memory.py")
        self.assertIsNone(self.index.resolve("main.c"))
        self.assertIsNone(self.index.resolve("missing.cpp"))

    def test_the_first_two_paths_of_a_location_are_used(self):
        text = "`FFBoard/Src/CmdParser.cpp:53` and `arm_sin_f32.c:88`, also `CDCcomm.cpp`"
        self.assertEqual(rat.files_from_location(text, self.index), ["Firmware/FFBoard/Src/CmdParser.cpp",
            "Firmware/Libraries/CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.c"])
        self.assertEqual(rat.files_from_location("in main.c", self.index), ["main.c (ambiguous or not found)"])
        self.assertEqual(rat.files_from_location("in main.c", self.index, resolved_only=True), [])

    def test_the_suite_name_selects_the_source(self):
        self.assertEqual(rat.files_from_test(self.make_file("test_cdc.cpp"), "CDCcomm", self.index),
            ["Firmware/FFBoard/Src/CDCcomm.cpp"])

    def test_the_test_file_name_selects_the_source(self):
        self.assertEqual(rat.files_from_test(self.make_file("test_cmdparser_gaps.cpp"), "Gaps", self.index),
            ["Firmware/FFBoard/Src/CmdParser.cpp"])
        self.assertEqual(rat.files_from_test(self.make_file("test_generate_memory_cases.py"), "m.T", self.index),
            ["Firmware/scripts/generate_memory.py"])

    def test_a_source_name_at_the_start_of_the_test_file_name_selects_the_source(self):
        self.assertEqual(rat.files_from_test(self.make_file("test_axis_config.cpp"), "", self.index),
            ["Firmware/FFBoard/Src/Axis.cpp"])
        self.assertEqual(rat.files_from_test(self.make_file("test_usbd_core.cpp"), "", self.index),
            ["Firmware/FFBoard/USB/device/usbd.c"])

    def test_an_included_header_matching_the_test_file_name_selects_the_source(self):
        path = self.make_file("test_cdc.cpp", '#include "doctest.h"\n#include "CDCcomm.h"\n')
        self.assertEqual(rat.files_from_test(path, "", self.index), ["Firmware/FFBoard/Src/CDCcomm.cpp"])

    def test_setup_tests_and_unmatched_files_are_unknown(self):
        self.assertEqual(rat.files_from_test(self.make_file("test_axis_setup.cpp"), "", self.index), [])
        self.assertEqual(rat.files_from_test(self.make_file("test_platform.cpp"), "Platform", self.index), [])

    def test_vendor_and_target_sources_are_not_matched_by_name(self):
        self.assertEqual(rat.files_from_test(self.make_file("test_main.cpp"), "", self.index), [])


class ProcessTest(unittest.TestCase):
    def test_a_process_without_progress_is_stopped_after_the_timeout(self):
        with tempfile.TemporaryDirectory() as tmp:
            progress = os.path.join(tmp, "progress.xml")
            rc, timed_out, _ = rat.run_process([sys.executable, "-c", "import time; time.sleep(30)"], tmp, "hang", 0.3,
                progress_file=progress)
        self.assertTrue(timed_out)

    def test_a_finished_process_gives_its_exit_status_and_stderr(self):
        with tempfile.TemporaryDirectory() as tmp:
            rc, timed_out, err = rat.run_process([sys.executable, "-c", "import sys; sys.stderr.write('x'); sys.exit(3)"],
                tmp, "exit", 30)
        self.assertEqual((rc, timed_out, err), (3, False, "x"))


class MessageTest(unittest.TestCase):
    def test_the_sanitizer_report_is_summarized_in_one_line(self):
        err = ("noise\n==1==ERROR: AddressSanitizer: stack-buffer-overflow on address 0x7bd23f412ac4 at pc 0x55e8\n"
            "SUMMARY: AddressSanitizer: ...\n")
        self.assertEqual(rat.stderr_summary(err), "ERROR: AddressSanitizer: stack-buffer-overflow on address 0x... at pc 0x55e8")

    def test_a_libstdcxx_assertion_is_summarized(self):
        self.assertIn("Assertion '__n < this->size()' failed", rat.stderr_summary(
            "/usr/include/c++/16/array:210: ... Assertion '__n < this->size()' failed.\n"))

    def test_signals_are_named(self):
        self.assertEqual(rat.signal_name(-int(signal.SIGABRT)), "SIGABRT")
        self.assertEqual(rat.signal_name(3), "exit status 3")

    def test_the_repository_root_is_removed_but_escapes_are_kept(self):
        text = os.path.join(rat.REPO_DIR, "Firmware", "x.cpp") + ": 'a\\n'"
        self.assertEqual(rat.strip_paths(text), os.path.join("Firmware", "x.cpp") + ": 'a\\n'")


if __name__ == "__main__":
    unittest.main()
