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

RUN_ALL_TESTS = helper_paths.FIRMWARE_DIR / "Tests" / "Code_Coverage" / "tools" / "run_all_tests.py"
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
    r = rat.Result(exe, "Suite", name, "Firmware/Tests/Code_Coverage/cases/test_x.cpp:1", description, should_fail)
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


class SkippedCrashAndMemoryCorruptionTest(unittest.TestCase):
    """Tool defect T-1 (a): a skip-decorated crash / memory corruption case that passes with --no-skip is classified
    like a passing skip-decorated UB case, not as 'known bug no longer fails'."""

    def skipped_pass(self, description):
        r = make_result("passed", description)
        r.skip_decorated = True
        return r

    def test_a_skipped_crash_case_that_passes_alone_is_bugged_and_not_evidence_of_a_fix(self):
        r = self.skipped_pass("KNOWN ISSUE (crash): B-V11: getEncoderType dereferences a null pointer. Skipped because it terminates the test run")
        self.assertEqual(rat.classify(r), rat.BUGGED)
        self.assertIn("crash, passes in this run", r.note)
        self.assertIn("not evidence of a fix", r.note)

    def test_a_skipped_memory_corruption_case_that_passes_alone_is_bugged(self):
        r = self.skipped_pass("KNOWN ISSUE (memory corruption): B-V5: no bound check. Skipped; run manually with --no-skip")
        self.assertEqual(rat.classify(r), rat.BUGGED)
        self.assertIn("memory corruption, passes in this run", r.note)

    def test_the_undefined_behaviour_rule_is_unchanged(self):
        r = self.skipped_pass("KNOWN ISSUE (undefined behaviour): x")
        self.assertEqual(rat.classify(r), rat.BUGGED)
        self.assertIn("undefined behaviour, passes in this run", r.note)

    def test_a_skipped_crash_case_that_crashes_is_still_bugged(self):
        r = make_result("crash", "KNOWN ISSUE (crash): x")
        r.skip_decorated = True
        self.assertEqual(rat.classify(r), rat.BUGGED)
        self.assertIn("crashes as expected", r.note)

    def test_a_crash_case_that_is_not_skip_decorated_and_passes_is_still_failed_as_no_longer_failing(self):
        r = make_result("passed", "KNOWN ISSUE (crash): x", should_fail=True)
        self.assertEqual(rat.classify(r), rat.FAILED)
        self.assertIn("no longer fails", r.note)

    def test_a_skipped_case_with_a_plain_known_issue_marker_that_passes_is_still_failed(self):
        r = self.skipped_pass("KNOWN ISSUE: x")
        self.assertEqual(rat.classify(r), rat.FAILED)


COVERAGE_PROGRESS = """# Progress

## Suspected firmware bugs

| # | Module | Location | Description | Evidence |
|---|---|---|---|---|
| 183 | VESC driver: Buffer protocol | `VescCAN.cpp:630, 638` | No bound check on FILL_RX_BUFFER | skipped (memory corruption) [V24] (B-V5) |
| 189 | VESC driver: Encoder | `VescCAN.cpp:152-157` | getEncoderType() dereferences a null encoder | skipped crash [V63] (B-V11) |
| V5 | EEPROM | `eeprom.c:100` | Page 1 is never erased | KNOWN_ISSUE |
| V11 | cworkqueue | `cworkqueue.cpp:20` | Item queue leaks | KNOWN_ISSUE |
| V87 | TinyUSB MIDI class | `midi_device.c:300` | Reserved packet returns 0 | KNOWN_ISSUE (B-MIDI-V2) |

## NEEDS REVIEW items

| # | Module | Test / location | Question | Resolved? |
|---|---|---|---|---|
| 3 | Existing suite | 17 cases | Carried over | n/a |

## Untestable code
"""


class DriverTrackLabelTest(unittest.TestCase):
    """Tool defect T-1 (b): B-V11 is a driver-track label, not vendor bug V11."""

    def setUp(self):
        self.progress = rat.Progress(COVERAGE_PROGRESS)

    def test_a_driver_track_label_is_not_a_vendor_bug_number(self):
        self.assertEqual(rat.extract_numbers("KNOWN ISSUE (crash): B-V11: getEncoderType"), ["B-V11"])
        self.assertEqual(rat.extract_numbers("memory corruption: B-V5: no bound check"), ["B-V5"])
        self.assertEqual(rat.extract_numbers("B-MIDI-V2 (TinyUSB, vendor): same as"), ["B-MIDI-V2"])

    def test_a_real_vendor_number_next_to_a_label_is_still_found(self):
        self.assertEqual(rat.extract_numbers("[V63] getEncoderType (B-V11) and V5"), ["V63", "B-V11", "V5"])
        self.assertEqual(rat.extract_numbers("is rejected (V29)"), ["V29"])

    def test_a_label_resolves_to_the_row_whose_evidence_names_it(self):
        self.assertEqual(self.progress.resolve_label("B-V11"), "bug 189")
        self.assertEqual(self.progress.resolve_label("B-V5"), "bug 183")
        self.assertEqual(self.progress.resolve_label("B-MIDI-V2"), "V87")

    def test_a_label_without_a_row_and_other_numbers_stay_as_they_are(self):
        self.assertEqual(self.progress.resolve_label("B-V99"), "B-V99")
        self.assertEqual(self.progress.resolve_label("V11"), "V11")
        self.assertEqual(self.progress.resolve_label("bug 5"), "bug 5")

    def test_the_number_of_a_skipped_vesc_case_is_its_firmware_bug_and_not_vendor_bug_v11(self):
        r = make_result("passed", "KNOWN ISSUE (crash): B-V11: getEncoderType dereferences",
            name="[V63] getEncoderType with useencoder off and no external encoder does not crash")
        rat.annotate_numbers(r, self.progress)
        self.assertEqual(r.numbers, ["bug 189"])
        self.assertEqual(self.progress.description("bug 189"), "getEncoderType() dereferences a null encoder")


INTEGRATION_PROGRESS = """# Integration progress

## Integration bugs

| # | Module | Location | Description | Same as (coverage) | Evidence (test) | Resolved? |
|---|---|---|---|---|---|---|
| IB 1 | FFBHIDMain | `FFBoard/UserExtensions/Src/FFBHIDMain.cpp:256` | The FFB timer period is ARR+1 | | `F1-A4` case | no |
| IB 3 | HID command interface | `FFBoard/Src/HidCommandInterface.cpp:39-46` | Use after free of \\| outBuffer | | `KNOWN_ISSUE_UB` case | no |

## Integration NEEDS REVIEW items

| # | Module | Test / location | Question | Same as (coverage) | Resolved? |
|---|---|---|---|---|---|
| INR 1 | Axis / TMC4671 | `Axis.cpp:534-543` | Motor start after a runtime driver selection | | no |
| INR 2 | HID descriptor | `usb_hid_ffb_desc.h`; the PID State case | PID State without Effect Block Index | | no |

## Test tool defects
"""


class IntegrationNumbersTest(unittest.TestCase):
    def setUp(self):
        self.progress = rat.Progress(COVERAGE_PROGRESS, INTEGRATION_PROGRESS)

    def test_integration_bug_and_review_numbers_are_found_in_a_marker_text(self):
        self.assertEqual(rat.extract_numbers("KNOWN ISSUE: (IB 3) HID_CommandInterface::Run iterates"), ["IB 3"])
        self.assertEqual(rat.extract_numbers("KNOWN ISSUE (undefined behaviour): (IB 3) x"), ["IB 3"])
        self.assertEqual(rat.extract_numbers("// INR 2 the PID State report"), ["INR 2"])

    def test_inr_is_not_read_as_nr_and_ib_is_not_read_inside_a_word(self):
        self.assertEqual(rat.extract_numbers("INR 2"), ["INR 2"])
        self.assertEqual(rat.extract_numbers("NR 2 and INR 2"), ["NR 2", "INR 2"])
        self.assertEqual(rat.extract_numbers("a kibibyte of 3 and DIB 4"), [])

    def test_integration_numbers_do_not_collide_with_coverage_numbers(self):
        self.assertEqual(rat.extract_numbers("bug 3, IB 3, NR 3, INR 3"), ["bug 3", "IB 3", "NR 3", "INR 3"])
        self.assertEqual(len({rat.table_key(n) for n in ("bug 3", "NR 3", "IB 3", "INR 3")}), 4)

    def test_the_integration_tables_are_read_with_the_descriptions(self):
        self.assertEqual(self.progress.description("IB 1"), "The FFB timer period is ARR+1")
        self.assertEqual(self.progress.description("IB 3"), "Use after free of | outBuffer")
        self.assertEqual(self.progress.description("INR 1"), "Motor start after a runtime driver selection")
        self.assertEqual(self.progress.description("IB 9"), "")

    def test_the_coverage_rows_are_unchanged_next_to_the_integration_rows(self):
        self.assertEqual(self.progress.description("bug 189"), "getEncoderType() dereferences a null encoder")
        self.assertEqual(self.progress.description("NR 3"), "Carried over")

    def test_the_integration_tables_are_not_read_without_the_integration_text(self):
        self.assertEqual(rat.Progress(COVERAGE_PROGRESS).description("IB 1"), "")

    def test_the_location_of_an_integration_bug_gives_its_file_under_test(self):
        self.assertIn("HidCommandInterface.cpp", self.progress.location("IB 3"))

    def test_numbers_sort_bugs_before_reviews_before_integration_items(self):
        nums = ["INR 1", "IB 2", "NR 1", "bug 5", "V1", "IB 1"]
        self.assertEqual(sorted(nums, key=rat.number_sort_key), ["bug 5", "V1", "NR 1", "IB 1", "IB 2", "INR 1"])

    def test_an_integration_known_issue_case_shows_its_ib_number(self):
        r = make_result("failed", "KNOWN ISSUE: (IB 3) HID_CommandInterface::Run iterates outBuffer", True, exe="ffb_integration_tests")
        rat.annotate_numbers(r, self.progress)
        self.assertEqual(r.numbers, ["IB 3"])
        self.assertEqual(rat.classify(r), rat.BUGGED)

    def test_an_integration_ub_known_issue_case_shows_its_ib_number(self):
        r = make_result("failed", "KNOWN ISSUE (undefined behaviour): (IB 3) use after free", exe="ffb_integration_tests")
        rat.annotate_numbers(r, self.progress)
        self.assertEqual(r.numbers, ["IB 3"])

    def test_a_characterization_case_shows_the_inr_number_of_the_comment_beside_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = pathlib.Path(tmp) / "test_f1_x.cpp"
            src.write_text('TEST_CASE_FIXTURE(F, "characterization: [F1-A2] the PID State report has no index" * IT_DELETES_FFBHIDMAIN) {\n'
                '\t// INR 2\n\tCHECK(true);\n}\n', encoding="utf-8")
            r = rat.Result("ffb_integration_tests", "F1Setup", "characterization: [F1-A2] the PID State report has no index",
                rat.rel(str(src)) + ":1")
            rat.annotate_numbers(r, self.progress)
        self.assertEqual(r.numbers, ["INR 2"])
        self.assertTrue(r.deletes_main)

    def test_a_characterization_case_ignores_an_ib_number_in_its_comments(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = pathlib.Path(tmp) / "test_f1_y.cpp"
            src.write_text('TEST_CASE("characterization: [F1-A1] x") {\n\t// IB 3 is something else\n}\n', encoding="utf-8")
            r = rat.Result("ffb_integration_tests", "S", "characterization: [F1-A1] x", rat.rel(str(src)) + ":1")
            rat.annotate_numbers(r, self.progress)
        self.assertEqual(r.numbers, [])

    def test_the_log_entry_gives_the_integration_register_description(self):
        r = make_result("failed", "KNOWN ISSUE: (IB 1) timer", True, exe="ffb_integration_tests")
        rat.annotate_numbers(r, self.progress)
        rat.classify(r)
        text = "\n".join(rat.entry_lines(r, self.progress))
        self.assertIn("item:            IB 1 - The FFB timer period is ARR+1", text)


class IntegrationExecutableTest(unittest.TestCase):
    def test_the_integration_executable_is_run_with_its_exclusion_lists(self):
        self.assertIn("ffb_integration_tests", rat.EXECUTABLES)
        self.assertEqual(rat.EXCLUDE_VARS["ffb_integration_tests"],
            ("FFB_INTEGRATION_TESTS_EXCLUDE", "FFB_INTEGRATION_TESTS_EXCLUDE_SANITIZE"))
        self.assertEqual(rat.EXCLUDE_VARS_32BIT["ffb_integration_tests"], "FFB_INTEGRATION_TESTS_EXCLUDE_32BIT")
        self.assertEqual(rat.EXCLUDE_VARS_32BIT["ffb_target_tests"], "FFB_TARGET_TESTS_EXCLUDE_32BIT")

    def test_the_it_deletes_marker_is_the_deletes_ffbhidmain_marker(self):
        self.assertIn(rat.DELETES_MAIN_MARKER, "IT_DELETES_FFBHIDMAIN")

    def test_a_case_with_the_it_marker_that_fails_in_a_sanitizer_build_is_bug_24(self):
        r = make_result("crash", exe="ffb_integration_tests")
        r.deletes_main = True
        self.assertEqual(rat.classify(r, sanitize=True), rat.BUGGED)
        self.assertEqual(r.numbers[0], "bug 24")

    def test_the_marker_in_a_case_header_sets_the_deletes_flag(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = pathlib.Path(tmp) / "test_f1_z.cpp"
            src.write_text('TEST_CASE_FIXTURE(F, "a case" * IT_DELETES_FFBHIDMAIN) {\n\tCHECK(true);\n}\n', encoding="utf-8")
            r = rat.Result("ffb_integration_tests", "S", "a case", rat.rel(str(src)) + ":1")
            rat.annotate_numbers(r, rat.Progress(""))
        self.assertTrue(r.deletes_main)

    def test_the_exclusions_of_a_build_are_read_from_the_cmake_cache(self):
        with tempfile.TemporaryDirectory() as tmp:
            pathlib.Path(tmp, "CMakeCache.txt").write_text(
                "FFB_TESTS_32BIT:BOOL=ON\nFFB_TESTS_SANITIZE:BOOL=OFF\n"
                "FFB_INTEGRATION_TESTS_EXCLUDE:STRING=case a\nFFB_INTEGRATION_TESTS_EXCLUDE_SANITIZE:STRING=san case\n"
                "FFB_INTEGRATION_TESTS_EXCLUDE_32BIT:STRING=wide a;wide b\nFFB_TARGET_TESTS_EXCLUDE_32BIT:STRING=target wide\n",
                encoding="utf-8")
            config = rat.build_config(tmp)
            self.assertEqual(config["excludes"]["ffb_integration_tests"], ["case a", "wide a", "wide b"])
            self.assertEqual(config["excludes"]["ffb_target_tests"], ["target wide"])
            pathlib.Path(tmp, "CMakeCache.txt").write_text(
                "FFB_TESTS_32BIT:BOOL=OFF\nFFB_TESTS_SANITIZE:BOOL=ON\nFFB_INTEGRATION_TESTS_EXCLUDE:STRING=case a\n"
                "FFB_INTEGRATION_TESTS_EXCLUDE_SANITIZE:STRING=san case\nFFB_INTEGRATION_TESTS_EXCLUDE_32BIT:STRING=wide a\n",
                encoding="utf-8")
            config = rat.build_config(tmp)
            self.assertEqual(config["excludes"]["ffb_integration_tests"], ["case a", "san case"])


class SkippedCrashAttentionTest(unittest.TestCase):
    """A skipped crash / memory-corruption case is skipped in every build: when it passes alone it stays Bugged but is flagged."""

    def skipped_pass(self, description):
        r = make_result("passed", description)
        r.skip_decorated = True
        return r

    def test_a_passing_skipped_crash_case_is_bugged_with_attention_and_listed(self):
        crash = self.skipped_pass("KNOWN ISSUE (crash): x")
        corruption = self.skipped_pass("KNOWN ISSUE (memory corruption): y")
        for r in (crash, corruption):
            self.assertEqual(rat.classify(r), rat.BUGGED)
            self.assertTrue(r.attention)
            self.assertIn("check whether the bug is fixed", r.note)
        listed = rat.skipped_crash_passed([crash, corruption, make_result("passed")])
        self.assertEqual(listed, [crash, corruption])

    def test_a_passing_skipped_ub_case_has_no_attention_and_is_not_listed(self):
        r = self.skipped_pass("KNOWN ISSUE (undefined behaviour): x")
        self.assertEqual(rat.classify(r), rat.BUGGED)
        self.assertFalse(r.attention)
        self.assertEqual(rat.skipped_crash_passed([r]), [])

    def test_the_kind_is_the_leading_marker(self):
        later = self.skipped_pass("KNOWN ISSUE: x no longer crash (crash) and memory corruption")
        self.assertEqual(later.skipped_defect_kind(), "")
        self.assertEqual(rat.classify(later), rat.FAILED)
        self.assertFalse(later.attention)
        mixed = self.skipped_pass("KNOWN ISSUE (crash): a null pointer, undefined behaviour")
        self.assertEqual(mixed.skipped_defect_kind(), "crash")

    def test_a_crash_case_that_crashes_has_no_attention(self):
        r = make_result("crash", "KNOWN ISSUE (crash): x")
        r.skip_decorated = True
        rat.classify(r)
        self.assertFalse(r.attention)

    def test_the_report_has_a_block_and_a_summary_count_for_them(self):
        r = self.skipped_pass("KNOWN ISSUE (crash): x")
        rat.classify(r)
        text, _ = rat.render_report([r], [], {"dsp": True, "sanitize": False, "compiler": "c", "build_type": "t"}, ".", ["ffb_unit_tests"],
            rat.Progress(""), __import__("datetime").datetime.now(), 0.0)
        self.assertIn("Skipped crash / memory-corruption cases that passed alone (included in Bugged; check for a fix): 1", text)
        self.assertIn("Skip-decorated crash / memory-corruption cases that passed when run alone: check whether the bug is fixed (1)", text)


class LabelResolutionTest(unittest.TestCase):
    TEXT = """## Suspected firmware bugs

| # | Module | Location | Description | Evidence |
|---|---|---|---|---|
| 10 | VESC | `a.cpp:1` | one | skipped [V1] (B-V1) |
| 11 | VESC | `a.cpp:2` | eleven | skipped [V2] (B-V11) |
| V88 | MIDI | `m.c:3` | midi | KNOWN_ISSUE (B-MIDI-V3, two cases) |

## NEEDS REVIEW items

| # | Module | Test / location | Question | Resolved? |
|---|---|---|---|---|
| 7 | X | y | asks about (B-V99) | no |
"""

    def setUp(self):
        self.progress = rat.Progress(self.TEXT)

    def test_a_short_label_does_not_match_the_row_of_a_longer_one(self):
        self.assertEqual(self.progress.resolve_label("B-V1"), "bug 10")
        self.assertEqual(self.progress.resolve_label("B-V11"), "bug 11")

    def test_a_label_followed_by_a_comma_resolves(self):
        self.assertEqual(self.progress.resolve_label("B-MIDI-V3"), "V88")

    def test_needs_review_rows_are_not_used(self):
        self.assertEqual(self.progress.resolve_label("B-V99"), "B-V99")


if __name__ == "__main__":
    unittest.main()
