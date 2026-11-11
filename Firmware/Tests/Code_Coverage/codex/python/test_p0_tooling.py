"""P0 safety-net regressions against real tools, with independent synthetic inputs.

Transport doubles supply gcov JSON/process status and doctest XML; production parsing,
accounting, validation and CLI status decisions run unchanged. They model no hardware.
Regression comments identify the confirmed findings now corrected. All stimuli run
in setUp; fixture, transport and production setup exceptions remain test errors.
Set P0_TOOLS_DIR to test an isolated candidate tool directory (no repository writes).
"""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

sys.dont_write_bytecode = True
TOOLS = Path(os.environ.get("P0_TOOLS_DIR", Path(__file__).resolve().parents[2] / "tools"))


def load(name):
    spec = importlib.util.spec_from_file_location("p0_" + name, TOOLS / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


cb = load("check_baseline")
coverage = load("coverage_report")
reporter = load("run_all_tests")


class ScratchFixture(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="codex-p0-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def main(self, tool, args):
        out, err = io.StringIO(), io.StringIO()
        with mock.patch.object(sys, "argv", [str(tool.__file__)] + args), \
                contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = tool.main()
        return code, out.getvalue(), err.getvalue()


def case(attrs='', body='<OverallResultsAsserts test_case_success="true"/>'):
    return '<TestCase name="original" ' + attrs + '>' + body + '</TestCase>'


def xml(*cases):
    return '<doctest><TestSuite name="Suite">' + ''.join(cases) + \
        '</TestSuite><OverallResultsTestCases successes="1" failures="0" skipped="0"/></doctest>'


class BaselinePreservation(ScratchFixture):
    def check(self, expected, text, sanitize=False, exclude="", rc=0, duplicate=False):
        baseline = self.root / "baseline.txt"
        line = expected + '\tSuite\toriginal\n'
        baseline.write_text(line * (2 if duplicate else 1), encoding="utf-8")
        exe = self.root / "test-executable"
        exe.touch()  # main validates existence; run_tests is the only transport double.
        with mock.patch.object(cb, "run_tests", return_value=(rc, text, "injected transport status")):
            return self.main(cb, ["--exe", str(exe), "--baseline", str(baseline),
                                  "--sanitize", "on" if sanitize else "off", "--exclude-list", exclude])

    def setUp(self):
        super().setUp()
        ub = 'description="KNOWN ISSUE (undefined behaviour): original defect" skipped="true"'
        self.ub_control = self.check("known_issue_ub", xml(case(ub)), sanitize=True)
        self.lost_marker = self.check("known_issue_ub", xml(case('skipped="true"')), sanitize=True)
        self.pass_control = self.check("pass", xml(case()))
        self.known_control = self.check("known_issue", xml(case('should_fail="true"')))
        self.skip_control = self.check("skip", xml(case('skipped="true"')))
        self.known_now_passes = self.check("known_issue", xml(case('should_fail="true"',
                                      '<OverallResultsAsserts test_case_success="false"/>')))
        self.excluded = self.check("known_issue_ub", xml(case('skipped="true"')), True, "or?g*")
        self.missing = self.check("pass", xml())
        self.nonzero = self.check("pass", xml(case()), rc=17)
        self.duplicate_baseline = self.check("pass", xml(case()), duplicate=True)
        self.duplicate_xml = self.check("pass", xml(case(), case()))
        with mock.patch.object(cb, "run_tests", return_value=(0, xml(case()), "")):
            self.print_current = self.main(cb, ["--exe", str(self.root / "test-executable"),
                                               "--baseline", str(self.root / "absent.txt"), "--print-current"])

    def test_matching_pass_and_known_issue_and_skip_preserve_the_baseline(self):
        for result in (self.pass_control, self.known_control, self.skip_control, self.ub_control):
            with self.subTest(result=result):
                self.assertEqual(result[0], 0, result)

    # P0-T3 / CDX-004: same skipped outcome hides lost conditional-UB metadata.
    def test_a_sanitizer_skip_without_the_original_ub_marker_is_rejected(self):
        self.assertNotEqual(self.lost_marker[0], 0, self.lost_marker)

    def test_print_current_does_not_require_a_readable_baseline(self):
        self.assertEqual(self.print_current[0], 0, self.print_current)
        self.assertIn("pass\tSuite\toriginal", self.print_current[1])

    def test_an_excluded_identity_remains_explicitly_unchecked(self):
        self.assertEqual(self.excluded[0], 0, self.excluded)
        self.assertIn("1 excluded (not checked)", self.excluded[1])

    def test_a_known_issue_that_did_not_fail_is_rejected(self):
        self.assertNotEqual(self.known_now_passes[0], 0, self.known_now_passes)

    def test_a_missing_original_is_rejected(self):
        self.assertNotEqual(self.missing[0], 0, self.missing)
        self.assertIn("MISSING", self.missing[1])

    def test_a_nonzero_child_status_is_rejected_even_with_passing_xml(self):
        self.assertNotEqual(self.nonzero[0], 0, self.nonzero)

    # P0-T6 / CDX-007: duplicate catalogue rows currently validate twice.
    def test_duplicate_baseline_identities_are_rejected_with_a_diagnostic(self):
        self.assertNotEqual(self.duplicate_baseline[0], 0, self.duplicate_baseline)
        self.assertIn("duplicate", self.duplicate_baseline[1].lower())

    # P0-T6 / CDX-007: dictionary overwrite hides duplicate report identity.
    def test_duplicate_report_identities_are_rejected_with_a_diagnostic(self):
        self.assertNotEqual(self.duplicate_xml[0], 0, self.duplicate_xml)
        self.assertIn("duplicate", self.duplicate_xml[1].lower())


class CoverageCompleteness(ScratchFixture):
    """Each gcov transport reply describes source line identities independently of report code."""
    def collect(self, target="HidFFB", missing=True, failed=(), data_only=False, merge=False, invalid=None):
        scenario = self.root / (target + str(missing) + str(data_only) + str(merge) + str(len(failed)) + str(invalid))
        fw, build = scenario / "firmware", scenario / "build"
        for directory in coverage.SOURCE_DIRS:
            (fw / directory).mkdir(parents=True, exist_ok=True)
        # Each scenario gets an independent object tree, so previous counters cannot fill a gap.
        objects = build / "CMakeFiles" / "test.dir"
        source_paths = {}
        for name in ("CmdParser", target):
            directory = "FFBoard/UserExtensions/Src" if target == name and name == "TMC4671" else "FFBoard/Src"
            rel = directory + "/" + name + ".cpp"
            source_paths[name] = rel
            (fw / rel).write_text('int f() {\n return 1;\n}\n', encoding="utf-8")
            note = objects / (rel + ".gcno")
            note.parent.mkdir(parents=True, exist_ok=True)
            note.touch()
            if name == "CmdParser" or not missing:
                note.with_suffix(".gcda").touch()
        if merge:
            extra = build / "CMakeFiles" / "other.dir" / (source_paths["CmdParser"] + ".gcda")
            extra.parent.mkdir(parents=True, exist_ok=True)
            extra.touch()

        def transport(cmd, **kwargs):
            path = Path(cmd[-1])
            name = path.name.split('.')[0]
            if name in failed:
                return subprocess.CompletedProcess(cmd, 3, b"", b"injected gcov failure")
            if invalid is not None and name == target:
                output = {"blank": "", "missing_files": "{}", "wrong_files": '{"files": {}}'}[invalid]
                return subprocess.CompletedProcess(cmd, 0, output.encode(), b"")
            # Notes-only executable lines have no runtime counts. Data-only is established
            # by gcov's successful empty file list, as real GCC emits for global constant data.
            count = 0 if path.suffix == '.gcno' else 1
            lines = [{"line_number": n, "count": count} for n in (1, 2)]
            if merge and name == "CmdParser":
                lines = [{"line_number": n, "count": int(n == (2 if 'other.dir' in str(path) else 1))} for n in (1, 2)]
            files = [] if data_only and name == target else [{"file": source_paths[name], "lines": lines, "functions": []}]
            doc = {"current_working_directory": str(fw), "files": files}
            return subprocess.CompletedProcess(cmd, 0, json.dumps(doc).encode(), b"")

        with mock.patch.object(coverage.subprocess, "run", side_effect=transport):
            return self.main(coverage, ["--build-dir", str(build), "--firmware-dir", str(fw), "--no-exclusions"])

    def setUp(self):
        super().setUp()
        self.missing = self.collect()
        self.driver = self.collect(target="TMC4671")
        self.all_failed = self.collect(failed=("CmdParser", "HidFFB"))
        self.partial_failed = self.collect(missing=False, failed=("HidFFB",))
        self.data_only = self.collect(target="DataTable", data_only=True)
        self.complete = self.collect(missing=False)
        self.union = self.collect(missing=False, merge=True)
        self.blank_counters = self.collect(missing=False, invalid="blank")
        self.blank_notes = self.collect(invalid="blank")
        self.missing_files = self.collect(invalid="missing_files")
        self.wrong_files = self.collect(invalid="wrong_files")

    # P0-T1 / CDX-002: notes-only executable workload must retain its denominator.
    def test_unrun_executable_source_contributes_zero_covered_executable_lines(self):
        self.assertIn('2 of 4 executable lines covered (50.0 %)', self.missing[1])
        self.assertEqual(self.missing[0], 0, self.missing)

    # P0-T1 / CDX-002: driver fallback has the same missing-denominator mechanism.
    def test_unrun_driver_source_is_zero_covered_in_the_driver_table(self):
        self.assertIn('`FFBoard/UserExtensions/Src/TMC4671.cpp` | 0 / 2 | 0.0 %', self.driver[1])
        self.assertEqual(self.driver[0], 0, self.driver)

    # P0-T2 / CDX-003: a failed collector cannot yield a successful measurement.
    def test_total_gcov_collection_failure_returns_nonzero(self):
        self.assertNotEqual(self.all_failed[0], 0, self.all_failed)

    # P0-T2 / CDX-003: one successful source cannot conceal another failed collector.
    def test_partial_gcov_collection_failure_returns_nonzero(self):
        self.assertNotEqual(self.partial_failed[0], 0, self.partial_failed)

    # CDX-003 / QC-2: successful process must return usable gcov documents.
    def test_blank_counter_output_is_incomplete_even_with_another_valid_source(self):
        self.assertNotEqual(self.blank_counters[0], 0, self.blank_counters)
        self.assertIn("Coverage report incomplete", self.blank_counters[2])
        self.assertIn("gcov", self.blank_counters[2])

    # CDX-003 / QC-2: successful process must return usable gcov documents.
    def test_blank_notes_output_is_incomplete_even_with_another_valid_source(self):
        self.assertNotEqual(self.blank_notes[0], 0, self.blank_notes)
        self.assertIn("Coverage report incomplete", self.blank_notes[2])
        self.assertIn("gcov", self.blank_notes[2])

    # CDX-003 / QC-2: successful process must return usable gcov documents.
    def test_a_document_without_files_is_incomplete_even_with_another_valid_source(self):
        self.assertNotEqual(self.missing_files[0], 0, self.missing_files)
        self.assertIn("Coverage report incomplete", self.missing_files[2])
        self.assertIn("gcov", self.missing_files[2])

    # CDX-003 / QC-2: successful process must return usable gcov documents.
    def test_a_document_with_nonlist_files_is_incomplete_even_with_another_valid_source(self):
        self.assertNotEqual(self.wrong_files[0], 0, self.wrong_files)
        self.assertIn("Coverage report incomplete", self.wrong_files[2])
        self.assertIn("gcov", self.wrong_files[2])

    def test_data_only_notes_preserve_a_zero_executable_source(self):
        self.assertEqual(self.data_only[0], 0, self.data_only)
        self.assertIn('2 files, 2 of 2 executable lines covered', self.data_only[1])

    def test_fully_measured_sources_keep_their_coverage(self):
        self.assertEqual(self.complete[0], 0, self.complete)
        self.assertIn('4 of 4 executable lines covered (100.0 %)', self.complete[1])

    def test_two_objects_covering_different_lines_form_a_line_union(self):
        self.assertEqual(self.union[0], 0, self.union)
        self.assertIn('4 of 4 executable lines covered (100.0 %)', self.union[1])


def listing(*cases):
    return '<doctest><Options no_skip="true"/>' + ''.join(cases) + \
        '<OverallResultsTestCases unskipped="%d"/></doctest>' % len(cases)


LIST_CASE = '<TestCase testsuite="S" name="case" filename="probe.cpp" line="1" skipped="false"/>'


class ReporterWorkloads(ScratchFixture):
    def inventory(self, rc=0, timeout=False, text=None, primary=True):
        text = listing() if text is None else text
        runner = reporter.ExeRunner("probe", "unused", str(self.root), 1, log=lambda *_: None)
        with mock.patch.object(runner, "doctest", return_value=(rc, timeout, "injected inventory status", text)):
            try:
                result = runner.run() if primary else runner.inventory(["--test-case=no-match"])
            except (RuntimeError, ValueError) as exc:
                return type(exc).__name__, str(exc)
        return False, result

    def setUp(self):
        super().setUp()
        self.exit_error = self.inventory(rc=17, text=listing(LIST_CASE), primary=False)
        self.timeout_error = self.inventory(timeout=True, text=listing(LIST_CASE), primary=False)
        self.empty_primary = self.inventory()
        self.empty_probe = self.inventory(primary=False)
        self.bad_root = self.inventory(text='<unrelated/>', primary=False)
        self.valid_inventory = self.inventory(text=listing(LIST_CASE), primary=False)
        self.missing_identity = self.inventory(text=listing('<TestCase/>'), primary=False)
        self.nested_run = self.inventory(text='<doctest><TestSuite name="S">' + LIST_CASE + '</TestSuite><OverallResultsTestCases unskipped="1"/></doctest>', primary=False)
        self.invalid_line = self.inventory(text=listing(LIST_CASE.replace('line="1"', 'line="invalid"')), primary=False)
        self.zero_line = self.inventory(text=listing(LIST_CASE.replace('line="1"', 'line="0"')), primary=False)
        self.run_results = self.inventory(text=listing(LIST_CASE.replace('/>', '><OverallResultsAsserts test_case_success="true"/></TestCase>')), primary=False)
        runner = reporter.ExeRunner("probe", "unused", str(self.root), 1, log=lambda *_: None)
        completed = '<doctest><TestSuite name="S"><TestCase name="case" filename="probe.cpp" line="1"><OverallResultsAsserts test_case_success="true"/></TestCase></TestSuite><OverallResultsTestCases successes="1" failures="0" skipped="0"/></doctest>'
        with mock.patch.object(runner, "doctest", side_effect=[(0, False, "", listing(LIST_CASE)), (0, False, "", completed)]):
            self.primary_results = runner.run()
        self.primary_problems = runner.problems
        self.missing_summary = self.inventory(text='<doctest>' + LIST_CASE + '</doctest>', primary=False)
        self.wrong_summary = self.inventory(text=listing(LIST_CASE).replace('unskipped="1"', 'unskipped="0"'), primary=False)
        empty = self.root / "empty_python"
        empty.mkdir()
        self.empty_python = reporter.python_results(str(empty), str(self.root), 1)
        nonempty = self.root / "nonempty_python"
        nonempty.mkdir()
        (nonempty / "test_control.py").write_text('import unittest\nclass Control(unittest.TestCase):\n def test_work(self): self.assertEqual(2 + 2, 4)\n', encoding="utf-8")
        self.nonempty_python = reporter.python_results(str(nonempty), str(self.root), 1)

    # P0-T4 / CDX-005: unittest's zero-discovery success is not a mandatory workload.
    def test_an_empty_mandatory_python_group_is_a_run_problem(self):
        self.assertTrue(self.empty_python[1], self.empty_python)

    def test_a_nonempty_python_group_runs_and_reports_its_case(self):
        results, problems = self.nonempty_python
        self.assertEqual(problems, [])
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].outcome, reporter.PASSED)

    # P0-T5 / CDX-006: valid XML does not override a failed inventory process.
    def test_nonzero_inventory_status_is_rejected(self):
        self.assertEqual(self.exit_error[0], "RuntimeError", self.exit_error)
        self.assertIn("--list-test-cases failed (exit status 17)", self.exit_error[1])

    # P0-T5 / CDX-006: timeout remains failure even if XML was flushed.
    def test_timed_out_inventory_is_rejected(self):
        self.assertEqual(self.timeout_error[0], "RuntimeError", self.timeout_error)
        self.assertIn("--list-test-cases failed (timeout)", self.timeout_error[1])

    # P0-T5 / CDX-006: a primary executable must provide at least one case.
    def test_an_empty_primary_inventory_is_rejected(self):
        self.assertTrue(self.empty_primary[0], self.empty_primary)

    # P0-T5 / CDX-006: unrelated valid XML is not a doctest inventory.
    def test_an_inventory_with_the_wrong_root_is_rejected(self):
        self.assertTrue(self.bad_root[0], self.bad_root)

    # CDX-006 / QC-3: reject malformed supported list shape gracefully.
    def test_inventory_missing_identity_is_a_controlled_run_problem(self):
        self.assertEqual(self.missing_identity[0], "RuntimeError", self.missing_identity)
        self.assertIn("invalid inventory", self.missing_identity[1])

    # CDX-006 / QC-3: reject malformed supported list shape gracefully.
    def test_inventory_nested_run_is_a_controlled_run_problem(self):
        self.assertEqual(self.nested_run[0], "RuntimeError", self.nested_run)
        self.assertIn("invalid inventory", self.nested_run[1])

    # CDX-006 / QC-3: reject malformed supported list shape gracefully.
    def test_inventory_invalid_line_is_a_controlled_run_problem(self):
        self.assertEqual(self.invalid_line[0], "RuntimeError", self.invalid_line)
        self.assertIn("invalid inventory", self.invalid_line[1])

    # CDX-006 / QC-3: reject malformed supported list shape gracefully.
    def test_inventory_zero_line_is_a_controlled_run_problem(self):
        self.assertEqual(self.zero_line[0], "RuntimeError", self.zero_line)
        self.assertIn("invalid inventory", self.zero_line[1])

    # CDX-006 / QC-3: reject malformed supported list shape gracefully.
    def test_inventory_missing_summary_is_a_controlled_run_problem(self):
        self.assertEqual(self.missing_summary[0], "RuntimeError", self.missing_summary)
        self.assertIn("invalid inventory", self.missing_summary[1])

    # CDX-006 / QC-3: reject malformed supported list shape gracefully.
    def test_inventory_wrong_summary_is_a_controlled_run_problem(self):
        self.assertEqual(self.wrong_summary[0], "RuntimeError", self.wrong_summary)
        self.assertIn("invalid inventory", self.wrong_summary[1])

    # CDX-006 / QC-3: run results cannot be mistaken for list records.
    def test_inventory_run_results_are_a_controlled_run_problem(self):
        self.assertEqual(self.run_results[0], "RuntimeError", self.run_results)
        self.assertIn("invalid inventory", self.run_results[1])

    def test_a_nonempty_primary_workload_runs_its_case(self):
        self.assertEqual(self.primary_problems, [])
        self.assertEqual(len(self.primary_results), 1)
        self.assertEqual(self.primary_results[0].outcome, reporter.PASSED)

    def test_an_internal_filter_probe_may_legitimately_match_no_cases(self):
        self.assertEqual(self.empty_probe, (False, []))

    def test_a_successful_nonempty_inventory_retains_its_identity(self):
        self.assertEqual(self.valid_inventory, (False, [("S", "case", "probe.cpp", 1, False)]))


if __name__ == '__main__':
    unittest.main()
