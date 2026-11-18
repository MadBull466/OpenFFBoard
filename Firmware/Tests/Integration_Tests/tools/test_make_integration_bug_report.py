"""Tests of make_integration_bug_report.py with a synthetic PROGRESS.md (nothing of the repository is read or written).

Python standard library only: python3 -B Firmware/Tests/Integration_Tests/tools/test_make_integration_bug_report.py
"""
import contextlib
import io
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import make_integration_bug_report as mk  # noqa: E402

PROGRESS = """# Progress

## Current state

| # | not a register |
|---|---|
| IB 99 | ignored: outside the four tables |

## Integration bugs

| # | Module | Location | Description | Same as (coverage) | Evidence (test) | Resolved? |
|---|---|---|---|---|---|---|
| IB 2 | Module B | `b.cpp:2` | Second bug with a pipe \\| inside | bug 201 | `case b` | no |
| IB 1 | Module A | `a.cpp:1` | First bug | | `case a` | yes (WO I1-1) |

## Integration NEEDS REVIEW items

| # | Module | Test / location | Question | Same as (coverage) | Resolved? |
|---|---|---|---|---|---|
| INR 1 | Axis | `Axis.cpp:5` | Is it intended? Asked as IQ6 | | no |
| INR 2 | HID | `d.h` | Another question | NR 7 | resolved 2026-11-19 |

## Test tool defects

| # | Tool | Defect | Status |
|---|---|---|---|
| T-1 | `tools/x.py` | Misreports a case | open, fixed in WO I0-8 (owner, IQ7) |
| T-2 | `tools/y.py` | Another | fixed in WO I1-2 |

## Fake audit register

| Fake | Audit (WO, date) | Result | Fix (WO, owner approval) | Coverage cases changed |
|---|---|---|---|---|
| `FakeOne` | I0-2, 2026-11-18 | no defect; gaps handled | none proposed | none |
| Fake SPI | I0-2, 2026-11-18 | **A-1 defect**: wrong error delivered | fixed in WO I0-3a | 2 cases edited |
| Fake RTOS | I0-2, 2026-11-18 | **A-3 gap, integration blocker**: delay moves the clock | not fixed yet | none |

## Environment

text
"""


def replace(old, new, text=PROGRESS):
    assert old in text, old
    return text.replace(old, new)


class TableParsing(unittest.TestCase):
    def test_the_four_tables_are_read_and_sorted_by_number(self):
        reg = mk.parse_register(PROGRESS)
        self.assertEqual([r[0] for r in reg["bugs"]], ["IB 1", "IB 2"])
        self.assertEqual([r[0] for r in reg["reviews"]], ["INR 1", "INR 2"])
        self.assertEqual([r[0] for r in reg["tools"]], ["T-1", "T-2"])
        self.assertEqual([f[0] for f in reg["fakes"]], ["A-1", "A-3"])

    def test_rows_outside_the_four_tables_are_ignored(self):
        reg = mk.parse_register(PROGRESS)
        self.assertNotIn("IB 99", [r[0] for r in reg["bugs"]])

    def test_an_escaped_pipe_stays_inside_its_cell(self):
        reg = mk.parse_register(PROGRESS)
        self.assertEqual(reg["bugs"][1][3], "Second bug with a pipe | inside")

    def test_numbers_sort_numerically_not_as_text(self):
        text = replace("| IB 1 | Module A", "| IB 10 | Module A")
        self.assertEqual([r[0] for r in mk.parse_register(text)["bugs"]], ["IB 2", "IB 10"])

    def test_audits_without_a_defect_id_are_kept_apart(self):
        reg = mk.parse_register(PROGRESS)
        self.assertEqual([r[0] for r in reg["audits_without_defect"]], ["`FakeOne`"])

    def test_the_last_table_ends_at_the_next_heading(self):
        text = replace("## Environment\n\ntext", "| X | Y | **A-9 defect** | z | w |\n\n## Environment\n\ntext")
        self.assertEqual(mk.parse_register(text)["fakes"][-1][0], "A-9")


class Errors(unittest.TestCase):
    def assertRegisterError(self, text, *parts):
        with self.assertRaises(mk.RegisterError) as ctx:
            mk.parse_register(text)
        for part in parts:
            self.assertIn(part, str(ctx.exception))

    def test_an_unescaped_pipe_is_an_error_naming_the_line(self):
        text = replace("pipe \\| inside", "pipe | inside")
        self.assertRegisterError(text, "Integration bugs", "IB 2", "8 cells, expected 7", "unescaped")

    def test_a_row_with_a_missing_column_is_an_error(self):
        self.assertRegisterError(replace("| `case a` | yes (WO I1-1) |", "| yes (WO I1-1) |"), "IB 1", "6 cells, expected 7")

    def test_a_duplicate_number_is_an_error(self):
        self.assertRegisterError(replace("| IB 1 | Module A", "| IB 2 | Module A"), "IB 2 appears twice")

    def test_a_duplicate_tool_or_review_number_is_an_error(self):
        self.assertRegisterError(replace("| T-2 |", "| T-1 |"), "T-1 appears twice")
        self.assertRegisterError(replace("| INR 2 |", "| INR 1 |"), "INR 1 appears twice")

    def test_a_duplicate_fake_defect_id_is_an_error(self):
        self.assertRegisterError(replace("**A-3 gap", "**A-1 gap"), "A-1 appears twice")

    def test_an_empty_number_is_an_error(self):
        self.assertRegisterError(replace("| IB 1 | Module A", "|  | Module A"), "empty number")

    def test_a_missing_table_is_an_error(self):
        self.assertRegisterError(replace("## Test tool defects", "## Tools"), "Test tool defects", "not found")

    def test_a_header_with_another_number_of_columns_is_an_error(self):
        self.assertRegisterError(replace("| # | Tool | Defect | Status |", "| # | Tool | Defect |"), "header row", "3 cells")

    def test_the_command_line_reports_an_error_with_status_2_and_writes_nothing(self):
        with tempfile.TemporaryDirectory() as tmp:
            progress = os.path.join(tmp, "PROGRESS.md")
            out = os.path.join(tmp, "BUG_REPORT.md")
            with open(progress, "w", encoding="utf-8") as f:
                f.write(replace("| IB 1 | Module A", "| IB 2 | Module A"))
            err = io.StringIO()
            with contextlib.redirect_stderr(err):
                rc = mk.main(["--progress", progress, "--out", out])
            self.assertEqual(rc, 2)
            self.assertIn("error:", err.getvalue())
            self.assertFalse(os.path.exists(out))

    def test_a_missing_progress_file_is_an_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            err = io.StringIO()
            with contextlib.redirect_stderr(err):
                rc = mk.main(["--progress", os.path.join(tmp, "nope.md"), "--out", os.path.join(tmp, "o.md")])
        self.assertEqual(rc, 2)
        self.assertIn("cannot read", err.getvalue())


class Content(unittest.TestCase):
    def setUp(self):
        self.text = mk.generate(PROGRESS)

    def test_the_header_says_the_file_is_generated_and_names_tool_and_tables(self):
        head = self.text.split("## Summary")[0]
        self.assertIn("Generated file, do not edit by hand", head)
        self.assertIn("Firmware/Tests/Integration_Tests/tools/make_integration_bug_report.py", head)
        for table in ("Integration bugs", "Integration NEEDS REVIEW items", "Test tool defects", "Fake audit register"):
            self.assertIn('"%s"' % table, head)
        self.assertIn("Tests/Integration_Tests/claude/plans/PROGRESS.md", head)

    def test_the_summary_counts_open_and_resolved_per_kind(self):
        summary = self.text.split("## Summary")[1].split("| # | Kind")[0]
        self.assertIn("| Suspected firmware bugs (IB) | 2 | 1 | 1 |", summary)
        self.assertIn("| NEEDS REVIEW items (INR) | 2 | 1 | 1 |", summary)
        self.assertIn("| Defects in shared test fakes (A) | 2 | 1 | 1 |", summary)
        self.assertIn("| Defects in test tools (T) | 2 | 1 | 1 |", summary)

    def test_the_index_lists_every_entry_with_its_status(self):
        self.assertIn("| IB 1 | firmware bug | Module A | resolved: yes (WO I1-1) |", self.text)
        self.assertIn("| IB 2 | firmware bug | Module B | open |", self.text)
        self.assertIn("| INR 2 | NEEDS REVIEW | HID | resolved: resolved 2026-11-19 |", self.text)
        self.assertIn("| A-1 | fake defect | Fake SPI | fixed |", self.text)
        self.assertIn("| A-3 | fake defect | Fake RTOS | open |", self.text)
        self.assertIn("| T-1 | tool defect | `tools/x.py` | open |", self.text)
        self.assertIn("| T-2 | tool defect | `tools/y.py` | resolved |", self.text)

    def test_every_column_of_a_bug_row_is_in_its_entry(self):
        entry = self.text.split("### IB 2: Module B")[1].split("### ")[0]
        self.assertIn("- **Where:** `b.cpp:2`", entry)
        self.assertIn("- **What:** Second bug with a pipe | inside", entry)
        self.assertIn("- **Same as (coverage):** bug 201", entry)
        self.assertIn("- **Test / evidence:** `case b`", entry)
        self.assertIn("- **Resolved?:** no", entry)

    def test_an_empty_same_as_cell_reads_none(self):
        entry = self.text.split("### IB 1: Module A")[1].split("### ")[0]
        self.assertIn("- **Same as (coverage):** (none)", entry)

    def test_every_column_of_a_review_row_is_in_its_entry(self):
        entry = self.text.split("### INR 2: HID")[1].split("## ")[0]
        self.assertIn("- **Test / location:** `d.h`", entry)
        self.assertIn("- **Question:** Another question", entry)
        self.assertIn("- **Same as (coverage):** NR 7", entry)
        self.assertIn("- **Resolved?:** resolved 2026-11-19", entry)

    def test_a_row_naming_an_iq_links_to_the_open_questions_file(self):
        entry = self.text.split("### INR 1: Axis")[1].split("### ")[0]
        self.assertIn("Asked as [IQ6](OPEN_QUESTIONS.md)", entry)
        tool = self.text.split("### T-1: ")[1].split("### ")[0]
        self.assertIn("(owner, [IQ7](OPEN_QUESTIONS.md))", tool)

    def test_a_row_without_an_iq_has_no_link_in_its_entry(self):
        entry = self.text.split("### IB 2: Module B")[1].split("### ")[0]
        self.assertNotIn("OPEN_QUESTIONS", entry)

    def test_the_fake_defects_show_audit_result_fix_cases_and_status(self):
        entry = self.text.split("### A-1: Fake SPI")[1].split("### ")[0]
        self.assertIn("- **Audit (WO, date):** I0-2, 2026-11-18", entry)
        self.assertIn("- **Result:** **A-1 defect**: wrong error delivered", entry)
        self.assertIn("- **Fix (WO, owner approval):** fixed in WO I0-3a", entry)
        self.assertIn("- **Coverage cases changed:** 2 cases edited", entry)
        self.assertIn("- **Status:** fixed", entry)
        self.assertIn("- **Status:** open", self.text.split("### A-3: Fake RTOS")[1].split("### ")[0])

    def test_audits_without_a_defect_are_listed_after_the_defects(self):
        self.assertIn("Audited without a defect:", self.text)
        self.assertIn("- `FakeOne` (I0-2, 2026-11-18): no defect; gaps handled Fix: none proposed. Coverage cases changed: none.", self.text)

    def test_the_tool_defect_entries_carry_defect_and_status(self):
        entry = self.text.split("### T-2: `tools/y.py`")[1]
        self.assertIn("- **Defect:** Another", entry)
        self.assertIn("- **Status:** fixed in WO I1-2", entry)

    def test_the_output_is_deterministic_and_ends_with_one_newline(self):
        self.assertEqual(self.text, mk.generate(PROGRESS))
        self.assertTrue(self.text.endswith("\n") and not self.text.endswith("\n\n"))
        self.assertNotIn("\r", self.text)

    def test_empty_tables_give_none_sections(self):
        text = replace("| IB 2 | Module B | `b.cpp:2` | Second bug with a pipe \\| inside | bug 201 | `case b` | no |\n", "")
        text = replace("| IB 1 | Module A | `a.cpp:1` | First bug | | `case a` | yes (WO I1-1) |\n", "", text)
        out = mk.generate(text)
        self.assertIn("## 1. Suspected firmware bugs (IB)\n\nNone.\n", out)
        self.assertIn("| Suspected firmware bugs (IB) | 0 | 0 | 0 |", out)


class CommandLine(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.progress = os.path.join(self.tmp.name, "plans", "PROGRESS.md")
        self.out = os.path.join(self.tmp.name, "reports", "BUG_REPORT.md")
        os.makedirs(os.path.dirname(self.progress))
        os.makedirs(os.path.dirname(self.out))
        with open(self.progress, "w", encoding="utf-8", newline="\n") as f:
            f.write(PROGRESS)

    def run_main(self, *extra):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = mk.main(["--progress", self.progress, "--out", self.out] + list(extra))
        return rc, out.getvalue(), err.getvalue()

    def read(self):
        with open(self.out, encoding="utf-8", newline="") as f:
            return f.read()

    def test_the_report_is_written_with_lf_line_ends(self):
        rc, out, _ = self.run_main()
        self.assertEqual(rc, 0)
        self.assertEqual(self.read(), mk.generate(PROGRESS))
        with open(self.out, "rb") as f:
            self.assertNotIn(b"\r", f.read())

    def test_check_passes_for_a_current_file_and_writes_nothing(self):
        self.run_main()
        before = os.stat(self.out).st_mtime_ns
        rc, out, err = self.run_main("--check")
        self.assertEqual((rc, err), (0, ""))
        self.assertIn("current", out)
        self.assertEqual(os.stat(self.out).st_mtime_ns, before)

    def test_check_fails_with_a_message_when_the_file_is_stale(self):
        self.run_main()
        with open(self.progress, "w", encoding="utf-8", newline="\n") as f:
            f.write(replace("| no |\n| INR 2", "| answered |\n| INR 2"))
        stale = self.read()
        rc, out, err = self.run_main("--check")
        self.assertEqual(rc, 1)
        self.assertIn("stale", err)
        self.assertIn("make_integration_bug_report.py", err)
        self.assertEqual(self.read(), stale)

    def test_check_fails_for_a_hand_written_file(self):
        with open(self.out, "w", encoding="utf-8") as f:
            f.write("# hand written\n")
        rc, _, err = self.run_main("--check")
        self.assertEqual(rc, 1)
        self.assertIn("stale", err)

    def test_check_fails_for_a_missing_file(self):
        rc, _, err = self.run_main("--check")
        self.assertEqual(rc, 1)
        self.assertIn("missing", err)

    def test_check_accepts_a_file_with_windows_line_ends(self):
        with open(self.out, "w", encoding="utf-8", newline="\r\n") as f:
            f.write(mk.generate(PROGRESS))
        rc, _, _ = self.run_main("--check")
        self.assertEqual(rc, 0)

    def test_a_progress_file_with_windows_line_ends_gives_the_same_report(self):
        with open(self.progress, "w", encoding="utf-8", newline="\r\n") as f:
            f.write(PROGRESS)
        self.run_main()
        self.assertEqual(self.read(), mk.generate(PROGRESS))

    def test_paths_with_spaces_are_accepted(self):
        folder = os.path.join(self.tmp.name, "a folder with spaces")
        os.makedirs(folder)
        self.out = os.path.join(folder, "BUG REPORT.md")
        rc, _, _ = self.run_main()
        self.assertEqual(rc, 0)
        self.assertTrue(os.path.isfile(self.out))

    def test_the_default_paths_are_the_integration_register_and_report(self):
        rel = lambda p: os.path.relpath(p, mk.REPO).replace(os.sep, "/")
        self.assertEqual(rel(mk.DEFAULT_PROGRESS), "Tests/Integration_Tests/claude/plans/PROGRESS.md")
        self.assertEqual(rel(mk.DEFAULT_OUT), "Tests/Integration_Tests/claude/reports/BUG_REPORT.md")
        self.assertTrue(os.path.isfile(os.path.join(mk.REPO, *mk.TOOL_NAME.split("/"))))


class StatusAndHeaderRules(unittest.TestCase):
    def status_of(self, cell, table="tools"):
        old = {"tools": "| fixed in WO I1-2 |", "bugs": "| yes (WO I1-1) |", "reviews": "| resolved 2026-11-19 |"}[table]
        reg = mk.parse_register(replace(old, "| %s |" % cell))
        row = {"tools": reg["tools"][1], "bugs": reg["bugs"][0], "reviews": reg["reviews"][1]}[table]
        return mk.is_open({"tools": "tool", "bugs": "bug", "reviews": "review"}[table], row)

    def test_only_an_explicit_value_at_the_start_of_the_status_resolves_a_row(self):
        for value in ("yes", "Yes (WO I1-1)", "fixed in WO I2", "Fixed", "resolved"):
            self.assertFalse(self.status_of(value), value)
        for value in ("open", "planned", "in progress", "partly fixed", "open, fixed in WO I0-8", "", "no", "not fixed", "unresolved"):
            self.assertTrue(self.status_of(value), value)

    def test_the_default_of_every_register_is_open_for_an_unknown_value(self):
        self.assertTrue(self.status_of("planned", "bugs"))
        self.assertTrue(self.status_of("later", "reviews"))
        self.assertFalse(self.status_of("yes", "bugs"))
        self.assertFalse(self.status_of("resolved 2026-11-19", "reviews"))

    def test_a_fake_defect_is_fixed_only_when_the_fix_column_says_fixed(self):
        text = replace("| not fixed yet |", "| planned in WO I1-3 |")
        out = mk.generate(text)
        self.assertIn("| A-3 | fake defect | Fake RTOS | open |", out)
        self.assertIn("| A-1 | fake defect | Fake SPI | fixed |", out)

    def test_a_header_with_the_wrong_column_names_is_an_error(self):
        text = replace("| # | Tool | Defect | Status |", "| # | Tool | Defect | State |")
        with self.assertRaises(mk.RegisterError) as ctx:
            mk.parse_register(text)
        self.assertIn("header row", str(ctx.exception))
        self.assertIn("State", str(ctx.exception))

    def test_header_names_are_compared_without_case_and_extra_blanks(self):
        mk.parse_register(replace("| # | Tool | Defect | Status |", "|  #  | tool |  DEFECT | Status  |"))

    def test_whitespace_in_a_number_is_normalised_before_the_duplicate_check(self):
        with self.assertRaises(mk.RegisterError) as ctx:
            mk.parse_register(replace("| IB 1 | Module A", "| IB   2 | Module A"))
        self.assertIn("IB 2 appears twice", str(ctx.exception))

    def test_a_number_with_the_prefix_of_another_table_is_an_error(self):
        for old, new in (("| IB 1 | Module A", "| INR 1 | Module A"), ("| INR 1 | Axis", "| IB 5 | Axis"), ("| T-2 |", "| IB 7 |"),
                         ("| IB 1 | Module A", "| 1 | Module A")):
            with self.assertRaises(mk.RegisterError, msg=new) as ctx:
                mk.parse_register(replace(old, new))
            self.assertIn("does not look like", str(ctx.exception))

    def test_an_escaped_pipe_in_a_module_does_not_break_the_summary_rows(self):
        text = replace("| Module B |", "| Module \\| B |")
        out = mk.generate(text)
        self.assertIn("| IB 2 | firmware bug | Module \\| B | open |", out)
        for line in out.splitlines():
            if line.startswith("| IB 2 |"):
                self.assertEqual(len([c for c in line.replace("\\|", "").split("|")]), 6)

    def test_the_evidence_column_of_a_bug_row_is_shown_as_test_evidence_and_not_as_same_as(self):
        out = mk.generate(replace("| bug 201 | `case b` |", "| bug 201 | `evidence of b` |"))
        entry = out.split("### IB 2: Module B")[1].split("### ")[0]
        self.assertIn("- **Test / evidence:** `evidence of b`", entry)
        self.assertIn("- **Same as (coverage):** bug 201", entry)


class CommittedReport(unittest.TestCase):
    """integration_tool_tests also runs --check on the committed report: it fails when a register changed and the report was not regenerated."""

    def test_the_committed_report_is_what_the_registers_generate(self):
        err = io.StringIO()
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(err):
            rc = mk.main(["--check"])
        self.assertEqual(rc, 0, err.getvalue().strip())


if __name__ == "__main__":
    unittest.main()
