# WO I0-8: tools (as sent, 2026-11-18)

One `integration-test-writer` run. File list: `wo/I0-8.files`. Approved by the owner on 2026-11-18 (edits of two coverage-suite tools, their Python tests and one `CMakeLists.txt` line, listed in `APPROVED_SHARED_EDITS.txt`).

## Order

You extend the test tools of the OpenFFBoard host test suite for the integration track and fix one tool defect. Repository root `/home/ben/Documents/OpenFFBoard/testing/OpenFFBoard`, branch `add-test-suite`. Python 3 standard library only, run with `python3 -B`; the tools must keep working on Windows (paths, no POSIX-only calls in the tools themselves).

Read first: `CLAUDE.md` (rules, "Integration track"), `Firmware/Tests/README.md` (tool section), `Tests/Integration_Tests/claude/plans/INTEGRATION_TEST_PLAN.md` sections 3.2 (tools paragraph) and 10 (register and marker formats), `Tests/Integration_Tests/claude/plans/COVERAGE_AND_SCOPE.md` section 1 (chain line coverage, the F1 chain file list), the four tables of `Tests/Integration_Tests/claude/plans/PROGRESS.md` ("Integration bugs", "Integration NEEDS REVIEW items", "Test tool defects", "Fake audit register"), the current hand-written `Tests/Integration_Tests/claude/reports/BUG_REPORT.md` (the content the generator must produce, in generated form), and the tools `Firmware/Tests/Code_Coverage/tools/{run_all_tests.py,coverage_report.py,make_bug_report.py,check_baseline.py}` with their tests in `Firmware/Tests/Code_Coverage/claude/python/`.

You may create or edit only the files in `Tests/Integration_Tests/claude/charters/wo/I0-8.files`. Never edit documents under `Tests/` (the lead regenerates the report and keeps the registers). The coverage tools' existing behaviour must stay the same except where this order changes it; their existing tests must still pass.

### 1. `run_all_tests.py`

- Add `ffb_integration_tests` to `EXECUTABLES` and to `EXCLUDE_VARS` (`FFB_INTEGRATION_TESTS_EXCLUDE`, `FFB_INTEGRATION_TESTS_EXCLUDE_SANITIZE`; the 32-bit list `FFB_INTEGRATION_TESTS_EXCLUDE_32BIT` too, following how the tool treats `FFB_TARGET_TESTS_EXCLUDE_32BIT`). The integration cases' `IT_DELETES_FFBHIDMAIN` marker behaves like the coverage suite's `DELETES_FFBHIDMAIN` (find how the tool classifies that and match it; the macro is in `Firmware/Tests/Integration_Tests/support/IntegrationFixture.h`).
- Integration numbers: the tool learns `IB n` and `INR n` from `--integration-progress` (default `Tests/Integration_Tests/claude/plans/PROGRESS.md`, the tables "Integration bugs" and "Integration NEEDS REVIEW items"), the same way it learns bug / NR numbers from the coverage `PROGRESS.md`. A `KNOWN_ISSUE("(IB 3) ...")` / `KNOWN_ISSUE_UB("(IB 3) ...")` case shows IB 3; a `characterization:` case with `// INR n` beside it (check whether the tool can see source comments; it reads test locations) shows INR n; report them in the log with the integration register's description, like the coverage items. Integration numbers never collide with coverage numbers in the log.
- **Fix of tool defect T-1** (owner, IQ7; `PROGRESS.md` "Test tool defects"): (a) a skip-decorated case whose description marks a crash or memory corruption (e.g. "KNOWN ISSUE (crash)", "KNOWN ISSUE (memory corruption)") and that passes when run alone with `--no-skip` is classified like a passing skip-decorated UB case (bugged, "passes in this run; not evidence of a fix"), not as Failed (`run_all_tests.py:352-362`); (b) the vendor-bug pattern (`:390`) must not match driver-track labels like `B-V11`; such a label resolves to its firmware bug through the coverage register (the row whose evidence column contains `(B-V11)`: bugs 189 and 183 today). Tests for both in `test_run_all_tests.py`.
- Run the tool on the default build before and after your change (`python3 -B Firmware/Tests/Code_Coverage/tools/run_all_tests.py --build-dir /home/ben/Documents/OpenFFBoard/testing/ffb-test-build/it-default --out <file>`; the gate's build directory has every executable) and report the difference: expected only the three T-1 cases moving from Failed to Bugged with bugs 189 / 183, and the new `ffb_integration_tests` rows. Explain any other change.

### 2. `make_integration_bug_report.py` (new, `Firmware/Tests/Integration_Tests/tools/`)

- Generates `Tests/Integration_Tests/claude/reports/BUG_REPORT.md` from the four `PROGRESS.md` tables (integration bugs, NEEDS REVIEW items, test tool defects, fake audit register: the fake defects with their fix status). Output: a header saying the file is generated (tool name, source tables, "do not edit"), the summary table (counts per kind, open / resolved) and the per-entry list (IB, INR, fake defects, tool defects) with every column of the register row, a link to `OPEN_QUESTIONS.md` where a row names an IQ, and the cross-references ("Same as"). Rows with an unescaped `|`, missing columns or duplicate numbers are errors (follow `make_bug_report.py`'s conventions).
- Options: `--progress`, `--out` (defaults the two paths above), `--check` (exit 1 with a message when the file on disk differs from what would be generated; no write).
- `test_make_integration_bug_report.py` (stdlib `unittest`, in the same folder, run by `integration_tool_tests`): tables parsed from a synthetic `PROGRESS.md`, output content, `--check`, the error cases, Windows-safe paths.
- `integration.cmake`: `integration_tool_tests` also runs `make_integration_bug_report.py --check`, so ctest fails when the report is stale (the lead regenerates it whenever a register changes). Keep the WIN32 rule for `test_hooks.py` as it is.

### 3. `coverage_report.py`

- A new section "Integration (F1 chain)" computed from the `.gcda` files of `ffb_integration_tests` **alone** (its object directory, not the other executables'), for the F1 chain files listed in the integration `COVERAGE_AND_SCOPE.md` section 1 (put the list in the tool with a comment pointing to that section; CMSIS-DSP functions on the path by function name if the tool's gcov parsing can do that, otherwise per file and say so). Per file: lines, executed, percentage; and the total. The existing sections must be unchanged (compare the tool's output before / after on a coverage build if you can build one quickly; otherwise test with synthetic gcov output and say so).
- Tests in `test_coverage_report.py` (new, `Firmware/Tests/Code_Coverage/claude/python/`) with synthetic gcov data; they run in `python_helper_tests`.

### 4. `Firmware/Tests/CMakeLists.txt`

Only: the `test_report` custom target (around line 1154) also depends on `ffb_integration_tests` when that target exists (`if(TARGET ffb_integration_tests)`), with a one-line comment. Nothing else in that file.

### Verification

`python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --wo I0-8 --variants default,m32` must print `GATE PASS` (ctest includes `python_helper_tests` and `integration_tool_tests`). Note: `make_integration_bug_report.py --check` will fail in ctest until the lead regenerates the report: run the generator once yourself with `--out` into a scratch file outside the repository to show its output, and tell the lead; if `--check` makes the gate fail only because the committed report is the hand-written one, say so under "Lead decision needed" rather than editing the report. Write cmake build paths literally.

Answer format: files with one-line summaries; the run_all_tests changes and the before / after difference on the default build; the generator's output format (attach the scratch output path); the coverage_report section design; tests added; anything open under "Lead decision needed"; the gate summary. If you are close to your turn limit, stop and report what you have. The last line must be the gate's last line exactly as printed.
