# Independent P0 safety-net and tooling review

Date: 2026-11-05. Reviewer: fresh GPT-6.1 Sol, high reasoning, task `/root/p0_reviewer`. Assignment: independently audit P0 baseline preservation, exclusions, coverage accounting and report-tool safety claims. Read-only review of protected files; this report is the only repository change made by this reviewer. Temporary finding IDs below require coordinator assignment.

## Result

The existing default and sanitizer baseline checks pass with their documented exclusions. The catalogue contains 318 unique suite/name pairs: 300 pass, 15 known_issue, 2 known_issue_ub and 1 skip. Confirmed fault-injection findings show that coverage and full-report tools can produce successful incomplete results, and the baseline checker can miss marker loss and duplicate identities. These are tooling findings, not demonstrated firmware failures or evidence that the current measured coverage is wrong.

The already authorised Python path corrections in CMake and `run_all_tests.py` were retained. No shared fix or new permanent test was made.

## Reviewed material

- Instructions/context: `Firmware/Tests/README.md`, root `AGENTS.md` and `CLAUDE.md`, `Tests/codex/AGENT_WORKFLOW.md`, `Tests/codex/BASELINE.md`, P0 and adjacent requirements in `Tests/claude/plans/TEST_PLAN.md`, P0 results/gate/review references in `Tests/claude/plans/PROGRESS.md`.
- All of `Firmware/Tests/tools/check_baseline.py`, `coverage_report.py`, `coverage_exclusions.txt` and `baseline/test_cases.txt`.
- `Firmware/Tests/CMakeLists.txt`: baseline registration, effective exclusion generation, per-executable exclusion patterns and Python/full-report registration; relevant source globs and coverage target routing.
- `Firmware/Tests/tools/run_all_tests.py`: XML/inventory parsing, result classification, execution/resumption/isolation, Python subprocess/result collection, source-attribution contract and main exit policy. Existing synthetic report tests in `Firmware/Tests/claude/python/test_run_all_tests.py`; existing Python helper suite run in full.
- `Firmware/Tests/support/TestHelpers.h`: KNOWN_ISSUE and KNOWN_ISSUE_UB expansion, read-only.

## Confirmed findings

### P0-T1 — Missing runtime coverage is counted as data-only code (high)

Location: `Firmware/Tests/tools/coverage_report.py:355-362` (and analogous driver-track fallback near line 412).

When a source has a matching `.gcno` but no row in collected `.gcda` data, the script assigns it zero executable lines. A notes file proves compilation, not that its source contains only data. An executable that was built but never run therefore loses its executable-line denominator and is shown among compiled files. This can inflate coverage while hiding the absence of its workload.

Reproduced twice:

1. Synthetic firmware tree with two executable sources and matching CMake-shaped notes paths; runtime JSON includes only CmdParser (2/2 lines). CLI exits 0 and reports two compiled files and 100% (2/2), with no uncompiled files. HidFFB has no data but contributes zero denominator.
2. Real existing GCC coverage artifacts, copied without changing originals: notes and counters for `ffb_unit_tests` CmdParser, notes only for HidFFB. Real gcov exits cleanly; report exits 0, reports two compiled files and 136/136 covered. HidFFB is absent from the per-source and not-compiled tables, despite its real executable code. This excludes data-generation/source-shape assumptions from the reproduction.

Artifacts: [/tmp/codex-p0-real-gcov/report.md](/tmp/codex-p0-real-gcov/report.md), copied notes/counters under that directory, and [/tmp/codex-p0-probes/coverage_missing_gcda.md](/tmp/codex-p0-probes/coverage_missing_gcda.md).

Recommended correction: read notes through gcov to obtain zero-count executable lines for compiled-but-unrun objects, or fail/explicitly mark missing data as incomplete. Preserve genuine data-only objects only when their zero executable lines are established by gcov. Add regressions for a missing executable workload and a genuine data-only object. The analogous driver fallback needs the same distinction. No line-number exclusion or path-attribution patch alone fixes this root cause.

### P0-T2 — gcov subprocess failure still produces a successful report (high)

Location: `Firmware/Tests/tools/coverage_report.py:164-168`.

The collector logs a failing gcov invocation and continues; main has no failure/completeness status to propagate. With one input `.gcda` and a synthetic gcov command exiting 3, the report exits 0 and prints a coverage report with 0/0 executable lines. With mixed successful/failed inputs the same mechanism can silently omit only the failing source. P0-T1 further classifies corresponding notes as zero-line compiled files, but continuing successfully is an independent defect even if that fallback is fixed.

Artifact: [/tmp/codex-p0-probes/coverage_gcov_error.md](/tmp/codex-p0-probes/coverage_gcov_error.md). The stderr explicitly contains `gcov failed` and the injected diagnostic; process exit remains 0.

Recommended correction: propagate any collection failure to a nonzero CLI status and mark output incomplete; test full and partial gcov failure. A missing or incompatible gcov should not constitute a successful coverage measurement.

### P0-T3 — Sanitizer baseline accepts loss of the UB marker (medium)

Location: `Firmware/Tests/tools/check_baseline.py:103-110`, `:188-196`.

`classify` computes a metadata category, but validation compares only actual outcome. For the original nonexcluded HidFFB case `set condition with a parameter block offset above the axis count does not corrupt the effect`, replace the real sanitizer XML's UB description with an ordinary skip while retaining its name and skipped state. The original baseline remains unchanged. The checker exits 0 and says 315 classifications checked/3 excluded. Replacing a conditional sanitizer-only UB skip with a permanent ordinary skip should be detected as a marker/category change; its result in this one configuration is still skipped.

Artifact: [/tmp/codex-p0-probes/ub_marker_replaced_by_plain_skip.log](/tmp/codex-p0-probes/ub_marker_replaced_by_plain_skip.log), matching XML beside it. A matching nonsanitizer gate can expose a permanent skip, so this is not evidence that the complete current multivariant gate misses that exact mutation.

Recommended correction: compare expected metadata for nonexcluded cases as well as expected outcome. Excluded cases are explicitly result-unchecked; any new metadata invariant for them needs a documented policy. Test marker removal with actual reporter-shaped sanitizer XML.

### P0-T4 — Full reporter accepts empty Python discovery (medium)

Location: `Firmware/Tests/tools/run_all_tests.py:911-935`, `:1207-1212`.

`python_results` discovers an existing empty directory, writes/reads `[]`, and returns zero results and zero problems. Main counts Python among executed components without requiring any discovered tests. If the directory remains present but tests are misplaced/renamed/deleted, the full report can succeed while executing no Python tests. This is distinct from the corrected nonexistent-directory path, which raises an import error and is reported as a problem.

Reproduction: call `python_results` on the empty directory `/tmp/codex-p0-probes/empty_python`; result `( [], [] )`. No test sleeps or installed packages are involved. Ordinary unittest also regards empty discovery as success, so plain CMake discovery has the same missing-workload weakness unless separately guarded.

Recommended correction: explicitly require nonzero discovery for this mandatory helper group and report zero-discovery as a run problem. Add a stdlib unittest with an empty existing directory. Do not hard-code the present total of 115 as a permanent invariant.

### P0-T5 — Full reporter ignores failing/empty doctest inventory (medium)

Location: `Firmware/Tests/tools/run_all_tests.py:753-758`, `:760-766`.

`inventory` only rejects XML parse failures; it ignores subprocess return code and timeout status. Injecting `(exit 17, not timed out, 'startup failed', valid empty doctest XML)` returns an empty inventory. `run()` then returns no results and no problems. Main considers that executable run, so a full report containing only such an executable and `--no-python` can succeed with zero tests.

The minimal reproduction subclasses ExeRunner only to return the above process result; production parser/run logic executes unchanged. This reproduces the decision error, not a claim that the present binaries emit this malformed-success pattern.

Recommended correction: reject nonzero/timeout inventory before parsing, require expected XML root/schema and nonempty primary inventories, while permitting empty inventories for legitimate internal filter probes. Add process-status and zero-primary-inventory regressions.

### P0-T6 — Duplicate baseline identity is silently collapsed (low)

Location: `Firmware/Tests/tools/check_baseline.py:122`.

Appending a duplicate original `CmdParser / get command` XML case still returns `RESULT: OK`; the dictionary overwrites by `(suite,name)`. Different locations do not distinguish collisions. The baseline reader also accepts duplicate entries. This does not currently affect real input: the stored baseline has 318 unique keys and the observed default/sanitizer XML each has 650 unique keys. Nonzero executable exits are separately rejected, so a duplicate assertion failure that causes exit 1 is not generally hidden.

Recommended correction: reject duplicate original keys in baseline and report XML, rather than changing the identity contract without review. Test both inputs and avoid claiming this currently masks a real failing test.

## Executed checks and controls

Commands ran with Python `-B`; scratch fixtures/logs were kept under `/tmp`. Reproducer: [/tmp/codex_p0_probes.py](/tmp/codex_p0_probes.py). It imports production tooling, mutates copies of actual XML, and runs the real baseline CLI through a small output-only executable. Its standalone synthetic coverage tree is solely for accounting tests; it contains no hardware register/protocol claims.

| Check | Observed result |
|---|---|
| Real default baseline checker | Exit 0; 646 doctest passed, 0 failed, 4 skipped; 317 checked and effectsForces excluded; 332 new cases |
| Real sanitizer baseline checker, outside ptrace sandbox | Exit 0; 643/0/7; 315 checked; effectsForces plus two spline cases excluded; 332 new cases |
| Existing Claude Python helper suite | 115 tests; 113 ordinary successes, 2 expected failures; exit 0 |
| Captured real default and sanitizer XML | Each 650 unique suite/name identities; real UB metadata agrees with TestHelpers expansion |
| Baseline control replay | Exit 0 |
| Remove one original case from XML | Exit 1, MISSING |
| Child exits 17 with otherwise passing XML | Checker exit 1 |
| Crash exception in original case | Checker exit 2 |
| Empty doctest XML against real baseline | Exit 1; original cases missing |
| Truncated malformed XML | Exit 2 |
| Remove UB marker but retain sanitizer skip | Exit 0 (P0-T3) |
| Duplicate original identity | Exit 0 (P0-T6) |
| Missing runtime coverage, synthetic + real artifacts | Exit 0; zero-denominator missing source (P0-T1) |
| gcov failure | Exit 0 (P0-T2) |
| Empty Python discovery / failing empty doctest inventory | Zero results, no problems (P0-T4/T5) |

An initial sandboxed sanitizer execution returned 1 after all classifications passed: LeakSanitizer reported a fatal error because ptrace prevents its operation. The authorised unsandboxed rerun above passed without disabling or suppressing LeakSanitizer. This was an execution-environment constraint, not a source finding. One initial relative-path baseline invocation from `/tmp` failed to locate the script; the corrected absolute-path invocation is the reported default result.

## Integration, source attribution and limitations

- CMake caches the effective unit-test exclusion argument and passes the sanitizer mode to its baseline entry. Its extra spline exclusions require both sanitizer and DSP. Reviewed logic agrees with documented policy; no new defect was reproduced there. Each exclusion deliberately leaves that original result unchecked.
- The baseline checker does reject current malformed XML, missing originals, nonzero child statuses and explicit crashes. It does not prove that a known-issue failure's first assertion matches the intended defect; `should_fail` success alone is its oracle. The full reporter likewise labels known failures by metadata, not semantic assertion matching. First-failure correctness needs per-case audit in later phases.
- Coverage merges line numbers by normalized path and sums execution counts before testing positivity, which appropriately unions observed coverage across targets. Standalone DSP targets are separated from the historical tier C. These paths were reviewed, not independently validated across every compiler/operating system or symlink/case-normalization combination.
- The hardware-only exclusion file currently contains comments only, so no active exclusion range influenced the reproduced real coverage. The parsing/filtering logic was read; invalid-range/path edge cases were not exhaustively tested.
- The existing full-report source lookup is heuristic and can yield `unknown`; inferred file-under-test labels are not requirements traceability or proof of coverage. No new specific erroneous source label was established during this bounded review.
- Stale post-move documentation remains: the baseline catalogue comment points at `Tests/TEST_PLAN.md`, and README Python discovery examples/table still mention `python/` despite the corrected executable paths using `claude/python`. These are documentation discrepancies, not a renewed Python-path execution failure. Protected files were left unchanged.
- This review did not run the entire no-skip report, force known crash cases, rebuild the seven configurations, mutate firmware, inspect all 318 test bodies, measure branch/state coverage, inspect specifications or verify hardware/target behavior. The original suite's existing real-time wait remains protected and was not changed. Current successful coverage totals are not independently re-certified by this review.

Next work: coordinator-approved Codex-owned stdlib regressions for the six mechanisms, independent review of those regressions, and narrowly scoped proposed shared fixes for owner approval. Do not silently broaden this review into edits of the original baseline, firmware, vendor code or Claude-owned material.
