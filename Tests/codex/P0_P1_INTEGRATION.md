> Edited by Claude 2026-11-09: this historical report mentions the CDX-008 ring-buffer exclusion/quarantine. That exclusion no longer exists (the Claude test was fixed instead). See `Tests/claude/reports/CODEX_REVIEW_AUDIT.md`. The text below is otherwise unchanged.

# Approved P0 tooling and P1 lifecycle integration

2026-11-06. Owner instruction: **“Apply both.”** Applied the exact previously reviewed P0_TOOLING_FIXES.patch and P1_TEST_INTEGRATION.patch. The earlier approved target-case registration remains in place. No further approval is pending for these three patches.

## Changes

- Coverage now retains executable denominators for notes-only firmware/tier-A/driver objects and rejects failed or unusable gcov collection.
- Baseline checking rejects duplicate identities and lost metadata; configured exclusions remain explicitly unchecked.
- Reporting rejects failed/malformed/empty required inventories and empty mandatory Python discovery, retaining legitimate unmatched internal probes.
- CMake's unit glob now registers the two Codex effect-lifecycle cases.
- Removed only the 23 resolved `expectedFailure` decorators from the owned P0 Python group, preserving assertions and finding comments. All 36 now pass as ordinary tests. This group is still invoked directly; the reviewed patches do not register it in CTest's existing Python entry.

The three shared tools and CMake changes match the approved patches. Firmware/vendor code, Claude-owned files, original 318 baseline cases and shared exclusion defaults remain unchanged. No dependency, Git configuration, commit or push.

## Verification

| Configuration | Unit passed / failed / skipped | Original identities / checked / excluded | CTest scope/result | New lifecycle cases |
|---|---|---|---|---|
| Default DSP | 647 / 0 / 5 | 318 / 317 / 1 | Full suite 9/9 | 2 ordinary passes /16 assertions |
| No DSP | 647 / 0 / 5 | 318 / 317 / 1 | Unit, baseline, Python 3/3 | 2 ordinary passes /16 assertions |
| GCC ASan/UBSan, DSP | 644 / 0 / 8 | 318 / 315 / 3 | Unit, baseline, Python 3/3 | 2 ordinary passes /16 assertions |
| GCC ASan/UBSan, no DSP | 646 / 0 / 6 | 318 / 317 / 1 | Unit, baseline, Python 3/3 | 2 ordinary passes /16 assertions |

Each checker reports 334 new identities, none failed. All CTest entries and focused lifecycle runs exit 0. Sanitizers remain enabled without suppressions. These verification builds explicitly add CDX-008's unsafe **nonbaseline** ring case to their local CMake exclusion cache, alongside normal exclusions. No shared exclusion default or original baseline identity was changed. This qualification is essential: the protected ring case is still unresolved.

Default's full CTest includes the existing standalone DSP regression executable; running it does not expand the P11 review scope. Other default executable totals are target 1823/0/18, flash 159/0/0, RTOS 271/0/0, USB 286/0/0, driver 929/0/9, standalone DSP 218/0/0. Doctest passes include known-issue expected failures. Existing Python tests: 115 run, 113 ordinary passes +2 expected failures. Codex P0 tests: 36 ordinary passes, 0 expected failures.

Applied report-tool compatibility checks list all six P0–P10 executables without executing firmware cases: 652, 1841, 159, 271, 286, 938 cases respectively. Each unmatched internal filter returns 0. Its actual Python subprocess returns 115 records and no run problems. No full forced-skip report was run.

Applied coverage tool against the frozen existing coverage artifacts exits0: 87 compiled firmware-ladder files, 8007/8095 lines (98.9%), 0 not compiled. These are baseline artifact measurements, not new-test coverage or an improvement claim. The independently tested missing-runtime denominator defect remains covered by the 36 regressions; current full-artifact totals were not demonstrated wrong.

## Evidence

Builds: `../ffb-test-build/codex-integrated-target-{default,nodsp,san,nodsp-san}`. Prior target-registration evidence is preserved separately. New `codex-p0-p1-integration-evidence/` directories contain `reproduce.py`, exact `commands.json`, configure/build/CTest logs, copied `last-test.log` and focused XML. Default also retains `codex-python.log`, `coverage.log`, `report_compat.py`, `report-compat-results.json` and raw inventory/Python files under `report-compat/`.

The current integration results supersede proposal-only and marker-pending statements in historical author/QC reports; those submission-time records remain attributed history. See REGRESSION_RESULTS for the full chain. Hardware, target execution, all possible report-recovery scenarios and unresolved protected-test/fake issues remain outside this verification.
