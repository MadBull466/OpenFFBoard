# Independent review findings

Current final dispositions: [CLOSEOUT_RESULTS](CLOSEOUT_RESULTS.md), accepted by [final QC](reviews/CLOSEOUT_FINAL_QC.md#final-integrated-release-acceptance). The statuses below describe earlier review/integration stages; this closeout addendum supersedes pending-regression/approval statements.

CDX-001–007 repairs are applied and routine regressions pass. CDX-008 is **no longer quarantined**; the protected Claude test was corrected and the safe Codex replacements remain [Edited by Claude 2026-11-09: the CDX-008 quarantine was removed from `Firmware/Tests/CMakeLists.txt`; the Claude test `typed access reads single bytes and counts elements` was fixed instead (empty-buffer check moved to its own case). See `Tests/claude/reports/CODEX_REVIEW_AUDIT.md`.] CDX-009/010 accepted lifecycle/motor regressions remain integrated. CDX-011/012 permanent ADS/Tasklet discriminators are integrated and independently checked against proper/faulty repairs (CDX-012: the Claude tasklet case was also strengthened on 2026-11-09 so that only a repair that rejects the second call flips it; edited by Claude). CDX-013 shared Sync set-mask correction and controls are applied. CDX-014 now includes a reusable finite-aware predicate and PID regression, while protected reducers remain unchanged. CDX-015 shared provenance is corrected and local policy documented in ODRIVE_POLICY. CDX-016 remains a protected editorial issue with compensating coverage. CDX-017 has a permanent signed nondefault packing regression; no production Hall bug is alleged. Bounded SPI/USB/timer and caller-buffer observations are covered as detailed in the closeout results. Production/vendor defects and external/model limits remain open.

## Historical review findings and integration stages

Updated: 2026-11-06. Assigned P0–P10 review is complete; unresolved findings remain open. Detailed reproductions, affected source locations, expected/observed behavior, recommendations and limits are in the linked phase reports. The owner approved and applied P0 tooling, P1 unit registration and the four target-case registration; current verification is recorded in REGRESSION_RESULTS.

## CDX-001 — Python entry points still use the pre-move test directory

| Field | Value |
|---|---|
| Severity | Medium |
| Category | Confirmed test integration/discovery defect |
| Affected files | `Firmware/Tests/CMakeLists.txt:1026`; `Firmware/Tests/tools/run_all_tests.py:1209` |
| Related tests | CTest `python_helper_tests`; full-report Python helper discovery; all six `claude/python/test_*.py` files |
| Verification | Reproduced in every fresh baseline variant; after correction, default full CTest passes 9/9 and the report Python helper returns 115 results without problems |
| Resolution | Fixed on 2026-11-05 after the owner's explicit approval; two shared path expressions changed |

The ownership split moved the Python tests into `Firmware/Tests/claude/python/`. Both callers still pass `Firmware/Tests/python`, which does not exist. CTest's `-s` and `-t` arguments point there, and the full report constructs the same directory using `os.path.join(TESTS_DIR, "python")`.

Expected: discover and execute the current Python suite. Observed: CTest raises `ImportError: Start directory is not importable`, giving 8/9 entries passed with DSP and 7/8 without DSP. The full-report helper returns no results and a discovery problem. Invoking unittest directly at the current location executes 115 tests successfully (113 ordinary successes, two expected failures). The moved `helper_paths.py` already resolves its firmware root correctly.

Impact: the standard gate is red and the full report cannot include the Python suite through its current entry point. No evidence of failing Python test behaviour was found in the direct run.

Minimal correction, approved and applied:

```diff
--- Firmware/Tests/CMakeLists.txt
+++ Firmware/Tests/CMakeLists.txt
@@
-            -s ${CMAKE_CURRENT_SOURCE_DIR}/python -t ${CMAKE_CURRENT_SOURCE_DIR}/python -p "test_*.py"
+            -s ${CMAKE_CURRENT_SOURCE_DIR}/claude/python -t ${CMAKE_CURRENT_SOURCE_DIR}/claude/python -p "test_*.py"
--- Firmware/Tests/tools/run_all_tests.py
+++ Firmware/Tests/tools/run_all_tests.py
@@
-            py, py_problems = python_results(os.path.join(TESTS_DIR, "python"), workdir, args.timeout, args.python)
+            py, py_problems = python_results(os.path.join(TESTS_DIR, "claude", "python"), workdir, args.timeout, args.python)
```

Actual indentation was preserved. No executable, feature macro, assertion, skip, exclusion or protected Claude file changed. Consumers: the CTest entry and full-report custom target/script; the direct Python tests already worked. Verification: default build reconfigured and rebuilt; full CTest 9/9, including baseline preservation and all Python helper self-tests. Report Python helper: 115 results (113 passed, two expected failures), no problems. Other variants were not rerun after this path-only correction; their original baseline remains unchanged in BASELINE. A full no-skip report was not run.

Evidence: each `codex-baseline-<variant>/codex-baseline-evidence/ctest.log`, plus `codex-baseline-default/codex-baseline-evidence/python-direct.log` and `report-python.log`, under the required sibling build root. Commands: [TEST_EXECUTION.md](TEST_EXECUTION.md).

## Discovery notes, not adjudicated defects

- Older gate wording conflates the 318 original cases with the expanded executable. Current checker preservation and executed counts are documented in BASELINE.
- `COVERAGE_AND_SCOPE.md`'s TinyUSB 48.0% figure is the old smoke result; fresh coverage reproduces PROGRESS's P9-final 85.1%. Existing protected documentation remains unchanged.
- Approved Codex unit/target globs now register all six C++ cases. The original discovery phase did not create executable test code.
- Historic hardware/protocol questions and suspected firmware/vendor bugs remain unverified inputs. No severity assessment of those claims has been performed by Codex.

## Confirmed review findings

| ID | Severity | Finding and detailed evidence | Status |
|---|---|---|---|
| CDX-002 | High | Missing coverage runtime data can disappear from the denominator — [P0 T1](reviews/P0_REVIEW.md) | Approved/applied correction; 36 ordinary Codex regressions pass; current measured totals not shown wrong |
| CDX-003 | High | Coverage collector failure can still produce a successful report — [P0 T2](reviews/P0_REVIEW.md) | Approved/applied correction; 36 ordinary Codex regressions pass |
| CDX-004 | Medium | Baseline checker can miss lost sanitizer UB metadata — [P0 T3](reviews/P0_REVIEW.md) | Approved/applied correction; 36 ordinary Codex regressions pass |
| CDX-005 | Medium | Mandatory Python discovery can yield no cases without a report problem — [P0 T4](reviews/P0_REVIEW.md) | Approved/applied correction; 36 ordinary Codex regressions pass |
| CDX-006 | Medium | Failed or empty doctest inventory can be accepted — [P0 T5](reviews/P0_REVIEW.md) | Approved/applied correction; 36 ordinary Codex regressions pass |
| CDX-007 | Low | Duplicate baseline/report identities collapse silently — [P0 T6](reviews/P0_REVIEW.md) | Approved/applied correction; 36 ordinary Codex regressions pass; no present collision found |
| CDX-008 | High | Ordinary ring-buffer test evaluates an indeterminate return — [P1-001](reviews/P1_REVIEW.md) | Partially valid (copying an indeterminate unsigned char is defined C++; only MemorySanitizer flags it). Fixed in the Claude test, no exclusion. [Edited by Claude 2026-11-09: the CDX-008 quarantine was removed from `Firmware/Tests/CMakeLists.txt`; the Claude test `typed access reads single bytes and counts elements` was fixed instead (empty-buffer check moved to its own case). See `Tests/claude/reports/CODEX_REVIEW_AUDIT.md`.] |
| CDX-009 | Medium | Known-issue masking conceals a running-effect lifecycle regression — [P1-002](reviews/P1_REVIEW.md) | Two Codex regressions accepted by fresh QC and registered after approval; normal CMake verification passes in all four affected variants |
| CDX-010 | High | Critical-error tests do not distinguish a flag-only repair from motor inhibition — [P2-1](reviews/P2_REVIEW.md) | Reproduced; two permanent motor-side cases accepted, registered and verified in default/no-DSP target gates with required sanitizer skips |
| CDX-011 | Medium | ADS threshold known-issue test hides a valid serialized repair behind setup failure — [P5-1](reviews/P5_REVIEW.md) | Reproduced; protected original unchanged; permanent regression pending |

Findings concern tests, host models and tools. A historical firmware diagnosis is not independently endorsed merely because its marker is mentioned. P10 dispositions are final within the scope recorded below.

| ID | Severity | Finding and evidence | Status |
|---|---|---|---|
| CDX-012 | Medium | Tasklet timeout test cannot distinguish checked lock failure from unchecked scheduling — [P8-1](reviews/P8_REVIEW.md) | Reproduced with proper/unsafe repair pair and callback-payload probe; permanent regression pending |
| CDX-013 | Medium | EventGroup Sync fake accepts reserved set bits rejected by the real kernel — [P8-2](reviews/P8_REVIEW.md) | Public-interface probe and scratch fake correction reproduce; existing 271-case suite unchanged; shared correction needs approval |

## P9 numerical-oracle finding

| ID | Severity | Finding and evidence | Status |
|---|---|---|---|
| CDX-014 | Medium | DSP error maxima ignore NaN results — [P9](reviews/P9_REVIEW.md) | NaN output corruptions escape the entire original CmsisDsp selection; identical finite-error controls fail. Fresh QC accepted two Codex finite-output guards, now registered/verified; protected aggregators remain unchanged and coverage is limited to those workloads |

## Driver oracle and scope findings (fresh QC accepted)

| ID | Severity | Finding and evidence | Status / recommendation |
|---|---|---|---|
| CDX-015 | Medium | ODrive tests/peer promote unsupported request-framing assumptions to vendor-spec checks — [P10](reviews/P10_REVIEW.md) | Retain recorded charter policy but attribute it truthfully; no current-frame defect established. Protected originals unchanged |
| CDX-016 | Low | O14 reply-promising title checks only requests — [P10](reviews/P10_REVIEW.md) | Local editorial/assertion-scope issue only; seven other cases detect suppressed replies. Rename or strengthen when owner edits; no duplicate high-priority test justified |
| CDX-017 | Low | Hall-angle tests accept hard-coded default first-pair packing — [P10](reviews/P10_REVIEW.md) | Public nondefault signed-input probe discriminates; no current production caller impact established. Supplementary signed-angle case recommended |

Final technical dispositions and independently repeated evidence: [FINAL_REVIEW_QC.md](reviews/FINAL_REVIEW_QC.md). CDX-011–014 are accepted within their stated scope. P2 motor cases and P9 finite-output guards passed fresh QC; the owner approved their shared target registration, now applied and verified through fresh build/CTest checks. Remaining firmware diagnoses, recovery policies and missing primary specifications are not resolved by review acceptance.

P0/P1 integration is complete after the owner's “Apply both” instruction. See [P0_P1_INTEGRATION.md](P0_P1_INTEGRATION.md): all 36 P0 tests are ordinary passes, both lifecycle cases are registered, default full CTest 9/9 and affected checks 3/3 in three other variants. Documented local ring exclusion applies; protected original issues remain qualified.

Before a new phase, use [P0_P10_CLOSEOUT.md](P0_P10_CLOSEOUT.md) to resolve or explicitly defer remaining gate, fake-fidelity and regression gaps. Review completion is not blanket issue closure.

## P12 addition

[P12-TICK-001](p12/FINDINGS.md): the real-C HAL_Delay can return early for a near-maximum finite delay with a coarse tick because wait addition wraps. Permanent marked regression, ordinary controls and scratch repair discrimination are recorded in [TIME_STATE_RESULTS](p12/TIME_STATE_RESULTS.md). This is separate from completed P0–P10 findings; no affected project caller or vendor repair is claimed.
