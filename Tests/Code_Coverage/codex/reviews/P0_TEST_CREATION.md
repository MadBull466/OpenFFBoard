# P0 regression creation and candidate verification

2026-11-05. GPT-6.1 Sol, medium effort, `/root/p0_writer`. Assignment: six confirmed P0 false-green mechanisms, owned tests and approval-only shared-fix proposal. Read `Firmware/Tests/README.md` first, root `AGENTS.md`/`CLAUDE.md`, Codex agent workflow, lead charter/P0 review, all baseline checker code, relevant coverage collector/ladder/driver code, reporter inventory/Python code and existing helper loader/report tests. This is author evidence; fresh independent review remains pending.

## Scope and independent oracles

Permanent executable work is confined to `Firmware/Tests/Code_Coverage/codex/python/test_p0_tooling.py`. Fixtures use `TemporaryDirectory`; all stimuli run in `setUp`, outside expected-failure test bodies. Transport doubles supply process status and independent doctest/gcov documents. Production parsing, classifications, accounting and `main` return decisions run unchanged. Python discovery controls execute the real reporter subprocess driver. Nothing downloads, installs, sleeps or accesses hardware. No register/protocol facts are modeled.

| Requirement / finding | Assertion locations in test file | Independent distinction |
|---|---|---|
| CDX-002 / P0-T1 | 189,194; controls 230,234,238 | Two explicit executable line identities per source mean one run source + one unrun source yields 2/4, not 2/2. Driver notes likewise yield 0/2. Successful gcov empty-file JSON preserves data-only 0-line accounting. Separate objects covering complementary lines form a union, not doubled denominator. |
| CDX-003 / P0-T2 | 199,203,207,213,219,225 | Injected gcov exit 3 must reject both total and partial collection; one successful source cannot make the measurement complete. |
| CDX-004 / P0-T3 | 96; controls 90,103,107 | An ordinary skip has the same sanitizer outcome as a conditional UB skip but lacks its original category. Matching pass/known/skip/UB cases pass; wildcard exclusions retain their documented unchecked result policy. |
| CDX-005 / P0-T4 | 291; control 294 | Existing empty test directory is not a mandatory workload. One actual unittest case succeeds and produces one passed result. No invariant fixes the total at 115. |
| CDX-006 / P0-T5 | 301,306,311,315,319,324,329,334,339,344,349; controls 353,358,361 | Process failure/timeout cannot be overridden by syntactically valid XML. Primary workload must be nonempty with doctest root; a successful internal zero-match filter probe remains permitted. |
| CDX-007 / P0-T6 | 118,123; controls 99,110,114 | Identity is the original suite/name pair. Repeated catalogue/report identities require a diagnostic rejection; missing originals and nonzero passing-child status already fail. Print-current must remain independent of readable baseline input. |

Twenty-three specific defect tests carry `@unittest.expectedFailure` with both P0 and stable CDX IDs; thirteen controls remain ordinary. On current tools all twenty-three expected failures were inspected programmatically: every traceback ends in `AssertionError` from the intended test body, with zero setup errors. `/tmp/codex-p0-expected-failure-assertions.log` retains the tracebacks. These assertions distinguish confirmed mechanisms; they do not assert observed buggy output as desired behavior.

## Executed commands and evidence

Current tools:

```sh
python3 -B -m unittest discover -s Firmware/Tests/Code_Coverage/codex/python -v
```

Result: **36 run, 13 ordinary passes, 23 expected failures**, no unexpected successes/errors. `/tmp/codex-p0-original-tests.log`.

Candidate tools were generated and exercised solely under `/tmp/codex-p0-candidate/tools`. The temporary test file copies permanent source and removes only lines containing `@unittest.expectedFailure`; it retains all assertions and fixtures:

```sh
P0_TOOLS_DIR=/tmp/codex-p0-candidate/tools python3 -B -m unittest discover -s /tmp/codex-p0-candidate/tests -v
```

Result: **36 ordinary passes**. `/tmp/codex-p0-patched-tests.log`.

Existing compatibility suite:

```sh
python3 -B -m unittest discover -s /tmp/codex-p0-compat/Firmware/Tests/claude/python -v
```

Result: **115 run, 113 ordinary passes, 2 original expected failures**. `/tmp/codex-p0-compat.log`. Existing tests/loader were copied byte-for-byte into the temporary matching layout; its tools symlink selects candidates, and firmware helper symlinks select original read-only source. No Claude file was edited.

Real gcov verification:

```sh
python3 -B /tmp/codex-p0-candidate/tools/coverage_report.py --build-dir ../ffb-test-build/codex-baseline-cov --firmware-dir Firmware --no-exclusions
python3 -B /tmp/codex-p0-candidate/tools/coverage_report.py --build-dir /tmp/codex-p0-real-gcov --firmware-dir Firmware --no-exclusions
```

Both exit 0 with empty stderr. Complete current tree: **87 compiled ladder files; 8007/8095 (98.9%); 0 not compiled**. Reduced missing-runtime reproduction: **HidFFB 0/237; combined 136/373 (36.5%)**, correcting its original missing denominator. Both real data-only notes probes (`eeprom_addresses.c`, `TMC4671_configs.cpp`) returned status 0 and JSON `files=[]`; current driver table preserves TMC config 0/0. Logs: `/tmp/codex-p0-candidate-{current,missing}-coverage.md` and corresponding `.err`.

Baseline preservation (no C++ rebuild needed: only owned Python tests/proposal changed):

```sh
python3 -B Firmware/Tests/Code_Coverage/tools/check_baseline.py --build-dir ../ffb-test-build/codex-baseline-default
python3 -B /tmp/codex-p0-candidate/tools/check_baseline.py --build-dir ../ffb-test-build/codex-baseline-default --baseline Firmware/Tests/Code_Coverage/baseline/test_cases.txt
```

Both exit 0: **318 baseline identities, 317 checked, 1 excluded; 646 passed / 0 failed / 4 skipped**, 332 new cases. The unchanged exclusion is `requesting the forces of a not existing axis is rejected`. Logs: `/tmp/codex-p0-{original,candidate}-baseline.log`.

## Initial QC corrections and discrimination evidence

Addressed lead `INITIAL_QC.md` holds QC-1 through QC-3 for independent re-review. Inventory process-status cases now use a valid nonempty direct-list document with one named file/positive-line record and final `unskipped="1"` summary. They call the internal probe path and assert `RuntimeError` plus the exact `--list-test-cases failed (exit status 17)` or `(timeout)` diagnostic. Thus the primary empty-count guard cannot satisfy these cases. A scratch mutant at `/tmp/codex-p0-status-mutant/tools` removes only the inventory process-status guard; both named cases fail at their intended rejection assertions with the valid inventory returned. Two-case mutation run: **2 failures, 0 errors**; `/tmp/codex-p0-status-mutant.log`. Unmutated candidate controls pass both.

Added ordinary successful primary execution control, retained empty-internal control and empty-primary rejection. Seven added malformed-list tests require controlled `RuntimeError` with an invalid-inventory diagnostic: absent identity, nested full-run suite, noninteger line, zero line, absent summary, wrong count, and direct case containing run results. The helper records an existing raw `ValueError` for malformed lines as evidence, so it fails the intended RuntimeError assertion rather than becoming a setup error. The UB positive control now uses description plus skipped only, matching the sanitizer macro without should_fail.

Four added coverage cases supply a successful process with blank counter output, blank notes output, missing files field, or nonlist files field, each alongside a separately valid source. They require incomplete/nonzero and a gcov diagnostic. Explicit `files=[]` remains an ordinary data-only control. After these validation changes, both the 115-test compatibility suite and real full coverage check were rerun, retaining **113 ordinary +2 expected failures** and **8007/8095 (98.9%), 87 compiled files** respectively.

Real blank-command check:

```sh
python3 -B /tmp/codex-p0-candidate/tools/coverage_report.py --build-dir /tmp/codex-p0-real-gcov --firmware-dir Firmware --no-exclusions --gcov /bin/true
```

Observed **exit 1**, `Coverage report incomplete: invalid gcov output ... no JSON documents`; `/tmp/codex-p0-blank-gcov.log`. The baseline and reduced missing-counter normal-gcov checks above are retained pre-QC evidence, rather than claims of fresh executions after every change. Current 36-test expected-failure inspection finds **23 intended AssertionErrors, 0 errors, 0 unexpected failures**.

The additional notes scan covers firmware, tier A and driver sources only. It does not extend coverage to P9 vendor tiers or P11. The CDX-005 Python workload guard applies to the full reporter; independently invoked CMake unittest discovery still has ordinary unittest's zero-discovery behavior.

## Boundaries and remaining work

Candidate behavior/scope and the approval artifact are in `Tests/Code_Coverage/codex/proposals/P0_TOOLING_FIXES.md` and `.patch`. The proposed diff touches only the three shared Python tools and is not applied. Already approved Python-path edits and other agents' work were left intact; no Git mutation occurred.

No firmware mutation, new C++ build, new sanitizer run, full no-skip executable report, target/hardware experiment, or exhaustive platform/compiler validation was performed. The current real report validates compatibility of this coverage artifact tree; it does not establish coverage effectiveness. Standalone DSP static-library notes remain outside extra notes collection under its existing linked-data-only policy. Test fixtures intentionally cover selected safety boundaries rather than every gcov JSON schema or inventory schema field. Fresh independent assertion/mutation review and shared-change approval are coordinator/owner next steps.

## Coordinator application update — 2026-11-06

The owner explicitly said “Apply both.” The reviewed P0 tool and P1 unit-glob patches are applied;23resolved owned Python decorators removed. All36P0regressions pass ordinarily and both lifecycle cases pass through normal CMake in default/no-DSP and both sanitizer variants. Default full CTest 9/9; affected checks 3/3 elsewhere, with documented build-local CDX-008 exclusion. Exact counts/evidence and limitations: ../P0_P1_INTEGRATION.md. Earlier proposal-only statements above are submission-time history.
