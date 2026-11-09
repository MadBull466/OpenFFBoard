# Regression results

> Current status (2026-11-06): the authorized closeout is complete; see [CLOSEOUT_RESULTS](CLOSEOUT_RESULTS.md) and [final QC](reviews/CLOSEOUT_FINAL_QC.md#final-integrated-release-acceptance). Earlier counts and pending statements below are historical staged evidence.

The original discovery configurations establish the pre-change baseline in [BASELINE.md](BASELINE.md). After the owner approved CDX-001, two shared path expressions were corrected to point to `claude/python`.

| Check | Before | After |
|---|---|---|
| Default full build | Succeeded | Reconfigured/rebuilt successfully |
| Default CTest | 8/9 | 9/9 |
| Original-case checker | 318 names present; 317 classifications checked, one excluded | Same, passes |
| Default C++ executables | 4,328 doctest passes (267 failed as expected), zero failures, 31 skipped/excluded | Same counts |
| Python CTest entry | Discovery ImportError | 115 tests, two expected failures; no unexpected failures |
| Report tool Python helper | Zero results, discovery problem | 115 results: 113 passed, two expected failures; no problems |

No new test cases were added by that path correction. Existing Python report/summariser self-tests run within the 115-test suite. No protected Claude file, production firmware, skip or exclusion changed. `git diff --check` passes. Remaining build variants, sanitizer variants, coverage and full no-skip report were not rerun after this path-only correction.

Evidence: `../ffb-test-build/codex-baseline-default/codex-pathfix-evidence/{configure,build,ctest,report-python}.log`. No commits or pushes.

## Supplemental review validation (shared proposals unapplied)

P1 creation report records two lifecycle cases/16 assertions passing in default, no-DSP and both sanitizer variants. The stopped-on-change mutant is detected at state and torque; original-case preservation passed with all318 identities. Those supplemental unit runs explicitly exclude the nonbaseline unsafe ring typed-read case CDX-008 as well as configured exclusions. They do not establish a standard integrated gate.

On 2026-11-06 the coordinator recreated the candidate tools from the exact reviewed P0 patch in an isolated matching layout with unchanged helper modules. Current Codex discovery:36 tests,13 ordinary passes+23 expected failures; candidate copy with only those decorators removed:36 ordinary passes. Durable logs: `../ffb-test-build/codex-p0-resume/{current,candidate}.log`; candidate tool/test copies retained beside them. The first reconstruction omitted an unchanged imported helper and failed at import; completing the matching layout fixed the harness, with no repository change. No shared patch was applied and no new C++ gate was run for this documentation/test-proposal check.

Phase-specific mutation and repair comparisons are in reviews/. Counts distinguish actual unexpected failures, expected-marker flips and controls. Historical temporary logs cleared between sessions are labelled unavailable; surviving sibling artifacts are the evidence available for inspection.

### Staged checker compatibility, 2026-11-06

Candidate checker against immutable existing binaries, retaining configured exclusions plus explicit nonbaseline CDX-008 ring-case exclusion:

| Variant | Doctest passed / failed / skipped | Original identities / checked / excluded | Result |
|---|---|---|---|
| Default | 645 / 0 / 5 | 318 / 317 / 1 | Pass |
| No DSP | 645 / 0 / 5 | 318 / 317 / 1 | Pass |
| DSP sanitizer | 642 / 0 / 8 | 318 / 315 / 3 | Pass |
| No-DSP sanitizer | 644 / 0 / 6 | 318 / 317 / 1 | Pass |

Each reports332 new identities, none failed. These counts omit the unsafe ordinary ring case and do not include the two unregistered Codex lifecycle cases. Sanitizer runs were outside the ptrace sandbox for LSan, with no suppressions. No rebuild was needed for this isolated Python-tool compatibility check. Durable exact commands/results: `../ffb-test-build/codex-p0-resume/baseline-results.json` and per-variant `*-baseline.log`. This is candidate evidence, not an applied standard CTest gate.

## Combined new target tests

On 2026-11-06 the coordinator scratch-linked both new target translation units (motor inhibition and finite DSP outputs) into one executable per existing default, no-DSP and GCC sanitizer configuration. Each ran only the two Codex suites in random orders 211 and 797, with zero unexpected failure:

- Default:4 executed each, three ordinary and the critical-error expected failure.
- No DSP:2 executed each, ordinary motor control and critical-error expected failure; DSP cases unregistered.
- GCC sanitizer:2 ordinary DSP cases executed each; both motor cases retain destructor skips.

Large XML skipped totals include all filtered-out original target cases. No forced skip or new suppression. Link/run argv, results, XML and stderr are durable under `../ffb-test-build/codex-combined-review/`. These are supplemental combined checks, not standard CMake integration.

A fifth safe candidate-checker configuration, Clang23ASan/UBSan, also preserves318 identities (315 checked/3 configured excluded),642 passed / 0 failed / 8 skipped with the additional nonbaseline ring exclusion. Exact command/log: `../ffb-test-build/codex-p0-resume/clang23-san-baseline-{command.json,log}` (actual log name `clang23-san-baseline.log`).

## Approved standard target integration — 2026-11-06

Owner reply: “Register the four target cases.” Applied only P2_TEST_INTEGRATION.patch, adding `codex/cases_target/test_*.cpp` to the shared target glob. P0 tooling and P1 unit registration remain unapplied. Fresh Debug builds configured from the actual repository and built the normal `ffb_target_tests` target:

| Fresh variant | Historical target baseline passed / failed / skipped | Integrated target passed / failed / skipped | Standard target CTest |
|---|---|---|---|
| Default DSP | 1819 / 0 / 18 | 1823 / 0 / 18 | 1/1 pass |
| No DSP | 1766 / 0 / 17 | 1768 / 0 / 17 | 1/1 pass |
| GCC ASan/UBSan | 1705 / 0 / 132 | 1707 / 0 / 134 | 1/1 pass |

Doctest passes include existing expected failures and the new critical-error marker. DSP-off has no finite-DSP cases; sanitizer adds two ordinary DSP cases and skips both new motor cases under the established destructor-UB policy. No sanitizer suppression was added. Focused Codex suites also passed random seed 797 in each fresh build (4,2,2 executed respectively); filtered-out target cases contribute to XML's large skipped totals. All configured original target exclusions/skips remain unchanged.

No diagnostic names either new Codex source file in any of the three compiler warning streams; full builds retain 667/666/667 pre-existing-source warning lines respectively, not a warning-clean claim. Compilation databases confirm both new target translation units are registered. Builds are `../ffb-test-build/codex-integrated-target-{default,nodsp,san}/`; each retains `codex-integration-evidence/{reproduce.py,commands.json,configure.log,build.log,ctest.log,last-test.log,focused.log,focused-totals.json,source-hashes.json,summary.json}`. Combined summary: `../ffb-test-build/codex-combined-review/integrated-target-results.json`. Raw logs retain original bytes; summary decoding replaces invalid diagnostic bytes.

Only the affected target entry was built/run in these fresh directories; this is not a fresh full nine-entry CTest gate. Original unit sources/glob did not change; the separate five-variant safe original-318 checks above remain preservation evidence. Historical baseline trees were not reconfigured or rebuilt. No new line/branch coverage measurement, hardware validation, commit or push.

## P0/P1 approved application complete

Owner said “Apply both” on 2026-11-06. Both reviewed patches are applied and all 23 resolved owned Python markers removed. [P0_P1_INTEGRATION.md](P0_P1_INTEGRATION.md) records the changes and exact evidence.

- Full default CTest:9/9. Other three affected variants (no-DSP, DSP sanitizer, no-DSP sanitizer):unit/baseline/Python3/3 each. These builds use the explicitly documented extra local nonbaseline ring exclusion, with shared defaults unchanged.
- Integrated unit totals default/no-DSP 647/0/5, DSP sanitizer 644/0/8, no-DSP sanitizer 646/0/6. New lifecycle cases 2 /16 assertions pass in all four variants. Original 318 identities preserved,317 checked /1 excluded except DSP sanitizer 315 checked /3 excluded. Each reports 334 new cases, 0 failed.
- Applied tools: 36 ordinary Codex Python passes; existing Python 115 tests, 113 ordinary +2 expected. Real six-executable inventories and unmatched probes valid; Python reporter 115 records/no problems.
- Applied gcov report on unchanged baseline artifacts: 8007/8095 firmware-ladder lines, 98.9%, 87 compiled files. No new-test coverage percentage claimed.

Evidence is in each build's codex-p0-p1-integration-evidence directory, separate from prior target-only integration evidence. Sanitizers used no suppression; firmware/Claude/vendor/baseline sources and shared exclusion defaults remain untouched. No commit or push.
