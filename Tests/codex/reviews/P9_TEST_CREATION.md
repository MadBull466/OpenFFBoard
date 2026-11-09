# P9 finite-output regression guards

Writer: GPT-6.1 Sol, medium effort, assignment `/root/p9_finite_writer`, 2026-11-06. Scope: CDX-014 in [P9_REVIEW.md](P9_REVIEW.md), within the authorised P0–P10 audit. Fresh independent QC remains required. No P11 work, protected-source changes, dependency installs/downloads, shared build edits, commits or pushes by this assignment.

Added two ordinary cases in `Firmware/Tests/codex/cases_target/test_dsp_finite_outputs.cpp`, suite `CodexDspFiniteOutputs`. They call public CMSIS-DSP APIs and independently `REQUIRE(std::isfinite(actual))` for each observed sample before comparing numerical error. Both NaN and infinities fail this predicate; no maximum reduction can discard the invalid sample. These are additive guards; the protected original aggregators themselves remain unchanged and retain the reviewed defect.

| Requirement | Inputs and independent oracle | Coverage boundary |
|---|---|---|
| Finite small-angle sine/cosine outputs | 32 angles from 0.2 to 0.21 radians, including the sparse sine-wrapper interval. Double `std::sin/cos` of the same float input; absolute tolerance `2.1e-5`, justified by the 512-interval interpolation bound `h²/8 < 1.89e-5` plus float rounding margin. | Stateless public calls; no global fixture needed. Other angles/functions and target intrinsic paths are not newly covered. |
| Finite output from a stable stateful biquad | 26 nonzero asymmetric dyadic inputs in two successive blocks of 13. One stage with independently chosen coefficients `{.5, -.125, .0625, .25, .125}` in CMSIS order. Feedback poles `.5` and `-.25`; absolute feedback sum `.375`; with `abs(x)<=1`, output bound `.6875/(1-.375)=1.1`. Independent double DF1 equation, zero history, absolute tolerance `2e-6`. | Fixture owns fresh coefficients/state/instance per case. Second block preserves history. No production filter coefficients or observed outputs were copied into the oracle. No zero-stage, spline, PID or broader DSP expansion. |

The finite-input and stability reasoning establishes that non-finite outputs are invalid here. Per-sample accuracy checks also reject ordinary finite corruption, so these guards are not merely finite-only smoke tests. Host portable C and target FPU contraction may differ in last bits; tolerances allow float roundoff. Windows/macOS and actual Cortex-M execution were not tested. Existing accuracy tests remain useful and unchanged.

## Validation and discrimination

Durable scratch directory: `/home/ben/Documents/OpenFFBoard/testing/ffb-test-build/codex-p9-writer/`. Builds compile the owned source copied there, using flags from immutable baseline compilation databases, then link existing target objects read-only. No shared CMake edit was needed. **These tests are not claimed to be integrated into the standard build.** The separate pending target-glob proposal is outside this assignment.

| Run | Actual result |
|---|---|
| Default DSP, new suite, random seed 7719 | 2 ordinary cases passed; stderr empty |
| GCC ASan/UBSan, new suite, seed 7719 | 2 ordinary cases passed; stderr empty |
| Clang 23 ASan/UBSan, new suite, seed 7719 | 2 ordinary cases passed; stderr empty |
| Default DSP, original `CmsisDsp` + new suite, seed 9273 | 53 executed, zero unexpected failures; existing V32 expected failure and V33 skip preserved |
| GCC and Clang ASan/UBSan, combined selection, seed 9273 | 52 executed each, zero unexpected failures; V32/V33 skips preserved; stderr empty |
| DSP-off compile/link and new-suite inventory | Source compiled without `USE_DSP_FUNCTIONS`; zero matching cases registered |
| Historical writer original-case checker, default / DSP-off / GCC sanitizer / Clang sanitizer | All four exit 0, `RESULT: OK`; original 318 baseline names retained with configured exclusions. These runs omitted the confirmed unsafe nonbaseline CDX-008 ring-case exclusion and are not UB-clean evidence. |

Doctest XML lists filtered-out target cases as skipped too; those large skipped totals are not additional selected-suite skips. No `--no-skip`, crash forcing or suppression was used. The historical writer baseline check ran unchanged `ffb_unit_tests`; default reported 646 passed / 0 failed / 4 skipped, including execution of `typed access reads single bytes and counts elements` (CDX-008). That unsafe ordinary case must be excluded; a green historical checker result does not establish absence of UB. These focused target builds do not add unit cases or change the baseline executable.

QC correction: the coordinator's separate safe checker evidence in `../ffb-test-build/codex-p0-resume/baseline-results.json` and corresponding `*-baseline.log` files adds the explicit CDX-008 nonbaseline exclusion while retaining normal configured exclusions and all 318 baseline names. Those runs report default 645, DSP-off 645, GCC sanitizer 642, Clang sanitizer 642 and DSP-off sanitizer 644 passed, all zero failed. This is coordinator evidence read by the writer, not a new writer execution. The owned durable `run.py` now preserves configured exclusions and adds the exact ring-case exclusion to every future baseline command, using `*-safe-baseline` log labels. No unsafe historical command was rerun for this correction, and focused DSP results are unchanged.

Reused a copied version of the review's `dsp_output_probe.cpp`: separate linker wrappers return NaN or finite `100` only for `0.2 < x < 0.21` sine calls, or overwrite output element zero after the real biquad executes when block size is 13. All probes select the entire original `CmsisDsp` plus the new suite.

| Wrapper | Injections | Exit / unexpected failures | First failure in new ordinary case |
|---|---:|---|---|
| Sine NaN | 17 | 1 / new case only | `std::isfinite(sine)` |
| Sine finite 100 | 46 | 1 / original sine accuracy + new case | Per-sample sine accuracy; original still fails `worst <= 1.0` |
| Biquad NaN | 6 | 1 / new case only | `std::isfinite(output[i])` |
| Biquad finite 100 | 6 | 1 / original block accuracy + new case | Per-sample DF1 accuracy; original still fails `maxAbsDiff(out, expected) <= TOL` |

Sine NaN stops the new case at its first invalid sample (`REQUIRE`), hence fewer wrapper hits than its finite control. Biquad processes both blocks before checking outputs. The original suite still accepts the NaN samples while each new ordinary case rejects its corresponding wrapper. No firmware/vendor defect is inferred from injected corruptions. Infinity rejection is established by the predicate, not a separately executed infinity wrapper.

## Reproduction and artifacts

From the repository root, exact build/run entry points are:

```sh
python3 -B ../ffb-test-build/codex-p9-writer/build.py
python3 -B ../ffb-test-build/codex-p9-writer/run.py
```

Sibling writes and sanitizer execution require the environment's escalation permission. `build.py` saves the exact compiler/linker argument arrays and working directories in `build_commands.json`; `build.log` retains commands/output, including an initial Clang link failure caused by its dependency-file option. The final script removes both GCC and Clang dependency-file options before linking, and all four final builds succeed. It does not rebuild or modify baseline objects.

`run.py` preserves the exact DSP selections, seeds and wrapper environment triggers, with the future safe-baseline correction described above. Historical commands and results remain in `run-historical-before-cdx008-exclusion.py` and `results-historical-before-cdx008-exclusion.json`; the historical script is evidence only and must not be executed. `results.json` records commands, exit codes, selected totals, failing expressions and stderr from the completed historical run. Each run has `<label>.stdout` and `<label>.stderr`; XML runs use XML content in `.stdout`. Copied source `test_dsp_finite_outputs.cpp` was byte-compared with the repository source after validation; they match. `dsp_output_probe.cpp` is the copied wrapper source. `default/`, `nodsp/`, `san/` and `clang23-san/` contain the separately linked binaries/owned objects.

One initial execution request was not run because automatic approval review hit its usage limit, explicitly a review failure rather than an unsafe-action determination. After the owner's Continue instruction, the same escalation path succeeded; all results above come from that completed run. No approval check was bypassed.

Bookkeeping: +2 ordinary target cases when DSP is enabled, +0 when disabled; no new known-issue or characterization marker. No line coverage was recomputed, no phase-wide gate or standard integration is claimed, and no owner/Claude/shared files were edited by this writer. Fresh QC should review the permanent source, mathematical bounds, fixture reset, wrapper first failures and durable evidence before integration.

## Coordinator integration update

2026-11-06: fresh Astra QC accepted both finite-output guards. The owner approved target-case registration and P2_TEST_INTEGRATION.patch is applied, registering this file alongside the new motor cases. Fresh normal target CTest passes in default/no-DSP/GCC sanitizer with exact counts in REGRESSION_RESULTS.md. Prior scratch-only/integration-pending statements describe submission-time state. Protected original comparison helpers remain unchanged.
