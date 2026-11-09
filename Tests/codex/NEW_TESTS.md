# New Codex tests

P12 addition: the standalone owned `Firmware/Tests/codex/hal/` project compiles the real GPIO source and executes eight ordinary cases / 231 assertions. See [P12 results](p12/GPIO_RESULTS.md) for six configurations, mutations and limits. These are separate from the existing shared gate.

Current final inventory is below; earlier integration paragraphs following it are historical. All groups are registered. Final gate counts/evidence: [CLOSEOUT_RESULTS](CLOSEOUT_RESULTS.md).

| Closeout file under `Firmware/Tests/codex/` | Cases / role | Contract |
|---|---|---|
| `cases/test_ringbuffer_defined_access.cpp` | 2 ordinary | Defined populated typed-byte and empty untyped access; avoids CDX-008 UB |
| `cases/test_numeric_assertions.cpp`, `support/NumericAssertions.h` | 3 ordinary plus reusable predicate | Finite equality/tolerance, NaN/Inf/invalid tolerance and subtraction-overflow rejection |
| `cases_target/test_closeout_inputs_buses.cpp` | 8: 4 ordinary, 4 expected failures | ADS threshold/control/caller buffer, exact shifter CS, SPI retry payload/completion/release |
| `cases_target/test_closeout_dsp_pid.cpp` | 1 ordinary when DSP enabled | Public PID versus independently derived positional equation |
| `cases_rtos/test_closeout_rtos.cpp` | 7: 6 ordinary, 1 expected failure | Sync masks/state, Tasklet payload/rejection, task-context drained-timer destruction; relevant sanitizer skip retained |
| `cases_usb/test_closeout_usb_guard.cpp` | 1 ordinary; separate injected-fault gate | Successful setup fault boundary before derived fault discard |
| `cases/driver/test_closeout_hall.cpp` | 1 ordinary | Six signed distinct nondefault Hall angles, independent packed words |
| `python/test_suite_discovery.py`, `python/run_suite.py` | 3 ordinary discovery controls and runner | Mandatory nonempty routine discovery |
| `python/test_usb_guard_runner.py`, `python/check_usb_guard.py` | 5 ordinary protocol controls and separate 1-unittest live gate | Precise USB first-failure validation; rejects stderr/crash/wrong failure |

Together with the 36 P0 tests, Python discovery runs 44 ordinary cases. The accepted six earlier C++ regressions remain registered. `cases_rtos/verify_closeout.py` retains bounded scratch discrimination support; detailed prerequisites and evidence are in [CLOSEOUT_RTOS](reviews/CLOSEOUT_RTOS.md). Independent acceptance and mutation/repair discrimination are in [final QC](reviews/CLOSEOUT_FINAL_QC.md) and [independent review](reviews/CLOSEOUT_INDEPENDENT_REVIEW.md). Expected failures preserve open production/vendor defects; they are not ordinary passes.

## Earlier integration record

P0-P10 review is now authorised and active. New cases remain in Codex-owned folders. The owner approved target-case CMake registration on 2026-11-06; that patch is applied and fresh standard target builds pass in default/no-DSP/GCC sanitizer. The owner subsequently said “Apply both”; P0 tooling and P1 unit registration are applied and normal build verification is complete.

| File | Cases | Requirement / finding | Verification |
|---|---:|---|---|
| `Firmware/Tests/codex/python/test_p0_tooling.py` | 36 | CDX-002–007: retain missing workload denominators, reject collector/status/empty-discovery failures, preserve original metadata and unique identities; positive controls preserve normal behavior | Applied tools: 36 ordinary passes; all 23 resolved expected-failure decorators removed. Fresh Astra QC had accepted the candidate; owner now approved/applied it. See the creation report for schema and real-gcov controls. |
| `Firmware/Tests/codex/cases/test_hid_effect_transition.cpp` | 2 | CDX-009: changing running/stopped periodic parameters must preserve explicit lifecycle state and distinguish force from silence | Writer ran default/no-DSP and both sanitizer variants, 2 focused cases/16 assertions pass; M7 stopped-on-change mutation killed at running-state and torque checks. Supplemented regression/checker runs pass with the explicitly documented additional nonbaseline ring-test UB exclusion. Fresh Astra QC accepted the cases; owner now approved unit-glob registration, applied and verified in normal CMake builds. |

Tests, independent oracles, dependencies, exact names/commands and evidence: [P0_TEST_CREATION.md](reviews/P0_TEST_CREATION.md) and [P1_TEST_CREATION.md](reviews/P1_TEST_CREATION.md). Proposals are stored under `Tests/codex/proposals/`; they do not modify protected code by themselves. Counts are a snapshot; update after QC refinements rather than treating historical candidate passes as final acceptance.

## Additional motor-side regressions (2026-11-06)

`Firmware/Tests/codex/cases_target/test_critical_error_motor.cpp` adds two cases in `CodexCriticalErrorMotor`: ordinary explicit-command stop and known-issue new-critical-error stop, both after observed nonzero force and including three later timer/main-task updates. CDX-010 requirement/dependencies/commands and exact case names are recorded in [P2_TEST_CREATION.md](reviews/P2_TEST_CREATION.md). Default/no-DSP controls pass; proper repair passes all intended motor assertions while flag-only repair fails them. Both sanitizer bodies retain required destructor skips. Fresh Astra QC accepted both cases. The owner approved P2_TEST_INTEGRATION.patch; normal CMake registration is applied and fresh default/no-DSP/GCC sanitizer target gates pass.

Two ordinary original-tier DSP cases in `Firmware/Tests/codex/cases_target/test_dsp_finite_outputs.cpp` address CDX-014. Exact names, independent references, dependencies and discrimination evidence are in [P9_TEST_CREATION.md](reviews/P9_TEST_CREATION.md). Default/GCC/Clang sanitizer author runs pass; both NaN wrappers are detected at per-sample finite predicates. Fresh Astra QC accepted them in FINAL_REVIEW_QC.md. They share the approved target-glob registration; fresh standard target gates pass. No-DSP builds register zero DSP cases.

Current state after “Apply both”: all six C++ cases are registered through the approved unit/target globs. P0's 36 unittest cases run directly and all pass as ordinary tests; the reviewed patches do not add them to CTest's existing 115-case Python entry. Normal P1 integration verification passes in default/no-DSP and both sanitizer variants; see P0_P1_INTEGRATION.md and REGRESSION_RESULTS.

## P12 GPIO register adapter

`Firmware/Tests/codex/hal/cases/test_gpio_model.cpp` adds 36 ordinary cases (18 direct model controls, 18 public HAL cases). `model_host/` supplies the source-inclusion observer and register boundary; unchanged vendor source is compiled as C++ in that separate target. The real-C storage target remains. Four current configurations pass both targets, nine HAL/model mutants are detected, and original-case preservation passes. See [results](p12/REGISTER_MODEL_RESULTS.md) for exact commands, scope, evidence and remaining review/integration limits.

## P12 real-C time and DMA-state group

`hal/cases/test_tick.cpp` adds 11 ordinary cases / 34 assertions; `test_delay.cpp` adds seven ordinary cases plus P12-TICK-001 (23 passing assertions, one expected failed assertion); `test_dma_state.cpp` adds 10 ordinary cases / 164 assertions. All paths under `Firmware/Tests/codex/`. [Time/state results](p12/TIME_STATE_RESULTS.md) records current five/four-entry standalone gates, seven fault controls, bounded repair discrimination and limits. Shared test globs are unchanged.
