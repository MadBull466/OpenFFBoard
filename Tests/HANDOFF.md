# Handoff: OpenFFBoard host test suite (for any AI agent taking over)

Written 2026-11-04 by Claude (Opus 5.5) for the project owner (GitHub `MadBull466`). You have no memory of this project: read this file first, then the files in "Read order". Where this file and `Tests/PROGRESS.md` disagree, **PROGRESS.md wins** (it is the source of truth and is updated after every step).

## What this is

A host-side (Linux/Windows, no hardware) unit-test suite for the OpenFFBoard firmware (STM32F407, C/C++), built with doctest 2.4.11 and CMake in `Firmware/Tests/`. It started from upstream's `feature/host-unit-tests` (commit `a5418ff5`, 318 baseline cases, kept unchanged) and was extended in phases P0-P11. The firmware itself was never changed. Everything is on branch `add-test-suite`.

## State at handoff

| Item | State |
|---|---|
| P0-P9 (in-scope firmware, FreeRTOS wrappers, TinyUSB, CMSIS-DSP functions the firmware calls, Python helpers) | done, approved by the owner |
| P10 driver track D0-D3 (TMC4671, ODrive, VESC) | done, approved 2026-11-04 |
| P11 CMSIS-DSP in scope (all functions and types of the modules the firmware uses) | **in progress: G1 tests written (2026-11-05), review pending** |
| Open questions for the owner | `Tests/OPEN_QUESTIONS.md` (project 3, 5-8; TMC4671 9-12; ODrive 13-14; VESC 16; P11 17) |
| Suspected bugs | `Tests/BUG_REPORT.md` (generated, do not hand-edit): firmware bugs numbered up to 193, vendor bugs up to V41, NEEDS REVIEW up to 199 |

Whole-project numbers are in `Tests/PROJECT_SUMMARY.md` and `Tests/DRIVER_TRACK_REPORT.md` (driver executable: 938 cases, 0 failed).

### P11 in detail (the unfinished work)

* Decisions (the owner's): scope = all functions and data types of the CMSIS-DSP modules the firmware uses (FastMath, Controller, Interpolation, Statistics, Filtering, plus helpers `arm_abs_q15/q31`, `arm_add_f32/f16`); oracle = ARM's reference patterns in `Firmware/Libraries/CMSIS-DSP/Testing/Patterns` plus mathematical definitions where ARM has none; new phase with a **stop for approval after each module group G1-G5**.
* Charter: `Firmware/Tests/dsp/CMSIS_DSP_REQUIREMENTS.md` (requirements C1-C6, lead decisions L1-L6, groups). Survey of the library and ARM tolerances: `Firmware/Tests/dsp/CMSIS_DSP_INVENTORY.md`. Work orders for G1: `Firmware/Tests/dsp/G1_WORK_ORDERS.md`.
* **Done (G1 infrastructure):** executable `ffb_dsp_tests` (CMake block, own ctest entry, f16 feature check), pattern loader `support/DspPatterns.h/.cpp`, 27 cases (loader self-tests and smoke tests), README section, run_all_tests and coverage integration. Commit `c1348426`.
* **Done (G1 tests, 2026-11-05, uncommitted until the docs commit):** `cases_dsp/test_dsp_fastmath_{f32,f64,f16,q31,q15}.cpp`, `test_dsp_basicmath_helpers.cpp`. `ffb_dsp_tests` 218 cases, 0 failed (default, clang 23); sanitizer 191 / 0 failed / 27 skipped; ctest 9/9. Suspected vendor bugs V37-V41, NEEDS REVIEW 192-199, open question 17.
* **Not done:** the fresh-reviewer review and mutation check of the G1 files (scratch copies of the CMSIS-DSP sources, never the repo), the no-DSP/coverage variants, G1 approval; then G2-G5.
* After G1 the lead must: run the full gate, get a fresh reviewer with mutation checks, update PROGRESS (vendor bug numbers start at V37, NEEDS REVIEW at 192), run `tools/make_bug_report.py`, write a short group report, commit, and **stop for the owner's approval**.

## Rules you must follow (from the owner's `CLAUDE.md`, repeated here because they are strict)

1. `Firmware/` is **read-only**, except `Firmware/Tests/` (never edit `Firmware/Tests/third_party/doctest.h`). Vendor code (CMSIS-DSP, TinyUSB, ST HAL, FreeRTOS, `eeprom.c`): bugs are **reported, never fixed**. Never edit firmware to make it testable; never add `#ifdef` for tests to it; use fakes in `fakes/` or stubs in `platform/src/host_stubs.cpp`.
2. Git: branch `add-test-suite`, never commit to main/master. Local commits (push only to the user's fork `MadBull466/OpenFFBoard`, only when asked; see `CLAUDE.md`) with a one-off identity: `git -c user.name="MadBull.Ben" -c user.email= commit ...` (name `MadBull.Ben`, email deliberately empty). **Push only to the fork, only when asked; never to `origin`; never add or change a remote; never change git config.** Never write the owner's email address into any file, commit message or report; grep every staged diff for email addresses before committing.
3. No hardware, flashing, serial/USB/CAN or network access in tests. Ask before adding any dependency that has to be downloaded. Install nothing globally. Do not edit anything under `.github/` (describe needed CI changes in PROGRESS and ask).
4. Specification files (ODrive/VESC/TMC4671 documents, `bldc-master`, `can-protocol.csv`) are supplied by the owner and **never committed or uploaded** (`Tests/.gitignore`). Never invent register maps, opcodes or CAN frames; if a spec is missing, stop and ask.
5. Expected values come from a spec, vendor documentation or a hand derivation written in a comment. Never copy the value the firmware currently produces into an assertion unless the test is named `characterization: ...` and has a NEEDS REVIEW entry.
6. Test conventions (full text in `Firmware/Tests/README.md`): use a fixture; test through public interfaces; one behaviour per test; the name is a statement that is true if the test passes; never sleep (advance simulated time); `KNOWN_ISSUE("...")` describes the INTENDED behaviour and fails today because of a suspected defect (the first failing assertion must be the defect), `KNOWN_ISSUE_UB` is the same for undefined behaviour and is skipped in sanitizer builds; never write a test that expects a suspected buggy result without a marker.
7. Keep the 318 baseline case names (`Firmware/Tests/baseline/test_cases.txt`); never edit existing test files, add new ones; after every CMake or test change rebuild and run `tools/check_baseline.py`.
8. Host is 64-bit, target is 32-bit ARM with a single-precision FPU: flag any test whose result could differ on target (type sizes, packing, endianness, float/double promotion, integer overflow, signed shifts, out-of-range float-to-int).
9. Out of scope (confirmed): PWM, MyActuator and SimpleMotion drivers, `TMCDebugBridge`, `MidiMain`/`MidiHandler` (feature macros stay off); ST HAL, CMSIS core, FreeRTOS kernel, `Firmware/Targets/**`, CubeMX code. Fake out-of-scope drivers at their interface.
10. Stop at the end of every phase or module group and wait for the owner's approval. Do not start the next one without it.

Note: `CLAUDE.md` still describes tier C as "only the CMSIS-DSP functions the firmware calls". Since 2026-11-04 tier C covers every function and type of the used modules (P11). The owner has not yet said whether to update that line.

## Known issues you will meet

* `effectsForces` out-of-bounds read: the case `requesting the forces of a not existing axis is rejected` (`cases/test_effects_calculator.cpp:1062`) aborts the run on GCC 16 (`_GLIBCXX_ASSERTIONS`). Exclude it with `--test-case-exclude=`; never disable assertions globally; report only.
* `arm_spline_f32` ASan finding in vendor code: in sanitizer runs skip only the two spline cases named in `Tests/PROGRESS.md`.
* Skipped crash: `command with an unknown id is rejected` stays skipped. Several VESC/TMC tests that corrupt memory are skipped by default (marked "KNOWN ISSUE (crash)"/"(memory corruption)").

## How to build and run

The exact, maintained commands are in `Tests/PROGRESS.md`, section "Environment". Short form from the repo root (Linux; the author used GCC 16, clang 23, CMake 4, Unix Makefiles):

```
cmake -S Firmware/Tests -B ../ffb-test-build/default
cmake --build ../ffb-test-build/default -j"$(nproc)"
ctest --test-dir ../ffb-test-build/default --output-on-failure
python3 -B Firmware/Tests/tools/check_baseline.py --build-dir ../ffb-test-build/default
```

Add `-DFFB_TESTS_SANITIZE=ON` for ASan+UBSan, `-DFFB_TESTS_USE_DSP=OFF` for no CMSIS-DSP, the clang compiler variables for clang. Always build **out of tree** in `../ffb-test-build/<variant>` (never inside the repo). Use `python3 -B`. `tools/summarise_results.py` condenses logs; `tools/run_all_tests.py` runs every case including skipped ones; `tools/coverage_report.py` needs gcov only; `tools/make_bug_report.py` regenerates `Tests/BUG_REPORT.md`.

## Method used for each phase (continue it)

Roles: one **lead** (a stronger model) decides scope, writes charters and does final quality control and writes no bulk tests; **workers** (Sonnet class) write tests from work orders, never decide scope or disputes. Agent definitions are in `.claude/agents/` (`driver-requirements-mapper`, `driver-test-writer`, `driver-test-reviewer`, `test-output-summariser`).

1. Inventory of the code or library (a worker). 2. Requirements charter from the specification (the lead; ambiguous points are decided by the lead or asked of the owner). 3. Fakes or infrastructure (a worker, reviewed by the lead). 4. Several writers in parallel, each with one file and its own absolute build directories (`FFB_*_TEST_FILTER` options restrict what a writer builds). 5. Full gate: build and run in every variant (default, san, clang23, nodsp, nodsp-san, clang23-san, cov), `check_baseline`, no warnings in test files, no sanitizer findings. 6. A **fresh reviewer** checks for gaps and false positives, runs tests in random order and isolated, and mutates the code in a scratch copy (a mutation that no test notices is a gap). 7. Lead re-runs the survivors. 8. Update PROGRESS, regenerate the bug report, write the report file, commit, stop.

Pitfalls seen: writers sometimes locked in a buggy value without a marker (always check), claimed a bug that was a misreading (verify in the source before recording), used relative build paths inside the repo (forbid), and hit usage limits (resume with `SendMessage`).

## Read order for a new agent

1. This file. 2. `CLAUDE.md` (rules). 3. `Tests/PROGRESS.md`: "Current state", "Next action", "Environment", decisions log. 4. `Firmware/Tests/README.md` (architecture, fakes, conventions). 5. `Tests/TEST_PLAN.md` (plan v2, section P11), `Tests/DRIVER_TEST_PLAN.md`. 6. `Firmware/Tests/dsp/CMSIS_DSP_REQUIREMENTS.md` and the G1 work orders (P11). 7. `Tests/OPEN_QUESTIONS.md` and `Tests/BUG_REPORT.md` before reviewing anything. 8. Per-driver charters `Firmware/Tests/driver/*_REQUIREMENTS.md` and reports `Tests/D1_TMC4671_REPORT.md`, `D2_ODRIVE_REPORT.md`, `DRIVER_TRACK_REPORT.md`, `PROJECT_SUMMARY.md`.

## Suggested ways to help

* **Continue P11:** review G1 (fresh reviewer, mutation checks), then stop for approval; then G2-G5. Stop after each group.
* **Review the owner's work:** pick a phase, run the gate, mutate the firmware in a scratch copy, read the tests for weak oracles (tests that would pass for a wrong implementation), check every `characterization:` test and its NEEDS REVIEW entry, and check the claims in the bug report against the source. Report findings; do not fix firmware.
* **Answer open questions** only if the owner gives you the answer; otherwise do not guess.

## What this repository cannot prove

The fakes model peripherals, not hardware: control loops, electrical timing, real CAN/SPI/USB bus behaviour, interrupt vs thread concurrency on target and MinGW/Windows runs were not verified. See "What the fakes cannot prove" in `Tests/DRIVER_TRACK_REPORT.md` and the fake fidelity ratings in each fake's header.
