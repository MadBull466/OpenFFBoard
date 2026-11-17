# Handoff: OpenFFBoard host test suite (for any AI agent taking over)

Written 2026-11-04 by Claude (Opus 5.5) for the project owner (GitHub `MadBull466`). You have no memory of this project: read this file first, then the files in "Read order". Where this file and `Tests/Code_Coverage/claude/plans/PROGRESS.md` disagree, **PROGRESS.md wins** (it is the source of truth and is updated after every step).

## What this is

A host-side (Linux/Windows, no hardware) unit-test suite for the OpenFFBoard firmware (STM32F407, C/C++), built with doctest 2.4.11 and CMake in `Firmware/Tests/`. It started from upstream's `feature/host-unit-tests` (commit `a5418ff5`, 318 baseline cases, kept unchanged) and was extended in phases P0-P11. The firmware itself was never changed. Everything is on branch `add-test-suite`.

## State

The state changes with every session and is kept in one place: `Tests/Code_Coverage/claude/plans/PROGRESS.md`, sections "Current state" (phases, test counts, gates, coverage, bug counters, open owner decisions) and "Next action" (open items). Coverage per tier: `COVERAGE_AND_SCOPE.md`; the phase-by-phase table with estimates and measured results: `TEST_PLAN.md`, "Coverage by phase (P0-P18)". Do not copy numbers into this file.

### Where to continue

The test plan P0-P18 is complete: every phase is approved, committed and pushed (last commit of 2026-11-17: `09b1d8e9`). Nothing is waiting for a decision of yours except the open questions (17, 18, 20-23, 32-52, 54-61 and 70 in `Tests/Code_Coverage/claude/reports/OPEN_QUESTIONS.md`) and the open items listed under "Next action" in `PROGRESS.md`. New work starts only on the owner's request; if an open question is answered, update the affected tests, `PROGRESS.md`, `OPEN_QUESTIONS.md` and `BUG_REPORT.md` (run `make_bug_report.py`). Reusable pieces for a new phase: the charter and work-order shape of `MIDI_REQUIREMENTS.md` / `P17_WORK_ORDERS.md` and `charters/hal/`, `support/RmdFixture.h` (threads with several `Delay` calls), `support/MidiFixture.h` and `platform/usb_midi/` (a recording double of a driver interface for a caller built into another executable), and `fakes/stm32_hal/` (register-level models for the real HAL).

## Rules you must follow

The full, current rules are in the owner's root `CLAUDE.md` (and `AGENTS.md` for Codex); they are not repeated here, because a copy drifts. The ones that matter most:

1. `Firmware/` is **read-only** except `Firmware/Tests/`; vendor code bugs are reported, never fixed; never edit firmware to make it testable.
2. Git: branch `add-test-suite`, never commit to main / master; commit only when the owner says so, with the one-off identity `git -c user.name="MadBull.Ben" -c user.email= commit ...`; push only to the owner's fork by URL and only when asked; never write the owner's email anywhere.
3. No hardware, no network in tests; ask before adding any dependency; install nothing; do not edit `.github/`.
4. Specification files in `Tests/Code_Coverage/shared/specs/` are supplied by the owner and never committed; never invent register maps, opcodes or CAN frames; if a spec is missing, stop and ask.
5. Expected values come from a spec or a hand derivation, never from the firmware's current output (otherwise `characterization:` plus a NEEDS REVIEW entry); one behaviour per test; never sleep; keep the 318 baseline cases (run `tools/check_baseline.py` after every CMake or test change).
6. Stop at the end of every phase or module group and wait for the owner's approval.

## Known issues you will meet

Three standing items (the `effectsForces` out-of-bounds read that aborts on GCC 16, the `arm_spline_f32` ASan finding, the skipped crash case `command with an unknown id is rejected`) plus the 32-bit build limits are described, with the exact case names and exclusions, in `CLAUDE.md` ("Known issues") and `Tests/Code_Coverage/claude/plans/TEST_PLAN.md` ("Scope and known issues"). Several VESC / TMC tests that corrupt memory are skipped by default (marked "KNOWN ISSUE (crash)" / "(memory corruption)").

## How to build and run

The exact, maintained commands are in `Tests/Code_Coverage/claude/plans/PROGRESS.md`, section "Environment". Short form from the repo root (Linux; the author used GCC 16, clang 23, CMake 4, Unix Makefiles):

```
cmake -S Firmware/Tests -B ../ffb-test-build/default
cmake --build ../ffb-test-build/default -j"$(nproc)"
ctest --test-dir ../ffb-test-build/default --output-on-failure
python3 -B Firmware/Tests/Code_Coverage/tools/check_baseline.py --build-dir ../ffb-test-build/default
```

Add `-DFFB_TESTS_SANITIZE=ON` for ASan+UBSan, `-DFFB_TESTS_USE_DSP=OFF` for no CMSIS-DSP, the clang compiler variables for clang. Always build **out of tree** in `../ffb-test-build/<variant>` (never inside the repo). Use `python3 -B`. `tools/summarise_results.py` condenses logs; `tools/run_all_tests.py` runs every case including skipped ones; `tools/coverage_report.py` needs gcov only; `tools/make_bug_report.py` regenerates `Tests/Code_Coverage/claude/reports/BUG_REPORT.md`.

## Method used for each phase (continue it)

Roles: one **lead** (a stronger model) decides scope, writes charters and does final quality control and writes no bulk tests; **workers** (Sonnet class) write tests from work orders, never decide scope or disputes. Agent definitions are in `.claude/agents/` (`driver-requirements-mapper`, `driver-test-writer`, `driver-test-reviewer`, `test-output-summariser`).

1. Inventory of the code or library (a worker). 2. Requirements charter from the specification (the lead; ambiguous points are decided by the lead or asked of the owner). 3. Fakes or infrastructure (a worker, reviewed by the lead). 4. Several writers in parallel, each with one file and its own absolute build directories (`FFB_*_TEST_FILTER` options restrict what a writer builds). 5. Full gate: build and run in every variant (default, san, clang23, nodsp, nodsp-san, clang23-san, cov), `check_baseline`, no warnings in test files, no sanitizer findings. 6. A **fresh reviewer** checks for gaps and false positives, runs tests in random order and isolated, and mutates the code in a scratch copy (a mutation that no test notices is a gap). 7. Lead re-runs the survivors. 8. Update PROGRESS, regenerate the bug report, write the report file, commit, stop.

Pitfalls seen: writers sometimes locked in a buggy value without a marker (always check), claimed a bug that was a misreading (verify in the source before recording), used relative build paths inside the repo (forbid), and hit usage limits (resume with `SendMessage`).

## Read order for a new agent

1. This file. 2. `CLAUDE.md` (rules). 3. `Tests/Code_Coverage/claude/plans/PROGRESS.md`: "Current state", "Next action", "Environment", decisions log. 4. `Firmware/Tests/README.md` (architecture, fakes, conventions). 5. `Tests/Code_Coverage/claude/plans/TEST_PLAN.md` (plan v2, section P11), `Tests/Code_Coverage/claude/plans/DRIVER_TEST_PLAN.md`. 6. `Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md` and the G1 work orders (P11). 7. `Tests/Code_Coverage/claude/reports/OPEN_QUESTIONS.md` and `Tests/Code_Coverage/claude/reports/BUG_REPORT.md` before reviewing anything. 8. Per-driver charters `Tests/Code_Coverage/claude/charters/driver/*_REQUIREMENTS.md` and reports `Tests/Code_Coverage/claude/reports/D1_TMC4671_REPORT.md`, `D2_ODRIVE_REPORT.md`, `DRIVER_TRACK_REPORT.md`, `PROJECT_SUMMARY.md`.

## Suggested ways to help

* **Work through the open items** listed under "Next action" in `PROGRESS.md`, only with the owner's go-ahead. Stop at each approval gate.
* **Review the owner's work:** pick a phase, run the gate, mutate the firmware in a scratch copy, read the tests for weak oracles (tests that would pass for a wrong implementation), check every `characterization:` test and its NEEDS REVIEW entry, and check the claims in the bug report against the source. Report findings; do not fix firmware.
* **Answer open questions** only if the owner gives you the answer; otherwise do not guess.

## What this repository cannot prove

The fakes model peripherals, not hardware: control loops, electrical timing, real CAN/SPI/USB bus behaviour, interrupt vs thread concurrency on target and MinGW/Windows runs were not verified. See "What the fakes cannot prove" in `Tests/Code_Coverage/claude/reports/DRIVER_TRACK_REPORT.md` and the fake fidelity ratings in each fake's header.
