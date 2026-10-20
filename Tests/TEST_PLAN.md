# Firmware Test Suite Plan

Extend the existing host-side doctest suite in `Firmware/Tests/` to cover all in-scope firmware logic in `Firmware/`. Permanent rules live in the root `CLAUDE.md` and apply throughout. The most important one is repeated here: **`Firmware/` is read-only except `Firmware/Tests/`.**

## How to start or resume

1. Read `CLAUDE.md`, this file, and `Tests/PROGRESS.md`.
2. If `PROGRESS.md` shows a phase in progress, resume from its "Next action" line. Do not redo completed work.
3. If `PROGRESS.md` shows Phase 1 not started, begin Phase 1.
4. Before stopping for any reason (phase end, context getting full, waiting for my approval), update `PROGRESS.md` so another session can pick up cleanly.

## Progress-file protocol (applies to every phase)

- Update `Tests/PROGRESS.md` after every completed module, every phase gate, every decision, and every `NEEDS REVIEW` or suspected-bug finding.
- Keep the "Current state", "Next action" and "Environment" sections accurate at all times. A future session reads these first and uses the Environment commands to rebuild.
- Record the exact commit hash on branch `add-test-suite` when a module is done.
- Commit `PROGRESS.md` together with the work it describes. Small, frequent local commits on `add-test-suite`, with a one-off `git -c user.name=... -c user.email= commit` (never change git config).
- NEVER push, and never add or change a git remote.
- Never mark a module "done" until its tests build, pass, and have been reviewed (Phase 3).

## Scope

IN scope: all firmware logic in `Firmware/` outside `Libraries/`: parsing, command handling, state machines, math/scaling/filters, effect and force-feedback calculations, config and persistent-storage handling, protocol/report encoding and decoding, error handling, boundary values.

Vendor code IN scope, in tiers. Bugs found in vendor code are reported in `PROGRESS.md` and never fixed:
- Tier A: `FFBoard/Src/eeprom.c` (ST EEPROM emulation, tested directly via the flash tests on the NOR-semantics fake flash), the FreeRTOS C++ wrappers (`FFBoard/Src/c*.cpp`), and both Python helpers (`Firmware/scripts/generate_memory.py`, `Firmware/FFBoard/USB/portable/synopsys/dwc2/dwc2_info.py`) in Phase 5.
- Tier B: TinyUSB device code in `Firmware/FFBoard/USB`, tested against a fake USB controller, as its own batch near the end of Phase 2.
- Tier C: only the CMSIS-DSP functions the firmware calls (biquad, spline, and whatever else `EffectsCalculator` and `Filters` use).

OUT of scope (do not write tests for these):
- The ST HAL, the FreeRTOS kernel, `Targets/**`, and the rest of `Firmware/Libraries/`, unless I say otherwise.
- Driver and external-board implementations for TMC4671, ODrive, VESC, PWM, MyActuator, SimpleMotion.
- `TMCDebugBridge`, `MidiMain` and `MidiHandler`. The `MIDI` feature macro stays off.
- Anything that only works by talking to real hardware.

## Known issue: `effectsForces` out-of-bounds read

GCC 16's libstdc++ turns on `_GLIBCXX_ASSERTIONS` at -O0, so a pre-existing out-of-bounds read in the `effectsForces` test case now aborts the doctest run. Before any new work, 192 of 317 test cases ran.

- Record it as a suspected firmware bug in `PROGRESS.md` with the evidence. Do not fix the firmware.
- Do not write a test that expects the buggy result.
- Exclude that one case using doctest's test-case-exclude option (`-tce`) in baseline and new runs. Confirm the exact test-case name from the source.
- List the exclusion explicitly in every report and in the final report.
- Do not disable `_GLIBCXX_ASSERTIONS` globally.

## Known issue: CMSIS-DSP `arm_spline_f32` over-read (sanitizer runs only)

ASan reports a stack-buffer-overflow in CMSIS-DSP `arm_spline_f32` (vendor code), reached from `EffectsCalculator::evaluateReconstructionFilter`. Recorded as bug #2 in `PROGRESS.md`.

- Keep ASan on the whole build, including CMSIS-DSP.
- In sanitizer runs only, also exclude `reconstruction of a constant signal is constant` and `reconstruction of a step never leaves the range of the input values and reaches the target` with `-tce`. List this exclusion in every sanitizer report.

## Phase 1: Explore and plan (read-only)

Goal: understand the code and the existing suite, then produce a plan I can approve. Do not write or change any test or source file in this phase except `Tests/PROGRESS.md` (and `Tests/TEST_PLAN.md` / `CLAUDE.md` if I ask).

1. Create branch `add-test-suite` if it does not exist.
2. Build the existing `Firmware/Tests/` suite out-of-tree and run it. Record the baseline in `PROGRESS.md`: exact configure, build and run commands, framework, pass/fail counts, runtime, and the effect of the `effectsForces` exclusion (the run should complete with that case excluded). Report any build problems before going further.
3. Check the sanitizer variants (ASan, UBSan) and clang if available. Record which variants build and which findings they produce.
4. Map `Firmware/` (excluding `Libraries/`): modules, classes, public functions, entry points, and which hardware/HAL/RTOS APIs each touches. Record the module list in the `PROGRESS.md` module table.
5. Identify every source file and class that implements the out-of-scope drivers (TMC4671, ODrive, VESC, PWM, MyActuator, SimpleMotion). List them for my confirmation, including any shared base classes or interfaces and whether each is driver code or caller code.
6. Inventory what `Firmware/Tests/` already covers and how it handles hardware boundaries (fakes, stubs, link substitution, shadowed headers). Reuse these techniques. Record coverage gaps per module.
7. Propose any additional fakes needed (HAL peripherals, RTOS primitives, flash/EEPROM emulation, USB/CAN/UART/SPI abstractions, timers/clock).
8. Write the test plan into `PROGRESS.md`: per module, what will be tested, what will be faked, what is excluded and why, with a priority order (highest-risk logic first).
9. Flag host-vs-target risks found in the code (type sizes, packing, float/double, overflow).
10. **STOP.** Present the plan and wait for my explicit approval. Set Phase 1 status to "awaiting approval".

## Phase 2: Build (use subagents)

Starts only after I approve the Phase 1 plan.

Test executables:
- `ffb_unit_tests` stays exactly as it is: same sources, same feature set, same fake `Axis.h`.
- New executables (e.g. for the real `Axis`, the flash/EEPROM code, TinyUSB) get their own feature header. Build it from the real F407 `Targets/F407VG/Core/Inc/target_constants.h`: copy its feature macros, remove `TMC4671DRIVER`, `PWMDRIVER`, `ODRIVE`, `VESC`, `SIMPLEMOTION`, `RMDCAN`, `TMCDEBUG`, leave `MIDI` off, and show me the resulting diff before using it.
- Firmware sources are compiled per executable. Only the platform fakes are shared.
- `ctest` runs every executable with the documented exclusions.
- After every CMake change, rebuild and confirm the existing suite still gives 316 passed, 0 failed, 2 skipped with the exclusion, and record it in `PROGRESS.md`.

1. One agent first creates the shared fakes and any CMake changes in `Firmware/Tests/`, then commits them. Later agents build on these. Record them in `PROGRESS.md`.
2. Then use subagents in parallel, each owning one module or group of modules and its own test files. Give each subagent:
   - the module(s) it owns,
   - the conventions and fakes from Phase 1,
   - the rules in `CLAUDE.md`, including: `Firmware/` is read-only except `Firmware/Tests/`.
3. Subagents must not edit each other's files, and must not edit shared fakes or CMake without telling me which change is needed.
4. Each subagent reports back: tests added, behaviours covered, fakes used, `NEEDS REVIEW` items, suspected firmware bugs, untestable code. The main agent records these in `PROGRESS.md`.
5. After each batch, build and run the whole suite (with the documented `effectsForces` exclusion). Do not continue to the next batch while anything fails to build or regresses.
6. Prioritise by the approved order. If usage or context is running low, stop at a clean module boundary and update `PROGRESS.md`.

## Phase 3: Review (use fresh reviewer subagents)

Use reviewers that did not write the tests being reviewed. Each reviewer checks:
- Do tests assert meaningful behaviour (no empty or tautological assertions)?
- Are edge cases, boundaries, error paths and invalid inputs covered?
- Is any out-of-scope driver or library code being tested by mistake?
- Are the fakes realistic, or could they hide real bugs?
- Are tests independent and deterministic: no sleeps, no timing flakiness, no global-state leakage between tests?
- Any host-vs-target differences (type sizes, packing, endianness, float vs double, overflow, signed shifts) that make a test misleading?
- Do any tests lock in a suspected firmware bug without a `NEEDS REVIEW` flag?

Fix the findings, re-run the suite, and record review outcomes per module in `PROGRESS.md`. A module is "done" only after this phase passes.

## Phase 4: Verify

1. Run the full suite (existing + new) and record the results, listing the `effectsForces` exclusion.
2. Run again with AddressSanitizer and UndefinedBehaviorSanitizer (both available with GCC on Linux), on the whole build including CMSIS-DSP. List both exclusions (`effectsForces` and the two spline cases). Record findings.
3. Measure coverage with plain gcov plus the stdlib-only Python script in `Firmware/Tests/` (no gcovr/lcov, nothing installed). Report firmware code and the tested vendor tiers (A, B, C) separately. Exclude `Firmware/Tests/`, the out-of-scope drivers, and untested vendor code. Report per-module coverage and explain uncovered code as hardware-only, unreachable, or missed.
4. Test strength check: in a scratch copy outside the repo, mutate a few lines of firmware logic and confirm tests fail. Report which mutations were caught or missed.
5. Confirm `git diff` shows no changes under `Firmware/` other than `Firmware/Tests/`, so the ARM Makefile build is unaffected. Confirm no build artefacts were written under `Firmware/`.

## Phase 5: CPython helper scripts (vendor tier A)

Add small pytest tests for `Firmware/scripts/generate_memory.py` and `Firmware/FFBoard/USB/portable/synopsys/dwc2/dwc2_info.py`. Keep them separate from the C/C++ suite and out of the CMake build. Ask before installing pytest if it is not available (never globally). Bugs in `dwc2_info.py` (TinyUSB) are reported, never fixed.

## Deliverables

- New and updated tests, fakes and CMake changes in `Firmware/Tests/`.
- Updated README for `Firmware/Tests/` (or equivalent): how to build out-of-tree, run, and measure coverage, and how the fakes work.
- Final report (add it to `Tests/PROGRESS.md` or a new `Tests/REPORT.md`): what is covered, what is excluded and why (including the `effectsForces` exclusion and the sanitizer-only spline exclusion), vendor bugs found per tier, `NEEDS REVIEW` items, suspected firmware bugs, host-vs-target caveats, untestable code with refactor suggestions, and the final coverage numbers.
