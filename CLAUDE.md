# Project notes

## Layout
- `Firmware/` - C/C++ firmware for STM32F407 (Makefile, ARM build). ~850 .c/.cpp/.h files outside `Firmware/Libraries/`.
- `Firmware/Tests/` - the existing host-side unit tests (doctest 2.4.11, C/C++, CMake). This is the suite we extend. Read `Firmware/Tests/README.md` first: it describes the architecture (fake platform, simulated time, mocks, fixtures) and the conventions below.
  - Shared: `cases/test_*.cpp` (the original 318 baseline cases), `mocks/`, `fakes/`, `support/`, `platform/include`, `platform/src`, `tools/`, `third_party/`, `baseline/`, `CMakeLists.txt`, `CMakePresets.json`.
  - `Firmware/Tests/claude/` - Claude's test cases (`cases/`, `cases/driver/`, `cases_target/`, `cases_flash/`, `cases_rtos/`, `cases_usb/`, `cases_dsp/`, `python/`). `Firmware/Tests/codex/` - Codex's test cases (do not touch).
- `Tests/` (repo root) - project-management files only, no test code: `Tests/claude/{plans,reports,charters}` (`TEST_PLAN.md`, `PROGRESS.md`, ...), `Tests/codex/` (Codex's documents, do not touch), `Tests/shared/specs/` (git-ignored specification files).
- `AGENTS.md` (repo root) - the same rules for Codex.
- Python helper scripts outside `Libraries/`: `Firmware/scripts/generate_memory.py` and `Firmware/FFBoard/USB/portable/synopsys/dwc2/dwc2_info.py`. Covered in Phase 5.

## Environment and commands
- Host: Linux, GCC 16, clang 23, CMake 4, default Makefiles generator (Ninja is only a shell alias, not on PATH). Python is available. Do not install anything.
- The suite must keep working on Windows (MinGW, `CMakePresets.json` presets `host` / `host-mingw`) and in GitHub Actions. Keep new code and CMake portable. Sanitizers are Linux/macOS only. Do not edit anything under `.github/`; if CI needs a change, describe it in `PROGRESS.md` and ask me.
- Build out-of-tree in `../ffb-test-build/<variant>` so nothing is written under `Firmware/`, not even gitignored build files.
- Exact configure / build / run / coverage / sanitizer commands: see "Environment" in `Tests/claude/plans/PROGRESS.md`. Keep that section accurate. It is the source of truth.

## Hard rules
- `Firmware/` is READ-ONLY, with one exception: `Firmware/Tests/` (but never edit `Firmware/Tests/third_party/doctest.h`). Never modify any other file under `Firmware/` (including the Makefile, ARM build, `Firmware/Libraries/`, TinyUSB, `eeprom.c` and other vendor code) unless I explicitly ask. Bugs in vendor code are reported, never fixed.
- Never edit the firmware sources to make them testable and never add `#ifdef` for tests to them. Use a test double in `fakes/` or a stub in `platform/src/host_stubs.cpp`.
- New tests, fakes, mocks and CMake changes go in `Firmware/Tests/` only, matching the existing structure and naming.
- Use doctest. Do not introduce a new test framework. Python tests use stdlib `unittest` (do not install pytest).
- Git: work on branch `add-test-suite`. Local commits are allowed (small and frequent). Pushing is allowed ONLY to the user's fork `https://github.com/MadBull466/OpenFFBoard` (branch `add-test-suite`), and only when the user asks. Push by URL (`git push https://github.com/MadBull466/OpenFFBoard.git add-test-suite`) so no remote is added. Never push to `origin` (upstream Ultrawipf), never push other branches or force-push, and never add or change a git remote. Do not change git config: use a one-off `git -c user.name="MadBull.Ben" -c user.email= commit ...` (author name `MadBull.Ben`, email deliberately empty). Never write the user's email address into any file, commit message or report. Never commit to main.
- No hardware access, flashing, serial/USB/CAN, or network access during tests.
- Ask before adding any dependency that has to be downloaded (e.g. CMake FetchContent). Do not install anything globally.
- Keep the existing tests: all 318 baseline cases (names in `Firmware/Tests/baseline/test_cases.txt`) must stay present with unchanged results. Never edit existing test files; add new files. After every CMake or test change, rebuild and run `tools/check_baseline.py`; `ffb_unit_tests` must report 0 failed (on GCC 16 that is 316 passed, 2 skipped with the documented exclusion).

## Test scope
- In scope: firmware logic outside `Firmware/Libraries/`, plus vendor code in these tiers:
  - Tier A: `eeprom.c` (directly, via flash tests on a fake flash with real NOR semantics), the FreeRTOS C++ wrappers (`c*.cpp`), the two Python helpers.
  - Tier B: TinyUSB device code in `Firmware/FFBoard/USB/`, tested against a fake USB controller.
  - Tier C: only the CMSIS-DSP functions the firmware calls (biquad, spline, whatever `EffectsCalculator` and `Filters` use).
- Not tested: ST HAL, CMSIS core, the FreeRTOS kernel, `Firmware/Targets/**` and CubeMX-generated code.
- Driver track (added 2026-11-02): the driver code for **TMC4671, ODrive and VESC** is now IN scope, in that order, and is driven by `Tests/claude/plans/DRIVER_TEST_PLAN.md`. Only those three drivers (plus `TMCDebugBridge`, see P13 below).
- P13 (added 2026-11-10): `TMCDebugBridge` is now IN scope, driven by `Tests/claude/plans/P13_TMCDEBUGBRIDGE_PLAN.md`. Its feature macro `TMCDEBUG` is on in `ffb_driver_tests` only (off in every other executable). TMCL values specified in the supplied documents (`Tests/shared/specs/TMCL_reference.pdf` and the TMC4671-TOSV-REF / LEV-REF manuals: frame, checksum, reply layout, status codes, GGP / GIO / version command) are plain tests; TMCL values they do not specify (opcodes 143 / 146 / 148, parameter meanings, version content) are `characterization:` tests; the bridge's silent drops of invalid frames are `KNOWN_ISSUE` (owner's decisions 2026-11-10).
- Still out of scope (confirmed): PWM, MyActuator and SimpleMotion drivers, `MidiMain` and `MidiHandler`. Keep their feature macros off, in every executable.
- Fake the still-out-of-scope drivers at their interface (`MockMotorDriver`, `MockEncoder`). Test only the in-scope caller (correct calls, arguments, ordering, handling of return values and errors), never the driver internals.

## Test conventions (from `Firmware/Tests/README.md`)
- Use a fixture. It resets the global state so tests do not depend on order.
- Test through public interfaces (reports, commands, class interfaces), not private members.
- One behaviour per test case. The name is a statement that is true if the test passes.
- Never sleep. Advance the simulated time (`HostPlatform::advanceMs`); run threads with `HostRtos::runUntilBlocked` / `runUntilIdle` or the fixtures' `process()`.
- `characterization: ...` in the test name = documents what the firmware does today where correct behaviour is unverified. Also record it as NEEDS REVIEW in `Tests/claude/plans/PROGRESS.md`.
- `KNOWN_ISSUE("...")` = test describes the INTENDED behaviour but fails because of a suspected defect (suite stays green). `KNOWN_ISSUE_UB(...)` is the same for undefined behaviour and is skipped in sanitizer builds. Never write a test that expects a suspected buggy result without one of these markers. In a `KNOWN_ISSUE` test the first failing assertion must be the one that describes the defect, so `should_fail` cannot hide a different failure.
- New firmware sources: add to `FIRMWARE_SOURCES` in `CMakeLists.txt`; extend the fake platform minimally.
- Every fake documents what it models, its source (RM0090 chapter, HAL source, spec section), what it does not model, and a confidence rating, and is strict (fails the test on use the hardware would reject). See "Fake fidelity rules" in `Tests/claude/plans/TEST_PLAN.md`.
- Host is 64-bit, target is 32-bit ARM with a single-precision FPU. Flag any test whose result could differ on target: type sizes, struct packing/alignment, endianness, float vs double promotion, integer overflow, signed shifts, out-of-range float-to-int conversions.

## Known issues
- **`effectsForces` out-of-bounds read.** GCC 16's libstdc++ enables `_GLIBCXX_ASSERTIONS` at -O0, so this existing read aborts the doctest run (baseline: 192 of 317 cases ran). Exact case name: `requesting the forces of a not existing axis is rejected` (`cases/test_effects_calculator.cpp:1062`). Treat it as a suspected firmware bug. Do NOT fix the firmware. Exclude it with `--test-case-exclude=` in runs, list the exclusion in every report, and do not disable `_GLIBCXX_ASSERTIONS` globally.
- **CMSIS-DSP `arm_spline_f32` ASan finding** (vendor code, reached from `EffectsCalculator::evaluateReconstructionFilter`). Keep ASan on the whole build. In sanitizer runs skip only the two spline cases (`reconstruction of a constant signal is constant`, `reconstruction of a step never leaves the range of the input values and reaches the target`). Report it, never fix it.

- **Skipped crash.** The case `command with an unknown id is rejected` (`cases/test_command_execution.cpp:186`) stays skipped: `executeCommands` dereferences the command definition before the null check and terminates the run. Report it, never fix it.

## Test suite work
- The test-suite project is driven by `Tests/claude/plans/TEST_PLAN.md` (plan v2: keep the suite, cover more code, phases P0 to P10). Progress is tracked in `Tests/claude/plans/PROGRESS.md`.
- At the start of any test-suite session, read `Firmware/Tests/README.md`, both plan files, and resume from `PROGRESS.md`. Do not restart from scratch.
- `Tests/claude/plans/COVERAGE_AND_SCOPE.md` (covered / out of scope / not covered) must be kept current: update it whenever coverage numbers, scope decisions or test groups change (end of every phase or group), in the same commit as the `PROGRESS.md` update.
- Driver track: also read `Tests/claude/plans/DRIVER_TEST_PLAN.md` and the "Driver track" section of `PROGRESS.md`. Order is fixed: TMC4671, then ODrive, then VESC. Stop at the end of every phase.

## Model roles (driver track)
- Opus 5.5 (`claude-opus-5-5`, effort medium) is the technical lead: ambiguous specifications, deciding what each suite must prove (the requirements charter), serious test-design disagreements, final quality control. It writes no bulk tests.
- Sonnet 5.5 (`claude-sonnet-5-5`, effort medium) does most agent work: mapping existing tests to requirements, ordinary unit tests, edge-case tests with clear expected behaviour, first-pass review for gaps and false positives.
- Haiku 5.5 (`claude-haiku-5-5`) or a script summarises large test output. Prefer a script.
- Agent definitions live in `.claude/agents/` (created in phase D0). Do not hard-code a model other than these.
- Expected values come from the spec or vendor documentation, or a hand derivation written in a comment. Never copy the value the firmware currently produces into an assertion without `characterization:` and a NEEDS REVIEW entry.
- Specification files are supplied by me under `Tests/shared/specs/` and are never committed. Do not download them. Never invent register maps, opcodes or CAN frames; if a spec is missing, stop and ask.
