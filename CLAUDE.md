# Project notes

## Layout
- `Firmware/` - C/C++ firmware for STM32F407 (Makefile, ARM build). ~850 .c/.cpp/.h files outside `Firmware/Libraries/`.
- `Firmware/Tests/` - the existing host-side unit tests (doctest 2.4.11, C/C++, CMake). This is the suite we extend. Read `Firmware/Tests/README.md` first: it describes the architecture (fake platform, simulated time, mocks, fixtures) and the conventions below.
  - `cases/test_*.cpp` test cases, `mocks/`, `fakes/`, `support/`, `platform/include`, `platform/src`, `CMakeLists.txt`, `CMakePresets.json`.
- `Tests/` (repo root) - project-management files only: `TEST_PLAN.md` and `PROGRESS.md`. No test code lives here.
- Python helper scripts outside `Libraries/`: `Firmware/scripts/generate_memory.py` and `Firmware/FFBoard/USB/portable/synopsys/dwc2/dwc2_info.py`. Covered in Phase 5.

## Environment and commands
- Host: Linux, GCC 16, clang 23, CMake 4, default Makefiles generator (Ninja is only a shell alias, not on PATH). Python is available. Do not install anything.
- The suite must keep working on Windows (MinGW, `CMakePresets.json` presets `host` / `host-mingw`) and in GitHub Actions. Keep new code and CMake portable. Sanitizers are Linux/macOS only. Do not edit anything under `.github/`; if CI needs a change, describe it in `PROGRESS.md` and ask me.
- Build out-of-tree in `../ffb-test-build/<variant>` so nothing is written under `Firmware/`, not even gitignored build files.
- Exact configure / build / run / coverage / sanitizer commands: see "Environment" in `Tests/PROGRESS.md`. Keep that section accurate. It is the source of truth.

## Hard rules
- `Firmware/` is READ-ONLY, with one exception: `Firmware/Tests/` (but never edit `Firmware/Tests/third_party/doctest.h`). Never modify any other file under `Firmware/` (including the Makefile, ARM build, `Firmware/Libraries/`, TinyUSB, `eeprom.c` and other vendor code) unless I explicitly ask. Bugs in vendor code are reported, never fixed.
- Never edit the firmware sources to make them testable and never add `#ifdef` for tests to them. Use a test double in `fakes/` or a stub in `platform/src/host_stubs.cpp`.
- New tests, fakes, mocks and CMake changes go in `Firmware/Tests/` only, matching the existing structure and naming.
- Use doctest. Do not introduce a new test framework. Python tests use stdlib `unittest` (do not install pytest).
- Git: work on branch `add-test-suite`. Local commits are allowed (small and frequent). NEVER push, and never add or change a git remote. Do not change git config: use a one-off `git -c user.name="Claude Code" -c user.email= commit ...`. Never commit to main.
- No hardware access, flashing, serial/USB/CAN, or network access during tests.
- Ask before adding any dependency that has to be downloaded (e.g. CMake FetchContent). Do not install anything globally.
- Keep the existing tests: all 318 baseline cases (names in `Firmware/Tests/baseline/test_cases.txt`) must stay present with unchanged results. Never edit existing test files; add new files. After every CMake or test change, rebuild and run `tools/check_baseline.py`; `ffb_unit_tests` must report 0 failed (on GCC 16 that is 316 passed, 2 skipped with the documented exclusion).

## Test scope
- In scope: firmware logic outside `Firmware/Libraries/`, plus vendor code in these tiers:
  - Tier A: `eeprom.c` (directly, via flash tests on a fake flash with real NOR semantics), the FreeRTOS C++ wrappers (`c*.cpp`), the two Python helpers.
  - Tier B: TinyUSB device code in `Firmware/FFBoard/USB/`, tested against a fake USB controller.
  - Tier C: only the CMSIS-DSP functions the firmware calls (biquad, spline, whatever `EffectsCalculator` and `Filters` use).
- Not tested: ST HAL, CMSIS core, the FreeRTOS kernel, `Firmware/Targets/**` and CubeMX-generated code.
- Out of scope (confirmed): the driver/external-board code for TMC4671, ODrive, VESC, PWM, MyActuator, SimpleMotion, plus `TMCDebugBridge`, `MidiMain` and `MidiHandler`. Leave the MIDI and driver feature macros off.
- Fake out-of-scope drivers at their interface (`MockMotorDriver`, `MockEncoder`). Test only the in-scope caller (correct calls, arguments, ordering, handling of return values and errors), never the driver internals.

## Test conventions (from `Firmware/Tests/README.md`)
- Use a fixture. It resets the global state so tests do not depend on order.
- Test through public interfaces (reports, commands, class interfaces), not private members.
- One behaviour per test case. The name is a statement that is true if the test passes.
- Never sleep. Advance the simulated time (`HostPlatform::advanceMs`); run threads with `HostRtos::runUntilBlocked` / `runUntilIdle` or the fixtures' `process()`.
- `characterization: ...` in the test name = documents what the firmware does today where correct behaviour is unverified. Also record it as NEEDS REVIEW in `Tests/PROGRESS.md`.
- `KNOWN_ISSUE("...")` = test describes the INTENDED behaviour but fails because of a suspected defect (suite stays green). `KNOWN_ISSUE_UB(...)` is the same for undefined behaviour and is skipped in sanitizer builds. Never write a test that expects a suspected buggy result without one of these markers. In a `KNOWN_ISSUE` test the first failing assertion must be the one that describes the defect, so `should_fail` cannot hide a different failure.
- New firmware sources: add to `FIRMWARE_SOURCES` in `CMakeLists.txt`; extend the fake platform minimally.
- Host is 64-bit, target is 32-bit ARM with a single-precision FPU. Flag any test whose result could differ on target: type sizes, struct packing/alignment, endianness, float vs double promotion, integer overflow, signed shifts, out-of-range float-to-int conversions.

## Known issues
- **`effectsForces` out-of-bounds read.** GCC 16's libstdc++ enables `_GLIBCXX_ASSERTIONS` at -O0, so this existing read aborts the doctest run (baseline: 192 of 317 cases ran). Exact case name: `requesting the forces of a not existing axis is rejected` (`cases/test_effects_calculator.cpp:1062`). Treat it as a suspected firmware bug. Do NOT fix the firmware. Exclude it with `--test-case-exclude=` in runs, list the exclusion in every report, and do not disable `_GLIBCXX_ASSERTIONS` globally.
- **CMSIS-DSP `arm_spline_f32` ASan finding** (vendor code, reached from `EffectsCalculator::evaluateReconstructionFilter`). Keep ASan on the whole build. In sanitizer runs skip only the two spline cases (`reconstruction of a constant signal is constant`, `reconstruction of a step never leaves the range of the input values and reaches the target`). Report it, never fix it.

- **Skipped crash.** The case `command with an unknown id is rejected` (`cases/test_command_execution.cpp:186`) stays skipped: `executeCommands` dereferences the command definition before the null check and terminates the run. Report it, never fix it.

## Test suite work
- The test-suite project is driven by `Tests/TEST_PLAN.md` (plan v2: keep the suite, cover more code, phases P0 to P9). Progress is tracked in `Tests/PROGRESS.md`.
- At the start of any test-suite session, read `Firmware/Tests/README.md`, both plan files, and resume from `PROGRESS.md`. Do not restart from scratch.
