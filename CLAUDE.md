# Project notes

## Layout
- `Firmware/` - C/C++ firmware for STM32F407 (Makefile, ARM build). ~850 .c/.cpp/.h files outside `Firmware/Libraries/`.
- `Firmware/Tests/` - the existing host-side unit tests (doctest, C/C++, CMake). This is the suite we extend.
- `Tests/` (repo root) - project-management files only: `TEST_PLAN.md` and `PROGRESS.md`. No test code lives here.
- Two standard CPython helper .py files exist outside `Libraries/`: `Firmware/scripts/generate_memory.py` and `Firmware/FFBoard/USB/portable/synopsys/dwc2/dwc2_info.py`. They run on the PC, not the board. They are tested in Phase 5 (vendor tier A, pytest, separate from the C/C++ suite).

## Environment and commands
- Host: Linux, GCC 16, default Makefiles generator (Ninja is only a shell alias, not on PATH).
- Test framework: doctest.
- Build out-of-tree (scratchpad or another directory outside the repo) so nothing is written under `Firmware/`, not even gitignored build files.
- Build root: `../ffb-test-build/<variant>` (sibling of the repo checkout).
- Coverage: plain `gcov` plus a stdlib-only Python script in `Firmware/Tests/`. Do not install gcovr/lcov or anything else.
- Exact configure / build / run / coverage / sanitizer commands: see "Environment" in `Tests/PROGRESS.md`. Keep that section accurate. It is the source of truth.

## Hard rules
- `Firmware/` is READ-ONLY, with one exception: `Firmware/Tests/`. Never modify any other file under `Firmware/` (including the Makefile, ARM build, and `Firmware/Libraries/`) unless I explicitly ask.
- New tests, fakes and CMake changes go in `Firmware/Tests/` only, matching the existing structure and naming.
- Use doctest. Do not introduce a new test framework.
- Never commit to main. Test work goes on branch `add-test-suite`.
- Local commits on `add-test-suite` are allowed. NEVER push, and never add or change a git remote.
- Commit identity: use a one-off `git -c user.name=MadBull.Ben -c user.email= commit`. Never change git config (global or repo).
- No hardware access, flashing, serial/USB/CAN, or network access during tests.
- Ask before adding any dependency that has to be downloaded (e.g. CMake FetchContent).
- Do not install anything globally.

## Test scope
- In scope: firmware logic outside `Firmware/Libraries/`, plus vendor code in the tiers below.
- Out of scope: the driver/external-board code for TMC4671, ODrive, VESC, PWM, MyActuator, SimpleMotion; `TMCDebugBridge`; `MidiMain` and `MidiHandler` (the `MIDI` feature macro stays off in all test builds).
- Fake out-of-scope drivers at their interface. Test only the in-scope caller (correct calls, arguments, ordering, handling of return values and errors), never the driver internals.
- Vendor code is tested in tiers. Bugs found in vendor code are reported in `Tests/PROGRESS.md` and NEVER fixed.
  - Tier A: `FFBoard/Src/eeprom.c` (directly, via the flash tests on the NOR-semantics fake flash), the FreeRTOS C++ wrappers (`FFBoard/Src/c*.cpp`), and both Python helpers (Phase 5).
  - Tier B: TinyUSB device code in `FFBoard/USB`, tested against a fake USB controller, as its own batch near the end.
  - Tier C: only the CMSIS-DSP functions the firmware calls (biquad, spline, and whatever else `EffectsCalculator` and `Filters` use).
  - Not tested unless I say otherwise: the ST HAL, the FreeRTOS kernel, `Targets/**`, the rest of `Firmware/Libraries/`.

## Test executables
- `ffb_unit_tests` stays exactly as it is: same sources, same feature set (`Firmware/Tests/platform/include/target_constants.h`), same fake `Axis.h`.
- New executables get their own feature header, derived from the real F407 `target_constants.h` with the driver macros (`TMC4671DRIVER`, `PWMDRIVER`, `ODRIVE`, `VESC`, `SIMPLEMOTION`, `RMDCAN`, `TMCDEBUG`) removed and `MIDI` off. Show me the diff of that header before using it.
- Firmware sources are compiled per executable. Only the platform fakes are shared.
- After every CMake change: rebuild and confirm the existing suite still gives 316 passed, 0 failed, 2 skipped with the exclusion, and record it in `Tests/PROGRESS.md`.

## Known issues
- **`effectsForces` out-of-bounds read.** GCC 16's libstdc++ enables `_GLIBCXX_ASSERTIONS` at -O0, so this existing read now aborts the doctest run. The baseline was 192 of 317 test cases running before the abort.
  - Treat it as a suspected firmware bug. Do NOT fix the firmware. Do NOT write a test that expects the buggy result.
  - Exclude that one case with doctest's test-case-exclude option (`-tce`) in baseline and new runs, and list the exclusion explicitly in every report.
  - Do not disable `_GLIBCXX_ASSERTIONS` globally. It is catching a real bug and may catch others.
- **CMSIS-DSP `arm_spline_f32` over-read (vendor bug #2).** ASan reports a stack-buffer-overflow reached from `EffectsCalculator::evaluateReconstructionFilter`.
  - Keep ASan on the whole build, including CMSIS-DSP.
  - In sanitizer runs only, also exclude the two cases `reconstruction of a constant signal is constant` and `reconstruction of a step never leaves the range of the input values and reaches the target` with `-tce`, and list that exclusion in every report.

## Conventions
- Do not invent behaviour. If intent is unclear, test what the code actually does, mark it `// NEEDS REVIEW`, and record it in `Tests/PROGRESS.md`.
- Do not write a test that locks in a suspected firmware bug without flagging it.
- Host is 64-bit; target is 32-bit ARM with a single-precision FPU. Flag any test whose result could differ on target: type sizes, struct packing/alignment, endianness, float vs double promotion, integer overflow, signed shifts.
- Tests must be deterministic and independent: no sleeps, no shared global state between tests.

## Test suite work
- The test-suite project is driven by `Tests/TEST_PLAN.md`. Progress is tracked in `Tests/PROGRESS.md`.
- At the start of any test-suite session, read both files and resume from `PROGRESS.md`. Do not restart from scratch.
