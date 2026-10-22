# Test Suite Progress

> This file is the resume point. Read "Current state" and "Next action" first.
> Claude updates it after every module, phase gate, decision and finding.
> Plan: `Tests/TEST_PLAN.md`. Rules: root `CLAUDE.md`. Suite docs: `Firmware/Tests/README.md`.

## Current state

- **Current phase:** P0 (safety net) done, **awaiting my approval** at the P0 gate. P1 not started.
- **Phase status:** gate green (see "P0 results"). Stop rule: P1 starts only after my go-ahead.
- **Branch:** add-test-suite (base commit `a5418ff5`, "Initial unit test system")
- **Last commit:** P0 commit (hash in the phase checklist). Earlier: `d12483c8` Phase 1 plan, `1876353e` F1, `2d91beed` F3 header.
- **Last updated:** 2026-10-21
- **Last session summary:** Installed plan v2 (`CLAUDE.md`, `Tests/TEST_PLAN.md`, this file) in this checkout and merged the add-test-suite progress into it. Did P0: baseline list (318 cases with expected results) + `tools/check_baseline.py` (also run by `ctest`), F1b (spline exclusion only in sanitizer builds with CMSIS-DSP), `tools/coverage_report.py` (plain gcov), README section. Gate green in 7 variants (GCC/clang, DSP on/off, ASan+UBSan) and with the `host-sanitize` preset. Measured baseline coverage: 2,136 of about 8,160 in-scope lines (26.2 %); compiled files 79.5 %. Read `.github/` (read-only): no workflow runs the host tests. Confirmed bug 5 in the source. Nothing under `Firmware/` outside `Firmware/Tests/` changed.

## Next action

**Waiting for my review of P0** (results below under "P0 results", open questions at the end). Do not start P1 without my go-ahead.

After approval: P1 as described in `Tests/TEST_PLAN.md` (new `cases/test_*.cpp` files only, gate after each batch, mutation spot-check, stop for approval).

## Phase checklist

| Phase | Executable | Est. lines gained | Status | Approved by me? | Commit |
|---|---|---|---|---|---|
| Phase 1 (explore and plan) | n/a | n/a | complete | yes, 2026-10-21 | |
| Review of the existing suite | n/a | n/a | complete | | |
| Batch 0 (plan v1) items done before plan v2 | n/a | n/a | F1 done, F3 header approved and committed | yes, 2026-10-21 | `1876353e` (F1), `2d91beed` (F3) |
| P0 Safety net | `ffb_unit_tests` | 0 (measured baseline 2,136 lines, 26.2 %) | done, awaiting approval | | see git log (`Tests: P0 safety net ...`) |
| P1 Gaps in compiled files | `ffb_unit_tests` | about 470 | not started | | |
| P2 Axis and main classes | `ffb_target_tests` (new) | about 790 | not started | | |
| P3 Flash and persistence | `ffb_flash_tests` (new) | about 365 | not started | | |
| P4 System, callbacks, misc | `ffb_target_tests` | about 460 | not started | | |
| P5 Buttons and analog | `ffb_target_tests` | about 775 | not started | | |
| P6 Encoders and buses | `ffb_target_tests` | about 640 | not started | | |
| P7 CAN and serial FFB | `ffb_target_tests` | about 420 | not started | | |
| P8 RTOS wrappers | `ffb_rtos_tests` (new) | about 365 | not started | | |
| P9 TinyUSB, CMSIS-DSP, Python helpers | `ffb_usb_tests` (new) | vendor, not in ladder | not started | | |

## Decisions log

| Date | Decision | Reason |
|---|---|---|
| 2026-10-20 | Extend the existing doctest suite in `Firmware/Tests/` instead of creating a new `Tests/` suite | It already exists with its CMake setup. A second suite would duplicate it and drift |
| 2026-10-20 | Rule is "`Firmware/` read-only except `Firmware/Tests/`" | Existing suite lives inside `Firmware/` |
| 2026-10-20 | Build out-of-tree | Nothing should be written under `Firmware/`, not even gitignored build files |
| 2026-10-20 | Use default Makefiles generator | Ninja is only a shell alias, not on PATH |
| 2026-10-20 | Exclude the `effectsForces` case, do not fix firmware, do not disable `_GLIBCXX_ASSERTIONS` globally | It is a real suspected bug. Assertions may catch others |
| 2026-10-20 | Exact exclusion name: `requesting the forces of a not existing axis is rejected` (`cases/test_effects_calculator.cpp:1062`, suite `EffectsCalculator`) | Confirmed from the source and from the abort output |
| 2026-10-20 | Build root `../ffb-test-build` (sibling of the repo checkout), one subdirectory per variant | Outside the repo and stable across sessions |
| 2026-10-21 | Q1: driver table confirmed. `TMCDebugBridge`, `MidiMain` and `MidiHandler` are out of scope. MIDI removed from the plan, MIDI feature macro stays off | My decision |
| 2026-10-21 | Q2: vendor code is tested in tiers. A: `eeprom.c`, FreeRTOS C++ wrappers, both Python helpers. B: TinyUSB device code vs a fake USB controller. C: only the CMSIS-DSP functions the firmware calls. Not tested: ST HAL, CMSIS core, FreeRTOS kernel, `Targets/**`, CubeMX code. Vendor bugs are reported, never fixed | My decision |
| 2026-10-21 | Q3: keep ASan on the whole build (including CMSIS-DSP). In sanitizer runs skip only the 2 spline cases. Keep bug #2 | Vendor DSP code is now in scope (Tier C) |
| 2026-10-21 | Q4/Q5: do F1. `ffb_unit_tests` stays exactly as is (same sources, same feature set, same fake `Axis.h`). New executables get their own feature header (real F407 `target_constants.h` minus driver macros, MIDI off; show me the diff before use). Firmware sources compiled per executable, only platform fakes shared. After every CMake change `ffb_unit_tests` must still give 316 passed, 0 failed, 2 skipped | Keeps the existing suite green and CI-safe |
| 2026-10-21 | Q6: plain `gcov` plus a small stdlib-only Python script in `Firmware/Tests/` for per-module coverage. Nothing installed | No downloads |
| 2026-10-21 | Q7: keep `../ffb-test-build/` | |
| 2026-10-21 | Q8: local commits allowed on `add-test-suite`. Never push, never touch remotes or git config. Identity via one-off `git -c user.name="Claude Code" -c user.email=` | No GitHub connection wanted |
| 2026-10-21 | Follow the conventions of `Firmware/Tests/README.md` (fixtures, public interfaces, one behaviour per test, statement names, no sleeps, `characterization:`, `KNOWN_ISSUE`, `KNOWN_ISSUE_UB`, no firmware `#ifdef`) | Existing suite style |
| 2026-10-21 | Keep Windows/MinGW, `CMakePresets.json` and GitHub Actions working. Do not edit `.github/` | README says tests run in CI |
| 2026-10-21 | Python helper tests use stdlib `unittest` | Do not install pytest |
| 2026-10-21 | Plan v2: keep all 318 baseline cases untouched; new tests go in new files; baseline names saved to `Firmware/Tests/baseline/test_cases.txt` and checked by `tools/check_baseline.py` | Guarantee that no existing test is lost or weakened |
| 2026-10-21 | Executables: `ffb_unit_tests` (unchanged), `ffb_target_tests`, `ffb_flash_tests`, `ffb_rtos_tests`, `ffb_usb_tests` | Fake `Axis.h`, fake `Flash_*` and the feature set cannot coexist with the real code in one binary |
| 2026-10-21 | Phase order: P0 safety net, P1 gaps in compiled files, then by risk and lines gained per effort (Axis first) | Cheapest gains first, highest-risk code early |
| 2026-10-21 | New `KNOWN_ISSUE` tests must fail on the assertion that describes the defect | `should_fail` hides a failure on the wrong line |
| 2026-10-21 | Stop after P0 and P1 for approval, then at the end of each phase unless I say "continue through P<n>" | I keep control of scope and usage |
| 2026-10-21 | Answer A (plan v1 session): commit identity `git -c user.name=MadBull.Ben -c user.email=`. Used for `d12483c8`, `1876353e`, `2d91beed`. **Conflicts with Q8 / `CLAUDE.md` v2 (`Claude Code`)**; the P0 commit follows `CLAUDE.md` v2. See open question 4 | My answer to question A |
| 2026-10-21 | `Firmware/Tests/` was moved out of the repo by accident and restored by me. Re-check after restore: no diff against HEAD under `Firmware/`, `ffb_unit_tests` with the exclusion = 316 passed / 0 failed / 2 skipped | Incident record |
| 2026-10-21 | F3 feature header `Firmware/Tests/platform/target_f407/target_constants.h` approved and committed (`2d91beed`): F407 copy minus `MIDI`, `TMCDEBUG`, `RMDCAN`, `TMC4671DRIVER`, `PWMDRIVER`, `ODRIVE`, `VESC`, `SIMPLEMOTION`. Not used by any target yet; for `ffb_target_tests` its directory goes before `platform/include` | My review of the diff |
| 2026-10-21 | Plan v2 installed in this checkout (`testing/OpenFFBoard`, branch `add-test-suite`). The v2 files came from `~/Downloads` (identical copies are untracked in the second checkout `Testing2/OpenFFBoard`, branch `feature/host-unit-tests` at `a5418ff5`, which has none of the add-test-suite commits). This file = v2 merged with the add-test-suite progress | My instruction to merge |
| 2026-10-21 | F1b: the 2 spline cases are excluded only when `FFB_TESTS_SANITIZE=ON` **and** CMSIS-DSP is used (DSP-off sanitizer builds run them again: 315 passed instead of 313). The effective list is cached as `FFB_TESTS_EXCLUDED_ARG` (INTERNAL) for the checker | Plan v2 P0 |
| 2026-10-21 | The baseline check also runs as a second `ctest` test `ffb_unit_tests_baseline` when Python 3 is found (otherwise a configure message, no failure). Not a new executable | Makes the safety net automatic in every `ctest` run |
| 2026-10-21 | Coverage ladder denominator = gcov executable lines of all `.c/.cpp` in `FFBoard/Src` + `FFBoard/UserExtensions/Src` minus drivers and MIDI (87 files, tier A wrappers and `eeprom.c` included). Files not compiled yet are estimated from their code lines (scaled by gcov/code ratio of the compiled files, 0.99). Headers are reported separately | Matches the 97-file inventory of the review |

## Environment

All commands run from the repo root (`OpenFFBoard/`). They are plain POSIX commands (work in bash/zsh; in fish replace `$(nproc)` with `(nproc)`).

- Host OS / compiler / generator: Linux (CachyOS, kernel 7.2), GCC 16.2.1, clang 23.1.3, CMake 4.4.4, Unix Makefiles (Ninja not on PATH)
- Test framework: doctest 2.4.11 (`Firmware/Tests/third_party/doctest.h`, do not edit)
- Test source location: `Firmware/Tests/` (documented in `Firmware/Tests/README.md`; README requires CMake >= 3.16, C++20, MinGW-w64 on Windows)
- CMSIS-DSP submodule: checked out, so the default build has `USE_DSP_FUNCTIONS=ON` (`FFB_TESTS_USE_DSP=AUTO`)
- Build directory (out-of-tree): `../ffb-test-build/<variant>` (variants: `default`, `nodsp`, `san`, `nodsp-san`, `clang23`, `clang23-san`, `cov`, `preset-host-sanitize`). `../ffb-test-build/clang` is a broken leftover (configure failed); delete it by hand
- Configure: `cmake -S Firmware/Tests -B ../ffb-test-build/default`
  - no DSP: add `-DFFB_TESTS_USE_DSP=OFF`
  - ASan+UBSan: add `-DFFB_TESTS_SANITIZE=ON`
  - clang: add `-DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++`
  - coverage: add `-DCMAKE_C_FLAGS=--coverage -DCMAKE_CXX_FLAGS=--coverage -DCMAKE_EXE_LINKER_FLAGS=--coverage`
- Build: `cmake --build ../ffb-test-build/default -j"$(nproc)"`
- Run (with exclusion): `../ffb-test-build/default/ffb_unit_tests --test-case-exclude="requesting the forces of a not existing axis is rejected"` (`-tce=` is the short form)
- Sanitizer run: `../ffb-test-build/san/ffb_unit_tests --test-case-exclude="requesting the forces of a not existing axis is rejected,reconstruction of a constant signal is constant,reconstruction of a step never leaves the range of the input values and reaches the target"`
- `ctest`: `ctest --test-dir ../ffb-test-build/default --output-on-failure`. Runs 2 tests: `ffb_unit_tests` with the exclusions (F1/F1b, cache lists `FFB_TESTS_EXCLUDE` and `FFB_TESTS_EXCLUDE_SANITIZE`, printed at configure time: `OpenFFBoard tests: excluded from ctest: ...`) and `ffb_unit_tests_baseline` (the baseline check)
- Baseline check (gate step 2): `python3 -B Firmware/Tests/tools/check_baseline.py --build-dir ../ffb-test-build/<variant>` (reads exclusions and sanitizer flag from `CMakeCache.txt`; exit 0 ok, 1 changed/missing, 2 crash). `--print-current` prints the current classification
- Presets: `CMakePresets.json` puts build dirs inside `Firmware/Tests/`. To use a preset out-of-tree: `cd Firmware/Tests && cmake --preset host-sanitize -B ../../../ffb-test-build/preset-host-sanitize`, then `ctest --test-dir <that dir>` (not `ctest --preset`, which uses the in-tree dir)
- Shell note: the interactive shell is fish, the tool shell is zsh; for loops over option strings use `bash -c` (zsh does not split `$var`)
- Coverage: `gcov` only (gcovr/lcov are not installed and must not be installed). Configure `cov` (see above), build, delete old data (`find ../ffb-test-build/cov -name '*.gcda' -delete`), `ctest --test-dir ../ffb-test-build/cov`, then `python3 -B Firmware/Tests/tools/coverage_report.py --build-dir ../ffb-test-build/cov` (Markdown to stdout; per file, per tier, ladder total, not-compiled files with estimates; `--headers` adds headers; `--gcov "llvm-cov gcov"` for clang)
- CI (GitHub Actions, read-only, P0): `.github/workflows/build-firmware.yml` (push/PR to `master`/`development`, manual, `workflow_call`: ARM `make` for `F411RE`, `F407VG`, `F407VG_DISCO` via `.github/actions/build-firmware`, uploads firmware), `doxygen.yml` (push to `master`), `release.yml` (tags `v*`: firmware + configurator builds on Windows/macOS, release). **No workflow builds or runs `Firmware/Tests/`**, although the README says the tests run in GitHub Actions. See open question 5
- Presets (`Firmware/Tests/CMakePresets.json`): `host`, `host-mingw` (MinGW Makefiles), `host-nodsp`, `host-sanitize`; configure/build/test presets for each. `host-sanitize` verified in P0 (out-of-tree): ctest 2/2 passed. `host-mingw` not verifiable here (no Windows)
- Sanity check after any build: `git status --short --ignored Firmware` must show nothing outside `Firmware/Tests/`.

## Baseline (existing suite, before any new work)

Commit `a5418ff5`. 318 test cases in total (`--list-test-cases`): 317 runnable + 1 already `skip()`-decorated. 17 cases carry a `KNOWN_ISSUE` marker (15 `KNOWN_ISSUE`, 2 `KNOWN_ISSUE_UB`); outside sanitizer builds they are `should_fail` and count as passed while they fail. With the exclusion, 16 of them run ("Failed as expected").

| Variant | Build | Run (with `effectsForces` exclusion) | Runtime |
|---|---|---|---|
| GCC 16, DSP ON, -O0 (default) | OK, 74 warnings (72 `-Wnon-c-typedef-for-linkage`, 1 `-Wmaybe-uninitialized` `Filters.cpp:30`, 1 `-Wformat` `CommandHandler.cpp:136`) | **316 passed, 0 failed, 2 skipped**; 2902 assertions, 34 failed inside `should_fail` cases; exit 0 | 0.33 s |
| Same, **unfiltered** | OK | Aborts with SIGABRT in `requesting the forces of a not existing axis is rejected`: 192 cases run, 126 not run, exit 134 | 0.36 s |
| GCC 16, DSP OFF | OK, 73 warnings | 316 passed, 0 failed, 2 skipped; exit 0 | not timed |
| clang 23, DSP ON | OK, 483 warnings (mostly `-Winconsistent-missing-override`, `-Wdeprecated-volatile`, `-Wunknown-warning-option`; notable: `-Warray-bounds` `CanCommandInterface.cpp:123`, `-Wformat` `CommandHandler.cpp:136`, `-Wvla-cxx-extension` `CmdParser.cpp:93`, `-Wunused-private-field` `UART.h:69`) | 316 passed, 0 failed, 2 skipped; exit 0 | not timed |
| GCC 16 ASan+UBSan, DSP ON | OK | **ASan stack-buffer-overflow** in CMSIS-DSP `arm_spline_f32` (bug #2), exit 1. With the 2 spline cases also excluded: 313 passed, 0 failed, 5 skipped, no UBSan/LSan findings | not timed |
| clang ASan+UBSan, DSP ON | OK | Same ASan finding; same clean result with the 2 extra exclusions | not timed |
| GCC 16 ASan+UBSan, DSP OFF | OK | 315 passed, 0 failed, 3 skipped, no findings (spline path not compiled) | not timed |

Notes:
- In sanitizer builds `KNOWN_ISSUE_UB` cases are `skip()`ped automatically (`support/TestHelpers.h:41`), so the `effectsForces` case is skipped there anyway; the exclusion is kept for consistency.
- Abort message: `/usr/include/c++/16/array:210: std::array<short,2>::operator[]: Assertion '__n < this->size()' failed.`

### Baseline line coverage (gcov, GCC 16, DSP ON, with exclusion)

Only the 24 firmware files currently compiled into the tests. Everything else in `FFBoard/` is 0 % (not compiled).

| File | Lines | File | Lines |
|---|---|---|---|
| CmdParser.cpp | 98.6 % of 140 | CanCommandInterface.cpp | 91.1 % of 237 |
| HidFFB.cpp | 98.3 % of 241 | MotorDriver.cpp | 90.9 % of 22 |
| FFBoardMainCommandThread.cpp | 94.7 % of 57 | HidCommandInterface.cpp | 85.9 % of 135 |
| CommandInterface.cpp | 94.1 % of 222 | Encoder.cpp | 85.7 % of 28 |
| EffectsCalculator.cpp | 93.3 % of 671 | CommandHandler.cpp | 83.6 % of 269 |
| CDCcomm.cpp | 70.3 % of 37 | UartHandler.cpp | 60.0 % of 15 |
| csemaphore.cpp | 60.5 % of 38 | ringbuffer.c | 59.4 % of 69 |
| UART.cpp | 57.4 % of 141 | ringbufferwrapper.cpp | 50.0 % of 72 |
| CAN.cpp | 47.4 % of 19 | cthread.cpp | 46.3 % of 54 |
| UsbHidHandler.cpp | 45.0 % of 20 | CanHandler.cpp | 42.9 % of 21 |
| PersistentStorage.cpp | 33.3 % of 27 | cmutex.cpp | 30.8 % of 39 |
| Filters.cpp | 27.6 % of 127 | ErrorHandler.cpp | 13.5 % of 104 |
### Review run 2026-10-21 (g++ 11.4.0 on Ubuntu 22.04, 2 cores, a script replicating `CMakeLists.txt`, CMake not available there)

| Build | Result |
|---|---|
| DSP on, unfiltered | 317 passed, 0 failed, 1 skipped; 2904 assertions, 35 failed inside `should_fail` cases; exit 0. No `effectsForces` abort (GCC 11 libstdc++ has no assertions at -O0). |
| DSP off | Same |
| ASan + UBSan, DSP on | Fails: `arm_spline_f32` stack-buffer-overflow (`arm_spline_interp_f32.c:223`). With the 2 spline cases excluded: 313 passed, 5 skipped, no other findings. |

- Total test cases: 318 (CmdParser 39, CommandExecution/CommandHandler 34, string interfaces 41, HID commands 22, CAN commands 20, HidFFB 53, EffectsCalculator 86, device mocks 17, platform 6). 17 carry a `KNOWN_ISSUE` marker and each fails on the assertion that matches its description. No empty or tautological tests; one real-time 300 ms wait (`test_cmdparser.cpp:460`).
- 23 of the 318 tests (device mocks, platform) mostly check the test doubles themselves.
- Inventory of firmware sources under `FFBoard/Src` and `FFBoard/UserExtensions/Src` (97 files): 24 tested (about 3,900 code lines), 63 with no tests (about 9,000), 10 drivers/MIDI out of scope (about 4,600).
- Mutation check on covered lines (operator flips, scratch copy): 42 of 48 killed. HidFFB 7/8, CmdParser 6/8, EffectsCalculator 8/10, HidCommandInterface 4/5, CommandInterface 6/6, CanCommandInterface 5/5, CommandHandler 6/6.
- Not verified: CMake itself, presets, `ctest`, the GitHub Actions workflow, Windows/MinGW, clang, GCC 16 behaviour.

## P0 results (2026-10-21)

Changes (all in `Firmware/Tests/`): `baseline/test_cases.txt` (F17), `tools/check_baseline.py` (F17), `tools/coverage_report.py` (F16), `CMakeLists.txt` (F1b, cached exclusion list, `ffb_unit_tests_baseline` ctest), `README.md` (options, baseline check, coverage). No existing test, mock, support or platform file changed (`git diff a5418ff5 -- Firmware/Tests/cases Firmware/Tests/support Firmware/Tests/mocks Firmware/Tests/fakes Firmware/Tests/platform/include Firmware/Tests/platform/src` is empty).

Baseline file: 318 cases = 300 `pass`, 15 `known_issue`, 2 `known_issue_ub`, 1 `skip`. Generated from the GCC 16 DSP-on build with `--print-current`; matches the Phase 1 counts. Checker tested against a modified baseline (changed result, renamed case, skip turned into pass: all reported) and against the unfiltered run (reports `CRASHED` in the `effectsForces` case, exit 2).

### Gate (P0, after the last CMake change)

| Variant | ctest | doctest (with exclusions) | Baseline check | Exclusions |
|---|---|---|---|---|
| GCC 16, DSP on (`default`) | 2/2 | 316 passed, 0 failed, 2 skipped | OK, 317 checked | `effectsForces` |
| GCC 16, DSP off (`nodsp`) | 2/2 | 316 / 0 / 2 | OK, 317 checked | `effectsForces` |
| GCC 16 ASan+UBSan, DSP on (`san`) | 2/2 | 313 / 0 / 5 | OK, 315 checked | `effectsForces` + 2 spline cases |
| GCC 16 ASan+UBSan, DSP off (`nodsp-san`) | 2/2 | 315 / 0 / 3 | OK, 317 checked | `effectsForces` |
| clang 23, DSP on (`clang23`) | 2/2 | 316 / 0 / 2 | OK, 317 checked | `effectsForces` |
| clang 23 ASan+UBSan, DSP on (`clang23-san`) | 2/2 | 313 / 0 / 5 | OK, 315 checked | `effectsForces` + 2 spline cases |
| GCC 16 coverage (`cov`) | 2/2 | 316 / 0 / 2 | (via ctest) | `effectsForces` |
| `host-sanitize` preset (out-of-tree) | 2/2 | | | `effectsForces` + 2 spline cases |

No sanitizer findings apart from the excluded spline cases. `git status --short --ignored Firmware` shows only `Firmware/Tests/` changes; no build files under `Firmware/`. Only `ffb_unit_tests` exists, so "every executable" = this one.

### Measured baseline coverage (gcov, GCC 16, DSP on, `effectsForces` excluded)

- In-scope sources: 87 files. Compiled into the tests: 24 files, **2,136 of 2,687** executable lines (**79.5 %**).
- Not compiled: 63 files, about 5,470 executable lines (estimate from 5,538 code lines, ratio 0.99; the estimate is within 1.3 % of gcov on the 24 compiled files).
- **All in-scope sources: 2,136 of about 8,160 lines = 26.2 %** (plan v2 estimate was about 24 % of about 8,900). Without tier A: 2,086 of about 7,340 (28.4 %).
- By tier (compiled files only): firmware sources 2,086 / 2,572 (81.1 %), firmware headers 147 / 187 (78.6 %); tier A sources (`cthread`, `csemaphore`, `cmutex`) 50 / 115 (43.5 %), headers 13 / 38; tier B: 3 header lines only (no TinyUSB source compiled); tier C (6 CMSIS-DSP sources) 118 / 147 (80.3 %).
- The line counts differ slightly from the Phase 1 table above (gcov JSON vs text output, e.g. CmdParser 136/136 instead of 138/140). P0 numbers are the reference from now on.

| Module | Covered / executable (incl. estimates) | % | Files not compiled |
|---|---|---|---|
| Command system | 932 / 1,018 | 91.6 | 0 |
| FFB | 859 / 1,058 | 81.2 | 1 (`SerialFFB`) |
| Device interfaces | 40 / 46 | 87.0 | 0 |
| Buffers | 73 / 135 | 54.1 | 0 |
| Filters / math | 35 / 139 | 25.2 | 1 (`CRC`) |
| Transport | 127 / 619 | 20.5 | 3 |
| Error handling | 14 / 99 | 14.1 | 0 |
| RTOS wrappers (tier A) | 50 / 566 | 8.8 | 9 |
| Persistence | 6 / 472 | 1.3 | 3 (incl. `eeprom.c`) |
| Axis | 0 / 755 | 0 | 2 |
| Button sources | 0 / 710 | 0 | 8 |
| Main classes | 0 / 659 | 0 | 10 |
| Encoders | 0 / 625 | 0 | 5 |
| Analog sources | 0 / 432 | 0 | 7 |
| System | 0 / 428 | 0 | 4 |
| Bus wrappers | 0 / 389 | 0 | 6 |
| USB descriptors | 0 / 9 | 0 | 4 (tables are data, not executable lines) |

Per-file numbers: rerun the coverage command (Environment).

### Other P0 findings

- Two baseline case names contain a comma (`constant force, spring and damper are summed`, `set periodic stores magnitude, offset, phase and period`). doctest splits filters at commas, so these could only be excluded with a `?` wildcard in place of the comma. No action needed now.
- On an abort doctest's XML reporter still closes the document; the aborting case carries `<Exception crash="true">` and `test_case_success="true"` (because of `should_fail`). The checker treats this as a crash.

## Out-of-scope driver files (confirmed)

| Path (under `Firmware/FFBoard/`) | Driver | Driver code or caller code? | Confirmed |
|---|---|---|---|
| `UserExtensions/Inc/TMC4671.h`, `UserExtensions/Src/TMC4671.cpp` (classes `TMC4671`, `TMC_1`, `TMC_2`, `TMC4671Biquad`, `TMC_ExternalEncoderUpdateThread`, enums/structs `TMC4671Limits` etc.) | TMC4671 | Driver | yes |
| `UserExtensions/Src/TMC4671_configs.cpp` | TMC4671 | Driver (hardware config tables) | yes |
| `UserExtensions/Inc/TMCDebugBridge.h`, `UserExtensions/Src/TMCDebugBridge.cpp` | TMC4671 | Driver tooling, out of scope | yes |
| `UserExtensions/Inc/ODriveCAN.h`, `UserExtensions/Src/ODriveCAN.cpp` (`ODriveCAN`, `ODriveCAN1`, `ODriveCAN2`) | ODrive | Driver | yes |
| `UserExtensions/Inc/VescCAN.h`, `UserExtensions/Src/VescCAN.cpp` (`VescCAN`, `VESC_1`, `VESC_2`) | VESC | Driver | yes |
| `UserExtensions/Inc/MotorPWM.h`, `UserExtensions/Src/MotorPWM.cpp` (`MotorPWM`, `PWMConfig`) | PWM | Driver | yes |
| `UserExtensions/Inc/RmdMotorCAN.h`, `UserExtensions/Src/RmdMotorCAN.cpp` (`RmdMotorCAN`, `RmdMotorCAN1`, `RmdMotorCAN2`) | MyActuator (RMD) | Driver | yes |
| `UserExtensions/Inc/MotorSimplemotion.h`, `UserExtensions/Src/MotorSimplemotion.cpp` (`MotorSimplemotion`, `MotorSimplemotion1/2`) | SimpleMotion | Driver | yes |
| `Targets/*/Core/Src/cpp_target_config.cpp` (`MotorPWM::timerConfig`, TMC pin config) | PWM, TMC4671 | Target hardware config | yes |
| `UserExtensions/Src/MidiMain.cpp`, `UserExtensions/Inc/MidiMain.h`, `Src/MidiHandler.cpp` | TMC4671 / MIDI | Out of scope (MIDI removed from the plan, feature macro off) | yes |
| `Inc/MotorDriver.h`, `Src/MotorDriver.cpp` (`MotorDriver` base class + driver `ClassChooser` registry) | shared base/interface | **Caller code, in scope.** Fake drivers at this interface (`MockMotorDriver` exists) | n/a |
| `Inc/Encoder.h`, `Src/Encoder.cpp` (`Encoder` base; all drivers above except PWM are also encoders) | shared base/interface | **Caller code, in scope.** `MockEncoder` exists | n/a |
| `Src/Axis.cpp`, `Inc/Axis.h` (driver/encoder choosers with `add_class<TMC_1/ODriveCAN1/VESC_1/...>`) | all | **Caller code, in scope.** Driver classes are compiled out by not defining the driver feature macros; mocks are registered instead | n/a |
| `UserExtensions/Src/FFBHIDMain.cpp`, `UserExtensions/Inc/FFBHIDMain.h` (includes `TMC4671.h`, `MotorPWM.h`) | TMC4671, PWM | **Caller code, in scope.** Needs the headers to compile, does not call driver internals | n/a |
| `Inc/Filters.h` (`friend TMC4671Biquad`) | TMC4671 | `Biquad` itself is in scope; only the friend declaration touches the driver | n/a |
| `Inc/constants.h`, `Inc/cpp_target_config.h`, `UserExtensions/Inc/ClassIDs.h`, `eeprom_addresses.*` | all | IDs/addresses/feature macros only. In scope as data (uniqueness checks) | n/a |

## Vendor and generated code outside `Firmware/Libraries/` (confirmed tiers)

| Path | What it is | Tier / decision |
|---|---|---|
| `Firmware/FFBoard/Src/eeprom.c`, `Inc/eeprom.h` | ST EEPROM emulation (AN3969) | **Tier A: tested directly** with the NOR-semantics fake flash (F9): page transfer, power loss, full page, invalid state |
| `Firmware/FFBoard/Src/c*.cpp` (`cthread`, `csemaphore`, `cmutex`, `cqueue`, `ctimer`, `ctasklet`, `cevent_groups`, `cworkqueue`, `cmem_pool`, `cread_write_lock`, `ccondition_variable`, `ctickhook`) + `Inc/*.hpp` | FreeRTOS C++ Wrappers (Michael Becker) | **Tier A: tested** against the fake RTOS; add F11 fakes only as needed |
| `Firmware/scripts/generate_memory.py`, `Firmware/FFBoard/USB/portable/synopsys/dwc2/dwc2_info.py` | Python helpers | **Tier A / Phase 5**, `unittest` |
| `Firmware/FFBoard/USB/**` | TinyUSB stack (MIT) | **Tier B: device code tested** against a fake USB controller (F15), own batch near the end |
| `Firmware/Libraries/CMSIS-DSP` | CMSIS-DSP | **Tier C: only the functions the firmware calls**: `arm_sin_f32`, `arm_cos_f32`, `arm_sqrt_f32`, `arm_biquad_cascade_df1_f32`/`_init_f32`, `arm_spline_f32`/`_init_f32`, `arm_pid_f32`/`_init_f32`/`_reset_f32`, `arm_std_f32` (called by `Filters.cpp`, `EffectsCalculator.cpp`, `HidFFB.cpp`, `Axis.cpp`) |
| `Firmware/Libraries/**` (rest), `Firmware/Targets/*/Drivers/**`, `Targets/*/Middlewares/**`, `Targets/*/Core/**` (CubeMX init, startup, linker scripts, `main.c`, `freertos.c`, ...) | ST HAL, CMSIS core, FreeRTOS kernel, generated code | **Not tested** |

Vendor bugs found are listed under "Suspected firmware bugs" and never fixed.

## Firmware map (in scope, outside `Libraries/`)

HAL/RTOS column: what each module touches. "Feature" is the `target_constants.h` macro that gates it; the current host test target only defines `CANBUS`, `CANCOMMANDS`, `UARTCOMMANDS`, so most modules below currently compile to empty files.

| Module | Files (`FFBoard/`) | Responsibility | HAL / RTOS / USB used | Feature gate |
|---|---|---|---|---|
| Command system | `Src/CmdParser.cpp`, `CommandHandler.cpp`, `CommandInterface.cpp`, `FFBoardMainCommandThread.cpp`, `HidCommandInterface.cpp`, `CanCommandInterface.cpp` | Parse/dispatch/reply | `HAL_GetTick`, thread notify, tusb HID | `CANCOMMANDS` |
| Transport | `Src/CDCcomm.cpp`, `UART.cpp`, `UartHandler.cpp`, `CAN.cpp`, `CanHandler.cpp`, `CANPort2B.cpp`, `global_callbacks.cpp`, `USBdevice.cpp`, `UsbHidHandler.cpp` | Byte/frame transport, ISR -> handler dispatch | HAL UART/CAN/ADC/TIM callbacks, tusb CDC/HID, UID | `CANBUS`, `CANTYPE_2B` |
| FFB | `Src/HidFFB.cpp`, `EffectsCalculator.cpp`, `SerialFFB.cpp` | PID reports, effect forces, reconstruction filter | `HAL_GetTick`, `micros()`, CMSIS-DSP | `USE_DSP_FUNCTIONS` |
| Axis | `Src/Axis.cpp`, `AxesManager.cpp` | Torque path: scaling, power, endstop, limiters, idle spring, expo, gear ratio, mech. effects, config, commands | TIM, EXTI, encoders/drivers via interfaces | always (needs real `Axis.h`) |
| Filters / math | `Src/Filters.cpp`, `Inc/FastAvg.h`, `Src/CRC.cpp`, `Inc/CRC.h` | Biquad, averages, CRC tables | `__REVSH` (CMSIS intrinsic) | `USE_DSP_FUNCTIONS` |
| Error handling | `Src/ErrorHandler.cpp` | Global error list, callbacks, commands | none | always |
| Persistence | `Src/PersistentStorage.cpp`, `flash_helpers.cpp`, `UserExtensions/Src/eeprom_addresses.c` (+ `eeprom.c`, Tier A) | Save/restore, flash variable access, dump/format, OTP | HAL FLASH, `vTaskDelay` | `USE_EEPROM_EMULATION` |
| System | `Src/SystemCommands.cpp`, `cppmain.cpp`, `ledEffects.cpp`, `voltagesense.cpp` | System commands, micros, LEDs, voltage/temperature sense | `NVIC_SystemReset`, UID/DEVID, `vTaskList`, GPIO, LL ADC macros, IWDG | various |
| Main classes | `UserExtensions/Src/FFBoardMain.cpp`, `FFBHIDMain.cpp`, `FFBHIDMain_commmands.cpp`, `FFBWheel.cpp`, `FFBJoystick.cpp`, `FFBHIDExt.cpp`, `CustomMain.cpp`, `CanInputMain.cpp`, `CanBridge.cpp`, `mainclass_chooser.cpp`, `Inc/ClassChooser.h` | Main class selection, HID gamepad report building, CAN bridge protocol | TIM, GPIO, tusb, `vTaskDelay` | `FFBWHEEL`, `FFBJOYSTICK`, `FFBHIDEXT`, `CANINPUTMAIN`, `CANBRIDGE`, `TIM_FFB` |
| Button sources | `Src/ButtonSource.cpp`, `SelectableInputs.cpp`, `UserExtensions/Src/LocalButtons.cpp`, `SPIButtons.cpp`, `ShifterAnalog.cpp`, `CanButtons.cpp`, `PCF8574.cpp`, `ButtonSources.cpp` | Button reading and decoding | GPIO, SPI, I2C, CAN | `LOCALBUTTONS`, `SPIBUTTONS`, `SHIFTERBUTTONS`, `CANBUTTONS`, `I2C_PORT` |
| Analog sources | `Src/AnalogSource.cpp`, `AnalogAxisProcessing.cpp`, `AdcHandler.cpp`, `UserExtensions/Src/LocalAnalog.cpp`, `CanAnalog.cpp`, `ADS111X.cpp`, `AnalogSources.cpp` | Analog axes, autoscale, filters | ADC DMA buffer, I2C, CAN | `ANALOGAXES`, `CANANALOG`, `I2C_PORT` |
| Encoders (non-driver) | `UserExtensions/Src/EncoderLocal.cpp`, `EncoderSSI.cpp`, `EncoderBissC.cpp`, `MtEncoderSPI.cpp`, `EncoderSources.cpp` | Quadrature/SSI/BiSS-C/MT6825 decoding, CRC/parity | TIM CNT, EXTI, SPI DMA | `LOCALENCODER`, `MTENCODERSPI` |
| Bus wrappers | `Src/SPI.cpp`, `SpiHandler.cpp`, `I2C.cpp`, `I2CHandler.cpp`, `TimerHandler.cpp`, `ExtiHandler.cpp` | Port/device arbitration, callbacks | HAL SPI/I2C, `portYIELD_FROM_ISR` | `I2CBUS` |
| Buffers | `Src/ringbuffer.c`, `ringbufferwrapper.cpp` | Ring buffer | none | always |
| USB descriptors | `UserExtensions/Src/usb_descriptors.cpp`, `usb_hid_1ffb_desc.c`, `usb_hid_2ffb_desc.c`, `usb_hid_gamepad.c` | Descriptor tables | none | `AXIS1_FFB_HID_DESC` etc. |
| (Out of scope) MIDI | `Src/MidiHandler.cpp`, `MidiMain.cpp` | MIDI | tusb MIDI | `MIDI` (keep off) |

Host compile probe (each file compiled alone with the current test flags, objects to scratch): compiles but empty because of feature gates: most `UserExtensions` files; fails today: `Axis.cpp` (fake `Axis.h` shadows the real one), `AxesManager.cpp`/`LocalAnalog.cpp` (`ADC_PINS`), `LocalButtons.cpp`/`ShifterAnalog.cpp` (`BUTTON_PINS`), `SPIButtons.cpp`/`EncoderSSI.cpp`/`EncoderBissC.cpp` (SPI HAL constants), `CANPort2B.cpp` (`CAN_FilterTypeDef`), `CRC.cpp` (`__REVSH`), `flash_helpers.cpp` (`__weak`, HAL FLASH), `SystemCommands.cpp` (`NVIC_SystemReset`), `USBdevice.cpp` (`HAL_GetUIDw2`), `voltagesense.cpp` (LL ADC macros), `ledEffects.cpp` (LED pins), `cppmain.cpp`, `global_callbacks.cpp`, `FFBHIDMain*.cpp` (`Control_t` from real `Axis.h`), RTOS wrappers needing queues/timers/event groups.

## Existing coverage and techniques (inventory)

Techniques already used in `Firmware/Tests/` (to be reused; see its README):
- **Link substitution**: fake platform in `platform/src/` implements HAL (`HAL_GPIO_*`, `HAL_UART_*`, `HAL_GetTick`), FreeRTOS (tasks, notifications, semaphores, `vTaskDelay`) and TinyUSB (`tud_cdc_n_write`, `tud_hid_n_report`), and the **`Flash_*` API itself** (map-backed, so `flash_helpers.cpp`/`eeprom.c` are not compiled).
- **Shadowed headers**: `platform/include/` replaces `main.h`, FreeRTOS headers, `target_constants.h`, `tusb_config.h`; `fakes/Axis.h` replaces the real `Axis.h` (include order matters).
- **Mocks of firmware interfaces**: `MockCommandHandler`, `MockCommandInterface`, `MockMotorDriver`, `MockEncoder`, `MockCANPort`; `PidHost` simulates the PID driver of the host; fixtures `CommandSystemFixture`, `FfbFixture`.
- **Simulated time/scheduler**: `HostPlatform::advanceMs`, `HostRtos::runUntilBlocked/runUntilIdle`; fixtures reset global state per test.
- **Markers**: `characterization:` names, `KNOWN_ISSUE(...)` (should_fail), `KNOWN_ISSUE_UB(...)` (skipped under sanitizers).
- README limitations to keep in mind: not compiled yet are `Axis`, the main classes, the real motor drivers/encoders and `SystemCommands`; USB descriptors are not parsed; no preemption.

Per-suite existing coverage (test case counts): CmdParser 39, CommandExecution/CommandHandler 34, StringCommandInterface/CDC/UART 41, HID_CommandInterface 22, CAN_CommandInterface 20, HidFFB 53, EffectsCalculator 86, Encoder/MotorDriver (mock interfaces) 17, platform self-test 6.

Coverage gaps (from gcov and the source map):
- **Not compiled at all**: Axis, AxesManager, SerialFFB, ErrorHandler commands, SystemCommands, flash_helpers, `eeprom.c`, all button/analog sources, non-driver encoders, CRC, FFBHIDMain + gamepad report, CanBridge, CanInputMain, CANPort2B, global_callbacks, SPI/I2C wrappers, ledEffects, voltagesense, USB descriptors, mainclass_chooser, TinyUSB device code.
- **Compiled but weakly covered**: ErrorHandler 13 %, Filters 28 % (only via effects), PersistentStorage 33 %, CanHandler/CAN 43-47 %, UsbHidHandler 45 %, ringbuffer/wrapper 50-59 % (wrap-around, overflow, peek paths), UART 57 %, CDCcomm 70 %, CommandHandler 84 % (help/info strings, class ID lookup), RTOS wrappers (`cthread`, `cmutex`, `csemaphore` 31-61 %).
- **EffectsCalculator**: high line coverage but no long-uptime tests (float timestamp precision, `micros()` wrap; see bug #4) and no test of `effectsForces` with a valid axis index > 0.

## Shared fakes and CMake changes (each is created in the phase that needs it, see TEST_PLAN.md)

Hard gate for Batch 0: after every CMake change `ffb_unit_tests` must still give 316 passed, 0 failed, 2 skipped (with the exclusion), in both DSP variants. Keep MinGW, presets and CI working.

| # | Fake / change | Location (`Firmware/Tests/`) | Purpose | Status |
|---|---|---|---|---|
| F1 | `FFB_TESTS_EXCLUDE` cache variable (default: the `effectsForces` case name) passed to `add_test` as `--test-case-exclude=` | `CMakeLists.txt` | Make `ctest` green and the exclusion visible in one place. Check CI impact first (older compilers do not abort on this case) | **done** `1876353e` (2026-10-21). Excluding the case on older compilers only skips a `KNOWN_ISSUE_UB` case; no CI runs the tests (P0) |
| F2 | Keep `ffb_unit_tests` exactly as is (same sources, same feature set, same fake `Axis.h`). Add `ffb_target_tests` (real `Axis.cpp`/`Axis.h`, `AxesManager`, `FFBHIDMain`, inputs, encoders, buses, CAN, system; no `fakes/Axis.h`), `ffb_rtos_tests` (the `cpp_freertos` wrappers), `ffb_flash_tests` (real `flash_helpers.cpp` + `eeprom.c` on fake HAL FLASH, without the `Flash_*` fake) and `ffb_usb_tests` (Tier B, real TinyUSB device code + fake USB controller). Firmware sources are compiled per executable; only the platform fakes are shared (static library). New cases go in separate directories (e.g. `cases_axis/`, `cases_flash/`, `cases_usb/`) outside the existing `cases/test_*.cpp` glob. Update `CMakePresets.json` so `cmake --preset host` builds all and `ctest` runs all | `CMakeLists.txt`, `CMakePresets.json` | The fake `Axis.h` and fake `Flash_*` cannot coexist with the real code in one binary. The README says every `cases/test_*.cpp` is picked up automatically, so new executables must not share that directory | approved |
| F3 | Own feature header for the new executables: read the real F407 target's `target_constants.h`, copy its feature macros, remove the driver macros (`TMC4671DRIVER`, `PWMDRIVER`, `ODRIVE`, `VESC`, `SIMPLEMOTION`, `RMDCAN`, `TMCDEBUG`), leave `MIDI` off, add pin tables (`BUTTON_PINS`, `ADC_PINS`, ...). Show me the resulting diff before use. `ffb_unit_tests` keeps its current header | `platform/target_f407/target_constants.h` (its directory goes before `platform/include` in the new executables only) | Compile the in-scope modules without the drivers | **done**: diff approved, committed `2d91beed`. Pin tables (`BUTTON_PINS`, `ADC_PINS`, ...) still to add in P2 batch 0 |
| F4 | Fake HAL SPI (records TX, scripted RX, DMA/IT completion callbacks driven by the test) | `platform/` | SPIButtons, EncoderSSI, EncoderBissC, MtEncoderSPI, SPI.cpp | proposed |
| F5 | Fake HAL I2C (scripted device register map per address, error injection) | `platform/` | PCF8574, ADS111X, I2C.cpp | proposed |
| F6 | Fake HAL ADC (settable DMA buffer, conversion-complete callback) + LL ADC voltage/temperature macros | `platform/` | LocalAnalog, AdcHandler, voltagesense | proposed |
| F7 | Fake HAL TIM (settable `CNT`, base start/stop record) and EXTI trigger | `platform/` | EncoderLocal, TimerHandler, ExtiHandler, Axis timers | proposed |
| F8 | Fake HAL CAN (filters, Tx mailboxes, Rx FIFO, error callbacks) | `platform/` | CANPort2B, global_callbacks | proposed |
| F9 | Fake HAL FLASH: RAM-backed sectors with real NOR semantics (erase to 0xFFFF, programming can only clear bits, error flags injectable, power-loss injection), `__weak`, OTP area | `platform/` | flash_helpers + `eeprom.c` (Tier A), page-transfer and power-loss scenarios. **Constraint:** `eeprom.c` dereferences absolute addresses (`*(__IO uint16_t*)PAGE0_BASE_ADDRESS`, `EEPROM_START_ADDRESS = 0x08004000`, cast to `uint32_t`), so the fake must `mmap` 2 x 16 KiB at exactly 0x08004000 (`MAP_FIXED_NOREPLACE`, fail loudly if taken); the OTP area likewise at its fixed address. Host-only technique (Linux/macOS; needs a Windows alternative or a skip for MinGW), verify under ASan | proposed |
| F10 | Misc CMSIS/HAL: `__REVSH`, `NVIC_SystemReset` (recorded, no exit), `HAL_GetUIDw0/1/2`, `HAL_GetDEVID/REVID`, IWDG refresh, `HAL_RCC_GetHCLKFreq`, LED pins | `platform/` | CRC, SystemCommands, USBdevice, cppmain, ledEffects | proposed |
| F11 | FreeRTOS: `xQueue*`, `xTimer*`, `xEventGroup*`, `xTimerPendFunctionCall`, `tskIDLE_PRIORITY`, recursive mutex, counting semaphores (simple non-preemptive implementations, with create-failure injection) | `platform/` | RTOS wrappers (Tier A) and modules that use cqueue/ctimer | proposed |
| F12 | New mocks: `MockButtonSource`, `MockAnalogSource`, `MockSPIDevice`/`MockI2CDevice` helpers, `AxisFixture` (real Axis + `MockMotorDriver` + `MockEncoder` selected via the public `Axis::driverChooser` / `Axis::encoderChooser`) | `mocks/`, `support/` | Caller-side tests at the interface | proposed |
| F14 | `-include cmath` (per-source compile option) for `Axis.cpp` if it still does not compile (README says `Axis.cpp` lacks `<cmath>`) | `CMakeLists.txt` | No firmware edit allowed | proposed, verify first |
| F15 | Fake USB device controller (`dcd_*` functions, scripted bus events/setup packets, endpoint buffers) so real TinyUSB device code runs on the host | `platform/` | Tier B (`ffb_usb_tests`) | proposed |
| F16 | Stdlib-only Python script that runs `gcov` and aggregates line coverage per module | `tools/` | Coverage report without installing gcovr | **done** P0: `tools/coverage_report.py` |
| F1b | When `FFB_TESTS_SANITIZE=ON` and DSP is on, `ctest` also excludes the 2 spline cases | `CMakeLists.txt` | Makes the `host-sanitize` preset pass (bug 6) | **done** P0, preset verified out-of-tree |
| F17 | `baseline/test_cases.txt` (the 318 baseline names) and `tools/check_baseline.py` | `Firmware/Tests/` | Detect any lost or changed baseline test | **done** P0, also run by `ctest` as `ffb_unit_tests_baseline` |

(F13 "build CMSIS-DSP without ASan" was dropped: ASan stays on for the whole build.)

## Phase plan (details in `Tests/TEST_PLAN.md`)

The per-module test plan from Phase 1 is now organised into phases P0 to P9 in `TEST_PLAN.md`. Status of each phase is in the "Phase checklist" at the top of this file. Estimated cumulative coverage: baseline about 24%, P1 29%, P2 38%, P3 42%, P4 47%, P5 56%, P6 63%, P7 68%, P8 72%. Measured numbers replace these after each phase.

## Test gaps found in review (to close in P1)

From the review run (g++ 11.4, scratch copy); not re-run in P0.

| # | File | Line | Change that survived | What no test covers |
|---|---|---|---|---|
| 1 | `CmdParser.cpp` | 53 | `>` to `>=` | Buffer exactly full (capacity boundary) |
| 2 | `CmdParser.cpp` | 156 | `&&` to `||` | Malformed `?` values in complex commands |
| 3 | `EffectsCalculator.cpp` | 235 | `+` to `-` | Phase offset of a square wave |
| 4 | `EffectsCalculator.cpp` | 1275 | `-` to `+` | Constant-force rate returns 0 after 1 s without updates |
| 5 | `HidFFB.cpp` | 423 | `!=` to `==` | Filter set-up when an effect starts |
| 6 | `HidCommandInterface.cpp` | 92 | `==` to `!=` | Reply branch for get-at commands |

Functions never run by any test (counts of never-run / total): ErrorHandler 15/20, PersistentStorage 5/7, Filters 5/11, UART 12/30, ringbuffer.c 6/14, RingBufferWrapper 9/17, CanHandler 6/8, CommandHandler 9/35, UsbHidHandler 5/8, EffectsCalculator 3/41 (`Run()`, `getInfo`, destructor).

## NEEDS REVIEW items

| # | Module | Test / location | Question | Resolved? |
|---|---|---|---|---|
| 1 | CAN command interface | `CanCommandInterface.cpp:118-124` | `msg.header.length >= 16/24` paths read `msg.data[8..23]` of an 8-byte array. Dead on classic CAN, out of bounds for any frame with length > 8. Intended for CAN-FD later? | no |
| 2 | EffectsCalculator | `evaluateReconstructionFilter` (`EffectsCalculator.cpp:584, 624-633`) | Timestamps are stored as `float` microseconds. Above 2^24 µs (≈16.8 s uptime) resolution drops (128 µs at ≈18 min, 512 µs near the 71.6 min `uint32_t` wrap). Also `now_us > last_known_time + 50000` misbehaves at the `micros()` wrap. Intended? To be confirmed by a test in Batch 2 | no |
| 3 | Existing suite | 17 `KNOWN_ISSUE` cases already in `Firmware/Tests/cases/` (CmdParser 4, EffectsCalculator 5, HidFFB 5, CAN/HID command interface 2 x "ERR never reported", StringCommandInterface 1) | Carried over unchanged; list with `grep -rn KNOWN_ISSUE Firmware/Tests/cases/` | n/a |

New tests that describe unclear behaviour use the `characterization:` name prefix and get a row here. Tests that describe intended behaviour but fail use `KNOWN_ISSUE` and get a row under "Suspected firmware bugs".

## Suspected firmware bugs

| # | Module | Location | Description | Evidence |
|---|---|---|---|---|
| 1 | EffectsCalculator | `FFBoard/Src/EffectsCalculator.cpp:1048` (`effectsForces`, `std::min<uint8_t>(cmd.adr, MAX_AXIS)`), read at line 1053 `effects_statslast[i].current[axis]` | Clamps the axis index to `MAX_AXIS` instead of `MAX_AXIS-1`, then reads `current[MAX_AXIS]` (`std::array<int16_t,2>`) out of bounds. Silent under old libstdc++, aborts under GCC 16 `_GLIBCXX_ASSERTIONS` at -O0 | Existing test `requesting the forces of a not existing axis is rejected` (`cases/test_effects_calculator.cpp:1062`, already `KNOWN_ISSUE_UB`). Message: `/usr/include/c++/16/array:210: ... std::array<short int, 2>::operator[] ... Assertion '__n < this->size()' failed.` -> SIGABRT after 192 of 317 cases. Excluded with `--test-case-exclude` |
| 2 | CMSIS-DSP (vendor, Tier C) reached from EffectsCalculator | `Libraries/CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_f32.c:223`, called from `EffectsCalculator.cpp:695` with `blockSize = 1` | `while( *pXq <= x[i+1] && blkCnt > 0 )` dereferences `pXq` before checking `blkCnt`, so it reads one float past the 1-element input (`interp_time`, a stack variable). Bug is in the vendor library; the firmware call is valid per the API. On target it reads one adjacent stack word; the result is not used, so likely harmless | ASan stack-buffer-overflow (GCC and clang), test cases `reconstruction of a constant signal is constant` and `reconstruction of a step never leaves the range of the input values and reaches the target`. Not visible without ASan, not in DSP-OFF build. Possible firmware-side workaround (not to be applied by me): pass a 2-element buffer |
| 3 | CommandHandler | `FFBoard/Src/CommandHandler.cpp:136` `snprintf(cmdhex,11,"0x%lX",cmd.cmdId)` | `%lX` with `uint32_t`: correct on ARM (where `uint32_t` is `unsigned long`), mismatched type on the 64-bit host. Portability issue rather than a target bug (`PRIX32` would fit both) | GCC `-Wformat` and clang `-Wformat` warnings |
| 4 | EffectsCalculator | see NEEDS REVIEW #2 | Possible precision/wrap problem of the reconstruction filter after long uptime | Code reading only; to be demonstrated by a test in Batch 2 |
| 5 | Command execution | `executeCommands` (find with `grep -rn executeCommands Firmware/FFBoard/Src`) | Dereferences the command definition before checking it for nullptr. A command with an unknown id terminates the run | **Confirmed in the source (P0):** `FFBoard/Src/FFBoardMainCommandThread.cpp:99-102`: `cmdDef = handler->getCommandFromId(cmd.cmdId)`, then `validFlags = cmdDef != nullptr`, then `if(cmdDef->flags & ...)` dereferences it unconditionally. Existing test `command with an unknown id is rejected` (`cases/test_command_execution.cpp:186`) stays `skip()`ped (known issue 3) |
| 6 | Test setup (not firmware) | `CMakePresets.json` `host-sanitize`, `CMakeLists.txt` | With the CMSIS-DSP submodule checked out, `ctest` in the sanitizer preset fails on the spline overflow (bug 2). Reproduced with the same flags by script, preset not run | Fixed by F1 (`1876353e`) + F1b (P0). Verified in P0: `cmake --preset host-sanitize -B <out-of-tree>`, `ctest` 2/2 passed |

## Untestable code and refactor suggestions

| Module | Location | Why untestable | Suggested refactor |
|---|---|---|---|
| Concurrency | all ISR/thread interaction | No preemption on the host | Keep ISR handlers small and testable as functions |
| CubeMX init | `Targets/*/Core/Src/*` | Hardware init only, not tested by decision | none |
| MidiMain | `UserExtensions/Src/MidiMain.cpp` | Out of scope. Constructs `TMC_1` directly (no `MotorDriver` interface use) | Take the driver from a chooser or as `MotorDriver*` |

## Host-vs-target caveats

| Module | Issue (size, packing, endianness, float/double, overflow, shift) | Test affected |
|---|---|---|
| all | Float -> int conversion of out-of-range values: ARM VCVT saturates, x86-64 SSE returns 0x80000000. Tests must not depend on out-of-range casts | Axis torque clipping, effect calculations |
| all | `long`/`size_t`/pointers are 64-bit on host, 32-bit on target. `int`/`int32_t` are 32-bit on both | `CommandHandler.cpp:136` `%lX`; `CommandReply::size()` uses `sizeof(CommandReply)` (contains `std::string`: 32 B host vs 24 B target), so reply memory accounting differs |
| HID reports | `__attribute__((packed))` structs in `ffb_defs.h`, `HidCommandInterface.h`: same layout with GCC on little-endian Linux; MinGW needs `-mno-ms-bitfields` (already set) | HID report tests |
| all | Implicit `double` promotion (e.g. `INTERNAL_AXIS_DAMPER_SCALER 0.7`, unsuffixed literals): target computes doubles in software; IEEE results are the same on host (SSE, no x87), so values match but timing does not | Axis/effects numeric tests: compare with tolerances |
| DSP | `USE_DSP_FUNCTIONS` uses the portable C CMSIS-DSP on the host, not the Cortex-M4 optimised paths. Small float differences possible | Filters, effects, Tier C: use tolerances, run both DSP ON and OFF |
| Endianness | Both little-endian. `memcpy` of CAN payloads (`CanCommandInterface.cpp`) behaves the same | CAN tests |
| Signed shifts | Arithmetic right shift on both (GCC/clang) | Encoder decoding |

## Review outcomes (Phase 3)

| Module | Reviewer | Findings | Fixed? |
|---|---|---|---|
| | | | |

## Verification results (Phase 4)

- Full suite result, all executables, both DSP variants (with `effectsForces` exclusion listed):
- ASan / UBSan findings (GCC and clang):
- Coverage per in-scope module and vendor tier:
- Uncovered code explained:
- Mutation check (caught / missed):
- `git status --short Firmware` shows only `Firmware/Tests/`: yes / no
- No build artefacts written under `Firmware/`: yes / no
- Nothing pushed, no remote or git config changed, `.github/` untouched: yes / no
- README instructions verified (`cmake -S . -B build`, presets, `ctest`): yes / no

## Open questions for me

1. **Coverage goal.** Is about 70% of in-scope firmware lines the right target? Going higher means testing interrupt and hardware-only code through fakes, which costs more per line.
2. **Pace.** Stop at the end of every phase (default), or run several phases in a row with a report at each? Say "continue through P<n>" to allow it.
3. **PID 1.01 specification** (optional): give the path of the spec file to have the `HidFFB` characterization tests verified against it. Without it that batch is skipped.
4. **Commit author name (conflict):** the first three add-test-suite commits use `MadBull.Ben` (my answer A), `CLAUDE.md` v2 says `Claude Code`. The P0 commit follows `CLAUDE.md` v2. Which one should stay? (Local commits only; author names can be changed before anything leaves the machine.)
5. **CI:** no GitHub Actions workflow runs the host tests (P0 finding). Should I describe a workflow job (configure, build, `ctest` with DSP on/off and sanitizers, MinGW) for you to add? I will not touch `.github/`.
6. **Second checkout:** `Testing2/OpenFFBoard` (branch `feature/host-unit-tests`) holds untracked copies of the v2 files and none of the add-test-suite work. Work continues only in `testing/OpenFFBoard` unless you say otherwise.

## Resume instructions

Start a new Claude Code session from the repo root and say:

> Read CLAUDE.md, Firmware/Tests/README.md, Tests/TEST_PLAN.md and Tests/PROGRESS.md. Resume from "Next action". Do not redo completed work, and do not move past an approval gate without my go-ahead.
