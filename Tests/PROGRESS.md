# Test Suite Progress

> This file is the resume point. Read "Current state" and "Next action" first.
> Claude updates it after every module, phase gate, decision and finding.
> Plan: `Tests/TEST_PLAN.md`. Rules: root `CLAUDE.md`. Suite docs: `Firmware/Tests/README.md`.

## Current state

- **Current phase:** P2 (Axis and main classes, new `ffb_target_tests`), started 2026-10-23. P1 approved 2026-10-23.
- **Phase status:** batch 0 done (`ffb_target_tests`, 11 smoke cases, gate green in 6 variants; see "P2 batch 0"). Waiting for my decision on sanitizer handling of main-class deletion (bug 24) before the module agents start.
- **Branch:** add-test-suite (base commit `a5418ff5`, "Initial unit test system")
- **Last commit:** `7a35d2f1` (P1 review fixes). Earlier: `a79ead3a` P1 tests before review, `540d358a` / `bfdadc7f` PROGRESS records.
- **Last updated:** 2026-10-23
- **Last session summary:** P1 built by 5 parallel agents in isolated scratch workspaces (A command, B ffb, C errors/storage, D filters/small, E transport), merged in `a79ead3a`, reviewed by 3 fresh reviewer agents (r1-r3), fixed by the authors and merged again. 332 new cases in 15 new files, no existing test file changed. Coverage of the 24 compiled files 79.5 % -> 98.3 %; all in-scope sources 26.2 % -> 32.4 %. 16 new suspected firmware bugs (7-22), 4 vendor tier A issues (V1-V4), NEEDS REVIEW #4-#24. The session scratchpad (`/tmp`) was wiped between 2026-10-22 and 2026-10-23; the agents rebuilt their workspaces from the repo and re-applied their fixes (no work lost: everything merged was in git).

## Next action

**P2 batch 0 running.** Then: review and commit batch 0 (gate: `ffb_unit_tests` compile commands unchanged, 316/0/2 + baseline OK in all variants, `ffb_target_tests` green in default/nodsp/san), P2 module plan (behaviours per file), module agents, review, coverage (85 % per file without hardware-only code), mutation check, stop for approval.

## Phase checklist

| Phase | Executable | Est. lines gained | Status | Approved by me? | Commit |
|---|---|---|---|---|---|
| Phase 1 (explore and plan) | n/a | n/a | complete | yes, 2026-10-21 | |
| Review of the existing suite | n/a | n/a | complete | | |
| Batch 0 (plan v1) items done before plan v2 | n/a | n/a | F1 done, F3 header approved and committed | yes, 2026-10-21 | `8861c911` (F1), `ef4101cf` (F3) |
| P0 Safety net | `ffb_unit_tests` | 0 (measured baseline 2,136 lines, 26.2 %) | done | yes, 2026-10-22 | `2461c51e` |
| P1 Gaps in compiled files | `ffb_unit_tests` | about 470 (measured +504: 2,640 lines, 32.4 %) | done | yes, 2026-10-23 | `a79ead3a` (tests), `7a35d2f1` (review fixes) |
| P2 Axis and main classes | `ffb_target_tests` (new) | about 790 | in progress (batch 0 done) | | |
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
| 2026-10-21 | Q8: local commits allowed on `add-test-suite`. Never push, never touch remotes or git config. Identity via one-off `git -c` (name superseded 2026-10-22: `MadBull.Ben`) | No GitHub connection wanted |
| 2026-10-21 | Follow the conventions of `Firmware/Tests/README.md` (fixtures, public interfaces, one behaviour per test, statement names, no sleeps, `characterization:`, `KNOWN_ISSUE`, `KNOWN_ISSUE_UB`, no firmware `#ifdef`) | Existing suite style |
| 2026-10-21 | Keep Windows/MinGW, `CMakePresets.json` and GitHub Actions working. Do not edit `.github/` | README says tests run in CI |
| 2026-10-21 | Python helper tests use stdlib `unittest` | Do not install pytest |
| 2026-10-21 | Plan v2: keep all 318 baseline cases untouched; new tests go in new files; baseline names saved to `Firmware/Tests/baseline/test_cases.txt` and checked by `tools/check_baseline.py` | Guarantee that no existing test is lost or weakened |
| 2026-10-21 | Executables: `ffb_unit_tests` (unchanged), `ffb_target_tests`, `ffb_flash_tests`, `ffb_rtos_tests`, `ffb_usb_tests` | Fake `Axis.h`, fake `Flash_*` and the feature set cannot coexist with the real code in one binary |
| 2026-10-21 | Phase order: P0 safety net, P1 gaps in compiled files, then by risk and lines gained per effort (Axis first) | Cheapest gains first, highest-risk code early |
| 2026-10-21 | New `KNOWN_ISSUE` tests must fail on the assertion that describes the defect | `should_fail` hides a failure on the wrong line |
| 2026-10-21 | Stop after P0 and P1 for approval, then at the end of each phase unless I say "continue through P<n>" | I keep control of scope and usage |
| 2026-10-21 | Answer A (plan v1 session): commit identity `git -c user.name=MadBull.Ben -c user.email=`. Used for `f46b31f5`, `8861c911`, `ef4101cf`. **Conflicts with Q8 / `CLAUDE.md` v2 (`Claude Code`)**; the P0 commit follows `CLAUDE.md` v2. See open question 4 | My answer to question A |
| 2026-10-23 | Commit identity: author and committer `MadBull.Ben` with an **empty email** (`git -c user.name=MadBull.Ben -c user.email= commit`); the email must not appear anywhere. All 11 add-test-suite commits rewritten with `git filter-branch` (metadata and file contents), `refs/original` deleted. Hash map: `d12483c8`->`f46b31f5`, `1876353e`->`8861c911`, `2d91beed`->`ef4101cf`, `d08e0e1b`->`2461c51e`, `24417fa9`->`359c2a6a`, `d17cb295`->`124fa93e`, `93b82a32`->`a79ead3a`, `32cc2ce1`->`540d358a`, `17e09826`->`bfdadc7f`, `449b63ae`->`7a35d2f1`, `1568343e`->`9a0a7d05` (hashes elsewhere in this file are the new ones) | My decision (privacy) |
| 2026-10-23 | P1 approved; start P2 | My go-ahead |
| 2026-10-23 | Pace: stop at the end of every phase (P2 to P9) for my approval | My answer to open question 2 |
| 2026-10-23 | Coverage goal: higher than the 70 % of plan v2 | My answer to open question 1 |
| 2026-10-23 | **Coverage goal 85 % or more; hardware-only code is ignored** (not tested, not counted). Mechanism: reviewed list `Firmware/Tests/tools/coverage_exclusions.txt` (file:range + reason), applied by `coverage_report.py` to covered and executable lines (`--no-exclusions` for raw numbers). Unreachable/defensive code stays counted. Per-file target for every phase 85 %. New fakes are modelled on the real ST HAL source and cite the function they reproduce. TEST_PLAN "Coverage goal" updated | My decision after the fake-fidelity discussion |
| 2026-10-23 | The specification behind the `HidFFB` characterization tests is the USB HID PID 1.01 (Physical Interface Device class) | My correction to open question 3 |
| 2026-10-23 | P1 review: 3 fresh reviewer agents (r1-r3), fixes applied by the authoring agents, merged by the main agent (no file changed by two groups). Bug 21 (line 536) moved from NEEDS REVIEW to suspected bugs because the code comment states the intent. The ringbuffer KNOWN_ISSUE_UB case became a skipped case because it read indeterminate memory in the test itself | Review r1-r3 |
| 2026-10-22 | Use `MadBull.Ben` for all commits. The 2 P0 commits were rewritten (local rebase, author reset): `88dc8516` -> `d08e0e1b`, `0bbec1cb` -> `24417fa9` (both rewritten again on 2026-10-23, see below). `CLAUDE.md` identity line updated. P0 approved, start P1 | My decision |
| 2026-10-21 | `Firmware/Tests/` was moved out of the repo by accident and restored by me. Re-check after restore: no diff against HEAD under `Firmware/`, `ffb_unit_tests` with the exclusion = 316 passed / 0 failed / 2 skipped | Incident record |
| 2026-10-21 | F3 feature header `Firmware/Tests/platform/target_f407/target_constants.h` approved and committed (`ef4101cf`): F407 copy minus `MIDI`, `TMCDEBUG`, `RMDCAN`, `TMC4671DRIVER`, `PWMDRIVER`, `ODRIVE`, `VESC`, `SIMPLEMOTION`. Not used by any target yet; for `ffb_target_tests` its directory goes before `platform/include` | My review of the diff |
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
- `ctest`: `ctest --test-dir ../ffb-test-build/default --output-on-failure`. Runs 3 tests: `ffb_unit_tests` with the exclusions (F1/F1b, cache lists `FFB_TESTS_EXCLUDE` and `FFB_TESTS_EXCLUDE_SANITIZE`, printed at configure time: `OpenFFBoard tests: excluded from ctest: ...`), `ffb_unit_tests_baseline` (the baseline check) and `ffb_target_tests` (P2, cases in `cases_target/`, own exclusion list `FFB_TARGET_TESTS_EXCLUDE`, default empty, printed as `OpenFFBoard tests: ffb_target_tests excluded from ctest: ...`)
- Run `ffb_target_tests` directly: `../ffb-test-build/default/ffb_target_tests` (no exclusion needed; in sanitizer builds the cases marked `DELETES_FFBHIDMAIN` skip themselves)
- Baseline check (gate step 2): `python3 -B Firmware/Tests/tools/check_baseline.py --build-dir ../ffb-test-build/<variant>` (reads exclusions and sanitizer flag from `CMakeCache.txt`; exit 0 ok, 1 changed/missing, 2 crash). `--print-current` prints the current classification
- Presets: `CMakePresets.json` puts build dirs inside `Firmware/Tests/`. To use a preset out-of-tree: `cd Firmware/Tests && cmake --preset host-sanitize -B ../../../ffb-test-build/preset-host-sanitize`, then `ctest --test-dir <that dir>` (not `ctest --preset`, which uses the in-tree dir)
- Shell note: the interactive shell is fish, the tool shell is zsh; for loops over option strings use `bash -c` (zsh does not split `$var`)
- Coverage: `gcov` only (gcovr/lcov are not installed and must not be installed). Configure `cov` (see above), build, delete old data (`find ../ffb-test-build/cov -name '*.gcda' -delete`), `ctest --test-dir ../ffb-test-build/cov`, then `python3 -B Firmware/Tests/tools/coverage_report.py --build-dir ../ffb-test-build/cov` (Markdown to stdout; hardware-only ranges from `tools/coverage_exclusions.txt` are left out, `--no-exclusions` for raw numbers; per file, per tier, ladder total, not-compiled files with estimates; `--headers` adds headers; `--gcov "llvm-cov gcov"` for clang)
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
| F1 | `FFB_TESTS_EXCLUDE` cache variable (default: the `effectsForces` case name) passed to `add_test` as `--test-case-exclude=` | `CMakeLists.txt` | Make `ctest` green and the exclusion visible in one place. Check CI impact first (older compilers do not abort on this case) | **done** `8861c911` (2026-10-21). Excluding the case on older compilers only skips a `KNOWN_ISSUE_UB` case; no CI runs the tests (P0) |
| F2 | Keep `ffb_unit_tests` exactly as is (same sources, same feature set, same fake `Axis.h`). Add `ffb_target_tests` (real `Axis.cpp`/`Axis.h`, `AxesManager`, `FFBHIDMain`, inputs, encoders, buses, CAN, system; no `fakes/Axis.h`), `ffb_rtos_tests` (the `cpp_freertos` wrappers), `ffb_flash_tests` (real `flash_helpers.cpp` + `eeprom.c` on fake HAL FLASH, without the `Flash_*` fake) and `ffb_usb_tests` (Tier B, real TinyUSB device code + fake USB controller). Firmware sources are compiled per executable; only the platform fakes are shared (static library). New cases go in separate directories (e.g. `cases_axis/`, `cases_flash/`, `cases_usb/`) outside the existing `cases/test_*.cpp` glob. Update `CMakePresets.json` so `cmake --preset host` builds all and `ctest` runs all | `CMakeLists.txt`, `CMakePresets.json` | The fake `Axis.h` and fake `Flash_*` cannot coexist with the real code in one binary. The README says every `cases/test_*.cpp` is picked up automatically, so new executables must not share that directory | approved |
| F3 | Own feature header for the new executables: read the real F407 target's `target_constants.h`, copy its feature macros, remove the driver macros (`TMC4671DRIVER`, `PWMDRIVER`, `ODRIVE`, `VESC`, `SIMPLEMOTION`, `RMDCAN`, `TMCDEBUG`), leave `MIDI` off, add pin tables (`BUTTON_PINS`, `ADC_PINS`, ...). Show me the resulting diff before use. `ffb_unit_tests` keeps its current header | `platform/target_f407/target_constants.h` (its directory goes before `platform/include` in the new executables only) | Compile the in-scope modules without the drivers | **done**: diff approved, committed `ef4101cf`. Pin tables (`BUTTON_PINS`, `ADC_PINS`, ...) still to add in P2 batch 0 |
| F4 | Fake HAL SPI (records TX, scripted RX, DMA/IT completion callbacks driven by the test) | `platform/` | SPIButtons, EncoderSSI, EncoderBissC, MtEncoderSPI, SPI.cpp | proposed |
| F5 | Fake HAL I2C (scripted device register map per address, error injection) | `platform/` | PCF8574, ADS111X, I2C.cpp | proposed |
| F6 | Fake HAL ADC (settable DMA buffer, conversion-complete callback) + LL ADC voltage/temperature macros | `platform/` | LocalAnalog, AdcHandler, voltagesense | proposed |
| F7 | Fake HAL TIM (settable `CNT`, base start/stop record) and EXTI trigger | `platform/src/host_target.cpp`, `support/HostTarget.h` | EncoderLocal, TimerHandler, ExtiHandler, Axis timers | **done** P2 batch 0 (models `HAL_TIM_Base_Start/_IT`, `_Stop/_IT`, `HAL_TIM_IRQHandler` update path, `HAL_GPIO_EXTI_IRQHandler`) |
| F8 | Fake HAL CAN (filters, Tx mailboxes, Rx FIFO, error callbacks) | `platform/` | CANPort2B, global_callbacks | proposed |
| F9 | Fake HAL FLASH: RAM-backed sectors with real NOR semantics (erase to 0xFFFF, programming can only clear bits, error flags injectable, power-loss injection), `__weak`, OTP area | `platform/` | flash_helpers + `eeprom.c` (Tier A), page-transfer and power-loss scenarios. **Constraint:** `eeprom.c` dereferences absolute addresses (`*(__IO uint16_t*)PAGE0_BASE_ADDRESS`, `EEPROM_START_ADDRESS = 0x08004000`, cast to `uint32_t`), so the fake must `mmap` 2 x 16 KiB at exactly 0x08004000 (`MAP_FIXED_NOREPLACE`, fail loudly if taken); the OTP area likewise at its fixed address. Host-only technique (Linux/macOS; needs a Windows alternative or a skip for MinGW), verify under ASan | proposed |
| F10 | Misc CMSIS/HAL: `__REVSH`, `NVIC_SystemReset` (recorded, no exit), `HAL_GetUIDw0/1/2`, `HAL_GetDEVID/REVID`, IWDG refresh, `HAL_RCC_GetHCLKFreq`, LED pins | `platform/src/host_target.cpp`, `platform/target_f407/main.h` | CRC, SystemCommands, USBdevice, cppmain, ledEffects | **done** P2 batch 0 (IWDG/LED: existing no-op fakes kept; GPIO ports A-E with `MX_GPIO_Init` idle levels; `__REV/__REV16/__REVSH`; tinyusb `tusb_rhport_init`, `tud_task_ext`, `tud_speed_get`) |
| F11 | FreeRTOS: `xQueue*`, `xTimer*`, `xEventGroup*`, `xTimerPendFunctionCall`, `tskIDLE_PRIORITY`, recursive mutex, counting semaphores (simple non-preemptive implementations, with create-failure injection) | `platform/` | RTOS wrappers (Tier A) and modules that use cqueue/ctimer | proposed |
| F12 | New mocks: `MockButtonSource`, `MockAnalogSource`, `MockSPIDevice`/`MockI2CDevice` helpers, `AxisFixture` (real Axis + `MockMotorDriver` + `MockEncoder` selected via the public `Axis::driverChooser` / `Axis::encoderChooser`) | `mocks/`, `support/` | Caller-side tests at the interface | `TargetFixture` + `AxisFixture` **done** P2 batch 0 (`support/AxisFixture.h`); button/analog/SPI/I2C mocks still proposed (P5/P6) |
| F14 | `-include cmath` (per-source compile option) for `Axis.cpp` if it still does not compile (README says `Axis.cpp` lacks `<cmath>`) | `CMakeLists.txt` | No firmware edit allowed | **done** P2 batch 0: needed only with DSP off (bug 25); clang also needs `-Wno-c++11-narrowing` on `Axis.cpp` (bug 26) |
| F15 | Fake USB device controller (`dcd_*` functions, scripted bus events/setup packets, endpoint buffers) so real TinyUSB device code runs on the host | `platform/` | Tier B (`ffb_usb_tests`) | proposed |
| F16 | Stdlib-only Python script that runs `gcov` and aggregates line coverage per module | `tools/` | Coverage report without installing gcovr | **done** P0: `tools/coverage_report.py` |
| F1b | When `FFB_TESTS_SANITIZE=ON` and DSP is on, `ctest` also excludes the 2 spline cases | `CMakeLists.txt` | Makes the `host-sanitize` preset pass (bug 6) | **done** P0, preset verified out-of-tree |
| F17 | `baseline/test_cases.txt` (the 318 baseline names) and `tools/check_baseline.py` | `Firmware/Tests/` | Detect any lost or changed baseline test | **done** P0, also run by `ctest` as `ffb_unit_tests_baseline` |

(F13 "build CMSIS-DSP without ASan" was dropped: ASan stays on for the whole build.)

## Phase plan (details in `Tests/TEST_PLAN.md`)

The per-module test plan from Phase 1 is now organised into phases P0 to P9 in `TEST_PLAN.md`. Status of each phase is in the "Phase checklist" at the top of this file. Estimated cumulative coverage: baseline about 24%, P1 29%, P2 38%, P3 42%, P4 47%, P5 56%, P6 63%, P7 68%, P8 72%. Measured numbers replace these after each phase.

## P2 batch 0 (2026-10-23)

New executable `ffb_target_tests` (CMake block appended, nothing above it changed; `ffb_unit_tests` compile commands identical in all 6 build dirs, checked against copies saved before the change).
- Feature header `platform/target_f407/target_constants.h` + standalone target `main.h` / `cmsis_os.h` in `platform/target_f407/` (first in the include path). No `fakes/`: real `Axis.h`. Cases in `cases_target/test_*.cpp`. Own ctest exclusion list `FFB_TARGET_TESTS_EXCLUDE` (empty).
- **Deviations from TEST_PLAN/PROGRESS:** platform fakes are compiled per executable (they include `main.h`/`target_constants.h`), not one shared static library. `-funsigned-char` for `ffb_target_tests` only (ARM `char` is unsigned; `CanBridge.h` needs it); `ffb_unit_tests` keeps signed `char` (host-vs-target caveat).
- Firmware compiled: the 24 `ffb_unit_tests` sources, the P2 targets (`Axis`, `AxesManager`, `FFBoardMain`, `FFBHIDMain`, `FFBHIDMain_commmands`, `FFBWheel`, `FFBJoystick`, `FFBHIDExt`, `CustomMain`, `mainclass_chooser`) and what the registries pull in (`CanBridge`, `CanInputMain`, `SystemCommands`, `SelectableInputs`, `ButtonSource`, `AnalogSource`, `AnalogAxisProcessing`, `AdcHandler`, `SerialFFB`, `TimerHandler`, `ExtiHandler`, `USBdevice`, `SPI`, `SpiHandler`, `I2C`, `I2CHandler`, `CRC`, `ButtonSources`, `AnalogSources`, `EncoderSources`, `LocalButtons`, `SPIButtons`, `ShifterAnalog`, `PCF8574`, `CanButtons`, `LocalAnalog`, `CanAnalog`, `ADS111X`, `EncoderLocal`, `MtEncoderSPI`, `EncoderBissC`, `EncoderSSI`, USB descriptors). Not compiled: `CANPort2B.cpp` (P7), drivers/MIDI.
- Stubs: `platform/src/host_target_config.cpp` replaces `Targets/**/cpp_target_config.cpp` (same objects; `canport` is a `MockCANPort` until the CAN fake, P7). `platform/src/host_stubs_target.cpp`: deferred parts of `cppmain.cpp`, `global_callbacks.cpp`, `voltagesense.cpp` (P4), each to be removed when the real source is compiled. `HAL_SPI_*`/`HAL_I2C_*` are link stubs that record the call; `TargetFixture` fails a test that calls an unmodelled HAL function (until F4/F5/F6).
- Fakes (F7, F10) in `platform/src/host_target.cpp` + `support/HostTarget.h`, each citing the ST HAL function it models.
- Fixtures `TargetFixture`, `AxisFixture` (`support/AxisFixture.h`). 11 smoke cases (`TargetSetup`), 2 KNOWN_ISSUE (bugs 23, 24).
- Gate (all with documented exclusions): `ffb_unit_tests` 646/0/4 (default, nodsp, clang23), 643/0/7 (san, clang23-san), 645/0/5 (nodsp-san), baseline OK everywhere; `ffb_target_tests` 11/0/0 (normal builds), 9 passed + 2 skipped (`DELETES_FFBHIDMAIN`) in sanitizer builds; ctest 3/3; no sanitizer findings; no warnings from `Firmware/Tests` files. Verified again by the main agent.
- Hardware-only candidate: `USBdevice.cpp:33-38` (`USBdevice::Run`, TinyUSB stack loop, tier B) - to be decided in the P2 review.
- Notes for module agents: 1-axis `HidFFB` default direction mask 0x02 (FFBWheel sets 0x04 in `usbInit()`); `FFBHIDMain::Run` is re-entered by the fake kernel (timer start returns `HAL_ERROR` when BUSY, like the HAL); motor start needs `tud_connected()`; fade-in 1 s at 1 kHz; axes inverted by default; main-class axis via `CommandHandler::getHandlerFromClassName("axis", 0)`.

## P1 plan (2026-10-22)

Executable `ffb_unit_tests`, new `cases/test_*.cpp` files only. Gaps taken from the P0 coverage build (never-run functions per file). Build: 5 subagents in parallel, each in an isolated scratch copy of `Firmware/Tests` (firmware symlinked read-only, own build dirs `../ffb-test-build/p1-<group>[-nodsp|-san]`); the main agent integrates their files and any additive fake changes, runs the gate and records results here.

| Group | Owns (new files) | Firmware covered | Behaviours to test |
|---|---|---|---|
| A command | `test_cmdparser_gaps.cpp`, `test_command_handler_gaps.cpp`, `test_hid_command_interface_gaps.cpp` | `CmdParser`, `CommandHandler`, `HidCommandInterface`, `CommandInterface` | Survivors 1, 2, 6 (buffer exactly full, malformed `?` values, get-at reply branch); `CommandHandler::command`, `setInstance`, `logsEnabled`, `logSerial`, `logSerialDebug`, `getHelpstring`, `getCommandHandlerInstance`, `overrideCommandFlags_INT`; `HID_CommandInterface::transferComplete`; `CommandInterface::readyToSend` |
| B ffb | `test_effects_calculator_gaps.cpp`, `test_hid_ffb_gaps.cpp` | `EffectsCalculator`, `HidFFB` | Survivors 3, 4, 5 (square wave phase offset, constant-force rate 0 after 1 s, filter set-up on effect start); `EffectsCalculator::Run()` thread loop and `getInfo`; other uncovered lines of `EffectsCalculator` |
| C errors/storage | `test_error_handler.cpp`, `test_persistent_storage.cpp` | `ErrorHandler`, `PersistentStorage` | Add/dedupe, both `clearError`, `clearTemp`, `sortErrors` order, `Error::toString` per type, callbacks, `ErrorPrinter` enable/disable/output; save/restore over registered handlers, delayed restore vs simulated time, startup callback |
| D filters/small | `test_filters.cpp`, `test_device_interfaces_gaps.cpp`, `test_rtos_wrappers.cpp` | `Filters`, `Encoder`, `MotorDriver`, tier A `cthread`/`cmutex`/`csemaphore` | All biquad types vs reference coefficients, default ctor, `setFc`/`setQ`/`getQ`/`setPeakGain`, DSP on/off agree; `Encoder::getPosAbs`/`setPos`, `MotorDriver::setupDriver`; `Thread::DelayUntil`/`ResetDelayUntil`, mutex standard/recursive lock/unlock, `LockGuard`, `Semaphore::TakeFromISR`, `CountingSemaphore` |
| E transport | `test_uart.cpp`, `test_ringbuffer.cpp`, `test_can_handler.cpp`, `test_usb_hid_handler.cpp`, `test_cdc.cpp` | `UART`, `UartHandler`, `CDCcomm`, `ringbuffer.c`, `RingBufferWrapper`, `CAN`, `CanHandler`, `UsbHidHandler` | UART transmit/receive blocking, DMA and IT, abort, `getErrors`, rx/tx complete dispatch; ring buffer append/get/peek/empty/max size/wrap-around/full/overflow; CAN callbacks dispatched to registered handlers, `isWaiting`, `handleToPort`; `hidGet`/`hidOut`/`setHidDesc`/`getHidDesc`; `CDCcomm` constructor and `connected` |

Not in P1: the skipped "unknown id" case stays skipped (known issue 3). Done when: gate green, at least 85 % line coverage per P1 file, mutation spot-check kills at least 80 %.

## P1 results (2026-10-22/23)

Commit `a79ead3a` (tests and fake changes, before review): 329 new cases. After the review fixes: **332 new cases** in 15 new files (C 62, E 114, others unchanged); no existing test file changed.

| Group | Files | Cases | Own mutation spot-check |
|---|---|---|---|
| A command | `test_cmdparser_gaps.cpp`, `test_command_handler_gaps.cpp`, `test_hid_command_interface_gaps.cpp` | 54 | 38/39 own files, 39/39 full suite |
| B ffb | `test_effects_calculator_gaps.cpp`, `test_hid_ffb_gaps.cpp` | 30 | 23/26 own, 24/26 full (2 equivalent) |
| C errors/storage | `test_error_handler.cpp`, `test_persistent_storage.cpp` | 61 | 28/29 (survivor equivalent on the fake: `xTaskResumeFromISR` vs `vTaskResume`) |
| D filters/small | `test_filters.cpp`, `test_device_interfaces_gaps.cpp`, `test_rtos_wrappers.cpp` | 72 | 55/58 (3 equivalent) |
| E transport | `test_uart.cpp`, `test_ringbuffer.cpp`, `test_can_handler.cpp`, `test_usb_hid_handler.cpp`, `test_cdc.cpp` | 112 | 63/65 (1 equivalent, 1 hidden by FIFO clamping of the fake) |
| **Total** | | **329 (332 after review)** | **207/217 = 95 %** (target 80 %); after review C's survivor at `ErrorHandler.cpp:177` is killed too (fake records the resume API) |

All 6 review survivors (CmdParser:53, :156, EffectsCalculator:235, :1275, HidFFB:423, HidCommandInterface:92) are now killed.

Shared fake changes (additive, needed by the tests):
- `platform/src/host_freertos.cpp`, `support/HostRtos.h`: `HostRtos::findTask(name)`, `isSuspended(task)`, `semaphoreCount()`; `vTaskSuspend(NULL)` suspends the caller and a task that suspends itself now blocks like on the target (needed for `ErrorPrinter::Run`; no other compiled code suspends itself).
- `platform/src/host_platform.cpp`, `support/HostPlatform.h`: UART call counters per HAL function, last timeout/size, `rxData` for blocking `HAL_UART_Receive`, injectable results (`initResult`, `txResult`, `rxArmResult`, `abortResult`); `HAL_UART_Receive_DMA` arms the buffer like `_IT`, `HAL_UART_AbortReceive` disarms it. Defaults keep the old behaviour.
- After review: C added `HostRtos::resumeStats()` (counts `vTaskResume`, `xTaskResumeFromISR`, and `vTaskResume` called in ISR context, which is only recorded); `xTaskResumeFromISR` no longer calls `vTaskResume`. E made the UART fake complete a receive only after the armed size (like the HAL), `AbortReceive` resets the count, and documented the remaining fake limits in `HostPlatform.h`.
- Fixes during integration: two unused helper functions (`-Wunused-function` in `test_can_handler.cpp`, `test_ringbuffer.cpp`). No warnings from the new files with GCC 16 or clang 23.

### Gate after integration (`ffb_unit_tests`, all with the documented exclusions)

| Variant | doctest | ctest | Baseline check |
|---|---|---|---|
| default (GCC 16, DSP on) | 646 passed, 0 failed, 4 skipped | 2/2 | OK, 317 checked, 332 new (0 failed) |
| nodsp | 646 / 0 / 4 | 2/2 | OK |
| san (ASan+UBSan, DSP on; + 2 spline cases excluded) | 643 / 0 / 7 | 2/2 | OK, 315 checked |
| nodsp-san | 645 / 0 / 5 | 2/2 | OK |
| clang23 | 646 / 0 / 4 | 2/2 | OK |
| clang23-san | 643 / 0 / 7 | 2/2 | OK |

Final state after the review fixes. No sanitizer findings. The 2 extra skips (all builds) are new crash/UB-documenting cases (`doctest::skip()`, bugs 17 and 18). Full suite in random order (seeds 5, 77, 31337): 646 passed, 0 failed each. No compiler warnings from the new files (GCC 16, clang 23).

### Coverage after P1 (gcov, GCC 16, DSP on; unchanged after the review fixes)

- Compiled files: **2,640 of 2,687 lines (98.3 %)**, was 79.5 %. +504 lines (plan estimate about 470).
- All in-scope sources: **2,640 of about 8,160 (32.4 %)**, was 26.2 %. Without tier A: 2,531 of about 7,340 (34.5 %).
- Every P1 file is at or above 87.9 %. Lowest: `CDCcomm.cpp` 87.9 % (private constructor/destructor never called), `csemaphore.cpp` 87.9 % (host-only exception paths), `ErrorHandler.cpp` 92.9 % (output branch of `ErrorPrinter::Run` needs `SystemCommands`, P4), `cmutex.cpp` 94.1 %, `ringbufferwrapper.cpp` 95.5 % (unreachable `goto error`), `CanCommandInterface.cpp` 91.6 % (not a P1 target; NEEDS REVIEW #1 lines).
- Remaining uncovered lines are explained in the group reports: deleting-destructor variants of abstract classes, constant private flags (`enableBroadcastFromOtherInterfaces`), dead code after `continue` in `EffectsCalculator::Run` and `EffectsCalculator.cpp:1233` (unreachable `filter[i].reset` in `free_effect`), exception paths that the target compiles out (`CPP_FREERTOS_NO_EXCEPTIONS`).
- Tier C (CMSIS-DSP): unchanged 80.3 % (P9).

## Test gaps found in review (to close in P1)

From the review run (g++ 11.4, scratch copy); not re-run in P0. **All 6 closed in P1** (each mutation re-applied and killed).

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
| 4 | CmdParser | `CmdParser.cpp:53` `freeSpace() > len`; `test_cmdparser_gaps.cpp` | A chunk that would exactly fill the free space is dropped, so one byte is never usable. Strict comparison intended? | no |
| 5 | CmdParser | `CmdParser.cpp:156-180` | `cmd=<int>?<garbage>` is accepted as a plain set of `<int>` instead of being rejected. Intended? | no |
| 6 | CommandHandler | `CommandHandler.cpp:536` | Moved to suspected bug 21 (2026-10-23, review r1) | moved |
| 7 | HID command interface | `HidCommandInterface.cpp:203-213` | A broadcast (instance 0xFF) returns at the first handler without the command, after commands for earlier handlers were queued, and without waking the thread. All or nothing intended? | no |
| 8 | HID command interface | `HidCommandInterface.cpp:162` | `transferComplete` is empty. Intended? | no |
| 9 | EffectsCalculator | `EffectsCalculator.cpp:51, 64, 1211-1216` | Effect monitoring: command not registered, thread never started, broadcast skipped by `continue` ("TODO"). Is the monitoring stream meant to be enabled later? | no |
| 10 | ErrorHandler | `Inc/ErrorHandler.h:29,31` | `systemError = 5` and `cmdNotFound = 5`: alias or numbering mistake? Clearing one by code clears the other | no |
| 11 | ErrorHandler | `ErrorHandler.cpp:97-111` | `clearError(const Error&)` notifies handlers with cleared=true even when the error was not active. Intended? | no |
| 12 | ErrorHandler | `ErrorHandler.cpp:66-75` | `clearTemp()` does not notify handlers (unlike `clearError`/`clearAll`). Intended? | no |
| 13 | ErrorHandler | `clearAll()` | Calls every handler's callback for all 16 slots, passing `Error()` for unused slots. Intended? | no |
| 14 | Filters | `Filters.cpp` peak filter | Uses the earlevel/Zoelzer form (V = 10^(dB/20), boost only in the numerator), not the RBJ cookbook (A = 10^(dB/40)). Gains at fc/DC/Nyquist match, the bandwidth for a given Q differs. Which Q definition is intended? | no |
| 15 | Filters | shelf filters | Ignore Q (fixed 1/sqrt2), Zoelzer design. Intended? | no |
| 16 | Filters | `setFc(0)` (and negative, clipped to 0) | All-zero numerator: the filter outputs 0 for every input. Intended? | no |
| 17 | Filters | Q = 0 | Not validated, NaN coefficients and output. Callers guard (`checkFilterCoeff` maps 0 to 1). Should `Biquad` validate? | no |
| 18 | UART | `UART.cpp:40-44` `reconfigurePort` | Stores the new configuration before `HAL_UART_Init`; after a failed init the same configuration counts as unchanged and is never retried | no |
| 19 | UART | `UART.cpp:154, 168, 187` | One `isTakenFlag` for tx and rx: any rx completion clears it while a tx transfer runs, so `isTaken()`/`UART_CommandInterface::readyToSend()` report a free port while the tx semaphore is taken | no |
| 20 | ringbuffer | `ringbuffer.c:72-88` | `ringBufferAppendOne/Multiple` do not check free space: appending to a full raw buffer loses all data (buffer looks empty). Only reachable through the raw C API; `RingBufferWrapper` clamps first | no |
| 22 | Filters | default `Biquad()` | Leaves type/Fc/Q/peakGain/coefficients uninitialised; setters and `calcBiquad` on it read them; an invalid `BiquadType` leaves the coefficients unset. No test (UB) | no |
| 23 | CANPort | `CAN.cpp:25-29` `handleToPort` | No `break`: two ports with the same handle return the last registered. Test "characterization: with two ports on the same handle the last registered port is returned" | no |
| 24 | RingBufferWrapper | `peek_as`/`get_as` | Only compile for byte types (`static_cast<uint8_t*>(&data)`); `peek_as` returns an uninitialised `T` on failure. Never instantiated in the firmware | no |
| 21 | HidFFB / CDC | `FirmwareFixture::resetGlobals` (test infrastructure) | Does not reset `CDCcomm`'s private `usb_busy_retry`; `test_cdc.cpp` works around it with its own fixture. Shared fix in `support/FirmwareFixture.h`? | no |

New tests that describe unclear behaviour use the `characterization:` name prefix and get a row here. Tests that describe intended behaviour but fail use `KNOWN_ISSUE` and get a row under "Suspected firmware bugs".

## Suspected firmware bugs

| # | Module | Location | Description | Evidence |
|---|---|---|---|---|
| 1 | EffectsCalculator | `FFBoard/Src/EffectsCalculator.cpp:1048` (`effectsForces`, `std::min<uint8_t>(cmd.adr, MAX_AXIS)`), read at line 1053 `effects_statslast[i].current[axis]` | Clamps the axis index to `MAX_AXIS` instead of `MAX_AXIS-1`, then reads `current[MAX_AXIS]` (`std::array<int16_t,2>`) out of bounds. Silent under old libstdc++, aborts under GCC 16 `_GLIBCXX_ASSERTIONS` at -O0 | Existing test `requesting the forces of a not existing axis is rejected` (`cases/test_effects_calculator.cpp:1062`, already `KNOWN_ISSUE_UB`). Message: `/usr/include/c++/16/array:210: ... std::array<short int, 2>::operator[] ... Assertion '__n < this->size()' failed.` -> SIGABRT after 192 of 317 cases. Excluded with `--test-case-exclude` |
| 2 | CMSIS-DSP (vendor, Tier C) reached from EffectsCalculator | `Libraries/CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_f32.c:223`, called from `EffectsCalculator.cpp:695` with `blockSize = 1` | `while( *pXq <= x[i+1] && blkCnt > 0 )` dereferences `pXq` before checking `blkCnt`, so it reads one float past the 1-element input (`interp_time`, a stack variable). Bug is in the vendor library; the firmware call is valid per the API. On target it reads one adjacent stack word; the result is not used, so likely harmless | ASan stack-buffer-overflow (GCC and clang), test cases `reconstruction of a constant signal is constant` and `reconstruction of a step never leaves the range of the input values and reaches the target`. Not visible without ASan, not in DSP-OFF build. Possible firmware-side workaround (not to be applied by me): pass a 2-element buffer |
| 3 | CommandHandler | `FFBoard/Src/CommandHandler.cpp:136` `snprintf(cmdhex,11,"0x%lX",cmd.cmdId)` | `%lX` with `uint32_t`: correct on ARM (where `uint32_t` is `unsigned long`), mismatched type on the 64-bit host. Portability issue rather than a target bug (`PRIX32` would fit both) | GCC `-Wformat` and clang `-Wformat` warnings |
| 4 | EffectsCalculator | see NEEDS REVIEW #2 | Possible precision/wrap problem of the reconstruction filter after long uptime | Code reading only; to be demonstrated by a test in Batch 2 |
| 5 | Command execution | `executeCommands` (find with `grep -rn executeCommands Firmware/FFBoard/Src`) | Dereferences the command definition before checking it for nullptr. A command with an unknown id terminates the run | **Confirmed in the source (P0):** `FFBoard/Src/FFBoardMainCommandThread.cpp:99-102`: `cmdDef = handler->getCommandFromId(cmd.cmdId)`, then `validFlags = cmdDef != nullptr`, then `if(cmdDef->flags & ...)` dereferences it unconditionally. Existing test `command with an unknown id is rejected` (`cases/test_command_execution.cpp:186`) stays `skip()`ped (known issue 3) |
| 7 | ErrorHandler | `ErrorHandler.cpp:130-143` (`getErrors` + `sortErrors`) | Empty slots (code 0) sort to the front; `getErrors()` stops at the first empty slot, so after any `clearError`/`clearTemp` it returns an empty list while errors are still stored. Likely visible via the "errors" command | KNOWN_ISSUE "the other errors stay listed after one error is cleared" (`test_error_handler.cpp`). Confirmed in source |
| 8 | ErrorHandler | `ErrorHandler.cpp:118-124` `clearError(ErrorCode)` | The slot is emptied before the callbacks, which receive `errors[i]` = an empty Error instead of the cleared one | KNOWN_ISSUE "clearing by code tells the handlers which error was cleared". Confirmed in source |
| 9 | ErrorHandler | `ErrorHandler.cpp:66-72` `clearTemp` | `break` after the first temporary error: only one is cleared per call; comment says "Clears errors marked as temporary" | KNOWN_ISSUE "clearTemp removes all temporary errors". Confirmed in source |
| 10 | EffectsCalculator | `EffectsCalculator.cpp:235` (square wave) | Raw phase (0.01 deg) added as milliseconds instead of `phase*period/36000` like sine/triangle; 90 deg on a 1000 ms period shifts by 984 ms instead of 250 ms | KNOWN_ISSUE "square phase is a fraction of the period like for the other periodic effects". Confirmed in source |
| 11 | HidFFB | `HidFFB.cpp:301-305` `set_effect` | `set_filters` runs before `effect_p->type` is updated: a type change sets up the filters of the OLD type | KNOWN_ISSUE "set effect with another effect type sets up the filters of the new type". Confirmed in source |
| 12 | HidFFB / EffectsCalculator | `HidFFB.cpp:301-305` + `:423`, crash at `EffectsCalculator.cpp:416/425/433` | **Crash:** a running effect without filters (e.g. sine) changed by Set Effect to damper/friction/inertia keeps null filters (423 skips `set_filters` for running effects); the next force calculation dereferences nullptr (UB: SIGSEGV on the host; on the target `Biquad::process` is non-virtual and address 0 aliases boot flash, so it may output garbage torque or corrupt memory instead of faulting) | KNOWN_ISSUE "a running effect changed to a damper by set effect has damper filters" (stops at `REQUIRE(filter[0] != nullptr)`); crash confirmed by the author with a temporary probe |
| 13 | CommandHandler | `CommandHandler.cpp:533-561` | The override placeholder from `overrideCommandFlags_INT` lacks `CMDFLAG_EXTOVERRIDE`, so `registerCommand_INT` treats the id as "already present": the real command is never registered (placeholder keeps `cmd == nullptr`; `getCommandFromName` would compare nullptr with a string = UB) | KNOWN_ISSUE "a command registered after a flag override gets its name and the masked flags". Confirmed in source |
| 14 | Filters | `Filters.cpp:44/86` + `:102`, reachable via `EffectsCalculator.cpp:868-872` | Clip allows Fc = 0.5 (comment: "Must be lower than 0.5"); with CMSIS-DSP `arm_cos_f32(PI/2)` is exactly 0, K = inf, NaN coefficients, NaN output. DSP off: finite but a double pole on the unit circle. `checkFilterCoeff` maps a stored freq of 0 to Fc = 0.5 | KNOWN_ISSUE in the DSP build only "lowpass at the highest allowed frequency 0.5 ...". Target result may differ (FMA) |
| 15 | Filters (latent) | `Inc/Filters.h`, `Filters.cpp:22/30` | In the DSP build the copied `arm_biquad_casd_df1_inst_f32` still points into the original's coefficients/state: a copy filters with the original's data and corrupts its state. No current caller copies a Biquad | KNOWN_ISSUE in the DSP build only "a copied filter works independently of the original" |
| 16 | UART | `UART.cpp:62-106` (`transmit_IT/_DMA`, `receive_IT/_DMA`) | The port is taken before the HAL call; on a HAL error no completion releases it. `UART_CommandInterface::Run` then waits forever in `while(uartport->isTaken()) Delay(1)` | 4 KNOWN_ISSUE tests in `test_uart.cpp` |
| 17 | UART | `UART.cpp:212-222` | `~UARTDevice` calls `uartport->freePort(this)` unconditionally; a default-constructed device has `uartport == nullptr` (crash). No current firmware class uses the default constructor | Skipped KNOWN ISSUE (crash) case "a device created without a port can be destroyed" |
| 18 | RingBufferWrapper | `ringbufferwrapper.cpp:29-34` (early return in the size constructor), `45-50` (destructor) | Size constructor returns before initialising `buffer` for a non-power-of-two size: `capacity()`/`freeSpace()` read garbage, destructor deletes an uninitialised pointer. Unreachable today (CmdParser uses 512/1024/4096) | Skipped KNOWN ISSUE cases only (review: the former KNOWN_ISSUE_UB read indeterminate memory in the test itself and could flip; made a `skip()` case) |
| 19 | UsbHidHandler | `UsbHidHandler.cpp:19-21` | Destructor does not clear `globalHidHandler`; TinyUSB callbacks (`global_callbacks.cpp:317, 332, 344`) would use a dangling pointer. Same root cause as the existing HidFFB case (`test_hid_ffb.cpp:51`): one bug | KNOWN_ISSUE "a registered handler unregisters when destroyed" |
| 20 | CDCcomm | `CDCcomm.cpp:69` | `std::min<uint16_t>(reply->length(), bufferRemaining)` truncates the length to 16 bit. Review r3: after a partial write the remainder can be exactly 65536 (any multiple of 65536), which truncates to 0; the retry writes 0 bytes without flush, no tx-complete follows, `cdcSems[0]` is never given back and every later `cdcSend` blocks: **CDC output dead** | KNOWN_ISSUE "a reply longer than 65535 bytes is sent in FIFO-sized parts" |
| 23 | Axis | `Axis.h:504-507`, `Axis.cpp:221-262, 498-501, 733-735` | An axis restored from empty flash reports power 5000 / effect ratio 204, but `torqueScaler` and `effectRatioScaler` stay 0 (only computed in `setPower`/`setEffectRatio`, which `restoreFlash` calls only when flash has a value): **no torque until power/ratio are set by a command or flash** | KNOWN_ISSUE "an axis restored from empty flash applies its default power of 5000 and effect ratio of 204" (`cases_target/test_target_setup.cpp`); first failing check is the driver torque. Confirmed in source |
| 24 | FFBHIDMain | `Inc/ffb_defs.h:135` (`HID_GamepadReport_base` without virtual destructor), deleted via `std::unique_ptr` in `~FFBHIDMain` (`FFBHIDMain.cpp:64`) | Deleting any FFBHIDMain main class (e.g. on main class change) is UB: ASan new-delete-type-mismatch | KNOWN_ISSUE on the type trait (no UB executed); tests that delete a main class are marked `DELETES_FFBHIDMAIN` and skipped in sanitizer builds (decision pending) |
| 25 | Axis | `Axis.cpp` | No `#include <cmath>`: compiles only with CMSIS-DSP (`-include cmath` added for `ffb_target_tests` with DSP off, F14) | Build |
| 26 | Axis | `Axis.cpp:1415` `clip<int8_t>(gain,-120,120)` | Narrowing inside `clip()` (`cppmain.h`): ill-formed C++, GCC warns, clang rejects (`-Wno-c++11-narrowing` for clang, `Axis.cpp` only, `ffb_target_tests` only). No runtime effect | Build |
| V1 | FreeRTOS wrappers (**vendor, tier A**) | `csemaphore.cpp:117-131` | `CountingSemaphore(0, x)` or initial > max throws before `handle` is set; `~Semaphore` then deletes an uninitialised handle (UB). Exception builds only (target uses `CPP_FREERTOS_NO_EXCEPTIONS`/`configASSERT`) | Seen with mutants; no test (would crash) |
| V2 | FreeRTOS wrappers (**vendor, tier A**) | `cthread.cpp:184-187` | `~Thread` of a never-started thread calls `vTaskDelete` with an uninitialised handle (NULL deletes the calling task on FreeRTOS). Also on target | Code reading; no test (UB) |
| V3 | FreeRTOS wrappers (**vendor, tier A**) | `cmutex.cpp`, `csemaphore.cpp` | On a kernel create failure (NULL), the destructors call `vSemaphoreDelete(NULL)` (real FreeRTOS asserts) | Code reading; no test |
| 21 | CommandHandler | `CommandHandler.cpp:536` `flags &= flags \| CMDFLAG_EXTOVERRIDE` | No-op although the comment says "honor flag mask": (re-)registering an overridden command drops the override mask (moved from NEEDS REVIEW #6 after review r1) | KNOWN_ISSUE "registering an overridden command again keeps the override mask" (`test_command_handler_gaps.cpp`) |
| 22 | UART | `UART.cpp:97, 104, 116-123`, used at `189-191` | `abortReceive()` and a failed `receive_IT/_DMA` never clear `rxbuf_t`: the next single-byte completion armed by `registerInterrupt()` passes the old (possibly dangling) user buffer to `uartRcv` instead of the received byte (found by review r3) | 2 KNOWN_ISSUE tests in `test_uart.cpp` (after abort, after a failed receive) |
| V4 | FreeRTOS wrappers (**vendor, tier A**) | `cthread.cpp` `~Thread` after `TaskFunctionAdapter` | When `Run()` returns, the adapter deletes the task; the Thread destructor then calls `vTaskDelete` again on the freed handle (double delete / use after free on target). The fake ignores it (found by review r2) | Comment in `test_rtos_wrappers.cpp`; no test (UB) |
| 6 | Test setup (not firmware) | `CMakePresets.json` `host-sanitize`, `CMakeLists.txt` | With the CMSIS-DSP submodule checked out, `ctest` in the sanitizer preset fails on the spline overflow (bug 2). Reproduced with the same flags by script, preset not run | Fixed by F1 (`8861c911`) + F1b (P0). Verified in P0: `cmake --preset host-sanitize -B <out-of-tree>`, `ctest` 2/2 passed |

## Untestable code and refactor suggestions

| Module | Location | Why untestable | Suggested refactor |
|---|---|---|---|
| CDCcomm | `CDCcomm.cpp:19-25` | Private constructor/destructor, never called | none needed (static class) |
| ErrorPrinter | `ErrorHandler.cpp:162-167` | Output branch needs a real `SystemCommands` (not compiled in `ffb_unit_tests`) | Covered in P4 (`ffb_target_tests`) |
| CAN/UART dispatch | `global_callbacks.cpp` | Not compiled in `ffb_unit_tests`; P1 tests mirror the loop in a local helper | Real file in P4 |
| RingBufferWrapper | copy constructor | Owns raw memory with an implicit copy (rule of three broken); works only by guaranteed elision (`CmdParser.cpp:20`, `CommandInterface.h:48`) | Delete copy, add move |
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
| Filters | Host CMSIS-DSP sin/cos are the portable C versions without FMA; the Fc = 0.5 KNOWN_ISSUE depends on `arm_cos_f32` returning exactly 0 | `test_filters.cpp` (tolerances: relative 2e-4) |
| RTOS wrappers | Exception paths (`throw` on create failure) exist only on the host; the target defines `CPP_FREERTOS_NO_EXCEPTIONS` | `test_rtos_wrappers.cpp` (paths not tested) |
| CDC command interface | `CommandResult::size()` uses `sizeof` (contains `std::string`), so buffer thresholds differ host/target | `test_command_handler_gaps.cpp` CDC cases rely only on "at least the buffer size" |
| ringbuffer | `size_t` 64-bit host / 32-bit target with `uint32_t` temporaries; same results for tested sizes | `test_ringbuffer.cpp` |

## Review outcomes (Phase 3)

| Module | Reviewer | Findings | Fixed? |
|---|---|---|---|
| P1 A command (`test_cmdparser_gaps`, `test_command_handler_gaps`, `test_hid_command_interface_gaps`) | r1 (fresh agent, 2026-10-22) | must: placeholder test asserted the exact flags of bug 13 unflagged; should: bug 13 test also failed for the line 536 defect (now bug 21, own KNOWN_ISSUE), missing NEEDS REVIEW comment; nits: comments. All 4 KNOWN_ISSUE claims confirmed, first failing assertion correct; random order and single runs green | yes (2026-10-23) |
| P1 B ffb (`test_effects_calculator_gaps`, `test_hid_ffb_gaps`) | r1 | should: "HardFault on target" wording (bug 12), the 180 deg square test passes by coincidence (rename + comment as mutation guard); nits: weak sign check, comments on dead monitoring code | yes (2026-10-23) |
| P1 C errors/storage | r2 | must: 3 tests asserted slot positions that exist only because of bug 7 (a plausible fix broke them); should: ISR vs thread resume not distinguishable (fake extension); nit: raw slot write. KNOWN_ISSUEs confirmed; vTaskSuspend fake change realistic; no static leakage | yes (2026-10-23) |
| P1 D filters/small | r2 | should: coefficient tolerance effectively absolute (small coefficients checked to ~20 %), vendor double delete in `~Thread` (V4); nits: comments on IEEE divide by zero, default Biquad UB note, fake-only assertion. Filters KNOWN_ISSUEs confirmed (arm_cos_f32(PI/2) == 0 exactly on host) | yes (2026-10-23) |
| P1 E transport | r3 | must: KNOWN_ISSUE_UB ringbuffer test read indeterminate memory (could flip in Release); should: wrong lines (bug 18), CDC bug 20 understated (CDC dead), new bug 22, UART fake completed receives after 1 byte regardless of size, a test that only tested `MockCANPort`, CAN dispatch helper scope wording, CDC reset relied on fake semaphore, missing CDC blocking test; nits | yes (2026-10-23) |

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

1. ~~Coverage goal~~: answered 2026-10-23, 85 % or more, hardware-only code ignored (see decisions log). Original question: Is about 70% of in-scope firmware lines the right target? Going higher means testing interrupt and hardware-only code through fakes, which costs more per line.
2. ~~Pace~~: answered 2026-10-23, stop at the end of every phase. Original question: Stop at the end of every phase (default), or run several phases in a row with a report at each? Say "continue through P<n>" to allow it.
3. **USB HID PID 1.01 specification** (USB Device Class Definition for Physical Interface Devices, optional): give the path of the spec file to have the `HidFFB` characterization tests verified against it. Without it that batch is skipped.
4. ~~Commit author name~~: answered 2026-10-22, `MadBull.Ben` for all commits.
5. **CI:** no GitHub Actions workflow runs the host tests (P0 finding). Should I describe a workflow job (configure, build, `ctest` with DSP on/off and sanitizers, MinGW) for you to add? I will not touch `.github/`.
6. **Second checkout:** `Testing2/OpenFFBoard` (branch `feature/host-unit-tests`) holds untracked copies of the v2 files and none of the add-test-suite work. Work continues only in `testing/OpenFFBoard` unless you say otherwise.

## Resume instructions

Start a new Claude Code session from the repo root and say:

> Read CLAUDE.md, Firmware/Tests/README.md, Tests/TEST_PLAN.md and Tests/PROGRESS.md. Resume from "Next action". Do not redo completed work, and do not move past an approval gate without my go-ahead.
