# Test Suite Progress

> This file is the resume point. Read "Current state" and "Next action" first.
> Claude updates it after every module, phase gate, decision and finding.
> Plan: `Tests/TEST_PLAN.md`. Rules: root `CLAUDE.md`. Suite docs: `Firmware/Tests/README.md`.

## Current state

- **Current phase:** P6 (encoders and buses in `ffb_target_tests`), started 2026-10-26. P5 approved 2026-10-26.
- **Phase status:** batch 0 done ("P6 batch 0"); 3 module agents running.
- **Branch:** add-test-suite (base commit `a5418ff5`, "Initial unit test system")
- **Last commit:** `13cb14a8` (P5 final).
- **Last updated:** 2026-10-26
- **Last session summary:** P5: fake HAL SPI (F4) and I2C (F5), bxCAN mask-filter model in `HostTarget::canReceive`, input mocks; 2 module agents (buttons: 179 cases; analog: 214 cases), 2 fresh reviewers (p5r1 buttons, p5r2 analog + infrastructure), fixes. `ffb_target_tests` 1,078 cases. P5 files 96.2-100 %; all in-scope sources **6,206 of about 8,108 (76.5 %)**, was 62.6 %. New bugs 53-75, NEEDS REVIEW 76-92. /tmp was cleaned again; work was merged early, nothing lost.

## Next action

**P6 batch 0 done.** Then: 3 module agents (see "P6 plan"), fresh reviewers, fixes, gate, coverage (85 % per file without hardware-only code), stop for approval.

## Phase checklist

| Phase | Executable | Est. lines gained | Status | Approved by me? | Commit |
|---|---|---|---|---|---|
| Phase 1 (explore and plan) | n/a | n/a | complete | yes, 2026-10-21 | |
| Review of the existing suite | n/a | n/a | complete | | |
| Batch 0 (plan v1) items done before plan v2 | n/a | n/a | F1 done, F3 header approved and committed | yes, 2026-10-21 | `8861c911` (F1), `ef4101cf` (F3) |
| P0 Safety net | `ffb_unit_tests` | 0 (measured baseline 2,136 lines, 26.2 %) | done | yes, 2026-10-22 | `2461c51e` |
| P1 Gaps in compiled files | `ffb_unit_tests` | about 470 (measured +504: 2,640 lines, 32.4 %) | done | yes, 2026-10-23 | `a79ead3a` (tests), `7a35d2f1` (review fixes) |
| P2 Axis and main classes | `ffb_target_tests` (new) | about 790 (measured: total 4,164 lines, 50.5 %) | done | yes, 2026-10-23 | `4aec799c` batch 0, `760f6f1a` tests, `76d89b40` infra review fixes, `41112d85` review fixes |
| P3 Flash and persistence | `ffb_flash_tests` (new) | about 365 (measured: total 4,480 lines, 55.3 %) | done | yes, 2026-10-24 | `bfe6ed56` batch 0, `ac030b2c` B tests, `b93f230e` A tests, `6410fb07` macOS skip, `8b178bbf` A review fixes, `52dce9ae` B review fixes |
| P4 System, callbacks, misc | `ffb_target_tests` | about 460 (measured: total 5,078 lines, 62.6 %) | done | yes, 2026-10-25 | `0644984e` batch 0, `00a1e00b`/`e7d467d4`/`61a20f9f` tests, `90554aaa` infra review fixes, `c00b7e2b` B fixes, `e51bc104` A fixes, `8e807f05` C fixes + results |
| P5 Buttons and analog | `ffb_target_tests` | about 775 (measured: total 6,206 lines, 76.5 %) | done | yes, 2026-10-26 | `73d4e482` batch 0, `3e36fdfd` analog, `663bce67` buttons, `2a18521a` A + infra review fixes, `13cb14a8` B review fixes + results |
| P6 Encoders and buses | `ffb_target_tests` | about 640 (uncovered now: about 785 in the P6 files) | in progress (modules) | | |
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
| 2026-10-23 | P2 approved; start P3 | My go-ahead |
| 2026-10-25 | P4 approved; start P5 | My go-ahead |
| 2026-10-24 | P3 approved ("Continue"); start P4 | My go-ahead |
| 2026-10-24 | `ffb_flash_tests` is not built on macOS (`if(APPLE)` in CMake, message at configure time): the 4 GiB `__PAGEZERO` covers the fixed flash addresses | My answer to open question 7 |
| 2026-10-23 | Bug 24 (main class deletion UB): tests that delete an FFBHIDMain main class are marked `DELETES_FFBHIDMAIN` and skipped in sanitizer builds; ASan stays fully on (no `ASAN_OPTIONS` suppression) | My decision |
| 2026-10-23 | Keep `-funsigned-char` for `ffb_target_tests` only (matches the ARM target; host-vs-target caveat for `ffb_unit_tests`) | My decision |
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
| 2026-10-26 | P5 review: `HostTarget::canReceive` follows the real bxCAN mask filters, but the baseline `MockCANPort::passesFilter` keeps its stricter behaviour (a baseline case asserts it); documented in both. The new CanAnalog filter question became NEEDS REVIEW 84 (free number; group B's 85-92 were used up) | Keep the baseline unchanged; fake fidelity for new tests |

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
- `ctest`: `ctest --test-dir ../ffb-test-build/default --output-on-failure`. Runs 3 tests: `ffb_unit_tests` with the exclusions (F1/F1b, cache lists `FFB_TESTS_EXCLUDE` and `FFB_TESTS_EXCLUDE_SANITIZE`, printed at configure time: `OpenFFBoard tests: excluded from ctest: ...`), `ffb_unit_tests_baseline` (the baseline check) and `ffb_target_tests` (P2, cases in `cases_target/`, own exclusion lists `FFB_TARGET_TESTS_EXCLUDE` and (sanitizer builds only) `FFB_TARGET_TESTS_EXCLUDE_SANITIZE`, both default empty, printed as `OpenFFBoard tests: ffb_target_tests excluded from ctest: ...`)
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
| F4 | Fake HAL SPI (records TX, scripted RX, DMA/IT completion callbacks driven by the test) | `platform/src/host_target_bus.cpp` | SPIButtons, EncoderSSI, EncoderBissC, MtEncoderSPI, SPI.cpp | **done** P5 batch 0 |
| F5 | Fake HAL I2C (scripted device register map per address, error injection) | `platform/src/host_target_bus.cpp` | PCF8574, ADS111X, I2C.cpp | **done** P5 batch 0 |
| F6 | Fake HAL ADC (settable DMA buffer, conversion-complete callback) + LL ADC voltage/temperature macros | `platform/src/host_target_system.cpp`, `platform/target_f407/main.h` | LocalAnalog, AdcHandler, voltagesense | **done** P4 batch 0 |
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

## P2 plan (2026-10-23)

Executable `ffb_target_tests`, new files in `cases_target/` only. Coverage after batch 0 (smoke tests): `Axis.cpp` 294/677 (43 %), `AxesManager.cpp` 41/62 (66 %), `FFBHIDMain.cpp` 66/173 (38 %), `FFBHIDMain_commmands.cpp` 13/76 (17 %), `FFBWheel.cpp` 6/16, `FFBoardMain.cpp` 6/31, `FFBJoystick.cpp`, `FFBHIDExt.cpp`, `CustomMain.cpp` 0 %. 4 parallel agents in isolated scratch workspaces (`scratchpad/p2/<group>`), same procedure as P1; per-file goal 85 % without hardware-only code; mutation spot-check 80 % (about 20 mutants for `Axis.cpp`).

| Group | Owns (new files) | Firmware | Behaviours |
|---|---|---|---|
| A axis torque | `test_axis_torque.cpp` | `Axis.cpp` (torque path) | power scaling and clipping to +-0x7fff, invert, range/degrees scaling, endstop torque (gain, beyond range), speed limiter, torque slew-rate limiter (axis and driver), idle spring, damper/friction/inertia strengths, expo curve, gear ratio, effect ratio, fade-in, emergency stop and reset, hands-off/suspend, position/speed/accel metrics |
| B axis config | `test_axis_config.cpp` | `Axis.cpp` (config, commands, flash) | `encodeConfToInt`/`decodeConfFromInt` round trip, save/restore flash (incl. empty flash, bug 23), every axis command get/set/info incl. limits and invalid values, driver/encoder selection via chooser and commands, zeroing/position commands, error handling |
| C axes + integration | `test_axes_manager.cpp`, `test_ffb_integration.cpp` | `AxesManager.cpp`, path through `HidFFB`/`EffectsCalculator`/`Axis` | axis count changes, update/updateTorque, emergency stop over all axes, usb suspend/resume, commands; end to end: PID reports (constant, spring, damper, periodic, conditions per axis) -> effects -> axis -> `MockMotorDriver` torque with the position from `MockEncoder` |
| D main classes | `test_ffbhidmain.cpp`, `test_main_classes.cpp` | `FFBHIDMain.cpp`, `FFBHIDMain_commmands.cpp`, `FFBWheel.cpp`, `FFBJoystick.cpp`, `FFBHIDExt.cpp`, `CustomMain.cpp`, `FFBoardMain.cpp`, `mainclass_chooser.cpp` | HID gamepad report packing (axes, buttons, double buffer), FFB on/off, update rate/timer, usb init/suspend/resume, button/analog source selection commands, main class info/ids unique and selectable, CustomMain |

## P6 plan (2026-10-26)

`ffb_target_tests`, new files in `cases_target/` only. Coverage before P6 (from the P5 run): `CRC` 0/12, `EncoderLocal` 34/84, `EncoderSSI` 2/155, `EncoderBissC` 2/166, `MtEncoderSPI` 0/244, `SPI` 108/212, `SpiHandler` 12/20, `I2C` 136/176, `I2CHandler` 6/12, `TimerHandler` 6/8, `ExtiHandler` 6/8: about 785 uncovered lines. At 95 % each the total reaches about 85.7 %. Batch 0 first (timer encoder registers, encoder SPI port, mock SPI/I2C devices, smoke tests), then 3 module agents in isolated scratch workspaces (`scratchpad/p6/<group>`), per-file goal 85 % without hardware-only code, mutation spot-check 80 %.

| Group | Owns (new files) | Firmware | Behaviours |
|---|---|---|---|
| A local and SSI encoders | `test_encoder_local.cpp`, `test_encoder_ssi.cpp`, `test_timer_exti_handlers.cpp` | `EncoderLocal.cpp`, `EncoderSSI.cpp`, `TimerHandler.cpp`, `ExtiHandler.cpp` | quadrature count and overflow, index pulse and centering, cpr, commands, flash config; SSI frame decoding, resolution, speed presets, errors; handler registration/removal and dispatch |
| B BiSS-C and MT encoders | `test_encoder_bissc.cpp`, `test_mtencoder_spi.cpp`, `test_crc.cpp` | `EncoderBissC.cpp`, `MtEncoderSPI.cpp`, `CRC.cpp` | BiSS-C frame, CRC6, error and warning bits, multiturn, timeouts; MT6825/6835 parity/CRC, status, position and multiturn, commands; CRC8/CRC16 against known check values |
| C bus wrappers | `test_spi_port.cpp`, `test_i2c_port.cpp` | `SPI.cpp`, `SpiHandler.cpp`, `I2C.cpp`, `I2CHandler.cpp` | port/device arbitration, CS pin handling, exclusive mode, prescaler selection, blocking/IT/DMA paths and error returns, callback dispatch; bugs 53, 57, 58 |

## P6 batch 0 (2026-10-26)

- **Resumable tasks** in the fake kernel (`host_freertos.cpp`, `HostRtos::makeResumable(handle|name)`, opt-in): `EncoderBissC` and `MtEncoderSPI` wait twice per loop (`requestSem.Take()`, DMA, `WaitForNotification()`), which the restart-from-`Run()` model cannot test. A resumable task runs on its own `std::thread` with a strict mutex/condition-variable hand-over (one thread runs at a time, deterministic); blocking calls with `portMAX_DELAY` park until the condition holds, finite timeouts park once then time out, `vTaskDelete` of a parked task unwinds and joins. Without `makeResumable` nothing changes. **Portability caveat:** needs `std::thread` (MinGW-w64 with the posix thread model, or win32 threads with GCC 13+); unverified on Windows.
- Encoder-mode timer fake: `HostTarget::encoderTimerCount(htim, counts)` (counts only with CEN, UIF at ARR->0 / 0->ARR, update branch of `HAL_TIM_IRQHandler` in ISR context), `timerCounter`/`timerSetCounter`, `encoderIndexPulse()` (EXTI falling edge per `MX_GPIO_Init`). `HostTarget::spiFrameBytes` (16-64 bit frames, MSB first). Unmodelled: DIR bit, input filter/polarity, CNT above ARR (recorded), UIF pending when UIE is enabled later.
- Reset of the static `inUse` flags of the 4 encoders; `AxisFixture::selectFirmwareEncoder(enctype, i)`; `mocks/MockSPIDevice.h`, `mocks/MockI2CDevice.h`. No new HAL functions needed; no CMake change.
- 14 smoke cases (`EncoderSetup`). Gate: ctest 4/4 and baseline OK in all 6 dirs, `ffb_unit_tests`/`ffb_flash_tests` compile commands identical, `ffb_target_tests` 1,084/0/8 (default, clang23), 1,085/0/8 (nodsp), 979 + 113 (san, clang23-san), 980 + 113 (nodsp-san); no findings; 30 sanitizer runs and random orders green. Verified by the main agent.
- Suspected bugs from code reading, handed to the module agents (to be confirmed by tests): `EncoderSSI.cpp:124` raw decode, `EncoderSSI.h:73` uninitialised `mode`, `EncoderSSI.cpp:110` AMT23 check-bit test, `EncoderLocal.cpp:85` `% cpr` with cpr 0, `MtEncoderSPI.cpp:113-114` / `301-302` clip-to-size, `MtEncoderSPI.cpp:335` unchecked mode, `EncoderBissC.cpp:252-264` `__builtin_clzll(0)`, unchecked `bits`, `hasFreePins` in BissC/SSI (bug 67 pattern), MtEncoderSPI deleted mid-DMA keeps the port semaphore.

## P5 results (2026-10-26)

**Final state after the review fixes:** `ffb_target_tests` 1,078 cases: 1,070 passed / 0 failed / 8 skipped (default, clang23), 1,071/0/8 (nodsp), 965 passed + 113 skipped (san, clang23-san), 966 + 113 (nodsp-san); ctest 4/4 in all 6 dirs; baseline OK; `ffb_unit_tests` 646/0/4 (643/0/7 DSP san, 645/0/5 nodsp-san) and `ffb_flash_tests` 158/0/0 unchanged; no sanitizer findings; no warnings located in `Firmware/Tests` files (GCC 16, clang 23); 3 random orders (seeds 11, 22, 33) green. Verified by the main agent.

Coverage (gcov, GCC 16, DSP on, `cov`): `LocalButtons` 99/99, `SPIButtons` 178/179, `ShifterAnalog` 175/182 (96.2 %, dead `setCSPin`, bug 71), `CanButtons` 77/77, `PCF8574` 107/107, `SelectableInputs` 67/67, `ButtonSource` 6/6, `AnalogAxisProcessing` 110/112 (dead free function), `LocalAnalog` 49/49, `CanAnalog` 70/70, `ADS111X` 206/206, `AnalogSource` 6/6, `AdcHandler` 14/14. **All in-scope sources 6,206 of about 8,108 (76.5 %)**, was 62.6 % after P4; about 690 lines from the 85 % goal (P6 encoders/buses about 640, P7 CAN/serial FFB about 420 still open). Hardware-only exclusions: none.

- **Review fixes:**
  - Shared infrastructure (p5r2): SPI/I2C fakes read the tx buffer at completion like the HAL ISR (exposes bug 66: the ADS111X threshold test is now a KNOWN_ISSUE); `canReceive` models the bxCAN 32-bit identifier mask filters (RM0090 32.7.4) built like `CANPort2B.cpp:272-287` (a standard filter also passes extended frames whose EXID[28:18] match); `MockCANPort::passesFilter` deliberately unchanged (baseline case `cases/test_can_command_interface.cpp:123` depends on it, documented); `static_assert` for the `SelectableInputs` cast in `InputSources.h`; stale stub comments.
  - A (p5r1): bug 69 test independent of the fix strategy, PCF8574 receive-completion guard tests (all 3 guard mutants killed), all `SPIButtons.cpp` citations corrected (were 111 lines off), NR 79 became bug 75 (KNOWN_ISSUE + a passing preset case), bug 71 check tightened, G27 sequential case, NR 83 characterizations. Fix experiments flip only their own cases.
  - B (p5r2): NR 91 test proves no transfer is left, NR 86 margin lock-ins made margin-independent or marked characterization, destructor flag in `test_analog_source.cpp`, CanAnalog filter characterization (NR 84) plus the even-id companion, I2C speed preset 1 -> 400000, FMA notes, includes.

### Build step


- **Group B** (analog) merged early: `test_analog_axis_processing.cpp` (73, 2 skipped crash cases), `test_local_analog.cpp` (32), `test_can_analog.cpp` (35), `test_ads111x.cpp` (62, 1 skipped crash case), `test_analog_source.cpp` (10) = 212 cases. No shared-helper change. Coverage: `AnalogAxisProcessing.cpp` 40/112 -> 110/112 (98.2 %; 58/60 an empty, undeclared free function `registerCommands(CommandHandler*)`: dead code), `LocalAnalog.cpp` 49/49, `CanAnalog.cpp` 70/70, `ADS111X.cpp` 206/206, `AnalogSource.cpp` 6/6. Filter checked against an independent double biquad in both DSP builds. Mutation 52/54 (2 equivalent). Bugs 59-66, NEEDS REVIEW 85-92. Gate after merge: `ffb_target_tests` 892/0/5 (default, clang23), 893/0/5 (nodsp), 789 + 108 skipped (san, clang23-san), 790 + 108 (nodsp-san); ctest 4/4; baseline OK.
- **Group A** (buttons): `test_local_buttons.cpp` (21), `test_spi_buttons.cpp` (42), `test_shifter_analog.cpp` (29), `test_can_buttons.cpp` (22), `test_pcf8574.cpp` (29), `test_selectable_inputs.cpp` (30) = 173 cases. No shared-helper change. Coverage: `LocalButtons.cpp` 99/99, `SPIButtons.cpp` 178/179 (base-class deleting destructor variant), `ShifterAnalog.cpp` 0 -> 175/182 (96.2 %; `setCSPin` 204-213 is dead code, see bug 71), `CanButtons.cpp` 77/77, `PCF8574.cpp` 107/107, `SelectableInputs.cpp` 67/67, `ButtonSource.cpp` 6/6. Mutation 75/75. Known bugs: 54 (KNOWN_ISSUE), 55 (KNOWN_ISSUE_UB with a derived class: in `SPI_Buttons_1/2` the overflow lands in tail padding, invisible to ASan, results stay correct), 56 (2 skipped crash cases). New bugs 67-74, NEEDS REVIEW 77-82.
- **Merged gate (A+B):** `ffb_target_tests` 1,070 cases: 1,062 passed / 0 failed / 8 skipped (default, clang23), 1,063/0/8 (nodsp), 957 + 113 skipped (san, clang23-san), 958 + 113 (nodsp-san); ctest 4/4; baseline OK; `ffb_unit_tests`/`ffb_flash_tests` unchanged; no sanitizer findings; 3 random orders green; equal test names only in different suites.

## P5 plan (2026-10-26)

`ffb_target_tests`, new files in `cases_target/` only. 2 module agents in isolated scratch workspaces (`scratchpad/p5/<group>`), per-file goal 85 % without hardware-only code, mutation spot-check 80 %.

| Group | Owns (new files) | Firmware | Behaviours |
|---|---|---|---|
| A buttons | `test_local_buttons.cpp`, `test_spi_buttons.cpp`, `test_shifter_analog.cpp`, `test_can_buttons.cpp`, `test_pcf8574.cpp`, `test_selectable_inputs.cpp` | `LocalButtons.cpp`, `SPIButtons.cpp`, `ShifterAnalog.cpp`, `CanButtons.cpp`, `PCF8574.cpp`, `SelectableInputs.cpp`, `ButtonSource.cpp`, `ButtonSources.cpp` | pin masks and polarity, SPI shift register modes (bit order, CS polarity, chain length, TM mode, 74xx165), shifter H-pattern and sequential decoding incl. reverse button and thresholds, CAN frame decoding (bug 54), PCF8574 chains, source selection/save/restore, commands of every class, flash config |
| B analog | `test_analog_axis_processing.cpp`, `test_local_analog.cpp`, `test_can_analog.cpp`, `test_ads111x.cpp` | `AnalogAxisProcessing.cpp`, `LocalAnalog.cpp`, `CanAnalog.cpp`, `ADS111X.cpp`, `AnalogSource.cpp`, `AnalogSources.cpp` | autoscale, manual min/max, invert, filters, raw mode, config encode/decode round trip, save/restore, local channel mask and mapping, CAN frame decoding, ADS111X register sequence (config, gain, data rate, single ended/differential), conversion and error handling, commands |

## P5 batch 0 (2026-10-26)

- F4 fake HAL SPI and F5 fake HAL I2C in `platform/src/host_target_bus.cpp` (each function modelled on `stm32f4xx_hal_spi.c` / `stm32f4xx_hal_i2c.c`: blocking/IT/DMA, state machine, `__HAL_LOCK`, error paths; completion and error callbacks in ISR context via `HostTarget::spiComplete/i2cComplete`; scripted MISO / I2C device models with register maps, NACK and bus-busy injection; `hi2c1` has no DMA like the target). Slave, 1-line, 16-bit SPI and 10-bit I2C recorded as unmodelled. `host_stubs_target.cpp` deleted (no stubs left). SPI/I2C handles initialised like `MX_SPIx_Init`/`MX_I2C1_Init`.
- F8 piece: `HostTarget::canReceive(id, data)` checks the started mock port and an active filter, then delivers through the P4 FIFOs and the real `global_callbacks.cpp`.
- F12 mocks: `mocks/MockButtonSource.h`, `mocks/MockAnalogSource.h` (selection id 15), `support/InputSources.h` (access to the `SelectableInputs` part of a main class and its choosers).
- Resets: SPI ports (CS pins, current device, exclusive flag, semaphore), `i2cport`, SPI/I2C handler lists restored to their static-init state (private members via explicit-instantiation member pointers in `host_target_config.cpp`). Still not reset: `motor_uart` device pointer.
- 10 smoke cases (`InputSetup`), 1 KNOWN_ISSUE (bug 53).
- Gate: ctest 4/4 in all 6 dirs, baseline OK, `ffb_unit_tests`/`ffb_flash_tests` unchanged (compile commands identical), `ffb_target_tests` 683/0/2 (default, clang23), 684/0/2 (nodsp), 580 + 105 skipped (sanitizer); no findings, no warnings from test files. Verified again by the main agent.

## P4 results (2026-10-25)

**Final state after the review fixes:** `ffb_target_tests` 675 cases: 673 passed / 0 failed / 2 skipped (default, clang23), 674/0/2 (nodsp), 571 passed + 104 skipped (san, clang23-san), 572 + 104 (nodsp-san); ctest 4/4; baseline OK; `ffb_unit_tests` (646/0/4 ...) and `ffb_flash_tests` (158/0/0) unchanged, compile commands identical; no sanitizer findings, no duplicates, 3 random orders green. Coverage: `SystemCommands.cpp` 231/231, `cppmain.cpp` 45/46 (`Flash_Init` failure: map fake never fails), `global_callbacks.cpp` 182/194 (93.8 %: the wrongly typed `tud_suspend_cb()` / `tud_hid_report_complete_cb` are no longer called by tests and never by TinyUSB - reachable only once bugs 42/43 are fixed; weak `HAL_TIM_PeriodElapsedCallback` dead code), `USBdevice`, `AdcHandler`, `ledEffects`, `voltagesense` 100 %. All in-scope sources **5,078 of about 8,108 (62.6 %)**, about 1,815 lines from the 85 % goal. Fix experiments: bugs 7, 42, 43, 46, 47, 48, 49, 52 and NEEDS REVIEW 53, 54, 57, 58, 60, 70, 73 fixed in scratch copies flip only their own KNOWN_ISSUE / characterization cases.
- Review fixes: shared infrastructure (OTP model reproducing `flash_helpers.cpp` incl. bug 41, MinGW `mallinfo` shim, IWDG prescaler as `main.c`, RTOS heap counters, configure-time include-list guard for `host_cppmain.cpp`, notes on `mainclass` in NOLOAD `.ccmram` and the reserved `malloc` macro); A: access-flag tests that fail without the flag check, `lsmain` visibility with a test registry, `signature` on the OTP model, boundary values (78 cases, `SystemCommands.cpp` 100 %); B: no lock-in of bug 47, bug 42/43 tests with weak stand-ins in the TinyUSB signatures (a firmware fix still links), malloc/free checked against the RTOS heap; C: no lock-in of NR 70, wrap test fixed, real `wTotalLength` checks incl. MIDI, NR 73 split into a KNOWN_ISSUE (bug 52) and a characterization, NR 75 characterization; main agent: `test_system_setup.cpp` LED smoke test tolerant of NR 70.

### Build step

- **Group A** (system commands) merged early (protection against /tmp cleaning): `test_system_commands.cpp` (63 cases; real failsafe main class from `mainchooser.Create(0)`, 5 end-to-end cases over CDC for the access flags). Coverage `SystemCommands.cpp` 50/231 -> 220/231 (95.2 %); uncovered: signature info/get paths (the `ffb_target_tests` OTP stub returns false without writing `dat`, so they would read an uninitialised value; on the target `OTP_Read` 0..7 succeeds), deleting-destructor variant, `Flash_Format` failure branch (fake never fails). Mutation 17/17. KNOWN_ISSUE: bug 7 seen through the `errors` command (worse than recorded: every `errors` read hides persistent errors from the next read, because `replyErrors` ends with `clearTemp()` -> `sortErrors()`). NEEDS REVIEW 54-59. Also covers `ErrorHandler.cpp:162-167` (error printer output, deferred from P1). Gate: `ffb_target_tests` 469/0/2 (default, clang23), 470/0/2 (nodsp), 370 + 101 skipped (san), ctest 4/4, baseline OK.
- **Group B** (callbacks and start-up) merged early: `test_global_callbacks.cpp` (44), `test_cppmain.cpp` (cppmain 18, USBdevice 14, AdcHandler 7) = 83 cases. Additive fake change: `HostTarget::State::errorHandlerLimit` + `ErrorHandlerLimitReached` (makes the wrong-chip-id `Error_Handler` loop at `cppmain.cpp:33-42` testable: **no longer a hardware-only candidate**). Coverage: `global_callbacks.cpp` 41/194 -> 191/194 (98.5 %), `cppmain.cpp` 31/46 -> 45/46, `AdcHandler.cpp` 14/14, `USBdevice.cpp` 45/45. Uncovered: `global_callbacks.cpp:80-82` (weak `HAL_TIM_PeriodElapsedCallback`, always overridden by main.c: dead code; **exclusion proposed by the agent and declined** - only hardware-only code is excluded, dead code stays counted), `cppmain.cpp:47` (`Flash_Init` failure: the map fake always succeeds). Mutation 54/54 (2 by timeout: endless loop / buffer overrun). KNOWN_ISSUE: bugs 42, 44. NEEDS REVIEW 60-63. Gate after merge: `ffb_target_tests` 552/0/2 (default, clang23), 553/0/2 (nodsp), 453 + 101 skipped (san, clang23-san), 454 + 101 (nodsp-san); ctest 4/4; baseline OK; no duplicates.
- **Group C** (LEDs, voltage, USB descriptors): `test_led_effects.cpp` (23), `test_voltagesense.cpp` (36), `test_usb_descriptors.cpp` (44; parses the descriptor bytes itself, includes the unmodified `usb_hid_*_desc.c` per namespace to also test the 1-axis and 32-bit variants). Shared-fake fix: the LED reset values in `host_target_system.cpp` were snapshotted after other files' command handlers had already called `pulseSysLed()` (so every test started inside a 25 ms pulse); now written literally like `ledEffects.cpp:10-18`. Coverage: `ledEffects.cpp` 41/59 -> 59/59, `voltagesense.cpp` 23/37 -> 37/37; descriptor files are data only (checked by the parser and by mutation). Mutation 52/54 (2 equivalent). KNOWN_ISSUE: bugs 46-49. NEEDS REVIEW 70-75.
- **Conflict to resolve in review:** `USBdevice` interface-string suffix: group B characterizes the current "(4259)" (NEEDS REVIEW 62), group C has a KNOWN_ISSUE for the documented "(0032)" (bug 47). The header documents the intent, so the KNOWN_ISSUE stays and the characterization must not assert the current value.
- **Merged gate (A+B+C):** `ffb_target_tests` 655/0/2 (default, clang23), 656/0/2 (nodsp), 553 + 104 skipped (san, clang23-san), 554 + 104 (nodsp-san); ctest 4/4; baseline OK; `ffb_unit_tests`, `ffb_flash_tests` unchanged; no sanitizer findings, no duplicates, 3 random orders green. Coverage: P4 files 95.2-100 % (`SystemCommands` 95.2, `global_callbacks` 98.5, `cppmain` 97.8, `USBdevice`, `AdcHandler`, `ledEffects`, `voltagesense` 100); all in-scope sources **5,076 of about 8,108 (62.6 %)**, was 55.3 % after P3.
- Proposed (not done): a map-backed OTP fake in `host_platform.cpp` for `ffb_target_tests` (offsets 0..65 like the real code) to reach the signature paths.

## P4 plan (2026-10-24)

`ffb_target_tests`, new files in `cases_target/` only. 3 module agents in isolated scratch workspaces (`scratchpad/p4/<group>`), per-file goal 85 % without hardware-only code, mutation spot-check 80 %.

| Group | Owns (new files) | Firmware | Behaviours |
|---|---|---|---|
| A system commands | `test_system_commands.cpp` | `SystemCommands.cpp` | every system command (help, info, hwtype, fw version, debug/heap/tasks, reset/dfu/erase recorded, UID/devid, main class selection and list, flash dump/raw read/write/format via the real flash map fake, vint/vext/temp, errors, log), access flags, invalid values |
| B callbacks + startup | `test_global_callbacks.cpp`, `test_cppmain.cpp` | `global_callbacks.cpp`, `cppmain.cpp`, `AdcHandler.cpp`, `USBdevice.cpp` | every HAL callback dispatched to the registered handlers (UART, CAN Rx/Tx/error, ADC, TIM, EXTI, SPI/I2C completions if declared), TinyUSB `tud_*_cb` callbacks (mount, suspend with null `mainclass`, HID/CDC), `micros()`, `cppmain()` start-up paths (failsafe, flash default, forceErase/format, main class selection, watchdog), `refreshWatchdog`, `getRunTimeCounterValue`, `USBdevice` task and descriptors registration, `AdcHandler` registry |
| C LEDs, voltage, descriptors | `test_led_effects.cpp`, `test_voltagesense.cpp`, `test_usb_descriptors.cpp` | `ledEffects.cpp`, `voltagesense.cpp`, `usb_descriptors.cpp`, `usb_hid_1ffb_desc.c`, `usb_hid_2ffb_desc.c`, `usb_hid_gamepad.c` | LED pulse/blink/update timing against simulated time, voltage scaling (`vSenseMult`), brake resistor hysteresis and emergency over-voltage, chip temperature average, descriptor tables self-consistent (`wTotalLength`, interface/endpoint counts, HID report descriptor lengths match the sizes used in the configuration descriptors, report IDs match `ffb_defs.h`, string descriptors) |

## P4 batch 0 (2026-10-24)

`ffb_target_tests` now compiles the real `global_callbacks.cpp`, `voltagesense.cpp`, `ledEffects.cpp` and `cppmain.cpp`; all batch 0 firmware stubs are gone (only SPI/I2C unmodelled-call recorders remain).
- `cppmain.cpp` is compiled through `platform/src/host_cppmain.cpp`, which includes cppmain's headers, `#define`s `malloc`/`free` to `hosttest_cppmain_malloc`/`_free` and then `#include`s the unmodified `cppmain.cpp` (cppmain defines `malloc`/`free`, which on the host would replace the C library and ASan allocators; a `-D` rename does not survive `<cstdlib>`). gcov reports the lines under `cppmain.cpp`. Mach-O `CCRAM_SEC` override for macOS (unverified).
- `ledEffects.cpp`: per-source defines rename `pulse*/set*Led` to `hosttest_real_*`; `host_platform.cpp` provides the public names (counting in `HostPlatform::leds()` for the existing tests, then calling the real functions). Guards `HOSTTEST_REAL_CPPMAIN`, `HOSTTEST_REAL_LED_EFFECTS` (only `ffb_target_tests`); `ffb_unit_tests` preprocessed output identical for all 60 TUs; compile commands of `ffb_unit_tests` and `ffb_flash_tests` identical.
- **F6 fake ADC** (`platform/src/host_target_system.cpp`, modelled on `stm32f4xx_hal_adc.c`: start/stop/poll/get value/DMA, state and error codes, `__HAL_LOCK`; conversion complete via `HostTarget::adcConversionComplete`/`adcConvert`), LL ADC macros copied from `stm32f4xx_ll_adc.h`, calibration values from host variables (not 0x1FFF7A2A..). TIM7 HAL tick (CNT from simulated time: real `micros()` = simulated time), strong `HAL_TIM_PeriodElapsedCallback` like `main.c`, `Error_Handler` (counted, returns), IWDG refresh, CAN receive FIFOs (`HAL_CAN_GetRxFifoFillLevel`/`GetRxMessage`, overrun), `canport` mock with `getHandle() == &hcan1` so the real CAN callbacks dispatch to it.
- Fixture: `HostTarget::resetSystem()` restores ADC/TIM7/IWDG/CAN fakes and the firmware globals of voltagesense, ledEffects and cppmain (snapshotted at static init); `TargetFixture` deletes the main class created by `runCppmain` and resets `SystemCommands::errorPrintingEnabled`.
- 10 smoke cases (`SystemSetup`), 1 characterization (NEEDS REVIEW 53).
- Gate: ctest 4/4 in all 6 dirs, baseline OK, `ffb_unit_tests` and `ffb_flash_tests` unchanged, `ffb_target_tests` 406/0/2 (default, clang23), 407/0/2 (nodsp), 307 + 101 skipped (san, clang23-san), 308 + 101 (nodsp-san); no sanitizer findings; no warnings from test files. Verified again by the main agent.
- Hardware-only candidates: `cppmain.cpp:33-42` (wrong chip id: endless `Error_Handler` loop) - made testable by group B (`errorHandlerLimit`), not excluded.

## P3 results (2026-10-24)

**Final state after the review fixes:** `ffb_flash_tests` 158 cases, 158 passed / 0 failed / 0 skipped in all 6 build dirs (14 KNOWN_ISSUE cases fail as expected), no sanitizer findings, no duplicate names, 3 random orders green; ctest 4/4; `ffb_unit_tests` (646/0/4, 643/0/7, 645/0/5) and `ffb_target_tests` (396/0/2, 397/0/2, 298+100 skipped, 299+100) unchanged, their compile commands identical; baseline OK. Coverage: `eeprom.c` 224/239 (93.7 %), `flash_helpers.cpp` 92/93 (98.9 %), `PersistentStorage.cpp` 22/22; all in-scope sources **4,480 of about 8,096 (55.3 %)**, about 2,400 lines from the 85 % goal. Fix experiments (reviewers and authors): with bug 41, V5 and V6 fixed only their KNOWN_ISSUE cases flip, no passing test fails.
- Review fixes, group B (p3r2): OTP second-word failure test (mutant at `flash_helpers.cpp:312` now killed), `FlashFixture::resetIteration()` / `powerCycle()` (RAM cleared on reboot: `DataVar`, startup flag), NEEDS REVIEW numbers in every characterization, header path for `test_eeprom_addresses.cpp` passed from CMake (`FFB_EEPROM_ADDRESSES_H`), stricter header parsing, fake: unmodelled sectors recorded before the lock check, HAL_BUSY injection like `__HAL_LOCK`, unaligned program -> PGPERR (model assumption), `<windows.h>` first; **model assumptions documented**: unlock failure (on the chip a wrong key sequence is a bus error / HardFault, RM0090 3.5.1) and "erase while locked returns HAL_OK" (inferred from RM0090 3.9.7).

### Build step

- **Group B** (flash helpers) merged early (files existed only in /tmp, which was cleaned 3 times): `test_flash_helpers.cpp` (58), `test_eeprom_addresses.cpp` (14), `test_persistent_storage_flash.cpp` (11) = 81 cases; `ffb_flash_tests` 90/0/0 in default, san, clang23; baseline OK; ctest 4/4. Coverage: `flash_helpers.cpp` 37/93 -> 92/93 (98.9 %; line 382 unreachable: no factory defaults defined), `PersistentStorage.cpp` 100 %, `eeprom_addresses.c` data only (0 executable lines; `coverage_report.py` now counts files with a `.gcno` but no line data as compiled). Mutation (before the /tmp cleanup, identical test logic): `flash_helpers.cpp` 38/38, `PersistentStorage.cpp` 9/9, `eeprom_addresses.c` 11/12 (survivor: swapping two table entries, no specified meaning) = 58/59. KNOWN_ISSUE: bug 41 (3 cases), bug V5 at the firmware layer (`Flash_Format` after a page transfer leaves page 1 VALID: a later write is lost). New NEEDS REVIEW 43-48.
- **Group A** (`eeprom.c`, vendor tier A): `test_eeprom.cpp` (63 cases: all 16 page-status combinations of `EE_Init`, read/write, page transfer both directions, format, every error return, power loss at every step of write/transfer/recovery/format incl. nested recovery cuts). Additive fake change `HostFlash::failNextAfter` (fail a later step of a sequence). Coverage `eeprom.c` 160/239 -> 224/239 (93.7 %); the 15 lines left: 12 unreachable because of V5 (page 1 erase branches 189-193, 205-209, 251-255, 485-489), 562-563 and 669 unreachable. Mutation 27/28 (survivor equivalent). KNOWN_ISSUE: V5 (6 cases), V6 (1). Characterizations: NEEDS REVIEW 41 and 49-52.
- Merged gate (both groups, documented exclusions): ctest 4/4 in all 6 dirs, baseline OK, `ffb_unit_tests` and `ffb_target_tests` unchanged, `ffb_flash_tests` 153/0/0 everywhere, no sanitizer findings, no duplicate names, 3 random orders green.

## P3 plan (2026-10-24)

Executable `ffb_flash_tests`, new files in `cases_flash/` only. 2 module agents in isolated scratch workspaces (`scratchpad/p3/<group>`), per-file goal 85 %, mutation spot-check 80 % (about 15 mutants per file).

| Group | Owns (new files) | Firmware | Behaviours |
|---|---|---|---|
| A eeprom | `test_eeprom.cpp` | `eeprom.c` (vendor tier A, AN3969) | `EE_Init` for every page-status combination (ERASED/RECEIVE_DATA/VALID/invalid, incl. power-loss states), `EE_ReadVariable` (newest record wins, missing variable), `EE_WriteVariable` (page full -> `EE_PageTransfer`, only variables in `VirtAddVarTab` transferred, the new value kept), `EE_Format`, HAL error propagation (program/erase/unlock failures), power loss at every step of write/transfer/format, bug V5 (`EE_VerifyPageFullyErased`) as KNOWN_ISSUE, NEEDS REVIEW 41 |
| B flash helpers | `test_flash_helpers.cpp`, `test_eeprom_addresses.cpp`, `test_persistent_storage_flash.cpp` | `flash_helpers.cpp`, `eeprom_addresses.c`, `PersistentStorage.cpp` on the real flash layer | `Flash_Init`, `Flash_Write` (write only if changed, return values), `Flash_Read`, `Flash_ReadWriteDefault`, `Flash_Dump`, `Flash_Format`, cache handling, locking, HAL error handling (NEEDS REVIEW 42), `OTP_Write`/`OTP_Read` (one-time semantics, bounds, bug 41), `VirtAddVarTab` (`NB_OF_VAR` entries, unique, no 0xFFFF, matches the `ADR_*` constants), save/restore of handlers through the real flash incl. reboot and page transfer, delayed restore ordering |

## P3 batch 0 (2026-10-24)

New executable `ffb_flash_tests` (CMake block appended; `ffb_unit_tests` and `ffb_target_tests` compile commands identical in all 6 build dirs). Cases in `cases_flash/`. Feature header: F407 test header (`USE_EEPROM_EMULATION`, `EEPROM_START_ADDRESS 0x08004000`, `OTPMEMORY`). Defines `HOSTTEST_FLASH_HAL` (target `main.h` includes `host_flash_hal.h`, values from the ST headers) and `HOSTTEST_REAL_FLASH_HELPERS` (the map-backed `Flash_*`/`OTP_*` fake in `host_platform.cpp` is left out). Exclusion lists `FFB_FLASH_TESTS_EXCLUDE` / `_SANITIZE` (empty). No CMSIS-DSP.
- Firmware compiled: `flash_helpers.cpp`, `eeprom.c` (tier A), `eeprom_addresses.c`, `PersistentStorage.cpp`; for linking `cmutex.cpp`, `csemaphore.cpp`, `UART.cpp`, `UartHandler.cpp`. No stubs.
- F9 fake HAL FLASH (`platform/src/host_flash.cpp`, `support/HostFlash.h`) modelled on `stm32f4xx_hal_flash(_ex).c` and RM0090: LOCK bit, program = old & data with 0->1 requests recorded, PGSERR when locked, PGAERR across a 128-bit row, pending SR flags, sector erase (sectors 0-3), erase while locked ignored, OTP 16x32 bytes + lock bytes (WRPERR on locked block), cache macros counted; injections (error flags, HAL_TIMEOUT/HAL_BUSY, unlock failure) and **power loss** (cut after N programs with torn-bit mask, partial erase), recording per sector, `unexpected` accesses fail the test.
- **Fixed-address memory:** 64 KiB mapped at 0x08000000 and at 0x1FFF0000 at startup (POSIX `mmap` hint + `MAP_FIXED_NOREPLACE`, never `MAP_FIXED`; Windows `VirtualAlloc` at the fixed base), abort with a message if unavailable. Verified on Linux x86-64 incl. ASan (LowMem 0x0-0x7fff7fff is application memory). **Not verified on Windows; not built on macOS** (4 GiB `__PAGEZERO`; decision 2026-10-24).
- `support/FlashFixture.h`: resets fake flash, injections, PersistentStorage lists/startup flag, `eeprom.c` `DataVar`; `init()`, `reboot()`, page image helpers (`writePage`, `fillPage`, `pageRecords`, ...), `REQUIRE_FLASH_ACCESSES_MODELLED()`.
- 9 smoke cases (`FlashSetup`), incl. a power loss at every program step of a page transfer (recovers).
- Gate: ctest 4/4 in all 6 dirs, baseline OK, `ffb_unit_tests` and `ffb_target_tests` unchanged, `ffb_flash_tests` 9/0/0 everywhere, no sanitizer findings, no warnings from test files (firmware `-Wint-to-pointer-cast` in `eeprom.c`/`flash_helpers.cpp`: 32-bit addresses as pointers, host only). Verified again by the main agent.
- Hardware-only candidates: none.

## P2 results (2026-10-23)

4 module agents (A axis torque, B axis config, C axes manager + integration, D main classes), merged by the main agent (each workspace only added its own files; no shared helper or CMake change). 377 new cases in 6 files in `cases_target/` (A 103/102 depending on DSP, B 80, C 81, D 114); `ffb_target_tests` now 388 cases.

| Group | Files | Mutation spot-check (passing tests only) |
|---|---|---|
| A | `test_axis_torque.cpp` | 36/37 `Axis.cpp` (survivor: hands-off timing, unobservable because of bug 35) |
| B | `test_axis_config.cpp` | 32/32 `Axis.cpp` |
| C | `test_axes_manager.cpp`, `test_ffb_integration.cpp` | 31/31 (14 `AxesManager.cpp`, 17 along `Axis`/`EffectsCalculator`/`HidFFB`) |
| D | `test_ffbhidmain.cpp`, `test_main_classes.cpp` | 70/72 (2 equivalent) |

**Final state after the review fixes:** `ffb_target_tests` 398 cases: 396 passed / 0 failed / 2 skipped (default, clang23), 397/0/2 (nodsp, one more build-specific case), 298 passed / 100 skipped (san, clang23-san), 299/100 (nodsp-san); the skips are the 2 crash cases (bugs 31, 37) plus, in sanitizer builds, the `DELETES_FFBHIDMAIN` cases and 1 KNOWN_ISSUE_UB. `ffb_unit_tests` 646/0/4, 643/0/7, 645/0/5 (unchanged), baseline OK, ctest 3/3, no sanitizer findings, no warnings from test files, no duplicate names, 3 random orders green. Coverage unchanged by the fixes (`Axis.cpp` 99.4 %, P2 files 96.5-100 %, all in-scope 4,164 / ~8,243 = 50.5 %). Distance to the 85 % goal: about 2,840 lines.

Gate after the first merge (before review, documented exclusions): `ffb_unit_tests` 646/0/4 (default, nodsp, clang23), 643/0/7 (san, clang23-san), 645/0/5 (nodsp-san), baseline OK; `ffb_target_tests` 386 passed / 0 failed / 2 skipped (default, clang23), 387/0/2 (nodsp), 292 passed / 96 skipped (sanitizer builds: `DELETES_FFBHIDMAIN` cases, decision 2026-10-23, plus 1 KNOWN_ISSUE_UB); ctest 3/3 everywhere; no sanitizer findings; no warnings from the new files; no duplicate test names; 3 random orders green.

Coverage (gcov, GCC 16, DSP on, both executables): `Axis.cpp` 673/677 (99.4 %, was 43 %), `AxesManager.cpp` 62/62, `FFBHIDMain.cpp` 167/173 (96.5 %), `FFBHIDMain_commmands.cpp` 76/76, `FFBWheel`, `FFBJoystick`, `FFBHIDExt`, `CustomMain`, `FFBoardMain` 100 %. All in-scope sources **4,163 of about 8,243 (50.5 %)**, was 32.4 % after P1 (part of the gain is incidental coverage of files compiled for the main classes). Hardware-only candidates: none in the P2 files (batch 0 proposed `USBdevice.cpp:33-38`, tier B, decide in review).

## P2 batch 0 (2026-10-23)

New executable `ffb_target_tests` (CMake block appended, nothing above it changed; `ffb_unit_tests` compile commands identical in all 6 build dirs, checked against copies saved before the change).
- Feature header `platform/target_f407/target_constants.h` + standalone target `main.h` / `cmsis_os.h` in `platform/target_f407/` (first in the include path). No `fakes/`: real `Axis.h`. Cases in `cases_target/test_*.cpp`. Own ctest exclusion list `FFB_TARGET_TESTS_EXCLUDE` (empty).
- **Deviations from TEST_PLAN/PROGRESS:** platform fakes are compiled per executable (they include `main.h`/`target_constants.h`), not one shared static library. `-funsigned-char` for `ffb_target_tests` only (ARM `char` is unsigned; `CanBridge.h` needs it); `ffb_unit_tests` keeps signed `char` (host-vs-target caveat).
- Firmware compiled: the 24 `ffb_unit_tests` sources, the P2 targets (`Axis`, `AxesManager`, `FFBoardMain`, `FFBHIDMain`, `FFBHIDMain_commmands`, `FFBWheel`, `FFBJoystick`, `FFBHIDExt`, `CustomMain`, `mainclass_chooser`) and what the registries pull in (`CanBridge`, `CanInputMain`, `SystemCommands`, `SelectableInputs`, `ButtonSource`, `AnalogSource`, `AnalogAxisProcessing`, `AdcHandler`, `SerialFFB`, `TimerHandler`, `ExtiHandler`, `USBdevice`, `SPI`, `SpiHandler`, `I2C`, `I2CHandler`, `CRC`, `ButtonSources`, `AnalogSources`, `EncoderSources`, `LocalButtons`, `SPIButtons`, `ShifterAnalog`, `PCF8574`, `CanButtons`, `LocalAnalog`, `CanAnalog`, `ADS111X`, `EncoderLocal`, `MtEncoderSPI`, `EncoderBissC`, `EncoderSSI`, USB descriptors). Not compiled: `CANPort2B.cpp` (P7), drivers/MIDI.
- Stubs: `platform/src/host_target_config.cpp` replaces `Targets/**/cpp_target_config.cpp` (same objects; `canport` is a `MockCANPort` until the CAN fake, P7). `platform/src/host_stubs_target.cpp` (deleted in P5 batch 0, nothing left to stub): deferred parts of `cppmain.cpp`, `global_callbacks.cpp`, `voltagesense.cpp` (P4), each to be removed when the real source is compiled. `HAL_SPI_*`/`HAL_I2C_*` are link stubs that record the call; `TargetFixture` fails a test that calls an unmodelled HAL function (until F4/F5/F6).
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
| 25 | AxesManager | `AxesManager.cpp:59-65` `getAxisValues` | Axes with the None encoder are skipped, so the next axis moves into the previous HID slot (Y reported as X; analog values move into X). Shift intended, or report 0 for that slot? Review p2r2: `send_report` (`FFBHIDMain.cpp:192-212`) fills the remaining slots with analog values and an `axisValues[i] =` line is commented out, which suggests the compaction is deliberate | no |
| 26 | Axis | `Axis.cpp:226, 514-516` | An unknown stored driver id is reported by `drvtype` and saved again although the driver is None | no |
| 27 | Axis | `Axis.cpp:557-563` | Invalid `enctype` reports 0 but keeps the old encoder; an encoder in use by another axis leaves the second axis with a nullptr encoder but enctype 2 | no |
| 28 | Axis | `Axis.cpp:535-538` | `setDrvType` sets `control->usb_disabled = false` when USB is NOT connected. Inverted? | no |
| 29 | Axis | `Axis.cpp:1287, 1009-1011, 1223` | Value handling of commands (power above 0x7fff is bug 32, not a review item); 8-bit settings keep the low byte; eq gain wraps through `int8_t` before the clip (200 -> -56); range masked to 15 bits and 0 restores the previous range; reduction 0 wraps to 256 | no |
| 30 | Axis | `Axis.cpp:1207-1216` | `curpos`/`curspd`/`curaccel` report `metric.previous` (one cycle late) | no |
| 31 | Axis | `eeprom_addresses.h`, `Axis.cpp:1399` | Config word comments ("0-2 ENC, 3-5 DRV", "0-6 enc, 7-12 Mot") do not match the code (bits 0-5 encoder, 6-11 driver) | no |
| 32 | Axis | `Axis.cpp:944-950` | Speed limiter strength differs by orders of magnitude between the CMSIS-DSP (`arm_pid` on deg/s error) and the fallback (error scaled by 0x7fff/maxspeed) build. Which scaling is intended? | no |
| 33 | Axis | idle spring | Clip at 10000 can never act (max 255*35 = 8925); internal damper on by default (strength 30); `scaleEncValue(angle, 0)` returns (0x7fff, 0.0) | no |
| 34 | Axis | `Axis.cpp:1339` | Hands-off counter is a function `static` shared by all axes and tests (cannot be reset), checks every 11 calls (see bug 35) | no |
| 35 | FFBHIDMain | `send_report` | Result of `tud_hid_report` ignored, buffers swapped anyway: a rejected report is lost until the next change or the 100-cycle resend | no |
| 36 | FFBHIDMain / SelectableInputs | `addBtnType`/`addAinType` | Do not update the `btntypes`/`aintypes` mask, so an added source is not saved to flash | no |
| 37 | FFBWheel | before `usbInit` | HidFFB default direction mask 0x02: a direction-90 constant force acts at half strength with the opposite sign until `usbInit` | no |
| 38 | FFBHIDMain | `FFBHIDMain.cpp:321-322` `usbResume` | Clears `control.emergency` when the E-stop pin is released, but never calls `axes_manager->emergencyStop(true)`: the drivers stay in emergency stop while `estop` reports 0. Intended? | no |
| 39 | FFBHIDMain | `FFBHIDMain.cpp:343` | An E-stop release within 1 s of the press is never applied (motor stays stopped until the next edge). Intended (fail-safe) or part of bug 40? | no |
| 40 | FFBHIDMain | `FFBHIDMain.cpp:128-159` `updateControl` | Does not check `usb_disabled`: during a USB suspend the axes keep computing torque and `turn()` it to the stopped driver (and the slew limiter winds down during a long suspend). Intended? (found while fixing bug 33 tests) | no |
| 41 | eeprom.c (vendor, AN3969 design) | `EE_Init` | Power loss while erasing the old page after a completed transfer, with its VALID header surviving: the next `EE_Init` copies the partially erased old page again and appends stale values (no "erase in progress" state). Accept as AN3969 limitation? | no |
| 42 | flash_helpers | `Flash_Init`, `Flash_Write`, `Flash_Format`, `Flash_ReadWriteDefault` | Ignore the result of `HAL_FLASH_Unlock`; `Flash_ReadWriteDefault` ignores the result of `EE_WriteVariable`. Note (review p3r2): an unlock failure is probably unreachable on the chip without a fault (wrong key sequence = bus error), so the unlock part is defensive | no |
| 43 | flash_helpers | `Flash_Init` with unlock failure (defensive: probably unreachable on the chip, see 42) | An erase-only repair (page 0 ERASED but dirty, page 1 VALID) "succeeds" while locked (HAL erase while locked returns HAL_OK and erases nothing): `Flash_Init` returns true and page 0 stays dirty (part of NEEDS REVIEW 42) | no |
| 44 | flash_helpers | `Flash_Format` | Does not clear pending error flags (Init/Write/ReadWriteDefault do): the first format after a pending flag fails, a second works | no |
| 45 | flash_helpers | `OTP_Write` | Programs the high word after the low word failed: a half-written slot that can never be rewritten. Stop after the first failure? Writing all ones returns true and the slot stays writable | no |
| 46 | flash_helpers | `Flash_Read` `checkempty` | Ignored with the EEPROM emulation (a stored 0xFFFF is found); the I2C backend would report it missing | no |
| 47 | flash_helpers | `Flash_Dump` | Appends to the caller's vector instead of clearing it | no |
| 48 | eeprom_addresses | `ADR_CANANALOG_CONF1` | Not exportable (no export flag in `memory_map.csv`), unlike every other input configuration. Intended? | no |
| 49 | eeprom.c (vendor) | `EE_Init` with a valid page 0 and an invalid page 1 (e.g. torn RECEIVE_DATA header) | Returns EE_OK and every variable is lost: the invalid page 1 is treated as "receive" (`eeprom.c:259`) but the copies are written back into page 0, which is then erased | no |
| 50 | eeprom.c (vendor) | `EE_Init` with an invalid page 0 and a valid page 1 | Everything is formatted away (`eeprom.c:308`), e.g. after a torn RECEIVE_DATA header on page 0 | no |
| 51 | eeprom.c (vendor) | torn VALID header after the old page was erased | `EE_Init` formats (`eeprom.c:137`): every variable lost | no |
| 52 | eeprom.c (vendor) | records | No checksum: a torn virtual-address half-word can alias another listed address (0x0001 -> 0x0101), so the interrupted write's data appears under another variable. (eeprom.c never unlocks the flash itself: see NEEDS REVIEW 43 at the flash_helpers layer) | no |
| 54 | SystemCommands | `uid?3` | Returns OK with no reply (no such word), while `signature?8` returns ERR. Consistent error handling intended? | no |
| 55 | SystemCommands | `signature` | Help says "set=1 to lock" but a plain set returns ERR and nothing locks the OTP blocks; setat has no range check against `SIGNATURELEN` (any OTP offset, see bug 41) and, unlike `otp`, no `CMDFLAG_DEBUG` | no |
| 56 | SystemCommands | `SystemCommands.cpp:224-237` `mallinfo` | Handler exists but no command table entry: unreachable through the command thread (an unknown id would hit bug 5). Dead code? | no |
| 57 | SystemCommands | `main` set | A main class id that is not in the registry changes nothing and returns OK | no |
| 58 | SystemCommands | `main` set | An id above 16 bits is truncated by `isValidClassId(uint16_t)` and stored as its low 16 bits; the 64-bit comparison at line 161 then always resets, even for the running class | no |
| 59 | SystemCommands | `flashraw` read | An address that is not stored replies 0 with OK (same as a stored 0). `SystemCommands::errorPrintingEnabled` is defined but never read | no |
| 60 | global_callbacks | `global_callbacks.cpp:118/134` CAN rx loop | The loop index is compared with a fill level re-read after each frame: one interrupt reads ceil(n/2) frames (none lost, the interrupt fires again). Intended? | no |
| 61 | UsbHidHandler | `UsbHidHandler.h:27` | `transferComplete` is not virtual: the call at `global_callbacks.cpp:345` always runs the empty base version | no |
| 62 | USBdevice | `USBdevice.h` `appendSerial` | Moved to suspected bug 47 (header documents the intent) | moved |
| 63 | global_callbacks | `tud_mount/umount/suspend/resume_cb` | Dereference `mainclass` without the null check `tud_cdc_rx_cb` has; unreachable today (the USB task starts only from `mainclass->usbInit()`), not tested | no |
| 70 | ledEffects | `ledEffects.cpp:85` | Because of the strict `>`, each blink phase lasts period + 1 ms | no |
| 71 | ledEffects | `blinkLed(period 0, blinks > 0)` | Turns the LED on and never ends the effect (no caller today) | no |
| 72 | voltagesense | 65 V over-voltage brake | No hysteresis: can toggle with every ADC2 conversion near 65 V | no |
| 73 | voltagesense | `voltagesense.cpp:85-88` | The missing error is now bug 52; remains: the shutdown triggers no emergency stop and no err LED signal. Intended? | no |
| 74 | voltagesense / FastAvg | `FastAvg.h:93` | Chip temperature average is an integer average: stops up to 2 C short of a constant reading | no |
| 75 | usb descriptors / HidFFB | 1-axis descriptor | `FFB_SetEffect_t::directionY` receives the X "type specific block offset"; the 2-axis descriptors add 4 bytes of offsets that `HidFFB` ignores (copies `min(sizeof, bufsize)`) | no |
| 64 | SystemCommands | `flashraw` setat | Address and value (int64) are truncated to 16 bits for `Flash_Write(uint16_t, uint16_t)`: `flashraw=0x10005?0x10007` stores 5 at 7, address -1 stores at 0xFFFF, both with OK (same class as 58) | no |
| 65 | cppmain | `cppmain.cpp:126` `micros()` | Reads `HAL_GetTick()` and `TIM7->CNT` without making them consistent: if TIM7 wraps between the reads (or `micros()` runs with interrupts masked / in a higher-priority ISR while the update is pending) the result jumps back about 1000 us. Not observable with the atomic fake (review p4r2) | no |
| 76 | SPI | `main.c:1377-1388` | All external and ext3 SPI CS pins start low after `MX_GPIO_Init` (the asserted level of active-low devices) until a device's first transfer. Intended? | no |
| 77 | SPIButtons | defaults | Without a stored config the speed preset is 0 ("Fast", prescaler 16) although `ButtonSourceConfig` documents Medium | no |
| 78 | SPIButtons | TM mode (`SPIButtons.cpp:122, 160-163`) | With a byte-aligned button count the offset is 8: the whole first byte is dropped (7 buttons drop 1 bit, 8 drop 8; with 64 TM buttons the last 8 are lost) | no |
| 79 | SPIButtons | `btncut` | Moved to suspected bug 75 (p5r1) | moved |
| 80 | LocalButtons | pulse mode (`LocalButtons.cpp:67-77`) | A pulse ends early when another pulse-mode button changes | no |
| 81 | PCF8574 | inverted inputs | Before the first complete read and while the expander does not answer every inverted button reads pressed | no |
| 82 | ShifterAnalog | mode | Any mode value accepted (command and flash); an unknown mode reports 0 buttons. Also: G27 reads start a DMA receive on every `readButtons` without checking the port (on the target the FFB thread blocks on the SPI semaphore). `getButtonValues`/`addBtnType`/`addAinType` do not take `sourcesSem` (race with a `btntypes` change, not testable without preemption) | no |
| 83 | CanButtons / PCF8574 / ShifterAnalog | `CanButtons.cpp:84, 109-127`, `PCF8574.cpp:173, 190-207`, `ShifterAnalog.cpp:221-229` | Unsupported request types (btnnum info, canid/speed/mode getat, mode setat) return OK with no reply instead of ERR; btnnum is truncated to `uint8_t` before the 1..64 clip (300 -> 44). Intended? | no |
| 84 | CanAnalog | `CanAnalog.cpp:39` | Filter mask `~(canId ^ (canId+1))` is exact for an even id (2 ids) but accepts 32 ids (96..127) for id 111; frames are never decoded wrongly, the filter is just wider. Intended? | no |
| 85 | AnalogAxisProcessing | `AnalogAxisProcessing.cpp:159` | Without manual scaling every `autocal` command (also get or set 0) resets the learned range (latent: both users allow manual scaling) | no |
| 86 | AnalogAxisProcessing | `autorangeScale` 1.05 (`AnalogAxisProcessing.h:96`) | Margin only on the top side: minimum -> -0x7fff, middle -> about +1638, output saturates at about 95 % of the range; also for a manual range | no |
| 87 | AnalogAxisProcessing | `AnalogAxisProcessing.cpp:97-99` | Filter sample counter shared by all axes: with 2 axes autorange starts after 251 calls instead of 500 | no |
| 88 | LocalAnalog | `LocalAnalog.cpp:79-87` | min/max and filter belong to the n-th enabled pin, not pin n: changing the mask moves every stored range to another pin | no |
| 89 | AnalogAxisProcessing | manual range min > max | Values stay unscaled (neither inverted nor rejected) | no |
| 90 | AnalogAxisProcessing | restore | With autorange restored on, a manual range still in flash becomes the starting range | no |
| 91 | ADS111X | `ADS111X.cpp:327` | 4 s timeout counts from the last finished cycle: after more than 4 s without requests the next `getAxes` resets the I2C port (all devices) instead of converting | no |
| 92 | CanAnalog | receive | No receive timeout: the axes keep the last values forever | no |
| 53 | cppmain | `cppmain.cpp:88-90` with `flash_helpers.cpp:105-120` | `Flash_ReadWriteDefault` returns false after writing a default, so `cppmain` calls `Error_Handler` on every first boot (empty flash); `Error_Handler` only switches the error LED off. Probably unintended | no |
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
| 7 | ErrorHandler | `ErrorHandler.cpp:130-143` (`getErrors` + `sortErrors`) | Empty slots (code 0) sort to the front; `getErrors()` stops at the first empty slot, so after any `clearError`/`clearTemp` it returns an empty list while errors are still stored. **Visible via the `errors` command after every read** (P4: `replyErrors` ends with `clearTemp()` -> `sortErrors()`, so each `errors` read or error-printer pass hides persistent errors from the next read) | KNOWN_ISSUE "the other errors stay listed after one error is cleared" (`test_error_handler.cpp`) and "a persistent error is still listed by a second errors command" (`test_system_commands.cpp`). Confirmed in source |
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
| 24 | FFBHIDMain | `Inc/ffb_defs.h:135` (`HID_GamepadReport_base` without virtual destructor), deleted via `std::unique_ptr` in `~FFBHIDMain` (`FFBHIDMain.cpp:64`) | Deleting any FFBHIDMain main class (e.g. on main class change) is UB: ASan new-delete-type-mismatch | KNOWN_ISSUE on the type trait (no UB executed); tests that delete a main class are marked `DELETES_FFBHIDMAIN` and skipped in sanitizer builds (decided 2026-10-23) |
| 25 | Axis | `Axis.cpp` | No `#include <cmath>`: compiles only with CMSIS-DSP (`-include cmath` added for `ffb_target_tests` with DSP off, F14) | Build |
| 26 | Axis | `Axis.cpp:1415` `clip<int8_t>(gain,-120,120)` | Narrowing inside `clip()` (`cppmain.h`): ill-formed C++, GCC warns, clang rejects (`-Wno-c++11-narrowing` for clang, `Axis.cpp` only, `ffb_target_tests` only). No runtime effect | Build |
| 27 | Axis | `Axis.cpp:519-525` `setDrvType` | A valid but not creatable driver id leaves `drv == nullptr`; `getEncType`, `setEncType`, `updateDriveTorque`, `emergencyStop` would dereference it | KNOWN_ISSUE "selecting a driver that cannot be created keeps a driver on the axis" (`test_axis_config.cpp`) |
| 28 | Axis | `Axis.cpp:366` `saveFlash` | `(uint16_t)(handsOffAccelThreshold*1000.0)` truncates: 985 of the values 0..1999 lose 1 per save (10 is saved as 9) | KNOWN_ISSUE "a hands-off acceleration threshold set by command survives a flash save and restore" |
| 29 | Axis | `Axis.cpp:1320` | The hands-off accel get reply truncates the float: 251 reads back as 250 | KNOWN_ISSUE "the hands-off acceleration threshold reads back the value that was set" |
| 30 | Axis | `Axis.cpp:875` vs `Axis.h:356-361, 473` | `startForceFadeIn(start, fadeTime)` is documented as "over fadeTime", but `fadeTime/samplerate` is added per cycle: the fade takes (1-start)/fadeTime s (zeroenc 1.5 s, usbResume/estop reset 2 s instead of 0.5 s) | KNOWN_ISSUE (B) "zeroenc fades the force in within the 0.5 s fade time" and (A) "after a USB resume the torque is back to full within the documented fade time of 0.5 s" |
| 31 | Axis | `Axis.cpp:294-299` `restoreFlash` | Stored filter profile used unchecked as index into the 4-element `filterSpeedCst`/`filterAccelCst`: out-of-bounds read (aborts under `_GLIBCXX_ASSERTIONS`) | Skipped KNOWN ISSUE (crash) "a stored filter profile out of range is limited to the last profile like the command does" |
| 32 | Axis | `Axis.cpp:498-501, 892-894` | `setPower` accepts any `uint16_t`; the torque is clipped to +-power and passed to `MotorDriver::turn(int16_t)`: power 40000 with full force sends -25536 (**motor reverses**) | KNOWN_ISSUE "the driver never receives a torque above 0x7fff or with the wrong sign when the power is above 0x7fff" |
| 33 | Axis | `Axis.cpp:917-921` with `618` | **Safety:** after an emergency stop reset the slew limiter starts from `metric.previous.torque` (torque before the stop), not from 0: first torque after reset 9899 with slew rate 100; bypasses slew limit and fade-in. Review p2r1 confirmed the same after `usbSuspend`/`usbResume` (first torque 9899); KNOWN_ISSUE "after a USB resume the slew rate limiter starts from the zero torque of the stopped motor" | KNOWN_ISSUE "after an emergency stop reset the slew rate limiter starts from the zero torque of the stopped motor" |
| 34 | Axis | `Axis.cpp:1026, 1347` | **Withdrawn (review p2r1, 2026-10-23): not a target bug.** On the target (arm-none-eabi-g++, Makefile flags) the float `abs` overload is visible through a transitive `<math.h>` (`TMC4671.h -> SPI.h:17`); only the host test build without the TMC driver saw `::abs(int)`. `ffb_target_tests` now compiles `Axis.cpp` with `-include math.h`; the two expo tests are normal passing tests. Kept as a host-vs-target caveat (fragile transitive include) | Expo tests pass |
| 35 | Axis | `Axis.cpp:1332-1392`, `Axis.h:455` | Hands-off detection computes `handsOff` but nothing reads it ("Safety override ... cut torque" never happens; unused since commit 40c65391) | 2 KNOWN_ISSUE (speed override, steady wheel) |
| 36 | Axis | `Axis.cpp:656` `getLastScaledEnc` | Clip in float: 0x7fffffff becomes 2^31; position >= 1.0 converts out of range to int32 (UB). Host gives INT_MIN (opposite end in the HID report), ARM saturates. UBSan does not check float-cast-overflow | KNOWN_ISSUE_UB "an axis beyond the end of its range reports the largest gamepad position" (host-vs-target) |
| 37 | FFBHIDMain | `FFBHIDMain.cpp:250` `setReportRate` | `clip(rateidx, 0, ffbrates.dividers.size())` allows index 4 of a 4-element array (command `hidsendspd=4`): out-of-bounds read at 256/260/262 (aborts under `_GLIBCXX_ASSERTIONS`; on target garbage dividers, possible division by 0) | Skipped KNOWN ISSUE (crash) "hidsendspd with an index beyond the last rate selects the last rate" |
| 38 | SelectableInputs | `SelectableInputs.cpp:44-46, 76-78` | `addbtn`/`addain` duplicate check compares the class id (`getInfo().id`) with the selection id: never matches, a second identical source is created | 2 KNOWN_ISSUE (`test_ffbhidmain.cpp`) |
| 39 | FFBHIDMain | `FFBHIDMain.cpp:361-364` with `298` | A new critical error calls `emergencyStop(true)`, but `true` means reset: **a critical error releases an active emergency stop** instead of triggering one. History (review p2r3): commit `8fad1a7b` (2022-06-02) introduced `emergencyStop(bool reset)` and converted this call to `true` while `exti` got `false`; `3105628c` added `control.emergency = !reset`. A regression: before `8fad1a7b` a critical error did stop | KNOWN_ISSUE "a critical error puts the main class into emergency stop" and "a critical error keeps an active emergency stop" |
| 40 | FFBHIDMain | `FFBHIDMain.cpp:343` (`exti`, E-stop debounce) | **Safety:** an E-stop edge within 1 s of the previous one is dropped and nothing re-reads the pin later (no polling in `updateControl`): pressing the E-stop within 1 s after a release, or within 1 s after boot (`lastEstop` starts at 0), leaves the motor running while the E-stop is held (found by review p2r3 with 2 probe tests) | 2 KNOWN_ISSUE: "an E-stop pressed within 1 s after a release still stops the motor", "an E-stop pressed within 1 s after start stops the motor" (`test_ffbhidmain.cpp`) |
| 41 | flash_helpers | `flash_helpers.cpp:298-326` `OTP_Write`/`OTP_Read` | Bound check `adr > FLASH_OTP_END` lets offsets 64/65 reach the OTP lock bytes (0x1FFF7A00..0x1FFF7A0F): `OTP_Write(64, x)` would permanently lock OTP blocks | 3 KNOWN_ISSUE (`OTP_Write` at offsets 64 and 65, `OTP_Read` at 64) in `test_flash_helpers.cpp`; the write programs the lock bytes |
| V5 | ST EEPROM emulation (**vendor, tier A**) | `eeprom.c:333-356` `EE_VerifyPageFullyErased` | Loop `while (Address <= PAGE0_END_ADDRESS)` for any page: for page 1 it never runs and reports "erased"; it also checks only every other half-word. Page 1 is never erased by `EE_Format` / the `EE_Init` RECEIVE/ERASED, VALID/ERASED branches; a VALID page 1 left over then receives writes that `Flash_Read` cannot find (silent loss). Confirmed in source by the main agent | KNOWN_ISSUE at the firmware layer: "a variable written after Flash_Format following a page transfer is read back" (`test_flash_helpers.cpp`); 6 direct KNOWN_ISSUE in `test_eeprom.cpp` (page 1 never erased by format/init; writes lost after format; any power loss during a page 1 -> page 0 transfer makes later writes vanish; old page 1 records reappear; page 0 checked only every other half-word, so a 0xFFFF-valued leftover record comes back). The page 0 -> page 1 direction is not affected |
| V6 | ST EEPROM emulation (**vendor, tier A**) | `eeprom.c:286` then `296` (and `177` then `187`) `EE_Init` recovery | The recovery of an interrupted transfer marks the new page VALID **before** erasing the old page (`EE_PageTransfer` does the safe order: erase `715`, then VALID `723`). A power loss during that erase that leaves the old header VALID gives two VALID pages; the next `EE_Init` formats (`233`): **every variable lost**. The 177/187 branch is hidden behind V5. Confirmed in source by the main agent | KNOWN_ISSUE "a power loss when the EE_Init recovery starts to erase page 0 keeps every variable at its old or new value", the full-erase-cut recovery loop (to page 1) and, with V5, the recovery loop to page 0 (`eeprom.c:177/187`) (`test_eeprom.cpp`). Review p3r1 fix experiments: with V5 and V6 fixed all 10 eeprom KNOWN_ISSUE cases pass and no passing test fails |
| 42 | global_callbacks | `global_callbacks.cpp:365` `tud_suspend_cb()` vs TinyUSB `usbd.h:162` / `usbd.c:659` `tud_suspend_cb(bool)` | **USB bus suspend never reaches the main class**: the firmware defines a parameterless overload, TinyUSB keeps calling its own empty weak default (`usbd.c:77`). On a bus suspend (e.g. PC sleeps with the wheel connected) `FFBHIDMain::usbSuspend` is never called: FFB not stopped, effects not reset, axes not suspended (only unmount does it). Mismatch since TinyUSB update `42bf907a` (2021-02-22). Probably safety relevant. Confirmed in source by the main agent | KNOWN_ISSUE "a USB bus suspend reported by TinyUSB suspends the main class" (`test_global_callbacks.cpp`, weak stand-in for the TinyUSB default) |
| 43 | global_callbacks | `global_callbacks.cpp:340` `tud_hid_report_complete_cb(..., uint8_t len)` vs `hid_device.h:146` (`uint16_t len`) | Same kind of mismatch: TinyUSB never calls the firmware version. No effect today (both `transferComplete` implementations are empty) | Comment only (nothing observable) |
| 44 | USBdevice | `USBdevice.cpp:87` `std::min<uint8_t>(31, field.length())` | Truncates the length to 8 bits before comparing: a 256-character string becomes 0 characters (latent, no current string that long) | KNOWN_ISSUE "a string of 256 characters is cut to 31 characters" |
| 45 | UART / global_callbacks | no `HAL_UART_ErrorCallback` | On an overrun or any error during DMA/IT reception the ST HAL aborts the reception (`UART_EndRxTransfer`, `stm32f4xx_hal_uart.c:2410-2457`) and calls only the weak no-op: `UARTPort` never re-arms, the UART command interface stops receiving | Code reading; not testable until the UART fake can inject errors (P5/P7) |
| 46 | usb_descriptors | `usb_descriptors.cpp:25` | `bDeviceClass` 0 with subclass 2 / protocol 1: USB 2.0 9.6.1 requires subclass 0 when the class is 0, the IAD ECN requires class 0xEF (the comment at line 24 states the IAD intent) | KNOWN_ISSUE (`test_usb_descriptors.cpp`) |
| 47 | USBdevice | `USBdevice.cpp:81` vs `USBdevice.h:24` | Interface strings get the first 4 decimal digits of UID word 0 ("(4259)"); the header documents the last n characters of the serial ("(0032)") (was NEEDS REVIEW 62) | KNOWN_ISSUE (`test_usb_descriptors.cpp`) |
| 48 | ledEffects | `ledEffects.cpp:85` | `HAL_GetTick() > tick + period` is not wrap-safe: a pulse started 16 ms before the 32-bit tick wraps ends after 1 ms | KNOWN_ISSUE (`test_led_effects.cpp`) |
| 49 | voltagesense | `voltagesense.cpp:78/83` | **Safety:** `brakeActiveTime == 0` means "not braking": a brake activated at tick 0 or after the 49.7-day wrap is never shut down by the 5 s stuck-on protection | KNOWN_ISSUE (`test_voltagesense.cpp`) |
| 50 | voltagesense | `voltagesense.h:28` vs `voltagesense.cpp:40` | `setupBrakePin(uint32_t, ...)` declared, `(int32_t, ...)` defined: any call through the header fails to link | Build (test declares the defined signature) |
| 51 | usb descriptors | `usb_hid_ffb_desc.h:757`, `usb_descriptors.h`, `FFBWheel.cpp:40` | The 1-axis 32-bit build (`FFBWHEEL_USE_1AXIS_DESC` + `HIDAXISRES_USE_32B_DESC`) does not compile: wrong size macro name, `usb_cdc_hid_conf_1axis_32b` not declared | Build (not a shipped configuration) |
| 52 | voltagesense | `voltagesense.cpp:24, 85-88` | A brake-resistor shutdown (stuck-on protection) does not report the defined critical error "Brake resistor stuck on" (the `addError` call is commented out, apparently because of ISR context): over-voltage protection stays off until reboot without indication (was NEEDS REVIEW 73) | KNOWN_ISSUE "a brake resistor shutdown reports the critical error Brake resistor stuck on" (`test_voltagesense.cpp`) |
| 53 | I2CHandler | `I2CHandler.cpp:15-17` | `~I2CHandler` calls `addCallbackHandler` instead of `removeCallbackHandler`: a deleted handler stays in the list and the next I2C callback calls a dead object. Confirmed in source | KNOWN_ISSUE "a destroyed I2C handler is removed from the I2C handler list" (`test_input_setup.cpp`); the fixture restores the handler list |
| 54 | CanButtons | `CanButtons.h:50` with the constructor | `mask` defaults to 0xff while `btnnum` is 32: without a stored config only 8 of the 32 reported buttons work. Confirmed in source | KNOWN_ISSUE "without a stored configuration all 32 reported buttons are received" |
| 55 | SPIButtons | `SPIButtons.h:81` `spi_buf[4]`, `SPIButtons.cpp:125-129, 174` | `bytes` can be up to 9 (TM mode above 29 buttons, PISOSR above 32): the DMA writes past the buffer and the `memcpy` reads past it. In `SPI_Buttons_1/2` the overflow lands in 5 bytes of tail padding (host and target), so it is silent today; any derived class whose members reuse the padding is corrupted | KNOWN_ISSUE_UB "a read of 8 bytes stays inside the receive buffer of the source" (derived class with a guard array) |
| 56 | SPIButtons | `setSpiSpeed` / `setConfig` (`SPIButtons.cpp:139/141, 274, 115`) | Clip to `speedPresets.size()` allows index 3 of a 3-entry array (command and stored `spi_speed`): out-of-bounds read (same class as bug 37) | 2 skipped KNOWN ISSUE (crash) cases |
| 57 | SPIDevice | destructor | Does not clear the port's `current_device`: deleting a source while its DMA read is pending (e.g. a `btntypes` change) makes the completion call a deleted object (the fixture clears it) | P6 |
| 58 | I2C | `I2C.cpp` `_IT`/`_DMA` wrappers | Take the semaphore before the HAL call and never release it when the HAL returns an error: the port stays blocked (like bug 16). `transmitMasterDMA`/`receiveMasterDMA` always fail on the F407 (no DMA stream, `DMA_PARAM`) | P6 |
| 59 | CanAnalog | `CanAnalog.cpp:110-121` | `extId` never checked: an extended frame whose low 11 bits equal the CAN id (accepted by another filter on the port) becomes axis values | KNOWN_ISSUE (`test_can_analog.cpp`) |
| 60 | CanAnalog | `CanAnalog.cpp:97` | `canid` limit `0x7FF - axes/4` instead of `0x7FF - (packets-1)`: with 8 axes 0x7FE is refused (0x7FD stored), with 4 axes 0x7FF is refused | KNOWN_ISSUE |
| 61 | CanAnalog | `amount` command | Does not re-check the id: `canid` 0x7FF then amount 8 needs a second frame id 0x800, axes 5-8 can never be received | KNOWN_ISSUE |
| 62 | ADS111X | `ADS111X.cpp:24-29` `readReg` | Returns the bytes in bus order without swapping (0x8583 reads as 0x8385). Latent: no caller | KNOWN_ISSUE |
| 63 | AnalogAxisProcessing | `AnalogAxisProcessing.cpp:174/187` | `min`/`max` check `adr <= axisAmount`: adr == axisAmount reads/writes one past the vector (reachable by a command; aborts under `_GLIBCXX_ASSERTIONS`). Confirmed in source | Skipped KNOWN ISSUE (crash) |
| 64 | AnalogAxisProcessing | `AnalogAxisProcessing.cpp:44-48` | Constructor dereferences a null `cmdHandler` for `values`/`rawval` although nullptr is documented to disable commands (SIGSEGV). Latent | Skipped KNOWN ISSUE (crash) |
| 65 | ADS111X | `ADS111X.cpp:336-341` with `:296` | `setAxes` (`inputs`/`diff` commands) during a conversion read: `rawbuf[lastAxis]` written out of bounds (host abort; on the target into vector spare capacity) | Skipped KNOWN ISSUE (crash) |
| 66 | ADS111X | `ADS111X.cpp:31-40` `writeRegIT`/`writeReg` | Write the shared `writeItBuffer` before taking the port semaphore: a second write overwrites a pending IT transfer's bytes (the real HAL reads the buffer byte by byte in the ISR; the fake copies at start, so not observable). Latent: `setThresh` not called | Code reading |
| 67 | SPIButtons / SPI | `SPIButtons.cpp:29-31` with `SPI.cpp:220-222`, constructor `SPIButtons.cpp:48` | `SPI_Buttons_1::isCreatable` uses `hasFreePins`, which counts all CS pins, not the free ones: offered with no free pin, then the constructor reads `getFreeCsPins()[0]` of an empty vector | KNOWN_ISSUE (`test_spi_buttons.cpp`) |
| 68 | SPIButtons | `SPIButtons.cpp:94-100` | `setConfig` selects the stored CS number without checking it is free (TODO at 46): two devices drive one CS pin; deleting the buttons frees the other device's pin | KNOWN_ISSUE |
| 69 | SPIButtons | `SPIButtons.cpp:91, 278` | btnnum 0 is accepted but saved as 63 and restores as 64 buttons (CanButtons/PCF8574 limit to 1..64) | KNOWN_ISSUE |
| 70 | ShifterAnalog | `ShifterAnalog.cpp:243-246, 55-56` | `xchan`/`ychan` unchecked: channel 0 reads `ADC1_BUF[-1]`, channels >= 9 read past the buffer | KNOWN_ISSUE (the OOB read itself not executed) |
| 71 | ShifterAnalog | `ShifterAnalog.cpp:241-242` | `cspin` command only stores the value; `setCSPin` is dead code, the G27 mode always uses the first free CS pin | KNOWN_ISSUE |
| 72 | CanButtons | `CanButtons.cpp:137-138` | Extended (29-bit) frames whose low 11 bits equal the CAN id set the buttons (`extId` not checked; same as bug 59) | KNOWN_ISSUE |
| 73 | SelectableInputs | `SelectableInputs.cpp:95` | Button offset shift of 64 or more is UB: with 64 previous buttons x86 ORs the next source onto buttons 1-8, ARM gives 0; the `uint8_t` shift also wraps above 255 (host-vs-target) | KNOWN_ISSUE_UB |
| 74 | I2C | `I2C.cpp:49` with `I2C.h:37` | `I2CPort::setSpeedPreset` clips to `size()`: preset 2 reads `presets[2]` of a 2-element span (reachable via PCF8574 `speed=2`; same pattern as 37/56) | Skipped KNOWN ISSUE (crash) |
| 75 | SPIButtons | `SPIButtons.cpp:104, 110`, command `219-224` | `btncut` set has no effect: `setConfig` always applies the mode preset's `cutRight` (was NEEDS REVIEW 79; the command exists to change it) | KNOWN_ISSUE "btncut set cuts the extra bits of 74xx165 buttons on the right" |
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
| Axis | `Axis.cpp:1026, 1347` call `abs()` on floats; the float overload comes only through a transitive `<math.h>` (`TMC4671.h -> SPI.h`). `ffb_target_tests` compiles `Axis.cpp` with `-include math.h` to match the target (was bug 34) | `test_axis_torque.cpp` expo cases |
| Axis | `getLastScaledEnc` out-of-range float->int: host INT_MIN, ARM saturates (bug 36) | `KNOWN_ISSUE_UB` case |
| flash | `ffb_flash_tests` maps memory at the target addresses (0x08000000, 0x1FFF0000); 32-bit addresses become pointers (host warnings). Fake flash is instantaneous (no BSY/timing); partial erase / torn programs are a deterministic model of an indeterminate real state. Linux only verified; Windows untested; not built on macOS (`__PAGEZERO`, decision 2026-10-24) | `ffb_flash_tests` |
| SystemCommands | `NVIC_SystemReset`/`RebootDFU` return on the host: `reboot`, `dfu` and a main class change finish with OK on the host (no reply on the target). `taskstats`/`tasklist`/`otp` are registered on the F407 but not compiled in `ffb_target_tests`. `uid` info `%08lx` with `uint32_t` (host type mismatch, like bug 3). **`mallinfo()` may be missing on MinGW: `ffb_target_tests` (compiles `SystemCommands.cpp` since P2) may not build on Windows - unverified** | `test_system_commands.cpp` |
| cppmain | On the target `mainclass` is in NOLOAD `.ccmram` (`STM32F407VGTX_FLASH.ld:150`) and not zeroed before `cppmain` sets it; on the host it starts as nullptr (null checks such as `tud_cdc_rx_cb`'s only help after cppmain ran) | `test_global_callbacks.cpp` |
| portability | `ffb_target_tests` relies on weak-symbol overriding (test stand-ins for TinyUSB weak defaults, main.c's strong `HAL_TIM_PeriodElapsedCallback`) and on the MinGW `mallinfo` shim: both unverified on MinGW/PE; `host_cppmain.cpp` `#define`s `malloc`/`free` (formally reserved, works on GCC/Clang, guarded by a configure-time include check) | `ffb_target_tests` on Windows |
| voltagesense | `VREFINT_CAL_VREF` / `TEMPSENSOR_CAL_VREFANALOG` are `3300UL`: 64-bit on the LP64 host, 32-bit on ARM; `data * ADC_INTREF_VOL` overflows only on the target and only for a VREFINT reading below about 5 counts (not reached by tests) | - |
| inputs | CanButtons reads `msg.data` as `uint64_t` (4-byte aligned `CAN_rx_msg`: fine for LDRD on Cortex-M4, tests use `alignas(8)` for UBSan); 64-bit button states shared between ISR and thread can tear on the 32-bit target; shift by >= 64 differs x86/ARM (bug 73); bug 55 padding layout identical on host and target | P5 button tests |
| all target tests | `ffb_target_tests` uses `-funsigned-char` like ARM; `ffb_unit_tests` keeps signed `char` | char-dependent code |
| ringbuffer | `size_t` 64-bit host / 32-bit target with `uint32_t` temporaries; same results for tested sizes | `test_ringbuffer.cpp` |

## Review outcomes (Phase 3)

| Module | Reviewer | Findings | Fixed? |
|---|---|---|---|
| P1 A command (`test_cmdparser_gaps`, `test_command_handler_gaps`, `test_hid_command_interface_gaps`) | r1 (fresh agent, 2026-10-22) | must: placeholder test asserted the exact flags of bug 13 unflagged; should: bug 13 test also failed for the line 536 defect (now bug 21, own KNOWN_ISSUE), missing NEEDS REVIEW comment; nits: comments. All 4 KNOWN_ISSUE claims confirmed, first failing assertion correct; random order and single runs green | yes (2026-10-23) |
| P1 B ffb (`test_effects_calculator_gaps`, `test_hid_ffb_gaps`) | r1 | should: "HardFault on target" wording (bug 12), the 180 deg square test passes by coincidence (rename + comment as mutation guard); nits: weak sign check, comments on dead monitoring code | yes (2026-10-23) |
| P1 C errors/storage | r2 | must: 3 tests asserted slot positions that exist only because of bug 7 (a plausible fix broke them); should: ISR vs thread resume not distinguishable (fake extension); nit: raw slot write. KNOWN_ISSUEs confirmed; vTaskSuspend fake change realistic; no static leakage | yes (2026-10-23) |
| P1 D filters/small | r2 | should: coefficient tolerance effectively absolute (small coefficients checked to ~20 %), vendor double delete in `~Thread` (V4); nits: comments on IEEE divide by zero, default Biquad UB note, fake-only assertion. Filters KNOWN_ISSUEs confirmed (arm_cos_f32(PI/2) == 0 exactly on host) | yes (2026-10-23) |
| P2 batch 0 infra (CMake block, target fakes, fixtures, smoke tests) | p2r4 (fresh agent, 2026-10-23) | HAL fakes match the ST HAL (pins/constants identical to the target `main.h`); should: smoke e-stop test satisfied by the mock, incomplete resets (startup flag, ADC buffers, CAN mock + `portUsers`, port objects), `triggerExti` edge semantics, README section; nits: TIM stop mask, DFU resets, stub comments, guard inside KNOWN_ISSUE tests, empty sanitizer exclusion list. `USBdevice.cpp:33-38` hardware-only exclusion **rejected** (thread logic, now tested) | yes (2026-10-23, batch 0 agent; plus `-include math.h` for `Axis.cpp`) |
| P2 A axis torque (`test_axis_torque.cpp`) | p2r1 | must: bug 34 is host-only (withdrawn, `-include math.h`); should: bug 33 also after USB suspend/resume (new KNOWN_ISSUE), e-stop order not checked, build-specific speed limiter gain tests need `characterization:`; nits: namespace, KNOWN_ISSUE texts. Bugs 30, 33 fixes in scratch flip only their KNOWN_ISSUE cases | yes (2026-10-23, authors; `41112d85`) |
| P2 B axis config (`test_axis_config.cpp`) | p2r1 | must: characterization asserted power 40000 is kept (locks in bug 32); should: "F407 offers only the None driver" name | yes (2026-10-23, authors; `41112d85`) |
| P2 C axes + integration | p2r2 | must: 2 tests depend on baseline known issue "stop all effects" (FFB off asserted / used to switch FFB off); should: square wave start sign without marker, None-encoder test without marker, constant-force sign not tied to PID semantics, fade comments; nits | yes (2026-10-23, authors; `41112d85`) |
| P2 D main classes | p2r3 | must: E-stop edge lost within the debounce (new bug 40, safety), test treating it as intended; should: weak "reports continue" check, bug 39 test covers only half, usbResume leaves drivers in estop (NEEDS REVIEW 38), empty CDC read test (needs the new CDC rx queue); nits. Bugs 37-39 confirmed, DELETES_FFBHIDMAIN applied exactly where needed | yes (2026-10-23, authors; `41112d85`) |
| P3 A eeprom.c (`test_eeprom.cpp`) | p3r1 (fresh agent, 2026-10-24) | must: 2 passing subcases locked in V6 (unsafe order); should: recovery loop avoided the V6 cut points (KNOWN_ISSUE twin with all erase cuts), no recovery coverage for page 1 -> page 0 (KNOWN_ISSUE V5+V6), V5 after formatting two receiving pages (KNOWN_ISSUE), unmodelled-access check lost between loop iterations; nits. V5/V6 confirmed; characterizations 41, 49-52 are real AN3969 limits | yes (2026-10-24, author; `8b178bbf`) |
| P3 B flash helpers + batch 0 flash infrastructure | p3r2 (fresh agent, 2026-10-24) | should: OTP second-word result untested, unmodelled-access check lost between loop iterations, two hardware assumptions stated as fact, NEEDS REVIEW tags missing/wrong; nits: header path via `__FILE__`, regex, CANANALOG case, `read()` helper, `reboot()` RAM state, fake deviations (alignment, busy, unexpected sectors while locked), Windows include order. Fake HAL FLASH follows the ST sources step by step; header values match; guards keep the other executables unchanged | yes (2026-10-24, author) |
| P4 A system commands (`test_system_commands.cpp`) | p4r1 (fresh agent, 2026-10-25) | should: 2 of 3 access-flag tests passed without the flag check, NR 58 test did not show the claim, `lsmain` visibility untested, `signature` only failure path (needs OTP model), missing `flashraw`/`debug` boundaries, MinGW `mallinfo`; nits. Bug 7 / NR 54/57/58 fixes flip only their cases | yes (2026-10-25; `e51bc104`, infra `90554aaa`) |
| P4 B callbacks/start-up + P4 batch 0 infrastructure | p4r2 (fresh agent, 2026-10-25) | must: "CDC(4259)" characterization locked in bug 47; fixing bug 42 (and 43) would break the link (tests declared the wrong signatures); should/low: malloc forwarding unverifiable, IWDG prescaler, include-list guard for the wrapper, `mainclass` not zeroed on target, weak-symbol portability, `micros()` race (NR 65). Fakes match the ST HAL/`main.c`; guards keep other executables unchanged | yes (2026-10-25; `c00b7e2b`, infra `90554aaa`) |
| P4 C LEDs, voltage, descriptors | p4r3 (fresh agent, 2026-10-25) | should: 5 LED tests locked in NR 70, the bug 48 test carried NR 70's +1 ms, `wTotalLength` tautology, missing `REQUIRE_ALL_HAL_MODELLED`, NR 75 unmarked, NR 73 really a bug; nits. Descriptor parser independent and correct (verified with a separate Python parser), C and C++ builds of the tables byte-identical | yes (2026-10-25) |
| P5 A buttons (6 files) | p5r1 (fresh agent, 2026-10-26) | should: bug 69 test tied to one fix strategy, PCF8574 completion guard only half tested, all `SPIButtons.cpp` citations 111 lines off, NR 79 really a bug (now 75), bug 71 check accepted -1, G27 sequential untested, NR 83 behaviour unrecorded; nit: NR 80 citation | yes (2026-10-26, authors; `2a18521a`) |
| P5 B analog (5 files) + P5 batch 0 infrastructure | p5r2 (fresh agent, 2026-10-26) | HAL SPI/I2C fakes match the ST HAL; member-pointer reset sound C++20. should: tx bytes copied at start (hid bug 66), CAN standard filter too strict, unchecked C-style cast, NR 86 margin lock-ins, NR 91 test incomplete, destructor not proven, CanAnalog odd-id mask unrecorded, wrong speed preset test; nits: helper docs, FMA, includes, stale comments | yes (2026-10-26, batch 0 agent `2a18521a`; authors, P5 final) |
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
7. ~~macOS / Windows for `ffb_flash_tests`~~: answered 2026-10-24, skip `ffb_flash_tests` on macOS (done in CMake). Windows (`VirtualAlloc` at the fixed address) is still untested.
4. ~~Commit author name~~: answered 2026-10-22, `MadBull.Ben` for all commits.
5. **CI:** no GitHub Actions workflow runs the host tests (P0 finding). Should I describe a workflow job (configure, build, `ctest` with DSP on/off and sanitizers, MinGW) for you to add? I will not touch `.github/`.
6. **Second checkout:** `Testing2/OpenFFBoard` (branch `feature/host-unit-tests`) holds untracked copies of the v2 files and none of the add-test-suite work. Work continues only in `testing/OpenFFBoard` unless you say otherwise.

## Resume instructions

Start a new Claude Code session from the repo root and say:

> Read CLAUDE.md, Firmware/Tests/README.md, Tests/TEST_PLAN.md and Tests/PROGRESS.md. Resume from "Next action". Do not redo completed work, and do not move past an approval gate without my go-ahead.
