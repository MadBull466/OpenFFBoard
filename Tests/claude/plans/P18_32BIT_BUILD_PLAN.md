# P18 plan: 32-bit build of the host test suite

Status: **approved by the owner 2026-11-11 with option 62 B**; 63 as recommended (the two crashing cases are excluded only in the 32-bit build). Lead's choice within 62 B: only the 5 `__int128` files are left out of the 32-bit `ffb_dsp_tests`, the rest of the DSP suite is built and run. T1-T4 approved, committed (`544b4162`) and pushed 2026-11-11.

## 1. Why

The host is 64-bit (LP64). The target, the STM32F407, is 32-bit ARM (ILP32): `size_t`, `long` and pointers are 4 bytes there. Code that mixes these types with `int64_t` or `uint32_t` can behave differently on the target, and the 64-bit suite cannot show that. GCC on this machine builds and links `-m32` (i386) code, so the whole suite can run as 32-bit code.

## 2. Trial result (2026-11-11, nothing committed)

Two out-of-tree trial builds, `../ffb-test-build/m32-trial` (`-m32`) and `../ffb-test-build/m32-sse-trial` (`-m32 -msse2 -mfpmath=sse -funsigned-char`), GCC 16, DSP on, default exclusions:

| Executable | 32-bit result | 64-bit gate build `p17t2g-default`, same exclusions |
|---|---|---|
| `ffb_unit_tests` | 654 passed, 4 skipped, assertions 7292 / 67 failed (the `KNOWN_ISSUE` ones) | identical |
| `ffb_target_tests` | **aborts** in 2 cases (below); with those 2 excluded 1830 passed, 20 skipped, assertions 40337 / 155 | identical with the same 2 excluded |
| `ffb_flash_tests` | 159 passed, assertions 8503 / 289 | identical |
| `ffb_rtos_tests` | 278 passed, assertions 3577 / 24 | identical |
| `ffb_usb_tests` | 606 passed, assertions 9616 / 54 | identical |
| `ffb_driver_tests` | 1717 passed, 9 skipped, assertions 17421 / 167 | identical |
| `ffb_dsp_tests` | **does not compile**: 5 test files (`test_dsp_filt_fir_q.cpp`, `_lattice`, `_lms`, `_sparse`, `test_dsp_stats_q31.cpp`) use `__int128` in their reference models, which i386 GCC does not have. Firmware and CMSIS-DSP sources compile | 2485 cases |
| Python | passes (not affected) | |

Both flag sets gave the same results. The identical assertion counts show that, apart from the crash below, no plain or `KNOWN_ISSUE` case behaves differently on 32-bit.

### Candidate firmware bug 225 (new, found by the trial)

`ParsedCommand::adr` is `int64_t` (`CommandHandler.h:70`). `SerialFFB::command` checks addresses with `cmd.adr < effects.size()` (10 places in `SerialFFB.cpp`, e.g. `:157`, `:164`, `:179`, `:186`, `:193`, `:201`, `:208`, `:220`).

- **64-bit host:** `size_t` is `unsigned long`, the same rank as `int64_t`, so `-1` is converted to `2^64-1` and the check rejects it. The existing cases `mag rejects reading an address beyond the effects` and `state rejects an address beyond the effects` pass.
- **32-bit (and so on ARM):** `size_t` is `unsigned int`. It is converted to `int64_t`, so `-1 < 40` is true and `effects[-1]` is used. That reads, and for `period`, `duration`, `offset`, `deadzone`, `sat`, `coeff`, `axisgain` and `state` writes, memory before the effects array.
- On the host, `_GLIBCXX_ASSERTIONS` aborts the run (`std::array::operator[]: __n < this->size()`). On the target, it is a silent out-of-bounds access, reachable with a single serial command such as `fxm.mag?-1`, if the parser accepts a negative address. That needs a check in T3.

The existing test was correct; the 64-bit host simply hid the defect.

Fix, as a suggestion for the firmware owner, not applied: `cmd.adr >= 0 && cmd.adr < (int64_t)effects.size()`. The cast `(size_t)cmd.adr < effects.size()` is **not** a fix on ILP32: it truncates `2^32` to 0 and accepts it as slot 0. The T2 reviewer's mutants 22 and 23 confirmed both points.

### T1 result (2026-11-11, approved and committed)

- `FFB_TESTS_32BIT` option, `FFB_TARGET_TESTS_EXCLUDE_32BIT` list (the 2 bug 225 cases), the 5 `__int128` DSP files are left out in 32-bit builds. `tools/run_all_tests.py` reads the 32-bit exclusion list.
- Gates, fresh builds `../ffb-test-build/p18t1-<variant>`: m32, default, san, clang, cov ctest 11 / 11; nodsp 10 / 10; `check_baseline.py` OK in all six.
- m32 counts (with ctest exclusions): unit 654 (4 skipped), target 1830 (20 skipped, 2 excluded), flash 159, rtos 278, usb 606, driver 1717 (9 skipped), dsp 1871. Case and assertion counts are identical to the default 64-bit gate build for every executable, with the same cases left out.
- DSP: 6 files contribute no cases on 32 bit, the 5 left out by CMake and `test_dsp_filt_conv_q31.cpp`, which guards itself with `#if defined(__SIZEOF_INT128__)`. 64-bit `ffb_dsp_tests` with those 6 files excluded (`--source-file-exclude`): 1871 cases, 401311 assertions, 319 failed (`KNOWN_ISSUE`). Identical to m32.
- 64-bit variants are unchanged: the option defaults to `OFF`, and the default build's target exclusion list is still empty.

### T2 result (2026-11-11, approved and committed)

- Lead infrastructure:
  - `support/KnownIssue32.h` adds two markers: `KNOWN_ISSUE_32BIT` (should_fail on ILP32) and `KNOWN_ISSUE_32BIT_UB` (skipped on ILP32). Both are plain tests on LP64.
  - `tools/run_all_tests.py` classifies them: passed in 64-bit builds, a known bug in 32-bit builds. Tests for that are in `claude/python/test_run_all_tests_ilp32.py` (10 cases).
- Work order `Tests/claude/charters/P18_WORK_ORDERS.md`, writer (Sonnet): `claude/cases_target/test_serial_ffb_32bit.cpp`, suite `SerialFFB32`, 31 cases.
  - 18 `KNOWN_ISSUE_32BIT_UB` cases: ten getat and seven setat cases with addresses -1, -40, -41 and INT64_MIN, plus one end-to-end case `fxm.0.type?-1` over CDC.
  - 1 `KNOWN_ISSUE_32BIT` case: `state` setat with -1, -40, -41, -216 and INT64_MIN, status checked first.
  - 12 plain cases: 2^32 and 2^32+1, the 39 / 40 boundaries for get and set, and controls (type, state, set-then-get on slot 1 with instance-0 and instance-1 fields).
- Review (Sonnet, fresh), 42 mutants:
  - Gaps F1, F2, F4 and F5 were closed by the writer.
  - My follow-up check found one more survivor, `sat` writing `conditions[1]` (M31). Also closed: the mutant now fails a case.
  - The plan's suggested fix in section 2 was corrected (F3).
  - Accepted weakness: on ILP32 a mutant `setEffectState` guard `>` turns -216 into an `effects[40]` abort inside the `state` should_fail case. doctest counts that crash as "failed as expected" and ends the run, so ctest stays green. `run_all_tests.py` continues after a crash and shows it. This guard is reachable only on ILP32.
- Gates (fresh builds `../ffb-test-build/p18t2f-<variant>`): m32, default, san, clang and cov ctest 11 / 11; nodsp 10 / 10; `check_baseline.py` OK in all six.
  - `ffb_target_tests` with ctest exclusions: default 1863 (1832 + 31), 18 skipped; clang 1863; san 1747; m32 1843 (1830 + 13), 38 skipped (20 + 18 `_UB`).
  - Python `claude/python` 125 (115 + 10).
- Bug 225 is not yet in `PROGRESS.md`; that happens in T4.

### T3 result (2026-11-11, approved and committed)

**Method (lead).** Every distinct firmware compile command of the gate builds `p18t2f-default` (LP64) and `p18t2f-m32` (ILP32) was recompiled with `-fsyntax-only`: 586 commands, all executables and their feature macros, including vendor code. The warnings of the two widths were diffed by file, line and warning flag. A warning on one width only marks code whose types change with the data model.
- `-Wsign-compare`: LP64 30 warnings, ILP32 13. Only on LP64 (signed on ILP32, so behaviour changes): the 10 `cmd.adr < effects.size()` guards of `SerialFFB.cpp` (bug 225), plus host artefacts (below). Only on ILP32: none.
- `-Wconversion -Wsign-conversion -Wshift-overflow=2 -Woverflow`: LP64 1397, ILP32 1364. Only on ILP32: the `effects[cmd.adr]` index conversions of `SerialFFB.cpp` (bug 225), `AnalogAxisProcessing.cpp:176/189` (index after an `adr >= 0` check: no width issue; the `<= axisAmount` off-by-one there is the known bug 63, same on both widths), and `size_t` to `int` in `CmdParser.cpp:80/158` and `EffectsCalculator.cpp:117`, which narrows on both widths and only changes the warning class.

**Host artefacts, not target defects.** These appear only on LP64:
- `size_t` to `uint32_t` narrowing in `CmdParser.cpp`, `CDCcomm.cpp`, `ringbuffer.c`, `flash_helpers.cpp`, `CanBridge.cpp`, `VescCAN.cpp:523` (a no-op on ILP32);
- integer-to-pointer casts of 32-bit flash addresses in `eeprom.c` and `flash_helpers.cpp`;
- `%llx` with `uint64_t` (`SystemCommands.cpp:344`).

**Vendor.** In `arm_vlog_q31.c:118` the shift `((int64_t)tmp * LOG_Q31_INVLOG2EXP) >> 31` is unsigned on LP64 and signed on ILP32. The result is truncated to 32 bits, which keeps bits 31-62 either way, so it is equivalent. The m32 DSP FastMath counts are identical to LP64.

**Other checks.**
- `long` is used only in `getRunTimeCounterValue` (returns the 32-bit `micros()`).
- Pointer-size use: only the alignment test `(uintptr_t)p % 4` in `ringbuffer.c`, which is width-independent.

**Result.** No new candidates: bug 225 is the only width-dependent behaviour the compiler shows. No writer wave was needed and no test file changed in T3. Not covered by this method: overflow of `size_t` / `long` arithmetic that no warning flags, and struct layout (proposal 3).

The diff script used was a scratch tool and is not part of the suite.

## 3. Limits: i386 is not ARM

The 32-bit build shows ILP32 effects only. It does not reproduce:

- `uint32_t` / `int32_t` are `unsigned long` / `long` in ARM newlib, but `unsigned int` / `int` in i386 glibc. Sizes are equal, but overload resolution and `printf` formats differ: the trial's `-Wformat` warnings in `CommandHandler.cpp:136`, `SystemCommands.cpp:307` and `USBdevice.cpp:45` are i386-only and not target bugs.
- Struct layout: i386 aligns `int64_t` / `double` members to 4 bytes, ARM AAPCS to 8. A 32-bit build therefore does **not** prove struct sizes or offsets. That is the separate `arm-none-eabi-gcc` layout check (proposal 3 of 2026-11-11, not part of P18).
- Floating point: with `-msse2 -mfpmath=sse` floats are IEEE single as on the Cortex-M4F. Without it, x87 uses extended precision. Double is in hardware here and software on the target (same IEEE results). FMA contraction may differ.
- `char` is unsigned on ARM, matched with `-funsigned-char`.
- Endianness: both little endian, matched.
- Real-time behaviour, interrupts: unchanged (cooperative fake RTOS).

## 4. Work items

### T1: CMake variant (lead, no agents)
- New option `FFB_TESTS_32BIT` (default `OFF`) in `Firmware/Tests/CMakeLists.txt`. When on, it adds `-m32 -msse2 -mfpmath=sse -funsigned-char` to C and C++ compile and link flags of all test targets.
- The option fails at configure time with a clear message if a `-m32` test program does not link (no multilib), or if the compiler is MSVC or the host is macOS (no 32-bit there).
- New exclusion list `FFB_TARGET_TESTS_EXCLUDE_32BIT` with the two crashing cases, documented like the sanitizer lists (bug 225).
- `check_baseline.py`: the 318 baseline cases already pass on 32-bit, so no change is expected. Verify it in the gate.
- The `ffb_dsp_tests` question: see section 5, owner's decision.
- Gate variant `m32`: build, ctest, `check_baseline.py`, plus the existing five variants (default, san, clang, nodsp, cov) to show nothing changed on 64-bit.

### T2: bug 225 tests (1 writer, 1 reviewer)
- New file `claude/cases_target/test_serial_ffb_32bit.cpp`: for every `SerialFFB` command that takes an address, a get and a set with address `-1` (and `INT64_MIN`, `-41`) must be rejected and change no effect.
- On 64-bit these are plain passing tests. On 32-bit they are `KNOWN_ISSUE_UB`-style cases. They cannot run in-process on 32-bit, because the out-of-bounds access aborts under `_GLIBCXX_ASSERTIONS`. So they are skipped there with a decorator `KNOWN_ISSUE_32BIT(...)`, a new macro in a new support header next to `TestHelpers.h`. The macro skips when `sizeof(size_t) == 4` and adds the bug text. `run_all_tests.py` runs skipped cases in their own process, so the full report still shows the abort.
- Check whether the command parser can produce a negative `adr` from serial text (`CmdParser`), and from CAN and HID command paths. That decides how reachable bug 225 is on the target.

### T3: search for further 32-bit differences (lead plus 1 writer, 1 reviewer)
- Compile the 64-bit tree once with `-Wsign-compare -Wconversion` into a scratch directory. List every comparison between a signed 64-bit value (`int64_t`, `long`) and `size_t` / `unsigned long`: on 64-bit it is an unsigned comparison, on 32-bit a signed one. Also list the arithmetic that relies on `long` or `size_t` being 64-bit, and `sizeof` of pointers or `size_t` in data that is stored or sent. This is a lead-made list; vendor code is reported, not tested.
- For each candidate reachable through a public interface: a test that differs between the 32- and 64-bit builds, or a NEEDS REVIEW row if the difference cannot be shown.
- Findings: firmware bugs from 225, vendor bugs from V89, NEEDS REVIEW from 281.

### T4: documents
- `PROGRESS.md`: "Environment" (the `m32` variant and its configure line), P18 section, bug 225, counts.
- `COVERAGE_AND_SCOPE.md` section 3.5: what the 32-bit build does and does not show (section 3 of this plan).
- `Firmware/Tests/README.md`: the option and the `KNOWN_ISSUE_32BIT` marker.
- `OPEN_QUESTIONS.md`, `BUG_REPORT.md` (generated).
- CI is not changed: a 32-bit job needs `gcc-multilib` on the Ubuntu runner and a `.github/` edit. Proposal only, as an addition to open question 5.

Approval gates: T0 (this plan), after T1, after T2 + T3, then the commit. No push without "push".

## 5. Owner's decisions needed (open questions 62, 63)

**62. `ffb_dsp_tests` on 32-bit.** The 5 files with `__int128` reference models do not compile.
- **A (recommended):** a small exact 128-bit integer helper in `support/` (multiply, add, shift, compare on two `uint64_t` halves). It replaces `__int128` in those 5 files and is self-tested against `__int128` in 64-bit builds. On 64-bit the reference values stay bit-identical, but this edits 5 existing Claude test files.
- **B:** leave `ffb_dsp_tests` out of the 32-bit build, or leave out only the 5 files, and record it as not covered. No existing file changes, but the 32-bit DSP check is partial or missing.

**63. Bug 225 in the existing test cases.** The two crashing cases stay as they are (they describe correct behaviour) and are excluded only in the 32-bit build (recommended). The alternative is to give them a `KNOWN_ISSUE_32BIT` marker, which edits an existing file.

## 6. Not in P18
- `arm-none-eabi-gcc` struct size / offset checks (proposal 3).
- Fuzzing (proposal 4), integration tests (proposal 1).
- Any firmware change: bug 225 is reported, not fixed.
