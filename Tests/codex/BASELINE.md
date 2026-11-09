# Verified baseline (2026-11-05)

Pre-change revision: `8a8041dcd6fbc8b706383f71cbda3f638961725b`, branch `add-test-suite`. Initial tracked/untracked state clean. Only Codex documentation changed during this investigation. Builds were fresh, independent of the pre-existing build trees, and out of tree under `../ffb-test-build/codex-baseline-<variant>`.

## Environment

Linux x86-64 host; GCC/G++ and gcov 16.2.1 (20260810), Clang 23.1.4, CMake 4.4.5, Python 3.14.9; Unix Makefiles; Debug; GNU C++20/C11. 16 logical CPUs and about 30 GiB RAM available. Concurrent builds were limited to 4–6 compile jobs each. doctest 2.4.11 already vendored; CMSIS-DSP at `1ba19d14a6491169b4d2873e1a0a59486c7636c6`, ARM patterns present, f16 probes successful in both compilers. No dependencies installed/downloaded.

## Executed configuration results

Every configure and full build succeeded. CTest exit status is 8 in every variant: its only failing entry is `python_helper_tests`, whose source directory was not updated after the ownership move (CDX-001). Coverage generation succeeded independently of that CTest failure.

| Fresh variant | CTest passed/total | C++ cases executed | Ordinary passes | Failed as expected | C++ failures | Skipped/excluded | Original-case checker |
|---|---|---|---|---|---|---|---|
| `default` | 8/9 | 4328 | 4061 | 267 | 0 | 31 | Pass: 317 checked, 1 excluded |
| `san` | 8/9 | 4169 | 3928 | 241 | 0 | 190 | Pass: 315 checked, 3 excluded |
| `nodsp` | 7/8 | 4057 | 3795 | 262 | 0 | 30 | Pass: 317 checked, 1 excluded |
| `nodsp-san` | 7/8 | 3928 | 3690 | 238 | 0 | 159 | Pass: 317 checked, 1 excluded |
| `clang23` | 8/9 | 4328 | 4061 | 267 | 0 | 31 | Pass: 317 checked, 1 excluded |
| `clang23-san` | 8/9 | 4169 | 3928 | 241 | 0 | 190 | Pass: 315 checked, 3 excluded |
| `cov` | 8/9 | 4328 | 4061 | 267 | 0 | 31 | Pass: 317 checked, 1 excluded |

No ASan, UBSan or LSan diagnostic was found in the three configured sanitizer CTest logs. This applies to the executed cases with their existing skips/exclusions; known UB/crash cases were not forced with `--no-skip`. No sanitizer options were added to suppress findings.

### Per-executable counts

Cells are doctest **passed / failed / skipped**, with failed-as-expected cases included in passed. Default, Clang and coverage counts agree; GCC/Clang DSP sanitizer counts agree. No-DSP omits the standalone DSP executable and its DSP-dependent target cases.

| Executable | Default / Clang / coverage | DSP sanitizer (GCC/Clang) | No DSP | No-DSP sanitizer |
|---|---|---|---|---|
| `ffb_unit_tests` | 646 / 0 / 4 | 643 / 0 / 7 | 646 / 0 / 4 | 645 / 0 / 5 |
| `ffb_target_tests` | 1819 / 0 / 18 | 1705 / 0 / 132 | 1766 / 0 / 17 | 1653 / 0 / 130 |
| `ffb_flash_tests` | 159 / 0 / 0 | 159 / 0 / 0 | 159 / 0 / 0 | 159 / 0 / 0 |
| `ffb_rtos_tests` | 271 / 0 / 0 | 266 / 0 / 5 | 271 / 0 / 0 | 266 / 0 / 5 |
| `ffb_usb_tests` | 286 / 0 / 0 | 286 / 0 / 0 | 286 / 0 / 0 | 286 / 0 / 0 |
| `ffb_driver_tests` | 929 / 0 / 9 | 919 / 0 / 19 | 929 / 0 / 9 | 919 / 0 / 19 |
| `ffb_dsp_tests` | 218 / 0 / 0 | 191 / 0 / 27 | Not built | Not built |

The default catalogue is 4,359 C++ cases: 4,328 execute and 31 skip or are excluded. This includes 267 expected-failure cases, so the ordinary-pass count is 4,061. No assertion-correctness or effectiveness audit has been performed.

### Python executed directly

`python3 -B -m unittest discover` using `Firmware/Tests/claude/python` completed **115 tests: 113 ordinary successes and 2 expected failures**, no unexpected failures/errors/skips. Run from `/tmp`, not the repository. This does not make the configured CTest entry pass: its stale path still prevents discovery. The full-report tool Python helper was separately invoked using its configured stale directory: zero results and an ImportError problem were reproduced. The full no-skip C++ report was not run.

### Original 318-case preservation

CTest executed `tools/check_baseline.py` successfully in all seven variants. All 318 original names are preserved. Outside DSP sanitizer builds, 317 classifications are checked and the effectsForces case is excluded (result not checked). DSP sanitizer builds check 315 classifications and exclude effectsForces plus the two spline cases. Skip-decorated UB cases remain skipped as the checker expects. Do not describe this as all 318 original cases passing normally.

### Explicit exclusions and skips

- Every variant excludes `requesting the forces of a not existing axis is rejected` from `ffb_unit_tests`.
- DSP sanitizer variants additionally exclude `reconstruction of a constant signal is constant` and `reconstruction of a step never leaves the range of the input values and reaches the target`.
- Other CMake executable exclusion lists are empty; existing doctest skip decorators still apply (unknown-command crash, driver memory corruption/crash cases, UB markers and main-class destruction in sanitizer builds).
- No exclusion list or test marker was changed. Counts above include excluded cases in doctest skipped totals.

## Build warnings

Counts below are emitted log lines containing `warning:`, including repeated diagnostics from common headers and parallel compiler output; they are not unique defects. Paths were resolved before identifying diagnostics located inside test infrastructure (`Tests/../FFBoard` is firmware, not a test file).

| Variant | Warning-bearing lines | Diagnostics with a resolved source location under `Firmware/Tests/` |
|---|---|---|
| `default` | 1549 | 0 |
| `san` | 1549 | 0 |
| `nodsp` | 1545 | 0 |
| `nodsp-san` | 1545 | 0 |
| `clang23` | 11209 | 0 |
| `clang23-san` | 11210 | 0 |
| `cov` | 1549 | 0 |

GCC warning groups include anonymous C linkage typedefs, signedness comparisons, format mismatches, integer-to-pointer casts, narrowing, possible uninitialised data, missing returns, deprecated mallinfo and ambiguous comparisons. Clang additionally emits missing-override, deprecated-volatile, unknown-warning/attribute and obsolete `<ciso646>` warnings. Zero located test diagnostics does not mean a warning-free build; some Clang option warnings have no file location. Firmware/vendor warnings were recorded, not fixed or adjudicated. Full logs are retained alongside the fresh build outputs.

## Coverage measured

Fresh GCC coverage build, with only its normal CTest workload and existing exclusions:

| Scope | Covered / executable source lines | Coverage |
|---|---|---|
| Existing 87-file firmware ladder (includes tier A, excludes drivers/MIDI) | 8,007 / 8,095 | 98.9% |
| Firmware only (without tier A) | 7,237 / 7,310 | 99.0% |
| Vendor tier A | 770 / 785 | 98.1% |
| TinyUSB tier B | 1,337 / 1,572 | 85.1% |
| Original CMSIS-DSP tier C | 198 / 198 | 100.0% |
| Driver track | 2,376 / 2,596 | 91.5% |
| Standalone DSP: FastMath linked sources | 443 / 451 | 98.2% |
| Standalone DSP: BasicMath linked helpers | 62 / 62 | 100.0% |

The standalone DSP aggregate is **505/513 (98.4%) over 27 source files with data**, with **231 of 258 in-scope source files without data** in that executable. It is not 98.4% of the expanded P11 scope. No hardware-only exclusion ranges are active. Branch/condition/state-transition coverage was not measured, and covered lines do not establish meaningful assertions. Full generated output: [COVERAGE_BASELINE.md](COVERAGE_BASELINE.md).

## Historical comparison and limits

- Fresh C++ counts agree with the P9/driver/P11 results where the same configuration is described. The driver report separates ordinary and expected-failure passes (860+69=929 default; 855+64=919 sanitizer); there is no count regression.
- Older PROJECT_SUMMARY is a P9 five-executable/3,203-case/61-Python-test snapshot, not current inventory. Current Python discovery adds report/summariser helper tests, reaching 115.
- Historical CTest 9/9 does not reproduce after the latest move: every fresh variant fails Python discovery. Latest commit explicitly states gate not rerun.
- COVERAGE_AND_SCOPE lists TinyUSB 755/1,572 (48.0%) while claiming P9-final provenance. Fresh measurement matches PROGRESS P9-final 1,337/1,572 (85.1%); 48.0% was the P9 smoke-stage figure. No protected document changed.
- Firmware ladder, tier A, original tier C and driver coverage reproduce the historical totals. Firmware header coverage is now 396/416 (95.2%), versus historical 392/413; no causal attribution made during discovery.
- No Windows/MinGW, macOS, physical board, target ARM code path, real bus timing, preemption or hardware safety property was verified. No Release build, preset invocation, full no-skip report, mutation experiment or new correctness audit was performed.
- Compiler/CMake/Python versions differ slightly from historical environment notes. Existing build trees were not reused, reset or deleted.
