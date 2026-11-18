# WO I0-4: CMake skeleton, smoke case and the gate script (as sent, 2026-11-18)

One `integration-test-writer` run. Files: `wo/I0-4.files` (the gate reads it; it also lists the approved I0-3 fixes, which are committed together with I0-4).

## Order

You build the skeleton of the integration test executable and the gate script of the OpenFFBoard integration track. Repository root `/home/ben/Documents/OpenFFBoard/testing/OpenFFBoard`, branch `add-test-suite`. Read first: `CLAUDE.md` (rules, "Integration track"), `Firmware/Tests/README.md`, `Tests/Integration_Tests/claude/plans/INTEGRATION_TEST_PLAN.md` section 3.2 (executable and CMake layout; it is the design you implement), `Tests/Integration_Tests/claude/plans/AGENT_PLAN.md` sections 2.3 and 2.4 (gate and evidence format; 2.4 wins where they differ), and `.claude/hooks/guard_lib.py` (the functions `fingerprint`, `blob_of_file`, `gate_dir`, `load_evidence`: the evidence you write must be readable by them; do not edit the hooks).

The working tree contains uncommitted, approved fixes of shared fakes (WO I0-3, files listed under "I0-3" in `Tests/Integration_Tests/claude/charters/wo/I0-4.files`). Leave them alone.

You may create or edit only:
- `Firmware/Tests/CMakeLists.txt`: exactly one added line, `include(Integration_Tests/integration.cmake)`, after the `ffb_driver_tests` block (after its `add_test` / `endif()`, before the `ffb_dsp_tests` section, around line 968), with a one-line comment above it. Nothing else in that file.
- `Firmware/Tests/Integration_Tests/integration.cmake`
- `Firmware/Tests/Integration_Tests/claude/cases/test_f1_setup.cpp`
- `Firmware/Tests/Integration_Tests/tools/it_gate.py`, `Firmware/Tests/Integration_Tests/tools/test_it_gate.py`

### 1. `integration.cmake`

Implement plan section 3.2 for **`ffb_integration_tests`**:
- Option `FFB_TESTS_INTEGRATION` (default ON). When OFF, nothing of this file takes effect.
- Sources: reuse the existing variables (`TARGET_FIRMWARE_SOURCES`, `USB_TINYUSB_SOURCES`, `USB_FIRMWARE_SOURCES`, `DRIVER_FIRMWARE_SOURCES`, `USB_PLATFORM_SOURCES` / `DRIVER_PLATFORM_SOURCES` as appropriate, the DSP sources) with `list(REMOVE_DUPLICATES)`; platform sources = the target set minus `host_usb.cpp`, plus `host_dcd.cpp`, `host_usb_osal.cpp`, `host_driver_timers.cpp`, `FakeTmc4671.cpp` and the other peer fakes `ffb_driver_tests` links. Test sources: glob `Integration_Tests/claude/cases/test_*.cpp` (+ `Code_Coverage/support/test_main.cpp`). Follow how `ffb_usb_tests` and `ffb_driver_tests` are built (read both blocks completely) and combine them: feature header directory `Code_Coverage/platform/target_f407_drivers` (MIDI off), definitions `HOSTTEST HOSTTEST_REAL_CPPMAIN HOSTTEST_REAL_LED_EFFECTS HOSTTEST_TARGET_OTP HOSTTEST_RTOS_EXT HOSTTEST_REAL_TINYUSB FFB_F407_TUSB_CONFIG_H` and whatever else those two blocks need for the same sources (the SimpleMotion clang rule, per-file flags such as `Axis.cpp -include math.h`, `-fexceptions` for C, `-funsigned-char`, `-Wall`, MinGW and APPLE handling, sanitizer options, `Threads::Threads`). Include order as the plan says; `Code_Coverage/fakes` is never an include directory. Where the two blocks conflict, choose, and list each choice in your answer.
- `TMCDEBUG`: the plan section 3.2 says the drivers feature header turns it on; `CLAUDE.md` says `TMCDEBUG` is on in `ffb_driver_tests` only. Leave `TMCDEBUG` **off** in `ffb_integration_tests` (the debug bridge is not part of F1) and say how you did it.
- If combining real TinyUSB with the driver sources fails to compile or link because of a genuine conflict (duplicate symbols, two definitions of a fake), stop and report it under "Lead decision needed" with the exact error. Do not edit shared code.
- ctest: `ffb_integration_tests` with cache lists `FFB_INTEGRATION_TESTS_EXCLUDE`, `FFB_INTEGRATION_TESTS_EXCLUDE_SANITIZE`, `FFB_INTEGRATION_TESTS_EXCLUDE_32BIT` (default empty), printed at configure time like the other executables. A ctest entry `integration_tool_tests` (only when Python 3 is found, like `python_helper_tests`) running `test_it_gate.py` and `test_hooks.py` with `python3 -B`.
- The other executables must keep byte-identical compile commands (see the gate, step 5).

### 2. `claude/cases/test_f1_setup.cpp`

One smoke case only (the fixture comes in WO I0-5): it resets `HostPlatform` and `HostTarget`, and checks one observable fact about the linked firmware that has an oracle you cite in a comment (for example: the TinyUSB device stack is not initialised before `tud_init`; or a constant of the TMC4671 driver equal to a datasheet value). Name it as a statement. Keep it independent of test order.

### 3. `tools/it_gate.py` (Python 3 standard library) and `tools/test_it_gate.py` (`unittest`)

`python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --wo <ID> [--variants default,m32] [--full] [--files <path>]`

- `--files` default: `Tests/Integration_Tests/claude/charters/wo/<ID>.files` (one repository-relative path or glob per line, `#` comments). Missing file: GATE FAIL.
- Variants and build directories `../ffb-test-build/it-<variant>` (resolved from the repository root, never inside it): `default` (no options), `m32` (`-DFFB_TESTS_32BIT=ON`); `--full` adds `san` (`-DFFB_TESTS_SANITIZE=ON`), `clang` (`-DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++`), `clang-san`, `nodsp` (`-DFFB_TESTS_USE_DSP=OFF`), `nodsp-san`, `cov` (the coverage flags from the coverage `PROGRESS.md` "Environment").
- Per variant: (1) configure if no `CMakeCache.txt`; build with `-j<cpu count>`; a compiler warning whose file is in the WO's file list is a failure; (2) `ctest --output-on-failure`; (3) `Firmware/Tests/Code_Coverage/tools/check_baseline.py --build-dir <dir>`; (4) run `ffb_integration_tests -r=xml` and summarise it with `Firmware/Tests/Code_Coverage/tools/summarise_results.py` (counts, failures, skips); (5) compile-command check: the entries of `compile_commands.json` of every target **other than** `ffb_integration_tests` must equal a recorded reference. `--record-compile-reference <build dir> --variant <name>` writes the reference to `../ffb-test-build/it-gate/compile_reference-<variant>.json`; the gate fails if the reference for a variant is missing (message says how to record it). Decide how to recognise integration entries (e.g. object path under the target's directory) and test it.
- After all variants, scope check: `git status --porcelain --ignored` restricted to `Firmware/`: nothing outside `Firmware/Tests/`; every changed or untracked path under `Firmware/` matches the WO file list (paths outside `Firmware/` are documents or other owners' files and are not checked); no path the hooks protect (use `guard_lib.path_block_reason(rel, root)` for the lead, imported from `.claude/hooks/`) unless it is in the WO list and approved.
- Evidence: write `../ffb-test-build/it-gate/<WO>/<UTC timestamp>.json` and copy it to `latest.json`: `wo`, `result` (`PASS` / `FAIL`), `files` (`{path: git blob of the working-tree file, or null if deleted}` for every changed or untracked path under `Firmware/` matching the list, computed like `guard_lib.blob_of_file`), `fingerprint` (`guard_lib.fingerprint(files)`), `variants` (per variant: exit codes and counts of build, ctest, check_baseline, the integration summary, compile check), `scope` (result and offending paths), `git_head`. Print a short per-variant summary and, as the very last line, `GATE PASS <absolute evidence path>` or `GATE FAIL <reason>`; exit 0 / 1.
- `test_it_gate.py`: unit-test the pure parts without building anything (file-list parsing and glob matching, variant table, warning attribution to files, compile-command comparison and integration-entry filtering, scope check on a temporary git repository, evidence JSON accepted by `guard_lib.load_evidence` and matching `guard_lib.fingerprint`, last-line format). Run it with `python3 -B`.

### 4. Verification

1. Before you change `CMakeLists.txt`, record the compile references from the lead's existing build directories, which contain the I0-3 state without the integration executable: `python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --record-compile-reference /home/ben/Documents/OpenFFBoard/testing/ffb-test-build/i03-default --variant default` and the same for `i03-m32` / `m32` (configure / build directories are not to be changed by you; if their `compile_commands.json` is missing, report it).
2. Then add the `include()` line and run `python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --wo I0-4 --variants default,m32`. It configures and builds `../ffb-test-build/it-default` and `it-m32` from scratch (several minutes each).
3. Required: build without warnings in your files, ctest all passed (including `ffb_integration_tests` and `integration_tool_tests`), `check_baseline` OK, compile check equal, scope check OK, GATE PASS.
4. Write cmake build paths literally in shell commands (the hook does not expand shell variables).

Answer format: files with one-line summaries; the CMake choices you made (each conflict between the usb and driver blocks and how you resolved it); the smoke case and its oracle; it_gate.py design notes (how integration entries are recognised, how warnings are attributed); per variant the gate summary; anything open under "Lead decision needed". The last line must be the gate's last line, exactly as printed (`GATE PASS <path>` or `GATE FAIL <reason>`). If the same problem fails twice, stop and report it.
