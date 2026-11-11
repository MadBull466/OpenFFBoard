# Firmware Test Suite Plan v2: keep the suite, cover more code

Keep every existing test in `Firmware/Tests/` and raise line coverage of the in-scope firmware from about 24% to **85% or more** (decided 2026-10-23; hardware-only code is not tested and not counted, see "Coverage goal"). Work in phases, ordered by risk and by lines gained per effort. Permanent rules are in the root `CLAUDE.md`. The two that matter most: **`Firmware/` is read-only except `Firmware/Tests/`, and never push.**

## Where we are (updated 2026-11-10)

- P0-P11 and P13-P17 are done, approved, committed and pushed to the fork. P18 (32-bit build) is approved, committed and pushed too (`544b4162`, 2026-11-11); question 64 (P18 paragraph in `CLAUDE.md` / `AGENTS.md`) is pushed as `7bb1390a`. On 2026-11-11 all test folders were moved under `Code_Coverage/` (`590779a6`); every path in this plan is the new one. The resume point is `Tests/Code_Coverage/claude/plans/PROGRESS.md`.
- Original starting point (2026-10-21, kept for the record): 318 test cases, 24 of 97 firmware source files compiled into the tests, in-scope line coverage measured in P0 at 2,136 of about 8,160 lines (26.2 %), mutation check 42 of 48.
- Today (GCC 16): in-scope firmware coverage 98.9 % (8,172 of about 8,260 lines, 2026-11-11 gcov run after the `Code_Coverage` move; 8,171 on 2026-11-10); `ffb_unit_tests` 654 / `ffb_target_tests` 1863 / `ffb_flash_tests` 159 / `ffb_rtos_tests` 278 / `ffb_usb_tests` 606 / `ffb_driver_tests` 1717 / `ffb_dsp_tests` 2485 cases, Python 125 tests, ctest 11 of 11 (2026-11-11, also in the 32-bit build). Per-file numbers: `Tests/Code_Coverage/claude/plans/COVERAGE_AND_SCOPE.md`.

## How to start or resume

1. Read `CLAUDE.md`, `Firmware/Tests/README.md`, this file and `Tests/Code_Coverage/claude/plans/PROGRESS.md`.
2. Resume from the "Next action" line in `PROGRESS.md`. Do not redo completed work.
3. Before stopping for any reason, update `PROGRESS.md` so another session can pick up cleanly.

## Rules for keeping the existing tests

1. **The baseline is the 318 existing cases.** P0 saves their names to `Firmware/Tests/Code_Coverage/baseline/test_cases.txt` (from `--list-test-cases`) and adds a stdlib-only checker, `Firmware/Tests/Code_Coverage/tools/check_baseline.py`. Every baseline case must stay present with an unchanged result: passing cases pass, `KNOWN_ISSUE` cases fail as expected, the one skipped case stays skipped.
2. **Do not edit the existing test files.** Never change an existing assertion, name or marker in `cases/`. Additive changes to shared helpers (`mocks/`, `support/`, `platform/`) are allowed only when a phase needs them, and each one is noted in `PROGRESS.md`. Never edit `third_party/doctest.h`.
3. **New tests go in new files.** For `ffb_unit_tests` that means new `cases/test_*.cpp` files (the glob picks them up). Each new executable gets its own case directory.
4. **`ffb_unit_tests` stays as it is:** same firmware sources, same feature set, same fake `Axis.h`. New firmware sources and the F407 feature header go into the new executables.
5. **A firmware fix changes a known-issue test only if I say so.** If a `KNOWN_ISSUE` test starts reporting "should have failed but didn't", stop and tell me.
6. **Known-issue tests must fail for the stated reason.** `should_fail` hides a failure on the wrong line. In every new `KNOWN_ISSUE` test the first failing assertion must be the one that describes the defect, and the description must say so.

## The gate (run after every batch and every CMake change)

1. Every executable builds, with DSP on and off.
2. `tools/check_baseline.py` passes: no baseline case missing or changed.
3. `ffb_unit_tests` reports 0 failed. On GCC 16 the expected result with the documented exclusion is 654 passed, 4 skipped (2026-11-10; it grew with the Claude and Codex gap cases); the 318 baseline cases alone are 316 passed, 2 skipped, which `check_baseline.py` verifies. On older compilers the exclusion is not needed.
4. A sanitizer run (ASan + UBSan) is clean, apart from the two documented spline cases.
5. `git status --short Firmware` shows changes only under `Firmware/Tests/`, and no build files were written under `Firmware/`.
6. If the gate fails, fix it before continuing. Never continue past a red gate.

## Coverage ladder (estimates at plan time; measured results in the last column)

Measured as gcov executable lines of in-scope firmware (P0: about 8,160, of which 2,687 measured by gcov and about 5,470 estimated for the files that are not compiled yet; the earlier estimate was 8,900). Drivers, MIDI, HAL, the RTOS kernel, CubeMX code, TinyUSB and the Python helpers are not in this number.

| Milestone | What gets covered | Est. lines gained | Est. cumulative | Measured (gcov, after approval) |
| --- | --- | --- | --- | --- |
| Baseline | 24 compiled files | none | **measured: 2,136 of about 8,160 (26.2%)** | 26.2 % |
| P1 | Gaps in the compiled files | about 470 | about 2,610 (29%) | 32.4 % |
| P2 | Axis and main classes | about 790 | about 3,400 (38%) | 50.5 % |
| P3 | Flash and persistence | about 365 | about 3,765 (42%) | 55.3 % |
| P4 | System, callbacks, misc | about 460 | about 4,225 (47%) | 62.6 % |
| P5 | Buttons and analog inputs | about 775 | about 5,000 (56%) | 76.5 % |
| P6 | Encoders and buses | about 640 | about 5,640 (63%) | 86.2 % |
| P7 | CAN and serial FFB | about 420 | about 6,060 (68%) | 93.3 % |
| P8 | RTOS wrappers | about 365 | about 6,425 (72%) | 98.9 % |

The cumulative percentages of P1 to P8 still use the old 8,900 denominator; with the measured 8,160 the same line gains would reach about 79% after P8. They were replaced by the measured numbers in the last column (98.9 % after P8, goal 85 % reached at P6). P9-P17 add vendor and driver tiers that are not part of this ladder.

### Coverage goal (2026-10-23)

- **Goal: 85% or more** of the in-scope firmware lines, measured with `tools/coverage_report.py`.
- **Hardware-only code is not tested and not counted.** Hardware-only means behaviour that can only be observed on the board: register and clock setup, busy waits on peripheral flags, interrupt priority and timing, electrical behaviour. Each such range is listed with a reason in `Firmware/Tests/Code_Coverage/tools/coverage_exclusions.txt`; the script leaves it out of both the covered and the executable lines. Every entry is reviewed in the phase that adds it and recorded in `PROGRESS.md`.
- Code that calls the HAL with logic around it (decisions, arguments, ordering, error handling) is not hardware-only: it is tested against the fakes, and each new fake reproduces the documented behaviour of the real ST HAL / FreeRTOS / TinyUSB function (the ST HAL source in `Targets/F407VG/Drivers` is the reference; cite the function in the fake).
- Unreachable or defensive code is not excluded; it is explained in `PROGRESS.md`.
- The ladder above (P1 to P8 gains as estimated) reaches about 79% of the measured 8,160 lines. To reach 85% every file a phase adds must reach **85% or more** of its countable lines, and each phase report states how far the total is from the goal.

## Executables

| Executable | Purpose | Added in |
| --- | --- | --- |
| `ffb_unit_tests` | The existing suite, unchanged, plus new files in `cases/` | P1 |
| `ffb_target_tests` | F407 feature set minus drivers, real `Axis.h`; Axis, main classes, inputs, encoders, buses, CAN, system | P2 |
| `ffb_flash_tests` | Real `flash_helpers.cpp` and `eeprom.c` on a fake HAL FLASH | P3 |
| `ffb_rtos_tests` | The `cpp_freertos` wrappers on a fake RTOS | P8 |
| `ffb_usb_tests` | TinyUSB device code on a fake USB controller; since P17 also MidiMain, MidiHandler and the MIDI class driver | P9, P17 |
| `ffb_driver_tests` | TMC4671, ODrive, VESC, TMCDebugBridge, PWM, SimpleMotion, MyActuator drivers against simulated peers | P10, P13-P16 |
| `ffb_dsp_tests` | CMSIS-DSP modules the firmware uses, against ARM's reference patterns | P11 |
| `python_helper_tests` | the two Python helpers (stdlib `unittest`) | P9 |

They share one static library of platform fakes. Firmware sources are compiled per executable. The feature header for `ffb_target_tests` is built from the real F407 `target_constants.h`: copy its feature macros, remove the driver macros (`TMC4671DRIVER`, `PWMDRIVER`, `ODRIVE`, `VESC`, `SIMPLEMOTION`, `RMDCAN`, `TMCDEBUG`), leave `MIDI` off, and show me the diff before using it.

## Phases

### P0: Safety net (no new firmware tests)

- Save the baseline list and add `check_baseline.py` (F17).
- Add the coverage script (F16): plain `gcov`, stdlib Python, per-module, in-scope files only.
- CMake (F1): add `FFB_TESTS_EXCLUDE` for `ctest`, and exclude the two spline cases when `FFB_TESTS_SANITIZE=ON` and DSP is on (F1b). Without F1b the `host-sanitize` preset fails whenever the CMSIS-DSP submodule is checked out. Read `.github/workflows` first (read-only) and record what CI runs.
- Measure real baseline coverage with the script and replace the estimates above.
- Record the new findings in `PROGRESS.md` (skipped crash bug, preset failure, the 6 survivors).
- **Done when:** the gate passes and the measured baseline is recorded. **Stop for my approval.**

### P1: Close the gaps in the 24 compiled files (`ffb_unit_tests`, new files only)

Estimated gain about 470 lines. Target at least 85% per file. New files such as `test_error_handler.cpp`, `test_persistent_storage.cpp`, `test_filters.cpp`, `test_uart.cpp`, `test_ringbuffer.cpp`, `test_can_handler.cpp`.

- **Mutation survivors (close all six):**
  - `CmdParser.cpp:53`: buffer exactly full (capacity boundary).
  - `CmdParser.cpp:156`: malformed `?` values in complex commands.
  - `EffectsCalculator.cpp:235`: phase offset of a square wave.
  - `EffectsCalculator.cpp:1275`: constant-force rate returns 0 after 1 s without updates.
  - `HidFFB.cpp:423`: filter set-up when an effect starts.
  - `HidCommandInterface.cpp:92`: reply branch for get-at commands.
- **`ErrorHandler`** (15 of 20 functions never run): add and dedupe, both `clearError` forms, `sortErrors` order, `Error::toString` per error type, callback delivery, `clearTemp`, `ErrorPrinter` enable, disable and output.
- **`PersistentStorage`** (5 of 7): save and restore across registered handlers, delayed restore against simulated time, the startup callback.
- **`Filters`** (5 of 11): default constructor, `setFc`, `setQ`, `getQ`, `setPeakGain`; every filter type against reference coefficients; DSP on and off agree within tolerance.
- **`UART`** (12 of 30): transmit, receive, DMA and IT variants, abort, `getErrors`, on the fake HAL UART.
- **`ringbuffer.c` and `RingBufferWrapper`** (15 of 31): append, get, peek, empty, max size, wrap-around, full, overflow.
- **`CanHandler`** (6 of 8) and **`CAN.cpp`:** all callbacks dispatched to registered handlers; `isWaiting`, `handleToPort`.
- **`UsbHidHandler`** (5 of 8): `hidGet`, `hidOut`, `setHidDesc`, `getHidDesc`.
- **`CommandHandler`** (9 of 35): `command(...)`, `setInstance`, `logSerial`, `logSerialDebug`, `getHelpstring`, `overrideCommandFlags_INT`.
- **`EffectsCalculator::Run()`** (the thread loop) and `getInfo`.
- **Small ones:** `Encoder::getPosAbs` and `setPos`, `MotorDriver::setupDriver`, `CDCcomm` constructor and `connected`, `cthread` `DelayUntil`, `cmutex` and `csemaphore` lock paths.
- The skipped "unknown id" case stays skipped (it terminates the run). It is recorded as a suspected bug, not fixed.
- **Done when:** gate passes, per-file coverage met, a mutation spot-check on the new tests kills at least 80%. **Stop for my approval.**

### P2: Axis and main classes (new `ffb_target_tests`)

Estimated gain about 790 lines. Highest-risk code: the whole torque path.

- Batch 0 (one agent first): F3 feature header, F7 fake TIM and EXTI, F10 misc CMSIS and HAL stubs, F12 `AxisFixture` (real Axis with `MockMotorDriver` and `MockEncoder` chosen through the public choosers), F14 `-include cmath` for `Axis.cpp` if needed. Run the gate.
- **`Axis.cpp`:** power scaling and clipping to ±0x7fff, invert, range and degrees scaling, endstop torque, speed limiter, slew-rate limiter, idle spring, damper, friction and inertia strengths, expo curve, gear ratio, effect ratio, fade-in, emergency stop and reset, hands-off detection, config encode and decode round trip, commands with invalid values, driver and encoder selection.
- **Integration:** PID report to effect to axis to `MockMotorDriver` torque, with position from `MockEncoder`.
- **`AxesManager`, `FFBHIDMain` and its commands, `FFBWheel`, `FFBJoystick`, `FFBHIDExt`, `CustomMain`, `FFBoardMain`, `mainclass_chooser`:** axis count changes, HID gamepad report packing, FFB on and off, USB suspend and resume, class IDs unique and selectable.
- **Done when:** gate passes, files at 85% or more (without hardware-only code), mutation spot-check at 80% or more. **Stop for my approval.**

### P3: Flash and persistence (new `ffb_flash_tests`)

Estimated gain about 365 lines.

- Fake HAL FLASH (F9) with real NOR semantics: erase to 0xFFFF, programming only clears bits, injectable errors, power-loss injection, OTP area.
- **`flash_helpers.cpp` and `eeprom.c` (vendor, tested directly):** read, write, default, dump, format, write-only-if-changed, missing variable, page full and page transfer, power loss during write and during transfer, invalid page states, OTP.
- **`eeprom_addresses.c`:** `VirtAddVarTab` entries unique, count equals `NB_OF_VAR`, no 0xFFFF.
- `PersistentStorage` restore ordering against the real flash layer.
- Vendor bugs are reported in `PROGRESS.md`, never fixed.

### P4: System, callbacks and misc (`ffb_target_tests`)

Estimated gain about 460 lines.

- `SystemCommands` (each command; resets recorded, not executed), `global_callbacks` (ISR to handler dispatch), `cppmain` `micros()`, `ledEffects` against simulated time, `voltagesense` scaling and over-voltage behaviour, `USBdevice`, `AdcHandler`.
- USB descriptor tables self-consistent: `wTotalLength`, HID report descriptor length, report IDs match `ffb_defs.h`.
- Needs F6 (fake ADC) and F10.

### P5: Buttons and analog inputs (`ffb_target_tests`)

Estimated gain about 775 lines. Needs F4 (fake SPI), F5 (fake I2C), F6 (fake ADC), F8 (fake CAN) and `MockButtonSource` and `MockAnalogSource` (F12).

- Buttons: `LocalButtons`, `SPIButtons`, `ShifterAnalog`, `CanButtons`, `PCF8574`, `SelectableInputs`, `ButtonSource`.
- Analog: `AnalogAxisProcessing` (autoscale, manual range, invert, filters, config round trip), `LocalAnalog`, `CanAnalog`, `ADS111X`, `AnalogSource`.

### P6: Encoders and buses (`ffb_target_tests`)

Estimated gain about 640 lines.

- `EncoderLocal`, `EncoderSSI`, `EncoderBissC` (CRC6, error and warning bits, multiturn), `MtEncoderSPI` (parity, status), `CRC` against known check values.
- `SPI`, `I2C` and their handlers, `TimerHandler`, `ExtiHandler`: port and device arbitration, callbacks.
- Driver encoders (TMC, ODrive, VESC, RMD, SimpleMotion) are not part of this phase; they are tested with their drivers in P10 and P13-P16 (`ffb_driver_tests`).

### P7: CAN and serial FFB (`ffb_target_tests`)

Estimated gain about 420 lines.

- `CanBridge` frame to host protocol, `CanInputMain`, `CANPort2B` (filters, tx, rx, bus-off), `SerialFFB` (effect commands over the string interface).

### P8: RTOS wrappers (new `ffb_rtos_tests`)

Estimated gain about 365 lines. Needs F11 (queues, timers, event groups, simple non-preemptive).

- `cqueue`, `ctimer`, `ctasklet`, `cevent_groups`, `cworkqueue`, `cmem_pool`, `cread_write_lock`, `ccondition_variable`, `ctickhook`: lifecycle, take and give semantics, timeouts against simulated time. The FreeRTOS kernel itself is not tested.

### P9: Remaining vendor tiers and helpers

- **Tier B:** TinyUSB device code in `Firmware/FFBoard/USB/` on a fake USB controller (F15), in `ffb_usb_tests`.
- **Tier C:** the CMSIS-DSP functions the firmware calls (biquad, spline, sin, cos), against reference values.
- **Python helpers** with stdlib `unittest`: `Firmware/scripts/generate_memory.py` and `dwc2_info.py` (vendor: report bugs, never fix).
- **Optional:** verify the `HidFFB` `characterization` tests against the USB HID PID 1.01 specification, only if I give you the spec file path.

### P10: Driver track (TMC4671, ODrive, VESC; new `ffb_driver_tests`)

Added 2026-11-03. Driven by `Tests/Code_Coverage/claude/plans/DRIVER_TEST_PLAN.md` (its phases D0-D3 are the sub-phases of P10; stop after each). Tests the firmware side of the three in-scope drivers against simulated peers (`FakeTmc4671` behind the fake SPI, `FakeOdrivePeer` and `FakeVescPeer` on the CAN fake): frames, scaling, limits, state machines, error handling, timeouts. Requirement charters in `Tests/Code_Coverage/claude/charters/driver/<name>_REQUIREMENTS.md`, tests in `Firmware/Tests/Code_Coverage/claude/cases/driver/`. Spec files are supplied by me and never committed. Reported as a separate driver tier, not in the in-scope coverage ladder.

### P11: CMSIS-DSP modules in scope (extends tier C)

Status 2026-11-10: P11 complete. G1 (infrastructure, FastMath, helpers), G2 (Interpolation, Controller), G3 (Statistics), G4 (Filtering part 1: biquad, FIR, decimation / interpolation) and G5 (Filtering part 2: conv, correlate, partial conv, Levinson-Durbin, LMS, lattices, sparse FIR) are written, reviewed, gated, approved and pushed (`caefd66e`, `8425c55a`, `aaa99958`, `2000a082`, `eb682e0b`); `ffb_dsp_tests` 2485 cases, vendor findings V37-V85 (PROGRESS bug table), open questions 17-23. Group results and decisions L7-L11: `Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md`.

Added 2026-11-04 at my request. Scope: all functions and all data types (f64, f32, f16, q31, q15, q7 where they exist) of the CMSIS-DSP modules the firmware uses (fast math, filtering, interpolation, controller, and the basic/support/common-table helpers they depend on), not only the functions the firmware calls. Oracle: ARM's reference patterns in `Firmware/Libraries/CMSIS-DSP/Testing/Patterns` (vendor-generated expected results) plus the mathematical definition / hand derivation for edge cases; never the library's own output. Same method as the driver track (Sonnet inventory, lead charter, Sonnet writers in parallel, fresh reviewer with mutation spot-check in a scratch copy, lead QC). Stop for my approval after each module group. Vendor bugs are reported, never fixed.

### P13-P16: TMCDebugBridge, PWM, SimpleMotion, MyActuator (`ffb_driver_tests`)

Added 2026-11-10 (owner). P13: `TMCDebugBridge.cpp`, `Tests/Code_Coverage/claude/plans/P13_TMCDEBUGBRIDGE_PLAN.md`. P14-P16: `MotorPWM.cpp`, `MotorSimplemotion.cpp`, `RmdMotorCAN.cpp`, `Tests/Code_Coverage/claude/plans/P14_P17_REMAINING_DRIVERS_PLAN.md`. Each phase had T0 (charter), T1 (wiring, fake, fixture) and T2 (tests, review with mutation checks, gate), with a stop for approval. Status: all four complete and pushed. Results: `TMCDebugBridge.cpp` 166 / 166 lines, `MotorPWM.cpp` 191 / 191, `MotorSimplemotion.cpp` 294 / 300, `RmdMotorCAN.cpp` 231 / 231; `ffb_driver_tests` 1717 cases; details in `PROGRESS.md` and `Tests/Code_Coverage/claude/reports/DRIVER_TRACK_REPORT.md`.

### P17: MIDI (`MidiMain`, `MidiHandler`, TinyUSB `class/midi`; `ffb_usb_tests`)

Added 2026-11-10 (owner). Status: T0 (`Tests/Code_Coverage/claude/charters/driver/MIDI_REQUIREMENTS.md`) and T1 approved 2026-11-10; T2 approved and committed (`fe232b2d`, pushed): 319 cases in `ffb_usb_tests`, `MidiHandler.cpp` 100 %, `MidiMain.cpp` 100 %, `midi_device.c` 98.5 %, firmware bugs 219-224, vendor bugs V86-V88. P17 is complete. Oracles supplied: `midi10.pdf` (USB Device Class Definition for MIDI Devices 1.0) and `MIDI_1.0_Detailed_Specification.pdf`; the firmware class is USB-MIDI 1.0 style, so `USB MIDI v2_0.pdf` is not needed. Where the specs are silent the tests are `characterization:`. The macro `MIDI` is turned on in `ffb_usb_tests` only, at T1. Next step: none; the owner's open questions 54-61 remain. See the P17 section of `P14_P17_REMAINING_DRIVERS_PLAN.md`.

### P18: 32-bit build (`FFB_TESTS_32BIT`; all executables)

Added 2026-11-11 (owner chose it from four proposals: integration tests, 32-bit build, ARM struct layout checks, fuzzing). Plan: `Tests/Code_Coverage/claude/plans/P18_32BIT_BUILD_PLAN.md`. The suite is built as ILP32 (`-m32 -msse2 -mfpmath=sse -funsigned-char`) so `size_t`, `long` and pointers have the target's size. T1 CMake option (5 `__int128` DSP files left out, owner's decision 62 B), T2 bug 225 tests (`test_serial_ffb_32bit.cpp`, 31 cases, markers `KNOWN_ISSUE_32BIT(_UB)`), T3 warning diff over all firmware compile commands (no further findings): approved 2026-11-11. T4 documentation approved; P18 committed and pushed 2026-11-11 (`544b4162`). Not covered: ARM's `uint32_t` type and struct alignment (separate ARM compile check, proposal only).

## Fake fidelity rules

Every fake (HAL peripheral, kernel, bus, peer device) states in its header:
- **Models:** the behaviour it reproduces, and the **source** for it (RM0090 chapter, ST HAL source file:line, FreeRTOS / TinyUSB source, datasheet or protocol spec section).
- **Does not model:** what it leaves out (timing, electrical effects, features the firmware does not use).
- **Confidence:** High (follows a source line by line), Medium (follows the documented behaviour, simplified), Low (assumed).
- **Strict:** use that the real hardware or peer would reject (unmodelled call, malformed frame, wrong state) fails the test (`REQUIRE_ALL_HAL_MODELLED()` / a fault list), never silently succeeds.
- Peers support fault injection (no reply, late reply, corrupted data, bus error) and use simulated time only.
Fidelity entries for the driver peers are kept in the "Driver track" section of `Tests/Code_Coverage/claude/plans/PROGRESS.md`.

## How each phase runs

1. **Plan:** the main agent reads the module sources, lists behaviours to test, and updates the module table in `PROGRESS.md`.
2. **Build:** subagents in parallel, one per module group, each owning its own new files. They follow `Firmware/Tests/README.md` and `CLAUDE.md`. They must not edit each other's files, shared fakes or CMake without telling me the change needed.
3. **Review:** fresh reviewer subagents check assertions are meaningful, edge cases and error paths are covered, no out-of-scope driver code is tested, fakes are realistic, tests are deterministic with no sleeps, `KNOWN_ISSUE` rules are followed, and nothing breaks MinGW or CI.
4. **Verify:** run the gate, measure coverage of the phase's files with the script, and run a mutation spot-check in a scratch copy outside the repo (about 8 mutants per file on lines the tests execute). List survivors in `PROGRESS.md`.
5. **Commit and record:** local commit on `add-test-suite` (push only to the user's fork, when asked), update `PROGRESS.md` with measured coverage, `characterization:` and `NEEDS REVIEW` items, suspected bugs and host-vs-target caveats.

## Stop and approval rules

- Stop after P0 and after P1 for my go-ahead. After that, stop at the end of each phase unless I wrote "continue through P<n>".
- Stop immediately if a baseline case changes result, the gate cannot be made green, a firmware change seems necessary, or something would have to be downloaded.
- Ask before adding an executable not listed above, touching `.github/`, or changing the feature set of `ffb_unit_tests`.

## Scope and known issues (unchanged)

- In scope: firmware outside `Firmware/Libraries/`, plus the vendor tiers A, B, C above. Since 2026-11-04 (P11) tier C covers every function and data type of the CMSIS-DSP modules the firmware uses. Not tested: ST HAL, CMSIS core, the FreeRTOS kernel, `Firmware/Targets/**`, CubeMX code.
- In scope since 2026-11-10 (P14 PWM, P15 SimpleMotion, P16 MyActuator, P17 MIDI incl. TinyUSB `class/midi`; plan `P14_P17_REMAINING_DRIVERS_PLAN.md`): P14, P15 and P16 are complete (their macros are on in `ffb_driver_tests` only); P17 is complete (T2 `fe232b2d`); `MIDI` is on in `ffb_usb_tests` only. The TMC4671, ODrive and VESC drivers are in scope since 2026-11-03 (P10, `ffb_driver_tests` only). `TMCDebugBridge` is in scope since 2026-11-10 (P13, complete, `ffb_driver_tests` only, `TMCDEBUG` on there).
- **Known issue 1:** `effectsForces` out-of-bounds read aborts on GCC 16. Exclude only the case `requesting the forces of a not existing axis is rejected`.
- **Known issue 2:** CMSIS-DSP `arm_spline_f32` overflow found by ASan. Skip only the two spline cases in sanitizer runs.
- **Known issue 3:** the skipped case `command with an unknown id is rejected` (`test_command_execution.cpp:186`) stays skipped. `executeCommands` dereferences the command definition before the null check.
- Never fix firmware or vendor code. Report it.

## Deliverables

- New tests, fakes, mocks and CMake changes in `Firmware/Tests/`, plus `baseline/test_cases.txt` and `tools/` (baseline checker, coverage script).
- Updated `Firmware/Tests/README.md`: new executables, case directories, options, the exclusion variable, tools, vendor tiers, suite table, limitations and next steps.
- Final report in `Tests/Code_Coverage/claude/plans/PROGRESS.md` and `Tests/Code_Coverage/claude/reports/PROJECT_SUMMARY.md`: measured coverage per module against the ladder, what is excluded and why, survivors, `NEEDS REVIEW` items, suspected firmware and vendor bugs, host-vs-target caveats, untestable code with refactor suggestions, and any CI changes I need to make myself.
