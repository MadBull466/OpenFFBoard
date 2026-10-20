# Test Suite Progress

> This file is the resume point. Read "Current state" and "Next action" first.
> Claude updates it after every module, phase gate, decision and finding.
> Plan: `Tests/TEST_PLAN.md`. Rules: root `CLAUDE.md`.

## Current state

- **Current phase:** 2 (Build), batch 0
- **Phase status:** in progress (go-ahead given 2026-10-21)
- **Branch:** add-test-suite (base commit `a5418ff5`, "Initial unit test system")
- **Last commit:** `d12483c8` (Phase 1 plan + decisions)
- **Last updated:** 2026-10-21
- **Last session summary:** Phase 1 steps 1-9 done. Baseline built and run out-of-tree with GCC 16 (DSP and no-DSP), clang 23, ASan+UBSan (GCC and clang) and gcov. With the `effectsForces` case excluded: 316/316 test cases pass (2 skipped by design). Sanitizer runs found one more issue: an ASan stack-buffer-overflow inside CMSIS-DSP `arm_spline_f32` reached from `EffectsCalculator::evaluateReconstructionFilter` (2 test cases). Firmware mapped, driver list drafted, coverage gaps recorded, plan written below. Nothing under `Firmware/` was changed or written.

## Next action

**Waiting for my review of the F3 feature header diff** (`Firmware/Tests/platform/target_f407/target_constants.h`, not committed, not used by any target yet). It is a copy of `Firmware/Targets/F407VG/Core/Inc/target_constants.h` with exactly these lines removed: `MIDI`, `TMCDEBUG`, `RMDCAN`, `TMC4671DRIVER`, `PWMDRIVER`, `ODRIVE`, `VESC`, `SIMPLEMOTION`, plus a provenance comment. Check with `diff -u Firmware/Targets/F407VG/Core/Inc/target_constants.h Firmware/Tests/platform/target_f407/target_constants.h`.

After approval: commit the header, then F2 (new executables, one CMake change at a time, 316/0/2 check after each) and the remaining fakes F4-F12, F14-F16 via one subagent, then batches 1..N.

## Phase checklist

| Phase | Status | Approved by me? | Notes |
|---|---|---|---|
| 1. Explore and plan | done | yes (2026-10-21) | Q1-Q8 and A answered |
| 2. Build | in progress | | batch 0: F1 done (`1876353e`); F3 header approved and committed |
| 3. Review | not started | | |
| 4. Verify | not started | | |
| 5. CPython helpers (vendor tier A) | not started | yes (2026-10-21, no longer optional) | `Firmware/scripts/generate_memory.py`, `Firmware/FFBoard/USB/portable/synopsys/dwc2/dwc2_info.py` |

## Decisions log

| Date | Decision | Reason |
|---|---|---|
| 2026-10-20 | Extend the existing doctest suite in `Firmware/Tests/` instead of creating a new `Tests/` suite | It already exists with its CMake setup. A second suite would duplicate it and drift |
| 2026-10-20 | Rule is "`Firmware/` read-only except `Firmware/Tests/`" | Existing suite lives inside `Firmware/` |
| 2026-10-20 | Build out-of-tree | Nothing should be written under `Firmware/`, not even gitignored build files |
| 2026-10-20 | Use default Makefiles generator | Ninja is only a shell alias, not on PATH |
| 2026-10-20 | Exclude the `effectsForces` case with `-tce`, do not fix firmware, do not disable `_GLIBCXX_ASSERTIONS` globally | It is a real suspected bug. Assertions may catch others |
| 2026-10-20 | Exact exclusion name: `requesting the forces of a not existing axis is rejected` (`cases/test_effects_calculator.cpp:1062`, suite `EffectsCalculator`) | Confirmed from the source and from the abort output |
| 2026-10-20 | Build root `../ffb-test-build` (sibling of the repo checkout), one subdirectory per variant | Outside the repo and stable across sessions (the session scratchpad is not). Confirmed by me 2026-10-21 (Q7) |
| 2026-10-21 | Commits: local commits on `add-test-suite` allowed, NEVER push, never add/change a git remote. Identity via one-off `git -c user.name=... -c user.email= commit`, no git config change. Commit hashes tracked here. No tar snapshot. Phase 4 keeps the "`git diff` shows no changes under `Firmware/` except `Firmware/Tests/`" check (Q8) | My decision |
| 2026-10-21 | `TMCDebugBridge`, `MidiMain` and `MidiHandler` are out of scope. MIDI removed from the plan; `MIDI` feature macro stays off (Q1) | My decision |
| 2026-10-21 | Vendor code is in scope in tiers; vendor bugs are reported here, never fixed. Tier A: `eeprom.c` (directly, on the NOR-semantics fake flash), FreeRTOS C++ wrappers (`c*.cpp`), both Python helpers (Phase 5). Tier B: TinyUSB device code in `FFBoard/USB` against a fake USB controller, own batch near the end. Tier C: only the CMSIS-DSP functions the firmware calls. Not tested: ST HAL, FreeRTOS kernel, `Targets/**` (Q2, revised) | My decision |
| 2026-10-21 | ASan stays on for the whole build including CMSIS-DSP. Sanitizer runs additionally exclude only the 2 spline cases with `-tce`. Bug #2 stays listed. The earlier option "build CMSIS-DSP without ASan" (F13) is dropped (Q3, revised) | My decision |
| 2026-10-21 | Do F1. `ffb_unit_tests` stays exactly as it is (sources, feature set, fake `Axis.h`). New executables get their own feature header derived from the F407 `target_constants.h` minus driver macros, `MIDI` off; diff shown to me before use. Firmware sources compiled per executable, only platform fakes shared. After every CMake change: rebuild, confirm 316 passed / 0 failed / 2 skipped with the exclusion, record it here (Q4/Q5) | My decision |
| 2026-10-21 | Commit identity: `git -c user.name=MadBull.Ben -c user.email= commit` | My answer to question A |
| 2026-10-21 | `Firmware/Tests/` was moved out of the repo by accident and restored by me. Re-check after restore: no diff against HEAD under `Firmware/`, `ffb_unit_tests` with the exclusion = 316 passed / 0 failed / 2 skipped | Incident record |
| 2026-10-21 | F3 feature header `Tests/platform/target_f407/target_constants.h` approved: F407 copy minus `MIDI`, `TMCDEBUG`, `RMDCAN`, `TMC4671DRIVER`, `PWMDRIVER`, `ODRIVE`, `VESC`, `SIMPLEMOTION` | My review of the diff |
| 2026-10-21 | Coverage: plain `gcov` + a stdlib-only Python script in `Firmware/Tests/`. Nothing installed (Q6) | My decision |

## Environment

All commands run from the repo root (`OpenFFBoard/`). They are plain POSIX commands (work in bash/zsh; in fish replace `$(nproc)` with `(nproc)`).

- Host OS / compiler / generator: Linux (CachyOS, kernel 7.2), GCC 16.2.1, clang 23.1.3, CMake 4.4.4, Unix Makefiles (Ninja not on PATH)
- Test framework: doctest 2.4.11 (`Firmware/Tests/third_party/doctest.h`)
- Test source location: `Firmware/Tests/`
- CMSIS-DSP submodule: checked out, so the default build has `USE_DSP_FUNCTIONS=ON`
- Build directory (out-of-tree): `../ffb-test-build/<variant>` (variants: `default`, `nodsp`, `san`, `nodsp-san`, `clang`, `clang-san`, `cov`)
- Configure: `cmake -S Firmware/Tests -B ../ffb-test-build/default`
  - no DSP: add `-DFFB_TESTS_USE_DSP=OFF`
  - ASan+UBSan: add `-DFFB_TESTS_SANITIZE=ON`
  - clang: add `-DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++`
  - coverage: add `-DCMAKE_C_FLAGS=--coverage -DCMAKE_CXX_FLAGS=--coverage -DCMAKE_EXE_LINKER_FLAGS=--coverage` (no CMake change needed)
- Build: `cmake --build ../ffb-test-build/default -j"$(nproc)"`
- Run (with exclusion): `../ffb-test-build/default/ffb_unit_tests --test-case-exclude="requesting the forces of a not existing axis is rejected"` (`-tce=` is the short form)
- Sanitizer run (both exclusions, as decided 2026-10-21): `../ffb-test-build/san/ffb_unit_tests --test-case-exclude="requesting the forces of a not existing axis is rejected,reconstruction of a constant signal is constant,reconstruction of a step never leaves the range of the input values and reaches the target"`
- `ctest`: `ctest --test-dir ../ffb-test-build/default --output-on-failure` (add `-V` to see the doctest summary). Since F1 the exclusions come from the CMake cache lists `FFB_TESTS_EXCLUDE` (always) and `FFB_TESTS_EXCLUDE_SANITIZE` (added when `FFB_TESTS_SANITIZE=ON`); the configure step prints the active list (`OpenFFBoard tests: excluded from ctest: ...`).
- Coverage command: gcovr and lcov are NOT installed. Plain `gcov` (part of GCC 16) works: after a run of the `cov` build, `cd <scratch dir> && gcov -n $(find ../ffb-test-build/cov -name '*.gcda' | grep FFBoard/)`. Per-module report: a stdlib-only Python script in `Firmware/Tests/` (to be written, F16) that aggregates `gcov` output; no gcovr/lcov.
- Sanity check after any build: `git status --short --ignored Firmware` must print nothing.

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
- In sanitizer builds `KNOWN_ISSUE_UB` cases are `skip()`ped automatically (`support/TestHelpers.h:41`), so the `effectsForces` case is skipped there anyway; the `-tce` is kept for consistency.
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

## Out-of-scope driver files (confirmed 2026-10-21)

| Path (under `Firmware/FFBoard/`) | Driver | Driver code or caller code? | Confirmed |
|---|---|---|---|
| `UserExtensions/Inc/TMC4671.h`, `UserExtensions/Src/TMC4671.cpp` (classes `TMC4671`, `TMC_1`, `TMC_2`, `TMC4671Biquad`, `TMC_ExternalEncoderUpdateThread`, enums/structs `TMC4671Limits` etc.) | TMC4671 | Driver | yes |
| `UserExtensions/Src/TMC4671_configs.cpp` | TMC4671 | Driver (hardware config tables) | yes |
| `UserExtensions/Inc/TMCDebugBridge.h`, `UserExtensions/Src/TMCDebugBridge.cpp` (main class `TMCDebugBridge`, friend of `TMC4671`) | TMC4671 | Driver tooling. Out of scope | yes |
| `UserExtensions/Inc/ODriveCAN.h`, `UserExtensions/Src/ODriveCAN.cpp` (`ODriveCAN`, `ODriveCAN1`, `ODriveCAN2`) | ODrive | Driver | yes |
| `UserExtensions/Inc/VescCAN.h`, `UserExtensions/Src/VescCAN.cpp` (`VescCAN`, `VESC_1`, `VESC_2`) | VESC | Driver | yes |
| `UserExtensions/Inc/MotorPWM.h`, `UserExtensions/Src/MotorPWM.cpp` (`MotorPWM`, `PWMConfig`) | PWM | Driver | yes |
| `UserExtensions/Inc/RmdMotorCAN.h`, `UserExtensions/Src/RmdMotorCAN.cpp` (`RmdMotorCAN`, `RmdMotorCAN1`, `RmdMotorCAN2`) | MyActuator (RMD) | Driver | yes |
| `UserExtensions/Inc/MotorSimplemotion.h`, `UserExtensions/Src/MotorSimplemotion.cpp` (`MotorSimplemotion`, `MotorSimplemotion1/2`) | SimpleMotion | Driver | yes |
| `Targets/*/Core/Src/cpp_target_config.cpp` (`MotorPWM::timerConfig`, TMC pin config) | PWM, TMC4671 | Target hardware config. Out of scope (`Targets/**` not tested) | yes |
| `Inc/MotorDriver.h`, `Src/MotorDriver.cpp` (`MotorDriver` base class + driver `ClassChooser` registry) | shared base/interface | **Caller code, in scope.** Fake drivers at this interface (`MockMotorDriver` already exists) | yes |
| `Inc/Encoder.h`, `Src/Encoder.cpp` (`Encoder` base; all drivers above except PWM are also encoders) | shared base/interface | **Caller code, in scope.** `MockEncoder` exists | yes |
| `Src/Axis.cpp`, `Inc/Axis.h` (driver/encoder choosers with `add_class<TMC_1/ODriveCAN1/VESC_1/...>`) | all | **Caller code, in scope.** Driver classes are compiled out by not defining the driver feature macros; mocks are registered instead | yes |
| `UserExtensions/Src/FFBHIDMain.cpp`, `UserExtensions/Inc/FFBHIDMain.h` (includes `TMC4671.h`, `MotorPWM.h`) | TMC4671, PWM | **Caller code, in scope.** Needs the headers to compile, does not call driver internals | yes |
| `UserExtensions/Src/MidiMain.cpp`, `UserExtensions/Inc/MidiMain.h` (constructs `TMC_1` directly, uses `TMC4671Limits`, `setEncoderType`) | TMC4671 | Out of scope (with `MidiHandler`). `MIDI` stays off | yes |
| `Inc/Filters.h` (`friend TMC4671Biquad`) | TMC4671 | `Biquad` itself is in scope; only the friend declaration touches the driver | yes |
| `Inc/constants.h`, `Inc/cpp_target_config.h`, `UserExtensions/Inc/ClassIDs.h`, `eeprom_addresses.*` | all | IDs/addresses/feature macros only. In scope as data (uniqueness checks) | yes |

## Vendor code tiers (decided 2026-10-21)

Vendor bugs are reported in "Suspected firmware bugs" (marked "vendor") and never fixed.

| Tier | Path | What it is | How it is tested |
|---|---|---|---|
| A | `Firmware/FFBoard/Src/eeprom.c`, `Inc/eeprom.h` | ST EEPROM emulation (AN3969, STMicroelectronics copyright) | Directly, in `ffb_flash_tests` on the NOR-semantics fake flash (F9): init/format, read/write, page full -> page transfer, interrupted transfer/erase states, invalid page headers |
| A | `Firmware/FFBoard/Src/c*.cpp` (`cthread`, `csemaphore`, `cmutex`, `cqueue`, `ctimer`, `ctasklet`, `cevent_groups`, `cworkqueue`, `cmem_pool`, `cread_write_lock`, `ccondition_variable`, `ctickhook`) + `Inc/*.hpp` | "FreeRTOS C++ Wrappers" by Michael Becker | Against the fake FreeRTOS (F11): argument forwarding, return value handling, create-failure paths, ISR variants. Only what the non-preemptive fake can express |
| A | `Firmware/scripts/generate_memory.py`, `Firmware/FFBoard/USB/portable/synopsys/dwc2/dwc2_info.py` | PC-side helper scripts | Phase 5, pytest, outside CMake |
| B | `Firmware/FFBoard/USB/**` (device stack: `tusb.c`, `device/usbd*.c`, `class/hid`, `class/cdc` as used by the firmware) | TinyUSB (MIT) | Own batch near the end, against a fake USB controller (`dcd_*` port layer, F15): enumeration, control requests, descriptor delivery, HID/CDC transfers. Host stack, other classes and `portable/**` drivers not tested |
| C | `Firmware/Libraries/CMSIS-DSP`: `arm_sin_f32`, `arm_cos_f32`, `arm_sqrt_f32`, `arm_biquad_cascade_df1_f32`/`_init_f32`, `arm_spline_f32`/`_init_f32`, `arm_pid_f32`/`_init_f32`/`_reset_f32`, `arm_std_f32` | Functions called by `Filters.cpp`, `EffectsCalculator.cpp`, `HidFFB.cpp`, `Axis.cpp` | Accuracy against `<cmath>`/reference values, boundary inputs, block sizes used by the firmware |
| — | `Firmware/Targets/*/Drivers/**`, `Targets/*/Middlewares/**`, `Targets/*/Core/**`, rest of `Firmware/Libraries/` | ST HAL, CMSIS core, FreeRTOS kernel, CubeMX init | Not tested unless I say otherwise |

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
| Persistence | `Src/PersistentStorage.cpp`, `flash_helpers.cpp`, `UserExtensions/Src/eeprom_addresses.c` | Save/restore, flash variable access, dump/format, OTP | HAL FLASH, `vTaskDelay` | `USE_EEPROM_EMULATION` |
| System | `Src/SystemCommands.cpp`, `cppmain.cpp`, `ledEffects.cpp`, `voltagesense.cpp` | System commands, micros, LEDs, voltage/temperature sense | `NVIC_SystemReset`, UID/DEVID, `vTaskList`, GPIO, LL ADC macros, IWDG | various |
| Main classes | `UserExtensions/Src/FFBoardMain.cpp`, `FFBHIDMain.cpp`, `FFBHIDMain_commmands.cpp`, `FFBWheel.cpp`, `FFBJoystick.cpp`, `FFBHIDExt.cpp`, `CustomMain.cpp`, `CanInputMain.cpp`, `CanBridge.cpp`, `mainclass_chooser.cpp`, `Inc/ClassChooser.h` | Main class selection, HID gamepad report building, CAN bridge protocol | TIM, GPIO, tusb, `vTaskDelay` | `FFBWHEEL`, `FFBJOYSTICK`, `FFBHIDEXT`, `CANINPUTMAIN`, `CANBRIDGE`, `TIM_FFB` |
| Button sources | `Src/ButtonSource.cpp`, `SelectableInputs.cpp`, `UserExtensions/Src/LocalButtons.cpp`, `SPIButtons.cpp`, `ShifterAnalog.cpp`, `CanButtons.cpp`, `PCF8574.cpp`, `ButtonSources.cpp` | Button reading and decoding | GPIO, SPI, I2C, CAN | `LOCALBUTTONS`, `SPIBUTTONS`, `SHIFTERBUTTONS`, `CANBUTTONS`, `I2C_PORT` |
| Analog sources | `Src/AnalogSource.cpp`, `AnalogAxisProcessing.cpp`, `AdcHandler.cpp`, `UserExtensions/Src/LocalAnalog.cpp`, `CanAnalog.cpp`, `ADS111X.cpp`, `AnalogSources.cpp` | Analog axes, autoscale, filters | ADC DMA buffer, I2C, CAN | `ANALOGAXES`, `CANANALOG`, `I2C_PORT` |
| Encoders (non-driver) | `UserExtensions/Src/EncoderLocal.cpp`, `EncoderSSI.cpp`, `EncoderBissC.cpp`, `MtEncoderSPI.cpp`, `EncoderSources.cpp` | Quadrature/SSI/BiSS-C/MT6825 decoding, CRC/parity | TIM CNT, EXTI, SPI DMA | `LOCALENCODER`, `MTENCODERSPI` |
| Bus wrappers | `Src/SPI.cpp`, `SpiHandler.cpp`, `I2C.cpp`, `I2CHandler.cpp`, `TimerHandler.cpp`, `ExtiHandler.cpp` | Port/device arbitration, callbacks | HAL SPI/I2C, `portYIELD_FROM_ISR` | `I2CBUS` |
| Buffers | `Src/ringbuffer.c`, `ringbufferwrapper.cpp` | Ring buffer | none | always |
| MIDI | `Src/MidiHandler.cpp`, `UserExtensions/Src/MidiMain.cpp` | **Out of scope** (decided 2026-10-21) | tusb MIDI, TMC4671 | `MIDI` (stays off) |
| USB descriptors | `UserExtensions/Src/usb_descriptors.cpp`, `usb_hid_1ffb_desc.c`, `usb_hid_2ffb_desc.c`, `usb_hid_gamepad.c` | Descriptor tables | none | `AXIS1_FFB_HID_DESC` etc. |

Host compile probe (each file compiled alone with the current test flags, objects to scratch): compiles but empty because of feature gates: most `UserExtensions` files; fails today: `Axis.cpp` (fake `Axis.h` shadows the real one), `AxesManager.cpp`/`LocalAnalog.cpp` (`ADC_PINS`), `LocalButtons.cpp`/`ShifterAnalog.cpp` (`BUTTON_PINS`), `SPIButtons.cpp`/`EncoderSSI.cpp`/`EncoderBissC.cpp` (SPI HAL constants), `CANPort2B.cpp` (`CAN_FilterTypeDef`), `CRC.cpp` (`__REVSH`), `flash_helpers.cpp` (`__weak`, HAL FLASH), `SystemCommands.cpp` (`NVIC_SystemReset`), `USBdevice.cpp` (`HAL_GetUIDw2`), `voltagesense.cpp` (LL ADC macros), `ledEffects.cpp` (LED pins), `cppmain.cpp`, `global_callbacks.cpp`, `FFBHIDMain*.cpp` (`Control_t` from real `Axis.h`), RTOS wrappers needing queues/timers/event groups.

## Existing coverage and techniques (inventory)

Techniques already used in `Firmware/Tests/` (to be reused):
- **Link substitution**: fake platform in `platform/src/` implements HAL (`HAL_GPIO_*`, `HAL_UART_*`, `HAL_GetTick`), FreeRTOS (tasks, notifications, semaphores, `vTaskDelay`) and TinyUSB (`tud_cdc_n_write`, `tud_hid_n_report`), and the **`Flash_*` API itself** (map-backed, so `flash_helpers.cpp`/`eeprom.c` are not compiled).
- **Shadowed headers**: `platform/include/` replaces `main.h`, FreeRTOS headers, `target_constants.h`, `tusb_config.h`; `fakes/Axis.h` replaces the real `Axis.h` (include order matters).
- **Mocks of firmware interfaces**: `MockCommandHandler`, `MockCommandInterface`, `MockMotorDriver`, `MockEncoder`, `MockCANPort`.
- **Simulated time/scheduler**: `HostPlatform::advanceMs`, `HostRtos::runUntilBlocked/runUntilIdle`; fixtures `CommandSystemFixture`, `FfbFixture` reset global state per test.
- **Markers**: `characterization:` names, `KNOWN_ISSUE(...)` (should_fail), `KNOWN_ISSUE_UB(...)` (skipped under sanitizers).

Per-suite existing coverage (test case counts): CmdParser 39, CommandExecution/CommandHandler 34, StringCommandInterface/CDC/UART 41, HID_CommandInterface 22, CAN_CommandInterface 20, HidFFB 53, EffectsCalculator 86, Encoder/MotorDriver (mock interfaces) 17, platform self-test 6.

Coverage gaps (from gcov and the source map):
- **Not compiled at all**: Axis, AxesManager, SerialFFB, ErrorHandler commands, SystemCommands, flash_helpers, all button/analog sources, non-driver encoders, CRC, FFBHIDMain + gamepad report, CanBridge, CanInputMain, CANPort2B, global_callbacks, SPI/I2C wrappers, ledEffects, voltagesense, USB descriptors, mainclass_chooser.
- **Compiled but weakly covered**: ErrorHandler 13 %, Filters 28 % (only via effects), PersistentStorage 33 %, CanHandler/CAN 43-47 %, UsbHidHandler 45 %, ringbuffer/wrapper 50-59 % (wrap-around, overflow, peek paths), UART 57 %, CDCcomm 70 %, CommandHandler 84 % (help/info strings, class ID lookup).
- **EffectsCalculator**: high line coverage but no long-uptime tests (float timestamp precision, `micros()` wrap; see bug #4) and no test of `effectsForces` with a valid axis index > 0.

## Shared fakes and CMake changes (Phase 2 batch 0)

Rule for every row: after each CMake change rebuild and confirm `ffb_unit_tests` still gives 316 passed / 0 failed / 2 skipped with the exclusion; record the result in the Status column.

| # | Fake / change | Location | Purpose | Status |
|---|---|---|---|---|
| F1 | Pass the `effectsForces` exclusion in `add_test` (and a `FFB_TESTS_EXCLUDE` cache variable listing excluded case names; sanitizer builds add the 2 spline cases) | `Tests/CMakeLists.txt` | Make `ctest` green and the exclusions visible in one place | **done** 2026-10-21. Check: default ctest 1/1 passed, doctest 316 passed / 0 failed / 2 skipped; san ctest 1/1 passed, 313 / 0 / 5 skipped; direct run with exclusion 316 / 0 / 2 |
| F2 | Additional test executables next to the unchanged `ffb_unit_tests`: `ffb_axis_tests` (real `Axis.cpp`/`Axis.h`, `AxesManager`, `FFBHIDMain`, no `fakes/` dir), `ffb_flash_tests` (real `flash_helpers.cpp` + `eeprom.c` on fake HAL FLASH, without the `Flash_*` fake), later `ffb_usb_tests` (Tier B). Firmware sources compiled per executable; only the platform fakes are shared (e.g. an object/static library of `platform/src` pieces) | `Tests/CMakeLists.txt` | The fake `Axis.h` and fake `Flash_*` cannot coexist with the real code in one binary | approved; `ffb_unit_tests` must stay exactly as it is |
| F3 | Separate feature header for the new executables only (the existing `platform/include/target_constants.h` stays unchanged). Built by copying the feature macros of `Firmware/Targets/F407VG/Core/Inc/target_constants.h`, removing `TMC4671DRIVER`, `PWMDRIVER`, `ODRIVE`, `VESC`, `SIMPLEMOTION`, `RMDCAN`, `TMCDEBUG`, leaving `MIDI` off; plus host pin tables (`BUTTON_PINS`, `ADC_PINS`, ...) and a host `cpp_target_config` as needed | `Tests/platform/` (exact path decided in batch 0) | Compile the in-scope modules without the drivers | drafted 2026-10-21 at `Tests/platform/target_f407/target_constants.h` (include dir to be placed before `platform/include` in the new executables only); diff shown, **approved by me 2026-10-21** |
| F4 | Fake HAL SPI (records TX, scripted RX, DMA/IT completion callbacks driven by the test) | `Tests/platform/` | SPIButtons, EncoderSSI, EncoderBissC, MtEncoderSPI, SPI.cpp | proposed |
| F5 | Fake HAL I2C (scripted device register map per address, error injection) | `Tests/platform/` | PCF8574, ADS111X, I2C.cpp | proposed |
| F6 | Fake HAL ADC (settable DMA buffer, conversion-complete callback) + LL ADC voltage/temperature macros | `Tests/platform/` | LocalAnalog, AdcHandler, voltagesense | proposed |
| F7 | Fake HAL TIM (settable `CNT`, base start/stop record) and EXTI trigger | `Tests/platform/` | EncoderLocal, TimerHandler, ExtiHandler, Axis timers | proposed |
| F8 | Fake HAL CAN (filters, Tx mailboxes, Rx FIFO, error callbacks) | `Tests/platform/` | CANPort2B, global_callbacks | proposed |
| F9 | Fake HAL FLASH: RAM-backed sectors with real NOR semantics (erase to 0xFFFF, programming can only clear bits, error flags injectable), `__weak`, OTP area. **Constraint:** `eeprom.c` dereferences absolute addresses (`*(__IO uint16_t*)PAGE0_BASE_ADDRESS`, `EEPROM_START_ADDRESS = 0x08004000`, cast to `uint32_t`), so the fake must `mmap` 2 x 16 KiB at exactly 0x08004000 (`MAP_FIXED_NOREPLACE`, fail loudly if taken); the OTP area likewise at its fixed address. Host-only technique, verify under ASan | `Tests/platform/` | flash_helpers + eeprom.c, power-loss/page-transfer scenarios | proposed |
| F10 | Misc CMSIS/HAL: `__REVSH`, `NVIC_SystemReset` (recorded, no exit), `HAL_GetUIDw0/1/2`, `HAL_GetDEVID/REVID`, IWDG refresh, `HAL_RCC_GetHCLKFreq`, LED pins | `Tests/platform/` | CRC, SystemCommands, USBdevice, cppmain, ledEffects | proposed |
| F11 | FreeRTOS: `xQueue*`, `xTimer*`, `xEventGroup*`, `xTimerPendFunctionCall`, `tskIDLE_PRIORITY`, recursive mutex, counting semaphores (simple non-preemptive implementations, with create-failure injection) | `Tests/platform/` | Tier A tests of the FreeRTOS C++ wrappers; modules using queues/timers | proposed |
| F12 | New mocks: `MockButtonSource`, `MockAnalogSource`, `MockSPIDevice`/`MockI2CDevice` helpers, `AxisFixture` (real Axis + MockMotorDriver + MockEncoder selected via the chooser) | `Tests/mocks/`, `Tests/support/` | Caller-side tests at the interface | proposed |
| ~~F13~~ | ~~Compile CMSIS-DSP without ASan~~ | — | Dropped 2026-10-21: ASan stays on everywhere, the 2 spline cases are excluded in sanitizer runs instead | dropped |
| F14 | `-include cmath` (per-source compile option) for `Axis.cpp` if it still does not compile (README says it lacks `<cmath>`) | `Tests/CMakeLists.txt` | No firmware edit allowed | proposed, verify first |
| F15 | Fake USB device controller: TinyUSB `dcd_*` port functions (`dcd_init`, `dcd_edpt_open`, `dcd_edpt_xfer`, `dcd_event_*` injection), host-side helpers to send SETUP packets and collect IN transfers. Only for `ffb_usb_tests`; the existing `tud_*` fakes stay in `ffb_unit_tests` | `Tests/platform/` | Tier B TinyUSB tests | proposed |
| F16 | `Tests/tools/coverage_report.py` (stdlib only): runs/reads `gcov` JSON or text output and aggregates per module (firmware, Tier A, B, C separately; excludes `Tests/`, drivers, untested vendor code) | `Tests/` | Phase 4 coverage report | proposed |

## Test plan (priority order, highest risk first)

Each batch = one subagent per module group, own test files `cases/test_<module>.cpp`. Fakes only from batch 0.

| Prio | Module | What will be tested | Faked | Excluded and why |
|---|---|---|---|---|
| 1 | **Axis** (`ffb_axis_tests`) | Torque path end to end with `MockMotorDriver`/`MockEncoder`: power scaling and clipping to ±0x7fff, invert, degrees/range scaling of position, endstop torque (gain, beyond-range behaviour), speed limiter, torque slew-rate limiter, idle spring, damper/friction/inertia strengths, expo curve, gear ratio, effect ratio, force fade-in, emergency stop/reset, hands-off detection, `encodeConfToInt`/`decodeConfFromInt` round trip, save/restore flash, commands (get/set/limits, invalid values), driver/encoder selection via chooser | Motor driver + encoder (mocks), TIM/EXTI, flash map | Real driver classes (compiled out) |
| 1 | **AxesManager / FFBHIDMain** | Axis count changes, report building of the HID gamepad (button/axis packing into `reportHID_t`, double-buffered report), FFB on/off, USB suspend/resume handling, command handling, update rate | Button/analog mocks, tusb HID (existing), TIM | Real USB stack |
| 2 | **CMSIS-DSP (vendor tier C)** | Only the functions in the tier C list: accuracy of `arm_sin_f32`/`arm_cos_f32`/`arm_sqrt_f32` against `<cmath>` over the input ranges the firmware uses, biquad DF1 against a reference difference equation, spline natural cubic on 4 points incl. `blockSize = 1` (bug #2 path: sanitizer runs exclude it), `arm_pid_*`, `arm_std_f32` | none | Bugs reported, never fixed |
| 2 | **Filters / FastAvg** (DSP ON and OFF) | Biquad lowpass/highpass/notch/peak/lowshelf/highshelf coefficients against reference values, DC gain, frequency response at fc, reset, invalid q/fc; `FastAvg` window, `FastMovingAverage` | none | — |
| 2 | **EffectsCalculator gaps** | Reconstruction after long uptime (float timestamps), `micros()` wrap-around, `effectsForces` for axis 1 (valid); no test of the excluded OOB case | existing | `effectsForces` axis ≥ MAX_AXIS (excluded, bug #1) |
| 3 | **Persistence** (`ffb_flash_tests`) | `Flash_Init`/`Write`/`Read`/`ReadWriteDefault`/`Dump`/`Format`, write-only-if-changed, missing variable, page full -> page transfer keeps only addresses in `VirtAddVarTab`, OTP read/write, `PersistentStorage` restore at startup and delayed restore; `VirtAddVarTab` has `NB_OF_VAR` unique entries and no 0xFFFF | Fake HAL FLASH (F9) | — |
| 3 | **`eeprom.c` (vendor tier A)** (`ffb_flash_tests`) | `EE_Init` on erased/valid/receive-data/invalid page header combinations (power-loss states), `EE_ReadVariable`/`EE_WriteVariable` incl. missing variable, page full -> `EE_PageTransfer`, `EE_Format`, HAL FLASH error returns propagated | Fake HAL FLASH (F9) | Bugs reported, never fixed |
| 4 | **ErrorHandler** | Add/clear/dedupe errors, max error count overflow, callbacks to all handlers, `Error::toString`, error commands/list | none | — |
| 4 | **SystemCommands** | Each command: help/info, `hwtype`, `fw` version, flash dump/raw read/write/format via the flash map, reset requests recorded (no exit), UID, main class selection, `vint`/`vext`, error list, debug/heap | NVIC, UID, RTOS stats | — |
| 5 | **Analog processing** | `AnalogAxisProcessing` autoscale, manual min/max, invert, filters, raw mode, conf encode/decode round trip, save/restore; `LocalAnalog` mask/channel mapping; `CanAnalog` frame decoding; `ADS111X` register decoding | ADC (F6), I2C (F5), CAN (`MockCANPort`) | — |
| 5 | **Buttons / shifter** | `LocalButtons` pin mask + polarity, `SPIButtons` bit order/cs/inversion/chain length, `ShifterAnalog` H-pattern and sequential gear decoding incl. reverse button and thresholds, `CanButtons` frames, `PCF8574` I2C decoding, `SelectableInputs` source selection | GPIO (existing), SPI (F4), I2C (F5), CAN | — |
| 6 | **Encoders (non-driver)** | `EncoderLocal` counts/index/rollover, `EncoderSSI` bit extraction, `EncoderBissC` CRC6 + error/warning bits + multiturn, `MtEncoderSPI` parity/status, `CRC` tables against known check values (CRC-8, CRC-16 variants) | SPI (F4), TIM (F7) | Driver encoders (TMC/ODrive/VESC/RMD/SimpleMotion) |
| 6 | **SerialFFB** | Effect commands over the string interface, effect allocation/free, torque passed to effects calculator | existing | — |
| 7 | **Comms gaps** | `CanBridge` frame <-> host protocol, `CanInputMain`, `CANPort2B` filter setup/tx/rx/bus-off handling, `CanHandler` registration, `global_callbacks` dispatch to registered handlers, `ringbuffer` wrap-around/overflow/peek, UART/CDC remaining paths, `UsbHidHandler` | CAN HAL (F8), existing tusb fakes | TinyUSB itself |
| 8 | **Misc** | `ledEffects` blink patterns vs simulated time, `voltagesense` ADC→mV scaling and over-voltage brake/emergency, `cppmain` `micros()` variants, `mainclass_chooser` / `ClassChooser` IDs unique and selectable, USB descriptor tables self-consistent (`wTotalLength`, HID report descriptor length, report IDs match `ffb_defs.h`) | ADC (F6), GPIO | `Targets/**`, `main.c` |
| 8 | **FreeRTOS C++ wrappers (vendor tier A)** | `Thread` create/delete/delayUntil, `Semaphore`/`Mutex`/recursive mutex take/give incl. ISR variants and timeouts, `Queue` send/receive/peek/overwrite, `Timer` start/stop/change period, `Tasklet`, `EventGroups`, `WorkQueue`, `MemoryPool`, read/write lock, condition variable, tick hook | Fake FreeRTOS (F11) | Real preemption; bugs reported, never fixed |
| 9 | **TinyUSB device stack (vendor tier B)** (`ffb_usb_tests`) | Enumeration against the firmware's descriptors (device, config, string, HID report), standard control requests (GET_DESCRIPTOR, SET_ADDRESS, SET_CONFIGURATION), HID class requests (GET_REPORT/SET_REPORT/SET_IDLE) reaching `HidFFB`/`HID_CommandInterface`, CDC line coding and data path, suspend/resume callbacks | Fake USB controller (F15) | Host stack, non-used classes, `portable/**`; bugs reported, never fixed |
| Phase 5 | **Python helpers (vendor tier A)** | `generate_memory.py` (CSV -> `eeprom_addresses.c/.h` generation, matches the checked-in files), `dwc2_info.py` | pytest outside CMake (ask before installing pytest) | — |

## NEEDS REVIEW items

| # | Module | Test / location | Question | Resolved? |
|---|---|---|---|---|
| 1 | CAN command interface | `CanCommandInterface.cpp:118-124` | `msg.header.length >= 16/24` paths read `msg.data[8..23]` of an 8-byte array. Dead on classic CAN, out of bounds for any frame with length > 8. Intended for CAN-FD later? | no |
| 2 | EffectsCalculator | `evaluateReconstructionFilter` (`EffectsCalculator.cpp:584, 624-633`) | Timestamps are stored as `float` microseconds. Above 2^24 µs (≈16.8 s uptime) resolution drops (128 µs at ≈18 min, 512 µs near the 71.6 min `uint32_t` wrap). Also `now_us > last_known_time + 50000` misbehaves at the `micros()` wrap. Intended? To be confirmed by a test in batch 2 | no |
| 3 | Existing suite | 17 `KNOWN_ISSUE` cases already in `Firmware/Tests/cases/` (CmdParser 4, EffectsCalculator 5, HidFFB 5, CAN/HID command interface 2 x "ERR never reported", StringCommandInterface 1) | Carried over unchanged; listed with `grep -rn KNOWN_ISSUE Firmware/Tests/cases/` | n/a |

## Suspected firmware bugs

| # | Module | Location | Description | Evidence |
|---|---|---|---|---|
| 1 | EffectsCalculator | `FFBoard/Src/EffectsCalculator.cpp:1048` (`effectsForces`, `std::min<uint8_t>(cmd.adr, MAX_AXIS)`), read at line 1053 `effects_statslast[i].current[axis]` | Clamps the axis index to `MAX_AXIS` instead of `MAX_AXIS-1`, then reads `current[MAX_AXIS]` (`std::array<int16_t,2>`) out of bounds. Silent under old libstdc++, aborts under GCC 16 `_GLIBCXX_ASSERTIONS` at -O0 | Existing test `requesting the forces of a not existing axis is rejected` (`cases/test_effects_calculator.cpp:1062`, already `KNOWN_ISSUE_UB`). Message: `/usr/include/c++/16/array:210: ... std::array<short int, 2>::operator[] ... Assertion '__n < this->size()' failed.` -> SIGABRT after 192 of 317 cases. Excluded with `-tce` |
| 2 | CMSIS-DSP (**vendor, tier C**, report only, never fix) reached from EffectsCalculator | `Libraries/CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_f32.c:223`, called from `EffectsCalculator.cpp:695` with `blockSize = 1` | `while( *pXq <= x[i+1] && blkCnt > 0 )` dereferences `pXq` before checking `blkCnt`, so it reads one float past the 1-element input (`interp_time`, a stack variable). Bug is in the vendor library (tier C: reported, never fixed); the firmware call is valid per the API. On target it reads one adjacent stack word; the result is not used, so likely harmless | ASan stack-buffer-overflow (GCC and clang). Sanitizer runs exclude the 2 cases with `-tce` (ASan stays on for CMSIS-DSP), test cases `reconstruction of a constant signal is constant` and `reconstruction of a step never leaves the range of the input values and reaches the target`. Not visible without ASan, not in DSP-OFF build. Possible firmware-side workaround (not to be applied by me): pass a 2-element buffer |
| 3 | CommandHandler | `FFBoard/Src/CommandHandler.cpp:136` `snprintf(cmdhex,11,"0x%lX",cmd.cmdId)` | `%lX` with `uint32_t`: correct on ARM (where `uint32_t` is `unsigned long`), mismatched type on the 64-bit host. Portability issue rather than a target bug (`PRIX32` would fit both) | GCC `-Wformat` and clang `-Wformat` warnings |
| 4 | EffectsCalculator | see NEEDS REVIEW #2 | Possible precision/wrap problem of the reconstruction filter after long uptime | Code reading only; to be demonstrated by a test in batch 2 |

## Untestable code and refactor suggestions

| Module | Location | Why untestable | Suggested refactor |
|---|---|---|---|
| MidiMain (out of scope, note only) | `UserExtensions/Src/MidiMain.cpp` | Constructs `TMC_1` directly (no `MotorDriver` interface use) | Take the driver from a chooser or as `MotorDriver*` |
| Concurrency | all ISR/thread interaction | No preemption on the host | Keep ISR handlers small and testable as functions |
| CubeMX init | `Targets/*/Core/Src/*` | Hardware init only | — |

## Host-vs-target caveats

| Module | Issue (size, packing, endianness, float/double, overflow, shift) | Test affected |
|---|---|---|
| all | Float -> int conversion of out-of-range values: ARM VCVT saturates, x86-64 SSE returns 0x80000000. Tests must not depend on out-of-range casts | Axis torque clipping, effect calculations |
| all | `long`/`size_t`/pointers are 64-bit on host, 32-bit on target. `int`/`int32_t` are 32-bit on both | `CommandHandler.cpp:136` `%lX`; `CommandReply::size()` uses `sizeof(CommandReply)` (contains `std::string`: 32 B host vs 24 B target), so reply memory accounting differs |
| HID reports | `__attribute__((packed))` structs in `ffb_defs.h`, `HidCommandInterface.h`: same layout with GCC on little-endian Linux; MinGW needs `-mno-ms-bitfields` (already set) | HID report tests |
| all | Implicit `double` promotion (e.g. `INTERNAL_AXIS_DAMPER_SCALER 0.7`, unsuffixed literals): target computes doubles in software; IEEE results are the same on host (SSE, no x87), so values match but timing does not | Axis/effects numeric tests: compare with tolerances |
| DSP | `USE_DSP_FUNCTIONS` uses the portable C CMSIS-DSP on the host, not the Cortex-M4 optimised paths. Small float differences possible | Filters, effects: use tolerances, run both DSP ON and OFF |
| Endianness | Both little-endian. `memcpy` of CAN payloads (`CanCommandInterface.cpp`) behaves the same | CAN tests |
| Signed shifts | Arithmetic right shift on both (GCC/clang) | Encoder decoding |

## Review outcomes (Phase 3)

| Module | Reviewer | Findings | Fixed? |
|---|---|---|---|
| | | | |

## Verification results (Phase 4)

- Full suite result (with `effectsForces` exclusion listed):
- ASan / UBSan findings:
- Coverage per in-scope module:
- Uncovered code explained:
- Mutation check (caught / missed):
- `git diff` under `Firmware/` empty except `Firmware/Tests/`: yes / no
- No build artefacts written under `Firmware/`: yes / no

## Open questions for me

Q1-Q8 answered on 2026-10-21 (see decisions log).

A. Commit name: answered 2026-10-21, `user.name=MadBull.Ben`.

## Resume instructions

Start a new Claude Code session from the repo root and say:

> Read CLAUDE.md, Tests/TEST_PLAN.md and Tests/PROGRESS.md. Resume from "Next action". Do not redo completed work, and do not move past an approval gate without my go-ahead.
