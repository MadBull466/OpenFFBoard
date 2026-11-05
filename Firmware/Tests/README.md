# OpenFFBoard firmware unit tests

Functional unit tests for the hardware independent parts of the firmware.
The tests run on the development PC. No board, debugger or ARM toolchain is required. They are meant to run in GitHub
Actions as well, but no workflow builds or runs them yet: the CI job is a proposal in `Tests/claude/plans/PROGRESS.md` (open
question 5).

## Concept

The **unmodified firmware sources** (`FFBoard/Src/*.cpp`) are compiled with the native compiler of the PC
and linked against a **fake platform** instead of the STM32 HAL, FreeRTOS and TinyUSB:

```
 test cases (cases/)         what should happen
     |
 mocks (mocks/)              MockCommandHandler, MockMotorDriver, MockEncoder, MockCANPort ...
 support (support/)          fixtures, simulated PID host, test control of the fake platform
     |
 firmware sources            CmdParser, CommandInterface, HidFFB, EffectsCalculator ... (real code)
     |
 fake platform (platform/)   HAL, FreeRTOS, TinyUSB, flash, clock (recording fakes)
```

Properties of the fake platform:

* **Time is simulated.** `HAL_GetTick()` and `micros()` only change when a test advances the clock
  (`HostPlatform::advanceMs`). Tests are deterministic and do not wait.
* **No scheduler.** Threads are registered but only run when a test executes them until they block
  (`HostRtos::runUntilBlocked`, `HostRtos::runUntilIdle` or `process()` of the fixtures). A blocked thread is restarted
  at the beginning of its function the next time; threads that wait in several places per loop are made resumable
  (`HostRtos::makeResumable`: the thread runs on its own host thread, strictly one at a time, and continues after the
  blocking call). `std::thread` was already required before P6 (`TestHelpers.h` `finishesWithin`, `Threads::Threads`
  linked by every executable), so the MinGW thread-model requirement is unchanged. Secondary threads have smaller
  stacks than the main thread on some hosts (macOS: 512 KiB). An infinite semaphore take from the test thread that
  would block forever on the target is recorded (`HostRtos::kernelFaults()`; in `ffb_target_tests` it fails the test
  like an unmodelled HAL call).
* **Everything sent to the outside is recorded**: CDC data and HID reports (`HostUsb`), uart bytes,
  flash content and LEDs (`HostPlatform`), CAN frames (`MockCANPort`), motor torque (`MockMotorDriver`).

Framework: [doctest](https://github.com/doctest/doctest) 2.4.11 (single header, MIT license, `third_party/doctest.h`).

## Running the tests

Requirements: CMake >= 3.16 and gcc/g++ with C++20 support (MinGW-w64 on Windows).

```sh
cd Firmware/Tests
cmake -S . -B build                # Windows with MinGW: add  -G "MinGW Makefiles"
cmake --build build -j
ctest --test-dir build --output-on-failure
```

or with presets (CMake >= 3.21): `cmake --preset host && cmake --build --preset host && ctest --preset host`
(`host-mingw` on Windows).

`ctest` runs 9 entries: the seven test executables `ffb_unit_tests`, `ffb_target_tests`, `ffb_flash_tests`,
`ffb_rtos_tests`, `ffb_usb_tests`, `ffb_driver_tests` and `ffb_dsp_tests` (each with its exclusion list), `ffb_unit_tests_baseline` (the baseline check, see
below) and `python_helper_tests` (the last two only if Python 3 is found; `ffb_flash_tests` is not built on macOS;
`ffb_dsp_tests` only with CMSIS-DSP and its `Testing/Patterns`, so 8 entries without them).

| Executable | Test cases | Section |
|---|---|---|
| `ffb_unit_tests` | `cases/` | this section (the original suite) |
| `ffb_target_tests` | `claude/cases_target/` | [`ffb_target_tests`](#ffb_target_tests-f407-feature-set-with-the-real-axis) |
| `ffb_flash_tests` | `claude/cases_flash/` | [`ffb_flash_tests`](#ffb_flash_tests-the-real-flash-layer-on-a-fake-hal-flash) |
| `ffb_rtos_tests` | `claude/cases_rtos/` | [`ffb_rtos_tests`](#ffb_rtos_tests-the-freertos-c-wrappers-on-a-fake-freertos-api) |
| `ffb_usb_tests` | `claude/cases_usb/` | [`ffb_usb_tests`](#ffb_usb_tests-the-tinyusb-device-stack-on-a-fake-usb-device-controller) |
| `ffb_driver_tests` | `claude/cases/driver/` | [`ffb_driver_tests`](#ffb_driver_tests-the-tmc4671-odrive-and-vesc-drivers-driver-track) |
| `ffb_dsp_tests` | `claude/cases_dsp/` | [`ffb_dsp_tests`](#ffb_dsp_tests-the-cmsis-dsp-modules-phase-p11) |
| `python_helper_tests` | `python/` | [Python helper tests](#python-helper-tests) |

The test executables can be started directly for more control. With GCC 16 (libstdc++ with `_GLIBCXX_ASSERTIONS` at
`-O0`) a run of `ffb_unit_tests` without the exclusion aborts in the case
`requesting the forces of a not existing axis is rejected` (suspected bug 1), so pass the exclusion of `ctest` (in a
sanitizer build with CMSIS-DSP also the two spline cases of `FFB_TESTS_EXCLUDE_SANITIZE`):

```sh
build/ffb_unit_tests -tce="requesting the forces of a not existing axis is rejected"   # all other tests, prints details of known issues
build/ffb_unit_tests -ts=CmdParser          # one test suite
build/ffb_unit_tests -tc="*sine*"           # test cases by name
build/ffb_unit_tests --list-test-cases
build/ffb_unit_tests -ts=CmdParser -s       # also print successful assertions
build/ffb_target_tests                      # the other executables need no exclusion by default
```

Expected result of `ffb_unit_tests` with the documented exclusions (GCC 16, Linux): 646 passed, 0 failed, 4 skipped
(the same with clang 23 and without CMSIS-DSP; sanitizer build with CMSIS-DSP: 643 / 0 / 7; sanitizer build without
CMSIS-DSP: 645 / 0 / 5). doctest counts a
`KNOWN_ISSUE` case that fails as expected as passed. The 318 baseline cases are a subset of these; their own result
(316 passed, 2 skipped on GCC 16 with the exclusion) is checked by `tools/check_baseline.py` (see below).

### Build options

| Option | Default | Description |
|---|---|---|
| `FFB_TESTS_USE_DSP` | `AUTO` | Build with `USE_DSP_FUNCTIONS` and CMSIS-DSP like the hardware targets. `AUTO` enables it if the `Firmware/Libraries/CMSIS-DSP` submodule is checked out. With `OFF` the fallback math of the firmware is tested. Both variants must pass. |
| `FFB_TESTS_SANITIZE` | `OFF` | Address and undefined behaviour sanitizers (Linux/macOS). |
| `FFB_TESTS_EXCLUDE` | the `effectsForces` case | Test cases `ctest` excludes (`--test-case-exclude`). Only cases that abort the whole run because of a suspected defect belong here. |
| `FFB_TESTS_EXCLUDE_SANITIZE` | the 2 spline cases | Additionally excluded in sanitizer builds with CMSIS-DSP (the address sanitizer reports an over-read inside `arm_spline_f32`). |
| `FFB_TARGET_TESTS_EXCLUDE`, `FFB_FLASH_TESTS_EXCLUDE`, `FFB_RTOS_TESTS_EXCLUDE`, `FFB_USB_TESTS_EXCLUDE` and the `..._SANITIZE` variants | empty | The same for the other executables (see their sections). |

The active exclusions are printed when CMake configures (`OpenFFBoard tests: excluded from ctest: ...`).
When running the executable directly, pass the same list, e.g.
`build/ffb_unit_tests --test-case-exclude="requesting the forces of a not existing axis is rejected"`.

doctest splits the values of `-tc` / `-tce` / `-ts` / `--test-case-exclude` at commas, and the exclusion options are
passed to doctest the same way, so a test name that contains a comma can not be given verbatim (in a filter or in an
`FFB_*_EXCLUDE` list). Replace the comma with `?` (doctest's single character wildcard), e.g.
`-tc="constant force? spring and damper are summed"` (with the comma the filter matches nothing).

### Baseline check and coverage

`ctest` also runs `tools/check_baseline.py` (if Python 3 is found). It checks that every test case listed in
`baseline/test_cases.txt` still exists with its expected result (passes, fails as a known issue, or is skipped).
New test cases are allowed. Run it directly with `python3 -B tools/check_baseline.py --build-dir build`.

Line coverage per file, module and vendor code, with plain `gcov` (no other tools needed):

```sh
cmake -S . -B build-cov -DCMAKE_C_FLAGS=--coverage -DCMAKE_CXX_FLAGS=--coverage -DCMAKE_EXE_LINKER_FLAGS=--coverage
cmake --build build-cov -j && ctest --test-dir build-cov
python3 -B tools/coverage_report.py --build-dir build-cov
```

Code that can only be checked on the board (register setup, busy waits on peripheral flags, interrupt timing) is
not tested. It is listed with a reason in `tools/coverage_exclusions.txt` and left out of the coverage numbers
(`--no-exclusions` shows the raw numbers).

### Full report (`test_report`)

`ctest` keeps its exclusions and skips. For a complete picture, `tools/run_all_tests.py` runs **every** test case of
every test executable in a build directory plus the Python helper tests and writes a plain text log:

```sh
cmake --build build --target test_report                      # builds the executables, writes build/test_report.log
python3 -B tools/run_all_tests.py --build-dir build [--out FILE] [--timeout S]   # the same directly
```

* **Nothing is skipped.** The executables run with the doctest XML reporter without the ctest exclusions. Cases with
  a skip decorator (skipped crash known issues, `KNOWN_ISSUE_UB` / `DELETES_FFBHIDMAIN` in sanitizer builds) run one
  by one in their own process with `--no-skip` and a name filter that is checked to select exactly that case. If a run
  stops in a case (signal, abort, sanitizer exit, or no progress for `--timeout` seconds, default 60), that case is
  recorded as crashed / timed out and the run continues after it (`--first`).
* **Passed / Failed / Bugged.** *Bugged*: a known-bug case (`KNOWN_ISSUE`, `KNOWN_ISSUE_UB`, skipped crash case,
  Python `@expectedFailure`) that fails as expected (assertion, crash or timeout); a `KNOWN_ISSUE_UB_HIDDEN` case
  ("UB, passes on the host"); a case of the ctest exclusion list that fails; in sanitizer builds a
  `DELETES_FFBHIDMAIN` case that fails (bug 24). *Failed*: every other failure, and a known bug that passes now
  ("fixed? remove the marker"). *Passed*: the rest; `characterization:` cases are also listed as open questions.
* **Log** (`<build-dir>/test_report.log`): date, commit, build configuration, the summary (Passed, Failed, Bugged,
  per executable), then the failed tests, the bugged tests grouped by the file under test and the open questions.
  Every entry gives the test location, the file under test, the bug / vendor / NEEDS REVIEW number with its
  description from `Tests/claude/plans/PROGRESS.md`, and the first failing assertion or the crash.
* **File under test:** the source files of the numbered row in `Tests/claude/plans/PROGRESS.md`; otherwise a firmware source
  named like the test suite or the test file (`test_<name>[_gaps].cpp`, also a source name the file name starts
  with) or an included header matching the file name; else `unknown`.
* A failed test is reported as failed and the run carries on with the next test case. The `test_report` target always succeeds (`--exit-zero`); run directly, the script exits with 0 when Failed is 0 and 1 otherwise (for CI), unless `--exit-zero` is given. Not part of `ctest`; `claude/python/test_run_all_tests.py` tests the script itself.

## Directory layout

| Path | Content |
|---|---|
| `cases/test_*.cpp` | The original 318 baseline cases of `ffb_unit_tests`. Every file is picked up automatically (together with `claude/cases/test_*.cpp`). |
| `claude/cases/test_*.cpp` | Claude's gap tests of `ffb_unit_tests`. |
| `claude/python/` | Python helper tests (`python_helper_tests`). |
| `codex/` | Codex's test cases and fakes (own folder; see `AGENTS.md`). |
| `claude/cases_target/test_*.cpp` | Test cases of `ffb_target_tests` (F407 feature set, real `Axis`). |
| `claude/cases_flash/test_*.cpp` | Test cases of `ffb_flash_tests` (real `flash_helpers.cpp` and `eeprom.c` on the fake HAL FLASH). |
| `claude/cases_rtos/test_*.cpp` | Test cases of `ffb_rtos_tests` (the `cpp_freertos` wrappers on the fake FreeRTOS API). |
| `claude/cases_usb/test_*.cpp` | Test cases of `ffb_usb_tests` (TinyUSB device stack on the fake USB device controller). |
| `claude/cases/driver/test_*.cpp` | Test cases of `ffb_driver_tests` (TMC4671, ODrive, VESC drivers; not picked up by `ffb_unit_tests`, whose glob is not recursive). |
| `claude/cases_dsp/test_dsp_*.cpp` | Test cases of `ffb_dsp_tests` (CMSIS-DSP modules against ARM's reference patterns). |
| (`Tests/claude/charters/dsp/`) | Requirement charter of the CMSIS-DSP track (`CMSIS_DSP_REQUIREMENTS.md`); documents live under `Tests/`, not here. |
| (`Tests/claude/charters/driver/`) | Requirement charters of the driver track (`<name>_REQUIREMENTS.md`). |
| `specs/` | Local copies of vendor specifications for the driver track. Ignored by git (`.gitignore`), never committed. |
| `python/` | `test_*.py`: stdlib `unittest` tests of the Python helper scripts; `helper_paths.py` (paths, loader). |
| `mocks/` | Implementations of firmware class interfaces for tests (`Mock*.h`, see "Mocks and helpers"). |
| `fakes/` | Headers replacing firmware headers in `ffb_unit_tests` only (currently only `Axis.h`). |
| `support/` | Fixtures and test control of the fakes, see the table below. |
| `platform/include` | Fake `main.h` (HAL), FreeRTOS, CMSIS headers and the `target_constants.h` of the generic test "target" (`ffb_unit_tests`, `ffb_rtos_tests`). |
| `platform/target_f407/` | F407 feature header (`target_constants.h` without drivers and MIDI), F407 fake `main.h`, `cmsis_os.h` and the fake HAL FLASH declarations `host_flash_hal.h`; first in the include path of `ffb_target_tests`, `ffb_flash_tests` and `ffb_usb_tests`. |
| `platform/target_f407_drivers/` | `target_constants.h` of `ffb_driver_tests`: the F407 test header plus `TMC4671DRIVER`, `ODRIVE`, `VESC`; first in its include path. |
| `platform/usb/` | `tusb_config.h` wrapper of the F407 one and the OSAL headers (`hosttest_osal/`) of `ffb_usb_tests`. |
| `platform/mingw/` | `malloc.h` shim and `host_mallinfo.cpp` (`struct mallinfo` / `mallinfo()` for `SystemCommands.cpp`), used only on MinGW and macOS (`ffb_target_tests`). |
| `platform/src` | Implementation of the fake platform (`host_platform.cpp`, `host_freertos.cpp` + `host_freertos_ext.inc`, `host_usb.cpp`, `host_stubs.cpp`, `host_target*.cpp`, `host_flash.cpp`, `host_dcd.cpp`, `host_usb_osal.cpp`, and the compile wrappers `host_cppmain.cpp`, `host_cmem_pool.cpp`). |
| `third_party/doctest.h` | doctest 2.4.11. Do not edit. |
| `CMakeLists.txt`, `CMakePresets.json` | Build of all executables (lists of the firmware sources under test), presets. |
| `baseline/test_cases.txt` | The test cases that existed before the suite was extended, with their expected result. Do not edit. |
| `tools/check_baseline.py` | Baseline check (also a `ctest` entry). |
| `tools/coverage_report.py` | gcov line coverage per file, module and tier; the driver files (`TMC4671*.cpp`, `ODriveCAN.cpp`, `VescCAN.cpp`) have their own section outside the ladder total. |
| `tools/coverage_exclusions.txt` | Reviewed list of hardware-only line ranges (file:range + reason) left out of the coverage numbers. |
| `tools/summarise_results.py` | Condenses a doctest console log, doctest XML report or ctest log (or a build log) into counts, failures with the first failed assertion, skipped and excluded cases and build errors. `claude/python/test_summarise_results.py` tests it. |
| `tools/run_all_tests.py` | Full report: runs every test case (nothing skipped) and writes `test_report.log` (target `test_report`). |
| `tools/make_bug_report.py` | Generates `Tests/claude/reports/BUG_REPORT.md` (repository root) from the "Suspected firmware bugs" and "NEEDS REVIEW items" tables of `Tests/claude/plans/PROGRESS.md`. |

The Python tools use the standard library only; run them with `python3 -B` (no `__pycache__` under `Firmware/`).

Headers in `support/`:

| Header | Executable | Content |
|---|---|---|
| `TestHelpers.h` | all | `KNOWN_ISSUE`, `KNOWN_ISSUE_UB`, `KNOWN_ISSUE_UB_HIDDEN`, `finishesWithin`, small helpers. |
| `HostPlatform.h` | all | Simulated time (`advanceMs`), interrupt context, LEDs, UART, flash map of `ffb_unit_tests`. |
| `HostRtos.h` | all | Fake FreeRTOS kernel: thread registry, `runUntilBlocked` / `runUntilIdle`, resumable tasks, kernel faults, heap counters. |
| `HostUsb.h` | `ffb_unit_tests`, `ffb_target_tests` | Recording TinyUSB fake (`platform/src/host_usb.cpp`): CDC data, HID reports. |
| `FirmwareFixture.h` | `ffb_unit_tests`, `ffb_target_tests` | `FirmwareFixture`, `CommandSystemFixture`. |
| `FfbFixture.h`, `PidHost.h` | `ffb_unit_tests` | `FfbFixture`, `FfbFixture1Axis`; simulated PID host (also used by `AxisFixture`). |
| `AxisFixture.h` | `ffb_target_tests` | `TargetFixture`, `AxisFixture`, `DELETES_FFBHIDMAIN`, `REQUIRE_ALL_HAL_MODELLED`. |
| `HostTarget.h` | `ffb_target_tests`, `ffb_usb_tests` | F407 fake platform: timers, EXTI, GPIO, ADC, SPI, I2C, CAN, OTP, `runCppmain`, unmodelled-HAL record. |
| `CommandHelpers.h` | `ffb_target_tests` | Run commands of a handler by name (`run`, `get`, `getAt`), `CdcConsole`. |
| `InputSources.h` | `ffb_target_tests` | Mock button / analog sources in the choosers of `SelectableInputs`. |
| `FlashFixture.h`, `HostFlash.h` | `ffb_flash_tests` | `FlashFixture`; fake HAL FLASH memory, recording, error and power-loss injection. |
| `RtosFixture.h`, `HostRtosExt.h` | `ffb_rtos_tests` (`HostRtosExt.h` also `ffb_usb_tests`) | `RtosFixture`; fake FreeRTOS API F11 (queues, timers, event groups, notifications, create failures). |
| `UsbFixture.h`, `UsbHost.h`, `HostDcd.h` | `ffb_usb_tests` | `UsbFixture`; simulated USB host; fake device controller. |
| `test_main.cpp` | all | doctest main. |

## Test suites

`ffb_unit_tests` (`cases/`):

| Suite | File | Tested firmware |
|---|---|---|
| `CmdParser` | `test_cmdparser.cpp` | String command syntax, addressing, buffering, malformed input |
| `CommandExecution`, `CommandHandler` | `test_command_execution.cpp` | Command thread, access flags, internal commands, reply distribution, handler registry |
| `StringCommandInterface`, `CDC_CommandInterface`, `UART_CommandInterface` | `test_string_command_interfaces.cpp` | Reply formatting, end to end tests text in -> text out |
| `HID_CommandInterface` | `test_hid_command_interface.cpp` | Vendor HID command reports |
| `CAN_CommandInterface` | `test_can_command_interface.cpp` | CAN command frames |
| `HidFFB` | `test_hid_ffb.cpp` | HID PID report handling, effect allocation, axis magnitudes from directions |
| `EffectsCalculator` | `test_effects_calculator.cpp` | Force calculation of all effect types, gains, envelope, filters, settings |
| `Encoder`, `MotorDriver` | `test_device_mocks.cpp` | Base class interfaces and the mocks |
| `platform` | `test_platform.cpp` | Self test of the fake platform |
| `CmdParser`, `CommandHandler`, `CommandInterface`, `CDC_CommandInterface`, `UART_CommandInterface`, `HID_CommandInterface` | `test_cmdparser_gaps.cpp`, `test_command_handler_gaps.cpp`, `test_hid_command_interface_gaps.cpp` | Buffer boundaries, malformed addresses, logging, help, flag overrides, reply buffering, HID reply branches |
| `EffectsCalculator`, `HidFFB` | `test_effects_calculator_gaps.cpp`, `test_hid_ffb_gaps.cpp` | Square phase, rate measurement, effects thread, settings commands, filter set-up on effect start and type change |
| `ErrorHandler`, `ErrorPrinter` | `test_error_handler.cpp` | Error list, clearing, sorting, callbacks, error printer thread |
| `PersistentStorage` | `test_persistent_storage.cpp` | Save and restore over all handlers, delayed restore at startup |
| `Filters` | `test_filters.cpp` | Biquad coefficients of all types against reference formulas, frequency response, setters (CMSIS-DSP on and off) |
| `Encoder`, `MotorDriver` | `test_device_interfaces_gaps.cpp` | Default implementations of the base classes |
| `cpp_freertos` | `test_rtos_wrappers.cpp` | FreeRTOS C++ wrappers (vendor): thread, mutex, semaphore |
| `UART`, `UartHandler` | `test_uart.cpp` | UART port transfers (blocking, IT, DMA), reservation, abort, completion dispatch |
| `ringbuffer`, `RingBufferWrapper` | `test_ringbuffer.cpp` | Ring buffer append, get, peek, wrap-around, full buffer |
| `CANPort`, `CanHandler` | `test_can_handler.cpp` | Port and handler registry, `handleToPort` (dispatch through the real `global_callbacks.cpp` is in `ffb_target_tests`) |
| `UsbHidHandler`, `CDCcomm` | `test_usb_hid_handler.cpp`, `test_cdc.cpp` | HID handler registration and descriptor, CDC send, retry and blocking |

`ffb_target_tests` (`claude/cases_target/`, one row per file group):

| Suites | Files | Tested firmware |
|---|---|---|
| `TargetSetup`, `SystemSetup`, `InputSetup`, `EncoderSetup`, `CanSetup` | `test_target_setup.cpp`, `test_system_setup.cpp`, `test_input_setup.cpp`, `test_encoder_setup.cpp`, `test_can_setup.cpp` | Smoke tests of the F407 fake platform per phase, self-tests of its helpers |
| `Axis`, `AxesManager`, `FFBIntegration` | `test_axis_config.cpp`, `test_axis_torque.cpp`, `test_axes_manager.cpp`, `test_ffb_integration.cpp` | Axis commands and flash, torque path (power, scaling, limiters, fade, hands-off, E-stop), axes manager, PID report to motor torque |
| `FFBHIDMain`, `FFBWheel`, `FFBJoystick`, `FFBHIDExt`, `FFBoardMain`, `CustomMain`, `MainClassChooser` | `test_ffbhidmain.cpp`, `test_main_classes.cpp` | Main classes: HID gamepad report, rates, E-stop, errors, USB suspend / resume, input sources, chooser |
| `SystemCommands`, `cppmain`, `AdcHandler`, `USBdevice`, `GlobalCallbacks`, `LedEffects`, `VoltageSense`, `TimerHandler`, `ExtiHandler`, `UsbDescriptors`, `CRC` | `test_system_commands.cpp`, `test_cppmain.cpp`, `test_global_callbacks.cpp`, `test_led_effects.cpp`, `test_voltagesense.cpp`, `test_timer_exti_handlers.cpp`, `test_usb_descriptors.cpp`, `test_crc.cpp` | System commands, start-up, HAL / TinyUSB callbacks, LEDs, voltage sensing and brake, timer / EXTI dispatch, descriptors, CRC |
| `SelectableInputs`, `ButtonSource(s)`, `AnalogSource(s)`, `LocalButtons`, `LocalAnalog`, `SPIButtons`, `PCF8574`, `ShifterAnalog`, `ADS111X`, `AnalogAxisProcessing`, `CanButtons`, `CanAnalog` | `test_selectable_inputs.cpp`, `test_analog_source.cpp`, `test_local_buttons.cpp`, `test_local_analog.cpp`, `test_spi_buttons.cpp`, `test_pcf8574.cpp`, `test_shifter_analog.cpp`, `test_ads111x.cpp`, `test_analog_axis_processing.cpp`, `test_can_buttons.cpp`, `test_can_analog.cpp` | Button and analog input sources |
| `EncoderLocal`, `EncoderSSI`, `EncoderBissC`, `MtEncoderSPI` | `test_encoder_local.cpp`, `test_encoder_ssi.cpp`, `test_encoder_bissc.cpp`, `test_mtencoder_spi.cpp` | Encoders on TIM3 / the encoder SPI port |
| `SPIPort`, `SPIDevice`, `SpiHandler`, `I2CPort`, `I2CDevice`, `I2CHandler` | `test_spi_port.cpp`, `test_i2c_port.cpp` | SPI and I2C ports, devices and handler lists |
| `CANPort_2B`, `CanBridge`, `CANInputMain`, `SerialFFB` | `test_canport2b.cpp`, `test_canbridge.cpp`, `test_caninputmain.cpp`, `test_serial_ffb.cpp` | CAN port on the fake HAL CAN, CAN main classes, FFB effects over the `fxm` commands |

`ffb_flash_tests` (`claude/cases_flash/`):

| Suites | Files | Tested firmware |
|---|---|---|
| `FlashSetup` | `test_flash_setup.cpp` | Smoke tests of the fake HAL FLASH and its helpers |
| `EEPROM`, `FlashHelpers`, `EepromAddresses`, `PersistentStorageFlash` | `test_eeprom.cpp`, `test_flash_helpers.cpp`, `test_eeprom_addresses.cpp`, `test_persistent_storage_flash.cpp` | `eeprom.c` (vendor) with power loss, `flash_helpers.cpp`, the address table, save / restore through the real flash layer |

`ffb_rtos_tests` (`claude/cases_rtos/`):

| Suites | Files | Tested firmware |
|---|---|---|
| `RtosSetup` | `test_rtos_setup.cpp` | Smoke tests of the fake FreeRTOS API: one per wrapper family, create failure, kernel faults, self-tests of its queries |
| `Queue`, `Timer`, `Tasklet`, `TickHook`, `WorkQueue`, `EventGroup`, `MemoryPool`, `Mutex`, `Semaphore`, `ReadWriteLock`, `ConditionVariable` | `test_cqueue.cpp`, `test_ctimer.cpp`, `test_ctasklet.cpp`, `test_ctickhook.cpp`, `test_cworkqueue.cpp`, `test_cevent_groups.cpp`, `test_cmem_pool.cpp`, `test_csync_gaps.cpp`, `test_cread_write_lock.cpp`, `test_ccondition_variable.cpp` | The `cpp_freertos` wrappers (vendor, tier A) |

`ffb_driver_tests` (`claude/cases/driver/`):

| Suites | Files | Tested firmware |
|---|---|---|
| `DriverSetup` | `test_driver_setup.cpp` | Skeleton (phase D0): the feature macros of the executable (TMC4671, ODrive, VESC on; MIDI, TMCDEBUG, RMDCAN, PWMDRIVER, SIMPLEMOTION off) |

`ffb_usb_tests` (`claude/cases_usb/`):

| Suites | Files | Tested firmware |
|---|---|---|
| `UsbSetup` | `test_usb_setup.cpp` | Smoke tests: bus reset and enumeration, string descriptor, CDC line coding and commands, HID input / output / feature reports, stall, suspend / resume, the OSAL on the fake FreeRTOS |

Python (`python/`, ctest entry `python_helper_tests`): `test_generate_memory.py` (`Firmware/scripts/generate_memory.py`),
`test_dwc2_info.py` (`dwc2_info.py`).

Test cases marked `KNOWN_ISSUE` and the suspected defects behind them are listed in `Tests/claude/plans/PROGRESS.md` (repository root).

## Mocks and helpers

| Class | Use |
|---|---|
| `MockCommandHandler` | Command handler with one command per access type and flag. Target for all command interface tests. |
| `MockCommandInterface` | Command interface without transport. Injects parsed commands, records results and broadcasts. |
| `MockMotorDriver` | `MotorDriver` recording torque, start/stop, emergency stop. Readiness and slew rate are controlled by the test. |
| `MockEncoder` | `Encoder` with a position set by the test in counts, rotations or degrees. |
| `MockCANPort` | `CANPort` recording sent frames. Received frames are injected in interrupt context (`ffb_unit_tests`; in `ffb_target_tests` only as the view `HostTarget::canPort()` of the real `canport`). |
| `MockButtonSource`, `MockAnalogSource` | `ButtonSource` / `AnalogSource` with test-set buttons / axis values (`ffb_target_tests`), selectable through the real choosers (`registry()`, `support/InputSources.h`). |
| `MockSPIDevice`, `MockI2CDevice` | `SPIDevice` / `I2CDevice` recording the calls of the port (transfer begin/end, completions, errors) for tests of `SPI.cpp` / `I2C.cpp` (`ffb_target_tests`). |
| `PidHost` | Simulates the PID driver of the host: create effect, set effect/condition/periodic..., effect operation, device control. |
| `Axis` (fake) | Replaces the real axis for the effect calculation: metrics are set by the test, the effect torque is recorded. |

Fixtures (in `support/`; test files derive local fixtures from them):

| Fixture | Executable | Use |
|---|---|---|
| `FirmwareFixture` | `ffb_unit_tests`, `ffb_target_tests` | Resets the fake platform (time, RTOS, USB, UART, flash map) and the global firmware state (command handlers, error handler, ...) before and after the test. |
| `CommandSystemFixture` | `ffb_unit_tests` | `FirmwareFixture` + the command thread. `process()` runs all threads until idle. |
| `FfbFixture`, `FfbFixture1Axis` | `ffb_unit_tests` | Effects calculator + `HidFFB` + `PidHost` + two (one) fake axes. |
| `TargetFixture` | `ffb_target_tests` | `FirmwareFixture` + the F407 fake platform (`HostTarget`); fails the test on HAL calls without a fake. |
| `AxisFixture` | `ffb_target_tests` | `TargetFixture` + real `Axis` objects with `MockMotorDriver` / `MockEncoder`, `EffectsCalculator`, `HidFFB`, `PidHost`. |
| `FlashFixture` | `ffb_flash_tests` | Fake flash, time, RTOS and flash-layer state; `init`, `powerCycle`, `reboot`, page image helpers. |
| `RtosFixture` | `ffb_rtos_tests` | Fake FreeRTOS API reset, `advance`, `runReady`, `runInTask`, leak and kernel fault checks. |
| `UsbFixture` | `ffb_usb_tests` | TinyUSB, fake device controller, RTOS and F407 platform reset; `startDevice`, `process`, `enumerate`, a simulated host. |

The fixtures of the other executables are described in their sections below.

`MockMotorDriver` and `MockEncoder` provide `classEntry()` to make them selectable in a `ClassChooser`.
`Axis::driverChooser` and `Axis::encoderChooser` are public, so a test can select the mocks in a real axis
without changing the firmware (`AxisFixture`).

## Writing tests

```cpp
#include "doctest.h"
#include "FfbFixture.h"

TEST_SUITE("EffectsCalculator") {

TEST_CASE_FIXTURE(FfbFixture, "constant force is passed to the axis") {
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);  // create, set effect and start like a game
	host.setConstantForce(cf, 10000);               // PID report
	CHECK(torque(0) == -10000);                     // torque passed to the X axis
}

}
```

Rules:

* Use a fixture. It resets the global state of the firmware so tests do not depend on their order.
* Test through the public interfaces (reports, commands, class interfaces), not private members.
* One behaviour per test case. The name is a statement that is true if the test passes.
* Never sleep. Advance the simulated time.

### Naming conventions

* **`characterization: ...`** documents what the firmware does today where the correct behaviour still has to be
  verified (for example the axis magnitudes against the PID specification). These tests protect against
  unintended changes during refactoring but do not claim the behaviour is correct. Every such open question is a
  **NEEDS REVIEW** item: a numbered row of the "NEEDS REVIEW items" table in `Tests/claude/plans/PROGRESS.md` (repository root),
  collected with the suspected bugs in `Tests/claude/reports/BUG_REPORT.md`. The test names its item in a comment next to the case
  (`// NEEDS REVIEW 29: ...`, above the case or at the start of its body) or in its name; the table row names the test.
  When the item is answered the test is renamed (correct behaviour) or turned into a `KNOWN_ISSUE` test (defect).
* **`KNOWN_ISSUE("...")`** marks a test that describes the intended behaviour but fails because of a suspected
  defect in the firmware. The suite stays green while the defect exists. When the firmware is fixed the test
  is reported as *"Should have failed but didn't"* and the marker must be removed.
  `KNOWN_ISSUE_UB` is the same for undefined behaviour and is skipped in sanitizer builds.
  `KNOWN_ISSUE_UB_HIDDEN` is for undefined behaviour whose intended result happens to occur on the host: the test
  asserts the intended result and passes in normal builds, and is skipped in sanitizer builds. The defect is only
  visible on a sanitizer build run with `--no-skip`. Its description starts with "KNOWN ISSUE", so the inventory below
  lists it.
* **Skipped crash:** a test of a suspected defect that would terminate the whole run (segmentation fault, abort by
  `_GLIBCXX_ASSERTIONS`, floating point exception) is written like a `KNOWN_ISSUE` test but marked
  `doctest::description("KNOWN ISSUE (crash): ...") * doctest::skip()`: it is reported as skipped in every build. Run
  it on purpose with `-tc="<name>" --no-skip`.
* **Traceability:** every `KNOWN_ISSUE`, `KNOWN_ISSUE_UB`, `KNOWN_ISSUE_UB_HIDDEN` and skipped crash case names its
  suspected bug in the marker text (`KNOWN_ISSUE("(bug 16) ...")`, vendor bugs `V<n>`), every `characterization:` case
  its NEEDS REVIEW number (see above). The tables in `Tests/claude/plans/PROGRESS.md` list the test names as evidence. The 318
  baseline cases of `ffb_unit_tests` are left unchanged (NEEDS REVIEW 3 lists their 17 `KNOWN_ISSUE` cases).

  In `ffb_target_tests` the fixture checks for HAL functions without a fake with a `CHECK` in its destructor. In a
  `KNOWN_ISSUE` test that check would also count as the expected failure, so such tests call
  `REQUIRE_ALL_HAL_MODELLED()` right before the assertion that describes the defect whenever they could reach a
  HAL call (see `ffb_target_tests` below).

List all known issues:

```sh
grep -rn "KNOWN_ISSUE\|KNOWN ISSUE" cases/ claude/cases_target/ claude/cases_flash/ claude/cases_rtos/
```

### Adding firmware sources to the tests

1. Add the file to `FIRMWARE_SOURCES` in `CMakeLists.txt`.
2. If it does not compile or link because something of the hardware is missing, extend the fake platform
   (`platform/include/main.h`, `platform/src/host_platform.cpp`). Keep the fakes minimal.
3. Do not add `#ifdef` for tests to firmware sources. If a class can not be used on the host because of its
   dependencies, give it a test double in `fakes/` or a stub in `platform/src/host_stubs.cpp`.

## `ffb_target_tests`: F407 feature set with the real `Axis`

A second executable built by the same `CMakeLists.txt` and run by `ctest`. It compiles the firmware with the
feature set of the F407 target without the motor drivers and MIDI, so `Axis.cpp`, `AxesManager`, the main classes
(`FFBWheel`, `FFBJoystick`, `FFBHIDExt`, ...), the class registries, inputs, the USB device class and (since P4) the
system sources `cppmain.cpp`, `global_callbacks.cpp`, `voltagesense.cpp` and `ledEffects.cpp` are real code.
Later phases add encoders, buses and CAN; since P5 the SPI and I2C HAL are faked (inputs).

* **Test cases:** `claude/cases_target/test_*.cpp` (own glob; `cases/` belongs to `ffb_unit_tests`).
* **Feature header:** `platform/target_f407/target_constants.h` (copy of the F407 header without `TMC4671DRIVER`,
  `PWMDRIVER`, `ODRIVE`, `VESC`, `SIMPLEMOTION`, `RMDCAN`, `TMCDEBUG`, `MIDI`). `platform/target_f407` comes before
  `platform/include`; it also has its own fake `main.h` (HAL types with the fields the F407 sources use, the F407 pin
  names) and `cmsis_os.h`. There is no `fakes/` directory: the real `Axis.h` is used. The platform sources are
  compiled again for this executable because they depend on the feature header.
* **Fake platform of the target:** `support/HostTarget.h` (implemented in `platform/src/host_target.cpp`): timers
  (`timerUpdate`, `timerRunning`), EXTI (`triggerExti`), GPIO levels (`setPin`, `outputPin`), chip ids, recorded
  resets and DFU requests, the USB device task (`queueUsbEvent`). `platform/src/host_target_config.cpp` replaces
  `cpp_target_config.cpp`: the same SPI/I2C/UART/CAN port objects; since P7 the global `canport` is the real
  `CANPort_2B` on `hcan1` (see "CAN part (P7)"). The SPI/I2C handles
  start as after `MX_SPIx_Init` / `MX_I2C1_Init` of `main.c` (master, 2 lines, 8 bit, the CPOL/CPHA/prescaler of each
  port; I2C1 100 kHz, 7 bit).
* **Bus part (P5)** (`platform/src/host_target_bus.cpp`, control in `support/HostTarget.h`), each function citing the
  `stm32f4xx_hal_spi.c` / `stm32f4xx_hal_i2c.c` function it models:
  * **Fake HAL SPI (F4):** `HAL_SPI_Init`, `HAL_SPI_Transmit/_Receive/_TransmitReceive` (blocking, `_IT`, `_DMA`),
    `HAL_SPI_GetState/_GetError`: State machine (READY, BUSY_TX/RX/TX_RX; `HAL_BUSY` while not READY, `__HAL_LOCK`),
    the master 2-line receive as `TransmitReceive(pData, pData)` (the buffer is clocked out as dummy bytes),
    `HAL_ERROR` for missing buffers. Every transfer is recorded (`spiTransfers`: call, mode, size, tx/rx bytes, the
    port configuration, CS pin levels at start and end, result). MISO bytes: queued responses (`spiQueueRx`), a
    responder function (`spiSetResponder`), the idle response (`spiSetIdleRx`), padded with 0x00. IT/DMA transfers
    stay pending until `spiComplete` (rx data, State READY, for DMA the half transfer callback, then
    `HAL_SPI_TxCpltCallback` / `RxCplt` / `TxRxCplt` in interrupt context) or `spiFail` (`HAL_SPI_ErrorCallback`).
    Injection: `spiFailNextBlocking` (`HAL_TIMEOUT` / `HAL_ERROR`), `spiFailNextDmaStart` (State stays busy like
    the HAL). Slave, 1-line, rx-only and 16-bit transfers are not modelled (recorded as unmodelled, `HAL_ERROR`).
  * **Fake HAL I2C (F5):** `HAL_I2C_Init/_DeInit`, `HAL_I2C_Master_Transmit/_Receive` (blocking, `_IT`, `_DMA`),
    `HAL_I2C_Mem_Write/_Read` (blocking, `_IT`), `HAL_I2C_IsDeviceReady`, `HAL_I2C_GetState/_GetMode/_GetError` on a
    bus with scripted devices per 7-bit address (`i2cDevice`): register model with pointer register (width, auto
    increment, `setRegister16` big endian) or plain model (PCF8574: reads return `registers[0]`), scripted reads
    (`readQueue`), NACK of the address (`nackAddress`, `nackAddressCount` for busy devices) or of a written byte
    (`nackWriteByte`): `HAL_ERROR` / `HAL_I2C_ERROR_AF`, and a hook (`onTransfer`) for device behaviour. IT transfers
    stay pending until `i2cComplete` (`HAL_I2C_MasterTx/RxCpltCallback`, `MemTx/RxCpltCallback`, or
    `HAL_I2C_ErrorCallback` with AF on a NACK) or `i2cFail` (BERR, ARLO, ...). `i2cSetBusBusy` (BUSY flag stuck:
    `HAL_BUSY`, TIMEOUT), `i2cFailNextBlocking`. `hi2c1` has no DMA stream (`stm32f4xx_hal_msp.c`): its `_DMA` calls
    return `HAL_ERROR` with `HAL_I2C_ERROR_DMA_PARAM` like the HAL (`i2cSetDmaLinked` for a linked handle).
  * **CAN frames to input sources:** `HostTarget::canReceive(id, data)` delivers a frame from the bus (since P7
    through the filter banks of the fake HAL CAN, see "CAN part (P7)"): the frame reaches every `CanHandler` (e.g.
    `CanButtons`, `CanAnalog`) through `global_callbacks.cpp` like on the target.
  * Timing is not modelled: blocking calls take no simulated time; timeouts are injected.
* **Encoders and buses (P6 batch 0)** (control in `support/HostTarget.h`, `support/HostRtos.h`, `support/AxisFixture.h`):
  * **Quadrature encoder timer:** `encoderTimerCount(&htim3, counts)` moves the counter of `TIM_ENC` like quadrature
    edges in encoder interface mode (RM0090, TIM2 to TIM5 encoder interface mode): only while CEN is set; ARR -> 0
    (overflow) and 0 -> ARR (underflow) are update events (UIF, then `HAL_TIM_IRQHandler`'s update branch if UIE:
    `HAL_TIM_PeriodElapsedCallback` -> `TimerHandler::timerElapsed` in interrupt context, at the wrap, no latency);
    returns the number of wraps. `timerCounter` / `timerSetCounter` read / write CNT. A counter above ARR and a pending
    UIF raised when UIE is enabled later are not modelled. `encoderIndexPulse()`: ENCODER_Z high, low + EXTI interrupt
    (falling edge, MX_GPIO_Init), high again.
  * **Encoder SPI port:** `ENCODER_SPI_PORT` is `ext3_spi` (`hspi3`, CS pins SPI3_SS1..3, base clock 42 MHz). All
    encoders use 8 bit transfers; `spiFrameBytes(frame, bytes)` gives the MISO bytes of a 16..64 bit frame, most
    significant first. MX_GPIO_Init resets all SPI3 CS outputs low and only a device drives its own pin, so
    `selectedCs()` finds no single active pin on `hspi3`: check `csAtStart[i]` of the device's pin instead. `csAtEnd`
    of IT/DMA transfers is recorded before the completion callback (the CS pin is released inside it).
  * **Encoder threads (resumable tasks):** `EncoderBissC` ("BISSENC") and `MtEncoderSPI` ("MTENC") wait twice per loop
    (request semaphore, then the DMA notification). Make their task resumable right after creating the encoder:
    `HostRtos::makeResumable("BISSENC")` (the `cpp_freertos::Thread` base is private: use `findTask` / the returned
    handle with `runUntilBlocked(TaskHandle_t)`). Flow: `getPosAbs()` (gives the request; at time 0 or within 2 ms of
    the last update it does not wait, a `Take(2)` on the test thread returns at once), `runUntilBlocked(task)` (DMA
    started), `spiQueueRx` + `spiComplete(&hspi3)` (completion notifies the task), `runUntilBlocked(task)` (frame
    processed), then read the position. Deleting the encoder unwinds the parked thread.
  * **Encoders in an axis:** `AxisFixture::selectFirmwareEncoder(enctype)` offers `Encoder::all_encoders` (2 local, 4
    MT, 5 BiSS-C, 6 SSI) and selects it with `setEncType`; `selectFirmwareEncoder(0)` deletes the real encoder.
  * **Mock bus devices (F12):** `mocks/MockSPIDevice.h` (`SPIDevice`; records begin/end transfer, tx/rx/txrx
    completion and errors with the port, the CS level and `isTaken()`; `callBase` keeps the semaphore / CS handling of
    `SPIDevice`), `mocks/MockI2CDevice.h` (`I2CDevice`; records start/end transfer, completions and errors with the
    port and the handle state; `callBase` keeps the semaphore handling and the port reset on errors).
  * **Reset:** the static `inUse` flags of `EncoderLocal`, `EncoderSSI`, `EncoderBissC`, `MtEncoderSPI`
    (`resetTargetConfig`); TIM3 and the SPI fake as before. The timer and EXTI handler lists are not reset (no
    static-init handlers; the destructors unregister).
* **CAN part (P7)** (`platform/src/host_target_can.cpp`, control in `support/HostTarget.h`):
  * **`canport` is the real `CANPort_2B`** on `hcan1` with the F407 presets (50k..1000k) and the silent pin `CAN_S`,
    like `cpp_target_config.cpp:51-56` (`HostTarget::canPort2B()`). The object is a `HostCANPort_2B`
    (`host_target_config.cpp`): its `start` / `stop` / `addCanFilter` / `removeCanFilter` call the real functions and
    then record the call in the view `HostTarget::canPort()` (a `MockCANPort` for the P5 tests: `started`,
    `startCalls`, `stopCalls`, `filters` as passed by the firmware, `passesFilter`; its `CANPort` functions forward to
    `canport`). Its `sent`, `speed`, `silent`, `sendResult` are unused. The view records the calls, not the hardware
    state (`started` also after a failed `start()`, `filters` also for an overwritten or rejected bank): tests about
    the hardware use `HostTarget::canActiveFilterBanks()` (bits of `CAN1->FA1R`), `canAccepts` and `hcan1.State`. Like on the target, `canport` is also a command
    handler (`can.0`), a `CanHandler` and a `PersistentStorage` (`ADR_CANCONF1`), and every main class takes it
    (`CAN_CommandInterface` of `FFBoardMain`, `CANCOMMANDS`).
  * **Fake HAL CAN (F8):** `HAL_CAN_Init/_DeInit/_ConfigFilter/_Start/_Stop/_RequestSleep/_WakeUp/_IsSleepActive/
    _AddTxMessage/_AbortTxRequest/_GetTxMailboxesFreeLevel/_IsTxMessagePending/_GetRxMessage/_GetRxFifoFillLevel/
    _ActivateNotification/_DeactivateNotification/_IRQHandler/_GetState/_GetError/_ResetError`, each citing
    `stm32f4xx_hal_can.c`, on the CAN1 registers (`*CAN1`: `MCR`, `MSR`, `TSR`, `RF0R/RF1R`, `IER`, `ESR`, `BTR`, mailboxes,
    `FMR`, `FM1R`, `FS1R`, `FFA1R`, `FA1R`, 28 filter banks). After the reset CAN1 is as after `MX_CAN1_Init` (State
    READY, initialisation mode, BTR 0x001A0005). Mode changes are acknowledged at once (`canFailNextModeChange`
    injects the HAL timeout).
  * **Transmit:** `HAL_CAN_AddTxMessage` uses the lowest empty of 3 mailboxes and records the frame
    (`canTxFrames()`: header as passed, mailbox, id / IDE / RTR / DLC / 8 data bytes decoded from the mailbox registers,
    time, result; `payload()`, `value()`). A frame stays pending (no other node acknowledges) until `canTxComplete(m)` /
    `canTxCompleteAll()` (TXOK, transmit interrupt: `HAL_CAN_TxMailboxXCompleteCallback`) or `canTxError(m)` (failed
    attempt; with automatic retransmission it stays pending and a later abort reports `HAL_CAN_ERROR_TX_TERRx`).
    `canSetTxAutoComplete(true)` completes pending frames at the start of the next HAL CAN call (never inside the call
    that queued them). An abort empties a pending mailbox at once and raises the abort interrupt (host-vs-target: on
    the target a mailbox in the middle of a transmission attempt is aborted only after that attempt).
  * **Receive:** `canReceive(id, data, extId, rtr)`: only in normal mode (started, not sleeping, not bus-off); the
    active filter banks decide (32 / 16 bit, mask / list, RM0090 priority rules, filter match index per FIFO); the frame
    goes into the FIFO of the bank, then the receive interrupt if enabled. `canAccepts` asks the filters only;
    `canFilterConfigs()` records every `HAL_CAN_ConfigFilter` argument. The CAN1/CAN2 ownership of the banks (FMR
    CAN2SB) is written like the HAL but not applied.
  * **Interrupts:** `HAL_CAN_IRQHandler` (modelled step by step) runs in interrupt context while an event is pending
    whose IER bit and NVIC line are enabled. Like `HAL_CAN_MspInit` (`stm32f4xx_hal_msp.c:291-294`) only `CAN1_TX` and
    `CAN1_RX0` are enabled: FIFO 1 frames and the status change events (bus-off, error warning / passive, last error
    code) raise no interrupt of their own on the F407; the handler services them at the next `CAN1_TX` / `CAN1_RX0`
    interrupt (it checks every flag whichever line fired). `canSetIrqLineEnabled(CanIrqLine::Sce, true)` models a board
    that enables them.
    Errors: `canSetErrorState(Warning / Passive / BusOff)` (bus-off stops transmit and receive until the next
    `HAL_CAN_Stop` + `HAL_CAN_Start`), `canSetLastErrorCode(lec)`, FIFO overrun through `canQueueRx` on a full FIFO.
    `canIrq()` raises pending interrupts; `canHalCalls()` lists the HAL calls in order (name, argument, result).
  * Not modelled: bit timing / bus speed, time-triggered mode, TEC/REC counters, automatic bus-off recovery and
    wake-up, loop back / silent mode (recorded as unmodelled at `HAL_CAN_Start`), a frame being sent while it is aborted,
    CAN2; other handles than `hcan1` and filter banks above 27 are recorded as unmodelled.
  * **Reset:** `HostTarget::resetCan()` (fake HAL CAN, records, injections, NVIC lines) and `canport` with its view, in
    `resetTargetConfig`. `canport` gets back the state the firmware constructed: its private members, port users, the
    waiting flag, the availability of both semaphores and the delayed-restore flag are copied once during the static
    initialisation right after `canport` (no literals), so a change of the firmware initialisers is seen by the tests.
  * **CDC and commands:** CAN main classes read the CDC port through `cdcRcv` (`HostUsb::cdcReceive` +
    `main.cdcRcvReady(0)`, or `main.cdcRcv(buf, &len)` directly) and write to `HostUsb::state().cdcTx`.
    `support/CommandHelpers.h`: `CommandHelpers::run` / `get` / `getAt` execute a command of a handler by name;
    `CommandHelpers::CdcConsole(main).send("fxm.0.new=1\n")` sends a command line through the CDC command interface
    and the command thread of a main class and returns the CDC answer.
* **Input source mocks (P5, F12):** `mocks/MockButtonSource.h`, `mocks/MockAnalogSource.h` implement `ButtonSource` /
  `AnalogSource` (test sets the bits / axis values, records the reads); `registry()` is the firmware registry
  (`all_buttonsources` / `all_analogsources`) plus the mock with selection id 15. `support/InputSources.h` replaces the
  choosers of `SelectableInputs` (also the private base of `FFBHIDMain`): `useMockRegistries(InputSources::of(wheel))`,
  then `btntypes` / `aintypes` with bit 15 create the mock through the real `ClassChooser`.
* **System part (P4)** (`platform/src/host_target_system.cpp`, control in `support/HostTarget.h`), each function citing
  the ST HAL / main.c function it models:
  * **Fake HAL ADC (F6):** `HAL_ADC_Start/_Stop/_PollForConversion/_GetValue/_Start_DMA/_Stop_DMA` with the ADC
    registers and states of `stm32f4xx_hal_adc.c`; `hadc1`/`hadc2` as after `MX_ADCx_Init` (TIM4 trigger, DMA,
    8 and 2 conversions). Test inputs per rank (`adcSetInput(s)`), `adcConversionComplete` = circular DMA transfer
    into the buffer armed by `HAL_ADC_Start_DMA` (`ADC1_BUF`/`ADC2_BUF` of `global_callbacks.cpp`) and
    `HAL_ADC_ConvCpltCallback` in interrupt context; `adcSetConversionResult` for polling. The LL ADC macros
    (`__LL_ADC_CALC_DATA_TO_VOLTAGE`, `__LL_ADC_CALC_VREFANALOG_VOLTAGE`, `__LL_ADC_CALC_TEMPERATURE`) are copied from
    `stm32f4xx_ll_adc.h` into the target `main.h`; the calibration values (`VREFINT_CAL`, `TS_CAL1/2`) come from
    variables instead of the system memory addresses (`setAdcCalibration`, defaults 1500/950/1200).
  * **HAL tick:** `htim7` as after `HAL_InitTick` (1 MHz, 1 ms update). `HAL_GetTick()` is the simulated time; the TIM7
    counter follows it (time % 1000), so the real `micros()` of `cppmain.cpp` (= `HAL_GetTick()*1000 + TIM7->CNT`)
    equals the simulated time. `timerUpdate(&htim7)` reaches main.c's `HAL_TIM_PeriodElapsedCallback` (modelled:
    TIM7 -> `HAL_IncTick`, counted; other timers -> `HAL_TIM_PeriodElapsedCallback_CPP`).
  * **IWDG** (`HAL_IWDG_Refresh`, counted), main.c's **`Error_Handler`** (error LED off, counted, returns),
    **CAN receive FIFOs** of `hcan1` (`HAL_CAN_GetRxFifoFillLevel`, `HAL_CAN_GetRxMessage`; `canQueueRx` and the
    level-triggered `canRxInterrupt` reach the CAN callbacks of `global_callbacks.cpp`). Since P7 part of the fake HAL
    CAN in `platform/src/host_target_can.cpp`; `canRxInterrupt` stays a forced interrupt (ignores IER and NVIC).
  * **Start-up:** `runCppmain(loops)` runs the real `cppmain()` with `loops` passes of its main loop (`taskYIELD()` is a
    hook in this executable that ends the loop); the created main class is deleted by `TargetFixture`
    (an FFBHIDMain class needs `DELETES_FFBHIDMAIN`; `BUTTON_A` high selects the failsafe class 0).
  * **Compile adjustments (no firmware change):** `cppmain.cpp` is compiled through `platform/src/host_cppmain.cpp`,
    which renames its `malloc`/`free` definitions (`hosttest_cppmain_malloc`/`_free`; they would replace the C
    library allocator of the executable). `ledEffects.cpp` is compiled with its pulse/set functions renamed; the
    public names in `host_platform.cpp` count the calls (`HostPlatform::leds()`) and forward to the real functions.
    Guard macros `HOSTTEST_REAL_CPPMAIN` / `HOSTTEST_REAL_LED_EFFECTS` leave the corresponding fakes of
    `host_platform.cpp` out of this executable only (`ffb_unit_tests` is unchanged). CMake checks at configure time
    that every `#include` of `cppmain.cpp` is in the wrapper's pre-include list (configuration fails otherwise).
    On the target `mainclass` is in the NOLOAD `.ccmram` section and not zeroed at start; on the host it starts as
    `nullptr`.
  * **OTP** (`HOSTTEST_TARGET_OTP`): `OTP_Write` / `OTP_Read` reproduce `flash_helpers.cpp:298-326` (not compiled
    here) on an in-memory OTP area (16 x 32 bytes + 16 lock bytes, erased 0xFF): 8 byte slots, the bound check as
    coded (bug 41), write only into an all-ones slot, two 32 bit programming steps that only clear bits, locked
    blocks (lock byte 0x00) refuse programming. Control: `HostTarget::otpByte` / `setOtpByte` / `otpSlot` /
    `setOtpSlot`, `state().otpCalls` (recorded calls), `state().otpPrograms`, `state().otpFailProgram` (the n-th
    programming step fails). Used by the `signature` command of `SystemCommands` (`OTPMEMORY` stays off in this
    executable, so there is no `otp` command, unlike the F407 target).
  * **RTOS heap:** `HostRtos::heapStats()` counts `pvPortMalloc` / `vPortFree` calls (all executables; reset by
    `HostRtos::reset()`), e.g. to check that the `malloc`/`free` of `cppmain.cpp` forward to the RTOS heap.
  * **MinGW and macOS (unverified, no MinGW or macOS toolchain here):** `SystemCommands.cpp` uses `struct mallinfo` /
    `mallinfo()` from `<malloc.h>`, which mingw-w64 does not declare and the macOS SDK does not have. Under
    `if(MINGW OR APPLE)` the include path gets `platform/mingw` (`malloc.h` shim: `#include_next <malloc.h>` if the
    system has one, otherwise `<stdlib.h>`) and `platform/mingw/host_mallinfo.cpp` (returns zeros) is compiled. Linux
    builds do not use them.
* **Unmodelled-HAL guard:** every HAL function the compiled sources call has a fake. A call the fake does not model
  (e.g. SPI slave mode, I2C 10-bit addressing, `triggerExti` on a pin without EXTI line) records a description in
  `HostTarget::state().unmodelledHalCalls` and returns `HAL_ERROR`; `TargetFixture` fails the test at the end if the
  list is not empty. In `KNOWN_ISSUE` tests use `REQUIRE_ALL_HAL_MODELLED()` before the defect assertion.
* **Fixtures** (`support/AxisFixture.h`): `TargetFixture` resets everything (`FirmwareFixture` plus `HostTarget`
  including the system part: fake ADC and ADC buffers, TIM7, IWDG, the fake HAL CAN and `canport` (P7), the globals of `cppmain.cpp`,
  `voltagesense.cpp` and `ledEffects.cpp`; the main class created by `runCppmain`, `PersistentStorage` startup flag,
  `SystemCommands::errorPrintingEnabled`, global HID handler, `mainclass`, `usb_device`; since P5 also the fake HAL
  SPI/I2C, the global ports `external_spi` / `motor_spi` / `ext3_spi` (CS pins free, no current device, not exclusive,
  semaphore free) and `i2cport` (no users, no current device, speed preset 0, delayed restore pending; private members
  reached through explicit-instantiation member pointers in `host_target_config.cpp` only), the SPI/I2C handler lists
  (back to the static-init handlers: `~I2CHandler` does not unregister, suspected bug), `MockButtonSource` /
  `MockAnalogSource::creatable`; not reset: the device pointer of `motor_uart`). Delete devices that use a port inside
  the test. After `HostTarget::reset` the three LEDs are on (main.c switches them on before the scheduler starts). `AxisFixture` creates real `Axis` objects with `MockMotorDriver` and
  `MockEncoder` selected through the public `driverChooser` / `encoderChooser`, a real `EffectsCalculator`, `HidFFB`
  and a `PidHost`; `update()` / `run(ms)` run the FFB cycle like `FFBHIDMain::updateControl`.
* **`DELETES_FFBHIDMAIN`:** test cases that delete an `FFBHIDMain` main class carry this marker. Deleting one is
  undefined behaviour (suspected bug 24: `HID_GamepadReport_base` has no virtual destructor), so they run in normal
  builds and are skipped in sanitizer builds.
* **Exclusions:** `FFB_TARGET_TESTS_EXCLUDE` (default empty) and `FFB_TARGET_TESTS_EXCLUDE_SANITIZE` (default empty,
  only in sanitizer builds), printed at configure time like `FFB_TESTS_EXCLUDE`.
* **Host-vs-target differences of the build:**
  * `-funsigned-char`: `char` is unsigned on the ARM target (`ffb_unit_tests` keeps the host's signed `char`).
  * Target include order: `Axis.cpp` is compiled with `-include math.h`. On the shipped targets it gets `<math.h>`
    through `TMC4671.h` -> `SPI.h`, so `abs(float)` uses the float overload; without the drivers the host would call
    `::abs(int)`. This also supplies the math functions `Axis.cpp` uses without including `<cmath>`.
  * Clang only: `-Wno-c++11-narrowing` for `Axis.cpp` (`clip<int8_t>` in `setEqGain`; GCC only warns).

## `ffb_flash_tests`: the real flash layer on a fake HAL FLASH

A third executable built by the same `CMakeLists.txt` and run by `ctest`. It compiles the real `flash_helpers.cpp`,
`eeprom.c` (ST AN3969 EEPROM emulation, vendor code: tested, never fixed), `eeprom_addresses.c` and
`PersistentStorage.cpp` with the F407 feature header (`USE_EEPROM_EMULATION`, `EEPROM_START_ADDRESS` 0x08004000) on a
fake HAL FLASH with NOR semantics. The map-backed `Flash_*` replacement of `host_platform.cpp` is left out
(`HOSTTEST_REAL_FLASH_HELPERS`).

* **Test cases:** `claude/cases_flash/test_*.cpp` (own glob).
* **Fake HAL FLASH:** declarations in `platform/target_f407/host_flash_hal.h` (included by the target `main.h` only when
  `HOSTTEST_FLASH_HAL` is defined, i.e. only in `ffb_flash_tests`; it defines `FLASH_OTP_BASE`, so `OTPMEMORY` is on
  like on the F407), implementation in `platform/src/host_flash.cpp` (each function cites the ST HAL function it
  models). Erase sets a sector to 0xFF, programming can only clear bits (`old & data`; 0 -> 1 requests are recorded),
  programming while locked fails with PGSERR, pending error flags make the next operation fail, OTP bytes are
  one-time programmable and lock bytes block programming. Sectors 0..3 (16 KiB) and the OTP area are modelled;
  anything else is recorded as unexpected (also while the flash is locked) and fails the test.
  **Model assumptions** (not stated explicitly by ST): erasing while locked does nothing and returns `HAL_OK`
  (inferred from RM0090 3.9.7: `FLASH_CR` writes are ignored while LOCK is set); a program address not aligned to
  the program size raises PGPERR (RM0090 3.9.5); `setUnlockFails` makes `HAL_FLASH_Unlock` return `HAL_ERROR` and
  the firmware continue, while on the chip a wrong key sequence is a bus error that locks `FLASH_CR` until reset
  (RM0090 3.5.1, effectively a HardFault), so tests of unlock-failure paths cover defensive code that is probably
  unreachable without a fault.
* **Fixed-address memory:** `eeprom.c` and `flash_helpers.cpp` dereference absolute addresses, so the fake memory is
  mapped at the target addresses when the executable starts: 64 KiB at 0x08000000 (sectors 0..3, EEPROM pages in
  sectors 1 and 2) and 64 KiB at 0x1FFF0000 (OTP 0x1FFF7800..0x1FFF7A0F). Linux/macOS: `mmap` with the address as
  hint (+ `MAP_FIXED_NOREPLACE` where available), never replacing an existing mapping; Windows: `VirtualAlloc` at the
  fixed address. If a range is in use the run aborts with a message (no fallback). Both ranges lie in ASan's
  application memory on x86-64 Linux (`LowMem` 0..0x7fff7fff). On 64-bit macOS the default 4 GiB `__PAGEZERO` covers
  these addresses, so `ffb_flash_tests` is not built on macOS (CMake prints a message; the other executables are).
* **Test control:** `support/HostFlash.h`: raw memory access (`read16`, `write16`, `fill`, `snapshot`, ...), recording
  (operations log, program/erase counts per sector, 0 -> 1 requests, unlock/lock, cache resets, unexpected accesses),
  error injection (`failNext` with an FLASH_SR error flag, `failNextAfter` for a later step, `failNextWithStatus`
  for `HAL_TIMEOUT`/`HAL_BUSY` (`HAL_BUSY` returns before anything else, like a taken `__HAL_LOCK`),
  `setPendingFlags`, `setUnlockFails`) and power-loss injection (`cutPowerAfterPrograms(n, tornMask)`,
  `cutPowerDuringErase(n, offset, length)`: the memory is frozen at the cut, later operations have no effect until
  the reboot).
* **Fixture** (`support/FlashFixture.h`): `FlashFixture` resets the fake flash (erased), simulated time, RTOS and the
  global firmware state (`PersistentStorage` handler list and startup flag, `DataVar` of `eeprom.c`). `init()` runs the
  start-up (`Flash_Init`). `powerCycle()` simulates a power cycle without the start-up (memory kept, hardware reset,
  the RAM state of the flash layer cleared: `DataVar` and the `PersistentStorage` startup flag; the handler list is
  kept because it holds objects the test owns), `reboot()` = `powerCycle()` + `Flash_Init`. Loops that reset the
  flash between iterations use `resetIteration()`, which first checks the iteration for unmodelled accesses (a plain
  reset would discard them). Page image helpers:
  `writePage(page, status, records)`, `fillPage`, `writeRecord`, `pageStatus`, `pageRecords`. The destructor fails the
  test on unmodelled flash accesses; in `KNOWN_ISSUE` tests use `REQUIRE_FLASH_ACCESSES_MODELLED()` before the defect
  assertion.
* **Exclusions:** `FFB_FLASH_TESTS_EXCLUDE` and `FFB_FLASH_TESTS_EXCLUDE_SANITIZE` (both default empty), printed at
  configure time like the others.
* No CMSIS-DSP sources (none of the compiled files use them). `-funsigned-char` like `ffb_target_tests`.

## `ffb_rtos_tests`: the FreeRTOS C++ wrappers on a fake FreeRTOS API

A fourth executable built by the same `CMakeLists.txt` and run by `ctest` (P8). It compiles the `cpp_freertos` wrappers
(vendor code, tier A: tested, never fixed) `cthread.cpp`, `cmutex.cpp`, `csemaphore.cpp`, `cqueue.cpp`, `ctimer.cpp`,
`ctasklet.cpp`, `ctickhook.cpp`, `cevent_groups.cpp`, `cworkqueue.cpp`, `cread_write_lock.cpp`, `ccondition_variable.cpp`
and `cmem_pool.cpp` (plus `UART.cpp` / `UartHandler.cpp`, only to link `host_platform.cpp`) on the fake FreeRTOS API F11.
The FreeRTOS kernel itself is not tested; the fake reproduces the documented behaviour of the target's FreeRTOS V10.3.1
(`Targets/F407VG/Middlewares/Third_Party/FreeRTOS/Source`, each function cites the file:line it models).

* **Test cases:** `claude/cases_rtos/test_*.cpp` (own glob). Feature header: the generic one of `platform/include` (the
  wrappers use no feature macro). No CMSIS-DSP. `-funsigned-char` like the other new executables. Built on Linux,
  macOS and MinGW (no fixed addresses); unverified on macOS and Windows.
* **`HOSTTEST_RTOS_EXT`** (defined only for this executable) switches on the fake API:
  * `platform/src/host_freertos.cpp` includes `platform/src/host_freertos_ext.inc` (the F11 fakes; not a separate
    translation unit, it shares the kernel's internal state) and leaves out the functions the extension replaces
    (heap, `xTaskCreate`, scheduler suspension, notifications, semaphores). The other executables compile this file
    exactly as before (checked: identical preprocessed sources and compile commands).
  * `platform/include/FreeRTOSConfig.h`: the F407 switches the wrappers evaluate (`configSUPPORT_STATIC_ALLOCATION`,
    `configUSE_TRACE_FACILITY`, `INCLUDE_xTimerPendFunctionCall`, `configTIMER_TASK_PRIORITY` 2,
    `configTIMER_QUEUE_LENGTH` 10, `CPP_FREERTOS_CONDITION_VARIABLES`) and `configUSE_TICK_HOOK` 1 (**host-vs-target:**
    the F407 has it 0, so `ctickhook.cpp` is empty on the target). `queue.h`, `timers.h`, `event_groups.h` declare the
    API with the FreeRTOS macros; `task.h` maps the critical section macros to the Cortex-M4F port functions.
  * `platform/src/host_platform.cpp`: advancing the simulated time runs one SysTick per 1 ms boundary
    (`vApplicationTickHook` of `ctickhook.cpp` in interrupt context, BASEPRI raised like `xPortSysTickHandler`). `setTimeUs` jumps without ticks.
  * `cmem_pool.cpp` is compiled through `platform/src/host_cmem_pool.cpp`: its `malloc()` calls go to `pvPortMalloc`
    like on the target (`cppmain.cpp:138`), so heap_4's `pvPortMalloc(0) == NULL` and injected heap failures reach it.
    CMake checks at configure time that the wrapper's pre-include list covers every `#include` of `cmem_pool.cpp`.
* **Fake FreeRTOS API (F11)**, test control in `support/HostRtosExt.h` (model details there):
  * Queues (create, send to back / front, overwrite, receive, peek, the ISR variants, messages waiting, spaces, reset,
    delete), software timers with the timer service task "Tmr Svc" (commands through a queue of 10, processed when the
    service task runs; expired timers before commands; one-shot / auto-reload with catch-up; start / stop / reset /
    change period / delete and the ISR variants; `xTimerPendFunctionCall` / `FromISR`; `pvTimerGetTimerID`,
    `xTimerIsTimerActive`), event groups (create, static create, set / clear (ISR variants through the timer service),
    wait all / any with clear on exit, sync, get, delete), binary / counting semaphores, mutexes with holder
    (giving a mutex held by another task is a configASSERT), recursive mutexes, task notifications with timeouts,
    critical sections / BASEPRI, scheduler suspension, the RTOS heap (live blocks, failures).
  * New tasks are ready (`runUntilIdle` starts every task that never ran). Blocking calls with a timeout use the
    simulated time: a resumable task parks until the condition holds or the timeout has passed; a task with the
    restart model is started again when the condition holds; the test thread fails an infinite wait ("would block
    forever", recorded) and lets a finite timeout pass tick by tick. `pxHigherPriorityTaskWoken` is set from the task
    priorities.
  * **Create-failure injection** for every create function: `HostRtos::failCreate(RtosObject::Queue / Timer /
    EventGroup / StaticEventGroup / BinarySemaphore / CountingSemaphore / Mutex / RecursiveMutex / Task / Heap, after,
    count)`.
  * **Faults:** everything FreeRTOS would stop in `configASSERT` (NULL or deleted handles, non-ISR functions in an
    interrupt, unbalanced critical sections, ...) and every call the fake does not model (tick count overflow,
    blocking timer callbacks, ...) is recorded in `HostRtos::kernelFaults()` and the call returns its failure value.
  * Not modelled: preemption and priority inheritance, the tick count overflow, queue sets, stream / message buffers,
    static creation other than event groups.
  * **Limitation of the test thread:** it is not a task, and no task runs while it waits. A wait of the test thread for
    something only a task or the timer service provides never succeeds ("would block forever" with `portMAX_DELAY`).
    Destructors that wait for another task (`~WorkQueue`, a synchronous `~Timer`) must run in a task:
    `RtosFixture::runInTask(fn, priority)`.
  * Queries for tests: `HostRtos::timerIdsInService()` (IDs of the timers that are active or have a queued start /
    reset / change-period command: the callbacks the service may still call), `HostRtos::objectCounts()` (leak check).
* **Fixture** (`support/RtosFixture.h`): `RtosFixture` resets the simulated time, the kernel and the F11 state before
  and after every test (injections, critical / suspension state, tick counter, the timer service task, its queue and
  timer list, timers deleted by the service). At the end it fails the test on recorded kernel faults or an open
  critical section / scheduler suspension; `KNOWN_ISSUE` tests call `REQUIRE_ALL_RTOS_MODELLED()` before the defect
  assertion, tests expecting a fault remove it with `expectKernelFault("...")`. `advance(ms)` lets time pass tick by
  tick with every ready task (timer service included) run after each tick; `runReady()` runs the ready tasks;
  `runInTask(fn, priority)` runs code that waits for another task inside a resumable task;
  `keepAlive(new MemoryPool(...))` keeps objects that can never be destroyed reachable for the leak sanitizer.
  **Leak check:** at the end of every test the fixture CHECKs that no more tasks, queues, semaphores / mutexes, timers
  and event groups exist than at its construction (the timer service task and timers whose delete command is queued
  do not count). Justified leaks: `allowLeak(Leak::Queues, n)`; `keepAlive` of a `MemoryPool` allows its mutex.
* **Exclusions:** `FFB_RTOS_TESTS_EXCLUDE` and `FFB_RTOS_TESTS_EXCLUDE_SANITIZE` (both default empty), printed at
  configure time like the others. Run directly: `build/ffb_rtos_tests` (`-ts=RtosSetup` for the smoke tests).
* **Host-vs-target:** the host uses the exception variant of the wrappers (the F407 defines `CPP_FREERTOS_NO_EXCEPTIONS`
  and stops in `configASSERT`); `sizeof(void *)` is 8 on the host and 4 on the target (`MemoryPool` raises the
  alignment to the pointer size, so item sizes differ).

## `ffb_usb_tests`: the TinyUSB device stack on a fake USB device controller

A fifth executable built by the same `CMakeLists.txt` and run by `ctest` (P9). It compiles the TinyUSB device stack of
`FFBoard/USB` (vendor tier B: tested, never fixed) `tusb.c`, `common/tusb_fifo.c`, `device/usbd.c`,
`device/usbd_control.c`, `class/cdc/cdc_device.c`, `class/hid/hid_device.c` and `class/midi/midi_device.c` with the
real `tusb_config.h` of the F407 target (`OPT_MCU_STM32F4`, `OPT_OS_FREERTOS`, full speed, EP0 64 bytes;
`CFG_TUD_MIDI` is 1 there, so the MIDI class driver is compiled, while the firmware's `MIDI` feature stays off), on a
fake device controller, together with the firmware's USB glue: `USBdevice.cpp`, `CDCcomm.cpp`, `UsbHidHandler.cpp`, the
descriptors and the TinyUSB callbacks of `global_callbacks.cpp`.

* **Test cases:** `claude/cases_usb/test_*.cpp` (own glob). Run directly: `build/ffb_usb_tests` (`-ts=UsbSetup` for the smoke
  tests). Exclusion lists `FFB_USB_TESTS_EXCLUDE` / `FFB_USB_TESTS_EXCLUDE_SANITIZE` (default empty), printed at
  configure time like the others. Built on Linux, macOS and MinGW (no fixed addresses; unverified on macOS / Windows).
* **Sources:** the firmware and platform sources of `ffb_target_tests` (F407 feature set; `global_callbacks.cpp` needs
  them) plus `ctickhook.cpp` and `ccondition_variable.cpp`, without `platform/src/host_usb.cpp` (the recording TinyUSB
  fake of the other executables); `HOSTTEST_REAL_TINYUSB` leaves the three TinyUSB models of `host_target.cpp` out.
  C sources get `-fexceptions` (a parked resumable task is unwound through the C frames of TinyUSB). CMSIS-DSP as in
  `ffb_target_tests`.
* **OSAL:** TinyUSB uses `osal_freertos.h` like on the target, on the fake FreeRTOS API F11 (`HOSTTEST_RTOS_EXT`, see
  `ffb_rtos_tests`): its event queue is a fake FreeRTOS queue. `platform/usb` is the first include directory: its
  `tusb_config.h` includes the F407 one (path in `FFB_F407_TUSB_CONFIG_H`) and sets `CFG_TUSB_OS_INC_PATH` (TinyUSB's
  hook for the location of the FreeRTOS headers) to `platform/usb/hosttest_osal/`. Those headers include the fake
  FreeRTOS headers and add (`host_osal_freertos.h`, implemented in `platform/src/host_usb_osal.cpp`) the static creation
  functions (`configSUPPORT_STATIC_ALLOCATION` is 1 on the F407; the fake creates the objects dynamically and ignores the
  storage) and an `xQueueReset` that also takes a `SemaphoreHandle_t` (`osal_semaphore_reset`; a semaphore is a queue in
  FreeRTOS but not in the fake). TinyUSB is not changed.
* **Fake device controller (F15)** (`platform/src/host_dcd.cpp`, control in `support/HostDcd.h`, model details there):
  the TinyUSB DCD port API (`device/dcd.h`) modelled on TinyUSB's DWC2 port of the F407 OTG_FS core
  (`portable/synopsys/dwc2/dcd_dwc2.c`, cited per function) and USB 2.0. The test is the cable and the host:
  `attach` / `detach` (unplugged event), `busReset` (address 0, endpoints closed, bus reset event, full speed),
  `suspend`, `resume`, `sof`, and single transactions `setup`, `out`, `in` that return the device's handshake
  (ACK, NAK when no transfer is armed, STALL, no response when the device is not on the bus, not reset yet or at another
  address). Transfers complete after a short packet or the requested length (IN: ceil(n / wMaxPacketSize) packets, a
  0-byte transfer is one ZLP), stall drops the armed transfer, a SETUP clears the EP0 stall, the new address of
  SET_ADDRESS takes effect after its status stage, the SOF interrupt follows `dcd_sof_enable` and remote wakeup, events
  held while `dcd_int_disable`. Events are raised in the USB interrupt (`in_isr`, IPSR of OTG_FS). Every packet and
  event is recorded. Not modelled (recorded in `HostDcd::state().faults`, which fails the test): high speed, isochronous
  endpoints, `dcd_edpt_xfer_fifo`, a transfer on a closed or busy endpoint, packets larger than wMaxPacketSize or than
  the armed transfer, remote wakeup while the bus is not suspended; not modelled at all: FIFO RAM, DMA, bus timing,
  bit errors.
* **Simulated host** (`support/UsbHost.h`): control transfers with data and status stages (NAKs are retried after
  letting the device work, a STALL ends the transfer), `enumerate()` (bus reset, device descriptor, SET_ADDRESS,
  configuration descriptor, string 0, SET_CONFIGURATION; parses the interfaces and endpoints), standard requests, bulk /
  interrupt transfers, CDC ACM requests and data (`cdcSetLineCoding`, `cdcSetControlLineState`, `cdcWrite`, `cdcRead`)
  and HID (`hidSetReport`, `hidGetReport`, `hidWrite`, `hidRead`). Protocol errors of the device seen by the host are
  recorded in `errors()`.
* **Fixture** (`support/UsbFixture.h`): `UsbFixture` resets TinyUSB (`tud_deinit`), the fake controller, the simulated
  time, the fake FreeRTOS, the F407 platform and the firmware globals the USB callbacks use, before and after every
  test. `startDevice(conf, hidReportDescriptor)` does what `FFBWheel::usbInit` and `USBdevice::Run` do (defaults: the
  F407 wheel configuration `usb_cdc_hid_conf_2axis` with `hid_2ffb_desc`): `mainclass` = a `UsbTestMain` (the failsafe
  `FFBoardMain` counting `usbSuspend` / `usbResume` / `cdcRcvReady`; its threads `CDCCMD` and `CMD_MAIN` are resumable),
  `usb_device` = a `USBdevice` with the real descriptors (kept for the whole run: never started, so never destroyed,
  vendor V2), then `tusb_init()`. `createHidFfb()` adds a real `EffectsCalculator` + `HidFFB` as the global HID handler.
  `process()` runs `tud_task_ext(0, false)` and the firmware threads until nothing is left (host-vs-target: on the target
  `tud_task()` blocks until the next event; the processing order is the same). `enumerate()` enumerates and requires the
  configured state. At the end the fixture fails the test on controller faults, host errors, unmodelled HAL calls,
  kernel faults and leaked kernel objects; `KNOWN_ISSUE` tests use `REQUIRE_USB_MODELLED()` before the defect assertion.
* **Host-vs-target:** the USB thread is not started (the test thread runs TinyUSB's task); static FreeRTOS objects are
  created dynamically (create-failure injection would also make them fail); the DWC2 writes the new address at once and
  the core keeps the old one for the status stage, the fake switches after the status stage (same on the bus).

## `ffb_driver_tests`: the TMC4671, ODrive and VESC drivers (driver track)

A sixth executable built by the same `CMakeLists.txt` and run by `ctest` (driver track, `Tests/claude/plans/DRIVER_TEST_PLAN.md`,
phases D0-D3). It tests the real motor drivers against simulated peers, not their hardware (938 cases; final report
`Tests/claude/reports/DRIVER_TRACK_REPORT.md`):

* **Feature header:** `platform/target_f407_drivers/target_constants.h` is the `ffb_target_tests` header plus
  `TMC4671DRIVER`, `ODRIVE` and `VESC` (lines 46, 50, 51 of the F407VG header). `MIDI`, `TMCDEBUG`, `RMDCAN`,
  `PWMDRIVER` and `SIMPLEMOTION` stay off; `HSPIDRV`, `CANBUS` and `TIM_TMC htim6` stay as on the F407VG. The directory
  is first in the include path, so only this header is shadowed (`main.h` etc. come from `platform/target_f407`).
* **Driver sources:** `FFBoard/UserExtensions/Src/TMC4671.cpp`, `TMC4671_configs.cpp`, `ODriveCAN.cpp` and `VescCAN.cpp`
  on top of the firmware and platform sources of `ffb_target_tests` (fake HAL SPI and CAN). `TMCDebugBridge.cpp`,
  `MotorPWM.cpp`, `RmdMotorCAN.cpp`, `MotorSimplemotion.cpp` and the MIDI sources are not compiled.
* **Platform:** `platform/src/host_driver_timers.cpp` adds the fake TIM6 (`htim6`, as left by `MX_TIM6_Init`), which the
  TMC4671 driver uses as its external encoder timer. It is reset by `HostTarget::reset()` through
  `HostTarget::addResetHook`; the other executables do not link it.
* **Test cases:** `claude/cases/driver/test_*.cpp` (own glob). Run directly: `build/ffb_driver_tests`. Exclusion lists
  `FFB_DRIVER_TESTS_EXCLUDE` / `FFB_DRIVER_TESTS_EXCLUDE_SANITIZE` (default empty), printed at configure time like the
  others. Also covered by `tools/run_all_tests.py` / `test_report`.
* **Requirements and specifications:** the requirement charters (requirements, spec facts, lead decisions) are
  `Tests/claude/charters/driver/TMC4671_REQUIREMENTS.md`, `Tests/claude/charters/driver/ODRIVE_REQUIREMENTS.md` and `Tests/claude/charters/driver/VESC_REQUIREMENTS.md`. Vendor
  specifications are kept locally and never committed: `Tests/shared/specs/` (ignored by `Firmware/Tests/.gitignore`)
  or the repository root `Tests/` (ignored by `Tests/.gitignore`): the TMC4671 datasheet
  `Tests/shared/specs/TMC4671-LA_datasheet_Rev2.07.pdf`, the ODrive 0.5.6 CAN page `Tests/shared/specs/CAN Protocol — ODrive Documentation 0.5.6
  documentation.pdf` with `Tests/shared/specs/can-protocol.csv`, the VESC slides `Tests/shared/specs/VESC6_CAN_CommandsTelemetry.pdf` and the VESC
  firmware source `Tests/shared/specs/bldc-master` (7.01). Tests cite them; without them the tests still build and run.
* **Simulated peers and fixtures** (each with a fidelity header: what it models, source, what it does not model,
  confidence): `fakes/FakeTmc4671` (register file behind the fake SPI, fixture `support/TmcFixture.h`),
  `fakes/FakeOdrivePeer` (ODrive CAN Simple node, `support/OdriveFixture.h`), `fakes/FakeVescPeer` (VESC CAN node with the
  buffer protocol and CRC, `support/VescFixture.h`). The peers are strict: a frame the real device would reject is
  recorded as a violation and the fixture fails the test at teardown unless the test declared it. The CAN fixtures
  schedule the driver thread themselves (500 ms loop, clock rewind after each `Delay`, see the fixture headers).
* **Test files:** `test_tmc4671_*.cpp`, `test_odrive_*.cpp`, `test_vesc_*.cpp`; each test name starts with its
  requirement id (`[R12]`, `[O33]`, `[V30]`). `FFB_DRIVER_TEST_FILTER` (cache variable, default `test_*.cpp`) compiles
  only matching files plus the setup and TMC fake self-tests (a developer aid for writing tests in parallel).
* **Markers:** suspected driver bugs are `KNOWN_ISSUE` / `KNOWN_ISSUE_UB` / `KNOWN_ISSUE_UB_HIDDEN` tests or skipped
  cases ("KNOWN ISSUE (crash)", "KNOWN ISSUE (memory corruption)", run them with `--no-skip`); firmware-only behaviour
  is `characterization:`. Bugs 139-193 in `Tests/claude/reports/BUG_REPORT.md`.
* **Reading a result:** `tools/summarise_results.py` condenses a log into counts, the failing cases with file:line and
  the first failed assertion, skipped and excluded cases and build errors:

```sh
ctest --test-dir build -V > ctest.log; python3 -B tools/summarise_results.py ctest.log
build/ffb_driver_tests -r=xml | python3 -B tools/summarise_results.py       # XML shows skipped cases by name
cmake --build build 2>&1 | python3 -B tools/summarise_results.py --format build
```

* **Coverage:** `tools/coverage_report.py` lists the four driver files in the section "Driver track". They are not part
  of the coverage ladder (its total is unchanged).

## `ffb_dsp_tests`: the CMSIS-DSP modules (phase P11)

A seventh executable built by the same `CMakeLists.txt` and run by `ctest` (`Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md`, requirements
C1-C6). It tests the CMSIS-DSP library (vendor code, never edited) of the modules the firmware uses (FastMath,
Controller, Interpolation, Statistics, Filtering, CommonTables and the four BasicMath helpers `arm_abs_q15`,
`arm_abs_q31`, `arm_add_f32`, `arm_add_f16`), for all data types (f64, f32, f16, q31, q15, q7). No firmware or platform
source is compiled into it.

* **Libraries:** the CMSIS-DSP sources are built as two static libraries with the same flags and definitions as the DSP
  sources of the other executables (`USE_DSP_FUNCTIONS ARM_MATH_LOOPUNROLL __GNUC_PYTHON__`: the portable C code path,
  not the Cortex-M intrinsic path): `ffb_cmsis_dsp` (every `arm_*.c` of the modules, without `*_f16.c`) and
  `ffb_cmsis_dsp_f16`. The other executables are not changed (identical compile commands).
* **When it is built:** only if the DSP build is on (`FFB_TESTS_USE_DSP`) and the submodule contains
  `Libraries/CMSIS-DSP/Testing/Patterns` (otherwise the configure output says so and nothing changes).
* **Patterns:** ARM's reference patterns are read at run time from the submodule (`FFB_CMSIS_PATTERN_DIR`, defined by
  CMake, absolute with forward slashes); nothing is copied under `Firmware/Tests`. `support/DspPatterns.h` holds the loader
  (type letter `W`/`H`/`B`/`D`, count, hex lines; `//` comment lines are never parsed; CRLF tolerant; a missing file
  fails the test) with typed accessors, and the comparison helpers (SNR, abs+rel, LSB bound, exact, guard words).
  Its self-tests are `claude/cases_dsp/test_dsp_patterns.cpp`.
* **f16:** CMSIS-DSP only compiles the f16 code if the compiler announces `__ARM_FP16_FORMAT_IEEE`, and defines
  `float16_t` as `__fp16`, which x86 compilers do not have. CMake checks that the compiler supports `_Float16` and then
  compiles the f16 sources (`ffb_cmsis_dsp_f16`) and the test files named `claude/cases_dsp/test_dsp_*_f16.cpp` with
  `-D__ARM_FP16_FORMAT_IEEE=1 -D__fp16=_Float16` (no source edited). Without `_Float16` those are not built and the
  configure output says so; the tests see `FFB_DSP_HAS_F16` (0 or 1). Put every test that uses `float16_t` or
  `arm_*_f16` into a file with the `_f16.cpp` suffix; other files stay free of the definitions.
* **Test cases:** `claude/cases_dsp/test_dsp_*.cpp` (own glob). `FFB_DSP_TEST_FILTER` (CMake cache variable, default
  `test_dsp_*.cpp`) limits the compiled files, e.g. `cmake -DFFB_DSP_TEST_FILTER="test_dsp_fastmath*.cpp" .`; the
  loader self-tests are always compiled. Exclusion lists `FFB_DSP_TESTS_EXCLUDE` / `FFB_DSP_TESTS_EXCLUDE_SANITIZE`
  (default empty). Test names carry the requirement id and the unit, e.g. `[C1 arm_sin_f32] ...`.
* **Run:** `build/ffb_dsp_tests` (from any directory; the loader tests write short-lived files into the current
  directory), or `ctest -R ffb_dsp_tests`. Also covered by `tools/run_all_tests.py` / `test_report`; the coverage report
  has its own "CMSIS-DSP in scope" section (outside tier C and the ladder, so their numbers stay comparable).
* **Host vs target:** the host runs the portable C path; results of the Cortex-M `ARM_MATH_DSP` path, FMA
  contraction and out-of-range float to integer conversions can differ on the target (each affected test says so).
  `arm_copy_q15` / `arm_fill_q15` of SupportFunctions are linked for the `arm_conv_opt_*` / `arm_correlate_opt_*`
  variants (charter L6) and have no tests of their own.
* **Test files so far (G1, FastMath and helpers):** `test_dsp_fastmath_{f32,f64,f16,q31,q15}.cpp` and
  `test_dsp_basicmath_helpers.cpp` (`arm_abs_q15/q31`). Suspected vendor bugs found there (Tests/claude/plans/PROGRESS.md V37-V41):
  `KNOWN_ISSUE_UB_HIDDEN` cases are skipped in sanitizer builds (UBSan stops at the first finding), so a sanitizer run
  of `ffb_dsp_tests` reports skipped cases on purpose. Files for Statistics, Interpolation, Controller and Filtering follow
  in G2-G5.

## Python helper tests

`claude/python/test_*.py`: stdlib `unittest` smoke tests of the Python helper scripts (P9, tier A: tested, never fixed):
`Firmware/scripts/generate_memory.py` (memory map CSV -> `eeprom_addresses.c` / `.h`) and
`FFBoard/USB/portable/synopsys/dwc2/dwc2_info.py` (TinyUSB's DWC2 register decoder). `claude/python/helper_paths.py` holds the
paths and a loader that imports a script without writing `__pycache__`. `claude/python/test_run_all_tests.py` tests the
report tool `tools/run_all_tests.py` with synthetic doctest XML and tables (no test executable is run).

* `ctest` runs them as `python_helper_tests` when Python 3 is found (like the baseline check):
  `python3 -B -m unittest discover -s python -t python` with `PYTHONDONTWRITEBYTECODE=1`, in the build directory.
  Directly: `python3 -B -m unittest discover -s Firmware/Tests/claude/python -t Firmware/Tests/claude/python -v`.
* The scripts never write into the repository: their outputs go to temporary directories created by the tests (the
  repository files are only read). No network, no other packages: `dwc2_info.py` imports `click` and `pandas`, which
  the tests replace with minimal stand-ins while it is loaded (nothing is installed).
* Generated files are compared with LF line ends (the scripts write text mode files, CRLF on Windows).

## Limitations

* The host is a 64 bit little endian machine with a different compiler backend. Stack usage, timing, alignment
  faults and float to integer conversions of out of range values are not covered.
* No preemption. Race conditions between threads and interrupts can not be found.
* The real motor drivers are not compiled (out of scope). `Axis`, the main classes and `SystemCommands` are only
  compiled in `ffb_target_tests`.
* Outside `ffb_usb_tests` the USB descriptors are not parsed; the report layouts are compared with constants in the
  tests. `ffb_usb_tests` parses the configuration descriptor during the enumeration (interfaces and endpoints only).
* macOS and Windows (MinGW) builds are unverified: no such toolchain was available. `ffb_flash_tests` is not built on
  macOS.

## Next steps

* **P9:** TinyUSB (tier B), the CMSIS-DSP functions the firmware calls (tier C) and the Python helpers. It was on hold
  for the wrap-up review and has since been started: batch 0 (`ffb_usb_tests`, the Python helper tests) is in place,
  the module tests follow (plan in `Tests/claude/plans/TEST_PLAN.md`, status in `Tests/claude/plans/PROGRESS.md`).
* Open questions (USB HID PID 1.01 specification for the `HidFFB` characterization tests, the CI job, Windows / macOS
  verification) are listed under "Open questions for me" in `Tests/claude/plans/PROGRESS.md`; suspected bugs and NEEDS REVIEW items
  in `Tests/claude/reports/BUG_REPORT.md`.
