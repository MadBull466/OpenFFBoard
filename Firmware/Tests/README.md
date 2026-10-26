# OpenFFBoard firmware unit tests

Functional unit tests for the hardware independent parts of the firmware.
The tests run on the development PC and in GitHub Actions. No board, debugger or ARM toolchain is required.

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
  (`HostRtos::runUntilBlocked`, `HostRtos::runUntilIdle` or `process()` of the fixtures).
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

The test executable can be started directly for more control:

```sh
build/ffb_unit_tests                        # all tests, prints details of known issues
build/ffb_unit_tests -ts=CmdParser          # one test suite
build/ffb_unit_tests -tc="*sine*"           # test cases by name
build/ffb_unit_tests --list-test-cases
build/ffb_unit_tests -s                     # also print successful assertions
```

### Build options

| Option | Default | Description |
|---|---|---|
| `FFB_TESTS_USE_DSP` | `AUTO` | Build with `USE_DSP_FUNCTIONS` and CMSIS-DSP like the hardware targets. `AUTO` enables it if the `Firmware/Libraries/CMSIS-DSP` submodule is checked out. With `OFF` the fallback math of the firmware is tested. Both variants must pass. |
| `FFB_TESTS_SANITIZE` | `OFF` | Address and undefined behaviour sanitizers (Linux/macOS). |
| `FFB_TESTS_EXCLUDE` | the `effectsForces` case | Test cases `ctest` excludes (`--test-case-exclude`). Only cases that abort the whole run because of a suspected defect belong here. |
| `FFB_TESTS_EXCLUDE_SANITIZE` | the 2 spline cases | Additionally excluded in sanitizer builds with CMSIS-DSP (the address sanitizer reports an over-read inside `arm_spline_f32`). |

The active exclusions are printed when CMake configures (`OpenFFBoard tests: excluded from ctest: ...`).
When running the executable directly, pass the same list, e.g.
`build/ffb_unit_tests --test-case-exclude="requesting the forces of a not existing axis is rejected"`.

### Baseline check and coverage

`ctest` also runs `tools/check_baseline.py` (if Python 3 is found). It checks that every test case listed in
`baseline/test_cases.txt` still exists with its expected result (passes, fails as a known issue, or is skipped).
New test cases are allowed. Run it directly with `python3 tools/check_baseline.py --build-dir build`.

Line coverage per file, module and vendor code, with plain `gcov` (no other tools needed):

```sh
cmake -S . -B build-cov -DCMAKE_C_FLAGS=--coverage -DCMAKE_CXX_FLAGS=--coverage -DCMAKE_EXE_LINKER_FLAGS=--coverage
cmake --build build-cov -j && ctest --test-dir build-cov
python3 tools/coverage_report.py --build-dir build-cov
```

Code that can only be checked on the board (register setup, busy waits on peripheral flags, interrupt timing) is
not tested. It is listed with a reason in `tools/coverage_exclusions.txt` and left out of the coverage numbers
(`--no-exclusions` shows the raw numbers).

## Directory layout

| Path | Content |
|---|---|
| `cases/test_*.cpp` | Test cases. Every file is picked up automatically. |
| `cases_target/test_*.cpp` | Test cases of `ffb_target_tests` (F407 feature set, real `Axis`). |
| `cases_flash/test_*.cpp` | Test cases of `ffb_flash_tests` (real `flash_helpers.cpp` and `eeprom.c` on the fake HAL FLASH). |
| `mocks/` | Implementations of firmware class interfaces for tests. |
| `fakes/` | Headers replacing firmware headers (currently only `Axis.h`). |
| `support/` | Fixtures and helpers: `FirmwareFixture.h`, `FfbFixture.h`, `PidHost.h`, `TestHelpers.h`, `Host*.h`. |
| `platform/include` | Fake `main.h` (HAL), FreeRTOS, CMSIS headers and the `target_constants.h` of the test "target". |
| `platform/src` | Implementation of the fake platform. |
| `CMakeLists.txt` | Build. Contains the list of firmware sources under test. |
| `baseline/test_cases.txt` | The test cases that existed before the suite was extended, with their expected result. Do not edit. |
| `tools/` | `check_baseline.py` (baseline check), `coverage_report.py` (gcov summary). Python standard library only. |

## Test suites

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

Test cases marked `KNOWN_ISSUE` and the suspected defects behind them are listed in `Tests/PROGRESS.md` (repository root).

## Mocks and helpers

| Class | Use |
|---|---|
| `MockCommandHandler` | Command handler with one command per access type and flag. Target for all command interface tests. |
| `MockCommandInterface` | Command interface without transport. Injects parsed commands, records results and broadcasts. |
| `MockMotorDriver` | `MotorDriver` recording torque, start/stop, emergency stop. Readiness and slew rate are controlled by the test. |
| `MockEncoder` | `Encoder` with a position set by the test in counts, rotations or degrees. |
| `MockCANPort` | `CANPort` recording sent frames. Received frames are injected in interrupt context. |
| `MockButtonSource`, `MockAnalogSource` | `ButtonSource` / `AnalogSource` with test-set buttons / axis values (`ffb_target_tests`), selectable through the real choosers (`registry()`, `support/InputSources.h`). |
| `PidHost` | Simulates the PID driver of the host: create effect, set effect/condition/periodic..., effect operation, device control. |
| `Axis` (fake) | Replaces the real axis for the effect calculation: metrics are set by the test, the effect torque is recorded. |
| `CommandSystemFixture` | Resets everything, owns the command thread. `process()` runs all threads until idle. |
| `FfbFixture` | Effects calculator + `HidFFB` + `PidHost` + two fake axes. |

`MockMotorDriver` and `MockEncoder` provide `classEntry()` to make them selectable in a `ClassChooser`.
`Axis::driverChooser` and `Axis::encoderChooser` are public, so a test can select the mocks in a real axis
without changing the firmware (see next steps).

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
  unintended changes during refactoring but do not claim the behaviour is correct.
* **`KNOWN_ISSUE("...")`** marks a test that describes the intended behaviour but fails because of a suspected
  defect in the firmware. The suite stays green while the defect exists. When the firmware is fixed the test
  is reported as *"Should have failed but didn't"* and the marker must be removed.
  `KNOWN_ISSUE_UB` is the same for undefined behaviour and is skipped in sanitizer builds.

  In `ffb_target_tests` the fixture checks for HAL functions without a fake with a `CHECK` in its destructor. In a
  `KNOWN_ISSUE` test that check would also count as the expected failure, so such tests call
  `REQUIRE_ALL_HAL_MODELLED()` right before the assertion that describes the defect whenever they could reach a
  HAL call (see `ffb_target_tests` below).

List all known issues:

```sh
grep -rn "KNOWN_ISSUE\|KNOWN ISSUE" cases/ cases_target/
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

* **Test cases:** `cases_target/test_*.cpp` (own glob; `cases/` belongs to `ffb_unit_tests`).
* **Feature header:** `platform/target_f407/target_constants.h` (copy of the F407 header without `TMC4671DRIVER`,
  `PWMDRIVER`, `ODRIVE`, `VESC`, `SIMPLEMOTION`, `RMDCAN`, `TMCDEBUG`, `MIDI`). `platform/target_f407` comes before
  `platform/include`; it also has its own fake `main.h` (HAL types with the fields the F407 sources use, the F407 pin
  names) and `cmsis_os.h`. There is no `fakes/` directory: the real `Axis.h` is used. The platform sources are
  compiled again for this executable because they depend on the feature header.
* **Fake platform of the target:** `support/HostTarget.h` (implemented in `platform/src/host_target.cpp`): timers
  (`timerUpdate`, `timerRunning`), EXTI (`triggerExti`), GPIO levels (`setPin`, `outputPin`), chip ids, recorded
  resets and DFU requests, the USB device task (`queueUsbEvent`). `platform/src/host_target_config.cpp` replaces
  `cpp_target_config.cpp`: the same SPI/I2C/UART port objects, but the global `canport` is a `MockCANPort`
  (`HostTarget::canPort()`, handle `&hcan1` like `CANPort_2B`) until the fake HAL CAN exists. The SPI/I2C handles
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
  * **CAN frames to input sources:** `HostTarget::canReceive(id, data)` applies the filters of the `canport` mock
    (started, active filter matches, FIFO of the filter), then `canQueueRx` + `canRxInterrupt`: the frame reaches every
    `CanHandler` (e.g. `CanButtons`, `CanAnalog`) through `global_callbacks.cpp` like on the target.
  * Timing is not modelled: blocking calls take no simulated time; timeouts are injected.
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
    level-triggered `canRxInterrupt` reach the CAN callbacks of `global_callbacks.cpp`).
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
  * **MinGW (unverified, no MinGW toolchain here):** `SystemCommands.cpp` uses `struct mallinfo` / `mallinfo()` from
    `<malloc.h>`, which mingw-w64 does not declare. Under `if(MINGW)` the include path gets `platform/mingw`
    (`malloc.h` shim with `#include_next`) and `platform/mingw/host_mallinfo.cpp` (returns zeros) is compiled.
* **Unmodelled-HAL guard:** every HAL function the compiled sources call has a fake. A call the fake does not model
  (e.g. SPI slave mode, I2C 10-bit addressing, `triggerExti` on a pin without EXTI line) records a description in
  `HostTarget::state().unmodelledHalCalls` and returns `HAL_ERROR`; `TargetFixture` fails the test at the end if the
  list is not empty. In `KNOWN_ISSUE` tests use `REQUIRE_ALL_HAL_MODELLED()` before the defect assertion.
* **Fixtures** (`support/AxisFixture.h`): `TargetFixture` resets everything (`FirmwareFixture` plus `HostTarget`
  including the system part: fake ADC and ADC buffers, TIM7, IWDG, CAN FIFOs, the globals of `cppmain.cpp`,
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

* **Test cases:** `cases_flash/test_*.cpp` (own glob).
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

## Limitations

* The host is a 64 bit little endian machine with a different compiler backend. Stack usage, timing, alignment
  faults and float to integer conversions of out of range values are not covered.
* No preemption. Race conditions between threads and interrupts can not be found.
* The real motor drivers are not compiled (out of scope). `Axis`, the main classes and `SystemCommands` are only
  compiled in `ffb_target_tests`.
* The USB descriptors are not parsed. The report layouts are compared with constants in the tests.

## Next steps

* Verification of the `characterization` tests of `HidFFB` against the USB HID PID 1.01 specification (direction handling,
  condition blocks, device control, block load status).
* Integration test with the real `Axis.cpp`: PID report -> effect -> axis -> `MockMotorDriver` torque with the
  position from `MockEncoder`. Requires a few more HAL declarations for the headers included by `Axis.h`
  and `#include <cmath>` in `Axis.cpp`.
* Tests for `Axis` (scaling, endstop, speed and slew rate limiter), `Filters` and `ErrorHandler`.
