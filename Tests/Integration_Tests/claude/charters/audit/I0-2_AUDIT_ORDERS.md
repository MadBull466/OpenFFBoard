# WO I0-2: fake audit orders (as sent, 2026-11-18)

Four `fake-auditor` instances ran these orders in parallel. Each order is the common part plus one group section; the auditor saw nothing else. Results: `audit/<group>.md` (written by the lead from the answers) and the "Fake audit register" in `../../plans/PROGRESS.md`.

## Common part (sent to every auditor)

You audit test fakes for the OpenFFBoard host **integration** test track. Repository root: `/home/ben/Documents/OpenFFBoard/testing/OpenFFBoard` (branch `add-test-suite`). Read `CLAUDE.md` (rules), `Firmware/Tests/README.md` (fake platform, simulated time, fixtures) and section 4 of `Tests/Integration_Tests/claude/plans/INTEGRATION_TEST_PLAN.md` first.

Context: the integration tests will run the real firmware chain USB HID PID -> `HidFFB` -> `EffectsCalculator` -> `Axis` -> `TMC4671` driver against these fakes, in one executable with real TinyUSB and the extended RTOS fake (`HOSTTEST_RTOS_EXT`). A wrong fake makes an integration test pass or fail for the wrong reason, so every fake is treated as possibly wrong. If you find a defect, the lead will propose a fix to the owner; you fix nothing.

Method, for every behaviour listed under "The integration relies on":
1. Read the fake's fidelity header and code, and the cited source (datasheet / RM0090 / ST HAL / TinyUSB / FreeRTOS). Specs are in `Tests/Code_Coverage/shared/specs/` (PDFs; use `pdftotext` if present, else read with the Read tool by page range). ST HAL reference copy: `Tests/Code_Coverage/shared/specs/stm32f4xx-hal-driver-1.8.3/`. Firmware sources: `Firmware/FFBoard/` (read-only).
2. Compare. Quote both sides briefly (fake file:line, spec section / page or source file:line). Check also how the firmware uses the fake's behaviour (caller file:line), because a deviation that no firmware path can reach is minor.
3. Classify each finding: **defect** (the fake does something the hardware / library would not do, or accepts what it would reject, in a way the integration relies on), **gap** (not modelled, needed by the integration; say what is needed), **doc** (header claims differ from the code, behaviour fine), or **ok**.
4. For each defect: find the coverage test cases that depend on it. Copy the tree to `../ffb-test-build/audit/<group>/` (for example `mkdir -p ../ffb-test-build/audit/<group> && cp -a Firmware ../ffb-test-build/audit/<group>/`), apply the minimal correction there only, configure and build there (`cmake -S ../ffb-test-build/audit/<group>/Firmware/Tests -B ../ffb-test-build/audit/<group>/build`, then `cmake --build <that build dir> -j"$(nproc)" --target <the executables you need>`), run the affected executables and compare with an unmodified build of the same copy. Summarise logs with `python3 -B Firmware/Tests/Code_Coverage/tools/summarise_results.py <doctest XML or log>` (run doctest executables with `-r=xml`). The `effectsForces` exclusion of `CLAUDE.md` applies to `ffb_unit_tests`. Never write inside the repository checkout; never commit; no installs or downloads (hooks enforce this). If a correction would take long to build, say which executables would need re-running instead of guessing results.

Rules: quote only results you ran; mark anything unverified as "not verified"; never invent register values, timings or protocol bytes; if a spec you need is not in `shared/specs/`, say "spec not supplied" and what it would decide. Stop at about 1500 words.

Answer format:
1. Table: behaviour | verdict (ok / defect / gap / doc) | fake file:line | source citation | firmware caller file:line.
2. Each defect / gap in detail: what is wrong, why it matters for the integration, the proposed minimal correction, the coverage cases whose results change (with the evidence you ran), confidence (high / medium / low).
3. Confidence for what F1 needs from this fake overall, and anything you did not get to.

## Group A: `FakeTmc4671`

Files: `Firmware/Tests/Code_Coverage/fakes/FakeTmc4671.{h,cpp}`, its use in `support/TmcFixture.h` and `support/HostTarget.h`; also look at `fakes/FakeTmcPhase.{h,cpp}` only as far as it changes register behaviour. Firmware caller: `Firmware/FFBoard/UserExtensions/Src/TMC4671.cpp` (+ `Inc/TMC4671.h`). Spec: `TMC4671-LA_datasheet_Rev2.07.pdf` (SPI datagram section, register map, reset defaults).

The integration relies on:
- the 40-bit SPI datagram (write bit, address, 32-bit data, MSB first) and the reply to a read (data of the address of *that* datagram, as the datasheet describes the read cycle);
- decoding of `PID_TORQUE_FLUX_TARGET` 0x64 (torque in [31:16], flux in [15:0], signed) as the driver writes it;
- `PID_POSITION_ACTUAL` 0x6B as a signed 32-bit value the test can set (65536 counts per mechanical revolution per the datasheet's PhiM scale) and what the chip does on a *write* to 0x6B; any mirroring to other registers (e.g. 0x68 / 0x6A / 0x6C) the fake implements or omits;
- `PID_TORQUE_FLUX_LIMITS` 0x5E, `CHIPINFO` reads (0x00 / 0x01 select-then-read), status flags (0x7C / 0x7D), `INPUTS_RAW` 0x76 bit 15 (ENABLE_IN), ADC raw registers used by the driver's calibration, `PHI_E_EXT` 0x1C;
- two chips on separate chip selects; the write log order;
- reads of a register right after a write (the value returned), reads of undocumented / reserved addresses, and writes to read-only registers (what the real chip does vs. the fake).

## Group B: fake HAL SPI and GPIO / EXTI

Files: `Firmware/Tests/Code_Coverage/platform/src/host_target_bus.cpp` (SPI part), `platform/src/host_target.cpp` (GPIO / EXTI), `support/HostTarget.h`, the fake HAL headers under `fakes/stm32_hal/`. Firmware callers: `Firmware/FFBoard/Src/SPI.cpp` (or wherever `SPIPort` / `SPIDevice` live; find them), `Firmware/FFBoard/UserExtensions/Src/TMC4671.cpp` (blocking `HAL_SPI_TransmitReceive` of 5-byte frames on `hspi1`, the interrupt path `writeRegAsync` used by the TMC encoder thread), the E-stop / enable / flag pin users. Sources: ST HAL `stm32f4xx_hal_spi.c` and `stm32f4xx_hal_gpio.c` (reference copy above), UM1725 (HAL API), RM0090 (SPI and GPIO chapters).

The integration relies on:
- blocking `HAL_SPI_TransmitReceive`: return codes, `State` / `ErrorCode` transitions, behaviour when busy, size 0, null buffers; error injection (`spiFailNextBlocking` or similar) matching a real HAL error path;
- the IT path (`HAL_SPI_TransmitReceive_IT` and the completion callback): when the callback fires relative to the caller, re-entry, what happens to a second request while one is in flight;
- chip-select framing: CS low before the first byte and high after the last, per device, as the firmware's `SPIPort` drives it; which device the fake routes a frame to;
- GPIO: `HAL_GPIO_WritePin` / `ReadPin` / `TogglePin`, output pins read back (the coverage track's limit L27: IDR does not mirror ODR for outputs; check what the fake does and what RM0090 says), EXTI callbacks for the E-stop input and how a test triggers them (edge vs. level).

## Group C: fake ADC, timers, `micros()` and the flash map

Files: `Firmware/Tests/Code_Coverage/platform/src/host_target_system.cpp` (ADC, timers), `platform/src/host_driver_timers.cpp`, `platform/src/host_platform.cpp` and `support/HostPlatform.h` (simulated time), `platform/src/host_flash.cpp` and `support/HostFlash.h` (flash map for settings), `support/HostTarget.h`. Firmware callers: `Firmware/FFBoard/Src/voltagesense.cpp` (`hasPower()`, `vSenseMult`), the FFB tick timer (TIM13) user in `FFBWheel` / `FFBHIDMain` (find it), `micros()` users in `EffectsCalculator.cpp` / `HidFFB.cpp` / `cppmain.cpp`, the TMC external-encoder timer (TIM6) in `TMC4671.cpp`, the flash / EEPROM emulation calls used by `Flash_Write` / `Flash_Read` for settings. Sources: ST HAL `stm32f4xx_hal_adc.c`, `stm32f4xx_hal_tim.c`, RM0090 (ADC, basic / general-purpose timer chapters: update event, ARR preload `ARPE`, counter wrap).

The integration relies on:
- ADC values that `voltagesense` turns into volts (the `hasPower()` thresholds, undervoltage), including the reset of `vSenseMult` between tests;
- TIM13 FFB tick: the update-interrupt rate from PSC / ARR, what happens when the firmware changes ARR at run time (immediately or at the next update, depending on `ARPE`), callback ordering with the simulated millisecond;
- TIM7 / `micros()`: the formula (ms tick x 1000 + counter), monotonicity across the counter wrap and the millisecond boundary, behaviour beyond 2^32 microseconds (about 71.6 minutes), and the cost of advancing simulated time that far (is a fast-forward possible without running every millisecond?);
- TIM6 (TMC external encoder) and TIM3 (quadrature, if used on F1) start / stop / callback semantics;
- the flash map: save and restore of all settings across a simulated reboot (what survives, what the fake resets), and whether it differs from the target in a way that matters (word size, erase value).

## Group D: `HostDcd`, `UsbHost` and `HostRtos` extended

Files: `Firmware/Tests/Code_Coverage/platform/src/host_dcd.cpp`, `support/HostDcd.h`, `support/UsbHost.h`, `platform/src/host_freertos.cpp`, `platform/src/host_freertos_ext.inc`, `support/HostRtos.h`, `support/HostRtosExt.h`, `support/UsbFixture.h` (how they are combined today), and the CMake block that builds `ffb_usb_tests` in `Firmware/Tests/CMakeLists.txt` (defines `HOSTTEST_RTOS_EXT`, `HOSTTEST_REAL_TINYUSB`). Real code under the fake: TinyUSB `Firmware/FFBoard/USB/` (`device/usbd.c`, `class/hid/hid_device.c`, the DCD contract in `device/dcd.h`; the real controller driver `portable/synopsys/dwc2/dcd_dwc2.c` is what `host_dcd.cpp` replaces). FreeRTOS reference: `Firmware/Targets/F407VG/Middlewares/Third_Party/FreeRTOS/Source/` (tasks.c, queue.c, timers.c; the fake states which version it follows). USB 2.0 and HID 1.11 are not in `shared/specs/` (only the PID 1.01 / 1.02 documents): for USB behaviour use TinyUSB's DCD contract and RM0090's OTG_FS chapter, and say "spec not supplied" where only USB 2.0 would decide.

The integration relies on:
- interrupt OUT transfers to the HID endpoint: one report of up to 64 bytes in one packet, longer data split into max-packet-size packets with a short / zero-length packet terminating the transfer as TinyUSB expects; NAK until the class re-arms the endpoint; `dcd_edpt_xfer` / `dcd_event_xfer_complete` sequencing;
- control transfers: SET_REPORT and GET_REPORT for feature reports (PID block load, pool), SETUP / DATA / STATUS stages, stall handling;
- bus reset, enumeration, suspend / resume;
- RTOS: task notifications from ISR context (`vTaskNotifyGiveFromISR`, `xTaskNotifyFromISR`) used by the FFB tick, `vTaskDelay` / `osDelay` and the delay budget (`setDelayBudgetMs` or equivalent), priorities and the order in which ready tasks run in `runUntilBlocked` / `runUntilIdle`, mutexes / semaphores / queues the USB and TMC threads use; whether anything in the extended RTOS has never been exercised together with a driver thread like the TMC4671 thread (`TMC4671::Run`, priority 25) and the TMC encoder thread (priority 33), and what breaks if it is.
