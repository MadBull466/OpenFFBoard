# Integration track: bug report

*State of 2026-11-18, commit `9fccde8f`.* All faults found by the integration track (`Tests/Integration_Tests/`): suspected firmware bugs (IB n), items that need the owner's review (INR n), defects found in the test fakes and in the test tools. The questions to the owner are in [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md).

**Maintained by hand until WO I0-8**, which adds `Firmware/Tests/Integration_Tests/tools/make_integration_bug_report.py`; from then on sections 1 and 2 are generated from the registers in [../plans/PROGRESS.md](../plans/PROGRESS.md) and must not be edited by hand. Where this file and `PROGRESS.md` disagree, `PROGRESS.md` wins.

The coverage suite has its own report ([../../../Code_Coverage/claude/reports/BUG_REPORT.md](../../../Code_Coverage/claude/reports/BUG_REPORT.md), bugs 1-226, V1-V129). An integration bug that is the same defect as a coverage bug says so in "Same as"; the coverage numbers are not reused.

Firmware and vendor code are never fixed by this track: each bug has a test that asserts the intended behaviour and is marked `KNOWN_ISSUE` (fails as expected, the suite stays green).

## Summary

| Kind | Count | Open |
|---|---|---|
| Suspected firmware bugs (IB) | 3 | 3 |
| NEEDS REVIEW items (INR) | 1 | 1 |
| Defects in shared test fakes | 3 | 0 (fixed in WO I0-3, owner-approved) |
| Defects in test tools | 1 | 1 (owner's decision) |

| # | Module | Short description | Severity (lead's estimate) | Test |
|---|---|---|---|---|
| IB 1 | FFBHIDMain | FFB timer period 1001 us (999 Hz) instead of 1000 us | low: 0.1 % rate error, the effects code is told 1000 Hz | `KNOWN_ISSUE` |
| IB 2 | TMC4671 | External-encoder timer 257 us (3891 Hz) instead of 250 us (4 kHz) | low: 2.8 % rate error of the encoder update | `KNOWN_ISSUE` |
| IB 3 | HID command interface | Use after free: replies queued while the HID IN endpoint is busy are read from freed memory and get lost | high: undefined behaviour on the target, lost command replies | `KNOWN_ISSUE_UB` |
| INR 1 | Axis / TMC4671 | Motor not started after a runtime driver selection; Running reachable with motor type NONE after a USB re-mount | open question | `characterization:` |

## 1. Suspected firmware bugs

### IB 1: FFB timer period is ARR+1 = 1001 us at the default rate

- **Where:** `Firmware/FFBoard/UserExtensions/Src/FFBHIDMain.cpp:256`.
- **What:** the FFB timer TIM13 counts at 1 MHz (PSC 83 at the 84 MHz APB1 timer clock, `Firmware/Targets/F407VG/Core/Src/main.c:1203`). The firmware sets `ARR = 1000000*basediv/basefreq` with the comment "Assumes 1µs timer steps", which gives ARR = 1000 at the default rate (`basefreq` 1000, `FFBHIDMain.h:124-125`).
- **Expected:** an up-counter counts from 0 to ARR, so its period is ARR+1 counts (RM0090 Rev 22, general-purpose timer up-counting mode). One cycle per 1000 us needs ARR = 999.
- **Effect:** the FFB loop runs every 1001 us (999 Hz) while `updateSamplerate` is told 1000 Hz. The integration test counts 1000 FFB cycles in 1001 simulated ms instead of 1001.
- **Same as:** none. Same pattern as coverage bug 201 (MotorPWM, RC frame period), at a different place.
- **Test:** `test_f1_setup.cpp`, the 1001 ms F1-A4 case (`KNOWN_ISSUE("(IB 1) ...")`), WO I0-5.

### IB 2: TMC external-encoder timer runs at 3891 Hz instead of 4 kHz

- **Where:** `Firmware/FFBoard/UserExtensions/Src/TMC4671.cpp:3156-3158`; `TIM_TMC_BCLK` in `Firmware/Targets/F407VG/Core/Inc/target_constants.h:72`; the "250 = 4khz" comment at `target_constants.h:96`.
- **What:** the firmware sets PSC = `((SystemCoreClock / 2)/1000000) + 1` = 85 with the comment "1µs ticks", and ARR = 250 ("250 = 4khz").
- **Expected:** 1 us ticks need PSC = 84 MHz / 1 MHz - 1 = 83; a 250 us period then needs ARR = 249. The timer period is (PSC+1)(ARR+1) clocks.
- **Effect:** (85+1)(250+1) = 21586 clocks = 257 us, so the external encoder is updated at 3891 Hz instead of 4 kHz.
- **Same as:** none (same pattern as IB 1 and coverage bug 201).
- **Test:** `test_f1_setup.cpp`, the TIM6 case (`KNOWN_ISSUE("(IB 2) ...")`), reached through the public commands `tmc.0.combineEncoder=1`, `axis.0.enctype=2`, `tmc.0.encsrc=5`; WO I0-5.

### IB 3: HID command interface reads freed memory and loses replies

- **Where:** `Firmware/FFBoard/Src/HidCommandInterface.cpp:39-46` (`HID_CommandInterface::Run`) and `:159` (`queueReplyValues`).
- **What:** `Run()` walks `outBuffer` with a range-for loop and, inside the loop, waits in `Delay(1)` until the HID IN endpoint is ready. While it waits, the command thread (priority 32, higher than the HIDCMD thread's 18 on the target) appends replies with `outBuffer.push_back`, which can reallocate the vector. The loop then reads through a dangling reference.
- **Expected:** every reply queued for the HID command interface is sent intact and in order.
- **Effect:** undefined behaviour. In the sanitizer build ASan reports a heap-use-after-free in `tud_hid_n_report` (`hid_device.c:123`) called from `sendHidCmd` (`HidCommandInterface.cpp:228`). In the default build, of six queued replies only one arrives (with a wrong value); no crash.
- **Not host-only:** the command thread preempts the HIDCMD thread on the target in the same way.
- **Same as:** none. The coverage report lists other HID command interface items (NR 7, NR 8), not this one.
- **Test:** `test_f1_setup.cpp`, "F1-A1: replies queued for the HID command interface while its IN endpoint is not read all arrive intact and in order" (`KNOWN_ISSUE_UB("(IB 3) ...")`, skipped in sanitizer builds by the marker); found in WO I0-5.

## 2. NEEDS REVIEW items

### INR 1: motor start after a runtime driver selection

- **Where:** `Firmware/FFBoard/Src/Axis.cpp:534-543` (`Axis::setDrvType`), `Firmware/FFBoard/UserExtensions/Src/TMC4671.cpp:1682-1687` (`TMC4671::startMotor`), `Firmware/FFBoard/Src/global_callbacks.cpp:375` (`tud_mount_cb`), `Axis.cpp:639` (`Axis::usbResume`).
- **What the firmware does today:** with USB connected and a fresh flash, `axis.0.drvtype=1` (TMC_1) sets up the driver and calls `startMotor()` at once, before the driver is initialised. `startMotor()` only sets `motorEnabledRequested`; when the state machine then reaches Shutdown (motor type and encoder type are NONE), the request is not acted on and the driver stays in Shutdown (checked for 10 s of simulated time). A USB re-mount (`tud_mount_cb` -> `Axis::usbResume` -> `startMotor`) then switches it to Running, although motor type and encoder type are still NONE.
- **Question:** see [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md) IQ6.
- **Test:** `test_f1_setup.cpp`, `characterization: [F1-A1] with USB connected the TMC stays in Shutdown after axis.0.drvtype=1 on a fresh flash and runs after a USB re-mount with motor and encoder type NONE`; WO I0-5. The integration fixture brings the firmware up through the re-mount ("board configured, PC connects afterwards").

## 3. Defects found in the shared test fakes (fixed)

Found by the fake audits of WO I0-2 (reports in [../charters/audit/](../charters/audit/)), fixed in place in WO I0-3 under the owner's override for this track, approved by the owner on 2026-11-18, committed in `096f43d3`. No existing coverage result changed; 4 coverage cases were edited and 9 added.

| # | Fake | Defect | Fix |
|---|---|---|---|
| A-1 | Fake HAL SPI (`host_target_bus.cpp`, `spiFail`) | Delivered an overrun error during a transmit-only transfer to `HAL_SPI_ErrorCallback`; the ST HAL clears OVR in `BUSY_TX` and returns without a callback (`stm32f4xx_hal_spi.c:2405-2422`). Also accepted MODF / FRE, which cannot occur with the firmware's SPI settings | Such errors are recorded as unmodelled and the transfer stays pending |
| A-2 | Map flash (`host_platform.cpp`, `Flash_Dump`) | Returned every stored value; the firmware returns only the addresses of its export list (`flash_helpers.cpp:268-287`), so calibration values were dumped | The fake walks the firmware's list; `eeprom_addresses.c` linked into the map executables |
| A-3 | Extended RTOS fake (`host_freertos.cpp`, `host_freertos_ext.inc`) | `vTaskDelay` in a task moved the shared simulated clock by the whole delay at once; ready tasks ran in creation order instead of by priority | Under `HOSTTEST_RTOS_EXT` the task waits for simulated time, a delay in a restart-model task is a kernel fault, the highest-priority ready task runs first |

Gaps of the fakes that are not defects (timers and the ADC not driven by simulated time, interrupt SPI transfers completed only on request, no plant behind `FakeTmc4671`, ENABLE_IN high by assumption) are handled by the integration fixture and scheduler; see the audit reports.

## 4. Defects found in the test tools

| # | Tool | Defect | Status |
|---|---|---|---|
| T-1 | `Firmware/Tests/Code_Coverage/tools/run_all_tests.py` (coverage suite) | Reports 3 skip-marked VESC cases of `ffb_driver_tests` as failed ("known bug no longer fails - fixed? remove the marker") and attributes them to unrelated vendor bugs: `[V63] getEncoderType with useencoder off and no external encoder does not crash` -> V11, and two `[V24] ...` FILL_RX_BUFFER cases -> V5. Same before and after WO I0-3 | open: [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md) IQ7 |

The integration track's own hooks had three false blocks during I0 (a `2>&1` redirect read as a push argument, `chmod +x` read as a path, shell variables in build paths); the first two are fixed with tests, the third is a documented limit (`../plans/AGENT_PLAN.md` section 2.4).
