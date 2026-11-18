# Integration test plan: OpenFFBoard firmware modules working together (host side)

Status: **draft, written 2026-11-18 by the lead (Opus 5.5), waiting for the owner's approval.** Nothing in `Firmware/` has been created or changed for this track yet. State and numbers: `PROGRESS.md` "Current state" (the only place for numbers). Agents, hooks and gates: `AGENT_PLAN.md`.

## 1. Goal and what "integration" means here

The coverage suite (P0-P18, `Tests/Code_Coverage/`) tests each module with its neighbours mocked. This track wires the **real** modules of a flow together and fakes only the hardware boundary (SPI, USB device controller, timers, ADC, GPIO, flash), to find defects at the seams: units and scaling, sign and direction, ordering, timing and update rate, shared state, overflow and precision loss across boundaries, and closed-loop instability.

Priority 1 is flow **F1**: USB HID PID output reports -> TinyUSB / `HidFFB` -> `EffectsCalculator` (every CMSIS-DSP filter, interpolation and spline step) -> `Axis` (gain, limits, damper / inertia / friction, endstops, safety) -> TMC4671 driver -> torque target at the fake TMC4671, including everything that feeds back or changes it (position feedback, ADC / supply, init and calibration, faults and E-stop, commands, save / restore). F1 must be complete (section 8, phase I5 exit) before priority 2 starts: the same chain with ODrive, VESC, PWM, SimpleMotion and MyActuator (flows F2-F6).

Owner decisions that shape the plan (`reports/OPEN_QUESTIONS.md`, IQ1-IQ4, 2026-11-18): own bug register and report with cross-references to the coverage register; oracle for the HID entry = PID 1.01 (`USB_PID1_01.pdf`, now supplied); nominal plant model plus setting sweeps for stability; existing functional fakes (not the real ST HAL).

## 2. The real call chain of F1 (exploration summary, 2026-11-18)

Line numbers are those of the exploration; writers re-check them in the work order.

| # | Step | Code | Data at the boundary | Thread / context |
|---|---|---|---|---|
| 1 | Host sends an output report on the HID interrupt OUT endpoint, or a feature SET_REPORT on EP0 | TinyUSB `hid_device.c:396` (OUT), `:313-329` (feature, strips the id byte only if it equals wValue) | bytes, report id in byte 0 for OUT | USB interrupt -> `USB` thread (prio 40, `tud_task`) |
| 2 | `tud_hid_set_report_cb` -> `UsbHidHandler::globalHidHandler->hidOut` | `global_callbacks.cpp:312-320` | id taken from byte 0 when TinyUSB passes id 0 | `USB` thread |
| 3 | `HidFFB::hidOut` dispatches by id (0x01 effect, 0x02 envelope, 0x03 condition, 0x04 periodic, 0x05 constant, 0x06 ramp, 0x0A operation, 0x0B free, 0x0C control, 0x0D gain, 0x11 create); 0x12 / 0x13 served by `hidGet` | `HidFFB.cpp:92-470` | `memcpy` into packed report structs; direction 0.01 deg -> `axisMagnitudes` via `sin` / `-cos` (`:333`); magnitudes pushed to the reconstruction filter with `micros()` timestamps | `USB` thread |
| 4 | Effects stored in `std::array<FFB_Effect, 40>` | `EffectsCalculator.h`, `ffb_defs.h:21` | int16 magnitudes, uint8 gains, float axis factors, unique_ptr biquads per axis | shared, no lock with step 6 (S11) |
| 5 | Timer tick: TIM13 (`TIM_FFB`) update -> `FFBHIDMain::timerElapsed` -> notify `FFBMAIN` | `FFBHIDMain.cpp:379`, rate dividers `FFBHIDMain.h:109-129` (1 kHz default at FS) | none | ISR -> `FFBMAIN` thread (prio 30) |
| 6 | `updateControl`: `AxesManager::update` = `Axis::prepareForUpdate` (reads the encoder = TMC 0x6B over SPI) for all axes, then `EffectsCalculator::calculateEffects` | `FFBHIDMain.cpp:129-157`, `AxesManager.cpp:42-48` | position degrees (float), speed deg/s and accel deg/s^2 from backward differences x `filter_f` through biquads (profile 1: 70 Hz Q 0.55, 55 Hz Q 0.30) | `FFBMAIN` |
| 7 | Per effect: envelope / waveform / reconstruction (none, linear, Hermite, `arm_spline_f32`), condition forces with gain / scaler / per-effect biquad (damper 30 Hz Q .40, inertia 15 Hz Q .20, friction 50 Hz Q .20), x `angle_ratio`, x global gain / 255; summed in int32 without clip | `EffectsCalculator.cpp:115-483, 573-745` | int32 per axis | `FFBMAIN` |
| 8 | `Axis::setFfbEffectTorque` -> `updateDriveTorque` -> `updateTorque`: x `effectRatioScaler`, equaliser / expo, + idle spring / axis damper / inertia / friction (biquads 60 / 20 / 50 Hz), + endstop, x `torqueScaler` (= power / 0x7fff), out-of-bounds, fade, speed limiter (`arm_pid_f32`), inversion, slew limiter, clip to +-power | `Axis.cpp:786-899` | float <-> int32 several times; clip to +-`power` (uint16) | `FFBMAIN` |
| 9 | `drv->turn(int16)` only if the torque changed and `motorReady()` | `Axis.cpp:488-495`, `MotorDriver.cpp:96` | implicit int32 -> int16 narrowing | `FFBMAIN` |
| 10 | `TMC4671::turn`: ready / enabled check, flux = idle flux - min(abs(power), fluxoffset), optional inversion, optional flux dissipation, `setFluxTorque` -> write 0x64 = flux [15:0] | torque [31:16] (forces motion mode torque first) | `TMC4671.cpp:1745-1774, 2080-2091`, frame `:2529-2553` | 5-byte SPI frame, 0x80 \| addr, big endian, mode 3 | `FFBMAIN` (blocking SPI on `motor_spi` = hspi1, CS PA4) |
| 11 | Chip limits torque by 0x5E (= `power` via `setPowerLimit`) | `TMC4671.cpp:2143-2152` | | |
| 12 | Feedback: next cycle reads 0x6B (s32), `getPos_f = getPos / getCpr`, cpr returned as 0xffff for the integrated encoder (S1) | `TMC4671.cpp:1910-1941`, `Encoder.cpp:67-72` | | `FFBMAIN` |
| 13 | TMC thread (`"TMC"`, prio 25, base `Delay(10)`): state machine uninitialized -> waitPower -> init with power (ADC calibration, enable pin PE7, PWM FOC) -> encoder init -> Running; every 2 s status: ENABLE_IN (0x76 bit 15), temperature; undervoltage guard from MCU ADC2 supply (`voltagesense.cpp`) | `TMC4671.cpp:443-660` | | `TMC` thread; FLAG pin EXTI PE8 |
| 14 | Optional external encoder: TIM6 at 4 kHz -> `TMCENC` thread writes 0x1C (PHI_E_EXT) asynchronously | `TMC4671.cpp:3149-3181` | | ISR -> `TMCENC` (prio 33) |
| 15 | Commands (CDC / HID vendor / CAN) through the command thread `CMD_MAIN` (prio 32); `PersistentStorage` save / restore (TMC 0x320-0x32C, axis 0x301-0x311) | `TMC4671.cpp:2769-2817`, `Axis.cpp` | | `CMD_MAIN` |

Already true today: `ffb_driver_tests` compiles the real `Axis`, `EffectsCalculator`, `HidFFB`, `FFBWheel` and `TMC4671` together, but no test drives them as a chain; `ffb_target_tests` drives the chain with `MockMotorDriver` / `MockEncoder` and struct injection (`test_ffb_integration.cpp`); `ffb_usb_tests` runs the real TinyUSB stack with `HidFFB` but without `Axis` or a driver.

## 3. Architecture

### 3.1 Real versus faked (F1)

| Layer | Real | Fake (boundary) |
|---|---|---|
| USB | TinyUSB device stack (`usbd.c`, `hid_device.c`, `cdc_device.c`, `midi_device.c` for the driver table), `USBdevice`, descriptors, `CDCcomm`, `UsbHidHandler`, `global_callbacks.cpp` | `HostDcd` (fake dwc2 device controller), `UsbHost` (simulated host) |
| FFB | `HidFFB`, `EffectsCalculator`, `Filters`, CMSIS-DSP (portable C path, as the coverage suite) | none |
| Main / axis | `FFBWheel` (`FFBHIDMain`), `AxesManager`, `Axis`, `ErrorHandler`, `CommandHandler`s, `PersistentStorage`, `SystemCommands`, `voltagesense`, `cppmain` start-up | none |
| Driver | `TMC4671.cpp`, `TMC4671_configs.cpp` (and in F2-F6 the other drivers) | `FakeTmc4671` on the fake HAL SPI (`hspi1`) |
| HAL | none | `HostTarget` (SPI, ADC incl. supply voltages, TIM including TIM13 / TIM6 / TIM7, GPIO, EXTI, CAN), `HostPlatform` (time, flash map), `host_driver_timers.cpp` |
| RTOS | the `cpp_freertos` wrappers | `HostRtos` with the extension `HOSTTEST_RTOS_EXT` (needed by the TinyUSB OSAL) |
| Physics (closed loop only) | none | `WheelPlant` (new, section 6), attached to `FakeTmc4671` through its `onWrite(0x64)` / register hooks |

No `MockMotorDriver` or `MockEncoder` on the F1 chain; `MockEncoder` only where a test needs an encoder the board cannot provide (none planned for F1; F1-T9 uses the real `EncoderLocal` on the fake TIM3).

### 3.2 Executable and CMake layout

* New executable **`ffb_integration_tests`**, defined in `Firmware/Tests/Integration_Tests/integration.cmake`, included by one line `include(Integration_Tests/integration.cmake)` after the driver block of `Firmware/Tests/CMakeLists.txt`. It reuses the existing source-list variables (`TARGET_FIRMWARE_SOURCES`, `USB_TINYUSB_SOURCES`, the driver sources, `TARGET_PLATFORM_SOURCES`) with `list(REMOVE_DUPLICATES)` so the lists cannot drift; the other executables keep byte-identical compile commands (checked by comparing `compile_commands.json` entries before / after, as in P11).
* Feature header: `Code_Coverage/platform/target_f407_drivers/target_constants.h` (TMC4671, ODrive, VESC, TMCDEBUG, PWM, SimpleMotion, RMDCAN on; MIDI off, so no TMC double is force-included). Include order: `Code_Coverage/platform/usb`, `.../target_f407_drivers`, `.../target_f407`, `.../include`, `Integration_Tests/support`, `Code_Coverage/support`, `Code_Coverage/mocks`, `Code_Coverage/third_party`, firmware directories. `Code_Coverage/fakes` is never an include directory (its `Axis.h` would shadow the real one; fakes are included by relative path as `TmcFixture.h` does).
* Definitions: the target set (`HOSTTEST HOSTTEST_REAL_CPPMAIN HOSTTEST_REAL_LED_EFFECTS HOSTTEST_TARGET_OTP`) plus `HOSTTEST_RTOS_EXT HOSTTEST_REAL_TINYUSB` and `FFB_F407_TUSB_CONFIG_H`; `HOSTTEST_NO_SIMPLEMOTION` with clang; DSP definitions as the other executables. Per-file flags carried over (`Axis.cpp -include math.h`, clang narrowing flags); `-fexceptions` for C sources (resumable tasks unwound through TinyUSB); mallinfo shim under `MINGW OR APPLE`. Platform sources: the target set minus `host_usb.cpp`, plus `host_dcd.cpp`, `host_usb_osal.cpp`, `host_driver_timers.cpp` and the peer fakes.
* Option `FFB_TESTS_INTEGRATION` (default ON: portable, so CI and every variant run it). ctest entry `ffb_integration_tests` with `FFB_INTEGRATION_TESTS_EXCLUDE` / `_EXCLUDE_SANITIZE` / `_EXCLUDE_32BIT`, printed at configure time like the others; a ctest entry `integration_oracle_tests` (Python stdlib, only when Python 3 is found) runs the oracle scripts' self-tests and checks that generated oracle tables are current.
* Folder `Firmware/Tests/Integration_Tests/` mirrors `Code_Coverage/`:

| Path | Content |
|---|---|
| `integration.cmake` | the executable (above) |
| `claude/cases/test_f1_<group>.cpp` | F1 cases, one file per charter group (`test_f1_hid.cpp`, `_effects`, `_recon`, `_axis`, `_tmc`, `_persist`, `_timing`, `_stability`, `_setup`); F2-F6 later as `test_f2_*.cpp` ... |
| `claude/oracles/` | stability analyst's Python stdlib scripts, their `unittest` self-tests and generated tables (`*.csv`, never hand-edited) |
| `support/` | integration-only fixtures and helpers: `IntegrationFixture.h`, `ChainScheduler.h`, `HidReportDescriptor.h` (parser), `PidReportBuilder.h`, `OscillationDetector.h`, `FrequencyResponse.h` |
| `fakes/` | integration-only fakes: `WheelPlant.{h,cpp}` (with the fidelity header) |
| `tools/` | `make_integration_bug_report.py` (generates `INTEGRATION_BUG_REPORT.md`), `it_gate.py` (the gate, `AGENT_PLAN.md` section 2.3) |
| `codex/` | reserved for Codex (not created by Claude) |

* Shared code stays in `Code_Coverage/` and is reused from there (`support/`, `fakes/`, `mocks/`, `platform/`, `third_party/doctest.h`, `support/test_main.cpp`).
* Tools: `run_all_tests.py` gets `ffb_integration_tests` in `EXECUTABLES` / `EXCLUDE_VARS` and learns the integration numbers (`IB n`, `INR n`) from a second table source (`--integration-progress`, default `Tests/Integration_Tests/claude/plans/PROGRESS.md`); `test_report` depends on the new executable; `coverage_report.py` gets an "Integration (F1 chain)" section computed from the integration executable's `.gcda` files only. Each tool change comes with tests in `Code_Coverage/claude/python/` (existing convention). `check_baseline.py` and `summarise_results.py` need no change.

### 3.3 New fixture and scheduler

`IntegrationFixture` (one struct, no multiple inheritance of the existing fixtures, because `UsbFixture`, `TargetFixture` and `TmcFixture` each reset the same globals and own `mainclass` differently):

1. Reset once, in this order: `tud_deinit`, `HostDcd::reset`, `HostPlatform::reset`, `HostRtos::reset`, `HostTarget::reset` (with the TMC reset hooks), `ErrorHandler::clearAll`, the globals `TargetFixture::resetTarget` resets (`mainclass`, `usb_device`, `globalHidHandler`, persistent-storage start-up flag), flash map cleared. Never `HostUsb::reset` (the recording TinyUSB fake is not linked).
2. Board set-up like `TmcFixture`: SPI1 CS lines idle high (L14 of the TMC charter), supply through the fake ADC2 (`powerOn()` 24 V default), `FakeTmc4671 chip1` (CS 0) and `chip2` (CS 1) on `hspi1`, ADC raw words at mid scale (calibration succeeds), 10 us per SPI frame option.
3. Start the firmware the way the board does: `HostTarget::runCppmain` selecting `FFBWheel`, the real `USBdevice` thread made resumable, the simulated host enumerates (`UsbHost::enumerate`); axis 0 driver selected by the `axis.0.drvtype` command over CDC (the user's path), so `setDrvType` -> `setupDriver` runs as on the board.
4. `ChainScheduler::advanceMs(n)`: per simulated ms, in a documented fixed order: (a) USB events and the `USB` thread until blocked, (b) the 1 ms tick: `HostPlatform::advanceMs(1)`, TIM7, and every timer whose period elapsed (TIM13 -> `FFBMAIN` cycle, TIM6 -> `TMCENC`), (c) `FFBMAIN` until blocked, (d) the `TMC` thread when its `Delay` expired, (e) `CMD_MAIN` / `CDCCMD`, (f) `WheelPlant::step(1 ms)` when a plant is attached. Host-vs-target: on the target these preempt by priority (USB 40 > TMCENC 33 > CMD 32 > FFBMAIN 30 > TMC 25); the fixed order is the order of a quiet target where each thread finishes within its tick. A few F1-H6 / F1-T6 cases permute the order to show the result does not depend on it.
5. Teardown checks (fail the case): unmodelled HAL calls, `HostDcd` faults, host errors, kernel faults, leaked kernel objects (except the documented long-lived objects of the main class), `FakeTmc4671` violations not declared, plant energy sanity. `KNOWN_ISSUE` cases call `REQUIRE_CHAIN_MODELLED()` right before the defect assertion (pattern of `REQUIRE_ALL_HAL_MODELLED`).
6. Observation helpers: torque / flux at the chip per cycle (`chip1.writesTo(0x64)` decoded as s16 halves), SPI frames per cycle, axis metrics through public commands (`axis.0.curpos` etc.), effect state through the PID State report.

`PidReportBuilder` builds every report from the **parsed** HID report descriptor (usage page 0x0F usages of PID §5 / the usage table), not from the firmware's C structs (the coverage suite's `PidHost` uses the firmware structs, so it cannot catch a struct / descriptor mismatch). `PidHost` stays in use by the coverage suite only.

## 4. Fake audit list (before an integration test relies on a fake)

Every fake is treated as possibly wrong. The `fake-auditor` (`AGENT_PLAN.md`) checks each against its fidelity header and source; a defect is fixed in place under the owner's override (section 9), never worked around in an integration test.

| Fake | F1 relies on it for | Audit focus (source) | Header confidence today | Planned change |
|---|---|---|---|---|
| `FakeTmc4671` (`Code_Coverage/fakes/`) | 0x64 decode, 0x6B position, 0x5E limit, CHIPINFO ping, ADC raw for calibration, 0x76 ENABLE_IN, status flags, two chips by CS | DS §4.2.1 datagram, §7.1 R/W map, §7.3 defaults; write ordering in the log; 0x6B write -> 0x68 mirror; behaviour of reads of a register just written; undocumented addresses | frame / registers High, flags Medium, plant hooks Low | none expected; add a typed plant hook API only if the audit shows `onWrite` / `onRead` cannot express it cleanly |
| Fake HAL SPI (`host_target_bus.cpp`) | blocking 5-byte frames on `hspi1`, CS framing, IT path of `writeRegAsync` (TMCENC), error injection | `stm32f4xx_hal_spi.c` state machine, CS levels at start / end, `spiFailNextBlocking` | Medium-High | none expected |
| Fake ADC2 / `voltagesense` inputs (`host_target_system.cpp`) | `hasPower()` thresholds (10 V / 78 V), undervoltage tests | `stm32f4xx_hal_adc.c`, `vSenseMult` reset (L26) | Medium | none expected |
| Fake timers (TIM13 FFB tick, TIM6 TMC ext encoder, TIM7 tick and `micros()`, TIM3 quadrature) | FFB rate, rate changes (ARR), `micros()` for reconstruction timestamps, external encoder | RM0090 timer update semantics; `micros()` = tick x 1000 + TIM7 CNT; ARR change timing (preload) | Medium | possibly: `micros()` beyond 2^32 us (F1-R2) needs the fake clock to run long without per-ms cost (fast-forward helper) |
| `HostDcd` + `UsbHost` (`host_dcd.cpp`, `UsbHost.h`) | interrupt OUT reports, feature SET / GET_REPORT, NAK / ACK flow, suspend / resume | TinyUSB `dcd_dwc2.c`, USB 2.0 ch. 8-9, HID 1.11 (SET_REPORT / GET_REPORT) | Medium-High | add an interrupt-OUT helper that sends a report in one or more packets if missing |
| `HostRtos` + `host_freertos_ext.inc` | task notifications from the timer ISR, `Delay`, resumable tasks, kernel-object counting | FreeRTOS V10.3.1 semantics (as the fake states) | Medium | never combined with the TMC driver thread before: check `Delay` budget handling (`setDelayBudgetMs`) under `HOSTTEST_RTOS_EXT` |
| `HostPlatform` flash map | board-wide save / restore (F1-P1, P2) | coverage P3 tests; the real `eeprom.c` is NOT in this executable (fixed-address NOR fake is not portable) | Medium | none; host-vs-target note |
| GPIO / EXTI (`host_target.cpp`) | enable pin PE7, FLAG pin PE8, E-stop input | RM0090 GPIO; L27 (IDR does not mirror ODR for outputs) | Medium | read outputs through ODR as today |
| `MockEncoder`, `MockMotorDriver` | not used on F1 | n/a | n/a | none |
| `WheelPlant` (new) | closed-loop tests | MATH, the stability derivation; energy and decay self-tests (F1-A3) | will be Low-Medium (documented) | built in I0 |

Audit output per fake: defects with spec citations, the coverage cases that rely on the defect (found by building a mutated copy of the fake in `../ffb-test-build/audit/<name>/`), and a confidence for what F1 needs.

## 5. Test categories (mapping to the charter)

| Category | Charter groups | Typical cases |
|---|---|---|
| Happy path | H, E, X, T | each effect type end to end to 0x64 with spec-derived values |
| Boundaries and saturation | E12, X1, X4, B2 | full-scale magnitudes, gains 0 / 255, power 0 / 0x7fff / above, saturation and dead band edges, int16 narrowing |
| Combined effects | E9, E12 | constant + conditions + endstop + idle spring; two axes |
| Extreme filter settings | R3-R6, O1-O4 | damper / inertia / friction gains 255, Q 1..127, fc 1..Nyquist, profiles 0-3, rate dividers |
| Timing and update rate | S1-S4, R1-R2, H6 | rate changes, report rates 60-1000 Hz, jitter, long uptime, SPI frame time |
| Faults and E-stop | T3-T5, T8, H5 | E-stop input, ENABLE_IN, undervoltage, over-temperature, SPI error, malformed reports |
| State changes | T1, T2, T6, T7, E10, E11, R5 | driver late ready, enable / disable, reset, commands mid-run, USB suspend |
| Persistence | P1, P2 | save, reboot, restore, same torque |
| 32-bit differences | B1-B3 | both builds per work order; `KNOWN_ISSUE_32BIT` where `size_t` matters |
| Stability / oscillation | O1-O5 | conformance, closed loop, sweeps, firmware-caused instability |

## 6. Stability and oscillation: assessment and proposed method

Oscillation was seen on real hardware and gets more likely with high damper and inertia settings. Two approaches were assessed.

### 6.1 Option A: gain, phase and stability of the filter chain (analysis)

The loop from position to torque is, at small signals and away from clips, a linear discrete-time system the code fully defines: backward difference x fs, the speed biquad (profile), the condition / axis-internal scale factors, the effect biquad, the gain chain (effect gain / global gain / `fxratio` / `power` / inversion), the inertia path's second difference and accel biquad, the slew limiter (nonlinear, ignored in the linear part) and the zero-order hold. With a plant J s^2 + b s (exact ZOH discretisation), the characteristic polynomial gives closed-loop poles, and the open-loop frequency response gives gain and phase margins.

* Feasible on the host: yes, in Python stdlib (complex arithmetic, frequency sweep for margins, Durand-Kerner roots for polynomials of degree < 15). No numpy needed.
* Fidelity: exact for the firmware's linear part (the transfer function is the code's math, and F1-O1 measures the real chain against it, so a wrong model or a wrong firmware filter is caught); the plant is nominal (no measured wheel); the chip's current loop is assumed ideal or a first-order lag (FakeTmc4671 has no FOC model); nonlinearities (friction relay, clips, slew limiter, deadbands) are not in the linear model.
* Alone it cannot show firmware-caused effects that are not linear (state resets, frozen filters, stale limiter state, NaN).

### 6.2 Option B: closed-loop wheel model (simulation through the real firmware)

`WheelPlant` integrates J theta'' = k_t i_q - b theta' - F_c sign(theta') (+ optional hands-on spring / damper) once per ms from the torque target the firmware writes to 0x64, and writes the resulting position to 0x6B as an integer PhiM count (65536 per revolution, DS). The real firmware then closes the loop exactly as on the board, at the FFB rate.

* Feasible: yes; the fake already has `onWrite` / register hooks, one plant step per scheduler ms, about 5 s simulated per case (5000 cycles).
* Fidelity: the firmware side is exact (real code, real filters, real gains, real sign flips, real nonlinearities); the plant is nominal: no cogging, no electrical dynamics beyond an optional current-loop lag, no backlash, no rim compliance, no real encoder noise (quantisation only; noise can be injected), no SPI / preemption latency. A host result therefore shows *whether the firmware loop is stable for the stated plant*, not for the owner's wheel.
* Alone it has no independent oracle: "it oscillated" is an observation, not an expected value.

### 6.3 Proposed approach: A as the oracle, B as the integration check

1. **Stability derivation (stability-analyst, I4 first work order):** writes `Tests/Integration_Tests/claude/charters/stability/F1_STABILITY_DERIVATION.md` and scripts in `Firmware/Tests/Integration_Tests/claude/oracles/`: the discrete loop model with every block cited to its source line and design value, the nominal plant with parameter ranges (stated, not measured: total inertia 0.005-0.08 kg m^2, viscous 0.0005-0.05 N m s/rad, Coulomb 0-0.2 N m, full-scale torque 5-25 N m for 32767 counts, current loop ideal or 0.2 ms lag, encoder 65536 counts per revolution, hands-off as the worst case), and a generated table of setting points with predicted closed-loop pole radius, dominant frequency, gain and phase margins.
2. **Conformance (F1-O1):** with no plant, the test drives 0x6B with a small sinusoid at chosen frequencies, measures the 0x64 torque by single-bin DFT over whole periods, and compares gain and phase with the model at each frequency. This is a plain test (MATH oracle); a mismatch is a firmware or model defect, investigated before any closed-loop result is trusted.
3. **Closed loop (F1-O2, O3):** for each setting point at least a stated margin away from the predicted boundary (no fragile tests on the boundary), apply a position impulse, run 5 s, and let `OscillationDetector` measure the cycle-to-cycle amplitude ratio and frequency; compare with the predicted pole radius and frequency within a derived tolerance. Sustained = amplitude ratio within 1 +- eps for >= 10 cycles above the quantisation level; growing = ratio > 1 + eps.
4. **Firmware-caused instability (F1-O4):** the same detector on runs where only the firmware's behaviour changes (setting change mid-run, rate change, filter reset, dead-band friction freeze): a configuration predicted stable must stay stable; a burst that the linear model does not predict is a suspected firmware defect (IB row with the derivation).
5. **Nonlinear limit cycles (F1-O5):** describing-function predictions for friction and the slew limiter; `characterization:` unless the derivation is exact.

Classification of results: instability predicted **and** observed inside the user-settable range at the nominal plant is not automatically a firmware bug (any controller can be made unstable by settings); it becomes an INR item with the derivation and the smallest unstable setting, for the owner to decide (warn, limit the setting, or accept). A mismatch between prediction and observation is a defect in the firmware or in the model and is investigated first.

## 7. Host-versus-target limits (what these tests cannot prove)

* No preemption: races between `hidOut` (USB thread) and `calculateEffects` (FFBMAIN), between the FLAG ISR and `updateReg`, and between TMCENC and FFBMAIN cannot be found (S11, TMC charter L3).
* Timing: blocking SPI takes zero or a fixed simulated time; real SPI latency, DMA, interrupt latency, jitter from higher-priority threads and CPU load (software double on the single-precision FPU) are not reproduced except by explicit injection.
* The TMC4671's FOC current / velocity / position loops, PWM, ADC conversion and encoder decoding are not modelled; the chip is a register file. Torque at the motor = the target at 0x64 under an ideal (or first-order) current-loop assumption.
* The wheel plant is nominal; the stability results hold for the stated parameter envelope only.
* CMSIS-DSP runs the portable C path, not the Cortex-M4 `ARM_MATH_DSP` kernels; float rounding and FMA contraction can differ on the target.
* The 32-bit build proves `size_t`, `long` and pointer widths, not ARM struct layout (`int64_t` / `double` 8-aligned on ARM, 4 on i386) or ARM's `uint32_t` = `unsigned long`; out-of-range float-to-int conversions differ (x86 gives INT_MIN, ARM saturates).
* Flash: the in-memory flash map, not `eeprom.c` on NOR flash (that is covered by `ffb_flash_tests`).
* MinGW / macOS are kept buildable in CMake but unverified (no toolchain here), as for the coverage suite.

## 8. Phases and stage gates

One phase per session; stop for the owner's approval at the end of every phase. Every work order (WO) passes the per-WO flow of `AGENT_PLAN.md` section 3.

| Phase | Content | Exit criteria (all required) |
|---|---|---|
| **I0 Infrastructure** | agent files and hooks installed (after approval); fake audit of section 4 and approved fixes; `integration.cmake`, `ffb_integration_tests` skeleton, ctest entries, option; `IntegrationFixture`, `ChainScheduler`, descriptor parser and `PidReportBuilder`, `WheelPlant`, `OscillationDetector`, `FrequencyResponse`; tool changes (`run_all_tests.py`, `make_integration_bug_report.py`, `it_gate.py`, `coverage_report.py` section) with Python tests; F1-A1-A5 | F1-A1-A5 green in 64-bit and 32-bit builds; all other executables' compile commands unchanged; `check_baseline.py` OK; ctest green in default, m32, san, clang, nodsp; fake audit report written and every fix approved by the owner; reviewer sign-off; documents updated |
| **I1 Open-loop chain** | G-H, G-E, G-X (F1-H1-H6, E1-E14, X1-X7) with the position scripted through 0x6B | every P1 requirement of these groups has a passing test or an approved `KNOWN_ISSUE`; P2 tested or reasoned out; both builds; baseline intact; reviewer sign-off; INTEGRATION_BUG_REPORT regenerated |
| **I2 Driver in the chain** | G-T, G-P (F1-T1-T9, P1-P2) | as I1 |
| **I3 DSP, reconstruction, timing** | G-R, G-S (F1-R1-R7, S1-S4); DSP-off variant | as I1, plus the nodsp build green |
| **I4 Stability** | stability derivation + oracle scripts (stability-analyst), G-O (F1-O1-O5) | derivation reviewed by `integration-reviewer`; conformance green; closed-loop cases agree with predictions; every predicted-unstable point inside the settable range recorded as INR; as I1 |
| **I5 F1 close-out** | G-B review across all files; integration coverage of the chain files (gcov of `ffb_integration_tests` alone) with every unexecuted chain line classified; mutation check (at least 30 mutants on the chain seams, survivors explained); F1 phase report (self-contained for a Codex review) | all F1 P1 requirements covered; mutation score >= 80 % with survivors explained; full variant gate (default, m32, san, clang, clang-san, nodsp, nodsp-san, cov); owner approval. **Only then F2 starts.** |
| I6-I10 | F2 ODrive, F3 VESC, F4 PWM, F5 SimpleMotion (GCC only), F6 MyActuator: one phase each in that order, each with its own charter (driver boundary = existing peer fake, audited first; CAN peers need the scheduler to host their 500 ms driver threads without the clock rewind of the unit fixtures) | as I1, per flow |

"Full coverage of F1" is measured as: every P1 requirement of the F1 charter proven (requirement coverage), plus the line coverage of the chain files reached by `ffb_integration_tests` alone, with each unexecuted line classified (unreachable from the chain, hardware-only, covered only by unit tests on purpose, or a gap).

## 9. Fake fixes (owner's override of `CLAUDE.md` for this track)

When the audit or a test shows that a shared fake in `Code_Coverage/` is wrong: (1) the auditor reports the defect with the spec citation; (2) the lead writes a fix WO; the writer fixes the fake in place; (3) the full coverage gate runs; every coverage case whose result changed is listed with the reason; a coverage test that passed only because of the wrong fake is edited too (exception to "never edit existing test files"); (4) the owner approves the list before the commit; (5) the same commit updates the coverage `PROGRESS.md` and `COVERAGE_AND_SCOPE.md` and regenerates `BUG_REPORT.md` with `make_bug_report.py`. `check_baseline.py` must pass, or each baseline difference is justified and approved. The PreToolUse hook blocks edits to `Code_Coverage/` except paths the lead has listed in `Tests/Integration_Tests/claude/plans/APPROVED_SHARED_EDITS.txt` after the owner's approval (`AGENT_PLAN.md` section 2).

## 10. Bugs, NEEDS REVIEW items and questions

* Own register (IQ1): tables "Integration bugs" (IB n) and "Integration NEEDS REVIEW items" (INR n) in `PROGRESS.md`; columns `# | Module | Location | Description | Same as (coverage) | Evidence (test name) | Resolved?`. Generated report `reports/INTEGRATION_BUG_REPORT.md` (`tools/make_integration_bug_report.py`; never edited by hand). A coverage bug hit again gets an IB row with "Same as: bug n" (so the integration report is complete on its own), and the coverage register is not changed.
* Markers: `KNOWN_ISSUE("(IB 3; same as bug 162) ...")`, `KNOWN_ISSUE_32BIT("(IB 7) ...")`; `characterization: [F1-E5] ...` with `// INR 4: ...` beside the case. The first failing assertion of a `KNOWN_ISSUE` case is the defect.
* Questions: `reports/OPEN_QUESTIONS.md`, numbered IQ n; asked in batches, never guessed.

## 11. Document set (`Tests/Integration_Tests/`)

| File | Content |
|---|---|
| `README.md` | map of the integration document trees and folder rules |
| `claude/README.md` | "Where do I find ..." |
| `claude/plans/INTEGRATION_TEST_PLAN.md` | this plan |
| `claude/plans/AGENT_PLAN.md` | agents, hooks, gate, per-WO flow, approvals |
| `claude/plans/PROGRESS.md` | "Current state" (all numbers), "Next action", decisions log, registers, environment pointer |
| `claude/plans/COVERAGE_AND_SCOPE.md` | requirement and chain-line coverage per flow, scope, not covered |
| `claude/plans/HANDOFF.md` | read order, method, pitfalls for a fresh session |
| `claude/plans/APPROVED_SHARED_EDITS.txt` | created in I0: paths under `Code_Coverage/` the owner approved for a fake fix (read by the hook) |
| `claude/reports/OPEN_QUESTIONS.md` | IQ questions |
| `claude/reports/INTEGRATION_BUG_REPORT.md` | generated |
| `claude/reports/I<n>_REPORT.md` | phase reports (self-contained, for Codex) |
| `claude/charters/F1_TMC4671_CHAIN_REQUIREMENTS.md` | F1 charter |
| `claude/charters/I0_WORK_ORDERS.md` | I0 work orders (draft) |
| `claude/charters/stability/` | stability derivation (I4) |
| `claude/charters/audit/` | fake audit reports (I0) |
