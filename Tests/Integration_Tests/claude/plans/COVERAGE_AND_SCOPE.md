# Integration track: coverage and scope

Kept current at the end of every work order group and phase, in the same commit as the `PROGRESS.md` update. Numbers: `PROGRESS.md` "Current state" only (this file names what is measured and where, not the values).

## 1. How coverage is measured

| Measure | Definition | Source |
|---|---|---|
| Requirement coverage | each requirement of a flow charter: covered (passing test), known issue (approved `KNOWN_ISSUE`), not tested (reason), open | the flow charter in `claude/charters/`, test names carry the id |
| Chain line coverage | gcov lines of the chain files executed by `ffb_integration_tests` **alone** (the other executables' `.gcda` are not counted), every unexecuted line classified: unreachable from the chain / hardware-only / covered only by unit tests on purpose / gap | `coverage_report.py` section "Integration (F1 chain)" (added in I0) |
| Mutation score (phase close) | mutants on the chain seams killed by the integration tests | phase report |

Chain files of F1: `HidFFB.cpp`, `EffectsCalculator.cpp`, `Filters.cpp`, `Axis.cpp`, `AxesManager.cpp`, `FFBHIDMain.cpp`, `FFBWheel.cpp`, `TMC4671.cpp`, `MotorDriver.cpp`, `Encoder.cpp`, `global_callbacks.cpp` (USB callbacks), `UsbHidHandler.cpp`, TinyUSB `hid_device.c`, and the CMSIS-DSP functions on the path (`arm_biquad_cascade_df1_*`, `arm_spline_*`, `arm_sin_f32`, `arm_cos_f32`, `arm_sqrt_f32`, `arm_pid_*`, `arm_std_f32`).

## 2. In scope

| Flow | Chain | Phase | Status |
|---|---|---|---|
| F1 | HID PID (USB) -> HidFFB -> EffectsCalculator (CMSIS-DSP) -> Axis -> TMC4671 (`FakeTmc4671`), with feedback, supply, init, faults, E-stop, commands, persistence, timing, stability | I0-I5 | I0 in progress: fixture and scheduler exist, group G-A (F1-A1 to F1-A5) covered (count in `PROGRESS.md`) |
| F2 | same chain with ODrive (`FakeOdrivePeer`, CAN) | I6 | planned after F1 |
| F3 | VESC (`FakeVescPeer`, CAN) | I7 | planned after F2 |
| F4 | PWM (`MotorPWM`, fake TIM1) | I8 | planned after F3 |
| F5 | SimpleMotion (`FakeSmDevice`, UART; GCC only) | I9 | planned after F4 |
| F6 | MyActuator (`FakeRmdPeer`, CAN) | I10 | planned after F5 |

## 3. Out of scope (decided)

| Item | Reason |
|---|---|
| Real ST HAL under the chain | IQ4 (owner, 2026-11-18): functional fakes only |
| Real `eeprom.c` / NOR flash in the integration executable | fixed-address fake is not portable (macOS / Windows); covered by `ffb_flash_tests`; the integration uses the in-memory flash map |
| MIDI, TMCDebugBridge, CAN bridge / input main classes, serial FFB (`fxm`) | not on the F1-F6 force chain; covered by the coverage suite |
| Preemption races, real timing, FOC physics, real wheel parameters | host limits (plan section 7) |

## 4. Not covered yet

Everything in section 2: no integration test exists yet (planning).
