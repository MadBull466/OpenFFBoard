# Coverage and scope: what is covered, what is out of scope, what is not covered

Date: 2026-11-05, branch `add-test-suite`. Sources of the numbers (none re-measured today):
- Firmware, tier A/B/C: the gcov report of the P9 final build (2026-11-02), `Tests/claude/reports/PROJECT_SUMMARY.md`, `Tests/claude/plans/PROGRESS.md`.
- Drivers: `Tests/claude/reports/DRIVER_TRACK_REPORT.md` (2026-11-04).
- CMSIS-DSP P11: no coverage measured yet for the G1 files (the `dsp-cov` build has no data). Pass counts are from the G1 gate of 2026-11-05.
- Regenerate per-file numbers with `tools/coverage_report.py` (see "Environment" in `Tests/claude/plans/PROGRESS.md`).

## 1. Covered (in scope and tested)

### 1.1 Firmware outside `Firmware/Libraries/` (coverage ladder): 8,007 of about 8,095 lines, 98.9 % (87 files, all compiled and tested)

| Module | Files (`FFBoard/Src` or `UserExtensions/Src`) | Lines covered |
|---|---|---|
| Command system | CanCommandInterface 92.0 %, CmdParser 100 %, CommandHandler 99.2 %, CommandInterface 98.6 %, FFBoardMainCommandThread 98.1 %, HidCommandInterface 99.2 % | |
| Transport | CAN, CANPort2B, CanHandler, UART, UartHandler, USBdevice, UsbHidHandler 100 %; CDCcomm 87.9 %; global_callbacks 93.8 % | |
| FFB | EffectsCalculator 99.9 %, HidFFB 100 %, SerialFFB 100 % | |
| Axis | Axis 99.4 %, AxesManager 100 % | |
| Filters / math | Filters 100 %, CRC 100 % | |
| Error handling, persistence | ErrorHandler 98.0 %, PersistentStorage 100 %, flash_helpers 98.9 % | |
| System | SystemCommands 100 %, cppmain 97.8 %, ledEffects 100 %, voltagesense 100 % | |
| Main classes | CanBridge 99.5 %, CanInputMain 99.4 %, CustomMain, FFBHIDExt, FFBHIDMain_commmands, FFBJoystick, FFBWheel, FFBoardMain 100 %; FFBHIDMain 96.5 % | |
| Button sources | ButtonSource, SelectableInputs, CanButtons, LocalButtons, PCF8574 100 %; SPIButtons 99.4 %; ShifterAnalog 96.2 % | |
| Analog sources | AdcHandler, AnalogSource, ADS111X, CanAnalog, LocalAnalog 100 %; AnalogAxisProcessing 98.2 % | |
| Device interfaces, encoders | Encoder, MotorDriver, EncoderBissC, EncoderLocal 100 %; EncoderSSI 99.4 %; MtEncoderSPI 99.6 % | |
| Bus wrappers, buffers | SPI, I2C, I2CHandler, SpiHandler, TimerHandler, ExtiHandler, ringbuffer 100 %; ringbufferwrapper 95.5 % | |
| Descriptors | `usb_descriptors.cpp`, `usb_hid_1ffb_desc.c`, `usb_hid_2ffb_desc.c`, `usb_hid_gamepad.c` (tested through the USB stack) | |

Headers of these modules: 392 / 413 lines (94.9 %).

### 1.2 Vendor tier A (tested directly, 98.1 %, 770 / 785 lines)
- `FFBoard/Src/eeprom.c` 93.7 % (224 / 239) against a fake NOR flash with power-loss.
- FreeRTOS C++ wrappers `c*.cpp`: ccondition_variable, cevent_groups, cmem_pool, cmutex, cqueue, cread_write_lock, csemaphore, ctasklet, cthread, ctickhook, ctimer, cworkqueue: all 100 %.
- Python helpers: `Firmware/scripts/generate_memory.py` and `.../dwc2/dwc2_info.py` (61 `unittest` tests, line coverage not measured).

### 1.3 Vendor tier B: TinyUSB device code against a fake dwc2 controller (48.0 %, 755 / 1,572 lines)

| File | Covered |
|---|---|
| `USB/device/usbd_control.c` | 95.4 % |
| `USB/class/cdc/cdc_device.c` | 77.6 % |
| `USB/device/usbd.c` | 59.7 % |
| `USB/class/hid/hid_device.c` | 55.4 % |
| `USB/common/tusb_fifo.c` | 35.4 % |
| `USB/tusb.c` | 19.9 % |
| `USB/class/midi/midi_device.c` (MIDI is out of scope) | 13.7 % |

### 1.4 Driver track (`ffb_driver_tests`, 938 cases, outside the ladder)
`TMC4671.cpp` 89.8 % (1,731 / 1,928), `ODriveCAN.cpp` 97.8 % (263 / 269), `VescCAN.cpp` 95.7 % (382 / 399); driver tier 91.5 %. `TMC4671_configs.cpp` is part of the TMC4671 suite; no separate figure was recorded.

### 1.5 CMSIS-DSP (tier C and P11)
- Tier C, the 10 functions the firmware calls (100 %, 198 / 198 lines): `arm_sin_f32`, `arm_cos_f32`, `arm_biquad_cascade_df1_f32` and `_init_f32`, `arm_spline_interp_f32` and `_init_f32`, `arm_std_f32`, `arm_var_f32`, `arm_pid_init_f32`, `arm_pid_reset_f32`.
- P11 `ffb_dsp_tests` (218 cases, 0 failed): G1 done as tests, review pending. Covers FastMath f32 / f64 / f16 / q31 / q15 (sin, cos, sqrt, divide, atan2, vlog, vexp, vinverse, recip_q15) and the helpers `arm_abs_q15/q31`, `arm_add_f32/f16`, plus the pattern-loader self-tests.

## 2. Out of scope (decided by you)

| Area | Why |
|---|---|
| ST HAL, CMSIS core, FreeRTOS kernel | Vendor platform code, replaced by fakes |
| `Firmware/Targets/F407VG`, `F407VG_DISCO`, `F411RE` (CubeMX code, startup, linker scripts, `main.c`, `freertos.c`, HAL init) | Generated / board specific |
| PWM driver `MotorPWM.cpp` | Driver not selected (faked at the interface by `MockMotorDriver`) |
| MyActuator driver `RmdMotorCAN.cpp` | same |
| SimpleMotion driver `MotorSimplemotion.cpp` | same |
| `TMCDebugBridge.cpp` | Not selected |
| MIDI: `MidiMain.cpp`, `MidiHandler.cpp` (and `class/midi` in TinyUSB) | Not selected, feature macro kept off |
| Other CMSIS-DSP modules: Matrix, Transform, Complex, Distance, Quaternion, SVM, Bayes, Window, rest of Support, Helium / Neon variants, `dsppp`, Python wrapper | Not used by the firmware |
| Hardware access, flashing, real USB / CAN / serial, network | Never part of host tests |
| CI workflow (`.github/`) | Not edited; a proposal is open question 5 |

## 3. In scope but NOT (fully) covered

### 3.1 Not yet tested (planned P11 groups)
- G2: CMSIS-DSP Interpolation (12 units) and Controller (19 units, including `sin_cos`, Clarke / Park).
- G3: Statistics (93 units, about 289 ARM test ids), plus the int64 helpers of `dsp/utils.h`.
- G4: Filtering part 1 (biquad variants, FIR, decimation / interpolation).
- G5: Filtering part 2 (convolution, correlation, LMS, lattice, sparse FIR, Levinson-Durbin), plus `arm_recip_q31`.
- G1 FastMath: review, mutation check and coverage measurement are still to do; approval pending.
- G1 deliberately untested: vlog of 0 / negative, q15 0/0, f64 sin / cos / sqrt (do not exist, ARM disables them), f16 sin / cos (do not exist).

### 3.2 Partly covered, lines left (firmware ladder, 88 lines of about 8,095)
`CanCommandInterface.cpp` (18), `global_callbacks.cpp` (12), `eeprom.c` (15, unreachable or unmodelled flash states), `FFBHIDMain.cpp` (6), `ShifterAnalog.cpp` (7), `CDCcomm.cpp` (4), `Axis.cpp` (4), `ringbufferwrapper.cpp` (3), and one to two lines each in about 20 other files. Line lists: run `tools/coverage_report.py`.

### 3.3 TinyUSB: large parts uncovered
The device stack is tested only for what the firmware uses: HID and CDC through the fake controller. Not covered: `tusb.c` (80 % of it), `tusb_fifo.c` (65 %), `hid_device.c` (45 %), `usbd.c` (40 %), parts of `cdc_device.c`. The TinyUSB directories `host/`, `typec/`, `osal/`, `portable/` (other ports) and the other classes are not compiled at all.

### 3.4 Drivers: lines left
- TMC4671: 197 lines (10.2 %); the per-line list is in `Tests/claude/reports/D1_TMC4671_REPORT.md` and the gcov data (I did not re-derive the reasons here).
- ODrive: 6 lines. VESC: 17 lines (`getInfo` via the class chooser, one gcov artefact, an unused third-VESC flash slot, the B-V11 null dereference, `sendPing`).
- Mutation survivors accepted as not killable: TMC M62, M75, M97; ODrive 6; VESC 6 plus M40, M71, M132.
- Driver behaviour that needs real hardware: timing at the real SPI / CAN clocks, electrical effects, real device firmware versions other than the supplied specs (see `Tests/claude/reports/DRIVER_TRACK_REPORT.md`).

### 3.5 Behaviour the host cannot show
- Host is 64-bit, the target is 32-bit ARM with a single-precision FPU: type sizes, FMA, out-of-range float to int, signed shifts. The CMSIS-DSP tests run the portable C path, not the Cortex-M intrinsic (`ARM_MATH_DSP`) path.
- Real-time behaviour: preemption, interrupt latency, priorities (the fake RTOS is cooperative with simulated time).
- Skipped by default: the crash `command with an unknown id is rejected`; the `effectsForces` out-of-range case (excluded); two spline cases in sanitizer builds; the heap-corrupting VESC cases (B-V5); the `KNOWN_ISSUE_UB_HIDDEN` DSP cases in sanitizer builds (27 skipped).
- Windows / MinGW and macOS: prepared in CMake, not verified here. `ffb_flash_tests` is skipped on macOS.
- Integration across modules (real driver inside `Axis`, end-to-end HID effect to bus frame, shared buses, board-wide save / restore) is not tested; it was proposed once and has not been approved.

## 4. Tests that exist but do not count as "coverage"
The suspected bugs (firmware bugs numbered up to 193, vendor bugs V1-V41), 199 NEEDS REVIEW items and 17 open questions are in `Tests/claude/reports/BUG_REPORT.md`, `Tests/claude/plans/PROGRESS.md` and `Tests/claude/reports/OPEN_QUESTIONS.md`. A `KNOWN_ISSUE` test runs the line but asserts the intended behaviour, so those lines count as covered while the behaviour is still wrong.
