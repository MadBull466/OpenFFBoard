# Phase I2 report: driver in the chain and persistence (groups G-T, G-P)

*2026-11-25. Branch `add-test-suite`, phase commits `0917d45c` (work orders) .. this report's commit (local, not pushed; last push `f3c3e55f`). Written by the lead (Opus 5.5) to be read without the conversation; links point to the details.*

## 1. Result

Phase I2 is complete and was approved by the owner on 2026-11-25. All 11 requirements of G-T (the TMC4671 driver inside the chain: start-up, readiness, E-stop, hardware enable input, supply and temperature, commands, USB state, SPI faults, external encoder) and G-P (settings across a power cycle) have cases. Every case brings up the real chain (USB HID PID -> `HidFFB` -> `EffectsCalculator` -> `Axis` -> `TMC4671` on `FakeTmc4671`) and observes the chip's registers and the driver enable pin; the encoder position is still scripted (closed loop is I3 / I4).

Numbers: 436 integration cases (325 at the end of I1, 111 new: 87 requirement cases, 23 helper self-tests, 1 hidden watchdog helper): 365 passed, 69 known issues (`KNOWN_ISSUE` / `KNOWN_ISSUE_UB`, failing as intended), 2 skipped (IB 36, a crash; the hidden watchdog helper), 0 failed. Chain line coverage 64.1 % (I1: 50.5 %). Registers: 13 new integration bugs (IB 38-50), 20 new NEEDS REVIEW items (INR 33-52), 2 new owner questions (IQ14, IQ15).

## 2. Phase gate (`it_gate.py --wo I2-7 --full --isolated`, 2026-11-25, on `51847987`)

Evidence: `../ffb-test-build/it-gate/I2-7/20261125T020705Z.json` (outside the repository). `rerun_passed`, `dead_shards` and `case_timeouts` empty.

| Variant | Build | ctest | Baseline | Integration (436 cases) |
|---|---|---|---|---|
| default | 0 warnings | 12 / 12 | OK | 365 passed, 69 known issues, 2 skipped, 0 failed |
| m32 | 0 warnings | 12 / 12 | OK | same |
| clang | 0 warnings | 12 / 12 | OK | same |
| nodsp | 0 warnings | 11 / 11 | OK | same |
| cov | 0 warnings | 12 / 12 | OK | same (one process, no shards) |
| san | 0 warnings | 12 / 12 | OK | 97 passed, 339 skipped |
| clang-san | 0 warnings | 12 / 12 | OK | 97 passed, 339 skipped |
| nodsp-san | 0 warnings | 11 / 11 | OK | 97 passed, 339 skipped |

Compile commands of every other executable equal the recorded references in all variants (re-recorded in WO I2-T2 after showing that the only differences are the per-case watchdog). Whole gate about 25 minutes (`cov` single process about 11 minutes).

A first I2-7 run on `1294fb84` (2026-11-24) passed in 7 variants; `cov` failed only on the 600 s whole-process limit of that time (a standalone run took 687 s, all cases passed). The owner then ordered per-case time limits instead of whole-run limits (tool WOs I2-T1 and I2-T2, section 7).

**Sanitizer skips.** The sanitizer variants skip every case that deletes the main class (coverage bug 24, `IT_DELETES_FFBHIDMAIN`). The 196 cases of the six files I2 added or extended (`test_f1_support.cpp`, `test_f1_driver.cpp`, `test_f1_safety.cpp`, `test_f1_runtime.cpp`, `test_f1_persistence.cpp`, `test_f1_encoder.cpp`) were run by hand in the `san` build, one process per case, with `--no-skip` and `ASAN_OPTIONS=new_delete_type_mismatch=0`: 184 gave no report; 12 report the same heap-use-after-free in `HidCommandInterface` (`tu_memcpy_s`, a HID command reply sent from a buffer a `vector` reallocation freed) = IB 3, reached by the reboot, USB replug and driver re-selection flows of the reboot helper's self-tests and of `test_f1_driver.cpp`. Their results in the normal builds do not depend on the freed memory (torque and register checks only); no case was marked for it. IB 48 (`KNOWN_ISSUE_UB`, uninitialised stack buffer) is not visible to ASan.

## 3. Chain line coverage (`coverage_report.py` on the `cov` build of the gate, objects of `ffb_integration_tests` only, raw gcov lines)

2867 of 4470 lines (64.1 %; I1: 2257, 50.5 %; I0: 1429, 32.0 %).

| File | Lines | I2 | I1 | Main unexecuted parts |
|---|---|---|---|---|
| `HidFFB.cpp` | 237 | 90.7 % | 89.9 % | rejected-report branches already marked by IB rows; descriptor variants of other main classes |
| `EffectsCalculator.cpp` | 667 | 74.2 % | 60.9 % | reconstruction filters and spline (I3), rate change paths (I3), command getters not on the chain |
| `Filters.cpp` | 127 | 69.3 % | 69.3 % | filter designs not used by the chain's defaults (I3) |
| `Axis.cpp` | 677 | 84.2 % | 70.3 % | other driver / encoder types, hands-off detection (bug 35), commands not on the chain |
| `AxesManager.cpp` | 62 | 85.5 % | 79.0 % | second-axis paths (FFBWheel has one axis) |
| `FFBHIDMain.cpp` | 173 | 76.9 % | 59.5 % | rate-change commands (I3), reporting commands |
| `FFBWheel.cpp` | 16 | 100 % | 100 % | |
| `TMC4671.cpp` | 1928 | 52.2 % | 32.8 % | encoder calibration and alignment for ABN / hall / analog encoders and stepper motors (need a plant: I4), debug-only commands, ADC calibration, other hardware versions |
| `MotorDriver.cpp` | 22 | 68.2 % | 59.1 % | base-class defaults overridden by TMC4671 |
| `Encoder.cpp` | 24 | 58.3 % | 41.7 % | base-class defaults |
| `global_callbacks.cpp` | 194 | 33.5 % | 27.8 % | callbacks of peripherals not on the chain (CAN, UART, other timers) |
| `UsbHidHandler.cpp` | 16 | 75.0 % | 75.0 % | |
| `hid_device.c` | 166 | 63.9 % | 63.9 % | TinyUSB HID paths the firmware does not use (boot protocol, idle rate) |

CMSIS-DSP on the path: pid init / reset 100 %, sin 87.5 %, cos 81.2 %, biquad df1 70.6 % (init 100 %), spline 0 %, std 0 % (reconstruction: I3); `arm_sqrt_f32`, `arm_pid_f32` inline (not measurable). A line-by-line classification of every unexecuted chain line is the I5 close-out task (plan section 8).

## 4. Requirements (F1 charter, groups G-T, G-P)

| Id | Pri | Cases (file) | Result |
|---|---|---|---|
| F1-T1 | P1 | `test_f1_driver.cpp` | no non-zero torque reaches 0x64 while 0x1A is not FOC or the enable pin is low, with a real stop window (plain); start-up order (INR 33); power stage enabled while 0x64 holds the previous driver's torque on re-selection (IB 38); 0x5E at the enable instant (IB 17) |
| F1-T2 | P1 | `test_f1_driver.cpp` (+ I1 late-readiness cases in `test_f1_axis.cpp`) | exactly 0 at 0x64 until Running; first torque after Running equals a reference run already running; slew base kept across a short USB detach (IB 39 = bug 33) |
| F1-T3 | P1 | `test_f1_safety.cpp` | E-stop zeroes the torque, drops the enable pin, switches the PWM off and stays off while reports arrive (plain); recovery (INR 36, 37); release jumps beyond slewrate (IB 40 = bug 33) |
| F1-T4 | P1 | `test_f1_safety.cpp` | ENABLE_IN low stops the drive and zeroes 0x64 within one poll period (plain; the zero depends on the bug-39 path, INR 45); error readable within 500 ms (plain), discarded after (INR 44); `sys.errors?` empty (IB 41 = bug 7); drive briefly re-enabled (IB 42 = bug 39) |
| F1-T5 | P1 | `test_f1_safety.cpp` | undervoltage stops the motion mode (plain) but leaves the torque target and PWM on (IB 43); warning reaches the host (plain); over-temperature unreachable with the shipped thermistor formula (INR 42, IQ14) |
| F1-T6 | P1 | `test_f1_runtime.cpp` | each TMC command writes its register per the datasheet and never splits or loses a 0x64 frame (plain, nine stream cases); `trqbq_f=0` enables 1 Hz (IB 44); biquad enable bit ambiguity (INR 51, IQ15). The 0x63 read-modify-write race is a stated host limit |
| F1-T7 | P2 | `test_f1_runtime.cpp` | unplug zeroes the torque and switches the PWM off (plain); replug with a pause restarts correctly (plain); suspend leaves the torque on (IB 45 = bug 42); immediate replug ends in Shutdown (IB 47) |
| F1-T8 | P2 | `test_f1_runtime.cpp` | HAL_ERROR on a 0x6B read or 0x64 write gives no jump (plain); a lost 0x64 write is never repeated (IB 46); a timed-out read uses an uninitialised buffer (IB 48 = bug 140, `KNOWN_ISSUE_UB`) |
| F1-T9 | P2 | `test_f1_encoder.cpp` | DC and BLDC external-encoder paths: position, PHI_E_EXT rate and value, pole-pair scaling, sign agreement with the torque (plain); 0x1C rate (IB 2); 0xffff scaling (IB 49); read-only write in the BLDC alignment (IB 50 = bug 165) |
| F1-P1 | P1 | `test_f1_persistence.cpp` | axis, TMC and effects settings read back and the same torque sequence after `sys.save` and a power cycle (plain); `trqbq_q` not saved, documented in the changelog (INR 35); `invertForce` without `combineEncoder` not restored (INR 34); default power cycle ends in Shutdown without a USB replug (IB 24, widened) |
| F1-P2 | P1 | `test_f1_persistence.cpp` | restored power at 0x5E and restored inversions active before the first non-zero torque (plain) |

## 5. Work orders and reviews

| WO | Content | Commit | Review |
|---|---|---|---|
| I2-1 | helpers: `rebootKeepingFlash`, `pressEstop` / `releaseEstop`, `ChipEventLog` (`armSpiFailure`, `armSpiTimeoutAfter`, `runCommandOnWrite`), 18 self-tests | `9da13640` | 3 majors (mid-write command did not interleave as claimed, lost step sampling, silent USB replug in the reboot), 6 minors, all fixed |
| I2-2 | F1-T1, F1-T2 | `bd73289f` | 1 blocker (the replug defect, moved to I2-4) and 4 majors (vacuous safety cases, a characterization pinning a defect, a preset instead of a real path, bug 33 label), fixed |
| I2-3 | F1-T3, F1-T4, F1-T5 | `6fc7b6d1` | 3 majors (vacuous undervoltage case, a too short wait bound, an over-temperature case asserting its own formula), fixed |
| I2-4 | F1-T6, F1-T7, F1-T8 | `bc6ffba7` | 3 majors (first failing assertion not the defect in m32, a timing-dependent replug case, the biquad enable reading), fixed |
| I2-5 | F1-T9 | `1294fb84` | 2 majors (the DC path hid the BLDC path and bug 165; a characterization pinning IB 2), fixed |
| I2-6 | F1-P1, F1-P2 | `7bdec6a9` | 2 majors (Q not saved is documented, not a bug; a fault clear that could hide another fault), fixed |
| I2-T1 (tool) | per-case watchdog in `ffb_integration_tests`, no whole-run limit | `3735b97d` | 0 majors, 3 minors fixed |
| I2-T2 (tool) | watchdog in every doctest executable (owner-approved shared CMake change), no whole-run limits in the gate except the two Python entries | `0835a697` | 0 majors, 10 minors fixed |
| docs | documentation review of both tracks | `716bda9e` | lead check of every change |

Parallel work: I2-2, I2-3, I2-4 and I2-6 ran at the same time on separate files after I2-1 was committed; I2-5 followed. Each writer gated with `--own-build --jobs 2`; the lead committed each WO after its own `--isolated` gate.

## 6. Findings: registers and questions

- **Integration bugs:** 50 (IB 1-50), 13 new in I2 (IB 38-50). Same as a coverage bug: IB 39 and IB 40 = bug 33, IB 41 = bug 7, IB 42 = bug 39, IB 45 = bug 42, IB 48 = bug 140, IB 50 = bug 165. IB 24 widened (the start request is also lost at boot). Most serious for a user: IB 38 (stale torque meets an enabled stage), IB 42 (drive re-enabled after a hardware stop), IB 43 (undervoltage leaves torque and PWM on), IB 45 (USB suspend leaves torque on), IB 47 (an immediate USB replug leaves the wheel stopped), IB 24 (a configured board stays stopped after power-up until USB is replugged).
- **NEEDS REVIEW:** 52 (INR 1-52), 20 new (INR 33-52); INR 42 (unreachable over-temperature limit) and INR 51 (biquad enable) wait for IQ14 / IQ15.
- **Questions for the owner:** IQ14 (thermistor formula: which divider does the board use?), IQ15 (TMC4671 torque biquad enable: bit 31 or value 1?), and the open IQ6, IQ9, IQ10, IQ11, IQ13.
- No shared fake was changed in I2. Shared code changed once, with the owner's approval: the per-case watchdog in `Firmware/Tests/CMakeLists.txt` and `Code_Coverage/support/CaseWatchdog.*` (I2-T2), with one more skipped case in `ffb_flash_tests`; no other coverage result changed.

## 7. Gate changes decided during I2 (owner, 2026-11-24)

- No time limit on a whole test run; a time limit per test case in every doctest executable (`FFB_IT_CASE_TIMEOUT_S`, compiled defaults 30 / 60 / 120 s from measurements). The only whole-run limits left are the two small Python ctest entries (60 s / 120 s). Codex's two Python entries now have no limit (not ours to change).
- Sanitizer builds are not changed; their skipped cases are run by hand at each phase end, one process per case.
- With parallel writers the 10-job budget is split with `--jobs`. Documents must not change while an `--isolated` gate runs.

## 8. Host-versus-target limits of I2

- No preemption: a CDC command runs only between two activations of the FFB or TMC thread, never between two SPI frames of one activation, so the 0x63 read-modify-write race of F1-T6 cannot be produced.
- The reboot models a cold start: only the flash map survives; "supply present at boot" and a held E-stop across a power cycle are not modelled. An MCU-only reset with the TMC4671 powered (IB 38's second path) cannot be modelled (the reboot resets the chip).
- The BLDC external-encoder case uses a rotor that follows the commanded alignment angle (a position script, not a plant); real alignment dynamics wait for I4.
- The 32-bit build does not prove ARM's out-of-range float conversions or struct layout (as in I1).

## 9. Hook gap noticed

During the documentation review, reviewer agents without write paths were blocked when they used the edit tools, but their shell or Python writes to repository files were not blocked (`guard_bash.py`). Every such edit was checked by the lead before the commit. The hooks were not changed (they need the owner's word).

## 10. Next phase

I3: DSP, reconstruction and timing (groups G-R, G-S; DSP-off variant), after the owner's approval of this report.
