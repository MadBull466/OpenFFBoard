# Phase I1 report: open-loop chain (groups G-H, G-E, G-X)

*2026-11-23. Branch `add-test-suite`, phase commits `76a19dcb` .. this report's commit (local, not pushed; last push `f4f61e7e`). Written by the lead (Opus 5.5) to be read without the conversation; links point to the details.*

## 1. Result

Phase I1 is complete and waits for the owner's approval. `ffb_integration_tests` now drives the real chain USB HID PID -> `HidFFB` -> `EffectsCalculator` (CMSIS-DSP) -> `Axis` -> `TMC4671` open-loop: the host sends PID reports built from the descriptor the device reports, the encoder position at the chip is scripted, and the torque the firmware writes to register 0x64 is checked against PID 1.01, the TMC4671 datasheet and hand derivations. All 27 requirements of the groups G-H (HID entry), G-E (effects) and G-X (axis) have cases.

| Exit criterion | Status | Evidence |
|---|---|---|
| Every G-H, G-E, G-X requirement has cases (P2 included) | met | section 3 |
| Green in the 64-bit and 32-bit builds; full gate | met | section 2: PASS in all 8 variants |
| All other executables unchanged | met | compile check equal in all 8 variants, `check_baseline` OK, ctest 13 / 13 (12 / 12 without DSP) |
| Every work order reviewed in a fresh context, findings fixed | met | section 4 |
| Every suspected defect has an oracle-backed `KNOWN_ISSUE` or a characterization with an INR item | met | section 5 |
| Documents updated | met | `PROGRESS.md`, `HANDOFF.md`, `COVERAGE_AND_SCOPE.md`, `Firmware/Tests/README.md`, generated `BUG_REPORT.md` |

Numbers: 325 integration cases (102 at the end of I0, 223 new): 271 passed, 53 known issues (`KNOWN_ISSUE` / `KNOWN_ISSUE_UB`, failing as intended), 1 skipped (IB 36, a crash), 0 failed. Chain line coverage 50.5 % (I0: 32.0 %).

## 2. Full phase gate (`it_gate.py --wo I1-9 --full --isolated`, 2026-11-23)

Built from HEAD `73e31a36` plus the README change of this work order, in `../ffb-test-build/it-iso-I1-9-<variant>`.

| Variant | ctest | `check_baseline` | Integration cases (325) | Compile check |
|---|---|---|---|---|
| default | 13 / 13 | OK | 271 passed, 53 known issues, 1 skipped | equal (828) |
| m32 (32-bit) | 13 / 13 | OK | 271 passed, 53 known issues, 1 skipped | equal (823) |
| san (ASan + UBSan) | 13 / 13 | OK | 92 passed, 233 skipped | equal (828) |
| clang | 13 / 13 | OK | 271 passed, 53 known issues, 1 skipped | equal (823) |
| clang-san | 13 / 13 | OK | 92 passed, 233 skipped | equal (823) |
| nodsp | 12 / 12 | OK | 271 passed, 53 known issues, 1 skipped | equal (488) |
| nodsp-san | 12 / 12 | OK | 92 passed, 233 skipped | equal (488) |
| cov | 13 / 13 | OK | 271 passed, 53 known issues, 1 skipped | equal (828) |

Scope check OK, no build warning; evidence `../ffb-test-build/it-gate/I1-9/20261123T184923Z.json`.

**Sanitizer skips.** The sanitizer builds skip every case that brings the firmware up and deletes its main class at teardown (coverage bug 24, `IT_DELETES_FFBHIDMAIN`): 233 of 325, so in the gate most I1 cases never run under ASan / UBSan. They were run by hand in the `san` build with `--no-skip` and `ASAN_OPTIONS=new_delete_type_mismatch=0`: all 325 cases ran (one process per file for `test_f1_support`, `test_f1_hid`, `test_f1_effects`; one process per case for the other three files, 10 at a time). 322 cases gave no ASan or UBSan report. The 3 reports are exactly the three cases already marked for them: IB 3 (`KNOWN_ISSUE_UB`, heap-use-after-free in `HidCommandInterface`), IB 36 (the skipped crash case, null `Biquad` call at `EffectsCalculator.cpp:425`, same as coverage bug 12) and IB 18 (`KNOWN_ISSUE_UB`, `abs(INT_MIN)` at `Axis.cpp:431` after the out-of-range float conversion). No plain or characterization case depends on undefined behaviour. A process per file stops at the first abort, so the files with a sanitizer case have to be run per case.

**Chain line coverage** (`coverage_report.py` on the `cov` build, objects of `ffb_integration_tests` only, raw gcov lines): 2257 of 4470 lines (50.5 %; I0: 1429, 32.0 %).

| File | I0 | I1 | Main unexecuted parts (by function) and where they belong |
|---|---|---|---|
| `HidFFB.cpp` | 13.5 % | 89.9 % | destructor, `getFfbActive`: not on the open-loop path |
| `EffectsCalculator.cpp` | 10.2 % | 60.9 % | reconstruction filter / spline and timing (I3, G-R / G-S), `saveFlash` (I2 persistence), command and logging interface (`getInfo`, `listEffectsUsed`, `Run`): commands are I2 |
| `Filters.cpp` | 60.6 % | 69.3 % | default constructor, `getQ`; the remaining biquad set-ups belong to the filter chain of I3 |
| `Axis.cpp` | 40.8 % | 70.3 % | command replies and class lists, flash, E-stop and fault paths (I2: G-T, G-P) |
| `AxesManager.cpp` | 79.0 % | 79.0 % | second-axis paths: one axis in F1 |
| `FFBHIDMain.cpp` | 56.1 % | 59.5 % | gamepad input report (`update`, buttons / analog sources), `emergencyStop`, `exti`, `saveFlash`: input direction and E-stop are not on the force chain of I1 (E-stop I2) |
| `FFBWheel.cpp` | 100 % | 100 % | |
| `TMC4671.cpp` | 30.3 % | 32.8 % | calibration, encoder types other than the integrated one, the command interface, fault handling: phase I2 (driver in the chain) |
| `MotorDriver.cpp` | 59.1 % | 59.1 % | base-class defaults overridden by TMC4671 |
| `Encoder.cpp` | 41.7 % | 41.7 % | base-class defaults overridden by TMC4671 |
| `global_callbacks.cpp` | 22.7 % | 27.8 % | HAL callbacks of CAN, I2C, UART, SPI DMA, EXTI, timers (not on the F1 chain; CAN for F2 / F3 / F6), `tud_suspend_cb` / `tud_resume_cb` (I2) |
| `UsbHidHandler.cpp` | 75.0 % | 75.0 % | `transferComplete` (IN report completion) |
| `hid_device.c` | 43.4 % | 63.9 % | keyboard / mouse / gamepad helpers and protocol / idle callbacks the firmware does not use |

CMSIS-DSP on the path: pid init / reset 100 %, sin 87.5 % (I0 81.2 %), cos 81.2 %, biquad df1 70.6 % (init 100 %); spline and std 0 % (reconstruction filter and statistics, phase I3); sqrt and pid inline (not measurable).

## 3. Requirements (F1 charter, groups G-H, G-E, G-X)

Cases per requirement: plain (passing, oracle from a document or a hand derivation), KI (`KNOWN_ISSUE`, intended behaviour, failing because of the listed bug), char (characterization of behaviour no supplied document fixes, with an INR item). Test files: `Firmware/Tests/Integration_Tests/claude/cases/`.

| Req | P | Requirement (short) | File | Plain | KI | Char | Findings |
|---|---|---|---|---|---|---|---|
| F1-H1 | P1 | Create New Effect + Block Load give a usable Effect Block Index | `test_f1_hid.cpp` | 5 | 1 | 0 | IB 7 |
| F1-H2 | P1 | every declared output report type changes the effect as PID defines | `test_f1_hid.cpp` | 14 | 3 | 0 | IB 8, IB 9 |
| F1-H3 | P1 | PID Pool values match the effects the device accepts | `test_f1_hid.cpp` | 2 | 3 | 1 | IB 4, IB 5, IB 6, INR 4 |
| F1-H4 | P2 | PID State reflects actuators / pause / playing | `test_f1_hid.cpp` | 0 | 7 | 1 | IB 10, IB 11, INR 5 |
| F1-H5 | P2 | short, long, unknown-id OUT reports change nothing and do not stall | `test_f1_hid.cpp` | 4 | 3 | 1 | IB 12, IB 13, INR 6 |
| F1-H6 | P1 | reports sent while the FFB loop runs take effect in the next cycle, never half-applied | `test_f1_hid.cpp` | 3 | 0 | 0 | |
| F1-E1 | P1 | constant force: sign by direction and inversion, linear in Magnitude | `test_f1_effects.cpp` | 9 | 0 | 4 | INR 10, 11, 12 |
| F1-E2 | P1 | periodic waveforms: span, period, phase | `test_f1_effects.cpp` | 9 | 2 | 3 | IB 30, IB 31, INR 25, 26, 29 |
| F1-E3 | P1 | ramp from Ramp Start to Ramp End over Duration | `test_f1_effects.cpp` | 1 | 0 | 1 | INR 13 |
| F1-E4 | P1 | envelope attack / fade | `test_f1_effects.cpp` | 3 | 2 | 0 | IB 20, IB 21 |
| F1-E5 | P1 | spring from the chip position | `test_f1_conditions.cpp` | 7 | 0 | 2 | INR 14, 15 |
| F1-E6 | P1 | damper from the velocity | `test_f1_conditions.cpp` | 5 | 0 | 1 | INR 15 |
| F1-E7 | P1 | inertia from the acceleration | `test_f1_conditions.cpp` | 3 | 0 | 1 | INR 15 |
| F1-E8 | P1 | friction opposes motion, saturates, ramps up around zero | `test_f1_conditions.cpp` | 5 | 1 | 2 | IB 19, INR 15, 16 |
| F1-E9 | P1 | condition blocks per axis and along a direction | `test_f1_conditions.cpp` | 3 | 5 | 0 | IB 33, 34, 35, INR 30 |
| F1-E10 | P1 | effect operation: start, solo, stop, Duration, Start Delay, Loop Count | `test_f1_effects.cpp` | 7 | 2 | 0 | IB 27, IB 28 |
| F1-E11 | P1 | device control: disable, stop all, reset, pause / continue | `test_f1_effects.cpp` | 2 | 6 | 2 | IB 8, 9, 29, 32, INR 27, 28 |
| F1-E12 | P1 | combined effects sum and saturate without wrapping | `test_f1_conditions.cpp` | 3 | 0 | 0 | |
| F1-E13 | P2 | per-effect Gain | `test_f1_conditions.cpp` | 2 | 0 | 1 | INR 31 |
| F1-E14 | P2 | effect type change keeps a valid filter state | `test_f1_conditions.cpp` | 7 | 1 (+1 skipped) | 0 | IB 36 (skipped: crash, same as bug 12), IB 37 |
| F1-X1 | P1 | axis limit and chip limit agree | `test_f1_axis.cpp` | 3 | 2 | 0 | IB 14, IB 17 |
| F1-X2 | P1 | sign table: restoring spring and opposing damper in every combination | `test_f1_axis.cpp` | 3 | 2 | 2 | IB 15, INR 7, 9 |
| F1-X3 | P1 | position chain: one revolution is 360 degrees | `test_f1_axis.cpp` | 3 | 2 | 0 | IB 16 |
| F1-X4 | P1 | endstop | `test_f1_axis.cpp` | 4 | 0 | 1 | INR 17 |
| F1-X5 | P1 | multi-turn position and out-of-range stop | `test_f1_axis.cpp` | 3 | 2 | 2 | IB 18 (UB), IB 22, INR 8 |
| F1-X6 | P1 | idle spring and axis-internal effects | `test_f1_axis.cpp` | 5 | 0 | 2 | INR 18, 19 |
| F1-X7 | P1 | slew / speed limiter, fade-in, expo, equalizer, late readiness | `test_f1_axis.cpp` | 7 | 5 | 6 | IB 23, 24, 25, 26, INR 20-24, 32 |

No P2 requirement was left out. The cases of `test_f1_setup.cpp` (G-A, I0) and `test_f1_support.cpp` (helper self-tests) are not in the table.

## 4. Work orders and reviews

| WO | Content | Commit |
|---|---|---|
| I1-1 | `FfbHost` (PID reports from the parsed descriptor), `PositionScript`, `TorqueTrace`; F1-H1, F1-H3 | `76a19dcb` |
| I1-T1 | gate support for parallel writers (`--own-build`, `charters/wo/ACTIVE`, `--isolated`); sequencing rule | `3ebbe176` |
| I1-2 | HID entry: report types, PID State, robustness, ordering (F1-H2, H4, H5, H6) | `13cbeefd` |
| I1-3 | constant force, ramp, envelope (F1-E1, E3, E4) | `c76dadf3` |
| I1-5 | conditions with a scripted position (F1-E5 to E8) | `ed082e8a` |
| I1-7 | axis limits, sign table, position chain, multi-turn (F1-X1, X2, X3, X5) | `367c2837` |
| hook fix | `GATE PASS` evidence paths with spaces (owner approved) | `7d529d1b` |
| I1-T2 | compile check for the repository path with a space | `a3ab012b` |
| I1-6 | condition geometry, combined effects, gain, type change (F1-E9, E12, E13, E14) | `46a0730d` |
| I1-4 | periodic effects, effect operation, device control (F1-E2, E10, E11) | `f03a1a1e` |
| I1-8 | endstop, internal effects, shaping, late readiness (F1-X4, X6, X7) | `73e31a36` |
| I1-9 | phase gate, this report, documents | this commit |

Every test work order was reviewed by `integration-reviewer` in a fresh context; the writer never reviewed its own work. Decisions that changed the suite (details in the `PROGRESS.md` decisions log):

- **Device Control encoding (owner, IQ12):** the PID 1.01 §5.13 / Figure 16 command numbers are the oracle; the firmware's bit-flag decoding is IB 8. Where a defect hides behind IB 8, a second case sends the flag encoding the firmware understands so the next defect is visible on its own (IB 29, IB 32).
- **Rotated condition blocks (I1-6 review):** a direction makes a condition act along a line (PID 1.01 §5.3 example): θ and θ + 180° are identical, the projection share is sin² / cos². The charter's F1-E9 wording was corrected; the firmware's linear sin θ is IB 34; a committed I1-5 case that asserted the reversal was corrected.
- **Polar zero and absolute counts:** PID 1.01 defines no polar zero, so plain tests assert only relative signs and the 180° reversal; absolute counts derived from firmware constants are always `characterization:` with an INR item.
- **Sawtooth direction:** no supplied document fixes it, so one characterization (INR 29) replaced two `KNOWN_ISSUE` cases and IQ13 asks the owner.
- **I1-8 lead check:** the planned "late readiness" slew case first failed at a setup step; the cause is a different defect (a start request made during a supply wait is lost, IB 24) and the slew jump of coverage bug 33 is not reachable on that path. The case was relabelled and its oracle (the firmware's own start sequence, charter F1-X7) replaced the writer's PID §5.13 citation.
- **First failures:** for every `KNOWN_ISSUE` case of I1-4 and I1-8 the lead ran the case alone in both builds and checked that the first failing assertion is the defect; every case of those files finishes alone under `-s` within 90 s.

## 5. Findings: registers and questions

Full text: [BUG_REPORT.md](BUG_REPORT.md) (generated from `PROGRESS.md`).

- **Integration bugs:** 37 (IB 1-37), 33 new in I1 (IB 5-37). Four are the same as a coverage bug (IB 7 = bug 23, IB 14 = bug 32, IB 23 = bug 30, IB 36 = bug 12); IB 24 and IB 25 are related to bug 152. Grouped:
  - HID entry and PID state: IB 4-13 (pool counting, Block Free, Device Control decoding, Stop All never re-enabled, PID State not sent and flags wrong, no report-length check, zero-length and long transfers).
  - Effects: IB 19-21, IB 27-37 (friction coefficient sign, envelope with infinite Duration and on ramps, Start Delay, Loop Count, Pause, square period and phase, Stop All, rotated conditions, filters and start time after a type change).
  - Axis and driver: IB 14-18, IB 22-26 (power above 0x7fff wraps, invertForce, cpr 65535, torque limit 30000, out-of-range float conversion, stale 0x64 after a restart or a supply dropout, lost start request, fade-in time, slew rate at 500 Hz).
- **NEEDS REVIEW items:** 32 (INR 1-32), 28 new in I1 (INR 5-32): behaviour no supplied document fixes, recorded as `characterization:` cases for the owner's review.
- **Open questions:** IQ6, IQ9, IQ10, IQ11 (from I0) and **IQ13 (sawtooth direction, INR 29)**: [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md).
- No shared fake was changed in I1 (`APPROVED_SHARED_EDITS.txt` unchanged).

## 6. Host-versus-target limits of I1

- The 64-bit and the 32-bit (i386, `-m32`) builds give the same results for every case. The i386 build does not prove ARM behaviour of out-of-range float-to-integer conversions: IB 18 (`KNOWN_ISSUE_UB`) and the F1-B2 note in the IB 27 case (`EffectsCalculator.cpp:251-252`, `:274-275`, `:292-293`) convert out-of-range floats, where x86 and ARM differ.
- Effect time comes from `micros()` held in a float in the firmware; long uptimes lose resolution (noted in the IB 27 case). The tests run short sessions.
- The encoder position is scripted (`PositionScript`); no plant feedback in I1 (closed loop is I3 / I4).

## 7. Gate speed-up decided for the next step

The owner decided on 2026-11-23 to speed up the gate before the next phase (tool work order I1-T3): the integration executable runs once per variant; a work order's gate runs only its own integration test files (all of them when shared support, fakes, tools or CMake change; phase gates always run everything); cases and variants run in parallel with at most 10 jobs, failures are rerun alone and a pass on rerun is recorded for investigation; random-order and repeat runs come at the end of the integration testing. Phases stay strictly one after another. I1-T3 is proven against the current gate (identical per-case results in all 8 variants) before it is committed.

## 8. Next phase

I2: driver in the chain and persistence (groups G-T, G-P), after the owner's approval of this report and after I1-T3.
