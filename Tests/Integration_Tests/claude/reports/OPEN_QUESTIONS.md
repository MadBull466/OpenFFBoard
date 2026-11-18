# Integration track: open questions for the owner

Questions of the integration track (`Tests/Integration_Tests/`), numbered IQ1, IQ2, ... so they never clash with the coverage suite's questions 1-70 (`Tests/Code_Coverage/claude/reports/OPEN_QUESTIONS.md`). Answered questions there are not asked again here. An answer is recorded below the question with its date and is copied into the decisions log of `Tests/Integration_Tests/claude/plans/PROGRESS.md`.

## Index

| # | Question | Asked | Status |
|---|---|---|---|
| IQ1 | Numbering of integration bugs and NEEDS REVIEW items | 2026-11-18 | answered 2026-11-18: own register (IB, INR) and own report |
| IQ2 | USB HID PID 1.01 specification | 2026-11-18 | answered 2026-11-18: supplied, PID 1.01 is the oracle |
| IQ3 | Oscillation seen on real hardware | 2026-11-18 | answered 2026-11-18: nominal plant plus sweeps |
| IQ4 | Hardware boundary for the integration executable | 2026-11-18 | answered 2026-11-18: functional fakes |
| IQ5 | ENABLE_IN of the TMC4671 on the OpenFFBoard | 2026-11-18 | answered 2026-11-18: keep the assumption (ENI high) |
| IQ6 | Motor start after a runtime driver selection (INR 1) | 2026-11-18 | **open** |
| IQ7 | `run_all_tests.py` misreports 3 VESC cases (tool defect T-1) | 2026-11-18 | **open** |

Faults found by the track are in [BUG_REPORT.md](BUG_REPORT.md).

## Batch 1 (planning session, 2026-11-18)

Asked in one batch before the test plan was written; all four answered the same day.

### IQ1. Numbering of integration bugs and NEEDS REVIEW items

Integration runs will hit many of the 220 known firmware bugs again (for example bugs 12/124, 32, 33, 39 and 162 lie on the HID -> EffectsCalculator -> Axis -> TMC4671 chain). Options:

- **A (recommended): continue the coverage suite's numbering.** New rows go into the existing "Suspected firmware bugs" / "NEEDS REVIEW items" tables of `Tests/Code_Coverage/claude/plans/PROGRESS.md`, with a column or tag saying "found by integration test" and the test name. `make_bug_report.py` and `run_all_tests.py` keep working unchanged, a known bug that an integration test hits again is referenced by its existing number (no duplicates), and `BUG_REPORT.md` stays the one list. Cost: the counters keep living in the coverage `PROGRESS.md`, so the integration `PROGRESS.md` links to them instead of holding them.
- **B: own register** in `Tests/Integration_Tests/claude/plans/PROGRESS.md` with its own prefix (IB1.., INR1..). Needs a second generator run or a change to `make_bug_report.py`, and known bugs must be cross-referenced by hand.

**Answer (2026-11-18): own register and own report.** Integration bugs and NEEDS REVIEW items get their own numbers (IB n, INR n) in `Tests/Integration_Tests/claude/plans/PROGRESS.md` and their own generated report `Tests/Integration_Tests/claude/reports/BUG_REPORT.md`. When an integration test hits a bug that the coverage suite already found, the IB row cross-references the coverage number (column "Same as"), the coverage register is not changed.

### IQ2. USB HID PID 1.01 specification

Open question 3 of the coverage suite is still unanswered: no `pid1_01.pdf` (USB Device Class Definition for Physical Interface Devices 1.01) is in `Tests/Code_Coverage/shared/specs/`. The integration chain starts at the PID output reports, so this spec would be the oracle for: report layouts and units (magnitude range, direction in 0.01 degrees, gain 0-255, condition coefficients, deadband, saturation), the sign convention of conditions, effect operation / device control semantics and the block load / pool reports. Without it, every expected value at the HID entry that the firmware's own report descriptor does not fix becomes a `characterization:` test with a NEEDS REVIEW entry, and the sign and direction conventions of the whole chain can only be checked for internal consistency, not against the spec.

**Answer (2026-11-18): supplied.** `Tests/Code_Coverage/shared/specs/USB_PID1_01.pdf` (Device Class Definition for PID, Version 1.0 / 1.01) and `USB_PID1_02.pdf` are now present. Oracle: PID 1.01 (the owner named 1.01 as the spec behind `HidFFB`, coverage decisions log 2026-10-23); 1.02 is used only to flag a difference.

### IQ3. Oscillation seen on real hardware

To build a faithful oracle for the stability tests, can you supply what is known about the oscillation: board and wheel (motor type, pole pairs, encoder and cpr, wheel or rim inertia if known, gear ratio), the settings at which it appeared (damper / inertia / friction gains, `axisdamper` / `axisinertia`, filter profile, `damper_f` / `_q`, `inertia_f` / `_q`, power, FFB rate) and what it looked like (frequency, growing or steady, with or without hands on the wheel)? Without data the plan uses a documented nominal plant (inertia + viscous + Coulomb friction, parameter ranges stated) and sweeps the settings; the tests then prove "no instability inside the analysed envelope", not "matches your wheel".

**Answer (2026-11-18): nominal model plus sweeps.** No hardware data; the plant is documented with stated parameter ranges, and the stability tests prove behaviour inside the analysed envelope.

### IQ4. Hardware boundary for the integration executable

The integration executable needs fakes under the firmware. Options:

- **A (recommended): the existing functional fakes** (`HostTarget` SPI / ADC / TIM / GPIO / CAN, `FakeTmc4671`, the fake dwc2 controller `HostDcd`), audited before use. Builds and runs in the 64-bit and 32-bit builds, portable to MinGW and CI.
- **B: the real ST HAL on the P12 register models** (`fakes/stm32_hal/`). Closer to the target, but 32-bit Linux only (mmap at fixed addresses, x86 trap flag), slow (per-access trapping) and the firmware's HAL use would have to be re-plumbed. Could be an optional later phase for selected flows.

**Answer (2026-11-18): A, the existing functional fakes**, audited before use. The real ST HAL is not used by the integration track.

### IQ5. ENABLE_IN of the TMC4671 on the OpenFFBoard (asked 2026-11-18, WO I0-2)

`FakeTmc4671` reports `INPUTS_RAW` 0x76 bit 15 (ENABLE_IN) as **high** by default; the datasheet's reset value is 0 and the fake marks "high" as an assumption. The driver's 2 s status check reads this bit and enters its E-stop path when it is low, so 9 coverage cases (every "Running" case of `ffb_driver_tests`) and every integration case with a running motor rely on it. The datasheet (§4.7.1 p52) says all controllers are off while ENI is low, so a working board must hold ENI high, but the OpenFFBoard schematic is not in `shared/specs/`. Options: keep the assumption as documented (integration tests also cover ENI low as the E-stop / fault case), or supply the schematic so the fake's default can be checked.

**Answer (2026-11-18): keep the assumption**, documented in the fake. A working board must hold ENI high for the controllers to run (DS §4.7.1); integration tests also cover ENI low as the fault / E-stop case.

## Batch 2 (phase I0, WO I0-5, 2026-11-18)

### IQ6. Motor start after a runtime driver selection (INR 1)

Found while bringing up the chain in WO I0-5 ([BUG_REPORT.md](BUG_REPORT.md) INR 1). With USB connected and a fresh flash, `axis.0.drvtype=1` (TMC_1) calls `startMotor()` before the driver is initialised (`Axis.cpp:534-543`); `startMotor()` only records the request (`TMC4671.cpp:1682-1687`), and when the driver reaches Shutdown (motor type and encoder type NONE) the request is not acted on: the motor stays off. Only a USB re-mount (`tud_mount_cb` -> `Axis::usbResume` -> `startMotor`) switches the driver to Running, and it does so although motor type and encoder type are still NONE.

Two questions:

1. After the user selects the TMC driver at run time (configurator connected), should the firmware start the motor by itself once the driver is set up, or is a re-connect / reboot the intended way?
2. Should the driver be able to reach Running with motor type NONE and encoder type NONE, or should it stay in Shutdown until a motor is configured?

Until answered, the test documents today's behaviour as `characterization:` (no `KNOWN_ISSUE`), and the integration fixture brings the firmware up through a USB re-mount ("board configured, PC connects afterwards"). If you decide either point is a defect, it becomes an IB entry with a `KNOWN_ISSUE` test of the intended behaviour.

**Answer:** open.

### IQ7. `run_all_tests.py` misreports three VESC cases (tool defect T-1)

Seen in every run of the coverage report tool during WO I0-3 (before and after the fake fixes): `Firmware/Tests/Code_Coverage/tools/run_all_tests.py` lists three skip-marked VESC cases of `ffb_driver_tests` as **Failed** with the note "known bug no longer fails - fixed? remove the marker" (they pass when the tool runs them alone with `--no-skip`), and attributes them to unrelated vendor bugs: `[V63] getEncoderType with useencoder off and no external encoder does not crash` -> V11, and two `[V24] ...` FILL_RX_BUFFER cases -> V5. This is a coverage-suite tool, outside the integration track's work orders. Options:

- **A:** investigate and fix the tool (and, if a marker is wrong, the cases) in a separate work order under the coverage track's rules.
- **B:** leave it as documented; the integration track compares reports before / after and ignores these three entries.

**Answer:** open.

