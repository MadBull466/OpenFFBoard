# Integration track: open questions for the owner

Questions of the integration track (`Tests/Integration_Tests/`), numbered IQ1, IQ2, ... so they never clash with the coverage suite's questions 1-70 (`Tests/Code_Coverage/claude/reports/OPEN_QUESTIONS.md`). Answered questions there are not asked again here. An answer is recorded below the question with its date and is copied into the decisions log of `Tests/Integration_Tests/claude/plans/PROGRESS.md`.

## Batch 1 (planning session, 2026-11-18)

Asked in one batch before the test plan was written; all four answered the same day.

### IQ1. Numbering of integration bugs and NEEDS REVIEW items

Integration runs will hit many of the 220 known firmware bugs again (for example bugs 12/124, 32, 33, 39 and 162 lie on the HID -> EffectsCalculator -> Axis -> TMC4671 chain). Options:

- **A (recommended): continue the coverage suite's numbering.** New rows go into the existing "Suspected firmware bugs" / "NEEDS REVIEW items" tables of `Tests/Code_Coverage/claude/plans/PROGRESS.md`, with a column or tag saying "found by integration test" and the test name. `make_bug_report.py` and `run_all_tests.py` keep working unchanged, a known bug that an integration test hits again is referenced by its existing number (no duplicates), and `BUG_REPORT.md` stays the one list. Cost: the counters keep living in the coverage `PROGRESS.md`, so the integration `PROGRESS.md` links to them instead of holding them.
- **B: own register** in `Tests/Integration_Tests/claude/plans/PROGRESS.md` with its own prefix (IB1.., INR1..). Needs a second generator run or a change to `make_bug_report.py`, and known bugs must be cross-referenced by hand.

**Answer (2026-11-18): own register and own report.** Integration bugs and NEEDS REVIEW items get their own numbers (IB n, INR n) in `Tests/Integration_Tests/claude/plans/PROGRESS.md` and their own generated report `Tests/Integration_Tests/claude/reports/INTEGRATION_BUG_REPORT.md`. When an integration test hits a bug that the coverage suite already found, the IB row cross-references the coverage number (column "Same as"), the coverage register is not changed.

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
