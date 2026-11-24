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
| IQ7 | `run_all_tests.py` misreports 3 VESC cases (tool defect T-1) | 2026-11-18 | answered 2026-11-18: fix in WO I0-8 |
| IQ8 | USB HID 1.11 class definition for the descriptor parser (WO I0-6) | 2026-11-18 | answered 2026-11-18: supplied (`hid1_11.pdf`) |
| IQ9 | PID State input report without Effect Block Index (INR 2) | 2026-11-18 | **open** |
| IQ10 | Direction as ordinal-usage values (INR 3) | 2026-11-18 | **open** (low priority) |
| IQ11 | Block Load index 0 on failure outside the declared range 1..40 (INR 4) | 2026-11-18 | **open** |
| IQ13 | Direction of the Sawtooth Up / Down waveforms (WO I1-4, INR 29) | 2026-11-23 | **open** |
| IQ12 | Device Control encoding: descriptor bit flags vs PID 1.01 command numbers (WO I1-2) | 2026-11-19 | answered 2026-11-19: PID 1.01 command numbers (IB 8) |

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

**Cause (lead's investigation, 2026-11-18):** (1) `run_all_tests.py:352-362` treats a passing skip-decorated case as "not evidence of a fix" only when its description contains "undefined behaviour"; these say "KNOWN ISSUE (crash)" / "(memory corruption)" (null call through `MotorDriver::getEncoder()`, `VescCAN.cpp:156`; FILL_RX_BUFFER without a bound check, `VescCAN.cpp:630`), which have no visible effect in an isolated run, so the tool reports "no longer fails" as Failed. (2) The vendor-bug pattern `(?<![A-Za-z0-9_])V(\d+)` (`run_all_tests.py:390`) matches the `V11` / `V5` inside the driver-track labels `B-V11` / `B-V5`, which are firmware bugs 189 and 183 (coverage `PROGRESS.md:1919`, `:1925`).

**Answer (2026-11-18): fix it in WO I0-8** (which edits `run_all_tests.py` anyway, with the owner's approval of that file): classify passing skip-decorated crash / memory-corruption cases like passing UB cases, and resolve `B-V<n>` labels to their firmware bug instead of a vendor bug; Python tests for both.

## Batch 3 (phase I0, WO I0-6, 2026-11-18)

### IQ8. USB HID 1.11 class definition

WO I0-6 parses the firmware's HID report descriptor (item encoding: short items, main / global / local items, Report ID / Size / Count, logical ranges) to build PID reports independently of the firmware's C structs. That encoding is defined in the HID 1.11 class definition (§6.2.2), which was not in `Tests/Code_Coverage/shared/specs/`; PID 1.01 defines the PID usages but not the item encoding, and TinyUSB's `hid.h` macros are code under test, not an oracle.

**Answer (2026-11-18): supplied.** `Tests/Code_Coverage/shared/specs/hid1_11.pdf` (Device Class Definition for HID, Version 1.11, 5/27/01). The HID Usage Tables are not supplied: usages outside the PID page (the joystick input report) are parsed but their meaning is not asserted.

### IQ9. PID State input report without Effect Block Index (INR 2)

The firmware's PID State input report (read from the device over USB in WO I0-6) declares the status flags Effect Playing, Device Paused, Actuators Enabled, Safety Switch and Actuator Power, but no Effect Block Index; the field is commented out in `usb_hid_ffb_desc.h` (around lines 162-169). PID 1.01 §5.12 describes the state of the effect identified by its index, and the §6 example and Figure 15 include Effect Block Index (and ROM Flag). Without the index, "Effect Playing" cannot be attributed to an effect. Is leaving it out intended (e.g. for host compatibility), or a defect? If a defect, it becomes an IB entry with a `KNOWN_ISSUE` test.

**Answer:** open.

### IQ10. Direction as ordinal-usage values (INR 3, low priority)

The Direction collection of the Set Effect report holds two 16-bit values (logical 0..36000, Report Count 2) with Ordinal usages (page 0x0A, instances 1 and 2), while PID 1.01 §5.1.1 says the Direction collection holds Generic Desktop axes or vectors (the report's Axes Enable collection does use Generic Desktop X / Y). Many force-feedback devices use ordinals here, so this may be intended for host compatibility. Intended?

**Answer:** open.

### IQ11. Block Load index 0 on failure (INR 4)

The PID Block Load feature report declares the Effect Block Index with logical range 1..40. PID 1.01 §9 (an example sequence) says that when a Block Load fails the device returns Effect Block Index 0, which a host following the descriptor would treat as out of range. Should 0 be inside the declared range (logical minimum 0), or does the firmware report failure only through Block Load Status? The behaviour on failure is tested in F1-H1 / F1-H3; this question is about the declared range.

**Answer:** open.

## Batch 4 (phase I1, WO I1-2, 2026-11-19)

### IQ12. Device Control encoding (WO I1-2 review)

The firmware's HID descriptor declares the PID Device Control field as 8 one-bit Variable fields (`usb_hid_ffb_desc.h:574-578`, `75 01 95 08 91 02`); HID 1.11 §6.2.2.5 reads that as eight on/off switches, and the firmware decodes the byte as bit flags (`HidFFB.cpp:221-239`: 1 Enable, 2 Disable, 4 Stop All, 8 Reset, 16 Pause, 32 Continue). PID 1.01 §5.13 calls the field a selector (NAry) and Figure 16 shows one command byte with the numbers 1..6 (Stop All 3, Reset 4, Continue 6); PID's own §6 example carries the same 1-bit bytes. Sent as PID numbers, Reset and Continue do the wrong thing; sent as flags they work. Which encoding is intended? (Stop All followed by Start gives no torque under either encoding: IB 9 regardless.)

**Answer (owner, 2026-11-19):** PID 1.01 command numbers. The descriptor layout plus the bit-flag decoding is integration bug IB 8; the tests keep sending Figure 16 command bytes. This supersedes the I0-6 decision log entry "Device Control ... no finding".

## Batch 5 (phase I1, WO I1-4, 2026-11-23)

### IQ13. Direction of the Sawtooth Up / Down waveforms (INR 29)

The firmware's Sawtooth Up force falls within each period and its Sawtooth Down force rises (`EffectsCalculator.cpp:239`, `:262`), relative to a constant force of positive Magnitude. PID 1.01 (and 1.02) only name the effect types ET Sawtooth Up / ET Sawtooth Down in Table 2 and section 5.1, and 5.1 even describes ET Sawtooth Down as defining "a Sawtooth Up waveform" (a typo); HID 1.11 says nothing; the conventional reading (e.g. DirectInput DIEFT_SAWTOOTHUP: rises) is not a supplied specification. Should "Sawtooth Up" rise (then the firmware swaps the two and the I1-4 characterization becomes a known issue), or is the firmware's direction intended? Until answered, a characterization case records what the firmware does.

**Answer:** open.

## Batch 6 (phase I2, 2026-11-24)

### IQ14. Thermistor formula and the unreachable over-temperature limit (INR 42)

The TMC driver converts the thermistor ADC word with r = R2 * 43252 / adc (`TMC4671.cpp:365-385`, 43252 = the count at 3.3 V on a 2.5 V reference) and a Beta formula, and stops the motor above `temp_limit` = 90 C. Temperature checking is on by default (hardware config v1.3, `TMC4671_configs.cpp:53-72`). Even at the largest ADC word (0xFFFF -> adc 32768) the formula gives 62.72 C for v1.3 (83.84 C for v1.2), so the over-temperature stop can never be taken, and no command changes the thermistor settings. r = R2*Vref/V matches neither standard divider equation (R2*(Vref-V)/V or R2*V/(Vref-V)); with the standard equation for an NTC at the top, 90 C would be reachable. Which divider does the board use (thermistor position, R2, reference), i.e. what should the ADC-to-temperature mapping be? With the schematic the case becomes a plain test (or a `KNOWN_ISSUE` if the formula is wrong).

**Answer:** open.

### IQ15. TMC4671 torque biquad enable: bit 31 or value 1? (INR 51)

The TMC4671-LA datasheet Rev 2.07 describes the biquad enable field (`biquad_t_enable`, CONFIG_DATA of the torque biquad) as `bit(31)` with range 0..0x1, the same notation it uses for MODE_PID_TYPE in 0x63, which the firmware and the tests treat as bit 31. The firmware writes the torque-biquad enable as the value 1, i.e. bit 0 (`TMC4671.cpp:2271`, `bq.enable & 0x1`). Read literally, the enable is bit 31 and the firmware never enables the torque filter on the chip. Which reading is right (Trinamic application note, register map file or a hardware check)? With the answer the enable checks of `test_f1_runtime.cpp` become plain tests or a `KNOWN_ISSUE`.

**Answer:** open.
