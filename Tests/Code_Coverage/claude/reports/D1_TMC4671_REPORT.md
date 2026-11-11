# Driver track D1: TMC4671 report (2026-11-03)

Status (historic report; D2, D3 and P13-P16 were completed later, see `PROGRESS.md`): **D1 complete, approved by me 2026-11-03.** D2 (ODrive) and D3 (VESC) not started: each needs my explicit authorisation and its spec file in `Tests/Code_Coverage/shared/specs/` (ODrive CAN protocol and VESC CAN protocol are still missing).

Details and the resume point: `Tests/Code_Coverage/claude/plans/PROGRESS.md` ("Driver track" section). Requirements, datasheet facts and all lead decisions (L1-L34): `Tests/Code_Coverage/claude/charters/driver/TMC4671_REQUIREMENTS.md`. Bugs: `Tests/Code_Coverage/claude/reports/BUG_REPORT.md` (bugs 139-168).

## Commits (local only, nothing pushed)

`1e9c8c2c`, `0340e400` (D0 preparation and skeleton); `043d93f7` (charter, fake, fixture), `372d4ea2` (WO1-WO4), `db8796fb` (WO5, WO7, WO8), `c05d0f32` (WO6), `58da90ec` (review fixes, PROGRESS, bug report).

## Results

| Item | Result |
|---|---|
| Tests | `ffb_driver_tests`: 436 cases, 0 failed, 30 known issues (fail as expected), 3 skipped crashes |
| Gate | Green in all 6 build variants + coverage build: ctest 8/8, `check_baseline` OK, no sanitizer findings, no warnings in test files |
| Coverage | `TMC4671.cpp` 1,731 / 1,928 lines = **89.8 %** (target 80 %). Uncovered: mostly code with no caller in this build (GPIO, position/velocity biquads, feed-forward), plus branches the simple motor plants do not reach |
| Requirements | Every requirement of the charter (R1-R114) has a test, except four left out on purpose: R8, R30, R55, R103 (reasons in the charter) |
| Review | Fresh reviewer: random order and isolated runs green; findings fixed |
| Mutation check | 96 of 99 mutations caught. The 3 survivors are not catchable today: 2 only after bugs 163/164 are fixed, 1 makes no difference for any existing board version |
| Fake chip | `FakeTmc4671` built from datasheet Rev 2.07: frame/register model High confidence, status flags Medium, simulated motor plants Low |

## Most serious suspected driver bugs (full list: bugs 139-168 in `Tests/Code_Coverage/claude/reports/BUG_REPORT.md`)

- **162 (B-T24)** By the datasheet the chip's flux and torque filters are never enabled: enable flag is bit 31, the driver writes bit 0.
- **139 (B-T1)** Register reads at 5.25 MHz without the 500 ns pause the datasheet requires above 2 MHz.
- **145 (B-T7)** Encoder index-clear flag written to bit 9 of 0x25 instead of bit 8.
- **146 (B-T8)** Analog-encoder setup writes the two halves of 0x45 swapped.
- **164 (B-T26)** Over-temperature ends in HardError with the PWM still running.
- **163 (B-T25)** Recovery after a hardware emergency stop never runs; the critical error stays.
- **153 (B-T15)** Failed ADC calibration loops forever instead of reaching HardError.
- **142 (B-T4)** A chip that never answers is never reported.
- **154-156 (B-T16..18)** Three divisions by zero (e.g. ADC calibration when the chip reads 0, its reset value).

## Open questions for me (open questions 9-12 in `PROGRESS.md`)

1. **Q1** Registers 0x1E, 0x47, 0x5C are not in datasheet Rev 2.07, and the driver writes 0x41, which Rev 2.07 lists as read-only. Was the driver written against an older register map I can supply?
2. **Q2** The engineering-sample version value is not in the datasheet; its handling is only recorded as current behaviour.
3. **Q3** Board start-up (`main.c:1377`) drives SPI1_SS2/SS3 low and nothing releases them before the first TMC_1 transfer, so a chip on those pins would also be selected. Intended?
4. **Q4** Is there real-hardware evidence that the TMC filters work? If so, the datasheet bit position is wrong rather than the driver (bug 162).

Earlier open items still waiting: CI job proposal, CLAUDE.md/TEST_PLAN.md gate wording, optional HID PID spec.

## To resume later

Read `Tests/Code_Coverage/claude/plans/PROGRESS.md` "Current state" and "Next action". To continue: approve D1, answer Q1-Q4 if possible, then authorise D2 (ODrive) and supply the ODrive CAN protocol spec.
