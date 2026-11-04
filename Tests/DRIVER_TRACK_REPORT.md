# Driver track (P10) final report: TMC4671, ODrive, VESC (2026-11-04)

Status: **D3 (VESC) complete, waiting for my review.** D1 TMC4671 approved 2026-11-03, D2 ODrive approved 2026-11-04. Per-driver details: `Tests/D1_TMC4671_REPORT.md`, `Tests/D2_ODRIVE_REPORT.md`, this file for D3 and the whole track. Open questions: `Tests/OPEN_QUESTIONS.md`. Bugs: `Tests/BUG_REPORT.md`. Resume point: `Tests/PROGRESS.md`.

Nothing was pushed; all commits are local on `add-test-suite`. No spec file is committed (PDFs, the ODrive CSV and the VESC source are git-ignored). `Firmware/` outside `Firmware/Tests/` is unchanged.

## What was built

| | TMC4671 (D1) | ODrive (D2) | VESC (D3) |
|---|---|---|---|
| Bus / peer | SPI, `FakeTmc4671` register file | CAN, `FakeOdrivePeer` (CAN Simple) | CAN ext ids, `FakeVescPeer` (buffer protocol + CRC) |
| Spec (oracle) | TMC4671-LA datasheet Rev 2.07 | ODrive 0.5.6 CAN page + CSV | VESC slides + VESC firmware source 7.01 |
| Charter | `driver/TMC4671_REQUIREMENTS.md` R1-R114, L1-L34 | `driver/ODRIVE_REQUIREMENTS.md` O1-O71, L1-L16 | `driver/VESC_REQUIREMENTS.md` V1-V81, L1-L16 |
| Driver tests (doctest cases) | 412 | 218 | 201 |
| Peer self-tests | 23 | 35 | 48 |
| Line coverage of the driver | `TMC4671.cpp` 1,731 / 1,928 = **89.8 %** | `ODriveCAN.cpp` 263 / 269 = **97.8 %** | `VescCAN.cpp` 382 / 399 = **95.7 %** |
| Mutation check (fresh reviewer, survivors re-run by the lead) | 96 / 99 (97 %) | 98 / 104 (94 %) | 124 / 133 (93 %) |
| Suspected driver bugs | 30 (bugs 139-168) | 10 (169-178) | 15 (179-193) |
| NEEDS REVIEW | 166-175 | 176-184 | 185-191 |

Whole driver executable `ffb_driver_tests`: **938 cases, 0 failed** (default/clang23/nodsp/cov: 860 passed, 9 skipped, 69 known issues failing as expected; sanitizer builds: 855 passed, 19 skipped, 64 known issues). Driver tier coverage 2,376 / 2,596 lines = 91.5 %. All 6 build variants + coverage build: ctest 8/8, `check_baseline.py` OK, no sanitizer finding, no warning in test files; the other five executables are unchanged.

Method per driver (`Tests/DRIVER_TEST_PLAN.md`): Sonnet inventory -> lead charter from the spec -> Sonnet matrix -> Sonnet fake peer reviewed by the lead -> 4-8 Sonnet writers in parallel -> lead decisions on every question -> full gate -> fresh Sonnet reviewer with random-order, isolated runs and mutations -> fixes -> lead re-run of all real mutation survivors and final gate.

## D3 VESC in short

- Spec: the slides cover only ids, packet numbers 0-27 and STATUS; everything else (byte order, buffer protocol, CRC-16/XMODEM, COMM layouts, POLL_ROTOR_POS, PING/PONG, timeout) is taken from the VESC firmware source 7.01 you supplied.
- Tests: frames 70, protocol 73, states 33, encoder/commands/flash 90 (doctest cases per file incl. table rows), smoke 6, peer self-tests 48.
- Uncovered VESC lines: `getInfo` (only via the class chooser), destructor variant (gcov artefact), unused flash slot for a third VESC, the B-V11 null-dereference line (skipped crash), unknown command, dead `sendPing`, two `default:` branches.
- Review: no high finding, no false positive; gaps closed (driver's own address check, ids >= 0x80, encoder rate, flash offsets, delivery checks). Lead check with `--no-skip`: the skipped memory-corruption tests fail as intended or are caught by AddressSanitizer, except a 2-byte overflow inside the driver object, which no host tool can see.

## Most important suspected driver bugs (all three drivers)

- **162 TMC4671 (B-T24)** By the datasheet the flux and torque filters are never enabled (enable bit 31, driver writes bit 0).
- **139 TMC4671 (B-T1)** Register reads at 5.25 MHz without the datasheet's 500 ns pause.
- **164 / 163 TMC4671** Over-temperature leaves the PWM running; the hardware estop recovery never runs.
- **174 ODrive (B-O6)** On two start-up paths torque is sent without first selecting torque control.
- **171 / 177 ODrive** Request frames are parsed as data; a silent ODrive gets a request flood at the axis loop rate.
- **184 VESC (B-V6)** The firmware check reads the pairing flag as the test version: a paired 5.3 release is rejected, a 5.3.50 test build accepted.
- **183 / 192 / 193 VESC** Receive buffer without bound or length checks: long firmware replies (long hardware/firmware names) overflow the 64-byte buffer, short frames underflow into huge copies (crash).
- **186 VESC (B-V8)** An unsolicited PONG freezes the driver permanently.
- **187 VESC (B-V9)** After stop and start the old torque is sent again without any command.
- **179 VESC (B-V1)** Changing a CAN id leaks a filter bank and keeps the old id accepted.

## What the fakes cannot prove (needs real hardware)

- TMC4671: the FOC control loop, PWM outputs, ADC conversion, encoder decoding, SPI electrical timing (only the configuration is checked), whether the filter enable bit really is bit 31 (Q4), the chip's behaviour on undocumented registers (Q1). Plant-driven procedures (alignment, calibration, autotune) run against simple scripted plants (Low confidence).
- ODrive: the real axis state machine and calibration timing, enum numbers (Q13), any 0.5.4 vs 0.5.6 difference (Q14), node-1 reception through FIFO1 with its interrupt disabled on the F407.
- VESC: firmware 5.x reply layouts (Q16), the motor/current limits behind SET_CURRENT_REL, real bus timing and multi-VESC arbitration, whether STATUS frames should count as liveness.
- All: concurrency between ISR and thread on target (the host runs sequentially), CAN bus errors beyond the fake HAL model, MinGW/Windows (not run here).

## Host-vs-target caveats

- Float-to-int conversions out of range (B-T6, B-T13, B-O8, B-V12) are undefined: x86 wraps or gives INT_MIN, ARM saturates; those tests are `KNOWN_ISSUE_UB` and assert the intended value.
- Bytes beyond the CAN DLC are 0 in the fake HAL, but may be stale mailbox contents on target (affects the malformed-frame tests).
- Biquad coefficients are computed in single precision (and with CMSIS `arm_sin/cos` on target): tolerance 1e-6 (L19 of D1).
- The skipped memory-corruption tests (B-V5) behave differently with other allocators; they are skipped by default for that reason.

## Open questions for me

Collected in `Tests/OPEN_QUESTIONS.md`: project items 3, 5-8; TMC4671 9-12; ODrive 13-14; VESC 16 (5.3 FW_VERSION layout; STATUS frames as liveness). Question 15 (VESC spec gaps) was answered by the VESC source.
