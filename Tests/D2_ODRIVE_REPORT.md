# Driver track D2: ODrive report (2026-11-03)

Status: **D2 complete, approved by me 2026-11-04.** D3 (VESC) not started: it needs my explicit authorisation and the VESC CAN protocol spec (packet ids, scaling, buffer/CRC scheme).

Details and the resume point: `Tests/PROGRESS.md` ("Driver track" section). Requirements, spec facts and all lead decisions (L1-L16): `Firmware/Tests/driver/ODRIVE_REQUIREMENTS.md`. Bugs: `Tests/BUG_REPORT.md` (bugs 169-178).

## Spec used

ODrive 0.5.6 "CAN Protocol" page (`Tests/CAN Protocol — ODrive Documentation 0.5.6 documentation.pdf`) and its "CAN Messages" table (`Tests/can-protocol.csv`). Both are git-ignored and never committed. The driver comment names firmware 0.5.4; every message it uses has the same id and layout in the 0.5.6 table, so 0.5.6 is the reference. The enum numbers (axis states, control/input modes, error bits) are in neither file, so tests on those numbers are `characterization:`.

## Commits (local only, nothing pushed)

`8e436a89` (ignore the CSV), `b27aeeac` (charter), `72c50795` (fake ODrive peer, fixture, smoke tests), `bc1c22d0` (tests OW1-OW4), plus the final D2 commit (review fixes, PROGRESS, bug report, this file).

## Results

| Item | Result |
|---|---|
| Tests | ODrive: frames 59, received frames/torque/encoder 54, state machine 42, commands/flash 57, peer self-tests 35, smoke 6. 22 known issues (suspected bugs) fail as expected |
| Whole driver executable | 689 cases, 0 failed (TMC4671 + ODrive) |
| Gate | Green in all 6 build variants + coverage build: ctest 8/8, `check_baseline` OK, no sanitizer findings, no warnings in test files |
| Coverage | `ODriveCAN.cpp` 263 / 269 lines = **97.8 %** (target 85 %). Uncovered: destructor variant lines (gcov artefact), two getters only the Axis calls, the "unknown command" reply |
| Review | Fresh reviewer: random order and isolated runs green; all findings fixed |
| Mutation check | 104 mutations; 98 killed (94 %). The 6 survivors cannot change any observable behaviour (equivalent). I re-ran all 9 former real survivors myself after the fixes: all killed |
| Fake ODrive | `FakeOdrivePeer` built from the 0.5.6 spec + CSV: frame encoding and call/response High confidence, cyclic timing Medium, axis-state behaviour and enum numbers Low |

## Suspected driver bugs (bugs 169-178 in `Tests/BUG_REPORT.md`)

- **169 (B-O1)** `maxtorque` reads back one hundredth lower than set for 282 of 4096 values (set 53 -> 52), and a save stores the lower value.
- **170 (B-O2)** The driver reports "connected" and requests encoder data during the first 2 s even when no ODrive has ever answered.
- **171 (B-O3)** Request (RTR) frames and too-short frames are parsed as data: position, voltage, error and state get overwritten with stale/zero values.
- **172 (B-O4)** Node ids above 63 or negative are accepted and reported, while the bus uses `node & 0x3F`; a reload changes the node.
- **173 (B-O5)** Extended-id frames are interpreted as ODrive standard frames.
- **174 (B-O6)** On two start-up paths the driver never sends "torque control mode" before it sends torque (ODrive already in closed loop at connect, or a late closed-loop heartbeat).
- **175 (B-O7)** A stored position offset of more than half a turn wraps on reload.
- **176 (B-O8)** Positions beyond about 32768 turns: undefined float-to-int conversion.
- **177 (B-O9)** Without replies, every position read sends a new request: a request flood at the axis loop rate.
- **178 (B-O10)** The second ODrive restores the first one's settings when the node-id flash word is missing.

## Notable observations (NEEDS REVIEW 176-184)

- On connect the driver starts a full calibration automatically (the motor moves), even if the motor was never started, and does not retry if the ODrive refuses.
- Torque frames are sent while disconnected or not ready (the ready check is commented out).
- ODrive errors are only readable through the `errors` command; nothing goes to the firmware error handler.
- On the real F407 the second ODrive's CAN FIFO has its interrupt disabled, so its frames are only processed when some other CAN interrupt fires.
- The flash layout comment in `eeprom_addresses.h` does not match the code.

## Open questions for me (open questions 13-14 in `PROGRESS.md`)

1. **D2-Q1** Supply the ODrive 0.5.6 enum definitions (`enums.py` or the API reference page) so the enum-number tests can be checked against the spec instead of being characterization.
2. **D2-Q2** Any differences between 0.5.4 and 0.5.6 for the messages the driver uses (0x01, 0x07, 0x09, 0x0B, 0x0E, 0x10, 0x17)?

TMC4671 questions Q1-Q4 (open questions 9-12) are still open as well.

## To resume later

Read `Tests/PROGRESS.md` "Current state" and "Next action". To continue: approve D2, answer the questions if possible, then authorise D3 (VESC) and supply the VESC CAN protocol spec.
