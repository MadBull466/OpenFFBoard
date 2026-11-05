# ODrive CAN driver: requirements charter (phase D2)

Status: **signed by the lead (Opus 5.5), 2026-11-03.** Changes only by the lead, recorded at the end of this file and in the "Lead decisions log" of `Tests/claude/plans/PROGRESS.md`.

Driver under test: `Firmware/FFBoard/UserExtensions/Src/ODriveCAN.cpp`, `Inc/ODriveCAN.h` (classes `ODriveCAN`, `ODriveCAN1` (motor 0), `ODriveCAN2` (motor 1)).
Executable: `ffb_driver_tests`, tests in `Firmware/Tests/claude/cases/driver/test_odrive_*.cpp`.
Inventory (line references used below, `ODC` = ODriveCAN.cpp, `ODH` = ODriveCAN.h): `../ffb-test-build/ws/d2/odrive_inventory.md` (not committed).

## Sources

| Code | Source |
|---|---|
| SPEC | "CAN Protocol — ODrive Documentation 0.5.6", supplied by the user as `Tests/shared/specs/CAN Protocol — ODrive Documentation 0.5.6 documentation.pdf` (never committed). Cite as `SPEC <section>`. |
| CSV | "CAN Messages" table of the same page, supplied as `Tests/shared/specs/can-protocol.csv` (never committed): start byte, signal type, bits, factor, offset per message. Cite as `CSV <cmd id>`. |
| HD | Hand derivation in a comment next to the assertion |
| FW | Firmware behaviour only: test name starts with `characterization:`, NEEDS REVIEW entry |

Version note (L1): the driver comment names ODrive firmware 0.5.4 (`ODH:116`); the supplied spec is 0.5.6. The spec's cyclic-message table is stated "as of firmware version 0.5.4" (SPEC "Cyclic Messages"), and every message the driver uses has the same id and layout in the 0.5.6 table. The 0.5.6 spec is used as the oracle; a difference that only a 0.5.4 document could settle becomes a question to the user.

Facts every test author needs (verified by the lead in the PDF text and the CSV):

* **Arbitration id** (SPEC "CAN Frame"): 11-bit standard id `can_id = node_id << 5 | cmd_id`; node id 0..0x3F (6 bits), command id 0..0x1F (5 bits). Example: node 1, cmd 0x0C -> 0x2C. Extended ids (node up to 0xFFFFFF) exist in the spec but the driver uses standard ids only.
* **Byte order** (SPEC): all multibyte values little endian.
* **Call and response** (SPEC note): messages marked `*` (0x003, 0x004, 0x005, 0x009, 0x00A, 0x014, 0x015, 0x01D) are requested by the master with the RTR bit set; the axis answers with the same id and the payload. 0x017 and 0x016 / 0x01C are "Master***" (may be sent to either axis of a board).
* **Messages used by the driver** (CSV):

  | Cmd | Name | Direction | Payload (start byte, type, bits) |
  |---|---|---|---|
  | 0x001 | Heartbeat | ODrive -> | Axis Error @0 uint 32; Axis Current State @4 uint 8; Motor Error Flag @5.0 1 bit; Encoder Error Flag @6.0 1 bit; Controller Error Flag @7.0 1 bit; Trajectory Done Flag @7.7 1 bit |
  | 0x007 | Set Axis Requested State | -> ODrive | Axis Requested State @0 uint 32 |
  | 0x009 | Get Encoder Estimates (RTR) | request / response | Pos Estimate @0 float32; Vel Estimate @4 float32 (factor 1, offset 0) |
  | 0x00B | Set Controller Modes | -> ODrive | Control Mode @0 signed int 32; Input Mode @4 signed int 32 |
  | 0x00E | Set Input Torque | -> ODrive | Input Torque @0 float32 |
  | 0x010 | Start Anticogging | -> ODrive | no signals |
  | 0x017 | Get Bus Voltage and Current | request / response | Bus Voltage @0 float32; Bus Current @4 float32 |
* **Cyclic messages** (SPEC "Cyclic Messages", defaults): heartbeat 0x01 every 100 ms, encoder estimates 0x09 every 10 ms, all others 0 (off).
* **Not in the supplied spec: enum numbers.** Axis state numbers (the driver uses IDLE 1, FULL_CALIBRATION_SEQUENCE 3, MOTOR_CALIBRATION 4, ENCODER_INDEX_SEARCH 6, ENCODER_OFFSET_CALIBRATION 7, CLOSED_LOOP_CONTROL 8, ENCODER_HALL_PHASE_CALIBRATION 13; `ODH:22`), control mode TORQUE_CONTROL 1, input mode PASSTHROUGH 1 (`ODH:23-24`) and error bit meanings are not in SPEC or CSV. Until the user supplies them (question D2-Q1), assertions on these numbers are `characterization:` (L2).

## Requirements

Priority: **P1** must be tested; **P2** should be tested; **P3** characterization only / recorded.

### G1 Construction, port, filter

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| O1 | P1 | Constructing ODriveCAN1 (node 0) / ODriveCAN2 (node 1) takes the CAN port (started, not silent), registers one standard-id filter for its node (id `node<<5`, mask 0x7E0 = node bits only), and raises the port speed to at least preset 3 (250 kbit/s). | SPEC CAN Frame (node bits = id bits 10..5); FW for the speed floor | filter id/mask HD; speed `characterization:` |
| O2 | P1 | Only one instance per class can exist (`isCreatable` false while one exists); destruction frees the port, removes the filter and sends torque 0. | FW | `characterization:` for the destructor; isCreatable plain |
| O3 | P2 | ODriveCAN2 receives on FIFO1; frames for node 1 are processed once the FIFO1 interrupt is enabled (or a CAN interrupt drains it). | FW / host model | `characterization:` (host IRQ model, documented) |

### G2 Transmitted frames

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| O10 | P1 | Every frame the driver sends has standard id `node<<5 | cmd`, extended-id flag clear. | SPEC CAN Frame | exact id per node 0, 1, 5, 63 |
| O11 | P1 | Set Axis Requested State 0x07: DLC 4, uint32 little endian. | CSV 0x007 | bytes for state 3 and 8 (`03 00 00 00`, `08 00 00 00`); the numbers 3/8 themselves `characterization:` (L2) |
| O12 | P1 | Set Controller Modes 0x0B: DLC 8, control mode int32 LE @0, input mode int32 LE @4. | CSV 0x00B | bytes `01 00 00 00 01 00 00 00` (values `characterization:` L2) |
| O13 | P1 | Set Input Torque 0x0E: DLC 4, float32 LE @0 in Nm. | CSV 0x00E | bytes of the float HD (e.g. 1.0f = `00 00 80 3F`, -0.5f = `00 00 00 BF`) |
| O14 | P1 | Encoder estimates request 0x09 and bus voltage request 0x17 are RTR frames with DLC 0. | SPEC call & response | rtr flag, dlc 0 |
| O15 | P2 | Start Anticogging 0x10 carries no signals. | CSV 0x010 | driver sends DLC 1 with 0x00: `characterization:` + NEEDS REVIEW (harmless difference, L3) |
| O16 | P1 | The node id is limited to the 6 bits the protocol allows: a node id outside 0..63 is rejected and the previous node stays. | SPEC node id max 0x3F | **suspected bug B-O4** (`canid` int8 unvalidated, ODC:326): the value is stored and reported as set (64, -1, 200 -> -56) while the bus uses `node & 0x3F` (TX id truncated to 11 bits by the CAN peripheral, filter `& 0x3F`, ODC:69) and the saved value is `& 0x3F`, so a reload changes the node (L16) `KNOWN_ISSUE` |

### G3 Received frames

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| O20 | P1 | Heartbeat 0x01: axis error = uint32 LE @0, axis state = uint8 @4; readable through the `errors` and `state` commands. | CSV 0x001 | values HD |
| O21 | P2 | Heartbeat error flags are single bits (@5.0, @6.0, @7.0) and bit 7.7 is "trajectory done", not an error. | CSV 0x001 | driver stores whole bytes 5-7 (unused): `characterization:` note, no KNOWN_ISSUE (no observable effect, L4) |
| O22 | P1 | Encoder estimates 0x09: position float32 LE @0 (turns), velocity float32 LE @4; the position is used by `getPos_f` / `getPos`. | CSV 0x009 | values HD |
| O23 | P1 | Bus voltage 0x17: voltage float32 LE @0; `vbus` reports mV. | CSV 0x017 | 24.5 V -> 24500 (HD; truncation `characterization:`) |
| O24 | P1 | Frames of other nodes are ignored (no state change, no value change). | SPEC CAN Frame | values unchanged |
| O25 | P1 | Only data frames with the payload length the spec defines are used as values: an RTR frame or a frame shorter than the message's payload (e.g. another master's RTR request 0x09 seen on the bus, DLC 0) does not overwrite position / voltage. | SPEC call & response, CSV payload sizes | **suspected bug B-O3** (no DLC/RTR check, ODC:390-435) `KNOWN_ISSUE` |
| O26 | P1 | Extended-id frames are not CAN Simple standard frames and are not interpreted as ODrive messages. | SPEC CAN Frame (driver configured for 11-bit ids) | **suspected bug B-O5** (extId unchecked, ODC:390) `KNOWN_ISSUE` (the fake HAL filter passes an extended frame whose upper bits match, HostTarget.h:392) |

### G4 Connection, state machine, timing

All times in simulated ms; thread period 500 ms (`Delay(500)`), node timeout `> 2000 ms` since the last frame of the node.

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| O30 | P1 | `connected` (command) becomes true once a frame of the node was received and false after more than 2000 ms without any frame; on loss the driver resets to IDLE / waits for a heartbeat. | FW (timeout) | timing window from the code path, `characterization:` for 2000 ms |
| O31 | P1 | Without any frame from the node the driver does not report `connected` and does not request encoder estimates. | intent of "connected" | **suspected bug B-O2** (`lastCanMessage` starts 0: connected true for the first 2 s, ODC:205-212) `KNOWN_ISSUE` |
| O32 | P1 | Start-up: first heartbeat -> WAIT_READY; ODrive IDLE -> request FULL_CALIBRATION_SEQUENCE; calibration states (3, 4, 6, 7, 13) -> wait; ODrive IDLE again -> request CLOSED_LOOP_CONTROL; when the ODrive reports closed loop -> RUNNING with Set Controller Modes (torque, passthrough) and position zeroed (unless `storepos`). | FW (no ODrive state machine in the spec) | `characterization:` sequence with exact frames (bytes per O11/O12) |
| O33 | P1 | Whenever the driver enters RUNNING it has sent Set Controller Modes (torque control) before any torque is used: also when the ODrive is already in closed loop at connect, and when the closed-loop heartbeat arrives after START_RUNNING. | intent of the T5 path (ODC:174-185: the driver sends the mode when it starts running) | **suspected bug B-O6** (T7 path ODC:196-199 and `state=RUNNING` before the check ODC:175) `KNOWN_ISSUE` (one test per path) |
| O34 | P2 | A calibration the ODrive refuses (stays IDLE after the request) is retried or reported. | intent | `characterization:` of today's behaviour (no retry, `WAIT_CALIBRATION` has no case) + NEEDS REVIEW (L5: no defect asserted, policy question) |
| O35 | P1 | `motorReady()` is true only in RUNNING with the ODrive in closed loop. | FW | combinations |
| O36 | P1 | `startMotor` requests closed loop (0x07 = 8) when the ODrive is IDLE; `stopMotor` sends torque 0 and requests IDLE (0x07 = 1) when in closed loop; `emergencyStop(false/true)` = stop/start. | FW | frames; numbers `characterization:` (L2) |
| O37 | P2 | Calibration is started automatically on connect even when the motor was never started. | FW | `characterization:` + NEEDS REVIEW (safety policy: the motor moves during calibration) |
| O38 | P1 | Bus voltage is requested with RTR 0x17 while the last voltage reply is older than 1000 ms. | FW | timing `characterization:` (every 500 ms iteration while no reply) |
| O39 | P2 | Node loss and reconnect: after a loss and a new heartbeat, the start-up sequence runs again (IDLE -> calibration request; closed loop -> RUNNING). | FW | `characterization:` |

### G5 Torque and encoder

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| O40 | P1 | `turn(power)` sends Set Input Torque = power / 32767 * maxTorque (Nm), float32. | CSV 0x00E (unit Nm, float); FW scaling | HD: power 32767 -> maxTorque, -16384 -> -0.50002*maxTorque (float tolerance) |
| O41 | P2 | `turn` while not ready (disconnected / ODrive idle) still sends torque frames. | FW (check commented out, ODC:272-274) | `characterization:` + NEEDS REVIEW |
| O42 | P1 | `getPos_f` sends one RTR 0x09 per call while connected (again only after > 5 ms without a reply) and returns the last received position minus the offset (turns); `getPos` = 65535 * that, truncated. | CSV 0x009; FW for cpr 0xFFFF | values HD |
| O43 | P1 | `setPos(counts)` sets the offset so that `getPos` then returns `counts` for the last received position. | FW | HD |
| O44 | P2 | Positions beyond the int32 count range (> about 32768 turns) are handled without undefined behaviour. | robustness | **suspected bug B-O8** (float -> int32 conversion, ODC:264) `KNOWN_ISSUE_UB` (P3 if not reachable cleanly) |

### G6 Commands (prefix `odrv`)

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| O50 | P1 | `canid`, `canspd`, `errors`, `state`, `maxtorque`, `vbus`, `anticogging`, `connected`, `storepos` answer get/set as documented; set commands produce the frames / settings of G1-G5. | FW + CSV layouts | per command |
| O51 | P1 | `maxtorque` get returns exactly the value set (hundredths of Nm, 0..4095) and a save/restore keeps it. | intent (get after set) | **suspected bug B-O1** (truncation `int32_t(maxTorque*100)`, e.g. set 53 -> get 52, ODC:356, 138) `KNOWN_ISSUE` |
| O52 | P2 | `canspd` set: values below 3 are raised to 3; values above the preset range are rejected. | FW | `characterization:` (uint8 narrowing: 256 -> 3) + NEEDS REVIEW |
| O53 | P2 | `canid` get does not change the filter; invalid command types are rejected. | FW | `characterization:` (filter rebuilt on get) |
| O54 | P2 | Both instances answer under their own instance number; both use class id 0x85 (`CLSID_MOT_ODRV0`). | FW | `characterization:` + NEEDS REVIEW |

### G7 Persistent settings

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| O60 | P1 | Save/restore of node ids (0x3D0: M0 bits 0-5, M1 bits 6-11, the other motor's field preserved), max torque and `storepos` flag (0x3D1/0x3D2: bits 0-11, bit 12). | FW (`eeprom_addresses.h` comment differs from the code: L6) | HD of the words; layout `characterization:` + NEEDS REVIEW for the comment mismatch |
| O61 | P1 | With `storepos` on, the position offset is saved (0x3D3/0x3D4) and restored at construction, so the reported position continues where it was, for offsets within the int16 range. | FW | HD |
| O62 | P1 | A stored offset of more than half a turn (> 32767 counts) is restored correctly. | intent of `storepos` | **suspected bug B-O7** (int16 wrap, ODC:145-146, 113-118) `KNOWN_ISSUE` |

### G8 Errors and robustness

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| O70 | P2 | ODrive axis errors (heartbeat) are not raised to the firmware error handler; CAN bus errors and TX failures are ignored by the driver; recovery is only the 2000 ms timeout. | FW | `characterization:` + NEEDS REVIEW |
| O71 | P2 | Bus-off / silent node: the driver detects the loss through the timeout and does not crash; TX failures (mailboxes full) do not block the thread longer than the port timeout. | FW + host model | `characterization:` |

## Fake peer: `FakeOdrivePeer`

Location `Firmware/Tests/fakes/FakeOdrivePeer.h/.cpp`, linked only into `ffb_driver_tests`. Works on the real `canport` through the fake HAL CAN (`HostTarget::canTxFrames()`, `canReceive`, `canTxComplete*`, error injection); `MockCANPort` cannot be used (the driver binds to `canport`, inventory 0.1).

Must model (fidelity header with SPEC/CSV citations):
1. One ODrive axis per node id; decodes frames addressed to its node by `id >> 5` / `id & 0x1F` (SPEC CAN Frame), only standard ids.
2. Strict decoding of master frames per CSV: 0x07 DLC 4 uint32, 0x0B DLC 8 two int32, 0x0E DLC 4 float32, 0x09 / 0x17 RTR DLC 0; anything else (wrong DLC, unknown cmd, extended id, RTR on a set message) is recorded as a violation; fixture teardown fails on unexpected violations (same pattern as `FakeTmc4671`). 0x10 with DLC != 0 is recorded as `undocumented` (L3), not a violation.
3. Responses: RTR 0x09 -> data frame with the current pos/vel; RTR 0x17 -> voltage/current; same id (SPEC call & response). Optional cyclic heartbeat (default 100 ms) and encoder estimates (default 10 ms, off by default to keep logs small) driven by simulated time (`advance(ms)` from the fixture).
4. Axis state model: requested state 0x07 applied after a configurable delay (calibration duration), so a test can script IDLE -> calibration -> IDLE -> closed loop. The numbers are the driver's enum numbers (L2: Low confidence until the user supplies the enums); refusal mode (stays IDLE).
5. Fault injection: silent (no heartbeat / no responses), late response, wrong node id, corrupted / short frame, extended-id frame, bus error states via `canSetErrorState`, TX mailboxes not completed.
6. Records: decoded frame log {time, cmd, rtr, dlc, decoded value}, last torque, last modes, requested states.
7. Not modelled: the ODrive's control loop and motor, its own error generation, CANopen, baud-rate mismatch (only presets), arbitration timing. Confidence: id/payload encoding High (CSV), call/response High, cyclic timing Medium, axis state behaviour Low.

Fixture `support/OdriveFixture.h` (driver tests only): target reset, flash clear, peer per node, `makeOdrive1()/makeOdrive2()` (resumable "ODRIVE" thread by handle, delay budget suited to the 500 ms loop: verify with a smoke test), `runFor(ms)` advancing time, the peer's cyclic messages and the thread together, `reachRunning()` helper (heartbeat IDLE -> calibration -> closed loop), TX auto-complete on by default, frame helpers, teardown violation check.

## Coverage target

`ODriveCAN.cpp` line coverage >= 85 %, remainder classified.

## Open questions to the user

* **D2-Q1** Enum numbers (axis states, control/input modes, error bits) are not in the supplied spec. Supply the 0.5.6 enums (e.g. `enums.py` or the API reference page) to turn the `characterization:` assertions on these numbers into spec checks.
* **D2-Q2** Version: driver written for 0.5.4 (comment), spec 0.5.6; any difference for the used messages?

## Lead decisions

* **L1** 0.5.6 spec is the oracle (see version note).
* **L2** Enum numbers are `characterization:` until D2-Q1 is answered; the byte encoding around them is asserted against the CSV.
* **L3** Start Anticogging with DLC 1 (spec: no signals) is a harmless difference: `characterization:` + NEEDS REVIEW, fake records it as undocumented, not as a violation.
* **L4** Heartbeat flag bytes vs 1-bit flags: no observable effect (flags unused): note only.
* **L5** Missing retry after a refused calibration and the automatic calibration on connect are policy questions (NEEDS REVIEW), not asserted defects.
* **L6** Flash layout of 0x3D0 differs from the `eeprom_addresses.h` comment (code: M0 bits 0-5, M1 bits 6-11; comment: 0-6, 7-12, speed 13-15): the code is characterized; the comment mismatch is a NEEDS REVIEW item.
* **L7** Suspected driver bugs are numbered B-O1..B-On; global numbers are assigned at the end of D2 (after bug 168).

## Lead decisions on the fake peer (2026-11-03, after the lead review of `FakeOdrivePeer` / `OdriveFixture`)

* **L8** FIFO1 / node 1: the fixture enables the CAN1 RX1 interrupt line by default (modelling deviation, documented in the fixture) so node-1 tests see frames at their time. The real F407 setting (RX1 line disabled in the CubeMX MSP init: a node-1 frame waits in FIFO1 until another CAN interrupt) is kept as one `characterization:` test (O3) with a NEEDS REVIEW entry: on target ODriveCAN2 only processes frames when a CAN TX completion or FIFO0 interrupt happens (in practice its own 500 ms voltage requests).
* **L9** Clock handling: the fixture schedules the "ODRIVE" thread itself and undoes the clock jump of `Delay(500)`. Accepted after review: the driver body always resumes at its exact wake time, so the clock the driver observes (thread body at t, received frames between t and t+500, next body at t+500) is monotonic. Restriction (documented): no other timed objects across `runFor()`.
* **L10** Axis-state model of the peer (calibration 2000 ms, ends in IDLE, refusal mode) is a Low-confidence assumption (not in the supplied spec); tests driven by it are `characterization:`.
* **L11** A master RTR request with DLC != 0 is recorded as a violation (assumption: CAN Simple requests carry no data; SPEC/CSV do not state the DLC of a remote frame). The driver sends DLC 0, so no test depends on the other case.

## Lead decisions during writing (2026-11-03)

* **L12 / B-O9** `getPos_f` measures its 5 ms re-request window from the last reply (`lastPosTime`, ODC:253) instead of from the last request: without replies every call sends an RTR 0x09, a request flood at the axis loop rate. `KNOWN_ISSUE` (O42).
* **L13 (O24)** The driver's own node check (ODC:390-393) is unreachable through the public interface (the CAN filter drops frames of other nodes); O24 is verified end to end only.
* **L14 / B-O10** `restoreFlash` chooses the M1 settings address (0x3D2) only inside the successful read of the node-id word 0x3D0 (ODC:96-102): without 0x3D0, ODriveCAN2 restores motor 0's settings word 0x3D1. `KNOWN_ISSUE` (O60). Rare in practice (`saveFlash` always writes 0x3D0).
* **L15** Command policies kept as `characterization:` + NEEDS REVIEW, no bug id: `canid` with an invalid command type answers OK (the ERR of `handleGetSet` is dropped), `canspd` narrows to uint8 / ignores presets above 5, both command handlers use class id 0x85.
* **L16** B-O4 scope: TX id and filter agree only because the CAN peripheral truncates the identifier to 11 bits; the defect is the stored/reported node id differing from the bus node and changing on reload. Tests that expect the truncated outcome without a marker are not allowed (removed in review of OW1).
