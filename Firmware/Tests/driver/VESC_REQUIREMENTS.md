# VESC CAN driver: requirements charter (phase D3)

Status: **signed by the lead (Opus 5.5), 2026-11-04.** Changes only by the lead, recorded at the end of this file and in `Tests/PROGRESS.md`.

Driver under test: `Firmware/FFBoard/UserExtensions/Src/VescCAN.cpp` (`VC`), `Inc/VescCAN.h` (`VH`); classes `VescCAN`, `VESC_1` (address 0), `VESC_2` (address 1).
Executable: `ffb_driver_tests`, tests in `Firmware/Tests/cases/driver/test_vesc_*.cpp`.
Inventory: `../ffb-test-build/ws/d3/vesc_inventory.md`; protocol facts extracted from the VESC source: `../ffb-test-build/ws/d3/vesc_protocol_facts.md` (both not committed; line references below come from them and were spot-checked by the lead).

## Sources

| Code | Source |
|---|---|
| SPEC | "VESC 6 CAN Formats v0.1" slides, supplied as `Tests/VESC6_CAN_CommandsTelemetry.pdf` (never committed): extended id layout, packet numbers 0-27, SET_* scalings, STATUS 1-5. Cite `SPEC <slide/line>`. Third-party document with the author's own guesses ("assumed ...?"): only its definite statements are used. |
| SRC | VESC firmware source supplied by the user, `Tests/bldc-master` (git-ignored, never committed): **VESC firmware 7.01, test build 1** (`conf_general.h:24-27`). Cite `SRC <path>:<line>`. Used for everything SPEC does not give (byte order, buffer protocol, CRC, COMM ids and layouts, POLL_ROTOR_POS, PING/PONG, timeout). |
| HD | Hand derivation in a comment next to the assertion |
| FW | Firmware (OpenFFBoard) behaviour only: `characterization:` + NEEDS REVIEW |

Version note (L1): the driver accepts VESC firmware >= 5.3 (`FW_MIN_RELEASE` 5.3.51 for test builds, `VH:25`); SRC is 7.01. SRC has no history of older layouts, so the 7.01 layout is the oracle; where only a 5.x layout could decide, the requirement is `characterization:` and a question to the user.

Facts every test author needs (lead-verified in SRC):

* **Ids** (SPEC; SRC `datatypes.h:1161-1231`): extended 29-bit id `unit_id | packet << 8` (unit id bits 0-7, packet bits 8-15, upper bits 0). Packet numbers: FILL_RX_BUFFER 5, FILL_RX_BUFFER_LONG 6, PROCESS_RX_BUFFER 7, PROCESS_SHORT_BUFFER 8, SET_CURRENT_REL 10, PING 17, PONG 18, POLL_ROTOR_POS 56. Unit id 255 = broadcast (SRC `comm_can.c:1609`). COMM ids (SRC `datatypes.h:969, 991, 1019`): FW_VERSION 0, ROTOR_POSITION 22, GET_VALUES_SELECTIVE 50.
* **Byte order** (SRC `util/buffer.c:34-39, 155-184`): all multi-byte fields big endian; `float32(x, scale)` = `(int32)(x*scale)` (truncation), `float16(x, scale)` = `(int16)(x*scale)`.
* **SET_CURRENT_REL** (SPEC; SRC `comm_can.c:1814-1820`): DLC 4, int32 BE of ratio * 1e5 (-1..1 = lower..upper current limit); the VESC's own sender truncates (SRC `comm_can.c` `comm_can_set_current_rel`, `buffer.c:75-76`).
* **Request with a COMM payload** (SRC `comm_can.c:443-505`): payload <= 6 bytes -> one PROCESS_SHORT_BUFFER frame `[sender id, flag, payload...]` with DLC = len + 2; flag 0 = "process and reply to the sender id". Longer payloads -> FILL_RX_BUFFER frames `[offset, up to 7 bytes]` at offsets 0, 7, 14, ... then PROCESS_RX_BUFFER `[sender id, flag, len BE, CRC BE]` (DLC 6). Replies go to id `requester | packet << 8` with the VESC's own id in payload byte 0 and flag 1. FILL_RX_BUFFER_LONG (2-byte offset) only for messages longer than about 259 bytes.
* **CRC** (SRC `util/crc.c:26-64`): CRC-16/XMODEM (poly 0x1021, init 0, no reflection, no final xor) over the reassembled COMM message (COMM id byte included), high byte first. Check value crc("123456789") = 0x31C3.
* **Receiver rules** (SRC `comm_can.c:1646-1771`): FILL frames whose offset does not continue the running offset are dropped; PROCESS_RX_BUFFER is only accepted when the buffered byte count equals the announced length; rx buffer 512 bytes.
* **COMM_FW_VERSION reply** (SRC `commands.c:231-293`): `[0, major, minor, HW_NAME\0, UUID(12), pairing_done, FW_TEST_VERSION_NUMBER (0 = release), HW_TYPE, custom cfg, phase filters, QML hw, QML app, nrf flags, FW_NAME\0, hw crc (4)]`, length 29 + n + m (n, m = name lengths; 3 + (n+1) + 12 + 8 + (m+1) + 4).
* **COMM_GET_VALUES_SELECTIVE** (SRC `commands.c:384-479`): request `[50, mask u32 BE]`; reply `[50, mask, fields of the set bits in ascending bit order]`; bit 8 v_in int16 x10 (V), bit 15 fault code uint8 (`mc_fault_code`, 0 = none, SRC `datatypes.h:144-179`), bit 16 pid_pos int32 x1e6 (degrees). Example (mask 0x8100, 24.0 V, fault 0): payload `32 00 00 81 00 00 F0 00`, CRC 0x523A (HD in the facts file).
* **POLL_ROTOR_POS** (SRC `comm_can.c:2023-2030`): request id `target | 56<<8`, payload ignored; reply id `VESC own id | 56<<8`, DLC 4, int32 BE = degrees * 1e5. Broadcast requests are answered by every VESC.
* **PING/PONG** (SRC `comm_can.c:660-669, 1843-1861`): PING DLC 1 `[sender id]`; PONG to `pinger | 18<<8`, DLC 2 `[VESC id, hw type]`.
* **Timeout** (SRC `appconf_default.h:27-31`, `timeout.c:186-253`): default 1000 ms; only SET commands reset it; on timeout the VESC brakes with 0 A.
* **STATUS 1-5** (SPEC; SRC `comm_can.c:1216-1274`): id = VESC own id | packet << 8, sent only when enabled (default off). Not decoded by the driver.

## Requirements

Priority P1 / P2 / P3 as in D1/D2. Suspected bugs B-V1.. get global numbers at the end of D3.

### G1 Construction, port, filters

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| V1 | P1 | Construction takes the CAN port and registers two extended-id filters: replies addressed to the driver's id (low byte = `OFFB_can_Id`, default 0x40) and POLL_ROTOR_POS replies (packet 56 from any unit when the VESC id is 255, else low byte = VESC id). | SRC reply addressing / POLL rule | filter id/mask HD; frames accepted / rejected through `canReceive` |
| V2 | P1 | Changing `offbcanid` or `vesccanid` replaces both filters: the old ids are no longer accepted and no filter bank leaks. | intent of `setCanFilter` | **B-V1** (`filterIdVesc = -1` before `removeCanFilter`, VC:107-110) `KNOWN_ISSUE` |
| V3 | P2 | Only one instance per class; destruction stops sending, frees the port and removes its filters; speed preset raised to >= 3 if lower. | FW | `characterization:` |

### G2 Transmitted frames

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| V10 | P1 | Every frame is an extended data frame with id `VESC id | packet << 8`, bits 16-28 zero. | SPEC id layout | exact id for VESC ids 0x01, 0x7F, 0xFF |
| V11 | P1 | SET_CURRENT_REL: DLC 4, int32 BE = (int32)(ratio * 1e5) truncated, ratio = power / 32767. | SPEC, SRC buffer.c | HD bytes: power 16384 -> 0.500015 -> 50001 = `00 00 C3 51`; -32767 -> -100000 = `FF FE 79 60` |
| V12 | P1 | FW version request: PROCESS_SHORT_BUFFER, DLC 3, `[OFFB id, 0x00, 0]`. | SRC comm_can.c:445-455 | exact bytes |
| V13 | P1 | Values request: PROCESS_SHORT_BUFFER `[OFFB id, 0x00, 50, 00 00 81 00]` (mask bit 8 v_in + bit 15 fault), DLC 7 = payload 5 + 2. | SRC comm_can.c:445-455, commands.c:384-395 | **B-V2** (DLC 8 with an uninitialised 8th byte, VC:449-459) `KNOWN_ISSUE` for the DLC; the 7 defined bytes plain |
| V14 | P1 | Rotor position request: packet 56, DLC 0. | SRC comm_can.c:2023 | exact frame (if UBSan reports the null `memcpy` at VC:497: `KNOWN_ISSUE_UB` B-V3) |
| V15 | P3 | PING is never sent (`sendPing` has no caller). | FW | dead code, no test |

### G3 Receive buffer protocol and CRC

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| V20 | P1 | A reply sent as FILL_RX_BUFFER frames + PROCESS_RX_BUFFER with a correct CRC-16/XMODEM (high byte first) is decoded; a wrong CRC is ignored. | SRC crc.c, comm_can.c | CRC values HD / check vector 0x31C3; state change or not |
| V21 | P1 | A short reply (PROCESS_SHORT_BUFFER, payload <= 6) is decoded. | SRC comm_can.c:1773-1810 | decode effect |
| V22 | P2 | FILL_RX_BUFFER_LONG (2-byte BE offset) fills the buffer. | SRC comm_can.c:1671-1697 | decode after LONG fill |
| V23 | P1 | PROCESS_RX_BUFFER is only accepted when the buffered byte count equals the announced length (a partial / stale buffer whose CRC happens to match is not decoded). | SRC comm_can.c:1722-1737 | **B-V4** (no length tracking, VC:643-667) `KNOWN_ISSUE` |
| V24 | P1 | A FW version reply longer than the driver's 64-byte buffer (hardware + firmware name lengths > 35, e.g. 70 bytes) is still decoded, and fill frames with offsets beyond the buffer never write outside it. | SRC (reply length 27+n+m, rx buffer 512) | **B-V5** (no bound check on FILL_RX_BUFFER, VC:630; LONG checks `ind` not `ind+len`, VC:638) `KNOWN_ISSUE_UB` (first assertion: state COMPATIBLE after the long reply) |
| V25 | P2 | FILL frames out of sequence (offset not continuing) are dropped. | SRC comm_can.c:1652-1660 | driver copies anywhere: `characterization:` + NEEDS REVIEW (only reachable with a broken sender; L4) |
| V26 | P2 | Frames with a DLC shorter than the packet needs (PROCESS < 6, FILL < 2) do not decode stale data. | robustness (SRC does not validate DLC either) | `characterization:` + NEEDS REVIEW (L4) |

### G4 Decoding

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| V30 | P1 | FW_VERSION: the test-version number is the byte AFTER the pairing flag; a paired release (pairing 1, test 0) of version 5.3 is COMPATIBLE. | SRC commands.c:250-251 | **B-V6** (driver reads the pairing flag as test version, VC:529-530) `KNOWN_ISSUE` |
| V31 | P1 | Compatibility rule: release >= 5.3 compatible, < 5.3 incompatible; test builds >= 5.3.51. | FW (the driver's own rule, `FW_MIN_RELEASE`) | `characterization:` table (7.01, 5.3, 5.2, 5.3 test 51/50) built with the 7.01 layout and pairing 0 |
| V32 | P2 | Decoding the FW reply does not modify the received data (UUID is read, not overwritten). | code intent | B-V7 (`memcpy` arguments swapped, VC:526): not observable through the public interface (L5): recorded, no test |
| V33 | P1 | GET_VALUES_SELECTIVE: v_in = int16 BE / 10 (V) -> `voltage` command in mV; fault code 0 -> READY, != 0 -> ERROR and `errorflags` = the code. | SRC commands.c:421-447 | HD (24.0 V -> 24000, 12.5 V -> 12500; fault 3 -> 3) |
| V34 | P2 | bit 16 pid_pos int32 / 1e6 degrees feeds the position like POLL_ROTOR_POS. | SRC commands.c:445-447 | HD |
| V35 | P3 | Fields of unrequested bits are not skipped. | SRC: a VESC answers only the requested bits | not reachable with a real VESC (the driver's mask is fixed): recorded, no test (L5) |

### G5 State machine and timing (thread period 500 ms)

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| V40 | P1 | UNKNOWN: a FW version request every 500 ms; COMPATIBLE: a values request every 500 ms (only while the port is not waiting for free mailboxes). | FW | `characterization:` timing windows |
| V41 | P1 | READY / ERROR fall back to UNKNOWN when no frame arrived for more than 1000 ms. | FW (VESC default timeout 1000 ms, SRC) | timing window, `characterization:` |
| V42 | P1 | An unsolicited PONG does not stop the driver's operation (requests continue / it returns to the normal cycle). | intent (PONG is only an answer to a PING the driver never sends) | **B-V8** (state PONG has no exit, VC:676-678, 358-369) `KNOWN_ISSUE` |
| V43 | P2 | INCOMPATIBLE and COMPATIBLE never time out; any accepted frame counts as a sign of life. | FW | `characterization:` + NEEDS REVIEW |
| V44 | P1 | `motorReady()` iff READY. | FW | states |

### G6 Torque

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| V50 | P1 | `turn` sends SET_CURRENT_REL only when READY and started; otherwise nothing is sent. | FW | frames |
| V51 | P1 | While running, the last non-zero torque is repeated every 500 ms, so the VESC's 1000 ms timeout never triggers. | SRC timeout 1000 ms, only SET commands reset it | frame times HD (gap <= 1000 ms) |
| V52 | P1 | `stopMotor` sends ratio 0; after a stop and a new start no old torque is sent until `turn` is called again. | intent (stop = no torque) | **B-V9** (`lastTorque` not reset, VC:132-139, 380-383) `KNOWN_ISSUE` |
| V53 | P2 | power -32768 gives ratio -1.00003 (not clamped). | FW | `characterization:` + NEEDS REVIEW |

### G7 Encoder

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| V60 | P1 | POLL_ROTOR_POS reply: int32 BE / 1e5 = degrees; accepted from the configured VESC id (or any unit when the VESC id is 255); position in turns = degrees / 360 plus counted turns; `getPos` = 65535 * turns. | SRC comm_can.c:2023-2030 | HD |
| V61 | P1 | A wrap through 0/360 between two samples counts one turn (|delta| > 180). | FW (multi-turn rule is the driver's, SRC gives no multi-turn) | `characterization:` |
| V62 | P1 | The first sample after start-up does not count a turn, whatever its angle. | intent (no movement has happened) | **B-V10** (`prevPos360` starts at 0: a first sample > 180 deg gives -1 turn, VC:478-483) `KNOWN_ISSUE` |
| V63 | P1 | `getEncoderType()` with `useencoder` off and no external encoder does not crash. | robustness | **B-V11** (null dereference VC:152-157) skipped crash |
| V64 | P1 | `setPos` / `offset` set the offset (turns) and store it in flash (see G9). | FW | HD |

### G8 Commands (prefix `vesc`)

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| V70 | P1 | `offbcanid`, `vesccanid`, `canspd`, `errorflags`, `vescstate`, `voltage`, `encrate`, `pos`, `torque`, `forceposread`, `useencoder`, `offset` answer as documented; set commands have the effects of G1-G7. | FW | per command |
| V71 | P1 | `pos` reports turns * 1e9 also beyond 2.147 turns. | intent | **B-V12** (float -> int32 overflow, VC:303) `KNOWN_ISSUE_UB` |
| V72 | P2 | `canspd` narrows to uint8 / presets > 5 ignored; `errorflags`, `voltage`, `torque` before the first reply read uninitialised members. | FW | narrowing `characterization:` + NEEDS REVIEW; uninitialised reads: B-V13, nondeterministic, recorded only (L5) |

### G9 Persistent settings

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| V80 | P1 | Save/restore: CANID word (0x3E0 / 0x3E3) = `VESC id << 8 | OFFB id`; DATA word (0x3E1 / 0x3E4) bit 3 = `useencoder`; OFFSET word (0x3E2 / 0x3E5) = int16 fractional offset * 10000 (normalised to [-0.5, 0.5]); both instances at their own addresses. | FW (`eeprom_addresses.h:206-211`) | HD words |
| V81 | P2 | Only the fractional part of the offset is stored (multi-turn part lost on restart). | FW | `characterization:` + NEEDS REVIEW |

## Fake peer: `FakeVescPeer`

`Firmware/Tests/fakes/FakeVescPeer.h/.cpp`, fixture `support/VescFixture.h`, driver tests only. Same infrastructure as `FakeOdrivePeer` (fake HAL CAN, `canTxFrames`, `canReceive` with extended ids, thread scheduling with the clock rewind of D2 L9). Models a VESC per SRC 7.01 (cite SRC lines in the fidelity header):
1. Unit id (default 0x01; 255 only as the broadcast target), accepts frames addressed to its id or 255, strict decoding of the driver's frames (ext id, DLC per packet, upper id bits 0); violations recorded, teardown fails on unexpected ones (same pattern as D1/D2); an 8th undefined byte of the values request is recorded as `undocumented` (B-V2), not a violation.
2. Replies built exactly like SRC `comm_can_send_buffer`: <= 6 bytes PROCESS_SHORT_BUFFER, else FILL_RX_BUFFER frames + PROCESS_RX_BUFFER with CRC-16/XMODEM (own implementation from SRC `crc.c`, checked against 0x31C3), reply id `requester | packet << 8`, payload byte 0 = own id, flag 1.
3. FW_VERSION reply in the 7.01 layout with configurable major/minor/test/pairing/HW_NAME/FW_NAME; GET_VALUES_SELECTIVE reply for the requested mask with configurable v_in, fault code, pid_pos (bits 8, 15, 16; other bits per the SRC table or a violation if the fake cannot model them); POLL_ROTOR_POS reply (own id, degrees * 1e5); PONG for a PING; optional STATUS frames (off by default).
4. Timeout model: SET commands reset a 1000 ms timer; `timedOut()` observable.
5. Fault injection: silence, delayed reply, wrong CRC, missing/out-of-order FILL frame, short DLC, unsolicited PONG, long names (reply > 64 bytes), wrong unit id.
6. Records: decoded frame log, last current ratio, requests per COMM id.
7. Not modelled: motor control, current limits, encoder hardware, multi-VESC arbitration timing. Confidence: framing / buffer protocol / CRC / COMM layouts High (SRC 7.01), 5.x differences not modelled (Low for older versions), timing Medium.

## Coverage target

`VescCAN.cpp` line coverage >= 85 %, remainder classified.

## Open questions to the user

* **D3-Q1** (open question 15, items) answered by the supplied source except: the 5.3 FW_VERSION layout (was `pairing_done` already before the test version in 5.3?) and whether unsolicited STATUS frames should count as liveness. Recorded as open question 16.

## Lead decisions

* **L1** SRC 7.01 is the oracle for everything SPEC does not give (version note above).
* **L2** The compatibility rule (`FW_MIN_RELEASE`) is the driver's own: `characterization:`.
* **L3** STATUS frames carry the VESC's own id and normally do not pass the driver's filters (SRC); the driver does not use them.
* **L4** Robustness against malformed traffic that SRC itself does not check (DLC, sequence on the driver side) is `characterization:` + NEEDS REVIEW, not a suspected bug, except where it causes out-of-bounds writes (B-V5).
* **L5** Defects without an observable effect through the public interface (B-V7 swapped `memcpy`, B-V13 uninitialised reads, unrequested value bits) are recorded without a test.
* **L6** FW_VERSION reply length is 29 + n + m (correction by the fake builder, verified: 3 + (n+1) + 12 + 8 + (m+1) + 4); the default fake reply ("60", "") is 31 bytes, so overflow of the driver's 64-byte buffer needs n + m > 35.
* **L7** Fake strictness accepted as built (flags other than 0, packets other than 8/10/17/56, DLC 8 on the FW request are violations; the 8th byte of the values request is `undocumented`, B-V2). FILL_RX_BUFFER_LONG framing of normal replies (`setLongFill`) is a test-only mode to reach the LONG path (a real VESC uses it only above ~259 bytes).
