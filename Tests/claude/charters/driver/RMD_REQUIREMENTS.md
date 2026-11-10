# MyActuator RMD driver (`RmdMotorCAN`): requirements charter (phase P16, T0)

Status: **T0 approved by the owner 2026-11-10** ("approve and start T1"); drafted by the lead (Sonnet 5.5). T1 approved and committed (`70a7598b`); T2 (256 cases, reviewed, gated) waiting for the owner's approval.

Unit under test: `Firmware/FFBoard/UserExtensions/Src/RmdMotorCAN.cpp` (412 lines, `RS` below) and `Inc/RmdMotorCAN.h` (144 lines, `RH`), classes `RmdMotorCAN : MotorDriver, PersistentStorage, Encoder, CanHandler, CommandHandler, cpp_freertos::Thread` and the two instances `RmdMotorCAN1` / `RmdMotorCAN2`, behind the macro `RMDCAN`, class ids `CLSID_MOT_RMD1` / `CLSID_MOT_RMD2`, motor drivers 11 and 12 (`MotorDriver.cpp:50-53`; `Axis.cpp:68-70` id 11 in axis 1 only and `Axis.cpp:92-94` id 12 in axis 2 only).
Executable: `ffb_driver_tests` (owner's decision, question 28). Plan: `Tests/claude/plans/P14_P17_REMAINING_DRIVERS_PLAN.md`. Tests: `Firmware/Tests/claude/cases/driver/test_rmd_*.cpp`, fixture `support/RmdFixture.h`, peer fake `fakes/FakeRmdPeer.{h,cpp}`.

## Sources of expected values (oracles)

| Code | Source |
|---|---|
| SP | MyActuator "CAN Bus Motor Motion Protocol V4.3" (`Tests/shared/specs/CAN_BUS_Motor_Motion_Protocol_V4.3.pdf`, 110 pages, supplied by the owner, git-ignored, never committed; cited by section number, never copied beyond a short citation). Section 1.1.2: standard 11 bit frames, DLC 8, single motor command id `0x140 + ID` (ID 1..32), multi motor id `0x280`, reply id `0x240 + ID`; 1 Mbit/s. Commands the firmware uses: 2.17 `0x80` shutdown (reply = the sent frame), 2.19 `0xA1` torque closed loop (data[4..5] = iqControl int16, 0.01 A/LSB, other bytes 0x00; reply: data[1] temperature int8 1 degC, data[2..3] iq int16 0.01 A, data[4..5] speed int16 1 dps, data[6..7] output shaft angle int16 1 degree), 2.12 `0x92` multi turn angle (reply data[4..7] int32, 0.01 degree/LSB), 2.13 `0x94` single turn angle (reply data[6..7] 0..35999, 0.01 degree), 2.14 `0x9A` status 1 (reply data[1] motor temperature int8, data[2] MOS temperature, data[3] brake state, data[4..5] voltage uint16 0.1 V, data[6..7] error flags: 0x0002 stall, 0x0004 low voltage, 0x0008 over voltage, 0x0010 over current, 0x0040 power overrun, 0x0080 calibration write error, 0x0100 overspeed, 0x0800 component overtemperature, 0x1000 motor overtemperature, 0x2000 encoder calibration error, 0x4000 encoder data error), 2.33 `0xB5` model (reply data[1..7] seven ASCII characters), 2.34 `0xB6` active reply (data[1] command to reply cyclically: 0x60 0x61 0x62 0x92 0x9A 0x9C 0x9D 0x9E, data[2] enable 0 / 1, data[3..4] interval, unit 10 ms (the example value 1 is "20 ms"); the motor then sends the replies unrequested and does not answer the 0xB6 itself), 2.27 `0x77` brake release (2.19: "This command cannot open the brake directly ... use the 0x77 command to open the brake first"), section 4 `0x79` CAN id set / read (host side of the motor's id, not sent by the firmware). |
| HD | Hand derivation in a comment next to the assertion: frame bytes, flash words, mA / 0.01 degree conversions (`turn`: `torque = (int16_t)((float) power / 32767.0f * maxTorque)`, truncation toward zero), bit positions of the error flags. |
| IF | The `MotorDriver` / `Encoder` interface: `emergencyStop(false)` = `stopMotor()`, `emergencyStop(true)` = `startMotor()`, `getEncoder()` returns the driver, absolute encoder, `hasIntegratedEncoder()` true. |
| FW | The firmware's own behaviour where nothing above decides it. Test name starts with `characterization:` and the case gets a NEEDS REVIEW entry. |
| (none) | Not in the supplied document: the thread timing constants (100 ms start delay, 20 ms, 150 ms, 1000 ms cycle, 10 ms position refresh), `maxTorque` default 1000 and its 0x7fff limit, the flash word layout, the host side meaning of `requestpos`, the reply latency of a real motor, brake behaviour of a given motor model. |

Rules inherited from `CLAUDE.md` and the D1-D3 / P13-P15 charters: fixture per test, simulated time only, never copy a library value into an assertion without `characterization:`, a suspected defect is a `KNOWN_ISSUE("...")` test of the INTENDED behaviour whose first failing assertion is the defect, `KNOWN_ISSUE_UB` for undefined behaviour (skipped in sanitizer builds), no test whose outcome depends on stale memory, tests go through public interfaces (`turn`, `startMotor`, `stopMotor`, `getPos`, `setPos`, `motorReady`, the commands, `canRxPendCallback`, `saveFlash` / `restoreFlash`, the frames at the peer fake, the fake CAN filter table and port user count, the fake flash).

## Facts every test author needs (read from the source by the lead)

* **Construction** (`RS:42-59`): `CommandHandler("rmd", CLSID_MOT_RMD1, instance)`, thread "RMD" (256 words, priority 25), `motorId = instance`, `nodeId` 1 for instance 0 and 2 for instance 1 (defaults), `restoreFlash()`, `setCanFilter()`, the port speed preset is raised to 3 (250 kbit/s; "Default 1M") when lower, `setSilentMode(false)`, `registerCommands()`, `takePort()`, `Start()`. Commands: `canid` (GET|SET), `errors` (GET), `maxtorque` (GET|SET), `curr`, `temp`, `vbus` (GET), `requestpos` (GET|SET, the member `activerequests`), `model` (GET|STR_ONLY). The port is `canport` (`CANPort* port = &canport`).
* **Destruction** (`RS:61-64`): `removeCanFilter(filterId)`, `stopMotor()` (A1 with 0, then 0x80). It never calls `port->freePort()` although the constructor calls `takePort()`; `ODriveCAN` and `VescCAN` do call `freePort()`.
* **CAN filter** (`RS:123-136`): `nodeId = min(nodeId, 32)`, filter id `0x240 + nodeId`, mask `0x3FF`, receive buffer (FIFO) 0 for instance 0 and 1 for instance 1; a previous filter is removed first.
* **Frames sent** (`sendMsg`, `RS:177-185`): id `nodeId + 0x140`, DLC 8 (the `len` parameter), 8 data bytes. `sendCmd(c)` = `{c, 0 x 7}`; `setTorque(t)` = `{0xA1, 0, 0, 0, t lo, t hi, 0, 0}`; `requestConstantReports(c, en, interval)` = `{0xB6, c, en & 1, interval lo, interval hi, 0, 0, 0}`; `updateStatus()` = 0x9A; `getPos` in request mode sends 0x92 when the last angle update is older than `angleUpdateMs` (10 ms); `startMotor()`: only when `lastErrors.asInt == 0`: `active = true`, `setTorque(0)`; `stopMotor()`: `active = false`, `setTorque(0)`, `sendCmd(0x80)`; `turn(p)`: nothing unless `active && available`, else `setTorque((int16_t)((float) p / (float) 0x7fff * maxTorque))`.
* **Thread** (`RS:98-121`, simulated by the fixture): `Delay(100)`, then forever: `nextAvailable = false`; when `!available`: send 0xB5, `Delay(20)`; when `activerequests`: send 0x9A; else when `!available || requestConstantReportEnable`: send `{0xB6, 0x92, 1, 0, 0, 0, 0, 0}` (enable cyclic position every 10 ms), `requestConstantReportEnable = false`, `Delay(150)`; then `Delay(1000)`; `available = nextAvailable`. In the passive mode (the default, `activerequests` false) the status 1 (0x9A, the only source of `lastErrors`) is never requested.
* **Receive** (`canRxPendCallback`, `RS:240-303`): frames with any id other than `0x240 + nodeId` are ignored (also the filter does that); any frame for the motor sets `nextAvailable`. 0xA1: `curTemp = data[1]` (uint8), `curCurrent = int16(data[2..3])`, speed int16, angle int16; in request mode and when `ustime - lastTorqueStatusUpdate_us != 0`: `interpolatedPos = speed / (1000000.0 * (ustime - lastTorqueStatusUpdate_us))`, `lastPos = (lastPos + (angle + interpolatedPos)) / 2`. 0x92: `lastAngleUpdate = tick`, `lastPos = int32(data[4..7]) * 0.01` (degrees). 0x94: `lastPos = (lastPos / 360) + (int16(data[6..7]) * 0.01)`. 0x9A: `curTemp = data[1]`, `curVoltage = uint16(data[4..5])`, `lastErrors.asInt = uint16(data[6..7])`, then `errorCb` (any error: `stopMotor()`). 0xB5: `modelName[0..6] = data[1..7]`.
* **Position** (`RS:161-175`): `getCpr() = 36000`, `getPos() = (int32)(lastPos * 100) - posOffset` (float arithmetic), `setPos(p)`: `posOffset = (int32)(lastPos * 100) - p`.
* **Flash** (`RS:67-96`): word 1 (`ADR_RMD1_DATA1` / `ADR_RMD2_DATA1`): bits 0-4 = `nodeId - 1` (restore: `(w & 0x1F) + 1`), bit 6 = `activerequests` (bit 5 and bits 7-15 unused), word 2 (`..._TORQUE`): `maxTorque & 0x7FFF`, word 3 (`..._OFFSET`): `posOffset % 36000` written as `uint16_t`, restored as `int16_t`. Each is restored only when `Flash_Read` finds the address.
* **Host versus target.** Single precision float for `turn` (the host x86-64 and the Cortex-M4F FPU give the same IEEE single results for the used operations; the test derives the expected integers by hand), `pos * 0.01` is a double product stored to float (software double on the target, same values), `lastPos * 100` is float, float-to-`int16_t` / `int32_t` conversion of an out-of-range value is undefined (B-RMD-3), `ErrorStatus` is a GCC bit field in a union (little endian both). The thread uses the RTOS fake (`HostRtos`, resumable thread like `ODriveCAN`).

## Requirements

### R1: identity, chooser and instances
| ID | Requirement | Oracle |
|---|---|---|
| R1.1 | `RmdMotorCAN1::info` name "RMD MyActuator (1)" and `RmdMotorCAN2::info` name "RMD MyActuator (2)" with the class ids of `ClassIDs.h`; `getHelpstring` n/a; `hasIntegratedEncoder()` true; `getEncoder()` returns the driver; `getEncoderType()` absolute; `getCpr()` 36000 (the SP unit 0.01 degree: 360.00 degrees = 36000) | FW (identity facts, plain) + IF + SP |
| R1.2 | `MotorDriver::all_drivers` contains ids 11 and 12 once each; the Axis 1 list id 11 only, the Axis 2 list id 12 only; `Create` builds a working object | FW, plain |
| R1.3 | `isCreatable` true / false / true over create and destroy of each instance, instance 2 stays creatable while instance 1 exists, a second chooser creation of the same instance returns nullptr | FW, plain |
| R1.4 | Defaults before any flash: instance 1 `canid` 1, instance 2 `canid` 2 (SP: ID 1..32), `maxtorque` 1000, `requestpos` 0, `errors` 0, `curr` / `temp` / `vbus` 0, model empty, `motorReady()` false | HD, SP for the id range |

### R2: construction
| ID | Requirement | Oracle |
|---|---|---|
| R2.1 | A CAN receive filter with id `0x240 + nodeId`, mask 0x3FF and the buffer (FIFO) 0 (instance 1) / 1 (instance 2) is registered; the filter id is released at destruction; changing the id replaces it | SP (reply id) + FW |
| R2.2 | The port speed preset is raised to 3 when it was lower, left alone when it was 3 or higher; silent mode is off; the port has one more user while the driver exists | FW, `characterization:` for the preset 3 (SP: 1 Mbit/s, the code comment says "Minimum 250k") |
| R2.3 | The eight commands are registered with their flags | FW, plain |
| R2.4 | Restore from flash at construction (R9) before the filter is built: a stored id gives the matching filter | FW, plain |
| R2.5 | The thread "RMD" exists and has not sent anything before its 100 ms start delay | FW, `characterization:` (constant) |

### R3: frames sent (host to motor)
| ID | Requirement | Oracle |
|---|---|---|
| R3.1 | Every frame is a standard id `0x140 + nodeId`, DLC 8, 8 data bytes (SP 1.1.2); the unused bytes are 0x00 (SP data field tables) | SP |
| R3.2 | `setTorque(t)` sends `{0xA1, 0, 0, 0, t lo, t hi, 0, 0}` for t = 0, 1, 100, -100, 32767, -32768 (little endian two's complement) | SP (2.19 and its examples 100 -> 0x64 0x00, -100 -> 0x9C 0xFF) |
| R3.3 | `turn(p)` sends `setTorque(trunc(p / 32767.0 * maxTorque))` (single precision) once `active` and `available`: p = 0, 32767, -32767, 16384, 1, -1 with `maxTorque` 1000: 0, 1000, -1000, 500, 0, 0; `turn` sends nothing while inactive or unavailable | HD + FW |
| R3.4 | `startMotor()` sends one `{0xA1, ..., 0, 0}` (zero torque, "enable torque mode") and sets `active`; with a non-zero `lastErrors` it sends nothing and stays inactive | FW |
| R3.5 | `stopMotor()` sends `{0xA1, 0 ...}` then `{0x80, 0 x 7}` (SP 2.17 shutdown) and clears `active`; `turn` is silent afterwards | SP + FW |
| R3.6 | `emergencyStop(false)` behaves as `stopMotor`, `emergencyStop(true)` as `startMotor` | IF |
| R3.7 | `requestConstantReports(0x92, true, 0)` sends `{0xB6, 0x92, 1, 0, 0, 0, 0, 0}`, interval 1 gives data[3..4] = 0x01, 0x00 and interval 300 (0x012C) gives 0x2C, 0x01 (lo, hi) and `enable = false` data[2] = 0 (SP 2.34; 0x92 is in its list) | SP + HD |
| R3.8 | The 0x80 is the only shutdown and no 0x77 brake release is ever sent (SP 2.19: a braked motor will not move without it) | SP, `characterization:` (owner question 50) |

### R4: thread behaviour (cyclic requests)
| ID | Requirement | Oracle |
|---|---|---|
| R4.1 | Passive mode, silent motor: nothing before 100 ms; then 0xB5 at 100 ms, `{0xB6, 0x92, 1, 0, 0, 0, 0, 0}` at 120 ms and again every 1170 ms (`20 + 150 + 1000`) while the motor does not answer | FW, `characterization:` (constants), plain for the frame contents |
| R4.2 | A motor that answers every frame (reply 0xB5 and the cyclic 0x92 reports) makes `motorReady()` true at the end of the first cycle; the 0xB5 is not repeated while the motor stays available | FW + SP |
| R4.3 | A motor that falls silent: `available` is cleared at the end of the next cycle, `motorReady()` false, `turn` silent, 0xB5 probes resume | FW |
| R4.4 | Active request mode (`requestpos` 1): 0x9A is requested in every cycle, no 0xB6 enable is sent; switching TO the request mode sends `{0xB6, 0x92, 0, ...}` once (disable cyclic reports); switching back to the passive mode sends nothing at once, the next cycle sends the enable (lead amendment, checked against `RS:305-320`) | FW + SP |
| R4.5 | Setting the same `requestpos` value twice sends nothing | FW, plain |
| R4.6 | The destructor stops the cyclic requests (the thread is deleted: no frame after destruction) | FW, plain |

### R5: receive path (motor to host)
| ID | Requirement | Oracle |
|---|---|---|
| R5.1 | Only frames with id `0x240 + nodeId` are used; frames of the other node, id 0x241 for node 2 etc., do not change any value and do not set `available` | SP + FW |
| R5.2 | 0xA1 reply: `temp`, `curr` (int16, 0.01 A, negative values), speed, angle parsed per SP 2.19 (examples: temp 0x32 = 50, iq 0x0064 = 100, 0xFF9C = -100) | SP |
| R5.3 | 0x9A reply: temperature, voltage (0x01E5 = 485 = 48.5 V in 0.1 V), error flags (0x0004 = low voltage) per SP 2.14, stored and returned by `temp` / `vbus` / `errors` | SP |
| R5.4 | The temperature is a signed byte (SP 2.14 / 2.19: int8_t): a reply of -10 degC (0xF6) reads as -10 | SP, **`KNOWN_ISSUE` B-RMD-7** (`curTemp` is `uint8_t`: -10 reads 246) |
| R5.5 | 0xB5 reply: 7 ASCII characters (example 0x58 0x38 0x53 0x32 0x56 0x31 0x30 = "X8S2V10"); the `model` command returns them and the string stays terminated | SP 2.33 |
| R5.6 | Every reply of the motor sets `available` for the cycle; a reply with an unknown command byte (0x60, 0x9C) only sets `available` and changes nothing else | FW |
| R5.7 | The error flags mapped by the public `ErrorStatus` bit field match the SP table (0x0002 `stall` ... 0x2000 `calibrationErr`): `stall` bit 1, `undervoltage` 2, `overvoltage` 3, `overcurrent` 4, then `overpower` for 0x0040 (bit 6), `calibrationWriteErr` 0x0080 (bit 7), `overspeed` 0x0100 (bit 8), the overtemperature flag for 0x0800 / 0x1000 (bits 11 / 12), `calibrationErr` 0x2000 (bit 13) | SP, **`KNOWN_ISSUE` B-RMD-4** (the bit field has `overpower` at bit 5, `calibrationWriteErr` at bit 6 and `overspeed` at bit 7, each one bit below the SP value, `overtemp` at bit 8 and `calibrationErr` at bit 9 where SP has bits 11 / 12 and 13; the first four flags agree). Only `asInt` is used by the firmware: no behavioural effect |

### R6: position
| ID | Requirement | Oracle |
|---|---|---|
| R6.1 | 0x92 reply (SP 2.12: 0x00008CA0 = 36000 = 360 degrees): `getPos()` = angle in 0.01 degree: 36000, negative values, 0 and a multi turn value 1 234 567 (HD: `int32 * 0.01 * 100` in float, within one count) | SP + HD |
| R6.2 | `setPos(p)` makes `getPos()` equal p (within one count of the float conversion) and later 0x92 replies are relative to it; `setPos` before any report | FW, plain |
| R6.3 | Request mode: `getPos()` sends 0x92 when the last 0x92 reply is older than 10 ms (boundary 10 / 11 ms), not otherwise; passive mode never sends it | FW, `characterization:` (constant) |
| R6.4 | 0xA1 reply in the passive mode does not change the position | FW, plain |
| R6.5 | 0xA1 reply in the request mode: `lastPos = (lastPos + angle + interpolation) / 2` with the interpolation `speed (dps) * dt (s)` degrees: speed 500 dps, 10 ms after the previous reply, previous position 0, angle 0: position 2.5 degrees = 250 counts | HD (the averaging is the firmware's filter, the dimension of the interpolation term is the defect), **`KNOWN_ISSUE` B-RMD-6** (`speed / (1e6 * dt_us)` is 500 / (1e6 * 10000) = 5e-11 degrees instead of 5) |
| R6.6 | 0x94 reply: `characterization:` the formula `(lastPos / 360) + angle * 0.01` mixes revolutions and degrees (lastPos 720 degrees then single turn 100 degrees gives 102.0 instead of a position near 820); the driver never requests 0x94 | FW, `characterization:` (owner question 50), no test of intent |
| R6.7 | Float resolution: positions above 2^24 counts (about 466 turns) lose counts | FW, informational (NEEDS REVIEW only) |

### R7: status, errors, safety
| ID | Requirement | Oracle |
|---|---|---|
| R7.1 | Any non-zero error flags in a 0x9A reply stop the motor: `stopMotor` frames (A1 0, 0x80) are sent, `active` is false, `errors` returns the flags, `motorReady()` is false | FW |
| R7.2 | While errors are set `startMotor()` is refused; after a 0x9A reply with 0 flags it works again | FW |
| R7.3 | A 0x9A reply with 0 flags sends nothing | FW, plain |
| R7.4 | In the passive mode (default) no status is ever requested, so errors of the motor are never seen: `characterization:` (owner question 50) | FW, `characterization:` |
| R7.5 | `motorReady()` = no errors and `available` | FW + IF |

### R8: commands
| ID | Requirement | Oracle |
|---|---|---|
| R8.1 | `canid` get; `canid` set 1..32 changes the frame ids (`0x140 + id`), the filter (`0x240 + id`) and the reply | SP (1..32) |
| R8.2 | `canid` set 0 is not accepted (SP: ID 1..32; 0x140 is not a motor command id) and 256 is not truncated to 0 | SP, **`KNOWN_ISSUE` B-RMD-2** (`nodeId = (uint8_t) cmd.val` is taken unchecked: 0 accepted, 256 becomes 0) |
| R8.3 | `canid` set 33..255 is clamped to 32 by `setCanFilter` and answered OK | FW, `characterization:` |
| R8.4 | `maxtorque` get / set: values 1, 1000, 32767 are stored and used by `turn` | FW |
| R8.5 | `maxtorque` above 0x7fff is not accepted (the header: "max 0x7fff"; the flash word keeps 15 bits) | FW intent, **`KNOWN_ISSUE` B-RMD-3** (accepted: 40000 is stored; `turn` then converts a float of up to 40000 to `int16_t`; the flash word masks it to 7232) |
| R8.6 | `turn(0x7fff)` with `maxtorque` 40000: no undefined conversion | FW, **`KNOWN_ISSUE_UB` B-RMD-3** (float-to-int16 out of range) |
| R8.7 | `curr`, `temp`, `vbus`, `errors` return the stored values; `model` returns the string; `requestpos` get / set | FW, plain |
| R8.8 | SET on a GET-only command and an unknown id: `ERR` / `NOT_FOUND` | FW, plain |

### R9: persistence
| ID | Requirement | Oracle |
|---|---|---|
| R9.1 | `saveFlash()` writes: word 1 = `(nodeId - 1) & 0x1F | activerequests << 6` (instance 1: `ADR_RMD1_DATA1`, instance 2: `ADR_RMD2_DATA1`), word 2 = `maxTorque & 0x7FFF`, word 3 = the position offset reduced modulo 36000 as `uint16_t`; for id 1 / 32, request mode on / off, `maxtorque` 1 / 1000 / 32767 | HD |
| R9.2 | `restoreFlash()` and the constructor restore id, request mode, torque and offset; a missing address leaves the default; the word's unused bits 5 and 7-15 are ignored | HD + FW |
| R9.3 | A position offset above 32767 survives a save / restore round trip (SP unit: 36000 counts per turn, offsets up to 35999) | HD, **`KNOWN_ISSUE` B-RMD-5** (the offset is stored as `uint16_t` and restored as `int16_t`: 35000 comes back as -30536) |
| R9.4 | Negative offsets -1 and -1234 round trip (HD); -35999 does not: it is below -32768, stored `uint16_t` 29537, restored `int16_t` +29537 | HD, -35999 is `KNOWN_ISSUE` B-RMD-5 (lead amendment) |
| R9.5 | The two instances use their own addresses | FW, plain |

### R10: life cycle
| ID | Requirement | Oracle |
|---|---|---|
| R10.1 | Destruction removes the filter, sends the stop frames (A1 0, 0x80) and clears the in-use flag | FW, plain |
| R10.2 | Destruction gives the port user back: `canport.getPortUsers()` is the value before construction | FW intent (`ODriveCAN` / `VescCAN` do it), **`KNOWN_ISSUE` B-RMD-1** (`freePort()` is never called: the CAN port stays started with one user per created driver) |
| R10.3 | Create / destroy / create twice leaves one consistent set-up (filters, in-use flag) | FW, plain |
| R10.4 | Two instances at the same time use filter buffers 0 and 1 and their own ids; the frames of one do not reach the other | FW + SP |

### R11: defect candidates (from reading the source and the SP)

1. B-RMD-1: no `freePort()` in the destructor (R10.2), `KNOWN_ISSUE`.
2. B-RMD-2: `canid set` is unchecked: 0 and out of range values accepted, 256 truncates to 0 (R8.2), `KNOWN_ISSUE`.
3. B-RMD-3: `maxtorque` above 0x7fff accepted: the float-to-int16 conversion in `turn` is undefined and the flash word does not keep the value (R8.5, R8.6), `KNOWN_ISSUE` and `KNOWN_ISSUE_UB`.
4. B-RMD-4: the `ErrorStatus` bit field disagrees with the SP error table from `overpower` on (R5.7), `KNOWN_ISSUE` (no behavioural effect, only `asInt` is used).
5. B-RMD-5: position offset stored as `uint16_t`, restored as `int16_t` (R9.3), `KNOWN_ISSUE`.
6. B-RMD-6: the interpolation term of the 0xA1 handler has the wrong dimension (R6.5), `KNOWN_ISSUE`; also the condition `ustime - lastTorqueStatusUpdate_us` is a truthiness test of a difference (informational).
7. B-RMD-7: `curTemp` is `uint8_t`, the SP says int8_t (R5.4), `KNOWN_ISSUE`.
8. B-RMD-8 (found in T2, lead amendment): `maxtorque` discards the status of `handleGetSet` (`RS:350-352`), so a command of an unsupported type answers OK where every other RMD command answers ERR (R8.8, `KNOWN_ISSUE`; owner question 51; theoretical because the parser's flag check normally rejects the type first).
9. Informational / `characterization:` (owner question 50): the passive default never requests status, so errors and over temperature are never seen (R7.4); no 0x77 brake release (R3.8); the 0x94 handler mixes units and is unreachable (R6.6); `canid set` only changes the host side id, the motor id is changed with 0x79 (SP section 4) and the firmware never sends it; `~RmdMotorCAN` stops the motor while its thread may still run; `getPos` is float based (R6.7).

## Fake: MyActuator RMD motor on the fake CAN (T1, owner approved question 31)

* **`FakeRmdPeer`** (`fakes/FakeRmdPeer.{h,cpp}`, driver executable only; the pattern of `FakeOdrivePeer`: it reads the frames the driver transmitted from `HostTarget::canTxFrames()` that the bus acknowledged, and injects replies with `HostTarget` CAN receive helpers): one motor per node id (default node 1 and 2), standard ids only.
  * Decoding per SP: frame for the node `0x140 + id` (id 0x280 broadcast is answered by every present motor; other ids are counted foreign), commands 0x80 / 0x81 / 0x77 / 0x78 (reply = the sent frame), 0xA1 (reply per SP 2.19 from the configured temperature, iq, speed and angle; the sent iq is recorded and sets the iq of the next reply when `followTorque` is on), 0x92, 0x94, 0x9A, 0xB5, 0xB6 (no reply; cyclic replies of the selected command at `(interval + 1) * 10` ms until disabled; several commands alternate).
  * State (configurable): temperature (int8), MOS temperature, brake state, voltage (0.1 V), error flags, model (7 characters), multi turn angle (int32, 0.01 degree), single turn angle (0..35999), shaft angle (int16 degree), speed (dps), iq (0.01 A); running flag (cleared by 0x80, set by 0xA1).
  * Strictness (violations, `expectViolation()`, `unexpectedViolations()`): an extended id, an RTR frame, a DLC other than 8, a command byte the firmware is not expected to send (anything other than 0x80, 0xA1, 0x92, 0x9A, 0xB5, 0xB6; others only if the test declares them), a non-zero byte in a field SP marks NULL, a 0xB6 command byte outside the SP list, an enable byte other than 0 / 1. A frame with a violation is logged but not answered.
  * Fault injection: silent, delayed replies, replies with the id of another node, wrong command byte, short DLC, raw frame injection, unsolicited replies, torque feedback on / off.
  * Records: every frame received (id, data, time), counts per command byte, the last iq, the cyclic reply schedule, foreign frames.
  * Header comment: models / source (SP sections above) / does not model (motor dynamics, PID modes 0xA2-0xA9, brake hardware, multi motor 0x280 semantics beyond a broadcast reply, motion mode 0x400 + ID, RS485 CRC) / confidence (High for the frame layouts of the used commands, Medium for the cyclic reply timing).
* **`RmdFixture : TargetFixture`** (`support/RmdFixture.h`): clears the fake flash, two peers (node 1, node 2), the thread "RMD" of each instance made resumable and scheduled by the fixture itself, the way `OdriveFixture` does it (the `Run` loop has several `Delay` calls: 100 ms start, then 20, 150 and 1000 ms inside the cycle; the helper activates the thread when the clock has reached its wake time; decided in T1: either the `OdriveFixture` approach with explicit wake times or `HostRtos` delay budgets as in `TmcFixture`), FIFO 1 handling as in `OdriveFixture` (the F407 has the CAN1_RX1 interrupt line disabled in the fake HAL; the fixture enables it as a documented modelling choice because instance 2 listens on buffer 1), `makeRmd1()` / `makeRmd2()`, `runFor(ms)`, `reachAvailable(driver)`, `txFrames(cmd)`, `clearTxFrames()`, fail on undeclared peer violations.
* **Build wiring** (T1): `RMDCAN` on in `platform/target_f407_drivers/target_constants.h` only (not with clang if the header needs a GCC extension: check at T1, as for P15), `RmdMotorCAN.cpp` in `DRIVER_FIRMWARE_SOURCES`, `test_driver_setup.cpp` expects `RMDCAN`, the chooser side effect (ids 11 and 12 in the driver and Axis lists) checked against all existing driver tests, the other executables keep identical compile commands, the `ADR_RMD*` addresses exist in `eeprom_addresses.c` (the flash tests fake already covers the table).

## Fixture and test layout (T1 / T2)

* Files and writer groups: `test_rmd_setup.cpp` (R1, R2, R10, T1 skeleton), `test_rmd_frames.cpp` (R3, R4), `test_rmd_receive.cpp` (R5, R6, R7), `test_rmd_commands.cpp` (R8, R9).
* Counts to aim at (writers): setup about 25, frames about 35, receive about 40, commands about 35; not a target to pad.

## Target differences to flag

Single precision float arithmetic in `turn` and `getPos`, double `* 0.01` in the receive path, float to int conversion limits (B-RMD-3), the GCC bit field union, `micros()` wrap (`ustime - last` as `uint32_t`), thread stack 256 words (not modelled).

## Out of scope for P16

The real motor, the RS485 variant of the protocol, the other control modes (speed, position, motion mode 0x400), PID and acceleration commands, the multi motor id 0x280, motor dynamics.

## Decisions (lead, Sonnet 5.5) and owner questions

* L1: the SP document is the oracle for ids, frame layouts, units and the error table; where it is silent (thread timing, defaults, flash layout, `requestpos` semantics) the test is `characterization:`.
* L2: B-RMD-1..7 as listed (`KNOWN_ISSUE`, B-RMD-3 also `KNOWN_ISSUE_UB`); everything under item 8 is `characterization:` plus NEEDS REVIEW.
* L3: `FakeRmdPeer` follows `FakeOdrivePeer` (CAN frames from the fake bus, standard ids), `process()` driven by the fixture.
* L4: the thread is driven by the fixture with explicit wake times (decided in T1 with a check of the `Run` delays).
* Owner questions (numbers 44 and up): (44) is the missing `freePort()` in the destructor a defect (B-RMD-1)? (45) should `canid set` reject 0 and values above 32 (B-RMD-2)? (46) should `maxtorque` be limited to 0x7fff (B-RMD-3)? (47) is the offset stored as `uint16_t` but restored as `int16_t` a defect (B-RMD-5)? (48) should the `ErrorStatus` bit field follow the SP table (B-RMD-4) and `curTemp` be signed (B-RMD-7)? (49) is the dimension of the 0xA1 interpolation term a defect (B-RMD-6)? (50) design points: no status polling in the passive default, no 0x77 brake release, the 0x94 handler, `canid` host-side only: intended?
