# SimpleMotion driver (`MotorSimplemotion`): requirements charter (phase P15, T0)

Status: **T0 approved by the owner 2026-11-10; T1 committed (`ff86dd7e`); T2 written 2026-11-10, waiting for approval.** Amendments made in T2 (lead): R1.2, R6.2, R6.1, R12 items 10-11 (marked "T2 amendment").

Unit under test: `Firmware/FFBoard/UserExtensions/Src/MotorSimplemotion.cpp` (528 lines, `MS` below) and `Inc/MotorSimplemotion.h` (288 lines, `MH`), classes `MotorSimplemotion : MotorDriver, Encoder, CommandHandler, UARTDevice` and the two instances `MotorSimplemotion1` / `MotorSimplemotion2`, behind the macro `SIMPLEMOTION`, class ids `CLSID_MOT_SM1` 0x89 / `CLSID_MOT_SM2` 0x8A, motor drivers 9 and 10 in `MotorDriver.cpp:46-49`, `Axis.cpp`.
Executable: `ffb_driver_tests` (owner's decision, question 28). Plan: `Tests/claude/plans/P14_P17_REMAINING_DRIVERS_PLAN.md`. Tests: `Firmware/Tests/claude/cases/driver/test_sm_*.cpp`, fixture `support/SmFixture.h`, peer fake `fakes/FakeSmDevice.{h,cpp}`.

## Sources of expected values (oracles)

| Code | Source |
|---|---|
| SM | SimpleMotion V2 reference library and documentation (`Tests/shared/specs/SimpleMotionV2-master/`, never committed, never copied into a test beyond a short citation): `sm485.h` (frame ids: `SMCMD_INSTANT_CMD` = `SMCMD_ID(4, N_PARAMS)` = 0x24, return 0x25, `SMCMD_FAST_UPDATE_CYCLE` 0x10, return 0x11, `SM485_CRCINIT` 0), `simplemotion.c` (`smSendSMCMD`: id, payload size, address, payload, CRC16 high byte then low byte; `smFastUpdateCycle`: 7 bytes `{0x10, addr, write1, write2 (native 16 bit, little endian on both host and target), crc8}`, reply 6 bytes, `calcCRC8Buf(.., 0x52)`), `sm_consts.c` (`table_crc8` is the table of polynomial 0x07, checked by script; `table_crc16_hi/lo` give the byte-swapped CRC-16/ARC: check string "123456789" gives 0x3DBB), `simplemotion_defs.h` (subpacket types `SM_SET_WRITE_ADDRESS` 2 (2 bytes), `SM_WRITE_VALUE_24B` 1 (3 bytes, 22 bit value), `SM_WRITE_VALUE_32B` 0 (4 bytes, 30 bit value), return types 32B / 24B / 16B / status; `SMP_RETURN_PARAM_ADDR` 9, `SMP_RETURN_PARAM_LEN` 10; parameter ids `SMP_DEVICE_TYPE` 6020, `SMP_STATUS` 553, `SMP_FAULTS` 552, `SMP_CUMULATIVE_STATUS` 13, `SMP_SYSTEM_CONTROL` 554 (2 = restart), `SMP_CONTROL_MODE` 559 (3 = torque), `SMP_FB1_DEVICE_SELECTION` 493 with the FBD values 0..10, `SMP_ENCODER_PPR` 565, `SMP_CONTROL_BITS1` 2533 (bit 0 enable), `SMP_ACTUAL_BUS_VOLTAGE` 900 (10 mV per count), `SMP_ACTUAL_TORQUE` 901 (560 counts per ampere); status bits `STAT_FAULTSTOP` bit 5, `STAT_INITIALIZED` bit 12; fast update format 0: write1 = setpoint, write2 unused, read1 = lowest 16 bits of the position feedback, read2 = first 16 bits of `SMP_STATUS`). |
| HD | Hand derivation (script or comment) following the SM algorithms above: frame bytes, CRC8 / CRC16 values, positions, mA / mV conversions. The CRC16 oracle in a test is an independent bitwise implementation (reflected polynomial 0xA001, init 0, result byte-swapped) written in the test file, never the firmware's table. |
| IF | The `MotorDriver` / `Encoder` interface (`MotorDriver.cpp`, `Encoder.h`): `emergencyStop(false)` = `stopMotor()`, `emergencyStop(true)` = `startMotor()`, `getEncoder()` returns the driver, absolute encoder type. |
| FW | The firmware's own behaviour where nothing above decides it. Test name starts with `characterization:` and the case gets a NEEDS REVIEW entry. |
| (none) | Not in the supplied documents: the RS485 transceiver direction handling (`gpMotor` high while transmitting), the timing constants (10 ms timeout, 150 ms settings interval, 100 ms position refresh), the cpr factors for Hall / Resolver / Serial (the code says "TODO check"), the default value of `SMP_RETURN_PARAM_LEN` of a real device, sign extension of negative returned values. |

Rules inherited from `CLAUDE.md` and the D1-D3 / P13 / P14 charters: fixture per test, simulated time only, never copy a library value into an assertion without `characterization:`, a suspected defect is a `KNOWN_ISSUE("...")` test of the INTENDED behaviour whose first failing assertion is the defect, no test whose outcome depends on stale memory or the stack, tests go through public interfaces (`turn`, `startMotor`, `getPos`, `read1Parameter`, `set1Parameter`, commands, the UART bytes at the peer fake, the write-enable pin, `motorReady`).

## Facts every test author needs (read from the source by the lead)

* **Construction** (`MS:30-52`): `CommandHandler("sm2", CLSID_MOT_SM1, instance)`, `UARTDevice(motor_uart)` (reserves the port), `address(address + 1)` (**the member initialises itself: B-SM-1**, `instance` is not used), the CRC tables are generated once (`makeCrcTable` CRC8 poly 0x07, CRC16 poly 0x8005 reflected), the port is reconfigured to 460800 baud, 8 bit, 1 stop, no parity, TX/RX, oversampling 8, `writeEnablePin.reset()`, `registerCommands()`. The commands: `crcerr`, `uarterr`, `voltage`, `torque`, `state`, `restart` (all GET), `reg` (GETADR | SETADR | DEBUG), `devtype` (GET).
* **Frames.** Fast update (`MS:62-99`): `{0x10, address, val1 LE, val2 LE, crc8}` (packed struct, 7 bytes, `calculateCrc8(tableCRC8, buf, 6, 0x52)`), sent with `transmit_IT`, then `registerInterrupt()`; `turn(p)` sends `(uint16_t) p, 0`. Instant command (`sendCommand`, `MS:300-324`): `{0x24, len, address, payload (len bytes), crc16 high, crc16 low}` (`calculateCrc16_8_rev(tableCRC16, buf, len + 3, 0)`), `len + 5` bytes. Subpackets (`queueCommand`, `MS:331-339`): type in the top 2 bits (`param32b` 0 = 4 bytes, `param24b` 1 = 3, `setparamadr` 2 = 2, `status` 3 = 1), value masked to 30 bits, big endian. Read request (`readParameter`): `[setparamadr 10][param24b len]` only when a length is given, then `[setparamadr 9][param24b paramId]`; the reply value is the last reply subpacket. Write (`writeParameter`): `[setparamadr id][param32b value]`.
* **Receive** (`uartRcv`, `MS:353-438`, called per byte from the fake interrupt): bytes are appended up to 32 (`RXBUF_SIZE`) unless `getErrors()` is non-zero (then `uarterrors++`, `uartErrorOccured`); fast reply `0x11` with exactly 6 bytes while `waitingFastUpdate`: crc8 over 6 bytes must be 0 (else `crcerrors++`, buffer reset), then `updatePosition(val1)`, `updateStatus(val2)`; instant reply `0x25` with more than 5 bytes while `waitingReply`: waits until `len + 5` bytes, a different address resets the buffer, CRC16 over the whole frame must be 0 (else `crcerrors++`), then the subpackets are decoded into `replyvalues` (type from the top 2 bits, 6 bits of the first byte plus the following bytes, **no sign extension**), `waitingReply = false`.
* **Timing**: `readParameter` / `writeParameter` busy-wait `HAL_GetTick() - start < 10 ms && waitingReply` calling `refreshWatchdog()`; a timeout sets `uartErrorOccured`, `uarterrors++`, `waitingReply = false` and returns false. `sendFastUpdate` does nothing while `waitingReply`, calls `getSettings()` first when not initialized, clears `waitingFastUpdate` after 10 ms without reply. `getSettings` returns false when called less than 150 ms after the last send. `getPos` sends a fast update (`turn(lastTorque)`) when the last position is older than 100 ms.
* **`getSettings`** (`MS:148-224`) order and formats: read `devtype` 6020 (param32b), write `faults` 552 = 0, read `status` 553 (24b), if the status is 0 return false, read `FBR` 565 (24b), read `FBD` 493 (24b) giving `cpr = FBR * 4` for Serial / Hall / Resolver / ABN1 / ABN2, `* 16` / `* 64` / `* 256` for Sincos16 / 64 / 256, any other value: `ErrorHandler::addError(configError)`, `hardfault = true`, status false; read `ControlMode` 559 (24b, must be 3). `initialized = status`. (`st` of the status read is uninitialised when that read fails: B-SM-4.)
* **Position** (`updatePosition`, `MS:261-275`): `value += 0x7fff` (16 bit wrap), `diff = value - lastPosRep` (initial `lastPosRep` 0x7fff), `|diff| > 0x7fff` counts an overflow (diff < 0: +1, else -1), `position = value + 0xffff * overflows - 0x7fff` (**0xffff, not 0x10000: B-SM-2**), `getPos() = position - position_offset`, `setPos(p)` sets `position_offset = position - p`.
* **Status** (`updateStatus`): `hardfault = bit 5`, `initialized = false` when bit 12 is clear. `motorReady() = initialized && !hardfault`.
* **Motor control**: `startMotor` writes `CB1` (2533) = 1, `stopMotor` is `turn(0)` then `CB1` = 0 (both `param32b` writes), `restart()` clears `hardfault` and writes `systemcontrol` (554) = 2. `getVoltage()` = `counts * 10` mV returned as `int16_t` (**overflow above 32767 mV: B-SM-3**), `getTorque()` = `(int16_t) counts * 1000 / 560` mA, both 0 on a failed read.
* **Host versus target.** Plain `char` is unsigned on ARM (AAPCS) and signed on the x86-64 host: `uartRcv` decodes `(uint32_t) data[i + b]` from a `char*`, which sign-extends bytes of 0x80 and above on the host. `MotorSimplemotion.cpp` is therefore compiled with `-funsigned-char` in `ffb_driver_tests` (CMake source property, documented, the firmware is not edited), like the target. Little-endian on both (fast update words, packed struct).

## Requirements

### R1: identity, chooser and instances
| ID | Requirement | Oracle |
|---|---|---|
| R1.1 | `MotorSimplemotion1::info` name "Simplemotion 1" id 0x89, `MotorSimplemotion2::info` name "Simplemotion 2" id 0x8A; `getHelpstring` text; `hasIntegratedEncoder()` true; `getEncoder()` returns the driver itself; `getEncoderType()` absolute | FW (identity facts of `ClassIDs.h`, plain) + IF |
| R1.2 | With `SIMPLEMOTION` on, `MotorDriver::all_drivers` contains ids 9 and 10 once each and `Create` builds a working object; **T2 amendment:** the Axis 1 list contains id 9 only, the id 10 entry of `axis2_drivers` is commented out in `Axis.cpp:95-97` ("TODO likely does not work reliably with a single uart port and multiple devices"): `characterization:` | FW, plain |
| R1.3 | Each instance has its own in-use flag: `isCreatable` true / false / true over create and destroy, instance 2 stays creatable while instance 1 exists; a second creation of the same instance through the chooser returns nullptr | FW, plain |
| R1.4 | Instance 1 uses device address 1 and instance 2 address 2 (`instance + 1`) | HD from the constructor intent; **`KNOWN_ISSUE_UB` B-SM-1** (member initialised from itself; deterministic only when constructed in a poisoned buffer, see Fixture) |

### R2: construction
| ID | Requirement | Oracle |
|---|---|---|
| R2.1 | The port `motor_uart` is reserved by the driver and reconfigured to 460800 baud, 8N1, TX and RX, oversampling 8; the write-enable pin is low | FW (config values are the code's constants) + HD; port settings read from `huart3.Init` of the fake |
| R2.2 | The CRC tables are generated once: after any construction a fast update frame carries the CRC8 of the SM algorithm and an instant frame the CRC16 of the SM algorithm (R3.1, R5.1 prove the tables through the bytes on the wire) | SM |
| R2.3 | The eight commands are registered with their flags | FW, plain |

### R3: fast update (torque setpoint and position feedback)
| ID | Requirement | Oracle |
|---|---|---|
| R3.1 | `turn(p)` on an initialized driver sends exactly `{0x10, address, p LE, 0, 0, crc8}` (7 bytes) with the crc8 of the SM algorithm (`{10,01,0,0,0,0}` -> 0x46 for address 1), for p = 0, 1, 1000, 32767, -1, -32768; the transmit goes in one transfer | SM + HD |
| R3.2 | The write-enable pin is high while the bytes are transmitted and low after the transmit-complete callback | (none) FW, `characterization:` unless the peer fake can show it plain (it is the RS485 direction contract of the board); decided in T1 |
| R3.3 | A valid reply `{0x11, pos LE, status LE, crc8}` updates position and status (R6, R7) and releases the port; the next `turn` is sent | SM + HD |
| R3.4 | A reply with a wrong CRC8 increments `crcerr` (command), leaves position and status unchanged and releases the port | FW, plain (the CRC contract is SM) |
| R3.5 | No reply within 10 ms (simulated time): the next `turn` clears the wait and is sent; no position update | FW, `characterization:` on the 10 ms constant, plain on the recovery |
| R3.6 | While a parameter transfer waits for its reply, no fast update is sent; a fast update does not start before `getSettings` succeeded (not initialized: only the settings sequence is sent, the torque is not) | FW, plain |
| R3.7 | `getPos()` sends a fast update with the last torque when the last position is older than 100 ms and not otherwise | FW (constant: `characterization:`), the use of `lastTorque` plain |

### R4: initialization (`getSettings`)
| ID | Requirement | Oracle |
|---|---|---|
| R4.1 | The settings sequence is read `devtype` (6020, 32b return), write `faults` 552 = 0, read `status` 553, read `FBR` 565, read `FBD` 493, read `ControlMode` 559, with the exact request bytes (R5.1 layout) and success only when all replies arrive, the status is non-zero, FBD is supported and the control mode is 3 | SM + HD |
| R4.2 | `cpr` equals `FBR * 4` for ABN1 / ABN2, `* 16` / `* 64` / `* 256` for the three sincos types; Hall, Resolver and Serial use `* 4`: **`characterization:`** (the code says "TODO check"); `getCpr()` re-runs the settings while not initialized | SM for ABN / sincos if the documentation states it (the library defines `SMP_ENCODER_PPR` as pulses per revolution: quadrature counts are 4 per pulse, plain), `characterization:` otherwise |
| R4.3 | FBD none (0), 9 or 10: `configError` (`externalConfigurationError`, critical) is added to the error handler, `hardfault` is set, `initialized` stays false and `motorReady()` is false | FW + IF, plain |
| R4.4 | Control mode other than 3: not initialized; status 0: not initialized and no further requests are sent | FW (spec: torque mode = 3) |
| R4.5 | Two calls within 150 ms of the last send: the second returns false without traffic; after 150 ms it runs again | `characterization:` (constant) |
| R4.6 | A silent peer: every request times out, `uarterr` counts them, `initialized` stays false | FW, plain |

### R5: parameter protocol (`read1Parameter`, `set1Parameter`, `queueCommand`, receive path)
| ID | Requirement | Oracle |
|---|---|---|
| R5.1 | Request layouts: `read1Parameter(553)` (no length) sends `{0x24, 5, adr, 0x80, 0x09, 0x40, 0x02, 0x29, crc16 hi, crc16 lo}` (set-address 9: type 2 in the top bits, value 9; 24b write of 553 = 0x0229: type 1, `0x40 | (553 >> 16)`, 0x02, 0x29; hand-derive every other id and the CRC16 of the SM algorithm, appended high byte first); with a length (`param24b` = 1) the request starts with `0x80, 0x0A, 0x40, 0x00, 0x01` (set-address 10, 24b write of the value 1) and carries 10 payload bytes; `set1Parameter(id, v)`: `[set id][32b v]`, 4 + 2 bytes, negative values masked to 30 bits | SM + HD |
| R5.2 | The returned value is the value of the last reply subpacket for reads (index (i+1)*3 or (i+1)*2-1 as coded) and the reply of the write goes to the optional pointer; types 32B / 24B / 16B / status are decoded with the SM sizes (4 / 3 / 2 / 1 bytes) | SM + HD |
| R5.3 | A reply with a wrong CRC16 is dropped (`crcerr`++, call returns false); a reply from another address is ignored (call times out); a reply split into single bytes with gaps below 10 ms is assembled; a reply shorter than announced times out | SM + FW |
| R5.4 | A reply longer than the 32 byte receive buffer (announced length above 27) is an overrun: `uarterr`++, never a buffer overwrite (sanitizer clean) | FW, plain (memory safety) |
| R5.5 | A UART error flag at the receive callback (`getErrors() != 0`) counts a uart error and drops the byte | FW, plain |
| R5.6 | After a failed or timed-out transfer the port is released and the next request is sent normally | FW, plain |
| R5.7 | Negative values: a returned two's complement value is NOT sign-extended (a 22 bit -1 is read as 0x3FFFFF) | `characterization:` (the SM documents give no sign rule), owner question |
| R5.8 | `reg getat` / `reg setat` with an address above 0x3FFF corrupts the subpacket header (the high address bits overwrite the type bits) | `characterization:` of the limit (spec: 14 address bits in the subpacket) |

### R6: position tracking
| ID | Requirement | Oracle |
|---|---|---|
| R6.1 | (T2 amendment: the first report is read as a signed 16 bit value around `lastPosRep` 0x7fff: 0x8000 gives -32767, one count short of -32768, the same B-SM-2 loss; no test) First reply: position equals the reported 16 bit value, for reports 0, 1, 0x7fff, 0x8000 (the first sample defines the zero point `lastPosRep = 0x7fff`: position 0 for report 0) | HD (derive from `updatePosition`'s documented intent: the position follows the 16 bit feedback) |
| R6.2 | Forward motion across the 16 bit wrap (0xfff0 -> 0x0010) advances by 0x20 counts; backward across the wrap likewise; several wraps accumulate 65536 counts per wrap | SM (read1 = lowest 16 bits of the position feedback, so the counter has period 65536), **`KNOWN_ISSUE` B-SM-2** (0xffff per wrap: after one wrap the position is one count short, after n wraps n counts) |
| R6.2a | **T2 amendment:** the one-count loss of B-SM-2 does NOT show at the 16 bit counter wrap (0xfff0 -> 0x0010 is exact, +0x20): the shifted counter `value + 0x7fff` wraps at the reports 0x7fff -> 0x8000, and there `0xffff * overflows` loses one count (0x7ff0 -> 0x8010 gives +0x1f instead of +0x20, three revolutions give 196605 instead of 196608). The `KNOWN_ISSUE` cases use these steps; bounded plain tests (position within 3 counts of the ideal) guard the direction logic | HD from `updatePosition` |
| R6.3 | `setPos(p)` makes `getPos()` equal p and later motion is relative to it | FW, plain |
| R6.4 | The jump detection threshold: a step of exactly 0x7fff is not an overflow, 0x8000 is (positions of single steps near half range) | `characterization:` |

### R7: status
| ID | Requirement | Oracle |
|---|---|---|
| R7.1 | Status bit 5 (FAULTSTOP) sets `hardfault`, so `motorReady()` is false; clearing it with a later reply makes the driver ready again | SM (STAT_FAULTSTOP bit 5) + FW |
| R7.2 | Status bit 12 (INITIALIZED) clear sets `initialized = false`: the next `turn` runs the settings sequence again before sending torque | SM (STAT_INITIALIZED bit 12) |
| R7.3 | `state` returns the last status word | FW, plain |

### R8: motor control
| ID | Requirement | Oracle |
|---|---|---|
| R8.1 | `startMotor()` writes `SMP_CONTROL_BITS1` (2533) = 1 (enable bit) with the R5.1 bytes | SM + HD |
| R8.2 | `stopMotor()` sends a zero-torque fast update and then writes `SMP_CONTROL_BITS1` = 0 | SM + FW |
| R8.3 | `emergencyStop(false)` / `(true)` behave as stop / start; `restart()` clears `hardfault` and writes `SMP_SYSTEM_CONTROL` (554) = 2 (restart) with no reply wanted; `restart` clears `hardfault` even when the device is still faulted | IF + SM, `characterization:` for the second part |

### R9: telemetry
| ID | Requirement | Oracle |
|---|---|---|
| R9.1 | `voltage` / `getVoltage()` = counts of parameter 900 x 10 mV (4800 counts = 48000 mV) | SM (10 mV per count) |
| R9.2 | `getVoltage()` above 32767 mV: the result does not wrap negative | SM, **`KNOWN_ISSUE` B-SM-3** (`int16_t`: 48000 mV becomes -17536) |
| R9.3 | `torque` / `getTorque()` = counts of parameter 901 x 1000 / 560 mA (560 counts = 1000 mA, -560 counts = -1000 mA, integer division as coded) | SM (560 counts per ampere) + HD |
| R9.4 | A failed read returns 0 for both | FW, plain |

### R10: commands
| ID | Requirement | Oracle |
|---|---|---|
| R10.1 | `crcerr`, `uarterr`, `devtype`, `state` replies; `restart` runs R8.3; `reg getat` returns the parameter value, `reg setat` writes and returns the reply; unknown id: `NOT_FOUND` | FW, plain |

### R11: life cycle
| ID | Requirement | Oracle |
|---|---|---|
| R11.1 | Destruction frees the UART port; a new instance can reserve it; the CRC tables stay valid | FW, plain |
| R11.2 | Two instances created while the first exists: characterize what the UART port reservation does (second instance on the same `motor_uart`) | `characterization:` |

### R12: defect candidates (from reading the source and the SM documents)

Items 1-3 are `KNOWN_ISSUE` / `KNOWN_ISSUE_UB` (decision L2); the others become a test only after a writer shows the library result contradicts the oracle.

1. B-SM-1: `address(address + 1)` initialises `address` from itself (`MS:30`, `instance` unused): indeterminate value, both instances share one unpredictable address (`KNOWN_ISSUE_UB`).
2. B-SM-2: `position = value + 0xffff * overflows - 0x7fff` loses one count per 16 bit wrap (R6.2).
3. B-SM-3: `getVoltage()` returns `int16_t` mV: wraps above 32.767 V (R9.2).
4. B-SM-4: `st` in `getSettings` is read uninitialised when the status read fails (`MS:170-178`): not testable deterministically (stack content), recorded as NEEDS REVIEW only.
5. (number 5 unused: the `reg` header corruption of R5.8 is `characterization:` unless the owner decides it is a defect, then B-SM-5.)
6. Sign extension of negative returned values is commented out (`MS:424-425`): R5.7 `characterization:`.
7. `sendCommand` failure does not set `uartErrorOccured` (`readParameter` does): informational.
8. `CommandHandler("sm2", CLSID_MOT_SM1, instance)`: the same pattern as the other drivers (class id of instance 0 + instance index); the handler name "sm2" is a leftover name: informational.
9. `readParameter` with `replylen == none` relies on a previous `getSettings` having fixed the device's return format: `getVoltage` / `getTorque` / `reg getat` before initialization read in the device default format (R4.1 order shows the dependency; informational).
10. B-SM-5 (**T2, found by the protocol writer**): when the receive path rejects a reply (wrong CRC-16 or another device address) it calls `resetBuffer()`, which clears `waitingReply`; the busy wait of `readParameter` / `writeParameter` then ends as if the reply had arrived, copies the stale `replyvalues[]` and returns true. Intended (R5.3): false. `KNOWN_ISSUE`, three cases (bad CRC read, bad CRC write, wrong address read).
11. B-SM-6 (**T2, review finding**): `getTorque()` returns `int16_t` mA: 20000 counts = 35714 mA wraps to -29822 (same shape as B-SM-3). `KNOWN_ISSUE`.
12. Informational (T2): after a timeout the next request is refused until more than 10 ms passed since the last RECEIVED byte (`prepareUartTransmit`): the unanswered frame starts no timeout of its own (`characterization:`, NEEDS REVIEW).

## Fake: SimpleMotion device on the motor UART (T1, owner approved question 31)

* **UART peer hook (shared platform, minimal, neutral by default):** `host_platform.cpp` gets an optional `HostPlatform::UartPeer` interface with `onTransmit(UART_HandleTypeDef*, data, size)` and `onReceiveArmed(UART_HandleTypeDef*)` called from `HAL_UART_Transmit_IT / _DMA / _Transmit` and `HAL_UART_Receive_IT / _DMA` (null by default: no change for any other test), and a `uartCompleteTx(UART_HandleTypeDef*)` overload (the existing function hard-codes `huart1`; the motor port is `huart3`).
* **`FakeSmDevice : UartPeer`** (`fakes/FakeSmDevice.{h,cpp}`, driver executable only): parses fast update frames (CRC8 init 0x52, replies `{0x11, position, status, crc8}` from a configurable position / status) and instant command frames (CRC16 of the SM algorithm, `SMP_RETURN_PARAM_ADDR` / `LEN` semantics: every outbound subpacket returns one subpacket of the configured return format holding the value at the current return address, written parameters stored in a map with configurable defaults for 6020, 553, 565, 493, 559, 900, 901, 13, 552, 554, 2533), stores every received frame for the tests, and delivers reply bytes when the firmware arms the receive (the firmware arms after `transmit_IT` returns, so the reply is held until `onReceiveArmed`). Strictness: a frame with a wrong CRC, a wrong length byte, a subpacket the SM format does not allow, or a transmit while the previous exchange is unfinished is a recorded violation that fails the test unless declared. Fault injection: silent peer, wrong CRC on the reply, wrong address, split / delayed bytes, overlong reply, UART error flag, status word, position sequence.
* **Header comment** with models / source (SM library files above, UART HAL contracts as in `host_platform.cpp`) / does not model (RS485 electrical layer, collisions, device-side parameter semantics beyond the table, buffered commands, baud timing) / confidence (High for frame formats and CRCs, checked against the reference library by script; Medium for the return-format semantics, taken from the `simplemotion_defs.h` comments only).
* **Other T1 items:** `SIMPLEMOTION` on in `platform/target_f407_drivers/target_constants.h` only (it brings `GPIO_MOTOR` via `constants.h`); `MotorSimplemotion.cpp` in `DRIVER_FIRMWARE_SOURCES` with `-funsigned-char` as a source property; the definition of `gpMotor` (`OutputPin` on `DRV_GP1_GPIO_Port` / `DRV_GP1_Pin`, `cpp_target_config.cpp:42`) in the platform of the driver executable; the busy-wait loops of `readParameter` need simulated time to advance while they poll: either the peer advances `HostPlatform` time per `refreshWatchdog()` call through a small hook in the fake watchdog or the driver fixture does, decided in T1 (the existing `refreshWatchdog` is the real one from `cppmain.cpp` in `ffb_target_tests`: check); `test_driver_setup.cpp` expects `SIMPLEMOTION` on; the chooser side effect (ids 9 / 10 appear in the driver lists) checked against all existing driver tests, as in P14; the other executables keep identical compile commands.

## Fixture and test layout (T1 / T2)

* `SmFixture : TargetFixture` (`support/SmFixture.h`, no edit of existing fixtures): clears the fake flash, installs a `FakeSmDevice` as the UART peer (removed at teardown), sets the peer defaults to a healthy ABN servo in torque mode, resets the in-use flags of both instances, provides `makeSm(instance)` (**placement-new into a buffer pre-filled with 0xA5, so the self-initialised `address` of B-SM-1 is deterministic: 0xA5 + 1 = 0xA6; every other test uses the address the peer sees and never assumes a value**), `initialize()` (runs `getSettings` after the 150 ms interval), fails the test on a peer violation not declared with `expectViolation()` and on `unmodelledHalCalls`.
* Files and writer groups: `test_sm_setup.cpp` (R1, R2, R11, T1 skeleton), `test_sm_protocol.cpp` (R4, R5, R10 `reg`), `test_sm_fast.cpp` (R3, R6, R7), `test_sm_control.cpp` (R8, R9, R10, remaining commands).
* Counts to aim at (writers): setup about 15, protocol about 45, fast about 35, control about 25; not a target to pad.

## Target differences to flag

`char` signedness (above, compiled `-funsigned-char`); `__REVSH` (ARM intrinsic, the host stub must equal it: swap of the low 16 bits with sign extension to `int`, checked by the CRC16 results of R5.1); `int16_t` narrowing in `getVoltage()` (implementation-defined, modular on ARM GCC: B-SM-3 is the same on both); `volatile` members written in the interrupt: single threaded host, no race shown; `uint8_t` array sizes computed at run time from the enum (variable length arrays in `readParameter`, GCC extension on both); packed struct, little endian.

## Out of scope for P15

The real RS485 bus, the SM device firmware, the IONI / Argon drives, `smBus` of the PC library, buffered motion, baud timing.

## Decisions (lead, Sonnet 5.5) and owner questions

* L1: the SM library and its documentation are the oracle for frame formats, CRCs, parameter ids, status bits and units; the library source is used only to derive expected values and to check the fake.
* L2: B-SM-1 (`KNOWN_ISSUE_UB`), B-SM-2 and B-SM-3 (`KNOWN_ISSUE`); everything the SM documents do not decide (cpr factors for Hall / Resolver / Serial, timing constants, sign extension, `reg` address limit, `restart` clearing a real fault) is `characterization:`.
* L3: the peer fake delivers replies at `onReceiveArmed`; time for the busy-wait loops advances through a hook (T1 decides where).
* L4: `-funsigned-char` for `MotorSimplemotion.cpp` in the test build only.
* Owner questions (to become numbers 38 and up): (a) is the one-count loss per 16 bit position wrap (B-SM-2) a defect? (b) should `getVoltage()` return more than 16 bits (B-SM-3)? (c) are the Hall / Resolver / Serial cpr factors (`FBR * 4`) intended? (d) should returned negative values be sign-extended (R5.7), and should `reg` reject addresses above 0x3FFF (R5.8)? (e) is the unused `instance` argument (B-SM-1, both instances share one address) a defect?
