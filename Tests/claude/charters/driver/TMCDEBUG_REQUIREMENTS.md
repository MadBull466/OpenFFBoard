# TMCDebugBridge: requirements charter (phase P13, T0)

Status: **APPROVED by the owner, 2026-11-10 (T0); T1 approved and committed; T2 done (results in `Tests/claude/plans/PROGRESS.md` "P13 review result (T2)"). Corrections after T2: mode 7 is `prbs_position_mode` per DS 0x63 (valid, not unnamed); header line numbers: limits at `TMCDebugBridge.h:55-63`, `tmcpids` at `:44-53`.** Revised the same day after the owner supplied the TMCL reference and two board manuals (sources table, R2, R5, L1, L5). R5 items 8 and 9 were resolved by reading the source (not defects). Changes only by the lead, recorded at the end of this file and in `Tests/claude/plans/PROGRESS.md`.

Unit under test: `Firmware/FFBoard/UserExtensions/Src/TMCDebugBridge.cpp` (258 lines, `TDB` below) and `Inc/TMCDebugBridge.h` (`TDH`). Class `TMCDebugBridge : FFBoardMain`, behind the macro `TMCDEBUG`.
Executable: `ffb_driver_tests` (owner's decision 2026-11-10), tests in `Firmware/Tests/claude/cases/driver/test_tmcdebug_*.cpp`, fixture `TmcFixture` (D1, approved) plus the CDC capture of `platform/src/host_usb.cpp` (`usbState.cdcTx`, `cdcWrites`, `cdcFlushes`).
Plan: `Tests/claude/plans/P13_TMCDEBUGBRIDGE_PLAN.md`. Driver charter it builds on: `Tests/claude/charters/driver/TMC4671_REQUIREMENTS.md`.

## Sources of expected values (oracles)

| Code | Source |
|---|---|
| DS | TMC4671-LA datasheet Rev 2.07 (`Tests/shared/specs/TMC4671-LA_datasheet_Rev2.07.pdf`, never committed), only through the register traffic the approved D1 driver tests already verify (`FakeTmc4671`): SPI frame layout, register numbers and bit fields. Cite `DS <section>`. |
| DRV | The TMC4671 driver behaviour already specified and approved in D1 (`TMC4671_REQUIREMENTS.md`): what `turn`, `setTargetPos`, `startMotor`, ... must do. The bridge tests check THAT the bridge calls them with the right arguments (observable at the fake chip), not their internals. |
| HD | Hand derivation in a comment next to the assertion (checksum sums, `__REV` byte order, version decoding). |
| FW | The firmware's own behaviour. Test name starts with `characterization:` and the case gets a NEEDS REVIEW entry. |
| TMCL | `Tests/shared/specs/TMCL_reference.pdf` (TMCL Reference and Programming Manual v2.22, 2008-05-15, supplied 2026-11-10, never committed): binary request format §2.1, checksum §2.1.1, reply format §2.2, status codes §2.2.1, GGP §3.10, GIO §3.15, control functions and command 136 (get firmware version) §3.35, global parameters §5. The frame, checksum, reply layout and status code tables are repeated unchanged in the 2021 manuals `TMC4671-TOSV-REF_HW_FW_manual_Rev1.1.pdf` §6.1 and `TMC4671-LEV-REF_datasheet_hw1.00_fw1.00_rev1.00.pdf` §5.1, so those parts are current. The module-specific parts of the 2008 reference (global-parameter numbers, GIO lines and ranges, which are for TMCM stepper modules) are NOT specification for this firmware. The two board manuals only document SAP / GAP / STAP / RSAP and the axis parameters of Trinamic's own boards. Cite `TMCL <section>`. |
| (none) | **Not in any supplied document:** the opcodes 143, 146 and 148, the meaning of GGP types 2 and 5 for this firmware, the GIO type 5 voltage value, the content of the version string and the byte layout of the binary version reply. These are FW: `characterization:` + NEEDS REVIEW. Never invent a TMCL value from memory. |

Rules inherited from `CLAUDE.md` and the D1-D3 charters: fixture per test, simulated time only, expected values never copied from the library without `characterization:`, a suspected defect is a `KNOWN_ISSUE("...")` test of the INTENDED behaviour (first failing assertion = the defect), UB is `KNOWN_ISSUE_UB(_HIDDEN)`, a crash is `doctest::skip` + "KNOWN ISSUE (crash)"; no firmware edit, no `#ifdef` in firmware, fakes in `fakes/` or stubs in `platform/src`.

## Facts every test author needs (read from the source by the lead)

* **Construction** (`TDB:236-252`): `registerCommands()`, `drv = make_unique<TMC_1>()`, `drv->conf = TMC4671MainConfig{}`, `setAddress(1)`, `restoreFlash()` (before `initialize`), `setLimits(tmclimits)` (values in `TDH:44-52`), `setEncoderType(NONE)`, `Start()` (starts the driver thread), `enablePin.set()`. `tmcpids` (`TDH:36-43`) is declared but NOT used (`setPids` is commented out). `TDH:67-68` declares `SPI_transmit_receive`, never defined or called.
* **Destructor** (`TDB:254`): `HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET)`.
* **Identity** (`TDB:15-23`): `info = {name "TMC Debug Bridge", id CLSID_MAIN_TMCDBG (11, comment), visibility ClassVisibility::debug}`; `isCreatable()` true (`TDH:30`); `getHelpstring()` fixed text (`TDH:33`).
* **Text commands** (`registerCommands`, `TDB:34-43`; `command`, `TDB:47-126`). Registration flags (name -> flags): `reg` GETADR|SETADR, `torque` GET|SET, `pos` GET|SET, `openloopspeed` SETADR|SET, `velocity` GET|SET, `mode` GET|SET, `openloopspeedpwm` SETADR|SET. Dispatch by `cmd.type`:

  | Command | get | set | getat | setat | other |
  |---|---|---|---|---|---|
  | `torque` | reply `getTorque()` | `startMotor()` first if `getState() != Running`, then `turn(val)` | - | - | `ERR` |
  | `pos` | reply `getPos()` | `setTargetPos(val)` | - | - | `ERR` |
  | `velocity` | reply `getVelocity()` | `setTargetVelocity(val)` | - | - | `ERR` |
  | `openloopspeed` | - | `setOpenLoopSpeedAccel(val, 100)` | - | `runOpenLoop(adr, 0, val, 10, true)` | `ERR` |
  | `openloopspeedpwm` | - | `setOpenLoopSpeedAccel(val, 100)` (same as above) | - | `runOpenLoop(adr, 0, val, 10, false)` | `ERR` |
  | `mode` | reply `(uint8_t) getMotionMode()` | if `val < (uint8_t) MotionMode::NONE`: `setMotionMode(MotionMode(val))` then `startMotor()` | - | - | else reply the fixed help text and return `OK` (also for `set` with an out-of-range or negative value) |
  | `reg` | - | - | reply `readReg(val)` (NOTE: the address is `cmd.val`, not `cmd.adr`) | `writeReg(adr, val)` | `ERR` |

  Unknown `cmdId`: `NOT_FOUND`. Return `OK` after a successful branch. `cmd.val` and `cmd.adr` are `int64_t` (`CommandHandler.h:70-71`); `MotionMode(cmd.val)` of a negative value is an out-of-range enum conversion.
* **Binary frames** (`cdcRcv`, `TDB:133-231`): only if `*Len == 9 && *Buf == 1`, else `FFBoardMain::cdcRcv(Buf, Len)`. Checksum = sum of the first `*Len - 1` bytes mod 256 (`TDB:25-32`); a frame whose last byte differs is dropped silently (`TDB:143-146`, no reply, no fallthrough to `FFBoardMain`). Fields: `cmd = dat[1]`, `addr = dat[2]`. Replies are 9 bytes via `sendCdc` = `tud_cdc_n_write(0, ...)` + `tud_cdc_write_flush()`:

  | cmd | what the code does | reply built |
  |---|---|---|
  | 148 | `readReg(addr)`, `__REV`, memcpy | `{2,1,0x64,0x94, v3,v2,v1,v0, checksum}` (value big endian) |
  | 146 | `ndat` = bytes 4..7 as read (`memcpy`), `writeReg(addr, __REV(ndat))` | `{2,1,0x64,0x92, dat[3..6], checksum}` (NOTE: echoes bytes 3..6, not 4..7) |
  | 143 | addr 3: `{2,1,40,0x8f,0,0,0,2}`; addr 4: `{2,1,40,0x8f,2,6,2,2}`; else 8 zero bytes | + checksum |
  | 10 | `{2,1,64,0x0A,0,0,0,0}`, byte 7 = 2 if addr 5, = 1 if addr 2, else 0 | + checksum |
  | 0x0F | `{2,1,64,0x0F,0,0,0,0}`; addr 5: `v = getIntV() / 100`, byte 6 = v >> 8, byte 7 = v & 0xFF | + checksum |
  | 0x88 | addr 0: the 9 raw bytes `{2,'0','0','1','5','V','3','0','7'}` (no checksum; first byte 2 = host address, then an 8 character string); addr 1: `{2,1,0x64,0x88, 0,15,3,7}` decoded from the string (`TDB:203-222`, hand derivation: module high `(v[1]-'0')*10+(v[2]-'0')` = 0, module low `(v[3]-'0')*10+(v[4]-'0')` = 15, fw high `v[6]-'0'` = 3, fw low `(v[7]-'0')*10+(v[8]-'0')` = 7); any other addr: no reply | + checksum for addr 1 |
  | any other | nothing happens, no reply | - |

  The version decode of the 9-byte string is arithmetic (`TDB:203-222`): hand-derive it in the test. All the byte values in this table are FW (no spec).

## Requirements

Every requirement names its oracle. "C" = `characterization:` test + NEEDS REVIEW.

### R1: text commands (TDB:34-126)

| Id | Requirement | Oracle |
|---|---|---|
| R1.1 | `registerCommands` registers exactly the seven commands with the flags above (name, id, help text, flags readable through the command list / `getCommandsInfo`-style public interface) | FW, plain (the flags are the interface) |
| R1.2 | Each command with each command type of the table behaves as the table says, observed at the fake chip (register writes of D1 DRV) and in the replies: `torque set` starts the motor first when the state is not `Running` and does not when it is (state check in the table), then writes the torque target; `pos`, `velocity` set write their target registers; `get` returns the driver's value | DRV + DS |
| R1.3 | Wrong command types return `CommandStatus::ERR`, an unknown `cmdId` returns `NOT_FOUND`, and no register traffic happens for them | FW, plain |
| R1.4 | `mode`: get returns the mode number; set `0..NONE-1` sets the mode and starts the motor; set `>= NONE` and a negative value return the fixed help text with `OK` and do NOT change the mode | FW (C for the text string); defect candidate for negative values (R5) |
| R1.5 | `reg`: `getat` returns the register addressed by `val` (and NOT by `adr`), `setat` writes `adr`, `val` | DS (SPI frame at the fake) |
| R1.6 | `openloopspeed` and `openloopspeedpwm`: `set` and `setat` call the driver with the arguments of the table (accel 100 / 10, `true` / `false`) and the visible effect at the fake chip | DRV + FW |
| R1.7 | The help string of `getHelpstring()` and the command help texts are the fixed strings (C) | FW |

### R2: binary frames (TDB:133-231)

Oracle column: TMCL = the supplied specification (plain test), FW = no specification (`characterization:` + NEEDS REVIEW), KI = `KNOWN_ISSUE` of the INTENDED behaviour (owner's decision L5: the silent drops are defects).

| Id | Requirement | Oracle |
|---|---|---|
| R2.1 | A frame is handled by the bridge only for length 9 and first byte 1; lengths 0, 1, 8, 10 and first byte 0, 2 go to `FFBoardMain::cdcRcv` (the text command parser) and produce no TMCL reply | FW, plain (routing is the firmware's own design) |
| R2.2 | Checksum rule: the checksum is the 8-bit sum of the first eight bytes including the module address; a valid frame is processed; a sum of 256 wraps to 0 | TMCL §2.1.1 + HD, plain |
| R2.3 | A frame with a wrong checksum must be answered with a reply whose status is 1 (wrong checksum), and must not touch the chip. The code drops the frame silently | TMCL §2.2, §2.2.1: **KI** (first failing assertion: a reply is sent; the no-register-traffic assertion is a separate plain test) |
| R2.4 | Reply layout for the commands the bridge answers: `{reply address 2, module address 1, status 100, command number, value MSB first, checksum = 8-bit sum of the first 8 bytes}` | TMCL §2.2, plain |
| R2.5 | cmd 148 (read register): the value is the register content, big endian on the wire (`__REV`), the register is read at the fake chip (SPI frame); the opcode number 148 itself is not in the supplied documents | TMCL §2.2 (layout) + DS (SPI), plain; the opcode number: FW (C) |
| R2.6 | cmd 146 (write register): the register at `addr` receives the big-endian value of bytes 4..7 (`__REV`); the reply echoes request bytes 3..6 (not 4..7); the opcode number 146 is not in the supplied documents | TMCL §2.1 (value MSB first) + DS for the write; the echo content and the opcode number: FW (C) |
| R2.7 | cmd 143 with addr 3 and 4: the two replies; any other addr must answer with an error status from the table (3 wrong type or 4 invalid value), not status 0 (the code builds an all-zero frame with a valid checksum: status 0 is not a status code) | replies for addr 3 / 4 and the opcode number 143: FW (C); the unknown-addr reply: **KI** (TMCL §2.2.1: valid statuses are 100 and 1..6) |
| R2.8 | cmd 10 (GGP) and cmd 0x0F (GIO) are the opcodes of GGP (TMCL §3.10) and GIO (TMCL §3.15) and answer with the layout of R2.4; GGP type 5 returns 2, type 2 returns 1, others 0; GIO type 5 returns the voltage in 0.1 V units (`getIntV() / 100`, big endian, hand-derive from the fake ADC value of `TmcFixture`), other types 0 | opcode numbers and layout: TMCL, plain; the type values and the voltage: FW (C) + HD |
| R2.9 | cmd 0x88 = 136 (get firmware version, TMCL §3.35): type (addr) 0 answers 9 bytes `{host address 2, 8 character version string}` with NO checksum (TMCL "special reply format of command 136"); type 1 answers the normal reply format with version bytes in the value field | the format of type 0 (9 bytes, host address first, 8 characters, no checksum) and the normal layout of type 1: TMCL, plain; the string content `"0015V307"` and the binary bytes `0,15,3,7` (hand-derived from the string, `TDB:203-222`): FW (C) + HD |
| R2.10 | cmd 0x88 with a type other than 0 and 1, and every unknown command number, must be answered with an error status (2 invalid command for an unknown command, 3 wrong type for a bad type) and no register traffic; the code sends nothing and does not pass the frame on | TMCL §2.2.1: **KI** |
| R2.11 | Every reply is written with one `tud_cdc_n_write` of 9 bytes followed by one flush (`cdcWrites`, `cdcFlushes`) | FW, plain |
| R2.12 | CDC transmit buffer shorter than 9 bytes (`usbState.cdcWriteAvailable`): the reply is truncated and nothing crashes | FW (C) |

### R3: life cycle and identity (TDB:15-23, 236-256)

| Id | Requirement | Oracle |
|---|---|---|
| R3.1 | Constructor effects visible at the fake chip and in the driver object: address 1 (`TMC_1`, chip select PA4), `restoreFlash()` runs before any other register access (flash values restored), limits of `tmclimits` written (`pid_*` registers) after `restoreFlash`, encoder type NONE (no alignment started), thread started (`Start()`), `enablePin` set (driver enable GPIO high) | DRV + DS |
| R3.2 | Destructor drives `DRV_ENABLE` low (read the fake GPIO) and the driver is destroyed with it (thread stops, no leak under ASan) | FW, plain |
| R3.3 | `getInfo()` returns name "TMC Debug Bridge", id `CLSID_MAIN_TMCDBG`, visibility debug; `isCreatable()` is true | FW, plain |
| R3.4 | `mainclass_chooser.cpp` with `TMCDEBUG` on lists the class with that id and can create it (a create / destroy round trip) | FW, plain |
| R3.5 | `tmcpids` are NOT applied (`setPids` commented out): after construction the PID registers hold the restored / default values, not `tmcpids` (C: documents the dead code) | FW (C) |

### R4: robustness

| Id | Requirement | Oracle |
|---|---|---|
| R4.1 | A frame split over two `cdcRcv` calls is not reassembled (each call is judged alone): a 9 byte frame delivered as 5 + 4 bytes goes to `FFBoardMain` twice | FW (C) |
| R4.2 | Repeated valid frames after an invalid one still work (no latched state) | FW, plain |
| R4.3 | `cdcRcv` with a valid frame while the driver is in each state (`Shutdown`, `Running`): the register access still reaches the chip (the SPI bus is shared with the driver thread: no deadlock under the fake kernel) | FW, plain |
| R4.4 | ASan / UBSan clean for every case (`Len` 0 reads `*Buf`: see R5) | UBSan |

### R5: defect candidates (from reading the source and the supplied TMCL documents)

Items 4 and 5 are `KNOWN_ISSUE` by the owner's decision L5 (TMCL §2.2, §2.2.1). The others become a `KNOWN_ISSUE` test of the intended behaviour only after the T2 writer shows the library result contradicts DS / DRV / a sound reading and the lead decides (T3); otherwise `characterization:` + NEEDS REVIEW.

1. `openloopspeed set` and `openloopspeedpwm set` are identical (`TDB:73-74` vs `83-84`): the pwm variant differs only for `setat`. Copy and paste? (the help text of both says "adr=strength;val=speed"). Intended behaviour is not specified: characterization.
2. `mode set` range check `cmd.val < (uint8_t) MotionMode::NONE` (`MotionMode : uint8_t`, `NONE` = 10, `TMC4671.h:71`): a negative `int64_t` passes the check and `MotionMode(negative)` truncates to a `uint8_t` value (e.g. -1 gives 255, a wrong mode followed by `startMotor`); the value 7 passes too and is valid (`prbs_position_mode`, DS 0x63; the enumerators of `MotionMode` skip it). Intended for negative values: rejected like an out-of-range value (confirmed by the T2 tests: B-DTDC-1).
3. `cdcRcv` with `*Len == 0`: `*Buf == 1` is evaluated only after `*Len == 9` (short circuit), so safe; verify nothing else reads `Buf[0]`.
4. cmd 143 with an unknown addr answers an all-zero frame with a valid checksum, status 0, which is not a TMCL status code (valid: 100 and 1..6): KNOWN_ISSUE (R2.7).
5. Opcodes without a branch, a cmd 0x88 with a type other than 0 / 1 and a frame with a wrong checksum are swallowed silently (no reply, not passed on), where TMCL §2.2 says every command is answered (status 2 invalid command, 3 wrong type, 1 wrong checksum): KNOWN_ISSUE (R2.3, R2.10). Counter-argument noted for the owner: the bridge shares the CDC port with the text parser and may be silent on purpose; decided as a defect by the owner (L5).
6. cmd 146 echoes bytes 3..6 of the request (the value is in 4..7): characterization (no spec; a TMCL echo of the value would be bytes 4..7, but that is not a supplied fact).
7. ~~cmd 0x88 addr 0 answers a raw string without checksum~~: matches the special reply format of command 136 (TMCL §3.35, host address + 8 characters, no checksum). Not a defect; R2.9.
8. ~~Command flag mismatch~~: checked by the lead (`CommandHandler.h:21-35`): `CMDtype::getat = CMDFLAG_GETADR`, `setat = CMDFLAG_SETADR`, so `reg` is reachable. Not a defect; the test R1.5 still proves it end to end.
9. ~~`reg getat` reads `cmd.val`~~: checked by the lead: `TMC4671::command` (`TMC4671.cpp:3052-3055`) uses the same convention (`getat` reads `readReg(cmd.val)`, `setat` writes `adr`, `val`). Not a defect; documented in R1.5.
10. Declared but unused / undefined members (`SPI_transmit_receive`, `tmcpids`): informational only.

## Fixture and test layout (T1 / T2)

* Fixture `TmcDebugFixture : TmcFixture` (new file `support/TmcDebugFixture.h`, no edit of existing fixtures): constructs the bridge after the fake chip exists, gives access to `usbState` (clear before each test, capture `cdcTx`), helpers `frame(cmd, addr, value)` (builds the 9 bytes with the checksum of the code's rule), `send(frame)` (calls `cdcRcv`), `reply()` (consumes `cdcTx`), `chipWrites()` (register writes seen by the fake).
* Files: `test_tmcdebug_setup.cpp` (macro, chooser, identity: R3.3, R3.4), `test_tmcdebug_commands.cpp` (R1), `test_tmcdebug_frames.cpp` (R2, R4), `test_tmcdebug_lifecycle.cpp` (R3.1, R3.2, R3.5).
* Build wiring (T1): `TMCDEBUG` added to `platform/target_f407_drivers/target_constants.h` ONLY; `TMCDebugBridge.cpp` added to `DRIVER_FIRMWARE_SOURCES` in `CMakeLists.txt`. The other executables keep identical compile commands (check with the compile command diff). The 318 baseline cases and every existing test of all executables must keep their results.
* Counts to aim at (writers): R1 about 35 cases, R2 about 30, R3 about 10, R4 about 8; not a target to pad.

## Out of scope for P13

TMCL semantics beyond the code (no spec), the real TMCL-IDE, USB enumeration (`ffb_usb_tests` covers CDC), the driver internals (D1), timing at the real SPI clock.

## Decisions (lead)

* **L1** (owner, 2026-11-10, revised the same day): the owner supplied the TMCL reference v2.22 and the TMC4671-TOSV-REF / LEV-REF manuals (2021). What they specify (frame, checksum, reply layout, status codes, opcodes 10 / 15 / 136, the type-0 string reply of 136) is tested as plain tests with the section cited; everything they do not specify (opcodes 143 / 146 / 148, GGP and GIO type meanings, version content, binary version layout) stays `characterization:` + NEEDS REVIEW. Structure that the firmware decides itself (routing by length and address byte, 9-byte reply write plus flush) is a plain test.
* **L5** (owner, 2026-11-10): the silent drops (wrong checksum, unknown command number, bad type of command 136) and the status-0 reply of command 143 are defects: `KNOWN_ISSUE` tests of the intended reply (status 1, 2, 3 per TMCL §2.2.1). The first failing assertion is that no reply was sent (or the status byte); register-traffic assertions are separate plain tests.
* **L2** (owner, 2026-11-10): the bridge is built into `ffb_driver_tests` only.
* **L3** (lead): a defect candidate of R5 becomes `KNOWN_ISSUE` only if DS / DRV / the command-handler convention gives an unambiguous intended result; DOC-less firmware oddities are `characterization:`.
* **L4** (lead): the bridge tests do not re-verify the TMC4671 driver; they check calls and arguments at the fake chip, using the register numbers the approved D1 tests already use.
