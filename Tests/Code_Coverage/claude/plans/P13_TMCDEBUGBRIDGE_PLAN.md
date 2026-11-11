# P13 plan: `TMCDebugBridge` (2026-11-10, COMPLETE: T0, T1, T2 approved and committed; T3 and T4 ran inside T2; commit `61535bd0`)

Scope change (owner, 2026-11-10): `Firmware/FFBoard/UserExtensions/Src/TMCDebugBridge.cpp` (258 lines) and its header are now IN scope. At the time of writing everything else stayed out (PWM, MyActuator, SimpleMotion, `MidiMain`, `MidiHandler`); the same day they were put in scope as P14-P17 (`P14_P17_REMAINING_DRIVERS_PLAN.md`). P12 is Codex's phase (`Tests/Code_Coverage/codex/p12`); this is P13.

## What the unit is

`TMCDebugBridge` is a `FFBoardMain` class behind the `TMCDEBUG` macro (default off in every executable of the suite). It owns a `TMC_1` driver (`TMC4671`) and offers two interfaces:

1. **Text commands** (`registerCommands` / `command`): `reg` (getat / setat), `torque`, `pos`, `velocity`, `mode` (get / set), `openloopspeed` and `openloopspeedpwm` (set / setat). They call the TMC4671 driver (`readReg`, `writeReg`, `turn`, `setTargetPos`, `setTargetVelocity`, `setMotionMode`, `startMotor`, `setOpenLoopSpeedAccel`, `runOpenLoop`).
2. **TMCL binary frames on the CDC port** (`cdcRcv`): a 9-byte frame starting with address byte 1 (anything else goes to `FFBoardMain::cdcRcv`). It checks an additive checksum and answers 9-byte replies via `tud_cdc_n_write` for the commands 148 (read register), 146 (write register), 143, 10 (get global parameter), 0x0F (get input, voltage from `getIntV`) and 0x88 (version string / binary version).

Constructor: `TMC_1`, `setAddress(1)`, `restoreFlash()`, `setLimits`, `setEncoderType(NONE)`, `Start()`, `enablePin.set()`. Destructor: writes `DRV_ENABLE` low. Registered in `mainclass_chooser.cpp` under `TMCDEBUG`; `TMC4671.h` has `friend class TMCDebugBridge` under the same macro.

## Where it is built

The driver executable `ffb_driver_tests` already compiles the real `TMC4671.cpp` against the fake SPI and `FakeTmc4671` (D1, approved) and the target platform, which has the CDC fake (`platform/src/host_usb.cpp`, `tud_cdc_n_write`) and the `voltagesense` globals (`host_target_system.cpp`). Proposal: add `TMCDEBUG` to the feature header of `ffb_driver_tests` only (`platform/target_f407_drivers/target_constants.h`, which already shadows the other macros) and `TMCDebugBridge.cpp` to `DRIVER_FIRMWARE_SOURCES`; the other executables stay unchanged (identical compile commands). Side effect to test and document: with the macro on, `mainclass_chooser.cpp` of that executable lists the bridge as a debug class.

## Requirements (charter to be signed by the lead before the writers start)

- **R1 TMCL frames** (needs the spec, see below): the reply bytes of every opcode (`2,1,0x64,<opcode>,<value>,<checksum>` layout as coded), register read / write byte order (`__REV`, big endian on the wire), the checksum (sum mod 256 over the first 8 bytes), a wrong checksum drops the frame without a reply, wrong length or address byte goes to `FFBoardMain::cdcRcv`, unknown opcodes and unknown `addr` of opcodes 10 / 0x0F / 0x88 / 143 (what is replied or not), the version string and the decoded binary version, voltage in 0.1 V units from `getIntV() / 100`.
- **R2 Text commands:** every command with every command type (get / set / getat / setat), wrong types return `ERR`, unknown id `NOT_FOUND`, the argument ranges (`mode` set only for `val < MotionMode::NONE`; otherwise the help string is returned), the driver calls and their arguments (`openloopspeed` set uses acceleration 100, `setat` uses `runOpenLoop(adr, 0, val, 10, true)`; the `pwm` variant passes `false`), the SPI traffic through `FakeTmc4671` (register reads and writes, state `Running` before `turn`, `startMotor` only when not running).
- **R3 Life cycle:** constructor effects (address 1, limits, encoder type NONE, `Start`, enable pin high via `enablePin`), destructor drives `DRV_ENABLE` low, `getInfo` (name "TMC Debug Bridge", class id 11 = `CLSID_MAIN_TMCDBG`, visibility debug), `isCreatable`, `getHelpstring`, the chooser registration.
- **R4 Robustness:** frames with `*Len != 9`, `*Len == 0`, `Buf` first byte 1 and length 9 but unknown opcode, replies larger than the CDC buffer, interaction with the text parser (a text command never starts with byte 1), repeated frames, state after a failed checksum.
- **Suspected defects to verify (candidates from reading the source, none asserted yet):** `openloopspeed` and `openloopspeedpwm` do the same for a plain `set` (copy and paste?); `mode set` checks only the upper bound (a negative `cmd.val` converts to `MotionMode`); unknown `addr` for opcode 143 answers an all-zero frame with status 0 and a valid checksum; opcodes that match no branch are swallowed silently without a reply; `SPI_transmit_receive` and `tmcpids` are declared but unused; the registered command types (`CMDFLAG_GETADR` / `CMDFLAG_SETADR`) against the types the code checks (`getat` / `setat`).

## Specifications needed (CLAUDE.md: never invent opcodes or frames; if a spec is missing, stop and ask)

The TMCL framing and the opcode numbers 143, 146, 148, 10, 0x0F and 0x88, the reply status byte 0x64, the global-parameter numbers (2, 5) and the version string format are not in `Tests/Code_Coverage/shared/specs/`: only the TMC4671 datasheet (Rev 2.07) is there. Needed from the owner (git-ignored under `Tests/Code_Coverage/shared/specs/`): the TMCL reference / programming manual and the TMC4671 TMCL firmware manual that the TMCL-IDE uses (command list with the TMC4671-specific commands 143 / 146 / 148, GGP / GIO numbers, reply format), and, if available, a TMCL-IDE capture of a session with the real board. Without it only R2, R3 and R4 (everything that is not a TMCL byte value) can be tested; the TMCL byte values would be `characterization:` at best.

## Phases (stop after each)

Result: T3 (4 reviewers with mutation checks, fixes) and T4 (full gate, docs) were run inside T2 and approved with it. 164 new cases (167 with T1), `TMCDebugBridge.cpp` 166 / 166 lines, firmware findings 194-200, NEEDS REVIEW 238-243: see `PROGRESS.md` "P13 review result (T2)".

| Phase | Content | Who |
|---|---|---|
| T0 | Charter `Tests/Code_Coverage/claude/charters/driver/TMCDEBUG_REQUIREMENTS.md` (R1-R4, oracles, decisions), spec check | lead |
| T1 | Build wiring (macro, source, CMake) + a CDC capture helper test double if `host_usb.cpp` is not enough + skeleton test; baseline check | one writer |
| T2 | Tests for R2 / R3 / R4 (and R1 once the spec is there) in `claude/cases/driver/test_tmcdebug_*.cpp`, fixture on `FakeTmc4671` | writers |
| T3 | Fresh reviewers with mutation checks, fixes | reviewers |
| T4 | Full gate (default, sanitizer, DSP off, clang, coverage), PROGRESS / COVERAGE_AND_SCOPE / README / bug report, stop for approval | lead |

Gate rule: the 318 baseline cases stay unchanged (`tools/check_baseline.py`), the other executables get no new definitions, and no firmware source is edited.

## Owner's decisions (2026-11-10)

1. TMCL specification: first answered "characterization tests"; the owner then supplied `TMCL_reference.pdf` (v2.22), `TMC4671-TOSV-REF_HW_FW_manual_Rev1.1.pdf` and `TMC4671-LEV-REF_datasheet_hw1.00_fw1.00_rev1.00.pdf` (copied to the git-ignored `Tests/Code_Coverage/shared/specs/`). What they specify (frame, checksum, reply layout, status codes, opcodes 10 / 15 / 136) is tested as plain tests; opcodes 143 / 146 / 148, GGP / GIO type meanings and the version content are not in them and stay `characterization:` + NEEDS REVIEW. The silent drops (wrong checksum, unknown command, bad type) and the status-0 reply of 143 are `KNOWN_ISSUE` (owner, L5 of the charter).
2. `CLAUDE.md` and `AGENTS.md` updated (P13 scope line; `TMCDEBUG` on in `ffb_driver_tests` only).
3. The bridge goes into `ffb_driver_tests`.
