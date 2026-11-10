# P14-P17 plan: PWM (P14), SimpleMotion (P15), MyActuator (P16), MIDI (P17) (2026-11-10; all four phases complete, approved and pushed; P17 T2 is `fe232b2d`)

Scope change (owner, 2026-11-10): these are now IN scope. Status 2026-11-10 (documentation review): P14, P15, P16 and P17 are built, tested, approved and pushed (P17: charter `Tests/claude/charters/driver/MIDI_REQUIREMENTS.md`, work orders `P17_WORK_ORDERS.md`, T0 `f21e631a`, T1 `88980f93`, T2 `fe232b2d`). `CLAUDE.md`, `AGENTS.md`, `TEST_PLAN.md` and `COVERAGE_AND_SCOPE.md` are updated; the feature headers are changed per phase (T1).

## Units

| Unit | Files (lines) | Feature macro (F407VG header) | Talks to |
|---|---|---|---|
| P14 PWM driver | `UserExtensions/Src/MotorPWM.cpp` (332), `Inc/MotorPWM.h` (107) | `PWMDRIVER` | HAL timers (PWM outputs), GPIO |
| P16 MyActuator RMD | `UserExtensions/Src/RmdMotorCAN.cpp` (412), `Inc/RmdMotorCAN.h` (144) | `RMDCAN` | CAN bus (fake HAL CAN exists) |
| P15 SimpleMotion | `UserExtensions/Src/MotorSimplemotion.cpp` (528), `Inc/MotorSimplemotion.h` (288) | `SIMPLEMOTION` | UART (`UARTDevice`), CRC, motor gpio pin |
| P17 MIDI main | `UserExtensions/Src/MidiMain.cpp` (180), `Inc/MidiMain.h` (78) | `MIDI` | `MidiHandler`, USB MIDI class |
| P17 MIDI handler | `Src/MidiHandler.cpp` (138), `Inc/MidiHandler.h` (33) | `MIDI` | P17 TinyUSB MIDI class |
| P17 TinyUSB MIDI class | `USB/class/midi/midi_device.c` (570), `midi_device.h`, `midi.h` (tier B vendor code) | `MIDI` | fake dwc2 controller (exists from tier B) |

About 2,800 lines in total. The MIDI class is vendor code (report bugs, never fix).

## Facts found

- Specs (original state, superseded): when the plan was drafted there was no spec for the RMD (MyActuator) CAN protocol, the SimpleMotion protocol or the USB-MIDI class. All were supplied later (see decision 27 and the P17 oracles below). PWM needs no protocol spec, only the timer / RM0090 chapters already supplied.
- Rule: never invent register maps, opcodes or frames. Without a spec, byte values are `characterization:` tests (as in P13) and the unit is not "specified".
- Existing fakes: fake HAL SPI and CAN (used by the driver executable), UART fake (check the platform for `UARTPort`), fake dwc2 and CDC capture (tier B), `MockMotorDriver`, `MockEncoder`. Timer / PWM HAL may need a new strict fake (`fakes/`, with the fidelity header rules).
- State now: `PWMDRIVER`, `SIMPLEMOTION` and `RMDCAN` are on in `ffb_driver_tests` only (`platform/target_f407_drivers/target_constants.h`); `MIDI` is on in `ffb_usb_tests` only (since P17 T1) and off everywhere else.

## Phases (each is its own phase, stop for the owner after each)

1. **P14 PWM** (`MotorPWM.cpp`; no spec needed, smallest). Charter, fake timer, tests.
2. **P15 SimpleMotion** (`MotorSimplemotion.cpp`; after spec).
3. **P16 MyActuator** (`RmdMotorCAN.cpp`; after spec; reuses the fake CAN).
4. **P17 MIDI** (`MidiHandler`, `MidiMain`, then the TinyUSB `class/midi` against the fake dwc2; after USB-MIDI spec).

Inside each phase: T0 charter by the lead, T1 build wiring and fixture, T2 writers (Sonnet 5.5) plus reviewers with mutation checks, full gate (default, sanitizer, DSP off, clang, coverage), docs, stop for approval. Same roles and rules as P13; max 4 parallel writers.

Gate rule: the 318 baseline cases stay unchanged, the other executables get no new definitions unless the owner decides otherwise (question 28), no firmware source is edited, no `#ifdef` for tests.

## Owner's decisions (2026-11-10)

27. Spec: `CAN_BUS_Motor_Motion_Protocol_V4.3.pdf` supplied (copied to the git-ignored `Tests/shared/specs/`) for MyActuator (P16). Later the same day: SimpleMotion (P15) docs and reference library supplied in `Tests/shared/specs/SimpleMotionV2-master/` (added to `Tests/.gitignore`, never committed; the reference source is an oracle only, not copied into tests); USB-MIDI (P17) spec `USB MIDI v2_0.pdf` supplied. Caveat for P17: the firmware class (`midi.h`: header, in/out jacks, elements) looks like USB-MIDI 1.0, while the supplied spec is 2.0; to be settled in the P17 T0 charter (version differences are `characterization:` or an owner question). PWM (P14) needs none.
28. Executables as proposed: PWM, SimpleMotion, RMD in `ffb_driver_tests`; MIDI in `ffb_usb_tests`.
29. `CLAUDE.md` and `AGENTS.md` updated.
30. Order P14 PWM, P15 SimpleMotion, P16 MyActuator, P17 MIDI.
31. New fakes (timer / PWM output, UART with framing if missing) approved, under `fakes/` with the fidelity header.

## Not decided yet

Requirements, oracles, case counts and coverage targets. They come with each phase's charter (T0).

## P17 oracles (supplied 2026-11-10, git-ignored under `Tests/shared/specs/`)

- `midi10.pdf` (restored 2026-11-10 after being briefly missing): despite its name this is "Universal Serial Bus Device Class Definition for MIDI Devices, Release 1.0" (Nov 1, 1999, 43 pages): USB-MIDI event packets and code index numbers, MIDIStreaming descriptors, class requests. Oracle for the TinyUSB `class/midi` code.
- `MIDI_1.0_Detailed_Specification.pdf` (document version 4.2.1, 86 pages): oracle for the message bytes.
- `USB MIDI v2_0.pdf` is no longer needed: the firmware class is USB-MIDI 1.0 style. This settles the caveat in decision 27; the P17 T0 charter records it.
- Where both are silent (the firmware's own mapping of MIDI messages to internal actions) the tests are `characterization:` with NEEDS REVIEW rows.
