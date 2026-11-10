# P14-P17 plan: PWM (P14), SimpleMotion (P15), MyActuator (P16), MIDI (P17) (2026-11-10, owner's answers to questions 27-31 recorded; P14 not started)

Scope change (owner, 2026-11-10): these are now IN scope. Nothing is built or tested yet. `CLAUDE.md`, `AGENTS.md`, `TEST_PLAN.md` and `COVERAGE_AND_SCOPE.md` are updated; the feature headers are changed per phase (T1).

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

- Specs in `Tests/shared/specs/` today: TMC4671, TMCL, ODrive CAN, VESC CAN, STM32F407 RM / datasheet / errata, `bldc-master`. There is NO spec for the RMD (MyActuator) CAN protocol, the SimpleMotion protocol (UART frames, commands, parameter ids, FBR) or the USB-MIDI class (USB-MIDI 1.0 class spec, MIDI 1.0 message table). PWM needs no protocol spec, only the timer / RM0090 chapters already supplied.
- Rule: never invent register maps, opcodes or frames. Without a spec, byte values are `characterization:` tests (as in P13) and the unit is not "specified".
- Existing fakes: fake HAL SPI and CAN (used by the driver executable), UART fake (check the platform for `UARTPort`), fake dwc2 and CDC capture (tier B), `MockMotorDriver`, `MockEncoder`. Timer / PWM HAL may need a new strict fake (`fakes/`, with the fidelity header rules).
- Today all four macros are off in every executable; the F407VG header turns them on. The `ffb_driver_tests` feature header (`platform/target_f407_drivers/target_constants.h`) says they stay off.

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
