# P14-P17 plan: PWM (P14), SimpleMotion (P15), MyActuator (P16), MIDI (P17) (2026-11-10, DRAFT, waiting for the owner's answers)

Scope change (owner, 2026-11-10): these are now IN scope. Nothing is built or tested yet. `CLAUDE.md`, `AGENTS.md`, `TEST_PLAN.md`, `COVERAGE_AND_SCOPE.md` and the feature headers are NOT edited until the owner answers questions 27-31.

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

## Open questions for the owner (in `OPEN_QUESTIONS.md` as 27-31)

27. **Specifications.** Please supply, or say "characterization only" for each: (a) RMD / MyActuator CAN protocol manual (the firmware's `RmdMotorCAN.cpp` uses its command bytes); (b) the SimpleMotion protocol / library documentation (frames, CRC, parameter ids, FBR enum); (c) USB MIDI class spec 1.0 and the MIDI 1.0 message table. PWM needs none.
28. **Executables.** Proposal: PWM, SimpleMotion and RMD go into `ffb_driver_tests` (it already has the motor-driver fixtures, fake CAN, `MockEncoder`), MIDI (handler, main, class) goes into `ffb_usb_tests` (it has the fake dwc2). Or one new executable for all four?
29. **`CLAUDE.md` and `AGENTS.md`.** They still say these are out of scope with their macros off. May I update both (scope lines, macros on in the chosen executables) and push, as in P13?
30. **Order.** P14 PWM, P15 SimpleMotion, P16 MyActuator, P17 MIDI (owner set the phase split 2026-11-10); is the order right?
31. **Fakes.** PWM needs a timer / PWM-output fake and SimpleMotion a UART fake with real framing behaviour if none exists. OK to add them under `fakes/` with the usual fidelity header?

## Not decided yet

Requirements, oracles, case counts and coverage targets. They come with each unit's charter (T0), after the answers above.
