# OpenFFBoard host test suite: project summary

Branch `add-test-suite`, from the baseline commit `a5418ff5` ("Initial unit test system") to the P9 final commit (see `Tests/Code_Coverage/claude/plans/PROGRESS.md`,
"Phase checklist"). Commits are all authored `MadBull.Ben` without an email; pushed to the fork `MadBull466/OpenFFBoard` only (last push 2026-11-10). Details, decisions and every number below
are in `Tests/Code_Coverage/claude/plans/PROGRESS.md` (the source of truth); all suspected bugs and open questions are in `Tests/Code_Coverage/claude/reports/BUG_REPORT.md`.

> **Update 2026-11-11 (P18).** This summary was written at P9 and is kept as the P9 record; the tables below are updated where they
> are plain counts. The "Open questions for you" list below is the P9 list; the current list is `Tests/Code_Coverage/claude/reports/OPEN_QUESTIONS.md`. Since then P10 (TMC4671, ODrive, VESC), P11 (CMSIS-DSP), P13-P16 (TMCDebugBridge, PWM, SimpleMotion, MyActuator) were added, approved and
> pushed to the fork. Current counts (GCC 16): `ffb_unit_tests` 654, `ffb_target_tests` 1863, `ffb_flash_tests` 159, `ffb_rtos_tests` 278, `ffb_usb_tests` 606,
> `ffb_driver_tests` 1717, `ffb_dsp_tests` 2485 = 7,762 C++ cases (unit tests counted with the documented exclusion), 125 Python tests, ctest 11 of 11. Suspected firmware bugs are numbered up to 225,
> vendor bugs up to V90, NEEDS REVIEW up to 287, open questions up to 64. Remaining out of scope: ST HAL, CMSIS core, FreeRTOS kernel, `Firmware/Targets/**`,
> CubeMX code; MIDI (P17) is done: 319 cases in `ffb_usb_tests` (606 in total), approved and pushed (`fe232b2d`). P18 (2026-11-11, pushed, `544b4162`) adds a 32-bit build variant (`-DFFB_TESTS_32BIT=ON`), which found bug 225 (31 new cases in `ffb_target_tests`). The authoritative numbers are in `Tests/Code_Coverage/claude/plans/PROGRESS.md` and `Tests/Code_Coverage/claude/plans/COVERAGE_AND_SCOPE.md`.

## Result in one paragraph

The existing doctest suite (318 cases) was kept unchanged and extended to **3,203 C++ test cases in five executables plus 61 Python tests (P9 figures; current counts in the update note above)**.
Line coverage of the in-scope firmware (everything outside `Firmware/Libraries/`, minus the confirmed out-of-scope drivers and MIDI) rose
from **26.2 % to 98.9 %** (8,007 of about 8,095 lines); every in-scope file is compiled and tested, and nothing was excluded as hardware-only.
The vendor code in scope is covered too: FreeRTOS C++ wrappers and the ST EEPROM emulation 98.1 %, TinyUSB device code 85.1 %, the CMSIS-DSP
functions the firmware calls 100 %. The firmware was never changed. The suite found **132 suspected firmware bugs, 33 vendor bugs and
145 open questions**, each tied to a test (or marked as code reading).

## What was built (all under `Firmware/Tests/`)

| Executable | Cases | What it runs | How |
|---|---|---|---|
| `ffb_unit_tests` | 654 | the original 318 cases (unchanged) plus gap tests of the command system, FFB, errors, filters, transport | the original fake `Axis`/flash/HAL platform |
| `ffb_target_tests` | 1,832 | the real F407 feature set without drivers/MIDI: axis, main classes, system, inputs, encoders, SPI/I2C/CAN buses, serial FFB, CMSIS-DSP | fake HAL (GPIO, TIM, ADC, SPI, I2C, CAN modelled on the ST HAL), fake FreeRTOS with simulated time and resumable tasks |
| `ffb_flash_tests` | 159 | the real `flash_helpers.cpp` and ST `eeprom.c` | a NOR-flash fake mapped at the real addresses (not on macOS) |
| `ffb_rtos_tests` | 278 | the `cpp_freertos` wrappers | a fake FreeRTOS API following V10.3.1 semantics, create-failure injection, configASSERT faults, leak check |
| `ffb_usb_tests` | 606 | the real TinyUSB device stack with the firmware's descriptors, CDC, HID and (P17) MIDI class driver, `MidiHandler` and `MidiMain` | a fake dwc2 device controller and a host-side USB driver |
| `python_helper_tests` | 115 (counted in `claude/python`) | `generate_memory.py`, `dwc2_info.py` | stdlib `unittest`, temp dirs only |

`ctest` runs all of them (11 entries today incl. the baseline check and the Codex entries; 7 at P9). Six build variants are verified: GCC 16 and clang 23, CMSIS-DSP on and off,
ASan+UBSan. Tools: `tools/check_baseline.py` (the 318 original cases keep their results), `tools/coverage_report.py` (gcov only, no
installs), `tools/make_bug_report.py` (generates `Tests/Code_Coverage/claude/reports/BUG_REPORT.md`).

Conventions that make the results trustworthy: every suspected bug is a `KNOWN_ISSUE` test that describes the **intended** behaviour (it fails
today and keeps the suite green; a firmware fix makes it report "should have failed"); unclear intent is a `characterization:` test with a
NEEDS REVIEW number; undefined behaviour is skipped in sanitizer builds. Each phase was written by isolated agents and checked by fresh
reviewers, who applied plausible firmware fixes in scratch copies to prove that only the matching test changes outcome, plus mutation
spot-checks (each group at or above 80 %, survivors explained).

## Coverage by phase

| After | In-scope lines covered |
|---|---|
| Baseline (P0) | 26.2 % |
| P1 gaps in compiled files | 32.4 % |
| P2 axis and main classes | 50.5 % |
| P3 flash and persistence | 55.3 % |
| P4 system, callbacks, LEDs, voltage, descriptors | 62.6 % |
| P5 buttons and analog inputs | 76.5 % |
| P6 encoders and buses | 86.2 % |
| P7 CAN and serial FFB | 93.3 % |
| P8 FreeRTOS wrappers | 98.9 % |
| P9 vendor tiers B/C and Python | 98.9 % (vendor code added) |

## Findings to look at first

The full list with locations, consequences on the 32-bit target and the test for each is in `Tests/Code_Coverage/claude/reports/BUG_REPORT.md` ("Look at these first"
and "Related bugs"). Highlights:
- **Bug 125 (safety):** a periodic effect created over the serial `fxm` commands has period 0; on the target this likely gives full-scale
  torque. Reachable on all three shipped targets (FFBHIDEXT).
- **Bug 124 / 12:** damper, inertia or friction effects without filters make the CMSIS biquad run about 2^32 stages: the device stops
  (fault or watchdog reset). Reachable over `fxm` and via an HID Set Effect type change.
- **Bug 42:** USB bus suspend never reaches the main class (wrong callback signature): effects keep running when the PC sleeps.
- **Bugs 40, 39, 33 (safety):** an E-stop press within 1 s is lost; a critical error releases an active E-stop; after an E-stop reset the
  slew limiter starts from the torque before the stop.
- **Bug 32:** a power setting above 0x7fff reverses the motor at full force.
- **Bug 137:** after the host resets or unplugs the board while a CDC reply is pending, no CDC reply is ever sent again until a power cycle.
- **Bugs 89, 98 (I2C), 110 (CAN), 131 (CAN input never starts the port on F407VG_DISCO), V5 (EEPROM format loses settings).**
- **Original suite:** "non numeric values are rejected without blocking the parser" is a hang: a typo such as `x.y=abc` over CDC or UART
  loops the command thread forever.

## Open questions for you (details in `Tests/Code_Coverage/claude/plans/PROGRESS.md`, "Open questions for me")

1. **CI:** no GitHub Actions workflow runs these tests. A minimal job is proposed (open question 5); I did not touch `.github/`.
2. **CLAUDE.md / TEST_PLAN.md wording:** both still give the original `ffb_unit_tests` result as "316 passed, 2 skipped"; the executable now
   gives 654/0/4 on GCC 16, with the 316/2 baseline subset checked by `check_baseline.py` (open question 8).
3. **USB HID PID 1.01 specification:** give the spec file path to have the `HidFFB` characterization tests checked against it (optional).
4. **The 145 NEEDS REVIEW questions** in `Tests/Code_Coverage/claude/reports/BUG_REPORT.md`: each answer decides whether a behaviour becomes a bug.

## Limitations

- Windows/MinGW and macOS are prepared in CMake but unverified (no toolchains here); `ffb_flash_tests` is skipped on macOS.
- The fakes model the documented hardware/HAL/FreeRTOS/TinyUSB behaviour the firmware relies on; real timing, preemption, electrical effects
  and peripherals not used by the firmware are not modelled (each fake lists what it records as unmodelled, which fails a test).
- Host 64-bit vs target 32-bit ARM: differences (type sizes, float rounding/FMA, out-of-range conversions, division by zero) are listed in
  `Tests/Code_Coverage/claude/plans/PROGRESS.md`, "Host-vs-target caveats", and handled with tolerances or skipped cases.
- Out of scope by decision: ST HAL, CMSIS core, the FreeRTOS kernel, `Firmware/Targets/**`, CubeMX code. (At P9 the motor drivers, TMCDebugBridge and
  MIDI were out of scope too; they were brought in scope on 2026-11-02 and 2026-11-10 and are tested now.)
