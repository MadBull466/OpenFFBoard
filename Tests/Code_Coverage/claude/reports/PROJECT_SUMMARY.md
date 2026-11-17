# OpenFFBoard host test suite: project summary

Branch `add-test-suite`, from the baseline commit `a5418ff5` ("Initial unit test system") to the commit recorded in `Tests/Code_Coverage/claude/plans/PROGRESS.md` ("Current state"). All commits are authored `MadBull.Ben` without an email and pushed to the fork `MadBull466/OpenFFBoard` only. Status of this document: **updated 2026-11-17 for P0 to P18** (the first version was written at P9). Details, decisions and every number below are in `Tests/Code_Coverage/claude/plans/PROGRESS.md` (the source of truth); all suspected bugs and open questions are in `Tests/Code_Coverage/claude/reports/BUG_REPORT.md` and `OPEN_QUESTIONS.md`.

## Result in one paragraph

The existing doctest suite (318 cases) was kept unchanged and extended, in 19 phases (P0 to P18), to **15,902 C++ test cases in nine executables plus 125 Python tests** (7,768 in the seven host executables, plus 4,067 in each of the two real-HAL executables, which run the same files against two targets; counts below). Line coverage of the in-scope firmware (everything outside `Firmware/Libraries/`) rose from **26.2 % to 98.9 %** (8,171 of about 8,260 lines). The vendor code in scope is covered as well: FreeRTOS C++ wrappers and the ST EEPROM emulation 98.1 %, TinyUSB device code 95.9 %, the motor drivers 93.6 %, CMSIS-DSP 99.9 % and the ST HAL of the F407VG and F407VG_DISCO targets 95.9 % per target. Measured on the 32-bit build with everything linked in, the whole project reaches **97.0 %** (both HAL targets counted) or **97.4 %** (HAL counted once); files that are not linked into any test are not measured (`COVERAGE_AND_SCOPE.md`, section 1.6). The firmware was never changed. The suite found **220 suspected firmware bugs, 126 vendor bugs and 407 open NEEDS REVIEW questions**, each tied to a test (or marked as code reading), and the owner has 70 open questions numbered so far (see "Open questions").

## What was built (all under `Firmware/Tests/`)

Counts are cases per executable on the default GCC 16 build, 2026-11-17 (skipped cases included in brackets where they differ).

| Executable | Cases | What it runs | How |
|---|---|---|---|
| `ffb_unit_tests` | 654 (+4 skipped) | the original 318 cases (unchanged) plus gap tests of the command system, FFB, errors, filters, transport | the original fake `Axis`/flash/HAL platform |
| `ffb_target_tests` | 1,863 (+18 skipped) | the real F407 feature set without drivers/MIDI: axis, main classes, system, inputs, encoders, SPI/I2C/CAN buses, serial FFB | fake HAL (GPIO, TIM, ADC, SPI, I2C, CAN modelled on the ST HAL), fake FreeRTOS with simulated time |
| `ffb_flash_tests` | 159 | the real `flash_helpers.cpp` and ST `eeprom.c` | a NOR-flash fake mapped at the real addresses (not on macOS) |
| `ffb_rtos_tests` | 278 | the `cpp_freertos` wrappers | a fake FreeRTOS API following V10.3.1 semantics, create-failure injection, configASSERT faults, leak check |
| `ffb_usb_tests` | 606 | the real TinyUSB device stack with the firmware's descriptors, CDC, HID and MIDI class driver, `MidiHandler` and `MidiMain` | a fake dwc2 device controller and a host-side USB driver |
| `ffb_driver_tests` | 1,722 (+9 skipped) | TMC4671, ODrive, VESC, TMCDebugBridge, PWM, SimpleMotion and MyActuator drivers | simulated peers (fake TMC4671 on a fake SPI, ODrive / VESC / RMD peers on the fake CAN, a fake SimpleMotion device on a fake UART) |
| `ffb_dsp_tests` | 2,486 (+5 skipped) | every function of the CMSIS-DSP modules the firmware uses, all data types | ARM's reference patterns and hand derivations (portable C path) |
| `ffb_hal_tests`, `ffb_hal_tests_disco` | 4,067 each | the real ST HAL (24 files) of the F407VG and F407VG_DISCO targets, compiled unchanged | register-level fakes mapped at the real addresses, 32-bit Linux build only (`FFB_TESTS_REAL_HAL`) |
| `python_helper_tests` | 125 (in `claude/python`) | `generate_memory.py`, `dwc2_info.py` and the report tools | stdlib `unittest`, temp dirs only |

`ctest` runs 11 entries in the default build (the seven C++ executables, the baseline check, the Python entries and the Codex entries) and 14 in the 32-bit real-HAL build. Variants verified at the last gate (2026-11-17): GCC 16 and clang 23, CMSIS-DSP on and off, ASan+UBSan, 32-bit, 32-bit with the real HAL (with GCC, clang and ASan); only `m32-san` has a known limit (`ffb_flash_tests` cannot start under the 32-bit ASan runtime). Tools: `tools/check_baseline.py` (the 318 original cases keep their results), `tools/coverage_report.py` (gcov only, no installs), `tools/run_all_tests.py` (full report: Passed 7141, Failed 3, Bugged 829, Total 7973 on the default build), `tools/make_bug_report.py` (generates `BUG_REPORT.md`).

Conventions that make the results trustworthy: every suspected bug is a `KNOWN_ISSUE` test that describes the **intended** behaviour (it fails
today and keeps the suite green; a firmware fix makes it report "should have failed"); unclear intent is a `characterization:` test with a
NEEDS REVIEW number; undefined behaviour is skipped in sanitizer builds. Each phase was written by isolated agents and checked by fresh
reviewers, who applied plausible firmware fixes in scratch copies to prove that only the matching test changes outcome, plus mutation
spot-checks (each group at or above 80 %, survivors explained).

## Coverage by phase

See `Tests/Code_Coverage/claude/plans/TEST_PLAN.md`, section "Coverage by phase (P0-P18)", for the table with the estimate and the measured result of every phase, and `COVERAGE_AND_SCOPE.md` for the current numbers per tier. In short: the firmware ladder went 26.2 % (P0), 32.4 % (P1), 50.5 % (P2), 55.3 % (P3), 62.6 % (P4), 76.5 % (P5), 86.2 % (P6), 93.3 % (P7), 98.9 % (P8) and stayed there; P9 to P18 add the separate tiers (vendor code, drivers, DSP, HAL) and the 32-bit build, which are not part of that number.

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

## Open questions for you (details in `Tests/Code_Coverage/claude/reports/OPEN_QUESTIONS.md`)

Pending: 17, 18, 20-23 (P11 CMSIS-DSP), 32-52 (P14-P16 drivers), 54-61 (P17 MIDI) and 70 (`sysexState`), plus the project questions 3 (USB HID PID 1.01 specification, optional), 5 (CI job; no GitHub Actions workflow runs these tests, a minimal job is proposed, `.github/` was never touched), 6 and 7 (Windows / macOS). The 407 open NEEDS REVIEW questions in `BUG_REPORT.md`: each answer decides whether a behaviour becomes a bug.

## Limitations

- Windows/MinGW and macOS are prepared in CMake but unverified (no toolchains here); `ffb_flash_tests` is skipped on macOS; the real-HAL executables exist only in the 32-bit Linux build.
- The fakes model the documented hardware/HAL/FreeRTOS/TinyUSB behaviour the firmware relies on; real timing, preemption, electrical effects
  and peripherals not used by the firmware are not modelled (each fake lists what it records as unmodelled, which fails a test).
- Host 64-bit vs target 32-bit ARM: the 32-bit build (P18) covers the size of `size_t`, `long` and pointers; ARM's `uint32_t` type (`unsigned long`) and the 8-byte alignment of `int64_t` / `double` struct members are not proven by it. Other differences (float rounding/FMA, out-of-range conversions, division by zero) are listed in `PROGRESS.md`, "Host-vs-target caveats", and handled with tolerances or skipped cases.
- Out of scope by decision: CMSIS core, the FreeRTOS kernel, the rest of `Firmware/Targets/**` (the F411RE HAL copy, CubeMX code, startup and linker files), OTG_HS and the USB host half of the HAL. (At P9 the motor drivers, TMCDebugBridge, MIDI and, until 2026-11-11, the ST HAL were out of scope too; they were brought in scope later and are tested now.)
