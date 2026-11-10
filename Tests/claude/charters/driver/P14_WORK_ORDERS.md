# P14 T2 work orders: MotorPWM tests (lead -> writers)

Charter (read it completely, it is the contract): `Tests/claude/charters/driver/PWM_REQUIREMENTS.md` (requirements R1-R6, oracles RM / HAL / RC / NAMES / HD / IF / FW, defect candidates R7, lead decisions L1-L4). Plan: `Tests/claude/plans/P14_P17_REMAINING_DRIVERS_PLAN.md`. The charter, T1 and its fake / fixture are approved by the owner.

## Common to every work order (read first)

- Read `CLAUDE.md`, the driver section of `Firmware/Tests/README.md`, the charter, `Firmware/Tests/support/PwmFixture.h` (T1 fixture), `claude/cases/driver/test_pwm_setup.cpp` (T1 skeleton, shows the fake and fixture in use), the header comment of `platform/src/host_driver_timers.cpp` (what the TIM1 fake models and does not), `support/CommandHelpers.h`, `support/FlashFixture.h` / `support/HostFlash.h` (fake flash access), and the style of `claude/cases/driver/test_tmcdebug_commands.cpp`. Firmware under test: `Firmware/FFBoard/UserExtensions/Src/MotorPWM.cpp` and `Inc/MotorPWM.h` (read-only).
- Specifications: no protocol document exists for this driver. Oracles are the charter's: RM0090 / HAL semantics through the fake, the firmware's own labels (NAMES), hand derivations written in a comment next to the assertion (RC / duty / PSC / ARR / flash word arithmetic with the 168 MHz clock), IF (`MotorDriver.cpp`). Never invent a value; never copy what the firmware currently produces into an assertion unless the test is `characterization:` and the charter says so.
- Create ONLY the test file named in your work order under `Firmware/Tests/claude/cases/driver/`. Do not edit the fixture, CMake, the fake, the firmware, other tests: report needed changes under "Changes needed" (helpers go into an anonymous namespace of your own file).
- Test names: `TEST_CASE_FIXTURE(PwmFixture, "[Rn.m <unit>] <statement that is true if the test passes>")`; suite name given in your work order; one behaviour per test; `characterization: ...` names start with `characterization:` after the id. Every file starts with a comment block: requirements covered, oracles, host-versus-target notes (float `tFreq`, int arithmetic, `int16_t` negation), the characterizations and KNOWN_ISSUEs of the file.
- Markers (`CLAUDE.md`): `KNOWN_ISSUE("B-PWM-<n>: ...")` for a test of the INTENDED behaviour that fails because of a suspected defect (first failing assertion = the defect, nothing that can fail before it); `KNOWN_ISSUE_UB(_HIDDEN)` for undefined behaviour; a crash is `doctest::skip` + "KNOWN ISSUE (crash)". NEVER assert a result you suspect is wrong without a marker. Defects of the charter: B-PWM-1 (RC frame period, R2.4), B-PWM-2 (`freq set` out of range, R4.4, R4.6), B-PWM-3 (`mode set` / stored mode out of range, R4.5, R4.6, R5.4). Use exactly these ids; further findings get the next numbers B-PWM-4... and are reported with evidence.
- A KNOWN_ISSUE whose pass or fail depends on stale heap content is not allowed. Call init unconditionally, never `status && init(...)`.
- Never sleep; there is no thread in `MotorPWM`, so no simulated time is needed. Every test must be order independent (the fixture resets flash, the singleton flag and the target). Keep the whole file well under 2 s in the sanitizer build.
- The TIM1 fake records strict-fake rejections in `HostTarget::state().unmodelledHalCalls`; the fixture fails the test when a test leaves entries behind. A test that provokes a rejection on purpose clears the list at its end (as the T1 skeleton does).
- Build ONLY in `/home/ben/Documents/OpenFFBoard/testing/ffb-test-build/<wo>`, `<wo>-san` (`-DFFB_TESTS_SANITIZE=ON`), `<wo>-clang` (`-DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++`), absolute paths, NEVER inside the repo, only fresh directories (do not delete anything: if a directory exists reuse it). Configure from `Firmware/Tests` as in `Tests/claude/plans/PROGRESS.md` "Environment" with `-DFFB_TESTS_USE_DSP=OFF -DFFB_DRIVER_TEST_FILTER="<your file>"`, target `ffb_driver_tests` only. Run the executable directly; in the sanitizer build skip nothing of your file (report any sanitizer finding).
- `git status --short` must show only your file. Never commit, never push, never write an email address.
- Report: file; tests per requirement (table); characterization list (NEEDS REVIEW); KNOWN_ISSUE list with B-PWM ids, evidence (measured versus intended, arithmetic); not tested and why; "Changes needed"; "Lead decisions needed"; summaries of the three builds (counts, from the doctest result lines or `tools/summarise_results.py`).

| WO | Requirements | File | Suite | Build dir |
|---|---|---|---|---|
| PWO | R2.1-R2.7 (timer set-up), R3.1-R3.11 (`turn` per mode, start / stop) | `test_pwm_output.cpp` | `PWM output` | `pwo` |
| PWC | R4.1-R4.7 (commands), R5.1-R5.5 (persistence) | `test_pwm_config.cpp` | `PWM config` | `pwc` |
| PWL | R1.1-R1.4 (identity, chooser, singleton, defaults), R6.1-R6.3 (life cycle) | `test_pwm_lifecycle.cpp` | `PWM lifecycle` | `pwl` |

## PWO: timer set-up and output (R2, R3)

- Drive the public interface (`setPwmSpeed`, `setMode`, `startMotor`, `stopMotor`, `turn`, `emergencyStop`, `motorReady`) and read the result at the fake TIM1 through the `PwmFixture` view (`arr`, `psc`, `ccr`, `pulseUs`, `duty`, `outputFrequencyHz`, `periodUs`). Hand-derive the expected ARR / PSC / CCR in a comment (charter "Facts").
- R2: table-driven where it keeps one behaviour per test (a parametrised loop over the four speeds is one behaviour: "every speed gives the nominal frequency"); the RC frame period of R2.4 is the `KNOWN_ISSUE B-PWM-1` (tolerance 0.1 %, measured 20.238 ms for LOW etc.; put the measured values of all four speeds in the evidence).
- R3.2: the RC sign is `characterization:`; the limits (1000 / 2000 us, centre 1500 us) are plain with the tolerance of one timer tick; include monotonicity over a sweep of powers (sample every 1024 and the extremes). R3.4-R3.6 use the one-count tolerance of the charter; R3.7 compares each mode against itself with the opposite sign; R3.8 is `characterization:`; R3.9 checks the unused channels stay 0 (record the CCR of all four channels after every mode / power); R3.10 start / stop neutral values (the "keeps running" part is `characterization:`); R3.11 the interface contract.
- Make sure each mode is exercised after a mode switch from every other mode (no stale CCR on channels the new mode does not use: R3.9, R2.5).

## PWC: commands and persistence (R4, R5)

- Run commands through `CommandHelpers.h` / the registered command list, resolve ids by name (`freq`, `mode`, `dir`), never by enum numbers. Check replies, the timer effect (R2 values) and `MotorPWM::getMode()` / `getPwmSpeed()`.
- R4.2 / R4.3: exact info lines (`3khz:0\n`, `20ms:0\n` ...). R4.4-R4.6: the KNOWN_ISSUE group (B-PWM-2, B-PWM-3): one test per behaviour (status `ERR`, value unchanged, no truncation to uint8); the first failing assertion names the defect; a test about `mode set 4` must also check what happens to the output afterwards (frozen) as a SEPARATE plain or characterization test only if the intended behaviour is not already covered.
- R5: write the flash word through `saveFlash()` and read it back at the fake flash (address `ADR_PWM_MODE`, see `HostFlash.h` / `FlashFixture.h` for how the D1 and P4 tests read the fake EEPROM), hand-derive the words (R5.1: mode 2, speed 3, invert on = 0x8032). Preload a word, construct, check state (R5.2 round trip of all 32 combinations is one behaviour). R5.4 / R5.5 as in the charter (`characterization:` for the ignored speed and bit 3, KNOWN_ISSUE B-PWM-3 for the stored invalid mode).

## PWL: identity, singleton, life cycle (R1, R6)

- R1.1: `getInfo()`, `getClassType()`, `getHelpstring()`; R1.2: `MotorDriver::all_drivers` (`MotorDriver.h`) and the Axis list (`Axis.h` / `Axis.cpp`: see how existing tests such as `claude/cases_target/test_axis_*` reach the list; if it cannot be reached through a public interface, test only `MotorDriver::all_drivers` and report) contain id 4 and create a working object; R1.3: `isCreatable` across create / destroy and a second creation through the chooser while the first exists; R1.4: defaults.
- R6.1: destruction effects at the fake (all four CCxE cleared, MOE and CEN cleared, in-use flag false); R6.2: create / destroy / create twice leaves one consistent set-up; R6.3: `characterization:` two instances created with `new` directly (the constructor does not enforce the singleton): record what the fake registers show, and delete both without a leak (the sanitizer build reports leaks).

## Lead decisions on the open points

- A helper you need in the fixture: put it in an anonymous namespace of your own file and list it under "Changes needed" (the lead merges into `PwmFixture.h` after the wave, so the three writers never edit the same file).
- If a requirement cannot be tested through the public interface, do not reach into private members and do not edit the firmware: list it under "Lead decisions needed".
- The decisions L1-L4 of the charter stand; owner questions 32-35 are open: tests of those items are `characterization:` or `KNOWN_ISSUE` as the charter states, whatever the owner later decides.
