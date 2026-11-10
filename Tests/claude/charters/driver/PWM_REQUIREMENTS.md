# PWM driver (`MotorPWM`): requirements charter (phase P14, T0)

Status: **DRAFT by the lead (Sonnet 5.5), 2026-11-10, waiting for the owner's approval of T0.** Nothing is built or tested yet.

Unit under test: `Firmware/FFBoard/UserExtensions/Src/MotorPWM.cpp` (332 lines, `MP` below) and `Inc/MotorPWM.h` (107 lines, `MH`). Class `MotorPWM : MotorDriver, CommandHandler, PersistentStorage`, behind the macro `PWMDRIVER`, class id `CLSID_MOT_PWM` 0x84, registered as motor driver 4 in `MotorDriver.cpp:35` and `Axis.cpp:56 / :83`.
Executable: `ffb_driver_tests` (owner's decision, question 28). Plan: `Tests/claude/plans/P14_P17_REMAINING_DRIVERS_PLAN.md`. Tests: `Firmware/Tests/claude/cases/driver/test_pwm_*.cpp`, fixture `support/PwmFixture.h`.

## Sources of expected values (oracles)

| Code | Source |
|---|---|
| RM | RM0090 (`Tests/shared/specs/dm00031020-...pdf`, never committed): the advanced-control timer TIM1 chapter, only for register semantics the fake models (ARR / PSC / CCRx / CCER / BDTR.MOE / CR1.CEN, PWM mode 1, preload bits). Period = (PSC+1)(ARR+1) / f_timer. |
| HAL | ST HAL `stm32f4xx_hal_tim.c` (the copy in the repository, `Firmware/Targets/F411RE/Drivers/...`): `HAL_TIM_PWM_Start` returns HAL_ERROR unless the channel state is READY, enables CCx, MOE (break instances) and the counter; `HAL_TIM_PWM_Stop` disables CCx, MOE and the counter only when no other channel is enabled (macros `__HAL_TIM_MOE_DISABLE`, `__HAL_TIM_DISABLE`), sets READY; `HAL_TIM_PWM_ConfigChannel` loads CCRx from `Pulse` and sets the CCRx preload bit. |
| RC | The classic RC servo / ESC signal as described in the firmware comment (`MP:66-69`): pulse 1000 to 2000 us, centre 1500 us, frame period named 20 / 15 / 10 / 5 ms (`RC_SpeedNames`). There is no supplied RC document: the pulse limits are plain tests because the code states them and they are the universal RC convention; the sign of the mapping is NOT stated anywhere (R4.2: `characterization:`). |
| NAMES | The firmware's own labels: `PWM_SpeedNames` "3khz", "9khz", "17khz", "24khz"; `PwmModeNames` "RC PPM", "0%-50%-100% Centered", "0-100% PWM/DIR", "0-100% dual PWM". The labels are the stated intent for frequency and duty. |
| HD | Hand derivation in a comment next to the assertion (PSC, ARR, CCR values, flash word bits). |
| IF | The `MotorDriver` interface (`MotorDriver.h`, `MotorDriver.cpp:46-60`): `emergencyStop(false)` = `stopMotor()`, `emergencyStop(true)` = `startMotor()`, `motorReady()` true. |
| FW | The firmware's own behaviour where nothing above decides it. Test name starts with `characterization:` and the case gets a NEEDS REVIEW entry. |

Rules inherited from `CLAUDE.md` and the D1-D3 / P13 charters: fixture per test, simulated time only, never copy a library value into an assertion without `characterization:`, a suspected defect is a `KNOWN_ISSUE("...")` test of the INTENDED behaviour whose first failing assertion is the defect, tests go through public interfaces (`turn`, `startMotor`, command handler, the timer registers at the fake, the fake flash).

## Facts every test author needs (read from the source by the lead)

* **Constants.** `timerConfig.timerFreq = 168000000` (`MH:45`), channels from `Targets/F407VG/Core/Src/cpp_target_config.cpp:62` (pwm 1, dir 3 / n 4, centre 1, rc 1, dual 1 / 2, timer `TIM_PWM` = `htim1`). TIM1 as left by MX_TIM1_Init: PSC 0, ARR 0xffff, up-counting, auto-reload preload ENABLED.
* **Construction** (`MP:35-46`): `pwmDriverInUse = true`, `restoreFlash()`, `setPwmSpeed(pwmspeed)`, `registerCommands()`, three commands `freq` (GET|SET|INFOSTRING), `mode` (GET|SET|INFOSTRING), `dir` (GET|SET). Defaults: mode `CENTERED_PWM` (1), speed `LOW` (0), `invertDir` false, `active` false.
* **Destruction** (`MP:48-55`): `pwmDriverInUse = false`, `HAL_TIM_PWM_Stop` on channels 1-4, `HAL_TIM_Base_Stop_IT`.
* **`setPwmSpeed`** (`MP:123-179`): per speed (LOW / MID / HIGH / VERYHIGH): RC mode period 40000 / 30000 / 20000 / 10000 with prescaler `timerFreq / 2000000` = 84; other modes `freqToPeriodPsc(3000 / 9000 / 17000 / 24000)` = ARR `timerFreq/f / (psc+1)` with `psc = (timerFreq/f) / 0xffff`. An unknown speed is ignored silently. On success: `pwmspeed` stored, `tFreq = (timerFreq/1000000) / (prescaler+1)` (ticks per us, float), `pwmInitTimer` for the four channels (ARR, PSC written, channel stopped, PWM mode 1 configured with Pulse 0, started, under `pwmTimMutex`), `HAL_TIM_MspPostInit`, `turn(0)`.
* **`turn`** (`MP:59-108`): returns at once when `!active`; `invertDir` negates; then per mode:
  * RC: `val = ((p*1000)/0x7fff) * tFreq` (integer division first, then float), `clip(1500*tFreq - val, 1000*tFreq, 2000*tFreq)`, written to CCR `rcpwm_chan`. Positive power gives a pulse SHORTER than 1500 us.
  * PWM_DIR: p < 0: dir CCR 0 and dir_n CCR 0xffff; else the reverse; `pwm_chan` CCR = `|p| * period / 0x7fff` (integer).
  * CENTERED: `pval = 0x7fff + p`, CCR = `pval * period / 0xffff` (integer, truncation toward zero).
  * DUAL: CCR = `|p| * period / 0x7fff`; p < 0 -> dual2 = value and dual1 = 0, else dual1 = value and dual2 = 0.
* **Flash** (`MP:204-223`, address `ADR_PWM_MODE`): save = `(mode & 7) | ((speed & 7) << 4) | (invertDir << 15)`; restore only when `Flash_Read` succeeds: mode = `var & 0xf` (note: save masks 3 bits, restore 4) through `setMode`, speed = `(var >> 4) & 7` through `setPwmSpeed`, invert = bit 15. `setMode` stores any value and re-runs `setPwmSpeed`.
* **Commands** (`MP:248-292`): `freq` set -> `setPwmSpeed((SpeedPWM_DRV) val)`, get -> `(uint8_t) speed`, info -> four lines "name:index\n" (RC names when mode is RC, else kHz names); `mode` set -> `setMode((ModePWM_DRV) val)` (no range check), get, info -> four lines; `dir` -> `handleGetSet(invertDir)`; unknown id -> `NOT_FOUND`; every other path returns `OK`.
* **Singleton.** `isCreatable()` is `!pwmDriverInUse`; the flag is a static, so the fixture must reset it (`MotorPWM::pwmDriverInUse = false`) before and after every test.

## Requirements

### R1: identity, chooser and singleton
| ID | Requirement | Oracle |
|---|---|---|
| R1.1 | `getInfo()` is name "PWM", id 0x84 (`CLSID_MOT_PWM`); `getClassType()` is `Motordriver`; `getHelpstring()` is the fixed text | FW (identity values are plain facts of `ClassIDs.h`, test plain) |
| R1.2 | With `PWMDRIVER` on, `MotorDriver::all_drivers` and the Axis driver list contain the class under id 4 and `createInstance` builds a working object | FW, plain (chooser contract) |
| R1.3 | `isCreatable()` is true when none exists, false while one exists, true again after destruction; a second object cannot be created through the chooser while the first exists | FW, plain |
| R1.4 | Default state after construction on an empty flash: mode CENTERED (1), speed LOW (0), direction not inverted, driver not active | HD from the header defaults |

### R2: timer set-up (what reaches the TIM1 fake)
| ID | Requirement | Oracle |
|---|---|---|
| R2.1 | After construction in the default state the timer holds ARR 56000 and PSC 0 (3 kHz on 168 MHz: `168000000/3000 = 56000`), all four channels are PWM mode 1, enabled, with CCR 0 or the zero-power value, CEN and MOE set | HD + HAL + RM |
| R2.2 | Every speed in every non-RC mode: LOW 56000, MID 18666 (168e6/9000 = 18666.67), HIGH 9882 (168e6/17000 = 9882.35), VERYHIGH 7000, PSC 0; the resulting output frequency (168e6 / ((PSC+1)(ARR+1))) is within 0.1 % of 3 / 9 / 17 / 24 kHz | NAMES + HD |
| R2.3 | RC mode: ARR 40000 / 30000 / 20000 / 10000, PSC 84 | HD (as coded; the frame time is R2.4) |
| R2.4 | RC frame time of the four speeds is 20 / 15 / 10 / 5 ms within 0.1 % (`PSC+1` divider, `(PSC+1)(ARR+1)/168e6`). Hand derivation: PSC 84 divides by 85, 40000 ticks take 20.24 ms, not 20 ms (`timerFreq / 2000000` is the divider, the register wants divider minus one) | NAMES, `KNOWN_ISSUE` (B-PWM-1), owner question |
| R2.5 | Changing the mode re-initialises the timer (period / prescaler of the new mode, channels re-configured, `turn(0)` outputs) | FW + HAL |
| R2.6 | Every channel re-initialisation follows Stop, ConfigChannel, Start in this order and the fake accepts it (a Start on a channel that was not stopped is rejected by the real HAL) | HAL (fake strictness) |
| R2.7 | `freqToPeriodPsc` never returns an ARR above 0xffff for the four frequencies and with `timerFreq` of 168 MHz (`uint16_t` truncation check); informational for other clocks | HD |

### R3: `turn` per mode
| ID | Requirement | Oracle |
|---|---|---|
| R3.1 | `turn` before `startMotor` (and after `stopMotor`) writes nothing to the timer | IF + FW (code gate); plain |
| R3.2 | RC: power +32767 / 0 / -32767 give pulses of 1000 / 1500 / 2000 us within one timer tick (`1 / tFreq` us, 0.51 us at speed LOW); intermediate values are monotonic and within one tick plus the integer-division step of the value; the pulse is clipped to 1000..2000 us for -32768 | RC + HD; the SIGN (positive power -> short pulse) is `characterization:` (no document), NEEDS REVIEW |
| R3.3 | RC: pulse width is independent of the speed setting (same microseconds for the four frame times) | RC + HD |
| R3.4 | CENTERED: power 0 gives a duty of 50 % within one count (CCR = floor(32767 x ARR / 65535)), +32767 gives (65534 / 65535) of ARR, -32767 gives (1 / 65535) of ARR, -32768 gives 0; duty is monotonic in the power | NAMES + HD |
| R3.5 | PWM_DIR: for power >= 0 the direction pin pair is (dir = always high, dir_n = 0), for power < 0 it is (dir = 0, dir_n = always high); duty = `|p| / 32767` of ARR; power 0 gives duty 0; -32768 gives a duty >= 100 % (CCR above ARR, no wrap) | NAMES + HD |
| R3.6 | DUAL: power > 0: channel 1 carries `|p| / 32767` of ARR and channel 2 is 0; power < 0: the reverse; 0: both 0 | NAMES + HD |
| R3.7 | Direction inversion swaps the sign of every mode above (equal CCR values to the opposite sign) for all powers except -32768 (R3.8) | FW (`dir` command exists to do exactly this); plain |
| R3.8 | With inversion on, power -32768: `-power` on an `int16_t` is 32768, narrowed to -32768 again | `characterization:` (implementation-defined narrowing, same on ARM GCC); NEEDS REVIEW |
| R3.9 | Outputs of channels a mode does not use stay at CCR 0 (no stray pulse) | HD |
| R3.10 | `startMotor` activates and outputs the neutral (power 0) value of the current mode; `stopMotor` outputs the neutral value, then ignores `turn`; the timer keeps running after `stopMotor` (the signal is not switched off) | IF + FW; the "keeps running" part is `characterization:` (a safety choice for an ESC, owner question) |
| R3.11 | `emergencyStop(false)` behaves as `stopMotor`, `emergencyStop(true)` as `startMotor`; `motorReady()` is true | IF, plain |

### R4: commands
| ID | Requirement | Oracle |
|---|---|---|
| R4.1 | `freq` get replies the speed index; set 0..3 changes the speed (and R2 timer values); `mode` get / set likewise; `dir` get / set toggles inversion | FW, plain |
| R4.2 | `info` of `freq`: four lines `3khz:0` ... `24khz:3` (non-RC) or `20ms:0` ... `5ms:3` (RC mode) | NAMES, plain |
| R4.3 | `info` of `mode`: four lines `RC PPM:0`, `0%-50%-100% Centered:1`, `0-100% PWM/DIR:2`, `0-100% dual PWM:3` | NAMES, plain |
| R4.4 | `freq set` with a value above 3 is rejected: reply status `ERR` and speed unchanged. Today the value is ignored and the status is `OK` | `KNOWN_ISSUE` (B-PWM-2), intended behaviour is the stated one |
| R4.5 | `mode set` with a value above 3 is rejected: reply `ERR`, mode unchanged and output still valid. Today `mode` stores the invalid value, `turn` then writes nothing (output frozen at the last value) and `mode get` returns it | `KNOWN_ISSUE` (B-PWM-3) |
| R4.6 | A negative or huge `cmd.val` for `freq` / `mode` (int64 truncated to `uint8_t`, e.g. 256 becomes speed 0) | `KNOWN_ISSUE` together with R4.4 / R4.5 (the same missing range check): one case each, first failing assertion names the truncation |
| R4.7 | Unknown command id returns `NOT_FOUND` | FW, plain |

### R5: persistence
| ID | Requirement | Oracle |
|---|---|---|
| R5.1 | `saveFlash` writes one 16-bit word at `ADR_PWM_MODE`: bit layout `mode (bits 0-2) | speed (bits 4-6) | inversion (bit 15)`, e.g. (mode 2, speed 3, invert on) = 0x8032 | HD |
| R5.2 | Round trip: all 4 x 4 x 2 combinations survive save, destroy, construct (the timer then holds the matching period) | HD |
| R5.3 | Empty flash: defaults of R1.4 and no flash write at construction | FW, plain |
| R5.4 | A stored word with an invalid speed (4..7) leaves the previous speed (LOW); a stored mode above 3 is accepted into `mode` (same missing check as R4.5) | `KNOWN_ISSUE` for the mode part (B-PWM-3), `characterization:` for the ignored speed |
| R5.5 | Restore masks the mode with 0xf while save writes 3 bits: bit 3 of a stored word changes the mode (word 0x0009 gives mode 9) | `characterization:` (the header comment "1 bit free" suggests bit 3 is reserved) |

### R6: lifecycle
| ID | Requirement | Oracle |
|---|---|---|
| R6.1 | Destruction stops all four channels, CCER bits clear, MOE and CEN cleared, and `pwmDriverInUse` false | HAL + FW |
| R6.2 | Create, destroy, create again works and leaves one consistent timer set-up (no state leak between instances other than the flash content) | FW |
| R6.3 | Two instances created with `new` directly (not through the chooser) do not break the register state asserted by the fake (informational: the singleton flag is not enforced by the constructor) | `characterization:` |

### R7: defect candidates (from reading the source)

Items 1-3 are `KNOWN_ISSUE` (the owner may later change the decision); items 4-6 become a test only after the writer shows the library result contradicts the oracle.

1. B-PWM-1: RC frame period is 1.2 % longer than named (R2.4): the prescaler register is divider minus one. Pulse widths are right because `tFreq` also uses `prescaler + 1`.
2. B-PWM-2 / B-PWM-3: no range check for `freq` set and `mode` set (R4.4 to R4.6, R5.4); a stored invalid mode freezes the output.
3. Duty never reaches exactly 100 % with CCR = ARR (one count low) and the PWM_DIR / DUAL / CENTERED full-scale duty is 99.998 %: informational, the tolerance in R3.4 to R3.6 is one count.
4. `HAL_TIM_Base_Start_IT` is commented out in the constructor (`MP:39`): the counter is started by `HAL_TIM_PWM_Start`; verified by R2.1 (CEN set), not a defect.
5. `setPWM_HAL` and `pwmInitTimer` are free functions in the header: `setPWM_HAL` is never called (dead code, no test) and has an uninitialised `sConfigOC` fields (`OCNPolarity`, `OCNIdleState`, `OCIdleState` partly set): informational.
6. Constructor calls the virtual `restoreFlash()` from the base `PersistentStorage` interface: safe here (no further derived class). Informational.

## Fake: TIM1 PWM (T1, owner approved question 31)

New strict fake of TIM1 with PWM: `htim1` and its register block in `platform/src/host_driver_timers.cpp` (ffb_driver_tests only, like `htim6`), plus the PWM HAL calls (`HAL_TIM_PWM_ConfigChannel`, `HAL_TIM_PWM_Start`, `HAL_TIM_PWM_Stop`, `HAL_TIM_MspPostInit`, `TIM_OC_InitTypeDef`, channel states, `__HAL_TIM_MOE_*`) and the definition of `MotorPWM::timerConfig` (copy of the F407VG `cpp_target_config.cpp:62` values). Needs `TIM_TypeDef` fields CCMR1 / CCMR2 / CCR1-4 / BDTR in `platform/target_f407/main.h` (shared header, aggregate fields appended, behaviour of every other executable unchanged; `platform/include/main.h` of `ffb_unit_tests` is not touched).
The fake header comment follows the fidelity rules: models (register writes, channel state machine, `Start` rejected unless READY, `Stop` disabling CCx and, when no channel remains enabled, MOE and CEN, ConfigChannel loading CCRx from `Pulse`, OCxPE set, invalid OC mode / channel rejected as a test failure), source (RM0090 TIM1 chapter, `stm32f4xx_hal_tim.c`), does not model (the counter, real pin levels, the shadow registers: ARR / PSC / CCR writes are visible at once instead of at the next update event, dead time, break input, DMA, the MSP GPIO set-up), confidence (High for the HAL call contracts, Medium for the timing semantics which are not simulated). Helper in `HostTarget.h` style: `pwmChannel(n)` returning CCR, enabled, mode; `pwmPeriodUs()`, `pwmPulseUs(n)` computed from ARR / PSC / CCR and the 168 MHz clock.

## Fixture and test layout (T1 / T2)

* `PwmFixture : TargetFixture` in the new `support/PwmFixture.h` (no edit of existing fixtures): clears the fake flash, resets `MotorPWM::pwmDriverInUse` before and after, builds the object through a helper (`makePwm()`), gives the timer view above, and fails the test on a strict-fake violation not declared with `expectViolation()`.
* Files: `test_pwm_setup.cpp` (R1, R2.1, defaults), `test_pwm_output.cpp` (R2, R3), `test_pwm_config.cpp` (R4, R5), `test_pwm_lifecycle.cpp` (R6, plus the T1 skeleton case that checks the macro is on).
* Build wiring (T1): `PWMDRIVER` added to `platform/target_f407_drivers/target_constants.h` ONLY and the header comment / `test_driver_setup.cpp` expectation updated (macro now expected on); `MotorPWM.cpp` added to `DRIVER_FIRMWARE_SOURCES`; the fake as above. Side effect to check in T1: the macro adds `MotorPWM` to `MotorDriver::all_drivers` and the Axis driver list (id 4); any existing driver-track test that lists or counts the drivers must be found and adjusted by a NEW case, never by editing the baseline. The other five executables keep identical compile commands (compare with the scratch-worktree method of P13 T1).
* Counts to aim at (writers): R2 about 20, R3 about 30, R4 about 12, R5 about 10, R6 about 8; not a target to pad.

## Target differences to flag

Host 64-bit, target 32-bit ARM with a single-precision FPU: `tFreq` and the RC CCR are `float` (both SSE and VFP single precision, same rounding for these few operations; R3.2 uses a one-tick tolerance); `(abs(power) * period)` is `int` arithmetic: the largest product 32768 x 56000 = 1.84e9 fits `int32_t` on both; `-power` narrowing (R3.8) is implementation-defined and wraps on GCC for both; float to `uint32_t` conversion of the clipped (always non-negative) RC value is in range.

## Out of scope for P14

The real timer counter and pin levels (no simulation of waveforms), ESC behaviour, the board's gate-driver hardware, the Axis integration beyond the chooser (Axis tests use `MockMotorDriver`).

## Decisions (lead, Sonnet 5.5) and owner questions

* L1: RC pulse limits and neutral are plain tests (code states them, universal convention); the mapping SIGN and the "signal keeps running after stop" are `characterization:`.
* L2: the 1.2 % RC frame-period error is a `KNOWN_ISSUE` (the names state 20 / 15 / 10 / 5 ms, tolerance 0.1 %).
* L3: missing range checks for `freq` / `mode` are `KNOWN_ISSUE` (an invalid mode freezing the output is unsafe for an actuator).
* L4: shadow-register timing is not modelled (register writes take effect at once in the fake); stated in the fake header and in the report.
* Owner questions to ask at approval (to become numbers 32 and up): (a) is a 1.2 % RC frame-period error acceptable (ESCs tolerate 50 Hz +/- a few percent) or a defect? (b) should `stopMotor` keep the neutral signal running (RC ESC arm state) or switch the outputs off? (c) is RC "positive power = shorter pulse" intended? (d) bit 3 of the flash word: reserved or part of the mode?
