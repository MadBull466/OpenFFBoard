# Integration track: progress

> Resume point of the integration track. Read "Current state" and "Next action" first.
> Plan: `INTEGRATION_TEST_PLAN.md`; agents, hooks, gate: `AGENT_PLAN.md`; rules: root `CLAUDE.md`. Every number of this track lives here and nowhere else.

## Current state

*Last updated: 2026-11-18.* Branch `add-test-suite`. Coverage-suite state at the start: `Tests/Code_Coverage/claude/plans/PROGRESS.md` "Current state" (commit `b3731f63`).

- **Phase:** I0 (infrastructure), started 2026-11-18. **I0-1 done** (commits `084835fc`, `c6f815c4`, pushed): five integration agents, hooks (`.claude/settings.json`, `.claude/hooks/`), hook tests `Firmware/Tests/Integration_Tests/tools/test_hooks.py` (31 tests, OK), `CLAUDE.md` sections, `APPROVED_SHARED_EDITS.txt`. All block rules checked live, including a subagent (`agent_type` reaches the hooks) and the `integration-test-writer` stop hook (blocked a stop without a gate line, let the second stop through with `stop_hook_active`); the writer's hand-back report can be sent before the block, so the lead re-checks the gate line. `MultiEdit` / `NotebookEdit` field names: unit tests only. **I0-2 done** (uncommitted): four audits, reports in `../charters/audit/`; owner decided fix A-1, A-2, A-3 and IQ5 (keep ENABLE_IN high). **I0-3 done** (owner-approved 2026-11-18): fake fixes A-1, A-2, A-3, reviewed; final coverage gate `../ffb-test-build/i03/after-I0-3fix-<variant>.*` (default, m32, san) green, no existing case changed its result. **I0-4 done**: `ffb_integration_tests` (`Firmware/Tests/Integration_Tests/integration.cmake`, one `include()` line in `Firmware/Tests/CMakeLists.txt`, option `FFB_TESTS_INTEGRATION` default ON, `TMCDEBUG` off through a generated wrapper header), ctest `integration_tool_tests` (gate and hook tests), smoke case, `it_gate.py` (82 unit tests; reviewed: 4 major gaps fixed, e.g. hidden warnings, untested pass/fail logic, stale build dirs, reduced runs now `PARTIAL`). Gate `--wo I0-4 --variants default,m32`: PASS (I0-3 and I0-4 committed together, `096f43d3`). **I0-5 done**: `IntegrationFixture` (reset in plan order, board, bring-up through `cppmain`, enumeration, `axis.0.drvtype=1` and a USB re-plug (INR 1), resumable threads, teardown fault and exact leak checks also after the main class is deleted) and `ChainScheduler` (fixed per-ms order, TIM13 / TIM6 from the firmware-written registers, ADC each ms, TMC thread counted by its handle); reviewed (3 major findings fixed and proven by mutation); new register entries IB 1 (FFB timer 1001 us), IB 2 (TMC encoder timer 257 us), IB 3 (HID command interface use after free, found by ASan), INR 1 (motor start after runtime driver selection).
- **Tests:** `ffb_integration_tests`: 10 cases in `test_f1_setup.cpp`: 7 passed, 3 known issues (IB 1, IB 2, IB 3 as `KNOWN_ISSUE_UB`), 0 failed in default and m32; sanitizer build (not yet a gate variant): clean except IB 3, which aborts with ASan when run with `--no-skip`. Bring-up takes 1181 ms simulated (1 us per SPI frame during bring-up).
- **Requirements (F1 charter):** 62 requirements (A 5, H 6, E 14, R 7, X 7, T 9, P 2, S 4, O 5, B 3; P1 and P2 split in the charter) plus 12 candidate seams S1-S12 (not requirements); 2 covered (F1-A1, F1-A4).
- **Gate:** `python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --wo <ID> --variants default,m32` (file list `../charters/wo/<ID>.files`); compile references recorded for default and m32 only; `--full` needs references for the other variants first (recorded from builds without the integration executable).
- **Registers:** 3 integration bugs (IB 1-3), 1 INR item (INR 1). **Counters:** next IB 4, next INR 2, next IQ 8 (IQ1-IQ5 answered; IQ6, IQ7 open).
- **Roles:** lead Opus 5.5 (high); workers per `AGENT_PLAN.md`.

## Next action

0. Owner: IQ6 (INR 1, motor start after a runtime driver selection) and IQ7 (coverage tool `run_all_tests.py` misreports three VESC cases) in `../reports/OPEN_QUESTIONS.md`. All faults so far: `../reports/BUG_REPORT.md` (hand-maintained until I0-8).
1. WO I0-6: `HidReportDescriptor` parser and `PidReportBuilder` from the parsed descriptor and PID 1.01 (F1-A2).
2. Then I0-7 (`WheelPlant` applying torque mode / ENI / 0x5E and advancing 0x6B by differences; detector; FRF), I0-8 (tools, incl. `make_integration_bug_report.py`, which then generates `../reports/BUG_REPORT.md` sections 1-2 and the hooks protect it again), I0-9 (phase gate `--full`, after recording the remaining compile references).
3. Coverage-tool observation: now tool defect T-1 in `../reports/BUG_REPORT.md`, question IQ7.

## Phase checklist

| Phase | Content | Status | Approved by the owner? | Commit |
|---|---|---|---|---|
| Planning | plan, F1 charter, agent plan, documents | done | yes, 2026-11-18 | |
| I0 | infrastructure, fake audit, agents / hooks / gate | in progress (I0-1 to I0-5 done) | | |
| I1 | open-loop chain (G-H, G-E, G-X) | not started | | |
| I2 | driver in the chain, persistence (G-T, G-P) | not started | | |
| I3 | DSP, reconstruction, timing (G-R, G-S) | not started | | |
| I4 | stability (G-O) | not started | | |
| I5 | F1 close-out | not started | | |
| I6-I10 | F2 ODrive, F3 VESC, F4 PWM, F5 SimpleMotion, F6 MyActuator | not started | | |

## Decisions log

| Date | Decision | Reason |
|---|---|---|
| 2026-11-18 | Integration track planned (owner's task): test code in `Firmware/Tests/Integration_Tests/`, documents in `Tests/Integration_Tests/`, new executable(s), shared code reused from `Code_Coverage/` | Owner's decisions in the task |
| 2026-11-18 | IQ1: own register (IB n, INR n) and own generated report `BUG_REPORT.md`; coverage bugs hit again are cross-referenced, the coverage register is not changed | Owner's answer |
| 2026-11-18 | IQ2: oracle for the HID entry = `USB_PID1_01.pdf` (supplied); `USB_PID1_02.pdf` only to flag differences | Owner's answer; coverage decisions log 2026-10-23 names PID 1.01 |
| 2026-11-18 | IQ3: nominal wheel plant with documented ranges plus setting sweeps; no hardware data | Owner's answer |
| 2026-11-18 | IQ4: existing functional fakes (audited); no real ST HAL in this track | Owner's answer |
| 2026-11-18 | Lead: one executable `ffb_integration_tests` (feature header `target_f407_drivers`, real TinyUSB, MIDI off) for all flows, defined in `Integration_Tests/integration.cmake` and included from `Firmware/Tests/CMakeLists.txt` | Exploration: `ffb_driver_tests` already compiles the real `Axis` + `TMC4671`; real TinyUSB needs `HOSTTEST_RTOS_EXT` / `HOSTTEST_REAL_TINYUSB` as in `ffb_usb_tests`; one binary avoids drifting source lists |
| 2026-11-18 | Lead: the integration host builds PID reports from the parsed report descriptor, not from the firmware's C structs | `PidHost` uses the firmware structs, so a struct / descriptor mismatch would be invisible |
| 2026-11-18 | Lead: stability = analytic loop model as oracle (stability analyst) + conformance of the real chain to it + closed-loop runs with a plant attached to `FakeTmc4671`; instability inside the settable range = INR item for the owner | Plan section 6 |
| 2026-11-18 | Lead (I0-1): gate evidence records each file of the WO with its git blob hash (`files`) and a fingerprint over them; the commit check compares the staged blobs with it, so unrelated dirty files (Codex's, `AGENTS.md`) neither block nor get committed. Staged `codex/` paths and `AGENTS.md` are always refused. Configuration commits (`.claude/`, `CLAUDE.md`, the hook tests) need no gate but run `test_hooks.py`. `git stash list/show` allowed for subagents (probe false positive) | Implementation of `AGENT_PLAN.md` 2.2 / 2.3; the planned whole-diff fingerprint would have been blocked by the pre-existing changes outside the track |
| 2026-11-18 | Owner: record all faults in a bug report and all questions in the open-questions file; lead: `reports/BUG_REPORT.md` (name like the coverage track's, replacing the planned `INTEGRATION_BUG_REPORT.md`), hand-maintained until the I0-8 generator, hook protection of it dropped until then; IQ6 (INR 1) and IQ7 (tool defect T-1) asked | Owner's request |
| 2026-11-18 | Lead (I0-5): bring-up by `axis.0.drvtype=1` plus a USB re-plug ("board configured, PC connects afterwards"; INR 1); 1 us per SPI frame during bring-up (`calibrateAdcOffset` busy-waits on `HAL_GetTick`), 0 afterwards; the fixture drains HID IN like a polling host (IB 3 has its own non-draining case); IB 1-3 registered | WO I0-5 findings and review |
| 2026-11-18 | Owner: `integration-reviewer` `maxTurns` 50 -> 100 | Reviews of I0-3 to I0-5 ran out of turns |
| 2026-11-18 | Lead (I0-4 review): gate evidence records `variants_requested` and the file list; a run without both default and m32 is `PARTIAL` (the hooks accept only `PASS`); warnings of WO sources are re-checked with `-fsyntax-only`; build dirs are checked against the variant's cache values | Reviewer showed four ways the first gate could print PASS wrongly |
| 2026-11-18 | Owner approved the I0-3 coverage changes (A-1, A-2, A-3 fixes, 4 edited and 9 new coverage cases); they are committed with I0-4's gate evidence; I0-4 started | Owner's answer "Approve, start I0-4" |
| 2026-11-18 | Lead (I0-3 review): under `HOSTTEST_RTOS_EXT` a restart-model task that delays is a kernel fault (no approximation); every delaying thread in an ext executable must run resumable; minor review findings fixed in the same round | Reviewer probes: the approximation ran work twice per period and livelocked on two delays, the patterns `TMC4671::Run` uses; resumable tasks were modelled correctly |
| 2026-11-18 | Owner (I0-2): fix A-1 (spiFail OVR on transmit), A-2 (Flash_Dump list) and A-3 (vTaskDelay under `HOSTTEST_RTOS_EXT` plus highest-priority-first in `runUntilIdle`); IQ5 keep the ENABLE_IN-high assumption; `fake-auditor` `maxTurns` 60 -> 100 | Audit reports in `../charters/audit/`; all four audits hit the 60-turn limit |
| 2026-11-18 | Owner approved the plan, the F1 charter, `AGENT_PLAN.md` (agents, hooks, gate) and the `CLAUDE.md` text; phase I0 started | Owner's answer "Approve, start I0" |

## Integration bugs

| # | Module | Location | Description | Same as (coverage) | Evidence (test) | Resolved? |
|---|---|---|---|---|---|---|
| IB 1 | FFBHIDMain | `FFBoard/UserExtensions/Src/FFBHIDMain.cpp:256` | The FFB timer TIM13 gets `ARR = 1000000*basediv/basefreq` ("Assumes 1µs timer steps", PSC 83 = 1 MHz count); an up-counter's period is ARR+1 counts (RM0090 §18.3.1), so the default rate is 1001 us (999 Hz) while `updateSamplerate` is told 1000 Hz: 1000 FFB cycles in 1001 ms instead of 1001 | similar pattern to bug 201 (MotorPWM PSC), different location | `F1-A4` 1001 ms case (`KNOWN_ISSUE`, WO I0-5) | no |
| IB 2 | TMC4671 | `FFBoard/UserExtensions/Src/TMC4671.cpp:3156-3158`, `target_constants.h:72, 96` | The external-encoder timer TIM6 gets PSC = `(SystemCoreClock/2)/1000000 + 1` = 85 ("1µs ticks": would need 84 MHz/1 MHz - 1 = 83) and ARR = 250 ("250 = 4khz"); with the up-counter's (PSC+1)(ARR+1) clocks the period is 21586 clocks = 257 us (3891 Hz) instead of 250 us (4 kHz) | similar pattern to bug 201 and IB 1 | TIM6 `KNOWN_ISSUE` case in `test_f1_setup.cpp` (reached through `tmc.0.combineEncoder`, `axis.0.enctype`, `tmc.0.encsrc`; WO I0-5) | no |
| IB 3 | HID command interface | `FFBoard/Src/HidCommandInterface.cpp:39-46`, `:159` | `HID_CommandInterface::Run` iterates `outBuffer` by reference and waits in `Delay(1)` for the HID IN endpoint inside the loop; the command thread (higher priority on the target) appends replies through `queueReplyValues` and the `push_back` can reallocate the vector, so the loop reads freed memory (ASan heap-use-after-free in `tud_hid_n_report` via `sendHidCmd`, found in WO I0-5) | | `KNOWN_ISSUE_UB` case in `test_f1_setup.cpp` (WO I0-5) | no |

## Integration NEEDS REVIEW items

| # | Module | Test / location | Question | Same as (coverage) | Resolved? |
|---|---|---|---|---|---|
| INR 1 | Axis / TMC4671 | `Axis.cpp:534-543`, `TMC4671.cpp:1682-1687`; characterization case in `test_f1_setup.cpp` (WO I0-5) | Selecting TMC_1 with `axis.0.drvtype` while USB is connected calls `startMotor()` before the driver is initialised; it only sets `motorEnabledRequested`, and when the state machine reaches Shutdown (motor and encoder type NONE on a fresh flash) the request is not acted on: the driver stays in Shutdown. A USB re-mount (`tud_mount_cb` -> `Axis::usbResume` -> `startMotor`) then goes to Running although motor and encoder type are still NONE. Intended: should the runtime driver selection start the motor once set up, and should Running be reachable with motor type NONE? Asked as IQ6 | | no |

## Fake audit register

| Fake | Audit (WO, date) | Result | Fix (WO, owner approval) | Coverage cases changed |
|---|---|---|---|---|
| `FakeTmc4671` | I0-2, 2026-11-18, `../charters/audit/group-a_faketmc4671.md` | no defect; gaps G1-G4 handled in the integration (plant, flags, time per frame 0); ENABLE_IN-high assumption carried by 9 coverage cases: IQ5 | none proposed | none |
| Fake HAL SPI + GPIO / EXTI | I0-2, 2026-11-18, `../charters/audit/group-b_spi_gpio.md` | **A-1 defect**: `spiFail` delivers OVR on a transmit (and MODF / FRE) to `HAL_SPI_ErrorCallback`; the HAL clears OVR in BUSY_TX and returns (`stm32f4xx_hal_spi.c:2411-2422`); gaps G1-G4 handled in the integration | fixed in WO I0-3a (uncommitted, waiting for the owner's commit approval) | 2 cases edited (inject `HAL_SPI_ERROR_FLAG` on a transmit; still pass): `a device without overrides takes the port ... interrupt driven transfer`, `a completion or error is reported to the current device only once`; 2 self-tests added (OVR on a transmit; MODF / FRE) |
| Fake ADC, timers, flash map | I0-2, 2026-11-18, `../charters/audit/group-c_adc_timers_flash.md` | **A-2 defect**: `Flash_Dump` returns every map entry instead of the firmware's address list (`flash_helpers.cpp:268-287`); gaps G1-G3 (timer rates, ADC conversions, reboot keeping flash) handled in the integration | fixed in WO I0-3b (uncommitted, waiting) | 1 case edited and renamed `flashdump returns every stored exportable flash variable as value and address` (exportable `ADR_` addresses; still passes); 2 cases added (calibration address not dumped; `includeAll`); `eeprom_addresses.c` added to `FIRMWARE_SOURCES` / `RTOS_FIRMWARE_SOURCES` |
| `HostDcd` + `UsbHost` + `HostRtos` ext | I0-2, 2026-11-18, `../charters/audit/group-d_usb_rtos.md` | **A-3 gap, integration blocker**: `vTaskDelay` inside a task advances the shared clock by the whole delay (`host_freertos.cpp:457-469`); R2 (no priorities) handled by the fixture or fixed with A-3; U1 minor | fixed in WO I0-3c (uncommitted, waiting): under `HOSTTEST_RTOS_EXT` a task's delay waits for simulated time, a delay in a restart-model task is a kernel fault, `runUntilIdle` runs the highest priority first | 1 case edited (`a command reply waits while a long reply is still pending and is sent after it`: the test advances time explicitly; still passes); 5 self-tests added in `test_rtos_setup.cpp` |

## Environment

The exact configure / build / run / sanitizer / coverage commands are those of `Tests/Code_Coverage/claude/plans/PROGRESS.md` "Environment" (source of truth for the toolchain). Integration additions, valid once I0 exists:

- Build directories: `../ffb-test-build/it-<variant>` (`default`, `m32`, `san`, `clang`, `clang-san`, `nodsp`, `nodsp-san`, `cov`); audit and review copies under `../ffb-test-build/audit/`.
- Gate of a work order: `python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --wo <ID> --variants default,m32`; phase gate: add `--full`.
- Run directly: `../ffb-test-build/it-default/ffb_integration_tests` (`-tc="[F1-E5]*"` for one requirement).
- Integration bug report: `python3 -B Firmware/Tests/Integration_Tests/tools/make_integration_bug_report.py`.
