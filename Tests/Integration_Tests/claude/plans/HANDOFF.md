# Handoff: OpenFFBoard integration test track (for any agent taking over)

Written 2026-11-18 by the lead (Opus 5.5), updated 2026-11-24. You have no memory of this project: read this file, then the read order below. Where this file and `PROGRESS.md` disagree, **`PROGRESS.md` wins**.

## What this is

A host-side integration test suite: the real firmware modules of a force-feedback flow wired together (USB HID PID -> `HidFFB` -> `EffectsCalculator` with CMSIS-DSP -> `Axis` -> motor driver), with only the hardware faked (fake SPI / USB controller / timers / ADC / GPIO / flash, `FakeTmc4671`, later the CAN / UART peers). It sits next to the finished code-coverage unit-test suite (`Tests/Code_Coverage/`, `Firmware/Tests/Code_Coverage/`) and reuses its fakes and support code. Branch `add-test-suite`.

## State (updated 2026-11-24)

Where things stand, with every number: `PROGRESS.md` "Current state" (this file holds no numbers). Short form:

* I0 and I1 are approved ([../reports/I0_REPORT.md](../reports/I0_REPORT.md), [../reports/I1_REPORT.md](../reports/I1_REPORT.md)).
* I2 (G-T, G-P; work orders [../charters/I2_WORK_ORDERS.md](../charters/I2_WORK_ORDERS.md), approved 2026-11-24): all writer work orders (I2-1 to I2-6) and the tool WO I2-T1 are done and committed. **Phase I2 complete (2026-11-25, [../reports/I2_REPORT.md](../reports/I2_REPORT.md)), waiting for the owner's approval.** Do not start I3 before that approval.
* Phases run strictly one after another.

### Helpers and conventions settled so far

* Since I1: `FfbHost` (PID reports from the parsed descriptor), `TorqueTrace`, `PositionScript` (see `Firmware/Tests/README.md`). Device Control uses the PID 1.01 Figure 16 command numbers (IQ12, IB 8), with a second case in the flag encoding when a defect hides behind IB 8. Rotated conditions act along a line (sin² / cos²). The polar zero is undefined (relative signs only). Absolute counts from firmware constants are `characterization:` with an INR.
* Since I2-1: `rebootKeepingFlash` (models a cold start with the flash kept; no USB replug unless allowed, `rebootReplugged()` for the replug), `pressEstop` / `releaseEstop`, `ChipEventLog` (`armSpiFailure`, `armSpiTimeoutAfter`, `runCommandOnWrite`). Limits are in `Firmware/Tests/README.md` and the decisions log (2026-11-24): the 0x63 read-modify-write race of F1-T6 cannot be produced on the cooperative host, and `bringUp()` time depends on the process-wide `powerCheckCounter`.
* **Attach order:** attach a `ChipEventLog` after any `TorqueTrace` and stop the trace only after the log is gone (lost sampling is a fixture fault). Stop traces and re-create `FfbHost` around a reboot.
* **Per-case watchdog (I2-T1, shared by I2-T2):** every doctest executable has one (`Code_Coverage/support/CaseWatchdog`, compiled default per executable, `FFB_IT_CASE_TIMEOUT_S` overrides); the only whole-run limits left are the two Python ctest entries. For the integration executable: every case has a time limit inside the executable (`support/CaseWatchdog`, `FFB_IT_CASE_TIMEOUT_S`, default 120 s, exit code 86 and a `CASE TIMEOUT` line); there is no limit on a whole run. A timed-out case is rerun alone by the gate (in `cov` it is a failure).
* **Gate** (`AGENT_PLAN.md` 2.4): the integration executable runs once per variant on at most 10 jobs in total (`--jobs`, default and maximum 10). Each gate process counts only its own jobs, so with parallel writers split the 10 between them with `--jobs` (e.g. `--jobs 2` each for four writers). A WO gate runs only its own case files when only case files changed. Before a commit check `rerun_passed` and `dead_shards` in the evidence (a case listed there is investigated, not committed over). Random-order and repeat runs come only at the end of the integration testing.
* **Do not change documents while an `--isolated` gate runs:** it overlays the lead's changed documents on HEAD (`docs_overlaid` in the evidence), and fails with "the documents under Tests/Integration_Tests/ changed while the isolated gate ran" if one changes during the run.

## Where to continue

`PROGRESS.md` "Current state" and "Next action". Do not start a phase without the owner's go-ahead; stop at the end of every phase.

## Read order

1. Root `CLAUDE.md` (rules, including the "Integration track" section).
2. `PROGRESS.md` ("Current state", "Next action", decisions log, registers).
3. `INTEGRATION_TEST_PLAN.md` (chain, architecture, fake audit list, stability method, phases, limits).
4. `AGENT_PLAN.md` (agents, hooks, gate, per-work-order flow, what goes to the owner).
5. The flow charter (`../charters/F1_TMC4671_CHAIN_REQUIREMENTS.md`) and the current phase's work orders (`../charters/I<n>_WORK_ORDERS.md`).
6. `Firmware/Tests/README.md` (fake platform, simulated time, fixtures, naming conventions).
7. Before reviewing or recording a bug: `../reports/OPEN_QUESTIONS.md` (IQ n), `../reports/BUG_REPORT.md` (generated) and the coverage suite's `Tests/Code_Coverage/claude/reports/BUG_REPORT.md` and `OPEN_QUESTIONS.md` (answered questions there settle intended behaviour).
8. The unit charters the flow builds on: `Tests/Code_Coverage/claude/charters/driver/TMC4671_REQUIREMENTS.md`, `.../dsp/CMSIS_DSP_REQUIREMENTS.md`.

## Method (per work order)

Up to four agents at once, only on independent work; a step that another uses or affects is finished, reviewed and committed first (owner, 2026-11-19): parallel writers on disjoint file lists, each listed in `../charters/wo/ACTIVE`, gate with `--own-build`; the lead's final gate in the shared folders before each commit (`AGENT_PLAN.md` section 3). Lead writes a self-contained work order -> `integration-test-writer` implements it and runs `it_gate.py` (64-bit and 32-bit) -> `integration-reviewer` reviews in a fresh context -> writer fixes -> gate -> lead QC -> `PROGRESS.md`, this file, registers, generated bug report -> local commit with the gate fingerprint. Details: `AGENT_PLAN.md` section 3.

## Hooks (since WO I0-1)

`.claude/settings.json` runs `.claude/hooks/guard_paths.py` (Edit / Write / MultiEdit / NotebookEdit), `guard_bash.py` (Bash) and, when `integration-test-writer` stops, `require_gate_evidence.py`. A blocked call prints `BLOCKED by ...` with the reason: change the approach, do not work around the hook. A commit with test code needs `WO: <id>` in the message and that WO's `GATE PASS` evidence (`it_gate.py --wo <id> --variants default,m32`, file list `../charters/wo/<id>.files`); stage files by name (Codex's changes and `AGENTS.md` stay unstaged). Rules and evidence format: `AGENT_PLAN.md` 2.4.

## Pitfalls known from the coverage track (apply here too)

* Writers sometimes locked a buggy value in without a marker: read every assertion's oracle comment.
* Claimed bugs that were misreadings: confirm in the source and with a failing test before an IB number.
* Relative build paths inside the repository: forbidden (the hook blocks them).
* `KNOWN_ISSUE` cases whose first failure was not the defect (a teardown check, a timeout): call `REQUIRE_CHAIN_MODELLED()` before the defect assertion; a bounded wait in a `KNOWN_ISSUE` case must not fail on timeout.
* Never depend on stale heap or stack content; call init-style functions unconditionally.
* Usage limits interrupt agents: resume them with `SendMessage` instead of starting over.

## Integration-specific pitfalls (expected)

* Three fixtures (`UsbFixture`, `TargetFixture`, `TmcFixture`) reset the same globals differently: use only `IntegrationFixture` (since I0-5; `bringUp()` = `reachShutdown()` + `mountAndRun()`).
* Every firmware thread that delays must be resumable (`ChainScheduler::adoptThreads`); a new delaying thread that is not adopted shows up as a kernel fault at teardown.
* Cases that bring up the firmware carry `IT_DELETES_FFBHIDMAIN` (main-class delete is coverage bug 24) and are skipped in sanitizer builds unless run with `--no-skip` and `ASAN_OPTIONS=new_delete_type_mismatch=0`.
* `HostUsb::reset` belongs to the recording TinyUSB fake, which is not linked here.
* The TMC driver thread relies on the delay budget of `HostRtos`; it was never combined with `HOSTTEST_RTOS_EXT` before I0.
* `FakeTmc4671` does not move the position by itself: open-loop tests script 0x6B, closed-loop tests attach `WheelPlant`.
* The scheduler order is a host choice; tests that depend on it say so.
* Audit results (I0-2, `../charters/audit/`): timers and the ADC are not driven by simulated time (the scheduler fires them from PSC / ARR); IT SPI transfers complete only on `spiComplete`; `FakeTmc4671` stores registers only (the plant applies torque mode, ENI and 0x5E itself and advances 0x6B by differences); `vTaskDelay` in a task jumps the clock until fix A-3 is decided; `runUntilIdle` ignores priorities.
* Fake auditors needed more than 60 turns for a group of fakes (now 100); keep audit orders small.
