# Handoff: OpenFFBoard integration test track (for any agent taking over)

Written 2026-11-18 by the lead (Opus 5.5). You have no memory of this project: read this file, then the read order below. Where this file and `PROGRESS.md` disagree, **`PROGRESS.md` wins**.

## What this is

A host-side integration test suite: the real firmware modules of a force-feedback flow wired together (USB HID PID -> `HidFFB` -> `EffectsCalculator` with CMSIS-DSP -> `Axis` -> motor driver), with only the hardware faked (fake SPI / USB controller / timers / ADC / GPIO / flash, `FakeTmc4671`, later the CAN / UART peers). It sits next to the finished code-coverage unit-test suite (`Tests/Code_Coverage/`, `Firmware/Tests/Code_Coverage/`) and reuses its fakes and support code. Branch `add-test-suite`.

## State (updated 2026-11-23, end of phase I1)

Phase I0 approved 2026-11-18 ([../reports/I0_REPORT.md](../reports/I0_REPORT.md)). Phase I1 (open-loop chain, groups G-H, G-E, G-X) is complete and was approved by the owner on 2026-11-24 ([../reports/I1_REPORT.md](../reports/I1_REPORT.md)): 325 integration cases in `test_f1_setup.cpp`, `test_f1_support.cpp`, `test_f1_hid.cpp`, `test_f1_effects.cpp`, `test_f1_conditions.cpp`, `test_f1_axis.cpp`. Helpers since I1: `FfbHost` (PID reports from the parsed descriptor), `TorqueTrace`, `PositionScript` (see `Firmware/Tests/README.md`). Conventions settled in I1: Device Control as PID 1.01 Figure 16 command numbers (IQ12, IB 8) with a second case in the flag encoding when a defect hides behind IB 8; rotated conditions act along a line (sin² / cos²); polar zero undefined (relative signs only); absolute counts from firmware constants are `characterization:` with an INR. Tool WO I1-T3 (gate speed-up, owner-ordered) is done: a two-variant WO gate takes about 4 minutes, a `--full` phase gate about 18 minutes (before: about 30 minutes and 2 hours); details `AGENT_PLAN.md` 2.4. Before a commit check `rerun_passed` and `dead_shards` in the evidence. Phase I2 (G-T, G-P): work orders approved 2026-11-24 (`../charters/I2_WORK_ORDERS.md`); WO I2-1 done (support helpers `rebootKeepingFlash`, `pressEstop` / `releaseEstop`, `ChipEventLog` with `armSpiFailure` / `armSpiTimeoutAfter` / `runCommandOnWrite`; limits in `Firmware/Tests/README.md` and the decisions log). Attach a `ChipEventLog` after any `TorqueTrace` and stop the trace only after the log is gone (lost sampling is a fixture fault); stop traces and re-create `FfbHost` around a reboot. Next: I2-2, I2-3, I2-4, I2-6 in parallel, then I2-5, I2-7. Phases run strictly one after another.

## Where to continue

`PROGRESS.md` "Current state" and "Next action". Do not start a phase without the owner's go-ahead; stop at the end of every phase.

## Read order

1. Root `CLAUDE.md` (rules; the "Integration track" section once added).
2. `PROGRESS.md` ("Current state", "Next action", decisions log, registers).
3. `INTEGRATION_TEST_PLAN.md` (chain, architecture, fake audit list, stability method, phases, limits).
4. `AGENT_PLAN.md` (agents, hooks, gate, per-work-order flow, what goes to the owner).
5. The flow charter (`../charters/F1_TMC4671_CHAIN_REQUIREMENTS.md`) and the current phase's work orders (`../charters/I<n>_WORK_ORDERS.md`).
6. `Firmware/Tests/README.md` (fake platform, simulated time, fixtures, naming conventions).
7. Before reviewing or recording a bug: `../reports/OPEN_QUESTIONS.md` (IQ n), `../reports/BUG_REPORT.md` (once generated) and the coverage suite's `Tests/Code_Coverage/claude/reports/BUG_REPORT.md` and `OPEN_QUESTIONS.md` (answered questions there settle intended behaviour).
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
