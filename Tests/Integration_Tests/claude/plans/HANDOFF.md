# Handoff: OpenFFBoard integration test track (for any agent taking over)

Written 2026-11-18 by the lead (Opus 5.5). You have no memory of this project: read this file, then the read order below. Where this file and `PROGRESS.md` disagree, **`PROGRESS.md` wins**.

## What this is

A host-side integration test suite: the real firmware modules of a force-feedback flow wired together (USB HID PID -> `HidFFB` -> `EffectsCalculator` with CMSIS-DSP -> `Axis` -> motor driver), with only the hardware faked (fake SPI / USB controller / timers / ADC / GPIO / flash, `FakeTmc4671`, later the CAN / UART peers). It sits next to the finished code-coverage unit-test suite (`Tests/Code_Coverage/`, `Firmware/Tests/Code_Coverage/`) and reuses its fakes and support code. Branch `add-test-suite`.

## State at the end of phase I0 (2026-11-18)

Phase I0 is complete and was approved by the owner on 2026-11-18 ([../reports/I0_REPORT.md](../reports/I0_REPORT.md)). Phase I1 (open-loop chain) is in progress: work orders approved (`../charters/I1_WORK_ORDERS.md`); WO I1-1 (host helpers `FfbHost`, `PositionScript`, `TorqueTrace`; F1-H1, F1-H3) done 2026-11-18; WO I1-2 (F1-H2, H4-H6; `FfbHost::queue`, `ChainScheduler::setUsbAfterFfb`) done 2026-11-19. Device Control is sent as PID 1.01 Figure 16 command numbers (owner, IQ12; IB 8). I1 test cases use `FfbHost` on top of `IntegrationFixture`, set `axis.0.power` and `axis.0.fxratio` first (IB 7 / bug 23: zero torque on a fresh flash otherwise), and never put a REQUIRE inside an INFO expression (doctest deadlocks; I1-1 review).

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
