# Integration track: progress

> Resume point of the integration track. Read "Current state" and "Next action" first.
> Plan: `INTEGRATION_TEST_PLAN.md`; agents, hooks, gate: `AGENT_PLAN.md`; rules: root `CLAUDE.md`. Every number of this track lives here and nowhere else.

## Current state

*Last updated: 2026-11-18.* Branch `add-test-suite`. Coverage-suite state at the start: `Tests/Code_Coverage/claude/plans/PROGRESS.md` "Current state" (commit `b3731f63`).

- **Phase:** I0 (infrastructure), started 2026-11-18 after the owner approved the plan. WO I0-1 done (not yet committed): five integration agent files and the `test-output-summariser` edit in `.claude/agents/`; hooks `.claude/settings.json` + `.claude/hooks/{guard_lib,guard_paths,guard_bash,require_gate_evidence}.py`; hook tests `Firmware/Tests/Integration_Tests/tools/test_hooks.py` (31 tests, OK); `CLAUDE.md` sections "Integration track" and "Model roles (driver track and integration track)"; `APPROVED_SHARED_EDITS.txt` (one line: `Firmware/Tests/CMakeLists.txt`, I0-4). Live checks 2026-11-18: lead Write into `Firmware/` and `codex/`, Bash redirect into `Firmware/Makefile`, `git push origin`, `git config`, in-tree `cmake -B`, `curl` all blocked; a subagent (`driver-test-writer` probe) was blocked from writing and from `git stash`, so `agent_type` reaches the hooks. Not yet verified live: `MultiEdit` / `NotebookEdit` field names (covered by unit tests only), the `SubagentStop` hook on `integration-test-writer` (new agent files load only in a new session).
- **Tests:** `ffb_integration_tests` not created; 0 cases.
- **Requirements (F1 charter):** 62 requirements (A 5, H 6, E 14, R 7, X 7, T 9, P 2, S 4, O 5, B 3; P1 and P2 split in the charter) plus 12 candidate seams S1-S12 (not requirements); 0 covered.
- **Gate:** `it_gate.py` does not exist yet (WO I0-4); until then a commit containing integration test code is blocked by the hook, and configuration / document commits are exempt (config commits run `test_hooks.py`).
- **Registers:** 0 integration bugs, 0 INR items. **Counters:** next IB 1, next INR 1, next IQ 5 (IQ1-IQ4 answered 2026-11-18).
- **Roles:** lead Opus 5.5 (high); workers per `AGENT_PLAN.md`.

## Next action

1. Owner: decide whether to commit WO I0-1 (agents, hooks, `CLAUDE.md`, planning documents) locally.
2. Next session (the new agent files load at session start): confirm the five integration agents are listed; run one `integration-test-writer` probe to verify the `SubagentStop` gate hook live; then WO I0-2, the four parallel fake audits (`claude/charters/I0_WORK_ORDERS.md`).

## Phase checklist

| Phase | Content | Status | Approved by the owner? | Commit |
|---|---|---|---|---|
| Planning | plan, F1 charter, agent plan, documents | done | yes, 2026-11-18 | |
| I0 | infrastructure, fake audit, agents / hooks / gate | in progress (I0-1 done) | | |
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
| 2026-11-18 | IQ1: own register (IB n, INR n) and own generated report `INTEGRATION_BUG_REPORT.md`; coverage bugs hit again are cross-referenced, the coverage register is not changed | Owner's answer |
| 2026-11-18 | IQ2: oracle for the HID entry = `USB_PID1_01.pdf` (supplied); `USB_PID1_02.pdf` only to flag differences | Owner's answer; coverage decisions log 2026-10-23 names PID 1.01 |
| 2026-11-18 | IQ3: nominal wheel plant with documented ranges plus setting sweeps; no hardware data | Owner's answer |
| 2026-11-18 | IQ4: existing functional fakes (audited); no real ST HAL in this track | Owner's answer |
| 2026-11-18 | Lead: one executable `ffb_integration_tests` (feature header `target_f407_drivers`, real TinyUSB, MIDI off) for all flows, defined in `Integration_Tests/integration.cmake` and included from `Firmware/Tests/CMakeLists.txt` | Exploration: `ffb_driver_tests` already compiles the real `Axis` + `TMC4671`; real TinyUSB needs `HOSTTEST_RTOS_EXT` / `HOSTTEST_REAL_TINYUSB` as in `ffb_usb_tests`; one binary avoids drifting source lists |
| 2026-11-18 | Lead: the integration host builds PID reports from the parsed report descriptor, not from the firmware's C structs | `PidHost` uses the firmware structs, so a struct / descriptor mismatch would be invisible |
| 2026-11-18 | Lead: stability = analytic loop model as oracle (stability analyst) + conformance of the real chain to it + closed-loop runs with a plant attached to `FakeTmc4671`; instability inside the settable range = INR item for the owner | Plan section 6 |
| 2026-11-18 | Lead (I0-1): gate evidence records each file of the WO with its git blob hash (`files`) and a fingerprint over them; the commit check compares the staged blobs with it, so unrelated dirty files (Codex's, `AGENTS.md`) neither block nor get committed. Staged `codex/` paths and `AGENTS.md` are always refused. Configuration commits (`.claude/`, `CLAUDE.md`, the hook tests) need no gate but run `test_hooks.py`. `git stash list/show` allowed for subagents (probe false positive) | Implementation of `AGENT_PLAN.md` 2.2 / 2.3; the planned whole-diff fingerprint would have been blocked by the pre-existing changes outside the track |
| 2026-11-18 | Owner approved the plan, the F1 charter, `AGENT_PLAN.md` (agents, hooks, gate) and the `CLAUDE.md` text; phase I0 started | Owner's answer "Approve, start I0" |

## Integration bugs

| # | Module | Location | Description | Same as (coverage) | Evidence (test) | Resolved? |
|---|---|---|---|---|---|---|

## Integration NEEDS REVIEW items

| # | Module | Test / location | Question | Same as (coverage) | Resolved? |
|---|---|---|---|---|---|

## Fake audit register

| Fake | Audit (WO, date) | Result | Fix (WO, owner approval) | Coverage cases changed |
|---|---|---|---|---|

## Environment

The exact configure / build / run / sanitizer / coverage commands are those of `Tests/Code_Coverage/claude/plans/PROGRESS.md` "Environment" (source of truth for the toolchain). Integration additions, valid once I0 exists:

- Build directories: `../ffb-test-build/it-<variant>` (`default`, `m32`, `san`, `clang`, `clang-san`, `nodsp`, `nodsp-san`, `cov`); audit and review copies under `../ffb-test-build/audit/`.
- Gate of a work order: `python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --wo <ID> --variants default,m32`; phase gate: add `--full`.
- Run directly: `../ffb-test-build/it-default/ffb_integration_tests` (`-tc="[F1-E5]*"` for one requirement).
- Integration bug report: `python3 -B Firmware/Tests/Integration_Tests/tools/make_integration_bug_report.py`.
