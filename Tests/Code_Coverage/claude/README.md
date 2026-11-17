# Claude: documents

Project-management files of the work done by Claude. Test code: `Firmware/Tests/Code_Coverage/claude/`. Map of all three document trees (`claude/`, `codex/`, `shared/`): [../README.md](../README.md).

## Where do I find ...

| I want to know | Read |
|---|---|
| where the work stands, what is open, the counters | [plans/PROGRESS.md](plans/PROGRESS.md): "Current state", "Next action" |
| how to take over (read order, method, pitfalls) | [plans/HANDOFF.md](plans/HANDOFF.md) |
| the plan, the phases P0-P18, estimate versus measured coverage | [plans/TEST_PLAN.md](plans/TEST_PLAN.md) |
| coverage per tier, what is out of scope, what is not covered | [plans/COVERAGE_AND_SCOPE.md](plans/COVERAGE_AND_SCOPE.md) |
| the result in one page | [reports/PROJECT_SUMMARY.md](reports/PROJECT_SUMMARY.md) |
| suspected bugs and NEEDS REVIEW items | [reports/BUG_REPORT.md](reports/BUG_REPORT.md) (generated from the tables in `PROGRESS.md`; do not edit) |
| questions waiting for the owner (and answered ones) | [reports/OPEN_QUESTIONS.md](reports/OPEN_QUESTIONS.md) |
| a decision and its date | [plans/PROGRESS.md](plans/PROGRESS.md): "Decisions log" |
| the independent review by Codex and Claude's answers | [reports/CODEX_REVIEW_AUDIT.md](reports/CODEX_REVIEW_AUDIT.md) and `../codex/reviews/` (index pages `claude_p12/INDEX.md`, `new_phases/INDEX.md`) |

## Folders

| Folder | Content |
|---|---|
| `plans/` | status: `PROGRESS.md` (resume point, source of truth), `TEST_PLAN.md`, `COVERAGE_AND_SCOPE.md`, `HANDOFF.md`. Per-phase plans: `DRIVER_TEST_PLAN.md` (P10), `P12_ST_HAL_PLAN.md`, `P13_TMCDEBUGBRIDGE_PLAN.md`, `P14_P17_REMAINING_DRIVERS_PLAN.md`, `P18_32BIT_BUILD_PLAN.md` |
| `reports/` | `BUG_REPORT.md` (generated), `OPEN_QUESTIONS.md`, `PROJECT_SUMMARY.md`, `CODEX_REVIEW_AUDIT.md`; driver track: `DRIVER_TRACK_REPORT.md` with the detail reports `D1_TMC4671_REPORT.md` and `D2_ODRIVE_REPORT.md` |
| `charters/driver/` | requirements charters `<DRIVER>_REQUIREMENTS.md` (TMC4671, ODrive, VESC, TMCDebugBridge, PWM, SimpleMotion, MyActuator, MIDI) and work orders `P13`-`P17_WORK_ORDERS.md` |
| `charters/dsp/` | CMSIS-DSP (P11): `CMSIS_DSP_REQUIREMENTS.md`, `CMSIS_DSP_INVENTORY.md`, `G1`-`G5_WORK_ORDERS.md` |
| `charters/hal/` | ST HAL (P12): `P12_T1`-`P12_T8_REQUIREMENTS.md` (T9 had no charter: close-out) |
| `charters/P18_WORK_ORDERS.md` | 32-bit build (P18) |

Shared inputs (specification files, git-ignored): `../shared/specs/`, listed in [../shared/README.md](../shared/README.md).
