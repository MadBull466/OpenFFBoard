# Claude: documents

Project-management files of the work done by Claude (plans, progress, reports, charters). Test code: `Firmware/Tests/claude/`.

| Folder | Content |
|---|---|
| `plans/` | `TEST_PLAN.md`, `DRIVER_TEST_PLAN.md`, `P13_TMCDEBUGBRIDGE_PLAN.md`, `P14_P17_REMAINING_DRIVERS_PLAN.md`, `PROGRESS.md` (resume point, source of truth), `HANDOFF.md`, `COVERAGE_AND_SCOPE.md` |
| `reports/` | `BUG_REPORT.md` (generated), `OPEN_QUESTIONS.md`, `D1_TMC4671_REPORT.md`, `D2_ODRIVE_REPORT.md`, `DRIVER_TRACK_REPORT.md`, `PROJECT_SUMMARY.md`, `CODEX_REVIEW_AUDIT.md` |
| `charters/` | requirements charters and work orders: `driver/` (TMC4671, ODrive, VESC, TMCDebugBridge, PWM, SimpleMotion, MyActuator; `P13`-`P16_WORK_ORDERS.md`) and `dsp/` (CMSIS-DSP requirements and inventory, `G1`-`G5_WORK_ORDERS.md`). `driver/MIDI_REQUIREMENTS.md` is the P17 MIDI charter (T0 approved) |

Shared inputs (specification files, git-ignored): `Tests/shared/specs/`.
