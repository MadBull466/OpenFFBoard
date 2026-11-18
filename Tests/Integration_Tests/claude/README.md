# Claude: integration track documents

Project-management files of the integration track. Test code: `Firmware/Tests/Integration_Tests/`. Map: [../README.md](../README.md). The coverage suite's documents: [../../Code_Coverage/claude/README.md](../../Code_Coverage/claude/README.md).

## Where do I find ...

| I want to know | Read |
|---|---|
| where the work stands, what is open, every number and counter | [plans/PROGRESS.md](plans/PROGRESS.md): "Current state", "Next action" |
| how to take over (read order, method, pitfalls) | [plans/HANDOFF.md](plans/HANDOFF.md) |
| the plan: chain, architecture, fake audit list, stability method, phases, limits | [plans/INTEGRATION_TEST_PLAN.md](plans/INTEGRATION_TEST_PLAN.md) |
| the agents, hooks, gate and the per-work-order flow; what needs the owner | [plans/AGENT_PLAN.md](plans/AGENT_PLAN.md) |
| what is covered, out of scope, not covered | [plans/COVERAGE_AND_SCOPE.md](plans/COVERAGE_AND_SCOPE.md) |
| what flow F1 must prove (requirements and oracles) | [charters/F1_TMC4671_CHAIN_REQUIREMENTS.md](charters/F1_TMC4671_CHAIN_REQUIREMENTS.md) |
| the work orders of a phase | `charters/I<n>_WORK_ORDERS.md` (I0: [charters/I0_WORK_ORDERS.md](charters/I0_WORK_ORDERS.md)) |
| the stability derivation and the plant model | `charters/stability/` (phase I4) |
| the result of a fake audit | `charters/audit/` (phase I0) and the "Fake audit register" in `PROGRESS.md` |
| integration bugs and NEEDS REVIEW items | `reports/INTEGRATION_BUG_REPORT.md` (generated from `PROGRESS.md`; do not edit) |
| questions to the owner and their answers | [reports/OPEN_QUESTIONS.md](reports/OPEN_QUESTIONS.md) |
| a decision and its date | [plans/PROGRESS.md](plans/PROGRESS.md): "Decisions log" |
| a phase result (self-contained, for an outside reviewer) | `reports/I<n>_REPORT.md` |

## Folders

| Folder | Content |
|---|---|
| `plans/` | `PROGRESS.md` (source of truth), `INTEGRATION_TEST_PLAN.md`, `AGENT_PLAN.md`, `COVERAGE_AND_SCOPE.md`, `HANDOFF.md`, `APPROVED_SHARED_EDITS.txt` (from I0; read by the hook) |
| `reports/` | `OPEN_QUESTIONS.md`, `INTEGRATION_BUG_REPORT.md` (generated), phase reports |
| `charters/` | flow charters `F<n>_*_REQUIREMENTS.md`, work orders `I<n>_WORK_ORDERS.md`, `stability/`, `audit/` |
