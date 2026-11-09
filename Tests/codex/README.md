# Codex: independent test-suite investigation

Active work: [P12 ST HAL](p12/PLAN.md), explicitly brought into scope by the owner. P11 is deferred. [P12 inventory](p12/INVENTORY.md) records discovery; no HAL coverage is claimed yet.

> Current status (2026-11-06): the authorized closeout is complete; see [CLOSEOUT_RESULTS](CLOSEOUT_RESULTS.md) and [final QC](reviews/CLOSEOUT_FINAL_QC.md#final-integrated-release-acceptance). Earlier counts and pending statements below are historical staged evidence.

The owner authorised independent review from P0 through P10 on 2026-11-05. Discovery, baseline verification and the approved CDX-001 Python path correction are complete. The assigned review is complete and accepted by fresh QC; four new target cases are integrated and verified, P0/P1 shared patches were subsequently approved and applied, with affected verification complete; P11 expanded DSP is excluded. The current phase matrix is [reviews/README.md](reviews/README.md).

Test code belongs in `Firmware/Tests/codex/`; documents belong here. Claude-owned files, firmware, vendor sources, original baseline cases and `.github/` remain unchanged. Shared infrastructure changes need the owner's approval under root `AGENTS.md`.

| Document | Purpose |
|---|---|
| [CLOSEOUT_RESULTS.md](CLOSEOUT_RESULTS.md) | Current accepted closeout, final gates, evidence and remaining limits |
| [DISCOVERY.md](DISCOVERY.md) | Architecture, inventory, documentation read and deferred, scope and outstanding decisions |
| [AGENT_WORKFLOW.md](AGENT_WORKFLOW.md) | Agreed agent models, reasoning effort, responsibilities and independent review process |
| [BASELINE.md](BASELINE.md) | Measured state at the starting revision, configuration results, warnings and limitations |
| [TEST_EXECUTION.md](TEST_EXECUTION.md) | Reproduction commands and evidence locations |
| [REVIEW_FINDINGS.md](REVIEW_FINDINGS.md) | Confirmed findings, evidence links and resolution status |
| [COVERAGE_GAPS.md](COVERAGE_GAPS.md) | Execution measurements, behavioural gaps and verification limits |
| [NEW_TESTS.md](NEW_TESTS.md) | Codex test inventory |
| [REGRESSION_RESULTS.md](REGRESSION_RESULTS.md) | Baseline, approved path-fix verification and supplemental regressions |
| [PROGRESS.md](PROGRESS.md) | Completed and outstanding work |
| [HANDOFF.md](HANDOFF.md) | Resume point and authorisation boundary |

Method: preserve the initial Git state, use fresh out-of-tree builds, retain the configured exclusions, distinguish expected failures from ordinary passes, and label historical or unexecuted claims explicitly. Passing tests and covered lines do not establish correct assertions or fake fidelity.

Approved P0/P1 application and final affected checks: [P0_P1_INTEGRATION.md](P0_P1_INTEGRATION.md). Default full CTest 9/9; documented local ring-case exclusion applies. All six new C++ cases are registered; 36 Codex Python regressions pass directly.

Before starting a new phase: [P0_P10_CLOSEOUT.md](P0_P10_CLOSEOUT.md) prioritizes unresolved findings and distinguishes closure work from explicit deferrals.
