# Tests/Integration_Tests: documents of the host integration test track

Project-management files only; no test code (that is in `Firmware/Tests/Integration_Tests/`, built as `ffb_integration_tests`). The integration track proves that the real firmware modules work together, with only the hardware boundary faked. It sits next to the code-coverage suite (`../Code_Coverage/`), whose fakes and support code it reuses.

| Folder | Owner | Content | Start with |
|---|---|---|---|
| [claude/](claude/README.md) | Claude | plans, progress, coverage, bug and question reports, requirement charters, work orders | [claude/README.md](claude/README.md) ("Where do I find ...") |
| `codex/` | Codex | reserved for Codex's independent reviews of this track (not created by Claude) | |

Specification files are not copied here: they stay in `../Code_Coverage/shared/specs/` (git-ignored, supplied by the owner, never committed).

## Rules for the folders

- Each agent writes only into its own tree; nothing here writes into `../Code_Coverage/` (a fix of a shared fake updates the coverage documents in the same commit, see the plan section 9).
- Every number (counts, coverage, register counters) lives in one place: `claude/plans/PROGRESS.md` "Current state". Other documents link to it.
- Generated files are never edited by hand: `claude/reports/BUG_REPORT.md` (generated from the registers in `PROGRESS.md` by `Firmware/Tests/Integration_Tests/tools/make_integration_bug_report.py`; `--check` verifies it) and the oracle tables under `Firmware/Tests/Integration_Tests/claude/oracles/`.
- Bugs and NEEDS REVIEW items of this track have their own numbers (IB n, INR n); a coverage-suite bug that an integration test hits again is cross-referenced, not renumbered.
