# Tests/Code_Coverage: documents of the host test suite

Project-management files only; no test code (that is in `Firmware/Tests/Code_Coverage/`, described in `Firmware/Tests/README.md`). Three trees:

| Folder | Owner | Content | Start with |
|---|---|---|---|
| [claude/](claude/README.md) | Claude | plans, progress, coverage, bug and question reports, requirement charters | [claude/README.md](claude/README.md) ("Where do I find ...") |
| [codex/](codex/README.md) | Codex | Codex's independent investigation: baseline, per-phase reviews, its P12 work, proposals | [codex/README.md](codex/README.md) |
| [shared/](shared/README.md) | the owner | specification files supplied by the owner (git-ignored, never committed) | [shared/README.md](shared/README.md) |

## Reviews of Claude's work by Codex

Both reviews are in `codex/reviews/` and have an index page that lists the reports, the resolution of every finding and the evidence files:

| Review | Scope | Index |
|---|---|---|
| P11 and P13-P18 | CMSIS-DSP, TMCDebugBridge, PWM, SimpleMotion, MyActuator, MIDI, 32-bit build | [codex/reviews/new_phases/INDEX.md](codex/reviews/new_phases/INDEX.md) |
| P12 | real ST HAL | [codex/reviews/claude_p12/INDEX.md](codex/reviews/claude_p12/INDEX.md) |
| P0-P10 (the first review) | everything up to the driver track | [codex/reviews/README.md](codex/reviews/README.md) |

Claude's verdicts on these reviews: [claude/reports/CODEX_REVIEW_AUDIT.md](claude/reports/CODEX_REVIEW_AUDIT.md) and [codex/reviews/CLAUDE_AUDIT_RESPONSE_2026-11-13.md](codex/reviews/CLAUDE_AUDIT_RESPONSE_2026-11-13.md) (Codex's assessment of that audit).

## Rules for the folders

- Each agent writes only into its own tree; `shared/specs/` is never written by an agent.
- Every number (counts, coverage, bug counters) lives in one place: `claude/plans/PROGRESS.md` "Current state". Other documents link to it instead of copying numbers.
- Files that are evidence of a review (JSON, logs, diffs) are not edited; they carry SHA-256 records of each other (`evidence/` folders).
