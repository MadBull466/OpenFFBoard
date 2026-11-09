# Codex agent allocation and review workflow

Recorded 2026-11-05 at the owner's request, following the proposed model allocation in chat. The owner subsequently authorised review from P0 through P10; this workflow governs that active work. P11 is excluded from the current audit.

## Model allocation

| Role | Model | Agent model ID | Reasoning effort |
|---|---|---|---|
| Technical lead: requirements, ambiguous specifications and final quality review | GPT-6 Astra | `gpt-6-astra` | `high` |
| Test creation and routine investigations | GPT-6.1 Sol | `gpt-6.1-sol` | `medium` |
| Independent review and mutation testing | Fresh GPT-6.1 Sol agents | `gpt-6.1-sol` | `high` |
| Difficult reviewer disagreements or unresolved technical questions | GPT-6 Astra | `gpt-6-astra` | `high` |

This preserves the division of responsibilities in Claude's Opus 5.5 medium / Sonnet 5.5 medium workflow. It is a role-based allocation, not a claim of equivalent capabilities or comparable reasoning-effort scales. Rationale: [OpenAI model-selection guidance](https://developers.openai.com/api/docs/guides/model-selection) positions Astra for demanding analysis and Sol for complex technical work with lower cost. Both are exposed by this session's agent tools; recheck availability in future sessions.

The primary Codex agent coordinates assignments, permissions, file ownership, integration, verification and reporting. Delegating the technical-lead role to Astra does not change the primary chat's model. This session supports up to three active subagents alongside the primary agent. Agents with explicit model/effort overrides start with focused task context (`fork_turns="none"`); provide the relevant requirements, paths, constraints and evidence in their brief.

## Responsibilities and sequence

1. **Scope and requirements:** after the owner authorises a review group, the coordinator supplies its documentation and boundaries. The technical lead checks specifications, defines observable requirements and acceptable oracles, and identifies questions requiring the owner. Existing Claude findings are evidence to investigate, not accepted conclusions.
2. **Investigation and test creation:** writers receive bounded requirement groups, explicit file ownership and separate out-of-tree build locations. Expected results come from supplied specifications, vendor documentation or independent derivations. New tests, fixtures and fakes stay in `Firmware/Tests/codex/`; documents stay in `Tests/codex/`.
3. **Independent review:** fresh reviewers examine work they did not author. Check assertion correctness, false positives, coverage gaps, fake fidelity, fixture resets, error paths, host-versus-target differences and KNOWN_ISSUE first-failure behaviour. Use isolated/random-order runs and mutation experiments where meaningful. Mutations stay in scratch copies outside the repository.
4. **Resolve findings:** writers address confirmed issues within their ownership. Astra handles difficult technical disagreements; material uncertainty about requirements or specifications goes to the owner. Never invent protocol facts, change protected firmware, weaken assertions or silently expand scope.
5. **Integration and verification:** the coordinator checks all changes, runs the original-case checker and affected regression configurations, and investigates unexpected failures. Shared infrastructure/build edits require the agreed owner approval. Parallel writers must not edit the same files or share mutable build outputs.
6. **Final technical review and handoff:** Astra reviews the requirement/test matrix, evidence, known issues and limitations. The coordinator independently verifies the combined result, updates Codex findings/progress/handoff, reports actual execution and stops at the agreed owner approval gate.

The lead handles requirements and difficult decisions rather than bulk test writing. Writers do not approve their own tests. Reviewers receive enough source/specification context to derive expectations independently, including known limitations; a fresh reviewer means a separate agent assignment, not a claim of a different model family.

## Standing boundaries

- Root `AGENTS.md`, the owner's instructions and the current authorised scope remain controlling.
- Claude-owned tests/documents, original baseline cases, production firmware, vendor code and `.github/` remain protected.
- Use doctest and stdlib unittest; install/download nothing; use simulated time and no hardware access.
- Build out of tree under `../ffb-test-build/<variant>`; retain documented skips/exclusions and distinguish ordinary passes from expected failures.
- Do not launch agents merely to repeat cheap sequential work or summarise logs that the existing scripts can summarise.
- Record model, effort, assignment, reviewed files, evidence and unresolved questions for each substantive agent task. Report an unavailable model before substituting another allocation.

Current status: Astra oversight, fresh Sol reviews and Sol writers have been launched for the authorised P0-P10 cycle. Exact assignments, assessed scope and pending QC are recorded under [reviews/](reviews/README.md); see [HANDOFF.md](HANDOFF.md). Shared edits still require specific approval.
