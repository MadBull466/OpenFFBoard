# Integration track: agents, hooks, gate and workflow

Status: **approved by the owner 2026-11-18.** Agent files, `.claude/settings.json` and the hook scripts exist since WO I0-1 (2026-11-18); the gate script follows in WO I0-4. Where the implementation differs from the text below, section 2.4 says so. Follows the Claude Code documentation as read on 2026-11-18 (subagents: frontmatter fields `name`, `description`, `tools`, `disallowedTools`, `model` (full IDs accepted), `effort`, `maxTurns`; hooks: `PreToolUse` with exit code 2 to block, `SubagentStop` with `decision: block`, hooks fire for subagent tool calls with `agent_type` in the input; best practices: fresh-context reviewer with tools enforced read-only, short summaries back, one task per session, `/clear` after two failed corrections).

## 1. Roster

Starting point: the four agents of the coverage track in `.claude/agents/` (`driver-requirements-mapper`, `driver-test-writer`, `driver-test-reviewer`, `test-output-summariser`). They stay unchanged for the coverage track, except that `test-output-summariser` is reused here and gets `maxTurns` and `disallowedTools: Agent` (small edit). The integration agents are new files because their allowed paths, models and build duties differ; the path rules are enforced by the hooks per `agent_type`, so a coverage agent cannot write integration files and vice versa.

| Role | Agent | Model / effort | Tools | maxTurns | Job |
|---|---|---|---|---|---|
| Lead | main session | Opus 5.5, high | all | n/a | plan, charters, work orders, decisions, final QC, commits, all communication with the owner; writes no bulk tests |
| Explorer | `integration-explorer` | Sonnet 5.5, medium | Read, Grep, Glob | 25 | call chains, units, existing coverage; short answers with file:line |
| Fake auditor | `fake-auditor` | Opus 5.5, high | Read, Grep, Glob, Bash | 100 (60 until 2026-11-18) | audits a fake against its header and source; reports defects and affected coverage cases; never edits the repository |
| Writer | `integration-test-writer` | Sonnet 5.5, medium | Read, Grep, Glob, Write, Edit, Bash | 80 | one work order at a time; builds and runs both builds; returns the gate evidence |
| Stability analyst | `stability-analyst` | Opus 5.5, high | Read, Grep, Glob, Write, Edit, Bash | 60 | stability derivation, oracle scripts (Python stdlib), generated prediction tables |
| Reviewer | `integration-reviewer` | Opus 5.5, high | Read, Grep, Glob, Bash | 100 (50 until 2026-11-18) | adversarial review in a fresh context; never edits the repository |
| Summaries | `test-output-summariser` (existing) | Haiku 5.5 | Read, Grep, Bash | 10 | only when `summarise_results.py` cannot do it |

Where the proposed design was changed, with the reason:

1. **The reviewer can read the repository, not only the diff.** It must check oracles against the specs and the firmware source; "sees only the diff, the work order and the charter" is applied to its *prompt* (no writer reasoning, no conversation), not to its tools.
2. **No separate requirements mapper.** Mapping a requirement to existing coverage tests is an explorer question; `driver-requirements-mapper` has `Write` and is not reused.
3. **Stability derivation review.** The analyst (Opus) and the reviewer (Opus) share a model. The rule "writer and reviewer differ" is kept as a different agent with a fresh context, and the derivation is additionally checked by the real firmware through the conformance tests F1-O1 (written by the Sonnet writer, reviewed by the reviewer), so a wrong model cannot pass silently. The owner / Codex may review it at the I4 gate.
4. **Writers also build fakes and fixtures** (I0, fake fixes) from work orders whose design is fixed by the lead; the reviewer checks them like tests.
5. **One writer at a time** on `add-test-suite`; read-only agents in parallel. No `isolation: worktree` (it branches from the default branch).

### 1.1 Agent files (to be created in WO I0-1)

`.claude/agents/integration-explorer.md`
```markdown
---
name: integration-explorer
description: Read-only mapping of firmware call chains, data types, units, threads and existing test coverage for the integration track. Returns short summaries with file:line references.
tools: Read, Grep, Glob
disallowedTools: Write, Edit, MultiEdit, NotebookEdit, Bash, Agent
model: claude-sonnet-5-5
effort: medium
maxTurns: 25
---
You answer one mapping question for the OpenFFBoard integration test track. Read CLAUDE.md first.
Return at most ~800 words: facts with file:line, no file dumps, no opinions on test design.
Mark anything you did not verify as "not verified". Never claim a bug without quoting the line.
If you run out of turns, return what you have and list what is missing.
```

`.claude/agents/fake-auditor.md`
```markdown
---
name: fake-auditor
description: Audits one test fake (FakeTmc4671, the HAL SPI/ADC/TIM/GPIO fakes, HostDcd, HostRtos, WheelPlant) against its fidelity header and its source specification, and lists the coverage tests that depend on any defect. Never edits the repository.
tools: Read, Grep, Glob, Bash
disallowedTools: Write, Edit, MultiEdit, NotebookEdit, Agent
model: claude-opus-5-5
effort: high
maxTurns: 100
---
You audit ONE fake named in the task, for the integration track (Tests/Integration_Tests/claude/plans/INTEGRATION_TEST_PLAN.md section 4).
Read CLAUDE.md, Firmware/Tests/README.md and the fake's header. For every behaviour the task says the integration relies on:
compare the fake with its cited source (datasheet / RM0090 / ST HAL / TinyUSB / FreeRTOS section) and quote both.
Report: defect (spec citation, fake file:line), impact on the integration, coverage cases that rely on it, confidence.
To find dependent coverage cases, copy the repository to ../ffb-test-build/audit/<fake>/ and build a mutated copy there only;
never write inside the repository checkout. Use tools/summarise_results.py for logs. Quote only results you ran.
Never fix anything. If out of turns, return partial results and what remains.
```

`.claude/agents/integration-test-writer.md`
```markdown
---
name: integration-test-writer
description: Implements one integration work order (tests, fixtures, fakes or tool changes named in it) in Firmware/Tests/Integration_Tests/ and builds and runs the 64-bit and 32-bit variants with the gate script.
tools: Read, Grep, Glob, Write, Edit, Bash
disallowedTools: Agent
model: claude-sonnet-5-5
effort: medium
maxTurns: 80
---
You implement ONE work order from Tests/Integration_Tests/claude/charters/. Follow CLAUDE.md, Firmware/Tests/README.md and the
work order. Create or edit only the files the work order lists (the hook enforces this). Expected values come from the oracle
the work order names (spec section, MATH, HD written in a comment); never from the firmware's current output, unless the test
is "characterization:" with its INR number. A KNOWN_ISSUE test asserts the intended behaviour and its first failing assertion
is the defect; it names its IB number. Build only out of tree (../ffb-test-build/it-<variant>).
Finish with: python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --wo <ID> --variants default,m32
and end your answer with the gate's last line ("GATE PASS <evidence path>" or "GATE FAIL ..."), the counts and anything left open.
If the same problem fails twice, stop and return it to the lead under "Lead decision needed" instead of retrying.
Never commit, push, change git config or remotes. Never write any email address anywhere.
```

`.claude/agents/stability-analyst.md`
```markdown
---
name: stability-analyst
description: Derives the stability and frequency-response oracle of the force-feedback loop (filter chain gain/phase, discrete closed-loop poles with the documented nominal wheel plant) and writes it as a derivation document plus Python stdlib oracle scripts and generated tables.
tools: Read, Grep, Glob, Write, Edit, Bash
disallowedTools: Agent
model: claude-opus-5-5
effort: high
maxTurns: 60
---
You write the stability oracle for the integration track (INTEGRATION_TEST_PLAN.md section 6). Read CLAUDE.md and the F1 charter.
Cite every block of the loop model to its firmware line and design value; state every plant parameter with its range and
the reason; separate exact parts (the firmware's linear math) from assumptions. Python: standard library only, run with
python3 -B; scripts and their unittest self-tests go to Firmware/Tests/Integration_Tests/claude/oracles/, the derivation
to Tests/Integration_Tests/claude/charters/stability/ (the hook allows only these). Generated tables are written by the
scripts, never by hand. Never write C++ tests. If out of turns, return partial results and what remains.
```

`.claude/agents/integration-reviewer.md`
```markdown
---
name: integration-reviewer
description: Fresh-context adversarial review of one integration work order's diff against its work order and charter. Reports only gaps that affect correctness or requirements. Never edits the repository.
tools: Read, Grep, Glob, Bash
disallowedTools: Write, Edit, MultiEdit, NotebookEdit, Agent
model: claude-opus-5-5
effort: high
maxTurns: 100
---
You did not write this code and you get no explanation from its author. Inputs: the diff (git diff of the files named),
the work order and the charter. Check: every requirement of the work order has a test that would fail for a wrong
implementation; every expected value has an oracle (spec section / MATH / HD in a comment) and none is copied firmware
output without "characterization:" + INR; KNOWN_ISSUE / KNOWN_ISSUE_32BIT / _UB use is correct and the first failing
assertion is the defect; fakes are used within their fidelity header; nothing outside the work order's files changed;
both builds were run (re-run the gate yourself). Mutate the firmware or the fake only in a copy under
../ffb-test-build/audit/review-<WO>/ (never in the checkout) to check that key tests fail. Skip style. Report findings as
a list: file:line, severity (blocker / major / minor), why, evidence. Quote only results you ran.
```

`test-output-summariser` (existing file): add `maxTurns: 10` and `disallowedTools: Write, Edit, Agent` to its frontmatter; body unchanged.

## 2. Hooks (hard rules as code)

`.claude/settings.json` (project, committed with the plan files after approval):

```json
{
  "hooks": {
    "PreToolUse": [
      { "matcher": "Edit|Write|MultiEdit|NotebookEdit",
        "hooks": [ { "type": "command", "command": "python3 -B \"$CLAUDE_PROJECT_DIR/.claude/hooks/guard_paths.py\"" } ] },
      { "matcher": "Bash",
        "hooks": [ { "type": "command", "command": "python3 -B \"$CLAUDE_PROJECT_DIR/.claude/hooks/guard_bash.py\"" } ] }
    ],
    "SubagentStop": [
      { "matcher": "integration-test-writer",
        "hooks": [ { "type": "command", "command": "python3 -B \"$CLAUDE_PROJECT_DIR/.claude/hooks/require_gate_evidence.py\"" } ] }
    ]
  }
}
```

Both guards read the hook JSON from stdin (`tool_name`, `tool_input`, `cwd`, and `agent_type` when the call comes from a subagent), resolve paths against the repository root (symlinks resolved), print the reason to stderr and exit 2 to block; exit 0 otherwise. Each hook script gets a stdlib `unittest` file in `Firmware/Tests/Integration_Tests/tools/` (feeding it sample JSON), run by the gate. The docs did not document `tool_input.file_path` for every edit tool, so WO I0-1 first logs the real input of one Edit, Write, MultiEdit and NotebookEdit call (`notebook_path`) and adapts the field names.

### 2.1 `guard_paths.py` (Edit / Write / MultiEdit / NotebookEdit)

Block (everyone, lead included):
* any path under `Firmware/` that is not under `Firmware/Tests/`;
* `Firmware/Tests/Code_Coverage/third_party/doctest.h`;
* any path with a `codex` directory component (`Tests/Code_Coverage/codex/`, `Firmware/Tests/Code_Coverage/codex/`, `Firmware/Tests/Integration_Tests/codex/`);
* `.github/`; `Tests/Code_Coverage/shared/specs/`;
* generated files: `Tests/Code_Coverage/claude/reports/BUG_REPORT.md`, `Tests/Integration_Tests/claude/reports/INTEGRATION_BUG_REPORT.md`, `Firmware/Tests/Integration_Tests/claude/oracles/*.csv` (their generators write them through Bash);
* `Firmware/Tests/Code_Coverage/baseline/test_cases.txt`;
* existing files under `Firmware/Tests/Code_Coverage/` **unless** the path is listed in `Tests/Integration_Tests/claude/plans/APPROVED_SHARED_EDITS.txt` (one path per line with the owner's approval date and the fix WO; the lead adds a line only after the owner approved that fix). New files under `Code_Coverage/` are blocked the same way (integration code lives in `Integration_Tests/`).

Block for subagents (when `agent_type` is present), in addition:
* `.claude/` (agents, settings, hooks) and `CLAUDE.md`, `AGENTS.md`;
* everything outside their allow-list:
  * `integration-test-writer`: `Firmware/Tests/Integration_Tests/**` except `codex/`, `oracles/*.csv` and the hooks' own tests are allowed; `Firmware/Tests/CMakeLists.txt` only when listed in `APPROVED_SHARED_EDITS.txt` (the single `include()` line in I0); `Code_Coverage/` paths only via `APPROVED_SHARED_EDITS.txt`; nothing under `Tests/` (documents are the lead's);
  * `stability-analyst`: `Tests/Integration_Tests/claude/charters/stability/**` and `Firmware/Tests/Integration_Tests/claude/oracles/**` (scripts and tests, not the generated `.csv`);
  * every other agent type (explorer, auditor, reviewer, summariser, coverage-track agents): no integration paths (they have no edit tools anyway; the hook is the second line).

### 2.2 `guard_bash.py` (Bash)

Parses the command with `shlex` per `&&` / `;` / `|` segment (fails closed: a command it cannot parse that contains `git` is blocked with a message to split it). Blocks:

* `git push`, except exactly `git push https://github.com/MadBull466/OpenFFBoard.git add-test-suite` (no `-f`, `--force`, `--force-with-lease`, `--mirror`, `--all`, `--tags`, `--delete`, `+` refspec, other refspec or remote name); blocked for every subagent;
* `git config`, `git remote`, `git -c remote.*`, `git -c user.email=<non-empty>`, `git submodule add`, `git filter-branch`, `git reset --hard`, `git push origin`;
* `git commit` unless it carries `-c user.name="MadBull.Ben" -c user.email=` (empty), the current branch is `add-test-suite`, and the gate evidence matches (2.3); blocked for every subagent; also `git checkout` / `switch` / `rebase` / `merge` / `stash` / `branch -D` for subagents;
* `cmake` with `-B` / `--build` / `--test-dir` / `-S . -B` pointing inside the repository (build directories must resolve under `../ffb-test-build/`); `cmake --preset` (in-tree binary dirs);
* package installs and downloads: `pip`, `pip3`, `apt`, `apt-get`, `dnf`, `pacman`, `brew`, `npm`, `curl`, `wget`, `git clone`, `git fetch`, `git pull`;
* write-like shell operations into protected paths (heuristic): `>`, `>>`, `tee`, `sed -i`, `cp`, `mv`, `rm`, `touch`, `truncate`, `patch` whose target resolves into a path `guard_paths.py` would block for this agent. The heuristic cannot see writes made by programs (a Python script writing a file); the gate's scope check (2.3) and `git status` are the backstop.

### 2.3 Gate script and evidence (`Firmware/Tests/Integration_Tests/tools/it_gate.py`, Python stdlib)

`python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --wo I1-3 --variants default,m32 [--full]`

For each variant (build directory `../ffb-test-build/it-<variant>`; variants `default`, `m32` (`-DFFB_TESTS_32BIT=ON`), and with `--full` also `san`, `clang`, `clang-san`, `nodsp`, `nodsp-san`, `cov`):
1. configure if needed, build with `-j$(nproc)`; a build warning in a file the WO touched is a failure;
2. `ctest --output-on-failure` (all entries, including `ffb_integration_tests`, the baseline check and `integration_oracle_tests`);
3. `check_baseline.py --build-dir ...`;
4. `ffb_integration_tests -r=xml` summarised by `summarise_results.py` (counts, failures with the first failed assertion, skips);
5. after all variants: `git status --short --ignored Firmware` shows nothing outside `Firmware/Tests/`; the changed and untracked files are inside the WO's `files:` list (parsed from the work order) plus the lead's document paths; no protected path changed; the other executables' compile commands are unchanged against the recorded `compile_commands` fingerprint (I0 onwards);
6. writes `../ffb-test-build/it-gate/<WO>/<timestamp>.json` and `latest.json`: WO, variants, per-variant counts and exit codes, `check_baseline` result, scope check, and a **fingerprint** (SHA-256 of `git diff HEAD --binary` plus the content of untracked files in scope), and prints one last line `GATE PASS <path>` or `GATE FAIL <reason>`.

`require_gate_evidence.py` (`SubagentStop`, writer only): blocks the writer's stop (`{"decision":"block","reason":...}`) unless its last message ends with `GATE PASS <path>` / `GATE FAIL ...` and, for PASS, the file exists, is newer than the writer's last edit and its fingerprint equals the current working tree. When `stop_hook_active` is true it lets the stop through (no loop) and the lead treats the result as unverified.

Commit check in `guard_bash.py`: a `git commit` passes only if the working tree has no unstaged changes and the fingerprint of the staged diff equals the fingerprint in the latest `GATE PASS` evidence of the WO named in the commit message (`WO: I1-3`). Commits that change only lead documents under `Tests/Integration_Tests/` (Markdown, no generated file) are exempt.

Windows note: the hooks use `python3`; on a Windows checkout the owner would change the command to `python` (or `py -3`). The tests themselves stay portable.

### 2.4 As implemented in WO I0-1 (differences from 2.1-2.3)

* Shared rules live in `.claude/hooks/guard_lib.py`; `guard_paths.py`, `guard_bash.py` and `require_gate_evidence.py` are thin wrappers. Tests: `Firmware/Tests/Integration_Tests/tools/test_hooks.py` (temporary git repository, hook scripts run with JSON on stdin).
* **Evidence format** (the contract `it_gate.py` must write in I0-4): `../ffb-test-build/it-gate/<WO>/latest.json` with `wo`, `result` (`PASS` / `FAIL`), `files` (`{repo-relative path: git blob hash, or null for a deleted file}` for every file the WO changed) and `fingerprint` (`guard_lib.fingerprint(files)`: SHA-256 over the sorted path / blob pairs) plus the per-variant counts. The commit check compares the *index* blob of every staged test-code file with `files`; the writer's stop check compares the *working-tree* blob. This replaces the whole-diff fingerprint, which the pre-existing changes outside the track (Codex's files, `AGENTS.md`) would always have broken.
* Commit classes: documents (`Tests/Integration_Tests/**`, `Tests/Code_Coverage/claude/**`) need nothing; configuration (`.claude/**`, `CLAUDE.md`, the hook tests) needs `test_hooks.py` to pass (the hook runs it); everything else needs a `WO: <id>` line and matching PASS evidence. Staged `codex/` paths and `AGENTS.md` are always refused; `-a`, `--include`, `--only`, pathspecs, `--author` and `--no-verify` are refused (commit only what is staged).
* `APPROVED_SHARED_EDITS.txt` lines count only with all three fields (`path | date | WO`).
* Heredoc bodies are treated as data; `$(...)` and backticks are checked as commands; redirects are resolved per command after any `cd`.
* `git stash list` / `show` are allowed for subagents.
* Gate as built in WO I0-4 (`Firmware/Tests/Integration_Tests/tools/it_gate.py`): the WO's file list is `Tests/Integration_Tests/claude/charters/wo/<ID>.files` (written by the lead); evidence also records `variants_requested`, `file_list`, `file_patterns`, `git_head`; `result` is `PASS` only with default and m32 (or `--full`) all passing, otherwise `PARTIAL` (refused by the hooks) or `FAIL`; WO sources are re-checked with `-fsyntax-only` so an incremental build cannot hide warnings; each variant's `CMakeCache.txt` must match its options; ctest must register `ffb_integration_tests` and `integration_tool_tests`; the other executables' compile commands must equal a recorded reference (`../ffb-test-build/it-gate/compile_reference-<variant>.json`, `--record-compile-reference`); timeouts on ctest and the integration run.
* Known limits found in use (2026-11-18): the guard sees the command text only, so shell variables are not expanded (`cmake --build $C/build` is read as a repository path and blocked: write build paths literally); a combined command is checked before any part runs (stage and commit in separate commands); `chmod` / `chown` treat their first word as the mode (fixed after a false block).
* `fake-auditor` `maxTurns` raised from 60 to 100 (owner, 2026-11-18): all four I0-2 audits ran out at 60.
* `integration-reviewer` `maxTurns` raised from 50 to 100 (owner, 2026-11-18): the I0-3, I0-4 and I0-5 reviews ran out at 50.

## 3. Per-work-order flow

```
lead: work order (charter file) ──> writer (one WO, both builds, gate) ──> gate evidence check (hook + lead re-run of it_gate.py)
   ──> integration-reviewer (fresh: diff + WO + charter) ──> findings
   ──> writer fixes (same writer via SendMessage, keeps context) ──> gate
   ──> lead QC (reads every KNOWN_ISSUE / characterization case, spot-checks oracles, re-runs the gate)
   ──> PROGRESS.md "Current state" + "Next action", HANDOFF.md, registers, INTEGRATION_BUG_REPORT regenerated
   ──> local commit (gate fingerprint matched), message "Integration: <WO> <summary>" + "WO: <id>"
```

* A work order is self-contained (the subagent never sees this conversation): requirements covered (ids), files to create / edit (the `files:` list the gate enforces), fakes and fixtures to use (with their audit result), the oracle for every expected value, out-of-scope items, the exact verification commands, and the expected new IB / INR numbers reserved by the lead.
* Escalation: a writer that fails twice on the same issue stops and returns it; the lead decides or asks the owner (spec gaps are never guessed). A reviewer "blocker" stops the commit until fixed or until the lead records a decision.
* Concurrency: explorer, auditor and reviewer instances may run in parallel on independent questions; never two writers.
* Context hygiene: one phase per session; verbose output stays in subagents and scripts; `PROGRESS.md` "Next action" and `HANDOFF.md` are updated after every WO so a fresh session resumes from them.
* Evidence, not claims: "done" is the gate's evidence file and the counts in it; a requirement is "covered" only with a passing test or an approved `KNOWN_ISSUE`.

## 4. What comes to the owner

| Item | When |
|---|---|
| This plan, the F1 charter, the agent and hook design, the `CLAUDE.md` text | now (planning session) |
| Every fix of a shared fake in `Code_Coverage/` with the list of changed coverage results and edited coverage tests | before the commit of that fix (and before the lead adds the path to `APPROVED_SHARED_EDITS.txt`) |
| Changes to `Firmware/Tests/CMakeLists.txt` beyond the planned `include()` line, to `CMakePresets.json`, or anything CI would need (described in `PROGRESS.md`, `.github/` untouched) | before the change |
| New specification needs, contradictory specs, intended-behaviour questions (IQ n, batched) | when found; work on the item stops |
| The stability classification of every "unstable inside the settable range" point | at the I4 gate (INR items) |
| End of every phase: phase report, gate evidence, bug report | stop and wait |
| Push to the fork | only when asked |

## 5. `CLAUDE.md` changes (proposed text; added only after approval)

### 5.1 Replace the heading and first line of "Model roles (driver track)"

```markdown
## Model roles (driver track and integration track)
- Opus 5.5 (`claude-opus-5-5`, effort medium on the driver track, high on the integration track) is the technical lead: ...
```
and add after the existing bullets:
```markdown
- Integration track agents (`.claude/agents/`): `integration-explorer` (Sonnet 5.5, medium, read-only), `fake-auditor` (Opus 5.5, high, read-only + build in `../ffb-test-build/audit/`), `integration-test-writer` (Sonnet 5.5, medium), `stability-analyst` (Opus 5.5, high, derivations and Python oracle scripts only), `integration-reviewer` (Opus 5.5, high, read-only). Only the lead delegates (no worker has the `Agent` tool); the writer of a work order never reviews it.
```

### 5.2 New section "Integration track"

```markdown
## Integration track (added 2026-11-18)
- Goal: prove the real firmware modules work together, with only the hardware boundary faked. Plan: `Tests/Integration_Tests/claude/plans/INTEGRATION_TEST_PLAN.md`; agents, hooks and gate: `AGENT_PLAN.md`; progress and every number: `Tests/Integration_Tests/claude/plans/PROGRESS.md` "Current state".
- Layout: test code in `Firmware/Tests/Integration_Tests/` (`claude/cases/test_f<n>_*.cpp`, `claude/oracles/`, `support/`, `fakes/`, `tools/`, `integration.cmake`), executable `ffb_integration_tests` (option `FFB_TESTS_INTEGRATION`, default ON); shared mocks, fakes, support and platform code stay in `Firmware/Tests/Code_Coverage/` and are reused. Documents in `Tests/Integration_Tests/` (`README.md`, `claude/{plans,reports,charters}`); specs stay in `Tests/Code_Coverage/shared/specs/`.
- Read order at the start of an integration session: this file, `Tests/Integration_Tests/claude/plans/HANDOFF.md`, `PROGRESS.md` ("Current state", "Next action"), the plan, the flow charter in `claude/charters/`, `Firmware/Tests/README.md`.
- Priority: flow F1 (HID PID -> EffectsCalculator / CMSIS-DSP -> Axis -> TMC4671) to completion (phase I5), then F2-F6 (ODrive, VESC, PWM, SimpleMotion, MyActuator), one phase each. Stop at the end of every phase.
- Oracles: PID 1.01 (`USB_PID1_01.pdf`) for the HID entry, the TMC4671 datasheet, MATH / hand derivations in comments, the documented nominal wheel plant for closed-loop tests. Hand derivations from firmware constants are `characterization:`.
- Both builds: every work order runs the 64-bit and the `FFB_TESTS_32BIT=ON` build (`tools/it_gate.py --variants default,m32`); phase gates run all variants.
- Bugs and NEEDS REVIEW items of this track have their own register (IB n, INR n) in the integration `PROGRESS.md` and their own generated report `Tests/Integration_Tests/claude/reports/INTEGRATION_BUG_REPORT.md` (`Firmware/Tests/Integration_Tests/tools/make_integration_bug_report.py`); a coverage bug hit again is cross-referenced ("same as bug n"), the coverage register is not changed. Questions: IQ n in `Tests/Integration_Tests/claude/reports/OPEN_QUESTIONS.md`.
- Fake fixes (owner's override of the hard rules for this track): a wrong shared fake in `Code_Coverage/` is fixed in place; coverage tests that passed only because of it are edited too; every such change lists the changed coverage results with reasons and needs the owner's approval before the commit; the path is then listed in `Tests/Integration_Tests/claude/plans/APPROVED_SHARED_EDITS.txt`; the coverage `PROGRESS.md` / `COVERAGE_AND_SCOPE.md` are updated and `BUG_REPORT.md` regenerated in the same commit.
- Hooks in `.claude/settings.json` enforce the hard rules (protected paths, git push / config / remote, commit identity and gate evidence, out-of-tree builds, no installs or downloads). Never disable or edit them without the owner's word.
```
