# Driver Test Plan: TMC4671, ODrive, VESC

Extension of `Tests/claude/plans/TEST_PLAN.md`. Permanent rules are in the root `CLAUDE.md` and apply throughout, in particular: `Firmware/` is read-only except `Firmware/Tests/`, push only to the user's fork when asked (see `CLAUDE.md`), never write the user's email address anywhere, commit as `MadBull.Ben` with an empty email.

## Scope

IN scope, in this fixed order:
1. **TMC4671** driver (motor controller IC, SPI register interface)
2. **ODrive** driver (CAN)
3. **VESC** driver (CAN)

Since 2026-11-10 `TMCDebugBridge` is IN scope as phase P13 (`Tests/claude/plans/P13_TMCDEBUGBRIDGE_PLAN.md`, `TMCDEBUG` on in `ffb_driver_tests` only; complete).
Since 2026-11-10 PWM, SimpleMotion, MyActuator and MIDI are IN scope as P14-P17 (`P14_P17_REMAINING_DRIVERS_PLAN.md`, not started). Still OUT of scope: ST HAL, CMSIS core, FreeRTOS kernel, `Firmware/Targets/**`, CubeMX code. Their feature macros stay off in every executable.

What "testing a driver" means here: the firmware side of the conversation. We test what the driver sends, how it interprets what comes back, its state machine, scaling, limits, error handling and timeouts, against a simulated peer. We do not test the silicon or the external board.

## Stop rule

Stop at the end of every phase (D0, D1, D2, D3). Update `PROGRESS.md`, commit locally, report, and wait for my go-ahead.

## Roles and models

| Task | Model | Who |
|---|---|---|
| Understand an ambiguous specification | Opus 5.5 | lead |
| Decide what the suite must prove (requirements charter) | Opus 5.5 | lead |
| Map existing tests to requirements | Sonnet 5.5 | `driver-requirements-mapper` |
| Implement ordinary unit tests | Sonnet 5.5 | `driver-test-writer` |
| Write a specific set of edge-case tests | Sonnet 5.5 | `driver-test-writer` |
| Summarise thousands of lines of test output | Haiku 5.5, or better a script | `test-output-summariser` / `tools/summarise_results.py` |
| Check tests for gaps and false positives | Sonnet 5.5 first | `driver-test-reviewer` |
| Resolve serious test-design disagreements | Opus 5.5 | lead |
| Final quality control of each driver | Opus 5.5 | lead |

The main session runs Opus 5.5 at medium effort as lead (`claude --model claude-opus-5-5`, or `/model`). Workers are subagents defined in `.claude/agents/` (Appendix A) with `model: claude-sonnet-5-5` and `effort: medium`. The lead never writes bulk tests; workers never decide scope, requirements, or disputes.

Escalation: a worker that hits an ambiguous spec, a conflict between spec and firmware, or a reviewer/writer disagreement writes it into `PROGRESS.md` under "Lead decisions needed" and stops that item. The lead decides and records the decision and reason in the Decisions log.

## Method per driver (phases D1 to D3 follow this)

1. **Inventory (Sonnet).** List the driver's source files, public methods, commands/registers/frames it handles, state machine, timeouts, and the existing tests that already touch it.
2. **Charter (Opus).** From the supplied spec and the inventory, write a numbered requirements list `R1..Rn` in `Tests/claude/charters/driver/<name>_REQUIREMENTS.md`: each requirement is one observable behaviour, with its source (spec section, datasheet page, or "firmware behaviour, unverified"), priority, and the test oracle (exact bytes/registers/values expected). Ambiguities are resolved here by the lead or listed as open questions to me.
3. **Map (Sonnet).** Matrix requirement -> existing test(s) -> gap. Existing tests are never edited.
4. **Fake peer (Sonnet, reviewed by Opus).** Build the simulated peer with a fidelity header (model, source, not-modelled, confidence High/Medium/Low) and register it in the PROGRESS.md fidelity register. It must be strict (reject malformed traffic loudly), support fault injection (no reply, late reply, corrupted frame, bad CRC, bus error), and use simulated time only.
5. **Write (Sonnet).** Work orders of one requirement group each. One behaviour per test, test names are true statements, test name carries the requirement id (e.g. `[R12]`).
6. **Summarise (script).** Run, then `tools/summarise_results.py` condenses output to: counts, failures with first assertion, skipped, excluded. Agents read the summary, not the raw log.
7. **Review (Sonnet, fresh agent).** Gaps vs charter, false positives (tests that pass for the wrong reason, tautologies, assertions on the fake's own behaviour), mutation spot-check (see below), oracle sources, host-vs-target issues.
8. **Final QC (Opus).** Checks the requirement coverage matrix, every `characterization:`/`KNOWN_ISSUE` item, fidelity of the fake, and the gate. Signs off or sends back with a list.

### Test-oracle rules

- Expected values come from the spec/datasheet/vendor docs or a hand derivation shown in a comment. Never paste in what the firmware currently returns.
- Where only the firmware defines the behaviour, name the test `characterization: ...` and add a NEEDS REVIEW entry.
- Where the firmware contradicts the spec, write the test for the spec behaviour with `KNOWN_ISSUE("...")` (or `KNOWN_ISSUE_UB`) and log a suspected bug. The first failing assertion must be the one describing the defect (a `should_fail` test can hide a failure on the wrong line).
- Assert exact bytes on the bus (SPI words, CAN IDs, DLC, payload), not just "something was sent".
- Never invent register maps, opcodes, CAN arbitration IDs, scaling factors or captures. If the spec is missing, stop and ask.

### Mutation spot-check (reviewer)

In a scratch copy outside the repo, flip operators or constants on driver lines that gcov shows as executed, rebuild only the mutated file, run `ffb_driver_tests`, and record caught/missed. Target kill rate at least 85%; every survivor becomes a test or a recorded reason.

## Phase D0: Preparation (no test writing yet)

1. Read `CLAUDE.md`, `Firmware/Tests/README.md`, `Tests/claude/plans/TEST_PLAN.md`, this file, and `Tests/claude/plans/PROGRESS.md`. Confirm the baseline gate still holds before touching anything.
2. Confirm which model this session is running (you can see it in the session; if it is not Opus 5.5, tell me and stop). Create the agent files from Appendix A in `.claude/agents/` (the repo root `.claude/agents/`; ask me if you think they should not be committed). Do not edit `.github/`.
3. Inventory the three drivers: files, classes, feature macros (verify the real macro names in the firmware: expected `TMC4671DRIVER`, `ODRIVE`, `VESC` but check), dependencies on SPI/CAN/timers/flash, what the existing suite already compiles of them. Write a table into `PROGRESS.md` ("Driver track" section) and update the "Out-of-scope driver files" table: TMC4671, ODrive, VESC move to in scope, others stay out.
4. Check `Tests/shared/specs/` for the specification files. Needed: TMC4671 datasheet and register list; ODrive CAN protocol documentation for the firmware version the driver targets; VESC CAN protocol (packet IDs and scaling). List exactly which are present and which are missing. **If any are missing, stop and ask me; do not download.** Make sure `Tests/shared/specs/` is not committed (tell me the ignore entry you would add and ask; do not change global git config).
5. Propose the CMake layout: one new executable `ffb_driver_tests`, own feature header (copy the F407 macros, enable only the three driver macros, MIDI and the other drivers off), new source list, `claude/cases/driver/` directory. `ffb_unit_tests` and all other executables must stay byte-for-byte the same configuration. Show me the proposed diff before applying.
6. After my approval: skeleton `ffb_driver_tests` with one trivial passing test, `tools/summarise_results.py` (stdlib only), and the gate run. Add the new executable to the baseline checker if applicable (new tests must not be mixed into `baseline/test_cases.txt` for the existing 318 cases).
7. Update `PROGRESS.md`, commit locally, **stop**.

## Phase D1: TMC4671

Follow the method. Typical requirement groups (confirm against the real driver and datasheet, do not assume): SPI frame encode/decode (address, write bit, 32-bit data), register read/write helpers, init sequence and expected register values, motor type/pole pairs/PWM configuration, ADC offset/scale setup, encoder selection and init (ABN, hall, sensorless handling as present in the driver), position/velocity/torque limits, flux/torque target scaling and clamping, PID parameter setting and packing, state machine (uninitialised, calibrating, ready, error), error and fault flags, timeouts when the chip does not reply, command interface (get/set), and persistent-settings handling.

Fake peer: `FakeTmc4671` register file behind the fake SPI, with documented reset values, read-only vs read-write registers, and the SPI pipelining behaviour **only as the datasheet states it**. Confidence will be Medium at best for anything dynamic (it models registers, not the control loop).

Exit: charter signed by lead, matrix complete, all gate checks pass, reviewer report resolved, lead QC done, PROGRESS.md updated, local commit. **Stop.**

## Phase D2: ODrive

Same method. Typical groups (confirm): CAN arbitration ID construction (node id + command id), heartbeat/state parsing, axis state requests, get/set endpoint frames, controller mode and input setpoints with exact payload packing (float endianness), encoder estimates parse, error/disarm flags, timeouts and node loss, RTR handling, scaling and unit conversion, reconnect behaviour.

Fake peer: `FakeOdrivePeer` on `MockCANPort`, emitting heartbeat/estimates at documented periods in simulated time, rejecting malformed IDs/DLC, with fault injection (silent node, bus-off, wrong node id). Confidence per command documented.

Exit as D1. **Stop.**

## Phase D3: VESC

Same method. Typical groups (confirm): CAN extended-ID construction (packet id + controller id), status message parsing (RPM, current, duty, temperatures where used), command packing and scaling (duty, current, brake, RPM, position), fragmented/long packets and CRC where the driver uses them, timeouts, parameter handling, state mapping, error handling.

Fake peer: `FakeVescPeer` on `MockCANPort` with fidelity header, CRC implementation checked against known vectors from the spec, fault injection as above.

Exit as D1. Then final driver-track report (see below). **Stop.**

## Gate (after every batch, same as the main plan)

- All executables build with DSP on and off, MinGW and presets unaffected, sanitizer build clean (except the two named spline cases).
- `tools/check_baseline.py` passes; `ffb_unit_tests` unchanged (GCC 16: 316 passed, 0 failed, 2 skipped with the documented exclusion).
- `ffb_driver_tests` all pass, with `KNOWN_ISSUE`/`characterization:` items listed.
- `git status --short Firmware` shows changes only under `Firmware/Tests/`; no build artefacts under `Firmware/`; nothing pushed; `.github/` untouched; no email address in any file or commit message (`grep -ri` the staged diff for it before committing).
- Coverage of the driver files measured with gcov; uncovered code explained as hardware-only, unreachable, or missed.

## PROGRESS.md additions

A "Driver track" section with: current driver/phase, Next action, driver file inventory, specs present/missing, requirements coverage per driver (R-id -> test), Lead decisions needed, Lead decisions log, fake fidelity entries (FakeTmc4671, FakeOdrivePeer, FakeVescPeer), mutation results, NEEDS REVIEW and suspected bugs for drivers, per-phase commit hashes.

## Deliverables

- `ffb_driver_tests` with tests under `Firmware/Tests/claude/cases/driver/`, fakes, peers, requirement documents under `Tests/claude/charters/driver/` (or the location agreed in D0), `tools/summarise_results.py`.
- Updated `Firmware/Tests/README.md` (new executable, specs folder, how to run).
- Final driver-track report: requirements covered per driver, coverage numbers, fake fidelity and what it cannot prove, characterization/NEEDS REVIEW items, suspected firmware bugs, host-vs-target caveats, untestable code, and a statement of what still needs real hardware.

## Appendix A: agent definitions (create in D0 under `.claude/agents/`)

`driver-requirements-mapper.md`
```
---
name: driver-requirements-mapper
description: Maps the requirements charter for one driver to existing tests and lists gaps. Read-only except the matrix file.
model: claude-sonnet-5-5
effort: medium
tools: Read, Grep, Glob, Write
---
You map requirements to tests for ONE driver (named in the task). Read CLAUDE.md and the charter. Produce a matrix: requirement id, existing test names (file:line), status (covered / partial / gap), notes. Never edit existing test files. Never decide scope or resolve spec ambiguity: list it under "Lead decisions needed". Only write the matrix file you are told to.
```

`driver-test-writer.md`
```
---
name: driver-test-writer
description: Implements the unit and edge-case tests for one requirement group of one driver, from a work order.
model: claude-sonnet-5-5
effort: medium
tools: Read, Grep, Glob, Write, Edit, Bash
---
You implement tests for ONE requirement group, from a work order. Follow CLAUDE.md and Firmware/Tests/README.md. Only create or edit the test files named in the work order. Do not edit firmware, existing tests, shared fakes or CMake: report the change needed instead. Expected values must come from the spec/charter or a hand derivation in a comment; never from the firmware's current output. If the spec is ambiguous or contradicts the firmware, stop that item and report it under "Lead decisions needed". Put the requirement id in each test name. Build and run only what the work order says, and report with the results summary script, not raw logs. Never write any email address anywhere.
```

`driver-test-reviewer.md`
```
---
name: driver-test-reviewer
description: Fresh-eyes review of driver tests for gaps, false positives, weak oracles and fidelity problems. Does not write tests.
model: claude-sonnet-5-5
effort: medium
tools: Read, Grep, Glob, Bash
---
You did not write these tests. Review against the charter: missing requirements, tautologies, assertions that only check the fake, tests that pass for the wrong reason, oracles without a source, characterization or KNOWN_ISSUE misuse, nondeterminism, host-vs-target differences, MinGW/CI portability. Run the mutation spot-check in a scratch copy outside the repo. Report findings as a list with file:line and severity. Do not edit repo files.
```

`test-output-summariser.md`
```
---
name: test-output-summariser
description: Condenses long build or test output into counts, failures, skips and exclusions. Prefer tools/summarise_results.py.
model: claude-haiku-5-5
tools: Read, Grep, Bash
---
Summarise the given log: totals, each failing test with its first failed assertion, skipped and excluded tests, build errors with file:line. No opinions, no fixes.
```
