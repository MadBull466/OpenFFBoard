# P1 running-effect transition regression

2026-11-05. Writer: GPT-6.1 Sol, medium effort. Assignment: close the ordinary detection gap P1-002 / CDX-009, without editing production firmware, Claude's cases or shared infrastructure. Fresh independent review remains pending. CDX-008 / P1-001 remains an existing protected-test UB issue; this work does not repair it.

## Added scope and independent oracle

Two ordinary doctest cases in `Firmware/Tests/codex/cases/test_hid_effect_transition.cpp`, suite `HidFFB`:

1. `changing a running periodic effect type preserves force until an explicit stop` (line 13).
2. `changing a stopped periodic effect type stays silent until an explicit start` (line 37).

The functional requirement is separation of parameter updates from explicit start/stop lifecycle operations. This preserves the running-state precondition already asserted by `claude/cases/test_hid_ffb_gaps.cpp:102`; it does not independently establish USB HID PID specification compliance. The supplied PID specification remains unavailable and no protocol fields/layouts were invented. Inputs use existing shared `PidHost` public report helpers; observations use the shared `FfbFixture` public effect view and calculator-to-axis torque. Tests do not mutate effect storage directly.

Each test allocates a sine block and configures periodic magnitude 1000, offset 4000, phase 0 and period 100 ms, then changes its type to triangle. The mathematical bound offset +/- magnitude is [3000,5000], strictly separated from zero at every phase. Thus a running effect with full gain and the fixture's nonzero X direction must produce nonzero torque before and after the change, irrespective of timestamp resets, phase rounding or waveform sign conventions. These chosen values are inputs to an invariant, not expected values copied from firmware. No exact numerical torque is asserted. The type assertion confirms the update reached the intended block. Explicit stop must silence it; restart proves the new effect is usable. The stopped control begins with observable force, explicitly stops, changes type, observes silence and then obtains force after an explicit start. It discriminates stopped state from a missing or zero-magnitude effect.

Sine and triangle require no conditional filter, avoiding bug 12's null-filter path and bug 11's filter-type mismatch. No conditional effect calculation, reconstruction spline, empty typed read, no-skip execution or deliberate UB reproduction occurs in these cases. Shared `FfbFixture` supplies resets, simulated time and axis double; no new fake/dependency was needed. The checks avoid exact floating-point equality, ABI assumptions and overflow-scale inputs; no additional host-versus-target numerical claim is made.

Reviewed sources supporting the bounded safety/path assessment: `HidFFB.cpp` set-effect/start/stop paths; `EffectsCalculator.cpp` active-state gate, periodic sine/triangle and axis force paths; `support/{FfbFixture,FirmwareFixture,PidHost}.h`; existing HID/gap tests. Instructions read first: suite README, root AGENTS/CLAUDE, Codex workflow, lead charter, P1 review, and relevant Claude plan/progress sections. This is a narrow supplementary regression, not completion of the remaining P1 assertion audit or a new line-coverage measurement.

## Verification

Shared CMake has **not** been changed: normal configuration does not yet discover these Codex cases. A concrete unapplied proposal is `Tests/codex/proposals/P1_TEST_INTEGRATION.patch`; it adds the Codex glob to the existing unit executable with unchanged firmware/features. Owner approval is required before applying it.

Existing verified build objects in `../ffb-test-build/codex-baseline-{default,nodsp,san,nodsp-san}` were read only. For each variant, `/tmp/codex_p1_writer.py` selected that variant's `test_hid_ffb_gaps.cpp` entry from `compile_commands.json`, retained its flags/includes, changed only source/output to the new test and an own object, then appended the new object to the existing unit link command. The `-Wl,--dependency-file=...` argument was removed to prevent modifying baseline outputs. No framework, executable target or feature macro was added. All generated outputs are under `../ffb-test-build/codex-p1-writer/<variant>/`; compile/link/run argv and cwd are retained exactly in each `commands.json`. Compiler warnings originate in existing `ffb_defs.h`; compilation/linking succeeded. Sibling writes and sanitizer runs used approved escalation. Sanitizers remained fully enabled, with no suppressions or environment changes.

Reproduction driver, run from repository root:

```sh
python3 -B /tmp/codex_p1_writer.py
```

Focused command for each resulting binary (matches exactly two cases):

```sh
../ffb-test-build/codex-p1-writer/default/ffb_unit_tests \
  '-tc=changing a * periodic effect type *'
```

Regression command (substitute variant directory as appropriate):

```sh
../ffb-test-build/codex-p1-writer/default/ffb_unit_tests \
  '-tce=requesting the forces of a not existing axis is rejected,typed access reads single bytes and counts elements' \
  --order-by=rand --rand-seed=20261105
python3 -B Firmware/Tests/tools/check_baseline.py \
  --exe ../ffb-test-build/codex-p1-writer/default/ffb_unit_tests \
  --exclude-list 'requesting the forces of a not existing axis is rejected,typed access reads single bytes and counts elements' \
  --sanitize off
```

DSP sanitizer additionally excludes the documented `reconstruction of a constant signal is constant` and `reconstruction of a step never leaves the range of the input values and reaches the target`; sanitizer checker runs use `--sanitize on`. Existing skip decorators remain active. The added CLI exclusion `typed access reads single bytes and counts elements` avoids the confirmed CDX-008 indeterminate return and was verified absent from `baseline/test_cases.txt`; no shared exclusion default or marker changed. Therefore the runs below are explicitly **supplemented regression runs with an additional safety exclusion**, not the standard historical 646-case full-suite claim.

| Variant | New focused cases / assertions | Supplemented regression passed / failed / skipped | Original checker |
|---|---|---|---|
| DSP default | 2 / 16, all pass | 647 / 0 / 5 | OK; 318 present, 317 checked, 1 standard exclusion |
| No DSP | 2 / 16, all pass | 647 / 0 / 5 | OK; 318 present, 317 checked, 1 standard exclusion |
| DSP ASan/UBSan | 2 / 16, all pass | 644 / 0 / 8 | OK; 318 present, 315 checked, 3 standard exclusions |
| No DSP ASan/UBSan | 2 / 16, all pass | 646 / 0 / 6 | OK; 318 present, 317 checked, 1 standard exclusion |

All commands above exited 0. Sanitizer runs reported no diagnostic. Passed totals include existing known-issue expected failures. Checker reports 334 new cases, zero failures, in each variant. All 318 original identities remain present and their configured classifications unchanged.

## Mutation discrimination

M7 adds `effect_p->state = 0;` immediately after the unique type-change `effect_p->startTime = 0;` in a scratch `HidFFB.cpp`. Reused the review's immutable default `codex-p1-review/M7.o`, replacing only the matching HidFFB link object and appending the new test object. No repository firmware or baseline object was changed.

```sh
../ffb-test-build/codex-p1-writer/default/M7_tests \
  '-tc=changing a * periodic effect type *'
../ffb-test-build/codex-p1-writer/default/M7_tests \
  '-tce=requesting the forces of a not existing axis is rejected,typed access reads single bytes and counts elements'
```

Focused result: exit 1; 1 passed / 1 failed / 650 skipped, 14 passed / 2 failed assertions. The running test fails first at **line 28**, `CHECK(effect(index).state == 1)` (actual 0), and also at **line 29**, `CHECK(torque() != 0)` (actual 0). The stopped control still passes. Supplemented regression result: exit 1; **646 passed / 1 failed / 5 skipped**, with the same ordinary test/assertions as killer. This closes the specific M7 survivor without relying on the old expected-failure case or replacing its failure reason.

Raw evidence: `../ffb-test-build/codex-p1-writer/results.json`, per-variant `commands.json`, `focused.log`, `regression.log`, `baseline.log`, and default `M7_{link,focused,regression}.log`. No further repeated checks are required before fresh review unless the source changes.

## Coordinator application update — 2026-11-06

The owner explicitly said “Apply both.” The reviewed P0 tool and P1 unit-glob patches are applied;23resolved owned Python decorators removed. All36P0regressions pass ordinarily and both lifecycle cases pass through normal CMake in default/no-DSP and both sanitizer variants. Default full CTest 9/9; affected checks 3/3 elsewhere, with documented build-local CDX-008 exclusion. Exact counts/evidence and limitations: ../P0_P1_INTEGRATION.md. Earlier proposal-only statements above are submission-time history.
