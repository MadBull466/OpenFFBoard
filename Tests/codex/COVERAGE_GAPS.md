# Coverage and scope bookkeeping

Scope update: ST HAL is now in scope for P12 at the owner's request; see [P12 plan](p12/PLAN.md). Older HAL exclusions below are historical. P12 currently has source inventory only, with no measured real-HAL coverage. P11 is deferred.

> Final closeout status (2026-11-06): [CLOSEOUT_RESULTS](CLOSEOUT_RESULTS.md) supersedes earlier staged counts and pending closeout statements. Coverage percentages below remain historical; no current/new-test percentage was measured.

Discovery/baseline measured 2026-11-05; review status updated 2026-11-06. Measurements are in [BASELINE.md](BASELINE.md), full gcov output in [COVERAGE_BASELINE.md](COVERAGE_BASELINE.md). New Codex tests are tracked in NEW_TESTS; no post-integration coverage improvement has been measured.

## Measured execution versus unverified behaviour

The existing 87-file firmware ladder reaches 8,007/8,095 executable lines (98.9%), leaving 88 lines. The driver track reaches 2,376/2,596 (91.5%), leaving 220 lines: TMC4671 197, ODrive 6, VESC 17. Original tier C reaches 198/198 lines. These are executed-line measurements, not assertion or requirement coverage. Independent assertion/source review is tracked by phase in reviews/README.md.

TinyUSB reaches 1,337/1,572 (85.1%), including the compiled MIDI source (out of scope). Per-file values and source/function counts are preserved in the generated report. The older 48.0% entry in Claude's overview does not match this measurement; PROGRESS's P9-final numbers do.

Standalone DSP FastMath has 443/451 covered lines; BasicMath helpers 62/62. The aggregate 98.4% is over only 27 source files with data. **231 of 258 in-scope source files have no data in `ffb_dsp_tests`**, including Controller, Interpolation, Statistics and Filtering objects not referenced by its current tests. Some firmware-used DSP functions are exercised in the other executables and reported separately as original tier C; absence from this standalone executable is not proof of no suite-wide execution. G2-G5 are documented future P11 work. No full P11 denominator percentage has been established here.

No hardware-only exclusion ranges are active in the shared exclusion file. No exclusions were added, removed or reinterpreted. No aggregate branch, condition or mutation coverage percentage was measured. Selected fault-injection and repair-discrimination experiments are documented in the phase reports.

## Areas requiring the later audit

- Whether existing assertions prove the stated behaviour and use independent oracles.
- Whether KNOWN_ISSUE tests fail first for the described defect instead of a fixture/model fault.
- Whether fake resets, bus constraints, fault injection and time handling match the production interfaces and supplied specifications.
- Safety-related torque, emergency-stop and recovery scenarios identified in historical reports.
- P11 G1 pending fresh review, mutation checks, target-path caveats and tolerance decisions (including existing open question 17).
- Skipped and excluded crash/UB paths: present by design, not verified clean by the configured sanitizer runs.
- Any integration scenarios proposed in Claude's scope overview remain suggestions pending the owner's choice; this baseline phase does not approve expanding scope.

Out-of-scope platform/vendor/driver/MIDI areas and hardware/ARM/Windows/macOS limitations are enumerated in [DISCOVERY.md](DISCOVERY.md). Specifications were located, not independently validated during discovery. No register, frame, tolerance or hardware policy was invented.

## Confirmed behavioural and measurement gaps from the active review

- Coverage/reporting tools can omit missing workload denominators, accept collector failures or empty/failed inventories, and collapse identities (CDX-002–007). Revised corrections passed independent QC, were approved/applied, and pass the 36 ordinary regressions. The measured baseline percentages above have not been demonstrated wrong.
- Running-effect lifecycle changes can be hidden by whole-case expected-failure masking (CDX-009). Two new ordinary cases independently observe state and force; CMake registration is applied and verified.
- Critical-error flag assertions omit motor-side inhibition and later torque output (CDX-010). A proper/unsafe repair pair demonstrates the gap despite equal existing-suite results.
- ADS threshold assertions can become unreachable after a valid implementation-strategy change (CDX-011).
- Tasklet timeout assertions omit return value and rejection of the second callback (CDX-012); EventGroup Sync omits the reserved set-mask argument and the fake differs from the supplied kernel (CDX-013).
- Ordinary ring-buffer test UB (CDX-008) limits sanitizer-cleanliness claims; ~~supplemental unit runs explicitly omit that nonbaseline case~~ the test was fixed and runs normally. [Edited by Claude 2026-11-09: the CDX-008 quarantine was removed from `Firmware/Tests/CMakeLists.txt`; the Claude test `typed access reads single bytes and counts elements` was fixed instead (empty-buffer check moved to its own case). See `Tests/claude/reports/CODEX_REVIEW_AUDIT.md`.]

These findings concern assertion reachability, state transitions, error handling and fake fidelity even where lines are already covered. See REVIEW_FINDINGS and phase reports for exact evidence, recommendations and limitations.
