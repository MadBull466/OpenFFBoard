# P0–P10 closeout before a new phase

> Current status (2026-11-06): the authorized closeout is complete; see [CLOSEOUT_RESULTS](CLOSEOUT_RESULTS.md) and [final QC](reviews/CLOSEOUT_FINAL_QC.md#final-integrated-release-acceptance). Earlier counts and pending statements below are historical staged evidence.

2026-11-06. The assigned review is complete, and the approved P0/P1 fixes plus six C++ regressions are applied and verified. Review completion does not mean every finding is resolved. This is a recommended closeout sequence, not authority to edit shared or protected files. The question prompting this check did not approve additional code changes.

## Recommended before starting new-phase implementation

| Priority | Work | Evidence / reason | Completion condition |
|---|---|---|---|
| 1 | Resolve the ordinary ring-buffer UB in the normal test gate (CDX-008) | ~~Current safe verification uses a build-local exclusion; shared defaults still run the unsafe protected case.~~ Resolved 2026-11-09 by correcting the Claude test, with no exclusion (edited by Claude; see `Tests/claude/reports/CODEX_REVIEW_AUDIT.md`). | Owner disposition: correction by the protected-file owner, or approved shared quarantine plus a safe Codex replacement preserving defined successful-access coverage. Retain the UB finding separately. Validate normal documented gates and all 318 original identities; no production repair. |
| 1 | Register the 36 Codex tooling regressions in the routine gate | P0_P1_INTEGRATION.md explicitly records direct execution only. Current CTest Python entry discovers Claude's 115 tests, not the Codex group. | Approved narrow shared registration; discovery must not silently accept zero cases. Run the standard gate and demonstrate that a representative collector/inventory defect makes the regression fail. No forced-UB full report needed. |
| 1 | Correct EventGroup Sync fake argument validation with a permanent regression (CDX-013) | Supplied kernel asserts reserved set bits; fake accepts them. Scratch fix/probe passes while existing 271 cases miss it. See P8_REVIEW.md and FINAL_REVIEW_QC.md. | Owner-approved small shared-fake correction, Codex regression independently covering set/wait masks, unchanged valid behavior and full affected RTOS regression. Do not fix the vendor kernel. |
| 2 | Turn ADS and Tasklet probes into permanent discriminating cases (CDX-011/012) | Existing expected-failure cases can conceal a valid ADS repair or cannot distinguish safe/unsafe Tasklet repairs. | Codex-owned tests using independent positive controls; verify first defect failures and proper/unsafe repair discrimination. Establish Tasklet timeout policy before asserting a broader timeout contract. Integrate with approved narrow shared globs. |
| 2 | Establish reusable non-finite-aware numerical assertions before further numerical tests | CDX-014 original comparison reductions remain protected/NaN-blind; new guards cover only selected trig and stable-biquad workloads. PID maximum has the same source pattern. | Define valid-domain/non-finite expectations; check each sample/reference/error before reduction. Cover PID and any other reductions the new phase will reuse, with NaN/finite controls. No need to duplicate every already-valid numerical test. |
| 2 | Complete release bookkeeping and the routine verification contract | Some historical author/QC reports describe staged states; some old temporary evidence was lost. Current fresh full 9/9 uses a local ring exclusion. Coverage re-report uses frozen pre-new-test artifacts. | One current status index; durable command/results manifests; explicit normal exclusions, expected failures, direct-only groups and deferred findings. After closure changes, run the agreed full default gate plus affected DSP-off/sanitizer variants. Recollect coverage only if claiming current/new-test coverage. |

Shared CMake/tools/fakes/support changes need owner approval under AGENTS.md. Claude files remain protected; no proposed resolution assumes authority to edit them. Existing six C++ cases and the applied P0 tools need no repeated review merely to recreate their accepted evidence.

## Follow-up investigation or explicit deferral

These are narrower review observations or verification limits, not additional confirmed firmware bugs:

- **P5:** shifter CS assertion accepts more than the exact requested destination; the ADS helper's caller-buffer assertion is absent locally but later conversion tests observe delivered values.
- **P6:** SPI failed-start cases establish release without proving the next transfer succeeds; BiSS finite-wait case does not independently establish exact elapsed deadline.
- **P8:** timer-destruction tests/model constrain possible synchronous repairs; re-evaluate when addressing V9. The cooperative fake does not establish preemption or priority inheritance.
- **P9:** USB control fixture discards initialization faults through takeFaults; confirm it rejects unintended setup faults. This was not promoted to a second proven whole-suite escape. Protected original numerical aggregators remain weak outside the new guard workloads.
- **P10:** CDX-015 requires truthful local-policy versus vendor-spec attribution, not changing currently adopted framing. CDX-016 is a low local title/assertion issue with seven compensating cases; no duplicate response test is justified. CDX-017 merits distinct signed Hall-angle values, but no current production caller impact was established.
- **Source/specification boundaries:** absent sensor/HID/GVRET references, actual SPI pause/electrical timing, ARM floating-point paths, DMA lifetime, arbitrary interleavings and Windows/macOS execution remain unverified. Obtain relevant primary material or explicitly defer claims that depend on it; unrelated new work need not wait for every hardware-only limitation.
- **Known firmware/vendor defects:** review of their test symptoms is not production remediation. Crash/UB skips and sanitizer-skipped motor destruction remain open limits. Do not force them just to call a phase closed.

## New-phase readiness

Recommended: complete the priority 1 items, add the ADS/Tasklet regressions, and decide the remaining items that the next phase will depend on. Record explicit reasons for any deferrals. Then agree the new phase's requirement/source inventory, allowed interfaces, independent oracles/tolerances, fake boundaries, model/agent allocation and exit checks before bulk test creation. P11 remains outside the completed audit; its existing tests/charters are inputs to a fresh phase review, not independently approved new-phase work.

No finite review can prove that nothing else was missed. Additional focused investigations above may produce new findings; they must be reproduced and classified before being called confirmed defects.
