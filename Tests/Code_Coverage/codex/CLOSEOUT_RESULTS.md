# P0–P10 final closeout — 2026-11-06

The authorized host-test closeout is complete. [Final technical QC](reviews/CLOSEOUT_FINAL_QC.md#final-integrated-release-acceptance) accepts the combined implementation and release gates against [CLOSEOUT_REQUIREMENTS](reviews/CLOSEOUT_REQUIREMENTS.md). This is the current status index; earlier staged counts and pending-work statements in phase reports are historical. P11 remains outside the independent review.

## Final verification

| Configuration / build suffix | CTest | Unit | Target | RTOS | USB | Driver | Baseline checked / excluded |
|---|---:|---:|---:|---:|---:|---:|---:|
| GCC DSP on / `default` | 11/11 | 652 | 1832 | 278 | 287 | 930 | 317 / 1 |
| GCC DSP off / `nodsp` | 10/10 | 652 | 1776 | 278 | 287 | 930 | 317 / 1 |
| GCC ASan/UBSan DSP on / `san` | 11/11 | 649 | 1716 | 273 | 287 | 920 | 315 / 3 |
| GCC ASan/UBSan DSP off / `nodsp-san` | 10/10 | 651 | 1661 | 273 | 287 | 920 | 317 / 1 |
| Clang 23 ASan/UBSan DSP on / `clang23-san` | 11/11 | 649 | 1716 | 273 | 287 | 920 | 315 / 3 |

Executable columns are doctest passed totals, including expected failures. Every executable has zero unexpected failures. Each configuration also passes 159 flash cases, 115 Claude Python tests (113 ordinary, two expected failures), 44 ordinary Codex Python tests, and the separate one-unittest USB setup-fault gate. DSP-on runs include the existing P11 executable: 218 passed normally, 191 passed / 27 skipped under sanitizers. Those regression runs do not constitute a P11 audit.

The baseline checker reports all 318 original identities present and RESULT: OK in every variant; 339 additional unit identities have zero failed results. Excluded identities are not executed checks. Normal exclusions are the original nonexistent-axis abort only [Edited by Claude 2026-11-09: the CDX-008 quarantine was removed from `Firmware/Tests/CMakeLists.txt`; the Claude test `typed access reads single bytes and counts elements` was fixed instead (empty-buffer check moved to its own case). See `Tests/Code_Coverage/claude/reports/CODEX_REVIEW_AUDIT.md`.]; DSP-on sanitizer runs additionally exclude the two known spline cases. Existing crash/UB, tasklet and motor-destruction sanitizer skips remain. No local FFB_TESTS_EXCLUDE override or sanitizer suppression is used in these final gates.

## Changes and disposition

[SHARED_FILE_CHANGES](SHARED_FILE_CHANGES.md) records all eight cumulative shared files. [NEW_TESTS](NEW_TESTS.md) inventories owned tests; the detailed independent experiments are in [CLOSEOUT_INDEPENDENT_REVIEW](reviews/CLOSEOUT_INDEPENDENT_REVIEW.md) and final QC.

- CDX-001–007 tooling corrections are applied; all 36 original tool regressions run routinely alongside three discovery controls and five USB-runner controls.
- CDX-008: ~~safely quarantined in the normal gate~~ no longer quarantined. [Edited by Claude 2026-11-09: the CDX-008 quarantine was removed from `Firmware/Tests/CMakeLists.txt`; the Claude test `typed access reads single bytes and counts elements` was fixed instead (empty-buffer check moved to its own case). See `Tests/Code_Coverage/claude/reports/CODEX_REVIEW_AUDIT.md`.] Two ordinary Codex replacements still cover successful typed byte access and defined empty untyped access; the production weakness (NEEDS REVIEW 24) remains open.
- CDX-009/010 lifecycle and motor discriminators remain registered. Motor destruction sanitizer skips remain a limit.
- CDX-011/012 now have permanent ADS and Tasklet controls and defect cases. Independent repair comparisons accept proper repairs and reject the relevant faulty alternatives. Underlying firmware/vendor defects remain open.
- CDX-013 reserved Sync set-mask validation is corrected in the shared fake, with independent set/wait/valid-mask regressions. The vendor kernel is unchanged.
- CDX-014 has a reusable finite-aware comparison, NaN/Inf/overflow controls and a public PID equation regression, in addition to the finite trig/biquad checks. Protected numerical reducers remain unchanged; compensation is workload-bounded.
- USB successful enumeration now checks unexpected DCD faults before a derived fixture can discard them. Its permanent negative CTest gate verifies the precise first assertion and rejects abnormal/sanitizer failures.
- ADS caller-buffer delivery, exact shifter CS ownership, SPI retry payload/completion/release, and stopped/drained timer destruction have bounded permanent coverage. Expected-failure markers retain the real defects.
- CDX-015 shared ODrive provenance now distinguishes supplied-source facts from adopted local request-framing policy; behavior is unchanged. CDX-016 remains an owner-local editorial mismatch with seven compensating cases. CDX-017 now has distinct signed nondefault Hall packing coverage and a default-constant mutant discriminator; no production Hall defect is alleged.

## Durable evidence and reproduction

Final build directories are `../ffb-test-build/codex-closeout-gate-{default,nodsp,san,nodsp-san,clang23-san}/`. Each contains `evidence/{reproduce.py,commands.json,configure.log,build.log,ctest.log,last-test.log,summary.json}`. The JSON command records preserve exact argv and working directory. Reproduce one configuration from the repository root, for example:

```sh
python3 -B ../ffb-test-build/codex-closeout-gate-default/evidence/reproduce.py default
```

The script configures, builds and runs the normal gate; other suffixes select the corresponding variant. Reproduction overwrites that build's evidence, so preserve existing artifacts first if historical comparison is needed. Builds require write access to the sibling build root; sanitizer execution may require the established environment permission for LSan. This continuation inspected all five summaries/command records and independently matched every recorded owned-source SHA-256 to the current files; it did not rerun completed gates after documentation-only edits.

Independent controls are retained under `../ffb-test-build/codex-closeout-final-qc/`, including `usb-permanent-live/`, `usb-runner-controls/` and the valid `usb-permanent-guard-off/` run. The earlier guard-off attempt with colliding executable/output paths failed before executing the child and is invalid discrimination evidence. The earlier same-class inline guard-removal attempt is likewise superseded. Historical unsafe unit runs and unavailable temporary artifacts remain historical, not accepted safe-gate evidence.

## Remaining boundaries and next action

No current/new-test coverage percentage is claimed; frozen coverage is historical. BiSS elapsed-deadline behavior remains unverified because the target fake expires finite waits on explicit activation without advancing time. Protected queued-timer assertions constrain synchronous repairs; the new task-context case accepts the independently checked synchronous repair. Hardware/electrical timing, ARM arithmetic, DMA lifetime, preemption, arbitrary interleavings, missing sensor/HID/GVRET references, version-specific driver questions and Windows/macOS remain unverified.

Known production/vendor defects remain open; this closeout repairs test infrastructure and improves discrimination. No production/vendor/Claude/original-baseline/.github files changed, and no dependency, download, hardware action, commit, push, remote or Git-config change occurred. Next action is owner selection of a new scoped phase; P11 requires its own requirements/review cycle. No closeout implementation or verification task remains pending.
