# Shared-file change record

Started 2026-11-06 for the owner's instruction: “Resolve all and record all changes to any shared files in a separate documentation file.” This record covers the cumulative shared diff from HEAD 8a8041dcd6fbc8b706383f71cbda3f638961725b, including earlier approved P0/P1/target integration and the closeout work. It does not imply authority to edit Claude-owned files, production firmware, vendor code, original baseline cases or .github. No such files are edited.

> [Edited by Claude 2026-11-09: the CDX-008 quarantine was removed from `Firmware/Tests/CMakeLists.txt`; the Claude test `typed access reads single bytes and counts elements` was fixed instead (empty-buffer check moved to its own case). See `Tests/claude/reports/CODEX_REVIEW_AUDIT.md`.]

## Authority and boundaries

Earlier explicit approvals covered the Python path fix, target registration and “Apply both” P0/P1 patches. The latest instruction authorizes necessary shared test-infrastructure changes to resolve the closeout. Default ring quarantine is a test-runner correction, not a production repair. No test assertion is weakened, baseline identity removed, sanitizer suppressed or dependency installed. Only host-test code/configuration/documentation changes are permitted.

## Final per-file record

| Shared file | Changes and reason | Consumers / verification |
|---|---|---|
| Firmware/Tests/CMakeLists.txt | Earlier: correct Claude Python path; register Codex unit/target cases. Closeout: ~~mandatory effective quarantine of CDX-008's unsafe nonbaseline case~~ (removed again by Claude 2026-11-09, see note below); register Codex RTOS/USB/driver source folders; add fail-on-empty Codex unittest CTest entry and the exact-first-failure USB setup-fault CTest gate. | Unit/baseline checker, target/RTOS/USB/driver executables, Python CTest. All five final normal gates pass; see CLOSEOUT_RESULTS.md. Original 318 manifest unchanged; safe Codex byte-access replacement preserves defined successful-access behavior. |
| Firmware/Tests/tools/check_baseline.py | Earlier P0: reject duplicate baseline/report identities and metadata loss, handle diagnostics, preserve print-current bootstrap. | Original 318 checker and Python regressions; prior four-variant integration passes. No production source or baseline catalogue change. |
| Firmware/Tests/tools/coverage_report.py | Earlier P0: obtain executable denominators from notes without counters; reject collector failure/blank/malformed documents; preserve explicit data-only files. | Coverage reporting and Codex regressions; prior full frozen-artifact report unchanged 8007/8095. Notes extension remains firmware/tier-A/drivers, not full P11 coverage. |
| Firmware/Tests/tools/run_all_tests.py | Earlier P0: corrected Python path, process/schema/nonempty validation and discovery safeguards. Closeout: collect both Claude and Codex Python groups so the full report includes the new regressions as well as CTest. | Existing 115 Python tests and own discovery/report tests, real inventories and both Python groups. Full forced-skip report is not used as a normal gate. |
| Firmware/Tests/platform/src/host_freertos_ext.inc | Closeout CDX-013: reject reserved set bits before Sync changes public state, matching supplied FreeRTOS Sync→SetBits assertion. Preserve separate wait-mask checks and valid inputs. | RTOS and USB executables share extended fake; permanent set/wait-mask/valid-rendezvous cases, full affected regressions and scratch guard-removal control. Vendor kernel unchanged. |

| Firmware/Tests/support/UsbFixture.h | Closeout: after successful host enumeration/mount, assert the first DCD fault is empty before derived fixtures can discard fault records. Does not clear faults or remove expected-fault APIs. | Full USB regression in all five configurations; permanent exact-first-failure negative CTest gate; independent guard-on and guard-removal discrimination. |
| Firmware/Tests/fakes/FakeOdrivePeer.h | Closeout CDX-015: comments/fidelity only, distinguishing source-supported 0x009 RTR and response payloads from local request-DLC and 0x017 RTR/response policy. | Driver regression in all five configurations; independent source/comment review. Wire/decoder behavior unchanged. |
| Firmware/Tests/README.md | Cumulative architecture/build/count/coverage clarification, Codex registrations, ~~mandatory ring quarantine~~ (removed), dual Python collection and links to current evidence. Its general 10/9-entry and 646-pass prose predates the final USB gate; final actual counts are 11/10 and those in CLOSEOUT_RESULTS.md. | Documentation reconciled against actual CMake and final command/results manifests; remaining historical counts explicitly superseded here. |

All eight changed shared paths are recorded above. Further shared changes require the applicable ownership authorization. Codex-owned test/helper files are inventoried in NEW_TESTS and closeout reports rather than mislabelled shared files.

## Final evidence

[CLOSEOUT_RESULTS](CLOSEOUT_RESULTS.md) records all five successful final gates and exact counts. [Final QC](reviews/CLOSEOUT_FINAL_QC.md#final-integrated-release-acceptance) accepts the implementation, permanent USB negative gate and combined release verification. Each sibling `codex-closeout-gate-<variant>/evidence/` retains exact commands, configure/build/test logs, source hashes and summary. All recorded owned-source hashes were matched again during final documentation closeout.

All 318 original identities remain; 44 ordinary Codex Python cases and the separate USB boundary checker run routinely. No local exclusion override, forced unsafe run, sanitizer suppression or current-coverage claim. Earlier integration remains historical in P0_P1_INTEGRATION and REGRESSION_RESULTS. No commit or push.

## P12 shared integration approval — 2026-11-07

The owner explicitly answered “Apply the prepared patch” to the P12 CTest ownership question. Applied exactly `Tests/codex/proposals/P12_CTEST_INTEGRATION.patch` to `Firmware/Tests/CMakeLists.txt`, preserving the previous shared changes. It adds opt-in `FFB_TESTS_ST_HAL` (default OFF) and `codex_st_hal`, which invokes the owned standalone gate with its own Clang 32-bit build under the selected out-of-tree build directory. Timeout: 300 seconds. No other newly authorized shared path is changed. Integration validation: the full default CTest run passed 12/12 entries, including the new real-HAL gate; exact retained log is sibling `codex-closeout-gate-default` with the continuation log `/tmp/p12-root-ctest.log` pending final archive. The standalone gate also passed before approval. This approval does not authorize production/vendor edits or a push.
