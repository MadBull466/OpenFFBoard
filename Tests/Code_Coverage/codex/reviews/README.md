# Independent review: P0 through P10

> Final closeout status (2026-11-06): [CLOSEOUT_RESULTS](../CLOSEOUT_RESULTS.md) supersedes earlier staged counts and pending closeout statements. Coverage percentages below remain historical; no current/new-test percentage was measured.

Authorised by the owner on 2026-11-05: "Start the review process, Start from P0 to P10". This is the review scope; P11 expanded DSP is excluded. Historical completion/approval is not independent Codex verification. This cycle works through the phases and records incomplete work explicitly.

Allocation follows [AGENT_WORKFLOW.md](../AGENT_WORKFLOW.md): Astra high technical oversight; fresh Sol high reviewers; Sol medium writers after a justified test group is specified; primary agent coordinates and independently verifies results. No shared edits beyond the already-approved CDX-001 correction are authorised by this record.

| Phase | Subject | Current review state |
|---|---|---|
| P0 | Original-case safety net, build/exclusions, coverage and reporting tools | Review complete; six findings and 36 Codex regressions; revised patch accepted, owner-approved and applied; 36 ordinary regressions pass; affected integration checks complete |
| P1 | Core command/FFB/filter/error/storage/transport gap tests | All15 files body-reviewed; two findings; two Codex lifecycle cases accepted by fresh QC; unit registration approved/applied; new cases pass in all four affected variants |
| P2 | Axis, axes manager, main classes, FFB integration | Seven files assessed; motor-inhibit repair-discrimination gap reproduced and QC accepted; two permanent motor-side cases and target-glob proposal accepted by fresh Astra QC; target registration approved/applied; fresh default/no-DSP/GCC sanitizer target gates pass |
| P3 | Flash, EEPROM emulation, persistence | Five files/159 cases assessed; four isolated faults detected; bounded review accepted by Astra; no new confirmed defect |
| P4 | System commands, startup/callbacks, LEDs, voltage, descriptors | Eight files assessed; targeted repair and boundary experiments completed; no new confirmed defect |
| P5 | Buttons and analog sources | Twelve files/405 cases body-reviewed, final analog-axis gap closed by Astra; ADS repair-masking finding accepted; protocol limits qualified |
| P6 | Encoders and SPI/I2C buses | All8 files/390 cases body-reviewed; deferred intervals completed with additional control/mutation/repair checks; no new confirmed defect |
| P7 | CAN port/main classes and serial FFB | Five files/297 cases body-read; five sampled faults detected; no new confirmed defect; hardware/protocol limits qualified |
| P8 | RTOS C++ wrappers and cooperative fake | Eleven files/271 cases body-read; two reproduced test/fake findings; no shared fix applied |
| P9 | TinyUSB, original firmware-used DSP tier, Python helpers | All15 assigned files body-reviewed; NaN error aggregation gap reproduced; USB/Python review and safe configured runs completed; new guards accepted; registered and verified in three fresh target gates |
| P10 | TMC4671, ODrive, VESC requirements/peers/tests | All24 driver files/938 cases body-reviewed; three bounded oracle/scope findings accepted with explicit severity/scope qualifications; supplied-reference and hardware limits qualified |

Cross-phase evidence: all six P0-P10 C++ executables ran with random-order seeds 211 and 797, using the existing unit exclusion and normal skip decorators. All 12 runs exited 0 with unchanged counts and no unexpected failed cases. This is order-check evidence, not a substitute for reviewing assertions or exercising forced crash/UB cases. The originally recorded `/tmp/codex-review-order/` evidence was cleared between sessions; those results remain historical recorded observations and cannot now be inspected. Durable per-phase mutation evidence remains under the sibling build root.

Reports will identify exact documents, files and cases assessed, discriminating experiments, confirmed/suspected findings, limitations and follow-up tests. A phase is not complete merely because its suite is green or a scan found no obvious issue.

Fresh Astra high (`final_review_qc`) accepted the bounded P0–P10 review and new P2 cases in FINAL_REVIEW_QC.md; the new P9 guards are also accepted and the four target cases are integrated/verified after owner approval. Initial P0/P1 acceptance remains in INITIAL_QC.md. Findings and completed body reading do not imply hardware/specification conformance or exhaustive mutant coverage.
