# P0–P10 closeout requirements

2026-11-06 · technical lead, GPT-6 Astra, high effort · assignment
`/root/closeout_lead`. This is the bounded acceptance contract for the owner's
request to resolve the closeout issues. It is not a results report. The coordinator
records implementation, verification and every shared-file change separately.
P11 remains outside this assignment. Protected Claude, baseline, firmware and
vendor files remain unchanged; necessary shared test-infrastructure corrections
are within the coordinator's recorded owner authorization.

## Acceptance matrix

| Item | Required closeout evidence |
|---|---|
| P0, CDX-002–007 | Keep the accepted tooling repairs and run all 36 Codex regressions automatically through CTest. A missing/empty discovery group must fail. Retain a discriminating collector/inventory negative control and the original-318 check. Existing accepted six-mechanism evidence need not be recreated without a relevant change. |
| P1, CDX-008 | Quarantine the named nonbaseline undefined-return case in the routine default gate, preserve its identity and defect record, and add safe Codex coverage of defined successful ring-buffer access. An explicit CMake cache override must not accidentally be represented as the documented default. Do not force the unsafe case or invoke undefined wrapper return paths in its replacement. |
| P1, CDX-009; P2, CDX-010 | Preserve and register the already accepted lifecycle/motor ordinary controls and defect regressions. Existing repair discrimination remains accepted within FINAL_REVIEW_QC's scope. Sanitizer-skipped motor destruction is a retained limit, not executed coverage. |
| P5, CDX-011 | Permanent public ADS threshold endpoint checks must accept a valid blocking or interrupt-driven implementation. An ordinary single-write/control case establishes the fake and oracle. The defect case must first fail at the missing requested threshold value, not at a required pending interrupt. Proper serialized repair passes; the current defect fails. |
| P5, lower observations | Assert the exact requested shifter CS destination, not merely membership in an allowed set. Add an independent caller-buffer/value observation for the ADS transfer helper, unless existing delivered-value evidence is explicitly mapped and proves the same public contract without dependence on implementation scheduling. |
| P6, SPI recovery | For failed-start transfer paths, prove the subsequent transfer starts and completes with observable payload/CS/release behavior. Merely seeing the port released is insufficient. Keep HAL and kernel guards. |
| P6, BiSS deadline | The existing finite-wait case proves fallback after the model signals timeout. It does not prove elapsed deadline. Record that scope precisely; see the model decision below. Do not manufacture a deadline proof by advancing time before manually resuming a task. |
| P8, CDX-013 | Correct shared EventGroup Sync set-mask reserved-bit validation, citing supplied kernel Sync → SetBits assertion. Permanent cases independently cover invalid set mask, invalid wait mask, retained user bits under the fake's recoverable-fault convention, and valid-mask behavior. Run affected RTOS regression. This is a fake correction, not a kernel correction. |
| P8, CDX-012 | Under existing V12 nonblocking-timeout intent, an ordinary control plus defect case checks public bool return, queued work and delivered parameters. Proper failed-lock rejection must pass; unchecked failed-lock scheduling must fail. The first defect failure must describe the timeout/rejection behavior. Do not generalize CmdTimeout into an independently established whole-operation deadline contract. |
| P8, timer V9 | Establish destruction safety through a task context where the daemon can progress, without requiring asynchronous deletion as the only valid implementation. Observe no callback remains able to use the destroyed object; never execute a dangling callback to prove danger. Existing protected queued-command assertions remain implementation-coupled and must be identified as such when evaluating a synchronous repair. No claim that every valid repair is accepted by the protected suite. |
| P9, CDX-014 | Provide a reusable finite-aware comparison oracle and its NaN/Inf/finite controls, plus an ordinary public PID mathematical regression. Reuse it for new numerical work. Existing finite trig/biquad guards remain accepted. The protected original reducers are not repaired; compensation is bounded to the selected public workloads and helper contract. |
| P9, USB initialization faults | Ensure an unexpected DCD fault arising during successful initialization/enumeration is asserted before the protected control fixture can discard it with takeFaults. A narrow shared successful-enumeration guard is appropriate; it must preserve explicit expected-fault APIs. Demonstrate clean setup passes and injected setup fault fails before discard. Run the full USB group. |
| P10, CDX-015 | Correct shared fake fidelity/provenance comments and publish the policy attribution below in Codex documents. Preserve adopted wire behavior. Identify protected ordinary exact-field assertions as local-policy enforcement where vendor support is missing. A fake rejection there is not independently proof of hardware rejection. |
| P10, CDX-016 | Record the protected title/body mismatch and seven compensating response cases. This closes the review interpretation; the protected editorial issue remains owner-local. Do not duplicate response tests solely to compensate for a title. |
| P10, CDX-017 | Add public setup_HALL coverage with distinct nondefault positive/negative values in all six fields and independently derived paired words. The retained default-constant mutant must be rejected. No production caller impact or actual packing bug is asserted. |
| Release bookkeeping | One current index links changes, exact commands/results, ordinary/expected-failure counts, exclusions/skips, retained defects and external limits. Record all shared changes in SHARED_FILE_CHANGES.md. Run the combined default gate, baseline checker and affected DSP-off/sanitizer configurations. Do not call frozen historical coverage current; recollect only if claiming current coverage. |

## Numerical oracle decision

For the finite valid-domain workloads in this closeout, actual sample, reference,
computed error, tolerance and any reduction input must be finite. Tolerance must
also be nonnegative. Reject NaN in either argument, positive/negative infinity,
invalid tolerance and finite operands whose subtraction overflows; do not let a
comparison or maximum reduction silently ignore them. A pure predicate/result
helper with direct ordinary tests makes invalid-input controls possible without
expected-failure markers. Comparison diagnostics should expose the failing sample.

The PID ordinary test uses public `arm_pid_init_f32`/`arm_pid_f32`, independently
specified finite gains and a signed asymmetric bounded sequence. Derive the
reference from `Kp*e + Ki*sum(e) + Kd*(e-previous_e)`, with zero initial history,
rather than production-derived A coefficients or returned state. Dyadic gains
and inputs simplify float error bounds. Compare every sample through the new
oracle; include positive and negative outputs and history-dependent behavior.
Finite corruption and non-finite controls must demonstrate rejection. Host
portable-C arithmetic is the execution claim; ARM intrinsic/FPU equivalence is
not established. No exhaustive invalid-domain CMSIS contract is inferred.

## ODrive policy attribution

The supplied 0.5.6 table/CSV and prior independent rendered-page review establish
0x009 call/response RTR semantics and the response layouts. They do not establish
request DLC 0, nor RTR request semantics for 0x017. The current choices remain
the adopted **local O14/L11 policy**: requests for both commands use RTR and DLC
0. This closeout neither changes that policy nor alleges the existing firmware
frames are wrong. Shared FakeOdrivePeer comments/confidence must distinguish the
source-supported fields from those policy choices, including the strict rejection
rules and response-on-request behavior for 0x017. The protected charter and test
wording are superseded for closeout interpretation by this explicit attribution;
no unauthorized edits or invented vendor facts are needed.

For Hall R52, the already reviewed signed vector
`{-123,2345,-3000,4095,12345,-23456}` yields paired words
`0x0929FF85`, `0x0FFFF448`, `0xA4603039` at the supplied-spec register addresses
0x34–0x36. Derive these as two's-complement low/high 16-bit fields, not from a
production response. The requirement is configuration discrimination, not a new
discovery of a driver fault.

## BiSS model decision and external limits

`HostRtos.h` documents, and non-EXT `xSemaphoreTake` implements, finite resumable
waits that expire on the next explicit activation without simulated time passing.
`ffb_target_tests` uses that model. Its BiSS fallback case therefore cannot prove
the exact elapsed wait deadline. Scheduling an activation after a chosen number
of ticks would test the script, not independent timeout behavior. A narrow
timeout-argument observer could prove requested ticks only. Changing the global
scheduler or linking a different kernel into the entire target suite is not a
necessary incidental closeout repair. Exact elapsed-deadline execution remains
**unverified because the target fake does not model it**, with the existing test
correctly scoped to fallback. This is a documented model boundary, not a resolved
timing guarantee.

Likewise, these remain explicit unverified claims: absent sensor/HID/GVRET
references and disputed descriptor semantics; physical SPI pause/electrical
timing; Cortex-M numerical paths/float conversions/alignment; DMA lifetime and
arbitrary interrupt/task interleavings; preemption and priority inheritance;
Windows/macOS execution; ODrive version/calibration differences and unsupported
VESC layouts. No downloads or invented protocol data are authorized. Supplied
references may support bounded additional tests, but documentation cannot turn
missing hardware/environment evidence into a verified result.

Known firmware/vendor defects, crash/UB quarantines and sanitizer skips remain
open production issues. Closing the actionable test-infrastructure gaps means
they are faithfully recorded and safely tested where possible; it does not mean
the underlying products have been repaired. P11 needs its own fresh requirement
and review cycle.

## Lead source assessment

Read the test README first (initial full output was truncated; relevant RTOS,
USB, driver and numerical portions were then consulted), root AGENTS/CLAUDE,
Codex workflow, closeout and final-QC reports, P8/P9/P10 reports, plan/resume
sections, the protected control fixture, shared UsbFixture/HostDcd,
non-EXT HostRtos semaphore waits, BiSS timeout case, PID reducer, existing Codex
trig/biquad tests and FakeOdrivePeer fidelity/decode paths. This assignment writes
requirements only and does not claim a new experiment, complete production
source audit or new independent PDF inspection. Prior reviewed specification
evidence is explicitly attributed to FINAL_REVIEW_QC/P10_REVIEW.
