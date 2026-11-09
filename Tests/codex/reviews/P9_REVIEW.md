# P9 independent review

Reviewer: fresh GPT-6.1 Sol agent, high effort, assignment `/root/p9_review`, 2026-11-06. Repository HEAD `8a8041dcd6fbc8b706383f71cbda3f638961725b`, branch `add-test-suite`. Only this report was added in the repository by this assignment. Existing working-tree changes belong to other work and were left alone. No permanent tests, shared infrastructure changes, firmware/vendor edits, downloads, installs or commits.

**Result: one confirmed test-oracle defect, CDX-014 (medium/P2).** The original DSP comparison aggregators ignore NaN errors; two discriminating output-corruption experiments escape the entire `CmsisDsp` selection. USB and Python review found useful coverage and no additional must-fix defect. This is an assessment of the explicitly bounded P9 files below; the P11 `claude/cases_dsp/` expansion was excluded.

## CDX-014 — DSP error aggregation silently accepts NaN results

Location: `Firmware/Tests/claude/cases_target/test_cmsis_dsp.cpp:70–86`, `:144–150`, and `:729–746`.

`worstTrigRatio` updates its maximum only when `ratio > worst`. If the function under test returns NaN for a finite input, the error and ratio become NaN and that comparison is false. `maxAbsDiff` and the positional PID test use `std::max(worst, fabs(actual-expected))`; with NaN as the second operand, `std::max` retains `worst`. The final `worst <= tolerance` therefore can pass despite invalid output. A NaN in the reference also fails open in these aggregators.

These references otherwise have meaningful independent arithmetic: trigonometry uses host double `sin/cos` at the same float input; the biquad oracle evaluates the DF1 difference equation in double; PID uses the positional expression rather than reproducing the library's incremental state update. The defect is in accepting their comparisons, not in replacing those references with production output.

Scratch-only linker wrappers call the original functions and alter selected returned outputs. They do not alter the tests, reference calculations, production/vendor sources, baseline objects or known-issue decorators. `--wrap=arm_sin_f32` replaces values only for `0.2 < x < 0.21`; `--wrap=arm_biquad_cascade_df1_f32` replaces output element 0 only for `blockSize == 13`, preserving the original state update. Each is run separately, with NaN and finite-100 variants using exactly the same trigger.

| Probe | Altered outputs | Entire `CmsisDsp` result | Discriminating observation |
|---|---:|---|---|
| Sine, quiet NaN | 16 | exit 0; no unexpected failure | Invalid samples discarded by the sweep maximum |
| Sine, finite 100 | 16 | exit 1; one unexpected failure | `:268 worst <= 1.0`, expanded `4.98405e+06 <= 1` |
| Biquad block 13, quiet NaN | 4 | exit 0; no unexpected failure | Invalid block outputs discarded by `maxAbsDiff` |
| Biquad block 13, finite 100 | 4 | exit 1; one unexpected failure | `:462 maxAbsDiff(out, expected) <= TOL`, expanded `100.33 <= 2e-06` |

All four executions select the **whole original `CmsisDsp` suite**, not merely the affected case: 52 listed, 51 executed, V33's existing zero-stage crash case skipped. V32 remains an expected failure. This establishes escape across that selected suite; no claim is made that every unrelated suite in `ffb_target_tests` or P11 would miss these wrappers. The PID aggregator's identical problem is established by source examination, not a separately executed PID mutation.

Recommended correction, subject to ownership: reject non-finite actual/reference/error values before maximum reduction in each of these comparisons, and retain a regression proving a sampled NaN fails. An initial finite check outside the loop is insufficient. No firmware defect is diagnosed by these deliberate corruptions.

Durable evidence directory: `/home/ben/Documents/OpenFFBoard/testing/ffb-test-build/codex-p9-review/`:

- `dsp_output_probe.cpp`, `probe.o`, `dsp_output_probe`: wrappers and separately linked executable.
- `codex-p9-build-probes.py`: build script; derives compiler arguments from the immutable default compilation database and links baseline objects read-only, removing linker dependency-file output.
- `codex-p9-run-dsp-probes.py`, `dsp_results.json`, `trig-{nan,finite}.{xml,stderr}`, `biquad-{nan,finite}.{xml,stderr}`: exact runs, injection counts and results.

Run the two scripts from the repository root. Build/output writes require access to the sibling build directory. No broad mutation score is claimed.

## Test-body scope completed

Every assertion and helper body in all 15 assigned test files was read, including the cases marked expected failure, characterization and skipped. Counts are the current source/inventory, not historical phase totals.

| File under `Firmware/Tests/claude/` | Cases/tests | Assessed behavior |
|---|---:|---|
| `cases_usb/test_usb_setup.cpp` | 14 | Enumeration, descriptors, completed address change, CDC/HID smoke paths, suspend/resume, halt recovery/V21, OSAL objects |
| `cases_usb/test_usbd_core.cpp` | 93 | States/configuration, request routing, endpoint ownership/busy/halt, reset/unplug, wakeup/SOF, init/deinit, descriptor helpers, buffered/unbuffered streams |
| `cases_usb/test_usbd_control.cpp` | 22 | Read/write packetization, truncation, short/ZLP and status stages, stage callbacks, refused data, aborted transfer, no-data direction/V30 |
| `cases_usb/test_tusb_fifo.cpp` | 46 | Queue order, capacity/wrap including uint16 boundary, multi-byte items, overwrite/overflow, DMA info pointers, constant-address copies, mutex selection/release |
| `cases_usb/test_cdc_device.cpp` | 53 | Line coding/state/break, receive/backpressure, transmit/flush/ZLP, persistence, callbacks, firmware command integration and CDCcomm retry/reset/unplug |
| `cases_usb/test_hid_device.cpp` | 44 | Descriptors, protocol/idle, report IDs/control sizes, absent handlers/reports, IN busy/oversize/unconfigured cases, interrupt OUT, PID and HID commands |
| `cases_usb/test_usb_firmware_glue.cpp` | 14 | Main-class lifecycle, CDC-only/gamepad configuration, UID strings, USBdevice's actual thread |
| `cases_target/test_cmsis_dsp.cpp` | 52 | Original firmware-used trig/sqrt, biquad/init/state, spline/boundaries/extrapolation, PID/init/reset, sample variance/std |
| `cases_target/test_axis_speed_limiter.cpp` | 3 | Long overspeed characterization, independent near-limit torque arithmetic, below-limit recovery through real Axis |
| `python/test_dwc2_info.py` | 2 | Module/CLI smoke and F407 decode |
| `python/test_dwc2_info_cases.py` | 19 | Independent field masks, names/version/unknown values, row order, committed artifact, transposition, CLI |
| `python/test_generate_memory.py` | 3 | Committed generated files, duplicate rejection, enabled/exported counts |
| `python/test_generate_memory_cases.py` | 37 | CSV parsing/quoting, emitted arrays/header/counts, numeric duplicates, invalid rows, CLI/error/file behavior |
| `python/test_run_all_tests.py` | 44 | Synthetic XML/results/classification, filters, numbering/file attribution, bounded subprocess timeout/exit and messages |
| `python/test_summarise_results.py` | 10 | Synthetic console/XML/ctest/build parsing and summary/CLI errors |

Python total 115 = 61 vendor-helper tests + 54 reporting-tool tests. The six files and `helper_paths.py` were read in full. P11 tests and ARM reference patterns were not reviewed or executed.

## USB evidence and fidelity boundaries

Read `support/UsbFixture.h`, `UsbHost.h`, `HostDcd.h` and `platform/src/host_dcd.cpp` in full. Compared the exercised request/endpoint/state paths in `USB/device/usbd.c`, control-transfer body in `usbd_control.c`, HID API/control/completion paths in `hid_device.c`, and firmware `CDCcomm.cpp`, `USBdevice.cpp`, `global_callbacks.cpp` callbacks. Checked DWC2 endpoint disable/stall/clear/xfer implementation against the fake's relevant behavior. This is targeted production-source review, not an exhaustive source audit of every TinyUSB file or the entire fake RTOS.

Positive evidence: packets carry asymmetric data and are compared in order; class/control tests bracket the status stage instead of checking only eventual success; full-packet reads distinguish required ZLPs from short transfers; CDC backpressure retains unread bytes; HID busy tests retain the first report; reset/unplug cases prove subsequent usable communication. The four recording CDC callbacks and inert application vendor driver are explicitly test-only linkage differences, documented in their files. The actual USB thread case gives useful additional lifecycle evidence beyond pumping TinyUSB on the test thread.

Limitations retained:

- SETUP cancellation and pending-address behavior are simplified as documented in `HostDcd.h`; aborted SET_ADDRESS is not a target-register-faithful experiment. Completed SET_ADDRESS is the tested timing claim.
- Data toggles are generated/recorded by the fake, not independently supplied and validated by a real host/controller. The halt-clear DATA0 assertion checks this software model.
- No hardware FIFO/DMA allocation, actual bus timing, preemption, CRC/bit errors, or failed-transfer injection. MIDI code is linked because of F407 config but remains outside the reviewed behavior.
- `UsbdControlFixture::startVendorDevice` unconditionally calls `HostDcd::takeFaults()` at `test_usbd_control.cpp:92`, discarding any initialization/enumeration fault rather than asserting none. This is a local strictness weakness worth removing; no currently recorded baseline fault or whole-executable escape was demonstrated for it, so it is not promoted to a second confirmed defect.
- Report-handler stand-ins and the fake kernel cannot establish all target scheduling/lifetime effects. Unverified HID PID intent is not inferred from passing transport tests.

All 22 default USB known-issue cases were checked for their **first** failing expression in the actual XML. None first failed on a setup/model/cleanup assertion in that execution:

| Historical diagnosis / condition | First failure in test file |
|---|---|
| bug 42, bus suspend | `test_usb_setup.cpp:172`, `main->suspends == 1` |
| V21, halt recovery | `test_usb_setup.cpp:191`, CDC write ACK |
| bug 134, invalid configuration value | `test_usbd_core.cpp:282`, `r.stalled()` |
| bug 134, invalid configuration index | `test_usbd_core.cpp:294`, `r.stalled()` |
| V22, invalid alternate setting | `test_usbd_core.cpp:641`, `r.stalled()` |
| V22, interface status | `test_usbd_core.cpp:653`, `r.ok()` |
| V23, unused endpoint status | `test_usbd_core.cpp:752`, `r.stalled()` |
| V23, unused endpoint halt | `test_usbd_core.cpp:764`, `r.stalled()` |
| V31, nonexistent controller endpoint | `test_usbd_core.cpp:779`, `r.stalled()` after extracting the specific controller fault |
| V23, EP0 clear halt | `test_usbd_core.cpp:791`, `r.ok() || r.stalled()` |
| V24, clear halt of busy endpoint | `test_usbd_core.cpp:816`, endpoint busy |
| V25, single-write double overflow | `test_tusb_fifo.cpp:368`, FIFO count 4 |
| V26, multi-byte read-info pointer | `test_tusb_fifo.cpp:520`, pointer at byte offset 4 |
| V27, unknown report | `test_hid_device.cpp:268`, `r.stalled()` |
| bug 138, control vs interrupt payload | `test_hid_device.cpp:370`, equal handler buffers |
| bug 138, PID enable via SET_REPORT | `test_hid_device.cpp:380`, FFB active |
| V28, report after rejected oversize | `test_hid_device.cpp:445`, small report accepted |
| V29, report while unconfigured | `test_hid_device.cpp:464`, report rejected |
| bug 137, pending short reply/reset | `test_cdc_device.cpp:708`, new command reply |
| bug 137, pending long reply/reset | `test_cdc_device.cpp:728`, new reply suffix |
| bug 137, pending reply/unplug | `test_cdc_device.cpp:755`, new command reply |
| V30, IN request with zero data length | `test_usbd_control.cpp:376`, `r.ok()` |

These are existing firmware/vendor diagnoses, distinct from the new **test** defect CDX-014. Their baseline symptoms and relevant local source mechanisms were inspected; this review does not independently re-certify every USB specification citation or all proposed target consequences. No physical device or downloaded specification was used. A green `should_fail` case still does not guarantee discrimination among every possible repair.

## DSP and Python oracle assessment

Compared the DSP tests against the supplied sin/cos portable C implementations, sqrt/PID inline bodies, PID initialization, biquad loop/state arrangement, spline initialization/evaluation and sample variance/std code, plus `Axis::applySpeedLimiterTorque`. The spline oracle solves a dense linear system with pivoting instead of the vendor's tridiagonal recursion. Closed-form line/parabola and knot/boundary checks support the oracle. The extra query sentinel deliberately keeps the documented vendor over-read inside a valid array; these tests establish values, not absence of that existing over-read. V33's skipped crash and V32's sanitizer skip were not forced. The near-limit Axis oracle independently gives reduction `0.33*10 + 99*0.03*10 = 33`, torque about 9967 (tolerance 3); long-overspeed windup remains explicitly a characterization, not a newly settled safety requirement.

Read both Python production helpers in full. Compared DWC2 test masks against supplied `dwc2_type.h` GHWCFG2/3/4 declarations/comments, rather than deriving the expected masks from Python's ctypes declarations. Generator cases check independent literal rows, array membership/counts and actual CLI outputs; committed-file comparisons are consistency checks alongside those expectations. The pandas stand-in does not establish real pandas rendering, but the separate transposition probe records `.T` and the committed-table check compares cells. No dependency was installed.

Read the reporting-helper implementations for the functions the six test files call: XML inventory/result parsing, classification, source/number lookup, process timeout, Python record merging, console/ctest/build parsing and summaries. No end-to-end all-executable recovery/report run was performed. Synthetic parser coverage is not proof of full report orchestration. The timeout case intentionally launches a sleeping child killed after a bounded wall-clock deadline; this is an existing helper-process test, not simulated firmware time. It is a literal exception to the repository's no-sleep wording, not evidence of firmware nondeterminism.

In-memory generator repair/error probes confirm expected-failure handling:

| Probe | Observation |
|---|---|
| V35: numeric-address comparison repair | 1 unexpected success, 0 expected failures/errors |
| V35: rejection raises `ValueError` | 1 setup error, 0 expected failures |
| V36: filter incomplete rows before generation | 1 unexpected success, 0 expected failures/errors |
| V36: rejection raises `ValueError` | 1 setup error, 0 expected failures |

Generation/duplicate checking occurs in `setUp`, so raising fixes are not swallowed by `@expectedFailure`. Durable scripts/log: `codex-p9-python-probes.py`, `codex-p9-python-probes.txt`. Numeric fixes are probe examples, not changes proposed for protected vendor code.

## Executed validation

Read immutable existing binaries only; no rebuild of baseline directories and no baseline object/coverage-data mutation. No `--no-skip`, crash forcing, added sanitizer suppression or excluded-case override.

| Configuration and selection | Observed result |
|---|---|
| Default USB, random seed 9273 | 286 executed = 264 ordinary + 22 expected-failure; 0 unexpected failures/skips |
| DSP-off USB, random seed 7719 | 286 executed; 0 unexpected failures/skips |
| GCC ASan/UBSan USB, random seed 7719 | 286 executed; 0 unexpected failures/skips; stderr empty |
| Clang 23 ASan/UBSan USB, random seed 7719 | 286 executed; 0 unexpected failures/skips; stderr empty |
| Default `CmsisDsp,AxisSpeedLimiter` | 54 executed = 53 ordinary + V32 expected-failure; V33 skipped |
| GCC and Clang sanitizer `CmsisDsp,AxisSpeedLimiter`, seed 7719 | 53 executed each; 0 unexpected failures; V32/V33 skipped; stderr empty |
| Stdlib unittest discovery, six Claude Python files | 115 tests, OK with 2 expected failures |

XML also lists filtered-out target cases as skipped; the target counts above are the selected suites only. Initial sanitizer launches inside the sandbox could not finish LSan because ptrace prevents it. The elevated reruns used the same binaries/options and completed with no diagnostic; this is an execution-environment issue, not a test finding. Sibling output writes and sanitizer reruns were automatically approved; there was no approval rejection.

Evidence includes `codex-p9-usb.xml`, `codex-p9-usb-nodsp.xml`, `usb-{san,clang23-san}.{xml,stderr}`, `codex-p9-dsp.xml`, `dsp-{san,clang23-san}.{xml,stderr}`, and `codex-p9-python.txt` in the durable directory above. Baseline checker/coverage were not rerun because this assignment made no test/build change; existing phase figures were not recomputed.

## Documentation read and remaining boundaries

`Firmware/Tests/README.md` was the first read operation. Read its architecture/build/baseline guidance, relevant fixture/table material, and USB, DSP/P11 distinction, Python and limitations sections. The initial whole-file output was truncated in the middle; it is not evidence that every unrelated RTOS/driver subsection was independently read. Root `AGENTS.md`, `CLAUDE.md` and `Tests/codex/AGENT_WORKFLOW.md` were read in full; recursive rule-file search found no nested AGENTS/CLAUDE files under `Firmware/Tests` or `Tests`.

Read `Tests/claude/plans/TEST_PLAN.md` in full, including current scope and fake-fidelity/gate requirements. Read `Tests/claude/plans/COVERAGE_AND_SCOPE.md` (100 lines), preserving its explicit warning that numbers were not remeasured. Its stale TinyUSB batch-0 figures conflict with P9 final figures in PROGRESS; neither was independently recalculated here. Read PROGRESS's P9 results/batch-0/plan and relevant bug/NEEDS REVIEW rows via targeted ranges/search; the very large PROGRESS document was **not** read in full. Historical P9 mutation/coverage claims were treated as claims, not independently verified results. `BUG_REPORT.md`, `PROJECT_SUMMARY.md`, P11 charter/inventory, driver reports/specifications and the original authors' scratch mutation logs were not audited in this assignment. P8_REVIEW.md was consulted for prior boundaries and report context, not re-reviewed.

Within the 15 assigned test files, **body review is complete**. Remaining boundaries are production-source paths outside the described comparisons, exhaustive malformed USB descriptors/requests, all possible expected-failure repair strategies, actual hardware/target numerics/scheduling, platform portability, complete reporting orchestration, and the excluded P11 expansion. These limitations do not weaken the executed NaN-vs-finite discrimination for CDX-014.
