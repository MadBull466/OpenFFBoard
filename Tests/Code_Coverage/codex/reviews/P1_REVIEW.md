# P1 independent review, completed deferred body audit

2026-11-05. Assigned reviewer: fresh GPT-6.1 Sol, high reasoning effort, independent of the test authors. Scope: P1 gap tests in `Firmware/Tests/Code_Coverage/claude/cases/test_*.cpp`, excluding the driver subdirectory. This report records two confirmed test-quality findings. No firmware, vendor, baseline, Claude-owned test, shared infrastructure or git metadata was changed by this reviewer. Only this report was added inside the repository.

The six historical mutation guards all detect their corresponding reproduced mutation in a focused run and in the full `ffb_unit_tests` suite. A separate effect-state mutation survives the full suite because a `KNOWN_ISSUE` case accepts an unrelated setup failure. An ordinary ring-buffer test executes an uninitialised-value return; an isolated MemorySanitizer reproduction confirms the diagnostic. Passing default and ASan/UBSan runs do not establish that the suite is free of this class of undefined behaviour.

## Findings

### P1-001 — P1: an ordinary typed-access test executes an indeterminate-value return

Location: `Firmware/Tests/Code_Coverage/claude/cases/test_ringbuffer.cpp:330-336`, specifically the empty `w.get_as<uint8_t>(&ok)` call at line 334. Production path: `Firmware/FFBoard/Inc/ringbufferwrapper.h:57,65-68,81-87`.

Observed: `peek_as<T>` declares `T data;`, sets `*ok = false` on an empty buffer, then returns the uninitialised `data`. `get_as<T>` receives and returns that value, and the test initialises `v` from it before checking the failure flag. Casting `v` to void does not prevent the uninitialised return. This test has no skip or known-UB marker and runs as an ordinary passing case. The historical NEEDS REVIEW 24 already records the firmware API's uninitialised failure return; the test nevertheless exercises it.

Intended test property: a normal test must have defined behaviour. It can prove successful typed access and element counts on populated buffers. The empty typed-return path should be isolated as a documented UB reproduction rather than exercised unconditionally by the ordinary test. There is no evidence of a production caller instantiating this template; this finding concerns test reliability and does not assert a new reachable firmware defect.

Minimal reproduction, using the original headers and source unchanged:

```cpp
#include <cstddef>
#include "ringbufferwrapper.h"
int main() {
    RingBufferWrapper w(8);
    bool ok = true;
    [[maybe_unused]] uint8_t v = w.get_as<uint8_t>(&ok);
    return ok ? 1 : 0;
}
```

Detection experiment: compiled the C source as C with Clang 23, then linked the C++ reproduction and original `ringbufferwrapper.cpp` with `-O0 -g -fsanitize=memory -fsanitize-memory-track-origins=2 -fPIE -pie`. Both runs exited 1 with `MemorySanitizer: use-of-uninitialized-value`, tracing through `peek_as<unsigned char>` and `get_as<unsigned char>` to the reproduction call, with the origin at local `data`. No dependency was installed and no production source was changed. This is a focused three-translation-unit diagnostic experiment, not a fully MSan-instrumented suite.

Evidence retained outside the repository in `../ffb-test-build/codex-p1-review/`: `ring_msan_repro.cpp`, `ring_msan`, `ring_msan_c.o`, `ring_msan.log`. Exact build commands:

```sh
clang -O0 -g -fsanitize=memory -fsanitize-memory-track-origins=2 -fPIE \
  -I Firmware/FFBoard/Inc -c Firmware/FFBoard/Src/ringbuffer.c \
  -o ../ffb-test-build/codex-p1-review/ring_msan_c.o
clang++ -std=c++20 -O0 -g -fsanitize=memory -fsanitize-memory-track-origins=2 -fPIE -pie \
  -I Firmware/FFBoard/Inc ../ffb-test-build/codex-p1-review/ring_msan_repro.cpp \
  Firmware/FFBoard/Src/ringbufferwrapper.cpp ../ffb-test-build/codex-p1-review/ring_msan_c.o \
  -o ../ffb-test-build/codex-p1-review/ring_msan
```

Resolution boundary: Codex-only supplementary tests cannot remove the unsafe call from the existing Claude-owned case. The owner must authorise an edit to that case or a shared exclusion/replacement arrangement. The smallest correction is to remove its empty typed-return setup at lines 333-336, retain its successful typed-access assertions, and record the failure-return hazard separately. Do not repair the production header as part of this test review.

### P1-002 — P2: a running-effect state regression is hidden by a known filter defect

Location: `Firmware/Tests/Code_Coverage/claude/cases/test_hid_ffb_gaps.cpp:94-107`, particularly `REQUIRE(effect(index).state == 1)` at line 102 before the intended null-filter failure at line 104. The ordinary type-change case at lines 66-77 checks type/start time but does not independently check running state.

Observed on the original source: the bug-12 case fails first at the intended null-filter `REQUIRE`, so its current first failure is correctly attributed. However, a plausible type-change regression can replace that failure with the setup assertion and still be counted as an expected pass. The full suite does not independently protect the running-state property used by this test.

Mutation M7, applied only to a scratch copy of `HidFFB.cpp`, added a stop immediately after the type-change timestamp reset:

```diff
 if (effect_p->type != effect->effectType){
     effect_p->startTime = 0;
+    effect_p->state = 0;
     set_filters(effect_p);
 }
```

This stops every running effect whose type is changed. The focused bug-12 test exits **0**, reporting a failure at line 102 (`0 == 1`) and then “Failed as expected”. The complete default `ffb_unit_tests` run still reports **646 passed / 0 failed / 4 skipped**, exit 0. The first failure is no longer the claimed filter defect. The test's final torque assertion is never reached in either the original or mutated run.

This is a confirmed detection gap and a violation of the intended first-failure protection under the tested regression. It is not a claim that the source currently stops such effects, nor an independent interpretation of the unavailable HID PID specification. The requirement being protected is the existing case's own stated precondition that the changed effect remains running; the technical lead should preserve its existing status until specification questions are resolved.

Suggested Codex-only supplement: an ordinary case that starts a sine effect, sends a Set Effect report changing it to a damper, and checks that the effect still has running state, with no torque calculation through the known null filters. A second public-output variant can switch between two periodic types that require no filter, use a nonzero periodic magnitude and sample away from a zero crossing; confirm running torque remains observable after the Set Effect report. Derive any direction/phase expectations from the already accepted conventions, and retain PID-specific uncertainties as characterizations. No shared approval is needed to write a new Codex case, but integrating it into the current executable requires the shared CMake approval/process selected by the coordinator.

Evidence retained in `../ffb-test-build/codex-p1-review/`: `M7_HidFFB.cpp`, `M7.o`, `M7_tests`, `M7_focused.log`, `M7_full.log`, `mutation_results.json`. Reproduce the focused result with:

```sh
../ffb-test-build/codex-p1-review/M7_tests \
  -tc='a running effect changed to a damper by set effect has damper filters'
```

## Assessed scope and limits

Read first: `Firmware/Tests/README.md`; root `AGENTS.md` and `CLAUDE.md`; `Tests/Code_Coverage/codex/AGENT_WORKFLOW.md` and `BASELINE.md`; P1 sections of Claude's `TEST_PLAN.md` and `PROGRESS.md`, historical findings/known issues and P1 review history. Existing findings were treated as investigation context, not independently accepted requirements.

Seven files received detailed assertion/source review in this bounded assignment:

| File under `Firmware/Tests/Code_Coverage/claude/cases/` | Assessment |
|---|---|
| `test_cmdparser_gaps.cpp` | Capacity and malformed-address cases checked against `CmdParser.cpp`, ring capacity semantics and fixture reset. Boundary and malformed-input mutation guards discriminate; uncertain rejection/full-capacity semantics are labelled characterizations. |
| `test_command_handler_gaps.cpp` | Default dispatch, instance addressing, logging, flag overrides, lifetime and CDC/UART buffering assertions checked against command-handler/interface logic and fixtures. Existing bug-13 and bug-21 cases separate the two defects. No new concrete false positive found. |
| `test_hid_command_interface_gaps.cpp` | Report generation/decoding, ACK-with-address, error/NONE replies, queue limits, broadcast partial enqueue and busy-endpoint behavior checked against HID interface source. GETAT branch mutation killed. Local report structs share firmware layouts, so they do not independently validate HID descriptor/protocol layout. |
| `test_effects_calculator_gaps.cpp` | Square/rate, reconstruction guard, statistics command, Q-profile and dormant thread assertions assessed against relevant calculator/control paths and original cases. Phase and idle-rate mutations killed. The two square-phase samples alone do not establish the whole waveform; the test already documents that limitation and the phase defect separately. |
| `test_hid_ffb_gaps.cpp` | Filter restart/output tests and type-change/lifetime assertions checked against HID source, filters and fixture. Restart mutation killed. M7 survives: P1-002. |
| `test_filters.cpp` | RBJ low/high/band/notch oracles use independent trigonometric forms; tolerance is relative plus absolute floor. Peak/shelf coefficient transcription is explicitly a characterization, with separate independent gain/response properties. Difference-equation oracle reads coefficients, so it validates execution, while the separate coefficient cases validate design. Known Nyquist/copy hazards and fallback-vs-DSP limits retained. No new concrete false positive found. |
| `test_ringbuffer.cpp` | FIFO order/capacity/clamping/wrap and typed access assessed against raw/wrapper sources. P1-001 confirmed. Invalid-wrapper UB remains separately skipped as documented. |

The following eight files were read in a **first pass**, with source/fixture spot checks of their principal paths. They have **not received exhaustive per-assertion independent-oracle or mutation review** in this assignment:

| File | First-pass observations / remaining review |
|---|---|
| `test_error_handler.cpp` | Checked error storage/list split, sorting, callback counts, known clear defects and ISR resume discrimination against source. Protected raw-storage probe avoids relying on the known `getErrors` defect. Further independent mutations of add/dedupe/callback isolation remain. |
| `test_persistent_storage.cpp` | Startup flag resets and delayed restore checks agree with source. Test-local save/restore loops prove registry/dispatch and doubles; real SystemCommands integration belongs to later-phase tests. Further registry mutation/order experiments remain. |
| `test_device_interfaces_gaps.cpp` | Base delegation/no-op/default behavior checked against Encoder/MotorDriver implementations. No concrete issue found; no independent mutation performed. |
| `test_usb_hid_handler.cpp` | Default methods, registration/descriptor and known destructor defect checked against source. Local dispatch tests virtual callbacks; they do not validate real global USB callbacks. No independent mutation performed. |
| `test_cdc.cpp` | FIFO retry, failure setup, reset helper and long-length known issue checked against source. Retry-state reset is explicit. No independent mutation of CDC paths performed. |
| `test_can_handler.cpp` | Registration/handle lookup checked against CAN/CanHandler source. Local dispatch helpers test registry contents and test doubles, not `global_callbacks.cpp`; file documents this correctly. No independent mutation performed. |
| `test_uart.cpp` | Transfer, reservation, locking/failure, fake receive-size handling and callbacks compared with UART source. Local dispatch scope and existing known-issue cases observed. Full error-path oracle audit and mutations remain. |
| `test_rtos_wrappers.cpp` | Delay state, semaphore/mutex forwarding and cleanup checks compared with wrapper source and fake reset semantics. Existing vendor uninitialised/double-delete caveats are explicit. No new scheduler/fake fidelity claim or full vendor audit made. |

No P1 test file was wholly unread. The eight first-pass files remain unfinished for a complete P1 review. No coverage measurement, ARM target execution, descriptor/specification validation, exhaustive KNOWN_ISSUE audit, model-checking, preemption or hardware behavior verification was done. No new firmware bug is claimed.

## Detection experiments

The scratch harness `/tmp/codex_p1_experiments.py` used the existing default build's compile commands and immutable object files, compiled only copied mutated firmware translation units, and relinked replacement objects into separate executables. Neither baseline objects nor production sources were modified. Exact replacement strings, commands' outputs and focused/full logs are retained in the review build directory. One initially mistyped square-phase search found zero matches and performed no experiment; it was corrected before the successful M3 run below. No compiler/sanitizer flag was weakened.

| ID | Mutation | Focused killer | Focused / full exit |
|---|---|---|---|
| M1 | `CmdParser`: `freeSpace() > *Len` → `>=` | Exact-full terminated chunk characterization, line 74 | 1 / 1 |
| M2 | `CmdParser`: signed-address digit/sign inner `&&` → `||` | Malformed address characterization, line 117 | 1 / 1 |
| M3 | Square phase: `elapsed_time + phase` → `- phase` | 18000-phase torque samples, lines 86-87 | 1 / 1 |
| M4 | Constant-rate timeout: `micros() - lastCfUpdate` → `+` | Nonzero 5-second-uptime rate, lines 110/113 | 1 / 1 |
| M5 | Starting filter guard: state `!= 1` → `== 1` | Stopped-effect first torque is small, line 54 | 1 / 1 |
| M6 | HID GETAT reply discriminator: `==` → `!=` | ACK-with-address reply count, line 144 | 1 / 1 |
| M7 | Type change additionally writes state 0 | None; setup failure becomes expected | 0 / 0 |

This is six historical guard checks plus one targeted gap experiment, not a statistically representative mutation score for P1. The M2 expression is stated precisely in `mutation_results.json`; the historical prose does not identify which of the several conjunction tokens on its source line was changed.

## Execution results

Used existing verified baseline binaries; no full rebuild was needed because no test or source file was edited.

| Run | Result |
|---|---|
| Default full unit suite, random seed `20261105` | 646 passed / 0 failed / 4 skipped, exit 0 |
| No-DSP full unit suite, same random seed | 646 / 0 / 4, exit 0 |
| DSP ASan/UBSan full unit suite, same random seed | 643 / 0 / 7, exit 0 outside sandbox; no diagnostic |
| Original-case checker, default | 318 names present; 317 classifications checked, one excluded; 332 new cases, zero failures; exit 0 |
| Isolated empty typed-return MSan reproduction | Exit 1; use-of-uninitialized-value confirmed twice |

All full-suite runs exclude exactly `requesting the forces of a not existing axis is rejected`. DSP sanitizer additionally excludes `reconstruction of a constant signal is constant` and `reconstruction of a step never leaves the range of the input values and reaches the target`; existing skip decorators remain active. “Passed” includes failures accepted by `KNOWN_ISSUE`; these are not all ordinary successes. No no-skip crash run was attempted.

The first sanitizer invocation inside the sandbox completed its doctest assertions but exited 1 because LeakSanitizer could not inspect the process under the sandbox/ptrace restrictions. Repeating the unchanged command outside the sandbox succeeded, with LSan still enabled. The first failure is an execution-environment limitation, not evidence of a leak. Logs are `/tmp/codex_p1_{default,nodsp,san}_random.log`; the successful sanitizer XML is `/tmp/codex_p1_san_random.xml`.

## Handoff

Coordinator can assign a Codex-only ordinary regression for P1-002, then require a fresh review and rerun M7 to confirm detection. P1-001 needs the owner's decision on editing or excluding the protected unsafe case. No approval is needed for further read-only review; remaining eight first-pass files should receive their own bounded follow-up before declaring P1 fully reviewed.

## Deferred body audit completed — 2026-11-06

Fresh Sol high reviewer `/root/remaining_p1_p6` completed the previously deferred body reading. The eight files above were read in full (2,686 lines), including every assertion, test-local helper and fixture. This closes their **body-reading** deferral; it does not claim exhaustive fault coverage. No additional confirmed test defect was found. The earlier P1-001/P1-002 findings and owner decisions remain separate open work.

| File (full current body assessed) | Independent assertion/source assessment |
|---|---|
| `test_error_handler.cpp` (1–625) | Compared `Error`, all list methods, registration, printer enable/disable and thread callbacks with `ErrorHandler.cpp`/header. Distinct code/type pairs establish dedupe and clear selection; full-list tests preserve content and explicitly separate notification from storage. Raw read-only probe avoids the known public-list sorting defect; it proves storage, not public-list usability. Known issues 7/8/9 first fail respectively at remaining-list size, cleared code and temporary-error count. Printer send branch is explicitly absent because `SystemCommands` is not linked here. |
| `test_persistent_storage.cpp` (1–227) | Compared constructor/destructor registry and all delayed-startup branches with `PersistentStorage.cpp`. Waiting/nonwaiting handlers use separate addresses/values; repeated callback, duplicate request, destroyed handler and post-startup request distinguish eligibility, clearing and lifetime. Save/restore loops are test-local dispatch over real registry plus fake flash; they do not establish SystemCommands integration or physical persistence. |
| `test_device_interfaces_gaps.cpp` (1–119) | Compared base Encoder/MotorDriver methods and declarations. Signed extrema are only passed through without arithmetic; −2048/4096 and 13312/4096 independently give −0.5/3.25 rotations. Read counts establish virtual delegation, and the externally assigned encoder case avoids a null-to-null retention-only oracle. Stateless base tests need no global reset. |
| `test_usb_hid_handler.cpp` (1–131) | Compared complete base implementation. Registry/descriptor pointer replacements and distinct handler data distinguish selection. Fixture restores static pointers after local handler destruction. Dispatch bodies invoke test doubles through the real global pointer; no claim about TinyUSB global callbacks follows. Destructor issue 19 only compares the stale address, without dereferencing it. |
| `test_cdc.cpp` (1–175) | Compared complete CDC send/retry implementation and HostUsb recording semantics. Independent stream partitions 4+4+2, 2+drop, zero-capacity retry and consecutive replies constrain both bytes and retry state. Thread waits observe unsent second data before completion and both streams after release. Fixture explicitly completes a one-byte synchronization send to clear the otherwise persistent retry flag. Long-reply issue 20 first fails at sent length, with no user buffer lifetime hazard. |
| `test_can_handler.cpp` (1–238) | Compared CANPort/CanHandler sources and semaphore accessors. Two handles and kept/destroyed objects discriminate lookup/lifetime; same-handle last-wins is marked characterization. Test-local dispatch validates registry order and default virtual callback tolerance, not `global_callbacks.cpp`. |
| `test_uart.cpp` (1–746) | Compared complete UART/UartHandler source, declarations, HAL fake receive/abort/recording paths and reset. All seven configuration fields have independent subcases. Receive byte/address assertions distinguish user buffer, internal buffer and stale pointer; another-handle cases preserve pending ownership. Failure-release issues 16 and stale-buffer issue 22 reach their stated release/data observations first. Local TX dispatch is distinguished from real global callback integration; recording HAL TX copies data synchronously, so these tests cannot establish DMA buffer lifetime. Skipped null-port destructor case was read only. |
| `test_rtos_wrappers.cpp` (1–425) | Compared full `cthread.cpp`, `cmutex.cpp`, `csemaphore.cpp`, delay declarations, and active fake reset/delay/semaphore semantics. Delay sequence derives from first reference at tick 103, then deadlines 113/123/133; late/reset cases distinguish retained deadline from reinitialization. Counts and scope guards constrain forwarding/cleanup within the fake. Finite mutex timeout establishes failure only, not elapsed time; recursive fake counts do not model another task's ownership. Vendor uninitialized/unstarted and repeated-delete caveats stay explicit; no unsafe creation-failure path was exercised. |

Three fresh, bounded scratch experiments provide effectiveness/repair evidence beyond the body audit. Each selected exactly one case using unchanged baseline objects, copied production translation units and separate replacement objects; baseline files/objects were never changed.

| ID | Change / selected case | Observed discrimination |
|---|---|---|
| P1R1 | Startup eligibility guard → always true; `the startup callback restores the handlers waiting for a delayed restore` | Control exit 0; mutant exit 1 at lines 174/175: nonwaiting handler was called and changed. |
| P1R2 | UART TX completion handle guard → always true; `a transmit complete of another uart is ignored` | Control exit 0; mutant exit 1 at lines 400/401: extra completion event and lost taken state. |
| P1R3 | Scratch repair removes premature `clearTemp` break; `clearTemp removes all temporary errors` | Control exit 0 with expected defect failure at line 384 (and 386). Repair has zero failed expressions and exit 1 because `should_fail` became obsolete. This proves local repair sensitivity, not correctness of every error-handling path. |

Filtered safe random-order run of these eight files, seed `20261106`: **184 executed, 0 unexpected failures, exit 0** (173 ordinary and 11 expected-failure passes); skipped null-port crash retained. No ringbuffer file or baseline `effectsForces` case was selected. No sanitizer or whole-suite rerun was needed for report-only changes; original-case preservation remains coordinator gate evidence.

Durable evidence directory: `/home/ben/Documents/OpenFFBoard/testing/ffb-test-build/codex-review-remaining-p1-p6/`: `experiments.py`, `results.json`, `P1R{1,2,3}_{control,mutant}.stdout` (XML), compile/link logs and `p1_safe_random.xml`. Historical `/tmp/codex_p1_experiments.py` and `/tmp` logs referenced earlier are no longer available after temporary-directory cleanup; retained older sibling build objects/results have not been represented as recovered scripts. No new hardware, preemption, target execution, protocol/specification or full-suite mutation claim is made.
