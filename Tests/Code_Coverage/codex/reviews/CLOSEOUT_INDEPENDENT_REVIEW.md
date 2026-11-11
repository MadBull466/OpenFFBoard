> Edited by Claude 2026-11-09: this historical report mentions the CDX-008 ring-buffer exclusion/quarantine. That exclusion no longer exists (the Claude test was fixed instead). See `Tests/Code_Coverage/claude/reports/CODEX_REVIEW_AUDIT.md`. The text below is otherwise unchanged.

# Independent closeout review: foundations, RTOS, inputs and buses

2026-11-06. Fresh GPT-6.1 Sol, high effort. **Accept the reviewed implementation**, subject to the coordinator's combined configuration/baseline gates and current release ledger. This is a bounded independent review of CLOSEOUT_REQUIREMENTS.md's foundation, P5/P6 and P8 rows; it does not approve P9/P10 numerical, USB or driver work or establish current coverage percentages.

Read Firmware/Tests/README.md first, then root AGENTS.md/CLAUDE.md, Codex workflow/requirements, relevant protected plan/progress and prior review records. Read every body/helper in the new ring file (two cases), RTOS file (seven), inputs/buses file (eight), Python discovery runner/regressions, and the relevant shared CMake/reporter/fake changes. Sampled production ring/ADS/SPI/timer/tasklet implementations, fixtures and supplied FreeRTOS event_groups.c. Only this report is a repository edit by this reviewer. Firmware, vendor, protected Claude/baseline files and Git were not modified.

## Findings resolved during review

The original SPI retry helper proved payload delivery but omitted its required CS/release observations. The coordinator added exact CS0 selection, inactive CS after completion, the public isTaken state, and a separate resumable actor acquiring/releasing the public port semaphore. A fresh compilation then exposed an ordinary-case failure: unrelated fake CS outputs began low, so selectedCs returned -1. The final fixture explicitly initializes all external SPI CS pins inactive before any transfer. The independently rebuilt final tests pass. This is an explicit test-model setup precondition, not evidence that firmware startup clears every pin.

No implementation hold remains in this assigned scope. The inputs writer report's old 646/0/4 unit/baseline evidence must be qualified as historical evidence that included the unsafe ordinary typed-ring case; it is not evidence of a UB-clean closeout gate. This reviewer did not repeat that run. The coordinator owns the replacement normal-configured gate and original-318 verification. The writer scratch verify.py originally ended by reading a /tmp script solely to copy itself: remove that incidental dependency before treating it as a durable standalone reproducer.

## Independent experiments

All new compilations and link substitutions were written to sibling `../ffb-test-build/codex-closeout-review`, using immutable baseline compile commands/objects. No baseline object was overwritten. `results.json` and `discriminator_results.json` record commands, working directories, return codes, selected counts and first assertion failures; corresponding stdout/stderr files retain XML and diagnostics. Every executable invocation selected only the reviewed safe cases; no skip override or unsafe old unit checker was used. Filtered doctest “skipped” totals below are unselected cases, not additional executed coverage.

| Independent run | Observed result | Discrimination |
|---|---:|---|
| Safe ring, minimal executable | 2 passed / 0 failed | Successful typed byte peek/get, FIFO consumption/counts and defined empty untyped failure |
| Ring header copy omitting consumption | 1 passed / 1 failed | Nonempty FIFO test fails; empty untyped control remains green |
| RTOS, randomized seed 731 | 7 passed / 0 failed | Six ordinary cases plus one expected failure |
| Old fake, four Sync cases | 3 passed / 1 failed | Reserved set-mask case detects missing guard; other mask/valid controls pass |
| Checked Tasklet repair; new marker removed in copied test | 7 passed / 0 failed | Failed lock is rejected without a second command/payload |
| Unchecked Tasklet repair; same copied test | 6 passed / 1 failed | False acceptance, two queued commands and delivered second payload are rejected |
| Copied synchronous timer destructor with FIFO daemon barrier | 1 passed / 0 failed | New dormant task-context destruction control accepts synchronous completion |
| Inputs/buses, randomized seed 1021 | 8 passed / 0 failed | Four ordinary cases plus four expected failures; final CS/release checks included |
| ADS helper copy delivering into a different buffer | 0 passed / 1 failed | Exact caller-buffer observation rejects missing 0x12/0x34 delivery; adjacent guards remain meaningful |
| Valid blocking ADS threshold repair | 7 passed / 1 obsolete marker | Both requested threshold registers and completed caller pass; the KI becomes obsolete |
| Blocking ADS alternative writing high value into low register | 8 passed / 0 failed | Defect remains detected at low-register value, independent of interrupt scheduling |
| SPI failed-start release-only alternative | 8 passed / 0 failed | Both failed-start regressions still detect absent retry bytes despite released ownership |
| SPI failed-start HAL recovery plus release | 6 passed / 2 obsolete markers | Both real retry exchanges pass payload/CS/release observations |

Initial ring mutation linked against all protected unit test objects and appeared to survive because a separate translation unit supplied the original weak inline template instantiation. That was an invalid mutation setup. The reported ring experiments use a minimal executable containing only own tests, doctest main and the ring implementation objects, eliminating that substitution ambiguity. No unsafe wrapper-return branch was invoked in either experiment.

Commands for the independent batches, from repository root, are preserved as `verify_independent.py` and `verify_discriminators.py` in the review scratch directory. The discriminator batch uses the writer's retained copied repair sources as inputs, compiles them afresh, and links them against this reviewer's final test objects; it does not treat earlier writer counts as fresh results.

## Foundation acceptance

Configured current CMake with DSP off and hostile cache values `FFB_TESTS_EXCLUDE=review-sentinel` and `FFB_TESTS_EXCLUDED_ARG:INTERNAL=forged-sentinel`. The generated CTest unit command and baseline arguments retain `typed access reads single bytes and counts elements` alongside the user exclusion. The mandatory append therefore survives a cached override; no excluded unit case was run. CMake registers codex_python_tests through run_suite.py and the correct protected Python directory separately.

Ran `python3 -B Firmware/Tests/Code_Coverage/codex/python/run_suite.py --start Firmware/Tests/Code_Coverage/codex/python`: **39 tests pass**, comprising the existing 36 tooling regressions and three new discovery controls. Those real subprocess controls establish empty discovery fails, a real successful test passes and a real assertion failure fails. Discovery import errors also become failing unittest cases or a nonzero runner result. Independently invoked the report collector's python_results for each owner: **115 Claude results and 39 Codex results**, no ordinary failed result or collector problem. Thus both mandatory Python groups are actually collected; these counts do not claim every result is ordinary rather than an expected failure/skip.

Reviewed the collector/inventory negative controls and fail-closed guards. This closeout does not recreate already accepted six-mechanism tooling experiments or claim the full report runner is a safe substitute for the quarantined default CTest gate.

## Oracle, fake and lifetime assessment

The Sync guard matches supplied kernel Sync calling SetBits and SetBits asserting reserved bits. Per-bit set and wait tests independently seed state, require the intended fault, reject residual faults and check retained user bits; the valid highest-user-bit and zero-set-mask checks prevent simply rejecting all calls. Recoverable faults and retained state are explicitly host-fake conventions, not target continuation after configASSERT.

Tasklet first failure is the public return observation before queue/payload evidence. Setup/kernel guards precede that assertion; cleanup allows the actual blocked call to finish while the Tasklet is alive. The test adopts existing V12 zero-lock-wait intent and does not invent a whole-operation CmdTimeout deadline. The timer control stops and drains while the object is live, destroys in a resumable task and checks no timer/callback remains. The synchronous barrier experiment confirms that new control does not require asynchronous destruction. Protected test-thread queued-command assertions remain an implementation constraint, accurately recorded in CLOSEOUT_RTOS.md; no stale callback was executed.

ADS threshold first failure is requested low-register value, with distinct initial sentinels and endpoints. Public serialized-write control and valid/wrong blocking alternatives separate endpoint correctness from the chosen completion mechanism. The guarded caller-buffer case observes object bytes, avoiding host endian assumptions. The shifter regression checks exact requested index 1 and unrelated ownership, with completion before object destruction. Current first failure is exact selected index; no prerequisite HAL failure substitutes for it.

SPI current failed-start cases first fail on absent retry bytes, rather than a synthetic main-thread semaphore fault. The final successful retry observes distinct TX/RX data, completion, selected CS, inactive pin and a subsequent actor's real semaphore acquisition. Failed retry actors are destroyed while referenced device/buffers still exist. HAL/kernel fixture guards remain; no unsupported transfer failure or arbitrary DMA-lifetime claim is introduced.

The BiSS elapsed deadline remains unverified under the documented non-EXT activation-time timeout model. The existing fallback test and report state that boundary correctly. No sensor specification was supplied for this review; ADS/shifter observations retain their local-source attribution. Electrical timing, Cortex-M execution, arbitrary interrupt interleavings, preemption, cross-platform execution and actual firmware/vendor repairs remain outside these measured host results.
