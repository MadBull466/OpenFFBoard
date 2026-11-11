# P8 independent review

2026-11-05 · GPT-6.1 Sol · high reasoning effort. Scope: all eleven assigned `claude/cases_rtos/test_*.cpp` files, **4,297 lines / 271 cases**. Read every case body/local helper. P11 excluded. Only this report was added; vendor sources, shared fakes, Claude files and Git were not modified. Stable IDs: P8-1 is CDX-012; P8-2 is CDX-013.

## P8-1 — the Tasklet timeout known issue cannot distinguish a failed-lock repair from unchecked scheduling

**Medium test gap**, `Firmware/Tests/Code_Coverage/claude/cases_rtos/test_ctasklet.cpp:250–273`. The V12 case calls a second `Schedule(2,0)` while parameter 1 is pending, discards its bool result, and asserts only `secondReturned`. It does not check whether the pending lock was acquired, whether the second command was rejected, command count, or delivered parameters.

Two scratch repairs of `ctasklet.cpp:87` were independently compared:

- Unsafe partial repair: `xSemaphoreTake(DtorLock, CmdTimeout);`, leaving its failed result ignored before queueing the command.
- Checked repair: `if (xSemaphoreTake(DtorLock, CmdTimeout) != pdTRUE) return false;`.

Both give **identical existing-suite repair signals**: focused V12 case all 14 assertions pass and its `should_fail` flips; full `ffb_rtos_tests` gives 270 passed / 1 marker failure, with no other case distinguishing the repairs. Control is 271 passed / 0 failed, and the V12 first failure is the claimed `secondReturned` at line 271.

A scratch-only positive-control probe uses the existing public Tasklet/helper classes and explicitly starts its priority-3 task before the timer daemon. It requires the first schedule succeed, then checks: the second call returned false without blocking, exactly one daemon command remains, and running the daemon delivers only parameter 1. Checked repair: 1 case / all 14 assertions pass. Unsafe repair: three assertions fail—secondResult remains true, two calls are queued, and parameters `{1,2}` are delivered. Control also fails this intended-timeout probe, as expected.

Oracle: the existing V12 intended nonblocking-timeout requirement, the bool contract in `tasklet.hpp:145–148`, and `CheckForSafeDelete`'s documented synchronization with pending calls (`:181–185`; DtorLock protects deletion). The unchecked repair queues a second callback without acquiring the synchronization resource. The experiment demonstrates unauthorized additional queueing/payload delivery; a subsequent target use-after-destruction is an inference from the binary-lock design, not an executed crash claim. The public header describes CmdTimeout specifically as daemon-send wait; whether that policy should also bound the DtorLock wait is already the V12 interpretation. This review does not settle a wider timeout-policy ambiguity. Once adopting that intended V12 repair, the test should check both rejection/result and absence of a second queued/delivered call.

## P8-2 — fake EventGroup Sync accepts reserved control bits that the real kernel rejects

**Medium fake-fidelity defect and missing argument case.** `Firmware/Tests/Code_Coverage/platform/src/host_freertos_ext.inc:1390–1406` checks control bits in `waitFor`, then passes `bitsToSet` directly to unchecked `setGroupBits`. The vendored FreeRTOS V10.3.1 implementation `Firmware/Targets/F407VG/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c:210` instead calls `xEventGroupSetBits`, whose `:531` asserts that no control bits are set. This is supplied primary source, not an inferred protocol convention.

Minimal public-interface counterexample: start with `group.SetBits(0x2)`, then `group.Sync(1u<<24,0x1,0)`. The fake records no kernel fault and leaves `group.GetBits()==0x01000002`. The real call reaches the reserved-control-bit assertion. Existing invalid Sync case (`test_cevent_groups.cpp:237–242`) covers only reserved bits in the wait mask; the set mask is absent.

A scratch-only probe expects the specific `xEventGroupSetBits of control bits` fault and retained value 2. Unmodified fake: 1 case fails both checks (observed value 16777218). A scratch fake guard against control bits in bitsToSet makes the probe pass 11/11 assertions. The complete existing RTOS suite still passes **271/0/0**, showing that its current cases miss the fidelity defect. No shared implementation was edited. A future shared-fake correction requires the owner's approval; a regression case should cover independently the two masks and assert that invalid set bits do not leak into public state.

## Scope and execution

| Test file in `Firmware/Tests/Code_Coverage/claude/cases_rtos/` | Executed cases | Assessed behavior |
|---|---:|---|
| `test_rtos_setup.cpp` | 20 | per-family smoke; timer-service catch-up self-tests; V7/V8; fake queries |
| `test_cqueue.cpp` | 45 | copied payload/order, front/overwrite/peek, full/empty, blocked task and separate producer/consumer, exact timeout, ISR wake flag |
| `test_ctimer.cpp` | 38 | one-shot/periodic timing, delayed command effects, queue-full failure and recovery, ISR methods, active timer lifetime/V9 |
| `test_ctasklet.cpp` | 16 | deferred callback parameters/task, failed queue recovery, pending/deletion synchronization, ISR scheduling, V12/V13 |
| `test_cworkqueue.cpp` | 20 | work order/payload ownership, worker identity, producer backpressure, queued-work completion before deletion, create failures |
| `test_ctickhook.cpp` | 17 | SysTick identity/tick values, registration/enable/lifetime, critical-section use, characterized order/re-registration/self-disable |
| `test_cevent_groups.cpp` | 33 | any/all, clear-on-exit snapshots, multiwaiter broadcast, timeout/deletion unblocking, deferred ISR set/clear and queue-full rejection, masks |
| `test_cmem_pool.cpp` | 27 | item spacing/rounding/alignment/count, free/exhaustion recovery, preallocated/AddMemory, allocation/create failures, V16–V18 |
| `test_cread_write_lock.cpp` | 19 | concurrent readers, exclusive writer, reader/writer preference ordering, last-reader ownership and create cleanup |
| `test_ccondition_variable.cpp` | 13 | blocked wait, release/reacquire external mutex, signal FIFO/broadcast, lost no-wait signal, timeout and stale-waiter V7 |
| `test_csync_gaps.cpp` | 23 | mutex ownership/recursion, semaphore count/timeouts/ISR wakeup, create failure and V1/V3 |

Executed immutable baseline binaries without extra exclusions or `--no-skip`:

```
../ffb-test-build/codex-baseline-default/ffb_rtos_tests -r=xml
../ffb-test-build/codex-baseline-san/ffb_rtos_tests -r=xml
```

Default **271 passed / 0 failed / 0 skipped**, comprising 253 ordinary and 18 expected-failure cases. Sanitizer **266 passed / 0 failed / 5 skipped**, comprising 253 ordinary and 13 expected-failure cases. No sanitizer diagnostic; execution was elevated to allow LSan outside the ptrace sandbox, with no sanitizer suppression/options added. Existing five sanitizer skips are the two V1 uninitialized-handle cases, two V18 signed-overflow cases and V11 exception leak. No skipped UB was forced. Logs `/tmp/p8_run.xml`, `/tmp/p8_san_run.xml`.

All default known-issue first failures were inspected against case bodies: condition-variable 309; memory pool 276/289/304/319; sync 212/223/323/334/350/364; tasklet 271/303; timer 538; workqueue 341/370; setup 319/351. They match the intended defect observations in this run; no unexpected first-failure masking was confirmed. P8-1 is repair discrimination, not baseline wrong-first-failure. Fault-report cases extract the expected fault type and use fixture guards; an arbitrary unexpected fault cannot be treated as equivalent evidence merely because the whole case has `should_fail`.

Experiments: `/tmp/codex_p8_experiments.py`; sibling `../ffb-test-build/codex-p8-p9-review/p8_results.json`, `p8_tl_{control,unsafe,proper}_{focused,full}.log`, corresponding `*_probe_probe.log`, `p8_eg_control_probe.log`, `p8_eg_fixed_full.log`, `p8_eg_fixed_probe_probe.log`. Compile commands came from the baseline build; scratch objects alone were substituted at link time, and linker dependency-file output was removed. Each focused filter matched exactly one case:

```
-tc=a schedule with a timeout of 0 returns at once while an earlier run is still pending
-ts=CodexP8Probe
```

The two differently linked probe binaries each contain one case in that suite. Whole-RTOS runs are the scope of the survival/repair claims; no statement is made about all firmware executables.

## Source assessment and limits

Read functional bodies of queue, timer, tasklet, workqueue, event-group, memory-pool, reader/writer-lock, condition-variable and tick-hook wrappers, plus Thread::Wait and semaphore methods; checked corresponding public contracts for timeout/lifetime/alignment. Read RtosFixture/HostRtosExt, and fake-kernel paths for waiting/deadlines, copied queue items, send/receive/peek/reset, ISR wake calculation, semaphores/ownership/recursion, timer-command ordering/catch-up and event-group waiter snapshots/clear/delete. This is targeted fake-source assessment, not a full audit of all 1,471 fake-kernel lines. Heap and critical-section assumptions were compared with vendored heap_4 and Cortex-M4F port sources; the decisive Sync path was compared directly with event_groups.c. Other FreeRTOS API contracts were source-sampled, not exhaustively re-proved.

Positive evidence is stronger than execution counts alone: queue tests retain nonzero/asymmetric payloads across a blocked producer or consumer; event-group multiwaiters observe the pre-clear snapshot; CV cases prove lock release while blocked and reacquisition after wake; read/write tests hold two distinct tasks behind gates and observe exclusive order; workqueue deletion observes queued work finishing and worker removal. Timer cases bracket exact expiry and distinguish deferred commands from active state. These observations exercise real wrapper methods; the kernel scheduling itself remains a model.

The fake has no priority scheduler/preemption or priority inheritance. Explicit task activation order and priorities show that chosen states can exist on target; they do not simulate arbitrary interleavings. A wait on the test thread advances time while no daemon/other task runs, so such timeout tests establish forwarded deadlines and failure behavior under a withheld actor, not realistic global scheduling. Exception paths are host-only; target CPP_FREERTOS_NO_EXCEPTIONS stops in configASSERT. TickHook is enabled by the test while F407 disables it. Pool pointer size is eight on this host versus four on target, handled by size-dependent assertions.

Existing documented V9 repair restriction remains: some timers are deleted directly on the test thread and `test_ctimer.cpp:223–233` explicitly demands a queued command after deletion. A synchronous destruction repair can hit the fake's no-task-progress limitation and/or violate that observation. This was read and recorded, not rerun as a new experiment or assigned a new finding here. Do not interpret current green tests as accepting every valid V9 lifetime remedy. No broad mutation score, hardware timing or new production fix is claimed.
