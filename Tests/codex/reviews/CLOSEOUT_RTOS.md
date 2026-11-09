# RTOS closeout implementation and evidence

2026-11-06. Scope: CDX-012, CDX-013, and the P8 V9 repair-model observation. Owner authorized necessary shared test changes for closeout; no firmware/vendor/Claude source was edited. New cases and the reproducer are in `Firmware/Tests/codex/cases_rtos/`. Counts below are measured scratch-link results, not a claim of sanitizer, target, cross-platform, preemption, or priority-inheritance validation.

## CDX-013 — corrected fake and permanent argument controls

`Firmware/Tests/platform/src/host_freertos_ext.inc`, `xEventGroupSync`, lines 1400–1404: five inserted lines reject `bitsToSet & eventEVENT_BITS_CONTROL_BYTES`, record the SetBits control-bit assertion, and return before mutation. Supplied FreeRTOS V10.3.1 `Firmware/Targets/F407VG/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c:210` calls `xEventGroupSetBits` from Sync; `:531` checks reserved set bits. This test configuration uses 32-bit ticks and mask `0xff000000`. Existing valid wait-mask behavior is untouched. Recoverable fault reporting and retained state are fake conventions; real `configASSERT` stops normal execution.

Four ordinary cases independently cover every reserved set bit, every reserved wait bit, zero wait mask, and valid highest application bit 23 with rendezvous clearing and zero set mask. Each loop iteration creates its own group, seeds state 2, requires the specific fault and absence of other faults, and checks state separately. The valid case verifies returned snapshot and surviving unrelated bits. The old fake fails the set-mask case on each bit's return, missing fault, and state (24 failures), while the other three controls pass. Corrected fake passes all four.

Consumer impact: `host_freertos.cpp` includes this file under `HOSTTEST_RTOS_EXT`, used by RTOS and USB executables. Invalid Sync set masks now record a fault; legitimate Sync calls are unchanged. The coordinator owns integration CMake and the central shared-change ledger and runs the final RTOS/USB/default/baseline gates.

## CDX-012 — public outcome and payload discrimination

Two permanent cases: ordinary accepted-schedule control and a new `KNOWN_ISSUE` for the existing V12 zero-lock-wait intent. The latter explicitly runs a priority-3 caller before the priority-2 daemon, requires the first schedule to succeed, checks model guards before the first defect assertion, then observes return, false acceptance, exactly one queued command and only payload 17 after drain. Current vendor blocks before the return flag and later delivers both 17 and 29. Cleanup lets the blocked call complete before object destruction; no stale-object callback is executed.

This preserves the timeout qualification: `tasklet.hpp:142–148` describes CmdTimeout as daemon-send waiting. It does **not** independently promise a general deadline covering every internal wait. The regression adopts the already documented V12 lock-wait interpretation and makes its failure handling discriminating. Any repair that chooses a bounded DtorLock wait must reject failed acquisition instead of queueing an unprotected callback. A broader API timeout policy remains a contract decision, not an invented test requirement.

On a copied test translation unit, only this new KI decoration is removed. Against actual vendor: first failure `CHECK(returned)` (line 101), then acceptance and payload failures. A copied checked repair (`if (xSemaphoreTake(DtorLock, CmdTimeout) != pdTRUE) return false;`) passes all seven closeout cases. A copied unchecked repair (`xSemaphoreTake(DtorLock, CmdTimeout);`) fails exactly the new Tasklet case on three independent observations: acceptance true, two queued commands, and delivered `{17,29}`. The ordinary success control passes both. Actual vendor files and the older expected-failure decorations remain untouched.

## V9 — task-context lifetime control and precise disposition

A new ordinary integration case starts a real periodic Timer, observes one callback, stops and drains it, establishes dormant state through `IsActive`, deletes in a priority-3 resumable task, drains the daemon, advances three more periods, and verifies no timer/no callback. Deleting in task context permits a synchronous destructor to wait for daemon progress; the test imposes no queued-command requirement after destruction. This covers the safe stop/drain/destroy consumer path and preserves the existing V9 finding without invoking stale pointers.

The original `claude/cases_rtos/test_ctimer.cpp:206–215` explicitly observes that destruction on the **test thread** queues a command. That is a characterization of the current asynchronous implementation, not a general destructor acceptance contract. `timer.hpp:148–150` declares a destructor without defining asynchronous completion; the fake documents that a test-thread wait cannot run the daemon. No safe general kernel change can make all such test-thread calls behave as real scheduled tasks without changing that test-model boundary. Therefore this observation is dispositioned as a protected implementation-characterization constraint, not another vendor defect or an authorization to alter vendor code. A future V9 implementation repair must reassess the old characterization and task-thread construction/destruction cases with their owner; this closeout does not certify that every possible synchronous repair passes the protected suite. The active-timer vendor defect remains reported, never fixed.

## Reproduction and measured results

From the repository root:

```
python3 -B Firmware/Tests/codex/cases_rtos/verify_closeout.py
```

The stdlib-only script reads compile commands and immutable objects from `../ffb-test-build/codex-baseline-default`, compiles only own/copied sources and the authorized shared fake into `../ffb-test-build/codex-closeout-rtos`, removes dependency-file linker output, and never overwrites baseline objects. It enforces expected process statuses and exact selected success/failure counts, preventing zero-test acceptance. `commands_results.json` retains argv, cwd, status and case/assertion metadata; `<label>.stdout/.stderr` retain full XML/build diagnostics. Sources, objects and binaries are retained beside them. An initial script guard counted both destructor and Schedule takes and stopped before repair compilation; the final script narrows replacement to Schedule and reruns all commands successfully.

| Run | Selected result | Meaning |
|---|---:|---|
| `current_full_run` | 278 passed / 0 failed / 0 skipped | Original 271 plus seven additions; 19 expected failures total, including new V12 |
| `current_tasklet_run` | 0 passed / 1 failed | New KI removed in copy only; first failure is intended return observation |
| `old_fake_run` | 3 passed / 1 failed | Missing reserved set-mask assertion detected independently of other masks |
| `proper_run` | 7 passed / 0 failed | Checked copied repair, new KI removed |
| `unsafe_run` | 6 passed / 1 failed | Unchecked copied repair rejected on result/queue/payload |

The new fake and six ordinary additions introduce no new exclusions. Existing sanitizer skip policy is unchanged. No skipped UB/crash is forced. Full default/sanitizer/DSP-off integration and original-318 checker results are coordinator-owned and must be recorded in the current release manifest; this report does not substitute scratch linking for them. No coverage percentage is claimed. No hardware, electrical timing, arbitrary interleaving, or Windows/macOS result is claimed.
