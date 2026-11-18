# Audit group D: `HostDcd`, `UsbHost`, `HostRtos` extended (WO I0-2, 2026-11-18)

Auditor: `fake-auditor` (Opus 5.5, high), order: `I0-2_AUDIT_ORDERS.md` group D. The auditor stopped at its turn limit and reported partially; probes and mutations ran in `../ffb-test-build/audit/group-d/` only (results in `res/`). The lead re-read R1 in the source and re-derived the M2 / M3 case diffs from the auditor's XML files.

## Result

| Behaviour | Verdict | Fake | Source |
|---|---|---|---|
| Interrupt OUT: packets of up to wMaxPacketSize, end on short packet / ZLP or armed length, NAK while not armed | ok | `host_dcd.cpp:267-307` | RM0090 Rev 22 p.1349; `dcd_dwc2.c:709-733, 784-815` |
| `dcd_edpt_xfer` / XFER_COMPLETE sequencing (IN packets, ZLP only for n = 0) | ok | `host_dcd.cpp:309-342, 461-480` | `dcd_dwc2.c:595-621` |
| Data toggles recorded, not checked | ok (no F1 path) | `host_dcd.cpp:301, 332` | RM0090 p.1349 |
| `UsbHost::hidWrite`: MPS packets, no ZLP | ok; USB 2.0 interrupt-transfer end rule: spec not supplied | `UsbHost.h:241-255, 329-336` | |
| Control SETUP / DATA / STATUS, SETUP clears EP0 stall, NZLSOHSK | ok | `host_dcd.cpp:244-280`; `UsbHost.h:103-161` | RM0090 DCFG bit 2; `dcd_dwc2.c:449, 657-663` |
| SET_ADDRESS after the status stage | ok (documented deviation) | `host_dcd.cpp:334-339, 380-385` | `dcd_dwc2.c:472-478` |
| Bus reset, enumeration, suspend / host resume | ok | `host_dcd.cpp:89-109, 200-227` | `dcd_dwc2.c:286-335, 914-938`; RM0090 p.1235 |
| **U1** `sof()` on a suspended bus ends the suspend without a RESUME event | gap (minor) | `host_dcd.cpp:229-242` | RM0090 p.1235; USB 2.0 §7.1.7.7 spec not supplied |
| dwc2 line numbers cited in the fake are 2-7 lines off | doc | `HostDcd.h:19-23`, `host_dcd.cpp` several | real lines e.g. NZLSOHSK `dcd_dwc2.c:449` (cited :444) |
| `vTaskNotifyGiveFromISR` / `ulTaskNotifyTake` | ok | `host_freertos_ext.inc:244-255, 889-914` | FreeRTOS V10.3.1 `tasks.c:5060-5093` |
| **R1** `vTaskDelay` inside a task advances the shared simulated clock by the whole delay at once; a resumable task that exhausts the budget parks with `ready = false` and `runUntilIdle` never wakes it | **gap, integration blocker** | `host_freertos.cpp:457-469` | `tasks.c:1341-1368`: the task waits in the delayed list and time advances only through the tick while other tasks run |
| **R2** `runUntilIdle` starts never-run tasks and runs in creation order, not by priority | gap (documented "priorities not modelled") | `host_freertos.cpp:311-344`; `HostRtosExt.h:11-13, 47` | `tasks.c:133-177` |
| `vTaskDelay` does not assert on a suspended scheduler | gap (minor, no caller) | `host_freertos.cpp:457` | `tasks.c:1348` |
| Semaphores / mutexes / queues used by the USB OSAL and CDC | ok as far as checked | `host_freertos_ext.inc:1002-1115` | `queue.c` |

## R1 evidence

* Lead read `host_freertos.cpp:457-469`: `vTaskDelay` calls `HostPlatform::advanceMs(ticks)` first, then parks or throws `TaskBlocked` when the budget is used up.
* Auditor's probe (copy only, `cases_rtos/test_audit_probe_groupd.cpp`): a 1 kHz `NotifyFromISR` tick to a priority-30 thread plus a priority-25 thread with `Delay(250)`, 1000 loop iterations, gave 1000 FFB cycles in 250 250 simulated ms (target: ~250 000 cycles). A single `runUntilIdle()` (what `UsbFixture::process` and the `UsbHost` pump call) started a never-run `Delay(250)` task and advanced the clock by 1000 ms.
* Mutation M2 (only under `HOSTTEST_RTOS_EXT` and only from a task: wait until `xTaskGetTickCount() >= wake`; resumable task parks with `ready = due`, restart-model task stores the wake time and throws `TaskBlocked`; the test thread keeps advancing the clock): the probe gave 1000 FFB cycles in 1000 ms. Case diff, re-derived by the lead from `res/base-*.xml` vs `res/m2-*.xml`:
  * `ffb_usb_tests` (606): 1 changes, `a command reply waits while a long reply is still pending and is sent after it` (`claude/cases_usb/test_cdc_device.cpp:655`); its comment states it relies on "The fake advances the time in vTaskDelay".
  * `ffb_rtos_tests` (278): 0 change.
  * `ffb_target_tests`, `ffb_driver_tests`, `ffb_unit_tests` are built without `HOSTTEST_RTOS_EXT` and are unaffected by an ext-only change (not run with M2).
  * Not run: sanitizer and 32-bit variants.
* Mutation M3 (highest-priority ready task first): 0 changes in `ffb_usb_tests`, `ffb_rtos_tests`, `ffb_target_tests`, `ffb_driver_tests` (re-derived by the lead).

Proposed fix (owner's approval needed, plan section 9): M2 as the fake's behaviour under `HOSTTEST_RTOS_EXT`; optionally M3 with it (no coverage case depends on the creation order); the CDC case rewritten so the host reads while the command thread waits for its delay to expire (the intent its comment describes), instead of relying on the clock jump.

## For the integration

* Without R1 fixed, `ChainScheduler` step (d) ("run the TMC thread when its delay expired") cannot be built: every TMC `Delay` would move the whole chain's clock. R1 blocks WO I0-5.
* `IntegrationFixture` must not call `UsbFixture::process()` / `runUntilIdle` blindly once driver threads exist (R2): it drives the threads in its fixed order (or M3 is adopted).
* A budget `TaskBlocked` inside `calibrateEncoder` restarts `TMC4671::Run` from the top in the restart model: the TMC thread has to run resumable.
* U1: suspend tests use `suspend()` -> `resume()` -> `sof()`. Mutation M1b (fault on SOF while suspended without resume) changed 1 of 606 usb cases (`with the SOF callback enabled an SOF ends a suspend`, `test_usbd_core.cpp:465`); not proposed as a fix without USB 2.0.

Confidence for F1: USB interrupt OUT, feature reports, enumeration high; suspend / resume medium; ext RTOS high for notifications and queues, not usable for the TMC thread until R1 is decided. Not verified: the TMC SPI semaphore with the TMCENC thread under the ext RTOS (no executable combines them today), `pxHigherPriorityTaskWoken` with the restart model during another task's activation, inactive-endpoint tokens (spec not supplied).
