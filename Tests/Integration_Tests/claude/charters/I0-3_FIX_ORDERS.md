# WO I0-3: fixes of the shared fakes (as sent, 2026-11-18)

Approved by the owner on 2026-11-18 (`../plans/PROGRESS.md` decisions log, fake audit register A-1 to A-3). One `integration-test-writer` run per fix, one after the other (a, b, c). Each run receives the common part plus its section and nothing else.

## Common part

You fix ONE defect in a shared test fake of the OpenFFBoard host test suite. Repository root `/home/ben/Documents/OpenFFBoard/testing/OpenFFBoard`, branch `add-test-suite`. Read `CLAUDE.md`, `Firmware/Tests/README.md` and the audit report named in your section first.

This work belongs to the integration track, where the owner overrode one hard rule: a wrong shared fake in `Firmware/Tests/Code_Coverage/` is fixed in place, and existing coverage test cases that passed only because of the defect are edited too. Every other rule of `CLAUDE.md` stays: `Firmware/` outside `Firmware/Tests/` is read-only, no `#ifdef` for tests in firmware sources, doctest only, portable C++ / CMake (MinGW, 32-bit build), no installs or downloads, no commits. The hooks allow you to edit only the paths your section lists (they are in `Tests/Integration_Tests/claude/plans/APPROVED_SHARED_EDITS.txt`); a blocked edit means the path is not part of this fix: stop and report it, do not work around it.

Rules for the change:
- The fake must behave like the cited source (HAL / RM0090 / firmware). Update its fidelity comment (what it models, source, what it does not model, confidence) where the behaviour changes.
- Add at least one test case that fails with the old fake and passes with the fixed one (a self-test of the fake, in a file your section lists), named as a statement that is true when it passes.
- An edited coverage case keeps its purpose: change only what relied on the defect, and say in a comment why (`// I0-3x: ...` with the HAL / source citation). Never weaken an assertion to make a case pass; never delete a case. If a case cannot keep its purpose with a faithful fake, stop and report it under "Lead decision needed".
- Expected values come from the cited source, not from what the fake or firmware currently produces.

Verification (the integration gate `it_gate.py` does not exist yet; use this coverage gate):
1. The lead has built baselines of the unmodified tree: `/home/ben/Documents/OpenFFBoard/testing/ffb-test-build/i03/base-<variant>.*` for variants `default`, `m32`, `san` (`*.report.log` = every failed and known-issue case per executable from `run_all_tests.py`; `*.ctest.log`; `*.baseline.log`).
2. After your change run, for each variant your section names: `/home/ben/Documents/OpenFFBoard/testing/ffb-test-build/i03/gate.sh <variant> after-<your WO id>` (it rebuilds `../ffb-test-build/i03-<variant>` incrementally, runs ctest, `check_baseline.py` and `run_all_tests.py`). It takes several minutes per variant; run it in the background if your tools allow and wait for it.
3. Compare `after-<WO>-<variant>.report.log` with `base-<variant>.report.log` (summary counts, the failed list, the known-issue list) and the ctest logs. Required: ctest all passed, `check_baseline` exit 0, no new failed case except where your section expects one before your coverage-case edit, and every changed result explained.
4. A build warning in a file you touched is a failure.

Answer format (the lead checks every line):
1. Files changed, each with a one-line summary.
2. The fake change with its source citation.
3. Coverage cases edited: name, file:line, what relied on the defect, what the case now does.
4. New self-test case(s).
5. Per variant: ctest result, `check_baseline` exit code, run_all_tests summary before and after, and every case whose result changed with the reason.
6. Anything open, under "Lead decision needed".
7. The last line must be exactly: `GATE FAIL it_gate.py not available until WO I0-4; coverage gate: <variants> <ok|not ok>` (the stop hook requires a gate line; this one is honest about the missing gate).

If the same problem fails twice, stop and report it instead of retrying.

## I0-3a: A-1, `spiFail` delivers errors the HAL never produces

Audit report: `Tests/Integration_Tests/claude/charters/audit/group-b_spi_gpio.md` (section D1).

Allowed files: `Firmware/Tests/Code_Coverage/platform/src/host_target_bus.cpp`, `Firmware/Tests/Code_Coverage/support/HostTarget.h`, `Firmware/Tests/Code_Coverage/claude/cases_target/test_spi_port.cpp`.

Defect: `spiFail(hspi, errorCode = HAL_SPI_ERROR_OVR)` (`host_target_bus.cpp:942-958`) always sets ErrorCode, State READY and calls `HAL_SPI_ErrorCallback`. The real `HAL_SPI_IRQHandler` (`Firmware/Targets/F407VG/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c:2405-2422`) clears OVR and returns without a callback while `State == HAL_SPI_STATE_BUSY_TX` (a transmit-only transfer, IT or DMA). MODF cannot occur with the firmware's master configuration (software NSS with SSI set: `SPI_MODE_MASTER` = MSTR|SSI) and FRE only in TI mode (RM0090 Rev 22 §28.3.10); a transmit can end with `HAL_SPI_ERROR_FLAG` (`SPI_CloseTx_ISR`, `hal_spi.c:3818-3835`).

Fix: when the pending transfer is a transmit (Tx callback) and the error contains OVR, record an unmodelled HAL call (`HostTarget::state().unmodelledHalCalls`, as the fake does elsewhere) and leave the transfer pending, returning false. Also record MODF / FRE as unmodelled for a port configured as the firmware configures it (check how the fake knows the configuration; if it cannot tell, record them always and say so). Keep `spiFail` working for receive / transmit-receive transfers with OVR and for `HAL_SPI_ERROR_FLAG` on a transmit. Document the new behaviour in the `spiFail` comment in `HostTarget.h`.

Expected coverage changes (lead's mutation run, `ffb_target_tests`): `a device without overrides takes the port and drives its CS pin around an interrupt driven transfer` (`test_spi_port.cpp:1041`) and `a completion or error is reported to the current device only once` (`test_spi_port.cpp:700-720`) fail with the fixed fake. Edit them to inject an error the HAL can deliver for their transfer (`HAL_SPI_ERROR_FLAG` on a transmit, or OVR on a receive / transmit-receive) so they keep testing the port's error routing. Add a self-test in `test_spi_port.cpp`: OVR on a pending transmit reaches no callback, leaves the transfer pending and is recorded as unmodelled.

Variants: `default`, `m32`.

## I0-3b: A-2, `Flash_Dump` ignores the firmware's address list

Audit report: `Tests/Integration_Tests/claude/charters/audit/group-c_adc_timers_flash.md` (section D1).

Allowed files: `Firmware/Tests/Code_Coverage/platform/src/host_platform.cpp`, `Firmware/Tests/Code_Coverage/support/HostPlatform.h`, `Firmware/Tests/CMakeLists.txt`, `Firmware/Tests/Code_Coverage/claude/cases_target/test_system_commands.cpp`.

Defect: the fake `Flash_Dump(result, includeAll)` (`host_platform.cpp:356-360`) returns every entry of the map flash. The real function (`Firmware/FFBoard/Src/flash_helpers.cpp:268-287`) walks `exportableFlashAddresses` (or `VirtAddVarTab` when `includeAll` is true) from `Firmware/FFBoard/UserExtensions/Src/eeprom_addresses.c` and returns only the entries `Flash_Read` finds.

Fix: the fake walks the same list in the same order with the same `includeAll` rule and reads through its own `Flash_Read`. `eeprom_addresses.c` is compiled today only into `ffb_flash_tests` (`Firmware/Tests/CMakeLists.txt` around line 479): add it to the sources of every executable that links the map-backed `host_platform.cpp` (not to the ones built with `HOSTTEST_REAL_FLASH_HELPERS`), as a firmware source, keeping the CMake portable. Check that no duplicate symbol appears and that `ffb_flash_tests` is unchanged.

Expected coverage change (by reading, not yet run): `flashdump returns every stored flash variable as value and address` (`test_system_commands.cpp:804`) stores at 0x10 and 0x20, which are in neither list. Edit it to store at exportable addresses (use `ADR_` constants from `eeprom_addresses.h`, not raw numbers) and expect them back in list order; add a case that a stored calibration address excluded from the export list (e.g. one of the `ADR_TMC1_*` offsets that `exportableFlashAddresses` leaves out: check the list) is not dumped by `flashdump`; check whether any other case uses `Flash_Dump` / `flashdump` / `replyFlashDump` and how its result changes.

Variants: `default`, `m32`.

## I0-3c: A-3, `vTaskDelay` in a task jumps the shared clock; `runUntilIdle` ignores priorities

Audit report: `Tests/Integration_Tests/claude/charters/audit/group-d_usb_rtos.md` (sections R1, R2).

Allowed files: `Firmware/Tests/Code_Coverage/platform/src/host_freertos.cpp`, `Firmware/Tests/Code_Coverage/platform/src/host_freertos_ext.inc`, `Firmware/Tests/Code_Coverage/support/HostRtos.h`, `Firmware/Tests/Code_Coverage/support/HostRtosExt.h`, `Firmware/Tests/Code_Coverage/claude/cases_usb/test_cdc_device.cpp`, `Firmware/Tests/Code_Coverage/claude/cases_rtos/test_rtos_setup.cpp`.

Defect R1: `vTaskDelay(ticks)` (`host_freertos.cpp:457-469`) calls `HostPlatform::advanceMs(ticks)` first, also when called from a task, so one task's delay moves the simulated clock for everything while no other task or timer runs; a resumable task that exhausts the delay budget parks with a ready condition that is never true. FreeRTOS V10.3.1 (`Firmware/Targets/F407VG/Middlewares/Third_Party/FreeRTOS/Source/tasks.c:1341-1368`, `prvAddCurrentTaskToDelayedList`) puts the task in the delayed list; time advances only through the tick while other tasks run.

Defect R2: `runUntilIdle` (`host_freertos.cpp:311-344`) runs ready tasks in creation order; FreeRTOS runs the highest-priority ready task first (`tasks.c:133-177`, `taskSELECT_HIGHEST_PRIORITY_TASK`).

Fix, **only under `HOSTTEST_RTOS_EXT`** (the executables built without it keep today's behaviour; say in the fidelity comment which executables have which model):
- R1: `vTaskDelay` called from a task does not advance the clock; the task waits until `xTaskGetTickCount() >= wake time`: a resumable task parks with that condition as its ready predicate; a restart-model task records the wake time (`extWake`) and throws `TaskBlocked`. Called from the test thread (no current task) it keeps advancing the clock. Check `vTaskDelayUntil` and `osDelay` for the same pattern and treat them alike. Keep the delay-budget mechanism consistent (document what the budget means now).
- R2: in `runUntilIdle`, among the ready tasks run the one with the highest priority first (ties: creation order), re-evaluating readiness after each activation. A never-run task counts as ready as today.
- A test that needs a delay to expire now advances simulated time itself; check every `HOSTTEST_RTOS_EXT` executable (`ffb_usb_tests`, `ffb_rtos_tests`; find any other in `Firmware/Tests/CMakeLists.txt`).

Expected coverage changes (auditor's mutation): `ffb_usb_tests` 1 case, `a command reply waits while a long reply is still pending and is sent after it` (`test_cdc_device.cpp:655`), whose comment says it relies on "The fake advances the time in vTaskDelay"; `ffb_rtos_tests` none; the priority order changed no case in the auditor's run. Edit the CDC case so the host reads while the command thread waits for its delay to expire (the behaviour its comment describes for the target), advancing simulated time explicitly. Add self-tests in `test_rtos_setup.cpp`: (1) a task's `vTaskDelay` does not move the simulated clock and the task continues only after the test advanced time past its wake time; (2) with two ready tasks of different priority, `runUntilIdle` runs the higher-priority one first.

Variants: `default`, `m32`, `san`.
