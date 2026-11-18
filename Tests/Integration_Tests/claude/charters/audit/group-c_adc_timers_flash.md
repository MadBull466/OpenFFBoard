# Audit group C: fake ADC, timers, `micros()` and the flash map (WO I0-2, 2026-11-18)

Auditor: `fake-auditor` (Opus 5.5, high), order: `I0-2_AUDIT_ORDERS.md` group C. The auditor stopped at its turn limit and reported partially, from reading only (no test run). The lead re-checked D1 by reading the fake, the firmware and the affected case.

## Result

| Behaviour | Verdict | Fake | Source |
|---|---|---|---|
| `HAL_ADC_Start_DMA` sequence, conversion to buffer, `ConvCpltCallback` in ISR | ok | `host_target_system.cpp:312-333`, `:493-513` | `stm32f4xx_hal_adc.c:1364ff` |
| LL voltage macros, VREFINT_CAL; `hasPower()` 10 V / 78 V | ok | `platform/target_f407/main.h:302-341`, `TmcFixture.h:15-19, 66-97` | `stm32f4xx_ll_adc.h:258-265, 1602-1682` |
| `vSenseMult` reset between tests | ok | `host_target_system.cpp:89, 273` | `voltagesense.cpp:19` |
| **G2** ADC converts only when the test calls it (target: both edges of TIM4 CC4, every ~101 us) | gap | | `main.c` MX_TIM4_Init, ADC2 `RISINGFALLING` |
| TIM13 start / stop / IT (HAL contract) | ok | `host_target.cpp:219-251` | `stm32f4xx_hal_tim.c:459-490` |
| **G1** timer rate from PSC / ARR, ARR / PSC change timing (TIM13, TIM6): `timerUpdate` is manual, no counter, no ARPE | gap | `host_target.cpp:99, 112-121`, `host_driver_timers.cpp:78-88` | RM0090 §19.3.1 (ARR preload / ARPE), §20.3.1 (PSC buffered, applied at the next update); MX_TIM13_Init `AutoReloadPreload = DISABLE` |
| TIM7 / `micros()` formula, 2^32 us wrap, ms-boundary monotonicity | ok (host-vs-target note below) | `host_target_system.cpp:217-229`, `host_platform.cpp:164` | real `cppmain.cpp:127` |
| Fast-forward beyond 2^32 us | ok: `setTimeUs` jumps without per-ms ticks | `host_platform.cpp:127-132` | |
| TIM3 quadrature | ok per its header (not re-audited) | `host_target.cpp:139-180` | |
| **D1** `Flash_Dump` ignores `includeAll` and the exportable list: returns every map entry | **defect** | `host_platform.cpp:356-360` | real `flash_helpers.cpp:268-287` walks `exportableFlashAddresses` (or `VirtAddVarTab` with `includeAll`) and returns only entries that read |
| `Flash_Write` always returns true (real: false when unchanged) | doc (no firmware caller uses the result) | `host_platform.cpp:333-336` | `flash_helpers.cpp:62-89` |
| Persistence of addresses outside `VirtAddVarTab` | ok for F1 (all 183 `ADR_` defines are in the table; computed addresses not verified) | | `eeprom.c:689-699` |
| **G3** no reboot that keeps flash (`HostPlatform::reset` clears the map) | gap | `host_platform.cpp:71` | needed for F1-P1 / P2 |

## D1 evidence (lead, 2026-11-18)

* Fake `host_platform.cpp:356-360` iterates `HostPlatform::flash()`; firmware `flash_helpers.cpp:268-287` iterates a fixed address list. Read by the lead.
* Affected case, by reading: `flashdump returns every stored flash variable as value and address` (`claude/cases_target/test_system_commands.cpp:804`) stores at 0x10 and 0x20, which are in neither list, and expects both back. With a faithful fake it gets no replies. `Flash_Dump` / flashdump appear in no other map-executable test (`cases_flash/` uses the real `flash_helpers.cpp`). **Not yet verified by a mutation run** (the correction needs `eeprom_addresses.c`, today compiled only into `ffb_flash_tests`).
* On the target the dump leaves out 15 exportable-excluded addresses (`ADR_HW_VERSION`, `ADR_SW_VERSION`, `ADR_CANANALOG_CONF1`, the TMC ADC / encoder / PhiE offsets of axes 1-3); the fake shows them.

Proposed fix (owner's approval needed): the fake walks the same list as the firmware (`eeprom_addresses.c` added to the map executables); the coverage case uses exportable addresses and gains a check that a calibration address is not dumped. Impact on F1: low (only if an integration case uses `sys.flashdump`).

## For the integration (no shared change needed)

* G1: `ChainScheduler` derives each timer's period from `Instance->PSC / ARR / CR1` (period = (ARR+1)(PSC+1)/f_CK, timer clock 84 MHz on APB1; TIM13 ARR applies at once (ARPE = 0), PSC at the next update) and calls `timerUpdate` when it elapses. RM0090 does not say what happens when a new ARR is below CNT: not modelled, no test relies on it.
* G2: the scheduler converts the ADC at least once per simulated ms and after every supply change.
* G3: an `IntegrationFixture::reboot()` that deletes the main class, resets the firmware globals without `HostPlatform::reset`, keeps the flash map and re-runs `runCppmain`. Static objects are not re-initialised on the host (host-vs-target note).

## Firmware observations (not registered; to be confirmed by a failing integration test before an IB number)

* `FFBoard/UserExtensions/Src/FFBHIDMain.cpp:256` sets `ARR = 1000000 * basediv / basefreq` ("Assumes 1µs timer steps"); an up-counter's period is ARR+1 counts, so e.g. 1001 us instead of 1000 us for ARR 1000 (same pattern as coverage bug 201). Checked by the lead in the source; `basefreq` value not re-checked.
* `TMC4671.cpp:3156` sets PSC = `((SystemCoreClock / 2)/1000000) + 1` = 85 (macro `TIM_TMC_BCLK`, `target_constants.h:72`; checked by the lead), dividing by 86, dividing by 86; with ARR 250 the external-encoder period is about 257 us (~3.9 kHz against a "4khz" comment). Assumes the 84 MHz APB1 timer clock (`target_constants.h:72`, `main.c`).

## Host-vs-target limits

* On the target `micros()` can step back by about 1000 us when the read races the TIM7 wrap and its ISR; the host cannot produce this.
* The map flash takes no time (a sector erase blocks the CPU on the target).
* Virtual address 0xFFFF is storable in the fake but not on the target (no firmware address uses it).

Confidence for F1: ADC / voltagesense high; `micros()` high; TIM13 / TIM6 HAL contract high, rate not modelled (scheduler supplies it); flash map medium-high. Not done: D1 mutation run, long `advanceUs` cost under `HOSTTEST_RTOS_EXT`, deeper TIM3 audit.
