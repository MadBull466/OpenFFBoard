# Audit group B: fake HAL SPI and GPIO / EXTI (WO I0-2, 2026-11-18)

Auditor: `fake-auditor` (Opus 5.5, high), order: `I0-2_AUDIT_ORDERS.md` group B. The auditor stopped at its turn limit and reported partially. The lead re-checked D1 against the HAL source and ran its mutation (below). Files audited: `Firmware/Tests/Code_Coverage/platform/src/host_target_bus.cpp` (SPI), `host_target.cpp` / `host_platform.cpp` (GPIO, EXTI), `support/HostTarget.h`. `fakes/stm32_hal/` is the P12 real-HAL harness, not on the F1 path (IQ4), not audited.

## Result

| Behaviour | Verdict | Fake | Source |
|---|---|---|---|
| Blocking `HAL_SPI_TransmitReceive` / `HAL_SPI_Transmit`: check order, lock, State, ErrorCode | ok | `host_target_bus.cpp:226-271`, `:135-164` | `stm32f4xx_hal_spi.c:781-923`, `:1160-1374` (checked-in file identical to ST 1.8.3) |
| `spiFailNextBlocking(HAL_ERROR / HAL_TIMEOUT)` | ok | `:138-161` | `hal_spi.c:1309-1354`, `:3527-3532` |
| `spiFailNextBlocking(HAL_OK / HAL_BUSY)` silently does nothing | doc | `:138-161` | |
| `HAL_SPI_Init` with a pending transfer | ok | `:205-220` | `hal_spi.c:361-428` |
| IT path: `HAL_SPI_Transmit_IT` checks, BUSY on a second request; completion: State READY before the callback, ISR context, re-arm from the callback | ok | `:293-310`, `:170-194`, `:909-940` | `hal_spi.c:1392-1456`, `:3809-3861` |
| **D1** `spiFail` with OVR (its default) on a pending Transmit (IT / DMA) calls `HAL_SPI_ErrorCallback`; MODF / FRE accepted on any transfer | **defect** (test-control function) | `host_target_bus.cpp:942-958`, `HostTarget.h:461` | `hal_spi.c:2411-2422`: in `HAL_SPI_STATE_BUSY_TX` the OVR flag is cleared and the handler returns without a callback; MODF impossible with MSTR+SSI (software NSS), FRE only in TI mode (RM0090 Rev 22 §28.3.10) |
| **G1** IT transfers complete only when the test calls `spiComplete` | gap | `HostTarget.h:407-409` | on the target TXE fires immediately after the IT enable (`hal_spi.c:1454`); 5 bytes take microseconds |
| CS framing and routing by the CS level at the start | ok | `:116-117`, `:159`, `:922`; `FakeTmc4671.cpp:139-156` | TMC4671 DS §4.2.1; `SPI.cpp:158-183` |
| **G2** a frame whose CS is released before completion is still delivered (`csAtEnd` recorded, never checked) | gap | `:922` | datasheet rule for aborted datagrams not verified |
| CS idle levels at reset (SS2 / SS3 low as `MX_GPIO_Init`) | doc (fixture choice L14) | `host_target.cpp:93`, `TmcFixture.h:58-62` | `Targets/F407VG/Core/Src/main.c` |
| GPIO Write / Read / Toggle | ok | `host_platform.cpp:182-194` | `stm32f4xx_hal_gpio.c:375-445` |
| L27: IDR does not follow ODR for outputs | doc, unreachable on F1 (no firmware `ReadPin` on an output) | `host_platform.cpp:187-189` | RM0090 §8.3.10 |
| **G3** `triggerExti` raises the callback without an edge / level check | gap (documented in the header) | `host_target.cpp:192-201` | RM0090 §12.2.4; edges in `MX_GPIO_Init`: E_STOP both, FLAG rising |
| **G4** the TMC STATUS output is not wired to FLAG (PE8) / EXTI | gap (shared with group A) | `FakeTmc4671` | TMC4671 DS |

## D1 evidence (lead, 2026-11-18)

* HAL text quoted from `Firmware/Targets/F407VG/.../stm32f4xx_hal_spi.c:2405-2422` (read by the lead).
* Mutation (copy in `../ffb-test-build/audit/group-b/`, auditor's script `mut1.py`): when the pending callback is Tx and the error contains OVR, record an unmodelled HAL call and leave the transfer pending. `ffb_target_tests` (1881 cases) before / after, per-case comparison: exactly two cases change from pass to fail:
  * `a device without overrides takes the port and drives its CS pin around an interrupt driven transfer` (`claude/cases_target/test_spi_port.cpp:1041`)
  * `a completion or error is reported to the current device only once` (`test_spi_port.cpp:700-720`)
* `spiFail` is used only in `test_spi_port.cpp` and `test_spi_buttons.cpp` (both in `ffb_target_tests`), so no other executable is affected.

Proposed fix (owner's approval needed, plan section 9): the mutation above as the fake's behaviour, plus MODF / FRE recorded as unmodelled on the F407's SPI configuration; the two coverage cases changed to inject an error the HAL can deliver on a transmit (`HAL_SPI_ERROR_FLAG`, the end-of-transfer flag timeout in `SPI_CloseTx_ISR`, `hal_spi.c:3818-3835`) or to use a receive transfer for the OVR path.

## For the integration (no shared change needed)

* G1: `ChainScheduler` completes the pending `hspi1` IT transfer after every TMCENC activation (or the fixture offers an auto-complete). Required for F1-T9.
* G3 / G4: an integration helper sets the pin level and raises EXTI only on the configured edge; the FLAG coupling is decided with group A.
* G2: only if an F1 case raises FLAG while an IT frame is pending.

Confidence for F1: blocking SPI, CS, GPIO high; IT path medium-high with G1 handled; E-stop / FLAG EXTI medium. Not done: `HAL_SPI_Receive` / DMA paths (not on the TMC chain), UM1725 pages.
