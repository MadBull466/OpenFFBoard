# P12 T4a requirements charter: SPI (real ST HAL, F407VG and F407VG_DISCO)

Status: 2026-11-12 (lead). Tests, review and gate done (see the plan, section "T4a result"; amendments: SI7 compiled out, SD6 no conversion in this HAL version, interrupt-mode abort success path untestable on the host, SQ6 abort in interrupt mode added). Platform done (`fakes/stm32_hal/HalSpi.{h,c}`, lead probe: `HAL_SPI_Init` and a 3 byte
`HAL_SPI_TransmitReceive` through the real HAL). Tests: writers. T4 is split (owner's "Start T4", plan: "largest stage, may split"):
T4a SPI (this charter), T4b UART, T4c I2C / I2C_Ex; each gets its own charter, model, review, gate and approval.
Plan: `Tests/Code_Coverage/claude/plans/P12_ST_HAL_PLAN.md`. Rules as in `P12_T1_REQUIREMENTS.md` ... `P12_T3_REQUIREMENTS.md`.

## Oracles

| Topic | Oracle |
|---|---|
| API semantics, parameters, notes, state / error code names | UM1725 Rev 8, chapter "SPI" (HAL_SPI_*; blocking / IT / DMA modes; callbacks) |
| Registers, bits, reset values, flags, error sequences, disable procedure, DMA use | RM0090 Rev 22, chapter 28 (28.3.5 transmission and reception procedures, 28.3.7 status flags, 28.3.8 disabling the SPI, 28.3.9 DMA, 28.3.10 error flags, 28.3.11 interrupts, 28.5 registers) |
| Instance addresses | RM0090 Table 1 (SPI1 0x40013000, SPI2 0x40003800, SPI3 0x40003C00) |

Where UM1725 says nothing (return codes of failing calls, timeouts, error code bits in uncommon cases, the order of register writes), the case is a
`characterization:` case with a NEEDS REVIEW entry (numbers from 307).

## Compiled code (target configuration)

`stm32f4xx_hal_spi.c` with `USE_SPI_CRC` 0 (the CRC code is compiled out; `CRCCalculation` / `CRCPolynomial` are accepted and written but no CRC runs),
`USE_HAL_SPI_REGISTER_CALLBACKS` 0 (gcov decides whether the Register / UnRegister functions are compiled in this version: do not assume).
I2S is not enabled (`HAL_I2S_MODULE_ENABLED` is off). Weak callbacks (`HAL_SPI_MspInit`, `MspDeInit`, `TxCpltCallback`, ...) are replaced by strong
test callbacks (one per file, names prefixed per file).

## Platform (lead)

`HalSpi` model (header: what is modelled): CR1 / CR2 / SR / DR / CRCPR / RXCRCR / TXCRCR / I2SCFGR / I2SPR, frames of `frameTicks` ticks with TXE / RXNE /
BSY / OVR, free-running receive for RXONLY and BIDI receive, a peer queue / idle word / loopback, slave frames (`hostSpiSlaveFrame`), MODF, FRE, counters of
a register change with SPE = 1 (`hostSpiConfigWhileEnabled`) and of a disable while BSY (`hostSpiDisabledBusy`), a silent bus for timeouts. DMA streams
(DMA1 / DMA2 model of T3) are used for the DMA modes: the SPI HAL starts the streams through `HAL_DMA_Start_IT`; the tests complete them with
`hostDmaComplete` / `hostDmaStepItems` and call `HAL_DMA_IRQHandler` of the stream, then `HAL_SPI_IRQHandler` where the HAL needs it. SPI1 is on DMA2
(stream 3 / 2 Tx / Rx: RM0090 Table 44), SPI2 on DMA1 (stream 4 / 3), SPI3 on DMA1 (stream 5 / 0, Table 43); the tests set the handle's stream and
channel from those tables.

## Requirement groups

### SI: init, deinit, state (UM1725 66.2, RM0090 28.5)

| ID | Requirement | Oracle |
|---|---|---|
| SI1 | `HAL_SPI_Init` writes CR1 from the init structure exactly: Mode (MSTR), Direction (2 lines: BIDIMODE 0; 2 lines RX only: RXONLY; 1 line: BIDIMODE, BIDIOE not set by Init), DataSize (DFF), CLKPolarity (CPOL), CLKPhase (CPHA), NSS soft (SSM, SSI by mode) / hard input / hard output (SSOE in CR2), BaudRatePrescaler (BR for all 8 values), FirstBit (LSBFIRST); CR2 FRF for TI mode; SPE is 0 after Init; I2SMOD is 0; state READY, error code NONE; the MSP init callback is called once | RM0090 28.5.1, 28.5.2 |
| SI2 | Init of all three instances; for each instance the registers of the others do not change | RM0090 Table 1 |
| SI3 | Init with NULL handle returns `HAL_ERROR`; Init of a handle in state RESET unlocks it and calls MspInit, in state READY (re-init) does not call MspInit (UM1725: "initialize the associated handle"): return codes characterization | UM1725 |
| SI4 | `HAL_SPI_DeInit`: SPE cleared, registers back to the reset values the peripheral reset gives (the HAL calls MspDeInit, which resets the peripheral: model-visible only through the HAL's own register writes: characterization of what DeInit writes), state RESET, error code NONE, unlocked | UM1725 |
| SI5 | `HAL_SPI_GetState` follows READY / BUSY_TX / BUSY_RX / BUSY_TX_RX / ABORT / ERROR as the transfer functions set them; `GetError` returns the code bits | UM1725 names |
| SI6 | Parameter checks of Init: values outside the HAL's `IS_SPI_*` macros are not asserted (`USE_FULL_ASSERT` is off): not tested | owner's decision 68 |
| SI7 | Register / UnRegister callback functions (only if gcov shows them compiled): ID storage, `HAL_ERROR` for an invalid ID or a state that is not READY | UM1725 |

### SP: polling transfers (UM1725, RM0090 28.3.5, 28.3.8)

| ID | Requirement | Oracle |
|---|---|---|
| SP1 | `HAL_SPI_Transmit` master 2 lines 8 bit: the bytes appear on the bus in order (`hostSpiTxAt`), return `HAL_OK`, state READY, unlocked; after the transfer BSY = 0 (RM0090 28.3.8: the HAL waits for BSY before returning) and OVR is cleared (read DR, then SR) | RM0090 28.3.5, 28.3.10 |
| SP2 | The same with 16 bit frames: `Size` counts frames, `pData` is read as 16 bit values; DFF is set in CR1 when the HAL enables the SPI | RM0090 28.5.1 DFF |
| SP3 | `HAL_SPI_Receive` master 2 lines: the received bytes equal the peer's words, `HAL_OK`; RX only and 1 line modes use free-running clocking (RM0090 28.3.5): SPE is cleared at the right time (no OVR left; the model counts a disable while BSY: expected 0 for the HAL's sequence) | RM0090 28.3.5 |
| SP4 | `HAL_SPI_TransmitReceive` master 2 lines: received data equals the peer words, the transmitted bytes are the tx buffer, sizes 1, 2, 3, 16 (8 bit) and 1, 5 (16 bit); loopback: rx == tx | RM0090 28.3.5 |
| SP5 | One line mode (BIDIMODE): `Transmit` sets BIDIOE, `Receive` clears it; the register bits at the time of SPE enable | RM0090 28.5.1 |
| SP6 | Timeout: the bus never completes a frame (`hostSpiPeerSilent`): the call returns `HAL_TIMEOUT` after about `Timeout` ticks (lower and upper bound), error code / state afterwards as UM1725 names them (characterization of the codes), the handle unlocked and the state READY; `Timeout` 0 and `HAL_MAX_DELAY` | UM1725 |
| SP7 | Invalid arguments: NULL data pointer or `Size` 0 returns `HAL_ERROR`; a state that is not READY returns `HAL_BUSY`; a locked handle returns `HAL_BUSY` (characterization of the codes) | UM1725 |
| SP8 | Slave mode: `HAL_SPI_Transmit` / `Receive` / `TransmitReceive` as slave with `hostSpiSlaveFrame` driven from the wait hook of the tick (the master clocks the frames): data moves both ways, OVR on a missed frame sets `HAL_SPI_ERROR_OVR` as UM1725 says | RM0090 28.3.2, 28.3.10 |
| SP9 | MODF: a mode fault during a transfer ends it with the error code MODF; the HAL clears the flag (read SR, write CR1) | RM0090 28.3.10 |

### SQ: interrupt mode and IRQ handler (UM1725, RM0090 28.3.11)

| ID | Requirement | Oracle |
|---|---|---|
| SQ1 | `Transmit_IT` / `Receive_IT` / `TransmitReceive_IT`: state BUSY_*, CR2 TXEIE / RXNEIE / ERRIE set as UM1725 / RM0090 28.3.11 allow, SPE set, the handle's ISR pointers set (not tested through private members: observed through `HAL_SPI_IRQHandler` behaviour), returns `HAL_OK`; arguments / state errors as SP7 | RM0090 28.3.11 |
| SQ2 | `HAL_SPI_IRQHandler` progress: each call with TXE (and TXEIE) writes one frame, with RXNE (and RXNEIE) reads one frame; after the last frame the interrupts are disabled, the state is READY, the matching `TxCplt` / `RxCplt` / `TxRxCplt` callback runs exactly once; 8 and 16 bit | UM1725 callbacks |
| SQ3 | Errors in IT mode: OVR, MODF (and FRE in TI mode) with ERRIE: the error code bit is set (`HAL_SPI_ERROR_OVR` / `MODF` / `FRE`), the flag is cleared by the sequence of RM0090 28.3.10, `HAL_SPI_ErrorCallback` runs once, the state is READY (UM1725: error callback): codes of the state are characterization | RM0090 28.3.10 |
| SQ4 | A flag whose interrupt enable is 0 does nothing; the handler on an idle peripheral does nothing | RM0090 28.3.11 |
| SQ5 | Receive-only master IT: the free-running clock is stopped as RM0090 28.3.5 requires before the last frame is read (`hostSpiDisabledBusy` 0, no OVR) | RM0090 28.3.5 |

### SD: DMA mode, abort, pause (UM1725, RM0090 28.3.9)

| ID | Requirement | Oracle |
|---|---|---|
| SD1 | `Transmit_DMA` / `Receive_DMA` / `TransmitReceive_DMA` set the DMA stream through `HAL_DMA_Start_IT` (peripheral address `&SPI->DR`, direction, `NDTR` = Size for 8 bit; for 16 bit with the HAL's data alignment rule; memory address), CR2 TXDMAEN / RXDMAEN (and ERRIE), SPE; state BUSY_*; returns `HAL_OK`; the stream's channel / enable bits are those the test configured (the SPI HAL does not configure the stream) | RM0090 28.3.9, 10.5 |
| SD2 | DMA completion: `hostDmaComplete` of the stream, `HAL_DMA_IRQHandler`: the SPI DMA callbacks run, the HAL waits for the end of the transaction (BSY / TXE per RM0090 28.3.8), clears the DMA enables, state READY, `TxCplt` / `RxCplt` / `TxRxCplt` once; half transfer callbacks on HTIF | UM1725 |
| SD3 | DMA errors: a stream error (TEIF) sets `HAL_SPI_ERROR_DMA`, `ErrorCallback` once, state READY; OVR as SQ3 | UM1725 error names |
| SD4 | `HAL_SPI_Abort` / `Abort_IT`: DMA streams aborted, SPE cleared per RM0090 28.3.8 with the BSY wait, flags cleared, state READY, `AbortCpltCallback` for `_IT`; abort with no transfer returns `HAL_OK` | UM1725 |
| SD5 | `HAL_SPI_DMAPause` / `DMAResume` / `DMAStop`: CR2 TXDMAEN / RXDMAEN cleared / set, `DMAStop` aborts the streams and the state is READY (the DMA_* return codes are characterization) | UM1725 |
| SD6 | `HAL_SPI_Transmit_DMA` with 16 bit data and an odd stream alignment: the size conversion (UM1725 gives no formula: characterization) | RM0090 28.3.9 |

## Defect candidates to check (reserve vendor bug V97 onwards)

Look for, and only report with a test that states the intended behaviour as `KNOWN_ISSUE`: a disable of SPE while BSY (the model counter), a register
change with SPE = 1 (counter), a lock that stays set after a return, OVR left set after a transfer, a timeout loop that never ends, a callback called
twice, an error callback without the error bit set.

## Writer rules

As in `P12_T1_REQUIREMENTS.md`. Test files (`claude/cases_hal/`): `test_hal_spi_init.cpp` (SI), `test_hal_spi_poll.cpp` (SP), `test_hal_spi_irq.cpp` (SQ),
`test_hal_spi_dma.cpp` (SD). Writers never edit `fakes/stm32_hal/`; a missing model feature is reported to the lead. Every file-local symbol (handles, helpers) carries the file's prefix
(`spiInit`, `spiPoll`, `spiIrq`, `spiDma`) or sits in an anonymous namespace. The strong HAL_SPI_* callbacks (MspInit, MspDeInit, the six completion
callbacks, ErrorCallback, AbortCpltCallback) are defined ONCE by the lead in `claude/cases_hal/test_hal_spi_support.cpp` with the counters
`spiCounts` and the fixture `SpiFixture` of `SpiTestSupport.h`: every SPI test file includes that header, uses `SpiFixture` (it resets the counters)
and never defines these symbols itself.
