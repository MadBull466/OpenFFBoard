# P12 T3 requirements charter: DMA / DMA_Ex (real ST HAL, F407VG and F407VG_DISCO)

Status: 2026-11-11 (lead). Platform, tests, review and gate done (see the plan, section "T3 result"). Amendments after the work: the callback registration API is compiled unconditionally in this HAL version (group DC added); DS2 (FEIE in `Start_IT`), DS7 (FE / DME while polling), DQ7 (no `Abort_IT` timeout; completion by the TCIF of the stop) and DI7 (no DeInit timeout) are characterization cases. Plan: `Tests/Code_Coverage/claude/plans/P12_ST_HAL_PLAN.md` (stage T3).
Platform and rules as in `P12_T1_REQUIREMENTS.md` and `P12_T2_REQUIREMENTS.md` (trapping bus, fixture, `KNOWN_ISSUE`,
`characterization:`, no sleeping, expected values never from the HAL source).

## Oracles

| Topic | Oracle |
|---|---|
| API semantics, parameters, notes, error code names | UM1725 Rev 8 (chapter "DMA" and "DMAEx") |
| Registers, bits, reset values, protection rules, flags, interrupts, FIFO tables | RM0090 Rev 22, chapter 10 (10.3.x functional, 10.4 interrupts, 10.5 registers; Tables 43-52) |
| Request mapping | RM0090 Tables 43 (DMA1) and 44 (DMA2) |
| Addresses | RM0090 Table 1 (DMA1 0x40026000, DMA2 0x40026400) |

Where UM1725 says nothing (return codes of failing calls, timeouts, the order of the register writes, `HAL_DMA_GetError`
bits in uncommon cases), the case is a `characterization:` case with a NEEDS REVIEW entry (numbers from 301).

## Compiled code (F407xx)

`stm32f4xx_hal_dma.c` and `stm32f4xx_hal_dma_ex.c` (`HAL_DMAEx_MultiBufferStart`, `_IT`, `ChangeMemory` exist for the F407). The
callback registration API (`HAL_DMA_RegisterCallback`, `UnRegisterCallback`) is compiled unconditionally in this HAL version (gcov showed it;
`HAL_DMA_CleanCallbacks` is only declared). Tests supply strong callbacks through the handle
(`XferCpltCallback`, `XferHalfCpltCallback`, `XferM1CpltCallback`, `XferM1HalfCpltCallback`, `XferErrorCallback`,
`XferAbortCallback`), not through weak symbols.

## Platform work (lead, before the writers)

`HalDma` model (`fakes/stm32_hal/HalDma.{h,c}`) for DMA1 and DMA2 (RM0090 10.5):
- LISR, HISR (read-only), LIFCR, HIFCR (write 1 clears the matching flag, read 0), stream registers SxCR, SxNDTR, SxPAR,
  SxM0AR, SxM1AR, SxFCR at 0x10 + 0x18 * x ... 0x24 + 0x18 * x; reset values 0 (SxFCR 0x21); reserved bits read 0.
- Write protection (RM0090 10.5.5 - 10.5.10): while EN reads 1, writes to the protected fields of SxCR (all except TCIE, HTIE,
  TEIE, DMEIE, EN and CT for software reads), SxNDTR, SxPAR, SxFCR (except FEIE) and SxM0AR / SxM1AR (the memory registers only
  allowed in double buffer mode for the buffer that is not the current target) are ignored and counted
  (`hostDmaProtectedWrites`); a write of M0AR / M1AR to the current target in double buffer mode sets TEIF (RM0090 10.3.18) and is
  ignored. A test that must not see ignored writes asserts the counter is 0.
- EN handling: EN = 1 starts the stream (a stream with NDT = 0 stays idle but EN reads 1, RM0090 10.5.6); EN = 0 is a request: the
  stream stops after `hostDmaSetDisableDelay(ticks)` HAL ticks (default 0 = at once); `hostDmaStickEnabled(c, s, 1)` makes EN never
  clear (the `HAL_TIMEOUT` paths of Init, DeInit, Abort). Hardware clears EN on the end of a normal-mode transfer, on TEIF, and
  on a FIFO threshold / burst incompatibility at enable time (RM0090 10.5.5, 10.3.18, Table 49: FEIF set, EN cleared).
- Forced bits at EN = 1 (RM0090 10.5.5): MBURST and PBURST = 0 and PINCOS = 0 in direct mode, MSIZE = PSIZE in direct mode, CIRC = 1
  if DBM = 1, CIRC = 0 if PFCTRL = 1, PFCTRL = 0 and DMDIS = 1 in memory-to-memory (RM0090 10.5.5, 10.5.10).
- Test hooks that stand for the transfer engine: `hostDmaRaise(c, s, flagMask)` sets TC / HT / TE / DME / FE flags; `hostDmaComplete(c, s)`
  completes a normal transfer (TCIF, NDT 0, EN cleared; in circular mode NDT reloads and EN stays); `hostDmaStepItems(c, s, n)` lowers
  NDT by n, sets HTIF at half and TCIF at the end (RM0090 10.3.? "half transfer / transfer complete"), and in double buffer mode toggles CT
  at the end. No data is moved: the HAL never reads the data.
- Interrupt requests are not delivered (no NVIC side); the tests call `HAL_DMA_IRQHandler`.
- Not modelled: request timing, priorities, arbitration, FIFO fill level, bus errors other than by `hostDmaRaise`.

## Requirement groups

### DI: init, deinit, state (UM1725 21.2, RM0090 10.5)

| ID | Requirement | Oracle |
|---|---|---|
| DI1 | `HAL_DMA_Init` of every stream of both controllers writes SxCR / SxFCR fields exactly (CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC, DIR, DMDIS, FTH), sets the state to READY and the error code to NONE; SxCR bits not in the init structure are 0 | RM0090 10.5.5, 10.5.10 |
| DI2 | Init clears the stream's flags in LISR / HISR (the bits of that stream only) | RM0090 10.4 note, UM1725 |
| DI3 | Init of a stream that is still enabled disables it and waits for EN = 0; a stream whose EN never clears returns `HAL_TIMEOUT` with the error code TIMEOUT (characterization of the code, UM1725 gives none) | RM0090 10.5.5 |
| DI4 | The FIFO / burst compatibility of RM0090 Table 49 for all MSIZE x FTH x MBURST (48 combinations) when direct mode is disabled: allowed ones return `HAL_OK`, forbidden ones `HAL_ERROR` with the error code PARAM (the codes are characterization, the allowed / forbidden table is RM0090); peripheral burst x size x FIFO threshold 3/4 rule (RM0090 10.3.? note) | RM0090 Table 49 |
| DI5 | Direct mode: MBURST / PBURST are not checked, MSIZE need not equal PSIZE in the structure (hardware forces it) | RM0090 10.5.5 |
| DI6 | Invalid handle (NULL) returns `HAL_ERROR`; a handle with a stream that belongs to neither controller: characterization | UM1725 |
| DI7 | `HAL_DMA_DeInit` disables the stream, waits for EN = 0, writes the reset values to SxCR, SxNDTR, SxPAR, SxM0AR, SxM1AR, SxFCR (0x21), clears all five flags of that stream, resets the handle's callbacks and error code, state RESET, process unlocked; EN never clears: `HAL_TIMEOUT` | RM0090 10.5.5-10.5.10 |
| DI8 | `HAL_DMA_GetState` returns READY / BUSY / RESET as UM1725 defines; `HAL_DMA_GetError` returns the code | UM1725 |
| DI9 | Base address / stream index / flag bit position of every stream (StreamBaseAddress, StreamIndex) so that all 16 streams address their own flags: Start / IRQ / Poll on stream x touch only the bits of stream x in LISR / HISR (RM0090 10.5.1-10.5.4 layout 0, 6, 16, 22 + 0 / 4 bit offsets) | RM0090 10.5.1-10.5.4 |

### DS: start, abort, poll (UM1725, RM0090 10.3)

| ID | Requirement | Oracle |
|---|---|---|
| DS1 | `HAL_DMA_Start`: state BUSY, NDT = DataLength, PAR / M0AR set for the direction (peripheral to memory: Src = PAR, Dst = M0AR; memory to peripheral: Src = M0AR, Dst = PAR; memory to memory: Src = PAR, Dst = M0AR; RM0090 10.3.? Table 45), EN set last, interrupts not enabled | RM0090 Table 45 |
| DS2 | `Start_IT`: as Start and TCIE, HTIE (when a half callback is set), TEIE, DMEIE set, FEIE set when direct mode is off (UM1725 / RM0090 10.4 enable control bits) | RM0090 Table 51 |
| DS3 | Start when the state is not READY returns `HAL_BUSY` and does not touch the registers; the lock makes a second start return `HAL_BUSY` (UM1725 notes) | UM1725 |
| DS4 | DataLength 0 and 65535 / 65536: NDT field limits (RM0090 10.5.6: 0 .. 65535); what the HAL does for 65536 is a `characterization:` case | RM0090 10.5.6 |
| DS5 | `HAL_DMA_Abort`: EN cleared, waits for EN = 0 (timeout 5 ms `HAL_TIMEOUT`, error TIMEOUT, state ERROR, UM1725 note), flags of the stream cleared, state READY, unlocked; abort with no transfer ongoing returns `HAL_ERROR` with the NO_XFER code (UM1725 error list) | UM1725 notes |
| DS6 | `Abort_IT`: starts the abort (EN cleared, interrupts), the abort callback runs from the IRQ handler once EN is 0; with no transfer `HAL_ERROR` NO_XFER | UM1725 |
| DS7 | `HAL_DMA_PollForTransfer` full transfer: returns `HAL_OK` when TCIF is set, clears TCIF, state READY, unlocked; half transfer level returns `HAL_OK` on HTIF and keeps the state BUSY, clears HTIF; timeout `HAL_TIMEOUT` (error TIMEOUT, abort of the stream, state READY); `HAL_MAX_DELAY` waits; a TE / FE / DME flag during polling returns `HAL_ERROR` with the matching error code and clears the flags (HAL_DMA_ERROR_TE / FE / DME) | UM1725, RM0090 10.4 |
| DS8 | Poll in circular mode (`CIRC` set) returns `HAL_ERROR` with the NOT_SUPPORTED code (UM1725 note: not usable in circular and double buffer mode) | UM1725 notes |
| DS9 | Poll on a stream that is not busy returns `HAL_ERROR` (characterization of the code) | UM1725 |

### DQ: IRQ handler (UM1725, RM0090 10.4)

| ID | Requirement | Oracle |
|---|---|---|
| DQ1 | TCIF with TCIE: flag cleared, callback `XferCpltCallback` once; in normal mode the stream is disabled by hardware (EN = 0), the HAL sets state READY and unlocks first | RM0090 10.4, UM1725 |
| DQ2 | HTIF with HTIE: flag cleared, `XferHalfCpltCallback` once; in non-circular mode HTIE is cleared by the HAL (UM1725 does not say: characterization) | RM0090 10.4 |
| DQ3 | TEIF with TEIE: flag cleared, error code TE, `XferErrorCallback`; with the stream disabled by the hardware the HAL sets READY / ERROR as UM1725 says (characterization of the state) | RM0090 10.3.18 |
| DQ4 | FEIF with FEIE, DMEIF with DMEIE: flag cleared, the matching error code, error callback | RM0090 10.3.18 |
| DQ5 | A flag whose interrupt enable is 0 does nothing (no callback, flag stays); a callback pointer that is NULL is not called (no crash) | RM0090 Table 51 |
| DQ6 | Double buffer mode: TCIF toggles between `XferCpltCallback` (CT = 1 just ended) and `XferM1CpltCallback` as the CT bit says; the half callbacks likewise | RM0090 10.3.9 |
| DQ7 | Abort_IT: on the next IRQ with EN = 0 the abort callback runs and the stream's flags are cleared; with EN still 1 only the timeout after 5 ms ends it (`HAL_DMA_ERROR_TIMEOUT`, state ERROR) | UM1725 |
| DQ8 | Each of the 16 streams: the IRQ handler reads and clears only its own bits (the two neighbours' flags stay set) | RM0090 10.5.1-10.5.4 |

### DX: DMA_Ex (UM1725 DMAEx)

| ID | Requirement | Oracle |
|---|---|---|
| DX1 | `HAL_DMAEx_MultiBufferStart` / `_IT`: DBM = 1, CIRC forced, M0AR = Src or Dst (as the direction needs), M1AR = SecondMemAddress, NDT, EN set last; `_IT` as Start_IT; not READY -> `HAL_BUSY`; memory-to-memory is not allowed in double buffer mode (`HAL_ERROR`, error code NOT_SUPPORTED: UM1725 lists the code) | RM0090 10.3.9, 10.5.5 |
| DX2 | `HAL_DMAEx_ChangeMemory`: writes M0AR or M1AR to the address (memory argument), only when the stream is enabled in double buffer mode and the other buffer is the current target (RM0090 10.5.8 / 10.5.9: M0AR writable when CT = 1, M1AR when CT = 0): the model counts ignored writes, the test asserts none and that the right register holds the address | RM0090 10.5.8, 10.5.9 |

## Defect candidates to check (reserve vendor bug V94 onwards)

Look for, and only report with a test that states the intended behaviour as `KNOWN_ISSUE`: a flag of the wrong stream cleared (StreamIndex /
bit offset), a register written while EN = 1 (the protected-write counter), HTIE / HTIF handling that clears the wrong enable, a
timeout loop that never ends, Poll in circular mode accepted, ChangeMemory writing the current target, state / lock left wrong after
an error.

## Writer rules

As in `P12_T1_REQUIREMENTS.md`. Test files (`claude/cases_hal/`): `test_hal_dma_init.cpp` (DI), `test_hal_dma_xfer.cpp` (DS, DX),
`test_hal_dma_irq.cpp` (DQ). Writers never edit `fakes/stm32_hal/`; a missing model feature is reported to the lead. A strong
callback table lives in a helper header owned by the first writer: each file declares its own handle objects and callbacks (names
prefixed per file) to avoid duplicate symbols.
