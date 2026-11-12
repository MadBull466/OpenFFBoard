# P12 T4c requirements charter: I2C (real ST HAL, F407VG and F407VG_DISCO)

Status: 2026-11-12 (lead). Charter and platform (`fakes/stm32_hal/HalI2c.{h,c}`, lead probe `test_hal_i2c_probe.cpp`) done; tests, review, gate pending.
Plan: `Tests/Code_Coverage/claude/plans/P12_ST_HAL_PLAN.md`. Rules as in the T1-T4b charters. `stm32f4xx_hal_i2c_ex.c` is compiled out on this target
(its body is inside `#if defined(I2C_FLTR_ANOFF) && defined(I2C_FLTR_DNF)`, and the F407 CMSIS header defines neither; RM0090 27.6.10: I2C_FLTR is
"available on STM32F42xxx and STM32F43xxx only"): nothing to test, recorded as a scope note (one lead case checks the macros are undefined).

## Oracles

| Topic | Oracle |
|---|---|
| API semantics, parameters, notes, state / error code names, transfer options | UM1725 Rev 8, chapter 36 "HAL I2C Generic Driver" (text: `../ffb-test-build/p12t4c-specs/um1725.txt`, from line 32102) |
| Registers, reset values, flag set / clear sequences, controller and target sequencing (EV1-EV9), error conditions, interrupt lines, DMA use, CCR / TRISE / FREQ meaning | RM0090 Rev 22 chapter 27 (`rm0090.txt`, "27 Inter-integrated circuit (I2C) interface", from line 45711): 27.3.2 target, 27.3.3 controller (Figures 241-244), 27.3.4 errors, 27.3.8 DMA, 27.4 Table 124, 27.6 registers, Table 125 |
| Instance addresses | I2C1 0x40005400, I2C2 0x40005800, I2C3 0x40005C00 (RM0090 Table 1); model index 0, 1, 2 |
| Errata | ES0182 section 2.10 (bus error handling, SWRST workaround): `rm0090.txt` has no errata; `es0182.txt` is in the same folder |

DMA requests (RM0090 Table 43, all on DMA1 = controller 0 of the model): I2C1 Tx channel 1 stream 6 or 7, Rx channel 1 stream 0 or 5; I2C2 Tx channel 7 stream 7, Rx channel 7 stream 2 or 3; I2C3 Tx channel 3 stream 4, Rx channel 3 stream 2 (checked in `rm0090.txt` lines 16189-16232).
Where UM1725 / RM0090 say nothing (return codes in uncommon cases, error codes in
uncommon cases, register write order, which of two flags the HAL checks first) the case is `characterization:` with a NEEDS REVIEW entry (numbers from 320; writers use `NR_I2C_<n>` placeholders).

## Compiled code (target configuration)

`stm32f4xx_hal_i2c.c`, `USE_HAL_I2C_REGISTER_CALLBACKS` 0 (RegisterCallback functions compiled out: check gcov, do not assume). No I2S, no FMPI2C module. Weak callbacks
(`HAL_I2C_MspInit`, `MspDeInit`, `MasterTxCplt`, `MasterRxCplt`, `SlaveTxCplt`, `SlaveRxCplt`, `ListenCplt`, `MemTxCplt`, `MemRxCplt`, `Error`, `AbortCplt`, `AddrCallback`) are strong
test callbacks defined once by the lead in `test_hal_i2c_support.cpp` (counters in `i2cCounts`, order log). `USE_FULL_ASSERT` off: parameter asserts are not tested.

## Platform (lead): `HalI2c` model

Read the header comment of `HalI2c.h`: registers CR1 / CR2 / OAR1 / OAR2 / DR / SR1 / SR2 / CCR / TRISE with write masks and flag clear sequences; controller engine (START, repeated
START, STOP, 7 / 10-bit address phase, transmitter pipeline TXE / BTF, receiver pipeline RXNE / BTF with stretching, ACK / POS, NACK, AF), peer devices (`hostI2cDeviceAdd`, `...NackAddress`,
`...NackDataAt`, `...Queue`), a bus log (`hostI2cLogAt`: the oracle for what went on the bus: address bytes with R/W bit, data bytes, acks, START / RESTART / STOP), target engine driven by an external
controller (`hostI2cExtStart / Restart / Write / ReadByte / Stop`), error injection (`hostI2cRaiseSr1`, `hostI2cArbitrationLost`, `hostI2cStuck`, `hostI2cBusBusy`), `hostI2cSetByteTicks`,
`hostI2cSetPollReads`. Interrupts are not delivered: `i2cRunIrqs(&hi2c, index)` calls `HAL_I2C_EV_IRQHandler` / `HAL_I2C_ER_IRQHandler` while the request line of RM0090 Table 124 is pending
(it stops after 64 calls: a handler that never clears its request is a defect). DMA requests are not generated: the tests start the streams with the real `HAL_DMA_Init`, link the handles, move bytes
with plain `I2Cx->DR` accesses (the CPU role of the DMA) and finish the stream with `hostDmaComplete` / `hostDmaStepItems` / `hostDmaRaise` + `HAL_DMA_IRQHandler` (T3 hooks).
Blocking slave calls (`HAL_I2C_Slave_Transmit` / `Receive`, `HAL_I2C_IsDeviceReady` with a slow peer) wait inside the HAL: script the external controller with `i2cScriptStart` (actions at HAL ticks;
one tick per `HAL_GetTick` call).
`HAL_I2C_Init` reads PCLK1: set `SystemCoreClock` and `RCC->CFGR` PPRE1 (tests use 168 MHz with PPRE1 /4 = 42 MHz, the firmware's clock tree) and `uwTickPrio = 15`.
HAL addresses: master functions take the 8-bit-aligned `DevAddress` (7-bit address << 1); `OwnAddress1` is likewise the already shifted value (0x30 matches bus address byte 0x30).
Model decisions the manual leaves open (state them in the report if a case depends on them): the first DR write of a transmitter goes to the shift register and TXE stays 1 (27.6.6 TxE note); TXE is 1 right
after ADDR of a transmitter; the byte of the address phase does not set TXE / RXNE; SB / ADD10 clear only with "read SR1 then write DR" (the model still sends the address and counts `hostI2cSequenceBreaks`);
POS = 1: the ack of byte k is the ACK bit value sampled when byte k-1 was received, the first byte is acked; a stop condition keeps RXNE / DR readable; the peer sends 0xFF after its queue.

## Requirement groups (one writer, one file each)

### I2I (`test_hal_i2c_init.cpp`): Init, DeInit, GetState / GetMode / GetError, helper macros
- II1 `HAL_I2C_Init`: NULL handle, state RESET runs MspInit once and unlocks, other states do not run it; PCLK1 below the minimum for the speed returns HAL_ERROR (2 MHz Sm / 4 MHz Fm; UM1725, RM0090 27.3.2) with state BUSY left? (characterization) ; SWRST pulse (count), PE cleared first.
- II2 FREQ field = PCLK1 in MHz; TRISE = Sm: FREQ + 1 (1000 ns / T_PCLK1 + 1), Fm: 300 ns / T_PCLK1 + 1 (RM0090 27.6.9; hand-derive for 8 / 16 / 24 / 42 MHz); CCR for Sm (PCLK / (2 * speed)), Fm duty 2 (PCLK / (3 * speed)), Fm duty 16/9 (PCLK / (25 * speed)), minimum CCR 4 (Sm / Fm duty 2) and 1 (duty 16/9) (27.6.8), F/S and DUTY bits; hand-derive with the worked example of 27.6.8 (FREQ 8, 100 kHz: CCR 0x28).
- II3 OAR1 (7 / 10-bit, bit 14 kept 1, own address bits), OAR2 (ENDUAL, ADD2), CR1 ENGC / NOSTRETCH; registers not touched by the HAL keep their values (CR2 interrupt enables).
- II4 `HAL_I2C_Init` end state: PE = 1, ErrorCode NONE, State READY, PreviousState NONE, Mode NONE; `HAL_I2C_DeInit`: PE = 0, MspDeInit once, ErrorCode NONE, State RESET, Mode NONE, lock released; DeInit of a never initialised handle; DeInit while busy (state).
- II5 GetState / GetMode / GetError return the fields; every `HAL_I2C_STATE_*`, `HAL_I2C_MODE_*`, `HAL_I2C_ERROR_*` value (UM1725 names, header values); the three instances (I2C1-I2C3) behave alike.
- II6 `__HAL_I2C_ENABLE / DISABLE / GET_FLAG / CLEAR_FLAG / ENABLE_IT / DISABLE_IT / GET_IT_SOURCE / CLEAR_ADDRFLAG / CLEAR_STOPFLAG` macros against RM0090 (flag clear sequences, which flags clear with write 0), `I2C_FREQRANGE / I2C_SPEED / I2C_RISE_TIME` results for the table of speeds used by the firmware.
- II7 compiled-out scope notes (I2C_Ex; no register callbacks): cases that check the macros `I2C_FLTR_ANOFF`, `I2C_FLTR_DNF`, `USE_HAL_I2C_REGISTER_CALLBACKS` have the values the target configuration says (compile-time facts as runtime CHECKs).

### I2P (`test_hal_i2c_poll.cpp`): controller, blocking mode
`HAL_I2C_Master_Transmit`, `HAL_I2C_Master_Receive`, `HAL_I2C_Mem_Write`, `HAL_I2C_Mem_Read`, `HAL_I2C_IsDeviceReady`.
- IP1 transmit: bus log (S, address | W, data ..., P), sizes 0 / 1 / 2 / 3 / 255 / 256, state BUSY_TX / MASTER during and READY after, ErrorCode, lock; DevAddress masking (bit 0 of the argument).
- IP2 receive: sizes 1 (ACK off before clearing ADDR, STOP after), 2 (POS), 3, N > 3 (ACK off at N-2, STOP at N-1): bus log with the nack on the last byte only (RM0090 27.3.3 receiver procedures), data delivered in order, state.
- IP3 errors: address nack (AF, `HAL_I2C_ERROR_AF`, STOP sent, HAL_ERROR), data nack, BUSY bit timeout (`hostI2cBusBusy`: HAL_BUSY after 25 ms, state READY), timeouts of every wait (`hostI2cStuck`, timeout argument 0 / 1 / n), arbitration lost, bus error, a call while state is not READY returns HAL_BUSY; ErrorCode values and state after each; PE off (HAL enables it: `HAL_I2C_Master_*` with PE = 0).
- IP4 Mem_Write / Mem_Read: 8 / 16-bit memory address (byte order, MSB first for 16 bit), restart before the read with address | R, sizes as above, the same error classes, state MEM mode.
- IP5 IsDeviceReady: ready on the first trial, ready on a later trial (nack then ack), never ready (HAL_ERROR after `Trials`; time spent), HAL_TIMEOUT, AF cleared between trials, STOP generated each trial, state READY, busy-bus handling; Trials = 0 / 1.
- IP6 10-bit addressing: `HAL_I2C_Master_Transmit` / `Receive` with `AddressingMode` 10-bit init: header 11110xx0, second byte, repeated start with 11110xx1 for a read; bus log exact.
- IP7 the HAL's use of the registers: order of CR1 / CR2 writes where RM0090 prescribes it (ACK before ADDR clear for a single byte; POS before ADDR clear for 2 bytes; STOP after BTF), `I2C_Flush_DR`.

### I2S (`test_hal_i2c_slave.cpp`): target mode, blocking, listen and address callbacks
`HAL_I2C_Slave_Transmit`, `HAL_I2C_Slave_Receive` (blocking, scripted external controller), `HAL_I2C_EnableListen_IT`, `HAL_I2C_DisableListen_IT`, `HAL_I2C_AddrCallback` (direction, match code) and `HAL_I2C_ListenCpltCallback` through the IT handlers.
- IS1 slave receive: bytes in order, ACK enabled by the HAL, stop ends the call (STOPF), size reached, errors: timeout (nothing addressed), AF, state SLAVE / READY, HAL_BUSY when not READY, size 0.
- IS2 slave transmit: bytes sent, nack from the controller ends it (AF cleared, HAL_OK), more bytes asked than given, state and ErrorCode, timeouts.
- IS3 address match: 7-bit OAR1, OAR2 with dual addressing (DUALF), general call (ENGC; GENCALL), 10-bit own address (header, second byte, repeated start read); non-matching addresses are ignored; direction reported to `AddrCallback` (`I2C_DIRECTION_TRANSMIT` = the controller transmits = the target receives; check the UM1725 wording) and the match code.
- IS4 listen: Enable / DisableListen state transitions (LISTEN), HAL_BUSY / HAL_ERROR returns, ListenCplt after a stop or a nack, listen with an own transfer.
- IS5 NOSTRETCH: overrun / underrun (OVR, `HAL_I2C_ERROR_OVR`), no clock stretching assumed.

### I2T (`test_hal_i2c_irq.cpp`): interrupt mode and the two IRQ handlers
`HAL_I2C_Master_Transmit_IT`, `Master_Receive_IT`, `Mem_Write_IT`, `Mem_Read_IT`, `Slave_Transmit_IT`, `Slave_Receive_IT`, `HAL_I2C_Master_Abort_IT`, `HAL_I2C_EV_IRQHandler`, `HAL_I2C_ER_IRQHandler`, `I2C_ITError` and the state machines behind them.
- IT1 each IT call: return codes, state / mode during the transfer, interrupts enabled (ITEVTEN, ITBUFEN, ITERREN as the HAL sets them; compare Table 124), completion callback exactly once with the right state, data and bus log (sizes 1, 2, 3, N, with the 1 / 2 / N-byte receive tricks), lock, HAL_BUSY when busy.
- IT2 EV handler paths one flag at a time and in combinations the RM0090 allows (SB, ADDR, ADD10, TXE, RXNE, BTF, STOPF): master tx, master rx, memory tx / rx, slave tx / rx; flags not enabled by CR2 are ignored (ITEVTEN / ITBUFEN gating).
- IT3 ER handler: BERR, ARLO, AF, OVR (each, with the right `HAL_I2C_ERROR_*` bit, the callback `HAL_I2C_ErrorCallback`, state after, flags cleared), several at once, AF in master tx with data pending, AF in slave tx (ends with SlaveTxCplt), bus error in a slave (ES0182 2.10), arbitration lost in a master, ITError when the other direction.
- IT4 Master_Abort_IT: state ABORT, STOP generated, `AbortCpltCallback`, the address check, abort when not master, abort while a transfer is running at each phase.
- IT5 slave IT: address match callback, SlaveTx / SlaveRx completion, early stop (STOPF before the size is reached), nack by the controller, data left over, listen interplay.

### I2Q (`test_hal_i2c_seq.cpp`): sequential transfer functions
`HAL_I2C_Master_Seq_Transmit_IT`, `Master_Seq_Receive_IT`, `Slave_Seq_Transmit_IT`, `Slave_Seq_Receive_IT` (+ the XferOptions `I2C_FIRST_FRAME`, `FIRST_AND_NEXT_FRAME`, `NEXT_FRAME`, `FIRST_AND_LAST_FRAME`, `LAST_FRAME_NO_STOP`, `LAST_FRAME`, `OTHER_FRAME`, `OTHER_AND_LAST_FRAME`), `I2C_ConvertOtherXferOptions`, restart handling.
- IQ1 each option for master transmit and receive: START / repeated START / STOP presence in the bus log exactly as UM1725 describes for the option (read the option descriptions), state after each frame (`BUSY_TX` -> `READY`, mode), callbacks per frame.
- IQ2 sequences: FIRST then NEXT then LAST (no extra START between frames, one STOP at the end), FIRST_AND_NEXT, LAST_NO_STOP followed by another FIRST, OTHER frames, direction change with restart.
- IQ3 slave sequential: first / next / last frame options, ACK handling at the last byte, listen state kept / ended, address callbacks per frame, stop and nack ends.
- IQ4 error paths in sequential mode (AF, timeout, HAL_BUSY, wrong option for the state) and the state after.

### I2D (`test_hal_i2c_dma.cpp`): DMA mode
`HAL_I2C_Master_Transmit_DMA`, `Master_Receive_DMA`, `Slave_Transmit_DMA`, `Slave_Receive_DMA`, `Mem_Write_DMA`, `Mem_Read_DMA`, `Master_Seq_Transmit_DMA`, `Master_Seq_Receive_DMA`, `Slave_Seq_Transmit_DMA`, `Slave_Seq_Receive_DMA`, `I2C_DMAXferCplt`, `I2C_DMAError`, `I2C_DMAAbort`.
- ID1 each DMA call: return codes (HAL_BUSY, HAL_ERROR on a failing `HAL_DMA_Start_IT`), DMA stream configured with the right addresses (DR, buffer), length (the HAL's size rule for the last bytes in receive: N-1 for >= 2 bytes? read UM1725 and RM0090 27.3.8: EOT_1, LAST bit), DMAEN / LAST / ITEVTEN / ITERREN set as the HAL sets them, callbacks assigned.
- ID2 completion: after the stream completes (`hostDmaComplete`, DMA IRQ handler) the I2C state, DMAEN cleared, LAST handling, STOP, callbacks exactly once (master tx waits for BTF; master rx; mem rx; slave tx / rx), bus log with data moved by the test through DR.
- ID3 errors: DMA transfer error (`hostDmaRaise` TEIF) -> `HAL_I2C_ERROR_DMA`, ErrorCallback, state; I2C error during DMA (AF, BERR, ARLO, OVR) -> DMA aborted (`I2C_DMAAbort`), abort callback ordering; abort of a master transfer in DMA mode.
- ID4 sequential DMA: options as in IQ1 with DMA, `LAST` bit behaviour; slave DMA: address callbacks, stop ends the transfer, DMA left running.

## Writer rules
As in the T4b brief (`../ffb-test-build/briefs/p12t4c_brief.md`): expected values from UM1725 / RM0090 / hand derivation, `characterization:` + `NR_I2C_<n>`, `KNOWN_ISSUE("V_I2C_<n> ...")` with the first failing assertion being the defect and a spec quote + HAL line in the report; names carry the requirement id; one behaviour per case; no sleeping; use `I2cFixture` and `i2cCounts` (never define HAL_I2C_* callbacks).
