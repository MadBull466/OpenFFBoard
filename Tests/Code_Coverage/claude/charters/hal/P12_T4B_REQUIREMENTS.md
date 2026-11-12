# P12 T4b requirements charter: UART (real ST HAL, F407VG and F407VG_DISCO)

Status: 2026-11-12 (lead). Charter, platform (`fakes/stm32_hal/HalUart.{h,c}`), tests, review and gate done (see the plan, section "T4b result"; amendments: UA1 no flush / error code only in `Abort`, UI7 compiled out, UM3 non-matching address enters mute mode per RM0090 30.3.6, the model stops the UART4 / UART5 unavailable bits). T4 is split (T4a SPI done, T4b UART = this
charter, T4c I2C / I2C_Ex); each stage has its own charter, model, review, gate and approval.
Plan: `Tests/Code_Coverage/claude/plans/P12_ST_HAL_PLAN.md`. Rules as in `P12_T1_REQUIREMENTS.md` ... `P12_T4A_REQUIREMENTS.md`.

## Oracles

| Topic | Oracle |
|---|---|
| API semantics, parameters, notes, state / error code names | UM1725 Rev 8, chapter 70 "HAL UART Generic Driver" (HAL_UART_*, HAL_UARTEx_*, HAL_LIN_*, HAL_MultiProcessor_*, HAL_HalfDuplex_*; blocking / IT / DMA modes; callbacks; abort) |
| Registers, bits, reset values, flags and their clear sequences, frame format, parity, mute mode, baud rate, DMA use | RM0090 Rev 22, chapter 30 (30.3.1 frame, 30.3.2 transmitter, 30.3.3 receiver, 30.3.4 baud rate, 30.3.6 multiprocessor, 30.3.7 parity, 30.3.8 LIN, 30.3.10 half duplex, 30.3.13 DMA, 30.6 registers) |
| Instance addresses, DMA requests | RM0090 Table 1 (USART1 0x40011000, USART2 0x40004400, USART3 0x40004800, UART4 0x40004C00, UART5 0x40005000, USART6 0x40011400); Tables 43 / 44 (below) |

Text of the PDFs (not committed): `pdftotext -layout` of `Tests/Code_Coverage/shared/specs/<pdf>`; the lead put copies in
`../ffb-test-build/p12t4b-specs/rm0090.txt` and `um1725.txt` (outside the repo). Where UM1725 says nothing (return codes of failing calls, timeouts,
error codes in uncommon cases, the order of register writes), the case is a `characterization:` case with a NEEDS REVIEW entry (numbers from 313).

DMA requests (RM0090 Tables 43 / 44, controller 0 = DMA1, 1 = DMA2 of the model; channel 4 unless noted): USART1 DMA2 Tx stream 7, Rx stream 2 or 5;
USART2 DMA1 Tx 6, Rx 5; USART3 DMA1 Tx 3, Rx 1; UART4 DMA1 Tx 4, Rx 2; UART5 DMA1 Tx 7, Rx 0; USART6 DMA2 Tx 6 or 7, Rx 1 or 2, channel 5.
Index of an instance in `HalUart.h`: 0 USART1, 1 USART2, 2 USART3, 3 UART4, 4 UART5, 5 USART6.

## Compiled code (target configuration)

`stm32f4xx_hal_uart.c` with `USE_HAL_UART_REGISTER_CALLBACKS` 0 (gcov decides whether the Register / UnRegister functions are compiled: do not assume).
The module is "UART" only: no USART (synchronous) module is enabled (`HAL_USART_MODULE_ENABLED` off), no IrDA, no smartcard. Weak callbacks
(`HAL_UART_MspInit`, `MspDeInit`, the completion, error and abort callbacks, `HAL_UARTEx_RxEventCallback`) are replaced by strong test callbacks defined
once by the lead (see Writer rules). `USE_FULL_ASSERT` is off: parameter checks are not tested.

## Platform (lead)

`HalUart` model (header: what is modelled): SR / DR / BRR / CR1 / CR2 / CR3 / GTPR of the six instances, TXE / TC / RXNE / IDLE / ORE / NF / FE / PE with the
RM0090 clear sequences, parity generation and checking, 8 / 9 bit frames, TDR + shift register with frames of `frameTicks` ticks, SBK break, a peer queue
(`hostUartRxPush`, one frame per frame time while UE and RE) or immediate frames (`hostUartRxNow`), `hostUartRxBreak`, `hostUartIdleLine`, mute mode,
`hostUartTxStuck` for timeouts, `hostUartSetPollReads` so that a `HAL_MAX_DELAY` loop that reads no tick still sees time pass, counters of a configuration
change with UE = 1 (`hostUartConfigWhileEnabled`) and of a disable of UE while a frame is in the shift register or TDR is full (`hostUartDisabledBusy`).
Interrupt delivery and DMA requests are not modelled: the tests call `HAL_UART_IRQHandler` and the DMA hooks of T3 (`hostDmaComplete`, `hostDmaStepItems`,
`hostDmaRaise`, then `HAL_DMA_IRQHandler` of the stream) themselves. The UART HAL starts the streams through `HAL_DMA_Start_IT` and does not configure them:
the tests initialise `DMA_HandleTypeDef` (channel 4 / 5, direction, increments, normal or circular) with the real `HAL_DMA_Init` and link it (`__HAL_LINKDMA`).
PCLK: UART_SetConfig reads `HAL_RCC_GetPCLK1Freq` (USART2, USART3, UART4, UART5) or `HAL_RCC_GetPCLK2Freq` (USART1, USART6): the tests set RCC CFGR PPRE1 / PPRE2
and `SystemCoreClock` (or run `HAL_RCC_ClockConfig`) to known values and derive the expected BRR by hand from RM0090 30.3.4.

## Requirement groups

### UI: init, deinit, state, baud rate (UM1725, RM0090 30.3.4, 30.6)

| ID | Requirement | Oracle |
|---|---|---|
| UI1 | `HAL_UART_Init` writes CR1 (M, PCE, PS, TE, RE, OVER8 from the init structure; UE set at the end), CR2 STOP (all four codes; LINEN and CLKEN cleared), CR3 CTSE / RTSE (HwFlowCtl; SCEN, HDSEL, IREN cleared); state READY (both gState and RxState), error code NONE; MspInit called once; a change of CR1 / CR2 / BRR happens with UE = 0 (`hostUartConfigWhileEnabled` 0) | RM0090 30.6.4-30.6.6 |
| UI2 | BRR for each of the six instances at two PCLK values (the two buses differ), OVER8 0 and 1, several baud rates (9600, 115200, 921600, a rate with a carry of the fraction): the value equals the hand derivation of RM0090 30.3.4 (mantissa, fraction rounded to the nearest 1/16 or 1/8, carry into the mantissa; with OVER8 the fraction bit 3 is 0); ties of the rounding are `characterization:` | RM0090 30.3.4 |
| UI3 | Init of every instance leaves the registers of the others unchanged; UART4 and UART5 have no flow control bits / no CK bits (the model reads them 0) | RM0090 Table 1, 30.6 |
| UI4 | `HAL_UART_Init` with a NULL handle returns `HAL_ERROR`; first Init (state RESET) calls MspInit, a repeated Init (state READY) does not; 9 bit words with and without parity (M bit, PCE) | UM1725 |
| UI5 | `HAL_UART_DeInit`: UE cleared, state RESET (both), error code NONE, MspDeInit called once, NULL handle `HAL_ERROR`; the HAL does not reset the registers itself (characterization of what DeInit writes) | UM1725 |
| UI6 | `HAL_UART_GetState` follows READY / BUSY_TX / BUSY_RX / BUSY_TX_RX as the transfer functions set them (tx and rx state are separate: a Tx and an Rx may run at the same time); `GetError` returns the code bits | UM1725 |
| UI7 | Register / UnRegister callback functions (only if gcov shows them compiled): ID storage, `HAL_ERROR` for an invalid ID or a state that is not READY | UM1725 |

### UM: special modes (`test_hal_uart_modes.cpp`; UM1725, RM0090 30.3.6, 30.3.8, 30.3.10)

| ID | Requirement | Oracle |
|---|---|---|
| UM1 | `HAL_HalfDuplex_Init`: HDSEL set, LINEN / CLKEN / SCEN / IREN cleared, state READY; `HAL_HalfDuplex_EnableTransmitter` sets TE and clears RE, `EnableReceiver` the reverse; both `HAL_BUSY`-style returns when a transfer runs (characterization of the code); data of a transmit appears on the line | RM0090 30.3.10 |
| UM2 | `HAL_LIN_Init`: LINEN set, LBDL from the parameter (10 / 11 bit), HDSEL / CLKEN / SCEN / IREN cleared, OVER8 reads 0 (RM0090 30.6.4: forced to 0 by hardware); `HAL_LIN_SendBreak` sets SBK, the model sends the break (`hostUartBreakCount`) and clears SBK after a frame time | RM0090 30.3.8, 30.6.4 |
| UM3 | `HAL_MultiProcessor_Init`: address in ADD[3:0], WAKE method, LINEN / CLKEN / HDSEL / SCEN / IREN cleared; `EnterMuteMode` sets RWU, `ExitMuteMode` clears it; in mute mode received frames are dropped (idle line wake-up: `hostUartIdleLine` clears RWU and IDLE is not set; address mark: a matching address wakes up and is received, a non-matching one is dropped) | RM0090 30.3.6 |
| UM4 | The state and error handling of these functions: NULL handle `HAL_ERROR`, a busy state (characterization of the code), state READY after success | UM1725 |

### UP: polling transfers (UM1725, RM0090 30.3.2, 30.3.3)

| ID | Requirement | Oracle |
|---|---|---|
| UP1 | `HAL_UART_Transmit` 8 bit, no parity: the bytes appear on the line in order (`hostUartTxAt`), `HAL_OK`, state READY; the call waits for TC (the model's TC is set at the end of the last frame: `hostUartTxCount` equals Size and no frame is left in flight) | RM0090 30.3.2 step 8 |
| UP2 | Parity: with PCE the MSB of the data is replaced by the parity (even and odd, 8 bit word: 7 data bits + parity at bit 7; 9 bit word: 8 data bits + parity at bit 8), hand derived frames; 9 bit words without parity: `pData` is read as 16 bit values, only 9 bits are sent | RM0090 30.3.7, Table 146 |
| UP3 | `HAL_UART_Receive`: bytes equal the frames the peer sent (8 bit, no parity); with parity the HAL returns the data without the parity bit (7 data bits for 8 bit words: UM1725 / RM0090 30.6.2 say the MSB read is the parity bit and the HAL masks it: characterization of the masking); 9 bit words 16 bit values | RM0090 30.6.2 |
| UP4 | Timeout: `hostUartTxStuck` or no frames: `HAL_TIMEOUT` after about `Timeout` ticks (lower and upper bound), state READY afterwards, error code (characterization); `Timeout` 0; `HAL_MAX_DELAY` completes (with `hostUartSetPollReads`) | UM1725 |
| UP5 | Invalid arguments: NULL data pointer or Size 0 returns `HAL_ERROR`; a state that is not READY returns `HAL_BUSY` (Tx and Rx states are independent) | UM1725 |
| UP6 | Receive errors in a polling `Receive`: an overrun (ORE) ends the call with `HAL_ERROR` and error code ORE, state READY (UM1725 names the code); PE / FE / NE during `Receive`: what the HAL returns and sets (characterization) | RM0090 30.3.3, UM1725 error codes |
| UP7 | `HAL_UARTEx_ReceiveToIdle` (polling): returns at the idle line with the received length in `*RxLen`, or when Size frames arrived (then the event type is the TC type), `HAL_TIMEOUT` otherwise; `GetRxEventType` | UM1725 |
| UP8 | A frame sent while the previous one is running waits for TXE (the model's TDR): the output order is kept for Size 1, 2, 3, 16 | RM0090 30.3.2 |

### UQ: interrupt mode and the IRQ handler (UM1725, RM0090 30.3.2, 30.3.3, 30.6.4, 30.6.6)

| ID | Requirement | Oracle |
|---|---|---|
| UQ1 | `Transmit_IT`: state BUSY_TX, TXEIE set, `HAL_OK`; arguments / state errors as UP5; each `HAL_UART_IRQHandler` with TXE sends one frame; after the last frame TXEIE is cleared and TCIE set; when TC is set the next handler call clears TCIE, state READY, `HAL_UART_TxCpltCallback` exactly once | RM0090 30.3.2, UM1725 |
| UQ2 | `Receive_IT`: state BUSY_RX, RXNEIE (and PEIE only with parity, EIE) set; each handler call with RXNE reads one frame; after the last the interrupts are cleared, state READY, `RxCpltCallback` once; 8 / 9 bit, parity masking as UP3 | RM0090 30.3.3 |
| UQ3 | Errors in IT mode: ORE (blocking: transfer ended, state READY, error code ORE, `ErrorCallback`), PE / FE / NE (non-blocking: code set, `ErrorCallback`, the code cleared again, the transfer goes on); each only with its enable (PEIE, EIE, RXNEIE: the handler's conditions follow RM0090 30.6.1 / 30.6.6) | RM0090 30.6.1, UM1725 |
| UQ4 | A flag whose interrupt enable is 0 does nothing; the handler on an idle peripheral does nothing; the order when several flags are set (error before RXNE, TXE before TC) is a `characterization:` case | RM0090 30.6 |
| UQ5 | `ReceiveToIdle_IT`: IDLEIE set, `HAL_UARTEx_RxEventCallback` with the length at the idle line (reception ended, state READY, interrupts cleared) or with Size when all frames came; the IDLE flag is cleared (read SR, read DR) | UM1725, RM0090 30.6.1 IDLE |
| UQ6 | Interrupt and polling of Tx and Rx run at the same time (Tx state and Rx state are independent); `HAL_UART_GetState` shows BUSY_TX_RX | UM1725 |

### UD: DMA mode (UM1725, RM0090 30.3.13)

| ID | Requirement | Oracle |
|---|---|---|
| UD1 | `Transmit_DMA` / `Receive_DMA` start the DMA stream through `HAL_DMA_Start_IT` (peripheral address `&USART->DR`, direction, NDTR = Size, memory address), TC cleared (Tx), DMAT / DMAR set (Rx also EIE, PEIE with parity, ORE cleared), state BUSY_*, `HAL_OK`; arguments / state errors as UP5 | RM0090 30.3.13, 10.5 |
| UD2 | DMA completion (`hostDmaComplete`, `HAL_DMA_IRQHandler`): Tx: DMAT cleared and TCIE set, then the handler at TC ends it with `TxCpltCallback` once, state READY; Rx: DMAR / EIE / PEIE cleared, state READY, `RxCpltCallback` once; half transfer callbacks on HTIF; circular Rx mode keeps going | UM1725 |
| UD3 | DMA errors: a stream error (TEIF) gives `HAL_UART_ERROR_DMA` and `ErrorCallback` (Tx and Rx), state READY; ORE / PE / FE / NE during an Rx DMA (the handler treats every error as blocking, aborts the stream; the callback after the DMA abort callback ran) | UM1725 error names |
| UD4 | `HAL_UART_DMAPause` / `DMAResume` / `DMAStop`: DMAT / DMAR (and EIE / PEIE) cleared / set, `DMAResume` clears ORE, `DMAStop` aborts the streams and the state is READY (the DMA return codes are characterization) | UM1725 |
| UD5 | `ReceiveToIdle_DMA`: IDLEIE set; at the idle line with data in the buffer the RxEvent callback gets the length (normal mode: stream aborted, DMAR cleared, state READY; circular mode: reception goes on); complete transfer gives the Size event | UM1725 |
| UD6 | `HAL_UART_Transmit_DMA` after a polling transmit with TC still set: TC is cleared first (model: write 0 to TC) so the next TC is the end of the DMA | RM0090 30.6.1 TC |

### UA: abort (UM1725)

| ID | Requirement | Oracle |
|---|---|---|
| UA1 | `HAL_UART_Abort` / `AbortTransmit` / `AbortReceive` (blocking): interrupt enables cleared (TXEIE / TCIE, RXNEIE / PEIE / EIE / IDLEIE), DMA requests cleared and the streams aborted when DMA runs, state READY, no callback; amended after the UA writer: the HAL does not flush DR / RXNE / IDLE (UM1725 silent: characterization), and only `HAL_UART_Abort` resets the error code (the other two: characterization); with nothing running `HAL_OK` | UM1725 |
| UA2 | The `_IT` variants: the same, the abort callbacks (`AbortCplt`, `AbortTransmitCplt`, `AbortReceiveCplt`) exactly once, after the DMA abort callbacks (completed with `hostDmaComplete` / the DMA IRQ handler as the model needs); without DMA the callback runs at once | UM1725 |
| UA3 | Abort with Tx and Rx both running by IT or DMA (each of the combinations, a few): both ended, a single `AbortCplt` | UM1725 |
| UA4 | Aborting only the transmitter leaves a running reception untouched (state, interrupts) and the reverse | UM1725 |

## Defect candidates to check (reserved V102 onwards; assigned V102-V105; the SPI extra clocked frame, if the owner promotes it, becomes V106)

Look for, and only report with a test that states the intended behaviour as `KNOWN_ISSUE`: a baud rate register that differs from the RM0090 derivation, a
configuration change with UE = 1 (counter), UE disabled while a frame is in flight (counter), a callback called twice or not at all, an error callback without
the error bit set, an error bit that stays set in `ErrorCode` after the transfer ended and a new one started, a polling loop that never ends, a state that
stays BUSY after an error, interrupt enables that stay set after an abort, ORE ignored in some modes (for example `HAL_MAX_DELAY` polling). Where UM1725 is silent
(for instance PE / FE / NE during a polling `Receive`) it is a `characterization:` case, not a defect.

## Writer rules

As in `P12_T1_REQUIREMENTS.md` and `P12_T4A_REQUIREMENTS.md`. Test files (`claude/cases_hal/`): `test_hal_uart_init.cpp` (UI), `test_hal_uart_modes.cpp` (UM),
`test_hal_uart_poll.cpp` (UP), `test_hal_uart_irq.cpp` (UQ), `test_hal_uart_dma.cpp` (UD), `test_hal_uart_abort.cpp` (UA). Writers never edit `fakes/stm32_hal/`; a missing
model feature is reported to the lead. Every file-local symbol (handles, helpers) carries the file's prefix (`uartInit`, `uartModes`, `uartPoll`, `uartIrq`,
`uartDma`, `uartAbort`) or sits in an anonymous namespace. The strong `HAL_UART_*` callbacks (MspInit, MspDeInit, TxCplt, TxHalfCplt, RxCplt, RxHalfCplt, ErrorCallback,
AbortCplt, AbortTransmitCplt, AbortReceiveCplt, `HAL_UARTEx_RxEventCallback`) are defined ONCE by the lead in `claude/cases_hal/test_hal_uart_support.cpp` with the
counters `uartCounts` and the fixture `UartFixture` of `UartTestSupport.h`: every UART test file includes that header, uses `UartFixture` (it resets the counters)
and never defines these symbols itself. Vendor bug numbers are assigned by the lead after all reports are in: use `V1xx` placeholders (`V_UART_<n>`) in markers
and the lead renames them. NEEDS REVIEW numbers likewise (`NR_UART_<n>`).
