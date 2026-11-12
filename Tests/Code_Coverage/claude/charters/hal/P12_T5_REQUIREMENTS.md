# P12 T5 requirements charter: CAN (real ST HAL, F407VG and F407VG_DISCO)

Status: 2026-11-12 (lead). Charter and platform (`fakes/stm32_hal/HalCan.{h,c}`, lead probe `test_hal_can_probe.cpp`, shared callbacks `CanTestSupport.h` / `test_hal_can_support.cpp`) done; tests, review, gate pending.
Plan: `Tests/Code_Coverage/claude/plans/P12_ST_HAL_PLAN.md`. Rules as in the T1-T4c charters. Both targets compile the same `stm32f4xx_hal_can.c` (identity check); the same test files run in both executables.

## Oracles

| Topic | Oracle |
|---|---|
| API semantics, parameters, state / error code names, filter and mailbox naming | UM1725 Rev 8, chapter 9 "HAL CAN Generic Driver" (`../ffb-test-build/p12t4c-specs/um1725.txt`, from line 5130) |
| Registers, reset values, mode handshakes (INRQ / INAK, SLEEP / SLAK), transmit mailbox states (Figure 340), receive FIFO (Figure 341), filter scale / mode / numbering / priority (Figures 342 / 343), interrupts (Table / Figure 348), error flags | RM0090 Rev 22 chapter 32 (`rm0090.txt`, from line 58616: 32.4 modes 58823, 32.7 functional description 59036, 32.8 interrupts 59744, 32.9 registers 59848) |
| Instance addresses | CAN1 0x40006400, CAN2 0x40006800 (RM0090 Table 1); model index 0, 1; the filter registers exist only in CAN1's block and are shared (CAN2SB splits the 28 banks) |

Where UM1725 / RM0090 say nothing (return codes of failing calls, error codes in uncommon cases, register write order, which of two flags the HAL checks first) the case is `characterization:` with a NEEDS REVIEW entry (numbers from 327; writers use `NR_CAN_<n>` placeholders and `V_CAN_<n>` for `KNOWN_ISSUE`).

## Compiled code (target configuration)

`stm32f4xx_hal_can.c`, `USE_HAL_CAN_REGISTER_CALLBACKS` 0 (Register / UnRegister functions compiled out: gcov decides). Weak callbacks (`HAL_CAN_MspInit`, `MspDeInit`, `TxMailbox0/1/2CompleteCallback`, `TxMailbox0/1/2AbortCallback`, `RxFifo0/1MsgPendingCallback`, `RxFifo0/1FullCallback`, `SleepCallback`, `WakeUpFromRxMsgCallback`, `ErrorCallback`) are strong test callbacks defined once by the lead (`canCounts`, order log). `USE_FULL_ASSERT` off.

## Platform (lead): `HalCan` model

Read the header comment of `HalCan.h`. Key points: INAK follows INRQ and SLAK follows SLEEP independently after `hostCanSetSyncTicks` ticks (`hostCanStuckSync` for timeouts; after the reset SLEEP = 1, so INRQ alone gives INAK = 1 with SLAK still 1, which is what `HAL_CAN_Init` relies on); mailboxes with TME / CODE / LOW and the request, abort and result flags, frames sent after `hostCanSetFrameTicks` ticks (0 = at once, so a TXRQ in Normal mode completes inside the register write unless `hostCanHoldBus(can, 1)` or a frame time keeps it pending), results from `hostCanQueueTxOutcome` (success / arbitration lost / error), `hostCanSetBusAck(0)` for an acknowledgement error, the sent-frame log (`hostCanTxAt`), loop back (BTR.LBKM) feeds sent frames through the filters, `hostCanRx(can, frame)` for frames from the bus (filters with FMI, FIFO 0 / 1 of three entries, FMP / FULL / FOVR, RFLM), `hostCanBusActivity` (WKUI, AWUM), `hostCanSetCounters` / `hostCanRaiseError` (ESR counters, flags, LEC, ERRI per IER), `hostCanIrqPending` (the four interrupt request lines, `HOST_CAN_IRQ_*`; the interrupts are not delivered: tests call `HAL_CAN_IRQHandler` themselves), `hostCanPoll`, `hostCanProtectedWrites` (writes RM0090 write protects, ignored: BTR outside init, filter registers outside FINIT, a pending mailbox's registers). Register writes outside the model (reserved offsets, CAN2's filter registers, writes to the receive mailboxes) are faults that fail the test case.
Model decisions the manual leaves open (state them in the report if a case depends on them): the INAK / SLAK independence above; arbitration key = base identifier, IDE, extended identifier (equal keys: lower mailbox first); filter numbering restarts per FIFO and per instance and counts deactivated banks; a message that matches filters of both FIFOs takes the better class (32-bit list, 32-bit mask, 16-bit list, 16-bit mask) then the lower number; FULL / FOVR clear only by software; the time stamp captured at SOF is the test value `hostCanSetTimer` and only written to a sent mailbox's TIME when TTCM = 1; the 16-bit filter mask format (FxR1[15:0] id, [31:16] mask; FxR2 likewise; list mode: four ids).
`HAL_CAN_Init` / `Start` / `Stop` / `RequestSleep` / `WakeUp` wait for the mode flags with `HAL_GetTick` timeouts of 10 ms (one tick per call).
`HAL_CAN_ConfigFilter` addresses the filter registers through `CAN1` for both instances; `SlaveStartFilterBank` is CAN2SB.

## Requirement groups (one writer, one file each)

### CI (`test_hal_can_init.cpp`): lifecycle
`HAL_CAN_Init`, `HAL_CAN_DeInit`, `HAL_CAN_Start`, `HAL_CAN_Stop`, `HAL_CAN_RequestSleep`, `HAL_CAN_WakeUp`, `HAL_CAN_IsSleepActive`, `HAL_CAN_GetState`, `HAL_CAN_GetError`, `HAL_CAN_ResetError`.
- CI1 Init: NULL handle; state RESET runs MspInit once; the register writes of the Init struct (MCR bits TTCM, ABOM, AWUM, NART, RFLM, TXFP; BTR SJW, TS1, TS2, BRP, mode bits LBKM / SILM for the four modes), state READY / ErrorCode NONE at the end, each timeout (INAK never set; SLAK never cleared; INAK not left) returns `HAL_ERROR` with `HAL_CAN_ERROR_TIMEOUT` and state ERROR, Init of an already initialised handle, all three instances (CAN1, CAN2).
- CI2 DeInit: `HAL_CAN_Stop`, MCR reset sequence, MspDeInit once, ErrorCode NONE, state RESET.
- CI3 Start: from READY: INRQ cleared, INAK cleared, state LISTENING, error code cleared; from other states `HAL_ERROR` with `HAL_CAN_ERROR_NOT_READY`; timeout; Stop: from LISTENING: INRQ set, state READY, SLEEP cleared (read the source for the exact write set); from other states `HAL_CAN_ERROR_NOT_STARTED`; timeout.
- CI4 RequestSleep / WakeUp / IsSleepActive: state READY or LISTENING required (UM1725), SLEEP bit, SLAK, WakeUp timeout, IsSleepActive reads SLAK, the other-state errors.
- CI5 GetState / GetError / ResetError: every `HAL_CAN_STATE_*` and `HAL_CAN_ERROR_*` value (UM1725 names, header coding), ResetError only in READY or LISTENING.
- CI6 compiled-out scope notes (register callbacks 0) and the test modes: loop back / silent / combined are BTR bits only.

### CF (`test_hal_can_filter.cpp`): filters
`HAL_CAN_ConfigFilter` (all registers it writes) and the filter behaviour seen through `hostCanRx` and `HAL_CAN_GetRxMessage` (FilterMatchIndex).
- CF1 ConfigFilter: state READY or LISTENING required, otherwise `HAL_ERROR` with `HAL_CAN_ERROR_NOT_INITIALIZED`; FMR.FINIT set during and cleared after; CAN2SB (`SlaveStartFilterBank`) written; FA1R deactivate / activate; FM1R (mask / list), FS1R (16 / 32 bit), FFA1R (FIFO 0 / 1), the register values written for the four scale-mode combinations (IdHigh / IdLow / MaskIdHigh / MaskIdLow packing, hand-derive from RM0090 Figure 342), all 28 banks, bank numbers above 27, other bits of FMR kept.
- CF2 receive through filters: identifiers standard and extended, IDE and RTR matching, mask don't-care bits, list exact match, 16-bit filters (two per bank in mask mode, four in list mode), FIFO assignment, deactivated banks do not match, FINIT = 1 deactivates reception, no filter = nothing is stored.
- CF3 filter match index and priority (RM0090 Figures 342 / 343 and the priority rules): numbering across banks and scales, deactivated banks count, list before mask, 32-bit before 16-bit, lower number first, FMI in `HAL_CAN_GetRxMessage` (`FilterMatchIndex`).
- CF4 CAN1 / CAN2 bank split (CAN2SB 0, 14, 28): banks below it filter CAN1, the others CAN2.

### CT (`test_hal_can_txrx.cpp`): mailboxes and FIFOs
`HAL_CAN_AddTxMessage`, `HAL_CAN_AbortTxRequest`, `HAL_CAN_GetTxMailboxesFreeLevel`, `HAL_CAN_IsTxMessagePending`, `HAL_CAN_GetTxTimestamp`, `HAL_CAN_GetRxMessage`, `HAL_CAN_GetRxFifoFillLevel`.
- CT1 AddTxMessage: state READY / LISTENING required (error `HAL_CAN_ERROR_NOT_INITIALIZED`), no free mailbox gives `HAL_CAN_ERROR_PARAM`? (read UM1725; characterization where silent) and `HAL_ERROR`, the mailbox chosen (CODE), the register values written (std / ext identifier, IDE, RTR, DLC, TGT when TTCM, data bytes in TDL / TDH, TXRQ last), the three mailboxes in order, `pTxMailbox` result as a bit mask (`CAN_TX_MAILBOX0 / 1 / 2`), the sent-frame log as oracle, hold the bus (`hostCanHoldBus`) to keep mailboxes pending.
- CT2 free level, `IsTxMessagePending` (per mailbox masks and combinations), Abort (pending mailbox emptied, mailbox being sent, empty mailbox; the mask of mailboxes; return codes by state), TX priority by identifier / TXFP request order observed through the log, NART with arbitration lost and error (`hostCanQueueTxOutcome`).
- CT3 GetTxTimestamp (TTCM, `hostCanSetTimer`; TIME field; mailbox argument), GetRxMessage: std / ext / remote frames, DLC 0..8, data bytes, FMI, timestamp, release of the FIFO output mailbox (RFOM), the next message after release, empty FIFO returns `HAL_ERROR` with `HAL_CAN_ERROR_PARAM`, state requirement, fill level 0..3, overrun with RFLM 0 (newest overwritten) and 1 (newest discarded), both FIFOs.
- CT4 loop back end to end (send then receive through a filter), 11-bit and 29-bit identifiers, all 8 data bytes, the `ExtId` / `StdId` split of the header structs.

### CQ (`test_hal_can_irq.cpp`): notifications and the IRQ handler
`HAL_CAN_ActivateNotification`, `HAL_CAN_DeactivateNotification`, `HAL_CAN_IRQHandler` and every callback.
- CQ1 Activate / Deactivate: IER bits for each `CAN_IT_*` (all 14 values), state READY / LISTENING required (`HAL_CAN_ERROR_NOT_INITIALIZED`), several bits at once, bits outside the IER mask.
- CQ2 IRQ handler transmit: RQCP0 / 1 / 2 with TXOK (complete callback of that mailbox, flag cleared by writing 1) and with ALST / TERR / abort (abort callback), the order and exactly-once rules, handler with no enabled source does nothing.
- CQ3 FIFO: message pending, full, overrun for FIFO 0 and 1 (callbacks, FOVR / FULL cleared by the HAL, ErrorCode `HAL_CAN_ERROR_RX_FOV0 / 1`, ErrorCallback), enable bits gate each branch.
- CQ4 status change: wake-up (WKUI, `WakeUpFromRxMsgCallback`), sleep (SLAKI, `SleepCallback`), error interrupt: EWGF, EPVF, BOFF, LEC 1..6 mapped to `HAL_CAN_ERROR_STF`, `FOR`, `ACK`, `BR`, `BD`, `CRC`, the MSR ERRI clear, ESR LEC clear, ErrorCallback with the accumulated ErrorCode, combined errors, and the ErrorCode `HAL_CAN_ERROR_EWG / EPV / BOF`.
- CQ5 shared: callbacks seen through `canCounts` (kinds and order), handler on CAN2, `hostCanIrqPending` as the oracle of which lines should be pending.

## Writer rules
As in the T4c brief (`../ffb-test-build/briefs/p12t5_brief.md`): expected values from UM1725 / RM0090 / hand derivation, `characterization:` + `NR_CAN_<n>`, `KNOWN_ISSUE("V_CAN_<n> ...")` with the first failing assertion being the defect and a spec quote + HAL line in the report; names carry the requirement id; one behaviour per case; no sleeping; use `CanFixture` and `canCounts` (never define HAL_CAN_* callbacks).
