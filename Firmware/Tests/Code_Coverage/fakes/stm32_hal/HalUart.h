/*
 * HalUart.h (P12 T4b, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Register model of USART1 (0x40011000), USART2 (0x40004400), USART3 (0x40004800), UART4 (0x40004C00), UART5 (0x40005000) and
 * USART6 (0x40011400) of the STM32F405/407 in asynchronous mode, on the trapping bus of HalRegisterSpace.h. Source: RM0090 Rev 22
 * chapter 30 (30.3 functional description, 30.6 registers). Confidence: High for the registers, reset values and the flag and
 * clear-sequence semantics; Medium for frame timing (a frame takes a configurable number of HAL ticks, default 1; the real time is
 * the baud rate divider times the frame bits: the divider is stored and read back, but does not set the timing); Low for everything
 * on the line side (the peer is a test hook).
 *
 * Registers (word access): SR 0x00 (reset 0x00C0), DR 0x04, BRR 0x08, CR1 0x0C, CR2 0x10, CR3 0x14, GTPR 0x18 (all reset 0).
 * Writable bits: SR none by software except "write 0 clears" on TC, RXNE, LBD, CTS (RM0090 30.6.1); DR 8:0; BRR 15:0; CR1 15 and
 * 13:0; CR2 14:12, 11:8, 6:5, 3:0 (UART4 / UART5: CLKEN, CPOL, CPHA, LBCL are not available and read 0); CR3 11:0 (UART4 / UART5:
 * CTSIE, CTSE, RTSE, SCEN are not available and read 0); GTPR 15:0. OVER8 reads 0 while LINEN, IREN or SCEN is set (RM0090 30.6.4).
 *
 * Transmitter (30.3.2): a write to DR (UE = 1, TE = 1) puts the frame into the shift register at once if it is free (TXE stays 1),
 * else into TDR (TXE = 0) until the running frame ends. With PCE = 1 the MSB of the data (bit 7, M = 1: bit 8) is replaced by the
 * parity (PS = 0 even, 1 odd; 30.3.7). A frame lasts `frameTicks` ticks. When a frame ends and TDR is empty, TC = 1 (TXE is 1); TC
 * clears with "read SR, then write DR" or by writing 0 to it, and not otherwise. With UE = 0 or TE = 0 a write to DR waits in TDR
 * (TXE = 0) and the frame starts when both are 1. SBK = 1 sends a break (10 / 11 low bits) after the current frame and clears itself
 * when the break ends (counted in hostUartBreakCount, not in the frame log). TE enable does not send an idle preamble (not modelled).
 * Receiver (30.3.3): frames come from the peer queue (one per `frameTicks` ticks while UE = 1 and RE = 1) or at once from
 * hostUartRxNow. The frame is given as the bits on the line (data and parity bit, 8 or 9 bits). A frame while RXNE = 1 sets ORE and
 * is lost (RDR keeps its content; no other flag is set). Otherwise RXNE = 1, PE if PCE and the parity of the whole frame is wrong,
 * FE / NF as the peer asks (NF only while ONEBIT = 0). DR reads the received frame (M = 0: 8 bits, M = 1: 9 bits; with PCE the MSB is the
 * received parity bit) and clears RXNE; "read SR, then read DR" clears PE, FE, NF, ORE and IDLE. A write to DR clears PE after a read of SR
 * (30.6.1 PE). IDLE (hostUartIdleLine): set once after a frame was received, not again before the next RXNE (30.6.1 IDLE).
 * Mute mode (30.3.6): RWU = 1 with WAKE = 0 drops frames until hostUartIdleLine (IDLE is not set then). WAKE = 1 (address mark): a frame whose
 * MSB (bit 7 / 8, with PCE bit 6 / 7) is 1 is an address: it matches when its low 4 bits equal CR2.ADD (RWU cleared if it was set, the frame is
 * received, no parity check on it); a non-matching address sets RWU (mute mode entered by hardware) and the frame is dropped; data frames in mute mode
 * are dropped. RWU cannot be written while RXNE = 1 and WAKE = 1. Not enforced: the "a data byte must have been received before muting" note.
 * Counters of what the RM0090 says should not happen: hostUartConfigWhileEnabled counts a change of BRR while UE = 1 (30.3.4 note: "the baud rate
 * register value should not be changed during communication") and, as a stricter model choice that the document does not state, a change of M, OVER8, PCE,
 * PS (CR1) or STOP (CR2) while UE = 1 (RM0090 30.6.4 only says M "must not be modified during a data transfer"); hostUartDisabledBusy counts a UE 1 to 0
 * while a frame is in the shift register or TDR is full (30.3.2 step 8: wait for TC = 1 before disabling the USART).
 * Polling loops that read no tick (HAL_MAX_DELAY) cannot advance the host time: hostUartSetPollReads(n) makes every n-th read of SR
 * count as one tick for this model (virtual time), 0 (default) turns that off.
 *
 * Not modelled: bit timing on the wire, TE / RE edges mid-frame, idle preamble, baud rate effects, hardware flow control (CTS / RTS),
 * synchronous / smartcard / IrDA / LIN break detection (LBD and CTS flags are never set), noise detection details, interrupt
 * delivery and DMA requests (the tests call the IRQ handlers and the DMA hooks themselves), wake-up from low power.
 */
#ifndef FFB_HAL_UART_H
#define FFB_HAL_UART_H

#include <stdint.h>

#include "HalRegisterSpace.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { HOST_UART_COUNT = 6 }; /* 0 = USART1, 1 = USART2, 2 = USART3, 3 = UART4, 4 = UART5, 5 = USART6 */

/* Error bits of a received frame (hostUartRxPush / hostUartRxNow). */
enum { HOST_UART_RX_FE = 1u << 0, HOST_UART_RX_NOISE = 1u << 1 };

void hostUartRegister(void);
void hostUartResetState(void);

/* Ticks per frame (>= 1, default 1). */
void hostUartSetFrameTicks(unsigned ticks);
/* Every n-th read of SR counts as one tick for the model (0 = off, default). */
void hostUartSetPollReads(unsigned n);

/* The peer: frames queued for reception (delivered one per frame time while UE = 1 and RE = 1), or delivered at once. */
void hostUartRxPush(unsigned uart, uint16_t lineBits, uint32_t errMask);
void hostUartRxNow(unsigned uart, uint16_t lineBits, uint32_t errMask);
/* A break on the line: a frame of zeros with a framing error (RM0090 30.3.3). */
void hostUartRxBreak(unsigned uart);
/* An idle frame on the line (see the header comment for the IDLE / mute mode semantics). */
void hostUartIdleLine(unsigned uart);
/* 1: the shift register never finishes a frame (e.g. the line is held): polling loops time out. */
void hostUartTxStuck(unsigned uart, int stuck);

/* Frames sent since the reset, as bits on the line (parity included), in order. */
unsigned hostUartTxCount(unsigned uart);
uint16_t hostUartTxAt(unsigned uart, unsigned index);
void hostUartTxClear(unsigned uart);
unsigned hostUartBreakCount(unsigned uart);

uint32_t hostUartConfigWhileEnabled(unsigned uart);
uint32_t hostUartDisabledBusy(unsigned uart);

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_UART_H */
