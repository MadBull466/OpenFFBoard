/*
 * HalCan.h (P12 T5, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Register and bus model of bxCAN CAN1 (0x40006400) and CAN2 (0x40006800) of the STM32F405/407, on the trapping bus of HalRegisterSpace.h.
 * Source: RM0090 Rev 22 chapter 32 (32.4 operating modes, Figure 336; 32.5 test modes; 32.7.1 transmission and Figure 340; 32.7.3 reception and
 * Figure 341; 32.7.4 filtering, Figures 342 / 343 and the priority rules; 32.7.5 message storage; 32.8 interrupts, Figure 348; 32.9 registers).
 * Confidence: High for the registers, reset values, mode handshakes, mailbox and FIFO state machines and the filter scale / mode / numbering /
 * priority rules; Medium for transmit arbitration (the bit-level order of standard and extended identifiers is a documented simplification) and for
 * what the manual leaves open (listed below); Low for everything on the wire (the bus is a test hook).
 *
 * Registers (word access): MCR 0x00 (reset 0x00010002, writable bits 16, 7:0; bit 15 RESET is a self-clearing master reset), MSR 0x04 (reset 0x0C02;
 * SLAKI / WKUI / ERRI are rc_w1), TSR 0x08 (reset 0x1C000000; RQCPx / TXOKx / ALSTx / TERRx are rc_w1 where RQCP clears the other three, ABRQx are rs),
 * RF0R / RF1R 0x0C / 0x10 (RFOM rs, FOVR and FULL rc_w1, FMP read-only), IER 0x14 (mask 0x00038F7F), ESR 0x18 (LEC rw bits 6:4), BTR 0x1C (reset 0x01230000, mask 0xC37F03FF,
 * writable only while INAK = 1), transmit mailboxes TIxR / TDTxR / TDLxR / TDHxR 0x180 + 0x10 * x (writable only while the mailbox is empty, TME = 1), receive mailboxes
 * RIxR / RDTxR / RDLxR / RDHxR 0x1B0 + 0x10 * x (read-only: a write is a fault), filter registers only in CAN1's block: FMR 0x200 (reset 0x2A1C0E01, mask 0x3F01), FM1R 0x204,
 * FS1R 0x20C, FFA1R 0x214 (writable only with FINIT = 1), FA1R 0x21C, filter bank registers F0R1 .. F27R2 0x240 + 8 * n (writable while the bank is not active or FINIT = 1).
 * Writes that RM0090 "write protects" are ignored and counted (hostCanProtectedWrites); CAN2's block has no filter registers (a fault).
 *
 * Modes (32.4): reset state is Sleep (SLEEP = 1, SLAK = 1). INAK follows INRQ and SLAK follows SLEEP, each after `syncTicks` HAL ticks (default 0 = at once) and independent of the other
 * (after the reset INRQ = 1 gives INAK = 1 while SLAK stays 1 until SLEEP is cleared: the HAL's own Init sequence relies on it; RM0090 32.4.3 only says the software must also clear SLEEP; Medium
 * confidence, NEEDS REVIEW). SLAKI is set when SLAK sets with SLKIE = 1 and cleared when SLAK clears. hostCanStuckSync stops all mode changes (polling loops time out). Normal mode is INAK = 0 and
 * SLAK = 0: frames are neither sent nor received outside it, and a TXRQ outside it stays pending. FINIT = 1 deactivates reception. RESET = 1: MCR to its reset value, FIFOs emptied, Sleep.
 * Transmission (32.7.1): TXRQ with TME = 1 clears TME and RQCP / TXOK / ALST / TERR of the mailbox. A pending mailbox is scheduled by identifier (key = base identifier, then IDE, then extended
 * identifier; equal keys: lower mailbox first) or in request order (TXFP). In Normal mode and with the bus free it is sent in `frameTicks` ticks (default 0) and then the next entry of the outcome queue
 * decides (default success): success sets RQCP and TXOK, TME = 1, logs the frame (hostCanTxAt), and with LBKM the frame is also received through the filters; arbitration lost (ALST) or error (TERR,
 * LEC): with NART the request is completed (RQCP, TXOK = 0, TME = 1), else it stays pending with ALST / TERR set and is sent again. Without an acknowledging peer (hostCanSetBusAck(0)) and not
 * in loop back mode a frame fails with TERR and LEC = 3. ABRQ on a pending, not yet sent mailbox empties it at once (RQCP = 1, TXOK = 0); on a mailbox being sent it ends with the frame. CODE is the
 * lowest empty mailbox, else the lowest-priority pending one; LOWx marks the lowest-priority mailbox when more than one is pending. The time stamp TIME of a sent mailbox is captured when TTCM = 1
 * (the 16-bit counter is a test value, hostCanSetTimer).
 * Reception (32.7.3 / 32.7.4): hostCanRx runs the filters (banks of the instance: CAN1 uses banks below CAN2SB, CAN2 the others; list mode before mask mode at equal scale, 32-bit before 16-bit, then
 * the lower filter number; filter numbers count every bank of the FIFO including deactivated ones, 32-bit list 2 per bank, 32-bit mask 1, 16-bit list 4, 16-bit mask 2; the numbering restarts at 0 for
 * each FIFO and each instance (Medium: Figure 343 shows one instance)), stores the frame with FMI and the time stamp in the FIFO, FMP / FULL / FOVR as in Figure 341 (RFLM = 0 overwrites the newest
 * message, RFLM = 1 discards the new one), RFOM releases the output mailbox. Interrupt request lines (Figure 348) are not delivered: hostCanIrqPending gives them.
 * Error state (32.7.6) is set by the test: hostCanSetCounters (REC, 9-bit TEC; EWGF at 96, EPVF above 127, BOFF above 255) and hostCanRaiseError (LEC 1..6); ERRI is set when an enabled source fires.
 *
 * Not modelled: bit timing (BTR is stored), time-triggered mode other than the time stamp, bus-off recovery (ABOM), wake-up by hardware other than hostCanBusActivity, debug freeze, error counter
 * evolution, the CAN2 / CAN1 shared SRAM details beyond the bank split, interrupt delivery.
 */
#ifndef FFB_HAL_CAN_H
#define FFB_HAL_CAN_H

#include <stdint.h>

#include "HalRegisterSpace.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { HOST_CAN_COUNT = 2 }; /* 0 = CAN1, 1 = CAN2 */

typedef struct HostCanFrame {
	uint32_t id;  /* 11 bit (ide = 0) or 29 bit (ide = 1) */
	uint8_t ide, rtr, dlc;
	uint8_t data[8];
} HostCanFrame;

enum { HOST_CAN_TX_OK = 0, HOST_CAN_TX_ARB_LOST, HOST_CAN_TX_ERROR };
enum { HOST_CAN_IRQ_TX = 1, HOST_CAN_IRQ_FIFO0 = 2, HOST_CAN_IRQ_FIFO1 = 4, HOST_CAN_IRQ_SCE = 8 };

void hostCanRegister(void);
void hostCanResetState(void);

/* Ticks a mode change (INAK / SLAK) takes, and ticks a frame takes on the bus (defaults 0). */
void hostCanSetSyncTicks(unsigned ticks);
void hostCanSetFrameTicks(unsigned ticks);
/* Every n-th read of MSR counts as one tick for the model (0 = off): for loops that read no tick, such as the counter loop of HAL_CAN_WakeUp. */
void hostCanSetPollReads(unsigned n);
void hostCanStuckSync(unsigned can, int stuck); /* 1: no mode change ever completes */
void hostCanHoldBus(unsigned can, int hold);    /* 1: the bus is busy, no frame starts */
void hostCanSetBusAck(unsigned can, int ack);   /* 1 (default): a peer acknowledges frames */
void hostCanSetTimer(unsigned can, uint16_t value); /* the time stamp captured at the next SOF */
/* Outcome of the next transmission attempts (queue of 8, empty = success). */
void hostCanQueueTxOutcome(unsigned can, int outcome);

/* Frames sent since the reset (in order). */
unsigned hostCanTxCount(unsigned can);
HostCanFrame hostCanTxAt(unsigned can, unsigned index);
void hostCanTxClear(unsigned can);

/* The peer: a frame arrives. Returns 1 stored, 0 not stored (filtered, wrong mode), -1 lost (FIFO full and locked, or overwritten: the new one replaced the newest). */
int hostCanRx(unsigned can, const HostCanFrame *frame);
/* Bus activity (a SOF) while in Sleep: WKUI = 1; with AWUM the hardware leaves Sleep. */
void hostCanBusActivity(unsigned can);

/* Error state set by the test. */
void hostCanSetCounters(unsigned can, unsigned rec, unsigned tec);
void hostCanRaiseError(unsigned can, unsigned lec);

/* Interrupt request lines (HOST_CAN_IRQ_*). */
unsigned hostCanIrqPending(unsigned can);
/* Advances the model to the current time (it also advances on every register access). */
void hostCanPoll(unsigned can);

uint32_t hostCanProtectedWrites(unsigned can); /* writes RM0090 write protects, ignored (BTR outside init, filter registers outside FINIT, busy mailbox) */

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_CAN_H */
