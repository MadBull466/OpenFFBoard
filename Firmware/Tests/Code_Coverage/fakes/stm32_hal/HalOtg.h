/*
 * HalOtg.h (P12 T8, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Model of the USB OTG full-speed core of the STM32F405/407 (OTG_FS, 0x50000000-0x5003FFFF) in DEVICE mode: the registers of the global block (0x000-0x03C, 0x100-0x10C),
 * the device block (0x800-0x834), the four IN and four OUT endpoint register sets (0x900 + 0x20 * n, 0xB00 + 0x20 * n), PCGCCTL (0xE00), the data FIFO windows
 * (0x1000 * (n + 1)), the receive FIFO with its packet status entries and four transmit FIFOs. The USB host is played by the test through the hostOtgHost* calls.
 * Source: RM0090 Rev 22 chapter 34 (34.12 FIFO architecture, 34.13 device functional description, 34.14 device programming model, 34.16.1-34.16.6 registers: Tables 198-201,
 * the register descriptions 34.16.2 global, 34.16.4 device, 34.16.5 PCGCCTL). Confidence: High for the register map, reset values, write masks, rc_w1 / rs / w behaviour and the
 * soft reset and flush handshakes; Medium for the endpoint transfer behaviour (when XFRC / STUP are raised, NAK / STALL priorities, ITTXFE, DAINT derivation from the masks), the
 * receive status entries and the TXFE level; Low for the speed of anything and for the reserved offsets (below).
 *
 * Registers: each register has a reset value, a write mask (rw bits), an rc_w1 mask and read-only bits as in RM0090; GRSTCTL (CSRST / TXFFLSH / RXFFLSH are rs bits that clear
 * themselves, at once or at the n-th read of GRSTCTL, hostOtgSetPollReads, default 1; AHBIDL reads 1; CSRST resets every CSR except DCFG.DSPD, PCGCCTL and flushes all FIFOs),
 * GINTSTS (rc_w1 bits, derived read-only bits RXFLVL, IEPINT, OEPINT, CMOD, GINAKEFF / GONAKEFF set and cleared through DCTL SGINAK / CGINAK / SGONAK / CGONAK),
 * GRXSTSR (peek) / GRXSTSP (read pops one entry, empty gives 0), DAINT (derived: bit n = DIEPINTn & (DIEPMSK | TXFE if DIEPEMPMSK bit n), bit 16 + n = DOEPINTn & DOEPMSK),
 * DCTL (SGINAK / CGINAK / SGONAK / CGONAK write-only and read 0, GINSTS / GONSTS read only), DTXFSTSn (free words of the TxFIFO), HNPTXSTS (free space of TxFIFO 0),
 * DIEPCTLn / DOEPCTLn (EPENA and EPDIS rs, SNAK / CNAK write only, STALL, NAKSTS read only, USBAEP, MPSIZ, TXFNUM; endpoint 0 has the 2 bit MPSIZ 64 / 32 / 16 / 8),
 * DIEPINTn / DOEPINTn (rc_w1; TXFE derived from the TxFIFO level and GAHBCFG.TXFELVL). GUSBCFG.PHYSEL always reads 1. Host mode registers 0x400-0x7FF, reserved offsets and
 * FIFO windows of endpoints above 3 are RESERVED: the model counts the access (hostOtgReservedAccesses), reads 0 and ignores the write (no fault: RM0090 does not say what the
 * core does; HAL_PCD_EP_Close writes DEACHMSK at 0x83C (0x838 is DEACHINT) which RM0090 does not define for the OTG_FS core).
 * FIFOs: the receive FIFO is a word ring of depth GRXFSIZ holding, as in RM0090 34.12.1, one status entry per packet (PKTSTS, DPID, BCNT, EPNUM) followed by the data words, and the
 * "transfer completed" / "SETUP completed" entries. The four TxFIFOs have the depth of DIEPTXF0 / DIEPTXF1-3 (bits 31:16); the window 0x1000 * (n + 1) writes TxFIFO n and
 * every window reads the receive FIFO. A write to a full TxFIFO, a data read with nothing pending and a write to a window of an endpoint that is not enabled are faults only when
 * hostOtgStrictFifo(1) (default 1) for the first two (overflow, underflow); counters hostOtgFifoOverflows / hostOtgFifoUnderflows.
 * Host side (device mode transfers, RM0090 34.13 / 34.14 and the IN / OUT / SETUP flows of 34.17): hostOtgHostSetup (SETUP token + 8 data bytes: status entries, STUPCNT--, STUP set when the
 * SETUP completed entry is popped), hostOtgHostOut (OUT token + data: NAK when the endpoint is NAKing or the global OUT NAK is set or the FIFO is full, STALL when STALL is set,
 * OTEPDIS when the endpoint is not enabled; else the data enters the receive FIFO, XFRSIZ / PKTCNT decrease, a short or last packet ends the transfer: EPENA clears and a completed entry
 * follows; XFRC is set when that entry is popped), hostOtgHostIn (IN token: STALL, NAK when not enabled, ITTXFE + NAK when the TxFIFO lacks the data of the next packet, else the packet
 * leaves the FIFO, XFRSIZ / PKTCNT decrease and at PKTCNT = 0 XFRC is set and EPENA clears), hostOtgUsbReset (USBRST), hostOtgEnumDone (ENUMSPD = full speed, ENUMDNE),
 * hostOtgSuspend (SUSPSTS, ESUSP, USBSUSP), hostOtgResume (WKUPINT), hostOtgSof, hostOtgVbus (GOTGCTL.BSVLD, SRQINT / GOTGINT.SEDET), hostOtgDisconnectEvent (DISCINT).
 * hostOtgIrqPending = GAHBCFG.GINTMSK & (GINTSTS & GINTMSK) != 0 (the OTG_FS line to the NVIC).
 *
 * GINTSTS.CMOD and GINTSTS.OTGINT are NOT derived (CMOD follows GUSBCFG.FHMOD / FDMOD, OTGINT follows GOTGINT, in RM0090): the test pokes them (the HAL tests of the mode switch and of the
 * OTG interrupt do), NEEDS REVIEW. OTEPDIS is set only for a control OUT endpoint (RM0090 DOEPINT bit 4); a bulk / interrupt OUT endpoint that is not armed sets NAK.
 * FIFO RAM: 320 words (RM0090 34.13). The first TxFIFO data write or host packet while a FIFO (GRXFSIZ depth, DIEPTXFx start + depth) lies outside the RAM is a bus fault.
 * Not modelled: host mode, DMA, the high speed core (OTG_HS is not tested), OTG protocols (SRP, HNP), isochronous endpoints (frame numbers, EONUM, incomplete isochronous),
 * the periodic frame interval, babble / CRC errors, the clock domains (PCGCCTL bits are stored), the USB turnaround time effect, power down, timing, DCFG.NZLSOHSK / STALL on
 * non zero status OUT, the debug FIFO RAM window at 0x20000, the STALL clearing by SETUP, and any behaviour on ID pin.
 */
#ifndef FFB_HAL_OTG_H
#define FFB_HAL_OTG_H

#include <stdint.h>

#include "HalRegisterSpace.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HOST_OTG_BASE 0x50000000u

typedef enum {
	HOSTOTG_ACK = 0,   /* the device accepted / sent the packet */
	HOSTOTG_NAK,
	HOSTOTG_STALL,
	HOSTOTG_IGNORED    /* no such endpoint / device in soft disconnect: no handshake */
} HostOtgHandshake;

void hostOtgRegister(void);
void hostOtgResetState(void);

/* GRSTCTL: the n-th read while CSRST / TXFFLSH / RXFFLSH is set completes the reset (1 = first read, default; 0 = never). */
void hostOtgSetPollReads(unsigned n);
/* GAHBCFG.GINTMSK & ((GINTSTS & GINTMSK) != 0): the OTG_FS interrupt line. */
int hostOtgIrqPending(void);

/* Host side. data may be NULL when len is 0. pid1 = 1: DATA1, else DATA0 (the PID of the received packet, as in the RX status entry). */
HostOtgHandshake hostOtgHostSetup(const uint8_t *data8);
HostOtgHandshake hostOtgHostOut(unsigned ep, const uint8_t *data, unsigned len, int pid1);
HostOtgHandshake hostOtgHostIn(unsigned ep, uint8_t *buf, unsigned cap, unsigned *len);
void hostOtgUsbReset(void);
void hostOtgEnumDone(void);
void hostOtgSuspend(void);
void hostOtgResume(void);
void hostOtgSof(unsigned frame);
void hostOtgVbus(int session);
void hostOtgDisconnectEvent(void);
/* Recomputes the derived registers (DAINT, GINTSTS.RXFLVL / IEPINT / OEPINT, DTXFSTS, TXFE) after a test poked a register. */
void hostOtgRefresh(void);
/* Endpoint disable handshake: with hold = 1 an EPDIS write on an enabled endpoint leaves EPENA and EPDIS set (the core is not done) until hostOtgReleaseDisable(), which clears both
 * and sets EPDISD (default 0: at once). For the HAL's timeout paths of USB_EPStopXfer / USB_DeactivateEndpoint. */
void hostOtgHoldEndpointDisable(int hold);
void hostOtgReleaseDisable(void);

/* FIFO state. */
unsigned hostOtgRxWordsUsed(void);
unsigned hostOtgRxEntries(void);               /* status entries waiting (RXFLVL when > 0) */
unsigned hostOtgTxWordsUsed(unsigned fifo);    /* fifo 0-3 */
const uint32_t *hostOtgTxWords(unsigned fifo); /* the words in the TxFIFO (first hostOtgTxWordsUsed) */
void hostOtgRxFill(uint32_t value, unsigned words); /* garbage data words into the receive FIFO (set-up of an overflow) */

/* Counters. */
uint32_t hostOtgReservedAccesses(void);
uint32_t hostOtgReservedOffset(unsigned i);   /* register offset of the i-th reserved access (first 16), 0xFFFFFFFF if none */
int hostOtgReservedWasWrite(unsigned i);
/* Accesses to registers of the OTG_HS core that the OTG_FS core does not have and that the generic HAL touches: GSNPSID 0x40 (read by the HAL to pick the core version path; reads 0 here), GLPMCFG 0x54, DIEPTXF4-15 0x110-0x13C, DEACHINT 0x838 / DEACHMSK 0x83C,
 * DINEP1MSK 0x844, DOUTEP1MSK 0x884, DIEPDMA / DOEPDMA (+0x14 of an endpoint block). Counted apart from hostOtgReservedAccesses, read 0, writes ignored. */
uint32_t hostOtgHsOnlyAccesses(void);
uint32_t hostOtgHsOnlyOffset(unsigned i);
int hostOtgHsOnlyWasWrite(unsigned i);
uint32_t hostOtgFifoOverflows(void);
uint32_t hostOtgFifoUnderflows(void);
uint32_t hostOtgRxRejects(void);
/* OUT packets of a non-control endpoint that repeated the previous DATA PID of the armed transfer: ACKed and discarded (no FIFO entry, no packet count change). */
uint32_t hostOtgDuplicatePackets(void);   /* OUT packets refused because the receive FIFO had no room */
void hostOtgStrictFifo(int strict); /* 1: an overflow / underflow is also a bus fault (default) */

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_OTG_H */
