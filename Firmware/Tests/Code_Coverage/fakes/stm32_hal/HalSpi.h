/*
 * HalSpi.h (P12 T4a, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Register model of SPI1 (0x40013000), SPI2 (0x40003800) and SPI3 (0x40003C00) of the STM32F405/407 in SPI mode, on the trapping bus
 * of HalRegisterSpace.h. Source: RM0090 Rev 22 chapter 28 (28.3 functional description, 28.5 registers). Confidence: High for the
 * registers, reset values and flag semantics; Medium for frame timing (a frame takes a configurable number of HAL ticks, default 1;
 * the real time is the baud rate divider times the frame bits); Low for everything on the bus side (the peer is a test hook).
 *
 * Registers (word or half-word access): CR1 0x00, CR2 0x04, SR 0x08, DR 0x0C, CRCPR 0x10 (reset 0x0007), RXCRCR 0x14, TXCRCR 0x18
 * (both read 0, the CRC engine is not modelled: the target compiles with USE_SPI_CRC 0), I2SCFGR 0x1C, I2SPR 0x20 (reset 0x0002,
 * stored; I2S mode is not modelled). Reset values: CR1 0, CR2 0, SR 0x0002 (TXE), DR 0.
 * CR1 and CR2 keep their writable bits (CR2 bits 7:4 and 2:0; bit 3 and 15:8 read 0). Changing DFF, CRCEN, BR, MSTR, CPOL, CPHA,
 * LSBFIRST, RXONLY (BIDIMODE and BIDIOE are not restricted) while SPE = 1 is allowed but counted (hostSpiConfigWhileEnabled): RM0090 says "should not be changed
 * when communication is ongoing / should be written only when SPI is disabled".
 * DR: a write puts the frame (8 or 16 bit, DFF) into the Tx buffer; in master mode with SPE = 1 the frame starts at once if the
 * shift register is free (TXE stays 1), else TXE = 0 until the running frame ends. A frame lasts `frameTicks` HAL ticks (BSY = 1);
 * at its end the received word (next word of the peer queue, else the idle word, or the transmitted word in loopback mode) goes to the
 * Rx buffer (RXNE = 1); if RXNE was still 1 the new word is lost and OVR is set (RM0090 28.3.10). A read of DR returns the Rx buffer
 * and clears RXNE. OVR clears with the sequence "read DR, then read SR" (28.3.10). Master with RXONLY = 1 (or BIDIMODE = 1 and
 * BIDIOE = 0) and SPE = 1 runs frames continuously (28.3.5). Slave mode (MSTR = 0): frames are delivered with hostSpiSlaveFrame.
 * MODF is raised by hostSpiRaiseModf (MSTR and SPE cleared by hardware) and cleared by "read SR, then write CR1" (28.3.10). Disabling
 * the SPI (SPE 1 to 0) while BSY = 1 aborts the frame and is counted (hostSpiDisabledBusy; 28.3.8 gives the correct sequence).
 * Writes to SPE = 0: DR accepted into the Tx buffer (TXE = 0) and the frame starts when SPE is set (master).
 *
 * BSY: not set during 1-line receive (BIDIMODE = 1, BIDIOE = 0; RM0090 28.3.7); cleared by hostSpiRaiseModf (28.3.10).
 *
 * Not modelled: bit order / clock phase on the wire, NSS pin, CRC, TI mode frame errors (hostSpiRaiseFre sets FRE only), I2S, interrupt
 * delivery, DMA requests (the tests call the DMA / SPI IRQ handlers themselves).
 */
#ifndef FFB_HAL_SPI_H
#define FFB_HAL_SPI_H

#include <stdint.h>

#include "HalRegisterSpace.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { HOST_SPI_COUNT = 3 }; /* 0 = SPI1, 1 = SPI2, 2 = SPI3 */

void hostSpiRegister(void);
void hostSpiResetState(void);

/* Ticks per frame (>= 1, default 1). */
void hostSpiSetFrameTicks(unsigned ticks);
/* The peer queue (words returned in order as the received data of the next frames); idle word when empty (default 0). */
void hostSpiPeerPush(unsigned spi, uint16_t word);
void hostSpiPeerSetIdle(unsigned spi, uint16_t word);
/* 1: MISO is wired to MOSI, the received word is the transmitted word (the peer queue is ignored). */
void hostSpiPeerLoopback(unsigned spi, int on);
/* 1: the bus never completes a frame (BSY stays 1): a polling loop times out. */
void hostSpiPeerSilent(unsigned spi, int silent);

/* Everything transmitted since the reset, in order. */
unsigned hostSpiTxCount(unsigned spi);
uint16_t hostSpiTxAt(unsigned spi, unsigned index);
void hostSpiTxClear(unsigned spi);

/* Slave mode: the master clocks one frame; `word` is what it sent. Returns the word the slave sent (the Tx buffer, else 0). */
uint16_t hostSpiSlaveFrame(unsigned spi, uint16_t word);
void hostSpiRaiseModf(unsigned spi);
void hostSpiRaiseFre(unsigned spi);

uint32_t hostSpiConfigWhileEnabled(unsigned spi);
uint32_t hostSpiDisabledBusy(unsigned spi);

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_SPI_H */
