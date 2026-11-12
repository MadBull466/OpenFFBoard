/*
 * HalI2c.h (P12 T4c, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Register and bus model of I2C1 (0x40005400), I2C2 (0x40005800) and I2C3 (0x40005C00) of the STM32F405/407, on the trapping bus of
 * HalRegisterSpace.h. Source: RM0090 Rev 22 chapter 27 (27.3 functional description incl. Figures 241-244 and the event list EV1-EV9,
 * 27.4 Table 124 interrupts, 27.6 registers). The F407 has no I2C_FLTR (RM0090 27.6.10: "STM32F42xxx and STM32F43xxx only"), so offset
 * 0x24 is reserved here and a access to it is a fault; the HAL's I2C_Ex file is compiled out on this target (I2C_FLTR_ANOFF is not
 * defined). Confidence: High for the registers, reset values, the flag set / clear sequences and the controller / target sequencing of
 * the Figures; Medium for byte timing (a byte takes `byteTicks` HAL ticks, default 0 = at once; the CCR / TRISE / FREQ values are stored but do not set
 * the timing); Low for everything on the wire (the peer is a test hook) and for the SWRST / PE = 0 corner cases the manual leaves open.
 *
 * Registers (16-bit registers, word access; RM0090 27.6): CR1 0x00 (mask 0xBFFB), CR2 0x04 (0x1F3F), OAR1 0x08 (0xC3FF), OAR2 0x0C (0x00FF),
 * DR 0x10 (0x00FF), SR1 0x14 (read-only except rc_w0 on SMBALERT, TIMEOUT, PECERR, OVR, AF, ARLO, BERR), SR2 0x18 (read-only, a write is a
 * fault), CCR 0x1C (0xCFFF), TRISE 0x20 (0x003F, reset 0x0002).
 *
 * Controller (Figures 243 / 244): CR1.START with PE = 1 and the bus free (BUSY = 0) sets BUSY, MSL, SB and clears START; with MSL = 1 it is a
 * repeated start after the current byte. SB clears with "read SR1, then write DR" (the address). The address byte goes on the bus (one
 * log entry); a matching peer device acks it: ADDR = 1, TRA = !R/W, TXE = 1 for a transmitter; no match: AF = 1 and nothing else (software must
 * stop). 10-bit: a header 11110xx0 sets ADD10 (clears with "read SR1, then write DR" of the second byte); the second byte then sets ADDR
 * (transmitter); a repeated start and the header 11110xx1 sets ADDR with TRA = 0. ADDR clears with "read SR1, then read SR2"; in a receiver
 * that starts the reception. Transmitter: the first DR write goes to the shift register at once (TXE stays 1, RM0090 27.6.6 TxE note); a further
 * write waits in DR (TXE = 0) until the byte on the bus ended, then moves to the shift register (TXE = 1). A byte ends with the peer's ack
 * (nack: AF = 1, no TXE / BTF, nothing more is sent). When the shift register and DR are empty after a byte: BTF = 1 (clock stretched) until DR is
 * written. Receiver: after a byte RXNE = 1 (DR read clears it); a byte received while RXNE = 1 stays in the shift register with BTF = 1 and the
 * bus is stretched until DR is read (RM0090 27.3.3); the byte is acked by ACK, with POS = 1 by the value ACK had when the previous byte was
 * received (the first byte is acked) (27.6.1 POS); a nack ends the transfer. Model decision (the manual leaves it open): a byte waiting in DR when START or STOP is pending is dropped when the current byte ends (the controller then restarts / stops). CR1.STOP: stop after the current byte / start condition: BUSY = 0,
 * MSL = 0, TRA = 0, STOP cleared, TXE and BTF cleared. The peer device sends its queue, then 0xFF. A transfer stops while `stuck`.
 * Target (Figures 241 / 242): the hostI2cExt* functions are an external controller. A matching address (OAR1 7 or 10 bit, OAR2 with ENDUAL,
 * general call with ENGC) sets ADDR, BUSY, TRA, DUALF / GENCALL, and is acked when ACK = 1. Bytes can flow when ADDR is cleared (not
 * with NOSTRETCH = 1). Receiver: RXNE as above; a byte with RXNE = 1 sets BTF and stretches (NOSTRETCH = 0), or sets OVR and is lost
 * (NOSTRETCH = 1). Transmitter: DR write with an empty shift register goes to the shift register (TXE stays 1); a byte is taken from the shift register;
 * with nothing to send: BTF = 1 and stretched, or OVR = 1 and the same byte again (NOSTRETCH = 1). A nack from the controller sets AF (no STOPF);
 * a stop sets STOPF (cleared by "read SR1, then write CR1"), clears BUSY, TRA, DUALF, GENCALL.
 * Interrupt requests are not delivered: hostI2cEventPending / hostI2cErrorPending give the request lines of Table 124 (event: ITEVTEN and
 * (SB, ADDR, ADD10, STOPF, BTF, or ITBUFEN and (TXE or RXNE)); error: ITERREN and (BERR, ARLO, AF, OVR, PECERR, TIMEOUT, SMBALERT)) and the
 * tests call the HAL IRQ handlers (the support file has a helper that does so while a line is pending). DMA requests are not generated: a test
 * moves the bytes with plain DR accesses.
 *
 * Not modelled: SCL / SDA wire timing, clock stretching as a state of the line (it is the "stalled" result and the BTF flag), arbitration other
 * than the hostI2cArbitrationLost hook, PEC calculation (CR1.PEC / ENPEC are stored only), SMBus (SMBUS, SMBTYPE, ENARP, ALERT are stored; TIMEOUT,
 * SMBALERT only through hostI2cRaiseSr1), the noise filters, DUTY / F/S timing, wake-up from Stop. SWRST (not specified by the manual beyond "reset
 * state"): resets the state machine, SR1, SR2 (BUSY stays when the bus is held) and CR1 except SWRST; CR2 / OAR / CCR / TRISE keep their values.
 */
#ifndef FFB_HAL_I2C_H
#define FFB_HAL_I2C_H

#include <stdint.h>

#include "HalRegisterSpace.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { HOST_I2C_COUNT = 3 }; /* 0 = I2C1, 1 = I2C2, 2 = I2C3 */

/* Bus log entry types. */
enum { HOST_I2C_EV_START = 1, HOST_I2C_EV_RESTART, HOST_I2C_EV_STOP, HOST_I2C_EV_BYTE_OUT, HOST_I2C_EV_BYTE_IN };
typedef struct HostI2cEvent {
	uint8_t type;  /* HOST_I2C_EV_* */
	uint8_t value; /* the byte, for BYTE_OUT (controller to target) and BYTE_IN (target to controller) */
	uint8_t ack;   /* 1 = acknowledged, 0 = nack (BYTE_* only) */
} HostI2cEvent;

void hostI2cRegister(void);
void hostI2cResetState(void);

/* Ticks a byte (address or data) takes on the bus when the HAL is the controller (default 0 = at once). */
void hostI2cSetByteTicks(unsigned ticks);
/* Every n-th read of SR1 counts as one tick for the model (0 = off): for loops that read no tick. */
void hostI2cSetPollReads(unsigned n);
/* 1: the bus never progresses (SCL held low): no byte, address or stop completes. */
void hostI2cStuck(unsigned i2c, int stuck);
/* 1: another device holds the bus (BUSY = 1, a START waits); 0: released. */
void hostI2cBusBusy(unsigned i2c, int busy);

/* Peer devices on the bus when the HAL is the controller (at most 4 per I2C). Returns the index, or -1. */
int hostI2cDeviceAdd(unsigned i2c, uint16_t address, int tenBit); /* address: 7 bit (not shifted) or 10 bit */
void hostI2cDeviceNackAddress(unsigned i2c, int device, int nack);
/* The device nacks the data byte with this index (0 = first data byte of a transfer since the last START), -1 = never. */
void hostI2cDeviceNackDataAt(unsigned i2c, int device, int index);
void hostI2cDeviceQueue(unsigned i2c, int device, const uint8_t *data, unsigned count); /* bytes it sends when read */

/* Bus log (the first 512 entries since the reset or the last clear). */
unsigned hostI2cLogCount(unsigned i2c);
HostI2cEvent hostI2cLogAt(unsigned i2c, unsigned index);
void hostI2cLogClear(unsigned i2c);

/* External controller (the HAL is the target). Return values: Start / Restart: 1 acked, 0 not acked or ignored. Write / ReadByte:
 * 1 acked, 0 nack, -1 stretched (nothing happened, try again after the HAL serviced its flags), -2 not addressed / wrong direction. */
int hostI2cExtStart(unsigned i2c, uint8_t addressByte);   /* 8 bit address incl. R/W, or 11110xxR header */
int hostI2cExtRestart(unsigned i2c, uint8_t addressByte);
int hostI2cExtWrite(unsigned i2c, uint8_t byte);          /* also the second byte of a 10-bit address */
int hostI2cExtReadByte(unsigned i2c, int masterAck, uint8_t *byte);
void hostI2cExtStop(unsigned i2c);

/* Error injection (set the flag; ARLO also goes back to target mode and aborts the transfer). */
void hostI2cRaiseSr1(unsigned i2c, uint32_t sr1Mask);
void hostI2cArbitrationLost(unsigned i2c);

/* Advances the controller engine to the current time (it also advances on every SR1 / SR2 / DR access and CR1 write). */
void hostI2cPoll(unsigned i2c);

/* Interrupt request lines (see above). */
int hostI2cEventPending(unsigned i2c);
int hostI2cErrorPending(unsigned i2c);

/* Counters of what the RM0090 says should not happen or what tests want to see. */
uint32_t hostI2cConfigWhileEnabled(unsigned i2c); /* writes that change CCR or TRISE while PE = 1 (27.6.8 / 27.6.9) */
uint32_t hostI2cStrayDrWrites(unsigned i2c);      /* DR writes that start nothing (no address phase, no transmit mode) */
uint32_t hostI2cSwrstCount(unsigned i2c);         /* 0 to 1 transitions of SWRST */
uint32_t hostI2cPeOffBusy(unsigned i2c);          /* PE 1 to 0 while the HAL is the controller and the bus is busy (27.6.1 PE note) */
uint32_t hostI2cSequenceBreaks(unsigned i2c);     /* address DR write without the SR1 read, ADDR clear SR2 read without the SR1 read is not counted */

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_I2C_H */
