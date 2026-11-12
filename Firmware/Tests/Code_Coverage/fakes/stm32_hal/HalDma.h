/*
 * HalDma.h (P12 T3, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Register model of DMA1 (0x40026000) and DMA2 (0x40026400) of the STM32F405/407 on the trapping bus of HalRegisterSpace.h.
 * Source: RM0090 Rev 22 chapter 10 (10.3 functional description, 10.4 interrupts, 10.5 registers). Confidence: High for the
 * registers, reset values, write protection, flag layout and the FIFO / burst rule (Table 49); Medium for the timing of EN
 * clearing (RM0090: "after the current data transfer"; here a configurable number of HAL ticks); Low for anything about
 * the transfer engine: no data is moved, the tests stand in for it with the hooks below.
 *
 * Registers: LISR (0x00), HISR (0x04) read-only, LIFCR (0x08), HIFCR (0x0C) write 1 clears the flag and read 0; per stream x
 * (0..7) at 0x10 + 0x18 * x: SxCR (+0x00), SxNDTR (+0x04), SxPAR (+0x08), SxM0AR (+0x0C), SxM1AR (+0x10), SxFCR (+0x14).
 * Reset values 0 except SxFCR 0x00000021. Reserved bits read 0 and ignore writes. SxFCR.FS reads 100 (FIFO empty).
 * Flag layout (RM0090 10.5.1-10.5.4): stream 0 / 4 at bit offset 0, 1 / 5 at 6, 2 / 6 at 16, 3 / 7 at 22; inside the group
 * FEIF bit 0, DMEIF bit 2, TEIF bit 3, HTIF bit 4, TCIF bit 5.
 *
 * Write protection (RM0090 10.5.5-10.5.10): while EN reads 1, a write to a protected field of SxCR (everything except TCIE, HTIE,
 * TEIE, DMEIE, EN), to SxNDTR, SxPAR, or to SxFCR except FEIE is ignored and counted (hostDmaProtectedWrites); M0AR is writable
 * with EN = 1 only in double buffer mode while CT = 1, M1AR only while CT = 0; a write to the memory register of the current
 * target in double buffer mode sets TEIF (10.3.18) and is ignored; other writes with EN = 1 are ignored and counted.
 *
 * EN: writing 1 starts the stream: the forced bits of 10.5.5 / 10.5.10 apply (direct mode: MBURST = PBURST = PINCOS = 0 and
 * MSIZE = PSIZE; DBM = 1 forces CIRC = 1; PFCTRL = 1 forces CIRC = 0; memory-to-memory forces PFCTRL = 0 and DMDIS = 1); if the
 * FIFO is in use (DMDIS = 1) and MBURST != 0 and the threshold does not hold a whole number of memory bursts (Table 49:
 * threshold bytes % (beats * MSIZE bytes) != 0, or beats * size > 16) FEIF is set and EN is cleared at once (10.3.18).
 * Writing 0 requests the stop: EN clears after hostDmaSetDisableDelay HAL ticks (default 0), or never while
 * hostDmaStickEnabled is set (a polling loop times out). The hardware also clears EN at the end of a normal-mode transfer and
 * when TEIF is raised. A software stop ends with TCIF set (RM0090 10.3.13, 10.3.14: "the transfer complete interrupt flag is set in order to
 * indicate the end of transfer"); the flush of the FIFO is not modelled.
 *
 * Hooks (the transfer engine): hostDmaRaise (set flags; TEIF also clears EN), hostDmaComplete (end of the transfer: TCIF, NDT 0 or
 * reloaded in circular mode, EN cleared in normal mode), hostDmaStepItems (n data items: NDT decreases, HTIF at half, TCIF at the
 * end, CT toggles in double buffer mode). Interrupt requests are not delivered (no NVIC side).
 * Not modelled: request timing, priorities, FIFO fill level, bus errors, the data itself.
 */
#ifndef FFB_HAL_DMA_H
#define FFB_HAL_DMA_H

#include <stdint.h>

#include "HalRegisterSpace.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { HOST_DMA_CONTROLLERS = 2, HOST_DMA_STREAMS = 8 };

/* Flag masks for hostDmaRaise (bit positions inside a stream's 6 bit group of LISR / HISR). */
enum { HOST_DMA_FEIF = 1u << 0, HOST_DMA_DMEIF = 1u << 2, HOST_DMA_TEIF = 1u << 3, HOST_DMA_HTIF = 1u << 4, HOST_DMA_TCIF = 1u << 5 };

void hostDmaRegister(void);
void hostDmaResetState(void);

void hostDmaSetDisableDelay(unsigned ticks);
void hostDmaStickEnabled(unsigned controller, unsigned stream, int stick);
uint32_t hostDmaProtectedWrites(unsigned controller, unsigned stream);

/* Completes the stream (as hostDmaComplete) when its status register has been read `reads` times: for polling loops that
 * never call HAL_GetTick (HAL_MAX_DELAY). */
void hostDmaCompleteAfterReads(unsigned controller, unsigned stream, uint32_t reads);
void hostDmaRaise(unsigned controller, unsigned stream, uint32_t flagMask);
void hostDmaComplete(unsigned controller, unsigned stream);
void hostDmaStepItems(unsigned controller, unsigned stream, uint32_t items);

/* Flags of one stream as a mask of HOST_DMA_* (reads LISR / HISR raw). */
uint32_t hostDmaFlags(unsigned controller, unsigned stream);

/* Model hooks of the register block (HalPeripherals registration). */
uintptr_t hostDmaBase(unsigned controller);

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_DMA_H */
