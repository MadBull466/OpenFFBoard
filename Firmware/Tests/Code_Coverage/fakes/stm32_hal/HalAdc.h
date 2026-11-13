/*
 * HalAdc.h (P12 T6b, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Register model of ADC1, ADC2, ADC3 and the common ADC registers of the STM32F405/407 on the trapping bus of HalRegisterSpace.h.
 * Source: RM0090 Rev 22 chapter 13 (13.3 functional description, 13.8 data management, 13.13 registers, Tables 71-73).
 * Confidence: High for the register layouts, write masks, reset values and flag clear rules; Medium for the conversion engine (it converts when the test
 * says so, one single channel conversion per step, no time, no sampling time, no ADC clock); Low for the multi ADC mode (not modelled), the
 * temperature sensor / VBAT / VREFINT channels (only the analog value the test gives), and the sign of an injected result below the offset.
 *
 * `adc` is 1..3. Bases: ADC1 0x40012000, ADC2 0x40012100, ADC3 0x40012200, common registers 0x40012300 (RM0090 Table 71). A register of a window
 * outside 0x00..0x4C (ADCx) or 0x00..0x08 (common) is a fault (strict).
 * Registers of one ADC (word access): SR 0x00 (rc_w0, mask 0x3F: AWD, EOC, JEOC, JSTRT, STRT, OVR), CR1 0x04 (0x07C0FFFF), CR2 0x08 (0x7F7F0F03, SWSTART and
 *   JSWSTART read 0), SMPR1 0x0C (0x07FFFFFF), SMPR2 0x10 (0x3FFFFFFF), JOFR1-4 0x14-0x20 (0xFFF), HTR 0x24 (reset 0xFFF), LTR 0x28 (0xFFF),
 *   SQR1 0x2C (0x00FFFFFF), SQR2 0x30, SQR3 0x34 (0x3FFFFFFF), JSQR 0x38 (0x003FFFFF), JDR1-4 0x3C-0x48 (read only, 16 bit), DR 0x4C (read only, 16 bit).
 * Common: CSR 0x00 (read only: a copy of the flags of the three ADCs, bits AWD / EOC / JEOC / JSTRT / STRT / OVR at 0 / 8 / 16 for ADC1 / 2 / 3), CCR 0x04
 *   (0x00C3EF1F: MULTI, DELAY, DDS, DMA, ADCPRE, VBATE, TSVREFE), CDR 0x08 (read only; hostAdcSetCommonData gives its value).
 *
 * Behaviour: ADON = 0 stops every conversion. SWSTART (CR2 bit 30) launches a regular sequence when ADON = 1, EXTEN = 00 and no regular conversion runs; it sets STRT and
 * reads 0. JSWSTART (bit 22) the same for the injected group with JEXTEN = 00 (sets JSTRT). hostAdcExternalTrigger plays an external trigger event when EXTEN / JEXTEN != 00.
 * hostAdcConvert performs single channel conversions; an injected conversion in progress has priority over the regular one (RM0090 13.3.9: the regular sequence resumes
 * afterwards). Regular: with SCAN = 0 only SQ1 is converted, with SCAN = 1 the L + 1 ranks of SQR3 (1-6), SQR2 (7-12), SQR1 (13-16); the result goes to DR aligned by ALIGN
 * (right: the RES bit value; left: MSB at bit 15, D5 at bit 7 for 6 bit data: RM0090 Figures 49 / 50) after being reduced to RES (12 / 10 / 8 / 6 bit: the 12 bit analog value >> 2 * RES); EOC is set after every
 * conversion when EOCS = 1 and after the last rank only when EOCS = 0; reading DR clears EOC. Overrun detection is active when DMA = 1 or EOCS = 1: a conversion that
 * finishes while EOC = 1 sets OVR (RM0090 13.8.1), the regular conversion is aborted and regular triggers are ignored until software clears OVR. With DMA = 1 each regular
 * conversion counts one DMA request (hostAdcDmaRequests, not counted while OVR = 1); the tests play the DMA stream themselves with hostDmaRaise. CONT = 1 restarts the sequence,
 * DISCEN converts DISCNUM + 1 ranks per trigger. JAUTO starts the injected group after the regular sequence. Injected: JL + 1 conversions from JSQ(4 - JL) to JSQ4
 * (RM0090 13.13.12 note), the result of the n-th injected conversion (n = 1 .. JL + 1, conversion order) goes to JDRn minus JOFRn (16 bit two's complement; left aligned: sign at bit 15 and the MSB at bit 14, 6 bit data D5 at bit 6; the manual is silent about the numbering for JL < 3), JEOC after the last, JDISCEN converts one channel per trigger. Analog watchdog:
 * with AWDEN (regular) / JAWDEN (injected) each result is compared with LTR / HTR (the 12 bit equivalent) and AWD is set when outside, on every channel or only on AWDCH with AWDSGL.
 * hostAdcIrqPending(adc) = (EOC & EOCIE) | (JEOC & JEOCIE) | (AWD & AWDIE) | (OVR & OVRIE).
 *
 * Not modelled: the ADC clock and conversion time, the sampling times (stored), the stabilization time (the HAL waits with a counter loop, no register is read), triggers by timers (the test
 * calls hostAdcExternalTrigger), the multi ADC mode (MULTI is stored, the master does not drive the slaves), the temperature sensor / VBAT / VREFINT hardware, DMA request handshakes.
 */
#ifndef FFB_HAL_ADC_H
#define FFB_HAL_ADC_H

#include <stdint.h>

#include "HalRegisterSpace.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { HOST_ADC_COUNT = 3 };

void hostAdcRegister(void);
void hostAdcResetState(void);

/* The analog input of channel ch (0..18) of ADC adc: a 12 bit value (what a conversion at full resolution gives). */
void hostAdcSetAnalog(unsigned adc, unsigned ch, uint16_t value12);
/* Performs up to n single channel conversions (injected first). Returns the number performed. */
unsigned hostAdcConvert(unsigned adc, unsigned n);
/* An external trigger event for the regular (injected = 0) or the injected group. Returns 1 when it launched a sequence (enabled by EXTEN / JEXTEN, ADON = 1, nothing running). */
int hostAdcExternalTrigger(unsigned adc, int injected);
/* After n reads of SR with a conversion pending, one conversion step runs (a poll loop sees the conversion complete). 0 = off. */
void hostAdcSetPollReads(unsigned adc, unsigned n);
/* (EOC & EOCIE) | (JEOC & JEOCIE) | (AWD & AWDIE) | (OVR & OVRIE) as 0 / non-zero. */
uint32_t hostAdcIrqPending(unsigned adc);
/* DMA requests generated by regular conversions with CR2.DMA = 1 since the last clear. */
uint32_t hostAdcDmaRequests(unsigned adc);
void hostAdcClearDmaRequests(unsigned adc);
/* Number of conversions performed (regular and injected) since reset. */
uint32_t hostAdcConversions(unsigned adc);
/* The value ADC_CDR reads. */
void hostAdcSetCommonData(uint32_t cdr);
/* True while a regular / injected sequence is running (launched and not finished). */
int hostAdcRegularRunning(unsigned adc);
int hostAdcInjectedRunning(unsigned adc);

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_ADC_H */
