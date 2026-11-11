/*
 * HalPeripherals.h (P12 real ST HAL tests, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Register models on the trapping bus of HalRegisterSpace.h (P12 T1 set). Every model is strict: an offset it does
 * not know, or a write to a read-only register, is a fault that fails the test (HalFixture). Sources: RM0090 Rev 22.
 *
 * GPIOA..GPIOI (0x40020000 + 0x400 * port; RM0090 Table 1, chapter 8). Confidence: High for the register semantics
 * below, Medium for the pin level model.
 *  - Reset values (8.4.1-8.4.4): MODER A 0xA8000000, B 0x00000280; OSPEEDR A 0x0C000000, B 0x000000C0; PUPDR
 *    A 0x64000000, B 0x00000100; all other registers and ports 0.
 *  - OTYPER and ODR keep bits 15:0 (bits 31:16 reserved, read 0).
 *  - IDR (8.4.5) is read-only (a write is a fault). Its value is the pin level: input / alternate function: the
 *    external level set with hostGpioSetInput; a floating input reads 1 with pull-up, 0 with pull-down, 0 without
 *    (counted in hostGpioFloatingReads; the real level is undefined). Output push-pull: the ODR bit (8.3.10 "A read
 *    access to the input data register gets the I/O state"). Output open-drain: 0 if ODR is 0, else the external
 *    level (floating: pull as above). Analog: 0 (8.3.12).
 *  - BSRR (8.3.5, 8.4.7): write-only; writing 1 to BSy sets ODRy, to BRy resets it, BSy wins if both; reads 0.
 *  - LCKR (8.3.6, 8.4.8): lock key sequence WR LCKK=1+LCK, WR LCKK=0+LCK, WR LCKK=1+LCK (same LCK[15:0]), then
 *    RD LCKR; the lock becomes active at that read (the sequence's mandatory read; the second read is optional).
 *    Any other LCKR write during the sequence aborts it. LCK[15:0] are writable while LCKK is 0. Once locked,
 *    LCKK reads 1, LCKR writes are ignored, and the MODER / OTYPER / OSPEEDR / PUPDR / AFRL / AFRH bits of the
 *    locked pins keep their value on writes, until hostRegsReset (MCU reset).
 *  - Accesses while the port's RCC_AHB1ENR.GPIOxEN is 0 are counted (hostGpioClockOffAccesses), nothing else:
 *    RM0090 does not define the behaviour.
 *  Not modelled: access width (LCKR word-only rule), speed, electrical effects, alternate-function outputs.
 *
 * EXTI (0x40013C00; RM0090 chapter 12). Confidence: High for the registers, Medium for edge detection.
 *  - IMR, EMR, RTSR, FTSR keep bits 22:0 (31:23 reserved, read 0). Reset values 0.
 *  - SWIER (12.3.5): writing 1 to a bit that is 0 sets the PR bit if the IMR bit is set (and counts an event if the
 *    EMR bit is set); the SWIER bit stays 1 until the PR bit is cleared. Writing 0 has no effect.
 *  - PR (12.3.6): rc_w1, writing 1 clears the bit and the SWIER bit.
 *  - Lines 0-15 follow the GPIO pin selected by SYSCFG_EXTICRx (9.2.3-9.2.6, figure 42). Lines 16-22 follow
 *    hostExtiSetLine. A selected edge (RTSR / FTSR) sets PR if the IMR bit is set (12.2.4; as for SWIER) and counts
 *    an event if the EMR bit is set. Changing the EXTICR selection resamples the line without an edge.
 *  Not modelled: the NVIC side (no interrupt is raised; tests call the IRQ handler), wake-up from WFE.
 *
 * SYSCFG (0x40013800; RM0090 9.2). MEMRMP bits 1:0 (reset 0: BOOT pins select main flash), PMC bit 23, EXTICR1-4
 * bits 15:0, CMPCR: CMP_PD (bit 0) rw, READY (bit 8) read-only. READY follows CMP_PD at once (RM0090 gives no
 * timing; Low confidence). Other offsets: fault.
 *
 * RCC (0x40023800; RM0090 7.3, T1 subset). Storage only: every register of 7.3.1-7.3.23 at its offset with its
 * reset value (RCC_CR 0x00000083: the undefined calibration bits are 0). No oscillator, PLL, ready-flag, reset or
 * reserved-bit behaviour yet (stage T2). Other offsets: fault.
 *
 * FLASH interface (0x40023C00; RM0090 3.9, T1 subset). FLASH_ACR (3.9.1) stored as written, reset 0. KEYR, OPTKEYR,
 * SR, CR, OPTCR: fault until stage T7.
 *
 * DBGMCU (0xE0042000; RM0090 38.6.1, 38.16.3-38.16.5). IDCODE read-only, value set by hostDbgSetIdcode (default
 * 0x10070413: DEV_ID 0x413 of STM32F405xx/07xx, REV_ID 0x1007 = revision 2, both values listed in 38.6.1); CR,
 * APB1_FZ, APB2_FZ stored as written. Other offsets: fault.
 *
 * Unique device id and flash size (RM0090 39.1, 39.2): 0x1FFF7A10, 0x1FFF7A14, 0x1FFF7A18 (UID words) and the
 * 16-bit flash size at 0x1FFF7A22 (the word at 0x1FFF7A20 reads size << 16), read-only, values set by hostUidSet /
 * hostFlashSizeSet (defaults 0x12345678, 0x9ABCDEF0, 0x0F1E2D3C, 1024 KiB). Other addresses of that page: fault.
 *
 * The system control space (NVIC, SCB, SysTick, MPU) is modelled in HalCorePeripherals.h; hostPeriphRegisterAll and
 * hostPeriphResetState include it. Everything else in the mapped ranges (option bytes, other peripherals) has no model
 * yet: any access is a fault.
 */
#ifndef FFB_HAL_PERIPHERALS_H
#define FFB_HAL_PERIPHERALS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { HOST_GPIO_PORTS = 9 }; /* A..I on STM32F407 */
typedef enum { HOST_PIN_FLOAT = 0, HOST_PIN_LOW = 1, HOST_PIN_HIGH = 2 } HostPinLevel;

/* Registers every T1 model with the bus (idempotent). */
void hostPeriphRegisterAll(void);
/* Resets the model state that is not stored in registers (pin levels, lock state, counters, ids). Called by
 * the fixture before hostRegsReset. */
void hostPeriphResetState(void);

/* GPIO: port 0 = A ... 8 = I, pin 0..15. */
void hostGpioSetInput(unsigned port, unsigned pin, HostPinLevel level);
int hostGpioPinLevel(unsigned port, unsigned pin); /* level the pin has now (0 / 1), as IDR would read it */
int hostGpioLocked(unsigned port, unsigned pin);
uint32_t hostGpioClockOffAccesses(void);
uint32_t hostGpioFloatingReads(void);

/* EXTI internal lines 16..22 (PVD, RTC alarm, USB OTG FS wake-up, Ethernet wake-up, USB OTG HS wake-up, RTC
 * tamper / time stamp, RTC wake-up: RM0090 12.2.5). */
void hostExtiSetLine(unsigned line, int level);
uint32_t hostExtiEventCount(void);

/* 1: writes to FLASH_ACR.LATENCY[2:0] keep the old value (the latency never reads back as written). */
void hostFlashLatencyStuck(int stuck);
void hostDbgSetIdcode(uint32_t idcode);
void hostUidSet(uint32_t w0, uint32_t w1, uint32_t w2);
void hostFlashSizeSet(uint16_t kib);

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_PERIPHERALS_H */
