/*
 * HalRegisterSpace.h (P12 real ST HAL tests, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Maps host RAM at the real STM32F407 register addresses, so the unmodified HAL can dereference its fixed
 * peripheral pointers (GPIOA, RCC, SCB, ...) in the 32-bit (-m32) host process. Same technique as the flash mapping
 * in platform/src/host_flash.cpp (mmap with MAP_FIXED_NOREPLACE: never replaces an existing mapping).
 *
 * MODELS (address ranges; source RM0090 Rev 22 section 2.3 Table 1 "STM32F4xx register boundary addresses",
 * the system control space from CMSIS Drivers/CMSIS/Include/core_cm4.h:1550-1557, SCS_BASE 0xE000E000; the Cortex-M4
 * programming manual PM0214 is not in the specs folder)
 *  - 0x40000000-0x4007FFFF: APB1 (TIM2 ... DAC), APB2 (TIM1 ... SPI4... ), AHB1 (GPIOA 0x40020000, RCC 0x40023800,
 *    FLASH interface 0x40023C00, DMA1 / DMA2 0x40026000 / 0x40026400, USB OTG HS 0x40040000).
 *  - 0x50000000-0x50060FFF: AHB2 (USB OTG FS 0x50000000-0x5003FFFF ... RNG 0x50060800-0x50060BFF).
 *  - 0xE000E000-0xE000EFFF: Cortex-M4 system control space (SysTick 0xE000E010, NVIC 0xE000E100, SCB 0xE000ED00).
 *  - 0xE0042000-0xE0042FFF: DBGMCU (RM0090 section 38.16).
 *  - 0x1FFF7000-0x1FFF7FFF: unique device id 0x1FFF7A10 and flash size 0x1FFF7A22 (RM0090 sections 39.1, 39.2).
 *  - 0x1FFFC000-0x1FFFCFFF: option bytes (RM0090 section 3.7, Table 5: 0x1FFFC000-0x1FFFC00F).
 *  hostRegsReset() fills every range with zeros. T0 gives no register its RM0090 reset value yet and no register any
 *  hardware behaviour: the mapped RAM is plain memory (a write is stored, a read returns the last write). Peripheral
 *  behaviour (write-1-to-clear, read-only bits, flags set by hardware) is added per peripheral in later stages.
 *
 * DOES NOT MODEL
 *  - Any peripheral behaviour (see above). A test that checks a register value after a HAL call therefore checks
 *    the HAL's write request, not the hardware effect.
 *  - Flash memory (0x08000000), SRAM, FSMC (0xA0000000), bit-band aliases, CCM RAM.
 *  - Accesses outside the mapped ranges: they crash the process (SIGSEGV), which fails the ctest run.
 *
 * CONFIDENCE: High for the addresses (RM0090 Table 1); not applicable for behaviour (none modelled yet).
 *
 * STRICT: mapping failure (address already in use) fails the test through hostRegsMapped() returning 0.
 */
#ifndef FFB_HAL_REGISTER_SPACE_H
#define FFB_HAL_REGISTER_SPACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maps every range once per process. Returns 1 when all ranges are mapped at their exact addresses, else 0. */
int hostRegsMap(void);
/* 1 if hostRegsMap() succeeded. */
int hostRegsMapped(void);
/* Zeroes every mapped range (does nothing if not mapped). */
void hostRegsReset(void);
/* Text of the first mapping failure, or "" (for test messages). */
const char *hostRegsError(void);
/* 1 if [addr, addr + size) lies inside one mapped range. */
int hostRegsContains(uintptr_t addr, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* FFB_HAL_REGISTER_SPACE_H */
