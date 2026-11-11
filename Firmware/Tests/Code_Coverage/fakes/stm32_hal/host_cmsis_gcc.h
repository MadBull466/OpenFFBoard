/*
 * host_cmsis_gcc.h (P12 real ST HAL tests, ffb_hal_tests / ffb_hal_tests_disco only)
 *
 * Host replacement for the CMSIS compiler header Drivers/CMSIS/Include/cmsis_gcc.h. It is force-included
 * (-include) into every HAL source and HAL test file. It defines the guard of the real header (__CMSIS_GCC_H), so
 * when core_cm4.h includes cmsis_compiler.h -> cmsis_gcc.h, the real header (ARM inline assembly that the i386
 * assembler rejects: isb, dsb, wfi, ldrex, ...) is skipped and the definitions below are used. No vendor file is
 * changed. The plain compiler macros are the same as in cmsis_gcc.h; only the CPU instructions are replaced.
 *
 * MODELS
 *  - Compiler macros (__ASM, __INLINE, __STATIC_INLINE, __STATIC_FORCEINLINE, __NO_RETURN, __USED, __WEAK, __PACKED*,
 *    __UNALIGNED_UINT*_READ / _WRITE, __ALIGNED, __RESTRICT, __COMPILER_BARRIER): same definitions as cmsis_gcc.h
 *    (CMSIS 5, Drivers/CMSIS/Include/cmsis_gcc.h lines 40-117).
 *  - __DSB / __ISB / __DMB: a compiler memory barrier plus a counter (hostCmsisBarrierCount). On the single host
 *    thread there is no write buffer or pipeline to drain, so the counter is all a test can observe.
 *  - __NOP: counted (hostCmsisNopCount), no effect.
 *  - __WFI / __WFE / __SEV: counted (hostCmsisWfiCount, ...), then hostCmsisWaitHook is called if set (a test can
 *    advance time or raise an event there). The core does not sleep.
 *  - __LDREXW / __LDREXH and __STREXW / __STREXH: a one-entry exclusive monitor (ARMv7-M Architecture Reference
 *    Manual A3.4: a store-exclusive succeeds, returns 0 and writes only if the monitor is open for that address;
 *    otherwise it returns 1 and does not write). LDREX opens the monitor for its address, STREX closes it.
 *    hostCmsisFailNextStrex makes the next N store-exclusives fail (an interrupt between LDREX and STREX on target),
 *    so the retry loop of the ATOMIC_* macros in stm32f4xx.h can be exercised.
 *  - __RBIT, __CLZ: exact bit operations (ARMv7-M ARM A7.7.112 RBIT, A7.7.24 CLZ; CLZ(0) = 32).
 *
 * DOES NOT MODEL
 *  - Interrupt masking (__disable_irq, PRIMASK, BASEPRI, FAULTMASK, CONTROL, MSP/PSP): the HAL does not use them
 *    (checked with grep over HAL Src/Inc, core_cm4.h, stm32f4xx.h on 2026-11-11); they are left undefined, so any
 *    new use fails to compile instead of silently doing nothing.
 *  - Real atomicity, the global monitor, bus faults, sleep modes, the FPU control registers.
 *
 * CONFIDENCE: High for the compiler macros and bit operations (same text / exact definition), Medium for the
 * exclusive monitor (single local monitor, no clearing on exception entry unless a test injects it).
 *
 * STRICT: a store-exclusive without an open monitor for that address fails (returns 1), like on target.
 */
#ifndef FFB_HOST_CMSIS_GCC_H
#define FFB_HOST_CMSIS_GCC_H

#define __CMSIS_GCC_H /* guard of Drivers/CMSIS/Include/cmsis_gcc.h: the real header is skipped */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __ASM
#define __ASM __asm
#endif
#ifndef __INLINE
#define __INLINE inline
#endif
#ifndef __STATIC_INLINE
#define __STATIC_INLINE static inline
#endif
#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE __attribute__((always_inline)) static inline
#endif
#ifndef __NO_RETURN
#define __NO_RETURN __attribute__((__noreturn__))
#endif
#ifndef __USED
#define __USED __attribute__((used))
#endif
#ifndef __WEAK
#define __WEAK __attribute__((weak))
#endif
#ifndef __PACKED
#define __PACKED __attribute__((packed, aligned(1)))
#endif
#ifndef __PACKED_STRUCT
#define __PACKED_STRUCT struct __attribute__((packed, aligned(1)))
#endif
#ifndef __PACKED_UNION
#define __PACKED_UNION union __attribute__((packed, aligned(1)))
#endif
#ifndef __UNALIGNED_UINT16_WRITE
__PACKED_STRUCT T_UINT16_WRITE { uint16_t v; };
#define __UNALIGNED_UINT16_WRITE(addr, val) (void)((((struct T_UINT16_WRITE *)(void *)(addr))->v) = (val))
#endif
#ifndef __UNALIGNED_UINT16_READ
__PACKED_STRUCT T_UINT16_READ { uint16_t v; };
#define __UNALIGNED_UINT16_READ(addr) (((const struct T_UINT16_READ *)(const void *)(addr))->v)
#endif
#ifndef __UNALIGNED_UINT32_WRITE
__PACKED_STRUCT T_UINT32_WRITE { uint32_t v; };
#define __UNALIGNED_UINT32_WRITE(addr, val) (void)((((struct T_UINT32_WRITE *)(void *)(addr))->v) = (val))
#endif
#ifndef __UNALIGNED_UINT32_READ
__PACKED_STRUCT T_UINT32_READ { uint32_t v; };
#define __UNALIGNED_UINT32_READ(addr) (((const struct T_UINT32_READ *)(const void *)(addr))->v)
#endif
#ifndef __ALIGNED
#define __ALIGNED(x) __attribute__((aligned(x)))
#endif
#ifndef __RESTRICT
#define __RESTRICT __restrict
#endif
#ifndef __COMPILER_BARRIER
#define __COMPILER_BARRIER() __asm volatile("" ::: "memory")
#endif

/* Observation counters and hooks, defined in host_cmsis.c, reset by hostCmsisReset(). */
extern volatile uint32_t hostCmsisBarrierCount; /* __DSB + __ISB + __DMB */
extern volatile uint32_t hostCmsisNopCount;
extern volatile uint32_t hostCmsisWfiCount;
extern volatile uint32_t hostCmsisWfeCount;
extern volatile uint32_t hostCmsisSevCount;
extern volatile uint32_t hostCmsisFailNextStrex; /* number of following store-exclusives that fail */
extern volatile uint32_t hostCmsisStrexFailures; /* store-exclusives that returned 1 */
extern void (*volatile hostCmsisWaitHook)(void); /* called after __WFI / __WFE when set */
void hostCmsisReset(void);
void hostCmsisWaited(void);
void hostCmsisMonitorOpen(volatile void *addr);
uint32_t hostCmsisMonitorClaim(volatile void *addr); /* 0 = store may proceed, 1 = store-exclusive fails */

__STATIC_FORCEINLINE void __NOP(void) { hostCmsisNopCount++; }
__STATIC_FORCEINLINE void __DSB(void) { __COMPILER_BARRIER(); hostCmsisBarrierCount++; }
__STATIC_FORCEINLINE void __ISB(void) { __COMPILER_BARRIER(); hostCmsisBarrierCount++; }
__STATIC_FORCEINLINE void __DMB(void) { __COMPILER_BARRIER(); hostCmsisBarrierCount++; }
#define __WFI() (hostCmsisWfiCount++, hostCmsisWaited())
#define __WFE() (hostCmsisWfeCount++, hostCmsisWaited())
#define __SEV() ((void)hostCmsisSevCount++)

__STATIC_FORCEINLINE uint32_t __LDREXW(volatile uint32_t *addr) {
	hostCmsisMonitorOpen(addr);
	return *addr;
}
__STATIC_FORCEINLINE uint16_t __LDREXH(volatile uint16_t *addr) {
	hostCmsisMonitorOpen(addr);
	return *addr;
}
__STATIC_FORCEINLINE uint32_t __STREXW(uint32_t value, volatile uint32_t *addr) {
	if (hostCmsisMonitorClaim(addr) != 0U) return 1U;
	*addr = value;
	return 0U;
}
__STATIC_FORCEINLINE uint32_t __STREXH(uint16_t value, volatile uint16_t *addr) {
	if (hostCmsisMonitorClaim(addr) != 0U) return 1U;
	*addr = value;
	return 0U;
}

__STATIC_FORCEINLINE uint32_t __RBIT(uint32_t value) {
	uint32_t result = 0U;
	for (int i = 0; i < 32; ++i) {
		result = (result << 1) | (value & 1U);
		value >>= 1;
	}
	return result;
}
__STATIC_FORCEINLINE uint8_t __CLZ(uint32_t value) {
	if (value == 0U) return 32U;
	return (uint8_t)__builtin_clz(value);
}

#ifdef __cplusplus
}
#endif

#endif /* FFB_HOST_CMSIS_GCC_H */
