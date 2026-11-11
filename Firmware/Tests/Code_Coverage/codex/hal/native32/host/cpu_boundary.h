#ifndef P12_CPU_BOUNDARY_H
#define P12_CPU_BOUNDARY_H
// Keep actual CMSIS layouts/bit definitions, replace only compiler intrinsics.
// No ARM instructions, CPU execution, physical MMIO or sleeping. WFI/WFE/SEV
// are recorded requests. Exclusive operations are single-thread host accesses;
// no preemption/atomicity claim. Source: supplied cmsis_gcc.h/core_cm4.h.
#include <stdint.h>
#include <string.h>
#define __CMSIS_GCC_H
#define __ASM __asm
#define __INLINE inline
#define __STATIC_INLINE static inline
#define __STATIC_FORCEINLINE static inline
#define __NO_RETURN __attribute__((noreturn))
#define __USED __attribute__((used))
#define __WEAK __attribute__((weak))
#define __PACKED __attribute__((packed,aligned(1)))
#define __PACKED_STRUCT struct __PACKED
#define __PACKED_UNION union __PACKED
#define __ALIGNED(x) __attribute__((aligned(x)))
#define __RESTRICT __restrict
#define __COMPILER_BARRIER() ((void)0)
#ifdef __cplusplus
extern "C" {
#endif
void p12_cpu_request(unsigned request);
#ifdef __cplusplus
}
#endif
static inline void __DSB(void) { p12_cpu_request(1); }
static inline void __DMB(void) { p12_cpu_request(2); }
static inline void __ISB(void) { p12_cpu_request(3); }
static inline void __WFI(void) { p12_cpu_request(4); }
static inline void __WFE(void) { p12_cpu_request(5); }
static inline void __SEV(void) { p12_cpu_request(6); }
static inline void __NOP(void) { p12_cpu_request(7); }
static inline uint32_t __CLZ(uint32_t x) { return x ? (uint32_t)__builtin_clz(x) : 32U; }
static inline uint32_t __RBIT(uint32_t x) {
    uint32_t r=0; for(unsigned i=0;i<32;++i) { r=(r<<1)|(x&1U); x>>=1; } return r;
}
static inline uint32_t __LDREXW(volatile uint32_t *p) { return *p; }
static inline uint32_t __STREXW(uint32_t x, volatile uint32_t *p) { *p=x; return 0; }
static inline uint16_t __LDREXH(volatile uint16_t *p) { return *p; }
static inline uint32_t __STREXH(uint16_t x, volatile uint16_t *p) { *p=x; return 0; }
static inline uint32_t p12_unaligned_read(const void *p) { uint32_t x; memcpy(&x,p,4); return x; }
static inline void p12_unaligned_write(void *p,uint32_t x) { memcpy(p,&x,4); }
#define __UNALIGNED_UINT32_READ(p) p12_unaligned_read(p)
#define __UNALIGNED_UINT32_WRITE(p,x) p12_unaligned_write(p,x)
#endif
