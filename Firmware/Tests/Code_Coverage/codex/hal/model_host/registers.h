#ifndef CODEX_P12_REGISTER_ADAPTER_H
#define CODEX_P12_REGISTER_ADAPTER_H

// Included from the real HAL header's extern-C block: only the adapter uses
// C++ linkage. Rename the ST storage typedefs while retaining the real device
// declarations/constants; validate every replacement offset below.
#ifdef __cplusplus
extern "C++" {
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#define GPIO_TypeDef P12_StorageGPIO
#define EXTI_TypeDef P12_StorageEXTI
#define SYSCFG_TypeDef P12_StorageSYSCFG
#include "stm32f4xx.h"
#undef GPIO_TypeDef
#undef EXTI_TypeDef
#undef SYSCFG_TypeDef

namespace p12 {
struct Register;
std::uint32_t read(const volatile Register *reg);
void write(volatile Register *reg, std::uint32_t value);
struct Register {
    std::uint32_t storage{};
    operator std::uint32_t() const volatile { return read(this); }
    void operator=(std::uint32_t value) volatile { write(this, value); }
    void operator|=(std::uint32_t value) volatile { write(this, read(this) | value); }
    void operator&=(std::uint32_t value) volatile { write(this, read(this) & value); }
};
}
struct GPIO_TypeDef {
    p12::Register MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR;
    p12::Register AFR[2];
};
struct EXTI_TypeDef { p12::Register IMR, EMR, RTSR, FTSR, SWIER, PR; };
struct SYSCFG_TypeDef {
    p12::Register MEMRMP, PMC, EXTICR[4];
    p12::Register RESERVED[2];
    p12::Register CMPCR;
};
#define P12_OFFSET(type, field) static_assert(offsetof(type##_TypeDef, field) == offsetof(P12_Storage##type, field))
static_assert(sizeof(p12::Register) == sizeof(std::uint32_t));
static_assert(alignof(p12::Register) == alignof(std::uint32_t));
static_assert(sizeof(GPIO_TypeDef) == sizeof(P12_StorageGPIO));
static_assert(sizeof(EXTI_TypeDef) == sizeof(P12_StorageEXTI));
static_assert(sizeof(SYSCFG_TypeDef) == sizeof(P12_StorageSYSCFG));
P12_OFFSET(GPIO, MODER); P12_OFFSET(GPIO, OTYPER); P12_OFFSET(GPIO, OSPEEDR);
P12_OFFSET(GPIO, PUPDR); P12_OFFSET(GPIO, IDR); P12_OFFSET(GPIO, ODR);
P12_OFFSET(GPIO, BSRR); P12_OFFSET(GPIO, LCKR); P12_OFFSET(GPIO, AFR);
P12_OFFSET(EXTI, IMR); P12_OFFSET(EXTI, EMR); P12_OFFSET(EXTI, RTSR);
P12_OFFSET(EXTI, FTSR); P12_OFFSET(EXTI, SWIER); P12_OFFSET(EXTI, PR);
P12_OFFSET(SYSCFG, MEMRMP); P12_OFFSET(SYSCFG, PMC); P12_OFFSET(SYSCFG, EXTICR);
P12_OFFSET(SYSCFG, RESERVED); P12_OFFSET(SYSCFG, CMPCR);
#undef P12_OFFSET

namespace p12 {
struct Access { std::string name; bool isWrite; std::uint32_t value; };
struct Callback { std::uint16_t pin; std::uint32_t pendingAtCallback; };
void reset();
GPIO_TypeDef *gpio(unsigned port);
EXTI_TypeDef *exti();
SYSCFG_TypeDef *syscfg();
void enableSyscfgClock();
bool syscfgClockEnabled();
void seed(Register &reg, std::uint32_t value); // fixture state injection, not a bus write
std::uint32_t peek(const Register &reg);       // fixture observation, not a bus read
void injectPending(std::uint16_t mask);
const std::vector<Access> &trace();
const std::vector<Callback> &callbacks();
}

#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef GPIOD
#undef GPIOE
#undef GPIOF
#undef GPIOG
#undef GPIOH
#undef GPIOI
#undef SYSCFG
#undef EXTI
#define GPIOA p12::gpio(0)
#define GPIOB p12::gpio(1)
#define GPIOC p12::gpio(2)
#define GPIOD p12::gpio(3)
#define GPIOE p12::gpio(4)
#define GPIOF p12::gpio(5)
#define GPIOG p12::gpio(6)
#define GPIOH p12::gpio(7)
#define GPIOI p12::gpio(8)
#define SYSCFG p12::syscfg()
#define EXTI p12::exti()
} // extern C++
#else
#error "The P12 access-observation adapter requires C++; retain the separate real-C storage build"
#endif
#endif
