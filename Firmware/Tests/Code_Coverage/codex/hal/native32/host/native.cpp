#include "native.h"
#include <sys/mman.h>
#include <cstring>
#include <stdexcept>
#include <array>
#include <cstdio>
#include <cstdlib>
extern "C" {
uint32_t SystemCoreClock = 16000000;
// Data from supplied system_stm32f4xx.c; no generated board code executed.
const uint8_t AHBPrescTable[16] = {0,0,0,0,0,0,0,0,1,2,3,4,6,7,8,9};
const uint8_t APBPrescTable[8] = {0,0,0,0,1,2,3,4};
}
namespace {
struct Window { uintptr_t address; size_t size; };
constexpr std::array<Window,8> windows{{
    {0x08000000,0x200000}, {0x1fff0000,0x10000}, {0x20000000,0x200000},
    {0x40000000,0x80000}, {0x42000000,0x1000000}, {0x50000000,0x40000}, {0xe000e000,0x2000}, {0xe0042000,0x1000}
}};
std::vector<unsigned> requests;
std::function<void(uint32_t)> step;
bool mapped;
}
namespace native {
void reset() {
    static_assert(sizeof(void*) == 4, "DMA address arithmetic requires actual 32-bit pointers");
    if (!mapped) {
        for (const auto &w : windows) {
            void *p = mmap(reinterpret_cast<void*>(w.address),w.size,PROT_READ|PROT_WRITE,
                           MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED_NOREPLACE,-1,0);
            if (p == MAP_FAILED || reinterpret_cast<uintptr_t>(p) != w.address)
                throw std::runtime_error("P12 anonymous address window unavailable");
        }
        mapped = true;
    }
#ifdef P12_ACCESS_MODEL
    resetBus();
#endif
    for (const auto &w : windows) std::memset(reinterpret_cast<void*>(w.address),0,w.size);
    std::memset(reinterpret_cast<void*>(0x08000000),0xff,0x200000);
    requests.clear(); step = {};
    uwTick = 0; uwTickFreq = HAL_TICK_FREQ_1KHZ; uwTickPrio = 4;
    SystemCoreClock = 16000000;
    RCC->CR = RCC_CR_HSION | RCC_CR_HSIRDY;
    RCC->PLLCFGR = 0x24003010; // RM0090 reset value
    FLASH->CR = FLASH_CR_LOCK;
    FLASH->OPTCR = FLASH_OPTCR_OPTLOCK;
    SysTick->CTRL = 0;
}
void event(std::function<void(uint32_t)> fn) { step = std::move(fn); }
const std::vector<unsigned> &cpuRequests() { return requests; }
}
extern "C" void p12_cpu_request(unsigned request) {
#ifdef P12_ACCESS_MODEL
    native::flush();
#endif
    requests.push_back(request);
}
extern "C" uint32_t HAL_GetTick(void) {
#ifdef P12_ACCESS_MODEL
    native::flush(); native::advanceBus();
#endif
    uint32_t now = uwTick; uwTick = now + 1U;
    if (step) step(now);
    return now;
}

#ifdef USE_FULL_ASSERT
namespace {bool expectedAssertion;}
namespace native {void expectAssertion(bool enabled){expectedAssertion=enabled;}}
extern "C" void assert_failed(uint8_t *file,uint32_t line) {
    if(expectedAssertion) throw native::AssertionObserved{};
    std::fprintf(stderr,"P12 vendor assertion: %s:%u\n",file,static_cast<unsigned>(line));
    std::abort();
}
#endif
