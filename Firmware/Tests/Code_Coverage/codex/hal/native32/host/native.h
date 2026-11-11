#pragma once
#include "doctest.h"
#include "stm32f4xx_hal.h"
#include <functional>
#include <vector>
#include <stdexcept>
namespace native {
#ifdef USE_FULL_ASSERT
struct AssertionObserved : std::exception {};
void expectAssertion(bool enabled);
#endif
void reset();
void event(std::function<void(uint32_t)> fn);
const std::vector<unsigned> &cpuRequests();
void resetBus();
void flush();
void advanceBus();
struct Write { uintptr_t address; unsigned width; uint64_t value; };
const std::vector<Write> &writes();
void i2cPeer(unsigned index,uint8_t address,const std::vector<uint8_t> &input={});
const std::vector<uint8_t> &i2cWritten(unsigned index);
void adcInput(unsigned index,const std::vector<uint32_t> &input);
void uartInput(unsigned index,const std::vector<uint16_t> &input);
void spiInput(unsigned index,const std::vector<uint16_t> &input);
void completeDMA(DMA_HandleTypeDef &handle);
void usbInput(unsigned index,const std::vector<uint32_t> &words);
struct Fixture {
    Fixture() { reset(); }
    ~Fixture() {
#ifdef P12_ACCESS_MODEL
        flush();
#endif
        reset();
    }
};
}
