#include "stm32f4xx_hal.h"
// Explicit public weak-function override ONLY in the HAL_Delay executable.
// The state/configuration executable separately tests the genuine GetTick.
uint32_t HAL_GetTick(void) { return p12_next_tick(); }
