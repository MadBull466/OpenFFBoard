#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t SystemCoreClock = 16000000U;
P12_TickControl p12_tick_control;
P12_TickObservations p12_tick_observations;
static uint32_t samples[32];
static size_t sampleCount, position;
static int exhausted;
static uint64_t elapsed;

void p12_reset_tick_boundary(void) {
    memset(&p12_tick_observations, 0, sizeof p12_tick_observations);
    p12_tick_control.CTRL = 0;
    SystemCoreClock = 16000000U;
    uwTick = 0;
    uwTickFreq = HAL_TICK_FREQ_DEFAULT;
    uwTickPrio = 1UL << __NVIC_PRIO_BITS;
    sampleCount = position = 0;
    exhausted = 0;
    elapsed = 0;
}

uint32_t HAL_SYSTICK_Config(uint32_t ticks) {
    p12_tick_observations.reload = ticks;
    ++p12_tick_observations.configCalls;
    return p12_tick_observations.configResult;
}
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t preempt, uint32_t sub) {
    ++p12_tick_observations.priorityCalls;
    p12_tick_observations.irq = irq;
    p12_tick_observations.preempt = preempt;
    p12_tick_observations.sub = sub;
}
void HAL_NVIC_SetPriorityGrouping(uint32_t grouping) {
    (void)grouping;
    p12_tick_unsupported("priority grouping outside tick slice");
}
void p12_tick_unsupported(const char *operation) {
    fprintf(stderr, "P12 unsupported tick boundary: %s\n", operation);
    abort();
}
void p12_set_tick_script(const uint32_t *values, size_t count) {
    if (count == 0 || count > 32) p12_tick_unsupported("invalid tick script");
    memcpy(samples, values, count * sizeof *values);
    sampleCount = count;
    position = 0;
    exhausted = 0;
    elapsed = 0;
}
uint32_t p12_next_tick(void) {
    if (sampleCount == 0) p12_tick_unsupported("tick read without a script");
    if (position >= sampleCount) {
        exhausted = 1; // a returning test must reject this fault explicitly
        return samples[sampleCount - 1];
    }
    if (position > 0) elapsed += (uint32_t)(samples[position] - samples[position - 1]);
    return samples[position++];
}
uint32_t p12_last_tick(void) {
    if (position == 0) p12_tick_unsupported("no observed tick");
    return samples[position - 1];
}
size_t p12_tick_reads(void) { return position; }
uint64_t p12_elapsed_ticks(void) { return elapsed; }
int p12_tick_script_exhausted(void) { return exhausted; }
