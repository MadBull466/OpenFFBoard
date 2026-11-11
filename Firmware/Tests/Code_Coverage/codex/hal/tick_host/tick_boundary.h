#ifndef CODEX_P12_TICK_BOUNDARY_H
#define CODEX_P12_TICK_BOUNDARY_H
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
// CPU boundary: only a CTRL storage observation. TICKINT bit position comes
// from supplied CMSIS core_cm4.h, not an invented peripheral register map.
// No CPU/SysTick/NVIC implementation, scheduling, timing or interrupt delivery.
typedef struct { volatile uint32_t CTRL; } P12_TickControl;
extern P12_TickControl p12_tick_control;
#define SysTick (&p12_tick_control)
#define SysTick_CTRL_TICKINT_Msk (1UL << 1U)
typedef struct {
    uint32_t reload;
    uint32_t configCalls;
    uint32_t configResult;
    uint32_t priorityCalls;
    int32_t irq;
    uint32_t preempt;
    uint32_t sub;
} P12_TickObservations;
extern P12_TickObservations p12_tick_observations;
void p12_reset_tick_boundary(void);
void p12_tick_unsupported(const char *operation);
void p12_set_tick_script(const uint32_t *values, size_t count);
uint32_t p12_next_tick(void);
uint32_t p12_last_tick(void);
uint64_t p12_elapsed_ticks(void);
size_t p12_tick_reads(void);
int p12_tick_script_exhausted(void);
#ifdef __cplusplus
}
#endif
#endif
