#include <stdint.h>
// Deliberate owned compiler-boundary controls, never substituted HAL code.
void p12_probe_write32(volatile uint32_t *p,uint32_t x) { *p=x; }
void p12_probe_write_twice(volatile uint32_t *p,uint32_t a,uint32_t b) { *p=a; *p=b; }
uint32_t p12_probe_read32(volatile uint32_t *p) { return *p; }
