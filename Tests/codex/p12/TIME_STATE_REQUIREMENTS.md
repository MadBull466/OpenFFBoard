# P12 HAL time and DMA-state requirements

This group compiles unchanged `stm32f4xx_hal.c` and `stm32f4xx_hal_dma.c` as C. Tick/state expectations come from their supplied public API documentation, public frequency/handle definitions and independently derived millisecond/field values. No STM32 registers, opcodes or peripheral frames are invented.

## Executable boundaries

| Target | Real implementation | Lower boundary / intentional substitution |
|---|---|---|
| p12_tick_tests | GetTick, IncTick, InitTick, Set/GetTickFreq, GetTickPrio, Suspend/ResumeTick | Record HAL_SYSTICK_Config and NVIC priority requests; program a returned configuration error; observe only SysTick CTRL storage |
| p12_delay_tests | HAL_Delay, frequency/configuration functions | Public weak HAL_GetTick override supplies deterministic samples; real GetTick remains separately exercised above |
| p12_dma_state_tests | DMA callback registration/unregistration, state/error accessors, null-init and inactive-poll guards | Null Instance is intentional: selected paths must return before touching a stream or computing derived addresses; real core GetTick is linked |

Fixtures reset the HAL's public exported `uwTick`, `uwTickFreq`, `uwTickPrio`, SystemCoreClock and all owned recorder/script state. This is fixture setup, not private-member access. Each test then calls public interfaces. No sleeping, scheduler, simulated CPU, peripheral transfer or target deadline is inferred.

## Contracts and independent oracles

- Default IncTick advances one millisecond; selected 100 Hz/10 Hz rates advance 10/100 ms. Unsigned wrap is defined. With a 16 MHz SystemCoreClock, requested intervals require reloads of 16000, 160000 and 1600000 cycles respectively; these are hand-derived clock/rate quotients.
- InitTick publishes the requested valid priority only after successful lower configuration; failed configuration/invalid priority returns HAL_ERROR without publishing a new priority. Tests do not require that a rejected priority leaves all lower hardware configuration untouched.
- SetTickFreq preserves the old logical frequency if reinitialization fails and avoids reconfiguration when the frequency is unchanged. This is logical state recovery, not proof the hardware reconfiguration can be rolled back.
- Suspend/Resume request changes only to the SysTick interrupt-enable bit. The bit position comes from the supplied CMSIS core_cm4.h. Observing a CTRL word is not executing CMSIS core/SysTick or delivering an interrupt.
- HAL_Delay must not return before the requested elapsed millisecond interval. Tests check this lower bound, not an exact polling count or the exact chosen extra-quantum formula. They include ordinary counter rollover, a coarse tick, maximum delay and a large default-frequency positive control.
- A large finite delay with a 100 ms tick has a separately marked intended-minimum assertion, documented in [FINDINGS](FINDINGS.md). The verifier checks its precise first failure and prohibits setup/model failures from satisfying the marker.
- DMA READY registration replaces the named callback only; six distinct header-named slots are checked against two distinct function pointers. Unregistration clears the named slot or all six, preserving other slots for a single-slot request. Nonready operations fail without replacing callbacks; locked operations return busy without releasing another owner's lock. State/error accessors preserve distinct public values. Null initialization and nonbusy polling fail safely at their documented source guards.

## Clock-script model

The model returns a fixed supplied sequence, records consumption and accumulates elapsed milliseconds from successive unsigned 32-bit deltas into a 64-bit observation. This is test-clock state, not a replacement delay implementation. Direct controls validate sequence/rollover and explicit exhaustion recording. An exhausted script repeats the last sample but records a fault; every returning delay test rejects that fault. A stuck mutant is bounded by process timeout and is not accepted as a valid discriminator.

The model assumes successive samples are separated by less than one complete 32-bit clock period. Repeated samples and single rollover are allowed. The large/coarse test samples are all possible multiples of a 100 ms increment modulo 2^32. The independent minimum-duration oracle uses the model's accumulated elapsed time; it does not read the HAL's local wait variable or copy its overflow-prone addition.

A scratch widened-duration accumulator is used only to check repair discrimination: ordinary assertions must pass and the old should_fail marker alone must flip. This does not establish arbitrary scheduling/interrupt behavior or endorse that scratch implementation as a production repair.

## Limits

The owned configuration enables only the declarations needed here. HAL_Init/DeInit, caches, compensation cells, debug/UID accesses and real RCC/Cortex functions are not executed. Cache configuration is disabled for this tick-only compile, unlike the production board configuration. Unsupported reset/grouping calls abort. Recording SysTick/NVIC requests is an explicit lower CPU boundary; it does not bring CMSIS core execution into scope.

DMA's target code converts stream pointers to uint32_t and calculates a physical register bank. Those paths cannot safely use arbitrary 64-bit host pointers. Their compiler warnings remain visible, exempted from -Werror only for the C DMA-state target; no runtime/sanitizer suppression. Active DMA initialization, start, abort, IRQ, FIFO, memory movement and timeout loops remain untested. No 32-bit address workaround, transformed vendor source, mapped physical-address window or downloaded emulator is introduced.

F411 builds establish checked-in source/header software compatibility only, pending its device references. No numeric coverage percentage or complete P12 acceptance is implied. No shared/production/vendor/Claude/original-baseline files are changed by this group.
