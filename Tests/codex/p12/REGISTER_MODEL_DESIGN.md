# P12 GPIO register observation adapter

The adapter executes the unchanged checked-in GPIO implementation in a separate C++ target so each register read and write can be observed. The independent `p12_gpio_tests` target still compiles the real source as C and tests its read/write/toggle requests. This is a deliberate alternate compilation boundary, not proof of equivalent Cortex-M C execution.

## Boundary and layout

`model_host/registers.h` first includes the actual ST device header with three storage typedefs renamed. It then substitutes GPIO, EXTI and SYSCFG structs whose 32-bit register fields dispatch accesses to the model. The original HAL headers, macros and numeric definitions remain in use. Static assertions compare size, alignment and every replaced member offset against the ST storage types. No register address map is invented and no memory is mapped at peripheral physical addresses.

`model_host/real_gpio.cpp` includes the vendor source directly, without rewriting its bytes. The C++ observer is F407-only (including DISCO's identical source/configuration); F411 remains in the C storage/header-compatibility track. Public callback override uses the real HAL weak-callback mechanism. Full parameter assertions are enabled in this observation target and report invalid inputs by C++ exception; that exception mechanism is a host boundary, not production assert_failed behavior.

The real GPIO implementation's SYSCFG clock-enable macro is a lower-boundary request recorded by the model. This does not execute or test RCC. GPIO port clocks are assumed already enabled by the fixture precondition. The adapter does not claim that port-clock gating is tested.

## Modeled behavior and source

| Register family | Modeled rule | Supplied source |
|---|---|---|
| MODER/OTYPER/OSPEEDR/PUPDR/AFRL/AFRH | Word reads/writes, selected locked-field preservation; reserved OTYPER bits and reserved pull encoding rejected | RM0090 Rev 22 §§8.4.1–4, 8.4.8–10 |
| IDR | Fixture-injected input data; software writes rejected | RM0090 §8.4.5 |
| ODR/BSRR | Word data writes; BSRR requests set/reset ODR with set priority, BSRR reads return zero | RM0090 §§8.4.6–7 |
| LCKR | Stable-mask 1/0/1 key sequence followed by read establishes lock; other three-write patterns/mask changes do not; lock cannot be retargeted before reset | RM0090 §8.4.8 |
| SYSCFG EXTICR | Four routing fields per word, guarded by a recorded SYSCFG clock request; unsupported/reserved routes rejected | RM0090 §§9.2.3–6; HAL GPIO clock prerequisite |
| EXTI IMR/EMR/RTSR/FTSR | Lower-16 GPIO line configuration storage | RM0090 §§12.3.1–4 |
| EXTI PR | Explicit injected pending events; writing one clears only that pending bit; zero leaves it unchanged | RM0090 §12.3.6 |

All register accesses are traced with register name, direction and value. Fixture seed/peek operations bypass bus semantics for deliberate setup/observation; they are not simulated CPU accesses. In particular, peek observes internal backing storage and is not a substitute for reading BSRR/LCKR through their protocol. The fixture initializes documented A/B debug configuration defaults; IDR is deliberately seeded to deterministic zero, not claimed to have a defined silicon reset input value.

The lock interpreter is validated independently of HAL against all seven invalid three-key patterns, a changed mask, valid sequences, nonzero retained fields, high/low alternate-register boundaries, output-data access while locked and reset. The HAL test separately checks its five actual LCKR accesses (three writes, two reads), success and protection of a subsequent configuration request.

## Strict limits

Unregistered registers, unsupported SYSCFG fields/reserved words, SWIER generation, input-register writes, unmodeled internal EXTI lines and unsupported/reserved bit patterns fail by exception. PH10/PI10 routing is rejected because ES0182 §2.2.10 requires silicon-revision-aware treatment; no idealized routing behavior is promised. Complete silicon-revision applicability mapping remains future work. No hardware edge detector or port-to-EXTI signal routing is simulated: injecting PR establishes an already-pending interrupt, independently of pin edge generation.

No byte/halfword accesses, GPIO output-to-input electrical loopback, pin/package bonding, oscillator/debug-pin ownership, analog electrical effects, bus timing, ordering barriers, CPU/NVIC, preemption, reset-controller implementation or RCC implementation is modeled. ODR effects do not automatically change IDR. No silicon or ARM validation is inferred. No F411 peripheral-model fidelity is claimed from RM0090/ES0182.

Confidence is bounded by the direct controls and discrimination experiments in [REGISTER_MODEL_RESULTS](REGISTER_MODEL_RESULTS.md). The source-level observer changes register types and compilation language; the real-C storage build remains essential, and broader language/target equivalence is unverified. All expectations come from the supplied references or independent field derivations, not simply from the HAL's current outputs. Fresh independent-agent review has not yet occurred.
