# P12: test the checked-in ST HAL

Started 2026-11-06 on the owner's instruction: “I want to leave P11 CMSIS-DSP for now, I want to start P12 and cover the current out of scope ST HAL section of the code, I want this to now be in scope.” P11 is deferred. This instruction supersedes the older ST HAL exclusion for testing; it does not authorize vendor repairs.

## Scope and ownership

In scope: the checked-in `Firmware/Targets/{F407VG,F407VG_DISCO,F411RE}/Drivers/STM32F4xx_HAL_Driver/` code, including extension and supporting LL code, with coverage tracked by module, function and device/configuration. Start from F407VG and explicitly account for DISCO/F411 differences. Unused/dormant modules remain inventoried and must receive a disposition rather than silently disappearing from a denominator.

Vendor sources remain read-only. New code/fakes/build scaffolding belong in `Firmware/Tests/codex/`, documentation here. Preserve existing baseline and regression policy. CMSIS core execution, the FreeRTOS kernel, CubeMX-generated board code, startup/linker code and physical hardware validation remain outside this scope change. CMSIS/device headers may supply types/register definitions; using those headers is not testing the core. Generated board configuration is input evidence, not a newly included implementation under test.

No dependency installation/download, firmware test macros, production edits or hardware access. New shared build changes still follow AGENTS.md ownership. Develop the first harness in an owned standalone CMake project so existing shared executables do not change incidentally. Build only in `../ffb-test-build/codex-p12-<variant>`; shared CTest integration must be prepared as a reviewable patch before its ownership approval gate.

## Architecture and evidence rules

Compile the real unmodified HAL C files. Replace only lower boundaries (CPU intrinsics as necessary, memory-mapped storage/peripheral events, interrupts and deterministic time). Existing fake HAL functions are not substitutes for the HAL under test. Prefer real ST peripheral types/constants and a small, documented boundary adapter over broad hand-copied HAL declarations.

A RAM register struct does not reproduce peripheral semantics. For each modeled behavior document its primary source, implemented transitions, omitted behavior and confidence. Explicitly distinguish checking a HAL register-write request from proving the requested hardware effect. Do not claim interrupt atomicity, write-one-to-clear or FIFO behavior from ordinary RAM assignments. Unsupported operations must fail visibly rather than silently succeed.

Requirements come from supplied API documentation and applicable ST references, not copies of the algorithm under test. Each case uses public interfaces, fixture reset, one behavior and independent expected values. Exercise valid controls, invalid state, busy/error/timeout, callbacks and cleanup as applicable. Use only deterministic simulated time; never sleep. Busy-loop tests need a deterministic event/tick mechanism plus an external process timeout to catch hangs, not a sleep-based success condition. Do not assert an exact hardware deadline the model cannot observe.

Separate valid-input behavior with production assertion configuration from a full-assert validation configuration; disabled `assert_param` is not evidence of runtime argument rejection. Flag pointer/address truncation, host width, alignment, volatile behavior, endianness and target-only instructions. Host success is not ARM/device validation.

## Work sequence

| Stage | Work | Exit evidence |
|---|---|---|
| P12.0 — discovery (started) | Pin all local copies, configs, enabled modules and caller evidence; map HAL versus LL/CMSIS/board boundaries and reference gaps. | Reproducible inventory and per-module proposed requirements; initial inventory is complete, requirement mapping remains. |
| P12.1 — harness and first vertical slice | Prove one unmodified real HAL module can compile/run against the narrow host boundary. Start with GPIO read/write software contracts; assess init/EXTI separately. | Doctest controls plus meaningful corrupted-result/register-write discrimination; fixture isolation; source hashes; clear request-versus-effect limits. No claim of peripheral fidelity from plain RAM. |
| P12.2 — shared prerequisites | Deterministic HAL ticks/timeouts, DMA, interrupt/callback boundaries and any RCC prerequisites required by later groups. | State/error/abort/cleanup contracts, no hangs, independently justified model transitions. |
| P12.3 — communication | SPI, I2C, UART, CAN, including required extensions and DMA/IT/blocking modes. | Data/count/state transitions, busy/error/timeout/abort and callback ordering; target-specific paths accounted for. |
| P12.4 — conversion and timing | ADC/ADCEx and TIM/TIMEx. | Mode/channel boundaries, state and callback sequences, error/stop behavior, scoped register/event fidelity. |
| P12.5 — remaining platform modules | FLASH/Ex/RAMFUNC, RCC/Ex, PWR/Ex, IWDG, EXTI, CORTEX and core HAL entry points. | Testability assessment and bounded tests for each; explicit non-host-testable dispositions. Prior flash tests with fake HAL do not count as real HAL coverage. |
| P12.6 — USB peripheral layer | PCD/PCDEx and LL USB, LL ADC as required by its callers. | Separate boundary from existing TinyUSB fake-DCD suite; no double-counting of that suite as PCD coverage. |
| P12.7 — integration and closeout | Shared gate proposal, independent review/discrimination, coverage collection and regression matrix. | Default/no-DSP/GCC and Clang sanitizer regression policy; original 318 identities; separate real-HAL executable denominator; requirements/source/model coverage and retained limitations. |

This order is initial dependency planning, not an assertion that every module is host-testable. Adjust it when a concrete compilation/reference dependency requires it and record the reason. Stop and report a missing hardware reference before constructing a dependent oracle. Pure API/software work can continue independently.

## Acceptance and reporting

Track each requirement as not started, modeled/tested, defect-marked, blocked on reference, or hardware-only, with precise reason. Report ordinary passes separately from expected failures/skips/exclusions. Demonstrate important assertions reject relevant mutations in scratch copies; do not weaken tests to accept the current vendor implementation. Report vendor bugs without repairing vendor code.

Coverage is a new separate ST HAL section with its own compiled/uncompiled/inactive/configuration accounting; do not inflate the prior firmware ladder. No numeric target is invented at discovery. Define a defensible denominator after compilation/testability mapping, and report untested code explicitly. Every group must leave durable source hashes, commands, results and reference/model limits under its sibling build directory.

## Current resume point

[INVENTORY](INVENTORY.md) and [ST_HAL_INVENTORY.json](ST_HAL_INVENTORY.json) are generated from the current checkout. The first real-source GPIO software-contract slice is implemented and verified: see [GPIO_REQUIREMENTS](GPIO_REQUIREMENTS.md) and [GPIO_RESULTS](GPIO_RESULTS.md). Eight ordinary cases pass in six configurations, and all four bounded mutations are detected. No line-coverage percentage is claimed. The F407 GPIO register adapter now has 36 ordinary model/public-HAL cases passing in default/DISCO/GCC sanitizer/Clang sanitizer, plus nine discriminating HAL/model mutations. See [REGISTER_MODEL_DESIGN](REGISTER_MODEL_DESIGN.md) and [REGISTER_MODEL_RESULTS](REGISTER_MODEL_RESULTS.md) for the C++ observation versus real-C boundary. The time/state prerequisite group is now also implemented; see TIME_STATE_RESULTS. Next: choose a safe address/access strategy before active DMA register tests, or continue independent HAL contracts, retaining existing GPIO/time gates. Shared CTest integration remains future work. The owner supplied the STM32F405xx/407xx datasheet, now indexed in [REFERENCES](REFERENCES.md). RM0090 Rev 22 and ES0182 Rev 19 are now supplied and indexed in REFERENCES; their F407 GPIO sections can support the next model after targeted reading. F411-specific references remain outstanding. Existing P0–P10 changes remain uncommitted and must be preserved.

## Latest continuation: real-C time/state prerequisites

[TIME_STATE_REQUIREMENTS](TIME_STATE_REQUIREMENTS.md) and [TIME_STATE_RESULTS](TIME_STATE_RESULTS.md) record 28 ordinary new cases plus P12-TICK-001. Five current configurations pass their standalone gates (5/5; F411 4/4), retaining GPIO checks. Seven fault controls and a bounded scratch repair discriminator pass their exact expected result checks. Tick/state source compiles as real C. Active DMA register paths still require a justified address strategy because the vendor stores derived addresses in uint32_t; no unsafe truncating host execution was used. Next is that address/access design or independent HAL software contracts, with vendor sources read-only.
