# P12 plan: tests of the real ST HAL (Claude, fresh start)

Status: **approved by the owner 2026-11-11.** T0 trial done, approved and committed 2026-11-11 (section "T0 result"). T1 (`f774247b`) and T2 (`f22d81f3`) done, approved and committed locally 2026-11-11, not pushed (sections "T1 result", "T2 result"). **T3 (DMA) finished and gated 2026-11-11, uncommitted, waiting for the owner's "approve ... and commit"** (section "T3 result"; charter `Tests/Code_Coverage/claude/charters/hal/P12_T3_REQUIREMENTS.md`). All questions (65-69) answered 2026-11-11 (section 6); 66 revised to F407VG and F407VG_DISCO.

Owner's instruction (2026-11-11): "I want to start on P12 that was started by Codex. I want to start from the beginning for that Phase and not use any of their files."

**Independence from Codex.** Codex's P12 lives in `Tests/Code_Coverage/codex/p12/` and `Firmware/Tests/Code_Coverage/codex/hal/`. Claude read only the first page of Codex's `p12/PLAN.md`, to find out what "P12" means (the checked-in ST HAL). Nothing else from Codex is read, copied, included, linked or built on: no harness, register model, inventory, requirements, results, tests or CMake. Codex's opt-in `FFB_TESTS_ST_HAL` gate and its findings stay as they are, untouched, and are not counted in Claude's numbers. Every new name avoids Codex's (option `FFB_TESTS_REAL_HAL`, executables `ffb_hal_tests` and `ffb_hal_tests_disco`). The owner's `P12-TICK-001` / `P12-I2C-001` entries in `CODEX_REVIEW_AUDIT.md` are historical; if Claude finds the same defect independently it gets a new Claude number.

## 1. Why

Every executable so far replaces the ST HAL with fakes (`platform/`, `fakes/`), so the firmware is tested against the behaviour the fakes assume. The HAL itself (49,723 lines in 27 files, version V1.8.3) has never been run by Claude's suite. `CLAUDE.md` still lists "ST HAL" and `Firmware/Targets/**` as not tested (question 65).

## 2. What is under test

- Sources: `Firmware/Targets/F407VG/Drivers/STM32F4xx_HAL_Driver/Src/*.c`, compiled unchanged with the F407VG `stm32f4xx_hal_conf.h` (`Firmware/Targets/F407VG/Core/Inc/`). Read-only, never edited, defects reported only (vendor bug numbers V89 onwards).
- `F407VG_DISCO` (in scope, question 66): its whole `Drivers/` folder and its `stm32f4xx_hal_conf.h` are byte-identical to F407VG's, the Makefile gives it the same `-DSTM32F407xx`, and its `SystemClock_Config` passes the HAL the same oscillator and clock values (PLLM 4, PLLN 168, PLLP /2, PLLQ 7, APB1 /4, APB2 /2, `FLASH_LATENCY_5`). Only its pins and board setup in the CubeMX `main.c` / `main.h` differ, and the HAL does not include `main.h`. So the DISCO HAL build is expected to be the same code; the plan still builds and runs it as its own executable (`ffb_hal_tests_disco`, same test files, DISCO include folder) and adds an identity check that fails the moment the two targets' HAL sources or `hal_conf` diverge. Board values (pins, clocks) taken from each target's `main.c` are test inputs only; the CubeMX code itself stays untested.
- `F411RE` is out (question 66): identical HAL sources except that it has no `stm32f4xx_hal_can.c`, but another device header (`STM32F411xE`).
- Upstream check (2026-11-11): all 27 checked-in `Src/*.c` files are byte-identical to ST's V1.8.3 release supplied in `Tests/Code_Coverage/shared/specs/stm32f4xx-hal-driver-1.8.3/` (`cmp`, 27 of 27).
- Enabled modules (F407VG `hal_conf`): HAL core, ADC, CAN, I2C, IWDG, SPI, TIM, UART, PCD, GPIO, EXTI, DMA, RCC, FLASH, PWR, CORTEX. `USE_FULL_ASSERT` is off in the firmware, so `assert_param` checks are dead code on target.
- Not under test: CMSIS core (`core_cm4.h` inline functions are used, not tested), CubeMX-generated `Core/Src`, startup and linker files, the FreeRTOS kernel.

## T0 result (2026-11-11, approved and committed)

Lead only, no agents. GCC 16, out of tree.

- **Compile trial:** all 27 F407VG HAL files compile unchanged as 32-bit code. Only 4 needed a host replacement for the ARM CPU instructions: `stm32f4xx_hal_cortex.c` and `stm32f4xx_hal_flash.c` (`isb` / `dsb`), `stm32f4xx_hal_pwr.c` (`wfi` / `wfe` / `sev`), `stm32f4xx_hal_uart.c` (`__LDREXW` / `__STREXW` in the `ATOMIC_SET_BIT` macro of `stm32f4xx.h`). The other 23 needed nothing. No warnings from the HAL, its fakes or its tests.
- **New files:**
  - `Firmware/Tests/Code_Coverage/fakes/stm32_hal/`:
    - `host_cmsis_gcc.h` / `host_cmsis.c`: force-included; defines the guard of `cmsis_gcc.h`, so the real header is skipped. Provides barriers, `__NOP`, `__WFI` / `__WFE` / `__SEV` (counted, wait hook), an exclusive monitor for `__LDREXW` / `__STREXW` / `...H` (failure injection) and `__RBIT` / `__CLZ`. Interrupt-mask intrinsics are left undefined on purpose: no HAL file uses them.
    - `HalRegisterSpace.h` / `.c`: `mmap(MAP_FIXED_NOREPLACE)` of 0x40000000 (+512 KiB), 0x50000000 (+388 KiB), 0xE000E000, 0xE0042000, 0x1FFF7000 and 0x1FFFC000 (RM0090 Table 1, core_cm4.h). Zeroed before every case. Plain RAM: no peripheral behaviour yet.
    - `HalHostSystem.h` / `.c`: `SystemCoreClock` (reset 16 MHz, RM0090 7.2.2), `AHBPrescTable` / `APBPrescTable` (derived from the RCC_CFGR field encodings, RM0090 7.3.3), and a strong `HAL_GetTick` that runs one simulated 1 ms tick interrupt (the real `HAL_IncTick`) per call. The tick step can be replaced or stopped by a test.
  - `Firmware/Tests/Code_Coverage/claude/cases_hal/`:
    - `HalFixture.h`: resets the registers, intrinsics, system globals and the HAL tick globals.
    - `test_hal_t0_smoke.cpp`: 6 cases (block addresses; `HAL_GPIO_WritePin` SET / RESET to BSRR; `HAL_GPIO_ReadPin` SET / RESET from IDR; `HAL_Delay` minimum delay).
    - `check_hal_identity.cmake`: compares the F407VG and DISCO HAL, CMSIS and `hal_conf` files; 110 files today.
- **CMake:** option `FFB_TESTS_REAL_HAL` (default OFF). It stops at configure time unless `FFB_TESTS_32BIT=ON` on Linux (checked). The HAL is an OBJECT library per target, so the strong `HAL_GetTick` always wins over the `__weak` one. Two executables come from the same test files. ctest entries `ffb_hal_tests`, `ffb_hal_tests_disco` (TIMEOUT 120 s) and `hal_identity_f407vg_disco`.
- **Gate:**

  | Build (`../ffb-test-build/`) | ctest | `check_baseline.py` | HAL executables |
  |---|---|---|---|
  | `p12t0-32hal` (`-DFFB_TESTS_32BIT=ON -DFFB_TESTS_REAL_HAL=ON`) | 14 / 14 passed | OK (340 new unit cases, 0 failed) | `ffb_hal_tests` 6 / 6, `ffb_hal_tests_disco` 6 / 6 (18 assertions each) |
  | `p12t0-default` (option off) | 11 / 11 passed | OK (340 new unit cases, 0 failed) | not built |
  | `p12t0-32san-hal` (32-bit + sanitizers, HAL targets only) | not run as ctest | not run | 6 / 6 each, no sanitizer report |

- **Mutation spot-check** (scratch copy `../ffb-test-build/audit/p12t0`, repo untouched): 8 of 8 mutants killed.
  - HAL code: `WritePin` SET writes the BR half; RESET writes the BS half; `ReadPin` inverted; `ReadPin` reads ODR; `HAL_Delay` returns 4 ms early.
  - Identity check: one DISCO HAL file changed; DISCO `hal_conf` HSE changed; DISCO CAN file missing.
  - The unchanged copy still passes the identity check.
- **Not done in T0:** no register reset values and no peripheral behaviour (T1 onwards); no coverage measured (tier D starts at T1); `run_all_tests.py` and `coverage_report.py` do not list the HAL executables yet (T9); not run with clang.
- **Limits found:**
  - The real `HAL_GetTick` (`return uwTick;`, `__weak`) is replaced and never runs. The firmware uses the real one; it replaces only `HAL_InitTick` (TIM-based time base in `Core/Src/stm32f4xx_hal_timebase_tim.c`).
  - `NVIC_SystemReset` ends in `for(;;) __NOP();`. A test that reaches it hangs until the ctest timeout, so T1 must treat it as hardware-only or give `__NOP` an escape hook.

## T1 result (2026-11-11, gated, not yet committed)

HAL core, CORTEX, GPIO and EXTI of both targets. Charter G1-G14, E1-E10, H1-H10, C1-C17. GCC 16, 32-bit, out of tree.

- **Platform (lead):** `HalRegisterSpace` is now a per-access trapping bus (pages `PROT_NONE`, SIGSEGV, x86 single step, SIGTRAP runs the model; an access without a model is a fault that fails the case; access log; bit-band alias 0x42000000 of 0x40000000-0x4007FFFF, RM0090 2.3.3). `HalPeripherals`: GPIO (LCKR lock sequence), EXTI, SYSCFG, RCC (storage), FLASH_ACR, DBGMCU, unique id (RM0090 Rev 22). `HalCorePeripherals`: NVIC (82 IRQs), SCB, SysTick, MPU (PM0214 Rev 10). `hostCmsisNopHook` lets a test leave the endless loop after `NVIC_SystemReset`.
- **Tests (`claude/cases_hal/`):** GPIO 63, EXTI 48, core 32, CORTEX 47 + 1, SysTick/core-tick 22 + 8, T0 smoke 6: 225 cases in each executable, 0 failed; 10 expected-fail assertions inside the `KNOWN_ISSUE` cases (V89: 3 cases, V90: 1 case).
- **Reviews:** GPIO / EXTI / core: 53 of 61 mutants killed, 4 real gaps fixed (EXTICR nibbles, analog pull, IDCODE bits, port I, line 14); survivors equivalent or too long to run. CORTEX: verdict accept, 53 mutants, 4 real survivors (MPU Enable/DisableRegion RNR, InitTick subpriority, SEV before WFE) and 5 small gaps (`HAL_SYSTICK_Config(0)`, tick frequency 100 Hz, failed `SetTickFreq` leaves SysTick unchanged); all fixed afterwards. The fixes were not re-mutated.
- **Gate (lead-run, `../ffb-test-build/`):**

  | Build | Result |
  |---|---|
  | `p12t1-gate` (32-bit + HAL) | ctest 14 / 14, `check_baseline.py` OK; `ffb_hal_tests` and `ffb_hal_tests_disco` 225 / 225, 1951 of 1961 assertions (10 expected) |
  | `p12t1-default` (option off) | ctest 11 / 11, `check_baseline.py` OK (run before the CORTEX test fixes, which only touch HAL test files) |
  | `p12t1-32san` (32-bit + HAL + sanitizers) | HAL executables 225 / 225 each, no sanitizer report; the full ctest aborts in `ffb_flash_tests` (see limits) |

- **Coverage (gcov, `p12t1-cov`, identical for both targets):** `stm32f4xx_hal.c` 98 / 100 (the replaced `HAL_GetTick`), `_gpio.c` 110 / 113 (`LockPin` HAL_ERROR path unreachable with a correct lock model, weak callback), `_exti.c` 98 / 98, `_cortex.c` 75 / 77 (weak callback): 381 / 388 lines per target.
- **Findings:** vendor V89 (`HAL_GPIO_DeInit` debug pins) and V90 (`HAL_Delay` near `HAL_MAX_DELAY` at 10 Hz); NEEDS REVIEW 281-287.
- **Model decisions:** the clock gate of a peripheral is not modelled (RM0090 is silent); an EXTI edge sets PR only with IMR set (medium confidence); the lock becomes active at the first read after the third LCKR write; STK_CALIB reads 0xC0000000 (PM0214 contradicts itself); H9 oracle is the RM0090 reset bits per register.
- **Limits:** the 32-bit sanitizer build cannot run `ffb_flash_tests` (the i386 ASan layout collides with its fixed mapping at 0x1FFF0000; not a P12 regression, the HAL executables run fine with sanitizers); not run with clang, MinGW, macOS or on ARM; peripheral timing and interrupts beyond the NVIC register model are not modelled.

## T2 result (2026-11-11, gated, not yet committed)

RCC / RCC_Ex and PWR / PWR_Ex of both targets (F405/407 branches that compile for the F407). Charter RC1-RC15, RX1-RX4, PW1-PW11. GCC 16, 32-bit, out of tree.

- **Platform (lead):** `fakes/stm32_hal/HalClockPower.{h,c}` replaces the storage-only RCC model: oscillator and PLL ready flags driven by the HAL tick (default 1 tick, knobs for start-up time, never ready, ready flag that never clears), SWS follows SW once the source is ready, ON bits that cannot be cleared while in use, PLLCFGR / PLLI2SCFGR write protection, CIR flags with their interrupt enables, BDCR (DBP protection, RTCSEL write once, BDRST), CSR, HSE failure with CSS, and the PWR block (PWR_CR / PWR_CSR, VOSRDY, BRR, PVDO against a VDD knob, PWRRST through APB1RSTR). `HalPeripherals.c` got a FLASH_ACR latency-stuck knob. A lead probe of the firmware clock configuration (HSE 8 MHz, M 4, N 168, P 2) gave SYSCLK 168 MHz, PCLK1 42 MHz, PCLK2 84 MHz through the real HAL.
- **Tests (`claude/cases_hal/`):** `test_hal_rcc_osc.cpp` 87, `test_hal_rcc_clock.cpp` 85, `test_hal_rcc_ex.cpp` 49, `test_hal_pwr.cpp` 81 (302 new): 527 cases in each executable, 0 failed; 14 expected-fail assertions inside the `KNOWN_ISSUE` cases (10 from T1, V91, V92, V93).
- **Reviews:** two reviewers, 62 and 78 mutants. RCC: acceptable with fixes (rejected-call assertions that could not tell `HAL_ERROR` from `HAL_TIMEOUT`, timeouts without an upper bound, the SYSCLK type gate, `GetSysClockFreq` with SWS = 3); model fixes for CIR (xxRDYF needs xxRDYIE) and the CSS-off HSE failure. RCC_Ex / PWR: accepted with fixes (second `ConfigPVD` sequences, `DisableWakeUpPin` clobbering write, vacuous `DisableBkUpReg` case, running PLLI2S reconfiguration, RTCPRE field, `RCC_PERIPHCLK_PLLI2S`, CWUF / CSBF). All fixes landed; the fixes were not re-mutated. Survivors left: equivalent ones (`BDRST` release, `FLatency >=`), model protections the HAL never triggers (HSEON cleared in use, PLLCFGR written with the PLL on, RDYC, RTCSEL write-once and DBP gating of the model), a timeout constant without a measurable bound.
- **Gate (lead-run, `../ffb-test-build/`):**

  | Build | Result |
  |---|---|
  | `p12t1-gate` (32-bit + HAL) | ctest 14 / 14, `check_baseline.py` OK; `ffb_hal_tests` and `ffb_hal_tests_disco` 527 / 527, 3306 of 3320 assertions (14 expected) |
  | `p12t1-default` (option off) | ctest 11 / 11, `check_baseline.py` OK |
  | `p12t1-32san` (32-bit + HAL + sanitizers) | HAL executables 527 / 527 each, no sanitizer report; the full ctest still aborts in `ffb_flash_tests` (fixed mapping at 0x1FFF0000 against the i386 ASan layout, see T1) |
  | no warning in the build logs | |

- **Coverage (gcov, `p12t1-cov`, identical for both targets):** `stm32f4xx_hal_rcc.c` 230 / 240 (weak DeInit stub overridden by the Ex file, the weak CSS callback, two timeout branches, one error return, one `GetOscConfig` branch), `stm32f4xx_hal_rcc_ex.c` 112 / 128 (timeout branches of DeInit, the DBP wait timeout, the PLLI2S wait timeout, the `GetPeriphCLKFreq` default case), `stm32f4xx_hal_pwr.c` 83 / 85 (weak callback), `stm32f4xx_hal_pwr_ex.c` 33 / 33. With T1: 7 of 27 HAL files measured, 0 for the other 20 (T3 onwards).
- **Findings:** vendor V91 (Sleep keeps the SysTick running, against the UM1725 note), V92 (`HAL_RCC_DeInit` clears LSION, against the UM1725 note), V93 (arguable: `HAL_RCC_ClockConfig` rejects a source that is not ready, UM1725 note says the switch happens when it is); NEEDS REVIEW 288-300.
- **Charter changes:** PW7 (CWUF in Standby) corrected: neither UM1725 nor RM0090 requires it, RM0090 Table 31 makes "WUF cleared" an application precondition. The charter's PLLSAI / `SelectLSEMode` / over-drive items are not compiled for the F407 and are not tested.
- **Limits:** clock speeds, jitter and start-up times are not modelled (ready flags follow the HAL tick); no RCC / PWR interrupts, no wake-up from Stop / Standby, no clock gating; not run with clang, MinGW, macOS or on ARM.

## T3 result (2026-11-11, gated, not yet committed)

DMA / DMA_Ex of both targets. Charter DI1-DI9, DS1-DS9, DQ1-DQ8, DX1-DX2, DC1-DC4 (added after gcov showed the callback registration API is compiled unconditionally in this HAL version). GCC 16, 32-bit, out of tree.

- **Platform (lead):** `fakes/stm32_hal/HalDma.{h,c}`: DMA1 and DMA2 (LISR / HISR / LIFCR / HIFCR, eight streams each) with RM0090 10.5 write protection while EN = 1 (counter `hostDmaProtectedWrites`), the forced bits at EN = 1, the double-buffer M0AR / M1AR rule with TEIF, the FIFO / burst rule of Table 49, TCIF on a software stop (10.3.13, 10.3.14), knobs for a stop delay, a stream whose EN never clears, and a completion after N status reads, and hooks that stand in for the transfer engine (`hostDmaRaise`, `hostDmaComplete`, `hostDmaStepItems`). No data is moved. A lead probe (`HAL_DMA_Init`, `Start`, `PollForTransfer` through the model) passed; the DMA reset state was first not wired into the fixture reset (found by a writer, fixed).
- **Tests (`claude/cases_hal/`):** `test_hal_dma_init.cpp` 47, `test_hal_dma_xfer.cpp` 97, `test_hal_dma_irq.cpp` 64 (208 new): 735 cases in each executable, 0 failed; 23 expected-fail assertions inside the `KNOWN_ISSUE` cases (14 before T3, V94, V95, V96).
- **Review:** one reviewer, about 100 mutants of the DMA HAL files and the model: accepted with fixes (a characterization case that pinned the V94 behaviour as correct, the V95 assertion order, the new defect V96, plain cases for return codes UM1725 does not give, a vacuous DI5 case, and about ten surviving mutants: `DBM` kept by a plain Start, flag clearing in `MultiBufferStart_IT`, NULL callbacks, the transfer-error wait of the IRQ handler, flag and enable side effects of poll and abort). All fixes landed; they were not re-mutated. Model fidelity checked against RM0090 (flag layout, mask `0x0F7D0F7D`, `SxCR` writable mask `0x0FEFFFFF`, `SxFCR` 0x21, Table 49: 23 allowed and 25 forbidden combinations).
- **Gate (lead-run, `../ffb-test-build/`):**

  | Build | Result |
  |---|---|
  | `p12t1-gate` (32-bit + HAL) | ctest 14 / 14, `check_baseline.py` OK; `ffb_hal_tests` and `ffb_hal_tests_disco` 735 / 735, 12163 of 12186 assertions (23 expected) |
  | `p12t1-default` (option off) | ctest 11 / 11, `check_baseline.py` OK |
  | `p12t1-32san` (32-bit + HAL + sanitizers) | HAL executables 735 / 735 each, no sanitizer report; the full ctest still aborts in `ffb_flash_tests` (see T1) |
  | no warning in the build logs | |

- **Coverage (gcov, `p12t1-cov`, identical for both targets):** `stm32f4xx_hal_dma.c` 355 / 361 (the three unreachable `default:` branches of the static helpers), `stm32f4xx_hal_dma_ex.c` 55 / 55. With T1 and T2: 9 of 27 HAL files measured.
- **Findings:** vendor V94 (DeInit does not wait for EN = 0), V95 (poll timeout leaves the stream enabled; weaker, UM1725 gives no state), V96 (`HAL_DMAEx_MultiBufferStart` keeps the lock on `HAL_BUSY`); NEEDS REVIEW 301-306.
- **Charter changes:** DS2 (`Start_IT` does not set FEIE; neither oracle requires it), DS7 (FE / DME during polling do not end it; only TE does, RM0090 10.3.18), DQ7 (`Abort_IT` has no timeout; completion is the TCIF raised by the stop), DI7 (DeInit has no timeout; no `HAL_TIMEOUT` expected) are characterized, not asserted as the charter draft said; the callback registration API is compiled (group DC).
- **Limits:** no data movement, request timing, FIFO fill level, bus errors or interrupt delivery; the 3 tick stop delay behind V94 is an assumption; not run with clang, MinGW, macOS or on ARM.

## 3. Design

### 3.1 Why only the 32-bit build

The HAL casts pointers to `uint32_t` (for example `HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DR, ...)` in `stm32f4xx_hal_spi.c:1709`). On the 64-bit host this truncates every buffer address. P18's `-m32` build (ILP32) keeps them exact. So `ffb_hal_tests` is built only when `FFB_TESTS_32BIT=ON` and `FFB_TESTS_REAL_HAL=ON` (default OFF, configure error otherwise). This is a deliberate limit: no MinGW, macOS or 64-bit run.

### 3.2 Peripheral and core registers at their real addresses

The HAL writes registers through fixed addresses (`RCC->AHB1ENR`, `GPIOA`, `SCB`, `NVIC`, `SysTick`; `PERIPH_BASE` 0x40000000, `SCS_BASE` 0xE000E000). Redefining those macros would mean editing or copying the vendor device header, which is not allowed. Instead a new fake maps RAM at the real address ranges with `mmap(MAP_FIXED_NOREPLACE)`, like `platform/src/host_flash.cpp` already does for the flash. A 32-bit process on the 64-bit Linux kernel can map 0xE000E000. Scratch check (2026-11-11, GCC 16 `-m32`, scratchpad only): `MAP_FIXED_NOREPLACE` at 0x40000000 (APB/AHB1 peripherals), 0xE000E000 (system control space) and 0x50000000 (USB OTG FS) all succeeded and were writable. T0 repeats this inside the real build before anything else is written.

### 3.3 Register fakes are strict, not plain RAM

Plain RAM does not behave like a peripheral (write-1-to-clear flags, read-to-clear status, data registers backed by FIFOs, bits set by hardware). Per peripheral a fake under `fakes/stm32_regs/` observes the mapped block and applies the RM0090 behaviour the tests need, with the usual fidelity header (models, source chapter of RM0090 Rev 22, not modelled, confidence). The fake fails the test on register writes the hardware would reject (reserved bits, writes to read-only registers) where the fake can see them. A test that only checks "the HAL wrote value X to register Y" is labelled as checking the request, not the hardware effect.

### 3.4 Simulated time and polling loops

`HAL_GetTick`, `HAL_IncTick` and `HAL_Delay` are `__weak` in `stm32f4xx_hal.c`, so the test platform provides strong versions (in the new HAL test platform, not in the firmware). Each `HAL_GetTick` call advances the simulated time by one step and runs the peripheral fakes. That is how polling loops with timeouts (`HAL_SPI_Transmit`, `HAL_RCC_OscConfig`, ...) see the hardware react or time out without sleeping. A loop with no tick call that the fakes never release is caught by a ctest `TIMEOUT` and reported as a hang, never "fixed".

### 3.5 Interrupts and DMA

No real interrupts. A test calls the IRQ handler (`HAL_SPI_IRQHandler(&h)`) after making a fake raise the flag, which is how the hardware would enter it. DMA transfers are performed by a DMA fake (RM0090 chapter 10) when the stream is enabled, moving bytes between the 32-bit addresses the HAL wrote into `SxPAR` / `SxM0AR`.

### 3.6 Oracle

Expected values come from documents, never from the HAL source:
- RM0090 Rev 22 (`dm00031020-...pdf`), datasheet `DS_dm00037051.pdf`, errata ES0182 (`es0182-...pdf`), all present in `Tests/Code_Coverage/shared/specs/`.
- The HAL API contract (return codes, state machine `HAL_xxx_StateTypeDef`, error codes, callback order) is defined in ST's UM1725 "Description of STM32F4 HAL and low-layer drivers", supplied 2026-11-11 as `um1725-description-of-stm32f4-hal-and-lowlayer-drivers-stmicroelectronics.pdf` (Rev 8, 22-Mar-2023). The owner notes ST still points to UM1725 as the STM32F4 HAL documentation in 2025-2026, so Rev 8 is the current manual. The checked-in HAL is V1.8.3; where UM1725 Rev 8 and the source disagree (for example a function the manual does not describe), the case is `characterization:` with a NEEDS REVIEW entry. Expectations from the HAL's own doxygen comments alone are `characterization:` too.
- ST's upstream V1.8.3 source (`Tests/Code_Coverage/shared/specs/stm32f4xx-hal-driver-1.8.3/`, git-ignored in `Tests/.gitignore`, never committed) is the code under test, not an oracle. It is used to prove the checked-in copy is unmodified and, through `Release_Notes.html`, to confirm the version and ST's listed changes and limitations.

### 3.7 Separate executable and coverage tier

`ffb_hal_tests` (F407VG) and `ffb_hal_tests_disco` (F407VG_DISCO) link the real HAL plus the new fakes only, from the same test files. It never links `platform/` HAL fakes or firmware sources, so no existing executable changes. Coverage is a new **tier D (ST HAL)** in `COVERAGE_AND_SCOPE.md` with its own denominator, never added to the firmware ladder.

## 4. Work items (one group per stage; stop at the end of every stage)

| Stage | Content | Agents |
|---|---|---|
| T0 trial | CMake option, `-m32` real-HAL compile of all 27 files for both targets, `mmap` of the register ranges, one GPIO smoke case in each executable, the F407VG / DISCO identity check. Report what compiles and runs. | lead only |
| T1 core | `stm32f4xx_hal.c` (init, tick, delay, version, `HAL_DBGMCU`), CORTEX (NVIC priority grouping, SysTick config), GPIO, EXTI | 1-2 writers, 1 reviewer |
| T2 clocks and power | RCC / RCC_Ex (oscillator and PLL configuration with ready flags, clock frequencies from RM0090 6.2), PWR / PWR_Ex | writers, reviewer |
| T3 DMA | DMA / DMA_Ex fake and tests (stream state machine, transfer complete / half / error flags, abort) | writers, reviewer |
| T4 serial buses | SPI, I2C / I2C_Ex, UART: blocking, IT and DMA modes, timeouts, errors, abort | writers, reviewer (largest stage, may split) |
| T5 CAN | bxCAN: init modes, filters (RM0090 32.7.4), mailboxes, FIFOs, error states | writers, reviewer |
| T6 timers and ADC | TIM / TIM_Ex (base, PWM, encoder, input capture as the firmware uses them), ADC / ADC_Ex / LL ADC | writers, reviewer |
| T7 flash and watchdog | FLASH / FLASH_Ex (erase, program, option bytes, errors on a fake with NOR semantics), RAMFUNC, IWDG | writers, reviewer |
| T8 USB | PCD / PCD_Ex / LL USB against an OTG_FS register fake. Kept apart from the TinyUSB `ffb_usb_tests` (that suite fakes the DCD, so it does not count as PCD coverage) | writers, reviewer |
| T9 close | coverage tier D (per target), regression of every existing variant, documents (`USE_FULL_ASSERT` off throughout, question 68) | lead |

Order follows dependencies (tick and RCC before buses, DMA before DMA modes). Each stage: lead writes the requirements charter from the documents, writers implement, reviewer mutates scratch copies under `../ffb-test-build/audit/`, gate (32-bit build with the option on, plus every existing variant unchanged, `check_baseline.py`), then stop for "approve ... and commit".

## 5. Limits (stated in every report)

- i386 is not ARM: no `LDREX`/`STREX` semantics, no real interrupt preemption, no bus faults, no cycle timing. `__disable_irq` and friends are host stubs.
- A register fake is a model of RM0090, not the silicon. Errata in ES0182 are modelled only where a test cites them.
- Linux only (fixed-address `mmap`, `-m32` multilib).

## 6. Owner's decisions (questions 65-69)

65. ~~**Scope in `CLAUDE.md` / `AGENTS.md`.**~~ **Answered 2026-11-11: yes, both updated.** Both say "Not tested: ST HAL ... `Firmware/Targets/**`". P12 needs a line like "P12 (added 2026-11-11): the F407VG ST HAL (`Firmware/Targets/F407VG/Drivers/STM32F4xx_HAL_Driver/`) is IN scope, executable `ffb_hal_tests`, 32-bit build only; CMSIS core and CubeMX code stay out". These are your files: shall Claude add it?
66. ~~**Targets.**~~ **Answered 2026-11-11: F407VG only; revised the same day: F407VG and F407VG_DISCO** (F411RE out). Recommendation: F407VG only (the sources are identical for all three; F411RE differs only in the device header and the missing CAN file). Alternative: also build the HAL against `STM32F411xE`.
67. ~~**HAL user manual UM1725.**~~ **Answered 2026-11-11: supplied (Rev 8);** ST's V1.8.3 source supplied too (`Tests/Code_Coverage/shared/specs/stm32f4xx-hal-driver-1.8.3/`). Please supply it under `Tests/Code_Coverage/shared/specs/` (Claude does not download). Without it, HAL API expectations not derivable from RM0090 are `characterization:` and NEEDS REVIEW.
68. ~~**`assert_param`.**~~ **Answered 2026-11-11: keep it off.** The firmware builds with `USE_FULL_ASSERT` off. Recommendation: test with it off (the shipped behaviour); a second build with it on is optional extra work.
69. ~~**Linux-only executable.**~~ **Answered 2026-11-11: accepted, 32-bit build only.** `ffb_hal_tests` cannot run on MinGW, macOS or 64-bit builds (section 3.1, 3.2). Accept that, behind a default-OFF option?

## 7. Not in P12

Firmware sources with the real HAL instead of the fakes (an integration step, possible after P12), ARM hardware, CMSIS core, the FreeRTOS kernel, CubeMX code, startup code. Codex's P12 files are not used, changed or removed.
