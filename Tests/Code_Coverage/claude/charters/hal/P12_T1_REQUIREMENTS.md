# P12 T1 requirements: HAL core, CORTEX, GPIO, EXTI

Status: charter by the lead (2026-11-11), not yet approved. Plan: `Tests/Code_Coverage/claude/plans/P12_ST_HAL_PLAN.md`.
Executables: `ffb_hal_tests` (F407VG) and `ffb_hal_tests_disco` (F407VG_DISCO), same test files. Sources under test
(read-only): `Firmware/Targets/F407VG/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c`, `stm32f4xx_hal_gpio.c`,
`stm32f4xx_hal_exti.c` and `stm32f4xx_hal_cortex.c` (group C).

## Oracles

- **UM1725 Rev 8** (`Tests/Code_Coverage/shared/specs/um1725-...pdf`), chapters 6 (HAL system driver), 24 (EXTI), 31 (GPIO): API contract.
  It is thin: most functions are described in one sentence ("Sets or clears the selected data port bit").
- **RM0090 Rev 22**: register semantics and encodings.
  - 8.3.5 bitwise handling, 8.3.6 locking, 8.3.10 output, 8.3.12 analog, 8.4.1-8.4.10 GPIO registers;
  - 9.2.3-9.2.7 SYSCFG EXTICR and CMPCR;
  - 12.2.4 EXTI functional description, 12.2.5 line mapping, 12.3.1-12.3.6 EXTI registers;
  - 7.3.5-7.3.9 RCC reset registers, 7.3.10 / 7.3.14 clock enables;
  - 38.6.1 DBGMCU_IDCODE, 38.16.3 DBGMCU_CR;
  - 39.1 unique id.
- **ST release notes** of the upstream V1.8.3 source (`shared/specs/stm32f4xx-hal-driver-1.8.3/Release_Notes.html`): the version number only.
- The HAL source may be read to find the code paths (coverage), **never** to take an expected value. Where neither
  document fixes a value: `characterization:` plus a NEEDS REVIEW entry in `PROGRESS.md` (numbers from 281).

Values from the HAL header that a test needs as *input* (`GPIO_MODE_IT_RISING`, `EXTI_LINE_16`, `EXTI_GPIOC`) may be
used as inputs; expected register values are derived from RM0090 encodings, written in a comment.

## Test platform (lead, done before the writers start)

- `fakes/stm32_hal/HalRegisterSpace.{h,c}`: the trapping bus. Every register access goes through a model, and any
  access to a register without a model is a fault that fails the test.
- `fakes/stm32_hal/HalPeripherals.{h,c}`: models for GPIOA..I, EXTI, SYSCFG, RCC (storage only), FLASH_ACR, DBGMCU,
  and the unique id. The header lists what each one models and its confidence.
- `claude/cases_hal/HalFixture.h`: resets and arms before every case, and checks for faults after it.
  - Helpers: `peek` / `poke` (raw access), `lastWritten(reg)` (the raw word of the last write), `faults()` / `clearFaults()`.
  - Test control: `hostGpioSetInput`, `hostGpioPinLevel`, `hostGpioLocked`, `hostExtiSetLine`, `hostExtiEventCount`,
    `hostDbgSetIdcode`, `hostUidSet`.
  - Tick control: `hostTickSet`, `hostTickPeek`, `hostTickSetAutoAdvance`, `hostTickSetStep`.
- The RCC model stores values only. `HAL_DeInit`'s peripheral resets do not reset the GPIO / EXTI models, so a case
  checks the RCC writes (the request), not the peripheral effect.
- The system control space (NVIC, SCB, SysTick, MPU) is modelled in `fakes/stm32_hal/HalCorePeripherals.{h,c}` (PM0214); the bit-band alias of the peripherals (RM0090 2.3.3) is built into the bus.

## G: GPIO (`stm32f4xx_hal_gpio.c`; writer W1, file `test_hal_gpio.cpp`, at most ~45 cases)

Ports used in tests: prefer GPIOC..GPIOI (reset values 0), and use A / B where their reset values matter.
Enable the port clock (`__HAL_RCC_GPIOx_CLK_ENABLE()`) before `HAL_GPIO_Init`, as UM1725 31.2.2 step 1 says.

| Id | Requirement | Oracle |
|---|---|---|
| G1 | `HAL_GPIO_Init` output push-pull / open-drain: MODERy = 01, OTYPERy = 0 / 1, OSPEEDRy = the speed, PUPDRy = the pull; the bits of the other pins keep their values. Do it for a low (0-7) and a high (8-15) pin, and for all four speeds and three pulls. | UM1725 31.2.2 step 2, RM0090 8.4.1-8.4.4 |
| G2 | Alternate function (push-pull / open-drain): MODERy = 10, the AF number in the AFRL nibble (pin < 8) or AFRH nibble (pin >= 8), the other nibbles unchanged. | RM0090 8.4.9, 8.4.10 |
| G3 | Input: MODERy = 00, PUPDRy = the pull, OSPEEDRy and OTYPERy unchanged. | RM0090 8.4.1 |
| G4 | Analog: MODERy = 11; `HAL_GPIO_ReadPin` then returns `GPIO_PIN_RESET` even with the pin driven high. | RM0090 8.3.12 |
| G5 | EXTI modes (`GPIO_MODE_IT_RISING`, `_FALLING`, `_RISING_FALLING`, `GPIO_MODE_EVT_*`): the SYSCFG clock is enabled (RCC_APB2ENR.SYSCFGEN, bit 14); SYSCFG_EXTICR selects the port (0000 = A ... 1000 = I) in the nibble of the line; RTSR / FTSR / IMR / EMR have the line bit as the mode asks and other lines unchanged. A selected edge on the pin then sets EXTI_PR (interrupt modes) or counts an event (event modes). | UM1725 31.2.1, RM0090 9.2.3-9.2.6, 12.3.1-12.3.4, 7.3.14 |
| G6 | Several pins in one `Init` call (and `GPIO_PIN_All`): every selected pin is configured, none of the others. | UM1725 GPIO_Init "the specified parameters" |
| G7 | `HAL_GPIO_DeInit` restores the RM0090 reset values of the pin's MODER, OTYPER, OSPEEDR, PUPDR and AFR bits. Debug pins (PA13, PA14, PA15, PB3, PB4) have non-zero reset values; candidate vendor bug **V89**, a `KNOWN_ISSUE`: the HAL clears them to 0. UM1725 says "to their default reset values"; RM0090 gives the values. The first assertion must be the MODER value. | UM1725 HAL_GPIO_DeInit, RM0090 8.4.1-8.4.4 |
| G8 | `HAL_GPIO_DeInit` of an EXTI pin clears IMR / EMR / RTSR / FTSR of that line and the EXTICR nibble, **only** when the EXTICR nibble selects this port; when another port owns the line, the EXTI configuration stays. | UM1725 DeInit, RM0090 9.2.3 (one port per line) |
| G9 | `HAL_GPIO_ReadPin`: input driven high / low; input floating with pull-up reads SET and with pull-down RESET; an output push-pull pin reads its ODR level. | RM0090 8.3.9, 8.3.10, 8.4.5 |
| G10 | `HAL_GPIO_WritePin` with several pins: ODR bits of exactly those pins set / reset; others unchanged. | RM0090 8.3.5 |
| G11 | `HAL_GPIO_TogglePin`: every selected pin changes its ODR level, the others keep theirs (mixed 0 / 1 start values, several pins). | UM1725 "Toggles the specified GPIO pins" |
| G12 | `HAL_GPIO_LockPin` returns `HAL_OK`, `hostGpioLocked` is 1 for the pins, and a later `HAL_GPIO_Init` of a locked pin leaves its MODER / OTYPER / OSPEEDR / PUPDR / AFR bits unchanged while an unlocked pin of the same port changes. | UM1725 LockPin notes, RM0090 8.4.8 |
| G13 | `characterization:` `HAL_GPIO_LockPin` on a port that is already locked returns `HAL_OK` although the new pins stay unlocked. RM0090: LCKR is locked until reset. UM1725 gives no error case (its "Return values: None" contradicts the prototype). NEEDS REVIEW. | RM0090 8.4.8 |
| G14 | `HAL_GPIO_EXTI_IRQHandler(pin)`: with the PR bit set it clears PR (write 1) and calls `HAL_GPIO_EXTI_Callback(pin)` once; with PR clear it calls nothing and leaves PR. Define the strong callback (a recorder) **only in this file**. | UM1725 31.2.4, RM0090 12.3.6 |

## E: EXTI (`stm32f4xx_hal_exti.c`; writer W2, file `test_hal_exti.cpp`, at most ~30 cases)

| Id | Requirement | Oracle |
|---|---|---|
| E1 | `HAL_EXTI_SetConfigLine` / `GetConfigLine` with a NULL handle or NULL config return `HAL_ERROR` and access no register; `ClearConfigLine(NULL)` and `GetHandle(NULL, line)` return `HAL_ERROR`. | UM1725 24.2.4 (HAL status) |
| E2 | `SetConfigLine` for a GPIO line (`EXTI_LINE_0..15`): Trigger rising / falling / both / none sets RTSR and FTSR; Mode interrupt / event / both / none sets IMR and EMR; GPIOSel sets the EXTICR nibble; other lines unchanged; the handle stores the line; returns `HAL_OK`. | UM1725 24.2.2, RM0090 12.3.1-12.3.4, 9.2.3 |
| E3 | `SetConfigLine` for a non-GPIO line (`EXTI_LINE_16..22`): RTSR / FTSR / IMR / EMR as in E2; no SYSCFG register is written. | RM0090 12.2.5 (lines 16-22 are internal) |
| E4 | `GetConfigLine` returns what `SetConfigLine` set (Mode, Trigger, GPIOSel) for GPIO and non-GPIO lines, and reads register values set with `poke`. | UM1725 24.2.2 step 2 |
| E5 | `ClearConfigLine` clears the line's IMR / EMR / RTSR / FTSR bits and its EXTICR nibble; other lines and nibbles unchanged. | UM1725 "Clear whole configuration of a dedicated Exti line" |
| E6 | `RegisterCallback` with `HAL_EXTI_COMMON_CB_ID` returns `HAL_OK` and stores the function; any other id returns `HAL_ERROR`. `GetHandle` stores the line and returns `HAL_OK`. | UM1725 24.2.4 |
| E7 | `HAL_EXTI_IRQHandler`: line pending, so PR is cleared and the registered callback is called once; no callback registered, so PR is cleared and nothing crashes; line not pending, so no callback and PR is unchanged. Only the handle's line is cleared. | UM1725 24.2.4, RM0090 12.3.6 |
| E8 | `GetPending` returns 1 / 0 for the line's PR bit; `ClearPending` clears exactly that bit (rc_w1). | RM0090 12.3.6 |
| E9 | `GenerateSWI` with the line's IMR bit set makes the line pending (PR); with IMR clear, PR stays 0 (RM0090 12.3.5). | RM0090 12.3.5 |
| E10 | A line configured with `SetConfigLine` (GPIO line, rising) becomes pending on a rising edge of the selected pin (`hostGpioSetInput`) and not on a falling edge; a line 16-22 becomes pending through `hostExtiSetLine`. | RM0090 12.2.4 |

## H: HAL core (`stm32f4xx_hal.c`, parts without the core peripherals; writer W2, file `test_hal_core.cpp`, at most ~25 cases)

| Id | Requirement | Oracle |
|---|---|---|
| H1 | `HAL_GetHalVersion()` == 0x01080300: UM1725 "0xXYZR (8bits for each decimal, R for RC)" and the release notes' V1.8.3 (X=1, Y=8, Z=3, R=0). | UM1725 6.1.4, release notes |
| H2 | `HAL_GetDEVID()` returns IDCODE bits 11:0 and `HAL_GetREVID()` bits 31:16 (use two different IDCODE values, e.g. 0x10070413 and 0x100F0413). | RM0090 38.6.1 |
| H3 | DBGMCU enable / disable for SLEEP / STOP / STANDBY sets / clears DBGMCU_CR bit 0 / 1 / 2 and no other bit (start from a value with other bits set). | RM0090 38.16.3 |
| H4 | `HAL_EnableCompensationCell` sets SYSCFG_CMPCR.CMP_PD (bit 0); `HAL_DisableCompensationCell` clears it. | RM0090 9.2.7 |
| H5 | `HAL_GetUIDw0/1/2` return the words at 0x1FFF7A10 / 14 / 18 (`hostUidSet`). | RM0090 39.1 |
| H6 | `HAL_IncTick` adds `uwTickFreq` (in ms: 1, 10, 100 for 1 kHz / 100 Hz / 10 Hz) to the tick; `HAL_GetTickFreq` and `HAL_GetTickPrio` return the current values. Set `uwTickFreq` directly (`HAL_SetTickFreq` needs SysTick, group C). | UM1725 6.1.4 |
| H7 | `HAL_Delay(n)` returns only after at least n ms of simulated time, for n = 0, 1, 7 and with `uwTickFreq` 1 and 10 (`hostTickPeek` before and after). | UM1725 "minimum delay" |
| H8 | `KNOWN_ISSUE`: `HAL_Delay(n)` close to `HAL_MAX_DELAY` (e.g. 0xFFFFFFF0) with `uwTickFreq` 100 returns before n ms (the HAL adds `uwTickFreq` to n and the sum wraps). This is candidate vendor bug **V90**; the first assertion is "elapsed >= n". It must finish fast when it fails, and it must stay below the ctest timeout if a fixed HAL loops for the full n (with 100 ms per step, 4.3e7 steps). | UM1725 "minimum delay" |
| H9 | `HAL_DeInit()` returns `HAL_OK`. For each of RCC_APB1RSTR, APB2RSTR, AHB1RSTR, AHB2RSTR, AHB3RSTR it writes the register with 1s (force reset) and then 0 (release), in that order (the access log), so all end at 0. Request level: the RCC model does not reset peripherals. `characterization:` whether all-ones (including reserved bits) is correct, NEEDS REVIEW. | RM0090 7.3.5-7.3.9 |
| H10 | `HAL_MspInit` / `HAL_MspDeInit` (weak defaults) access no register (the access log is empty). | UM1725 6.1.4 |

Not compiled for STM32F407xx: `HAL_EnableMemorySwappingBank` / `HAL_DisableMemorySwappingBank` (STM32F42x/43x/469/479 only, UM1725 note). Record as "not compiled", not "untested".

## C: CORTEX and the SysTick-dependent HAL core (writer W3; PM0214 supplied 2026-11-11)

Oracles: **PM0214 Rev 10** (`shared/specs/pm0214-...pdf`; text extract `scratchpad/pm.txt`), chapter 4:
4.2.6-4.2.9 MPU, 4.3.2-4.3.8 NVIC, 4.4.5 AIRCR with Table 51 (priority grouping), 4.4.8 SHPR with Table 52,
4.4.9 SHCSR, 4.5.1-4.5.3 SysTick. Also UM1725 chapter 11 (`scratchpad/um_cortex.txt`) and chapter 6 (`um_sys.txt`),
and RM0090 Table 62 (IRQ numbers 0..81) and 3.9.1 (FLASH_ACR).

Model: `fakes/stm32_hal/HalCorePeripherals.h`; read its header for what is modelled. Test control: `hostNvicEnabled`,
`hostNvicPending`, `hostNvicSetActive`, `hostScbResetRequests`, `hostSysTickCountedToZero`. To leave the endless
`for(;;) __NOP();` after the reset request: set `hostCmsisNopHook` to a function that `longjmp`s back to a `setjmp`
in the test (plain C `jmp_buf`; no C++ objects with destructors between the two).

Files: `test_hal_cortex.cpp` (C1-C13, about 30-40 cases) and `test_hal_core_systick.cpp` (C14-C17, about 12-15 cases).
Define the strong `HAL_SYSTICK_Callback` recorder only in `test_hal_cortex.cpp`. Do not define `HAL_MspInit`,
`HAL_InitTick` or any other weak HAL function (W2's H10 relies on the defaults).

| Id | Requirement | Oracle |
|---|---|---|
| C1 | `HAL_NVIC_SetPriorityGrouping(g)` for each of `NVIC_PRIORITYGROUP_0..4` stores the PRIGROUP the group means (Table 51: group 4 = 4 group bits, 0b011; group 0 = 0 group bits, 0b111) using the key, so AIRCR reads 0xFA05 << 16 \| PRIGROUP << 8; `HAL_NVIC_GetPriorityGrouping` returns the group value. | UM1725 11 ("using the required unlock sequence"), PM0214 4.4.5 |
| C2 | `HAL_NVIC_SetPriority(irq, pre, sub)` writes the IPR byte of the interrupt as Table 51 splits bits 7:4 between group and subpriority, for groups 0, 2 and 4 at least, and for interrupts in IPR word 0 and in a high word (e.g. 6 EXTI0, 40 EXTI15_10, 81 FPU); other bytes unchanged. | PM0214 4.3.7, Table 51 |
| C3 | `HAL_NVIC_SetPriority` of system exceptions (SysTick_IRQn, PendSV_IRQn, MemoryManagement_IRQn, SVCall_IRQn) writes the PRI_n field of SHPR1-3 that Table 52 assigns. | PM0214 4.4.8 |
| C4 | `HAL_NVIC_GetPriority` returns the preemption and subpriority that were set (round trip for several groups and values). | UM1725 11 |
| C5 | `HAL_NVIC_EnableIRQ` / `DisableIRQ` set / clear only that interrupt's enable bit (ISER / ICER, words 0, 1, 2). | PM0214 4.3.2, 4.3.3 |
| C6 | `HAL_NVIC_SetPendingIRQ` / `GetPendingIRQ` / `ClearPendingIRQ` (ISPR / ICPR). | PM0214 4.3.4, 4.3.5 |
| C7 | `HAL_NVIC_GetActive` returns 1 / 0 from IABR (`hostNvicSetActive`). | PM0214 4.3.6 |
| C8 | `HAL_SYSTICK_Config(n)` for n in 1..0x1000000: LOAD = n - 1 (4.5.2, "a period of N processor clock cycles, use a RELOAD value of N-1"), VAL cleared, ENABLE and TICKINT set, returns 0; n = 0x1000001 (RELOAD would exceed 24 bits) returns non-zero and leaves SysTick unchanged. The processor clock source (CLKSOURCE) and the SysTick priority it sets are `characterization:` unless UM1725 states them. | UM1725 11, PM0214 4.5.1-4.5.3 |
| C9 | `HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK)` sets CLKSOURCE, `..._HCLK_DIV8` clears it, and other CTRL bits keep their values. | PM0214 4.5.1 (1 = processor clock, 0 = AHB/8) |
| C10 | `HAL_SYSTICK_IRQHandler` calls `HAL_SYSTICK_Callback` once. | UM1725 11 |
| C11 | MPU: `HAL_MPU_Disable` clears MPU_CTRL and SHCSR.MEMFAULTENA (bit 16); `HAL_MPU_Enable(ctrl)` sets CTRL = ctrl \| ENABLE and MEMFAULTENA; `EnableRegion` / `DisableRegion(n)` select region n (RNR) and set / clear RASR.ENABLE of that region only; `ConfigRegion` stores RBAR and the RASR fields at their PM0214 4.2.9 positions (XN 28, AP 26:24, TEX 21:19, S 18, C 17, B 16, SRD 15:8, SIZE 5:1, ENABLE 0) for region n, and other regions keep theirs. | PM0214 4.2.6-4.2.9, 4.4.9 |
| C12 | `HAL_CORTEX_ClearEvent` executes one SEV and then one WFE (`hostCmsisSevCount`, `hostCmsisWfeCount`). | UM1725 11, PM0214 2.5 (event register) |
| C13 | `HAL_NVIC_SystemReset` writes AIRCR once with the key and SYSRESETREQ (`hostScbResetRequests` == 1) and does not return (leave through the `__NOP` hook). Whether it keeps PRIGROUP is `characterization:`. | UM1725 11, PM0214 4.4.5 |
| C14 | `HAL_InitTick(prio)` (weak default) with prio < 16: SysTick reload for a 1 ms time base at `SystemCoreClock` (16 MHz after reset, so LOAD 15999; also another clock such as 168 MHz), SysTick priority = prio in SHPR3 PRI_15, `HAL_GetTickPrio` = prio, returns `HAL_OK`. prio >= 16: returns `HAL_ERROR`. | UM1725 6.1.2 and 6.1.4 (1 ms time base), PM0214 4.5.2 |
| C15 | `HAL_Init` returns `HAL_OK`; FLASH_ACR has PRFTEN, ICEN, DCEN (bits 8, 9, 10; the F407 `hal_conf` enables all three); SysTick is configured for 1 ms; `characterization:` the priority grouping and the tick priority it chooses. | UM1725 6.1.4, RM0090 3.9.1 |
| C16 | `HAL_SetTickFreq`: to 10 Hz at 16 MHz returns `HAL_OK`, `HAL_GetTickFreq` = 10 Hz and LOAD = 1 600 000 - 1. At a `SystemCoreClock` where the reload exceeds 24 bits (168 MHz at 10 Hz: 16 800 000 - 1 > 0xFFFFFF) it returns `HAL_ERROR` and the frequency stays the previous one. The same frequency again returns `HAL_OK` without a change. | UM1725 6.1.4, PM0214 4.5.2 |
| C17 | `HAL_SuspendTick` clears STK_CTRL.TICKINT and `HAL_ResumeTick` sets it; ENABLE unchanged. | UM1725 6.1.4 notes |

## Rules for the writers

- Follow `Firmware/Tests/README.md` and `CLAUDE.md`. Use doctest, `TEST_CASE_FIXTURE(HalFixture, "...")`, one
  behaviour per case, and a statement as the name. Never sleep.
- Each expected register value has a comment with its RM0090 / UM1725 section and, where needed, the bit derivation.
- Check the hardware effect through the models (ODR after BSRR, PR after SWIER, frozen bits after lock). Use
  `lastWritten` / the access log only where the requirement is the request (H9).
- A case may provoke a fault only on purpose; it then checks `faults()` and calls `clearFaults()`.
- Do not edit the fakes, the fixture, CMake or other writers' files. If a model is missing or wrong, stop and
  report it to the lead.
- Build in your own build directory: `cmake -S Firmware/Tests -B ../ffb-test-build/p12t1-<writer> -DFFB_TESTS_32BIT=ON -DFFB_TESTS_REAL_HAL=ON`,
  then `cmake --build <dir> --target ffb_hal_tests ffb_hal_tests_disco -j4`. Both executables must pass. A
  `KNOWN_ISSUE` case counts as passing; its first failing assertion must be the one that describes the defect.
- Report: the cases (name, requirement id), the characterization / KNOWN_ISSUE cases with their reason, and any model
  problem.
