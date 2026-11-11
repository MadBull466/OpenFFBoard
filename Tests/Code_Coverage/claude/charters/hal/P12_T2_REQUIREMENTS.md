# P12 T2 requirements charter: RCC / RCC_Ex and PWR / PWR_Ex (real ST HAL, F407VG and F407VG_DISCO)

Status: 2026-11-11 (lead). Platform done (`fakes/stm32_hal/HalClockPower.{h,c}`, lead probe: the firmware clock configuration HSE 8 MHz / M4 N168 P2 gives SYSCLK 168 MHz, PCLK1 42 MHz, PCLK2 84 MHz). Tests: four writers, two reviewers, all fixes landed, gated 2026-11-11 (see the plan, section "T2 result"). Plan: `Tests/Code_Coverage/claude/plans/P12_ST_HAL_PLAN.md` (stage T2).
Platform and rules as in `P12_T1_REQUIREMENTS.md` (trapping bus, fixture, `KNOWN_ISSUE`, `characterization:`, no sleeping,
expected values never from the HAL source).

## Oracles

| Topic | Oracle |
|---|---|
| API semantics, parameters, notes | UM1725 Rev 8 (chapter "RCC", "RCCEx", "PWR", "PWREx") |
| RCC registers, bits, reset values, formulas | RM0090 Rev 22, 7.3.1-7.3.23 (F405/407), 7.2 (clocks), 7.1 (reset) |
| PWR registers | RM0090 Rev 22, 5.4.1 (PWR_CR 0x00004000), 5.4.2 (PWR_CSR 0) |
| Flash latency (wait states) | RM0090 Rev 22, 3.4.1 (Table 10 of the F405/407 chapter) |
| SCB.SCR (SLEEPDEEP, SLEEPONEXIT, SEVONPEND) | PM0214 Rev 10 |
| Maximum frequencies | RM0090 / datasheet: SYSCLK and HCLK 168 MHz, PCLK2 84 MHz, PCLK1 42 MHz |

Where UM1725 says nothing (return codes of failing calls, timeouts, side effects such as GPIO configuration inside
`HAL_RCC_MCOConfig`), the case is a `characterization:` case and gets a NEEDS REVIEW entry (numbers from 288).

## Compiled code (F407xx variant, checked in the HAL sources)

`stm32f4xx_hal_rcc.c`, the `STM32F405/407/415/417` branches of `stm32f4xx_hal_rcc_ex.c` (`HAL_RCCEx_PeriphCLKConfig`
for RTC and I2S, `GetPeriphCLKConfig`, `GetPeriphCLKFreq`, `EnablePLLI2S` / `DisablePLLI2S`, `HAL_RCC_GetSysClockFreq`
of the Ex file where it applies), `stm32f4xx_hal_pwr.c` and the F405/407 branch of `stm32f4xx_hal_pwr_ex.c`
(`EnableBkUpReg`, `DisableBkUpReg`, `Enable/DisableFlashPowerDown`, `GetVoltageRange`, `ControlVoltageScaling`).
Not compiled for this device: PLLSAI, `SelectLSEMode`, over-drive, under-drive, low-voltage regulators. They are not
tested; gcov shows exactly which lines are absent.
Target constants (`stm32f4xx_hal_conf.h`): `HSE_VALUE` 8 MHz, `HSI_VALUE` 16 MHz, `HSE_STARTUP_TIMEOUT` 100 ms,
`LSE_STARTUP_TIMEOUT`, `TICK_INT_PRIORITY` 4. The firmware uses HSE 8 MHz, PLLM 4, PLLN 168, PLLP 2, PLLQ 7.

## Platform work (lead, before the writers)

1. `HalRcc` model replacing the storage-only RCC model of `HalPeripherals`:
   - CR: HSION/HSIRDY, HSEON/HSERDY, HSEBYP only writable with HSEON = 0 (RM0090 7.3.1), PLLON/PLLRDY, PLLI2SON/PLLI2SRDY,
     CSSON; HSION, HSEON, PLLON cannot be cleared while they feed SYSCLK (SWS); ready flags follow ON after a configurable
     number of simulated ms (default 1) and clear immediately when the ON bit is cleared (RM0090: 6 cycles, not modelled).
   - Knobs for tests: `hostRccSetStartup(osc, ms)`, `hostRccFail(osc)` (the oscillator never gets ready; used for the
     `HAL_TIMEOUT` paths), `hostRccHseFailure()` (CSS event: CSSF, HSE off, SW forced to HSI, NMI pending flag).
   - PLLCFGR: writable only with PLLON = 0 and PLLI2SON = 0 (fields PLLM, PLLN, PLLP, PLLQ, PLLSRC); a write while
     the PLL is on is ignored (RM0090 7.3.2 "can be written only when ..."); reset value 0x24003010.
   - CFGR: SW/SWS (SWS follows SW once the source is ready; until then it keeps the old value, UM1725 note of
     `HAL_RCC_ClockConfig`), HPRE, PPRE1, PPRE2, MCO1/2, MCO prescalers, RTCPRE, I2SSRC.
   - CIR: ready flags (RDYF) set when a ready flag rises, ready interrupt enables, `xxRDYC` clear bits (write 1 to clear),
     CSSF / CSSC.
   - BDCR and CSR: write protected until PWR_CR.DBP is set (RM0090 7.3.20), LSEON/LSERDY, LSEBYP, RTCSEL (writable once
     until BDRST), RTCEN, BDRST; CSR LSION/LSIRDY, RMVF clears the reset flags.
   - PLLI2SCFGR (reset 0x20003000), the AHBxENR / APBxENR / RSTR registers keep their T1 storage behaviour.
   - Flash latency: `FLASH_ACR` LATENCY field stays as in T1 (readback is real: RM0090 3.4: "the new value is
     effective once the LATENCY bits read back the written value").
2. `HalPwr` model (RM0090 5.4): PWR_CR (reset 0x4000; CSBF and CWUF read 0 and clear SBF / WUF; DBP; PLS; PVDE; FPDS;
   PDDS; LPDS; VOS), PWR_CSR (VOSRDY follows VOS, BRR follows BRE after a delay, PVDO driven by a test knob
   `hostPwrSetVdd(mV)` against the PLS threshold when PVDE = 1, otherwise 0; SBF, WUF settable by knobs; BRE and EWUP; BRE
   writable only with DBP = 1).
3. SCB.SCR is already modelled (T1). `__WFI` / `__WFE` counters exist; add a hook that lets a test observe SCR (SLEEPDEEP)
   and PWR_CR at the moment of the `__WFI`.
4. `HAL_GetTick` host replacement unchanged. `HAL_InitTick` is called by `HAL_RCC_ClockConfig` (UM1725 says the tick
   is re-initialised); the fixture sets a known uwTickPrio.

## Requirement groups

### RC: RCC core (UM1725 54.2, RM0090 7.3)

| ID | Requirement | Oracle |
|---|---|---|
| RC1 | After reset the model has the RM0090 reset values (CR 0x83, PLLCFGR 0x24003010, CFGR 0, CIR 0, CSR 0x0E000000, PLLI2SCFGR 0x20003000); `HAL_RCC_GetSysClockFreq` returns `HSI_VALUE` | RM0090 7.3, UM1725 |
| RC2 | `HAL_RCC_DeInit`: HSI on and selected, HSE and PLL off, prescalers 1, CSS and MCO off, interrupts disabled, returns HAL_OK; PLLCFGR returns to its reset value; peripheral clock enables, LSI, LSE and RTC untouched; `SystemCoreClock` equals HSI_VALUE (the last is `characterization:`) | UM1725 DeInit notes |
| RC3 | `HAL_RCC_OscConfig` HSE: ON sets HSEON and returns HAL_OK when HSERDY comes; bypass sets HSEBYP and HSEON; OFF clears both; an oscillator that never gets ready gives a timeout (`HAL_TIMEOUT`, characterization of the code); HSE ON to bypass or bypass to ON without OFF is not supported (UM1725 note): the bit is not changed | UM1725 notes, RM0090 7.3.1 |
| RC4 | HSE cannot be switched off while it is SYSCLK or the PLL source of SYSCLK: the call does not clear HSEON and does not hang (return code is `characterization:`) | RM0090 7.3.1 |
| RC5 | HSI: ON, OFF (HSIRDY clears), calibration trim written to HSITRIM[7:3], cannot be switched off while SYSCLK | RM0090 7.3.1 |
| RC6 | LSI: ON / OFF with LSIRDY (CSR); timeout when never ready | RM0090 7.3.21 |
| RC7 | LSE: ON / bypass / OFF writes BDCR; the HAL sets DBP first (PWR_CR.DBP is 1 afterwards); timeout; ON to bypass not supported (UM1725) | RM0090 7.3.20, 5.4.1 |
| RC8 | PLL: with PLLON = 0, `OscConfig` writes PLLM / PLLN / PLLP / PLLQ / PLLSRC to PLLCFGR (exact field positions), sets PLLON, waits for PLLRDY, returns HAL_OK; PLL OFF clears PLLON and waits for PLLRDY to clear; the firmware configuration (HSE 8 MHz, 4, 168, 2, 7) gives VCO 336 MHz, PLLCLK 168 MHz, PLLQ clock 48 MHz | RM0090 7.3.2 formulas |
| RC9 | The PLL is not disabled or reconfigured while it is SYSCLK (UM1725 note): the call returns an error and PLLCFGR / PLLON are unchanged; PLL update while HSE is used for PLLSRC and SYSCLK is the PLL | UM1725, RM0090 7.3.1-7.3.2 |
| RC10 | `HAL_RCC_ClockConfig`: FLASH latency is written before HCLK goes up and after it goes down (RM0090 3.4: the new latency must be effective first when increasing), SW is written, SWS is polled until it equals the selected source (timeout when the source never gets ready), HPRE / PPRE1 / PPRE2 fields are written exactly, `SystemCoreClock` equals the new HCLK, the tick is re-initialised, returns HAL_OK; a latency that does not read back gives an error | UM1725 notes, RM0090 3.4, 7.3.3 |
| RC11 | Frequencies: `GetSysClockFreq` for HSI, HSE and PLL (HSI or HSE source, M, N, P) equals the RM0090 formula for the whole legal range of M (2-63), N (50-432), P (2,4,6,8); `GetHCLKFreq` for all 9 HPRE codes (1,2,4,...,512, RM0090 7.3.3), `GetPCLK1Freq` / `GetPCLK2Freq` for the 5 PPRE codes; the DISCO and F407VG results are identical | RM0090 7.2, 7.3.2, 7.3.3 |
| RC12 | `HAL_RCC_GetOscConfig` and `GetClockConfig` return what the registers hold (round trip with `OscConfig` / `ClockConfig` for HSE, HSI, LSE, LSI, PLL, bypass states; latency from FLASH_ACR) | UM1725 |
| RC13 | `HAL_RCC_MCOConfig`: MCO1 / MCO2 source and prescaler bits at their positions (MCO1 22:21 and 26:24, MCO2 31:30 and 29:27; prescaler 0xx = 1, 100 = 2, ... 111 = 5), RM0090 codes for the sources; PA8 / PC9 configuration done by the HAL is a `characterization:` case | RM0090 7.3.3, UM1725 |
| RC14 | CSS: `EnableCSS` / `DisableCSS` set / clear CSSON; `HAL_RCC_NMI_IRQHandler` with CSSF set calls `HAL_RCC_CSSCallback` and writes CSSC (the flag clears); without CSSF neither happens | RM0090 7.3.1, 7.3.4, UM1725 |
| RC15 | Clock switch errors: `ClockConfig` with a latency, source or prescaler that cannot be reached is rejected as UM1725 allows; selecting a source that is not ready: the switch happens when it is ready (UM1725 note) | UM1725 |

### RX: RCC_Ex (UM1725 RCCEx)

| ID | Requirement | Oracle |
|---|---|---|
| RX1 | `HAL_RCCEx_PeriphCLKConfig` RTC: source LSE, LSI or HSE (with RTCPRE); changing the source resets the backup domain first (UM1725 note: BDCR returns to its reset value, then the new RTCSEL is set); LSE start waits for LSERDY with a timeout; the same source is not re-selected needlessly (`characterization:`) | UM1725, RM0090 7.3.20 |
| RX2 | I2S: I2SSRC bit for PLLI2S or the external clock | RM0090 7.3.3 |
| RX3 | `EnablePLLI2S`: PLLI2SN / PLLI2SR written (fields, exact positions), PLLI2SON, waits for PLLI2SRDY, HAL_OK; timeout when never ready; `DisablePLLI2S` clears PLLI2SON and waits for the flag to clear | RM0090 7.3.23, 7.3.1 |
| RX4 | `GetPeriphCLKConfig` returns the RTC and I2S selections; `GetPeriphCLKFreq` for I2S (PLLI2S or I2S_CKIN fixed value of the hal_conf) and RTC (LSE, LSI 32 kHz, HSE / RTCPRE) returns the RM0090 formula, 0 for an identifier it does not manage (UM1725 note) | UM1725, RM0090 |

### PW: PWR (UM1725 PWR, RM0090 5.4)

| ID | Requirement | Oracle |
|---|---|---|
| PW1 | `HAL_PWR_DeInit` resets the PWR block through APB1RSTR bit 28 (PWR reset), then releases it; PWR_CR returns to 0x4000 | RM0090 5.4.1, 7.3.16 |
| PW2 | `EnableBkUpAccess` / `DisableBkUpAccess` set / clear DBP (read back, the other PWR_CR bits are unchanged) | RM0090 5.4.1 |
| PW3 | `ConfigPVD`: PLS at bits 7:5 for all 8 levels (2.0 V ... 2.9 V); mode IT / EVT, rising / falling edge map to the EXTI line 16 registers (IMR / EMR / RTSR / FTSR) exactly as the 6 `PWR_MODE_*` values say; unrelated EXTI lines unchanged; `Enable/DisablePVD` set / clear PVDE | RM0090 5.4.1, 12.2 |
| PW4 | `PWR_PVD_IRQHandler`: with EXTI.PR bit 16 set calls `HAL_PWR_PVDCallback` and clears the flag (write 1); without it, nothing | RM0090 12.3, UM1725 |
| PW5 | `Enable/DisableWakeUpPin`: EWUP bit; the other CSR bits unchanged | RM0090 5.4.2 |
| PW6 | `EnterSLEEPMode`: SLEEPDEEP cleared, WFI or WFE executed once per call as `SLEEPEntry` says; the regulator argument has no effect | RM0090 5.3.4, PM0214 |
| PW7 | `EnterSTOPMode`: PDDS cleared, LPDS from the regulator argument (main regulator or low power), SLEEPDEEP set while waiting, WFI or WFE once, SLEEPDEEP cleared after; `EnterSTANDBYMode`: PDDS set, SLEEPDEEP set, WFI (the charter draft said CWUF written: neither UM1725 nor RM0090 requires it; RM0090 Table 31 lists "WUF cleared" as an entry condition the application must satisfy, the HAL does not clear it: `characterization:`) | RM0090 5.3.5-5.3.6 |
| PW8 | `Enable/DisableSleepOnExit`, `Enable/DisableSEVOnPend`: SCR bits 1 and 4 only | PM0214 |
| PW9 | `EnableBkUpReg`: BRE set with DBP = 1, waits for BRR, timeout when never ready, `DisableBkUpReg` clears BRE and waits for BRR to clear | RM0090 5.4.2 |
| PW10 | `Enable/DisableFlashPowerDown` set / clear FPDS; `GetVoltageRange` returns scale 1 or 2 from VOS (UM1725 constants) | RM0090 5.4.1 |
| PW11 | `ControlVoltageScaling`: VOS written, VOSRDY waited, timeout when never ready (a `characterization:` case) | RM0090 5.4.1-5.4.2 |

## Defect candidates to check (reserve vendor bug V91 onwards)

Look for, and only report with a test that states the intended behaviour as `KNOWN_ISSUE`: a return value of `HAL_OK` with a not
reached clock; the DISCO and F407VG results differing; `HAL_RCC_ClockConfig` with a rising HCLK writing the latency after the switch; a
`HAL_TIMEOUT` loop that never ends; reserved bits written in CFGR / CR.

## Writer rules

As in `P12_T1_REQUIREMENTS.md`. Test files (`claude/cases_hal/`): `test_hal_rcc_osc.cpp` (RC1-RC9), `test_hal_rcc_clock.cpp` (RC10-RC15),
`test_hal_rcc_ex.cpp` (RX), `test_hal_pwr.cpp` (PW). Writers never edit `fakes/stm32_hal/`; a missing model feature is reported to the lead. One behaviour per case, names start with the requirement id. Every `characterization:` case carries its
NEEDS REVIEW text in a comment. No test sleeps; time is `hostTickAdvance` / the replaced `HAL_GetTick`.
