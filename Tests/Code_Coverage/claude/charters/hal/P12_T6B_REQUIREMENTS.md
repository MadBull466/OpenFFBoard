# P12 T6b requirements charter: ADC / ADC_Ex (real ST HAL, F407VG and F407VG_DISCO)

Status: 2026-11-13 (lead). T6b follows T6a (timers, gated, awaiting approval). Each stage has its own charter, model, review, gate and approval.
Platform (`fakes/stm32_hal/HalAdc.{h,c}`, lead probe `test_hal_adc_probe.cpp`, shared callbacks `AdcTestSupport.h` / `test_hal_adc_support.cpp`) done; tests, review, gate pending.
Plan: `Tests/Code_Coverage/claude/plans/P12_ST_HAL_PLAN.md`. Rules as in the T1-T6a charters. Both targets compile identical files; the same test files run in both executables.

## Oracles

| Topic | Oracle |
|---|---|
| API semantics, parameters, states, error codes, return values, IT / DMA behaviour | UM1725 Rev 8 chapters "HAL ADC Generic Driver" (from line 3152 of `../ffb-test-build/p12t4c-specs/um1725.txt`) and "HAL ADC Extension Driver" (line 4550) |
| Registers, bit meanings, reset values, conversion modes, scan / continuous / discontinuous, injected channels, analog watchdog, alignment, EOC / OVR rules, DMA, multi ADC | RM0090 Rev 22 (`rm0090.txt`) chapter 13 (line 20707), registers 13.13 (line 22174), register map Tables 71-73 (line 22989) |
| Instance addresses, DMA request mapping | RM0090 Table 1; Tables 43 / 44 (DMA2: ADC1 stream 0 or 4 channel 0, ADC2 stream 2 or 3 channel 1, ADC3 stream 0 or 1 channel 2: check the tables before use) |

Where UM1725 / RM0090 say nothing the case is `characterization:` with a NEEDS REVIEW entry (numbers from 338; writers use `NR_ADC_<n>` and `V_ADC_<n>` placeholders).

## Compiled code and instances

`stm32f4xx_hal_adc.c` and `_adc_ex.c`; `USE_HAL_ADC_REGISTER_CALLBACKS` 0 (Register / UnRegister compiled out: gcov decides). `stm32f4xx_ll_adc.c` compiles to nothing (`USE_FULL_LL_DRIVER` is not defined): out of scope, state it in the report. Weak callbacks (`HAL_ADC_MspInit / MspDeInit`, `ConvCplt`, `ConvHalfCplt`, `LevelOutOfWindow`, `Error`, `HAL_ADCEx_InjectedConvCplt`) are replaced by the strong ones of `AdcTestSupport.h` (never define them in a test file). Instances ADC1, ADC2, ADC3 (parameterise where the code branches on the instance, e.g. multi mode, the common register).
Target facts: the firmware enables the HAL ADC module; PCLK2 is 84 MHz. `HAL_ADC_Init` waits for the stabilization with a counter loop (no register read, no tick).

## Platform (lead): `HalAdc` model

Read the header comment of `HalAdc.h`: register masks, SR rc_w0, conversion engine (`hostAdcConvert`, `hostAdcExternalTrigger`, `hostAdcSetPollReads`), EOC / OVR / AWD / JEOC / STRT / JSTRT rules, DMA request counting, analog values (`hostAdcSetAnalog`), common register data (`hostAdcSetCommonData`). A register outside the ADC windows is a model fault that fails the test case (clear it with `HalFixture::clearFaults()` and report it if the HAL does it).
Model decisions the manual leaves open (state them in the report if a case depends on them): the conversion runs only when the test calls `hostAdcConvert` or the poll-read hook; SWSTART / JSWSTART work only with EXTEN / JEXTEN = 00; the n-th injected conversion goes to JDRn minus JOFRn (the HAL numbers injected ranks that way); the AWD compare uses the 12 bit equivalent; a conversion that finishes while EOC = 1 with overrun detection aborts the regular sequence and sets OVR; the multi ADC mode does not drive the slaves (CDR comes from `hostAdcSetCommonData`).

## Requirement groups (one writer, one file each)

### AB (`test_hal_adc_base.cpp`): init, channels, blocking conversion, watchdog
`HAL_ADC_Init / DeInit / ConfigChannel / AnalogWDGConfig / Start / Stop / PollForConversion / PollForEvent / GetValue / GetState / GetError`, `ADC_Init` (static, through Init).
- AB1 Init / DeInit: NULL handle; state RESET runs MspInit once; the register writes of `ADC_Init` (CCR ADCPRE, CR1 RES / SCAN, CR2 ALIGN / EXTEN / EXTSEL / CONT / DMA / DDS / EOCS, discontinuous mode DISCEN / DISCNUM, SQR1 length) for every Init field; ErrorCode / State after Init, the unlock; DeInit resets registers and runs MspDeInit; `HAL_ERROR` paths.
- AB2 ConfigChannel: SQR1-3 rank fields for ranks 1-16, SMPR1 / SMPR2 sampling time per channel 0-18, the temperature sensor / VREFINT (channels 16 / 18: CCR TSVREFE, VBAT channel 18: VBATE, per instance), lock / busy handling and return codes.
- AB3 Start / Stop (poll): ADON, stabilization, flag clear, SWSTART (software) or no start (external trigger), the injected-running case, state transitions (READY, REG_BUSY, INJ_BUSY, EOC), `HAL_BUSY`; Stop with ADON = 0.
- AB4 PollForConversion / PollForEvent / GetValue: EOC wait with tick timeout (`HAL_TIMEOUT`, state TIMEOUT), the EOCS + DMA configuration error, EOC clears STRT / EOC, event types (EOC, AWD, JEOC, OVR) and their state bits, GetValue returns DR (alignment, resolution), GetState, GetError.
- AB5 AnalogWDGConfig: HTR / LTR, CR1 AWDEN / JAWDEN / AWDSGL / AWDCH combinations, IT on or off (AWDIE), watchdog modes of UM1725, lock.

### AQ (`test_hal_adc_irq.cpp`): interrupt and DMA modes, regular group
`HAL_ADC_Start_IT / Stop_IT / IRQHandler / Start_DMA / Stop_DMA`, `ADC_DMAConvCplt / DMAHalfConvCplt / DMAError` (through the DMA handle), the callbacks.
- AQ1 Start_IT / Stop_IT: CR1 EOCIE / OVRIE, the pending injected state, flag clear, Stop_IT disables IT and ADC.
- AQ2 IRQHandler: EOC (regular, single and sequence), JEOC, AWD, OVR branches in the order of the HAL, flags cleared by the handler (SR snapshots `srAtLast` / `srOrder`: flag cleared before or after the callback; other flags untouched), state bits set, ErrorCode OVR, callbacks counted; no flag or IT disabled: no callback; the EOC with EOCS / scan / continuous / software-start state transitions (READY vs REG_BUSY).
- AQ3 Start_DMA / Stop_DMA: DMA handle callbacks (ConvCplt, HalfCplt, Error), CR2 DMA / DDS, `HAL_DMA_Start_IT` with NDTR / PAR (DR address) / M0AR, OVR IT enable, ADC and DMA disable at Stop, errors (DMA start failure, busy); the DMA stream of the ADC from Tables 43 / 44 (link your own `DMA_HandleTypeDef` as the HAL expects: read the source; real `HAL_DMA_Init`).
- AQ4 DMA callbacks: completion (state EOC / READY decisions by mode: circular, software start, EOCS, `HAL_ADC_STATE_REG_EOC`), half completion, error (ErrorCode DMA, callback, state ERROR_DMA), the injected-busy case. Overrun with DMA (OVR, `HAL_ADC_ERROR_OVR`).

### AX (`test_hal_adc_ex.cpp`): injected group, multi mode
`HAL_ADCEx_InjectedStart / Start_IT / Stop / Stop_IT / PollForConversion / GetValue / ConfigChannel`, `HAL_ADCEx_MultiModeStart_DMA / Stop_DMA / GetValue / ConfigChannel`, the multi mode DMA callbacks, `HAL_ADCEx_InjectedConvCpltCallback` via the IRQ handler (IRQHandler JEOC branch is AQ's; here the injected-specific state bits).
- AX1 InjectedConfigChannel: JSQR rank fields for JL 1-4 and ranks 1-4, JOFRx offset, SMPR, JEXTSEL / JEXTEN, JAUTO / JDISCEN / JDISCNUM (DISCNUM), the injected AWD enable, the temperature channels, lock / busy, return codes.
- AX2 InjectedStart / Start_IT / Stop / Stop_IT: ADON, flag clear (JSTRT, JEOC), JSWSTART (software) or none, regular-busy / auto-injection special cases (`ADC_IS_SOFTWARE_START_INJECTED`, `JAUTO`), JEOCIE, state transitions, Stop with the regular group running.
- AX3 InjectedPollForConversion / GetValue: JEOC wait with tick timeout, flags, state bits, JDRx for ranks 1-4 (alignment, offset), GetValue clears JEOC.
- AX4 Multi mode: ConfigChannel (CCR MULTI / DMAACCESSMODE / TWOSAMPLINGDELAY, master only: ADC1 and ADC3 handling in the source), Start_DMA (pData / length, DMA callbacks, CCR DDS / DMA bits, ADC enable of the master and slaves), Stop_DMA, GetValue (CDR), the multi mode DMA callbacks (completion, half, error).

## Writer rules
As in the T6a brief (`../ffb-test-build/briefs/p12t6b_brief.md`): expected values from UM1725 / RM0090 / hand derivation, `characterization:` + `NR_ADC_<n>`, `KNOWN_ISSUE("V_ADC_<n> ...")` with the first failing assertion being the defect and a spec quote + HAL line in the report; names carry the requirement id; one behaviour per case; fixture `AdcFixture`; no sleeping; gcov per target.
