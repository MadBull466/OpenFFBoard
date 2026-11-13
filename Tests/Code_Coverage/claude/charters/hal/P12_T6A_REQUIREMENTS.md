# P12 T6a requirements charter: timers TIM / TIM_Ex (real ST HAL, F407VG and F407VG_DISCO)

Status: 2026-11-12 (lead). T6 is split: T6a timers (`stm32f4xx_hal_tim.c`, `stm32f4xx_hal_tim_ex.c`), T6b ADC (`stm32f4xx_hal_adc.c`, `_adc_ex.c`, `stm32f4xx_ll_adc.c`), each with its own charter, model, review, gate and approval.
Charter and platform (`fakes/stm32_hal/HalTim.{h,c}`, lead probe `test_hal_tim_probe.cpp`, shared callbacks `TimTestSupport.h` / `test_hal_tim_support.cpp`) done; tests, review, gate pending.
Plan: `Tests/Code_Coverage/claude/plans/P12_ST_HAL_PLAN.md`. Rules as in the T1-T5 charters. Both targets compile identical files; the same test files run in both executables.

## Oracles

| Topic | Oracle |
|---|---|
| API semantics, parameters, state / channel state / active channel names, error returns, DMA / IT behaviour | UM1725 Rev 8, chapters "HAL TIM Generic Driver" and "HAL TIM Extension Driver" (`../ffb-test-build/p12t4c-specs/um1725.txt`, generic driver from line 65337, extension driver from line 69796; sequences "Initialize the TIM low level resources / Time Base / Output Compare / PWM / Input Capture / One Pulse / Encoder / Hall") |
| Registers, bit meanings, reset values, event generation, counter modes, capture / compare, break and dead time, lock, DMA burst, slave / trigger modes | RM0090 Rev 22 (`rm0090.txt`): chapter 17 TIM1 / TIM8 (line 27708), 18 TIM2-TIM5 (31816), 19 TIM9-TIM14 (35297), 20 TIM6 / TIM7 (37850); registers 17.4, 18.4, 19.4 / 19.5, 20.4 |
| Instance addresses, DMA request mapping | RM0090 Table 1 (see `HalTim.h`); Tables 43 / 44 (DMA1 / DMA2 request mapping, channel and stream per timer event; the HAL maps the DMA handles through `hdma[TIM_DMA_ID_*]`: the tests link their own `DMA_HandleTypeDef` for the burst / event they test with the real `HAL_DMA_Init`) |

Where UM1725 / RM0090 say nothing (return codes in uncommon cases, which register a function touches, order of writes) the case is `characterization:` with a NEEDS REVIEW entry (numbers from 332; writers use `NR_TIM_<n>` and `V_TIM_<n>` placeholders).

## Compiled code and instances

`stm32f4xx_hal_tim.c` and `_tim_ex.c`, `USE_HAL_TIM_REGISTER_CALLBACKS` 0 (Register / UnRegister functions compiled out: gcov decides). Weak callbacks (Msp init / deinit of Base, OC, PWM, IC, OnePulse, Encoder, HallSensor; PeriodElapsed (+Half), OC_DelayElapsed, IC_Capture (+Half), PWM_PulseFinished (+Half), Trigger (+Half), Error, Commut (+Half), Break) are strong test callbacks defined once by the lead (`timCounts`, order log, active channel and state at the last call). `USE_FULL_ASSERT` off, so the `IS_TIM_*` asserts are not tested, but the HAL's own instance branches are (`IS_TIM_CCX_INSTANCE`, `IS_TIM_COUNTER_MODE_SELECT_INSTANCE`, ...): vary the instance (TIM1 / TIM8 advanced, TIM2 / TIM5 32-bit, TIM3 / TIM4, TIM6 / TIM7 basic, TIM9 / TIM12, TIM10 / 11 / 13 / 14) wherever the HAL source branches on it.

## Platform (lead): `HalTim` model

Read the header comment of `HalTim.h`: per-type register sets and masks (a register the timer does not have is a fault that fails the test case), SR rc_w0, EGR with its effects (UG, CCxG with capture for input channels, COMG, TG, BG), counter advance `hostTimAdvance` (up, down, centre-aligned, RCR, OPM, compare flags), `hostTimCapture`, `hostTimSetCounter`, `hostTimRaiseFlags`, `hostTimBreak`, `hostTimIrqPending` (the interrupt line; tests call `HAL_TIM_IRQHandler`), CCR of an input channel read-only, CCxS only while the channel is off, BDTR.LOCK levels, DMAR redirect with DBA / DBL (the test plays the DMA with plain DMAR accesses). Output pins, the prescaler division and slave modes are not modelled (SMCR is stored). DMA modes: the HAL starts streams with `HAL_DMA_Start_IT`; tests set up the handle (T3 hooks: `hostDmaComplete`, `hostDmaStepItems`, `hostDmaRaise`, `HAL_DMA_IRQHandler`) and check the DMA stream registers and the DIER / CCER / CR1 changes.
Model decisions the manual leaves open (state them in the report if a case depends on them): the prescaler does not divide, `hostTimAdvance` counts counter-clock events; UEV with `URS = 1` from UG does not set UIF; centre-aligned mode updates at the underflow only; a UEV with AOE sets MOE; the register write masks of TIM9 / TIM12 / TIM10 / TIM11 / TIM13 / TIM14 follow RM0090 chapter 19 as read by the lead (compare and report differences); DMAR access redirect starts at DBA with the burst index counting 0 .. DBL.

## Requirement groups (one writer, one file each)

### TB (`test_hal_tim_base.cpp`): base, output compare, PWM
`HAL_TIM_Base_Init / DeInit / Start / Stop / Start_IT / Stop_IT / Start_DMA / Stop_DMA`, `HAL_TIM_OC_Init / DeInit / Start / Stop / Start_IT / Stop_IT / Start_DMA / Stop_DMA`, `HAL_TIM_PWM_Init / DeInit / Start / Stop / Start_IT / Stop_IT / Start_DMA / Stop_DMA`, `HAL_TIM_OC_ConfigChannel`, `HAL_TIM_PWM_ConfigChannel`, `HAL_TIM_Base_GetState`, `HAL_TIM_OC_GetState`, `HAL_TIM_PWM_GetState`, `HAL_TIM_GetChannelState`, `HAL_TIM_GetActiveChannel`.
- TB1 Init / DeInit: NULL handle; state RESET runs MspInit once and unlocks; the register writes of `TIM_Base_SetConfig` (CR1: counter mode, direction, clock division, ARPE, per instance; ARR, PSC; RCR for TIM1 / TIM8; the UG that reloads), state READY, channel states READY, DeInit (CEN clear, MspDeInit, state RESET, channel states), all instances and counter modes (up, down, centre 1-3).
- TB2 Start / Stop (poll, IT, DMA): CEN, DIER bits (UIE, CCxIE, UDE, CCxDE), CCER CCxE and the MOE of TIM1 / TIM8 (`__HAL_TIM_MOE_ENABLE`), channel state transitions and the HAL_ERROR returns when the channel state is not READY, slave-mode trigger special case (`IS_TIM_SLAVE_INSTANCE` with SMS: enable only when the SMS is not trigger mode: read the source), stop restoring state, all four channels and the invalid-channel branches.
- TB3 ConfigChannel OC / PWM: CCMR / CCER / CCR writes for each channel, polarity, N polarity and idle state for TIM1 / TIM8, fast mode (PWM), OCMode values, lock state, HAL_BUSY when locked, return codes and the `HAL_LOCKED` handling.
- TB4 GetState / GetChannelState / GetActiveChannel values.

### TI (`test_hal_tim_ic.cpp`): input capture, one pulse, encoder, hall sensor
`HAL_TIM_IC_*`, `HAL_TIM_OnePulse_*`, `HAL_TIM_Encoder_*`, `HAL_TIMEx_HallSensor_*` (Init / DeInit / Start / Stop / IT / DMA variants), `HAL_TIM_IC_ConfigChannel`, `HAL_TIM_OnePulse_ConfigChannel`, `HAL_TIM_ReadCapturedValue`, `HAL_TIM_IC_GetState / OnePulse_GetState / Encoder_GetState / HallSensor_GetState`.
- TI1 IC: init, config of channels 1-4 (polarity, selection direct / indirect / TRC, prescaler, filter; TI_SetConfig helpers), Start / Stop in the three modes (CCxE, DIER bits, channel state, DMA stream), ReadCapturedValue for the four channels, captures through `hostTimCapture` and the IRQ handler is the other writer's.
- TI2 One pulse: init with OPM, ConfigChannel for OutputChannel and InputChannel (the two-channel configuration), Start / Stop (+IT, N variants belong to TN), CEN not set by Start (trigger).
- TI3 Encoder: init (SMS encoder mode 1-3, IC1 / IC2 polarity, selection, prescaler, filter), Start for channel 1, 2, both, Stop, IT and DMA variants with their channel / state rules.
- TI4 Hall sensor: init (the TI1 XOR bit TI1S, commutation delay in CCR2, trigger output OC2REF), Start / Stop in poll / IT / DMA mode, state.

### TC (`test_hal_tim_cfg.cpp`): configuration, events, burst, synchronisation
`HAL_TIM_ConfigClockSource`, `HAL_TIM_SlaveConfigSynchro`, `HAL_TIM_SlaveConfigSynchro_IT`, `HAL_TIM_ConfigOCrefClear`, `HAL_TIM_ConfigTI1Input`, `HAL_TIM_GenerateEvent`, `HAL_TIM_DMABurst_WriteStart / MultiWriteStart / WriteStop / ReadStart / MultiReadStart / ReadStop`, `HAL_TIMEx_ConfigCommutEvent`, `_IT`, `_DMA`, `HAL_TIMEx_MasterConfigSynchronization`, `HAL_TIMEx_ConfigBreakDeadTime`, `HAL_TIMEx_RemapConfig`.
- TC1 ConfigClockSource: every clock source (internal, ETR mode 1 / 2, ITR0-3, TI1ED, TI1FP1, TI2FP2) with SMCR writes, polarity / prescaler / filter, state BUSY / READY and lock, invalid source.
- TC2 SlaveConfigSynchro (+IT): trigger, mode, polarity, prescaler, filter per trigger source, TIF enable in the IT variant, return codes.
- TC3 OCref clear, TI1 input selection, GenerateEvent (each event source into EGR with the model's effects), the HAL_BUSY / state rules.
- TC4 DMA burst write / read: DCR (DBA, DBL), DIER request enable per burst source, DMAR address, burst length checks, stop clearing the request, state `HAL_TIM_STATE_BUSY` / DMABurstState rules; the test plays the DMA with plain DMAR accesses (model redirect).
- TC5 TimEx: commutation event (+IT, +DMA) CR2 CCUS / CCPC and trigger selection per InputTrigger, MasterConfigSynchronization (MMS, MSM per instance), BreakDeadTime (every BDTR field, LOCK levels and the model's lock effects, the OSSR / OSSI / AOE / MOE bits), RemapConfig (OR register for TIM2 / TIM5 / TIM11).

### TN (`test_hal_tim_exn.cpp`): complementary outputs
`HAL_TIMEx_OCN_Start / Stop / Start_IT / Stop_IT / Start_DMA / Stop_DMA`, `HAL_TIMEx_PWMN_Start / Stop / Start_IT / Stop_IT / Start_DMA / Stop_DMA`, `HAL_TIMEx_OnePulseN_Start / Stop / Start_IT / Stop_IT`.
- TN1 each function for channels 1-3 (CCxNE, MOE, DIER bits, channel N state), the HAL_ERROR rules by channel state, channel 4 invalid, TIM1 / TIM8 only (other instances are a source branch the writer reads: assert nothing else than what the source and UM1725 define), CEN not set by OnePulseN start.
- TN2 Stop variants: CCxNE cleared, MOE disabled only when no other channel is enabled (`TIM_CCxN_SET_CAPTURE_COMPARE... __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY` rules in the source), DIER bits cleared, DMA streams aborted, channel N states READY.
- TN3 DMA variants: DMA stream configuration per channel (addresses, length), `HAL_ERROR` when the DMA start fails or the channel state is not READY, completion callbacks of N channels (shared with TQ).

### TQ (`test_hal_tim_irq.cpp`): IRQ handler and DMA callbacks
`HAL_TIM_IRQHandler` (every flag branch) and the static DMA callbacks reached through the DMA streams: `TIM_DMAPeriodElapsedCplt`, `TIM_DMAPeriodElapsedHalfCplt`, `TIM_DMADelayPulseCplt`, `TIM_DMADelayPulseHalfCplt`, `TIM_DMACaptureCplt`, `TIM_DMACaptureHalfCplt`, `TIM_DMATriggerCplt`, `TIM_DMATriggerHalfCplt`, `TIM_DMAError`, `TIM_DMACommutationCplt`, `TIM_DMACommutationHalfCplt`, plus `TIM_DMAErrorCCxN`.
- TQ1 IRQ handler: CC1-CC4 (output compare -> OC_DelayElapsed + PWM_PulseFinished; input capture -> IC_Capture, ActiveChannel set), UIF (PeriodElapsed), BIF (Break), TIF (Trigger), COMIF (Commut), flag cleared before the callback, ActiveChannel and its reset to CLEARED, enable gating by DIER, nothing happens when the flag is set but the interrupt is not enabled, several flags at once (order), CCxOF overcapture (no special handling: check the source).
- TQ2 DMA callbacks per direction: complete and half-complete callbacks, the channel state updates (DMA completion sets channel state READY for non-circular? read the source), ErrorCallback with the channel states on `TIM_DMAError`, the commutation callbacks.
- TQ3 each callback is seen once per event through `timCounts` kinds and order.

## Writer rules
As in the T5 brief (`../ffb-test-build/briefs/p12t6a_brief.md`): expected values from UM1725 / RM0090 / hand derivation, `characterization:` + `NR_TIM_<n>`, `KNOWN_ISSUE("V_TIM_<n> ...")` with the first failing assertion being the defect and a spec quote + HAL line in the report; names carry the requirement id; one behaviour per case; no sleeping; use `TimFixture` and `timCounts` (never define HAL_TIM_* callbacks).
