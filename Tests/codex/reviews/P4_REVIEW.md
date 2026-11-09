# P4 independent review

2026-11-05 · GPT-6.1 Sol · high reasoning effort. Review only, following the lead charter. All eight assigned test files were read, including every case body and their local helpers: **4,633 source lines, 292 default-build cases**. Production, Claude-owned tests, shared fakes, baseline objects and Git configuration were not changed. P11 excluded.

## Outcome

No new confirmed P4 test defect was established in the assessed paths. Callback signature binding, three existing known-issue repairs and two meaningful voltage boundaries were independently checked by execution. This is scoped positive evidence; it does not certify every descriptor semantic or every firmware branch.

One descriptor-parser semantic question remains for lead adjudication: `test_usb_descriptors.cpp:352–355` assigns both Logical Minimum and Logical Maximum using `signedValue`. The parser therefore interprets the two-byte maximum `FF FF` as -1 even with minimum 0. A compact unsigned 0..65535 field would be reported as an inverted range. None of the current arrays was shown to trigger this; no current baseline false positive or changed descriptor was demonstrated. No owner-supplied HID specification is present locally, so this review records the exact implementation limitation and defers the normative signedness conclusion, rather than silently calling a hypothetical descriptor change a firmware defect. Descriptor length/ID/endpoint observations below do not depend on that unresolved semantic claim.

## Scope, requirements and assessed assertions

| File under `Firmware/Tests/claude/cases_target/` | Cases | Checked observations and limits |
|---|---:|---|
| `test_system_setup.cpp` | 12 | ADC-to-voltage smoke, timer forwarding, LED pulse, UART/CAN recipient and bytes, startup/watchdog and OTP fake self-tests |
| `test_system_commands.cpp` | 78 | command flags and CDC enforcement; asymmetric UID/OTP values; flash map updates; reboot versus DFU recording; main-class list/selection; errors/printer and access rejection |
| `test_cppmain.cpp` | 40 | startup selection/version/format paths, watchdog/yield/LED loop work, wrong-chip loop sentinel, micros boundaries, allocator forwarding, USB task/descriptors/strings, ADC handler lifetime/resolution |
| `test_global_callbacks.cpp` | 44 | ADC pointer/count/data and brake-before-handler order; timer/EXTI/UART dispatch; CAN header/data/FIFO and recipient; SPI/I2C callback types/order; actual TinyUSB-signature calls, CDC partial-send recovery |
| `test_led_effects.cpp` | 23 | GPIO output levels rather than wrapper counters; own-LED separation, pulse/blink/restart behavior, elapsed-time/wrap observation, period ambiguity explicitly characterized |
| `test_voltagesense.cpp` | 37 | ADC/reference/calibration arithmetic, mV units, strict activation/deactivation thresholds and hysteresis, timeout/permanent inhibit/restart, tick-zero defect, temperature scaling/average |
| `test_usb_descriptors.cpp` | 46 | byte-level config/HID parsing, independent config-length sums, interfaces/endpoints, report IDs and byte totals versus firmware structs, strings and per-main descriptor callback matching |
| `test_timer_exti_handlers.cpp` | 12 | lifetime, correct handle/pin, multi-recipient order, ISR context, default-handler continuation and separate registries |

All 292 executed; none of these files has a default skipped case. Nine cases are expected failures, leaving 283 ordinary passes. Default only was executed here. Coordinator baseline/sanitizer/random-order results are separate supporting evidence, not reviewer-performed runs.

Production source checks: complete `cppmain.cpp`, `USBdevice.cpp`, `voltagesense.cpp`, `ledEffects.cpp`, `usb_descriptors.cpp`; targeted `global_callbacks.cpp` ADC/TIM/EXTI/UART/CAN and USB portions; `SystemCommands.cpp` destructive commands, main selection, flash, UID/signature and error formatting. Local helpers/fixtures and relevant fake timer/EXTI/ADC definitions were examined. This was not a full source audit of all generated target code, HAL, RTOS or TinyUSB. The descriptor tables' case bodies and parser were read in full; the thousands of raw HID table bytes were not independently hand-decoded, and full HID PID meaning was not re-derived.

## Callback entry points and destination evidence

`test_global_callbacks.cpp:45–64` supplies weak C-linkage stand-ins with the actual TinyUSB signatures. The tests deliberately call `tud_suspend_cb(bool)` and `tud_hid_report_complete_cb(...,uint16_t)`, not the firmware's wrong C++ overloads. These signatures and call sites were cross-checked against vendored `usbd.h:162`, `usbd.c:77,659`, `hid_device.h:146` and `hid_device.c:87,389`.

The default linked symbols independently confirm the separation (`nm -C ffb_target_tests`): strong `tud_suspend_cb()` and strong HID-complete overload with `unsigned char len`, alongside weak unmangled `tud_suspend_cb` and `tud_hid_report_complete_cb`. Thus the bug-42 test's empty event log comes from the same-signature default, not from directly calling the wrong firmware overload. The callback repair experiment below overrides the weak bool definition without a link conflict and flips exactly its intended known issue.

The mount/unmount/resume case observes ordered events `resume,suspend,resume`. ADC cases observe distinct 8-channel/2-channel buffers, original handles and asymmetric payloads, plus brake output already asserted when the handler receives ADC2. CAN cases inspect concrete standard/extended IDs, RTR, DLC, data, FIFO, filter/timestamp, global port and arrival order. SPI/I2C cases call the real firmware completion entrypoints with recording handlers in simulated ISR context. Those direct callback cases establish dispatch, not real bus interrupt generation; later bus suites own that additional path.

Timer tests reach real `HAL_TIM_PeriodElapsedCallback_CPP` via the fake's strong `HAL_TIM_PeriodElapsedCallback`. The strong fake was compared with F407VG `main.c:1554–1567`: TIM7 increments the HAL tick; other timers forward to C++. Tests check the split and correct recipients. Generated main.c itself is **not linked**; wording such as “through main.c” in existing case names denotes this modeled bridge, not an executed target ISR. The weak fallback in real global_callbacks is consequently not exercised. EXTI injection reaches the real firmware GPIO callback in simulated interrupt context, but does not establish electrical edge timing or hardware NVIC behavior.

## Units, thresholds, elapsed time and descriptor bytes

Voltage tests derive one count = one mV using calibration 1365, VREFINT read 1100 and calibration reference 3300 mV: 1365*3300/1100 = 4095 mV. With the selected multiplier 20, ADC differences resolve thresholds in 20 mV steps. This permits distinct checks at 5000/5020 mV activation, 4020/4000 mV release, 6480/6500 mV minimum-power enable and 65000/65020 mV forced braking. Scaling/temperature fake LL macros were compared with the vendored STM32 LL ADC header; the assertions include independently computed units/values rather than only calling the same production helper for expected values. This proves host arithmetic, not physical calibration accuracy. The intentional high synthetic reference is an arithmetic fixture, not a claim of real supply operation at 4.095 V.

The five-second case observes an active brake at exactly 5000 elapsed ms and inactive at 5001. Separate cases test reset of the timer after a normal off transition, permanent inhibit after failure, and the tick-zero sentinel bug. LED cases inspect ODR pin state at times bracketing the pulse period. Existing NEEDS REVIEW 70 keeps strict > versus >= endpoint policy open; the wrap-specific known issue does not demand the disputed extra millisecond. `micros()` tests exercise 7999 us, the next HAL tick in smoke coverage and wrap at 2^32 us; they do not model the tick/CNT race already noted in historical NEEDS REVIEW 65.

Configuration lengths are now checked against independent byte sums: CDC-only 9+66=75, CDC/HID 9+66+32=107 and CDC/MIDI 9+66+92=167. Variant arrays use `sizeof` in addition to their length field. The parser-alone equality `parsedLength==totalLength` is bounded by totalLength and would be tautological by itself; the independent sums/array sizes and interface/endpoint content checks supply the missing external constraint. Report descriptor tests compare bytes consumed and final End Collection with actual array size, and match report IDs/lengths to public packed structures. This is host descriptor self-consistency, not complete HID/USB specification certification. The unbuildable one-axis 32-bit variant remains explicitly outside execution (existing bug 51); MIDI descriptor data is checked without enabling the out-of-scope MIDI main class.

System reboot and DFU cases distinguish `systemResets` from `dfuReboots`; the fake's DFU additionally counts its reset. On target those operations do not return; the host's subsequent OK is explicitly qualified in the tests. The tested reset APIs take no command-side parameters. Format guards use prepopulated flash, and CDC read-only rejection uses operations whose execution would be observable (reset count or retained error), with an allowed reboot positive control. The fake map's Flash_Format and Flash_Init never fail: those failure paths remain deferred, not proven by line coverage. Signature uses a model of OTP, not actual one-time hardware memory; extended write-range policy is an existing characterization/bug-41 concern.

## Executed evidence and mutation checks

Baseline phase command (default binary `../ffb-test-build/codex-baseline-default/ffb_target_tests`):

```
ffb_target_tests -r=xml -sf='*test_system_setup.cpp,*test_system_commands.cpp,*test_cppmain.cpp,*test_global_callbacks.cpp,*test_led_effects.cpp,*test_voltagesense.cpp,*test_usb_descriptors.cpp,*test_timer_exti_handlers.cpp'
```

Result: **292 executed / 0 unexpected failures**, log `/tmp/p4_run.xml`. Doctest's 1545 skipped includes all filtered-out cases; it is not a P4 skip count.

Scratch mutations compile one copied production source and replace only its object at link time, preserving baseline objects. The baseline linker dependency-file output option is removed. Each focused filter below matched exactly one case; every experiment also ran the entire existing target executable (1819 executed, 18 existing skips).

| Experiment | Focused observation | Whole-target outcome |
|---|---|---|
| Change suspend definition from `void tud_suspend_cb()` to `void tud_suspend_cb(bool remote_wakeup_en)` | bug-42 test all 2 assertions pass; marker flips | 1818 passed / 1 marker failure; no other changed outcome |
| LED deadline compare -> `HAL_GetTick() - led->tick > led->period` | wrap case all 6 assertions pass; marker flips | 1818 / 1 marker failure; only bug48 |
| Brake timeout guard `brakeActiveTime` -> `braking_flag` | tick-zero case all 11 assertions pass; marker flips | 1818 / 1 marker failure; only bug49 |
| Incorrect timeout `>5000` -> `>=5000` | killed at `test_voltagesense.cpp:241`: exact 5 s brake unexpectedly off | 1818 passed / 1 ordinary failure; same case |
| Incorrect activation `vint > vext+voltageDiffActivate` -> `>=` | killed at `test_voltagesense.cpp:174`: exact 5 V brake unexpectedly on | 1817 passed / 2 ordinary failures; named case plus configured 2 V threshold case |

Focused case filters, in row order:

```
-tc=a USB bus suspend reported by TinyUSB suspends the main class
-tc=a sys LED pulse started 16 ms before the 32 bit tick wraps around lasts 25 ms
-tc=braking that starts at tick 0 is also shut down after 5 s
-tc=braking for 5 s is accepted? longer switches the brake off permanently
-tc=the brake switches on when the internal voltage exceeds the supply by more than 5 V
```

The `?` substitutes for the comma in the case name because doctest splits comma filters. Unmutated controls for all these named cases ran in the phase baseline; full-target unmutated control was 1819/0/18 in the preceding P2 experiment against the same baseline. These five spot checks are not a mutation score for the phase.

Evidence: `/tmp/codex_p4_experiment.py`; sibling `../ffb-test-build/codex-p2-p4-review/p4_results.json` and `p4_{suspend_fix,wrap_fix,tickzero_fix,timeout_mutant,threshold_mutant}_{focused,full}.log`. No sanitizer was used in this experiment. No sanitizer settings or suppressions were changed.

The nine baseline known-issue first-failure locations were checked against XML/case bodies: cppmain 329/424 (invalid config index/long string length), callbacks 869 (bool suspend recipient), LEDs 230 (wrap), system commands 891 (second persistent-error listing), descriptors 572/1090 (IAD class/suffix), voltage 296/323 (missing critical report/tick-zero shutdown). They match their stated observations in this execution. No wrong-first-failure was confirmed. The selected repair checks additionally show unrelated assertions do not keep those three expected-failure cases green after their targeted repair.

## Remaining limits

No broad firmware mutation campaign, no-DSP comparison, sanitizer re-run, forced crash/UB cases, target hardware, real bus timing, preemption or interrupt races were assessed by this reviewer. Native-endian `memcpy` expectations and printf ABI caveats remain host/target limitations already documented by the suite. The moving temperature average asserts current integer smoothing behavior, not physical sensor dynamics. The NEEDS REVIEW 73 case titled “stops nothing” checks only error-LED output; it does not instantiate a motor and establish motor behavior. That part of its title is a scope overstatement, recorded here rather than promoted to a new firmware finding.

P4 host observations are reasonably discriminating in the paths checked. Broader sign-off still needs the lead's resolution of policy/specification questions and the P9 real-device-stack evidence for actual USB tokens and lifecycle recovery.
