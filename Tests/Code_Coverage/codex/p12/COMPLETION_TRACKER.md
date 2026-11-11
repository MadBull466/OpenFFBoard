# P12 completion tracker

The owner now requests: “Finish P12 until it is complete and covered.” Continue autonomously through remaining work; do not stop after each small group or call inventory/compilation alone coverage. P11 remains deferred. P12 is **not complete** at this checkpoint.

## Latest implementation checkpoint

A new owned `Firmware/Tests/Code_Coverage/codex/hal/native32/` project compiles all 27 canonical F407 HAL C translation units unchanged, using installed Clang/GCC multilib. A 32-bit process avoids the actual DMA pointer truncation problem. Anonymous private memory windows at device numeric addresses are process RAM, never device mappings or hardware access. This supersedes the earlier self-imposed no-window design preference; it changes no vendor source and installs nothing.

The observation build uses Clang kernel-address instrumentation only as a C load/store callback mechanism, **not as an ASan memory-safety runtime**. Stack/global sanitizer metadata and access deduplication are disabled so every C register access is observed; genuine 64-bit ASan/UBSan gates remain separate. The supplied CMSIS layouts/constants are retained; CPU intrinsics are the lower boundary. Only vendor C is access-instrumented; fixture/observer C++ accesses inject hardware inputs or observe backing/request state and require explicit flush for a final pending store.

The native bus implements bounded W0C/W1C, data-read, key, bit-band and USB self-clearing rules plus explicit I2C/SPI peer input and DMA-completion events. This is not complete silicon fidelity. Models need direct controls, mutation checks and a written fidelity/reference map before final acceptance. Initial raw-RAM errors and missing/de-duplicated access hooks were model defects, not vendor defects; corrected in owned code only. GCC native raw build and earlier failed/crashed runs are developmental, not final evidence.

Latest clean F407 observed run: **232 ordinary cases / 1155 passing assertions**, plus one exact expected failure for P12-I2C-001, random seed 271, no stderr (`codex-p12-native32-observed/evidence/adc-recovery-corrected`). The verifier rejects every other failure, marker, skipped/empty inventory or child diagnostic. F411 and DISCO earlier checkpoints passed 209 and 214 ordinary cases respectively plus the same known issue; genuine native32 UBSan passed the 214-case checkpoint. These variants must be rerun after the current ADC/timer changes. Four stdlib collector failure controls pass.

Latest collected F407 coverage is the **224 ordinary-case checkpoint**: 5630 / 12285 active HAL C lines (45.8%), with all 27 files retained and per-function summaries/uncovered lines in `evidence/timer-extended/coverage/coverage.json`. It predates the latest ADC cases. No completeness claim follows from this percentage. F407 flash_ramfunc/I2CEx are compiler-inactive and remain explicit rows.
Build command: `cmake -S Firmware/Tests/Code_Coverage/codex/hal/native32 -B ../ffb-test-build/codex-p12-native32-observed -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DP12_ACCESS_MODEL=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`. Build/run must have the established sibling-directory/out-of-sandbox permission; 32-bit execution is blocked by the sandbox's syscall policy. All files remain local. Coverage is collected with `coverage_native.py`, using llvm-cov gcov. The collector requires a nonempty compile-command inventory and validates inactive-TU notes against an owned compiler-produced empty control; it does not silently drop missing outputs.

A preliminary **98-case** sample (before USB/EXTI/full-LL/expanded serial cases) measured 2094 / 12161 active C lines (17.2%); it is historical, not current coverage. Largest remaining regions: I2C, TIM/TIMEx, SPI/UART and LL USB. Full optional LL ADC is now enabled in the native build; its new denominator must be included in the next measurement.

## Still required

- Extend software/reference-backed requirements and meaningful positive/error/IRQ/DMA cases across all modules; add direct access/peer/DMA model controls. Current native files cover active DMA, serial, ADC, timers, CAN, RCC/power/flash, EXTI, USB/PCD and full LL ADC subsets, not complete module behavior.
- Memory-to-memory direction/width/increment now has a passing direct control. UART/SPI/I2C/ADC completion now requires appropriate peripheral request/data conditions; expand direct negative controls and document the remaining event abstractions.
- More timer and communication lifecycle/error paths, PCD IRQ/LL USB host helpers, FLASH option/interrupt/erase behavior and target-specific extensions remain.
- F411 native source/model arrays now use actual device availability and its four RAMFUNC request cases pass. Supply written per-device reference qualifications; DISCO duplicates must remain tracked. F407 I2CEx/flash_ramfunc are compiler-inactive; F411 adds those bodies. Dormant LL headers need an explicit inventory/disposition, not silent denominator removal.
- Collect fresh complete coverage with missing-data/collector controls, exact per-file/per-function uncovered lists and separate C/C++/native claims. Coverage is not assertion quality.
- Add representative native HAL/model mutation controls, retain prior GPIO/time-state gates, run original 318 checker and full existing default/affected sanitizer/no-DSP gates. No forced UB/skip runs.
- Shared CTest integration needs a concrete reviewable patch and the AGENTS.md ownership gate; none has been applied for P12. Preserve eight prior authorized shared-file edits and log any newly approved shared changes separately.
- Final review/QC and release manifest, source hashes, durable commands/results, finding dispositions and clear remaining hardware/reference limits. No fresh independent-agent review has occurred for P12. Do not spawn agents unless explicitly authorized by user/applicable AGENTS.md/skill instructions.

No completion claim, current percentage, vendor repair, commit or push is authorized by this checkpoint. Continue work rather than requesting design input from the owner.
