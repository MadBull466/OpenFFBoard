# P12 ST HAL inventory — 2026-11-06

The owner brought ST HAL into scope and deferred P11. This is the initial source inventory, not test execution or a coverage result. Reproduce from the repository root:

```sh
python3 -B Firmware/Tests/Code_Coverage/codex/tools/inventory_st_hal.py > Tests/Code_Coverage/codex/p12/ST_HAL_INVENTORY.json
```

The stdlib-only scanner reads vendor files without modifying them. JSON records paths, SHA-256, physical line counts, active HAL module defines and lexical HAL occurrences in FFBoard (excluding its USB subtree) and board Core files. Occurrences include declarations, definitions and inactive branches; they are not a compiler call graph or proof a function executes.

## Targets and deduplication

| Target | HAL version from local source | C sources | Comparison to F407VG |
|---|---|---:|---|
| F407VG | 1.8.3, RC 0 | 27 | Initial reference configuration |
| F407VG_DISCO | 1.8.3, RC 0 | 27 | All C source bytes identical |
| F411RE | 1.8.3, RC 0 | 26 | All common C source bytes identical; no HAL CAN source |

This does not establish identical device headers, preprocessor paths, peripheral availability or board configuration. Separate device/configuration coverage remains necessary. The firmware Makefile selects F407VG by default and collects every target HAL `Src/*.c` at lines 30–44. Module defines can still remove function bodies. Source presence, compilation and execution must be reported separately.

F407VG and DISCO enable ADC, CAN, I2C, IWDG, SPI, TIM, UART, PCD, GPIO, EXTI, DMA, RCC, FLASH, PWR and CORTEX modules. F411RE enables the same list except CAN. Extension sources and LL ADC/USB support must be accounted for, even though they are not separate enabled-module defines. Other dormant LL headers are inventoried; their API breadth and active use need assessment before assigning test counts.

## Initial source status

Physical lines below are inventory sizes, not executable denominators.

| Canonical source | Physical lines | P12 status |
|---|---:|---|
| `stm32f4xx_hal.c` | 616 | Bounded real-C tick/delay or handle-state subset; see TIME_STATE_RESULTS |
| `stm32f4xx_hal_adc.c` | 2110 | Not yet tested in P12 |
| `stm32f4xx_hal_adc_ex.c` | 1115 | Not yet tested in P12 |
| `stm32f4xx_hal_can.c` | 2464 | Not yet tested in P12 |
| `stm32f4xx_hal_cortex.c` | 538 | Not yet tested in P12 |
| `stm32f4xx_hal_dma.c` | 1305 | Bounded real-C tick/delay or handle-state subset; see TIME_STATE_RESULTS |
| `stm32f4xx_hal_dma_ex.c` | 313 | Not yet tested in P12 |
| `stm32f4xx_hal_exti.c` | 553 | Not yet tested in P12 |
| `stm32f4xx_hal_flash.c` | 769 | Not yet tested in P12 |
| `stm32f4xx_hal_flash_ex.c` | 1344 | Not yet tested in P12 |
| `stm32f4xx_hal_flash_ramfunc.c` | 172 | Not yet tested in P12 |
| `stm32f4xx_hal_gpio.c` | 533 | Bounded C storage and C++ register-observation tests; see REGISTER_MODEL_RESULTS |
| `stm32f4xx_hal_i2c.c` | 7571 | Not yet tested in P12 |
| `stm32f4xx_hal_i2c_ex.c` | 182 | Not yet tested in P12 |
| `stm32f4xx_hal_iwdg.c` | 262 | Not yet tested in P12 |
| `stm32f4xx_hal_pcd.c` | 2393 | Not yet tested in P12 |
| `stm32f4xx_hal_pcd_ex.c` | 347 | Not yet tested in P12 |
| `stm32f4xx_hal_pwr.c` | 598 | Not yet tested in P12 |
| `stm32f4xx_hal_pwr_ex.c` | 600 | Not yet tested in P12 |
| `stm32f4xx_hal_rcc.c` | 1124 | Not yet tested in P12 |
| `stm32f4xx_hal_rcc_ex.c` | 3833 | Not yet tested in P12 |
| `stm32f4xx_hal_spi.c` | 3930 | Not yet tested in P12 |
| `stm32f4xx_hal_tim.c` | 7632 | Not yet tested in P12 |
| `stm32f4xx_hal_tim_ex.c` | 2410 | Not yet tested in P12 |
| `stm32f4xx_hal_uart.c` | 3771 | Not yet tested in P12 |
| `stm32f4xx_ll_adc.c` | 922 | Not yet tested in P12 |
| `stm32f4xx_ll_usb.c` | 2316 | Not yet tested in P12 |

## Supplied references and missing material

Local sources provide HAL API documentation, implementation, configuration, CMSIS device definitions and licenses. The owner subsequently supplied `Tests/Code_Coverage/shared/specs/DS_dm00037051.pdf`: verified as DS8626 Rev 12 (March 2026), covering STM32F405xx/407xx including F407VG. See [REFERENCES](REFERENCES.md) for identity/hash, inspected sections and limits. The earlier missing-datasheet observation is superseded. Subsequently supplied RM0090 Rev 22 and ES0182 Rev 19 are recorded in REFERENCES; the earlier absence is historical. F411-specific references remain outstanding. No material was downloaded.

Source comments can establish bounded API contracts. Register layouts must come from the supplied device headers; no handwritten invented map is acceptable. Hardware behavior such as write-one-to-clear, FIFO transitions, clock readiness and bus error flags needs the matching ST reference before it can underpin an independent model/oracle. Missing references do not prevent source inventory or pure software-contract work; dependent hardware-semantic claims remain pending.

## Coverage boundary

Earlier firmware callers execute against fake HAL functions. That coverage is not execution of this real ST HAL. Existing P0–P10 results remain valid within their old boundaries. The inventory itself introduces no test or coverage claim. Subsequent GPIO storage/register-observation work is recorded separately in REGISTER_MODEL_RESULTS; shared integration and numeric coverage percentages remain unclaimed.
