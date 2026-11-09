# P12 supplied references

## STM32F405xx / STM32F407xx datasheet

Owner supplied `Tests/shared/specs/DS_dm00037051.pdf` on 2026-11-06. Verified document identity: ST DS8626, revision 12, March 2026, 206 pages. Table 1 on page 2 includes STM32F407VG. SHA-256: `1f41b4fa49a9960215ebb76cd0a75f20241db82eed60d308345a80463caeed22`.

Read the title/device-summary pages, complete introduction (page 13) and complete functional-overview page 41; visually checked page 41 after rendering. This is a targeted reference check, not a full 206-page technical review. PDF metadata/text extraction used local Poppler; the original file was read-only. No specification bytes or extracted text are committed/copied into this documentation.

| Material | Established use / boundary |
|---|---|
| DS8626 sections 1 and 3.34, pages 13 and 41 | Device applicability and high-level GPIO modes/pulls/alternate functions/configuration locking. Useful for requirement categories; not a complete register-transition oracle. |
| DS8626 section 3.35, page 41 | High-level ADC conversion/DMA/watchdog/trigger capabilities; detailed test requirements remain unmapped. |
| DS8626 pin tables/electrical sections | Available for future targeted review; not yet read or used to assert package/pin/timing details. |
| STM32F4 reference manual | Now supplied: RM0090 Rev 22, May 2026. See verified reference entry below. The earlier missing-reference observation is superseded. |
| STM32F405xx/407xx errata ES0182 | Now supplied: Rev 19, July 2026. See entry below; silicon-revision applicability still needs per-requirement mapping. |
| STM32F411 device references | This datasheet does not establish F411-specific behavior; separate material remains necessary for device-specific claims. |

The local HAL API documentation and ST CMSIS device headers remain additional supplied sources. Keep software-contract tests separate from claims that need peripheral hardware semantics. Missing reference material does not prevent the first bounded GPIO real-source harness. No downloads were performed; no changes were made to the shared specs directory.

## Reference manual and errata supplied subsequently

The owner provided paths under `Tests/shared/`; the actual local files were found under `Tests/shared/specs/`. The datasheet there is the already recorded DS8626 Rev 12. No files were moved or copied into the shared directory, and no downloads were performed.

| Local supplied reference | Verified identity | SHA-256 |
|---|---|---|
| `Tests/shared/specs/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf` | RM0090 Rev 22, May 2026, 1741 pages; STM32F405/415, F407/417, F427/437 and F429/439 | `f3ff2322eec21f4e49081f68481b764506eee81dba6031bc767e2f8955b27cca` |
| `Tests/shared/specs/es0182-stm32f405407xx-and-stm32f415417xx-device-errata-stmicroelectronics.pdf` | ES0182 Rev 19, July 2026, 45 pages; STM32F405/407xx and STM32F415/417xx | `1d3da0f99b85013548d17df07919516bf862f8f38cbd1a61f08e75abb8e4c54f` |

Read RM0090 introduction/contents and complete pages 286–287 (sections 8.4.7–8.4.9); visually checked the BSRR/lock diagram on page 286. Read the ES0182 applicability page and complete page 11, including section 2.2.10. This is targeted reading, not full review of either document. Their versions postdate HAL 1.8.3; specification/implementation disagreement must be assessed rather than automatically treated as a HAL defect.

### Constraints for the next GPIO model

- RM0090 section 8.4.7: BSRR reads return zero; writing set/reset bits affects the corresponding ODR bits, and set wins when both target the same pin. The current RAM observation harness deliberately does not implement those effects. It must not be relabeled a peripheral model merely because the reference is now available.
- RM0090 section 8.4.8: locking requires the specified word-access key sequence with a constant pin mask and confirmation read. Invalid sequences do not establish a lock. Post-lock configuration protection and reset/unlock behavior need explicit model tests, including negative sequences; a final stored LCKR word alone cannot establish correct behavior.
- ES0182 section 2.2.10 (page 11): PH10/PI10 interrupt routing has an interaction with PH2/PI2 selection. Map its revision applicability from the errata summary before making device-specific routing claims. This is a documented silicon limitation, not a newly discovered HAL defect or an authorized vendor repair.
- Still to read before dependent EXTI tests: RM0090 section 12.2 and pending-clear register semantics, SYSCFG routing/clock requirements, and the errata applicability summary. Do not invent these details from the prior fake.
- F411-specific references remain outstanding; RM0090 and ES0182 do not establish F411 device behavior.

The reference-material blocker for F407 GPIO model design is removed. The implementation/interception strategy for observing actual volatile accesses still requires design and verification; no additional GPIO tests or coverage were executed by this reference update.

## Register-adapter implementation reading

Subsequently inspected GPIO configuration register descriptions on RM0090 pages 283–285 (including reset values and pull encodings), EXTI_PR/SWIER page 388, GPIO type layouts and the HAL implementation. The implemented lower-16 pending/configuration adapter is bounded in REGISTER_MODEL_DESIGN. SWIER generation, physical routing and full silicon-revision applicability remain unmodeled; PH10/PI10 routing is rejected explicitly. The earlier reference-update statement that no further tests had run is historical; REGISTER_MODEL_RESULTS records the subsequent execution.
