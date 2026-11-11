# P5 independent input-test review

2026-11-05; fresh review assignment `/root/p0_reviewer`, GPT-6.1 Sol, high reasoning. Scope: twelve assigned input case files, phase requirements and source/interface checks. This report is the only P5 repository edit; original tests, shared infrastructure, firmware, vendor code and git state were preserved.

## Result and finding

**P5-1 — The ADS threshold known-issue case can hide a repaired threshold write behind an unrelated setup failure (medium).** `Firmware/Tests/Code_Coverage/claude/cases_target/test_ads111x.cpp:271-290` marks the whole case `KNOWN_ISSUE` for low-threshold bytes being overwritten by the high-threshold call. Before checking the bytes it requires a pending interrupt transfer and exactly one main-thread semaphore fault.

Control: the isolated original case selects exactly one case, exits 0, and its first failing assertion is the intended `adc.register16(0x02) == 0x1000` at line 290. Scratch repair P5M1 changes only the two `ADS111X::setThresh` calls from existing `writeRegIT` to existing serialized blocking `writeReg`, retaining register numbers and distinct values. Both writes can then finish before return. The unchanged case still selects one case and exits 0 as an expected failure, but its first failure is now `REQUIRE(HostTarget::i2cComplete(&hi2c1))` at line 279: there is no pending interrupt transfer. The register assertion is never reached. This is a reproduced first-failure masking defect, not a claim that blocking I2C is the preferred product implementation.

The public `setThresh` interface has no `IT` suffix or documented asynchronous completion contract in the reviewed header/source. If asynchronous behavior is a product requirement it needs to be established separately. The case currently couples defect detection to the existing implementation strategy and synthetic main-thread blocking. Recommended next work: run the operation in a resumable task with a distinct actor completing transfers, keep setup failures outside whole-case expected-failure masking, and independently assert both distinct threshold registers. A valid repair must make the defect assertion succeed or expose its marker as obsolete; an unrelated setup REQUIRE must not count as the original bug.

Evidence: scratch source, compile/link logs and XML in `/home/ben/Documents/OpenFFBoard/testing/ffb-test-build/codex-p5-p6-review/P5M1_*`; combined [mutation results](/home/ben/Documents/OpenFFBoard/testing/ffb-test-build/codex-p5-p6-review/mutation_results.json). Reproducer: [/tmp/codex_p5_p6_experiments.py](/tmp/codex_p5_p6_experiments.py). It compiles a copied firmware source and relinks against immutable default baseline objects, removing the link dependency-file output option. No original object or source is changed.

## Assessed scope

The twelve files contain **405 catalogued cases**. Files and case bodies were inspected at source-review level, with these limits: command/default/flash cases were assessed against local public conventions and source declarations; they were not each independently mutation-tested. The detailed behavior review focused on packing, asymmetric channel offsets, distinguishable expanders, error delivery, nonzero retained state, busy ownership and calibration/filter arithmetic. Long combined reads of `test_analog_axis_processing.cpp` were partly truncated; its scaling/filter/command/flash groups were inspected, but no exhaustive line-by-line reading claim is made for that file. No supplied ADS/PCF/shifter/shift-register datasheet was opened; sensor-specific register and bit conventions remain supported by the test/source comments rather than an independently supplied primary specification.

| File under `Firmware/Tests/Code_Coverage/claude/cases_target/` | Cases | Assessment |
|---|---:|---|
| `test_input_setup.cpp` | 12 | ADC/SPI/I2C/CAN real callback path, chooser/HID smoke integration and two fake self-tests; these establish host plumbing, not peer electrical fidelity |
| `test_selectable_inputs.cpp` | 30 | Different source lengths/order and distinct analog vectors; returned button count controls offset; destruction/selection and registries; 64-bit overflow known UB read only |
| `test_analog_source.cpp` | 10 | Public buffer and distinct destructor flag; persistence registration and registry construction |
| `test_local_buttons.cpp` | 21 | Sparse enabled pins and polarity; unrelated pressed pin excluded; pulse 50/51 ms boundary, release edge and explicit pulse-interaction characterization |
| `test_spi_buttons.cpp` | 43 | Distinct raw bytes, mask/inversion/cut, CS level and pending-transfer handling; known overrun/crash bodies read only |
| `test_pcf8574.cpp` | 31 | Two expanders 0x20/0x21 with distinct bytes, eight-expander packing, publication after last byte, NACK/bus-error retention, 250/251 ms, wrong-port callback guard |
| `test_shifter_analog.cpp` | 31 | Asymmetric ADC channels, H/sequential states, threshold equality, reverse routing, G27 bytes and user/gear offset; local/default policies separated from characterizations |
| `test_can_buttons.cpp` | 23 | Explicit eight-byte little-endian vector, masking/inversion, short/RTR/id cases and extended-frame known issue |
| `test_local_analog.cpp` | 32 | Six distinct ADC ranks, nonadjacent mask compaction, 10/12-bit arithmetic, manual/autocal path and explicit pin-versus-channel characterization |
| `test_can_analog.cpp` | 37 | Four signed words and separate next-ID channel group; short/RTR/id behavior, exact hardware-filter delivery, software rejection and filter-width characterization |
| `test_ads111x.cpp` | 62 | Config bit arithmetic from stated local map, asymmetric MUX-controlled samples, busy/poll/timeout/error/restart/destruction; P5-1 above |
| `test_analog_axis_processing.cpp` | 73 | Range endpoints, distinct per-axis ranges, learning boundaries, clipping, independent double recurrence, reset/state isolation, raw/config/flash interfaces; skipped crash bodies read only |

Independent arithmetic spot checks: PCF bytes F0/3C invert to 0F/C3 and pack to C30F; one pressed bit per expander produces 8040201008040201. CAN bytes 01/23/45/67/89/AB/CD/EF produce EFCDAB8967452301. Local 12-bit ADC multiplies by 16 and recentres by 32767 (0→−32767, 2048→1, 4095→32753). ADS single-ended endpoints map 0→−32767 and 32767→32767 under the stated local mapping; differential EDCC is −4660. The lowpass reference uses an independent direct recurrence/state representation; its Fc/Q and the 5% scale margin are implementation settings, with margin behavior appropriately characterized rather than treated as an independently verified product policy.

## Execution and selective experiments

Default binary: `/home/ben/Documents/OpenFFBoard/testing/ffb-test-build/codex-baseline-default/ffb_target_tests` (no rebuild). Exact source filter was the comma-separated list `*test_<stem>.cpp` for all twelve table stems. Explicitly excluded two deliberate UB cases: `a read of 8 bytes stays inside the receive buffer of the source` and `buttons of a source beyond the 64th button do not change the first buttons`. Existing crash skips were retained; `--no-skip` was not used.

- Phase-filtered run: **397 executed, 397 doctest passes, 0 unexpected failures**, including **14 expected-failure cases** (383 ordinary passes). There are 405 selected catalogue cases: six existing crash skips and the two explicit UB exclusions above. XML reports 1,440 skipped overall because 1,432 cases outside the source filter also count as skipped. Evidence: [/tmp/codex-p5-phase.xml](/tmp/codex-p5-phase.xml), stderr beside it.
- P5M1 repair experiment: one selected control/repair case each; both exit 0; first failure changes from intended line 290 to unrelated line 279 (P5-1).
- P5M2 offset mutation: change CanAnalog destination from `buf[i + packet*4]` to `buf[i]`. Original selected case `the axes 5 to 8 are received in the frame with the next CAN id` passes; mutant exits 1 at `test_can_analog.cpp:134`, the exact vector {1,2,3,4,5,6,7,8}. **Killed by the named observation**, establishing that second-frame delivery does not merely pass on unchanged defaults.
- P5M3 wrong-port guard mutation: remove only `port != &this->port` from PCF completion guard. Original `a receive completion of another I2C port during a read changes nothing` passes; mutant exits 1 at `test_pcf8574.cpp:311` when the expected real completion is no longer available. **Killed by subsequent transfer-state observation**, rather than by an asserted recipient counter. The selected case resumes the real port with two distinct devices; this supports the guard test while leaving interrupt-preemption fidelity unproven.

The combined JSON records exact before/after expressions, filters, totals and failing assertions. No whole-mutant-suite mutation score is claimed. No random-order or sanitizer run was repeated in this bounded assignment; coordinator-supplied default/random/sanitizer evidence is separate.

## Additional limits and follow-up

- The shifter bug71 case at line 408 checks `selected > 0` after requesting CS2; this accepts both nondefault CS indices 1 and 2. It detects the present pin0 bug but does not by itself establish the exact requested destination. This is a local assertion-scope weakness, not a newly executed mutant or independently established firmware defect. A future fix/regression should assert index1 and unchanged unrelated pin ownership.
- The interrupt ADS register-read helper case inspects transfer bytes but does not assert the caller buffer; the later source conversion and idle-read cases do observe delivered values. Do not treat that helper case alone as proof of destination delivery.
- CAN rejection assertions differ in strength: `canReceive(...)=0` proves filter rejection, whereas the additional accepted other-filter frame proves source-level id rejection. Ordinary short-frame CanButtons checks start from zero but use all-set incoming bytes, so removing its guard would distinguish a changed value; CanAnalog additionally establishes nonzero prior state for empty/RTR frames.
- Teardown/reset properties rely on TargetFixture and shared host bus models. Their relevant use and constraints were traced, not the full HAL/RTOS implementations independently revalidated. Simulated ISR order cannot establish hardware concurrency, electrical pullups, sensor timing or physical calibration correctness.
- Sensor specifications are missing from this assignment's independently verified references; no register map or frame was invented. The review supports internal host stimulus/observation and arithmetic effectiveness, with protocol-level conformance left qualified.
- No permanent test or protected fix was authored. P5-1 merits a bounded Codex-owned regression/proposal with explicit owner approval for any shared/Claude edit. This report does not constitute full phase sign-off or endorse historical 100% line coverage as assertion adequacy.

## Final body-reading completion

On 2026-11-06 fresh Astra QC read all778 lines/73 bodies of test_analog_axis_processing.cpp and complete production implementation/declaration, closing the truncated-read limitation above. Independent safe file run:71ordinary passes,0failures,two existing crash skips. No additional confirmed defect. Exact scope, observations and evidence are recorded in FINAL_REVIEW_QC.md; this completes assigned test-body reading without certifying missing sensor specifications or hardware fidelity.
