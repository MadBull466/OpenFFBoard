# P12 HAL time and DMA-state results — 2026-11-06

The new real-C group executes 28 ordinary cases and one known-issue case. Requirements, seam choices and host limitations are in [TIME_STATE_REQUIREMENTS](TIME_STATE_REQUIREMENTS.md). The GPIO regression targets remain in every applicable build.

| Executable | Ordinary cases | Known-issue cases | Assertion result |
|---|---:|---:|---|
| p12_tick_tests | 11 | 0 | 34 pass |
| p12_delay_tests | 7 | 1 | 23 pass, 1 fails as expected |
| p12_dma_state_tests | 10 | 0 | 164 pass |
| p12_gpio_tests (retained) | 8 | 0 | 231 pass |
| p12_gpio_model_tests (retained, F407 only) | 36 | 0 | 115 pass |

No unexpected failures or skips in these selections. The expected assertion is P12-TICK-001, not an ordinary pass or a vendor repair. It is checked independently for exact marker, first expression, source location, one failed assertion and no script fault. See [FINDINGS](FINDINGS.md).

## Final matrix

Directories are `../ffb-test-build/codex-p12-time-<suffix>/`.

| Suffix | Configuration | Standalone CTest | Ordinary / expected-failure cases |
|---|---|---:|---:|
| default | GCC 16 F407VG | 5/5 | 72 / 1 |
| disco | GCC 16 F407VG_DISCO | 5/5 | 72 / 1 |
| f411 | GCC 16 F411RE software/header configuration | 4/4 | 36 / 1 |
| san | GCC 16 ASan/UBSan F407VG | 5/5 | 72 / 1 |
| clang-san | Clang 23 ASan/UBSan F407VG | 5/5 | 72 / 1 |

All focused XML runs use random order seed 271. No child stderr or sanitizer diagnostics in accepted runs. F411 omits the F407-specific C++ register model; no F411 peripheral/electrical fidelity is implied. This matrix supersedes earlier standalone 2-entry counts; earlier phase logs remain historical evidence.

## Seven fault controls and one bounded repair control

All changes are scratch C files in sibling evidence, not vendor edits. Fault controls must exit through ordinary doctest assertions with exactly the intended failed identities. A compiler failure, signal, timeout, stderr/sanitizer diagnostic, empty/duplicate/malformed result or unrelated failure is not accepted as discrimination.

| Scratch change | Observed discrimination |
|---|---|
| IncTick always adds 1 | Both selected 10/100 ms quantum cases fail |
| SetTickFreq omits rollback | Logical previous-frequency case fails |
| InitTick ignores lower configuration failure | Frequency rollback and priority-preservation cases fail |
| Delay exits its loop early | Five ordinary minimum-duration/boundary cases fail; script controls remain passing |
| Delay adds quantum to HAL_MAX_DELAY | Maximum-delay ordinary control fails |
| DMA stores M1-half callback in half slot | Named-slot registration case fails |
| DMA unregister-all retains abort callback | All-slot unregistration case fails |
| Scratch wider elapsed accumulator | All ordinary delay assertions pass; P12-TICK-001 intended assertion passes and only its should_fail marker flips, as required |

The scratch wider accumulator is a bounded repair discriminator, not an endorsed vendor patch or proof of arbitrary scheduling/target behavior. Ordinary default-frequency large-delay and normal rollover controls establish the clock script can represent successful long waits. The independent scripted elapsed-time accumulator is validated separately, including rollover and fault recording at exhaustion.

The marker exposes an actual early return in the bounded real-C HAL execution: at 10 Hz, a requested 4294967200 ms wait returns after sampled 100 ms because the added quantum wraps the uint32_t wait to 4. No directly affected project caller was found in the lexical FFBoard/board-Core scan. The production HAL remains unchanged.

## Reproduction and durable evidence

From the repository root:

```sh
python3 -B Firmware/Tests/Code_Coverage/codex/hal/verify_time_state.py
```

Requires existing local CMake/GCC/Clang/Python and the earlier safe baseline build. No downloads or dependencies added. Writes to the required sibling build root; sanitizer runs used the established permission outside the LSan-restricting sandbox. Subprocess timeouts bound hangs; tests never sleep.

`../ffb-test-build/codex-p12-time-evidence/` retains commands.json (exact argv/cwd/status), per-command stdout/stderr, source_sha256.json, summary.json, all seven fault copies, wide-delay-repair.c and archival scripts. Source hashes include owned harness inputs, real HAL/GPIO/DMA sources, HAL/device/CMSIS core headers, doctest, TestHelpers and the original baseline catalogue. They are checked at completion. Run the source-tree script, not its archival copy, because repository discovery depends on its location. Previous runs are archived before a rerun.

The original-case checker freshly passed against the existing safe closeout default executable: all 318 identities present, 317 checked / 1 excluded, 652 passed / 0 failed / 5 skipped, 339 additional identities with no failures. Existing original-axis and unsafe-ring exclusions remain. This is an existing-binary baseline check, not a fresh shared-suite rebuild/full P0–P10 run.

## Remaining scope and next step

DMA stream address calculation, FIFO/flags, active polling, abort/IRQ callbacks and actual transfers remain untested. Their target uint32_t pointer conversions cannot safely operate on arbitrary 64-bit host pointers. Warnings for those conversions remain visible, exempted from -Werror only for the C DMA-state target. The tick CPU boundary also remains explicit: real SysTick/NVIC/RCC execution and interrupt scheduling are not simulated. Cache/init/deinit/device-identification paths are compiled but not exercised.

Next: select a justified address/access strategy before active DMA register tests, or continue other reference-backed HAL software contracts independently. Preserve the real-C, GPIO-observer and time/state gates. No numeric line-coverage percentage, fresh independent-agent QC, complete P12 closure, shared CTest integration or hardware equivalence is claimed. No vendor/firmware/Claude/original-baseline/shared infrastructure edit, commit or push occurred for this group. P11 remains deferred.
