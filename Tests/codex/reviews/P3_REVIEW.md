# P3 flash and persistence review

2026-11-05. Reviewer: primary Codex coordinator. Source/assertion pass of all five Claude flash test files (2,766 lines, 159 registered/executed cases). This is a bounded host-suite review, not a physical flash or complete firmware safety certification. No protected source/test changed.

## Scope and independent expectations

Read all case bodies/helpers in `claude/cases_flash/test_{eeprom,eeprom_addresses,flash_helpers,flash_setup,persistent_storage_flash}.cpp`; `support/FlashFixture.h` and `HostFlash.h`; the 639-line `platform/src/host_flash.cpp`; `PersistentStorage.cpp`; flash helper read/write/default/dump/OTP implementations; EEPROM initialization, page verification/read/write/format and transfer logic as needed for the assertions. Cross-checked the fake's HAL program, unlock/lock, wait/error and sector-erase/cache-flush control flow against the vendored F407 HAL source functions. The HAL itself was not modified or executed.

Independent functional expectations are: newest committed key/value wins, incomplete record does not become committed before its virtual address is programmed, a page transfer preserves listed variables, changes remain after simulated reboot, unchanged writes do not program, failed operations report failure and retain earlier data, delayed restores occur once in registration order, and OTP slot writes do not alter neighbours. Exact F407 page/record addresses are already defined by the target headers and compiled EEPROM interface; none was invented for this audit. Address-table tests separately compare generated constants and tables, uniqueness, sentinel bounds and real transfer preservation; round trips alone are not their only oracle.

The power-cut tests derive a reference operation count, arm a cut for each selected program/erase point, require that the cut occurred, preserve flash across the simulated reset, and compare retained values to independently tracked old/new maps. In-loop reset helpers check unmodelled accesses before clearing recordings. Recovery tests cover both page-transfer directions, while V5/V6 expectations remain explicit known issues. The independent reference is a committed-value model and raw retained memory, not merely the fake's selected return value.

## Executed evidence

The unmodified default flash executable completed all 159 cases with zero unexpected failures in both random-order runs (seeds 211 and 797; coordinator evidence in `/tmp/codex-review-order/`). Existing baseline normal/sanitizer evidence is recorded separately in BASELINE, not rerun as part of this mutation experiment.

Four single-fault experiments copied one source file at a time into `../ffb-test-build/codex-p3-review`, compiled using the default flash target's recorded flags and relinked using immutable baseline objects. The link dependency-output option was removed to avoid baseline writes. Tests ran with the ordinary skip policy. Each entire 159-case flash run failed with exit 1; the named discriminating cases failed rather than only an incidental crash:

| Mutant | Scratch fault | Unexpected failed cases | Discriminating ordinary case |
|---|---|---:|---|
| M1 | Flash_Write treats equal rather than different existing data as requiring a write | 8 | `writing a different value appends a record with the new value and returns true`; unchanged-write and persistence cases also fail |
| M2 | EEPROM read scan stops before the first record's virtual-address slot | 62 | `EE_ReadVariable finds a variable in the first record slot`; first/newest value and reboot cases also fail |
| M3 | Omit data-cache disable before firmware cache reset | 4 | `Flash_Init leaves the flash locked and the data cache reset and enabled`; write/default/format cache checks also fail |
| M4 | Keep restoreDelayedFlag set after startup restore | 1 | `the startup callback restores the waiting handlers in registration order, each once` |

Script: `/tmp/codex_p3_mutations.py`. Durable mutation sources, compile/link logs, complete XML/stderr and `mutation_results.json` are under the sibling `codex-p3-review` build directory. EEPROM vendor code was mutated only in an isolated copy for this detection experiment; no vendor fix or working-tree mutation was made. This four-mutant result does not reproduce or validate the historical much larger mutation campaign.

## Findings and limits

No additional confirmed assertion/fake defect was established in this P3 pass. The four sampled faults are detected by ordinary tests, and the known EEPROM and OTP behaviours remain distinctly marked rather than asserted as universal correct hardware behaviour. No new P3 test is justified merely to duplicate those observations.

Important existing boundaries remain:

- The simulated torn-program masks and partial-erase byte ranges are selected deterministic models of indeterminate hardware states, not exhaustive brownout physics. Ordinary recovery loops deliberately limit some masks/header combinations; adjacent characterization/known-issue cases document excluded failure modes.
- Wrong-key unlock continuation, erase-while-locked result and alignment-error behaviour are explicitly model assumptions. Local HAL control flow was checked; the RM0090 hardware statements were not independently revalidated from a supplied manual during this pass. No new claim of silicon fidelity is made.
- Cache tests prove the intended disable/reset/enable calls and ordering in the model. The fake does not emulate stale ART cached reads or electrical flash timing.
- Some known-issue recovery tests include model/setup REQUIREs before the intended defect assertion. They passed in the recorded baseline and ordinary twin tests exercise cut reachability, but this does not eliminate the general `should_fail` setup-masking risk established in P1. No exhaustive repair-discrimination experiment was done for every V5/V6 case.
- Real concurrent flash callers, target exception/reset behavior, MinGW mapping and macOS remain unverified. The I2C EEPROM backend is outside this F407 EEPROM-emulation configuration; it was not silently added to scope.

Status: source/assertion pass and four fault-detection spot checks complete; final Astra technical QC pending. No infrastructure change or new test in this phase.

Resumption evidence note (2026-11-06): the original temporary reproducer was cleared between sessions. Durable per-mutant artifacts and JSON survive. A `reproduce.py` script reconstructed from that JSON is now saved in the same sibling directory; reconstruction did not rerun the mutations.
