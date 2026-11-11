# P12 GPIO register-model results — 2026-11-06

Thirty-six ordinary doctest cases / 115 assertions pass in the new observation target: 18 model controls and 18 public-HAL cases. The separate real-C storage target retains eight ordinary cases / 231 assertions. No expected failures, skips, exclusions or new vendor defects occur in these selections.

## Scope exercised

The HAL cases cover modeled set/reset/toggle output effects; input/output/alternate/analog configuration with independent field expectations; rising, falling-event and both-edge interrupt setup; selected deinitialization and preservation of another port's route; the lock protocol and subsequent protected configuration; pending-bit clearing before callback; unrelated/nonpending handler behavior; and full-assert rejection of empty masks, invalid uint32_t modes and unknown instances before register accesses. Invalid enum conversion was deliberately avoided to keep invalid-input tests within defined host C++ behavior.

Model controls separately exercise BSRR read-zero/set priority/neighbor preservation, PR write-one/zero behavior, valid/invalid/changed-mask lock sequences, locked nonzero/high alternate fields, lock reset, documented A/B debug defaults, reserved pull rejection and unsupported/unregistered/readonly/clock/erratum-dependent accesses. [REGISTER_MODEL_DESIGN](REGISTER_MODEL_DESIGN.md) records exact fidelity and language boundaries.

## Final matrix

| Suffix under `codex-p12-gpio-register-` | Configuration | Standalone CTest | Model XML | Real-C XML |
|---|---|---:|---:|---:|
| default | GCC 16 F407VG | 2/2 | 36/0/0 | 8/0/0 |
| disco | GCC 16 F407VG_DISCO | 2/2 | 36/0/0 | 8/0/0 |
| san | GCC 16 ASan/UBSan F407VG | 2/2 | 36/0/0 | 8/0/0 |
| clang-san | Clang 23 ASan/UBSan F407VG | 2/2 | 36/0/0 | 8/0/0 |

XML columns are passed/failed/skipped, run in random order seed 271. Both sanitizer configurations have no child stderr/diagnostics. Doctest's existing Clang/libstdc++16 ciso646 compile warning remains visible under the previously documented narrow preprocessor-warning exception. No runtime suppression. An initial Clang build caught an owned unused-value expression in an exception-control macro; an explicit void cast corrected it. That failed build is archived and is not successful test evidence.

## Nine discrimination controls

All mutations operate on sibling scratch copies. The runner requires eight or 36 distinct unskipped identities as appropriate, normal doctest exit status, exact failed identities and empty child stderr. Crashes, compiler failures, malformed XML, duplicate inventory or unrelated failures cannot count as a detected mutant.

| Scratch change | Failure |
|---|---|
| HAL omits middle low-key lock write | HAL lock protocol case only |
| HAL calls callback before pending clear | HAL pending/callback-order case only |
| HAL clears all pending lines | Same case, detecting lost neighboring request |
| HAL writes alternate mapping into wrong register bank | Cross-bank alternate case only |
| HAL deinitialization retains pull configuration | Own-route deinit case only; nonzero pull/type/speed setup prevents a vacuous pass |
| Model returns nonzero from BSRR read | BSRR read-zero control only |
| Model gives reset priority over set | Simultaneous set/reset control only |
| Model treats PR as ordinary RAM | Both model PR controls and HAL pending/callback case |
| Model ignores configuration locks | Four model/HAL cases covering zero/nonzero/high fields and public lock protection |

Each HAL mutation leaves model controls passing; each accepted run reports only the listed failed identities. Direct model tests are required rather than relying on HAL tests to validate their own fake. These are self-run bounded experiments, not exhaustive mutation coverage or fresh independent-agent QC.

## Baseline and evidence

The original-case checker was freshly rerun against the existing safe default closeout binary: all 318 original identities present, 317 checked / 1 excluded, RESULT: OK; 652 passed / 0 failed / 5 skipped and 339 new identities with no failures. Existing original-axis/ring exclusions are retained. This is not a rebuild or full P0–P10 rerun. No shared source list changed.

Reproduce from the repository root:

```sh
python3 -B Firmware/Tests/Code_Coverage/codex/hal/verify_register_model.py
```

Requires existing local CMake/GCC/Clang/Python and the prior safe baseline build. Writes only under sibling `../ffb-test-build/codex-p12-*`, with the required permission for build writes and sanitizer execution. Process timeouts bound accidental hangs; no sleeps or downloads.

Durable evidence: `../ffb-test-build/codex-p12-register-evidence/` holds exact argv/cwd/status in commands.json, per-command stdout/stderr, all nine scratch sources, source_sha256.json and summary.json. Earlier attempts are archived in previous-attempts/. Copied scripts are archival inputs; run the source-tree script so repository discovery is correct. Hashes cover owned harness inputs, real ST GPIO sources/headers, doctest and original baseline catalogue and are checked again at script completion. CMake cache/compile_commands remain in each build directory. Earlier eight-case-only results are historical; this matrix supersedes them for the now-expanded standalone CTest project.

## Remaining scope

No quantitative line-coverage percentage, ARM/device result, complete HAL/EXTI coverage or full P12 completion is claimed. Vendor weak no-op callback execution is not covered by the strong callback override. Model and C builds are separate evidence, not a merged production-language percentage. Shared CTest integration and fresh independent review remain pending future work. F411 storage results remain historical header/software checks; F411 has no register-model target without its own references.

The next P12 group is deterministic HAL time/state and DMA prerequisites, retaining the existing GPIO gates. Additional GPIO branch/reference/revision coverage can be added against explicit requirements; missing F407 manuals no longer block this work. Vendor, Claude, original-baseline and shared infrastructure files remain unchanged by this P12 group. No commit or push.
