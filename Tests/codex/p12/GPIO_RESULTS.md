# P12.1 first GPIO slice — 2026-11-06

Current extension: [REGISTER_MODEL_DESIGN](REGISTER_MODEL_DESIGN.md) and [REGISTER_MODEL_RESULTS](REGISTER_MODEL_RESULTS.md) add 36 access-observation/model cases in a separate C++ target. The real-C eight-case evidence below remains valid within its original storage-only scope; earlier missing-manual statements are historical.

The standalone owned harness compiles the real unmodified ST GPIO translation unit. Eight ordinary doctest cases, 231 assertions, pass with no skips or expected-failure markers. Scope/oracles: [GPIO_REQUIREMENTS](GPIO_REQUIREMENTS.md).

## Executed matrix

| Build suffix `codex-p12-gpio-` | Configuration | CTest | Focused XML, randomized seed 271 |
|---|---|---:|---:|
| default | GCC 16, F407VG, assertions disabled | 1/1 | 8 ordinary passes |
| assert | GCC 16, F407VG, full vendor assertions | 1/1 | 8 ordinary passes |
| disco | GCC 16, F407VG_DISCO | 1/1 | 8 ordinary passes |
| f411 | GCC 16, F411RE | 1/1 | 8 ordinary passes |
| san | GCC 16 ASan/UBSan, F407VG | 1/1 | 8 ordinary passes |
| clang-san | Clang 23 ASan/UBSan, F407VG | 1/1 | 8 ordinary passes |

No sanitizer diagnostic or child stderr occurred in accepted XML runs. No runtime suppression. A Clang/libstdc++16 warning from doctest 2.4.11's ciso646 include initially stopped the strict-warning build. Marking the dependency as a system include alone did not resolve it. Final CMake retains the warning and narrowly exempts C++ preprocessor warnings from -Werror with -Wno-error=#warnings. C and other C++ warnings remain errors. The retained previous attempt is a compilation failure, not test evidence.

## Discrimination

All mutations are copies under the sibling evidence directory, never vendor edits. Each runs all eight cases and must return normal doctest exit 1, with valid XML, exactly the listed failures and no stderr; a crash, compile failure or unrelated failing identity is rejected by the reproducer.

| Mutation | Failed case(s) |
|---|---|
| Always report input set | GIO-2 only |
| Shift reset request by 15 instead of 16 | GIO-5 only |
| Add pin 0 to every set request | GIO-4 only |
| Always set toggled outputs | GIO-7 and GIO-8 only |

These experiments discriminate selected plausible faults; they are not exhaustive mutation coverage or fresh independent review.

The unchanged existing safe default unit executable was also checked with tools/check_baseline.py: 318 original identities present, 317 checked / 1 excluded, RESULT: OK; 652 passed / 0 failed / 5 skipped and 339 additional identities with no failures. The existing original-axis/ring exclusions apply. This is a fresh baseline-checker run on the existing closeout binary, not a rebuild or fresh full P0–P10 gate. No shared source list changed, and the nested standalone HAL tests are not collected by existing unit globs.

## Reproduction and evidence

From the repository root:

```sh
python3 -B Firmware/Tests/codex/hal/verify_gpio.py
```

Requires the existing local CMake, GCC, Clang, Python and baseline build `../ffb-test-build/codex-closeout-gate-default/`. No dependencies are installed. Build/runtime writes need the required sibling-directory access; sanitizer verification ran with the established out-of-sandbox permission for LSan. The runner uses subprocess timeouts as a hang guard and does not sleep. Existing evidence is archived before reruns.

Evidence: `../ffb-test-build/codex-p12-gpio-evidence/` contains commands.json (argv/cwd/status), per-command stdout/stderr, source_sha256.json, summary.json, scratch mutant C files and an archival verify_gpio.py. Run the source-tree script above; the archival copy preserves the executed script but relies on its original location for repository discovery. Source hashes include owned harness files, real GPIO sources across targets, ST HAL/device headers, doctest and original baseline catalogue; they are rechecked at completion. Each build directory retains CMake cache and compilation database.

## Remaining work

Only three GPIO public functions are executed; no real-HAL coverage percentage has been collected. Init/deinit, full invalid-argument controls, lock sequences, EXTI, clock/event behavior and other HAL modules remain. The F411 run is not independent F411 hardware validation. Missing reference manual/errata remain pending. The standalone project has not been added to shared CTest; no shared file changed for P12 and no approval is requested merely for the owned harness.

Next: obtain the matching STM32 reference manual/errata for GPIO register-effect and EXTI/lock models, while extending independently sourced software-contract work where possible. P11 remains deferred; no commit, push or vendor repair occurred.
