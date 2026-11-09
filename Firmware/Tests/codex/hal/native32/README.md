# P12 standalone real ST HAL gate

From the repository root, with the installed Linux Clang multilib and llvm-cov:

```sh
python3 -B Firmware/Tests/codex/hal/native32/verify_native.py --build ../ffb-test-build/codex-p12-native32-observed
python3 -B Firmware/Tests/codex/hal/native32/verify_native.py --build ../ffb-test-build/codex-p12-native32-f411 --device F411RE
python3 -B Firmware/Tests/codex/hal/native32/verify_native.py --build ../ffb-test-build/codex-p12-native32-ubsan --ubsan
```

Full assertions: configure the standalone CMake project with `-DP12_FULL_ASSERT=ON`, build, then run `run_native.py`. `P12_FULL_LL=ON` also tests supplied optional LL ADC C APIs; production builds without it must keep their inactive accounting separate. Real C coverage does not include model/test C++ in its denominator. See `Tests/codex/p12/NATIVE_MODEL.md` for precise fidelity limits and `COMPLETION_TRACKER.md` for remaining work; P12 is not yet declared complete.

The owner-approved shared CTest entry is enabled by `-DFFB_TESTS_ST_HAL=ON` when configuring `Firmware/Tests`. It invokes the same gate in its own build beneath that selected out-of-tree directory. The option defaults OFF because this gate requires installed Linux/Clang 32-bit support. Missing requirements fail the requested gate; nothing is installed or downloaded.

`run_native.py` clears stale counters and rejects diagnostics, unexpected raw failures, skipped/empty cases, or a wrong known-issue failure. `test_evidence.py` uses stdlib unittest to verify failure handling. `coverage_native.py` retains all compiled HAL files, including compiler-proven inactive units, and rejects missing notes/counters/collector output. Evidence, logs and hashes remain in the selected build.

The Clang kernel-address callbacks are an access-observation ABI, **not an ASan runtime**. `P12_UBSAN=ON` enables the actual undefined-behavior sanitizer. The earlier 64-bit harness has its own genuine ASan/UBSan gates. No board/hardware access, sleeps, firmware patches or vendor patches are used. Mutation/repair experiments are isolated sibling copies.
