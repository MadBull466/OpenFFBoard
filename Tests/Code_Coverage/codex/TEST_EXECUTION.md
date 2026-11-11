# Baseline execution and evidence

> Current status (2026-11-06): the authorized closeout is complete; see [CLOSEOUT_RESULTS](CLOSEOUT_RESULTS.md) and [final QC](reviews/CLOSEOUT_FINAL_QC.md#final-integrated-release-acceptance). Earlier counts and pending statements below are historical staged evidence.

Date: 2026-11-05; revision and results: [BASELINE.md](BASELINE.md). Run commands from `/home/ben/Documents/OpenFFBoard/testing/OpenFFBoard` unless noted. Do not use default preset build locations: they write inside the repository.

## Configuration and run commands

Each variant was configured in a new `../ffb-test-build/codex-baseline-<variant>` directory, built completely, and run with `ctest -V`. Flags below are the exact additional configure arguments; default CMake settings supply Debug and Unix Makefiles.

| Variant | Extra configure arguments | Build parallelism |
|---|---|---|
| `default` | none | `-j6` |
| `san` | `-DFFB_TESTS_SANITIZE=ON` | `-j6` |
| `nodsp` | `-DFFB_TESTS_USE_DSP=OFF` | `-j6` |
| `nodsp-san` | `-DFFB_TESTS_USE_DSP=OFF -DFFB_TESTS_SANITIZE=ON` | `-j4` |
| `clang23` | `-DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++` | `-j4` |
| `clang23-san` | `-DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DFFB_TESTS_SANITIZE=ON` | `-j4` |
| `cov` | `-DCMAKE_C_FLAGS=--coverage -DCMAKE_CXX_FLAGS=--coverage -DCMAKE_EXE_LINKER_FLAGS=--coverage` | `-j4` |

Example default commands:

```sh
cmake -S Firmware/Tests -B ../ffb-test-build/codex-baseline-default
cmake --build ../ffb-test-build/codex-baseline-default -j6
ctest --test-dir ../ffb-test-build/codex-baseline-default -V
```

CTest runs `tools/check_baseline.py` as an entry, so the original-case checker was executed in all variants. Every CTest invocation exits 8 because of CDX-001; every C++ executable and baseline entry exits successfully. Do not treat the coverage orchestration shell's final exit 0 (coverage report succeeded) as evidence of a successful coverage CTest gate.

Coverage command, run after the fresh coverage CTest workload without clearing or mixing any old counters:

```sh
python3 -B Firmware/Tests/Code_Coverage/tools/coverage_report.py --build-dir ../ffb-test-build/codex-baseline-cov
```

It exited successfully, emitted no stderr, and read 466 `.gcda` files. No tests were rerun in the coverage build before this measurement. The baseline checker entry itself necessarily runs the unit executable again, matching normal CTest behaviour.

Direct Python command (run with working directory `/tmp`):

```sh
python3 -B -m unittest discover \
  -s /home/ben/Documents/OpenFFBoard/testing/OpenFFBoard/Firmware/Tests/claude/python \
  -t /home/ben/Documents/OpenFFBoard/testing/OpenFFBoard/Firmware/Tests/claude/python -v
```

To reproduce the full-report Python path problem without forcing any skipped C++ crash/UB tests, import `Firmware/Tests/Code_Coverage/tools/run_all_tests.py` through `importlib.util`, create a `tempfile.TemporaryDirectory`, and call its public module helper `python_results(str(Path(module.TESTS_DIR) / 'python'), temporary_directory, 60)`. The recorded invocation returned zero results and a discovery problem. It did not execute the full report CLI or its no-skip C++ workload.

## Evidence locations

Build root: `/home/ben/Documents/OpenFFBoard/testing/ffb-test-build/` (outside Git).

Every `codex-baseline-<variant>/codex-baseline-evidence/` contains:

- `configure.log`: compiler detection, f16/DSP support, active exclusions.
- `build.log`: full fresh compile/link output and warnings.
- `ctest.log`: verbose test output, expected failures, counts, baseline result, Python discovery failure.

Default evidence additionally contains `python-direct.log`, `report-python.log`, and `results.json` (all variant case counts). Coverage evidence additionally contains `coverage.md` and `coverage.err`; the Markdown output is also retained as `Tests/Code_Coverage/codex/COVERAGE_BASELINE.md` for continuity. CTest's own `Testing/Temporary/LastTest.log` remains in each build tree.

Logs were initially captured under `/tmp/codex-baseline-*` and copied to these durable build-tree locations. C++ output can contain non-UTF8 bytes; parsers used UTF-8 replacement decoding for summaries. Raw logs were preserved unchanged. No raw specification content is copied to Codex documentation or evidence.

Summariser available:

```sh
python3 -B Firmware/Tests/Code_Coverage/tools/summarise_results.py \
  ../ffb-test-build/codex-baseline-default/codex-baseline-evidence/ctest.log
```

No full no-skip run, mutation testing, Release build, actual preset invocation, Windows/macOS/ARM build or physical hardware test was performed. Evidence does not establish those results.

## Approved CDX-001 correction

After the owner approved the two path edits, repeated the default configure/build/CTest commands above, capturing logs under `codex-baseline-default/codex-pathfix-evidence/`. Full CTest passed 9/9, including unchanged C++ counts, baseline preservation and 115 Python tests (two expected failures). Invoking the full-report Python helper using `Path(module.TESTS_DIR) / 'claude' / 'python'` returned 115 results without problems. No full no-skip report was run. See REGRESSION_RESULTS for comparison; BASELINE retains the original pre-fix results.

## Resumed P0 candidate verification (2026-11-06)

The reviewed patch was applied only to matching tool copies under `../ffb-test-build/codex-p0-resume/candidate/Firmware/Tests/tools/`, alongside unchanged imported helpers. No shared repository patch was applied.

```sh
python3 -B -m unittest discover -s Firmware/Tests/Code_Coverage/codex/python -v
P0_TOOLS_DIR=../ffb-test-build/codex-p0-resume/candidate/Firmware/Tests/tools python3 -B -m unittest discover -s ../ffb-test-build/codex-p0-resume/tests -v
```

Current:36 tests with23 expected failures; candidate:36 ordinary passes. Candidate test copy differs only by removing expected-failure decorator lines. Matching layout matters because the reporter imports unchanged companion modules. Logs are `current.log` and `candidate.log` beside the retained candidate copies. Historical `/tmp` evidence referenced elsewhere may have been cleared; retained sibling evidence and explicitly labelled historical observations are distinguished in HANDOFF and phase reports.

## Approved target registration

After the owner's explicit approval, applied `git apply Tests/Code_Coverage/codex/proposals/P2_TEST_INTEGRATION.patch`. Fresh configure/build/test steps were run for default DSP, no DSP and GCC sanitizer through the script retained at each `../ffb-test-build/codex-integrated-target-<variant>/codex-integration-evidence/reproduce.py`; exact argv in commands.json. It sets Debug, DSP ON/OFF and sanitizers ON/OFF, builds only ffb_target_tests with parallel 4, then runs:

```sh
ctest --test-dir <fresh-build> -R '^ffb_target_tests$' --output-on-failure
<fresh-build>/ffb_target_tests -ts=CodexCriticalErrorMotor,CodexDspFiniteOutputs -r=xml --order-by=rand --rand-seed=797
```

All three standard target gates pass; see REGRESSION_RESULTS for exact counts and sanitizer skips. P0/P1 proposals remain staged.

## Approved P0/P1 application verification

After “Apply both,” applied the two reviewed patches and removed the 23 resolved owned Python markers. Four configurations are reproduced by each `codex-p0-p1-integration-evidence/reproduce.py` under `../ffb-test-build/codex-integrated-target-{default,nodsp,san,nodsp-san}`. Exact argv and all logs are retained. Default builds all targets and runs full CTest 9/9; other variants build the unit target and run unit/baseline/Python CTest 3/3. Each uses an explicit build-local FFB_TESTS_EXCLUDE value retaining the normal original exclusion plus the unsafe nonbaseline ring identity. Shared defaults are unchanged.

The default evidence directory also contains direct Codex Python 36 ordinary-pass output, the applied-tool report over frozen coverage artifacts, and report_compat.py for real six-executable list-only inventories plus actual Python discovery. P0_P1_INTEGRATION.md contains exact counts and limitations. No forced-skip full report was run.
