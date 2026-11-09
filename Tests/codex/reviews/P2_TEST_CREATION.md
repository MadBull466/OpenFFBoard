# P2 permanent motor-inhibition regressions

2026-11-06 · GPT-6.1 Sol · medium effort · bounded writer assignment for CDX-010.
Ready for fresh independent QC; this writer does not approve the tests.

## Scope and requirement

Added `Firmware/Tests/codex/cases_target/test_critical_error_motor.cpp` (two cases, one local fixture; no new fake). This addresses the motor-side gap accepted in `INITIAL_QC.md` and described in `P2_REVIEW.md`. The emergency API contract is observable in `FFBHIDMain::emergencyStop`, `Axis::emergencyStop`, and the main update gate: stop an already-driven motor and prevent later ticks from reapplying force. It does not settle critical-error release/recovery policy or claim physical motor safety.

| Case in suite `CodexCriticalErrorMotor` | Role and oracle | Current default/no-DSP result |
|---|---|---|
| `an explicit emergency command stops applied motor force through later ticks` | Ordinary independent control: full-scale nonzero force followed by the public `estop=1` command must engage motor emergency, zero motor torque immediately and on each of three later real main-task ticks. | Ordinary pass; 1139 assertions |
| `a new critical error stops applied motor force through later ticks` | Same setup and motor oracles after public `ErrorHandler::addError(overtemp, critical)`. Existing bug 39 marker retained. | Expected failure; 1130 successful and 7 failing assertions |

The fixture reuses `TargetFixture`, `AxisFixture`'s public chooser registries, `PidHost`, `CommandHelpers`, and the existing recording `MockMotorDriver`/`MockEncoder`. It creates a real `FFBWheel` and calls `usbInit` before effect creation to establish the actual HID direction mask. Real public axis commands set power 32767 and effect ratio 255; PID starts a constant 10000 effect. After 1100 simulated 1 ms timer interrupts/main-task activations, setup requires torque in `(9900,10000]` and no driver emergency. This bound derives from requested force at full scale with rounding allowance, rather than a copied exact output.

`requireModelled()` requires zero unmodelled HAL calls and zero kernel faults before the first defect assertion and after each later tick. The **first failed assertion is line 67, `motor->emergency`**, a motor-interface observation, in original and flag-only repair runs. The immediate/later torque and `estop` observations follow it. The ordinary twin, isolated runs, unmarked scratch comparisons and first-failure XML inspection guard against unrelated setup failures being mistaken for bug evidence. As with every doctest `should_fail`, later regressions still require actual first-failure review; the decorator alone cannot guarantee that.

## Execution and repair discrimination

All evidence is durable under `../ffb-test-build/codex-p2-writer/`. `reproduce.py` reproduces compilation/link/focused comparisons; `checks_reproduce.py` reproduces full target, isolated and checker runs. `commands.json` and `checks_commands.json` contain exact argv arrays and working directories (no shell escaping ambiguity). Scripts use only Python stdlib and existing compilers/objects. They compile new objects with each immutable baseline's `flags.make`, relink its `link.txt` with an added Codex object and a separate output path, and remove the baseline linker dependency-file output option. Baseline sources, objects and executables are unchanged.

| Run | Default DSP | No DSP |
|---|---|---|
| Both permanent tests focused | 1 ordinary + 1 expected pass; exit 0 | Same |
| Each case in its own process | Same intended classification; exit 0 | Same |
| Focused random order seed 797 | Same; exit 0 | Same |
| Entire scratch-linked target executable, permanent markers retained | 1821 passed / 0 failed / 18 skipped | 1768 passed / 0 failed / 17 skipped |
| Original callback, new bug marker removed in scratch only | Ordinary control passes; critical fails 7 assertions; exit 1 | Same |
| Proper repair `this->emergencyStop(false);`, new marker removed in scratch only | Both pass; critical 1137/1137 assertions; exit 0 | Same |
| Flag-only repair `control.emergency = true;`, new marker removed in scratch only | Ordinary control passes; critical fails 8 assertions; exit 1 | Same |

Repair copies originate from `Firmware/FFBoard/UserExtensions/Src/FFBHIDMain.cpp` and replace only the call inside the new-critical-error branch in scratch source. Production firmware was never edited. Unmarked comparisons preserve every intended assertion, removing only the new `KNOWN_ISSUE` decoration from a scratch copy of the new test. With flag-only repair, driver emergency remains false and torque remains **9999 immediately and after all three ticks**. With original callback, immediate torque is zero due to its reset path, but motor emergency is false and later torque returns (4 then 9 at the second/third later ticks). This distinction explains why the first motor oracle checks emergency state and the remaining assertions independently check torque.

Sanitizer: `san.compile.log`, `san.link.log`, `san-focused.xml` and `san-random.xml` show successful ASan/UBSan compilation/link and **both new cases skipped** by the required `DELETES_FFBHIDMAIN` policy (existing bug 24). XML aggregate skipped counts also include filtered-out existing cases. No sanitizer runtime coverage of the new bodies is claimed; no skipped destructor UB was forced, and no sanitizer flag was weakened.

Exact top-level reproduction commands, from repository root (scripts already retained at these paths):

```sh
python3 -B ../ffb-test-build/codex-p2-writer/reproduce.py
python3 -B ../ffb-test-build/codex-p2-writer/checks_reproduce.py
git apply --check Tests/codex/proposals/P2_TEST_INTEGRATION.patch
```

Example focused command (the scripts record every compiler/linker invocation):

```sh
../ffb-test-build/codex-p2-writer/default -ts=CodexCriticalErrorMotor -r=xml
../ffb-test-build/codex-p2-writer/default-proper_fix-unmarked -ts=CodexCriticalErrorMotor -r=xml
../ffb-test-build/codex-p2-writer/default-flag_only_fix-unmarked -ts=CodexCriticalErrorMotor -r=xml
```

## Baseline and integration gate

`check_baseline.py` exits 0 in default, no-DSP and sanitizer configurations, against unchanged baseline binaries. All 318 original identities remain present. Default/no-DSP: 317 checked, one excluded; unit aggregate 645 passed / 0 failed / 5 skipped. Sanitizer: 315 checked, three excluded; aggregate 642 / 0 / 8. Logs: `{default,nodsp,san}-baseline.log`.

Every checker invocation explicitly excludes `requesting the forces of a not existing axis is rejected` and the nonbaseline unsafe ordinary case `typed access reads single bytes and counts elements` (CDX-008). Sanitizer additionally excludes the two documented spline cases. The second exclusion is deliberately CLI-only and does not change CMake policy; it accounts for one fewer aggregate unit pass than historical runs. Baseline counts are unaffected by excluding this nonbaseline case. Exact `--exclude-list` arguments are retained in `checks_commands.json`.

`Tests/codex/proposals/P2_TEST_INTEGRATION.patch` proposes only adding `codex/cases_target/test_*.cpp` to the existing target source glob. `git apply --check` succeeds. **The patch has not been applied; shared-file approval remains pending.** These tests have been executed through scratch linking and are not yet registered in normal CMake/CTest. No shared, Claude, baseline, firmware or vendor file was edited. No commit/push or dependency download occurred.

## Coverage and limitations

Covered: one wheel axis, an already-active constant effect, ordinary software emergency stop control, new critical-error callback, immediate motor interface and three later timer/main-task updates. Existing fake fidelity and reset policies apply; no custom peer or invented protocol frame was added. No line-coverage percentage or phase completion claim is made.

Not covered: error arriving during an already-active stop, clearing/recovery while an error persists, GPIO stop/debounce, real preemption, multiple-axis asymmetry, physical motor/driver electronics and target compiler timing/float differences. Both host math configurations are exercised; assertions concern recorded int16 torque/emergency state with rounding allowance. Whole mutation-repair suites are not rerun here: the prior reviewer already archived existing-suite repair equivalence, while this assignment compares the new identical intended assertions. Fresh QC should independently inspect fixtures, oracles, first failures and repair discrimination before integration approval.

## Coordinator integration update

2026-11-06: fresh Astra QC accepted both motor cases and the P9 guards; the owner explicitly approved registering the four target cases. P2_TEST_INTEGRATION.patch is now applied. Fresh normal target builds/CTest pass in default/no-DSP/GCC sanitizer; exact counts and retained skips in REGRESSION_RESULTS.md. The proposal-only statements above describe the writer submission before approval. No firmware repair was applied.
