# P2 independent review

2026-11-05 · GPT-6.1 Sol · high reasoning effort. Review only; production, Claude-owned tests, shared infrastructure and baseline objects were not changed. P11 excluded. Coordinator-assigned finding identifier: CDX-010 (original reviewer identifier P2-1).

## Finding CDX-010 — critical-error tests accept a flag-only repair that leaves torque applied

**Priority: high test gap.** `Firmware/Tests/claude/cases_target/test_ffbhidmain.cpp:1059–1081` contains the two bug-39 cases. Each checks only `get(wheel,"estop")==1`. Neither selects a motor driver, creates nonzero torque, checks the driver emergency state, or runs the next timer update. Both correctly expose the existing wrong-reset defect at lines 1068 and 1080, but do not establish motor inhibition.

The requirement is a functional safety invariant already stated by the test names and firmware emergency-stop API: a new critical error must zero pending motor torque and put the motor into emergency stop; later control ticks must not reapply it. This does not require an unsupplied HID PID specification. `FFBHIDMain.cpp:296–299` delegates to all axes, `Axis.cpp:617–623` explicitly sends zero before invoking the driver's emergency stop, and `FFBHIDMain.cpp:153–159` gates subsequent torque updates. Merely gating future updates leaves the previously sent torque latched at the driver interface.

An isolated scratch experiment compared three versions of `FFBHIDMain::errorCallback` at line 363:

| Version | Scratch replacement | Existing two bug-39 cases | Entire existing target executable |
|---|---|---|---|
| Control | `this->emergencyStop(true);` | 2 passed as expected; first failures 1068, 1080 | 1819 passed / 0 failed / 18 skipped |
| Proper repair | `this->emergencyStop(false);` | all 12 assertions pass; both should-fail markers flip | 1817 passed / 2 failed / 18 skipped |
| Unsafe repair | `control.emergency = true;` | identical 12/12 assertions and marker flips | identical 1817 / 2 / 18 |

Thus the existing *whole target suite*, not only the two named cases, gives the same repair signal for the unsafe repair. This is repair discrimination, not a claim that a surviving ordinary mutant keeps the baseline gate green: the known-issue markers deliberately fail when their assertions become true.

A scratch-only probe used `FFBWheel::usbInit`, mock driver/encoder selected through the real chooser, power 32767, effect ratio 255, PID constant force 10000 and 1100 real timer/main-task updates. It required motor torque >9900 before `ErrorHandler::addError(overtemp, critical)`, then observed `estop`, motor torque and driver emergency state immediately and after three more timer updates. Proper repair: 1 case, all 1123 assertions pass. Unsafe repair: `estop` is 1 but torque remains **9999**, driver emergency stays false, and torque is still 9999 after three ticks. Original control: `estop` is 0, driver emergency false, and torque returns to 9 after three ticks. The probe initially omitted `usbInit` and its positive control failed at -4999 (known direction-mask setup); that setup was corrected before the reported experiment. No result from that invalid first probe is used as defect evidence.

Recommended later test work: extend the critical-error observations to motor-side torque/call order and subsequent ticks, using nonzero pending force. Include error-before-E-stop and E-stop-before-error and a release/reset check while the critical condition remains; recovery policy beyond the stated emergency invariant remains an owner question. Existing state-only cases can remain focused, with separate motor integration cases.

Evidence: `/tmp/codex_p2_experiment.py`; sibling `../ffb-test-build/codex-p2-p4-review/p2_results.json`, `{control,proper_fix,flag_only_fix}_{critical,full,probe}.log`. Scratch compile uses baseline compile commands and substitutes only scratch objects at link time; the baseline dependency-file linker option is removed. No sanitizer was used for this main-class experiment because the existing main-class destructor defect is already a documented sanitizer skip.

## Assessed scope and positive evidence

All seven assigned P2 test files were read, including setup/helpers and every case body (5,553 source lines). These are 398 registered cases in the DSP default build, 396 executed and two existing crash skips. Counts by file below show executed cases; axis_config and ffbhidmain each have one additional skipped crash case.

| File under `Firmware/Tests/claude/cases_target/` | Executed | Assessed behavior |
|---|---:|---|
| `test_target_setup.cpp` | 12 | real-axis chooser binding, scaling, PID-to-motor smoke, explicit emergency-zero path, timer/main smoke, default-scale known issue |
| `test_axis_torque.cpp` | 103 | power/ratio/sign/clip, fade, explicit emergency zero and order, slew recovery, endstops/out-of-range, metrics, mechanical effects, expo/equalizer, speed limiter, hands-off |
| `test_axis_config.cpp` | 80 | commands/flags, asymmetric flash words and round trip, limits, chooser errors, encoder/driver interactions, zeroing/metrics and suspend/resume |
| `test_axes_manager.cpp` | 27 | axis lifetime/count, recipient-specific encoders, all-axis torque/inhibit/recovery, scaled report data, samplerate |
| `test_ffb_integration.cpp` | 55 | real PID/effects/axis/motor path, ratio/sign/timing, asymmetric conditions and axes, endstop mixing, main-class force integration |
| `test_ffbhidmain.cpp` | 74 | report bytes/buffering/rates, timer-to-task updates, software and GPIO E-stop, errors, suspend/resume, input commands |
| `test_main_classes.cpp` | 45 | class/chooser wiring, CDC execution/bounds, descriptor selection, wheel/joystick/ext setup |

Production source inspection was targeted, not a full line-by-line firmware audit: all `FFBHIDMain.cpp` and `AxesManager.cpp`; `FFBHIDMain_commmands.cpp` command entry and estop/ffbactive; `Axis.cpp` driver/encoder selection, emergency/USB methods, mechanical effects, metrics, torque composition, limiters and filter/rate updates; mock motor/encoder and fixture update logic. Absolute tuning magnitudes copied from firmware constants protect tuning stability, not independent PID compliance. Ratio/linearity/restoring-force expectations and time arithmetic are stronger independent invariants; unavailable HID PID interpretation remains attributed to existing tests.

The local `EstopOnlyDriver` is a useful anti-mock control: it does not zero torque during its emergency method and records whether the explicit zero precedes that call (`test_axis_torque.cpp:315–349`). The real-main case at `test_ffbhidmain.cpp:670–688` exercises the actual updateControl emergency gate with a pending fade-generated torque change, avoiding reliance solely on the fixture's own emergency gate. Integration tests assert full motor magnitudes after startup fade, including 10000 full-scale, power and effect-ratio multiplication, 20000 clipping, opposite-force cancellation, and asymmetric axis conditions. No defect was confirmed in those checked assertions.

The suspend entry in P2 directly calls `usbSuspend`/`usbResume`; real TinyUSB callback binding belongs to P4/P9. USB-disabled torque policy and resume's driver-emergency mismatch are already explicit NEEDS REVIEW 40 and 38; this review does not silently promote them to settled requirements. The fake task restart model re-reads the E-stop at each restart; the debounce known-issue cases deliberately avoid subsequent task activation, as documented at lines 937–939. Real preemption and motor-driver integration remain outside this phase.

## Execution and expected-failure assessment

Default baseline binary: `../ffb-test-build/codex-baseline-default/ffb_target_tests`. Exact phase command:

```
ffb_target_tests -r=xml -sf='*test_target_setup.cpp,*test_axis_torque.cpp,*test_axis_config.cpp,*test_axes_manager.cpp,*test_ffb_integration.cpp,*test_ffbhidmain.cpp,*test_main_classes.cpp'
```

Result: 396 executed, zero unexpected failures; `/tmp/p2_run.xml`. Doctest prints 1441 skipped because that includes all filtered-out cases, not 1441 P2 skips. The phase's only two default skips are stored out-of-range filter profile (bug 31) and report-rate index (bug 37); neither was forced. Whole target scratch control additionally passed 1819/0/18. Coordinator-supplied seed-211/797 whole-executable results are separate supporting evidence, not runs performed by this reviewer.

Executed known-issue first failure locations were checked in the XML against their case bodies: config 455/480/704/824/980/1004; torque 216/308/374/392/676/1213/1223; main 752/806/969/988/1068/1080; setup 102/174. They point at the stated defect observations in this execution. Passing a REQUIRE in one run does not guarantee a later setup failure cannot be absorbed by `should_fail`; no stronger general claim is made. The two critical cases have the material repair gap above, not a wrong-first-failure at baseline.

No full mutation campaign, no-DSP rerun, sanitizer forced-UB/crash run, physical-board check, or independent re-derivation of every command policy was done here. No original baseline test, shared fixture, protected source, Git state or Python-path correction was changed.
