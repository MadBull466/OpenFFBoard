# Phase I0 report: integration track infrastructure

*2026-11-18. Branch `add-test-suite`, phase commits `084835fc` .. this report's commit (local at the time of writing; pushed to the fork up to `f4f61e7e` before I1). Written by the lead (Opus 5.5) to be read without the conversation or other files; links point to the details.*

## 1. Result

Phase I0 is complete and waits for the owner's approval. The host integration test executable `ffb_integration_tests` runs the real firmware chain USB HID PID -> `HidFFB` -> `EffectsCalculator` -> `Axis` -> `TMC4671` on the audited fakes, with the tools, agents, hooks and gate the later phases need. The infrastructure requirements F1-A1 to F1-A5 of the F1 charter are covered.

| Exit criterion (plan section 8) | Status | Evidence |
|---|---|---|
| F1-A1 to F1-A5 green in the 64-bit and 32-bit builds | met | full gate, section 3 |
| All other executables' compile commands unchanged | met | compile check equal in all 8 variants (references recorded from builds with the integration executable switched off) |
| `check_baseline.py` OK | met | all 8 variants |
| ctest green in default, m32, san, clang, nodsp | met (also clang-san, nodsp-san, cov) | section 3 |
| Fake audit report written, every fix approved by the owner | met | [../charters/audit/](../charters/audit/), fixes A-1 to A-3 approved 2026-11-18 |
| Reviewer sign-off | met | an independent `integration-reviewer` pass on every work order from I0-3 on; all blocker / major findings fixed, see section 4 |
| Documents updated | met | `PROGRESS.md`, `HANDOFF.md`, `COVERAGE_AND_SCOPE.md`, `AGENT_PLAN.md`, `Firmware/Tests/README.md`, generated `BUG_REPORT.md` |

## 2. What was built (work orders)

| WO | Content | Commit |
|---|---|---|
| I0-1 | Five integration agents; PreToolUse / SubagentStop hooks enforcing the hard rules (protected paths, push / config / remote rules, commit identity and gate evidence, out-of-tree builds, no installs); `CLAUDE.md` "Integration track" section | `084835fc`, `c6f815c4` |
| I0-2 | Audit of the shared fakes in four groups (TMC4671, SPI / GPIO, ADC / timers / flash, USB / RTOS) | `096f43d3` |
| I0-3 | Owner-approved fixes of three shared fakes (A-1 SPI error injection, A-2 `Flash_Dump`, A-3 RTOS delay and priorities); 4 coverage cases edited, 9 added, no other coverage result changed | `096f43d3` |
| I0-4 | `integration.cmake` (one `include()` line in `CMakeLists.txt`, option `FFB_TESTS_INTEGRATION`), ctest entries, the gate `it_gate.py` (82 unit tests) | `096f43d3` |
| I0-5 | `IntegrationFixture` (bring-up like the board, resumable threads, teardown fault and leak checks) and `ChainScheduler` (fixed per-ms order, timers from the firmware-written registers); F1-A1, F1-A4 | `9fccde8f` |
| I0-6 | HID report-descriptor parser (HID 1.11) and PID report builder (PID 1.01) working from the bytes the device sends over USB, never from firmware structs; F1-A2 | `1130d0ce` |
| I0-7 | `WheelPlant` (exact per-step solution, friction, torque gating and 0x6B handling per the TMC4671 datasheet), `OscillationDetector`, `FrequencyResponse`; F1-A3, F1-A5 | `7f106955` |
| I0-8 | `run_all_tests.py` with the integration executable and the T-1 fix; generated integration bug report; coverage section "Integration (F1 chain)"; `test_report` dependency | `77e4a25b` |
| I0-9 | Compile references for all variants, full phase gate, this report, `Firmware/Tests/README.md` section | this commit |

## 3. Full phase gate (`it_gate.py --wo I0-9 --full`, 2026-11-18)

| Variant | ctest | `check_baseline` | Integration cases | Compile check |
|---|---|---|---|---|
| default | 13 / 13 | OK | 102: 98 passed, 4 known issues | equal (828) |
| m32 (32-bit) | 13 / 13 | OK | 102: 98 passed, 4 known issues | equal (823) |
| san (ASan + UBSan) | 13 / 13 | OK | 102: 79 passed, 23 skipped | equal (828) |
| clang | 13 / 13 | OK | 102: 98 passed, 4 known issues | equal (823) |
| clang-san | 13 / 13 | OK | 102: 79 passed, 23 skipped | equal (823) |
| nodsp | 12 / 12 | OK | 102: 98 passed, 4 known issues | equal (488) |
| nodsp-san | 12 / 12 | OK | 102: 79 passed, 23 skipped | equal (488) |
| cov | 13 / 13 | OK | 102: 98 passed, 4 known issues | equal (828) |

Scope check OK; evidence `../ffb-test-build/it-gate/I0-9/20261118T193126Z.json`. No build warning in a file of the work order. The 23 skips in sanitizer builds are the cases that delete the firmware main class (coverage bug 24, `IT_DELETES_FFBHIDMAIN`) and the IB 3 case (`KNOWN_ISSUE_UB`); run by hand with `--no-skip` and `ASAN_OPTIONS=new_delete_type_mismatch=0` (WO I0-5), all passed except IB 3, which ASan reports as heap-use-after-free.

**Integration coverage of the F1 chain files** (`coverage_report.py` on the `cov` build, objects of `ffb_integration_tests` only): 1429 of 4470 lines (32.0 %). This is the starting point of the track, not a phase-I0 target: I0 brings the chain up but sends no force-feedback effects (effects come with I1). Per file and per CMSIS-DSP function: `PROGRESS.md` "Current state" and the coverage output. `arm_sqrt_f32` and `arm_pid_f32` are inline and cannot be measured by gcov. Classifying every unexecuted chain line is an I5 task.

## 4. Review findings and how they were handled

Every work order from I0-3 on was reviewed by `integration-reviewer` (Opus 5.5) in a fresh context, with mutations in a copy outside the checkout. Majors per WO, all fixed before the commit:

| WO | Major findings (fixed) |
|---|---|
| I0-3 | The first RTOS delay fix approximated delays in restarting tasks (work ran twice per period, two delays livelocked): replaced by a kernel fault; every delaying thread must be resumable |
| I0-4 | The first gate could print PASS wrongly (incremental builds hid warnings, untested pass / fail logic, stale build folders trusted, reduced runs accepted): fixed, reduced runs are now `PARTIAL` and refused by the hooks |
| I0-5 | Faults during teardown unchecked; F1-A1 did not check what the chip received; the TMC wake-up count was not tied to the TMC thread: fixed, each proven by a mutation |
| I0-6 | Duplicate report IDs undetected; an optional PID report asserted as mandatory: fixed |
| I0-7 | Friction with damping / spring untested; counts per revolution checked only against itself; detector missed a limit cycle after a decay: fixed with hand-derived values; detector now classifies on the trailing window |
| I0-8 | The first T-1 fix would have hidden a real crash fix; two inline DSP functions vanished silently: fixed |

## 5. Findings: register and questions

All faults are listed in the generated [BUG_REPORT.md](BUG_REPORT.md) (source: the registers in `PROGRESS.md`); questions in [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md).

| # | Finding | Status |
|---|---|---|
| IB 1 | FFB timer period 1001 us instead of 1000 us (`FFBHIDMain.cpp:256`, ARR+1 counts): 999 Hz while the effects code is told 1000 Hz | open, `KNOWN_ISSUE` |
| IB 2 | TMC external-encoder timer 257 us (3891 Hz) instead of 250 us (4 kHz) (`TMC4671.cpp:3156-3158`, PSC +1 instead of -1) | open, `KNOWN_ISSUE` |
| IB 3 | `HID_CommandInterface::Run` reads freed memory when replies are queued during its wait (`HidCommandInterface.cpp:39-46, 159`); in the default build 5 of 6 replies were lost | open, `KNOWN_ISSUE_UB` |
| IB 4 | RAM Pool Available declared Array instead of Variable in the Block Load report (PID 1.01 §5.11.2) | open, `KNOWN_ISSUE` |
| INR 1 | After a runtime driver selection the motor stays off until a USB re-mount, which then runs it with motor type NONE | owner question IQ6 |
| INR 2 | PID State report without Effect Block Index | IQ9 |
| INR 3 | Direction as ordinal-usage values instead of Generic Desktop axes | IQ10 (low) |
| INR 4 | Block Load index 0 on failure (PID 1.01 §9 example) outside the declared range 1..40 | IQ11 |
| A-1 to A-3 | Shared-fake defects | fixed in I0-3 |
| T-1 | `run_all_tests.py` misreported three VESC cases | fixed in I0-8 |

Answered owner questions: IQ1 (own register), IQ2 (PID 1.01 supplied), IQ3 (nominal plant with sweeps), IQ4 (functional fakes), IQ5 (ENABLE_IN high kept), IQ7 (T-1 fixed in I0-8), IQ8 (HID 1.11 supplied). Open: IQ6, IQ9, IQ10, IQ11; none blocks I1.

## 6. Host-versus-target limits and modelling choices of I0

- Threads run in a fixed per-millisecond order, not preemptively; tests that depend on the order say so (plan 3.3).
- During bring-up each SPI frame takes 1 us of simulated time, because the driver's ADC calibration busy-waits on the tick; afterwards 0.
- The firmware is brought up through a USB re-plug after `axis.0.drvtype=1` ("board configured, PC connects afterwards"); see INR 1.
- The fixture reads the HID IN endpoint like a polling host; IB 3 has its own non-draining case.
- The fake TMC4671 stores registers only; `WheelPlant` applies torque mode, FOC PWM, ENI and the 0x5E limit itself and moves 0x6B by whole counts. After the fresh-flash bring-up the position source 0x51 is the electrical angle, so closed-loop cases (I3 / I4) must first configure an encoder with a mechanical source.
- The sanitizer variants skip the cases that delete the main class (coverage bug 24); see section 3.
- The hooks see command text only (no shell variables, no writes made by programs); the gate's scope check is the backstop.
- The i386 build does not prove ARM struct layout.

## 7. Next phase

I1, open-loop chain: requirement groups G-H (HID entry), G-E (effects to the chip) and G-X (axis) of the F1 charter, with the position scripted through 0x6B. Starts after the owner approves I0.
