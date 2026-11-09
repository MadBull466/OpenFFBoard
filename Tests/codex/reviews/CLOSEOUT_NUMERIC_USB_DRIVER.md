# Numerical, USB and driver closeout implementation

2026-11-06 · GPT-6.1 Sol writer · assignment
`/root/closeout_numeric_usb_driver`. Implements the bounded P9/P10 requirements
in [CLOSEOUT_REQUIREMENTS.md](CLOSEOUT_REQUIREMENTS.md). Independent final
acceptance belongs to the technical lead. P11 remains excluded.

## Delivered checks and scope

| Requirement | Permanent change | Evidence |
|---|---|---|
| CDX-014 finite oracle | `codex/support/NumericAssertions.h`, three ordinary unit cases | Reject NaN and either infinity in either operand/tolerance, negative tolerance, finite-subtraction overflow and finite out-of-bound error; accept equality and inclusive boundary |
| Ordinary PID equation | `codex/cases_target/test_closeout_dsp_pid.cpp` | Nine asymmetric signed dyadic samples, fixed Kp/Ki/Kd, independent positional equation using input histories; each sample compared through finite oracle; both output signs observed |
| USB setup faults | Shared `support/UsbFixture.h` successful-enumeration guard; `codex/cases_usb/test_closeout_usb_guard.cpp` ordinary derived setup | Clean enumeration and later clear pass; injected public-DCD fault fails shared guard before derived clear; guard-off scratch comparison reaches clear |
| CDX-015 | Shared `fakes/FakeOdrivePeer.h` comments and [ODRIVE_POLICY.md](../ODRIVE_POLICY.md) | Clearly separates supplied 0x009 RTR/response and payload facts from local 0x017 RTR/response trigger and both request-DLC choices; behavior unchanged |
| CDX-016 | Policy addendum disposition | Protected title/body mismatch retained; prior seven compensating response cases mapped; no duplicate added solely for title |
| CDX-017 | `codex/cases/driver/test_closeout_hall.cpp` | All six signed nondefault public fields; independent literal paired words; constant-first-pair mutant rejected |

All test/code paths above are under `Firmware/Tests/`. The finite helper and PID
work use local state and pure math, so no hardware/global-state fixture is needed.
Hall uses `TmcFixture`; USB uses `UsbFixture`. No protected firmware, vendor,
Claude, baseline or charter file was edited. No dependency was added/downloaded.

The finite helper checks operands and tolerance before subtraction, then checks
computed absolute error before its bound. It cannot silently discard NaN through
an unordered comparison or a maximum reduction. PID reference is
`Kp*e + Ki*sum(e) + Kd*(e-previous_e)` from zero history; neither returned PID
state nor derived production A coefficients are read. Dyadic gains/inputs bound
rounding; the absolute allowance is 1e-6. Direct finite and NaN rejection controls
also run per sample. The existing protected reducers remain unchanged; this is
bounded compensation for selected public workloads, not exhaustive repair.

Hall input is `{-123,2345,-3000,4095,12345,-23456}`. Supplied Rev2.07 §7.2 p89
low/high signed 16-bit fields yield `0x0929FF85`, `0x0FFFF448`, `0xA4603039` at
0x34–0x36. This uses the prior reviewed supplied-spec derivation recorded in the
requirements/P10 report, not a fresh PDF inspection or production-generated
expected value. No actual Hall packing bug or current production caller impact
is alleged.

## Exact shared changes for coordinator inventory

1. `Firmware/Tests/support/UsbFixture.h`, `UsbFixture::enumerate()`: after the
   existing successful `host.enumerate()` and `tud_mounted()` requirements, add an
   explanatory two-line comment, first-fault INFO and
   `REQUIRE(firstFault(host).empty())`. Consumers are every USB test calling the
   shared successful-enumeration helper, including protected
   `UsbdControlFixture::startVendorDevice`, which subsequently calls
   `HostDcd::takeFaults()`. This is a narrow fault boundary; no expected-fault API
   changes, no guard on failed-enumeration paths, no reset/clearing behavior
   changes. Full USB regression confirms existing negative-request cases pass.
2. `Firmware/Tests/fakes/FakeOdrivePeer.h`: comment-only model, source,
   strictness and confidence attribution corrections, including violation enum
   comments. Source-supported 0x009 RTR and response layouts remain distinct
   from adopted O14/L11 request DLC 0 for both commands and 0x017 RTR/response
   trigger. Consumers are driver tests using the peer and readers of its
   fidelity contract. No implementation, frame, enforcement, response or ABI
   behavior changes. Full driver regression passes.

Coordinator owns CMake registration and the combined shared-file summary. All
four relevant Codex globs were already registered when this build configured.

## Executed validation and durable artifacts

Build root:
`../ffb-test-build/codex-closeout-numeric-usb-driver/` (outside the repository).
No baseline build/object directory was mutated. Commands:

```sh
cmake -S Firmware/Tests -B ../ffb-test-build/codex-closeout-numeric-usb-driver -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build ../ffb-test-build/codex-closeout-numeric-usb-driver --target ffb_unit_tests ffb_target_tests ffb_usb_tests ffb_driver_tests -j 6
python3 -B ../ffb-test-build/codex-closeout-numeric-usb-driver/run_numeric_closeout.py
python3 -B ../ffb-test-build/codex-closeout-numeric-usb-driver/fix_usb_guard_probe.py
```

The two scripts are retained under the build root with their original names.
The experiment scripts originally ran from `/tmp`; the commands above are the
recommended retained-script reproduction commands. Archival-only `/tmp`
self-copy/log-copy lines have since been removed; the existing `build.log` and
all recorded results/history were preserved without rerunning experiments.
Both retained scripts were syntax-checked with Python's `compile`, which writes
no bytecode. The USB source transformation now matches `enumerate();` independent
of indentation, accommodating the coordinator's later tabs-only formatting.

Reproduction prerequisites are this repository at the reviewed sources, the
configured and built four executables plus their `compile_commands.json`,
object files and CMake `link.txt` files in the stated build root, and the retained
P10 `../ffb-test-build/codex-p10-review/hall_defaults.o` (its transformation
manifest is in that review's evidence directory). Run the main script before
the guard-off correction script. Paths in the scripts/argv manifest are absolute
for this workstation; relocation requires updating the repository/build root.
The scripts recreate scratch sources/binaries and overwrite named run logs and
the command manifest, so archive the evidence directory before intentionally
rerunning. No `/tmp` content is needed by the retained scripts.

`commands.json` records each test/compile/link argument array, cwd and exit code;
matching `.stdout`/`.stderr` files retain XML/diagnostics. `build.log` retains the
successful build; initial build failed for missing `<initializer_list>` in the
new pure helper test, corrected before successful rebuild. Initial sandbox
configure could not write the sibling directory; elevated configure/build and
scratch writes were automatically approved. No approval rejection occurred.
Successful configure output was observed in tool output; no separate durable
successful configure log was captured. Compiler is GCC 16.2.1, DSP enabled,
portable C CMSIS path. No coverage collection or hardware execution is claimed.

| Selection | Executed result |
|---|---|
| `ffb_unit_tests -ts=CodexNumeric -r=xml` | 3 ordinary cases / 15 assertions; 0 failures |
| `ffb_target_tests -ts=CodexPidCloseout -r=xml` | 1 ordinary case / 29 assertions; 0 failures |
| `ffb_usb_tests -r=xml` | 287 cases = 265 ordinary + 22 expected-failure; 0 unexpected failures / 0 skips |
| `ffb_driver_tests -r=xml` | 930 cases = 861 ordinary + 69 expected-failure; 0 unexpected failures / 9 skips |
| `python3 -B Firmware/Tests/tools/check_baseline.py --build-dir ../ffb-test-build/codex-closeout-numeric-usb-driver` | OK; 318 present, 317 checked, 1 excluded; unit 652 passed / 0 failed / 5 skipped |

The baseline command uses the newly configured default quarantine, including
nonbaseline `typed access reads single bytes and counts elements`; it does not
invoke that undefined-return body. Original baseline exclusion remains
`requesting the forces of a not existing axis is rejected`. Default crash skips
remain active. No `--no-skip`, unsafe whole-suite override or UB forcing ran.
Filtered-out cases in XML from suite selections are not executed skips.

## Discriminating experiments

All transformations are scratch-only; scripts and transformed sources remain
in the build root. Exact command arrays and exits are in `commands.json`.

- **PID actual-output corruption:** copied only the new test TU. Execute real
  `arm_pid_f32` and preserve its state, then substitute sample index 3's observed
  output with quiet NaN or finite `123.0f`. Both binaries select exactly one case
  and fail exactly one assertion: the main
  `CodexNumeric::finiteWithin(actual, reference, 1e-6)` check. Neither failure is
  merely a synthetic negative-control check. Files: `pid-{nan,finite}.cpp`,
  `pid-{nan,finite}-run.stdout`.
- **USB setup-before-discard:** copied new setup fixture calls public
  `dcd_remote_wakeup(0)` on the nonsuspended bus after `startDevice()` and before
  `enumerate()`. Fake DCD records its protocol fault. The copied fixture would
  consume that specific fault and emit `DISCARD_REACHED` after enumeration.
  Guarded run first fails shared `UsbFixture.h:230`
  `firstFault(host).empty()`; destructor separately observes the retained fault.
  The discard marker is absent. `usb-setup-fault-run.stdout` records it.
- **USB guard-off control:** scratch-only header removes the new requirement;
  it uses a distinct class name so existing inline definitions cannot override
  it at link time. Same public fault plus successful enumeration reaches
  `takeFault`, consumes the expected fault, and fails only the explicit
  `DISCARD_REACHED` marker. Thus an enumeration failure did not create the
  guarded result. `usb-no-guard-run.stdout` and `fix_usb_guard_probe.py` retain
  this result. An earlier same-class scratch attempt selected an existing inline
  guarded body and is invalid for guard-off evidence; it is preserved as
  `usb-no-guard-invalid-odr-run.*` and superseded by the distinct-class run.
- **Hall configuration discrimination:** relink own new case against the prior
  retained P10 `hall_defaults.o`, which replaces only the first paired word
  with default constant `0x2AAA0000`. Exactly one new case selected; fails only
  register 0x34 comparison against `0x0929FF85`. Other pairs and the platform
  guard pass (5 successful assertions, 1 failure). `hall-defaults-run.stdout`
  and retained P10 transformation manifest describe provenance; no firmware
  source was edited here.

## Retained boundaries

This assignment locally validates DSP-on default builds and the affected full
USB/driver groups. Coordinator runs the combined default, DSP-off and sanitizer
gates; no independent sanitizer/DSP-off result is claimed here. The historical
P9 finite trig/biquad guards remain accepted but were not newly modified.
Protected reducers remain defective for nonfinite outputs outside these bounded
compensating workloads. Open production/vendor defects and skips remain open.

The host portable-C PID result does not establish Cortex-M FPU/intrinsic
equivalence, timing, preemption, DMA lifetime, arbitrary interrupt/task ordering,
physical USB/CAN/SPI behavior, sensor references, ODrive version/calibration
behavior or Windows/macOS execution. No fresh coverage percentage is reported.
ODrive unsupported policy fields remain explicitly unverified vendor fidelity;
comments do not turn them into hardware facts. Fresh independent Astra QC is
required before coordinator final acceptance.
