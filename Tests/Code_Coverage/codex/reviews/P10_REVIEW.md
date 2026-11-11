# P10 independent driver review

2026-11-06 · GPT-6.1 Sol · high reasoning effort. Assignment: fresh review of D0–D3, the three drivers and all **24 driver test files / 12,062 source lines / 938 registered cases**. P11 excluded. This assignment changed only this report; all experiments used scratch source copies and substituted objects outside the repository. No firmware, vendor code, Claude files, shared infrastructure, dependencies or Git configuration changed.

## CDX-015 — O14 promotes framing assumptions to ordinary specification checks

**Medium oracle/classification issue.** `Firmware/Tests/Code_Coverage/claude/cases/driver/test_odrive_frames.cpp:470–511` asserts RTR and DLC 0 for both encoder-estimates and bus-voltage requests as ordinary O14 tests. `Firmware/Tests/Code_Coverage/fakes/FakeOdrivePeer.cpp:396–425` rejects other choices, and the fixture fails on unexpected peer violations. This also affects ordinary O42 request assertions and the fake's own strictness cases (`test_odrive_fake.cpp:130–151`).

Independent check of the supplied ODrive 0.5.6 PDF (rendered complete relevant pages 3–4) and CSV:

- 0x009 has the call/response asterisk. The associated note supports RTR and a response with the same ID and specified payload.
- 0x017 lacks that asterisk; its sender is `Master***`, whose note permits addressing either axis on a board. That footnote does not specify RTR.
- Neither source supplies the request DLC for these messages. Response signal offsets/types are not a request-DLC definition.

Charter O14 explicitly adopts RTR/DLC 0, and L11 expressly calls the DLC an assumption. Thus these tests **do enforce a recorded local charter policy**. Their failure is not, by itself, proof of vendor-protocol nonconformance for the unspecified fields. L11's statement that no test depends on the other case understates the ordinary exact-DLC assertions and strict peer checks. A future owner-approved framing change could fail them without contradicting the supplied vendor references.

Recommendation: retain source-supported IDs, 0x009 RTR, and response layouts as normative checks; label the unsupported request fields as characterization or explicit project policy, including the strict peer rule and confidence description. An additional owner-supplied reference could make those fields normative later. This review does **not** establish that the current RTR/DLC 0 frames are wrong, or that DLC 8/data requests would work on a real ODrive.

## CDX-016 — the O14 reply-title assertion checks only requests

**Low local assertion-scope issue; compensating suite coverage exists.** The case at `test_odrive_frames.cpp:493–503`, “the peer accepts both requests and answers them with the same id,” checks only empty violations and two request counters. `FakeOdrivePeer::rtrCount` counts logged requests independently of emitting responses. The case does not inspect a reply, its ID, payload or delivery.

Bounded fault injection: a scratch copy of `FakeOdrivePeer.cpp` changes only both request-handler `if (respond_)` conditions to `if (false && respond_)`. All request logging/validation, manual sends and cyclic heartbeats remain as before. The named O14 case still passes **4/4 assertions**. The full suite catches the fault in **seven other cases**: five fake reply/capture/delay cases, O42's returned position, and O38's reply-dependent voltage-request cadence. Full result is 922 doctest passes / 7 failures / 9 skips, compared with control 929 / 0 / 9.

In particular, `test_odrive_fake.cpp:191–219` independently captures response IDs and asymmetric payload bytes, `test_odrive_rx_control.cpp:589–598` observes the driver's returned position, and `test_odrive_states.cpp:411–420` depends on voltage replies updating their timer. This is **not a missing whole-suite response check**. Rename the local case to its actual request-acceptance scope or assert the responses it names.

## CDX-017 — Hall angle packing is tested only at default values

**Low bounded test gap.** `test_tmc4671_config.cpp:881–892` tests the six Hall positions using the default `TMC4671HALLConf`. Other Hall setup cases vary flags, blanking, offsets and dPhiMax but leave those six angles at defaults. The public `setup_HALL` accepts six configurable signed angles; supplied Rev 2.07 §7.2 p89 specifies their paired low/high signed 16-bit fields.

Scratch mutation of **only** `TMC4671.cpp:1359` replaces the computed first pair with `uint32_t posA = 0x2AAA0000u;`, ignoring the supplied `pos0` and `pos60`. The existing full suite still gives **929 / 0 / 9**, with the same expected-failure outcomes as control. This demonstrates failure to discriminate configurable first-pair packing from a default constant, despite executing the line.

A scratch-only public-interface probe supplies `{-123, 2345, -3000, 4095, 12345, -23456}` for the six angles. Independent two's-complement packing gives registers `0x34 = 0x0929FF85`, `0x35 = 0x0FFFF448`, `0x36 = 0xA4603039`. Control passes all three angle checks; the mutant fails the first (`0x2AAA0000` observed) while the other pairs remain correct. Including the fixture check: control 4/4 assertions pass, mutant 3 pass / 1 fail.

A nondefault signed-angle case would close this gap. This is not an existing firmware defect. No impact on a current production caller using customized Hall angles was established; reviewed production calls pass the stored/default Hall configuration, and the review did not establish a path that changes those angle members. The finding is limited to the public configuration contract and the scope of the R52 assertion.

## Assessed assertion bodies and behavior

Every case body and local helper in these files was read, including generated PI cases, skipped bodies, `KNOWN_ISSUE` bodies and firmware-only characterization. Counts below are registered cases, including default skips; they are not line-coverage claims.

| File under `Firmware/Tests/Code_Coverage/claude/cases/driver/` | Cases | Assessed behavior |
|---|---:|---|
| `test_driver_setup.cpp` | 1 | D0 feature macros and excluded features |
| `test_tmc4671_smoke.cpp` | 4 | presence, state helpers, missing supply |
| `test_tmc4671_fake.cpp` | 23 | independent raw SPI, stacked selectors, defaults, read-only writes, CS routing, hooks/reset |
| `test_tmc4671_spi.cpp` | 21 | exact MOSI/MISO bytes, asymmetric values, own CS, prescalers, read-modify-write preservation, async completion/release |
| `test_tmc4671_identification.cpp` | 18 | chip/version words, absent/wrong chip, delayed appearance, retry cadence and communication error |
| `test_tmc4671_init.cpp` | 41 | PWM/motor/ADC fields, PI precision, independent bilinear/Q29 filter oracle, stale-target clearing and enable order |
| `test_tmc4671_config.cpp` | 76 | signed/asymmetric PI/config fields, masks, selectors, PWM frequency, lowpass/notch/peak response, ABN/AENC/Hall setup |
| `test_tmc4671_control.cpp` | 53 | signed torque/flux halves, inhibit preconditions, inversion, field weakening/brake policy, readouts, ramp endpoints, voltage/temperature arithmetic |
| `test_tmc4671_states.cpp` | 47 | power/no-power, exact six-check window, shutdown/restart, hardware/software E-stop, supply/temperature/ADC errors and recovery |
| `test_tmc4671_encoder.cpp` | 34 | following/nonmoving/reversed plants, alignment success/failure, broadcasts, calibration/index/autotune, external timer/async angle |
| `test_tmc4671_commands.cpp` | 89 | 42 registrations, public command results and register effects, invalid/narrowing values, unsupported/crash paths |
| `test_tmc4671_flash.cpp` | 29 | independent key/word expectations, complementary settings, both instances, cold-chip reapplication, precision/CPR loss, calibration persistence |
| `test_odrive_smoke.cpp` | 6 | construction, scheduling, startup and FIFO1 model deviation |
| `test_odrive_fake.cpp` | 35 | strict wire decoding, exact captured reply bytes/IDs, cyclic timing, delayed/silent/refused/wrong-node/short/extended traffic |
| `test_odrive_frames.cpp` | 59 | filters/port lifetime, standard IDs including node63, raw signed/asymmetric modes, torque bytes, requests, invalid node policy |
| `test_odrive_rx_control.cpp` | 54 | independently built LE payloads, node isolation, malformed/RTR/extended frames, torque scaling, request rate, offsets/count conversion |
| `test_odrive_states.cpp` | 42 | no heartbeat, calibration states, delayed/already-closed-loop modes, stop/restart, reconnect, timeout/bus-off/mailbox behavior |
| `test_odrive_commands_flash.cpp` | 57 | get/set/error results, register-independent packed words, both instances, missing keys, offset continuity and narrowing |
| `test_vesc_smoke.cpp` | 6 | initial request bytes/times, silence/retry, peer-based readiness and second instance |
| `test_vesc_fake.cpp` | 48 | CRC known vectors, independent frame capture/fragment sequence, reply layouts, strictness, broadcast, faults and SET-only timeout |
| `test_vesc_frames.cpp` | 46 | extended IDs/masks, filter lifetime/leak, exact current scaling/bytes, requester/destination IDs, request DLC and UB marker scope |
| `test_vesc_protocol.cpp` | 49 | fixed payload/CRC vectors, byte order, stale/partial/reversed/long/short buffer paths, version and values decoding/recovery |
| `test_vesc_states.cpp` | 33 | readiness/timeout/reconnect/PONG, nonready/stopped inhibit, nonzero torque repeats, SET gaps, zero/stop/restart |
| `test_vesc_encoder_commands.cpp` | 67 | signed/wrapping positions, public commands, high-bit IDs, encrate periods, flash keys/normalization and instance isolation |

Sources/fixtures assessed: all three `Fake*.cpp` peer implementations and their relevant API/fidelity headers; all three driver fixture headers; `AxisFixture.h` reset/guard behavior; driver CMake section and F407 driver feature header. `ODriveCAN.cpp` and `VescCAN.cpp` were read throughout; `TMC4671.cpp` was read in selected construction/flash/init/state, encoder/calibration, Hall/config, control/PID/biquad, SPI and command-related sections. `TMC4671_configs.cpp` was read throughout. This is not a claim of a fresh line-by-line audit of all 3,190 TMC source lines or all linked target/platform modules.

All three driver requirement charters, the driver plan, relevant main-plan/progress sections and `LEAD_REVIEW.md` were consulted. Source oracles were independently checked at the supplied ODrive CSV/PDF framing entries, TMC datasheet read timing and Hall fields, and supplied VESC 7.01 `comm_can_send_buffer`, FW-version layout, CRC and timeout comparison. Other register/command expectations were checked against charter facts and selected production bodies, not every vendor register page. Historical coverage/mutation percentages were not accepted as proof of adequacy or remeasured here.

## Verification and boundaries

Existing immutable binaries, with default skips retained and no extra exclusions:

```
../ffb-test-build/codex-baseline-default/ffb_driver_tests -r=xml
../ffb-test-build/codex-baseline-san/ffb_driver_tests -r=xml
../ffb-test-build/codex-baseline-default/ffb_driver_tests -r=xml --order-by=rand --rand-seed=20261106
```

Default **929 / 0 / 9** = 860 ordinary passes + 69 expected-failure cases. Sanitizer **919 / 0 / 19** = 855 ordinary passes + 64 expected-failure cases. Random order reproduces **929 / 0 / 9**. Default XML has 10,682 assertions, including 76 expected failed assertions; sanitizer has 10,618 assertions, including 71 expected failed assertions. Initial sandbox sanitizer execution hit the known LSan/ptrace restriction; rerunning elevated completed without a sanitizer diagnostic or suppression. No `--no-skip`/forced-crash execution was used. Default known-issue first failures were extracted and compared to the case bodies: all 69 failed at their stated defect observations in this run; no wrong-first-failure finding was confirmed. This does not guarantee that a future unrelated failure cannot be masked by a whole-case expected-failure marker.

Scratch artifacts are under `../ffb-test-build/codex-p10-review/`: `FakeOdrivePeer_no_response.cpp`, `no_response.o`, `no_response_{o14b,full}.xml`, `TMC4671_hall_defaults.cpp`, `hall_defaults.o`, `hall_probe.cpp`, `hall_probe.o`, `{control,hall_defaults}_{full,probe}` binaries and matching XML. Baseline XML and local reference-reading evidence have been durably copied to `../ffb-test-build/codex-p10-review/evidence/`: `p10_{default,san,random}.xml`, `p10_odrive*` and `p10_tmc*` (10 existing files, with checksums in the manifest). The evidence directory's `experiment_argv_manifest.json` records compile/link/run argument arrays, working directories, exact source transformations, output paths and source hashes for the no-response/Hall trials. `EXECUTION_NOTES.md` records reference-extraction/render commands and the initial LSan diagnostic transcribed from tool output. Separate baseline stderr/compiler logs were not captured originally and cannot be archived; the notes distinguish that limitation from the available XML. Specs, extracts and renders remain outside the repository and must not be committed. This archive operation reran no experiments. Experiments reuse the recorded compile command flags and existing objects, replacing only the named scratch object; the Hall probe adds one scratch test object. `/usr/bin/c++` reports GCC16.2.1. No percentage mutation score, full clean rebuild or new baseline-318 run is claimed for this report-only assignment.

R5 is an explicitly documented proxy, not a nanosecond timing measurement. `test_tmc4671_spi.cpp:210–222` observes <=2MHz SCK or an initial one-byte transfer; Rev2.07 §4.2.1 p19 requires an actual >=500ns pause for faster reads. A split alone does not prove that duration, and the peer only accepts complete five-byte datagrams. No new firmware defect was established from the proxy. A timing repair involving split transactions would also require adapting the fake/transport tests with owner approval.

Other retained limits: no physical bus timing/arbitration, interrupt preemption or motor/control-loop proof; scripted TMC plants have low fidelity; default fixtures raise all CS lines before construction; ODrive FIFO1 is enabled by the fixture and the target deviation has separate cases; CAN scheduling rewinds the fake Delay jump and excludes unrelated timed objects across `runFor`. ODrive enum/calibration policy/version differences and VESC5.x layout remain unresolved supplied-reference boundaries. Sanitizer skips hide the known null-zero-length POLL call paths; memory-corruption/crash cases remain skipped. The skipped AENC constant-channel test additionally asserts a particular non-HardError outcome after its no-crash claim; no safe repair discrimination for that policy was executed here. The accepted outcomes for such a future repair need review before treating that skipped body as a complete oracle. Existing green runs and historical line coverage do not settle these limits.

Status: **review complete; CDX-015/016/017 await coordinator/Astra disposition.** No permanent test creation or shared-file correction was assigned.
