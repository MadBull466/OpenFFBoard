# P7 independent CAN and serial FFB review

2026-11-05; coordinator source/assertion review. All five phase test files were read: `test_can_setup.cpp` (17 cases), `test_canport2b.cpp` (79), `test_canbridge.cpp` (63), `test_caninputmain.cpp` (45) and `test_serial_ffb.cpp` (93), under `Firmware/Tests/Code_Coverage/claude/cases_target/`. Total 297 catalogued cases, with four existing skip decorators. Deliberate UB/crash cases were read, not forced. The actual CANPort2B, CanInputMain, CanBridge and SerialFFB implementations were traced against public stimuli and asserted observations. No new confirmed test defect emerged from this bounded review.

The review checked asymmetric frame bytes and channel offsets, standard/extended/RTR transitions, mailbox occupancy and acknowledgment, timeout boundaries, wrong-recipient callbacks, filter bank bookkeeping, source selection, changed-value suppression, report rates, command addressing and effect lifecycle. CAN input construction deliberately supplies zeroed backing storage around the documented uninitialized-digital-buffer issue; this is a fixture limitation, not proof that production initialization is correct. Serial force tests exercise the calculator and compare HID and serial paths with positive-force and zero-force controls, including waveform progression. Shared path comparison does not independently establish USB PID conformance.

Five isolated single-fault experiments compiled copied firmware sources and relinked against immutable default target objects. All full target runs exited 1 at the intended ordinary observations:

| Fault | Relevant detection | Unexpected failed cases |
|---|---|---:|
| Toggle bit 0 of transmitted standard ID | Exact standard frame ID, plus downstream bridge/input observations | 22 |
| Abort full mailboxes at `>=20` rather than `>20` ms | `pending frames are not aborted within 20 ms of the last acknowledged frame` | 1 |
| Pack analog high byte from shift 7 rather than 8 | `analog values are sent as 16 bit little endian values with the analog id` | 1 |
| Normalize serial axis gain by 65534 rather than 65535 | `axisgain sets the axis magnitude of the instance from 16 bit` | 1 |
| Scale bridge sync milliseconds by 1001 rather than 1000 | `the time sync reply carries the microseconds since start up` | 1 |

Each full run retained the other target phases and normal skip policy. Controls are the recorded green default baseline and two random-order target runs, rather than separate freshly relinked unmodified controls. This is selective effectiveness evidence, not a mutation score or exhaustive repair-discrimination exercise. The ordinary unit ring-buffer UB finding is in another executable and is not exercised here.

Evidence: `/home/ben/Documents/OpenFFBoard/testing/ffb-test-build/codex-p7-review/mutation_results.json`, copied sources and per-mutant compile/link/XML/stderr files in that directory. A reproduction script reconstructed from the surviving JSON is archived beside the results as reproduce.py; it was not rerun after reconstruction. No firmware, original test, shared fake or baseline object was edited.

Limits: GVRET exact vectors were checked against local packing and stated comments; an independent supplied GVRET specification was not available/read. Bus arbitration, electrical timing and real mailbox interrupt concurrency are not established by the cooperative host model. Automatic acknowledgment in the fixture is relevant to bursts exceeding three mailboxes. Characterizations of accepted out-of-range IDs, source initialization and unfinished commands remain characterizations. The review did not force period-zero/invalid-index/crash paths, establish all historical firmware diagnoses independently, or prove that every known-issue marker flips only after its intended repair. No duplicative permanent case was added for the five already-detected faults.
