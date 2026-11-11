# Repository discovery (2026-11-05)

## Starting point and authority

- Checkout: `/home/ben/Documents/OpenFFBoard/testing/OpenFFBoard`, branch `add-test-suite`.
- Revision: `8a8041dcd6fbc8b706383f71cbda3f638961725b`.
- Initial tracked/untracked working tree: clean (`git status --short`). An ignored `Firmware/Tests/Code_Coverage/build/` directory predates this investigation (directory timestamp 2026-11-02); it was neither used nor removed.
- Latest commit splits Claude/Codex ownership and moves tests/documents; its subject explicitly says "gate not re-run".
- The initial instruction authorised discovery/baseline only; subsequent owner authorisation started P0–P10 review and approved CDX-001. Current scope/status is in reviews/README.md. No commits or pushes performed.
- Root `AGENTS.md` and `CLAUDE.md` read. No nested `AGENTS.md` found under `Firmware/Tests/`.

## Architecture established from build files and support headers

Real, unmodified C/C++ sources compile against shadow platform headers and link-time HAL/RTOS/USB doubles. Firmware is compiled separately for each executable because feature macros and include order differ. The default build is Debug, GNU C++20/C11, using Unix Makefiles. doctest 2.4.11 is vendored; Python uses stdlib unittest.

| Executable | Test files | Firmware/platform boundary |
|---|---|---|
| `ffb_unit_tests` | 9 original `cases/test_*.cpp` + 15 `claude/cases/test_*.cpp` | Generic feature header, fake Axis and map-backed flash, recording USB |
| `ffb_target_tests` | 42 `claude/cases_target/test_*.cpp` | F407 feature set, real Axis/main classes/inputs/encoders/buses, simulated HAL and recording USB |
| `ffb_flash_tests` | 5 `claude/cases_flash/test_*.cpp` | Real EEPROM emulation and flash helpers on fixed-address simulated NOR flash; omitted on macOS |
| `ffb_rtos_tests` | 11 `claude/cases_rtos/test_*.cpp` | Real FreeRTOS C++ wrappers, extended cooperative RTOS fake; no DSP |
| `ffb_usb_tests` | 7 `claude/cases_usb/test_*.cpp` | Real TinyUSB device stack and firmware glue, fake controller and simulated USB host, extended RTOS fake |
| `ffb_driver_tests` | 24 `claude/cases/driver/test_*.cpp` | Real TMC4671/ODrive/VESC drivers, strict simulated SPI/CAN peers, F407 driver feature header |
| `ffb_dsp_tests` | 9 `claude/cases_dsp/test_*.cpp` | CMSIS-DSP portable C implementation and ARM reference patterns; no firmware/platform sources; conditional f16 support |
| Python helpers | 6 `claude/python/test_*.py` | Firmware Python tools and test-report helper self-tests; temporary outputs and dependency stand-ins |

Total: 122 C++ test source files (9 original + 113 Claude), 6 Python test files. Counts describe files, not independently reviewed assertions. No Codex tests exist; `Firmware/Tests/Code_Coverage/codex/` currently contains only its README. CMake has no Codex test glob or executable integration.

Time is simulated through HostPlatform/HostRtos. Cooperative/resumable execution has no target preemption. FirmwareFixture resets generic globals; TargetFixture adds F407 resets and HAL-fault checks; FlashFixture adds flash state/power cuts; RtosFixture adds kernel-fault/object-leak checks; UsbFixture adds controller/host/stack resets; driver fixtures add peer checks and scheduling. ODrive/VESC scheduling deliberately rewinds simulated time after driver activations (documented fixture limitation). These mechanisms were mapped, not audited for correctness.

`KNOWN_ISSUE` uses doctest `should_fail`; expected assertion failures count as passed. UB markers and main-class destruction markers skip selected tests under sanitizers. The separate full-report tool forces skipped cases in isolated processes and distinguishes Passed/Failed/Bugged; it is not the CTest gate and its target uses `--exit-zero`. It still contains the obsolete Python directory (see CDX-001).

## Scope and available specifications

Current documented scope includes firmware logic, EEPROM emulation, C++ RTOS wrappers, Python helpers, TinyUSB device stack, TMC4671/ODrive/VESC, and P11's expanded CMSIS-DSP modules/all types. P0-P10 are historically approved; P11 G1 FastMath/helper tests exist but fresh review/mutation/approval remain pending. G2-G5 remain future work. Do not interpret older driver exclusions or tier-C wording as current status.

Out of scope remains ST HAL, CMSIS core, FreeRTOS kernel, generated target code, PWM/MyActuator/SimpleMotion drivers, TMCDebugBridge and firmware MIDI. Source lists and feature headers implement distinct test configurations; TinyUSB's MIDI class still compiles because the F407 USB configuration enables it.

Verified present (not independently validated as protocol oracles during discovery): TMC4671 Rev 2.07 PDF, ODrive 0.5.6 protocol PDF and CSV, VESC CAN slides and `bldc-master` source under ignored `Tests/Code_Coverage/shared/specs/`. CMSIS-DSP submodule is present at `1ba19d14a6491169b4d2873e1a0a59486c7636c6`, with ARM patterns. No HID PID 1.01 specification appears at that directory's top level. No files downloaded or dependencies installed.

Read-only workflow inspection found no reference to `Firmware/Tests`, `ctest` or `cmake` in `.github/workflows`/`.github/actions`; existing workflows build firmware/release artifacts. No test CI execution verified. Presets exist for host, no-DSP, sanitizer and MinGW; their default paths are in-tree, so this investigation uses explicit out-of-tree `-B` paths instead.

## Documentation inventory

There are 18 Markdown files under `Tests/Code_Coverage/claude/`. Read status below deliberately separates discovery reading from a substantive correctness review.

| Document under `Tests/Code_Coverage/claude/` | Discovery status |
|---|---|
| `README.md` | Read |
| `plans/HANDOFF.md` | Read |
| `plans/TEST_PLAN.md` | Read |
| `plans/DRIVER_TEST_PLAN.md` | Read; historical Claude-specific model/worker assignments are not a new Codex work order |
| `plans/COVERAGE_AND_SCOPE.md` | Read |
| `plans/PROGRESS.md` | Targeted reading: current state, next action, decisions, environment, original baseline/P0, firmware map, P9 results, verification and open questions; other phase/driver sections indexed, detailed matrices/findings deferred |
| `reports/PROJECT_SUMMARY.md` | Read; explicitly historical P9 snapshot |
| `reports/DRIVER_TRACK_REPORT.md` | Read |
| `reports/D1_TMC4671_REPORT.md` | Read |
| `reports/D2_ODRIVE_REPORT.md` | Read |
| `reports/OPEN_QUESTIONS.md` | Read |
| `reports/BUG_REPORT.md` | Opening/counts/highlights/module index read; individual defect claims deferred |
| `charters/dsp/CMSIS_DSP_REQUIREMENTS.md` | Read |
| `charters/dsp/G1_WORK_ORDERS.md` | Read |
| `charters/dsp/CMSIS_DSP_INVENTORY.md` | Sections 1-2 read; remaining tolerances/pattern inventory deferred |
| `charters/driver/ODRIVE_REQUIREMENTS.md` | Opening, sources and construction/TX/RX/state requirements read; remaining requirements deferred |
| `charters/driver/TMC4671_REQUIREMENTS.md` | Located; detailed reading deferred to driver audit |
| `charters/driver/VESC_REQUIREMENTS.md` | Located; detailed reading deferred to driver audit |

12 read in full for discovery, 4 partially read, 2 located only. Additionally read root rules, suite/ownership READMEs, presets, build source/glob/feature/sanitizer/integration blocks, helper path definitions and fixture/marker contracts. At discovery time no C++ test file had received an assertion-correctness review. Subsequent review status supersedes that historical boundary in the ledger below.

## Questions for the next stage

1. Resolved by owner approval: CDX-001's shared path corrections are applied and verified; see REGRESSION_RESULTS.
2. Choose the first substantive audit: safety-related torque/E-stop/integration tests, or unfinished P11 G1 DSP tests, or an owner-specified component. Historical review/approval claims are inputs, not independently verified conclusions.
3. For later Codex test integration, authorise a targeted shared CMake change when a concrete test group is proposed, or retain case-by-case approval. No executable/feature-set change proposed during discovery.

Existing hardware/protocol policy questions remain recorded in Claude's OPEN_QUESTIONS. They need answers only when the chosen audit depends on them; no behaviour, register layout, tolerance contract or version compatibility is invented here.

## Subsequent review reading ledger (2026-11-06)

The table above preserves discovery-time reading status. Later source/assertion review and exact assessed/deferred bodies are recorded by phase under reviews/. The technical lead read all three driver charters fully (LEAD_REVIEW.md); the fresh P10 reviewer is independently checking them and supplied references. P1 and P6 deferred test bodies are being completed, with P9/P10 active. PROGRESS/BUG_REPORT phase-specific claims have been consulted by the assigned reviewers; this does not imply every historical diagnosis in those large documents has been independently verified. P11 expanded DSP documents remain outside the authorised audit. New Codex tests now exist and their integration proposals are staged.
