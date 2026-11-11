# Audit of the Codex review (2026-11-09)

Scope: Codex's findings CDX-001 to CDX-017 and P12-TICK-001 (`Tests/Code_Coverage/codex/`), its edits to shared files in `Firmware/Tests/`, and the fixes applied by Claude. Firmware sources were not changed. The audit and the integrated Codex work were committed on 2026-11-09 (`211228a1`, `63a3db81`) and have been pushed to the fork since.

## Status 2026-11-10 (kept current)

- The audit's results table is a 2026-11-09 snapshot. Since then P11 G4-G5, P13-P16 and P17 changed the executables: `ffb_dsp_tests` 218 -> 2485 cases, `ffb_driver_tests` 930 -> 1717, `ffb_usb_tests` 287 -> 606 (P17, MIDI), `ffb_target_tests` 1832 -> 1863 (P18, bug 225); `ffb_unit_tests`, `ffb_flash_tests` and `ffb_rtos_tests` are unchanged (654 / 159 / 278). Current counts and ctest results (11 of 11 default / sanitizer / clang / coverage / 32-bit, 10 of 10 DSP off) are in `Tests/Code_Coverage/claude/plans/PROGRESS.md` ("Current state"). The driver coverage figure in the gcov bullet below (2,376 / 2,596, 91.5 %) is now 3,262 / 3,484 (93.6 %) with P13-P16; the TinyUSB figure (1,337 / 1,572, 85.1 %) is now 1,508 / 1,572 (95.9 %) with P17; the firmware, tier A, TinyUSB and tier C figures are unchanged.
- Findings CDX-001..017 and P12-TICK-001 / P12-I2C-001: verdicts and actions below are final; nothing was reopened. The Codex tools and tests audited here are still part of the suite (`codex_python_tests`, `codex_usb_setup_guard`, opt-in `codex_st_hal`).
- Left open item 1 (bug 39) is open question 20 in `Tests/Code_Coverage/claude/reports/OPEN_QUESTIONS.md`; item 2 (real-hardware questions) is unchanged.

## Method
1. Read all Codex documents; listed findings, proposed fixes and gap claims.
2. `git diff` of the nine modified tracked files; every hunk read.
3. Verified each finding against the firmware, the supplied kernel and ODrive spec, and the tests. Where a claim could be checked by an experiment, I ran one in a scratch copy under `../ffb-test-build/audit/` (never in the repository).
4. Full build and `ctest` before and after, in several variants (below).

## Test results

Builds are out of tree in `../ffb-test-build/audit/`. Counts are doctest cases (passed / failed / skipped); "skipped" includes cases that are excluded or skipped on purpose.

| Run | ctest | unit | target | flash | rtos | usb | driver | dsp |
|---|---|---|---|---|---|---|---|---|
| **Before**: pristine HEAD `8a8041dc`, DSP on | **8 of 9: `python_helper_tests` fails (CDX-001)** | 646 / 0 / 4 | 1819 / 0 / 18 | 159 / 0 / 0 | 271 / 0 / 0 | 286 / 0 / 0 | 929 / 0 / 9 | 218 / 0 / 0 |
| Before, DSP off | 7 of 8 (same failure) | not recorded | not recorded | not recorded | not recorded | not recorded | not recorded | n/a |
| Codex state as received (default) | 11 of 11 | 652 / 0 / 5 | 1832 / 0 / 18 | 159 / 0 / 0 | 278 / 0 / 0 | 287 / 0 / 0 | 930 / 0 / 9 | 218 / 0 / 0 |
| **After**: default | 11 of 11 | 654 / 0 / 4 | 1832 / 0 / 18 | 159 / 0 / 0 | 278 / 0 / 0 | 287 / 0 / 0 | 930 / 0 / 9 | 218 / 0 / 0 |
| After: ASan + UBSan | 11 of 11 | 651 / 0 / 7 | 1716 / 0 / 134 | 159 / 0 / 0 | 273 / 0 / 5 | 287 / 0 / 0 | 920 / 0 / 19 | 191 / 0 / 27 |
| After: DSP off | 10 of 10 | 654 / 0 / 4 | 1776 / 0 / 17 | 159 / 0 / 0 | 278 / 0 / 0 | 287 / 0 / 0 | 930 / 0 / 9 | n/a |
| After: clang 23 | 11 of 11 | 654 / 0 / 4 | 1832 / 0 / 18 | 159 / 0 / 0 | 278 / 0 / 0 | 287 / 0 / 0 | 930 / 0 / 9 | 218 / 0 / 0 |
| After: `--coverage` (gcc) | 11 of 11 | 654 / 0 / 4 | 1832 / 0 / 18 | 159 / 0 / 0 | 278 / 0 / 0 | 287 / 0 / 0 | 930 / 0 / 9 | 218 / 0 / 0 |

- Net change from my fixes: `ffb_unit_tests` +2 executed cases (the ring-buffer case Codex had quarantined runs again, plus one new case). `baseline` check passes in every run. No compiler warning comes from a test file (the ~1,600 gcc warnings are in firmware headers, as before).
- `ctest` entries: 9 before (8 without DSP); Codex added `codex_python_tests` and `codex_usb_setup_guard`.
- Opt-in `FFB_TESTS_ST_HAL=ON` (Codex P12 real-HAL gate, needs clang multilib): `codex_st_hal` passes (1 of 1, 4.3 s). Default is OFF, so it is not in the normal suite.
- Fresh gcov run with the Codex tools: firmware 8,007 / 8,095 lines (98.9 %), tier A 770 / 785, TinyUSB 1,337 / 1,572 (85.1 %), tier C 198 / 198, drivers 2,376 / 2,596 (91.5 %), DSP FastMath 443 / 451. These match the earlier recorded numbers, so the tool changes do not change the results.

## Findings

Verdicts: **confirmed** (I reproduced or verified it in source), **partially valid**, **incorrect**, **not verified** (relied on Codex's evidence).

| Finding | Verdict | Evidence and action |
|---|---|---|
| CDX-001 Python test path still the pre-move folder | **Confirmed. My error** from the folder reorganisation. | Before: `ctest` ImportError, `python_helper_tests` fails. Codex's two-line path fix kept. |
| CDX-002 coverage denominator can drop files without runtime data | Valid by code reading (old code skipped `.gcno`-only files and treated them as 0 lines). | Codex's tool change kept. Fresh coverage numbers unchanged (see above). |
| CDX-003 gcov failure still gave a successful report | Valid by code reading (old `run_gcov` printed an error and continued). | Kept. |
| CDX-004 baseline checker ignored the expected-result metadata | Valid by code reading; the stricter check passes on all 318 identities. | Kept. |
| CDX-005 empty Python discovery not reported | Valid by code reading. | Kept. |
| CDX-006 failed or empty doctest inventory accepted | Valid by code reading. | Kept. |
| CDX-007 duplicate baseline/report identities collapse | Valid by code reading; no duplicate exists today (checker passes). | Kept. |
| CDX-008 ring-buffer test reads an indeterminate value | **Partially valid.** Copying an indeterminate `unsigned char` is defined in C++, so ASan and UBSan are clean; only MemorySanitizer's return-value check flags it. The firmware weakness is NEEDS REVIEW 24. | **Codex's CMake quarantine reverted** (it silently dropped a Claude test, including its valid `numElements` checks, from the normal run). Test fixed instead: `test_ringbuffer.cpp` split into a populated case and a new empty-buffer case that asserts only the failure flag. README text restored. |
| CDX-009 known-issue masking hides a running-effect lifecycle regression | **Confirmed** (second pass, 2026-11-09): I reran M7 (state cleared on type change) in a scratch copy. All original unit cases still pass; only Codex's `changing a running periodic effect type preserves force until an explicit stop` fails. | Two Codex regression cases kept; they pass. |
| CDX-010 critical-error tests do not check motor inhibition | **Confirmed.** `FFBHIDMain::errorCallback` calls `emergencyStop(true)`, and `true` means "reset" in `FFBHIDMain`/`Axis::emergencyStop` (`control.emergency = !reset`). | My mutation (change to `false`): both existing Claude cases and the new Codex motor case flip, as intended. Codex cases kept. |
| CDX-011 ADS threshold known issue can hide a repair behind setup REQUIREs | **Partially valid.** The REQUIREs before the defect check rely on the current non-blocking implementation. Real design weakness; the repair Codex used is hypothetical. | Codex's implementation-independent regression kept. The Claude case left unchanged. |
| CDX-012 Tasklet timeout case cannot tell a checked repair from an unsafe one | **Confirmed.** `Schedule` ignores a failed `DtorLock` take; the Claude case discarded the result. | **Claude case edited** (`test_ctasklet.cpp`, 2026-11-09): it now also asserts that the second `Schedule` returns false and that only the first parameter is delivered. Verified in scratch copies: current code fails first at `CHECK(secondReturned)` (the defect); an unsafe repair (unchecked take) no longer flips the marker; a proper repair (checked take, return false) flips it, and so does Codex's case. |
| CDX-013 EventGroup Sync fake accepts reserved set bits | **Confirmed** against the supplied kernel: `xEventGroupSync` calls `xEventGroupSetBits` (`event_groups.c:210`), which asserts no control bits (`:531`). | Codex's fake fix kept. My mutation (revert the guard) makes the new Codex case fail. |
| CDX-014 DSP error maxima ignore NaN | **Confirmed.** `std::max(worst, fabs(NaN))` returns `worst`. Present in `maxAbsDiff`, `worstTrigRatio` and the PID loop of `test_cmsis_dsp.cpp`. | My mutation (PID returns NaN): the old test passes, so the masking is real. **Fixed in the three helpers** (a NaN result now fails); with the fix the mutant is caught. Codex's finite-output cases kept. |
| CDX-015 ODrive peer attributes request framing to the spec | **Confirmed.** The supplied PDF notes mark 0x009 as call and response with RTR; 0x017 has only "can be sent to either address". Request DLC is stated nowhere. | Codex's change to `FakeOdrivePeer.h` is comments only; kept. |
| CDX-016 O14 title promises reply checks but checks only requests | **Confirmed** (second pass): `test_odrive_frames.cpp:493-503` asserts only `violations().empty()` and the two request counters; the replies are checked in other cases (peer fake, O42, O38). Low, editorial. | **Fixed 2026-11-09:** the case now also checks the two replies from the peer (data frames, ids 0x009 / 0x017, DLC 8, payload `00 00 C0 3F 00 00 00 00` and `00 00 C0 41 00 00 00 00`). With Codex's fault injection (peer never replies) it now fails; before, it passed. Full driver executable still 930 cases, 0 failed (default, sanitizer, DSP off, clang). |
| CDX-017 Hall-angle tests accept hard-coded default packing | **Confirmed** (second pass): `TMC4671.cpp:1359-1364` packs `pos0 | pos60 << 16` as in datasheet Rev 2.07 (0x34, s16 fields). My hand check of Codex's expected values `0x0929FF85 / 0x0FFFF448 / 0xA4603039` is correct. Mutation (`posA = 0x2AAA0000u`): all 929 other driver cases pass, only Codex's `public Hall setup packs six distinct signed nondefault angles` fails. | Codex's case kept. Not a firmware defect. |
| P12-TICK-001 real HAL_Delay can return early near the maximum delay | **Confirmed by source reading** (second pass): `HAL_Delay` does `wait += uwTickFreq` for any finite delay; with a 100 ms tick and `Delay = 4294967200` the sum wraps to 4. Unsigned arithmetic (not UB); no FFBoard caller changes the tick frequency (Codex's lexical scan; I did not repeat it). | Codex's marked case kept. Vendor code untouched. |
| P12-I2C-001 delayed memory-write interrupt skips the low address byte | **Confirmed by source reading**: `Mem_Write_IT` sets `State = BUSY_TX`, so a BTF event is routed to `I2C_MasterTransmit_BTF`, which writes payload data without the pending address LSB; `I2C_MemoryTransmit_TXE_BTF` (which sends it) is reached only for a TXE event. Whether TXE+BTF can occur on real silicon was not checked (needs hardware; Codex also says so). | Codex's marked case kept. Vendor code untouched. |

## Audit of Codex's changes to shared files

| File | Result |
|---|---|
| `CMakeLists.txt` | Python path fix: correct. New source globs for `codex/…`: correct. New entries `codex_python_tests`, `codex_usb_setup_guard`, opt-in `codex_st_hal`: fine. **Quarantine of the ring-buffer case: reverted** (masks, not fixes). |
| `README.md` | Out-of-tree commands and registrations: fine. Quarantine text and the extended exclusion in the examples: removed. |
| `tools/check_baseline.py` | Duplicate and metadata checks: tightening, no loosening. |
| `tools/coverage_report.py` | Hardening plus gcov on notes-only objects; reproduces the recorded numbers. |
| `tools/run_all_tests.py` | Hardening plus the second Python group. |
| `platform/src/host_freertos_ext.inc` | Sync set-mask guard: matches the kernel. |
| `support/UsbFixture.h` | After `enumerate()` the first fault must be empty. Tightening; all USB tests still pass. |
| `fakes/FakeOdrivePeer.h` | Comments only. |

No Codex edit changes a firmware source, a baseline case, a Claude test, `.github/` or `Tests/Code_Coverage/claude/`.

## Other corrections found during the audit
- `COVERAGE_AND_SCOPE.md` (mine) carried a stale TinyUSB figure of 48.0 %; a fresh gcov run gives 85.1 %. Corrected, and the file now notes the re-measurement.
- Codex's gap list (assertions that lines are covered but not checked) is largely closed by its own new cases, which pass in all variants. Its remaining items are descriptive limits, not defects.

## Left open (needs your decision)
1. **Suspected firmware bug 39 / CDX-010:** a critical error calls `emergencyStop(true)`, which resets rather than engages the stop. Not changed; safety-relevant, already a KNOWN_ISSUE.
2. **Real-hardware questions** (not answerable on the host): whether TXE+BTF can really coincide on the I2C controller (P12-I2C-001), and whether any production path reaches a non-default HAL tick frequency (P12-TICK-001).

## Follow-up edits (2026-11-09, second round)
- Bug 39 stays open as requested.
- `Firmware/Tests/Code_Coverage/claude/cases_rtos/test_ctasklet.cpp`: CDX-012 edit above.
- Codex documents edited, each with a visible "Edited by Claude 2026-11-09" note: `CLOSEOUT_RESULTS.md`, `HANDOFF.md`, `PROGRESS.md`, `REVIEW_FINDINGS.md`, `SHARED_FILE_CHANGES.md`, `COVERAGE_GAPS.md`, `P0_P10_CLOSEOUT.md` (statements corrected) and a notice at the top of seven historical reports (`REGRESSION_RESULTS.md`, `P0_P1_INTEGRATION.md`, five files in `reviews/`) that still mention the removed exclusion.
- Full suite re-run after the tasklet edit: default 12 of 12 (includes the opt-in HAL gate), sanitizers 11 of 11, DSP off 10 of 10, clang 23 11 of 11; 0 failed cases everywhere.
