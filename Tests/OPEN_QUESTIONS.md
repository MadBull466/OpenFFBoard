# Open questions for me (collected 2026-11-04)

All questions still waiting for my answer, in one place. The numbers match "Open questions for me" in `Tests/PROGRESS.md` (the source; answered ones are struck through there). The roughly 165 smaller "is this intended?" items about firmware behaviour are the NEEDS REVIEW list in `Tests/BUG_REPORT.md` (open questions section) and are not repeated here.

## Project / infrastructure

3. **USB HID PID 1.01 specification (optional).** Give the path of the spec file to have the `HidFFB` characterization tests checked against it. Without it that batch stays characterization.
5. **CI job (proposal, I add it myself; Claude does not touch `.github/`).** No GitHub Actions workflow builds or runs `Firmware/Tests/`. Proposal: `.github/workflows/host-tests.yml` on push/PR to `master` and `development`, `ubuntu-latest`, `actions/checkout` + `git submodule update --init --depth 1 Firmware/Libraries/CMSIS-DSP`, matrix `opts: ["", "-DFFB_TESTS_USE_DSP=OFF", "-DFFB_TESTS_SANITIZE=ON"]`, each `cmake -S Firmware/Tests -B build $opts && cmake --build build -j && ctest --test-dir build --output-on-failure`. MinGW job later (unverified). Runner GCC 13 untested (low risk).
6. **Second checkout.** `Testing2/OpenFFBoard` (branch `feature/host-unit-tests`) holds untracked copies of the v2 files and none of the add-test-suite work. Work continues only in `testing/OpenFFBoard` unless I say otherwise.
7. **Windows / macOS (partly answered).** `ffb_flash_tests` is skipped on macOS (done). Still open: Windows (`VirtualAlloc` at the fixed flash address) is untested; macOS for `ffb_target_tests` (mallinfo shim) is unverified.
8. **Gate wording in CLAUDE.md and TEST_PLAN.md.** Both still say `ffb_unit_tests` gives "316 passed, 2 skipped"; it now gives 646/0/4 on GCC 16 (san 643/0/7, nodsp-san 645/0/5), and the 316/2 baseline subset is checked by `check_baseline.py`. These are my files: shall Claude update the wording?

## TMC4671 (D1, approved)

9. **Q1 Older register map?** The driver uses registers 0x1E, 0x47 and 0x5C, which datasheet Rev 2.07 does not document, and writes 0x41, which Rev 2.07 lists as read-only (bug 160). Was the driver written against an older register map I can supply?
10. **Q2 Engineering sample.** The engineering-sample version value (SI_VERSION 0x00010000) is not in the datasheet; its handling is tested as characterization only.
11. **Q3 CS pins at start-up.** The board init (`main.c:1377`, CubeMX) drives SPI1_SS2/SS3 low and nothing releases them before the first TMC_1 transfer, so a chip on those pins would also be selected. Intended?
12. **Q4 Biquad enable bit (bug 162).** Datasheet §7.2 says bit 31 of CONFIG_DATA, the driver writes bit 0: by the datasheet the flux and torque filters are never enabled. Is there real-hardware evidence that the filters work (which would mean the datasheet is wrong)?

## ODrive (D2, approved)

13. **D2-Q1 Enum numbers.** The axis state, control mode, input mode numbers and error bits are not in the supplied 0.5.6 spec (PDF + CSV). Supply the 0.5.6 enums (`enums.py` or the API reference page) to turn those characterization assertions into spec checks.
14. **D2-Q2 Version.** The driver comment names firmware 0.5.4, the spec is 0.5.6. Any differences for the messages the driver uses (0x01, 0x07, 0x09, 0x0B, 0x0E, 0x10, 0x17)?

## VESC (D3)

15. **D3-Q1 VESC spec gaps.** The supplied `Tests/VESC6_CAN_CommandsTelemetry.pdf` (third-party slides "VESC 6 CAN Formats v0.1") covers the extended-id layout (command in id bits 8-15, unit id in bits 0-7), packet numbers 0-27, SET_* scalings and STATUS 1-5. The driver also relies on these facts, which it does not give:
    1. Byte order of all multi-byte fields (the driver uses big endian).
    2. FILL_RX_BUFFER (5) payload: offset width, data bytes per frame, maximum offset.
    3. FILL_RX_BUFFER_LONG (6) payload: 2-byte offset and data bytes per frame.
    4. PROCESS_RX_BUFFER (7) payload: sender id, send flag, length (bytes 2-3), CRC (bytes 4-5).
    5. PROCESS_SHORT_BUFFER (8) payload: sender id, flag, then COMM id and data.
    6. CRC algorithm and coverage (the driver uses CRC-16/XMODEM, poly 0x1021, init 0, high byte first, over the reassembled payload).
    7. PING (17) / PONG (18) payloads and whether a master should ping.
    8. COMM numbers: FW_VERSION 0, GET_VALUES_SELECTIVE 50, ROTOR_POSITION 22.
    9. FW_VERSION reply layout (major, minor, name, 12-byte UUID, paired flag, test version).
    10. Target VESC firmware version and the compatibility rule (driver: release >= 5.3, test build >= 5.3.51).
    11. GET_VALUES_SELECTIVE request: mask width, bit numbers (driver: bit 8 input voltage, 15 fault code, 16 PID position), frame length.
    12. GET_VALUES_SELECTIVE reply: field order, widths, scales (v_in int16/10, fault uint8, pid_pos int32/1e6), meaning of fault codes, handling of unrequested bits.
    13. POLL_ROTOR_POS: packet number 56, request/reply ids, payload (int32 / 100000 = degrees 0..360), broadcast id 0xFF.
    14. Meaning of id 0xFF (driver default VESC id) and 0x40 (driver's own id), reply addressing.
    15. VESC timeout without commands (driver comment: cuts the motor after 1 s) and the intended refresh period.
    16. Whether unsolicited STATUS frames count as liveness.
    17. Position unit and multi-turn rule (driver: wrap at 180 degrees).
    18. Rounding of ratio * 100000 (driver truncates).
    A matching source would be the VESC firmware files `comm_can.c`, `datatypes.h`, `buffer.c`, `crc.c` and `commands.c` (GET_VALUES_SELECTIVE) of the version the driver targets (5.3 or later).
    **Answered 2026-11-04:** the user supplied the VESC firmware source `Tests/bldc-master` (7.01, git-ignored); all 18 items are taken from it (`Firmware/Tests/driver/VESC_REQUIREMENTS.md`), except question 16.
16. **D3-Q2 VESC (remaining after the source).** The supplied VESC source is 7.01. (a) In firmware 5.3 (the oldest version the driver accepts), did the FW_VERSION reply already have the `pairing_done` byte before the test-version byte? If not, bug B-V6 only affects newer VESC firmware. (b) Should unsolicited VESC STATUS frames count as a sign of life (the driver counts any received frame)?
17. **P11 G1-Q1 sqrt q15 accuracy (V41).** `arm_sqrt_q15` is up to 7 LSB from the rounded double sqrt at 6 of 32767 inputs (e.g. x=30840 gives 31796, reference 31789); ARM's own bound is 6 LSB and its pattern does not contain these inputs. Is ARM's 6 LSB the contract (then V41 becomes a NEEDS REVIEW tolerance item and the test bound is widened to a derived 7.2), or is the full-range accuracy a defect (the KNOWN_ISSUE test stays)? Until you decide, the KNOWN_ISSUE test stays.
