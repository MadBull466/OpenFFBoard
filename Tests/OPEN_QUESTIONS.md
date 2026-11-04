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

(Questions from D3 are added here as they come up.)
