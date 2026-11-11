# P18 T2 work order: bug 225 tests (lead -> writer)

Plan: `Tests/Code_Coverage/claude/plans/P18_32BIT_BUILD_PLAN.md` (approved by the owner 2026-11-11 with option 62 B; T1 approved 2026-11-11). Read it completely, especially section 2 (bug 225) and section 3 (what i386 does not model).

## Facts established by the lead

- `ParsedCommand::adr` is `int64_t` (`Firmware/FFBoard/Inc/CommandHandler.h:70`). `SerialFFB::command` (`Firmware/FFBoard/Src/SerialFFB.cpp:124-251`) guards the effect index with `cmd.adr < effects.size()`.
  - **LP64 (64-bit host):** `size_t` is `unsigned long`, the same rank as `int64_t`. The usual arithmetic conversions make the comparison unsigned, so every negative address is rejected.
  - **ILP32 (target, `FFB_TESTS_32BIT`):** `size_t` is `unsigned int`. It is converted to `int64_t`, so the comparison is signed, any negative address passes the guard, and `effects[adr]` is accessed out of bounds.
  - Addresses `>= 2^32` are rejected on both widths: the comparison is done in `int64_t` on ILP32, in `uint64_t` on LP64.
- Affected commands, with the access the bug causes on ILP32:
  - `mag` getat (read, `:157-158`)
  - `state` getat (read, `:168`)
  - `period`, `duration`, `deadzone` (get and set through `handleGetSet`, `:179-202`)
  - `offset`, `sat`, `coeff` (two fields each, `:193-195`, `:220-222`, `:235-236`)
  - `axisgain` getat and setat (`:208-212`)
  - `type` getat (`:228`)
- Two of these do not crash, and two commands are not affected:
  - **`state` setat** (`:164-166`): the guard passes, then `setEffectState(uint8_t id, ...)` truncates `-1` to 255, and `id >= effects.size()` returns without touching the array (`:114-122`). No out-of-bounds access, but the command answers `OK` instead of `ERR`.
  - **`mag` setat** goes through `setMagnitude(uint8_t)` and **`reset` getat** goes through `free_effect(uint16_t)`. Both truncate before comparing, so bug 225 does not apply to them (truncation is bug 122's territory). Do not test them for bug 225.
- Reachability:
  - **Serial:** the parser turns `<cmd>?-1` and `<cmd>=v?-1` into `adr = -1`, already tested in `claude/cases/test_cmdparser_gaps.cpp:130,147`.
  - **HID:** `HidCommandInterface.cpp:173` copies a `uint64_t addr` into the `int64_t`, so `0xFFFFFFFFFFFFFFFF` becomes -1.
  - **CAN:** `CanCommandInterface.cpp:169` copies an `int64_t`.
- The existing cases `mag rejects reading an address beyond the effects` and `state rejects an address beyond the effects` (`claude/cases_target/test_serial_ffb.cpp:447` and the `state` one) abort on ILP32 and are excluded from ctest there (`FFB_TARGET_TESTS_EXCLUDE_32BIT`). Do not edit that file.
- Markers: `Firmware/Tests/Code_Coverage/support/KnownIssue32.h` (written by the lead, read its comment):
  - **`KNOWN_ISSUE_32BIT(text)`:** a plain test on LP64; `should_fail` on ILP32. Use it for `state` setat with a negative address. The first assertion must be the status check (`ERR` intended, `OK` on ILP32).
  - **`KNOWN_ISSUE_32BIT_UB(text)`:** a plain test on LP64; skipped on ILP32. Use it for every case that indexes `effects[adr]` with a negative address.

## Work order T2-SFB: `claude/cases_target/test_serial_ffb_32bit.cpp`, suite `SerialFFB32`

Create only this file. Its own fixture goes in an anonymous namespace: copy what you need of `SerialFfbFixture` from `test_serial_ffb.cpp:40-75`, under a different name.

Cases (one behaviour each; a loop over addresses within one command is one behaviour):

1. **Each of the ten commands, getat (or the set form named below), with address `-1`:** the status is `ERR`, no reply is produced, and no field of any of the 40 effect slots changes. Compare the full `calc->effects` array before and after, field by field or with a helper comparing the fields the command can touch. Create one effect first with `new`, so that slot 0 is in use and non-default values are visible.
   - Marker `KNOWN_ISSUE_32BIT_UB("bug 225: ...")`.
   - For the setters (`period`, `duration`, `deadzone`, `offset`, `sat`, `coeff`, `axisgain` with setat), use a separate case each, also `KNOWN_ISSUE_32BIT_UB`.
2. **Other negative addresses** in the same cases, or one extra case per command: `-40` (would be slot 0 minus 40 slots), `-41` and `INT64_MIN`. Same expectations.
3. **`state` setat with `-1`:** status `ERR` (first assertion), and no slot's `state` changes. Marker `KNOWN_ISSUE_32BIT("bug 225: ...")`.
4. **Plain cases on both widths** (no marker):
   - For each of the ten commands, getat with `2^32` and `2^32 + 1` is `ERR` and changes nothing. Hand derivation in a comment: on ILP32, `40` is converted to `int64_t` and `4294967296 < 40` is false.
   - Getat with `max_effects` (40) and `max_effects - 1` (39), as a boundary pair per command. Slot 39 is valid and answers `OK` with one reply; for `type` with no effect created the reply is 0 (`FFB_EFFECT_NONE`, `ffb_defs.h`).
5. **One end-to-end case** through the CDC string interface, if `CommandHelpers::CdcConsole` with a failsafe `FFBoardMain` can reach an `fxm` instance without deleting an `FFBHIDMain`. Look at how `test_serial_ffb.cpp` does its end-to-end cases. The case sends `fxm.0.type?-1` and expects an error reply. Marker `KNOWN_ISSUE_32BIT_UB`. If it needs `DELETES_FFBHIDMAIN` or another fixture, add that decorator as the existing cases do. If it is not reachable cleanly, skip item 5 and say why.

Bug text: define one macro `BUG225_TEXT` in your file, like `BUG125_TEXT` in `test_serial_ffb.cpp`. It must say: the type of `adr` and where it is declared, the guard and its lines, the LP64 and ILP32 behaviour, which commands read and which write out of bounds, the reachability (serial `fxm.0.<cmd>?-1`, HID addr all ones, CAN), and that shipped firmware with `FFBHIDEXT` (`F407VG`, `F407VG_DISCO`, `F411RE`) creates `SerialFFB`. Check the last point in `Firmware/Targets/*/Core/Inc/target_constants.h` or wherever `FFBHIDEXT` is defined; if you cannot verify it, leave it out.

File header comment:
- what is tested and the oracle: the intended behaviour is the documented one (`SerialFFB.cpp` registers these commands as "effect adr", and the LP64 host's rejection is the behaviour the existing tests assert; HD for the conversions);
- the width notes;
- the marker list.

## Common rules

- Read `CLAUDE.md`, `Firmware/Tests/README.md` (the `ffb_target_tests` section), `support/KnownIssue32.h`, `support/TestHelpers.h`, `support/CommandHelpers.h`, and `claude/cases_target/test_serial_ffb.cpp` for style.
- Do not edit fixtures, fakes, CMake, the firmware or other tests. Report needed changes under "Changes needed". Never reach into private members.
- Expected values come from the firmware's documented intent or a hand derivation in a comment. Never copy an observed value into an assertion without `characterization:` and report it.
- Build only in fresh directories under `/home/ben/Documents/OpenFFBoard/testing/ffb-test-build/`, with absolute paths, never inside the repo. Build the target `ffb_target_tests` only (`-j6`) in three builds:
  - `t2sfb-64`: `-DFFB_TESTS_USE_DSP=OFF`
  - `t2sfb-san`: `-DFFB_TESTS_USE_DSP=OFF -DFFB_TESTS_SANITIZE=ON`
  - `t2sfb-32`: `-DFFB_TESTS_USE_DSP=OFF -DFFB_TESTS_32BIT=ON`

  Run with `-ts="SerialFFB32"`.
  - Expected on 64-bit and sanitizer: every case passes, nothing skipped.
  - Expected on 32-bit: the `_UB` cases are skipped, the `state` setat case is "failed as expected", and the plain cases pass.
  - Then run the 32-bit executable with `-ts="SerialFFB32" --no-skip -tc="<one UB case>"` and show that it aborts. That is the evidence that the marker is needed.
- `git status --short` must show only your file. Never commit, never push, never write an email address anywhere.
- Report:
  - the file;
  - the case list with markers;
  - the counts of the three builds, from doctest result lines you ran yourself;
  - the `--no-skip` evidence;
  - what you did not test and why;
  - "Changes needed";
  - "Lead decisions needed".
