# Instructions for Codex (and other agents besides Claude)

Mirror of the hard rules in `CLAUDE.md`. Read `Firmware/Tests/README.md` first.

## Your folders
- Test cases and your own fakes: `Firmware/Tests/Code_Coverage/codex/`. Documents (plan, progress, reports): `Tests/Code_Coverage/codex/`.
- Claude's work is in `Firmware/Tests/Code_Coverage/claude/` and `Tests/Code_Coverage/claude/`: read it, do not edit it.
- Shared, change only after asking the owner: `Firmware/Tests/Code_Coverage/{platform,support,mocks,fakes,tools,third_party,baseline,cases}`, `Firmware/Tests/CMakeLists.txt`, `CMakePresets.json`, `README.md`, `Tests/Code_Coverage/shared/`.
  Prefer adding new files (your own fixtures and fakes under `codex/`) over editing shared ones; never edit `third_party/doctest.h` or the original baseline cases.

## Scope notes (mirror of `CLAUDE.md`)
- Since 2026-11-10 `TMCDebugBridge` is in scope (Claude's phase P13; do not duplicate it under `codex/`). `TMCDEBUG` is on in `ffb_driver_tests` only. Since 2026-11-10 the PWM, SimpleMotion and MyActuator drivers and MIDI (`MidiMain`, `MidiHandler`, TinyUSB `class/midi`) are in scope as Claude's phases P14-P17 (do not duplicate them under `codex/`): PWM, SimpleMotion and MyActuator in `ffb_driver_tests`, MIDI in `ffb_usb_tests`, each feature macro on there only.
- Since 2026-11-11 (Claude's phase P18) `-DFFB_TESTS_32BIT=ON` (default OFF, Linux gcc / clang with 32-bit multilib only) builds the suite as 32-bit code like the target. In that build the two SerialFFB cases in `FFB_TARGET_TESTS_EXCLUDE_32BIT` are excluded from ctest (bug 225: `int64_t` address compared with `size_t`, out-of-bounds access on 32 bit) and 5 `__int128` DSP test files are left out of `ffb_dsp_tests`. Defects that exist only with a 32-bit `size_t` use `KNOWN_ISSUE_32BIT` / `KNOWN_ISSUE_32BIT_UB` (`support/KnownIssue32.h`): plain on the 64-bit host, should_fail / skipped on 32 bit. If your tests touch such code, keep them green in both widths.

## Hard rules
- `Firmware/` is read-only except `Firmware/Tests/` (your folder there). Never edit firmware, vendor code (`Firmware/Libraries/`, TinyUSB, `eeprom.c`) or `.github/`. Vendor bugs are reported, never fixed.
- No `#ifdef` for tests in firmware sources; use test doubles in your fakes.
- doctest only; Python tests use stdlib `unittest`. Do not install or download anything. Ask before adding a dependency.
- Build out of tree in `../ffb-test-build/<variant>` (never inside the repo). Never sleep in tests: use the simulated time.
- Specs are in `Tests/Code_Coverage/shared/specs/` (git-ignored, never commit or download them, never invent register maps or frames).
- Test conventions: fixtures, public interfaces, one behaviour per test, statement names; `characterization:` and `KNOWN_ISSUE*` markers as in `CLAUDE.md`. The original 318 baseline cases must keep passing (`tools/check_baseline.py`).
- Git: branch `add-test-suite`; commit with a one-off identity `git -c user.name="<your name>" -c user.email= commit ...` (email empty, never write the owner's email anywhere); push only to the owner's fork `https://github.com/MadBull466/OpenFFBoard` and only when the owner asks, by URL; never to `origin`, never force, never add a remote or change git config.
- Keep `Tests/COVERAGE_AND_SCOPE.md`-style bookkeeping for your work in `Tests/Code_Coverage/codex/`.
