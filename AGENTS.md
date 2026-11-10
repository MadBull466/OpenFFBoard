# Instructions for Codex (and other agents besides Claude)

Mirror of the hard rules in `CLAUDE.md`. Read `Firmware/Tests/README.md` first.

## Your folders
- Test cases and your own fakes: `Firmware/Tests/codex/`. Documents (plan, progress, reports): `Tests/codex/`.
- Claude's work is in `Firmware/Tests/claude/` and `Tests/claude/`: read it, do not edit it.
- Shared, change only after asking the owner: `Firmware/Tests/{platform,support,mocks,fakes,tools,third_party,baseline,cases}`, `Firmware/Tests/CMakeLists.txt`, `CMakePresets.json`, `README.md`, `Tests/shared/`.
  Prefer adding new files (your own fixtures and fakes under `codex/`) over editing shared ones; never edit `third_party/doctest.h` or the original baseline cases.

## Scope notes (mirror of `CLAUDE.md`)
- Since 2026-11-10 `TMCDebugBridge` is in scope (Claude's phase P13; do not duplicate it under `codex/`). `TMCDEBUG` is on in `ffb_driver_tests` only. Since 2026-11-10 the PWM, SimpleMotion and MyActuator drivers and MIDI (`MidiMain`, `MidiHandler`, TinyUSB `class/midi`) are in scope as Claude's phases P14-P17 (do not duplicate them under `codex/`): PWM, SimpleMotion and MyActuator in `ffb_driver_tests`, MIDI in `ffb_usb_tests`, each feature macro on there only.

## Hard rules
- `Firmware/` is read-only except `Firmware/Tests/` (your folder there). Never edit firmware, vendor code (`Firmware/Libraries/`, TinyUSB, `eeprom.c`) or `.github/`. Vendor bugs are reported, never fixed.
- No `#ifdef` for tests in firmware sources; use test doubles in your fakes.
- doctest only; Python tests use stdlib `unittest`. Do not install or download anything. Ask before adding a dependency.
- Build out of tree in `../ffb-test-build/<variant>` (never inside the repo). Never sleep in tests: use the simulated time.
- Specs are in `Tests/shared/specs/` (git-ignored, never commit or download them, never invent register maps or frames).
- Test conventions: fixtures, public interfaces, one behaviour per test, statement names; `characterization:` and `KNOWN_ISSUE*` markers as in `CLAUDE.md`. The original 318 baseline cases must keep passing (`tools/check_baseline.py`).
- Git: branch `add-test-suite`; commit with a one-off identity `git -c user.name="<your name>" -c user.email= commit ...` (email empty, never write the owner's email anywhere); push only to the owner's fork `https://github.com/MadBull466/OpenFFBoard` and only when the owner asks, by URL; never to `origin`, never force, never add a remote or change git config.
- Keep `Tests/COVERAGE_AND_SCOPE.md`-style bookkeeping for your work in `Tests/codex/`.
