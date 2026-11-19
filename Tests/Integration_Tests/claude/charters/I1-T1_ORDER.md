# WO I1-T1: gate support for up to four parallel agents (as sent, 2026-11-19)

One `integration-test-writer` run. File list: `wo/I1-T1.files`. Owner's decision 2026-11-19: up to 4 agents at once (writers and reviewers together) in the one shared checkout on `add-test-suite`; writers only on disjoint file lists; the lead runs the final gate before each commit.

## Order

You change the integration gate `Firmware/Tests/Integration_Tests/tools/it_gate.py` so several writers can run it at the same time in one working tree, and extend its tests `tools/test_it_gate.py`. Repository root `/home/ben/Documents/OpenFFBoard/testing/OpenFFBoard`, branch `add-test-suite`.

Read first: `CLAUDE.md` ("Integration track", hooks), `Tests/Integration_Tests/claude/plans/AGENT_PLAN.md` sections 2.3, 2.4 and 3, the whole of `it_gate.py` and `test_it_gate.py`, `.claude/hooks/guard_lib.py` (evidence format, `gate_dir`, the commit check: what it reads from the evidence) and `.claude/hooks/require_gate_evidence.py`. You may edit only the two files in `Tests/Integration_Tests/claude/charters/wo/I1-T1.files`. The hooks (`.claude/`) must not change; if the design below cannot work without a hook change, stop and report under "Lead decision needed".

### What to build

1. **Own build folders**: new option `--own-build`: every variant of this run builds in `../ffb-test-build/it-<WO>-<variant>` (resolved like `build_dir_for`) instead of `../ffb-test-build/it-<variant>`. Default unchanged (the lead's final gate uses the shared folders). The compile-reference check stays against the same per-variant references in `it-gate/` (the reference is independent of the folder: check that `normalise_entries` removes the build-dir part so a reference recorded in `it-<variant>` compares equal from `it-<WO>-<variant>`; if not, make it). Cache checks (`check_cache`) apply as before. The evidence records the build folder per variant (it already has `build_dir`) and `own_build: true/false`.
2. **Active work orders**: the lead keeps `Tests/Integration_Tests/claude/charters/wo/ACTIVE` (one WO id per line, `#` comments and blank lines ignored; missing file = no other active WO). In `scope_check`: a changed or untracked path under `Firmware/Tests/` that is not in this WO's list but matches the file list (`wo/<ID>.files`) of another WO named in ACTIVE is **not** an offender; it is listed in the result and the evidence as `other_active: ["<path> (<WO>)", ...]`. A path that this WO's list and another active WO's list both claim, or two other active WOs both claim, while it is changed, **is** an offender ("claimed by active WOs X and Y") — the file lists of parallel writers must be disjoint for changed files. A WO named in ACTIVE whose `.files` is missing is an offender line (configuration error). Paths outside `Firmware/Tests/` and protected paths stay offenders exactly as today. The current WO may or may not be listed in ACTIVE.
3. **Evidence and `files`**: unchanged meaning: `files` / `fingerprint` cover only this WO's files (the commit check must keep working unchanged). Print the `other_active` paths in the console summary next to the scope line.
4. **Docstring and `AGENT_PLAN`-style header**: update the module docstring (per-variant build folder, `--own-build`, ACTIVE rule).
5. **Tests** in `test_it_gate.py` (stdlib `unittest`, the existing style with injected `status_raw` / fake runners, no real builds): `--own-build` folder names for each variant; reference comparison equal across the two folder layouts; ACTIVE parsing (comments, blanks, missing file); scope with another active WO's changed file (OK, listed), with a file claimed twice (offender), with an ACTIVE WO lacking `.files` (offender), with a file of a non-active WO (offender as today), protected path still an offender; evidence contains `own_build` and `other_active`.

### Verification

- `python3 -B -m unittest discover -s Firmware/Tests/Integration_Tests/tools -p "test_it_gate.py"` passes.
- `python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --wo I1-T1 --variants default,m32 --own-build` prints `GATE PASS`. Note: the working tree also holds the paused WO I1-2's uncommitted files (`test_f1_hid.cpp`, `support/FfbHost.*`, `support/ChainScheduler.*`); ACTIVE lists I1-2, so with your change they are `other_active`, not offenders. If the build fails because of them, report it (do not touch them).
- Write paths literally (the hook does not expand shell variables). Stage nothing, commit nothing.

Answer format: files with one-line summaries; the new options and rules; whether the compile reference compares equal from a per-WO folder (and what you changed for it); the test list; the gate summary with the `other_active` lines; anything under "Lead decision needed". If you are within ~10 turns of your limit, stop and report. The last line must be the gate's last line exactly as printed.
