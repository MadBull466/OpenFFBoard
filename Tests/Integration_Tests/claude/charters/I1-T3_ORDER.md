# WO I1-T3: faster gate (one integration run, WO-scoped cases, parallel shards and variants) (as sent, 2026-11-23)

One `integration-test-writer` run. File list: `wo/I1-T3.files`. No other agent runs while this WO is open (owner's rule: a tool change is finished, reviewed and committed before dependent work). Owner's decisions: `PROGRESS.md` decisions log 2026-11-23 ("gate speed-up as tool WO I1-T3").

## Background

A two-variant WO gate takes about 30 minutes and the `--full` phase gate about 2 hours. Per variant (I1-8 gate, default): configure + build about 40 s with `-j16`; ctest 407 s, of which `ffb_integration_tests` 389 s; then `run_variant` runs `ffb_integration_tests` a second time for its XML summary (about 6.5 min). The integration executable runs its 325 cases serially on one core; variants run one after another. Machine: 16 cores, 30 GB RAM.

## Owner's decisions (binding)

1. The integration executable runs **once** per variant; the gate's case summary comes from that run.
2. A WO gate runs **only the WO's own integration test files**. Lead's rule for the boundary: the subset is used only when every file of the WO's file list that is not a document is an integration case file (`Firmware/Tests/Integration_Tests/claude/cases/test_*.cpp`); any other code file (support, fakes, `integration.cmake`, `CMakeLists.txt`, tools, anything under `Firmware/Tests/Code_Coverage/`) means the full integration run. `--full` (phase gate) always runs every case. The other ctest entries always run.
3. Cases and variants run **in parallel, at most 10 jobs at once in total** (test processes plus compile jobs plus concurrent variants). A case that fails in a parallel run is **rerun alone, serially**: failing again = a failure; passing on the rerun = the gate still passes but records the case (evidence and a printed warning line); the lead investigates every such case before a commit.
4. The `cov` variant is never sharded (one integration process; coverage files must not be written by concurrent processes).
5. Random-order and repeat runs are **not** part of this WO (they come at the end of the integration testing).
6. The current behaviour stays available behind `--serial` (exactly today's sequence: full ctest including `ffb_integration_tests`, then the XML run, variants one after another).

## Order

Repository root `/mnt/SATA_SSD1/OpenFFBoard Testing/OpenFFBoard` (quote it in every shell command), branch `add-test-suite`, HEAD `a2ad77ce`. Read `CLAUDE.md` ("Integration track", hooks), `Tests/Integration_Tests/claude/plans/AGENT_PLAN.md` 2.3 / 2.4 / 3, the whole of `Firmware/Tests/Integration_Tests/tools/it_gate.py` and `test_it_gate.py`, and how `integration.cmake` registers `ffb_integration_tests` (`add_test` with `--test-case-exclude`).

1. **One integration run.** In the default (non-serial) mode run ctest without the integration entry (`ctest -E '^ffb_integration_tests$'`), keep the check that ctest registers `ffb_integration_tests` and `integration_tool_tests`, and run the executable from the gate with the same exclusion list as ctest (`integration_excludes`). The per-variant result must still fail on any failed case, a timeout, zero cases run, or a summary error.
2. **Per-case result file.** In both modes write `<variant>.integration.cases.json` in the evidence log folder: for every case run, its name, its doctest status (passed / failed / skipped / known issue as `summarise_results.py` classifies it) and its assertion counts. Keep the existing `integration` counts in the evidence. This file is what the equivalence check compares.
3. **WO-scoped cases.** Decide the scope from the WO file list as in decision 2; run the subset with doctest's source-file filter (`-sf`), record `integration_scope` (`"full"` or the file list) in the evidence and print it in the variant line. A WO with no case file and no other code file (documents only) runs the full set.
4. **Shards.** Split the selected cases over several processes of the same executable (doctest's `--first` / `--last` with `--order-by=file`, or explicit `-tc` lists with commas escaped; choose and justify). Every selected case must run exactly once: compare the union of the shard results with the case list (`-ltc` with the same filters); a shard that dies (abort, sanitizer, timeout) leaves cases unrun, which are then run alone (and a crash of a case alone is a failure). Merge the shard results into the one summary and the cases file.
5. **Reruns.** Every case that failed in a shard is run alone once; record `rerun_failed` and `rerun_passed` lists in the evidence; print `WARNING: <n> case(s) passed only when rerun alone: ...` for a non-empty `rerun_passed`. The last line stays exactly `GATE PASS <evidence path>` / `GATE FAIL ...` (the hook `require_gate_evidence.py` parses it; do not change any hook).
6. **Parallel variants and the job budget.** Option `--jobs N` (default 10, values above 10 refused). Run variants concurrently and shards within them so that compile jobs plus test processes never exceed N (for example, build variants with a `-j` share of N, run tests with the remaining slots). Keep each variant's console output together (no interleaved lines). The `--isolated` source tree is shared by all variants of one run (as today).
7. **`--serial`.** Exactly today's behaviour (also writes the cases file).
8. **Tests** (stdlib `unittest`, existing style, injected runner): scope decision (case files only / mixed / support / Code_Coverage / documents only / `--full`); the ctest command without the integration entry; shard partition covers every case exactly once (including names with commas and doctest wildcard characters); a dead shard's unrun cases are run alone; rerun classification (fails again / passes on rerun); merged counts; job budget never exceeded (count concurrent runner calls in the fake runner); `--jobs 11` refused; `cov` unsharded; `--serial` issues the old command sequence; evidence fields.
9. **Verification:**
   - `python3 -B -m unittest discover -s Firmware/Tests/Integration_Tests/tools -p "test_it_gate.py"` and `-p "test_hooks.py"` pass.
   - `python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --wo I1-T3 --variants default,m32 --isolated` prints `GATE PASS` (full integration scope, since the WO changes tools). Report its wall time and the old gate's time for comparison (about 30 min).
   - Do not run the 8-variant equivalence check: the lead runs it after the review (old `--serial` gate and new gate on the same commit, all 8 variants, per-case results, ctest, baseline and coverage numbers identical).
   - Write the status to `/tmp/claude-1000/-mnt-SATA-SSD1-OpenFFBoard-Testing-OpenFFBoard/4de283e1-6ca1-40fb-8fd7-ddbecc886c3d/scratchpad/I1-T3_status.md`. Stage / commit nothing. Report before your turn limit (at about 80 % of it), with what is done and what remains.

Answer: the design (shard method and why, job budget split, scope rule), the changed functions, the tests added, the gate summary with its wall time, and the gate's last line exactly as printed.
