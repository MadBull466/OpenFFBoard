**Current status — 2026-11-06:** owner said “Apply both”; this exact reviewed patch is applied, alongside P1 unit registration. The 23 resolved owned Python decorators are removed and 36 ordinary regressions pass. Normal build/regression verification is complete; see ../P0_P1_INTEGRATION.md. The proposal and candidate evidence below are preserved as submission-time history.

# P0 tooling fix proposal — approval required

2026-11-05. Writer: GPT-6.1 Sol, medium effort, `/root/p0_writer`. Proposed shared changes are stored only in [P0_TOOLING_FIXES.patch](P0_TOOLING_FIXES.patch); they have **not** been applied to repository tools. The patch is based on current shared files including the previously approved Python discovery path correction. It changes only `Firmware/Tests/Code_Coverage/tools/{check_baseline,coverage_report,run_all_tests}.py`. No CMake, firmware, vendor, Claude, original baseline or Git change is proposed here.

| Finding | Candidate behavior |
|---|---|
| CDX-002 / P0-T1 | For in-scope firmware/tier-A/driver objects with notes but no counters, invoke gcov on notes. Merge executable line identities with zero runtime counts; create a zero-source-line row only after successful notes collection. Remove both notes-existence-only data-only fallbacks. |
| CDX-003 / P0-T2 | Abort collection on any nonzero gcov status; main reports incomplete coverage and exits 1. Missing commands, malformed JSON, blank successful output, absent files fields and nonlist files fields also become incomplete/nonzero. Every invocation must return at least one JSON object with an explicit files list; an empty list remains legitimate. |
| CDX-004 / P0-T3 | Compare metadata category as well as actual outcome for each nonexcluded original identity. Excluded identities remain explicitly unchecked. |
| CDX-005 / P0-T4 | Empty mandatory Python records become a run problem; nonzero/timeout subprocess status also becomes a problem even if a records file exists. No hard-coded helper-test count. |
| CDX-006 / P0-T5 | Reject nonzero/timeout inventory processes and XML with a non-doctest root. Validate direct list records with nonempty name/file, positive integer source line, no nested run results, and a final matching unskipped summary. Require nonempty primary inventory in `run`; internal filtered inventory probes may legitimately return empty. |
| CDX-007 / P0-T6 | Reject repeated suite/name identities in both baseline and XML with an error diagnostic and exit 2. `--print-current` still works without a readable baseline. |

Coverage preserves genuine data-only objects: real GCC emits a successful JSON document with `files=[]` for the checked `eeprom_addresses.c` and `TMC4671_configs.cpp` notes. This is evidence from gcov, rather than an inference from file existence. Standalone DSP notes are deliberately excluded from this additional collection: its documented section counts linked objects with runtime data, leaving unreferenced static-library objects in the no-data inventory.

The 212-line unified diff has 98 added/deleted content lines combined. It applies to temporary copies without additional dependencies. Candidate tools are at `/tmp/codex-p0-candidate/tools/`. Scratch paths and logs are execution evidence, not committed inputs.

## QC revisions

Initial Astra QC holds QC-1 through QC-3 have been addressed for lead re-review. Nonzero and timeout tests now use genuine nonempty list-shaped XML via internal inventory probes and require the corresponding process-status diagnostic. Removing only the inventory status guard in a scratch candidate causes both named tests to fail, each returning the otherwise valid one-case inventory. Additional cases discriminate empty primary versus empty internal inventories, malformed identities, invalid/zero lines, nested full-run structure, run-result children and missing/mismatched summaries. The sanitizer UB control now matches the actual macro shape: description and skip, without should_fail.

Successful-but-unusable gcov output is tested for counters and notes alongside another valid source. Real command `/bin/true` now produces exit 1 with `Coverage report incomplete: invalid gcov output ... no JSON documents`. Existing 115-test compatibility and real full coverage were rerun after these changes and retained their results below. These revisions are proposed, not applied to shared tools.

## Verification

- Codex suite on current tools: **36 cases; 13 ordinary passes, 23 expected failures**. Each expected failure is an `AssertionError` in the named defect test body, rather than a setup error.
- Temporary test copy with only `@unittest.expectedFailure` lines removed, candidate tools selected with `P0_TOOLS_DIR`: **36 ordinary passes**.
- Existing Claude Python suite copied unchanged to a temporary matching layout, pointing at candidate tools and read-only original firmware helpers: **115 cases; 113 ordinary passes, 2 existing expected failures**.
- Real existing `codex-baseline-cov` artifacts, read-only candidate report with `--no-exclusions`: exit 0; **87 compiled ladder files, 8007/8095 executable lines (98.9%), 0 not compiled**. `TMC4671_configs.cpp` retains a measured 0/0 row. Full report: `/tmp/codex-p0-candidate-current-coverage.md`; stderr empty.
- Real reduced notes/counters reproduction at `/tmp/codex-p0-real-gcov`: candidate exit 0; missing-runtime `HidFFB.cpp` now **0/237**, combined measured **136/373 (36.5%)**, rather than the original false 136/136. `/tmp/codex-p0-candidate-missing-coverage.md`; stderr empty. Different source rows may include compiler header accounting; these numbers describe this artifact set only.
- Real default baseline, both original and candidate checker: exit 0; **318 original identities, 317 checked, 1 excluded**, 646 doctest passed / 0 failed / 4 skipped; 332 new cases. Excluded original: `requesting the forces of a not existing axis is rejected`.

Commands and test scope are recorded in [P0_TEST_CREATION.md](../reviews/P0_TEST_CREATION.md). Fresh independent Astra QC accepted the revision (INITIAL_QC.md); owner approval remains required before shared application. The proposal does not recertify every historical coverage percentage, compiler platform, sanitizer configuration, or semantic assertion inside known-issue firmware cases. The added notes lookup follows the existing CMake source-suffix layout and gcov JSON contract; other generators/compiler formats need separate validation.
