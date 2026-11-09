# Codex progress — 2026-11-06

Active owner request: finish P12 to completion. Resume from [p12/COMPLETION_TRACKER.md](p12/COMPLETION_TRACKER.md). Native 32-bit real-C work is underway; P12 is not complete and the prior small-group next-step snapshots are historical.

Current active resume: [P12 time/state results](p12/TIME_STATE_RESULTS.md). GPIO and bounded real-C time/DMA-state groups are implemented and self-verified. Next is active-DMA address/access design or independent HAL software contracts. Earlier group snapshots below are chronological evidence, not pending tasks.

P0–P10 review and the subsequently authorized closeout are complete. [CLOSEOUT_RESULTS](CLOSEOUT_RESULTS.md) is the current status index, with exact final counts, commands, exclusions, finding dispositions and retained limits. [Final technical QC](reviews/CLOSEOUT_FINAL_QC.md#final-integrated-release-acceptance) accepts the combined release gates.

All five configurations passed: GCC DSP on 11/11, DSP off 10/10, GCC ASan/UBSan on 11/11 and off 10/10, Clang 23 ASan/UBSan on 11/11. All 318 baseline identities remain present. Forty-four ordinary Codex Python tests and the separate USB fault-boundary gate run automatically. Normal ring quarantine replaces the earlier build-local exclusion. Expected failures/skips retain their documented meanings.

All eight shared-file changes are recorded in [SHARED_FILE_CHANGES](SHARED_FILE_CHANGES.md). Permanent regressions and independent repair/mutation evidence address the authorized actionable gaps; protected test weaknesses, production/vendor defects and external/model limits remain explicitly bounded. Frozen coverage is not current coverage. P11 remains outside this audit.

Branch `add-test-suite`, HEAD `8a8041dcd6fbc8b706383f71cbda3f638961725b`; changes remain uncommitted. No push or Git configuration/remote change. The final documentation continuation checked all five retained manifests and owned-source hashes without repeating completed gates.

## Active phase: P12 ST HAL

The owner explicitly brought ST HAL into scope and deferred P11 on 2026-11-06. [P12 plan](p12/PLAN.md) is the active work record; it supersedes older HAL out-of-scope statements for testing only. Vendor sources remain read-only; other core/kernel/generated-board exclusions remain.

P12.0 source inventory is complete: HAL 1.8.3, 27 C sources in each F407 target, 26 in F411RE; common source bytes match and F411 lacks CAN. Inventory includes headers/config hashes, enabled modules and lexical caller locations. See [inventory](p12/INVENTORY.md). This inventory itself is not coverage. P12 now has the separate first GPIO slice described below; no shared build changes.

First GPIO slice complete: [requirements](p12/GPIO_REQUIREMENTS.md), [results](p12/GPIO_RESULTS.md). Eight ordinary cases / 231 assertions pass in GCC default/full-assert/DISCO/F411/GCC sanitizer/Clang sanitizer; four scratch mutations detected. Original baseline checker remains OK. This is a standalone owned project, not shared CTest integration, complete GPIO coverage or an independently reviewed P12 closeout. Next: source-supported software/error contracts and reference-backed GPIO register/event models. Hardware-semantic model claims needing missing STM32 references remain pending. P0–P10 closeout stays complete; P11 is deferred.

Reference update: owner supplied the STM32F405xx/407xx datasheet; verified DS8626 Rev 12, March 2026, including F407VG. [P12 references](p12/REFERENCES.md) records its hash and targeted reading. The reference manual, errata and F411-specific material remain pending.

Reference update: RM0090 Rev 22 (May 2026) and ES0182 Rev 19 (July 2026) are now verified under `Tests/shared/specs/`. [P12 references](p12/REFERENCES.md) records hashes, targeted GPIO/lock reading and the EXTI10 erratum. Earlier missing-F407-manual/errata statements are historical. Next is register-access model design and targeted EXTI/reference applicability reading; F411-specific references remain outstanding. Existing GPIO results are unchanged.

P12 register-model continuation: [design](p12/REGISTER_MODEL_DESIGN.md) and [results](p12/REGISTER_MODEL_RESULTS.md) now record 36 ordinary model/public-HAL cases / 115 assertions, plus the retained eight C storage cases / 231 assertions. Default, DISCO, GCC sanitizer and Clang sanitizer standalone gates pass 2/2. All nine HAL/model mutants are detected at exact intended identities; original-318 checker remains OK. The adapter compiles the unchanged HAL through a C++ register boundary, separately from the real-C build; ARM/C equivalence, line percentages and independent-agent QC remain unverified. No shared integration/edit. Next is HAL time/state and DMA prerequisites.

Latest P12 time/state group: [requirements](p12/TIME_STATE_REQUIREMENTS.md), [results](p12/TIME_STATE_RESULTS.md). Added 28 ordinary cases and one precise known-issue delay-overflow case, all in real-C vendor builds. Current standalone gates pass in default/DISCO/GCC sanitizer/Clang sanitizer (5/5) and F411 software configuration (4/4); GPIO checks retained. Seven fault controls detected; scratch wider-delay assertions pass and only the known marker flips. All 318 original identities preserved by a fresh existing-binary check. [P12-TICK-001](p12/FINDINGS.md) is reported, not repaired; no direct affected caller established. Next is a safe DMA address/access strategy or other independent HAL contracts. No shared integration, line percentage or fresh independent-agent QC yet.
