> Edited by Claude 2026-11-09: this historical report mentions the CDX-008 ring-buffer exclusion/quarantine. That exclusion no longer exists (the Claude test was fixed instead). See `Tests/Code_Coverage/claude/reports/CODEX_REVIEW_AUDIT.md`. The text below is otherwise unchanged.

# Final technical QC of closeout changes

2026-11-06 · GPT-6 Astra · high effort · assignment `/root/closeout_lead`.

**Accept the bounded implementation and test-oracle resolutions against
[CLOSEOUT_REQUIREMENTS.md](CLOSEOUT_REQUIREMENTS.md), including the final combined
release gates below. No implementation or test-verification hold remains.**
Integration results are coordinator evidence inspected by this lead, not
executions claimed as lead-run. The coordinator's final shared-change ledger
and release manifest remain the record of all cumulative changes and commands.

This decision closes actionable test-infrastructure work within the authorized
scope. It does not declare all firmware/vendor defects repaired, every protected
test rewritten, or external/model limitations verified. P11 is excluded.

## Independent lead assessment

Read all new finite-helper, PID, USB and Hall test bodies, their shared changes,
the ODrive policy addendum and the writer's final report. Inspected the new ring,
RTOS and inputs/buses bodies, CMake registrations, discovery runner and relevant
fixture/model paths. Foundation, RTOS and P5/P6 fresh independent experiments
are attributed to [CLOSEOUT_INDEPENDENT_REVIEW.md](CLOSEOUT_INDEPENDENT_REVIEW.md),
whose final implementation decision and evidence I reviewed. This is not a fresh
audit of every historical P0–P10 test or every linked firmware source.

The numerical predicate explicitly rejects non-finite actual/reference/tolerance,
negative tolerance and overflow in the absolute error. Its ordinary controls
check equality, inclusive tolerance, finite out-of-bound results, NaN and both
infinities in all argument positions, and finite subtraction overflow. No
maximum reduction can silently swallow those observations.

The PID reference uses fixed dyadic gains and the independent positional
equation from zero input history; it reads neither CMSIS A coefficients nor PID
state. Nine signed asymmetric samples exercise state and both output signs.
Each sample is checked through the finite predicate. This is an ordinary public
CMSIS regression for a bounded finite workload, not a specification for arbitrary
non-finite CMSIS inputs or a target-FPU execution result.

The shared USB guard checks successful enumeration before the protected derived
fixture can discard faults. It does not clear faults or alter expected-fault
consumption APIs. The injected fault experiment establishes this boundary rather
than relying on the clean setup case alone. A constructor failure also reaches
the existing base destructor guard, so the retained secondary diagnostic is
expected and does not change the first-failure conclusion.

Hall expected words follow signed low/high 16-bit packing of all six distinct
nondefault inputs. ODrive changes are comments only: source-supported 0x009 RTR
semantics and payload layouts remain distinguished from both request-DLC choices
and the 0x017 request/response trigger adopted locally. No wire behavior, decoder
or response implementation changed. The policy addendum accurately identifies
the protected assertions that enforce local policy.

## Lead-executed controls and discriminators

Durable evidence is in `../ffb-test-build/codex-closeout-final-qc/`:
`independent_runs.json`, `independent_mutations.json`, corresponding XML/stderr
and `reviewed_source_sha256.json`. Commands record exact argv and working
directory. Every selection was checked for a nonzero executed count; suite-filter
XML skips are unselected cases, not newly executed skip coverage. Existing writer
scratch binaries were read-only inputs. No unsafe ring body, skip override or
forced crash/UB was executed.

| Independent selection | Result and decisive observation |
|---|---|
| Finite predicate controls | 3 ordinary passes |
| PID public equation | 1 ordinary pass |
| Hall nondefault packing | 1 ordinary pass |
| USB clean derived setup | 1 ordinary pass |
| Checked Tasklet repair, new marker removed in copied tests | 7 passes |
| Unchecked Tasklet repair, same copied tests | 6 passes / 1 failure; false acceptance, queue count and second payload are independently rejected |
| PID sample 3 replaced with NaN after actual PID execution | 1 failure, only the main finite comparison |
| PID same sample replaced with finite 123 | 1 failure, only the main finite comparison |
| USB public remote-wakeup fault before enumeration | First fails shared `UsbFixture::enumerate` fault boundary; discard marker is never reached |
| Same USB setup with scratch guard removed | Reaches fault consumption and fails only explicit `DISCARD_REACHED` marker |
| Hall default-constant first-pair mutant | 1 failure, only register 0x34; the other pairs and platform guard pass |

The USB guard-off comparison uses a distinct scratch fixture class. The writer's
earlier same-class attempt allowed an existing inline guarded definition to be
selected at link time; it is explicitly invalid and superseded, not counted as
guard-removal evidence. The prior retained Hall mutation object is attributed
to the reviewed P10 experiment; this lead reran its new-case discrimination,
not a fresh vendor-source mutation compilation.

## Foundation, RTOS and input closure

The independent Sol reviewer confirmed mandatory ring quarantine survives hostile
cached exclusions, all 39 Codex Python tests execute (36 tooling regressions plus
three discovery controls), and both Python owners are collected. Safe ring access
is tested without empty typed-return UB; a minimal isolated no-consumption mutant
is rejected. The original 318 identity catalogue is unchanged.

The Sync guard follows the supplied kernel's Sync → SetBits reserved-bit assertion.
Per-bit invalid set/wait inputs and valid highest-user-bit behavior prevent an
indiscriminate rejection implementation from passing. Retained bits after fault
are explicitly a recoverable-fault convention of this fake. Tasklet tests adopt
the existing V12 lock-wait intent and reject unchecked failed-lock scheduling
without executing stale-object callbacks. The stopped/drained timer case uses a
task context; the fresh reviewer also confirmed a copied synchronous destructor
with a daemon barrier passes it. Protected asynchronous queue observations remain
implementation constraints rather than universal lifetime requirements.

ADS endpoint tests accept a valid serialized implementation and reject wrong
values independently of interrupt preconditions. The caller-buffer test detects
delivery into a different destination. Shifter assertions name the exact CS and
retain unrelated ownership. SPI retry now observes payload, transmitted bytes,
completion, exact CS, inactive CS after completion and public semaphore reacquisition.
Review found that unrelated CS lines initially low made the exact destination
ambiguous; the fixture now establishes inactive CS before each test. The fresh
reviewer rebuilt and reran final randomized cases and valid/release-only repair
discriminators after that correction. This is an explicit model precondition,
not proof of physical firmware-startup pin levels.

## Required interpretation of remaining limits

- Original protected NaN-blind reducers remain unchanged. The reusable oracle and
  selected trig/biquad/PID checks are bounded compensation; they do not prove
  every numerical output or every possible workload.
- CDX-016 remains a protected editorial title/body mismatch, with seven prior
  compensating response cases. No missing whole-suite response claim or redundant
  new response test is justified. CDX-015's unsupported vendor fields remain local
  policy, not proven hardware behavior.
- BiSS exact elapsed timeout remains unverified under the non-EXT fake, which
  expires a finite wait on the next explicit activation without advancing time.
  The existing test proves fallback after that modeled event. Scripted tick
  advancement cannot establish an independent deadline oracle.
- Preemption, priority inheritance, arbitrary interrupt ordering, DMA lifetime,
  physical SPI pauses/electrical behavior, target arithmetic, missing sensor/HID/
  GVRET references, version-specific driver questions and Windows/macOS execution
  remain explicit external/model limits. No unavailable specification was invented
  or downloaded.
- Known firmware/vendor defects and crash/UB/sanitizer skips remain open production
  issues. The new expected-failure cases improve discrimination; their green suite
  status is not a product repair.

The inputs writer's old 646-pass unit run included the unsafe ordinary typed-ring
case. It is historical evidence only, not an accepted UB-clean closeout gate;
fresh normal-configured quarantined runs supersede it. Frozen earlier coverage
also remains historical; this review claims no newly measured percentage.

Final release bookkeeping must preserve this distinction, use durable sibling
commands/artifacts rather than disposable `/tmp` dependencies, and enumerate
every cumulative shared-file change in `Tests/Code_Coverage/codex/SHARED_FILE_CHANGES.md`.
The coordinator removed the input reproducer's archival-only `/tmp` self-copy;
the numerical writer likewise removed those incidental self/log-copy dependencies
and documented retained-script prerequisites and rerun order. Existing historical
outputs are preserved. Combined integration gates remain pending at this technical
review's completion and will be recorded in an addendum after the coordinator's
fresh default, DSP-off, GCC sanitizer and Clang sanitizer runs finish.

## Permanent USB negative gate addendum

The coordinator subsequently made the injected-fault proof a routine CTest entry,
`codex_usb_setup_guard`. The existing ordinary USB case opts into the public DCD
fault only when its dedicated environment variable equals `1`; its default run
remains clean. No intentionally failing/skipped case was added to the ordinary
inventory, and the full-report runner needs no exception for this check.

The Codex stdlib runner invokes the exact case, requires one ordinary assertion
failure and no success, checks the first expression and shared fixture location,
requires the injected fault diagnostic and rejects reaching the discard marker.
It rejects abnormal exit, missing/malformed results and any child stderr, including
sanitizer diagnostics. During review, I required replacing removable Python
`assert` validation with explicit checks and rejecting child stderr; both were
corrected. The runner itself uses stdlib unittest, and five permanent protocol
controls cover valid, wrong-first-failure, sanitizer-stderr, signal and discard
results. Codex unittest discovery therefore increases from 39 to 44 cases.

I independently exercised nine synthetic result controls under `python3 -O -B`:
the valid result passed; wrong first expression, zero selected cases, success
exit, sanitizer stderr, signal crash, reached discard marker, duplicate identity
and skipped identity were all rejected. Evidence is under
`codex-closeout-final-qc/usb-runner-controls/`. I also ran the permanent checker
under optimized Python against the coordinator's default USB executable: one
unittest passed and the real injected fault failed at the intended boundary.
Its XML/stderr/result manifest is under `usb-permanent-live/` in the QC directory.
The permanent runner and CMake entry are accepted. Final combined gate counts
must include this additional entry: 11 with DSP enabled and 10 without it.

## Final integrated release acceptance

I directly inspected all five final `evidence/summary.json` files under
`../ffb-test-build/codex-closeout-gate-{default,nodsp,san,nodsp-san,clang23-san}/`,
their configuration command records, and the permanent probe result artifacts.
All owned-source hashes in those five final summaries match the current files.
The command records configure normal default exclusion policy, with no local
`FFB_TESTS_EXCLUDE` override. Earlier 10/9-entry stages and 39-test Python counts
are superseded by these final results.

| Final configuration | CTest | Unit | Target | RTOS | USB | Driver | Baseline identities checked / excluded |
|---|---:|---:|---:|---:|---:|---:|---:|
| GCC DSP on | 11/11 | 652 | 1832 | 278 | 287 | 930 | 317 / 1 |
| GCC DSP off | 10/10 | 652 | 1776 | 278 | 287 | 930 | 317 / 1 |
| GCC ASan/UBSan DSP on | 11/11 | 649 | 1716 | 273 | 287 | 920 | 315 / 3 |
| GCC ASan/UBSan DSP off | 10/10 | 651 | 1661 | 273 | 287 | 920 | 317 / 1 |
| Clang 23 ASan/UBSan DSP on | 11/11 | 649 | 1716 | 273 | 287 | 920 | 315 / 3 |

Executable columns are doctest passed totals, including expected failures; they
are not all ordinary passes. Every listed executable reports zero unexpected
failures. Each configuration also passes the 159 flash cases, 115 Claude Python
tests (two expected failures), 44 ordinary Codex Python tests and the separate
one-unittest USB fault-boundary gate. DSP-on integration additionally executes
the existing P11 executable (218 passes without sanitizers; 191 with 27 skips
under sanitizers); this regression result does not expand this review into P11.

All 318 original identities remain present with the checker reporting OK; its
339 new unit identities have zero failed results. Normal exclusions include the
original nonexistent-axis case and the mandatory nonbaseline unsafe typed-ring
case. DSP-on sanitizer runs also retain the two known spline exclusions. The
normal crash/UB and motor-destruction skip policies remain in force; exclusion
and skip counts are not execution coverage. The 44-test routine group and
permanent USB probe now run automatically in every configuration.

I found and rejected an initial coordinator guard-off proof whose executable and
output-directory paths collided: its exit 1 came from `FileExistsError`, before
running the mutant. I independently reran the existing uniquely named
guard-removed fixture binary with a distinct output directory. The child then
reached `CODEX_USB_DISCARD_REACHED` and first failed `discarded.empty()`; the
permanent checker rejected that different first assertion, as required. This
valid result is retained in
`codex-closeout-final-qc/usb-permanent-guard-off/{probe.xml,result.json}`. Together
with `usb-permanent-live/`, it establishes real guard-on acceptance and real
guard-removal rejection. The invalid path-collision attempt is not evidence of
discrimination and must remain identified as superseded in the coordinator's
archive.

The final gates satisfy the host-test release contract. They do not remove any
source/specification/model/target limit listed above, establish current coverage
percentages, authorize production fixes or certify every possible future repair.
The separate shared-file ledger must cover the eight changed shared files:
CMakeLists, README, three tooling scripts, the RTOS extension, UsbFixture and
FakeOdrivePeer's header. Protected firmware/vendor/Claude/original baseline files
remain unchanged.
