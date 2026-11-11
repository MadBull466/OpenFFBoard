#!/usr/bin/env python3
"""
Generates Tests/Code_Coverage/claude/reports/BUG_REPORT.md from the "Suspected firmware bugs" and "NEEDS REVIEW items" tables in
Tests/Code_Coverage/claude/plans/PROGRESS.md (the source of truth). Standard library only.

Usage (from the repo root): python3 -B Firmware/Tests/Code_Coverage/tools/make_bug_report.py [--progress Tests/Code_Coverage/claude/plans/PROGRESS.md] [--out Tests/Code_Coverage/claude/reports/BUG_REPORT.md]
"""
import argparse
import datetime
import re
import subprocess
import sys
from pathlib import Path

# Bugs to look at first, with the reason they are on the list (safety and reachability on the shipped targets F407VG,
# F407VG_DISCO and F411RE first; order reviewed in the final review FR2). Numbers refer to the bug table.
PRIORITY = [
    ("125", "Safety: a periodic effect created over the fxm commands has period 0; on the target likely full-scale torque (FFBHIDEXT on all 3 targets)"),
    ("124", "Device stops: damper/inertia over fxm run a null filter; vector-table read, ~2^32-stage loop until a fault or watchdog reset (all 3 targets)"),
    ("42", "USB bus suspend never reaches the main class: when the PC sleeps, the effects keep running (all targets)"),
    ("40", "Safety: an E-stop press within 1 s of the previous edge is lost (F407VG, F407VG_DISCO)"),
    ("39", "Safety: a new critical error releases an active emergency stop (TMC4671 critical errors on F407VG, F411RE)"),
    ("32", "Safety: a power above 0x7fff reverses the motor at full force (reachable by command on all targets)"),
    ("33", "Safety: after an E-stop reset the slew limiter starts from the torque before the stop"),
    ("12", "Device stops: an HID Set Effect type change to damper/friction/inertia keeps null filters (same mechanism as 124)"),
    ("41", "Flash: the OTP lock bytes can be written, also through sys.signature setat without debug mode (permanent)"),
    ("89", "I2C: a PCF8574 waiting for the bus takes over the ADS111X transfer; port blocked until a timeout reset"),
    ("98", "I2C: a destroyed PCF8574 keeps receiving into its freed buffer"),
    ("110", "CAN: a send with all mailboxes full fails at once instead of waiting (matters for CAN motor drivers on F407VG)"),
    ("V5", "Settings lost: Flash_Format (the format command) when page 1 is active, every target (vendor EEPROM emulation)"),
    ("131", "CAN input main class never starts the CAN port: a fresh F407VG_DISCO board sends nothing"),
]

# Groups of bugs that share a root cause or a pattern (final review FR2); one fix may cover several.
FAMILIES = [
    ("Null effect filters (device stops)", "11 -> 12, 124 (`EffectsCalculator.cpp:416/425/433`, reached from HID PID and from fxm)"),
    ("`EncoderLocal::setPos` (`:80-85`)", "79, 93, 94, 95 (one rewrite fixes all four)"),
    ("GVRET parser (`CanBridge.cpp:156-193`)", "117, 118, 119, 129, 132; NR 137"),
    ("Index clipped to `size()` instead of `size()-1`", "37, 56, 74, 81, 116; also 1, 63, 122 (`idx > size`), 126; NR 120 (`CAN.h:52`, latent)"),
    ("CS pin taken without checking it is free (`hasFreePins`)", "67, 80, 85; related 68, 92"),
    ("Extended CAN frames not distinguished", "59, 72; NR 117 (port side)"),
    ("HAL error leaves the port taken", "16, 58, 86"),
    ("I2C current device lifecycle", "89, 98, 99; NR 113"),
    ("E-stop debounce (`FFBHIDMain.cpp:343`)", "40; NR 39"),
    ("Unchecked bits/mode commands of the SSI encoder", "96, 102, 103, 105, 77, 104"),
    ("USB callbacks with the wrong signature", "42, 43; NR 61 (unreachable while 43 stands)"),
]

NOT_FIRMWARE = {"3": "portability (host)", "4": "withdrawn: same item as NR 2", "6": "test setup", "34": "withdrawn"}
VENDOR = {"2"}  # vendor code with a plain number (CMSIS-DSP); the V-numbered rows are vendor too


def split_row(line):
    cells = re.split(r"(?<!\\)\|", line.strip())
    return [c.strip().replace("\\|", "|") for c in cells[1:-1]]


def table(text, heading):
    start = text.index(heading)
    end = text.find("\n## ", start + len(heading))
    rows = []
    for line in text[start:end if end != -1 else None].splitlines():
        if not line.startswith("|") or set(line.replace("|", "").strip()) <= set("-: "):
            continue
        cells = split_row(line)
        if cells and cells[0] not in ("#",):
            rows.append(cells)
    return rows


def sort_key(num):
    m = re.match(r"(V?)(\d+)", num)
    if not m:
        return (2, 0, num)
    return (1 if m.group(1) else 0, int(m.group(2)), num)


def git_head(repo):
    try:
        return subprocess.run(["git", "-C", str(repo), "log", "-1", "--format=%h"], capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def baseline_known_issues(repo):
    """Known issues of the original suite (commit a5418ff5), from Firmware/Tests/Code_Coverage/baseline/test_cases.txt."""
    path = repo / "Firmware" / "Tests" / "Code_Coverage" / "baseline" / "test_cases.txt"
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("known_issue"):
            result, suite, name = line.split("\t", 2)
            rows.append((result, suite, name))
    return rows


def main():
    repo = Path(__file__).resolve().parents[4]
    ap = argparse.ArgumentParser()
    ap.add_argument("--progress", default=str(repo / "Tests" / "Code_Coverage" / "claude" / "plans" / "PROGRESS.md"))
    ap.add_argument("--out", default=str(repo / "Tests" / "Code_Coverage" / "claude" / "reports" / "BUG_REPORT.md"))
    args = ap.parse_args()

    text = Path(args.progress).read_text(encoding="utf-8")
    for heading in ("## Suspected firmware bugs", "## NEEDS REVIEW items"):
        for row in table(text, heading):
            if len(row) != 5:
                print(f"warning: {heading[3:]} row {row[0]} has {len(row)} cells (unescaped | ?)", file=sys.stderr)
    bugs = sorted(table(text, "## Suspected firmware bugs"), key=lambda r: sort_key(r[0]))
    reviews = sorted(table(text, "## NEEDS REVIEW items"), key=lambda r: sort_key(r[0]))

    firmware = [b for b in bugs if not b[0].startswith("V") and b[0] not in NOT_FIRMWARE and b[0] not in VENDOR]
    vendor = [b for b in bugs if b[0].startswith("V") or b[0] in VENDOR]
    other = [b for b in bugs if b[0] in NOT_FIRMWARE]
    status = lambda r: r[4].lower() if len(r) >= 5 else ""
    open_reviews = [r for r in reviews if status(r) not in ("moved", "n/a", "infra")]
    infra_reviews = [r for r in reviews if status(r) == "infra"]
    closed_reviews = [r for r in reviews if status(r) in ("moved", "n/a")]
    by_num = {b[0]: b for b in bugs}

    modules = {}
    for b in firmware:
        modules[b[1]] = modules.get(b[1], 0) + 1

    out = []
    w = out.append
    w("# OpenFFBoard firmware: suspected bugs and open questions")
    w("")
    w(f"Generated {datetime.date.today().isoformat()} on top of commit `{git_head(repo)}` by `Firmware/Tests/Code_Coverage/tools/make_bug_report.py` from the tables in "
      "`Tests/Code_Coverage/claude/plans/PROGRESS.md` (the source of truth; regenerated after every phase, so do not edit this file by hand).")
    w("")
    w("Every item comes from the host test suite (`Firmware/Tests`) or from code reading during it. None was fixed: the firmware is read only "
      "for the test work. Markers in the \"Test\" lines:")
    w("")
    w("- `KNOWN_ISSUE`: a test describes the intended behaviour and fails today (the suite stays green). Fixing the bug makes the test report "
      "\"should have failed\": then remove the marker.")
    w("- `KNOWN_ISSUE_UB`: the same for undefined behaviour; skipped in sanitizer builds.")
    w("- `KNOWN_ISSUE_UB_HIDDEN`: undefined behaviour whose intended result happens to occur on the host; visible only with `--no-skip` in a sanitizer build.")
    w("- skipped / crash: the case crashes the test run today and is skipped (`doctest::skip`); run it with `--no-skip` after a fix.")
    w("- code reading: no test (not observable through a public interface, or not deterministic).")
    w("")
    baseline = baseline_known_issues(repo)
    w(f"Counts: {len(firmware)} suspected firmware bugs, {len(vendor)} in vendor code (reported, never fixed), {len(other)} not firmware defects, "
      f"{len(open_reviews)} open NEEDS REVIEW questions, {len(baseline)} known issues of the original test suite.")
    w("")
    w("Numbering: bug and NEEDS REVIEW numbers were reserved per test group, so some were never used (vendor V14, V15; NEEDS REVIEW 66-69, "
      "111, 112, 147). Moved or withdrawn items keep their number.")
    w("")
    w("## Look at these first")
    w("")
    for num, why in PRIORITY:
        if num in by_num:
            w(f"- **{'Vendor ' if num.startswith('V') else 'Bug '}{num}** ({by_num[num][1].replace('**', '')}): {why}")
    w("")
    w("## Related bugs (shared root cause or pattern)")
    w("")
    for title, nums in FAMILIES:
        w(f"- **{title}:** {nums}")
    w("")
    w("## Suspected firmware bugs by module")
    w("")
    w("| Module | Bugs |")
    w("|---|---|")
    for mod in sorted(modules, key=str.lower):
        nums = ", ".join(b[0] for b in firmware if b[1] == mod)
        w(f"| {mod} | {nums} |")
    w("")
    w("## Suspected firmware bugs")
    w("")
    for b in firmware:
        num, mod, loc, desc, ev = (b + [""] * 5)[:5]
        w(f"### Bug {num}: {mod}")
        w("")
        w(f"- **Where:** {loc}")
        w(f"- **What:** {desc}")
        w(f"- **Test / evidence:** {ev}")
        w("")
    w("## Vendor code (reported, never fixed)")
    w("")
    for b in vendor:
        num, mod, loc, desc, ev = (b + [""] * 5)[:5]
        w(f"### {'Bug ' if num in VENDOR else ''}{num}: {mod}")
        w("")
        w(f"- **Where:** {loc}")
        w(f"- **What:** {desc}")
        w(f"- **Test / evidence:** {ev}")
        w("")
    w("## Not firmware defects (kept for the record)")
    w("")
    for b in other:
        num, mod, loc, desc, ev = (b + [""] * 5)[:5]
        w(f"- **{num}** ({NOT_FIRMWARE[num]}; {mod}, {loc}): {desc}")
    w("")
    w("## Open questions (NEEDS REVIEW)")
    w("")
    w("Behaviour whose intent is unclear. Most are documented by a `characterization:` test that records what the firmware does today; "
      "a few are from code reading only (the row says so). Answering the question decides whether it becomes a bug (then the test changes "
      "into a `KNOWN_ISSUE`) or stays as it is.")
    w("")
    for r in open_reviews:
        num, mod, loc, q = (r + [""] * 4)[:4]
        w(f"- **NR {num}** ({mod}, {loc}): {q}")
    w("")
    if infra_reviews:
        w("Questions about the test infrastructure or host-only behaviour (not firmware decisions): "
          + "; ".join(f"NR {r[0]} ({r[1]}): {r[3]}" for r in infra_reviews))
        w("")
    w("## Known issues of the original test suite")
    w("")
    w("These `KNOWN_ISSUE` cases were already in the suite before this work (commit a5418ff5) and are kept unchanged. "
      "Bug 1 above is one of them. **\"non numeric values are rejected without blocking the parser\" is a hang:** "
      "`CmdParser.cpp:205-206` does `continue` without moving on, so a typo such as `x.y=abc` over CDC or UART puts the command thread "
      "into an endless loop on every target (final review FR2).")
    w("")
    for result, suite, name in baseline:
        w(f"- {suite}: \"{name}\"" + (" (undefined behaviour)" if result == "known_issue_ub" else ""))
    w("")
    if closed_reviews:
        w("Moved to the bug list or unused: " + ", ".join(f"NR {r[0]} ({r[3]})" for r in closed_reviews) + ".")
        w("")
    Path(args.out).write_text("\n".join(out), encoding="utf-8")
    print(f"wrote {args.out}: {len(firmware)} bugs, {len(vendor)} vendor, {len(other)} other, {len(open_reviews)} open questions")
    return 0


if __name__ == "__main__":
    sys.exit(main())
