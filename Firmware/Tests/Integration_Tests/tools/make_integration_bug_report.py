#!/usr/bin/env python3
"""Generates Tests/Integration_Tests/claude/reports/BUG_REPORT.md from the four register tables of
Tests/Integration_Tests/claude/plans/PROGRESS.md (the source of truth): "Integration bugs" (IB n), "Integration NEEDS REVIEW items"
(INR n), "Test tool defects" (T-n) and "Fake audit register" (the fake defects A-n with their fix status).

  python3 -B Firmware/Tests/Integration_Tests/tools/make_integration_bug_report.py [--progress <PROGRESS.md>] [--out <BUG_REPORT.md>]
  python3 -B Firmware/Tests/Integration_Tests/tools/make_integration_bug_report.py --check

The output is deterministic (no date, no commit), so --check can compare it with the file on disk: exit status 1 with a message
when the file differs from what would be generated (nothing is written), 0 when it is current. ctest runs --check
(integration_tool_tests): the report is regenerated whenever a register changes.

Errors (message on stderr, exit status 2, nothing written): a table is missing, a row has another number of cells than the table
header (an unescaped | inside a cell shows up like this; write \\| as in the coverage PROGRESS.md), a number is empty or appears twice.
Same table conventions as Code_Coverage/tools/make_bug_report.py (split_row, "## " headings end a table). Python standard library only;
paths are built with os.path / pathlib, output is written with LF line ends on every platform.
"""
import argparse
import os
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "Firmware" / "Tests" / "Code_Coverage" / "tools"))
sys.dont_write_bytecode = True
import make_bug_report  # noqa: E402  (split_row: the cell splitting of the coverage register tables)

DEFAULT_PROGRESS = REPO / "Tests" / "Integration_Tests" / "claude" / "plans" / "PROGRESS.md"
DEFAULT_OUT = REPO / "Tests" / "Integration_Tests" / "claude" / "reports" / "BUG_REPORT.md"
# Paths as written into the generated file (independent of --progress / --out, so the file is the same wherever it is generated)
TOOL_NAME = "Firmware/Tests/Integration_Tests/tools/make_integration_bug_report.py"
PROGRESS_NAME = "Tests/Integration_Tests/claude/plans/PROGRESS.md"

# heading -> (columns of the header row; the first cell of the header names the number column)
TABLES = {
    "bugs": ("## Integration bugs", ["#", "Module", "Location", "Description", "Same as (coverage)", "Evidence (test)", "Resolved?"]),
    "reviews": ("## Integration NEEDS REVIEW items", ["#", "Module", "Test / location", "Question", "Same as (coverage)", "Resolved?"]),
    "tools": ("## Test tool defects", ["#", "Tool", "Defect", "Status"]),
    "fakes": ("## Fake audit register", ["Fake", "Audit (WO, date)", "Result", "Fix (WO, owner approval)", "Coverage cases changed"]),
}
IQ = re.compile(r"\bIQ(\d+)\b")
FAKE_ID = re.compile(r"\*\*(A-\d+)\b[^*]*\*\*")


class RegisterError(Exception):
    """A register table that cannot be read."""


def table_rows(text, key):
    """Data rows of one table as [(line number in PROGRESS.md, [cells])]. The first table line is the header row (its cell count
    defines the columns), the separator line is skipped."""
    heading, columns = TABLES[key]
    lines = text.splitlines()
    start = None
    for i, line in enumerate(lines):
        if line.strip() == heading:
            start = i
            break
    if start is None:
        raise RegisterError("table '%s' not found" % heading)
    rows = []
    header_seen = False
    for i in range(start + 1, len(lines)):
        line = lines[i]
        if line.startswith("## "):
            break
        if not line.startswith("|"):
            continue
        if set(line.replace("|", "").strip()) <= set("-: "):
            continue
        cells = make_bug_report.split_row(line)
        if not header_seen:
            header_seen = True
            if len(cells) != len(columns):
                raise RegisterError("table '%s': header row (line %d) has %d cells, expected %d" % (
                    heading[3:], i + 1, len(cells), len(columns)))
            names = [" ".join(c.split()).lower() for c in cells]
            if names != [c.lower() for c in columns]:
                raise RegisterError("table '%s': header row (line %d) is '%s', expected '%s'" % (
                    heading[3:], i + 1, " | ".join(cells), " | ".join(columns)))
            continue
        if len(cells) != len(columns):
            raise RegisterError("table '%s', line %d (%s): %d cells, expected %d (unescaped | in a cell?)" % (
                heading[3:], i + 1, cells[0] if cells else "?", len(cells), len(columns)))
        rows.append((i + 1, cells))
    if not header_seen:
        raise RegisterError("table '%s' has no header row" % heading[3:])
    return rows


def number_key(num):
    m = re.match(r"^([A-Za-z-]*?)\s*(\d+)$", num)
    return (m.group(1), int(m.group(2))) if m else (num, 0)


def numbered(rows, heading, key, prefix):
    """Rows keyed by their number cell (whitespace normalised); an empty number, a number whose prefix does not belong to the table and
    a duplicate number are errors. Sorted by number."""
    seen = {}
    for line, cells in rows:
        num = " ".join(cells[0].split())
        cells[0] = num
        if not num:
            raise RegisterError("table '%s', line %d: empty number" % (heading[3:], line))
        if not re.match(r"^%s\d+$" % re.escape(prefix), num):
            raise RegisterError("table '%s', line %d: number '%s' does not look like %sn" % (heading[3:], line, num, prefix))
        if num in seen:
            raise RegisterError("table '%s', line %d: number %s appears twice (also line %d)" % (heading[3:], line, num, seen[num][0]))
        seen[num] = (line, cells)
    return [seen[n][1] for n in sorted(seen, key=key)]


def fake_defects(rows):
    """[(id, row cells)] of the audit rows whose Result names a defect or gap id in bold (**A-1 defect**), sorted by id; the audits
    without such an id are returned separately. A repeated id is an error."""
    defects = {}
    clean = []
    for line, cells in rows:
        ids = FAKE_ID.findall(cells[2])
        if not ids:
            clean.append(cells)
        for fid in ids:
            if fid in defects:
                raise RegisterError("table 'Fake audit register', line %d: %s appears twice (also line %d)" % (line, fid, defects[fid][0]))
            defects[fid] = (line, cells)
    ordered = [(fid, defects[fid][1]) for fid in sorted(defects, key=number_key)]
    return ordered, clean


RESOLVED = re.compile(r"^(yes|fixed|resolved)\b", re.IGNORECASE)


def is_open(kind, cells):
    """Open / resolved by the status cell of the register row: resolved only on an explicit value (yes / fixed / resolved at the start
    of the cell); anything else (no, open, planned, in progress, partly, empty) is open."""
    status = {"bug": 6, "review": 5, "tool": 3, "fake": 3}[kind]  # fake: the Fix column
    return not RESOLVED.match(cells[status])


def tc(text):
    """Text for a cell of a Markdown table row (a | would end the cell)."""
    return text.replace("|", "\\|")


def link_questions(text):
    """Cell text with IQ n turned into links to the open questions file."""
    return IQ.sub(lambda m: "[IQ%s](OPEN_QUESTIONS.md)" % m.group(1), text)


def cell(text):
    return link_questions(text) if text else "(none)"


def parse_register(progress_text):
    """The four tables as a dict of sorted lists; raises RegisterError."""
    bugs = numbered(table_rows(progress_text, "bugs"), TABLES["bugs"][0], number_key, "IB ")
    reviews = numbered(table_rows(progress_text, "reviews"), TABLES["reviews"][0], number_key, "INR ")
    tools = numbered(table_rows(progress_text, "tools"), TABLES["tools"][0], number_key, "T-")
    defects, clean = fake_defects(table_rows(progress_text, "fakes"))
    return {"bugs": bugs, "reviews": reviews, "tools": tools, "fakes": defects, "audits_without_defect": clean}


def render(register):
    bugs, reviews, tools, fakes = register["bugs"], register["reviews"], register["tools"], register["fakes"]
    out = []
    w = out.append
    w("# Integration track: bug report")
    w("")
    w("**Generated file, do not edit by hand.** Generated by `%s` from the tables \"Integration bugs\", \"Integration NEEDS REVIEW items\", "
      "\"Test tool defects\" and \"Fake audit register\" of `%s` (the source of truth). Regenerate with "
      "`python3 -B %s`; `--check` (run by ctest, `integration_tool_tests`) fails when this file is stale." % (TOOL_NAME, PROGRESS_NAME, TOOL_NAME))
    w("")
    w("All faults found by the integration track (`Tests/Integration_Tests/`): suspected firmware bugs (IB n), items that need the "
      "owner's review (INR n), defects found in the shared test fakes (A-n) and in the test tools (T-n). The questions to the owner are in "
      "[OPEN_QUESTIONS.md](OPEN_QUESTIONS.md). The coverage suite has its own report and numbers "
      "([../../../Code_Coverage/claude/reports/BUG_REPORT.md](../../../Code_Coverage/claude/reports/BUG_REPORT.md)); an integration bug "
      "that is the same defect as a coverage bug says so under \"Same as\", the coverage numbers are not reused.")
    w("")
    w("Firmware and vendor code are never fixed by this track: each bug has a test that asserts the intended behaviour and is marked "
      "`KNOWN_ISSUE` (fails as expected, the suite stays green).")
    w("")
    w("## Summary")
    w("")
    w("| Kind | Count | Open | Resolved |")
    w("|---|---|---|---|")
    for title, kind, rows in (("Suspected firmware bugs (IB)", "bug", bugs), ("NEEDS REVIEW items (INR)", "review", reviews),
                              ("Defects in shared test fakes (A)", "fake", [c for _, c in fakes]),
                              ("Defects in test tools (T)", "tool", tools)):
        n_open = sum(1 for r in rows if is_open(kind, r))
        w("| %s | %d | %d | %d |" % (title, len(rows), n_open, len(rows) - n_open))
    w("")
    w("| # | Kind | Module / fake / tool | Status |")
    w("|---|---|---|---|")
    for cells in bugs:
        w("| %s | firmware bug | %s | %s |" % (cells[0], tc(cells[1]), "open" if is_open("bug", cells) else "resolved: " + tc(cells[6])))
    for cells in reviews:
        w("| %s | NEEDS REVIEW | %s | %s |" % (cells[0], tc(cells[1]), "open" if is_open("review", cells) else "resolved: " + tc(cells[5])))
    for fid, cells in fakes:
        w("| %s | fake defect | %s | %s |" % (fid, tc(cells[0]), "open" if is_open("fake", cells) else "fixed"))
    for cells in tools:
        w("| %s | tool defect | %s | %s |" % (cells[0], tc(cells[1]), "open" if is_open("tool", cells) else "resolved"))
    w("")
    w("## 1. Suspected firmware bugs (IB)")
    w("")
    if not bugs:
        w("None.")
        w("")
    for num, module, loc, desc, same, evidence, resolved in bugs:
        w("### %s: %s" % (num, module))
        w("")
        w("- **Where:** %s" % cell(loc))
        w("- **What:** %s" % cell(desc))
        w("- **Same as (coverage):** %s" % cell(same))
        w("- **Test / evidence:** %s" % cell(evidence))
        w("- **Resolved?:** %s" % cell(resolved))
        w("")
    w("## 2. NEEDS REVIEW items (INR)")
    w("")
    if not reviews:
        w("None.")
        w("")
    for num, module, where, question, same, resolved in reviews:
        w("### %s: %s" % (num, module))
        w("")
        w("- **Test / location:** %s" % cell(where))
        w("- **Question:** %s" % cell(question))
        w("- **Same as (coverage):** %s" % cell(same))
        w("- **Resolved?:** %s" % cell(resolved))
        w("")
    w("## 3. Defects found in the shared test fakes")
    w("")
    w("From the fake audits (register \"Fake audit register\"). A gap that blocks the integration is listed like a defect.")
    w("")
    if not fakes:
        w("None.")
        w("")
    for fid, (fake, audit, result, fix, changed) in fakes:
        w("### %s: %s" % (fid, fake))
        w("")
        w("- **Audit (WO, date):** %s" % cell(audit))
        w("- **Result:** %s" % cell(result))
        w("- **Fix (WO, owner approval):** %s" % cell(fix))
        w("- **Coverage cases changed:** %s" % cell(changed))
        w("- **Status:** %s" % ("open" if is_open("fake", [fake, audit, result, fix, changed]) else "fixed"))
        w("")
    clean = register["audits_without_defect"]
    if clean:
        w("Audited without a defect:")
        w("")
        for fake, audit, result, fix, changed in clean:
            w("- %s (%s): %s Fix: %s. Coverage cases changed: %s." % (
                fake, audit, cell(result).rstrip("."), cell(fix).rstrip("."), cell(changed).rstrip(".")))
        w("")
    w("## 4. Defects found in the test tools")
    w("")
    if not tools:
        w("None.")
        w("")
    for num, tool, defect, status in tools:
        w("### %s: %s" % (num, tool))
        w("")
        w("- **Defect:** %s" % cell(defect))
        w("- **Status:** %s" % cell(status))
        w("")
    return "\n".join(out).rstrip("\n") + "\n"


def generate(progress_text):
    """The report text for the content of a PROGRESS.md; raises RegisterError."""
    return render(parse_register(progress_text))


def read_lf(path):
    """File content with LF line ends (None if the file does not exist)."""
    try:
        with open(path, encoding="utf-8", newline="") as f:
            return f.read().replace("\r\n", "\n")
    except OSError:
        return None


def main(argv=None):
    ap = argparse.ArgumentParser(description="Generates the integration track bug report from the PROGRESS.md registers")
    ap.add_argument("--progress", default=str(DEFAULT_PROGRESS), help="PROGRESS.md with the four register tables")
    ap.add_argument("--out", default=str(DEFAULT_OUT), help="report file")
    ap.add_argument("--check", action="store_true", help="exit 1 when the report on disk differs from the generated one (nothing is written)")
    args = ap.parse_args(argv)
    try:
        with open(args.progress, encoding="utf-8") as f:
            text = generate(f.read())
    except OSError as exc:
        print("error: cannot read %s: %s" % (args.progress, exc), file=sys.stderr)
        return 2
    except RegisterError as exc:
        print("error: %s: %s" % (os.path.basename(args.progress), exc), file=sys.stderr)
        return 2
    if args.check:
        current = read_lf(args.out)
        if current is None:
            print("integration bug report is missing: %s; regenerate with: python3 -B %s" % (args.out, TOOL_NAME), file=sys.stderr)
            return 1
        if current != text:
            print("integration bug report is stale: %s differs from the registers in %s; regenerate with: python3 -B %s" % (
                args.out, args.progress, TOOL_NAME), file=sys.stderr)
            return 1
        print("integration bug report is current: %s" % args.out)
        return 0
    with open(args.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print("wrote %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
