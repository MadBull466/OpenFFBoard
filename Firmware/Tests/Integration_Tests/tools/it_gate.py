#!/usr/bin/env python3
"""Gate of the OpenFFBoard integration track (Tests/Integration_Tests/claude/plans/AGENT_PLAN.md sections 2.3 and 2.4).

  python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --wo <ID> [--variants default,m32] [--full] [--own-build | --isolated] [--files <path>] [--serial] [--jobs N]
  python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --record-compile-reference <build dir> --variant <name>

Per variant (build directory ../ffb-test-build/it-<variant>, outside the repository; with --own-build
../ffb-test-build/it-<WO>-<variant>, so parallel writers do not share a build folder; the lead's final gate uses the
shared folders (an existing build folder whose CMAKE_HOME_DIRECTORY is not this repository's Firmware/Tests is deleted and
configured fresh, with a note in the run log); the compile reference of it-gate/ is the same for both layouts, the build directory is normalised away):
  1. configure (if there is no CMakeCache.txt) and build; a compiler or CMake warning in a file of the WO's file list fails;
  2. ctest --output-on-failure (without the ffb_integration_tests entry, see below; ctest must still register it);
  3. Code_Coverage/tools/check_baseline.py --build-dir <dir>;
  4. ffb_integration_tests, once (see "Speed"); its per-case results go to <variant>.integration.cases.json;
  5. the compile commands of every target other than ffb_integration_tests equal the recorded reference
     (../ffb-test-build/it-gate/compile_reference-<variant>.json).
--isolated (the lead's final gate before a commit): the build uses a source tree outside the repository,
../ffb-test-build/it-iso-<WO>/src = `git archive <HEAD commit id, resolved once>` (the whole repository: the tool tests of
ctest read .claude and Tests/) plus the submodules below Firmware/ (CMSIS-DSP: `git archive` of the commit HEAD records,
from the submodule's local repository) plus exactly this WO's changed / untracked files copied from the working tree
(deleted files removed) plus every changed / untracked / deleted path under Tests/Integration_Tests/ (the documents of
the commit: PROGRESS.md, BUG_REPORT.md, charters; never codex/ or AGENTS.md), so integration_tool_tests checks the
documents being committed. The evidence records `git_head` = the archived commit (the gate fails when HEAD moved during
the run), `isolated`, `source_root` and `docs_overlaid` ({path: blob}; not part of `files`). The scripts of the gate
(check_baseline.py, summarise_results.py, its baseline) are taken from the isolated tree. Build directories
../ffb-test-build/it-iso-<WO>-<variant> (never the writer's it-<WO>-<variant>); both are
deleted and re-created per run. Nothing in the shared working tree, index or stash is touched, so a PASS cannot depend
on another active WO's uncommitted files. The compile reference compares equal because the build directory and the
source root are both normalised (@BUILD@, @SRC@).
After all variants: scope check (git status of Firmware/ against the WO file list and the protected paths of the hooks).
Parallel work orders: Tests/Integration_Tests/claude/charters/wo/ACTIVE (one WO id per line, # comments; missing file =
none) names the other active WOs. A changed path under Firmware/Tests/ that is not in this WO's list but is in the
.files list of exactly one other active WO is not an offender; it is reported as `other_active`. A changed path claimed
by two WOs (this one and an active one, or two active ones) is an offender, and so is an active WO without a .files file.
A path only another active WO lists still goes through the protected-path check (offender when protected). Other active
WOs must use exact paths (no `*?[` in their list) and an id matching ^I[0-9]+-[A-Za-z0-9]+$ (else offender, file not
read); this WO's own list may use patterns. The current WO is skipped in ACTIVE by exact (case-sensitive) id match.
Evidence: ../ffb-test-build/it-gate/<WO>/<UTC timestamp>.json and latest.json, in the format of .claude/hooks/guard_lib.py
(`files` = git blob of every changed or untracked file under Firmware/ that is in the WO list, `fingerprint`;
plus `own_build`, `isolated`, `source_root`, `other_active` and `other_active_blobs` (git blob of each other_active path,
so a later gate can see that another WO's file changed during a run)).
Speed (tool WO I1-T3): the variants run at the same time and never use more than --jobs (default and maximum 10)
compile jobs plus test processes in all (the builds share the budget with -j jobs // variants, ctest and check_baseline take
one slot each, the rest runs the shards). ffb_integration_tests runs once per variant, with the exclusion list of ctest
(integration_excludes): `-ltc` lists the cases, then shards `--order-by=file --first=a --last=b` of the same executable run in
parallel (index ranges of that list: no escaping of commas or doctest wildcards in case names, every case in exactly one
shard). A failed case, and a case that a dead shard (abort, sanitizer, timeout) left unrun, runs alone, one after the other:
failing again = failure; passing alone = the gate passes with `rerun_passed` in the evidence and a WARNING line (the lead
looks at it before a commit). The cov variant is never sharded (one process, one writer of the coverage files). A WO whose
file list holds, besides documents, only integration case files (Firmware/Tests/Integration_Tests/claude/cases/test_*.cpp)
runs only those files' cases (doctest -sf); any other code file, a list without a case file and --full run every case
(`integration_scope` in the evidence). There is no time limit for a whole run, a shard, a rerun, the cov run or --serial (owner's
decision 2026-11-24, tool WO I2-T1); every test case has its own limit inside the executable (support/CaseWatchdog: a host thread
ends the process with exit code 86 and the line `CASE TIMEOUT <s> s: <case> (<file>:<line>)` when one case runs longer than
FFB_IT_CASE_TIMEOUT_S seconds; unset = the compiled default, 0 = off). The gate sets the variable for every integration process
(`case_timeout_s` of the variant in the evidence). A shard that ends that way is a dead shard (its unrun cases run alone); the
timed-out case runs alone once: timing out again = failure `<variant>: case timed out: <case>`, passing = `rerun_passed` with the
reason "timeout in shard" (`rerun_reasons`); the cov run (one process) fails with the case name. `case_timeouts` lists the cases.
--serial keeps the old sequence: variants one after another, the full ctest, then the
executable again for its XML summary; it writes the cases file too. The per-case file lists name, file, line, status and
assertion counts of every case that ran; the old run and the new run must give the same file.
The last line printed is `GATE PASS <evidence path>` or `GATE FAIL <reason>`; exit status 0 / 1.

Python standard library only (plus .claude/hooks/guard_lib.py of this repository).
"""

import argparse
import contextlib
import datetime
import io
import fnmatch
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import threading
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor

sys.dont_write_bytecode = True

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ROOT = os.path.realpath(os.path.join(TOOLS_DIR, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(DEFAULT_ROOT, ".claude", "hooks"))
import guard_lib  # noqa: E402

EXE = "ffb_integration_tests"
REQUIRED_TESTS = ("ffb_integration_tests", "integration_tool_tests")
REQUIRED_VARIANTS = ("default", "m32")
REF_FORMAT = 2
CTEST_TIMEOUT = 600  # seconds per ctest entry (the HAL tests use 600 in sanitizer builds)
CTEST_TOTAL_TIMEOUT = 3600
BASELINE_TIMEOUT = 900
# No time limit for a whole integration run, a shard, an alone rerun, the cov run or --serial (owner's decision 2026-11-24, tool WO
# I2-T1: the suite grows every phase and a whole-run limit failed the I2-7 gate). The limit is per test case, inside the executable
# (support/CaseWatchdog.cpp): the gate hands it over through the environment variable below, per variant.
CASE_TIMEOUT_ENV = "FFB_IT_CASE_TIMEOUT_S"
CASE_TIMEOUT_RC = 86  # exit code of the executable's watchdog (support/CaseWatchdog.h kCaseTimeoutExitCode)
DEFAULT_CASE_TIMEOUT_S = 120  # same as kDefaultCaseTimeoutS in CaseWatchdog.h (a test compares them); derivation in that header
CASE_TIMEOUT_FACTOR = {}  # variant name part -> factor of the default; empty = none needed (see the measurements in CaseWatchdog.h)
CASE_TIMEOUT_LINE = re.compile(r"^CASE TIMEOUT (\d+) s: (.*) \((.*):(\d+)\)\s*$", re.M)
SOURCE_SUFFIXES = (".c", ".cc", ".cpp", ".cxx")
SOURCE_DIR = "Firmware/Tests"
MAX_JOBS = 10  # owner's limit: compile jobs + test processes at once, in total
SHARD_FACTOR = 4  # shards per job slot (more shards than slots balances the load; a dead shard loses few cases)
CASE_DIR = "Firmware/Tests/Integration_Tests/claude/cases/"
COV_FLAGS = "--coverage"  # coverage recipe (cc-cov): CMAKE_C_FLAGS / CMAKE_CXX_FLAGS / CMAKE_EXE_LINKER_FLAGS = --coverage

BASE_VARIANTS = ("default", "m32")
FULL_VARIANTS = ("san", "clang", "clang-san", "nodsp", "nodsp-san", "cov")

# variant -> cmake -D arguments
VARIANT_ARGS = {
    "default": [],
    "m32": ["-DFFB_TESTS_32BIT=ON"],
    "san": ["-DFFB_TESTS_SANITIZE=ON"],
    "clang": ["-DCMAKE_C_COMPILER=clang", "-DCMAKE_CXX_COMPILER=clang++"],
    "clang-san": ["-DCMAKE_C_COMPILER=clang", "-DCMAKE_CXX_COMPILER=clang++", "-DFFB_TESTS_SANITIZE=ON"],
    "nodsp": ["-DFFB_TESTS_USE_DSP=OFF"],
    "nodsp-san": ["-DFFB_TESTS_USE_DSP=OFF", "-DFFB_TESTS_SANITIZE=ON"],
    "cov": ["-DCMAKE_C_FLAGS=" + COV_FLAGS, "-DCMAKE_CXX_FLAGS=" + COV_FLAGS, "-DCMAKE_EXE_LINKER_FLAGS=" + COV_FLAGS],
}


# ------------------------------------------------------------------ pure helpers (unit-tested)

def variant_list(spec, full):
    """Variant names of --variants (comma separated) plus the --full set; unknown names raise ValueError."""
    names = [v.strip() for v in spec.split(",") if v.strip()]
    if full:
        names += [v for v in FULL_VARIANTS if v not in names]
    for v in names:
        if v not in VARIANT_ARGS:
            raise ValueError("unknown variant %r (known: %s)" % (v, ", ".join(VARIANT_ARGS)))
    out = []
    for v in names:
        if v not in out:
            out.append(v)
    return out


def build_dir_for(root, variant, wo=None, iso=False):
    """../ffb-test-build/it-<variant>, resolved from the repository root; with a WO id (--own-build)
    ../ffb-test-build/it-<WO>-<variant>; iso (--isolated) ../ffb-test-build/it-iso-<WO>-<variant>."""
    name = "it-" + variant
    if wo:
        name = "it-%s%s-%s" % ("iso-" if iso else "", re.sub(r"[^A-Za-z0-9._-]", "_", wo), variant)
    return os.path.realpath(os.path.join(root, "..", "ffb-test-build", name))


ACTIVE_REL = "Tests/Integration_Tests/claude/charters/wo/ACTIVE"
WO_DIR_REL = "Tests/Integration_Tests/claude/charters/wo"
ACTIVE_ID = re.compile(r"^I[0-9]+-[A-Za-z0-9]+$")


def parse_active(text):
    """WO ids of the ACTIVE file: one per line, # comments and blank lines ignored, no duplicates."""
    out = []
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if line and line not in out:
            out.append(line)
    return out


def load_active(root, wo=None):
    """(patterns of the other active WOs {id: [pattern]}, configuration errors [str]). The current WO is skipped (exact,
    case-sensitive id match). A missing ACTIVE file means no other active WO. Errors: an id that does not match
    ACTIVE_ID (its file is not read), an unreadable / undecodable ACTIVE or .files file, a missing .files file, a
    pattern with glob characters in the list of another active WO (parallel WOs need exact paths)."""
    try:
        with open(os.path.join(root, ACTIVE_REL), encoding="utf-8") as f:
            ids = parse_active(f.read())
    except FileNotFoundError:
        return {}, []
    except (OSError, ValueError) as e:
        return {}, ["cannot read %s: %s (configuration error)" % (ACTIVE_REL, e)]
    active, errors = {}, []
    for i in ids:
        if i == wo:
            continue
        if not ACTIVE_ID.match(i):
            errors.append("ACTIVE id %r does not match ^I[0-9]+-[A-Za-z0-9]+$ (configuration error)" % i)
            continue
        path = os.path.join(root, WO_DIR_REL, i + ".files")
        try:
            with open(path, encoding="utf-8") as f:
                pats = parse_file_list(f.read())
        except FileNotFoundError:
            errors.append("ACTIVE names WO %s but %s is missing (configuration error)" % (i, path))
            continue
        except (OSError, ValueError) as e:
            errors.append("ACTIVE WO %s: cannot read %s: %s (configuration error)" % (i, path, e))
            continue
        globs = [x for x in pats if any(c in x for c in "*?[")]
        if globs:
            errors.append("active WO %s uses a pattern (%s): parallel WOs need exact paths" % (i, globs[0]))
            continue
        active[i] = pats
    return active, errors


def parse_file_list(text):
    """Patterns of a WO .files file: one repository-relative path or glob per line, # comments."""
    out = []
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            out.append(line)
    return out


def matches(rel, patterns):
    """True when the repository-relative POSIX path matches a pattern (fnmatch; `*` also crosses `/`)."""
    return any(fnmatch.fnmatchcase(rel, p) for p in patterns)


_GCC_WARNING = re.compile(r"^(?P<file>(?:[A-Za-z]:)?[^\s:][^:]*?):(?P<line>\d+)(?::\d+)?: warning: (?P<msg>.*)$")
_DRIVE = re.compile(r"^[A-Za-z]:[\\/]")
_CMAKE_WARNING = re.compile(r"^CMake (?:Deprecation )?Warning(?: \(dev\))? at (?P<file>[^\s:]+):(?P<line>\d+)")


def _rel_of(path, root, base):
    if _DRIVE.match(path):
        path = path.replace("\\", "/")  # C:\... as printed by MinGW / MSVC-style compilers
    full = path if os.path.isabs(path) else os.path.join(base, path)
    full = os.path.realpath(full)
    root = os.path.realpath(root)
    if full == root or not full.startswith(root + os.sep):
        return None
    return os.path.relpath(full, root).replace(os.sep, "/")


def attribute_warnings(log_text, root, patterns, build_dir=None):
    """Warnings of a configure / build log whose file is in the WO list: [(rel path, line, message)].

    Compiler warnings carry the path as passed to the compiler (absolute here); relative paths are resolved
    against the build directory (compile working directory). CMake warnings name the file relative to the
    source directory Firmware/Tests."""
    found = []
    for raw in log_text.splitlines():
        line = raw.strip()
        m = _GCC_WARNING.match(line)
        if m:
            rel = _rel_of(m.group("file"), root, build_dir or root)
            if rel and matches(rel, patterns):
                found.append((rel, int(m.group("line")), m.group("msg")))
            continue
        m = _CMAKE_WARNING.match(line)
        if m:
            rel = _rel_of(m.group("file"), root, os.path.join(root, SOURCE_DIR))
            if rel and matches(rel, patterns):
                found.append((rel, int(m.group("line")), "CMake warning"))
    return found


def is_integration_entry(entry):
    """A compile_commands.json entry of ffb_integration_tests: its object file is under
    CMakeFiles/ffb_integration_tests.dir/ (CMake names the object directory after the target)."""
    out = entry.get("output") or ""
    if out:
        return out.startswith("CMakeFiles/%s.dir/" % EXE) or ("/CMakeFiles/%s.dir/" % EXE) in out
    return ("CMakeFiles/%s.dir/" % EXE) in entry.get("command", "")


def cmake_object_name_form(path):
    """The form CMake gives an absolute path inside the object file name of a source outside its binary tree
    (CMakeFiles/<target>.dir/mnt/SATA_SSD1/OpenFFBoard_Testing/...).
    Observed rule (CMake 4.5.1, Linux; found in compile_commands.json): the leading '/' is dropped, a space becomes '_',
    and the characters + ( ) = , # ~ are kept. NOT verified here: ':' -> '_' (the drive letter of a Windows path,
    'C:/x y' -> 'C_/x_y'); that is taken from cmLocalGenerator::CreateSafeUniqueObjectFileName and was not run.
    Limits (both fail in the safe direction, a false FAIL and never a false PASS): CMake hashes object paths longer than
    CMAKE_OBJECT_PATH_MAX (likely on Windows for the long isolated-tree paths), and the commands quote a path only when
    it contains a space, so quoting differs when only one of the two trees has a space in its path."""
    return path.lstrip("/").replace(" ", "_").replace(":", "_")


# A literal root is replaced only where it starts a path and ends a path component, so '/other/mnt/repo' or '/repo2' keep
# their text when the root is '/mnt/repo' / '/repo'. Start: the beginning, whitespace, a quote, '=' or ',;:(|' before it,
# or a flag glued to it (-I/root, -o/root, "-include/root"). End: '/', a quote, '|' (key separator), the end, or white
# space followed by something that starts the next argument (-flag, '/', a quote) or the end; so for a spaced root
# '<root> 2/inc' and '<root> (copy)' are other folders, not '@SRC@ 2/inc'.
_FLAGS = ("I", "L", "B", "F", "o", "MF", "MT", "MQ", "include", "imacros", "iquote", "isystem", "idirafter", "iprefix",
          "iwithprefix", "iwithprefixbefore", "isysroot", "isystem-after")
_PATH_START = r"""(^|[\s"'=,;:(|]|(?:^|[\s"'])-(?:%s))""" % "|".join(sorted(_FLAGS, key=len, reverse=True))
_PATH_END = r"""(?=/|["'|]|$|\s(?:-|/|["']|$))"""


def _normalise_string(s, build_dir_fwd, src_root_fwd):
    """Backslashes to '/', then the root and build directory in their object-name form (only right after '.dir/' and up to
    a component boundary), then in their literal form (_PATH_START / _PATH_END). The build directory goes first, so a
    build directory inside the source root stays @BUILD@."""
    s = s.replace("\\", "/")
    for root, tag in ((build_dir_fwd, "@BUILD@"), (src_root_fwd, "@SRC@")):
        if root:
            s = re.sub(r"(\.dir/)%s(?=/|$)" % re.escape(cmake_object_name_form(root)), lambda m, t=tag: m.group(1) + t, s)
    for root, tag in ((build_dir_fwd, "@BUILD@"), (src_root_fwd, "@SRC@")):
        if root:
            s = re.sub(_PATH_START + re.escape(root) + _PATH_END, lambda m, t=tag: m.group(1) + t, s)
    return s


def normalise_entries(entries, build_dir, src_root=None):
    """{ "<output>|<file>": command } of the entries of all targets except ffb_integration_tests; the absolute build
    directory is replaced by @BUILD@ and the source root (repository root or isolated source tree) by @SRC@, so two
    build directories or source trees of the same variant compare equal."""
    bd = os.path.realpath(build_dir).replace("\\", "/").rstrip("/")
    sd = os.path.realpath(src_root).replace("\\", "/").rstrip("/") if src_root else None
    out = {}
    for e in entries:
        if is_integration_entry(e):
            continue

        def norm(s):
            if not isinstance(s, str):
                return s
            return _normalise_string(s, bd, sd)
        key = "%s|%s" % (norm(e.get("output", "")), norm(e.get("file", "")))
        out[key] = norm(e.get("command", " ".join(e.get("arguments", []))))
    return out


def compare_entries(reference, current):
    """(missing, extra, changed) key lists of two normalise_entries() dicts."""
    missing = sorted(k for k in reference if k not in current)
    extra = sorted(k for k in current if k not in reference)
    changed = sorted(k for k in reference if k in current and reference[k] != current[k])
    return missing, extra, changed


def reference_path(root, variant):
    return os.path.join(guard_lib.gate_dir(root), "compile_reference-%s.json" % variant)


def read_compile_commands(build_dir):
    with open(os.path.join(build_dir, "compile_commands.json"), encoding="utf-8") as f:
        return json.load(f)


def renorm_reference(entries, root):
    """A stored reference with the repository root replaced by @SRC@ (no-op for references recorded with @SRC@; makes the
    references recorded before the source root was normalised, or with the previous object-name handling, comparable).
    A reference recorded by the previous code at a root without a space holds 'CMakeFiles/x.dir@SRC@/...' (the root's
    '/mnt/..' form was replaced inside the object name); it is mapped to the current 'x.dir/@SRC@/...'. REF_FORMAT is kept:
    the stored files stay valid, so they need no re-recording."""
    rd = os.path.realpath(root).replace("\\", "/").rstrip("/")

    def fix(s):
        return _normalise_string(re.sub(r"(\.dir)(@SRC@|@BUILD@)", r"\1/\2", s), None, rd)
    return {fix(k): fix(v) for k, v in entries.items()}


def record_reference(root, build_dir, variant):
    entries = normalise_entries(read_compile_commands(build_dir), build_dir, root)
    path = reference_path(root, variant)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"format": REF_FORMAT, "variant": variant, "source_build_dir": os.path.realpath(build_dir), "entries": entries},
                  f, indent=1, sort_keys=True)
    return path, len(entries)


def check_compile_reference(root, build_dir, variant, src_root=None):
    """Result dict: ok, entries, missing / extra / changed (first few), message."""
    path = reference_path(root, variant)
    if not os.path.isfile(path):
        return {"ok": False, "message": "no compile reference for variant %s at %s; record it before the integration "
                "executable exists: it_gate.py --record-compile-reference <build dir> --variant %s" % (variant, path, variant)}
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        if data.get("format") != REF_FORMAT:
            return {"ok": False, "message": "compile reference %s has format %r, expected %d: record it again" % (
                path, data.get("format"), REF_FORMAT)}
        ref = renorm_reference(data["entries"], root)
        cur = normalise_entries(read_compile_commands(build_dir), build_dir, src_root or root)
    except (OSError, ValueError, KeyError) as e:
        return {"ok": False, "message": "cannot compare compile commands: %s" % e}
    missing, extra, changed = compare_entries(ref, cur)
    ok = not (missing or extra or changed)
    return {"ok": ok, "entries": len(cur), "missing": missing[:5], "extra": extra[:5], "changed": changed[:5],
            "counts": [len(missing), len(extra), len(changed)],
            "message": "equal (%d entries)" % len(cur) if ok else
            "compile commands differ: %d missing, %d extra, %d changed (e.g. %s)" % (
                len(missing), len(extra), len(changed), (changed or missing or extra)[0])}


def parse_porcelain(raw):
    """[(status, path)] of `git status --porcelain=v1 -z`; renames and copies yield the new path."""
    out = []
    parts = raw.split("\0")
    i = 0
    while i < len(parts):
        e = parts[i]
        i += 1
        if len(e) < 4:
            continue
        status, path = e[:2], e[3:]
        if status[0] in "RC" or status[1] in "RC":
            i += 1  # the old path follows
        out.append((status, path))
    return out


def _scope_result(offenders, changed, other_active):
    return {"result": "FAIL" if offenders else "OK", "offenders": offenders, "changed": sorted(changed),
            "other_active": sorted(p + " (" + w + ")" for p, w in other_active),
            "other_active_paths": sorted(p for p, _ in other_active)}


def scope_check(root, patterns, block_reason=None, status_raw=None, active=None, active_errors=None, wo="this WO"):
    """Scope of the working tree below Firmware/ against the WO file list.

    Returns {"result": "OK"|"FAIL", "offenders": [str], "changed": [rel paths in the list],
             "other_active": ["<path> (<WO>)"], "other_active_paths": [path]}.
    `active` = {WO id: patterns} of the other active WOs, `active_errors` = their configuration errors (offenders).
      - ignored paths outside Firmware/Tests/, and any path outside Firmware/Tests/: offender
      - a changed or untracked path under Firmware/Tests/ that is not in the list: offender, unless exactly one other
        active WO lists it (then it is reported in "other_active"); a path claimed by two WOs is an offender
      - a changed path that the hooks protect (guard_lib.path_block_reason, i.e. not approved): offender, also when only
        another active WO lists it"""
    if block_reason is None:
        def block_reason(rel):
            return guard_lib.path_block_reason(rel, root)
    active = active or {}
    offenders, changed, other_active = list(active_errors or []), [], []
    if status_raw is None:
        r = subprocess.run(["git", "-C", root, "status", "--porcelain=v1", "-z", "-uall", "--ignored", "--no-renames", "--", "Firmware"],
                           capture_output=True, text=True)
        if r.returncode != 0:
            return _scope_result(offenders + ["git status failed: " + " ".join(r.stderr.split())], [], [])
        status_raw = r.stdout
    for status, rel in parse_porcelain(status_raw):
        ignored = status == "!!"
        if not rel.startswith("Firmware/"):
            continue
        if not rel.startswith("Firmware/Tests/"):
            offenders.append("%s: outside Firmware/Tests/ (%s)" % (rel, "ignored file" if ignored else "changed"))
            continue
        if ignored:
            continue  # ignored files below Firmware/Tests/ (build leftovers) are not part of the change
        own = matches(rel, patterns)
        owners = ([wo] if own else []) + [w for w, pats in sorted(active.items()) if matches(rel, pats)]
        if len(owners) > 1:
            offenders.append("%s: claimed by active WOs %s" % (rel, " and ".join(owners)))
            continue
        if not own and not owners:
            offenders.append("%s: not in the WO file list" % rel)
            continue
        reason = block_reason(rel)
        if reason:
            offenders.append("%s: protected (%s)%s" % (rel, reason, "" if own else ", claimed by active WO " + owners[0]))
            continue
        if own:
            changed.append(rel)
        else:
            other_active.append((rel, owners[0]))
    return _scope_result(offenders, changed, other_active)


def make_evidence(root, wo, variants, scope, git_head, result, now=None, requested=None, files_path=None, patterns=None,
                  own_build=False, isolated=False, source_root=None, docs_overlaid=None, integration_scope=None, serial=False,
                  jobs=None):
    """The evidence dict in the format guard_lib.load_evidence / fingerprint expect."""
    files = {rel: guard_lib.blob_of_file(root, rel) for rel in scope.get("changed", [])}
    return {
        "wo": wo,
        "result": result,
        "time_utc": (now or datetime.datetime.now(datetime.timezone.utc)).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "git_head": git_head,
        "files": files,
        "fingerprint": guard_lib.fingerprint(files),
        "variants_requested": list(requested if requested is not None else variants),
        "file_list": files_path,
        "file_patterns": list(patterns or []),
        "variants": variants,
        "scope": {"result": scope["result"], "offenders": scope["offenders"]},
        "own_build": bool(own_build),
        "isolated": bool(isolated),
        "docs_overlaid": dict(docs_overlaid or {}),
        "source_root": source_root or root,
        "other_active": list(scope.get("other_active", [])),
        "other_active_blobs": {rel: guard_lib.blob_of_file(root, rel) for rel in scope.get("other_active_paths", [])},
        "integration_scope": "full" if integration_scope is None else list(integration_scope),
        "serial": bool(serial),
        "jobs": jobs,
        "dead_shards": {v: r["integration"]["dead_shards"] for v, r in variants.items() if r.get("integration", {}).get("dead_shards")},
        "rerun_failed": {v: r["integration"]["rerun_failed"] for v, r in variants.items() if r.get("integration", {}).get("rerun_failed")},
        "case_timeout_s": {v: r["integration"]["case_timeout_s"] for v, r in variants.items()
                           if r.get("integration", {}).get("case_timeout_s") is not None},
        "case_timeouts": {v: r["integration"]["case_timeouts"] for v, r in variants.items() if r.get("integration", {}).get("case_timeouts")},
        "rerun_reasons": {v: r["integration"]["rerun_reasons"] for v, r in variants.items() if r.get("integration", {}).get("rerun_reasons")},
        "rerun_passed": {v: r["integration"]["rerun_passed"] + r["integration"].get("rerun_unrun_passed", [])
                         for v, r in variants.items()
                         if r.get("integration", {}).get("rerun_passed") or r.get("integration", {}).get("rerun_unrun_passed")},
    }


def final_line(ok, evidence_path=None, reason=""):
    return ("GATE PASS %s" % evidence_path) if ok else ("GATE FAIL %s" % (" ".join(reason.split()) or "see the evidence"))  # always one line


def parse_counts(summary_text):
    """(cases, passed, failed, skipped, known) from the 'Test cases:' line of summarise_results.py, or None."""
    m = re.search(r"Test cases:\s*(\d+)\s+passed:\s*(\d+)\s+failed:\s*(\d+)\s+skipped:\s*(\d+)(?:\s+known issue:\s*(\d+))?", summary_text)
    if not m:
        return None
    return tuple(int(x or 0) for x in m.groups())


def parse_ctest_counts(text):
    """(failed, total) of the ctest summary line, or None."""
    m = re.search(r"(\d+)% tests passed(?:, (\d+) tests? failed)? out of (\d+)", text)  # CMake 4: no "failed" part when all passed
    return (int(m.group(2) or 0), int(m.group(3))) if m else None


def read_cache(build_dir):
    out = {}
    try:
        with open(os.path.join(build_dir, "CMakeCache.txt"), encoding="utf-8", errors="replace") as f:
            for line in f:
                m = re.match(r"^([A-Za-z0-9_]+)(?::[A-Z]+)?=(.*)$", line.rstrip("\n"))
                if m:
                    out[m.group(1)] = m.group(2)
    except OSError:
        pass
    return out


def integration_excludes(cache):
    """The --test-case-exclude list ctest uses for ffb_integration_tests, from the cache."""
    names = [x for x in cache.get("FFB_INTEGRATION_TESTS_EXCLUDE", "").split(";") if x]
    if cache.get("FFB_TESTS_SANITIZE", "OFF").upper() in ("ON", "1", "TRUE", "YES"):
        names += [x for x in cache.get("FFB_INTEGRATION_TESTS_EXCLUDE_SANITIZE", "").split(";") if x]
    if cache.get("FFB_TESTS_32BIT", "OFF").upper() in ("ON", "1", "TRUE", "YES"):
        names += [x for x in cache.get("FFB_INTEGRATION_TESTS_EXCLUDE_32BIT", "").split(";") if x]
    return names


# ------------------------------------------------------------------ scope, per-case results, shards (unit-tested)

def is_case_pattern(p):
    """True when the WO-list pattern can only match integration case files: a path (a glob allowed in the file name only)
    `Firmware/Tests/Integration_Tests/claude/cases/test_*.cpp`. A character class ([) cannot be turned into a doctest
    file filter, so it does not count."""
    if not p.startswith(CASE_DIR):
        return False
    rest = p[len(CASE_DIR):]
    return bool(rest) and "/" not in rest and rest.startswith("test_") and rest.endswith(".cpp") and "[" not in rest


def is_document(p):
    """True for a path that holds no code: below the repository's Tests/ (plans, reports, charters), or a .md file."""
    if p[:1] in "*?[":
        return False
    return p.startswith("Tests/") or p.endswith(".md") or p in ("AGENTS.md", "CLAUDE.md")


def integration_scope(patterns, full=False):
    """None = run every integration case; else the list of case-file patterns of the WO. The subset is used only when every
    entry of the WO file list that is not a document is an integration case file; --full, any other code file (support,
    fakes, integration.cmake, CMakeLists.txt, tools, anything under Code_Coverage/) and a list without a case file mean
    the full run."""
    if full:
        return None
    cases = []
    for p in patterns:
        if is_case_pattern(p):
            cases.append(p)
        elif not is_document(p):
            return None
    return cases or None


def gate_scope(patterns, changed, full=False, isolated=False):
    """The case-file subset of a gate run, or None (every case). Only an --isolated run (the tree is exactly HEAD plus
    the WO's files) may use it, and only when integration_scope() allows it by the file list and every changed non-document
    file of the WO (`changed`, the scope check's list) is directly in the cases directory and named test_*.cpp; a WO with
    no changed file runs everything."""
    if not isolated:
        return None
    base = integration_scope(patterns, full)
    if base is None:
        return None
    code = [c for c in changed if not is_document(c)]
    if not code:
        return None
    for c in code:
        rest = c[len(CASE_DIR):] if c.startswith(CASE_DIR) else None
        if rest is None or "/" in rest or not (rest.startswith("test_") and rest.endswith(".cpp")):
            return None
    return base


def aggregate_warning(variants):
    """One WARNING line for every variant that has cases that passed only when rerun alone or dead shards, or None."""
    parts = []
    for v, r in variants.items():
        i = r.get("integration") or {}
        n = len(i.get("rerun_passed") or []) + len(i.get("rerun_unrun_passed") or [])
        d = len(i.get("dead_shards") or [])
        if n or d:
            parts.append("%s: %d case(s) passed only when rerun alone, %d dead shard(s)" % (v, n, d))
    return ("WARNING: " + "; ".join(parts) + " (the lead looks at every one before a commit)") if parts else None


def scope_filters(case_patterns):
    """doctest -sf file patterns of the case files (`?` stands for the path separator, so only the cases directory matches)."""
    return ["*cases?" + p[len(CASE_DIR):] for p in case_patterns]


def partition(n, parts):
    """Contiguous 1-based inclusive ranges (first, last) that cover 1..n exactly once, at most `parts` of them."""
    if n <= 0:
        return []
    parts = max(1, min(parts, n))
    base, extra = divmod(n, parts)
    out, a = [], 1
    for i in range(parts):
        size = base + (1 if i < extra else 0)
        out.append((a, a + size - 1))
        a += size
    return out


def short_file(path):
    path = (path or "").replace("\\", "/")
    i = path.find("Firmware/")
    return path[i:] if i >= 0 else path.rsplit("/", 1)[-1]


def parse_cases_xml(text):
    """[{name, file, line, status, asserts_passed, asserts_failed}] of the TestCase elements of a doctest XML report (or
    list) in document order, or None when there is no complete report (the run died). The status is the one
    summarise_results.py gives: skipped / failed / known (failed as expected, test_case_success true) / passed."""
    a = text.find("<?xml")
    if a < 0:
        a = text.find("<doctest")
    b = text.rfind("</doctest>")
    if a < 0 or b < 0:
        return None
    try:
        root = ET.fromstring(text[a:b + len("</doctest>")])
    except ET.ParseError:
        return None
    out = []
    for tc in root.iter("TestCase"):
        own = tc.find("OverallResultsAsserts")
        failures = int(own.get("failures", "0")) if own is not None else 0
        passed = int(own.get("successes", "0")) if own is not None else 0
        ok = own is None or own.get("test_case_success") == "true"
        if tc.get("skipped") == "true":
            status = "skipped"
        elif not ok:
            status = "failed"
        elif failures:
            status = "known"
        else:
            status = "passed"
        out.append({"name": tc.get("name", ""), "file": short_file(tc.get("filename", "")), "line": int(tc.get("line", "0") or 0),
                    "status": status, "asserts_passed": passed, "asserts_failed": failures})
    return out


def case_key(c):
    return (c["name"], c["file"], c["line"])


def case_counts(cases):
    """(cases, passed, failed, skipped, known) of a case list, counted like summarise_results.py."""
    n = {"passed": 0, "failed": 0, "skipped": 0, "known": 0}
    for c in cases:
        n[c["status"]] += 1
    return len(cases), n["passed"], n["failed"], n["skipped"], n["known"]


def summary_text(cases):
    n, p, f, s, k = case_counts(cases)
    ap = sum(c["asserts_passed"] for c in cases)
    af = sum(c["asserts_failed"] for c in cases)
    lines = ["Format: xml (merged by it_gate.py)", "Test cases: %d  passed: %d  failed: %d  skipped: %d  known issue: %d" % (n, p, f, s, k),
             "Assertions: %d  passed: %d  failed: %d (including the failed-as-expected ones of the known issues)" % (ap + af, ap, af)]
    failed = [c for c in cases if c["status"] == "failed"]
    if failed:
        lines.append("Failed test cases (%d):" % len(failed))
        lines += ["  %s%s" % (c["name"], "  (%s)" % c["note"] if c.get("note") else "") for c in failed]
    return "\n".join(lines) + "\n"


def write_cases_file(path, variant, scope, cases, extra=None):
    """<variant>.integration.cases.json: every case that ran, sorted by (file, line, name), with its status and assertion counts."""
    data = {"variant": variant, "scope": "full" if scope is None else list(scope), "count": len(cases),
            "cases": sorted(cases, key=lambda c: (c["file"], c["line"], c["name"]))}
    data.update(extra or {})
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=1, sort_keys=True)


class Budget:
    """The job budget: at most `total` weight at once, in first-come order (a wide request, a build with -j k, is not starved
    by single test processes). Every subprocess of a variant run holds its weight while it runs."""

    def __init__(self, total):
        self.total, self.free, self.peak = total, total, 0
        self._cond, self._queue = threading.Condition(), []

    @contextlib.contextmanager
    def hold(self, n=1):
        n = max(1, min(n, self.total))
        ticket = object()
        with self._cond:
            self._queue.append(ticket)
            while not (self._queue[0] is ticket and self.free >= n):
                self._cond.wait()
            self._queue.pop(0)
            self.free -= n
            self.peak = max(self.peak, self.total - self.free)
            self._cond.notify_all()
        try:
            yield
        finally:
            with self._cond:
                self.free += n
                self._cond.notify_all()


SANITIZER_REPORT = re.compile(r"ERROR: (?:AddressSanitizer|LeakSanitizer|ThreadSanitizer)|runtime error:")
_STARTED = re.compile(r"<TestCase\s+([^>]*?)(/?)>")


def last_started_case(text):
    """Name of the last test case a (truncated) XML report shows as started (not skipped), or None."""
    name = None
    for m in _STARTED.finditer(text or ""):
        if 'skipped="true"' in m.group(1):
            continue
        n = re.search(r'name="([^"]*)"', m.group(1))
        if n:
            name = (n.group(1).replace("&lt;", "<").replace("&gt;", ">").replace("&quot;", '"').replace("&apos;", "'")
                    .replace("&amp;", "&"))
    return name


def case_timeout_s(variant):
    """The per-case limit in seconds that the gate gives the executable for a variant: the default times the variant's factor
    (the largest factor of the variant's name parts; none = 1)."""
    f = max([CASE_TIMEOUT_FACTOR.get(n, 1) for n in variant.split("-")] + [1])
    return int(DEFAULT_CASE_TIMEOUT_S * f)


def timed_out_case(rc, text):
    """Name of the case that the executable's watchdog ended the process for (exit code CASE_TIMEOUT_RC and a `CASE TIMEOUT` line
    in the output), else None."""
    if rc != CASE_TIMEOUT_RC:
        return None
    m = None
    for m in CASE_TIMEOUT_LINE.finditer(text or ""):
        pass
    return m.group(2) if m else None


def run_integration(job, exe, excl, scope_files, logs, slots, unsharded=False, cwd=None, env=None, label=""):
    """The one run of ffb_integration_tests of a variant, split into shards of the same executable.

    Shard method: doctest `--order-by=file --first=a --last=b` over the case list that `-ltc` prints with the same filters
    (-sf of the WO's case files, --test-case-exclude of ctest). The ranges are indices into that list, so a case name with
    a comma or a doctest wildcard (`*`, `?`) needs no escaping, every case is in exactly one shard, and the same index
    runs one case alone. unsharded (the cov variant) = one process for the whole selection, no list run.
    A doctest XML report names every registered case: a case that was not selected (filter, exclusion, doctest::skip(),
    another shard's range) is `skipped="true"` in it; the cases of the result are the selected ones with their result plus
    those skipped ones (as the old summary counted them). A shard is dead (its results are not used) when its report is
    incomplete, its exit code is not 0 (or 1 explained by a failed case), or its output holds a sanitizer report. A failed
    case and the cases of a dead shard are run alone, one after the other; a case that fails or dies alone, or whose output
    has a sanitizer report, is a failure. In an unsharded run a failed case is a failure whatever its rerun says (the
    rerun is only recorded in cov_alone). There is no time limit for a process or for the sum of them (tool WO I2-T1): the
    executable limits every case itself (watchdog, `env` carries FFB_IT_CASE_TIMEOUT_S). A process that ends with the
    watchdog's exit code and a `CASE TIMEOUT` line is a dead shard (its unrun cases run alone) naming the timed-out case; that
    case runs alone once: it times out again = a failure `<label>: case timed out: <name>`, it passes = `rerun_passed` with the
    reason "timeout in shard" (rerun_reasons). The unsharded (cov) run has no rerun: its case timeout is a failure.
    `job(cmd, log, cwd=, env=, on_done=)` runs one test process (it holds the job budget). Returns a dict: cases, selected,
    problems, shards, rerun_failed, rerun_passed (failed in a shard, passed alone, or timed out in a shard and passed alone),
    rerun_reasons, rerun_unrun_passed, dead_shards, cov_alone, case_timeouts, wall_sum (information only), exe_rc."""
    base = [exe, "-r=xml", "--order-by=file"]
    if scope_files:
        base.append("-sf=" + ",".join(scope_filters(scope_files)))
    if excl:
        base.append("--test-case-exclude=" + ",".join(excl))
    out = {"cases": [], "selected": 0, "problems": [], "shards": 0, "rerun_failed": [], "rerun_passed": [],
           "rerun_unrun_passed": [], "dead_shards": [], "cov_alone": {}, "wall_sum": 0.0, "exe_rc": 0,
           "rerun_reasons": {}, "case_timeouts": []}
    prefix = (label + ": ") if label else ""
    shard_timeouts = set()  # names of the cases the watchdog ended a shard for

    def note_timeout(name):
        if name not in out["case_timeouts"]:
            out["case_timeouts"].append(name)
    others = {}  # key -> case of the registered cases that were not selected (from any complete report)
    clock = threading.Lock()

    def add_time(d):
        with clock:
            out["wall_sum"] += d

    def run(cmd, log, timed=True):
        kw = {}
        if env:
            kw["env"] = env
        if cwd is not None:
            kw["cwd"] = cwd
        if timed:
            kw["on_done"] = add_time  # called by job() with the time of the process itself, not of the wait for a slot
        return job(cmd, log, **kw)

    def bad(msg, rc=1):
        out["problems"].append(msg)
        out["exe_rc"] = rc
        return out

    def clean(rc, cases):
        """rc 0, or rc 1 explained by a failed case; anything else (abort, signal, sanitizer exit code) is not trusted."""
        return rc == 0 or (rc == 1 and any(c["status"] == "failed" for c in cases))

    def take(rc, text, a, b, entries):
        """({index: case} of the selected cases a..b of a run, why the run is dead or None). Only a complete report with an
        explained exit code and no sanitizer report is used."""
        if SANITIZER_REPORT.search(text or ""):
            return {}, "sanitizer report in the output (rc %s)" % rc
        cases = parse_cases_xml(text)
        if cases is None:
            return {}, "no complete report (rc %s%s)" % (rc, ", timed out" if rc == 124 else "")
        if not clean(rc, cases):
            return {}, "exit code %s is not explained by the report" % rc
        by_key = {}
        for c in cases:
            by_key.setdefault(case_key(c), c)
        keys = {case_key(e) for e in entries}
        for k, c in by_key.items():
            if k not in keys and c["status"] == "skipped":
                others.setdefault(k, c)
        got = {}
        for p in range(a, b + 1):
            c = by_key.get(case_key(entries[p - 1]))
            if c is not None and c["status"] != "skipped":
                got[p] = c
        return got, None

    if unsharded:
        rc, text = run(base, logs("integration.xml"))
        hung = timed_out_case(rc, text)
        if hung is not None:
            note_timeout(hung)
            return bad("%scase timed out: %s (unsharded run, nothing is rerun)" % (prefix, hung), rc)
        everything = parse_cases_xml(text)
        if everything is None:
            return bad("%s died without a report (rc %s)" % (EXE, rc), rc)
        entries = [c for c in everything if c["status"] != "skipped"]
        out["shards"] = 1
        results, why = take(rc, text, 1, len(entries), entries) if entries else ({}, None)
        if entries and not results:
            return bad("%s run is not usable: %s (unsharded run, nothing is rerun)" % (EXE, why), rc)
        for c in everything:
            if c["status"] == "skipped":
                others.setdefault(case_key(c), c)
    else:
        rc, text = run(base + ["-ltc"], logs("integration.list.xml"), timed=False)
        entries = parse_cases_xml(text)
        if rc != 0 or entries is None:
            return bad("cannot list the test cases of %s (rc %s)" % (EXE, rc), rc)
        ranges = partition(len(entries), slots * SHARD_FACTOR)
        out["shards"] = len(ranges)
        results = {}
        if ranges:
            def shard(k):
                a, b = ranges[k]
                return run(base + ["--first=%d" % a, "--last=%d" % b], logs("integration.shard%03d.xml" % (k + 1)))
            with ThreadPoolExecutor(max_workers=max(1, slots)) as ex:
                runs = list(ex.map(shard, range(len(ranges))))
            for k, ((a, b), (rc, text)) in enumerate(zip(ranges, runs)):
                got, why = take(rc, text, a, b, entries)
                results.update(got)
                if why:
                    hung = timed_out_case(rc, text)
                    if hung is not None:
                        shard_timeouts.add(hung)
                        note_timeout(hung)
                        why = "case timed out: %s (%s)" % (hung, why)
                    out["dead_shards"].append({"shard": k + 1, "first": a, "last": b, "rc": rc, "reason": why,
                                               "last_started": last_started_case(text),
                                               "log": logs("integration.shard%03d.xml" % (k + 1))})
    out["selected"] = len(entries)
    if not entries:
        return bad("%s ran zero test cases" % EXE)

    failed = [p for p in range(1, len(entries) + 1) if p in results and results[p]["status"] == "failed"]
    unrun = [p for p in range(1, len(entries) + 1) if p not in results]
    for p in sorted(failed + unrun):
        rc, text = run(base + ["--first=%d" % p, "--last=%d" % p], logs("integration.alone%04d.xml" % p))
        gots, why = take(rc, text, p, p, entries)
        got = gots.get(p)
        name = entries[p - 1]["name"]
        hung = timed_out_case(rc, text)
        if hung is not None:
            note_timeout(hung)
        if unsharded:
            out["cov_alone"][name] = "passed" if (got is not None and got["status"] != "failed") else "failed"
            continue  # the failure of the full-order run stands
        if got is not None and got["status"] != "failed":
            results[p] = got
            if name in shard_timeouts:
                out["rerun_passed"].append(name)
                out["rerun_reasons"][name] = "timeout in shard"
            else:
                out["rerun_passed" if p in failed else "rerun_unrun_passed"].append(name)
            continue
        if got is None:
            got = dict(entries[p - 1], status="failed", asserts_passed=0, asserts_failed=0,
                       note="no complete result when run alone: %s" % (why or "no result"))
            if hung is not None:
                got["note"] = "case timed out when run alone (%s)" % why
                out["problems"].append("%scase timed out: %s" % (prefix, hung))
                out["exe_rc"] = rc
        results[p] = got
        out["rerun_failed"].append(name)
    out["cases"] = [results[p] for p in range(1, len(entries) + 1)] + list(others.values())
    nf = [c for c in out["cases"] if c["status"] == "failed"]
    if nf:
        bad("%d integration case(s) failed, first: %s" % (len(nf), nf[0]["name"]))
    return out


# properties of the integration test in `ctest --show-only=json-v1` that the gate reproduces (WORKING_DIRECTORY) or that carry no
# behaviour (CMake's own source location); any other property (WILL_FAIL, *_REGULAR_EXPRESSION, SKIP_RETURN_CODE, TIMEOUT,
# DISABLED, ...) would change what ctest counts as a pass and is a mismatch
CTEST_REPRODUCED_PROPERTIES = ("WORKING_DIRECTORY", "_BACKTRACE_TRIPLES")


def ctest_integration_mismatch(text, exe, excl, wd_default=None):
    """(working directory to use or None, [mismatch]) between the integration test that ctest registers
    (`ctest --show-only=json-v1 -R '^ffb_integration_tests$'`) and the command the gate runs: executable, arguments after it
    (only --test-case-exclude=<the exclusion list>), working directory and environment properties (the gate sets none)."""
    try:
        tests = json.loads(text).get("tests", [])
    except (ValueError, AttributeError):
        return None, ["cannot read the ctest test definition of %s" % EXE]
    tests = [t for t in tests if t.get("name") == EXE]
    if len(tests) != 1:
        return None, ["ctest lists %d tests named %s, expected 1" % (len(tests), EXE)]
    t = tests[0]
    cmd = t.get("command") or []
    want = (["--test-case-exclude=" + ",".join(excl)] if excl else [])
    bad = []
    if not cmd or os.path.realpath(cmd[0]) != os.path.realpath(exe):
        bad.append("executable %s, the gate runs %s" % (cmd[0] if cmd else None, exe))
    if list(cmd[1:]) != want:
        bad.append("arguments %s, the gate uses %s" % (list(cmd[1:]), want))
    wd = None
    for prop in t.get("properties", []):
        if prop.get("name") == "WORKING_DIRECTORY":
            wd = prop.get("value")
        elif prop.get("name") in ("ENVIRONMENT", "ENVIRONMENT_MODIFICATION"):
            bad.append("ctest sets %s %s, the gate sets none" % (prop["name"], prop.get("value")))
        elif prop.get("name") == "TIMEOUT" and prop.get("value") == 0:
            pass  # no ctest time limit for this entry (integration.cmake): the gate runs it without a process limit too
        elif prop.get("name") not in CTEST_REPRODUCED_PROPERTIES:
            bad.append("ctest sets the property %s (%s) that the gate does not reproduce" % (prop.get("name"), prop.get("value")))
    return wd, bad


# ------------------------------------------------------------------ running things

def _truthy(v):
    return str(v).strip().upper() in ("ON", "1", "TRUE", "YES", "Y")


def check_cache(cache, variant):
    """Mismatches between a CMakeCache.txt (dict) and the variant it is used as: the values that define the variant.
    A stale or hand-configured build directory must not be trusted."""
    bad = []
    names = variant.split("-")
    want = {
        "FFB_TESTS_32BIT": variant == "m32",
        "FFB_TESTS_SANITIZE": "san" in names,
    }
    for k, w in want.items():
        if _truthy(cache.get(k, "OFF")) != w:
            bad.append("%s is %s, the variant %s needs %s" % (k, cache.get(k, "(unset)"), variant, "ON" if w else "OFF"))
    dsp = cache.get("FFB_TESTS_USE_DSP", "AUTO").upper()
    if ("nodsp" in names) != (dsp == "OFF"):
        bad.append("FFB_TESTS_USE_DSP is %s, the variant %s needs %s" % (dsp, variant, "OFF" if "nodsp" in names else "AUTO or ON"))
    if not _truthy(cache.get("FFB_TESTS_INTEGRATION", "")):
        bad.append("FFB_TESTS_INTEGRATION is %s, it must be ON" % cache.get("FFB_TESTS_INTEGRATION", "(unset)"))
    cxx = os.path.basename(cache.get("CMAKE_CXX_COMPILER", "")).lower()
    cc = os.path.basename(cache.get("CMAKE_C_COMPILER", "")).lower()
    is_clang = "clang" in names
    for label, c in (("CMAKE_CXX_COMPILER", cxx), ("CMAKE_C_COMPILER", cc)):
        if c and ("clang" in c) != is_clang:
            bad.append("%s is %s, the variant %s needs %s" % (label, c, variant, "clang" if is_clang else "a non-clang compiler"))
        if not c and is_clang:
            bad.append("%s is not set in the cache" % label)
    cov = "cov" in names
    for k in ("CMAKE_C_FLAGS", "CMAKE_CXX_FLAGS", "CMAKE_EXE_LINKER_FLAGS"):
        if (COV_FLAGS in cache.get(k, "")) != cov:
            bad.append("%s is %r, the variant %s %s --coverage" % (k, cache.get(k, ""), variant, "needs" if cov else "must not have"))
    return bad


def registered_tests(text):
    """Names of the tests listed by `ctest -N`."""
    return set(re.findall(r"^\s*Test\s+#\d+:\s+(\S+)\s*$", text, re.M))


def has_build_target(text, name):
    """True when the output of `cmake --build <dir> --target help` lists the target."""
    return any(l.strip() in ("... " + name, "... %s (the default if no target is provided)" % name) for l in text.splitlines())


def syntax_jobs(entries, root, patterns):
    """Compile commands (argument lists, -o dropped, -fsyntax-only added) of the C/C++ sources of the WO list."""
    jobs, seen = [], set()
    for e in entries:
        directory = e.get("directory", root)
        rel = _rel_of(e.get("file", ""), root, directory)
        if not rel or not rel.endswith(SOURCE_SUFFIXES) or not matches(rel, patterns):
            continue
        if "arguments" in e:
            args = list(e["arguments"])
        else:
            args = shlex.split(e["command"], posix=(os.name != "nt"))
        out, skip = [], False
        for a in args:
            if skip:
                skip = False
            elif a == "-o":
                skip = True
            elif a.startswith("-o") and len(a) > 2:
                pass
            else:
                out.append(a)
        out.append("-fsyntax-only")
        key = (directory, tuple(out))
        if key not in seen:
            seen.add(key)
            jobs.append((directory, out))
    return jobs


def syntax_recheck(jobs, runner, workers=None):
    """Runs every job; returns (combined output, number of failed jobs). An incremental build only recompiles what
    changed, so a warning of an unchanged WO source (or of a WO header seen through another source) would be missed;
    this pass compiles each listed source again, without output. A warning that is only in a WO header and appears
    only when a source that includes it is rebuilt can still be missed here when no listed source includes it."""
    if not jobs:
        return "", 0
    from concurrent.futures import ThreadPoolExecutor
    with ThreadPoolExecutor(max_workers=max(1, workers or os.cpu_count() or 2)) as ex:
        results = list(ex.map(lambda j: runner(j[1], None, cwd=j[0], timeout=BASELINE_TIMEOUT), jobs))
    return "\n".join(t for _, t in results), sum(1 for rc, _ in results if rc != 0)


def run(cmd, log_path, cwd=None, timeout=None, env=None):
    """Runs cmd, writes stdout+stderr to log_path (unless None), returns (returncode, text). rc 124 = timeout.
    env = variables added to the environment of the process (the rest is inherited)."""
    full_env = None if not env else dict(os.environ, **env)
    try:
        p = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                           errors="replace", timeout=timeout, env=full_env)
        rc, text = p.returncode, p.stdout
    except subprocess.TimeoutExpired as e:
        out = e.stdout if isinstance(e.stdout, str) else (e.stdout or b"").decode("utf-8", "replace")
        rc, text = 124, out + "\n[timeout after %s s]" % timeout
    except OSError as e:
        rc, text = 127, "cannot run %s: %s" % (cmd[0], e)
    if log_path:
        with open(log_path, "w", encoding="utf-8") as f:
            f.write(text)
    return rc, text


def iso_paths(root, wo):
    """(base folder, source root) of the isolated tree of a WO: ../ffb-test-build/it-iso-<WO>[/src]."""
    safe = re.sub(r"[^A-Za-z0-9._-]", "_", wo)
    base = os.path.realpath(os.path.join(root, "..", "ffb-test-build", "it-iso-" + safe))
    return base, os.path.join(base, "src")


def head_submodules(root):
    """[(path, commit)] of the gitlinks of HEAD below Firmware/."""
    r = subprocess.run(["git", "-C", root, "ls-tree", "-r", "-z", "HEAD", "Firmware"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if r.returncode != 0:
        raise OSError("git ls-tree failed: " + " ".join(r.stderr.split()))
    out = []
    for line in r.stdout.split("\0"):
        meta, _, path = line.partition("\t")
        parts = meta.split()
        if len(parts) == 3 and parts[0] == "160000":
            out.append((path, parts[2]))
    return out


def _extract_tar(data, dest):
    with tarfile.open(fileobj=io.BytesIO(data)) as t:
        if hasattr(tarfile, "fully_trusted_filter"):
            t.extractall(dest, filter="fully_trusted")  # our own `git archive`
        else:
            t.extractall(dest)


def copy_submodules(root, sroot):
    """Extracts `git archive <recorded commit>` of every submodule below Firmware/ into the isolated tree (an exact copy
    of the commit HEAD records, whatever the submodule's checkout looks like); the commit must be available locally."""
    for path, commit in head_submodules(root):
        sub = os.path.join(root, path)
        r = subprocess.run(["git", "-C", sub, "archive", "--format=tar", commit], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if r.returncode != 0:
            raise OSError("commit %s of submodule %s is not available locally (git archive: %s)" % (
                commit[:12], path, " ".join(r.stderr.decode("utf-8", "replace").split())))
        dst = os.path.join(sroot, path)
        shutil.rmtree(dst, ignore_errors=True)
        os.makedirs(dst)
        _extract_tar(r.stdout, dst)


def resolve_head(root):
    """The commit id of HEAD ('' when git fails)."""
    r = subprocess.run(["git", "-C", root, "rev-parse", "HEAD"], capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else ""


def make_isolated_tree(root, wo, changed, head=None):
    """Deletes and re-creates the isolated source tree of `wo`: `git archive <head>` (default: HEAD resolved now;
    read-only on the shared repository), the submodules below Firmware/ (git archive leaves them empty) from their
    recorded commits, plus the WO's changed / untracked files copied from the working tree (a deleted file is removed).
    Returns (source root, {rel: git blob of the file in the isolated tree})."""
    head = head or resolve_head(root)
    if not head:
        raise OSError("cannot resolve HEAD")
    base, sroot = iso_paths(root, wo)
    shutil.rmtree(base, ignore_errors=True)
    os.makedirs(sroot)
    r = subprocess.run(["git", "-C", root, "archive", "--format=tar", head], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if r.returncode != 0:
        raise OSError("git archive failed: " + " ".join(r.stderr.decode("utf-8", "replace").split()))
    _extract_tar(r.stdout, sroot)
    copy_submodules(root, sroot)
    blobs = {}
    for rel in changed:
        src, dst = os.path.join(root, rel), os.path.join(sroot, rel)
        if os.path.isfile(src):
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(src, dst)
        elif os.path.lexists(dst):
            os.remove(dst)
        blobs[rel] = guard_lib.blob_of_file(sroot, rel)
    return sroot, blobs


DOCS_DIR = "Tests/Integration_Tests"


def changed_docs(root, status_raw=None):
    """Repo-relative paths under Tests/Integration_Tests/ that are changed, untracked or deleted in the working tree,
    without codex/ paths and AGENTS.md."""
    if status_raw is None:
        r = subprocess.run(["git", "-C", root, "status", "--porcelain=v1", "-z", "-uall", "--no-renames", "--", DOCS_DIR],
                           capture_output=True, text=True)
        if r.returncode != 0:
            raise OSError("git status failed: " + " ".join(r.stderr.split()))
        status_raw = r.stdout
    out = []
    for _, rel in parse_porcelain(status_raw):
        parts = rel.split("/")
        if not rel.startswith(DOCS_DIR + "/") or "codex" in parts or rel == "AGENTS.md":
            continue
        out.append(rel)
    return sorted(out)


def overlay_docs(root, sroot, status_raw=None):
    """Copies the changed documents under Tests/Integration_Tests/ into the isolated tree (deleted ones are removed).
    Returns {rel: git blob of the working-tree file (None = deleted)}."""
    blobs = {}
    for rel in changed_docs(root, status_raw):
        src, dst = os.path.join(root, rel), os.path.join(sroot, rel)
        if os.path.isfile(src):
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(src, dst)
        elif os.path.lexists(dst):
            os.remove(dst)
        blobs[rel] = guard_lib.blob_of_file(root, rel)
    return blobs


def _fail(res, reason):
    res["ok"] = False
    res["reason"] = reason
    return res


def run_variant(root, variant, patterns, evid_dir, runner=run, own_wo=None, src_root=None, iso=False,
                serial=False, jobs=MAX_JOBS, budget=None, scope_files=None, echo=print, build_jobs=None):
    """Configure, build, test one variant. `runner(cmd, log_path, cwd=None, timeout=None) -> (rc, text)` runs every
    subprocess (injectable for the unit tests). `own_wo` = WO id of --own-build. Returns a result dict with `ok` and,
    when not ok, `reason`.

    serial=True: the sequence of the old gate (build with -j<cpus>, the whole ctest including ffb_integration_tests, then
    ffb_integration_tests again for its XML report); in both modes <variant>.integration.cases.json is written.
    serial=False: every subprocess holds its weight in `budget` (default: a new Budget(jobs)): the build -j<build_jobs>
    (default all of jobs), ctest without the integration entry and check_baseline one process each, beside the shards of
    the single integration run (run_integration; `scope_files` = case-file patterns, None = every case; the cov variant is
    not sharded). `echo` receives the notes of the run (the caller keeps the output of concurrent variants together)."""
    bd = build_dir_for(root, variant, own_wo, iso)
    sroot = src_root or root  # repository root or isolated source tree
    os.makedirs(os.path.dirname(bd), exist_ok=True)
    res = {"build_dir": bd}
    budget = budget or Budget(jobs)

    def job(cmd, log, weight=1, cwd=None, timeout=None, on_done=None, env=None):
        """Runs one subprocess holding `weight` of the budget; on_done(seconds) gets the time of the runner call only
        (measured inside the hold: the wait for a free slot is not counted)."""
        kw = {}
        if cwd is not None:
            kw["cwd"] = cwd
        if timeout is not None:
            kw["timeout"] = timeout
        if env:
            kw["env"] = env
        if serial:
            return runner(cmd, log, **kw)
        with budget.hold(weight):
            t = time.monotonic()
            try:
                return runner(cmd, log, **kw)
            finally:
                if on_done:
                    on_done(time.monotonic() - t)

    def logs(n):
        return os.path.join(evid_dir, "%s.%s.log" % (variant, n))
    src = os.path.join(sroot, SOURCE_DIR)
    cache_file = os.path.join(bd, "CMakeCache.txt")

    # a build folder configured for another source tree (another checkout or an isolated tree) is not reused
    home = read_cache(bd).get("CMAKE_HOME_DIRECTORY") if os.path.isfile(cache_file) else None
    if home and os.path.realpath(home) != os.path.realpath(src):
        note = "build directory %s was configured for %s, not %s: deleted, configuring fresh" % (bd, home, src)
        echo("   " + note)
        res["fresh_configure"] = note
        shutil.rmtree(bd, ignore_errors=True)

    # (3) an existing build directory must be the variant it is used as; it is re-configured every time so the
    # configure warnings are always in the log (the -D options are only given to a new directory).
    cfg = ["cmake", "-S", src, "-B", bd]
    if os.path.isfile(cache_file):
        bad = check_cache(read_cache(bd), variant)
        if bad:
            return _fail(res, "stale build directory %s: %s" % (bd, "; ".join(bad)))
    else:
        cfg += VARIANT_ARGS[variant]
    rc, warn_text = job(cfg, logs("configure"))
    res["configure"] = {"rc": rc}
    if rc != 0:
        return _fail(res, "configure failed (%s)" % logs("configure"))
    bad = check_cache(read_cache(bd), variant)
    if bad:
        return _fail(res, "build directory %s does not match the variant: %s" % (bd, "; ".join(bad)))

    rc, text = job(["ctest", "--test-dir", bd, "-N"], logs("ctest_list"))
    missing = [t for t in REQUIRED_TESTS if t not in registered_tests(text)]
    if rc != 0 or missing:
        return _fail(res, "ctest does not register %s in this configuration (%s)" % (", ".join(missing) or "any test", logs("ctest_list")))
    rc, text = job(["cmake", "--build", bd, "--target", "help"], logs("targets"))
    if rc != 0 or not has_build_target(text, EXE):
        return _fail(res, "the build target %s does not exist in this configuration (%s)" % (EXE, logs("targets")))

    bj = (os.cpu_count() or 2) if serial else max(1, min(build_jobs or jobs, jobs))
    rc, text = job(["cmake", "--build", bd, "-j%d" % bj], logs("build"), weight=bj)
    warn_text += "\n" + text
    res["build"] = {"rc": rc}
    if rc != 0:
        res["build"]["warnings_in_wo_files"] = []
        return _fail(res, "build failed (%s)" % logs("build"))
    jobs_list = []
    try:
        jobs_list = syntax_jobs(read_compile_commands(bd), sroot, patterns)
    except (OSError, ValueError):
        pass
    syn_text, syn_failed = syntax_recheck(jobs_list, job, None if serial else budget.total)
    warnings = attribute_warnings(warn_text + "\n" + syn_text, sroot, patterns, bd)
    res["build"]["warnings_in_wo_files"] = sorted({"%s:%d: %s" % w for w in warnings})[:20]
    res["build"]["syntax_rechecked_sources"] = len(jobs_list)
    if syn_failed:
        return _fail(res, "%d WO source(s) failed the -fsyntax-only re-check" % syn_failed)
    if warnings:
        return _fail(res, "%d warning(s) in WO files, first: %s:%d: %s" % ((len(warnings),) + warnings[0]))

    exe = os.path.join(bd, EXE + (".exe" if os.name == "nt" else ""))
    excl = integration_excludes(read_cache(bd))
    limit_s = case_timeout_s(variant)
    limit_env = {CASE_TIMEOUT_ENV: str(limit_s)}  # the per-case limit of the executable (no limit for any whole run)

    def run_ctest():
        """(res["ctest"], res["check_baseline"], [problem]) : ctest (without the integration entry unless serial), check_baseline."""
        problems = []
        cmd = ["ctest", "--test-dir", bd, "--output-on-failure", "--timeout", str(CTEST_TIMEOUT)]
        if not serial:
            # the one integration run is made by run_integration; one entry at a time, so ctest is one job of the budget
            cmd += ["-E", "^%s$" % EXE, "--parallel", "1"]
        # serial only: no 3600 s total cap on this ctest, because it also runs the integration entry (a whole-run limit is not wanted); the other entries lose the cap too
        rc, text = (job(cmd, logs("ctest"), env=limit_env) if serial else job(cmd, logs("ctest"), timeout=CTEST_TOTAL_TIMEOUT))
        cc = parse_ctest_counts(text)
        r_ctest = {"rc": rc, "failed": cc[0] if cc else None, "total": cc[1] if cc else None}
        if rc == 124:
            problems.append("ctest timed out after %d s (%s)" % (CTEST_TOTAL_TIMEOUT, logs("ctest")))
        elif "***Timeout" in text:
            problems.append("a ctest entry exceeded its %d s timeout (%s)" % (CTEST_TIMEOUT, logs("ctest")))
        elif rc != 0 or cc is None or cc[0] != 0 or cc[1] == 0:
            problems.append("ctest failed (%s)" % logs("ctest"))
        rc_b, text = job([sys.executable, "-B", os.path.join(sroot, "Firmware/Tests/Code_Coverage/tools/check_baseline.py"),
                          "--build-dir", bd], logs("check_baseline"), timeout=BASELINE_TIMEOUT)
        r_base = {"rc": rc_b, "last": (text.strip().splitlines() or [""])[-1]}
        if rc_b == 124:
            problems.append("check_baseline timed out after %d s" % BASELINE_TIMEOUT)
        elif rc_b != 0:
            problems.append("check_baseline failed (%s)" % logs("check_baseline"))
        return r_ctest, r_base, problems

    def run_integration_serial():
        """The old second run of the executable with its XML summary (and the per-case file). (res["integration"], [problem])."""
        problems = []
        cmd = [exe, "-r=xml"] + (["--test-case-exclude=" + ",".join(excl)] if excl else [])
        rc_i, xml_text = job(cmd, logs("integration.xml"), env=limit_env)
        rc_s, summary = job([sys.executable, "-B", os.path.join(sroot, "Firmware/Tests/Code_Coverage/tools/summarise_results.py"),
                             "--format", "xml", logs("integration.xml")], logs("integration.summary"))
        counts = parse_counts(summary)
        r = {"exe_rc": rc_i, "summary_rc": rc_s,
             "cases": counts[0] if counts else None, "passed": counts[1] if counts else None,
             "failed": counts[2] if counts else None, "skipped": counts[3] if counts else None,
             "known_issue": counts[4] if counts else None, "scope": "full", "mode": "serial",
             "case_timeout_s": limit_s, "case_timeouts": []}
        hung = timed_out_case(rc_i, xml_text)
        if hung is not None:
            r["case_timeouts"] = [hung]
        cases = parse_cases_xml(xml_text or "")
        if cases is not None:
            write_cases_file(os.path.join(evid_dir, "%s.integration.cases.json" % variant), variant, None, cases,
                             {"rerun_failed": [], "rerun_passed": [], "rerun_unrun_passed": []})
            r["cases_file"] = "%s.integration.cases.json" % variant
        if hung is not None:
            problems.append("%s: case timed out: %s" % (variant, hung))
        elif rc_i != 0 or rc_s != 0 or counts is None or counts[2] != 0:
            problems.append("%s failed (%s)" % (EXE, logs("integration.summary")))
        elif counts[0] == 0:
            problems.append("%s ran zero test cases" % EXE)
        return r, problems

    def run_integration_parallel():
        t_start = time.monotonic()
        r, problems = run_integration_parallel_inner()
        r["wall_s"] = round(time.monotonic() - t_start, 1)  # the integration phase of this variant (information, not a limit)
        return r, problems

    def run_integration_parallel_inner():
        problems = []
        rc_j, jtext = job(["ctest", "--test-dir", bd, "--show-only=json-v1", "-R", "^%s$" % EXE], logs("ctest_show"))
        wd, mismatch = ctest_integration_mismatch(jtext, exe, excl) if rc_j == 0 else (None, ["ctest --show-only=json-v1 failed (rc %s)" % rc_j])
        if mismatch:
            return ({"exe_rc": None, "mode": "parallel", "note": "ctest registers another integration command"},
                    ["the integration command that ctest registers differs from the one the gate runs: %s (%s)" % (
                        "; ".join(mismatch), logs("ctest_show"))])
        o = run_integration(job, exe, excl, scope_files, logs, budget.total, unsharded=("cov" in variant.split("-")),
                            cwd=wd, env=limit_env, label=variant)
        cases = o["cases"]
        counts = case_counts(cases) if cases else None
        r = {"exe_rc": o["exe_rc"], "summary_rc": 0,
             "cases": counts[0] if counts else None, "passed": counts[1] if counts else None,
             "failed": counts[2] if counts else None, "skipped": counts[3] if counts else None,
             "known_issue": counts[4] if counts else None,
             "selected": o["selected"], "scope": "full" if not scope_files else list(scope_files), "mode": "parallel",
             "shards": o["shards"],
             "rerun_failed": o["rerun_failed"], "rerun_passed": o["rerun_passed"], "rerun_unrun_passed": o["rerun_unrun_passed"],
             "dead_shards": o["dead_shards"], "cov_alone": o["cov_alone"], "wall_sum_s": round(o["wall_sum"], 3),
             "rerun_reasons": o["rerun_reasons"], "case_timeout_s": limit_s, "case_timeouts": o["case_timeouts"]}
        if cases:
            write_cases_file(os.path.join(evid_dir, "%s.integration.cases.json" % variant), variant, scope_files, cases,
                             {"rerun_failed": o["rerun_failed"], "rerun_passed": o["rerun_passed"],
                              "rerun_unrun_passed": o["rerun_unrun_passed"]})
            r["cases_file"] = "%s.integration.cases.json" % variant
            with open(logs("integration.summary"), "w", encoding="utf-8") as f:
                f.write(summary_text(cases))
        problems += o["problems"]
        for d in o["dead_shards"]:
            echo("WARNING: dead shard %d (cases %d-%d, rc %s, last started: %s, %s): %s" % (
                d["shard"], d["first"], d["last"], d["rc"], d["last_started"] or "unknown", d["log"], d["reason"]))
        again = o["rerun_passed"] + o["rerun_unrun_passed"]
        if again:
            echo("WARNING: %d case(s) passed only when rerun alone: %s (variant %s)" % (len(again), "; ".join(again), variant))
        return r, problems

    problems = []
    if serial:
        res["ctest"], res["check_baseline"], p = run_ctest()
        problems += p
        if os.path.isfile(exe):
            res["integration"], p = run_integration_serial()
            problems += p
        else:
            res["integration"] = {"exe_rc": None, "note": "%s not found in the build directory" % EXE}
            problems.append("%s not found in %s" % (EXE, bd))
    else:
        if os.path.isfile(exe):
            with ThreadPoolExecutor(max_workers=2) as ex:
                f_ctest = ex.submit(run_ctest)
                f_int = ex.submit(run_integration_parallel)
                res["ctest"], res["check_baseline"], p_ctest = f_ctest.result()
                res["integration"], p_int = f_int.result()
            problems += p_ctest + p_int
        else:
            res["ctest"], res["check_baseline"], p_ctest = run_ctest()
            problems += p_ctest
            res["integration"] = {"exe_rc": None, "note": "%s not found in the build directory" % EXE}
            problems.append("%s not found in %s" % (EXE, bd))

    if os.path.isfile(os.path.join(bd, "compile_commands.json")):
        res["compile_check"] = check_compile_reference(root, bd, variant, sroot)
    else:
        res["compile_check"] = {"ok": False, "message": "compile_commands.json missing in " + bd}
    if not res["compile_check"]["ok"]:
        problems.append(res["compile_check"]["message"])
    res["ok"] = not problems
    if problems:
        res["reason"] = "; ".join(problems)
    return res


def variant_summary(variant, r):
    parts = ["%s: %s" % (variant, "OK" if r.get("ok") else "FAIL")]
    if "build" in r:
        parts.append("build rc=%s warnings=%d (%s sources re-checked)" % (
            r["build"]["rc"], len(r["build"]["warnings_in_wo_files"]), r["build"].get("syntax_rechecked_sources", "-")))
    if "ctest" in r:
        parts.append("ctest rc=%s (%s failed of %s)" % (r["ctest"]["rc"], r["ctest"]["failed"], r["ctest"]["total"]))
    if "check_baseline" in r:
        parts.append("baseline rc=%s" % r["check_baseline"]["rc"])
    if "integration" in r and r["integration"].get("cases") is not None:
        i = r["integration"]
        parts.append("integration %s cases, %s passed, %s failed, %s skipped, %s known issue" % (
            i["cases"], i["passed"], i["failed"], i["skipped"], i["known_issue"]))
        if "mode" in i:
            sc = i.get("scope", "full")
            extra = ["scope " + ("full" if sc == "full" else "%d case file(s)" % len(sc)), i["mode"]]
            if i.get("case_timeout_s") is not None:
                extra.append("case limit %s s" % i["case_timeout_s"])
            if i.get("wall_s") is not None:
                extra.append("%s s wall" % i["wall_s"])
            if i.get("case_timeouts"):
                extra.append("%d case timeout(s)" % len(i["case_timeouts"]))
            if i.get("shards"):
                extra.append("%d shards" % i["shards"])
            if i.get("dead_shards"):
                extra.append("%d dead shard(s)" % len(i["dead_shards"]))
            if i.get("rerun_failed") or i.get("rerun_passed") or i.get("rerun_unrun_passed"):
                extra.append("reruns: %d failed again, %d passed alone, %d unrun passed alone" % (
                    len(i.get("rerun_failed", [])), len(i.get("rerun_passed", [])), len(i.get("rerun_unrun_passed", []))))
            parts[-1] += " [%s]" % ", ".join(extra)
    if "compile_check" in r:
        parts.append("compile check: %s" % r["compile_check"]["message"])
    if not r.get("ok") and r.get("reason"):
        parts.append(r["reason"])
    return "; ".join(parts)


def git_head(root):
    r = subprocess.run(["git", "-C", root, "rev-parse", "HEAD"], capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else ""


def write_evidence(root, wo, evidence, now=None):
    d = os.path.join(guard_lib.gate_dir(root), wo)
    os.makedirs(d, exist_ok=True)
    stamp = (now or datetime.datetime.now(datetime.timezone.utc)).strftime("%Y%m%dT%H%M%SZ")
    path = os.path.join(d, stamp + ".json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(evidence, f, indent=1, sort_keys=True)
    shutil.copyfile(path, os.path.join(d, "latest.json"))
    return path


def run_variants_parallel(root, names, patterns, evid_dir, runner, own_wo, src_root, iso, jobs, scope_files):
    """All variants at once, under one job budget of `jobs` (compile jobs plus test processes): the builds share it
    (-j jobs // number of concurrent variants), the shards and the other ctest entries take what is free. The output of
    each variant is printed as one block when it is done. Returns {variant: result}."""
    budget = Budget(jobs)
    workers = max(1, min(len(names), jobs))
    share = max(1, jobs // workers)
    lock = threading.Lock()
    print("== %d variant(s) in parallel (%s), at most %d jobs at once" % (len(names), ", ".join(names), jobs), flush=True)

    def one(v):
        lines = ["== variant %s (%s)" % (v, build_dir_for(root, v, own_wo, iso))]
        try:
            if iso:
                shutil.rmtree(build_dir_for(root, v, own_wo, True), ignore_errors=True)
            r = run_variant(root, v, patterns, evid_dir, runner, own_wo, src_root, iso, serial=False, jobs=jobs, budget=budget,
                            scope_files=scope_files, echo=lines.append, build_jobs=share)
        except Exception as e:  # noqa: BLE001 - one variant's error must not hide the others
            r = {"ok": False, "reason": "internal error: %s: %s" % (type(e).__name__, e)}
        lines.append("   " + variant_summary(v, r))
        with lock:
            print("\n".join(lines), flush=True)
        return v, r

    with ThreadPoolExecutor(max_workers=workers) as ex:
        done = dict(ex.map(one, names))
    return {v: done[v] for v in names}


def decide(names, full, ok):
    """(result, covers_required): PASS needs everything ok and default and m32 (or --full) in the run; a reduced run
    that passed is PARTIAL."""
    covers = full or all(v in names for v in REQUIRED_VARIANTS)
    if not ok:
        return "FAIL", covers
    return ("PASS" if covers else "PARTIAL"), covers


def main(argv=None, runner=run):
    """Runs the gate; always ends with a GATE PASS / GATE FAIL line (an unexpected error is a GATE FAIL)."""
    try:
        return _main(argv, runner)
    except SystemExit as e:
        if e.code in (0, None):
            return 0
        print("GATE FAIL bad command line")
        return 1
    except Exception as e:  # noqa: BLE001 - the gate must always say GATE FAIL
        print("GATE FAIL internal error: %s: %s" % (type(e).__name__, e))
        return 1


def _main(argv, runner):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog="Time limits: none for a whole integration run (no process or sum limit, owner's decision 2026-11-24). "
                                        "Every test case of ffb_integration_tests has its own limit, enforced inside the executable (support/CaseWatchdog): "
                                        "environment variable %s = whole seconds, 0 = off, unset = the compiled default (%d s; derivation in "
                                        "support/CaseWatchdog.h). The gate sets it per variant (%d s here), a case that exceeds it ends its process "
                                        "with exit code %d and the line `CASE TIMEOUT <s> s: <case> (<file>:<line>)`; the gate runs that case alone once "
                                        "(failure `<variant>: case timed out: <case>` if it times out again, `rerun_passed` otherwise; cov: failure)."
                                        % (CASE_TIMEOUT_ENV, DEFAULT_CASE_TIMEOUT_S, case_timeout_s("default"), CASE_TIMEOUT_RC))
    ap.add_argument("--wo", help="work order id (e.g. I0-4)")
    ap.add_argument("--variants", default="default,m32", help="comma separated: %s" % ", ".join(VARIANT_ARGS))
    ap.add_argument("--full", action="store_true", help="also run the variants: %s" % ", ".join(FULL_VARIANTS))
    ap.add_argument("--own-build", action="store_true",
                    help="build in ../ffb-test-build/it-<WO>-<variant> instead of the shared it-<variant> (parallel writers)")
    ap.add_argument("--isolated", action="store_true",
                    help="build HEAD plus only this WO's changed files in ../ffb-test-build/it-iso-<WO>/src (final gate before a commit)")
    ap.add_argument("--files", help="WO file list (default: Tests/Integration_Tests/claude/charters/wo/<ID>.files)")
    ap.add_argument("--record-compile-reference", metavar="BUILD_DIR",
                    help="record the compile commands of BUILD_DIR as the reference of --variant and exit")
    ap.add_argument("--variant", help="variant name for --record-compile-reference")
    ap.add_argument("--serial", action="store_true",
                    help="the old sequence: variants one after another, the whole ctest (with ffb_integration_tests), then the integration executable again for its XML report")
    ap.add_argument("--jobs", type=int, default=MAX_JOBS,
                    help="most compile jobs plus test processes at once, in total (default and maximum %d)" % MAX_JOBS)
    ap.add_argument("--root", default=DEFAULT_ROOT, help=argparse.SUPPRESS)
    args = ap.parse_args(argv)
    root = os.path.realpath(args.root)

    if not 1 <= args.jobs <= MAX_JOBS:
        print("GATE FAIL --jobs must be between 1 and %d (the owner's limit for compile jobs plus test processes), got %d" % (MAX_JOBS, args.jobs))
        return 1
    if args.record_compile_reference:
        if not args.variant or args.variant not in VARIANT_ARGS:
            print("GATE FAIL --record-compile-reference needs --variant (%s)" % ", ".join(VARIANT_ARGS))
            return 1
        try:
            path, n = record_reference(root, os.path.realpath(args.record_compile_reference), args.variant)
        except (OSError, ValueError) as e:
            print("GATE FAIL cannot record the compile reference: %s" % e)
            return 1
        print("recorded %d compile commands (without %s) to %s" % (n, EXE, path))
        return 0

    if not args.wo:
        print("GATE FAIL --wo <ID> is required")
        return 1
    files_path = args.files or os.path.join(root, "Tests/Integration_Tests/claude/charters/wo/%s.files" % args.wo)
    try:
        with open(files_path, encoding="utf-8") as f:
            patterns = parse_file_list(f.read())
    except FileNotFoundError:
        print("GATE FAIL WO file list %s is missing" % files_path)
        return 1
    except (OSError, ValueError) as e:
        print("GATE FAIL WO file list %s cannot be read: %s" % (files_path, e))
        return 1
    if not patterns:
        print("GATE FAIL WO file list %s is empty" % files_path)
        return 1
    try:
        names = variant_list(args.variants, args.full)
    except ValueError as e:
        print("GATE FAIL %s" % e)
        return 1

    now = datetime.datetime.now(datetime.timezone.utc)
    evid_dir = os.path.join(guard_lib.gate_dir(root), args.wo, "logs-" + now.strftime("%Y%m%dT%H%M%SZ"))
    os.makedirs(evid_dir, exist_ok=True)
    active, active_errors = load_active(root, args.wo)
    variants, failures = {}, []
    sroot, overlay, docs, head0 = root, None, None, None
    scope = None
    if args.isolated:
        head0 = resolve_head(root)  # resolved once: the archive and the evidence use this commit id
        scope = scope_check(root, patterns, active=active, active_errors=active_errors, wo=args.wo)
        if scope["result"] == "OK":
            try:
                sroot, overlay = make_isolated_tree(root, args.wo, scope["changed"], head0)
                docs = overlay_docs(root, sroot)
            except (OSError, tarfile.TarError) as e:
                failures.append("cannot build the isolated source tree: %s" % e)
            else:
                print("== isolated source tree %s (%s + %d file(s) of %s + %d document(s) of Tests/Integration_Tests)" % (
                    sroot, head0[:12], len(overlay), args.wo, len(docs)), flush=True)
    own_wo = args.wo if (args.own_build or args.isolated) else None
    scope_files = None
    if args.isolated and not args.serial and not args.full and scope and scope["result"] == "OK":
        scope_files = gate_scope(patterns, scope["changed"], False, True)
    if not failures and not (scope and scope["result"] != "OK"):
        if args.serial:
            for v in names:
                if args.isolated:
                    shutil.rmtree(build_dir_for(root, v, own_wo, True), ignore_errors=True)
                print("== variant %s (%s)" % (v, build_dir_for(root, v, own_wo, args.isolated)), flush=True)
                variants[v] = run_variant(root, v, patterns, evid_dir, runner, own_wo, sroot if args.isolated else None, args.isolated,
                                          serial=True)
                print("   " + variant_summary(v, variants[v]), flush=True)
        else:
            variants.update(run_variants_parallel(root, names, patterns, evid_dir, runner, own_wo,
                                                  sroot if args.isolated else None, args.isolated, args.jobs, scope_files))

    scope = scope_check(root, patterns, active=active, active_errors=active_errors, wo=args.wo)
    print("== scope: %s%s" % (scope["result"], "".join("\n   " + o for o in scope["offenders"])))
    if scope["other_active"]:
        print("   other active WOs' changed files (not checked here): %d" % len(scope["other_active"]))
        for o in scope["other_active"]:
            print("   other_active: " + o)
    if overlay is not None:
        now_blobs = {rel: guard_lib.blob_of_file(root, rel) for rel in scope["changed"]}
        if now_blobs != overlay:
            failures.append("the files of %s changed while the isolated gate ran (overlay != working tree)" % args.wo)
        try:
            docs_now = {rel: guard_lib.blob_of_file(root, rel) for rel in changed_docs(root)}
        except OSError as e:
            docs_now = None
            failures.append("cannot re-read the documents: %s" % e)
        if docs_now is not None and docs_now != docs:
            failures.append("the documents under Tests/Integration_Tests/ changed while the isolated gate ran")
    if args.isolated:
        head_end = resolve_head(root)
        if head_end != head0:
            failures.append("HEAD moved during the gate (%s -> %s)" % (head0[:12], head_end[:12]))
    failures += ["%s: %s" % (v, r.get("reason", "failed")) for v, r in variants.items() if not r.get("ok")]
    if scope["result"] != "OK":
        failures.append("scope check failed (%s)" % scope["offenders"][0])
    result, covers = decide(names, args.full, not failures)
    evidence = make_evidence(root, args.wo, variants, scope, head0 if args.isolated else git_head(root), result, now,
                             requested=names, files_path=files_path, patterns=patterns, own_build=args.own_build,
                             isolated=args.isolated, source_root=sroot, docs_overlaid=docs, integration_scope=scope_files,
                             serial=args.serial, jobs=None if args.serial else args.jobs)
    path = write_evidence(root, args.wo, evidence, now)
    warning = aggregate_warning(variants)
    if warning:
        print(warning)
    if result == "PASS":
        print(final_line(True, path))
        return 0
    if result == "PARTIAL":
        print("GATE FAIL partial run (variants: %s); commit evidence needs %s" % (",".join(names), ",".join(REQUIRED_VARIANTS)))
        return 1
    print(final_line(False, path, "; ".join(failures)))
    return 1


if __name__ == "__main__":
    sys.exit(main())
