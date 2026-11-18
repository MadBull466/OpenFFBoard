#!/usr/bin/env python3
"""Gate of the OpenFFBoard integration track (Tests/Integration_Tests/claude/plans/AGENT_PLAN.md sections 2.3 and 2.4).

  python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --wo <ID> [--variants default,m32] [--full] [--files <path>]
  python3 -B Firmware/Tests/Integration_Tests/tools/it_gate.py --record-compile-reference <build dir> --variant <name>

Per variant (build directory ../ffb-test-build/it-<variant>, outside the repository):
  1. configure (if there is no CMakeCache.txt) and build; a compiler or CMake warning in a file of the WO's file list fails;
  2. ctest --output-on-failure;
  3. Code_Coverage/tools/check_baseline.py --build-dir <dir>;
  4. ffb_integration_tests -r=xml, summarised by Code_Coverage/tools/summarise_results.py;
  5. the compile commands of every target other than ffb_integration_tests equal the recorded reference
     (../ffb-test-build/it-gate/compile_reference-<variant>.json).
After all variants: scope check (git status of Firmware/ against the WO file list and the protected paths of the hooks).
Evidence: ../ffb-test-build/it-gate/<WO>/<UTC timestamp>.json and latest.json, in the format of .claude/hooks/guard_lib.py
(`files` = git blob of every changed or untracked file under Firmware/ that is in the WO list, `fingerprint`).
The last line printed is `GATE PASS <evidence path>` or `GATE FAIL <reason>`; exit status 0 / 1.

Python standard library only (plus .claude/hooks/guard_lib.py of this repository).
"""

import argparse
import datetime
import fnmatch
import json
import os
import re
import shlex
import shutil
import subprocess
import sys

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
INTEGRATION_TIMEOUT = 600
SOURCE_SUFFIXES = (".c", ".cc", ".cpp", ".cxx")
SOURCE_DIR = "Firmware/Tests"
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


def build_dir_for(root, variant):
    """../ffb-test-build/it-<variant>, resolved from the repository root."""
    return os.path.realpath(os.path.join(root, "..", "ffb-test-build", "it-" + variant))


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


def normalise_entries(entries, build_dir):
    """{ "<output>|<file>": command } of the entries of all targets except ffb_integration_tests; the absolute build
    directory is replaced by @BUILD@ so two build directories of the same variant compare equal."""
    bd = os.path.realpath(build_dir).replace("\\", "/").rstrip("/")
    out = {}
    for e in entries:
        if is_integration_entry(e):
            continue

        def norm(s):
            return s.replace("\\", "/").replace(bd, "@BUILD@") if isinstance(s, str) else s
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


def record_reference(root, build_dir, variant):
    entries = normalise_entries(read_compile_commands(build_dir), build_dir)
    path = reference_path(root, variant)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"format": REF_FORMAT, "variant": variant, "source_build_dir": os.path.realpath(build_dir), "entries": entries},
                  f, indent=1, sort_keys=True)
    return path, len(entries)


def check_compile_reference(root, build_dir, variant):
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
        ref = data["entries"]
        cur = normalise_entries(read_compile_commands(build_dir), build_dir)
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


def scope_check(root, patterns, block_reason=None, status_raw=None):
    """Scope of the working tree below Firmware/ against the WO file list.

    Returns {"result": "OK"|"FAIL", "offenders": [str], "changed": [rel paths in the list]}.
      - ignored paths outside Firmware/Tests/, and any path outside Firmware/Tests/: offender
      - a changed or untracked path under Firmware/Tests/ that is not in the list: offender
      - a changed path in the list that the hooks protect (guard_lib.path_block_reason, i.e. not approved): offender"""
    if block_reason is None:
        def block_reason(rel):
            return guard_lib.path_block_reason(rel, root)
    if status_raw is None:
        r = subprocess.run(["git", "-C", root, "status", "--porcelain=v1", "-z", "-uall", "--ignored", "--no-renames", "--", "Firmware"],
                           capture_output=True, text=True)
        if r.returncode != 0:
            return {"result": "FAIL", "offenders": ["git status failed: " + r.stderr.strip()], "changed": []}
        status_raw = r.stdout
    offenders, changed = [], []
    for status, rel in parse_porcelain(status_raw):
        ignored = status == "!!"
        if not rel.startswith("Firmware/"):
            continue
        if not rel.startswith("Firmware/Tests/"):
            offenders.append("%s: outside Firmware/Tests/ (%s)" % (rel, "ignored file" if ignored else "changed"))
            continue
        if ignored:
            continue  # ignored files below Firmware/Tests/ (build leftovers) are not part of the change
        if not matches(rel, patterns):
            offenders.append("%s: not in the WO file list" % rel)
            continue
        reason = block_reason(rel)
        if reason:
            offenders.append("%s: protected (%s)" % (rel, reason))
            continue
        changed.append(rel)
    return {"result": "FAIL" if offenders else "OK", "offenders": offenders, "changed": sorted(changed)}


def make_evidence(root, wo, variants, scope, git_head, result, now=None, requested=None, files_path=None, patterns=None):
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
    }


def final_line(ok, evidence_path=None, reason=""):
    return ("GATE PASS %s" % evidence_path) if ok else ("GATE FAIL %s" % (reason or "see the evidence"))


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


def syntax_recheck(jobs, runner):
    """Runs every job; returns (combined output, number of failed jobs). An incremental build only recompiles what
    changed, so a warning of an unchanged WO source (or of a WO header seen through another source) would be missed;
    this pass compiles each listed source again, without output. A warning that is only in a WO header and appears
    only when a source that includes it is rebuilt can still be missed here when no listed source includes it."""
    if not jobs:
        return "", 0
    from concurrent.futures import ThreadPoolExecutor
    with ThreadPoolExecutor(max_workers=max(1, os.cpu_count() or 2)) as ex:
        results = list(ex.map(lambda j: runner(j[1], None, cwd=j[0], timeout=BASELINE_TIMEOUT), jobs))
    return "\n".join(t for _, t in results), sum(1 for rc, _ in results if rc != 0)


def run(cmd, log_path, cwd=None, timeout=None):
    """Runs cmd, writes stdout+stderr to log_path (unless None), returns (returncode, text). rc 124 = timeout."""
    try:
        p = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                           errors="replace", timeout=timeout)
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


def _fail(res, reason):
    res["ok"] = False
    res["reason"] = reason
    return res


def run_variant(root, variant, patterns, evid_dir, runner=run):
    """Configure, build, test one variant. `runner(cmd, log_path, cwd=None, timeout=None) -> (rc, text)` runs every
    subprocess (injectable for the unit tests). Returns a result dict with `ok` and, when not ok, `reason`."""
    bd = build_dir_for(root, variant)
    os.makedirs(os.path.dirname(bd), exist_ok=True)
    res = {"build_dir": bd}

    def logs(n):
        return os.path.join(evid_dir, "%s.%s.log" % (variant, n))
    src = os.path.join(root, SOURCE_DIR)
    cache_file = os.path.join(bd, "CMakeCache.txt")

    # (3) an existing build directory must be the variant it is used as; it is re-configured every time so the
    # configure warnings are always in the log (the -D options are only given to a new directory).
    cfg = ["cmake", "-S", src, "-B", bd]
    if os.path.isfile(cache_file):
        bad = check_cache(read_cache(bd), variant)
        if bad:
            return _fail(res, "stale build directory %s: %s" % (bd, "; ".join(bad)))
    else:
        cfg += VARIANT_ARGS[variant]
    rc, warn_text = runner(cfg, logs("configure"))
    res["configure"] = {"rc": rc}
    if rc != 0:
        return _fail(res, "configure failed (%s)" % logs("configure"))
    bad = check_cache(read_cache(bd), variant)
    if bad:
        return _fail(res, "build directory %s does not match the variant: %s" % (bd, "; ".join(bad)))

    rc, text = runner(["ctest", "--test-dir", bd, "-N"], logs("ctest_list"))
    missing = [t for t in REQUIRED_TESTS if t not in registered_tests(text)]
    if rc != 0 or missing:
        return _fail(res, "ctest does not register %s in this configuration (%s)" % (", ".join(missing) or "any test", logs("ctest_list")))
    rc, text = runner(["cmake", "--build", bd, "--target", "help"], logs("targets"))
    if rc != 0 or not has_build_target(text, EXE):
        return _fail(res, "the build target %s does not exist in this configuration (%s)" % (EXE, logs("targets")))

    rc, text = runner(["cmake", "--build", bd, "-j%d" % (os.cpu_count() or 2)], logs("build"))
    warn_text += "\n" + text
    res["build"] = {"rc": rc}
    if rc != 0:
        res["build"]["warnings_in_wo_files"] = []
        return _fail(res, "build failed (%s)" % logs("build"))
    jobs = []
    try:
        jobs = syntax_jobs(read_compile_commands(bd), root, patterns)
    except (OSError, ValueError):
        pass
    syn_text, syn_failed = syntax_recheck(jobs, runner)
    warnings = attribute_warnings(warn_text + "\n" + syn_text, root, patterns, bd)
    res["build"]["warnings_in_wo_files"] = sorted({"%s:%d: %s" % w for w in warnings})[:20]
    res["build"]["syntax_rechecked_sources"] = len(jobs)
    if syn_failed:
        return _fail(res, "%d WO source(s) failed the -fsyntax-only re-check" % syn_failed)
    if warnings:
        return _fail(res, "%d warning(s) in WO files, first: %s:%d: %s" % ((len(warnings),) + warnings[0]))

    problems = []
    rc, text = runner(["ctest", "--test-dir", bd, "--output-on-failure", "--timeout", str(CTEST_TIMEOUT)],
                      logs("ctest"), timeout=CTEST_TOTAL_TIMEOUT)
    cc = parse_ctest_counts(text)
    res["ctest"] = {"rc": rc, "failed": cc[0] if cc else None, "total": cc[1] if cc else None}
    if rc == 124:
        problems.append("ctest timed out after %d s (%s)" % (CTEST_TOTAL_TIMEOUT, logs("ctest")))
    elif "***Timeout" in text:
        problems.append("a ctest entry exceeded its %d s timeout (%s)" % (CTEST_TIMEOUT, logs("ctest")))
    elif rc != 0 or cc is None or cc[0] != 0 or cc[1] == 0:
        problems.append("ctest failed (%s)" % logs("ctest"))

    rc_b, text = runner([sys.executable, "-B", os.path.join(root, "Firmware/Tests/Code_Coverage/tools/check_baseline.py"),
                         "--build-dir", bd], logs("check_baseline"), timeout=BASELINE_TIMEOUT)
    res["check_baseline"] = {"rc": rc_b, "last": (text.strip().splitlines() or [""])[-1]}
    if rc_b == 124:
        problems.append("check_baseline timed out after %d s" % BASELINE_TIMEOUT)
    elif rc_b != 0:
        problems.append("check_baseline failed (%s)" % logs("check_baseline"))

    exe = os.path.join(bd, EXE + (".exe" if os.name == "nt" else ""))
    if os.path.isfile(exe):
        excl = integration_excludes(read_cache(bd))
        cmd = [exe, "-r=xml"] + (["--test-case-exclude=" + ",".join(excl)] if excl else [])
        rc_i, _ = runner(cmd, logs("integration.xml"), timeout=INTEGRATION_TIMEOUT)
        rc_s, summary = runner([sys.executable, "-B", os.path.join(root, "Firmware/Tests/Code_Coverage/tools/summarise_results.py"),
                                "--format", "xml", logs("integration.xml")], logs("integration.summary"))
        counts = parse_counts(summary)
        res["integration"] = {"exe_rc": rc_i, "summary_rc": rc_s,
                              "cases": counts[0] if counts else None, "passed": counts[1] if counts else None,
                              "failed": counts[2] if counts else None, "skipped": counts[3] if counts else None,
                              "known_issue": counts[4] if counts else None}
        if rc_i == 124:
            problems.append("%s timed out after %d s" % (EXE, INTEGRATION_TIMEOUT))
        elif rc_i != 0 or rc_s != 0 or counts is None or counts[2] != 0:
            problems.append("%s failed (%s)" % (EXE, logs("integration.summary")))
        elif counts[0] == 0:
            problems.append("%s ran zero test cases" % EXE)
    else:
        res["integration"] = {"exe_rc": None, "note": "%s not found in the build directory" % EXE}
        problems.append("%s not found in %s" % (EXE, bd))

    if os.path.isfile(os.path.join(bd, "compile_commands.json")):
        res["compile_check"] = check_compile_reference(root, bd, variant)
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


def decide(names, full, ok):
    """(result, covers_required): PASS needs everything ok and default and m32 (or --full) in the run; a reduced run
    that passed is PARTIAL."""
    covers = full or all(v in names for v in REQUIRED_VARIANTS)
    if not ok:
        return "FAIL", covers
    return ("PASS" if covers else "PARTIAL"), covers


def main(argv=None, runner=run):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--wo", help="work order id (e.g. I0-4)")
    ap.add_argument("--variants", default="default,m32", help="comma separated: %s" % ", ".join(VARIANT_ARGS))
    ap.add_argument("--full", action="store_true", help="also run the variants: %s" % ", ".join(FULL_VARIANTS))
    ap.add_argument("--files", help="WO file list (default: Tests/Integration_Tests/claude/charters/wo/<ID>.files)")
    ap.add_argument("--record-compile-reference", metavar="BUILD_DIR",
                    help="record the compile commands of BUILD_DIR as the reference of --variant and exit")
    ap.add_argument("--variant", help="variant name for --record-compile-reference")
    ap.add_argument("--root", default=DEFAULT_ROOT, help=argparse.SUPPRESS)
    args = ap.parse_args(argv)
    root = os.path.realpath(args.root)

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
    except OSError:
        print("GATE FAIL WO file list %s is missing" % files_path)
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
    variants = {}
    for v in names:
        print("== variant %s (%s)" % (v, build_dir_for(root, v)), flush=True)
        variants[v] = run_variant(root, v, patterns, evid_dir, runner)
        print("   " + variant_summary(v, variants[v]), flush=True)

    scope = scope_check(root, patterns)
    print("== scope: %s%s" % (scope["result"], "".join("\n   " + o for o in scope["offenders"])))
    failures = ["%s: %s" % (v, r.get("reason", "failed")) for v, r in variants.items() if not r.get("ok")]
    if scope["result"] != "OK":
        failures.append("scope check failed (%s)" % scope["offenders"][0])
    result, covers = decide(names, args.full, not failures)
    evidence = make_evidence(root, args.wo, variants, scope, git_head(root), result, now,
                             requested=names, files_path=files_path, patterns=patterns)
    path = write_evidence(root, args.wo, evidence, now)
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
