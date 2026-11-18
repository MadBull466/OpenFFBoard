"""Shared rules of the integration-track hooks (Tests/Integration_Tests/claude/plans/AGENT_PLAN.md section 2).

Python standard library only. Every function takes the repository root explicitly so the unit tests
(Firmware/Tests/Integration_Tests/tools/test_hooks.py) can run them against a temporary repository.
"""

import fnmatch
import hashlib
import json
import os
import re
import shlex
import subprocess

DEFAULT_ROOT = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

APPROVED_FILE = "Tests/Integration_Tests/claude/plans/APPROVED_SHARED_EDITS.txt"
FORK_URL = "https://github.com/MadBull466/OpenFFBoard.git"
BRANCH = "add-test-suite"
COMMIT_NAME = "MadBull.Ben"

GENERATED = (
    "Tests/Code_Coverage/claude/reports/BUG_REPORT.md",
    "Tests/Integration_Tests/claude/reports/INTEGRATION_BUG_REPORT.md",
    "Firmware/Tests/Integration_Tests/claude/oracles/*.csv",
    "Firmware/Tests/Integration_Tests/claude/oracles/**/*.csv",
    "Firmware/Tests/Code_Coverage/baseline/test_cases.txt",
)

# Lead configuration: committed without gate evidence when the hook tests pass.
CONFIG_PATHS = (".claude/*", "CLAUDE.md", "Firmware/Tests/Integration_Tests/tools/test_hooks.py")
HOOK_TESTS = "Firmware/Tests/Integration_Tests/tools/test_hooks.py"

WRITER = "integration-test-writer"
ANALYST = "stability-analyst"


def gate_dir(root):
    return os.path.realpath(os.path.join(root, "..", "ffb-test-build", "it-gate"))


def _match(rel, pattern):
    if pattern.endswith("/*"):
        return rel.startswith(pattern[:-1])
    return fnmatch.fnmatchcase(rel, pattern)


def rel_path(path, root, cwd=None):
    """Repository-relative POSIX path of `path`, or None when it is outside the repository."""
    if not path:
        return None
    base = cwd or root
    full = os.path.realpath(os.path.join(base, os.path.expanduser(path)))
    root = os.path.realpath(root)
    if full == root:
        return "."
    if not full.startswith(root + os.sep):
        return None
    return os.path.relpath(full, root).replace(os.sep, "/")


def load_approved(root):
    """Paths listed in APPROVED_SHARED_EDITS.txt (`<path> | <approval date> | <WO>`; # comments)."""
    out = set()
    try:
        with open(os.path.join(root, APPROVED_FILE), encoding="utf-8") as f:
            for line in f:
                line = line.split("#", 1)[0].strip()
                if not line:
                    continue
                parts = [p.strip() for p in line.split("|")]
                if len(parts) >= 3 and parts[0] and parts[1] and parts[2]:
                    out.add(parts[0])
    except OSError:
        pass
    return out


def path_block_reason(rel, root, agent_type=None):
    """Reason why writing repository path `rel` is blocked for this caller, or None."""
    if rel is None:
        return None  # outside the repository (scratchpad, ../ffb-test-build, memory)
    parts = rel.split("/")
    approved = load_approved(root)
    if rel == ".git" or rel.startswith(".git/"):
        return "the .git directory is never written directly"
    if "codex" in parts:
        return "Codex's files are never edited (CLAUDE.md)"
    if rel.startswith(".github/"):
        return ".github/ is never edited; describe CI needs in PROGRESS.md and ask the owner"
    if rel.startswith("Tests/Code_Coverage/shared/specs/") or rel == "Tests/Code_Coverage/shared/specs":
        return "specification files are supplied by the owner and never written"
    if rel == "Firmware/Tests/Code_Coverage/third_party/doctest.h":
        return "doctest.h is never edited"
    for g in GENERATED:
        if _match(rel, g):
            return "%s is generated or fixed; write it only through its tool" % rel
    if rel.startswith("Firmware/") and not rel.startswith("Firmware/Tests/"):
        return "Firmware/ is read-only outside Firmware/Tests/ (CLAUDE.md hard rules)"
    if rel.startswith("Firmware/Tests/Code_Coverage/") and rel not in approved:
        return ("%s is shared coverage code: edit it only after the owner approved the fix and the lead listed it in %s"
                % (rel, APPROVED_FILE))
    if rel in ("Firmware/Tests/CMakeLists.txt", "Firmware/Tests/CMakePresets.json") and rel not in approved:
        return "%s changes need the owner's approval and a line in %s" % (rel, APPROVED_FILE)
    if not agent_type:
        return None
    # Subagents.
    if rel.startswith(".claude/") or rel in ("CLAUDE.md", "AGENTS.md"):
        return "agents, hooks, settings and rule files are the lead's"
    if agent_type == WRITER:
        if rel == HOOK_TESTS:
            return "the hook tests are lead configuration"
        if rel.startswith("Firmware/Tests/Integration_Tests/") or rel in approved:
            return None
        return "%s is outside the writer's paths (Firmware/Tests/Integration_Tests/ and %s)" % (rel, APPROVED_FILE)
    if agent_type == ANALYST:
        if rel.startswith("Tests/Integration_Tests/claude/charters/stability/") or \
           rel.startswith("Firmware/Tests/Integration_Tests/claude/oracles/"):
            return None
        return "%s is outside the stability analyst's paths" % rel
    return "agent %s has no write paths in this repository" % agent_type


# ---------------------------------------------------------------- Bash parsing

_HEREDOC = re.compile(r"<<-?\s*(['\"]?)(\w+)\1[^\n]*\n.*?\n\s*\2\s*(?:\n|$)", re.S)


def split_segments(command):
    """Split a shell command into (words, redirect targets) per simple command.

    Heredoc bodies are dropped (they are data). `$(...)` and backticks are opened up so the inner
    command is checked as its own segment. Raises ValueError when the command cannot be parsed."""
    text = _HEREDOC.sub("\n", command)
    text = text.replace("\\\n", " ").replace("$(", " ; ").replace("`", " ; ")
    lex = shlex.shlex(text, posix=True, punctuation_chars=";&|()<>\n")
    lex.whitespace = " \t\r"
    lex.whitespace_split = True
    lex.commenters = ""
    tokens = list(lex)
    segments, cur = [], []
    i = 0
    while i < len(tokens):
        t = tokens[i]
        if re.fullmatch(r"&?>{1,2}[|&]?", t):
            if cur and isinstance(cur[-1], str) and cur[-1].isdigit():
                cur.pop()  # file-descriptor number of "2>" / "2>&1" (shlex splits it off)
            if i + 1 < len(tokens) and not (t.endswith("&") and tokens[i + 1].isdigit()):
                cur.append(("REDIRECT", tokens[i + 1]))
            i += 2
            continue
        if t.startswith("<"):
            i += 2
            continue
        if set(t) <= set(";&|()\n"):
            if cur:
                segments.append(cur)
                cur = []
            i += 1
            continue
        cur.append(t)
        i += 1
    if cur:
        segments.append(cur)
    out = []
    for seg in segments:
        words = [t for t in seg if not isinstance(t, tuple)]
        reds = [t[1] for t in seg if isinstance(t, tuple)]
        out.append((words, reds))
    return out


def strip_assignments(seg):
    i = 0
    while i < len(seg) and re.match(r"^[A-Za-z_][A-Za-z0-9_]*=", seg[i]):
        i += 1
    seg = seg[i:]
    while seg and seg[0] in ("sudo", "env", "command", "exec", "time", "nice", "nohup"):
        seg = seg[1:]
    return seg


def parse_git(seg):
    """(global options as (flag, value) list, subcommand, args) for a `git ...` token list."""
    opts, i = [], 1
    while i < len(seg) and seg[i].startswith("-"):
        t = seg[i]
        if t in ("-c", "-C", "--git-dir", "--work-tree", "--namespace") and i + 1 < len(seg):
            opts.append((t, seg[i + 1]))
            i += 2
        else:
            opts.append((t, None))
            i += 1
    sub = seg[i] if i < len(seg) else ""
    return opts, sub, seg[i + 1:]


def git(root, *args):
    return subprocess.run(["git", "-C", root] + list(args), capture_output=True, text=True)


def blob_of_file(root, rel):
    p = os.path.join(root, rel)
    if not os.path.lexists(p):
        return None
    r = git(root, "hash-object", "--", rel)
    return r.stdout.strip() if r.returncode == 0 else None


def blob_of_index(root, rel):
    r = git(root, "rev-parse", "-q", "--verify", ":" + rel)
    return r.stdout.strip() if r.returncode == 0 else None


def fingerprint(files):
    """SHA-256 over the sorted (path, blob) pairs; the gate writes it, the hooks compare it."""
    h = hashlib.sha256()
    for k in sorted(files):
        h.update(("%s\0%s\n" % (k, files[k] or "deleted")).encode())
    return h.hexdigest()


def load_evidence(root, wo):
    p = os.path.join(gate_dir(root), wo, "latest.json")
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f), p
    except (OSError, ValueError):
        return None, p


def is_doc(rel):
    return (rel.startswith("Tests/Integration_Tests/") or rel.startswith("Tests/Code_Coverage/claude/")) \
        and "codex" not in rel.split("/")


def is_config(rel):
    return any(_match(rel, p) for p in CONFIG_PATHS)


def commit_block_reason(root, opts, args, cwd, run_hook_tests=True):
    """Checks of AGENT_PLAN.md 2.3 for a `git commit` by the lead."""
    cvals = [v for f, v in opts if f == "-c"]
    if "user.name=%s" % COMMIT_NAME not in cvals or "user.email=" not in cvals:
        return 'commit with: git -c user.name="%s" -c user.email= commit ... (email deliberately empty)' % COMMIT_NAME
    if any(v.startswith("user.email=") and v != "user.email=" for v in cvals):
        return "the commit e-mail must be empty"
    if any(f in ("-C", "--git-dir", "--work-tree") for f, _ in opts):
        return "commit from the repository itself (no -C / --git-dir / --work-tree)"
    msg_parts, i = [], 0
    while i < len(args):
        a = args[i]
        if a in ("-m", "--message") and i + 1 < len(args):
            msg_parts.append(args[i + 1]); i += 2; continue
        if a.startswith("-m") and len(a) > 2:
            msg_parts.append(a[2:]); i += 1; continue
        if a.startswith("--message="):
            msg_parts.append(a.split("=", 1)[1]); i += 1; continue
        if a in ("-F", "--file") and i + 1 < len(args):
            try:
                with open(os.path.join(cwd or root, args[i + 1]), encoding="utf-8") as f:
                    msg_parts.append(f.read())
            except OSError:
                return "cannot read the commit message file %s" % args[i + 1]
            i += 2; continue
        if a in ("-a", "--all", "-i", "--include", "-o", "--only", "--no-verify", "-n", "--author") or \
           a.startswith("--author=") or not a.startswith("-"):
            return "commit only what is staged: no -a / --include / --only / pathspecs / --author / --no-verify (%s)" % a
        i += 1
    if not msg_parts:
        return "give the commit message with -m or -F so the hook can read the WO id"
    msg = "\n".join(msg_parts)
    br = git(root, "rev-parse", "--abbrev-ref", "HEAD").stdout.strip()
    if br != BRANCH:
        return "commits go to branch %s only (current: %s)" % (BRANCH, br)
    st = git(root, "diff", "--cached", "--name-only", "--no-renames", "-z")
    staged = [p for p in st.stdout.split("\0") if p]
    if not staged:
        return "nothing is staged"
    for p in staged:
        if "codex" in p.split("/") or p == "AGENTS.md":
            return "%s is not Claude's to commit; unstage it" % p
        r = path_block_reason(p, root)
        if r and not is_doc(p):
            return "staged %s: %s" % (p, r)
    code = [p for p in staged if not is_doc(p) and not is_config(p)]
    config = [p for p in staged if is_config(p)]
    if config and run_hook_tests:
        t = subprocess.run(["python3", "-B", os.path.join(root, HOOK_TESTS)], cwd=root, capture_output=True, text=True)
        if t.returncode != 0:
            return "hook configuration is staged but %s fails:\n%s" % (HOOK_TESTS, t.stderr[-1500:])
    if not code:
        return None
    m = re.search(r"^WO:\s*([A-Za-z0-9_.-]+)\s*$", msg, re.M)
    if not m:
        return "staged test code needs a 'WO: <id>' line in the commit message and that WO's gate evidence"
    ev, evp = load_evidence(root, m.group(1))
    if not ev or ev.get("result") != "PASS":
        return "no GATE PASS evidence at %s; run it_gate.py --wo %s" % (evp, m.group(1))
    files = ev.get("files") or {}
    for p in code:
        if p not in files:
            return "staged %s is not in the gate evidence of %s (%s)" % (p, m.group(1), evp)
        if blob_of_index(root, p) != files[p]:
            return "staged %s differs from what the gate of %s tested; re-run the gate" % (p, m.group(1))
    return None


def bash_block_reason(command, root, cwd=None, agent_type=None, run_hook_tests=True):
    try:
        segments = split_segments(command)
    except ValueError:
        if re.search(r"\bgit\b", command):
            return "the command could not be parsed and contains git: split it into simple commands"
        return None
    cwd = cwd or root
    for seg, redirects in segments:
        for t in redirects:
            r = path_block_reason(rel_path(t, root, cwd), root, agent_type)
            if r:
                return "redirect into %s: %s" % (t, r)
        seg = strip_assignments(seg)
        if not seg:
            continue
        prog = os.path.basename(seg[0])
        args = seg[1:]
        if prog == "cd":
            if args:
                cwd = os.path.realpath(os.path.join(cwd, os.path.expanduser(args[0])))
            continue
        if prog in ("pip", "pip3", "apt", "apt-get", "dnf", "yum", "pacman", "brew", "npm", "npx", "yarn",
                    "pnpm", "curl", "wget", "conda", "snap", "flatpak", "yay", "paru"):
            return "no installs or downloads (%s)" % prog
        if prog.startswith("python") and len(args) >= 2 and args[0] == "-m" and args[1] in ("pip", "ensurepip"):
            return "no installs (python -m pip)"
        if prog == "git":
            r = git_block_reason(seg, root, cwd, agent_type, run_hook_tests)
            if r:
                return r
            continue
        if prog in ("cmake", "ctest"):
            r = cmake_block_reason(prog, args, root, cwd)
            if r:
                return r
            continue
        targets = write_targets(prog, args)
        for t in targets:
            r = path_block_reason(rel_path(t, root, cwd), root, agent_type)
            if r:
                return "%s %s: %s" % (prog, t, r)
    return None


def write_targets(prog, args):
    files = [a for a in args if not a.startswith("-")]
    if prog in ("chmod", "chown", "chgrp"):
        return files[1:]  # the first word is the mode / owner
    if prog in ("rm", "rmdir", "touch", "truncate", "shred", "unlink", "ln"):
        return files
    if prog == "tee":
        return files
    if prog in ("cp", "mv", "install", "rsync"):
        for i, a in enumerate(args):
            if a in ("-t", "--target-directory") and i + 1 < len(args):
                return [args[i + 1]]
            if a.startswith("--target-directory="):
                return [a.split("=", 1)[1]]
        return files[-1:] if len(files) >= 2 else []
    if prog in ("sed", "perl") and any(a == "-i" or a.startswith("-i") or a.startswith("--in-place") for a in args):
        rest = [a for a in args if not a.startswith("-")]
        if prog == "sed" and not any(a in ("-e", "--expression", "-f") for a in args):
            rest = rest[1:]  # first non-option word is the script
        return rest
    if prog == "patch":
        return files or ["."]
    if prog == "dd":
        return [a[3:] for a in args if a.startswith("of=")]
    return []


def cmake_block_reason(prog, args, root, cwd):
    flags = {"cmake": ("-B", "--build", "--install"), "ctest": ("--test-dir", "--build-and-test")}[prog]
    if prog == "cmake" and "--preset" in args or any(a.startswith("--preset=") for a in args):
        if prog == "cmake":
            return "cmake --preset builds inside the repository; use -B ../ffb-test-build/<variant>"
    for i, a in enumerate(args):
        val = None
        if a in flags and i + 1 < len(args):
            val = args[i + 1]
        elif a.startswith("-B") and len(a) > 2 and prog == "cmake":
            val = a[2:]
        elif "=" in a and a.split("=", 1)[0] in flags:
            val = a.split("=", 1)[1]
        if val is not None and rel_path(val, root, cwd) is not None:
            return "build directories must be outside the repository (../ffb-test-build/<variant>), not %s" % val
    if prog == "cmake" and not any(a in flags or a.startswith("-B") or a.startswith("-P") or a == "-E"
                                   or a.startswith("--") for a in args):
        # plain `cmake <src>` builds in the current directory
        if rel_path(".", root, cwd) is not None:
            return "configure with -S <src> -B ../ffb-test-build/<variant>; the current directory is in the repository"
    return None


def git_block_reason(seg, root, cwd, agent_type, run_hook_tests):
    opts, sub, args = parse_git(seg)
    for f, v in opts:
        if f == "-c" and v and (v.startswith("remote.") or v.startswith("url.") or v.startswith("core.hooksPath")
                                or v.startswith("credential.")):
            return "git -c %s is not allowed (no remote or config changes)" % v
    if sub in ("config", "remote", "filter-branch", "filter-repo", "clone", "fetch", "pull", "replace"):
        return "git %s is not allowed (CLAUDE.md git rules)" % sub
    if sub == "submodule" and args[:1] == ["add"]:
        return "git submodule add is not allowed"
    if sub == "reset" and "--hard" in args:
        return "git reset --hard is not allowed"
    if sub == "push":
        if agent_type:
            return "subagents never push"
        if opts or args != [FORK_URL, BRANCH]:
            return "the only allowed push is: git push %s %s (and only when the owner asks)" % (FORK_URL, BRANCH)
        return None
    if sub == "commit":
        if agent_type:
            return "subagents never commit"
        return commit_block_reason(root, opts, args, cwd, run_hook_tests)
    if agent_type and sub == "stash" and args[:1] in (["list"], ["show"]):
        return None
    if agent_type and sub in ("checkout", "switch", "rebase", "merge", "stash", "cherry-pick", "revert", "restore",
                              "tag", "am", "apply", "add", "rm", "mv", "clean", "gc", "worktree"):
        return "subagents run read-only git commands only (git %s)" % sub
    if agent_type and sub == "branch" and any(a in ("-D", "-d", "-m", "-M", "--delete") for a in args):
        return "subagents never change branches"
    if sub == "branch" and any(a in ("-D", "--delete", "-f", "--force", "-M") for a in args):
        return "branches are not deleted or forced"
    if sub in ("checkout", "switch") and any(a in ("-B", "--force", "-f") for a in args):
        return "no forced checkout"
    if sub == "clean" and any(a.startswith("-") and ("f" in a or a == "--force") for a in args):
        return "git clean -f is not allowed"
    return None


def read_hook_input(stream):
    try:
        return json.load(stream)
    except ValueError:
        return {}
