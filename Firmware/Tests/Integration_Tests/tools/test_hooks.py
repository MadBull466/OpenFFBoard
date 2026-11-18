"""Unit tests of the integration-track hooks in .claude/hooks/ (AGENT_PLAN.md section 2).

Python standard library only: python3 -B Firmware/Tests/Integration_Tests/tools/test_hooks.py
Path rules are checked against a temporary repository (its own APPROVED_SHARED_EDITS.txt); commit checks
run real git in that temporary repository; the hook scripts are run as subprocesses with hook JSON on stdin.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.realpath(os.path.join(HERE, "..", "..", "..", ".."))
HOOKS = os.path.join(REPO, ".claude", "hooks")
sys.path.insert(0, HOOKS)
import guard_lib as g  # noqa: E402

FORK = "https://github.com/MadBull466/OpenFFBoard.git"
ID = ["git", "-c", "user.name=MadBull.Ben", "-c", "user.email="]


def sh(root, *args):
    r = subprocess.run(list(args), cwd=root, capture_output=True, text=True)
    if r.returncode != 0:
        raise AssertionError("%s failed: %s" % (args, r.stderr))
    return r.stdout


class TempRepo(unittest.TestCase):
    """A throw-away git repository at <tmp>/repo with <tmp>/ffb-test-build/it-gate beside it."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.root = os.path.join(self.tmp, "repo")
        os.makedirs(os.path.join(self.root, "Tests/Integration_Tests/claude/plans"))
        os.makedirs(os.path.join(self.root, "Firmware/Tests/Integration_Tests/tools"))
        os.makedirs(os.path.join(self.root, "Firmware/Tests/Code_Coverage/fakes"))
        self.write(g.APPROVED_FILE, "# path | approval date | WO\n"
                   "Firmware/Tests/CMakeLists.txt | 2026-11-18 | I0-4\n"
                   "Firmware/Tests/Code_Coverage/fakes/Approved.h | 2026-11-18\n")  # no WO: not approved
        sh(self.root, "git", "init", "-q", "-b", "add-test-suite")
        self.write("README", "x\n")
        sh(self.root, "git", "add", "-A")
        sh(self.root, *ID, "commit", "-q", "-m", "init")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, rel, text):
        p = os.path.join(self.root, rel)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            f.write(text)

    def block(self, rel, agent=None):
        return g.path_block_reason(rel, self.root, agent)

    def bash(self, cmd, agent=None, cwd=None):
        return g.bash_block_reason(cmd, self.root, cwd or self.root, agent, run_hook_tests=False)

    def evidence(self, wo, files, result="PASS"):
        d = os.path.join(g.gate_dir(self.root), wo)
        os.makedirs(d, exist_ok=True)
        p = os.path.join(d, "latest.json")
        with open(p, "w") as f:
            json.dump({"wo": wo, "result": result, "files": files, "fingerprint": g.fingerprint(files)}, f)
        return p


class PathRules(TempRepo):
    def test_firmware_outside_tests_is_blocked_for_the_lead(self):
        self.assertIsNotNone(self.block("Firmware/FFBoard/Src/Axis.cpp"))
        self.assertIsNotNone(self.block("Firmware/Makefile"))

    def test_integration_test_code_is_allowed_for_the_lead_and_the_writer(self):
        self.assertIsNone(self.block("Firmware/Tests/Integration_Tests/claude/cases/test_f1_setup.cpp"))
        self.assertIsNone(self.block("Firmware/Tests/Integration_Tests/claude/cases/test_f1_setup.cpp", g.WRITER))

    def test_doctest_codex_github_specs_are_blocked(self):
        for rel in ("Firmware/Tests/Code_Coverage/third_party/doctest.h", "Tests/Code_Coverage/codex/README.md",
                    "Firmware/Tests/Code_Coverage/codex/x.cpp", "Firmware/Tests/Integration_Tests/codex/a.cpp",
                    ".github/workflows/ci.yml", "Tests/Code_Coverage/shared/specs/a.pdf"):
            self.assertIsNotNone(self.block(rel), rel)

    def test_generated_files_are_blocked(self):
        for rel in ("Tests/Code_Coverage/claude/reports/BUG_REPORT.md",
                    "Tests/Integration_Tests/claude/reports/INTEGRATION_BUG_REPORT.md",
                    "Firmware/Tests/Integration_Tests/claude/oracles/biquad_frf.csv",
                    "Firmware/Tests/Code_Coverage/baseline/test_cases.txt"):
            self.assertIsNotNone(self.block(rel), rel)

    def test_shared_coverage_code_needs_an_approved_line_with_a_wo(self):
        self.assertIsNotNone(self.block("Firmware/Tests/Code_Coverage/fakes/FakeTmc4671.h"))
        self.assertIsNotNone(self.block("Firmware/Tests/Code_Coverage/fakes/Approved.h"))  # line lacks the WO
        self.write(g.APPROVED_FILE, "Firmware/Tests/Code_Coverage/fakes/FakeTmc4671.h | 2026-11-19 | I0-3a\n")
        self.assertIsNone(self.block("Firmware/Tests/Code_Coverage/fakes/FakeTmc4671.h"))
        self.assertIsNone(self.block("Firmware/Tests/Code_Coverage/fakes/FakeTmc4671.h", g.WRITER))

    def test_cmakelists_needs_approval(self):
        self.assertIsNone(self.block("Firmware/Tests/CMakeLists.txt"))
        self.assertIsNotNone(self.block("Firmware/Tests/CMakePresets.json"))

    def test_rule_files_are_blocked_for_subagents_only(self):
        for rel in (".claude/settings.json", ".claude/hooks/guard_lib.py", "CLAUDE.md", "AGENTS.md"):
            self.assertIsNotNone(self.block(rel, g.WRITER), rel)
        self.assertIsNone(self.block(".claude/settings.json"))
        self.assertIsNone(self.block("CLAUDE.md"))

    def test_writer_may_not_write_documents_or_hook_tests(self):
        self.assertIsNotNone(self.block("Tests/Integration_Tests/claude/plans/PROGRESS.md", g.WRITER))
        self.assertIsNotNone(self.block(g.HOOK_TESTS, g.WRITER))
        self.assertIsNone(self.block("Tests/Integration_Tests/claude/plans/PROGRESS.md"))

    def test_analyst_paths(self):
        self.assertIsNone(self.block("Tests/Integration_Tests/claude/charters/stability/DERIVATION.md", g.ANALYST))
        self.assertIsNone(self.block("Firmware/Tests/Integration_Tests/claude/oracles/loop_model.py", g.ANALYST))
        self.assertIsNotNone(self.block("Firmware/Tests/Integration_Tests/claude/oracles/loop.csv", g.ANALYST))
        self.assertIsNotNone(self.block("Firmware/Tests/Integration_Tests/claude/cases/test_f1_x.cpp", g.ANALYST))

    def test_read_only_agents_write_nothing_in_the_repository(self):
        for agent in ("integration-reviewer", "fake-auditor", "integration-explorer", "driver-test-writer"):
            self.assertIsNotNone(self.block("Firmware/Tests/Integration_Tests/x.cpp", agent), agent)

    def test_paths_outside_the_repository_are_allowed(self):
        self.assertIsNone(g.path_block_reason(g.rel_path("../ffb-test-build/x/y", self.root), self.root, g.WRITER))
        self.assertIsNone(g.path_block_reason(g.rel_path("/tmp/x", self.root), self.root))

    def test_relative_and_symlinked_paths_are_resolved(self):
        rel = g.rel_path("../../FFBoard/Src/Axis.cpp", self.root, os.path.join(self.root, "Firmware/Tests/x"))
        self.assertEqual(rel, "Firmware/FFBoard/Src/Axis.cpp")
        os.makedirs(os.path.join(self.root, "Firmware/FFBoard"))
        os.symlink(os.path.join(self.root, "Firmware/FFBoard"),
                   os.path.join(self.root, "Firmware/Tests/Integration_Tests/link"))
        rel = g.rel_path("Firmware/Tests/Integration_Tests/link/Axis.cpp", self.root)
        self.assertIsNotNone(g.path_block_reason(rel, self.root))


class BashRules(TempRepo):
    def test_ordinary_commands_pass(self):
        for cmd in ("ls -la", "git status --short", "git diff HEAD", "git log --oneline -5",
                    "cmake -S Firmware/Tests -B ../ffb-test-build/it-default",
                    "cmake --build ../ffb-test-build/it-default -j8",
                    "ctest --test-dir ../ffb-test-build/it-default --output-on-failure",
                    "python3 -B Firmware/Tests/Code_Coverage/tools/check_baseline.py --build-dir ../ffb-test-build/x",
                    "grep -rn foo Firmware/FFBoard > ../ffb-test-build/out.txt",
                    "cat <<'EOF' > ../ffb-test-build/a.txt\ngit push origin master\nEOF",
                    "echo hi 2>&1 | tee ../ffb-test-build/log.txt", "chmod +x ../ffb-test-build/gate.sh"):
            self.assertIsNone(self.bash(cmd), cmd)

    def test_only_the_fork_push_passes(self):
        self.assertIsNone(self.bash("git push %s add-test-suite" % FORK))
        self.assertIsNone(self.bash("git push %s add-test-suite 2>&1 | tail -5" % FORK))
        for cmd in ("git push", "git push origin add-test-suite", "git push -f %s add-test-suite" % FORK,
                    "git push %s +add-test-suite" % FORK, "git push %s master" % FORK,
                    "git push --force-with-lease %s add-test-suite" % FORK, "git push --tags %s add-test-suite" % FORK,
                    "git push %s add-test-suite:master" % FORK, "ls && git push origin",
                    "echo $(git push origin master)"):
            self.assertIsNotNone(self.bash(cmd), cmd)
        self.assertIsNotNone(self.bash("git push %s add-test-suite" % FORK, g.WRITER))

    def test_git_config_and_remote_changes_are_blocked(self):
        for cmd in ("git config user.email x", "git remote add fork u", "git -c remote.origin.url=u fetch",
                    "git reset --hard HEAD~1", "git fetch", "git pull", "git clone u", "git filter-branch",
                    "git submodule add u", "git branch -D x", "git clean -fdx"):
            self.assertIsNotNone(self.bash(cmd), cmd)

    def test_installs_and_downloads_are_blocked(self):
        for cmd in ("pip install x", "python3 -m pip install x", "sudo pacman -S x", "curl https://x", "wget u",
                    "npm i x", "FOO=1 apt-get install y"):
            self.assertIsNotNone(self.bash(cmd), cmd)

    def test_in_tree_builds_are_blocked(self):
        for cmd in ("cmake -S Firmware/Tests -B build", "cmake -B Firmware/Tests/build",
                    "cmake --build Firmware/Tests/build", "cmake --preset host", "ctest --test-dir build",
                    "cd Firmware/Tests && cmake .", "cmake -Bbuild"):
            self.assertIsNotNone(self.bash(cmd), cmd)

    def test_shell_writes_into_protected_paths_are_blocked(self):
        for cmd in ("echo x > Firmware/FFBoard/Src/Axis.cpp", "echo x >> Firmware/Makefile && ls",
                    "sed -i s/a/b/ Firmware/Makefile", "cp a.h Firmware/FFBoard/Inc/", "rm Firmware/Makefile",
                    "touch Tests/Code_Coverage/codex/x", "tee .github/x < y",
                    "mv x Tests/Code_Coverage/claude/reports/BUG_REPORT.md",
                    "cd Firmware && echo x > Makefile", "chmod +x Firmware/Makefile"):
            self.assertIsNotNone(self.bash(cmd), cmd)
        self.assertIsNotNone(self.bash("echo x > CLAUDE.md", g.WRITER))
        self.assertIsNone(self.bash("echo x > CLAUDE.md"))

    def test_subagents_cannot_commit_or_change_branches(self):
        for cmd in (" ".join(ID) + " commit -m x", "git checkout -b y", "git stash", "git add -A", "git rebase m"):
            self.assertIsNotNone(self.bash(cmd, g.WRITER), cmd)
        self.assertIsNone(self.bash("git diff HEAD --stat", g.WRITER))
        self.assertIsNone(self.bash("git stash list", g.WRITER))
        self.assertIsNone(self.bash("git -C /x stash show", g.WRITER))


class CommitRules(TempRepo):
    def commit(self, msg, extra=""):
        return self.bash('git -c user.name="MadBull.Ben" -c user.email= commit %s -m "%s"' % (extra, msg))

    def test_identity_is_required(self):
        self.write("Tests/Integration_Tests/claude/plans/PROGRESS.md", "p\n")
        sh(self.root, "git", "add", "-A")
        self.assertIsNotNone(self.bash('git commit -m "x"'))
        self.assertIsNotNone(self.bash('git -c user.name="MadBull.Ben" -c user.email=a@b commit -m "x"'))
        self.assertIsNone(self.commit("Integration: docs"))

    def test_wrong_branch_is_blocked(self):
        sh(self.root, "git", "checkout", "-q", "-b", "other")
        self.write("Tests/Integration_Tests/claude/plans/PROGRESS.md", "p\n")
        sh(self.root, "git", "add", "-A")
        self.assertIn("branch", self.commit("docs"))

    def test_codex_and_agents_md_are_never_committed(self):
        self.write("Tests/Code_Coverage/codex/README.md", "c\n")
        sh(self.root, "git", "add", "-A")
        self.assertIsNotNone(self.commit("x"))
        sh(self.root, "git", "reset", "-q")
        self.write("AGENTS.md", "a\n")
        sh(self.root, "git", "add", "AGENTS.md")
        self.assertIsNotNone(self.commit("x"))

    def test_commit_all_and_pathspecs_are_blocked(self):
        self.write("Tests/Integration_Tests/claude/plans/PROGRESS.md", "p\n")
        sh(self.root, "git", "add", "-A")
        self.assertIsNotNone(self.commit("x", "-a"))
        self.assertIsNotNone(self.bash(" ".join(ID) + ' commit -m "x" Tests'))

    def test_test_code_needs_matching_gate_evidence(self):
        rel = "Firmware/Tests/Integration_Tests/claude/cases/test_f1_setup.cpp"
        self.write(rel, "// v1\n")
        sh(self.root, "git", "add", rel)
        self.assertIn("WO:", self.commit("Integration: setup"))
        self.assertIn("GATE PASS", self.commit("Integration: setup\n\nWO: I0-4"))
        self.evidence("I0-4", {rel: g.blob_of_file(self.root, rel)})
        self.assertIsNone(self.commit("Integration: setup\n\nWO: I0-4"))
        self.write(rel, "// v2\n")
        sh(self.root, "git", "add", rel)
        self.assertIn("differs", self.commit("Integration: setup\n\nWO: I0-4"))

    def test_failed_gate_evidence_does_not_count(self):
        rel = "Firmware/Tests/Integration_Tests/support/ChainScheduler.h"
        self.write(rel, "x\n")
        sh(self.root, "git", "add", rel)
        self.evidence("I0-5", {rel: g.blob_of_file(self.root, rel)}, result="FAIL")
        self.assertIsNotNone(self.commit("x\n\nWO: I0-5"))

    def test_unrelated_dirty_files_do_not_block(self):
        self.write("AGENTS.md", "dirty, not staged\n")
        self.write("Tests/Integration_Tests/claude/plans/HANDOFF.md", "h\n")
        sh(self.root, "git", "add", "Tests/Integration_Tests/claude/plans/HANDOFF.md")
        self.assertIsNone(self.commit("docs"))


class HookScripts(TempRepo):
    def run_hook(self, name, payload):
        env = dict(os.environ, CLAUDE_PROJECT_DIR=self.root)
        return subprocess.run([sys.executable, "-B", os.path.join(HOOKS, name)], input=json.dumps(payload),
                              capture_output=True, text=True, env=env)

    def test_guard_paths_exit_codes(self):
        bad = {"tool_name": "Edit", "cwd": self.root, "tool_input": {"file_path": self.root + "/Firmware/Makefile"}}
        r = self.run_hook("guard_paths.py", bad)
        self.assertEqual(r.returncode, 2)
        self.assertIn("read-only", r.stderr)
        nb = {"tool_name": "NotebookEdit", "tool_input": {"notebook_path": self.root + "/.github/a.ipynb"}}
        self.assertEqual(self.run_hook("guard_paths.py", nb).returncode, 2)
        ok = {"tool_name": "Write", "tool_input": {"file_path": self.root + "/Tests/Integration_Tests/a.md"}}
        self.assertEqual(self.run_hook("guard_paths.py", ok).returncode, 0)
        sub = dict(ok, agent_type=g.WRITER)
        self.assertEqual(self.run_hook("guard_paths.py", sub).returncode, 2)

    def test_guard_bash_exit_codes(self):
        r = self.run_hook("guard_bash.py", {"tool_name": "Bash", "cwd": self.root,
                                            "tool_input": {"command": "git push origin master"}})
        self.assertEqual(r.returncode, 2)
        r = self.run_hook("guard_bash.py", {"tool_name": "Bash", "tool_input": {"command": "ls"}})
        self.assertEqual(r.returncode, 0)

    def test_writer_stop_needs_a_gate_line(self):
        r = self.run_hook("require_gate_evidence.py", {"agent_type": g.WRITER, "last_assistant_message": "done"})
        self.assertEqual(json.loads(r.stdout)["decision"], "block")
        r = self.run_hook("require_gate_evidence.py",
                          {"agent_type": g.WRITER, "last_assistant_message": "x\nGATE FAIL build error"})
        self.assertEqual(r.stdout.strip(), "")
        r = self.run_hook("require_gate_evidence.py",
                          {"agent_type": g.WRITER, "stop_hook_active": True, "last_assistant_message": "done"})
        self.assertEqual(r.stdout.strip(), "")
        r = self.run_hook("require_gate_evidence.py", {"agent_type": "integration-reviewer",
                                                       "last_assistant_message": "done"})
        self.assertEqual(r.stdout.strip(), "")

    def test_writer_stop_checks_the_evidence_against_the_tree(self):
        rel = "Firmware/Tests/Integration_Tests/a.cpp"
        self.write(rel, "1\n")
        p = self.evidence("I0-4", {rel: g.blob_of_file(self.root, rel)})
        msg = {"agent_type": g.WRITER, "last_assistant_message": "counts\nGATE PASS " + p}
        self.assertEqual(self.run_hook("require_gate_evidence.py", msg).stdout.strip(), "")
        self.write(rel, "2\n")
        self.assertIn("block", self.run_hook("require_gate_evidence.py", msg).stdout)

    def test_writer_stop_reads_the_transcript_when_no_message_is_given(self):
        tr = os.path.join(self.tmp, "t.jsonl")
        with open(tr, "w") as f:
            f.write(json.dumps({"type": "assistant", "message": {"role": "assistant",
                                "content": [{"type": "text", "text": "x\nGATE FAIL ctest"}]}}) + "\n")
        r = self.run_hook("require_gate_evidence.py", {"agent_type": g.WRITER, "agent_transcript_path": tr})
        self.assertEqual(r.stdout.strip(), "")


if __name__ == "__main__":
    unittest.main(verbosity=1)
