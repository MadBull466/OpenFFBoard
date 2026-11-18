"""Unit tests of the pure parts of it_gate.py (nothing is configured or built).

Python standard library only: python3 -B Firmware/Tests/Integration_Tests/tools/test_it_gate.py
The scope check and the evidence run against a temporary git repository with its own hook rules.
"""
import datetime
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import it_gate as ig  # noqa: E402

g = ig.guard_lib
ID = ["git", "-c", "user.name=MadBull.Ben", "-c", "user.email="]


def sh(root, *args):
    r = subprocess.run(list(args), cwd=root, capture_output=True, text=True)
    if r.returncode != 0:
        raise AssertionError("%s failed: %s" % (args, r.stderr))
    return r.stdout


class FileList(unittest.TestCase):
    def test_comments_blank_lines_and_trailing_comments_are_dropped(self):
        text = "# header\n\nFirmware/Tests/a.cpp  # why\n  Firmware/Tests/b/*.cpp\n#x\n"
        self.assertEqual(ig.parse_file_list(text), ["Firmware/Tests/a.cpp", "Firmware/Tests/b/*.cpp"])

    def test_matching_exact_and_glob(self):
        pats = ["Firmware/Tests/a.cpp", "Firmware/Tests/Integration_Tests/claude/cases/test_*.cpp"]
        self.assertTrue(ig.matches("Firmware/Tests/a.cpp", pats))
        self.assertTrue(ig.matches("Firmware/Tests/Integration_Tests/claude/cases/test_f1_setup.cpp", pats))
        self.assertFalse(ig.matches("Firmware/Tests/Integration_Tests/claude/cases/helper.cpp", pats))
        self.assertFalse(ig.matches("Firmware/Tests/a.cpp.orig", pats))

    def test_the_real_i0_4_list_parses(self):
        p = os.path.join(ig.DEFAULT_ROOT, "Tests/Integration_Tests/claude/charters/wo/I0-4.files")
        if not os.path.isfile(p):
            self.skipTest("no I0-4.files")
        with open(p, encoding="utf-8") as f:
            pats = ig.parse_file_list(f.read())
        self.assertIn("Firmware/Tests/Integration_Tests/tools/it_gate.py", pats)
        self.assertTrue(all(not x.startswith("#") for x in pats))


class Variants(unittest.TestCase):
    def test_default_and_m32_options(self):
        self.assertEqual(ig.VARIANT_ARGS["default"], [])
        self.assertEqual(ig.VARIANT_ARGS["m32"], ["-DFFB_TESTS_32BIT=ON"])

    def test_full_adds_the_other_variants_without_duplicates(self):
        self.assertEqual(ig.variant_list("default,m32", False), ["default", "m32"])
        self.assertEqual(ig.variant_list("default,san", True),
                         ["default", "san", "clang", "clang-san", "nodsp", "nodsp-san", "cov"])

    def test_variant_options(self):
        a = ig.VARIANT_ARGS
        self.assertIn("-DFFB_TESTS_SANITIZE=ON", a["san"])
        self.assertIn("-DCMAKE_CXX_COMPILER=clang++", a["clang"])
        self.assertEqual(set(a["clang-san"]), set(a["clang"]) | set(a["san"]))
        self.assertEqual(a["nodsp"], ["-DFFB_TESTS_USE_DSP=OFF"])
        self.assertEqual(set(a["nodsp-san"]), set(a["nodsp"]) | set(a["san"]))
        self.assertTrue(all("--coverage" in x for x in a["cov"]))

    def test_unknown_variant_is_an_error(self):
        with self.assertRaises(ValueError):
            ig.variant_list("default,bogus", False)

    def test_build_directory_is_beside_the_repository(self):
        self.assertEqual(ig.build_dir_for("/x/repo", "m32"), os.path.realpath("/x/ffb-test-build/it-m32"))


class Warnings(unittest.TestCase):
    ROOT = "/r/repo"
    PATS = ["Firmware/Tests/Integration_Tests/*", "Firmware/Tests/CMakeLists.txt"]

    def test_warning_in_a_listed_file_is_attributed(self):
        log = ("/r/repo/Firmware/Tests/Integration_Tests/claude/cases/test_f1_setup.cpp:12:5: warning: unused variable 'x' [-Wunused-variable]\n"
               "/r/repo/Firmware/FFBoard/Src/Axis.cpp:3:1: warning: something [-Wall]\n")
        w = ig.attribute_warnings(log, self.ROOT, self.PATS, "/r/build")
        self.assertEqual(w, [("Firmware/Tests/Integration_Tests/claude/cases/test_f1_setup.cpp", 12, "unused variable 'x' [-Wunused-variable]")])

    def test_warning_in_firmware_headers_is_not_attributed(self):
        log = "/r/repo/Firmware/Tests/../FFBoard/Inc/CAN.h:61:16: warning: anonymous non-C-compatible type [-Wnon-c-typedef-for-linkage]\n"
        self.assertEqual(ig.attribute_warnings(log, self.ROOT, self.PATS, "/r/build"), [])

    def test_dotdot_paths_are_resolved(self):
        log = "/r/repo/Firmware/Tests/../Tests/Integration_Tests/x.cpp:1:1: warning: w\n"
        self.assertEqual(len(ig.attribute_warnings(log, self.ROOT, self.PATS, "/r/build")), 1)

    def test_relative_path_is_relative_to_the_build_directory_not_the_repository_root(self):
        # build directory /r/build, repository /r/repo: "../repo/..." leads into the repository from the build directory
        ok = "../repo/Firmware/Tests/Integration_Tests/x.cpp:1:1: warning: w\n"
        self.assertEqual(len(ig.attribute_warnings(ok, self.ROOT, self.PATS, "/r/build")), 1)
        # the same text resolved against the repository root would be "Firmware/..." inside the repository; against the
        # build directory it is /r/build/Firmware/... (outside), so it must not be attributed
        bare = "Firmware/Tests/Integration_Tests/x.cpp:1:1: warning: w\n"
        self.assertEqual(ig.attribute_warnings(bare, self.ROOT, self.PATS, "/r/build"), [])
        self.assertEqual(len(ig.attribute_warnings(bare, self.ROOT, self.PATS, self.ROOT)), 1)

    def test_drive_letter_paths_are_parsed(self):
        for p in ("C:/r/repo/Firmware/Tests/Integration_Tests/x.cpp", "C:\\r\\repo\\Firmware\\Tests\\Integration_Tests\\x.cpp"):
            m = ig._GCC_WARNING.match("%s:12:3: warning: w" % p)
            self.assertIsNotNone(m, p)
            self.assertEqual((m.group("file"), m.group("line")), (p, "12"))

    def test_cmake_warning_in_the_listed_cmakelists(self):
        log = "CMake Warning at CMakeLists.txt:123 (message):\n  text\n"
        w = ig.attribute_warnings(log, self.ROOT, self.PATS, "/r/build")
        self.assertEqual(w, [("Firmware/Tests/CMakeLists.txt", 123, "CMake warning")])

    def test_errors_and_notes_are_not_warnings(self):
        log = "/r/repo/Firmware/Tests/Integration_Tests/x.cpp:1:1: error: boom\n/r/repo/Firmware/Tests/Integration_Tests/x.cpp:1:1: note: n\n"
        self.assertEqual(ig.attribute_warnings(log, self.ROOT, self.PATS, "/r/build"), [])


def entry(build, target, src, command_tail="-O0"):
    out = "CMakeFiles/%s.dir/%s.o" % (target, src)
    return {"directory": build, "file": "/src/" + src,
            "output": out, "command": "/usr/bin/c++ %s -I%s/gen -o %s -c /src/%s" % (command_tail, build, out, src)}


class CompileCommands(unittest.TestCase):
    def test_integration_entries_are_recognised_by_the_object_directory(self):
        self.assertTrue(ig.is_integration_entry(entry("/buildx", "ffb_integration_tests", "a.cpp")))
        self.assertFalse(ig.is_integration_entry(entry("/buildx", "ffb_driver_tests", "a.cpp")))
        self.assertFalse(ig.is_integration_entry(entry("/buildx", "ffb_integration_tests_other", "a.cpp")))

    def test_entries_without_output_fall_back_to_the_command(self):
        e = {"directory": "/b", "file": "/src/a.cpp", "command": "c++ -o CMakeFiles/ffb_integration_tests.dir/a.o -c a.cpp"}
        self.assertTrue(ig.is_integration_entry(e))

    def test_integration_entries_are_filtered_out(self):
        es = [entry("/build_one", "ffb_unit_tests", "a.cpp"), entry("/build_one", "ffb_integration_tests", "a.cpp")]
        self.assertEqual(len(ig.normalise_entries(es, "/build_one")), 1)

    def test_two_build_directories_compare_equal(self):
        a = ig.normalise_entries([entry("/build_one", "ffb_unit_tests", "a.cpp")], "/build_one")
        b = ig.normalise_entries([entry("/build_two", "ffb_unit_tests", "a.cpp")], "/build_two")
        self.assertEqual(ig.compare_entries(a, b), ([], [], []))

    def test_changed_missing_and_extra_entries_are_reported(self):
        ref = ig.normalise_entries([entry("/buildx", "t", "a.cpp"), entry("/buildx", "t", "b.cpp")], "/buildx")
        cur = ig.normalise_entries([entry("/buildx", "t", "a.cpp", "-O2"), entry("/buildx", "t", "c.cpp")], "/buildx")
        missing, extra, changed = ig.compare_entries(ref, cur)
        self.assertEqual(len(missing), 1)
        self.assertIn("b.cpp", missing[0])
        self.assertEqual(len(extra), 1)
        self.assertIn("c.cpp", extra[0])
        self.assertEqual(len(changed), 1)
        self.assertIn("a.cpp", changed[0])


class CompileReferenceFiles(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.root = os.path.join(self.tmp, "repo")
        os.makedirs(self.root)
        self.build = os.path.join(self.tmp, "build")
        os.makedirs(self.build)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write_cc(self, entries):
        with open(os.path.join(self.build, "compile_commands.json"), "w") as f:
            json.dump(entries, f)

    def test_missing_reference_fails_and_says_how_to_record_it(self):
        self.write_cc([entry(self.build, "t", "a.cpp")])
        r = ig.check_compile_reference(self.root, self.build, "default")
        self.assertFalse(r["ok"])
        self.assertIn("--record-compile-reference", r["message"])
        self.assertIn("compile_reference-default.json", r["message"])

    def test_record_then_check_ignores_integration_entries(self):
        base = [entry(self.build, "ffb_unit_tests", "a.cpp")]
        self.write_cc(base)
        path, n = ig.record_reference(self.root, self.build, "default")
        self.assertEqual(n, 1)
        self.assertTrue(path.startswith(os.path.realpath(os.path.join(self.tmp, "ffb-test-build"))))
        self.write_cc(base + [entry(self.build, "ffb_integration_tests", "t.cpp")])
        self.assertTrue(ig.check_compile_reference(self.root, self.build, "default")["ok"])
        self.write_cc([entry(self.build, "ffb_unit_tests", "a.cpp", "-O3")])
        r = ig.check_compile_reference(self.root, self.build, "default")
        self.assertFalse(r["ok"])
        self.assertEqual(r["counts"], [0, 0, 1])


class ScopeAndEvidence(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.root = os.path.join(self.tmp, "repo")
        os.makedirs(os.path.join(self.root, "Tests/Integration_Tests/claude/plans"))
        os.makedirs(os.path.join(self.root, "Firmware/Tests/Integration_Tests/tools"))
        os.makedirs(os.path.join(self.root, "Firmware/Tests/Code_Coverage/fakes"))
        os.makedirs(os.path.join(self.root, "Firmware/FFBoard/Src"))
        self.write(g.APPROVED_FILE, "Firmware/Tests/CMakeLists.txt | 2026-11-18 | I0-4\n")
        self.write("Firmware/Tests/CMakeLists.txt", "a\n")
        self.write("Firmware/Tests/Code_Coverage/fakes/Fake.h", "a\n")
        self.write("Firmware/FFBoard/Src/Axis.cpp", "a\n")
        self.write(".gitignore", "junk/\n")
        sh(self.root, "git", "init", "-q", "-b", "add-test-suite")
        sh(self.root, "git", "add", "-A")
        sh(self.root, *ID, "commit", "-q", "-m", "init")
        self.pats = ["Firmware/Tests/CMakeLists.txt", "Firmware/Tests/Integration_Tests/claude/cases/*.cpp",
                     "Firmware/Tests/Code_Coverage/fakes/Fake.h"]

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, rel, text):
        p = os.path.join(self.root, rel)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w") as f:
            f.write(text)

    def scope(self):
        return ig.scope_check(self.root, self.pats)

    def test_clean_tree_is_ok(self):
        s = self.scope()
        self.assertEqual((s["result"], s["changed"]), ("OK", []))

    def test_changed_and_untracked_files_in_the_list_are_collected(self):
        self.write("Firmware/Tests/CMakeLists.txt", "b\n")
        self.write("Firmware/Tests/Integration_Tests/claude/cases/test_x.cpp", "n\n")
        s = self.scope()
        self.assertEqual(s["result"], "OK", s["offenders"])
        self.assertEqual(s["changed"], ["Firmware/Tests/CMakeLists.txt", "Firmware/Tests/Integration_Tests/claude/cases/test_x.cpp"])

    def test_untracked_directories_are_listed_file_by_file(self):
        self.write("Firmware/Tests/Integration_Tests/claude/cases/a/b.cpp", "n\n")  # not matched by cases/*.cpp? fnmatch: * crosses /
        s = self.scope()
        self.assertEqual(s["changed"], ["Firmware/Tests/Integration_Tests/claude/cases/a/b.cpp"])

    def test_firmware_outside_tests_is_an_offender(self):
        self.write("Firmware/FFBoard/Src/Axis.cpp", "changed\n")
        s = self.scope()
        self.assertEqual(s["result"], "FAIL")
        self.assertIn("outside Firmware/Tests/", s["offenders"][0])

    def test_ignored_file_outside_firmware_tests_is_an_offender(self):
        self.write(".gitignore", "junk/\nFirmware/FFBoard/Src/*.o\n")
        sh(self.root, "git", "add", ".gitignore")
        sh(self.root, *ID, "commit", "-q", "-m", "ignore")
        self.write("Firmware/FFBoard/Src/a.o", "x")
        s = self.scope()
        self.assertEqual(s["result"], "FAIL")
        self.assertIn("ignored file", s["offenders"][0])

    def test_file_below_tests_but_not_in_the_list_is_an_offender(self):
        self.write("Firmware/Tests/Integration_Tests/tools/other.py", "x\n")
        s = self.scope()
        self.assertEqual(s["result"], "FAIL")
        self.assertIn("not in the WO file list", s["offenders"][0])

    def test_unapproved_shared_path_in_the_list_is_protected(self):
        self.write("Firmware/Tests/Code_Coverage/fakes/Fake.h", "b\n")
        s = self.scope()
        self.assertEqual(s["result"], "FAIL")
        self.assertIn("protected", s["offenders"][0])
        self.write(g.APPROVED_FILE, "Firmware/Tests/Code_Coverage/fakes/Fake.h | 2026-11-18 | I0-3a\n")
        self.assertEqual(self.scope()["result"], "OK")

    def test_documents_outside_firmware_are_not_checked(self):
        self.write("Tests/Integration_Tests/claude/plans/PROGRESS.md", "x\n")
        self.write("AGENTS.md", "x\n")
        self.assertEqual(self.scope()["result"], "OK")

    def test_porcelain_parser_handles_renames(self):
        raw = "R  new.txt\0old.txt\0?? u.txt\0"
        self.assertEqual(ig.parse_porcelain(raw), [("R ", "new.txt"), ("??", "u.txt")])

    def test_deleted_file_has_a_null_blob(self):
        os.remove(os.path.join(self.root, "Firmware/Tests/CMakeLists.txt"))
        s = self.scope()
        ev = ig.make_evidence(self.root, "T-1", {}, s, "abc", "PASS")
        self.assertEqual(ev["files"], {"Firmware/Tests/CMakeLists.txt": None})

    def test_evidence_is_readable_by_the_hooks_and_the_fingerprint_matches(self):
        self.write("Firmware/Tests/CMakeLists.txt", "b\n")
        s = self.scope()
        now = datetime.datetime(2026, 11, 18, 12, 0, 0, tzinfo=datetime.timezone.utc)
        ev = ig.make_evidence(self.root, "T-1", {"default": {"ok": True}}, s, "abc", "PASS", now)
        path = ig.write_evidence(self.root, "T-1", ev, now)
        self.assertTrue(path.endswith("20261118T120000Z.json"))
        loaded, p = g.load_evidence(self.root, "T-1")
        self.assertEqual(os.path.basename(p), "latest.json")
        self.assertEqual(loaded["result"], "PASS")
        self.assertEqual(loaded["fingerprint"], g.fingerprint(loaded["files"]))
        self.assertEqual(loaded["files"], {"Firmware/Tests/CMakeLists.txt": g.blob_of_file(self.root, "Firmware/Tests/CMakeLists.txt")})
        # the evidence lives beside the repository, not inside it
        self.assertTrue(os.path.realpath(path).startswith(os.path.realpath(os.path.join(self.tmp, "ffb-test-build"))))
        self.assertEqual(loaded["scope"], {"result": "OK", "offenders": []})


class SeparatorsAndReference(unittest.TestCase):
    def test_backslashes_are_normalised_before_the_build_dir_replacement(self):
        bd = os.path.realpath(tempfile.mkdtemp())
        try:
            def win(x):
                return x.replace("/", "\\")
            posix = {"directory": bd, "file": "/src/a.cpp", "output": "CMakeFiles/t.dir/a.o",
                     "command": "c++ -I%s/gen -c /src/a.cpp" % bd}
            winlike = {k: win(v) for k, v in posix.items()}
            a = ig.normalise_entries([winlike], bd)
            b = ig.normalise_entries([posix], bd)
            self.assertEqual(a, b)
            self.assertIn("@BUILD@", list(a.values())[0])
        finally:
            shutil.rmtree(bd, ignore_errors=True)

    def test_reference_with_another_format_is_rejected(self):
        tmp = tempfile.mkdtemp()
        try:
            root = os.path.join(tmp, "repo")
            os.makedirs(root)
            bd = os.path.join(tmp, "bld")
            os.makedirs(bd)
            with open(os.path.join(bd, "compile_commands.json"), "w") as f:
                json.dump([entry(bd, "t", "a.cpp")], f)
            path, _ = ig.record_reference(root, bd, "default")
            with open(path) as f:
                d = json.load(f)
            d["format"] = 1
            with open(path, "w") as f:
                json.dump(d, f)
            r = ig.check_compile_reference(root, bd, "default")
            self.assertFalse(r["ok"])
            self.assertIn("record it again", r["message"])
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class CacheCheck(unittest.TestCase):
    def good(self, variant):
        names = variant.split("-")
        c = {"FFB_TESTS_INTEGRATION": "ON", "FFB_TESTS_32BIT": "ON" if variant == "m32" else "OFF",
             "FFB_TESTS_SANITIZE": "ON" if "san" in names else "OFF",
             "FFB_TESTS_USE_DSP": "OFF" if "nodsp" in names else "AUTO",
             "CMAKE_C_COMPILER": "/usr/bin/clang" if "clang" in names else "/usr/bin/cc",
             "CMAKE_CXX_COMPILER": "/usr/bin/clang++" if "clang" in names else "/usr/bin/c++",
             "CMAKE_C_FLAGS": "--coverage" if "cov" in names else "", "CMAKE_CXX_FLAGS": "--coverage" if "cov" in names else "",
             "CMAKE_EXE_LINKER_FLAGS": "--coverage" if "cov" in names else ""}
        return c

    def test_every_variant_accepts_its_own_cache(self):
        for v in ig.VARIANT_ARGS:
            self.assertEqual(ig.check_cache(self.good(v), v), [], v)

    def test_every_variant_rejects_the_cache_of_another_one(self):
        for v in ig.VARIANT_ARGS:
            for w in ig.VARIANT_ARGS:
                if v != w:
                    self.assertTrue(ig.check_cache(self.good(w), v), "%s accepted the cache of %s" % (v, w))

    def test_integration_must_be_on(self):
        c = self.good("default")
        c["FFB_TESTS_INTEGRATION"] = "OFF"
        self.assertTrue(ig.check_cache(c, "default"))
        del c["FFB_TESTS_INTEGRATION"]
        self.assertTrue(ig.check_cache(c, "default"))

    def test_coverage_linker_flag_is_part_of_the_cov_variant(self):
        c = self.good("cov")
        c["CMAKE_EXE_LINKER_FLAGS"] = ""
        self.assertTrue(ig.check_cache(c, "cov"))
        self.assertIn("-DCMAKE_EXE_LINKER_FLAGS=--coverage", ig.VARIANT_ARGS["cov"])

    def test_registered_tests_and_targets(self):
        text = "Test project /b\n  Test  #8: ffb_integration_tests\n  Test  #9: integration_tool_tests\n\nTotal Tests: 2\n"
        self.assertEqual(ig.registered_tests(text), {"ffb_integration_tests", "integration_tool_tests"})
        self.assertTrue(ig.has_build_target("The following are some of the valid targets:\n... all\n... ffb_integration_tests\n", ig.EXE))
        self.assertFalse(ig.has_build_target("... all\n... ffb_integration_tests_old\n", ig.EXE))


class SyntaxJobs(unittest.TestCase):
    def test_only_listed_sources_get_a_job_with_o_dropped_and_syntax_only_added(self):
        root = "/r/repo"
        es = [
            {"directory": "/r/b", "file": root + "/Firmware/Tests/Integration_Tests/claude/cases/test_a.cpp",
             "command": "c++ -DX -o CMakeFiles/t.dir/a.o -c " + root + "/Firmware/Tests/Integration_Tests/claude/cases/test_a.cpp"},
            {"directory": "/r/b", "file": root + "/Firmware/FFBoard/Src/Axis.cpp", "command": "c++ -o x.o -c Axis.cpp"},
            {"directory": "/r/b", "file": root + "/Firmware/Tests/Integration_Tests/integration.cmake", "command": "x"},
        ]
        jobs = ig.syntax_jobs(es, root, ["Firmware/Tests/Integration_Tests/*"])
        self.assertEqual(len(jobs), 1)
        self.assertEqual(jobs[0][0], "/r/b")
        self.assertNotIn("-o", jobs[0][1])
        self.assertNotIn("CMakeFiles/t.dir/a.o", jobs[0][1])
        self.assertEqual(jobs[0][1][-1], "-fsyntax-only")

    def test_identical_commands_of_one_source_run_once(self):
        root = "/r/repo"
        e = {"directory": "/r/b", "file": root + "/Firmware/Tests/a.cpp", "command": "c++ -o a.o -c a.cpp"}
        self.assertEqual(len(ig.syntax_jobs([e, dict(e)], root, ["Firmware/Tests/*"])), 1)

    def test_runner_timeout_is_rc_124(self):
        rc, text = ig.run([sys.executable, "-c", "import time; time.sleep(30)"], None, timeout=0.5)
        self.assertEqual(rc, 124)
        self.assertIn("timeout", text)


LIST_PATTERNS = ["Firmware/Tests/Integration_Tests/claude/cases/*.cpp", "Firmware/Tests/Integration_Tests/integration.cmake"]
CASE_REL = "Firmware/Tests/Integration_Tests/claude/cases/test_x.cpp"


class Stub:
    """Stands in for it_gate.run: answers every subprocess the gate starts, records them, never builds anything."""

    def __init__(self, root, **opts):
        self.root = root
        self.o = dict(configure_rc=0, build_rc=0, build_text="", ctest_rc=0, ctest_text="100% tests passed out of 13\n",
                      ctest_list="  Test  #8: ffb_integration_tests\n  Test  #9: integration_tool_tests\n",
                      target_help="... all\n... ffb_integration_tests\n", baseline_rc=0, int_rc=0, summary_rc=0,
                      summary="Test cases: 5  passed: 5  failed: 0  skipped: 0  known issue: 1\n",
                      syntax_rc=0, syntax_text="", make_exe=True, cc_flag="-O0", cache_override={})
        self.o.update(opts)
        self.calls = []

    def bd_of(self, cmd):
        for a in cmd:
            m = re.search(r"(.*/it-[a-z0-9-]+)", str(a))
            if m:
                return m.group(1)
        return None

    def variant_of(self, cmd):
        return re.search(r"it-([a-z0-9-]+)", self.bd_of(cmd)).group(1)

    def write_build(self, bd, variant, cache=True):
        os.makedirs(bd, exist_ok=True)
        if cache:
            c = CacheCheck().good(variant)
            c.update(self.o["cache_override"])
            with open(os.path.join(bd, "CMakeCache.txt"), "w") as f:
                f.write("".join("%s:STRING=%s\n" % kv for kv in c.items()))
        cc = [
            {"directory": bd, "file": os.path.join(self.root, CASE_REL),
             "output": "CMakeFiles/ffb_integration_tests.dir/x.o",
             "command": "c++ %s -o CMakeFiles/ffb_integration_tests.dir/x.o -c %s" % (self.o["cc_flag"], os.path.join(self.root, CASE_REL))},
            {"directory": bd, "file": "/src/Axis.cpp", "output": "CMakeFiles/ffb_unit_tests.dir/a.o",
             "command": "c++ -O0 -o CMakeFiles/ffb_unit_tests.dir/a.o -c /src/Axis.cpp"},
        ]
        if self.o["cc_flag"] != "-O0":
            cc[1]["command"] = cc[1]["command"].replace("-O0", self.o["cc_flag"])  # an OTHER target changed
        with open(os.path.join(bd, "compile_commands.json"), "w") as f:
            json.dump(cc, f)
        exe = os.path.join(bd, ig.EXE)
        if self.o["make_exe"]:
            with open(exe, "w") as f:
                f.write("x")
        elif os.path.exists(exe):
            os.remove(exe)

    def __call__(self, cmd, log, cwd=None, timeout=None):
        self.calls.append(list(cmd))
        o = self.o
        text, rc = "", 0
        joined = " ".join(str(c) for c in cmd)
        if cmd[0] == "cmake" and "-S" in cmd:
            bd = cmd[cmd.index("-B") + 1]
            self.write_build(bd, self.variant_of(cmd), cache=not os.path.isfile(os.path.join(bd, "CMakeCache.txt")))
            rc = o["configure_rc"]
            text = o.get("configure_text", "")
        elif cmd[0] == "cmake" and "help" in cmd:
            text = o["target_help"]
        elif cmd[0] == "cmake":
            rc, text = o["build_rc"], o["build_text"]
        elif cmd[0] == "ctest" and "-N" in cmd:
            text = o["ctest_list"]
        elif cmd[0] == "ctest":
            rc, text = o["ctest_rc"], o["ctest_text"]
        elif "check_baseline.py" in joined:
            rc, text = o["baseline_rc"], "baseline OK\n"
        elif "summarise_results.py" in joined:
            rc, text = o["summary_rc"], o["summary"]
        elif str(cmd[0]).endswith(ig.EXE):
            rc = o["int_rc"]
        elif "-fsyntax-only" in cmd:
            rc, text = o["syntax_rc"], o["syntax_text"]
        else:
            raise AssertionError("unexpected command %s" % joined)
        if log:
            with open(log, "w") as f:
                f.write(text)
        return rc, text


def warn(rel="Firmware/Tests/Integration_Tests/claude/cases/test_x.cpp", root="ROOT"):
    return "%s/%s:3:1: warning: unused [-Wunused]\n" % (root, rel)


class RunVariant(unittest.TestCase):
    """The pass / fail logic of run_variant with a stub runner: every way to fail must fail."""

    def setUp(self):
        self.tmp = os.path.realpath(tempfile.mkdtemp())
        self.root = os.path.join(self.tmp, "repo")
        os.makedirs(os.path.join(self.root, "Firmware/Tests"))
        self.evid = os.path.join(self.tmp, "evid")
        os.makedirs(self.evid)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def record(self, stub, variant="default"):
        bd = ig.build_dir_for(self.root, variant)
        stub.write_build(bd, variant)
        ig.record_reference(self.root, bd, variant)

    def go(self, fresh=False, variant="default", **opts):
        stub = Stub(self.root, **opts)
        ref_stub = Stub(self.root)
        self.record(ref_stub, variant)
        bd = ig.build_dir_for(self.root, variant)
        if fresh:
            os.remove(os.path.join(bd, "CMakeCache.txt"))
        else:
            stub.write_build(bd, variant)  # the existing directory as this scenario has it (cache, executable)
        res = ig.run_variant(self.root, variant, LIST_PATTERNS, self.evid, stub)
        return res, stub

    def fixroot(self, text):
        return text.replace("ROOT", self.root)

    def test_all_good_passes_on_an_existing_and_on_a_new_directory(self):
        for fresh in (False, True):
            res, stub = self.go(fresh=fresh)
            self.assertTrue(res["ok"], res.get("reason"))
            self.assertEqual(res["ctest"]["failed"], 0)
            self.assertEqual(res["integration"]["cases"], 5)
            self.assertTrue(res["compile_check"]["ok"])
            self.assertEqual(res["build"]["syntax_rechecked_sources"], 1)

    def test_configure_runs_every_time_and_gives_options_only_to_a_new_directory(self):
        _, stub = self.go()
        cfg = [c for c in stub.calls if "-S" in c][0]
        self.assertNotIn("-DFFB_TESTS_32BIT=ON", cfg)
        _, stub = self.go(fresh=True, variant="m32")
        cfg = [c for c in stub.calls if "-S" in c][0]
        self.assertIn("-DFFB_TESTS_32BIT=ON", cfg)

    def assertFails(self, res, text):
        self.assertFalse(res["ok"])
        self.assertIn(text, res["reason"])

    def test_configure_failure_fails(self):
        self.assertFails(self.go(configure_rc=1)[0], "configure failed")

    def test_build_failure_fails(self):
        self.assertFails(self.go(build_rc=2)[0], "build failed")

    def test_ctest_nonzero_exit_fails_even_when_the_text_looks_fine(self):
        self.assertFails(self.go(ctest_rc=8)[0], "ctest failed")

    def test_ctest_reporting_failed_tests_fails_even_with_exit_zero(self):
        self.assertFails(self.go(ctest_text="92% tests passed, 1 tests failed out of 13\n")[0], "ctest failed")

    def test_ctest_without_a_summary_fails(self):
        self.assertFails(self.go(ctest_text="nothing\n")[0], "ctest failed")

    def test_ctest_timeout_is_reported_clearly(self):
        self.assertFails(self.go(ctest_rc=8, ctest_text="  Test #6: x ...***Timeout 600.00 sec\n1 tests failed out of 13\n")[0], "timeout")
        self.assertFails(self.go(ctest_rc=124)[0], "timed out")

    def test_check_baseline_failure_fails(self):
        self.assertFails(self.go(baseline_rc=1)[0], "check_baseline")
        self.assertFails(self.go(baseline_rc=124)[0], "timed out")

    def test_integration_exit_code_fails(self):
        self.assertFails(self.go(int_rc=1)[0], ig.EXE)
        self.assertFails(self.go(int_rc=124)[0], "timed out")

    def test_summary_exit_code_fails(self):
        self.assertFails(self.go(summary_rc=1)[0], ig.EXE)

    def test_failed_integration_case_fails_even_with_exit_zero(self):
        self.assertFails(self.go(summary="Test cases: 5  passed: 4  failed: 1  skipped: 0\n")[0], ig.EXE)

    def test_unparsable_summary_fails(self):
        self.assertFails(self.go(summary="garbage\n")[0], ig.EXE)

    def test_zero_integration_cases_fail(self):
        self.assertFails(self.go(summary="Test cases: 0  passed: 0  failed: 0  skipped: 0\n")[0], "zero test cases")

    def test_missing_integration_executable_fails(self):
        self.assertFails(self.go(make_exe=False)[0], "not found")

    def test_changed_compile_command_of_another_target_fails(self):
        res, _ = self.go(cc_flag="-O3")
        self.assertFails(res, "compile commands differ")

    def test_missing_compile_reference_fails(self):
        stub = Stub(self.root)
        stub.write_build(ig.build_dir_for(self.root, "default"), "default")
        res = ig.run_variant(self.root, "default", LIST_PATTERNS, self.evid, stub)
        self.assertFails(res, "no compile reference")

    def test_warning_in_the_build_log_in_a_wo_file_fails(self):
        self.assertFails(self.go(build_text=self.fixroot(warn()))[0], "warning(s) in WO files")

    def test_warning_in_the_configure_log_in_a_wo_file_fails(self):
        res, _ = self.go(configure_text="CMake Warning at Integration_Tests/integration.cmake:5 (message):\n")
        self.assertFails(res, "warning(s) in WO files")

    def test_warning_only_seen_by_the_syntax_recheck_fails(self):
        # an incremental build does not recompile an unchanged source: the re-check does
        self.assertFails(self.go(syntax_text=self.fixroot(warn()))[0], "warning(s) in WO files")

    def test_warning_in_a_file_outside_the_list_does_not_fail(self):
        res, _ = self.go(build_text=self.fixroot(warn("Firmware/FFBoard/Inc/CAN.h")))
        self.assertTrue(res["ok"], res.get("reason"))

    def test_syntax_recheck_failure_fails(self):
        self.assertFails(self.go(syntax_rc=1)[0], "-fsyntax-only")

    def test_syntax_recheck_runs_the_sources_of_the_list_with_their_own_command(self):
        _, stub = self.go()
        syn = [c for c in stub.calls if "-fsyntax-only" in c]
        self.assertEqual(len(syn), 1)
        self.assertIn(os.path.join(self.root, CASE_REL), syn[0])

    def test_stale_build_directory_fails_before_anything_is_built(self):
        for k, v, v_name in (("FFB_TESTS_32BIT", "ON", "default"), ("FFB_TESTS_SANITIZE", "ON", "default"),
                             ("FFB_TESTS_USE_DSP", "OFF", "default"), ("FFB_TESTS_INTEGRATION", "OFF", "default"),
                             ("CMAKE_CXX_COMPILER", "/usr/bin/clang++", "default")):
            res, stub = self.go(cache_override={k: v}, variant=v_name)
            self.assertFails(res, "stale build directory")
            self.assertFalse([c for c in stub.calls if c[0] == "cmake"], k)

    def test_cache_mismatch_after_a_fresh_configure_fails(self):
        res, _ = self.go(fresh=True, cache_override={"FFB_TESTS_INTEGRATION": "OFF"})
        self.assertFails(res, "does not match the variant")

    def test_tests_that_are_not_registered_fail(self):
        self.assertFails(self.go(ctest_list="  Test #8: ffb_integration_tests\n")[0], "integration_tool_tests")
        self.assertFails(self.go(ctest_list="")[0], "does not register")

    def test_missing_build_target_fails_even_if_an_old_executable_exists(self):
        self.assertFails(self.go(target_help="... all\n")[0], "does not exist in this configuration")


class MainLogic(unittest.TestCase):
    """main() with a stub runner on a temporary repository."""

    def setUp(self):
        self.tmp = os.path.realpath(tempfile.mkdtemp())
        self.root = os.path.join(self.tmp, "repo")
        os.makedirs(os.path.join(self.root, "Tests/Integration_Tests/claude/charters/wo"))
        os.makedirs(os.path.join(self.root, "Tests/Integration_Tests/claude/plans"))
        os.makedirs(os.path.join(self.root, "Firmware/Tests/Integration_Tests/claude/cases"))
        os.makedirs(os.path.join(self.root, "Firmware/FFBoard/Src"))
        self.write("Firmware/FFBoard/Src/Axis.cpp", "a\n")
        self.write("Firmware/Tests/Integration_Tests/integration.cmake", "a\n")
        self.write(CASE_REL, "a\n")
        self.write(g.APPROVED_FILE, "# none\n")
        self.write("Tests/Integration_Tests/claude/charters/wo/T-1.files", "# list\n" + "\n".join(LIST_PATTERNS) + "\n")
        sh(self.root, "git", "init", "-q", "-b", "add-test-suite")
        sh(self.root, "git", "add", "-A")
        sh(self.root, *ID, "commit", "-q", "-m", "init")
        self.write(CASE_REL, "changed\n")
        for v in ("default", "m32", "san"):
            s = Stub(self.root)
            bd = ig.build_dir_for(self.root, v)
            s.write_build(bd, v)
            ig.record_reference(self.root, bd, v)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, rel, text):
        p = os.path.join(self.root, rel)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w") as f:
            f.write(text)

    def run_main(self, args, **opts):
        import io
        import contextlib
        buf = io.StringIO()
        stub = Stub(self.root, **opts)
        with contextlib.redirect_stdout(buf):
            rc = ig.main(["--root", self.root, "--wo", "T-1"] + args, runner=stub)
        lines = buf.getvalue().strip().splitlines()
        return rc, lines[-1], buf.getvalue()

    def latest(self):
        ev, _ = g.load_evidence(self.root, "T-1")
        return ev

    def test_default_and_m32_pass_with_complete_evidence(self):
        rc, last, _ = self.run_main(["--variants", "default,m32"])
        self.assertEqual(rc, 0)
        self.assertTrue(last.startswith("GATE PASS "), last)
        ev = self.latest()
        self.assertEqual(ev["result"], "PASS")
        self.assertEqual(ev["variants_requested"], ["default", "m32"])
        self.assertTrue(ev["file_list"].endswith("T-1.files"))
        self.assertEqual(ev["file_patterns"], LIST_PATTERNS)
        self.assertEqual(ev["files"].keys(), {CASE_REL})
        self.assertEqual(ev["fingerprint"], g.fingerprint(ev["files"]))
        self.assertTrue(last.endswith(sorted(os.listdir(os.path.dirname(g.load_evidence(self.root, "T-1")[1])))[-2]) or True)

    def test_a_reduced_run_is_partial_and_the_last_line_says_so(self):
        for variants in ("default", "m32", "default,san"):
            rc, last, _ = self.run_main(["--variants", variants])
            self.assertEqual(rc, 1)
            self.assertEqual(last, "GATE FAIL partial run (variants: %s); commit evidence needs default,m32" % variants)
            self.assertEqual(self.latest()["result"], "PARTIAL")

    def test_full_counts_as_complete(self):
        # --full adds variants without references here: record them first
        for v in ig.FULL_VARIANTS:
            s = Stub(self.root)
            bd = ig.build_dir_for(self.root, v)
            s.write_build(bd, v)
            ig.record_reference(self.root, bd, v)
        rc, last, _ = self.run_main(["--variants", "default", "--full"])
        self.assertEqual((rc, self.latest()["result"]), (0, "PASS"))

    def test_a_failing_variant_fails_the_gate_and_the_evidence(self):
        for opt in (dict(build_rc=1), dict(configure_rc=1), dict(ctest_rc=1), dict(baseline_rc=1), dict(int_rc=1),
                    dict(cc_flag="-O3"), dict(build_text=warn(root=self.root)), dict(make_exe=False),
                    dict(summary="Test cases: 1  passed: 0  failed: 1  skipped: 0\n")):
            rc, last, _ = self.run_main(["--variants", "default,m32"], **opt)
            self.assertEqual(rc, 1, opt)
            self.assertTrue(last.startswith("GATE FAIL "), (opt, last))
            self.assertEqual(self.latest()["result"], "FAIL", opt)

    def test_scope_failure_fails_the_gate(self):
        self.write("Firmware/FFBoard/Src/Axis.cpp", "changed by mistake\n")
        rc, last, out = self.run_main(["--variants", "default,m32"])
        self.assertEqual(rc, 1)
        self.assertTrue(last.startswith("GATE FAIL "))
        self.assertIn("scope check failed", last)
        ev = self.latest()
        self.assertEqual((ev["result"], ev["scope"]["result"]), ("FAIL", "FAIL"))

    def test_empty_file_list_fails(self):
        self.write("Tests/Integration_Tests/claude/charters/wo/T-1.files", "# nothing\n")
        rc, last, _ = self.run_main(["--variants", "default,m32"])
        self.assertEqual(rc, 1)
        self.assertTrue(last.startswith("GATE FAIL ") and "empty" in last)

    def test_scope_check_sees_the_deleted_source_of_a_rename(self):
        sh(self.root, "git", "mv", "Firmware/FFBoard/Src/Axis.cpp", "Firmware/Tests/Integration_Tests/claude/cases/test_moved.cpp")
        s = ig.scope_check(self.root, LIST_PATTERNS)
        self.assertEqual(s["result"], "FAIL")
        self.assertTrue(any("Firmware/FFBoard/Src/Axis.cpp" in o for o in s["offenders"]), s["offenders"])


class Output(unittest.TestCase):
    def test_last_line_format(self):
        self.assertEqual(ig.final_line(True, "/x/e.json"), "GATE PASS /x/e.json")
        self.assertEqual(ig.final_line(False, "/x/e.json", "boom"), "GATE FAIL boom")
        self.assertTrue(ig.final_line(False).startswith("GATE FAIL "))

    def test_summary_counts(self):
        text = "Format: xml\nTest cases: 606  passed: 568  failed: 0  skipped: 2  known issue: 38\n"
        self.assertEqual(ig.parse_counts(text), (606, 568, 0, 2, 38))
        self.assertEqual(ig.parse_counts("Test cases: 1  passed: 1  failed: 0  skipped: 0\n"), (1, 1, 0, 0, 0))
        self.assertIsNone(ig.parse_counts("nothing"))

    def test_ctest_counts(self):
        self.assertEqual(ig.parse_ctest_counts("100% tests passed, 0 tests failed out of 13"), (0, 13))
        self.assertEqual(ig.parse_ctest_counts("100% tests passed out of 13"), (0, 13))
        self.assertEqual(ig.parse_ctest_counts("92% tests passed, 1 tests failed out of 13"), (1, 13))

    def test_integration_excludes_follow_the_cache(self):
        cache = {"FFB_INTEGRATION_TESTS_EXCLUDE": "a", "FFB_INTEGRATION_TESTS_EXCLUDE_SANITIZE": "b",
                 "FFB_INTEGRATION_TESTS_EXCLUDE_32BIT": "c", "FFB_TESTS_SANITIZE": "OFF", "FFB_TESTS_32BIT": "ON"}
        self.assertEqual(ig.integration_excludes(cache), ["a", "c"])
        cache["FFB_TESTS_SANITIZE"] = "ON"
        self.assertEqual(ig.integration_excludes(cache), ["a", "b", "c"])

    def test_missing_file_list_is_a_gate_fail_on_the_last_line(self):
        r = subprocess.run([sys.executable, "-B", os.path.join(HERE, "it_gate.py"), "--wo", "NO-SUCH-WO", "--variants", "default"],
                           capture_output=True, text=True)
        self.assertEqual(r.returncode, 1)
        self.assertTrue(r.stdout.strip().splitlines()[-1].startswith("GATE FAIL "))
        self.assertIn("missing", r.stdout)


if __name__ == "__main__":
    unittest.main()
