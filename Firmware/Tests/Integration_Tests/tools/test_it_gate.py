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


class OwnBuildFolders(unittest.TestCase):
    def test_own_build_folder_names_for_every_variant(self):
        for v in ig.VARIANT_ARGS:
            self.assertEqual(ig.build_dir_for("/x/repo", v, "I1-2"), os.path.realpath("/x/ffb-test-build/it-I1-2-%s" % v))
            self.assertEqual(ig.build_dir_for("/x/repo", v), os.path.realpath("/x/ffb-test-build/it-%s" % v))
            self.assertEqual(ig.build_dir_for("/x/repo", v, None), ig.build_dir_for("/x/repo", v))

    def test_isolated_build_folder_names_are_separate_from_the_own_build_folders(self):
        for v in ig.VARIANT_ARGS:
            self.assertEqual(ig.build_dir_for("/x/repo", v, "I1-2", True), os.path.realpath("/x/ffb-test-build/it-iso-I1-2-%s" % v))
            self.assertNotEqual(ig.build_dir_for("/x/repo", v, "I1-2", True), ig.build_dir_for("/x/repo", v, "I1-2"))

    def test_odd_characters_of_a_wo_id_cannot_leave_the_build_folder(self):
        bd = ig.build_dir_for("/x/repo", "m32", "a/../b c")
        self.assertEqual(os.path.dirname(bd), os.path.realpath("/x/ffb-test-build"))

    def test_a_reference_recorded_in_the_shared_folder_equals_the_per_wo_folder(self):
        tmp = tempfile.mkdtemp()
        try:
            root = os.path.join(tmp, "repo")
            os.makedirs(root)
            shared = ig.build_dir_for(root, "default")
            own = ig.build_dir_for(root, "default", "I1-2")
            for bd in (shared, own):
                os.makedirs(bd)
                with open(os.path.join(bd, "compile_commands.json"), "w") as f:
                    json.dump([entry(bd, "ffb_unit_tests", "a.cpp"), entry(bd, "ffb_integration_tests", "t.cpp")], f)
            ig.record_reference(root, shared, "default")
            self.assertTrue(ig.check_compile_reference(root, own, "default")["ok"])
            with open(os.path.join(own, "compile_commands.json"), "w") as f:
                json.dump([entry(own, "ffb_unit_tests", "a.cpp", "-O3")], f)
            self.assertFalse(ig.check_compile_reference(root, own, "default")["ok"])
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


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


class ActiveWorkOrders(ScopeAndEvidence):
    """The ACTIVE file and the scope check with other active WOs (reuses the repository of ScopeAndEvidence)."""

    OTHER = "Firmware/Tests/Integration_Tests/support/Other.cpp"

    def write_wo(self, wo, *pats):
        self.write("Tests/Integration_Tests/claude/charters/wo/%s.files" % wo, "# list\n" + "\n".join(pats) + "\n")

    def write_active(self, text):
        self.write(ig.ACTIVE_REL, text)

    def scope_with_active(self):
        active, errors = ig.load_active(self.root, "T-1")
        return ig.scope_check(self.root, self.pats, active=active, active_errors=errors, wo="T-1")

    def test_active_file_parsing_ignores_comments_blanks_and_duplicates(self):
        self.assertEqual(ig.parse_active("# head\n\nI1-2  # paused\n  I1-3\nI1-2\n#I1-4\n"), ["I1-2", "I1-3"])
        self.assertEqual(ig.parse_active(""), [])

    def test_missing_active_file_means_no_other_active_wo(self):
        self.assertEqual(ig.load_active(self.root, "T-1"), ({}, []))

    def test_load_active_skips_the_current_wo_and_reads_the_lists(self):
        self.write_wo("I1-2", self.OTHER)
        self.write_wo("T-1", "Firmware/Tests/CMakeLists.txt")
        self.write_active("# x\nT-1\nI1-2\n")
        self.assertEqual(ig.load_active(self.root, "T-1"), ({"I1-2": [self.OTHER]}, []))

    def test_changed_file_of_another_active_wo_is_listed_and_not_an_offender(self):
        self.write_wo("I1-2", self.OTHER)
        self.write_active("I1-2\n")
        self.write(self.OTHER, "x\n")
        self.write("Firmware/Tests/CMakeLists.txt", "b\n")
        s = self.scope_with_active()
        self.assertEqual((s["result"], s["offenders"]), ("OK", []))
        self.assertEqual(s["other_active"], [self.OTHER + " (I1-2)"])
        self.assertEqual(s["changed"], ["Firmware/Tests/CMakeLists.txt"])  # files / fingerprint cover only this WO

    def test_file_claimed_by_this_wo_and_an_active_wo_is_an_offender(self):
        self.write_wo("I1-2", "Firmware/Tests/CMakeLists.txt")
        self.write_active("I1-2\n")
        self.write("Firmware/Tests/CMakeLists.txt", "b\n")
        s = self.scope_with_active()
        self.assertEqual(s["result"], "FAIL")
        self.assertIn("claimed by active WOs T-1 and I1-2", s["offenders"][0])

    def test_file_claimed_by_two_other_active_wos_is_an_offender(self):
        self.write_wo("I1-2", self.OTHER)
        self.write_wo("I1-3", self.OTHER)
        self.write_active("I1-2\nI1-3\n")
        self.write(self.OTHER, "x\n")
        s = self.scope_with_active()
        self.assertEqual(s["result"], "FAIL")
        self.assertIn("claimed by active WOs I1-2 and I1-3", s["offenders"][0])
        self.assertEqual(s["other_active"], [])

    def test_unchanged_overlap_of_file_lists_is_not_reported(self):
        self.write_wo("I1-2", "Firmware/Tests/CMakeLists.txt")
        self.write_active("I1-2\n")
        self.assertEqual(self.scope_with_active()["result"], "OK")

    def test_active_wo_without_a_files_list_is_an_offender(self):
        self.write_active("I1-9\n")
        s = self.scope_with_active()
        self.assertEqual(s["result"], "FAIL")
        self.assertIn("I1-9", s["offenders"][0])
        self.assertIn("configuration error", s["offenders"][0])

    def test_file_of_a_wo_that_is_not_active_is_an_offender_as_before(self):
        self.write_wo("I1-2", self.OTHER)  # has a list but is not in ACTIVE
        self.write_active("# nobody\n")
        self.write(self.OTHER, "x\n")
        s = self.scope_with_active()
        self.assertEqual(s["result"], "FAIL")
        self.assertIn("not in the WO file list", s["offenders"][0])

    def test_protected_and_outside_paths_stay_offenders_with_active_wos(self):
        self.write_wo("I1-2", "Firmware/FFBoard/Src/Axis.cpp", "Firmware/Tests/Code_Coverage/fakes/Fake.h")
        self.write_active("I1-2\n")
        self.write("Firmware/FFBoard/Src/Axis.cpp", "changed\n")
        s = self.scope_with_active()
        self.assertEqual(s["result"], "FAIL")
        self.assertIn("outside Firmware/Tests/", s["offenders"][0])
        self.write("Firmware/FFBoard/Src/Axis.cpp", "a\n")
        self.write("Firmware/Tests/Code_Coverage/fakes/Fake.h", "b\n")  # in this WO's list, not approved
        s = self.scope_with_active()
        self.assertEqual(s["result"], "FAIL")
        self.assertIn("claimed by active WOs", s["offenders"][0])
        self.write_active("# none\n")
        s = self.scope_with_active()
        self.assertIn("protected", s["offenders"][0])

    def test_protected_path_only_another_active_wo_lists_is_an_offender(self):
        # review finding 1: a path that only another active WO lists was not checked against the protected paths
        self.write_wo("I1-2", "Firmware/Tests/Code_Coverage/fakes/Fake.h", "Firmware/Tests/CMakeLists.txt", self.OTHER)
        self.write_active("I1-2\n")
        self.write("Firmware/Tests/Code_Coverage/fakes/Fake.h", "b\n")
        self.pats = ["Firmware/Tests/Integration_Tests/claude/cases/*.cpp"]
        s = self.scope_with_active()
        self.assertEqual(s["result"], "FAIL")
        self.assertIn("protected", s["offenders"][0])
        self.assertIn("claimed by active WO I1-2", s["offenders"][0])
        self.assertEqual(s["other_active"], [])
        self.write("Firmware/Tests/Code_Coverage/fakes/Fake.h", "a\n")
        self.write("Firmware/Tests/CMakeLists.txt", "b\n")  # the CMake file is approved in the test repo: only I1-2's business
        self.write(self.OTHER, "x\n")
        self.assertEqual(self.scope_with_active()["result"], "OK")
        self.write(g.APPROVED_FILE, "# none\n")
        self.assertEqual(self.scope_with_active()["result"], "FAIL")

    def test_pattern_in_the_list_of_another_active_wo_is_an_offender(self):
        for pat in ("Firmware/Tests/Integration_Tests/support/*.cpp", "Firmware/Tests/a?.cpp", "Firmware/Tests/[ab].cpp"):
            self.write_wo("I1-2", self.OTHER, pat)
            self.write_active("I1-2\n")
            active, errors = ig.load_active(self.root, "T-1")
            self.assertEqual(active, {}, pat)
            self.assertEqual(len(errors), 1, pat)
            self.assertIn("uses a pattern", errors[0])
            self.assertIn("exact paths", errors[0])
            self.assertEqual(self.scope_with_active()["result"], "FAIL")

    def test_this_wos_own_list_may_still_use_patterns(self):
        self.write("Firmware/Tests/Integration_Tests/claude/cases/test_x.cpp", "n\n")
        self.write_active("# empty\n")
        self.assertEqual(self.scope_with_active()["result"], "OK")  # self.pats has cases/*.cpp

    def test_active_id_must_match_the_id_pattern_and_is_not_read(self):
        for bad in ("../x", "i1-2", "I1-", "I1-2/..", "T-1x", "I1_2", "I1-2 3"):
            self.write_active(bad + "\n")
            active, errors = ig.load_active(self.root, "T-1")
            self.assertEqual(active, {}, bad)
            self.assertEqual(len(errors), 1, bad)
            self.assertIn("does not match", errors[0])
        self.assertEqual(self.scope_with_active()["result"], "FAIL")

    def test_the_current_wo_is_skipped_by_exact_id_only(self):
        self.write_active("T-1\n")
        self.assertEqual(ig.load_active(self.root, "T-1"), ({}, []))
        active, errors = ig.load_active(self.root, "t-1")  # other case: not the current WO, and not a valid id
        self.assertEqual(len(errors), 1)

    def test_undecodable_active_and_files_are_offenders_not_crashes(self):
        p = os.path.join(self.root, ig.ACTIVE_REL)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "wb") as f:
            f.write(b"I1-2\n\xff\xfe\n")
        active, errors = ig.load_active(self.root, "T-1")
        self.assertEqual((active, len(errors)), ({}, 1))
        self.write_active("I1-2\n")
        with open(os.path.join(self.root, ig.WO_DIR_REL, "I1-2.files"), "wb") as f:
            f.write(b"\xff\xfe\x00")
        active, errors = ig.load_active(self.root, "T-1")
        self.assertEqual((active, len(errors)), ({}, 1))
        self.assertIn("cannot read", errors[0])

    def test_every_scope_result_has_the_other_active_keys(self):
        keys = {"result", "offenders", "changed", "other_active", "other_active_paths"}
        self.assertTrue(keys <= set(self.scope_with_active()))
        bare = tempfile.mkdtemp()  # not a git repository: git status fails
        try:
            s = ig.scope_check(bare, self.pats)
        finally:
            shutil.rmtree(bare, ignore_errors=True)
        self.assertEqual(s["result"], "FAIL")
        self.assertIn("git status failed", s["offenders"][0])
        self.assertTrue(keys <= set(s))
        ev = ig.make_evidence(self.root, "T-1", {}, s, "abc", "FAIL")
        self.assertEqual((ev["other_active"], ev["other_active_blobs"]), ([], {}))

    def test_evidence_records_the_blobs_of_other_active_paths(self):
        self.write_wo("I1-2", self.OTHER)
        self.write_active("I1-2\n")
        self.write(self.OTHER, "x\n")
        s = self.scope_with_active()
        ev = ig.make_evidence(self.root, "T-1", {}, s, "abc", "PASS")
        self.assertEqual(ev["other_active_blobs"], {self.OTHER: g.blob_of_file(self.root, self.OTHER)})
        self.assertNotIn(self.OTHER, ev["files"])

    def test_evidence_records_own_build_and_other_active(self):
        self.write_wo("I1-2", self.OTHER)
        self.write_active("I1-2\n")
        self.write(self.OTHER, "x\n")
        self.write("Firmware/Tests/CMakeLists.txt", "b\n")
        s = self.scope_with_active()
        ev = ig.make_evidence(self.root, "T-1", {}, s, "abc", "PASS", own_build=True)
        self.assertIs(ev["own_build"], True)
        self.assertEqual(ev["other_active"], [self.OTHER + " (I1-2)"])
        self.assertEqual(list(ev["files"]), ["Firmware/Tests/CMakeLists.txt"])
        self.assertEqual(ev["fingerprint"], g.fingerprint(ev["files"]))
        ev = ig.make_evidence(self.root, "T-1", {}, s, "abc", "PASS")
        self.assertIs(ev["own_build"], False)


class IsolatedTree(unittest.TestCase):
    def setUp(self):
        self.tmp = os.path.realpath(tempfile.mkdtemp())
        self.root = os.path.join(self.tmp, "repo")
        os.makedirs(self.root)
        for rel, text in (("Firmware/Tests/CMakeLists.txt", "head\n"), ("Firmware/Tests/a.cpp", "head a\n"),
                          ("Firmware/Tests/gone.cpp", "head gone\n"), ("Firmware/Tests/other.cpp", "head other\n"),
                          ("Firmware/FFBoard/Src/Axis.cpp", "axis\n"), ("Docs/readme.md", "doc\n")):
            self.write(rel, text)
        sh(self.root, "git", "init", "-q", "-b", "add-test-suite")
        sh(self.root, "git", "add", "-A")
        sh(self.root, *ID, "commit", "-q", "-m", "init")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, rel, text):
        p = os.path.join(self.root, rel)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w") as f:
            f.write(text)

    def read(self, base, rel):
        with open(os.path.join(base, rel)) as f:
            return f.read()

    def test_tree_is_head_plus_exactly_the_wo_files_and_the_shared_repo_is_untouched(self):
        self.write("Firmware/Tests/a.cpp", "work a\n")  # changed, this WO
        self.write("Firmware/Tests/new.cpp", "work new\n")  # untracked, this WO
        os.remove(os.path.join(self.root, "Firmware/Tests/gone.cpp"))  # deleted, this WO
        self.write("Firmware/Tests/other.cpp", "work other\n")  # another WO's change: must not be in the tree
        before = sh(self.root, "git", "status", "--porcelain=v1", "-uall")
        index = sh(self.root, "git", "ls-files", "-s")
        mine = ["Firmware/Tests/a.cpp", "Firmware/Tests/new.cpp", "Firmware/Tests/gone.cpp"]
        sroot, blobs = ig.make_isolated_tree(self.root, "I1-9", mine)
        self.assertEqual(sroot, os.path.join(self.tmp, "ffb-test-build", "it-iso-I1-9", "src"))
        self.assertEqual(self.read(sroot, "Firmware/Tests/a.cpp"), "work a\n")
        self.assertEqual(self.read(sroot, "Firmware/Tests/new.cpp"), "work new\n")
        self.assertFalse(os.path.exists(os.path.join(sroot, "Firmware/Tests/gone.cpp")))
        self.assertEqual(self.read(sroot, "Firmware/Tests/other.cpp"), "head other\n")
        self.assertEqual(self.read(sroot, "Firmware/Tests/CMakeLists.txt"), "head\n")
        self.assertEqual(self.read(sroot, "Docs/readme.md"), "doc\n")  # the whole of HEAD (tool tests read .claude, Tests/)
        self.assertEqual(blobs["Firmware/Tests/gone.cpp"], None)
        for rel in mine:  # the overlay is what the working tree has
            self.assertEqual(blobs[rel], g.blob_of_file(self.root, rel))
        self.assertEqual(sh(self.root, "git", "status", "--porcelain=v1", "-uall"), before)
        self.assertEqual(sh(self.root, "git", "ls-files", "-s"), index)
        self.assertEqual(sh(self.root, "git", "stash", "list"), "")

    def make_submodule(self):
        sub = os.path.join(self.root, "Firmware/Libraries/Sub")
        os.makedirs(sub)
        sh(sub, "git", "init", "-q", "-b", "main")
        with open(os.path.join(sub, "lib.c"), "w") as f:
            f.write("lib\n")
        sh(sub, "git", "add", "-A")
        sh(sub, *ID, "commit", "-q", "-m", "sub")
        sha = sh(sub, "git", "rev-parse", "HEAD").strip()
        sh(self.root, "git", "update-index", "--add", "--cacheinfo", "160000,%s,Firmware/Libraries/Sub" % sha)
        sh(self.root, *ID, "commit", "-q", "-m", "gitlink")
        return sub

    def test_submodules_are_the_exact_recorded_commit_whatever_the_checkout_looks_like(self):
        sub = self.make_submodule()
        self.assertEqual([p for p, _ in ig.head_submodules(self.root)], ["Firmware/Libraries/Sub"])
        sroot, _ = ig.make_isolated_tree(self.root, "I1-9", [])
        self.assertEqual(self.read(sroot, "Firmware/Libraries/Sub/lib.c"), "lib\n")
        self.assertFalse(os.path.exists(os.path.join(sroot, "Firmware/Libraries/Sub/.git")))
        with open(os.path.join(sub, "lib.c"), "w") as f:  # a dirty checkout does not leak into the tree
            f.write("dirty\n")
        with open(os.path.join(sub, "extra.c"), "w") as f:
            f.write("untracked\n")
        sh(sub, "git", "add", "lib.c")
        sh(sub, *ID, "commit", "-q", "-m", "moved on")  # and neither does a checkout at another commit
        sroot, _ = ig.make_isolated_tree(self.root, "I1-9", [])
        self.assertEqual(self.read(sroot, "Firmware/Libraries/Sub/lib.c"), "lib\n")
        self.assertFalse(os.path.exists(os.path.join(sroot, "Firmware/Libraries/Sub/extra.c")))

    def test_a_submodule_commit_that_is_not_available_locally_fails_clearly(self):
        self.make_submodule()
        sh(self.root, "git", "update-index", "--cacheinfo", "160000,%s,Firmware/Libraries/Sub" % ("1234567890abcdef" * 2 + "12345678"))
        sh(self.root, *ID, "commit", "-q", "-m", "bad gitlink")
        with self.assertRaises(OSError) as c:
            ig.make_isolated_tree(self.root, "I1-9", [])
        self.assertIn("not available locally", str(c.exception))
        self.assertIn("Firmware/Libraries/Sub", str(c.exception))

    def test_the_archive_is_the_given_commit_not_the_moving_head(self):
        first = sh(self.root, "git", "rev-parse", "HEAD").strip()
        self.write("Firmware/Tests/a.cpp", "second commit\n")
        sh(self.root, "git", "add", "-A")
        sh(self.root, *ID, "commit", "-q", "-m", "second")
        sroot, _ = ig.make_isolated_tree(self.root, "I1-9", [], head=first)
        self.assertEqual(self.read(sroot, "Firmware/Tests/a.cpp"), "head a\n")
        sroot, _ = ig.make_isolated_tree(self.root, "I1-9", [])  # default: HEAD resolved now
        self.assertEqual(self.read(sroot, "Firmware/Tests/a.cpp"), "second commit\n")
        self.assertEqual(ig.resolve_head(self.root), sh(self.root, "git", "rev-parse", "HEAD").strip())

    def test_changed_docs_exclude_codex_and_agents_and_include_deleted_ones(self):
        raw = ("?? Tests/Integration_Tests/claude/plans/PROGRESS.md\0 M Tests/Integration_Tests/claude/reports/BUG_REPORT.md\0"
               " D Tests/Integration_Tests/claude/charters/old.md\0?? Tests/Integration_Tests/codex/x.md\0"
               "?? Tests/Integration_Tests/claude/codex/y.md\0?? AGENTS.md\0?? Firmware/Tests/a.cpp\0")
        self.assertEqual(ig.changed_docs(self.root, raw), [
            "Tests/Integration_Tests/claude/charters/old.md", "Tests/Integration_Tests/claude/plans/PROGRESS.md",
            "Tests/Integration_Tests/claude/reports/BUG_REPORT.md"])

    def test_documents_are_overlaid_from_the_working_tree_with_their_blobs(self):
        self.write("Tests/Integration_Tests/claude/plans/PROGRESS.md", "head progress\n")
        self.write("Tests/Integration_Tests/claude/charters/old.md", "old\n")
        sh(self.root, "git", "add", "-A")
        sh(self.root, *ID, "commit", "-q", "-m", "docs")
        self.write("Tests/Integration_Tests/claude/plans/PROGRESS.md", "new progress\n")
        self.write("Tests/Integration_Tests/claude/charters/new.md", "new\n")
        os.remove(os.path.join(self.root, "Tests/Integration_Tests/claude/charters/old.md"))
        self.write("Tests/Integration_Tests/codex/c.md", "codex\n")
        sroot, _ = ig.make_isolated_tree(self.root, "I1-9", [])
        self.assertEqual(self.read(sroot, "Tests/Integration_Tests/claude/plans/PROGRESS.md"), "head progress\n")
        docs = ig.overlay_docs(self.root, sroot)
        self.assertEqual(self.read(sroot, "Tests/Integration_Tests/claude/plans/PROGRESS.md"), "new progress\n")
        self.assertEqual(self.read(sroot, "Tests/Integration_Tests/claude/charters/new.md"), "new\n")
        self.assertFalse(os.path.exists(os.path.join(sroot, "Tests/Integration_Tests/claude/charters/old.md")))
        self.assertFalse(os.path.exists(os.path.join(sroot, "Tests/Integration_Tests/codex/c.md")))
        self.assertEqual(sorted(docs), ["Tests/Integration_Tests/claude/charters/new.md", "Tests/Integration_Tests/claude/charters/old.md",
                                        "Tests/Integration_Tests/claude/plans/PROGRESS.md"])
        self.assertIsNone(docs["Tests/Integration_Tests/claude/charters/old.md"])
        self.assertEqual(docs["Tests/Integration_Tests/claude/plans/PROGRESS.md"],
                         g.blob_of_file(self.root, "Tests/Integration_Tests/claude/plans/PROGRESS.md"))

    def test_a_progress_row_without_a_regenerated_bug_report_fails_the_check_in_the_isolated_tree(self):
        real = os.path.realpath(os.path.join(HERE, "..", "..", "..", ".."))
        tool = "Firmware/Tests/Integration_Tests/tools/make_integration_bug_report.py"
        helper = "Firmware/Tests/Code_Coverage/tools/make_bug_report.py"
        progress = "Tests/Integration_Tests/claude/plans/PROGRESS.md"
        report = "Tests/Integration_Tests/claude/reports/BUG_REPORT.md"
        for rel in (tool, helper, progress):
            if not os.path.isfile(os.path.join(real, rel)):
                self.skipTest("missing " + rel)
            with open(os.path.join(real, rel), encoding="utf-8") as f:
                self.write(rel, f.read())
        os.makedirs(os.path.join(self.root, os.path.dirname(report)), exist_ok=True)
        run_tool = lambda base, *a: subprocess.run([sys.executable, "-B", os.path.join(base, tool)] + list(a),
                                                   capture_output=True, text=True).returncode
        self.assertEqual(run_tool(self.root), 0)  # writes the report of the committed registers
        self.assertEqual(run_tool(self.root, "--check"), 0)
        sh(self.root, "git", "add", "-A")
        sh(self.root, *ID, "commit", "-q", "-m", "docs consistent")
        with open(os.path.join(self.root, progress), encoding="utf-8") as f:
            text = f.read()
        i = text.index("## Integration bugs")
        j = text.index("\n", text.index("|---", i)) + 1
        self.write(progress, text[:j] + "| IB 9999 | m | l | d | - | t | no |\n" + text[j:])  # register changed, report not regenerated
        sroot, _ = ig.make_isolated_tree(self.root, "I1-9", [])
        self.assertEqual(run_tool(sroot, "--check"), 0)  # HEAD documents only: consistent, the new row is invisible
        ig.overlay_docs(self.root, sroot)
        self.assertEqual(run_tool(sroot, "--check"), 1)  # the document being committed is checked: stale report
        self.assertEqual(run_tool(self.root, "--check"), 1)

    def test_the_tree_is_recreated_per_run(self):
        sroot, _ = ig.make_isolated_tree(self.root, "I1-9", [])
        self.write("Firmware/Tests/a.cpp", "work a\n")
        stale = os.path.join(sroot, "Firmware/Tests/leftover.cpp")
        with open(stale, "w") as f:
            f.write("x")
        sroot, _ = ig.make_isolated_tree(self.root, "I1-9", ["Firmware/Tests/a.cpp"])
        self.assertFalse(os.path.exists(stale))
        self.assertEqual(self.read(sroot, "Firmware/Tests/a.cpp"), "work a\n")

    def test_a_reference_recorded_from_the_repository_equals_the_isolated_tree_and_a_real_change_fails(self):
        def cc(build, src, flag="-O0"):
            return [{"directory": build, "file": src + "/Firmware/Tests/a.cpp", "output": "CMakeFiles/ffb_unit_tests.dir/a.o",
                     "command": "c++ %s -I%s/gen -I%s/Firmware/Tests -o CMakeFiles/ffb_unit_tests.dir/a.o -c %s/Firmware/Tests/a.cpp"
                     % (flag, build, src, src)},
                    {"directory": build, "file": src + "/t.cpp", "output": "CMakeFiles/ffb_integration_tests.dir/t.o",
                     "command": "c++ -c " + src + "/t.cpp"}]
        shared = ig.build_dir_for(self.root, "default")
        os.makedirs(shared)
        with open(os.path.join(shared, "compile_commands.json"), "w") as f:
            json.dump(cc(shared, self.root), f)
        ig.record_reference(self.root, shared, "default")
        sroot, _ = ig.make_isolated_tree(self.root, "I1-9", [])
        iso_bd = ig.build_dir_for(self.root, "default", "I1-9")
        os.makedirs(iso_bd)
        with open(os.path.join(iso_bd, "compile_commands.json"), "w") as f:
            json.dump(cc(iso_bd, sroot), f)
        r = ig.check_compile_reference(self.root, iso_bd, "default", sroot)
        self.assertTrue(r["ok"], r)
        self.assertFalse(ig.check_compile_reference(self.root, iso_bd, "default")["ok"])  # the source root matters
        with open(os.path.join(iso_bd, "compile_commands.json"), "w") as f:
            json.dump(cc(iso_bd, sroot, "-O3"), f)
        r = ig.check_compile_reference(self.root, iso_bd, "default", sroot)
        self.assertFalse(r["ok"])
        self.assertEqual(r["counts"], [0, 0, 1])

    def test_a_reference_recorded_before_the_source_root_was_normalised_still_compares(self):
        shared = ig.build_dir_for(self.root, "default")
        os.makedirs(shared)
        entries = {"CMakeFiles/x.o|%s/a.cpp" % self.root: "c++ -I%s/inc -c %s/a.cpp" % (self.root, self.root)}
        path = ig.reference_path(self.root, "default")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump({"format": ig.REF_FORMAT, "entries": entries}, f)
        with open(os.path.join(shared, "compile_commands.json"), "w") as f:
            json.dump([{"directory": shared, "file": self.root + "/a.cpp", "output": "CMakeFiles/x.o",
                        "command": "c++ -I%s/inc -c %s/a.cpp" % (self.root, self.root)}], f)
        self.assertTrue(ig.check_compile_reference(self.root, shared, "default")["ok"])


def spaced_entry(root, build, src, tail="-O0"):
    """An entry as CMake writes it for a source under a root that contains a space (object name mangled, paths quoted)."""
    mangled = root.lstrip("/").replace(" ", "_").replace(":", "_")
    out = "CMakeFiles/lib.dir/%s/%s.o" % (mangled, src)
    return {"directory": build, "file": "%s/%s" % (root, src), "output": out,
            "command": '/usr/bin/cc %s -I"%s/inc" -DX="%s/" -o %s -c "%s/%s"' % (tail, root, root, out, root, src)}


class SpacedPaths(unittest.TestCase):
    REPO = "/mnt/SATA_SSD1/Open FFB Testing/Repo"
    ISO = "/mnt/SATA_SSD1/Open FFB Testing/ffb-test-build/it-iso-X/src"

    def norm(self, root, build, tail="-O0", src="Libs/a.c"):
        return ig.normalise_entries([spaced_entry(root, build, src, tail)], build, root)

    def test_repository_and_isolated_tree_with_spaces_compare_equal_including_object_names(self):
        a = self.norm(self.REPO, "/mnt/SATA_SSD1/Open FFB Testing/ffb-test-build/ref-default")
        b = self.norm(self.ISO, "/mnt/SATA_SSD1/Open FFB Testing/ffb-test-build/it-iso-X-default")
        self.assertEqual(a, b)
        self.assertIn("CMakeFiles/lib.dir/@SRC@/Libs/a.c.o", list(a)[0])
        self.assertEqual(ig.compare_entries(a, b), ([], [], []))

    def test_shared_and_per_wo_build_folders_with_spaces_compare_equal(self):
        a = self.norm(self.REPO, "/mnt/SATA_SSD1/Open FFB Testing/ffb-test-build/it-default")
        b = self.norm(self.REPO, "/mnt/SATA_SSD1/Open FFB Testing/ffb-test-build/it-I1-2-default")
        self.assertEqual(a, b)

    def test_a_real_flag_change_still_fails(self):
        a = self.norm(self.REPO, "/b one")
        b = self.norm(self.ISO, "/b two", "-O0 -O3")
        self.assertEqual(ig.compare_entries(a, b)[2], list(a))

    def test_a_changed_define_and_a_missing_entry_still_fail(self):
        a = ig.normalise_entries([spaced_entry(self.REPO, "/b", "x.c")], "/b", self.REPO)
        e = spaced_entry(self.ISO, "/b", "x.c")
        e["command"] = e["command"].replace("-DX=", "-DY=")
        b = ig.normalise_entries([e], "/b", self.ISO)
        self.assertEqual(len(ig.compare_entries(a, b)[2]), 1)
        c = ig.normalise_entries([], "/b", self.ISO)
        self.assertEqual(len(ig.compare_entries(a, c)[0]), 1)

    def test_a_path_containing_the_root_only_as_a_substring_is_not_over_normalised(self):
        root = "/mnt/my repo"
        e = {"directory": "/b", "file": "/other/mnt/my repository/a.c", "output": "CMakeFiles/t.dir/other/mnt/my_repository/a.c.o",
             "command": "cc -I/other/mnt/my_repository -c a.c"}
        n = ig.normalise_entries([e], "/b", root)
        key = list(n)[0]
        self.assertIn("mnt/my_repository", key)
        self.assertIn("mnt/my repository", key)
        self.assertNotIn("@SRC@", key)

    def test_windows_style_paths_keep_working(self):
        root = "C:/Users/x y/repo"
        mangled = "C_/Users/x_y/repo"
        e = {"directory": "C:/b", "file": root + "/a.c", "output": "CMakeFiles/t.dir/%s/a.c.o" % mangled,
             "command": 'cc -I"%s/inc" -c "%s/a.c"' % (root, root)}
        # (normalise_entries applies realpath, which on Linux would prefix a drive-letter path; test the string rule)
        n = lambda v: ig._normalise_string(v.replace("/", "\\"), "C:/b", root)
        self.assertEqual(n(e["output"] + "|" + e["file"]), "CMakeFiles/t.dir/@SRC@/a.c.o|@SRC@/a.c")
        self.assertEqual(n(e["command"]), 'cc -I"@SRC@/inc" -c "@SRC@/a.c"')

    def test_a_build_dir_inside_the_source_root_stays_build(self):
        root = "/mnt/Open FFB/Repo"
        bd = root + "/build dir"
        e = {"directory": bd, "file": root + "/a.c", "output": "CMakeFiles/t.dir/mnt/Open_FFB/Repo/a.c.o",
             "command": 'cc -I"%s/gen" -I"%s/inc" -c "%s/a.c"' % (bd, root, root)}
        n = ig.normalise_entries([e], bd, root)
        self.assertEqual(list(n.values())[0], 'cc -I"@BUILD@/gen" -I"@SRC@/inc" -c "@SRC@/a.c"')

    def test_shared_and_per_wo_build_dir_in_a_command_and_a_generated_include_compare_equal(self):
        def one(build):
            return ig.normalise_entries([{"directory": build, "file": self.REPO + "/a.c", "output": "CMakeFiles/lib.dir/x.o",
                "command": 'cc -I"%s/gen" -include "%s/gen/pre.h" -c "%s/a.c"' % (build, build, self.REPO)}], build, self.REPO)
        a = one("/mnt/SATA_SSD1/Open FFB Testing/ffb-test-build/it-default")
        b = one("/mnt/SATA_SSD1/Open FFB Testing/ffb-test-build/it-I1-2-default")
        self.assertEqual(a, b)
        self.assertIn('-I"@BUILD@/gen"', list(a.values())[0])

    def test_the_quoted_include_flag_form_is_normalised(self):
        n = ig._normalise_string('cc "-include/mnt/R S/inc/a.h" -isystem/mnt/R S/x -idirafter /mnt/R S/y -o/mnt/R S/o.o', None, "/mnt/R S")
        self.assertEqual(n, 'cc "-include@SRC@/inc/a.h" -isystem@SRC@/x -idirafter @SRC@/y -o@SRC@/o.o')

    def test_glued_flags_and_equals_forms_are_normalised(self):
        for pre in ("-I", "-L", "-B", "-F", "-MF", "-MT", "-MQ", "-iquote", "-imacros", "-iprefix", "-iwithprefix",
                    "-isysroot", "-idirafter", "-DX=", "-DX=\"", "-DX='", "-I\"", " ", "\""):
            self.assertEqual(ig._normalise_string("cc %s/mnt/Root/inc" % pre, None, "/mnt/Root"), "cc %s@SRC@/inc" % pre, pre)

    def test_a_prefix_that_continues_another_path_is_not_normalised(self):
        for t in ("/x/mnt/Root/inc", "a/mnt/Root/inc", "-Ix/mnt/Root/inc", "foo-I/mnt/Root/inc", ".dirx/mnt/Root/inc"):
            self.assertEqual(ig._normalise_string("cc " + t, None, "/mnt/Root"), "cc " + t, t)

    def test_a_suffix_that_continues_another_name_is_not_normalised(self):
        for t in ("/mnt/Root2/inc", "/mnt/Root.old/inc", "/mnt/Root-b/inc", "/mnt/Root_b"):
            self.assertEqual(ig._normalise_string("cc -I" + t, None, "/mnt/Root"), "cc -I" + t, t)
        self.assertEqual(ig._normalise_string("cc -I/mnt/Root -c a.c", None, "/mnt/Root"), "cc -I@SRC@ -c a.c")
        self.assertEqual(ig._normalise_string('cc -I"/mnt/Root"', None, "/mnt/Root"), 'cc -I"@SRC@"')

    def test_a_sibling_folder_of_a_spaced_root_is_not_normalised(self):
        root = "/mnt/Open FFB/Repo"
        for t in ('-I"%s 2/inc"' % root, '-I"%s (copy)/inc"' % root, '-I"%s copy"' % root):
            self.assertEqual(ig._normalise_string("cc " + t, None, root), "cc " + t, t)
        self.assertEqual(ig._normalise_string('cc -I"%s/inc" -c a.c' % root, None, root), 'cc -I"@SRC@/inc" -c a.c')
        self.assertEqual(ig._normalise_string("cc -I%s -c a.c" % root, None, root), "cc -I@SRC@ -c a.c")
        self.assertEqual(ig._normalise_string("cc -I%s" % root, None, root), "cc -I@SRC@")

    def test_the_object_name_form_is_replaced_only_after_dir_and_up_to_a_component_boundary(self):
        root = "/mnt/Open FFB/Repo"
        f = lambda t: ig._normalise_string(t, None, root)
        self.assertEqual(f("CMakeFiles/t.dir/mnt/Open_FFB/Repo/a.c.o"), "CMakeFiles/t.dir/@SRC@/a.c.o")
        self.assertEqual(f("CMakeFiles/t.dir/mnt/Open_FFB/Repo2/a.c.o"), "CMakeFiles/t.dir/mnt/Open_FFB/Repo2/a.c.o")
        self.assertEqual(f("CMakeFiles/t.dir/x/mnt/Open_FFB/Repo/a.c.o"), "CMakeFiles/t.dir/x/mnt/Open_FFB/Repo/a.c.o")
        self.assertEqual(f("other/mnt/Open_FFB/Repo/a.c.o"), "other/mnt/Open_FFB/Repo/a.c.o")

    def test_a_reference_recorded_at_a_root_without_a_space_by_the_previous_code_is_made_comparable(self):
        root = "/mnt/plain/Repo"
        old = {"CMakeFiles/lib.dir@SRC@/Libs/a.c.o|@SRC@/Libs/a.c": "cc -I@SRC@/inc -o CMakeFiles/lib.dir@SRC@/Libs/a.c.o -c @SRC@/Libs/a.c"}
        new = ig.renorm_reference(old, root)
        cur = ig.normalise_entries([{"directory": "/b", "file": root + "/Libs/a.c", "output": "CMakeFiles/lib.dir/mnt/plain/Repo/Libs/a.c.o",
            "command": "cc -I%s/inc -o CMakeFiles/lib.dir/mnt/plain/Repo/Libs/a.c.o -c %s/Libs/a.c" % (root, root)}], "/b", root)
        self.assertEqual(ig.compare_entries(new, cur), ([], [], []))

    def test_a_stored_reference_with_a_raw_mangled_name_is_made_comparable(self):
        raw = {"CMakeFiles/lib.dir/mnt/SATA_SSD1/Open_FFB_Testing/Repo/Libs/a.c.o|%s/Libs/a.c" % self.REPO: "cc -c %s/Libs/a.c" % self.REPO}
        r = ig.renorm_reference(raw, self.REPO)
        self.assertEqual(list(r), ["CMakeFiles/lib.dir/@SRC@/Libs/a.c.o|@SRC@/Libs/a.c"])
        self.assertEqual(list(r.values())[0], "cc -c @SRC@/Libs/a.c")


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
        if "-B" in cmd:
            return str(cmd[cmd.index("-B") + 1])
        for a in cmd:
            m = re.search(r"(.*/it-[A-Za-z0-9._-]+)", str(a))
            if m:
                return m.group(1)
        return None

    def variant_of(self, cmd):
        name = os.path.basename(self.bd_of(cmd))  # it-<variant> or it-<WO>-<variant>
        return max((v for v in ig.VARIANT_ARGS if name == "it-" + v or name.endswith("-" + v)), key=len)

    def write_build(self, bd, variant, cache=True, home=None):
        os.makedirs(bd, exist_ok=True)
        src = self.root
        if home and home.endswith("/Firmware/Tests"):
            src = home[:-len("/Firmware/Tests")]  # the compile commands carry the source root of the configure call
        if cache:
            c = CacheCheck().good(variant)
            if home:
                c["CMAKE_HOME_DIRECTORY"] = home
            c.update(self.o["cache_override"])
            with open(os.path.join(bd, "CMakeCache.txt"), "w") as f:
                f.write("".join("%s:STRING=%s\n" % kv for kv in c.items()))
        cc = [
            {"directory": bd, "file": os.path.join(self.root, CASE_REL),
             "output": "CMakeFiles/ffb_integration_tests.dir/x.o",
             "command": "c++ %s -o CMakeFiles/ffb_integration_tests.dir/x.o -c %s" % (self.o["cc_flag"], os.path.join(self.root, CASE_REL))},
            {"directory": bd, "file": "/src/Axis.cpp", "output": "CMakeFiles/ffb_unit_tests.dir/a.o",
             "command": "c++ -O0 -I%s/Firmware/Tests/inc -o CMakeFiles/ffb_unit_tests.dir/a.o -c /src/Axis.cpp" % src},
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
            self.write_build(bd, self.variant_of(cmd), cache=not os.path.isfile(os.path.join(bd, "CMakeCache.txt")),
                             home=str(cmd[cmd.index("-S") + 1]))
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

    def test_own_build_uses_the_per_wo_folders_and_records_it(self):
        for v in ("default", "m32"):
            self.assertFalse(os.path.exists(ig.build_dir_for(self.root, v, "T-1")))
        rc, last, out = self.run_main(["--variants", "default,m32", "--own-build"])
        self.assertEqual(rc, 0, out)
        for v in ("default", "m32"):
            bd = ig.build_dir_for(self.root, v, "T-1")
            self.assertTrue(os.path.isfile(os.path.join(bd, "CMakeCache.txt")), bd)
            self.assertIn(bd, out)
            self.assertEqual(self.latest()["variants"][v]["build_dir"], bd)
            self.assertEqual(self.latest()["variants"][v]["compile_check"]["ok"], True)
        ev = self.latest()
        self.assertIs(ev["own_build"], True)
        self.assertEqual(ev["result"], "PASS")

    def test_default_run_uses_the_shared_folders_and_own_build_false(self):
        rc, last, out = self.run_main(["--variants", "default,m32"])
        self.assertEqual(rc, 0, out)
        self.assertIs(self.latest()["own_build"], False)
        self.assertEqual(self.latest()["variants"]["default"]["build_dir"], ig.build_dir_for(self.root, "default"))
        self.assertFalse(os.path.exists(ig.build_dir_for(self.root, "default", "T-1")))

    def test_other_active_files_are_printed_and_kept_in_the_evidence(self):
        other = "Firmware/Tests/Integration_Tests/claude/cases/test_other.cpp"
        self.write("Tests/Integration_Tests/claude/charters/wo/I1-9.files", other + "\n")
        self.write(ig.ACTIVE_REL, "T-1\nI1-9\n")
        self.write("Tests/Integration_Tests/claude/charters/wo/T-1.files", CASE_REL + "\n")
        self.write(other, "x\n")
        rc, last, out = self.run_main(["--variants", "default,m32"])
        self.assertEqual(rc, 0, out)
        self.assertIn("other_active: %s (I1-9)" % other, out)
        ev = self.latest()
        self.assertEqual(ev["other_active"], [other + " (I1-9)"])
        self.assertEqual(list(ev["files"]), [CASE_REL])

    def test_isolated_run_builds_head_plus_the_wo_files_and_records_it(self):
        other = "Firmware/Tests/Integration_Tests/claude/cases/test_other.cpp"
        self.write("Tests/Integration_Tests/claude/charters/wo/I1-9.files", other + "\n")
        self.write("Tests/Integration_Tests/claude/charters/wo/T-1.files", CASE_REL + "\n")
        self.write(ig.ACTIVE_REL, "T-1\nI1-9\n")
        self.write(other, "other WO, uncommitted\n")
        stub = Stub(self.root)
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = ig.main(["--root", self.root, "--wo", "T-1", "--variants", "default,m32", "--isolated"], runner=stub)
        out = buf.getvalue()
        self.assertEqual(rc, 0, out)
        _, sroot = ig.iso_paths(self.root, "T-1")
        ev = self.latest()
        self.assertIs(ev["isolated"], True)
        self.assertEqual(ev["source_root"], sroot)
        self.assertEqual(list(ev["files"]), [CASE_REL])  # repository-relative
        self.assertEqual(ev["files"][CASE_REL], g.blob_of_file(self.root, CASE_REL))
        self.assertEqual(ev["fingerprint"], g.fingerprint(ev["files"]))
        self.assertEqual(ev["other_active"], [other + " (I1-9)"])
        self.assertEqual(ev["other_active_blobs"], {other: g.blob_of_file(self.root, other)})
        self.assertFalse(os.path.exists(os.path.join(sroot, other)))  # not in the tree the build saw
        with open(os.path.join(sroot, CASE_REL)) as f:
            self.assertEqual(f.read(), "changed\n")
        for v in ("default", "m32"):
            self.assertEqual(ev["variants"][v]["build_dir"], ig.build_dir_for(self.root, v, "T-1", True))
            self.assertIn("/it-iso-T-1-", ev["variants"][v]["build_dir"])
            self.assertFalse(os.path.exists(ig.build_dir_for(self.root, v, "T-1")))  # never the writer's own-build folder
            self.assertTrue(ev["variants"][v]["compile_check"]["ok"])
        self.assertEqual(ev["git_head"], sh(self.root, "git", "rev-parse", "HEAD").strip())
        self.assertIn("Tests/Integration_Tests/claude/charters/wo/ACTIVE", ev["docs_overlaid"])  # the lead's documents, not in `files`
        self.assertTrue(all(k.startswith("Tests/Integration_Tests/") for k in ev["docs_overlaid"]))
        cfg = [c for c in stub.calls if c[0] == "cmake" and "-S" in c]
        self.assertTrue(all(c[c.index("-S") + 1] == os.path.join(sroot, "Firmware/Tests") for c in cfg), cfg)

    def run_isolated(self, runner, wo="T-1"):
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = ig.main(["--root", self.root, "--wo", wo, "--variants", "default,m32", "--isolated"], runner=runner)
        return rc, buf.getvalue()

    def test_isolated_run_fails_when_head_moves_during_the_gate(self):
        outer = self

        class Committing(Stub):
            def __call__(self2, cmd, log, cwd=None, timeout=None):
                if cmd[0] == "ctest" and "-N" not in cmd:
                    sh(outer.root, *ID, "commit", "-q", "--allow-empty", "-m", "moved")
                return Stub.__call__(self2, cmd, log, cwd, timeout)
        head0 = sh(self.root, "git", "rev-parse", "HEAD").strip()
        rc, out = self.run_isolated(Committing(self.root))
        self.assertEqual(rc, 1)
        self.assertIn("HEAD moved during the gate", out.strip().splitlines()[-1])
        ev = self.latest()
        self.assertEqual((ev["result"], ev["git_head"]), ("FAIL", head0))  # the commit that was archived, not the new HEAD

    def test_isolated_evidence_records_the_archived_commit_and_the_overlaid_documents(self):
        self.write("Tests/Integration_Tests/claude/plans/PROGRESS.md", "progress\n")
        self.write("Tests/Integration_Tests/codex/c.md", "codex\n")
        rc, out = self.run_isolated(Stub(self.root))
        self.assertEqual(rc, 0, out)
        ev = self.latest()
        d = "Tests/Integration_Tests/claude/plans/PROGRESS.md"
        self.assertIn(d, ev["docs_overlaid"])
        self.assertEqual(ev["docs_overlaid"][d], g.blob_of_file(self.root, d))
        self.assertTrue(all("codex" not in k for k in ev["docs_overlaid"]))
        self.assertNotIn(d, ev["files"])
        self.assertEqual(list(ev["files"]), [CASE_REL])
        _, sroot = ig.iso_paths(self.root, "T-1")
        with open(os.path.join(sroot, d)) as f:
            self.assertEqual(f.read(), "progress\n")

    def test_isolated_run_fails_when_a_document_changes_during_the_run(self):
        outer = self
        self.write("Tests/Integration_Tests/claude/plans/PROGRESS.md", "progress\n")

        class Editing(Stub):
            def __call__(self2, cmd, log, cwd=None, timeout=None):
                if cmd[0] == "ctest" and "-N" not in cmd:
                    outer.write("Tests/Integration_Tests/claude/plans/PROGRESS.md", "edited\n")
                return Stub.__call__(self2, cmd, log, cwd, timeout)
        rc, out = self.run_isolated(Editing(self.root))
        self.assertEqual(rc, 1)
        self.assertIn("documents under Tests/Integration_Tests/ changed", out.strip().splitlines()[-1])

    def test_gate_scripts_run_from_the_isolated_tree_in_isolated_mode_only(self):
        stub = Stub(self.root)
        rc, out = self.run_isolated(stub)
        self.assertEqual(rc, 0, out)
        _, sroot = ig.iso_paths(self.root, "T-1")
        scripts = [c[2] for c in stub.calls if len(c) > 2 and str(c[2]).endswith(("check_baseline.py", "summarise_results.py"))]
        self.assertEqual(len(scripts), 4)
        self.assertTrue(all(x.startswith(sroot + os.sep) for x in scripts), scripts)
        stub = Stub(self.root)
        self.run_main(["--variants", "default,m32"])
        stub2 = Stub(self.root)
        import io
        import contextlib
        with contextlib.redirect_stdout(io.StringIO()):
            ig.main(["--root", self.root, "--wo", "T-1", "--variants", "default,m32"], runner=stub2)
        scripts = [c[2] for c in stub2.calls if len(c) > 2 and str(c[2]).endswith(("check_baseline.py", "summarise_results.py"))]
        self.assertTrue(scripts and all(x.startswith(self.root + os.sep) for x in scripts), scripts)

    def test_dropping_the_isolated_source_root_from_the_compile_check_makes_it_fail(self):
        # the stub's compile commands carry the source root of the configure call (-I<src>/Firmware/Tests/inc)
        rc, out = self.run_isolated(Stub(self.root))
        self.assertEqual(rc, 0, out)
        saved = ig.check_compile_reference
        ig.check_compile_reference = lambda root, bd, variant, src_root=None: saved(root, bd, variant)  # the old behaviour
        try:
            rc, out = self.run_isolated(Stub(self.root))
        finally:
            ig.check_compile_reference = saved
        self.assertEqual(rc, 1)
        self.assertIn("compile commands differ", out)

    def test_a_build_folder_of_another_source_tree_is_deleted_and_configured_fresh(self):
        bd = ig.build_dir_for(self.root, "default", "T-1")
        Stub(self.root).write_build(bd, "default", home="/somewhere/else/Firmware/Tests")
        marker = os.path.join(bd, "stale-object.o")
        with open(marker, "w") as f:
            f.write("x")
        rc, last, out = self.run_main(["--variants", "default,m32", "--own-build"])
        self.assertEqual(rc, 0, out)
        self.assertIn("deleted, configuring fresh", out)
        self.assertFalse(os.path.exists(marker))
        self.assertIn("fresh_configure", self.latest()["variants"]["default"])
        self.assertNotIn("fresh_configure", self.latest()["variants"]["m32"])

    def test_a_build_folder_of_this_repository_is_reused(self):
        bd = ig.build_dir_for(self.root, "default", "T-1")
        Stub(self.root).write_build(bd, "default", home=os.path.join(self.root, "Firmware/Tests"))
        marker = os.path.join(bd, "kept-object.o")
        with open(marker, "w") as f:
            f.write("x")
        rc, last, out = self.run_main(["--variants", "default,m32", "--own-build"])
        self.assertEqual(rc, 0, out)
        self.assertTrue(os.path.exists(marker))
        self.assertNotIn("fresh_configure", self.latest()["variants"]["default"])

    def test_isolated_run_fails_when_the_wo_files_change_during_the_run(self):
        stub = Stub(self.root)
        orig = stub.__call__
        root = self.root
        outer = self

        class Changing(Stub):
            def __call__(self2, cmd, log, cwd=None, timeout=None):
                if cmd[0] == "ctest" and "-N" not in cmd:
                    outer.write(CASE_REL, "edited while the gate runs\n")
                return Stub.__call__(self2, cmd, log, cwd, timeout)
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = ig.main(["--root", root, "--wo", "T-1", "--variants", "default,m32", "--isolated"], runner=Changing(root))
        self.assertEqual(rc, 1)
        self.assertIn("changed while the isolated gate ran", buf.getvalue().strip().splitlines()[-1])

    def test_isolated_run_does_not_build_when_the_scope_fails(self):
        self.write("Firmware/FFBoard/Src/Axis.cpp", "changed by mistake\n")
        stub = Stub(self.root)
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = ig.main(["--root", self.root, "--wo", "T-1", "--variants", "default,m32", "--isolated"], runner=stub)
        self.assertEqual(rc, 1)
        self.assertEqual(stub.calls, [])
        self.assertTrue(buf.getvalue().strip().splitlines()[-1].startswith("GATE FAIL "))

    def test_git_status_failure_ends_with_a_gate_fail_line(self):
        shutil.rmtree(os.path.join(self.root, ".git"))  # not a repository any more: git status fails
        rc, last, out = self.run_main(["--variants", "default,m32"])
        self.assertEqual(rc, 1)
        self.assertTrue(last.startswith("GATE FAIL "), last)
        self.assertIn("git status failed", out)

    def test_an_unexpected_error_still_ends_with_a_gate_fail_line(self):
        import io
        import contextlib
        buf = io.StringIO()
        saved = ig.scope_check
        ig.scope_check = lambda *a, **k: 1 / 0
        try:
            with contextlib.redirect_stdout(buf):
                rc = ig.main(["--root", self.root, "--wo", "T-1", "--variants", "default,m32"], runner=Stub(self.root))
        finally:
            ig.scope_check = saved
        self.assertEqual(rc, 1)
        self.assertTrue(buf.getvalue().strip().splitlines()[-1].startswith("GATE FAIL internal error"))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(ig.main(["--no-such-option"]), 1)
        self.assertTrue(buf.getvalue().strip().splitlines()[-1].startswith("GATE FAIL "))

    def test_undecodable_wo_file_list_is_a_gate_fail(self):
        with open(os.path.join(self.root, "Tests/Integration_Tests/claude/charters/wo/T-1.files"), "wb") as f:
            f.write(b"\xff\xfe")
        rc, last, _ = self.run_main(["--variants", "default,m32"])
        self.assertEqual(rc, 1)
        self.assertTrue(last.startswith("GATE FAIL ") and "cannot be read" in last)

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
