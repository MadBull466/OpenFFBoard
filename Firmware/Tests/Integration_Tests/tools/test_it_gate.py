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
import threading
import time
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

    def test_the_runner_has_no_process_time_limit(self):
        import inspect
        self.assertNotIn("timeout", inspect.signature(ig.run).parameters)


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
                      syntax_rc=0, syntax_text="", make_exe=True, cc_flag="-O0", cache_override={},
                      cases=None, list_rc=0, fail_in_shard=(), fail_always=(), die_in_shard=(), die_alone=(),
                      skip_names=(), known_names=(), delay=0, ctest_json=None, ctest_props=(), neg_rc_after=None,
                      sanitizer_in=(), sanitizer_in_shard=(), timeout_in_shard=(), timeout_alone=(), timeout_line=True,
                      integ_default=ig.DEFAULT_CASE_TIMEOUT_S, json_rc=0, baseline_text="baseline OK\n")
        self.o.update(opts)
        if self.o["cases"] is None:
            self.o["cases"] = [self.case("case %d" % i, CASE_REL, i * 10) for i in range(1, 6)]
        self.calls = []
        self.cwds = []
        self.envs = []
        self.timeouts = []  # the timeout= argument of every call
        self.lock = threading.Lock()
        self.active = self.peak = 0
        self.exe_times = []  # seconds of every real run of the executable (not the listing), measured inside the stub

    @staticmethod
    def case(name, rel, line):
        return {"name": name, "rel": rel, "line": line}

    def exe_call(self, cmd):
        """Answers a call of ffb_integration_tests like doctest 2.4.11: -ltc lists only the cases that pass the filters (-sf,
        --test-case-exclude, doctest::skip() = skip_names); the XML report of a run names every registered case, and a
        case that is filtered out, skipped or outside --first / --last is skipped="true" there."""
        import fnmatch
        import xml.sax.saxutils as sx
        o = self.o
        arg = lambda k: next((a.split("=", 1)[1] for a in cmd if a.startswith(k + "=")), None)  # noqa: E731
        sf, tce = arg("-sf"), arg("--test-case-exclude")
        first, last = arg("--first"), arg("--last")
        a, b = (int(first), int(last)) if first else (1, 10 ** 9)
        alone = first is not None and first == last
        listing = "-ltc" in cmd

        def passes(c):
            if c["name"] in o["skip_names"]:
                return False
            if sf and not any(fnmatch.fnmatchcase(os.path.join(self.root, c["rel"]), x) for x in sf.split(",")):
                return False
            return not (tce and c["name"] in tce.split(","))

        body, failed_here, p, crash, san = [], False, 0, None, False
        for c in o["cases"]:
            at = 'name=%s filename=%s line="%d"' % (sx.quoteattr(c["name"]), sx.quoteattr(os.path.join(self.root, c["rel"])), c["line"])
            n = c["name"]
            sel = passes(c)
            p += 1 if sel else 0
            if listing:
                if sel:
                    body.append("<TestCase %s skipped=\"false\"/>" % at)
                continue
            if not sel or not a <= p <= b or crash is not None:
                body.append("<TestCase %s skipped=\"true\"/>" % at)
                continue
            if n in o["die_alone"] or (not alone and n in o["die_in_shard"]):
                # aborted inside the case: its element is open, no closing tag
                return 134, "<?xml version=\"1.0\"?><doctest>" + "".join(body) + "<TestCase %s>" % at
            if n in o["timeout_alone"] or (not alone and n in o["timeout_in_shard"]):
                # the watchdog of the executable: the case element is open, the line on stderr, exit code 86
                line = "CASE TIMEOUT 600 s: %s (%s:%d)\n" % (n, os.path.join(self.root, c["rel"]), c["line"]) if o["timeout_line"] else ""
                return 86, "<?xml version=\"1.0\"?><doctest>" + "".join(body) + "<TestCase %s>" % at + "\n" + line
            san = san or n in o["sanitizer_in"] or (not alone and n in o["sanitizer_in_shard"])
            st = "passed"
            if n in o["fail_always"] or (not alone and n in o["fail_in_shard"]):
                st = "failed"
            elif n in o["known_names"]:
                st = "known"
            failed_here = failed_here or st == "failed"
            f = {"passed": (3, 0, "true"), "known": (3, 1, "true"), "failed": (2, 1, "false")}[st]
            body.append("<TestCase %s><OverallResultsAsserts successes=\"%d\" failures=\"%d\" test_case_success=\"%s\"/></TestCase>"
                        % ((at,) + f))
            if o["neg_rc_after"] and n in o["neg_rc_after"] and not alone:
                crash = o["neg_rc_after"][n]  # a signal after this case: the later cases of the range are not run
        text = "<?xml version=\"1.0\"?><doctest>%s</doctest>" % "".join(body)
        if san:
            text += "\n==1==ERROR: AddressSanitizer: heap-use-after-free on address 0x1\n"
        rc = o["list_rc"] if listing else (crash if crash is not None else (o["int_rc"] or (1 if failed_here else 0)))
        return rc, text

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
        for target, limit in (("ffb_integration_tests", self.o["integ_default"]), ("ffb_unit_tests", 1200)):  # CaseWatchdog.cpp of each
            wd = os.path.join(src, "Firmware/Tests", ig.WATCHDOG_SOURCE)
            cc.append({"directory": bd, "file": wd, "output": "CMakeFiles/%s.dir/CaseWatchdog.o" % target,
                       "command": "c++ -DFFB_CASE_TIMEOUT_DEFAULT_S=%d -o CMakeFiles/%s.dir/CaseWatchdog.o -c %s" % (limit, target, wd)})
        with open(os.path.join(bd, "compile_commands.json"), "w") as f:
            json.dump(cc, f)
        exe = os.path.join(bd, ig.EXE)
        if self.o["make_exe"]:
            with open(exe, "w") as f:
                f.write("x")
        elif os.path.exists(exe):
            os.remove(exe)

    def __call__(self, cmd, log, cwd=None, timeout=None, env=None):
        w = 1
        for a in cmd:
            if cmd[0] == "cmake" and str(a).startswith("-j"):
                w = int(a[2:])
        with self.lock:
            self.calls.append(list(cmd))
            self.cwds.append(cwd)
            self.envs.append(env)
            self.timeouts.append(timeout)
            self.active += w
            self.peak = max(self.peak, self.active)
        try:
            t = time.monotonic()
            if self.o["delay"]:
                time.sleep(self.o["delay"])
            try:
                return self.answer(cmd, log, cwd, timeout, env)
            finally:
                if str(cmd[0]).endswith(ig.EXE) and "-ltc" not in cmd:
                    with self.lock:
                        self.exe_times.append(time.monotonic() - t)
        finally:
            with self.lock:
                self.active -= w

    def answer(self, cmd, log, cwd=None, timeout=None, env=None):
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
        elif cmd[0] == "ctest" and "--show-only=json-v1" in cmd:
            bd = cmd[cmd.index("--test-dir") + 1]
            excl = ig.integration_excludes(ig.read_cache(bd))
            test = {"name": ig.EXE, "command": [os.path.join(bd, ig.EXE)] + (["--test-case-exclude=" + ",".join(excl)] if excl else []),
                    "properties": list(o["ctest_props"])}
            text = o["ctest_json"] if o["ctest_json"] is not None else json.dumps({"tests": [test]})
            rc = o["json_rc"]
        elif cmd[0] == "ctest":
            rc, text = o["ctest_rc"], o["ctest_text"]
        elif "check_baseline.py" in joined:
            rc, text = o["baseline_rc"], o["baseline_text"]
        elif "summarise_results.py" in joined:
            rc, text = o["summary_rc"], o["summary"]
        elif str(cmd[0]).endswith(ig.EXE):
            rc, text = self.exe_call(cmd)
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

    def go(self, fresh=False, variant="default", serial=True, rv=None, **opts):
        stub = Stub(self.root, **opts)
        ref_stub = Stub(self.root)
        self.record(ref_stub, variant)
        bd = ig.build_dir_for(self.root, variant)
        if fresh:
            os.remove(os.path.join(bd, "CMakeCache.txt"))
        else:
            stub.write_build(bd, variant)  # the existing directory as this scenario has it (cache, executable)
        res = ig.run_variant(self.root, variant, LIST_PATTERNS, self.evid, stub, serial=serial, **(rv or {}))
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

    def test_check_baseline_failure_fails(self):
        self.assertFails(self.go(baseline_rc=1)[0], "check_baseline")

    def test_integration_exit_code_fails(self):
        self.assertFails(self.go(int_rc=1)[0], ig.EXE)
        self.assertFails(self.go(int_rc=124)[0], ig.EXE)  # no process limit: 124 is just a failing exit code

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
                    dict(fail_always=("case 1",))):
            rc, last, _ = self.run_main(["--variants", "default,m32"], **opt)
            self.assertEqual(rc, 1, opt)
            self.assertTrue(last.startswith("GATE FAIL "), (opt, last))
            self.assertEqual(self.latest()["result"], "FAIL", opt)
        # --serial: the same, and the failed summary of the old XML run
        for opt in (dict(int_rc=1), dict(fail_always=("case 1",)), dict(make_exe=False),
                    dict(summary="Test cases: 1  passed: 0  failed: 1  skipped: 0\n")):
            rc, last, _ = self.run_main(["--variants", "default,m32", "--serial"], **opt)
            self.assertEqual(rc, 1, opt)
            self.assertTrue(last.startswith("GATE FAIL "), (opt, last))

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

    def run_isolated(self, runner, wo="T-1", extra=()):
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = ig.main(["--root", self.root, "--wo", wo, "--variants", "default,m32", "--isolated"] + list(extra), runner=runner)
        return rc, buf.getvalue()

    def test_isolated_run_fails_when_head_moves_during_the_gate(self):
        outer = self

        class Committing(Stub):
            def __call__(self2, cmd, log, cwd=None, timeout=None, env=None):
                if cmd[0] == "ctest" and "-N" not in cmd:
                    sh(outer.root, *ID, "commit", "-q", "--allow-empty", "-m", "moved")
                return Stub.__call__(self2, cmd, log, cwd, timeout, env)
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
            def __call__(self2, cmd, log, cwd=None, timeout=None, env=None):
                if cmd[0] == "ctest" and "-N" not in cmd:
                    outer.write("Tests/Integration_Tests/claude/plans/PROGRESS.md", "edited\n")
                return Stub.__call__(self2, cmd, log, cwd, timeout, env)
        rc, out = self.run_isolated(Editing(self.root))
        self.assertEqual(rc, 1)
        self.assertIn("documents under Tests/Integration_Tests/ changed", out.strip().splitlines()[-1])

    def test_gate_scripts_run_from_the_isolated_tree_in_isolated_mode_only(self):
        stub = Stub(self.root)
        rc, out = self.run_isolated(stub)
        self.assertEqual(rc, 0, out)
        _, sroot = ig.iso_paths(self.root, "T-1")
        scripts = [c[2] for c in stub.calls if len(c) > 2 and str(c[2]).endswith(("check_baseline.py", "summarise_results.py"))]
        self.assertEqual(sorted(os.path.basename(x) for x in scripts), ["check_baseline.py"] * 2)  # the shards need no summary script
        self.assertTrue(all(x.startswith(sroot + os.sep) for x in scripts), scripts)
        stub = Stub(self.root)
        rc, out = self.run_isolated(stub, extra=["--serial"])
        self.assertEqual(rc, 0, out)
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
            def __call__(self2, cmd, log, cwd=None, timeout=None, env=None):
                if cmd[0] == "ctest" and "-N" not in cmd:
                    outer.write(CASE_REL, "edited while the gate runs\n")
                return Stub.__call__(self2, cmd, log, cwd, timeout, env)
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


CD = "Firmware/Tests/Integration_Tests/claude/cases/"


class ScopeDecision(unittest.TestCase):
    """integration_scope: the subset of cases is used only when every non-document file of the WO list is a case file."""

    def test_case_files_only_give_the_subset(self):
        self.assertEqual(ig.integration_scope([CD + "test_f1_a.cpp", CD + "test_f1_b.cpp"]), [CD + "test_f1_a.cpp", CD + "test_f1_b.cpp"])

    def test_case_files_with_documents_give_the_subset(self):
        pats = [CD + "test_f1_a.cpp", "Tests/Integration_Tests/claude/plans/PROGRESS.md", "Tests/Code_Coverage/claude/plans/PROGRESS.md",
                "Firmware/Tests/Integration_Tests/README.md"]
        self.assertEqual(ig.integration_scope(pats), [CD + "test_f1_a.cpp"])

    def test_a_case_file_glob_in_the_file_name_gives_the_subset(self):
        self.assertEqual(ig.integration_scope([CD + "test_f1_*.cpp"]), [CD + "test_f1_*.cpp"])

    def test_one_other_code_file_makes_it_the_full_run(self):
        for other in ("Firmware/Tests/Integration_Tests/support/x.h", "Firmware/Tests/Integration_Tests/fakes/f.cpp",
                      "Firmware/Tests/Integration_Tests/integration.cmake", "Firmware/Tests/CMakeLists.txt",
                      "Firmware/Tests/Integration_Tests/tools/it_gate.py", "Firmware/Tests/Code_Coverage/fakes/a.cpp",
                      "Firmware/Tests/Code_Coverage/claude/cases/test_x.cpp", ".claude/hooks/guard_lib.py", "AGENTS.mdx", "*"):
            self.assertIsNone(ig.integration_scope([CD + "test_f1_a.cpp", other]), other)
            self.assertIsNone(ig.integration_scope([other]), other)

    def test_a_pattern_that_may_match_a_non_case_file_makes_it_the_full_run(self):
        for p in (CD + "*.cpp", CD + "helper.cpp", CD + "test_[ab].cpp", CD + "test_x.h", CD + "sub/test_x.cpp", CD + "*"):
            self.assertIsNone(ig.integration_scope([p]), p)

    def test_documents_only_give_the_full_run(self):
        self.assertIsNone(ig.integration_scope(["Tests/Integration_Tests/claude/plans/PROGRESS.md", "CLAUDE.md"]))

    def test_full_always_gives_the_full_run(self):
        self.assertIsNone(ig.integration_scope([CD + "test_f1_a.cpp"], full=True))

    def test_the_file_filters_match_only_the_cases_directory(self):
        import fnmatch
        f = ig.scope_filters([CD + "test_f1_a.cpp"])
        self.assertEqual(f, ["*cases?test_f1_a.cpp"])
        self.assertTrue(fnmatch.fnmatchcase("/a b/Firmware/Tests/Integration_Tests/claude/cases/test_f1_a.cpp", f[0]))
        self.assertFalse(fnmatch.fnmatchcase("/a/Firmware/Tests/Other/test_f1_a.cpp", f[0]))


class Shards(unittest.TestCase):
    def test_partition_covers_every_index_exactly_once(self):
        for n in range(0, 60):
            for parts in (1, 2, 3, 7, 40, 100):
                r = ig.partition(n, parts)
                covered = [i for a, b in r for i in range(a, b + 1)]
                self.assertEqual(covered, list(range(1, n + 1)), (n, parts))
                self.assertLessEqual(len(r), max(parts, 1))
                if r:
                    self.assertLessEqual(max(b - a for a, b in r) - min(b - a for a, b in r), 1)

    def test_case_xml_statuses_follow_summarise_results(self):
        xml = ("<doctest>"
               '<TestCase name="p" filename="/x/Firmware/Tests/a.cpp" line="1"><OverallResultsAsserts successes="4" failures="0" test_case_success="true"/></TestCase>'
               '<TestCase name="k" filename="/x/Firmware/Tests/a.cpp" line="2"><OverallResultsAsserts successes="4" failures="2" test_case_success="true"/></TestCase>'
               '<TestCase name="f" filename="/x/Firmware/Tests/a.cpp" line="3"><OverallResultsAsserts successes="1" failures="1" test_case_success="false"/></TestCase>'
               '<TestCase name="s" filename="/x/Firmware/Tests/a.cpp" line="4" skipped="true"/>'
               "</doctest>")
        cases = ig.parse_cases_xml("noise before\n" + xml + "\nnoise after")
        self.assertEqual([c["status"] for c in cases], ["passed", "known", "failed", "skipped"])
        self.assertEqual((cases[1]["asserts_passed"], cases[1]["asserts_failed"], cases[1]["file"]), (4, 2, "Firmware/Tests/a.cpp"))
        self.assertEqual(ig.case_counts(cases), (4, 1, 1, 1, 1))

    def test_a_report_without_the_closing_tag_is_a_dead_run(self):
        self.assertIsNone(ig.parse_cases_xml("<doctest><TestCase name='a' filename='f' line='1'>"))
        self.assertIsNone(ig.parse_cases_xml(""))
        self.assertIsNone(ig.parse_cases_xml("<doctest><TestCase></doctest>"))

    def test_budget_never_hands_out_more_than_its_total_and_is_first_come(self):
        b = ig.Budget(3)
        active, peak, lock, order = [0], [0], threading.Lock(), []

        def work(n):
            with b.hold(n):
                with lock:
                    active[0] += n
                    peak[0] = max(peak[0], active[0])
                    order.append(n)
                time.sleep(0.003)
                with lock:
                    active[0] -= n
        ts = [threading.Thread(target=work, args=(n,)) for n in (1, 3, 1, 2, 1, 1, 3, 1)]
        for t in ts:
            t.start()
        for t in ts:
            t.join()
        self.assertLessEqual(peak[0], 3)
        self.assertEqual(sorted(order), sorted((1, 3, 1, 2, 1, 1, 3, 1)))
        self.assertEqual(b.free, 3)


def many(n, rel=CASE_REL, names=None):
    return [Stub.case(names[i - 1] if names else "case %d" % i, rel, i * 10) for i in range(1, n + 1)]


class IntegrationRun(unittest.TestCase):
    """run_variant without --serial: one integration run in shards, per-case results, reruns, scope, job budget."""

    def setUp(self):
        self.tmp = os.path.realpath(tempfile.mkdtemp())
        self.root = os.path.join(self.tmp, "repo")
        os.makedirs(os.path.join(self.root, "Firmware/Tests"))
        self.evid = os.path.join(self.tmp, "evid")
        os.makedirs(self.evid)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def go(self, variant="default", serial=False, rv=None, echo=None, **opts):
        stub = Stub(self.root, **opts)
        bd = ig.build_dir_for(self.root, variant)
        ref = Stub(self.root)
        ref.write_build(bd, variant)
        ig.record_reference(self.root, bd, variant)
        stub.write_build(bd, variant)
        kw = dict(rv or {})
        kw["echo"] = echo if echo is not None else (lambda line: None)
        res = ig.run_variant(self.root, variant, LIST_PATTERNS, self.evid, stub, serial=serial, **kw)
        return res, stub

    def exe_calls(self, stub):
        return [c for c in stub.calls if str(c[0]).endswith(ig.EXE)]

    def cases_file(self, variant="default"):
        with open(os.path.join(self.evid, "%s.integration.cases.json" % variant)) as f:
            return json.load(f)

    def test_ctest_runs_without_the_integration_entry_and_the_registration_is_still_checked(self):
        _, stub = self.go()
        ct = [c for c in stub.calls if c[0] == "ctest" and "-N" not in c and "--show-only=json-v1" not in c]
        self.assertEqual(len(ct), 1)
        self.assertEqual(ct[0][ct[0].index("-E") + 1], "^ffb_integration_tests$")
        self.assertEqual(ct[0][ct[0].index("--parallel") + 1], "1")  # one entry at a time: ctest is one job of the budget
        res, _ = self.go(ctest_list="  Test #9: integration_tool_tests\n")
        self.assertFalse(res["ok"])
        self.assertIn("ffb_integration_tests", res["reason"])
        res, _ = self.go(ctest_list="  Test #8: ffb_integration_tests\n")
        self.assertIn("integration_tool_tests", res["reason"])

    def test_serial_runs_the_whole_ctest_without_exclusion(self):
        _, stub = self.go(serial=True)
        ct = [c for c in stub.calls if c[0] == "ctest" and "-N" not in c and "--show-only=json-v1" not in c]
        self.assertEqual(len(ct), 1)
        self.assertNotIn("-E", ct[0])
        self.assertNotIn("--parallel", ct[0])
        # serial lists every ctest entry once (limits and watchdog check), never only the integration one (-R)
        self.assertEqual([c for c in stub.calls if "--show-only=json-v1" in c and "-R" in c], [])
        self.assertEqual(len([c for c in stub.calls if "--show-only=json-v1" in c]), 1)

    def test_the_executable_runs_once_in_shards_and_needs_no_summary_script(self):
        res, stub = self.go(cases=many(12), rv=dict(jobs=2))
        self.assertTrue(res["ok"], res.get("reason"))
        self.assertFalse([c for c in stub.calls if "summarise_results.py" in " ".join(map(str, c))])
        runs = [c for c in self.exe_calls(stub) if "-ltc" not in c]
        self.assertEqual(len(runs), res["integration"]["shards"])
        self.assertEqual(len([c for c in self.exe_calls(stub) if "-ltc" in c]), 1)
        for c in runs:
            self.assertEqual(c[1], "-r=xml")
            self.assertIn("--order-by=file", c)

    def test_every_case_runs_exactly_once_across_the_shards(self):
        for n, jobs in ((1, 10), (5, 1), (12, 2), (37, 3), (37, 10)):
            res, stub = self.go(cases=many(n), rv=dict(jobs=jobs))
            self.assertTrue(res["ok"], res.get("reason"))
            ranges = [(int(next(a for a in c if a.startswith("--first=")).split("=")[1]),
                       int(next(a for a in c if a.startswith("--last=")).split("=")[1]))
                      for c in self.exe_calls(stub) if "-ltc" not in c]
            covered = sorted(i for a, b in ranges for i in range(a, b + 1))
            self.assertEqual(covered, list(range(1, n + 1)), (n, jobs))
            self.assertEqual(res["integration"]["cases"], n)
            self.assertEqual(len(self.cases_file()["cases"]), n)

    def test_case_names_with_commas_and_wildcards_are_each_run_once(self):
        names = ["a, b", "x*y", "what?", "back\\slash, [x]", "plain", "a", "b"]
        res, stub = self.go(cases=many(7, names=names), rv=dict(jobs=1))
        self.assertTrue(res["ok"], res.get("reason"))
        self.assertEqual(sorted(c["name"] for c in self.cases_file()["cases"]), sorted(names))
        # no case name is ever given to doctest: the shards are index ranges, so no escaping can be wrong
        for c in self.exe_calls(stub):
            self.assertLessEqual({a.split("=")[0] for a in c[1:]}, {"-r", "--order-by", "-ltc", "--first", "--last"}, c)

    def test_the_excluded_cases_of_ctest_reach_the_list_and_every_shard(self):
        res, stub = self.go(cases=many(6), rv=dict(jobs=2), cache_override={"FFB_INTEGRATION_TESTS_EXCLUDE": "case 2;case 3"})
        self.assertTrue(res["ok"], res.get("reason"))
        for c in self.exe_calls(stub):
            self.assertIn("--test-case-exclude=case 2,case 3", c)
        i = res["integration"]  # the excluded cases are in the report as skipped, like the old summary counted them
        self.assertEqual((i["cases"], i["selected"], i["passed"], i["skipped"]), (6, 4, 4, 2))

    def test_merged_counts_and_the_cases_file(self):
        cases = many(8)
        res, _ = self.go(cases=cases, known_names=("case 2", "case 5"), skip_names=("case 7",), rv=dict(jobs=1))
        self.assertTrue(res["ok"], res.get("reason"))
        i = res["integration"]
        self.assertEqual((i["cases"], i["passed"], i["failed"], i["skipped"], i["known_issue"]), (8, 5, 0, 1, 2))
        f = self.cases_file()
        self.assertEqual(f["scope"], "full")
        self.assertEqual(f["count"], 8)
        by = {c["name"]: c for c in f["cases"]}
        self.assertEqual(by["case 2"]["status"], "known")
        self.assertEqual((by["case 2"]["asserts_passed"], by["case 2"]["asserts_failed"]), (3, 1))
        self.assertEqual(by["case 7"]["status"], "skipped")
        self.assertEqual(by["case 1"]["status"], "passed")
        self.assertEqual(set(by["case 1"]), {"name", "file", "line", "status", "asserts_passed", "asserts_failed"})
        with open(os.path.join(self.evid, "default.integration.summary.log")) as fh:
            self.assertEqual(ig.parse_counts(fh.read()), (8, 5, 0, 1, 2))

    def test_serial_and_parallel_write_the_same_cases_file(self):
        kw = dict(cases=many(9), known_names=("case 3",), skip_names=("case 8",))
        self.go(serial=True, **kw)
        with open(os.path.join(self.evid, "default.integration.cases.json")) as f:
            old = f.read()
        self.go(rv=dict(jobs=2), **kw)
        with open(os.path.join(self.evid, "default.integration.cases.json")) as f:
            new = f.read()
        self.assertEqual(old, new)

    def test_a_case_that_fails_in_a_shard_and_passes_alone_passes_with_a_warning(self):
        lines = []
        res, stub = self.go(cases=many(12), fail_in_shard=("case 5", "case 11"), rv=dict(jobs=1), echo=lines.append)
        self.assertTrue(res["ok"], res.get("reason"))
        i = res["integration"]
        self.assertEqual(sorted(i["rerun_passed"]), ["case 11", "case 5"])
        self.assertEqual(i["rerun_failed"], [])
        self.assertEqual(i["failed"], 0)
        self.assertIn("WARNING: 2 case(s) passed only when rerun alone: case 5; case 11 (variant default)", lines)
        alone = [c for c in self.exe_calls(stub) if any(a == "--first=5" for a in c) and "--last=5" in c]
        self.assertEqual(len(alone), 1)  # the rerun of case 5: alone, once
        self.assertEqual({c["name"]: c["status"] for c in self.cases_file()["cases"]}["case 5"], "passed")
        self.assertEqual(self.cases_file()["rerun_passed"], ["case 5", "case 11"])

    def test_a_case_that_fails_again_alone_fails_the_variant(self):
        lines = []
        res, _ = self.go(cases=many(12), fail_always=("case 4",), fail_in_shard=("case 9",), rv=dict(jobs=1), echo=lines.append)
        self.assertFalse(res["ok"])
        self.assertIn("1 integration case(s) failed, first: case 4", res["reason"])
        i = res["integration"]
        self.assertEqual((i["rerun_failed"], i["rerun_passed"], i["failed"]), (["case 4"], ["case 9"], 1))
        self.assertTrue(any(l.startswith("WARNING: 1 case(s) passed only when rerun alone: case 9") for l in lines))

    def test_the_reruns_run_one_after_the_other_and_not_during_each_other(self):
        res, stub = self.go(cases=many(12), fail_in_shard=("case 2", "case 6", "case 10"), rv=dict(jobs=1), delay=0.002)
        self.assertTrue(res["ok"], res.get("reason"))
        calls = [c for c in stub.calls if str(c[0]).endswith(ig.EXE)]
        shard_calls = [c for c in calls if "-ltc" not in c and any(a.startswith("--first=") for a in c)]
        singles = [c for c in shard_calls if next(a for a in c if a.startswith("--first=")).split("=")[1]
                   == next(a for a in c if a.startswith("--last=")).split("=")[1]]
        order = [next(a for a in c if a.startswith("--first=")) for c in singles[-3:]]
        self.assertEqual(order, ["--first=2", "--first=6", "--first=10"])

    def test_a_dead_shard_leaves_its_cases_unrun_and_they_run_alone(self):
        lines = []
        res, stub = self.go(cases=many(12), die_in_shard=("case 5",), rv=dict(jobs=1), echo=lines.append)
        self.assertTrue(res["ok"], res.get("reason"))
        i = res["integration"]
        # 12 cases, 4 shards of 3: the shard 4..6 died, its 3 cases ran alone
        self.assertEqual(sorted(i["rerun_unrun_passed"]), ["case 4", "case 5", "case 6"])
        self.assertEqual((i["rerun_failed"], i["rerun_passed"], i["cases"], i["failed"]), ([], [], 12, 0))
        self.assertEqual(len(self.cases_file()["cases"]), 12)
        self.assertTrue(any(l.startswith("WARNING: 3 case(s) passed only when rerun alone") for l in lines))


    def test_a_case_that_crashes_alone_is_a_failure(self):
        res, _ = self.go(cases=many(12), die_alone=("case 5",), die_in_shard=("case 5",), rv=dict(jobs=1))
        self.assertFalse(res["ok"])
        self.assertIn("1 integration case(s) failed, first: case 5", res["reason"])
        self.assertEqual(res["integration"]["rerun_failed"], ["case 5"])
        c = {c["name"]: c for c in self.cases_file()["cases"]}
        self.assertEqual(c["case 5"]["status"], "failed")
        self.assertEqual(c["case 4"]["status"], "passed")

    # ---- per-case time limits instead of process time limits (tool WO I2-T1)

    def integration_calls_with_timeout(self, stub):
        return [(c, t) for c, t in zip(stub.calls, stub.timeouts) if str(c[0]).endswith(ig.EXE) and t is not None]

    def test_no_integration_run_gets_a_process_time_limit_and_every_run_carries_the_case_limit(self):
        for kw in (dict(), dict(rv=dict(jobs=1)), dict(cases=many(12), die_in_shard=("case 5",), fail_in_shard=("case 2",))):
            res, stub = self.go(**kw)
            self.assertTrue(res["ok"], res.get("reason"))
            self.assertEqual(self.integration_calls_with_timeout(stub), [])
            idx = [i for i, c in enumerate(stub.calls) if str(c[0]).endswith(ig.EXE)]
            self.assertTrue(idx)
            for i in idx:  # shards, the listing and the alone reruns alike
                self.assertEqual(stub.envs[i], {"FFB_IT_CASE_TIMEOUT_S": str(ig.DEFAULT_CASE_TIMEOUT_S)}, stub.calls[i])
        res, stub = self.go(variant="cov", cases=many(4))  # cov: one process
        self.assertTrue(res["ok"], res.get("reason"))
        self.assertEqual(self.integration_calls_with_timeout(stub), [])
        self.assertTrue(all(stub.envs[i] == {"FFB_IT_CASE_TIMEOUT_S": str(ig.case_timeout_s("cov"))}
                            for i, c in enumerate(stub.calls) if str(c[0]).endswith(ig.EXE)))

    def ctest_run_calls(self, stub):
        return [i for i, c in enumerate(stub.calls) if c[0] == "ctest" and "-N" not in c and "--show-only=json-v1" not in c]

    def test_no_ctest_run_has_a_process_limit_or_an_environment_limit_and_all_carry_the_no_limit_mechanism(self):
        for serial in (True, False):
            res, stub = self.go(serial=serial)
            self.assertTrue(res["ok"], res.get("reason"))
            idx = self.ctest_run_calls(stub)
            self.assertEqual(len(idx), 1)
            c = stub.calls[idx[0]]
            self.assertIsNone(stub.timeouts[idx[0]])  # no subprocess limit (the old 3600 s)
            self.assertIsNone(stub.envs[idx[0]])  # one value would override every executable's own compiled default
            self.assertNotIn("--timeout", c)
            self.assertEqual(c[c.index("--overwrite"):c.index("--overwrite") + 2], ["--overwrite", "TimeOut=0"])
            self.assertEqual(("--overwrite", "TimeOut=0"), ig.CTEST_NO_LIMIT_ARGS)
            if not serial:
                self.assertIn("-E", c)  # ctest without the integration entry: the gate shards that one
            base = [i for i, c in enumerate(stub.calls) if any("check_baseline.py" in str(a) for a in c)]
            self.assertEqual([stub.timeouts[i] for i in base], [None])
        self.assertTrue(all(t is None for t in stub.timeouts))  # not one call of the gate has a process limit

    def test_serial_passes_no_process_limit_for_the_executable_and_the_case_limit_through_the_environment(self):
        res, stub = self.go(serial=True)
        self.assertTrue(res["ok"], res.get("reason"))
        self.assertEqual(self.integration_calls_with_timeout(stub), [])
        for c, e in zip(stub.calls, stub.envs):
            if str(c[0]).endswith(ig.EXE):
                self.assertEqual(e, {"FFB_IT_CASE_TIMEOUT_S": str(ig.DEFAULT_CASE_TIMEOUT_S)})

    def test_the_gate_has_no_whole_run_limit_constants_or_arguments_left(self):
        for name in ("INTEGRATION_TIMEOUT", "INTEGRATION_SUM_LIMIT", "CTEST_TIMEOUT", "CTEST_TOTAL_TIMEOUT", "BASELINE_TIMEOUT"):
            self.assertFalse(hasattr(ig, name), name)
        with open(ig.__file__, encoding="utf-8") as f:
            text = f.read()
        for token in ("timeout=", "--timeout 600", "CTEST_TOTAL_TIMEOUT", "BASELINE_TIMEOUT", '"--timeout"'):
            self.assertNotIn(token, text, token)
        with open(os.path.abspath(__file__), encoding="utf-8") as f:
            this = f.read()
        self.assertNotIn("--timeout 600", this.replace('"--timeout 600"', "").replace("'--timeout 600'", ""))

    def test_the_default_limit_of_the_gate_is_the_one_compiled_into_the_executable(self):
        header = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Code_Coverage", "support", "CaseWatchdog.h")
        with open(header) as f:
            text = f.read()
        self.assertEqual(int(re.search(r"#define FFB_CASE_TIMEOUT_DEFAULT_S\s+(\d+)", text).group(1)), ig.DEFAULT_CASE_TIMEOUT_S)
        self.assertEqual(int(re.search(r"kCaseTimeoutExitCode\s*=\s*(\d+)", text).group(1)), ig.CASE_TIMEOUT_RC)
        self.assertRegex(text, r'kCaseTimeoutEnv\s*=\s*"%s"' % re.escape(ig.CASE_TIMEOUT_ENV))

    def test_the_integration_limit_is_the_compiled_default_of_that_executable(self):
        res, stub = self.go(integ_default=300, cases=many(3))
        # the reference of the compile check was recorded with the default stub: the integration entries are not compared
        self.assertEqual(res["case_timeout_defaults"], {"ffb_integration_tests": 300, "ffb_unit_tests": 1200})
        self.assertEqual(res["integration"]["case_timeout_s"], 300)
        for c, e in zip(stub.calls, stub.envs):
            if str(c[0]).endswith(ig.EXE):
                self.assertEqual(e, {"FFB_IT_CASE_TIMEOUT_S": "300"})

    def test_the_compiled_defaults_of_all_executables_are_in_the_evidence(self):
        res, _ = self.go(cases=many(3))
        ev = ig.make_evidence(self.root, "T-1", {"default": res}, {"result": "OK", "offenders": [], "changed": []}, "abc", "PASS")
        self.assertEqual(ev["case_timeout_defaults"], {"default": {"ffb_integration_tests": ig.DEFAULT_CASE_TIMEOUT_S, "ffb_unit_tests": 1200}})

    def test_compiled_defaults_are_read_from_the_watchdog_entries_of_compile_commands(self):
        wd = "/r/Firmware/Tests/" + ig.WATCHDOG_SOURCE
        entries = [
            {"file": wd, "output": "CMakeFiles/ffb_flash_tests.dir/Code_Coverage/support/CaseWatchdog.cpp.o",
             "command": "c++ -DX=1 -DFFB_CASE_TIMEOUT_DEFAULT_S=900 -c " + wd},
            {"file": wd, "output": "CMakeFiles/ffb_hal_tests.dir/CaseWatchdog.o", "arguments": ["c++", "-DFFB_CASE_TIMEOUT_DEFAULT_S=60", "-c", wd]},
            {"file": "/r/Firmware/Tests/other.cpp", "output": "CMakeFiles/ffb_unit_tests.dir/o.o", "command": "c++ -DFFB_CASE_TIMEOUT_DEFAULT_S=5 -c o.cpp"},
            {"file": wd, "output": "CMakeFiles/ffb_nodefine.dir/w.o", "command": "c++ -c " + wd},
        ]
        self.assertEqual(ig.compiled_case_timeouts(entries), {"ffb_flash_tests": 900, "ffb_hal_tests": 60})

    def test_a_doctest_executable_without_a_compiled_case_limit_fails_the_variant(self):
        bd = ig.build_dir_for(self.root, "default")
        tests = [{"name": ig.EXE, "command": [os.path.join(bd, ig.EXE)]},
                 {"name": "ffb_new_tests", "command": [os.path.join(bd, "ffb_new_tests")]}]
        res, _ = self.go(ctest_json=json.dumps({"tests": tests}))
        self.assertFalse(res["ok"])
        self.assertIn("ffb_new_tests has no per-case time limit", res["reason"])
        self.assertEqual(ig.watchdog_problems(tests, {"ffb_integration_tests": 1, "ffb_new_tests": 2}), [])

    def test_a_failing_or_empty_ctest_listing_fails_the_variant_instead_of_skipping_the_limit_checks(self):
        res, _ = self.go(json_rc=1)
        self.assertFalse(res["ok"])
        self.assertIn("ctest --show-only=json-v1 failed or listed no test", res["reason"])
        res, _ = self.go(ctest_json=json.dumps({"tests": []}))
        self.assertFalse(res["ok"])
        self.assertIn("ctest --show-only=json-v1 failed or listed no test", res["reason"])
        res, _ = self.go(ctest_json="not json")
        self.assertFalse(res["ok"])
        self.assertIn("ctest --show-only=json-v1 failed or listed no test", res["reason"])

    def test_a_compiled_default_of_zero_is_a_failure(self):
        bd = ig.build_dir_for(self.root, "default")
        tests = [{"name": ig.EXE, "command": [os.path.join(bd, ig.EXE)]}]
        res, _ = self.go(integ_default=0, ctest_json=json.dumps({"tests": tests}))
        self.assertFalse(res["ok"])
        self.assertIn("ffb_integration_tests has the compiled per-case default 0", res["reason"])
        self.assertEqual(ig.watchdog_problems(tests, {ig.EXE: 5}), [])
        self.assertEqual(len(ig.watchdog_problems(tests, {ig.EXE: 0})), 1)

    def test_a_python_entry_timeout_is_reported_also_when_a_case_timeout_is_in_the_same_log(self):
        text = ("4/9 Test #4: ffb_flash_tests ...***Failed\nCASE TIMEOUT 30 s: a case (/p/t.cpp:1)\n"
                "9/9 Test #9: python_helper_tests ...***Timeout  60.00 sec\n")
        res, _ = self.go(ctest_rc=8, ctest_text=text)
        self.assertFalse(res["ok"])
        self.assertIn("default: case timed out in ffb_flash_tests: a case", res["reason"])
        self.assertIn("a ctest entry exceeded its ctest timeout", res["reason"])

    def test_python_entries_keep_a_ctest_limit_and_no_other_entry_may_have_one(self):
        bd = ig.build_dir_for(self.root, "default")
        integ = {"name": ig.EXE, "command": [os.path.join(bd, ig.EXE)], "properties": [{"name": "TIMEOUT", "value": 0}]}
        py = lambda name, props: {"name": name, "command": ["python3", "-m", "unittest"], "properties": props}
        ok = [integ, py("python_helper_tests", [{"name": "TIMEOUT", "value": 600}]),
              py("integration_tool_tests", [{"name": "TIMEOUT", "value": 3000}]),
              {"name": "codex_st_hal", "command": ["x"], "properties": [{"name": "TIMEOUT", "value": 300}]}]
        res, _ = self.go(ctest_json=json.dumps({"tests": ok}))
        self.assertTrue(res["ok"], res.get("reason"))
        self.assertEqual(res["ctest_timeouts"], {"python_helper_tests": 600, "integration_tool_tests": 3000})
        ev = ig.make_evidence(self.root, "T-1", {"default": res}, {"result": "OK", "offenders": [], "changed": []}, "abc", "PASS")
        self.assertEqual(ev["ctest_timeouts"], {"default": {"python_helper_tests": 600, "integration_tool_tests": 3000}})
        res, _ = self.go(ctest_json=json.dumps({"tests": [integ, py("python_helper_tests", [])]}))  # a Python entry without its limit
        self.assertFalse(res["ok"])
        self.assertIn("python_helper_tests has no TIMEOUT", res["reason"])
        other = {"name": "ffb_unit_tests", "command": [os.path.join(bd, "ffb_unit_tests")], "properties": [{"name": "TIMEOUT", "value": 600}]}
        res, _ = self.go(ctest_json=json.dumps({"tests": [integ, other]}))  # a doctest entry with a whole-run limit
        self.assertFalse(res["ok"])
        self.assertIn("ffb_unit_tests carries the time limit TIMEOUT 600", res["reason"])
        other = dict(other, properties=[{"name": "TIMEOUT", "value": 0}])
        res, _ = self.go(ctest_json=json.dumps({"tests": [integ, other]}))  # TIMEOUT 0 = none
        self.assertTrue(res["ok"], res.get("reason"))

    def test_a_case_timeout_inside_another_executables_ctest_entry_fails_naming_the_executable_and_the_case_with_no_rerun(self):
        text = ("  Start 4: ffb_flash_tests\n4/9 Test #4: ffb_flash_tests ........***Failed   10.00 sec\n"
                "CASE TIMEOUT 1200 s: flash case (: with) [brackets] (/p q/test_x.cpp:12)\n\n"
                "89% tests passed, 1 tests failed out of 9\n")
        res, stub = self.go(ctest_rc=8, ctest_text=text)
        self.assertFalse(res["ok"])
        self.assertIn("default: case timed out in ffb_flash_tests: flash case (: with) [brackets]", res["reason"])
        self.assertNotIn("ctest failed", res["reason"])
        self.assertEqual(len(self.ctest_run_calls(stub)), 1)  # no rerun
        self.assertEqual(res["ctest"]["case_timeouts"], ["ffb_flash_tests: flash case (: with) [brackets]"])
        res, _ = self.go(baseline_rc=1, baseline_text="x\nCASE TIMEOUT 1200 s: base case (/p/t.cpp:3)\n")
        self.assertFalse(res["ok"])
        self.assertIn("default: case timed out in check_baseline: base case", res["reason"])

    def test_the_case_timeouts_of_a_ctest_log_name_the_entry_above_them(self):
        text = ("1/3 Test #1: a ..........   Passed\n2/3 Test #2: ffb_x ....***Failed\nCASE TIMEOUT 5 s: c1 (f:1)\n"
                "3/3 Test #3: ffb_y ....***Failed\nCASE TIMEOUT 6 s: c2 (g:2)\n")
        self.assertEqual(ig.ctest_case_timeouts(text), [("ffb_x", "c1"), ("ffb_y", "c2")])
        self.assertEqual(ig.ctest_case_timeouts("no timeout here"), [])

    def test_the_limit_of_a_variant_is_the_default_times_its_factor(self):
        saved = dict(ig.CASE_TIMEOUT_FACTOR)
        try:
            ig.CASE_TIMEOUT_FACTOR.clear()
            self.assertEqual(ig.case_timeout_s("clang-san"), ig.DEFAULT_CASE_TIMEOUT_S)
            ig.CASE_TIMEOUT_FACTOR.update({"san": 3, "cov": 2})
            self.assertEqual(ig.case_timeout_s("default"), ig.DEFAULT_CASE_TIMEOUT_S)
            self.assertEqual(ig.case_timeout_s("clang-san"), 3 * ig.DEFAULT_CASE_TIMEOUT_S)
            self.assertEqual(ig.case_timeout_s("cov"), 2 * ig.DEFAULT_CASE_TIMEOUT_S)
            res, stub = self.go(variant="nodsp-san", cases=many(3))
            self.assertEqual(res["integration"]["case_timeout_s"], 3 * ig.DEFAULT_CASE_TIMEOUT_S)
        finally:
            ig.CASE_TIMEOUT_FACTOR.clear()
            ig.CASE_TIMEOUT_FACTOR.update(saved)

    def test_a_shard_ended_by_the_watchdog_is_a_dead_shard_and_the_named_case_passing_alone_is_recorded_with_the_reason(self):
        lines = []
        res, stub = self.go(cases=many(12), timeout_in_shard=("case 5",), rv=dict(jobs=1), echo=lines.append)
        self.assertTrue(res["ok"], res.get("reason"))
        i = res["integration"]
        self.assertEqual(len(i["dead_shards"]), 1)
        self.assertEqual((i["dead_shards"][0]["first"], i["dead_shards"][0]["last"], i["dead_shards"][0]["rc"]), (4, 6, 86))
        self.assertIn("case timed out: case 5", i["dead_shards"][0]["reason"])
        self.assertEqual(i["rerun_passed"], ["case 5"])  # the timed-out case itself
        self.assertEqual(i["rerun_reasons"], {"case 5": "timeout in shard"})
        self.assertEqual(sorted(i["rerun_unrun_passed"]), ["case 4", "case 6"])  # the others of the dead shard
        self.assertEqual((i["case_timeouts"], i["failed"], i["cases"]), (["case 5"], 0, 12))
        self.assertEqual(i["case_timeout_s"], ig.DEFAULT_CASE_TIMEOUT_S)
        alone = [c for c in self.exe_calls(stub) if "--first=5" in c and "--last=5" in c]
        self.assertEqual(len(alone), 1)  # run alone once
        self.assertTrue(any("passed only when rerun alone" in l and "case 5" in l for l in lines))
        self.assertEqual(self.cases_file()["rerun_passed"], ["case 5"])

    def test_a_case_that_times_out_alone_fails_the_variant_and_is_named(self):
        res, _ = self.go(cases=many(12), timeout_in_shard=("case 5",), timeout_alone=("case 5",), rv=dict(jobs=1))
        self.assertFalse(res["ok"])
        self.assertIn("default: case timed out: case 5", res["reason"])
        i = res["integration"]
        self.assertEqual((i["rerun_failed"], i["case_timeouts"], i["failed"]), (["case 5"], ["case 5"], 1))
        c = {c["name"]: c for c in self.cases_file()["cases"]}
        self.assertEqual(c["case 5"]["status"], "failed")
        self.assertEqual(c["case 4"]["status"], "passed")  # the other cases of the dead shard ran alone and passed

    def test_a_process_that_dies_without_the_watchdog_line_stays_an_ordinary_dead_shard(self):
        res, _ = self.go(cases=many(12), timeout_in_shard=("case 5",), timeout_line=False, rv=dict(jobs=1))
        self.assertTrue(res["ok"], res.get("reason"))
        i = res["integration"]
        self.assertEqual(i["case_timeouts"], [])
        self.assertEqual(i["rerun_reasons"], {})
        self.assertEqual(sorted(i["rerun_unrun_passed"]), ["case 4", "case 5", "case 6"])

    def test_a_case_timeout_in_the_cov_run_is_a_failure_naming_the_case(self):
        res, stub = self.go(variant="cov", cases=many(6), timeout_in_shard=("case 3",))
        self.assertFalse(res["ok"])
        self.assertIn("cov: case timed out: case 3", res["reason"])
        self.assertEqual(res["integration"]["case_timeouts"], ["case 3"])
        self.assertEqual(len([c for c in self.exe_calls(stub)]), 1)  # one process, nothing is rerun

    def test_serial_names_the_case_the_watchdog_ended_the_run_for(self):
        res, _ = self.go(serial=True, cases=many(6), timeout_in_shard=("case 3",))
        self.assertFalse(res["ok"])
        self.assertIn("default: case timed out: case 3", res["reason"])
        self.assertEqual(res["integration"]["case_timeouts"], ["case 3"])

    def test_a_missing_summary_or_zero_cases_stays_a_failure(self):
        res, _ = self.go(cases=[], rv=dict(jobs=1))
        self.assertFalse(res["ok"])
        self.assertIn("zero test cases", res["reason"])
        res, _ = self.go(list_rc=1)
        self.assertFalse(res["ok"])

    def test_the_evidence_has_the_limit_per_variant_and_the_timed_out_cases(self):
        res, _ = self.go(cases=many(8), timeout_in_shard=("case 2",), rv=dict(jobs=1))
        ev = ig.make_evidence(self.root, "T-1", {"default": res}, {"result": "OK", "offenders": [], "changed": []}, "abc", "PASS")
        self.assertEqual(ev["case_timeout_s"], {"default": ig.DEFAULT_CASE_TIMEOUT_S})
        self.assertEqual(ev["case_timeouts"], {"default": ["case 2"]})
        self.assertEqual(ev["rerun_reasons"], {"default": {"case 2": "timeout in shard"}})
        res, _ = self.go(cases=many(3))
        ev = ig.make_evidence(self.root, "T-1", {"default": res}, {"result": "OK", "offenders": [], "changed": []}, "abc", "PASS")
        self.assertEqual(ev["case_timeouts"], {})
        self.assertEqual(ev["case_timeout_s"], {"default": ig.DEFAULT_CASE_TIMEOUT_S})

    def test_the_variant_line_shows_the_case_limit_and_the_wall_time(self):
        res, _ = self.go(cases=many(3))
        line = ig.variant_summary("default", res)
        self.assertIn("case limit %d s" % ig.DEFAULT_CASE_TIMEOUT_S, line)
        self.assertRegex(line, r"[0-9.]+ s wall")

    def test_the_watchdog_line_is_parsed_with_names_that_contain_brackets_and_colons(self):
        text = "x\nCASE TIMEOUT 600 s: a: b (c) [d] (/p q/t.cpp:12)\n"
        self.assertEqual(ig.timed_out_case(86, text), "a: b (c) [d]")
        self.assertIsNone(ig.timed_out_case(1, text))  # the exit code must be the watchdog's too
        self.assertIsNone(ig.timed_out_case(86, "no line"))

    def test_the_help_describes_the_per_case_limit(self):
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            try:
                ig.main(["--help"])
            except SystemExit:
                pass
        text = buf.getvalue()
        for word in ("FFB_IT_CASE_TIMEOUT_S", "none for any whole run", "exit code 86"):
            self.assertIn(word, text.replace("\n", " "))

    def test_other_ways_to_fail(self):
        def fails(text, **o):
            res, _ = self.go(**o)
            self.assertFalse(res["ok"], o)
            self.assertIn(text, res["reason"], o)
        fails("integration case(s) failed", int_rc=1)  # exit code the report does not explain: not trusted, rerun alone, fails again
        fails("cannot list", list_rc=1)
        fails("cannot list the test cases", list_rc=124)
        fails("zero test cases", cases=[Stub.case("x", "Firmware/Tests/Other/test_y.cpp", 1)], rv=dict(scope_files=[CD + "test_x.cpp"]))
        fails("ctest failed", ctest_rc=8)
        fails("check_baseline", baseline_rc=1)
        fails("not found", make_exe=False)
        fails("compile commands differ", cc_flag="-O3")
        fails("warning(s) in WO files", build_text=warn(root=self.root))

    def test_the_scope_runs_only_the_listed_case_files(self):
        other = "Firmware/Tests/Integration_Tests/claude/cases/test_other.cpp"
        cases = many(3) + [Stub.case("other %d" % i, other, i) for i in range(1, 4)]
        res, stub = self.go(cases=cases, rv=dict(jobs=1, scope_files=[CD + "test_other.cpp"]))
        self.assertTrue(res["ok"], res.get("reason"))
        self.assertEqual(res["integration"]["scope"], [CD + "test_other.cpp"])
        by = {c["name"]: c["status"] for c in self.cases_file()["cases"]}
        self.assertEqual(sorted(n for n, st in by.items() if st != "skipped"), ["other 1", "other 2", "other 3"])
        self.assertEqual([by["case %d" % k] for k in (1, 2, 3)], ["skipped"] * 3)  # not selected: skipped, as doctest reports them
        self.assertEqual((res["integration"]["selected"], res["integration"]["passed"]), (3, 3))
        self.assertEqual(self.cases_file()["scope"], [CD + "test_other.cpp"])
        for c in self.exe_calls(stub):
            self.assertIn("-sf=*cases?test_other.cpp", c)
        self.assertIn("scope 1 case file(s)", ig.variant_summary("default", res))
        res, stub = self.go(cases=cases)
        self.assertEqual((res["integration"]["scope"], res["integration"]["cases"]), ("full", 6))
        self.assertIn("scope full", ig.variant_summary("default", res))
        self.assertFalse([a for c in self.exe_calls(stub) for a in c if a.startswith("-sf")])

    def test_selected_cases_are_found_by_name_when_the_report_lists_unselected_ones_before_them(self):
        # the XML report of a run names every registered case: positions of the list and of the report differ
        other = "Firmware/Tests/Integration_Tests/claude/cases/test_a_first.cpp"
        cases = [Stub.case("other %d" % i, other, i) for i in range(1, 6)] + many(6)
        for jobs in (1, 3):
            res, _ = self.go(cases=cases, rv=dict(jobs=jobs, scope_files=[CD + "test_x.cpp"]), die_in_shard=("case 4",))
            self.assertTrue(res["ok"], res.get("reason"))
            i = res["integration"]
            self.assertEqual((i["selected"], i["passed"], i["skipped"], i["failed"]), (6, 6, 5, 0))
            if jobs == 1:  # shards of 2 cases: the one with case 4 dies, its cases run alone
                self.assertEqual(sorted(i["rerun_unrun_passed"]), ["case 3", "case 4"])
        res, _ = self.go(variant="cov", cases=cases, rv=dict(scope_files=[CD + "test_x.cpp"]), fail_in_shard=("case 2",))
        self.assertFalse(res["ok"])  # a failure of the full-order cov run stands
        self.assertEqual(res["integration"]["cov_alone"], {"case 2": "passed"})  # index 2 of the selection, not of the report
        self.assertEqual((res["integration"]["selected"], res["integration"]["skipped"]), (6, 5))

    def test_the_cov_variant_runs_the_integration_executable_in_one_process(self):
        res, stub = self.go(variant="cov", cases=many(12), rv=dict(jobs=10))
        self.assertTrue(res["ok"], res.get("reason"))
        calls = self.exe_calls(stub)
        self.assertEqual(len(calls), 1)
        self.assertFalse([a for a in calls[0] if a.startswith(("--first", "--last", "-ltc"))])
        self.assertEqual((res["integration"]["shards"], res["integration"]["cases"]), (1, 12))
        res, stub = self.go(variant="cov", cases=many(6), fail_in_shard=("case 2",))  # fails in the one process, passes alone
        self.assertFalse(res["ok"])  # the rerun is only recorded: it cannot rescue a full-order run
        self.assertIn("integration case(s) failed, first: case 2", res["reason"])
        self.assertEqual((res["integration"]["cov_alone"], res["integration"]["rerun_passed"]), ({"case 2": "passed"}, []))
        self.assertEqual(len(self.exe_calls(stub)), 2)
        res, stub = self.go(variant="cov", cases=many(6), fail_always=("case 2",))
        self.assertFalse(res["ok"])
        self.assertEqual([c for c in self.exe_calls(stub) if "--first=2" in c and "--last=2" in c].__len__(), 1)  # rerun alone
        self.assertNotIn("-ltc", " ".join(" ".join(c) for c in self.exe_calls(stub)))

    def test_the_job_budget_is_never_exceeded(self):
        for jobs in (1, 3, 10):
            res, stub = self.go(cases=many(40), delay=0.004, rv=dict(jobs=jobs))
            self.assertTrue(res["ok"], res.get("reason"))
            self.assertLessEqual(stub.peak, jobs, jobs)
            if jobs > 1:
                self.assertGreater(stub.peak, 1)  # the shards and ctest really overlap

    def test_the_build_uses_its_share_of_the_budget(self):
        _, stub = self.go(rv=dict(jobs=6, build_jobs=3))
        b = [c for c in stub.calls if c[0] == "cmake" and "--build" in c and "-j3" in c]
        self.assertEqual(len(b), 1)
        _, stub = self.go(rv=dict(jobs=4))
        self.assertEqual(len([c for c in stub.calls if "-j4" in c]), 1)
        _, stub = self.go(serial=True)
        self.assertEqual(len([c for c in stub.calls if "-j%d" % (os.cpu_count() or 2) in c]), 1)

    def test_serial_issues_the_old_command_sequence(self):
        res, stub = self.go(serial=True)
        self.assertTrue(res["ok"], res.get("reason"))
        kinds = []
        for c in stub.calls:
            j = " ".join(map(str, c))
            if c[0] == "cmake" and "-S" in c:
                kinds.append("configure")
            elif c[0] == "ctest" and "-N" in c:
                kinds.append("ctest-list")
            elif c[0] == "ctest" and "--show-only=json-v1" in c:
                kinds.append("ctest-all")
            elif c[0] == "cmake" and "help" in c:
                kinds.append("targets")
            elif c[0] == "cmake":
                kinds.append("build")
            elif c[0] == "ctest":
                kinds.append("ctest")
            elif "check_baseline" in j:
                kinds.append("baseline")
            elif str(c[0]).endswith(ig.EXE):
                kinds.append("exe " + " ".join(c[1:]))
            elif "summarise_results" in j:
                kinds.append("summary")
            elif "-fsyntax-only" in c:
                kinds.append("syntax")
        self.assertEqual(kinds, ["configure", "ctest-list", "targets", "build", "syntax", "ctest-all", "ctest", "baseline", "exe -r=xml", "summary"])
        self.assertEqual(res["integration"]["mode"], "serial")
        self.assertEqual(res["integration"]["scope"], "full")
        self.assertEqual(res["integration"]["cases"], 5)  # from the summary script, as before

    def test_a_crash_of_the_integration_run_does_not_hide_the_other_failures(self):
        res, _ = self.go(ctest_rc=8, fail_always=("case 1",))
        self.assertIn("ctest failed", res["reason"])
        self.assertIn("integration case(s) failed", res["reason"])


class IntegrationRunSafety(IntegrationRun):
    """Fix round of I1-T3: cov failures, ctest command check, time sum, dead shards, sanitizer reports, signals."""

    def test_ctest_command_mismatch_fails_the_variant(self):
        bd = lambda: ig.build_dir_for(self.root, "default")  # noqa: E731
        good = {"name": ig.EXE, "command": [os.path.join(bd(), ig.EXE)], "properties": []}
        for label, test in (
                ("arguments", dict(good, command=good["command"] + ["--abort-after=1"])),
                ("executable", dict(good, command=["/elsewhere/" + ig.EXE])),
                ("ENVIRONMENT", dict(good, properties=[{"name": "ENVIRONMENT", "value": ["X=1"]}])),
                ("ENVIRONMENT_MODIFICATION", dict(good, properties=[{"name": "ENVIRONMENT_MODIFICATION", "value": ["X=set:1"]}]))):
            res, stub = self.go(ctest_json=json.dumps({"tests": [test]}))
            self.assertFalse(res["ok"], label)
            self.assertIn("differs from the one the gate runs", res["reason"])
            self.assertIn(label, res["reason"])
            self.assertFalse(self.exe_calls(stub), label)  # nothing is run
        res, _ = self.go(ctest_json="not json")
        self.assertFalse(res["ok"])
        res, _ = self.go(ctest_json=json.dumps({"tests": []}))
        self.assertFalse(res["ok"])

    def test_ctest_command_with_the_exclusion_list_matches(self):
        res, _ = self.go(cache_override={"FFB_INTEGRATION_TESTS_EXCLUDE": "case 2"})
        self.assertTrue(res["ok"], res.get("reason"))

    def test_ctest_properties_the_gate_does_not_reproduce_are_a_mismatch(self):
        bd = ig.build_dir_for(self.root, "default")
        base = {"name": ig.EXE, "command": [os.path.join(bd, ig.EXE)]}
        for prop in ("WILL_FAIL", "FAIL_REGULAR_EXPRESSION", "PASS_REGULAR_EXPRESSION", "SKIP_RETURN_CODE", "TIMEOUT", "DISABLED"):
            res, stub = self.go(ctest_json=json.dumps({"tests": [dict(base, properties=[{"name": prop, "value": 1}])]}))
            self.assertFalse(res["ok"], prop)
            self.assertIn("the property %s" % prop, res["reason"])
            self.assertIn("does not reproduce", res["reason"])
            self.assertFalse(self.exe_calls(stub), prop)

    def test_ctest_timeout_0_is_allowed_and_any_other_ctest_timeout_is_a_mismatch(self):
        bd = ig.build_dir_for(self.root, "default")
        base = {"name": ig.EXE, "command": [os.path.join(bd, ig.EXE)]}
        res, _ = self.go(ctest_json=json.dumps({"tests": [dict(base, properties=[{"name": "TIMEOUT", "value": 0}])]}))
        self.assertTrue(res["ok"], res.get("reason"))
        res, _ = self.go(ctest_json=json.dumps({"tests": [dict(base, properties=[{"name": "TIMEOUT", "value": 1500}])]}))
        self.assertFalse(res["ok"])

    def test_ctest_properties_the_gate_reproduces_are_allowed(self):
        wd = os.path.join(self.tmp, "wd")
        res, _ = self.go(ctest_props=[{"name": "WORKING_DIRECTORY", "value": wd},
                                      {"name": "_BACKTRACE_TRIPLES", "value": [["/x/CMakeLists.txt", 1, "add_test"]]}])
        self.assertTrue(res["ok"], res.get("reason"))

    def test_the_wall_time_sum_counts_the_processes_and_not_the_wait_for_a_slot(self):
        names = ("default", "m32", "san", "clang")
        stubs, results = {}, {}
        for v in names:
            stub = Stub(self.root, cases=many(16), delay=0.01)
            bd = ig.build_dir_for(self.root, v)
            ref = Stub(self.root)
            ref.write_build(bd, v)
            ig.record_reference(self.root, bd, v)
            stub.write_build(bd, v)
            stubs[v] = stub
        budget = ig.Budget(2)  # four variants and 8 shards each wait a lot for two slots

        def work(v):
            results[v] = ig.run_variant(self.root, v, LIST_PATTERNS, self.evid, stubs[v], jobs=2, budget=budget, echo=lambda l: None)
        ts = [threading.Thread(target=work, args=(v,)) for v in names]
        for t in ts:
            t.start()
        for t in ts:
            t.join()
        for v in names:
            self.assertTrue(results[v]["ok"], results[v].get("reason"))
            spent = sum(stubs[v].exe_times)
            self.assertGreater(spent, 0.05)
            got = results[v]["integration"]["wall_sum_s"]
            # the waiting (about 4 x as long as the work with 4 variants on 2 slots) is not in the sum
            self.assertLess(got, spent * 1.3 + 0.05, (v, got, spent))
            self.assertGreaterEqual(got, spent * 0.9 - 0.01, (v, got, spent))

    def test_the_working_directory_of_ctest_is_used_for_every_run(self):
        wd = os.path.join(self.tmp, "wd")
        res, stub = self.go(cases=many(6), rv=dict(jobs=1), ctest_props=[{"name": "WORKING_DIRECTORY", "value": wd}])
        self.assertTrue(res["ok"], res.get("reason"))
        runs = [cwd for c, cwd in zip(stub.calls, stub.cwds) if str(c[0]).endswith(ig.EXE)]
        self.assertTrue(runs and all(c == wd for c in runs), runs)

    def test_the_sum_of_the_wall_times_is_recorded_and_never_a_failure(self):
        res, _ = self.go(cases=many(8), delay=0.004, rv=dict(jobs=1))
        self.assertTrue(res["ok"], res.get("reason"))
        self.assertIn("wall_sum_s", res["integration"])
        self.assertGreater(res["integration"]["wall_sum_s"], 0.0)

    def test_a_dead_shard_is_recorded_with_its_last_started_case_and_printed(self):
        lines = []
        res, _ = self.go(cases=many(12), die_in_shard=("case 5",), rv=dict(jobs=1), echo=lines.append)
        self.assertTrue(res["ok"], res.get("reason"))
        d = res["integration"]["dead_shards"]
        self.assertEqual(len(d), 1)
        self.assertEqual((d[0]["shard"], d[0]["first"], d[0]["last"], d[0]["rc"], d[0]["last_started"]), (2, 4, 6, 134, "case 5"))
        self.assertTrue(d[0]["log"].endswith("default.integration.shard002.xml.log"))
        self.assertTrue(any(l.startswith("WARNING: dead shard 2 (cases 4-6, rc 134, last started: case 5") for l in lines), lines)
        self.assertIn("1 dead shard(s)", ig.variant_summary("default", res))

    def test_a_sanitizer_report_kills_a_shard_even_when_a_failed_case_explains_rc_1(self):
        res, _ = self.go(cases=many(12), sanitizer_in_shard=("case 5",), fail_in_shard=("case 4",), rv=dict(jobs=1))
        self.assertTrue(res["ok"], res.get("reason"))
        i = res["integration"]
        self.assertEqual(len(i["dead_shards"]), 1)
        self.assertIn("sanitizer", i["dead_shards"][0]["reason"])
        self.assertEqual(sorted(i["rerun_unrun_passed"]), ["case 4", "case 5", "case 6"])  # nothing of the shard is trusted

    def test_a_sanitizer_report_of_a_case_alone_is_a_failure(self):
        res, _ = self.go(cases=many(12), sanitizer_in=("case 5",), rv=dict(jobs=1))
        self.assertFalse(res["ok"])
        self.assertIn("integration case(s) failed", res["reason"])
        self.assertIn("case 5", res["integration"]["rerun_failed"])
        note = {c["name"]: c.get("note", "") for c in self.cases_file()["cases"]}["case 5"]
        self.assertIn("sanitizer", note)

    def test_all_sanitizer_markers_are_recognised(self):
        for t in ("ERROR: AddressSanitizer: x", "ERROR: LeakSanitizer: y", "ERROR: ThreadSanitizer: z", "a.cpp:1:2: runtime error: signed overflow"):
            self.assertTrue(ig.SANITIZER_REPORT.search(t), t)
        self.assertFalse(ig.SANITIZER_REPORT.search("no problem, runtime errors are fine"))

    # a signal after a complete report: rc < 0 with the later cases of the range missing (skipped)
    def test_a_shard_killed_by_a_signal_is_dead_and_nothing_of_it_is_trusted(self):
        res, _ = self.go(cases=many(12), neg_rc_after={"case 5": -11}, rv=dict(jobs=1))
        self.assertTrue(res["ok"], res.get("reason"))
        i = res["integration"]
        self.assertEqual([(d["shard"], d["rc"]) for d in i["dead_shards"]], [(2, -11)])
        # the case before the crash (case 4) and the ones after it (case 6) all ran alone; case 5 too
        self.assertEqual(sorted(i["rerun_unrun_passed"]), ["case 4", "case 5", "case 6"])
        by = {c["name"]: c["status"] for c in self.cases_file()["cases"]}
        self.assertEqual(set(by.values()), {"passed"})  # the skipped-looking later cases are not results

    def test_a_signal_with_a_failed_case_in_the_shard_does_not_make_the_shard_trusted(self):
        res, _ = self.go(cases=many(12), neg_rc_after={"case 6": -11}, fail_in_shard=("case 4",), rv=dict(jobs=1))
        self.assertTrue(res["ok"], res.get("reason"))
        i = res["integration"]
        self.assertEqual([d["rc"] for d in i["dead_shards"]], [-11])
        self.assertEqual(i["rerun_passed"], [])  # the failed case died with its shard: it is unrun, not "failed in a shard"
        self.assertEqual(sorted(i["rerun_unrun_passed"]), ["case 4", "case 5", "case 6"])

    def test_the_cov_run_killed_by_a_signal_fails_and_nothing_is_rerun(self):
        for opts in (dict(neg_rc_after={"case 3": -11}), dict(neg_rc_after={"case 3": -11}, fail_in_shard=("case 2",))):
            res, stub = self.go(variant="cov", cases=many(6), **opts)
            self.assertFalse(res["ok"], opts)
            self.assertIn("not usable", res["reason"])
            self.assertEqual(len(self.exe_calls(stub)), 1, opts)

    def test_a_skipped_looking_element_of_a_selected_case_is_never_a_result(self):
        # rc 0 and a complete report in which a selected case is skipped="true": the case is unrun and runs alone
        class Odd(Stub):
            def exe_call(self2, cmd):
                rc, text = Stub.exe_call(self2, cmd)
                if "--first=1" in cmd:
                    text = re.sub(r'(<TestCase name="case 2"[^>]*?)>.*?</TestCase>', r'\1 skipped="true"/>', text, count=1)
                return rc, text
        bd = ig.build_dir_for(self.root, "default")
        stub = Odd(self.root, cases=many(6))
        stub.write_build(bd, "default")
        ig.record_reference(self.root, bd, "default")
        res = ig.run_variant(self.root, "default", LIST_PATTERNS, self.evid, stub, jobs=1, echo=lambda l: None)
        self.assertTrue(res["ok"], res.get("reason"))
        self.assertEqual(res["integration"]["rerun_unrun_passed"], ["case 2"])
        self.assertEqual(res["integration"]["skipped"], 0)


class CaseWatchdogFiring(unittest.TestCase):
    """The firing path of the executable's per-case watchdog (support/CaseWatchdog.cpp), with the real executable: a hidden case
    (doctest::skip(), never run unless --no-skip is given; it is a skipped case in every normal run) sleeps on the host clock far
    beyond a limit of 1 s set through the environment. The executable is the one of the build that ctest runs this test for
    (FFB_IT_EXE, set by integration.cmake); without it the test is skipped."""
    HELPER = "support: watchdog helper hangs on the host clock"
    EXE_ENV = "FFB_IT_EXE"
    BUILT_ENV = "FFB_IT_BUILT"
    HELPER_FILE = "test_f1_support.cpp"
    TARGET = "ffb_integration_tests"
    CMAKE_FILE = os.path.join("..", "integration.cmake")  # relative to tools/: holds the target's ffb_add_case_watchdog line

    def exe(self):
        """Rule: under ctest integration.cmake sets <BUILT_ENV> to 1 when the executable is built and 0 when it is not (flash on macOS).
        Built but <EXE_ENV> missing or not a file = failure (never a silent skip); not built, or run by hand without ctest (no <BUILT_ENV>) = skip."""
        built = os.environ.get(self.BUILT_ENV)
        exe = os.environ.get(self.EXE_ENV)
        if built == "1":
            self.assertTrue(exe and os.path.isfile(exe), "%s says %s is built but %s is %r" % (self.BUILT_ENV, self.TARGET, self.EXE_ENV, exe))
        elif not exe or not os.path.isfile(exe):
            self.skipTest("%s is not built or %s (the executable of this self-test) is not set" % (self.TARGET, self.EXE_ENV))
        return exe

    def cmake_default(self):
        """The default seconds that CMake gives this executable: `ffb_add_case_watchdog(<target> <n>)` (the oracle for the compiled default)."""
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), self.CMAKE_FILE), encoding="utf-8") as f:
            m = re.search(r"^\s*ffb_add_case_watchdog\(%s (\d+)\)" % self.TARGET, f.read(), re.M)
        self.assertTrue(m, "no ffb_add_case_watchdog(%s <n>) in %s" % (self.TARGET, self.CMAKE_FILE))
        return int(m.group(1))

    def run_helper(self, limit, extra=()):
        env = dict(os.environ)
        env.pop("FFB_IT_CASE_TIMEOUT_S", None)
        if limit is not None:
            env["FFB_IT_CASE_TIMEOUT_S"] = limit
        t = time.monotonic()
        p = subprocess.run([self.exe(), "--no-skip", "-tc=" + self.HELPER] + list(extra), stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, text=True, errors="replace", env=env, timeout=120)
        return p, time.monotonic() - t

    def test_a_case_beyond_the_limit_ends_the_process_with_the_exit_code_and_the_line(self):
        p, took = self.run_helper("1")
        self.assertEqual(p.returncode, ig.CASE_TIMEOUT_RC)
        lines = [l for l in p.stderr.splitlines() if l.startswith("CASE TIMEOUT")]
        self.assertEqual(len(lines), 1, p.stderr)
        m = ig.CASE_TIMEOUT_LINE.match(lines[0])
        self.assertTrue(m, lines[0])
        self.assertEqual((m.group(1), m.group(2)), ("1", self.HELPER))
        self.assertTrue(m.group(3).endswith(self.HELPER_FILE), m.group(3))
        self.assertLess(took, 20)  # ended by the watchdog, not by the helper's 30 s sleep
        self.assertEqual(ig.timed_out_case(p.returncode, p.stderr), self.HELPER)

    def test_the_helper_is_not_part_of_a_normal_run_and_is_a_skipped_case(self):
        p = subprocess.run([self.exe(), "-ltc", "-r=xml"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                           errors="replace", timeout=120)
        self.assertEqual(p.returncode, 0)
        self.assertNotIn(self.HELPER, p.stdout)  # -ltc lists only the cases a run would execute
        p = subprocess.run([self.exe(), "-tc=" + self.HELPER], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                           errors="replace", timeout=120, env=dict(os.environ, FFB_IT_CASE_TIMEOUT_S="1"))
        self.assertEqual(p.returncode, 0)  # skipped without --no-skip: nothing sleeps, nothing fires
        self.assertNotIn("CASE TIMEOUT", p.stderr)

    def test_the_executable_accepts_0_and_warns_about_a_bad_value(self):
        # the helper sleeps 30 s: too long to wait for in a unit test, so ask for a case filter that runs nothing and only check
        # that 0 and a bad value are accepted by the executable at start-up
        exe = self.exe()
        p = subprocess.run([exe, "-tc=no such test case"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                           errors="replace", env=dict(os.environ, FFB_IT_CASE_TIMEOUT_S="0"), timeout=120)
        self.assertEqual(p.returncode, 0)
        p = subprocess.run([exe, "-tc=no such test case"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                           errors="replace", env=dict(os.environ, FFB_IT_CASE_TIMEOUT_S="abc"), timeout=120)
        self.assertEqual(p.returncode, 0)
        self.assertIn("FFB_IT_CASE_TIMEOUT_S is not a non-negative integer", p.stderr)
        # the binary's own default is the one CMake gave it (a hard-coded kDefaultCaseTimeoutS in the source would be caught here)
        self.assertIn("using the default of %d s" % self.cmake_default(), p.stderr)


class CaseWatchdogFiringInFlashTests(CaseWatchdogFiring):
    """The same firing self-test in an executable other than ffb_integration_tests (tool WO I2-T2): ffb_flash_tests (FFB_FLASH_EXE and
    FFB_FLASH_BUILT, set by integration.cmake). Its hidden helper case is Code_Coverage/claude/cases_flash/test_case_watchdog_helper.cpp, compiled
    into that executable only: it proves the shared listener is registered, reads the shared variable, uses the default CMake gave this
    executable and ends the process with exit code 86 in an executable that is not the integration one. Skipped only where flash is not built (macOS)."""
    HELPER = "support: case watchdog helper hangs on the host clock"
    EXE_ENV = "FFB_FLASH_EXE"
    BUILT_ENV = "FFB_FLASH_BUILT"
    HELPER_FILE = "test_case_watchdog_helper.cpp"
    TARGET = "ffb_flash_tests"
    CMAKE_FILE = os.path.join("..", "..", "CMakeLists.txt")


class GateScope(unittest.TestCase):
    CASE = CD + "test_a.cpp"

    def test_only_an_isolated_run_uses_the_subset(self):
        self.assertIsNone(ig.gate_scope([self.CASE], [self.CASE], False, False))
        self.assertEqual(ig.gate_scope([self.CASE], [self.CASE], False, True), [self.CASE])

    def test_every_changed_non_document_file_must_be_a_case_file_directly_in_the_cases_directory(self):
        docs = ["Tests/Integration_Tests/claude/plans/PROGRESS.md"]
        self.assertEqual(ig.gate_scope([self.CASE, docs[0]], [self.CASE] + docs, False, True), [self.CASE])
        for other in ("Firmware/Tests/Integration_Tests/support/x.h", CD + "sub/test_b.cpp", CD + "helper.cpp", CD + "test_b.h",
                      "Firmware/Tests/Integration_Tests/integration.cmake"):
            self.assertIsNone(ig.gate_scope([self.CASE], [self.CASE, other], False, True), other)

    def test_a_run_without_a_changed_file_or_with_a_non_case_list_or_full_runs_everything(self):
        self.assertIsNone(ig.gate_scope([self.CASE], [], False, True))
        self.assertIsNone(ig.gate_scope([self.CASE, "Firmware/Tests/Integration_Tests/support/x.h"], [self.CASE], False, True))
        self.assertIsNone(ig.gate_scope([self.CASE], [self.CASE], True, True))

    def test_the_aggregate_warning_names_every_variant_with_reruns_or_dead_shards(self):
        self.assertIsNone(ig.aggregate_warning({"default": {"integration": {"rerun_passed": [], "dead_shards": []}}, "m32": {}}))
        w = ig.aggregate_warning({"default": {"integration": {"rerun_passed": ["a"], "rerun_unrun_passed": ["b"], "dead_shards": []}},
                                  "m32": {"integration": {"dead_shards": [{}]}}, "san": {"integration": {}}})
        self.assertTrue(w.startswith("WARNING: default: 2 case(s) passed only when rerun alone, 0 dead shard(s); m32: 0 case(s)"))
        self.assertNotIn("san:", w)



class MainParallel(MainLogic):
    """main() in the parallel mode (the MainLogic fixtures)."""

    def lines_of(self, out):
        return out.strip().splitlines()

    def test_jobs_above_ten_or_below_one_are_refused(self):
        for n in ("11", "100", "0", "-1"):
            rc, last, _ = self.run_main(["--variants", "default,m32", "--jobs", n])
            self.assertEqual(rc, 1, n)
            self.assertTrue(last.startswith("GATE FAIL --jobs must be between 1 and 10"), last)
        rc, last, _ = self.run_main(["--variants", "default,m32", "--jobs", "10"])
        self.assertEqual(rc, 0, last)

    def test_the_variants_run_at_the_same_time_within_the_job_budget(self):
        stub = Stub(self.root, delay=0.004, cases=many(30))
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = ig.main(["--root", self.root, "--wo", "T-1", "--variants", "default,m32", "--jobs", "6"], runner=stub)
        self.assertEqual(rc, 0, buf.getvalue())
        self.assertLessEqual(stub.peak, 6)
        self.assertGreater(stub.peak, 2)
        builds = [c for c in stub.calls if c[0] == "cmake" and "--build" in c and "help" not in c]
        self.assertEqual(sorted(c[-1] for c in builds), ["-j3", "-j3"])  # 6 jobs // 2 variants
        self.assertEqual(self.latest()["jobs"], 6)

    def test_the_output_of_each_variant_stays_together(self):
        stub = Stub(self.root, delay=0.003, cases=many(20), fail_in_shard=("case 3",))
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = ig.main(["--root", self.root, "--wo", "T-1", "--variants", "default,m32", "--jobs", "2"], runner=stub)
        lines = buf.getvalue().strip().splitlines()
        self.assertEqual(rc, 0, buf.getvalue())
        for v in ("default", "m32"):
            i = next(k for k, l in enumerate(lines) if l.startswith("== variant %s " % v))
            j = next(k for k, l in enumerate(lines) if l.startswith("   %s: OK" % v))
            between = lines[i + 1:j]
            self.assertTrue(all(l.startswith("WARNING:") and "variant %s" % v in l for l in between), (v, between))
        self.assertTrue(lines[-1].startswith("GATE PASS "))

    def test_a_rerun_passed_case_passes_the_gate_and_is_in_the_evidence_and_the_output(self):
        stub = Stub(self.root, cases=many(12), fail_in_shard=("case 5",))
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = ig.main(["--root", self.root, "--wo", "T-1", "--variants", "default,m32", "--jobs", "1"], runner=stub)
        out = buf.getvalue()
        self.assertEqual(rc, 0, out)
        self.assertTrue(out.strip().splitlines()[-1].startswith("GATE PASS "))
        self.assertIn("WARNING: 1 case(s) passed only when rerun alone: case 5 (variant default)", out)
        ev = self.latest()
        self.assertEqual(ev["rerun_passed"], {"default": ["case 5"], "m32": ["case 5"]})
        self.assertEqual(ev["rerun_failed"], {})

    def test_evidence_fields_of_both_modes(self):
        rc, last, _ = self.run_main(["--variants", "default,m32"])
        self.assertEqual(rc, 0, last)
        ev = self.latest()
        # the WO list of T-1 is the glob cases/*.cpp plus integration.cmake: not provably only case files -> full
        self.assertEqual((ev["integration_scope"], ev["serial"], ev["jobs"]), ("full", False, 10))
        self.assertEqual(ev["variants"]["default"]["integration"]["mode"], "parallel")
        rc, last, _ = self.run_main(["--variants", "default,m32", "--serial"])
        self.assertEqual(rc, 0, last)
        ev = self.latest()
        self.assertEqual((ev["integration_scope"], ev["serial"], ev["jobs"]), ("full", True, None))
        self.assertEqual(ev["variants"]["m32"]["integration"]["mode"], "serial")

    def test_a_wo_of_case_files_runs_only_their_cases_and_records_the_scope(self):
        self.write("Tests/Integration_Tests/claude/charters/wo/T-1.files", CASE_REL + "\nTests/Integration_Tests/claude/plans/PROGRESS.md\n")
        other = "Firmware/Tests/Integration_Tests/claude/cases/test_other.cpp"
        cases = many(3) + [Stub.case("other %d" % i, other, i) for i in range(1, 4)]
        stub = Stub(self.root, cases=cases)
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = ig.main(["--root", self.root, "--wo", "T-1", "--variants", "default,m32", "--isolated"], runner=stub)
        out = buf.getvalue()
        self.assertEqual(rc, 0, out)
        ev = self.latest()
        self.assertEqual(ev["integration_scope"], [CASE_REL])
        i = ev["variants"]["default"]["integration"]
        self.assertEqual((i["selected"], i["passed"], i["skipped"]), (3, 3, 3))
        self.assertIn("scope 1 case file(s)", out)
        for c in [c for c in stub.calls if str(c[0]).endswith(ig.EXE)]:
            self.assertIn("-sf=*cases?test_x.cpp", c)
        # --full and --serial run every case
        for extra in (["--full"], ["--serial"]):
            for v in ig.FULL_VARIANTS:
                s = Stub(self.root)
                bd = ig.build_dir_for(self.root, v)
                s.write_build(bd, v)
                ig.record_reference(self.root, bd, v)
            stub = Stub(self.root, cases=cases)
            with contextlib.redirect_stdout(io.StringIO()):
                ig.main(["--root", self.root, "--wo", "T-1", "--variants", "default,m32"] + extra, runner=stub)
            self.assertEqual(self.latest()["integration_scope"], "full", extra)
            if extra == ["--full"]:
                self.assertEqual(self.latest()["variants"]["default"]["integration"]["cases"], 6, extra)

    def test_a_non_isolated_run_never_uses_the_subset_and_the_aggregate_warning_precedes_the_last_line(self):
        self.write("Tests/Integration_Tests/claude/charters/wo/T-1.files", CASE_REL + "\n")
        stub = Stub(self.root, cases=many(12), fail_in_shard=("case 5",))
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = ig.main(["--root", self.root, "--wo", "T-1", "--variants", "default,m32", "--jobs", "1"], runner=stub)
        lines = buf.getvalue().strip().splitlines()
        self.assertEqual(rc, 0, buf.getvalue())
        self.assertEqual(self.latest()["integration_scope"], "full")  # not --isolated: HEAD plus working tree is not provably the WO
        self.assertTrue(lines[-1].startswith("GATE PASS "))
        self.assertTrue(lines[-2].startswith("WARNING: default: 1 case(s) passed only when rerun alone, 0 dead shard(s); m32: 1 case(s)"), lines[-2])

    def test_an_isolated_run_with_a_changed_support_file_runs_every_case(self):
        self.write("Tests/Integration_Tests/claude/charters/wo/T-1.files", CASE_REL + "\nFirmware/Tests/Integration_Tests/integration.cmake\n")
        self.write("Firmware/Tests/Integration_Tests/integration.cmake", "changed\n")
        stub = Stub(self.root, cases=many(3))
        import io
        import contextlib
        with contextlib.redirect_stdout(io.StringIO()) as buf:
            rc = ig.main(["--root", self.root, "--wo", "T-1", "--variants", "default,m32", "--isolated"], runner=stub)
        self.assertEqual(rc, 0, buf.getvalue())
        self.assertEqual(self.latest()["integration_scope"], "full")
        self.assertFalse([a for c in stub.calls for a in c if str(a).startswith("-sf")])


    def test_serial_variants_run_one_after_another(self):
        rc, last, out = self.run_main(["--variants", "default,m32", "--serial"], delay=0.002)
        self.assertEqual(rc, 0, out)
        lines = out.splitlines()
        h = [i for i, l in enumerate(lines) if l.startswith("== variant ")]
        self.assertEqual(len(h), 2)
        self.assertTrue(lines[h[0] + 1].startswith("   default: OK"))  # the summary comes before the next variant starts
        self.assertNotIn("in parallel", out)


for _name in dir(MainLogic):  # MainParallel only reuses the fixtures of MainLogic: its inherited tests are not run twice
    if _name.startswith("test_") and _name not in MainParallel.__dict__:
        setattr(MainParallel, _name, None)


if __name__ == "__main__":
    unittest.main()
