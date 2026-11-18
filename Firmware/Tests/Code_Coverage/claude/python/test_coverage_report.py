"""Tests of tools/coverage_report.py: the section "Integration (F1 chain)" (integration track, WO I0-8) and its separation from the other
sections, with synthetic gcov data (a fake gcov command that prints prepared JSON). No test executable is run, no real gcov is needed.
"""
import contextlib
import io
import json
import os
import pathlib
import shlex
import sys
import tempfile
import unittest

import helper_paths

COVERAGE_REPORT = helper_paths.FIRMWARE_DIR / "Tests" / "Code_Coverage" / "tools" / "coverage_report.py"
cr = helper_paths.load_script(COVERAGE_REPORT, "coverage_report_under_test")

FW = os.path.abspath(os.sep + os.path.join("fw", "Firmware"))

# A fake gcov: prints the JSON found next to the .gcda argument (<gcda>.json), one document per line like gcov --json-format --stdout
FAKE_GCOV = """import sys
with open(sys.argv[-1] + ".json", encoding="utf-8") as f:
    sys.stdout.write(f.read().strip() + "\\n")
"""


def abs_of(rel):
    return os.path.normpath(os.path.join(FW, rel))


def entry(lines, functions=None, starts=None, ends=None):
    """Merged-data entry like run_gcov returns it: lines {n: count}, functions {name: count}, fn_start / fn_end."""
    functions = functions or {}
    return {"lines": dict(lines), "functions": dict(functions), "fn_start": dict(starts or {}), "fn_end": dict(ends or {})}


def measured(functions):
    """The function rows with data (the inline rows without a function record have no file)."""
    return [f for f in functions if f[1] is not None]


class GcdaClassification(unittest.TestCase):
    def test_the_objects_of_the_integration_target_are_recognised_on_both_path_styles(self):
        self.assertTrue(cr.is_integration_gcda("/b/CMakeFiles/ffb_integration_tests.dir/x/Axis.cpp.gcda"))
        self.assertTrue(cr.is_integration_gcda("C:\\b\\CMakeFiles\\ffb_integration_tests.dir\\Axis.cpp.gcda"))

    def test_the_objects_of_the_other_targets_are_not_integration_objects(self):
        for name in ("ffb_unit_tests", "ffb_target_tests", "ffb_driver_tests", "ffb_dsp_tests", "ffb_cmsis_dsp", "ffb_hal_tests_hal"):
            self.assertFalse(cr.is_integration_gcda("/b/CMakeFiles/%s.dir/Axis.cpp.gcda" % name), name)
        self.assertFalse(cr.is_integration_gcda("/b/CMakeFiles/ffb_integration_tests_extra.dir/x.gcda"))

    def test_the_chain_file_list_has_the_files_of_the_scope_document(self):
        names = [os.path.basename(f) for f in cr.INTEGRATION_CHAIN_FILES]
        for expected in ("HidFFB.cpp", "EffectsCalculator.cpp", "Filters.cpp", "Axis.cpp", "AxesManager.cpp", "FFBHIDMain.cpp", "FFBWheel.cpp",
                         "TMC4671.cpp", "MotorDriver.cpp", "Encoder.cpp", "global_callbacks.cpp", "UsbHidHandler.cpp", "hid_device.c"):
            self.assertIn(expected, names)
        self.assertEqual(len(names), len(set(names)))

    def test_the_chain_files_of_the_list_exist_in_the_firmware(self):
        for rel in cr.INTEGRATION_CHAIN_FILES:
            self.assertTrue((helper_paths.FIRMWARE_DIR / rel).is_file(), rel)

    def test_the_dsp_function_pattern_selects_the_functions_on_the_path_only(self):
        for name in ("arm_biquad_cascade_df1_f32", "arm_biquad_cascade_df1_init_f32", "arm_spline_f32", "arm_spline_init_f32", "arm_sin_f32",
                     "arm_cos_f32", "arm_sqrt_f32", "arm_pid_init_f32", "arm_pid_reset_f32", "arm_std_f32"):
            self.assertTrue(cr.INTEGRATION_DSP_FUNCTIONS.match(name), name)
        for name in ("arm_var_f32", "arm_sin_q31", "arm_std_q31", "arm_biquad_cascade_df2T_f32", "x_arm_sin_f32", "arm_sin_f32_extra"):
            self.assertFalse(cr.INTEGRATION_DSP_FUNCTIONS.match(name), name)


class ChainRows(unittest.TestCase):
    def setUp(self):
        self.axis = "FFBoard/Src/Axis.cpp"
        self.sin = "Libraries/CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.c"

    def rows(self, data):
        return cr.integration_chain_rows({abs_of(rel): e for rel, e in data.items()}, FW)

    def test_a_chain_file_gives_its_executable_and_executed_lines(self):
        files, functions, total = self.rows({self.axis: entry({1: 3, 2: 0, 3: 1, 4: 0})})
        self.assertEqual(files[cr.INTEGRATION_CHAIN_FILES.index(self.axis)], (self.axis, 2, 4))
        self.assertEqual(measured(functions), [])
        self.assertEqual(total, (2, 4))

    def test_a_chain_file_without_data_is_listed_without_numbers(self):
        files, _, _ = self.rows({self.axis: entry({1: 1})})
        missing = [f for f in files if f[0] == "FFBoard/Src/HidFFB.cpp"][0]
        self.assertEqual(missing, ("FFBoard/Src/HidFFB.cpp", None, None))

    def test_the_files_come_in_the_order_of_the_list(self):
        files, _, _ = self.rows({})
        self.assertEqual([f[0] for f in files], cr.INTEGRATION_CHAIN_FILES)

    def test_a_dsp_function_counts_only_the_lines_between_its_start_and_end_line(self):
        data = {self.sin: entry({10: 5, 11: 5, 12: 0, 20: 0, 21: 0, 30: 7}, {"arm_sin_f32": 5, "arm_sin_q31": 0},
                                {"arm_sin_f32": 10, "arm_sin_q31": 20}, {"arm_sin_f32": 12, "arm_sin_q31": 21})}
        _, functions, total = self.rows(data)
        self.assertEqual(measured(functions), [("arm_sin_f32", self.sin, 2, 3, False)])
        self.assertEqual(total, (2, 3))

    def test_a_dsp_function_without_an_end_line_counts_the_whole_file_and_says_so(self):
        data = {self.sin: entry({10: 5, 11: 0, 30: 7}, {"arm_sin_f32": 5}, {"arm_sin_f32": 10}, {})}
        _, functions, _ = self.rows(data)
        self.assertEqual(measured(functions), [("arm_sin_f32", self.sin, 2, 3, True)])
        out = []
        cr.print_integration_section({abs_of(self.sin): data[self.sin]}, FW, out.append)
        self.assertTrue(any("whole file: gcov gave no end line" in line for line in out))

    def test_dsp_functions_off_the_path_and_files_outside_the_dsp_library_are_not_counted(self):
        data = {self.sin: entry({10: 1}, {"arm_var_f32": 1}, {"arm_var_f32": 10}, {"arm_var_f32": 10}),
                self.axis: entry({1: 1}, {"arm_sin_f32": 1}, {"arm_sin_f32": 1}, {"arm_sin_f32": 1})}
        _, functions, _ = self.rows(data)
        self.assertEqual(measured(functions), [])

    def test_inline_functions_without_a_function_record_are_listed_as_not_measured(self):
        # realistic gcov data: line data for the header, but no function record for the __STATIC_FORCEINLINE functions
        header = "Libraries/CMSIS-DSP/Include/dsp/fast_math_functions.h"
        _, functions, total = self.rows({header: entry({236: 4, 237: 4, 238: 0})})
        self.assertEqual(measured(functions), [])
        self.assertEqual([(f[0], f[1]) for f in functions if f[1] is None], [("arm_sqrt_f32", None), ("arm_pid_f32", None)])
        self.assertEqual(total, (0, 0))
        out = []
        cr.print_integration_section({abs_of(header): entry({236: 4})}, FW, out.append)
        text = "\n".join(out)
        self.assertIn("| `arm_sqrt_f32` | not measured: inline function, gcov gives no function record | n/a | n/a | n/a |", text)
        self.assertIn("| `arm_pid_f32` | not measured: inline function, gcov gives no function record |", text)

    def test_an_inline_function_with_a_function_record_is_measured_by_name(self):
        header = "Libraries/CMSIS-DSP/Include/dsp/fast_math_functions.h"
        data = {header: entry({236: 4, 237: 4, 238: 0, 300: 9}, {"arm_sqrt_f32": 4}, {"arm_sqrt_f32": 236}, {"arm_sqrt_f32": 238})}
        _, functions, _ = self.rows(data)
        self.assertEqual(measured(functions), [("arm_sqrt_f32", header, 2, 3, False)])
        self.assertEqual([f[0] for f in functions if f[1] is None], ["arm_pid_f32"])

    def test_the_total_adds_the_chain_files_and_the_dsp_functions(self):
        data = {self.axis: entry({1: 1, 2: 0}),
                self.sin: entry({10: 1, 11: 0}, {"arm_sin_f32": 1}, {"arm_sin_f32": 10}, {"arm_sin_f32": 11})}
        self.assertEqual(self.rows(data)[2], (2, 4))


class Section(unittest.TestCase):
    def render(self, data):
        out = []
        cr.print_integration_section({abs_of(rel): e for rel, e in data.items()}, FW, out.append)
        return "\n".join(out)

    def test_the_section_has_a_heading_per_file_lines_executed_and_percentage_and_a_total(self):
        text = self.render({"FFBoard/Src/Axis.cpp": entry({1: 1, 2: 0, 3: 1, 4: 1}),
                            "FFBoard/Src/HidFFB.cpp": entry({1: 0, 2: 0})})
        self.assertIn("### Integration (F1 chain)", text)
        self.assertIn("| File | Lines | Executed | Covered |", text)
        self.assertIn("| `FFBoard/Src/Axis.cpp` | 4 | 3 | 75.0 % |", text)
        self.assertIn("| `FFBoard/Src/HidFFB.cpp` | 2 | 0 | 0.0 % |", text)
        self.assertIn("| `FFBoard/Src/Filters.cpp` | no data | n/a | n/a |", text)
        self.assertIn("**Total chain: 3 of 6 lines executed (50.0 %)**", text)

    def test_the_dsp_functions_have_their_own_table(self):
        sin = "Libraries/CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.c"
        text = self.render({sin: entry({10: 1, 11: 0}, {"arm_sin_f32": 1}, {"arm_sin_f32": 10}, {"arm_sin_f32": 11})})
        self.assertIn("| Function | File | Lines | Executed | Covered |", text)
        self.assertIn("| `arm_sin_f32` | `%s` | 2 | 1 | 50.0 %% |" % sin, text)

    def test_without_integration_data_the_section_says_so(self):
        out = []
        cr.print_integration_section({}, FW, out.append)
        text = "\n".join(out)
        self.assertIn("### Integration (F1 chain)", text)
        self.assertIn("No ffb_integration_tests coverage data", text)
        self.assertNotIn("Total chain", text)

    def test_an_empty_total_is_n_a(self):
        self.assertIn("**Total chain: 0 of 0 lines executed (n/a)**", self.render({"FFBoard/Src/Other.cpp": entry({1: 1})}))


class RunGcov(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = pathlib.Path(self.tmp.name)
        self.script = self.root / "fake_gcov.py"
        self.script.write_text(FAKE_GCOV, encoding="utf-8")

    def gcda(self, name, doc):
        path = self.root / name
        path.write_bytes(b"")
        (self.root / (name + ".json")).write_text(json.dumps(doc), encoding="utf-8")
        return str(path)

    def test_function_start_and_end_lines_are_read_and_the_counts_of_two_objects_are_added(self):
        def doc(count):
            return {"current_working_directory": FW, "files": [{"file": "Libraries/CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.c",
                "lines": [{"line_number": 10, "count": count}, {"line_number": 11, "count": 0}],
                "functions": [{"name": "arm_sin_f32", "execution_count": count, "start_line": 10, "end_line": 11}]}]}
        files = [self.gcda("a.gcda", doc(2)), self.gcda("b.gcda", doc(3))]
        data = cr.run_gcov([sys.executable, str(self.script)], files)
        e = data[abs_of("Libraries/CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.c")]
        self.assertEqual(e["lines"], {10: 5, 11: 0})
        self.assertEqual(e["functions"], {"arm_sin_f32": 5})
        self.assertEqual((e["fn_start"]["arm_sin_f32"], e["fn_end"]["arm_sin_f32"]), (10, 11))

    def test_a_function_without_an_end_line_gets_end_line_0(self):
        doc = {"current_working_directory": FW, "files": [{"file": "x.c", "lines": [], "functions": [
            {"name": "f", "execution_count": 1, "start_line": 4}]}]}
        data = cr.run_gcov([sys.executable, str(self.script)], [self.gcda("c.gcda", doc)])
        self.assertEqual(data[abs_of("x.c")]["fn_end"]["f"], 0)


class WholeReport(unittest.TestCase):
    """main() on a small synthetic firmware tree and build directory: the integration objects are a section of their own."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = pathlib.Path(self.tmp.name)
        self.fw = root / "Firmware"
        for d in ("FFBoard/Src", "FFBoard/UserExtensions/Src"):
            (self.fw / d).mkdir(parents=True)
        for name in ("Axis.cpp", "HidFFB.cpp"):
            (self.fw / "FFBoard" / "Src" / name).write_text("int f()\n{\n\treturn 1;\n}\nint g()\n{\n\treturn 2;\n}\n", encoding="utf-8")
        self.build = root / "build"
        self.script = root / "fake_gcov.py"
        self.script.write_text(FAKE_GCOV, encoding="utf-8")

    def add_gcda(self, target, name, lines_by_file):
        folder = self.build / "CMakeFiles" / (target + ".dir")
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / (name + ".gcda")
        path.write_bytes(b"")
        files = [{"file": rel, "lines": [{"line_number": n, "count": c} for n, c in sorted(lines.items())], "functions": []}
                 for rel, lines in lines_by_file.items()]
        pathlib.Path(str(path) + ".json").write_text(json.dumps({"current_working_directory": str(self.fw), "files": files}), encoding="utf-8")

    def report(self):
        out = io.StringIO()
        gcov = "%s %s" % (shlex.quote(sys.executable.replace("\\", "/")), shlex.quote(str(self.script).replace("\\", "/")))
        with contextlib.redirect_stdout(out):
            rc = cr.main(["--build-dir", str(self.build), "--firmware-dir", str(self.fw), "--gcov", gcov, "--no-exclusions"])
        self.assertEqual(rc, 0)
        return out.getvalue()

    def setup_data(self):
        self.add_gcda("ffb_unit_tests", "Axis.cpp", {"FFBoard/Src/Axis.cpp": {3: 1, 7: 0}})
        self.add_gcda("ffb_integration_tests", "Axis.cpp", {"FFBoard/Src/Axis.cpp": {3: 5, 7: 5}})
        self.add_gcda("ffb_integration_tests", "HidFFB.cpp", {"FFBoard/Src/HidFFB.cpp": {3: 0, 7: 0}})

    def test_the_integration_section_counts_the_integration_objects_alone(self):
        self.setup_data()
        text = self.report()
        self.assertIn("### Integration (F1 chain)", text)
        self.assertIn("| `FFBoard/Src/Axis.cpp` | 2 | 2 | 100.0 % |", text)
        self.assertIn("| `FFBoard/Src/HidFFB.cpp` | 2 | 0 | 0.0 % |", text)

    def test_the_other_sections_do_not_count_the_integration_objects(self):
        self.setup_data()
        text = self.report()
        self.assertIn("| firmware | Axis | `FFBoard/Src/Axis.cpp` | 1 / 2 | 50.0 % | 0 / 0 |", text)
        self.assertNotIn("HidFFB.cpp` | 0 / 2", text.split("### Integration (F1 chain)")[0])

    def test_the_gcda_count_of_the_header_line_leaves_out_the_integration_objects(self):
        self.setup_data()
        self.assertIn(", 1 .gcda files", self.report())

    def test_without_integration_objects_the_section_reports_no_data(self):
        self.add_gcda("ffb_unit_tests", "Axis.cpp", {"FFBoard/Src/Axis.cpp": {3: 1, 7: 0}})
        text = self.report()
        self.assertIn("No ffb_integration_tests coverage data", text)
        self.assertIn("| firmware | Axis | `FFBoard/Src/Axis.cpp` | 1 / 2 | 50.0 % | 0 / 0 |", text)

    def test_the_other_sections_are_the_same_with_and_without_the_integration_objects(self):
        self.add_gcda("ffb_unit_tests", "Axis.cpp", {"FFBoard/Src/Axis.cpp": {3: 1, 7: 0}})
        without = self.report()
        self.add_gcda("ffb_integration_tests", "Axis.cpp", {"FFBoard/Src/Axis.cpp": {3: 5, 7: 5}})
        with_it = self.report()

        def others(text):
            head, rest = text.split("### Integration (F1 chain)")
            tail = rest.split("### ", 1)[1]
            return "\n".join(l for l in (head + tail).splitlines() if not l.startswith("Build directory"))
        self.assertEqual(others(with_it), others(without))

    def test_an_integration_object_that_never_ran_counts_its_lines_as_not_executed(self):
        # only a .gcno (compiled, never run): gcov is asked for its lines through the notes file
        self.add_gcda("ffb_integration_tests", "Axis.cpp", {"FFBoard/Src/Axis.cpp": {3: 5, 7: 5}})
        folder = self.build / "CMakeFiles" / "ffb_integration_tests.dir"
        note = folder / "HidFFB.cpp.gcno"
        note.write_bytes(b"")
        doc = {"current_working_directory": str(self.fw), "files": [{"file": "FFBoard/Src/HidFFB.cpp", "functions": [],
               "lines": [{"line_number": 3, "count": 0}, {"line_number": 7, "count": 0}]}]}
        pathlib.Path(str(note) + ".json").write_text(json.dumps(doc), encoding="utf-8")
        text = self.report()
        self.assertIn("| `FFBoard/Src/HidFFB.cpp` | 2 | 0 | 0.0 % |", text)
        self.assertNotIn("HidFFB.cpp` | 0 / 2", text.split("### Integration (F1 chain)")[0])


if __name__ == "__main__":
    unittest.main()
