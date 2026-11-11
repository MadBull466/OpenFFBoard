"""Cases of Firmware/scripts/generate_memory.py (tier A helper: tested, never fixed). Extends test_generate_memory.py.

The script reads a memory map CSV (header row; columns section comment, name, address, comment, VirtAddVarTab flag,
exportableFlashAddresses flag) and writes eeprom_addresses.c / .h. scripts/README.md: it "generates ... and does a
sanity check" (check_duplicated). Every file the script reads or writes here lives in a temporary directory.

Suspected defects are tests of the intended behaviour marked @unittest.expectedFailure (the stdlib counterpart of
KNOWN_ISSUE: an unexpected success fails the run once the script is fixed). `characterization:` tests document
behaviour whose intent is unclear (NEEDS REVIEW numbers in the names).
"""
import contextlib
import io
import pathlib
import subprocess
import sys
import tempfile
import unittest

import helper_paths as paths

HEADER_ROW = "Section comment,Name,Value,Comment on key,VirtAddVarTab,exportableFlashAddresses\n"


def entry(sec=None, name=None, adr=None, comment=None, enable=None, export=None):
    return {"sec": sec, "name": name, "adr": adr, "comment": comment, "enable": enable, "export": export}


class ScriptCase(unittest.TestCase):
    """Temporary directory, the script loaded as a module and a runner for the program."""

    @classmethod
    def setUpClass(cls):
        cls.gm = paths.load_script(paths.GENERATE_MEMORY, "generate_memory_cases_under_test")

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = pathlib.Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def write_csv(self, rows, name="map.csv"):
        path = self.dir / name
        path.write_text(HEADER_ROW + "".join(r + "\n" for r in rows), encoding="utf-8")
        return path

    def run_script(self, *args):
        return subprocess.run([sys.executable, "-B", str(paths.GENERATE_MEMORY), *args], cwd=self.dir,
                              capture_output=True, text=True, timeout=60, check=False)

    def run_on(self, rows):
        """Runs the program on a CSV with these rows; outputs out.c / out.h in the temporary directory."""
        csv_file = self.write_csv(rows)
        return self.run_script("-f", str(csv_file), "--srcfile", "out.c", "--hfile", "out.h")

    def outputs_exist(self):
        return (self.dir / "out.c").exists() or (self.dir / "out.h").exists()

    def duplicates(self, entries):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            count = self.gm.check_duplicated(entries)
        return count, out.getvalue()


class ParseCsvTest(ScriptCase):
    def test_the_first_row_is_skipped_as_the_header_whatever_it_contains(self):
        path = self.dir / "map.csv"
        path.write_text(",ADR_IN_HEADER,0x10,,1,1\n,ADR_A,0x11,,1,1\n", encoding="utf-8")
        self.assertEqual([e["name"] for e in self.gm.parse_csv(path)], ["ADR_A"])

    def test_the_six_columns_map_to_section_name_address_comment_enable_and_export(self):
        path = self.write_csv(["// Section,,,,,", ",ADR_A,0x101,a comment,1,0"])
        self.assertEqual(self.gm.parse_csv(path), [
            entry(sec="// Section"),
            entry(name="ADR_A", adr="0x101", comment="a comment", enable="1", export="0"),
        ])

    def test_empty_cells_become_none(self):
        path = self.write_csv([",ADR_A,7,,,"])
        self.assertEqual(self.gm.parse_csv(path), [entry(name="ADR_A", adr="7")])

    def test_a_quoted_comment_keeps_its_commas(self):
        path = self.write_csv([',ADR_A,0x284,"// 0-7 inertia, 8-15 friction",1,1'])
        self.assertEqual(self.gm.parse_csv(path)[0]["comment"], "// 0-7 inertia, 8-15 friction")

    def test_addresses_are_kept_as_written_in_decimal_and_hex(self):
        path = self.write_csv([",ADR_DEC,10,,1,1", ",ADR_HEX,0xC1,,1,1"])
        self.assertEqual([e["adr"] for e in self.gm.parse_csv(path)], ["10", "0xC1"])

    def test_an_empty_row_of_commas_gives_an_entry_without_fields(self):
        # memory_map.csv uses ",,,,," as spacer rows (lines 163, 164, 181)
        path = self.write_csv([",,,,,"])
        self.assertEqual(self.gm.parse_csv(path), [entry()])


class SourceFileTest(ScriptCase):
    def test_section_comments_appear_in_both_arrays(self):
        src = self.gm.generatesrc([entry(sec="// Ports"), entry(name="ADR_A", adr="1", enable="1", export="1")])
        self.assertEqual(src.count("// Ports\n\tADR_A,\n"), 2)

    def test_an_enabled_variable_is_listed_in_virtaddvartab_and_a_disabled_one_is_commented_out(self):
        src = self.gm.generatesrc([entry(name="ADR_ON", adr="1", enable="1"), entry(name="ADR_OFF", adr="2")])
        table = src.split("VirtAddVarTab[NB_OF_VAR] =")[1].split("};")[0]
        self.assertIn("\n\tADR_ON,\n", table)
        self.assertIn("\n//\tADR_OFF,\n", table)

    def test_an_exported_variable_is_listed_in_the_export_table_and_another_one_is_commented_out(self):
        src = self.gm.generatesrc([entry(name="ADR_EXP", adr="1", enable="1", export="1"),
                                   entry(name="ADR_NOEXP", adr="2", enable="1", export="0")])
        table = src.split("exportableFlashAddresses[NB_EXPORTABLE_ADR] =")[1].split("};")[0]
        self.assertIn("\n\tADR_EXP,\n", table)
        self.assertIn("\n//\tADR_NOEXP,\n", table)

    def test_a_flag_other_than_1_counts_as_not_set(self):
        src = self.gm.generatesrc([entry(name="ADR_A", adr="1", enable="yes", export=" 1")])
        self.assertEqual(src.count("//\tADR_A,"), 2)

    def test_a_comment_gets_a_line_comment_marker_unless_it_has_one(self):
        src = self.gm.generatesrc([entry(name="ADR_A", adr="1", comment="plain", enable="1"),
                                   entry(name="ADR_B", adr="2", comment="// marked", enable="1")])
        self.assertIn("\tADR_A,// plain\n", src)
        self.assertIn("\tADR_B, // marked\n", src)
        self.assertNotIn("//// marked", src)

    def test_rows_without_a_name_or_without_an_address_produce_no_array_element(self):
        src = self.gm.generatesrc([entry(name="ADR_NOADR", enable="1", export="1"),
                                   entry(adr="0x55", enable="1", export="1")])
        self.assertNotIn("ADR_NOADR", src)
        self.assertNotIn("0x55", src)

    def test_the_source_includes_the_header_and_closes_both_arrays(self):
        src = self.gm.generatesrc([])
        self.assertIn('#include "eeprom_addresses.h"', src)
        self.assertTrue(src.endswith("{\n};"))
        self.assertEqual(src.count("};"), 2)


class HeaderFileTest(ScriptCase):
    def test_every_named_address_becomes_a_define_with_the_address_as_written(self):
        header = self.gm.generate_header([entry(name="ADR_DEC", adr="10"), entry(name="ADR_HEX", adr="0xC1")])
        self.assertIn("#define ADR_DEC 10\n", header)
        self.assertIn("#define ADR_HEX 0xC1\n", header)

    def test_disabled_variables_still_get_a_define(self):
        header = self.gm.generate_header([entry(name="ADR_OLD", adr="0x20", enable="0", export="0")])
        self.assertIn("#define ADR_OLD 0x20\n", header)
        self.assertIn("#define NB_OF_VAR 0\n", header)

    def test_the_counts_are_the_numbers_of_enabled_and_of_exported_variables(self):
        entries = [entry(name=f"ADR_{i}", adr=str(i), enable="1", export="1" if i % 3 == 0 else "0")
                   for i in range(1, 8)]
        header = self.gm.generate_header(entries)
        self.assertIn("#define NB_OF_VAR 7\n", header)
        self.assertIn("#define NB_EXPORTABLE_ADR 2\n", header)

    def test_the_header_has_an_include_guard_and_section_comments(self):
        header = self.gm.generate_header([entry(sec="// FFBWheel"), entry(name="ADR_A", adr="1", comment="c")])
        self.assertIn("#ifndef EEPROM_ADDRESSES_H_\n#define EEPROM_ADDRESSES_H_\n", header)
        self.assertIn("// FFBWheel\n#define ADR_A 1// c\n", header)
        self.assertTrue(header.endswith("#endif /* EEPROM_ADDRESSES_H_ */\n"))



class IncompleteEnabledRowTest(ScriptCase):
    """bug V36: generate_header counts every row with enable "1" (generate_memory.py:111-113) but generatesrc and
    generate_header only emit rows with a name AND an address (:84, :118). An enabled row whose address (or name) is
    missing makes NB_OF_VAR larger than the initialiser list: C zero-fills the rest of VirtAddVarTab, so virtual
    address 0 becomes a registered variable for the EEPROM page transfer. The same for NB_EXPORTABLE_ADR (address 0
    in the flash dump list).

    The generation runs in setUp (not under @expectedFailure): a fix that rejects the row with an exception is
    reported as an error instead of being swallowed as the expected failure; only the count assertion is expected
    to fail today."""

    def setUp(self):
        super().setUp()
        self.header = self.gm.generate_header([entry(name="ADR_A", adr="1", enable="1", export="1"),
                                               entry(name="ADR_TODO", enable="1", export="1")])

    @unittest.expectedFailure
    def test_the_variable_count_matches_the_array_elements_also_for_an_enabled_row_without_an_address(self):
        self.assertIn("#define NB_OF_VAR 1\n", self.header)
        self.assertIn("#define NB_EXPORTABLE_ADR 1\n", self.header)


class DuplicateCheckTest(ScriptCase):
    def test_unique_addresses_give_no_duplicates(self):
        count, out = self.duplicates([entry(name="A", adr="1", enable="1"), entry(name="B", adr="2", export="1")])
        self.assertEqual(count, 0)
        self.assertEqual(out, "")

    def test_each_address_used_more_than_once_counts_once(self):
        entries = [entry(name=n, adr=a, enable="1") for n, a in
                   (("A", "0x10"), ("B", "0x10"), ("C", "0x10"), ("D", "0x20"), ("E", "0x20"), ("F", "0x30"))]
        count, out = self.duplicates(entries)
        self.assertEqual(count, 2)
        self.assertIn("Found 2 duplicate addresses:", out)

    def test_an_exported_but_disabled_entry_takes_part_in_the_check(self):
        count, _ = self.duplicates([entry(name="A", adr="5", enable="1"), entry(name="B", adr="5", export="1")])
        self.assertEqual(count, 1)

    def test_an_entry_with_empty_enable_and_export_columns_is_not_checked(self):
        # retired addresses may stay in the map without their flags
        count, _ = self.duplicates([entry(name="A", adr="5", enable="1"), entry(name="OLD", adr="5")])
        self.assertEqual(count, 0)

    # NEEDS REVIEW 164: check_duplicated only looks at addresses. Two rows with the same name and different
    # addresses pass and give two different #defines of one macro (a compiler error or warning in the firmware
    # build, not in the script); a flag cell "0" counts as set for the check (`d["enable"] or d["export"]` is true
    # for the string "0") although the generators treat it as not set. Should names be checked and only "1" count?
    def test_characterization_a_name_used_twice_is_not_reported_needs_review_164(self):
        count, out = self.duplicates([entry(name="ADR_A", adr="1", enable="1"), entry(name="ADR_A", adr="2",
                                                                                      enable="1")])
        self.assertEqual(count, 0)
        self.assertEqual(out, "")

    def test_characterization_flags_of_0_still_take_part_in_the_check_needs_review_164(self):
        count, _ = self.duplicates([entry(name="A", adr="5", enable="1"), entry(name="B", adr="5", enable="0",
                                                                               export="0")])
        self.assertEqual(count, 1)


class NumericDuplicateTest(ScriptCase):
    """bug V35: check_duplicated (generate_memory.py:137-143) compares the address cells as strings. The map mixes
    notations (memory_map.csv: "1", "10" decimal, "0xC1" hex), so 16 and 0x10, 0x101 and 0x0101 or 0x1A and 0x1a are
    not found although the C compiler sees the same virtual address: two variables would share one EEPROM cell.
    Intended: compare the numeric values (int(adr, 0)).

    The checks run in setUp (not under @expectedFailure), so a fix that raises is reported as an error."""

    PAIRS = (("16", "0x10"), ("0x101", "0x0101"), ("0x1A", "0x1a"))

    def setUp(self):
        super().setUp()
        self.counts = [self.duplicates([entry(name="A", adr=a, enable="1"), entry(name="B", adr=b, enable="1")])[0]
                       for a, b in self.PAIRS]

    @unittest.expectedFailure
    def test_the_same_address_written_in_decimal_and_in_hex_is_a_duplicate(self):
        self.assertEqual(self.counts, [1, 1, 1])


class CommandLineTest(ScriptCase):
    def test_without_arguments_it_reads_memory_map_csv_and_writes_the_default_files_in_the_current_directory(self):
        self.write_csv([",ADR_A,0x101,,1,1"], name="memory_map.csv")
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("\tADR_A,", paths.read_text_lf(self.dir / "eeprom_addresses.c"))
        self.assertIn("#define ADR_A 0x101", paths.read_text_lf(self.dir / "eeprom_addresses.h"))

    def test_the_long_options_select_the_input_and_both_output_paths(self):
        csv_file = self.write_csv([",ADR_A,0x101,,1,1"], name="in.csv")
        (self.dir / "sub").mkdir()
        result = self.run_script("--filename", str(csv_file), "--srcfile", "sub/a.c", "--hfile", "sub/a.h")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue((self.dir / "sub" / "a.c").is_file())
        self.assertTrue((self.dir / "sub" / "a.h").is_file())
        self.assertFalse((self.dir / "eeprom_addresses.c").exists())

    def test_a_valid_map_prints_nothing(self):
        result = self.run_on([",ADR_A,1,,1,1"])
        self.assertEqual((result.returncode, result.stdout, result.stderr), (0, "", ""))

    def test_existing_output_files_are_overwritten(self):
        (self.dir / "out.c").write_text("old", encoding="utf-8")
        result = self.run_on([",ADR_A,1,,1,1"])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("old", paths.read_text_lf(self.dir / "out.c"))

    def test_the_help_names_the_program_and_its_options(self):
        result = self.run_script("--help")
        self.assertEqual(result.returncode, 0)
        for text in ("Memory map generator", "--filename", "--srcfile", "--hfile", "eeprom emulation"):
            self.assertIn(text, result.stdout)

    def test_an_unknown_option_is_rejected_with_usage_and_exit_status_2(self):
        result = self.run_script("--bogus")
        self.assertEqual(result.returncode, 2)
        self.assertIn("unrecognized arguments: --bogus", result.stderr)

    def test_a_missing_map_file_fails_without_writing_outputs(self):
        result = self.run_script("-f", "missing.csv", "--srcfile", "out.c", "--hfile", "out.h")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("missing.csv", result.stderr)
        self.assertFalse(self.outputs_exist())

    def test_a_row_with_a_missing_column_fails_without_writing_outputs(self):
        result = self.run_on([",ADR_A,1,,1,1", ",ADR_B,2,,1"])
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.outputs_exist())

    def test_an_output_directory_that_does_not_exist_fails(self):
        csv_file = self.write_csv([",ADR_A,1,,1,1"])
        result = self.run_script("-f", str(csv_file), "--srcfile", "nodir/out.c", "--hfile", "out.h")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("nodir", result.stderr)

    # NEEDS REVIEW 163: error exits. A failed sanity check prints "Aborting" but main() returns normally (exit status
    # 0, generate_memory.py:155-157), so a build step or script cannot see the failure; malformed input (a blank line,
    # a row with too few cells) ends with a Python traceback (exit 1) instead of a message naming the line. Should a
    # failed check exit non-zero, and should blank lines be skipped?
    def test_characterization_a_failed_duplicate_check_exits_with_status_0_needs_review_163(self):
        result = self.run_on([",ADR_A,7,,1,1", ",ADR_B,7,,1,1"])
        self.assertEqual(result.returncode, 0)
        self.assertIn("Found 1 duplicate addresses", result.stdout)
        self.assertTrue(result.stdout.endswith("Aborting\n"))
        self.assertFalse(self.outputs_exist())

    def test_characterization_a_row_with_too_few_cells_stops_the_script_with_a_traceback_needs_review_163(self):
        result = self.run_on([",ADR_A,1,,1,1", ",ADR_B,2,,1"])
        self.assertEqual(result.returncode, 1)
        self.assertIn("ValueError: not enough values to unpack", result.stderr)

    def test_characterization_a_blank_line_stops_the_script_with_a_traceback_needs_review_163(self):
        result = self.run_on([",ADR_A,1,,1,1", "", ",ADR_B,2,,1,1"])
        self.assertEqual(result.returncode, 1)
        self.assertIn("ValueError: not enough values to unpack", result.stderr)
        self.assertFalse(self.outputs_exist())


if __name__ == "__main__":
    unittest.main()
