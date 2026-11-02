"""Smoke tests of Firmware/scripts/generate_memory.py (tier A helper: tested, never fixed).

The script reads memory_map.csv and writes eeprom_addresses.c / .h. The tests only read files of the repository;
everything the script writes goes to a temporary directory.
"""
import contextlib
import io
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest

import helper_paths as paths


class GenerateMemoryTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = pathlib.Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def run_script(self, csv_file):
        """Runs the script as a program (like a developer) with the outputs in the temporary directory."""
        return subprocess.run(
            [sys.executable, "-B", str(paths.GENERATE_MEMORY), "-f", str(csv_file),
             "--srcfile", str(self.dir / "out.c"), "--hfile", str(self.dir / "out.h")],
            cwd=self.dir, capture_output=True, text=True, timeout=60, check=False)

    def test_the_memory_map_reproduces_the_committed_eeprom_address_files(self):
        result = self.run_script(paths.MEMORY_MAP_CSV)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(paths.read_text_lf(self.dir / "out.c"), paths.read_text_lf(paths.EEPROM_ADDRESSES_C))
        self.assertEqual(paths.read_text_lf(self.dir / "out.h"), paths.read_text_lf(paths.EEPROM_ADDRESSES_H))

    def test_a_duplicate_address_aborts_without_writing_files(self):
        csv_file = self.dir / "map.csv"
        csv_file.write_text("section,name,address,comment,enable,export\n"
                            ",ADR_A,0x0101,first,1,1\n"
                            ",ADR_B,0x0101,same address,1,0\n", encoding="utf-8")
        result = self.run_script(csv_file)
        self.assertEqual(result.returncode, 0)  # main() returns normally after printing "Aborting"
        self.assertIn("Found 1 duplicate addresses", result.stdout)
        self.assertIn("Aborting", result.stdout)
        self.assertFalse((self.dir / "out.c").exists())
        self.assertFalse((self.dir / "out.h").exists())

    def test_the_header_counts_the_enabled_and_the_exported_variables(self):
        module = paths.load_script(paths.GENERATE_MEMORY, "generate_memory_under_test")
        entries = [
            {"sec": "// Section", "name": None, "adr": None, "comment": None, "enable": None, "export": None},
            {"sec": None, "name": "ADR_A", "adr": "0x0101", "comment": "a", "enable": "1", "export": "1"},
            {"sec": None, "name": "ADR_B", "adr": "0x0102", "comment": None, "enable": "1", "export": None},
            {"sec": None, "name": "ADR_C", "adr": "0x0103", "comment": None, "enable": None, "export": None},
        ]
        header = module.generate_header(entries)
        self.assertIn("#define NB_OF_VAR 2\n", header)
        self.assertIn("#define NB_EXPORTABLE_ADR 1\n", header)
        self.assertIn("#define ADR_A 0x0101// a\n", header)
        source = module.generatesrc(entries)
        self.assertIn("\tADR_B,\n", source)          # enabled: in VirtAddVarTab
        self.assertIn("//\tADR_C,\n", source)        # neither enabled nor exported: commented out
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(module.check_duplicated(entries), 0)


if __name__ == "__main__":
    unittest.main()
