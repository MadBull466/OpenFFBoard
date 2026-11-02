"""Smoke tests of FFBoard/USB/portable/synopsys/dwc2/dwc2_info.py (TinyUSB helper, vendor tier A: tested, never fixed).

The script decodes the DWC2 hardware configuration registers of a list of MCUs and writes the table to dwc2_info.md
in the current directory with pandas. It imports `click` and `pandas`, which are not part of the standard library and
must not be installed for the tests: both are replaced by minimal stand-ins while the script is loaded. The script
runs in a temporary directory.
"""
import os
import pathlib
import sys
import tempfile
import types
import unittest

import helper_paths as paths


class FakeDataFrame:
    """The part of pandas.DataFrame dwc2_info.main() uses: DataFrame(rows).set_index(col).T.to_markdown()"""
    created = []

    def __init__(self, rows):
        self.rows = rows
        FakeDataFrame.created.append(self)

    def set_index(self, column):
        self.index = column
        return self

    @property
    def T(self):  # noqa: N802 (pandas name)
        return self

    def to_markdown(self):
        return "\n".join(str(row[self.index]) for row in self.rows)


def load_dwc2_info():
    """Imports dwc2_info.py with stand-ins for click and pandas (removed again afterwards)."""
    saved = {name: sys.modules.get(name) for name in ("click", "pandas")}
    sys.modules["click"] = types.ModuleType("click")
    pandas = types.ModuleType("pandas")
    pandas.DataFrame = FakeDataFrame
    sys.modules["pandas"] = pandas
    try:
        return paths.load_script(paths.DWC2_INFO, "dwc2_info_under_test")
    finally:
        for name, module in saved.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module


F407_FS = "ST F207/F407/411/429 FS"


class Dwc2InfoTest(unittest.TestCase):
    def setUp(self):
        self.module = load_dwc2_info()
        FakeDataFrame.created.clear()
        self.tmp = tempfile.TemporaryDirectory()
        self.cwd = os.getcwd()
        os.chdir(self.tmp.name)
        self.argv = sys.argv
        sys.argv = ["dwc2_info.py"]

    def tearDown(self):
        sys.argv = self.argv
        os.chdir(self.cwd)
        self.tmp.cleanup()

    def rows(self):
        self.module.main()
        self.assertEqual(len(FakeDataFrame.created), 1)
        return {row["Device"]: row for row in FakeDataFrame.created[0].rows}

    def test_main_writes_the_table_of_all_devices_to_dwc2_info_md_in_the_current_directory(self):
        rows = self.rows()
        self.assertEqual(set(rows), set(self.module.dwc2_reg_value))
        text = (pathlib.Path(self.tmp.name) / "dwc2_info.md").read_text(encoding="utf-8")
        self.assertIn(F407_FS, text)

    def test_the_f407_full_speed_core_decodes_as_synopsys_otg_2_81a_with_3_device_endpoints_besides_ep0(self):
        row = self.rows()[F407_FS]
        self.assertEqual(row["GSNPSID"], "0x4F54281A")
        self.assertEqual(row[" - specs version"], "2.81a")  # GSNPSID 0x4F54281A: "OT2" core release 2.81a
        self.assertEqual(row[" - num_dev_ep"], 3)            # RM0090 OTG_FS: 4 bidirectional endpoints incl. EP0
        self.assertEqual(row[" - arch"], "Slave only")       # OTG_FS has no DMA (RM0090 34.1)
        self.assertEqual(row[" - fs_phy_type"], "Dedicated")


if __name__ == "__main__":
    unittest.main()
