"""Cases of FFBoard/USB/portable/synopsys/dwc2/dwc2_info.py (TinyUSB helper, vendor tier A: tested, never fixed).
Extends test_dwc2_info.py and reuses its stand-ins for `click` and `pandas` (imported as a module, so its test class
is not collected twice).

Independent reference for the register fields: the bit positions of the Synopsys DWC_otg GHWCFG2/3/4 registers as
listed per field in TinyUSB's dwc2_type.h (dwc2_ghwcfg2_t / _3_t / _4_t comments "0..2", "3..4", ...) written here as
(name, lowest bit, width) and decoded with shifts and masks, and the value names of those comments. GSNPSID: Synopsys
ID 0x4F54xxxx ("OT"), release x.yz in the next three nibbles, the last nibble 0xA = "a".
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
import test_dwc2_info as batch0

# (field, lowest bit, width), dwc2_type.h dwc2_ghwcfg2_t / dwc2_ghwcfg3_t / dwc2_ghwcfg4_t
LAYOUT = {
    "GHWCFG2": [("op_mode", 0, 3), ("arch", 3, 2), ("single_point", 5, 1), ("hs_phy_type", 6, 2),
                ("fs_phy_type", 8, 2), ("num_dev_ep", 10, 4), ("num_host_ch", 14, 4),
                ("period_channel_support", 18, 1), ("enable_dynamic_fifo", 19, 1), ("mul_proc_intrpt", 20, 1),
                ("reserved21", 21, 1), ("nptx_q_depth", 22, 2), ("ptx_q_depth", 24, 2), ("token_q_depth", 26, 5),
                ("otg_enable_ic_usb", 31, 1)],
    "GHWCFG3": [("xfer_size_width", 0, 4), ("packet_size_width", 4, 3), ("otg_enable", 7, 1), ("i2c_enable", 8, 1),
                ("vendor_ctrl_itf", 9, 1), ("optional_feature_removed", 10, 1), ("synch_reset", 11, 1),
                ("otg_adp_support", 12, 1), ("otg_enable_hsic", 13, 1), ("battery_charger_support", 14, 1),
                ("lpm_mode", 15, 1), ("dfifo_depth", 16, 16)],
    "GHWCFG4": [("num_dev_period_in_ep", 0, 4), ("partial_powerdown", 4, 1), ("ahb_freq_min", 5, 1),
                ("hibernation", 6, 1), ("extended_hibernation", 7, 1), ("reserved8", 8, 1),
                ("enhanced_lpm_support1", 9, 1), ("service_interval_flow", 10, 1), ("ipg_isoc_support", 11, 1),
                ("acg_support", 12, 1), ("enhanced_lpm_support", 13, 1), ("phy_data_width", 14, 2),
                ("ctrl_ep_num", 16, 4), ("iddg_filter", 20, 1), ("vbus_valid_filter", 21, 1),
                ("a_valid_filter", 22, 1), ("b_valid_filter", 23, 1), ("session_end_filter", 24, 1),
                ("dedicated_fifos", 25, 1), ("num_dev_in_eps", 26, 4), ("dma_desc_enable", 30, 1),
                ("dma_desc_dynamic", 31, 1)],
}

# Value names (dwc2_type.h comments; Synopsys databook)
NAMES = {
    "op_mode": {0: "HNP SRP", 1: "SRP", 2: "noHNP noSRP", 3: "SRP Device", 4: "noOTG Device", 5: "SRP Host",
                6: "noOTG Host"},
    "arch": {0: "Slave only", 1: "DMA external", 2: "DMA internal"},
    "single_point": {0: "hub", 1: "n/a"},
    "hs_phy_type": {0: "n/a", 1: "UTMI+", 2: "ULPI", 3: "UTMI+/ULPI"},
    "fs_phy_type": {0: "n/a", 1: "Dedicated", 2: "Shared UTMI+", 3: "Shared ULPI"},
    "nptx_q_depth": {0: "2", 1: "4", 2: "8"},
    "ptx_q_depth": {0: "2", 1: "4", 2: "8", 3: "16"},
    "phy_data_width": {0: "8 bit", 1: "16 bit", 2: "8/16 bit", 3: "Reserved"},
}

F407_FS = "ST F207/F407/411/429 FS"
COMMITTED_MD = paths.DWC2_INFO.parent / "dwc2_info.md"


def expected_field(register_value, lsb, width, name):
    raw = (register_value >> lsb) & ((1 << width) - 1)
    if name in NAMES:
        return NAMES[name].get(raw, f"Unknown ({raw})")
    return raw


def parse_markdown(text):
    """{device: {row label: cell text}} from a pandas to_markdown() pipe table (devices are the columns)."""
    lines = [line for line in text.splitlines() if line.startswith("|")]
    devices = [c.strip() for c in lines[0].strip("|").split("|")][1:]
    table = {d: {} for d in devices}
    labels = []
    for line in lines[2:]:
        cells = [c.strip() for c in line.strip("|").split("|")]
        labels.append(cells[0])
        for device, cell in zip(devices, cells[1:]):
            table[device][cells[0]] = cell
    return table, labels


class Dwc2Case(unittest.TestCase):
    def setUp(self):
        self.module = batch0.load_dwc2_info()
        batch0.FakeDataFrame.created.clear()
        self.tmp = tempfile.TemporaryDirectory()
        self.cwd = os.getcwd()
        os.chdir(self.tmp.name)
        self.argv = sys.argv
        sys.argv = ["dwc2_info.py"]

    def tearDown(self):
        sys.argv = self.argv
        os.chdir(self.cwd)
        self.tmp.cleanup()

    def rows(self, info=None):
        """Runs main() (optionally on another register list) and returns {device: row dict}."""
        if info is not None:
            self.module.dwc2_info = info
        self.module.main()
        return {row["Device"]: row for row in batch0.FakeDataFrame.created[-1].rows}

    @staticmethod
    def device(guid=0, gsnpsid=0x4F54281A, cfg1=0, cfg2=0, cfg3=0, cfg4=0):
        return dict(zip(["GUID", "GSNPSID", "GHWCFG1", "GHWCFG2", "GHWCFG3", "GHWCFG4"],
                        [guid, gsnpsid, cfg1, cfg2, cfg3, cfg4]))


class RegisterLayoutTest(Dwc2Case):
    def test_each_register_structure_has_the_fields_of_the_databook_layout_covering_32_bits(self):
        for register, layout in LAYOUT.items():
            with self.subTest(register=register):
                fields = getattr(self.module, register)._fields_
                self.assertEqual([(name, width) for name, _, width in fields], [(n, w) for n, _, w in layout])
                self.assertEqual(sum(width for _, _, width in fields), 32)

    def test_the_register_list_maps_every_device_to_the_six_registers(self):
        self.assertEqual(self.module.dwc2_reg_list, ["GUID", "GSNPSID", "GHWCFG1", "GHWCFG2", "GHWCFG3", "GHWCFG4"])
        for device, values in self.module.dwc2_reg_value.items():
            with self.subTest(device=device):
                self.assertEqual(list(self.module.dwc2_info[device].values()), values)


class DecodingTest(Dwc2Case):
    def test_every_field_of_every_known_core_decodes_like_the_bit_layout(self):
        rows = self.rows()
        for device, registers in self.module.dwc2_info.items():
            for register, layout in LAYOUT.items():
                for name, lsb, width in layout:
                    with self.subTest(device=device, field=name):
                        self.assertEqual(rows[device][f" - {name}"],
                                         expected_field(registers[register], lsb, width, name))

    def test_registers_are_shown_as_8_digit_upper_case_hex(self):
        row = self.rows()["XMC4500"]
        self.assertEqual(row["GUID"], "0x00AEC000")
        self.assertEqual(row["GHWCFG1"], "0x00000000")
        self.assertEqual(row["GHWCFG3"], "0x027A01E5")

    def test_the_specs_version_of_each_known_core_is_the_release_in_gsnpsid(self):
        rows = self.rows()
        expected = {0x4F54280A: "2.80a", 0x4F54281A: "2.81a", 0x4F54292A: "2.92a", 0x4F54310A: "3.10a",
                    0x4F54320A: "3.20a", 0x4F54330A: "3.30a", 0x4F54400A: "4.00a", 0x4F54411A: "4.11a"}
        for device, registers in self.module.dwc2_info.items():
            if registers["GSNPSID"] in expected:
                with self.subTest(device=device):
                    self.assertEqual(rows[device][" - specs version"], expected[registers["GSNPSID"]])

    def test_a_later_patch_letter_follows_a(self):
        rows = self.rows({"X": self.device(gsnpsid=0x4F54420C)})
        self.assertEqual(rows["X"][" - specs version"], "4.20c")

    def test_values_without_a_name_are_shown_as_unknown_with_the_raw_value(self):
        cfg2 = 7 | (3 << 3) | (3 << 22)  # op_mode 7, arch 3, nptx_q_depth 3: reserved encodings
        row = self.rows({"X": self.device(cfg2=cfg2)})["X"]
        self.assertEqual(row[" - op_mode"], "Unknown (7)")
        self.assertEqual(row[" - arch"], "Unknown (3)")
        self.assertEqual(row[" - nptx_q_depth"], "Unknown (3)")

    def test_the_reserved_phy_data_width_is_named_reserved(self):
        row = self.rows({"X": self.device(cfg4=3 << 14)})["X"]
        self.assertEqual(row[" - phy_data_width"], "Reserved")

    def test_all_ones_set_every_numeric_field_to_its_maximum(self):
        row = self.rows({"X": self.device(cfg2=0xFFFFFFFF, cfg3=0xFFFFFFFF, cfg4=0xFFFFFFFF)})["X"]
        self.assertEqual(row[" - token_q_depth"], 31)
        self.assertEqual(row[" - dfifo_depth"], 0xFFFF)
        self.assertEqual(row[" - num_dev_in_eps"], 15)
        self.assertEqual(row[" - dma_desc_dynamic"], 1)
        self.assertEqual(row[" - ptx_q_depth"], "16")

    def test_ghwcfg1_and_guid_are_not_split_into_fields(self):
        row = self.rows({"X": self.device(guid=0xFFFFFFFF, cfg1=0xFFFFFFFF)})["X"]
        self.assertEqual(row["GHWCFG1"], "0xFFFFFFFF")
        self.assertEqual(row["GUID"], "0xFFFFFFFF")
        self.assertFalse(any(key.startswith(" - ") and key[3:] not in
                             {n for layout in LAYOUT.values() for n, _, _ in layout} | {"specs version"}
                             for key in row))

    def test_the_rows_list_registers_in_order_each_followed_by_its_fields(self):
        row = self.rows({"X": self.device()})["X"]
        expected = ["Device", "GUID", "GSNPSID", " - specs version", "GHWCFG1"]
        for register in ("GHWCFG2", "GHWCFG3", "GHWCFG4"):
            expected += [register] + [f" - {n}" for n, _, _ in LAYOUT[register]]
        self.assertEqual(list(row), expected)

    # NEEDS REVIEW 165: GD32VF103 has placeholder zeros for GSNPSID and GHWCFG2-4 (the values are not known), but the
    # script decodes them like real register values: specs version "0.00W" (chr(0 + ord('a') - 0xA)), op_mode
    # "HNP SRP", nptx_q_depth "2" ... (committed dwc2_info.md, column GD32VF103). Fields are raw register encodings
    # (xfer_size_width n = 11 + n bits, num_host_ch n = n + 1 channels, dwc2_type.h comments), not decoded sizes.
    # Should unknown cores (GSNPSID not 0x4F54xxxx) be shown as unknown?
    def test_characterization_a_core_with_an_unknown_id_of_0_shows_specs_version_0_00w_needs_review_165(self):
        row = self.rows()["GD32VF103"]
        self.assertEqual(row[" - specs version"], "0.00W")
        self.assertEqual(row[" - op_mode"], "HNP SRP")


class MarkdownOutputTest(Dwc2Case):
    def test_the_markdown_file_is_the_table_text_followed_by_a_newline(self):
        self.rows()
        text = (pathlib.Path(self.tmp.name) / "dwc2_info.md").read_text(encoding="utf-8")
        self.assertEqual(text, batch0.FakeDataFrame.created[-1].to_markdown() + "\n")

    def test_the_table_has_one_column_per_device_indexed_by_device(self):
        self.rows()
        frame = batch0.FakeDataFrame.created[-1]
        self.assertEqual(frame.index, "Device")
        self.assertEqual([r["Device"] for r in frame.rows], list(self.module.dwc2_reg_value))

    def test_the_committed_dwc2_info_md_matches_the_decoded_table(self):
        # the committed file was written by the real pandas (DataFrame.T.to_markdown); compare cell by cell
        table, labels = parse_markdown(COMMITTED_MD.read_text(encoding="utf-8"))
        rows = self.rows()
        self.assertEqual(list(table), list(rows))
        self.assertEqual(labels, [k.strip() for k in rows[F407_FS] if k != "Device"])
        for device, row in rows.items():
            for key, value in row.items():
                if key != "Device":
                    with self.subTest(device=device, row=key):
                        self.assertEqual(table[device][key.strip()], str(value))

    def test_an_argument_is_rejected_with_exit_status_2(self):
        sys.argv = ["dwc2_info.py", "--bogus"]
        with self.assertRaises(SystemExit) as raised, contextlib.redirect_stderr(io.StringIO()):
            self.module.main()
        self.assertEqual(raised.exception.code, 2)
        self.assertFalse((pathlib.Path(self.tmp.name) / "dwc2_info.md").exists())


class ProgramTest(unittest.TestCase):
    """Runs the script as a program with stand-in click / pandas modules on PYTHONPATH (nothing installed)."""

    def test_running_the_script_writes_dwc2_info_md_into_the_current_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            stubs = pathlib.Path(tmp) / "stubs"
            stubs.mkdir()
            (stubs / "click.py").write_text("", encoding="utf-8")
            (stubs / "pandas.py").write_text(
                "class DataFrame:\n"
                "    def __init__(self, rows): self.rows = rows\n"
                "    def set_index(self, col): self.col = col; return self\n"
                "    @property\n"
                "    def T(self): return self\n"
                "    def to_markdown(self): return '|'.join(r[self.col] for r in self.rows)\n", encoding="utf-8")
            env = dict(os.environ, PYTHONPATH=str(stubs), PYTHONDONTWRITEBYTECODE="1")
            result = subprocess.run([sys.executable, "-B", str(paths.DWC2_INFO)], cwd=tmp, env=env,
                                    capture_output=True, text=True, timeout=60, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
            text = (pathlib.Path(tmp) / "dwc2_info.md").read_text(encoding="utf-8")
            self.assertTrue(text.startswith("BCM2711 (Pi4)|EFM32GG|"))
            self.assertTrue(text.endswith("GD32VF103\n"))


if __name__ == "__main__":
    unittest.main()
