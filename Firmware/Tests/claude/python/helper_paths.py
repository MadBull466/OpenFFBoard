"""Shared helpers of the Python helper-script tests (stdlib only).

Paths of the scripts under test and a loader that imports a script from its path without writing __pycache__
into the repository.
"""
import importlib.util
import pathlib
import sys

sys.dont_write_bytecode = True  # Never write __pycache__ next to the scripts (also run with python3 -B)

FIRMWARE_DIR = pathlib.Path(__file__).resolve().parents[3]
GENERATE_MEMORY = FIRMWARE_DIR / "scripts" / "generate_memory.py"
MEMORY_MAP_CSV = FIRMWARE_DIR / "scripts" / "memory_map.csv"
EEPROM_ADDRESSES_C = FIRMWARE_DIR / "FFBoard" / "UserExtensions" / "Src" / "eeprom_addresses.c"
EEPROM_ADDRESSES_H = FIRMWARE_DIR / "FFBoard" / "UserExtensions" / "Inc" / "eeprom_addresses.h"
DWC2_INFO = FIRMWARE_DIR / "FFBoard" / "USB" / "portable" / "synopsys" / "dwc2" / "dwc2_info.py"


def load_script(path, name):
    """Imports a script as a module (its `if __name__ == "__main__"` part does not run)."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_text_lf(path):
    """File content with LF line ends (the scripts write text mode files: CRLF on Windows)."""
    return pathlib.Path(path).read_text(encoding="utf-8").replace("\r\n", "\n")
