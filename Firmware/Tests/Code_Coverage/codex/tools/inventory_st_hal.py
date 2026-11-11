"""Read-only inventory of the checked-in ST HAL; no build or device access.

Run from any directory. Output is JSON on stdout. Lexical HAL call occurrences
are discovery evidence, not a compiler call graph or execution coverage.
"""

import hashlib
import json
from pathlib import Path
import re


def inventory(repo):
    targets = {}
    for name in ("F407VG", "F407VG_DISCO", "F411RE"):
        target = repo / "Firmware" / "Targets" / name
        driver = target / "Drivers" / "STM32F4xx_HAL_Driver"
        config = target / "Core" / "Inc" / "stm32f4xx_hal_conf.h"
        core = (driver / "Src" / "stm32f4xx_hal.c").read_text()
        version = [
            int(re.search(r"#define\s+__STM32F4xx_HAL_VERSION_" + part
                          + r"\s+\(0x([0-9A-Fa-f]+)U\)", core).group(1), 16)
            for part in ("MAIN", "SUB1", "SUB2", "RC")
        ]
        files = []
        for path in sorted(driver.rglob("*")):
            if path.is_file() and path.suffix in (".c", ".h"):
                data = path.read_bytes()
                files.append({"path": str(path.relative_to(repo)),
                              "sha256": hashlib.sha256(data).hexdigest(),
                              "lines": len(data.splitlines())})
        targets[name] = {
            "version_components": version,
            "config_path": str(config.relative_to(repo)),
            "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest(),
            "enabled_modules": re.findall(
                r"^\s*#define\s+(HAL_\w+_MODULE_ENABLED)\b",
                config.read_text(), re.MULTILINE),
            "files": files,
        }

    canonical = targets["F407VG"]["files"]
    canonical_sources = {Path(f["path"]).name: f["sha256"] for f in canonical
                         if f["path"].endswith(".c")}
    comparisons = {}
    for name in ("F407VG_DISCO", "F411RE"):
        sources = {Path(f["path"]).name: f["sha256"]
                   for f in targets[name]["files"] if f["path"].endswith(".c")}
        comparisons[name] = {
            "different_common_sources": sorted(
                n for n in canonical_sources.keys() & sources.keys()
                if canonical_sources[n] != sources[n]),
            "missing_from_target": sorted(canonical_sources.keys() - sources.keys()),
            "additional_in_target": sorted(sources.keys() - canonical_sources.keys()),
        }

    calls = {}
    # FFBoard wrappers and board integration only. Tests and vendor sources are
    # deliberately not counted as production caller evidence.
    roots = [repo / "Firmware" / "FFBoard"]
    roots += [repo / "Firmware" / "Targets" / n / "Core" for n in targets]
    for root in roots:
        for path in sorted(root.rglob("*")):
            if not path.is_file() or path.suffix not in (".c", ".cpp", ".h"):
                continue
            if "USB" in path.relative_to(root).parts:
                continue
            raw = path.read_text(errors="replace")
            # Preserve newlines so locations remain meaningful. This is still
            # lexical: declarations, definitions and inactive branches can occur.
            text = re.sub(r"/\*.*?\*/|//[^\n]*",
                          lambda m: "\n" * m.group().count("\n"),
                          raw, flags=re.DOTALL)
            for match in re.finditer(r"\b(HAL_[A-Za-z0-9_]+)\s*\(", text):
                calls.setdefault(match.group(1), []).append({
                    "path": str(path.relative_to(repo)),
                    "line": text.count("\n", 0, match.start()) + 1})

    return {"schema": 1, "kind": "source inventory, not coverage",
            "targets": targets, "source_comparison_to_F407VG": comparisons,
            "lexical_HAL_occurrences": dict(sorted(calls.items()))}


if __name__ == "__main__":
    repository = Path(__file__).resolve().parents[5]
    print(json.dumps(inventory(repository), indent=2))
