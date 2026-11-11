"""Reproduce the bounded P12 GPIO matrix and scratch mutation controls.

Writes only to the required sibling ffb-test-build/codex-p12-* directories.
No downloads, hardware access, skips, sleeps or vendor-file edits.
"""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
BUILD_ROOT = REPO.parent / "ffb-test-build"
EVIDENCE = BUILD_ROOT / "codex-p12-gpio-evidence"


def run(label, argv, expected=0):
    record = {"label": label, "cwd": str(REPO), "argv": list(map(str, argv))}
    commands.append(record)
    (EVIDENCE / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
    result = subprocess.run(record["argv"], cwd=REPO, capture_output=True, timeout=120)
    (EVIDENCE / (label + ".stdout")).write_bytes(result.stdout)
    (EVIDENCE / (label + ".stderr")).write_bytes(result.stderr)
    record["returncode"] = result.returncode
    (EVIDENCE / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
    if result.returncode != expected:
        raise RuntimeError(f"{label}: exit {result.returncode}, expected {expected}; see evidence")
    return result


def build(name, options):
    directory = BUILD_ROOT / ("codex-p12-gpio-" + name)
    run(name + "-configure", ["cmake", "-S", HERE, "-B", directory,
                              "-DCMAKE_BUILD_TYPE=Debug", *options])
    run(name + "-build", ["cmake", "--build", directory, "--parallel", "2"])
    return directory


def check_xml(name, directory, expected_failed=(), executable_name="p12_gpio_tests", count=8):
    executable = directory / executable_name
    if sys.platform == "win32":
        executable = executable.with_suffix(".exe")
    result = run(name + "-xml", [executable, "--reporters=xml", "--no-version",
                                  "--order-by=rand", "--rand-seed=271"],
                 expected=1 if expected_failed else 0)
    if result.stderr:
        raise RuntimeError(name + ": unexpected child stderr (including sanitizer diagnostics)")
    root = ET.fromstring(result.stdout)
    cases = root.findall(".//TestCase")
    if len(cases) != count or any(c.get("skipped") == "true" for c in cases):
        raise RuntimeError(name + ": incorrect number of unskipped cases")
    if len({c.get("name") for c in cases}) != count:
        raise RuntimeError(name + ": duplicate or missing identity")
    failed = []
    for case in cases:
        totals = case.find("OverallResultsAsserts")
        if totals is None:
            raise RuntimeError(name + ": missing case result")
        if int(totals.get("failures", "0")):
            failed.append(case.get("name"))
    if sorted(failed) != sorted(expected_failed):
        raise RuntimeError(f"{name}: unexpected failed identities {failed}")
    return {"executed": len(cases), "failed": failed}


if __name__ == "__main__":
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    if (EVIDENCE / "commands.json").exists():
        archive = EVIDENCE / "previous-attempts"
        archive.mkdir(exist_ok=True)
        attempt = archive / str(len(list(archive.iterdir())) + 1)
        attempt.mkdir()
        for path in EVIDENCE.iterdir():
            if path.is_file():
                shutil.copy2(path, attempt / path.name)
    shutil.copy2(__file__, EVIDENCE / "verify_gpio.py")
    commands = []
    results = {}
    sources = list(HERE.rglob("*"))
    vendor = REPO / "Firmware/Targets/F407VG/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"
    sources += [REPO / "Firmware/Tests/Code_Coverage/third_party/doctest.h",
                REPO / "Firmware/Tests/Code_Coverage/baseline/test_cases.txt"]
    for device in ("F407VG", "F407VG_DISCO", "F411RE"):
        drivers = REPO / "Firmware/Targets" / device / "Drivers"
        sources += list((drivers / "STM32F4xx_HAL_Driver/Inc").rglob("*.h"))
        sources += list((drivers / "CMSIS/Device/ST/STM32F4xx/Include").glob("*.h"))
        sources += [drivers / "STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"]
    hashes = {str(p.relative_to(REPO)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sources if p.is_file()}
    (EVIDENCE / "source_sha256.json").write_text(json.dumps(hashes, indent=2) + "\n")
    matrix = {
        "default": [],
        "assert": ["-DP12_FULL_ASSERT=ON"],
        "disco": ["-DP12_DEVICE=F407VG_DISCO"],
        "f411": ["-DP12_DEVICE=F411RE"],
        "san": ["-DP12_SANITIZE=ON"],
        "clang-san": ["-DP12_SANITIZE=ON", "-DCMAKE_C_COMPILER=clang",
                      "-DCMAKE_CXX_COMPILER=clang++"],
    }
    for name, options in matrix.items():
        directory = build(name, options)
        run(name + "-ctest", ["ctest", "--test-dir", directory, "--output-on-failure"])
        results[name] = check_xml(name, directory)
        print(name, results[name], flush=True)

    mutants = {
        "read-always-set": (
            "if((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET)",
            "if(((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET) || 1)",
            ["unrelated high inputs do not set the selected low input"]),
        "wrong-reset-half": (
            "GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U;",
            "GPIOx->BSRR = (uint32_t)GPIO_Pin << 15U;",
            ["resetting each output requests only its named BSRR reset bit"]),
        "set-extra-pin": (
            "GPIOx->BSRR = GPIO_Pin;", "GPIOx->BSRR = GPIO_Pin | 1U;",
            ["setting each output requests only its named BSRR set bit"]),
        "toggle-always-set": (
            "GPIOx->BSRR = ((odr & GPIO_Pin) << GPIO_NUMBER) | (~odr & GPIO_Pin);",
            "GPIOx->BSRR = GPIO_Pin; (void)odr;",
            ["toggling a selected high output requests its reset bit",
             "a mixed toggle requests distinct set and reset bits only for selected outputs"]),
    }
    original = vendor.read_text()
    for name, (before, after, failed) in mutants.items():
        if original.count(before) != 1:
            raise RuntimeError(name + ": mutation anchor is not unique")
        copy = EVIDENCE / (name + ".c")
        copy.write_text(original.replace(before, after))
        directory = build("mutant-" + name, ["-DP12_GPIO_SOURCE=" + str(copy)])
        results[name] = check_xml(name, directory, failed)
        print(name, results[name], flush=True)

    baseline = BUILD_ROOT / "codex-closeout-gate-default"
    run("original-baseline", [sys.executable, "-B", REPO / "Firmware/Tests/Code_Coverage/tools/check_baseline.py",
                              "--build-dir", baseline])
    for path, digest in hashes.items():
        if hashlib.sha256((REPO / path).read_bytes()).hexdigest() != digest:
            raise RuntimeError("Source changed during verification: " + path)
    (EVIDENCE / "summary.json").write_text(json.dumps(results, indent=2) + "\n")
