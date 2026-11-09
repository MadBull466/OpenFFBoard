"""Verify real-HAL register adapter separately from the real-C storage slice.

All process/format/identity failures are fatal; scratch changes are outside the
checkout. This is orchestration of doctest tests, not a second test framework.
"""
import hashlib
import json
from pathlib import Path
import shutil
import sys
import verify_gpio as gate


def check_model(name, directory, failed=()):
    return gate.check_xml(name, directory, failed, "p12_gpio_model_tests", count=36)


if __name__ == "__main__":
    gate.EVIDENCE = gate.BUILD_ROOT / "codex-p12-register-evidence"
    evidence = gate.EVIDENCE
    evidence.mkdir(parents=True, exist_ok=True)
    if (evidence / "commands.json").exists():
        archive = evidence / "previous-attempts"
        archive.mkdir(exist_ok=True)
        attempt = archive / str(len(list(archive.iterdir())) + 1)
        attempt.mkdir()
        for path in evidence.iterdir():
            if path.is_file():
                shutil.copy2(path, attempt / path.name)
    for script in (Path(__file__), gate.HERE / "verify_gpio.py"):
        shutil.copy2(script, evidence / script.name)
    gate.commands = []
    results = {}
    inputs = [p for p in gate.HERE.rglob("*") if p.is_file() and "__pycache__" not in p.parts]
    for device in ("F407VG", "F407VG_DISCO", "F411RE"):
        drivers = gate.REPO / "Firmware/Targets" / device / "Drivers"
        inputs += list((drivers / "STM32F4xx_HAL_Driver/Inc").rglob("*.h"))
        inputs += list((drivers / "CMSIS/Device/ST/STM32F4xx/Include").glob("*.h"))
        inputs.append(drivers / "STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c")
    inputs += [gate.REPO / "Firmware/Tests/third_party/doctest.h",
               gate.REPO / "Firmware/Tests/baseline/test_cases.txt"]
    hashes = {str(p.relative_to(gate.REPO)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    (evidence / "source_sha256.json").write_text(json.dumps(hashes, indent=2) + "\n")

    matrix = {
        "default": [],
        "disco": ["-DP12_DEVICE=F407VG_DISCO"],
        "san": ["-DP12_SANITIZE=ON"],
        "clang-san": ["-DP12_SANITIZE=ON", "-DCMAKE_C_COMPILER=clang", "-DCMAKE_CXX_COMPILER=clang++"],
    }
    for name, options in matrix.items():
        directory = gate.build("register-" + name, options)
        gate.run(name + "-ctest", ["ctest", "--test-dir", directory, "--output-on-failure"])
        results[name] = {"model": check_model(name, directory),
                         "real_C": gate.check_xml(name + "-C", directory)}
        print(name, results[name], flush=True)

    vendor = gate.REPO / "Firmware/Targets/F407VG/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"
    vendor_mutants = {
        "missing-lock-low-write": (
            "GPIOx->LCKR = GPIO_Pin;", "(void)GPIO_Pin;",
            ["real HAL lock completes the key protocol and protects subsequent pin configuration"]),
        "callback-before-clear": (
            "__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);\n    HAL_GPIO_EXTI_Callback(GPIO_Pin);",
            "HAL_GPIO_EXTI_Callback(GPIO_Pin);\n    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);",
            ["real HAL interrupt handler clears the selected pending bit before its callback"]),
        "clear-extra-pending": (
            "__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);", "__HAL_GPIO_EXTI_CLEAR_IT(0xffffU);",
            ["real HAL interrupt handler clears the selected pending bit before its callback"]),
        "wrong-alternate-bank": (
            "GPIOx->AFR[position >> 3U] = temp;", "GPIOx->AFR[0] = temp;",
            ["real HAL alternate initialization maps pins across both alternate registers"]),
        "deinit-keeps-pull": (
            "GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));",
            "(void)GPIOx;",
            ["real HAL deinitialization releases its own route and selected configuration only"]),
    }
    model = gate.HERE / "model_host/model.cpp"
    model_mutants = {
        "BSRR-read-not-zero": (
            "if (d.kind == Kind::SetReset) result = 0;",
            "if (d.kind == Kind::SetReset) result = 1;",
            ["model write-only BSRR reads zero after a write"]),
        "reset-wins-BSRR": (
            "odr &= ~(value >> 16U);\n        odr |= value & 0xffffU;",
            "odr |= value & 0xffffU;\n        odr &= ~(value >> 16U);",
            ["model BSRR gives set priority over reset for the same pin"]),
        "PR-is-ordinary-RAM": (
            "reg->storage = reg->storage & ~value; return;",
            "reg->storage = value; return;",
            ["model pending clear removes only the written one bits",
             "model pending zero write preserves pending requests",
             "real HAL interrupt handler clears the selected pending bit before its callback"]),
        "configuration-lock-ignored": (
            "if (d.kind == Kind::Config && locks[d.port].locked)",
            "if (d.kind == Kind::Config && locks[d.port].locked && false)",
            ["model valid lock freezes selected configuration fields only",
             "model locks high alternate fields without freezing low alternate fields",
             "model lock preserves nonzero fields and cannot be retargeted before reset",
             "real HAL lock completes the key protocol and protects subsequent pin configuration"]),
    }
    for kind, source, mutants, option in (
            ("HAL", vendor, vendor_mutants, "P12_GPIO_SOURCE"),
            ("model", model, model_mutants, "P12_MODEL_SOURCE")):
        original = source.read_text()
        for name, (before, after, failed) in mutants.items():
            if original.count(before) != 1:
                raise RuntimeError(name + ": nonunique mutation anchor")
            scratch = evidence / (name + source.suffix)
            scratch.write_text(original.replace(before, after))
            directory = gate.build("register-mutant-" + name, ["-D" + option + "=" + str(scratch)])
            results[name] = {"kind": kind, **check_model(name, directory, failed)}
            print(name, results[name], flush=True)

    gate.run("original-baseline", [sys.executable, "-B", gate.REPO / "Firmware/Tests/tools/check_baseline.py",
                                  "--build-dir", gate.BUILD_ROOT / "codex-closeout-gate-default"])
    for path, digest in hashes.items():
        if hashlib.sha256((gate.REPO / path).read_bytes()).hexdigest() != digest:
            raise RuntimeError("Source changed during verification: " + path)
    (evidence / "summary.json").write_text(json.dumps(results, indent=2) + "\n")
