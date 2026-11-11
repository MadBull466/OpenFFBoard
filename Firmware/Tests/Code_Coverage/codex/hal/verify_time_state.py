"""Real-C tick/delay/DMA-state matrix and bounded fault/repair controls."""
import hashlib
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET
import verify_gpio as gate

KNOWN = "large finite delay at ten hertz retains its requested minimum interval"


def build(name, options=()):
    directory = gate.BUILD_ROOT / ("codex-p12-time-" + name)
    gate.run(name + "-configure", ["cmake", "-S", gate.HERE, "-B", directory,
                                  "-DCMAKE_BUILD_TYPE=Debug", *options])
    gate.run(name + "-build", ["cmake", "--build", directory, "--parallel", "2"])
    return directory


def delay_xml(name, directory, unexpected=(), repaired=False):
    executable = directory / ("p12_delay_tests.exe" if sys.platform == "win32" else "p12_delay_tests")
    result = gate.run(name + "-delay-xml", [executable, "--reporters=xml", "--no-version",
                                           "--order-by=rand", "--rand-seed=271"],
                      expected=1 if unexpected or repaired else 0)
    if result.stderr:
        raise RuntimeError(name + ": child stderr, including sanitizer diagnostic")
    root = ET.fromstring(result.stdout)
    cases = root.findall(".//TestCase")
    if len(cases) != 8 or len({c.get("name") for c in cases}) != 8:
        raise RuntimeError(name + ": invalid delay inventory")
    raw_ordinary = []
    for case in cases:
        if case.get("skipped") == "true":
            raise RuntimeError(name + ": unexpected skip")
        totals = case.find("OverallResultsAsserts")
        if totals is None:
            raise RuntimeError(name + ": missing case totals")
        if case.get("name") == KNOWN:
            if case.get("should_fail") != "true" or "P12-TICK-001" not in case.get("description", ""):
                raise RuntimeError(name + ": missing defect marker")
            failures = case.findall(".//Expression[@success='false']")
            if repaired:
                if int(totals.get("failures")) != 0 or totals.get("test_case_success") != "false":
                    raise RuntimeError(name + ": repair must pass assertions and flip only the marker")
            else:
                if int(totals.get("failures")) != 1 or len(failures) != 1:
                    raise RuntimeError(name + ": expected only the minimum-delay defect assertion")
                expression = failures[0]
                if (expression.findtext("Original") or "").strip() != "p12_elapsed_ticks() >= 4294967200ULL":
                    raise RuntimeError(name + ": wrong first known-issue failure")
                if not expression.get("filename", "").endswith("/hal/cases/test_delay.cpp"):
                    raise RuntimeError(name + ": wrong defect assertion location")
                if totals.get("test_case_success") != "true":
                    raise RuntimeError(name + ": expected marker classification")
        else:
            if case.get("should_fail") == "true":
                raise RuntimeError(name + ": unexpected expected-failure case")
            if int(totals.get("failures")):
                raw_ordinary.append(case.get("name"))
    if sorted(raw_ordinary) != sorted(unexpected):
        raise RuntimeError(f"{name}: unexpected failing ordinary cases {raw_ordinary}")
    total = root.find("OverallResultsTestCases")
    if total is None or int(total.get("failures")) != len(unexpected) + int(repaired):
        raise RuntimeError(name + ": mismatched whole-process result")
    return {"ordinary_executed": 7, "unexpected": raw_ordinary,
            "known_issue": "repair assertions pass; marker flips" if repaired else "one intended assertion failure"}


if __name__ == "__main__":
    gate.EVIDENCE = gate.BUILD_ROOT / "codex-p12-time-evidence"
    ev = gate.EVIDENCE
    ev.mkdir(parents=True, exist_ok=True)
    if (ev / "commands.json").exists():
        archive = ev / "previous-attempts"
        archive.mkdir(exist_ok=True)
        attempt = archive / str(len(list(archive.iterdir())) + 1)
        attempt.mkdir()
        for path in ev.iterdir():
            if path.is_file(): shutil.copy2(path, attempt / path.name)
    gate.commands = []
    for script in (Path(__file__), gate.HERE / "verify_gpio.py"):
        shutil.copy2(script, ev / script.name)
    inputs = [p for p in gate.HERE.rglob("*") if p.is_file() and "__pycache__" not in p.parts]
    inputs += [gate.REPO / "Firmware/Tests/Code_Coverage/third_party/doctest.h",
               gate.REPO / "Firmware/Tests/Code_Coverage/support/TestHelpers.h",
               gate.REPO / "Firmware/Tests/Code_Coverage/baseline/test_cases.txt"]
    for device in ("F407VG", "F407VG_DISCO", "F411RE"):
        drivers = gate.REPO / "Firmware/Targets" / device / "Drivers"
        inputs += list((drivers / "STM32F4xx_HAL_Driver/Inc").rglob("*.h"))
        inputs += list((drivers / "CMSIS/Device/ST/STM32F4xx/Include").glob("*.h"))
        inputs += [drivers / "STM32F4xx_HAL_Driver/Src" / name for name in
                   ("stm32f4xx_hal.c", "stm32f4xx_hal_dma.c", "stm32f4xx_hal_gpio.c")]
        inputs.append(drivers / "CMSIS/Include/core_cm4.h")
    hashes = {str(p.relative_to(gate.REPO)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    (ev / "source_sha256.json").write_text(json.dumps(hashes, indent=2) + "\n")
    results = {}
    for name, options in {
        "default": [], "disco": ["-DP12_DEVICE=F407VG_DISCO"],
        "f411": ["-DP12_DEVICE=F411RE"], "san": ["-DP12_SANITIZE=ON"],
        "clang-san": ["-DP12_SANITIZE=ON", "-DCMAKE_C_COMPILER=clang", "-DCMAKE_CXX_COMPILER=clang++"],
    }.items():
        directory = build(name, options)
        gate.run(name + "-ctest", ["ctest", "--test-dir", directory, "--output-on-failure"])
        results[name] = {
            "tick": gate.check_xml(name + "-tick", directory, (), "p12_tick_tests", 11),
            "delay": delay_xml(name, directory),
            "DMA_state": gate.check_xml(name + "-dma", directory, (), "p12_dma_state_tests", 10),
            "GPIO_C": gate.check_xml(name + "-gpio-C", directory),
        }
        if name != "f411":
            results[name]["GPIO_model"] = gate.check_xml(name + "-gpio-model", directory, (), "p12_gpio_model_tests", 36)
        print(name, results[name], flush=True)

    source_dir = gate.REPO / "Firmware/Targets/F407VG/Drivers/STM32F4xx_HAL_Driver/Src"
    core = (source_dir / "stm32f4xx_hal.c").read_text()
    dma = (source_dir / "stm32f4xx_hal_dma.c").read_text()
    mutants = {
        "increment-always-one": (core, "P12_HAL_SOURCE", "uwTick += uwTickFreq;", "uwTick += 1U;", "tick",
            ["real tick increment uses the selected ten millisecond quantum",
             "ten hertz selection requests a hundred millisecond reload and increment"]),
        "frequency-rollback-removed": (core, "P12_HAL_SOURCE", "uwTickFreq = prevTickFreq;", "(void)prevTickFreq;", "tick",
            ["failed time-source configuration leaves the previous frequency selected"]),
        "lower-config-failure-ignored": (core, "P12_HAL_SOURCE",
            "if (HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq)) > 0U)",
            "if ((HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq)) > 0U) && 0U)", "tick",
            ["failed time-source configuration leaves the previous frequency selected",
             "failed time-source configuration does not publish a new interrupt priority"]),
        "delay-returns-early": (core, "P12_HAL_SOURCE", "while((HAL_GetTick() - tickstart) < wait)",
            "while(((HAL_GetTick() - tickstart) < wait) && 0U)", "delay",
            ["delay does not finish before the requested sampled millisecond interval",
             "delay preserves its minimum interval across counter rollover",
             "coarse tick frequency does not let a short delay return before its requested interval",
             "maximum delay does not overflow into an immediate return",
             "large finite delay at the default tick retains its requested minimum interval"]),
        "maximum-delay-addition-overflow": (core, "P12_HAL_SOURCE", "if (wait < HAL_MAX_DELAY)", "if (1)", "delay",
            ["maximum delay does not overflow into an immediate return"]),
        "DMA-wrong-callback-slot": (dma, "P12_DMA_SOURCE", "hdma->XferM1HalfCpltCallback = pCallback;",
            "hdma->XferHalfCpltCallback = pCallback;", "dma",
            ["ready DMA registration replaces only the selected callback slot"]),
        "DMA-unregister-all-keeps-abort": (dma, "P12_DMA_SOURCE",
            "hdma->XferM1HalfCpltCallback = NULL;\n      hdma->XferErrorCallback = NULL;\n      hdma->XferAbortCallback = NULL;",
            "hdma->XferM1HalfCpltCallback = NULL;\n      hdma->XferErrorCallback = NULL;\n      (void)hdma;", "dma",
            ["ready DMA unregister all clears every callback slot"]),
    }
    for name, (text, option, before, after, kind, failed) in mutants.items():
        if text.count(before) != 1: raise RuntimeError(name + ": nonunique mutation anchor")
        scratch = ev / (name + ".c")
        scratch.write_text(text.replace(before, after))
        directory = build("mutant-" + name, ["-D" + option + "=" + str(scratch)])
        if kind == "delay": results[name] = delay_xml(name, directory, failed)
        else:
            executable, count = ("p12_tick_tests", 11) if kind == "tick" else ("p12_dma_state_tests", 10)
            results[name] = gate.check_xml(name, directory, failed, executable, count)
        print(name, results[name], flush=True)

    # Bounded repair discriminator: wider elapsed-time accumulation, only in a
    # scratch source copy. It must pass ordinary assertions and make the known
    # marker flip. This is not a proposed/committed vendor repair or proof of
    # arbitrary scheduling/target equivalence.
    begin = core.index("__weak void HAL_Delay(uint32_t Delay)")
    end = core.index("\n}", begin) + 2
    replacement = """__weak void HAL_Delay(uint32_t Delay)
{
  uint32_t previous = HAL_GetTick();
  uint64_t elapsed = 0U;
  const uint64_t wait = (uint64_t)Delay + (Delay < HAL_MAX_DELAY ? (uint32_t)uwTickFreq : 0U);
  while (elapsed < wait)
  {
    uint32_t current = HAL_GetTick();
    elapsed += (uint32_t)(current - previous);
    previous = current;
  }
}"""
    scratch = ev / "wide-delay-repair.c"
    scratch.write_text(core[:begin] + replacement + core[end:])
    directory = build("scratch-wide-delay", ["-DP12_HAL_SOURCE=" + str(scratch)])
    results["wide-delay-repair"] = delay_xml("wide-delay-repair", directory, repaired=True)
    gate.run("original-baseline", [sys.executable, "-B", gate.REPO / "Firmware/Tests/Code_Coverage/tools/check_baseline.py",
                                  "--build-dir", gate.BUILD_ROOT / "codex-closeout-gate-default"])
    for path, digest in hashes.items():
        if hashlib.sha256((gate.REPO / path).read_bytes()).hexdigest() != digest:
            raise RuntimeError("Source changed during verification: " + path)
    (ev / "summary.json").write_text(json.dumps(results, indent=2) + "\n")
