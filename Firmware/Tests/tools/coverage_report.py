#!/usr/bin/env python3
"""Line coverage of the firmware per file, module and vendor tier, from plain gcov.

Build the tests with --coverage in a separate build directory, run them, then:

  cmake -S Firmware/Tests -B ../ffb-test-build/cov -DCMAKE_C_FLAGS=--coverage \
        -DCMAKE_CXX_FLAGS=--coverage -DCMAKE_EXE_LINKER_FLAGS=--coverage
  cmake --build ../ffb-test-build/cov && ctest --test-dir ../ffb-test-build/cov
  python3 Firmware/Tests/tools/coverage_report.py --build-dir ../ffb-test-build/cov

The report (Markdown, stdout) separates:
  firmware   FFBoard/** without the vendor code and the out-of-scope drivers
  tier A     eeprom.c and the FreeRTOS C++ wrappers (c*.cpp, Inc/*.hpp)
  tier B     TinyUSB (FFBoard/USB/**)
  tier C     CMSIS-DSP (Libraries/CMSIS-DSP/**)
  dsp        the in-scope CMSIS-DSP modules of ffb_dsp_tests (phase P11, Tests/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md). Own section,
             built from the .gcda files of the CMake targets ffb_cmsis_dsp, ffb_cmsis_dsp_f16 and ffb_dsp_tests ONLY, so the
             numbers of tier C (the other executables) stay comparable. NOT part of the in-scope coverage ladder.
  driver     the driver track (Tests/claude/plans/DRIVER_TEST_PLAN.md): TMC4671.cpp, TMC4671_configs.cpp, ODriveCAN.cpp, VescCAN.cpp, TMCDebugBridge.cpp (P13).
             Own section; NOT part of the in-scope coverage ladder (its total is unchanged by the driver track)
Firmware/Tests/, the remaining out-of-scope drivers, MIDI, the rest of Libraries/ and Targets/ are not reported.

"In-scope sources" is the denominator of the coverage ladder in Tests/claude/plans/TEST_PLAN.md: every .c/.cpp in
FFBoard/Src and FFBoard/UserExtensions/Src except the out-of-scope drivers and MIDI (tier A included).
gcov only knows the files that are compiled into a test executable. For the other files the number of
executable lines is ESTIMATED from their code lines, scaled by the ratio gcov lines / code lines of the
compiled files. Preprocessor branches are not evaluated, so the estimate is approximate.

Hardware-only code (behaviour that can only be observed on the board: register and clock setup, busy waits on
peripheral flags, interrupt timing) is not tested. It is listed in tools/coverage_exclusions.txt with a reason and
is left out of the covered AND the executable line counts. Unreachable or defensive code is NOT excluded.
Use --no-exclusions to see the raw numbers.

Only the Python standard library and gcov are used. gcov runs in a temporary directory with --stdout,
nothing is written to the build or source tree.
"""

import argparse
import collections
import json
import os
import re
import shlex
import subprocess
import sys
import tempfile

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_EXCLUSIONS = os.path.join(TOOLS_DIR, "coverage_exclusions.txt")
COVERAGE_TARGET = 85.0  # % of in-scope lines without hardware-only code (Tests/claude/plans/TEST_PLAN.md)
DEFAULT_FIRMWARE_DIR = os.path.normpath(os.path.join(TOOLS_DIR, "..", ".."))

# Driver and MIDI code that is out of scope (Tests/claude/plans/PROGRESS.md, "Out-of-scope driver files")
OUT_OF_SCOPE = {"MotorPWM", "RmdMotorCAN", "MotorSimplemotion", "MidiMain", "MidiHandler"}
# The three drivers of the driver track (ffb_driver_tests): reported in their own tier, not in the ladder
DRIVER_TRACK = {"TMC4671", "TMC4671_configs", "ODriveCAN", "VescCAN", "TMCDebugBridge"}  # TMCDebugBridge: P13 (in scope since 2026-11-10)
# ffb_dsp_tests: the CMSIS-DSP sources are built by these CMake targets (object directories CMakeFiles/<target>.dir)
DSP_TARGET_DIRS = ("/ffb_cmsis_dsp.dir/", "/ffb_cmsis_dsp_f16.dir/", "/ffb_dsp_tests.dir/")
# In-scope CMSIS-DSP modules (CMakeLists.txt, ffb_dsp_tests): every arm_*.c of these directories, plus CommonTables
# (arm_common_tables.c, arm_common_tables_f16.c) and the four BasicMath helpers
DSP_MODULE_DIRS = ["FastMathFunctions", "ControllerFunctions", "InterpolationFunctions", "StatisticsFunctions", "FilteringFunctions"]
DSP_COMMON_TABLES = ["arm_common_tables.c", "arm_common_tables_f16.c"]
DSP_BASICMATH_HELPERS = ["arm_abs_q15.c", "arm_abs_q31.c", "arm_add_f32.c", "arm_add_f16.c"]
# FreeRTOS C++ wrappers (vendor tier A); their headers are FFBoard/Inc/*.hpp
RTOS_WRAPPERS = {
	"cthread", "csemaphore", "cmutex", "cqueue", "ctimer", "ctasklet", "cevent_groups", "cworkqueue",
	"cmem_pool", "cread_write_lock", "ccondition_variable", "ctickhook",
}
MODULES = collections.OrderedDict([
	("Command system", ["CmdParser", "CommandHandler", "CommandInterface", "FFBoardMainCommandThread",
		"HidCommandInterface", "CanCommandInterface"]),
	("Transport", ["CDCcomm", "UART", "UartHandler", "CAN", "CanHandler", "CANPort2B", "global_callbacks",
		"USBdevice", "UsbHidHandler"]),
	("FFB", ["HidFFB", "EffectsCalculator", "SerialFFB"]),
	("Axis", ["Axis", "AxesManager"]),
	("Filters / math", ["Filters", "FastAvg", "CRC"]),
	("Error handling", ["ErrorHandler"]),
	("Persistence", ["PersistentStorage", "flash_helpers", "eeprom_addresses", "eeprom"]),
	("System", ["SystemCommands", "cppmain", "ledEffects", "voltagesense"]),
	("Main classes", ["FFBoardMain", "FFBHIDMain", "FFBHIDMain_commmands", "FFBWheel", "FFBJoystick",
		"FFBHIDExt", "CustomMain", "CanInputMain", "CanBridge", "mainclass_chooser", "ClassChooser"]),
	("Button sources", ["ButtonSource", "SelectableInputs", "LocalButtons", "SPIButtons", "ShifterAnalog",
		"CanButtons", "PCF8574", "ButtonSources"]),
	("Analog sources", ["AnalogSource", "AnalogAxisProcessing", "AdcHandler", "LocalAnalog", "CanAnalog",
		"ADS111X", "AnalogSources"]),
	("Device interfaces", ["Encoder", "MotorDriver"]),
	("Encoders", ["EncoderLocal", "EncoderSSI", "EncoderBissC", "MtEncoderSPI", "EncoderSources"]),
	("Bus wrappers", ["SPI", "SpiHandler", "I2C", "I2CHandler", "TimerHandler", "ExtiHandler"]),
	("Buffers", ["ringbuffer", "ringbufferwrapper"]),
	("USB descriptors", ["usb_descriptors", "usb_hid_1ffb_desc", "usb_hid_2ffb_desc", "usb_hid_gamepad"]),
	("RTOS wrappers", sorted(RTOS_WRAPPERS)),
])
MODULE_OF = {stem: module for module, stems in MODULES.items() for stem in stems}
SOURCE_DIRS = ("FFBoard/Src", "FFBoard/UserExtensions/Src")


def stem_of(rel):
	return os.path.splitext(os.path.basename(rel))[0]


def classify(rel):
	"""Tier of a path relative to Firmware/, or None if it is not reported."""
	rel = rel.replace("\\", "/")
	if rel.startswith("Tests/") or rel.startswith("../"):
		return None
	if rel.startswith("Libraries/CMSIS-DSP/"):
		return "C"
	if rel.startswith("Libraries/") or rel.startswith("Targets/"):
		return None
	if rel.startswith("FFBoard/USB/"):
		return "B"
	stem = stem_of(rel)
	if stem in OUT_OF_SCOPE:
		return None
	if stem in DRIVER_TRACK:
		return "driver"
	if stem == "eeprom" or stem in RTOS_WRAPPERS or (rel.startswith("FFBoard/Inc/") and rel.endswith(".hpp")):
		return "A"
	if rel.startswith("FFBoard/"):
		return "firmware"
	return None


def module_of(rel, tier):
	if tier == "B":
		return "TinyUSB"
	if tier == "C":
		return "CMSIS-DSP"
	if tier == "driver":
		return "Driver track"
	if rel.endswith(".hpp"):
		return "RTOS wrappers"
	return MODULE_OF.get(stem_of(rel), "other")


def is_dsp_gcda(path):
	"""True for a .gcda file of the ffb_dsp_tests targets (they are reported in the dsp tier, not in tier C)."""
	path = path.replace("\\", "/")
	return any(d in path for d in DSP_TARGET_DIRS)


def dsp_in_scope_sources(fw):
	"""Repo relative (to Firmware/) paths of the in-scope CMSIS-DSP sources, same rule as CMakeLists.txt."""
	base = "Libraries/CMSIS-DSP/Source/"
	result = []
	for d in DSP_MODULE_DIRS:
		path = os.path.join(fw, base, d)
		if os.path.isdir(path):
			result += [base + d + "/" + n for n in sorted(os.listdir(path)) if n.startswith("arm_") and n.endswith(".c")]
	result += [base + "CommonTables/" + n for n in DSP_COMMON_TABLES if os.path.isfile(os.path.join(fw, base, "CommonTables", n))]
	result += [base + "BasicMathFunctions/" + n for n in DSP_BASICMATH_HELPERS if os.path.isfile(os.path.join(fw, base, "BasicMathFunctions", n))]
	return result


def is_source(rel):
	return rel.endswith(".c") or rel.endswith(".cpp")


def run_gcov(gcov_cmd, gcda_files, notes_sources=None):
	"""Merged line and function data: {abs path: {"lines": {n: count}, "functions": {name: count}}}."""
	data = {}
	with tempfile.TemporaryDirectory() as tmp:
		for gcda in gcda_files:
			proc = subprocess.run(gcov_cmd + ["--json-format", "--stdout", gcda], cwd=tmp,
				stdout=subprocess.PIPE, stderr=subprocess.PIPE)
			if proc.returncode != 0:
				raise RuntimeError("gcov failed for %s:\n%s" % (gcda, proc.stderr.decode(errors="replace")))
			valid_documents = 0
			for line in proc.stdout.decode("utf-8", errors="replace").splitlines():
				if not line.strip():
					continue
				doc = json.loads(line)
				if not isinstance(doc, dict) or not isinstance(doc.get("files"), list):
					raise ValueError("invalid gcov document for %s: expected files list" % gcda)
				valid_documents += 1
				base = doc.get("current_working_directory", "")
				for f in doc.get("files", []):
					path = os.path.normpath(os.path.join(base, f["file"]))
					entry = data.setdefault(path, {"lines": {}, "functions": {}})
					for ln in f.get("lines", []):
						n = ln["line_number"]
						entry["lines"][n] = entry["lines"].get(n, 0) + ln["count"]
					for fn in f.get("functions", []):
						name = fn["name"]
						entry["functions"][name] = entry["functions"].get(name, 0) + fn["execution_count"]
			if not valid_documents:
				raise ValueError("invalid gcov output for %s: no JSON documents" % gcda)
			# Successful gcov on notes establishes zero source lines even when files=[] (data-only objects).
			if notes_sources and gcda in notes_sources:
				data.setdefault(notes_sources[gcda], {"lines": {}, "functions": {}})
	return data


def strip_comments_and_strings(text):
	out = []
	i, n = 0, len(text)
	while i < n:
		c = text[i]
		if text.startswith("//", i):
			j = text.find("\n", i)
			i = n if j < 0 else j
		elif text.startswith("/*", i):
			j = text.find("*/", i + 2)
			j = n if j < 0 else j + 2
			out.append("\n" * text.count("\n", i, j))
			i = j
		elif c in "\"'":
			j = i + 1
			while j < n and text[j] != c and text[j] != "\n":
				j += 2 if text[j] == "\\" else 1
			out.append(c + c)
			i = j + 1
		else:
			out.append(c)
			i += 1
	return "".join(out)


def read_exclusions(path, fw):
	"""Hardware-only exclusions: {rel path: [(first, last, reason)]}, last None = whole file.
	Format, one per line: <path relative to Firmware/>:<first>-<last> <reason>  or  <path>:* <reason>"""
	result = collections.OrderedDict()
	if not path or not os.path.isfile(path):
		return result
	with open(path, encoding="utf-8") as f:
		for number, line in enumerate(f, 1):
			line = line.strip()
			if not line or line.startswith("#"):
				continue
			m = re.match(r"^(\S+):(\*|(\d+)(?:-(\d+))?)\s+(.+)$", line)
			if not m:
				raise ValueError("%s:%d: expected '<file>:<first>-<last> <reason>' or '<file>:* <reason>'" % (path, number))
			rel, whole, first, last, reason = m.group(1), m.group(2) == "*", m.group(3), m.group(4), m.group(5)
			if not os.path.isfile(os.path.join(fw, rel)):
				raise ValueError("%s:%d: no such file %s" % (path, number, rel))
			if whole:
				entry = (1, None, reason)
			else:
				entry = (int(first), int(last or first), reason)
				if entry[1] < entry[0]:
					raise ValueError("%s:%d: empty range" % (path, number))
			result.setdefault(rel, []).append(entry)
	return result


def is_excluded(exclusions, rel, line):
	return any(first <= line and (last is None or line <= last) for first, last, _ in exclusions.get(rel, []))


def code_lines(path):
	"""Line numbers with code: not blank, not a comment, not a preprocessor line, not only braces and
	not part of a file scope initializer (tables like "const uint8_t desc[] = { ... };" are data)."""
	with open(path, encoding="utf-8", errors="replace") as f:
		text = strip_comments_and_strings(f.read())
	text = re.sub(r"\boperator\s*[^\s\w(]+\s*\(", "operator(", text)  # operator=, operator== ...
	numbers = set()
	depth = 0
	parens = 0
	assignment = False  # '=' seen at file scope (not in a parameter list) in the current declaration
	initializer = False
	for number, line in enumerate(text.splitlines(), 1):
		s = line.strip()
		if s.startswith("#"):
			continue
		data = initializer
		for ch in s:
			if ch == "(":
				parens += 1
			elif ch == ")":
				parens = max(parens - 1, 0)
			elif ch == "=" and depth == 0 and parens == 0:
				assignment = True
			elif ch == ";" and depth == 0:
				assignment = False
			elif ch == "{":
				if depth == 0 and assignment:
					initializer = data = True
				depth += 1
			elif ch == "}":
				depth = max(depth - 1, 0)
				if depth == 0:
					initializer = False
		if data or not s or re.fullmatch(r"[{}();,\s]*", s):
			continue
		numbers.add(number)
	return numbers


def pct(covered, total):
	return "n/a" if total == 0 else "%.1f %%" % (100.0 * covered / total)


def main():
	parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
	parser.add_argument("--build-dir", required=True, help="build directory of a --coverage build after a test run")
	parser.add_argument("--firmware-dir", default=DEFAULT_FIRMWARE_DIR, help="the Firmware/ directory")
	parser.add_argument("--gcov", default="gcov", help='gcov command, e.g. "llvm-cov gcov" for clang builds')
	parser.add_argument("--headers", action="store_true", help="also list header files in the per-file table")
	parser.add_argument("--exclusions", default=DEFAULT_EXCLUSIONS, help="hardware-only exclusion list")
	parser.add_argument("--no-exclusions", action="store_true", help="count hardware-only code too (raw numbers)")
	args = parser.parse_args()

	fw = os.path.abspath(args.firmware_dir)
	exclusions = {} if args.no_exclusions else read_exclusions(args.exclusions, fw)
	excluded_lines = collections.OrderedDict()  # rel -> (excluded executable lines, of which covered)
	gcda_files = []
	for root, _, files in os.walk(args.build_dir):
		gcda_files += [os.path.abspath(os.path.join(root, f)) for f in files if f.endswith(".gcda")]
	if not gcda_files:
		sys.stderr.write("No .gcda files in %s. Build with --coverage and run the tests first.\n" % args.build_dir)
		return 2
	# ffb_dsp_tests data is kept apart: tier C keeps its meaning (the CMSIS-DSP functions the firmware tests reach)
	dsp_gcda = [g for g in gcda_files if is_dsp_gcda(g)]
	gcda_files = [g for g in gcda_files if not is_dsp_gcda(g)]
	# Notes-only objects may contain executable code that has never run. Ask gcov for their
	# executable lines instead of inferring data-only status from the existence of .gcno.
	# Standalone DSP static-library objects retain their documented linked-data-only scope.
	notes_sources = {}
	for root, _, files in os.walk(args.build_dir):
		for name in files:
			if not name.endswith(".gcno"):
				continue
			note = os.path.abspath(os.path.join(root, name))
			if is_dsp_gcda(note) or os.path.isfile(note[:-5] + ".gcda"):
				continue
			for directory in SOURCE_DIRS:
				marker = "/" + directory + "/"
				if marker not in note.replace("\\", "/"):
					continue
				rel = directory + "/" + note.replace("\\", "/").rsplit(marker, 1)[1][:-5]
				if is_source(rel) and classify(rel) in ("firmware", "A", "driver"):
					notes_sources[note] = os.path.normpath(os.path.join(fw, rel))
	try:
		data = run_gcov(shlex.split(args.gcov), sorted(gcda_files) + sorted(notes_sources), notes_sources)
		dsp_data = run_gcov(shlex.split(args.gcov), sorted(dsp_gcda)) if dsp_gcda else {}
	except (OSError, ValueError, RuntimeError) as exc:
		sys.stderr.write("Coverage report incomplete: %s\n" % exc)
		return 1

	rows = []  # (tier, module, rel, covered, executable, fn_exec, fn_total)
	for path, entry in data.items():
		rel = os.path.relpath(path, fw).replace("\\", "/")
		tier = classify(rel)
		if tier is None:
			continue
		lines = entry["lines"]
		if rel in exclusions:
			dropped = {n: c for n, c in lines.items() if is_excluded(exclusions, rel, n)}
			lines = {n: c for n, c in lines.items() if n not in dropped}
			excluded_lines[rel] = (len(dropped), sum(1 for c in dropped.values() if c > 0))
		fns = entry["functions"]
		rows.append((tier, module_of(rel, tier), rel, sum(1 for c in lines.values() if c > 0), len(lines),
			sum(1 for c in fns.values() if c > 0), len(fns)))
	rows.sort(key=lambda r: (r[0] != "firmware", r[0], list(MODULES).index(r[1]) if r[1] in MODULES else 99, r[2]))
	measured = {r[2]: r for r in rows}

	print("## Coverage report\n")
	print("Build directory: `%s`, %d .gcda files\n" % (args.build_dir, len(gcda_files)))

	print("### Per file (compiled, %s)\n" % ("sources and headers" if args.headers else "sources only"))
	print("| Tier | Module | File | Lines | Covered | Functions |")
	print("|---|---|---|---|---|---|")
	for tier, module, rel, cov, exe, fe, ft in rows:
		if exe == 0 or tier == "driver" or (not args.headers and not is_source(rel)):
			continue  # the driver track has its own section below
		print("| %s | %s | `%s` | %d / %d | %s | %d / %d |" % (tier, module, rel, cov, exe, pct(cov, exe), fe, ft))

	print("\n### Per tier (compiled files)\n")
	print("| Tier | Kind | Files | Lines covered / executable | Covered |")
	print("|---|---|---|---|---|")
	for tier in ("firmware", "A", "B", "C", "driver"):
		for kind, pred in (("sources", is_source), ("headers", lambda r: not is_source(r))):
			sel = [r for r in rows if r[0] == tier and pred(r[2]) and r[4] > 0]
			if sel:
				cov = sum(r[3] for r in sel)
				exe = sum(r[4] for r in sel)
				print("| %s | %s | %d | %d / %d | %s |" % (tier, kind, len(sel), cov, exe, pct(cov, exe)))

	# Coverage ladder: all in-scope firmware sources, estimates for files gcov does not know
	in_scope = []
	for d in SOURCE_DIRS:
		for name in sorted(os.listdir(os.path.join(fw, d))):
			rel = d + "/" + name
			if is_source(rel) and classify(rel) in ("firmware", "A"):
				in_scope.append(rel)
	compiled = [rel for rel in in_scope if rel in measured]
	not_compiled = [rel for rel in in_scope if rel not in measured]
	code = {}
	for rel in in_scope:
		numbers = code_lines(os.path.join(fw, rel))
		kept = set(n for n in numbers if not is_excluded(exclusions, rel, n))
		if rel in exclusions and rel not in measured:
			excluded_lines[rel] = (len(numbers) - len(kept), 0)
		code[rel] = len(kept)
	gcov_lines = sum(measured[r][4] for r in compiled)
	code_compiled = sum(code[r] for r in compiled)
	ratio = gcov_lines / code_compiled if code_compiled else 1.0
	estimated = {r: int(round(code[r] * ratio)) for r in not_compiled}
	covered = sum(measured[r][3] for r in compiled)
	total = gcov_lines + sum(estimated.values())

	print("\n### In-scope firmware sources (coverage ladder)\n")
	print("- In-scope sources: %d (`FFBoard/Src`, `FFBoard/UserExtensions/Src`, without drivers and MIDI, tier A included)" % len(in_scope))
	print("- Compiled into the tests: %d files, %d of %d executable lines covered (%s)" % (
		len(compiled), covered, gcov_lines, pct(covered, gcov_lines)))
	print("- Not compiled: %d files, %d code lines, estimated %d executable lines (ratio gcov/code lines of the compiled files: %.2f)" % (
		len(not_compiled), sum(code[r] for r in not_compiled), sum(estimated.values()), ratio))
	print("- **All in-scope sources: %d of about %d executable lines covered (about %s)**" % (covered, total, pct(covered, total)))
	fw_only = [r for r in in_scope if classify(r) == "firmware"]
	fw_cov = sum(measured[r][3] for r in fw_only if r in measured)
	fw_total = sum(measured[r][4] if r in measured else estimated[r] for r in fw_only)
	print("- Without tier A (firmware only): %d of about %d (about %s)" % (fw_cov, fw_total, pct(fw_cov, fw_total)))
	if args.no_exclusions:
		print("- Hardware-only exclusions: NOT applied (--no-exclusions)")
	else:
		print("- Hardware-only code left out of these numbers: %d files, %d lines (see below). Target: %.0f %% of all in-scope sources" % (
			len(excluded_lines), sum(v[0] for v in excluded_lines.values()), COVERAGE_TARGET))

	if excluded_lines:
		print("\n### Hardware-only code (not counted; lines: gcov executable lines for compiled files, code lines otherwise)\n")
		print("| File | Lines | Of which covered anyway | Ranges and reasons |")
		print("|---|---|---|---|")
		for rel, (n, c) in excluded_lines.items():
			reasons = "; ".join("%s: %s" % ("whole file" if last is None else "%d-%d" % (first, last), reason)
				for first, last, reason in exclusions[rel])
			print("| `%s` | %d | %d | %s |" % (rel, n, c, reasons))

	# Driver track: own tier, outside the ladder above
	print("\n### Driver track (ffb_driver_tests; not part of the coverage ladder)\n")
	print("| File | Lines | Covered | Functions |")
	print("|---|---|---|---|")
	driver_cov = driver_exe = 0
	for name in sorted(DRIVER_TRACK):
		rel = "FFBoard/UserExtensions/Src/%s.cpp" % name
		row = measured.get(rel)
		if row is None:
			print("| `%s` | not compiled | n/a | n/a |" % rel)
			continue
		tier, module, rel, cov, exe, fe, ft = row
		driver_cov += cov
		driver_exe += exe
		print("| `%s` | %d / %d | %s | %d / %d |" % (rel, cov, exe, pct(cov, exe), fe, ft))
	print("| **Total** | %d / %d | %s | |" % (driver_cov, driver_exe, pct(driver_cov, driver_exe)))

	# dsp tier: ffb_dsp_tests (own data, see the module docstring)
	print("\n### CMSIS-DSP in scope (ffb_dsp_tests; not part of the coverage ladder, not part of tier C)\n")
	dsp_rows = {}
	for path, entry in dsp_data.items():
		rel = os.path.relpath(path, fw).replace("\\", "/")
		if rel.startswith("Libraries/CMSIS-DSP/Source/"):
			lines = entry["lines"]
			fns = entry["functions"]
			dsp_rows[rel] = (sum(1 for c in lines.values() if c > 0), len(lines),
				sum(1 for c in fns.values() if c > 0), len(fns))
	dsp_sources = dsp_in_scope_sources(fw)
	if not dsp_data:
		print("No ffb_dsp_tests coverage data (ffb_dsp_tests not built or not run).")
	else:
		print("| Module | Files with data | Lines covered / executable | Covered | Functions | In-scope files without data |")
		print("|---|---|---|---|---|---|")
		total = [0, 0, 0, 0, 0, 0]
		missing = []
		for d in DSP_MODULE_DIRS + ["CommonTables", "BasicMathFunctions"]:
			mod = [r for r in dsp_sources if r.split("/")[3] == d]
			have = [r for r in mod if r in dsp_rows and dsp_rows[r][1] > 0]
			none = [r for r in mod if r not in dsp_rows]
			missing += none
			cov = sum(dsp_rows[r][0] for r in have)
			exe = sum(dsp_rows[r][1] for r in have)
			fe = sum(dsp_rows[r][2] for r in have)
			ft = sum(dsp_rows[r][3] for r in have)
			for i, v in enumerate((len(have), cov, exe, fe, ft, len(none))):
				total[i] += v
			print("| %s | %d | %d / %d | %s | %d / %d | %d |" % (d, len(have), cov, exe, pct(cov, exe), fe, ft, len(none)))
		print("| **Total** | %d | %d / %d | %s | %d / %d | %d |" % (total[0], total[1], total[2], pct(total[1], total[2]), total[3], total[4], total[5]))
		print("\nFiles with data are the sources linked into ffb_dsp_tests (a static library object that no test references is not "
			"linked and has no data, so its lines are NOT in the executable line count above: the percentage is over linked files only). "
			"In-scope files without data: %d of %d." % (len(missing), len(dsp_sources)))

	print("\n### Not compiled (0 %, estimated executable lines)\n")
	print("| Module | File | Code lines | Est. executable lines |")
	print("|---|---|---|---|")
	for rel in sorted(not_compiled, key=lambda r: (list(MODULES).index(module_of(r, classify(r))) if module_of(r, classify(r)) in MODULES else 99, r)):
		print("| %s | `%s` | %d | %d |" % (module_of(rel, classify(rel)), rel, code[rel], estimated[rel]))
	return 0


if __name__ == "__main__":
	sys.exit(main())
