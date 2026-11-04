#!/usr/bin/env python3
"""Runs every test case of the host test suite and writes a report (passed / failed / bugged).

  run_all_tests.py --build-dir <dir> [--out <file>] [--timeout S] [--progress Tests/PROGRESS.md]

Nothing is skipped: every doctest executable found in the build directory (ffb_unit_tests, ffb_target_tests,
ffb_flash_tests, ffb_rtos_tests, ffb_usb_tests, ffb_driver_tests, ffb_dsp_tests) and the Python helper tests (Firmware/Tests/python) are run.

* The executables run with the doctest XML reporter and WITHOUT the ctest exclusions. If a run stops in a test case
  (signal, abort, sanitizer exit, or no progress for --timeout seconds), that case is recorded as crashed / timed out
  and the run continues after it (doctest --first), until every case has a result.
* Cases skipped by a decorator (skipped crash known issues, sanitizer-only skips) run one by one in their own process
  with --no-skip and a name filter that is checked to match exactly that case (fallback: --first=k --last=k).
  A crash or a timeout is a failure of that case.

Classification:
  Bugged  a known-bug case (doctest should_fail / description "KNOWN ISSUE...") that fails as expected (assertion,
          crash or timeout); a KNOWN_ISSUE_UB_HIDDEN case (passes on the host); a case excluded from ctest because of
          a suspected defect (CMake FFB_*_EXCLUDE lists) that fails; in a sanitizer build a DELETES_FFBHIDMAIN case that
          fails (bug 24); Python @expectedFailure that fails.
  Failed  any other failure, and a known bug that unexpectedly passes ("fixed? remove the marker").
  Passed  everything else. "characterization:" cases also appear under "Open questions".

The report is written to <build-dir>/test_report.log by default. Exit status 0 when Failed is 0, 1 otherwise,
2 on usage errors. Standard library only; nothing is written into the repository.
"""

import argparse
import datetime
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True  # the imports below must not write __pycache__ into the repository

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
TESTS_DIR = os.path.dirname(TOOLS_DIR)
FIRMWARE_DIR = os.path.dirname(TESTS_DIR)
REPO_DIR = os.path.dirname(FIRMWARE_DIR)
sys.path.insert(0, TOOLS_DIR)
import check_baseline  # noqa: E402  (read_cmake_cache, wildcard_to_regex, cmake_bool)
import make_bug_report  # noqa: E402  (table, sort_key: parsing of the PROGRESS.md tables)

EXECUTABLES = ("ffb_unit_tests", "ffb_target_tests", "ffb_flash_tests", "ffb_rtos_tests", "ffb_usb_tests", "ffb_driver_tests", "ffb_dsp_tests")
PYTHON_EXE = "python"
# CMake cache variables with the ctest exclusions of each executable: (always, additionally in sanitizer builds)
EXCLUDE_VARS = {
	"ffb_target_tests": ("FFB_TARGET_TESTS_EXCLUDE", "FFB_TARGET_TESTS_EXCLUDE_SANITIZE"),
	"ffb_flash_tests": ("FFB_FLASH_TESTS_EXCLUDE", "FFB_FLASH_TESTS_EXCLUDE_SANITIZE"),
	"ffb_rtos_tests": ("FFB_RTOS_TESTS_EXCLUDE", "FFB_RTOS_TESTS_EXCLUDE_SANITIZE"),
	"ffb_usb_tests": ("FFB_USB_TESTS_EXCLUDE", "FFB_USB_TESTS_EXCLUDE_SANITIZE"),
	"ffb_driver_tests": ("FFB_DRIVER_TESTS_EXCLUDE", "FFB_DRIVER_TESTS_EXCLUDE_SANITIZE"),
	"ffb_dsp_tests": ("FFB_DSP_TESTS_EXCLUDE", "FFB_DSP_TESTS_EXCLUDE_SANITIZE"),
}
KNOWN_PREFIX = "KNOWN ISSUE"
UB_HIDDEN_PREFIX = "KNOWN ISSUE (undefined behaviour, intended result on the host)"
DELETES_MAIN_MARKER = "DELETES_FFBHIDMAIN"
DELETES_MAIN_BUG = "24"  # HID_GamepadReport_base without a virtual destructor (README, AxisFixture.h)
BASELINE_KNOWN_ISSUES_NR = "NR 3"  # NEEDS REVIEW 3: the known issues of the original suite (no number of their own)
NOT_FIXED_NOTE = "known bug no longer fails - fixed? remove the marker"
SOURCE_EXTS = (".cpp", ".c", ".h", ".py")

PASSED, FAILED, BUGGED = "passed", "failed", "bugged"


# --------------------------------------------------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------------------------------------------------

def rel(path, root=REPO_DIR):
	"""Repository relative path with forward slashes (absolute paths outside the repository stay as they are)."""
	if not path:
		return path
	try:
		ap = os.path.abspath(path)
		r = os.path.relpath(ap, root)
	except ValueError:  # other drive on Windows
		return path.replace("\\", "/")
	if r.startswith(".."):
		return path.replace("\\", "/")
	return r.replace("\\", "/")


def strip_paths(text):
	"""Removes the repository root from the paths in a message, so messages contain repo relative paths."""
	if not text:
		return text
	for root in (REPO_DIR + os.sep, REPO_DIR.replace("\\", "/") + "/"):
		text = text.replace(root, "")
	return text


def one_line(text, limit=220):
	text = " ".join((text or "").split())
	return text if len(text) <= limit else text[:limit - 3] + "..."


def escape_filter(name):
	"""doctest splits -tc/-tce at commas and treats * and ? as wildcards: replace , * ? [ ] by ? (any one char)."""
	return re.sub(r"[,*?\[\]]", "?", name)


def signal_name(returncode):
	if returncode is None:
		return "killed"
	if returncode < 0:
		try:
			return signal.Signals(-returncode).name
		except (ValueError, AttributeError):
			return "signal %d" % -returncode
	windows = {0xC0000005: "access violation", 0xC0000094: "integer divide by zero", 0xC00000FD: "stack overflow",
		0xC0000409: "stack buffer overrun / abort", 0x80000003: "breakpoint"}
	if returncode in windows or (returncode & 0xFFFFFFFF) in windows:
		return windows.get(returncode, windows.get(returncode & 0xFFFFFFFF))
	if returncode == 134:
		return "SIGABRT (exit status 134)"
	return "exit status %d" % returncode


def stderr_summary(text):
	"""First line of a sanitizer / assertion / terminate message in the stderr of a crashed run, or ''."""
	if not text:
		return ""
	patterns = (
		r"ERROR: (?:AddressSanitizer|LeakSanitizer|MemorySanitizer|ThreadSanitizer): [^\n]*",
		r"[^\n]*runtime error: [^\n]*",
		r"[^\n]*Assertion [^\n]* failed[^\n]*",
		r"terminate called [^\n]*",
		r"[^\n]*(?:Segmentation fault|Floating point exception)[^\n]*",
	)
	best = None
	for pattern in patterns:
		m = re.search(pattern, text)
		if m and (best is None or m.start() < best.start()):
			best = m
	if not best:
		return ""
	line = best.group(0).strip()
	line = re.sub(r"0x[0-9a-fA-F]{6,}", "0x...", line)
	return one_line(strip_paths(line), 200)


# --------------------------------------------------------------------------------------------------------------------
# doctest XML
# --------------------------------------------------------------------------------------------------------------------

class CaseRun:
	"""The result of one test case in one doctest XML report."""

	def __init__(self, suite, attrs):
		self.suite = suite
		self.name = attrs.get("name", "")
		self.filename = attrs.get("filename", "")
		self.line = int(attrs.get("line", "0") or 0)
		self.description = attrs.get("description", "")
		self.should_fail = attrs.get("should_fail") == "true"
		self.skipped = attrs.get("skipped") == "true"
		self.finished = self.skipped  # a skipped (not run) case has no end of its own
		self.asserts_failed = 0
		self.success = None
		self.crash = None  # text of a doctest crash exception (signal caught by doctest)
		self.exception = None  # text of an unexpected exception
		self.first_failure = None  # "file:line EXPR (expanded)"

	def key(self):
		return (os.path.basename(self.filename), self.line, self.name)


def parse_doctest_xml(text):
	"""Parses a (possibly truncated) doctest XML report.

	Returns (cases, complete, totals): every TestCase in report order (skipped ones included), whether the report
	ended with </doctest>, and the attributes of OverallResultsTestCases. A truncated report ends in the case that
	was running (finished is False)."""
	parser = ET.XMLPullParser(events=("start", "end"))
	cases = []
	totals = {}
	complete = False
	suite = ""
	current = None
	stack = []
	try:
		parser.feed(text)
		parser.close()
	except ET.ParseError:
		pass  # truncated: the events read so far are still delivered below
	try:
		events = list(parser.read_events())
	except ET.ParseError:
		events = []
	for event, elem in events:
		tag = elem.tag
		if event == "start":
			parent = stack[-1] if stack else None
			stack.append(tag)
			if tag == "TestSuite":
				suite = elem.get("name", "")
			elif tag == "TestCase" and parent in ("TestSuite", "doctest"):
				current = CaseRun(suite if parent == "TestSuite" else "", elem.attrib)
				cases.append(current)
			continue
		if stack:
			stack.pop()
		parent = stack[-1] if stack else None
		if tag == "doctest":
			complete = True
		elif tag == "OverallResultsTestCases" and parent == "doctest":
			totals = dict(elem.attrib)
		elif current is None:
			continue
		elif tag == "TestCase" and parent in ("TestSuite", "doctest"):
			current.finished = True
			current = None
		elif tag == "OverallResultsAsserts" and parent == "TestCase":
			current.asserts_failed = int(elem.get("failures", "0") or 0)
			current.success = elem.get("test_case_success") == "true"
		elif tag == "Exception" and "Expression" not in stack:
			msg = one_line(strip_paths(elem.text or ""))
			if elem.get("crash") == "true":
				current.crash = msg or "crash"
			elif current.exception is None:
				current.exception = msg or "exception"
		elif tag == "Expression" and elem.get("success") == "false" and current.first_failure is None:
			original = elem.findtext("Original") or ""
			expanded = elem.findtext("Expanded")
			threw = elem.findtext("Exception")
			text = "%s:%s %s( %s )" % (rel(elem.get("filename", "")), elem.get("line", "?"), elem.get("type", ""),
				one_line(original, 120))
			if threw:
				text += " threw: " + one_line(threw, 80)
			elif expanded and one_line(expanded) != one_line(original):
				text += " is " + one_line(expanded, 80)
			current.first_failure = one_line(strip_paths(text), 260)
		elif tag == "Message" and current.first_failure is None and elem.get("type", "") in ("ERROR", "FATAL ERROR"):
			current.first_failure = one_line(strip_paths("%s:%s %s: %s" % (
				rel(elem.get("filename", "")), elem.get("line", "?"), elem.get("type"), elem.findtext("Text") or "")), 260)
	return cases, complete, totals


def parse_inventory(text):
	"""--list-test-cases --no-skip -r=xml: [(suite, name, filename, line, skip_decorated)] in run order."""
	root = ET.fromstring(text)
	out = []
	for tc in root.iter("TestCase"):
		out.append((tc.get("testsuite", ""), tc.get("name", ""), tc.get("filename", ""), int(tc.get("line", "0") or 0),
			tc.get("skipped") == "true"))
	return out


# --------------------------------------------------------------------------------------------------------------------
# Results and classification
# --------------------------------------------------------------------------------------------------------------------

class Result:
	"""One test case of the report."""

	def __init__(self, exe, suite, name, location, description="", should_fail=False):
		self.exe = exe
		self.suite = suite
		self.name = name
		self.location = location  # repo relative "file:line"
		self.description = description
		self.should_fail = should_fail
		self.outcome = None  # passed | failed | crash | timeout | expected_failure | unexpected_success | skipped
		self.reason = ""
		self.isolated = False  # ran in its own process (skip-decorated case)
		self.skip_decorated = False
		self.excluded = False  # in the ctest exclusion list of this build
		self.deletes_main = False  # DELETES_FFBHIDMAIN marker
		self.python_expected_failure = False
		self.category = None
		self.note = ""
		self.numbers = []  # e.g. ["bug 16"], ["V29"], ["NR 39"]
		self.number_source = ""
		self.files_under_test = []
		self.attention = False  # passed, but listed under "Passed with a note"

	@property
	def known_bug(self):
		return self.should_fail or self.description.startswith(KNOWN_PREFIX) or self.python_expected_failure

	@property
	def ub_hidden(self):
		return self.description.startswith(UB_HIDDEN_PREFIX)

	@property
	def characterization(self):
		return self.name.lower().startswith("characterization:")

	def sort_key(self):
		file, _, line = self.location.rpartition(":")
		return (exe_order(self.exe), file, int(line) if line.isdigit() else 0, self.name)


def exe_order(exe):
	return EXECUTABLES.index(exe) if exe in EXECUTABLES else len(EXECUTABLES)


def result_from_run(exe, run):
	r = Result(exe, run.suite, run.name, "%s:%d" % (rel(run.filename), run.line), run.description, run.should_fail)
	if run.crash is not None:
		r.outcome = "crash"
		r.reason = "crash: " + run.crash
	elif run.exception is not None:
		r.outcome = "failed"
		r.reason = "exception: " + run.exception
	elif run.asserts_failed > 0 or (run.success is False and not run.should_fail):
		r.outcome = "failed"
		r.reason = run.first_failure or ("%d failed assertion(s)" % run.asserts_failed)
	elif run.should_fail and run.success is False:
		r.outcome = "passed"  # doctest: "should have failed but didn't"
	else:
		r.outcome = "passed"
	return r


def classify(r, sanitize=False):
	"""Sets r.category (passed / failed / bugged) and r.note. See the module docstring for the rules."""
	failed = r.outcome in ("failed", "crash", "timeout", "expected_failure")
	if r.outcome == "skipped":
		r.category, r.note = FAILED, "skipped (nothing may be skipped)"
	elif r.outcome == "unexpected_success":
		r.category, r.note = FAILED, NOT_FIXED_NOTE
	elif r.ub_hidden:
		r.category = BUGGED
		if failed:
			r.note = "undefined behaviour, reported in this build"
		elif sanitize:
			r.note = "UB, passes on the host (the sanitizer did not report it in this run)"
		else:
			r.note = "UB, passes on the host"
	elif r.known_bug and not failed and r.skip_decorated and "undefined behaviour" in r.description:
		# only run because of --no-skip: a pass shows the UB had no visible effect in this run, not a fix (the normal
		# builds, where KNOWN_ISSUE_UB is should_fail and not skipped, still report a fix as "no longer fails")
		r.category = BUGGED
		r.note = "undefined behaviour, passes in this run (skipped in this build; not evidence of a fix)"
	elif r.known_bug:
		if failed:
			r.category = BUGGED
			r.note = {"crash": "known bug, crashes as expected", "timeout": "known bug, hangs (timeout)"}.get(r.outcome, "")
		else:
			r.category, r.note = FAILED, NOT_FIXED_NOTE
	elif r.excluded:
		if failed:
			r.category, r.note = BUGGED, "excluded from ctest in this build because of a suspected defect"
		else:
			r.category, r.note = PASSED, "in the ctest exclusion list of this build but passes - is the exclusion still needed?"
			r.attention = True
	elif r.deletes_main and sanitize and failed:
		r.category = BUGGED
		r.note = "deletes an FFBHIDMain: undefined behaviour reported by the sanitizer (bug %s); passes in normal builds" % DELETES_MAIN_BUG
		if "bug " + DELETES_MAIN_BUG not in r.numbers:
			r.numbers = ["bug " + DELETES_MAIN_BUG] + r.numbers
			r.number_source = "DELETES_FFBHIDMAIN marker"
	elif failed:
		r.category = FAILED
	else:
		r.category = PASSED
	if r.isolated and not r.note.startswith("skipped"):
		r.note = (r.note + "; " if r.note else "") + "ran isolated with --no-skip"
	return r.category


# --------------------------------------------------------------------------------------------------------------------
# Bug / NEEDS REVIEW numbers and the PROGRESS.md tables
# --------------------------------------------------------------------------------------------------------------------

NUMBER_PATTERNS = (
	(re.compile(r"\bbug\s*#?\s*(\d+)\b", re.IGNORECASE), "bug {}"),
	(re.compile(r"(?<![A-Za-z0-9_])V(\d+)\b"), "V{}"),
	(re.compile(r"\bNEEDS[ _]REVIEW\s*#?\s*(\d+)\b", re.IGNORECASE), "NR {}"),
	(re.compile(r"\bNR\s*#?\s*(\d+)\b"), "NR {}"),
)


def extract_numbers(text):
	"""Bug / vendor / NEEDS REVIEW numbers in a marker text, in order of appearance (no duplicates).

	"(bug 16)", "bug 116:", "Bug 81" -> "bug N"; "V29", "(V29)" -> "VN"; "NEEDS REVIEW 29", "NEEDS REVIEW #39",
	"NR 120" -> "NR N". "bugs 37 and 56" (a reference to other bugs) is not matched."""
	found = []
	for pattern, fmt in NUMBER_PATTERNS:
		for m in pattern.finditer(text or ""):
			found.append((m.start(), fmt.format(m.group(1))))
	out = []
	for _, num in sorted(found):
		if num not in out:
			out.append(num)
	return out


def table_key(num):
	"""'bug 16' -> ('bug', '16'), 'V29' -> ('bug', 'V29'), 'NR 3' -> ('nr', '3')."""
	if num.startswith("NR "):
		return ("nr", num[3:])
	if num.startswith("bug "):
		return ("bug", num[4:])
	return ("bug", num)


def number_sort_key(num):
	kind, n = table_key(num)
	return (1 if kind == "nr" else 0,) + tuple(make_bug_report.sort_key(n))


class Progress:
	"""The "Suspected firmware bugs" and "NEEDS REVIEW items" tables of Tests/PROGRESS.md."""

	def __init__(self, text=""):
		self.rows = {}  # ('bug', '16') / ('nr', '39') -> [num, module, location, description, evidence]
		for heading, kind in (("## Suspected firmware bugs", "bug"), ("## NEEDS REVIEW items", "nr")):
			if heading not in text:
				continue
			for row in make_bug_report.table(text, heading):
				row = (row + [""] * 5)[:5]
				self.rows[(kind, row[0])] = row

	@classmethod
	def load(cls, path):
		try:
			with open(path, encoding="utf-8") as f:
				return cls(f.read())
		except OSError:
			return cls("")

	def row(self, num):
		return self.rows.get(table_key(num))

	def description(self, num):
		row = self.row(num)
		if not row:
			return ""
		text = row[3].replace("**", "").replace("`", "")
		return one_line(text, 200)

	def location(self, num):
		"""The Location cell; if it names no source file, the Description and Evidence cells."""
		row = self.row(num)
		if not row:
			return ""
		for cell in (row[2], row[3], row[4]):
			if PATH_IN_TEXT.search(cell):
				return cell
		return row[2]

	def find_by_test_name(self, name, kinds=("bug", "nr")):
		"""Rows whose evidence / description / location names the test (exact substring, at least 12 characters)."""
		if len(name) < 12:
			return []
		hits = []
		for (kind, n), row in self.rows.items():
			if kind in kinds and any(name in cell for cell in row[2:]):
				hits.append("NR " + n if kind == "nr" else (n if n.startswith("V") else "bug " + n))
		return sorted(hits, key=number_sort_key)


def source_lines(path, cache={}):
	if path not in cache:
		try:
			with open(path, encoding="utf-8", errors="replace") as f:
				cache[path] = f.read().splitlines()
		except OSError:
			cache[path] = []
	return cache[path]


def case_header_text(path, line):
	"""The source text that belongs to a TEST_CASE at path:line: the comment block right above it, the macro with
	its decorators up to the opening brace, and the comments at the start of the body."""
	lines = source_lines(path)
	if not lines or line < 1 or line > len(lines):
		return ""
	idx = line - 1
	above = []
	i = idx - 1
	while i >= 0:
		s = lines[i].strip()
		if s.startswith("//") or s.startswith("/*") or s.startswith("*") or s.endswith("*/"):
			above.insert(0, lines[i])
			i -= 1
		else:
			break
	header = []
	i = idx
	while i < len(lines) and i < idx + 25:
		header.append(lines[i])
		if lines[i].rstrip().endswith("{"):
			i += 1
			break
		i += 1
	while i < len(lines) and lines[i].strip().startswith("//"):
		header.append(lines[i])
		i += 1
	return "\n".join(above + header)


def strip_comments(text):
	return "\n".join(l.split("//", 1)[0] for l in text.splitlines())


# --------------------------------------------------------------------------------------------------------------------
# File under test
# --------------------------------------------------------------------------------------------------------------------

def normalize(name):
	return re.sub(r"[^a-z0-9]", "", name.lower())


class SourceIndex:
	"""All .c/.cpp/.h/.py files under Firmware/ (without Firmware/Tests except tools/, and without build folders)."""

	def __init__(self, firmware_dir=FIRMWARE_DIR, files=None):
		self.files = []  # repo relative paths
		if files is not None:
			self.files = list(files)
		else:
			tests = os.path.join(firmware_dir, "Tests")
			for dirpath, dirnames, filenames in os.walk(firmware_dir):
				dirnames[:] = [d for d in dirnames if not d.startswith(".") and d != "__pycache__"]
				if os.path.abspath(dirpath) == os.path.abspath(tests):
					dirnames[:] = [d for d in dirnames if d == "tools"]
				for fn in filenames:
					if fn.endswith(SOURCE_EXTS):
						self.files.append(rel(os.path.join(dirpath, fn)))
		self.by_base = {}
		self.by_stem = {}
		for f in self.files:
			base = f.rsplit("/", 1)[-1]
			self.by_base.setdefault(base, []).append(f)
			if "/Libraries/" not in "/" + f and "/Targets/" not in "/" + f:
				self.by_stem.setdefault(normalize(os.path.splitext(base)[0]), []).append(f)

	@staticmethod
	def _vendor_or_target(path):
		return "/Libraries/" in "/" + path or "/Targets/" in "/" + path or "/Tests/" in "/" + path

	def resolve(self, fragment):
		"""A path as written in a table ('FFBoard/Src/X.cpp', 'hid_device.c', 'Firmware/scripts/x.py') -> the repo
		relative path if exactly one file matches (outside Libraries/Targets preferred), else None."""
		fragment = fragment.replace("\\", "/").lstrip("./")
		if fragment.startswith("Firmware/"):
			fragment = fragment[len("Firmware/"):]
		base = fragment.rsplit("/", 1)[-1]
		cands = [f for f in self.by_base.get(base, []) if ("/" + f).endswith("/" + fragment)]
		if len(cands) > 1:
			own = [f for f in cands if not self._vendor_or_target(f)]
			cands = own if len(own) == 1 else cands
		return cands[0] if len(cands) == 1 else None

	def sources_for_stem(self, stem):
		"""Firmware sources whose file name (normalized) equals stem: .cpp/.c first, the header if there is none."""
		cands = self.by_stem.get(normalize(stem), [])
		impl = [f for f in cands if f.endswith((".cpp", ".c", ".py"))]
		return sorted(impl or cands)


PATH_IN_TEXT = re.compile(r"[A-Za-z0-9_./-]+\.(?:cpp|c|h|py)\b")


def files_from_location(text, index, limit=2, resolved_only=False):
	"""The first *.c/*.cpp/*.h/*.py paths of a table Location cell (or a marker text), resolved to repo relative
	paths where unambiguous. Unresolved names are kept as written (marked when they have no folder), unless
	resolved_only."""
	out = []
	for m in PATH_IN_TEXT.finditer(text or ""):
		frag = m.group(0)
		resolved = index.resolve(frag)
		if resolved is None and resolved_only:
			continue
		item = resolved or frag
		if resolved is None and not frag.count("/"):
			item = frag + " (ambiguous or not found)"
		if item not in out:
			out.append(item)
		if len(out) >= limit:
			break
	return out


def test_file_stem(test_path):
	stem = os.path.splitext(os.path.basename(test_path))[0]
	stem = re.sub(r"^test_", "", stem)
	stem = re.sub(r"_(gaps|cases)$", "", stem)
	return stem


def quoted_includes(test_path):
	return re.findall(r'^\s*#\s*include\s+"([^"]+)"', "\n".join(source_lines(test_path)), re.MULTILINE)


def files_from_test(test_path, suite, index):
	"""Heuristic for cases without a numbered row (documented in the README):
	1. the suite name equals a firmware source name (CmdParser -> FFBoard/Src/CmdParser.cpp);
	2. the test file name without test_ / _gaps / _cases equals a source name (test_hid_ffb.cpp -> HidFFB.cpp,
	   python/test_generate_memory_cases.py -> generate_memory.py);
	3. the longest source name (4+ characters) the stem starts with (test_axis_config.cpp -> Axis.cpp,
	   test_usbd_core.cpp -> usbd.c); not for *_setup.cpp files (they test the test setup);
	4. a firmware header the test file includes whose name contains that stem or is contained in it;
	otherwise unknown. Sources under Libraries/ and Targets/ are not candidates (vendor code is named by the
	numbered PROGRESS.md row instead)."""
	for candidate in (suite.split(".")[-1] if suite else "", test_file_stem(test_path)):
		if candidate:
			found = index.sources_for_stem(candidate)
			if found:
				return found[:3]
	key = normalize(test_file_stem(test_path))
	if not test_file_stem(test_path).endswith("_setup"):
		prefixes = [s for s in index.by_stem if len(s) >= 4 and key.startswith(s)]
		if prefixes:
			return index.sources_for_stem(max(prefixes, key=len))[:3]
	hits = []
	for inc in quoted_includes(test_path):
		inc_stem = normalize(os.path.splitext(os.path.basename(inc))[0])
		if len(inc_stem) < 3 or len(key) < 3 or not (inc_stem in key or key in inc_stem):
			continue
		for f in index.sources_for_stem(inc_stem):
			if f not in hits:
				hits.append(f)
	return hits[:3]


def annotate_numbers(r, progress, baseline_known=frozenset()):
	"""Sets the bug / NEEDS REVIEW number of a result (and the DELETES_FFBHIDMAIN flag). Order: the marker text
	(description), the test name, the comments at the TEST_CASE, a PROGRESS.md row that names the test, the
	baseline known issues (NR 3)."""
	test_path = os.path.join(REPO_DIR, r.location.rpartition(":")[0])
	line = int(r.location.rpartition(":")[2] or 0)
	header = case_header_text(test_path, line) if r.exe != PYTHON_EXE else ""
	if r.exe != PYTHON_EXE and DELETES_MAIN_MARKER in strip_comments(header):
		r.deletes_main = True
	if not r.numbers:
		for source, text in (("marker", r.description), ("name", r.name), ("source comment", header)):
			nums = extract_numbers(text)
			if r.characterization and source != "marker":
				nums = [n for n in nums if n.startswith("NR ")] or nums
			if nums:
				r.numbers, r.number_source = nums[:1], source
				break
	if not r.numbers:
		key = r.name if r.exe != PYTHON_EXE else "%s.%s" % (r.suite.split(".")[-1], r.name)
		kinds = ("nr",) if r.characterization else ("bug", "nr")
		hits = progress.find_by_test_name(key, kinds)
		if not hits and r.characterization:
			hits = progress.find_by_test_name(re.sub(r"^characterization:\s*", "", r.name), kinds)
		if hits:
			r.numbers, r.number_source = hits[:1], "PROGRESS.md row naming the test"
	if not r.numbers and r.exe == "ffb_unit_tests" and (r.suite, r.name) in baseline_known:
		r.numbers, r.number_source = [BASELINE_KNOWN_ISSUES_NR], "known issue of the original suite"


def annotate_files(r, progress, index):
	"""Sets the files under test: the Location of the numbered PROGRESS.md row, else the test file heuristic."""
	test_path = os.path.join(REPO_DIR, r.location.rpartition(":")[0])
	files = []
	for num in r.numbers:
		files = [f for f in files_from_location(progress.location(num), index) if "(ambiguous" not in f] or files
		if files:
			break
	if not files and r.known_bug:
		files = files_from_location(r.description, index, resolved_only=True)
	if not files:
		files = files_from_test(test_path, r.suite, index)
	r.files_under_test = files


def read_baseline_known(path=os.path.join(TESTS_DIR, "baseline", "test_cases.txt")):
	known = set()
	try:
		with open(path, encoding="utf-8") as f:
			for line in f:
				parts = line.rstrip("\r\n").split("\t")
				if len(parts) == 3 and parts[0].startswith("known_issue"):
					known.add((parts[1], parts[2]))
	except OSError:
		pass
	return frozenset(known)


# --------------------------------------------------------------------------------------------------------------------
# Running the executables
# --------------------------------------------------------------------------------------------------------------------

def run_process(cmd, workdir, tag, timeout, progress_file=None, cwd=None, env=None):
	"""Runs cmd with stdout/stderr to files. Without progress_file the timeout is for the whole process; with it the
	process is stopped when progress_file has not grown for timeout seconds. Returns (returncode, timed_out, stderr)."""
	out_path = os.path.join(workdir, tag + ".stdout")
	err_path = os.path.join(workdir, tag + ".stderr")
	with open(out_path, "wb") as out, open(err_path, "wb") as err:
		proc = subprocess.Popen(cmd, stdout=out, stderr=err, stdin=subprocess.DEVNULL, cwd=cwd or workdir, env=env)
		last_size, last_change = -1, time.monotonic()
		timed_out = False
		while True:
			try:
				proc.wait(timeout=0.05)
				break
			except subprocess.TimeoutExpired:
				pass
			now = time.monotonic()
			if progress_file:
				try:
					size = os.path.getsize(progress_file)
				except OSError:
					size = 0
				if size != last_size:
					last_size, last_change = size, now
			if now - last_change > timeout:
				timed_out = True
				proc.kill()
				proc.wait()
				break
	with open(err_path, "rb") as f:
		stderr = f.read().decode("utf-8", errors="replace")
	return proc.returncode, timed_out, stderr


def read_text(path):
	try:
		with open(path, "rb") as f:
			return f.read().decode("utf-8", errors="replace")
	except OSError:
		return ""


class ExeRunner:
	def __init__(self, exe_name, exe_path, workdir, timeout, log=print):
		self.name = exe_name
		self.path = exe_path
		self.workdir = workdir
		self.timeout = timeout
		self.log = log
		self.counter = 0
		self.problems = []  # run level problems (not attributable to a case)

	def _tag(self, what):
		self.counter += 1
		return "%s-%s-%d" % (self.name, what, self.counter)

	def doctest(self, args, what, whole_process_timeout=False):
		tag = self._tag(what)
		xml_path = os.path.join(self.workdir, tag + ".xml")
		cmd = [self.path, "--reporters=xml", "--no-version", "--out=" + xml_path] + args
		rc, timed_out, stderr = run_process(cmd, self.workdir, tag, self.timeout,
			progress_file=None if whole_process_timeout else xml_path)
		return rc, timed_out, stderr, read_text(xml_path)

	def inventory(self, extra=()):
		rc, timed_out, stderr, text = self.doctest(["--list-test-cases", "--no-skip"] + list(extra), "list", True)
		try:
			return parse_inventory(text)
		except ET.ParseError:
			raise RuntimeError("%s --list-test-cases failed (%s): %s" % (self.name, signal_name(rc), one_line(stderr)))

	def run(self):
		"""Returns the results of every case of the executable."""
		inventory = self.inventory()
		ident = lambda suite, name, fn, line: (os.path.basename(fn), line, name)
		order = [ident(*c[:4]) for c in inventory]
		unskipped = [ident(*c[:4]) for c in inventory if not c[4]]
		results = {}
		first = 1
		attempts = 0
		while first <= len(unskipped) and attempts < len(unskipped) + 5:
			attempts += 1
			rc, timed_out, stderr, text = self.doctest(["--first=%d" % first], "run")
			cases, complete, totals = parse_doctest_xml(text)
			started = [c for c in cases if not c.skipped]
			for c in started:
				if c.finished and c.crash is None:
					results[c.key()] = result_from_run(self.name, c)
			stopped = [c for c in started if not c.finished or c.crash is not None]
			if not stopped and not timed_out and complete:
				if rc != 0 and int(totals.get("failures", "0") or 0) == 0:
					self.problems.append("%s exited with %s after all test cases finished: %s" % (
						self.name, signal_name(rc), stderr_summary(stderr) or one_line(stderr[-300:])))
				break
			if not stopped:
				# stopped outside a test case (e.g. at start-up): nothing to attribute, give up on the rest
				self.problems.append("%s stopped outside a test case (%s): %s" % (
					self.name, "timeout" if timed_out else signal_name(rc), stderr_summary(stderr) or one_line(stderr[-300:])))
				break
			c = stopped[-1]
			r = result_from_run(self.name, c)
			if timed_out:
				r.outcome, r.reason = "timeout", "timeout: no progress for %g s (process killed)" % self.timeout
			else:
				detail = stderr_summary(stderr)
				what = c.crash or signal_name(rc)
				r.outcome = "crash"
				r.reason = "crash: %s%s" % (what, " - " + detail if detail and detail not in what else "")
			results[c.key()] = r
			try:
				first = unskipped.index(c.key()) + 2
			except ValueError:
				self.problems.append("%s: the crashed case %r is not in the case list" % (self.name, c.name))
				break
		# skip-decorated cases: one process each, --no-skip, exact filter
		for index, (suite, name, fn, line, skipped) in enumerate(inventory):
			key = ident(suite, name, fn, line)
			if skipped:
				r = self.run_isolated(suite, name, fn, line, index + 1)
				r.skip_decorated = True
				results[key] = r
		ordered = []
		for (suite, name, fn, line, skipped) in inventory:
			key = ident(suite, name, fn, line)
			r = results.get(key)
			if r is None:
				r = Result(self.name, suite, name, "%s:%d" % (rel(fn), line))
				r.outcome, r.reason = "failed", "no result: the case was not reached"
			r.suite = r.suite or suite
			ordered.append(r)
		if len(ordered) != len(order):
			self.problems.append("%s: %d cases listed, %d results" % (self.name, len(order), len(ordered)))
		return ordered

	def run_isolated(self, suite, name, fn, line, position):
		"""Runs one skip-decorated case with --no-skip. The name filter (+ source file filter) must list exactly this
		case; otherwise the case is selected by its position (--first=k --last=k)."""
		filters = ["--test-case=" + escape_filter(name), "--source-file=*" + os.path.basename(fn)]
		listed = self.inventory(filters)
		target = (os.path.basename(fn), line, name)
		if not (len(listed) == 1 and (os.path.basename(listed[0][2]), listed[0][3], listed[0][1]) == target):
			filters = ["--first=%d" % position, "--last=%d" % position]
			listed = self.inventory(filters)
			if not (len(listed) == 1 and (os.path.basename(listed[0][2]), listed[0][3], listed[0][1]) == target):
				r = Result(self.name, suite, name, "%s:%d" % (rel(fn), line))
				r.outcome, r.reason = "failed", "no result: could not select exactly this case"
				r.isolated = True
				return r
		rc, timed_out, stderr, text = self.doctest(["--no-skip"] + filters, "isolated")
		cases, complete, totals = parse_doctest_xml(text)
		started = [c for c in cases if not c.skipped]
		if started:
			c = started[0]
			r = result_from_run(self.name, c)
			if timed_out:
				r.outcome, r.reason = "timeout", "timeout: no progress for %g s (process killed)" % self.timeout
			elif not c.finished or c.crash is not None or (rc != 0 and not complete):
				detail = stderr_summary(stderr)
				what = c.crash or signal_name(rc)
				r.outcome = "crash"
				r.reason = "crash: %s%s" % (what, " - " + detail if detail and detail not in what else "")
			elif rc != 0 and r.outcome == "passed" and not c.should_fail:
				r.outcome = "crash"
				r.reason = "exit: %s - %s" % (signal_name(rc), stderr_summary(stderr) or one_line(stderr[-200:]))
		else:
			r = Result(self.name, suite, name, "%s:%d" % (rel(fn), line))
			if timed_out:
				r.outcome, r.reason = "timeout", "timeout before the case started"
			else:
				r.outcome, r.reason = "crash", "no result: %s - %s" % (signal_name(rc), stderr_summary(stderr))
		r.isolated = True
		return r


# --------------------------------------------------------------------------------------------------------------------
# Python helper tests
# --------------------------------------------------------------------------------------------------------------------

PY_DRIVER = r'''
import inspect, json, os, sys, traceback, unittest
sys.dont_write_bytecode = True
start, out_path = sys.argv[1], sys.argv[2]
records = []

def location(test):
	try:
		fn = getattr(test, test._testMethodName)
		fn = inspect.unwrap(getattr(fn, "__func__", fn))
		return inspect.getsourcefile(fn), inspect.getsourcelines(fn)[1]
	except Exception:
		mod = sys.modules.get(type(test).__module__)
		return (getattr(mod, "__file__", "") or "", 0)

def where(err, test_file):
	tb = traceback.extract_tb(err[2])
	frames = [f for f in tb if os.path.abspath(f.filename) == os.path.abspath(test_file or "")] or list(tb)
	f = frames[-1] if frames else None
	last = traceback.format_exception_only(err[0], err[1])[-1].strip()
	return ("%s:%d %s" % (f.filename, f.lineno, last)) if f else last

class Result(unittest.TestResult):
	def _rec(self, test, outcome, detail=""):
		if test.__class__.__name__ in ("_FailedTest", "_ErrorHolder", "ModuleImportFailure"):
			fn, line, suite, name = "", 0, test.id().rsplit(".", 1)[0], test.id().rsplit(".", 1)[-1]
		else:
			fn, line = location(test)
			suite, name = "%s.%s" % (type(test).__module__, type(test).__name__), test._testMethodName
		records.append({"suite": suite, "name": name, "file": fn, "line": line, "outcome": outcome, "detail": detail})
	def addSuccess(self, test): self._rec(test, "passed")
	def addFailure(self, test, err): self._rec(test, "failed", where(err, location(test)[0]))
	def addError(self, test, err):
		if hasattr(test, "_testMethodName"):
			self._rec(test, "failed", "error: " + where(err, location(test)[0]))
		else:
			self._rec(test, "failed", "error: " + traceback.format_exception_only(err[0], err[1])[-1].strip())
	def addSkip(self, test, reason): self._rec(test, "skipped", reason)
	def addExpectedFailure(self, test, err): self._rec(test, "expected_failure", where(err, location(test)[0]))
	def addUnexpectedSuccess(self, test): self._rec(test, "unexpected_success")
	def addSubTest(self, test, subtest, err):
		if err is not None:
			self._rec(test, "failed", "subtest %s: %s" % (subtest._subDescription(), where(err, location(test)[0])))

suite = unittest.TestLoader().discover(start, pattern="test_*.py", top_level_dir=start)
suite.run(Result())
with open(out_path, "w", encoding="utf-8") as f:
	json.dump(records, f)
'''


def python_results(python_dir, workdir, timeout, python=sys.executable):
	"""Runs the Python helper tests (unittest discover) in a subprocess; returns (results, problems)."""
	driver = os.path.join(workdir, "py_driver.py")
	out_json = os.path.join(workdir, "py_results.json")
	with open(driver, "w", encoding="utf-8") as f:
		f.write(PY_DRIVER)
	env = dict(os.environ)
	env["PYTHONDONTWRITEBYTECODE"] = "1"
	rc, timed_out, stderr = run_process([python, "-B", driver, python_dir, out_json], workdir, "python",
		max(timeout, 300), cwd=workdir, env=env)
	problems = []
	try:
		with open(out_json, encoding="utf-8") as f:
			records = json.load(f)
	except (OSError, ValueError):
		return [], ["python tests did not finish (%s): %s" % ("timeout" if timed_out else signal_name(rc),
			one_line(stderr[-400:]))]
	return records_to_results(records), problems


def records_to_results(records):
	merged = {}
	order = []
	for rec in records:
		key = (rec["suite"], rec["name"])
		r = merged.get(key)
		if r is None:
			loc = "%s:%d" % (rel(rec["file"]) if rec["file"] else rec["suite"], rec["line"] or 0)
			r = Result(PYTHON_EXE, rec["suite"], rec["name"], loc)
			merged[key] = r
			order.append(key)
		outcome = rec["outcome"]
		if outcome == "expected_failure":
			r.python_expected_failure = True
		if outcome == "unexpected_success":
			r.python_expected_failure = True
		# a failed subtest followed by the overall success keeps the failure
		if r.outcome in (None, "passed") or outcome != "passed":
			if not (r.outcome == "failed" and outcome == "passed"):
				r.outcome = outcome
				r.reason = one_line(strip_paths(rec.get("detail", "")), 260)
	for key in order:
		r = merged[key]
		if r.outcome == "skipped":
			r.reason = "skipped: " + r.reason
	return [merged[k] for k in order]


# --------------------------------------------------------------------------------------------------------------------
# Build configuration and report
# --------------------------------------------------------------------------------------------------------------------

def find_exe(build_dir, name):
	for sub in ("", "Debug", "Release", "RelWithDebInfo", "MinSizeRel"):
		for fn in (name, name + ".exe"):
			path = os.path.join(build_dir, sub, fn)
			if os.path.isfile(path):
				return path
	return None


def cmake_list(value):
	return [v for v in (value or "").split(";") if v.strip()]


def build_config(build_dir):
	cache = check_baseline.read_cmake_cache(build_dir)
	sanitize = check_baseline.cmake_bool(cache.get("FFB_TESTS_SANITIZE", "OFF"))
	dsp = None
	for flags in (os.path.join(build_dir, "CMakeFiles", "ffb_unit_tests.dir", "flags.make"),
			os.path.join(build_dir, "build.ninja")):
		text = read_text(flags)
		if text:
			dsp = "-DUSE_DSP_FUNCTIONS" in text
			break
	if dsp is None:
		setting = cache.get("FFB_TESTS_USE_DSP", "AUTO").upper()
		if setting == "AUTO":
			dsp = os.path.isfile(os.path.join(FIRMWARE_DIR, "Libraries", "CMSIS-DSP", "Include", "arm_math_types.h"))
		else:
			dsp = check_baseline.cmake_bool(setting)
	compiler = cache.get("CMAKE_CXX_COMPILER", "?")
	cmake_files = os.path.join(build_dir, "CMakeFiles")
	try:
		for d in sorted(os.listdir(cmake_files), reverse=True):
			text = read_text(os.path.join(cmake_files, d, "CMakeCXXCompiler.cmake"))
			mid = re.search(r'set\(CMAKE_CXX_COMPILER_ID "([^"]*)"\)', text)
			mver = re.search(r'set\(CMAKE_CXX_COMPILER_VERSION "([^"]*)"\)', text)
			if mid:
				compiler = "%s %s (%s)" % (mid.group(1), mver.group(1) if mver else "", compiler)
				break
	except OSError:
		pass
	excludes = {"ffb_unit_tests": [e for e in cache.get("FFB_TESTS_EXCLUDED_ARG", "").split(",") if e.strip()]}
	for exe, (always, san) in EXCLUDE_VARS.items():
		excludes[exe] = cmake_list(cache.get(always, "")) + (cmake_list(cache.get(san, "")) if sanitize else [])
	return {
		"sanitize": sanitize, "dsp": dsp, "compiler": compiler,
		"build_type": cache.get("CMAKE_BUILD_TYPE", "") or "(none)",
		"generator": cache.get("CMAKE_GENERATOR", "?"), "excludes": excludes,
	}


def git_commit():
	try:
		head = subprocess.run(["git", "-C", REPO_DIR, "log", "-1", "--format=%h %s"], capture_output=True, text=True,
			check=True).stdout.strip()
		dirty = subprocess.run(["git", "-C", REPO_DIR, "status", "--porcelain", "--untracked-files=no"],
			capture_output=True, text=True, check=True).stdout.strip()
		return head + (" (+ uncommitted changes)" if dirty else "")
	except (OSError, subprocess.CalledProcessError):
		return "unknown"


def counts(results):
	c = {PASSED: 0, FAILED: 0, BUGGED: 0, "open": 0, "total": len(results)}
	for r in results:
		c[r.category] += 1
		if r.category == PASSED and r.characterization:
			c["open"] += 1
	return c


def entry_lines(r, progress, label=None):
	lines = []
	head = "%s / %s" % (r.exe, r.suite) if r.suite else r.exe
	lines.append("%s%s: \"%s\"" % ("[%s] " % label if label else "", head, r.name))
	lines.append("    test:            %s" % r.location)
	lines.append("    file under test: %s" % (", ".join(r.files_under_test) if r.files_under_test else "unknown"))
	if r.numbers:
		num = r.numbers[0]
		desc = progress.description(num)
		lines.append("    item:            %s%s" % (num, " - " + desc if desc else " (not in the PROGRESS.md tables)"))
		if len(r.numbers) > 1:
			lines.append("    also:            %s" % ", ".join(r.numbers[1:]))
	else:
		lines.append("    item:            none")
	if r.reason or r.outcome == "passed":
		lines.append("    result:          %s" % (r.reason if r.reason else "passed"))
	if r.note:
		lines.append("    note:            %s" % r.note)
	return lines


def attention_banner(attention):
	"""Highlighted block for cases that are excluded from ctest in this build but pass here (e.g. the second spline
	case of bug 2 in sanitizer builds, kept excluded on purpose). Empty when there are none."""
	if not attention:
		return []
	bar = "!" * 110
	lines = [bar, "!!! ATTENTION: %d test case(s) excluded from ctest in this build pass here (kept excluded on purpose;"
	         % len(attention), "!!! see \"Passed with a note - ATTENTION\" below):"]
	for r in attention:
		lines.append("!!!   %s / %s: \"%s\" (%s)" % (r.exe, r.suite, r.name, r.location))
	lines.extend([bar, ""])
	return lines


def render_report(results, problems, config, build_dir, exes_run, progress, started, duration):
	c = counts(results)
	out = []
	w = out.append
	w("OpenFFBoard host test suite - full test report")
	w("=" * 46)
	w("")
	w("Date:        %s" % started.strftime("%Y-%m-%d %H:%M:%S"))
	w("Commit:      %s" % git_commit())
	w("Build dir:   %s" % os.path.abspath(build_dir))
	w("Config:      CMSIS-DSP %s, sanitizers %s, compiler %s, build type %s" % (
		"on" if config["dsp"] else "off", "on" if config["sanitize"] else "off", config["compiler"], config["build_type"]))
	w("Run:         every test case, nothing skipped (skip-decorated cases ran isolated with --no-skip); %.1f s" % duration)
	w("")
	w("Summary")
	w("-------")
	w("Passed: %d" % c[PASSED])
	w("Failed: %d" % c[FAILED])
	w("Bugged: %d" % c[BUGGED])
	w("Total:  %d" % c["total"])
	w("Open questions (included in Passed): %d" % c["open"])
	if problems:
		w("Run problems: %d (see the end of the report)" % len(problems))
	w("")
	w("%-18s %7s %7s %7s %7s %10s" % ("Executable", "Passed", "Failed", "Bugged", "Total", "Open q."))
	for exe in exes_run:
		ce = counts([r for r in results if r.exe == exe])
		w("%-18s %7d %7d %7d %7d %10d" % (exe, ce[PASSED], ce[FAILED], ce[BUGGED], ce["total"], ce["open"]))
	w("%-18s %7d %7d %7d %7d %10d" % ("all", c[PASSED], c[FAILED], c[BUGGED], c["total"], c["open"]))
	w("")
	attention = sorted([r for r in results if r.category == PASSED and r.attention], key=Result.sort_key)
	out.extend(attention_banner(attention))
	w("Categories: Bugged = a known bug (KNOWN_ISSUE, KNOWN_ISSUE_UB(_HIDDEN), skipped crash case, ctest exclusion,")
	w("DELETES_FFBHIDMAIN in sanitizer builds, Python @expectedFailure) that fails as expected. Failed = any other")
	w("failure, or a known bug that passes now. Items: bug N / VN (vendor) / NR N (NEEDS REVIEW) of Tests/PROGRESS.md.")
	w("")
	w("")
	failed = sorted([r for r in results if r.category == FAILED], key=Result.sort_key)
	w("Failed tests (%d)" % len(failed))
	w("-" * 40)
	if not failed:
		w("none")
	for r in failed:
		out.extend(entry_lines(r, progress, "FAILED"))
		w("")
	w("")
	bugged = [r for r in results if r.category == BUGGED]
	w("Bugged tests (%d; known bugs, grouped by file under test, then bug number)" % len(bugged))
	w("-" * 40)
	if not bugged:
		w("none")
	groups = {}
	for r in bugged:
		groups.setdefault(r.files_under_test[0] if r.files_under_test else "unknown", []).append(r)
	for group in sorted(groups, key=lambda g: (g == "unknown", g.lower())):
		w("")
		w("== %s (%d)" % (group, len(groups[group])))
		for r in sorted(groups[group], key=lambda r: (number_sort_key(r.numbers[0]) if r.numbers else (9,), r.sort_key())):
			out.extend(entry_lines(r, progress))
	w("")
	w("")
	openq = [r for r in results if r.category == PASSED and r.characterization]
	w("Open questions (NEEDS REVIEW, pass today; behaviour to confirm) (%d)" % len(openq))
	w("-" * 40)
	if not openq:
		w("none")
	for r in sorted(openq, key=lambda r: (number_sort_key(r.numbers[0]) if r.numbers else (9,), r.sort_key())):
		out.extend(entry_lines(r, progress))
	if attention:
		w("")
		w("")
		w("!!! Passed with a note - ATTENTION (%d) !!!" % len(attention))
		w("-" * 40)
		for r in attention:
			out.extend(">>> " + line if line else line for line in entry_lines(r, progress))
	if problems:
		w("")
		w("")
		w("Run problems (%d)" % len(problems))
		w("-" * 40)
		for p in problems:
			w("- " + p)
	w("")
	return "\n".join(out), c


def main(argv=None):
	parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
	parser.add_argument("--build-dir", required=True, help="CMake build directory with the test executables")
	parser.add_argument("--out", help="report file (default: <build-dir>/test_report.log)")
	parser.add_argument("--timeout", type=float, default=60.0,
		help="seconds a single test case may run without progress before it is stopped (default 60)")
	parser.add_argument("--progress", default=os.path.join(REPO_DIR, "Tests", "PROGRESS.md"),
		help="PROGRESS.md with the bug and NEEDS REVIEW tables (default: Tests/PROGRESS.md)")
	parser.add_argument("--python", default=sys.executable, help="Python interpreter for the Python helper tests")
	parser.add_argument("--no-python", action="store_true", help="do not run the Python helper tests")
	parser.add_argument("--exit-zero", action="store_true",
	                    help="always exit with status 0 (failed tests are still reported in the log and the summary)")
	args = parser.parse_args(argv)

	build_dir = os.path.abspath(args.build_dir)
	if not os.path.isdir(build_dir):
		parser.error("build directory not found: %s" % build_dir)
	out_path = os.path.abspath(args.out or os.path.join(build_dir, "test_report.log"))
	started = datetime.datetime.now()
	t0 = time.monotonic()
	config = build_config(build_dir)
	progress = Progress.load(args.progress)
	index = SourceIndex()
	baseline_known = read_baseline_known()

	results, problems, exes_run = [], [], []
	workdir = tempfile.mkdtemp(prefix="test_report_", dir=build_dir)
	try:
		for exe in EXECUTABLES:
			path = find_exe(build_dir, exe)
			if not path:
				continue
			print("running %s ..." % exe, flush=True)
			runner = ExeRunner(exe, path, workdir, args.timeout)
			try:
				exe_results = runner.run()
			except RuntimeError as e:
				problems.append(str(e))
				continue
			patterns = [check_baseline.wildcard_to_regex(p) for p in config["excludes"].get(exe, [])]
			for r in exe_results:
				r.excluded = check_baseline.is_excluded(r.name, patterns)
			results.extend(exe_results)
			problems.extend(runner.problems)
			exes_run.append(exe)
		if not args.no_python:
			print("running the python helper tests ...", flush=True)
			py, py_problems = python_results(os.path.join(TESTS_DIR, "python"), workdir, args.timeout, args.python)
			results.extend(py)
			problems.extend(py_problems)
			exes_run.append(PYTHON_EXE)
	finally:
		shutil.rmtree(workdir, ignore_errors=True)
	if not exes_run:
		parser.error("no test executable found in %s" % build_dir)

	for r in results:
		annotate_numbers(r, progress, baseline_known)
		classify(r, config["sanitize"])
		annotate_files(r, progress, index)
	text, c = render_report(results, problems, config, build_dir, exes_run, progress, started, time.monotonic() - t0)
	with open(out_path, "w", encoding="utf-8", newline="\n") as f:
		f.write(text)
	print("Passed: %d" % c[PASSED])
	print("Failed: %d" % c[FAILED])
	print("Bugged: %d" % c[BUGGED])
	print("Total:  %d (open questions, included in Passed: %d)" % (c["total"], c["open"]))
	if problems:
		print("Run problems: %d" % len(problems))
	for line in attention_banner(sorted([r for r in results if r.category == PASSED and r.attention], key=Result.sort_key)):
		print(line)
	print("Report: %s" % out_path)
	if args.exit_zero:
		return 0
	return 0 if c[FAILED] == 0 and not problems else 1


if __name__ == "__main__":
	sys.exit(main())
