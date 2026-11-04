#!/usr/bin/env python3
"""Checks that every baseline test case of ffb_unit_tests is still present with its expected result.

The baseline (baseline/test_cases.txt) lists the test cases that existed before the suite was extended,
one per line: <expected result> TAB <test suite> TAB <test case name>. Expected results:

  pass            runs and passes
  known_issue     KNOWN_ISSUE(...): runs and fails as expected (doctest should_fail)
  known_issue_ub  KNOWN_ISSUE_UB(...): like known_issue, skipped in sanitizer builds
  skip            skipped by a doctest::skip() decorator

The checker runs the test executable with the doctest XML reporter and the same exclusions as ctest.
Excluded cases must still exist, their result is not checked (they are listed in the report).
New test cases are allowed and only counted.

  check_baseline.py --build-dir ../ffb-test-build/default
  check_baseline.py --exe path/to/ffb_unit_tests --exclude-list "name 1,name 2" [--sanitize]
  check_baseline.py --build-dir ... --print-current     # classification of the current cases (stdout)

With --build-dir the exclusions and the sanitizer setting are read from CMakeCache.txt
(FFB_TESTS_EXCLUDED_ARG, FFB_TESTS_SANITIZE). Exit code: 0 ok, 1 baseline violated, 2 usage error or crash.
Only the Python standard library is used. Nothing is written to disk.
"""

import argparse
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_BASELINE = os.path.join(TOOLS_DIR, "..", "baseline", "test_cases.txt")
EXE_NAME = "ffb_unit_tests"
EXPECTED_VALUES = ("pass", "known_issue", "known_issue_ub", "skip")
UB_PREFIX = "KNOWN ISSUE (undefined behaviour):"


def read_cmake_cache(build_dir):
	values = {}
	path = os.path.join(build_dir, "CMakeCache.txt")
	if not os.path.isfile(path):
		return values
	with open(path, encoding="utf-8", errors="replace") as f:
		for line in f:
			m = re.match(r"^([A-Za-z0-9_]+):[A-Z]+=(.*)$", line.rstrip("\r\n"))
			if m:
				values[m.group(1)] = m.group(2)
	return values


def find_exe(build_dir):
	for sub in ("", "Debug", "Release", "RelWithDebInfo", "MinSizeRel"):
		for name in (EXE_NAME, EXE_NAME + ".exe"):
			path = os.path.join(build_dir, sub, name)
			if os.path.isfile(path):
				return path
	return None


def cmake_bool(value):
	return value.strip().upper() in ("1", "ON", "YES", "TRUE", "Y")


def read_baseline(path):
	entries = []
	with open(path, encoding="utf-8") as f:
		for number, line in enumerate(f, 1):
			line = line.rstrip("\r\n")
			if not line or line.startswith("#"):
				continue
			parts = line.split("\t")
			if len(parts) != 3 or parts[0] not in EXPECTED_VALUES:
				raise ValueError("%s:%d: expected '<result>\\t<suite>\\t<name>', got %r" % (path, number, line))
			entries.append((parts[0], parts[1], parts[2]))
	return entries


def run_tests(exe, excluded):
	cmd = [exe, "--reporters=xml", "--no-version"]
	if excluded:
		cmd.append("--test-case-exclude=" + ",".join(excluded))
	proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	return proc.returncode, proc.stdout.decode("utf-8", errors="replace"), proc.stderr.decode("utf-8", errors="replace")


def last_started_case(xml_text):
	names = re.findall(r'<TestCase name="([^"]*)"', xml_text)
	return names[-1] if names else "?"


def classify(xml_text, excluded_patterns):
	"""Returns {(suite, name): (metadata class, actual result)} and the doctest totals."""
	root = ET.fromstring(xml_text)
	cases = {}
	for suite in root.iter("TestSuite"):
		suite_name = suite.get("name", "")
		for case in suite.findall("TestCase"):
			name = case.get("name", "")
			description = case.get("description", "")
			should_fail = case.get("should_fail") == "true"
			skipped = case.get("skipped") == "true"
			if description.startswith(UB_PREFIX):
				meta = "known_issue_ub"
			elif should_fail:
				meta = "known_issue"
			elif skipped and not is_excluded(name, excluded_patterns):
				meta = "skip"
			else:
				meta = "pass"
			if skipped:
				actual = "excluded" if is_excluded(name, excluded_patterns) else "skipped"
			elif any(e.get("crash") == "true" for e in case.iter("Exception")):
				actual = "crashed" # doctest stops the run, later cases are missing from the report
			else:
				result = case.find("OverallResultsAsserts")
				success = result is not None and result.get("test_case_success") == "true"
				if should_fail:
					actual = "failed_as_expected" if success else "did_not_fail"
				else:
					actual = "passed" if success else "failed"
			cases[(suite_name, name)] = (meta, actual)
	totals = root.find("OverallResultsTestCases")
	return cases, (totals.attrib if totals is not None else {})


def wildcard_to_regex(pattern):
	# doctest filters: '*' any sequence, '?' one character, case insensitive, whole name
	out = ""
	for ch in pattern:
		out += ".*" if ch == "*" else "." if ch == "?" else re.escape(ch)
	return re.compile("^" + out + "$", re.IGNORECASE | re.DOTALL)


def is_excluded(name, patterns):
	return any(p.match(name) for p in patterns)


def expected_actual(expected, sanitize):
	if expected == "pass":
		return "passed"
	if expected == "known_issue":
		return "failed_as_expected"
	if expected == "known_issue_ub":
		return "skipped" if sanitize else "failed_as_expected"
	return "skipped"


def main():
	parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
	parser.add_argument("--build-dir", help="CMake build directory (exe, exclusions and sanitizer flag are read from it)")
	parser.add_argument("--exe", help="path of the ffb_unit_tests executable")
	parser.add_argument("--baseline", default=DEFAULT_BASELINE, help="baseline file (default: baseline/test_cases.txt)")
	parser.add_argument("--exclude-list", help="comma separated test cases to exclude, like doctest --test-case-exclude")
	parser.add_argument("--sanitize", choices=("auto", "on", "off"), default="auto",
		help="sanitizer build: KNOWN_ISSUE_UB cases are expected to be skipped (auto: from CMakeCache.txt)")
	parser.add_argument("--print-current", action="store_true", help="print the current cases in baseline format and exit")
	args = parser.parse_args()

	cache = read_cmake_cache(args.build_dir) if args.build_dir else {}
	exe = args.exe or (find_exe(args.build_dir) if args.build_dir else None)
	if not exe or not os.path.isfile(exe):
		parser.error("test executable not found (use --build-dir or --exe)")
	exclude_arg = args.exclude_list if args.exclude_list is not None else cache.get("FFB_TESTS_EXCLUDED_ARG", "")
	excluded = [e for e in exclude_arg.split(",") if e.strip()]
	patterns = [wildcard_to_regex(e) for e in excluded]
	sanitize = cmake_bool(cache.get("FFB_TESTS_SANITIZE", "OFF")) if args.sanitize == "auto" else args.sanitize == "on"

	code, out, err = run_tests(exe, excluded)
	try:
		cases, totals = classify(out, patterns)
	except ET.ParseError:
		print("ERROR: %s did not finish (exit code %d). Last test case started: %s" % (exe, code, last_started_case(out)))
		print(err[-2000:])
		return 2

	if args.print_current:
		for (suite, name), (meta, _) in cases.items():
			print("%s\t%s\t%s" % (meta, suite, name))
		return 0

	baseline = read_baseline(args.baseline)
	problems = []
	not_checked = []
	for expected, suite, name in baseline:
		key = (suite, name)
		if key not in cases:
			problems.append("MISSING   %s / %s" % (suite, name))
			continue
		meta, actual = cases[key]
		if actual == "excluded":
			not_checked.append("%s / %s (expected: %s)" % (suite, name, expected))
			continue
		want = expected_actual(expected, sanitize)
		if actual != want:
			problems.append("CHANGED   %s / %s: expected %s, got %s" % (suite, name, want, actual))
	baseline_keys = set((s, n) for _, s, n in baseline)
	new_cases = [k for k in cases if k not in baseline_keys]
	new_failed = [k for k in new_cases if cases[k][1] in ("failed", "did_not_fail", "crashed")]
	crashed = [k for k in cases if cases[k][1] == "crashed"]
	for suite, name in crashed:
		problems.insert(0, "CRASHED   %s / %s (the run stopped here)" % (suite, name))

	print("Executable:   %s" % exe)
	print("Sanitizer:    %s" % ("on" if sanitize else "off"))
	print("Excluded:     %s" % (", ".join(excluded) if excluded else "none"))
	print("doctest:      %s passed, %s failed, %s skipped (exit code %d)" % (
		totals.get("successes", "?"), totals.get("failures", "?"), totals.get("skipped", "?"), code))
	print("Baseline:     %d cases, %d checked, %d excluded (not checked)" % (
		len(baseline), len(baseline) - len(not_checked) - sum(p.startswith("MISSING") for p in problems), len(not_checked)))
	for line in not_checked:
		print("  not checked: " + line)
	print("New cases:    %d (%d failed)" % (len(new_cases), len(new_failed)))
	for suite, name in new_failed:
		print("  FAILED    %s / %s" % (suite, name))
	for line in problems:
		print(line)
	if crashed:
		print("RESULT: FAIL (crash)")
		return 2
	if problems or new_failed or code != 0:
		print("RESULT: FAIL")
		return 1
	print("RESULT: OK")
	return 0


if __name__ == "__main__":
	sys.exit(main())
