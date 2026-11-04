#!/usr/bin/env python3
"""Condenses a test log into counts, failures, skipped and excluded cases and build errors.

  summarise_results.py [--format auto|doctest|xml|ctest|build] [--exclude NAME]... [FILE | -]

Input (FILE or standard input), format detected automatically or given with --format:
  doctest  the console output of a doctest executable (ffb_unit_tests, ffb_target_tests, ...)
  xml      a doctest XML report (`ffb_xxx_tests -r=xml`)
  ctest    a ctest log (`ctest --output-on-failure`, `ctest -V`): per-test results, the doctest console output of
           every executable (lines prefixed `N: `), the exclusions (`--test-case-exclude=` of the test command)
  build    a compiler / linker log: only the build errors are reported

Output (plain text):
  counts      passed / failed / skipped / known issue. A known issue is a test case that failed as expected
              (KNOWN_ISSUE: doctest should_fail), it is counted in `passed` by doctest and listed separately here
  failed      each failing test case with file:line and its first failed assertion (or exception / crash)
  skipped     the skipped test cases (names only in XML input; the console output only has the count)
  excluded    test cases excluded with --test-case-exclude (from a ctest log) or given with --exclude
  build       compiler errors (file:line) and linker errors (undefined reference ...) found in the input

Exit status: 0 nothing failed, 1 a test case failed or the build failed, 2 usage error or unreadable input.
Standard library only; reads the input, writes nothing.
"""

import argparse
import re
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True

RULE = re.compile(r"^={20,}\s*$")
PREFIX = re.compile(r"^(\d+): ?(.*)$")
SUMMARY = re.compile(r"\[doctest\] test cases:\s*(\d+)\s*\|\s*(\d+) passed\s*\|\s*(\d+) failed\s*\|\s*(\d+) skipped")
ASSERTS = re.compile(r"\[doctest\] assertions:\s*(\d+)\s*\|\s*(\d+) passed\s*\|\s*(\d+) failed")
LOC = re.compile(r"^(.+?):(\d+):\s*$")
ASSERT_LINE = re.compile(r"^(.+?):(\d+): (ERROR|FATAL ERROR|WARNING)?:?\s*(.*)$")
EXPECTED = re.compile(r"Failed (?:exactly \d+ times? )?as expected|Failed as expected")
SHOULD_FAIL_BUT_PASSED = re.compile(r"Should have failed but didn't")
CTEST_RESULT = re.compile(r"^\s*(?:\d+/\d+\s+)?Test\s+#(\d+):\s+(\S+)\s+\.+\s*(\*\*\*)?\s*(.*?)\s+([\d.]+) sec\s*$")
CTEST_START = re.compile(r"^\s*Start\s+(\d+):\s+(\S+)")
CTEST_SUMMARY = re.compile(r"(\d+)% tests passed, (\d+) tests? failed out of (\d+)")
CC_ERROR = re.compile(r"^(.+?):(\d+)(?::(\d+))?: (?:fatal )?error: (.*)$")
LD_ERROR = re.compile(r"(undefined reference to .*|multiple definition of .*|cannot find -l.*)")
WARNING = re.compile(r"^.+?:\d+(?::\d+)?: warning: ")
EXCLUDE_ARG = re.compile(r"(?:--test-case-exclude|-tce)=(\S.*?)(?:\s+--|\s*$)")


class Case:
	def __init__(self, name, location, exe=None):
		self.name = name
		self.location = location  # "file:line"
		self.exe = exe
		self.description = ""
		self.assertion = ""  # first failed assertion: "file:line: text" (+ values)
		self.status = "passed"  # passed | failed | known | skipped
		self.note = ""


class Result:
	def __init__(self):
		self.cases = []
		self.counts = {"cases": 0, "passed": 0, "failed": 0, "skipped": 0}
		self.asserts = {"total": 0, "passed": 0, "failed": 0}
		self.summaries = 0
		self.excluded = []
		self.ctest = []  # (number, name, result, seconds)
		self.build_errors = []
		self.warnings = 0
		self.crashes = []  # (exe, text)
		self.per_exe = []  # (exe, cases, passed, failed, skipped) of each doctest summary line


def short(path):
	"""Shortens an absolute path to the part below Firmware/ (or the last three components)."""
	path = path.replace("\\", "/")
	i = path.find("Firmware/")
	if i >= 0:
		return path[i:]
	return "/".join(path.split("/")[-3:])


def detect(text):
	head = text.lstrip()[:200]
	if head.startswith("<?xml") or head.startswith("<doctest"):
		return "xml"
	if CTEST_SUMMARY.search(text) or CTEST_START.search(text) or re.search(r"^Test project ", text, re.M):
		return "ctest"
	if SUMMARY.search(text) or re.search(r"^TEST CASE:", text, re.M):
		return "doctest"
	return "build"


def scan_build(lines, res):
	prev = ""
	for line in lines:
		if WARNING.match(line):
			res.warnings += 1
		m = CC_ERROR.match(line)
		if m:
			res.build_errors.append("%s:%s: %s" % (short(m.group(1)), m.group(2), m.group(4)))
		else:
			m = LD_ERROR.search(line)
			if m:
				loc = line[:m.start()].strip()
				where = loc if re.match(r".+:\d+:$", loc) else prev
				res.build_errors.append("%s%s" % (short(where) + " " if where else "", m.group(1)))
		prev = line.strip() if "in function" in line else ""


def parse_console(lines, res, exe=None):
	"""Parses doctest console output (prefix already removed). Appends cases that printed something."""
	blocks, cur = [], []
	for line in lines:
		if RULE.match(line):
			blocks.append(cur)
			cur = []
		else:
			cur.append(line)
	blocks.append(cur)
	for block in blocks:
		if not any(l.startswith("TEST CASE:") for l in block):
			for l in block:
				m = SUMMARY.search(l)
				if m:
					total, passed, failed, skipped = (int(g) for g in m.groups())
					res.counts["cases"] += total
					res.counts["passed"] += passed
					res.counts["failed"] += failed
					res.counts["skipped"] += skipped
					res.summaries += 1
					res.per_exe.append((exe or "-", total, passed, failed, skipped))
				m = ASSERTS.search(l)
				if m:
					for k, g in zip(("total", "passed", "failed"), m.groups()):
						res.asserts[k] += int(g)
			continue
		name = next(l[len("TEST CASE:"):].strip() for l in block if l.startswith("TEST CASE:"))
		location = ""
		for l in block:
			m = LOC.match(l)
			if m:
				location = "%s:%s" % (short(m.group(1)), m.group(2))
				break
		case = Case(name, location, exe)
		for i, l in enumerate(block):
			if l.startswith("DESCRIPTION:"):
				case.description = l[len("DESCRIPTION:"):].strip()
			m = ASSERT_LINE.match(l)
			if m and m.group(3) and not case.assertion:
				text = m.group(4).strip()
				values = ""
				if i + 1 < len(block) and block[i + 1].strip().startswith("values:"):
					values = " " + block[i + 1].strip()
				case.assertion = "%s:%s: %s%s" % (short(m.group(1)), m.group(2), text, values)
		joined = "\n".join(block)
		if EXPECTED.search(joined):
			case.status = "known"
		else:
			case.status = "failed"
			if SHOULD_FAIL_BUT_PASSED.search(joined):
				case.note = "should have failed but passed (known issue fixed? remove the marker)"
		res.cases.append(case)


def strip_ctest_prefix(lines):
	"""Splits a ctest log into {test number: [lines without the 'N: ' prefix]} and the unprefixed lines."""
	per_test, other = {}, []
	for line in lines:
		m = PREFIX.match(line)
		if m:
			per_test.setdefault(int(m.group(1)), []).append(m.group(2))
		else:
			other.append(line)
	return per_test, other


def parse_ctest(text, res):
	lines = text.splitlines()
	per_test, other = strip_ctest_prefix(lines)
	names = {}
	for line in other + [l for ls in per_test.values() for l in ls]:
		m = CTEST_START.match(line)
		if m:
			names[int(m.group(1))] = m.group(2)
	for line in other:
		m = CTEST_RESULT.match(line)
		if m:
			result = (m.group(3) or "") + m.group(4)
			res.ctest.append((int(m.group(1)), m.group(2), result.strip(), float(m.group(5))))
			names.setdefault(int(m.group(1)), m.group(2))
	for number, tlines in sorted(per_test.items()):
		for l in tlines:
			if l.startswith("Test command:"):
				for m in EXCLUDE_ARG.finditer(l):
					for item in m.group(1).replace('"', "").split(","):
						if item and item not in res.excluded:
							res.excluded.append(item)
		parse_console(tlines, res, names.get(number))
	# Without -V the output of passing tests is not in the log: counts come from the ctest summary
	if not per_test:
		m = CTEST_SUMMARY.search(text)
		if m:
			failed, total = int(m.group(2)), int(m.group(3))
			res.counts["cases"], res.counts["failed"], res.counts["passed"] = total, failed, total - failed
	# A test that crashed or timed out printed no doctest summary
	for number, name, result, _ in res.ctest:
		if result not in ("Passed",) and not result.startswith("Passed"):
			if not any(c.exe == name for c in res.cases if c.status == "failed"):
				res.crashes.append((name, result))
	scan_build(lines, res)


def parse_xml(text, res, exe=None):
	root = ET.fromstring(text)
	exe = exe or root.get("binary")
	for tc in root.iter("TestCase"):
		name = tc.get("name")
		loc = "%s:%s" % (short(tc.get("filename", "")), tc.get("line", ""))
		case = Case(name, loc, exe)
		case.description = tc.get("description", "")
		own = tc.find("OverallResultsAsserts")
		failures = int(own.get("failures", "0")) if own is not None else 0
		ok = own is None or own.get("test_case_success") == "true"
		if tc.get("skipped") == "true":
			case.status = "skipped"
		elif not ok:
			case.status = "failed"
		elif failures:
			case.status = "known"
		for el in tc.iter():
			if el.tag == "Expression" and el.get("success") == "false":
				orig = " ".join((el.findtext("Original") or "").split())
				exp = " ".join((el.findtext("Expanded") or "").split())
				case.assertion = "%s:%s: %s(%s)%s" % (short(el.get("filename", "")), el.get("line", ""),
					el.get("type", ""), orig, " values: " + exp if exp else "")
				break
			if el.tag in ("Exception", "FatalError") and not case.assertion:
				case.assertion = "%s: %s" % (el.tag, " ".join((el.text or "").split()))
				break
		if case.status != "passed":
			res.cases.append(case)
	m = root.find("OverallResultsTestCases")
	if m is not None:
		res.counts["failed"] += int(m.get("failures", "0"))
		res.counts["skipped"] += int(m.get("skipped", "0"))
		res.counts["passed"] += int(m.get("successes", "0"))
		res.counts["cases"] += int(m.get("successes", "0")) + int(m.get("failures", "0")) + int(m.get("skipped", "0"))
		res.summaries += 1
	m = root.find("OverallResultsAsserts")
	if m is not None:
		s, f = int(m.get("successes", "0")), int(m.get("failures", "0"))
		res.asserts["passed"] += s
		res.asserts["failed"] += f
		res.asserts["total"] += s + f
	# The XML report has no list of the excluded cases: use --exclude


def report(res, fmt):
	out = []
	known = [c for c in res.cases if c.status == "known"]
	failed = [c for c in res.cases if c.status == "failed"]
	skipped = [c for c in res.cases if c.status == "skipped"]
	out.append("Format: %s" % fmt)
	if res.summaries or res.counts["cases"]:
		c = res.counts
		out.append("Test cases: %d  passed: %d  failed: %d  skipped: %d  known issue: %d" % (
			c["cases"], c["passed"] - len(known), c["failed"], c["skipped"], len(known)))
		if len(known):
			out.append("  (doctest counts the %d known issue cases as passed: they failed as expected)" % len(known))
		if res.asserts["total"]:
			a = res.asserts
			out.append("Assertions: %d  passed: %d  failed: %d (including the failed-as-expected ones of the known issues)" % (
				a["total"], a["passed"], a["failed"]))
	elif fmt != "build":
		out.append("Test cases: no summary found in the input")
	if len(res.per_exe) > 1:
		out.append("Per executable (doctest passed includes its known issues):")
		for exe, total, passed, failed_n, skipped_n in res.per_exe:
			out.append("  %-18s %5d cases  %5d passed  %3d failed  %3d skipped" % (exe, total, passed, failed_n, skipped_n))
	if res.ctest:
		out.append("")
		out.append("ctest:")
		for number, name, result, seconds in res.ctest:
			out.append("  #%d %-22s %-14s %.2f s" % (number, name, result, seconds))
	if failed:
		out.append("")
		out.append("Failed test cases (%d):" % len(failed))
		for c in failed:
			out.append("  %s%s" % (c.name, "  [%s]" % c.exe if c.exe else ""))
			out.append("    at %s" % c.location)
			if c.assertion:
				out.append("    first failed assertion: %s" % c.assertion)
			if c.note:
				out.append("    note: %s" % c.note)
	if res.crashes:
		out.append("")
		out.append("ctest tests that did not pass without a doctest result (crash, timeout, not run):")
		for name, result in res.crashes:
			out.append("  %s: %s" % (name, result))
	if skipped:
		out.append("")
		out.append("Skipped test cases (%d):" % len(skipped))
		for c in skipped:
			out.append("  %s  (%s)" % (c.name, c.location))
	elif res.counts["skipped"]:
		out.append("")
		out.append("Skipped test cases: %d (the console output does not name them; use the XML report)" % res.counts["skipped"])
	if known:
		out.append("")
		out.append("Known issues, failed as expected (%d):" % len(known))
		for c in known:
			out.append("  %s  (%s)" % (c.name, c.location))
	if res.excluded:
		out.append("")
		out.append("Excluded test cases (%d):" % len(res.excluded))
		for name in res.excluded:
			out.append("  %s" % name)
	if res.build_errors:
		out.append("")
		out.append("Build errors (%d):" % len(res.build_errors))
		for e in res.build_errors:
			out.append("  %s" % e)
	if fmt in ("build", "ctest") and res.warnings:
		out.append("")
		out.append("Compiler warnings: %d" % res.warnings)
	return "\n".join(out)


def summarise(text, fmt="auto", exclude=()):
	"""Returns (report text, exit status) for the log text."""
	if fmt == "auto":
		fmt = detect(text)
	res = Result()
	if fmt == "xml":
		parse_xml(text, res)
	elif fmt == "ctest":
		parse_ctest(text, res)
	elif fmt == "doctest":
		parse_console(text.splitlines(), res)
		scan_build(text.splitlines(), res)
	elif fmt == "build":
		scan_build(text.splitlines(), res)
	else:
		raise ValueError("unknown format %s" % fmt)
	for name in exclude:
		for item in name.split(","):
			if item and item not in res.excluded:
				res.excluded.append(item)
	bad = res.counts["failed"] > 0 or any(c.status == "failed" for c in res.cases) or bool(res.build_errors) or bool(res.crashes)
	return report(res, fmt), 1 if bad else 0


def main(argv=None):
	parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
	parser.add_argument("file", nargs="?", default="-", help="log file (default and -: standard input)")
	parser.add_argument("--format", choices=("auto", "doctest", "xml", "ctest", "build"), default="auto")
	parser.add_argument("--exclude", action="append", default=[], metavar="NAME",
		help="test case that was excluded from the run (repeatable, or comma separated)")
	args = parser.parse_args(argv)
	try:
		if args.file == "-":
			text = sys.stdin.read()
		else:
			with open(args.file, encoding="utf-8", errors="replace") as f:
				text = f.read()
		text_out, status = summarise(text, args.format, args.exclude)
	except (OSError, ET.ParseError, ValueError) as e:
		sys.stderr.write("summarise_results.py: %s\n" % e)
		return 2
	print(text_out)
	return status


if __name__ == "__main__":
	sys.exit(main())
