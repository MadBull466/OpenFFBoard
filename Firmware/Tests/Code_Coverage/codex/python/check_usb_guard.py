"""Verify the safe USB setup-fault probe fails at the intended boundary."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET

CASE = 'successful enumeration reaches derived log clearing without a setup fault'


def verify(args):
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    command = [args.exe, '--reporters=xml', '--test-case=' + CASE]
    try:
        result = subprocess.run(command, capture_output=True, timeout=30,
                                env=dict(os.environ, CODEX_USB_SETUP_FAULT_PROBE='1'))
    except (OSError, subprocess.TimeoutExpired) as error:
        print('USB guard probe did not finish: ' + str(error), file=sys.stderr)
        return 1
    (output / 'probe.xml').write_bytes(result.stdout)
    (output / 'probe.stderr').write_bytes(result.stderr)
    evidence = dict(argv=command, exit=result.returncode)
    problem = ''

    def require(condition, message):
        if not condition:
            raise ValueError(message)

    try:
        root = ET.fromstring(result.stdout)
        cases = [node for node in root.iter('TestCase') if node.get('name') == CASE]
        totals = root.find('OverallResultsTestCases')
        require(result.returncode == 1, 'expected ordinary assertion exit 1, not success/crash')
        require(not result.stderr, 'unexpected stderr: possible sanitizer/runtime diagnostic')
        require(totals is not None and totals.get('failures') == '1' and totals.get('successes') == '0', 'expected one selected failed case')
        require(len(cases) == 1 and cases[0].get('skipped') != 'true', 'probe must execute exactly once')
        failures = [node for node in cases[0].iter('Expression') if node.get('success') == 'false']
        require(bool(failures), 'expected an assertion failure')
        first = failures[0]
        expression = ''.join(first.find('Original').itertext()).strip()
        require(expression == 'firstFault(host).empty()', 'unexpected first assertion: ' + expression)
        require(first.get('filename', '').replace('\\', '/').endswith('/UsbFixture.h'), 'first failure must be the shared setup guard')
        text = result.stdout.decode(errors='replace')
        require('dcd_remote_wakeup' in text, 'injected fault was not reported')
        require('CODEX_USB_DISCARD_REACHED' not in text, 'fault reached the discard operation')
        evidence['first_failure'] = dict(first.attrib, expression=expression)
    except (ET.ParseError, ValueError, AttributeError) as error:
        problem = str(error)
    evidence['problem'] = problem
    (output / 'result.json').write_text(json.dumps(evidence, indent=2) + '\n', encoding='utf-8')
    if problem:
        print('USB guard verification failed: ' + problem, file=sys.stderr)
        return 1
    print('USB setup fault rejected at the shared guard before log clearing')
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exe', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    class SetupGuardCheck(unittest.TestCase):
        def runTest(self):
            self.assertEqual(verify(args), 0, 'setup fault must fail at the shared boundary before discard')

    return 0 if unittest.TextTestRunner(verbosity=2).run(SetupGuardCheck()).wasSuccessful() else 1


if __name__ == '__main__':
    sys.exit(main())
