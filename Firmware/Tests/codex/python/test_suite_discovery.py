"""The routine gate must execute tests, not accept empty discovery."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


class SuiteDiscovery(unittest.TestCase):
    def run_group(self, body=None):
        with tempfile.TemporaryDirectory(prefix='codex-gate-') as directory:
            if body is not None:
                Path(directory, 'test_sample.py').write_text(body, encoding='utf-8')
            return subprocess.run([sys.executable, '-B', str(Path(__file__).with_name('run_suite.py')),
                                   '--start', directory], capture_output=True, text=True, timeout=10)

    def test_empty_discovery_fails_the_gate(self):
        result = self.run_group()
        self.assertEqual(result.returncode, 1)
        self.assertIn('found no tests', result.stderr)

    def test_a_real_successful_case_passes_the_gate(self):
        result = self.run_group('import unittest\nclass Sample(unittest.TestCase):\n def test_value(self): self.assertEqual(2+3,5)\n')
        self.assertEqual(result.returncode, 0)
        self.assertIn('Ran 1 test', result.stderr)

    def test_a_real_failed_case_fails_the_gate(self):
        result = self.run_group('import unittest\nclass Sample(unittest.TestCase):\n def test_value(self): self.assertEqual(2+3,6)\n')
        self.assertEqual(result.returncode, 1)
        self.assertIn('FAILED (failures=1)', result.stderr)
