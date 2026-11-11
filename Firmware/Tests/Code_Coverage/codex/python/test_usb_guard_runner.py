"""Fail-closed protocol checks for the exact USB setup-guard subprocess."""
import contextlib
import io
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock
import check_usb_guard as guard


class UsbGuardRunner(unittest.TestCase):
    def verify(self, expression='firstFault(host).empty()', code=1, stderr=b'', discard=False):
        text = ('<doctest><TestSuite><TestCase name="' + guard.CASE + '">'
                '<Expression success="false" filename="fixture/UsbFixture.h"><Original>' + expression +
                '</Original><Info>dcd_remote_wakeup</Info></Expression>' +
                ('<Message>CODEX_USB_DISCARD_REACHED</Message>' if discard else '') +
                '</TestCase></TestSuite><OverallResultsTestCases successes="0" failures="1"/></doctest>')
        with tempfile.TemporaryDirectory(prefix='codex-usb-check-') as directory:
            result = subprocess.CompletedProcess([], code, text.encode(), stderr)
            with mock.patch.object(guard.subprocess, 'run', return_value=result) as run, \
                    contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                status = guard.verify(SimpleNamespace(exe='fake-usb', output=directory))
            self.assertEqual(run.call_args.kwargs['env']['CODEX_USB_SETUP_FAULT_PROBE'], '1')
            self.assertTrue(Path(directory, 'result.json').is_file())
            return status

    def test_exact_guard_failure_is_accepted(self):
        self.assertEqual(self.verify(), 0)

    def test_an_unrelated_first_failure_is_rejected(self):
        self.assertEqual(self.verify(expression='unrelated_setup_check'), 1)

    def test_a_sanitizer_diagnostic_is_rejected_even_with_the_expected_xml(self):
        self.assertEqual(self.verify(stderr=b'ERROR: AddressSanitizer: example'), 1)

    def test_a_process_crash_is_rejected(self):
        self.assertEqual(self.verify(code=-11), 1)

    def test_reaching_log_discard_is_rejected(self):
        self.assertEqual(self.verify(discard=True), 1)
