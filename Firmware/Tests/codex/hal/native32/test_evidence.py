"""Failure controls for P12 native evidence collection (stdlib unittest)."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from run_native import validate_xml

COLLECTOR=Path(__file__).with_name('coverage_native.py')

class CoverageEvidence(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.build=self.root/'build';self.build.mkdir()
        (self.build/'empty_control.c.gcno').write_bytes(b'empty-notes')
    def collect(self):
        return subprocess.run([sys.executable,'-B',str(COLLECTOR),'--build',str(self.build),
                               '--out',str(self.root/'out')],capture_output=True,text=True)
    def inventory(self):
        (self.build/'compile_commands.json').write_text(json.dumps([
            {'file':'/fixture/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c'}]))
    def test_empty_inventory_cannot_report_a_percentage(self):
        (self.build/'compile_commands.json').write_text('[]')
        p=self.collect();self.assertNotEqual(p.returncode,0)
        self.assertIn('empty HAL source inventory',p.stderr)
        self.assertFalse((self.root/'out/coverage.json').exists())
    def test_missing_notes_cannot_disappear_from_denominator(self):
        self.inventory();p=self.collect();self.assertNotEqual(p.returncode,0)
        self.assertIn('missing or duplicate notes',p.stderr)
    def test_missing_execution_is_not_treated_as_an_inactive_file(self):
        self.inventory();(self.build/'stm32f4xx_hal_gpio.c.gcno').write_bytes(b'active-notes')
        p=self.collect();self.assertNotEqual(p.returncode,0)
        self.assertIn('missing execution counters',p.stderr)
        self.assertFalse((self.root/'out/coverage.json').exists())
    def test_duplicate_notes_cannot_select_a_convenient_copy(self):
        self.inventory()
        for name in ('a','b'):
            directory=self.build/name;directory.mkdir()
            (directory/'stm32f4xx_hal_gpio.c.gcno').write_bytes(b'active-notes')
        p=self.collect();self.assertNotEqual(p.returncode,0)
        self.assertIn('missing or duplicate notes',p.stderr)

class NativeEvidence(unittest.TestCase):
    def document(self):
        root=ET.Element('doctest')
        case=ET.SubElement(root,'TestCase',name='characterization: I2C memory interrupt write retains both address bytes when BTF precedes service',
                           description='KNOWN ISSUE: P12-I2C-001: address byte',should_fail='true')
        expr=ET.SubElement(case,'Expression',success='false',type='CHECK')
        ET.SubElement(expr,'Original').text='observed==std::vector<uint8_t>{0x52,0x12,0x34,0x91,0x42}'
        ET.SubElement(case,'OverallResultsAsserts',successes='10',failures='1',test_case_success='true')
        ET.SubElement(root,'OverallResultsTestCases',successes='1',failures='0',skipped='0')
        ET.SubElement(root,'OverallResultsAsserts',successes='10',failures='1')
        return root,case
    def validate(self,root):return validate_xml(ET.tostring(root))
    def test_exact_isolated_known_failure_is_accepted(self):
        root,_=self.document();self.assertEqual(self.validate(root)[2],1)
    def test_fixture_failure_cannot_satisfy_a_known_issue(self):
        root,case=self.document();case.find('Expression/Original').text='HAL_I2C_Init(&h)==HAL_OK'
        with self.assertRaises(SystemExit):self.validate(root)
    def test_an_extra_failure_invalidates_the_marker(self):
        root,case=self.document();case.find('OverallResultsAsserts').set('failures','2')
        with self.assertRaises(SystemExit):self.validate(root)
    def test_skip_cannot_satisfy_a_known_issue(self):
        root,case=self.document();case.set('skipped','true')
        with self.assertRaises(SystemExit):self.validate(root)
    def test_a_repaired_behavior_requires_marker_review(self):
        root,case=self.document();case.remove(case.find('Expression'))
        case.find('OverallResultsAsserts').set('failures','0')
        with self.assertRaises(SystemExit):self.validate(root)

if __name__=='__main__':unittest.main()
