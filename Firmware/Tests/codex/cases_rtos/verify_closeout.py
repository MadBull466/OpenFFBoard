"""Scratch linking against immutable baseline objects; no vendor source mutation.
Run from repository root. Results/commands under ../ffb-test-build/codex-closeout-rtos.
"""
import json
from pathlib import Path
import shlex
import subprocess
import xml.etree.ElementTree as ET

root = Path.cwd()
base = root.parent / 'ffb-test-build/codex-baseline-default'
out = root.parent / 'ffb-test-build/codex-closeout-rtos'
out.mkdir(parents=True, exist_ok=True)
entries = json.loads((base / 'compile_commands.json').read_text())
records = []
def run(label, args):
    result = subprocess.run([str(x) for x in args], cwd=out, capture_output=True, text=True)
    (out / (label + '.stdout')).write_text(result.stdout)
    (out / (label + '.stderr')).write_text(result.stderr)
    rec = dict(label=label, argv=[str(x) for x in args], cwd=str(out), code=result.returncode)
    if result.stdout.startswith('<?xml'):
        tree = ET.fromstring(result.stdout)
        rec['cases'] = [{**n.attrib, 'failed': [e.attrib for e in n.findall('.//Expression') if e.get('success') == 'false']} for n in tree.findall('.//TestCase')]
    records.append(rec)
    (out / 'commands_results.json').write_text(json.dumps(records, indent=2))
    if result.returncode and not label.endswith('_run'):
        raise RuntimeError(label + ': ' + result.stderr + result.stdout)
    return result

def verify_run(label, args, expected_code, expected_successes, expected_failures):
    result = run(label, args)
    counts = ET.fromstring(result.stdout).find('OverallResultsTestCases').attrib
    assert result.returncode == expected_code, (label, result.returncode)
    assert (int(counts['successes']), int(counts['failures'])) == (expected_successes, expected_failures), (label, counts)

def compile_file(label, source, template):
    e = next(e for e in entries if 'ffb_rtos_tests.dir' in e['command'] and e['file'].endswith(template))
    cmd = shlex.split(e['command']); pos = cmd.index('-o'); cmd[pos+1] = str(out / (label + '.o')); cmd[-1] = str(source)
    run(label+'_compile', cmd)
    return out / (label+'.o')

source = root / 'Firmware/Tests/codex/cases_rtos/test_closeout_rtos.cpp'
# Only remove this new expected-failure decoration in the copied test; originals remain intact.
plain = out / 'unmarked_test.cpp'
lines = source.read_text().splitlines()
plain.write_text('\n'.join('    ) {' if '* KNOWN_ISSUE(' in line else line for line in lines)+'\n')
new = compile_file('new', source, 'test_ctasklet.cpp')
unmarked = compile_file('unmarked', plain, 'test_ctasklet.cpp')
fixed = compile_file('fixed_fake', root/'Firmware/Tests/platform/src/host_freertos.cpp', 'host_freertos.cpp')
link = shlex.split((base/'CMakeFiles/ffb_rtos_tests.dir/link.txt').read_text())
link = [str(base/x) if x.endswith('.o') else x for x in link if not x.startswith('-Wl,--dependency-file=')]

def binary(label, test, fake=None, tasklet=None):
    args = list(link)
    for i,x in enumerate(args):
        if fake and x.endswith('/platform/src/host_freertos.cpp.o'): args[i] = str(fake)
        if tasklet and x.endswith('/Src/ctasklet.cpp.o'): args[i] = str(tasklet)
    args[args.index('-o')+1] = str(out/label)
    args.insert(args.index('-o'), str(test))
    run(label+'_link', args)
    return out/label

current = binary('current', new, fixed)
verify_run('current_full_run', [current, '-r=xml'], 0, 278, 0)
raw = binary('current_unmarked', unmarked, fixed)
verify_run('current_tasklet_run', [raw, '-ts=CodexRtosCloseout', '-tc=*pending tasklet*', '-r=xml'], 1, 0, 1)
oldfake = binary('old_fake', new)
verify_run('old_fake_run', [oldfake, '-ts=CodexRtosCloseout', '-tc=Sync*', '-r=xml'], 1, 3, 1)
for name, replacement in [('proper', 'if (xSemaphoreTake(DtorLock, CmdTimeout) != pdTRUE) return false;'), ('unsafe', 'xSemaphoreTake(DtorLock, CmdTimeout);')]:
    copied = out / (name+'_ctasklet.cpp')
    original = (root/'Firmware/FFBoard/Src/ctasklet.cpp').read_text()
    before, schedule = original.split('bool Tasklet::Schedule(', 1)
    assert schedule.count('xSemaphoreTake(DtorLock, portMAX_DELAY);') == 1
    copied.write_text(before + 'bool Tasklet::Schedule(' + schedule.replace('xSemaphoreTake(DtorLock, portMAX_DELAY);', replacement))
    obj = compile_file(name, copied, '/ctasklet.cpp')
    exe = binary(name, unmarked, fixed, obj)
    verify_run(name+'_run', [exe, '-ts=CodexRtosCloseout', '-r=xml'], 0 if name == 'proper' else 1, 7 if name == 'proper' else 6, 0 if name == 'proper' else 1)
print(out/'commands_results.json')
