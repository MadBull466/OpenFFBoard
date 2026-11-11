"""Bounded clean-counter native run; diagnostics can never count as success."""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import subprocess
import xml.etree.ElementTree as ET

def validate_xml(xml):
    r=ET.fromstring(xml);cases=r.findall(".//TestCase")
    if not cases or any(c.get("skipped")=="true" for c in cases):raise SystemExit("empty/skipped native inventory")
    known=0
    for c in cases:
        t=c.find("OverallResultsAsserts")
        if t is None or t.get("test_case_success")!="true":raise SystemExit("unexpected native case failure")
        failures=c.findall(".//Expression[@success='false']")
        if c.get('should_fail')=='true':
            if ("P12-I2C-001" not in c.get('description','') or int(t.get('failures','0'))!=1 or
                c.get('name')!='characterization: I2C memory interrupt write retains both address bytes when BTF precedes service' or
                len(failures)!=1 or failures[0].get('type')!='CHECK' or
                ''.join(failures[0].find('Original').itertext()).strip()!='observed==std::vector<uint8_t>{0x52,0x12,0x34,0x91,0x42}'):
                raise SystemExit("unexpected known-issue failure or invalid setup")
            known+=1
        elif int(t.get("failures","0")):raise SystemExit("unexpected native failure")
    if known!=1:raise SystemExit("missing known-issue regression")
    totals=r.find("OverallResultsTestCases")
    if (totals is None or int(totals.get("successes"))!=len(cases) or
        int(totals.get("failures")) or int(totals.get('skipped','0'))):raise SystemExit("invalid native totals")
    assertions=r.find('OverallResultsAsserts')
    if assertions is None or int(assertions.get('failures','0'))!=known:
        raise SystemExit('invalid native assertion totals')
    successes=sum(int(c.find('OverallResultsAsserts').get('successes','0')) for c in cases)
    if int(assertions.get('successes','0'))!=successes:raise SystemExit('inconsistent native assertion totals')
    return r,cases,known

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--build",type=Path,required=True);p.add_argument("--out",type=Path,required=True)
    p.add_argument("--seed",type=int,default=271);a=p.parse_args()
    b=a.build.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
    (out/"summary.json").unlink(missing_ok=True)
    # Recompiled LLVM gcov objects cannot merge counters from a prior shape.
    for path in b.rglob("*.gcda"):path.unlink()
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))
    cmd=[str(b/"p12_native32"),"--reporters=xml","--no-version","--order-by=rand",f"--rand-seed={a.seed}"]
    (out/"command.json").write_text(json.dumps(dict(argv=cmd,cwd=str(b)),indent=2)+"\n")
    result=subprocess.run(cmd,cwd=b,capture_output=True,timeout=30)
    (out/"results.xml").write_bytes(result.stdout);(out/"stderr.log").write_bytes(result.stderr)
    if result.returncode or result.stderr:raise SystemExit(f"native failure/diagnostic: exit={result.returncode}; see {out}")
    r,cases,known=validate_xml(result.stdout)
    sources=json.loads((b/"compile_commands.json").read_text());paths={c["file"] for c in sources}
    here=Path(__file__).resolve().parent
    paths.update(str(p) for p in here.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    for source in sources:
        path=Path(source['file'])
        if '/STM32F4xx_HAL_Driver/Src/' in str(path):
            drivers=path.parents[2]
            paths.update(str(p) for p in drivers.rglob('*.h'))
            paths.add(str(drivers.parent/'Core/Inc/stm32f4xx_hal_conf.h'))
    firmware=here.parents[3]
    for relative in ('Tests/support/TestHelpers.h','Tests/third_party/doctest.h','Tests/baseline/test_cases.txt'):
        paths.add(str(firmware/relative))
    paths.add(str(b/'compile_commands.json'))
    paths.add(str(b/'CMakeCache.txt'))
    hashes={s:hashlib.sha256(Path(s).read_bytes()).hexdigest() for s in sorted(paths)}
    summary=dict(cases=len(cases),ordinary=len(cases)-known,known_issues=known,assertions=r.find("OverallResultsAsserts").attrib,source_sha256=hashes)
    (out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print("native:",summary["ordinary"],"ordinary passes;",known,"known issue;",summary["assertions"])
