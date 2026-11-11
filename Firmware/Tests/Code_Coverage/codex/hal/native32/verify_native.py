"""One reproducible P12 native gate, including evidence and coverage controls."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True)
    p.add_argument('--device',choices=('F407VG','F407VG_DISCO','F411RE'),default='F407VG')
    p.add_argument('--ubsan',action='store_true');p.add_argument('--full-assert',action='store_true');a=p.parse_args()
    build=a.build.resolve();build.mkdir(parents=True,exist_ok=True)
    evidence=build/'evidence/gate';evidence.mkdir(parents=True,exist_ok=True)
    commands=[['cmake','-S',str(HERE),'-B',str(build),'-DCMAKE_C_COMPILER=clang','-DCMAKE_CXX_COMPILER=clang++',
               '-DP12_DEVICE='+a.device,'-DP12_UBSAN='+('ON' if a.ubsan else 'OFF'),
               '-DP12_FULL_ASSERT='+('ON' if a.full_assert else 'OFF'),'-DP12_FULL_LL=ON','-DP12_COVERAGE=ON',
               '-DP12_ACCESS_MODEL=ON','-DP12_BUS_SOURCE='+str(HERE/'host/access.cpp'),
               '-DP12_NATIVE_SOURCE_DIR='+str(HERE.parents[3]/'Targets'/a.device/'Drivers/STM32F4xx_HAL_Driver/Src'),
               '-DCMAKE_BUILD_TYPE=Debug'],
              ['cmake','--build',str(build),'--parallel','2'],
              [sys.executable,'-B',str(HERE/'test_evidence.py')],
              [sys.executable,'-B',str(HERE/'run_native.py'),'--build',str(build),'--out',str(evidence)],
              [sys.executable,'-B',str(HERE/'coverage_native.py'),'--build',str(build),'--out',str(evidence/'coverage')]]
    manifest=evidence/'gate.json';manifest.unlink(missing_ok=True)
    for index,argv in enumerate(commands):
        result=subprocess.run(argv,cwd=HERE,capture_output=True,timeout=180)
        (evidence/f'step-{index}.log').write_bytes(result.stdout+result.stderr)
        if result.returncode:raise SystemExit(f'P12 gate failed at step {index}; see {evidence}')
    manifest.write_text(json.dumps(dict(device=a.device,ubsan=a.ubsan,full_assert=a.full_assert,commands=commands),indent=2)+'\n')
    print(f'P12 {a.device} gate passed; evidence: {evidence}')
