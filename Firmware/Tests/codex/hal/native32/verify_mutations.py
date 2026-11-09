"""Bounded native real-C/model fault discrimination; scratch copies only."""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import shutil
import subprocess
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
FW=HERE.parents[3]
CASES={
 'gpio-set':('stm32f4xx_hal_gpio.c','GPIOx->BSRR = GPIO_Pin;',
             'GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U;',
             'real C GPIO set reset and toggle obey modeled output-bit effects','->ODR==0xaaab'),
 'spi-word':('stm32f4xx_hal_spi.c','hspi->Instance->DR = *((const uint16_t *)hspi->pTxBuffPtr);',
             'hspi->Instance->DR = *((const uint16_t *)hspi->pTxBuffPtr) ^ 1U;',
             'SPI sixteen bit transmit retains full word width','->DR==0x7654'),
 'bus-w1c':('access.cpp','store(a,n,old&~value); EXTI->SWIER=',
            'store(a,n,old&value); EXTI->SWIER=',
            'write-one-to-clear model preserves unselected pending requests','->PR==(1U<<9)')}

def command(argv,cwd,output):
    result=subprocess.run(argv,cwd=cwd,capture_output=True,timeout=120)
    output.write_bytes(result.stdout+result.stderr)
    if result.returncode:raise SystemExit('build/config failure: '+str(output))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True)
    p.add_argument('--variant',choices=CASES,required=True);a=p.parse_args()
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))
    build=a.root.resolve()/a.variant;build.mkdir(parents=True,exist_ok=True)
    (build/'mutation.json').unlink(missing_ok=True)
    file,old,new,case,expression=CASES[a.variant]
    source=FW/'Targets/F407VG/Drivers/STM32F4xx_HAL_Driver/Src'
    args=['cmake','-S',str(HERE),'-B',str(build),'-DCMAKE_C_COMPILER=clang','-DCMAKE_CXX_COMPILER=clang++']
    if file=='access.cpp':
        original=HERE/'host/access.cpp';mutant=build/'access.cpp';args+=['-DP12_BUS_SOURCE='+str(mutant)]
    else:
        copied=build/'source';copied.mkdir(exist_ok=True)
        for path in source.glob('*.c'):shutil.copy2(path,copied/path.name)
        original=source/file;mutant=copied/file;args+=['-DP12_NATIVE_SOURCE_DIR='+str(copied)]
    text=original.read_text();count=text.count(old)
    if not count:raise SystemExit('mutation anchor absent')
    mutant.write_text(text.replace(old,new))
    command(args,HERE,build/'configure.log')
    command(['cmake','--build',str(build),'--parallel','2'],HERE,build/'build.log')
    for path in build.rglob('*.gcda'):path.unlink()
    argv=[str(build/'p12_native32'),'--reporters=xml','--no-version','--test-case='+case]
    result=subprocess.run(argv,cwd=build,capture_output=True,timeout=30)
    (build/'results.xml').write_bytes(result.stdout);(build/'stderr.log').write_bytes(result.stderr)
    if result.returncode!=1 or result.stderr:raise SystemExit('mutation crashed, diagnosed or survived')
    root=ET.fromstring(result.stdout);cases=root.findall('.//TestCase')
    active=[c for c in cases if c.get('skipped')!='true']
    if len(active)!=1 or active[0].get('name')!=case:raise SystemExit('wrong mutation inventory')
    c=active[0];failures=c.findall(".//Expression[@success='false']")
    failed=[''.join(e.find('Original').itertext()).strip() for e in failures]
    if c.get('should_fail')=='true' or not failures or any(e.get('type')!='CHECK' for e in failures) or not any(f.endswith(expression) for f in failed):
        raise SystemExit('mutation did not fail its independent ordinary assertion')
    manifest=dict(variant=a.variant,replacements=count,argv=argv,expected_expression=expression,
                  failures=failed,original_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
                  mutant_sha256=hashlib.sha256(mutant.read_bytes()).hexdigest())
    (build/'mutation.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(a.variant+': detected by ordinary assertions')
