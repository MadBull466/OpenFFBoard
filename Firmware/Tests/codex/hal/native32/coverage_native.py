"""Report real HAL C coverage; retain zero-data files and inactive units."""
import argparse
import json
from pathlib import Path
import re
import subprocess

if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--build",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args()
    build=args.build.resolve(); out=args.out.resolve(); out.mkdir(parents=True,exist_ok=True)
    (out/"coverage.json").unlink(missing_ok=True)
    commands=json.loads((build/"compile_commands.json").read_text())
    sources=sorted({c["file"] for c in commands if "/STM32F4xx_HAL_Driver/Src/" in c["file"]})
    if not sources: raise SystemExit("empty HAL source inventory")
    rows=[]
    controls=list(build.rglob("empty_control.c.gcno"))
    if len(controls)!=1: raise SystemExit("missing compiler empty-TU control")
    empty_notes=controls[0].read_bytes()
    for source in sources:
        name=Path(source).name
        notes=list(build.rglob(name+".gcno"))
        if len(notes)!=1: raise SystemExit("missing or duplicate notes: "+name)
        if notes[0].read_bytes()!=empty_notes and not notes[0].with_suffix('.gcda').is_file():
            raise SystemExit("missing execution counters: "+name)
        gcov=out/(name+".gcov")
        if gcov.exists(): gcov.unlink()
        p=subprocess.run(["llvm-cov","gcov","-f","-b","-c",str(notes[0])],cwd=out,capture_output=True,text=True)
        (out/(name+".collector.log")).write_text(p.stdout+p.stderr)
        if p.returncode or p.stderr: raise SystemExit("collector failed: "+name+"\n"+p.stderr)
        if not gcov.exists():
            if notes[0].read_bytes()!=empty_notes:
                raise SystemExit("missing gcov document: "+name)
            rows.append(dict(source=source,lines=0,covered=0,inactive=True,functions=[]));continue
        lines={}
        functions=[dict(name=m.group(1),percent=float(m.group(2)),lines=int(m.group(3)))
                   for m in re.finditer(r"Function '([^']+)'\nLines executed:([0-9.]+)% of (\d+)",p.stdout)]
        if not functions:raise SystemExit("missing function summary: "+name)
        for line in gcov.read_text().splitlines():
            match=re.match(r"\s*([^:]+):\s*(\d+):(.*)",line)
            if match:
                count,number,_=match.groups();count=count.strip().rstrip("*");number=int(number)
                if number and count!="-":
                    lines[number]=int(count) if count.isdigit() else 0

        rows.append(dict(source=source,lines=len(lines),covered=sum(n>0 for n in lines.values()),
                         inactive=not lines,functions=functions,uncovered=[n for n,c in lines.items() if not c]))
    result=dict(files=rows,lines=sum(r["lines"] for r in rows),covered=sum(r["covered"] for r in rows))
    (out/"coverage.json").write_text(json.dumps(result,indent=2)+"\n")
    for r in rows: print(Path(r['source']).name,r['covered'],r['lines'],"inactive" if r['inactive'] else "")
    print("TOTAL",result['covered'],result['lines'])
