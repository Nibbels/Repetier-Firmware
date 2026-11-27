#!/usr/bin/env python3
"""Independent direct-period variant of the green unified-Z candidate."""
import argparse,difflib,importlib.util,pathlib,tempfile,zipfile,re
HERE=pathlib.Path(__file__).resolve().parent
def load(name,file):
    spec=importlib.util.spec_from_file_location(name,HERE/file)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
u=load("unified","unified-z-source.py")
ir=load("inverse_root","inverse-root-research.py")
q=u.q

def modify(dst,tag):
    assert tag in ("direct-z","direct-z-merge-study")
    u.modify(dst,"unified-z")
    (dst/"inverse-root-generated.h").write_text(ir.header())
    (dst/"z-correction-clock.h").write_text((HERE/"z-correction-clock.h").read_text())
    (dst/"direct-z-ramp.h").write_text((HERE/"direct-z-ramp.h").read_text())
    p=dst/"StepExecutor.cpp";s=p.read_text()
    s=q.replace(s,'#include "ZCorrectionResearch.h"','#include "ZCorrectionResearch.h"\n#include "direct-z-ramp.h"')
    start=s.index("// Reuse the already linked RFx000 math")
    end=s.index("    int32_t armedError",start)
    s=s[:start]+"""struct CorrectionState {
    DirectZResearch::Ramp ramp;
    DirectZResearch::Limits limits{};
    DirectZResearch::Pulse armed{};
"""+s[end:]
    s=q.replace(s,'sizeof(CorrectionState)==41','sizeof(CorrectionState)==43')
    s=q.replace(s,'ZCorrectionResearch::Limits::prepare(acceleration,speed)','DirectZResearch::Limits::prepare(acceleration,speed)')
    p.write_text(s)
    if tag=="direct-z-merge-study":
        # The threshold is mutable ONLY to sweep one identical linked
        # image. It is not a proposed operator setting or production policy.
        p=dst/"HAL.cpp";s=p.read_text()
        s=q.replace(s,"static uint32_t nominalDeadline=0;",
            "static uint32_t nominalDeadline=0;\nstatic volatile uint16_t correctionGapResearch=512;")
        s=q.replace(s,"next-z>=StepExecutor::minimumStepIntervalTicks","next-z>=correctionGapResearch")
        p.write_text(s)
    diff=[]
    for p in sorted(dst.glob("*")):
        if not p.is_file():continue
        old=(q.FW/p.name).read_bytes() if (q.FW/p.name).is_file() else b""
        new=p.read_bytes()
        if old!=new:
            diff.extend(difflib.unified_diff(old.decode().splitlines(True),new.decode().splitlines(True),fromfile="a/Repetier/"+p.name,tofile="b/Repetier/"+p.name))
    return "".join(diff)

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--out",type=pathlib.Path,required=True)
    p.add_argument("--tools",type=pathlib.Path,required=True)
    p.add_argument("--merge-study",action="store_true")
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    q.modify=modify
    with tempfile.TemporaryDirectory(prefix="direct-z-") as tmp:
        archive=q.build(pathlib.Path(tmp)/"Repetier",a.out,"direct-z-merge-study" if a.merge_study else "direct-z",a.tools)
    # LPM near table access must be proven in the actual link, not inferred
    # from total image size (the ATmega2560 has more than 64 KiB flash).
    with zipfile.ZipFile(archive) as z:
        for b in q.FLAGS:
            rows=z.read(b+"/symbols.txt").decode().splitlines()
            matches=[r for r in rows if "InverseRootResearch" in r and "periods" in r]
            assert len(matches)==1,(b,matches)
            addr,size,*_=matches[0].split()
            assert int(size,16)==98 and int(addr,16)+int(size,16)<=65536,(b,matches)
            print("direct-z near flash",b,matches[0],flush=True)

if __name__=="__main__":main()
