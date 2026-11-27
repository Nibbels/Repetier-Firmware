#!/usr/bin/env python3
"""Actual-source Z ledger and independent DIR ownership counterfactual.
No production edits. Host GPIO counts are not mechanical/timing proof.
"""
import pathlib,tempfile,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[2]
HERE=pathlib.Path(__file__).resolve().parent
FW=ROOT/"Repetier"

def patch(header,source):
    a="bool stepPath(Path path);"
    assert header.count(a)==1
    header=header.replace(a,"bool stepPath(Path path, int8_t direction);")
    a="bool ZMotion::stepPath(Path path) {\n    const int8_t direction = directionIsPositive() ? 1 : -1;"
    assert source.count(a)==1
    source=source.replace(a,"bool ZMotion::stepPath(Path path, int8_t direction) {")
    a="""        if (result == Step::Pulse)
            startPulse(direction);
        HAL::forbidInterrupts();"""
    assert source.count(a)==1
    source=source.replace(a,"""        HAL::forbidInterrupts();
        if (result == Step::Pulse) {
            if (directionIsPositive() != (direction > 0)) {
                setDirection(direction > 0);
#ifdef __AVR__
                __builtin_avr_delay_cycles(2);
#endif
            }
            startPulse(direction);
        }""")
    return header,source

def main():
    source=(FW/"ZMotion.cpp").read_text().removeprefix('#include "Repetier.h"\n')
    header=(FW/"ZMotion.h").read_text()
    with tempfile.TemporaryDirectory(prefix="z-hop-") as tmp:
        d=pathlib.Path(tmp)
        for candidate in (False,True):
            h,s=patch(header,source) if candidate else (header,source)
            (d/"ZMotion.h").write_text(h)
            (d/"actual-z-motion.inc").write_text(s)
            for board in ("RF1000","RF2000","RF2000v2"):
                exe=d/"replay"
                subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
                    "-I",str(d),"-I",str(FW),"-DMOTHERBOARD=DEVICE_TYPE_"+board,
                    "-DEXPLICIT_DIR="+str(int(candidate)),str(HERE/"z-hop-reversibility.cpp"),"-o",str(exe)],check=True)
                subprocess.run([str(exe)],check=True)
                print("z-hop",board,"explicit" if candidate else "legacy",flush=True)
if __name__=="__main__":main()
