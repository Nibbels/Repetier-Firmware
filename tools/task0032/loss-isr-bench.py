#!/usr/bin/env python3
"""Measure isolated sticky-gate ISR cost using the original 0033 path walker."""
from pathlib import Path
import argparse
import hashlib
import json
import runpy
import subprocess
import shutil
from tempfile import TemporaryDirectory

ROOT=Path(__file__).resolve().parents[2]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--prior-isr",required=True,type=Path)
    p.add_argument("--compiler",required=True,type=Path)
    p.add_argument("--out",required=True,type=Path)
    a=p.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
    prior=runpy.run_path(str(a.prior_isr))
    gate=runpy.run_path(str(Path(__file__).with_name("loss-gate-replay.py")))
    cc=a.compiler.resolve()
    run=prior["run"]
    flags=["-mmcu=atmega2560","-DF_CPU=16000000UL","-Os","-flto","-std=gnu++11",
           "-ffunction-sections","-fdata-sections","-fno-exceptions","-fno-threadsafe-statics"]
    report={"schema":1,"qualification":"isolated linked structural paths, branch feasibility not solved, no concurrent ISR or final P0 layout","flags":flags,
            "compiler":run([cc,"--version"]).splitlines()[0],"variants":{}}
    with TemporaryDirectory(prefix="task0032-isr-") as directory:
        root=Path(directory)
        for name in ("UsbSerial.cpp","UsbSerial.h","GCodeInput.cpp"):
            shutil.copyfile(ROOT/"Repetier"/name,root/name)
        gate["patch"](root)
        uart=(root/"UsbSerial.cpp").read_text()
        uart=prior["replace_once"](uart,prior["BASE_TX"],prior["CACHED_TX"])
        uart=prior["replace_once"](uart,"inline void UsbSerial::receiveInterrupt()",
              "inline __attribute__((always_inline)) void UsbSerial::receiveInterrupt()")
        # A second minimal form combines the already-sticky gate and new flag.
        alternate=prior["replace_once"](uart,
            "    if (error) {\n        lossFlags_ |= error;\n        return;\n    }\n    if (lossFlags_) return;",
            "    const uint8_t loss = lossFlags_ | error;\n    if (loss) {\n        lossFlags_ = loss;\n        return;\n    }")
        (root/"main.cpp").write_text(prior["MAIN"].replace("500000","250000"))
        for name,src in (("sticky",uart),("combined",alternate)):
            (root/"UsbSerial.cpp").write_text(src)
            elf=out/(name+".elf")
            run([cc,*flags,root/"UsbSerial.cpp",root/"main.cpp","-Wl,--gc-sections","-o",elf])
            dis=run([cc.with_name("avr-objdump"),"-d","-z",elf])
            (out/(name+".dis")).write_text(dis)
            rx=prior["path_set"](dis,25);tx=prior["path_set"](dis,26)
            assert all(len(p["read_complete"])==1 and p["status_reads"][0]<p["read_complete"][0] for p in rx)
            info={"rx":prior["summarize"](rx),"tx":prior["summarize"](tx),
                  "elf_sha256":hashlib.sha256(elf.read_bytes()).hexdigest(),
                  "size":run([cc.with_name("avr-size"),elf]).strip(),
                  "source_sha256":hashlib.sha256(src.encode()).hexdigest()}
            (out/(name+"-paths.json")).write_text(json.dumps({"rx":rx,"tx":tx},indent=2))
            report["variants"][name]=info
            print(name,json.dumps(info,separators=(",",":")))
    (out/"summary.json").write_text(json.dumps(report,indent=2)+"\n")

if __name__=="__main__":
    main()
