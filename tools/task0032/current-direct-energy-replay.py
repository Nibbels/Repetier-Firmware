#!/usr/bin/env python3
"""Compile actual candidate motion, ZMotion and executor in the existing GPIO fixture."""
import argparse
from pathlib import Path
import subprocess
import tempfile

HERE=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True,help='Exact prior Repetier directory')
    p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();fw=a.source.resolve();candidate=a.candidate.resolve();out=a.out.resolve()
    out.mkdir(parents=True,exist_ok=True)
    z=(fw/'ZMotion.cpp').read_text();e=(candidate/'StepExecutor.cpp').read_text();m=(candidate/'motion.cpp').read_text()
    (out/'actual.inc').write_text(z.removeprefix('#include "Repetier.h"\n')+'\n'+e[e.index('template <int8_t limitDirection>'):]+'\n'+
        m[m.index('void PrintLine::stopDirectMove()'):m.index('/** \\brief Check if move is new.')])
    base=(HERE/'current-stop-tail-replay.cpp').read_text()
    base=base[:base.index('unsigned runStop(')]
    base=base.replace('constexpr int32_t upper=1000;','constexpr int32_t upper=10000;')
    base=base.replace('z.configureTravel(100,10);','z.configureTravel(100,100);')
    base=base.replace('uint16_t ComputeV(uint32_t,uint32_t){return 0;}','''uint16_t ComputeV(uint32_t,uint32_t){return 0;}
uint16_t integerSqrt(int32_t x){assert(interruptsEnabled && x>0);maybeStop();return uint16_t(sqrt(double(x))+0.5);}''')
    base=base.replace('static int32_t xAt=0;','''static int32_t xAt=0;
static uint8_t primaryAxis=0;
static int32_t primaryAt=0;
static uint32_t energyAt=0;
static unsigned maxTail=0;''')
    base=base.replace('pulsesAt=pulses;xAt=physical[X_AXIS];','pulsesAt=pulses;xAt=physical[X_AXIS];primaryAt=physical[primaryAxis];energyAt=StepExecutor::stepClocks[FOR_DIRECT].ticks;')
    # The fixture includes the actual class header. Candidate include path
    # must precede prior firmware; no source copied over production.
    cpp=out/'direct-energy-replay.cpp';cpp.write_text(base+(HERE/'current-direct-energy-cases.inc').read_text())
    boards={"RF1000":["DEVICE_TYPE_RF1000"],"RF2000":["DEVICE_TYPE_RF2000"],
            "RF2000v2":["DEVICE_TYPE_RF2000v2"],
            "RF2000-X44":["DEVICE_TYPE_RF2000","NUM_EXTRUDER=1","EXT0_EXTRUDER_COOLER_PIN=X44_PIN"]}
    with tempfile.TemporaryDirectory() as d:
        exe=Path(d)/'replay'
        for board,defs in boards.items():
            subprocess.run(['c++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-fsanitize=undefined',
                '-fno-sanitize-recover=all','-I',str(candidate),'-I',str(fw),'-I',str(out),'-DMOTHERBOARD='+defs[0],
                *['-D'+flag for flag in defs[1:]],str(cpp),'-o',str(exe)],check=True)
            log=subprocess.check_output([str(exe)],text=True,timeout=90)
            print(board,log,end='',flush=True);(out/(board+'.log')).write_text(log)

if __name__=='__main__':main()
