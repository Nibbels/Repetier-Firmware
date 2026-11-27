#!/usr/bin/env python3
"""Read-only baseline characterization, not desired M908 semantics.
Runs with canonical CI's automatic tests/*.py discovery. No hardware."""
import ast
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
# Reuse the canonical actual-driver host stub without executing/editing its test.
tree = ast.parse((ROOT / "tests/drv8711-trace.py").read_text())
stub = next(ast.literal_eval(n.value) for n in tree.body
            if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "STUB" for t in n.targets))
source = (ROOT / "Repetier/GCodeMotion.cpp").read_text()
start = source.index("void setStepperCurrent(GCode* com)")
end = source.index("\n}", start) + 2
function = source[start:end]
harness = r'''
#include "Drv8711.cpp"
#include <climits>
struct GCode {
    int32_t S=110, P=0;
    float E=0;
    unsigned flags=0;
    bool hasS() { return flags&1; }
    bool hasP() { return flags&2; }
    bool hasX() { return flags&4; }
    bool hasY() { return flags&8; }
    bool hasZ() { return flags&16; }
    bool hasE() { return flags&32; }
};
#define MOTOR_CURRENT_MIN 32
''' + function + r'''
static void check(unsigned flags, int32_t s, int32_t p, float e,
                  int selected, int word) {
    actual.clear();
    GCode c; c.flags=flags; c.S=s; c.P=p; c.E=e;
    setStepperCurrent(&c);
    int cs=0, rises=0, bit=0, shifted=0, masked=0, restored=0;
    for (auto v: actual) {
        if (v[0]==3) ++masked;
        if (v[0]==4) ++restored;
        if (v[0]!=0) continue;
        if (v[1]>=1 && v[1]<=5 && v[2]) {
            assert(!cs); cs=v[1];
        }
        if (v[1]==DRV_SDATI) bit=v[2];
        if (v[1]==DRV_SCLK && v[2]) { ++rises; shifted=(shifted<<1)|bit; }
    }
    assert(cs==selected);
    assert(rises==(word<0 ? 0 : 16));
    assert(masked==(word<0 ? 0 : 1) && restored==masked);
    if (word>=0) assert(shifted==word);
    for (uint8_t a=0;a<5;++a) assert(Drv8711::configuredCurrent(a)==80+a);
}
int main() {
    for (uint8_t a=0;a<5;++a) Drv8711::stageCurrent(a,80+a);
    check(1|4,110,0,0,0,0x116e); // X: no select
    check(1|8,110,0,0,1,0x116e); // Y: physical X
    check(1|16,110,0,0,2,0x116e); // Z: physical Y
    check(1|32,110,0,0,4,0x116e); // E0
    check(1|32,110,0,1,NUM_EXTRUDER==2?5:0,NUM_EXTRUDER==2?0x116e:-1);
    check(1|2,110,0,0,0,0x116e); // P0: clocks without select
    for (int p=1;p<=3+NUM_EXTRUDER;++p)
        check(1|2,150,p,0,p,0x1196); // global limit accepts above per-axis limits
    check(1|2,150,3+NUM_EXTRUDER+1,0,0,-1);
    check(1|2,151,1,0,0,-1);
    check(1|2,31,1,0,0,-1);
    check(1|2,32,1,0,1,0x1120);
    check(1|2,366,257,0,1,0x116e); // wraps before validation
    check(1|2,-146,-255,0,1,0x116e);
    check(1|2,INT32_MIN+110,INT32_MIN+1,0,1,0x116e);
    check(1|2|4|8|16|32,110,3,0,4,0x116e); // last selector wins
    check(1|32,110,0,2,0,-1);
    check(2,110,1,0,0,-1);
    std::printf("M908 baseline counterexamples confirmed (tools=%d)\n", NUM_EXTRUDER);
}
'''
with tempfile.TemporaryDirectory() as tmp:
    work = Path(tmp)
    (work / "Repetier.h").write_text(stub)
    (work / "Drv8711.cpp").write_bytes((ROOT / "Repetier/Drv8711.cpp").read_bytes())
    (work / "replay.cpp").write_text(harness)
    for n in (1, 2):
        exe = work / "replay"
        subprocess.run([os.environ.get("CXX","c++"), "-std=c++11","-O2","-fsigned-char",
                        f"-DNUM_EXTRUDER={n}", str(work/"replay.cpp"), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)

def screen(period, blackout, initial, read_latency=31):
    """Conservative completed-byte FIFO of depth 2, no shift register slack.
    No competing IRQs; arrival at read boundary happens first.
    Isolated blackout followed by first UDR read, not full AVR DOR model."""
    losses = []
    for phase in range(period):
        arrivals = 0 if phase > blackout+read_latency else 1+(blackout+read_latency-phase)//period
        losses.append(max(0, initial+arrivals-2))
    return dict(period=period, blackout=blackout, initial=initial,
                phases=len(losses), loss_phases=sum(x>0 for x in losses), max_lost=max(losses))

assert screen(640, 0, 0)["loss_phases"] == 0
assert screen(320, 1536, 0)["loss_phases"] == 320
assert screen(640, 1536, 0)["max_lost"] == 1
rows = [screen(p,b,i) for p in (640,320) for b in (0,32,80,160,320,594,1536,2048)
        for i in (0,1)]
print("DRV8711 isolated blackout sensitivity:", json.dumps(rows, separators=(",",":")))
