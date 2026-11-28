#!/usr/bin/env python3
"""Task-private actual PrintPause replay using canonical source extraction.
Only temporary copies of the shared replay are instrumented. No source edits."""
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
def once(text, old, new):
    assert text.count(old)==1, old
    return text.replace(old,new,1)

cpp=(ROOT/"tests/pause-sense-offset.cpp").read_text()
cpp=once(cpp, """void setExtruderCurrent(int, int) {}
namespace Drv8711 { uint8_t configuredCurrent(uint8_t) { return 0; } }""", r'''
static uint8_t normalCurrent[5]={110,110,105,90,95};
static uint8_t effectiveCurrent[NUM_EXTRUDER]={};
static unsigned currentWrites=0;
void setExtruderCurrent(int i, int v) {
    assert(i>=0 && i<NUM_EXTRUDER && v>=0 && v<=255);
    effectiveCurrent[i]=static_cast<uint8_t>(v);
    ++currentWrites;
}
namespace Drv8711 { uint8_t configuredCurrent(uint8_t a) {
    assert(a<5); return normalCurrent[a];
} }
''')
scenario=r'''
void motorCurrentLifecycle() {
    pollDuringMoves=false;
    for (int i=0;i<NUM_EXTRUDER;++i) {
        effectiveCurrent[i]=normalCurrent[E_AXIS+i];
        extruder[i].tempControl.targetTemperatureC=200;
    }
    Printer::pause.request(PrintPause::Destination::InPlace);
    Printer::pause.serviceMotion();
    assert(Printer::pause.isWarmPause());
    const uint32_t due=clockMs+EXTRUDER_CURRENT_PAUSE_DELAY;
    Printer::pause.serviceStandby(due);
    assert(Printer::pause.isWarmPause() && currentWrites==0);
    clockMs=due+1;
    Printer::pause.serviceStandby(clockMs);
    assert(Printer::pause.isSettled() && !Printer::pause.isWarmPause());
    assert(currentWrites==NUM_EXTRUDER);
    for (int i=0;i<NUM_EXTRUDER;++i) {
        assert(effectiveCurrent[i]==32);
        assert(normalCurrent[E_AXIS+i]==90+5*i);
    }
    Printer::pause.serviceStandby(clockMs+1);
    assert(currentWrites==NUM_EXTRUDER);
    // Model a configuration change staged by its owner while paused.
    // Deliberately do not claim this is the current UI apply behavior.
    normalCurrent[E_AXIS]=100;
    duringHeatWait=[]() {
        for(int i=0;i<NUM_EXTRUDER;++i)
            assert(effectiveCurrent[i]==normalCurrent[E_AXIS+i]);
    };
    assert(Printer::pause.resume());
    duringHeatWait=nullptr;
    assert(Printer::pause.isWarmPause());
    assert(currentWrites==2*NUM_EXTRUDER);
    for(int i=0;i<NUM_EXTRUDER;++i)
        assert(effectiveCurrent[i]==normalCurrent[E_AXIS+i]);
    assert(Printer::pause.resume() && Printer::pause.allowsQueueAdvance());
    for(int i=0;i<NUM_EXTRUDER;++i)
        assert(effectiveCurrent[i]==normalCurrent[E_AXIS+i]);
}
'''
cpp=once(cpp,"int main(int argc, char** argv) {",scenario+"\nint main(int argc, char** argv) {")
cpp=once(cpp,'    if (!std::strcmp(argv[1], "park"))',
         '    if (!std::strcmp(argv[1], "motor-current"))\n        motorCurrentLifecycle();\n    else if (!std::strcmp(argv[1], "park"))')
runner=(ROOT/"tests/pause-sense-offset.py").read_text()
# Retain compilation and real firmware extraction; replace just scenario lists.
runner,n=re.subn(r"for scenario in \([\s\S]*?\):\n            subprocess\.run",
                 'for scenario in ("motor-current",):\n            subprocess.run',runner)
assert n==2
with tempfile.TemporaryDirectory() as tmp:
    work=Path(tmp)
    (work/"Repetier").symlink_to(ROOT/"Repetier",target_is_directory=True)
    (work/"tests").mkdir()
    (work/"tests/pause-sense-offset.cpp").write_text(cpp)
    script=work/"tests/pause-sense-offset.py"
    script.write_text(runner)
    subprocess.run([sys.executable,str(script)],check=True)
