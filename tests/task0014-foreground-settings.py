#!/usr/bin/env python3
"""Replay the actual private-state accessors and coupled foreground updates."""
import pathlib
import subprocess
import tempfile

fw=pathlib.Path(__file__).resolve().parents[1]/"Repetier"
printer=(fw/"Printer.h").read_text()
start=printer.index("    static float requestedFeedrate()")
end=printer.index("    static void markTemperatureReadingsAvailable()",start)
owner=printer[start:end]
source=r"""
#include <cassert>
#include <cmath>
#include <cstdint>
class Printer {
    static float feedrate, menuExtrusionFactor;
    static int feedrateMultiply;
    static uint8_t debugLevel;
    static char ZMode;
public:
"""+owner+r"""
};
float Printer::feedrate=10, Printer::menuExtrusionFactor=1;
int Printer::feedrateMultiply=100;
uint8_t Printer::debugLevel=6;
char Printer::ZMode=1;
int main() {
    float expected=10;
    int previous=100;
    for(int multiplier : {25, 100, 500, 101, 73, 100}) {
        expected *= (float)multiplier / (float)previous;
        Printer::setFeedrateMultiplier(multiplier);
        assert(Printer::requestedFeedrate()==expected);
        assert(Printer::feedrateMultiplier()==multiplier);
        previous=multiplier;
    }
    for(float v : {0.1f, 0.93f, 1.0f, 2.0f}) {
        Printer::setFlowMultiplier(v);
        assert(Printer::flowMultiplier()==v);
    }
    for(int i=0;i<256;++i) {
        for(uint8_t bit : {1,2,4,8}) {
            Printer::setDebugMask(i);
            Printer::toggleDebugFlag(bit);
            int expected = (i & ~8) ^ (bit & ~8);
            assert(Printer::debugMask()==expected);
            Printer::toggleDebugFlag(bit);
            assert(Printer::debugMask()==(i & ~8));
        }
        Printer::setDebugMask(i);
        assert(Printer::debugMask()==(i & ~8));
        Printer::restoreZDisplayMode(static_cast<char>(i));
        assert(static_cast<uint8_t>(Printer::zDisplayMode())==i);
        Printer::toggleZDisplayMode();
        assert(Printer::zDisplayMode()==(i==1?2:1));
    }
}
"""
source=source.replace("#include <cmath>","#include <cmath>\n#include <initializer_list>")
with tempfile.TemporaryDirectory() as tmp:
    p=pathlib.Path(tmp)/"test.cpp"
    p.write_text(source)
    exe=pathlib.Path(tmp)/"test"
    subprocess.run(["g++","-std=c++11","-Wall","-Wextra","-Werror",str(p),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print("Coupled feedrate, flow, all 256 debug masks and all 256 Z display bytes passed.")
