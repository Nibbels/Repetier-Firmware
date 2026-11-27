"""Exercise actual guarded feature/scan dispatch on nested periodic calls."""
import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parents[1]
rf = (ROOT / "Repetier/RF.cpp").read_text()
def section(start, end):
    i = rf.index(start)
    return rf[i:rf.index(end, i)]
scan = section("static __attribute__((noinline)) void dispatchScanWork", "\nvoid handleStartStandby")
feature = section("static __attribute__((noinline)) void servicePrintFeatures", "\nvoid queueTask")
harness = r"""
#include <cassert>
#include <cstdint>
#include <vector>
#include "CalibrationScan.h"
using millis_t = uint32_t;
std::vector<int> events;
bool scanRunning = true, bed = true, stop = false, paused = false;
bool ready = false, nested = false, cancelDuring = false;
void handleScanWorkTasks(bool);
void loopFeatures();
namespace HAL { millis_t timeInMilliseconds() { return 123; } }
namespace Calibration {
ScanSession scanSession;
struct Pressure {
    bool service(millis_t t) { assert(t == 123); events.push_back(1); return ready; }
} scanPressure;
void tickHeatBedScan() {
    events.push_back(2);
    assert(scanSession.dispatching());
    if (nested) handleScanWorkTasks(true);
    if (cancelDuring) assert(scanSession.deferDiscard());
}
void tickZOffsetScan() { events.push_back(3); }
}
bool isAnyScanRunning() { return scanRunning; }
bool heatBedScanRunning() { return bed; }
void discardScans() { assert(!Calibration::scanSession.dispatching()); events.push_back(4); }
namespace Printer {
bool blockAll = false;
struct Job {
    bool commandsBlocked() { return stop; }
    void service(millis_t t) {
        assert(t == 123);
        events.push_back(5);
        if (nested) loopFeatures();
    }
} printJob;
struct Pause {
    bool isPaused() { return paused; }
    void serviceStandby(millis_t t) { assert(t == 123); events.push_back(8); }
    void serviceMotion() { events.push_back(9); }
} pause;
}
void handleStartStandby(millis_t t) { assert(t == 123); events.push_back(6); }
void handleStrainGaugeFeatures(millis_t t) { assert(t == 123); events.push_back(7); }
"""
main = r"""
int main() {
    nested = true;
    handleScanWorkTasks(true);
    assert((events == std::vector<int>{1,2}));
    events.clear(); cancelDuring = true;
    handleScanWorkTasks(true);
    assert((events == std::vector<int>{1,2,4}));
    events.clear(); cancelDuring = false;
    bed = false; ready = true;
    handleScanWorkTasks(false);
    assert((events == std::vector<int>{1,3}));
    events.clear(); ready = false;
    handleScanWorkTasks(false);
    assert((events == std::vector<int>{1}));
    events.clear(); paused = true;
    handleScanWorkTasks(true); assert(events.empty());
    paused = false; stop = true;
    handleScanWorkTasks(true); assert((events == std::vector<int>{4}));
    events.clear(); stop = false; scanRunning = false;
    handleScanWorkTasks(true); assert(events.empty());
    loopFeatures();
    assert((events == std::vector<int>{5,9,6,7,8,9}));
    events.clear(); nested = false;
    loopFeatures();
    assert((events == std::vector<int>{5,6,7,8,9}));
}
"""
with TemporaryDirectory() as tmp:
    source = Path(tmp) / "dispatch.cpp"
    source.write_text(harness + scan + feature + main)
    executable = Path(tmp) / "dispatch"
    subprocess.run([os.environ.get("CXX","c++"), "-std=c++11", "-O2",
                    "-Wall", "-Wextra", "-Werror", "-I", str(ROOT/"Repetier"),
                    str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True, timeout=10)
print("Actual scan/feature dispatch: nested admission, deferred discard, pause and ordering PASS")
