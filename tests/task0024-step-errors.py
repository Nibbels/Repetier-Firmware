"""Compare actual executor Bresenham state with the former per-slot algorithm."""
import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parents[1]
header = (ROOT / "Repetier/StepExecutor.h").read_text()
executor = (ROOT / "Repetier/StepExecutor.cpp").read_text()
planner = (ROOT / "Repetier/MotionPlanner.cpp").read_text()
move = (ROOT / "Repetier/motion.h").read_text()

start = header.index("    struct StepErrors {")
end = header.index("    enum class DirectionOwner", start)
state = header[start:end]
assert "error[4]" not in move
assert "move.error[" not in planner
assert executor.count("stepErrors[FOR_QUEUE].start(queuedPrimarySteps);") == 1
assert executor.count("stepErrors[FOR_DIRECT].start(directPrimarySteps);") == 1
assert executor.count("auto& errors = stepErrors[forQueue];") == 1
assert "template<uint8_t forQueue>\nlong StepExecutor::performMove(PrintLine* move)" in executor
assert executor.count("performMove<FOR_QUEUE>(move)") == 1
assert executor.count("performMove<FOR_DIRECT>(&direct)") == 1
assert executor.count("errors.advance(") == 4
assert executor.count("errors.account(") == 8
# The actual activation sites must occur before the first-step half interval,
# not on the direction-owner switch/resume path.
for source, target in (("FOR_QUEUE", "queued"), ("FOR_DIRECT", "direct")):
    init = executor.index(f"stepErrors[{source}].start({target}PrimarySteps);")
    assert init < executor.index("return (stepIntervalTicks >> 1);", init)
    assert init < executor.index("if (directionOwner !=", init)

program = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <climits>
''' + state + r'''
struct Reference {
    int32_t value[4];
    void reset(int32_t primary) {
        value[0] = value[1] = value[2] = primary >> 1;
        value[3] = 0;
    }
};
struct Path {
    int32_t primary;
    int32_t delta[4];
    bool active[4];
    Reference reference;
    void start(StepErrors& actual, int32_t p, const int32_t* d) {
        primary = p;
        actual.start(p);
        reference.reset(p);
        for (unsigned a = 0; a < 4; ++a) {
            delta[a] = d[a];
            active[a] = d[a] != 0;
            assert(actual.value[a] == reference.value[a]);
        }
    }
    void tick(StepErrors& actual, bool direct, unsigned stoppedAxis = 4) {
        for (unsigned a = 0; a < 4; ++a) {
            if (!active[a]) continue;
            bool expected = (reference.value[a] -= delta[a]) < 0;
            bool emitted = actual.advance(a, delta[a]);
            assert(emitted == expected);
            if (emitted) {
                // E always accounts the logical step even if output is inhibited.
                // An XY/Z direct limit can instead clear the axis before accounting.
                if (direct && a != 3 && a == stoppedAxis) active[a] = false;
                else {
                    reference.value[a] += primary;
                    actual.account(a, primary);
                }
            }
            assert(actual.value[a] == reference.value[a]);
        }
    }
};
int main() {
    StepErrors actual[2] = {};
    Path path[2] = {};
    const int32_t queued[4] = {97, 31, 13, 71};
    const int32_t direct[4] = {5, 17, 7, 9};
    path[1].start(actual[1], 97, queued);
    for (int i = 0; i < 19; ++i) path[1].tick(actual[1], false);
    path[0].start(actual[0], 17, direct);
    for (int i = 0; i < 17; ++i) {
        path[0].tick(actual[0], true, 1);
        path[1].tick(actual[1], false);
    }
    // Direct reuse cannot reset a suspended queued path, and queue wrap/reuse
    // resets a new queued phase rather than carrying the predecessor's errors.
    path[0].start(actual[0], 17, direct);
    for (int i = 0; i < 61; ++i) path[1].tick(actual[1], false);
    for (int slot = 0; slot < 72; ++slot) {
        int32_t p = slot + 3;
        const int32_t d[4] = {p, p/2, p/3, p-1};
        path[1].start(actual[1], p, d);
        for (int i = 0; i < p; ++i) {
            path[1].tick(actual[1], false);
            if (i % 3 == 0) path[0].tick(actual[0], true);
        }
    }
    // Full-width counts, including primary/secondary ties and zero E phase.
    const int32_t wide[4] = {INT32_MAX, INT32_MAX-1, 1, INT32_MAX/2};
    path[1].start(actual[1], INT32_MAX, wide);
    for (int i = 0; i < 10000; ++i) path[1].tick(actual[1], false);
    std::puts("Task0024 step errors: PASS");
}
'''
with TemporaryDirectory(prefix="rfx-step-errors-") as temp:
    root = Path(temp)
    source = root / "replay.cpp"
    source.write_text(program)
    exe = root / "replay"
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-O2",
                    "-Wall", "-Wextra", "-Werror", "-fsanitize=undefined",
                    "-fno-sanitize-recover=all", str(source), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=10)
