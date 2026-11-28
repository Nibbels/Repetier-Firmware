#!/usr/bin/env python3
"""Exercise unsupported legacy panel command against the real M dispatcher."""
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory

fw = Path(__file__).resolve().parents[1] / "Repetier"
source = (fw / "GCodeCommands.cpp").read_text()
start = source.index("void dispatchMachine(GCode* com)")
dispatch = source[start:source.index("} // namespace", start)]
# There must be no special branch or legacy helper for the retired command.
assert not re.search(r"\b3029\b|\baccessPanel\b", dispatch)
for file in ("GCodeSystem.cpp", "GCodeCommands.h", "ui.h", "ui.cpp"):
    assert not re.search(r"\b(?:accessPanel|executeRemoteAction|cachedRow|requiresLocalSafetyConfirmation)\b", (fw / file).read_text()), file
handlers = sorted(set(re.findall(r"return (\w+(?:::\w+)?)\(", dispatch)))
# Replace only the handler bodies, not the switch or its control flow.
stubs = []
for name in handlers:
    parts = name.split("::")
    body = "template<typename... T> void " + parts[-1] + "(T...) { ++calls; }"
    stubs.append("namespace " + parts[0] + " { " + body + " }" if len(parts) == 2 else body)
harness = r'''
#include <cassert>
#define MOTHERBOARD 1
#define DEVICE_TYPE_RF1000 1
#define DEVICE_TYPE_RF2000 2
#define DEVICE_TYPE_RF2000v2 3
#define CASE_FAN_PIN -1
struct GCode { unsigned M; };
static unsigned calls = 0;
constexpr unsigned PanelMessage = 117;
namespace DiagnosticHistory { constexpr unsigned ReadCommand = 3950; }
namespace Printer {
    struct StartLine { void run(GCode&) { ++calls; } } startLine;
}
'''
main = r'''
int main() {
    GCode com{3029};
    dispatchMachine(&com);
    assert(calls == 0);
    com.M = 65535; // ordinary unsupported M-code follows the same path
    dispatchMachine(&com);
    assert(calls == 0);
    com.M = 117;
    dispatchMachine(&com);
    assert(calls == 1);
    com.M = 115;
    dispatchMachine(&com);
    assert(calls == 2);
}
'''
with TemporaryDirectory() as tmp:
    cpp = Path(tmp) / "dispatch.cpp"
    cpp.write_text(harness + "\n".join(stubs) + "\n" + dispatch + main)
    exe = Path(tmp) / "dispatch"
    subprocess.run(["c++", "-std=c++11", "-Wall", "-Wextra", "-Werror", str(cpp), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("Retired M3029 follows ordinary unsupported M-code dispatch; M117/M115 retained: PASS")
