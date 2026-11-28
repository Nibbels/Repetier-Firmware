"""Replay case fan requests against the firmware's 32-bit uptime clock."""

import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def main():
    tests = Path(__file__).resolve().parent
    firmware = tests.parent / "Repetier"
    fan = (firmware / "CaseFan.cpp").read_text(encoding="utf-8")
    commands = (firmware / "GCodeThermal.cpp").read_text(encoding="utf-8")
    fan = fan[fan.index("namespace CaseFan {"):fan.rindex("#endif")]
    command_start = commands.index("void turnOnCaseFan()")
    command_end = commands.index("#endif", command_start)
    with TemporaryDirectory(prefix="rfx-case-fan-") as temp:
        build = Path(temp)
        (build / "case-fan-firmware.inc").write_text(fan + commands[command_start:command_end], encoding="utf-8")
        executable = build / "case-fan"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++11", "-Wall", "-Wextra", "-Werror", "-O2",
            "-I", str(build), str(tests / "case-fan.cpp"), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True, timeout=10)
        print("Case fan rollover and timer requests passed", flush=True)


if __name__ == "__main__":
    main()
