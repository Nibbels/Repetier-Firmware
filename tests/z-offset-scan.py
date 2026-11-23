"""Run the firmware's Z-offset scan against deterministic probe and EEPROM inputs."""

import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def main():
    tests = Path(__file__).resolve().parent
    source = (tests.parent / "Repetier/RF.cpp").read_text(encoding="utf-8")
    start = source.index("namespace {\n\nclass ZOffsetScan")
    end = source.index("\nvoid startZOScan(", start)
    with TemporaryDirectory(prefix="rfx-z-offset-scan-") as temp:
        build = Path(temp)
        (build / "z-offset-scan-firmware.inc").write_text(source[start:end], encoding="utf-8")
        executable = build / "scan"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(build), str(tests / "z-offset-scan.cpp"), "-o", str(executable),
        ], check=True)
        for scenario in ("normal", "retry", "missing-matrix", "level"):
            subprocess.run([str(executable), scenario], check=True, timeout=10)


if __name__ == "__main__":
    main()
