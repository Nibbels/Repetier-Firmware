"""Drive the AVR TWI transaction against simulated peripheral events."""

import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def main():
    tests = Path(__file__).resolve().parent
    firmware = tests.parent / "Repetier"
    source = (firmware / "TwiTransfer.cpp").read_text(encoding="utf-8")
    with TemporaryDirectory(prefix="rfx-twi-") as temp:
        build = Path(temp)
        (build / "twi-transfer-firmware.inc").write_text(
            source[source.index("namespace {"):], encoding="utf-8",
        )
        executable = build / "twi-transfer"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(build), "-I", str(firmware), str(tests / "twi-transfer.cpp"), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True, timeout=10)


if __name__ == "__main__":
    main()
