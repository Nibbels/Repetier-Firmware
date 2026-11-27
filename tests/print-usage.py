"""Exercise the shared counters before and after saving a completed heat cycle."""

import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def main():
    tests = Path(__file__).resolve().parent
    firmware = tests.parent / "Repetier"
    owner = (firmware / "PrintUsage.cpp").read_text(encoding="utf-8")
    storage = (firmware / "Eeprom.cpp").read_text(encoding="utf-8")
    with TemporaryDirectory(prefix="rfx-print-usage-") as temp:
        build = Path(temp)
        (build / "print-usage-firmware.inc").write_text(owner[owner.index("namespace {"):], encoding="utf-8")
        (build / "usage-storage-firmware.inc").write_text(storage[storage.index("EEPROM::UsageTotals EEPROM::usageTotals()"):storage.index("void EEPROM::initBaudrate()")], encoding="utf-8")
        executable = build / "print-usage"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(firmware), "-I", str(build), str(tests / "print-usage.cpp"), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True, timeout=10)
        print("Print usage replay passed", flush=True)


if __name__ == "__main__":
    main()
