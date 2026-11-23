"""Drive the AVR TWI transaction and ADS1100 against simulated bus events."""

import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def main():
    tests = Path(__file__).resolve().parent
    firmware = tests.parent / "Repetier"
    source = (firmware / "TwiTransfer.cpp").read_text(encoding="utf-8")
    gauge = (firmware / "StrainGauge.cpp").read_text(encoding="utf-8")
    with TemporaryDirectory(prefix="rfx-twi-") as temp:
        build = Path(temp)
        (build / "twi-transfer-firmware.inc").write_text(
            source[source.index("namespace {"):], encoding="utf-8",
        )
        (build / "strain-gauge-firmware.inc").write_text(
            gauge.removeprefix('#include "Repetier.h"\n'), encoding="utf-8",
        )
        for test in ("twi-transfer", "strain-gauge"):
            executable = build / test
            subprocess.run([
                os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                "-I", str(build), "-I", str(firmware), str(tests / f"{test}.cpp"), "-o", str(executable),
            ], check=True)
            subprocess.run([str(executable)], check=True, timeout=10)


if __name__ == "__main__":
    main()
