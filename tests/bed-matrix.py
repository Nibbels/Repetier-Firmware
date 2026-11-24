"""Replay bed matrix storage, preparation and interpolation with simulated EEPROM."""

import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def main():
    tests = Path(__file__).resolve().parent
    firmware = tests.parent / "Repetier"
    source = (firmware / "BedMatrix.cpp").read_text(encoding="utf-8")
    eeprom = (firmware / "ExternalEeprom.cpp").read_text(encoding="utf-8")
    sections = (
        ("static void initCompensationMatrix(void) {", "static void outputCompensationMatrix("),
        ("static char prepareCompensationMatrix(void) {", "static void adjustCompensationMatrix("),
        ("static bool writeMatrixSector(", "static void clearExternalEEPROM("),
        ("namespace {\nstruct MatrixSpan", "long getZMatrixDepth_CurrentXY("),
    )
    with TemporaryDirectory(prefix="rfx-bed-matrix-") as directory:
        build = Path(directory)
        (build / "bed-matrix-firmware.inc").write_text(
            eeprom[eeprom.index("namespace ExternalEeprom {"):] + "\n" + "\n".join(
            source[source.index(start):source.index(end, source.index(start))]
            for start, end in sections
        ), encoding="utf-8")
        executable = build / "bed-matrix"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(firmware), "-I", str(build), str(tests / "bed-matrix.cpp"), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True, timeout=10)


if __name__ == "__main__":
    main()
