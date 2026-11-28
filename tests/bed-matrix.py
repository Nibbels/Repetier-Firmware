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
    interpolation = (firmware / "BedMatrixInterpolation.h").read_text(encoding="utf-8")
    grid = (firmware / "BedMatrixGrid.h").read_text(encoding="utf-8")
    storage = (firmware / "BedMatrixStorage.h").read_text(encoding="utf-8")
    diagnostics = (firmware / "BedMatrixDiagnostics.h").read_text(encoding="utf-8")
    sections = (
        (source, "static void initCompensationMatrix(void) {", '#include "BedMatrixGrid.h"'),
        (interpolation, "namespace {\nstruct MatrixSpan", "#endif"),
        (grid, "static char prepareCompensationMatrix(void) {", "#endif"),
        (storage, "static bool writeMatrixSector(", "bool switchActiveHeatBed("),
        (diagnostics, "void fixKeramikLochInMatrix(", "#endif"),
    )
    with TemporaryDirectory(prefix="rfx-bed-matrix-") as directory:
        build = Path(directory)
        (build / "bed-matrix-firmware.inc").write_text(
            eeprom[eeprom.index("namespace ExternalEeprom {"):] + "\n" + "\n".join(
            part[part.index(start):part.index(end, part.index(start))]
            for part, start, end in sections
        ), encoding="utf-8")
        executable = build / "bed-matrix"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(firmware), "-I", str(build), str(tests / "bed-matrix.cpp"), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True, timeout=10)


if __name__ == "__main__":
    main()
