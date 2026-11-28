"""Execute the production direct-slot pulse path with simulated switch contacts.

The AVR interrupt dispatcher, planner and GPIO timing are not emulated.
The prepared move, guard, Bresenham, ramp, XY/Z travel and ledgers are real.
"""
import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

tests = Path(__file__).resolve().parent
firmware = tests.parent / "Repetier"
executor = (firmware / "StepExecutor.cpp").read_text()
z = (firmware / "ZMotion.cpp").read_text()
with TemporaryDirectory(prefix="rfx-guarded-") as directory:
    build = Path(directory)
    (build / "guarded-firmware.inc").write_text(
        z.removeprefix('#include "Repetier.h"\n') + "\n" +
        executor[executor.index("template <int8_t limitDirection>"):],
        encoding="utf-8")
    for model in ("RF1000", "RF2000", "RF2000v2"):
        executable = build / model
        subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-O2",
                        "-Wall", "-Wextra", "-Werror",
                        "-I", str(firmware), "-I", str(build),
                        "-DMOTHERBOARD=DEVICE_TYPE_" + model,
                        str(tests / "task0034-guarded-drive.cpp"), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
        print(model + " guarded drive replay passed", flush=True)
