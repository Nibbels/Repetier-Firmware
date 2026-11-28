#!/usr/bin/env python3
"""A45 representative combined thermal policy model, not firmware authorization."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as d:
    exe=Path(d)/"decouple"
    subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
                    str(root/"tools/task0034/thermal-profile-decouple.cpp"),"-o",str(exe)],check=True)
    subprocess.run([str(exe),"4"],check=True,timeout=240)
