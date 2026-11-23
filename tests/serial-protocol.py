#!/usr/bin/env python3
"""Replay the firmware's serial receiver with a byte stream and a test clock."""

from pathlib import Path
from tempfile import TemporaryDirectory
import os
import re
import subprocess


def main():
    tests = Path(__file__).resolve().parent
    firmware = tests.parent / "Repetier"
    receiver = (firmware / "gcode.cpp").read_text(encoding="utf-8")
    communication = (firmware / "Communication.cpp").read_text(encoding="utf-8")
    configuration = (firmware / "Configuration.h").read_text(encoding="utf-8")
    names = set(re.findall(r"\bCom::(t\w+)\b", receiver)) | {"tError"}
    constants = {
        name: value for name, value in re.findall(r'^FSTRINGVALUE\(Com::(t\w+), ("[^"]*")\)', communication, re.M)
        if name in names
    }
    assert names == constants.keys()
    capacity = re.search(r"^#define GCODE_BUFFER_SIZE\s+(\d+)", configuration, re.M)[1]

    with TemporaryDirectory(prefix="rfx-serial-replay-") as directory:
        build = Path(directory)
        (build / "gcode-settings.inc").write_text(f"#define GCODE_BUFFER_SIZE {capacity}\n", encoding="utf-8")
        (build / "gcode-receiver.inc").write_text(receiver.replace('#include "Repetier.h"\n', "", 1), encoding="utf-8")
        (build / "gcode-replies.inc").write_text(
            "".join(f"const char {name}[] = {value};\n" for name, value in sorted(constants.items())),
            encoding="utf-8",
        )
        replay = build / "serial-replay"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-fno-strict-aliasing", "-I", str(firmware), "-I", str(build),
            str(tests / "serial-protocol.cpp"), "-o", str(replay),
        ], check=True)
        for scenario in ("numbered", "queue", "display", "idle", "binary", "stopping"):
            subprocess.run([str(replay), scenario], check=True, timeout=10)


if __name__ == "__main__":
    main()
