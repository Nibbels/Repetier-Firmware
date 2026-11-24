"""Exercise the AVR panel formatters with their native integer widths."""

import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def main():
    tests = Path(__file__).resolve().parent
    ui = (tests.parent / "Repetier" / "ui.cpp").read_text(encoding="utf-8")
    start = ui.index("void UIDisplay::addInt(")
    end = ui.index("void UIDisplay::addStringP(", start)
    with TemporaryDirectory(prefix="rfx-ui-numbers-") as directory:
        build = Path(directory)
        (build / "ui-numbers-firmware.inc").write_text(ui[start:end], encoding="utf-8")
        for columns in (16, 20):
            executable = build / "ui-numbers"
            subprocess.run([
                os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                "-fsanitize=undefined,float-cast-overflow", "-fno-sanitize-recover=all",
                "-I", str(build), f"-DUI_COLS={columns}", str(tests / "ui-numbers.cpp"),
                "-o", str(executable),
            ], check=True)
            subprocess.run([str(executable)], check=True, timeout=10)
            print(f"Panel number formatting passed at {columns} columns", flush=True)


if __name__ == "__main__":
    main()
