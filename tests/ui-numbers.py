"""Exercise panel formatting, including numeric limits and clipped status fields."""

import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def main():
    tests = Path(__file__).resolve().parent
    ui = (tests.parent / "Repetier" / "UiFormatting.cpp").read_text(encoding="utf-8")
    def section(first, last):
        start = ui.index(first)
        return ui[start:ui.index(last, start + len(first))]

    parse_prefix = section("void UIDisplay::parse(", "        case 'V':")
    parse_prefix = parse_prefix.replace("    int ivalue = 0;\n    float fvalue = 0;\n", "")
    finish_parse = "            break;\n        }\n        }\n    }\n    printCols[col] = 0;\n}\n"
    formatting = (
        section("void UIDisplay::addInt(", "void UIDisplay::addStringP(")
        + parse_prefix
        + section("        case 'h':", "        case 'l':")
        + section("        case 'm':", "        case 'M':")
        + section("        case 'o':", "            if (c2 == 'c')")
        + finish_parse
        + ui[ui.index("void UIDisplay::setStatusP("):]
    )
    with TemporaryDirectory(prefix="rfx-ui-numbers-") as directory:
        build = Path(directory)
        (build / "ui-numbers-firmware.inc").write_text(formatting, encoding="utf-8")
        for columns in (16, 20):
            executable = build / "ui-numbers"
            subprocess.run([
                os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                "-fsanitize=address,undefined,float-cast-overflow", "-fno-pie", "-no-pie", "-fno-sanitize-recover=all",
                "-I", str(build), f"-DUI_COLS={columns}", str(tests / "ui-numbers.cpp"),
                "-o", str(executable),
            ], check=True)
            subprocess.run([str(executable)], check=True, timeout=10,
                           env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})
            print(f"Panel formatting passed at {columns} columns", flush=True)


if __name__ == "__main__":
    main()
