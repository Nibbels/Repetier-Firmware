"""Replay profile migration, menu edits, display labels and heater requests."""

import os
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory


def section(source, start, end):
    first = source.index(start)
    return source[first:source.index(end, first + len(start))]


def main():
    tests = Path(__file__).resolve().parent
    firmware = tests.parent / "Repetier"
    ui = (firmware / "ui.cpp").read_text(encoding="utf-8")
    menu = (firmware / "uimenu.h").read_text(encoding="utf-8")
    definitions = "\n".join((firmware / name).read_text(encoding="utf-8")
                            for name in ("Configuration.h", "Eeprom.h", "ui.h", "uilang.h"))
    with TemporaryDirectory(prefix="rfx-preheat-") as directory:
        build = Path(directory)
        (build / "preheat-settings.inc").write_text("\n".join(re.findall(
            r"^#define (?:PREHEAT_\w+|EPR_RF_PREHEAT_\w+|UI_\w+_PREHEAT_\w+|UI_SET_(?:MIN|MAX)_\w+_TEMP)\s+[^\n]+",
            definitions, re.M)), encoding="utf-8")
        (build / "preheat-firmware.inc").write_text(
            (firmware / "PreheatProfiles.cpp").read_text(encoding="utf-8").removeprefix('#include "Repetier.h"\n'),
            encoding="utf-8")
        (build / "preheat-panel.inc").write_text(
            "void edit(int action, int8_t increment) { switch (action) {\n"
            + section(ui, "    case UI_ACTION_PREHEAT_A_BED:", "    case UI_ACTION_FANSPEED:")
            + "} }\nvoid activate(int action) { switch (action) {\n"
            + section(ui, "    case UI_ACTION_PREHEAT_A:\n", "    case UI_ACTION_COOLDOWN:")
            + "} }\nvoid renderField(char c2) { switch ('V') {\n"
            + section(ui, "        case 'V':", "        case 'a':")
            + "} }\n#define UI_MENU_ACTIONCOMMAND(name, text, action) constexpr char name[] = text;\n"
            + section(menu, '#if UI_COLS <= 16\nUI_MENU_ACTIONCOMMAND(ui_menu_quick_preheat_a', "UI_MENU_ACTIONCOMMAND(ui_menu_quick_cooldown"),
            encoding="utf-8")
        for columns in (16, 20):
            executable = build / "preheat"
            subprocess.run([
                os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                "-I", str(build), "-I", str(firmware), f"-DUI_COLS={columns}", str(tests / "preheat-profiles.cpp"),
                "-o", str(executable),
            ], check=True)
            subprocess.run([str(executable)], check=True, timeout=10)
            print(f"Preheat profile replay passed at {columns} columns", flush=True)


if __name__ == "__main__":
    main()
