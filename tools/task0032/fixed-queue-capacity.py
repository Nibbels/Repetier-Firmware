#!/usr/bin/env python3
"""Prepare/test the fixed-capacity cleanup on temporary source copies only.

After P0, rebase the explicit changes on the actual integration tree, run all
four canonical builds and check linked layout. This is not a production patch.
"""
from pathlib import Path
from tempfile import TemporaryDirectory
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
TOKEN = re.compile(r"\bMOVE_CACHE_SIZE\b")
CAPACITY = 24


def edit(path, transform):
    before = path.read_text(encoding="utf-8")
    after = transform(before)
    assert before != after, path
    path.write_text(after, encoding="utf-8")
    print(f"candidate: {path.name}", flush=True)


def once(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)


def apply(fw):
    # Fail on a new consumer instead of blindly changing preprocessor semantics.
    allowed = {"Configuration.h", "MoveQueue.h", "MotionPlanner.cpp",
               "motion.cpp", "Printer.cpp"}
    found = {p.name for p in fw.rglob("*")
             if p.is_file() and p.suffix in (".h", ".cpp", ".ino", ".c")
             and TOKEN.search(p.read_text(encoding="utf-8"))}
    assert found == allowed, (found, allowed)
    def config(s):
        s, n = re.subn(r"^#define\s+MOVE_CACHE_SIZE\s+24\b[^\n]*\n", "", s, flags=re.M)
        assert n == 1
        return s
    edit(fw / "Configuration.h", config)
    edit(fw / "MoveQueue.h", lambda s: once(s, "class MoveQueue {\npublic:",
        "class MoveQueue {\npublic:\n    static constexpr uint8_t capacity = 24;")
        .replace("MOVE_CACHE_SIZE", "capacity"))
    edit(fw / "MotionPlanner.cpp", lambda s: once(s,
        "constexpr millis_t planningReserveTicks = MOVE_CACHE_SIZE < 10 ? 4500L * MOVE_CACHE_SIZE : 45000L;",
        "constexpr millis_t planningReserveTicks = 45000L;"))
    def motion(s):
        s, n = re.subn(r"^#if MOVE_CACHE_SIZE < 5\n#error MOVE_CACHE_SIZE must be at least 5\n#endif[^\n]*\n", "", s, flags=re.M)
        assert n == 1
        return once(s, "queue.count() + slots <= MOVE_CACHE_SIZE",
                       "queue.count() + slots <= MoveQueue::capacity")
    edit(fw / "motion.cpp", motion)
    edit(fw / "Printer.cpp", lambda s: once(s,
        'Com::config(PSTR("PrintlineCache:"), MOVE_CACHE_SIZE);',
        'Com::config(PSTR("PrintlineCache:"), MoveQueue::capacity);'))
    for p in fw.rglob("*"):
        if p.is_file() and p.suffix in (".h", ".cpp", ".ino", ".c"):
            assert not TOKEN.search(p.read_text(encoding="utf-8")), p
    # Deliberately retain pacing policy. It is not a capacity setting.
    planner = (fw / "MotionPlanner.cpp").read_text()
    assert "MOVE_CACHE_LOW" in planner and "LOW_TICKS_PER_MOVE" in planner


def fixture(s):
    anchor = '#include "pause-sense-offset-firmware.inc"'
    before, after = s.split(anchor)
    # The mock PrintLine definition precedes the extracted MoveQueue class.
    # Its one early assertion needs a fixture literal, checked against actual
    # capacity below. All later tests use the real typed capacity.
    assert len(TOKEN.findall(before)) == 1
    before = once(before, "linesCount < MOVE_CACHE_SIZE", "linesCount < 24")
    return before + anchor + '\nstatic_assert(MoveQueue::capacity == 24, "production queue capacity");' + TOKEN.sub("MoveQueue::capacity", after)


def main():
    with TemporaryDirectory(prefix="task0032-fixed-queue-") as directory:
        root = Path(directory)
        shutil.copytree(ROOT / "Repetier", root / "Repetier")
        (root / "tests").mkdir()
        for name in ("pause-sense-offset.py", "pause-sense-offset.cpp"):
            shutil.copyfile(ROOT / "tests" / name, root / "tests" / name)
        apply(root / "Repetier")
        edit(root / "tests/pause-sense-offset.cpp", fixture)
        subprocess.run([sys.executable, str(root / "tests/pause-sense-offset.py")], check=True)
        print("Fixed 24-slot queue candidate: existing actual-source motion/pause replays passed")


if __name__ == "__main__":
    main()
