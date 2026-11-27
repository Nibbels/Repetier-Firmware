#!/usr/bin/env python3
"""Stage small already-selected deltas in a disposable exact P0 checkout.

No production edits. Reuse earlier candidate sources and actual P0 replays.
This is a preflight for a moving development baseline, not a P0 release gate.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import runpy
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory

HERE = Path(__file__).resolve().parent
P0 = "9345d9f52f9428be0f920e6777c1e50286c42d71"


def run(*args, cwd=None):
    subprocess.run([str(x) for x in args], cwd=cwd, check=True)


def edit(p, transform):
    before = p.read_text(encoding="utf-8")
    after = transform(before)
    assert before != after, p
    p.write_text(after, encoding="utf-8")
    print("candidate:", p.name, flush=True)


def once(s, old, new):
    assert s.count(old) == 1, old
    return s.replace(old, new)


def replace_function(s, start, end, body):
    i = s.index(start)
    j = s.index(end, i)
    return s[:i] + body + s[j:]


def fixed_uart(root):
    fw, tests = root/"Repetier", root/"tests"
    candidate = runpy.run_path(str(HERE/"fixed-uart-init.py"))
    edit(fw/"UsbSerial.h", lambda s: once(s, "static void begin(unsigned long baud);", "static void begin();"))
    edit(fw/"UsbSerial.cpp", lambda s: replace_function(s, "void UsbSerial::begin(unsigned long baud)",
        "bool UsbSerial::available()", candidate["CANDIDATE"]))
    edit(fw/"HAL.h", lambda s: once(s,
        "static inline void serialSetBaudrate(long baud) {\n        UsbSerial::begin(baud);\n\n    } // serialSetBaudrate",
        "static inline void initializeSerial() {\n        UsbSerial::begin();\n    }"))
    def printer(s):
        assert s.count("HAL::serialSetBaudrate(firmwareSerialBaudrate);") == 2
        return once(s.replace("HAL::serialSetBaudrate(firmwareSerialBaudrate);", "HAL::initializeSerial();"),
            'Com::config(PSTR("Baudrate:"), firmwareSerialBaudrate);',
            'Com::config(PSTR("Baudrate:"), 250000UL);')
    edit(fw/"Printer.cpp", printer)
    edit(fw/"Constants.h", lambda s: once(s,
        "// Fixed USB application protocol rate; not a bootloader setting.\nconstexpr unsigned long firmwareSerialBaudrate = 250000UL;\n", ""))
    def uart_test(s):
        s = once(s, "volatile uint8_t UCSR0A, UCSR0B,", "enum { UCSZ00 = 1, UCSZ01 = 2 };\nvolatile uint8_t UCSR0C, UCSR0A, UCSR0B,")
        s = replace_function(s, "void checkBaud(", "void receive()", candidate["CHECK"])
        return once(s, "    checkBaud(57600, false, 16);\n    checkBaud(300, false, 3332);\n    checkBaud(115200, true, 16);", "    checkFixedStartup();")
    edit(tests/"usb-serial.cpp", uart_test)
    # Check every source consumer; legacy EEPROM address rejection remains
    # untouched. Neither bootloader nor recovery settings are changed.
    for p in fw.rglob("*"):
        if p.is_file() and p.suffix in (".h", ".cpp", ".ino", ".c"):
            s = p.read_text(encoding="utf-8")
            assert not re.search(r"\b(?:firmwareSerialBaudrate|serialSetBaudrate)\b", s), p
            assert not re.search(r"\bUsbSerial::begin\s*\(\s*[^)\s]", s), p


def fixed_queue(root):
    q = runpy.run_path(str(HERE/"fixed-queue-capacity.py"))
    q["apply"](root/"Repetier")
    tests = root/"tests"
    edit(tests/"pause-sense-offset.cpp", lambda s: q["fixture"](s).replace(
        "firmwareSerialBaudrate", "fixtureWireBaudrate").replace(
        "constexpr uint8_t fixtureQueueCapacity = 24;",
        "constexpr uint8_t fixtureQueueCapacity = 24;\nconstexpr uint32_t fixtureWireBaudrate = 250000UL;"))
    edit(tests/"pause-sense-offset.py", lambda s: once(s,
        'pause.removeprefix(\'#include "Repetier.h"\\n\'),',
        'pause.removeprefix(\'#include "Repetier.h"\\n\').replace("MoveQueue::capacity", "fixtureQueueCapacity"),'))


def parser_cleanup(root):
    fw, tests = root/"Repetier", root/"tests"
    decimal = runpy.run_path(str(HERE/"parser-avr-bench.py"))["DECIMAL_CANDIDATE"]
    edit(fw/"Communication.cpp", lambda s: replace_function(s,
        "void Com::printNumber(uint32_t n)", "\n\nvoid Com::printArrayFLN", decimal.rstrip()))
    edit(fw/"GCodeInput.cpp", lambda s: once(s,
        "command.params2 = *(uint16_t*)p;\n        p += 2;",
        "command.params2 = *(uint16_t*)p;\n        p += 2;\n        if (command.params2 & 0xF800u)\n            return false;"))
    wire = runpy.run_path(str(HERE/"parser-wire-replay.py"))["SCENARIOS"]
    def cpp(s):
        if "#include <cstring>" not in s:
            s = once(s, "#include <cstdlib>", "#include <cstdlib>\n#include <cstring>")
        s = once(s, "void numberedLines() {", wire+"\nvoid numberedLines() {")
        s = once(s, 'if (scenario == "numbered") numberedLines();',
            'if (scenario == "wire-ascii" || scenario == "wire-binary-v1" || scenario == "wire-binary-v2" || scenario == "wire-mixed") wireMoves(scenario);\n'
            '    else if (scenario == "wire-unsupported") unsupportedExtensionBaseline();\n'
            '    else if (scenario == "numbered") numberedLines();')
        return "#define TASK0032_REJECT_UNSUPPORTED 1\n"+s
    edit(tests/"serial-protocol.cpp", cpp)
    edit(tests/"serial-protocol.py", lambda s: once(s, 'for scenario in ("numbered",',
        'for scenario in ("wire-ascii", "wire-binary-v1", "wire-binary-v2", "wire-mixed", "wire-unsupported", "numbered",'))


def sqrt_cleanup(root):
    p = root/"Repetier/HAL.cpp"
    def change(s):
        i = s.index("uint16_t HAL::integerSqrt(")
        j = s.index("} // integerSqrt", i)
        old = s[i:j]
        new = once(old, ': "=r"(b)\n        : "r"(a)\n        : "r18", "r19", "r27", "r26");',
                         ': "=r"(b), "+r"(a)\n        :\n        : "r18", "r19", "r27", "r26", "cc");')
        return s[:i]+new+s[j:]
    edit(p, change)


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", required=True, type=Path)
    p.add_argument("--out", required=True, type=Path)
    a = p.parse_args()
    source = a.source.resolve()
    out = a.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    head = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    assert head == P0, (head, P0)
    assert not subprocess.check_output(["git","-C",str(source),"status","--porcelain"], text=True)
    with TemporaryDirectory(prefix="task0032-p0-preflight-") as directory:
        root = Path(directory)
        for folder in ("Repetier", "tests"):
            shutil.copytree(source/folder, root/folder)
        fixed_uart(root)
        fixed_queue(root)
        parser_cleanup(root)
        sqrt_cleanup(root)
        files = []
        import difflib
        diff = []
        for p in sorted(root.rglob("*")):
            if not p.is_file():
                continue
            relative = p.relative_to(root)
            old = source/relative
            if old.is_file() and digest(p) != digest(old):
                files.append({"file": str(relative),"old_sha256":digest(old),"candidate_sha256":digest(p)})
                diff.extend(difflib.unified_diff(old.read_text().splitlines(True), p.read_text().splitlines(True),
                    fromfile="a/"+str(relative), tofile="b/"+str(relative)))
        (out/"candidate.diff").write_text("".join(diff))
        (out/"sources.json").write_text(json.dumps({"schema":1,"p0":head,"changes":files,
            "qualification":"temporary source changes only; selected host replays, no linked four-board, heater or hardware qualification"},indent=2)+"\n")
        for test in ("serial-protocol.py","pause-sense-offset.py"):
            run(sys.executable,root/"tests"/test)
        print("P0 small cleanup preflight: serial including thermal mode, ring spans and motion/pause replays passed",flush=True)


if __name__ == "__main__":
    main()
