#!/usr/bin/env python3
"""Task-private linked AVR ISR comparison; no production files modified.

Enumerates acyclic structural machine-code paths, NOT a CPU/peripheral emulator.
Branch conditions are not solved: infeasible combinations may be included.
Interrupt entry=5 cycles + 3-cycle vector JMP, RETI=5 on ATmega2560.
Unknown opcodes, calls, loops, non-JMP vectors fail rather than guess.
"""
import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASE_RX = """    const uint8_t byte = UDR0;
    const uint8_t next = rx_.next(rx_.head);
    if (next != rx_.tail) {
        rx_.bytes[rx_.head] = byte;
        rx_.head = next;
    }"""
CACHED_RX = """    const uint8_t byte = UDR0;
    const uint8_t head = rx_.head;
    const uint8_t next = rx_.next(head);
    if (next != rx_.tail) {
        rx_.bytes[head] = byte;
        rx_.head = next;
    }"""
CHECKED_RX = """    const uint8_t status = UCSR0A;
    const uint8_t byte = UDR0;
    const uint8_t error = status & (_BV(FE0) | _BV(DOR0) | _BV(UPE0));
    if (error) {
        lossFlags |= error;
        return;
    }
    const uint8_t head = rx_.head;
    const uint8_t next = rx_.next(head);
    if (next != rx_.tail) {
        rx_.bytes[head] = byte;
        rx_.head = next;
    } else {
        lossFlags |= 1;
    }"""
BASE_TX = """    if (tx_.head == tx_.tail) {
        UCSR0B &= ~_BV(UDRIE0);
        return;
    }
    const uint8_t byte = tx_.bytes[tx_.tail];
    tx_.tail = tx_.next(tx_.tail);
    UDR0 = byte;"""
CACHED_TX = """    const uint8_t tail = tx_.tail;
    if (tx_.head == tail) {
        UCSR0B &= ~_BV(UDRIE0);
        return;
    }
    const uint8_t byte = tx_.bytes[tail];
    UDR0 = byte;
    tx_.tail = tx_.next(tail);"""
MAIN = """#include "UsbSerial.h"
int main() {
    UsbSerial::begin(500000);
    for (;;) {
        int v = UsbSerial::read();
        if (v >= 0) UsbSerial::write(static_cast<uint8_t>(v));
    }
}
"""


def replace_once(s, old, new):
    if s.count(old) != 1:
        raise ValueError("source anchor changed: " + old[:80])
    return s.replace(old, new)


def candidate(src, name):
    if name == "baseline":
        return src
    if name == "cached":
        return replace_once(replace_once(src, BASE_RX, CACHED_RX), BASE_TX, CACHED_TX)
    src = replace_once(src, BASE_RX, CHECKED_RX)
    src = replace_once(src, BASE_TX, CACHED_TX)
    src = replace_once(src, "inline void UsbSerial::receiveInterrupt()",
                       "static volatile uint8_t lossFlags;\n"
                       "inline __attribute__((always_inline)) void UsbSerial::receiveInterrupt()")
    if name == "checked-drain2":
        src = replace_once(src, "    UsbSerial::receiveInterrupt();\n}",
                           "    UsbSerial::receiveInterrupt();\n"
                           "    if (UCSR0A & _BV(RXC0)) UsbSerial::receiveInterrupt();\n}")
    if name == "checked-tx-last":
        src = replace_once(src, "    tx_.tail = tx_.next(tail);",
                           "    const uint8_t next = tx_.next(tail);\n"
                           "    tx_.tail = next;\n"
                           "    if (next == tx_.head) UCSR0B &= ~_BV(UDRIE0);")
    return src


def run(cmd):
    return subprocess.check_output([str(x) for x in cmd], text=True, stderr=subprocess.STDOUT)


def insns(dis):
    result = {}
    for line in dis.splitlines():
        m = re.match(r"^\s*([0-9a-f]+):\s+((?:[0-9a-f]{2}\s+)+)\s*([a-z][a-z0-9]*)\s*(.*)$", line)
        if m:
            result[int(m[1], 16)] = (len(m[2].split()), m[3], m[4])
    return result


def target(arg, next_pc):
    m = re.search(r";\s*0x([0-9a-f]+)", arg, re.I)
    if m:
        return int(m[1], 16)
    m = re.search(r"\.([+-]\d+)", arg)
    if m:
        return next_pc + int(m[1])
    m = re.match(r"\s*0x([0-9a-f]+)", arg, re.I)
    if m:
        return int(m[1], 16)
    raise ValueError("no branch target: " + arg)


def path_set(dis, vec):
    ins = insns(dis)
    start = int(re.search(r"^([0-9a-f]+) <__vector_" + str(vec) + r">:", dis, re.M)[1], 16)
    size, op, arg = ins[4 * vec]
    assert size == 4 and op == "jmp" and target(arg, 4 * vec + size) == start
    paths = []
    one = set("in out eor clr subi sbci andi ori ldi cp cpc cpi add adc sub sbc and or mov movw inc dec com neg lsr asr ror swap tst ser nop bld bst".split())
    two = set("push pop lds sts ld ldd st std adiw sbiw mul muls mulsu sbi cbi".split())
    skips = {"cpse", "sbrc", "sbrs", "sbic", "sbis"}
    def walk(pc, cycles, stack, peak, seen, trace, reads, statuses, writes, stores, pushes, overhead):
        if pc in seen or len(seen) > 160 or len(paths) > 4096:
            raise ValueError("loop/path bound at " + hex(pc))
        n, op, arg = ins[pc]
        nxt = pc + n
        seen = seen | {pc}
        trace = trace + [[hex(pc), op, arg]]
        if op == "reti":
            assert stack == 0
            paths.append(dict(cycles=cycles + 5, read_complete=reads,
                              status_reads=statuses, write_complete=writes,
                              indirect_stores=stores, pushed_registers=pushes,
                              stack_bytes_including_return=peak + 3,
                              entry_exit_push_pop_cycles=overhead + 13,
                              trace=trace))
            return
        if op.startswith("br") and op != "break":
            edges = [(nxt, 1), (target(arg, nxt), 2)]
        elif op in skips:
            skipped = ins[nxt][0]
            edges = [(nxt, 1), (nxt + skipped, 2 if skipped == 2 else 3)]
        elif op in ("rjmp", "jmp"):
            edges = [(target(arg, nxt), 2 if op == "rjmp" else 3)]
        elif op in one | two:
            edges = [(nxt, 1 if op in one else 2)]
        else:
            raise ValueError((vec, hex(pc), op, arg))
        if op == "push":
            stack += 1
            pushes += 1
            peak = max(peak, stack)
        if op == "pop":
            stack -= 1
            assert stack >= 0
        if op in ("push", "pop"):
            overhead += 2
        for dest, cost in edges:
            end = cycles + cost
            # memory-mapped USART0 registers are extended I/O, LDS/STS.
            addr = re.search(r"\b0x0*([cC][06])\b", arg)
            reg = addr[1].lower() if addr else ""
            walk(dest, end, stack, peak, seen, trace,
                 reads + ([end] if op == "lds" and reg == "c6" else []),
                 statuses + ([end] if op == "lds" and reg == "c0" else []),
                 writes + ([end] if op == "sts" and reg == "c6" else []),
                 stores + int(op in ("st", "std")), pushes, overhead)
    walk(start, 8, 0, 0, set(), [], [], [], [], 0, 0, 0)
    assert paths
    if vec == 25:
        assert all(p["read_complete"] for p in paths)
    else:
        assert all(len(p["write_complete"]) <= 1 for p in paths)
    return paths


def summarize(paths):
    groups = {}
    for p in paths:
        key = (len(p["read_complete"]), len(p["write_complete"]), p["indirect_stores"])
        groups.setdefault(key, []).append(p)
    return [
        dict(reads=key[0], writes=key[1], indirect_stores=key[2],
             paths=len(ps), cycles=sorted(set(p["cycles"] for p in ps)),
             first_read=sorted(set(p["read_complete"][0] for p in ps if p["read_complete"])),
             last_read=sorted(set(p["read_complete"][-1] for p in ps if p["read_complete"])),
             first_write=sorted(set(p["write_complete"][0] for p in ps if p["write_complete"])),
             max_stack=max(p["stack_bytes_including_return"] for p in ps),
             overhead_cycles=sorted(set(p["entry_exit_push_pop_cycles"] for p in ps)))
        for key, ps in sorted(groups.items())
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    cc = args.compiler.resolve()
    objdump = cc.with_name("avr-objdump")
    size = cc.with_name("avr-size")
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    src = (ROOT / "Repetier/UsbSerial.cpp").read_text()
    hdr = (ROOT / "Repetier/UsbSerial.h").read_text()
    report = dict(source_sha256=hashlib.sha256(src.encode()).hexdigest(),
                  header_sha256=hashlib.sha256(hdr.encode()).hexdigest(),
                  compiler=run([cc, "--version"]).splitlines()[0],
                  flags=["-mmcu=atmega2560", "-DF_CPU=16000000UL", "-Os", "-flto",
                         "-std=gnu++11", "-ffunction-sections", "-fdata-sections",
                         "-fno-exceptions", "-fno-threadsafe-statics"],
                  qualification="isolated linked structural paths, not integrated WCET",
                  variants={})
    for name in ("baseline", "cached", "checked", "checked-drain2", "checked-tx-last"):
        directory = out / name
        directory.mkdir(exist_ok=True)
        (directory / "UsbSerial.cpp").write_text(candidate(src, name))
        (directory / "UsbSerial.h").write_text(hdr)
        (directory / "main.cpp").write_text(MAIN)
        elf = directory / "candidate.elf"
        run([cc, *report["flags"], directory / "UsbSerial.cpp", directory / "main.cpp",
             "-Wl,--gc-sections", "-o", elf])
        dis = run([objdump, "-d", "-z", elf])
        (directory / "disassembly.txt").write_text(dis)
        rx, tx = path_set(dis, 25), path_set(dis, 26)
        assert all(1 <= len(p["read_complete"]) <= (2 if name == "checked-drain2" else 1) for p in rx)
        if name.startswith("checked"):
            for p in rx:
                assert all(any(previous < status < read for status in p["status_reads"])
                           for previous, read in zip([0] + p["read_complete"][:-1], p["read_complete"]))
        (directory / "paths.json").write_text(json.dumps(dict(rx=rx, tx=tx), indent=2))
        report["variants"][name] = dict(rx=summarize(rx), tx=summarize(tx),
                                       size=run([size, elf]).strip(),
                                       elf_sha256=hashlib.sha256(elf.read_bytes()).hexdigest())
    (out / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
