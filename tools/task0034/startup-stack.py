#!/usr/bin/env python3
"""Fail-closed proof of the pinned AVR application CRT startup stack.

Checks reset-to-main initialization and the single LTO constructor against
exact linked ELF text and disassembly. Does not cover the bootloader, main
setup, arbitrary corruption, or hardware timing. The main entry return PC
is already included by the foreground frame inventory.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import struct
import sys
import zipfile

sys.dont_write_bytecode = True


def elf_text(data):
    h = struct.unpack_from('<16sHHIIIIIHHHHHH', data)
    assert h[0][:6] == b'\x7fELF\x01\x01' and h[2] == 83 and h[11] == 40
    sections = [struct.unpack_from('<10I', data, h[6]+i*h[11]) for i in range(h[12])]
    names = sections[h[13]]
    names = data[names[4]:names[4]+names[5]]
    text, = (s for s in sections if names[s[0]:].split(b'\0', 1)[0] == b'.text')
    assert text[3] == 0
    return data[text[4]:text[4]+text[5]]


def verify(board, flash, dump, symbols, fg):
    ins, _, labels = fg.decode_linked(dump, board)
    # Compare every decoded instruction used here against actual ELF bytes.
    raw = {int(m[1], 16): bytes.fromhex(m[2]) for m in re.finditer(
        r'^\s*([0-9a-f]+):\t((?:[0-9a-f]{2} )+)\s*\t', dump, re.M)}

    def sym(name):
        m = re.search(r'^([0-9a-f]+) (?:[0-9a-f]+ )?\w '+re.escape(name)+r'$', symbols, re.M)
        assert m, name
        return int(m[1], 16)

    def read(pc):
        w, op, arg = ins[pc]
        assert flash[pc:pc+w] == raw[pc] and len(raw[pc]) == w
        return w, op, arg.split(';')[0].strip()

    def sequence(start, expected, end=None):
        pc = start
        for op, arg in expected:
            w, actual, value = read(pc)
            assert actual == op, (board, hex(pc), op, actual)
            if isinstance(arg, int):
                assert fg.target(ins[pc][2]) == arg, (board, hex(pc), arg)
            else:
                assert value == arg, (board, hex(pc), arg, value)
            pc += w
        if end is not None:
            assert pc == end, (board, hex(pc), hex(end))
        return pc

    def imm(reg, n): return (reg + ', 0x%02X' % (n & 255))
    start, end = sym('__ctors_start'), sym('__ctors_end')
    early, copy, clear, ctors = (sym(n) for n in ('heaterWatchdogEarlyInit', '__do_copy_data', '__do_clear_bss', '__do_global_ctors'))
    helper, main, exit_ = (sym(n) for n in ('__tablejump2__', 'main', '_exit'))
    ds, de, bs, be = (sym(n)-0x800000 for n in ('__data_start', '__data_end', '__bss_start', '__bss_end'))
    load = sym('__data_load_start')
    cause = sym('heaterWatchdogResetCause')-0x800000
    assert ds == 0x200 and ds < de == bs < be <= cause < 0x2200
    assert end-start == 2 and end % 2 == 0 and sym('__stack') == 0x21ff
    assert 0 < load < 0x3e000 and load+de-ds <= 0x3e000
    sequence(0, [('jmp', end)], 4)
    sequence(end, [('eor','r1, r1'), ('out','0x3f, r1'), ('ldi','r28, 0xFF'),
                   ('ldi','r29, 0x21'), ('out','0x3e, r29'), ('out','0x3d, r28'),
                   ('ldi','r16, 0x00'), ('out','0x3c, r16')], early)
    sequence(early, [('cli',''), ('in','r24, 0x34'), ('sts',f'0x{cause:04X}, r24'),
                     ('andi','r24, 0xF7'), ('out','0x34, r24'), ('wdr',''),
                     ('ldi','r24, 0x18'), ('sts','0x0060, r24'), ('sts','0x0060, r1'),
                     ('ldi','r24, 0x00')], copy)
    sequence(copy, [('ldi',imm('r17',de>>8)), ('ldi',imm('r26',ds)), ('ldi',imm('r27',ds>>8)),
                    ('ldi',imm('r30',load)), ('ldi',imm('r31',load>>8)), ('ldi',imm('r16',load>>16)),
                    ('out','0x3b, r16'), ('rjmp',copy+20), ('elpm','r0, Z+'), ('st','X+, r0'),
                    ('cpi',imm('r26',de)), ('cpc','r27, r17'), ('brne',copy+16)], clear)
    sequence(clear, [('ldi',imm('r18',be>>8)), ('ldi',imm('r26',bs)), ('ldi',imm('r27',bs>>8)),
                     ('rjmp',clear+10), ('st','X+, r1'), ('cpi',imm('r26',be)),
                     ('cpc','r27, r18'), ('brne',clear+8)], ctors)
    # The generated AVR libc walker decrements the word address once,
    # calls the helper which tail-jumps to the constructor, then calls main.
    sw, ew = start//2, end//2
    assert ew < 65536
    sequence(ctors, [('ldi',imm('r17',sw>>8)), ('ldi',imm('r28',ew)), ('ldi',imm('r29',ew>>8)),
                     ('ldi','r16, 0x00'), ('rjmp',ctors+22), ('sbiw','r28, 0x01'),
                     ('sbc','r16, r1'), ('mov','r24, r16'), ('movw','r30, r28'), ('call',helper),
                     ('cpi',imm('r28',sw)), ('cpc','r29, r17'), ('ldi','r24, 0x00'),
                     ('cpc','r16, r24'), ('brne',ctors+10), ('call',main), ('jmp',exit_)])
    sequence(helper, [('add','r30, r30'), ('adc','r31, r31'), ('adc','r24, r24'),
                      ('out','0x3b, r24'), ('elpm','r0, Z+'), ('elpm','r31, Z'),
                      ('mov','r30, r0'), ('eijmp','')])
    entry = 2*int.from_bytes(flash[start:end], 'little')
    pc = entry
    if sym('__trampolines_start') <= pc < sym('__trampolines_end'):
        assert read(pc)[:2] == (4, 'jmp')
        pc = fg.target(ins[pc][2])
    constructor = pc
    assert labels[pc].startswith('global constructors keyed to ')
    # This pinned LTO constructor is straight-line initialization only.
    # Check every store's SRAM address; no stack, call, branch, I/O or
    # callee-saved register mutation (the walker keeps its index there).
    known = {'r1': 0}
    count = 0
    while True:
        assert count < 1024
        w, op, arg = read(pc)
        pc += w
        count += 1
        if op == 'ret':
            assert arg == '' and w == 2
            break
        if op == 'ldi':
            reg, value = arg.split(', ')
            assert reg in ('r24','r25','r30','r31')
            known[reg] = int(value, 16)
            continue
        assert op in ('sts','st','std'), (board, hex(pc-w), op, arg)
        dst, src = arg.split(', ')
        assert src in known
        if op == 'sts':
            address = int(dst, 16)
        else:
            m = re.fullmatch(r'Z(?:\+(\d+))?', dst)
            assert m and 'r30' in known and 'r31' in known
            address = known['r30']+256*known['r31']+int(m[1] or 0)
        assert ds <= address < be, (board, hex(address), 'non-static store')
    return {'constructorTable': [hex(start),hex(end)], 'constructorEntry':hex(entry),
            'constructor':hex(constructor), 'constructorInstructions':count,
            'preMainStackBytes':3, 'interruptsEnabledBeforeMain':False,
            'mainReturnPcAlreadyCounted':True}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive')
    parser.add_argument('sha256')
    args = parser.parse_args()
    with open(args.archive,'rb') as f:
        assert hashlib.sha256(f.read()).hexdigest() == args.sha256
    spec = importlib.util.spec_from_file_location('fg', Path(__file__).with_name('foreground-stack-depth.py'))
    fg = importlib.util.module_from_spec(spec); spec.loader.exec_module(fg)
    with zipfile.ZipFile(args.archive) as z:
        summary = json.loads(z.read('summary.json')); assert not summary['failures']
        reports = {}
        for board in fg.BOARDS:
            prefix = board+'/'
            assert json.loads(z.read(prefix+'verification.json'))['status']=='verified'
            elf, = [name for name in z.namelist() if name.startswith(prefix) and name.endswith('.elf')]
            reports[board] = verify(board,elf_text(z.read(elf)),z.read(prefix+'disassembly.txt').decode(),z.read(prefix+'symbols.txt').decode(),fg)
    print(json.dumps({'testedMerge':summary['sha'],'sha256':args.sha256,'boards':reports,
                      'limitations':['pinned application CRT only; bootloader and main excluded','intact hardware/CPU/flash assumed']},indent=2))
