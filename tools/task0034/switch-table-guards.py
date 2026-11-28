#!/usr/bin/env python3
"""A73 conditional GCC7 switch-table range/target audit.

Uses the linked foreground decoder's candidate table words and instruction
boundaries. Assumes ABI-zero r1 at compares, EIND=0 at the helper, intact
control flow, and no unmodeled indirect transfers into a guarded suffix.
CRT constructor dispatch is explicitly reported, not proved here.
"""
import argparse
import bisect
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import sys
import zipfile

sys.dont_write_bytecode = True

HELPER = [('add', 'r30, r30'), ('adc', 'r31, r31'), ('adc', 'r24, r24'),
          ('out', '0x3b, r24'), ('elpm', 'r0, Z+'), ('elpm', 'r31, Z'),
          ('mov', 'r30, r0'), ('eijmp', '')]


def prove_tables(ins, tables, labels, target):
    keys = sorted(labels)

    def owner(pc):
        i = bisect.bisect_right(keys, pc)-1
        return labels[keys[i]] if i >= 0 else None

    def inst(pc):
        w, op, arg = ins[pc]
        return w, op, arg.split(';')[0].strip()

    helper = [a for a, n in labels.items() if n == '__tablejump2__']
    assert len(helper) == 1, helper
    h = helper[0]
    assert [inst(h+2*i) for i in range(len(HELPER))] == [(2, *x) for x in HELPER]
    reports, regions, crt_calls = [], [], []
    permitted_entries = set()
    for pc, ts in sorted(tables.items()):
        w, op, arg = ins[pc]
        assert op in ('jmp', 'rjmp') and target(arg) == h
        start = pc+w
        end = start+2*len(ts)
        assert 0 < len(ts) < 128
        resolved = []
        trampolines = 0
        for t in ts:
            assert t in ins and not start <= t < end
            if owner(t) == '__trampolines_start':
                assert ins[t][:2] == (4, 'jmp')
                t = target(ins[t][2])
                trampolines += 1
            assert t in ins and owner(t) == owner(pc) and not start <= t < end, (hex(pc), hex(t), 'table destination')
            resolved.append(t)

        # Walk backwards from final base addition and zero-extension.
        suffix = pc-6
        neg_base = 0
        for i, (op, reg) in enumerate([('subi', 'r30'), ('sbci', 'r31'), ('sbci', 'r24')]):
            width, actual, arg = inst(suffix+2*i)
            assert width == 2 and actual == op and re.fullmatch(reg+r', 0x[0-9A-Fa-f]{2}', arg), (hex(pc), 'base')
            neg_base |= int(arg.split(', ')[1], 16) << (8*i)
        assert ((-neg_base) & 0xffffff)*2 == start, (hex(pc), 'wrong table base')
        assert inst(suffix-2) == (2, 'eor', 'r24, r24')
        p = suffix-4
        if inst(p) == (2, 'ldi', 'r31, 0x00'):
            p -= 2
            reg = 'r30'
            m = re.fullmatch(r'r30, (r\d+)', inst(p)[2]) if p in ins and inst(p)[1] == 'mov' else None
            if m:
                assert inst(p)[0] == 2
                reg = m[1]
                p -= 2
            kind = 'u8'
        elif all(p-k in ins for k in (4, 2, 0)) and [inst(p-k) for k in (4, 2, 0)] == [
                (2, 'mov', 'r0, r30'), (2, 'add', 'r0, r0'), (2, 'sbc', 'r31, r31')]:
            # The admitted index is <128, so ADD cannot carry and SBC
            # sign-extension gives zero. The low byte remains unchanged.
            p -= 6
            reg, kind = 'r30', 'sign8'
        else:
            reg, kind = 'r30', 'u16'
        # p points at the range branch or last word of the default jump.
        suffix_start = p+2
        if p in ins and inst(p)[:2] == (2, 'brcc'):
            assert not suffix_start <= target(ins[p][2]) < end
            branch = p
        else:
            if p not in ins or inst(p)[:2] != (2, 'rjmp'):
                assert p-2 in ins and inst(p-2)[:2] == (4, 'jmp'), (hex(pc), 'default jump')
                p -= 2
            assert inst(p)[1] in ('jmp', 'rjmp') and not suffix_start <= target(ins[p][2]) < end
            branch = p-2
            assert inst(branch)[:2] == (2, 'brcs') and target(ins[branch][2]) == suffix_start, (hex(pc), 'in-range branch')
            permitted_entries.add((branch, suffix_start))
        cmp = branch-2
        if kind == 'u16':
            assert inst(cmp) == (2, 'cpc', 'r31, r1')
            cmp -= 2
        assert inst(cmp) == (2, 'cpi', f'{reg}, 0x{len(ts):02X}'), (hex(pc), hex(cmp), kind, inst(cmp))
        assert owner(cmp) == owner(pc)
        assert all(owner(a) == owner(pc) for a in range(cmp, pc+1, 2))
        regions.append((cmp, pc, start, end))
        reports.append({'dispatch': hex(pc), 'guard': hex(cmp), 'kind': kind,
                        'count': len(ts), 'owner': owner(pc), 'trampolineWords': trampolines,
                        'targets': [hex(t) for t in resolved]})

    # Reject any direct alternate entry, even a table destination returning
    # to its own guard suffix. Only the exact validated BRCS may enter it.
    skip = {'sbrc', 'sbrs', 'sbic', 'sbis', 'cpse'}
    for a, (w, op, arg) in ins.items():
        if any(start <= a < end for _, _, start, end in regions):
            continue  # table words, not code
        destinations = []
        if a in tables:
            destinations = list(tables[a])
        elif op in ('call', 'rcall', 'jmp', 'rjmp') or op.startswith('br'):
            destinations = [target(arg)]
            if h <= destinations[0] < h+2*len(HELPER):
                # Startup constructor dispatch has its own table and
                # return semantics, and is outside this proof.
                assert owner(a) == '__do_global_ctors' and op == 'call' and destinations[0] == h, (hex(a), 'unmodeled table helper entry')
                crt_calls.append(hex(a))
        elif op in skip:
            n = a+w
            if n in ins:
                destinations = [n+ins[n][0]]
        for dest in destinations:
            for cmp, pc, start, end in regions:
                assert not start <= dest < end, (hex(a), hex(dest), 'transfer into table data')
                if cmp < dest <= pc:
                    assert (a, dest) in permitted_entries, (hex(a), hex(dest), 'alternate range entry')
    return {'tables': reports, 'unprovedCrtHelperCalls': crt_calls}


def fixture(kind, far_default=False, moved=False):
    """Minimal instruction graph with two cases and exact helper."""
    ins, labels = {}, {0x100: 'switch', 0x400: '__tablejump2__'}
    p = 0x100

    def put(op, arg='', w=2):
        nonlocal p
        a = p
        ins[a] = w, op, arg
        p += w
        return a

    put('cpi', ('r22' if moved else 'r30')+', 0x02')
    if kind == 'u16':
        put('cpc', 'r31, r1')
    branch = put('brcs' if far_default else 'brcc')
    default = put('jmp', w=4) if far_default else branch
    suffix = p
    if kind == 'u8':
        if moved:
            put('mov', 'r30, r22')
        put('ldi', 'r31, 0x00')
    elif kind == 'sign8':
        put('mov', 'r0, r30')
        put('add', 'r0, r0')
        put('sbc', 'r31, r31')
    put('eor', 'r24, r24')
    base = p
    for op, reg in [('subi', 'r30'), ('sbci', 'r31'), ('sbci', 'r24')]:
        put(op, reg)
    dispatch = put('jmp', '0x400', 4)
    start = p
    neg = (-(start//2)) & 0xffffff
    for i in range(3):
        a = base+2*i
        w, op, reg = ins[a]
        ins[a] = w, op, reg+f', 0x{(neg>>(8*i))&255:02X}'
    p += 4  # two table words
    cases = [put('ret'), put('ret')]
    fallback = put('ret')
    if far_default:
        ins[branch] = (2, 'brcs', hex(suffix))
        ins[default] = (4, 'jmp', hex(fallback))
    else:
        ins[branch] = (2, 'brcc', hex(fallback))
    for i, (op, arg) in enumerate(HELPER):
        ins[0x400+2*i] = (2, op, arg)
    return ins, {dispatch: cases}, labels, {'branch': branch, 'suffix': suffix, 'base': base,
                                           'dispatch': dispatch, 'start': start, 'fallback': fallback}


def self_test():
    from copy import deepcopy
    target = lambda a: int(a, 0)
    for kind, far, moved in [('u8', False, False), ('u8', True, True),
                             ('sign8', True, False), ('u16', False, False)]:
        ins, tables, labels, _ = fixture(kind, far, moved)
        assert prove_tables(ins, tables, labels, target)['tables'][0]['kind'] == kind
    ins, tables, labels, p = fixture('u8')

    def reject(edit):
        a, t, l = deepcopy((ins, tables, labels))
        edit(a, t, l)
        try:
            prove_tables(a, t, l, target)
        except (AssertionError, KeyError):
            return
        raise AssertionError('invalid range/target graph accepted')

    reject(lambda a,t,l: a.__setitem__(0x100, (2, 'cpi', 'r30, 0x03')))
    reject(lambda a,t,l: a.__setitem__(p['branch'], (2, 'breq', hex(p['fallback']))))
    reject(lambda a,t,l: a.__setitem__(p['base'], (2, 'subi', 'r30, 0x00')))
    reject(lambda a,t,l: a.__setitem__(p['suffix'], (2, 'ldi', 'r31, 0x01')))
    reject(lambda a,t,l: a.__setitem__(p['fallback'], (2, 'rjmp', hex(p['suffix']))))
    reject(lambda a,t,l: t[p['dispatch']].__setitem__(0, p['suffix']))
    reject(lambda a,t,l: t[p['dispatch']].__setitem__(0, p['start']))
    reject(lambda a,t,l: a.__setitem__(p['fallback'], (4, 'call', '0x400')))
    reject(lambda a,t,l: a.__setitem__(0x400, (2, 'nop', '')))
    print('Task0034 switch table guards: four valid and nine rejected graphs PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', nargs='?')
    parser.add_argument('sha256', nargs='?')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
    else:
        assert args.archive and args.sha256
        with open(args.archive, 'rb') as f:
            assert hashlib.sha256(f.read()).hexdigest() == args.sha256
        spec = importlib.util.spec_from_file_location('fg', Path(__file__).with_name('foreground-stack-depth.py'))
        fg = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(fg)
        with zipfile.ZipFile(args.archive) as z:
            summary = json.loads(z.read('summary.json'))
            assert not summary['failures']
            reports = {}
            for board in fg.BOARDS:
                assert json.loads(z.read(board+'/verification.json'))['status'] == 'verified'
                ins, tables, labels = fg.decode_linked(z.read(board+'/disassembly.txt').decode(), board)
                reports[board] = prove_tables(ins, tables, labels, fg.target)
        print(json.dumps({'testedMerge': summary['sha'], 'sha256': args.sha256,
                          'boards': reports, 'releaseProof': False,
                          'assumptions': ['decoded table words and instruction boundaries',
                                          'ABI zero register r1 and EIND=0',
                                          'no unmodeled indirect entry; CRT not proved']}, indent=2))
